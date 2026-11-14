"""The REST hot-apply listener on the perception rail's Shield (WP1-15).

    python tests/test_pas_hotapply.py      (no pytest needed)

demo/pas_ardupilot/perception_node.py serves guardrail.api.FlightHotApply on
its own Shield since 2026-11-15 (demo/pas_ardupilot/rail_hotapply.py), as
sitl/ros2_shield_node.py and sitl/run_sitl_demo.py do on theirs. Checked
here, on the PAS rail's own stand-ins (tests/test_pas_ardupilot.py: a fake
ArduPilot that flies what it is sent, a fake detector, a virtual clock):

  * the five flags are the ROS 2 node's, names and defaults;
  * the node's thread holds the listener's lock from the hover on and lets
    go of it only while it sleeps (rail_hotapply.LoopLock), with a wall-clock
    yield after a sleep too short to let a waiting request in;
  * an event posted while the loop sleeps lands between two ticks: the rows
    before it carry generation 0 and the rows after generation 1, the
    generation is written as policy_g1.json / csp_g1.json with a
    hot_apply_rest event, metrics.json `hot_apply` is FlightHotApply's
    summary, and a zone spawned in flight counts in the dwell from its
    arrival - through a stand-in listener that runs the request on its own
    thread exactly as guardrail.api.build_app's call() does (no network);
  * the GeoFence follows the generation: the first tick after the event
    uploads the new zones (FenceBackstop.recheck), one near the aircraft held
    back;
  * after the episode the listener answers 503 and the Shield is untouched;
  * a listener that cannot start stops the run before arming;
  * over real HTTP on loopback (fastapi, uvicorn, httpx: the 3.11 extras;
    SKIP elsewhere) and with mutual TLS (cryptography too).

Shown failing first on master 478dcc7: the parser refuses --hotapply-port
(SystemExit 2) and rail_hotapply does not exist (ImportError).
"""
from __future__ import annotations

import json
import socket
import sys
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "demo" / "pas_ardupilot", ROOT / "sitl",
           ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import autopilot as ap                                               # noqa: E402
import perception_node as pn                                         # noqa: E402
import rail_hotapply as RH                                           # noqa: E402
import test_pas_ardupilot as T                                       # noqa: E402
from guardrail import api as A                                       # noqa: E402

SKIP = "SKIP"
HARD_P0 = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
           "violation_action": "project_fix"}


def _zone(zid, x0, y0, x1, y1, **kw):
    """A POST /dynamic_nfz body: a static zone over the whole fence column."""
    return {"id": zid, **HARD_P0, "altitude_floor_m": 0, "altitude_ceiling_m": 100,
            "margin_m": 1.0, **kw,
            "vertices": [{"x": x0, "y": y0}, {"x": x1, "y": y0},
                         {"x": x1, "y": y1}, {"x": x0, "y": y1}]}


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _events(run):
    return [json.loads(x) for x in (run / "events.jsonl").read_text(
        encoding="utf-8").splitlines()]


# --------------------------------------------------------------------------- #
# The flags and the lock
# --------------------------------------------------------------------------- #

def test_the_listener_flags_are_the_ros2_nodes_names_and_defaults():
    """Same five flags as sitl/ros2_shield_node.py (its make_parser imports
    without ROS), same defaults: one command line drives either rail."""
    import ros2_shield_node as N

    def flags(parser):
        return {a.option_strings[0]: (a.dest, a.default, a.type, type(a).__name__)
                for a in parser._actions
                if a.option_strings and a.option_strings[0].startswith("--hotapply")}
    ros, pas = flags(N.make_parser()), flags(pn.build_parser())
    assert set(ros) == {"--hotapply-port", "--hotapply-host", "--hotapply-pki",
                        "--hotapply-server-cert", "--hotapply-client"}, sorted(ros)
    assert pas == ros, (pas, ros)
    assert RH.DEFAULT_HOTAPPLY_HOST == N.DEFAULT_HOTAPPLY_HOST == "127.0.0.1"
    a = pn.build_parser().parse_args(["--hotapply-port", "8071", "--hotapply-client", "ops",
                                      "--hotapply-client", "harness-client"])
    assert a.hotapply_port == 8071 and a.hotapply_client == ["ops", "harness-client"]
    assert a.hotapply_host == "127.0.0.1" and a.hotapply_pki is None
    none = pn.build_parser().parse_args([])
    assert none.hotapply_port is None
    hot, gate = RH.start(none, None, None, mission_t=lambda: None)
    assert (hot, gate) == (None, None), "no port, no listener"


def test_the_node_lets_go_of_the_lock_only_while_it_sleeps():
    gate = RH.LoopLock(wall_sleep=lambda s: walls.append(s))
    walls, slept, got = [], [], []

    def try_lock():
        ok = gate.lock.acquire(timeout=0.2)
        got.append(ok)
        if ok:
            gate.lock.release()
    sleep = gate.sleeper(lambda dt: (slept.append(dt), try_other()))

    def try_other():
        th = threading.Thread(target=try_lock)
        th.start()
        th.join(5)
    # Not held (before the hover): sleep is plain sleep, no yield.
    sleep(0.05)
    assert slept == [0.05] and walls == [] and got == [True]
    gate.hold()
    gate.hold()                                     # idempotent
    try_other()
    assert got[-1] is False, "another thread took the lock while the node held it"
    sleep(0.08)                                     # let go inside the sleep
    assert got[-1] is True and slept[-1] == 0.08 and walls == []
    assert gate.held and gate.lock._is_owned(), "taken back after the sleep"
    sleep(0.0)                                      # an overrun: the wall yield
    assert slept[-1] == 0.0 and walls == [RH.LOCK_YIELD_S] and gate.n_yields == 1
    assert gate.n_sleeps == 2
    gate.let_go()
    gate.let_go()                                   # idempotent
    try_other()
    assert got[-1] is True and not gate.held


# --------------------------------------------------------------------------- #
# A stand-in listener: guardrail.api.build_app's call(), on its own thread
# --------------------------------------------------------------------------- #

class FakeListener:
    """What serve_in_background serves, without HTTP: each request runs on a
    thread of its own under the rail's lock, refuse_when first, then the
    route's function on the Shield, then on_applied - build_app's call()."""
    last = None

    def __init__(self, shield, port, lock, on_applied, refuse_when, pki_dir,
                 allowed_clients):
        self.shield, self.lock = shield, lock
        self.on_applied, self.refuse_when = on_applied, refuse_when
        self.url = f"{'https' if pki_dir else 'http'}://127.0.0.1:{port}"
        self.allowed_clients = (None if pki_dir is None
                                else sorted(set(allowed_clients or ["harness-client"])))
        self.server = SimpleNamespace(should_exit=False)
        self.answers = []
        FakeListener.last = self

    def _call(self, fn, *args):
        received = time.monotonic_ns()
        with self.lock:
            why = self.refuse_when()
            if why:
                return 503, why
            try:
                res = fn(self.shield, *args)
            except A.ApiError as e:
                return e.status, e.detail
            try:
                self.on_applied(res, received_ns=received, applied_ns=time.monotonic_ns())
            except Exception:                                     # noqa: BLE001
                pass
            return 200, res

    def post(self, fn, *args, wait_s: float = 5.0):
        """The request on its own thread; (status, body), or None if it was
        still waiting on the lock after `wait_s`."""
        box = []
        th = threading.Thread(target=lambda: box.append(self._call(fn, *args)))
        th.start()
        th.join(wait_s)
        ans = box[0] if box else None
        self.answers.append(ans)
        return ans


def _fake_serve(shield, host="127.0.0.1", port=8071, *, lock=None, on_applied=None,
                refuse_when=None, pki_dir=None, server_name=None, allowed_clients=None,
                wait_s=5.0):
    if pki_dir is None and host not in ("127.0.0.1", "::1", "localhost"):
        raise ValueError(f"refusing a plaintext hot-apply listener on {host!r}: "
                         f"plaintext is for loopback only")
    return FakeListener(shield, port, lock, on_applied, refuse_when, pki_dir,
                        allowed_clients)


class _Patched:
    """guardrail.api.serve_in_background swapped for `serve`, and
    rail_hotapply.start wrapped so the test sees the FlightHotApply."""

    def __init__(self, serve=_fake_serve):
        self.serve, self.hot = serve, None

    def __enter__(self):
        self._serve, self._start = A.serve_in_background, RH.start
        A.serve_in_background = self.serve

        def start(*a, **k):
            hot, gate = self._start(*a, **k)
            self.hot = hot
            return hot, gate
        RH.start = start
        return self

    def __exit__(self, *exc):
        A.serve_in_background, RH.start = self._serve, self._start
        return False


class PostClock(T.Clock):
    """The virtual clock, whose sleep runs `action(mission_t)` once the
    listener's mission time has passed `at` (inside the node's sleep: the
    lock is let go there)."""

    def __init__(self, patched, at, action):
        super().__init__()
        self.patched, self.at, self.action, self.done = patched, at, action, []

    def sleep(self, dt):
        super().sleep(dt)
        hot = self.patched.hot
        mt = None if hot is None else hot.mission_t()
        if not self.done and mt is not None and mt >= self.at:
            self.done.append(self.action(mt))


class FenceLink(T.FencedLink):
    """The fake ArduPilot with a GeoFence the node uploads zones to."""

    def __init__(self, *a, **k):
        super().__init__(*a, fence_vals={"FENCE_ENABLE": 1.0, "FENCE_ALT_MAX": 50.0,
                                         "FENCE_RADIUS": 400.0, "FENCE_ACTION": 1.0,
                                         "FENCE_TYPE": 7.0, "FENCE_MARGIN": 2.0}, **k)
        self.fence_items, self.fence_uploads = [], []

    def read_fence(self, timeout=None):
        return {"count": len(self.fence_items), "items": None, "why": None}

    def upload_fence(self, items, timeout=None):
        self.fence_items = list(items)
        self.fence_uploads.append({"t": self.clock.t, "n": len(items)})
        return {"success": True, "items": len(items), "transferred": len(items),
                "upload_ms": 1.0}


# --------------------------------------------------------------------------- #
# The flight
# --------------------------------------------------------------------------- #

def test_an_event_lands_between_two_ticks_and_is_a_generation():
    """A zone posted while the loop sleeps (from another thread) is applied
    at once; the rows before carry generation 0, the rows after 1; the
    generation's policy and CSP are written; metrics carry the summary; the
    zone, spawned over the aircraft, counts in the dwell from its arrival."""
    with _Patched() as P:
        def spawn(mt):
            # Over the aircraft (the fake starts at 35, -20 and flies at most
            # 4 m/s: it stays inside for the rest of the 3 s mission).
            return FakeListener.last.post(A.apply_dynamic_nfz,
                                          _zone("nfz-rest", -5, -60, 75, 20))
        clk = PostClock(P, 1.0, spawn)
        rc, run, link = T._fly("off", extra=["--hotapply-port", "18071"], clk=clk)
    assert rc == 0
    assert clk.done and clk.done[0] is not None, "the request waited on the lock in a sleep"
    status, body = clk.done[0]
    assert status == 200 and body["generation"] == 1, body
    rows = T._rows(run)
    gens = [r["generation"] for r in rows]
    k = gens.index(1)
    assert k > 0 and set(gens[:k]) == {0} and set(gens[k:]) == {1}, gens
    m = T._json(run, "metrics.json")
    ha = m["hot_apply"]
    assert ha["n_applied"] == 1 and ha["endpoint"]["url"] == "http://127.0.0.1:18071", ha
    ev_t = ha["events"][0]["t"]
    assert rows[k - 1]["t"] < ev_t <= rows[k]["t"], (rows[k - 1]["t"], ev_t, rows[k]["t"])
    assert ha["events"][0]["rule_id"] == "nfz-rest" and ha["spawn_target_ms"] == 50.0
    assert ha["receipt_to_swap_ms_max"] is not None and ha["n_record_failed"] == 0
    assert (run / "policy_g1.json").is_file() and (run / "csp_g1.json").is_file()
    assert [g["generation"] for g in m["generations"]] == [0, 1]
    ev = _events(run)
    assert [e["kind"] for e in ev].count("hot_apply_listener") == 1
    rest = [e for e in ev if e["kind"] == "hot_apply_rest"]
    assert len(rest) == 1 and rest[0]["rule_id"] == "nfz-rest", rest
    # Dwell from the arrival on: the aircraft was inside from the start.
    nfz = m["nfz_by_zone_s"].get("nfz-rest")
    flown = rows[-1]["t"] - rows[0]["t"]
    assert nfz is not None and 0.0 < nfz < flown - 0.5, (nfz, flown)
    assert abs(nfz - (rows[-1]["t"] - ev_t + 0.1)) < 0.25, (nfz, ev_t, rows[-1]["t"])
    # After the episode: refused before the Shield is touched.
    late = FakeListener.last.post(A.apply_dynamic_nfz, _zone("late", 0, 0, 5, 5))
    assert late is not None and late[0] == 503 and "ended" in late[1], late
    assert P.hot.shield.policy.generation == 1
    assert FakeListener.last.server.should_exit is True


def test_an_event_lands_while_the_tick_reads_the_link_without_waiting_for_it():
    """Since 2026-11-15 the node's thread holds the lock from the Shield's
    inputs to the tick's records, not through the whole tick: a request that
    arrives while the tick reads the link's state is applied at once (on hil
    a spawn waited 53 ms for a tick in progress, pas_hil_adapter_01), and
    still lands between two Shield decisions."""
    with _Patched() as P:
        done = []

        class StateLink(T.FakeLink):
            def state(self, anchor):
                hot = P.hot
                mt = None if hot is None else hot.mission_t()
                if not done and mt is not None and mt >= 1.0:
                    # Synchronous: the node's thread waits here for the answer.
                    done.append(FakeListener.last.post(
                        A.apply_dynamic_nfz, _zone("nfz-far", 200, 200, 210, 210),
                        wait_s=2.0))
                return super().state(anchor)
        rc, run, link = T._fly("off", link_cls=StateLink,
                               extra=["--hotapply-port", "18072"])
    assert rc == 0 and done, done
    assert done[0] is not None, "the request waited on the lock while the tick read the link"
    status, body = done[0]
    assert status == 200 and body["generation"] == 1, body
    rows = T._rows(run)
    gens = [r["generation"] for r in rows]
    k = gens.index(1)
    assert k > 0 and set(gens[:k]) == {0} and set(gens[k:]) == {1}, gens
    ev_t = T._json(run, "metrics.json")["hot_apply"]["events"][0]["t"]
    assert rows[k - 1]["t"] < ev_t <= rows[k]["t"], (rows[k - 1]["t"], ev_t, rows[k]["t"])


def test_the_geofence_follows_the_new_generation_with_a_near_zone_held_back():
    """The listener's thread never touches the link: the first tick after
    the event uploads generation 1's zones (FenceBackstop.recheck), the zone
    over the aircraft held back, a far one uploaded."""
    with _Patched() as P:
        def spawn(mt):
            L = FakeListener.last
            a = L.post(A.apply_dynamic_nfz, _zone("nfz-far", 200, 200, 210, 210))
            b = L.post(A.apply_dynamic_nfz, _zone("nfz-over", 25, -30, 45, -10))
            return a, b
        clk = PostClock(P, 1.0, spawn)
        rc, run, link = T._fly("off", link_cls=FenceLink,
                               extra=["--hotapply-port", "18072"], clk=clk)
    assert rc == 0 and [a[0] for a in clk.done[0]] == [200, 200], clk.done
    m = T._json(run, "metrics.json")
    ups = m["fence_uploads"]
    assert ups[0]["generation"] == 0 and ups[0]["zone_ids"] == [], ups[0]
    follow = [u for u in ups if "changed in flight" in u["reason"]]
    assert follow and follow[0]["generation"] == 2, [(u["reason"], u["generation"]) for u in ups]
    f = follow[0]
    assert f["zone_ids"] == ["nfz-far"] and "nfz-over" in f["deferred"], f
    assert f["in_flight"] is True and f["success"] is True
    assert len(follow) == 1, "one upload per generation"
    assert link.fence_uploads[-1]["n"] > 0, "the far zone reached the autopilot's fence"


def test_without_the_flag_the_run_has_no_listener_and_nothing_changes():
    with _Patched() as P:
        rc, run, link = T._fly("on")
    assert rc == 0 and P.hot is None
    m = T._json(run, "metrics.json")
    assert m["hot_apply"] is None
    assert not any(e["kind"].startswith("hot_apply") for e in _events(run))


class CountingLink(T.FakeLink):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.bring_ups = 0

    def bring_up(self, alt):
        self.bring_ups += 1
        super().bring_up(alt)


def test_a_listener_that_cannot_start_stops_the_run_before_arming():
    """Plaintext off loopback is refused (guardrail.api.serve_in_background,
    the real one: ValueError, or ImportError where uvicorn is not installed);
    either way nothing was armed and no hover was flown."""
    try:
        T._fly("on", link_cls=CountingLink,
               extra=["--hotapply-port", str(_free_port()), "--hotapply-host", "0.0.0.0"])
    except (ValueError, ImportError) as exc:
        link = T._LAST["link"]
        assert link.bring_ups == 0 and not link.snap.armed, "armed before the listener ran"
        assert not (T._LAST["run"] / "hover_log.jsonl").exists()
        if isinstance(exc, ValueError):
            assert "loopback only" in str(exc), exc
    else:
        raise AssertionError("a plaintext listener on 0.0.0.0 was accepted")


# --------------------------------------------------------------------------- #
# Over real HTTP on loopback, and mutual TLS
# --------------------------------------------------------------------------- #

def _have_http() -> bool:
    try:
        import fastapi  # noqa: F401
        import httpx  # noqa: F401
        import uvicorn  # noqa: F401
    except ImportError:
        return False
    return True


def test_over_real_http_an_event_reaches_the_flying_pas_shield():
    if not _have_http():
        return SKIP
    import httpx
    port = _free_port()
    url = f"http://127.0.0.1:{port}"
    with _Patched(serve=A.serve_in_background) as P:
        def spawn(mt):
            return httpx.post(f"{url}/dynamic_nfz", json=_zone("nfz-http", 25, -30, 45, -10),
                              timeout=10)
        clk = PostClock(P, 1.0, spawn)
        rc, run, link = T._fly("off", extra=["--hotapply-port", str(port)], clk=clk)
    assert rc == 0 and clk.done[0].status_code == 200, clk.done[0].text
    m = T._json(run, "metrics.json")
    assert m["hot_apply"]["n_applied"] == 1 and m["hot_apply"]["endpoint"]["url"] == url
    gens = [r["generation"] for r in T._rows(run)]
    assert gens[0] == 0 and gens[-1] == 1, gens
    try:
        r = httpx.post(f"{url}/dynamic_nfz", json=_zone("late", 0, 0, 5, 5), timeout=5)
        assert r.status_code == 503, r.text
    except httpx.HTTPError:
        pass                                    # the server already stopped
    assert P.hot.shield.policy.generation == 1


def test_with_the_pki_the_listener_is_mutual_tls_with_its_allow_list():
    if not _have_http():
        return SKIP
    try:
        from guardrail import harness_service as H
        H._crypto()
    except Exception:                                   # noqa: BLE001
        return SKIP
    import ssl

    import httpx
    pki = Path(tempfile.mkdtemp(prefix="pki_"))
    H.generate_pki(pki)
    H.issue_cert(pki, "ops")
    port = _free_port()

    def tls(name):
        ctx = ssl.create_default_context(cafile=str(H.pki_paths(pki, name)[0]))
        _, cert, key = H.pki_paths(pki, name)
        ctx.load_cert_chain(str(cert), str(key))
        return ctx
    with _Patched(serve=A.serve_in_background) as P:
        def probe(mt):
            ok = httpx.get(f"https://127.0.0.1:{port}/health", verify=tls("ops"), timeout=10)
            try:
                bad = httpx.get(f"https://127.0.0.1:{port}/health",
                                verify=tls("harness-client"), timeout=10)
            except httpx.HTTPError:
                bad = None
            return ok.status_code, None if bad is None else bad.status_code
        clk = PostClock(P, 0.5, probe)
        rc, run, link = T._fly("on", clk=clk, extra=[
            "--hotapply-port", str(port), "--hotapply-pki", str(pki),
            "--hotapply-client", "ops"])
    assert rc == 0 and clk.done[0] == (200, None), clk.done
    ep = T._json(run, "metrics.json")["hot_apply"]["endpoint"]
    assert ep["mutual_tls"] is True and ep["allowed_clients"] == ["ops"], ep
    assert ep["url"] == f"https://127.0.0.1:{port}"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs fastapi, uvicorn, httpx; the TLS "
                      f"test also cryptography: the 3.11 extras)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
