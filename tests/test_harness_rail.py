"""The stress harness on a flight rail (2026-11-10, WP4-03 and WP4-05): zone
edits forwarded as PATCH /dynamic_nfz, StartEpisode's rail backends, and the
episode bundle under the grant's names.

    python tests/test_harness_rail.py

NOTHING HERE FLIES. The rig is the fake tests/test_sweep.py flies its hil
campaign on (`_FakeRig`: no WSL, no ssh; its "Orin" flies the scenario on the
headless harness and writes the run folder the Shield node would), extended
with what a mission launched with `--hotapply-port` adds: a REST listener on
that port running guardrail/api.py's own handlers on a live Shield (served by
the standard library, as tests/test_wp4outputs_harness.py does), a pad phase
before the Shield's first tick, the generation files, a manifest, a bag
directory and an audit log. So these tests show the harness's own wiring -
what it launches, forwards, records, scores and bundles - and nothing about
ArduPilot, MAVROS, the Orin or the real listener's timing.

  * an edit of a zone (translate, rotate, scale, move) addressed to a
    REST-bound session goes out as PATCH <endpoint>/dynamic_nfz/<id> with the
    route's own body, and the endpoint's receipt comes back; a spawn, a switch
    and a swap still go to POST /hot_apply (the null: the same edit POSTed to
    /hot_apply is refused by the endpoint, which is what the harness did
    before);
  * a hil session is one flown, scored episode: StartEpisode launches the
    mission through sweep_scenarios.run_hil and the rig's transport with the
    campaign's own arguments plus the listener's flags, a HotApply to the
    episode is forwarded at its mission time (never early), EndEpisode scores
    the fetched run as the campaign does and returns the bundle;
  * the bundle carries the grant's six names, episode.bag2/ an alias of the
    run's bag/ (which the run folder keeps), every generation's IR in
    policy_bundle.tar.gz, and each forwarded hot-apply in harness_events.jsonl
    with its generation and the endpoint's receipt; a file the run lacks is
    "not written", never made up;
  * what a rail cannot fly launches nothing; a flight the rig cannot complete
    or that flew another policy is FAILED_PRECONDITION and frees the rig;
  * a session with no rail behaves as before (the headless bundle's result
    carries the same fields), and a campaign's missions are untouched: with
    no endpoint the launched arguments are rail_node_args' own;
  * the sitl backend goes through run_sitl's signature with the same flags;
  * the same over gRPC, through tools/harness_client.py `run --backend hil`.
"""
from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
import os
import shutil
import socket
import sys
import tarfile
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

os.environ.setdefault("GRPC_VERBOSITY", "ERROR")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
sys.path.insert(0, str(ROOT / "tools"))

from guardrail import api                                          # noqa: E402
from guardrail import harness_service as H                         # noqa: E402
from guardrail import load_policy                                  # noqa: E402
from guardrail.shield import Action4D, Shield, State               # noqa: E402

S = H._sweep()


def _load_test_sweep():
    """tests/test_sweep.py as a module, for the fake rig its hil campaign
    flies on (`_FakeRig`) and its bundle folder (`_signed_like_bundles`)."""
    spec = importlib.util.spec_from_file_location("test_sweep_for_rail",
                                                  ROOT / "tests" / "test_sweep.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


TS = _load_test_sweep()

SKIP = "SKIP"
_TMP: list[Path] = []
try:
    _M = H._grpc_modules()
except H.HarnessUnavailable:
    _M = None

SIM = ROOT / "policies" / "sim_demo_policy.yaml"
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}
# Far from every library mission's path: the fake flight does not react to it.
ZONE = {"type": "dynamic_nfz", **HARD_P0, "id": "nfz-hot",
        "vertices": [{"x": 210, "y": 230}, {"x": 220, "y": 230},
                     {"x": 220, "y": 250}, {"x": 210, "y": 250}]}
TRANSLATE = {"type": "dynamic_nfz", "op": "translate", "id": "nfz-hot", "dx": 5.0, "dy": -2.0}
ROTATE = {"type": "dynamic_nfz", "op": "rotate", "id": "nfz-hot", "angle_deg": 30.0}
SCALE = {"type": "dynamic_nfz", "op": "scale", "id": "nfz-hot", "factor": 1.5}
MOVE = {"type": "dynamic_nfz", "op": "move", "id": "nfz-hot",
        "vertices": [{"x": 300, "y": 300}, {"x": 310, "y": 300}, {"x": 310, "y": 310}]}
SWITCH_OFF = {"type": "time_window_switch", **HARD_P0, "target_id": "nfz-hot",
              "active": False}
CAMPAIGN = "tsthr"


def _tmp(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


def _refused(fn, *a, **kw) -> tuple[str, str]:
    try:
        fn(*a, **kw)
    except H.HarnessError as e:
        return e.code, e.message
    raise AssertionError(f"{getattr(fn, '__name__', fn)} was accepted")


def _wait_until(pred, timeout=30.0):
    t = time.monotonic()
    while time.monotonic() - t < timeout:
        if pred():
            return True
        time.sleep(0.01)
    return False


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _sha(b: bytes) -> str:
    return "sha256:" + hashlib.sha256(b).hexdigest()


# --------------------------------------------------------------------------- #
# guardrail/api.py's handlers on a live Shield, served by the standard library
# --------------------------------------------------------------------------- #

class _Endpoint:
    """GET /health, POST /hot_apply and PATCH /dynamic_nfz/<id> of
    guardrail/api.py on a live Shield: each call runs the handler build_app
    routes to, and a refusal answers its status with {"detail": ...} as
    FastAPI does. Records every request; `on_applied(result)` runs after an
    accepted event (the flying node's hook)."""

    def __init__(self, policy_path: Path, port: int = 0, on_applied=None):
        self.shield = Shield(load_policy(policy_path), escalation=False)
        self.requests: list[tuple] = []
        self.health_n = 0
        self.lock = threading.Lock()
        ep = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, status, doc):
                data = json.dumps(doc).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def _body(self):
                return json.loads(self.rfile.read(int(self.headers["Content-Length"])))

            def _apply(self, fn, *args):
                with ep.lock:
                    try:
                        res = fn(ep.shield, *args)
                    except api.ApiError as e:
                        return self._send(e.status, {"detail": e.detail})
                    if on_applied is not None:
                        on_applied(res)
                self._send(200, res)

            def do_GET(self):
                if self.path == "/health":
                    with ep.lock:
                        doc = api.health(ep.shield)
                        ep.health_n += 1
                    self._send(200, doc)
                else:
                    self._send(404, {"detail": "Not Found"})

            def do_POST(self):
                body = self._body()
                ep.requests.append(("POST", self.path, body))
                if self.path != "/hot_apply":
                    return self._send(404, {"detail": "Not Found"})
                self._apply(api.apply_event, body)

            def do_PATCH(self):
                from urllib.parse import unquote
                body = self._body()
                ep.requests.append(("PATCH", self.path, body))
                if not self.path.startswith("/dynamic_nfz/"):
                    return self._send(404, {"detail": "Not Found"})
                self._apply(api.apply_nfz_edit, unquote(self.path[len("/dynamic_nfz/"):]),
                            body)

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


# --------------------------------------------------------------------------- #
# the fake rig, with what a mission launched with --hotapply-port adds
# --------------------------------------------------------------------------- #

class _Rig(TS._FakeRig):
    """tests/test_sweep.py's fake hil rig, whose mission also serves the REST
    hot-apply listener when it is launched with `--hotapply-port=N`: up on
    the pad (the Shield has not ticked: mission_started false) until its
    /health has been read once and `pad_s` more, then the mission, until it
    has been sent `expect` events and `mission_s` more (or 30 s: a test whose
    harness sends fewer fails on its own assertions), then closed. The waits
    are on what happened, not on the clock, so a loaded host changes how long
    a test takes and not what it sees. It writes
    what the Shield node's run folder holds beside the fake's own files:
    manifest.json, shield_audit.jsonl, bag/ (two files), policy_g<N>.json of
    every generation, and metrics.json `hot_apply` with the mission's own
    time for each accepted event. `manifest_hash` makes the manifest name
    another policy; `bare` writes none of the extras."""

    def __init__(self, cfg, pad_s=0.2, mission_s=0.3, expect=0, manifest_hash=None,
                 bare=False, pre_started=False, **kw):
        super().__init__(cfg, **kw)
        self.pad_s, self.mission_s, self.expect = pad_s, mission_s, expect
        self.pre_started = pre_started
        self.manifest_hash, self.bare = manifest_hash, bare
        self.applied: list[dict] = []
        self.requests: list[tuple] = []
        self.t0 = None

    def run_mission(self, args, tag, shield, timeout_s):
        rc, out_text = super().run_mission(args, tag, shield, timeout_s)
        out = self.remote / "demo" / "out" / tag
        port = next((int(a.split("=", 1)[1]) for a in args
                     if a.startswith("--hotapply-port=")), None)
        g0 = (out / "policy_g0.json").read_bytes()
        if port is not None:
            def on_applied(res):
                now = time.monotonic()
                self.applied.append({
                    "t": None if self.t0 is None else round(now - self.t0, 3),
                    "op": res["op"], "type": res["type"], "rule_id": res["applied"],
                    "generation": res["generation"], "policy_hash": res["policy_hash"],
                    "receipt_to_swap_ms": 0.5})
                (out / f"policy_g{res['generation']}.json").write_bytes(
                    ep.shield.policy.canonical_bytes())
            ep = _Endpoint(SIM, port=port, on_applied=on_applied)
            try:
                if self.pre_started:        # a listener already past its mission start
                    with ep.lock:
                        ep.shield.filter(State(x=-50.0, y=-50.0, up=15.0), Action4D())
                _wait_until(lambda: ep.health_n >= 1, 30.0)     # on the pad
                time.sleep(self.pad_s)
                with ep.lock:
                    ep.shield.filter(State(x=-50.0, y=-50.0, up=15.0), Action4D())
                    self.t0 = time.monotonic()          # the mission's own time 0
                _wait_until(lambda: len(ep.requests) >= self.expect, 30.0)
                time.sleep(self.mission_s)
            finally:
                ep.close()
            self.requests = list(ep.requests)
        if self.bare:
            return rc, out_text
        metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
        metrics["hot_apply"] = {"n_applied": len(self.applied), "events": self.applied}
        (out / "metrics.json").write_text(json.dumps(metrics), encoding="utf-8")
        (out / "manifest.json").write_bytes(json.dumps({
            "code_revision": "fake-rig", "vla_model_hash": "fake",
            "policy_hash": self.manifest_hash or _sha(g0), "random_seed": 1001,
            "sim_speedup": 1.0, "topology": "hil"}, indent=2).encode("utf-8") + b"\n")
        (out / "shield_audit.jsonl").write_bytes(b'{"t": 0.1, "violations": []}\n')
        (out / "bag").mkdir()
        (out / "bag" / "bag_0.mcap").write_bytes(b"\x89MCAP0\r\n" + bytes(range(256)) * 8)
        (out / "bag" / "metadata.yaml").write_bytes(
            b"rosbag2_bagfile_information:\n  relative_file_paths:\n    - bag_0.mcap\n")
        return rc, out_text


def _rail_harness(backends=("hil",), **rig_kw):
    """(harness, rigs, td): a Harness on a fake rig; `rigs` fills as missions
    are launched. The campaign folders it writes under the checkout's
    demo/out are removed by _cleanup."""
    td = _tmp("hrail_")
    rigs: list = []
    harness_kw = {k: rig_kw.pop(k) for k in ("controls", "sitl_runner", "sitl_out",
                                             "hotapply_pki", "hotapply_clients",
                                             "hotapply_server_cert") if k in rig_kw}
    cfg = S.HilConfig(store=str(td / "store"), bundles_dir=str(TS._signed_like_bundles(td)))
    rig = H.RailRig(backends=tuple(backends), hil_config=cfg, campaign=CAMPAIGN,
                    transport=lambda c: rigs.append(_Rig(c, **rig_kw)) or rigs[-1],
                    **harness_kw)
    return H.Harness(runs_dir=td / "runs", rail=rig), rigs, td


def _cleanup():
    for base in (S.CAMPAIGN_DIR, S.RAIL_STRESS_DIR):
        for d in (ROOT / base).glob(CAMPAIGN + "*"):
            shutil.rmtree(d, ignore_errors=True)
    for d in _TMP:
        shutil.rmtree(d, ignore_errors=True)


def _policy_bundle(d: Path):
    with tarfile.open(d / H.POLICY_BUNDLE_FILE, "r:gz") as tar:
        names = sorted(tar.getnames())
        blobs = {n: tar.extractfile(n).read() for n in names}
    return names, blobs, json.loads(blobs[H.POLICY_BUNDLE_INDEX])


def _events(d: Path) -> list[dict]:
    return [json.loads(x) for x in
            (d / "harness_events.jsonl").read_text(encoding="utf-8").splitlines()]


@contextlib.contextmanager
def _fast_polls():
    """The driver's poll periods, shortened for a fake pad of half a second
    (the code reads them at each wait)."""
    saved = H.RAIL_ENDPOINT_POLL_S, H.RAIL_HEALTH_POLL_S
    H.RAIL_ENDPOINT_POLL_S, H.RAIL_HEALTH_POLL_S = 0.05, 0.05
    try:
        yield
    finally:
        H.RAIL_ENDPOINT_POLL_S, H.RAIL_HEALTH_POLL_S = saved


# --------------------------------------------------------------------------- #
# WP4-03: zone edits through the bound REST endpoint
# --------------------------------------------------------------------------- #

def test_a_hot_apply_body_names_its_own_route():
    """rest_route: an edit of a dynamic_nfz is PATCH /dynamic_nfz/<id> with
    the route's body (no `type`, no `id`); everything else is POST /hot_apply
    unchanged, a body with an op the PATCH route does not know included (the
    endpoint judges it, as before)."""
    assert H.rest_route(TRANSLATE) == ("PATCH", "/dynamic_nfz/nfz-hot",
                                       {"op": "translate", "dx": 5.0, "dy": -2.0})
    assert H.rest_route(ROTATE)[2] == {"op": "rotate", "angle_deg": 30.0}
    assert H.rest_route(SCALE)[2] == {"op": "scale", "factor": 1.5}
    assert H.rest_route(dict(MOVE, id="zone 7"))[1] == "/dynamic_nfz/zone%207"
    for body in (ZONE, SWITCH_OFF, {"type": "corridor_swap", "target_id": "c"},
                 dict(ZONE, op="add"), {"type": "time_window_switch", "op": "translate"}):
        assert H.rest_route(body) == ("POST", "/hot_apply", body), body
    for bad in ({}, None, [1], dict(TRANSLATE, id=""), dict(TRANSLATE, id="a/b"),
                dict(TRANSLATE, id=".."), {"type": "dynamic_nfz", "op": "scale"}):
        try:
            H.rest_route(bad)
        except H.HarnessError as e:
            assert e.code == "INVALID_ARGUMENT", (bad, e.code)
        else:
            raise AssertionError(f"{bad!r} was routed")


def test_a_zone_edit_is_forwarded_as_patch_dynamic_nfz_with_the_endpoints_receipt():
    """A session bound to a REST endpoint (the 3 Nov `drive` flow, the rail
    flown outside the harness): spawn over POST /hot_apply, then translate,
    rotate, scale and move over PATCH /dynamic_nfz/<id>. Each ack carries the
    route, the generation and the hash the endpoint's policy has after it,
    and the endpoint's Shield holds the zone a twin Shield gets from the same
    handler calls. The endpoint's refusals keep their codes. The null: the
    same edit POSTed to /hot_apply - what the harness did until 2026-11-10 -
    is refused by the endpoint's own handler."""
    h = H.Harness(runs_dir=_tmp("hrail_fw_"))
    ep = _Endpoint(SIM)
    twin = Shield(load_policy(SIM), escalation=False)
    try:
        s = h.start_session(rest_endpoint=ep.url)["session_id"]
        ack = h.hot_apply("", 0.0, ZONE, session_id=s)
        assert (ack["forwarded_method"], ack["forwarded_to"]) == ("POST", ep.url + "/hot_apply")
        api.apply_event(twin, ZONE)
        gen = 1
        for body in (TRANSLATE, ROTATE, SCALE, MOVE):
            ack = h.hot_apply("", 0.0, body, session_id=s)
            gen += 1
            want = api.apply_nfz_edit(twin, "nfz-hot", {k: v for k, v in body.items()
                                                         if k not in ("type", "id")})
            assert ack["forwarded_method"] == "PATCH", ack
            assert ack["forwarded_to"] == ep.url + "/dynamic_nfz/nfz-hot", ack
            assert (ack["rule_id"], ack["op"], ack["type"]) == ("nfz-hot", body["op"],
                                                                "dynamic_nfz"), ack
            assert ack["generation"] == gen == want["generation"], (ack, want)
            assert ack["policy_hash"] == ep.shield.policy.policy_hash == want["policy_hash"]
        assert ep.shield.policy.policy_hash == twin.policy.policy_hash
        sent = [r for r in ep.requests if r[0] == "PATCH"]
        assert [r[2] for r in sent] == [{k: v for k, v in b.items() if k not in ("type", "id")}
                                        for b in (TRANSLATE, ROTATE, SCALE, MOVE)], sent
        assert all(r[1] == "/dynamic_nfz/nfz-hot" for r in sent)
        # the endpoint's refusals, with their meaning
        assert _refused(h.hot_apply, "", 0.0, dict(TRANSLATE, id="no-such-zone"),
                        session_id=s)[0] == "NOT_FOUND"
        code, why = _refused(h.hot_apply, "", 0.0,
                             {"type": "dynamic_nfz", "op": "scale", "id": "nfz-hot"},
                             session_id=s)
        assert code == "INVALID_ARGUMENT" and "422" in why, (code, why)
        assert _refused(h.hot_apply, "", 0.0, dict(TRANSLATE, layer="regulation"),
                        session_id=s)[0] == "INVALID_ARGUMENT"
        assert _refused(h.hot_apply, "", 0.0, dict(TRANSLATE, id="a/b"),
                        session_id=s)[0] == "INVALID_ARGUMENT"
        assert ep.shield.policy.generation == gen, "a refused edit changed the policy"
        items, _, _ = h.wait_stream(s, "events", 0, 0.1)
        fw = [json.loads(i["detail_json"]) for i in items if i["kind"] == "hot_apply_forwarded"]
        assert "method" not in fw[0] and fw[1]["method"] == "PATCH", fw[:2]
        assert fw[1]["response"]["generation"] == 2 and fw[5]["refused"] == "NOT_FOUND", fw
        # The null: the edit body POSTed to /hot_apply is refused there.
        try:
            api.apply_event(twin, TRANSLATE)
        except api.ApiError as e:
            assert e.status == 422, e.status
        else:
            raise AssertionError("/hot_apply took a zone edit")
    finally:
        ep.close()


# --------------------------------------------------------------------------- #
# WP4-05: one harness session = one flown, scored episode with its bundle
# --------------------------------------------------------------------------- #

def _fly_hil(h, rigs, port, *, timeline=(), session_hot=(), late=(), keep_log=False,
             inject=True, prompt="", during=None):
    """The page's lifecycle on the fake rig. `timeline`: (at, body) to the
    episode; `session_hot`: bodies addressed to the session once every
    timeline entry has been forwarded; `late`: (at, body) queued, expected
    never to be sent."""
    sim = load_policy(SIM)
    url = f"http://127.0.0.1:{port}" if port else ""
    s = h.start_session(backend="hil", rest_endpoint=url, seed=1001)["session_id"]
    if inject:
        ack = h.inject_policy(s, bundle_path="policies/sim_demo_policy.yaml",
                              expected_hash=sim.policy_hash)
        assert ack["endpoint_policy_hash"] == "", ack     # nothing flies yet
    if prompt:
        h.issue_task(session_id=s, prompt=prompt)
    ref = h.start_episode(scenario_id="speed-cap", session_id=s, keep_flight_log=keep_log)
    eid = ref["episode_id"]
    acks = [h.hot_apply(eid, at, body) for at, body in list(timeline) + list(late)]
    if during is not None:
        during(s, eid)
    if session_hot:
        assert _wait_until(lambda: len(h._get(eid).rail.forwarded) >= len(timeline)), \
            (h.status(eid), h._get(eid).rail.forwarded)
        for body in session_hot:
            try:
                h.hot_apply("", 0.0, body, session_id=s)
            except H.HarnessError:
                pass
    return s, ref, acks


def test_a_hil_session_is_one_flown_scored_episode_with_the_grants_bundle():
    h, rigs, td = _rail_harness(expect=6)
    port = _free_port()
    timeline = [(0.2, ZONE), (0.5, TRANSLATE), (0.8, ROTATE), (1.1, SCALE),
                (1.3, dict(TRANSLATE, id="no-such-zone"))]
    seen: dict = {}

    def during(s, eid):
        # While it flies: the rig is taken, and a rail episode takes no EmitEvent
        # without a REST form (since 2026-11-14 one with a REST form is
        # forwarded: tests/test_harness_hil.py), nor one that does not name
        # its zone.
        seen["busy"] = _refused(h.start_episode, scenario_id="speed-cap", session_id=s)
        seen["emit"] = _refused(h.emit_event, eid, 1.0, "spawn_polygon_fence",
                                {"width_m": 10, "height_m": 10, "ahead_of_vehicle_m": 20})
        seen["emit_noid"] = _refused(h.emit_event, eid, 1.0, "scale_radius", {"factor": 2})
        seen["status"] = h.status(eid)
        seen["far"] = _refused(h.hot_apply, eid, 9999.0, ZONE)
        seen["live"] = _refused(h.start_episode, scenario_id="speed-cap", session_id=s,
                                live=True)

    with _fast_polls():
        s, ref, acks = _fly_hil(h, rigs, port, timeline=timeline, late=[(40.0, MOVE)],
                                session_hot=[SWITCH_OFF], keep_log=True,
                                prompt="hold 5 m/s along the road", during=during)
        b = h.end_episode(ref["episode_id"])
    assert h.end_episode(ref["episode_id"]) is b, "EndEpisode is idempotent"
    stop = h.stop_session(s)
    rig = rigs[0]

    # ---- launched through the sweep's own transport, with the campaign's arguments
    assert seen["busy"][0] == "RESOURCE_EXHAUSTED" and seen["emit"][0] == "UNIMPLEMENTED", seen
    assert seen["emit_noid"][0] == "INVALID_ARGUMENT" and "`id`" in seen["emit_noid"][1], seen
    assert seen["far"][0] == "OUT_OF_RANGE" and seen["live"][0] == "INVALID_ARGUMENT", seen
    assert seen["status"]["state"] == "RUNNING" and seen["status"]["backend"] == "hil"
    assert ref["backend"] == "hil" and not h._rail_busy.locked()
    assert all(a["pending"] and a["queue_index"] == i for i, a in enumerate(acks)), acks
    assert acks[1]["forwarded_method"] == "PATCH" and acks[0]["forwarded_method"] == "POST"
    kinds = [c[0] for c in rig.calls]
    assert kinds == ["restart_sitl", "push", "restart_mavros", "run_mission", "stop_sitl"], kinds
    run = next(c for c in rig.calls if c[0] == "run_mission")
    tag = f"{CAMPAIGN}_001_speed-cap"
    assert run[1] == tag and run[2] == "on", run[:3]
    spec = TS.BY_ID["speed-cap"]
    own = S.rail_node_args(spec, 1001, TS.DEFAULTS, backend="hil",
                           meta=f"{S.CAMPAIGN_DIR}/{tag}/scenario.json",
                           bundles_dir=str(td / "bundles"))
    assert run[3] == own + [f"--hotapply-port={port}"], (run[3], own)

    # ---- scored as the campaign scores it
    run_dir = td / "store" / tag
    assert Path(b["rail_run_path"]) == run_dir and b["backend"] == "hil"
    score = json.loads((run_dir / S.SCENARIO_SCORE).read_text(encoding="utf-8"))
    d = Path(b["bundle_path"])
    kpi = json.loads((d / "kpi.json").read_text(encoding="utf-8"))
    assert b["status"] == score["status"] == kpi["status"] == "pass", (b["status"],
                                                                       score["status"])
    assert kpi["kpi"] == score["kpi"] and kpi["gates"] == score["gates"]
    assert score["gates_basis"] == "ideal" and kpi["harness"]["backend"] == "hil"
    assert b["kpi"]["episodes"] == 1 and b["mission_outcome"]
    # not KPI-grade: the fake manifest says hil, and no Orin evidence stands beside it
    assert b["kpi_grade"] is False and b["kpi_grade_reasons"], b["kpi_grade_reasons"]

    # ---- the grant's six names
    files = b["bundle_files"]
    assert files == kpi["harness"]["bundle_files"]
    assert set(files) == {"manifest.json", "policy_bundle.tar.gz", "episode.bag2/",
                          "shield_audit.jsonl", "harness_events.jsonl", "kpi.json"}, files
    assert all(v.startswith("written") for v in files.values()), files
    assert (d / "manifest.json").read_bytes() == (run_dir / "manifest.json").read_bytes()
    assert (d / "shield_audit.jsonl").read_bytes() == \
        (run_dir / "shield_audit.jsonl").read_bytes()
    assert (d / "flight_log.jsonl").read_bytes() == (run_dir / "flight_log.jsonl").read_bytes()
    # episode.bag2/: the run's bag/, which the run folder keeps
    bag, alias = run_dir / "bag", d / "episode.bag2"
    assert bag.is_dir() and sorted(p.name for p in alias.iterdir()) == \
        ["bag_0.mcap", "metadata.yaml"]
    for p in bag.iterdir():
        assert (alias / p.name).read_bytes() == p.read_bytes(), p.name
    info = kpi["harness"]["episode_bag"]
    assert {f["path"]: f["sha256"] for f in info["files"]} == {
        p.name: _sha(p.read_bytes()) for p in bag.iterdir()}, info
    assert info["how"] in ("hard links", "copies", "hard links and copies")

    # ---- every hot-apply generation's IR, not its hash only
    names, blobs, idx = _policy_bundle(d)
    gens = [g["generation"] for g in idx["generations"]]
    assert gens == [0, 1, 2, 3, 4, 5], idx["generations"]
    assert idx["complete"] is True and files["policy_bundle.tar.gz"] == "written, complete"
    for g in idx["generations"]:
        blob = blobs[g["ir"]]
        assert g["ir"] == f"generations/policy_g{g['generation']}.json"
        assert _sha(blob) == g["policy_hash"] == idx["members"][g["ir"]], g
        assert blob == (run_dir / f"policy_g{g['generation']}.json").read_bytes()
    assert idx["injected"]["member"] == "injected/" + S.rail_policy_args(
        spec, "hil", str(td / "bundles"))[1].split("/")[-1], idx["injected"]

    # ---- harness_events.jsonl: each forward with its generation and the receipt
    ev = _events(d)
    assert ev[0]["type"] == "episode_start" and ev[-1]["type"] == "episode_end"
    assert ev[0]["rail_run"] == tag and ev[0]["endpoint_policy_hash"] == \
        load_policy(SIM).policy_hash
    fw = [e for e in ev if e["type"] == "hot_apply"]
    ok = [e for e in fw if not e["refused"]]
    assert [(e["method"], e["op"], e["generation"]) for e in ok if e["source"] == "episode"] \
        == [("POST", "spawn", 1), ("PATCH", "translate", 2), ("PATCH", "rotate", 3),
            ("PATCH", "scale", 4)], ok
    by_gen = {a["generation"]: a for a in rig.applied}
    for e in ok:
        r = e["receipt"]
        assert (r["generation"], r["policy_hash"]) == (e["generation"], e["policy_hash"])
        assert e["policy_hash"] == by_gen[e["generation"]]["policy_hash"]
        assert e["rail"]["t"] == by_gen[e["generation"]]["t"], e
        assert e["url"].startswith(f"http://127.0.0.1:{port}/")
    for e in (x for x in ok if x["source"] == "episode"):
        # at its mission time: never early, on this server's clock or the
        # mission's own (how late is the host's load here, not a property)
        assert e["at_sim_t"] <= e["t"] < e["at_sim_t"] + 10.0, e
        assert e["at_sim_t"] - 1e-3 <= e["rail"]["t"] < e["at_sim_t"] + 10.0, e
    sess = [e for e in ok if e["source"] == "session"]
    assert len(sess) == 1 and sess[0]["event"] == "time_window_switch" and \
        sess[0]["generation"] == 5 and sess[0]["t"] is not None, sess
    bad = [e for e in fw if e["refused"] and e.get("sent", True)]
    assert len(bad) == 1 and bad[0]["code"] == "NOT_FOUND" and "generation" not in bad[0], bad
    unsent = [e for e in fw if e.get("sent") is False]
    assert len(unsent) == 1 and unsent[0]["at_sim_t"] == 40.0 and \
        "ended" in unsent[0]["reason"], unsent
    assert (ev[-1]["forwarded_applied"], ev[-1]["forwarded_refused"], ev[-1]["not_sent"]) \
        == (5, 1, 1), ev[-1]
    assert [a["via"] for a in b["applied_events"]] == ["rest"] * len(fw)
    assert json.loads(b["applied_events"][0]["receipt_json"])["generation"] == 1
    # what the mission was sent, by route
    assert [(m, p) for m, p, _ in rig.requests] == [
        ("POST", "/hot_apply"), ("PATCH", "/dynamic_nfz/nfz-hot"),
        ("PATCH", "/dynamic_nfz/nfz-hot"), ("PATCH", "/dynamic_nfz/nfz-hot"),
        ("PATCH", "/dynamic_nfz/no-such-zone"), ("POST", "/hot_apply")], rig.requests

    # ---- the session's streams, the task, the request
    items, _, done = h.wait_stream(s, "events", 0, 0.1)
    k = [i["kind"] for i in items]
    assert done and k[-1] == "session_stopped" and "episode_ended" in k, k
    phases = [json.loads(i["detail_json"])["phase"] for i in items if i["kind"] == "rail_phase"]
    assert phases == ["awaiting_endpoint", "awaiting_mission_start", "mission", "fetched"], phases
    assert k.count("hot_apply_forwarded") == len(fw) and k.count("hot_apply") == 6
    rows = (run_dir / "flight_log.jsonl").read_text(encoding="utf-8").splitlines()
    assert stop["telemetry_frames"] >= 1 and stop["episodes"] == 1
    assert b["task_prompt_sha256"].startswith("sha256:")
    extras = json.loads((d / "manifest_extras.json").read_text(encoding="utf-8"))
    assert extras["rail_run"] == tag and extras["task_read_by_pilot"] is False
    req = json.loads((d / "harness_request.json").read_text(encoding="utf-8"))
    assert len(req["events"]) == 6 and req["events"][1]["body"] == TRANSLATE
    assert len(rows) >= stop["telemetry_frames"] > 0


def test_a_session_with_no_endpoint_launches_the_campaigns_own_mission():
    """No rest_endpoint: the mission's arguments are rail_node_args' own, not
    one flag more (a `--backend hil` campaign's mission), and a HotApply to
    the episode is refused, not queued. The run folder here is the bare
    fake's - no manifest, no bag, no audit log - and the bundle says "not
    written" for each instead of making one up; generation 0's IR is still
    in policy_bundle.tar.gz."""
    h, rigs, td = _rail_harness(bare=True)
    seen: dict = {}

    def during(s, eid):
        seen["hot"] = _refused(h.hot_apply, eid, 1.0, ZONE)

    s, ref, _ = _fly_hil(h, rigs, 0, inject=False, during=during)
    b = h.end_episode(ref["episode_id"])
    assert seen["hot"][0] == "FAILED_PRECONDITION" and "not bound" in seen["hot"][1], seen
    tag = f"{CAMPAIGN}_001_speed-cap"
    run = next(c for c in rigs[0].calls if c[0] == "run_mission")
    assert run[3] == S.rail_node_args(TS.BY_ID["speed-cap"], 1001, TS.DEFAULTS, backend="hil",
                                      meta=f"{S.CAMPAIGN_DIR}/{tag}/scenario.json",
                                      bundles_dir=str(td / "bundles")), run[3]
    files = b["bundle_files"]
    for name in ("manifest.json", "episode.bag2/", "shield_audit.jsonl"):
        assert files[name].startswith("not written"), (name, files[name])
    d = Path(b["bundle_path"])
    assert not (d / "manifest.json").exists() and not (d / "episode.bag2").exists()
    assert not (d / "shield_audit.jsonl").exists()
    assert b["status"] == "pass" and b["kpi_grade"] is False
    assert "no manifest.json" in b["kpi_grade_reasons"][0]
    _, _, idx = _policy_bundle(d)
    assert [g["generation"] for g in idx["generations"]] == [0] and idx["complete"]
    ev = _events(d)
    assert [e["type"] for e in ev] == ["episode_start", "episode_end"]
    assert ev[0]["rest_endpoint"] is None and ev[0]["mission_t0_basis"] is None
    h.stop_session(s)


def test_what_a_rail_cannot_fly_launches_nothing():
    plain = H.Harness(runs_dir=_tmp("hrail_none_"))
    for backend in ("sitl", "hil"):
        code, why = _refused(plain.start_session, backend=backend)
        assert code == "UNIMPLEMENTED" and "--rail" in why, (code, why)
    h, rigs, td = _rail_harness()
    assert _refused(h.start_session, backend="sitl")[0] == "UNIMPLEMENTED"   # a hil rig only
    code, why = _refused(h.start_session, backend="hil", rest_endpoint="http://127.0.0.1")
    assert code == "INVALID_ARGUMENT" and "port" in why, (code, why)
    s = h.start_session(backend="hil", seed=1001)["session_id"]
    code, why = _refused(h.inject_policy, s, bundle_bytes=b"x", expected_hash="sha256:0")
    assert code == "INVALID_ARGUMENT" and "both hosts" in why, (code, why)
    for sid, cell in (("nonfinite-action", ""),
                      ("altitude-recovery-depth", "altitude-recovery-depth@start_up=0.5")):
        code, why = _refused(h.start_episode, scenario_id=sid, cell_id=cell, session_id=s)
        assert code == "FAILED_PRECONDITION" and "hil rail cannot fly" in why, (sid, code, why)
    code, why = _refused(h.start_episode, scenario_id="nfz-head-on-control", session_id=s)
    assert code == "FAILED_PRECONDITION" and "control arm" in why, (code, why)
    assert _refused(h.start_episode, scenario_id="speed-cap", session_id=s, pace=2.0)[0] == \
        "INVALID_ARGUMENT"
    assert rigs == [] and not h._rail_busy.locked(), "something was launched"
    # the listener's flags, loopback and off it
    sess = h._session(s)
    assert h._rail_hot_args(sess) == []
    sess.rest_endpoint = "http://127.0.0.1:8071"
    assert h._rail_hot_args(sess) == ["--hotapply-port=8071"]
    sess.rest_endpoint = "https://10.42.0.3:8071"
    assert _refused(h._rail_hot_args, sess)[0] == "FAILED_PRECONDITION"
    h.rail.hotapply_pki, h.rail.hotapply_clients = "/opt/pki", ("harness-forwarder",)
    h.rail.hotapply_server_cert = "orin-hotapply"
    args = h._rail_hot_args(sess)
    assert args == ["--hotapply-port=8071", "--hotapply-host=10.42.0.3",
                    "--hotapply-pki=/opt/pki", "--hotapply-server-cert=orin-hotapply",
                    "--hotapply-client=harness-forwarder"], args
    assert S.rail_extra_args(args) == args          # each declared by the Shield node
    # only the listener's flags may be added to a scenario's own
    for bad in (["--start=0,0"], ["--hotapply-port", "8071"], ["--no-such-flag=1"]):
        try:
            S.rail_extra_args(bad)
        except S.UnsupportedScenario:
            pass
        else:
            raise AssertionError(f"{bad} was accepted")
    assert S.rail_extra_args(None) == [] == S.rail_extra_args([])


def test_a_flight_the_rig_cannot_complete_fails_the_episode_and_frees_the_rig():
    """An unfinished mission (the fake writes no mission_end) is refused by
    accept_hil_run, a manifest naming another policy by the harness: both are
    FAILED_PRECONDITION at EndEpisode with the reason, nothing is bundled,
    and the rig takes the next episode."""
    h, rigs, _ = _rail_harness(finish=False, bare=True)
    s, ref, _ = _fly_hil(h, rigs, 0)
    code, why = _refused(h.end_episode, ref["episode_id"])
    assert code == "FAILED_PRECONDITION" and "no mission_end" in why, (code, why)
    assert h.status(ref["episode_id"])["state"] == "FAILED" and not h._rail_busy.locked()
    assert not list((h.runs_dir / ref["episode_id"]).glob("*")), "a failed flight was bundled"
    h.stop_session(s)

    h2, rigs2, _ = _rail_harness(manifest_hash="sha256:" + "0" * 64)
    s2, ref2, _ = _fly_hil(h2, rigs2, 0)
    code, why = _refused(h2.end_episode, ref2["episode_id"])
    assert code == "FAILED_PRECONDITION" and "flew policy" in why, (code, why)
    # the rig is free: the same harness flies the next one
    ref3 = h2.start_episode(scenario_id="speed-cap", session_id=s2)
    code, _ = _refused(h2.end_episode, ref3["episode_id"])
    assert code == "FAILED_PRECONDITION" and len(rigs2) == 2
    assert next(c for c in rigs2[1].calls if c[0] == "run_mission")[1] == \
        f"{CAMPAIGN}_002_speed-cap"
    h2.stop_session(s2)


def test_an_endpoint_flying_another_policy_gets_nothing_forwarded():
    """The driver checks the mission's listener against the episode's policy
    when it first answers: a mismatch forwards nothing, says so on the queued
    hot-apply, and refuses later ones."""
    h, rigs, td = _rail_harness()
    port = _free_port()
    demo = load_policy(ROOT / "policies" / "demo_policy.yaml")
    with _fast_polls():
        s = h.start_session(backend="hil", rest_endpoint=f"http://127.0.0.1:{port}",
                            seed=1001)["session_id"]
        ref = h.start_episode(scenario_id="speed-cap", session_id=s)
        eid = ref["episode_id"]
        # as if the session had armed another policy than the mission flies
        h._get(eid).policy = demo
        h.hot_apply(eid, 0.2, ZONE)
        assert _wait_until(lambda: h._get(eid).rail.endpoint_problem), "no mismatch seen"
        code, why = _refused(h.hot_apply, eid, 0.5, TRANSLATE)
        assert code == "FAILED_PRECONDITION" and "flies policy" in why, (code, why)
        _refused(h.end_episode, eid)          # the manifest check refuses the bundle too
    assert rigs[0].requests == [], rigs[0].requests
    items, _, _ = h.wait_stream(s, "events", 0, 0.1)
    unsent = [json.loads(i["detail_json"]) for i in items
              if i["kind"] == "hot_apply_forwarded"]
    assert len(unsent) == 1 and "flies policy" in unsent[0]["not_sent"], unsent
    h.stop_session(s)


def test_a_listener_already_past_its_mission_start_gets_nothing_forwarded():
    """Mission time 0 is the first mission_started this server SEES after a
    not-yet: a listener whose first answer already says mission_started (one
    left by another mission, or reached too late) gives no time 0, so nothing
    is forwarded at a guessed time - the queued hot-apply is recorded as
    never sent, with the reason, in the bundle."""
    h, rigs, td = _rail_harness(pre_started=True)
    port = _free_port()
    with _fast_polls():
        s = h.start_session(backend="hil", rest_endpoint=f"http://127.0.0.1:{port}",
                            seed=1001)["session_id"]
        ref = h.start_episode(scenario_id="speed-cap", session_id=s)
        h.hot_apply(ref["episode_id"], 0.1, ZONE)
        b = h.end_episode(ref["episode_id"])
    assert rigs[0].requests == [], rigs[0].requests
    fw = [e for e in _events(Path(b["bundle_path"])) if e["type"] == "hot_apply"]
    assert len(fw) == 1 and fw[0]["sent"] is False and fw[0]["refused"] and \
        "mission time 0 was not observed" in fw[0]["reason"], fw
    kpi = json.loads((Path(b["bundle_path"]) / "kpi.json").read_text(encoding="utf-8"))
    assert "not observed" in kpi["harness"]["rail"]["endpoint_problem"]
    _, _, idx = _policy_bundle(Path(b["bundle_path"]))
    assert [g["generation"] for g in idx["generations"]] == [0] and idx["complete"]
    h.stop_session(s)


def test_the_sitl_backend_flies_through_run_sitl_with_the_listeners_flags():
    """backend sitl: the rig's runner is called as sweep_scenarios.run_sitl is
    (spec, tag, seed, defaults, episode=, extra_args=), its run folder read
    from sitl_out/<tag>, the run scored and bundled; no scenario_score.json
    is written beside a sitl run (the campaign writes none either). The
    desktop rail's command takes the same flags at its end, and without them
    is the command it was."""
    td = _tmp("hrail_sitl_")
    calls: list = []
    fake = _Rig(S.HilConfig(), expect=2)

    def runner(spec, tag, seed, defaults, episode=None, *, extra_args=None):
        calls.append((spec.scenario_id, tag, seed, episode.episode_id, extra_args))
        meta = S.rail_meta(spec, episode, seed, "sitl", tag, "n0nce", None, None, defaults)
        (fake.remote / "meta").mkdir(parents=True, exist_ok=True)
        (fake.remote / "meta" / "scenario.json").write_text(json.dumps(S._json_safe(meta)),
                                                            encoding="utf-8")
        fake.run_mission(["--scenario-meta=meta/scenario.json"] + list(extra_args or []),
                         tag, "on", 60.0)
        out = fake.fetch(tag, td / "out")
        rows = [json.loads(x) for x in (out / "flight_log.jsonl").read_text(
            encoding="utf-8").splitlines()]
        metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
        pol = spec.policy.load(ROOT)
        extra = S.sitl_extra(spec, rows, metrics, pol, S.run_generations(out),
                             max_s=meta["max_s"], dt=defaults.dt)
        return S.rail_scored_rows(rows, meta["max_s"]), extra

    h = H.Harness(runs_dir=td / "runs", rail=H.RailRig(
        backends=("sitl",), sitl_runner=runner, sitl_out=td / "out", campaign=CAMPAIGN))
    port = _free_port()
    with _fast_polls():
        s = h.start_session(backend="sitl", rest_endpoint=f"http://127.0.0.1:{port}",
                            seed=1001)["session_id"]
        ref = h.start_episode(scenario_id="speed-cap", session_id=s)
        h.hot_apply(ref["episode_id"], 0.2, ZONE)
        h.hot_apply(ref["episode_id"], 0.5, SCALE)
        b = h.end_episode(ref["episode_id"])
    tag = f"{CAMPAIGN}_001_speed-cap"
    assert calls == [("speed-cap", tag, 1001, "speed-cap", [f"--hotapply-port={port}"])], calls
    assert b["backend"] == "sitl" and b["status"] == "pass", b["status"]
    assert Path(b["rail_run_path"]) == td / "out" / tag
    assert not (td / "out" / tag / S.SCENARIO_SCORE).exists()
    d = Path(b["bundle_path"])
    _, _, idx = _policy_bundle(d)
    assert [g["generation"] for g in idx["generations"]] == [0, 1, 2] and idx["complete"]
    fw = [e for e in _events(d) if e["type"] == "hot_apply"]
    assert [(e["method"], e["generation"]) for e in fw] == [("POST", 1), ("PATCH", 2)], fw
    assert (d / "episode.bag2" / "bag_0.mcap").is_file()
    h.stop_session(s)
    # the desktop rail's command line
    spec = TS.BY_ID["speed-cap"]
    plain = S.build_sitl_cmd(spec, "t", 1001, TS.DEFAULTS)
    assert S.build_sitl_cmd(spec, "t", 1001, TS.DEFAULTS, extra_args=None) == plain
    assert S.build_sitl_cmd(spec, "t", 1001, TS.DEFAULTS,
                            extra_args=["--hotapply-port=8071"]) == \
        plain + ["--hotapply-port=8071"]


# --------------------------------------------------------------------------- #
# the headless path: every generation kept, and nothing else changed
# --------------------------------------------------------------------------- #

OPEN = """
scenario_id: rail-open-flight
template: dynamic_nfz_movement
description: "Across open ground at 15 m; nothing in the way unless an event puts it there."
mission:
  start_pose: {x: -20.0, y: -20.0, up: 15.0}
  target: {x: 40.0, y: -20.0, up: 15.0}
  pilot: {type: goto, speed: 4.0}
policy: {path: policies/sim_demo_policy.yaml}
expected_kpis:
  P0_escape_rate: 0
"""


def test_the_headless_runner_keeps_every_generation_and_the_bundle_packs_them():
    """Three hot-applies and an EmitEvent on the headless backend: the run
    holds the canonical IR of generations 0-4 (EpisodeRun.generations), each
    hashing to the hash its harness event recorded, and policy_bundle.tar.gz
    holds all five - until 2026-11-10 generations 1-3 were there by hash
    only. A held IR that does not hash to the recorded hash is not packed
    (the null). A session with no rail answers as before, with the new
    fields saying so."""
    near = dict(ZONE, vertices=[{"x": 10, "y": -30}, {"x": 20, "y": -30},
                                {"x": 20, "y": -10}, {"x": 10, "y": -10}])
    h = H.Harness(runs_dir=_tmp("hrail_hl_"))
    ref = h.start_episode(scenario_yaml=OPEN, seed=7)
    h.hot_apply(ref["episode_id"], 3.0, near)
    h.hot_apply(ref["episode_id"], 5.0, SWITCH_OFF)
    h.hot_apply(ref["episode_id"], 7.0, dict(SWITCH_OFF, active=True))
    h.emit_event(ref["episode_id"], 9.0, "scale_radius", {"id": "nfz-hot", "factor": 1.2})
    b = h.end_episode(ref["episode_id"])
    assert b["backend"] == "headless" and "rail_run_path" not in b
    assert b["bundle_files"]["policy_bundle.tar.gz"] == "written, complete", b["bundle_files"]
    assert b["bundle_files"]["episode.bag2/"].startswith("not written")
    assert [a["via"] for a in b["applied_events"]] == ["runner"] * 4
    d = Path(b["bundle_path"])
    _, blobs, idx = _policy_bundle(d)
    want = S.policy_generations(_events(d))
    assert [g["generation"] for g in want] == [0, 1, 2, 3, 4], want
    assert [(g["generation"], g["policy_hash"], g["ir"]) for g in idx["generations"]] == [
        (g["generation"], g["policy_hash"], f"generations/policy_g{g['generation']}.json")
        for g in want], idx["generations"]
    assert all(_sha(blobs[g["ir"]]) == g["policy_hash"] for g in idx["generations"])
    assert len({g["policy_hash"] for g in want}) == 5
    # the runner's own record, and the null
    spec = h._family_from_yaml(OPEN).first.spec
    run = S.run_episode(spec.model_copy(update={"events": [
        H.ScenarioEvent.model_validate(json.loads(h.hot_apply(
            h.start_episode(scenario_yaml=OPEN, seed=7)["episode_id"], 3.0,
            near)["scenario_event_json"]))]}), TS.DEFAULTS, 7)
    assert sorted(run.generations) == [0, 1]
    assert _sha(run.generations[1]) == run.policy.policy_hash
    gens = S.policy_generations(run.events)
    swapped = {0: run.generations[0], 1: run.generations[0]}
    bad = H.write_policy_bundle(_tmp("hrail_pb_"), SIM, None, None, gens, held=swapped)
    assert [g["ir"] is None for g in bad["generations"]] == [False, True] and \
        bad["complete"] is False and "[1]" in bad["missing"], bad
    good = H.write_policy_bundle(_tmp("hrail_pb_"), SIM, None, None, gens,
                                 held=run.generations)
    assert good["complete"] is True


# --------------------------------------------------------------------------- #
# over gRPC, through the client tool
# --------------------------------------------------------------------------- #

def test_the_client_tool_flies_a_hil_episode_over_grpc():
    if _M is None:
        return SKIP
    import harness_client
    h, rigs, td = _rail_harness(expect=2)
    port = _free_port()
    zone, tr = td / "zone.json", td / "translate.json"
    zone.write_text(json.dumps(ZONE), encoding="utf-8")
    tr.write_text(json.dumps(TRANSLATE), encoding="utf-8")
    srv = H.serve(h)                              # plaintext, loopback
    try:
        base = ["--insecure", "--target", srv.address, "--timeout", "60"]
        with _fast_polls(), contextlib.redirect_stdout(io.StringIO()) as out:
            rc = harness_client.main(base + [
                "run", "--backend", "hil", "--rest-endpoint", f"http://127.0.0.1:{port}",
                "--scenario", "speed-cap", "--seed", "1001", "--end-timeout", "120",
                "--hot-apply", "0.2", str(zone), "--hot-apply", "0.6", str(tr)])
        text = out.getvalue()
        assert rc == 0, text
        assert "on the hil rail" in text and "(launched on the hil rail)" in text, text
        assert f"queued for PATCH http://127.0.0.1:{port}/dynamic_nfz/nfz-hot" in text, text
        assert "dynamic_nfz/translate nfz-hot (over rest): generation 2" in text, text
        assert "episode.bag2/" in text and "hil rail run:" in text, text
        assert "NOT KPI-grade" in text
        # a usage error is refused before any call
        with contextlib.redirect_stderr(io.StringIO()) as err:
            rc = harness_client.main(base + ["run", "--backend", "hil", "--scenario",
                                             "speed-cap", "--hot-apply", "0.2", str(zone)])
        assert rc == 2 and "--rest-endpoint" in err.getvalue(), err.getvalue()
        assert len(rigs) == 1
        # the typed fields, and a server with no rig
        with harness_client.HarnessClient(srv.address, insecure=True, timeout=60) as c:
            s = c.start_session(backend="hil", seed=1001)
            assert s.backend == _M.pb.BACKEND_HIL
            ref = c.start_episode(scenario_id="speed-cap", session_id=s.session_id)
            st = c.status(ref.episode_id)
            assert st.backend == "hil" and st.state in (_M.pb.RUNNING, _M.pb.ENDED), st
            b = c.end_episode(ref.episode_id, timeout=120)
            assert b.backend == "hil" and b.rail_run_path and dict(b.bundle_files), b
            c.stop_session(s.session_id)
    finally:
        srv.stop(None)
    plain = H.serve(H.Harness(runs_dir=_tmp("hrail_g_")))
    try:
        with harness_client.HarnessClient(plain.address, insecure=True, timeout=30) as c:
            try:
                c.start_session(backend="hil")
            except _M.grpc.RpcError as e:
                assert e.code().name == "UNIMPLEMENTED", e.code()
            else:
                raise AssertionError("a server with no rig opened a hil session")
    finally:
        plain.stop(None)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")
           and (len(sys.argv) < 2 or any(a in k for a in sys.argv[1:]))]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            import traceback
            where = traceback.extract_tb(e.__traceback__)[-1]
            print(f"FAIL  {fn.__name__}\n      line {where.lineno}: {where.line}\n      {e}")
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            import traceback
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
            traceback.print_exc()
    _cleanup()
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
