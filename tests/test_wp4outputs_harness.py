"""The gRPC control plane's sessions, live mode, REST forwarding and streams
(guardrail/harness_service.py, tools/harness_client.py), 2026-10-12.

    python tests/test_wp4outputs_harness.py      (no pytest needed)

What must hold, and how each test tries to break it:

  * Live mode is the same flight. A live episode (EpisodeRunner stepped on a
    thread, paced) with a hot-apply injected while it flies - at a future
    at_sim_t, or `now` - flies byte for byte as the scheduled episode with
    the same event at the time the ack reports; the null, the same live
    episode without it, flies differently. A time already flown is
    OUT_OF_RANGE, `now` or `pace` on a scheduled episode INVALID_ARGUMENT.
  * StopSession stops a live episode (FAILED, no bundle) and fails an ARMED
    one; nothing is accepted after it.
  * InjectPolicy pins the hash (a wrong one is FAILED_PRECONDITION, a path
    outside policies/ and bundles/ PERMISSION_DENIED, a missing one
    INVALID_ARGUMENT) and the session's episodes fly the injected policy -
    beside the null, an episode outside the session flying the scenario's.
    An archive sent as bytes is kept and flown, its signature status said.
  * IssueTask puts the prompt's SHA-256 in manifest_extras.json, and the
    flight log is the same as without it (the headless pilot reads no text:
    the null that the field is a record, not an input).
  * A HotApply addressed to a session bound to a REST endpoint is POSTed to
    <endpoint>/hot_apply (a stdlib server running guardrail/api.py's own
    handler), with the endpoint's 409 / 404 / 422 as gRPC codes and a dead
    endpoint UNAVAILABLE; an environment proxy is not used; an endpoint off
    loopback is refused unless the server allows its host; InjectPolicy in a
    bound session refuses a policy the endpoint does not fly.
  * backend SITL is reserved: UNIMPLEMENTED.
  * The streams replay a session from its start and end when it stops; the
    whole page lifecycle runs over mutual TLS, and its live bundle re-flies
    identically through tools/rerun_episode.py.

On the 4418ae0 copy none of these RPCs or arguments exist: every test fails
there (a TypeError on start_episode's new keywords, or AttributeError).

Added 2026-10-13 (review of the 2026-10-12 version):

  * A live EmitEvent flies: a spawn emitted while the episode flies is the
    same flight as the scheduled episode with that event, and not the null.
  * A session's seed is its episodes' default; an unset StartSession seed
    stays unset over gRPC (not 0).
  * A telemetry frame's `executed` is the row's `emitted` (what flew), not
    the raw command.
  * StopSession stops the stepping thread (ticks_flown stops rising) and
    EndEpisode lets a slow live episode finish unpaced.
  * An idle session expires (its stream says so) and frees its slot under
    MAX_OPEN_SESSIONS; one with an armed episode does not.
  * ticks_flown is unset on a scheduled episode until it has ENDED, then the
    flown rows; over gRPC an unset field, not 0.
"""
from __future__ import annotations

import contextlib
import hashlib
import io
import json
import os
import shutil
import sys
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

from guardrail import harness_service as H                       # noqa: E402
from guardrail import load_policy                                  # noqa: E402

SKIP = "SKIP"
_TMP: list[Path] = []
try:
    _M = H._grpc_modules()
except H.HarnessUnavailable:
    _M = None
try:
    import cryptography                                           # noqa: F401
    _TLS = _M is not None
except ImportError:
    _TLS = False

SIM = ROOT / "policies" / "sim_demo_policy.yaml"
DEMO = ROOT / "policies" / "demo_policy.yaml"
OPEN = """
scenario_id: wp4-open-flight
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
ZONE = {"type": "dynamic_nfz", "id": "nfz-hot",
        "vertices": [{"x": 10, "y": -30}, {"x": 20, "y": -30},
                     {"x": 20, "y": -10}, {"x": 10, "y": -10}]}
JITTER = """
scenario_id: wp4-jitter
template: static_nfz
description: "Head-on into the demo zone from a start drawn from the seed."
mission:
  start_pose: {x: -20.0, y: 15.0, up: 15.0}
  target: {x: 40.0, y: 15.0, up: 15.0}
  pilot: {type: goto, speed: 4.0}
policy: {path: policies/sim_demo_policy.yaml}
stress: {start_jitter_m: 3.0}
expected_kpis:
  P0_escape_rate: 0
"""
# A zone spawned 15 m ahead of the aircraft at 5 s: the runner places it
# against the flying vehicle, so only a live injection can reproduce it.
AHEAD = {"id": "nfz-ahead", "width_m": 10.0, "height_m": 10.0, "ahead_of_vehicle_m": 15.0}


def _tmp(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


_H: dict = {}


def _harness() -> H.Harness:
    if "h" not in _H:
        _H["h"] = H.Harness(runs_dir=_tmp("wp4h_runs_"))
    return _H["h"]


def _refused(fn, *a, **kw) -> tuple[str, str]:
    try:
        fn(*a, **kw)
    except H.HarnessError as e:
        return e.code, e.message
    raise AssertionError(f"{getattr(fn, '__name__', fn)} was accepted")


def _log(b: dict) -> bytes:
    return (Path(b["bundle_path"]) / "flight_log.jsonl").read_bytes()


def _scheduled(h, hot=(), **kw) -> dict:
    ref = h.start_episode(scenario_yaml=OPEN, keep_flight_log=True, **kw)
    for at, body in hot:
        h.hot_apply(ref["episode_id"], at, body)
    return h.end_episode(ref["episode_id"])


def _wait_until(pred, timeout=20.0):
    t = time.monotonic()
    while time.monotonic() - t < timeout:
        if pred():
            return True
        time.sleep(0.01)
    return False


# --------------------------------------------------------------------------- #
# live mode
# --------------------------------------------------------------------------- #

def test_a_live_episode_with_an_injected_hot_apply_flies_as_the_scheduled_one():
    h = _harness()
    # a future time, injected while the episode flies (3 s at pace 4 is 0.75 s
    # of wall time away; EndEpisode then flies the rest unpaced)
    ref = h.start_episode(scenario_yaml=OPEN, keep_flight_log=True, live=True, pace=4.0)
    assert ref["live"] is True
    ack = h.hot_apply(ref["episode_id"], 3.0, ZONE)
    assert ack["at_sim_t"] == 3.0 and ack["validated_generation"] == 1
    live = h.end_episode(ref["episode_id"])
    assert live["live"] is True
    assert [a["t"] for a in live["applied_events"]] == [3.0], live["applied_events"]
    sched = _scheduled(h, hot=[(3.0, ZONE)])
    assert _log(live) == _log(sched), "the live flight differs from the scheduled one"
    # `now`: whatever tick it lands on, the ack names it and the scheduled
    # episode with the event at that time is the same flight
    ref = h.start_episode(scenario_yaml=OPEN, keep_flight_log=True, live=True, pace=15.0)
    assert _wait_until(lambda: h.status(ref["episode_id"]).get("ticks_flown", 0) >= 5)
    ack = h.hot_apply(ref["episode_id"], 0.0, ZONE, now=True)
    at = ack["at_sim_t"]
    assert 0.4 <= at < 29.9, at
    live_now = h.end_episode(ref["episode_id"])
    assert _log(live_now) == _log(_scheduled(h, hot=[(at, ZONE)])), at
    # the null: the same live episode with no hot-apply is another flight
    ref = h.start_episode(scenario_yaml=OPEN, keep_flight_log=True, live=True, pace=0.0)
    assert _log(h.end_episode(ref["episode_id"])) != _log(live)


def test_now_lands_on_the_next_tick_to_fly():
    """`now` is the next tick, not a later one. At pace 0.05 a tick takes 2 s
    of wall time, so the tick GetStatus names as next is still next when the
    HotApply arrives a few milliseconds later."""
    h = _harness()
    ref = h.start_episode(scenario_yaml=OPEN, live=True, pace=0.05)
    eid = ref["episode_id"]
    assert _wait_until(lambda: h.status(eid).get("ticks_flown", 0) >= 1)
    nxt = h.status(eid)["next_tick_sim_t"]
    ack = h.hot_apply(eid, 0.0, ZONE, now=True)
    assert abs(ack["at_sim_t"] - nxt) < 1e-6, (ack["at_sim_t"], nxt)
    # Paced, the rest of this episode would take ten minutes of wall time;
    # EndEpisode flies it as fast as the host can.
    t0 = time.monotonic()
    b = h.end_episode(eid)
    assert time.monotonic() - t0 < 60.0, "EndEpisode let the live episode finish paced"
    assert [round(a["t"], 6) for a in b["applied_events"]] == [round(nxt, 6)]


def test_live_timing_rules_and_their_codes():
    h = _harness()
    ref = h.start_episode(scenario_yaml=OPEN, live=True, pace=10.0)
    eid = ref["episode_id"]
    assert _wait_until(lambda: (h.status(eid).get("next_tick_sim_t") or 0) > 2.0)
    code, why = _refused(h.hot_apply, eid, 1.0, ZONE)
    assert code == "OUT_OF_RANGE" and "already flown" in why, (code, why)
    code, _ = _refused(h.emit_event, eid, 1.0, "scale_radius", {"factor": 2})
    assert code == "OUT_OF_RANGE", code
    st = h.status(eid)
    assert st["state"] == "RUNNING" and st["live"] and st["ticks_flown"] > 20, st
    h.end_episode(eid)
    code, _ = _refused(h.hot_apply, eid, 1.0, ZONE, now=True)
    assert code == "FAILED_PRECONDITION", code
    sch = h.start_episode(scenario_yaml=OPEN)
    code, why = _refused(h.hot_apply, sch["episode_id"], 1.0, ZONE, now=True)
    assert code == "INVALID_ARGUMENT" and "live" in why, why
    code, _ = _refused(h.start_episode, scenario_yaml=OPEN, pace=2.0)
    assert code == "INVALID_ARGUMENT"
    code, _ = _refused(h.start_episode, scenario_yaml=OPEN, live=True, pace=-1.0)
    assert code == "INVALID_ARGUMENT"


def test_stop_session_stops_a_live_episode_unbundled_and_fails_an_armed_one():
    h = H.Harness(runs_dir=_tmp("wp4h_stop_"))
    s = h.start_session()["session_id"]
    live = h.start_episode(scenario_yaml=OPEN, session_id=s, live=True, pace=1.0)["episode_id"]
    armed = h.start_episode(scenario_yaml=OPEN, session_id=s)["episode_id"]
    assert _wait_until(lambda: h.status(live)["ticks_flown"] >= 2)
    out = h.stop_session(s)
    assert out["episodes"] == 2 and out["episodes_closed_unfinished"] == 2, out
    for eid in (live, armed):
        st = h.status(eid)
        assert st["state"] == "FAILED" and "stopped" in st["error"], st
        assert _refused(h.end_episode, eid)[0] == "FAILED_PRECONDITION"
    time.sleep(0.3)                                  # the stepping thread has seen it
    assert h.status(live)["state"] == "FAILED"
    # ... and stopped stepping: at pace 1.0 a running loop adds ~10 ticks/s,
    # and a loop that ignored the stop would fly the rest at once (the stop
    # also wakes its pause) to the episode's 300 ticks
    n = h.status(live)["ticks_flown"]
    assert n < 100, f"the live loop flew {n} ticks of a stopped episode"
    time.sleep(0.6)
    assert h.status(live)["ticks_flown"] == n, "the live loop kept flying after the stop"
    assert not any(h.runs_dir.rglob("kpi.json")), "a stopped episode was bundled"
    assert _refused(h.start_episode, scenario_yaml=OPEN, session_id=s)[0] == \
        "FAILED_PRECONDITION"
    assert _refused(h.issue_task, session_id=s, prompt="x")[0] == "FAILED_PRECONDITION"
    items, _, done = h.wait_stream(s, "events", 0, 0.1)
    assert done and items[-1]["kind"] == "session_stopped", [i["kind"] for i in items]
    assert h.stop_session(s)["episodes_closed_unfinished"] == 0     # idempotent


def test_a_live_emit_event_flies_as_the_scheduled_one():
    h = _harness()
    ref = h.start_episode(scenario_yaml=OPEN, keep_flight_log=True, live=True, pace=2.0)
    ack = h.emit_event(ref["episode_id"], 5.0, "spawn_polygon_fence", AHEAD)
    assert ack["at_sim_t"] == 5.0
    live = h.end_episode(ref["episode_id"])
    sch = h.start_episode(scenario_yaml=OPEN, keep_flight_log=True)
    h.emit_event(sch["episode_id"], 5.0, "spawn_polygon_fence", AHEAD)
    sched = h.end_episode(sch["episode_id"])
    assert _log(live) == _log(sched), "the live EmitEvent flew differently"
    null = h.start_episode(scenario_yaml=OPEN, keep_flight_log=True, live=True, pace=0.0)
    assert _log(h.end_episode(null["episode_id"])) != _log(live), \
        "the spawned zone changed nothing: the comparison is empty"


def test_a_sessions_seed_is_the_default_of_its_episodes():
    h = _harness()
    s7 = h.start_session(seed=7)["session_id"]
    inside = h.end_episode(h.start_episode(scenario_yaml=JITTER, session_id=s7,
                                           keep_flight_log=True)["episode_id"])
    assert json.loads(inside["manifest_json"])["random_seed"] == 7
    # an episode's own seed wins over the session's
    own = h.end_episode(h.start_episode(scenario_yaml=JITTER, session_id=s7, seed=8,
                                        keep_flight_log=True)["episode_id"])
    assert json.loads(own["manifest_json"])["random_seed"] == 8
    # the same seed outside the session is the same flight; another is not
    out7 = h.end_episode(h.start_episode(scenario_yaml=JITTER, seed=7,
                                         keep_flight_log=True)["episode_id"])
    assert _log(inside) == _log(out7) and _log(inside) != _log(own)


def test_an_idle_session_expires_and_frees_its_slot():
    h = H.Harness(runs_dir=_tmp("wp4h_ttl_"), session_idle_ttl_s=0.3)
    saved = H.MAX_OPEN_SESSIONS
    H.MAX_OPEN_SESSIONS = 2
    try:
        a = h.start_session()["session_id"]
        b = h.start_session()["session_id"]
        h.start_episode(scenario_yaml=OPEN, session_id=b)      # b has an ARMED episode
        # the null: inside the TTL the cap holds
        assert _refused(h.start_session)[0] == "RESOURCE_EXHAUSTED"
        time.sleep(0.5)
        c = h.start_session()["session_id"]                    # a expired; b did not
        items, _, done = h.wait_stream(a, "events", 0, 0.1)
        kinds = [i["kind"] for i in items]
        assert done and kinds[-2:] == ["session_expired", "session_stopped"], kinds
        assert h._session(b).state == "OPEN" and h._session(c).state == "OPEN"
        assert _refused(h.start_episode, scenario_yaml=OPEN, session_id=a)[0] == \
            "FAILED_PRECONDITION"
        # a call on a session keeps it alive past the TTL it would have had;
        # its null, a session opened beside it with no call, expires
        H.MAX_OPEN_SESSIONS = 4
        d = h.start_session()["session_id"]
        for _ in range(4):
            time.sleep(0.15)
            h.issue_task(session_id=c, prompt="keep going")
        h.start_session()                                      # sweeps the idle ones
        assert h._session(c).state == "OPEN" and h._session(d).state == "STOPPED"
    finally:
        H.MAX_OPEN_SESSIONS = saved
    try:
        H.Harness(runs_dir=_tmp("wp4h_ttl2_"), session_idle_ttl_s=-1)
    except ValueError:
        pass
    else:
        raise AssertionError("a negative session TTL was accepted")


def test_ticks_flown_is_unset_until_it_is_known():
    h = _harness()
    ref = h.start_episode(scenario_yaml=OPEN, keep_flight_log=True)
    assert "ticks_flown" not in h.status(ref["episode_id"]), h.status(ref["episode_id"])
    b = h.end_episode(ref["episode_id"])
    rows = _log(b).decode("utf-8").splitlines()
    assert h.status(ref["episode_id"])["ticks_flown"] == len(rows) == 300
    if _M is None:
        return None
    with _server() as srv, _client(srv, insecure=True) as c:
        armed = c.start_episode(scenario_yaml=OPEN)
        st = c.status(armed.episode_id)
        assert not st.HasField("ticks_flown"), "an unflown episode reported ticks_flown"
        c.end_episode(armed.episode_id)
        st = c.status(armed.episode_id)
        assert st.HasField("ticks_flown") and st.ticks_flown == 300, st


# --------------------------------------------------------------------------- #
# InjectPolicy, IssueTask, backends
# --------------------------------------------------------------------------- #

def test_inject_policy_pins_its_hash_and_the_session_flies_it():
    h = _harness()
    demo, sim = load_policy(DEMO), load_policy(SIM)
    s = h.start_session()["session_id"]
    rel = DEMO.relative_to(ROOT).as_posix()
    code, why = _refused(h.inject_policy, s, bundle_path=rel, expected_hash=sim.policy_hash)
    assert code == "FAILED_PRECONDITION" and demo.policy_hash in why, why
    assert _refused(h.inject_policy, s, bundle_path=rel)[0] == "INVALID_ARGUMENT"
    assert _refused(h.inject_policy, s, bundle_path="README.md",
                    expected_hash="sha256:00")[0] == "PERMISSION_DENIED"
    assert _refused(h.inject_policy, s, bundle_path="policies/nope.yaml",
                    expected_hash="sha256:00")[0] == "NOT_FOUND"
    ack = h.inject_policy(s, bundle_path=rel, expected_hash=demo.policy_hash)
    assert ack["policy_hash"] == demo.policy_hash and ack["generation"] == 0
    assert ack["signature"] == "unsigned", ack
    inside = _scheduled(h, session_id=s)
    outside = _scheduled(h)
    assert json.loads(inside["manifest_json"])["policy_hash"] == demo.policy_hash
    assert json.loads(outside["manifest_json"])["policy_hash"] == sim.policy_hash
    assert inside["session_id"] == s and outside["session_id"] == ""


def test_inject_policy_takes_an_archive_as_bytes():
    from guardrail.bundle import write_bundle
    h = _harness()
    d = _tmp("wp4h_bytes_")
    pol = load_policy(DEMO)
    arch = write_bundle(pol, d / "demo.tar.gz", signer=None)
    data = Path(arch).read_bytes()
    s = h.start_session()["session_id"]
    ack = h.inject_policy(s, bundle_bytes=data, expected_hash=pol.policy_hash)
    assert ack["policy_hash"] == pol.policy_hash and ack["signature"] == "unsigned", ack
    b = _scheduled(h, session_id=s)
    assert json.loads(b["manifest_json"])["policy_hash"] == pol.policy_hash
    kept = list((h.runs_dir / "sessions" / s).glob("policy-*.tar.gz"))
    assert len(kept) == 1 and kept[0].read_bytes() == data
    bad = data[:-40] + bytes(40)
    assert _refused(h.inject_policy, s, bundle_bytes=bad,
                    expected_hash=pol.policy_hash)[0] == "INVALID_ARGUMENT"


def test_issue_task_is_recorded_by_hash_and_changes_nothing_flown():
    h = _harness()
    prompt = "Fly east along the road to the blue tower at fifteen metres."
    s = h.start_session()["session_id"]
    ack = h.issue_task(session_id=s, prompt=prompt)
    want = "sha256:" + hashlib.sha256(prompt.encode("utf-8")).hexdigest()
    assert ack["prompt_sha256"] == want and ack["pilot_reads_text"] is False
    told = _scheduled(h, session_id=s)
    ex = json.loads((Path(told["bundle_path"]) / "manifest_extras.json").read_text(
        encoding="utf-8"))
    assert ex["task_prompt_sha256"] == want and ex["task_read_by_pilot"] is False, ex
    assert told["task_prompt_sha256"] == want
    req = json.loads((Path(told["bundle_path"]) / "harness_request.json").read_text(
        encoding="utf-8"))
    assert req["task"]["text"] == prompt
    # the null: the same episode with no task flies the same log
    assert _log(told) == _log(_scheduled(h))
    # an episode-level task, only before it flies
    ref = h.start_episode(scenario_yaml=OPEN)
    h.issue_task(episode_id=ref["episode_id"], prompt="other")
    h.end_episode(ref["episode_id"])
    assert _refused(h.issue_task, episode_id=ref["episode_id"], prompt="late")[0] == \
        "FAILED_PRECONDITION"
    assert _refused(h.issue_task, session_id=s, prompt="  ")[0] == "INVALID_ARGUMENT"


def test_the_sitl_backend_is_reserved_and_endpoints_are_checked():
    h = _harness()
    code, why = _refused(h.start_session, backend="sitl")
    assert code == "UNIMPLEMENTED" and "reserved" in why, (code, why)
    assert _refused(h.start_session, backend="gazebo")[0] == "INVALID_ARGUMENT"
    assert _refused(h.start_session, rest_endpoint="ftp://127.0.0.1:1")[0] == \
        "INVALID_ARGUMENT"
    assert _refused(h.start_session, rest_endpoint="http://10.1.2.3:8071")[0] == \
        "PERMISSION_DENIED"
    allowed = H.Harness(runs_dir=_tmp("wp4h_allow_"), rest_hosts=("10.1.2.3",),
                        manifest_base=h._manifest_base)
    ref = allowed.start_session(rest_endpoint="http://10.1.2.3:8071/")
    assert ref["rest_endpoint"] == "http://10.1.2.3:8071"
    s = h.start_session()["session_id"]
    code, why = _refused(h.hot_apply, "", 0.0, ZONE, session_id=s)
    assert code == "FAILED_PRECONDITION" and "not bound" in why, why
    assert _refused(h.hot_apply, "ep-x", 0.0, ZONE, session_id=s)[0] == "INVALID_ARGUMENT"
    assert _refused(h.stop_session, "ss-nope")[0] == "NOT_FOUND"


# --------------------------------------------------------------------------- #
# REST forwarding, against guardrail/api.py's own handler
# --------------------------------------------------------------------------- #

class _Endpoint:
    """guardrail/api.py's routes /hot_apply and /health on a live Shield, served
    by the standard library (no FastAPI here): each call runs the same handler
    build_app runs, and a refusal answers its status with {"detail": ...} as
    FastAPI's HTTPException does. Records every body it was sent."""

    def __init__(self, policy_path: Path):
        from guardrail import api
        from guardrail.shield import Shield
        self.shield = Shield(load_policy(policy_path), escalation=False)
        self.bodies: list[dict] = []
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

            def do_GET(self):
                if self.path == "/health":
                    self._send(200, api.health(ep.shield))
                else:
                    self._send(404, {"detail": "Not Found"})

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                ep.bodies.append(body)
                if self.path != "/hot_apply":
                    return self._send(404, {"detail": "Not Found"})
                try:
                    self._send(200, api.apply_event(ep.shield, body))
                except api.ApiError as e:
                    self._send(e.status, {"detail": e.detail})

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def test_a_session_hot_apply_is_posted_to_its_rest_endpoint():
    h = _harness()
    ep = _Endpoint(SIM)
    old_proxy = {k: os.environ.get(k) for k in ("HTTP_PROXY", "http_proxy", "NO_PROXY",
                                                 "no_proxy")}
    # A proxy in the environment must not see a loopback hot-apply.
    os.environ["HTTP_PROXY"] = os.environ["http_proxy"] = "http://127.0.0.1:9"
    for k in ("NO_PROXY", "no_proxy"):
        os.environ.pop(k, None)
    try:
        s = h.start_session(rest_endpoint=ep.url)["session_id"]
        # InjectPolicy checks the endpoint flies the injected policy
        demo, sim = load_policy(DEMO), load_policy(SIM)
        code, why = _refused(h.inject_policy, s, bundle_path="policies/demo_policy.yaml",
                             expected_hash=demo.policy_hash)
        assert code == "FAILED_PRECONDITION" and sim.policy_hash in why, why
        ack = h.inject_policy(s, bundle_path="policies/sim_demo_policy.yaml",
                              expected_hash=sim.policy_hash)
        assert ack["endpoint_policy_hash"] == sim.policy_hash
        ack = h.hot_apply("", 0.0, ZONE, session_id=s)
        assert ack["forwarded_to"] == ep.url + "/hot_apply", ack
        assert ep.bodies == [ZONE], ep.bodies
        assert ack["rule_id"] == "nfz-hot" and ack["generation"] == 1, ack
        assert ack["policy_hash"] == ep.shield.policy.policy_hash != sim.policy_hash
        # the endpoint's refusals, as the REST endpoint means them
        assert _refused(h.hot_apply, "", 0.0, ZONE, session_id=s)[0] == "ALREADY_EXISTS"
        sw = {"type": "time_window_switch", "id": "sw", "target_id": "no-such-rule",
              "active": False}
        assert _refused(h.hot_apply, "", 0.0, sw, session_id=s)[0] == "NOT_FOUND"
        locked = dict(ZONE, type="polygon_fence", id="pf")
        assert _refused(h.hot_apply, "", 0.0, locked, session_id=s)[0] == "INVALID_ARGUMENT"
        items, _, _ = h.wait_stream(s, "events", 0, 0.1)
        fw = [json.loads(i["detail_json"]) for i in items if i["kind"] == "hot_apply_forwarded"]
        assert len(fw) == 4 and "response" in fw[0] and fw[1]["refused"] == "ALREADY_EXISTS"
    finally:
        ep.close()
        for k, v in old_proxy.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    code, why = _refused(h.hot_apply, "", 0.0, dict(ZONE, id="nfz-2"), session_id=s)
    assert code == "UNAVAILABLE" and "did not answer" in why, (code, why)


# --------------------------------------------------------------------------- #
# over gRPC: the streams, the lifecycle on mTLS, the client tool
# --------------------------------------------------------------------------- #

@contextlib.contextmanager
def _server(**kw):
    srv = H.serve(kw.pop("harness", None) or _harness(), **kw)
    try:
        yield srv
    finally:
        srv.stop(None)


def _client(srv, **kw):
    import harness_client
    return harness_client.HarnessClient(srv.address, timeout=60, **kw)


def test_the_streams_replay_the_session_from_its_start_and_end_when_it_stops():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        unseeded = c.start_session(scene="open-ground")
        assert not unseeded.HasField("seed"), "an unset seed came back as 0"
        c.stop_session(unseeded.session_id)
        s = c.start_session(scene="open-ground", seed=0).session_id
        got: dict[str, list] = {"events": [], "frames": []}

        def watch():                                 # subscribed before anything flies
            for ev in c.stream_events(s, timeout=120):
                got["events"].append(ev)
        t = threading.Thread(target=watch)
        t.start()
        ref = c.start_episode(scenario_yaml=OPEN, session_id=s, keep_flight_log=True)
        c.hot_apply(ref.episode_id, 3.0, ZONE)
        b = c.end_episode(ref.episode_id)
        live = c.start_episode(scenario_yaml=OPEN, session_id=s, live=True, pace=0.0)
        c.end_episode(live.episode_id)
        stop = c.stop_session(s)
        t.join(30)
        assert not t.is_alive(), "the event stream did not end with the session"
        late = list(c.stream_events(s, timeout=30))     # a late subscriber: everything
        frames = list(c.stream_telemetry(s, timeout=30))
    kinds = [e.kind for e in got["events"]]
    assert kinds[0] == "session_started" and kinds[-1] == "session_stopped", kinds
    assert [e.seq for e in got["events"]] == list(range(len(kinds)))
    assert [e.kind for e in late] == kinds and stop.events == len(kinds)
    assert kinds.count("episode_ended") == 2 and "hot_apply" in kinds
    applied = [json.loads(e.detail_json) for e in got["events"] if e.kind == "run_event"
               and json.loads(e.detail_json).get("type") == "hot_apply"]
    assert applied and applied[0]["generation"] == 1
    rows = (Path(b.bundle_path) / "flight_log.jsonl").read_text(encoding="utf-8").splitlines()
    first = [f for f in frames if f.episode_id == ref.episode_id]
    assert len(first) == len(rows) and all(not f.live for f in first)
    r0 = json.loads(rows[10])
    assert abs(first[10].x - r0["x"]) < 1e-9 and abs(first[10].planner_output.vx
                                                     - r0["raw"]["vx"]) < 1e-9
    # `executed` is what flew (the row's `emitted`), on a tick where the
    # Shield changed the command so the two cannot be confused
    parsed = [json.loads(x) for x in rows]
    k = next(i for i, r in enumerate(parsed) if abs(r["emitted"]["vy"] - r["raw"]["vy"]) > 1e-6
             or abs(r["emitted"]["vx"] - r["raw"]["vx"]) > 1e-6)
    ex, em = first[k].executed, parsed[k]["emitted"]
    assert (abs(ex.vx - em["vx"]), abs(ex.vy - em["vy"])) < (1e-9, 1e-9), (k, ex, em)
    clock = [f for f in frames if f.episode_id == live.episode_id and f.live]
    assert len(clock) == 300 and not clock[0].HasField("x") and "x" in clock[0].unset
    assert stop.telemetry_frames == len(frames) and stop.telemetry_dropped == 0


def test_the_pages_session_lifecycle_runs_over_mtls_and_its_live_bundle_reflies():
    if not _TLS:
        return SKIP
    import rerun_episode as RR
    d = _tmp("wp4h_pki_")
    H.generate_pki(d)
    sim = load_policy(SIM)
    with _server(pki_dir=d) as srv, _client(srv, pki_dir=d) as c:
        s = c.start_session(scene="open-ground", airframe="quad", seed=0)
        assert s.client == "harness-client" and s.backend == _M.pb.BACKEND_HEADLESS
        ack = c.inject_policy(s.session_id, bundle_path="policies/sim_demo_policy.yaml",
                              expected_hash=sim.policy_hash)
        assert (ack.policy_hash, ack.generation) == (sim.policy_hash, 0)
        c.issue_task("Fly east along the road.", session_id=s.session_id)
        ref = c.start_episode(scenario_yaml=OPEN, session_id=s.session_id, live=True,
                              pace=10.0, keep_flight_log=True)
        hot = c.hot_apply(ref.episode_id, 0.0, ZONE, now=True)
        assert hot.validated_generation == 1
        b = c.end_episode(ref.episode_id)
        stop = c.stop_session(s.session_id)
        try:
            c.start_session(backend="sitl")
        except _M.grpc.RpcError as e:
            assert e.code().name == "UNIMPLEMENTED", e.code()
        else:
            raise AssertionError("the reserved SITL backend answered")
    assert b.live and b.task_prompt_sha256.startswith("sha256:") and b.session_id
    assert [a.t for a in b.applied_events] == [hot.at_sim_t]
    assert stop.episodes == 1 and stop.telemetry_frames > 300
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        rc = RR.main([b.bundle_path, "--json"])
    rep = json.loads(buf.getvalue())
    assert rc == 0 and rep["match"] is True, rep
    if hasattr(RR.sweep(), "episode_extras"):
        assert rep["rebuilt_from"] == "manifest_extras", rep["rebuilt_from"]


def test_the_client_tool_runs_a_live_session_and_forwards_to_an_endpoint():
    if _M is None:
        return SKIP
    import harness_client
    d = _tmp("wp4h_tool_")
    body = d / "zone.json"
    body.write_text(json.dumps(ZONE), encoding="utf-8")
    ep = _Endpoint(SIM)
    try:
        with _server() as srv:
            base = ["--insecure", "--target", srv.address]
            with contextlib.redirect_stdout(io.StringIO()) as out:
                rc = harness_client.main(base + [
                    "run", "--scenario", "nfz-head-on", "--seed", "1001", "--live",
                    # paced, so 20 s is still ahead when the HotApply arrives;
                    # EndEpisode then flies the rest unpaced
                    "--pace", "5", "--prompt", "hold the lane", "--hot-apply", "20", str(body)])
            text = out.getvalue()
            assert rc == 0, text
            assert "(live, flying)" in text and "stopped: 600 telemetry frames" in text, text
            with contextlib.redirect_stdout(io.StringIO()) as out:
                rc = harness_client.main(base + ["forward", "--rest-endpoint", ep.url,
                                                 str(body)])
            assert rc == 0 and f"forwarded to {ep.url}/hot_apply" in out.getvalue(), \
                out.getvalue()
            assert ep.bodies == [ZONE]
    finally:
        ep.close()


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
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
            print(f"FAIL  {fn.__name__}\n      {e}")
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    for d in _TMP:
        shutil.rmtree(d, ignore_errors=True)
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
