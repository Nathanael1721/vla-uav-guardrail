"""
The stress harness's gRPC control plane (guardrail/harness_service.py, the
stubs under guardrail/protos/, tools/harness_client.py).

    python tests/test_harness_service.py      (no pytest needed)

What must hold, and how each test tries to break it:

  * Both transports carry the same episode: an insecure round trip and a
    mutual-TLS round trip on localhost return byte-identical KPI tables, and
    those equal what experiments/sweep_scenarios.py's own run_episode + score
    produce for the same cell and seed.
  * mTLS refuses: a client with no certificate, one signed by another CA, a
    plaintext client on the TLS port (UNAVAILABLE, the handshake), and a
    certificate signed by the right CA but not on the allow-list
    (PERMISSION_DENIED) - with a positive control that the same certificate
    is let in once it is allowed.
  * A bad scenario is refused with a clear status code and creates nothing.
  * Events are not decoration: a hot-applied zone changes the flight (its null,
    the same episode without it, does not repair at all), and a HotApply or
    EmitEvent flies exactly as the same event written in the scenario - with
    non-default fields too (margin, breach action, motion, altitude band, a
    switch's time window), each beside a null showing the field matters.
  * Hot-applies are validated in the order they will fly (at_sim_t), not the
    order they arrived, so an ack means the flight will accept the body.
  * Determinism by seed, with its null: the same seed twice is byte-identical,
    and another seed flies differently - otherwise the first half would hold
    for a harness that ignored the seed.
  * No silent zeros: a figure the episode has no value for is UNSET with a
    reason, and the Shield's 0 P0 escapes sit beside a passthrough null that
    is not 0, while the Shield-off control scores escapes. The typed KPI row
    is checked field by field against guardrail.kpi.rollup, with the paths
    written out here rather than read from the service.
  * The verdict is the sweep's own function (sweep_scenarios.episode_verdict,
    the one its main() calls; the harness carried a copy until 2026-10-11):
    a stand-in put in its place decides the harness's status, every status
    is reached on a synthetic scenario, and the sweep's own run is compared
    episode by episode.
  * Lifecycle: an idle ARMED episode expires and frees its slot; a second
    EndEpisode waits for the flight in progress; the lab PKI defaults to a
    folder outside the checkout and refuses a cloud-synced one.

The logic tests drive `Harness` directly, so they run without grpcio; tests
that need grpcio (or cryptography) print SKIP where it is absent (never
PASS). They need no network beyond 127.0.0.1, no simulator, no WSL and no
GPU.
"""
from __future__ import annotations

import atexit
import contextlib
import dataclasses
import hashlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

os.environ.setdefault("GRPC_VERBOSITY", "ERROR")       # handshake failures are expected here

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
sys.path.insert(0, str(ROOT / "tools"))

import sweep_scenarios as S                                       # noqa: E402
from guardrail import harness_service as H                       # noqa: E402
from guardrail.scenario_spec import Episode, Library             # noqa: E402

SKIP = "SKIP"
_TMP: list[Path] = []

try:
    _M = H._grpc_modules()
except H.HarnessUnavailable:
    _M = None
try:
    import cryptography                                           # noqa: F401
    _TLS = _M is not None             # mTLS tests need the certificates too
except ImportError:
    _TLS = False


def _tmpdir(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


@atexit.register
def _cleanup():
    for d in _TMP:
        shutil.rmtree(d, ignore_errors=True)


_STATE: dict = {}


def _harness() -> H.Harness:
    if "harness" not in _STATE:
        _STATE["harness"] = H.Harness(runs_dir=_tmpdir("harness_runs_"))
    return _STATE["harness"]


def _pki() -> Path:
    if "pki" not in _STATE:
        d = _tmpdir("harness_pki_")
        H.generate_pki(d)
        H.issue_cert(d, "other-client")                  # signed by the CA, not allowed
        _STATE["pki"] = d
    return _STATE["pki"]


@contextlib.contextmanager
def _server(**kw):
    srv = H.serve(kw.pop("harness", None) or _harness(), **kw)
    try:
        yield srv
    finally:
        srv.stop(None)


def _client(srv, **kw):
    import harness_client
    if srv.secure and "insecure" not in kw and "ca_cert" not in kw:
        kw.setdefault("pki_dir", _pki())
    return harness_client.HarnessClient(srv.address, timeout=kw.pop("timeout", 60), **kw)


def _refused(fn, *a, **kw) -> tuple[str, str]:
    try:
        fn(*a, **kw)
    except _M.grpc.RpcError as e:
        return e.code().name, e.details() or ""
    raise AssertionError(f"{getattr(fn, '__name__', fn)} was accepted")


def _kpi(bundle) -> dict:
    return json.loads(bundle.kpi_json)


# ---- the same, on Harness directly (no transport) ------------------------- #

def _own_harness(prefix: str, **kw) -> H.Harness:
    """A Harness of the test's own (limits, TTL or patches of its own)."""
    return H.Harness(runs_dir=_tmpdir(prefix), manifest_base=_harness()._manifest_base, **kw)


def _h_refused(fn, *a, what: str = "", **kw) -> tuple[str, str]:
    try:
        fn(*a, **kw)
    except H.HarnessError as e:
        return e.code, e.message
    raise AssertionError(what or f"{getattr(fn, '__name__', fn)} was accepted")


def _h_fly(h, yaml_text: str = "", *, scenario_id: str = "", hot=(), emit=(),
           seed: int | None = None, keep_log: bool = False):
    ref = h.start_episode(scenario_id=scenario_id, scenario_yaml=yaml_text, seed=seed,
                          keep_flight_log=keep_log)
    acks = [h.hot_apply(ref["episode_id"], at, body) for at, body in hot]
    acks += [h.emit_event(ref["episode_id"], at, t, p) for at, t, p in emit]
    return ref, acks, h.end_episode(ref["episode_id"])


def _log(bundle: dict) -> bytes:
    return (Path(bundle["bundle_path"]) / "flight_log.jsonl").read_bytes()


def _bundle_kpi(bundle: dict) -> dict:
    return json.loads((Path(bundle["bundle_path"]) / "kpi.json").read_text(encoding="utf-8"))


# ---- scenarios written for these tests ------------------------------------ #

OPEN = """
scenario_id: grpc-open-flight
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
# The zone hot-applied across OPEN's path (x 10..20 straddles y = -20).
ZONE = {"type": "dynamic_nfz", "id": "nfz-hot",
        "vertices": [{"x": 10, "y": -30}, {"x": 20, "y": -30},
                     {"x": 20, "y": -10}, {"x": 10, "y": -10}]}
OPEN_WITH_EVENT = OPEN + """
events:
  - at_sim_t: 3.0
    type: spawn_polygon_fence
    payload:
      id: nfz-hot
      vertices: [{x: 10, y: -30}, {x: 20, y: -30}, {x: 20, y: -10}, {x: 10, y: -10}]
"""
OPEN_WITH_AHEAD = OPEN + """
events:
  - at_sim_t: 2.0
    type: spawn_polygon_fence
    payload: {width_m: 20, height_m: 8, ahead_of_vehicle_m: 12}
"""
# Into the demo zone from a jittered start: the seed moves the start, so it
# moves the flight.
JITTER = """
scenario_id: grpc-jitter
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
SWEPT = OPEN.replace("speed: 4.0}", "speed: $v}").replace(
    "scenario_id: grpc-open-flight", "scenario_id: grpc-swept") + """
parameter_sweep:
  v: [3.0, 4.0]
"""
# OPEN with a task prompt of its own: no stored paraphrase set holds it.
TOLD = OPEN.replace("pilot: {type: goto, speed: 4.0}",
                    "pilot: {type: goto, speed: 4.0}\n  task_prompt: \"Fly east along "
                    "the road to the blue tower at fifteen metres.\"")
# OPEN with a mission clock: Monday 2026-10-05, 10:00 at t = 0.
CLOCKED = OPEN.replace("pilot: {type: goto, speed: 4.0}",
                       "pilot: {type: goto, speed: 4.0}\n  clock_start: '2026-10-05T10:00:00'")
# A switch that turns the hot zone off, and the same switch on Tuesdays only.
SW_OFF = {"type": "time_window_switch", "id": "sw1", "target_id": "nfz-hot", "active": False}
SW_OFF_TUE = dict(SW_OFF, valid_time={"recurrence": {"days": ["Tue"], "start_time": "09:00",
                                                     "end_time": "11:00"}})
# A zone far off OPEN's path (it changes nothing the aircraft does).
FAR = {"type": "dynamic_nfz", "id": "nfz-far",
       "vertices": [{"x": 100, "y": 100}, {"x": 110, "y": 100},
                    {"x": 110, "y": 110}, {"x": 100, "y": 110}]}


def _written(base: str, events: list[dict]) -> str:
    """`base` with `events:` written in the scenario (JSON is YAML)."""
    return base + "events:\n" + "".join(f"  - {json.dumps(e)}\n" for e in events)


def _as_spawn(at: float, body: dict) -> dict:
    """The spawn_polygon_fence a scenario author writes for a dynamic_nfz body."""
    return {"at_sim_t": at, "type": "spawn_polygon_fence",
            "payload": {k: v for k, v in body.items() if k != "type"}}


# --------------------------------------------------------------------------- #
# stubs and imports
# --------------------------------------------------------------------------- #

def test_committed_stubs_are_what_the_proto_generates():
    """The generated files are committed; a .proto edited without
    regenerating them must fail here, not at a client in the field."""
    try:
        import grpc_tools                                     # noqa: F401
    except ImportError:
        return SKIP
    out = _tmpdir("protoc_")
    (out / "guardrail" / "protos").mkdir(parents=True)
    shutil.copy(ROOT / "guardrail" / "protos" / "harness.proto",
                out / "guardrail" / "protos" / "harness.proto")
    r = subprocess.run([sys.executable, "-m", "grpc_tools.protoc", "-I", ".",
                        "--python_out=.", "--grpc_python_out=.",
                        "guardrail/protos/harness.proto"],
                       cwd=str(out), capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr
    for name in ("harness_pb2.py", "harness_pb2_grpc.py"):
        got = (out / "guardrail" / "protos" / name).read_bytes()
        have = (ROOT / "guardrail" / "protos" / name).read_bytes()
        assert got == have, f"guardrail/protos/{name} is stale: regenerate it"


def test_the_module_imports_without_grpc_and_names_what_is_missing():
    code = (
        "import sys\n"
        f"sys.path.insert(0, {str(ROOT)!r})\n"
        "for b in ('grpc', 'google.protobuf', 'google', 'cryptography'):\n"
        "    sys.modules[b] = None\n"
        "from guardrail import harness_service as H\n"
        "for fn in (H.serve, lambda: H.generate_pki('x')):\n"
        "    try:\n"
        "        fn()\n"
        "    except H.HarnessUnavailable as e:\n"
        "        print('UNAVAILABLE', e)\n"
        "print('kpi_row', H.kpi_row({'episodes': 1}, 'f', 'c', 'on')['unset']['p0_escape_rate'])\n")
    r = subprocess.run([sys.executable, "-B", "-c", code], cwd=str(ROOT),
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, (r.stdout + r.stderr)[-800:]
    assert "grpcio" in r.stdout and "cryptography" in r.stdout, r.stdout
    assert r.stdout.count("UNAVAILABLE") == 2, r.stdout
    assert "kpi_row not measured" in r.stdout, r.stdout


def test_stubs_refusing_the_installed_protobuf_is_named_not_a_traceback():
    """The generated harness_pb2 checks the protobuf runtime on import and
    raises protobuf's VersionError (not an ImportError) on a mismatch: that
    must reach the operator as HarnessUnavailable naming the versions."""
    if _M is None:
        return SKIP
    code = (
        "import sys\n"
        f"sys.path.insert(0, {str(ROOT)!r})\n"
        "from google.protobuf import runtime_version as rv\n"
        "real = rv.ValidateProtobufRuntimeVersion\n"
        "def refuse(*a, **k):\n"                    # only our gencode is refused
        "    if str(a[-1] if a else k.get('location', '')).endswith('harness.proto'):\n"
        "        raise rv.VersionError('Detected mismatched Protobuf Gencode/Runtime (test)')\n"
        "    return real(*a, **k)\n"
        "rv.ValidateProtobufRuntimeVersion = refuse\n"
        "from guardrail import harness_service as H\n"
        "try:\n"
        "    H.serve()\n"
        "except H.HarnessUnavailable as e:\n"
        "    print('UNAVAILABLE', e)\n")
    r = subprocess.run([sys.executable, "-B", "-c", code], cwd=str(ROOT),
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 0 and "UNAVAILABLE" in r.stdout, (r.stdout + r.stderr)[-800:]
    assert "VersionError" in r.stdout and "protobuf >= 7.35.1" in r.stdout, r.stdout


# --------------------------------------------------------------------------- #
# round trips
# --------------------------------------------------------------------------- #

def _round_trip(srv, **start) -> tuple:
    with _client(srv, **({"insecure": True} if not srv.secure else {})) as c:
        ref = c.start_episode(**start)
        st = c.status(ref.episode_id)
        assert _M.pb.EpisodeState.Name(st.state) == "ARMED", st
        b = c.end_episode(ref.episode_id)
        st2 = c.status(ref.episode_id)
    return ref, st2, b


def test_insecure_round_trip_writes_the_episode_bundle():
    """The episode bundle as the headless runner writes it: the grant's
    manifest.json, harness_events.jsonl and kpi.json, plus the request. (The
    grant's policy_bundle.tar.gz, episode.bag2/ and shield_audit.jsonl are
    not written by the headless runner; the DESIGN note says so.)"""
    if _M is None:
        return SKIP
    with _server() as srv:
        ref, st, b = _round_trip(srv, scenario_id="nfz-head-on", seed=1001)
    assert ref.seed == 1001 and ref.run_end_sim_t == 29.9 and ref.policy_hash.startswith("sha256:")
    # GetStatus agrees with EndEpisode. (Whether nfz-head-on passes its gates
    # is tests/test_sweep.py's question, not the transport's.)
    assert _M.pb.EpisodeState.Name(st.state) == "ENDED" and st.status == b.status, st
    assert st.bundle_path == b.bundle_path and st.client == ""
    d = Path(b.bundle_path)
    assert d.name.startswith("episode-") and d.name.endswith("--seed1001"), d.name
    for f in ("manifest.json", "kpi.json", "harness_events.jsonl", "harness_request.json"):
        assert (d / f).is_file(), f"{f} missing from {d}"
    # Not asked for, so not written.
    assert not (d / "flight_log.jsonl").exists()
    man = json.loads((d / "manifest.json").read_text(encoding="utf-8"))
    assert tuple(man) == S.MANIFEST_FIELDS, list(man)
    assert man["random_seed"] == 1001 and man["topology"] == "headless-kinematic"
    assert b.kpi_grade is False and any("sim_speedup" in r for r in b.kpi_grade_reasons)
    kd = json.loads((d / "kpi.json").read_text(encoding="utf-8"))
    assert kd["harness"]["transport"] == "grpc" and kd["kpi"] == _kpi(b)
    # The Shield's 0 beside a null that is not 0: the zero says something.
    assert b.kpi.HasField("p0_escape_rate") and b.kpi.p0_escape_rate == 0.0
    assert b.kpi.null_passthrough_p0_escape_rate > 0, b.kpi
    # An unset figure crosses the wire as unset (proto3 optional), with its
    # reason in the map - not as a 0.0 the client cannot tell apart.
    assert not b.kpi.HasField("mean_time_to_safe_s") and b.kpi.mean_time_to_safe_s == 0.0
    assert b.kpi.unset["mean_time_to_safe_s"] == "never_unsafe", dict(b.kpi.unset)
    assert json.loads(b.rollup_json)["mean_time_to_safe_s"] is None


def test_mtls_round_trip_carries_the_same_episode_and_names_the_client():
    if not _TLS:
        return SKIP
    with _server() as plain:
        _, _, b0 = _round_trip(plain, scenario_id="nfz-head-on", seed=1001)
    with _server(pki_dir=_pki()) as srv:
        assert srv.secure
        _, st, b1 = _round_trip(srv, scenario_id="nfz-head-on", seed=1001)
    assert st.client == H.DEFAULT_CLIENT_CN, st.client
    assert b1.kpi_json == b0.kpi_json, "the transport changed the episode"
    assert b1.kpi == b0.kpi


def test_the_service_scores_exactly_as_the_sweep_does():
    """The same cell and seed through sweep_scenarios' own functions: the
    KPI table, and the rollup the KPI row is read from (built here the way
    the sweep's main() builds it)."""
    from guardrail import kpi as K
    lib = Library(S.LIB)
    cell = lib.by_id()["nfz-head-on"].cells()[0]
    ep = Episode(cell=cell, seed=1001, paraphrase_arm=-1, prefix="on", ab=True)
    run = S.run_episode(cell.spec, lib.defaults, 1001, arm=S.build_arm(ep))
    want = S.score(cell.spec, run.rows, run.extra, run.policy, run.priorities)
    roll = K.rollup([{**want, "null_hover_mission": K.null_hover_mission(
        run.rows, {"reached_goal": run.extra.get("reached_goal")})}])
    _, _, b = _h_fly(_harness(), scenario_id="nfz-head-on", seed=1001)
    norm = lambda d: json.loads(json.dumps(H._json_safe(d), sort_keys=True))   # noqa: E731
    assert json.loads(b["kpi_json"]) == norm(want)
    assert json.loads(b["rollup_json"]) == norm(roll)


# --------------------------------------------------------------------------- #
# mutual TLS: who gets in
# --------------------------------------------------------------------------- #

def _no_episode_added(h, fn) -> tuple[str, str]:
    before = len(h._episodes)
    got = _refused(fn)
    assert len(h._episodes) == before, "a refused client created an episode"
    return got


def test_a_client_without_a_certificate_is_refused():
    if not _TLS:
        return SKIP
    ca, _, _ = H.pki_paths(_pki(), "x")
    with _server(pki_dir=_pki()) as srv:
        with _client(srv, ca_cert=ca, name="", timeout=10) as c:
            code, _ = _no_episode_added(srv.servicer.harness,
                                        lambda: c.start_episode(scenario_id="nfz-head-on"))
    assert code == "UNAVAILABLE", code


def test_a_client_certificate_from_another_ca_is_refused():
    if not _TLS:
        return SKIP
    rogue = _tmpdir("rogue_pki_")
    H.generate_pki(rogue)                       # a CA of its own, same names
    ca, _, _ = H.pki_paths(_pki(), "x")         # trusts the real server...
    _, cert, key = H.pki_paths(rogue, H.DEFAULT_CLIENT_CN)   # ...presents a rogue cert
    with _server(pki_dir=_pki()) as srv:
        with _client(srv, ca_cert=ca, cert=cert, key=key, timeout=10) as c:
            code, _ = _no_episode_added(srv.servicer.harness,
                                        lambda: c.start_episode(scenario_id="nfz-head-on"))
    assert code == "UNAVAILABLE", code


def test_a_plaintext_client_on_the_mtls_port_is_refused():
    if not _TLS:
        return SKIP
    with _server(pki_dir=_pki()) as srv:
        with _client(srv, insecure=True, timeout=10) as c:
            code, _ = _no_episode_added(srv.servicer.harness,
                                        lambda: c.start_episode(scenario_id="nfz-head-on"))
    assert code == "UNAVAILABLE", code


def test_a_certificate_from_the_ca_but_not_on_the_allow_list_is_denied():
    """The same CA will sign every SROS 2 node identity; a valid chain alone
    must not be enough to drive the harness."""
    if not _TLS:
        return SKIP
    with _server(pki_dir=_pki()) as srv:
        with _client(srv, name="other-client") as c:
            code, why = _no_episode_added(srv.servicer.harness,
                                          lambda: c.start_episode(scenario_id="nfz-head-on"))
            assert code == "PERMISSION_DENIED" and "other-client" in why, (code, why)
            code, _ = _refused(c.status, "ep-anything")
            assert code == "PERMISSION_DENIED", code
    # Positive control: the very same certificate, once allowed, gets in.
    with _server(pki_dir=_pki(), allowed_clients=("harness-client", "other-client")) as srv:
        with _client(srv, name="other-client") as c:
            ref = c.start_episode(scenario_id="nfz-head-on")
            assert c.status(ref.episode_id).client == "other-client"


def test_plaintext_is_refused_off_loopback_and_an_empty_allow_list_is_refused():
    if _M is None:
        return SKIP
    for host in ("0.0.0.0", "192.168.1.10", "::"):
        try:
            leaked = H.serve(_harness(), host=host)
        except ValueError as e:
            assert "loopback" in str(e)
        else:
            leaked.stop(None)                   # never leave it listening
            raise AssertionError(f"an insecure port opened on {host}")
    try:
        # Refused before any certificate is read, so no PKI is needed here.
        H.serve(_harness(), pki_dir=_tmpdir("no_pki_"), allowed_clients=())
    except ValueError:
        pass
    else:
        raise AssertionError("mTLS with an empty allow-list started")
    assert H.is_loopback("127.0.0.1") and H.is_loopback("[::1]") and H.is_loopback("localhost")
    assert not H.is_loopback("0.0.0.0") and not H.is_loopback("10.0.0.2")


# --------------------------------------------------------------------------- #
# refusals with clear status codes
# --------------------------------------------------------------------------- #

def test_a_bad_scenario_is_refused_with_invalid_argument_and_creates_nothing():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        h = srv.servicer.harness
        code, why = _no_episode_added(h, lambda: c.start_episode(
            scenario_yaml=OPEN.replace("policy:", "shiled: true\npolicy:")))
        assert code == "INVALID_ARGUMENT" and "shiled" in why, (code, why)
        # A swept parameter nothing reads: N cells of one flight.
        code, why = _no_episode_added(h, lambda: c.start_episode(
            scenario_yaml=OPEN + "parameter_sweep:\n  unused: [1, 2]\n"))
        assert code == "INVALID_ARGUMENT" and "nothing in the scenario reads" in why, why
        code, _ = _no_episode_added(h, lambda: c.start_episode(scenario_yaml="[1, 2"))
        assert code == "INVALID_ARGUMENT"
        code, why = _no_episode_added(h, lambda: c.start_episode())
        assert code == "INVALID_ARGUMENT" and "exactly one" in why
        # Its own event after the last tick: refused at start, not at the end.
        late = OPEN + ("events:\n  - {at_sim_t: 45.0, type: spawn_polygon_fence, "
                       "payload: {width_m: 4, height_m: 4}}\n")
        code, why = _no_episode_added(h, lambda: c.start_episode(scenario_yaml=late))
        assert code == "INVALID_ARGUMENT" and "never fires" in why, why
        # And a valid one still goes through (the refusals were about the input).
        assert c.start_episode(scenario_yaml=OPEN).episode_id.startswith("ep-")


def test_unknown_names_are_not_found():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        assert _refused(c.start_episode, scenario_id="no-such-family")[0] == "NOT_FOUND"
        assert _refused(c.status, "ep-000000000000")[0] == "NOT_FOUND"
        assert _refused(c.end_episode, "ep-000000000000")[0] == "NOT_FOUND"
        assert _refused(c.start_episode, scenario_id="nfz-head-on",
                        profile="nope")[0] == "NOT_FOUND"
        assert _refused(c.start_episode, scenario_id="nfz-head-on",
                        profile="../profiles/smoke")[0] == "INVALID_ARGUMENT"
        assert _refused(c.start_episode, scenario_yaml=SWEPT,
                        cell_id="grpc-swept@v=9")[0] == "NOT_FOUND"
        bad_policy = OPEN.replace("sim_demo_policy.yaml", "no_such_policy.yaml")
        assert _refused(c.start_episode, scenario_yaml=bad_policy)[0] == "NOT_FOUND"


def test_the_pages_ten_rpcs_and_get_status_are_served():
    """The Stress Testing page's ten RPCs and GetStatus are all served since
    2026-10-12 (until then StartSession, StopSession, InjectPolicy, IssueTask
    and the two streams answered UNIMPLEMENTED). Each is called raw with an
    empty request and must reach the servicer - a validation code, never
    UNIMPLEMENTED; the control, a method the .proto does not have, still
    answers UNIMPLEMENTED from gRPC itself."""
    if _M is None:
        return SKIP
    grpc = _M.grpc
    pb = _M.pb
    want = {"StartSession": None, "StopSession": "NOT_FOUND",
            "InjectPolicy": "NOT_FOUND", "HotApply": "NOT_FOUND",
            "StartEpisode": "INVALID_ARGUMENT", "IssueTask": "INVALID_ARGUMENT",
            "EmitEvent": "NOT_FOUND", "GetStatus": "NOT_FOUND", "EndEpisode": "NOT_FOUND",
            "StreamTelemetry": "NOT_FOUND", "StreamEvents": "NOT_FOUND"}
    served = set(pb.DESCRIPTOR.services_by_name["StressHarness"].methods_by_name)
    assert served == set(want), sorted(served ^ set(want))
    with _server() as srv, grpc.insecure_channel(srv.address) as ch:
        for rpc, code in want.items():
            kind = ch.unary_stream if rpc.startswith("Stream") else ch.unary_unary
            call = kind(f"/guardrail.harness.v1.StressHarness/{rpc}",
                        request_serializer=lambda b: b, response_deserializer=lambda b: b)
            try:
                out = call(b"", timeout=10)
                if rpc.startswith("Stream"):
                    list(out)
            except grpc.RpcError as e:
                assert e.code().name == code, (rpc, e.code(), e.details())
            else:
                assert code is None, f"{rpc} answered an empty request"
        call = ch.unary_unary("/guardrail.harness.v1.StressHarness/CancelEpisode",
                              request_serializer=lambda b: b,
                              response_deserializer=lambda b: b)
        try:
            call(b"", timeout=10)
        except grpc.RpcError as e:
            assert e.code() == grpc.StatusCode.UNIMPLEMENTED, e.code()
        else:
            raise AssertionError("a method the .proto lacks answered")


def test_server_side_failures_map_to_their_codes():
    """A package or source file missing on the SERVER is FAILED_PRECONDITION
    (not UNAVAILABLE, which a client retries and which a refused handshake
    already means); anything unexpected is INTERNAL with its type."""
    if _M is None:
        return SKIP

    class Down:
        def start_episode(self, **kw):
            raise H.HarnessUnavailable("experiments/sweep_scenarios.py is missing")

        def status(self, episode_id):
            raise RuntimeError("boom")

    with _server(harness=Down()) as srv, _client(srv, insecure=True) as c:
        code, why = _refused(c.start_episode, scenario_id="nfz-head-on")
        assert code == "FAILED_PRECONDITION" and "sweep_scenarios" in why, (code, why)
        code, why = _refused(c.status, "ep-x")
        assert code == "INTERNAL" and "RuntimeError: boom" in why, (code, why)


def test_a_client_scenario_may_not_name_a_policy_outside_the_policy_folders():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        for path in ("experiments/profiles/smoke.yaml", "../outside.yaml"):
            sc = OPEN.replace("policies/sim_demo_policy.yaml", path)
            code, why = _no_episode_added(srv.servicer.harness,
                                          lambda: c.start_episode(scenario_yaml=sc))
            assert code == "PERMISSION_DENIED", (path, code, why)


def test_a_family_of_several_cells_needs_one_named():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        code, why = _refused(c.start_episode, scenario_yaml=SWEPT)
        assert code == "INVALID_ARGUMENT" and "grpc-swept@v=3" in why, why
        ref = c.start_episode(scenario_yaml=SWEPT, cell_id="grpc-swept@v=4")
        assert ref.cell_id == "grpc-swept@v=4"


def test_profile_seed_and_override_rules():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        ref = c.start_episode(scenario_id="nfz-head-on", profile="smoke")
        assert ref.seed == 1001 and ref.profile == "smoke", ref
        code, why = _refused(c.start_episode, scenario_id="nfz-head-on", profile="smoke",
                             seed=7)
        assert code == "INVALID_ARGUMENT" and "1001" in why, why
        code, why = _refused(c.start_episode, scenario_id="nfz-head-on", profile="nightly")
        assert code == "INVALID_ARGUMENT" and "name one" in why, why
        # The profile's sweep override narrows the family to the smoke cell.
        assert _refused(c.start_episode, scenario_id="dyn-nfz-translate")[0] == "INVALID_ARGUMENT"
        ref = c.start_episode(scenario_id="dyn-nfz-translate", profile="smoke")
        assert "zone_mps=2" in ref.cell_id, ref.cell_id


def test_arms_are_chosen_by_tag_and_fly_the_same_flight():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        a = c.start_episode(scenario_id="nfz-head-on", seed=1001)
        b = c.start_episode(scenario_id="nfz-head-on", seed=1001, arm="pp1,prefix-off")
        assert b.arm == "pp1,prefix-off" and b.run_id.endswith("#pp1,prefix-off"), b
        code, why = _refused(c.start_episode, scenario_id="nfz-head-on", arm="pp9")
        assert code == "INVALID_ARGUMENT" and "canonical,prefix-on" in why, why
        ka, kb = _kpi(c.end_episode(a.episode_id)), _kpi(c.end_episode(b.episode_id))
    assert ka == kb, "a headless arm flew differently (its pilot reads no text)"


def test_a_paraphrase_arm_this_server_cannot_draw_is_refused_at_start():
    """A schema-valid scenario whose paraphrase arm this server cannot draw
    (no LLM client here; a stored set that does not hold the prompt) is
    refused by StartEpisode with FAILED_PRECONDITION and creates nothing -
    not accepted, then failed as INTERNAL at EndEpisode."""
    if _M is None:
        return SKIP
    llm = OPEN + "paraphrase: {k: 2, backend: 'llm:any-model'}\n"
    unstored = TOLD + "paraphrase: {k: 2, backend: stored, source: task_prompt}\n"
    with _server() as srv, _client(srv, insecure=True) as c:
        h = srv.servicer.harness
        for y, word in ((llm, "llm:any-model"), (unstored, "no stored paraphrase set")):
            code, why = _no_episode_added(
                h, lambda: c.start_episode(scenario_yaml=y, arm="pp1"))
            assert code == "FAILED_PRECONDITION" and word in why, (code, why)
            # The canonical arm draws nothing: the same scenario is accepted.
            assert c.start_episode(scenario_yaml=y).arm == "canonical"
        # Positive control: an arm this host can draw (the template backend on
        # the rule summary) is accepted and flown, and the bundle records the
        # draw made at StartEpisode.
        tmpl = OPEN + "paraphrase: {k: 2, backend: template}\n"
        ref = c.start_episode(scenario_yaml=tmpl, arm="pp1")
        b = c.end_episode(ref.episode_id)
    arm = json.loads((Path(b.bundle_path) / "kpi.json").read_text(encoding="utf-8"))["arm_record"]
    assert arm["arm"] == "pp1" and arm["instruction_source"] == "csp", arm
    assert arm["paraphrase"]["seed"] == ref.seed and \
        arm["paraphrase"]["backend"].startswith("template"), arm["paraphrase"]


def test_a_protected_prompt_with_a_paraphrase_arm_is_the_clients_error():
    """AerialVLA's fine-tuned prompt must never be reworded: a paraphrase arm
    on it is the scenario's mistake (INVALID_ARGUMENT), not something this
    server lacks (FAILED_PRECONDITION, as for an LLM backend it has no client
    for). Its canonical arm draws nothing and is accepted."""
    av = OPEN.replace("pilot: {type: goto, speed: 4.0}",
                      "pilot: {type: goto, speed: 4.0}\n  task_prompt: \"<image>\\nFly "
                      "forward and find the target. a red car\\nAction: \"") + \
        "paraphrase: {k: 2, backend: template, source: task_prompt}\n"
    h = _harness()
    n = len(h._episodes)
    code, why = _h_refused(h.start_episode, scenario_yaml=av, arm="pp1")
    assert code == "INVALID_ARGUMENT" and "protected" in why, (code, why)
    assert len(h._episodes) == n
    assert h.start_episode(scenario_yaml=av)["arm"] == "canonical"


# --------------------------------------------------------------------------- #
# events
# --------------------------------------------------------------------------- #

def _fly(c, yaml_text: str, hot=(), emit=(), seed: int = 0, keep_log: bool = False):
    ref = c.start_episode(scenario_yaml=yaml_text, seed=seed, keep_flight_log=keep_log)
    acks = [c.hot_apply(ref.episode_id, at, body) for at, body in hot]
    acks += [c.emit_event(ref.episode_id, at, t, p) for at, t, p in emit]
    return ref, acks, c.end_episode(ref.episode_id)


def test_a_hot_applied_zone_changes_the_flight_and_its_null_does_not():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        _, _, null = _fly(c, OPEN)
        _, acks, hit = _fly(c, OPEN, hot=[(3.0, ZONE)])
    kn, kh = _kpi(null), _kpi(hit)
    # The null: nothing in the way, nothing repaired, the goal reached.
    assert kn["repair_count"] == 0 and kn["reached_goal"] is True, kn["repair_count"]
    assert not null.applied_events
    # The hot-applied zone was in force and the Shield held it.
    a = acks[0]
    assert (a.rule_id, a.op, a.type, a.validated_generation) == \
        ("nfz-hot", "spawn", "dynamic_nfz", 1), a
    assert json.loads(a.scenario_event_json)["type"] == "spawn_polygon_fence"
    assert kh["repair_count"] > 0 and kh["reached_goal"] is False, kh["repair_count"]
    assert hit.kpi.p0_escape_rate == 0.0 and hit.kpi.null_passthrough_p0_escape_rate > 0
    ev = list(hit.applied_events)
    assert [(e.event, e.rule_id, e.generation, e.refused) for e in ev] == \
        [("spawn_polygon_fence", "nfz-hot", 1, False)], ev
    assert abs(ev[0].t - 3.0) < 1e-6
    req = json.loads((Path(hit.bundle_path) / "harness_request.json").read_text(encoding="utf-8"))
    assert req["events"][0]["rpc"] == "HotApply" and req["events"][0]["body"]["id"] == "nfz-hot"


def test_hot_apply_and_emit_event_fly_as_the_same_event_written_in_the_scenario():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        _, _, written = _fly(c, OPEN_WITH_EVENT, keep_log=True)
        _, _, hot = _fly(c, OPEN, hot=[(3.0, ZONE)], keep_log=True)
        _, _, w2 = _fly(c, OPEN_WITH_AHEAD, keep_log=True)
        _, _, emitted = _fly(c, OPEN, emit=[(2.0, "spawn_polygon_fence",
                                             {"width_m": 20, "height_m": 8,
                                              "ahead_of_vehicle_m": 12})], keep_log=True)
    log = lambda b: (Path(b.bundle_path) / "flight_log.jsonl").read_bytes()  # noqa: E731
    assert _kpi(hot) == _kpi(written) and log(hot) == log(written)
    assert _kpi(emitted) == _kpi(w2) and log(emitted) == log(w2)
    assert _kpi(w2)["repair_count"] > 0, "the written event did nothing; the comparison is empty"


def test_a_hot_applied_corridor_swap_flies_as_the_library_scenario_writes_it():
    """The third hot-applicable class, against the library's own
    corridor-swap-shifted (whose swap is written in its `events:`)."""
    if _M is None:
        return SKIP
    import yaml
    raw = dict(Library(S.LIB).raw("corridor-swap-shifted"))
    (written_ev,) = raw.pop("events")
    body = {"type": "corridor_swap", **written_ev["payload"]}
    with _server() as srv, _client(srv, insecure=True) as c:
        ref = c.start_episode(scenario_id="corridor-swap-shifted", keep_flight_log=True)
        written = c.end_episode(ref.episode_id)
        _, acks, hot = _fly(c, yaml.safe_dump(raw), hot=[(written_ev["at_sim_t"], body)],
                            keep_log=True)
        _, _, null = _fly(c, yaml.safe_dump(raw))
    assert json.loads(acks[0].scenario_event_json)["type"] == "swap_corridor"
    log = lambda b: (Path(b.bundle_path) / "flight_log.jsonl").read_bytes()  # noqa: E731
    assert _kpi(hot) == _kpi(written) and log(hot) == log(written)
    assert written.status == hot.status, (written.status, list(hot.gates_failed))
    # The null: without the swap the aircraft is never outside the corridor.
    assert _kpi(written)["time_to_safe_episodes"] >= 1
    assert _kpi(null)["time_to_safe_episodes"] == 0, _kpi(null)["time_to_safe_episodes"]


def test_capacity_limits_are_resource_exhausted():
    h = H.Harness(runs_dir=_tmpdir("cap_runs_"), manifest_base=_harness()._manifest_base)
    keep = (H.MAX_OPEN_EPISODES, H.MAX_EVENTS_PER_EPISODE)
    H.MAX_OPEN_EPISODES, H.MAX_EVENTS_PER_EPISODE = 2, 1
    try:
        a = h.start_episode(scenario_yaml=OPEN)["episode_id"]
        h.start_episode(scenario_yaml=OPEN)
        try:
            h.start_episode(scenario_yaml=OPEN)
        except H.HarnessError as e:
            assert e.code == "RESOURCE_EXHAUSTED", e
        else:
            raise AssertionError("a third open episode was accepted")
        h.emit_event(a, 1.0, "spawn_polygon_fence", {"width_m": 4, "height_m": 4})
        try:
            h.emit_event(a, 2.0, "spawn_polygon_fence", {"width_m": 4, "height_m": 4})
        except H.HarnessError as e:
            assert e.code == "RESOURCE_EXHAUSTED", e
        else:
            raise AssertionError("an event over the per-episode limit was queued")
        h.end_episode(a)                                     # frees a slot
        assert h.start_episode(scenario_yaml=OPEN)["episode_id"].startswith("ep-")
    finally:
        H.MAX_OPEN_EPISODES, H.MAX_EVENTS_PER_EPISODE = keep


def test_hot_apply_refusals_carry_the_rest_endpoints_codes_and_queue_nothing():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        ref = c.start_episode(scenario_yaml=OPEN)
        eid = ref.episode_id
        fence = dict(ZONE, type="polygon_fence")
        cases = [
            (fence, "INVALID_ARGUMENT", "locked"),
            ({"type": "time_window_switch", "target_id": "no-such-rule", "active": False},
             "NOT_FOUND", "no-such-rule"),
            (dict(ZONE, layer="regulation"), "INVALID_ARGUMENT", "layer"),
            (dict(ZONE, valid_time={"recurrence": {"days": ["Mon"]}}), "UNIMPLEMENTED",
             "valid_time"),
            ({"type": "obstacle"}, "INVALID_ARGUMENT", "cannot be hot-applied"),
            ({}, "INVALID_ARGUMENT", "body"),
        ]
        for body, want, word in cases:
            code, why = _refused(c.hot_apply, eid, 3.0, body)
            assert code == want and word in why, (body, code, why)
        assert not c.status(eid).events, "a refused hot-apply was queued"
        ok = c.hot_apply(eid, 3.0, ZONE)
        # Validated from the injected policy: the refused bodies left nothing.
        assert ok.validated_generation == 1, ok
        code, why = _refused(c.hot_apply, eid, 4.0, ZONE)
        assert code == "ALREADY_EXISTS" and "nfz-hot" in why, (code, why)
        sw = c.hot_apply(eid, 5.0, {"type": "time_window_switch", "target_id": "nfz-hot",
                                    "active": False})
        assert sw.validated_generation == 2 and json.loads(
            sw.scenario_event_json)["type"] == "deactivate_rule", sw
        assert [(q.source, q.rule_id) for q in c.status(eid).events] == \
            [("hot_apply", "nfz-hot"), ("hot_apply", sw.rule_id)]
        b = c.end_episode(eid)
    assert [(e.event, e.generation) for e in b.applied_events] == \
        [("spawn_polygon_fence", 1), ("deactivate_rule", 2)], list(b.applied_events)


def test_hot_applies_are_validated_in_flight_order():
    """run_episode applies events by at_sim_t (ties by arrival), so that is
    the order a hot-apply must be validated in. Before 9 Oct the dry run
    replayed ARRIVAL order: a switch at 5 s sent after its zone at 10 s was
    acknowledged, and the episode then failed at EndEpisode ("no rule
    nfz-hot")."""
    h = _harness()
    sw = dict(SW_OFF)
    sw.pop("id")
    # 1. A switch scheduled before its zone exists is refused as the REST
    #    endpoint refuses a switch for an unknown rule - and queues nothing.
    eid = h.start_episode(scenario_yaml=OPEN)["episode_id"]
    z = h.hot_apply(eid, 10.0, ZONE)
    code, why = _h_refused(h.hot_apply, eid, 5.0, sw,
                           what="a switch scheduled before its zone was acknowledged")
    assert code == "NOT_FOUND" and "nfz-hot" in why, (code, why)
    assert len(h.status(eid)["events"]) == 1
    # Control: the same switch after the zone is accepted and flies.
    s = h.hot_apply(eid, 12.0, sw)
    assert (z["validated_generation"], s["validated_generation"]) == (1, 2), (z, s)
    b = h.end_episode(eid)
    assert [(a["event"], a["generation"], a["refused"]) for a in b["applied_events"]] == \
        [("spawn_polygon_fence", 1, False), ("deactivate_rule", 2, False)], b["applied_events"]

    # 2. Arriving out of order, each ack's generation is the one it flies at.
    eid = h.start_episode(scenario_yaml=OPEN)["episode_id"]
    late = h.hot_apply(eid, 10.0, ZONE)
    early = h.hot_apply(eid, 5.0, FAR)
    assert early["validated_generation"] == 1, early      # it flies first
    flown = {a["rule_id"]: a["generation"] for a in h.end_episode(eid)["applied_events"]}
    assert flown == {"nfz-far": 1, "nfz-hot": 2}, flown
    assert early["validated_generation"] == flown["nfz-far"]
    assert late["validated_generation"] == 1               # as of its own ack (documented)

    # 3. A body that is fine where it lands but breaks an earlier-acked body
    #    further down the timeline is refused, naming that body.
    eid = h.start_episode(scenario_yaml=OPEN)["episode_id"]
    h.hot_apply(eid, 10.0, ZONE)
    code, why = _h_refused(h.hot_apply, eid, 5.0, ZONE)
    assert code == "FAILED_PRECONDITION" and "#0" in why and "409" in why, (code, why)
    assert len(h.status(eid)["events"]) == 1
    # ... and the episode still flies what was acknowledged.
    assert [a["rule_id"] for a in h.end_episode(eid)["applied_events"]] == ["nfz-hot"]


def test_non_default_zone_fields_fly_as_written():
    """Each field a hot-applied zone carries (margin, breach action, motion,
    altitude band) reaches the flight: the hot-applied zone flies
    byte-identical to the same zone written in the scenario. The null: the
    written zone flies differently from the plain ZONE, so the fields are
    not decoration and a translation that dropped one would be caught."""
    h = _harness()
    _, _, plain = _h_fly(h, OPEN, hot=[(3.0, ZONE)], keep_log=True)
    zm = dict(ZONE, margin_m=6, violation_action="brake",
              motion={"vx_mps": 0, "vy_mps": 0.6, "yaw_rate_dps": 0})
    zb = dict(ZONE, altitude_floor_m=20, altitude_ceiling_m=60)
    for body in (zm, zb):
        _, acks, hot = _h_fly(h, OPEN, hot=[(3.0, body)], keep_log=True)
        _, _, written = _h_fly(h, _written(OPEN, [_as_spawn(3.0, body)]), keep_log=True)
        pay = json.loads(acks[0]["scenario_event_json"])["payload"]
        for k in set(body) - {"type", "vertices"}:
            assert k in pay, (k, pay)
        assert _log(hot) == _log(written), f"{sorted(body)}: hot-applied != written"
        assert hot["kpi_json"] == written["kpi_json"]
        assert _log(written) != _log(plain), f"{sorted(body)} flies as the plain zone"


def test_a_windowed_switch_flies_as_written():
    """A time_window_switch's window reaches the flight. The clock reads
    Monday; the switch acts on Tuesdays only, so the zone stays in force -
    exactly as the same switch written in the scenario. The null: the same
    switch with no window turns the zone off and flies differently."""
    h = _harness()
    _, acks, hot = _h_fly(h, CLOCKED, hot=[(3.0, ZONE), (5.0, SW_OFF_TUE)], keep_log=True)
    ev = json.loads(acks[1]["scenario_event_json"])
    assert ev["type"] == "deactivate_rule" and ev["payload"]["window"]["days"] == ["Tue"], ev
    _, _, written = _h_fly(h, _written(CLOCKED, [
        _as_spawn(3.0, ZONE),
        {"at_sim_t": 5.0, "type": "deactivate_rule",
         "payload": {"rule_id": "nfz-hot", "id": "sw1",
                     "window": SW_OFF_TUE["valid_time"]["recurrence"]}}]), keep_log=True)
    assert _log(hot) == _log(written), "the hot-applied window flew differently"
    _, _, always = _h_fly(h, CLOCKED, hot=[(3.0, ZONE), (5.0, SW_OFF)], keep_log=True)
    assert _log(always) != _log(hot), "the window changed nothing: the comparison is empty"

    # Which way it changed: on a Monday the Tuesday-only switch leaves the zone
    # in force, so it still raises violations after 5 s; switched off with no
    # window it raises none. (Until 2026-10-12 this compared repair counts.
    # On 4418ae0 the Monday flight's 250 zone ticks after the switch were
    # Shield-flown Brake ticks, 257 repairs against 7; on the 2026-10-12
    # tree the escalation FSM hands the zone to LOITER, so those ticks are
    # autopilot ticks, which compute() does not count as repairs, and the
    # two flights log 2 repairs each. The zone ticks separate them on both.)
    def zone_ticks_after_switch(b):
        rows = [json.loads(x) for x in _log(b).decode("utf-8").splitlines()]
        return sum(1 for r in rows if r["t"] >= 5.0
                   and any(v["rule_id"] == "nfz-hot" for v in r["violations"]))
    assert zone_ticks_after_switch(hot) > 0, "the Monday flight never met the zone"
    assert zone_ticks_after_switch(always) == 0, "the switch did not turn the zone off"


def test_a_schedule_that_never_happens_is_refused_at_start():
    """The schedule checks run_episode makes, made before anything exists: a
    GPS dropout that starts after the last tick never denies anything."""
    h = _harness()
    n = len(h._episodes)
    late = OPEN + "stress: {gps_dropout_s: [40.0, 45.0]}\n"
    code, why = _h_refused(h.start_episode, scenario_yaml=late)
    assert code == "INVALID_ARGUMENT" and "never happens" in why, (code, why)
    assert len(h._episodes) == n
    # Null: the same dropout inside the run is accepted.
    ok = OPEN + "stress: {gps_dropout_s: [5.0, 8.0]}\n"
    assert h.start_episode(scenario_yaml=ok)["episode_id"].startswith("ep-")


def test_event_timing_and_schema_codes():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        eid = c.start_episode(scenario_yaml=OPEN).episode_id
        sp = {"width_m": 4, "height_m": 4}
        assert _refused(c.emit_event, eid, 30.0, "spawn_polygon_fence", sp)[0] == "OUT_OF_RANGE"
        assert _refused(c.hot_apply, eid, 30.0, ZONE)[0] == "OUT_OF_RANGE"
        assert _refused(c.emit_event, eid, -1.0, "spawn_polygon_fence", sp)[0] == \
            "INVALID_ARGUMENT"
        code, why = _refused(c.emit_event, eid, 1.0, "teleport", {})
        assert code == "INVALID_ARGUMENT", (code, why)
        code, why = _refused(c.emit_event, eid, 1.0, "spawn_polygon_fence",
                             dict(sp, colour="red"))
        assert code == "INVALID_ARGUMENT" and "colour" in why, why
        code, why = _refused(c.emit_event, eid, 1.0, "activate_rule",
                             {"rule_id": "nfz-square", "window": {"days": ["Mon"]}})
        assert code == "FAILED_PRECONDITION" and "clock_start" in why, why
        assert not c.status(eid).events
        ack = c.emit_event(eid, 29.9, "spawn_polygon_fence", sp)     # the last tick fires
        assert ack.queue_index == 0


def test_an_episode_the_runner_refuses_in_flight_is_failed_precondition():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        eid = c.start_episode(scenario_yaml=OPEN).episode_id
        c.emit_event(eid, 1.0, "translate_polygon_fence", {"id": "no-zone", "dx_m": 2.0})
        code, why = _refused(c.end_episode, eid)
        assert code == "FAILED_PRECONDITION" and "no-zone" in why, (code, why)
        st = c.status(eid)
        assert _M.pb.EpisodeState.Name(st.state) == "FAILED" and "no-zone" in st.error, st
        assert _refused(c.end_episode, eid)[0] == "FAILED_PRECONDITION"
        assert not st.bundle_path


def test_end_episode_is_idempotent_and_closes_the_episode():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        eid = c.start_episode(scenario_yaml=OPEN).episode_id
        b1, b2 = c.end_episode(eid), c.end_episode(eid)
        assert b1 == b2
        assert _refused(c.hot_apply, eid, 3.0, ZONE)[0] == "FAILED_PRECONDITION"
        assert _refused(c.emit_event, eid, 3.0, "spawn_polygon_fence",
                        {"width_m": 4, "height_m": 4})[0] == "FAILED_PRECONDITION"


def test_a_second_end_episode_waits_for_the_flight_in_progress():
    """A client whose deadline expired while its EndEpisode queued, and which
    retries, must get the episode - not FAILED_PRECONDITION "being flown by
    another call" (what it got before 9 Oct). The flight is held open until
    the second call is waiting, so the race is not left to timing."""
    for scenario, emit, ok in ((OPEN, (), True),
                               (OPEN, ((1.0, "translate_polygon_fence",
                                        {"id": "no-zone", "dx_m": 2.0}),), False)):
        h = _own_harness("wait_runs_")
        entered, release = threading.Event(), threading.Event()
        real = h._fly

        def held(e, queued, real=real, entered=entered, release=release):
            entered.set()
            assert release.wait(60), "never released"
            return real(e, queued)

        h._fly = held
        eid = h.start_episode(scenario_yaml=scenario)["episode_id"]
        for at, t, p in emit:
            h.emit_event(eid, at, t, p)
        out: dict = {}

        def call(k, out=out, h=h, eid=eid):
            try:
                out[k] = h.end_episode(eid)
            except H.HarnessError as err:
                out[k] = err

        first = threading.Thread(target=call, args=("first",), daemon=True)
        first.start()
        assert entered.wait(60)
        second = threading.Thread(target=call, args=("second",), daemon=True)
        second.start()
        e = h._episodes[eid]
        deadline = time.monotonic() + 30
        while getattr(e, "waiters", 0) < 1 and second.is_alive() and time.monotonic() < deadline:
            time.sleep(0.01)
        release.set()
        first.join(60)
        second.join(60)
        if ok:
            assert isinstance(out["first"], dict), out["first"]
            assert isinstance(out["second"], dict), \
                f"the second EndEpisode was refused while the first flew: {out['second']}"
            assert out["second"] == out["first"]
        else:
            # A flight that fails fails both calls, with the runner's reason.
            for k in ("first", "second"):
                assert isinstance(out[k], H.HarnessError) and \
                    out[k].code == "FAILED_PRECONDITION" and "no-zone" in out[k].message, out[k]
        assert e.waiters == 0 and h.status(eid)["state"] == ("ENDED" if ok else "FAILED")


def test_an_idle_armed_episode_expires_and_frees_its_slot():
    """StartEpisode with no EndEpisode (a client that gave up, or the client
    tool after a refused HotApply) must not hold a slot for ever: before
    9 Oct, MAX_OPEN_EPISODES such episodes wedged the server until restart."""
    keep = H.MAX_OPEN_EPISODES
    H.MAX_OPEN_EPISODES = 2
    try:
        h = _own_harness("ttl_runs_", armed_idle_ttl_s=0)
        a = h.start_episode(scenario_yaml=OPEN)["episode_id"]
        b = h.start_episode(scenario_yaml=OPEN)["episode_id"]
        c = h.start_episode(scenario_yaml=OPEN)["episode_id"]   # a and b expire first
        st = h.status(a)
        assert st["state"] == "FAILED" and "expired" in st["error"], st
        code, why = _h_refused(h.emit_event, b, 1.0, "spawn_polygon_fence",
                               {"width_m": 4, "height_m": 4})
        assert code == "FAILED_PRECONDITION" and "expired" in why, (code, why)
        code, why = _h_refused(h.end_episode, b)
        assert code == "FAILED_PRECONDITION" and "expired" in why, (code, why)
        assert c.startswith("ep-")
        # The null: with the default TTL the same three starts are refused.
        h2 = _own_harness("ttl_null_")
        h2.start_episode(scenario_yaml=OPEN)
        h2.start_episode(scenario_yaml=OPEN)
        assert _h_refused(h2.start_episode, scenario_yaml=OPEN)[0] == "RESOURCE_EXHAUSTED"
    finally:
        H.MAX_OPEN_EPISODES = keep
    # An accepted event restarts the idle clock: an hour is idle time, not age.
    h3 = _own_harness("ttl_touch_", armed_idle_ttl_s=3600)
    eid = h3.start_episode(scenario_yaml=OPEN)["episode_id"]
    e = h3._episodes[eid]
    e.idle_since -= 3000
    h3.emit_event(eid, 1.0, "spawn_polygon_fence", {"width_m": 4, "height_m": 4})
    e.idle_since -= 3000                     # 3000 s idle since that event: not expired
    assert h3.status(eid)["state"] == "ARMED"
    e.idle_since -= 700                      # now 3700 s idle
    assert h3.status(eid)["state"] == "FAILED"
    try:
        H.Harness(runs_dir=_tmpdir("ttl_bad_"), manifest_base=h3._manifest_base,
                  armed_idle_ttl_s=-1)
    except ValueError:
        pass
    else:
        raise AssertionError("a negative TTL was accepted")


def test_finished_episodes_beyond_the_cap_are_forgotten_oldest_first():
    keep = H.MAX_KEPT_FINISHED
    H.MAX_KEPT_FINISHED = 1
    try:
        h = _own_harness("evict_runs_")
        a = _h_fly(h, OPEN)[2]
        b = _h_fly(h, OPEN)[2]
        assert _h_refused(h.status, a["episode_id"])[0] == "NOT_FOUND"
        assert h.status(b["episode_id"])["state"] == "ENDED"
        # Forgotten by the server, not deleted: the bundle stays on disk.
        assert (Path(a["bundle_path"]) / "kpi.json").is_file()
    finally:
        H.MAX_KEPT_FINISHED = keep


# --------------------------------------------------------------------------- #
# determinism, nulls, unset figures
# --------------------------------------------------------------------------- #

def test_determinism_by_seed_with_its_null():
    if _M is None:
        return SKIP
    with _server() as srv, _client(srv, insecure=True) as c:
        runs = [_fly(c, JITTER, seed=s, keep_log=True)[2] for s in (5, 5, 6)]
    logs = [(Path(b.bundle_path) / "flight_log.jsonl").read_bytes() for b in runs]
    assert logs[0] == logs[1] and runs[0].kpi_json == runs[1].kpi_json, \
        "the same seed flew two different episodes"
    # The null: a harness that ignored the seed would pass the line above too.
    assert logs[0] != logs[2], "seed 6 flew seed 5's episode: the seed is not applied"
    assert hashlib.sha256(logs[0]).hexdigest() != hashlib.sha256(logs[2]).hexdigest()
    man = [json.loads((Path(b.bundle_path) / "manifest.json").read_text(encoding="utf-8"))
           for b in runs]
    assert [m["random_seed"] for m in man] == [5, 5, 6]


def test_the_shield_off_control_scores_escapes():
    """The KPI path can say no: the unshielded run into the zone escapes."""
    _, _, b = _h_fly(_harness(), scenario_id="nfz-head-on-control", seed=1001)
    k = b["kpi"]
    assert k["shield"] == "off" and k["p0_escape_rate"] > 0.1, k
    # Off, the Shield IS the passthrough null: the two must coincide.
    assert k["p0_escape_rate"] == k["null_passthrough_p0_escape_rate"], k
    assert b["status"] == "pass", "its gates REQUIRE escapes; a clean control would fail"
    # Its logged repairs were never flown: unset with the reason, not 0.
    assert "repair_success_rate" not in k and k["unset"]["repair_success_rate"] == "shield_off"
    assert "mean_repair_magnitude_mps" not in k and "mean_repair_magnitude_mps" in k["unset"]


def test_unset_figures_are_unset_with_a_reason_never_zero():
    _, _, b = _h_fly(_harness(), OPEN)
    k = b["kpi"]
    # OPEN carries no expected_failsafe label and is never unsafe.
    for f in ("fail_safe_correctness", "null_never_trigger", "null_always_trigger"):
        assert f not in k and "label" in k["unset"][f], (f, k)
    assert "mean_time_to_safe_s" not in k and k["unset"]["mean_time_to_safe_s"] == "never_unsafe"
    assert "p0_escape_rate" in k and "p0_escape_rate" not in k["unset"]
    assert set(k["unset"]) | (set(H.KPI_ROW_FIELDS) & set(k)) == set(H.KPI_ROW_FIELDS)
    assert not set(k["unset"]) & set(k), "a figure both set and unset"
    # The mapping itself: a rollup with no numbers sets nothing.
    row = H.kpi_row({"episodes": 0, "time_to_safe_status": "no_episodes"}, "f", "c", "on")
    assert set(row["unset"]) == set(H.KPI_ROW_FIELDS)
    assert row["unset"]["mean_time_to_safe_s"] == "no_episodes"
    assert not set(H.KPI_ROW_FIELDS) & set(row)


# Where guardrail.kpi.rollup() keeps each KpiRow figure - written out HERE,
# not read from the service's own table, so a wrong path in the service
# cannot also be the test's expectation.
ROLLUP_PATHS = {
    "p0_escape_rate": ("p0_escape_rate",),
    "null_passthrough_p0_escape_rate": ("null_passthrough_p0_escape_rate",),
    "fail_safe_correctness": ("failsafe_labelled_episodes", "failsafe_trigger_correctness"),
    "null_never_trigger": ("failsafe_labelled_episodes", "null_never_trigger"),
    "null_always_trigger": ("failsafe_labelled_episodes", "null_always_trigger"),
    "mission_success_rate": ("mission_success_rate",),
    "null_hover_mission_success_rate": ("null_hover_mission_success_rate",),
    "mean_repair_count": ("mean_repair_count_per_episode",),
    "mean_repair_magnitude_mps": ("mean_repair_magnitude_mps",),
    "repair_success_rate": ("repair_success_rate",),
    "null_always_brake_repair_success_rate": ("null_always_brake_repair_success_rate",),
    "mean_time_to_safe_s": ("mean_time_to_safe_s",),
}


def test_kpi_row_is_the_rollup_field_by_field():
    """The typed KPI row is the unit's main output: every figure must be the
    rollup's own number or, where the rollup has none, absent and named in
    `unset`. Three episodes between them set every figure: nfz-head-on
    (labelled; fail-safe 1.0 beside never-trigger 1.0 and always-trigger
    0.0), its Shield-off control (a time to safe) and OPEN (a mission that
    succeeds beside a hover null that does not)."""
    assert set(ROLLUP_PATHS) == set(H.KPI_ROW_FIELDS)
    h = _harness()
    set_by = {}
    for start in ({"scenario_id": "nfz-head-on", "seed": 1001},
                  {"scenario_id": "nfz-head-on-control", "seed": 1001},
                  {"scenario_yaml": OPEN}):
        b = h.end_episode(h.start_episode(**start)["episode_id"])
        roll, row = json.loads(b["rollup_json"]), b["kpi"]
        for f, path in ROLLUP_PATHS.items():
            v = roll
            for part in path:
                v = v.get(part) if isinstance(v, dict) else None
            if v is None:
                assert f not in row and f in row["unset"], (start, f, row)
            else:
                assert f in row and row[f] == float(v) and f not in row["unset"], \
                    (start, f, v, row.get(f), row["unset"].get(f))
                set_by.setdefault(f, start.get("scenario_id", "OPEN"))
    assert set(set_by) == set(ROLLUP_PATHS), f"never set: {set(ROLLUP_PATHS) - set(set_by)}"


def test_the_harness_judges_with_the_sweeps_own_verdict_function():
    """REGRESSION (2026-10-11): the harness carried a copy of the sweep's
    verdict, so the two could drift. It now calls
    sweep_scenarios.episode_verdict; a stand-in put there must decide the
    status the harness reports, with the arguments the sweep would pass."""
    assert not hasattr(H, "episode_verdict"), "the harness still carries its own verdict"
    h = _harness()
    seen = []
    real = S.episode_verdict

    def stand_in(spec, res, params, seed):
        seen.append((spec.scenario_id, dict(params), seed, res.get("mission_outcome")))
        return "known_failure", ["stand-in gate"], [], "stand-in reason"

    S.episode_verdict = stand_in
    try:
        ref, _, b = _h_fly(h, OPEN, seed=7)
    finally:
        S.episode_verdict = real
    kd = _bundle_kpi(b)
    assert (b["status"], h.status(ref["episode_id"])["status"], kd["status"]) == \
        ("known_failure",) * 3, (b["status"], kd["status"])
    assert b["gates_failed"] == ["stand-in gate"], b["gates_failed"]
    assert kd["known_failure_reason"] == "stand-in reason", kd
    # Called once, on the flown episode's own scenario, cell, seed and score.
    assert seen == [("grpc-open-flight", kd["params"], 7, kd["kpi"]["mission_outcome"])], seen
    # With the real function back, the same scenario passes again (the null).
    _, _, b2 = _h_fly(h, OPEN)
    assert b2["status"] == "pass", (b2["status"], b2["gates_failed"])


def test_the_verdict_gives_every_status_the_sweep_gives():
    """The verdict is the sweep's episode_verdict; each status is reached
    from OPEN (which reaches its goal, gates passing) by changing only the
    labels, and lands in EndEpisode, GetStatus and the bundle alike."""
    h = _harness()
    cases = [
        ("", "pass"),
        ("expected_outcome: fail\n", "fail"),                     # a label mismatch fails
        ("expect: known_failure\nexpected_outcome: fail\n", "known_failure"),
        ("expect: known_failure\n", "unexpected_pass"),           # the defect is fixed
        ("expect: tracked\n", "tracked"),
        ("expect: tracked\nexpected_outcome: fail\n", "tracked"),  # reported, not failed
    ]
    for extra, want in cases:
        ref, _, b = _h_fly(h, OPEN + extra)
        kd = _bundle_kpi(b)
        assert (b["status"], h.status(ref["episode_id"])["status"], kd["status"]) == \
            (want, want, want), (extra, b["status"], b["gates_failed"])
        if want in ("fail", "known_failure"):
            assert any("outcome" in g for g in b["gates_failed"]), b["gates_failed"]
        if want == "unexpected_pass":
            assert "known_failure" in b["gates_failed"][0], b["gates_failed"]
        if "known_failure" in extra:
            assert kd["known_failure_reason"], kd
        if want == "tracked" and "expected_outcome" in extra:
            assert kd["labels_failed"] and not b["gates_failed"], kd
        # Not asked for, so not written.
        assert not (Path(b["bundle_path"]) / "flight_log.jsonl").exists()


def test_the_verdict_matches_the_sweeps_own_run():
    """experiments/sweep_scenarios.py main() on a few families of the smoke
    profile, then every one of its episodes again through the harness: the
    same status, gates and label failures, episode by episode."""
    want = ["three-simultaneous-violations", "gps-noise-beyond-margin",
            "standoff-reclassified", "altitude-floor-recovery", "nfz-head-on-control"]
    have = Library(S.LIB).by_id()
    fams = [f for f in want if f in have]
    assert len(fams) >= 3, f"the library lost these families: {set(want) - set(fams)}"
    tmp = _tmpdir("verdict_sweep_")
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        rc = S.main(["--only", *fams, "--profile", "smoke", "--out", str(tmp / "sweep.json"),
                     "--runs", str(tmp / "runs"), "--no-bundles"])
    assert rc in (0, 1), out.getvalue()[-2000:]
    rows = json.loads((tmp / "sweep.json").read_text(encoding="utf-8"))["results"]
    assert len(rows) >= len(fams), rows
    h = _harness()
    seen = set()
    for r in rows:
        assert r["status"] in ("pass", "fail", "tracked", "known_failure", "unexpected_pass"), r
        arm = r["id"].split("#", 1)[1] if "#" in r["id"] else ""
        ref = h.start_episode(scenario_id=r["scenario_id"], cell_id=r["id_cell"],
                              seed=r["seed"], profile="smoke", arm=arm)
        b = h.end_episode(ref["episode_id"])
        kd = _bundle_kpi(b)
        got = (b["status"], b["gates_failed"], kd.get("labels_failed") or [],
               kd.get("known_failure_reason"))
        assert got == (r["status"], r["gates_failed"], r.get("labels_failed") or [],
                       r.get("known_failure_reason")), (r["id"], got, r["status"])
        seen.add(r["status"])
    assert len(seen) >= 2, f"every episode had status {seen}: the comparison is thin"


def test_an_episode_whose_policy_changed_since_start_is_refused():
    """The policy file is hashed at StartEpisode; if it reads differently when
    the episode flies, the bundle would record a policy that did not fly."""
    h = _own_harness("policy_changed_")
    real = S.run_episode
    h._S = types.SimpleNamespace(**vars(S))
    h._S.run_episode = lambda *a, **k: dataclasses.replace(
        real(*a, **k), policy_hash_start="sha256:" + "0" * 64)
    eid = h.start_episode(scenario_yaml=OPEN)["episode_id"]
    code, why = _h_refused(h.end_episode, eid)
    assert code == "FAILED_PRECONDITION" and "policy file changed" in why, (code, why)
    assert h.status(eid)["state"] == "FAILED" and not h.status(eid)["bundle_path"]
    # Null: the real runner (same harness, same scenario) flies it.
    h._S = S
    assert _h_fly(h, OPEN)[2]["status"] == "pass"


# --------------------------------------------------------------------------- #
# the PKI
# --------------------------------------------------------------------------- #

def test_the_pki_is_one_ca_a_server_and_a_client_on_p256():
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives.asymmetric import ec
        from cryptography.x509.oid import ExtendedKeyUsageOID as EKU
    except ImportError:
        return SKIP
    d = _tmpdir("pki_check_")
    files = H.generate_pki(d)
    load = lambda p: x509.load_pem_x509_certificate(Path(p).read_bytes())   # noqa: E731
    ca, srv, cli = (load(files["ca_cert"]), load(files["harness-server_cert"]),
                    load(files["harness-client_cert"]))
    assert ca.extensions.get_extension_for_class(x509.BasicConstraints).value.ca
    assert ca.extensions.get_extension_for_class(x509.KeyUsage).value.key_cert_sign
    for leaf, usage in ((srv, EKU.SERVER_AUTH), (cli, EKU.CLIENT_AUTH)):
        assert not leaf.extensions.get_extension_for_class(x509.BasicConstraints).value.ca
        assert list(leaf.extensions.get_extension_for_class(x509.ExtendedKeyUsage).value) \
            == [usage]
        leaf.verify_directly_issued_by(ca)
        assert isinstance(leaf.public_key().curve, ec.SECP256R1)
    san = srv.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
    assert san.get_values_for_type(x509.DNSName) == ["localhost"]
    assert [str(a) for a in san.get_values_for_type(x509.IPAddress)] == ["127.0.0.1"]
    assert isinstance(ca.public_key().curve, ec.SECP256R1)
    # A rogue CA's leaf does not verify against this CA (the refusal's basis).
    rogue = H.generate_pki(_tmpdir("pki_rogue_"))
    try:
        load(rogue["harness-client_cert"]).verify_directly_issued_by(ca)
    except Exception:                                            # noqa: BLE001
        pass
    else:
        raise AssertionError("a rogue CA's certificate verified against the lab CA")
    # Replacing a CA is refused unless forced, and forcing does replace it.
    fp = H.fingerprint(files["ca_cert"])
    try:
        H.generate_pki(d)
    except FileExistsError as e:
        assert "invalidates" in str(e)
    else:
        raise AssertionError("an existing root CA was silently replaced")
    assert H.fingerprint(files["ca_cert"]) == fp
    H.generate_pki(d, overwrite=True)
    assert H.fingerprint(files["ca_cert"]) != fp
    assert len(fp.split(":")) == 32


def test_issue_cert_adds_a_server_certificate_for_another_host():
    try:
        from cryptography import x509
    except ImportError:
        return SKIP
    d = _tmpdir("pki_issue_")
    H.generate_pki(d)
    cert, key = H.issue_cert(d, "orin-harness", server_hosts=["192.168.50.20", "orin.local"],
                             client=False)
    c = x509.load_pem_x509_certificate(cert.read_bytes())
    san = c.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
    assert [str(a) for a in san.get_values_for_type(x509.IPAddress)] == ["192.168.50.20"]
    assert san.get_values_for_type(x509.DNSName) == ["orin.local"]
    assert key.read_bytes().startswith(b"-----BEGIN PRIVATE KEY-----")
    for bad in ("../escape", "a b"):
        try:
            H.issue_cert(d, bad)
        except ValueError:
            continue
        raise AssertionError(f"certificate name {bad!r} accepted")


def test_the_default_pki_folder_is_outside_the_checkout_and_unsynced():
    """The CA key is unencrypted and is meant to seed SROS 2 keystores. Until
    9 Oct its default folder was demo/out/harness_pki/ in the checkout:
    gitignored, but this checkout sits in OneDrive, which uploads it."""
    d = H.DEFAULT_PKI_DIR.resolve()
    assert not d.is_relative_to(ROOT.resolve()), f"{d} is inside the checkout"
    assert H.synced_folder(d) is None, f"{d} is in {H.synced_folder(d)}"
    base = _tmpdir("sync_names_")
    for name in ("OneDrive", "OneDrive - Taipei Tech", "Dropbox", "Google Drive"):
        assert H.synced_folder(base / name / "pki"), name
    # The null: an ordinary folder is not synced (else every path would be).
    assert H.synced_folder(base / "pki") is None
    assert H.synced_folder(base / "OneDriveNotes" / "pki") is None
    od = os.environ.get("OneDrive")
    if od and ROOT.resolve().is_relative_to(Path(od).resolve()):
        # On the development desktop the checkout itself is synced.
        assert H.synced_folder(ROOT / "demo" / "out" / "harness_pki")


def test_gen_pki_refuses_a_synced_folder_unless_told():
    try:
        import cryptography                                   # noqa: F401
    except ImportError:
        return SKIP
    base = _tmpdir("sync_pki_")
    target = base / "OneDrive" / "harness_pki"
    try:
        H.generate_pki(target)
    except ValueError as e:
        assert "synced" in str(e) and "allow_synced" in str(e), e
    else:
        raise AssertionError("keys were written into a synced folder")
    assert not target.exists(), "a refused gen-pki left files behind"
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        rc = H.main(["gen-pki", "--out", str(base / "Dropbox" / "pki")])
    assert rc == 2 and "--allow-synced" in err.getvalue() and not (base / "Dropbox").exists()
    # Told on purpose, it writes; issuing there is refused the same way.
    files = H.generate_pki(target, allow_synced=True)
    assert files["ca_key"].is_file()
    try:
        H.issue_cert(target, "another-client")
    except ValueError:
        pass
    else:
        raise AssertionError("issue_cert wrote a key into a synced folder")
    assert H.issue_cert(target, "another-client", allow_synced=True)[1].is_file()


# --------------------------------------------------------------------------- #
# the client tool
# --------------------------------------------------------------------------- #

def _tool(*argv) -> tuple[int, str, str]:
    import harness_client
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = harness_client.main(list(argv))
    return rc, out.getvalue(), err.getvalue()


def test_the_client_tool_prints_each_score_beside_its_null():
    if not _TLS:
        return SKIP
    body = _tmpdir("tool_") / "zone.json"
    body.write_text(json.dumps(ZONE), encoding="utf-8")
    scen = body.parent / "open.yaml"
    scen.write_text(OPEN, encoding="utf-8")
    with _server(pki_dir=_pki()) as srv:
        with _client(srv) as c:                       # what the tool must print
            ref = c.start_episode(scenario_id="nfz-head-on", seed=1001)
            want = c.end_episode(ref.episode_id).kpi
        rc, out, err = _tool("--target", srv.address, "--pki", str(_pki()), "run",
                             "--scenario", "nfz-head-on", "--seed", "1001")
        assert rc in (0, 1) and not err, (rc, err)
        null = f"null passthrough {want.null_passthrough_p0_escape_rate:.6g}"
        assert "P0 escape rate" in out and null in out, (null, out)
        assert "unset (never_unsafe)" in out and "NOT KPI-grade" in out, out
        rc, out, err = _tool("--target", srv.address, "--pki", str(_pki()), "run",
                             "--scenario-file", str(scen), "--hot-apply", "3.0", str(body))
        assert rc in (0, 1) and "spawn_polygon_fence nfz-hot: generation 1" in out, (rc, out, err)
        rc, out, err = _tool("--target", srv.address, "--pki", str(_pki()), "run",
                             "--scenario", "no-such-family")
        assert rc == 2 and "REFUSED NOT_FOUND" in err, (rc, err)
        assert "ARMED" not in err, err                # nothing was started
        # A refused HotApply after StartEpisode: the tool names the episode it
        # leaves ARMED (it expires on the server; nothing flew).
        locked = body.parent / "locked.json"
        locked.write_text(json.dumps(dict(ZONE, type="polygon_fence")), encoding="utf-8")
        rc, out, err = _tool("--target", srv.address, "--pki", str(_pki()), "run",
                             "--scenario-file", str(scen), "--hot-apply", "3.0", str(locked))
        m = re.search(r"(ep-[0-9a-f]{12}) was left ARMED", err)
        assert rc == 2 and "REFUSED INVALID_ARGUMENT" in err and m, (rc, err)
        assert srv.servicer.harness.status(m.group(1))["state"] == "ARMED"
        rc, out, err = _tool("--target", srv.address, "--pki", str(_pki()), "--name",
                             "other-client", "status", "ep-x")
        assert rc == 2 and "REFUSED PERMISSION_DENIED" in err, (rc, err)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs grpcio, and cryptography for mTLS)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}  {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}  {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
