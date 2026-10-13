"""The ROS 2 Shield node's new inputs, driven through the real ShieldNode code
on a stand-in ROS stack (no rclpy, no MAVROS, no autopilot).

    python tests/test_railinputs_node.py      (no pytest needed)

sitl/ros2_shield_node.py defines ShieldNode only where rclpy and mavros_msgs
import, so until now its tick was checked by reading the source. Here a
minimal stand-in for rclpy and the message packages lets the node be built
from its own parser and ticked by hand on a virtual monotonic clock. It
proves code paths and records, not flight dynamics: the aircraft does not
move, and nothing reaches an autopilot.

  * the stale-action watchdog: a quiet /vla/action_4d holds the aircraft
    (LOITER through the adapter, flown as BRAKE) after one tick of the
    Shield's answer to a stop, the FSM does not step while held, and the
    next action returns it to GUIDED;
  * the tick latency each row records (receipt -> publication);
  * the rule clock reaching the Shield and the episode record;
  * the REST hot-apply listener on the flying Shield, over real HTTP (needs
    fastapi + uvicorn + httpx: the 3.11 env; SKIP elsewhere), and its mutual
    TLS allow-list (also cryptography).

Shown failing first against a pristine HEAD 4418ae0 copy (git archive; the
same stand-in, unknown flags dropped by parse_known_args): the stale stream
was flown on (no action_stale event, every row "pass" with the last action),
rows carry no e2e_ms, the Shield has no clock, and no listener answers.
"""
from __future__ import annotations

import importlib.util
import json
import socket
import sys
import tempfile
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sitl"))

from guardrail.models import State                                    # noqa: E402

POLICY = ROOT / "policies" / "sim_demo_policy.yaml"
SKIP = "SKIP"


# --------------------------------------------------------------------------- #
# A stand-in for the ROS 2 packages the node imports
# --------------------------------------------------------------------------- #

class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Srv:
    Request = _Msg


class _Pub:
    def __init__(self, topic):
        self.topic, self.sent = topic, []

    def publish(self, msg):
        self.sent.append(msg)


class _Client:
    def service_is_ready(self):
        return False

    def wait_for_service(self, timeout_sec=None):
        return False

    def call_async(self, req):
        return None


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    warning = warn

    def error(self, m):
        self.lines.append(("error", m))


class _Node:
    def __init__(self, name):
        self._logger = _Logger()
        self.pubs: dict[str, _Pub] = {}
        self.timers = []

    def get_logger(self):
        return self._logger

    def create_subscription(self, *a, **kw):
        return None

    def create_publisher(self, msg_type, topic, qos):
        self.pubs[topic] = _Pub(topic)
        return self.pubs[topic]

    def create_client(self, *a, **kw):
        return _Client()

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def count_subscribers(self, topic):
        return 1                                   # an adapter is listening

    def destroy_node(self):
        pass


def _fake_ros() -> dict:
    m = {n: types.ModuleType(n) for n in (
        "rclpy", "rclpy.node", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg",
        "std_msgs", "std_msgs.msg", "mavros_msgs", "mavros_msgs.msg",
        "mavros_msgs.srv", "rcl_interfaces", "rcl_interfaces.msg",
        "rcl_interfaces.srv")}
    m["rclpy"].spin_once = lambda *a, **kw: None
    m["rclpy"].spin_until_future_complete = lambda *a, **kw: None
    m["rclpy"].ok = lambda: False
    m["rclpy.node"].Node = _Node
    q = m["rclpy.qos"]
    q.QoSProfile = _Msg
    q.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    q.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    q.qos_profile_sensor_data = None
    for mod, names in (("geometry_msgs.msg", ("PoseStamped",)),
                       ("std_msgs.msg", ("Float32MultiArray", "String")),
                       ("mavros_msgs.msg", ("ExtendedState", "State", "StatusText",
                                            "HomePosition", "PositionTarget",
                                            "Waypoint")),
                       ("rcl_interfaces.msg", ("Parameter", "ParameterValue"))):
        for n in names:
            setattr(m[mod], n, type(n, (_Msg,), {}))
    m["rcl_interfaces.msg"].ParameterType = types.SimpleNamespace(PARAMETER_DOUBLE=3)
    for mod, names in (("mavros_msgs.srv", ("CommandBool", "CommandTOL", "SetMode",
                                            "StreamRate", "VehicleInfoGet",
                                            "WaypointPull", "WaypointPush")),
                       ("rcl_interfaces.srv", ("GetParameters", "SetParameters"))):
        for n in names:
            setattr(m[mod], n, type(n, (_Srv,), {}))
    return m


class _Clock:
    """time.monotonic / monotonic_ns for the node module, moved by hand."""

    def __init__(self, t=1000.0):
        self.t = float(t)

    def monotonic(self):
        return self.t

    def monotonic_ns(self):
        return int(round(self.t * 1e9))

    def time(self):
        return 1.76e9 + self.t

    def sleep(self, s):
        self.t += float(s)


def _node_module():
    fakes = _fake_ros()
    saved = {k: sys.modules.get(k) for k in fakes}
    sys.modules.update(fakes)
    try:
        spec = importlib.util.spec_from_file_location(
            "ros2_shield_node_standin", ROOT / "sitl" / "ros2_shield_node.py")
        N = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(N)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    assert N.HAVE_ROS, "the stand-in ROS stack did not import"
    return N


def _build(argv: list[str]):
    """(module, node, virtual clock, out dir): the node built from its own
    parser, on the pad at 15 m, mission started."""
    N = _node_module()
    clk = _Clock()
    N.time = clk
    args, _ = N.make_parser().parse_known_args(
        ["--policy", str(POLICY), "--dev-yaml"] + argv)
    out = Path(tempfile.mkdtemp(prefix="ros2node_"))
    node = N.ShieldNode(args, out)
    node.state = State(x=0.0, y=0.0, up=15.0, yaw_deg=0.0)
    return N, node, clk, out


def _act(N, node, vx=2.0):
    node._on_action(N.Float32MultiArray(data=[vx, 0.0, 0.0, 0.0]))


def _events(out: Path) -> list[dict]:
    return [json.loads(x) for x in (out / "events.jsonl").read_text(
        encoding="utf-8").splitlines()]


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


# --------------------------------------------------------------------------- #
# The tests
# --------------------------------------------------------------------------- #

def test_a_quiet_action_stream_is_held_and_the_next_action_resumes_it():
    # 0.55 s keeps every tick off the threshold (no float ties at 0.5).
    N, node, clk, out = _build(["--action-timeout-s", "0.55"])
    node.start_mission()
    for _ in range(4):                                  # 10 Hz pilot
        _act(N, node)
        clk.t += 0.1
        node._tick()
    for _ in range(8):                                  # the pilot goes quiet
        clk.t += 0.1
        node._tick()
    _act(N, node)
    clk.t += 0.1
    node._tick()
    rows = node.rows
    sp = [r["setpoint"] for r in rows]
    stale = [r["action_stale"] for r in rows]
    assert stale == [False] * 8 + [True] * 4 + [False], stale
    # One tick of the Shield's answer to a stop, then the autopilot holds.
    assert sp[8] == "pass" and sp[9:12] == ["none"] * 3 and sp[12] == "pass", sp
    assert rows[8]["raw_body"]["vx"] == 0.0, "the stale action itself was flown"
    assert all(r["flown"] is False for r in rows[9:12])
    # The FSM does not step while held.
    assert all("fsm_state_after" not in r for r in rows[9:12])
    assert "fsm_state_after" in rows[12]
    modes = [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent]
    assert modes == ["LOITER", "GUIDED"], modes
    kinds = [e["kind"] for e in _events(out)]
    assert "action_stale" in kinds and "action_resumed" in kinds, kinds
    w = node.watchdog.summary(lambda v: v - node.mission_t0)
    assert w["n_spells"] == 1 and w["spells"][0]["t_end"] is not None, w


def test_no_action_at_all_is_held_once_the_timeout_has_passed():
    N, node, clk, out = _build([])                      # default timeout 0.5 s
    assert node.watchdog.timeout_s == N.ACTION_TIMEOUT_S == 0.5
    node.start_mission()
    clk.t += 0.05                                       # ticks between the tenths
    for _ in range(7):
        clk.t += 0.1
        node._tick()
    # Ticks 0.15 .. 0.75 s after mission start: quiet for > 0.5 s from 0.55 s.
    assert [r["action_stale"] for r in node.rows] == [False] * 4 + [True] * 3
    assert [r["setpoint"] for r in node.rows][4:] == ["pass", "none", "none"]
    assert [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent] == ["LOITER"]


def test_each_row_records_its_latency_from_receipt_to_publication():
    N, node, clk, out = _build([])
    node.start_mission()
    _act(N, node)                                       # received at T
    clk.t += 0.03
    node._tick()                                        # published at T + 30 ms
    clk.t += 0.1
    node._tick()                                        # the same action again
    r0, r1 = node.rows
    assert r0["e2e_ms"] == 30.0 and r0["action_age_ms"] == 30.0, r0
    assert r0["tick_ms"] == 0.0                         # the virtual clock stood still
    assert r1["e2e_ms"] is None and r1["action_age_ms"] == 130.0, r1
    s = N.latency_summary(node.rows)
    assert s["e2e_ms"]["n"] == 1 and s["e2e_ms"]["p99"] == 30.0
    assert s["budget_ms"] == 100.0 and s["action_age_ms"]["over_budget"] == 1


def test_the_shield_and_the_episode_read_the_nodes_rule_clock():
    N, node, clk, out = _build(["--clock-start", "2026-10-12T08:30:00"])
    assert node.shield._now is node.clock
    start = _events(out)[0]
    assert start["rule_clock"] == node.clock_record and start["rule_clock"]["kind"] == "scripted"
    assert node.rec.generations[0]["rule_clock_reading"] == "2026-10-12T08:30:00"
    node.start_mission()
    clk.t += 12.0
    assert node.clock().isoformat() == "2026-10-12T08:30:12"
    _, wall_node, _, _ = _build([])
    assert wall_node.clock_record["kind"] == "wall"


def test_a_hot_apply_over_rest_reaches_the_flying_shield_and_its_records():
    try:
        import fastapi  # noqa: F401
        import httpx
        import uvicorn  # noqa: F401
    except ImportError:
        return SKIP
    port = _free_port()
    N, node, clk, out = _build(["--hotapply-port", str(port)])
    node.serve_hot_apply()
    node.start_mission()
    _act(N, node)
    clk.t += 0.1
    node._tick()
    zone = {"id": "nfz-rest", "vertices": [{"x": 40, "y": 40}, {"x": 50, "y": 40},
                                           {"x": 50, "y": 50}, {"x": 40, "y": 50}]}
    url = f"http://127.0.0.1:{port}"
    r = httpx.post(f"{url}/dynamic_nfz", json=zone, timeout=10)
    assert r.status_code == 200 and r.json()["generation"] == 1, r.text
    assert (out / "policy_g1.json").is_file() and (out / "csp_g1.json").is_file()
    fences = [json.loads(m.data) for m in node.pubs[N.TOPIC_FENCES].sent]
    assert fences and fences[-1]["generation"] == 1 and "over REST" in fences[-1]["reason"]
    hot = [e for e in _events(out) if e["kind"] == "hot_apply_rest"]
    assert len(hot) == 1 and hot[0]["rule_id"] == "nfz-rest"
    assert hot[0]["receipt_to_swap_ms"] >= 0.0 and hot[0]["t"] == 0.1, hot
    clk.t += 0.1
    _act(N, node)
    node._tick()
    assert node.rows[-1]["generation"] == 1, node.rows[-1]
    node._finish("test over")
    metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    ha = metrics["hot_apply"]
    assert ha["n_applied"] == 1 and ha["endpoint"]["url"] == url, ha
    assert ha["spawn_target_ms"] == 50.0 and ha["receipt_to_swap_ms_max"] is not None
    assert node.hot.live_from == {"nfz-rest": 0.1}
    # After the episode: refused before the Shield is touched (503), or the
    # listener is already gone; either way generation 1 stays the last.
    try:
        r2 = httpx.post(f"{url}/dynamic_nfz", json={**zone, "id": "late"}, timeout=5)
        assert r2.status_code == 503, r2.text
    except httpx.HTTPError:
        pass
    assert node.policy.generation == 1


def test_the_listener_refuses_plaintext_off_loopback_and_a_stranger_over_tls():
    try:
        import httpx
        import uvicorn  # noqa: F401
        from guardrail import harness_service as H
        H._crypto()
    except Exception:                                   # noqa: BLE001
        return SKIP
    from guardrail import api as A
    from guardrail.replay import rail_shield
    from guardrail import load_policy
    sh = rail_shield(load_policy(POLICY), lookahead_s=5.0, dt=0.1)
    try:
        A.serve_in_background(sh, "0.0.0.0", _free_port())
    except ValueError as e:
        assert "loopback only" in str(e)
    else:
        raise AssertionError("a plaintext listener was opened off loopback")
    pki = Path(tempfile.mkdtemp(prefix="pki_"))
    H.generate_pki(pki)
    H.issue_cert(pki, "other-client")                   # the CA's, not allowed
    port = _free_port()
    th = A.serve_in_background(sh, "127.0.0.1", port, pki_dir=pki)
    import ssl

    def tls(name):
        ctx = ssl.create_default_context(cafile=str(H.pki_paths(pki, name)[0]))
        if name:
            _, cert, key = H.pki_paths(pki, name)
            ctx.load_cert_chain(str(cert), str(key))
        return ctx
    ok = httpx.get(f"https://127.0.0.1:{port}/health", verify=tls("harness-client"),
                   timeout=10)
    assert ok.status_code == 200 and ok.json()["generation"] == 0
    for name in ("other-client", ""):                   # a stranger, and no cert
        try:
            r = httpx.get(f"https://127.0.0.1:{port}/health", verify=tls(name),
                          timeout=10)
        except httpx.HTTPError:
            continue
        raise AssertionError(f"client {name or '(none)'!r} was answered: {r.status_code}")
    th.server.should_exit = True


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs fastapi, uvicorn, httpx: the 3.11 env)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        # SystemExit too: a rail whose parser refuses a flag exits, and
        # that is this test failing, not the run ending.
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
