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
    TLS allow-list (also cryptography), including a stranger whose POST
    arrives in the same write as its TLS Finished (2026-10-19: applied to
    the Shield on 1b95383, generation 0 -> 5 in the test's five tries);
  * (2026-10-19) a pilot taking the aircraft out of the node's or the FSM's
    hold is a takeover, after which the node sends neither GUIDED nor the
    mission-end LAND; mode requests go to the adapter by its node name, not
    to whatever subscribes (a bag); a /vla/identity that is not a JSON object
    is refused, not fatal; a pose stream that stops is held like a quiet
    action stream, and GUIDED comes back only once no hold reason is left.

Shown failing first against a pristine HEAD 4418ae0 copy (git archive; the
same stand-in, unknown flags dropped by parse_known_args): the stale stream
was flown on (no action_stale event, every row "pass" with the last action),
rows carry no e2e_ms, the Shield has no clock, and no listener answers.
"""
from __future__ import annotations

import importlib.util
import json
import math
import socket
import sys
import tempfile
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sitl"))

from guardrail.fsm import FSMState                                    # noqa: E402
from guardrail.models import State                                    # noqa: E402

POLICY = ROOT / "policies" / "sim_demo_policy.yaml"
SKIP = "SKIP"
# A node these tests tick by hand (_build): the 10 Hz timer's tick, and
# CPython's own garbage collector (sitl/ros2_shield_node.py --tick-trigger,
# --gc-pacing; tests/test_sitl_rails.py and test_sitl_stressors.py use it too).
HAND_TICKED = ["--tick-trigger", "timer", "--gc-pacing", "off"]
# The four ConstraintBase fields a REST rule body states (IR schema 3.0.0:
# constraint_type, priority, violation_action; 4.0.0: scope), at the values
# the deprecated POST /nfz body means.
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}


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

    # Who is subscribed to every topic: the adapter, by its ROS name. A test
    # sets the instance's `subscribers` to change it (a bag alone, ...).
    subscribers = ("mavlink_adapter",)

    def count_subscribers(self, topic):
        return len(self.subscribers)

    def get_subscriptions_info_by_topic(self, topic):
        return [types.SimpleNamespace(node_name=n, topic_name=topic)
                for n in self.subscribers]

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


def _pose_msg(N, st: State):
    """/mavros/local_position/pose for `st`: ENU (x = East, y = North), the
    heading as an ENU yaw counter-clockwise from East."""
    yaw = math.radians(90.0 - st.yaw_deg)
    return N.PoseStamped(pose=_Msg(
        position=_Msg(x=st.y, y=st.x, z=st.up),
        orientation=_Msg(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


def _build(argv: list[str]):
    """(module, node, virtual clock, out dir): the node built from its own
    parser, on the pad at 15 m, mission started.

    The stand-in MAVROS publishes the pose before every tick (the node's
    stale-pose watchdog, 2026-10-19, holds the aircraft once it stops);
    `node.standin_pose_on = False` stops it.

    These tests tick by hand: the node is built on its 10 Hz timer
    (--tick-trigger timer, so an action does not tick by itself) and with
    CPython's own collector (--gc-pacing off: nothing frozen in the test
    process). The defaults are tested in tests/test_sitl_rails.py; an argv
    that names either flag overrides these."""
    N = _node_module()
    clk = _Clock()
    N.time = clk
    args, _ = N.make_parser().parse_known_args(
        ["--policy", str(POLICY), "--dev-yaml"] + HAND_TICKED + argv)
    out = Path(tempfile.mkdtemp(prefix="ros2node_"))
    node = N.ShieldNode(args, out)
    node.state = State(x=0.0, y=0.0, up=15.0, yaw_deg=0.0)
    node.standin_pose_on = True
    tick = node._tick

    def tick_after_pose():
        if node.standin_pose_on:
            node._on_pose(_pose_msg(N, node.state))
        tick()
    node._tick = tick_after_pose
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
    zone = {"id": "nfz-rest", **HARD_P0,
            "vertices": [{"x": 40, "y": 40}, {"x": 50, "y": 40},
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


# --------------------------------------------------------------------------- #
# 2026-10-19 (bh_final sitl-ros2/*): takeovers out of a hold, no LAND over a
# pilot, mode requests routed to the adapter by name, a VLA identity that is
# not a JSON object, and a pose stream that stops. Each fails on 1b95383.
# --------------------------------------------------------------------------- #

def _modes(N, node) -> list[str]:
    return [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent]


def _mode(N, node, mode: str) -> None:
    """/mavros/state reporting `mode` (connected, armed)."""
    node._on_state(N.MavState(connected=True, armed=True, mode=mode))


def _fly(N, node, clk, n: int, act: bool = True) -> None:
    for _ in range(n):
        if act:
            _act(N, node)
        clk.t += 0.1
        node._tick()


def test_a_pilot_taking_the_aircraft_out_of_the_nodes_hold_is_a_takeover():
    """ros2-takeover-only-from-guided: only GUIDED -> X counted, so with the
    node holding (a quiet action stream, LOITER flown as BRAKE) a pilot's or
    failsafe's BRAKE -> LAND went unseen, and the resumed action stream sent
    GUIDED, pulling the aircraft out of the pilot's LAND."""
    N, node, clk, out = _build(["--action-timeout-s", "0.55"])
    node.start_mission()
    _mode(N, node, "GUIDED")
    _fly(N, node, clk, 3)
    for _ in range(20):                                 # the pilot goes quiet
        if _modes(N, node):
            break
        _fly(N, node, clk, 1, act=False)
    assert _modes(N, node) == ["LOITER"], _modes(N, node)
    _mode(N, node, "BRAKE")                             # the adapter's hold
    assert node.takeover is None, "the node's own hold read as a takeover"
    _mode(N, node, "LAND")                              # the operator lands it
    assert node.takeover is not None and node.takeover["mode"] == "LAND" \
        and node.takeover["previous"] == "BRAKE", node.takeover
    _fly(N, node, clk, 3)                               # the actions resume
    assert _modes(N, node) == ["LOITER"], (
        f"the node took the aircraft back from the pilot: {_modes(N, node)}")
    assert [r["setpoint"] for r in node.rows[-3:]] == ["none"] * 3
    resumed = [e for e in _events(out) if e["kind"] == "action_resumed"]
    assert resumed and resumed[-1]["held"] is True \
        and resumed[-1]["guided_requested"] is False, resumed


def test_a_pilot_taking_the_aircraft_out_of_the_fsms_loiter_is_a_takeover():
    """The FSM path of the same gap: FSM Loiter (BRAKE), the pilot switches
    to LAND, the FSM kept stepping, and its G8 (Loiter -> Normal once clear
    for 2 s) sent GUIDED over the pilot."""
    N, node, clk, out = _build([])
    node.start_mission()
    _mode(N, node, "GUIDED")
    _fly(N, node, clk, 1)
    node.fsm._enter(FSMState.LOITER, node._t())
    node._request_mode("LOITER", "test: the FSM's Loiter")
    _mode(N, node, "BRAKE")
    assert node.takeover is None
    _mode(N, node, "LAND")
    assert node.takeover is not None and node.takeover["mode"] == "LAND", node.takeover
    _fly(N, node, clk, 30)                              # 3 s of clean actions
    assert _modes(N, node) == ["LOITER"], _modes(N, node)
    g8 = [e for e in _events(out) if e["kind"] == "fsm_transition"]
    assert not g8, f"the FSM stepped after the takeover: {g8}"
    assert all(r["setpoint"] == "none" for r in node.rows[1:]), \
        [r["setpoint"] for r in node.rows]


def test_takeover_controls_guided_to_rtl_is_one_and_a_requested_mode_is_not():
    N, node, clk, out = _build([])
    node.start_mission()
    _mode(N, node, "GUIDED")
    _mode(N, node, "RTL")                               # nobody asked: GeoFence
    assert node.takeover == {"t": node.takeover["t"], "mode": "RTL",
                             "previous": "GUIDED"}, node.takeover
    N, node, clk, out = _build([])
    node.start_mission()
    _mode(N, node, "GUIDED")
    node._request_mode("LOITER", "test")
    _mode(N, node, "BRAKE")
    _mode(N, node, "GUIDED")                            # back from the node's hold
    assert node.takeover is None, node.takeover
    node._request_mode("LOITER", "test")
    _mode(N, node, "BRAKE")
    node._request_mode("RTL", "test: the FSM's RTL")
    _mode(N, node, "RTL")                               # asked for by the FSM
    assert node.takeover is None, node.takeover


def test_no_mission_end_land_over_a_pilot_who_took_the_aircraft():
    """ros2-land-after-takeover: after a takeover into LOITER (or ALT_HOLD,
    BRAKE, SMART_RTL) _finish still sent LAND, in the middle of the pilot's
    manoeuvre; only RTL and LAND were exempt."""
    N, node, clk, out = _build([])
    node.start_mission()
    _mode(N, node, "GUIDED")
    _fly(N, node, clk, 2)
    _mode(N, node, "LOITER")                            # a pilot on Mission Planner
    assert node.takeover is not None and node.takeover["mode"] == "LOITER"
    _fly(N, node, clk, 1)
    node._finish("autopilot took over (LOITER)")
    assert "LAND" not in _modes(N, node), _modes(N, node)
    end = [e for e in _events(out) if e["kind"] == "mission_end"][-1]
    assert end["mission_end_land"] is False \
        and "LOITER" in end["land_skipped_because"], end
    # Control: no takeover, the node still flying in GUIDED -> LAND.
    N, node, clk, out = _build([])
    node.start_mission()
    _mode(N, node, "GUIDED")
    _fly(N, node, clk, 2)
    node._finish("time cap")
    assert _modes(N, node) == ["LAND"], _modes(N, node)
    end = [e for e in _events(out) if e["kind"] == "mission_end"][-1]
    assert end["mission_end_land"] is True and end["land_skipped_because"] is None


class _ReadyClient:
    """A MAVROS service client that is up and keeps every request."""

    def __init__(self, clk=None):
        self.clk, self.sent = clk, []

    def service_is_ready(self):
        return True

    def call_async(self, req):
        self.sent.append((None if self.clk is None else self.clk.t, req))


def test_a_bag_on_the_mode_topic_is_not_taken_for_the_adapter():
    """mode-request-fallback-dead-under-bag: any subscriber of
    /shield/mode_request counted as the adapter, and rosbag2 subscribes to
    it in every recorded run, so with the adapter down LOITER / RTL / LAND
    went to the bag alone, recorded `via: adapter`."""
    N, node, clk, out = _build([])
    node.cli_mode = mavros = _ReadyClient()
    node.subscribers = ("rosbag2_recorder",)            # the adapter is down
    node.start_mission()
    node._request_mode("LOITER", "test")
    assert _modes(N, node) == [], "published to a topic only the bag reads"
    ev = [e for e in _events(out) if e["kind"] == "mode_request"][-1]
    assert [r.custom_mode for _, r in mavros.sent] == [ev["autopilot_mode"]]
    assert ev["autopilot_mode"] in N.HOLD_MODES, ev
    assert ev["via"].startswith("mavros") and "rosbag2_recorder" in ev["via"], ev
    node.subscribers = ("rosbag2_recorder", "mavlink_adapter")
    node._request_mode("RTL", "test")
    assert _modes(N, node) == ["RTL"] and len(mavros.sent) == 1
    assert [e for e in _events(out) if e["kind"] == "mode_request"][-1]["via"] == "adapter"
    assert not N.adapter_listening([]) and not N.adapter_listening(["rosbag2_recorder"])
    assert N.adapter_listening(["mavlink_adapter"])


def test_a_vla_identity_that_is_not_a_json_object_is_refused_not_fatal():
    """identity-non-object-crash: valid JSON that is not an object raised
    AttributeError inside the callback (ending spin and the node), or, when
    a _call() wrapper swallowed it, stayed in vla_identity for _report to
    crash on, losing every record of the flight."""
    N, node, clk, out = _build([])
    for d in ("null", "[1]", '"openvla"', "3", "{bad"):
        node._on_identity(N.String(data=d))
        assert node.vla_identity is None, (d, node.vla_identity)
    good = {"model_id": "guardrail.vla_stub.StubVLA", "kind": "stub", "host": None}
    node._on_identity(N.String(data=json.dumps(good)))
    assert node.vla_identity == good
    node._on_identity(N.String(data="[1]"))
    assert node.vla_identity == good, "a bad message replaced a good identity"
    fences_before = list(node.fence_status)
    node._on_adapter(N.String(data="[1]"))              # the sibling callback
    refused = [e for e in _events(out) if e["kind"] == "vla_identity_refused"]
    assert len(refused) == 6, refused
    # The sibling's refusal is its own record, and nothing reached the fence
    # status (review of 2026-10-19: the call was made and nothing about it
    # was checked, so its record could go without a test failing).
    adapter_refused = [e for e in _events(out) if e["kind"] == "adapter_status_refused"]
    assert len(adapter_refused) == 1 and adapter_refused[0]["data"] == "[1]", \
        adapter_refused
    assert node.fence_status == fences_before, node.fence_status
    node.start_mission()
    _fly(N, node, clk, 2)
    node._finish("test over")
    assert (out / "metrics.json").is_file() and (out / "manifest.json").is_file()


def test_a_pose_that_stops_is_held_and_the_next_pose_releases_it():
    """frozen-pose-undetected: nothing watched /mavros/local_position/pose,
    so a pose that stopped while the setpoints went through was re-checked
    every tick, repeated in the rows and the trajectory, and zone dwell over
    it could read 0 for a flight that crossed a zone."""
    N, node, clk, out = _build([])                      # default timeout 1.0 s
    assert node.pose_watchdog.timeout_s == N.POSE_TIMEOUT_S == 1.0
    node.cli_rate = rate = _ReadyClient(clk)
    node.start_mission()
    _mode(N, node, "GUIDED")
    _fly(N, node, clk, 5)
    clk.t += 0.05                                       # ticks off the threshold
    node.standin_pose_on = False                        # the pose stops; actions go on
    _fly(N, node, clk, 14)
    rows = node.rows
    assert [r["pose_stale"] for r in rows] == [False] * 14 + [True] * 5, \
        [r["pose_stale"] for r in rows]
    assert [r["action_stale"] for r in rows] == [False] * 19
    assert all(r["pose_age_ms"] > 1000.0 for r in rows[14:]), \
        [r["pose_age_ms"] for r in rows]
    # One tick of the Shield's answer to a stop, then the autopilot holds.
    assert [r["setpoint"] for r in rows[13:]] == ["pass", "pass"] + ["none"] * 4
    assert rows[14]["raw_body"]["vx"] == 0.0, "an action was flown on a stale pose"
    assert _modes(N, node) == ["LOITER"], _modes(N, node)
    onset_t = clk.t - 0.4
    assert any(abs(t - onset_t) < 1e-6 for t, _ in rate.sent), \
        f"the streams were not asked for again at the onset: {rate.sent}"
    _mode(N, node, "BRAKE")
    assert node.takeover is None
    node.standin_pose_on = True                         # the pose comes back
    _fly(N, node, clk, 1)
    assert _modes(N, node) == ["LOITER", "GUIDED"], _modes(N, node)
    assert rows[-1]["pose_stale"] is False and rows[-1]["setpoint"] == "pass"
    kinds = [e["kind"] for e in _events(out)]
    assert "pose_stale" in kinds and "pose_resumed" in kinds, kinds
    node._finish("test over")
    metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    pw = metrics["pose_watchdog"]
    assert pw["n_spells"] == 1 and pw["stale_ticks"] == 5 and pw["stale_s"] == 0.5, pw
    kpi = json.loads((out / "kpi.json").read_text(encoding="utf-8"))
    assert kpi["kpi_grade"] is False and any(
        "pose stream went stale" in w for w in kpi["kpi_grade_reasons"]), kpi
    assert "on a stale pose, not observed" in (out / "report.md").read_text(
        encoding="utf-8")


def test_an_action_that_resumes_does_not_release_a_hold_the_pose_still_needs():
    """The node holds for two reasons now; GUIDED comes back only once
    neither is left."""
    N, node, clk, out = _build(["--action-timeout-s", "0.55",
                                "--pose-timeout-s", "0.95"])
    node.start_mission()
    _mode(N, node, "GUIDED")
    clk.t += 0.05
    _fly(N, node, clk, 3)
    node.standin_pose_on = False                        # both streams stop
    _fly(N, node, clk, 12, act=False)
    assert node.rows[-1]["hold_reasons"] == ["action", "pose"], node.rows[-1]
    assert _modes(N, node) == ["LOITER"], "a second reason sent a second request"
    _fly(N, node, clk, 2)                               # actions, still no pose
    assert _modes(N, node) == ["LOITER"], (
        f"the resumed actions released a hold the pose still needs: {_modes(N, node)}")
    assert node.rows[-1]["hold_reasons"] == ["pose"] and node.rows[-1]["setpoint"] == "none"
    node.standin_pose_on = True
    _fly(N, node, clk, 1)
    assert _modes(N, node) == ["LOITER", "GUIDED"], _modes(N, node)
    assert node.rows[-1]["setpoint"] == "pass" and node.rows[-1]["hold_reasons"] == []


def test_the_fsm_does_not_step_on_a_stale_pose_or_count_it_as_cleared():
    """Review of 2026-10-19 (P1): the node holds for a stale pose only while
    it is the one flying, so with the FSM in Loiter (BRAKE) a stale pose
    added no hold and the FSM went on stepping on the frozen position. G8
    (Loiter -> Normal, clear for T_recover = 2 s) sent GUIDED 1.1 s after
    the pose stopped, and the pose hold sent LOITER on the next tick. With
    the FSM merely paused, its clean streak would still run through the gap
    and G8 would fire on the first fresh tick; it must wait T_recover of
    fresh clean ticks.
    The FSM's Loiter flown as BRAKE is the old flow since 2026-11-07 night
    (--hold-flight brake); the default holds it in GUIDED, the next test."""
    N, node, clk, out = _build(["--hold-flight", "brake"])   # pose timeout 1.0 s
    node.start_mission()
    _mode(N, node, "GUIDED")
    _fly(N, node, clk, 1)
    node.fsm._enter(FSMState.LOITER, node._t())
    node._request_mode("LOITER", "test: the FSM's Loiter")
    _mode(N, node, "BRAKE")
    clk.t += 0.05                                       # ticks off the threshold
    node.standin_pose_on = False                        # the pose stops; actions go on
    _fly(N, node, clk, 30)                              # 3 s of clean actions
    assert _modes(N, node) == ["LOITER"], (
        f"the FSM stepped on a frozen pose: {_modes(N, node)}")
    moved = [e for e in _events(out) if e["kind"] == "fsm_transition"]
    assert not moved, moved
    assert node.fsm.state == FSMState.LOITER and not node._holds
    stale = [r for r in node.rows if r["pose_stale"]]
    assert len(stale) == 21, len(stale)                 # 1.15 s .. 3.15 s
    # No FSM fields (the FSM did not step), nothing streamed.
    assert all(r.get("fsm_state_after") is None and r["setpoint"] == "none"
               and r["emitted_body"] is None for r in stale), \
        [(r.get("fsm_state_after"), r["setpoint"]) for r in stale]
    node.standin_pose_on = True                         # the pose comes back
    _fly(N, node, clk, 19)                              # 1.8 s of fresh clean ticks
    assert _modes(N, node) == ["LOITER"], (
        f"G8 counted the stale spell as cleared: {_modes(N, node)}")
    assert node.fsm.state == FSMState.LOITER
    _fly(N, node, clk, 3)                               # past T_recover, fresh
    assert _modes(N, node) == ["LOITER", "GUIDED"], _modes(N, node)
    g8 = [e for e in _events(out) if e["kind"] == "fsm_transition"]
    assert [e["edge"] for e in g8] == ["G8"], g8


def test_a_loiter_held_in_guided_goes_to_brake_on_a_stale_pose_and_comes_back():
    """2026-11-07 night: the FSM's Loiter is held in GUIDED by this node
    (guardrail/hold_flight.py, the default), flown from the pose alone. A
    stale pose hands it to BRAKE - the node's own pose hold, LOITER through
    the adapter - with nothing of the hold streamed and the FSM not stepping
    on the frozen pose; the next pose asks GUIDED again and the companion
    holds once more; G8 still waits T_recover of fresh clean ticks. A quiet
    action stream would not have touched it (_streaming)."""
    N, node, clk, out = _build([])                      # --hold-flight guided
    node.start_mission()
    _mode(N, node, "GUIDED")
    _fly(N, node, clk, 1)
    node.fsm._enter(FSMState.LOITER, node._t())
    clk.t += 0.05                                       # ticks off the threshold
    _fly(N, node, clk, 2)                               # the companion holds
    assert node.hold_flight.guiding and _modes(N, node) == [], _modes(N, node)
    held = [r for r in node.rows[-2:] if r["emitted_body"] is not None]
    assert len(held) == 2, [r["setpoint"] for r in node.rows[-2:]]
    node.standin_pose_on = False                        # the pose stops
    _fly(N, node, clk, 30)
    assert _modes(N, node) == ["LOITER"], _modes(N, node)
    assert node._holds == {"pose"} and node.fsm.state == FSMState.LOITER
    assert not [e for e in _events(out) if e["kind"] == "fsm_transition"]
    stale = [r for r in node.rows if r["pose_stale"]]
    assert len(stale) == 21, len(stale)
    assert all(r.get("fsm_state_after") is None for r in stale), stale[:1]
    # The hold's onset flies the Shield's stop while SET_MODE goes through;
    # after it nothing is streamed, the GUIDED hold's law included.
    assert [r["setpoint"] for r in stale] == ["pass"] + ["none"] * 20, \
        [r["setpoint"] for r in stale]
    assert all(r["emitted_body"] is None for r in stale[1:])
    _mode(N, node, "BRAKE")
    node.standin_pose_on = True                         # the pose comes back
    _fly(N, node, clk, 1)
    assert _modes(N, node) == ["LOITER", "GUIDED"], _modes(N, node)
    assert not node._holds and node.hold_flight.guiding
    _mode(N, node, "GUIDED")
    _fly(N, node, clk, 18)                              # 1.9 s of fresh clean ticks
    assert node.fsm.state == FSMState.LOITER, "G8 counted the stale spell as cleared"
    assert node.rows[-1]["emitted_body"] is not None    # held in GUIDED again
    _fly(N, node, clk, 3)                               # past T_recover, fresh
    g8 = [e for e in _events(out) if e["kind"] == "fsm_transition"]
    assert [e["edge"] for e in g8] == ["G8"], g8


def test_the_node_hands_its_fsm_the_holds_distance_and_x9_sends_it_home():
    """2026-11-11 (X9; hil smoke sm1111_048). Before each FSM step in Loiter
    the node asks its HoldWatch for the hold's distance and reach - with its
    own FSM and its own HoldFlight, whose GUIDED target is the point the
    hold flies to - passes both to the FSM, and writes them on the row; a
    hold carried away goes home by X9 (RTL through the adapter), before G8
    can hand it back. Here the watch is a stand-in reporting an aircraft
    beyond its reach from the first Loiter tick and receding at 3 m/s; the
    real watch, on the stand-in pose that does not move, reports a kept
    hold (distance 0)."""
    N, node, clk, out = _build([])
    node.start_mission()
    _mode(N, node, "GUIDED")
    _fly(N, node, clk, 1)
    node.fsm._enter(FSMState.LOITER, node._t())
    clk.t += 0.05
    _fly(N, node, clk, 3)
    kept = [r for r in node.rows[-3:] if "hold_off_m" in r]
    assert kept and all(r["hold_off_m"] == 0.0 for r in kept), node.rows[-3:]
    assert node.hold_flight.point is not None and node.fsm.state == FSMState.LOITER
    real, seen = node.hold_watch.judge, []
    t0 = node._t()

    def carried_away(fsm, hold_flight, x, y, t, speed):
        seen.append((fsm is node.fsm, hold_flight is node.hold_flight))
        if real(fsm, hold_flight, x, y, t, speed) == (None, None):
            return None, None                                # not in Loiter
        return 10.0 + 3.0 * (t - t0), 5.0
    node.hold_watch.judge = carried_away
    _fly(N, node, clk, 25)
    edges = [e for e in _events(out) if e["kind"] == "fsm_transition"]
    assert [e["edge"] for e in edges] == ["X9"], edges
    assert "hold not kept" in edges[0]["reason"], edges[0]["reason"]
    assert seen and all(s == (True, True) for s in seen), seen
    assert "RTL" in _modes(N, node), _modes(N, node)
    rows = [r for r in node.rows if r.get("hold_reach_m") == 5.0]
    assert rows and rows[-1]["fsm_edge"] == "X9", rows[-1]
    assert node.fsm.summary()["outcome_label"] == "RTL_triggered"


def _coalesced_post(port: int, pki: Path, name: str, body: dict) -> bytes:
    """POST /dynamic_nfz over mutual TLS as `name`, with the client's last
    handshake flight (certificate, verify, Finished) and the whole request in
    ONE TCP write, as a deliberate client sends it (or as one recv sees it on
    a server whose loop is late). Returns the decrypted answer, b"" if none."""
    import ssl
    from guardrail import harness_service as H
    ca, cert, key = H.pki_paths(pki, name)
    ctx = ssl.create_default_context(cafile=str(ca))
    ctx.load_cert_chain(str(cert), str(key))
    inc, out = ssl.MemoryBIO(), ssl.MemoryBIO()
    tls = ctx.wrap_bio(inc, out, server_hostname="localhost")
    raw = json.dumps(body).encode()
    req = (f"POST /dynamic_nfz HTTP/1.1\r\nHost: localhost\r\n"
           f"Content-Type: application/json\r\nContent-Length: {len(raw)}\r\n"
           f"Connection: close\r\n\r\n").encode() + raw
    got = b""
    with socket.create_connection(("127.0.0.1", port), timeout=5) as sock:
        while True:
            try:
                tls.do_handshake()
                break
            except ssl.SSLWantReadError:
                pending = out.read()
                if pending:
                    sock.sendall(pending)
                data = sock.recv(65536)
                if not data:
                    return got
                inc.write(data)
        tls.write(req)                   # queued behind the client's Finished
        sock.sendall(out.read())         # one write: Finished + the request
        try:
            while True:
                data = sock.recv(65536)
                if not data:
                    break
                inc.write(data)
                while True:
                    try:
                        chunk = tls.read(65536)
                    except ssl.SSLWantReadError:
                        break
                    if not chunk:        # the server's close_notify
                        return got
                    got += chunk
        except (OSError, ssl.SSLError):
            pass                         # refused: reset, or the TLS closed under it
    return got


def test_a_stranger_cannot_hot_apply_by_sending_its_request_with_its_finished():
    """2026-10-19 bug hunt: the allow-list closed a stranger's connection but
    uvicorn still parsed and RAN a request that came in the same read as the
    client's Finished. Five such POSTs from 'other-client' (signed by the same
    CA) were applied to the Shield (generation 0 -> 5) while the server logged
    each connection as closed. The test above only sent a GET and only checked
    that nothing answered. Here a stranger's coalesced POST must leave the
    Shield untouched, and the allowed client's identical POST is the control."""
    try:
        import uvicorn  # noqa: F401
        from guardrail import harness_service as H
        H._crypto()
    except Exception:                                   # noqa: BLE001
        return SKIP
    import time
    from guardrail import api as A
    from guardrail.replay import rail_shield
    from guardrail import load_policy
    sh = rail_shield(load_policy(POLICY), lookahead_s=5.0, dt=0.1)
    pki = Path(tempfile.mkdtemp(prefix="pki_"))
    H.generate_pki(pki)
    H.issue_cert(pki, "other-client")                   # the CA's, not allowed
    port = _free_port()
    th = A.serve_in_background(sh, "127.0.0.1", port, pki_dir=pki)
    square = [{"x": 40, "y": 40}, {"x": 50, "y": 40}, {"x": 50, "y": 50},
              {"x": 40, "y": 50}]
    try:
        for i in range(5):
            answer = _coalesced_post(port, pki, "other-client",
                                     {"id": f"intruder-{i}", **HARD_P0, "vertices": square})
            assert b"200 OK" not in answer, answer[:200]
        answer = _coalesced_post(port, pki, "harness-client",
                                 {"id": "friend", **HARD_P0, "vertices": square})
        assert answer.startswith(b"HTTP/1.1 200"), answer[:200]
        time.sleep(0.3)                  # anything still queued in the thread pool
        intruders = [c.id for c in sh.policy.constraints if c.id.startswith("intruder")]
        assert not intruders and sh.policy.generation == 1, \
            (sh.policy.generation, intruders)
        assert json.loads(answer.split(b"\r\n\r\n", 1)[1])["generation"] == 1
    finally:
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
