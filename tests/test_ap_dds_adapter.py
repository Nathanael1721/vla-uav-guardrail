"""The AP_DDS adapter (ARCH-08): the Shield's setpoint onto ArduPilot's
/ap/cmd_vel instead of MAVROS, checked without ROS or a simulator.

    python tests/test_ap_dds_adapter.py      (no pytest needed)

sitl/ap_dds_adapter_node.py is opt-in (nothing launches it; the KPI rail
still flies sitl/mavlink_adapter_node.py). What is checked here:

  * the equivalence sweep: for every whole-degree heading and a grid of body
    setpoints, the NED command ArduPilot's AP_DDS takes from the adapter's
    "map"-frame TwistStamped (AP_DDS_External_Control's ENU -> NED, modelled
    by ap_dds_receives) equals the NED command MAVROS sends for the same
    setpoint on the MAVLink path (mavlink_adapter_node.mavros_sends) and the
    single body -> NED crossing (body_to_ned). The sweep prints its count and
    the largest difference;
  * forward at heading 90 is east; a short or non-finite setpoint is refused;
  * the FSM's modes as ArduCopter mode numbers (LOITER -> BRAKE 17);
  * the node on a stand-in ROS stack: nothing is sent before a heading, a
    body setpoint is published on /ap/cmd_vel in frame "map", the delivery
    report and beat go out on /mavlink_adapter/sent, a second publisher on
    /shield/setpoint stops forwarding, a mode request becomes a ModeSwitch
    call (only with --forward-modes), and every send is logged;
  * the SITL check's pure verdicts (sitl/ap_dds/verify_ap_dds.py): a missing
    topic fails rather than reading as zero, and a flown velocity off the
    command or off its bearing fails.
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
import tempfile
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl", ROOT / "sitl" / "ap_dds"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import ap_dds_adapter_node as D                                      # noqa: E402
import mavlink_adapter_node as A                                     # noqa: E402
import verify_ap_dds as V                                            # noqa: E402

GRID = [(vx, vy, vz, wz) for vx in (-2.0, 0.0, 0.7, 3.0)
        for vy in (-1.5, 0.0, 1.0) for vz in (-1.0, 0.0, 0.5)
        for wz in (-0.4, 0.0, 0.3)]


def _ned_of_both_paths(body, heading):
    cv = D.cmd_vel_from_body(body, heading)
    via_dds = D.ap_dds_receives(cv["frame_id"], cv["linear"], cv["angular_z"])
    via_mavros = A.mavros_sends(A.setpoint_from_body(body, heading)["fields"])
    return via_dds, via_mavros, A.body_to_ned(body, heading)


def test_ap_dds_and_mavros_paths_command_the_same_ned_for_every_heading():
    worst, n = 0.0, 0
    for h in range(360):
        for body in GRID:
            dds, mav, ned = _ned_of_both_paths(body, float(h))
            for a, b, c in zip(dds, mav, ned):
                worst = max(worst, abs(a - b), abs(a - c))
            n += 1
    print(f"      equivalence sweep: {n} setpoints (360 headings x {len(GRID)}), "
          f"max |NED(AP_DDS) - NED(MAVROS)| = {worst:.3g}")
    assert n == 360 * len(GRID)
    assert worst <= 1e-12, worst


def test_forward_at_heading_90_flies_east_and_frame_is_map():
    cv = D.cmd_vel_from_body([2.0, 0.0, 0.0, 0.0], 90.0)
    assert cv["frame_id"] == "map"
    e, n, u = cv["linear"]
    assert abs(e - 2.0) < 1e-12 and abs(n) < 1e-12 and u == 0.0, cv
    # Up-positive in, up-positive ENU z out; a clockwise yaw rate is a
    # negative ENU angular z.
    cv = D.cmd_vel_from_body([0.0, 0.0, 0.8, 0.25], 0.0)
    assert abs(cv["linear"][2] - 0.8) < 1e-12 and abs(cv["angular_z"] + 0.25) < 1e-12


def test_bad_setpoints_and_frames_are_refused():
    for body in ([1.0, 0.0, 0.0], [float("nan"), 0, 0, 0], [0, float("inf"), 0, 0]):
        try:
            D.cmd_vel_from_body(body, 0.0)
        except ValueError:
            continue
        raise AssertionError(f"{body} was accepted")
    try:
        D.ap_dds_receives("base_link", (1.0, 0.0, 0.0), 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("a base_link command was modelled as map")


def test_fsm_modes_map_to_arducopter_numbers():
    assert D.mode_number("GUIDED") == ("GUIDED", 4)
    assert D.mode_number("LOITER") == ("BRAKE", 17)
    assert D.mode_number("LOITER", "LOITER") == ("LOITER", 5)
    assert D.mode_number("RTL") == ("RTL", 6)
    assert D.mode_number("LAND") == ("LAND", 9)
    for allowed in A.ALLOWED_MODES:
        D.mode_number(allowed)          # every mode the FSM can ask for has a number
    try:
        D.mode_number("STABILIZE")
    except ValueError:
        pass
    else:
        raise AssertionError("STABILIZE was given a number")


# --------------------------------------------------------------------------- #
# The node on a stand-in ROS stack
# --------------------------------------------------------------------------- #

class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Future:
    def __init__(self, result):
        self._r = result

    def result(self):
        return self._r

    def add_done_callback(self, cb):
        cb(self)


def _twist():
    v = lambda: _Msg(x=0.0, y=0.0, z=0.0)                             # noqa: E731
    return _Msg(header=_Msg(stamp=None, frame_id=""),
                twist=_Msg(linear=v(), angular=v()))


def _node_module():
    names = ("rclpy", "rclpy.node", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg",
             "std_msgs", "std_msgs.msg", "ardupilot_msgs", "ardupilot_msgs.srv")
    m = {n: types.ModuleType(n) for n in names}

    class Node:
        def __init__(self, name):
            self.name, self.pubs, self.subs, self.timers = name, {}, {}, []
            self.calls, self.n_publishers = [], 1

        def get_logger(self):
            return types.SimpleNamespace(info=lambda s: None, warn=lambda s: None,
                                         error=lambda s: None)

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
                to_msg=lambda: "stamp"))

        def create_subscription(self, typ, topic, cb, qos):
            self.subs[topic] = cb

        def create_publisher(self, typ, topic, qos):
            self.pubs[topic] = _Pub()
            return self.pubs[topic]

        def create_client(self, typ, name):
            node = self

            class C:
                def service_is_ready(self):
                    return True

                def call_async(self, req):
                    node.calls.append((name, req))
                    return _Future(_Msg(status=True, curr_mode=req.mode))
            return C()

        def create_timer(self, period, cb):
            self.timers.append(cb)

        def count_publishers(self, topic):
            return self.n_publishers

    m["rclpy.node"].Node = Node
    q = m["rclpy.qos"]
    q.QoSProfile = _Msg
    q.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    q.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    q.qos_profile_sensor_data = None
    m["geometry_msgs.msg"].PoseStamped = _Msg
    m["geometry_msgs.msg"].TwistStamped = _twist
    m["std_msgs.msg"].Float32MultiArray = _Msg
    m["std_msgs.msg"].String = _Msg
    m["ardupilot_msgs.srv"].ModeSwitch = types.SimpleNamespace(Request=_Msg)
    saved = {k: sys.modules.get(k) for k in m}
    sys.modules.update(m)
    try:
        spec = importlib.util.spec_from_file_location(
            "ap_dds_adapter_node_standin", ROOT / "sitl" / "ap_dds_adapter_node.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    assert mod.HAVE_ROS and mod.ModeSwitch is not None, "stand-in ROS did not import"
    return mod


def _pose(heading_deg):
    """An ENU PoseStamped whose orientation is this compass heading."""
    yaw_enu = math.radians(90.0 - heading_deg)
    return _Msg(pose=_Msg(orientation=_Msg(x=0.0, y=0.0, z=math.sin(yaw_enu / 2),
                                           w=math.cos(yaw_enu / 2))))


def test_the_node_sends_shield_setpoints_on_ap_cmd_vel():
    M = _node_module()
    with tempfile.TemporaryDirectory() as td:
        node = M.ApDdsAdapterNode(Path(td), forward_modes=True)
        assert node.name == "ap_dds_adapter"
        cmd = node.pubs[M.TOPIC_CMD_VEL]
        sent = node.pubs[M.TOPIC_SENT]
        on_sp = node.subs[M.TOPIC_SETPOINT]
        # No heading yet: dropped, not rotated with a guess.
        on_sp(_Msg(data=[1.0, 0.0, 0.0, 0.0]))
        assert cmd.sent == [] and node.n_dropped == 1
        node.subs[M.TOPIC_AP_POSE](_pose(30.0))
        body = [1.2, -0.4, 0.3, 0.1]
        on_sp(_Msg(data=body))
        assert len(cmd.sent) == 1
        tw = cmd.sent[0]
        assert tw.header.frame_id == "map"
        got = M.ap_dds_receives(tw.header.frame_id,
                                (tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z),
                                tw.twist.angular.z)
        want = A.body_to_ned(body, 30.0)
        assert max(abs(a - b) for a, b in zip(got, want)) < 1e-9, (got, want)
        rep = json.loads(sent.sent[-1].data)
        assert rep["n_sent"] == 1 and rep["n_in"] == 2 and rep["beat"] is False
        node.timers[0]()                                    # the 1 Hz beat
        assert json.loads(sent.sent[-1].data)["beat"] is True
        # A second publisher on /shield/setpoint: nothing more is flown.
        node.n_publishers = 2
        on_sp(_Msg(data=body))
        assert len(cmd.sent) == 1
        status = [json.loads(s.data) for s in node.pubs[M.TOPIC_STATUS].sent]
        assert status[-1]["kind"] == "extra_setpoint_publisher"
        node.n_publishers = 1
        on_sp(_Msg(data=body))
        assert len(cmd.sent) == 2
        # Mode requests -> /ap/mode_switch with ArduCopter numbers.
        node.subs[M.TOPIC_MODE_REQUEST](_Msg(data="LOITER"))
        node.subs[M.TOPIC_MODE_REQUEST](_Msg(data="stabilize"))
        assert [(n, r.mode) for n, r in node.calls] == [("/ap/mode_switch", 17)]
        status = [json.loads(s.data) for s in node.pubs[M.TOPIC_STATUS].sent]
        assert status[-2]["autopilot_mode"] == "BRAKE" and status[-2]["mode_sent"]
        assert status[-1]["kind"] == "mode_refused"
        node.close()
        lines = (Path(td) / "ap_dds_adapter_log.jsonl").read_text("utf-8").splitlines()
        assert len(lines) == 2
        rec = json.loads(lines[0])
        assert rec["ap_receives_ned"] == rec["ned"], rec


def test_modes_are_not_forwarded_without_the_flag():
    M = _node_module()
    node = M.ApDdsAdapterNode(None)
    assert M.TOPIC_MODE_REQUEST not in node.subs and node.cli_mode is None


# --------------------------------------------------------------------------- #
# The SITL check's verdicts (sitl/ap_dds/verify_ap_dds.py)
# --------------------------------------------------------------------------- #

def test_topic_check_needs_every_topic_at_a_rate():
    avail = {"/ap/time": ["builtin_interfaces/msg/Time"],
             "/ap/pose/filtered": ["geometry_msgs/msg/PoseStamped"],
             "/ap/navsat/navsat0": ["sensor_msgs/msg/NavSatFix"],
             "/ap/geopose/filtered": ["geographic_msgs/msg/GeoPoseStamped"]}
    res = V.resolve_topics(avail)
    assert res["navsat"][0] == "/ap/navsat/navsat0"          # 4.5 name accepted
    ok = V.topic_verdict(res, {"time": 50, "pose": 50, "navsat": 25, "geopose": 50}, 5.0)
    assert ok["ok"] and ok["topics"]["navsat"]["hz"] == 5.0
    slow = V.topic_verdict(res, {"time": 50, "pose": 50, "navsat": 0, "geopose": 50}, 5.0)
    assert not slow["ok"]
    del avail["/ap/geopose/filtered"]
    gone = V.topic_verdict(V.resolve_topics(avail), {"time": 50, "pose": 50,
                                                     "navsat": 25}, 5.0)
    assert not gone["ok"] and gone["topics"]["geopose"]["name"] is None


def test_velocity_verdict():
    good = V.velocity_verdict((1.0, 0.0), (0.93, 0.05), (5.2, 0.3))
    assert good["ok"] and good["bearing_error_deg"] < 5, good
    assert not V.velocity_verdict((1.0, 0.0), (0.5, 0.0), (5.0, 0.0))["ok"]
    assert not V.velocity_verdict((1.0, 0.0), (1.0, 0.0), (0.0, 5.0))["ok"]   # flew north
    assert not V.velocity_verdict((1.0, 0.0), None, (5.0, 0.0))["ok"]
    assert not V.velocity_verdict((1.0, 0.0), (1.0, 0.0), None)["ok"]
    assert V.bearing_deg(1.0, 0.0) == 90.0 and V.bearing_deg(0.0, -1.0) == 180.0
    # Through the adapter, body-forward is flown along the heading.
    e, n = V.expected_enu(True, 1.0, 180.0)
    assert abs(e) < 1e-12 and abs(n + 1.0) < 1e-12
    assert V.expected_enu(False, 1.0, None) == (1.0, 0.0)


def test_nothing_on_the_kpi_rail_launches_the_ap_dds_adapter():
    """Opt-in: the launchers that fly KPI scenarios do not start it."""
    n = 0
    for rel in ("sitl/run_ros2_demo.sh", "sitl/hil_desktop.sh", "sitl/ros2_ws/src"):
        p = ROOT / rel
        assert p.exists(), f"{rel} moved: point this check at the launcher"
        files = sorted(p.rglob("*")) if p.is_dir() else [p]
        for f in files:
            n += f.is_file()
            if f.is_file():
                assert "ap_dds_adapter" not in f.read_text("utf-8", errors="replace"), f
    assert n > 3, n


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
