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
    the largest difference. It is a model consistency check, not a
    measurement: both sides share setpoint_from_body and our ENU <-> NED
    model, so only the SITL run can show ArduPilot's own handling;
  * the SITL check fails a short climb, reports a refused MAVLink link
    instead of raising, and writes its JSON when the run raises; the DDS
    build holds the rig's ArduPilot commit; the DDS SITL runs on its own ROS
    domain and adds the MAVROS endpoint only on request;
  * forward at heading 90 is east; a short or non-finite setpoint is refused;
  * the FSM's modes as ArduCopter mode numbers (LOITER -> BRAKE 17);
  * the node on a stand-in ROS stack: nothing is sent before a heading, a
    body setpoint is published on /ap/cmd_vel in frame "map", the delivery
    report and beat go out on /mavlink_adapter/sent, a second publisher on
    /shield/setpoint stops forwarding, a mode request becomes a ModeSwitch
    call (only with --forward-modes), and every send is logged;
  * (2026-11-28) the Shield node's report.md Path row names the adapter seen
    on /shield/setpoint (the AP_DDS one: /ap/cmd_vel, no MAVROS setpoint),
    not a fixed MAVLink path: stage1_02 reported the MAVROS path it never
    flew;
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
    # A model consistency check, not a measurement: both paths share
    # setpoint_from_body, and ap_dds_receives is our model of ArduPilot's
    # ENU -> NED, so this cannot catch an error in that model. Only the SITL
    # run (ap_dds_verify.json: velocity_error, bearing_error_deg) can.
    print(f"      model consistency check (not a measurement): {n} setpoints "
          f"(360 headings x {len(GRID)}), max |NED(AP_DDS model) - NED(MAVROS)| "
          f"= {worst:.3g}")
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
            self.calls, self.n_publishers, self.qos = [], 1, {}

        def get_logger(self):
            return types.SimpleNamespace(info=lambda s: None, warn=lambda s: None,
                                         error=lambda s: None)

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
                to_msg=lambda: "stamp"))

        def create_subscription(self, typ, topic, cb, qos):
            self.subs[topic] = cb

        def create_publisher(self, typ, topic, qos):
            self.pubs[topic], self.qos[topic] = _Pub(), qos
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


def test_the_shields_delivery_monitor_accepts_the_adapters_reports():
    """RUNBOOK-ap-dds step 6's open point: the Shield node's setpoint-delivery
    monitor reads this node as it reads the MAVLink adapter. Same topic and
    depth as the Shield's subscription (ros2_shield_node imports TOPIC_SENT
    from mavlink_adapter_node and SENT_QOS_DEPTH from guardrail.delivery),
    and the Shield's own sequence - stamp, publish, feed every report and
    beat to a DeliveryMonitor - ends with every setpoint answered, no gap and
    no shortfall. Flown on ap_dds_stage1_01 (26 Nov): 244 published, 244
    reports, 101 beats, no gap."""
    import time
    from guardrail.delivery import (SENT_QOS_DEPTH, DeliveryMonitor,
                                    grade_reasons, parse_report)
    M = _node_module()
    assert M.TOPIC_SENT == A.TOPIC_SENT == "/mavlink_adapter/sent"
    node = M.ApDdsAdapterNode(None)
    assert node.qos[M.TOPIC_SENT] == SENT_QOS_DEPTH, node.qos
    sent = node.pubs[M.TOPIC_SENT]
    on_sp = node.subs[M.TOPIC_SETPOINT]
    node.subs[M.TOPIC_AP_POSE](_pose(45.0))
    mon, read = DeliveryMonitor(), 0

    def drain():
        nonlocal read
        for msg in sent.sent[read:]:
            rec = parse_report(msg.data)
            assert rec is not None, msg.data
            mon.report(rec, time.monotonic())
        read = len(sent.sent)

    for i in range(30):
        mon.published(time.monotonic())      # stamped before the publish
        on_sp(_Msg(data=[1.0, 0.5 * (i % 3), 0.0, 0.0]))
        drain()
        if i % 10 == 9:
            node.timers[0]()                  # the 1 Hz beat
            drain()
    mon.close(time.monotonic())
    s = mon.summary()
    assert (s["published"], s["reports"], s["beats"]) == (30, 30, 3), s
    assert s["shortfall"] == 0 and s["n_gaps"] == 0 and s["delivery_gaps"] == [], s
    assert s["adapter_n_sent"] == 30 and s["adapter_n_in"] == 30, s
    assert grade_reasons(s) == [], grade_reasons(s)


# --------------------------------------------------------------------------- #
# The Shield node's report.md Path row (2026-11-28)
# --------------------------------------------------------------------------- #

# The row every ROS 2 rail report.md carried until 2026-11-28, whatever the
# adapter: the MAVLink rail's path, which stays byte for byte its row.
MAVLINK_ROW = ("vla node -> /vla/action_4d (body) -> safety_shield -> /shield/setpoint "
               "(body) -> mavlink_adapter -> /mavros/setpoint_raw/local -> MAVROS 2 "
               "-> ArduPilot")


def _shield_report(at_start, at_end=None):
    """(node module, report.md's Path rows, metrics.json `setpoint_path`) of
    the Shield node flown two ticks on the stand-in ROS stack
    (tests/test_railinputs_node.py) with the node names `at_start`
    subscribed to /shield/setpoint, and `at_end` (default: the same) when
    the report is written."""
    import test_railinputs_node as RN
    N, node, clk, out = RN._build([])
    node.subscribers = tuple(at_start)
    node.start_mission()
    RN._mode(N, node, "GUIDED")
    RN._fly(N, node, clk, 2)
    if at_end is not None:
        node.subscribers = tuple(at_end)
    node._finish("test over")
    rows = [ln for ln in (out / "report.md").read_text(encoding="utf-8").splitlines()
            if ln.startswith("| Path |")]
    metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    return N, rows, metrics["setpoint_path"]


def test_the_shields_path_row_names_the_adapter_and_its_topic():
    """setpoint_path: one row per adapter, from the adapters' own names."""
    import test_railinputs_node as RN
    N = RN._node_module()
    # The Shield node writes the AP_DDS names down rather than importing the
    # adapter: they must be the adapter's own.
    assert N.AP_DDS_ADAPTER_NODE_NAME == D.ADAPTER_NODE_NAME == "ap_dds_adapter"
    assert N.AP_DDS_TOPIC_CMD_VEL == D.TOPIC_CMD_VEL == "/ap/cmd_vel"
    assert N.ADAPTER_NODE_NAME == A.ADAPTER_NODE_NAME
    assert N.TOPIC_RAW_LOCAL == A.TOPIC_RAW_LOCAL
    head = "vla node -> /vla/action_4d (body) -> safety_shield -> /shield/setpoint (body)"
    assert N.setpoint_path(["mavlink_adapter", "vla_bridge"]) == MAVLINK_ROW
    ap = N.setpoint_path(["ap_dds_adapter", "rosbag2_recorder"])
    assert ap == (head + " -> ap_dds_adapter -> /ap/cmd_vel -> micro-ROS agent "
                  "-> ArduPilot (AP_DDS)"), ap
    assert "mavros" not in ap.lower(), ap
    both = N.setpoint_path({"mavlink_adapter", "ap_dds_adapter"})
    assert "MORE THAN ONE ADAPTER" in both, both
    assert "/ap/cmd_vel" in both and "/mavros/setpoint_raw/local" in both, both
    nobody = N.setpoint_path(["rosbag2_recorder"])
    assert nobody == (head + " -> no setpoint adapter seen subscribed "
                      "(subscribers: rosbag2_recorder)"), nobody
    assert N.setpoint_path([]).endswith("(subscribers: none)")
    assert N.setpoint_path(None).endswith("were not read)")


def test_the_shield_nodes_report_follows_the_adapter_on_its_graph():
    """ap_dds_stage1_02 (26 Nov) flew /shield/setpoint -> ap_dds_adapter ->
    /ap/cmd_vel with no MAVLink setpoint adapter running, and its report.md
    said '... -> mavlink_adapter -> /mavros/setpoint_raw/local -> MAVROS 2'.
    The node now writes the path of the adapter it saw subscribed."""
    N, rows, sp = _shield_report(["ap_dds_adapter"])
    assert rows == [f"| Path | {N.setpoint_path(['ap_dds_adapter'])} |"], rows
    assert "/ap/cmd_vel" in rows[0] and "mavros" not in rows[0].lower(), rows
    assert sp == {"topic": "/shield/setpoint", "subscribers_seen": ["ap_dds_adapter"],
                  "path": N.setpoint_path(["ap_dds_adapter"])}, sp
    # The MAVLink rail (every KPI campaign): the row it always had.
    N, rows, sp = _shield_report(["mavlink_adapter"])
    assert rows == [f"| Path | {MAVLINK_ROW} |"], rows
    # Seen at mission start, gone by the report: still the path flown.
    N, rows, sp = _shield_report(["ap_dds_adapter"], [])
    assert "/ap/cmd_vel" in rows[0] and sp["subscribers_seen"] == ["ap_dds_adapter"], sp
    # Nothing that forwards setpoints: said so, not assumed to be MAVROS.
    N, rows, sp = _shield_report(["rosbag2_recorder"])
    assert "no setpoint adapter seen subscribed (subscribers: rosbag2_recorder)" \
        in rows[0] and "MAVROS" not in rows[0], rows


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


def test_climb_verdict_fails_short_or_missing():
    assert V.climb_verdict(9.5, 10.0)["ok"]
    assert not V.climb_verdict(4.0, 10.0)["ok"]
    assert not V.climb_verdict(None, 10.0)["ok"]


def test_mavlink_takeoff_reports_a_refused_link_instead_of_raising():
    """A taken or closed tcp:5760 comes back as ok False with the error, so
    run() still lands the copter and writes ap_dds_verify.json."""
    fake = types.ModuleType("pymavlink")
    mu = types.ModuleType("pymavlink.mavutil")

    def refuse(url):
        raise ConnectionRefusedError(111, "Connection refused")
    mu.mavlink_connection = refuse
    fake.mavutil = mu
    saved = {k: sys.modules.get(k) for k in ("pymavlink", "pymavlink.mavutil")}
    sys.modules["pymavlink"], sys.modules["pymavlink.mavutil"] = fake, mu
    try:
        r = V._mavlink_takeoff("tcp:127.0.0.1:5760", 10.0)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    assert r["ok"] is False and "ConnectionRefusedError" in r["error"], r


def test_verify_writes_its_report_when_the_run_raises():
    orig = V.run
    V.run = lambda args: (_ for _ in ()).throw(RuntimeError("rclpy missing"))
    try:
        with tempfile.TemporaryDirectory() as d:
            rc = V.main(["--out", d])
            rep = json.loads((Path(d) / "ap_dds_verify.json").read_text("utf-8"))
    finally:
        V.run = orig
    assert rc == 1 and rep["ok"] is False and "rclpy missing" in rep["error"], rep


def test_ap_dds_build_pins_the_rigs_ardupilot_commit():
    """setup_ap_dds.sh's default ref and SHA are sitl/setup_sitl.sh's pin."""
    import re
    rig = (ROOT / "sitl" / "setup_sitl.sh").read_text("utf-8")
    dds = (ROOT / "sitl" / "ap_dds" / "setup_ap_dds.sh").read_text("utf-8")
    rig_ref = re.search(r'ARDUPILOT_REF="\$\{ARDUPILOT_REF:-([^}]+)\}"', rig).group(1)
    rig_sha = re.search(r'ARDUPILOT_SHA="\$\{ARDUPILOT_SHA:-([0-9a-f]{40})\}"', rig).group(1)
    assert f'RIG_REF="{rig_ref}"' in dds and f'RIG_SHA="{rig_sha}"' in dds
    assert f'AP_DDS_REF="${{AP_DDS_REF:-{rig_ref}}}"' in dds
    assert "PIN FAIL" in dds


def test_ap_dds_sitl_is_isolated_and_mavros_endpoint_is_opt_in():
    s = (ROOT / "sitl" / "ap_dds" / "start_ap_dds_sitl.sh").read_text("utf-8")
    assert 'if [ "${AP_DDS_MAVROS:-0}" = "1" ]' in s
    assert 'udpclient:127.0.0.1:${SITL_ROUTER_PORT:-14550}' in s
    assert 'DOMAIN="${AP_DDS_DOMAIN_ID:-42}"' in s and "DDS_DOMAIN_ID" in s
    assert 'export ROS_DOMAIN_ID="$DOMAIN"' in s
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in s
    assert "AP_DDS_Client" in (ROOT / "sitl" / "ap_dds" / "setup_ap_dds.sh").read_text("utf-8")


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
