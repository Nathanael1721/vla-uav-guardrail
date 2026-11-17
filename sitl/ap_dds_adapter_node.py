"""
ROS 2 node: the AP_DDS adapter - the Shield's setpoint to ArduPilot's own DDS
interface instead of MAVROS (ARCH-08, the grant's "Q3 optional migration";
Architecture constraints p1: "For Year 1, MAVROS 2 is the canonical bridge.
AP_DDS ... is treated as a Q3 optional migration, not a blocker").

    [safety_shield] --/shield/setpoint--> [THIS NODE] --/ap/cmd_vel--> micro-ROS agent --> ArduPilot (AP_DDS)
                     Float32MultiArray                 geometry_msgs/TwistStamped
                     body [vx_fwd, vy_right,           frame_id "map" (ENU), linear
                     vz_up, yaw_rate_cw]               (east, north, up), angular.z CCW

OPT-IN. Nothing launches this node: sitl/run_ros2_demo.sh, the KPI campaigns
and the hil rig start sitl/mavlink_adapter_node.py exactly as before, so no
flown scenario changes. The node is run by hand on an ArduPilot SITL built
with DDS in a separate checkout (sitl/ap_dds/, docs/RUNBOOK-ap-dds.md).
Never run it beside the MAVLink adapter on one autopilot: both would fly the
same setpoints.

THE SAME ROTATION AS THE MAVLINK ADAPTER

The body -> local-NED crossing is mavlink_adapter_node.setpoint_from_body
(guardrail.frames.from_body then to_local_ned), called here unchanged, at the
heading the pose topic gives. The NED setpoint is then packed for AP_DDS's
"map" frame. ArduPilot's AP_DDS_External_Control::handle_velocity_control
takes a TwistStamped whose frame_id is "map" as ENU and converts it itself:
NED = (linear.y, linear.x, -linear.z), yaw rate = -angular.z; a frame_id of
"base_link" is a body-frame command, which this node never sends (ArduPilot
would rotate it with its own AHRS heading, not the one the Shield checked
against). So the message carries x = east, y = north, z = up and a
counter-clockwise yaw rate - the same packing MAVROS needs
(mavlink_adapter_node.ned_to_mavros_enu). `ap_dds_receives()` models
ArduPilot's conversion and tests/test_ap_dds_adapter.py checks that it gives
back the NED setpoint the MAVLink path sends, for every heading.

WHAT IS AND IS NOT CARRIED (the grant: "AP_DDS coverage is improving but
uneven across topics needed by the Shield")

  * /shield/setpoint -> /ap/cmd_vel (above), only while the Shield is the
    single publisher on /shield/setpoint, as in the MAVLink adapter.
  * /shield/mode_request -> /ap/mode_switch (ardupilot_msgs/srv/ModeSwitch,
    ArduCopter mode numbers), with --forward-modes. Off by default: in the
    first migration stage the Shield node still reads MAVROS and sends its
    own mode requests through /mavros/set_mode.
  * /mavlink_adapter/sent: the same per-send report and 1 Hz beat
    (guardrail.delivery), so the Shield's setpoint-delivery monitor reads this
    node the way it reads the MAVLink adapter.
  * NOT carried: fences and missions (/shield/fences, /shield/mission*): AP_DDS
    has no GeoFence or mission upload; the GCS-set fence (sitl/gcs_backstop.py)
    or MAVROS stays the path for them.

Run (after `source /opt/ros/jazzy/setup.bash` and the AP_DDS workspace's
install/setup.bash, which provides ardupilot_msgs):

    ~/venv-ros/bin/python sitl/ap_dds_adapter_node.py [--out <dir>] [--forward-modes]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SITL = Path(__file__).resolve().parent
for _p in (ROOT, SITL):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.delivery import (BEAT_PERIOD_S, SENT_QOS_DEPTH,          # noqa: E402
                                sent_report)
from mavlink_adapter_node import (DEFAULT_HOLD_MODE, HOLD_MODES,         # noqa: E402
                                  TOPIC_MODE_REQUEST, TOPIC_SENT,
                                  TOPIC_SETPOINT, autopilot_mode,
                                  heading_deg_from_enu_quaternion,
                                  mode_request, setpoint_from_body)

ADAPTER_NODE_NAME = "ap_dds_adapter"
TOPIC_CMD_VEL = "/ap/cmd_vel"
TOPIC_AP_POSE = "/ap/pose/filtered"
SRV_MODE_SWITCH = "/ap/mode_switch"
TOPIC_STATUS = "/ap_dds_adapter/status"
# AP_DDS_External_Control: "map" = ENU world frame, "base_link" = body frame.
FRAME_MAP = "map"
FRAME_BASE_LINK = "base_link"

# ArduCopter flight-mode numbers (ArduCopter/mode.h, Mode::Number), the
# values /ap/mode_switch takes. Only the modes the escalation FSM can end in
# (autopilot_mode's outputs) are listed.
COPTER_MODE_NUMBER = {"GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9, "BRAKE": 17}


# --------------------------------------------------------------------------- #
# Pure conversions (tested on any machine)
# --------------------------------------------------------------------------- #

def cmd_vel_from_body(body: list | tuple, heading_deg: float) -> dict:
    """The /ap/cmd_vel fields for one Shield setpoint.

    Returns {"frame_id", "linear" (east, north, up), "angular_z" (CCW rad/s),
    "ned" (v_n, v_e, v_d, yaw_rate_cw)}. Raises ValueError for a short or
    non-finite setpoint, as setpoint_from_body does."""
    sp = setpoint_from_body(body, heading_deg)
    v_n, v_e, v_d, yaw_cw = sp["ned"]
    return {"frame_id": FRAME_MAP,
            "linear": (v_e, v_n, -v_d),
            "angular_z": -yaw_cw,
            "ned": sp["ned"]}


def ap_dds_receives(frame_id: str, linear: tuple, angular_z: float) -> tuple:
    """What ArduPilot's AP_DDS commands for a "map"-frame TwistStamped:
    (v_n, v_e, v_d, yaw_rate_cw), as AP_DDS_External_Control converts ENU to
    NED ((x, y, z) -> (y, x, -z), yaw rate -z). Only "map" is modelled: the
    adapter never sends "base_link"."""
    if frame_id != FRAME_MAP:
        raise ValueError(f"only frame_id {FRAME_MAP!r} is modelled, got {frame_id!r}")
    x, y, z = linear
    return (y, x, -z, -angular_z)


def mode_number(fsm_mode: str, hold_mode: str = DEFAULT_HOLD_MODE) -> tuple[str, int]:
    """(ArduCopter mode name, number) for an FSM mode request: the FSM's
    LOITER becomes the hold mode as in the MAVLink adapter (BRAKE by
    default). Raises ValueError for a mode the FSM cannot ask for."""
    name = autopilot_mode(fsm_mode, hold_mode)
    return name, COPTER_MODE_NUMBER[name]


# --------------------------------------------------------------------------- #
# The ROS 2 node (needs rclpy + ardupilot_msgs for --forward-modes)
# --------------------------------------------------------------------------- #

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from std_msgs.msg import Float32MultiArray, String
    HAVE_ROS = True
except ImportError:                                   # Windows, CI: pure part only
    HAVE_ROS = False

try:
    from ardupilot_msgs.srv import ModeSwitch
except ImportError:                                   # only --forward-modes needs it
    ModeSwitch = None

if HAVE_ROS:
    LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class ApDdsAdapterNode(Node):
        def __init__(self, out: Path | None, pose_topic: str = TOPIC_AP_POSE,
                     forward_modes: bool = False,
                     hold_mode: str = DEFAULT_HOLD_MODE) -> None:
            super().__init__(ADAPTER_NODE_NAME)
            if forward_modes and ModeSwitch is None:
                raise RuntimeError("--forward-modes needs ardupilot_msgs: source the "
                                   "AP_DDS workspace (docs/RUNBOOK-ap-dds.md)")
            self.out = out
            self.hold_mode = hold_mode
            self.heading_deg: float | None = None
            self.n_sent = self.n_in = self.n_dropped = 0
            self._publishers_seen: int | None = None
            self._log_fh = None
            # AP_DDS publishes the pose best-effort; sensor-data QoS reads
            # best-effort and reliable writers alike.
            self.create_subscription(PoseStamped, pose_topic, self._on_pose,
                                     qos_profile_sensor_data)
            self.create_subscription(Float32MultiArray, TOPIC_SETPOINT,
                                     self._on_setpoint, 10)
            # Reliable writer: matches a reliable or a best-effort reader.
            self.pub = self.create_publisher(TwistStamped, TOPIC_CMD_VEL, 10)
            self.status = self.create_publisher(String, TOPIC_STATUS, LATCHED)
            self.sent_pub = self.create_publisher(String, TOPIC_SENT, SENT_QOS_DEPTH)
            self.create_timer(BEAT_PERIOD_S, self._beat)
            self.cli_mode = None
            if forward_modes:
                self.cli_mode = self.create_client(ModeSwitch, SRV_MODE_SWITCH)
                self.create_subscription(String, TOPIC_MODE_REQUEST,
                                         self._on_mode_request, 10)
            self.get_logger().info(
                f"AP_DDS adapter up: {TOPIC_SETPOINT} (body) -> {TOPIC_CMD_VEL} "
                f"(frame_id {FRAME_MAP!r}, ENU), heading from {pose_topic}, "
                f"modes {'-> ' + SRV_MODE_SWITCH if forward_modes else 'not forwarded'}")

        def _on_pose(self, msg) -> None:
            q = msg.pose.orientation
            self.heading_deg = heading_deg_from_enu_quaternion(q.x, q.y, q.z, q.w)

        def _on_setpoint(self, msg) -> None:
            self.n_in += 1
            body = list(msg.data)
            # Only the Shield flies (as in the MAVLink adapter).
            n_pub = self.count_publishers(TOPIC_SETPOINT)
            if n_pub != self._publishers_seen:
                self._publishers_seen = n_pub
                if n_pub > 1:
                    self.get_logger().error(
                        f"{n_pub} publishers on {TOPIC_SETPOINT}: forwarding stopped")
                    self._status({"kind": "extra_setpoint_publisher",
                                  "publishers": n_pub})
            if n_pub > 1:
                self.n_dropped += 1
                return
            if self.heading_deg is None:
                self.n_dropped += 1
                if self.n_dropped in (1, 10, 100):
                    self.get_logger().warn(
                        f"no heading yet: dropped {self.n_dropped} setpoint(s)")
                return
            try:
                cv = cmd_vel_from_body(body, self.heading_deg)
            except ValueError as e:
                self.n_dropped += 1
                self.get_logger().warn(f"dropping setpoint: {e}")
                return
            tw = TwistStamped()
            tw.header.stamp = self.get_clock().now().to_msg()
            tw.header.frame_id = cv["frame_id"]
            tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = (
                float(v) for v in cv["linear"])
            tw.twist.angular.z = float(cv["angular_z"])
            self.pub.publish(tw)
            self.n_sent += 1
            t_sent = time.monotonic()
            self.sent_pub.publish(String(data=sent_report(self.n_sent, self.n_in,
                                                          t_sent)))
            self._log({"t": round(t_sent, 4), "body": body,
                       "heading_deg": round(self.heading_deg, 3),
                       "ned": [round(v, 4) for v in cv["ned"]],
                       "cmd_vel_enu": [round(float(v), 4) for v in cv["linear"]]
                       + [round(float(cv["angular_z"]), 4)],
                       "ap_receives_ned": [round(v, 4) for v in ap_dds_receives(
                           cv["frame_id"], cv["linear"], cv["angular_z"])]})

        def _on_mode_request(self, msg) -> None:
            mode, refused = mode_request(msg.data)
            if refused is not None:
                self._status(refused)
                return
            name, number = mode_number(mode, self.hold_mode)
            if not self.cli_mode.service_is_ready():
                self._status({"kind": "mode_request", "mode": mode,
                              "autopilot_mode": name, "mode_sent": False,
                              "why": f"{SRV_MODE_SWITCH} not ready"})
                return
            fut = self.cli_mode.call_async(ModeSwitch.Request(mode=number))

            def done(f, mode=mode, name=name):
                r = f.result()
                self._status({"kind": "mode_request", "mode": mode,
                              "autopilot_mode": name, "via": "ap_dds",
                              "mode_sent": bool(r and r.status)})
            fut.add_done_callback(done)

        def _beat(self) -> None:
            self.sent_pub.publish(String(data=sent_report(
                self.n_sent, self.n_in, time.monotonic(), beat=True)))

        def _status(self, rec: dict) -> None:
            self.status.publish(String(data=json.dumps(rec)))

        def _log(self, rec: dict) -> None:
            if self.out is None:
                return
            if self._log_fh is None:
                self.out.mkdir(parents=True, exist_ok=True)
                self._log_fh = open(self.out / "ap_dds_adapter_log.jsonl", "a",
                                    encoding="utf-8", newline="\n")
            self._log_fh.write(json.dumps(rec) + "\n")
            self._log_fh.flush()

        def close(self) -> None:
            self.get_logger().info(f"AP_DDS adapter: {self.n_sent} sent, "
                                   f"{self.n_dropped} dropped")
            if self._log_fh is not None:
                self._log_fh.close()
                self._log_fh = None


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    ap.add_argument("--out", default=None,
                    help="directory for ap_dds_adapter_log.jsonl")
    ap.add_argument("--pose-topic", default=TOPIC_AP_POSE,
                    help="PoseStamped (ENU) the heading is read from: AP_DDS's "
                         f"{TOPIC_AP_POSE} (default), or /mavros/local_position/pose "
                         "when the Shield node still reads MAVROS, so both rotate "
                         "with one heading")
    ap.add_argument("--forward-modes", action="store_true",
                    help=f"forward /shield/mode_request to {SRV_MODE_SWITCH}")
    ap.add_argument("--hold-mode", choices=HOLD_MODES, default=DEFAULT_HOLD_MODE,
                    help="ArduCopter mode for the FSM's LOITER (default BRAKE)")
    return ap


def main(argv: list[str] | None = None) -> None:
    args, ros_args = build_parser().parse_known_args(argv)
    if not HAVE_ROS:
        raise SystemExit("ap_dds_adapter_node needs rclpy: source "
                         "/opt/ros/jazzy/setup.bash and use ~/venv-ros/bin/python")
    rclpy.init(args=ros_args)
    node = ApDdsAdapterNode(Path(args.out) if args.out else None,
                            pose_topic=args.pose_topic,
                            forward_modes=args.forward_modes,
                            hold_mode=args.hold_mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
