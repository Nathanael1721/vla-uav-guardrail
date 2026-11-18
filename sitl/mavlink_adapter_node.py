"""
ROS 2 node: the MAVLink adapter - the one place a body-frame setpoint becomes
local NED (grant: Architecture constraints p2, Safety Shield p5).

    [safety_shield] --/shield/setpoint--> [THIS NODE] --/mavros/setpoint_raw/local--> MAVROS 2
                     Float32MultiArray                 mavros_msgs/PositionTarget
                     body [vx_fwd, vy_right,           FRAME_LOCAL_NED, velocity +
                     vz_up, yaw_rate]                  yaw_rate (type_mask 1479)

The grant locks the VLA action as "a = (vx, vy, vz, yaw_rate) # body-frame
velocities + yaw rate" and puts the conversion to NED in the MAVLink adapter,
after the Shield. The PI's reference implementation (kuanting-vla-uav-
guardrail/ros2_ws/src/mavlink_adapter/) has this node; this one keeps its
topic, message and type_mask.

THE MESSAGE IS FILLED IN ENU, BECAUSE MAVROS CONVERTS IT

MAVROS treats every geometry field of a ROS message as ROS-convention ENU and
converts it itself. For PositionTarget with FRAME_LOCAL_NED, setpoint_raw's
local callback applies ftf::transform_frame_enu_ned to position, velocity and
acceleration (x, y, z) -> (y, x, -z), and flips yaw_rate's sign, before it
sends SET_POSITION_TARGET_LOCAL_NED. So the NED setpoint is computed with
guardrail.frames.body_to_local_ned (the one body -> NED function every rail's
adapter calls) and then packed the way MAVROS
expects: x = east, y = north, z = up, yaw_rate counter-clockwise. NED values
written into the message unconverted would reach ArduPilot with north and
east swapped and the vertical inverted. `mavros_sends()` reproduces MAVROS's
conversion, and tests/test_mavlink_adapter.py checks that it gives back
exactly the NED setpoint for every heading. Flown in SITL on 2026-10-06: the
flown displacement followed the commanded direction (docs/DESIGN-ros2-
interface.md, "The frame chain").

Heading comes from /mavros/local_position/pose (the same topic the Shield node
reads), not from compass_hdg as in the reference: both nodes must rotate with
the same heading, or the round trip body -> world -> body -> NED would
rotate the command by the difference.

WHAT ELSE GOES THROUGH HERE (the MAVROS-facing side, WP3-09)

  * /shield/mode_request (std_msgs/String) -> /mavros/set_mode: the Shield's
    escalation FSM and the mission end ask for GUIDED / LOITER / RTL / LAND.
    Only the modes the FSM can ask for (guardrail.fsm.MODE_FOR) are
    forwarded; anything else (STABILIZE, ACRO, ...) is refused and reported.
    The FSM's LOITER is sent as ArduCopter's BRAKE (`autopilot_mode`; see
    DEFAULT_HOLD_MODE for why: LOITER sinks with no pilot on the throttle).
  * /shield/fences (std_msgs/String JSON, latched) -> /mavros/geofence/push:
    the policy's keep-out zones, converted to lat/lon about the autopilot's
    home, uploaded as ArduPilot exclusion polygons / circles. With
    sitl/fence/guardrail_fence.parm (FENCE_ENABLE=1, FENCE_ACTION=1) the
    autopilot's own GeoFence is the backstop behind the Shield (Architecture
    constraints p1). Only zones whose altitude band covers the whole fence
    column, and that no clock can turn off (no time window, no
    time_window_switch that switches them off), are uploaded, so the
    backstop is never stricter than the policy,
    and a zone the aircraft is inside (or within FENCE_MARGIN of) is held back
    until it is clear, so a hot-applied zone on top of the aircraft cannot
    breach the autopilot fence the moment it arrives. A policy with a
    geographic frame origin far from the autopilot's home is refused.
  * /shield/mission (std_msgs/String JSON, latched) -> /mavros/mission/push:
    the grant's second mission style (Safety Shield page, MAVLink adapter
    table: "Pre-planned waypoint mission | MISSION_ITEM_INT | For
    mission-style flight; replaces or updates the next waypoint"). A waypoint
    plan in the policy's local metres becomes ArduPilot mission items about
    the autopilot's home (`mission_items`: seq 0 home, seq 1 DO_CHANGE_SPEED,
    then one NAV_WAYPOINT per waypoint), pushed whole and read back
    (/mavros/mission/pull). MAVROS 2 writes them with the mission protocol,
    as MISSION_ITEM_INT (the rail's bag carries the raw frames on
    /uas1/mavlink_sink). Since 2026-11-03.
  * /shield/mission_update (std_msgs/String JSON) -> /mavros/mission/push:
    ONE item replaced in the autopilot's mission, the grant's "replaces or
    updates the next waypoint". ArduCopter 4.5.7 applies the item as it
    arrives and, when it is the item being flown, restarts it ("Auto mission
    changed, restarted command"): the new waypoint is flown at once. The
    items written are then read back and compared, and that read-back - not
    MAVROS's push result - is the update's verdict. Since 2026-11-15 the
    update is written as the WHOLE mission with that item replaced
    (start_index 0: MISSION_COUNT, every item as MISSION_ITEM_INT,
    MISSION_ACK ACCEPTED), one update at a time (MissionPushQueue). Until
    then it went with start_index = the item's seq, a partial write, and
    MAVROS 2.14 writes MISSION_WRITE_PARTIAL_LIST with end_index = start +
    count, which MAVLink and ArduPilot read as the last index INCLUDED, so
    ArduPilot asked for one item more, MAVROS refused it as out of range, and
    the push ended in ArduPilot's 8 s upload timeout (MISSION_ACK 15) with the
    item already written (flown 2026-11-03: mission_wp_20261103c, decoded
    from its bag's /uas1/mavlink_sink and /uas1/mavlink_source); a second
    update inside those 8 s would have been refused. MISSION_UPDATE_WRITES
    says why the full write; `--mission-update-write partial` re-flies the
    old one. sitl/ros2_mission_node.py, the mission-style Shield, sends both
    topics.
  * /mavlink_adapter/status (std_msgs/String JSON, latched): every upload and
    mode request result, which the Shield node writes into events.jsonl.
  * /mavlink_adapter/sent (std_msgs/String JSON): one report per setpoint
    handed to MAVROS (n_sent, n_in = setpoint callbacks run, t_sent on this
    host's monotonic clock) and a beat once a second from a timer, so the
    Shield can compare what it published with what went out (SETPOINT
    DELIVERY below; guardrail/delivery.py). Since 2026-11-17.
  * <out>/adapter_log.jsonl: every setpoint in, the heading used, the NED
    setpoint and the ENU message out.

A GCS-SET GEOFENCE (2026-11-03, WP3-10). With `--fences-by-gcs` this node
uploads no zone: the operator's ground station set the fence parameters and
the signed bundle's zones before the flight (sitl/gcs_backstop.py on the
router's GCS endpoint, run_ros2_demo.sh GCS_BACKSTOP=1), as the grant has it
("GCS-set AP GeoFence parameters ... remain configured as a last-line
backstop"), and a push from here would replace the GCS's list. Each
/shield/fences message is answered with a status that says so.

ONLY THE SHIELD FLIES. A setpoint is forwarded only while the Shield node is
the single publisher on /shield/setpoint: a second publisher (a VLA backend
configured to publish there) would fly unshielded actions, so its arrival
stops forwarding and is reported (`extra_setpoint_publisher`); ArduPilot then
stops in GUIDED for want of setpoints, which is the safe side.

SETPOINT DELIVERY (2026-11-17). In hil run ntFIN2_175_dyn-nfz-spawn-ahead
(12 Nov) this node's setpoint callback did not run for 25.6 s (adapter_log
.jsonl: 441 sent, a gap from 7.99 s; its shutdown line: 441 sent, 0
dropped) while the Shield published 245 setpoints and ticked on time. The
node spins one single-threaded executor (rclpy.spin), and none of its
callbacks blocks: every service call is call_async, nothing sleeps or spins
inside a callback; per setpoint it runs a graph query (count_publishers) and
appends one line to adapter_log.jsonl. When the callback ran again it took
ten setpoints within 18 ms - the subscription's KEEP_LAST depth - so either
the executor did not run for 25.6 s or the subscription delivered nothing
for that long; that run's logs cannot tell which (the beat below can). The
executor model is unchanged: no cause in this node's code is shown, and a
threaded executor would change what flies on a guess. What changed is that
the stall is now seen: every send is reported on /mavlink_adapter/sent and
a 1 Hz timer beats there, the Shield records any publication no send
followed for over 0.5 s as a delivery gap and compares its publication
count with the reports (a shortfall: setpoints lost while other sends
continued), in metrics.json `setpoint_delivery`, and a run with either is
not KPI-grade. Beats during a gap: the executor ran and the setpoints did
not arrive; none: this process was stopped. What is checked is this node's
publish() to /mavros/setpoint_raw/local, not that MAVROS forwarded it to the
autopilot (guardrail/delivery.py, WHAT IS CHECKED).

SROS 2: under ROS_SECURITY_STRATEGY=Enforce a keystore whose permissions
were signed before 2026-11-17 does not grant /mavlink_adapter/sent, so
creating its publisher here (and the Shield's subscription) fails at node
construction (rclpy RCLError, docs/DESIGN-sros2.md): both nodes refuse to
start and nothing flies. Re-sign first (tools/sros2_setup.sh
--permissions-only, then tools/sros2_check.sh); sitl/run_ros2_demo.sh with
SROS2=1 checks the enclaves' permissions name the topic before launching.

Run (after `source /opt/ros/jazzy/setup.bash`):

    ~/venv-ros/bin/python sitl/mavlink_adapter_node.py [--out demo/out/<tag>]
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail.delivery import (BEAT_PERIOD_S, SENT_QOS_DEPTH,          # noqa: E402
                                sent_report)
from guardrail.frames import body_to_local_ned                            # noqa: E402
# time_controls and zone_clearance_m are re-exported: this module's callers
# and tests read them here, where they lived until 2026-10-12.
from guardrail.fence_export import (deferral, is_enforced, iter_zones,    # noqa: E402,F401
                                    time_controls, zone_clearance_m)
from guardrail.fence_export import zones_near as _zones_near             # noqa: E402
from guardrail.fsm import MODE_FOR, FSMState                             # noqa: E402
from guardrail.projection import LocalProjection                         # noqa: E402

# mavros_msgs/PositionTarget constants, written out so this module imports
# without ROS. The ROS node checks them against the message at start-up.
FRAME_LOCAL_NED = 1
IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
FORCE = 512
IGNORE_YAW, IGNORE_YAW_RATE = 1024, 2048
# The reference adapter's mask: ignore position, acceleration and yaw; command
# velocity + yaw_rate. Equal to sitl/run_sitl_demo.py's VEL_YAWRATE_MASK.
TYPE_MASK_VEL_YAWRATE = (IGNORE_PX | IGNORE_PY | IGNORE_PZ | IGNORE_AFX
                         | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW)

# MAVLink fence items (mission protocol, MAV_MISSION_TYPE_FENCE).
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002
MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003
MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

# MAVLink mission items (mission protocol, MAV_MISSION_TYPE_MISSION).
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_FRAME_GLOBAL = 0
MAV_FRAME_MISSION = 2
# The mission layout `mission_items` writes: ArduPilot keeps seq 0 for home
# (an upload's item 0 is replaced by the home position), seq 1 sets the speed,
# the waypoints follow from FIRST_WAYPOINT_SEQ.
SPEED_ITEM_SEQ = 1
FIRST_WAYPOINT_SEQ = 2
# DO_CHANGE_SPEED param1: 1 = ground speed (ArduCopter reads air and ground
# speed alike); param3 -1 = leave the throttle.
SPEED_TYPE_GROUND = 1.0
# NAV_WAYPOINT param1 is the hold time in seconds.
MAX_HOLD_S = 3600.0

# HOW AN IN-FLIGHT UPDATE IS WRITTEN (2026-11-15, WP3-09).
#
#   full     (default) the whole mission the autopilot holds, with the one
#            item replaced, through /mavros/mission/push with start_index 0:
#            MAVROS 2 sends MISSION_COUNT(n), ArduPilot asks for every item
#            (MISSION_REQUEST 0 .. n-1), MAVROS answers each with
#            MISSION_ITEM_INT, ArduPilot ends the transfer with MISSION_ACK
#            ACCEPTED - the exchange the pre-flight upload has always used,
#            flown and read back since 2026-11-03.
#   partial  the one item at start_index = seq: MAVROS 2.14 sends
#            MISSION_WRITE_PARTIAL_LIST(seq, seq + 1), which MAVLink and
#            ArduPilot read as the last index INCLUDED, so ArduPilot writes
#            the item, asks for seq + 1, which MAVROS refuses as out of range,
#            and ends the transfer 8 s later with MISSION_ACK 15 (operation
#            cancelled, "Mission upload timeout"): flown on 2026-11-03
#            (mission_wp_20261103c / _d, decoded from the bags). The item
#            lands, but a second update inside those 8 s would be refused.
#
# The full write is what a ground station does when a plan is edited in
# flight, and the only one MAVROS 2.14 completes. The items it rewrites
# unchanged should not disturb the flight: ArduCopter's ModeAuto restarts
# the current command only when the CONTENT of the upcoming nav commands
# changes (AP_Mission_ChangeDetector, read from the ArduPilot 4.5 source,
# not yet seen in a flight with this write), as it did for the partial write
# ("Auto mission changed, restarted command"). The partial write is kept,
# behind --mission-update-write partial, to re-fly the 3 Nov behaviour.
MISSION_UPDATE_WRITES = ("full", "partial")
DEFAULT_MISSION_UPDATE_WRITE = "full"

# The modes the adapter forwards: those the escalation FSM can ask for (Normal
# and Brake fly GUIDED; Loiter, RTL, Land), which include the mission-end LAND.
ALLOWED_MODES = frozenset(MODE_FOR.values())
MODE_FOR_LOITER = MODE_FOR[FSMState.LOITER]

# THE FSM'S LOITER IS ARDUCOPTER'S BRAKE MODE (2026-10-08).
#
# The grant names the mode "LOITER" (Safety Shield p5, "SET_MODE GUIDED /
# LOITER / RTL / LAND"), and that is what the FSM requests and the audit
# records. What it asks of the autopilot is "hold the aircraft where it is".
# ArduCopter's LOITER does that only with a pilot on the sticks: its climb
# rate is the throttle stick's (Copter-4.5.7 mode_loiter.cpp: "target_climb_rate
# = get_pilot_desired_climb_rate(channel_throttle->get_control_in())"), and
# with no RC input in SITL that is a steady descent - 14.2 m to the ground
# on 7 Oct (ros2fix2_on_fsm); on 8 Oct (pas_ap_20261008_224440) every one of
# the seven LOITER spells sank at 1.4-1.8 m/s, the deepest from 7.6 m to
# 1.8 m in 3.3 s, below the policy's 6 m floor. The alternatives a companion
# computer has:
#   * LOITER with the throttle centred by RC_CHANNELS_OVERRIDE: the companion
#     impersonates the pilot. ArduPilot accepts overrides only from
#     SYSID_MYGCS (Mission Planner's 255), they lapse after RC_OVERRIDE_TIME
#     unless re-sent, and while they are sent the safety pilot's own throttle
#     is overridden. A hold that lives in the companion is not the
#     autopilot's hold.
#   * GUIDED with a position-hold setpoint: holds altitude, but the companion
#     keeps flying the aircraft, the mode never changes (Mission Planner shows
#     no escalation), and the FSM's Loiter means the autopilot owns it.
#   * BRAKE: ArduCopter's own stop-and-hold (mode.h: "full-brake using
#     inertial/GPS system, no pilot input"). mode_brake.cpp stops with the
#     position controller at zero velocity and holds altitude
#     ("set_pos_target_z_from_climb_rate_cm(0.0f)"), with no timeout unless
#     a Solo button sets one. It needs a position estimate, as LOITER does.
# BRAKE is what this adapter sends for the FSM's LOITER. `hold_mode="LOITER"`
# restores the literal mapping, for a flight with a pilot holding the
# throttle centred.
HOLD_MODES = ("BRAKE", "LOITER")
DEFAULT_HOLD_MODE = "BRAKE"


def autopilot_mode(fsm_mode: str, hold_mode: str = DEFAULT_HOLD_MODE) -> str:
    """The ArduCopter mode to request for one of the FSM's modes: the FSM's
    LOITER becomes `hold_mode` (BRAKE by default), the rest pass through."""
    mode = str(fsm_mode).strip().upper()
    if hold_mode not in HOLD_MODES:
        raise ValueError(f"hold_mode {hold_mode!r} is not one of {HOLD_MODES}")
    if mode not in ALLOWED_MODES:
        raise ValueError(f"{mode!r} is not a mode the escalation FSM asks for "
                         f"({sorted(ALLOWED_MODES)})")
    return hold_mode if mode == MODE_FOR_LOITER else mode


class ModeRequests:
    """The modes a node asked the autopilot for, in both vocabularies, and
    whether one of them explains the autopilot leaving GUIDED now.

    `fsm` is what was asked in the FSM's (and the grant's) words - LOITER,
    RTL, LAND, GUIDED - and `autopilot` what ArduCopter was asked for (the
    FSM's LOITER as `hold_mode`, BRAKE by default). Both rails record both
    (metrics.json `mode_requests` / `autopilot_mode_requests`), so one
    escalation reads the same on either rail. Until 2026-10-09 the Project
    AirSim node recorded ArduCopter's words and the ROS node the FSM's.

    A mode change is OURS only if a request made since the aircraft last came
    back to GUIDED can explain it (`explains`). The old check asked whether
    the mode was ANYWHERE in the episode's list, so after one FSM hold a
    pilot's BRAKE from Mission Planner could never read as a takeover. A
    request that went through the adapter (`via_adapter=True`) is flown as
    the ADAPTER's hold mode, which this node does not know, so either hold
    mode explains it."""

    def __init__(self, hold_mode: str = DEFAULT_HOLD_MODE) -> None:
        if hold_mode not in HOLD_MODES:
            raise ValueError(f"hold_mode {hold_mode!r} is not one of {HOLD_MODES}")
        self.hold_mode = hold_mode
        self.fsm: list[str] = []
        self.autopilot: list[str] = []
        # (FSM mode, autopilot modes it may produce), since GUIDED was entered.
        self._pending: list[tuple[str, frozenset[str]]] = []

    def ask(self, mode: str, *, via_adapter: bool = False) -> str:
        """Record one request (the FSM's word) and return ArduCopter's."""
        m = str(mode).strip().upper()
        ap = autopilot_mode(m, self.hold_mode)
        self.fsm.append(m)
        self.autopilot.append(ap)
        may = (frozenset(HOLD_MODES) if via_adapter and m == MODE_FOR_LOITER
               else frozenset({ap}))
        self._pending.append((m, may))
        return ap

    def entered_guided(self) -> None:
        """The autopilot is back in GUIDED: the requests that led here are
        spent. Requests made AFTER the last GUIDED request stay (a hold asked
        for before the GUIDED report arrived)."""
        last = max((i for i, (m, _) in enumerate(self._pending) if m == "GUIDED"),
                   default=None)
        self._pending = [] if last is None else self._pending[last + 1:]

    def explains(self, autopilot_mode_now: str) -> bool:
        """Did a pending request ask for the mode the autopilot just entered?"""
        mode = str(autopilot_mode_now).strip().upper()
        return any(mode in may for m, may in self._pending if m != "GUIDED")
# ArduPilot's FENCE_MARGIN default; the node passes the autopilot's own value.
FENCE_MARGIN_M = 2.0
# How far above the policy's highest legal altitude the autopilot's altitude
# fence must sit, so it never fires on a flight the Shield considers legal.
FENCE_ALT_HEADROOM_M = 5.0
# A policy anchored to lat/lon whose frame origin is further than this from the
# autopilot's home would put the Shield's zones and the uploaded fence in the
# wrong place by that much.
ORIGIN_TOLERANCE_M = 5.0

# Topics, shared with sitl/ros2_shield_node.py.
TOPIC_SETPOINT = "/shield/setpoint"
TOPIC_MODE_REQUEST = "/shield/mode_request"
TOPIC_FENCES = "/shield/fences"
TOPIC_STATUS = "/mavlink_adapter/status"
# What this node actually sent (guardrail/delivery.py): a report per setpoint
# handed to MAVROS, and a beat once a second, for the Shield's delivery monitor.
TOPIC_SENT = "/mavlink_adapter/sent"
TOPIC_RAW_LOCAL = "/mavros/setpoint_raw/local"
TOPIC_MISSION = "/shield/mission"
TOPIC_MISSION_UPDATE = "/shield/mission_update"
# This node's ROS name: the Shield node routes its mode requests through the
# adapter only when a subscriber of this name is on /shield/mode_request
# (ros2_shield_node.adapter_listening), not merely any subscriber - rosbag2
# subscribes to that topic in every recorded run.
ADAPTER_NODE_NAME = "mavlink_adapter"

# The rails' X,Y options (sitl/ros2_shield_node.py, ros2_vla_stub_node.py,
# run_sitl_demo.py). --start (a scenario's start, south or west of a policy's
# frame origin) and --action (the constant pilot's world-frame action, a
# negative first channel flies south) since 2026-11-06.
XY_FLAGS = ("--target", "--subject", "--start", "--action")
_NEGATIVE_VALUE = re.compile(r"^-\.?\d")


def join_negative_xy(argv, flags=XY_FLAGS) -> list[str]:
    """`[flag, "-20,15"]` -> `[f"{flag}=-20,15"]` for each X,Y flag.

    argparse takes a separate value that starts with '-' for an option unless
    it reads as a negative NUMBER, and "-20,15" does not (the comma), so
    `--target -20,15` - a target south of home - exited with status 2 on
    every rail, the interpreters the rails run (3.11, Ubuntu's 3.12.3)
    included; the stress harness passes the value as its own argv element
    (experiments/sweep_scenarios.py build_sitl_cmd). Safe because no option
    of these parsers starts with '-' and a digit: a real option after the
    flag is never swallowed."""
    argv = list(argv)
    out: list[str] = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in flags and i + 1 < len(argv) and _NEGATIVE_VALUE.match(argv[i + 1]):
            out.append(f"{a}={argv[i + 1]}")
            i += 2
            continue
        out.append(a)
        i += 1
    return out


class XYArgumentParser(argparse.ArgumentParser):
    """An ArgumentParser whose X,Y options take a negative X as a separate
    value (join_negative_xy), whichever way it is called: parse_args goes
    through parse_known_args."""

    def parse_known_args(self, args=None, namespace=None):
        args = sys.argv[1:] if args is None else args
        return super().parse_known_args(join_negative_xy(args), namespace)


# --------------------------------------------------------------------------- #
# Pure conversions (tested on any machine)
# --------------------------------------------------------------------------- #

def heading_deg_from_enu_quaternion(qx: float, qy: float, qz: float,
                                    qw: float) -> float:
    """Vehicle heading, 0 = North, clockwise positive, in [0, 360).

    MAVROS publishes the pose in ENU with a FLU body, so the quaternion's
    yaw is measured counter-clockwise from East. The heading the grant's
    body frame rotates with is measured clockwise from North:
    heading = 90 deg - yaw_enu.
    """
    yaw_enu = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
    h = (90.0 - math.degrees(yaw_enu)) % 360.0
    # -1e-14 % 360.0 rounds to 360.0 in floating point: keep [0, 360).
    return 0.0 if h >= 360.0 else h


def body_to_ned(body: list | tuple, heading_deg: float) -> tuple:
    """Body [vx_fwd, vy_right, vz_up, yaw_rate_cw] -> (v_n, v_e, v_d, yaw_rate_cw).

    The ROS 2 rail's call of THE body -> local-NED crossing,
    guardrail.frames.body_to_local_ned, which every rail's adapter calls
    (tests/test_frame_boundary.py fails on any other crossing).
    """
    vx, vy, vz, wz = (float(v) for v in body[:4])
    return body_to_local_ned((vx, vy, vz, wz), heading_deg)


def ned_to_mavros_enu(v_n: float, v_e: float, v_d: float,
                      yaw_rate_cw: float) -> dict:
    """The PositionTarget fields MAVROS needs to SEND (v_n, v_e, v_d, yaw_rate).

    MAVROS converts ENU -> NED itself (see the module docstring), so the
    message carries x = east, y = north, z = up, and a counter-clockwise yaw
    rate.
    """
    return {"coordinate_frame": FRAME_LOCAL_NED,
            "type_mask": TYPE_MASK_VEL_YAWRATE,
            "velocity": (v_e, v_n, -v_d),
            "yaw_rate": -yaw_rate_cw}


def mavros_sends(fields: dict) -> tuple:
    """What MAVROS 2's setpoint_raw puts on the wire for these message fields.

    A model of ftf::transform_frame_enu_ned ((x, y, z) -> (y, x, -z)) and of
    the yaw-rate flip (ENU angular z -> NED angular z = -z) for
    FRAME_LOCAL_NED. Returns (v_n, v_e, v_d, yaw_rate) as in
    SET_POSITION_TARGET_LOCAL_NED. Used by the tests and written into the
    adapter log, so the log shows the command ArduPilot received.
    """
    if fields["coordinate_frame"] != FRAME_LOCAL_NED:
        raise ValueError("only FRAME_LOCAL_NED is modelled")
    x, y, z = fields["velocity"]
    return (y, x, -z, -fields["yaw_rate"])


def mode_request(text: str) -> tuple[str, dict | None]:
    """`(mode, refusal)` for a /shield/mode_request message: refusal is None
    for a mode the escalation FSM can ask for, else the status record."""
    mode = str(text).strip().upper()
    if mode in ALLOWED_MODES:
        return mode, None
    return mode, {"kind": "mode_refused", "mode": mode,
                  "allowed": sorted(ALLOWED_MODES)}


def setpoint_from_body(body: list | tuple, heading_deg: float) -> dict:
    """Everything the adapter computes for one setpoint, for the message and
    the log."""
    if len(body) < 4:
        raise ValueError(f"a setpoint needs 4 fields, got {len(body)}")
    if not all(math.isfinite(float(v)) for v in body[:4]):
        raise ValueError(f"non-finite setpoint {list(body)}")
    ned = body_to_ned(body, heading_deg)
    fields = ned_to_mavros_enu(*ned)
    return {"ned": ned, "fields": fields}


# --------------------------------------------------------------------------- #
# Policy fences -> ArduPilot fence items
# --------------------------------------------------------------------------- #

# The selection itself - which zones a clockless 2-D autopilot fence may hold
# exactly as the policy does - is guardrail.fence_export.policy_zones since
# 2026-10-12, so the perception rail shares it; this module keeps its names.


def policy_fences(policy, column_m: float | None,
                  position: tuple[float, float] | None = None,
                  margin_m: float = FENCE_MARGIN_M, enforced=None
                  ) -> tuple[list[dict], list[dict]]:
    """The policy's keep-out zones the autopilot fence can enforce NOW.

    Returns `(fences, skipped)`. The zones are guardrail.fence_export's
    clock-proof selection (`policy_zones`; its docstring has the rules): a
    polygon_fence and a static dynamic_nfz become an exclusion polygon, a
    circle_fence an exclusion circle; a corridor, a moving dynamic_nfz, a zone
    a clock can turn off (`time_controlled: True`), and a zone whose altitude
    band does not cover the column 0 .. `column_m` (FENCE_ALT_MAX) are
    skipped with why. With `column_m` unknown nothing is exported.

    `position` is the aircraft's (x North, y East). A zone the aircraft is
    inside, or within `margin_m` of, is held back (`deferred: True` in
    `skipped`): uploading it would breach the autopilot fence at once and
    ArduPilot would RTL while the Shield is still steering out - the grant's
    GeoFence is to "fire only if the Shield itself crashes or fails to emit".
    The caller uploads it once the aircraft is clear. The skip list keeps the
    policy's order, deferred zones in place, as before the move.

    `enforced` (id -> zone as last uploaded with success): a zone the
    autopilot already holds with that shape is never held back, since every
    upload replaces the whole list (guardrail.fence_export.defer_near).
    """
    fences, skipped = [], []
    for kind, entry in iter_zones(policy, column_m=column_m):
        if kind == "zone" and position is not None                 and not is_enforced(entry, enforced):
            held = deferral(entry, position, margin_m)
            if held is not None:
                kind, entry = "skipped", held
        (fences if kind == "zone" else skipped).append(entry)
    return fences, skipped


def zones_near(fences: list[dict], x: float, y: float,
               margin_m: float = FENCE_MARGIN_M) -> list[str]:
    """Ids of exported zones that (x, y) is inside or within `margin_m` of.

    Used with the home position: a keep-out zone over home is one the
    autopilot's own RTL (the escalation FSM's, or the GeoFence's) flies back
    into. Flown 2026-10-07 with policies/poly_test.yaml, whose tri-nfz has a
    vertex on the origin: held back at take-off, uploaded once clear, then
    "Fence Breached" when the RTL brought the aircraft home."""
    return _zones_near(fences, x, y, margin_m)


def policy_ceiling_m(policy, fallback_m: float | None = None, *, ground=None,
                     home_msl_m: float | None = None) -> tuple[float | None, str]:
    """The highest altitude the policy allows, and where that figure came
    from: its altitude envelopes, else `fallback_m` (the mission's cruise
    altitude), else None.

    With `ground` (the Shield node's --ground, X-11) every envelope's ceiling
    is turned into a height above HOME, the reference ArduPilot measures
    FENCE_ALT_MAX in (`home_msl_m`: home's elevation, m MSL; default the
    ground's own at home, for a flat ground its one elevation): an MSL
    envelope's ceiling less home's elevation; an AGL envelope's plus the
    highest ground the ground covers less home's elevation (over a DEM's
    ridge an AGL ceiling is that much above home; over a flat ground at home,
    the ceiling itself). The highest of those, as without a ground. Until
    2026-11-18 an MSL ceiling (624 m for experiments/policies/msl_dem_sitl.
    yaml) was handed on as metres above home, and the node would have raised
    SITL's FENCE_ALT_MAX to 629 m. Without `ground`: unchanged."""
    envs = [c for c in getattr(policy, "constraints", [])
            if getattr(c, "type", "") == "altitude_envelope"
            and getattr(c, "alt_max_m", None) is not None]
    if envs and ground is not None:
        from guardrail.terrain import ground_band, is_dem, is_msl
        top = float(ground_band(ground)[1])
        if home_msl_m is None:
            home_msl_m = float(ground.elevation_at(0.0, 0.0) if is_dem(ground)
                               else ground.elevation_at())
        home = float(home_msl_m)
        ceilings = [float(c.alt_max_m) - home if is_msl(c)
                    else float(c.alt_max_m) + (top - home) for c in envs]
        return max(ceilings), (f"altitude_envelope, above home ({home:g} m MSL): MSL "
                               f"ceilings less home, AGL ceilings plus the ground's "
                               f"highest {top:g} m MSL less home")
    ceilings = [float(c.alt_max_m) for c in envs]
    if ceilings:
        return max(ceilings), "altitude_envelope"
    if fallback_m is not None:
        return float(fallback_m), "mission cruise altitude (no altitude_envelope)"
    return None, "no altitude_envelope and no mission"


def fence_alt_plan(policy_ceiling: float | None, fence_alt_max: float | None,
                   headroom_m: float = FENCE_ALT_HEADROOM_M) -> dict:
    """Is the autopilot's altitude fence above every altitude the policy
    allows? `{"ok", "raise_to", "why"}`; `raise_to` is the FENCE_ALT_MAX the
    node should set when it is not.

    The .parm file carries FENCE_ALT_MAX 30 for the demo policies (ceiling
    20 m), but the node flies any --policy, and 18 of the policies in
    policies/ allow more than 25 m (up to 80 m, most of them 55 m): with 30 m
    the backstop would RTL a flight the Shield considers legal."""
    if policy_ceiling is None:
        return {"ok": False, "raise_to": None,
                "why": "the policy's ceiling is unknown"}
    need = float(policy_ceiling) + headroom_m
    if fence_alt_max is None:
        return {"ok": False, "raise_to": need,
                "why": "FENCE_ALT_MAX was not read from the autopilot"}
    if float(fence_alt_max) >= need:
        return {"ok": True, "raise_to": None, "why": None}
    return {"ok": False, "raise_to": need,
            "why": (f"FENCE_ALT_MAX {float(fence_alt_max):g} m is below the "
                    f"policy ceiling {float(policy_ceiling):g} m + "
                    f"{headroom_m:g} m: the backstop would fire on a flight "
                    f"the Shield considers legal")}


def origin_offset_m(origin: tuple[float, float] | None,
                    home_lat: float, home_lon: float) -> tuple[float, float] | None:
    """(north, east) of the autopilot's home in the policy's local frame, or
    None for a policy with no geographic origin (local metres about home)."""
    if not origin:
        return None
    n, e = LocalProjection(float(origin[0]), float(origin[1])).to_local(
        home_lat, home_lon)
    return n, e


def fence_frame(doc: dict, home_lat: float, home_lon: float,
                home_ne: tuple[float, float] = (0.0, 0.0)
                ) -> tuple[tuple[float, float], tuple[float, float] | None, dict | None]:
    """`(home_local_ne, origin offset, refusal)` for one /shield/fences
    document: where the autopilot's home sits in the frame the Shield node
    flies, the geographic offset of home from the policy's frame origin
    (origin_offset_m; None for a metre policy), and a refusal record or None.

    The Shield node flies the policy's frame with the EKF origin at the
    policy-frame position it declares in the document (`home_policy_ne`, its
    --start: a scenario that starts 200 m south of the frame origin boots
    SITL there; absent, as in every document before 2026-11-06, (0, 0)), so
    home, `home_ne` metres from the EKF origin (HomePosition.position, 0 in
    SITL), is at declared + home_ne. fence_items projects the zones about
    home from there, and a geo-anchored policy whose frame origin puts the
    autopilot's home more than ORIGIN_TOLERANCE_M from it is refused: the
    zones would be uploaded off by the difference. Until 2026-11-06 the
    offset was compared with (0, 0), so the frame origin had to BE home."""
    dec = doc.get("home_policy_ne") or (0.0, 0.0)
    try:
        dn, de = float(dec[0]), float(dec[1])
    except (TypeError, ValueError, IndexError):
        dn = de = math.nan
    base = {"home_policy_ne": dec}
    if not (math.isfinite(dn) and math.isfinite(de)):
        return (0.0, 0.0), None, dict(base, why=f"home_policy_ne {dec!r} is not a "
                                                f"finite (north, east) position")
    hn, he = dn + float(home_ne[0]), de + float(home_ne[1])
    org = doc.get("origin")
    off = origin_offset_m((org["lat"], org["lon"]) if org else None,
                          home_lat, home_lon)
    if off is not None:
        miss = math.hypot(off[0] - hn, off[1] - he)
        if miss > ORIGIN_TOLERANCE_M:
            where = ("the policy's frame origin is "
                     f"{math.hypot(*off):.1f} m from the autopilot's home"
                     if abs(dn) < 1e-9 and abs(de) < 1e-9 else
                     f"the autopilot's home lies at ({off[0]:.1f}, {off[1]:.1f}) m in "
                     f"the policy's frame, {miss:.1f} m from ({hn:.1f}, {he:.1f}) m "
                     f"where the Shield node flies it (its --start)")
            return (hn, he), off, dict(
                base, origin_offset_m=[round(off[0], 2), round(off[1], 2)],
                why=(f"{where} (tolerance {ORIGIN_TOLERANCE_M:g} m): the zones "
                     f"would be uploaded in the wrong place"))
    return (hn, he), off, None


def fence_items(fences: list[dict], home_lat: float, home_lon: float,
                home_local_ne: tuple[float, float] = (0.0, 0.0)) -> list[dict]:
    """mavros_msgs/Waypoint fields for /mavros/geofence/push.

    Policy coordinates are local metres (x North, y East) about the
    autopilot's local origin; the home position sits at `home_local_ne` in
    that frame (0, 0 in SITL, where the EKF origin is home). Each vertex is
    projected to lat/lon about home with the same equirectangular projection
    the policy loader uses (guardrail/projection.py). Polygon vertices carry
    the vertex count in param1; circles carry the radius.
    """
    proj = LocalProjection(home_lat, home_lon)
    hn, he = home_local_ne

    def ll(x, y):
        return proj.to_latlon(x - hn, y - he)

    items = []
    for f in fences:
        if f["kind"] == "polygon":
            verts = f["vertices"]
            if len(verts) < 3:
                raise ValueError(f"{f['id']}: a polygon needs >= 3 vertices")
            for x, y in verts:
                lat, lon = ll(x, y)
                items.append({"frame": MAV_FRAME_GLOBAL_RELATIVE_ALT,
                              "command": MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                              "param1": float(len(verts)), "x_lat": lat,
                              "y_long": lon, "z_alt": 0.0, "id": f["id"]})
        elif f["kind"] == "circle":
            lat, lon = ll(*f["center"])
            items.append({"frame": MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          "command": MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                          "param1": float(f["radius_m"]), "x_lat": lat,
                          "y_long": lon, "z_alt": 0.0, "id": f["id"]})
        else:
            raise ValueError(f"unknown fence kind {f['kind']!r}")
    return items


def read_parm(parm_path: str | Path) -> dict[str, float]:
    """Every NAME VALUE pair of an ArduPilot .parm file ({} if unreadable)."""
    vals: dict[str, float] = {}
    try:
        text = Path(parm_path).read_text(encoding="utf-8")
    except OSError:
        return vals
    for line in text.splitlines():
        parts = line.split("#", 1)[0].replace(",", " ").split()
        if len(parts) >= 2:
            try:
                vals[parts[0]] = float(parts[1])
            except ValueError:
                continue
    return vals


def read_fence_alt_max(parm_path: str | Path) -> float | None:
    """FENCE_ALT_MAX from an ArduPilot .parm file, or None. A 0 is returned
    as 0.0, never replaced by a default."""
    return read_parm(parm_path).get("FENCE_ALT_MAX")


# --------------------------------------------------------------------------- #
# Waypoint missions -> ArduPilot mission items (MISSION_ITEM_INT through MAVROS)
# --------------------------------------------------------------------------- #

def waypoint_item(wp: dict, home_lat: float, home_lon: float,
                  home_local_ne: tuple[float, float] = (0.0, 0.0)) -> dict:
    """One waypoint {"x" North, "y" East, "up", "hold_s"} in the policy's
    local metres -> NAV_WAYPOINT fields for mavros_msgs/Waypoint, relative
    altitude, projected about home as the fences are (`fence_items`)."""
    for k in ("x", "y", "up"):
        if not math.isfinite(float(wp[k])):
            raise ValueError(f"waypoint {k} {wp[k]!r} is not a finite number")
    hold = float(wp.get("hold_s") or 0.0)
    if not 0.0 <= hold <= MAX_HOLD_S:
        raise ValueError(f"hold_s {hold!r} outside 0..{MAX_HOLD_S:g} s")
    if float(wp["up"]) <= 0.0:
        raise ValueError(f"waypoint altitude {wp['up']!r} m is not above home")
    lat, lon = LocalProjection(home_lat, home_lon).to_latlon(
        float(wp["x"]) - home_local_ne[0], float(wp["y"]) - home_local_ne[1])
    # param4 (yaw) 0: ArduCopter ignores it for NAV_WAYPOINT and yaws by
    # WP_YAW_BEHAVIOR; a NaN would also make the adapter log invalid JSON.
    return {"frame": MAV_FRAME_GLOBAL_RELATIVE_ALT, "command": MAV_CMD_NAV_WAYPOINT,
            "autocontinue": True, "param1": hold, "param2": 0.0, "param3": 0.0,
            "param4": 0.0, "x_lat": lat, "y_long": lon,
            "z_alt": float(wp["up"])}


def mission_items(doc: dict, home_lat: float, home_lon: float,
                  home_local_ne: tuple[float, float] = (0.0, 0.0)) -> list[dict]:
    """A /shield/mission plan -> the whole mission: seq 0 home (ArduPilot
    overwrites it with its own), seq 1 DO_CHANGE_SPEED at the plan's speed,
    then the waypoints in order from FIRST_WAYPOINT_SEQ."""
    wps = doc.get("waypoints") or []
    if not wps:
        raise ValueError("a mission needs at least one waypoint")
    speed = float(doc.get("speed_mps") or 0.0)
    if not (math.isfinite(speed) and speed > 0.0):
        raise ValueError(f"speed_mps {doc.get('speed_mps')!r} is not a positive speed")
    home = {"frame": MAV_FRAME_GLOBAL, "command": MAV_CMD_NAV_WAYPOINT,
            "autocontinue": True, "param1": 0.0, "param2": 0.0, "param3": 0.0,
            "param4": 0.0, "x_lat": home_lat, "y_long": home_lon, "z_alt": 0.0}
    spd = {"frame": MAV_FRAME_MISSION, "command": MAV_CMD_DO_CHANGE_SPEED,
           "autocontinue": True, "param1": SPEED_TYPE_GROUND, "param2": speed,
           "param3": -1.0, "param4": 0.0, "x_lat": 0.0, "y_long": 0.0, "z_alt": 0.0}
    return [home, spd] + [waypoint_item(w, home_lat, home_lon, home_local_ne)
                          for w in wps]


def mission_update(doc: dict, n_items: int | None, home_lat: float, home_lon: float,
                   home_local_ne: tuple[float, float] = (0.0, 0.0)) -> tuple[int, dict]:
    """A /shield/mission_update {"seq", "waypoint"} -> (seq, item): the one
    NAV_WAYPOINT to write in place. Refused for the home and speed items and
    for a seq past the mission the autopilot holds (`n_items`, the last
    upload's count): a partial write only replaces."""
    seq = int(doc["seq"])
    if seq < FIRST_WAYPOINT_SEQ:
        raise ValueError(f"seq {seq}: only a waypoint (seq >= {FIRST_WAYPOINT_SEQ}) "
                         f"is replaced in flight")
    if n_items is None:
        raise ValueError("no mission has been uploaded to update")
    if seq >= n_items:
        raise ValueError(f"seq {seq} is past the mission's {n_items} items")
    return seq, waypoint_item(doc["waypoint"], home_lat, home_lon, home_local_ne)


def update_push(sent: list[dict], seq: int, item: dict,
                write: str = DEFAULT_MISSION_UPDATE_WRITE) -> dict:
    """The /mavros/mission/push request that puts `item` at `seq` in the
    mission the autopilot holds (`sent`: what was last written there, item
    by item): {"write", "start_index", "items", "check"}, `check` the items
    the read-back compares. "full" (MISSION_UPDATE_WRITES) writes every item
    of `sent` with `seq` replaced, from start_index 0, and checks them all;
    "partial" writes the one item at start_index = seq and checks it. A full
    write needs `sent` to hold the item it replaces."""
    if write not in MISSION_UPDATE_WRITES:
        raise ValueError(f"write {write!r} is not one of {MISSION_UPDATE_WRITES}")
    seq = int(seq)
    if write == "partial":
        return {"write": "partial", "start_index": seq, "items": [item], "check": [seq]}
    if not 0 <= seq < len(sent):
        raise ValueError(f"seq {seq}: the adapter holds {len(sent)} item(s) of the "
                         f"mission, so a full write cannot replace it")
    items = list(sent)
    items[seq] = item
    return {"write": "full", "start_index": 0, "items": items,
            "check": list(range(len(items)))}


class MissionPushQueue:
    """One mission write at a time, the updates that arrive meanwhile in
    order (2026-11-15). MAVROS 2's waypoint plugin answers a push made while
    another transfer is in progress - a push, or the read-back's pull - with
    success false and nothing sent (its state is not IDLE), so an update that
    arrived during the previous one's write or read-back was lost, while the
    mission node believed it sent. `offer(doc)` says whether to write it now;
    `done()`, called once a write has its verdict, hands over the next one
    (still busy) or None (idle)."""

    def __init__(self) -> None:
        self.busy = False
        self.waiting: list[dict] = []
        self.n_queued = 0

    def offer(self, doc: dict) -> bool:
        if self.busy:
            self.waiting.append(doc)
            self.n_queued += 1
            return False
        self.busy = True
        return True

    def done(self) -> dict | None:
        if self.waiting:
            return self.waiting.pop(0)
        self.busy = False
        return None


def same_item(a: dict, b: dict, tol_deg: float = 2e-7) -> bool:
    """Does a read-back item hold what was sent? Command, hold time and
    position (to the 1e-7 degree MISSION_ITEM_INT carries); the altitude to
    a centimetre. The home item (seq 0) is the autopilot's own: any home
    reads as the same."""
    if int(a["command"]) != int(b["command"]):
        return False
    if int(a["command"]) == MAV_CMD_DO_CHANGE_SPEED:
        return abs(float(a["param2"]) - float(b["param2"])) < 1e-3
    if int(a.get("frame", MAV_FRAME_GLOBAL_RELATIVE_ALT)) == MAV_FRAME_GLOBAL:
        return True
    return (abs(float(a["x_lat"]) - float(b["x_lat"])) <= tol_deg
            and abs(float(a["y_long"]) - float(b["y_long"])) <= tol_deg
            and abs(float(a["z_alt"]) - float(b["z_alt"])) < 0.01
            and abs(float(a["param1"]) - float(b["param1"])) < 0.5)


# --------------------------------------------------------------------------- #
# The ROS 2 node (needs rclpy + mavros_msgs; WSL venv-ros or the Orin)
# --------------------------------------------------------------------------- #

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float32MultiArray, String
    from mavros_msgs.msg import HomePosition, PositionTarget, Waypoint, WaypointList
    from mavros_msgs.srv import SetMode, WaypointPull, WaypointPush
    HAVE_ROS = True
except ImportError:                                   # Windows, CI: pure part only
    HAVE_ROS = False

LATCHED = None
if HAVE_ROS:
    LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class MavlinkAdapterNode(Node):
        def __init__(self, out: Path | None, upload_fences: bool = True,
                     hold_mode: str = DEFAULT_HOLD_MODE,
                     fences_by_gcs: bool = False,
                     mission_update_write: str = DEFAULT_MISSION_UPDATE_WRITE) -> None:
            super().__init__(ADAPTER_NODE_NAME)
            if hold_mode not in HOLD_MODES:
                raise ValueError(f"hold_mode {hold_mode!r} is not one of {HOLD_MODES}")
            if mission_update_write not in MISSION_UPDATE_WRITES:
                raise ValueError(f"mission_update_write {mission_update_write!r} is not "
                                 f"one of {MISSION_UPDATE_WRITES}")
            self.hold_mode = hold_mode
            # How a /shield/mission_update is written (MISSION_UPDATE_WRITES),
            # one at a time (MissionPushQueue).
            self.mission_update_write = mission_update_write
            self._mq = MissionPushQueue()
            for name in ("IGNORE_PX", "IGNORE_PY", "IGNORE_PZ", "IGNORE_AFX",
                         "IGNORE_AFY", "IGNORE_AFZ", "IGNORE_YAW",
                         "FRAME_LOCAL_NED"):
                if getattr(PositionTarget, name) != globals()[name]:
                    raise RuntimeError(f"PositionTarget.{name} differs from "
                                       f"this module's constant")
            self.out = out
            # A GCS-set GeoFence (--fences-by-gcs): no zone is uploaded here.
            self.fences_by_gcs = fences_by_gcs
            self.upload_fences = upload_fences and not fences_by_gcs
            self.heading_deg: float | None = None
            self.home = None            # (lat, lon, (north, east))
            self.pending_fences: dict | None = None
            self.n_sent = self.n_dropped = 0
            self.n_in = 0                # setpoint callbacks run (SETPOINT DELIVERY)
            self.n_foreign_dropped = 0
            self._publishers_seen: int | None = None
            self._log_fh = None
            self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                     self._on_pose, qos_profile_sensor_data)
            self.create_subscription(HomePosition, "/mavros/home_position/home",
                                     self._on_home, qos_profile_sensor_data)
            self.create_subscription(Float32MultiArray, TOPIC_SETPOINT,
                                     self._on_setpoint, 10)
            self.create_subscription(String, TOPIC_MODE_REQUEST,
                                     self._on_mode_request, 10)
            self.create_subscription(String, TOPIC_FENCES, self._on_fences,
                                     LATCHED)
            self.pub = self.create_publisher(PositionTarget, TOPIC_RAW_LOCAL, 10)
            self.status = self.create_publisher(String, TOPIC_STATUS, LATCHED)
            # What went out, for the Shield's delivery monitor: a report per
            # send (_on_setpoint) and a beat from this timer.
            self.sent_pub = self.create_publisher(String, TOPIC_SENT, SENT_QOS_DEPTH)
            self.create_timer(BEAT_PERIOD_S, self._beat)
            self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
            self.cli_push = self.create_client(WaypointPush,
                                               "/mavros/geofence/push")
            self.cli_pull = self.create_client(WaypointPull,
                                               "/mavros/geofence/pull")
            # The waypoint mission (MISSION_ITEM_INT through MAVROS 2).
            self.mission_n: int | None = None      # items the autopilot holds
            self.mission_sent: list[dict] = []     # what was last written
            self.mission_list: list | None = None  # /mavros/mission/waypoints
            self.pending_mission: dict | None = None
            self.create_subscription(String, TOPIC_MISSION, self._on_mission, LATCHED)
            self.create_subscription(String, TOPIC_MISSION_UPDATE,
                                     self._on_mission_update, 10)
            self.create_subscription(WaypointList, "/mavros/mission/waypoints",
                                     self._on_waypoints, LATCHED)
            self.cli_mpush = self.create_client(WaypointPush, "/mavros/mission/push")
            self.cli_mpull = self.create_client(WaypointPull, "/mavros/mission/pull")
            self.get_logger().info(
                f"MAVLink adapter up: {TOPIC_SETPOINT} (body) -> "
                f"{TOPIC_RAW_LOCAL} (FRAME_LOCAL_NED, type_mask "
                f"{TYPE_MASK_VEL_YAWRATE})")

        # ---------------- inputs ---------------- #

        def _on_pose(self, msg: PoseStamped) -> None:
            q = msg.pose.orientation
            self.heading_deg = heading_deg_from_enu_quaternion(q.x, q.y, q.z, q.w)

        def _on_home(self, msg: HomePosition) -> None:
            first = self.home is None
            # HomePosition.position is local ENU: x east, y north.
            self.home = (msg.geo.latitude, msg.geo.longitude,
                         (msg.position.y, msg.position.x))
            if first:
                self.get_logger().info(
                    f"home {msg.geo.latitude:.7f}, {msg.geo.longitude:.7f}")
            if self.pending_fences is not None:
                pend, self.pending_fences = self.pending_fences, None
                self._upload(pend)
            if self.pending_mission is not None:
                pend, self.pending_mission = self.pending_mission, None
                self._push_mission(pend)

        def _on_setpoint(self, msg: Float32MultiArray) -> None:
            self.n_in += 1
            body = list(msg.data)
            # Only the Shield flies: with a second publisher on the topic, a
            # message may be an unshielded action, so nothing is forwarded.
            n_pub = self.count_publishers(TOPIC_SETPOINT)
            if n_pub != self._publishers_seen:
                self._publishers_seen = n_pub
                if n_pub > 1:
                    self.get_logger().error(
                        f"{n_pub} publishers on {TOPIC_SETPOINT}: only the "
                        f"Shield may publish there; forwarding stopped")
                    self._status({"kind": "extra_setpoint_publisher",
                                  "publishers": n_pub,
                                  "action": "setpoints dropped until one remains"})
                elif self.n_foreign_dropped:
                    self._status({"kind": "extra_setpoint_publisher",
                                  "publishers": n_pub,
                                  "action": "forwarding resumed",
                                  "dropped": self.n_foreign_dropped})
            if n_pub > 1:
                self.n_foreign_dropped += 1
                self.n_dropped += 1
                return
            if self.heading_deg is None:
                self.n_dropped += 1
                if self.n_dropped in (1, 10, 100):
                    self.get_logger().warn(
                        f"no heading yet: dropped {self.n_dropped} setpoint(s) "
                        f"rather than rotate with a guessed one")
                return
            try:
                sp = setpoint_from_body(body, self.heading_deg)
            except ValueError as e:
                self.n_dropped += 1
                self.get_logger().warn(f"dropping setpoint: {e}")
                return
            f = sp["fields"]
            pt = PositionTarget()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.coordinate_frame = f["coordinate_frame"]
            pt.type_mask = f["type_mask"]
            pt.velocity.x, pt.velocity.y, pt.velocity.z = (
                float(v) for v in f["velocity"])
            pt.yaw_rate = float(f["yaw_rate"])
            self.pub.publish(pt)
            self.n_sent += 1
            t_sent = time.monotonic()
            self.sent_pub.publish(String(data=sent_report(self.n_sent, self.n_in,
                                                          t_sent)))
            self._log({"t": round(t_sent, 4), "body": body,
                       "heading_deg": round(self.heading_deg, 3),
                       "ned": [round(v, 4) for v in sp["ned"]],
                       "msg_enu": [round(float(v), 4) for v in f["velocity"]]
                       + [round(float(f["yaw_rate"]), 4)],
                       "sent_ned": [round(v, 4) for v in mavros_sends(f)]})

        def _log(self, rec: dict) -> None:
            if self.out is None:
                return
            if self._log_fh is None:
                # Opened lazily, on the first setpoint: by then the Shield
                # node has started the episode and moved stale files aside.
                self.out.mkdir(parents=True, exist_ok=True)
                self._log_fh = open(self.out / "adapter_log.jsonl", "a",
                                    encoding="utf-8", newline="\n")
            self._log_fh.write(json.dumps(rec) + "\n")
            self._log_fh.flush()

        def _status(self, rec: dict) -> None:
            self.status.publish(String(data=json.dumps(rec)))

        def _beat(self) -> None:
            """The 1 Hz beat on /mavlink_adapter/sent: this executor is running
            (SETPOINT DELIVERY), with the counters as they stand."""
            self.sent_pub.publish(String(data=sent_report(
                self.n_sent, self.n_in, time.monotonic(), beat=True)))

        # ---------------- mode requests ---------------- #

        def _on_mode_request(self, msg: String) -> None:
            mode, refused = mode_request(msg.data)
            if refused is not None:
                self.get_logger().warn(f"mode {mode!r} refused: not one the "
                                       f"escalation FSM can ask for")
                self._status(refused)
                return
            # The FSM's vocabulary in, ArduCopter's mode out (LOITER -> BRAKE,
            # see DEFAULT_HOLD_MODE); the status carries both.
            ap = autopilot_mode(mode, self.hold_mode)
            if not self.cli_mode.service_is_ready():
                self._status({"kind": "mode_request", "mode": mode,
                              "autopilot_mode": ap,
                              "mode_sent": False, "why": "set_mode not ready"})
                return
            fut = self.cli_mode.call_async(SetMode.Request(custom_mode=ap))

            def done(f, mode=mode, ap=ap):
                r = f.result()
                self._status({"kind": "mode_request", "mode": mode,
                              "autopilot_mode": ap,
                              "mode_sent": bool(r and r.mode_sent)})
            fut.add_done_callback(done)
            self.get_logger().info(f"SET_MODE {ap} requested (FSM: {mode})")

        # ---------------- fences ---------------- #

        def _on_fences(self, msg: String) -> None:
            try:
                doc = json.loads(msg.data)
            except ValueError:
                self.get_logger().warn("unreadable /shield/fences message")
                return
            if not self.upload_fences:
                why = ("the GeoFence is GCS-set (sitl/gcs_backstop.py, "
                       "--fences-by-gcs): this adapter uploads no zone"
                       if self.fences_by_gcs else "fence upload disabled (--no-fences)")
                self._status({"kind": "fence_upload", "success": False,
                              "generation": doc.get("generation"),
                              "fences_by_gcs": self.fences_by_gcs, "why": why})
                return
            if self.home is None:
                self.pending_fences = doc
                self.get_logger().info("fences received; waiting for home")
                return
            self._upload(doc)

        def _upload(self, doc: dict) -> None:
            lat, lon, ne = self.home
            # A geo-anchored policy flown from another home than the one the
            # Shield node declares (its --start, `home_policy_ne`): the
            # Shield's zones and the fence would sit off by the difference.
            # Refused, and the offset recorded (fence_frame).
            home_local, off, refused = fence_frame(doc, lat, lon, ne)
            if refused is not None:
                self._status({"kind": "fence_upload", "success": False,
                              "generation": doc.get("generation"), **refused})
                return
            try:
                items = fence_items(doc.get("fences") or [], lat, lon, home_local)
            except ValueError as e:
                self._status({"kind": "fence_upload", "success": False,
                              "generation": doc.get("generation"), "why": str(e)})
                return
            req = WaypointPush.Request(start_index=0)
            for it in items:
                wp = Waypoint()
                wp.frame = it["frame"]
                wp.command = it["command"]
                wp.is_current = False
                wp.autocontinue = True
                wp.param1 = it["param1"]
                wp.x_lat, wp.y_long, wp.z_alt = it["x_lat"], it["y_long"], 0.0
                req.waypoints.append(wp)
            base = {"kind": "fence_upload", "generation": doc.get("generation"),
                    "policy_hash": doc.get("policy_hash"),
                    "fence_ids": sorted({it["id"] for it in items}),
                    "items": len(items), "skipped": doc.get("skipped") or [],
                    "reason": doc.get("reason"),
                    "backstop_active": doc.get("backstop_active"),
                    "home": [lat, lon],
                    "home_local_ne": [round(home_local[0], 3), round(home_local[1], 3)]}
            if off is not None:
                base["origin_offset_m"] = [round(off[0], 2), round(off[1], 2)]
            if not self.cli_push.service_is_ready():
                self._status({**base, "success": False,
                              "why": "/mavros/geofence/push not available"})
                return
            fut = self.cli_push.call_async(req)

            def pushed(f):
                r = f.result()
                ok = bool(r and r.success)
                n = int(r.wp_transfered) if r else 0
                # A CLEAR (no item) is judged by the read-back alone: MAVROS 2
                # answers success=False to a zero-item push that the autopilot
                # takes (measured 15 Nov 2026 on the AGX against ArduPilot
                # 4.5.7 SITL: False in 54 ms, the list read back 0 items).
                clear = not items and r is not None
                if (not ok and not clear) or not self.cli_pull.service_is_ready():
                    self._status({**base, "success": ok, "transferred": n})
                    return
                fut2 = self.cli_pull.call_async(WaypointPull.Request())

                def pulled(f2):
                    r2 = f2.result()
                    got = (int(r2.wp_received) if r2 and getattr(r2, "success", True)
                           else None)
                    self._status({**base, "success": (ok or clear) and got == len(items),
                                  "transferred": n, "read_back": got,
                                  **({"push_success": ok} if clear else {})})
                    self.get_logger().info(
                        f"fence g{doc.get('generation')}: {len(items)} item(s) "
                        f"pushed, {got} read back")
                fut2.add_done_callback(pulled)
            fut.add_done_callback(pushed)

        # ---------------- waypoint missions ---------------- #

        def _on_waypoints(self, msg) -> None:
            self.mission_list = list(msg.waypoints)

        @staticmethod
        def _wp(it: dict):
            wp = Waypoint()
            wp.frame = int(it["frame"])
            wp.command = int(it["command"])
            wp.is_current = False
            wp.autocontinue = bool(it.get("autocontinue", True))
            wp.param1, wp.param2 = float(it["param1"]), float(it["param2"])
            wp.param3, wp.param4 = float(it["param3"]), float(it["param4"])
            wp.x_lat, wp.y_long = float(it["x_lat"]), float(it["y_long"])
            wp.z_alt = float(it["z_alt"])
            return wp

        @staticmethod
        def _as_dict(wp) -> dict:
            return {"frame": int(wp.frame), "command": int(wp.command),
                    "param1": float(wp.param1), "param2": float(wp.param2),
                    "x_lat": float(wp.x_lat), "y_long": float(wp.y_long),
                    "z_alt": float(wp.z_alt)}

        def _on_mission(self, msg: String) -> None:
            try:
                doc = json.loads(msg.data)
            except ValueError:
                self._status({"kind": "mission_upload", "success": False,
                              "why": "unreadable /shield/mission message"})
                return
            if self.home is None:
                self.pending_mission = doc
                self.get_logger().info("mission received; waiting for home")
                return
            self._push_mission(doc)

        def _push_mission(self, doc: dict) -> None:
            lat, lon, ne = self.home
            base = {"kind": "mission_upload", "generation": doc.get("generation"),
                    "policy_hash": doc.get("policy_hash"), "reason": doc.get("reason"),
                    "home": [lat, lon]}
            try:
                items = mission_items(doc, lat, lon, ne)
            except (KeyError, TypeError, ValueError) as e:
                self._status({**base, "success": False, "why": str(e)})
                return
            if not self.cli_mpush.service_is_ready():
                self._status({**base, "success": False,
                              "why": "/mavros/mission/push not available"})
                return
            req = WaypointPush.Request(start_index=0)
            req.waypoints = [self._wp(it) for it in items]
            base["items"] = len(items)
            self._log({"t": round(time.monotonic(), 4), "mission_push": {
                "start_index": 0, "items": items}})
            fut = self.cli_mpush.call_async(req)

            def pushed(f):
                r = f.result()
                ok = bool(r and r.success)
                n = int(r.wp_transfered) if r else 0
                if ok:
                    self.mission_n, self.mission_sent = len(items), list(items)
                self._read_back_mission({**base, "success": ok, "transferred": n},
                                        list(range(len(items))) if ok else [])
            fut.add_done_callback(pushed)
            self.get_logger().info(f"mission: {len(items)} item(s) pushed "
                                   f"({doc.get('reason')})")

        def _on_mission_update(self, msg: String) -> None:
            try:
                doc = json.loads(msg.data)
            except ValueError:
                self._status({"kind": "mission_update", "success": False,
                              "why": "unreadable /shield/mission_update message"})
                return
            # One write at a time (MissionPushQueue): MAVROS refuses a push
            # while the previous write or its read-back is in progress.
            if not self._mq.offer(doc):
                self._status({"kind": "mission_update_queued", "seq": doc.get("seq"),
                              "update_id": doc.get("update_id"),
                              "waiting": len(self._mq.waiting),
                              "why": "the previous update's write or read-back is "
                                     "in progress"})
                return
            self._write_update(doc)

        def _update_done(self) -> None:
            """An update has its verdict: the next queued one is written."""
            nxt = self._mq.done()
            if nxt is not None:
                self._write_update(nxt)

        def _write_update(self, doc: dict) -> None:
            base = {"kind": "mission_update", "seq": doc.get("seq"),
                    "generation": doc.get("generation"), "reason": doc.get("reason"),
                    "update_id": doc.get("update_id"),
                    "write": self.mission_update_write}

            def refuse(why: str) -> None:
                self._status({**base, "success": False, "why": why})
                self._update_done()
            if self.home is None:
                refuse("no home position yet")
                return
            lat, lon, ne = self.home
            try:
                seq, item = mission_update(doc, self.mission_n, lat, lon, ne)
                plan = update_push(self.mission_sent, seq, item,
                                   self.mission_update_write)
            except (KeyError, TypeError, ValueError) as e:
                refuse(str(e))
                return
            if not self.cli_mpush.service_is_ready():
                refuse("/mavros/mission/push not available")
                return
            # "full" (the default): the whole mission with the item replaced,
            # from start_index 0 (MISSION_COUNT, then every item as
            # MISSION_ITEM_INT, ACK ACCEPTED). "partial": start_index = seq,
            # which MAVROS 2.14 sends as MISSION_WRITE_PARTIAL_LIST(seq,
            # seq + 1) and ArduPilot ends in its 8 s upload timeout
            # (MISSION_UPDATE_WRITES).
            req = WaypointPush.Request(start_index=plan["start_index"])
            req.waypoints = [self._wp(it) for it in plan["items"]]
            self._log({"t": round(time.monotonic(), 4), "mission_push": {
                "write": plan["write"], "start_index": plan["start_index"],
                "items": plan["items"], "seq": seq,
                "update_id": doc.get("update_id")}})
            fut = self.cli_mpush.call_async(req)

            def pushed(f, seq=seq, item=item, plan=plan):
                r = f.result()
                ok = bool(r and r.success)
                n = int(r.wp_transfered) if r else 0
                st = {**base, "push_success": ok, "transferred": n, "item": item,
                      "items_written": len(plan["items"])}
                if not ok:
                    st["push_note"] = (
                        "MAVROS reported the partial push failed; the item is read "
                        "back to see whether the autopilot holds it (MAVROS 2.14 sends "
                        "end_index = start + count, ArduPilot reads it inclusive and "
                        "waits for one item more until its upload timeout)"
                        if plan["write"] == "partial" else
                        "MAVROS reported the full write failed; the mission is read "
                        "back to see what the autopilot holds")
                if not ok and n < 1:
                    self._status({**st, "success": False})
                    self._update_done()
                    return
                # The verdict is the autopilot's own copy of the items written.
                prev = self.mission_sent[seq] if seq < len(self.mission_sent) else None
                if seq < len(self.mission_sent):
                    self.mission_sent[seq] = item
                self._read_back_mission(st, plan["check"], restore=(seq, prev),
                                        on_done=self._update_done)
            fut.add_done_callback(pushed)
            self.get_logger().info(f"mission item {seq} replaced ({plan['write']} write, "
                                   f"{len(plan['items'])} item(s); {doc.get('reason')})")

        def _read_back_mission(self, status: dict, check: list[int],
                               restore: tuple | None = None, on_done=None) -> None:
            """Pull the mission back and compare the items in `check` with
            what was sent; the status carries the verdict. For an update the
            verdict IS the comparison (`success`); `restore` (seq, item) puts
            back what the autopilot held when the read-back shows the new item
            did not land. `on_done()` runs once the verdict is published (the
            next queued update)."""
            if not check or not self.cli_mpull.service_is_ready():
                if status.get("kind") == "mission_update":
                    status = {**status, "success": False,
                              "why": status.get("why") or "no read-back: /mavros/mission/pull "
                                                          "not available"}
                self._status(status)
                if on_done is not None:
                    on_done()
                return
            self.mission_list = None
            fut = self.cli_mpull.call_async(WaypointPull.Request())

            def pulled(f):
                r = f.result()
                got = int(r.wp_received) if r else None
                state = {"tries": 0}

                # The pulled list arrives on /mavros/mission/waypoints; wait
                # for it (up to 2 s), then compare.
                def compare():
                    state["tries"] += 1
                    lst = self.mission_list
                    if lst is None and state["tries"] < 8:
                        return
                    timer.cancel()
                    lst = lst or []
                    bad = [i for i in check
                           if i >= len(lst) or i >= len(self.mission_sent)
                           or not same_item(self.mission_sent[i], self._as_dict(lst[i]))]
                    rec = {**status, "read_back": got, "match": not bad,
                           "mismatched": bad}
                    if status.get("kind") == "mission_update":
                        rec["success"] = not bad
                        if bad and restore is not None and restore[1] is not None:
                            self.mission_sent[restore[0]] = restore[1]
                    if status.get("kind") == "mission_update" and restore is not None \
                            and restore[0] < len(lst):
                        rec["read_back_item"] = self._as_dict(lst[restore[0]])
                    self._status(rec)
                    self._log({"t": round(time.monotonic(), 4), "mission_status": rec})
                    if on_done is not None:
                        on_done()
                timer = self.create_timer(0.25, compare)
            fut.add_done_callback(pulled)

        def close(self) -> None:
            if self._log_fh is not None:
                self._log_fh.close()
            self.get_logger().info(f"adapter: {self.n_sent} setpoint(s) sent, "
                                   f"{self.n_dropped} dropped "
                                   f"({self.n_foreign_dropped} while another "
                                   f"node also published)")


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None,
                    help="episode directory for adapter_log.jsonl")
    ap.add_argument("--no-fences", action="store_true",
                    help="do not upload the policy's fences to the autopilot")
    ap.add_argument("--fences-by-gcs", action="store_true",
                    help="the GeoFence is GCS-set (sitl/gcs_backstop.py): upload no "
                         "zone, and say so in every fence status")
    ap.add_argument("--hold-mode", choices=HOLD_MODES, default=DEFAULT_HOLD_MODE,
                    help="ArduCopter mode for the FSM's LOITER: BRAKE (default; holds "
                         "with no pilot input) or LOITER (needs the throttle centred)")
    ap.add_argument("--mission-update-write", choices=MISSION_UPDATE_WRITES,
                    default=DEFAULT_MISSION_UPDATE_WRITE,
                    help="how /shield/mission_update reaches the autopilot: full "
                         "(default; the whole mission with the item replaced, "
                         "MISSION_COUNT + MISSION_ITEM_INT) or partial (one item, "
                         "MISSION_WRITE_PARTIAL_LIST, which MAVROS 2.14 sends one "
                         "past the item: ArduPilot's 8 s upload timeout follows)")
    args, ros_args = ap.parse_known_args(argv)
    if not HAVE_ROS:
        raise SystemExit("mavlink_adapter_node needs rclpy and mavros_msgs: "
                         "source /opt/ros/jazzy/setup.bash and use "
                         "~/venv-ros/bin/python")
    rclpy.init(args=ros_args)
    node = MavlinkAdapterNode(Path(args.out) if args.out else None,
                              upload_fences=not args.no_fences,
                              hold_mode=args.hold_mode,
                              fences_by_gcs=args.fences_by_gcs,
                              mission_update_write=args.mission_update_write)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:          # noqa: BLE001
        if not isinstance(e, KeyboardInterrupt):
            node.get_logger().error(f"{type(e).__name__}: {e}")
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
