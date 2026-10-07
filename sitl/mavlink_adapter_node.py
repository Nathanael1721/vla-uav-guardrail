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
guardrail.frames (from_body, then to_local_ned) and then packed the way MAVROS
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
  * /shield/fences (std_msgs/String JSON, latched) -> /mavros/geofence/push:
    the policy's keep-out zones, converted to lat/lon about the autopilot's
    home, uploaded as ArduPilot exclusion polygons / circles. With
    sitl/fence/guardrail_fence.parm (FENCE_ENABLE=1, FENCE_ACTION=1) the
    autopilot's own GeoFence is the backstop behind the Shield (Architecture
    constraints p1). Only zones whose altitude band covers the whole fence
    column are uploaded, so the backstop is never stricter than the policy,
    and a zone the aircraft is inside (or within FENCE_MARGIN of) is held back
    until it is clear, so a hot-applied zone on top of the aircraft cannot
    breach the autopilot fence the moment it arrives. A policy with a
    geographic frame origin far from the autopilot's home is refused.
  * /mavlink_adapter/status (std_msgs/String JSON, latched): every upload and
    mode request result, which the Shield node writes into events.jsonl.
  * <out>/adapter_log.jsonl: every setpoint in, the heading used, the NED
    setpoint and the ENU message out.

ONLY THE SHIELD FLIES. A setpoint is forwarded only while the Shield node is
the single publisher on /shield/setpoint: a second publisher (a VLA backend
configured to publish there) would fly unshielded actions, so its arrival
stops forwarding and is reported (`extra_setpoint_publisher`); ArduPilot then
stops in GUIDED for want of setpoints, which is the safe side.

Run (after `source /opt/ros/jazzy/setup.bash`):

    ~/venv-ros/bin/python sitl/mavlink_adapter_node.py [--out demo/out/<tag>]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail.frames import from_body, to_local_ned                     # noqa: E402
from guardrail.fsm import MODE_FOR                                       # noqa: E402
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

# The modes the adapter forwards: those the escalation FSM can ask for (Normal
# and Brake fly GUIDED; Loiter, RTL, Land), which include the mission-end LAND.
ALLOWED_MODES = frozenset(MODE_FOR.values())
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
TOPIC_RAW_LOCAL = "/mavros/setpoint_raw/local"


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

    The single body -> local-NED crossing: guardrail.frames.from_body (body to
    our North/East/up contract) then to_local_ned (up -> down). Nothing else
    in the stack rotates a command.
    """
    vx, vy, vz, wz = (float(v) for v in body[:4])
    return to_local_ned(from_body(vx, vy, vz, wz, heading_deg))


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

def _xy(p) -> tuple[float, float]:
    if isinstance(p, dict):
        return float(p["x"]), float(p["y"])
    if isinstance(p, (list, tuple)):
        return float(p[0]), float(p[1])
    return float(p.x), float(p.y)


def _point_in_polygon(x: float, y: float, verts: list) -> bool:
    inside = False
    n = len(verts)
    for i in range(n):
        (x1, y1), (x2, y2) = verts[i], verts[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


def _segment_distance(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def zone_clearance_m(fence: dict, x: float, y: float) -> float:
    """Signed horizontal distance from (x, y) to an exported zone's edge:
    positive outside, negative inside (the depth)."""
    if fence["kind"] == "circle":
        cx, cy = fence["center"]
        return math.hypot(x - cx, y - cy) - fence["radius_m"]
    verts = [tuple(v) for v in fence["vertices"]]
    d = min(_segment_distance(x, y, *verts[i], *verts[(i + 1) % len(verts)])
            for i in range(len(verts)))
    return -d if _point_in_polygon(x, y, verts) else d


def policy_fences(policy, column_m: float | None,
                  position: tuple[float, float] | None = None,
                  margin_m: float = FENCE_MARGIN_M
                  ) -> tuple[list[dict], list[dict]]:
    """The policy's keep-out zones the autopilot fence can enforce NOW.

    Returns `(fences, skipped)`. A polygon_fence, and a dynamic_nfz without
    motion (what a mid-flight zone becomes where the Shield enforces the
    grant's update model), become an exclusion polygon; a moving dynamic_nfz
    is skipped (the autopilot fence is static); a circle_fence (centre +
    radius, read by duck type) an exclusion circle. ArduPilot's polygon and
    circle fences are 2-D: they apply at
    every altitude. So a zone is exported only if its altitude band covers
    the whole column the autopilot fence guards (0 .. FENCE_ALT_MAX);
    otherwise the backstop would forbid airspace the policy allows, and
    would fire before the Shield. With `column_m` unknown nothing is
    exported. Keep-in rules (corridors) are not exported: an inclusion fence
    around a corridor would breach on the take-off pad.

    `position` is the aircraft's (x North, y East). A zone the aircraft is
    inside, or within `margin_m` of, is held back (`deferred: True` in
    `skipped`): uploading it would breach the autopilot fence at once and
    ArduPilot would RTL while the Shield is still steering out - the grant's
    GeoFence is to "fire only if the Shield itself crashes or fails to emit".
    The caller uploads it once the aircraft is clear.
    """
    fences, skipped = [], []
    for c in getattr(policy, "constraints", []):
        ctype = getattr(c, "type", "")
        if ctype not in ("polygon_fence", "circle_fence", "dynamic_nfz"):
            if ctype == "corridor":
                skipped.append({"id": c.id, "why": "keep-in corridor: not "
                                "exported as an autopilot inclusion fence"})
            continue
        if ctype == "dynamic_nfz" and getattr(c, "motion", None) is not None:
            skipped.append({"id": c.id, "why": "moving dynamic_nfz: the "
                            "autopilot fence is static, so the Shield alone "
                            "enforces it"})
            continue
        if column_m is None:
            skipped.append({"id": c.id, "why": "the autopilot's FENCE_ALT_MAX "
                            "is unknown, so whether a 2-D fence would be "
                            "stricter than the policy cannot be decided"})
            continue
        lo = float(getattr(c, "altitude_floor_m", 0.0) or 0.0)
        hi = getattr(c, "altitude_ceiling_m", None)
        hi = float("inf") if hi is None else float(hi)
        if lo > 0.0 or hi < column_m:
            skipped.append({"id": c.id, "why": (
                f"altitude band {lo:g}-{hi:g} m does not cover the fence "
                f"column 0-{column_m:g} m; a 2-D autopilot fence would be "
                f"stricter than the policy")})
            continue
        if ctype in ("polygon_fence", "dynamic_nfz"):
            f = {"id": c.id, "kind": "polygon",
                 "vertices": [list(_xy(v)) for v in c.vertices]}
        else:
            centre = getattr(c, "center", None) or getattr(c, "centre", None)
            radius = getattr(c, "radius_m", None) or getattr(c, "radius", None)
            if centre is None or not radius:
                skipped.append({"id": c.id, "why": "circle_fence without a "
                                "centre and radius this exporter can read"})
                continue
            f = {"id": c.id, "kind": "circle", "center": list(_xy(centre)),
                 "radius_m": float(radius)}
        if position is not None:
            clr = zone_clearance_m(f, float(position[0]), float(position[1]))
            if clr < margin_m:
                skipped.append({"id": c.id, "deferred": True,
                                "clearance_m": round(clr, 2), "why": (
                                    f"aircraft {'inside' if clr < 0 else 'within'}"
                                    f" {abs(clr):.1f} m of the zone (margin "
                                    f"{margin_m:g} m): deferred until clear, "
                                    f"so the upload cannot breach the "
                                    f"autopilot fence on arrival")})
                continue
        fences.append(f)
    return fences, skipped


def zones_near(fences: list[dict], x: float, y: float,
               margin_m: float = FENCE_MARGIN_M) -> list[str]:
    """Ids of exported zones that (x, y) is inside or within `margin_m` of.

    Used with the home position: a keep-out zone over home is one the
    autopilot's own RTL (the escalation FSM's, or the GeoFence's) flies back
    into. Flown 2026-10-07 with policies/poly_test.yaml, whose tri-nfz has a
    vertex on the origin: held back at take-off, uploaded once clear, then
    "Fence Breached" when the RTL brought the aircraft home."""
    return [f["id"] for f in fences
            if zone_clearance_m(f, float(x), float(y)) < margin_m]


def policy_ceiling_m(policy, fallback_m: float | None = None) -> tuple[float | None, str]:
    """The highest altitude the policy allows, and where that figure came
    from: its altitude envelopes, else `fallback_m` (the mission's cruise
    altitude), else None."""
    ceilings = [float(c.alt_max_m) for c in getattr(policy, "constraints", [])
                if getattr(c, "type", "") == "altitude_envelope"
                and getattr(c, "alt_max_m", None) is not None]
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
# The ROS 2 node (needs rclpy + mavros_msgs; WSL venv-ros or the Orin)
# --------------------------------------------------------------------------- #

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float32MultiArray, String
    from mavros_msgs.msg import HomePosition, PositionTarget, Waypoint
    from mavros_msgs.srv import SetMode, WaypointPull, WaypointPush
    HAVE_ROS = True
except ImportError:                                   # Windows, CI: pure part only
    HAVE_ROS = False

LATCHED = None
if HAVE_ROS:
    LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class MavlinkAdapterNode(Node):
        def __init__(self, out: Path | None, upload_fences: bool = True) -> None:
            super().__init__("mavlink_adapter")
            for name in ("IGNORE_PX", "IGNORE_PY", "IGNORE_PZ", "IGNORE_AFX",
                         "IGNORE_AFY", "IGNORE_AFZ", "IGNORE_YAW",
                         "FRAME_LOCAL_NED"):
                if getattr(PositionTarget, name) != globals()[name]:
                    raise RuntimeError(f"PositionTarget.{name} differs from "
                                       f"this module's constant")
            self.out = out
            self.upload_fences = upload_fences
            self.heading_deg: float | None = None
            self.home = None            # (lat, lon, (north, east))
            self.pending_fences: dict | None = None
            self.n_sent = self.n_dropped = 0
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
            self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
            self.cli_push = self.create_client(WaypointPush,
                                               "/mavros/geofence/push")
            self.cli_pull = self.create_client(WaypointPull,
                                               "/mavros/geofence/pull")
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

        def _on_setpoint(self, msg: Float32MultiArray) -> None:
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
            self._log({"t": round(time.monotonic(), 4), "body": body,
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

        # ---------------- mode requests ---------------- #

        def _on_mode_request(self, msg: String) -> None:
            mode, refused = mode_request(msg.data)
            if refused is not None:
                self.get_logger().warn(f"mode {mode!r} refused: not one the "
                                       f"escalation FSM can ask for")
                self._status(refused)
                return
            if not self.cli_mode.service_is_ready():
                self._status({"kind": "mode_request", "mode": mode,
                              "mode_sent": False, "why": "set_mode not ready"})
                return
            fut = self.cli_mode.call_async(SetMode.Request(custom_mode=mode))

            def done(f, mode=mode):
                r = f.result()
                self._status({"kind": "mode_request", "mode": mode,
                              "mode_sent": bool(r and r.mode_sent)})
            fut.add_done_callback(done)
            self.get_logger().info(f"SET_MODE {mode} requested")

        # ---------------- fences ---------------- #

        def _on_fences(self, msg: String) -> None:
            try:
                doc = json.loads(msg.data)
            except ValueError:
                self.get_logger().warn("unreadable /shield/fences message")
                return
            if not self.upload_fences:
                self._status({"kind": "fence_upload", "success": False,
                              "generation": doc.get("generation"),
                              "why": "fence upload disabled (--no-fences)"})
                return
            if self.home is None:
                self.pending_fences = doc
                self.get_logger().info("fences received; waiting for home")
                return
            self._upload(doc)

        def _upload(self, doc: dict) -> None:
            lat, lon, ne = self.home
            # A geo-anchored policy flown from another home: the Shield's
            # zones (local metres about the EKF origin) and the fence would
            # both sit off by the offset. Refused, and the offset recorded.
            org = doc.get("origin")
            off = origin_offset_m((org["lat"], org["lon"]) if org else None,
                                  lat, lon)
            if off is not None and math.hypot(*off) > ORIGIN_TOLERANCE_M:
                self._status({"kind": "fence_upload", "success": False,
                              "generation": doc.get("generation"),
                              "origin_offset_m": [round(off[0], 2), round(off[1], 2)],
                              "why": (f"the policy's frame origin is "
                                      f"{math.hypot(*off):.1f} m from the "
                                      f"autopilot's home (tolerance "
                                      f"{ORIGIN_TOLERANCE_M:g} m): the zones "
                                      f"would be uploaded in the wrong place")})
                return
            try:
                items = fence_items(doc.get("fences") or [], lat, lon, ne)
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
                    "home": [lat, lon]}
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
                if not ok or not self.cli_pull.service_is_ready():
                    self._status({**base, "success": ok, "transferred": n})
                    return
                fut2 = self.cli_pull.call_async(WaypointPull.Request())

                def pulled(f2):
                    r2 = f2.result()
                    got = int(r2.wp_received) if r2 else None
                    self._status({**base, "success": ok and got == len(items),
                                  "transferred": n, "read_back": got})
                    self.get_logger().info(
                        f"fence g{doc.get('generation')}: {len(items)} item(s) "
                        f"pushed, {got} read back")
                fut2.add_done_callback(pulled)
            fut.add_done_callback(pushed)

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
    args, ros_args = ap.parse_known_args(argv)
    if not HAVE_ROS:
        raise SystemExit("mavlink_adapter_node needs rclpy and mavros_msgs: "
                         "source /opt/ros/jazzy/setup.bash and use "
                         "~/venv-ros/bin/python")
    rclpy.init(args=ros_args)
    node = MavlinkAdapterNode(Path(args.out) if args.out else None,
                              upload_fences=not args.no_fences)
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
