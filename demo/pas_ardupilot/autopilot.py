"""The autopilot side of the rail: frame conversions and the pymavlink link.

FRAMES, CONVERTED IN ONE PLACE

  * The pilot speaks the grant's 4-D action in the BODY frame: forward, right,
    up (m/s) and yaw rate (rad/s, clockwise seen from above). Architecture
    constraints: "a = (vx, vy, vz, yaw_rate) # body-frame velocities + yaw rate".
  * The Shield works in the scene's local frame, x = North, y = East, up
    positive (guardrail/models.py), because every policy and obstacle map in
    this repository is authored there.
  * The body action is rotated into that frame ONCE, at the Shield's input
    (`body_to_world`), with the autopilot's own heading.
  * Out of the Shield, the action goes to ArduPilot as
    SET_POSITION_TARGET_LOCAL_NED velocities (`ned_setpoint`): North and East
    unchanged, up negated to NED down, yaw rate passed through in rad/s
    (MAVLink's unit, and clockwise-positive like NED).

POSITION comes from ArduPilot's EKF (GLOBAL_POSITION_INT), not from the
simulator: on the real drone there is no simulator, and the node must not
change. Latitude/longitude are projected into the scene frame about an anchor
(`FrameAnchor`): the policy's own WGS84 origin when it has one, else the
scene's home-geo-point (the simulator's NED origin, which its GPS is computed
from). Height is relative_alt, metres above the arming point.

WHY THE HEARTBEAT FILTER EXISTS

mavlink-router forwards every endpoint's traffic to every other, so this node
also receives Mission Planner's own HEARTBEAT (system 255, type GCS). pymavlink's
wait_heartbeat() latches whichever heartbeat arrives first; with Mission Planner
already open that can be the GCS, and every arm, mode and setpoint command would
then be addressed to the ground station. `PymavlinkLink.connect()` accepts only
an ArduPilot autopilot heartbeat and addresses that system explicitly.

WHY CONNECT RETRIES THE TCP CONNECT ITSELF

The node and g1_check.py connect to the router's TCP port right after starting
WSL's side (start_ardupilot.sh), and that port opens only after WSL boots, the
pin check runs (18.5 s measured on 2026-10-06), SITL starts and the router
follows. pymavlink 2.4.49's mavlink_connection() tries a refused TCP connect
retries=3 times about a second apart and then raises, so a single call failed
every --start-sitl run long before its heartbeat timeout began. connect() now
retries the connect itself until the same deadline the heartbeat wait uses.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail.manifest import (collect_host_evidence,               # noqa: E402
                                decode_custom_version,
                                describe_autopilot_version)
from guardrail.frames import from_body, to_local_ned                 # noqa: E402
from guardrail.models import Action4D, State                       # noqa: E402
from guardrail.projection import LocalProjection                     # noqa: E402

# MAVLink numbers used here, spelled out so the conversion code can be tested
# without pymavlink (values from the common and ardupilotmega dialects).
MAV_FRAME_LOCAL_NED = 1
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_TYPE_GCS = 6
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE = 512
MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
MSG_AUTOPILOT_VERSION = 148
MAV_RESULT_ACCEPTED = 0
COPTER_MODES = {"STABILIZE": 0, "AUTO": 3, "GUIDED": 4, "LOITER": 5,
                "RTL": 6, "LAND": 9}
COPTER_MODE_NAMES = {v: k for k, v in COPTER_MODES.items()}

# type_mask: use velocity + yaw_rate, ignore position, acceleration and yaw.
# bits (1 = ignore): x y z | vx vy vz | ax ay az | force yaw yaw_rate
# The same value as sitl/run_sitl_demo.py's VEL_YAWRATE_MASK (a test checks).
VEL_YAWRATE_MASK = 0b0101_1100_0111  # 1479

# Messages the node asks for, by id, and the rate (Hz).
STREAMS = {
    "GLOBAL_POSITION_INT": (33, 20.0),
    "ATTITUDE": (30, 20.0),
    "LOCAL_POSITION_NED": (32, 10.0),
    "EKF_STATUS_REPORT": (193, 2.0),
    "SERVO_OUTPUT_RAW": (36, 10.0),
    "EXTENDED_SYS_STATE": (245, 2.0),
    "SYSTEM_TIME": (2, 2.0),
    "GPS_RAW_INT": (24, 5.0),
    # The autopilot's own fence (the grant's backstop, citylife-quad.param):
    # without this stream a breach would be invisible to the node's record.
    "FENCE_STATUS": (162, 2.0),
}

# The autopilot parameters that make its GeoFence the grant's backstop, read
# back after take-off and recorded (the two SITL rails record the same set,
# plus FENCE_RADIUS: this rail's param file sets a circle fence).
FENCE_PARAMS = ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX",
                "FENCE_RADIUS", "FENCE_MARGIN", "AVOID_ENABLE")

# EKF_STATUS_REPORT flags that together mean "position estimate usable".
EKF_ATTITUDE = 1
EKF_VELOCITY_HORIZ = 2
EKF_VELOCITY_VERT = 4
EKF_POS_HORIZ_ABS = 16
EKF_POS_VERT_ABS = 32
EKF_CONST_POS_MODE = 128
EKF_READY_MASK = (EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_VELOCITY_VERT
                  | EKF_POS_HORIZ_ABS | EKF_POS_VERT_ABS)


def ekf_ready(flags: int | None) -> bool:
    """All position/velocity solutions valid and not in constant-position mode."""
    if flags is None:
        return False
    return (flags & EKF_READY_MASK) == EKF_READY_MASK and not (flags & EKF_CONST_POS_MODE)


# --------------------------------------------------------------------------- #
# Conversions
# --------------------------------------------------------------------------- #

def body_to_world(fwd: float, right: float, vz_up: float, yaw_rate: float,
                  heading_rad: float) -> Action4D:
    """Body (forward, right, up, yaw rate CW) -> scene frame (N, E, up, yaw rate).

    heading_rad is the NED heading: 0 = North, +pi/2 = East. The rotation is
    guardrail.frames.from_body, the one place the project crosses from a
    body-frame producer into the Shield's world frame (its docstring asks for
    exactly that), so this rail and tests/test_frame_contract.py share it.
    The yaw RATE passes through in rad/s, unconverted, as that module requires.
    """
    return from_body(fwd, right, vz_up, yaw_rate, math.degrees(heading_rad))


def ned_setpoint(a: Action4D) -> dict:
    """SET_POSITION_TARGET_LOCAL_NED fields for one emitted action, through
    guardrail.frames.to_local_ned (North, East, down, yaw rate in rad/s)."""
    vn, ve, vd, yr = to_local_ned(a)
    return {"coordinate_frame": MAV_FRAME_LOCAL_NED, "type_mask": VEL_YAWRATE_MASK,
            "vx": float(vn), "vy": float(ve), "vz": float(vd), "yaw_rate": float(yr)}


def enu_position_target(a: Action4D) -> dict:
    """mavros_msgs/PositionTarget fields for /mavros/setpoint_raw/local.

    MAVROS takes ENU there and converts to NED itself: velocity.x = East,
    velocity.y = North, velocity.z = up, and yaw_rate counter-clockwise
    positive, which MAVROS negates on the way to ArduPilot. coordinate_frame 1
    (FRAME_LOCAL_NED) is the MAVLink frame MAVROS sends; the ENU->NED swap is
    MAVROS's (setpoint_raw plugin), not ours.
    """
    return {"coordinate_frame": MAV_FRAME_LOCAL_NED, "type_mask": VEL_YAWRATE_MASK,
            "velocity": (float(a.vy), float(a.vx), float(a.vz_up)),
            "yaw_rate": float(-a.yaw_rate)}


class FrameAnchor:
    """Geodetic -> the scene frame the Shield and the policies work in."""

    def __init__(self, lat: float, lon: float, alt_m: float | None, source: str):
        self.proj = LocalProjection(lat, lon)
        self.lat, self.lon, self.alt_m, self.source = lat, lon, alt_m, source

    @classmethod
    def choose(cls, policy=None, scene_home=None) -> "FrameAnchor | None":
        """The policy's own origin first (the frame its rules were written
        in), else the scene's home-geo-point. None if neither exists: then
        the caller must say so rather than invent an origin."""
        origin = getattr(policy, "origin", None) if policy is not None else None
        if origin is not None:
            return cls(origin.lat, origin.lon, None, "policy.origin")
        if scene_home is not None:
            lat, lon, alt = scene_home
            return cls(lat, lon, alt, "scene.home-geo-point")
        return None

    def to_scene(self, lat: float, lon: float) -> tuple[float, float]:
        return self.proj.to_local(lat, lon)

    def to_latlon(self, x: float, y: float) -> tuple[float, float]:
        return self.proj.to_latlon(x, y)

    def to_dict(self) -> dict:
        return {"lat": self.lat, "lon": self.lon, "alt_m": self.alt_m,
                "source": self.source}


def is_autopilot_heartbeat(msg) -> bool:
    """An ArduPilot vehicle's heartbeat - not a GCS's, not a companion's."""
    try:
        return (msg.get_type() == "HEARTBEAT"
                and int(msg.autopilot) == MAV_AUTOPILOT_ARDUPILOTMEGA
                and int(msg.type) != MAV_TYPE_GCS)
    except AttributeError:
        return False


# --------------------------------------------------------------------------- #
# Snapshot of what the autopilot last said
# --------------------------------------------------------------------------- #

@dataclass
class Snapshot:
    t_pos: float | None = None          # wall time of the last position fix
    lat: float | None = None
    lon: float | None = None
    alt_msl: float | None = None        # m
    rel_alt: float | None = None        # m above home
    vn: float | None = None
    ve: float | None = None
    vd: float | None = None
    yaw: float | None = None            # rad, NED heading
    pitch: float | None = None
    roll: float | None = None
    armed: bool | None = None
    mode: str | None = None
    t_heartbeat: float | None = None
    ekf_flags: int | None = None
    servo: list = field(default_factory=list)
    t_servo: float | None = None
    landed_state: int | None = None
    time_boot_ms: int | None = None
    t_time_boot: float | None = None
    gps_fix: int | None = None
    statustext: list = field(default_factory=list)
    # FENCE_STATUS; None until one arrives (an unmeasured fence is not a
    # fence that never breached).
    fence_breach_count: int | None = None
    fence_breach_status: int | None = None
    t_fence: float | None = None
    # Where time_boot_ms came from (SYSTEM_TIME, or MAVROS's time_reference).
    fcu_clock_source: str | None = None


class PymavlinkLink:
    """ArduPilot over pymavlink. Grant-wise this is NOT the safety path (that
    is MAVROS 2; see mavros_link.py); it is the link a Windows process can run
    today, and topology.classify() labels its runs accordingly."""

    name = "pymavlink"

    def __init__(self, url: str = "tcp:127.0.0.1:5790", mavutil=None,
                 source_system: int = 1, source_component: int = 191,
                 clock=time.time, sleep=time.sleep):
        if mavutil is None:
            from pymavlink import mavutil as _m          # deferred: optional dependency
            mavutil = _m
        self.mavutil = mavutil
        self.url = url
        self.source_system = source_system
        self.source_component = source_component
        self.clock = clock
        self.sleep = sleep
        self.m = None
        self.connect_attempts = 0
        self.connect_errors: list[str] = []
        self.target_system = None
        self.target_component = None
        self.snap = Snapshot()
        self.events: list[dict] = []        # mode/arm changes, statustext, acks
        self.messages_seen: dict[str, int] = {}
        self.ignored_heartbeats = 0
        self.on_message = None              # optional hook(msg, t)
        self.vehicle_type = None            # MAV_TYPE from the autopilot's heartbeat
        self.autopilot_version: dict | None = None
        self.autopilot_kind: str | None = None   # "sitl" | "hardware" | None
        self.sim_speedup: float | None = None

    # ---- connection --------------------------------------------------- #

    def connect(self, timeout: float = 60.0, retry_s: float = 1.0) -> None:
        """Open the link and wait for the vehicle's heartbeat, both within
        `timeout`. A refused TCP connect (the router's port not open yet) is
        retried every `retry_s` until the deadline; pymavlink's own retries
        are switched off (retries=0) so this loop is the only one."""
        deadline = self.clock() + timeout
        while True:
            self.connect_attempts += 1
            try:
                self.m = self.mavutil.mavlink_connection(
                    self.url, source_system=self.source_system,
                    source_component=self.source_component, retries=0)
                break
            except OSError as exc:              # ConnectionRefusedError, timeout, ...
                self.connect_errors.append(f"{type(exc).__name__}: {exc}")
                if self.clock() + retry_s >= deadline:
                    raise TimeoutError(
                        f"could not open {self.url} within {timeout:.0f} s "
                        f"({self.connect_attempts} attempt(s); last: "
                        f"{self.connect_errors[-1]})") from exc
                self.sleep(retry_s)
        while self.clock() < deadline:
            msg = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is None:
                continue
            if is_autopilot_heartbeat(msg):
                self.target_system = msg.get_srcSystem()
                self.target_component = msg.get_srcComponent()
                self.vehicle_type = int(msg.type)
                # pymavlink's own helpers address master.target_system; point
                # it at the vehicle, not at whatever heartbeat it saw first.
                self.m.target_system = self.target_system
                self.m.target_component = self.target_component
                self._on_heartbeat(msg, self.clock())
                break
            self.ignored_heartbeats += 1
        else:
            raise TimeoutError(f"no ArduPilot heartbeat on {self.url} within "
                               f"{timeout:.0f} s ({self.ignored_heartbeats} other "
                               f"heartbeat(s) ignored)")
        self.request_streams()

    def request_streams(self) -> None:
        for _name, (msg_id, hz) in STREAMS.items():
            self.command(MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, int(1e6 / hz))

    def command(self, cmd: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self.m.mav.command_long_send(self.target_system, self.target_component,
                                     cmd, 0, *p[:7])

    # ---- receive ---------------------------------------------------------- #

    def pump(self, max_msgs: int = 500) -> int:
        """Drain what has arrived, without blocking. Returns messages read."""
        n = 0
        while n < max_msgs:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            n += 1
            self._handle(msg, self.clock())
        return n

    def wait(self, pred, timeout: float, poll: float = 0.05) -> bool:
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            self.pump()
            if pred(self.snap):
                return True
            self.sleep(poll)
        self.pump()
        return bool(pred(self.snap))

    def _from_vehicle(self, msg) -> bool:
        try:
            return msg.get_srcSystem() == self.target_system
        except AttributeError:
            return False

    def _on_heartbeat(self, msg, t: float) -> None:
        armed = bool(int(msg.base_mode) & MAV_MODE_FLAG_SAFETY_ARMED)
        mode = COPTER_MODE_NAMES.get(int(msg.custom_mode), str(int(msg.custom_mode)))
        if armed != self.snap.armed:
            self.events.append({"t": t, "kind": "armed" if armed else "disarmed"})
        if mode != self.snap.mode:
            self.events.append({"t": t, "kind": "mode", "mode": mode})
        self.snap.armed, self.snap.mode, self.snap.t_heartbeat = armed, mode, t

    def _handle(self, msg, t: float) -> None:
        kind = msg.get_type()
        if kind == "BAD_DATA":
            return
        if kind == "HEARTBEAT":
            if is_autopilot_heartbeat(msg) and self._from_vehicle(msg):
                self._on_heartbeat(msg, t)
            else:
                self.ignored_heartbeats += 1
            return
        if not self._from_vehicle(msg):
            return
        self.messages_seen[kind] = self.messages_seen.get(kind, 0) + 1
        s = self.snap
        if kind == "GLOBAL_POSITION_INT":
            s.t_pos = t
            s.lat, s.lon = msg.lat / 1e7, msg.lon / 1e7
            s.alt_msl, s.rel_alt = msg.alt / 1000.0, msg.relative_alt / 1000.0
            s.vn, s.ve, s.vd = msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0
        elif kind == "ATTITUDE":
            s.roll, s.pitch, s.yaw = msg.roll, msg.pitch, msg.yaw
        elif kind == "EKF_STATUS_REPORT":
            s.ekf_flags = int(msg.flags)
        elif kind == "SERVO_OUTPUT_RAW":
            s.servo = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
            s.t_servo = t
        elif kind == "EXTENDED_SYS_STATE":
            s.landed_state = int(msg.landed_state)
        elif kind == "SYSTEM_TIME":
            s.time_boot_ms, s.t_time_boot = int(msg.time_boot_ms), t
            s.fcu_clock_source = "SYSTEM_TIME.time_boot_ms"
        elif kind == "GPS_RAW_INT":
            s.gps_fix = int(msg.fix_type)
        elif kind == "FENCE_STATUS":
            n = int(msg.breach_count)
            if s.fence_breach_count is not None and n > s.fence_breach_count:
                self.events.append({"t": t, "kind": "fence_breach", "breach_count": n,
                                    "breach_type": int(getattr(msg, "breach_type", 0))})
            s.fence_breach_count, s.t_fence = n, t
            s.fence_breach_status = int(msg.breach_status)
        elif kind == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else str(msg.text)
            s.statustext.append(text)
            self.events.append({"t": t, "kind": "statustext", "text": text})
        elif kind == "COMMAND_ACK":
            self.events.append({"t": t, "kind": "ack", "command": int(msg.command),
                                "result": int(msg.result)})
        elif kind == "AUTOPILOT_VERSION":
            custom = decode_custom_version(list(msg.flight_custom_version))
            self.autopilot_version = {
                "ardupilot_version": describe_autopilot_version(
                    msg.flight_sw_version, custom, self.vehicle_type,
                    MAV_AUTOPILOT_ARDUPILOTMEGA),
                "ardupilot_version_source": "AUTOPILOT_VERSION (pymavlink)",
                "flight_sw_version": f"{int(msg.flight_sw_version):08x}",
                "git_hash": custom}
        if self.on_message is not None:
            self.on_message(msg, t)

    # ---- commands --------------------------------------------------------- #

    def request_mode(self, name: str) -> bool:
        """Ask for a mode ONCE and return at once: the escalation FSM's
        requests come from inside the 10 Hz loop, which must not block on the
        autopilot. Whether the mode took is read from later heartbeats."""
        self.command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                     COPTER_MODES[name])
        self.events.append({"t": self.clock(), "kind": "mode_request", "mode": name})
        return True

    def set_mode(self, name: str, timeout: float = 10.0) -> bool:
        mode_id = COPTER_MODES[name]
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            self.command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
            if self.wait(lambda s: s.mode == name, timeout=2.0):
                return True
        return False

    def arm(self, timeout: float = 60.0) -> bool:
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            self.command(MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if self.wait(lambda s: s.armed is True, timeout=3.0):
                return True
        return False

    def takeoff(self, alt_m: float, timeout: float = 60.0) -> bool:
        self.command(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m)
        return self.wait(lambda s: s.rel_alt is not None and s.rel_alt >= alt_m - 1.0,
                         timeout=timeout)

    def identify(self, timeout: float = 5.0) -> None:
        """The autopilot names itself (AUTOPILOT_VERSION) and shows whether it
        is SITL: only SITL has SIM_SPEEDUP; a hardware ArduPilot answers
        SYSID_THISMAV and not SIM_SPEEDUP. Read, never assumed - the same rule
        sitl/ros2_shield_node.py applies through MAVROS."""
        for cmd, p1 in ((MAV_CMD_REQUEST_MESSAGE, MSG_AUTOPILOT_VERSION),
                        (MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1)):
            self.command(cmd, p1)
            if self.wait(lambda s: self.autopilot_version is not None, timeout=timeout):
                break
        self.sim_speedup = self.read_param("SIM_SPEEDUP")
        if self.sim_speedup is not None:
            self.autopilot_kind = "sitl"
        elif self.read_param("SYSID_THISMAV") is not None:
            self.autopilot_kind = "hardware"

    def autopilot_fence(self) -> dict:
        """The GeoFence backstop as the AUTOPILOT holds it, not as the param
        file says (a stored or Mission Planner value may differ). Each value
        is a number, or None when the autopilot did not answer - never a
        default."""
        return {**{n: self.read_param(n) for n in FENCE_PARAMS},
                "source": "PARAM_VALUE (pymavlink)"}

    def bring_up(self, alt_m: float, ekf_timeout: float = 180.0) -> None:
        """Identify -> EKF usable -> GUIDED -> armed -> take-off to alt_m. Raises
        with the autopilot's own pre-arm messages if a step does not happen."""
        self.identify()
        if not self.wait(lambda s: ekf_ready(s.ekf_flags) and s.lat is not None,
                         timeout=ekf_timeout):
            raise RuntimeError(f"EKF never reported a usable position "
                               f"(flags {self.snap.ekf_flags}); last messages: "
                               f"{self.snap.statustext[-5:]}")
        if not self.set_mode("GUIDED"):
            raise RuntimeError("GUIDED was not accepted")
        if not self.arm():
            raise RuntimeError(f"arming refused: {self.snap.statustext[-5:]}")
        if not self.takeoff(alt_m):
            raise RuntimeError(f"take-off did not reach {alt_m} m "
                               f"(rel_alt {self.snap.rel_alt})")

    def send(self, a: Action4D, raw_body: tuple | None = None) -> dict:
        """Send the Shield's emitted action. `raw_body` is accepted for parity
        with MavrosLink (which publishes a record copy on /pas/raw_action_4d);
        MAVLink has no place for it, and the flight log records it instead."""
        f = ned_setpoint(a)
        self.m.mav.set_position_target_local_ned_send(
            int((self.clock() * 1000) % 4294967295), self.target_system,
            self.target_component, f["coordinate_frame"], f["type_mask"],
            0, 0, 0, f["vx"], f["vy"], f["vz"], 0, 0, 0, 0, f["yaw_rate"])
        return f

    def land(self) -> bool:
        return self.set_mode("LAND", timeout=5.0)

    def read_param(self, name: str, timeout: float = 3.0) -> float | None:
        """A parameter read, or None. Used for SIM_SPEEDUP, whose presence
        also tells SITL from a real autopilot, and for the GeoFence values.

        Everything else that arrives while waiting goes through _handle():
        pymavlink's recv_match(type=...) DISCARDS the messages it skips, so a
        typed wait would lose a mode change, a STATUSTEXT or a fence breach
        that happened to arrive during a parameter read."""
        self.m.mav.param_request_read_send(self.target_system, self.target_component,
                                           name.encode(), -1)
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            msg = self.m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() != "PARAM_VALUE":
                self._handle(msg, self.clock())
                continue
            if not self._from_vehicle(msg):
                continue
            pid = msg.param_id
            pid = pid.decode() if isinstance(pid, bytes) else pid
            if pid.strip("\x00") == name:
                return round(float(msg.param_value), 4)
        return None

    # ---- state for the Shield --------------------------------------------- #

    def state(self, anchor: FrameAnchor) -> State | None:
        s = self.snap
        if s.lat is None or s.rel_alt is None:
            return None
        x, y = anchor.to_scene(s.lat, s.lon)
        return State(x=x, y=y, up=s.rel_alt,
                     yaw_deg=math.degrees(s.yaw or 0.0))

    def evidence(self) -> dict:
        """What the link saw, and the topology evidence in the shape
        guardrail.manifest.check_topology_evidence reads. There are no MAVROS
        keys: this link is not MAVROS, and the check says so."""
        connected = (self.snap.t_heartbeat is not None
                     and self.clock() - self.snap.t_heartbeat < 5.0)
        ver = self.autopilot_version or {}
        tev = {"ardupilot_version": ver.get("ardupilot_version"),
               "ardupilot_version_source": ver.get("ardupilot_version_source"),
               "autopilot_kind": self.autopilot_kind,
               "network_link": {"pymavlink_url": self.url},
               **collect_host_evidence()}
        return {"link": self.name, "url": self.url,
                "heartbeat": self.target_system is not None,
                "is_ardupilot": self.target_system is not None,
                "fcu_connected_at_end": bool(connected),
                "target_system": self.target_system,
                "target_component": self.target_component,
                "ignored_heartbeats": self.ignored_heartbeats,
                "connect_attempts": self.connect_attempts,
                "autopilot_kind": self.autopilot_kind,
                "sim_speedup": self.sim_speedup,
                "flight_sw_version": ver.get("flight_sw_version"),
                "messages_seen": dict(self.messages_seen),
                "topology_evidence": tev}
