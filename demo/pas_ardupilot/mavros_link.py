"""ArduPilot over MAVROS 2 - the grant's safety path - and a ROS 2 camera source.

The grant's interface table (Architecture constraints, p3):

    VLA -> Shield        ROS 2 topic, std_msgs/Float32MultiArray (4-D action), 10 Hz
    Shield -> MAVROS 2   ROS 2 topic, mavros_msgs/PositionTarget, 10-50 Hz
    MAVROS 2 -> ArduPilot  MAVLink SET_POSITION_TARGET_LOCAL_NED

So the Shield's output goes out on /mavros/setpoint_raw/local as a
PositionTarget (not the geometry_msgs/Twist that sitl/ros2_shield_node.py
publishes).

THE VLA -> SHIELD HOP IS IN-PROCESS ON THIS NODE. The pilot, the Shield and
this link share one process, so the table's first row is a function call here,
not a topic. Record copies of the raw and the filtered 4-D actions are
published as Float32MultiArray on /pas/raw_action_4d and /pas/safe_action_4d -
deliberately NOT on /vla/action_4d, which is sitl/ros2_shield_node.py's input:
if that node (and sitl/mavlink_adapter_node.py, which also writes
/mavros/setpoint_raw/local) were up on the same ROS graph, a raw action
published there would be filtered by a second Shield and two Shields would
command one aircraft. A rosbag2 of /pas/* shows what this node's pilot asked
for and what its Shield let through; it is not the grant's topic chain.

ONE NODE, THREE TOPOLOGIES, NO CODE CHANGE

    dev     this node in WSL, MAVROS in WSL, fcu_url udp://:14555@
    hil     this node and MAVROS on the Jetson Orin, fcu_url
            udp://:14555@<desktop-ip>:14555, fed by mavlink-router's UDP output
            on the desktop. This is the reference repository's layout, the one
            docs/DESIGN-topologies.md chose, and the one guardrail.manifest's
            hil evidence check accepts (it requires a non-loopback fcu_url
            remote). The grant's table words it "Desktop runs sims + MAVROS +
            GCS"; that variant needs MAVROS in WSL with mirrored networking and
            Cyclone DDS peers across the cable, and a manifest.py change -
            docs/DESIGN-projectairsim-ardupilot.md records the difference.
    flight  this node on the Orin, MAVROS on the Orin with fcu_url pointing at
            the flight controller's serial port, camera from a ROS 2 image topic

Only MAVROS's launch arguments and --camera change between them.

WHAT THE AUTOPILOT DOES ON ITS OWN is recorded too: /mavros/state (mode, arm),
/mavros/extended_state (landed), /mavros/statustext/recv (ArduPilot's
messages) and /mavros/time_reference (the FCU clock, from SYSTEM_TIME, which
gives the real-time factor). MAVROS 2 publishes no FENCE_STATUS, so on this
link a fence breach shows only as the autopilot leaving GUIDED on its own (the
node records that as a takeover) and the fence breach count stays None, never 0.

rclpy and the message classes are injectable (`rclpy=`, `msgs=`) so the tests
can drive this module with stand-ins; nothing here is exercised against a live
MAVROS by the test suite.
"""
from __future__ import annotations

import math
import os
import time
from pathlib import Path
import sys
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for _p in (ROOT, HERE):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.manifest import (collect_host_evidence,               # noqa: E402
                                decode_custom_version,
                                describe_autopilot_version,
                                network_link_evidence)
from guardrail.models import Action4D, State                         # noqa: E402

from autopilot import (COPTER_MODES, FENCE_PARAMS, FrameAnchor,      # noqa: E402
                       Snapshot, enu_position_target)

TOPIC_SETPOINT = "/mavros/setpoint_raw/local"
# Record copies only; see the module docstring for why these are not the
# grant's /vla/action_4d.
TOPIC_RAW_ACTION = "/pas/raw_action_4d"
TOPIC_SAFE_ACTION = "/pas/safe_action_4d"
LANDED_STATE_ON_GROUND = 1


def load_msgs() -> SimpleNamespace:
    """The ROS 2 message and service types this module uses."""
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import ExtendedState, PositionTarget, RCOut, StatusText
    from mavros_msgs.msg import State as MavState
    from mavros_msgs.srv import (CommandBool, CommandTOL, SetMode, StreamRate,
                                 VehicleInfoGet)
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, NavSatFix, TimeReference
    from std_msgs.msg import Float32MultiArray, Float64
    return SimpleNamespace(
        PoseStamped=PoseStamped, PositionTarget=PositionTarget, RCOut=RCOut,
        MavState=MavState, CommandBool=CommandBool, CommandTOL=CommandTOL,
        SetMode=SetMode, StreamRate=StreamRate, GetParameters=GetParameters,
        VehicleInfoGet=VehicleInfoGet, qos_sensor=qos_profile_sensor_data,
        Image=Image, NavSatFix=NavSatFix, Float32MultiArray=Float32MultiArray,
        Float64=Float64, ExtendedState=ExtendedState, StatusText=StatusText,
        TimeReference=TimeReference)


def enu_quat_to_ned_yaw(q) -> float:
    """MAVROS pose orientation (ENU, base_link FLU) -> NED heading, rad.

    ENU yaw is measured from East, counter-clockwise; NED heading from North,
    clockwise. heading = pi/2 - yaw_enu, wrapped to (-pi, pi].
    """
    yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    h = math.pi / 2.0 - yaw_enu
    return math.atan2(math.sin(h), math.cos(h))


class MavrosLink:
    name = "mavros"

    def __init__(self, node=None, rclpy=None, msgs=None, clock=time.time,
                 node_name: str = "pas_perception_node"):
        if rclpy is None:
            import rclpy as _rclpy                  # deferred: only on a ROS host
            rclpy = _rclpy
        self.rclpy = rclpy
        self.msgs = msgs or load_msgs()
        self.clock = clock
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node(node_name)
        self.node = node
        self.snap = Snapshot()
        self.events: list[dict] = []
        self.connected = None
        self.state_msgs = 0
        self.sim_speedup: float | None = None
        # The evidence check_topology_evidence() reads, in the keys
        # sitl/ros2_shield_node.py records, gathered from the live system.
        self.hil_evidence: dict = {
            "ros_distro": os.environ.get("ROS_DISTRO"), "mavros_node": None,
            "fcu_connected": None, "ardupilot_version": None,
            "ardupilot_version_source": None, "autopilot_kind": None,
            "network_link": None, **collect_host_evidence()}
        m = self.msgs
        node.create_subscription(m.MavState, "/mavros/state", self._on_state, 10)
        node.create_subscription(m.NavSatFix, "/mavros/global_position/global",
                                 self._on_global, m.qos_sensor)
        node.create_subscription(m.Float64, "/mavros/global_position/rel_alt",
                                 self._on_rel_alt, m.qos_sensor)
        node.create_subscription(m.PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, m.qos_sensor)
        node.create_subscription(m.RCOut, "/mavros/rc/out", self._on_rc_out, 10)
        node.create_subscription(m.ExtendedState, "/mavros/extended_state",
                                 self._on_ext, 10)
        node.create_subscription(m.StatusText, "/mavros/statustext/recv",
                                 self._on_text, 10)
        node.create_subscription(m.TimeReference, "/mavros/time_reference",
                                 self._on_time_ref, 10)
        self.pub_sp = node.create_publisher(m.PositionTarget, TOPIC_SETPOINT, 10)
        self.pub_raw = node.create_publisher(m.Float32MultiArray, TOPIC_RAW_ACTION, 10)
        self.pub_safe = node.create_publisher(m.Float32MultiArray, TOPIC_SAFE_ACTION, 10)
        self.cli_mode = node.create_client(m.SetMode, "/mavros/set_mode")
        self.cli_arm = node.create_client(m.CommandBool, "/mavros/cmd/arming")
        self.cli_tol = node.create_client(m.CommandTOL, "/mavros/cmd/takeoff")
        self.cli_rate = node.create_client(m.StreamRate, "/mavros/set_stream_rate")
        self.cli_param = node.create_client(m.GetParameters,
                                            "/mavros/param/get_parameters")
        self.cli_mavros_param = node.create_client(m.GetParameters,
                                                   "/mavros/get_parameters")
        self.cli_info = node.create_client(m.VehicleInfoGet, "/mavros/vehicle_info_get")

    # ---- subscriptions ---------------------------------------------------- #

    def _on_state(self, msg) -> None:
        t = self.clock()
        self.state_msgs += 1
        self.connected = bool(msg.connected)
        # Receiving /mavros/state at all shows a MAVROS node on the graph;
        # msg.connected is MAVROS's own answer about the flight controller.
        self.hil_evidence["mavros_node"] = "/mavros"
        self.hil_evidence["fcu_connected"] = self.connected
        if bool(msg.armed) != self.snap.armed:
            self.events.append({"t": t, "kind": "armed" if msg.armed else "disarmed"})
        if msg.mode != self.snap.mode:
            self.events.append({"t": t, "kind": "mode", "mode": msg.mode})
        self.snap.armed, self.snap.mode, self.snap.t_heartbeat = bool(msg.armed), msg.mode, t

    def _on_global(self, msg) -> None:
        self.snap.t_pos = self.clock()
        self.snap.lat, self.snap.lon = float(msg.latitude), float(msg.longitude)
        # /global carries ellipsoid height (MAVROS converts from AMSL); kept for
        # the record only - the Shield uses rel_alt.
        self.snap.alt_msl = float(msg.altitude)

    def _on_rel_alt(self, msg) -> None:
        self.snap.rel_alt = float(msg.data)

    def _on_pose(self, msg) -> None:
        self.snap.yaw = enu_quat_to_ned_yaw(msg.pose.orientation)

    def _on_rc_out(self, msg) -> None:
        self.snap.servo = list(msg.channels[:4])
        self.snap.t_servo = self.clock()

    def _on_ext(self, msg) -> None:
        self.snap.landed_state = int(msg.landed_state)

    def _on_text(self, msg) -> None:
        text = str(msg.text)
        self.snap.statustext.append(text)
        self.events.append({"t": self.clock(), "kind": "statustext", "text": text,
                            "severity": int(getattr(msg, "severity", -1))})

    def _on_time_ref(self, msg) -> None:
        """MAVROS's sys_time plugin republishes SYSTEM_TIME.time_unix_usec as
        time_ref. On SITL that clock advances with simulated time, so its
        progress against the wall clock is ArduPilot's real-time factor - the
        number that tells whether this run met the grant's sim_speedup 1.0."""
        ref = msg.time_ref
        self.snap.time_boot_ms = int(ref.sec) * 1000 + int(ref.nanosec) // 1_000_000
        self.snap.t_time_boot = self.clock()
        self.snap.fcu_clock_source = ("SYSTEM_TIME.time_unix_usec via "
                                      "/mavros/time_reference")

    # ---- spinning ------------------------------------------------------- #

    def pump(self, max_spins: int = 20) -> int:
        n = 0
        for _ in range(max_spins):
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            n += 1
        return n

    def wait(self, pred, timeout: float, poll: float = 0.05) -> bool:
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=poll)
            if pred(self):
                return True
        return bool(pred(self))

    def _call(self, client, req, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    # ---- commands ------------------------------------------------------- #

    def _get_params(self, client, names: list[str]) -> dict:
        """ROS 2 parameters by name; a string, a number, or None when unset.
        MAVROS reports SIM_SPEEDUP as INTEGER with the value in double_value,
        so a numeric value is taken from whichever field carries one."""
        try:
            res = self._call(client, self.msgs.GetParameters.Request(names=names), 5.0)
        except Exception:                                     # noqa: BLE001
            return {}
        out: dict = {}
        if res is not None:
            for n, v in zip(names, res.values):
                if v.type == 0:                       # PARAMETER_NOT_SET
                    out[n] = None
                elif v.type == 4:                     # PARAMETER_STRING
                    out[n] = v.string_value
                else:
                    out[n] = float(v.double_value) or float(v.integer_value)
        return out

    def identify(self, timeout: float = 20.0) -> None:
        """Name the autopilot, tell SITL from hardware, and record the link -
        as sitl/ros2_shield_node.py does, so both rails' evidence is judged by
        the same check_topology_evidence()."""
        m = self.msgs
        deadline = self.clock() + timeout
        while self.clock() < deadline and not self.hil_evidence["ardupilot_version"]:
            res = self._call(self.cli_info, m.VehicleInfoGet.Request(
                sysid=0, compid=0, get_all=False), 5.0)
            if res is not None and res.success and res.vehicles:
                v = res.vehicles[0]
                if v.available_info & 2:              # HAVE_INFO_AUTOPILOT_VERSION
                    custom = decode_custom_version(v.flight_custom_version)
                    self.hil_evidence.update(
                        ardupilot_version=describe_autopilot_version(
                            v.flight_sw_version, custom, v.type, v.autopilot),
                        ardupilot_version_source="AUTOPILOT_VERSION via "
                                                 "/mavros/vehicle_info_get",
                        flight_sw_version=f"{int(v.flight_sw_version):08x}")
                    break
            self.rclpy.spin_once(self.node, timeout_sec=0.5)
        self.sim_speedup = self.read_sim_speedup(timeout=timeout)
        url = self._get_params(self.cli_mavros_param, ["fcu_url"]).get("fcu_url")
        self.hil_evidence["network_link"] = {**network_link_evidence(url),
                                             "source": "/mavros fcu_url parameter"
                                             if url else None}

    def autopilot_fence(self, timeout: float = 20.0) -> dict:
        """The GeoFence parameters as the autopilot holds them, through
        /mavros/param. Polled until ALL of them answer: MAVROS fills its
        parameter table while it pulls the list, and sitl/ros2_shield_node.py
        once read FENCE_ENABLE before FENCE_ALT_MAX existed. What never
        answered stays None."""
        deadline = self.clock() + timeout
        vals: dict = {}
        while self.clock() < deadline:
            vals = self._get_params(self.cli_param, list(FENCE_PARAMS))
            if all(vals.get(k) is not None for k in FENCE_PARAMS):
                break
            self.rclpy.spin_once(self.node, timeout_sec=0.5)
        return {**{n: vals.get(n) for n in FENCE_PARAMS},
                "source": "/mavros/param/get_parameters"}

    def bring_up(self, alt_m: float, timeout: float = 180.0) -> None:
        m = self.msgs
        if not self.wait(lambda s: s.connected, timeout=timeout):
            raise RuntimeError("MAVROS never reported a connected FCU")
        self.identify()
        self._call(self.cli_rate, m.StreamRate.Request(stream_id=0, message_rate=10,
                                                       on_off=True))
        if not self.wait(lambda s: s.snap.lat is not None and s.snap.rel_alt is not None,
                         timeout=timeout):
            raise RuntimeError("no global position from MAVROS (EKF not ready?)")
        for name, req, ok in (
                ("GUIDED", (self.cli_mode, m.SetMode.Request(custom_mode="GUIDED")),
                 lambda s: s.snap.mode == "GUIDED"),
                ("arm", (self.cli_arm, m.CommandBool.Request(value=True)),
                 lambda s: s.snap.armed is True)):
            deadline = self.clock() + timeout
            while self.clock() < deadline:
                self._call(*req)
                if self.wait(ok, timeout=3.0):
                    break
            else:
                raise RuntimeError(f"{name} was not accepted")
        self._call(self.cli_tol, m.CommandTOL.Request(altitude=float(alt_m)))
        if not self.wait(lambda s: (s.snap.rel_alt or 0.0) >= alt_m - 1.0, timeout=60.0):
            raise RuntimeError(f"take-off did not reach {alt_m} m")

    def send(self, a: Action4D, raw_body: tuple | None = None) -> dict:
        f = enu_position_target(a)
        msg = self.msgs.PositionTarget()
        msg.coordinate_frame = f["coordinate_frame"]
        msg.type_mask = f["type_mask"]
        msg.velocity.x, msg.velocity.y, msg.velocity.z = f["velocity"]
        msg.yaw_rate = f["yaw_rate"]
        self.pub_sp.publish(msg)
        safe = self.msgs.Float32MultiArray()
        safe.data = [float(a.vx), float(a.vy), float(a.vz_up), float(a.yaw_rate)]
        self.pub_safe.publish(safe)
        if raw_body is not None:
            raw = self.msgs.Float32MultiArray()
            raw.data = [float(v) for v in raw_body]
            self.pub_raw.publish(raw)
        return f

    def request_mode(self, name: str) -> bool:
        """Ask MAVROS for a mode and return at once (the escalation FSM calls
        this from inside the 10 Hz loop). False when /mavros/set_mode is not
        there to ask; whether the mode took is read from /mavros/state."""
        if name not in COPTER_MODES:
            raise ValueError(f"{name!r} is not an ArduCopter mode this rail uses")
        ready = self.cli_mode.wait_for_service(timeout_sec=0.0)
        if ready:
            self.cli_mode.call_async(self.msgs.SetMode.Request(custom_mode=name))
        self.events.append({"t": self.clock(), "kind": "mode_request", "mode": name,
                            "via": "/mavros/set_mode" if ready
                            else "nobody: /mavros/set_mode not ready"})
        return bool(ready)

    def land(self) -> bool:
        res = self._call(self.cli_mode, self.msgs.SetMode.Request(custom_mode="LAND"))
        return bool(res and getattr(res, "mode_sent", False))

    def read_sim_speedup(self, timeout: float = 20.0) -> float | None:
        """SIM_SPEEDUP through /mavros/param's ROS 2 parameters, polled until
        positive (MAVROS fills them itself after FCU connect). Sets
        autopilot_kind: "sitl" when SIM_SPEEDUP answers, "hardware" when
        SYSID_THISMAV does and SIM_SPEEDUP is unset, else None."""
        deadline = self.clock() + timeout
        vals: dict = {}
        while self.clock() < deadline:
            vals = self._get_params(self.cli_param, ["SIM_SPEEDUP", "SYSID_THISMAV"])
            sp = vals.get("SIM_SPEEDUP")
            if isinstance(sp, float) and sp > 0.0:
                self.hil_evidence["autopilot_kind"] = "sitl"
                return round(sp, 4)
            self.rclpy.spin_once(self.node, timeout_sec=0.5)
        hardware = (bool(vals.get("SYSID_THISMAV")) and "SIM_SPEEDUP" in vals
                    and vals["SIM_SPEEDUP"] is None)
        self.hil_evidence["autopilot_kind"] = "hardware" if hardware else None
        return None

    # ---- state for the Shield -------------------------------------------- #

    def state(self, anchor: FrameAnchor) -> State | None:
        s = self.snap
        if s.lat is None or s.rel_alt is None:
            return None
        x, y = anchor.to_scene(s.lat, s.lon)
        return State(x=x, y=y, up=s.rel_alt, yaw_deg=math.degrees(s.yaw or 0.0))

    def evidence(self) -> dict:
        ardupilot_mode = self.snap.mode in COPTER_MODES if self.snap.mode else False
        return {"link": self.name,
                "heartbeat": bool(self.connected),
                # /mavros/state has no autopilot field; ArduCopter's mode names
                # (GUIDED, LOITER, ...) are the evidence - PX4's are OFFBOARD,
                # POSCTL, AUTO.LOITER.
                "is_ardupilot": bool(self.connected and ardupilot_mode),
                "autopilot_kind": self.hil_evidence.get("autopilot_kind"),
                "sim_speedup": self.sim_speedup,
                "fence_status": "not available over MAVROS 2 (no FENCE_STATUS "
                                "topic); a fence action shows as a takeover",
                "fcu_clock_source": self.snap.fcu_clock_source,
                "topology_evidence": dict(self.hil_evidence)}


class Ros2ImageSource:
    """sensor_msgs/Image -> the message dict SemanticObs.put_front() takes.

    The flight topology's camera: whatever driver publishes the topic. bgr8 and
    rgb8 are accepted; anything else is refused loudly once rather than decoded
    into a wrong-coloured image, because the colour gate would then reject the
    right target in silence.
    """

    def __init__(self, node, obs, topic: str = "/camera/image_raw", msgs=None):
        self.obs = obs
        self.msgs = msgs or load_msgs()
        self.n = 0
        self.refused: str | None = None
        node.create_subscription(self.msgs.Image, topic, self._on_image,
                                 self.msgs.qos_sensor)

    @staticmethod
    def to_frame_dict(msg) -> dict:
        enc = str(msg.encoding).lower()
        data = bytes(msg.data)
        if enc == "rgb8":
            import numpy as np
            arr = np.frombuffer(data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            data = arr[:, :, ::-1].tobytes()
        elif enc != "bgr8":
            raise ValueError(f"image encoding {msg.encoding!r} is not bgr8/rgb8")
        stamp = getattr(msg.header, "stamp", None)
        ns = (int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)) if stamp else None
        return {"data": data, "encoding": "BGR", "height": int(msg.height),
                "width": int(msg.width), "time_stamp": ns}

    def _on_image(self, msg) -> None:
        if self.refused:
            return
        try:
            self.obs.put_front(self.to_frame_dict(msg))
            self.n += 1
        except ValueError as exc:
            self.refused = str(exc)
            print(f"[camera] *** {exc}; frames from this topic are ignored")
