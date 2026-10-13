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

WHAT THE PYMAVLINK LINK DOES, THIS ONE DOES TOO (2026-10-12): the body pitch
and roll from /mavros/local_position/pose (the depth rules read the pitch; it
was always None here, so they judged every box at level), the GeoFence zones
through /mavros/geofence/push and a read-back through /mavros/geofence/pull,
the throttle override for the literal LOITER hold on /mavros/rc/override
(with /mavros/rc/in to see it took), the fcu_url from the node MAVROS 2
declares it on (/mavros_node; then /mavros, then the router's fcu_urls array -
the manifest's dev label beside this rail's scene needs it, and every stored
ROS 2 run found none on /mavros), else the launch script's MAVROS_FCU_URL,
the fence list held before the upload (`read_fence`), and the hil evidence
sitl/ros2_shield_node.py
records beside it (which processes run on this host, and so where MAVROS
runs), without which an Orin node's run behind mavlink-router
(`udp://:14555@`) could never show the autopilot runs elsewhere. MAVROS
sends an RC override as its own system id (its `system_id` parameter), and
ArduPilot takes one only from SYSID_MYGCS: the plan reads both and says
whether they match.

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
                                mavros_location, network_link_evidence,
                                scan_local_processes)
from guardrail.models import Action4D, State                         # noqa: E402

from autopilot import (COPTER_MODES, FENCE_PARAMS, RC_OVERRIDE_IGNORE,  # noqa: E402
                       RC_OVERRIDE_RELEASE, FrameAnchor, Snapshot,
                       enu_position_target)

TOPIC_SETPOINT = "/mavros/setpoint_raw/local"
# Record copies only; see the module docstring for why these are not the
# grant's /vla/action_4d.
TOPIC_RAW_ACTION = "/pas/raw_action_4d"
TOPIC_SAFE_ACTION = "/pas/safe_action_4d"
# The literal LOITER hold's throttle (created on first use, so a BRAKE run
# publishes nothing there).
TOPIC_RC_OVERRIDE = "/mavros/rc/override"
LANDED_STATE_ON_GROUND = 1


def load_msgs() -> SimpleNamespace:
    """The ROS 2 message and service types this module uses."""
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import (ExtendedState, OverrideRCIn, PositionTarget, RCIn,
                                 RCOut, StatusText, Waypoint)
    from mavros_msgs.msg import State as MavState
    from mavros_msgs.srv import (CommandBool, CommandTOL, SetMode, StreamRate,
                                 VehicleInfoGet, WaypointPull, WaypointPush)
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, NavSatFix, TimeReference
    from std_msgs.msg import Float32MultiArray, Float64
    return SimpleNamespace(
        PoseStamped=PoseStamped, PositionTarget=PositionTarget, RCOut=RCOut,
        RCIn=RCIn, OverrideRCIn=OverrideRCIn, Waypoint=Waypoint,
        WaypointPush=WaypointPush, WaypointPull=WaypointPull,
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


def enu_quat_to_ned_pitch_roll(q) -> tuple[float, float]:
    """MAVROS pose orientation (ENU, base_link FLU) -> (pitch, roll) in the
    NED / FRD convention MAVLink's ATTITUDE uses, rad: nose up and right
    wing down positive. The ZYX angles of the FLU body in ENU have the same
    roll and the opposite pitch (FLU's y axis points left, so a nose-up
    rotation is negative about it)."""
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    s = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    return -math.asin(s), roll


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
        node.create_subscription(m.RCIn, "/mavros/rc/in", self._on_rc_in, 10)
        self.pub_sp = node.create_publisher(m.PositionTarget, TOPIC_SETPOINT, 10)
        self.pub_raw = node.create_publisher(m.Float32MultiArray, TOPIC_RAW_ACTION, 10)
        self.pub_safe = node.create_publisher(m.Float32MultiArray, TOPIC_SAFE_ACTION, 10)
        self.pub_rc = None
        self.cli_fence_push = node.create_client(m.WaypointPush, "/mavros/geofence/push")
        self.cli_fence_pull = node.create_client(m.WaypointPull, "/mavros/geofence/pull")
        self.cli_mode = node.create_client(m.SetMode, "/mavros/set_mode")
        self.cli_arm = node.create_client(m.CommandBool, "/mavros/cmd/arming")
        self.cli_tol = node.create_client(m.CommandTOL, "/mavros/cmd/takeoff")
        self.cli_rate = node.create_client(m.StreamRate, "/mavros/set_stream_rate")
        self.cli_param = node.create_client(m.GetParameters,
                                            "/mavros/param/get_parameters")
        self.cli_mavros_param = node.create_client(m.GetParameters,
                                                   "/mavros/get_parameters")
        # MAVROS 2's mavros_node executable is three nodes: /mavros_node
        # declares fcu_url (what `-p fcu_url:=...` sets) and hands it to the
        # router /mavros_router as the string array fcu_urls; the UAS node
        # /mavros has none, which is why every stored ROS 2 rail run
        # (ros2fix2_*, ros2_shield_*) found none there and fell back to the
        # launch script's MAVROS_FCU_URL. Names from MAVROS 2's mavros_node.cpp
        # as recalled (review 2026-10-13), not yet read off a live MAVROS.
        self.cli_node_param = node.create_client(m.GetParameters,
                                                 "/mavros_node/get_parameters")
        self.cli_router_param = node.create_client(m.GetParameters,
                                                   "/mavros_router/get_parameters")
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
        q = msg.pose.orientation
        self.snap.yaw = enu_quat_to_ned_yaw(q)
        self.snap.pitch, self.snap.roll = enu_quat_to_ned_pitch_roll(q)

    def _on_rc_out(self, msg) -> None:
        self.snap.servo = list(msg.channels[:4])
        self.snap.t_servo = self.clock()

    def _on_rc_in(self, msg) -> None:
        ch = list(msg.channels)
        if len(ch) >= 3:
            self.snap.rc3, self.snap.t_rc = int(ch[2]), self.clock()

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

    def _get_params(self, client, names: list[str], timeout: float = 5.0) -> dict:
        """ROS 2 parameters by name; a string, a number, a list (an array
        parameter, such as the router's fcu_urls), or None when unset. MAVROS
        reports SIM_SPEEDUP as INTEGER with the value in double_value, so a
        numeric value is taken from whichever field carries one."""
        try:
            res = self._call(client, self.msgs.GetParameters.Request(names=names),
                             timeout)
        except Exception:                                     # noqa: BLE001
            return {}
        out: dict = {}
        if res is not None:
            for n, v in zip(names, res.values):
                if v.type == 0:                       # PARAMETER_NOT_SET
                    out[n] = None
                elif v.type == 4:                     # PARAMETER_STRING
                    out[n] = v.string_value
                elif v.type == 9:                     # PARAMETER_STRING_ARRAY
                    out[n] = [str(x) for x in (getattr(v, "string_array_value",
                                                       None) or [])]
                elif v.type in (5, 6, 7, 8):          # the other arrays
                    out[n] = list(getattr(v, {5: "byte_array_value",
                                              6: "bool_array_value",
                                              7: "integer_array_value",
                                              8: "double_array_value"}[v.type],
                                          None) or [])
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
        # The link MAVROS flies over, and which MAVLink actors run on THIS
        # host - as sitl/ros2_shield_node.py read_link() records them: where
        # SITL and MAVROS are decides dev vs hil, whatever the URL says.
        url = src = None
        for cli, name, key in ((self.cli_node_param, "/mavros_node", "fcu_url"),
                               (self.cli_mavros_param, "/mavros", "fcu_url"),
                               (self.cli_router_param, "/mavros_router", "fcu_urls")):
            got = self._get_params(cli, [key], timeout=2.0).get(key)
            if isinstance(got, list):         # the router's string array
                got = next((u for u in got if u), None)
            if isinstance(got, str) and got:
                url, src = got, f"{name} {key} parameter"
                break
        if not url:
            url, src = os.environ.get("MAVROS_FCU_URL"), "MAVROS_FCU_URL (launch script)"
        self.hil_evidence["network_link"] = {**network_link_evidence(url),
                                             "source": src if url else None}
        self.hil_evidence["mavlink_router"] = os.environ.get("MAVLINK_ROUTER")
        self.hil_evidence["local_processes"] = scan_local_processes()
        self.hil_evidence["mavros_on"] = mavros_location(self.hil_evidence)

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

    def bring_up(self, alt_m: float, timeout: float = 180.0, before_arm=None) -> None:
        """As PymavlinkLink.bring_up: `before_arm()` runs once the position is
        known and before GUIDED is asked for (the GeoFence zones)."""
        m = self.msgs
        if not self.wait(lambda s: s.connected, timeout=timeout):
            raise RuntimeError("MAVROS never reported a connected FCU")
        self.identify()
        self._call(self.cli_rate, m.StreamRate.Request(stream_id=0, message_rate=10,
                                                       on_off=True))
        if not self.wait(lambda s: s.snap.lat is not None and s.snap.rel_alt is not None,
                         timeout=timeout):
            raise RuntimeError("no global position from MAVROS (EKF not ready?)")
        if before_arm is not None:
            before_arm()
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

    # ---- the GeoFence's zones and the throttle ---------------------------- #

    def upload_fence(self, items: list[dict], timeout: float = 10.0) -> dict:
        """Replace the autopilot's fence list with `items` (fence_items()
        dicts) through /mavros/geofence/push, then read it back through
        /mavros/geofence/pull, as sitl/mavlink_adapter_node.py does. Same
        result keys as PymavlinkLink.upload_fence; the read-back is MAVROS's
        count of what it pulled."""
        t_start = self.clock()
        m, n = self.msgs, len(items)
        out = {"via": "/mavros/geofence/push", "items": n, "transferred": 0,
               "ack": None, "read_back": None, "read_back_match": None}

        def done(ok: bool, why: str | None) -> dict:
            out.update(success=bool(ok), why=why,
                       upload_ms=round((self.clock() - t_start) * 1000.0, 1))
            return out
        wps = []
        for it in items:
            wp = m.Waypoint()
            wp.frame, wp.command = int(it["frame"]), int(it["command"])
            wp.is_current, wp.autocontinue = False, True
            wp.param1 = float(it["param1"])
            wp.param2 = wp.param3 = wp.param4 = 0.0
            wp.x_lat, wp.y_long, wp.z_alt = float(it["x_lat"]), float(it["y_long"]), 0.0
            wps.append(wp)
        res = self._call(self.cli_fence_push,
                         m.WaypointPush.Request(start_index=0, waypoints=wps), timeout)
        if res is None:
            return done(False, "/mavros/geofence/push is not available")
        out["transferred"] = int(getattr(res, "wp_transfered", 0) or 0)
        if not getattr(res, "success", False):
            return done(False, f"MAVROS refused the push after {out['transferred']}/{n}")
        pulled = self._call(self.cli_fence_pull, m.WaypointPull.Request(), timeout)
        if pulled is None or not getattr(pulled, "success", False):
            return done(False, "pushed, but /mavros/geofence/pull did not read it back")
        out["read_back"] = got = int(pulled.wp_received)
        out["read_back_match"] = got == n
        return done(got == n, None if got == n else
                    f"read back {got} item(s), not the {n} pushed")

    def read_fence(self, timeout: float = 5.0) -> dict:
        """The fence list the autopilot holds NOW, before the node replaces
        it: MAVROS's count of what /mavros/geofence/pull read (the items are
        not read here). Same keys as PymavlinkLink.read_fence."""
        out = {"via": "/mavros/geofence/pull", "count": None, "items": None,
               "why": None}
        res = self._call(self.cli_fence_pull, self.msgs.WaypointPull.Request(), timeout)
        if res is None or not getattr(res, "success", False):
            out["why"] = "/mavros/geofence/pull did not answer with success"
            return out
        out["count"] = int(res.wp_received)
        return out

    def rc_override_plan(self) -> dict:
        """Whether ArduPilot will take /mavros/rc/override: MAVROS sends it
        as its own system id (its `system_id` parameter), and ArduPilot takes
        an override only from SYSID_MYGCS. Both are read; a mismatch means the
        throttle stays where the radio has it and LOITER sinks."""
        gcs = self._get_params(self.cli_param, ["SYSID_MYGCS"]).get("SYSID_MYGCS")
        mine = self._get_params(self.cli_mavros_param, ["system_id"]).get("system_id")
        ok = None if gcs is None or mine is None else int(gcs) == int(mine)
        return {"via": TOPIC_RC_OVERRIDE, "sysid": None if mine is None else int(mine),
                "sysid_source": "/mavros system_id parameter" if mine is not None
                                else "unread",
                "SYSID_MYGCS": None if gcs is None else int(gcs),
                "accepted_expected": ok,
                "why": (None if ok else
                        "SYSID_MYGCS or MAVROS's system_id was not read" if ok is None
                        else f"ArduPilot takes overrides from SYSID_MYGCS {int(gcs)}, "
                             f"and MAVROS sends as {int(mine)}: set one to the other")}

    def override_throttle(self, pwm: int | None) -> bool:
        """Channel 3 to `pwm`, or back to the radio (None), on
        /mavros/rc/override; every other channel unchanged."""
        if self.pub_rc is None:
            self.pub_rc = self.node.create_publisher(self.msgs.OverrideRCIn,
                                                     TOPIC_RC_OVERRIDE, 10)
        msg = self.msgs.OverrideRCIn()
        ch = [RC_OVERRIDE_IGNORE] * 18
        ch[2] = RC_OVERRIDE_RELEASE if pwm is None else int(pwm)
        msg.channels = ch
        self.pub_rc.publish(msg)
        return True

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
