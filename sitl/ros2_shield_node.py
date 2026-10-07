"""
ROS 2 node: the Safety Shield, wired the way the grant's interface locks it.

    [vla node] --/vla/action_4d--> [THIS NODE] --/shield/setpoint--> [mavlink_adapter] --/mavros/setpoint_raw/local--> MAVROS 2 --> ArduPilot
                Float32MultiArray                Float32MultiArray                      mavros_msgs/PositionTarget
                BODY [vx_fwd, vy_right,          BODY, post-Shield                      FRAME_LOCAL_NED
                vz_up, yaw_rate] 10 Hz

THE FRAME (WP3-04, ARCH-04, WP2-17). The grant locks the VLA action as
"a = (vx, vy, vz, yaw_rate) # body-frame velocities + yaw rate" and puts the
NED conversion in the MAVLink adapter after the Shield. This node takes the
body-frame action, rotates it ONCE into the world frame the Shield's geometry
is written in (guardrail.frames.from_body, with the heading of the same pose
the adapter reads), lets the Shield check and repair it, rotates the result
back to body frame (frames.to_body) and publishes that on /shield/setpoint.
The Shield therefore repairs the body-frame action it was given and returns a
body-frame action; the rotation is exact and is undone with the same heading.
sitl/mavlink_adapter_node.py turns /shield/setpoint into NED - the single
body -> local-NED crossing - and owns every MAVROS command after take-off
(SET_MODE, fence upload). Until 2026-10-06 this node published a Twist on
/mavros/setpoint_velocity/cmd_vel_unstamped and did the ENU conversion itself
(audit cards WP3-03, WP3-09, ARCH-05).

WHAT EVERY RUN RECORDS (WP2-11, WP2-15, WP1-24, WP4-13, WP4-20, WP3-11, WP3-17)

  flight_log.jsonl  one row per tick: position, heading, the action in both
                    frames, violations, repairs, `unsafe` + `unsafe_rules`
                    (is the POSITION illegal), policy_hash + generation,
                    shield_ms (the Shield's own time), the declared subject
                    (tgt_x/tgt_y), the FSM state, the autopilot's mode, the
                    episode id, and `flown`: whether anything was sent on
                    that tick (false while the autopilot holds the aircraft,
                    e.g. a GeoFence RTL - such a tick cannot be an escape).
  policy_g<N>.json, csp_g<N>.json
                    the policy and the typed CSP of every generation, written
                    at take-off and after each hot-apply (guardrail.replay.
                    EpisodeRecord).
  audit.jsonl       this episode's Shield audit only; a stale one is moved to
                    _previous/, never appended to.
  events.jsonl      episode start, generations, hot-applies, mode requests and
                    changes, fence uploads, FSM transitions, autopilot text.
  metrics.json      incl. hil_evidence (the MAVROS chain, the autopilot's
                    own version, the host, the link) and the pilot record.
  manifest.json     the grant's six fields; topology from the evidence.
  <tag>.replay.tar.gz  written here, and re-packed by run_ros2_demo.sh once
                    the rosbag is closed so it carries bag/ as well.

TOPOLOGY. One desktop running ArduPilot SITL + MAVROS 2 + this node is the
grant's `dev`. On a Jetson Orin, with ArduPilot SITL on the desktop, the same
node gathers the Orin's evidence (device tree, L4T / JetPack, the processes
running on the Orin, the VLA node's own host from /vla/identity, MAVROS's
fcu_url) and its manifest says `hil`, whether MAVROS runs on the desktop (the
grant's table) or on the Orin (docs/RUNBOOK-orin-hil.md); with a hardware
autopilot it says `flight`. The label comes from that evidence
(guardrail.manifest.detect_topology), never from a flag, so moving the stack
from the desktop to the Orin to the drone changes no code.

GEOFENCE BACKSTOP. At take-off the node reads FENCE_ENABLE, FENCE_ALT_MAX,
AVOID_ENABLE and the rest back from the autopilot, raises FENCE_ALT_MAX above
the policy's ceiling when it sits below it, and publishes each generation's
keep-out zones to the adapter - except a zone the aircraft is in or near,
which is held back until the aircraft is clear, so a zone hot-applied on top
of the aircraft is the Shield's to resolve, not an instant autopilot RTL.

SHIELD HORIZON. 5 s at 0.1 s steps, 50 future poses, as the grant's Safety
Shield page locks it (3 s at 0.5 s until 2026-10-07).

ESCALATION (WP3-07 wiring). With the Shield on, every decision goes through
guardrail.fsm.EscalationFSM (docs/DESIGN-escalation-fsm.md, "Integration").
Its mode requests go to the adapter on /shield/mode_request. Theta is applied
to the Shield's per-operator Repair.magnitude_m when shield.py reports it,
else to the velocity proxy with an explicit horizon (0.1 s, which cannot fire
on these flights); the source is recorded in metrics.json as
`fsm.theta_basis`. Flown with magnitude_m on 2026-10-07, the default mission
left Normal at the first tick (a forecast penetration of 8.1 m against theta
2.0 m), and ArduCopter's LOITER, with no RC input on SITL, descended to the
ground (docs/DESIGN-ros2-interface.md, "Escalation FSM wiring"). This node
runs the one FSM: its Shield is built without an FSM of its own
(guardrail.replay.rail_shield), because only the node sees the autopilot, and
each audit record carries the node's FSM verdict and its log row's tick
number (guardrail.replay.audit_tick).

Run (after `source /opt/ros/jazzy/setup.bash`; normally via run_ros2_demo.sh
or the launch file in sitl/ros2_ws):

    ~/venv-ros/bin/python sitl/ros2_shield_node.py --shield on [--dynamic]
    ~/venv-ros/bin/python sitl/ros2_shield_node.py --bundle bundles/fase3-sim-demo-v0.1.1.tar.gz

POLICY. `--bundle` flies the signed bundle (guardrail/bundle.py): the policy
the run used is then the issued artefact, not whatever the YAML says today.
`--policy` (or the default YAML) stays as the fallback and is recorded as
unsigned. The VLA node announces its own identity and policy hash on
/vla/identity; a mismatch with this node's policy is logged and recorded.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import Action4D, AuditLogger, Shield, State             # noqa: E402
from guardrail.bundle import load_for_flight                              # noqa: E402
from guardrail.compiler import ConstraintCompiler                         # noqa: E402
from guardrail.frames import from_body, to_body                           # noqa: E402
from guardrail.fsm import (FAILSAFE_STATES, EscalationFSM, FSMConfig,     # noqa: E402
                           tick_input_from_decision)
from guardrail.geometry import fence_polygon                              # noqa: E402
from guardrail.kpi import compute_from_dir                                # noqa: E402
from guardrail.manifest import (TOPOLOGY_ARDUPILOT_SITL, UNRESOLVED,      # noqa: E402
                                build_manifest, collect_host_evidence,
                                decode_custom_version,
                                describe_autopilot_version, detect_topology,
                                is_kpi_grade, mavros_location,
                                network_link_evidence, pilot_record,
                                scan_local_processes)
from guardrail.models import (AltitudeEnvelope, PolygonFence,             # noqa: E402
                              SubjectStandoff, XY)
from guardrail.replay import (EpisodeRecord, audit_tick,                  # noqa: E402
                              episode_row_fields, flown_fields,
                              kpi_evidence_grade, rail_shield,
                              verify_replay, write_replay)
from guardrail.shield import Repair                                       # noqa: E402
from mavlink_adapter_node import (FENCE_MARGIN_M, TOPIC_FENCES,           # noqa: E402
                                  TOPIC_MODE_REQUEST, TOPIC_SETPOINT,
                                  TOPIC_STATUS, fence_alt_plan,
                                  heading_deg_from_enu_quaternion,
                                  policy_ceiling_m, policy_fences, read_parm,
                                  zones_near)

TICK = 0.1
MAX_S = 120
REACH_M = 2.0
DYNAMIC_AT_S = 8.0
DYNAMIC_FENCE = PolygonFence(
    id="nfz-dynamic", type="polygon_fence",
    vertices=[XY(x=23, y=6), XY(x=31, y=6), XY(x=31, y=14), XY(x=23, y=14)],
    margin_m=1.0,
)
# `--dynamic-zone on-aircraft`: the grant's spawn_polygon_fence perturbation at
# its hardest, a zone hot-applied ON TOP of the aircraft, so the run starts
# unsafe and time to safe is measured with the Shield on (WP3-17).
ON_AIRCRAFT_HALF_M = 5.0


def keepout_zones(policy) -> list:
    """Every keep-out zone with vertices: polygon and circle fences, and the
    dynamic_nfz a mid-flight zone becomes where the Shield enforces the
    grant's update model. (A moving dynamic_nfz is drawn and scored where it
    was spawned; the rails spawn static ones.)"""
    return [c for c in getattr(policy, "constraints", [])
            if getattr(c, "type", "") in ("polygon_fence", "circle_fence",
                                          "dynamic_nfz")
            and getattr(c, "vertices", None)]


def zone_around(x: float, y: float, half_m: float = ON_AIRCRAFT_HALF_M,
                zone_id: str = "nfz-on-aircraft") -> PolygonFence:
    """A square keep-out zone centred on (x North, y East)."""
    return PolygonFence(
        id=zone_id, type="polygon_fence", margin_m=1.0,
        vertices=[XY(x=x - half_m, y=y - half_m), XY(x=x + half_m, y=y - half_m),
                  XY(x=x + half_m, y=y + half_m), XY(x=x - half_m, y=y + half_m)])
FENCE_PARM = ROOT / "sitl" / "fence" / "guardrail_fence.parm"
# The grant's Safety Shield horizon: "5 s of predicted trajectory at 10 Hz =
# 50 future poses". Until 2026-10-07 both SITL rails ran 3 s at 0.5 s steps
# (6 poses).
LOOKAHEAD_S = 5.0
LOOKAHEAD_DT_S = 0.1
# The autopilot parameters that make the GeoFence a backstop, read back at
# take-off and recorded (hil_evidence.autopilot_fence).
FENCE_PARAMS = ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX",
                "FENCE_MARGIN", "AVOID_ENABLE")
# The FSM's theta needs a per-operator magnitude (Repair.magnitude_m). Until
# shield.py reports it, the velocity proxy with this horizon is used, and the
# basis is written into metrics.json.
PROXY_HORIZON_S = 0.1
# After the autopilot takes the aircraft away from the Shield (a GeoFence
# breach RTL, a pilot on the GCS), how long to keep recording before ending
# the episode if it has not landed.
TAKEOVER_GRACE_S = 25.0
# How long after the FSM or the autopilot entered RTL / Land to wait for the
# landing before the episode is ended anyway.
FAILSAFE_GRACE_S = 40.0


# --------------------------------------------------------------------------- #
# Pure helpers (tests/test_sitl_rails.py imports these without ROS)
# --------------------------------------------------------------------------- #

def parse_xy(text: str | None) -> tuple[float, float] | None:
    if not text:
        return None
    parts = [float(v) for v in text.split(",")]
    if len(parts) != 2 or not all(math.isfinite(v) for v in parts):
        raise ValueError(f"expected X,Y, got {text!r}")
    return parts[0], parts[1]


def default_tag(shield: str, dynamic: bool) -> str:
    """Kept identical to run_ros2_demo.sh's rule, which records the bag there."""
    return f"ros2_shield_{shield}" + ("_dynamic" if dynamic else "")


def shield_step(shield: Shield, state: State, body: Action4D, shield_on: bool):
    """One tick through the Shield in the grant's body frame.

    Returns `(decision, raw_world, emitted_world, emitted_body)`. The Shield
    always evaluates (the control arm needs its violations); with it off the
    body action passes through untouched, not rotated there and back.
    """
    raw_world = from_body(body.vx, body.vy, body.vz_up, body.yaw_rate,
                          state.yaw_deg)
    decision = shield.filter(state, raw_world)
    if not shield_on:
        return decision, decision.raw, decision.raw, body
    em = decision.emitted
    vx, vy, vz, wz = to_body(em, state.yaw_deg)
    return decision, decision.raw, em, Action4D(vx=vx, vy=vy, vz_up=vz,
                                                yaw_rate=wz)


def build_row(*, now: float, tick: int, state: State, body: Action4D,
              decision, raw_world: Action4D, emitted_world: Action4D,
              emitted_body: Action4D | None, shield_on: bool, extra: dict,
              subject: tuple | None, fsm_out=None, stop_illegal=None,
              mode: str | None = None, setpoint: str = "pass",
              shield: Shield | None = None) -> dict:
    """The per-tick row, in the shape guardrail/kpi.py reads.

    raw / emitted stay in the world frame the Shield's checks use (kpi.py's
    repair magnitudes are frame-invariant anyway); raw_body / emitted_body are
    what the grant's interface carried. `flown` and `emitted_violations` say
    what was FLOWN and the check on it (guardrail.replay.flown_fields): the
    re-check with the Shield on, the violations of `raw` with it off - which
    is what earns the control arm its escapes - the zero action's on a Brake
    tick, and nothing (`flown: false`) on a tick where the rail sent nothing
    because the autopilot or a fault held the aircraft. `shield` is needed
    for a Brake tick, whose check is the Shield's own.
    """
    if setpoint == "brake" and shield is None:
        raise ValueError("a Brake row needs the Shield to check the zero action")
    flown = flown_fields(shield, state, decision, shield_on=shield_on,
                         setpoint=setpoint)
    row = {
        "t": round(now, 3), "tick": tick,
        "x": round(state.x, 3), "y": round(state.y, 3), "up": round(state.up, 3),
        "yaw_deg": round(state.yaw_deg, 2),
        "frame": "body",
        "raw_body": body.model_dump(),
        "emitted_body": None if emitted_body is None else emitted_body.model_dump(),
        "raw": raw_world.model_dump(),
        "emitted": emitted_world.model_dump(),
        "violations": [v.model_dump() for v in decision.violations],
        **flown,
        "repairs": ([r.model_dump() for r in decision.repairs]
                    if shield_on else []),
        "braked": bool(shield_on and decision.braked),
        "touched": bool(shield_on and decision.touched),
        "setpoint": setpoint,
        "mode": mode,
        **extra,
    }
    if subject is not None:
        row["tgt_x"], row["tgt_y"] = subject[0], subject[1]
    if fsm_out is not None:
        row.update({"fsm_state_before": fsm_out.before.value,
                    "fsm_state_after": fsm_out.state.value,
                    "fsm_edge": fsm_out.edge, "set_mode": fsm_out.set_mode,
                    "stop_illegal": bool(stop_illegal),
                    "failsafe": bool(fsm_out.transition
                                     and fsm_out.state in FAILSAFE_STATES)})
    elif shield_on is False:
        row["fsm_state_after"] = None
    return row


def theta_basis(horizon_s: float | None) -> dict:
    """How theta's repair magnitude is measured on this run."""
    if horizon_s is None:
        return {"source": "per-operator magnitude_m", "horizon_s": None}
    return {"source": "velocity proxy |dv| * h (guardrail.fsm.magnitude_from_actions)",
            "horizon_s": horizon_s,
            "note": ("at h = 0.1 s the theta cap cannot fire on these flights "
                     "(docs/DESIGN-escalation-fsm.md); N-in-T, X6 and direct "
                     "RTL / Land / Loiter actions are live")}


def default_theta_horizon() -> float | None:
    return None if "magnitude_m" in getattr(Repair, "model_fields", {}) \
        else PROXY_HORIZON_S


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    k = min(len(s) - 1, max(0, int(math.ceil(q / 100.0 * len(s))) - 1))
    return round(s[k], 3)


# --------------------------------------------------------------------------- #
# The node (needs rclpy + mavros_msgs)
# --------------------------------------------------------------------------- #

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float32MultiArray, String
    from mavros_msgs.msg import ExtendedState, State as MavState, StatusText
    from mavros_msgs.srv import (CommandBool, CommandTOL, SetMode, StreamRate,
                                 VehicleInfoGet)
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

if HAVE_ROS:
    LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    LANDED_STATE_ON_GROUND = 1

    class ShieldNode(Node):
        def __init__(self, args, out: Path, subject=None) -> None:
            super().__init__("safety_shield")
            self.args = args
            self.shield_on = args.shield == "on"
            self.dynamic = args.dynamic
            self.out = out
            self.max_s = float(args.max_s)

            # The signed bundle when one is given; otherwise the YAML, recorded
            # as unsigned. Refuses before take-off, never mid-air.
            self.policy, self.policy_source = load_for_flight(
                args.bundle,
                args.policy or (None if args.bundle else
                                ROOT / "policies" / "sim_demo_policy.yaml"),
                allow_unverified=args.allow_unverified_bundle)
            self.get_logger().info(
                f"policy {self.policy.policy_id} v{self.policy.version} "
                f"{self.policy.policy_hash} from {self.policy_source['kind']} "
                f"{self.policy_source['path']} - signature "
                f"{self.policy_source['signature']}")
            compiler = ConstraintCompiler(self.policy)
            # The default mission is written out literally: the stress
            # harness (experiments/sweep_scenarios.py SITL_RAIL_COMMAND) and
            # its test compare scenarios against exactly this text.
            self.mission = (compiler.parse_command(args.command) if args.command
                            else compiler.parse_command("fly to the northeast pad at 6 m/s"))
            tgt = parse_xy(args.target)
            if tgt is not None:
                self.mission = self.mission.model_copy(
                    update={"target_x": tgt[0], "target_y": tgt[1]})
            if args.speed is not None:
                self.mission = self.mission.model_copy(
                    update={"speed_pref_mps": float(args.speed)})

            # Episode record FIRST: it moves an earlier flight's audit and
            # generation files aside, so the logger below starts a fresh file.
            self.rec = EpisodeRecord(out, self.policy, self.mission,
                                     lookahead_s=LOOKAHEAD_S)
            (out / "prompt.yaml").write_text(compiler.build_prompt(self.mission),
                                             encoding="utf-8")
            # No escalation FSM inside the Shield: this node runs THE FSM,
            # because only it sees the autopilot (guardrail.replay.rail_shield).
            self.shield = rail_shield(self.policy, lookahead_s=LOOKAHEAD_S,
                                      dt=LOOKAHEAD_DT_S)
            self.audit = AuditLogger(out / "audit.jsonl", self.policy,
                                     episode_id=str(self.rec.episode_id))

            # Escalation FSM, Shield-on arm only (the control arm records
            # fsm_state_after: null).
            self.theta_horizon_s = (default_theta_horizon()
                                    if args.theta_horizon_s == "auto"
                                    else (None if args.theta_horizon_s == "none"
                                          else float(args.theta_horizon_s)))
            cfg = FSMConfig.from_yaml(args.fsm_config) if args.fsm_config else FSMConfig()
            self.fsm = EscalationFSM(cfg) if self.shield_on else None
            self.fsm_fault: str | None = None

            # A DECLARED subject, so subject_standoff rules bind on a rail with
            # no camera. Ground truth, not perception.
            self.subject = subject
            self.subject_class = args.subject_class
            if subject is not None:
                self.shield.set_subject(subject[0], subject[1], self.subject_class)
                rings = [(c.id, c.min_range_m)
                         for c in self.policy.by_type(SubjectStandoff)
                         if c.binds(self.subject_class)]
                self.get_logger().info(
                    f"{self.subject_class} DECLARED at ({subject[0]:.1f}, "
                    f"{subject[1]:.1f}); "
                    + (", ".join(f"{i} {r:.0f}m" for i, r in rings) if rings
                       else "WARNING no standoff rule binds - the ring is inert"))

            self.state: State | None = None
            self.raw_body = Action4D()
            self.n_actions = 0
            self.mav_connected = False
            self.mode: str | None = None
            self.landed: bool | None = None
            self.traj: list[dict] = []
            self.rows: list[dict] = []
            # Evidence of the topology, gathered from the live system rather
            # than asserted (guardrail.manifest.check_topology_evidence).
            self.hil_evidence: dict = {
                "ros_distro": os.environ.get("ROS_DISTRO"),
                "mavros_node": None,
                "fcu_connected": None,
                "ardupilot_version": None,
                "ardupilot_version_source": None,
                # The VLA node's own host, from /vla/identity; None until (and
                # unless) it reports one. The grant puts it on the Orin.
                "vla_host": None,
                **collect_host_evidence(),
            }
            self.vla_identity: dict | None = None
            self.fence_status: list[dict] = []
            # The autopilot's fence parameters as read back at take-off, and
            # the zones held back because the aircraft was in or near them.
            self.autopilot_fence: dict = {}
            self.fence_column_m: float | None = None
            self.fence_margin_m: float = FENCE_MARGIN_M
            self.deferred_fences: list[str] = []
            self.zones_over_home: list[str] = []
            self.sim_speedup: float | None = None
            self.n_touched = self.n_braked = 0
            self.spawn_t = self.spawn_pos = None
            self.dynamic_fence_id: str | None = None
            self.mission_t0: float | None = None
            self.done = False
            self.reached = False
            self.takeover: dict | None = None
            self.failsafe_t: float | None = None
            self.n_pose = 0
            self._last_rate_req = 0.0
            self._requested_modes: list[str] = []

            self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                     self._on_pose, qos_profile_sensor_data)
            self.create_subscription(MavState, "/mavros/state", self._on_state, 10)
            self.create_subscription(ExtendedState, "/mavros/extended_state",
                                     self._on_ext, qos_profile_sensor_data)
            self.create_subscription(StatusText, "/mavros/statustext/recv",
                                     self._on_text, qos_profile_sensor_data)
            self.create_subscription(Float32MultiArray, "/vla/action_4d",
                                     self._on_action, 10)
            self.create_subscription(String, "/vla/identity", self._on_identity,
                                     LATCHED)
            self.create_subscription(String, TOPIC_STATUS, self._on_adapter,
                                     LATCHED)
            self.pub = self.create_publisher(Float32MultiArray, TOPIC_SETPOINT, 10)
            self.pub_mode = self.create_publisher(String, TOPIC_MODE_REQUEST, 10)
            self.pub_fences = self.create_publisher(String, TOPIC_FENCES, LATCHED)
            self.pub_intercept = self.create_publisher(String, "/shield/intercept", 10)

            self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
            self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
            self.cli_tol = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
            self.cli_rate = self.create_client(StreamRate, "/mavros/set_stream_rate")
            self.cli_param = self.create_client(GetParameters,
                                                "/mavros/param/get_parameters")
            self.cli_mavros_param = self.create_client(GetParameters,
                                                       "/mavros/get_parameters")
            self.cli_info = self.create_client(VehicleInfoGet,
                                               "/mavros/vehicle_info_get")
            self.cli_param_set = self.create_client(
                SetParameters, "/mavros/param/set_parameters")
            # The fences are published in bring_up(), once the autopilot's
            # fence parameters and the aircraft's position are known.

        # ---------------- subscriptions ---------------- #

        def _t(self) -> float | None:
            return None if self.mission_t0 is None else time.monotonic() - self.mission_t0

        def _on_pose(self, msg: PoseStamped) -> None:
            q = msg.pose.orientation
            # MAVROS ENU -> our up-positive frame (x = N, y = E), heading
            # clockwise from North. The adapter reads the same topic.
            self.state = State(x=msg.pose.position.y, y=msg.pose.position.x,
                               up=msg.pose.position.z,
                               yaw_deg=heading_deg_from_enu_quaternion(
                                   q.x, q.y, q.z, q.w))
            if self.mission_t0 is not None:
                self.n_pose += 1

        def _on_state(self, msg: MavState) -> None:
            self.mav_connected = msg.connected
            self.hil_evidence["mavros_node"] = "/mavros"
            self.hil_evidence["fcu_connected"] = bool(msg.connected)
            if msg.mode != self.mode:
                self.rec.event("mode", t=self._t(), mode=msg.mode,
                               previous=self.mode, armed=bool(msg.armed))
                if (self.mission_t0 is not None and not self.done
                        and self.mode == "GUIDED" and msg.mode != "GUIDED"
                        and self.takeover is None
                        and not self._fsm_asked(msg.mode)):
                    # The autopilot left GUIDED without the Shield asking: a
                    # GeoFence breach, a failsafe, or a pilot on the GCS.
                    self.takeover = {"t": self._t(), "mode": msg.mode}
                    self.get_logger().warn(
                        f"autopilot took the aircraft: GUIDED -> {msg.mode}")
            self.mode = msg.mode

        def _fsm_asked(self, mode: str) -> bool:
            """Did this node ask for the mode the autopilot just entered?"""
            return mode in self._requested_modes

        def _on_ext(self, msg: ExtendedState) -> None:
            self.landed = msg.landed_state == LANDED_STATE_ON_GROUND

        def _on_text(self, msg: StatusText) -> None:
            self.rec.event("autopilot_text", t=self._t(), severity=int(msg.severity),
                           text=msg.text)

        def _on_action(self, msg: Float32MultiArray) -> None:
            d = list(msg.data)
            if len(d) >= 4 and all(math.isfinite(v) for v in d[:4]):
                self.raw_body = Action4D(vx=d[0], vy=d[1], vz_up=d[2], yaw_rate=d[3])
                self.n_actions += 1

        def _on_identity(self, msg: String) -> None:
            try:
                self.vla_identity = json.loads(msg.data)
            except ValueError:
                return
            self.rec.event("vla_identity", identity=self.vla_identity)
            host = self.vla_identity.get("host")
            # None stays None: a VLA node that does not say where it runs
            # cannot be shown to be on the Orin (manifest._vla_host_missing).
            self.hil_evidence["vla_host"] = host if isinstance(host, dict) else None
            vh = self.vla_identity.get("policy_hash")
            if vh and not self.policy.matches_hash(vh):
                # In the ros2_ped_* runs the two nodes read different files and
                # nothing showed it (audit card WP1-23).
                self.get_logger().warn(
                    f"the VLA node flies policy {vh}, this node "
                    f"{self.policy.policy_hash}")
                self.rec.event("policy_mismatch", vla=vh,
                               shield=self.policy.policy_hash)

        def _on_adapter(self, msg: String) -> None:
            try:
                rec = json.loads(msg.data)
            except ValueError:
                return
            if rec.get("kind") == "fence_upload":
                self.fence_status.append(rec)
            self.rec.event(f"adapter_{rec.get('kind', 'status')}", t=self._t(),
                           **{k: v for k, v in rec.items() if k not in ("kind", "t")})

        # ---------------- outputs ---------------- #

        def _publish_fences(self, reason: str) -> None:
            """This generation's keep-out zones to the adapter, minus any the
            aircraft is in or near (`deferred_fences`, re-sent from _tick once
            clear). The column is the autopilot's own FENCE_ALT_MAX as read
            back at take-off; unknown, nothing is exported."""
            st = self.state
            # No pose yet means the aircraft is still on the pad, at the
            # local origin. "Unknown, so upload everything" armed a zone that
            # covers home under the aircraft on a 2026-10-07 flight: the
            # GeoFence held it on the pad and take-off never completed.
            if st is not None:
                pos, pos_src = (st.x, st.y), "pose"
            else:
                pos, pos_src = (0.0, 0.0), "assumed home: no pose yet (on the pad)"
            fences, skipped = policy_fences(
                self.policy, column_m=self.fence_column_m, position=pos,
                margin_m=self.fence_margin_m)
            self.deferred_fences = sorted(s["id"] for s in skipped
                                          if s.get("deferred"))
            # Zones over home (the local origin in SITL): an RTL flies back
            # into them. Recorded and warned, not withheld.
            every, _ = policy_fences(self.policy, column_m=self.fence_column_m)
            over_home = zones_near(every, 0.0, 0.0, self.fence_margin_m)
            if over_home and over_home != self.zones_over_home:
                self.get_logger().warn(
                    f"zone(s) {', '.join(over_home)} cover home: an RTL lands "
                    f"in them and the GeoFence will report a breach there")
            self.zones_over_home = over_home
            # A geographic policy's frame origin (lat, lon), or None for local
            # metres about home; the adapter refuses one far from home.
            origin = getattr(self.policy, "frame_origin", None)
            doc = {"generation": self.policy.generation,
                   "policy_hash": self.policy.policy_hash,
                   "fences": fences, "skipped": skipped, "reason": reason,
                   "fence_alt_max_m": self.fence_column_m,
                   "position": [round(pos[0], 2), round(pos[1], 2)],
                   "position_source": pos_src,
                   "origin": (None if origin is None else
                              {"lat": origin[0], "lon": origin[1]}),
                   "backstop_active": self.autopilot_fence.get("backstop_active")}
            self.pub_fences.publish(String(data=json.dumps(doc)))
            self.rec.event("fences_published", t=self._t(), reason=reason,
                           generation=doc["generation"],
                           fence_ids=[f["id"] for f in fences], skipped=skipped,
                           deferred=self.deferred_fences,
                           zones_over_home=self.zones_over_home)

        def _recheck_deferred(self, st: State, unsafe_rules: list[str]) -> None:
            """Upload a held-back zone once the aircraft is clear of it: out
            of the zone and its margin, and the Shield no longer finds the
            position unsafe under that rule."""
            if not self.deferred_fences:
                return
            _, skipped = policy_fences(self.policy, column_m=self.fence_column_m,
                                       position=(st.x, st.y),
                                       margin_m=self.fence_margin_m)
            still = {s["id"] for s in skipped if s.get("deferred")}
            cleared = [i for i in self.deferred_fences
                       if i not in still and i not in unsafe_rules]
            if cleared:
                self._publish_fences(f"deferred zone(s) {', '.join(cleared)} "
                                     f"now clear")

        def _request_mode(self, mode: str, why: str) -> None:
            """Through the adapter (WP3-09); straight to MAVROS only when no
            adapter is listening, and the record says which."""
            self._requested_modes.append(mode)
            via = "adapter"
            if self.count_subscribers(TOPIC_MODE_REQUEST) > 0:
                self.pub_mode.publish(String(data=mode))
            elif self.cli_mode.service_is_ready():
                self.cli_mode.call_async(SetMode.Request(custom_mode=mode))
                via = "mavros (no adapter subscribed)"
            else:
                via = "nobody: set_mode not ready"
            self.rec.event("mode_request", t=self._t(), mode=mode, why=why, via=via)
            self.get_logger().info(f"{mode} requested ({why}) via {via}")

        def _publish_body(self, a: Action4D) -> None:
            self.pub.publish(Float32MultiArray(
                data=[float(a.vx), float(a.vy), float(a.vz_up), float(a.yaw_rate)]))

        # ---------------- helpers ---------------- #

        def _call(self, client, req, timeout: float = 5.0):
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            return fut.result()

        def _get_params(self, client, names: list[str]) -> dict:
            try:
                res = self._call(client, GetParameters.Request(names=names), 5.0)
            except Exception:                                 # noqa: BLE001
                return {}
            out = {}
            if res is not None:
                for n, v in zip(names, res.values):
                    # type 0 = PARAMETER_NOT_SET. MAVROS sometimes reports
                    # SIM_SPEEDUP as INTEGER with the value in double_value,
                    # so take whichever field carries one.
                    if v.type == 0:
                        out[n] = None
                    elif v.type == 4:
                        out[n] = v.string_value
                    else:
                        out[n] = float(v.double_value) or float(v.integer_value)
            return out

        def read_sim_speedup(self) -> float | None:
            """SIM_SPEEDUP from the autopilot, or None - and whether the
            autopilot is SITL at all.

            MAVROS 2 surfaces autopilot parameters as node parameters of
            /mavros/param (rcl_interfaces/GetParameters); it populates them
            itself on FCU connect, and a forced pull raced the refill and
            returned 0.0, so this polls and accepts the first positive answer.
            A hardware autopilot has no SIM_ parameters: SYSID_THISMAV present
            and SIM_SPEEDUP absent marks it `hardware`. Never assumed.
            """
            deadline = time.time() + 20.0
            vals: dict = {}
            while time.time() < deadline:
                vals = self._get_params(self.cli_param, ["SIM_SPEEDUP",
                                                         "SYSID_THISMAV"])
                sp = vals.get("SIM_SPEEDUP")
                if isinstance(sp, float) and sp > 0.0:
                    self.hil_evidence["autopilot_kind"] = "sitl"
                    return round(sp, 4)
                rclpy.spin_once(self, timeout_sec=0.5)
            # Only after the whole wait: while MAVROS is still loading the
            # parameter set, SYSID_THISMAV can be present before SIM_* is.
            hardware = bool(vals.get("SYSID_THISMAV")) and \
                "SIM_SPEEDUP" in vals and vals["SIM_SPEEDUP"] is None
            self.hil_evidence["autopilot_kind"] = "hardware" if hardware else None
            if not hardware:
                self.get_logger().warn("[speedup] SIM_SPEEDUP never resolved")
            return None

        def read_autopilot_version(self) -> None:
            """AUTOPILOT_VERSION as MAVROS received it (ARCH-31); the setup
            script's banner only as a labelled fallback."""
            deadline = time.time() + 15.0
            while time.time() < deadline:
                try:
                    res = self._call(self.cli_info, VehicleInfoGet.Request(
                        sysid=0, compid=0, get_all=False), 5.0)
                except Exception:                             # noqa: BLE001
                    res = None
                if res is not None and res.success and res.vehicles:
                    v = res.vehicles[0]
                    if v.available_info & 2:      # HAVE_INFO_AUTOPILOT_VERSION
                        custom = decode_custom_version(v.flight_custom_version)
                        self.hil_evidence.update({
                            "ardupilot_version": describe_autopilot_version(
                                v.flight_sw_version, custom, v.type, v.autopilot),
                            "ardupilot_version_source":
                                "AUTOPILOT_VERSION via /mavros/vehicle_info_get",
                            "flight_sw_version": f"{int(v.flight_sw_version):08x}",
                            "flight_custom_version_raw": v.flight_custom_version,
                        })
                        return
                rclpy.spin_once(self, timeout_sec=0.5)
            env = os.environ.get("ARDUPILOT_VERSION")
            if env:
                self.hil_evidence.update({
                    "ardupilot_version": env,
                    "ardupilot_version_source":
                        "sitl/setup_sitl.sh --verify binary banner (the "
                        "autopilot itself did not answer)",
                    "ardupilot_pin_ok": os.environ.get("ARDUPILOT_PIN_OK") == "1"})
            else:
                self.get_logger().warn("autopilot version not read")

        def read_link(self) -> None:
            """The MAVLink link MAVROS flies over: its own fcu_url parameter,
            else the URL the launch script started it with (labelled)."""
            url = self._get_params(self.cli_mavros_param, ["fcu_url"]).get("fcu_url")
            src = "/mavros fcu_url parameter"
            if not url:
                url, src = os.environ.get("MAVROS_FCU_URL"), "MAVROS_FCU_URL (launch script)"
            self.hil_evidence["network_link"] = {**network_link_evidence(url),
                                                 "source": src if url else None}
            self.hil_evidence["mavlink_router"] = os.environ.get("MAVLINK_ROUTER")
            # Which MAVLink actors run on THIS host (None off Linux): where
            # SITL and MAVROS are decides dev vs hil, whatever the URL says.
            procs = scan_local_processes()
            self.hil_evidence["local_processes"] = procs
            self.hil_evidence["mavros_on"] = mavros_location(self.hil_evidence)

        def _set_param(self, name: str, value: float) -> bool:
            """One autopilot parameter through MAVROS 2 (/mavros/param is a
            node whose parameters are the autopilot's)."""
            p = Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
            try:
                res = self._call(self.cli_param_set,
                                 SetParameters.Request(parameters=[p]), 5.0)
            except Exception:                                 # noqa: BLE001
                return False
            return bool(res and res.results and res.results[0].successful)

        def read_autopilot_fence(self) -> None:
            """The GeoFence parameters as the AUTOPILOT holds them, not as the
            .parm file says (GUARDRAIL_FENCE=0 starts SITL without it, and the
            node flies any --policy). FENCE_ALT_MAX is raised above the
            policy's ceiling when it sits below it, so the backstop is never
            stricter than the policy; if that fails the record says so."""
            vals: dict = {}
            deadline = time.time() + 20.0
            while time.time() < deadline:
                vals = self._get_params(self.cli_param,
                                        list(FENCE_PARAMS) + ["SIM_SPEEDUP"])
                # ALL of them: MAVROS fills /mavros/param while it pulls the
                # parameter list, and a 2026-10-07 flight read FENCE_ENABLE
                # before FENCE_ALT_MAX existed, so no zone was exported.
                if all(vals.get(k) is not None for k in FENCE_PARAMS):
                    break
                rclpy.spin_once(self, timeout_sec=0.5)
            ceiling, ceiling_src = policy_ceiling_m(self.policy,
                                                    self.mission.cruise_alt_m)
            alt_max = vals.get("FENCE_ALT_MAX")
            plan = fence_alt_plan(ceiling, alt_max)
            raised_from = None
            # Only SITL's parameters are changed by this node. On a hardware
            # autopilot the grant has the GeoFence parameters "GCS-set", and a
            # change would persist in its EEPROM: there a fence below the
            # policy's ceiling is recorded (stricter_than_policy), not fixed.
            is_sitl = isinstance(vals.get("SIM_SPEEDUP"), float) \
                and vals["SIM_SPEEDUP"] > 0.0
            if plan["raise_to"] is not None and alt_max is not None and is_sitl:
                if self._set_param("FENCE_ALT_MAX", plan["raise_to"]):
                    raised_from = alt_max
                    alt_max = self._get_params(
                        self.cli_param, ["FENCE_ALT_MAX"]).get("FENCE_ALT_MAX")
            enabled = vals.get("FENCE_ENABLE")
            self.autopilot_fence = {
                **{k: vals.get(k) for k in FENCE_PARAMS},
                "FENCE_ALT_MAX": alt_max,
                "source": ("/mavros/param (read from the autopilot)"
                           if enabled is not None else None),
                "parm_file": {k: v for k, v in read_parm(FENCE_PARM).items()
                              if k in FENCE_PARAMS},
                "policy_ceiling_m": ceiling, "policy_ceiling_source": ceiling_src,
                "fence_alt_max_raised_from": raised_from,
                "raise_allowed": is_sitl,
                "backstop_active": (None if enabled is None else enabled >= 1.0),
                "stricter_than_policy": (
                    None if alt_max is None or ceiling is None
                    else not fence_alt_plan(ceiling, alt_max)["ok"]),
                "plan_why": plan["why"],
            }
            self.hil_evidence["autopilot_fence"] = self.autopilot_fence
            self.fence_column_m = alt_max
            if vals.get("FENCE_MARGIN") is not None:
                self.fence_margin_m = float(vals["FENCE_MARGIN"])
            self.rec.event("autopilot_fence", **self.autopilot_fence)
            self.get_logger().info(
                f"autopilot fence: ENABLE {enabled}, ALT_MAX {alt_max}"
                + (f" (raised from {raised_from})" if raised_from is not None else "")
                + f", AVOID_ENABLE {vals.get('AVOID_ENABLE')}, policy ceiling "
                f"{ceiling} m")
            if self.autopilot_fence["backstop_active"] is False:
                self.get_logger().warn("FENCE_ENABLE is 0: no GeoFence backstop "
                                       "behind the Shield on this run")
            if self.autopilot_fence["stricter_than_policy"]:
                self.get_logger().warn(f"backstop stricter than the policy: "
                                       f"{plan['why']}")

        def bring_up(self) -> None:
            log = self.get_logger()
            t_end = time.time() + 180
            while time.time() < t_end and not self.mav_connected:
                rclpy.spin_once(self, timeout_sec=0.5)
            if not self.mav_connected:
                raise RuntimeError("mavros never connected to FCU")
            log.info("FCU connected")
            self.read_autopilot_version()
            log.info(f"autopilot: {self.hil_evidence.get('ardupilot_version')} "
                     f"({self.hil_evidence.get('ardupilot_version_source')})")
            self.read_link()

            # ArduPilot streams nothing until asked.
            self._call(self.cli_rate,
                       StreamRate.Request(stream_id=0, message_rate=10, on_off=True))
            log.info("stream rate 10 Hz requested")

            # The backstop as the autopilot holds it, then this generation's
            # zones - once the aircraft's position is known, so a zone over
            # the pad is held back rather than armed under the aircraft.
            self.read_autopilot_fence()
            pose_deadline = time.time() + 10.0
            while self.state is None and time.time() < pose_deadline:
                rclpy.spin_once(self, timeout_sec=0.5)
            self._publish_fences("take-off")

            # Give the adapter a moment to upload the fences before arming, so
            # the autopilot's backstop is in place from take-off.
            fence_deadline = time.time() + 15.0
            while time.time() < fence_deadline and not self.fence_status:
                rclpy.spin_once(self, timeout_sec=0.5)
            log.info("autopilot fence: " + (json.dumps(self.fence_status[-1])
                                            if self.fence_status else
                                            "no upload reported by the adapter"))

            while time.time() < t_end:
                r = self._call(self.cli_mode, SetMode.Request(custom_mode="GUIDED"))
                if r and r.mode_sent:
                    break
                time.sleep(1)
            log.info("GUIDED requested")

            while time.time() < t_end:
                r = self._call(self.cli_arm, CommandBool.Request(value=True))
                if r and r.success:
                    log.info("armed")
                    break
                time.sleep(2)
                rclpy.spin_once(self, timeout_sec=0.1)
            else:
                raise RuntimeError("arming timed out (EKF)")

            alt = self.mission.cruise_alt_m
            while time.time() < t_end:
                r = self._call(self.cli_tol, CommandTOL.Request(altitude=float(alt)))
                if r and r.success:
                    log.info(f"takeoff accepted -> {alt} m")
                    break
                time.sleep(2)
            else:
                raise RuntimeError("takeoff never accepted")

            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.state and self.state.up >= alt - 1.0:
                    # ArduPilot may set home only when the EKF is ready (at
                    # arming), and the adapter uploads the fence once home is
                    # known: give it a bounded wait before the mission.
                    wait_end = time.time() + 10.0
                    while not self.fence_status and time.time() < wait_end:
                        rclpy.spin_once(self, timeout_sec=0.5)
                    # FENCE_ALT_MAX still unread (MAVROS slow to fill its
                    # parameters): read again now, and re-send the zones,
                    # which were all withheld without a column.
                    if self.fence_column_m is None:
                        self.read_autopilot_fence()
                        if self.fence_column_m is not None:
                            self._publish_fences("fence parameters read late")
                            late_end = time.time() + 5.0
                            while time.time() < late_end:
                                rclpy.spin_once(self, timeout_sec=0.5)
                    self.rec.event("fence_before_mission",
                                   uploaded=bool(self.fence_status and
                                                 self.fence_status[-1].get("success")),
                                   fence_alt_max_m=self.fence_column_m)
                    log.info(f"at {self.state.up:.1f} m - mission start")
                    # Read here, not in _finish(): a service call from a timer
                    # callback is a nested spin and never completes.
                    self.sim_speedup = self.read_sim_speedup()
                    log.info(f"SIM_SPEEDUP read from autopilot: {self.sim_speedup}")
                    return
            raise RuntimeError("never reached takeoff altitude")

        # ---------------- mission tick ---------------- #

        def start_mission(self) -> None:
            # The first mission tick took 116 ms on a 2026-10-07 flight (33 ms
            # on the pymavlink rail; every later tick <= 3.2 ms), over the
            # grant's 100 ms tick budget. One Shield call before the mission,
            # on a throwaway Shield (the real one's history stays the
            # flight's), and its time recorded: on the two flights since, the
            # first mission tick took 1.2 ms. The spike's cause is not
            # established; the record keeps it visible if it recurs.
            if self.state is not None:
                warm = rail_shield(self.policy, lookahead_s=LOOKAHEAD_S,
                                   dt=LOOKAHEAD_DT_S)
                b = self.raw_body
                warm.filter(self.state, from_body(b.vx, b.vy, b.vz_up,
                                                  b.yaw_rate, self.state.yaw_deg))
                self.rec.event("shield_warmup", t=None,
                               ms=round(warm.history[-1].elapsed_ms, 3))
            self.mission_t0 = time.monotonic()
            self.rec.event("mission_start", t=0.0,
                           target=[self.mission.target_x, self.mission.target_y],
                           speed_pref_mps=self.mission.speed_pref_mps,
                           shield=self.args.shield)
            self.create_timer(TICK, self._tick)

        def _tick(self) -> None:
            if self.done or self.state is None:
                return
            now = time.monotonic() - self.mission_t0
            if now > self.max_s:
                self.get_logger().warn("mission time cap")
                self._finish("time cap")
                return
            st = self.state

            # Re-assert 10 Hz streams: a ground station on the same router
            # link (Mission Planner) sets its own rates for the whole link.
            if now - self._last_rate_req > 5.0 and self.cli_rate.service_is_ready():
                self.cli_rate.call_async(StreamRate.Request(
                    stream_id=0, message_rate=10, on_off=True))
                self._last_rate_req = now

            if self.dynamic and self.spawn_t is None and now >= DYNAMIC_AT_S:
                fence = (DYNAMIC_FENCE if self.args.dynamic_zone == "route"
                         else zone_around(st.x, st.y))
                self.dynamic_fence_id = fence.id
                # As a dynamic_nfz where the Shield enforces the grant's
                # mid-flight update model (EpisodeRecord.apply_zone).
                applied = self.rec.apply_zone(self.shield, fence, t=now)
                self._publish_fences(f"hot_apply {applied.id} ({applied.type})")
                self.spawn_t, self.spawn_pos = now, (st.x, st.y)
                self.get_logger().info(
                    f"dynamic NFZ applied t={now:.1f}s -> generation "
                    f"{self.policy.generation}")

            dist = math.hypot(self.mission.target_x - st.x,
                              self.mission.target_y - st.y)
            if dist < REACH_M and self.takeover is None:
                self.reached = True
                self.get_logger().info(f"target reached t={now:.1f}s")
                self._finish("target reached")
                return

            body = self.raw_body
            # This tick's row number (1-based), bound once: the audit record
            # and the log row of one decision carry the same tick.
            tick_no = len(self.traj) + 1
            decision, raw_w, em_w, em_body = shield_step(
                self.shield, st, body, self.shield_on)
            extra = episode_row_fields(self.shield, st, self.policy,
                                       self.rec.episode_id)
            self._recheck_deferred(st, extra["unsafe_rules"])

            fsm_out = None
            stop_illegal = None
            setpoint = "pass"
            # Once the autopilot has taken the aircraft (a GeoFence RTL), the
            # FSM no longer steps: its mode requests would fight the autopilot.
            if self.fsm is not None and self.fsm_fault is None \
                    and self.takeover is None:
                try:
                    stop_illegal = self.shield.state_is_unsafe(st)
                    inp = tick_input_from_decision(
                        now, decision, self.policy, horizon_s=self.theta_horizon_s,
                        rtl_failed=False,
                        home_reached=(self.mode == "RTL"
                                      and math.hypot(st.x, st.y) < REACH_M),
                        landed=bool(self.landed) and self.mode in ("RTL", "LAND"),
                        stop_illegal=stop_illegal)
                    fsm_out = self.fsm.step(inp)
                except ValueError as e:
                    # A Shield/FSM contract break mid-flight: hold, say why,
                    # stop streaming. The run is not KPI-grade.
                    self.fsm_fault = str(e)
                    self.get_logger().error(f"FSM input refused: {e}")
                    self.rec.event("fsm_fault", t=now, error=str(e))
                    self._request_mode("LOITER", "FSM input refused")
                    self.failsafe_t = now       # end the episode after the grace
            if fsm_out is not None:
                setpoint = fsm_out.setpoint
                if fsm_out.transition:
                    self.rec.event("fsm_transition", t=now,
                                   **{k: fsm_out.record[k] for k in (
                                       "fsm_state_before", "fsm_state_after",
                                       "transition", "edge", "edge_source",
                                       "reason", "fsm_config_hash")})
                if fsm_out.set_mode:
                    self._request_mode(fsm_out.set_mode, fsm_out.reason)
                if fsm_out.state in FAILSAFE_STATES and self.failsafe_t is None:
                    self.failsafe_t = now
            if self.fsm_fault is not None or self.takeover is not None:
                setpoint = "none"
            # After the FSM stepped, with ITS verdict (not a Shield-internal
            # FSM's): the audit and the flight log describe one state machine.
            audit_tick(self.audit, tick_no, decision, fsm_out=fsm_out,
                       fsm_fault=self.fsm_fault)

            if decision.touched and self.shield_on:
                self.n_touched += 1
                self.n_braked += int(decision.braked)
                self.pub_intercept.publish(String(data=json.dumps({
                    "t": round(now, 3),
                    "violations": [v.rule_id for v in decision.violations],
                    "repairs": [r.operator for r in decision.repairs],
                    "fsm": fsm_out.state.value if fsm_out else None})))

            flown = {"pass": em_body, "brake": Action4D(), "none": None}[setpoint]
            if flown is not None:
                self._publish_body(flown)

            self.traj.append({"t": round(now, 2), "x": st.x, "y": st.y,
                              "up": st.up,
                              "touched": bool(self.shield_on and decision.touched)})
            self.rows.append(build_row(
                now=now, tick=tick_no, state=st, body=body,
                decision=decision, raw_world=raw_w, emitted_world=em_w,
                emitted_body=flown, shield_on=self.shield_on, extra=extra,
                subject=self.subject, fsm_out=fsm_out,
                stop_illegal=stop_illegal, mode=self.mode, setpoint=setpoint,
                shield=self.shield))

            # The episode ends when the autopilot has the aircraft and is done
            # with it: landed, or a bounded wait so a run stays short.
            if self.takeover is not None and (
                    self.landed or now - self.takeover["t"] > TAKEOVER_GRACE_S):
                self._finish(f"autopilot took over ({self.takeover['mode']})")
            elif self.fsm is not None and (self.fsm.terminal or (
                    self.failsafe_t is not None
                    and now - self.failsafe_t > FAILSAFE_GRACE_S)):
                self._finish("escalation FSM ended the mission")

        # ---------------- wrap-up ---------------- #

        def _standoff_metrics(self) -> dict:
            if self.subject is None:
                return {"standoff_s": None, "standoff_min_range_m": None,
                        "subject": None}
            rings = [c.min_range_m for c in self.policy.by_type(SubjectStandoff)
                     if c.binds(self.subject_class)]
            ring = max(rings) if rings else 0.0
            d = [math.hypot(p["x"] - self.subject[0], p["y"] - self.subject[1])
                 for p in self.traj]
            return {
                "standoff_s": round(sum(1 for v in d if v < ring) * TICK, 2),
                "standoff_min_range_m": (round(min(d), 2) if d else None),
                "subject": {"class": self.subject_class, "x": self.subject[0],
                            "y": self.subject[1], "position_source": "declared"},
            }

        def _finish(self, why: str) -> None:
            self.done = True
            self.rec.end(why, t=self._t())
            # The mission-end LAND is not a fail-safe. If the FSM or the
            # autopilot already has the aircraft in RTL / LAND, leave it be.
            fsm_owns = self.fsm is not None and (
                self.fsm.terminal or self.fsm.state in FAILSAFE_STATES)
            if not fsm_owns and self.mode not in ("RTL", "LAND"):
                self._request_mode("LAND", "mission end")
            self._report()

        def _report(self) -> None:
            fences = [(f, fence_polygon(f)) for f in keepout_zones(self.policy)]

            def active(f, p):
                if f.id == self.dynamic_fence_id:
                    return self.spawn_t is not None and p["t"] >= self.spawn_t
                return True

            from shapely.geometry import Point
            inside = sum(1 for p in self.traj for f, poly in fences
                         if active(f, p)
                         and f.altitude_floor_m <= p["up"] <= f.altitude_ceiling_m
                         and poly.contains(Point(p["x"], p["y"])))
            nfz_s = inside * TICK
            kpi = "PASS" if nfz_s == 0 else "FAIL"

            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                fig, ax = plt.subplots(figsize=(7, 7))
                for f, poly in fences:
                    dyn = f.id == self.dynamic_fence_id
                    xs, ys = poly.exterior.xy
                    ax.fill(ys, xs, alpha=0.25, color="purple" if dyn else "red",
                            label="dynamic NFZ" if dyn else f"NFZ {f.id}")
                if self.spawn_pos:
                    ax.plot(self.spawn_pos[1], self.spawn_pos[0], "X",
                            color="purple", markersize=12)
                if self.traj:
                    ax.plot([p["y"] for p in self.traj], [p["x"] for p in self.traj],
                            "-", color="tab:blue", linewidth=2, label="flight path")
                    tx = [p["y"] for p in self.traj if p["touched"]]
                    if tx:
                        ax.plot(tx, [p["x"] for p in self.traj if p["touched"]], ".",
                                color="orange", markersize=4, label="shield active")
                    ax.plot(self.traj[0]["y"], self.traj[0]["x"], "go",
                            markersize=10, label="start")
                ax.plot(self.mission.target_y, self.mission.target_x, "k*",
                        markersize=16, label="target")
                ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
                ax.set_title(f"ROS 2 + MAVROS rail - shield "
                             f"{'ON' if self.shield_on else 'OFF'}\n"
                             f"NFZ time: {nfz_s:.1f}s | "
                             f"{'reached' if self.reached else 'NOT reached'}")
                ax.legend(loc="upper left", fontsize=9)
                ax.set_aspect("equal"); ax.grid(alpha=0.3)
                fig.tight_layout(); fig.savefig(self.out / "trajectory.png", dpi=130)
            except ImportError:
                pass

            (self.out / "trajectory.json").write_text(json.dumps(self.traj),
                                                      encoding="utf-8")

            envelopes = self.policy.by_type(AltitudeEnvelope)
            alt_bad = sum(1 for p in self.traj for e in envelopes
                          if not (e.alt_min_m <= p["up"] <= e.alt_max_m))
            alt_s = alt_bad * TICK
            shield_ms = [r["shield_ms"] for r in self.rows
                         if isinstance(r.get("shield_ms"), (int, float))]
            dur = self._t() or 0.0
            ident = self.vla_identity or {}
            model_id = ident.get("model_id") or "unresolved:no-vla-identity"
            pilot = None
            if ident.get("model_id") and ident.get("kind") in ("vla", "stub", "none"):
                pilot = pilot_record(ident["model_id"], ident["kind"],
                                     yaw_rate=ident.get("yaw_rate"),
                                     frame=ident.get("frame"))

            metrics = {
                "tag": self.out.name,
                "episode_id": self.rec.episode_id,
                "ticks": len(self.rows),
                # Ticks on which the VLA's (or the Shield's) action was sent,
                # and those on which the autopilot or a fault held the
                # aircraft and nothing was (flown: false in the row).
                "ticks_flown": sum(1 for r in self.rows if r.get("flown")),
                "ticks_not_flown": sum(1 for r in self.rows
                                       if r.get("flown") is False),
                "shield": "on" if self.shield_on else "off",
                "shield_lookahead": {"horizon_s": LOOKAHEAD_S,
                                     "dt_s": LOOKAHEAD_DT_S,
                                     "poses": int(round(LOOKAHEAD_S
                                                        / LOOKAHEAD_DT_S))},
                "reached": self.reached,
                # compute() reads this as "did the mission happen"; a waypoint
                # mission expresses reaching the target in its units.
                "frac_within_30m": 1.0 if self.reached else 0.0,
                "nfz_s": round(nfz_s, 2),
                **self._standoff_metrics(),
                "alt_violation_s": round(alt_s, 2),
                "interventions": self.n_touched,
                "brakes": self.n_braked,
                "frame": "body (grant Architecture constraints p2)",
                "seed": self.args.seed,
                "target": [self.mission.target_x, self.mission.target_y],
                "speed_pref_mps": self.mission.speed_pref_mps,
                "pilot": pilot,
                "vla_identity": self.vla_identity,
                "vla_actions_received": self.n_actions,
                "pose_hz": (round(self.n_pose / dur, 2) if dur > 0 else None),
                "shield_ms_p50": percentile(shield_ms, 50),
                "shield_ms_p99": percentile(shield_ms, 99),
                "shield_ms_max": (round(max(shield_ms), 3) if shield_ms else None),
                "fsm": ({**self.fsm.summary(),
                         "theta_basis": theta_basis(self.theta_horizon_s),
                         "fault": self.fsm_fault} if self.fsm else None),
                "autopilot_took_over": self.takeover,
                "fence_upload": (self.fence_status[-1] if self.fence_status else None),
                "fence_uploads": self.fence_status,
                "fences_still_deferred": self.deferred_fences,
                "zones_over_home": self.zones_over_home,
                "autopilot_fence": self.autopilot_fence or None,
                "generations": self.rec.generations,
                # Beside the manifest, which is exactly the grant's six fields.
                "hil_evidence": self.hil_evidence,
                "policy_source": self.policy_source,
            }
            (self.out / "flight_log.jsonl").write_text(
                "".join(json.dumps(r) + "\n" for r in self.rows), encoding="utf-8")

            speedup = self.sim_speedup
            topology = detect_topology(self.hil_evidence)
            try:
                manifest = build_manifest(
                    policy_hash=self.policy.policy_hash, model_id=model_id,
                    seed=self.args.seed, scene_path=None, sim_speedup=speedup,
                    topology=topology, hil_evidence=self.hil_evidence)
            except ValueError as e:
                self.get_logger().warn(f"not the {topology} topology: {e}")
                metrics["topology_refused"] = str(e)
                manifest = build_manifest(
                    policy_hash=self.policy.policy_hash, model_id=model_id,
                    seed=self.args.seed, scene_path=None, sim_speedup=speedup,
                    topology=TOPOLOGY_ARDUPILOT_SITL)
            metrics["topology"] = manifest["topology"]
            (self.out / "metrics.json").write_text(json.dumps(metrics, indent=2),
                                                   encoding="utf-8")
            (self.out / "manifest.json").write_text(json.dumps(manifest, indent=2),
                                                    encoding="utf-8")

            kpi_res = compute_from_dir(self.out, self.policy)
            graded, why = is_kpi_grade(manifest, {**metrics, "det_hz": None,
                                                  "start_heading_err_deg": None})
            if self.fsm_fault:
                graded, why = False, why + [f"FSM fault: {self.fsm_fault}"]
            kpi_res["kpi_grade"] = graded
            kpi_res["kpi_grade_reasons"] = why
            kpi_res["manifest"] = manifest
            kpi_res["policy_source"] = self.policy_source
            kpi_res["episode_id"] = self.rec.episode_id
            kpi_path = self.out / "kpi.json"
            kpi_path.write_text(json.dumps(kpi_res, indent=2), encoding="utf-8")

            # The bundle, while the policy that governed the flight is still in
            # memory. The rosbag is still open here, so it is left out;
            # run_ros2_demo.sh re-packs (python -m guardrail.replay pack) once
            # the bag is closed.
            rb_path = self.out / f"{self.out.name}.replay.tar.gz"
            try:
                rb = write_replay(self.out, self.policy, rb_path,
                                  changelog=f"ros2 {self.out.name}",
                                  policy_source=self.policy_source)
                ok, notes = verify_replay(rb)
                self.get_logger().info(
                    f"[replay] {rb.name} "
                    f"{'re-derives its own KPIs' if ok else 'FAILED verification'}")
                for n_ in notes:
                    self.get_logger().info(f"[replay]   - {n_}")
                # KPI evidence needs a SIGNED bundle: a graded run whose
                # bundle is keyless is demoted, and kpi.json and the bundle
                # are rewritten to say so.
                graded2, why2 = kpi_evidence_grade(graded, why, rb)
                if graded2 != graded:
                    graded, why = graded2, why2
                    kpi_res["kpi_grade"], kpi_res["kpi_grade_reasons"] = graded, why
                    kpi_path.write_text(json.dumps(kpi_res, indent=2),
                                        encoding="utf-8")
                    write_replay(self.out, self.policy, rb_path,
                                 changelog=f"ros2 {self.out.name}",
                                 policy_source=self.policy_source)
            except Exception as exc:                            # noqa: BLE001
                self.get_logger().warn(
                    f"[replay] not written: {type(exc).__name__}: {exc}")
                if graded:
                    graded, why = False, why + [f"no replay bundle: {exc}"]
                    kpi_res["kpi_grade"], kpi_res["kpi_grade_reasons"] = graded, why
                    kpi_path.write_text(json.dumps(kpi_res, indent=2),
                                        encoding="utf-8")
            self.get_logger().info(
                f"P0 escape rate {kpi_res['p0_violation_escape_rate']} | "
                f"NFZ {nfz_s:.1f}s | alt {alt_s:.1f}s | interventions "
                f"{self.n_touched} | topology {manifest['topology']}")
            self.get_logger().info(
                f"KPI-GRADE: {'YES' if graded else 'no'}"
                + ("" if graded else "  (" + "; ".join(why) + ")"))
            fsm_line = (f"{self.fsm.summary()['max_state']} (max state)"
                        if self.fsm else "not run (control arm)")
            (self.out / "report.md").write_text(
                f"""# ROS 2 rail report - shield {'ON' if self.shield_on else 'OFF'}

| Item | Value |
|------|-------|
| Path | vla node -> /vla/action_4d (body) -> safety_shield -> /shield/setpoint (body) -> mavlink_adapter -> /mavros/setpoint_raw/local -> MAVROS 2 -> ArduPilot |
| Autopilot | {self.hil_evidence.get('ardupilot_version')} ({self.hil_evidence.get('ardupilot_version_source')}) |
| Pilot | `{model_id}` |
| Policy | `{self.policy.policy_id}` `{self.policy.policy_hash}` |
| Policy source | {self.policy_source['kind']} `{self.policy_source['path']}` - signature **{self.policy_source['signature']}** |
| Topology | `{manifest['topology']}` |
| Target reached | {'yes' if self.reached else 'NO'} |
| Ticks | {len(self.traj)} ({metrics['ticks_flown']} flown, {metrics['ticks_not_flown']} held by the autopilot or a fault) |
| Shield lookahead | {LOOKAHEAD_S:g} s at {LOOKAHEAD_DT_S:g} s ({metrics['shield_lookahead']['poses']} poses) |
| Shield interventions | {self.n_touched} |
| Escalation FSM | {fsm_line} |
| Autopilot took over | {self.takeover or 'no'} |
| Autopilot fence parameters | {self.autopilot_fence or 'not read'} |
| Autopilot fence upload | {(self.fence_status[-1] if self.fence_status else 'no upload reported')} |
| Dynamic NFZ | {f'hot-applied t={self.spawn_t:.1f}s, generation {self.policy.generation}' if self.spawn_t else 'not used'} |
| **Time inside NFZ** | **{nfz_s:.1f} s** |
| **P0 KPI** | **{kpi}** |
""", encoding="utf-8")
            print(f"[report] NFZ {nfz_s:.1f}s -> {kpi} | touched {self.n_touched} "
                  f"| braked {self.n_braked} | ticks {len(self.traj)}")


def make_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--dynamic", action="store_true")
    ap.add_argument("--dynamic-zone", choices=["route", "on-aircraft"],
                    default="route",
                    help="with --dynamic: 'route' hot-applies the 8 x 8 m zone "
                         "across the route at 8 s; 'on-aircraft' a 10 x 10 m "
                         "zone centred on the aircraft, so the run starts "
                         "unsafe and time to safe is measured")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--policy", default=None,
                    help="policy YAML; defaults to policies/sim_demo_policy.yaml. "
                         "Recorded as UNSIGNED.")
    ap.add_argument("--bundle", default=None,
                    help="signed policy bundle. Refused unless its signature "
                         "verifies.")
    ap.add_argument("--allow-unverified-bundle", action="store_true")
    ap.add_argument("--command", default=None,
                    help="mission text (default: fly to the northeast pad at 6 m/s)")
    ap.add_argument("--target", default=None, metavar="X,Y",
                    help="override the mission target (local NED metres), so "
                         "the rail can fly a stress scenario's mission")
    ap.add_argument("--speed", type=float, default=None,
                    help="override the pilot's preferred speed, m/s")
    ap.add_argument("--seed", type=int, default=0,
                    help="recorded in the manifest. StubVLA itself draws no "
                         "random numbers.")
    ap.add_argument("--max-s", type=float, default=MAX_S,
                    help="mission time cap, seconds")
    ap.add_argument("--subject", default=None, metavar="X,Y",
                    help="declare a subject at this NED position so "
                         "subject_standoff rules bind (declared, not perceived)")
    ap.add_argument("--subject-class", default="pedestrian")
    ap.add_argument("--fsm-config", default=None,
                    help="escalation FSM thresholds YAML (guardrail.fsm.FSMConfig)")
    ap.add_argument("--theta-horizon-s", default="auto",
                    help="'auto' (per-operator magnitude when shield.py reports "
                         "it, else the 0.1 s proxy), 'none', or seconds")
    return ap


def main(argv: list[str] | None = None) -> None:
    args, ros_args = make_parser().parse_known_args(argv)
    if not HAVE_ROS:
        raise SystemExit("ros2_shield_node needs rclpy and mavros_msgs: source "
                         "/opt/ros/jazzy/setup.bash and use ~/venv-ros/bin/python")
    tag = args.tag or default_tag(args.shield, args.dynamic)
    out = ROOT / "demo" / "out" / tag
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = ShieldNode(args, out, subject=parse_xy(args.subject))
    try:
        node.bring_up()
        node.start_mission()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
        # keep spinning briefly so the LAND request goes out
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 3:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
