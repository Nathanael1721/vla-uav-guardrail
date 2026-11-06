"""
ROS 2 node: VLA stub publisher.

Publishes the grant's action contract on a ROS topic:

    /vla/action_4d   std_msgs/Float32MultiArray   [vx_fwd, vy_right, vz_up, yaw_rate] @ 10 Hz

BODY FRAME, as the grant locks it ("a = (vx, vy, vz, yaw_rate) # body-frame
velocities + yaw rate", Architecture constraints p2): vx forward along the
nose, vy to the right, vz up, yaw_rate rad/s clockwise. StubVLA plans in the
world frame (fly straight at the target), so this node rotates its answer into
the body frame with the heading it reads from /mavros/local_position/pose
(guardrail.frames.to_body). Until 2026-10-06 it published the world-frame
action under the same topic name, and the Shield node read it as world frame.

`--yaw-rate` makes the stub turn while it flies. The planned track is
unchanged, so a body-frame conversion that is wrong anywhere in the chain
(stub, Shield node, adapter) shows up as a curved or missed track - the run
that exercises yaw_rate on ArduPilot, which no stored run ever did (max
|yaw_rate| 0.0 on all twelve SITL runs).

It also announces what it is on /vla/identity (std_msgs/String JSON, latched):
model id, kind, frame, the policy hash it flies under, where that policy came
from, and the host it runs on (so a hil run can show the VLA is on the Orin,
as the grant puts it). The Shield node writes that into the run record instead of a
hard-coded "StubVLA" (audit card ARCH-06), and warns when the two nodes fly
different policies (WP1-23). A real VLA backend replaces THIS NODE ONLY -
same topic, same message, same identity announcement.

THE STRESS LIBRARY'S PILOTS (2026-11-06). The scenario library flies three
laws (guardrail.vla_stub): `--pilot stub_vla` (StubVLA, the default, as
before), `--pilot goto` (straight at --target at --speed, slowing into the
goal at the measured ArduPilot acceleration) and `--pilot constant` with
`--action VX,VY,VZ,YAW` (the same WORLD-frame action every tick: m/s North,
East, up and rad/s clockwise, turned into the body frame like the others).
Each announces its own model id (guardrail.vla_stub.MODEL_IDS), pinned by the
module's source in the manifest. `--start X,Y[,UP]` is the policy-frame
position of the autopilot's home: the MAVROS pose (metres about the EKF
origin, which SITL puts at home) is offset by it before any law reads it, so
a scenario that starts 200 m south of its policy's frame origin flies its
target in that frame. `--target X,Y,UP` gives the target an altitude (the
goto law climbs toward it, StubVLA holds it); `--no-target` flies a law that
needs none (constant). The entrypoint's mission role hands this node every
argument it hands the Shield node, so both read one --start, --target and
--speed; each ignores the other's flags (parse_known_args).

Run (after `source /opt/ros/jazzy/setup.bash`):

    ~/venv-ros/bin/python sitl/ros2_vla_stub_node.py [--bundle B | --policy P] [--yaw-rate 0.2]
    ~/venv-ros/bin/python sitl/ros2_vla_stub_node.py --pilot goto --start=-200,0,15 --target=-80,0,15 --speed 4
    ~/venv-ros/bin/python sitl/ros2_vla_stub_node.py --pilot constant --action=3,0,0,0 --no-target
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import Action4D, State                          # noqa: E402
from guardrail.bundle import load_for_flight                   # noqa: E402
from guardrail.compiler import ConstraintCompiler              # noqa: E402
from guardrail.frames import to_body                           # noqa: E402
from guardrail.manifest import collect_host_evidence           # noqa: E402
from guardrail.vla_stub import (APPROACH_ACCEL_MPS2, MODEL_IDS,  # noqa: E402
                                ConstantPilot, GotoPilot, StubVLA)
from mavlink_adapter_node import (XYArgumentParser,            # noqa: E402
                                  heading_deg_from_enu_quaternion)

MODEL_ID = MODEL_IDS["stub_vla"]
PILOTS = tuple(MODEL_IDS)


def parse_floats(text: str | None, counts: tuple[int, ...], what: str
                 ) -> tuple[float, ...] | None:
    """`text` ("1,2" / "1,2,3") as finite floats, with one of `counts`
    values; None for no text. Raises ValueError otherwise (before anything
    flies)."""
    if text is None or text == "":
        return None
    try:
        vals = tuple(float(v) for v in str(text).split(","))
    except ValueError:
        raise ValueError(f"{what}: expected numbers, got {text!r}") from None
    if len(vals) not in counts or not all(math.isfinite(v) for v in vals):
        raise ValueError(f"{what}: expected {' or '.join(map(str, counts))} finite "
                         f"values, got {text!r}")
    return vals


def pose_state(px_enu: float, py_enu: float, pz: float, q, start=None) -> State:
    """/mavros/local_position/pose in the policy frame: ENU (x East, y North)
    to x North, y East, offset by `start` (the policy-frame position of the
    EKF origin, which SITL puts at home; None = 0, 0); heading clockwise
    from North from the ENU quaternion `q` (x, y, z, w)."""
    sx, sy = (0.0, 0.0) if start is None else (float(start[0]), float(start[1]))
    return State(x=float(py_enu) + sx, y=float(px_enu) + sy, up=float(pz),
                 yaw_deg=heading_deg_from_enu_quaternion(*q))


def mission_from_args(args, mission):
    """The compiled mission with the scenario's target (and its altitude as
    the cruise altitude), speed and start altitude applied, as
    experiments/sweep_scenarios.py _resolve_mission builds StubVLA's: the
    cruise altitude is the target's when it names one, else the start's."""
    tgt = None if getattr(args, "no_target", False) else parse_floats(
        args.target, (2, 3), "--target")
    start = parse_floats(getattr(args, "start", None), (2, 3), "--start")
    upd = {}
    if tgt is not None:
        upd.update(target_x=tgt[0], target_y=tgt[1])
    if tgt is not None and len(tgt) == 3:
        upd["cruise_alt_m"] = tgt[2]
    elif start is not None and len(start) == 3:
        upd["cruise_alt_m"] = start[2]
    if start is not None:
        upd.update(start_x=start[0], start_y=start[1])
    if args.speed is not None:
        if not (math.isfinite(args.speed) and args.speed > 0):
            raise ValueError(f"--speed must be a positive number, got {args.speed!r}")
        upd["speed_pref_mps"] = float(args.speed)
    return (mission.model_copy(update=upd) if upd else mission), tgt, start


def pilot_from_args(args, mission, target) -> tuple[object, dict]:
    """(pilot, its record) for --pilot: StubVLA on the mission, the goto law
    on the target (at the mission's speed, slowing at APPROACH_ACCEL_MPS2) or
    the constant --action. Refused (ValueError) when the law lacks what it
    flies on: a target for stub_vla and goto, an action for constant."""
    kind = getattr(args, "pilot", "stub_vla") or "stub_vla"
    if kind not in MODEL_IDS:
        raise ValueError(f"--pilot {kind!r} is not one of {list(MODEL_IDS)}")
    no_target = bool(getattr(args, "no_target", False))
    if kind in ("stub_vla", "goto") and no_target:
        raise ValueError(f"--pilot {kind} flies to a target; --no-target leaves it none")
    if kind != "constant" and getattr(args, "action", None):
        raise ValueError(f"--action is the constant pilot's; --pilot {kind} flies to "
                         f"its target")
    rec = {"type": kind, "model_id": MODEL_IDS[kind]}
    if kind == "stub_vla":
        return StubVLA(mission), dict(rec, speed_mps=mission.speed_pref_mps,
                                      cruise_alt_m=mission.cruise_alt_m)
    if kind == "goto":
        up = target[2] if target is not None and len(target) == 3 else None
        pilot = GotoPilot((mission.target_x, mission.target_y, up),
                          mission.speed_pref_mps, approach_accel=APPROACH_ACCEL_MPS2)
        return pilot, dict(rec, speed_mps=pilot.speed, target_up=up,
                           approach_accel_mps2=pilot.approach_accel,
                           goal_tol_m=pilot.goal_tol_m)
    act = parse_floats(getattr(args, "action", None), (3, 4), "--action")
    if act is None:
        raise ValueError("--pilot constant needs --action VX,VY,VZ[,YAW] (world frame)")
    pilot = ConstantPilot(dict(zip(("vx", "vy", "vz_up", "yaw_rate"), act)))
    return pilot, dict(rec, action=pilot.action, frame="world, turned to body")


def body_action(vla, state: State, yaw_rate: float | None) -> list[float]:
    """The pilot's world-frame plan as the grant's body-frame 4-vector, with
    `yaw_rate` (rad/s clockwise) - None keeps the pilot's own (the constant
    pilot's action carries one)."""
    a = vla.act(state)
    yr = a.yaw_rate if yaw_rate is None else float(yaw_rate)
    a = Action4D(vx=a.vx, vy=a.vy, vz_up=a.vz_up, yaw_rate=yr)
    return [float(v) for v in to_body(a, state.yaw_deg)]


def identity_record(policy, source: dict, mission, *, yaw_rate: float,
                    seed: int, host: dict | None = None, pilot: dict | None = None,
                    start=None, has_target: bool = True) -> dict:
    """What this node announces on /vla/identity.

    `host` is the machine the VLA runs on (guardrail.manifest.
    collect_host_evidence), read here, not by the Shield node: the grant puts
    the VLA on the Jetson Orin with the Shield, and only this node's own
    host can show it is. The Shield node copies it into hil_evidence.vla_host.
    `pilot` (pilot_from_args's record) names the law and its model id; the
    stub's StubVLA when none is given, as before."""
    pilot = pilot or {"type": "stub_vla", "model_id": MODEL_ID}
    rec = {"model_id": pilot.get("model_id", MODEL_ID), "kind": "stub", "frame": "body",
           "yaw_rate": float(yaw_rate), "seed": seed,
           "policy_id": policy.policy_id,
           "policy_hash": policy.policy_hash,
           "policy_source": source,
           "target": [mission.target_x, mission.target_y] if has_target else None,
           "speed_pref_mps": mission.speed_pref_mps,
           "host": collect_host_evidence() if host is None else host}
    if pilot.get("type") != "stub_vla" or start is not None:
        rec["pilot"] = pilot
    if start is not None:
        rec["start"] = [float(v) for v in start]
    return rec


def make_parser() -> argparse.ArgumentParser:
    # `--target -20,15` (negative X) parses as a value instead of exiting
    # with status 2 (mavlink_adapter_node.join_negative_xy); so do --start
    # and --action since 2026-11-06. allow_abbrev off, as the Shield node's
    # and the VLA bridge's: every argument reaches both nodes, and an
    # abbreviation of one node's flag must not be read as another's.
    ap = XYArgumentParser(allow_abbrev=False)
    ap.add_argument("--policy", default=None)
    ap.add_argument("--bundle", default=None)
    ap.add_argument("--allow-unverified-bundle", action="store_true")
    ap.add_argument("--command", default="fly to the northeast pad at 6 m/s")
    ap.add_argument("--target", default=None, metavar="X,Y[,UP]",
                    help="the target in the policy frame (metres North, East), with "
                         "its altitude (the goto law climbs to it, StubVLA holds it)")
    ap.add_argument("--no-target", action="store_true",
                    help="no target: only --pilot constant flies without one")
    ap.add_argument("--start", default=None, metavar="X,Y[,UP]",
                    help="the policy-frame position of the autopilot's home: the MAVROS "
                         "pose is offset by it (UP: the start altitude, the cruise "
                         "altitude when the target names none)")
    ap.add_argument("--pilot", choices=PILOTS, default="stub_vla",
                    help="the law flown: stub_vla (StubVLA, the default), goto or "
                         "constant (guardrail.vla_stub)")
    ap.add_argument("--action", default=None, metavar="VX,VY,VZ[,YAW]",
                    help="--pilot constant: the world-frame action every tick (m/s "
                         "North, East, up; rad/s clockwise)")
    ap.add_argument("--speed", type=float, default=None)
    ap.add_argument("--yaw-rate", type=float, default=0.0,
                    help="constant yaw rate, rad/s clockwise (0 = hold heading)")
    ap.add_argument("--seed", type=int, default=0)
    return ap


try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float32MultiArray, String
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

if HAVE_ROS:
    class VlaStubNode(Node):
        def __init__(self, args) -> None:
            super().__init__("vla_stub")
            policy, source = load_for_flight(
                args.bundle,
                args.policy or (None if args.bundle else
                                ROOT / "policies" / "sim_demo_policy.yaml"),
                allow_unverified=args.allow_unverified_bundle)
            mission = ConstraintCompiler(policy).parse_command(args.command)
            mission, target, start = mission_from_args(args, mission)
            self.vla, pilot = pilot_from_args(args, mission, target)
            self.start = start
            # The constant pilot flies its own yaw rate; the others --yaw-rate.
            self.yaw_rate = None if pilot["type"] == "constant" else float(args.yaw_rate)
            self.state: State | None = None

            # mavros publishes sensor topics best-effort; QoS must match.
            self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                     self._on_pose, qos_profile_sensor_data)
            self.pub = self.create_publisher(Float32MultiArray, "/vla/action_4d", 10)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_id = self.create_publisher(String, "/vla/identity", latched)
            ident = identity_record(policy, source, mission,
                                    yaw_rate=self.yaw_rate or 0.0, seed=args.seed,
                                    pilot=pilot, start=start,
                                    has_target=not args.no_target)
            self.pub_id.publish(String(data=json.dumps(ident)))
            self.create_timer(0.1, self._tick)                 # 10 Hz - the contract
            self.get_logger().info(
                f"VLA stub up: pilot {pilot['type']} ({pilot['model_id']}), policy "
                f"{policy.policy_id} {policy.policy_hash} "
                f"({source['kind']}, signature {source['signature']}); "
                f"start offset {start}; publishing body-frame /vla/action_4d, "
                f"yaw_rate {'the action' if self.yaw_rate is None else self.yaw_rate}")

        def _on_pose(self, msg: PoseStamped) -> None:
            q = msg.pose.orientation
            # ENU (mavros) -> our up-positive policy frame: x=N, y=E, offset
            # by the policy-frame position of home (--start).
            self.state = pose_state(msg.pose.position.x, msg.pose.position.y,
                                    msg.pose.position.z, (q.x, q.y, q.z, q.w),
                                    self.start)

        def _tick(self) -> None:
            if self.state is None:
                return
            self.pub.publish(Float32MultiArray(
                data=body_action(self.vla, self.state, self.yaw_rate)))


def check_args(args) -> None:
    """Raise ValueError, before ROS starts and before any policy is read, for
    arguments no law can fly: a --start / --target / --action that does not
    parse, a law that lacks what it flies on (pilot_from_args)."""
    from guardrail.compiler import Mission
    probe = Mission(task_text="", target_x=0.0, target_y=0.0, cruise_alt_m=15.0,
                    speed_pref_mps=6.0)
    mission, target, _ = mission_from_args(args, probe)
    pilot_from_args(args, mission, target)


def main(argv: list[str] | None = None) -> None:
    args, ros_args = make_parser().parse_known_args(argv)
    try:
        check_args(args)
    except ValueError as exc:
        raise SystemExit(f"[vla_stub] REFUSED: {exc}")
    if not HAVE_ROS:
        raise SystemExit("ros2_vla_stub_node needs rclpy: source "
                         "/opt/ros/jazzy/setup.bash and use ~/venv-ros/bin/python")
    rclpy.init(args=ros_args)
    node = VlaStubNode(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:                  # noqa: BLE001
        if not isinstance(e, KeyboardInterrupt):
            node.get_logger().error(f"{type(e).__name__}: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
