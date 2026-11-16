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

THE PREFIX ARM (2026-11-16, audit card WP4-15). `--prefix on|off` flies the
chosen law through guardrail.prefix_consumer.PrefixConsumer, the grant's
default action-set adapter: `on` with the Prefix Compiler's CSP - compiled
ONCE, before ROS starts, from this node's own policy (its bundle), for the
mission its arguments name (--start, --target, --speed) at --clock-start,
and refused unless its action sets hold for the whole flight (--max-s):
static policies only - and `off` with none, the same pilot without its
Prefix. Either way the node announces the consumer's own model id
(guardrail.prefix_consumer.MODEL_ID) and, under `prefix` on /vla/identity,
the arm, the law it wraps with that law's own source pin (`wraps_model_hash`:
the manifest's vla_model_hash pins the consumer's module only) and the
identity of the CSP it flies (its policy
hash, its action sets' SHA-256, its content hash); the CSP is written to
demo/out/<--tag>/vla_prefix/csp_g0.json. Without `--prefix` - every mission
flown until this date - nothing here runs: the law is published as it
always was and the identity record is the one it was, key for key.

Run (after `source /opt/ros/jazzy/setup.bash`):

    ~/venv-ros/bin/python sitl/ros2_vla_stub_node.py [--bundle B | --policy P] [--yaw-rate 0.2]
    ~/venv-ros/bin/python sitl/ros2_vla_stub_node.py --pilot goto --start=-200,0,15 --target=-80,0,15 --speed 4
    ~/venv-ros/bin/python sitl/ros2_vla_stub_node.py --pilot constant --action=3,0,0,0 --no-target
    ~/venv-ros/bin/python sitl/ros2_vla_stub_node.py --pilot goto --start=-20,15,15 --target=40,15,15 --speed 4 --prefix on --tag t1
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import Action4D, State                          # noqa: E402
from guardrail import prefix_consumer as PC                    # noqa: E402
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


def clock_start_from(text: str | None) -> datetime | None:
    """--clock-start (the Shield node's scripted rule clock) as the instant
    the prefix arm's CSP is compiled at, read by the Shield node's own parser
    (guardrail.replay.parse_clock_start), so both nodes read one instant;
    None for none (no clock: every rule is read as in force). Raises
    ValueError for text that is not a time."""
    if text is None or text == "":
        return None
    from guardrail.replay import parse_clock_start
    return parse_clock_start(text)


def flight_length_from(text) -> float | None:
    """--max-s (the Shield node's flight length, seconds) as the time the
    prefix arm's CSP must hold over; None for none. Raises ValueError for
    anything but a positive finite number."""
    if text is None or text == "":
        return None
    try:
        s = float(text)
    except (TypeError, ValueError):
        raise ValueError(f"--max-s must be a positive number, got {text!r}") from None
    if not (math.isfinite(s) and s > 0):
        raise ValueError(f"--max-s must be a positive number, got {text!r}")
    return s


def prefix_pilot(args, policy, mission, pilot, pilot_rec: dict, start=None
                 ) -> tuple[object, dict, dict | None, object]:
    """(pilot, its record, the /vla/identity `prefix` record, the CSP) for
    --prefix. None: the pilot and its record as they are, no prefix record,
    no CSP - the node as it always flew. `on`: the pilot inside
    guardrail.prefix_consumer.PrefixConsumer with the CSP compiled here from
    `policy` for the mission (the start, target, speed and cruise altitude
    the arguments gave it; a pilot with --no-target has its start as its
    target) at --clock-start, static over --max-s. `off`: the same consumer
    with no CSP. The record names the consumer's model id and the law it
    wraps. Raises ValueError (and the compiler's own refusals) when no such
    CSP exists, before anything flies."""
    arm = getattr(args, "prefix", None)
    if arm is None:
        return pilot, pilot_rec, None, None
    wraps = pilot_rec["model_id"]
    csp = None
    if arm == "on":
        s = ((mission.start_x, mission.start_y) if start is None
             else (float(start[0]), float(start[1])))
        target = (None if getattr(args, "no_target", False)
                  else (mission.target_x, mission.target_y))
        cm = PC.consumer_mission(s, target, mission.speed_pref_mps,
                                 cruise_alt_m=mission.cruise_alt_m)
        csp = PC.static_csp(policy, cm,
                            now=clock_start_from(getattr(args, "clock_start", None)),
                            duration_s=flight_length_from(getattr(args, "max_s", None)))
    flown = PC.PrefixConsumer(pilot, csp, policy if csp is not None else None, wraps=wraps)
    rec = dict(pilot_rec, model_id=PC.MODEL_ID, wraps=wraps, prefix=arm)
    return flown, rec, flown.record(), csp


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
                    start=None, has_target: bool = True,
                    prefix: dict | None = None) -> dict:
    """What this node announces on /vla/identity.

    `host` is the machine the VLA runs on (guardrail.manifest.
    collect_host_evidence), read here, not by the Shield node: the grant puts
    the VLA on the Jetson Orin with the Shield, and only this node's own
    host can show it is. The Shield node copies it into hil_evidence.vla_host.
    `pilot` (pilot_from_args's record) names the law and its model id; the
    stub's StubVLA when none is given, as before. `prefix` (prefix_pilot's
    record, a --prefix mission only) is announced under its own key, with
    the file the CSP was written to; without it the record has no such key."""
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
    if prefix is not None:
        rec["prefix"] = prefix
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
    # THE PREFIX ARM (module docstring). --clock-start, --max-s and --tag are
    # the Shield node's flags too (both nodes get every argument); this node
    # reads them only for a prefix arm: without --prefix their values are
    # neither converted nor checked here, as when this parser did not know
    # them.
    ap.add_argument("--prefix", choices=("on", "off"), default=None,
                    help="fly the law through the prefix consumer (guardrail."
                         "prefix_consumer): on, with the Prefix Compiler's CSP for this "
                         "mission, compiled once from this node's policy; off, without "
                         "it. Default: no consumer, the law as it always flew")
    ap.add_argument("--clock-start", default=None,
                    help="--prefix on: the instant the CSP is compiled at (the Shield "
                         "node's scripted rule clock, ISO local time); none: no clock, "
                         "every rule in force")
    ap.add_argument("--max-s", default=None,
                    help="--prefix on: the flight length the CSP's action sets must "
                         "hold over (refused otherwise: static policies only)")
    ap.add_argument("--tag", default=None,
                    help="--prefix on: the run directory demo/out/<tag>, where the CSP "
                         "is written (vla_prefix/csp_g0.json)")
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
            # THE PREFIX ARM (module docstring): nothing changes without
            # --prefix. main() has compiled the same CSP once already
            # (prefix_preflight), so a refusal never reaches this point.
            self.vla, pilot, prefix, csp = prefix_pilot(args, policy, mission, self.vla,
                                                        pilot, start)
            if csp is not None:
                prefix = dict(prefix, csp_file=write_prefix_csp(args, csp))
            self._prefix = prefix
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
                                    has_target=not args.no_target, prefix=prefix)
            self.pub_id.publish(String(data=json.dumps(ident)))
            self.create_timer(0.1, self._tick)                 # 10 Hz - the contract
            if prefix is not None:
                self.create_timer(10.0, self._prefix_stats)
                self.get_logger().info(
                    f"prefix arm {prefix['arm']}: {prefix['id']} around {prefix['wraps']}, "
                    f"CSP action sets {prefix['action_set_sha256']} "
                    f"(file {prefix.get('csp_file')})")
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

        def _prefix_stats(self) -> None:
            # What the consumer did so far (its tick counts), for the console
            # log of a prefix mission.
            self.get_logger().info(f"prefix arm {self._prefix['arm']}: {self.vla.stats}")


def check_args(args) -> None:
    """Raise ValueError, before ROS starts and before any policy is read, for
    arguments no law can fly: a --start / --target / --action that does not
    parse, a law that lacks what it flies on (pilot_from_args) and, for a
    prefix arm only, a --clock-start that is no time, a --max-s that is no
    flight length, a --tag that is a path."""
    from guardrail.compiler import Mission
    probe = Mission(task_text="", target_x=0.0, target_y=0.0, cruise_alt_m=15.0,
                    speed_pref_mps=6.0)
    mission, target, _ = mission_from_args(args, probe)
    pilot_from_args(args, mission, target)
    if getattr(args, "prefix", None) is None:
        return
    # A prefix arm only: the three flags it reads besides the mission's.
    clock_start_from(getattr(args, "clock_start", None))
    flight_length_from(getattr(args, "max_s", None))
    tag = getattr(args, "tag", None)
    if tag is not None and (not tag or "/" in tag or "\\" in tag or tag.startswith(".")):
        raise ValueError(f"--tag {tag!r} must be a plain directory name")


def write_prefix_csp(args, csp) -> str | None:
    """The prefix-on arm's CSP written under the run directory the mission's
    --tag names (demo/out/<tag>/vla_prefix/csp_g0.json), as a repo-relative
    path; None without a --tag (nothing is written: the CSP is still
    announced by its hashes)."""
    tag = getattr(args, "tag", None)
    if not tag:
        return None
    return PC.write_csp(ROOT / "demo" / "out" / tag, csp).relative_to(ROOT).as_posix()


def prefix_preflight(args) -> dict | None:
    """The --prefix arm's record, built once before ROS starts from the
    policy and mission the node will build again (the same calls, in the
    same order), so a CSP that cannot be compiled - a rule switching in
    flight, the P0 rules over the token budget - refuses the mission here
    instead of killing the node after launch. None without --prefix: no
    policy is read."""
    if getattr(args, "prefix", None) is None:
        return None
    policy, _ = load_for_flight(
        args.bundle,
        args.policy or (None if args.bundle else ROOT / "policies" / "sim_demo_policy.yaml"),
        allow_unverified=args.allow_unverified_bundle)
    mission = ConstraintCompiler(policy).parse_command(args.command)
    mission, target, start = mission_from_args(args, mission)
    pilot, rec = pilot_from_args(args, mission, target)
    return prefix_pilot(args, policy, mission, pilot, rec, start)[2]


def main(argv: list[str] | None = None) -> None:
    args, ros_args = make_parser().parse_known_args(argv)
    try:
        check_args(args)
        prefix_preflight(args)
    except ValueError as exc:
        raise SystemExit(f"[vla_stub] REFUSED: {exc}")
    except Exception as exc:                                     # noqa: BLE001
        if getattr(args, "prefix", None) is None:
            raise
        # The compiler's own refusals of a prefix arm's CSP (CSPBudgetExceeded
        # and CSPUnenforcedRules are deliberately not ValueErrors).
        raise SystemExit(f"[vla_stub] REFUSED: --prefix {args.prefix}: "
                         f"{type(exc).__name__}: {exc}")
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
