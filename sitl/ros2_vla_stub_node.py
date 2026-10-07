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

Run (after `source /opt/ros/jazzy/setup.bash`):

    ~/venv-ros/bin/python sitl/ros2_vla_stub_node.py [--bundle B | --policy P] [--yaw-rate 0.2]
"""
from __future__ import annotations

import argparse
import json
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
from guardrail.vla_stub import StubVLA                         # noqa: E402
from mavlink_adapter_node import heading_deg_from_enu_quaternion  # noqa: E402

MODEL_ID = "guardrail.vla_stub.StubVLA"


def body_action(vla: StubVLA, state: State, yaw_rate: float) -> list[float]:
    """The stub's world-frame plan as the grant's body-frame 4-vector."""
    a = vla.act(state)
    a = Action4D(vx=a.vx, vy=a.vy, vz_up=a.vz_up, yaw_rate=float(yaw_rate))
    return [float(v) for v in to_body(a, state.yaw_deg)]


def identity_record(policy, source: dict, mission, *, yaw_rate: float,
                    seed: int, host: dict | None = None) -> dict:
    """What this node announces on /vla/identity.

    `host` is the machine the VLA runs on (guardrail.manifest.
    collect_host_evidence), read here, not by the Shield node: the grant puts
    the VLA on the Jetson Orin with the Shield, and only this node's own
    host can show it is. The Shield node copies it into hil_evidence.vla_host.
    """
    return {"model_id": MODEL_ID, "kind": "stub", "frame": "body",
            "yaw_rate": float(yaw_rate), "seed": seed,
            "policy_id": policy.policy_id,
            "policy_hash": policy.policy_hash,
            "policy_source": source,
            "target": [mission.target_x, mission.target_y],
            "speed_pref_mps": mission.speed_pref_mps,
            "host": collect_host_evidence() if host is None else host}


def make_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default=None)
    ap.add_argument("--bundle", default=None)
    ap.add_argument("--allow-unverified-bundle", action="store_true")
    ap.add_argument("--command", default="fly to the northeast pad at 6 m/s")
    ap.add_argument("--target", default=None, metavar="X,Y")
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
            if args.target:
                x, y = (float(v) for v in args.target.split(","))
                mission = mission.model_copy(update={"target_x": x, "target_y": y})
            if args.speed is not None:
                mission = mission.model_copy(update={"speed_pref_mps": args.speed})
            self.vla = StubVLA(mission)
            self.yaw_rate = float(args.yaw_rate)
            self.state: State | None = None

            # mavros publishes sensor topics best-effort; QoS must match.
            self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                     self._on_pose, qos_profile_sensor_data)
            self.pub = self.create_publisher(Float32MultiArray, "/vla/action_4d", 10)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_id = self.create_publisher(String, "/vla/identity", latched)
            ident = identity_record(policy, source, mission,
                                    yaw_rate=self.yaw_rate, seed=args.seed)
            self.pub_id.publish(String(data=json.dumps(ident)))
            self.create_timer(0.1, self._tick)                 # 10 Hz - the contract
            self.get_logger().info(
                f"VLA stub up: policy {policy.policy_id} {policy.policy_hash} "
                f"({source['kind']}, signature {source['signature']}); "
                f"publishing body-frame /vla/action_4d, yaw_rate {self.yaw_rate}")

        def _on_pose(self, msg: PoseStamped) -> None:
            q = msg.pose.orientation
            # ENU (mavros) -> our up-positive local frame: x=N, y=E.
            self.state = State(x=msg.pose.position.y, y=msg.pose.position.x,
                               up=msg.pose.position.z,
                               yaw_deg=heading_deg_from_enu_quaternion(
                                   q.x, q.y, q.z, q.w))

        def _tick(self) -> None:
            if self.state is None:
                return
            self.pub.publish(Float32MultiArray(
                data=body_action(self.vla, self.state, self.yaw_rate)))


def main(argv: list[str] | None = None) -> None:
    args, ros_args = make_parser().parse_known_args(argv)
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
