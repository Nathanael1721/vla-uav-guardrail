"""Safety Shield ROS 2 node (Humble) — thin wiring around the pure core.

Position in the graph (architecture-constraints.md / data-flow.md): between the
VLA's action publisher and the MAVLink adapter.

* subscribes ``/vla/action_4d``  (``std_msgs/Float32MultiArray`` = [vx,vy,vz,yaw_rate], 10 Hz)
* subscribes ``/mavros/global_position/global`` (``sensor_msgs/NavSatFix``)
* subscribes ``/mavros/global_position/compass_hdg`` (``std_msgs/Float64``, degrees)
* publishes  ``/shield/setpoint`` (post-Shield 4-D action, body frame)
* publishes  ``/shield/intercept`` (``std_msgs/String`` JSON event on any intercept)

All policy logic lives in ``safety_shield`` (the ROS-free core); this file only
translates messages <-> the core's dataclasses. The body->local-NED conversion is
NOT done here — it is the MAVLink adapter's single responsibility.

Note: AGL altitude is derived as ``NavSatFix.altitude - home_alt_m`` (a node
parameter); replacing this with a proper terrain/home reference is a Phase-2 task.
"""

from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64, String

from sensor_msgs.msg import NavSatFix  # isort: skip

from policy_dsl import load_bundle
from safety_shield import AuditLog, SafetyShield, VehicleState
from vlaguard_common import Action4D


class ShieldNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_shield_node")
        self.declare_parameter("bundle_path", "")
        self.declare_parameter("audit_path", "shield_audit.jsonl")
        self.declare_parameter("home_alt_m", 0.0)

        bundle_path = self.get_parameter("bundle_path").value
        if not bundle_path:
            raise RuntimeError("safety_shield_node requires the 'bundle_path' parameter")
        ir = load_bundle(bundle_path)
        audit = AuditLog(
            self.get_parameter("audit_path").value, ir.policy_hash, ir.generation
        )
        self.shield = SafetyShield(ir, audit=audit)
        self.get_logger().info(f"loaded policy bundle {ir.policy_id} hash={ir.policy_hash}")

        self._lat = 0.0
        self._lon = 0.0
        self._alt = 0.0
        self._yaw = 0.0
        self._home_alt = float(self.get_parameter("home_alt_m").value)

        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._on_fix, 10)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg", self._on_hdg, 10)
        self.create_subscription(Float32MultiArray, "/vla/action_4d", self._on_action, 10)
        self._sp_pub = self.create_publisher(Float32MultiArray, "/shield/setpoint", 10)
        self._ev_pub = self.create_publisher(String, "/shield/intercept", 10)

    def _on_fix(self, msg: NavSatFix) -> None:
        self._lat, self._lon, self._alt = msg.latitude, msg.longitude, msg.altitude

    def _on_hdg(self, msg: Float64) -> None:
        # compass_hdg is degrees from North, clockwise — matches the NED yaw the core expects
        self._yaw = math.radians(msg.data)

    def _on_action(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warn("dropping malformed /vla/action_4d (<4 fields)")
            return
        action = Action4D(vx=msg.data[0], vy=msg.data[1], vz=msg.data[2], yaw_rate=msg.data[3])
        state = VehicleState(
            lat=self._lat, lon=self._lon, alt_agl_m=self._alt - self._home_alt, yaw_rad=self._yaw
        )
        ts = self.get_clock().now().to_msg()
        decision = self.shield.tick(state, action, ts=f"{ts.sec}.{ts.nanosec:09d}")

        out = Float32MultiArray()
        e = decision.emitted_action
        out.data = [e.vx, e.vy, e.vz, e.yaw_rate]
        self._sp_pub.publish(out)

        if decision.intercepted:
            self._ev_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "state": str(decision.state_after),
                            "violations": [v.rule_id for v in decision.violations],
                        }
                    )
                )
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ShieldNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
