"""ROS 2 wrapper around the in-house VLA stub (demo/sim use only).

Publishes ``/vla/action_4d`` (``std_msgs/Float32MultiArray`` = [vx,vy,vz,yaw_rate])
at 10 Hz, steering toward a target waypoint read from node parameters. Swapped for
a real VLA backend in the hil / flight topologies; the wire format is identical so
the Shield node is agnostic to which one is upstream.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64

from sensor_msgs.msg import NavSatFix  # isort: skip

from safety_shield import VehicleState

from demo.vla_stub import Target, bearing_rad, stub_action


class RosVlaStub(Node):
    def __init__(self) -> None:
        super().__init__("ros_vla_stub")
        self.declare_parameter("target_lat", 25.04320)
        self.declare_parameter("target_lon", 121.531012)
        self.declare_parameter("target_alt_agl_m", 50.0)
        self.declare_parameter("cruise_mps", 5.0)
        self._target = Target(
            lat=float(self.get_parameter("target_lat").value),
            lon=float(self.get_parameter("target_lon").value),
            alt_agl_m=float(self.get_parameter("target_alt_agl_m").value),
        )
        self._cruise = float(self.get_parameter("cruise_mps").value)
        self._lat = self._lon = self._alt = self._yaw = 0.0

        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._on_fix, 10)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg", self._on_hdg, 10)
        self._pub = self.create_publisher(Float32MultiArray, "/vla/action_4d", 10)
        self.create_timer(0.1, self._tick)  # 10 Hz

    def _on_fix(self, msg: NavSatFix) -> None:
        self._lat, self._lon, self._alt = msg.latitude, msg.longitude, msg.altitude

    def _on_hdg(self, msg: Float64) -> None:
        self._yaw = math.radians(msg.data)

    def _tick(self) -> None:
        state = VehicleState(lat=self._lat, lon=self._lon, alt_agl_m=self._alt, yaw_rad=self._yaw)
        action = stub_action(state, self._target, self._cruise)
        # yaw the body toward the target so vx is "toward target" (sim convenience)
        _ = bearing_rad(state, self._target)
        self._pub.publish(
            Float32MultiArray(data=[action.vx, action.vy, action.vz, action.yaw_rate])
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RosVlaStub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
