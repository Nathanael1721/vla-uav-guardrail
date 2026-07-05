"""MAVLink adapter node — Shield setpoint -> MAVROS 2 (the body->NED boundary).

* subscribes ``/shield/setpoint`` (``std_msgs/Float32MultiArray`` = body [vx,vy,vz,yaw_rate])
* subscribes ``/mavros/global_position/compass_hdg`` (``std_msgs/Float64``, deg) for heading
* publishes  ``/mavros/setpoint_raw/local`` (``mavros_msgs/PositionTarget``, local-NED velocity)

The body->local-NED conversion uses ``vlaguard_common.frames.body_to_local_ned``
— the one and only place this conversion happens (safety-shield.md). Mode
escalation (SET_MODE GUIDED/LOITER/RTL/LAND via the MAVROS service) is a Phase-2
addition; the Phase-1 slice forwards velocity setpoints only.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64

from mavros_msgs.msg import PositionTarget  # isort: skip

from vlaguard_common import Action4D, body_to_local_ned

# PositionTarget bit-mask: ignore position + acceleration + yaw (use yaw_rate),
# i.e. command velocity + yaw_rate only.
_IGNORE = (
    PositionTarget.IGNORE_PX
    | PositionTarget.IGNORE_PY
    | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX
    | PositionTarget.IGNORE_AFY
    | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW
)


class MavlinkAdapter(Node):
    def __init__(self) -> None:
        super().__init__("mavlink_adapter")
        self._yaw = 0.0
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg", self._on_hdg, 10)
        self.create_subscription(Float32MultiArray, "/shield/setpoint", self._on_setpoint, 10)
        self._pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)

    def _on_hdg(self, msg: Float64) -> None:
        self._yaw = math.radians(msg.data)

    def _on_setpoint(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warn("dropping malformed /shield/setpoint (<4 fields)")
            return
        action = Action4D(vx=msg.data[0], vy=msg.data[1], vz=msg.data[2], yaw_rate=msg.data[3])
        v_north, v_east, v_down, yaw_rate = body_to_local_ned(action, self._yaw)

        pt = PositionTarget()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pt.type_mask = _IGNORE
        pt.velocity.x = v_north
        pt.velocity.y = v_east
        pt.velocity.z = v_down
        pt.yaw_rate = yaw_rate
        self._pub.publish(pt)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MavlinkAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
