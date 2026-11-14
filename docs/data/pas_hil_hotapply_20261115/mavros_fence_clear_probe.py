"""Live probe on the AGX: can the fence list be cleared through MAVROS 2's
raw path (/uas1/mavlink_sink, MISSION_CLEAR_ALL mission_type=FENCE)?"""
import struct, sys, time
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Mavlink, Waypoint
from mavros_msgs.srv import WaypointPush, WaypointPull
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data

def x25(data, crc=0xFFFF):
    for b in data:
        t = b ^ (crc & 0xFF); t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc

rclpy.init(); n = Node("fence_clear_probe")
pub = n.create_publisher(Mavlink, "/uas1/mavlink_sink", 10)
push = n.create_client(WaypointPush, "/mavros/geofence/push")
pull = n.create_client(WaypointPull, "/mavros/geofence/pull")
gp = n.create_client(GetParameters, "/mavros/get_parameters")
fix = {}
n.create_subscription(NavSatFix, "/mavros/global_position/global",
                      lambda m: fix.update(lat=m.latitude, lon=m.longitude), qos_profile_sensor_data)

def call(cli, req, t=10.0):
    if not cli.wait_for_service(timeout_sec=t): return None
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=t); return f.result()

r = call(gp, GetParameters.Request(names=["system_id", "component_id", "target_system_id", "target_component_id"]))
ids = [v.integer_value for v in r.values]
print("mavros ids sys/comp/tsys/tcomp", ids)
t0 = time.time()
while "lat" not in fix and time.time() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.2)
print("fix", fix)
p = call(pull, WaypointPull.Request()); print("pull before", p.success, p.wp_received)
wp = Waypoint(); wp.frame = 3; wp.command = 5004; wp.autocontinue = True; wp.param1 = 5.0
wp.x_lat = fix["lat"] + 0.005; wp.y_long = fix["lon"]
r = call(push, WaypointPush.Request(start_index=0, waypoints=[wp])); print("push 1", r.success, r.wp_transfered)
p = call(pull, WaypointPull.Request()); print("pull after push", p.success, p.wp_received)
t1=time.time(); r = call(push, WaypointPush.Request(start_index=0, waypoints=[])); print("push EMPTY", r.success, r.wp_transfered, "ms", round((time.time()-t1)*1000,1))
p = call(pull, WaypointPull.Request()); print("pull after empty push", p.success, p.wp_received)
r = call(push, WaypointPush.Request(start_index=0, waypoints=[wp])); print("push 1 again", r.success, r.wp_transfered)
p = call(pull, WaypointPull.Request()); print("pull before CLEAR_ALL", p.success, p.wp_received)
sysid, compid, tsys, tcomp = ids
payload = bytes([tsys, tcomp, 1])          # MISSION_CLEAR_ALL: target_system, target_component, mission_type FENCE
hdr = struct.pack("<BBBBBB", len(payload), 0, 0, 0, sysid, compid) + struct.pack("<I", 45)[:3]
crc = x25(bytes([232]), x25(hdr + payload))
m = Mavlink(); m.header.stamp = n.get_clock().now().to_msg(); m.framing_status = 1; m.magic = 253
m.len = len(payload); m.seq = 0; m.sysid = sysid; m.compid = compid; m.msgid = 45; m.checksum = crc
m.payload64 = list(struct.unpack("<Q", payload.ljust(8, b"\0")))
time.sleep(0.5)
print("subs on sink:", n.count_subscribers("/uas1/mavlink_sink"))
pub.publish(m); t0 = time.time()
while time.time() - t0 < 1.0: rclpy.spin_once(n, timeout_sec=0.1)
p = call(pull, WaypointPull.Request()); print("pull after CLEAR_ALL", p.success, p.wp_received)
