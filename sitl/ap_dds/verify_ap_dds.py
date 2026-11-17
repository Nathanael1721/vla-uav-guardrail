"""
Check an ArduCopter SITL built with AP_DDS end to end (ARCH-08): the /ap/*
topics arrive, and a velocity setpoint sent through /ap/cmd_vel in GUIDED is
flown. docs/RUNBOOK-ap-dds.md has the commands around it.

    1. topics   wait for /ap/time, /ap/pose/filtered, /ap/navsat (4.5:
                /ap/navsat/navsat0) and /ap/geopose/filtered, then count each
                for --window s: a rate per topic.
    2. GUIDED   /ap/mode_switch (ardupilot_msgs/srv/ModeSwitch, mode 4).
    3. arm      /ap/arm_motors (ardupilot_msgs/srv/ArmMotors), retried while
                the pre-arm checks settle.
    4. takeoff  /ap/experimental/takeoff when the firmware has it (4.6+), else
                MAV_CMD_NAV_TAKEOFF over MAVLink (--mavlink, pymavlink): 4.5.7
                has no DDS takeoff and GUIDED velocity does not lift off.
    5. command  --vel m/s east for --secs s, 10 Hz: straight onto /ap/cmd_vel
                (frame_id "map", ENU), or with --via-adapter as a body setpoint
                on /shield/setpoint through sitl/ap_dds_adapter_node.py (body
                forward, flown along the vehicle's heading).
    6. measure  the mean of /ap/twist/filtered over the second half of the
                command, and the pose displacement's bearing, against what was
                commanded; then a zero setpoint and LAND (mode 9).

Writes <out>/ap_dds_verify.json; exit 0 only when every check passes.

    ~/venv-ros/bin/python sitl/ap_dds/verify_ap_dds.py --out demo/out/ap_dds_verify_01
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SITL = ROOT / "sitl"
for _p in (ROOT, SITL):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from ap_dds_adapter_node import (COPTER_MODE_NUMBER, FRAME_MAP,          # noqa: E402
                                 TOPIC_CMD_VEL, cmd_vel_from_body)
from mavlink_adapter_node import heading_deg_from_enu_quaternion        # noqa: E402

# The topics the row names, with the names each ArduPilot version uses
# (4.5 suffixes the instance: /ap/navsat/navsat0; 4.6+ drops it).
REQUIRED_TOPICS = {
    "time": ("/ap/time",),
    "pose": ("/ap/pose/filtered",),
    "navsat": ("/ap/navsat", "/ap/navsat/navsat0"),
    "geopose": ("/ap/geopose/filtered",),
}
OPTIONAL_TOPICS = {"twist": ("/ap/twist/filtered",)}
SRV_ARM = "/ap/arm_motors"
SRV_MODE = "/ap/mode_switch"
SRV_TAKEOFF = "/ap/experimental/takeoff"
MIN_RATE_HZ = 1.0
SPEED_TOL_MPS = 0.3
BEARING_TOL_DEG = 15.0


# --------------------------------------------------------------------------- #
# Pure checks (tested on any machine: tests/test_ap_dds_adapter.py)
# --------------------------------------------------------------------------- #

def resolve_topics(available: dict, wanted: dict = REQUIRED_TOPICS) -> dict:
    """{key: (name, type) or None}: the first of each key's names the graph has."""
    out = {}
    for key, names in wanted.items():
        hit = next((n for n in names if n in available), None)
        out[key] = (hit, available[hit][0]) if hit else None
    return out


def topic_verdict(resolved: dict, counts: dict, window_s: float,
                  min_hz: float = MIN_RATE_HZ) -> dict:
    """Per required topic: name, messages, rate; ok when it exists and its
    rate is at least min_hz. A topic that is missing is a failure, never a
    zero that reads as fine."""
    rows, ok = {}, True
    for key, hit in resolved.items():
        if hit is None:
            rows[key] = {"name": None, "n": 0, "hz": 0.0, "ok": False,
                         "why": "not on the ROS graph"}
            ok = False
            continue
        n = int(counts.get(key, 0))
        hz = n / window_s if window_s > 0 else 0.0
        good = hz >= min_hz
        rows[key] = {"name": hit[0], "type": hit[1], "n": n, "hz": round(hz, 2),
                     "ok": good}
        ok = ok and good
    return {"ok": ok, "window_s": window_s, "min_hz": min_hz, "topics": rows}


def bearing_deg(d_east: float, d_north: float) -> float:
    """Compass bearing of an ENU displacement, 0 = North, clockwise, [0, 360)."""
    b = math.degrees(math.atan2(d_east, d_north)) % 360.0
    return 0.0 if b >= 360.0 else b


def angle_diff_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def velocity_verdict(cmd_en: tuple, measured_en: tuple | None,
                     displacement_en: tuple | None,
                     speed_tol: float = SPEED_TOL_MPS,
                     bearing_tol: float = BEARING_TOL_DEG) -> dict:
    """Commanded ENU horizontal velocity against the measured mean velocity
    (/ap/twist/filtered) and the displacement's bearing (/ap/pose/filtered).
    Passes when the measured velocity is within speed_tol of the command
    (vector error) and the track is within bearing_tol of the command's
    bearing. A missing measurement fails."""
    ce, cn = cmd_en
    cmd_speed = math.hypot(ce, cn)
    cmd_bearing = bearing_deg(ce, cn)
    out = {"cmd_enu": [round(ce, 3), round(cn, 3)], "cmd_speed": round(cmd_speed, 3),
           "cmd_bearing_deg": round(cmd_bearing, 1), "speed_tol": speed_tol,
           "bearing_tol_deg": bearing_tol}
    ok = cmd_speed > 0
    if measured_en is None:
        out["measured"] = None
        ok = False
    else:
        me, mn = measured_en
        err = math.hypot(me - ce, mn - cn)
        out["measured_enu"] = [round(me, 3), round(mn, 3)]
        out["velocity_error"] = round(err, 3)
        ok = ok and err <= speed_tol
    if displacement_en is None:
        out["track"] = None
        ok = False
    else:
        de, dn = displacement_en
        tb = bearing_deg(de, dn)
        out["displacement_enu"] = [round(de, 2), round(dn, 2)]
        out["track_bearing_deg"] = round(tb, 1)
        out["bearing_error_deg"] = round(angle_diff_deg(tb, cmd_bearing), 1)
        ok = ok and math.hypot(de, dn) > 0.5 and out["bearing_error_deg"] <= bearing_tol
    out["ok"] = ok
    return out


def expected_enu(via_adapter: bool, vel: float, heading_deg: float | None) -> tuple:
    """The ENU horizontal velocity a run commands: --vel east straight onto
    /ap/cmd_vel, or --vel body-forward through the adapter, which flies it
    along the heading (cmd_vel_from_body, the adapter's own conversion)."""
    if not via_adapter:
        return (vel, 0.0)
    if heading_deg is None:
        raise ValueError("no heading: the pose topic has not been read")
    cv = cmd_vel_from_body([vel, 0.0, 0.0, 0.0], heading_deg)
    return (cv["linear"][0], cv["linear"][1])


# --------------------------------------------------------------------------- #
# The ROS 2 run (WSL, beside the DDS SITL and the micro-ROS agent)
# --------------------------------------------------------------------------- #

def _mavlink_takeoff(url: str, alt: float) -> str:
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(url)
    if m.wait_heartbeat(timeout=10) is None:
        return "no MAVLink heartbeat"
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, float(alt))
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    m.close()
    return f"COMMAND_ACK result {ack.result}" if ack else "no COMMAND_ACK"


def run(args) -> dict:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message, get_service
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Float32MultiArray

    rclpy.init()
    node = rclpy.create_node("ap_dds_verify")
    report: dict = {"started": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "via_adapter": bool(args.via_adapter)}

    def spin_for(s: float) -> None:
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    # 1. topics
    deadline = time.monotonic() + args.discover_s
    resolved = {}
    while time.monotonic() < deadline:
        avail = dict(node.get_topic_names_and_types())
        resolved = resolve_topics(avail, {**REQUIRED_TOPICS, **OPTIONAL_TOPICS})
        if all(resolved[k] for k in REQUIRED_TOPICS):
            break
        spin_for(0.5)
    report["ap_topics_seen"] = sorted(n for n in dict(node.get_topic_names_and_types())
                                      if n.startswith("/ap/"))
    counts = {k: 0 for k in resolved}
    state = {"pose": None, "twist": []}

    def make_cb(key):
        def cb(msg):
            counts[key] += 1
            if key == "pose":
                state["pose"] = msg
            elif key == "twist" and state.get("collect"):
                state["twist"].append((msg.twist.linear.x, msg.twist.linear.y))
        return cb

    for key, hit in resolved.items():
        if hit:
            node.create_subscription(get_message(hit[1]), hit[0], make_cb(key),
                                     qos_profile_sensor_data)
    spin_for(args.window)
    req = {k: v for k, v in resolved.items() if k in REQUIRED_TOPICS}
    report["topics"] = topic_verdict(req, counts, args.window)
    if args.topics_only or not report["topics"]["ok"]:
        report["ok"] = report["topics"]["ok"]
        node.destroy_node()
        rclpy.shutdown()
        return report

    def call(name, srv_type, **fields):
        cli = node.create_client(get_service(srv_type), name)
        if not cli.wait_for_service(timeout_sec=5.0):
            return None
        fut = cli.call_async(cli.srv_type.Request(**fields))
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    # 2. GUIDED
    r = call(SRV_MODE, "ardupilot_msgs/srv/ModeSwitch", mode=COPTER_MODE_NUMBER["GUIDED"])
    report["guided"] = {"status": bool(r and r.status),
                        "curr_mode": int(r.curr_mode) if r else None}
    # 3. arm, retried while pre-arm checks settle (EKF, GPS)
    armed, t0 = False, time.monotonic()
    while not armed and time.monotonic() - t0 < args.arm_timeout:
        r = call(SRV_ARM, "ardupilot_msgs/srv/ArmMotors", arm=True)
        armed = bool(r and r.result)
        if not armed:
            spin_for(2.0)
    report["armed"] = armed
    report["arm_s"] = round(time.monotonic() - t0, 1)
    if not (report["guided"]["status"] and armed):
        report["ok"] = False
        node.destroy_node()
        rclpy.shutdown()
        return report
    # 4. takeoff
    srvs = dict(node.get_service_names_and_types())
    if SRV_TAKEOFF in srvs:
        r = call(SRV_TAKEOFF, "ardupilot_msgs/srv/Takeoff", alt=float(args.alt))
        report["takeoff"] = {"via": "ap_dds", "status": bool(r and r.status)}
    else:
        report["takeoff"] = {"via": "mavlink", "result": _mavlink_takeoff(args.mavlink,
                                                                           args.alt)}
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.climb_timeout:
        spin_for(0.5)
        p = state["pose"]
        if p is not None and p.pose.position.z >= 0.9 * args.alt:
            break
    p = state["pose"]
    report["climb"] = {"z_m": round(p.pose.position.z, 2) if p else None,
                       "s": round(time.monotonic() - t0, 1)}
    # 5. command
    q = p.pose.orientation
    heading = heading_deg_from_enu_quaternion(q.x, q.y, q.z, q.w)
    cmd = expected_enu(args.via_adapter, args.vel, heading)
    report["heading_deg"] = round(heading, 1)
    if args.via_adapter:
        pub = node.create_publisher(Float32MultiArray, "/shield/setpoint", 10)

        def send(v):
            pub.publish(Float32MultiArray(data=[float(v), 0.0, 0.0, 0.0]))
    else:
        pub = node.create_publisher(TwistStamped, TOPIC_CMD_VEL, 10)

        def send(v):
            tw = TwistStamped()
            tw.header.stamp = node.get_clock().now().to_msg()
            tw.header.frame_id = FRAME_MAP
            tw.twist.linear.x = float(v)          # ENU: x = east
            pub.publish(tw)
    p0 = (p.pose.position.x, p.pose.position.y)
    n_ticks = int(args.secs * 10)
    for i in range(n_ticks):
        state["collect"] = i >= n_ticks // 2
        send(args.vel)
        spin_for(0.1)
    state["collect"] = False
    p1 = state["pose"]
    disp = (p1.pose.position.x - p0[0], p1.pose.position.y - p0[1])
    tw = state["twist"]
    meas = ((sum(t[0] for t in tw) / len(tw), sum(t[1] for t in tw) / len(tw))
            if tw else None)
    report["velocity"] = velocity_verdict(cmd, meas, disp)
    report["velocity"]["twist_samples"] = len(tw)
    # 6. stop and land
    for _ in range(10):
        send(0.0)
        spin_for(0.1)
    r = call(SRV_MODE, "ardupilot_msgs/srv/ModeSwitch", mode=COPTER_MODE_NUMBER["LAND"])
    report["land"] = {"status": bool(r and r.status)}
    report["ok"] = bool(report["topics"]["ok"] and report["velocity"]["ok"])
    node.destroy_node()
    rclpy.shutdown()
    return report


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="AP_DDS SITL check (ARCH-08)")
    ap.add_argument("--out", required=True, help="directory for ap_dds_verify.json")
    ap.add_argument("--topics-only", action="store_true",
                    help="stop after the topic check (no arming, no flight)")
    ap.add_argument("--via-adapter", action="store_true",
                    help="command through /shield/setpoint and ap_dds_adapter_node.py")
    ap.add_argument("--vel", type=float, default=1.0, help="m/s (default 1.0)")
    ap.add_argument("--secs", type=float, default=6.0, help="command length, s")
    ap.add_argument("--alt", type=float, default=10.0, help="takeoff altitude, m")
    ap.add_argument("--window", type=float, default=5.0, help="topic count window, s")
    ap.add_argument("--discover-s", type=float, default=30.0)
    ap.add_argument("--arm-timeout", type=float, default=90.0)
    ap.add_argument("--climb-timeout", type=float, default=40.0)
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5760",
                    help="MAVLink URL for the takeoff on firmware without the DDS "
                         "takeoff service (4.5.7)")
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    out = Path(args.out)
    report = run(args)
    out.mkdir(parents=True, exist_ok=True)
    path = out / "ap_dds_verify.json"
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(report, fh, indent=2)
    print(json.dumps(report, indent=2))
    print(f"\n{'PASS' if report.get('ok') else 'FAIL'}  -> {path}")
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
