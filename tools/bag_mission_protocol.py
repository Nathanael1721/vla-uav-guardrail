"""The mission-protocol exchange in a ROS 2 rail run's raw MAVLink, decoded.

    ~/venv-bags/bin/python tools/bag_mission_protocol.py demo/out/<tag> [--all]

The rail's rosbag2 records MAVROS 2's own link to its router in both
directions, /uas1/mavlink_sink (to the autopilot) and /uas1/mavlink_source
(from it), as mavros_msgs/Mavlink (sitl/run_ros2_demo.sh, since 2026-11-02).
This prints, in time order, every MISSION_WRITE_PARTIAL_LIST, MISSION_COUNT,
MISSION_REQUEST(_INT), MISSION_ITEM(_INT), MISSION_ACK and MISSION_SET_CURRENT
with its fields, the autopilot's mission status texts, and a count per
direction and message - the record that shows WHICH message carried a
mission item to ArduPilot (WP3-09: MISSION_ITEM_INT, not MISSION_ITEM) and
how a partial write ended. Written 2026-11-03 for mission_wp_20261103c / d.

Needs `rosbags` (WSL ~/venv-bags) and the mavros_msgs message definition
from ROS 2 Jazzy (/opt/ros/jazzy/share/mavros_msgs/msg/Mavlink.msg). The
payload is unpacked by hand in MAVLink's wire order (the messages here are
fixed-size; a MAVLink 2 zero-trimmed payload is padded back). MISSION_CURRENT
is counted but printed only with --all."""
import struct
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# msgid -> (name, struct format of the MAVLink wire order, field names)
SPECS = {
    38: ("MISSION_WRITE_PARTIAL_LIST", "<hhBBB", ("start_index", "end_index", "tsys", "tcomp", "mission_type")),
    39: ("MISSION_ITEM", "<fffffffHHBBBBBB", ("p1", "p2", "p3", "p4", "x", "y", "z", "seq", "command", "tsys", "tcomp", "frame", "current", "autocontinue", "mission_type")),
    40: ("MISSION_REQUEST", "<HBBB", ("seq", "tsys", "tcomp", "mission_type")),
    41: ("MISSION_SET_CURRENT", "<HBB", ("seq", "tsys", "tcomp")),
    44: ("MISSION_COUNT", "<HBBB", ("count", "tsys", "tcomp", "mission_type")),
    47: ("MISSION_ACK", "<BBBB", ("tsys", "tcomp", "type", "mission_type")),
    51: ("MISSION_REQUEST_INT", "<HBBB", ("seq", "tsys", "tcomp", "mission_type")),
    73: ("MISSION_ITEM_INT", "<ffffiifHHBBBBBB", ("p1", "p2", "p3", "p4", "x", "y", "z", "seq", "command", "tsys", "tcomp", "frame", "current", "autocontinue", "mission_type")),
    42: ("MISSION_CURRENT", "<H", ("seq",)),
    253: ("STATUSTEXT", "<B50s", ("severity", "text")),
}

run = Path(sys.argv[1])
show_all = "--all" in sys.argv
ts = get_typestore(Stores.ROS2_JAZZY)
ts.register(get_types_from_msg(Path("/opt/ros/jazzy/share/mavros_msgs/msg/Mavlink.msg").read_text(),
                               "mavros_msgs/msg/Mavlink"))
rows, counts = [], {}
with AnyReader([run / "bag"], default_typestore=ts) as r:
    conns = [c for c in r.connections if c.topic in ("/uas1/mavlink_sink", "/uas1/mavlink_source")]
    assert conns, "no raw MAVLink topics in this bag"
    for c, t, raw in r.messages(connections=conns):
        msg = ts.deserialize_cdr(raw, "mavros_msgs/msg/Mavlink")
        mid = int(msg.msgid)
        if mid not in SPECS:
            continue
        name, fmt, names = SPECS[mid]
        payload = b"".join(struct.pack("<Q", int(w)) for w in msg.payload64)[: int(msg.len)]
        vals = dict(zip(names, struct.unpack(fmt, payload.ljust(struct.calcsize(fmt), b"\0")[: struct.calcsize(fmt)])))
        d = "->FCU" if c.topic.endswith("sink") else "<-FCU"
        if name == "STATUSTEXT":
            txt = vals["text"].split(b"\0")[0].decode("ascii", "replace")
            if "ission" not in txt and "MISSION" not in txt:
                continue
            vals = {"text": txt}
        counts[f"{d} {name}"] = counts.get(f"{d} {name}", 0) + 1
        rows.append((t, d, name, int(msg.sysid), int(msg.compid), vals))
t0 = rows[0][0] if rows else 0
for t, d, name, s, cpt, vals in rows:
    if name == "MISSION_CURRENT" and not show_all:
        continue
    if name in ("MISSION_ITEM_INT", "MISSION_ITEM"):
        vals = {k: vals[k] for k in ("seq", "command", "frame", "p1", "p2", "x", "y", "z", "mission_type")}
    print(f"{(t - t0) / 1e9:8.3f} {d} {name:28s} {s}/{cpt} {vals}")
print(dict(sorted(counts.items())))
