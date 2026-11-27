"""The episode bundle's episode.bag2/: the run's rosbag2 recording, trimmed
to begin at its manifest start frame (ARCH-22, 2026-11-28).

The grant's data-flow page: the six manifest fields are "emitted by the
orchestrator at episode start and stored as the first frame of every
rosbag". The rails' recorder starts before the Guardrail nodes and records
MAVROS from its own start (sitl/run_ros2_demo.sh, deploy/docker/
entrypoint.sh), so in the run's bag/ MAVROS messages come before the Shield
node's start frame on /guardrail/manifest (smREL072_001: 24 messages over
4.0 s; since 2026-11-28 the start frame goes out at arming, later still).
bag/ stays as recorded - the replay pack, the exporters and
tools/replay_from_bag.py read it - and the bundle's episode.bag2/ is written
from it here (trim_episode_bag):

  1. the start frame, the bag's first message, at its own receive time;
  2. the LATCHED VALUES a recorder joining at that moment would be handed:
     for each topic whose publisher offers transient_local durability (the
     Shield's fences, the adapter's status, /vla/identity, MAVROS's state,
     extended state and home), the last message before the start frame,
     stamped 1 ns apart right after it, in the order they were received;
     their receive times are kept in the bag's custom_data (CARRIED_KEY);
  3. every message after the start frame, as recorded (the end frame among
     them).

Every other message before the start frame is left out and counted. The
connections (topic, type, definition, hash, offered QoS) are the
recording's, in the same storage (mcap or sqlite3); the recording's own
custom_data is kept beside this module's keys. The result is read back, and
it is used only when its first message is the start frame.

NO MESSAGE OF THE EPISODE IS LEFT OUT (review of 346713b, 2026-11-28).
What is left out must be the recording's from before the episode: MAVROS's
telemetry, the raw link, the nodes' bring-up. A start frame published late
(its thread waited for git) would lie after messages of the episode itself,
and the trim would drop them; then nothing is trimmed (the caller aliases
bag/ and records why). The episode's messages, from the recorders' topic
list (sitl/run_ros2_demo.sh, deploy/docker/entrypoint.sh):

  * EPISODE_TOPICS, published only within the episode - the Shield's
    outputs and the setpoints sent on to the autopilot, from the mission on
    (smREL072_001/027/047, wgs01_001, x11r1125_001: none before the arm,
    the first 12 s or more after it): any one before the start frame;
  * FROM_NODE_START_TOPICS, published from their node's own start, before
    the arm as well - the pilot's actions, the adapter's record of what it
    sent, the raw MAVLink sent to the autopilot (the same five recordings:
    27 to 58 actions and 4 to 8 adapter records before the arm): one that
    lies at or after the arm (below); where the recording shows no arm it
    can read, any one before the start frame.

THE ARM (review of 4edd3f7, 2026-11-28) is the autopilot's accepted
COMMAND_ACK for the arm command (ACK_TOPIC: MAVLink COMMAND_ACK, command
MAV_CMD_COMPONENT_ARM_DISARM, result ACCEPTED), the moment the Shield node
learns the arm was accepted and publishes the start frame: the last such
ack at or before both the first /mavros/state saying armed and the start
frame (the same command answers a disarm), else that armed state; with no
armed state, the first such ack at or before the start frame. MAVROS's
armed state comes with the autopilot's next heartbeat,
after the ack on EVERY flight, not only before a late start frame: 1.6 to
392 ms on nine flown hil bags (smREL072_001/003/010/020/027/047, wgs01_001,
x11r1125_001, pdyn1126_001), where the take-off command went out on
/uas1/mavlink_sink 7.5 to 15 ms after the ack. Read from the armed state
alone, a message in that lag before the start frame was taken for bring-up
and dropped without notice; read from the ack, it refuses the trim. The
node publishes the start frame on the thread that then asks for the
take-off (sitl/ros2_shield_node.py start_frame_at_arm, git long finished),
so the take-off command lies after it.

WHAT DIFFERS FROM bag/. A message's count in episode.bag2/ is not its count
in the recording, by the messages left out (the node's `action_seq` counts
the actions from its own start, as bag/ does; tools/replay_from_bag.py reads
bag/ first wherever a folder has it, and its log_start correction holds only
where the two counts agree). A carried latched value is stamped after the
start frame; its receive time is in custom_data, not its timestamp.

Nothing is trimmed - the caller aliases bag/ as before and says why, from
the returned record - without `rosbags` (pure Python, optional), with a
recording it cannot read, with no start frame on /guardrail/manifest (every
bag before 2026-11-21), with a message of the episode before the start
frame (above), or when the destination already exists.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import struct
import tempfile
from pathlib import Path
from typing import Any

from guardrail.manifest import FRAME_START, MANIFEST_TOPIC, parse_manifest_frame

# This module's keys in the trimmed bag's metadata.yaml custom_data.
SOURCE_KEY = "episode_bag.source"
START_NS_KEY = "episode_bag.start_frame_ns"
DROPPED_KEY = "episode_bag.dropped_before_start"
CARRIED_KEY = "episode_bag.latched_carried"          # "topic@receive_ns,..."
HOW = "trimmed to begin at the manifest start frame (guardrail/episode_bag.py)"

# The episode's messages (module docstring, NO MESSAGE OF THE EPISODE IS
# LEFT OUT), non-latched topics of the recorders' list. /shield/mission is
# latched (carried); MAVROS's telemetry and the raw link from the autopilot
# are not the episode's.
EPISODE_TOPICS = ("/shield/setpoint", "/shield/intercept", "/shield/mode_request",
                  "/mavros/setpoint_raw/local", "/shield/mission_update")
FROM_NODE_START_TOPICS = ("/vla/action_4d", "/mavlink_adapter/sent", "/uas1/mavlink_sink")
STATE_TOPIC = "/mavros/state"
# The autopilot's answer to the arm (module docstring, THE ARM): the raw
# link from the autopilot, a COMMAND_ACK for MAV_CMD_COMPONENT_ARM_DISARM
# with MAV_RESULT_ACCEPTED.
ACK_TOPIC = "/uas1/mavlink_source"
MAVLINK_TYPE = "mavros_msgs/msg/Mavlink"
COMMAND_ACK, ARM_DISARM, ACCEPTED = 77, 400, 0


def _latched(conn) -> bool:
    """Does the publisher of `conn` offer transient_local durability (a late
    subscriber is handed its last sample)?"""
    q = getattr(conn.ext, "offered_qos_profiles", None) or []
    if isinstance(q, str):
        return "transient_local" in q.lower()
    return any(getattr(getattr(p, "durability", None), "name", "") == "TRANSIENT_LOCAL"
               for p in q)


def _frame_phase(store, raw: bytes, msgtype: str) -> str | None:
    """The phase of a /guardrail/manifest message, or None when it is not a
    manifest frame."""
    try:
        text = str(store.deserialize_cdr(raw, msgtype).data)
        return parse_manifest_frame(text).get("phase")
    except Exception:                                            # noqa: BLE001
        return None


def _register(store, conns) -> None:
    """Register the message types of `conns` from the recording's own
    definitions."""
    from rosbags.typesys import get_types_from_msg
    for c in conns:
        if c.msgtype not in store.fielddefs:
            typs = get_types_from_msg(c.msgdef.data, c.msgtype)
            store.register({k: v for k, v in typs.items() if k not in store.fielddefs})


def _state_armed_at(reader, store) -> int | None:
    """The receive time of the first STATE_TOPIC message saying armed, or
    None where the recording shows none it can read."""
    conns = [c for c in reader.connections if c.topic == STATE_TOPIC]
    if not conns:
        return None
    try:
        _register(store, conns)
        for c, ts, raw in reader.messages(connections=conns):
            if getattr(store.deserialize_cdr(raw, c.msgtype), "armed", False) is True:
                return int(ts)
    except Exception:                                            # noqa: BLE001
        return None
    return None


def _arm_acks(reader, store, stop: int) -> list[int]:
    """The receive times, before `stop`, of the autopilot's accepted
    COMMAND_ACKs for MAV_CMD_COMPONENT_ARM_DISARM on ACK_TOPIC (an arm or a
    disarm: the caller picks), those read before a failure where one
    occurs. The payload is MAVLink's wire order (command uint16, result
    uint8), which MAVLink 2 trims of trailing zeros."""
    conns = [c for c in reader.connections
             if c.topic == ACK_TOPIC and c.msgtype == MAVLINK_TYPE]
    acks: list[int] = []
    if not conns:
        return acks
    try:
        _register(store, conns)
        for c, ts, raw in reader.messages(connections=conns, stop=stop):
            m = store.deserialize_cdr(raw, c.msgtype)
            if int(m.msgid) != COMMAND_ACK:
                continue
            p = b"".join(struct.pack("<Q", int(w) & 0xFFFFFFFFFFFFFFFF)
                         for w in m.payload64)[: int(m.len)].ljust(3, b"\0")
            if int.from_bytes(p[0:2], "little") == ARM_DISARM and p[2] == ACCEPTED:
                acks.append(int(ts))
    except Exception:                                            # noqa: BLE001
        pass
    return acks


def _armed_at(reader, store, t_start: int) -> int | None:
    """The arm (module docstring, THE ARM): the last accepted arm ack at or
    before both the first armed state and the start frame, else that state;
    with no armed state the first accepted arm ack at or before the start
    frame; None where the recording shows neither."""
    state = _state_armed_at(reader, store)
    acks = _arm_acks(reader, store, (t_start if state is None else min(state, t_start)) + 1)
    if state is not None:
        return max(acks, default=state)
    return min(acks, default=None)


def _episode_before(reader, store, t_start: int) -> tuple[dict[str, int], int | None]:
    """({topic: count} of the episode's messages before the start frame,
    the arm the recording shows or None) - module docstring, NO MESSAGE OF
    THE EPISODE IS LEFT OUT."""
    armed = _armed_at(reader, store, t_start)
    conns = [c for c in reader.connections if not _latched(c)
             and c.topic in EPISODE_TOPICS + FROM_NODE_START_TOPICS]
    before: dict[str, int] = {}
    if conns:
        for c, ts, _raw in reader.messages(connections=conns, stop=t_start):
            ts = int(ts)
            if ts >= t_start or (c.topic in FROM_NODE_START_TOPICS
                                 and armed is not None and ts < armed):
                continue                       # a node's bring-up, before the arm
            before[c.topic] = before.get(c.topic, 0) + 1
    return before, armed


def _digests(root: Path) -> list[dict]:
    files = []
    for f in sorted(p for p in root.rglob("*") if p.is_file()):
        h = hashlib.sha256()
        with open(f, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
        files.append({"path": f.relative_to(root).as_posix(), "bytes": f.stat().st_size,
                      "sha256": "sha256:" + h.hexdigest()})
    return files


def trim_episode_bag(src: str | Path, dst: str | Path) -> dict[str, Any]:
    """Write `dst` (a new rosbag2 directory) from the recording `src`,
    trimmed to begin at its first manifest start frame (module docstring).
    `src` is only read. Returns {"trimmed": True, "start_frame_ns",
    "episode_id", "dropped_before_start", "carried" [{"topic",
    "receive_ns"}], "messages", "how", "files" [{"path", "bytes",
    "sha256"}]} or {"trimmed": False, "why"} with nothing left at `dst`,
    among them a recording where a message of the episode lies before the
    start frame. Never raises."""
    src, dst = Path(src), Path(dst)
    try:
        from rosbags.rosbag2 import Reader, StoragePlugin, Writer
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        return {"trimmed": False, "why": "`rosbags` is not installed (pip install "
                                         "\"rosbags>=0.10\"; pure Python, no ROS)"}
    if dst.exists():
        return {"trimmed": False, "why": f"{dst.name} exists already; left as it is"}
    tmp_parent = None
    try:
        store = get_typestore(getattr(Stores, "ROS2_JAZZY", None) or Stores.LATEST)
        with Reader(src) as reader:
            meta = reader.metadata or {}
            start = None
            frames = [c for c in reader.connections if c.topic == MANIFEST_TOPIC]
            for conn, ts, raw in reader.messages(connections=frames):
                if _frame_phase(store, raw, conn.msgtype) == FRAME_START:
                    start = (conn.id, int(ts), bytes(raw))
                    break
            if start is None:
                return {"trimmed": False, "why": (
                    f"no manifest start frame on {MANIFEST_TOPIC} (every bag recorded "
                    f"before 2026-11-21, or an episode that never armed)")}
            start_id, t_start, start_raw = start
            episode_id = json.loads(str(store.deserialize_cdr(
                start_raw, next(c for c in frames if c.id == start_id).msgtype).data)
            ).get("episode_id")
            before, armed = _episode_before(reader, store, t_start)
            if before:
                return {"trimmed": False, "why": (
                    f"the start frame lies after {sum(before.values())} message(s) of the "
                    f"episode (" + ", ".join(f"{n} on {t}" for t, n in before.items())
                    + ("" if armed is not None else
                       f"; no arm could be read (an accepted arm COMMAND_ACK on "
                       f"{ACK_TOPIC} or {STATE_TOPIC} saying armed), so every one "
                       f"before the start frame counts") + "), which trimming would leave out")}
            latched = {c.id for c in reader.connections
                       if c.topic != MANIFEST_TOPIC and _latched(c)}
            version = 9 if int(meta.get("version") or 9) >= 9 else 8
            storage = (StoragePlugin.MCAP if meta.get("storage_identifier") == "mcap"
                       else StoragePlugin.SQLITE3)
            dst.parent.mkdir(parents=True, exist_ok=True)
            tmp_parent = Path(tempfile.mkdtemp(prefix=".episode_bag_", dir=dst.parent))
            out = tmp_parent / dst.name
            carried: list[dict] = []
            dropped = written = 0
            writer = Writer(out, version=version, storage_plugin=storage)
            with writer as w:
                conns = {}
                for c in reader.connections:
                    conns[c.id] = w.add_connection(
                        c.topic, c.msgtype, msgdef=c.msgdef.data, rihs01=c.digest,
                        serialization_format=c.ext.serialization_format,
                        offered_qos_profiles=c.ext.offered_qos_profiles)
                last: dict[int, tuple[int, bytes]] = {}
                flushed = False
                for conn, ts, raw in reader.messages():
                    ts = int(ts)
                    if not flushed and ts < t_start:
                        if conn.id in latched:
                            if last.pop(conn.id, None) is not None:
                                dropped += 1                 # an older sample
                            last[conn.id] = (ts, bytes(raw))  # in receive order
                        else:
                            dropped += 1
                        continue
                    if not flushed:
                        # The start frame's own receive time is reached (it is
                        # a message of the recording): it goes first, then the
                        # latched values a recorder joining now is handed.
                        w.write(conns[start_id], t_start, start_raw)
                        written += 1
                        for i, (cid, (t0, data)) in enumerate(last.items(), start=1):
                            w.write(conns[cid], t_start + i, data)
                            carried.append({"topic": conns[cid].topic, "receive_ns": t0})
                            written += 1
                        flushed = True
                    if conn.id == start_id and ts == t_start and bytes(raw) == start_raw:
                        continue                              # written first, above
                    w.write(conns[conn.id], ts, raw)
                    written += 1
                for k, v in (meta.get("custom_data") or {}).items():
                    w.set_custom_data(str(k), str(v))
                w.set_custom_data(SOURCE_KEY, src.name)
                w.set_custom_data(START_NS_KEY, str(t_start))
                w.set_custom_data(DROPPED_KEY, str(dropped))
                w.set_custom_data(CARRIED_KEY, ",".join(
                    f"{c['topic']}@{c['receive_ns']}" for c in carried))
        # Read back: the start frame must be the first message.
        with Reader(out) as back:
            first = next(iter(back.messages()), None)
            n_back = sum(int(c.msgcount) for c in back.connections)
        if first is None or first[0].topic != MANIFEST_TOPIC or int(first[1]) != t_start \
                or bytes(first[2]) != start_raw:
            raise RuntimeError("read back, its first message is not the start frame")
        if n_back != written:
            raise RuntimeError(f"read back {n_back} message(s), {written} written")
        os.replace(out, dst)
        return {"trimmed": True, "how": HOW, "start_frame_ns": t_start,
                "episode_id": episode_id, "dropped_before_start": dropped,
                "carried": carried, "messages": written, "files": _digests(dst)}
    except Exception as e:                                       # noqa: BLE001
        return {"trimmed": False,
                "why": f"the recording could not be trimmed: {type(e).__name__}: {e}"}
    finally:
        if tmp_parent is not None:
            shutil.rmtree(tmp_parent, ignore_errors=True)
