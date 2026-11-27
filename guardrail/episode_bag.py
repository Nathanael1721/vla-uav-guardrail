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

WHAT DIFFERS FROM bag/. A message's count in episode.bag2/ is not its count
in the recording, by the messages left out (the node's `action_seq` counts
the actions from its own start, as bag/ does; tools/replay_from_bag.py reads
bag/ first wherever a folder has it, and its log_start correction holds only
where the two counts agree). A carried latched value is stamped after the
start frame; its receive time is in custom_data, not its timestamp.

Nothing is trimmed - the caller aliases bag/ as before and says why, from
the returned record - without `rosbags` (pure Python, optional), with a
recording it cannot read, with no start frame on /guardrail/manifest (every
bag before 2026-11-21), or when the destination already exists.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
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
    "sha256"}]} or {"trimmed": False, "why"} with nothing left at `dst`.
    Never raises."""
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
