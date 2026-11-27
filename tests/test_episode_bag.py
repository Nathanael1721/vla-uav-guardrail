"""The episode bundle's episode.bag2/ begins with the manifest start frame
(ARCH-22, 2026-11-28; guardrail/episode_bag.py).

    python tests/test_episode_bag.py      (no pytest needed)

The grant's data-flow page: the six manifest fields are "stored as the first
frame of every rosbag". The rails' recorder starts before the Guardrail
nodes, so in a run's bag/ MAVROS messages precede the start frame
(smREL072_001: 24 messages over 4.0 s). These tests write small rosbag2
recordings with `rosbags` and check that:

  * the trimmed bag's first message is the start frame, at its own receive
    time; the latched values a recorder joining then would be handed (the
    last message of each transient_local topic before it) follow 1 ns apart
    in their receive order; every later message is the recording's, time
    and bytes; the rest before it is left out and counted; the connections
    and the recording's custom_data are kept; the recording is not touched;
  * a recording with no start frame, one rosbags cannot read and an
    existing destination are not trimmed, and nothing is left behind;
  * rail_bag_and_audit (the Harness's rail session and the sweep's rail
    campaigns) writes the trimmed bag as episode.bag2/, falls back to the
    alias of bag/ as before when it cannot trim, and says which;
  * with EPISODE_BAG_REAL=<a run's bag/ directory> (read only: copied to a
    temporary folder first), the same on a flown recording.

Each fails on 69e9545: there is no guardrail/episode_bag.py, and
episode.bag2/ was bag/'s files aliased, MAVROS first. SKIPPED where
`rosbags` is not installed (the lab's vla-real environment has 0.10.11).
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import struct
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.manifest import manifest_frame                        # noqa: E402

SKIP = "SKIP"
_TMP: list[Path] = []
T0 = 1_795_769_648_000_000_000
MS = 1_000_000


def _tmp(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


def _have_rosbags() -> bool:
    try:
        import rosbags.rosbag2                                       # noqa: F401
        return True
    except ImportError:
        return False


def _qos(latched: bool):
    from rosbags.interfaces import (Qos, QosDurability, QosHistory, QosLiveliness,
                                    QosReliability, QosTime)
    forever = QosTime(sec=9223372036, nsec=854775807)
    return [Qos(history=QosHistory.KEEP_LAST, depth=10 if latched else 100,
                reliability=QosReliability.RELIABLE,
                durability=(QosDurability.TRANSIENT_LOCAL if latched
                            else QosDurability.VOLATILE),
                deadline=forever, lifespan=forever, liveliness=QosLiveliness.AUTOMATIC,
                liveliness_lease_duration=forever, avoid_ros_namespace_conventions=False)]


# topic -> latched (the publisher offers transient_local)
TOPICS = {"/mavros/state": True, "/uas1/mavlink_source": False, "/shield/fences": True,
          "/vla/identity": True, "/vla/action_4d": False, "/guardrail/manifest": True,
          "/shield/setpoint": False}
# /mavros/state is mavros_msgs/msg/State (the trim reads its `armed`); the
# test's text is its `mode`, and "armed" in it (not "disarmed") arms it.
STATE_TYPE = "mavros_msgs/msg/State"
STATE_MSG = ("std_msgs/Header header\nbool connected\nbool armed\nbool guided\n"
             "bool manual_input\nstring mode\nuint8 system_status\n")
# The raw MAVLink link (the fields only, which is all the CDR layout
# depends on; tools/replay_from_bag.py FALLBACK_MSGS), for the tests that
# record it as mavros_msgs/msg/Mavlink rather than as text.
MAVLINK_TYPE = "mavros_msgs/msg/Mavlink"
MAVLINK_MSG = ("std_msgs/Header header\nuint8 framing_status\nuint8 magic\nuint8 len\n"
               "uint8 incompat_flags\nuint8 compat_flags\nuint8 seq\nuint8 sysid\n"
               "uint8 compid\nuint32 msgid\nuint16 checksum\nuint64[] payload64\n"
               "uint8[] signature\n")
START, END = T0 + 400 * MS, T0 + 2000 * MS
MAN = {"code_revision": "418b1517da9f", "vla_model_hash": "m@src:" + "0" * 64,
       "policy_hash": "sha256:" + "a" * 64, "random_seed": 1001, "sim_speedup": 1.0,
       "topology": "hil"}


def _recording() -> list[tuple[str, int, str]]:
    """(topic, receive time, text) in receive order: MAVROS and the pilot
    before the start frame (two samples of the state, one of the fences and
    of the identity), the start frame, the episode, the end frame."""
    msgs = [("/mavros/state", T0, "state disarmed 1"),
            ("/uas1/mavlink_source", T0 + 50 * MS, "heartbeat 1"),
            ("/vla/identity", T0 + 60 * MS, '{"model_id": "m"}'),
            ("/mavros/state", T0 + 100 * MS, "state disarmed 2"),
            ("/uas1/mavlink_source", T0 + 150 * MS, "heartbeat 2"),
            ("/shield/fences", T0 + 200 * MS, '{"generation": 0}'),
            ("/vla/action_4d", T0 + 300 * MS, "action 0"),
            ("/guardrail/manifest", START, json.dumps(manifest_frame("start", MAN, 7)))]
    for i in range(1, 6):
        msgs += [("/vla/action_4d", START + i * 100 * MS, f"action {i}"),
                 ("/uas1/mavlink_source", START + i * 100 * MS + 5, f"heartbeat {i + 2}")]
    msgs += [("/mavros/state", START + 300 * MS + 1, "state armed"),
             ("/guardrail/manifest", END, json.dumps(manifest_frame("end", MAN, 7)))]
    return msgs


def _store():
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    store = get_typestore(Stores.ROS2_JAZZY)
    store.register(get_types_from_msg(STATE_MSG, STATE_TYPE))
    store.register(get_types_from_msg(MAVLINK_MSG, MAVLINK_TYPE))
    return store


def _msgtype(topic: str, mavlink=()) -> str:
    if topic == "/mavros/state":
        return STATE_TYPE
    return MAVLINK_TYPE if topic in mavlink else "std_msgs/msg/String"


def _mavlink(store, header, msgid: int, payload: bytes):
    """A mavros_msgs/msg/Mavlink of `payload`, trimmed of its trailing zeros
    as MAVLink 2 sends it, in little-endian 64-bit words."""
    import numpy as np
    payload = payload.rstrip(b"\0") or b"\0"
    padded = payload + b"\0" * (-len(payload) % 8)
    return store.types[MAVLINK_TYPE](
        header=header, framing_status=1, magic=253, len=len(payload), incompat_flags=0,
        compat_flags=0, seq=0, sysid=1, compid=1, msgid=msgid, checksum=0,
        payload64=np.frombuffer(padded, dtype="<u8").copy(),
        signature=np.zeros(0, dtype=np.uint8))


def _write(bag: Path, msgs, *, custom: dict | None = None, mavlink=()) -> None:
    """Write `msgs` (topic, receive time, text); on a topic in `mavlink`
    the "text" is (msgid, payload), written as mavros_msgs/msg/Mavlink."""
    from rosbags.rosbag2 import StoragePlugin, Writer
    store = _store()
    String, State = store.types["std_msgs/msg/String"], store.types[STATE_TYPE]
    Header, Time = store.types["std_msgs/msg/Header"], store.types["builtin_interfaces/msg/Time"]
    topics = {**TOPICS, **{t: False for t in mavlink if t not in TOPICS}}
    with Writer(bag, version=9, storage_plugin=StoragePlugin.MCAP) as w:
        conns = {t: w.add_connection(t, _msgtype(t, mavlink), typestore=store,
                                     offered_qos_profiles=_qos(latched))
                 for t, latched in topics.items()}
        for k, v in (custom or {}).items():
            w.set_custom_data(k, v)
        for topic, ts, text in msgs:
            header = Header(stamp=Time(sec=ts // 10**9, nanosec=ts % 10**9), frame_id="")
            if topic == "/mavros/state":
                msg = State(header=header,
                            connected=True, armed="armed" in text.replace("disarmed", ""),
                            guided=True, manual_input=False, mode=text, system_status=3)
            elif topic in mavlink:
                msg = _mavlink(store, header, *text)
            else:
                msg = String(data=text)
            w.write(conns[topic], ts, store.serialize_cdr(msg, _msgtype(topic, mavlink)))


def _read(bag: Path) -> list[tuple[str, int, bytes]]:
    from rosbags.rosbag2 import Reader
    with Reader(bag) as r:
        return [(c.topic, int(ts), bytes(raw)) for c, ts, raw in r.messages()]


def _texts(bag: Path) -> list[tuple[str, int, str]]:
    store = _store()
    out = []
    for t, ts, raw in _read(bag):
        m = store.deserialize_cdr(raw, _msgtype(t))
        out.append((t, ts, str(m.mode if t == "/mavros/state" else m.data)))
    return out


def _digest(d: Path) -> dict:
    return {p.relative_to(d).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(d.rglob("*")) if p.is_file()}


def test_the_episode_bag_begins_with_the_start_frame():
    if not _have_rosbags():
        print("      SKIP: rosbags is not installed")
        return SKIP
    from guardrail.episode_bag import CARRIED_KEY, DROPPED_KEY, START_NS_KEY, trim_episode_bag
    from rosbags.rosbag2 import Reader
    root = _tmp("epbag_")
    src = root / "run" / "bag"
    src.parent.mkdir()
    msgs = _recording()
    _write(src, msgs, custom={"manifest.random_seed": "1001"})
    before = _digest(src)
    assert _texts(src)[0][0] == "/mavros/state"           # MAVROS first, as recorded
    (root / "ep").mkdir()
    dst = root / "ep" / "episode.bag2"
    res = trim_episode_bag(src, dst)
    assert res["trimmed"] is True, res
    assert _digest(src) == before, "the recording was changed"
    got = _texts(dst)
    # 1. the start frame first, at its own receive time
    assert got[0][0] == "/guardrail/manifest" and got[0][1] == START, got[:3]
    assert json.loads(got[0][2])["phase"] == "start"
    # 2. the latched values, the last of each before it, in receive order
    assert got[1:4] == [("/vla/identity", START + 1, '{"model_id": "m"}'),
                        ("/mavros/state", START + 2, "state disarmed 2"),
                        ("/shield/fences", START + 3, '{"generation": 0}')], got[1:4]
    assert [c["topic"] for c in res["carried"]] == ["/vla/identity", "/mavros/state",
                                                     "/shield/fences"]
    assert res["carried"][1]["receive_ns"] == T0 + 100 * MS
    # 3. every later message as recorded, times and bytes
    after = [m for m in _read(src) if m[1] > START]
    assert _read(dst)[4:] == after
    assert got[-1][0] == "/guardrail/manifest" and json.loads(got[-1][2])["phase"] == "end"
    # the rest before it left out and counted: 2 heartbeats, an action, the older state
    assert res["dropped_before_start"] == 4 and res["messages"] == len(got) == 4 + len(after)
    assert res["start_frame_ns"] == START and res["episode_id"] == 7
    with Reader(dst) as r, Reader(src) as s:
        assert [(c.topic, c.msgtype, c.digest) for c in r.connections] == \
            [(c.topic, c.msgtype, c.digest) for c in s.connections]
        assert [[q.durability for q in c.ext.offered_qos_profiles] for c in r.connections] \
            == [[q.durability for q in c.ext.offered_qos_profiles] for c in s.connections]
        assert r.metadata["storage_identifier"] == "mcap"
        custom = r.metadata.get("custom_data") or {}
    assert custom["manifest.random_seed"] == "1001", custom       # the recording's own
    assert custom[START_NS_KEY] == str(START) and custom[DROPPED_KEY] == "4", custom
    assert custom[CARRIED_KEY].split(",")[0] == f"/vla/identity@{T0 + 60 * MS}", custom
    assert {f["path"] for f in res["files"]} == {p.name for p in dst.iterdir()}
    assert not [p for p in (root / "ep").iterdir() if p.name != "episode.bag2"], \
        "a temporary folder was left behind"


def test_what_cannot_be_trimmed_is_said_and_nothing_is_left_behind():
    if not _have_rosbags():
        print("      SKIP: rosbags is not installed")
        return SKIP
    from guardrail.episode_bag import trim_episode_bag
    root = _tmp("epbag_no_")
    # no start frame: every bag before 2026-11-21
    old = root / "old"
    _write(old, [m for m in _recording() if m[0] != "/guardrail/manifest"])
    (root / "a").mkdir()
    res = trim_episode_bag(old, root / "a" / "episode.bag2")
    assert set(res) == {"trimmed", "why"} and res["trimmed"] is False, res
    assert "no manifest start frame" in res["why"], res
    assert list((root / "a").iterdir()) == []
    # an end frame alone (an episode that never armed) is no start frame either
    ends = root / "ends"
    _write(ends, [m for m in _recording() if not (m[0] == "/guardrail/manifest"
                                                  and m[1] == START)])
    (root / "b").mkdir()
    assert "no manifest start frame" in trim_episode_bag(ends, root / "b" / "x")["why"]
    # not a rosbag2
    junk = root / "junk"
    junk.mkdir()
    (junk / "bag_0.mcap").write_bytes(b"not a bag")
    (junk / "metadata.yaml").write_bytes(b"rosbag2_bagfile_information:\n  version: 9\n")
    (root / "c").mkdir()
    res = trim_episode_bag(junk, root / "c" / "episode.bag2")
    assert res["trimmed"] is False and "could not be trimmed" in res["why"], res
    assert list((root / "c").iterdir()) == []
    # an existing destination is left as it is
    good = root / "good"
    _write(good, _recording())
    (root / "d" / "episode.bag2").mkdir(parents=True)
    (root / "d" / "episode.bag2" / "keep.txt").write_text("x", encoding="utf-8")
    res = trim_episode_bag(good, root / "d" / "episode.bag2")
    assert res["trimmed"] is False and "exists already" in res["why"], res
    assert [p.name for p in (root / "d" / "episode.bag2").iterdir()] == ["keep.txt"]


def test_the_rail_bundle_writes_the_trimmed_bag_or_the_alias_and_says_which():
    if not _have_rosbags():
        print("      SKIP: rosbags is not installed")
        return SKIP
    from guardrail.harness_service import EPISODE_BAG_DIR, rail_bag_and_audit
    root = _tmp("epbag_rail_")
    run = root / "store" / "smX_001"
    run.mkdir(parents=True)
    _write(run / "bag", _recording())
    (run / "shield_audit.jsonl").write_text("{}\n", encoding="utf-8")
    before = _digest(run)
    ep = root / "ep"
    ep.mkdir()
    files, info = rail_bag_and_audit(run, ep)
    text = files[EPISODE_BAG_DIR + "/"]
    assert text.startswith("written") and "trimmed to begin at its manifest start frame" \
        in text and "4 earlier message(s) left out, 3 latched value(s) carried" in text, text
    assert info["trim"]["trimmed"] is True and info["trim"]["start_frame_ns"] == START
    assert info["source"] == (run / "bag").as_posix() and "guardrail/episode_bag.py" in \
        info["how"]
    assert {f["path"]: f["sha256"] for f in info["files"]} == {
        k: "sha256:" + v for k, v in _digest(ep / "episode.bag2").items()}
    assert _texts(ep / "episode.bag2")[0][0] == "/guardrail/manifest"
    assert _digest(run) == before, "the run folder was changed"
    # a bag with no start frame: bag/ aliased as before, and why it was not trimmed
    run2 = root / "store" / "old_001"
    run2.mkdir()
    _write(run2 / "bag", [m for m in _recording() if m[0] != "/guardrail/manifest"])
    ep2 = root / "ep2"
    ep2.mkdir()
    files, info = rail_bag_and_audit(run2, ep2)
    text = files[EPISODE_BAG_DIR + "/"]
    assert "not trimmed to its manifest start frame: no manifest start frame" in text, text
    assert info["how"] in ("hard links", "copies", "hard links and copies"), info
    assert info["trim"]["trimmed"] is False
    assert _digest(ep2 / "episode.bag2") == _digest(run2 / "bag")


def test_a_start_frame_after_messages_of_the_episode_is_not_trimmed_over_them():
    """Review of 346713b: a start frame published late (its thread waited
    for git) lies after messages of the episode, and the trim dropped them.
    Not trimmed then, nothing left behind, and the bundle aliases bag/ and
    says why: a Shield output before it (published only within the
    episode); a pilot action at or after the arm /mavros/state shows (the
    actions before the arm are the VLA node's bring-up, and are trimmed as
    in the main test); any action before it where the recording shows no
    arm. Fails on 346713b: each was trimmed, the messages left out."""
    if not _have_rosbags():
        print("      SKIP: rosbags is not installed")
        return SKIP
    from guardrail.episode_bag import trim_episode_bag
    from guardrail.harness_service import EPISODE_BAG_DIR, rail_bag_and_audit
    root = _tmp("epbag_late_")
    armed = ("/mavros/state", T0 + 250 * MS, "state armed")
    cases = {
        # the arm at +250 ms, the start frame at +400 ms: action 0 (+300 ms)
        # is the episode's
        "action_after_arm": (sorted(_recording() + [armed], key=lambda m: m[1]),
                             "1 on /vla/action_4d"),
        # a Shield output before the start frame, whenever the arm
        "shield_output": (sorted(_recording() + [("/shield/setpoint", T0 + 10 * MS, "[0, 0]")],
                                 key=lambda m: m[1]), "1 on /shield/setpoint"),
        # no /mavros/state: the arm cannot be read, so action 0 counts
        "no_arm": ([m for m in _recording() if m[0] != "/mavros/state"],
                   "no arm could be read"),
    }
    for name, (msgs, said) in cases.items():
        src = root / name / "bag"
        src.parent.mkdir()
        _write(src, msgs)
        before = _digest(src)
        (root / name / "ep").mkdir()
        res = trim_episode_bag(src, root / name / "ep" / "episode.bag2")
        assert res["trimmed"] is False and set(res) == {"trimmed", "why"}, (name, res)
        assert "the start frame lies after 1 message(s) of the episode" in res["why"] \
            and said in res["why"], (name, res)
        assert list((root / name / "ep").iterdir()) == [], name
        assert _digest(src) == before, name
    # the arm read before the start frame, every action before it before the
    # arm: trimmed (the main test's arm lies after the start frame, as
    # MAVROS's armed state comes with the next heartbeat)
    ok = root / "ok" / "bag"
    ok.parent.mkdir()
    _write(ok, sorted(_recording() + [("/mavros/state", T0 + 350 * MS, "state armed")],
                      key=lambda m: m[1]))
    res = trim_episode_bag(ok, root / "ok" / "x")
    assert res["trimmed"] is True and res["dropped_before_start"] == 5, res
    # the bundle: bag/ aliased, and why
    run = root / "store" / "smX_002"
    run.mkdir(parents=True)
    _write(run / "bag", cases["action_after_arm"][0])
    (run / "shield_audit.jsonl").write_text("{}\n", encoding="utf-8")
    ep = root / "bundle"
    ep.mkdir()
    files, info = rail_bag_and_audit(run, ep)
    text = files[EPISODE_BAG_DIR + "/"]
    assert "not trimmed to its manifest start frame: the start frame lies after 1 " \
        "message(s) of the episode (1 on /vla/action_4d)" in text, text
    assert info["trim"]["trimmed"] is False and "/vla/action_4d" in info["trim"]["why"]
    assert _digest(ep / "episode.bag2") == _digest(run / "bag")


SOURCE, SINK = "/uas1/mavlink_source", "/uas1/mavlink_sink"
HEARTBEAT = (0, struct.pack("<IBBBBB", 0, 2, 3, 0, 0, 4))
TAKEOFF = (76, struct.pack("<7fHBBB", 0, 0, 0, 0, 0, 0, 15.0, 22, 1, 1, 0))   # COMMAND_LONG


def _ack(result: int) -> tuple[int, bytes]:
    """COMMAND_ACK for MAV_CMD_COMPONENT_ARM_DISARM (400) with `result`."""
    return 77, struct.pack("<HB", 400, result)


def _with_link(msgs, extra):
    """`msgs` with the link's heartbeats as MAVLink HEARTBEATs, and `extra`
    added, in receive order."""
    return sorted([(t, ts, HEARTBEAT if t == SOURCE else x) for t, ts, x in msgs] + extra,
                  key=lambda m: m[1])


def test_the_arm_is_the_autopilots_accepted_ack_not_the_later_armed_state():
    """Review of 4edd3f7: MAVROS's armed state comes 1.6 to 392 ms after the
    autopilot's accepted COMMAND_ACK for the arm (nine flown hil bags), and
    the take-off command reached /uas1/mavlink_sink 7.5 to 15 ms after that
    ack. With the armed state as the arm, a take-off command between the ack
    and a start frame published later lay "before the arm", was taken for
    bring-up and dropped without notice. The arm is now the last accepted
    arm ack at or before the armed state and the start frame: that command
    refuses the trim. A
    start frame before the take-off command (the node publishes it first)
    is trimmed with the command kept; an earlier accepted ack of the same
    command (a disarm) and a refused one are not the arm; with no armed
    state an accepted ack alone is, and a refused ack alone is no arm.
    Fails on 4edd3f7: the take-off command was left out and the trim made."""
    if not _have_rosbags():
        print("      SKIP: rosbags is not installed")
        return SKIP
    from guardrail.episode_bag import trim_episode_bag
    root = _tmp("epbag_ack_")

    def trim(name, msgs):
        src = root / name / "bag"
        src.parent.mkdir()
        _write(src, msgs, mavlink=(SOURCE, SINK))
        before = _digest(src)
        (root / name / "ep").mkdir()
        dst = root / name / "ep" / "episode.bag2"
        res = trim_episode_bag(src, dst)
        assert _digest(src) == before, name
        if not res["trimmed"]:
            assert list(dst.parent.iterdir()) == [], name
        return res, dst

    # the arm accepted at +360 ms, the take-off command at +370 ms, the start
    # frame at +400 ms, MAVROS's armed state only after it (+701 ms)
    res, _ = trim("takeoff_before_frame", _with_link(_recording(), [
        (SOURCE, T0 + 360 * MS, _ack(0)), (SINK, T0 + 370 * MS, TAKEOFF)]))
    assert res["trimmed"] is False and set(res) == {"trimmed", "why"}, res
    assert "the start frame lies after 1 message(s) of the episode (1 on " \
        "/uas1/mavlink_sink)" in res["why"], res
    # the same, with a second accepted ack after the start frame (an arm
    # repeated): the frame went out at an arm before it, so it is not the arm
    res, _ = trim("repeated_ack", _with_link(_recording(), [
        (SOURCE, T0 + 360 * MS, _ack(0)), (SINK, T0 + 370 * MS, TAKEOFF),
        (SOURCE, START + 50 * MS, _ack(0))]))
    assert res["trimmed"] is False and "1 on /uas1/mavlink_sink" in res["why"], res
    # the start frame before the take-off command: trimmed, the command kept
    res, dst = trim("frame_before_takeoff", _with_link(_recording(), [
        (SOURCE, T0 + 360 * MS, _ack(0)), (SINK, START + 10 * MS, TAKEOFF)]))
    assert res["trimmed"] is True, res
    got = _read(dst)
    assert got[0][0] == "/guardrail/manifest" and got[0][1] == START, got[:2]
    assert (SINK, START + 10 * MS) in [(t, ts) for t, ts, _ in got], got
    # left out: two heartbeats, the ack, action 0 (+300 ms), the older state
    assert res["dropped_before_start"] == 5, res
    # an accepted ack of the same command long before (a disarm) and a refused
    # one: the arm is the accepted ack at +360 ms, action 0 stays bring-up
    res, _ = trim("earlier_disarm", _with_link(_recording(), [
        (SOURCE, T0 + 20 * MS, _ack(0)), (SOURCE, T0 + 200 * MS, _ack(4)),
        (SOURCE, T0 + 360 * MS, _ack(0)), (SINK, START + 10 * MS, TAKEOFF)]))
    assert res["trimmed"] is True, res
    # no /mavros/state: an accepted ack is the arm (action 0 before it is
    # bring-up); a refused ack is no arm, so action 0 counts
    no_state = [m for m in _recording() if m[0] != "/mavros/state"]
    res, _ = trim("ack_no_state", _with_link(no_state, [(SOURCE, T0 + 350 * MS, _ack(0))]))
    assert res["trimmed"] is True, res
    res, _ = trim("refused_no_state", _with_link(no_state, [(SOURCE, T0 + 350 * MS, _ack(4))]))
    assert res["trimmed"] is False and "1 on /vla/action_4d" in res["why"] \
        and "no arm could be read" in res["why"], res


def test_a_flown_recording_is_trimmed_to_its_start_frame():
    """EPISODE_BAG_REAL=<a run's bag/ directory>: copied to a temporary folder
    (the store is only read), trimmed, read back. SKIPPED without it."""
    real = os.environ.get("EPISODE_BAG_REAL")
    if not real or not _have_rosbags():
        print("      SKIP: set EPISODE_BAG_REAL to a flown run's bag/ (rosbags needed)")
        return SKIP
    from guardrail.episode_bag import trim_episode_bag
    from rosbags.rosbag2 import Reader
    root = _tmp("epbag_real_")
    src = root / "bag"
    shutil.copytree(real, src)
    res = trim_episode_bag(src, root / "episode.bag2")
    assert res["trimmed"] is True, res
    with Reader(src) as r:
        first_src = next(iter(r.messages()))
        n_src = sum(int(c.msgcount) for c in r.connections)
    from rosbags.typesys import Stores, get_typestore
    got = _read(root / "episode.bag2")
    assert got[0][0] == "/guardrail/manifest" and got[0][1] == res["start_frame_ns"]
    first = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(got[0][2], "std_msgs/msg/String")
    assert json.loads(first.data)["phase"] == "start"
    assert res["messages"] == len(got) == n_src - res["dropped_before_start"]
    print(f"      {real}: first message {first_src[0].topic} -> /guardrail/manifest; "
          f"{res['dropped_before_start']} left out, "
          f"{[c['topic'] for c in res['carried']]} carried, {len(got)} messages")


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    for d in _TMP:
        shutil.rmtree(d, ignore_errors=True)
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
