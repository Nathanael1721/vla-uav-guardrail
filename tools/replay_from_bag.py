"""Replay from the rosbag2 capture: what the bag alone says an episode was.

    python tools/replay_from_bag.py demo/out/hil_smoke/ntFIN2_034_corridor-along
    python tools/replay_from_bag.py --campaign demo/out/hil_ntFIN2 \
        --campaign demo/out/hil_ntFIN2s --out demo/out/replay_from_bag_ntFIN2
    python tools/replay_from_bag.py demo/out/<tag>/<tag>.replay.tar.gz

WHY THIS EXISTS (ARCH-22, 2026-11-16)

The grant's simulation layer asks for "rosbag2 recording for replay" and says
the "Determinism contract for stress-test KPI runs depends on rosbag2
capture" (03-simulation/layer-requirements.md, L3). Every run of the ROS 2
Shield rail has recorded a bag since 2026-11-02, and until this tool the
exporter (tools/export_episodes.py) was its only reader: it took the
setpoints MAVROS was sent and nothing else. `guardrail.replay.verify_replay`
re-scores an episode from flight_log.jsonl, a file the Shield node writes
about itself. Nothing checked the episode against the recording a second
process, `ros2 bag record`, made of the same topics.

This does. For one run folder (or its replay bundle) it reads the bag
directory with `rosbags` and nothing else, rebuilds each tick, re-derives
the trajectory and the part of the KPI table the topics can carry, and only
then opens the run's flight log, trajectory.json and kpi.json to compare.

WHAT IS REBUILT, AND FROM WHICH TOPIC

  the tick            one /vla/action_4d message: the rail ticks on each
                      valid action (metrics.json `tick_schedule.mode`
                      "action", every hil campaign since 2026-11-12)
  raw_body            that message's four float32
  pose                /mavros/local_position/pose, the last message logged at
                      or before the tick's action, moved into the policy's
                      frame by `home_policy_ne` of the first /shield/fences
                      message (the node's --start)
  emitted_body        the /shield/setpoint message logged in the tick's
                      window (up to the next action), or none
  executed            the /mavros/setpoint_raw/local message in the same
                      window: what the MAVLink adapter sent MAVROS
  mode                /mavros/state, the last message at or before the tick
  violations, repairs, fsm state
                      /shield/intercept, which the node publishes on a tick
                      the Shield touched with the Shield on: rule ids and
                      operator names only

WHAT THE BAG DOES NOT HOLD (each is reported per run, never filled in)

  * Which pose message a tick consumed. The node and the recorder are two
    subscribers; near a tick they can take the same two messages in a
    different order. The rebuilt pose is the recorder's order. Where the
    flight log's point is the bag's NEXT (or previous) pose message, bit for
    bit, the tick is counted as `neighbour`, not as a difference: the logged
    trajectory is still a sequence of the bag's own messages.
  * The mission's end. The first tick is the action the Shield's first
    message answers. After the last message the Shield published, a tick
    that streamed nothing and touched nothing (the autopilot flying an RTL
    with a clean request) leaves no message at all. The bag holds the
    actions and poses of those ticks, not the fact that the node ticked on
    them: their number is taken from the flight log and said so
    (`ticks_past_bag_end`, and the source of `ticks` / `autopilot_ticks`).
  * A tick the timer ran (no action arrived): no message marks it.
  * Whether a setpoint was the Shield's answer or a hold's own leg
    (guardrail/hold_flight.py streams on the same topic while the FSM's
    verdict is "none"). It is INFERRED: a setpoint that is not the action,
    with no intercept, or with an intercept whose FSM state is Loiter / RTL
    / Land after the hand-over tick, is the hold's.
  * The Shield's re-check of what it flew (`emitted_violations`), whether a
    position is legal (`unsafe`), each repair's size in metres, the FSM's
    state on a tick the Shield did not touch, the action the Shield repaired
    on a tick the FSM stopped instead, and everything metrics.json adds
    (zone dwell, the goal, the fail-safe label). So the P0 escape rate, the
    fail-safe figure, time to safe, repair success and the mission outcome
    are NOT re-derived here; the summary lists them as such with the reason.
    They need the rules (the policy generations) and the Shield, which is
    the open-loop replay of WP4-11, not this tool.

THE KPI TABLE, FIELD BY FIELD

`guardrail.kpi.compute` - the function that wrote kpi.json - runs on the
rebuilt rows. A field is then one of:

  bag                       every input is a topic of the bag;
  bag + policy_g<N>.json    also the rule priorities (a rule id is on the
                            topic, its P0 / P1 / P2 is in the policy);
  not from the bag          with the reason; never copied from kpi.json.

A field that is from the bag in general can still be out of reach on one
run (a Shield-off arm publishes no intercept; a tick the FSM stopped has no
repaired action on any topic): the run's record says which and why.

WHAT "IDENTICAL" MEANS

Every tick of the flight log has a tick in the bag with the same action
(float32, bit for bit), the same published setpoint, the same verdict of who
flew it, mode, violated rules, operators and FSM state; every logged point
is a pose message of the bag; and every KPI field the bag can carry equals
kpi.json. Otherwise the verdict is `differs` with the first differing layer,
field and tick. `not_replayable` is a run with no bag, no action or no
Shield message.

WHAT THIS IS NOT. It is not a re-flight and not the grant's bit-for-bit
replay of an episode (WP4-11): the Shield and the FSM are not run again, and
the flown episodes of ntFIN2 cannot be, because the bag does not say which
input each tick consumed. It shows that the recording and the node's own
log describe one flight.

Needs `rosbags` >= 0.10 (pure Python, no ROS). Exit code: 0 every run
identical, 1 a run differs, 2 a run could not be replayed or the arguments
are wrong, 3 `rosbags` is not installed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
import tarfile
import tempfile
import time
from bisect import bisect_right
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "tools", ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import export_episodes as X                                      # noqa: E402
from guardrail import kpi as K                                   # noqa: E402
from sitl_stressors import parse_marker                          # noqa: E402

# The topics read (sitl/run_ros2_demo.sh and deploy/docker/entrypoint.sh
# record them; tests/test_replay_from_bag.py keeps these names in both lists).
ACTION_TOPIC = "/vla/action_4d"
SETPOINT_TOPIC = "/shield/setpoint"
INTERCEPT_TOPIC = "/shield/intercept"
FENCES_TOPIC = "/shield/fences"
MODE_REQUEST_TOPIC = "/shield/mode_request"
EXECUTED_TOPIC = X.SETPOINT_TOPIC                 # /mavros/setpoint_raw/local
POSE_TOPIC = X.POSE_TOPIC                         # /mavros/local_position/pose
STATE_TOPIC = "/mavros/state"
MAVLINK_SINK_TOPIC = "/uas1/mavlink_sink"
READ_TOPICS = (ACTION_TOPIC, SETPOINT_TOPIC, INTERCEPT_TOPIC, FENCES_TOPIC,
               MODE_REQUEST_TOPIC, EXECUTED_TOPIC, POSE_TOPIC, STATE_TOPIC,
               MAVLINK_SINK_TOPIC)
BAG_DIRS = ("bag", "episode.bag2")                # the run's, the grant's name

# Message definitions for a bag that does not carry them (a bag rosbag2
# recorded on Jazzy does, and that one is used): the fields only, which is
# all the CDR layout depends on.
FALLBACK_MSGS = {
    X.POSITION_TARGET_TYPE: X.POSITION_TARGET_MSG,
    "mavros_msgs/msg/State": (
        "std_msgs/Header header\nbool connected\nbool armed\nbool guided\n"
        "bool manual_input\nstring mode\nuint8 system_status\n"),
    "mavros_msgs/msg/Mavlink": (
        "std_msgs/Header header\nuint8 framing_status\nuint8 magic\nuint8 len\n"
        "uint8 incompat_flags\nuint8 compat_flags\nuint8 seq\nuint8 sysid\n"
        "uint8 compid\nuint32 msgid\nuint16 checksum\nuint64[] payload64\n"
        "uint8[] signature\n"),
}
MAVLINK_STATUSTEXT = 253

KEYS = ("vx", "vy", "vz_up", "yaw_rate")
HOLD_STATES = ("Loiter", "RTL", "Land")           # the FSM streams nothing in these
MAX_START_SHIFT = 2         # how many ticks before the first candidate the mission may begin
# The recorder's log time of a tick's action against the node's time of the
# tick, after the common offset. At the start the node ticks on actions the
# recorder logged up to a period (0.1 s) earlier; half a period on top.
T_JITTER_S = 0.15
POSE_EXACT_M = 1e-9         # trajectory.json holds the pose message's own doubles
NEIGHBOURS = (1, -1)        # the bag's next / previous pose message
MAGNITUDE_TOL = 2e-4        # kpi.json rounds to 4 decimals; float32 body against float64 world
SECONDS_TOL = T_JITTER_S    # a duration on the recorder's clock against the node's


class ReplayError(Exception):
    """This run cannot be replayed from its bag; the message says why."""


# --------------------------------------------------------------------------- #
# reading the bag
# --------------------------------------------------------------------------- #

@dataclass
class BagStreams:
    """The topics of one bag, each a list of (log time ns, value) in log order."""
    reader: str = ""
    topics: dict = field(default_factory=dict)        # topic -> messages in the bag
    start_ns: int | None = None
    end_ns: int | None = None
    actions: list = field(default_factory=list)       # [vx, vy, vz_up, yaw_rate] float32
    setpoints: list = field(default_factory=list)
    intercepts: list = field(default_factory=list)    # the JSON document
    fences: list = field(default_factory=list)        # the JSON document
    mode_requests: list = field(default_factory=list)
    executed: list = field(default_factory=list)      # PositionTarget fields (export_episodes)
    poses: list = field(default_factory=list)         # {north, east, up, yaw_deg} about the EKF origin
    states: list = field(default_factory=list)        # {mode, armed, connected}
    markers: list = field(default_factory=list)       # (nonce8, mission seconds)
    problems: list = field(default_factory=list)


def f32(v: Any) -> float | None:
    """`v` as the float32 a Float32MultiArray carries; None when not a number."""
    try:
        return struct.unpack("<f", struct.pack("<f", float(v)))[0]
    except (TypeError, ValueError, OverflowError, struct.error):
        return None


def same_f32(a: Any, b: Any) -> bool:
    x, y = f32(a), f32(b)
    if x is None or y is None:
        return False
    return x == y or (math.isnan(x) and math.isnan(y))


def vec(d: Any) -> list | None:
    """An action dict (or a 4-list) as [vx, vy, vz_up, yaw_rate]; None for none."""
    if d is None:
        return None
    if isinstance(d, dict):
        return [d.get(k) for k in KEYS]
    return list(d)[:4]


def same_vec(a: Any, b: Any) -> bool:
    a, b = vec(a), vec(b)
    if a is None or b is None:
        return a is None and b is None
    return len(a) == len(b) == 4 and all(same_f32(x, y) for x, y in zip(a, b))


def is_zero(v: list | None) -> bool:
    return v is not None and all(f32(x) == 0.0 for x in v)


def statustext(msg) -> str | None:
    """The text of a STATUSTEXT in a mavros_msgs/Mavlink message, else None.
    The payload is MAVLink's wire order (severity, then 50 characters), which
    MAVLink 2 trims of trailing zeros."""
    if int(msg.msgid) != MAVLINK_STATUSTEXT:
        return None
    payload = b"".join(struct.pack("<Q", int(w)) for w in msg.payload64)[: int(msg.len)]
    return payload[1:51].split(b"\0")[0].decode("ascii", "replace")


def _deserialize(reader, raw: bytes, msgtype: str):
    try:
        return reader.deserialize(raw, msgtype)
    except Exception:                                            # noqa: BLE001
        store = getattr(reader, "typestore", None)
        if msgtype not in FALLBACK_MSGS or store is None:
            raise
        from rosbags.typesys import get_types_from_msg
        store.register(get_types_from_msg(FALLBACK_MSGS[msgtype], msgtype))
        return reader.deserialize(raw, msgtype)


def read_streams(bag_dir: Path) -> BagStreams:
    """The topics this tool reads, from a rosbag2 directory, with `rosbags`.
    Raises ReplayError when the package is missing or the bag cannot be read."""
    try:
        from rosbags.highlevel import AnyReader
    except ImportError:
        raise ReplayError(f"`rosbags` is not installed ({X.ROSBAGS_INSTALL_HINT})") from None
    kw: dict = {}
    try:
        from rosbags.typesys import Stores, get_typestore
        kw["default_typestore"] = get_typestore(getattr(Stores, "ROS2_JAZZY", None)
                                                or Stores.LATEST)
    except ImportError:
        pass
    s = BagStreams(reader=X._rosbags_version())
    try:
        try:
            cm = AnyReader([Path(bag_dir)], **kw)
        except TypeError:                 # rosbags < 0.10: no default_typestore
            cm = AnyReader([Path(bag_dir)])
        with cm as reader:
            for c in reader.connections:
                s.topics[c.topic] = s.topics.get(c.topic, 0) + int(c.msgcount)
            conns = [c for c in reader.connections if c.topic in READ_TOPICS]
            for conn, ts, raw in reader.messages(connections=conns):
                ts = int(ts)
                s.start_ns = ts if s.start_ns is None else min(s.start_ns, ts)
                s.end_ns = ts if s.end_ns is None else max(s.end_ns, ts)
                _take(s, reader, conn, ts, raw)
    except ReplayError:
        raise
    except Exception as e:                                       # noqa: BLE001
        raise ReplayError(f"the bag could not be read with rosbags: "
                          f"{type(e).__name__}: {e}") from None
    for name in ("actions", "setpoints", "intercepts", "fences", "mode_requests",
                 "executed", "poses", "states", "markers"):
        getattr(s, name).sort(key=lambda x: x[0])
    return s


def _take(s: BagStreams, reader, conn, ts: int, raw: bytes) -> None:
    topic = conn.topic
    m = _deserialize(reader, raw, conn.msgtype)
    if topic in (ACTION_TOPIC, SETPOINT_TOPIC):
        data = [float(x) for x in m.data]
        if len(data) != 4:
            s.problems.append(f"{topic}: a message of {len(data)} values, not 4, is left out")
            return
        (s.actions if topic == ACTION_TOPIC else s.setpoints).append((ts, data))
    elif topic in (INTERCEPT_TOPIC, FENCES_TOPIC):
        try:
            doc = json.loads(m.data)
        except ValueError:
            s.problems.append(f"{topic}: a message that is not JSON is left out")
            return
        (s.intercepts if topic == INTERCEPT_TOPIC else s.fences).append((ts, doc))
    elif topic == MODE_REQUEST_TOPIC:
        s.mode_requests.append((ts, str(m.data)))
    elif topic == EXECUTED_TOPIC:
        s.executed.append((ts, {
            "coordinate_frame": int(m.coordinate_frame), "type_mask": int(m.type_mask),
            "velocity": [float(m.velocity.x), float(m.velocity.y), float(m.velocity.z)],
            "yaw_rate": float(m.yaw_rate)}))
    elif topic == POSE_TOPIC:
        p, q = m.pose.position, m.pose.orientation
        # MAVROS ENU -> x North, y East, up, as the Shield node reads the topic.
        s.poses.append((ts, {"north": float(p.y), "east": float(p.x), "up": float(p.z),
                             "yaw_deg": X.heading_deg_from_enu_quaternion(q.x, q.y, q.z, q.w)}))
    elif topic == STATE_TOPIC:
        s.states.append((ts, {"mode": str(m.mode), "armed": bool(m.armed),
                              "connected": bool(m.connected)}))
    elif topic == MAVLINK_SINK_TOPIC:
        got = parse_marker(statustext(m))
        if got is not None:
            s.markers.append((ts, got))


# --------------------------------------------------------------------------- #
# the ticks, from the bag alone
# --------------------------------------------------------------------------- #

def place(ticks_ns: list[int], msgs: list) -> list[int]:
    """The tick of each message of one topic the Shield publishes at most
    once a tick, in order: the tick whose action was the last logged before
    the message, and no later than the tick before the next message's. Read
    from the last message back, so a message logged after the NEXT action
    (the node ran late: the first ticks of a mission) still goes to its own
    tick. An index below 0 says the mission began before `ticks_ns[0]`."""
    out, nxt = [], len(ticks_ns)
    for ns, _ in reversed(msgs):
        nxt = min(bisect_right(ticks_ns, ns) - 1, nxt - 1)
        out.append(nxt)
    out.reverse()
    return out


def assign(ticks_ns: list[int], msgs: list) -> tuple[list, dict]:
    """(per tick: its (log ns, value) or None; the account) - `place`, with
    the messages before the first tick left out and counted."""
    out: list = [None] * len(ticks_ns)
    acct = {"messages": len(msgs), "before_the_first_tick": 0,
            "logged_after_the_next_action": 0}
    for (ns, m), k in zip(msgs, place(ticks_ns, msgs)):
        if k < 0:
            acct["before_the_first_tick"] += 1
            continue
        out[k] = (ns, m)
        acct["logged_after_the_next_action"] += k + 1 < len(ticks_ns) and ns >= ticks_ns[k + 1]
    return out, acct


def world_from_body(body: list, yaw_deg: float) -> dict:
    """guardrail.frames.from_body's rotation (heading clockwise from North),
    on plain floats: a NaN channel stays a NaN instead of failing a model."""
    psi = math.radians(yaw_deg)
    c, s = math.cos(psi), math.sin(psi)
    return {"vx": body[0] * c - body[1] * s, "vy": body[0] * s + body[1] * c,
            "vz_up": body[2], "yaw_rate": body[3]}


def classify(raw: list, pub: list | None, intercept: dict | None,
             holding: bool) -> tuple[str, bool]:
    """Who flew this tick, as the flight log's `setpoint` says it - "pass",
    "brake" or "none" - and whether the aircraft is in a hold after it.
    INFERRED (module docstring): the bag carries no such field."""
    fsm = intercept.get("fsm") if intercept else None
    if pub is None:
        return "none", holding or fsm in HOLD_STATES
    if intercept is None:
        # Untouched: the Shield's answer IS the action. Anything else on the
        # topic is a hold's own leg.
        return ("pass", False) if same_vec(pub, raw) else ("none", True)
    stop = is_zero(pub) and not is_zero(raw)
    if fsm in HOLD_STATES:
        if holding:
            return "none", True
        # The tick that hands over is the Shield's: a stop or its action.
        return ("brake" if stop else "pass"), True
    return ("brake" if stop and fsm == "Brake" else "pass"), False


@dataclass
class Rebuilt:
    rows: list                      # every candidate tick from the first, flight-log shaped
    evidenced: int                  # ... of which the bag itself shows the node ticked
    window: dict
    account: dict


def rebuild(s: BagStreams) -> Rebuilt:
    """The episode's ticks from the bag's topics and nothing else."""
    if not s.actions:
        raise ReplayError(f"the bag holds no {ACTION_TOPIC} message: nothing marks a tick")
    outs = sorted([ns for ns, _ in s.setpoints] + [ns for ns, _ in s.intercepts])
    if not outs:
        raise ReplayError(f"the bag holds no {SETPOINT_TOPIC} or {INTERCEPT_TOPIC} message: "
                          f"the Shield published nothing, so no tick can be placed")
    a_ns = [ns for ns, _ in s.actions]
    first = bisect_right(a_ns, outs[0]) - 1
    if first < 0:
        raise ReplayError("the Shield's first message is logged before any action: the "
                          "first tick has no /vla/action_4d message (a timer tick)")
    last = bisect_right(a_ns, outs[-1]) - 1
    # The node can run behind the recorder at the start (it ticks on an
    # action the recorder logged up to a period earlier), so its first
    # message may be logged after the SECOND action. The order of its
    # messages says so: one that has to go before the first tick.
    behind = -min([0] + place(a_ns[first:], s.setpoints) + place(a_ns[first:], s.intercepts))
    if behind > MAX_START_SHIFT or behind > first:
        raise ReplayError(f"the Shield's messages do not fit one a tick from the first "
                          f"action before them ({behind} too many at the start)")
    first -= behind
    ticks_ns = a_ns[first:]
    sp, acct_sp = assign(ticks_ns, s.setpoints)
    ic, acct_ic = assign(ticks_ns, s.intercepts)
    ex, acct_ex = assign(ticks_ns, s.executed)
    home = [0.0, 0.0]
    home_source = (f"no {FENCES_TOPIC} message: the policy frame's home is not in the bag; "
                   f"positions are about the EKF origin")
    if s.fences:
        h = s.fences[0][1].get("home_policy_ne")
        if isinstance(h, (list, tuple)) and len(h) == 2:
            home = [float(h[0]), float(h[1])]
            home_source = f"home_policy_ne of the first {FENCES_TOPIC} message"
    p_ns = [ns for ns, _ in s.poses]
    st_ns = [ns for ns, _ in s.states]

    # Mission time: the Shield writes it on every intercept; else the t = 0
    # marker (logged after MAVROS relayed it, so late); else the first tick.
    pairs = sorted(ticks_ns[k] - int(round(float(v[1]["t"]) * 1e9)) for k, v in enumerate(ic)
                   if v is not None and isinstance(v[1].get("t"), (int, float)))
    if pairs:
        t0, t0_source = pairs[len(pairs) // 2], f"mission time on {INTERCEPT_TOPIC} (median)"
    elif s.markers:
        t0 = min(ns - int(round(t * 1e9)) for ns, (_, t) in s.markers)
        t0_source = (f"the t = 0 marker in {MAVLINK_SINK_TOPIC} (logged after MAVROS "
                     f"relayed it: late by its delay)")
    else:
        t0, t0_source = ticks_ns[0], "the first tick's action (its mission time taken as 0)"

    rows, holding = [], False
    for k, ns in enumerate(ticks_ns):
        raw = s.actions[first + k][1]
        pub = sp[k][1] if sp[k] is not None else None
        icpt = ic[k][1] if ic[k] is not None else None
        kind, holding = classify(raw, pub, icpt, holding)
        j = bisect_right(p_ns, ns) - 1
        pose = s.poses[j][1] if j >= 0 else None
        m = bisect_right(st_ns, ns) - 1
        ops = [str(o) for o in (icpt.get("repairs") or [])] if icpt else []
        row: dict[str, Any] = {
            "t": round((ns - t0) / 1e9, 3), "tick": k + 1, "frame": "body",
            "x": None if pose is None else pose["north"] + home[0],
            "y": None if pose is None else pose["east"] + home[1],
            "up": None if pose is None else pose["up"],
            "yaw_deg": None if pose is None else pose["yaw_deg"],
            "raw_body": dict(zip(KEYS, raw)),
            "emitted_body": None if pub is None else dict(zip(KEYS, pub)),
            "setpoint": kind, "flown": kind != "none",
            "mode": s.states[m][1]["mode"] if m >= 0 else None,
            "touched": icpt is not None,
            "violations": [{"rule_id": str(r)} for r in (icpt.get("violations") or [])]
            if icpt else [],
            "repairs": [{"operator": o} for o in ops],
            "executed": None,
            "_bag": {"action_ns": ns, "action_index": first + k + 1, "pose_index": j,
                     "state_index": m},
        }
        if icpt is not None:
            row["fsm_state_after"] = icpt.get("fsm")
        if ex[k] is not None:
            w, why = X.setpoint_world(ex[k][1])
            row["executed"] = w if w is not None else {"not_converted": why}
        # The action pair the repair magnitude is measured on, in one frame.
        # A tick the FSM stopped published the stop, not the Shield's repaired
        # action, unless the stop is the Shield's own (its Brake operator).
        if pose is not None:
            row["raw"] = world_from_body(raw, pose["yaw_deg"])
            if kind == "pass" or (kind == "brake" and "Brake" in ops):
                row["emitted"] = world_from_body(pub, pose["yaw_deg"])
        rows.append(row)
    window = {
        "tick_source": f"{ACTION_TOPIC} messages",
        "first_tick": {"action_index": first + 1, "log_time_ns": ticks_ns[0],
                       "source": ("the action the Shield's first message answers" + (
                           f" ({behind} before the last one logged ahead of it: the "
                           f"order of the Shield's messages needs the room)" if behind else ""))},
        "last_evidenced_tick": {"action_index": last + 1, "log_time_ns": a_ns[last],
                                "source": "the last message the Shield published"},
        "actions_before_the_first_tick": first,
        "actions_after_the_last_evidenced_tick": len(a_ns) - 1 - last,
        "t0_ns": t0, "t0_source": t0_source,
        "markers": [{"log_time_ns": ns, "nonce": n, "t": t} for ns, (n, t) in s.markers],
        "first_tick_after_marker_ms": (
            None if not s.markers else
            round((ticks_ns[0] - min(ns - int(round(t * 1e9)) for ns, (_, t) in s.markers))
                  / 1e6, 3)),
        "home_policy_ne": home, "home_source": home_source,
    }
    account = {"setpoints": acct_sp, "intercepts": acct_ic, "executed": acct_ex,
               "ticks_without_a_pose": sum(1 for r in rows[: last - first + 1]
                                           if r["x"] is None)}
    return Rebuilt(rows=rows, evidenced=last - first + 1, window=window, account=account)


# --------------------------------------------------------------------------- #
# the KPI table the topics can carry
# --------------------------------------------------------------------------- #

BAG = "bag"
BAG_POLICY = "bag + policy_g<N>.json (rule priorities)"
# kpi.json field -> where its inputs are.
FROM_BAG = {
    "ticks": BAG, "shield_ticks": BAG, "autopilot_ticks": BAG,
    "repair_count": BAG, "repairs_per_episode": BAG,
    "repaired_ticks": BAG, "mean_repair_magnitude_mps": BAG,
    "max_repair_magnitude_mps": BAG, "mean_yaw_repair_dps": BAG,
    "mean_intervention_s": BAG, "max_intervention_s": BAG,
    "intervention_episodes": BAG, "intervention_censored": BAG,
    "p0_violation_ticks": BAG_POLICY, "failsafe_expected_ticks": BAG_POLICY,
    "failsafe_permitted_ticks": BAG_POLICY, "violation_free_ticks": BAG_POLICY,
    "violations_by_risk_level": BAG_POLICY,
}
_RECHECK = ("the Shield's re-check of the action it flew (emitted_violations) is on no "
            "topic; it needs the rules and the Shield (WP4-11's replay)")
_UNSAFE = ("whether a position is legal (the row's `unsafe`) is on no topic: the "
           "trajectory is in the bag, the rule check is not")
_THETA = ("each repair's size in metres and the re-check are on no topic "
          "(/shield/intercept carries operator names only)")
_FSM = "the FSM's state on a tick the Shield did not touch is on no topic"
_METRICS = ("decided from metrics.json (zone dwell, the goal, the FSM's summary) and "
            "the rows' `unsafe`, none of which a topic carries")
NOT_FROM_BAG = {
    "p0_violation_escape_rate": _RECHECK, "p0_violation_escape_rate_status": _RECHECK,
    "p0_escapes": _RECHECK, "p0_ticks_not_measurable": _RECHECK,
    "p0_acted_tick_share": _RECHECK, "failsafe_trigger_correctness": _RECHECK,
    "failsafe_correct_ticks": _RECHECK,
    "p0_ticks_blind": "the rows' `off_map` flag is on no topic",
    "collision_contacts": "the simulator's contact reports are not recorded in the bag",
    "collision_objects": "the simulator's contact reports are not recorded in the bag",
    "false_trigger_ticks": _FSM, "false_trigger_rate": _FSM, "failsafe_instrumented": _FSM,
    "repair_success_rate": _THETA, "repair_success_status": _THETA,
    "repair_attempt_ticks": _THETA, "repair_success_ticks": _THETA,
    "repair_unmeasured_ticks": _THETA, "repair_outcomes": _THETA,
    "repair_not_applied_ticks": _THETA, "repair_not_applied_autopilot_ticks": _THETA,
    "repair_floored_ticks": _THETA, "repairs_to_standstill": _THETA,
    "repairs_converged_while_unsafe": _THETA, "repair_success_basis": _THETA,
    "repair_theta_unchecked_ticks": _THETA, "repair_theta_sources": _THETA,
    "repair_ticks_not_measurable": _THETA,
    "mean_time_to_safe_s": _UNSAFE, "max_time_to_safe_s": _UNSAFE,
    "time_to_safe_episodes": _UNSAFE, "time_to_safe_censored": _UNSAFE,
    "time_to_safe_not_measurable": _UNSAFE, "unsafe_p0_ticks": _UNSAFE,
    "mission_started_unsafe": _UNSAFE,
    "outcome": _METRICS, "mission_success": _METRICS, "mission_fail_reasons": _METRICS,
    "mission_goal_status": _METRICS,
    "violations_by_type": "a violation's category is not on /shield/intercept (rule ids only)",
    "repair_magnitude_by_priority": "a breakdown of the magnitudes above; not rebuilt here",
    "repair_magnitude_by_operator_class": "a breakdown of the magnitudes above; not rebuilt here",
    "repair_magnitude_breakdown_status": "a breakdown of the magnitudes above; not rebuilt here",
}
# Not KPIs: the table's own bookkeeping, left out of the account.
NOT_A_KPI = ("kpi_version", "episode_id", "manifest", "policy_source", "kpi_grade",
             "kpi_grade_reasons", "failsafe_trigger_correctness_is", "repair_theta_m",
             "repair_theta_horizon_s", "intervention_not_measurable", "harness")
_MAGNITUDES = ("repaired_ticks", "mean_repair_magnitude_mps", "max_repair_magnitude_mps",
               "mean_yaw_repair_dps")
_INTERCEPT_FIELDS = ("repair_count", "repairs_per_episode", "mean_intervention_s",
                     "max_intervention_s", "intervention_episodes", "intervention_censored",
                     *_MAGNITUDES, *(f for f, src in FROM_BAG.items() if src == BAG_POLICY))
_SECONDS = ("mean_intervention_s", "max_intervention_s")


def bag_kpis(rows: list[dict], priorities: dict[str, str] | None) -> dict:
    """guardrail.kpi.compute on the rebuilt rows: the same function that
    wrote kpi.json, with no metrics.json. Only FROM_BAG's fields are read
    off the result."""
    return K.compute([{k: v for k, v in r.items() if not k.startswith("_")} for r in rows],
                     priorities or {}, None)


def _equal(name: str, a: Any, b: Any) -> bool:
    if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None:
        return a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        tol = (SECONDS_TOL if name in _SECONDS else
               MAGNITUDE_TOL if isinstance(a, float) or isinstance(b, float) else 0)
        return abs(float(a) - float(b)) <= tol
    return a == b


def compare_kpis(rows: list[dict], stored: dict | None, priorities: dict[str, str] | None,
                 *, arm: str | None, past_end: int) -> dict:
    """The KPI table from the bag against kpi.json, field by field."""
    fresh = bag_kpis(rows, priorities)
    stops = sum(1 for r in rows if r["setpoint"] == "brake" and r["repairs"]
                and "emitted" not in r)
    fields: dict[str, dict] = {}
    first = None
    counts = {"equal": 0, "differs": 0, "not_on_this_run": 0, "not_from_the_bag": 0}
    names = list(FROM_BAG) + [f for f in (stored or {}) if f not in FROM_BAG
                              and f not in NOT_A_KPI]
    for name in names:
        if name not in FROM_BAG:
            fields[name] = {"source": "not from the bag",
                            "why": NOT_FROM_BAG.get(name, "not rebuilt by this tool"),
                            "kpi_json": (stored or {}).get(name)}
            counts["not_from_the_bag"] += 1
            continue
        rec: dict[str, Any] = {"source": FROM_BAG[name], "bag": fresh.get(name)}
        why_not = None
        if arm == "off" and name in _INTERCEPT_FIELDS:
            why_not = (f"a Shield-off arm (metrics.json): the node publishes "
                       f"{INTERCEPT_TOPIC} only with the Shield on, so the bag carries "
                       f"no violation or repair")
        elif name in _MAGNITUDES and stops:
            why_not = (f"{stops} tick(s) the FSM stopped: the stop was published, the "
                       f"action the Shield repaired (which the KPI measures) is on no topic")
        elif FROM_BAG[name] == BAG_POLICY and priorities is None:
            why_not = "no policy_g<N>.json beside the bag: a rule's priority is not on a topic"
        if past_end and name in ("ticks", "autopilot_ticks"):
            rec["source"] = (f"{FROM_BAG[name]} + flight_log.jsonl (the mission's end: "
                             f"{past_end} tick(s) after the Shield's last message)")
        if why_not:
            rec.update(not_on_this_run=why_not, kpi_json=(stored or {}).get(name))
            rec.pop("bag")
            counts["not_on_this_run"] += 1
        elif stored is None or name not in stored:
            rec["kpi_json"] = "absent"
        else:
            rec["kpi_json"] = stored[name]
            rec["equal"] = _equal(name, rec["bag"], stored[name])
            counts["equal" if rec["equal"] else "differs"] += 1
            if not rec["equal"] and first is None:
                first = {"layer": "kpi", "field": name, "tick": None,
                         "bag": rec["bag"], "log": stored[name]}
        fields[name] = rec
    verdict = ("no_kpi_json" if stored is None else
               "differs" if counts["differs"] else "identical")
    return {"verdict": verdict, **counts, "first_difference": first, "fields": fields}


# --------------------------------------------------------------------------- #
# against the flight log
# --------------------------------------------------------------------------- #

def _pose_match(s: BagStreams, home: list, j: int, want: tuple, exact: bool) -> str | None:
    """Is the log's point the bag's pose message `j`, or a neighbour of it?
    `exact`: the log holds the message's own doubles (trajectory.json), else
    the flight log's rounding (3 decimals)."""
    def same(i: int) -> bool:
        if not 0 <= i < len(s.poses):
            return False
        p = s.poses[i][1]
        got = (p["north"] + home[0], p["east"] + home[1], p["up"])
        if exact:
            return all(abs(a - b) <= POSE_EXACT_M for a, b in zip(got, want))
        return tuple(round(v, 3) for v in got) == want
    if same(j):
        return "exact"
    for d in NEIGHBOURS:
        if same(j + d):
            return "next" if d > 0 else "previous"
    return None


def _align(rb: Rebuilt, rows: list[dict]) -> tuple[list, str]:
    """Per log row, the index of its tick among the rebuilt rows (None: the
    bag marks no such tick), and how they were paired. With `action_seq` on
    every row the log's own count of actions pairs them; else by position."""
    seqs = [r.get("action_seq") for r in rows]
    if rows and all(isinstance(q, int) and not isinstance(q, bool) for q in seqs):
        out: list = []
        for i, q in enumerate(seqs):
            b = q - seqs[0]
            repeat = i > 0 and q == seqs[i - 1]      # no new action: a timer tick
            out.append(b if 0 <= b < len(rb.rows) and not repeat else None)
        return out, "the log's action_seq against the bag's action count"
    return [i if i < len(rb.rows) else None for i in range(len(rows))], "position"


def compare_ticks(s: BagStreams, rb: Rebuilt, rows: list[dict],
                  trajectory: list | None = None) -> dict:
    """The rebuilt ticks against flight_log.jsonl (and trajectory.json)."""
    pairs, how = _align(rb, rows)
    home = rb.window["home_policy_ne"]
    diffs: dict[str, int] = {}
    first: dict | None = None
    pose = {"exact": 0, "next": 0, "previous": 0, "differs": 0}
    worst_m = 0.0
    dts: list[tuple[int, float]] = []
    compared = 0

    def differ(layer: str, name: str, row: dict, bag: Any, log: Any) -> None:
        nonlocal first
        diffs[name] = diffs.get(name, 0) + 1
        if first is None:
            first = {"layer": layer, "field": name, "tick": row.get("tick"),
                     "t": row.get("t"), "bag": bag, "log": log}

    exact_traj = isinstance(trajectory, list) and len(trajectory) == len(rows)
    for i, (row, b) in enumerate(zip(rows, pairs)):
        if b is None:
            differ("ticks", "tick", row, None,
                   f"a tick with no {ACTION_TOPIC} message of its own (trigger "
                   f"{row.get('tick_trigger')!r}): the bag marks no such tick")
            continue
        got = rb.rows[b]
        compared += 1
        if not same_vec(got["raw_body"], row.get("raw_body")):
            differ("ticks", "raw_body", row, got["raw_body"], row.get("raw_body"))
        if not same_vec(got["emitted_body"], row.get("emitted_body")):
            differ("ticks", "emitted_body", row, got["emitted_body"], row.get("emitted_body"))
        for name in ("setpoint", "mode"):
            if name in row and got[name] != row[name]:
                if name == "mode" and _mode_at_edge(s, got, row[name]):
                    continue
                differ("ticks", name, row, got[name], row[name])
        if "touched" in row and bool(row["touched"]) != got["touched"]:
            differ("ticks", "touched", row, got["touched"], row["touched"])
        if got["touched"]:
            # Only a tick with an intercept carries these on a topic.
            ids = [v.get("rule_id") for v in row.get("violations") or []]
            if [v["rule_id"] for v in got["violations"]] != ids:
                differ("ticks", "violations", row, [v["rule_id"] for v in got["violations"]], ids)
            ops = [r.get("operator") for r in row.get("repairs") or []]
            if [r["operator"] for r in got["repairs"]] != ops:
                differ("ticks", "repairs", row, [r["operator"] for r in got["repairs"]], ops)
            if "fsm_state_after" in row and got.get("fsm_state_after") != row["fsm_state_after"]:
                differ("ticks", "fsm_state_after", row, got.get("fsm_state_after"),
                       row["fsm_state_after"])
        why = _executed_differs(got, row)
        if why:
            differ("ticks", "executed", row, got["executed"], why)
        # The pose: the logged point must be a pose message of the bag.
        if exact_traj:
            p = trajectory[i]
            want, exact = (p.get("x"), p.get("y"), p.get("up")), True
        else:
            want, exact = (row.get("x"), row.get("y"), row.get("up")), False
        if all(isinstance(v, (int, float)) for v in want):
            m = _pose_match(s, home, got["_bag"]["pose_index"], want, exact)
            pose[m or "differs"] += 1
            if got["x"] is not None:
                worst_m = max(worst_m, math.dist((got["x"], got["y"], got["up"]), want))
            if m is None:
                differ("trajectory", "pose", row,
                       None if got["x"] is None else [got["x"], got["y"], got["up"]],
                       list(want))
        if isinstance(row.get("t"), (int, float)):
            dts.append((i, got["t"] - float(row["t"])))
    offset = sorted(d for _, d in dts)[len(dts) // 2] if dts else None
    jitter = max((abs(d - offset) for _, d in dts), default=None)
    if jitter is not None and jitter > T_JITTER_S:
        i = next(i for i, d in dts if abs(d - offset) > T_JITTER_S)
        differ("ticks", "t", rows[i], rb.rows[pairs[i]]["t"], rows[i]["t"])
    used = [b for b in pairs if b is not None]
    last_used = max(used) if used else -1
    past_end = sum(1 for b in used if b >= rb.evidenced)
    if last_used + 1 < rb.evidenced:
        # The Shield published after the log's last row.
        differ("ticks", "ticks", {"tick": None}, rb.evidenced, len(rows))
    skipped = sorted(set(range(last_used + 1)) - set(used))
    if skipped:
        r = rb.rows[skipped[0]]
        differ("ticks", "tick", {"tick": None, "t": r["t"]},
               f"action {r['_bag']['action_index']} in the mission's window", "no row ticked on it")
    tick_fields = [k for k in diffs if k != "pose"]
    return {
        "paired_by": how, "log_rows": len(rows), "compared": compared,
        "bag_ticks_evidenced": rb.evidenced, "ticks_past_bag_end": past_end,
        "rows_the_bag_marks_no_tick_for": sum(1 for b in pairs if b is None),
        "bag_actions_no_row_ticked_on": len(skipped),
        "ticks": {"verdict": "differs" if tick_fields else "identical",
                  "differences": {k: diffs[k] for k in tick_fields},
                  "t_offset_ms": None if offset is None else round(offset * 1e3, 3),
                  "t_jitter_ms": None if jitter is None else round(jitter * 1e3, 3),
                  # How long before the node's first tick the recorder had
                  # logged the action it ran on (the node's lag at the start).
                  "first_action_logged_before_its_tick_ms": (
                      round((offset - dts[0][1]) * 1e3, 3) if dts and dts[0][0] == 0 else None)},
        "trajectory": {"verdict": "differs" if pose["differs"] else "identical",
                       "against": ("trajectory.json (the pose message's own doubles)"
                                   if exact_traj else "flight_log.jsonl (3 decimals)"),
                       "points": sum(pose.values()), **pose,
                       "bag_order_max_error_m": round(worst_m, 4)},
        "first_difference": first,
        "rows_used": last_used + 1,
    }


def _mode_at_edge(s: BagStreams, got: dict, logged: Any) -> bool:
    """The log's mode is the bag's next or previous /mavros/state message: a
    mode change logged beside the tick, taken in the other order."""
    m = got["_bag"]["state_index"]
    return any(0 <= i < len(s.states) and s.states[i][1]["mode"] == logged
               for i in (m + 1, m - 1))


def _executed_differs(got: dict, row: dict) -> str | None:
    """What MAVROS was sent against what the log says was streamed; a string
    (the log's side) when they disagree."""
    ex, sent = got["executed"], row.get("emitted_body") is not None
    if (ex is None) == sent:
        return ("a setpoint was streamed (emitted_body)" if sent else
                "nothing was streamed (emitted_body null)")
    if ex is None or "not_converted" in ex or row.get("setpoint") not in ("pass", "brake"):
        return None
    want = ({"vx": 0.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}
            if row.get("setpoint") == "brake" else row.get("emitted"))
    if not isinstance(want, dict) or X._agrees(ex, want):
        return None
    return f"emitted {want}"


# --------------------------------------------------------------------------- #
# one run
# --------------------------------------------------------------------------- #

@dataclass
class Episode:
    """A run's bag, and the records it is compared with (None when absent)."""
    name: str
    source: str
    bag_dir: Path | None
    rows: list | None = None
    trajectory: list | None = None
    kpi: dict | None = None
    priorities: dict | None = None
    priority_source: str | None = None
    arm: str | None = None
    not_from_bag: list = field(default_factory=list)


def _jsonl(text: str) -> list[dict]:
    return [json.loads(line) for line in text.splitlines() if line.strip()]


def _json_file(p: Path) -> Any:
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def find_bag(run: Path) -> Path | None:
    for name in BAG_DIRS:
        if (run / name / "metadata.yaml").is_file():
            return run / name
    return None


def _last_generation_priorities(policies: dict) -> dict | None:
    """rule id -> priority of the last generation: the policy the node scored
    kpi.json with (guardrail.kpi.rule_priorities at the end of the flight)."""
    return K.rule_priorities(policies[max(policies)]) if policies else None


def load_run(run: Path) -> Episode:
    """A run folder: its bag/ (or episode.bag2/) and the records beside it."""
    from guardrail.replay import _generation_files, generation_table
    ep = Episode(name=run.name, source=str(run), bag_dir=find_bag(run))
    log = run / "flight_log.jsonl"
    if log.is_file():
        ep.rows = _jsonl(log.read_text(encoding="utf-8"))
        ep.not_from_bag.append("flight_log.jsonl (compared with, after the rebuild)")
    traj = _json_file(run / "trajectory.json")
    if isinstance(traj, list):
        ep.trajectory = traj
        ep.not_from_bag.append("trajectory.json (compared with)")
    kpi = _json_file(run / "kpi.json")
    if isinstance(kpi, dict):
        ep.kpi = kpi
        ep.not_from_bag.append("kpi.json (compared with)")
    try:
        _, policies, problems = generation_table(_generation_files(run))
    except Exception:                                            # noqa: BLE001
        policies, problems = {}, ["unreadable"]
    if policies and not problems:
        ep.priorities = _last_generation_priorities(policies)
        ep.priority_source = f"policy_g{max(policies)}.json (the last generation)"
        ep.not_from_bag.append(f"{ep.priority_source}: rule priorities")
    metrics = _json_file(run / "metrics.json")
    if isinstance(metrics, dict) and metrics.get("shield") in ("on", "off"):
        ep.arm = metrics["shield"]
        ep.not_from_bag.append("metrics.json `shield` (the arm, to say what a Shield-off "
                               "bag cannot carry)")
    return ep


def load_bundle(path: Path, into: Path) -> Episode:
    """A replay bundle (guardrail.replay): its verified bag members unpacked
    into `into`, and the log, KPI table and generations it carries."""
    from guardrail.replay import BAG_DIR, EPISODE, read_replay
    try:
        got = read_replay(path)
    except (ValueError, KeyError, tarfile.TarError, OSError, EOFError) as e:
        # EOFError: a truncated .tar.gz, which gzip raises past tarfile.
        raise ReplayError(f"{path}: {type(e).__name__}: {e}") from None
    declared = got["index"].get("members") or {}
    prefix = EPISODE + BAG_DIR + "/"
    ep = Episode(name=path.name.replace(".replay.tar.gz", ""), source=str(path), bag_dir=None,
                 rows=got["rows"], kpi=got["kpi"])
    ep.not_from_bag += ["the bundle's flight_log.jsonl (compared with)",
                        "the bundle's kpi.json (compared with)"]
    with tarfile.open(path, "r:gz") as tar:
        for name in got["bag_files"]:
            data = tar.extractfile(name).read()
            if hashlib.sha256(data).hexdigest() != declared.get(name):
                raise ReplayError(f"{path}: {name} does not match its digest")
            # A rosbag2 folder is flat: each file by its own name only.
            (into / Path(name[len(prefix):]).name).write_bytes(data)
    if (into / "metadata.yaml").is_file():
        ep.bag_dir = into
    pols = dict(got["generation_policies"]) or {0: got["policy"]}
    ep.priorities = _last_generation_priorities(pols)
    ep.priority_source = "the bundle's last policy generation"
    ep.not_from_bag.append(f"{ep.priority_source}: rule priorities")
    arm = (got["metrics"] or {}).get("shield")
    if arm in ("on", "off"):
        ep.arm = arm
        ep.not_from_bag.append("the bundle's metrics.json `shield` (the arm)")
    return ep


def replay(ep: Episode) -> tuple[dict, list[dict] | None]:
    """(the verdict record of one episode, its rebuilt ticks or None)."""
    rec: dict[str, Any] = {"run": ep.name, "source": ep.source, "verdict": None,
                           "first_difference": None}
    try:
        if ep.bag_dir is None:
            raise ReplayError(f"no rosbag2 here ({' or '.join(d + '/' for d in BAG_DIRS)} "
                              f"with a metadata.yaml)")
        s = read_streams(ep.bag_dir)
        rb = rebuild(s)
    except ReplayError as e:
        rec.update(verdict="not_replayable", why=str(e))
        return rec, None
    rec["bag"] = {"dir": ep.bag_dir.name, "reader": s.reader,
                  "messages": sum(s.topics.values()),
                  "topics_read": {t: s.topics.get(t, 0) for t in READ_TOPICS},
                  "problems": s.problems}
    rec["window"] = rb.window
    rec["account"] = rb.account
    rec["not_from_the_bag"] = ep.not_from_bag
    limits: list[str] = []
    n = rb.evidenced
    if ep.rows is None:
        rec["compared_with_the_log"] = None
        limits.append("no flight_log.jsonl: the ticks and the trajectory are rebuilt, not compared")
        first = None
        past_end = 0
    else:
        if not ep.rows:
            rec.update(verdict="not_replayable",
                       why="the flight log holds no rows: nothing to compare the bag with")
            return rec, rb.rows[:n]
        cmp_ = compare_ticks(s, rb, ep.rows, ep.trajectory)
        first = cmp_.pop("first_difference")
        n = max(n, cmp_.pop("rows_used"))
        past_end = cmp_["ticks_past_bag_end"]
        rec["compared_with_the_log"] = cmp_
        if past_end:
            limits.append(
                f"{past_end} tick(s) after the Shield's last message: nothing was "
                f"published on them, so the bag holds their actions and poses but not "
                f"that the node ticked; their number is the flight log's")
        nb = cmp_["trajectory"]["next"] + cmp_["trajectory"]["previous"]
        if nb:
            limits.append(
                f"{nb} of {cmp_['trajectory']['points']} logged points are the bag's "
                f"next or previous pose message, not the last logged before the tick's "
                f"action: the bag does not say which message a tick consumed")
    used = rb.rows[:n]
    rec["rebuilt"] = {
        "ticks": n, "evidenced_by_the_bag": rb.evidenced,
        "setpoint": {k: sum(1 for r in used if r["setpoint"] == k)
                     for k in ("pass", "brake", "none")},
        "with_a_published_setpoint": sum(1 for r in used if r["emitted_body"] is not None),
        "with_an_executed_setpoint": sum(1 for r in used if r["executed"] is not None),
        "with_an_intercept": sum(1 for r in used if r["touched"]),
        "modes": {m: sum(1 for r in used if r["mode"] == m)
                  for m in sorted({str(r["mode"]) for r in used})},
        "path_m": round(sum(math.dist((a["x"], a["y"], a["up"]), (b["x"], b["y"], b["up"]))
                            for a, b in zip(used, used[1:])
                            if a["x"] is not None and b["x"] is not None), 3),
    }
    kp = compare_kpis(used, ep.kpi, ep.priorities, arm=ep.arm, past_end=past_end)
    rec["kpi"] = kp
    if first is None:
        first = kp["first_difference"]
    kp.pop("first_difference")
    rec["limits"] = limits
    rec["first_difference"] = first
    rec["verdict"] = "differs" if first is not None else "identical"
    if ep.rows is None or ep.kpi is None:
        rec["verdict_covers"] = ("the bag was rebuilt; " +
                                 ("no flight log" if ep.rows is None else "the flight log") +
                                 (" and no kpi.json" if ep.kpi is None else " and kpi.json") +
                                 " to compare with")
    return rec, used


def replay_path(path: Path) -> tuple[dict, list[dict] | None]:
    """A run folder or a .replay.tar.gz bundle."""
    if path.is_dir():
        return replay(load_run(path))
    if path.is_file() and path.name.endswith(".tar.gz"):
        with tempfile.TemporaryDirectory(prefix="replay-from-bag-") as td:
            try:
                ep = load_bundle(path, Path(td))
            except ReplayError as e:
                return {"run": path.name, "source": str(path), "verdict": "not_replayable",
                        "first_difference": None, "why": str(e)}, None
            return replay(ep)
    return {"run": path.name, "source": str(path), "verdict": "not_replayable",
            "first_difference": None,
            "why": "neither a run folder nor a .replay.tar.gz bundle"}, None


# --------------------------------------------------------------------------- #
# a campaign
# --------------------------------------------------------------------------- #

def campaign_runs(path: Path) -> list[Path]:
    """The run folders a hil campaign accepted, from its sweep.json (`path`,
    or the folder holding it): every result with a `run` that is not an
    error, as tools/kpi_report.py load_campaign takes them. A run whose
    stored path is gone is looked for beside the summary's folder."""
    sw = path / "sweep.json" if path.is_dir() else path
    doc = _json_file(sw)
    if not isinstance(doc, dict) or not isinstance(doc.get("results"), list):
        raise ReplayError(f"{sw}: not an experiments/sweep_scenarios.py result (no `results`)")
    out: list[Path] = []
    for r in doc["results"]:
        if not r.get("run") or r.get("status") == "error":
            continue
        p = Path(str(r["run"]))
        if not p.is_dir():
            moved = sw.parent.parent / p.parent.name / p.name
            p = moved if moved.is_dir() else p
        out.append(p)
    return out


def summarise(records: list[dict], command: str, seconds: float) -> dict:
    """The campaign summary: the count per verdict and per layer, each KPI
    field's account over the runs, the limits, and every first difference."""
    by: dict[str, int] = {}
    layers = {"ticks": {}, "trajectory": {}, "kpi": {}}
    fields: dict[str, dict] = {}
    totals = {"ticks": 0, "ticks_evidenced_by_the_bag": 0, "ticks_past_bag_end": 0,
              "runs_whose_end_is_not_in_the_bag": 0, "pose_points": 0, "pose_exact": 0,
              "pose_neighbour": 0, "pose_differs": 0,
              "rows_the_bag_marks_no_tick_for": 0}
    firsts, not_replayable, per_run = [], [], []
    reader = None
    for rec in records:
        v = rec["verdict"]
        by[v] = by.get(v, 0) + 1
        row = {"run": rec["run"], "verdict": v}
        if v == "not_replayable":
            not_replayable.append({"run": rec["run"], "why": rec.get("why")})
            per_run.append(row)
            continue
        reader = reader or (rec.get("bag") or {}).get("reader")
        c = rec.get("compared_with_the_log")
        if c:
            for layer in ("ticks", "trajectory"):
                lv = c[layer]["verdict"]
                layers[layer][lv] = layers[layer].get(lv, 0) + 1
            tr = c["trajectory"]
            totals["ticks"] += c["log_rows"]
            totals["ticks_past_bag_end"] += c["ticks_past_bag_end"]
            totals["runs_whose_end_is_not_in_the_bag"] += bool(c["ticks_past_bag_end"])
            totals["rows_the_bag_marks_no_tick_for"] += c["rows_the_bag_marks_no_tick_for"]
            totals["pose_points"] += tr["points"]
            totals["pose_exact"] += tr["exact"]
            totals["pose_neighbour"] += tr["next"] + tr["previous"]
            totals["pose_differs"] += tr["differs"]
            row.update(ticks=c["ticks"]["verdict"], trajectory=tr["verdict"],
                       log_rows=c["log_rows"], ticks_past_bag_end=c["ticks_past_bag_end"])
        totals["ticks_evidenced_by_the_bag"] += rec["rebuilt"]["evidenced_by_the_bag"]
        kp = rec["kpi"]
        layers["kpi"][kp["verdict"]] = layers["kpi"].get(kp["verdict"], 0) + 1
        row["kpi"] = kp["verdict"]
        for name, f in kp["fields"].items():
            acc = fields.setdefault(name, {"source": f["source"].split(" + flight_log")[0],
                                           "equal": 0, "differs": 0, "not_on_this_run": 0,
                                           "not_compared": 0})
            if f["source"] == "not from the bag":
                acc.setdefault("why", f.get("why"))
                acc["not_compared"] += 1
            elif "not_on_this_run" in f:
                acc["not_on_this_run"] += 1
            elif "equal" in f:
                acc["equal" if f["equal"] else "differs"] += 1
            else:
                acc["not_compared"] += 1
        if rec["first_difference"] is not None:
            firsts.append({"run": rec["run"], **rec["first_difference"]})
        per_run.append(row)
    return {
        "_what": ("Each run's ticks, trajectory and KPI table rebuilt from its rosbag2 "
                  "alone (tools/replay_from_bag.py) and compared with its flight log, "
                  "trajectory.json and kpi.json. Not a re-flight and not a bit-for-bit "
                  "replay: the Shield and the FSM are not run again."),
        "_command": command, "_seconds": round(seconds, 1), "reader": reader,
        "runs": len(records), "verdicts": by, "layers": layers, "totals": totals,
        "kpi_fields": fields,
        "first_differences": firsts, "not_replayable": not_replayable, "per_run": per_run,
    }


def _dump(path: Path, doc: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(X.json_safe(doc), fh, indent=1)
        fh.write("\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="run folders (with bag/ or episode.bag2/) or "
                    ".replay.tar.gz bundles")
    ap.add_argument("--campaign", action="append", default=[], metavar="DIR_OR_SWEEP_JSON",
                    help="every run a hil campaign's sweep.json accepted; repeatable")
    ap.add_argument("--out", default=None, metavar="DIR",
                    help="write <run>.replay_from_bag.json per run and summary.json here "
                         "(nothing is ever written into a run folder)")
    ap.add_argument("--write-ticks", action="store_true",
                    help="with --out: also <run>.bag_ticks.jsonl, the rebuilt ticks")
    ap.add_argument("--limit", type=int, default=None, help="only the first N runs")
    a = ap.parse_args(argv)
    paths = [Path(p) for p in a.runs]
    try:
        for c in a.campaign:
            paths += campaign_runs(Path(c))
    except ReplayError as e:
        print(f"replay_from_bag: {e}", file=sys.stderr)
        return 2
    if a.limit is not None:
        paths = paths[: max(0, a.limit)]
    if not paths:
        ap.error("no run given (a run folder, a bundle, or --campaign)")
    if a.write_ticks and not a.out:
        ap.error("--write-ticks needs --out")
    if not X.have_rosbags():
        print(f"replay_from_bag: `rosbags` is not installed ({X.ROSBAGS_INSTALL_HINT})",
              file=sys.stderr)
        return 3
    out = Path(a.out) if a.out else None
    started = time.time()
    records = []
    for i, p in enumerate(paths, 1):
        rec, ticks = replay_path(p)
        records.append(rec)
        fd = rec.get("first_difference")
        tail = (f": {rec.get('why')}" if rec["verdict"] == "not_replayable" else
                f": first at {fd['layer']}.{fd['field']}, tick {fd['tick']}" if fd else "")
        print(f"[{i}/{len(paths)}] {rec['verdict']:14s} {rec['run']}{tail}", flush=True)
        if out is not None:
            _dump(out / f"{rec['run']}.replay_from_bag.json", rec)
            if a.write_ticks and ticks is not None:
                with open(out / f"{rec['run']}.bag_ticks.jsonl", "w", encoding="utf-8",
                          newline="\n") as fh:
                    for r in ticks:
                        fh.write(json.dumps(X.json_safe(r)) + "\n")
    cmd = "python tools/replay_from_bag.py " + " ".join(argv if argv is not None else sys.argv[1:])
    summary = summarise(records, cmd, time.time() - started)
    if out is not None:
        _dump(out / "summary.json", summary)
    print(f"\n{summary['runs']} run(s): " + ", ".join(f"{n} {v}" for v, n in
                                                      sorted(summary["verdicts"].items())))
    for layer, counts in summary["layers"].items():
        if counts:
            print(f"  {layer}: " + ", ".join(f"{n} {v}" for v, n in sorted(counts.items())))
    if out is not None:
        print(f"  written to {out}")
    if summary["verdicts"].get("not_replayable"):
        return 2
    return 1 if summary["verdicts"].get("differs") else 0


if __name__ == "__main__":
    sys.exit(main())
