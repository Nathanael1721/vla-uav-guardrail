"""Export flown episodes as the grant's step and episode records (JSONL, Parquet).

    python tools/export_episodes.py demo/out/pas_ap_20261012_033329 --out demo/out/export
    python tools/export_episodes.py bundles/retarget_fixed.replay.tar.gz --out x/
    python tools/export_episodes.py demo/out/sweep/episode-* --out x/ --format jsonl

WHAT THE GRANT ASKS FOR

Stress Testing, "Data record schema": every step is recorded with
`observation` (rosbag2 image + state topics), `policy_state` (the active rules,
carrying `policy_hash` + `generation`), `action.planner_output` (the VLA's raw
4-D output), `action.post_repair` (the Shield's emitted action),
`action.executed` (what MAVROS actually published) and `outcome` (success |
fail | RTL_triggered | Land_triggered), with two auto-labels derived from the
Shield's audit log: `violation_type` in {fence, alt, schedule, envelope,
distance, ...} and `risk_level` in {P0, P1, P2}, "copied from the rule that
triggered". "Two export formats - episode-based (one row per mission, full
trajectory) and step-based (one row per tick) - produced by exporters that
consume the same episode bundle." Outputs table: "Episode exporters - Python
CLI (rosbag2 -> JSONL / Parquet)". Audit card WP4-09 recorded that none
existed.

INPUTS (each argument is one episode)

  * a replay bundle (`*.replay.tar.gz`, guardrail/replay.py): opened with
    `read_replay`, which checks every member's digest and the signature
    before anything is read; `adapter_log.jsonl` and the bag's files come out
    of the same verified archive (their digests are checked against the
    index again here);
  * a rail's run folder: `flight_log.jsonl`, the audit log
    (`shield_audit.jsonl` since 2026-11-02, `audit.jsonl` in a run stored
    before then, or the per-episode `shield_audit_<id>.jsonl` /
    `audit_<id>.jsonl`; guardrail.audit.audit_files), `events.jsonl`,
    `adapter_log.jsonl`, `metrics.json`, `kpi.json`, `manifest.json`,
    `policy_g<N>.json`, `bag/`;
  * a headless bundle (experiments/sweep_scenarios.py, the gRPC harness):
    `manifest.json`, `kpi.json` (the table nested under `kpi`, with the
    episode's `rule_priorities`), `harness_events.jsonl`,
    `manifest_extras.json`, and `flight_log.jsonl` when it was kept. Without
    the log only the episode record is written, and it says so;
    tools/rerun_episode.py re-flies the episode and writes the log.

WHAT EACH FIELD IS, WRITER BY WRITER (the part a reader must not assume)

  observation        a reference, never the image: the flight-log line and
                     the logged state (x, y, up, yaw); with a rosbag, the bag
                     file, its topics and the tick's wall time on the bag's
                     clock (`t_ns`: the mission_start event's wall clock plus
                     the row's `t`, else the episode start's; `clock` says
                     which) - and, read with `rosbags` (or, for the times
                     alone, the `mcap` reader; neither needs ROS), the log
                     time of the nearest message per topic and the pose
                     MAVROS reported nearest the tick (`state`,
                     /mavros/local_position/pose). Without either reader that
                     lookup prints SKIP and is left out.
  policy_state       the row's own `policy_hash` / `generation` (every rail
                     since 2026-10-06), else the audit record joined to the
                     tick (below), else (headless) the generation in force
                     at the tick, from the episode's accepted hot-applies in
                     harness_events.jsonl.
  planner_output     the row's `raw` (the joined audit record's world-frame
                     `raw_action_world` when the row has none).
  post_repair        the Shield's emitted action. On the rails that is the
                     row's `emitted`. In a HEADLESS row `emitted` is what
                     FLEW (sweep_scenarios: the raw action with the Shield
                     off, the stop in Brake, the autopilot's action in
                     LOITER / RTL / LAND), so post_repair is that only on a
                     tick the Shield flew a repair through; elsewhere it is
                     None and `post_repair_source` says why.
  executed           with a rosbag2 of the episode (the ROS 2 rail), what
                     MAVROS was sent: the setpoint on /mavros/setpoint_raw/
                     local logged in the tick's window, in the world frame
                     (`executed_from` "bag"; "the rosbag2" below), None on a
                     tick nothing was sent on. Otherwise the FALLBACK, from
                     the flight log, said on every step (`executed_from`
                     "flight_log", `executed_fallback` the reason: no bag,
                     `rosbags` not installed, the join refused, a headless
                     episode): `flown_action` (the pymavlink rail); else the
                     rail's verdict of what it streamed: `setpoint` "pass" ->
                     the emitted action, "brake" -> zero, "none" / `flown`
                     false -> None (the autopilot held the aircraft); else
                     the headless `emitted` (what flew). `executed_source`
                     names the case. The MAVLink adapter's own log
                     (`adapter_log.jsonl`) carries no tick and a clock the
                     flight log does not share, so it is attached to the
                     episode record, not joined per step.
  outcome            the episode's (the run's stored KPI table, else
                     guardrail.kpi.compute on the log), on every step.
  violation_type     per violation, from this project's category (a flight
                     log row's `category`, an audit record's `rule_category`):
                     geofence and corridor -> fence, altitude -> alt,
                     kinematic -> envelope, clearance and standoff ->
                     distance, contract -> contract (an extension: a
                     non-finite command). The grant's category `time-window`
                     (audit record version 2 on: a rule whose window edge
                     falls in the lookahead) is `schedule`; a violation with
                     only the grant's category maps geometric -> fence.
                     Before record version 2 `schedule` never occurs: a rule
                     out of its window raises nothing. A row's violation,
                     which carries neither `priority` nor the grant's
                     category, takes both from the joined audit record's
                     violation of the same rule. Each step's violation
                     carries both categories: `category` the grant's (null
                     when no audit record lent it), `rule_category` ours.
  risk_level         the highest priority among the tick's violations (the
                     violation's own `priority` when its audit record
                     carries it,
                     else the rule's from the policy generations, the
                     episode's recorded `rule_priorities`, or the policy the
                     manifest's hash resolves to); an unresolved rule counts
                     as P0, as everywhere in guardrail/kpi.py, and the step
                     says `risk_level_defaulted`.
  stop_advice        (an extension, 2026-10-20) on a tick whose stop the
                     ArduPilot rail flew instead of the pilot's action: the
                     stop's decision (emitted action, rule ids, repairs) from
                     its `decision_kind: "stop_advised"` audit record, else
                     from the row's `stop_advice`; None elsewhere. On such a
                     tick `executed` is that stop and `post_repair` the
                     repair of the planner output, which did not fly (its
                     source says so).

THE AUDIT LOG, AND WHICH OF ITS RECORDS BELONG TO WHICH TICK

Every shape guardrail/audit.py has stored is read: each record is first put
in the version-4 shape by guardrail.audit.upgrade_record, so the readers below
read one set of names. Before record version 2 a record has `tick` only;
version 2 (2026-10-12, policies/audit_record.schema.json) adds
`monitor_tick`, `episode_id`, a violation's `priority` (the rule's, copied
into the record) and the grant's category (whose "time-window", a rule at
its window edge, is labelled `schedule`). Version 4 (2026-11-02) puts the
grant's content under the grant's names: the world-frame actions every
reader here joins on are `raw_action_world` / `emitted_action_world` (they
were `raw_action` / `emitted_action` until then, and the upgrade moves them
there); the body-frame `raw_action` (vx forward) is never read as a world
velocity. The repairs made are `repair_results` (the old `repair_attempts`),
the grant's category is a violation's `category` and ours its
`rule_category`.

A record is joined to a row only when it is shown to be that row's. Until
`rotate_stale_episode` existed (audit card WP4-20) a reused run folder
APPENDED flights to one audit.jsonl (demo/out/ros2_shield_on holds twelve),
and the ROS 2 rail counts `monitor_tick` from 0 where its log counts `tick`
from 1. So the audit is split into flights (the tick restarts, the
`episode_id` changes, or the clock jumps further than the ticks account
for), and the flight and tick offset (0, +1 or -1) whose records carry the
raw actions of the log's rows is joined (`audit_blocks`, `join_audit`); at
least half of the comparable records must match (the AirSim follow loop
audits a pre-scaling command on some ticks, about 70 % match there). When no
flight matches, nothing is joined and the episode record says why
(`audit_join`): a wrong join would label a clean flight with another
flight's rules.

A TICK CAN HAVE TWO RECORDS (since 2026-10-20). On a tick whose stop the
ArduPilot rail flew instead of the pilot's action (the hold that stops
clear, docs/DESIGN-own-velocity-forecast.md), it writes the tick's filter()
record and then, at the SAME tick, the stop's record, tagged
`decision_kind: "stop_advised"` (guardrail/audit.py AuditRecord; every
other record has no `decision_kind`). Such a record is its tick's second
decision, never the start of another flight: `audit_blocks` keeps it in the
flight of the record before it when that one is the same tick's untagged
record. It is never joined as the tick's record or compared with the row
(its raw action is the zero stop, not the pilot's), and it rides on the
step as `stop_advice` instead. Before this, every stop-advised tick
started a new "flight" and the join kept the best single fragment: the hold
rail run of tests/test_own_velocity_forecast.py split into 22 "flights"
with 40 of 60 rows joined, its FSM-brake run into 61 with 2 of 60 (the
2026-10-20 re-review) - the only trace a lower `joined`.

OUTPUT

`<out>/steps.jsonl` (one row per tick) and `<out>/episodes.jsonl` (one row
per episode, the full trajectory as columns), strict JSON (a non-finite
number becomes the string "NaN" / "Infinity", never a bare token). With
`pyarrow` importable (optional: `pip install .[export]`), also
`steps.parquet` and `episodes.parquet`: one typed column per field, nested
fields and mixed columns as JSON text named in the file's metadata, each
file read back and compared with the JSONL records (every row, every key
of every record as a column, every value) before it is kept
(`read_parquet` returns those records; see "Parquet" below). Without
pyarrow the tool prints SKIP, says how to install it, removes a Parquet
file an earlier export left in the folder, and writes JSONL only
(`--format parquet` alone then writes nothing and exits 3). Exit 0 when
every input exported, 2 when one could not be read or a Parquet file did not
read back as its JSONL.
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import json
import math
import sys
import tarfile
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail import kpi as K                                     # noqa: E402
from guardrail.audit import audit_files, upgrade_record            # noqa: E402

# 2 since 2026-11-02: a step's violation carries the grant's `category` and
# ours as `rule_category` (until then `category` was ours, alone).
SCHEMA = "vlaguard.episode_export/2"
ACTION_KEYS = ("vx", "vy", "vz_up", "yaw_rate")
ZERO = {k: 0.0 for k in ACTION_KEYS}

# This project's violation categories -> the grant's violation_type labels.
VIOLATION_TYPE = {"geofence": "fence", "corridor": "fence", "altitude": "alt",
                  "kinematic": "envelope", "clearance": "distance",
                  "standoff": "distance", "contract": "contract"}
# ... and the grant's own three categories, where that is all a record says.
GRANT_CATEGORY_TYPE = {"geometric": "fence", "envelope": "envelope",
                       "time-window": "schedule"}


class ExportError(Exception):
    """An input that cannot be read as an episode."""


# --------------------------------------------------------------------------- #
# small readers
# --------------------------------------------------------------------------- #

def _first(d: dict, *keys, default=None):
    """The first of `keys` present in `d` (old and new audit spellings)."""
    for k in keys:
        if isinstance(d, dict) and k in d and d[k] is not None:
            return d[k]
    return default


def _jsonl(data: bytes | str | None) -> list[dict]:
    if not data:
        return []
    text = data.decode("utf-8") if isinstance(data, bytes) else data
    out = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(rec, dict):
            out.append(rec)
    return out


def _json(p: Path) -> dict:
    try:
        return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}
    except (OSError, json.JSONDecodeError):
        return {}


def json_safe(o):
    """Non-finite floats as the strings "NaN" / "Infinity" / "-Infinity":
    strict JSON has no token for them, and a raw action can carry one."""
    if isinstance(o, float) and not math.isfinite(o):
        return "NaN" if o != o else ("Infinity" if o > 0 else "-Infinity")
    if isinstance(o, dict):
        return {k: json_safe(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [json_safe(v) for v in o]
    return o


def action(a: Any) -> dict | None:
    """A world-frame Action4D dict (vx North, vy East, vz_up, yaw_rate), as
    every flight log writes it and an audit record's `raw_action_world` /
    `emitted_action_world` carry it (record version 4; `raw_action` /
    `emitted_action` before). The grant's body-frame `raw_action` (vx
    forward, `vz`) is not read: its vx and vy are not these."""
    if not isinstance(a, dict):
        return None
    out = {k: a.get(k) for k in ACTION_KEYS}
    return out if any(v is not None for v in out.values()) else None


def _same_action(a: dict | None, b: dict | None, tol: float = 1e-9) -> bool:
    """Two actions equal channel by channel (NaN equals NaN: the non-finite
    scenarios log one on purpose)."""
    if a is None or b is None:
        return False
    for k in ACTION_KEYS:
        x, y = a.get(k), b.get(k)
        try:
            fx, fy = float(x), float(y)
        except (TypeError, ValueError):
            if x != y:
                return False
            continue
        if math.isnan(fx) and math.isnan(fy):
            continue
        if not abs(fx - fy) <= tol:
            return False
    return True


def violation_type(v: dict) -> str:
    """The grant's violation_type label of one violation, in the form
    `_violation` gives it: `rule_category` ours, `grant_category` the
    grant's (module docstring)."""
    if v.get("grant_category") == "time-window":
        # Audit record version 2 (2026-10-12) on marks a rule whose window edge
        # falls inside the lookahead as the grant's time-window category.
        return "schedule"
    cat = v.get("rule_category")
    if cat in VIOLATION_TYPE:
        return VIOLATION_TYPE[cat]
    g = v.get("grant_category") or cat
    return GRANT_CATEGORY_TYPE.get(g, str(g) if g else "unknown")


def _violation(v: dict, *, audit: bool) -> dict:
    """One violation in the form the labels read: an audit record's (version
    4: `category` the grant's, `rule_category` ours) or a flight-log row's
    (`category` ours; no grant category)."""
    out = {"rule_id": v.get("rule_id")}
    if audit:
        out.update(rule_category=v.get("rule_category"), grant_category=v.get("category"))
    else:
        out["rule_category"] = v.get("category")
    if v.get("priority") is not None:
        out["priority"] = v.get("priority")
    return {k: x for k, x in out.items() if x is not None}


def _audit_v4(a: dict) -> dict:
    """An audit record of any stored version in the version-4 shape
    (guardrail.audit.upgrade_record), so every reader here reads one set of
    names; one the upgrade cannot read is kept as it is."""
    try:
        return upgrade_record(a)
    except Exception:                                            # noqa: BLE001
        return a


# --------------------------------------------------------------------------- #
# one episode, read from any of the three inputs
# --------------------------------------------------------------------------- #

@dataclass
class Episode:
    key: str
    source: str
    kind: str                                  # replay | run | headless
    rows: list[dict] = field(default_factory=list)
    audit: list[dict] = field(default_factory=list)
    events: list[dict] = field(default_factory=list)
    adapter: list[dict] = field(default_factory=list)
    manifest: dict = field(default_factory=dict)
    extras: dict = field(default_factory=dict)
    kpi_doc: dict = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)
    priorities: dict = field(default_factory=dict)
    priority_source: str = "none"
    generations: list[dict] = field(default_factory=list)
    shield: str | None = None
    bag: dict | None = None
    bag_messages: dict | None = None           # topic -> sorted log times (ns)
    bag_data: "BagData | None" = None          # read with rosbags (read_bag)
    bag_error: str | None = None               # why the bag was not read
    executed: dict | None = None               # bag_execution's account, once built
    signature: dict | None = None
    notes: list[str] = field(default_factory=list)
    audit_join: dict | None = None              # join_audit's account, once built


def _rel(p: Path) -> str:
    try:
        return p.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return p.as_posix()


def _priorities_from_generations(pols: dict) -> dict[str, str]:
    out: dict[str, str] = {}
    for g in sorted(pols):
        out.update(K.rule_priorities(pols[g]))
    return out


def _arm_of(metrics: dict, events: list[dict], doc: dict) -> str | None:
    for v in (metrics.get("shield"), (doc.get("metrics") or {}).get("shield"),
              next((e.get("shield") for e in events
                    if e.get("type") == "episode_start" or e.get("kind") == "episode_start"),
                   None)):
        if v in ("on", "off"):
            return v
        if v is True:
            return "on"
        if v is False:
            return "off"
    return None


def _bag_meta_text(text: str) -> dict:
    import yaml
    try:
        doc = yaml.safe_load(text) or {}
    except yaml.YAMLError:
        return {}
    info = doc.get("rosbag2_bagfile_information") or {}
    topics = {}
    for t in info.get("topics_with_message_count") or []:
        name = ((t.get("topic_metadata") or {}).get("name"))
        if name:
            topics[name] = t.get("message_count")
    start = (info.get("starting_time") or {}).get("nanoseconds_since_epoch")
    dur = (info.get("duration") or {}).get("nanoseconds")
    return {"storage": info.get("storage_identifier"), "topics": topics,
            "starting_time_ns": start, "duration_ns": dur,
            "files": [f.get("path") for f in info.get("files") or [] if f.get("path")]
            or info.get("relative_file_paths") or []}


def mcap_message_times(paths: list[Path], topics: list[str] | None = None
                       ) -> dict[str, list[int]] | None:
    """topic -> sorted log times (ns) of every message in the MCAP files, read
    with the `mcap` package (pure Python, no ROS). None when it is not
    installed (the caller prints SKIP)."""
    try:
        from mcap.reader import make_reader
    except ImportError:
        return None
    out: dict[str, list[int]] = {}
    for p in paths:
        with open(p, "rb") as fh:
            reader = make_reader(fh)
            for _schema, channel, message in reader.iter_messages(topics=topics):
                out.setdefault(channel.topic, []).append(int(message.log_time))
    for v in out.values():
        v.sort()
    return out


def _episode_start_ns(events: list[dict]) -> int | None:
    for e in events:
        if e.get("kind") == "episode_start" and isinstance(e.get("wall_ns"), int):
            return e["wall_ns"]
    return None


# --------------------------------------------------------------------------- #
# the rosbag2: what MAVROS was sent (2026-11-02)
# --------------------------------------------------------------------------- #
#
# The grant's `action.executed` is "what MAVROS actually published". On the
# ROS 2 rail that is the mavros_msgs/PositionTarget the MAVLink adapter
# publishes on /mavros/setpoint_raw/local, which MAVROS sends to the
# autopilot as SET_POSITION_TARGET_LOCAL_NED; sitl/run_ros2_demo.sh records it
# in the run's bag. It is read with `rosbags` (pure Python, no ROS; optional,
# like pyarrow): each message is placed on the flight-log tick whose window
# holds its log time - from the tick's wall time (the mission_start event's
# wall clock plus the row's `t`) less BAG_EARLY_S, up to the next tick's -
# and converted back to the world frame (the adapter fills the message in ENU
# for MAVROS: x east, y north, z up, yaw rate counter-clockwise;
# sitl/mavlink_adapter_node.ned_to_mavros_enu).
#
# The join is trusted only when the bag agrees with the flight log: at least
# BAG_AGREE_SHARE of the ticks where both say something was sent must carry
# the same command within the adapter's heading slack (it rotates with the
# pose it last read, the Shield with the pose of its tick). Otherwise - a
# clock anchor off by more than a tick, another flight's bag - nothing is
# taken from the bag and the episode says why. Where it is trusted, a tick
# with no message in its window executed nothing (the adapter dropped it, or
# nothing was published), whatever the log's verdict, and a tick whose
# message differs from the log keeps the bag's: what MAVROS was sent. The
# flight log's verdict rides beside it (`executed_flight_log`).
#
# Without a bag, without `rosbags`, or with the join refused, `executed` is
# the flight log's verdict as before, and every step says so:
# `executed_from` "flight_log" and `executed_fallback` the reason.

SETPOINT_TOPIC = "/mavros/setpoint_raw/local"
POSE_TOPIC = "/mavros/local_position/pose"
ROSBAGS_INSTALL_HINT = ('pip install "rosbags>=0.10" (pure Python, no ROS; or run the '
                        "exporter in WSL with it installed); it is optional, the "
                        "flight-log fallback never needs it")
# mavros_msgs/msg/PositionTarget (MAVROS 2), registered only for a bag that
# does not carry the message's definition (a bag rosbag2 recorded on Jazzy
# carries it, and that one is used).
POSITION_TARGET_TYPE = "mavros_msgs/msg/PositionTarget"
POSITION_TARGET_MSG = """\
std_msgs/Header header
uint8 coordinate_frame
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_LOCAL_OFFSET_NED = 7
uint8 FRAME_BODY_NED = 8
uint8 FRAME_BODY_OFFSET_NED = 9
uint16 type_mask
uint16 IGNORE_PX = 1
uint16 IGNORE_PY = 2
uint16 IGNORE_PZ = 4
uint16 IGNORE_VX = 8
uint16 IGNORE_VY = 16
uint16 IGNORE_VZ = 32
uint16 IGNORE_AFX = 64
uint16 IGNORE_AFY = 128
uint16 IGNORE_AFZ = 256
uint16 FORCE = 512
uint16 IGNORE_YAW = 1024
uint16 IGNORE_YAW_RATE = 2048
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration_or_force
float32 yaw
float32 yaw_rate
"""
FRAME_LOCAL_NED = 1
IGNORE_VELOCITY = 8 | 16 | 32
IGNORE_YAW_RATE = 2048
BAG_EARLY_S = 0.02          # a message logged this much before its tick's time is still its
BAG_AGREE_SHARE = 0.5
BAG_POSE_WITHIN_S = 0.5     # the observation's pose: the nearest message within this


@dataclass
class BagData:
    """What the exporter reads from a rosbag2 with `rosbags`."""
    reader: str
    times: dict = field(default_factory=dict)        # topic -> sorted log times (ns)
    setpoints: list = field(default_factory=list)    # (log ns, PositionTarget fields)
    poses: list = field(default_factory=list)        # (log ns, {x, y, up, yaw_deg})


def have_rosbags() -> bool:
    try:
        import rosbags.highlevel                                 # noqa: F401
    except ImportError:
        return False
    return True


def _rosbags_version() -> str:
    try:
        from importlib.metadata import version
        return f"rosbags {version('rosbags')}"
    except Exception:                                            # noqa: BLE001
        return "rosbags"


def heading_deg_from_enu_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Heading clockwise from North, from a MAVROS ENU orientation: the
    formula of sitl/mavlink_adapter_node.heading_deg_from_enu_quaternion
    (tests/test_wp4outputs_exporters.py keeps the two equal)."""
    yaw_enu = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    h = (90.0 - math.degrees(yaw_enu)) % 360.0
    return 0.0 if h >= 360.0 else h


def _deserialize(reader, raw: bytes, msgtype: str):
    try:
        return reader.deserialize(raw, msgtype)
    except Exception:                                            # noqa: BLE001
        store = getattr(reader, "typestore", None)
        if msgtype != POSITION_TARGET_TYPE or store is None:
            raise
        from rosbags.typesys import get_types_from_msg
        store.register(get_types_from_msg(POSITION_TARGET_MSG, POSITION_TARGET_TYPE))
        return reader.deserialize(raw, msgtype)


def read_bag(bag_dir: Path) -> BagData | None:
    """Every topic's log times, the setpoints sent to MAVROS and the poses it
    reported, read with `rosbags`; None when it is not installed. A bag it
    cannot read raises."""
    try:
        from rosbags.highlevel import AnyReader
    except ImportError:
        return None
    kw: dict = {}
    try:
        from rosbags.typesys import Stores, get_typestore
        kw["default_typestore"] = get_typestore(getattr(Stores, "ROS2_JAZZY", None)
                                                or Stores.LATEST)
    except ImportError:
        pass
    data = BagData(reader=_rosbags_version())
    try:
        reader_cm = AnyReader([Path(bag_dir)], **kw)
    except TypeError:                     # rosbags < 0.10: no default_typestore
        reader_cm = AnyReader([Path(bag_dir)])
    with reader_cm as reader:
        for conn, ts, raw in reader.messages():
            data.times.setdefault(conn.topic, []).append(int(ts))
            if conn.topic == SETPOINT_TOPIC:
                m = _deserialize(reader, raw, conn.msgtype)
                data.setpoints.append((int(ts), {
                    "coordinate_frame": int(m.coordinate_frame), "type_mask": int(m.type_mask),
                    "velocity": [float(m.velocity.x), float(m.velocity.y), float(m.velocity.z)],
                    "yaw_rate": float(m.yaw_rate),
                    "stamp_ns": int(m.header.stamp.sec) * 1_000_000_000
                    + int(m.header.stamp.nanosec)}))
            elif conn.topic == POSE_TOPIC:
                m = _deserialize(reader, raw, conn.msgtype)
                p, q = m.pose.position, m.pose.orientation
                # MAVROS ENU -> this project's x North, y East, up, as the
                # Shield node reads the same topic.
                data.poses.append((int(ts), {
                    "x": float(p.y), "y": float(p.x), "up": float(p.z),
                    "yaw_deg": heading_deg_from_enu_quaternion(q.x, q.y, q.z, q.w)}))
    for v in data.times.values():
        v.sort()
    data.setpoints.sort(key=lambda x: x[0])
    data.poses.sort(key=lambda x: x[0])
    return data


def _read_bag_into(ep: "Episode", bag_dir: Path) -> None:
    """ep.bag_data (and the topics' message times) from `bag_dir`, or why not."""
    try:
        ep.bag_data = read_bag(bag_dir)
    except Exception as e:                                       # noqa: BLE001
        ep.bag_data = None
        ep.bag_error = f"the bag could not be read with rosbags: {type(e).__name__}: {e}"
        return
    if ep.bag_data is None:
        ep.bag_error = f"the bag was not read: `rosbags` is not installed ({ROSBAGS_INSTALL_HINT})"
    else:
        ep.bag_messages = ep.bag_data.times


def setpoint_world(f: dict) -> tuple[dict | None, str | None]:
    """A PositionTarget's command in this project's world frame (vx North, vy
    East, vz_up, yaw_rate clockwise rad/s), undoing the adapter's ENU packing;
    (None, why) when it is not a FRAME_LOCAL_NED velocity setpoint."""
    if f.get("coordinate_frame") != FRAME_LOCAL_NED:
        return None, (f"coordinate_frame {f.get('coordinate_frame')}, not "
                      f"FRAME_LOCAL_NED (1): not converted")
    if int(f.get("type_mask") or 0) & IGNORE_VELOCITY:
        return None, (f"type_mask {f.get('type_mask')} ignores the velocity: not a "
                      f"velocity setpoint")
    east, north, up = f["velocity"]
    yaw = None if int(f["type_mask"]) & IGNORE_YAW_RATE else -float(f["yaw_rate"])
    return {"vx": north, "vy": east, "vz_up": up, "yaw_rate": yaw}, None


def _iso_ns(s: Any) -> int | None:
    from datetime import datetime, timezone
    if not isinstance(s, str):
        return None
    try:
        dt = datetime.fromisoformat(s.replace("Z", "+00:00"))
    except ValueError:
        return None
    if dt.tzinfo is None:
        return None
    d = dt - datetime(1970, 1, 1, tzinfo=timezone.utc)
    return (d.days * 86400 + d.seconds) * 1_000_000_000 + d.microseconds * 1000


def mission_clock(events: list[dict]) -> tuple[int | None, str | None]:
    """(wall ns at the flight log's t = 0, what it is). The ROS 2 rail's rows
    count `t` from its `mission_start` event (t 0.0), written with its wall
    clock (ms) after take-off; a rail without one counts from the episode
    start (`episode_start` wall_ns)."""
    for e in events:
        if e.get("kind") == "mission_start" and e.get("t") in (0, 0.0):
            ns = _iso_ns(e.get("wall"))
            if ns is not None:
                return ns, "mission_start event's wall clock (ms)"
    ns = _episode_start_ns(events)
    if ns is not None:
        return ns, "episode_start wall_ns"
    return None, None


def _agrees(a: dict | None, b: dict | None) -> bool:
    """Two world-frame commands the same within the adapter's heading slack:
    horizontal within 0.1 m/s plus 10 % of the speed, vertical within 0.05
    m/s, yaw rate within 0.01 rad/s (a float32 on the wire)."""
    if a is None or b is None:
        return False
    try:
        dvx = float(a["vx"]) - float(b["vx"])
        dvy = float(a["vy"]) - float(b["vy"])
        dvz = float(a["vz_up"]) - float(b["vz_up"])
        speed = math.hypot(float(b["vx"]), float(b["vy"]))
    except (TypeError, ValueError, KeyError):
        return False
    if not all(math.isfinite(x) for x in (dvx, dvy, dvz, speed)):
        return False
    ya, yb = a.get("yaw_rate"), b.get("yaw_rate")
    yaw_ok = (ya is None or yb is None
              or (isinstance(ya, (int, float)) and isinstance(yb, (int, float))
                  and abs(float(ya) - float(yb)) <= 0.01))
    return math.hypot(dvx, dvy) <= 0.1 + 0.1 * speed and abs(dvz) <= 0.05 and yaw_ok


def join_bag_setpoints(rows: list[dict], setpoints: list, anchor_ns: int) -> list[dict | None]:
    """Per row, the setpoint messages logged in its tick's window: from the
    tick's wall time (anchor + t) less BAG_EARLY_S to the next tick's less
    the same (the last tick: one median tick period). None for a row with no
    numeric `t`; n 0 for a tick nothing was logged in; else the first
    message's fields, its log time, its lag behind the tick and how many."""
    from bisect import bisect_left
    ns = []
    for r in rows:
        t = r.get("t")
        ok = isinstance(t, (int, float)) and not isinstance(t, bool) and math.isfinite(float(t))
        ns.append(anchor_ns + int(round(float(t) * 1e9)) if ok else None)
    known = [x for x in ns if x is not None]
    gaps = sorted(b - a for a, b in zip(known, known[1:]) if b > a)
    period = gaps[len(gaps) // 2] if gaps else 100_000_000
    early = int(BAG_EARLY_S * 1e9)
    log = [ts for ts, _ in setpoints]
    out: list[dict | None] = []
    for i, x in enumerate(ns):
        if x is None:
            out.append(None)
            continue
        nxt = next((y for y in ns[i + 1:] if y is not None and y > x), x + period)
        a, b = bisect_left(log, x - early), bisect_left(log, nxt - early)
        if a == b:
            out.append({"n": 0, "tick_ns": x})
            continue
        ts, f = setpoints[a]
        out.append({"n": b - a, "tick_ns": x, "log_time_ns": ts,
                    "lag_ms": round((ts - x) / 1e6, 3), "fields": f})
    return out


def bag_execution(ep: "Episode", flight_log: list[dict]) -> tuple[list | None, dict]:
    """(per row: the bag's setpoint join, or None when the bag is not used;
    the account the episode record carries). `flight_log` is each row's
    `_actions`, the flight log's verdict the join is checked against."""
    acct: dict[str, Any] = {"topic": SETPOINT_TOPIC}

    def fallback(why: str):
        acct.update({"from": "flight_log", "why": why})
        return None, acct

    if ep.kind == "headless":
        return fallback("a headless episode: no ROS 2 graph and no MAVROS; executed is "
                        "what the kinematic runner flew, from the flight log")
    if ep.bag is None:
        return fallback("no rosbag2 was recorded for this episode")
    if ep.bag_data is None:
        return fallback(ep.bag_error or "the bag was not read")
    acct["reader"] = ep.bag_data.reader
    sp = ep.bag_data.setpoints
    acct["messages"] = len(sp)
    if not sp:
        return fallback(f"the bag holds no {SETPOINT_TOPIC} message")
    anchor, clock = mission_clock(ep.events)
    acct["clock"] = clock
    if anchor is None:
        return fallback("events.jsonl gives no wall clock to place the log's ticks on the bag's")
    joined = join_bag_setpoints(ep.rows, sp, anchor)
    comparable = agree = 0
    for j, fl in zip(joined, flight_log):
        if j and j.get("n") and fl.get("executed") is not None:
            w, _ = setpoint_world(j["fields"])
            if w is not None:
                comparable += 1
                agree += _agrees(w, fl["executed"])
    acct.update(comparable=comparable, agree=agree)
    if not comparable or agree < BAG_AGREE_SHARE * comparable:
        return fallback(
            f"the bag's setpoints do not line up with the flight log's ticks ({agree} of "
            f"{comparable} comparable ticks agree, {BAG_AGREE_SHARE:.0%} needed): a clock "
            f"anchor off by more than a tick, or another flight's bag; none is used")
    used = [j for j in joined if j is not None]
    acct.update({"from": "bag", "why": None,
                 "ticks_with_message": sum(1 for j in used if j["n"]),
                 "ticks_without_message": sum(1 for j in used if not j["n"]),
                 "ticks_with_several": sum(1 for j in used if j["n"] > 1),
                 "messages_on_no_tick": len(sp) - sum(j["n"] for j in used),
                 "disagree_with_flight_log": comparable - agree})
    return joined, acct


def _executed_from_bag(act: dict, j: dict | None, why_not: str | None) -> dict:
    """A step's action block with `executed` from the bag's join `j`, the
    flight log's verdict beside it; or the fallback, labelled."""
    if j is None:
        act["executed_from"] = "flight_log"
        act["executed_fallback"] = why_not or "this row has no t to place on the bag"
        return act
    fl_exe, fl_src = act["executed"], act["executed_source"]
    if j["n"]:
        w, why = setpoint_world(j["fields"])
        act["executed"] = w
        act["executed_source"] = f"bag:{SETPOINT_TOPIC}" + (f" ({why})" if why else "")
        act["executed_bag"] = {"topic": SETPOINT_TOPIC, "log_time_ns": j["log_time_ns"],
                               "lag_ms": j["lag_ms"], "messages_in_window": j["n"],
                               "coordinate_frame": j["fields"].get("coordinate_frame"),
                               "type_mask": j["fields"].get("type_mask")}
    else:
        act["executed"] = None
        act["executed_source"] = (f"bag:{SETPOINT_TOPIC}: no message in this tick's window "
                                  f"(nothing was sent to MAVROS)")
        act["executed_bag"] = {"topic": SETPOINT_TOPIC, "log_time_ns": None, "lag_ms": None,
                               "messages_in_window": 0}
    act["executed_from"] = "bag"
    act["executed_fallback"] = None
    act["executed_flight_log"] = fl_exe
    act["executed_flight_log_source"] = fl_src
    return act


def load_replay(path: Path, trust_store=None) -> Episode:
    from guardrail.replay import BAG_DIR, EPISODE, read_replay
    try:
        got = read_replay(path, trust_store)
    except (ValueError, KeyError, tarfile.TarError, OSError) as e:
        raise ExportError(f"{path}: {e}") from None
    declared = got["index"].get("members") or {}
    extra: dict[str, bytes] = {}
    with tarfile.open(path, "r:gz") as tar:
        for name in declared:
            if name == EPISODE + "adapter_log.jsonl" or name.startswith(EPISODE + BAG_DIR + "/"):
                data = tar.extractfile(name).read()
                if hashlib.sha256(data).hexdigest() != declared[name]:
                    raise ExportError(f"{path}: {name} does not match its digest")
                extra[name] = data
    ep = Episode(key=path.name.replace(".replay.tar.gz", ""), source=_rel(path),
                 kind="replay", rows=got["rows"], audit=[_audit_v4(a) for a in got["audit"]],
                 events=got["events"], manifest=got["manifest"] or {},
                 kpi_doc=got["kpi"] or {}, metrics=got["metrics"] or {},
                 signature=got["signature"])
    ep.adapter = _jsonl(extra.get(EPISODE + "adapter_log.jsonl"))
    pols = dict(got["generation_policies"]) or {0: got["policy"]}
    ep.priorities = _priorities_from_generations(pols)
    ep.priorities.update({k: v for k, v in K.rule_priorities(got["policy"]).items()
                          if k not in ep.priorities})
    ep.priority_source = "replay bundle's policy generations"
    ep.generations = [{"generation": g, "policy_hash": p.policy_hash}
                      for g, p in sorted(pols.items())]
    bag_files = {n[len(EPISODE + BAG_DIR) + 1:]: b for n, b in extra.items()
                 if n.startswith(EPISODE + BAG_DIR + "/")}
    if bag_files:
        meta = _bag_meta_text(bag_files.get("metadata.yaml", b"").decode("utf-8", "replace"))
        ep.bag = dict(meta, where=f"{ep.source}!{EPISODE}{BAG_DIR}/")
        with tempfile.TemporaryDirectory(prefix="export-bag-") as td:
            # A rosbag2 folder is flat: each file by its own name only, so
            # no member name can place a file outside the folder.
            for n, b in bag_files.items():
                (Path(td) / Path(n).name).write_bytes(b)
            _read_bag_into(ep, Path(td))
            mcaps = sorted(Path(td).glob("*.mcap"))
            if ep.bag_messages is None and mcaps:
                ep.bag_messages = mcap_message_times(mcaps)
    ep.shield = _arm_of(ep.metrics, ep.events, {})
    return ep


def _audit_files(run: Path) -> list[Path]:
    """shield_audit.jsonl, else a stored run's audit.jsonl, else the
    per-episode files (guardrail.audit.audit_files)."""
    return audit_files(run)


def _resolve_policy_by_hash(manifest: dict):
    try:
        from guardrail.bundle import policy_candidates
        from guardrail.manifest import resolve_run_policy
        pol, _form, label = resolve_run_policy(manifest, policy_candidates())
        return pol, label
    except Exception:                                            # noqa: BLE001
        return None, None


def load_dir(run: Path) -> Episode:
    if not run.is_dir():
        raise ExportError(f"{run}: not a folder")
    manifest = _json(run / "manifest.json")
    doc = _json(run / "kpi.json")
    headless = (run / "harness_events.jsonl").is_file() or (
        isinstance(doc.get("kpi"), dict) and bool(doc.get("template")))
    ep = Episode(key=run.name, source=_rel(run), kind="headless" if headless else "run",
                 manifest=manifest, extras=_json(run / "manifest_extras.json"),
                 kpi_doc=doc, metrics=_json(run / "metrics.json"))
    log = run / "flight_log.jsonl"
    if log.is_file():
        ep.rows = _jsonl(log.read_bytes())
    for a in _audit_files(run):
        ep.audit += [_audit_v4(x) for x in _jsonl(a.read_bytes())]
    ev = run / ("harness_events.jsonl" if headless else "events.jsonl")
    ep.events = _jsonl(ev.read_bytes()) if ev.is_file() else []
    ad = run / "adapter_log.jsonl"
    ep.adapter = _jsonl(ad.read_bytes()) if ad.is_file() else []
    if headless:
        ep.priorities = dict(doc.get("rule_priorities") or {})
        ep.priority_source = "the episode's recorded rule_priorities (kpi.json)"
        gens = ep.extras.get("generations")
        if not isinstance(gens, list):
            gens = [{"generation": e.get("generation"), "policy_hash": e.get("policy_hash"),
                     "t": 0.0 if e.get("type") == "episode_start" else e.get("t")}
                    for e in ep.events
                    if e.get("type") == "episode_start"
                    or (e.get("type") == "hot_apply" and not e.get("refused"))]
        ep.generations = gens
        if not ep.rows:
            ep.notes.append("no flight_log.jsonl in the bundle (flown without "
                            "--keep-logs / keep_flight_log): episode record only; "
                            "tools/rerun_episode.py re-flies it and writes the log")
    else:
        from guardrail.models import Policy, era_context
        pols = {}
        for f in sorted(run.glob("policy_g*.json")):
            try:
                # A run's generation file records no IR version: read as
                # written (scope absent in a run from before IR schema 4.0.0).
                pol = Policy.model_validate(_json(f), context=era_context(None))
                pols[int(pol.generation)] = pol
            except Exception:                                    # noqa: BLE001
                ep.notes.append(f"{f.name} does not load as a policy")
        if pols:
            ep.priorities = _priorities_from_generations(pols)
            ep.priority_source = "the run's policy_g<N>.json generations"
            ep.generations = [{"generation": g, "policy_hash": p.policy_hash}
                              for g, p in sorted(pols.items())]
        elif isinstance(doc.get("rule_priorities"), dict):
            ep.priorities = dict(doc["rule_priorities"])
            ep.priority_source = "kpi.json rule_priorities"
        else:
            pol, label = _resolve_policy_by_hash(manifest)
            if pol is not None:
                ep.priorities = K.rule_priorities(pol)
                ep.priority_source = f"the policy the manifest's hash resolves to ({label})"
        bag = run / "bag"
        if (bag / "metadata.yaml").is_file():
            ep.bag = dict(_bag_meta_text((bag / "metadata.yaml").read_text(encoding="utf-8")),
                          where=_rel(bag) + "/")
            _read_bag_into(ep, bag)
            mcaps = sorted(bag.glob("*.mcap"))
            if ep.bag_messages is None and mcaps:
                # Without rosbags, the `mcap` reader still gives the times.
                ep.bag_messages = mcap_message_times(mcaps)
    if not ep.priorities:
        ep.notes.append("no rule priorities resolved: every violated rule is labelled "
                        "P0 (risk_level_defaulted), as guardrail.kpi counts it")
    ep.shield = _arm_of(ep.metrics, ep.events, doc)
    return ep


def load_episode(path: str | Path, trust_store=None) -> Episode:
    p = Path(path)
    if p.is_file() and p.name.endswith(".tar.gz"):
        return load_replay(p, trust_store)
    return load_dir(p)


# --------------------------------------------------------------------------- #
# the records
# --------------------------------------------------------------------------- #

def episode_outcome(ep: Episode) -> tuple[str | None, str, bool | None]:
    """(outcome, where it came from, mission_success)."""
    table = ep.kpi_doc.get("kpi") if isinstance(ep.kpi_doc.get("kpi"), dict) else ep.kpi_doc
    oc = (table or {}).get("mission_outcome") or (table or {}).get("outcome")
    if oc in K.OUTCOMES:
        return oc, "the stored KPI table (kpi.json)", K.mission_success_with_goal(table)
    if ep.rows:
        m = dict(ep.metrics)
        if ep.shield and "shield" not in m:
            m["shield"] = ep.shield
        k = K.compute(ep.rows, ep.priorities, m)
        return k["outcome"], "guardrail.kpi.compute on the flight log", k["mission_success"]
    return None, "n/m: no KPI table and no flight log", None


def _generation_at(gens: list[dict], t: Any) -> dict | None:
    """The generation in force at simulated time t (headless timelines)."""
    best = None
    for g in gens:
        gt = g.get("t")
        if gt is None:
            continue
        if isinstance(t, (int, float)) and gt <= t + 1e-9:
            if best is None or gt >= best.get("t", -1):
                best = g
    return best


def _actions(row: dict, audit: dict | None, shield: str | None,
             advised: bool = False) -> dict:
    raw = action(row.get("raw")) or action((audit or {}).get("raw_action_world"))
    if "owner" in row:
        # experiments/sweep_scenarios.py: `emitted` is what FLEW.
        owner = row.get("owner")
        flown = action(row.get("emitted"))
        sp = row.get("setpoint", "pass")
        if owner == "shield" and sp != "brake":
            post, post_src = flown, "emitted (the Shield's action, flown)"
        elif owner == "shield":
            post, post_src = None, "not logged: the FSM braked, and the stop is what flew"
        elif owner == "pilot":
            post, post_src = None, "not logged: Shield off, the raw action flew"
        else:
            post, post_src = None, "not logged: the autopilot flew this tick"
        src = f"headless:{owner}" + (f":{row['autopilot']}" if row.get("autopilot") else "")
        return {"planner_output": raw, "post_repair": post, "executed": flown,
                "post_repair_source": post_src, "executed_source": src}
    post = action(row.get("emitted")) or action((audit or {}).get("emitted_action_world"))
    if "flown_action" in row:
        if row.get("flown") is False:
            exe, src = None, "not_flown: the rail streamed nothing"
        else:
            exe, src = action(row["flown_action"]), "flown_action"
    elif row.get("flown") is False or row.get("setpoint") == "none":
        exe, src = None, "not_flown: the autopilot held the aircraft"
    elif row.get("setpoint") == "brake":
        exe, src = dict(ZERO), "setpoint:brake (zero streamed)"
    elif shield == "off" and ("setpoint" in row or "flown" in row):
        exe, src = raw, "shield_off: the raw action streamed"
    elif "setpoint" in row or "flown" in row:
        exe, src = post, "setpoint:pass (the emitted action streamed)"
    else:
        exe, src = post, "inferred: the log records no flown action; the emitted one"
    # A stop-advised tick (the ArduPilot rail, 2026-10-20): `emitted` is the
    # Shield's repair of the planner output, which was NOT sent; the rail sent
    # the Shield's answer to a stop (`executed`, flown_action), whose decision
    # is the step's `stop_advice`. post_repair stays the planner output's
    # repair, so planner_output -> post_repair is one decision; the source
    # says the action that flew was another.
    post_src = ("emitted (the repair of the planner output; not flown: this tick "
                "flew the Shield's answer to a stop, see stop_advice)"
                if advised else "emitted")
    return {"planner_output": raw, "post_repair": post, "executed": exe,
            "post_repair_source": post_src, "executed_source": src}


def _nearest(times: list[int], x: int) -> int:
    from bisect import bisect_left
    k = bisect_left(times, x)
    return min(times[max(0, k - 1):k + 1], key=lambda y: abs(y - x))


def _observation(ep: Episode, i: int, row: dict, start_ns: int | None,
                 clock: str | None = None) -> dict:
    obs = {"ref": f"{ep.source}#flight_log.jsonl:L{i + 1}",
           "state": {k: row.get(k) for k in ("x", "y", "up", "yaw_deg") if k in row}}
    if ep.bag:
        b = {"where": ep.bag.get("where"), "topics": sorted(ep.bag.get("topics") or {})}
        t = row.get("t")
        if start_ns is not None and isinstance(t, (int, float)):
            b["t_ns"] = start_ns + int(round(float(t) * 1e9))
            b["clock"] = clock
            if ep.bag_messages:
                b["nearest_message_ns"] = {topic: _nearest(times, b["t_ns"])
                                           for topic, times in ep.bag_messages.items()
                                           if times}
            poses = ep.bag_data.poses if ep.bag_data is not None else []
            if poses:
                k = _nearest([p[0] for p in poses], b["t_ns"])
                if abs(k - b["t_ns"]) <= BAG_POSE_WITHIN_S * 1e9:
                    pose = next(p for ts, p in poses if ts == k)
                    b["state"] = {**pose, "topic": POSE_TOPIC, "log_time_ns": k}
        obs["bag"] = b
    return obs


# Splitting an appended audit log into flights (module docstring): a jump in
# the record clock beyond AUDIT_GAP_S plus AUDIT_TICK_S per tick advanced
# starts another flight (a 10 Hz loop advances 0.1 s per tick, so the slack is
# fivefold). A flight is joined when at least AUDIT_MATCH_SHARE of its
# records that can be compared carry the raw action of the row they land on.
AUDIT_GAP_S = 30.0
AUDIT_TICK_S = 0.5
AUDIT_MATCH_SHARE = 0.5
AUDIT_TICK_OFFSETS = (0, 1, -1)     # row tick = record tick + offset


def _audit_ts(v: Any):
    from datetime import datetime
    if not isinstance(v, str):
        return None
    try:
        return datetime.fromisoformat(v.replace("Z", "+00:00"))
    except ValueError:
        return None


def _audit_tick(a: dict) -> int | None:
    tk = _first(a, "monitor_tick", "tick")
    return tk if isinstance(tk, int) and not isinstance(tk, bool) else None


def _second_decision(a: dict) -> bool:
    """A record that is its tick's SECOND decision: one with a
    `decision_kind` (guardrail/audit.py AuditRecord, since 2026-10-20 only
    "stop_advised", the stop the ArduPilot rail flew instead of the pilot's
    action). It is written right after its tick's filter() record, at the
    same tick; it is never that tick's record and never compared with the
    row (module docstring, "A TICK CAN HAVE TWO RECORDS")."""
    return a.get("decision_kind") is not None


def audit_blocks(audit: list[dict]) -> list[list[dict]]:
    """The audit records split into flights, in file order: a new flight
    starts where the tick does not advance, the `episode_id` changes (record
    version 2), or the clock jumps further than the ticks account for.

    The one exception to "advance": a tick's second decision (a record with a
    `decision_kind`) right after the same tick's untagged filter() record is
    the same tick of the same flight. Until 2026-10-20 it started a new
    flight, so every stop-advised tick of the ArduPilot rail split its audit
    (22 "flights" on the hold rail run of tests/test_own_velocity_forecast.py,
    61 on its FSM-brake run). A second tagged record at that tick still starts
    a new flight: a flight writes one of each per tick."""
    blocks: list[list[dict]] = []
    cur: list[dict] = []
    prev: tuple | None = None
    for a in audit:
        tk, ts, eid = _audit_tick(a), _audit_ts(a.get("ts")), a.get("episode_id")
        second = _second_decision(a)
        if cur and prev is not None:
            ptk, pts, peid, psecond = prev
            same_tick = second and not psecond and tk is not None and tk == ptk
            new = (tk is not None and ptk is not None and tk <= ptk and not same_tick)
            new = new or (eid is not None and peid is not None and str(eid) != str(peid))
            if not new and ts is not None and pts is not None:
                try:
                    gap = (ts - pts).total_seconds()
                except TypeError:                 # an aware and a naive clock
                    gap = 0.0
                adv = (tk - ptk) if tk is not None and ptk is not None else 0
                new = gap > AUDIT_GAP_S + AUDIT_TICK_S * max(0, adv)
            if new:
                blocks.append(cur)
                cur = []
        cur.append(a)
        prev = (tk, ts, eid, second)
    if cur:
        blocks.append(cur)
    return blocks


def join_audit(rows: list[dict], audit: list[dict]) -> tuple[dict, dict]:
    """(row tick -> the audit record of that tick, what was joined and why).
    `join_audit_records` without the ticks' second decisions."""
    by_tick, _second, info = join_audit_records(rows, audit)
    return by_tick, info


def join_audit_records(rows: list[dict], audit: list[dict]) -> tuple[dict, dict, dict]:
    """(row tick -> the audit record of that tick, row tick -> that tick's
    second decision (a record with a `decision_kind`), what was joined and
    why).

    The flight of the audit and the tick offset whose records carry the raw
    actions of the rows they land on; a flight whose records name another
    episode_id than the rows' is never a candidate. Ties go to offset 0,
    then to the later flight (the newest in an appended file). Nothing is
    joined when no flight reaches AUDIT_MATCH_SHARE. A tick's second
    decision is neither compared with its row (its raw action is the stop
    the rail flew, not the pilot's: on the hold rail run each one counted
    as a mismatch) nor joined as the tick's record; it is returned beside."""
    if not audit:
        return {}, {}, {"records": 0, "joined": 0}
    blocks = audit_blocks(audit)
    row_by_tick = {r.get("tick", i): r for i, r in enumerate(rows)}
    row_eids = {str(r["episode_id"]) for r in rows if r.get("episode_id") is not None}
    best = None
    for bi, block in enumerate(blocks):
        eids = {str(a["episode_id"]) for a in block if a.get("episode_id") is not None}
        if row_eids and eids and not row_eids & eids:
            continue
        for off in AUDIT_TICK_OFFSETS:
            comparable = matched = 0
            for a in block:
                if _second_decision(a):
                    continue
                tk = _audit_tick(a)
                r = row_by_tick.get(tk + off) if tk is not None else None
                raw = action(r.get("raw")) if r is not None else None
                if raw is None:
                    continue
                comparable += 1
                matched += _same_action(raw, action(a.get("raw_action_world")))
            key = (matched, -abs(off), bi)
            if best is None or key > best[0]:
                best = (key, bi, off, matched, comparable)
    info: dict[str, Any] = {"records": len(audit), "flights_in_audit": len(blocks)}
    if best is None or not best[3] or best[3] < AUDIT_MATCH_SHARE * best[4]:
        info.update(joined=0, why=(
            "no flight in the audit log carries the raw actions of this log's rows "
            f"(best {best[3]} of {best[4]} comparable records)" if best else
            "every flight in the audit log names another episode_id")
            + ": nothing joined, so no step takes a record from another flight")
        return {}, {}, info
    _, bi, off, matched, comparable = best
    by_tick: dict = {}
    second: dict = {}
    for a in blocks[bi]:
        tk = _audit_tick(a)
        if tk is not None and (tk + off) in row_by_tick:
            # ticks only rise within one flight; a tick's second decision
            # goes beside its record, never over it
            (second if _second_decision(a) else by_tick)[tk + off] = a
    kinds: dict[str, int] = {}
    for a in second.values():
        kinds[str(a.get("decision_kind"))] = kinds.get(str(a.get("decision_kind")), 0) + 1
    info.update(joined=len(by_tick), flight=bi + 1, tick_offset=off,
                raw_matched=matched, raw_comparable=comparable,
                records_of_other_flights=len(audit) - len(blocks[bi]),
                stop_advice_records=kinds.pop("stop_advised", 0))
    if kinds:
        # A decision_kind this reader does not know (none exists today): kept
        # off the steps, and said.
        info["unread_decision_kinds"] = kinds
    return by_tick, second, info


def _stop_advice(row: dict, rec: dict | None) -> dict | None:
    """The decision that flew on a stop-advised tick (the ArduPilot rail's
    hold that stops clear, 2026-10-20): from the tick's "stop_advised" audit
    record when joined, else from the row's own `stop_advice`; None on every
    other tick."""
    def ids(vs):
        return [str(v.get("rule_id")) for v in vs or [] if isinstance(v, dict)]
    if rec is not None and rec.get("decision_kind") == "stop_advised":
        return {"emitted": action(rec.get("emitted_action_world")),
                "violations": ids(rec.get("violations")),
                "emitted_violations": ids(rec.get("emitted_violations")),
                "repairs": [str(r.get("operator")) for r in
                            _first(rec, "repair_results", "repairs", default=[]) or []
                            if isinstance(r, dict)],
                "source": "audit record (decision_kind stop_advised)"}
    adv = row.get("stop_advice")
    if isinstance(adv, dict):
        return {"emitted": action(adv.get("emitted")),
                "violations": ids(adv.get("violations")),
                "emitted_violations": ids(adv.get("emitted_violations")),
                "repairs": [str(r.get("operator")) for r in adv.get("repairs") or []
                            if isinstance(r, dict)],
                "source": "flight_log row (stop_advice)"}
    return None


def build_steps(ep: Episode, outcome: str | None) -> list[dict]:
    by_tick, second, ep.audit_join = join_audit_records(ep.rows, ep.audit)
    start_ns, clock = mission_clock(ep.events)
    start_gen = next((g for g in ep.generations if g.get("t") in (0, 0.0)), None) \
        or (ep.generations[0] if ep.generations else None)
    # The flight log's verdict of each tick's action, then what the bag says
    # MAVROS was sent, where a bag is read and its join is trusted.
    advices = [_stop_advice(row, second.get(row.get("tick", i)))
               for i, row in enumerate(ep.rows)]
    acts = [_actions(row, by_tick.get(row.get("tick", i)), ep.shield,
                     advised=bool(row.get("stop_advised")) or advices[i] is not None)
            for i, row in enumerate(ep.rows)]
    joined, ep.executed = bag_execution(ep, acts)
    steps = []
    for i, row in enumerate(ep.rows):
        tick = row.get("tick", i)
        audit = by_tick.get(tick)
        if row.get("policy_hash"):
            ps = {"policy_hash": row.get("policy_hash"), "generation": row.get("generation"),
                  "source": "flight_log row"}
        elif audit is not None and audit.get("policy_hash"):
            ps = {"policy_hash": audit.get("policy_hash"),
                  "generation": audit.get("generation"), "source": "audit record"}
        else:
            g = _generation_at(ep.generations, row.get("t")) or start_gen
            ps = ({"policy_hash": g.get("policy_hash"), "generation": g.get("generation"),
                   "source": "the episode's generation timeline"} if g else
                  {"policy_hash": ep.manifest.get("policy_hash"), "generation": None,
                   "source": "manifest (policy at t = 0 only)"})
        vios = row.get("violations")
        from_audit = vios is None and audit is not None
        if from_audit:
            vios = audit.get("violations")
        # The joined record's violation of the same rule lends what a row's
        # violation does not carry (record version 2's priority and grant
        # category); the row's own fields win where both have one.
        recorded = {str(v.get("rule_id", "")): _violation(v, audit=True)
                    for v in (audit or {}).get("violations") or [] if isinstance(v, dict)}
        vios = [{**recorded.get(str(v.get("rule_id", "")), {}),
                 **_violation(v, audit=from_audit)}
                for v in (vios or []) if isinstance(v, dict)]
        labelled = []
        defaulted = False
        for v in vios:
            rid = str(v.get("rule_id", ""))
            # The record's own priority (audit record version 2 copies the
            # rule's) wins over the episode's table, which wins over P0.
            lvl = v.get("priority") or ep.priorities.get(rid)
            if lvl is None:
                lvl, defaulted = "P0", True
            labelled.append({"rule_id": rid, "category": v.get("grant_category"),
                             "rule_category": v.get("rule_category"),
                             "violation_type": violation_type(v), "risk_level": lvl})
        levels = {x["risk_level"] for x in labelled}
        risk = next((lvl for lvl in K.PRIORITY_LEVELS if lvl in levels), None)
        reps = row.get("repairs")
        if reps is None and audit is not None:
            # The repairs made (repair_results; the grant's repair_attempts
            # lists the operators tried, skips included).
            reps = _first(audit, "repair_results", "repairs", default=[])
        advice = advices[i]
        step = {
            "schema": SCHEMA, "episode": ep.key, "step": i, "tick": tick, "t": row.get("t"),
            "observation": _observation(ep, i, row, start_ns, clock),
            "policy_state": ps,
            "action": _executed_from_bag(acts[i], joined[i] if joined is not None else None,
                                         ep.executed["why"] if joined is None else None),
            "outcome": outcome,
            "violation_type": sorted({x["violation_type"] for x in labelled}),
            "risk_level": risk,
            "risk_level_defaulted": defaulted,
            "violations": labelled,
            "emitted_violations": len(row.get("emitted_violations") or []),
            "repairs": [str(r.get("operator")) for r in (reps or []) if isinstance(r, dict)],
            "fsm_state_after": K._fsm_field(row, "fsm_state_after"),
            "audit_record": audit is not None,
            # The decision that flew on a stop-advised tick (_stop_advice);
            # None on every other tick and on every rail that writes none.
            "stop_advice": advice,
        }
        steps.append(step)
    return steps


def build_episode(ep: Episode, steps: list[dict], outcome, outcome_src, success) -> dict:
    table = ep.kpi_doc.get("kpi") if isinstance(ep.kpi_doc.get("kpi"), dict) else ep.kpi_doc
    vt: dict[str, int] = {}
    rl: dict[str, int] = {}
    for s in steps:
        for x in s["violations"]:
            vt[x["violation_type"]] = vt.get(x["violation_type"], 0) + 1
            rl[x["risk_level"]] = rl.get(x["risk_level"], 0) + 1
    worst = next((lvl for lvl in K.PRIORITY_LEVELS if lvl in rl), None)

    def col(fn):
        return [fn(s) for s in steps]
    return {
        "schema": SCHEMA, "episode": ep.key, "source": ep.source, "kind": ep.kind,
        "manifest": ep.manifest or None,
        "manifest_extras": ep.extras or None,
        "signature": ep.signature,
        "shield": ep.shield,
        "outcome": outcome, "outcome_source": outcome_src, "mission_success": success,
        "n_steps": len(steps),
        "policy_hash_start": (ep.generations[0].get("policy_hash") if ep.generations
                              else ep.manifest.get("policy_hash")),
        "generations": ep.generations,
        "priority_source": ep.priority_source,
        "labels": {"violation_type": vt, "risk_level": rl, "max_risk_level": worst},
        "kpi": {k: (table or {}).get(k) for k in (
            "p0_violation_escape_rate", "p0_escapes", "p0_violation_ticks",
            "mission_success", "repair_count", "mean_repair_magnitude_mps",
            "mean_time_to_safe_s")} if table else None,
        "audit_join": ep.audit_join,
        # Where each step's `executed` came from: the bag (what MAVROS was
        # sent) or, labelled a fallback, the flight log's verdict; and why.
        "executed": ep.executed,
        "adapter_log": ({"entries": len(ep.adapter),
                         "first_t": ep.adapter[0].get("t"), "last_t": ep.adapter[-1].get("t"),
                         "joined_per_step": False,
                         "why": "its entries carry no tick and a clock the flight log "
                                "does not share"} if ep.adapter else None),
        "bag": ep.bag,
        "trajectory": {
            "t": col(lambda s: s["t"]), "tick": col(lambda s: s["tick"]),
            "state": col(lambda s: s["observation"]["state"]),
            "planner_output": col(lambda s: s["action"]["planner_output"]),
            "post_repair": col(lambda s: s["action"]["post_repair"]),
            "executed": col(lambda s: s["action"]["executed"]),
            "policy_hash": col(lambda s: s["policy_state"]["policy_hash"]),
            "generation": col(lambda s: s["policy_state"]["generation"]),
            "violation_type": col(lambda s: s["violation_type"]),
            "risk_level": col(lambda s: s["risk_level"]),
        },
        "notes": ep.notes,
    }


def export(ep: Episode) -> tuple[list[dict], dict]:
    outcome, src, success = episode_outcome(ep)
    steps = build_steps(ep, outcome)
    return steps, build_episode(ep, steps, outcome, src, success)


# --------------------------------------------------------------------------- #
# writing
# --------------------------------------------------------------------------- #

def write_jsonl(path: Path, records: list[dict]) -> None:
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        for r in records:
            fh.write(json.dumps(json_safe(r), allow_nan=False) + "\n")


# --------------------------------------------------------------------------- #
# Parquet (2026-10-19)
# --------------------------------------------------------------------------- #
#
# The grant's Outputs table: "Episode exporters - Python CLI (rosbag2 -> JSONL
# / Parquet)". `pyarrow` is OPTIONAL (pyproject extra `export`): nothing in
# the Shield's import path needs it, the JSONL is written without it, and its
# absence is said (SKIP with `--format both`, exit 3 with `--format parquet`).
#
# THE FIRST WRITER, AND WHY IT WAS REPLACED. Until 2026-10-19 this was
# `pa.Table.from_pylist(flat)` with every dict / list cell turned into JSON
# text. It had never run on this project's machines (pyarrow was in neither
# env), and on a probe with pyarrow 26.0.0 it:
#   * took its columns from the FIRST record only - a key missing there was
#     dropped from every row, silently ([{a:1},{a:2,b:3}] -> no column b);
#   * crashed (ArrowInvalid, after the JSONL was written) on a column whose
#     values JSON-safety had made mixed - a float column holding a NaN became
#     a float and the string "NaN" - and on a bool / int mix;
#   * gave a reader no way to tell a JSON-encoded column from a plain string
#     one, and a column that was a dict in one row and a string in another
#     came out half encoded.
# On the four real episodes tried (a headless bundle, two rail runs, a replay
# bundle) none of that happened, which is why the tests below build the cases.
#
# THE SCHEMA NOW. Every column is decided once, over all records, in the
# order keys first appear:
#   bool              all values bool                    -> bool
#   int               all int, within int64              -> int64
#   float             int / float mixed, or float        -> float64 (NaN and
#                     (non-finite values stored as such;     +-inf kept native)
#                     every int exact in a double)
#   string            all str, each one valid UTF-8      -> string
#   json              anything else: a dict or list in any row, mixed kinds,
#                     an int beyond int64, a str with a lone surrogate (which
#                     a UTF-8 string column cannot hold: pyarrow raised
#                     UnicodeEncodeError, the T1d review) -> string, each
#                     value JSON text (the JSONL's own encoding, so non-finite
#                     floats are "NaN" / "Infinity", a surrogate is \ud800, a
#                     dict's int key is "1"), named in the schema metadata
# A column with no value is a null column. The file's metadata carries the
# export schema (SCHEMA) and `vlaguard.json_columns`; `read_parquet` decodes
# exactly those and gives non-finite floats the JSONL's strings, so a record
# read back EQUALS the record written to the JSONL (json.loads of its line),
# value for value (an int in a float column compares equal as a float, as JSON
# numbers do). A key a record does not have reads back as null. The writer
# reads its file back and compares before it reports success: a Parquet file
# that does not say what the JSONL says is deleted and the export fails (exit
# 2), never published.
#
# WHAT THE READ-BACK COMPARES (review of the T1d branch, 2026-10-20). Its
# first version took the column list from the file it had just read
# (`back[0]`) and the values from json_safe(record). So a file that had lost
# a whole column - exactly the first writer's failure, a key dropped from
# every row - compared equal on the columns it still had and was kept
# (shown: parquet_table dropping `b` gave [{a: 1}, {a: 2}] for
# [{a: 1, b: 10}, {a: 2, b: 20}], and write_parquet returned True); and a
# dict with int keys was refused although the JSONL encodes it fine. Now the
# reference is the JSONL itself: every record as json.loads reads its line
# back (`_as_jsonl`), every key of every record a column, and a missing or
# an extra column is a failure named in the error.

PARQUET_INSTALL_HINT = ("pip install pyarrow (or `pip install .[export]`); it is an "
                        "optional dependency, the JSONL never needs it")
_PQ_JSON_COLUMNS = b"vlaguard.json_columns"
_PQ_SCHEMA = b"vlaguard.schema"
_INT64 = (-(2 ** 63), 2 ** 63 - 1)


def have_pyarrow() -> bool:
    try:
        import pyarrow                                           # noqa: F401
        import pyarrow.parquet                                   # noqa: F401
    except ImportError:
        return False
    return True


def _column_kind(values: list) -> str:
    present = [v for v in values if v is not None]
    if not present:
        return "null"
    if all(isinstance(v, bool) for v in present):
        return "bool"
    if any(isinstance(v, bool) for v in present):
        return "json"
    if all(isinstance(v, int) for v in present):
        return "int" if all(_INT64[0] <= v <= _INT64[1] for v in present) else "json"
    if all(isinstance(v, (int, float)) for v in present):
        return ("float" if all(isinstance(v, float) or abs(v) <= 2 ** 53 for v in present)
                else "json")
    if all(isinstance(v, str) for v in present):
        return "string" if all(_utf8(v) for v in present) else "json"
    return "json"


def _utf8(s: str) -> bool:
    """Can a Parquet (UTF-8) string column hold `s`? Not a lone surrogate,
    which json.loads gives for a "\\ud800" escape in a flight log."""
    try:
        s.encode("utf-8")
    except UnicodeEncodeError:
        return False
    return True


def parquet_table(records: list[dict]):
    """The pyarrow Table for `records` (schema above)."""
    import pyarrow as pa
    names: list[str] = []
    seen: set = set()
    for r in records:
        for k in r:
            if k not in seen:
                seen.add(k)
                names.append(k)
    arrays, json_cols = [], []
    for k in names:
        vals = [r.get(k) for r in records]
        kind = _column_kind(vals)
        if kind == "null":
            arrays.append(pa.nulls(len(vals)))
        elif kind == "bool":
            arrays.append(pa.array(vals, pa.bool_()))
        elif kind == "int":
            arrays.append(pa.array(vals, pa.int64()))
        elif kind == "float":
            arrays.append(pa.array([None if v is None else float(v) for v in vals],
                                   pa.float64()))
        elif kind == "string":
            arrays.append(pa.array(vals, pa.string()))
        else:
            json_cols.append(k)
            arrays.append(pa.array([None if v is None else
                                    json.dumps(json_safe(v), allow_nan=False)
                                    for v in vals], pa.string()))
    table = pa.Table.from_arrays(arrays, names=names)
    return table.replace_schema_metadata({
        _PQ_SCHEMA: SCHEMA.encode(), _PQ_JSON_COLUMNS: json.dumps(json_cols).encode()})


def read_parquet(path: Path) -> list[dict]:
    """Records from a file write_parquet wrote, equal to the JSONL's."""
    import pyarrow.parquet as pq
    table = pq.read_table(path)
    meta = table.schema.metadata or {}
    if _PQ_JSON_COLUMNS not in meta:
        raise ExportError(f"{path}: no {_PQ_JSON_COLUMNS.decode()} in its metadata; "
                          f"not written by this exporter (or by its first version, "
                          f"which could not say which columns were JSON)")
    json_cols = set(json.loads(meta[_PQ_JSON_COLUMNS]))
    out = []
    for row in table.to_pylist():
        out.append({k: (json.loads(v) if k in json_cols and v is not None else json_safe(v))
                    for k, v in row.items()})
    return out


def _same_value(a, b) -> bool:
    """JSON value equality: numbers by value, containers element-wise."""
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_same_value(a[k], b[k]) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_same_value(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return type(a) is type(b) and a == b
    return a == b


def _as_jsonl(records: list[dict]) -> list[dict]:
    """The records as the JSONL holds them: json.loads of the line
    write_jsonl writes for each (non-finite floats as strings, tuples as
    lists, a dict's int keys as strings)."""
    return [json.loads(json.dumps(json_safe(r), allow_nan=False)) for r in records]


def write_parquet(path: Path, records: list[dict]) -> bool:
    """Parquet beside the JSONL (schema above). False when pyarrow is not
    installed; ExportError (and no file) when the file it wrote does not read
    back as the JSONL's records - every row, every key of every record as a
    column, every value (WHAT THE READ-BACK COMPARES, above). Any other error
    while writing or reading back also leaves no file behind."""
    if not have_pyarrow():
        return False
    import pyarrow.parquet as pq
    try:
        pq.write_table(parquet_table(records), path)
        back = read_parquet(path)
        lines = _as_jsonl(records)
    except BaseException:
        path.unlink(missing_ok=True)
        raise
    keys = list(dict.fromkeys(k for r in lines for k in r))
    got = list(back[0]) if back else []
    missing = [k for k in keys if k not in got]
    extra = [k for k in got if k not in keys]
    want = [{k: r.get(k) for k in keys} for r in lines]
    bad = next((i for i, (w, b) in enumerate(zip(want, back)) if not _same_value(w, b)), None)
    why = []
    if len(back) != len(records):
        why.append(f"{len(back)}/{len(records)} rows")
    if back and missing:
        why.append(f"missing column(s) {missing}")
    if extra:
        why.append(f"extra column(s) {extra}")
    if bad is not None:
        why.append(f"first difference at row {bad}")
    if why:
        path.unlink(missing_ok=True)
        raise ExportError(f"{path.name} did not read back as the JSONL's records "
                          f"({'; '.join(why)}); deleted, not published")
    return True


def expand(inputs: list[str]) -> list[str]:
    out = []
    for a in inputs:
        hits = sorted(glob.glob(a)) if any(c in a for c in "*?[") else [a]
        out += hits or [a]
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("inputs", nargs="+", help="replay bundles, run folders, headless bundles")
    ap.add_argument("--out", required=True, help="output folder")
    ap.add_argument("--format", choices=("both", "jsonl", "parquet"), default="both")
    ap.add_argument("--trust-store", default=None,
                    help="trust store for replay-bundle signatures (default: guardrail.bundle's)")
    args = ap.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    steps_all, eps_all, failed = [], [], 0
    for inp in expand(args.inputs):
        try:
            ep = load_episode(inp, args.trust_store)
        except ExportError as e:
            print(f"cannot read {inp}: {e}", file=sys.stderr)
            failed += 1
            continue
        steps, episode = export(ep)
        steps_all += steps
        eps_all.append(episode)
        print(f"{ep.source}: {ep.kind}, {len(steps)} steps, outcome {episode['outcome']}"
              + (f" ({'; '.join(ep.notes)})" if ep.notes else ""))
        if ep.bag and ep.bag_messages is None:
            print(f"  SKIP nearest bag message: neither `rosbags` nor the `mcap` reader "
                  f"is installed (pip install rosbags, or mcap); observations name the "
                  f"bag and the time")
        if ep.bag and ep.bag_data is None:
            print(f"  SKIP executed from the bag: {ep.bag_error}; every step's executed "
                  f"is the flight log's verdict, labelled a fallback")
    if args.format in ("both", "jsonl"):
        write_jsonl(out / "steps.jsonl", steps_all)
        write_jsonl(out / "episodes.jsonl", eps_all)
        print(f"wrote {out / 'steps.jsonl'} ({len(steps_all)} rows) and "
              f"{out / 'episodes.jsonl'} ({len(eps_all)} rows)")
    if args.format in ("both", "parquet"):
        if not have_pyarrow():
            if args.format == "both":
                # A Parquet file left from an earlier export would sit beside
                # this JSONL and disagree with it without a word.
                stale = [p for p in (out / "steps.parquet", out / "episodes.parquet")
                         if p.exists()]
                for p in stale:
                    p.unlink()
                print(f"SKIP parquet: pyarrow is not installed (JSONL written; "
                      f"{PARQUET_INSTALL_HINT})"
                      + (f"; removed the earlier export's "
                         f"{', '.join(p.name for p in stale)}" if stale else ""))
            else:
                # Parquet alone was asked for and cannot be written: nothing
                # was written, and the exit code says so (3), not success.
                print(f"cannot write parquet: pyarrow is not installed, and --format "
                      f"parquet writes nothing else (use --format both or jsonl; "
                      f"{PARQUET_INSTALL_HINT})", file=sys.stderr)
                return 3
        else:
            try:
                write_parquet(out / "steps.parquet", steps_all)
                write_parquet(out / "episodes.parquet", eps_all)
            except Exception as e:                               # noqa: BLE001
                # Neither file, so no half export reads as a whole one. Any
                # exception, not only ExportError (review of the T1d branch,
                # P2): a pyarrow or encoding error on episodes.parquet used
                # to escape as a traceback, leaving the steps.parquet just
                # written beside a stale episodes.parquet of an earlier run.
                for p in (out / "steps.parquet", out / "episodes.parquet"):
                    p.unlink(missing_ok=True)
                print(f"parquet NOT written: {type(e).__name__}: {e}", file=sys.stderr)
                return 2
            print(f"wrote {out / 'steps.parquet'} ({len(steps_all)} rows) and "
                  f"{out / 'episodes.parquet'} ({len(eps_all)} rows)")
    return 2 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
