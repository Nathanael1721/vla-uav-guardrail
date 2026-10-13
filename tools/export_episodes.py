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
  * a rail's run folder: `flight_log.jsonl`, the audit log (`audit.jsonl` or
    `audit_<id>.jsonl`), `events.jsonl`, `adapter_log.jsonl`, `metrics.json`,
    `kpi.json`, `manifest.json`, `policy_g<N>.json`, `bag/`;
  * a headless bundle (experiments/sweep_scenarios.py, the gRPC harness):
    `manifest.json`, `kpi.json` (the table nested under `kpi`, with the
    episode's `rule_priorities`), `harness_events.jsonl`,
    `manifest_extras.json`, and `flight_log.jsonl` when it was kept. Without
    the log only the episode record is written, and it says so;
    tools/rerun_episode.py re-flies the episode and writes the log.

WHAT EACH FIELD IS, WRITER BY WRITER (the part a reader must not assume)

  observation        a reference, never the image: the flight-log line and
                     the logged state (x, y, up, yaw); with a rosbag, the bag
                     file, its topics and the episode-relative time - and,
                     when the `mcap` reader is installed (it needs no ROS),
                     the log time of the nearest message per topic. Without
                     `mcap` that lookup prints SKIP and is left out.
  policy_state       the row's own `policy_hash` / `generation` (every rail
                     since 2026-10-06), else the audit record joined to the
                     tick (below), else (headless) the generation in force
                     at the tick, from the episode's accepted hot-applies in
                     harness_events.jsonl.
  planner_output     the row's `raw` (the joined audit record's `raw_action`
                     when the row has none).
  post_repair        the Shield's emitted action. On the rails that is the
                     row's `emitted`. In a HEADLESS row `emitted` is what
                     FLEW (sweep_scenarios: the raw action with the Shield
                     off, the stop in Brake, the autopilot's action in
                     LOITER / RTL / LAND), so post_repair is that only on a
                     tick the Shield flew a repair through; elsewhere it is
                     None and `post_repair_source` says why.
  executed           `flown_action` (the pymavlink rail); else the rail's
                     verdict of what it streamed: `setpoint` "pass" -> the
                     emitted action, "brake" -> zero, "none" / `flown`
                     false -> None (the autopilot held the aircraft); else
                     the headless `emitted` (what flew). `executed_source`
                     names the case. The MAVLink adapter's own log
                     (`adapter_log.jsonl`) carries no tick and a clock the
                     flight log does not share, so it is attached to the
                     episode record, not joined per step.
  outcome            the episode's (the run's stored KPI table, else
                     guardrail.kpi.compute on the log), on every step.
  violation_type     per violation, from this project's category: geofence
                     and corridor -> fence, altitude -> alt, kinematic ->
                     envelope, clearance and standoff -> distance, contract
                     -> contract (an extension: a non-finite command). The
                     grant's category `time-window` (audit record version 2:
                     a rule whose window edge falls in the lookahead) is
                     `schedule`; a record with only the grant's category maps
                     geometric -> fence. Before record version 2 `schedule`
                     never occurs: a rule out of its window raises nothing.
                     A row's violation, which carries neither `priority` nor
                     `grant_category`, takes both from the joined audit
                     record's violation of the same rule.
  risk_level         the highest priority among the tick's violations (the
                     violation's own `priority` when its audit record
                     carries it,
                     else the rule's from the policy generations, the
                     episode's recorded `rule_priorities`, or the policy the
                     manifest's hash resolves to); an unresolved rule counts
                     as P0, as everywhere in guardrail/kpi.py, and the step
                     says `risk_level_defaulted`.

THE AUDIT LOG, AND WHICH OF ITS RECORDS BELONG TO WHICH TICK

Every shape guardrail/audit.py has stored is read (`read_audit` /
`upgrade_record` list them): before record version 2 a record has `tick`
only; version 2 (2026-10-12, policies/audit_record.schema.json) adds
`monitor_tick`, `episode_id`, a violation's `priority` (the rule's, copied
into the record) and `grant_category` (whose "time-window", a rule at its
window edge, is labelled `schedule`). The world-frame actions stay
`raw_action` / `emitted_action`; the grant's body-frame keys
(`raw_action_body`, vx forward) are never read as world velocities.

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

OUTPUT

`<out>/steps.jsonl` (one row per tick) and `<out>/episodes.jsonl` (one row
per episode, the full trajectory as columns), strict JSON (a non-finite
number becomes the string "NaN" / "Infinity", never a bare token). With
`pyarrow` importable, also `steps.parquet` and `episodes.parquet` (nested
fields as JSON strings); without it the tool prints SKIP and writes JSONL
only (`--format parquet` alone then writes nothing and exits 3). Exit 0
when every input exported, 2 when one could not be read.
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

SCHEMA = "vlaguard.episode_export/1"
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
    every flight log and every audit record's `raw_action` / `emitted_action`
    write it. The body-frame `raw_action_body` (vx forward, `vz`) is not
    read: its vx and vy are not these."""
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
    """The grant's violation_type label of one violation (module docstring)."""
    if v.get("grant_category") == "time-window":
        # Audit record version 2 (2026-10-12) marks a rule whose window edge
        # falls inside the lookahead as the grant's time-window category.
        return "schedule"
    cat = v.get("category")
    if cat in VIOLATION_TYPE:
        return VIOLATION_TYPE[cat]
    g = v.get("grant_category") or cat
    return GRANT_CATEGORY_TYPE.get(g, str(g) if g else "unknown")


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
                 kind="replay", rows=got["rows"], audit=got["audit"],
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
        mcaps = {n: b for n, b in bag_files.items() if n.endswith(".mcap")}
        if mcaps:
            with tempfile.TemporaryDirectory(prefix="export-bag-") as td:
                paths = []
                for n, b in mcaps.items():
                    q = Path(td) / Path(n).name
                    q.write_bytes(b)
                    paths.append(q)
                ep.bag_messages = mcap_message_times(paths)
    ep.shield = _arm_of(ep.metrics, ep.events, {})
    return ep


def _audit_files(run: Path) -> list[Path]:
    one = run / "audit.jsonl"
    if one.is_file():
        return [one]
    return sorted(run.glob("audit_*.jsonl"))


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
        ep.audit += _jsonl(a.read_bytes())
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
        from guardrail.models import Policy
        pols = {}
        for f in sorted(run.glob("policy_g*.json")):
            try:
                pol = Policy.model_validate(_json(f))
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
            mcaps = sorted(bag.glob("*.mcap"))
            if mcaps:
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


def _actions(row: dict, audit: dict | None, shield: str | None) -> dict:
    raw = action(row.get("raw")) or action((audit or {}).get("raw_action"))
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
    post = action(row.get("emitted")) or action((audit or {}).get("emitted_action"))
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
    return {"planner_output": raw, "post_repair": post, "executed": exe,
            "post_repair_source": "emitted", "executed_source": src}


def _observation(ep: Episode, i: int, row: dict, start_ns: int | None) -> dict:
    obs = {"ref": f"{ep.source}#flight_log.jsonl:L{i + 1}",
           "state": {k: row.get(k) for k in ("x", "y", "up", "yaw_deg") if k in row}}
    if ep.bag:
        b = {"where": ep.bag.get("where"), "topics": sorted(ep.bag.get("topics") or {})}
        t = row.get("t")
        if start_ns is not None and isinstance(t, (int, float)):
            b["t_ns"] = start_ns + int(round(float(t) * 1e9))
            if ep.bag_messages:
                near = {}
                for topic, times in ep.bag_messages.items():
                    if times:
                        near[topic] = min(times, key=lambda x: abs(x - b["t_ns"]))
                b["nearest_message_ns"] = near
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


def audit_blocks(audit: list[dict]) -> list[list[dict]]:
    """The audit records split into flights, in file order: a new flight
    starts where the tick does not advance, the `episode_id` changes (record
    version 2), or the clock jumps further than the ticks account for."""
    blocks: list[list[dict]] = []
    cur: list[dict] = []
    prev: tuple | None = None
    for a in audit:
        tk, ts, eid = _audit_tick(a), _audit_ts(a.get("ts")), a.get("episode_id")
        if cur and prev is not None:
            ptk, pts, peid = prev
            new = tk is not None and ptk is not None and tk <= ptk
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
        prev = (tk, ts, eid)
    if cur:
        blocks.append(cur)
    return blocks


def join_audit(rows: list[dict], audit: list[dict]) -> tuple[dict, dict]:
    """(row tick -> the audit record of that tick, what was joined and why).

    The flight of the audit and the tick offset whose records carry the raw
    actions of the rows they land on; a flight whose records name another
    episode_id than the rows' is never a candidate. Ties go to offset 0,
    then to the later flight (the newest in an appended file). Nothing is
    joined when no flight reaches AUDIT_MATCH_SHARE."""
    if not audit:
        return {}, {"records": 0, "joined": 0}
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
                tk = _audit_tick(a)
                r = row_by_tick.get(tk + off) if tk is not None else None
                raw = action(r.get("raw")) if r is not None else None
                if raw is None:
                    continue
                comparable += 1
                matched += _same_action(raw, action(a.get("raw_action")))
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
        return {}, info
    _, bi, off, matched, comparable = best
    by_tick = {}
    for a in blocks[bi]:
        tk = _audit_tick(a)
        if tk is not None and (tk + off) in row_by_tick:
            by_tick[tk + off] = a             # ticks only rise within one flight
    info.update(joined=len(by_tick), flight=bi + 1, tick_offset=off,
                raw_matched=matched, raw_comparable=comparable,
                records_of_other_flights=len(audit) - len(blocks[bi]))
    return by_tick, info


def build_steps(ep: Episode, outcome: str | None) -> list[dict]:
    by_tick, ep.audit_join = join_audit(ep.rows, ep.audit)
    start_ns = _episode_start_ns(ep.events)
    start_gen = next((g for g in ep.generations if g.get("t") in (0, 0.0)), None) \
        or (ep.generations[0] if ep.generations else None)
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
        if vios is None and audit is not None:
            vios = audit.get("violations")
        # The joined record's violation of the same rule lends what a row's
        # violation does not carry (record version 2's priority and grant
        # category); the row's own fields win where both have one.
        recorded = {str(v.get("rule_id", "")): v for v in (audit or {}).get("violations") or []
                    if isinstance(v, dict)}
        vios = [{**recorded.get(str(v.get("rule_id", "")), {}), **v}
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
            labelled.append({"rule_id": rid, "category": v.get("category"),
                             "violation_type": violation_type(v), "risk_level": lvl})
        levels = {x["risk_level"] for x in labelled}
        risk = next((lvl for lvl in K.PRIORITY_LEVELS if lvl in levels), None)
        reps = row.get("repairs")
        if reps is None and audit is not None:
            reps = _first(audit, "repair_attempts", "repairs", default=[])
        step = {
            "schema": SCHEMA, "episode": ep.key, "step": i, "tick": tick, "t": row.get("t"),
            "observation": _observation(ep, i, row, start_ns),
            "policy_state": ps,
            "action": _actions(row, audit, ep.shield),
            "outcome": outcome,
            "violation_type": sorted({x["violation_type"] for x in labelled}),
            "risk_level": risk,
            "risk_level_defaulted": defaulted,
            "violations": labelled,
            "emitted_violations": len(row.get("emitted_violations") or []),
            "repairs": [str(r.get("operator")) for r in (reps or []) if isinstance(r, dict)],
            "fsm_state_after": K._fsm_field(row, "fsm_state_after"),
            "audit_record": audit is not None,
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


def write_parquet(path: Path, records: list[dict]) -> bool:
    """Parquet beside the JSONL, nested fields as JSON strings (one flat
    schema for every writer). False when pyarrow is not installed."""
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError:
        return False
    flat = [{k: (json.dumps(json_safe(v), allow_nan=False) if isinstance(v, (dict, list))
                 else json_safe(v)) for k, v in r.items()} for r in records]
    pq.write_table(pa.Table.from_pylist(flat), path)
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
            print(f"  SKIP nearest bag message: the `mcap` reader is not installed "
                  f"(pip install mcap); observations name the bag and the time")
    if args.format in ("both", "jsonl"):
        write_jsonl(out / "steps.jsonl", steps_all)
        write_jsonl(out / "episodes.jsonl", eps_all)
        print(f"wrote {out / 'steps.jsonl'} ({len(steps_all)} rows) and "
              f"{out / 'episodes.jsonl'} ({len(eps_all)} rows)")
    if args.format in ("both", "parquet"):
        if write_parquet(out / "steps.parquet", steps_all) and \
                write_parquet(out / "episodes.parquet", eps_all):
            print(f"wrote {out / 'steps.parquet'} and {out / 'episodes.parquet'}")
        elif args.format == "both":
            print("SKIP parquet: pyarrow is not installed (JSONL written)")
        else:
            # Parquet alone was asked for and cannot be written: nothing was
            # written, and the exit code says so (3), not success.
            print("cannot write parquet: pyarrow is not installed, and --format "
                  "parquet writes nothing else (use --format both or jsonl)",
                  file=sys.stderr)
            return 3
    return 2 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
