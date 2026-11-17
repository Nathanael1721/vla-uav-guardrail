"""Today's Shield re-run, open loop, on the Q3 runs' logged inputs (WP4-11).

    python tools/replay_q3.py --root demo/out --out docs/data/replay_q3_20261117
    python tools/replay_q3.py --root demo/out --only ros2_hotapply --json
    python tools/replay_q3.py demo/out/ros2fix2_on_fsm          # one run folder

WHAT THE GRANT ASKS FOR (WP4-11), AND WHAT THIS IS

02-implementation/stress-testing.md: "Every episode produces a self-contained
bundle that can be replayed bit-for-bit", and the Q4 report is to say that the
Q3 runs (August - October 2026) were "replayed for reproducibility". The
grant's determinism contract (03-simulation/data-flow.md, six fields with
code_revision first; layer-requirements.md L5, "a single git SHA + seed +
policy hash") defines reproducibility AT THE REVISION THE RUN FLEW. This tool
does not do that: it runs the Shield code of the checkout it is run from
(`code_revision_now` in the summary) on each run's logged inputs. It is a
regression and coverage check of today's Shield against stored flights, not
a reproducibility replay in the contract's sense. Every record names the
revision the run was flown at (`code_revision_flown`) and whether that is a
clean commit (`flown_revision_clean`): a '-dirty', '-unknown' or
'unversioned' revision cannot be checked out to replay the run as flown, and
the contract keeps such a run out of the KPI report. The headless harness
re-flies its own episodes (tools/rerun_episode.py) and the flown hil episodes
are checked against their rosbag2 capture (tools/replay_from_bag.py).

WHAT IS REPLAYED

For every run folder under `--root` whose manifest.json is dated inside the
window (the manifest's own date: the file's modification time, local; the
manifest holds no date field), the Shield the run flew is rebuilt from what
the run stored, and every logged tick's raw action is put through it again,
on the state the tick logged, in the tick order of the log:

  the policy     every generation the rows name (`generation`, else the
                 row's `policy_hash`), from the run's policy_g<N>.json files,
                 else from its signed replay bundle (<run>.replay.tar.gz);
                 a later generation is installed before the first row that
                 flew it, as the Shield commits a hot-apply (on its own
                 counted clock, the rails' `shield_t=None`: a new or edited
                 dynamic_nfz anchored at that time, an expired one's anchor
                 dropped), and its hash must be the stored generation's; a
                 generation today's Shield.hot_apply would refuse at that
                 point (a fence added after the first tick, any change to a
                 rule class locked at mission start) makes the run
                 not_replayable, since today's Shield cannot reach the state
                 the run flew;
  the Shield     guardrail.replay.rail_shield (no FSM inside: every flight
                 rail runs its own), with the forecast metrics.json recorded
                 (`shield_lookahead`: horizon_s, dt_s); a run that ran the
                 Shield's own FSM (`escalation_fsm: true`) gets it;
                 filter() is called as the flight rails call it, without a
                 time (the Shield counts its ticks at 10 Hz);
  the state      x, y, up, yaw_deg of the row; the position's own doubles
                 from trajectory.json when it has one point per row (the ROS 2
                 rail writes the pose message's doubles there; the flight log
                 rounds to 1 mm); no measured velocity and no fix flag (a run
                 whose Shield read either is refused, below);
  the rule clock the reading each point of trajectory.json (or the row)
                 carries, pinned per tick; none (every scheduled rule in
                 force) when the run logged none, as every rail flew until
                 2026-10-12.

The pipe (Shield.note_flown: what the rail SENT each tick, on a real-time
pipe clock) is NOT replayed: no rail logs the sent action with its pipe time
per tick. The Shield reads it only for the own-velocity forecast (a row with
`shield_velocity`) and for dead reckoning while the fix is lost (a row with
`fix_ok` false); a run with either row is refused.

WHAT IS COMPARED, PER TICK

  raw          the action the Shield judged (world frame) against the logged
               `raw`: a difference here means the log's raw is not the input
               the Shield had (a rail that smoothed it first);
  violations   the rule ids, in order;
  repairs      the operator names, in order        (Shield-on arm only)
  touched, braked                                   (Shield-on arm only)
  emitted      the four floats, bit for bit         (Shield-on arm only)

A row that does not log touched or braked cannot be compared on it; such
ticks are counted per field (`ticks_field_not_logged`) and named in the
record's limits, never passed silently.

`identical`: every field on every tick, emitted bit for bit. `differs`: the
first tick and field that do not, with the logged and the replayed value; the
record also says whether the DECISIONS (raw, violations, repairs, touched,
braked) agreed on every tick (`decisions_identical`) and the largest emitted
difference (`emitted_max_abs_diff`). `not_replayable`: what the run lacks,
said once, never filled in.

WHAT MAKES A RUN NOT REPLAYABLE (each is the record's `why`)

  * no flight log (the headless sweeps kept harness_events.jsonl and kpi.json
    for most episodes);
  * a headless-kinematic episode: the Shield's configuration is the scenario
    cell's (forecast, wind, response arm), which the episode does not store;
    tools/rerun_episode.py re-flies those from the library;
  * no policy IR for a generation a row names;
  * the Shield's forecast not recorded (no metrics.json `shield_lookahead`):
    the follow and PAS rails before 2026-10 did not record it;
  * an obstacle_clearance or distance_envelope rule over a map whose content
    at flight time no file records (the city maps were rebuilt on 2026-11-04
    and no run recorded a digest of the one it loaded);
  * a subject_standoff rule with a subject on some row, and no record of the
    subject's class the rail gave set_subject;
  * a policy with scheduled rules (valid_time) and no clock reading per tick;
  * the Shield flown on a measured velocity the log does not hold per tick;
  * a row where the Shield read the pipe (`shield_velocity`, or `fix_ok`
    false): the pipe is not logged per tick;
  * a generation today's Shield.hot_apply would refuse where the run
    installed it.

WHAT THIS IS NOT. It is open loop: the aircraft's path is the logged one, so
a tick after a difference is judged where the flown aircraft was, not where
the replayed Shield would have sent it. It runs TODAY's Shield code, not the
revision each run was flown at: `identical` means today's Shield decides
every logged tick as the flight did, and `differs` means it does not, which
may be a code change since the flight, a fault of the flight, or an input
the log does not carry; this tool does not tell them apart. Rail calls
between ticks that may touch the Shield's state (the hold planner's queries,
the FSM's legality checks) are not replayed. A mode that checks out each
run's clean code_revision in a git worktree and replays there would be the
contract's replay; it is not built, and no Q3 run it could have replayed
has a clean revision (17 Nov 2026: all 18 runs replayed here were flown from
a '-dirty' or 'unversioned' tree).

Exit code: 0 every replayed run identical, 1 a run differs, 2 no run could be
replayed or the arguments are wrong.
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import date, datetime
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail.models import Action4D, Policy, State                 # noqa: E402
from guardrail.replay import (_generation_files, generation_table,   # noqa: E402
                              rail_shield, read_replay)

# Q3 2026: the window the Q4 report names.
Q3_FROM = date(2026, 8, 1)
Q3_TO = date(2026, 10, 31)
# Sub-folders of a run that are never runs themselves (and are large).
SKIP_DIRS = {"bag", "episode.bag2", "vla_frames", "vla_captures", "vla_record", "view",
             "frames", "gate", "_previous", "policy_bundle"}
MAP_RULES = ("obstacle_clearance", "distance_envelope")
# What Shield.hot_apply (and the zone edits) may add, remove or change after
# the first tick; a fence may be added only before it.
HOT_EVENTS = ("dynamic_nfz", "time_window_switch", "corridor_swap")
PRE_START_ONLY = ("polygon_fence", "circle_fence")
SUBJECT_RULES = ("subject_standoff",)
VEC = ("vx", "vy", "vz_up", "yaw_rate")
POS_MATCH_M = 6e-4          # a trajectory.json point is the row's when it rounds to it
VERDICTS = ("identical", "differs", "not_replayable")


class NotReplayable(Exception):
    """The run lacks what the replay needs; the message says what."""


# --------------------------------------------------------------------------- #
# finding the runs
# --------------------------------------------------------------------------- #

def manifest_date(run: Path) -> date:
    """The manifest's date: manifest.json's modification time (local)."""
    return datetime.fromtimestamp((run / "manifest.json").stat().st_mtime).date()


def episode_wall_date(run: Path) -> str | None:
    """The episode_start event's wall date, where events.jsonl has one."""
    p = run / "events.jsonl"
    if not p.is_file():
        return None
    try:
        with p.open(encoding="utf-8") as fh:
            for line in fh:
                ev = json.loads(line)
                if ev.get("kind") == "episode_start" and isinstance(ev.get("wall"), str):
                    return ev["wall"][:10]
    except (OSError, ValueError):
        return None
    return None


def find_runs(root: Path, since: date, until: date) -> list[Path]:
    """Every folder under `root` holding a manifest.json dated in [since, until]."""
    out: list[Path] = []
    for dp, dirs, files in os.walk(root):
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS)
        if "manifest.json" in files:
            p = Path(dp)
            try:
                d = manifest_date(p)
            except OSError:
                continue
            if since <= d <= until:
                out.append(p)
    return sorted(out)


# --------------------------------------------------------------------------- #
# what a run stored
# --------------------------------------------------------------------------- #

def _json(p: Path) -> Any:
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _jsonl(p: Path) -> list[dict]:
    out = []
    with p.open(encoding="utf-8") as fh:
        for line in fh:
            if line.strip():
                out.append(json.loads(line))
    return out


@dataclass
class Inputs:
    rows: list
    policies: dict                      # generation -> Policy
    metrics: dict
    manifest: dict
    traj: list | None
    sources: dict = field(default_factory=dict)
    limits: list = field(default_factory=list)


def _bundle(run: Path) -> Path | None:
    hits = sorted(run.glob("*.replay.tar.gz"))
    return hits[0] if hits else None


def load_inputs(run: Path) -> Inputs:
    manifest = _json(run / "manifest.json") or {}
    if str(manifest.get("topology", "")).startswith("headless"):
        raise NotReplayable(
            "a headless-kinematic episode: the Shield's configuration is the scenario "
            "cell's (forecast, wind, response arm), which the episode does not store; "
            "tools/rerun_episode.py re-flies it from the scenario library")
    metrics = _json(run / "metrics.json") or {}
    sources: dict[str, str] = {}
    rows = None
    bundle = _bundle(run)
    opened = None
    if not (run / "flight_log.jsonl").is_file() and bundle is None:
        raise NotReplayable("no flight log (no flight_log.jsonl and no replay bundle "
                            "holding one)")
    shield_config(metrics)          # the cheap refusal before a bundle is opened
    if (run / "flight_log.jsonl").is_file():
        try:
            rows = _jsonl(run / "flight_log.jsonl")
        except (OSError, ValueError) as e:
            raise NotReplayable(f"flight_log.jsonl does not parse ({type(e).__name__})")
        sources["rows"] = "flight_log.jsonl"
    elif bundle is not None:
        opened = _open_bundle(bundle)
        rows = opened["rows"]
        sources["rows"] = f"{bundle.name} (flight_log.jsonl)"
    if not rows:
        raise NotReplayable("no flight log (no flight_log.jsonl and no replay bundle "
                            "holding one)")
    policies: dict[int, Policy] = {}
    files = _generation_files(run)
    if files:
        _, policies, problems = generation_table(files)
        if problems:
            raise NotReplayable("the stored policy generations do not agree: "
                                + "; ".join(problems))
        sources["policies"] = "policy_g<N>.json"
    if not policies and bundle is not None:
        opened = opened or _open_bundle(bundle)
        policies = dict(opened["generation_policies"]) or {
            int(opened["policy"].generation): opened["policy"]}
        sources["policies"] = f"{bundle.name}"
    if not policies:
        raise NotReplayable(
            f"no policy IR stored with the run (the manifest names "
            f"{manifest.get('policy_hash')}; no policy_g<N>.json and no replay bundle)")
    traj = _json(run / "trajectory.json")
    return Inputs(rows=rows, policies=policies, metrics=metrics, manifest=manifest,
                  traj=traj if isinstance(traj, list) else None, sources=sources)


def _open_bundle(path: Path) -> dict:
    try:
        return read_replay(path)
    except Exception as e:                                       # noqa: BLE001
        raise NotReplayable(f"{path.name} does not open: {type(e).__name__}: {e}")


def row_generation(row: dict, policies: dict) -> int:
    g = row.get("generation")
    if isinstance(g, int) and not isinstance(g, bool):
        if g not in policies:
            raise NotReplayable(f"tick {row.get('tick')} flew generation {g}, whose "
                                f"policy is not stored")
        return g
    h = row.get("policy_hash")
    if h:
        for n, p in sorted(policies.items()):
            if p.matches_hash(h):
                return n
        raise NotReplayable(f"tick {row.get('tick')} names policy {h}, which no stored "
                            f"generation has")
    if len(policies) == 1:
        return next(iter(policies))
    raise NotReplayable("the rows name no generation and the run stores "
                        f"{len(policies)}: which one each tick flew is not known")


# --------------------------------------------------------------------------- #
# the Shield the run flew
# --------------------------------------------------------------------------- #

def shield_config(metrics: dict) -> dict:
    la = metrics.get("shield_lookahead")
    if not isinstance(la, dict) or not all(
            isinstance(la.get(k), (int, float)) for k in ("horizon_s", "dt_s")):
        raise NotReplayable("the Shield's forecast is not recorded (no metrics.json "
                            "`shield_lookahead` with horizon_s and dt_s)")
    return {"lookahead_s": float(la["horizon_s"]), "dt": float(la["dt_s"]),
            "escalation": bool(la.get("escalation_fsm", False))}


def check_world(inp: Inputs) -> None:
    """The inputs a rule needs beyond the state, where the log lacks them."""
    rules = [c for p in inp.policies.values() for c in p.constraints]
    types = {c.type for c in rules}
    maps = sorted(types & set(MAP_RULES))
    if maps:
        where = (inp.metrics.get("params") or {}).get("citymap") if isinstance(
            inp.metrics.get("params"), dict) else None
        raise NotReplayable(
            f"the policy has {', '.join(maps)} rule(s), which read a map; "
            + (f"the flight loaded {Path(str(where)).name} " if where else
               "the map the flight gave the Shield is ")
            + "not stored with the run and no digest of its content at flight time "
              "is recorded (the city maps were rebuilt on 2026-11-04)")
    if types & set(SUBJECT_RULES) and any(
            r.get("tgt_x") is not None or r.get("subject") for r in inp.rows):
        raise NotReplayable(
            "a subject_standoff rule with a subject on the rows, and the class the "
            "rail gave Shield.set_subject is not recorded per run")
    if any(getattr(c, "valid_time", None) is not None for c in rules) and \
            not _clock_readings(inp):
        raise NotReplayable("the policy has scheduled rules (valid_time) and the run "
                            "logged no rule-clock reading per tick")
    if inp.metrics.get("shield_velocity_source") and not any(
            r.get("shield_velocity") for r in inp.rows):
        raise NotReplayable(
            f"the Shield flew on a measured velocity "
            f"({inp.metrics['shield_velocity_source']}) that the log does not hold "
            f"per tick")
    sv = sum(1 for r in inp.rows if r.get("shield_velocity"))
    nofix = sum(1 for r in inp.rows if r.get("fix_ok") is False)
    if sv or nofix:
        why = []
        if sv:
            why.append(f"{sv} row(s) flew the own-velocity forecast (`shield_velocity`)")
        if nofix:
            why.append(f"{nofix} row(s) had no valid fix (`fix_ok` false: dead reckoning)")
        raise NotReplayable(
            "the pipe the Shield read is not logged per tick: "
            + " and ".join(why) + "; both read what the rail sent (Shield.note_flown) "
            "on a real-time pipe clock, which no rail logs")


def _clock_readings(inp: Inputs) -> list | None:
    """Per row, the rule clock's reading (datetime), or None: trajectory.json
    points carry it on the ROS 2 rail, a row may carry `clock` itself."""
    if all(isinstance(r.get("clock"), str) for r in inp.rows):
        return [datetime.fromisoformat(r["clock"]) for r in inp.rows]
    if inp.traj is not None and len(inp.traj) == len(inp.rows) and all(
            isinstance(p, dict) and isinstance(p.get("clock"), str) for p in inp.traj):
        return [datetime.fromisoformat(p["clock"]) for p in inp.traj]
    return None


def _positions(inp: Inputs) -> tuple[list, str]:
    """Per row (x, y, up): trajectory.json's doubles when every point is the
    row's (rounds to it), else the row's own."""
    rows = inp.rows
    if inp.traj is not None and len(inp.traj) == len(rows):
        out = []
        for r, p in zip(rows, inp.traj):
            try:
                got = (float(p["x"]), float(p["y"]), float(p["up"]))
            except (KeyError, TypeError, ValueError):
                break
            if not all(abs(a - float(r[k])) <= POS_MATCH_M
                       for a, k in zip(got, ("x", "y", "up"))):
                break
            out.append(got)
        else:
            return out, "trajectory.json (the position's own doubles)"
    return ([(float(r["x"]), float(r["y"]), float(r["up"])) for r in rows],
            "flight_log.jsonl (rounded as logged)")


def _same_rule(a, b) -> bool:
    return a.model_dump(mode="json") == b.model_dump(mode="json")


def refused_today(shield, target: Policy) -> list[str]:
    """What in the step from the Shield's policy to `target` today's
    Shield.hot_apply (and its zone edits) would refuse at this point of the
    flight: [] when every difference is an event it accepts. After the first
    tick it accepts a dynamic_nfz added, edited or expired, and a
    time_window_switch / corridor_swap added (an earlier unscheduled one on
    the same target superseded); before it, also a polygon/circle fence
    added. Anything else is locked at mission start (LockedRuleClass)."""
    cur = {c.id: c for c in shield.policy.constraints}
    new = {c.id: c for c in target.constraints}
    started = bool(shield.mission_started)
    out = []
    for i in sorted(new.keys() - cur.keys()):
        t = new[i].type
        if t in HOT_EVENTS or (t in PRE_START_ONLY and not started):
            continue
        out.append(f"adds {t} {i!r}" + (" after the first tick" if t in PRE_START_ONLY
                                        else ""))
    for i in sorted(cur.keys() - new.keys()):
        if cur[i].type not in HOT_EVENTS:
            out.append(f"removes {cur[i].type} {i!r}")
    for i in sorted(cur.keys() & new.keys()):
        a, b = cur[i], new[i]
        if a.type == b.type == "dynamic_nfz" or _same_rule(a, b):
            continue
        out.append(f"changes {b.type} {i!r}")
    return out


def install(shield, target: Policy) -> None:
    """Make `target` the Shield's policy the way a hot-apply commits one: the
    rules replaced in place; a dynamic_nfz that is new, or whose definition
    changed (an edit: Shield._edit_zone re-anchors the zone at the event
    time), anchored on the Shield's counted clock (the rails apply events
    with shield_t=None); the anchor of a zone that is gone dropped (expire).
    Refused (NotReplayable) when today's hot_apply would not take this step.
    Its hash must then be the stored generation's."""
    bad = refused_today(shield, target)
    if bad:
        raise NotReplayable(
            f"generation {target.generation} " + "; ".join(bad) + ", which today's "
            f"Shield.hot_apply refuses at that point of the flight (LockedRuleClass: "
            f"the class is locked at mission start), so today's Shield cannot reach "
            f"the state the run flew")
    cur = {c.id: c for c in shield.policy.constraints}
    keep = {c.id for c in target.constraints}
    anchors = {k: v for k, v in shield._anchors.items() if k in keep}
    te = shield._event_time(None)
    for c in target.constraints:
        if c.type == "dynamic_nfz" and (c.id not in cur or not _same_rule(cur[c.id], c)):
            anchors[c.id] = te
    shield.policy.generation = target.generation - 1     # _commit adds one
    shield._commit(list(target.constraints), "replay_generation", "-", "-", None, anchors)
    if not target.matches_hash(shield.policy.policy_hash):
        raise NotReplayable(
            f"generation {target.generation} rebuilt in the Shield hashes to "
            f"{shield.policy.policy_hash}, not the stored {target.policy_hash}")


# --------------------------------------------------------------------------- #
# one run
# --------------------------------------------------------------------------- #

def _vec(d: Any) -> list | None:
    if not isinstance(d, dict):
        return None
    try:
        return [float(d[k]) for k in VEC]
    except (KeyError, TypeError, ValueError):
        return None


def compare(row: dict, dec, shield_on: bool,
            not_logged: dict | None = None) -> list[tuple[str, Any, Any]]:
    """The fields of one tick that differ: (field, logged, replayed). A field
    the row does not log (touched, braked) is counted in `not_logged`."""
    out = []
    got_raw = [float(getattr(dec.raw, k)) for k in VEC]
    if _vec(row.get("raw")) != got_raw:
        out.append(("raw", row.get("raw"), dict(zip(VEC, got_raw))))
    ids = [v.rule_id for v in dec.violations]
    logged = [v.get("rule_id") for v in row.get("violations") or []]
    if ids != logged:
        out.append(("violations", logged, ids))
    if shield_on:
        ops = [r.operator for r in dec.repairs]
        logged = [r.get("operator") for r in row.get("repairs") or []]
        if ops != logged:
            out.append(("repairs", logged, ops))
        for name in ("touched", "braked"):
            if name not in row:
                if not_logged is not None:
                    not_logged[name] = not_logged.get(name, 0) + 1
            elif bool(row[name]) != bool(getattr(dec, name)):
                out.append((name, row[name], bool(getattr(dec, name))))
        em = [float(getattr(dec.emitted, k)) for k in VEC]
        if _vec(row.get("emitted")) != em:
            out.append(("emitted", row.get("emitted"), dict(zip(VEC, em))))
    return out


def _code_revision_now() -> str | None:
    """The revision of the code this replay runs, as a run's manifest records
    one: '-dirty' when the tree had uncommitted changes or untracked files
    (the tool itself uncommitted, for one), '-unknown' when git could not say."""
    try:
        from guardrail.manifest import code_revision
        return code_revision(ROOT)
    except Exception:                                            # noqa: BLE001
        return None


def revision_clean(rev: Any) -> bool:
    """A code_revision a checkout reproduces: a commit, no dirty/unknown flag."""
    return (isinstance(rev, str) and bool(rev) and rev != "unversioned"
            and not rev.endswith(("-dirty", "-unknown")) and "unresolved" not in rev)


def replay_run(run: Path, *, root: Path | None = None) -> dict:
    name = str(run.relative_to(root)) if root is not None else run.name
    rec: dict[str, Any] = {"run": name.replace("\\", "/"), "verdict": None}
    try:
        rec["manifest_date"] = manifest_date(run).isoformat()
    except OSError:
        rec["manifest_date"] = None
    wall = episode_wall_date(run)
    if wall:
        rec["episode_start_date"] = wall
    try:
        inp = load_inputs(run)
        rec["code_revision_flown"] = inp.manifest.get("code_revision")
        rec["flown_revision_clean"] = revision_clean(rec["code_revision_flown"])
        cfg = shield_config(inp.metrics)
        check_world(inp)
        gens = [row_generation(r, inp.policies) for r in inp.rows]
        for r in inp.rows:
            for k in ("x", "y", "up"):
                if not isinstance(r.get(k), (int, float)):
                    raise NotReplayable(f"tick {r.get('tick')} logs no {k}")
            if _vec(r.get("raw")) is None:
                raise NotReplayable(f"tick {r.get('tick')} logs no raw action (world "
                                    f"frame, the Shield's input)")
    except NotReplayable as e:
        rec.update(verdict="not_replayable", why=str(e))
        return rec
    arm = str(inp.metrics.get("shield") or "on")
    shield_on = arm != "off"
    pos, pos_source = _positions(inp)
    clocks = _clock_readings(inp)
    limits = list(inp.limits)
    if not all(isinstance(r.get("yaw_deg"), (int, float)) for r in inp.rows):
        limits.append("yaw_deg is not on every row: 0 is given where it is missing")
    if pos_source.startswith("flight_log"):
        limits.append("positions are the flight log's, rounded to 1 mm (no "
                      "trajectory.json with one point per row): a repair's size can "
                      "differ in its last digits for that alone")
    if not revision_clean(inp.manifest.get("code_revision")):
        limits.append(f"flown at {inp.manifest.get('code_revision')!r}: not a clean "
                      f"commit, so the run cannot be replayed at the code it flew; "
                      f"this is today's Shield on its inputs")
    pin: list = [None]
    gen0 = gens[0]
    try:
        shield = rail_shield(inp.policies[gen0], lookahead_s=cfg["lookahead_s"],
                             dt=cfg["dt"], clock=(lambda: pin[0]) if clocks else None)
        if cfg["escalation"]:
            from guardrail.shield import Shield
            shield = Shield(inp.policies[gen0], lookahead_s=cfg["lookahead_s"],
                            dt=cfg["dt"], now=(lambda: pin[0]) if clocks else None)
    except Exception as e:                                       # noqa: BLE001
        rec.update(verdict="not_replayable",
                   why=f"today's Shield refuses the stored policy: {type(e).__name__}: {e}")
        return rec
    current = gen0
    first = first_decision = None
    touched = repaired = 0
    counts: dict[str, int] = {}
    not_logged: dict[str, int] = {}
    decisions_identical = True
    worst = 0.0
    t0 = time.perf_counter()
    for i, row in enumerate(inp.rows):
        if gens[i] != current:
            try:
                install(shield, inp.policies[gens[i]])
            except NotReplayable as e:
                rec.update(verdict="not_replayable", why=str(e))
                return rec
            except Exception as e:                               # noqa: BLE001
                rec.update(verdict="not_replayable",
                           why=f"generation {gens[i]} could not be installed: "
                               f"{type(e).__name__}: {e}")
                return rec
            current = gens[i]
        if clocks:
            pin[0] = clocks[i]
        # No velocity, no fix flag: check_world refused every run whose
        # Shield read either (and with them the pipe, which is not logged).
        st = State(x=pos[i][0], y=pos[i][1], up=pos[i][2],
                   yaw_deg=float(row.get("yaw_deg") or 0.0))
        dec = shield.filter(st, Action4D(**{k: float(row["raw"][k]) for k in VEC}))
        diffs = compare(row, dec, shield_on, not_logged)
        for f_, logged, got in diffs:
            counts[f_] = counts.get(f_, 0) + 1
            if f_ != "emitted":
                decisions_identical = False
            else:
                lv, gv = _vec(logged), _vec(got)
                if lv is not None and gv is not None:
                    worst = max(worst, max(abs(a - b) for a, b in zip(lv, gv)))
            here = {"tick": row.get("tick"), "row": i, "t": row.get("t"),
                    "field": f_, "logged": logged, "replayed": got}
            if first is None:
                first = here
            if first_decision is None and f_ != "emitted":
                first_decision = here
        touched += bool(row.get("touched"))
        repaired += bool(row.get("repairs"))
    if not_logged:
        limits.append("not compared, the row does not log it: " + ", ".join(
            f"{k} on {n} tick(s)" for k, n in sorted(not_logged.items())))
    rec.update({
        "verdict": "differs" if first else "identical",
        "first_difference": first,
        "ticks": len(inp.rows),
        "ticks_differing_by_field": counts,
        "ticks_field_not_logged": not_logged,
        "decisions_identical": decisions_identical,
        "first_decision_difference": first_decision,
        # What the run had for the Shield to do: an identical run whose log
        # shows no touched tick would have checked little.
        "logged_touched_ticks": touched,
        "logged_repaired_ticks": repaired,
        "emitted_max_abs_diff": (None if not shield_on else
                                 (0.0 if not counts.get("emitted") else
                                  float(f"{worst:.3g}") if math.isfinite(worst) else worst)),
        "arm": arm,
        "generations": sorted(set(gens)),
        "shield": {**cfg, "clock": "pinned per tick" if clocks else "none"},
        "state_source": pos_source,
        "sources": inp.sources,
        "code_revision_flown": inp.manifest.get("code_revision"),
        "flown_revision_clean": revision_clean(inp.manifest.get("code_revision")),
        "replay_s": round(time.perf_counter() - t0, 2),
        "limits": limits,
    })
    return rec


# --------------------------------------------------------------------------- #
# the campaign
# --------------------------------------------------------------------------- #

def summarise(records: list[dict], *, command: str, seconds: float, window: tuple,
              code_now: str | None, scanned: int) -> dict:
    by = {v: sum(1 for r in records if r["verdict"] == v) for v in VERDICTS}
    reasons: dict[str, int] = {}
    for r in records:
        if r["verdict"] == "not_replayable":
            key = _reason_key(r["why"])
            reasons[key] = reasons.get(key, 0) + 1
    replayed = [r for r in records if r["verdict"] in ("identical", "differs")]
    return {
        "command": command, "seconds": round(seconds, 1),
        "what": ("today's Shield (code_revision_now) re-run open loop on each run's "
                 "logged inputs: a regression check, not a replay at the revision "
                 "each run was flown at"),
        "window": {"from": window[0].isoformat(), "to": window[1].isoformat(),
                   "by": "manifest.json modification date (local)"},
        "code_revision_now": code_now,
        "runs_in_window": scanned,
        "verdicts": by,
        "replayed_flown_at_a_clean_revision": sum(
            1 for r in replayed if r.get("flown_revision_clean")),
        "decisions_identical_on_replayed": sum(1 for r in replayed if r["decisions_identical"]),
        "not_replayable_by_reason": dict(sorted(reasons.items(), key=lambda kv: -kv[1])),
        "replayed": [{k: r.get(k) for k in (
            "run", "manifest_date", "verdict", "ticks", "logged_touched_ticks",
            "logged_repaired_ticks", "decisions_identical", "emitted_max_abs_diff",
            "first_difference", "first_decision_difference", "ticks_differing_by_field",
            "ticks_field_not_logged", "generations", "arm", "state_source",
            "code_revision_flown", "flown_revision_clean")}
            for r in replayed],
    }


def _reason_key(why: str) -> str:
    """A not-replayable reason with its run-specific parts dropped, to count."""
    for stem in ("a headless-kinematic episode", "no flight log", "no policy IR stored",
                 "the Shield's forecast is not recorded", "the policy has",
                 "a subject_standoff rule", "the Shield flew on a measured velocity",
                 "the pipe the Shield read is not logged per tick",
                 "the stored policy generations", "today's Shield refuses"):
        if why.startswith(stem):
            if stem == "the policy has":
                return ("the policy has scheduled rules and no clock per tick"
                        if "scheduled" in why else
                        "a map rule, and the map flown is not recorded")
            return stem
    if why.startswith("generation ") and "today's Shield.hot_apply refuses" in why:
        return "a generation today's Shield.hot_apply refuses where the run installed it"
    if why.startswith("tick ") and "generation" in why:
        return "a generation's policy is not stored"
    if why.startswith("tick "):
        return "a row lacks a state or raw field"
    return why[:80]


def _table(summary: dict) -> str:
    lines = ["| run | manifest date | flown at | generations | ticks | touched | verdict | "
             "decisions identical | max abs emitted diff | first difference | "
             "first decision difference |", "|" + "---|" * 11]
    for r in summary["replayed"]:
        fd, dd = r["first_difference"], r["first_decision_difference"]
        lines.append(f"| {r['run']} | {r['manifest_date']} | {r['code_revision_flown']} | "
                     f"{','.join(str(g) for g in r['generations'])} | {r['ticks']} | "
                     f"{r['logged_touched_ticks']} | {r['verdict']} | "
                     f"{'yes' if r['decisions_identical'] else 'no'} | "
                     f"{r['emitted_max_abs_diff']} | "
                     f"{'-' if not fd else 'tick ' + str(fd['tick']) + ' ' + fd['field']} | "
                     f"{'-' if not dd else 'tick ' + str(dd['tick']) + ' ' + dd['field']} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="*", type=Path, help="run folders (default: --root's)")
    ap.add_argument("--root", type=Path, default=ROOT / "demo" / "out")
    ap.add_argument("--since", default=Q3_FROM.isoformat())
    ap.add_argument("--until", default=Q3_TO.isoformat())
    ap.add_argument("--only", default=None, help="a substring the run path must hold")
    ap.add_argument("--out", type=Path, default=None,
                    help="write summary.json, per_run.jsonl.gz and table.md here")
    ap.add_argument("--json", action="store_true", help="print the summary as JSON")
    a = ap.parse_args(argv)
    try:
        since, until = date.fromisoformat(a.since), date.fromisoformat(a.until)
    except ValueError as e:
        print(f"bad date: {e}", file=sys.stderr)
        return 2
    t0 = time.perf_counter()
    if a.runs:
        runs, root = list(a.runs), None
    else:
        if not a.root.is_dir():
            print(f"{a.root}: not a folder", file=sys.stderr)
            return 2
        runs, root = find_runs(a.root, since, until), a.root
    if a.only:
        runs = [r for r in runs if a.only in str(r).replace("\\", "/")]
    records = []
    for r in runs:
        rec = replay_run(r, root=root)
        records.append(rec)
        if rec["verdict"] != "not_replayable":
            fd = rec["first_difference"]
            print(f"{rec['run']:60s} {rec['verdict']:10s} ticks {rec['ticks']:5d}"
                  + (f"  first: tick {fd['tick']} {fd['field']}" if fd else ""),
                  flush=True)
    # The command as run, with --out's folder named by its last part only
    # (where the summary was written is not part of what was computed).
    args = list(argv if argv is not None else sys.argv[1:])
    for i, x in enumerate(args[:-1]):
        if x == "--out":
            args[i + 1] = f"<out>/{Path(args[i + 1]).name}"
    summary = summarise(records, command=" ".join(["python", "tools/replay_q3.py"] + args),
                        seconds=time.perf_counter() - t0, window=(since, until),
                        code_now=_code_revision_now(), scanned=len(runs))
    if a.out is not None:
        a.out.mkdir(parents=True, exist_ok=True)
        (a.out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n",
                                            encoding="utf-8", newline="\n")
        with gzip.open(a.out / "per_run.jsonl.gz", "wt", encoding="utf-8",
                       newline="\n") as fh:
            for rec in records:
                fh.write(json.dumps(rec) + "\n")
        (a.out / "table.md").write_text(_table(summary) + "\n", encoding="utf-8",
                                        newline="\n")
    if a.json:
        print(json.dumps(summary, indent=1))
    else:
        print(json.dumps({k: summary[k] for k in ("verdicts", "not_replayable_by_reason",
                                                  "runs_in_window", "seconds")}, indent=1))
    by = summary["verdicts"]
    if by["differs"]:
        return 1
    return 0 if by["identical"] else 2


if __name__ == "__main__":
    sys.exit(main())
