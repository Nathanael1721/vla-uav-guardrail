"""Replay an episode at the code revision it was flown at (WP4-11).

    python tools/replay_at_revision.py demo/out/hil_smoke/ntFIN2_001_nfz-head-on
    python tools/replay_at_revision.py --root demo/out --window 2026-08-01:2026-10-31 \
        --topology hil --jobs 8 --out docs/data/replay_at_revision_20261118.json

WHAT THE GRANT ASKS FOR

02-implementation/stress-testing.md, "Episode bundle": "Every episode
produces a self-contained bundle that can be replayed bit-for-bit".
03-simulation/data-flow.md, "Determinism contract": "Every recorded episode
is reproducible bit-for-bit from six fields", the first of them
`code_revision: <git SHA across the entire stack>`; layer-requirements.md L5:
"reproducible bit-for-bit from a single git SHA + seed + policy hash".
02-implementation/overview.md, Q4: "the full KPI matrix from Q3 runs replayed
for reproducibility".

Reproducibility in that sense is AT THE REVISION THE EPISODE FLEW. Until this
tool the lab's replays ran today's code: tools/replay_q3.py re-runs today's
Shield on logged inputs (a regression check), tools/rerun_episode.py re-flies
a headless episode with the checkout it is run from, and
tools/replay_from_bag.py reads the rosbag2 capture without running the Shield.
This tool checks the manifest's `code_revision` out and replays there.

WHAT IT DOES, PER EPISODE

  1. Reads manifest.json (the run folder's, else the one inside the run's
     <run>.replay.tar.gz) and refuses, with the reason, a revision that a
     checkout cannot reproduce: none recorded, 'unversioned', '-dirty' (the
     tree had uncommitted changes), '-unknown', or a SHA that is not a commit
     of this repository (`refused`).
  2. Groups the episodes by revision and, per revision, makes a TEMPORARY
     DETACHED git worktree at that SHA (`git worktree add --detach`, under
     --scratch, outside the repository). Nothing is linked into it (no
     junction, no symlink: removing a worktree that holds a junction deletes
     the target's files, docs/memory of 22 Oct).
  3. Runs this file's driver (`--driver`) in a separate interpreter started
     with -I, its import path the worktree only: every module the replay
     imports - the Shield, the audit writer, the scenario runner, that
     revision's own tools/rerun_episode.py - is the flown revision's.
  4. Removes the worktree (`git worktree remove --force`, then `git worktree
     prune`), after unlinking any reparse point found in it without following
     it; the record of each removal is in the output (`worktrees`).

TWO REPLAYS, BY WHAT THE EPISODE IS

  refly      a headless-kinematic episode (manifest topology
             'headless-kinematic'): re-FLOWN, closed loop, by the revision's
             own tools/rerun_episode.py `rebuild_and_fly` from the bundle
             (manifest_extras.json's self-contained episode, else the
             library cell). The simulated aircraft, the pilot, the Shield
             and the FSM all run again. Compared, each where the bundle
             stored it:
               shield_audit.jsonl    every record, bit for bit (JSON text),
                                     except `ts` (the wall-clock time the
                                     record was written);
               flight_log.jsonl      byte for byte (line endings to LF);
               harness_events.jsonl  every event, bit for bit, except
                                     `policy_source.path` (the checkout the
                                     policy file was read from);
               kpi.json `kpi`        the whole KPI table, re-scored by the
                                     revision's own score();
               the policy hash at t = 0 against the manifest's.
  open_loop  a flown rail episode (hil, sitl, dev, Project AirSim): the
             aircraft cannot be flown again without the rig, so the
             revision's Shield is run OPEN LOOP on the inputs the episode
             logged, tick by tick, in order (the method of
             tools/replay_q3.py, at the flown code): the policy generation
             each row names (installed as the Shield commits a hot-apply,
             its hash checked against the stored generation's), the
             position (trajectory.json's doubles when it has one point per
             row, else the log's 1 mm), the heading (the audit record's
             `body_yaw_deg`, the full double, on a tick that has one; the
             log's 0.01 degree elsewhere), the rule clock's reading, the
             fix and estimator flags, and the world-frame raw action; and,
             where every row logs it (`note_flown`, the ROS 2 node since
             2026-11-21), the pipe the Shield read: each tick's pipe-clock
             readings handed to its filter in order, then what the rail
             sent noted at its pipe time (Shield.note_flown), so the dead
             reckoning of a lost fix or estimator and the clearance pipe
             are rebuilt as flown. Under the ROS 2 node's perception
             stressors (--gps-noise-m / --latency-ticks, PerceptionStress,
             since 2026-11-21: every row logs `shield_pose`, metrics.json
             `perception_stress`) the Shield judged a degraded pose, not the
             true one: the replay hands it that pose with the node's
             pos_sigma_m and the current heading, the doubles rebuilt as
             the node drew them (trajectory.json's positions, the recorded
             seed, sigma and latency) when every row's rebuilt pose rounds
             to its shield_pose, else the rows' shield_pose (0.1 mm, a
             limit); shield_pose on some rows only is refused.
             Compared:
               shield_audit.jsonl    every record, bit for bit (JSON text),
                                     except `ts` and the `fsm_*` fields: the
                                     escalation FSM of a rail runs in the
                                     NODE, on the autopilot's mode and the
                                     hold planner, which an open-loop replay
                                     does not have; it re-derives the
                                     Shield's part of each record, not the
                                     node's. A tick the replayed Shield
                                     touched with no stored record, or a
                                     stored record on a tick with no row,
                                     is a difference;
               flight_log.jsonl      per row: the raw action the Shield
                                     judged, the rule ids violated, the
                                     repair operators, touched, braked and
                                     the four emitted floats bit for bit
                                     (Shield-on arm; raw and violations on a
                                     Shield-off arm); on a run that logs the
                                     pipe, also the Shield's X8 verdict
                                     (`position_lost`, where the row has
                                     it) and how many pipe-clock readings
                                     the tick took (`pipe_reads`).

VERDICTS

  identical       every compared item agreed, bit for bit as above;
  differs         the first difference: where (audit / log / events / kpi),
                  the tick, the field, the stored and the replayed value;
  not_replayable  the episode lacks an input the replay needs, or the flown
                  revision cannot run it (the reason, said once, never
                  filled in): for open_loop the reasons of
                  tools/replay_q3.py (no forecast recorded, a map rule, a
                  subject rule, scheduled rules without a clock, the pipe
                  the Shield read - a measured velocity, or a lost fix or
                  estimator on a run whose rows do not log `note_flown` -
                  not logged per tick) plus what the revision lacks; for
                  refly what rebuild_and_fly refuses;
  refused         the revision is not a clean commit of this repository.

WHAT THIS IS NOT

An open-loop replay judges each tick where the flown aircraft was; it is the
Shield's half of the episode at the flown code, not a re-flight of the
autopilot and simulator, which needs the rig (docs/RUNBOOK-orin-hil.md): it
does not reproduce the trajectory (the grant's data-flow.md "Replay path"),
and it does not compare the node's escalation FSM fields (fsm_*), which step
on the autopilot's mode. The replay runs on the interpreter and libm it is
started under (--python), not the companion's that flew a hil episode
(aarch64 on the Orin); each record names both (`flown_on`: host and, where
the run recorded it, the Python version; `replayed_on`), so a last-digit
difference can be put to the host or to the interpreter only by replaying
under the other - on this desktop the interpreter (--python), on the Orin
the host (a rig step) - never by assumption. The rows log no velocity the
Shield did not use, and before 2026-11-21 no pipe clock: an episode whose
Shield read a measured velocity, or the pipe of a lost fix or estimator on
rows without `note_flown`, is refused, not approximated.

Exit code: 0 every replayed episode identical, 1 one differs, 2 nothing
could be replayed or the arguments are wrong.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import platform
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import date, datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
TOOL = Path(__file__).resolve()

VERDICTS = ("identical", "differs", "not_replayable", "refused")
# What a stored audit record holds that no replay can reproduce: the wall-clock
# time it was written.
AUDIT_WALL_FIELDS = ("ts",)
# Open loop only: the rail node's escalation FSM fields (module docstring).
NODE_FSM_PREFIX = "fsm_"
# The audit record's body-frame actions: the writer turns the Shield's
# world-frame raw / emitted into the body frame at the tick's heading
# (guardrail.audit._body). A difference in these alone, under the size below,
# with the world-frame actions and the heading equal, is in the cos / sin of
# the heading, not a decision (difference_class); whose cos / sin - the
# host's libm or the interpreter's - the class does not say.
BODY_FRAME_FIELDS = ("raw_action", "emitted_action")
BODY_FRAME_ULP_TOL = 1e-12
# A difference in floats only - every rule id, operator, verdict and string
# equal - of at most this much (m/s or m) is classed `float_last_digits`:
# arithmetic in the last digits, not a decision. The class names no cause
# (the host's libm, the interpreter's float routines, or the replay itself);
# replaying under another interpreter (--python) or host tells them apart.
# Any larger, or any other kind of difference, is `decision`.
FLOAT_LAST_DIGITS_TOL = 1e-9
# harness_events.jsonl: where the flying checkout read the policy file from.
EVENT_HOST_FIELDS = (("policy_source", "path"),)
SKIP_DIRS = {"bag", "episode.bag2", "vla_frames", "vla_captures", "vla_record", "view",
             "frames", "gate", "_previous", "policy_bundle"}
VEC = ("vx", "vy", "vz_up", "yaw_rate")
POS_MATCH_M = 6e-4          # a trajectory.json point is the row's when it rounds to it
MAP_RULES = ("obstacle_clearance", "distance_envelope")
SUBJECT_RULES = ("subject_standoff",)
# metrics.json keys of a rail's Shield configuration the open-loop replay
# rebuilds; another key there with a value is a configuration it does not
# know, and the episode is refused rather than replayed on a guess.
KNOWN_FORECAST_KEYS = {"mode", "flag", "response", "velocity_source", "velocity_topic",
                       "velocity_max_age_s", "path_repair", "note", "source",
                       "velocity_ticks", "velocity_ticks_seen", "shield_velocity_ticks",
                       "velocity_nonfinite", "clear_response"}
# Counts in metrics.json shield_forecast of ticks the Shield was GIVEN a
# measured velocity: any above zero and the pipe was read.
VELOCITY_USED_KEYS = ("velocity_ticks", "shield_velocity_ticks")
# The flight-log row key of the pipe the Shield read on the tick (guardrail.
# replay PIPE_ROW_KEY / PipeRecorder, the ROS 2 node since 2026-11-21): what
# the rail sent (note_flown's t, vx, vy) and the pipe clock's readings inside
# the tick's filter. Named here, not imported: the driver runs older
# revisions that have no such constant.
PIPE_ROW_KEY = "note_flown"
# The ROS 2 node's perception stressors (sitl/ros2_shield_node.py
# PerceptionStress, since 2026-11-21): the generator and draw order its
# metrics.json perception_stress names, and the decimals of a row's
# `shield_pose`. Named here, not imported: the node module pulls in the
# whole rail stack, and a run's metrics.json says which generator it drew.
PERCEPTION_RNG = "random.Random(f'{seed}:gps')"
PERCEPTION_DRAW_ORDER = ["x", "y", "up"]
SHIELD_POSE_ROUND = 4


class Refused(Exception):
    """The episode cannot be replayed at its revision; the message says why."""


# --------------------------------------------------------------------------- #
# small shared helpers (standard library only: the driver runs them under the
# flown revision's import path)
# --------------------------------------------------------------------------- #

def revision_refusal(rev: Any) -> str | None:
    """Why `rev` cannot be checked out to replay the episode as flown, or
    None for a clean commit id."""
    if not isinstance(rev, str) or not rev.strip():
        return "the manifest records no code_revision"
    if rev == "unversioned":
        return "flown from an unversioned tree ('unversioned'): no commit holds the code"
    if rev.endswith("-dirty"):
        return (f"flown from {rev!r}: the tree had uncommitted changes or untracked "
                f"files, so no commit holds the code that flew")
    if rev.endswith("-unknown") or "unresolved" in rev:
        return f"flown from {rev!r}: git could not say what the tree was"
    if not all(c in "0123456789abcdef" for c in rev.lower()) or len(rev) < 7:
        return f"code_revision {rev!r} is not a commit id"
    return None


def git(repo: Path, *args: str, check: bool = True,
        timeout: float = 300.0) -> subprocess.CompletedProcess:
    r = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=timeout)
    if check and r.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)}: {r.stderr.strip() or r.stdout.strip()}")
    return r


def resolve_commit(repo: Path, rev: str) -> str | None:
    """The full SHA of `rev` when it names a commit of `repo`, else None."""
    r = git(repo, "rev-parse", "--verify", "--quiet", f"{rev}^{{commit}}", check=False)
    out = r.stdout.strip()
    return out if r.returncode == 0 and len(out) == 40 else None


def _json_file(p: Path) -> Any:
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _replay_tar(run: Path) -> Path | None:
    hits = sorted(run.glob("*.replay.tar.gz"))
    return hits[0] if hits else None


def _tar_member(tar_path: Path, name: str) -> bytes | None:
    try:
        with tarfile.open(tar_path, "r:gz") as tf:
            try:
                f = tf.extractfile(name)
            except KeyError:
                return None
            return f.read() if f is not None else None
    except (OSError, tarfile.TarError):
        return None


def read_run_file(run: Path, name: str) -> tuple[bytes | None, str | None]:
    """A run's file: the folder's, else the replay bundle's `episode/<name>`.
    Returns (bytes, where it came from)."""
    p = run / name
    if p.is_file():
        return p.read_bytes(), name
    tar = _replay_tar(run)
    if tar is not None:
        data = _tar_member(tar, "episode/" + name)
        if data is not None:
            return data, f"{tar.name} (episode/{name})"
    return None, None


def read_manifest(run: Path) -> dict | None:
    data, _ = read_run_file(run, "manifest.json")
    if data is None:
        return None
    try:
        doc = json.loads(data)
    except ValueError:
        return None
    return doc if isinstance(doc, dict) else None


EPISODE_STAMP = re.compile(r"(\d{4}-\d{2}-\d{2})T\d{2}-\d{2}-\d{2}Z")
FLIGHT_DATE_BASES = ("the episode folder's UTC stamp",
                     "events.jsonl's first wall time (UTC)",
                     "manifest.json's file time (local; the run records no flight time)")


def _first_wall(data: bytes | None) -> date | None:
    if not data:
        return None
    for line in data.decode("utf-8", "replace").splitlines()[:5]:
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        wall = rec.get("wall") if isinstance(rec, dict) else None
        if isinstance(wall, str):
            try:
                t = datetime.fromisoformat(wall.replace("Z", "+00:00"))
            except ValueError:
                continue
            if t.tzinfo is not None:
                t = t.astimezone(timezone.utc)
            return t.date()
    return None


def flight_date(run: Path) -> tuple[date | None, str | None]:
    """(the date the episode flew, how it was read). From the flight's own
    records, never from a file time a copy, a restore, a sync or a re-sign
    can change (until 2026-11-18 the window read manifest.json's
    modification time alone): the UTC stamp the harness put in the episode
    folder's name (episode-<UTC>--...), else the first wall time in the
    run's events.jsonl (in the folder, or in its replay bundle), in UTC.
    Only a run holding neither falls back to manifest.json's file time,
    and says so (FLIGHT_DATE_BASES)."""
    m = EPISODE_STAMP.search(run.name)
    if m:
        try:
            return date.fromisoformat(m.group(1)), FLIGHT_DATE_BASES[0]
        except ValueError:
            pass
    ev = run / "events.jsonl"
    data = None
    if ev.is_file():
        with open(ev, "rb") as fh:
            data = b"".join(fh.readline() for _ in range(5))
    else:
        data, _ = read_run_file(run, "events.jsonl")
    d = _first_wall(data)
    if d is not None:
        return d, FLIGHT_DATE_BASES[1]
    p = run / "manifest.json"
    if not p.is_file():
        return None, None
    return datetime.fromtimestamp(p.stat().st_mtime).date(), FLIGHT_DATE_BASES[2]


def _jsonl_bytes(data: bytes) -> list[dict]:
    return [json.loads(line) for line in data.decode("utf-8").splitlines() if line.strip()]


def _drop(rec: dict, keys: tuple = (), prefix: str | None = None,
          nested: tuple = ()) -> dict:
    out = {k: v for k, v in rec.items()
           if k not in keys and not (prefix and k.startswith(prefix))}
    for outer, inner in nested:
        if isinstance(out.get(outer), dict) and inner in out[outer]:
            out[outer] = {k: v for k, v in out[outer].items() if k != inner}
    return out


def first_field_difference(a: dict, b: dict) -> tuple[str, Any, Any] | None:
    """The first key (in `a`'s order, then `b`'s) whose JSON text differs."""
    for k in list(a) + [k for k in b if k not in a]:
        if k not in a or k not in b:
            return k, a.get(k, "<absent>"), b.get(k, "<absent>")
        if json.dumps(a[k]) != json.dumps(b[k]):
            return k, a[k], b[k]
    if list(a) != list(b):
        return "<key order>", list(a), list(b)
    return None


def max_abs_numeric(a: Any, b: Any) -> float | None:
    """The largest |a - b| over the numbers two JSON values hold at the same
    places; None when they differ in anything but numbers (shape, a string,
    a list's length)."""
    if isinstance(a, bool) or isinstance(b, bool):
        return 0.0 if a == b else None
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b))
    if isinstance(a, dict) and isinstance(b, dict):
        if list(a) != list(b):
            return None
        vals = [max_abs_numeric(a[k], b[k]) for k in a]
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return None
        vals = [max_abs_numeric(x, y) for x, y in zip(a, b)]
    else:
        return 0.0 if a == b else None
    if any(v is None for v in vals):
        return None
    return max(vals, default=0.0)


def compare_records(stored: list[dict], replayed: list[dict], *, keys: tuple = (),
                    prefix: str | None = None, nested: tuple = (),
                    label: str = "record") -> dict:
    """Two lists of JSON records, bit for bit as JSON text once the named
    fields are dropped from both. Returns {n_stored, n_replayed, identical,
    first_difference, records_differing, fields_differing}: per field the
    number of records it differs on and the largest numeric difference
    (null where it differs in anything but numbers)."""
    out = {"n_stored": len(stored), "n_replayed": len(replayed), "identical": True,
           "first_difference": None, "records_differing": 0, "fields_differing": {}}
    fields: dict[str, dict] = {}
    for i, (s, r) in enumerate(zip(stored, replayed)):
        a, b = _drop(s, keys, prefix, nested), _drop(r, keys, prefix, nested)
        if json.dumps(a) == json.dumps(b):
            continue
        out["records_differing"] += 1
        if out["identical"]:
            k, sv, rv = first_field_difference(a, b) or ("<record>", a, b)
            out.update(identical=False, first_difference={
                "index": i, label: s.get("tick", s.get("t")), "field": k,
                "stored": sv, "replayed": rv})
        for k in list(a) + [k for k in b if k not in a]:
            if k in a and k in b and json.dumps(a[k]) == json.dumps(b[k]):
                continue
            f = fields.setdefault(k, {"records": 0, "max_abs": 0.0})
            f["records"] += 1
            m = max_abs_numeric(a.get(k), b.get(k)) if k in a and k in b else None
            f["max_abs"] = None if (m is None or f["max_abs"] is None) else max(f["max_abs"], m)
    out["fields_differing"] = fields
    if len(stored) != len(replayed):
        out.update(identical=False, first_difference={
            "index": min(len(stored), len(replayed)), "field": "<count>",
            "stored": len(stored), "replayed": len(replayed)})
    return out


def tool_revision() -> str | None:
    """This tool's own commit, '-dirty' when the file differs from it."""
    try:
        head = git(ROOT, "rev-parse", "HEAD").stdout.strip()[:12]
        dirty = git(ROOT, "status", "--porcelain", "--", str(TOOL)).stdout.strip()
        return head + ("-dirty" if dirty else "")
    except Exception:                                            # noqa: BLE001
        return None


def host_facts() -> dict:
    return {"python": platform.python_version(), "machine": platform.machine(),
            "system": platform.system(), "platform": platform.platform(terse=True)}


# --------------------------------------------------------------------------- #
# finding and planning the episodes
# --------------------------------------------------------------------------- #

def find_runs(root: Path) -> list[Path]:
    """Every folder under `root` holding a manifest.json (or a replay bundle
    with one), never inside a run's bag/ or frames."""
    out: list[Path] = []
    for dp, dirs, files in os.walk(root):
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS)
        if "manifest.json" in files or any(f.endswith(".replay.tar.gz") for f in files):
            out.append(Path(dp))
    return sorted(out)


def parse_window(text: str) -> tuple[date, date]:
    a, sep, b = text.partition(":")
    if not sep:
        raise ValueError(f"--window {text!r} must be FROM:TO (ISO dates)")
    return date.fromisoformat(a), date.fromisoformat(b)


def select(runs: list[Path], windows: list[tuple], topologies: list[str]) -> list[tuple]:
    """(run, manifest, why selected) for each run in a window (the date it
    flew, flight_date) or of a named topology; every run when neither is
    given."""
    out = []
    for run in runs:
        man = read_manifest(run)
        if man is None:
            continue
        why = []
        d, _ = flight_date(run)
        for lo, hi in windows:
            if d is not None and lo <= d <= hi:
                why.append(f"window {lo.isoformat()}:{hi.isoformat()}")
        if str(man.get("topology")) in topologies:
            why.append(f"topology {man.get('topology')}")
        if why or not (windows or topologies):
            out.append((run, man, why or ["all"]))
    return out


def plan(selected: list[tuple], repo: Path, root: Path | None) -> tuple[dict, list[dict]]:
    """({full sha: [job]}, [refused record])."""
    groups: dict[str, list[dict]] = {}
    refused: list[dict] = []
    resolved: dict[str, str | None] = {}
    for run, man, why in selected:
        name = (run.relative_to(root) if root is not None else Path(run.name)).as_posix()
        d, basis = flight_date(run)
        rec = {"run": name, "dir": str(run), "selected_by": why,
               "flight_date": d.isoformat() if d else None, "flight_date_basis": basis,
               "topology": man.get("topology"),
               "code_revision_flown": man.get("code_revision"),
               "kind": ("refly" if str(man.get("topology", "")).startswith("headless")
                        else "open_loop")}
        bad = revision_refusal(man.get("code_revision"))
        if bad is None:
            rev = man["code_revision"]
            if rev not in resolved:
                resolved[rev] = resolve_commit(repo, rev)
            sha = resolved[rev]
            if sha is None:
                bad = (f"code_revision {rev!r} is not a commit of this repository "
                       f"({repo.name}): it cannot be checked out")
        if bad is not None:
            refused.append({**rec, "verdict": "refused", "why": bad})
            continue
        groups.setdefault(sha, []).append({**rec, "sha": sha})
    return groups, refused


# --------------------------------------------------------------------------- #
# the temporary worktree
# --------------------------------------------------------------------------- #

def _is_reparse(entry: os.DirEntry) -> bool:
    try:
        st = entry.stat(follow_symlinks=False)
    except OSError:
        return False
    attrs = getattr(st, "st_file_attributes", 0)
    return bool(entry.is_symlink() or attrs & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0))


def unlink_reparse_points(top: Path) -> list[str]:
    """Unlink every symlink / junction under `top` WITHOUT following it (a
    junction removed by a recursive delete takes its target's files with
    it). Returns what was unlinked."""
    gone: list[str] = []
    stack = [top]
    while stack:
        d = stack.pop()
        try:
            entries = list(os.scandir(d))
        except OSError:
            continue
        for e in entries:
            if _is_reparse(e):
                try:
                    if e.is_dir(follow_symlinks=False) or os.path.isdir(e.path):
                        os.rmdir(e.path)          # removes the link, never the target
                    else:
                        os.unlink(e.path)
                    gone.append(e.path)
                except OSError:
                    os.unlink(e.path)
                    gone.append(e.path)
            elif e.is_dir(follow_symlinks=False):
                stack.append(Path(e.path))
    return gone


class TempWorktree:
    """`git worktree add --detach <path> <sha>`, removed on exit."""

    def __init__(self, repo: Path, sha: str, base: Path):
        self.repo, self.sha = repo, sha
        self.path = base / sha[:12]
        self.record: dict = {"sha": sha, "path": str(self.path)}

    def __enter__(self) -> Path:
        t0 = time.perf_counter()
        git(self.repo, "worktree", "add", "--detach", str(self.path), self.sha)
        head = git(self.path, "rev-parse", "HEAD").stdout.strip()
        if head != self.sha:
            raise RuntimeError(f"the worktree at {self.path} is at {head}, not {self.sha}")
        self.record["created_s"] = round(time.perf_counter() - t0, 2)
        return self.path

    def __exit__(self, *exc) -> None:
        self.remove()

    def remove(self) -> None:
        links = unlink_reparse_points(self.path) if self.path.exists() else []
        r = git(self.repo, "worktree", "remove", "--force", str(self.path), check=False)
        if self.path.exists():
            # git leaves a folder it could not empty (a file still open);
            # nothing linked is left inside to follow.
            shutil.rmtree(self.path, ignore_errors=True)
        git(self.repo, "worktree", "prune", check=False)
        listed = self.sha in git(self.repo, "worktree", "list", "--porcelain",
                                 check=False).stdout and str(self.path).replace(
            "\\", "/") in git(self.repo, "worktree", "list", "--porcelain",
                              check=False).stdout.replace("\\", "/")
        self.record.update(removed=not self.path.exists() and not listed,
                           git_remove_rc=r.returncode,
                           reparse_points_unlinked=len(links))


# --------------------------------------------------------------------------- #
# running the driver
# --------------------------------------------------------------------------- #

def _chunks(items: list, n: int) -> list[list]:
    n = max(1, min(n, len(items)))
    return [items[i::n] for i in range(n)]


def run_driver(wt: Path, jobs: list[dict], scratch: Path, n_proc: int,
               python: str, dump: Path | None = None) -> list[dict]:
    """The driver over `jobs` in `n_proc` interpreters at once, each started
    isolated (-I) with the worktree as its code root (`python`: which
    interpreter; `dump`: where an open-loop episode that differs writes all
    of its differences, _dump_differences)."""
    def one(i: int, chunk: list[dict]) -> list[dict]:
        job = scratch / f"job_{wt.name}_{i}.json"
        out = scratch / f"out_{wt.name}_{i}.jsonl"
        job.write_text(json.dumps({"runs": chunk,
                                   "dump_dir": str(dump) if dump else None}),
                       encoding="utf-8")
        r = subprocess.run([python, "-I", "-X", "utf8", str(TOOL), "--driver",
                            "--code-root", str(wt), "--job", str(job), "--out", str(out)],
                           cwd=str(wt), capture_output=True, text=True, encoding="utf-8",
                           errors="replace")
        got = {}
        if out.is_file():
            for line in out.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    rec = json.loads(line)
                    got[rec["run"]] = rec
        res = []
        for j in chunk:
            if j["run"] in got:
                res.append(got[j["run"]])
            else:
                tail = (r.stderr or "").strip().splitlines()[-3:]
                res.append({**j, "verdict": "not_replayable",
                            "why": f"the driver at {j['sha'][:12]} ended (rc {r.returncode}) "
                                   f"before this episode: " + " | ".join(tail)})
        return res
    chunks = _chunks(jobs, n_proc)
    with ThreadPoolExecutor(max_workers=len(chunks)) as ex:
        parts = list(ex.map(lambda ic: one(*ic), enumerate(chunks)))
    return [r for p in parts for r in p]


def replay(selected: list[tuple], *, repo: Path, root: Path | None, scratch: Path | None,
           n_proc: int = 1, python: str | None = None, dump: Path | None = None,
           log=print) -> dict:
    """Plan, replay every group at its revision, clean up. Returns
    {records, worktrees}."""
    groups, records = plan(selected, repo, root)
    python = python or sys.executable
    worktrees = []
    base = Path(tempfile.mkdtemp(prefix="replay_at_rev_", dir=scratch))
    try:
        for sha, jobs in sorted(groups.items(), key=lambda kv: -len(kv[1])):
            wt = TempWorktree(repo, sha, base)
            t0 = time.perf_counter()
            try:
                with wt as path:
                    got = run_driver(path, jobs, base, n_proc, python, dump)
            except Exception as e:                               # noqa: BLE001
                got = [{**j, "verdict": "not_replayable",
                        "why": f"no worktree at {sha[:12]}: {type(e).__name__}: {e}"}
                       for j in jobs]
            wt.record.update(episodes=len(jobs), seconds=round(time.perf_counter() - t0, 1))
            worktrees.append(wt.record)
            records.extend(got)
            by = {v: sum(1 for g in got if g["verdict"] == v) for v in VERDICTS}
            log(f"{sha[:12]}: {len(jobs)} episode(s) {by} worktree removed "
                f"{wt.record.get('removed')}")
    finally:
        shutil.rmtree(base, ignore_errors=True)
    return {"records": sorted(records, key=lambda r: r["run"]), "worktrees": worktrees}


# --------------------------------------------------------------------------- #
# the summary
# --------------------------------------------------------------------------- #

def _reason_key(why: str) -> str:
    stems = ("flown from an unversioned tree", "the manifest records no code_revision",
             "the Shield's forecast is not recorded", "a map rule", "no flight log",
             "the pipe the Shield read", "a subject_standoff rule",
             "scheduled rules and no clock", "no policy IR", "the stored policy generations",
             "the Shield flew on a measured velocity", "rebuild_and_fly refused",
             "this revision", "a Shield configuration this replay does not rebuild")
    if why.startswith("flown from") and "-dirty" in why:
        return "flown from a '-dirty' tree"
    if "is not a commit of this repository" in why:
        return "the revision is not a commit of this repository"
    for s in stems:
        if s in why:
            return s
    return why[:90]


def _replayed_on(records: list[dict]) -> dict | list[dict]:
    """The interpreter and host the drivers ran under, as each record says
    (with --python not this process's); one dict when they all agree."""
    seen: list[dict] = []
    for r in records:
        f = r.get("replayed_on")
        if isinstance(f, dict) and f not in seen:
            seen.append(f)
    if not seen:
        return host_facts()
    return seen[0] if len(seen) == 1 else seen


def summarise(records: list[dict], worktrees: list[dict], *, command: str,
              seconds: float, groups: list[str]) -> dict:
    def counts(rs):
        return {v: sum(1 for r in rs if r["verdict"] == v) for v in VERDICTS}
    reasons: dict[str, int] = {}
    for r in records:
        if r["verdict"] in ("not_replayable", "refused"):
            k = f"{r['verdict']}: {_reason_key(r.get('why') or '')}"
            reasons[k] = reasons.get(k, 0) + 1
    by_group = {g: counts([r for r in records if g in r.get("selected_by", [])])
                for g in groups}
    by_rev: dict[str, dict] = {}
    for r in records:
        if r["verdict"] == "refused":
            continue
        k = r.get("code_revision_flown")
        e = by_rev.setdefault(k, {"kind": set(), **{v: 0 for v in VERDICTS}})
        e[r["verdict"]] += 1
        e["kind"].add(r.get("kind"))
    for e in by_rev.values():
        e["kind"] = sorted(x for x in e["kind"] if x)
    replayed = [r for r in records if r["verdict"] in ("identical", "differs")]
    return {
        "what": ("each episode replayed at the code_revision its manifest records, in a "
                 "temporary detached git worktree at that SHA (tools/replay_at_revision.py; "
                 "refly: the headless episode re-flown closed loop by that revision's "
                 "tools/rerun_episode.py; open_loop: that revision's Shield on the "
                 "episode's logged inputs)"),
        "command": command, "seconds": round(seconds, 1), "replayed_on": _replayed_on(records),
        "tool_revision": tool_revision(),
        "compared": {
            "refly": "shield_audit.jsonl records except `ts`; flight_log.jsonl bytes; "
                     "harness_events.jsonl except policy_source.path; kpi.json `kpi`; "
                     "the policy hash at t = 0",
            "open_loop": "shield_audit.jsonl records except `ts` and the node's fsm_* "
                         "fields; per flight-log row raw, violations, repairs, touched, "
                         "braked, emitted (bit for bit)"},
        "episodes": len(records), "verdicts": counts(records),
        "verdicts_by_kind": {k: counts([r for r in records if r.get("kind") == k])
                             for k in ("refly", "open_loop")},
        "verdicts_by_selection": by_group,
        "by_revision": dict(sorted(by_rev.items())),
        "not_replayed_by_reason": dict(sorted(reasons.items(), key=lambda kv: -kv[1])),
        "audit_records_compared": sum((r.get("audit") or {}).get("n_stored", 0)
                                      for r in replayed),
        "ticks_compared": sum(r.get("ticks") or 0 for r in replayed),
        "differs_by_class": {c: sum(1 for r in records if r["verdict"] == "differs"
                                    and r.get("difference_class") == c)
                             for c in sorted({r.get("difference_class") or "decision"
                                              for r in records if r["verdict"] == "differs"})},
        # Where each difference is; the values are in the per-run records.
        "differs": [{**{k: r.get(k) for k in ("run", "kind", "code_revision_flown",
                                              "difference_class")},
                     "first_difference": {k: (r.get("first_difference") or {}).get(k)
                                          for k in ("where", "tick", "field")},
                     "fields_max_abs": {f: v.get("max_abs") for f, v in (
                         (r.get("audit") or {}).get("fields_differing") or {}).items()},
                     "log_emitted_max_abs": (r.get("log") or {}).get("emitted_max_abs_diff")}
                    for r in records if r["verdict"] == "differs"],
        "worktrees": worktrees,
        "worktrees_left_behind": [w["path"] for w in worktrees if not w.get("removed")],
    }


def _compact(r: dict) -> dict:
    keep = ("run", "kind", "verdict", "why", "flight_date", "flight_date_basis", "topology",
            "selected_by",
            "code_revision_flown", "difference_class", "ticks", "audit", "log", "events",
            "kpi",
            "flight_log", "first_difference", "generations", "limits", "flown_on",
            "state_source", "installs", "dump", "decision_ticks")
    return {k: r[k] for k in keep if k in r and r[k] is not None}


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv[:1] == ["--driver"]:
        return driver_main(argv[1:])
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="*", type=Path, help="run folders (default: --root's)")
    ap.add_argument("--root", type=Path, default=None)
    ap.add_argument("--window", action="append", default=[],
                    help="FROM:TO, ISO dates: runs that flew inside (flight_date: the "
                         "episode's own stamp or first wall time) "
                         "(repeatable; with --topology, a run matching either is taken)")
    ap.add_argument("--topology", action="append", default=[],
                    help="runs whose manifest names this topology (repeatable)")
    ap.add_argument("--only", default=None, help="a substring the run path must hold")
    ap.add_argument("--repo", type=Path, default=ROOT, help="the git repository")
    ap.add_argument("--scratch", type=Path, default=None,
                    help="where the temporary worktrees go (default: the system's temp)")
    ap.add_argument("--jobs", type=int, default=1, help="driver processes per revision")
    ap.add_argument("--out", type=Path, default=None,
                    help="write the summary JSON here, and the per-episode records "
                         "beside it as <stem>.per_run.jsonl.gz")
    ap.add_argument("--json", action="store_true", help="print the summary as JSON")
    ap.add_argument("--python", type=Path, default=None,
                    help="the interpreter the replay runs under (default: this one); "
                         "each record's replayed_on says which ran it")
    ap.add_argument("--dump-dir", type=Path, default=None,
                    help="write every difference of each open-loop episode that differs "
                         "here, one JSON file per episode (the records keep the first)")
    a = ap.parse_args(argv)
    try:
        windows = [parse_window(w) for w in a.window]
    except ValueError as e:
        print(e, file=sys.stderr)
        return 2
    t0 = time.perf_counter()
    if a.runs:
        # absolute: the drivers run inside the revision's worktree, where a path
        # relative to the caller's directory names nothing (2026-11-19: every
        # run of a relative glob read 'no flight log')
        runs, root = [Path(r).resolve() for r in a.runs], None
    elif a.root is not None and a.root.is_dir():
        runs, root = find_runs(a.root), a.root
    else:
        print("give run folders or an existing --root", file=sys.stderr)
        return 2
    if a.only:
        runs = [r for r in runs if a.only in r.as_posix()]
    selected = select(runs, windows, a.topology)
    if not selected:
        print("no run with a manifest.json selected", file=sys.stderr)
        return 2
    if a.scratch is not None:
        a.scratch.mkdir(parents=True, exist_ok=True)
    if a.python is not None and not Path(a.python).is_file():
        print(f"--python {a.python}: no such interpreter", file=sys.stderr)
        return 2
    got = replay(selected, repo=a.repo, root=root, scratch=a.scratch, n_proc=a.jobs,
                 python=str(a.python) if a.python else None,
                 dump=a.dump_dir.resolve() if a.dump_dir else None)
    shown = list(argv)
    for i, x in enumerate(shown[:-1]):
        if x in ("--out", "--scratch", "--repo", "--python", "--dump-dir"):
            shown[i + 1] = f"<{x[2:]}>"
    groups = ([f"window {lo.isoformat()}:{hi.isoformat()}" for lo, hi in windows]
              + [f"topology {t}" for t in a.topology])
    summary = summarise(got["records"], got["worktrees"],
                        command=" ".join(["python", "tools/replay_at_revision.py"] + shown),
                        seconds=time.perf_counter() - t0, groups=groups)
    if a.out is not None:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        per = a.out.with_name(a.out.stem + ".per_run.jsonl.gz")
        summary["per_run_file"] = per.name
        a.out.write_text(json.dumps(summary, indent=1) + "\n", encoding="utf-8",
                         newline="\n")
        with gzip.GzipFile(per, "wb", mtime=0) as gz:
            for r in got["records"]:
                gz.write((json.dumps(_compact(r)) + "\n").encode("utf-8"))
    if a.json:
        print(json.dumps(summary, indent=1))
    else:
        print(json.dumps({k: summary[k] for k in ("episodes", "verdicts", "verdicts_by_kind",
                                                  "not_replayed_by_reason",
                                                  "worktrees_left_behind", "seconds")},
                         indent=1))
    by = summary["verdicts"]
    if by["differs"]:
        return 1
    return 0 if by["identical"] else 2


# =========================================================================== #
# THE DRIVER: runs in a separate interpreter (-I) whose import path is the
# flown revision's worktree. Everything below imports that revision's code.
# =========================================================================== #

def driver_main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--code-root", type=Path, required=True)
    ap.add_argument("--job", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    sys.dont_write_bytecode = True
    code = str(a.code_root.resolve())
    here = str(TOOL.parent)
    sys.path[:] = [p for p in sys.path if os.path.abspath(p or ".") not in
                   (here, str(ROOT), os.path.abspath("."))]
    for p in (code + os.sep + "tools", code):
        sys.path.insert(0, p)
    import importlib
    importlib.invalidate_caches()
    job = json.loads(a.job.read_text(encoding="utf-8"))
    dump = Path(job["dump_dir"]) if job.get("dump_dir") else None
    with a.out.open("w", encoding="utf-8", newline="\n") as fh:
        for j in job["runs"]:
            t0 = time.perf_counter()
            try:
                rec = (_refly(Path(j["dir"]), Path(code)) if j["kind"] == "refly"
                       else _open_loop(Path(j["dir"]), Path(code), dump=dump))
            except Refused as e:
                rec = {"verdict": "not_replayable", "why": str(e)}
            except Exception as e:                               # noqa: BLE001
                import traceback
                tb = traceback.format_exc().strip().splitlines()[-1]
                rec = {"verdict": "not_replayable",
                       "why": f"this revision's code raised on the episode: "
                              f"{type(e).__name__}: {e} ({tb})"[:600]}
            rec = {**j, **rec, "replay_s": round(time.perf_counter() - t0, 2),
                   "replayed_on": host_facts()}
            fh.write(json.dumps(rec, default=str) + "\n")
            fh.flush()
    return 0


def _module_from(code_root: Path, rel: str, name: str):
    import importlib.util
    path = code_root / rel
    if not path.is_file():
        return None
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _check_imported_from(code_root: Path, *mods) -> None:
    """Every module the replay runs must be the flown revision's file."""
    for m in mods:
        f = Path(getattr(m, "__file__", "") or "").resolve()
        if code_root.resolve() not in f.parents:
            raise RuntimeError(f"{m.__name__} was imported from {f}, not from the "
                               f"worktree {code_root}")


# --------------------------------------------------------------------------- #
# refly: a headless episode re-flown by the revision's own rerun tool
# --------------------------------------------------------------------------- #

def _refly(run: Path, code_root: Path) -> dict:
    R = _module_from(code_root, "tools/rerun_episode.py", "rerun_episode_at_revision")
    if R is None or not hasattr(R, "rebuild_and_fly"):
        raise Refused("this revision has no tools/rerun_episode.py rebuild_and_fly: it "
                      "cannot re-fly a headless episode")
    try:
        got = R.rebuild_and_fly(run)
    except getattr(R, "RerunError", Exception) as e:
        raise Refused(f"rebuild_and_fly refused the bundle: {e}") from None
    S = R.sweep()
    _check_imported_from(code_root, R, S)
    import guardrail
    _check_imported_from(code_root, guardrail)
    ep = got["run"]
    manifest = got.get("manifest") or {}
    rec: dict[str, Any] = {"rebuilt_from": got.get("rebuilt_from"),
                           "ticks": len(ep.rows),
                           "policy_hash_rerun": getattr(ep, "policy_hash_start", None)}
    diffs: list[dict] = []
    if manifest.get("policy_hash") and rec["policy_hash_rerun"] != manifest["policy_hash"]:
        rec.update(verdict="not_replayable",
                   why=f"the policy at t = 0 hashes to {rec['policy_hash_rerun']}, the "
                       f"manifest names {manifest['policy_hash']}: not the policy flown")
        return rec
    compared = []
    # shield_audit.jsonl
    p = run / "shield_audit.jsonl"
    if p.is_file():
        audit = getattr(ep, "audit", None)
        if audit is None:
            rec["audit"] = {"compared": False,
                            "why": "this revision's runner keeps no audit records"}
        else:
            stored = _jsonl_bytes(p.read_bytes())
            mine = [json.loads(json.dumps(S._json_safe(x))) for x in audit]
            c = compare_records(stored, mine, keys=AUDIT_WALL_FIELDS, label="tick")
            rec["audit"] = {"compared": True, **c}
            compared.append("shield_audit.jsonl")
            if not c["identical"]:
                diffs.append({"where": "shield_audit.jsonl", **c["first_difference"]})
    else:
        rec["audit"] = {"compared": False, "why": "the bundle stored no shield_audit.jsonl"}
    # flight_log.jsonl
    p = run / "flight_log.jsonl"
    if p.is_file():
        data = R.log_bytes(ep.rows)
        stored = p.read_bytes().replace(b"\r\n", b"\n")
        same = stored == data
        rec["flight_log"] = {"compared": True, "identical": same,
                             "stored_sha256": "sha256:" + hashlib.sha256(stored).hexdigest(),
                             "replayed_sha256": "sha256:" + hashlib.sha256(data).hexdigest()}
        compared.append("flight_log.jsonl")
        if not same:
            a_, b_ = stored.decode().splitlines(), data.decode().splitlines()
            i = next((i for i, (x, y) in enumerate(zip(a_, b_)) if x != y),
                     min(len(a_), len(b_)))
            k = None
            if i < min(len(a_), len(b_)):
                k = first_field_difference(json.loads(a_[i]), json.loads(b_[i]))
            diffs.append({"where": "flight_log.jsonl", "index": i,
                          "field": k[0] if k else "<count>",
                          "stored": k[1] if k else len(a_), "replayed": k[2] if k else len(b_)})
    else:
        rec["flight_log"] = {"compared": False, "why": "the bundle kept no flight_log.jsonl"}
    # harness_events.jsonl
    p = run / "harness_events.jsonl"
    if p.is_file():
        stored = _jsonl_bytes(p.read_bytes())
        mine = [json.loads(json.dumps(S._json_safe(e))) for e in ep.events]
        c = compare_records(stored, mine, nested=EVENT_HOST_FIELDS, label="t")
        rec["events"] = {"compared": True, **c}
        compared.append("harness_events.jsonl")
        if not c["identical"]:
            diffs.append({"where": "harness_events.jsonl", **c["first_difference"]})
    # kpi.json's table, re-scored by the revision's own score()
    doc = _json_file(run / "kpi.json") or {}
    if isinstance(doc.get("kpi"), dict) and hasattr(S, "score"):
        spec = got.get("spec")
        if spec is None:
            from guardrail.scenario_spec import ScenarioSpec
            extras = _json_file(run / "manifest_extras.json") or {}
            raw = extras.get("scenario_spec") or {}
            import guardrail.scenario_spec as SS
            if hasattr(SS, "legacy_pins_read_as_before"):
                raw = SS.legacy_pins_read_as_before(raw)
            spec = ScenarioSpec.model_validate(raw)
        sspec = spec
        if hasattr(S, "scoring_spec") and getattr(got.get("spec"), "events", None) is None:
            pass
        res = S.score(sspec, ep.rows, ep.extra, ep.policy, ep.priorities)
        mine = json.loads(json.dumps(S._json_safe(res)))
        c = compare_records([doc["kpi"]], [mine], label="t")
        rec["kpi"] = {"compared": True, "identical": c["identical"],
                      "first_difference": c["first_difference"]}
        compared.append("kpi.json kpi")
        if not c["identical"]:
            diffs.append({"where": "kpi.json kpi", **c["first_difference"]})
    if not compared:
        raise Refused("the bundle holds nothing the re-flight can be compared with")
    rec["compared"] = compared
    rec["verdict"] = "differs" if diffs else "identical"
    rec["first_difference"] = diffs[0] if diffs else None
    rec["differences"] = diffs
    return rec


# --------------------------------------------------------------------------- #
# open_loop: the revision's Shield on a flown episode's logged inputs
# --------------------------------------------------------------------------- #

def _vec(d: Any) -> list | None:
    if not isinstance(d, dict):
        return None
    try:
        return [float(d[k]) for k in VEC]
    except (KeyError, TypeError, ValueError):
        return None


def _pipe_note(d: Any) -> dict | None:
    """A row's PIPE_ROW_KEY field as the replay feeds it back, or None when
    the row has none or it is malformed (2026-11-21): {t, vx, vy, pipe_reads},
    t and every reading a finite number, vx and vy both numbers (what was
    sent) or both None (nothing sent, or a non-finite setpoint, which
    Shield.note_flown records alike)."""
    def num(v: Any) -> bool:
        return (isinstance(v, (int, float)) and not isinstance(v, bool)
                and v == v and v not in (float("inf"), float("-inf")))
    if not isinstance(d, dict) or not num(d.get("t")):
        return None
    vx, vy, reads = d.get("vx"), d.get("vy"), d.get("pipe_reads")
    if not ((vx is None and vy is None) or (num(vx) and num(vy))):
        return None
    if not isinstance(reads, list) or not all(num(v) for v in reads):
        return None
    return {"t": float(d["t"]), "vx": None if vx is None else float(vx),
            "vy": None if vy is None else float(vy),
            "pipe_reads": [float(v) for v in reads]}


def _flown_on(metrics: dict) -> dict | None:
    """The host the Shield node ran on, as the run's hil_evidence says, and
    the interpreter it ran under where the run recorded it (metrics.json
    gc.python, written by the node's GC pacing since 2026-11-10; `python`
    absent on an earlier run, never guessed). Both are needed to read a
    last-digit difference: the host's libm and the interpreter's own float
    routines (Python 3.12 changed sum()) are separate candidates."""
    out: dict = {}
    ev = metrics.get("hil_evidence")
    if isinstance(ev, dict):
        out = {k: ev[k] for k in ("host_arch", "device_model", "l4t_release", "jetpack",
                                  "hostname") if isinstance(ev.get(k), (str, int, float))}
    gc = metrics.get("gc")
    if isinstance(gc, dict) and isinstance(gc.get("python"), str):
        out["python"] = gc["python"]
    return out or None


def _finite(v: Any) -> bool:
    return (isinstance(v, (int, float)) and not isinstance(v, bool)
            and v == v and v not in (float("inf"), float("-inf")))


def perceived_poses(rows: list[dict], pos: list[tuple], metrics: dict,
                    pos_exact: bool) -> dict | None:
    """The pose the Shield was handed on each row of a run flown under the
    ROS 2 node's perception stressors (sitl/ros2_shield_node.py
    PerceptionStress, --gps-noise-m / --latency-ticks, WP3-19, since
    2026-11-21), or None for a run without them (no row logs `shield_pose`
    and metrics.json records no perception_stress applied): the replay then
    hands the Shield the true pose, as before.

    The node hands its Shield, per tick, the true pose of latency_ticks rows
    ago (the first until the line fills) plus white Gaussian noise of sigma
    gps_noise_m on x, y and up drawn from random.Random(f"{seed}:gps") in
    that order, the CURRENT heading, and pos_sigma_m = sigma (None at sigma
    0); the row logs that pose rounded to 0.1 mm (`shield_pose`) and
    `pos_sigma_m`. The doubles the Shield read are rebuilt here as the node
    built them, from the positions' own doubles (`pos`, trajectory.json:
    `pos_exact`) and metrics.json perception_stress (sigma, latency, seed,
    the generator and draw order it names), and used only when every row's
    rebuilt pose rounds to the row's shield_pose; otherwise the rows'
    shield_pose itself is handed on (0.1 mm, a limit says so). Standard
    library only: the driver runs this under the flown revision's path.

    Returns {"poses": [(x, y, up)], "pos_sigma_m": [float | None],
    "source", "record", "limits"}; raises Refused when the record is
    incomplete (shield_pose on some rows only, or perception_stress applied
    and no row logs it)."""
    import random
    ps = metrics.get("perception_stress")
    applied = isinstance(ps, dict) and ps.get("applied") is True
    logged = [r.get("shield_pose") for r in rows]
    has = [isinstance(p, list) and len(p) == 3 and all(_finite(v) for v in p)
           for p in logged]
    if not applied and not any("shield_pose" in r for r in rows):
        return None
    if not all(has):
        raise Refused(f"the pose the Shield judged is not logged on every row: {sum(has)} of "
                      f"{len(rows)} row(s) carry a shield_pose"
                      + (" and metrics.json perception_stress says it was applied"
                         if applied else ""))
    sigmas: list = []
    for r in rows:
        s = r.get("pos_sigma_m") if "pos_sigma_m" in r else (
            ps.get("pos_sigma_m") if applied else None)
        if s is not None and not (_finite(s) and s > 0):
            raise Refused(f"tick {r.get('tick')} logs a pos_sigma_m of {s!r} beside its "
                          f"shield_pose")
        sigmas.append(None if s is None else float(s))
    limits: list[str] = []
    record: dict = {"applied": applied}
    poses = None
    why = None
    if not applied:
        why = "metrics.json records no perception_stress applied (no seed to rebuild from)"
    elif not pos_exact:
        why = "the positions are the log's 1 mm (no trajectory.json with one point per row)"
    else:
        sigma, lat, seed = ps.get("gps_noise_m"), ps.get("latency_ticks"), ps.get("seed")
        record.update(gps_noise_m=sigma, latency_ticks=lat, seed=seed)
        if not (_finite(sigma) and sigma >= 0 and isinstance(lat, int)
                and not isinstance(lat, bool) and lat >= 0
                and isinstance(seed, int) and not isinstance(seed, bool)):
            why = (f"metrics.json perception_stress does not say sigma, latency and seed "
                   f"({sigma!r}, {lat!r}, {seed!r})")
        elif ps.get("rng", PERCEPTION_RNG) != PERCEPTION_RNG or \
                list(ps.get("draw_order", PERCEPTION_DRAW_ORDER)) != PERCEPTION_DRAW_ORDER:
            why = (f"metrics.json perception_stress names another generator or draw order "
                   f"({ps.get('rng')!r}, {ps.get('draw_order')!r})")
        else:
            s = float(sigma)
            rng = random.Random(f"{seed}:gps")
            cand = []
            for i in range(len(rows)):
                ox, oy, ou = pos[max(0, i - lat)]
                # The node's order: x, then y, then up; nothing drawn at 0.
                nx = rng.gauss(0, s) if s else 0.0
                ny = rng.gauss(0, s) if s else 0.0
                nu = rng.gauss(0, s) if s else 0.0
                cand.append((ox + nx, oy + ny, ou + nu))
            bad = next((i for i, (c, p) in enumerate(zip(cand, logged))
                        if [round(v, SHIELD_POSE_ROUND) for v in c] != p), None)
            if bad is None:
                poses = cand
            else:
                why = (f"the pose rebuilt from trajectory.json and the seed does not round to "
                       f"row {bad}'s shield_pose (tick {rows[bad].get('tick')}: "
                       f"{[round(v, SHIELD_POSE_ROUND) for v in cand[bad]]} vs {logged[bad]})")
    if poses is not None:
        source = ("rebuilt as the node built it (trajectory.json's doubles, the recorded "
                  "seed, sigma and latency), every row rounding to its shield_pose")
    else:
        poses = [(float(p[0]), float(p[1]), float(p[2])) for p in logged]
        source = "the rows' shield_pose (rounded to 0.1 mm)"
        limits.append(f"the pose the Shield was handed is the rows' shield_pose, rounded to "
                      f"0.1 mm, not the doubles it read: {why}")
    record["shield_pose"] = source
    return {"poses": poses, "pos_sigma_m": sigmas, "source": source, "record": record,
            "limits": limits}


def _replayed_ground(run: Path, code_root: Path, grec: dict, policy, params) -> dict:
    """The Shield's ground keywords (`ground`, `up_ref`) of a run that flew over
    a declared ground (metrics.json `ground`), rebuilt from the file it named.
    Refused when the file is missing, hashes to another sha256, or does not
    rebuild the grid the run recorded."""
    import hashlib
    if "ground" not in params:
        raise Refused("this revision's Shield takes no ground, and the run flew over one "
                      "(metrics.json ground)")
    try:
        from guardrail import terrain as T
    except ImportError:
        raise Refused("this revision has no guardrail.terrain, and the run flew over a "
                      "ground (metrics.json ground)") from None
    rec = grec.get("ground") if isinstance(grec.get("ground"), dict) else {}
    frec = grec.get("file") if isinstance(grec.get("file"), dict) else None
    if rec.get("kind") == "flat" and frec is None:
        ground = T.FlatGround(rec["elevation_msl_m"], rec.get("source") or "metrics.json")
    else:
        if not frec or not frec.get("path") or not frec.get("sha256"):
            raise Refused("the run flew over a ground and metrics.json names no ground file "
                          "(ground.file path and sha256)")
        name = Path(str(frec["path"])).name
        tar = _replay_tar(run)
        data = _tar_member(tar, "ground/" + name) if tar is not None else None
        src = f"{tar.name} (ground/{name})" if data is not None else None
        if data is None:
            p = code_root / str(frec["path"])
            if p.is_file():
                data, src = p.read_bytes(), f"the revision's {frec['path']}"
        if data is None:
            raise Refused(f"the ground file {frec['path']} is neither in the replay bundle "
                          f"nor at this revision")
        got = hashlib.sha256(data).hexdigest()
        if got != frec["sha256"]:
            raise Refused(f"the ground file from {src} hashes to {got}, the run named "
                          f"{frec['sha256']}")
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / name
            p.write_bytes(data)
            ground = T.load_ground(p, getattr(policy, "frame_origin", None))
        if rec.get("sha256"):
            built = ground.record().get("sha256") if hasattr(ground, "record") else None
            if built != rec["sha256"]:
                raise Refused(f"the ground rebuilt from {src} is grid {built}, the run "
                              f"recorded {rec['sha256']}")
    kw = {"ground": ground}
    up_ref = grec.get("up_ref")
    if up_ref is not None:
        if "up_ref" not in params:
            raise Refused("this revision's Shield takes no up_ref, and the run recorded one")
        kw["up_ref"] = up_ref
    return kw


def _open_loop(run: Path, code_root: Path, dump: Path | None = None) -> dict:
    import inspect
    from datetime import datetime as _dt
    try:
        from guardrail import replay as RP
        from guardrail.audit import AuditLogger
        from guardrail.models import Action4D, State
        from guardrail.shield import Shield
    except ImportError as e:
        what = getattr(e, "name", None) or str(e).split(" (")[0]
        raise Refused(f"this revision cannot import the Shield's replay pieces "
                      f"({what}: not at this revision)") from None
    _check_imported_from(code_root, RP, sys.modules["guardrail.shield"])
    rail_shield = getattr(RP, "rail_shield", None)
    limits: list[str] = []

    data, src_rows = read_run_file(run, "flight_log.jsonl")
    if data is None:
        raise Refused("no flight log (no flight_log.jsonl and no replay bundle holding one)")
    try:
        rows = _jsonl_bytes(data)
    except ValueError as e:
        raise Refused(f"flight_log.jsonl does not parse ({e})") from None
    if not rows:
        raise Refused("no flight log (the log holds no row)")
    mdata, _ = read_run_file(run, "metrics.json")
    metrics = json.loads(mdata) if mdata else {}
    la = metrics.get("shield_lookahead")
    if not isinstance(la, dict) or not all(isinstance(la.get(k), (int, float))
                                           for k in ("horizon_s", "dt_s")):
        raise Refused("the Shield's forecast is not recorded (no metrics.json "
                      "`shield_lookahead` with horizon_s and dt_s)")
    fc = metrics.get("shield_forecast") if isinstance(metrics.get("shield_forecast"),
                                                      dict) else None
    if fc is not None:
        if str(fc.get("mode", "commanded")) != "commanded":
            raise Refused(f"the Shield flew on a measured velocity (shield_forecast mode "
                          f"{fc.get('mode')!r}): the pipe the Shield read is not logged "
                          f"per tick")
        used = {k: fc[k] for k in VELOCITY_USED_KEYS
                if isinstance(fc.get(k), (int, float)) and fc[k] > 0}
        if used:
            raise Refused(f"the Shield flew on a measured velocity ({used}): the pipe the "
                          f"Shield read is not logged per tick")
        unknown = sorted(k for k, v in fc.items() if k not in KNOWN_FORECAST_KEYS
                         and v not in (None, False, 0, "", [], {}))
        if unknown:
            raise Refused(f"a Shield configuration this replay does not rebuild "
                          f"(metrics.json shield_forecast: {', '.join(unknown)})")
    elif any(r.get("shield_velocity") for r in rows) or metrics.get("shield_velocity_source"):
        raise Refused("the pipe the Shield read is not logged per tick: the rows say the "
                      "Shield was given a measured velocity (`shield_velocity`) and no "
                      "metrics.json shield_forecast says otherwise")
    nofix = sum(1 for r in rows if r.get("fix_ok") is False)
    noest = sum(1 for r in rows if r.get("estimator_ok") is False)
    # The pipe the Shield read, when every row logs it (PIPE_ROW_KEY, the
    # ROS 2 node since 2026-11-21): fed back tick by tick, so a lost fix or
    # estimator replays like the rest. A run without it on every row keeps
    # the refusal below, word for word.
    notes = [_pipe_note(r.get(PIPE_ROW_KEY)) for r in rows]
    piped = all(n is not None for n in notes)
    if (nofix or noest) and not piped:
        raise Refused(f"the pipe the Shield read is not logged per tick: {nofix} row(s) "
                      f"with no valid fix and {noest} with the estimator unhealthy, where "
                      f"the Shield dead-reckons on what the rail sent (note_flown, on a "
                      f"real-time pipe clock no rail logs)")

    # every generation's policy
    policies: dict[int, Any] = {}
    files = RP._generation_files(run) if hasattr(RP, "_generation_files") else {}
    if not files:
        tar = _replay_tar(run)
        if tar is not None:
            try:
                with tarfile.open(tar, "r:gz") as tf:
                    for m in tf.getmembers():
                        nm = m.name.rsplit("/", 1)[-1]
                        if m.name.startswith("episode/") and m.isfile() and \
                                RP._is_generation_file(nm):
                            files[nm] = tf.extractfile(m).read()
            except (OSError, tarfile.TarError, AttributeError):
                files = {}
    if files and hasattr(RP, "generation_table"):
        _, policies, problems = RP.generation_table(files)
        if problems:
            raise Refused("the stored policy generations do not agree: " + "; ".join(problems))
    if not policies:
        raise Refused("no policy IR stored with the episode (no policy_g<N>.json in the "
                      "folder or its replay bundle)")
    rules = [c for p in policies.values() for c in p.constraints]
    types = {c.type for c in rules}
    maps = sorted(types & set(MAP_RULES))
    if maps:
        raise Refused(f"a map rule ({', '.join(maps)}): the map the flight gave the Shield "
                      f"is not stored with the episode")
    # A subject: replayable when the rail DECLARED it (ground truth, set once
    # with its class before the first tick, as the ROS 2 node does with
    # --subject) and metrics.json records that class and place, which every
    # row's tgt_x / tgt_y repeats. A subject that moved (perception) is not.
    subject = None
    if types & set(SUBJECT_RULES) and any(r.get("tgt_x") is not None or r.get("subject")
                                          for r in rows):
        ms = metrics.get("subject") if isinstance(metrics.get("subject"), dict) else {}
        declared = (ms.get("position_source") == "declared"
                    and isinstance(ms.get("class"), str)
                    and all(isinstance(ms.get(k), (int, float)) for k in ("x", "y")))
        if not declared or not all(r.get("tgt_x") == ms["x"] and r.get("tgt_y") == ms["y"]
                                   and not r.get("subject") for r in rows):
            raise Refused("a subject_standoff rule with a subject on the rows, and the class "
                          "the rail gave Shield.set_subject is not recorded (metrics.json "
                          "has no declared subject that every row repeats)")
        subject = (float(ms["x"]), float(ms["y"]), ms["class"])
    clocks = None
    if all(isinstance(r.get("clock"), str) for r in rows):
        clocks = [_dt.fromisoformat(r["clock"]) for r in rows]
    elif any(getattr(c, "valid_time", None) is not None for c in rules):
        raise Refused("the policy has scheduled rules (valid_time) and no rule-clock reading "
                      "per tick: scheduled rules and no clock")

    def gen_of(r: dict) -> int:
        g = r.get("generation")
        if isinstance(g, int) and not isinstance(g, bool):
            if g not in policies:
                raise Refused(f"tick {r.get('tick')} flew generation {g}, whose policy is "
                              f"not stored: no policy IR")
            return g
        h = r.get("policy_hash")
        if h:
            for n, p in sorted(policies.items()):
                if p.matches_hash(h):
                    return n
            raise Refused(f"tick {r.get('tick')} names policy {h}, which no stored "
                          f"generation has: no policy IR")
        if len(policies) == 1:
            return next(iter(policies))
        raise Refused("the rows name no generation and several are stored: no policy IR")
    gens = [gen_of(r) for r in rows]
    for r in rows:
        if not all(isinstance(r.get(k), (int, float)) for k in ("x", "y", "up")) or \
                _vec(r.get("raw")) is None:
            raise Refused(f"tick {r.get('tick')} logs no position or no world-frame raw "
                          f"action")

    # the stored audit
    adata, asrc = read_run_file(run, "shield_audit.jsonl")
    if adata is None:
        adata, asrc = read_run_file(run, "audit.jsonl")
    stored_audit = _jsonl_bytes(adata) if adata is not None else []
    kinds = [a for a in stored_audit if a.get("decision_kind")]
    plain = [a for a in stored_audit if not a.get("decision_kind")]
    if kinds:
        limits.append(f"{len(kinds)} audit record(s) of a decision the rail flew instead "
                      f"(decision_kind) are not re-derived")
    by_tick: dict[int, dict] = {}
    for a in plain:
        if a.get("tick") in by_tick:
            raise Refused(f"two audit records for tick {a.get('tick')}")
        by_tick[a.get("tick")] = a

    # the Shield as the rail built it
    cfg = {"lookahead_s": float(la["horizon_s"]), "dt": float(la["dt_s"]),
           "escalation": bool(la.get("escalation_fsm", False))}
    params = inspect.signature(Shield).parameters
    kw: dict[str, Any] = {}
    # PathRepair: as recorded (the ROS 2 node records it since 2026-11-17);
    # a run that records nothing flew this revision's default, which is what
    # its rail passed (nothing), so nothing is passed here either.
    pr = (fc or {}).get("path_repair")
    if isinstance(pr, bool):
        if "path_repair" in params:
            kw["path_repair"] = pr
        elif pr:
            raise Refused("a Shield configuration this replay does not rebuild (path_repair "
                          "recorded on, and this revision's Shield takes no such argument)")
    # The clearance horizon's vehicle response (Shield clear_response, the
    # rails record it since 2026-11-19): rebuilt as recorded; a run that
    # records none flew the instant vehicle, which is this revision's default.
    cr = (fc or {}).get("clear_response")
    if cr not in (None, {}):
        if "clear_response" not in params:
            raise Refused("a Shield configuration this replay does not rebuild "
                          "(clear_response recorded, and this revision's Shield takes no "
                          "such argument)")
        try:
            from guardrail.response import VelocityResponse
            kw["clear_response"] = VelocityResponse(**cr)
        except (ImportError, TypeError, ValueError) as e:
            raise Refused(f"a Shield configuration this replay does not rebuild "
                          f"(metrics.json shield_forecast clear_response {cr!r}: {e})")
    # The declared ground (X-11, 2026-11-25): a run over a ground (metrics.json
    # `ground`, the rails record it since 2026-11-18) is rebuilt over the same
    # ground, or its MSL rules refuse the Shield. The ground file is the one
    # the run named (`ground.file`: path and sha256): the replay bundle's
    # `ground/<name>`, else the revision's own copy, refused unless it hashes
    # to the recorded sha256; it is placed in the frame of the stored policy,
    # as the rail placed it (terrain.load_ground), and the result must be the
    # grid the run recorded (`ground.ground` sha256), else Refused.
    grec = metrics.get("ground") if isinstance(metrics.get("ground"), dict) else None
    if grec is not None:
        kw.update(_replayed_ground(run, code_root, grec, policies[gens[0]], params))
        cfg["ground"] = {"file": grec.get("file"), "up_ref": grec.get("up_ref")}
    pin: list = [None]
    clock = (lambda: pin[0]) if clocks else None
    # The pipe clock, pinned (2026-11-21): each tick's filter is handed the
    # readings its row logs, in order; a read beyond them (`short`) or one
    # left unread is a difference (`pipe_reads`).
    pipe_q: list[float] = []
    pipe_st = {"short": 0, "last": 0.0}

    def pipe_clock() -> float:
        if pipe_q:
            pipe_st["last"] = pipe_q.pop(0)
            return pipe_st["last"]
        pipe_st["short"] += 1
        return pipe_st["last"]
    if piped:
        if "pipe_clock" not in params or not hasattr(Shield, "note_flown"):
            raise Refused("the rows log the pipe the Shield read (note_flown), and this "
                          "revision's Shield takes no pipe_clock or has no note_flown")
        kw["pipe_clock"] = pipe_clock
    if cfg["escalation"] or rail_shield is None:
        if clock is not None:
            kw["now"] = clock
        shield = Shield(policies[gens[0]], lookahead_s=cfg["lookahead_s"], dt=cfg["dt"], **kw)
    else:
        shield = rail_shield(policies[gens[0]], lookahead_s=cfg["lookahead_s"], dt=cfg["dt"],
                             clock=clock, **kw)
    cfg["path_repair"] = getattr(shield, "path_repair", None)
    cfg["path_repair_from"] = "metrics.json" if isinstance(pr, bool) else "revision default"
    cfg["clear_response"] = cr
    cfg["pipe"] = (f"fed from the rows' {PIPE_ROW_KEY} (what the rail sent, the pipe "
                   f"clock's readings)" if piped else "none")
    if subject is not None:
        if not hasattr(shield, "set_subject"):
            raise Refused("this revision's Shield has no set_subject")
        shield.set_subject(subject[0], subject[1], subject[2])
    state_fields = getattr(State, "model_fields", {})
    installs: list[dict] = []

    def install(target) -> dict:
        """Commit stored generation `target` on the Shield as a hot-apply
        does: the rules replaced, every dynamic_nfz it adds or changes
        anchored at the Shield's own event time (its last tick, which is
        what the node's events get: shield_t=None), the others keeping
        theirs. Returns what was done, for the record."""
        for need in ("_commit", "_event_time", "_anchors"):
            if not hasattr(shield, need):
                raise Refused(f"this revision's Shield has no {need}: a hot-applied "
                              f"generation cannot be installed as it committed one")
        cur = {c.id: c for c in shield.policy.constraints}
        keep = {c.id for c in target.constraints}
        anchors = {k: v for k, v in shield._anchors.items() if k in keep}
        te = shield._event_time(None)
        moved = []
        for c in target.constraints:
            if c.type == "dynamic_nfz" and (c.id not in cur or cur[c.id].model_dump(
                    mode="json") != c.model_dump(mode="json")):
                anchors[c.id] = te
                moved.append(c.id)
        shield.policy.generation = target.generation - 1
        shield._commit(list(target.constraints), "replay_generation", "-", "-", None, anchors)
        if not target.matches_hash(shield.policy.policy_hash):
            raise Refused(f"generation {target.generation} rebuilt in this revision's Shield "
                          f"hashes to {shield.policy.policy_hash}, not the stored "
                          f"{target.policy_hash}")
        return {"generation": target.generation, "event_time": te,
                "anchored": {k: anchors[k] for k in moved}}

    def install_through(frm: int, to: int, i: int, tick) -> None:
        """Rows that go from generation `frm` to `to` in one tick (two hot
        applies between ticks: a spawn and a rotate at one mission time)
        are installed through every stored generation between, in order,
        as the node committed them (until 2026-11-18 the replay committed
        `to` alone, straight from `frm`)."""
        chain = [n for n in sorted(policies) if frm < n <= to] if to > frm else [to]
        if not chain or chain[-1] != to:
            chain = [to]
        gaps = [n for n in range(frm + 1, to) if n not in policies] if to > frm else []
        if gaps:
            limits.append(f"row {i} (tick {tick}): generation(s) {gaps} between {frm} and "
                          f"{to} are not stored, so {to} was committed without them")
        installs.append({"row": i, "tick": tick, "from": frm, "to": to,
                         "commits": [install(policies[n]) for n in chain]})

    # positions: trajectory.json's own doubles when it has one point per row
    tdata, _ = read_run_file(run, "trajectory.json")
    traj = json.loads(tdata) if tdata else None
    pos = None
    if isinstance(traj, list) and len(traj) == len(rows):
        pos = []
        for r, p in zip(rows, traj):
            try:
                got = (float(p["x"]), float(p["y"]), float(p["up"]))
            except (KeyError, TypeError, ValueError):
                pos = None
                break
            if not all(abs(a - float(r[k])) <= POS_MATCH_M for a, k in zip(got, ("x", "y", "up"))):
                pos = None
                break
            pos.append(got)
    state_source = "trajectory.json (the position's own doubles)" if pos else \
        "flight_log.jsonl (rounded to 1 mm)"
    pos_exact = pos is not None
    if pos is None:
        pos = [(float(r["x"]), float(r["y"]), float(r["up"])) for r in rows]
        limits.append("positions are the flight log's, rounded to 1 mm (no trajectory.json "
                      "with one point per row)")
    # The pose the Shield was handed under the node's perception stressors
    # (GPS noise / latency, since 2026-11-21): the Shield judged that pose,
    # not the true one the rows and trajectory.json carry.
    perceived = perceived_poses(rows, pos, metrics, pos_exact)
    if perceived is not None:
        limits.extend(perceived["limits"])
        if any(s is not None for s in perceived["pos_sigma_m"]) and \
                "pos_sigma_m" not in state_fields:
            raise Refused("the rows log the pos_sigma_m the Shield was handed, and this "
                          "revision's State has no pos_sigma_m")
        state_source = f"{perceived['source']} (the perception stressors' pose)"

    arm = str(metrics.get("shield") or "on")
    shield_on = arm != "off"
    ep_id = next((a.get("episode_id") for a in plain if a.get("episode_id") is not None), None)
    if hasattr(AuditLogger, "in_memory"):
        logger = AuditLogger.in_memory(shield.policy, episode_id=ep_id)
    else:
        raise Refused("this revision's AuditLogger has no in_memory(): its records cannot "
                      "be rebuilt without writing a file")
    line = getattr(AuditLogger, "line", None)
    rec_params = inspect.signature(logger.record).parameters
    if "fsm_record" not in rec_params:
        raise Refused("this revision's AuditLogger.record takes no fsm_record")

    current = gens[0]
    log_counts: dict[str, int] = {}
    emitted_max = 0.0
    not_logged: dict[str, int] = {}
    first_log = None
    replayed_audit: list[dict] = []
    stored_in_order: list[dict] = []
    touched_without_record: list = []
    rows_ticks = set()
    yaw_exact = 0
    log_diff_ticks: list[dict] = []
    emitted_by_tick: dict = {}
    log_decision: dict = {}
    for i, row in enumerate(rows):
        if gens[i] != current:
            install_through(current, gens[i], i, row.get("tick"))
            current = gens[i]
        if clocks:
            pin[0] = clocks[i]
        tick = row.get("tick")
        rows_ticks.add(tick)
        stored = by_tick.get(tick)
        yaw = row.get("yaw_deg") or 0.0
        if stored is not None and isinstance(stored.get("body_yaw_deg"), (int, float)):
            yaw = stored["body_yaw_deg"]
            yaw_exact += 1
        if perceived is not None:
            # As the node built it: the perceived position, the current
            # heading, the sigma it reported (None: not reported).
            px, py, pu = perceived["poses"][i]
            sk = {"x": px, "y": py, "up": pu, "yaw_deg": float(yaw)}
            if perceived["pos_sigma_m"][i] is not None:
                sk["pos_sigma_m"] = perceived["pos_sigma_m"][i]
        else:
            sk = {"x": pos[i][0], "y": pos[i][1], "up": pos[i][2], "yaw_deg": float(yaw)}
        for k in ("fix_ok", "estimator_ok"):
            if k in state_fields and isinstance(row.get(k), bool):
                sk[k] = row[k]
        if piped:
            pipe_q[:] = notes[i]["pipe_reads"]
            short0 = pipe_st["short"]
        dec = shield.filter(State(**sk), Action4D(**{k: float(row["raw"][k]) for k in VEC}))
        # the flight log's Shield fields
        diffs = []
        if piped:
            # The pipe: the readings this tick's filter took against the
            # row's, then what the rail sent, noted as the rail noted it
            # (after the tick, at its pipe time).
            n_short, n_left = pipe_st["short"] - short0, len(pipe_q)
            if n_short or n_left:
                n_log = len(notes[i]["pipe_reads"])
                diffs.append(("pipe_reads", n_log, n_log - n_left + n_short))
            pipe_q.clear()
            nf = notes[i]
            shield.note_flown(None if nf["vx"] is None else
                              Action4D(vx=nf["vx"], vy=nf["vy"]), t=nf["t"])
            if "position_lost" in row:
                # X8's input, the Shield's verdict on the position it was
                # handed: the dead reckoning's output.
                lost = getattr(dec, "position_lost", None)
                got_lost = lost[0] if lost else None
                if row["position_lost"] != got_lost:
                    diffs.append(("position_lost", row["position_lost"], got_lost))
        got_raw = [float(getattr(dec.raw, k)) for k in VEC]
        if _vec(row.get("raw")) != got_raw:
            diffs.append(("raw", row.get("raw"), dict(zip(VEC, got_raw))))
        ids = [v.rule_id for v in dec.violations]
        if ids != [v.get("rule_id") for v in row.get("violations") or []]:
            diffs.append(("violations", [v.get("rule_id") for v in row.get("violations") or []],
                          ids))
        if shield_on:
            ops = [r.operator for r in dec.repairs]
            if ops != [r.get("operator") for r in row.get("repairs") or []]:
                diffs.append(("repairs", [r.get("operator") for r in row.get("repairs") or []],
                              ops))
            for nm in ("touched", "braked"):
                if nm not in row:
                    not_logged[nm] = not_logged.get(nm, 0) + 1
                elif bool(row[nm]) != bool(getattr(dec, nm)):
                    diffs.append((nm, row[nm], bool(getattr(dec, nm))))
            em = [float(getattr(dec.emitted, k)) for k in VEC]
            lv = _vec(row.get("emitted"))
            if lv != em:
                diffs.append(("emitted", row.get("emitted"), dict(zip(VEC, em))))
                if lv is not None:
                    emitted_max = max(emitted_max, max(abs(a - b) for a, b in zip(lv, em)))
                    emitted_by_tick[tick] = max(abs(a - b) for a, b in zip(lv, em))
                else:
                    emitted_by_tick[tick] = None
        for f, lv, gv in diffs:
            if f != "emitted":
                log_decision.setdefault(tick, []).append(f)
            log_counts[f] = log_counts.get(f, 0) + 1
            if first_log is None:
                first_log = {"where": "flight_log.jsonl", "tick": tick, "row": i, "field": f,
                             "stored": lv, "replayed": gv}
        if diffs and dump is not None:
            log_diff_ticks.append({"row": i, "tick": tick, "generation": gens[i],
                                   "fields": [{"field": f, "stored": lv, "replayed": gv}
                                              for f, lv, gv in diffs]})
        # the audit record, the Shield's part
        if stored is not None or dec.touched:
            r = logger.record(tick, dec, fsm_record={"transition": "replay"})
            mine = line(r) if line is not None else r.model_dump(by_alias=True)
            mine = json.loads(json.dumps(mine, default=str))
            if stored is None:
                touched_without_record.append(tick)
            else:
                replayed_audit.append(mine)
                stored_in_order.append(stored)
    audit = compare_records(stored_in_order, replayed_audit, keys=AUDIT_WALL_FIELDS,
                            prefix=NODE_FSM_PREFIX, label="tick")
    orphans = sorted(t for t in by_tick if t not in rows_ticks)
    audit.update(n_stored=len(plain), source=asrc, records_with_kind=len(kinds),
                 touched_ticks_without_record=len(touched_without_record),
                 records_without_row=len(orphans),
                 excluded_fields=list(AUDIT_WALL_FIELDS) + [NODE_FSM_PREFIX + "*"])
    diffs_all = []
    if not audit["identical"]:
        diffs_all.append({"where": "shield_audit.jsonl", **audit["first_difference"]})
    if touched_without_record:
        diffs_all.append({"where": "shield_audit.jsonl", "tick": touched_without_record[0],
                          "field": "<record>", "stored": None,
                          "replayed": "a touched decision with no stored record"})
    if orphans:
        diffs_all.append({"where": "shield_audit.jsonl", "tick": orphans[0],
                          "field": "<row>", "stored": "a record", "replayed": "no flight-log row"})
    if first_log:
        diffs_all.append(first_log)
    cls = None
    if diffs_all:
        fd = audit.get("fields_differing") or {}
        if (not first_log and not touched_without_record and not orphans and fd
                and set(fd) <= set(BODY_FRAME_FIELDS)
                and all(f.get("max_abs") is not None and f["max_abs"] <= BODY_FRAME_ULP_TOL
                        for f in fd.values())):
            # Same world-frame decision, same heading (both compared and
            # equal), same code: only the writer's turn into the body frame
            # (cos / sin of the heading) differs, in the last digits.
            cls = "body_frame_rotation_last_digits"
        elif (not touched_without_record and not orphans
              and all(f.get("max_abs") is not None and f["max_abs"] <= FLOAT_LAST_DIGITS_TOL
                      for f in fd.values())
              and set(log_counts) <= {"emitted"} and emitted_max <= FLOAT_LAST_DIGITS_TOL):
            # The same rules violated, the same operators, the same touched /
            # braked, every string equal: only floats differ, by at most the
            # tolerance (reported per field as max_abs).
            cls = "float_last_digits"
        else:
            cls = "decision"
    decision_ticks = (_decision_ticks(stored_in_order, replayed_audit, log_decision,
                                      emitted_by_tick) if cls == "decision" else None)
    if not_logged:
        limits.append("not compared, the row does not log it: " + ", ".join(
            f"{k} on {n} tick(s)" for k, n in sorted(not_logged.items())))
    if yaw_exact < len(rows):
        limits.append(f"heading: the audit record's full double on {yaw_exact} tick(s), the "
                      f"log's 0.01 degree on {len(rows) - yaw_exact}")
    rail_gens = _rail_generation_records(run)
    dumped = None
    if dump is not None and diffs_all:
        dumped = _dump_differences(dump, run, stored_in_order, replayed_audit, log_diff_ticks,
                                   installs, rail_gens)
    return {
        "verdict": "differs" if diffs_all else "identical",
        "first_difference": diffs_all[0] if diffs_all else None,
        "differences": diffs_all,
        "difference_class": cls,
        # A `decision` episode's ticks that differ in more than numbers (at
        # most 20), each with whether the action the Shield emitted there
        # is bit for bit the flown one.
        "decision_ticks": decision_ticks,
        "ticks": len(rows),
        "audit": audit,
        "log": {"rows": len(rows), "source": src_rows,
                "ticks_differing_by_field": log_counts,
                "emitted_max_abs_diff": emitted_max,
                "first_difference": first_log},
        "generations": sorted(set(gens)),
        "arm": arm,
        "shield": {**cfg, "clock": "pinned per tick" if clocks else "none",
                   **({} if perceived is None
                      else {"perception_stress": perceived["record"]})},
        "state_source": state_source,
        "flown_on": _flown_on(metrics),
        # Each generation change on the rows and how the replay committed
        # it (the event time and the zones it anchored there), beside the
        # rail's own record of the same generations (events.jsonl): what
        # separates a difference the replay made from one the flown code
        # made (2026-11-18 review).
        "installs": installs,
        "rail_generations": rail_gens,
        "dump": dumped,
        "limits": limits,
    }


def _structural_fields(a: dict, b: dict) -> list[str]:
    """The fields two records differ in by more than numbers at the same
    places (a string, a shape, a list's length)."""
    return [k for k in list(a) + [k for k in b if k not in a]
            if json.dumps(a.get(k)) != json.dumps(b.get(k))
            and (k not in a or k not in b or max_abs_numeric(a[k], b[k]) is None)]


def _decision_ticks(stored: list[dict], replayed: list[dict], log_decision: dict,
                    emitted_by_tick: dict, limit: int = 20) -> list[dict]:
    out: dict = {}
    for s, r in zip(stored, replayed):
        a = _drop(s, AUDIT_WALL_FIELDS, NODE_FSM_PREFIX)
        b = _drop(r, AUDIT_WALL_FIELDS, NODE_FSM_PREFIX)
        f = _structural_fields(a, b)
        if f:
            out.setdefault(s.get("tick"), {"audit_fields": [], "log_fields": []})[
                "audit_fields"] = f
    for t, f in log_decision.items():
        out.setdefault(t, {"audit_fields": [], "log_fields": []})["log_fields"] = f
    ticks = sorted(out, key=lambda t: (t is None, t if t is not None else 0))
    return [{"tick": t, **out[t], "emitted_bit_identical": t not in emitted_by_tick,
             "emitted_max_abs_diff": emitted_by_tick.get(t)} for t in ticks[:limit]]


def _rail_generation_records(run: Path) -> list[dict]:
    """The rail's own record of each policy generation and of the scenario
    event that made it (events.jsonl: `policy_generation`, `scenario_event`),
    the mission time and wall time of each, or [] when the run kept none."""
    data, _ = read_run_file(run, "events.jsonl")
    if data is None:
        return []
    out = []
    for line in data.decode("utf-8", "replace").splitlines():
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if isinstance(r, dict) and r.get("kind") in ("policy_generation", "scenario_event"):
            out.append({k: r.get(k) for k in ("kind", "t", "wall", "generation", "policy_hash",
                                              "reason", "event", "op", "rule_id", "refused")
                        if r.get(k) is not None})
    return out


def _dump_differences(dump: Path, run: Path, stored: list[dict], replayed: list[dict],
                      log_ticks: list[dict], installs: list[dict],
                      rail_gens: list[dict]) -> str:
    """Every differing audit record (stored and replayed, `ts` and the
    node's fsm_* dropped from both, as compared) and every differing
    flight-log tick of one episode, with how each generation was installed
    and the rail's record of it, as `<dump>/<run path, slashes as __>.json`.
    For finding a difference's cause; the per-run record keeps the first
    difference only. Returns the file's name."""
    pairs = []
    for s, r in zip(stored, replayed):
        a = _drop(s, AUDIT_WALL_FIELDS, NODE_FSM_PREFIX)
        b = _drop(r, AUDIT_WALL_FIELDS, NODE_FSM_PREFIX)
        if json.dumps(a) == json.dumps(b):
            continue
        fields = [k for k in list(a) + [k for k in b if k not in a]
                  if json.dumps(a.get(k)) != json.dumps(b.get(k))]
        pairs.append({"tick": s.get("tick"), "fields": fields,
                      "stored": {k: a.get(k) for k in fields},
                      "replayed": {k: b.get(k) for k in fields}})
    name = "__".join(run.resolve().parts[-3:]).replace(":", "") + ".json"
    dump.mkdir(parents=True, exist_ok=True)
    (dump / name).write_text(json.dumps({
        "run": run.as_posix(), "audit_records_differing": pairs,
        "log_ticks_differing": log_ticks, "installs": installs,
        "rail_generations": rail_gens}, indent=1, default=str) + "\n",
        encoding="utf-8", newline="\n")
    return name


if __name__ == "__main__":
    sys.exit(main())
