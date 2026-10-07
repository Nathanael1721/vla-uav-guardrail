"""The replay bundle — WP4's deployable artefact, and the last named one missing.

    write_replay("demo/out/city_kpi", policy, "bundles/city_kpi.replay.tar.gz")
    ok, why = verify_replay("bundles/city_kpi.replay.tar.gz")

WHAT THE GRANT ASKS FOR

WP1 is accepted on "policy load round-trip; **bundle replayability**", and WP4's
artefact list names replay bundles beside the determinism manifest.
`guardrail/bundle.py` closed the policy half. This closes the flight half: one
archive holding the episode, the policy that governed it, the determinism
manifest, and the KPI table it produced.

WHAT "REPLAYABLE" HAD BETTER MEAN

Not "the files are in one place". A tar of four files nobody can re-derive
anything from would satisfy the word and none of the intent. So
`verify_replay()` reloads the policy from the archived IR, **recomputes the KPIs
from the archived log**, and compares them to the archived KPI table field by
field. A bundle that cannot reproduce its own headline numbers is refused.

That check has teeth on exactly one day: the day someone changes how a KPI is
computed without re-deriving the bundles that quote the old value. It passes
trivially today because `compute()` is deterministic, which is the point of
writing it now rather than after that day.

INTEGRITY, HONESTLY DESCRIBED

Every member in the archive is digested, the digests are listed in
`replay.json`, and the index is digested into `signature.txt`.

Since 2026-10-06 that digest is also SIGNED, with the same Ed25519 lab key and
trust store `guardrail/bundle.py` uses for policy bundles: line 2 of
`signature.txt` is a detached signature over the exact `replay.json` bytes. A
consistent forgery - edit a member, recompute the index, recompute the digest -
now fails, because the forger cannot recompute the signature.

Bundles written before that date, and any written on a machine without the key
or without `cryptography`, are KEYLESS: `signature.txt` is `sha256(replay.json)`
and nothing more, so a consistent forgery passes them. They still verify,
because four tracked bundles are in that format and refusing them would orphan
the episodes they hold - but `verify_replay` says "signature UNSIGNED" out
loud every time. A 2026-09-08 review found a test in `tests/test_replay.py`
claiming keyless protection the format never had; the honest limit is still
asserted for the keyless format, and the keyed one is tested to refuse the same
forgery.

Reading is not writing: a keyed bundle is VERIFIED on any interpreter,
`cryptography` or not, through the pure-Python RFC 8032 verifier in
`guardrail/bundle.py`. A keyed bundle whose Ed25519 line is dropped while its
signer's identity stays is refused as a stripped signature. Dropping the line
AND swapping the identity for the keyless placeholder still reads as keyless:
that downgrade is loud, not refusable, for the reason above.

POLICY IDENTITY

The archived IR is `Policy.canonical_bytes()` and the index records the
64-hex `policy_hash`. The episode's own manifest may carry an older 16-hex
form (any run flown before 2026-10-06); it binds as long as it is one of the
forms `Policy.hash_form` recognises, and the form is written into the index
as `flown_hash_form` rather than glossed over.

WHAT AN EPISODE HOLDS (since 2026-10-06)

Until this date a bundle carried four files: the log, metrics, the KPI table
and the manifest (audit card WP4-11: "re-scores, does not replay"). A run that
hot-applied a rule kept only the LAST policy, so 75 % of the dynamic KPI run's
log pointed at a generation no saved file described (WP2-15), and the Shield's
own audit log was appended across flights instead of belonging to one
(WP4-20). Now an episode directory, written through `EpisodeRecord`, holds:

  policy_g<N>.json   the canonical IR of EVERY generation in force, written at
                     take-off and after each hot-apply;
  csp_g<N>.json      the typed Constraint Summary Pack compiled from it;
  audit.jsonl        the Shield's audit log for this episode only (a stale
                     one is moved to _previous/ first, never appended to);
  events.jsonl       hot-applies, mode changes, fence uploads, FSM
                     transitions, in order;
  bag/               the rosbag2 recording, when the rail recorded one and it
                     overlaps this episode's start;
  adapter_log.jsonl  what the MAVLink adapter turned each setpoint into.

and `verify_replay` refuses a bundle whose log or audit records name a policy
hash that none of the archived generations has, which is exactly the WP2-15
defect, now a check that can fail.

ONE EPISODE, NOT TWO (since 2026-10-07)

A run tag is reused on purpose, and a re-run that crashed before it wrote its
own log used to leave the PREVIOUS flight's log, metrics, KPI table and
manifest in place beside the new episode's events and bag. `pack` then
bundled flight A's 283 rows under flight B's events, rc 0, "re-derives its own
KPIs" (found in review by re-running `pack` on a copy of a real run). Three
things now stop that:

  * starting an episode moves EVERY file a flight writes aside
    (`STALE_PATTERNS`), not only the audit and generation files;
  * `EpisodeRecord` gives the episode an id (its `episode_start` wall_ns),
    and the rails write it into every log row, metrics.json and kpi.json;
    `write_replay` and `verify_replay` refuse a log, metrics or KPI table
    carrying another episode's id;
  * an episode whose events.jsonl has no `mission_end` - a crashed or
    still-running flight - is refused, so `pack` cannot bundle it (under an
    id-less log from before 2026-10-07 this is a note instead: the pymavlink
    rail wrote no mission_end then, so its stored runs cannot show one).

ONE ESCALATION FSM PER RAIL (since 2026-10-07)

A Shield can run the escalation FSM inside filter() (`Shield(escalation=...)`,
on by default), and AuditLogger.log records that FSM's verdict unless the
caller hands it a record of its own. The SITL rails run THE FSM in the node,
because only the node sees the autopilot: its mode, home reached, landed, a
GeoFence takeover. With both running, audit.jsonl recorded the Shield's FSM
and flight_log.jsonl the node's; they disagreed on 3 of 83 ticks of the flight
ros2fix_on_dyn_nfz. `rail_shield` builds a rail's Shield without an FSM, and
`audit_tick` writes each audit record with the rail's own FSM verdict.
"""
from __future__ import annotations

import gzip
import hashlib
import inspect
import io
import json
import re
import shutil
import tarfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from .bundle import (LEGACY_PLACEHOLDER_SIGNER, SIG_BAD, SIG_VERIFIED,
                     _resolve_signer, manifest_for, parse_signature,
                     signature_text, verify_signature)
from .kpi import compute, rule_priorities
from .models import Policy

INDEX_NAME = "replay.json"
# The keyless legacy signer, kept under its old name for importers.
_SIGNED_BY = LEGACY_PLACEHOLDER_SIGNER
SIGNATURE_NAME = "signature.txt"
EPISODE = "episode/"
POLICY = "policy/"

# The episode files, and whether a bundle may be written without each. The log
# is the one thing a replay bundle cannot be missing — without it there is
# nothing to replay and the artefact is a filing cabinet.
EPISODE_FILES = {
    "flight_log.jsonl": True,
    "metrics.json": False,
    "kpi.json": False,
    "manifest.json": False,
    # Since 2026-10-06: see "WHAT AN EPISODE HOLDS" above.
    "audit.jsonl": False,
    "events.jsonl": False,
    "prompt.yaml": False,
    "adapter_log.jsonl": False,
}

# One policy IR and one CSP per policy generation, by generation number.
GENERATION_RE = re.compile(r"^(policy|csp)_g(\d+)\.json$")
# The rosbag2 directory. rosbag2 writes metadata.yaml when the recording is
# CLOSED, so a bag without it is still open (or crashed) and is not bundled.
BAG_DIR = "bag"
BAG_METADATA = "metadata.yaml"
# How far before the episode's own start a bag may begin and still be this
# episode's bag: the rail starts `ros2 bag record` before the Shield node.
BAG_LEAD_S = 600.0
# Episode files that would mix two flights if a run tag is reused. Moved to
# `_previous/<UTC stamp>/` when a new episode starts in the same directory.
# Until 2026-10-07 only the first five were moved, so a re-run that crashed
# before writing its own log left the previous flight's log, metrics, KPI
# table and manifest to be bundled with the new episode's events and bag.
STALE_PATTERNS = ("audit.jsonl", "events.jsonl", "adapter_log.jsonl",
                  "policy_g*.json", "csp_g*.json",
                  "flight_log.jsonl", "metrics.json", "kpi.json",
                  "manifest.json", "report.md", "prompt.yaml",
                  "trajectory.json", "trajectory.png", "*.replay.tar.gz")
PREVIOUS_DIR = "_previous"
# A flight log WITHOUT episode ids (rows written before 2026-10-07) is bound to
# the episode only by its file time: one last written more than this long
# before the episode started belongs to another flight.
LOG_CLOCK_SLACK_S = 2.0

# KPI fields compared on verification. Deliberately the contractual figures and
# the counts they derive from, rather than every key: a bundle should not be
# refused because a later version of `compute()` added a field.
#
# THE NAMES ARE CHECKED AGAINST `compute()` AT VERIFY TIME. The first version of
# this tuple contained "p0_ticks" and "fail_safe_correctness", neither of which
# is a key that `guardrail/kpi.py` ever emits — the real names are
# `p0_violation_ticks` and `failsafe_trigger_correctness`. A `field not in
# stored: continue` skipped both silently, so the grant's fail-safe KPI was
# never re-derived and a bundle whose fail-safe figure had been edited from 1.0
# to 0.10 verified clean. Two typos, no error, five fields compared where seven
# were claimed. See `_unknown_fields()`.
VERIFIED_KPI_FIELDS = (
    "p0_violation_escape_rate",
    "p0_violation_ticks",
    "p0_escapes",
    "p0_ticks_not_measurable",
    "failsafe_trigger_correctness",
    "mean_repair_magnitude_mps",
    "max_repair_magnitude_mps",
    "mean_time_to_safe_s",
    "max_time_to_safe_s",
)


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _add(tar: tarfile.TarFile, name: str, data: bytes) -> None:
    info = tarfile.TarInfo(name=name)
    info.size = len(data)
    info.mtime = 0          # same episode -> same bytes; see bundle.py
    tar.addfile(info, io.BytesIO(data))


def _read(tar: tarfile.TarFile, name: str) -> bytes:
    try:
        member = tar.extractfile(name)
    except KeyError:
        member = None
    if member is None:
        raise ValueError(f"replay bundle missing required member: {name}")
    return member.read()


# --------------------------------------------------------------------------- #
# Policy generations, the rosbag, and the binding checks between them
# --------------------------------------------------------------------------- #

def _generation_files(run: Path) -> dict[str, bytes]:
    if not run.is_dir():
        return {}
    return {f.name: f.read_bytes() for f in sorted(run.iterdir())
            if f.is_file() and GENERATION_RE.match(f.name)}


def generation_table(files: dict[str, bytes]) -> tuple[list[dict], dict, list[str]]:
    """`files` maps policy_g<N>.json / csp_g<N>.json names to their bytes.

    Returns `(table, policies, problems)`: one row per generation
    ({generation, policy_hash, csp, csp_policy_hash}), the rebuilt Policy per
    generation number, and every inconsistency found - a policy file that
    does not hold the generation its name says, a CSP compiled from another
    policy than the one beside it, a CSP with no policy at all.
    """
    gens: dict[int, dict] = {}
    policies: dict[int, Policy] = {}
    problems: list[str] = []
    for name in sorted(files):
        m = GENERATION_RE.match(name)
        if not m:
            continue
        kind, n = m.group(1), int(m.group(2))
        row = gens.setdefault(n, {"generation": n, "policy_hash": None,
                                  "csp": False, "csp_policy_hash": None})
        try:
            doc = json.loads(files[name])
        except ValueError as exc:
            problems.append(f"{name} is not JSON ({exc})")
            continue
        if kind == "policy":
            try:
                pol = Policy.model_validate(doc)
            except Exception as exc:                       # noqa: BLE001
                problems.append(f"{name} is not a policy IR ({type(exc).__name__})")
                continue
            if pol.generation != n:
                problems.append(f"{name} holds generation {pol.generation}, "
                                f"not {n}")
            row["policy_hash"] = pol.policy_hash
            policies[n] = pol
        else:
            row["csp"] = True
            row["csp_policy_hash"] = doc.get("policy_hash")
    for n, row in gens.items():
        if row["csp"] and n not in policies:
            problems.append(f"csp_g{n}.json has no policy_g{n}.json beside it")
        elif row["csp"] and not policies[n].matches_hash(row["csp_policy_hash"]):
            problems.append(
                f"csp_g{n}.json was compiled from policy "
                f"{row['csp_policy_hash']}, not generation {n} "
                f"({row['policy_hash']})")
    return [gens[n] for n in sorted(gens)], policies, problems


def _jsonl(data: bytes | None) -> list[dict]:
    if not data:
        return []
    out = []
    for line in data.decode("utf-8").splitlines():
        if line.strip():
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
    return out


def _json_or_none(data: bytes | None) -> dict | None:
    if not data:
        return None
    try:
        doc = json.loads(data)
    except ValueError:
        return None
    return doc if isinstance(doc, dict) else None


def unbound_policy_hashes(records: list[dict], policies: dict) -> list[str]:
    """Policy hashes named by log or audit records that no archived
    generation has, under any hash form it knows."""
    seen = sorted({str(r["policy_hash"]) for r in records
                   if isinstance(r, dict) and r.get("policy_hash")})
    return [h for h in seen
            if not any(p.matches_hash(h) for p in policies.values())]


def _bag_metadata(bag_dir: Path) -> dict | None:
    meta = bag_dir / BAG_METADATA
    if not meta.is_file():
        return None
    import yaml
    try:
        doc = yaml.safe_load(meta.read_text(encoding="utf-8")) or {}
    except Exception:                                     # noqa: BLE001
        return None
    info = doc.get("rosbag2_bagfile_information") or {}
    start = ((info.get("starting_time") or {}).get("nanoseconds_since_epoch"))
    dur = ((info.get("duration") or {}).get("nanoseconds"))
    topics = {}
    for t in info.get("topics_with_message_count") or []:
        md = t.get("topic_metadata") or {}
        if md.get("name"):
            topics[md["name"]] = int(t.get("message_count") or 0)
    return {"storage_identifier": info.get("storage_identifier"),
            "starting_time_ns": start, "duration_ns": dur,
            "message_count": info.get("message_count"),
            "topics": topics,
            "relative_file_paths": list(info.get("relative_file_paths") or [])}


def episode_start_ns(events: list[dict]) -> int | None:
    for e in events:
        if e.get("kind") == "episode_start" and isinstance(e.get("wall_ns"), int):
            return e["wall_ns"]
    return None


def bag_binding(meta: dict, start_ns: int | None) -> tuple[str, str]:
    """Is this rosbag this episode's? `("episode" | "unverified" | "refused",
    why)`. A recording that starts more than BAG_LEAD_S before the episode,
    or ends before it began, belongs to another flight that left its bag in
    a reused run directory."""
    b0, dur = meta.get("starting_time_ns"), meta.get("duration_ns")
    if start_ns is None:
        return "unverified", "no episode_start event to bind the bag to"
    if not isinstance(b0, int) or not isinstance(dur, int):
        return "refused", "the bag's metadata.yaml carries no start time"
    if b0 > start_ns + int(30e9):
        return "refused", (f"the bag starts {(b0 - start_ns) / 1e9:.1f} s "
                           f"after the episode did")
    if b0 + dur < start_ns or b0 < start_ns - int(BAG_LEAD_S * 1e9):
        return "refused", (f"the bag ({b0}..{b0 + dur} ns) does not overlap "
                           f"this episode's start ({start_ns} ns)")
    return "episode", "the recording spans the episode start"


def episode_binding(events: list[dict], rows: list[dict],
                    metrics: dict | None = None, kpi: dict | None = None,
                    log_mtime_ns: int | None = None,
                    audit: list[dict] | None = None
                    ) -> tuple[str, list[str], list[str]]:
    """Do the log, metrics, KPI table and audit belong to THIS episode?

    Returns `(binding, problems, notes)`. `binding` is "none" for a run
    written without `EpisodeRecord` (no `episode_start` event: the AirSim
    rails, every bundle before 2026-10-06) - nothing to check against;
    "episode" when every row carries this episode's id; "unverified" when the
    rows carry none (a log written between 2026-10-06 and 2026-10-07). Any
    problem means the records mix flights or the episode never finished, and
    the caller refuses. `log_mtime_ns` (write side only) is the flight log's
    file time, the one clue an id-less log leaves.

    A missing `mission_end` is a problem for an episode whose rows carry ids
    (every writer of those ends its episode). Under an id-less log it is a
    note: the pymavlink rail wrote no mission_end before 2026-10-07, so its
    stored runs cannot show one either way.

    `audit`: audit records that carry an `episode_id` (AuditLogger writes the
    one the rail gives it, as a string, since 2026-10-07) must name this
    episode; records without one bind nothing.
    """
    start = episode_start_ns(events)
    if start is None:
        return "none", [], []
    problems: list[str] = []
    notes: list[str] = []
    ended = any(e.get("kind") == "mission_end" for e in events)
    ids = {r.get("episode_id") for r in rows if "episode_id" in r}
    n_without = sum(1 for r in rows if "episode_id" not in r)
    foreign = sorted(str(i) for i in ids if i != start)
    if foreign:
        problems.append(
            f"the flight log names episode(s) {', '.join(foreign)}, not this "
            f"episode ({start}): rows from another flight")
    if ids and n_without:
        problems.append(f"{n_without} flight-log row(s) carry no episode id "
                        f"beside {len(rows) - n_without} that do")
    binding = "episode" if ids and not foreign and not n_without else "unverified"
    if not ended:
        why = ("events.jsonl has no mission_end: the episode never finished (a "
               "crashed or still-running flight), so its directory cannot hold "
               "a complete episode")
        if ids or not rows:
            problems.append(why)
        else:
            notes.append(f"{why} - or it was written by a rail that recorded "
                         f"no mission_end (the pymavlink rail before "
                         f"2026-10-07); its id-less log cannot say which")
    a_ids = {str(r.get("episode_id")) for r in (audit or [])
             if isinstance(r, dict) and r.get("episode_id") is not None}
    a_foreign = sorted(i for i in a_ids if i != str(start))
    if a_foreign:
        problems.append(f"audit.jsonl names episode(s) {', '.join(a_foreign)}, "
                        f"not this episode ({start}): records from another "
                        f"flight")
    for label, doc in (("metrics.json", metrics), ("kpi.json", kpi)):
        if not isinstance(doc, dict) or not doc:        # absent ({} on read)
            continue
        got = doc.get("episode_id")
        if got is None and binding == "episode":
            problems.append(f"{label} carries no episode id while the log does: "
                            f"it may be another flight's")
        elif got is not None and got != start:
            problems.append(f"{label} belongs to episode {got}, not this "
                            f"episode ({start})")
    if binding == "unverified" and log_mtime_ns is not None \
            and log_mtime_ns < start - int(LOG_CLOCK_SLACK_S * 1e9):
        problems.append(
            f"flight_log.jsonl was last written "
            f"{(start - log_mtime_ns) / 1e9:.1f} s before this episode "
            f"started: it is another flight's log")
    return binding, problems, notes


def write_replay(run_dir: str | Path, policy: Policy, out_path: str | Path,
                 changelog: str = "initial", *, signer="auto",
                 policy_source: dict | None = None) -> Path:
    """Package a finished flight directory and its policy into one archive.

    The policy is stored as the same canonical IR `bundle.py` hashes, so the
    policy inside a replay bundle and the policy inside a policy bundle are the
    same bytes and reload through the same path.

    `signer` follows `bundle.write_bundle`: "auto" signs with the lab key when
    present and writes a KEYLESS bundle otherwise. `policy_source` - where the
    flight got its policy (a verified bundle, or a YAML file and therefore
    unsigned) - is copied into the index when the flight supplies it.
    """
    run, out = Path(run_dir), Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    members: dict[str, bytes] = {}
    for name, required in EPISODE_FILES.items():
        f = run / name
        if f.is_file():
            members[EPISODE + name] = f.read_bytes()
        elif required:
            raise ValueError(
                f"{run} has no {name}; there is nothing to replay. A replay "
                f"bundle without the episode log is a filing cabinet.")

    # Every policy generation the episode recorded, and its CSP. Checked
    # HERE as well as on read: a bundle is not written around an episode
    # whose own records disagree about which rules were in force.
    gen_files = _generation_files(run)
    generations, gen_policies, gen_problems = generation_table(gen_files)
    if gen_problems:
        raise ValueError(f"{run.name}: policy generations are inconsistent: "
                         + "; ".join(gen_problems))
    if gen_policies:
        last = gen_policies[max(gen_policies)]
        if last.policy_hash != policy.policy_hash:
            raise ValueError(
                f"{run.name}: the policy passed here ({policy.policy_hash}) is "
                f"not the last generation the episode recorded "
                f"(g{last.generation}, {last.policy_hash})")
        for label, data in (("flight_log.jsonl", members.get(EPISODE + "flight_log.jsonl")),
                            ("audit.jsonl", members.get(EPISODE + "audit.jsonl"))):
            stray = unbound_policy_hashes(_jsonl(data), gen_policies)
            if stray:
                raise ValueError(
                    f"{run.name}: {label} names policy hash(es) "
                    f"{', '.join(stray)} that no recorded generation has")
    for name, data in gen_files.items():
        members[EPISODE + name] = data

    # One finished episode, not two (see "ONE EPISODE, NOT TWO" above).
    events = _jsonl(members.get(EPISODE + "events.jsonl"))
    log_file = run / "flight_log.jsonl"
    ep_binding, ep_problems, _ = episode_binding(
        events, _jsonl(members.get(EPISODE + "flight_log.jsonl")),
        _json_or_none(members.get(EPISODE + "metrics.json")),
        _json_or_none(members.get(EPISODE + "kpi.json")),
        log_mtime_ns=log_file.stat().st_mtime_ns,
        audit=_jsonl(members.get(EPISODE + "audit.jsonl")))
    if ep_problems:
        raise ValueError(f"{run.name}: not one finished episode: "
                         + "; ".join(ep_problems))

    # The rosbag, only when closed (metadata.yaml written) and bound to this
    # episode's start; otherwise the index says why it was left out.
    bag_index = None
    bag_dir = run / BAG_DIR
    if bag_dir.is_dir():
        meta = _bag_metadata(bag_dir)
        if meta is None:
            bag_index = {"included": False,
                         "why": "bag/ has no readable metadata.yaml: the "
                                "recording was not closed (still running, or "
                                "killed)"}
        else:
            binding, why = bag_binding(meta, episode_start_ns(events))
            bag_index = {"included": binding != "refused", "binding": binding,
                         "why": why, **{k: meta[k] for k in (
                             "storage_identifier", "starting_time_ns",
                             "duration_ns", "message_count", "topics")}}
            if binding != "refused":
                for f in sorted(bag_dir.rglob("*")):
                    if f.is_file():
                        rel = f.relative_to(run).as_posix()
                        members[EPISODE + rel] = f.read_bytes()

    # The policy has to be the one that actually governed this flight. Without
    # this, any policy bundles with any episode and `verify_replay` still says
    # yes, because the KPI recompute uses whatever priorities it is handed - and
    # it cannot notice, because `kpi.compute` treats an unknown rule_id as P0
    # (guardrail/kpi.py), so substituting a foreign policy changes no KPI field
    # at all. The run's own manifest records the hash that settles it.
    #
    # The guard used to be skipped entirely when manifest.json was absent, which
    # EPISODE_FILES permits - and a real scored flight then bundled cleanly under
    # each of the 26 other policies on disk. A run that carries KPI figures must
    # carry the manifest that binds them to a policy; one that carries no figures
    # may be bundled unbound, and says so in the index.
    run_manifest = members.get(EPISODE + "manifest.json")
    has_kpi = (EPISODE + "kpi.json") in members
    policy_binding = "manifest"
    flown = flown_form = None
    if run_manifest is not None:
        flown = json.loads(run_manifest).get("policy_hash")
        flown_form = policy.hash_form(flown) if flown else None
        if flown and flown_form is None:
            raise ValueError(
                f"{run.name} was flown under policy {flown}, but the policy "
                f"passed here hashes to {policy.policy_hash}. A replay bundle "
                f"must carry the policy that governed the episode.")
        if not flown:
            policy_binding = "unverified"
    elif has_kpi:
        raise ValueError(
            f"{run.name} has kpi.json but no manifest.json, so nothing binds "
            f"its KPI figures to a policy. Any policy would bundle and verify "
            f"clean. Bundle the manifest, or bundle the episode without its "
            f"KPI table.")
    else:
        policy_binding = "unverified"

    sgn = _resolve_signer(signer)
    members[POLICY + "ir.json"] = policy.canonical_bytes()
    members[POLICY + "manifest.json"] = json.dumps(
        manifest_for(policy, changelog,
                     signed_by=sgn.identity if sgn else None),
        indent=2, sort_keys=True).encode()

    index = {
        "kind": "vla-guardrail-replay",
        # 2: canonical v2 IR, 64-hex policy_hash, optional Ed25519 signature.
        "version": 2,
        "run": run.name,
        "policy_id": policy.policy_id,
        "policy_hash": policy.policy_hash,
        # The hash the EPISODE recorded, and which form it is in. A run flown
        # before 2026-10-06 carries a 16-hex legacy form of the same policy.
        "flown_policy_hash": flown,
        "flown_hash_form": flown_form,
        "changelog": changelog,
        # "manifest" = the episode's own manifest agreed this policy governed it.
        # "unverified" = nothing in the episode binds it to any policy; the
        # bundle is still honest, and verify_replay says so out loud.
        "policy_binding": policy_binding,
        "members": {k: _digest(v) for k, v in sorted(members.items())},
        # Every generation the episode flew under (empty for runs written
        # before 2026-10-06, which kept only the last policy).
        "generations": generations,
        # The episode the log, metrics and KPI table were checked to belong
        # to, and how ("episode" = every row carries its id; "unverified" =
        # an id-less log from before 2026-10-07; "none" = no EpisodeRecord).
        "episode_id": episode_start_ns(events),
        "log_binding": ep_binding,
    }
    if bag_index is not None:
        index["bag"] = bag_index
    if policy_source is not None:
        index["policy_source"] = policy_source
    index_bytes = json.dumps(index, indent=2, sort_keys=True).encode()
    signature = signature_text(_digest(index_bytes), sgn, index_bytes)

    with open(out, "wb") as fh:
        with gzip.GzipFile(fileobj=fh, mode="wb", mtime=0, filename="") as gz:
            with tarfile.open(fileobj=gz, mode="w") as tar:
                for name, data in sorted(members.items()):
                    _add(tar, name, data)
                _add(tar, INDEX_NAME, index_bytes)
                _add(tar, SIGNATURE_NAME, signature)
    return out


def read_replay(path: str | Path,
                trust_store: str | Path | dict | None = None) -> dict[str, Any]:
    """Open a bundle, check every digest and the signature, and return its parts.

    Raises `ValueError` on the first member whose bytes do not match the index,
    naming it, and on an Ed25519 signature that is present and wrong. Returns
    `{"index", "policy", "hash_form", "signature", "rows", "metrics", "kpi",
    "manifest"}`; `signature` is `{"status", "signed_by", "detail"}` with one
    of the statuses defined in `guardrail/bundle.py`.
    """
    with tarfile.open(path, "r:gz") as tar:
        index_bytes = _read(tar, INDEX_NAME)
        digest, signed_by, sig_hex = parse_signature(
            _read(tar, SIGNATURE_NAME).decode())
        if not digest or digest != _digest(index_bytes):
            raise ValueError(
                f"{path}: signature does not match the index; the bundle has "
                f"been modified or truncated.")
        status, detail = verify_signature(signed_by, sig_hex, index_bytes,
                                          trust_store)
        if status == SIG_BAD:
            raise ValueError(
                f"{path}: signature check failed - {detail}. The index was "
                f"rewritten after signing.")
        index = json.loads(index_bytes)
        declared = index.get("members") or {}

        # Everything the archive holds must be declared. Iterating the INDEX
        # alone left an undeclared member neither digested nor rejected, while
        # the docstring said "check every digest" - so a bundle could carry a
        # second KPI table, another policy IR, or an operator's note that no
        # digest and no signature ever covered.
        in_archive = {m.name for m in tar.getmembers() if m.isfile()}
        undeclared = sorted(in_archive - set(declared) - {INDEX_NAME,
                                                          SIGNATURE_NAME})
        if undeclared:
            raise ValueError(
                f"{path}: archive carries {len(undeclared)} member(s) the index "
                f"does not declare, so nothing covers them: "
                f"{', '.join(undeclared)}.")

        # Required members are checked BEFORE use, so a bundle without the log
        # says so instead of dying on a bare KeyError deep inside the parse -
        # the same reason `_read` exists.
        for req in (POLICY + "ir.json", EPISODE + "flight_log.jsonl"):
            if req not in declared:
                raise ValueError(
                    f"{path}: the index does not declare {req!r}, which every "
                    f"replay bundle must contain.")

        blobs: dict[str, bytes] = {}
        for name, want in declared.items():
            data = _read(tar, name)
            got = _digest(data)
            if got != want:
                raise ValueError(
                    f"{path}: member {name!r} does not match its digest "
                    f"(index {want[:12]}..., archive {got[:12]}...).")
            blobs[name] = data

    def _json(name, default=None):
        b = blobs.get(name)
        return default if b is None else json.loads(b)

    # model_validate, not load_policy: the IR is already projected and
    # validated, exactly as load_bundle does it.
    policy = Policy.model_validate(_json(POLICY + "ir.json"))
    form = policy.hash_form(index.get("policy_hash"))
    if form is None:
        raise ValueError(
            f"{path}: the policy rebuilt from ir.json hashes to "
            f"{policy.policy_hash}, but the index says {index.get('policy_hash')}"
            f" - under no form this code knows.")
    rows = [json.loads(line) for line
            in blobs[EPISODE + "flight_log.jsonl"].decode("utf-8").splitlines()
            if line.strip()]
    gen_blobs = {n[len(EPISODE):]: b for n, b in blobs.items()
                 if n.startswith(EPISODE) and GENERATION_RE.match(n[len(EPISODE):])}
    _, gen_policies, gen_problems = generation_table(gen_blobs)
    return {"index": index, "policy": policy, "hash_form": form,
            "signature": {"status": status, "signed_by": signed_by,
                          "detail": detail},
            "rows": rows,
            "metrics": _json(EPISODE + "metrics.json", {}),
            "kpi": _json(EPISODE + "kpi.json"),
            "manifest": _json(EPISODE + "manifest.json"),
            "audit": _jsonl(blobs.get(EPISODE + "audit.jsonl")),
            "events": _jsonl(blobs.get(EPISODE + "events.jsonl")),
            "generation_policies": gen_policies,
            "generation_problems": gen_problems,
            "bag_files": sorted(n for n in blobs
                                if n.startswith(EPISODE + BAG_DIR + "/"))}


def verify_replay(path: str | Path,
                  trust_store: str | Path | dict | None = None,
                  *, require_signature: bool = False
                  ) -> tuple[bool, list[str]]:
    """Can the bundle still produce the numbers it ships? `(ok, reasons)`.

    Digests, the signature, the policy round-trip, and then the part that earns
    the name: the KPIs are recomputed from the archived log and compared to the
    archived KPI table. `True` with an empty list means a reader can re-derive
    every contractual figure in the bundle from the bundle alone AND its
    signature verified against a trusted key. Any weaker signature status is
    reported as a note, never passed over in silence.

    `require_signature=True` is for KPI evidence: a keyless or unverified
    bundle is then REFUSED rather than noted. Keyless bundles stay readable by
    default because four tracked ones predate the key, but the downgrade a
    forger can perform on any bundle (drop the Ed25519 line and swap the
    signer for the keyless placeholder) cannot pass a check that demands a
    signature.
    """
    try:
        got = read_replay(path, trust_store)
    except (ValueError, KeyError, tarfile.TarError, OSError) as exc:
        return False, [str(exc)]

    flown = (got["manifest"] or {}).get("policy_hash")
    if flown and not got["policy"].matches_hash(flown):
        return False, [f"episode flown under policy {flown}, bundle carries "
                       f"{got['policy'].policy_hash}"]

    notes = []
    sig = got["signature"]
    if require_signature and sig["status"] != SIG_VERIFIED:
        return False, [f"signature {sig['status'].upper()}: {sig['detail']} - "
                       f"a signed bundle was required (KPI evidence), and a "
                       f"keyless or unverified one cannot rule out a "
                       f"consistent forgery"]
    if sig["status"] != SIG_VERIFIED:
        notes.append(
            f"signature {sig['status'].upper()}: {sig['detail']}"
            + (" - a keyless digest catches accident, not a consistent forgery"
               if sig["status"] == "unsigned" else ""))
    if got["index"].get("policy_binding") != "manifest":
        notes.append("policy binding UNVERIFIED: the episode carries no "
                     "manifest recording which policy governed it, so the KPI "
                     "recompute below is against a policy nothing confirms")

    # Every policy generation the episode flew under (WP2-15). A log row or
    # an audit record naming a hash that none of the archived generations
    # has means the bundle cannot say which rules governed that tick.
    gens = got["generation_policies"]
    if got["generation_problems"]:
        return False, got["generation_problems"]
    if gens:
        last = gens[max(gens)]
        if not got["policy"].matches_hash(last.policy_hash):
            return False, [f"the bundle's policy {got['policy'].policy_hash} is "
                           f"not its last archived generation "
                           f"(g{last.generation}, {last.policy_hash})"]
        for label, recs in (("flight log", got["rows"]),
                            ("audit log", got["audit"])):
            stray = unbound_policy_hashes(recs, gens)
            if stray:
                return False, [f"the {label} names policy hash(es) "
                               f"{', '.join(stray)} that no archived "
                               f"generation has"]
        table = sorted(got["index"].get("generations") or [],
                       key=lambda r: r.get("generation", 0))
        missing_csp = [r.get("generation") for r in table if not r.get("csp")]
        for n in missing_csp:
            why = next((e.get("error") for e in got["events"]
                        if e.get("kind") == "csp_failed"
                        and e.get("generation") == n), None)
            notes.append(f"generation {n} has no CSP on file"
                         + (f": {why}" if why else " and no event says why"))
    else:
        # A bundle from before 2026-10-06 archives only the final policy. Its
        # logs are fully bound when they name only that policy; any other
        # hash they name belongs to an earlier generation nobody kept.
        stray = unbound_policy_hashes(got["rows"] + got["audit"],
                                      {0: got["policy"]})
        if stray:
            notes.append(
                f"no per-generation policies archived (written before "
                f"2026-10-06, or by a rail that keeps none): the logs also "
                f"name {', '.join(stray)}, which the bundle cannot show")

    # One finished episode: the archived log, metrics and KPI table carry the
    # archived events' episode id, and the events say the mission ended.
    ep_binding, ep_problems, ep_notes = episode_binding(
        got["events"], got["rows"], got["metrics"], got["kpi"],
        audit=got["audit"])
    if ep_problems:
        return False, ep_problems
    notes += ep_notes
    if ep_binding == "unverified":
        notes.append("episode binding UNVERIFIED: the flight log carries no "
                     "episode id (written before 2026-10-07), so only the "
                     "writer's file-time check tied it to this episode's events")
    bag = got["index"].get("bag")
    if bag and bag.get("included"):
        empty = sorted(t for t, n in (bag.get("topics") or {}).items() if not n)
        if empty:
            notes.append(f"rosbag topics with 0 messages: {', '.join(empty)}")
        if bag.get("binding") != "episode":
            notes.append(f"rosbag binding {str(bag.get('binding')).upper()}: "
                         f"{bag.get('why')}")
    elif bag:
        notes.append(f"rosbag NOT included: {bag.get('why')}")

    stored = got["kpi"]
    if stored is None:
        # Honest, and not a failure: an episode may be bundled before it is
        # scored. Say so rather than passing silently.
        return True, notes + ["no kpi.json in the bundle; nothing to re-derive"]

    fresh = compute(got["rows"], rule_priorities(got["policy"]), got["metrics"])

    # A name in VERIFIED_KPI_FIELDS that neither side has is a typo in this
    # module, not a property of the bundle - and skipping it silently is how
    # two of the grant's KPIs went unchecked for a day. Fail loudly instead.
    unknown = [f for f in VERIFIED_KPI_FIELDS
               if f not in stored and f not in fresh]
    if unknown:
        return False, [f"VERIFIED_KPI_FIELDS names {len(unknown)} field(s) that "
                       f"neither the stored table nor compute() produces, so "
                       f"they were never compared: {', '.join(unknown)}"]

    bad = []
    for field in VERIFIED_KPI_FIELDS:
        if field not in stored:
            # Present in compute() but not in this (older) stored table. Worth
            # saying; not a mismatch.
            notes.append(f"{field}: absent from the bundle's KPI table, so it "
                         f"could not be re-derived")
            continue
        a, b = stored.get(field), fresh.get(field)
        if isinstance(a, float) and isinstance(b, float):
            if abs(a - b) > 1e-9:
                bad.append(f"{field}: bundle {a}, recomputed {b}")
        elif a != b:
            bad.append(f"{field}: bundle {a!r}, recomputed {b!r}")
    return (not bad), (bad if bad else notes)


def kpi_evidence_grade(graded: bool, why: list[str], bundle: str | Path,
                       trust_store: str | Path | dict | None = None
                       ) -> tuple[bool, list[str]]:
    """The KPI grade, once the run's bundle has been checked AS EVIDENCE.

    `is_kpi_grade` judges the manifest; it cannot see the bundle. A run it
    passes is still demoted here if its bundle does not verify with a
    trusted signature (`require_signature=True`): a keyless bundle cannot
    rule out a consistent forgery, and until 2026-10-07 no KPI path asked
    for the signature at all, so `require_signature` existed and protected
    nothing. Returns `(graded, why)`; a run that was not graded is returned
    unchanged (its bundle is still verified by the caller, for the notes).
    """
    if not graded:
        return False, list(why)
    ok, notes = verify_replay(bundle, trust_store, require_signature=True)
    if ok:
        return True, list(why)
    return False, list(why) + [
        "its replay bundle cannot serve as KPI evidence: "
        + (notes[0] if notes else "verification refused it")]


# --------------------------------------------------------------------------- #
# Writing an episode directory the bundle can trust
# --------------------------------------------------------------------------- #

def rotate_stale_episode(run_dir: str | Path,
                         stamp: str | None = None) -> list[str]:
    """Move an earlier episode's files out of the way, never append to them.

    A run tag is reused on purpose (the A/B arms always write
    `ros2_shield_on`), and every other file in the directory is rewritten by
    the next flight - but `AuditLogger` APPENDS, so `ros2_shield_on/audit.jsonl`
    held records from three dates and `ros2_shield_on_dynamic/audit.jsonl` from
    four (audit card WP4-20). The per-generation files would mix in the same
    way, and so would the log, metrics, KPI table, manifest, report and the
    old bundle when a re-run dies before rewriting them (`STALE_PATTERNS`).
    Nothing is deleted: the stale files go to `_previous/<UTC stamp>/`.
    The rosbag directory is not touched here; the rail's script moves it
    before it starts recording, and `write_replay` refuses a bag that does
    not overlap the episode.
    """
    run = Path(run_dir)
    if not run.is_dir():
        return []
    stale = sorted({p for pat in STALE_PATTERNS for p in run.glob(pat)
                    if p.is_file()})
    if not stale:
        return []
    stamp = stamp or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    dest = run / PREVIOUS_DIR / stamp
    dest.mkdir(parents=True, exist_ok=True)
    moved = []
    for p in stale:
        shutil.move(str(p), str(dest / p.name))
        moved.append(p.name)
    return moved


def episode_row_fields(shield, state, policy,
                       episode_id: int | None = None) -> dict[str, Any]:
    """The per-tick fields the KPIs need and the rails did not log.

    unsafe / unsafe_rules  is the POSITION illegal whatever the vehicle does
                           next (`Shield.state_is_unsafe`). Mean time to safe
                           is measured from this; without it the ROS rail
                           reported "not measurable" (audit card WP3-17).
    policy_hash / generation  the policy in force on this tick, so a row is
                           bound to one archived generation (WP4-13, WP2-15).
    shield_ms              the Shield's own wall time for this tick's
                           `filter()` call (WP3-13: the 100 ms budget).
    episode_id             `EpisodeRecord.episode_id`, when given, so the row
                           is bound to one episode (see "ONE EPISODE, NOT TWO").
    Call it AFTER `shield.filter()` for the tick.
    """
    unsafe = shield.state_is_unsafe(state)
    hist = shield.history
    out = {
        "unsafe": bool(unsafe),
        "unsafe_rules": sorted({v.rule_id for v in unsafe}),
        "policy_hash": policy.policy_hash,
        "generation": policy.generation,
        "shield_ms": round(hist[-1].elapsed_ms, 3) if hist else None,
    }
    if episode_id is not None:
        out["episode_id"] = episode_id
    return out


SETPOINTS = ("pass", "brake", "none")


def flown_fields(shield, state, decision, *, shield_on: bool,
                 setpoint: str = "pass") -> dict[str, Any]:
    """What was actually FLOWN on this tick, and the Shield's check on it.

    `setpoint` is what the rail sent to the autopilot:
      pass   the Shield's output (Shield on) or the VLA's raw action (Shield
             off): `emitted_violations` is the re-check of that action, or the
             raw action's own violations when the Shield is off - which is
             what earns the control arm its escapes;
      brake  the zero action the escalation FSM sends in Brake: its
             violations are those of standing still here
             (`Shield.state_is_unsafe`), not those of `decision.emitted`;
      none   nothing was sent - the autopilot (a GeoFence RTL, a fail-safe)
             or a fault holds the aircraft. `flown` is False and
             `emitted_violations` is empty: no VLA action was flown, so none
             can have escaped.

    Until 2026-10-07 both rails wrote the raw action's violations on every
    shield-off tick, flown or not, so 250 of the 283 "escapes" of the
    shield-off GeoFence run were ticks on which ArduPilot's RTL flew the
    aircraft and the rail streamed nothing.
    """
    if setpoint not in SETPOINTS:
        raise ValueError(f"setpoint {setpoint!r} is not one of {SETPOINTS}")
    if setpoint == "none":
        return {"flown": False, "emitted_violations": []}
    if setpoint == "brake":
        found = shield.state_is_unsafe(state)
    else:
        found = decision.emitted_violations if shield_on else decision.violations
    return {"flown": True,
            "emitted_violations": [v.model_dump() for v in found]}


def rail_shield(policy: Policy, *, lookahead_s: float, dt: float, **kw):
    """A flight rail's Shield, without an escalation FSM of its own.

    The rail runs THE FSM itself, fed with what only the rail sees (the
    autopilot's mode, home reached, landed, a takeover); a second FSM inside
    the Shield, stepping on the same decisions without those inputs, made the
    audit disagree with the flight log (module docstring, "One escalation FSM
    per rail"). A Shield that has no `escalation` parameter has no FSM to
    switch off."""
    from .shield import Shield
    if "escalation" in inspect.signature(Shield).parameters:
        kw.setdefault("escalation", False)
    return Shield(policy, lookahead_s=lookahead_s, dt=dt, **kw)


def audit_tick(audit, tick: int, decision, *, fsm_out=None,
               fsm_fault: str | None = None) -> None:
    """Write this tick's audit record with the RAIL's escalation verdict.

    fsm_out    the rail's FSM output for this tick (guardrail.fsm.FSMOutput),
               or None when its FSM did not step (the control arm, after an
               autopilot takeover or a fault);
    fsm_fault  the rail's FSM refusal, once it has happened (the Shield's own
               FSM carries its fault on every later decision the same way).
    Call it after the FSM stepped, so a transition is on record on its tick."""
    if fsm_fault is not None and "fsm_fault" in getattr(type(decision),
                                                         "model_fields", {}):
        decision = decision.model_copy(update={"fsm_fault": fsm_fault})
    if "fsm_record" in inspect.signature(audit.log).parameters:
        audit.log(tick, decision,
                  fsm_record=fsm_out.record if fsm_out is not None else None)
    else:
        audit.log(tick, decision)


class EpisodeRecord:
    """One flight's directory: fresh audit, every generation, the event list.

        rec = EpisodeRecord(out, policy, mission, lookahead_s=5.0)
        audit = AuditLogger(out / "audit.jsonl", policy)   # after rec: fresh
        ...
        row.update(episode_row_fields(shield, state, policy, rec.episode_id))
        rec.apply_zone(shield, fence, t=now)        # hot-apply + generation
        ...
        rec.end("target reached", t=now)
        write_replay(out, policy, out / f"{out.name}.replay.tar.gz")

    Construction moves an earlier episode's files aside
    (`rotate_stale_episode`), writes an `episode_start` event and the
    generation-0 policy and CSP. `hot_applied()` writes the next generation's.
    A CSP that cannot be compiled (no mission, a P0 set over the token
    budget, jinja2 missing in the flight venv) is recorded as an event with
    its error rather than skipped in silence; `verify_replay` quotes it.
    `episode_id` (the `episode_start` wall_ns) goes into every row, metrics
    and the KPI table; `end()` writes the `mission_end` event, without which
    the episode is refused as unfinished.
    """

    def __init__(self, run_dir: str | Path, policy: Policy, mission=None, *,
                 lookahead_s: float | None = None,
                 clock: Callable[[], datetime] | None = None,
                 rotate: bool = True,
                 wall_ns: Callable[[], int] = time.time_ns) -> None:
        self.dir = Path(run_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.policy = policy
        self.mission = mission
        self.lookahead_s = lookahead_s
        self.clock = clock
        self._wall_ns = wall_ns
        self.moved = rotate_stale_episode(self.dir) if rotate else []
        self.events_path = self.dir / "events.jsonl"
        self.generations: list[dict] = []
        self.episode_id: int = int(wall_ns())
        self.ended = False
        self.event("episode_start", wall_ns=self.episode_id,
                   episode_id=self.episode_id,
                   policy_id=policy.policy_id, policy_hash=policy.policy_hash,
                   generation=policy.generation, moved_aside=self.moved)
        self.write_generation("take-off")

    def end(self, why: str, t: float | None = None, **data) -> dict:
        """The `mission_end` event. Once only; a second call is a no-op."""
        if self.ended:
            return {}
        self.ended = True
        return self.event("mission_end", t=t, why=why, **data)

    def event(self, kind: str, /, t: float | None = None, **data) -> dict:
        """Append one event. `t` is mission time in seconds when known.

        `kind` is positional-only, so a payload that carries its own "kind"
        (a status message, an identity record) cannot collide with it; such
        a key is stored as "data_kind" rather than raising inside a ROS
        callback mid-flight."""
        rec = {"kind": kind, "t": None if t is None else round(float(t), 3),
               "wall": datetime.now(timezone.utc).isoformat(
                   timespec="milliseconds")}
        for k, v in data.items():
            rec[f"data_{k}" if k in rec else k] = v
        with open(self.events_path, "a", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(rec, default=str) + "\n")
        return rec

    def write_generation(self, reason: str, t: float | None = None) -> dict:
        g = self.policy.generation
        pol_file = self.dir / f"policy_g{g}.json"
        pol_file.write_bytes(self.policy.canonical_bytes())
        csp_file, err = self.dir / f"csp_g{g}.json", None
        if self.mission is None:
            err = "no mission given, so no CSP can be compiled"
        else:
            try:
                from .compiler import ConstraintCompiler
                kw = {} if self.lookahead_s is None else {
                    "lookahead_s": self.lookahead_s}
                ConstraintCompiler(self.policy).write_csp(
                    csp_file, self.mission,
                    now=self.clock() if self.clock else None, **kw)
            except Exception as exc:                          # noqa: BLE001
                err = f"{type(exc).__name__}: {exc}"
        row = {"generation": g, "policy_hash": self.policy.policy_hash,
               "reason": reason, "csp": err is None}
        self.generations.append(row)
        if err is not None:
            self.event("csp_failed", t=t, generation=g, error=err)
        self.event("policy_generation", t=t, **row)
        return row

    def hot_applied(self, rule_id: str, t: float | None = None) -> dict:
        """Call right after `Shield.hot_apply(...)` bumped the generation."""
        return self.write_generation(f"hot_apply {rule_id}", t=t)

    def apply_zone(self, shield, fence, t: float | None = None):
        """Hot-apply a keep-out zone mid-flight and record the new generation.
        Returns the rule that was applied.

        The grant's mid-flight update model lets only dynamic_nfz,
        time_window_switch and corridor_swap change after mission start
        (Policy DSL page). A Shield that enforces that (it offers
        `guardrail.shield.as_dynamic_nfz`) refuses a polygon_fence once the
        mission has begun, so the zone goes in as a dynamic_nfz with the same
        id, vertices, altitude band and margin and no motion. A Shield
        without it takes the polygon_fence as the rails always sent it."""
        try:
            from .shield import as_dynamic_nfz
        except ImportError:
            rule = fence
        else:
            rule = as_dynamic_nfz(fence)
        shield.hot_apply(rule)
        self.hot_applied(rule.id, t=t)
        return rule


# --------------------------------------------------------------------------- #
# CLI: python -m guardrail.replay {pack,verify}
# --------------------------------------------------------------------------- #

def last_generation_policy(run_dir: str | Path) -> Policy:
    """The policy in force at the end of an episode, from its own records."""
    _, policies, problems = generation_table(_generation_files(Path(run_dir)))
    if problems:
        raise ValueError("; ".join(problems))
    if not policies:
        raise ValueError(f"{run_dir} holds no policy_g<N>.json; pack it from "
                         f"the flight script, which has the policy in memory")
    return policies[max(policies)]


def _main(argv: list[str] | None = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(
        prog="python -m guardrail.replay",
        description="Pack a finished episode directory into a replay bundle, "
                    "or verify one.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("pack", help="(re)write RUN/<name>.replay.tar.gz from "
                       "the episode's own policy generations - used after the "
                       "rosbag is closed, so the bundle carries it")
    p.add_argument("run_dir")
    p.add_argument("--out", default=None)
    p.add_argument("--keyless", action="store_true",
                   help="do not sign even if the lab key is present")
    v = sub.add_parser("verify", help="verify a bundle and print every note")
    v.add_argument("bundle")
    v.add_argument("--require-signature", action="store_true")
    args = ap.parse_args(argv)

    if args.cmd == "pack":
        run = Path(args.run_dir)
        src = None
        try:
            src = json.loads((run / "metrics.json").read_text(
                encoding="utf-8")).get("policy_source")
        except (OSError, ValueError):
            pass
        out = Path(args.out) if args.out else run / f"{run.name}.replay.tar.gz"
        # Refused, not packed: an unfinished episode (no mission_end), records
        # from two flights, or generations that disagree. run_ros2_demo.sh
        # runs this after every launch, including one whose Shield node died.
        try:
            policy = last_generation_policy(run)
            write_replay(run, policy, out, changelog=f"pack {run.name}",
                         signer=None if args.keyless else "auto",
                         policy_source=src)
        except ValueError as exc:
            print(f"[replay] REFUSED {run}: {exc}")
            return 1
        ok, why = verify_replay(out)
        idx = read_replay(out)["index"]
        bag = idx.get("bag") or {}
        print(f"[replay] {out} - {'re-derives its own KPIs' if ok else 'FAILED'}"
              f" | generations {[g['generation'] for g in idx['generations']]}"
              f" | bag {'included' if bag.get('included') else 'absent'}")
        for w in why:
            print(f"[replay]   - {w}")
        return 0 if ok else 1
    ok, why = verify_replay(args.bundle, require_signature=args.require_signature)
    print(f"{'OK' if ok else 'FAILED'}: {args.bundle}")
    for w in why:
        print(f"  - {w}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_main())
