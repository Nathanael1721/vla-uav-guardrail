"""
Audit log — one JSONL record per Shield decision that touched the action.

Mirrors the grant's audit-log rule: every record carries policy_hash so any
KPI number can be traced back to the exact policy that produced it.

Since 2026-10-12 a record is an `AuditRecord`, a Pydantic v2 model (the grant,
Safety Shield page: "Pydantic v2 for the audit-log records"), validated
before it is written: a record with a missing field, a field of the wrong
type or a key nobody declared is refused at write time instead of being
found by a reader later. `policies/audit_record.schema.json` is its JSON
Schema (`python -m guardrail.audit schema`; tests/test_shieldcore_audit.py
fails when the file and the model disagree). `read_audit` reads every shape
this project has stored since July 2026 into the same model.

THE GRANT'S RECORD (Safety Shield page, "Audit log"), field by field

Since record version 4 (2026-11-02) each of the grant's ten names carries the
grant's content, and the ten are written first, in the grant's order. Until
then three of them carried this project's: `raw_action` / `emitted_action`
were the world-frame actions (vz_up), a violation's `category` was this
project's finer one, and `repair_attempts` listed the repairs made, not the
operators tried (the independent review of 29 Oct, WP3-11).

    ts                 ISO-8601 UTC, milliseconds
    policy_hash        the hash of the rule set the decision was made under
                       (ShieldDecision.policy_hash, taken when the tick began;
                       a hot-apply between ticks cannot restamp a decision)
    generation         that rule set's generation (bumps on every hot-apply)
    monitor_tick       the caller's tick counter (also kept as `tick`)
    raw_action         the action in the grant's body frame, keys {vx, vy, vz,
                       yaw_rate}: vx forward, vy right, vz up, yaw_rate rad/s;
                       guardrail.frames.to_body of the Shield's world-frame
                       action at the tick's heading (`body_yaw_deg`), converted
                       when the record is written (the Shield itself still
                       repairs in the world frame, WP3-04). Null when the
                       decision does not carry the heading (one built by
                       hand, outside Shield.filter / advise): never a guess
    violations         each with the grant's `category` (geometric / envelope
                       / time-window) and `boundary_intersect_at_s` (=
                       predicted_at_s, when in the lookahead the rule breaks).
                       `time-window` is a rule not in force now that the
                       forecast breaks after a window edge brings it into
                       force (the Shield's `ShieldDecision.trace
                       ["window_edge"]`); `contract` (a non-finite command
                       zeroed) is this project's extension, not a grant
                       category. This project's finer category (geofence,
                       corridor, altitude, kinematic, clearance, standoff,
                       contract) is `rule_category`; `priority` when the
                       logger holds the policy
    repair_attempts    the repair OPERATORS tried (guardrail/repair.py), in the
                       order they ran, each with the grant's keys: operator,
                       result (ok / skip / fail / fallback / relaxed), reason
                       (why a skip skipped), magnitude_m (the largest of the
                       repairs it logged) and iterations (the pass it ran in:
                       0 before the iterated loop, 1-3 inside it, null for
                       the last resorts after it); and this project's `run`
                       (the chain run: 1, 2, ... when soft rules were relaxed
                       and the stack ran again), `axis` and the `repairs` it
                       logged. The flat form of `repair_passes`. Null when the
                       decision carries no trace (built by hand), and on a
                       stored record from before record version 2, which kept
                       no operator list
    emitted_action     what the repair stack produced, in the body frame as
                       raw_action
    fsm_state_before / fsm_state_after
                       the escalation FSM's state around this tick

THIS PROJECT'S CONTENT, UNDER ITS OWN NAMES

    tick, episode_id   the tick again, and the rail's episode id
    raw_action_world / emitted_action_world
                       the same two actions in the Shield's world frame (this
                       project's Action4D): vx North, vy East, vz_up (+up),
                       yaw_rate rad/s; the frame of a flight log's `raw` /
                       `emitted`, which a reader joins on. What `raw_action`
                       / `emitted_action` held until record version 4
    repairs            every Repair the decision logged (operator, detail,
                       magnitude_m, axis, recovery)
    repair_results     one per Repair: operator, result (ok / fallback /
                       relaxed), reason, magnitude_m, axis, recovery, and
                       `iterations`: the pass of the flown run's iterated
                       repair loop in which that repair (operator AND
                       detail) was first logged (0 = before the loop; null
                       for the last resorts after it). What `repair_attempts`
                       held until record version 4
    repair_passes      the operator list pass by pass, in every run of the
                       repair stack (`run` 1, 2, ...; `chain_runs` counts them
                       and `flown_run` is the one whose action was flown).
                       `iterations` is the number of loop passes that ran,
                       summed over the runs (measured, not the configured 3;
                       0 when no loop ran)
    fsm_edge, fsm_edge_source, fsm_set_mode, fsm_setpoint, fsm_reason,
    fsm_config_hash    the FSM's edge, edge source (grant / extension), the
                       mode it requested, the setpoint it streamed, why, and
                       the config hash
    fsm_hold_point     where that mode holds (Loiter) or lands (Land): the
                       FSM's hold point (fsm.HoldPoint, chosen from the
                       Shield's own rules by guardrail/holdpoint.py) - in
                       place or moved, the point, whether it is legal and
                       the rules it breaks if not, and why; null in every
                       other state and on a rail that runs its own FSM and
                       chooses none (record version 3, since 2026-10-20)

plus `emitted_violations`, `braked`, `relaxed`, `fsm_fault`, `repair_mode`,
`body_yaw_deg` and `record_version` (4 since the grant's names carry the
grant's content, 2026-11-02; 3 since `fsm_hold_point`, 2026-10-20; 2 since
AuditRecord, 2026-10-12; absent, read as 1, before). A record is
written when the decision saw a violation, and also on a tick where the FSM
changed state, requested a mode, refused its input or chose a new hold
point, so a Brake -> Normal recovery on a clean tick is on record too, and
so is every place the FSM told the autopilot to hold or land. Since
2026-10-20 a tick can have a second record, `decision_kind` "stop_advised":
the Shield's answer to a stop that the ArduPilot rail flew instead of the
tick's filter() decision (the hold that stops clear), always written; the
field is absent from every other record (it did not change the record
version: a reader of version 3 that knows no `decision_kind` reads every
record it wrote before the field as before). A READER that groups records by
tick must take such a record as its tick's second decision, written right
after that tick's filter() record at the same tick - never as a tick that
failed to advance, i.e. a new flight (tools/export_episodes.audit_blocks
did, and split the hold rail run's audit into 22 "flights", joining 40 of
its 60 rows: 2026-10-20 re-review), and never as the tick's own record
(demo/track_truth.score_standoff_firings skips it: its violations are the
stop's, not those the flight log's row holds).

READING A STORED RECORD. `upgrade_record` / `read_audit` give a record of any
version the version-4 shape and keep `record_version` as its writer wrote it.
A record of version 3 or older: its world-frame `raw_action` /
`emitted_action` become `raw_action_world` / `emitted_action_world`; its
`raw_action_body` / `emitted_action_body` (versions 2-3) become `raw_action`
/ `emitted_action` (null on version 1, which kept no heading: never rotated
at a guess); a violation's `category` becomes `rule_category` and its
`grant_category` (derived from ours where it is absent) becomes `category`;
its `repair_attempts` become `repair_results` (derived from `repairs` where
it is absent, as the writer of the third shape derived them); and
`repair_attempts` is rebuilt from `repair_passes` (null before version 2).
A record is taken as version 4 when its `raw_action` has no `vz_up` key, so
the upgrade can be run twice, and the grant's own example record reads as
one.

THE FILE. Every rail writes `shield_audit.jsonl` (AUDIT_FILE, since
2026-11-02), the grant's name for the Shield's audit log in an episode bundle
(Stress Testing, "Episode bundle"). A run stored before then holds
`audit.jsonl` (LEGACY_AUDIT_FILE); `audit_path` and `audit_files` find
either, the new name first.

ONE FILE PER EPISODE ("Python module; rotated per mission", Outputs table).
`AuditLogger.for_episode(directory, policy, episode_id)` opens
`shield_audit_<episode_id>.jsonl` fresh (`audit_<episode_id>.jsonl` until
2026-11-02); `rotate(episode_id)` starts the next one.
The plain constructor still APPENDS to the path it is given, because the rails
pair it with guardrail/replay.py's EpisodeRecord, which moves a stale audit
file aside before the logger opens it.
"""
from __future__ import annotations

import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from .frames import to_body
from .shield import ShieldDecision

# This project's violation categories in the grant's three (Safety Shield page,
# "Violation checking": geometric / envelope / time-window). A rule out of its
# window raises nothing; a rule a window edge brings into force inside the
# lookahead is "time-window" whatever its own category (`_grant_category`).
# "contract" (the Shield's own non-finite-command check) has no grant
# category and is written as itself: an extension, not one of the three.
GRANT_CATEGORY = {
    "geofence": "geometric", "corridor": "geometric",
    "altitude": "envelope", "kinematic": "envelope", "clearance": "envelope",
    "standoff": "envelope", "contract": "contract",
}

# 4 since 2026-11-02: the grant's raw_action / emitted_action (body frame), a
# violation's category (the grant's three) and repair_attempts (the operators
# tried) carry the grant's content; this project's moved to raw_action_world /
# emitted_action_world, rule_category and repair_results.
# 3 since 2026-10-20: `fsm_hold_point`. Until then the trail copied only
# _FSM_FIELDS, and ShieldDecision.hold_point is excluded from model_dump(), so
# where the FSM told the autopilot to hold or land - and whether that point
# was legal - reached only the dumped fsm_record and the harness's rows, never
# the grant's audit record (review of T1c, P7).
RECORD_VERSION = 4
# The audit log's file in a run folder: the grant's name (Stress Testing,
# "Episode bundle": shield_audit.jsonl) since 2026-11-02, audit.jsonl before.
AUDIT_FILE = "shield_audit.jsonl"
LEGACY_AUDIT_FILE = "audit.jsonl"
# The grant's ten fields, in the order its example writes them (Safety Shield
# page, "Audit log"); AuditRecord writes them first, in this order.
GRANT_FIELDS = ("ts", "policy_hash", "generation", "monitor_tick", "raw_action",
                "violations", "repair_attempts", "emitted_action",
                "fsm_state_before", "fsm_state_after")
_FALLBACK_OPERATORS = ("Brake", "ClearanceEscape")
_FSM_FIELDS = ("fsm_state_before", "fsm_state_after", "edge", "edge_source",
               "set_mode", "setpoint", "reason", "fsm_config_hash")
_SAFE_ID = re.compile(r"^[A-Za-z0-9._-]+$")
Result = Literal["ok", "skip", "fail", "fallback", "relaxed"]


# --------------------------------------------------------------------------- #
# The record
# --------------------------------------------------------------------------- #

class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid", populate_by_name=True)


class WorldAction(_Strict):
    """This project's Action4D: world frame, vz up-positive, yaw rate rad/s."""
    vx: float
    vy: float
    vz_up: float
    yaw_rate: float


class BodyAction(_Strict):
    """The grant's action keys: body frame (vx forward, vy right), vz up,
    yaw rate rad/s (guardrail/frames.py)."""
    vx: float
    vy: float
    vz: float
    yaw_rate: float


class ViolationRecord(_Strict):
    """`category` is the grant's (geometric / envelope / time-window, or this
    project's `contract`), `rule_category` this project's finer one (record
    version 4; until then `category` held ours and `grant_category` the
    grant's)."""
    rule_id: str
    category: str
    boundary_intersect_at_s: float | None = None
    rule_category: str | None = None
    detail: str = ""
    predicted_at_s: float | None = None
    priority: str | None = None


class RepairRecord(_Strict):
    """One `Repair` of the decision (guardrail/shield.py)."""
    operator: str
    detail: str = ""
    magnitude_m: float | None = None
    axis: str | None = None
    recovery: bool = False


class RepairResult(_Strict):
    """One Repair the decision logged, with its result and the pass that first
    logged it: `repair_results` (what `repair_attempts` held until record
    version 4)."""
    operator: str
    result: Result
    reason: str = ""
    magnitude_m: float | None = None
    axis: str | None = None
    recovery: bool = False
    iterations: int | None = None


class OperatorAttempt(_Strict):
    """One repair operator, one call, in the grant's `repair_attempts` keys
    (Safety Shield page: {"operator": "AltitudeClamp", "result": "skip",
    "reason": ...}, {"operator": "LateralProjection", "result": "ok",
    "magnitude_m": 0.7, "iterations": 1}). `iterations` is the pass it ran in
    (guardrail/repair.py RepairAttempt.iterations); `run`, `axis` and
    `repairs` are this project's."""
    operator: str
    result: Result
    reason: str = ""
    magnitude_m: float | None = None
    iterations: int | None = None
    run: int | None = None
    axis: str | None = None
    repairs: list[str] = []


class HoldPointRecord(_Strict):
    """Where the escalation FSM's Loiter holds or its Land lands (fsm.HoldPoint
    .to_dict(), the same dict as ShieldDecision.hold_point): `kind` in_place
    (ArduPilot's own LOITER / BRAKE / LAND where the aircraft is) or moved
    (fly to x, y, up first, `travel_s` at `speed_mps`); `legal` whether a
    stop (Loiter) or the descent (Land) there breaks no hard enforced rule
    over `checked_s`, `breaks` the rules it breaks if not; `t_s` the monitor
    time the FSM took it."""
    mode: Literal["LOITER", "LAND"]
    kind: Literal["in_place", "moved"]
    x: float
    y: float
    up: float
    legal: bool
    travel_s: float = 0.0
    speed_mps: float | None = None
    climb_mps: float | None = None
    checked_s: float | None = None
    breaks: list[str] = []
    reason: str = ""
    t_s: float | None = None


class PassAttempt(_Strict):
    """One repair operator, one call (guardrail/repair.py RepairAttempt)."""
    operator: str
    result: Result
    reason: str = ""
    iterations: int | None = None
    repairs: list[str] = []
    magnitude_m: float | None = None
    axis: str | None = None


class RepairPass(_Strict):
    """One pass of one run of the repair stack. `run` numbers the chain runs
    of the tick (1, 2, ... - more than one when soft rules were relaxed);
    null on the one pass of a tick where no chain ran (a Brake on a stop
    rule, or a Sanitise alone)."""
    run: int | None = None
    pass_no: int = Field(alias="pass")
    attempts: list[PassAttempt] = []


class AuditRecord(_Strict):
    """One audit line. Field order is the order written: the grant's ten
    fields first, in its example's order (GRANT_FIELDS), then this
    project's. `raw_action` / `emitted_action` are required keys and null
    only for a decision without a heading. The world-frame actions and a
    violation's `rule_category` are optional here only so the grant's own
    example record validates; AuditLogger writes them on every record."""
    ts: str
    policy_hash: str
    generation: int | None = None
    monitor_tick: int
    raw_action: BodyAction | None
    violations: list[ViolationRecord] = []
    # None: no operator list was recorded (a decision built by hand, or a
    # stored record from before record version 2) - not "no operator ran".
    repair_attempts: list[OperatorAttempt] | None = None
    emitted_action: BodyAction | None
    fsm_state_before: str | None = None
    fsm_state_after: str | None = None
    # This project's, from here on.
    tick: int
    episode_id: str | int | None = None
    raw_action_world: WorldAction | None = None
    emitted_action_world: WorldAction | None = None
    repairs: list[RepairRecord] = []
    repair_results: list[RepairResult] = []
    # None on a stored record from before the field existed: not measured,
    # which is not the same as "nothing escaped".
    emitted_violations: list[ViolationRecord] | None = None
    braked: bool = False
    relaxed: list[str] | None = None
    fsm_edge: str | None = None
    fsm_fault: str | None = None
    fsm_edge_source: str | None = None
    fsm_set_mode: str | None = None
    fsm_setpoint: str | None = None
    fsm_reason: str | None = None
    fsm_config_hash: str | None = None
    # Since AuditRecord (2026-10-12).
    record_version: int = 1
    body_yaw_deg: float | None = None
    repair_mode: str | None = None
    iterations: int | None = None
    chain_runs: int | None = None
    flown_run: int | None = None
    repair_passes: list[RepairPass] | None = None
    # Since record version 3 (2026-10-20).
    fsm_hold_point: HoldPointRecord | None = None
    # Since 2026-10-20: which decision this record is when one tick has two.
    # Absent (None) is the tick's filter() decision, every record before the
    # field existed; "stop_advised" is the Shield's answer to a stop
    # (Shield.advise of a zero action) that a rail flew INSTEAD of it - the
    # ArduPilot rail's hold that stops clear (docs/DESIGN-own-velocity-
    # forecast.md). Such a record follows its tick's filter() record (when
    # that one is written) and is always written, touched or not, so the
    # decision behind every action sent is on record. Omitted from the line
    # when None, so every other record is written exactly as before.
    decision_kind: Literal["stop_advised"] | None = None


# The record kinds a caller may ask `AuditLogger.log` for (`decision_kind`).
DECISION_KINDS = ("stop_advised",)


def audit_record_schema() -> dict:
    """The record's JSON Schema (draft 2020-12, which Pydantic v2 emits)."""
    schema = AuditRecord.model_json_schema(by_alias=True)
    return {"$schema": "https://json-schema.org/draft/2020-12/schema",
            "$id": f"urn:guardrail:audit-record:{RECORD_VERSION}", **schema}


SCHEMA_PATH = Path(__file__).resolve().parents[1] / "policies" / "audit_record.schema.json"


def write_audit_record_schema(path: str | Path = SCHEMA_PATH) -> Path:
    """Write the schema with LF line endings on every OS, so a regenerated
    file diffs only where the model changed."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(audit_record_schema(), indent=2) + "\n")
    return p


# --------------------------------------------------------------------------- #
# Reading stored records
# --------------------------------------------------------------------------- #

def _attempt_result(operator: str) -> str:
    return ("fallback" if operator in _FALLBACK_OPERATORS
            else "relaxed" if operator == "SoftRelax" else "ok")


def operator_attempts(passes: list | None) -> list[dict] | None:
    """The grant's `repair_attempts`: every operator of `repair_passes`, in the
    order it ran, flat, each with its chain run (OperatorAttempt). None when
    no operator list was recorded."""
    if passes is None:
        return None
    out = []
    for p in passes:
        run = p.get("run") if isinstance(p, dict) else None
        for a in (p.get("attempts") or []) if isinstance(p, dict) else []:
            out.append({"operator": a.get("operator"), "result": a.get("result"),
                        "reason": a.get("reason", ""), "magnitude_m": a.get("magnitude_m"),
                        "iterations": a.get("iterations"), "run": run,
                        "axis": a.get("axis"), "repairs": list(a.get("repairs") or [])})
    return out


def is_version_4(raw: dict) -> bool:
    """A record already in the version-4 shape: its `raw_action` is the grant's
    body-frame action (or null), not the world-frame one with `vz_up` every
    record before version 4 carries."""
    act = raw.get("raw_action")
    return not (isinstance(act, dict) and "vz_up" in act)


def upgrade_record(raw: dict) -> dict:
    """A stored record of any shape this project has written, in the current
    (version 4) shape; `record_version` stays the writer's. Four shapes are on
    disk: July-August 2026 (no emitted_violations), late August to October
    (emitted_violations, no generation or FSM fields), 2026-10-07 to
    2026-11-01 (the grant's field names, this project's content in three of
    them: record versions 1-3), and version 4. What a shape did not record
    stays None - an absent `emitted_violations` is "not measured", never
    "none"; a version-1 record's body-frame actions (no heading kept) and an
    operator list before version 2 are null, never derived. The repair
    results of a record that has none are derived from its `repairs` exactly
    as the writer of the third shape derived them. Idempotent (`is_version_4`)."""
    r = dict(raw)
    r.setdefault("record_version", 1)
    if r.get("monitor_tick") is None:
        r["monitor_tick"] = r.get("tick")
    if r.get("tick") is None:
        r["tick"] = r.get("monitor_tick")
    if is_version_4(r):
        r.setdefault("raw_action", None)
        r.setdefault("emitted_action", None)
        return r
    r["raw_action_world"] = r.pop("raw_action")
    r["emitted_action_world"] = r.pop("emitted_action", None)
    r["raw_action"] = r.pop("raw_action_body", None)
    r["emitted_action"] = r.pop("emitted_action_body", None)
    r["violations"] = [_upgrade_violation(v) for v in r.get("violations") or []]
    if r.get("emitted_violations") is not None:
        r["emitted_violations"] = [_upgrade_violation(v) for v in r["emitted_violations"]]
    if "repair_attempts" in r:
        r["repair_results"] = r.pop("repair_attempts")
    else:
        r["repair_results"] = [{
            "operator": rep.get("operator"), "result": _attempt_result(rep.get("operator")),
            "reason": rep.get("detail", ""), "magnitude_m": rep.get("magnitude_m"),
            "axis": rep.get("axis"), "recovery": bool(rep.get("recovery", False))}
            for rep in r.get("repairs") or []]
    r["repair_attempts"] = operator_attempts(r.get("repair_passes"))
    return r


def _upgrade_violation(v: dict) -> dict:
    """A violation of a record before version 4: ours `category` ->
    `rule_category`, the grant's `grant_category` (or ours mapped) ->
    `category`."""
    v = dict(v)
    if "boundary_intersect_at_s" not in v:
        v["boundary_intersect_at_s"] = v.get("predicted_at_s")
    ours = v.get("category")
    grant = v.pop("grant_category", None)
    v["rule_category"] = ours
    v["category"] = grant if grant is not None else GRANT_CATEGORY.get(ours, ours)
    return v


def audit_path(run_dir: str | Path) -> Path | None:
    """The audit log of a run folder: `shield_audit.jsonl`, else (a run stored
    before 2026-11-02) `audit.jsonl`; None when it holds neither."""
    for name in (AUDIT_FILE, LEGACY_AUDIT_FILE):
        p = Path(run_dir) / name
        if p.is_file():
            return p
    return None


def audit_files(run_dir: str | Path) -> list[Path]:
    """Every audit file of a run folder, in reading order: its one log
    (`audit_path`), else its per-episode files (`shield_audit_<id>.jsonl`,
    then the older `audit_<id>.jsonl`)."""
    one = audit_path(run_dir)
    if one is not None:
        return [one]
    d = Path(run_dir)
    return sorted(d.glob("shield_audit_*.jsonl")) + sorted(d.glob("audit_*.jsonl"))


def read_audit(path: str | Path) -> list[AuditRecord]:
    """Every record of a stored audit file, validated, in the current model.
    A line that does not validate raises, naming the line."""
    out = []
    with open(path, encoding="utf-8") as fh:
        for n, line in enumerate(fh, 1):
            if not line.strip():
                continue
            try:
                out.append(AuditRecord.model_validate(upgrade_record(json.loads(line))))
            except Exception as e:                       # noqa: BLE001
                raise ValueError(f"{path}:{n}: not an audit record: {e}") from e
    return out


# --------------------------------------------------------------------------- #
# Writing
# --------------------------------------------------------------------------- #

def _body(action, yaw_deg) -> dict | None:
    if yaw_deg is None:
        return None
    vx, vy, vz, wz = to_body(action, yaw_deg)
    return {"vx": vx, "vy": vy, "vz": vz, "yaw_rate": wz}


class AuditLogger:
    def __init__(self, path: str | Path, policy_hash: "str | object", *,
                 episode_id: str | None = None, fresh: bool = False):
        """`policy_hash` may be a string OR the Policy itself.

        Pass the POLICY when the run can hot-apply a rule. A string is a
        snapshot taken at construction, and `Shield.hot_apply` mutates the live
        policy and bumps its generation, so every record written afterwards
        carried the hash of a policy that no longer applied - the opposite of
        what hot_apply's own docstring promises ("every artefact after this
        instant carries a different policy_hash - the audit trail shows exactly
        which rules were active when").

        Reproduced before the fix: a record whose violation was `nfz-hot`
        stamped with the hash of a policy that did not contain `nfz-hot`. That
        is precisely the traceability this module exists to provide, and it
        failed on the one feature built for an external team (`POST /nfz`).

        The string form still works, so the seven existing call sites are
        unaffected; they simply do not benefit. A decision from a Shield of
        2026-10-07 or later carries its own policy_hash and generation, and
        those win over both forms.

        `fresh=True` truncates the file (a new episode); the default appends.
        """
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._policy = None if isinstance(policy_hash, str) else policy_hash
        self._static_hash = policy_hash if isinstance(policy_hash, str) else None
        self.episode_id = episode_id
        self._n = 0
        if fresh:
            self.path.write_text("", encoding="utf-8")

    @classmethod
    def in_memory(cls, policy: "str | object", *,
                  episode_id: str | None = None) -> "AuditLogger":
        """A logger with no file: `record()` and `line()` build the validated
        records and the caller keeps them (the Stress Testing harness packs
        them into its episode bundle as shield_audit.jsonl, Stress Testing:
        "aggregated into the episode bundle ... at episode end"). `log()`
        refuses: there is nowhere to write."""
        self = cls.__new__(cls)
        self.path = None
        self._policy = None if isinstance(policy, str) else policy
        self._static_hash = policy if isinstance(policy, str) else None
        self.episode_id = episode_id
        self._n = 0
        return self

    @staticmethod
    def line(rec: "AuditRecord") -> dict:
        """The JSON object `log()` writes for `rec` (decision_kind dropped when
        it is None, as every record before that field existed)."""
        out = rec.model_dump(by_alias=True)
        if out.get("decision_kind") is None:
            out.pop("decision_kind", None)
        return out

    @classmethod
    def for_episode(cls, directory: str | Path, policy: "str | object",
                    episode_id: str) -> "AuditLogger":
        """A fresh `shield_audit_<episode_id>.jsonl` in `directory`, one per
        episode (`audit_<episode_id>.jsonl` until 2026-11-02)."""
        return cls(Path(directory) / cls._episode_name(episode_id), policy,
                   episode_id=episode_id, fresh=True)

    @staticmethod
    def _episode_name(episode_id: str) -> str:
        if not isinstance(episode_id, str) or not _SAFE_ID.match(episode_id):
            raise ValueError(f"episode_id {episode_id!r} must be letters, digits, "
                             f"'.', '_' or '-' (it names a file)")
        return f"shield_audit_{episode_id}.jsonl"

    def rotate(self, episode_id: str) -> Path:
        """Close this episode's file and start the next one beside it."""
        self.path = self.path.parent / self._episode_name(episode_id)
        self.path.write_text("", encoding="utf-8")
        self.episode_id = episode_id
        self._n = 0
        return self.path

    @property
    def policy_hash(self) -> str:
        """Read live when a policy was supplied, so a hot-applied rule shows up."""
        if self._policy is not None:
            return getattr(self._policy, "policy_hash", None) or ""
        return self._static_hash or ""

    @property
    def generation(self) -> int | None:
        return getattr(self._policy, "generation", None) if self._policy is not None else None

    def _priorities(self) -> dict:
        cons = getattr(self._policy, "constraints", None) or ()
        return {c.id: getattr(c, "priority", None) for c in cons}

    def record(self, tick: int, decision: ShieldDecision,
               fsm_record: dict | None = None, *,
               kind: str | None = None) -> AuditRecord | None:
        """The validated record `log` would write, or None when nothing
        happened on this tick (no violation, no FSM event). `kind`
        ("stop_advised", AuditRecord.decision_kind) marks a decision a rail
        flew instead of the tick's filter() decision; it is always recorded."""
        if kind is not None and kind not in DECISION_KINDS:
            raise ValueError(f"kind must be one of {DECISION_KINDS} or None, got {kind!r}")
        fsm = fsm_record if fsm_record is not None else (decision.fsm_record or {})
        # A new hold point (the Shield's _escalate marks it `hold_chosen`) is
        # an FSM event too: a Loiter that re-plans on a tick with no
        # violation and no transition still goes on record.
        fsm_event = bool(fsm.get("transition") or fsm.get("set_mode")
                         or decision.set_mode or decision.fsm_fault
                         or fsm.get("hold_chosen"))
        if not decision.touched and not fsm_event and kind is None:
            return None
        trace = decision.trace or {}
        edge = set(trace.get("window_edge") or ())
        prio = self._priorities()
        yaw = trace.get("yaw_deg")
        # The pass each Repair came from, index for index with
        # decision.repairs (Shield._traced). A first version mapped it by
        # operator NAME through the ledger, so a second repair the same
        # operator logged in a later pass (the ClimbClamp closing pass 1 after
        # one in pass 0) was written with the first one's pass: wrong on 1324
        # of 4257 seeded records (review, 2026-10-13; 0 now).
        its = trace.get("repair_iterations")
        if its is not None and len(its) != len(decision.repairs):
            its = None

        def violation(v) -> dict:
            # The grant's category; ours as rule_category (record version 4).
            return {"rule_id": v.rule_id,
                    "category": ("time-window" if v.rule_id in edge
                                 else GRANT_CATEGORY.get(v.category, v.category)),
                    "boundary_intersect_at_s": v.predicted_at_s,
                    "rule_category": v.category, "detail": v.detail,
                    "predicted_at_s": v.predicted_at_s,
                    "priority": prio.get(v.rule_id)}

        passes = trace.get("passes") if trace else None
        rec = {
            "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "policy_hash": decision.policy_hash or self.policy_hash,
            "generation": (decision.generation if decision.generation is not None
                           else self.generation),
            "monitor_tick": tick,
            "tick": tick,
            "episode_id": self.episode_id,
            # The grant's body frame, converted here at the tick's heading;
            # the Shield's own world frame beside it.
            "raw_action": _body(decision.raw, yaw),
            "raw_action_world": decision.raw.model_dump(),
            "violations": [violation(v) for v in decision.violations],
            "repairs": [r.model_dump() for r in decision.repairs],
            # The grant's: every operator tried, skips included.
            "repair_attempts": operator_attempts(passes),
            "repair_results": [{
                "operator": r.operator, "result": _attempt_result(r.operator),
                "reason": r.detail, "magnitude_m": r.magnitude_m, "axis": r.axis,
                "recovery": r.recovery,
                "iterations": its[i] if its is not None else None}
                for i, r in enumerate(decision.repairs)],
            "emitted_action": _body(decision.emitted, yaw),
            "emitted_action_world": decision.emitted.model_dump(),
            # The Shield's re-check of `emitted_action` (the repair stack's
            # output), of ANY priority: it also lists monitor_only rules and
            # soft rules given up (`relaxed`), so filter on priority == "P0"
            # for the grant's escape KPI. It is the FLOWN action only when the
            # Shield is on and `fsm_setpoint` is "pass" (or null: no FSM). On
            # "brake" a zero action flew, on "none" nothing did, and on a
            # Shield-off run the raw action flew. The KPI's escape count comes
            # from the rails' flown re-check (guardrail.replay.flown_fields),
            # not from this file. (Until 2026-10-19 this comment called it the
            # flown action and any non-empty list a P0 escape.)
            "emitted_violations": [violation(v) for v in decision.emitted_violations],
            "braked": decision.braked,
            "relaxed": list(decision.relaxed),
            "fsm_state_before": fsm.get("fsm_state_before", decision.fsm_state_before),
            "fsm_state_after": fsm.get("fsm_state_after", decision.fsm_state_after),
            "fsm_edge": fsm.get("edge", decision.fsm_edge),
            "fsm_fault": decision.fsm_fault,
        }
        for k in _FSM_FIELDS[3:]:
            rec[f"fsm_{k}" if not k.startswith("fsm_") else k] = fsm.get(k)
        if rec.get("fsm_set_mode") is None and decision.set_mode is not None:
            rec["fsm_set_mode"] = decision.set_mode
        rec.update({
            "record_version": RECORD_VERSION,
            "body_yaw_deg": yaw,
            "repair_mode": trace.get("mode"),
            "iterations": trace.get("iterations"),
            "chain_runs": trace.get("chain_runs"),
            "flown_run": trace.get("flown_run"),
            "repair_passes": passes,
            # The plan in force on a Loiter / Land tick: the FSM record's copy
            # (updated by the Shield when it chose one this tick), else the
            # decision's own; None in every other state.
            "fsm_hold_point": fsm.get("hold_point", decision.hold_point),
        })
        if kind is not None:
            rec["decision_kind"] = kind
        return AuditRecord.model_validate(rec)

    def log(self, tick: int, decision: ShieldDecision,
            fsm_record: dict | None = None, *, kind: str | None = None) -> None:
        """Write a record when something happened: a violation seen, or an
        FSM transition / mode request / refused input on this tick.
        `fsm_record` is an FSMOutput.record from a rail that runs its own FSM;
        by default the decision's own (`ShieldDecision.fsm_record`) is used.
        `kind` tags a decision the rail flew instead of the tick's filter()
        decision (AuditRecord.decision_kind), always written.
        The record is validated (AuditRecord) before it is written."""
        if self.path is None:
            raise RuntimeError("an in-memory AuditLogger has no file: keep record() / line()")
        rec = self.record(tick, decision, fsm_record, kind=kind)
        if rec is None:
            return
        # Absent decision_kind means the tick's filter() decision: every record
        # without a kind is written exactly as before the field existed.
        line = self.line(rec)
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(line) + "\n")
        self._n += 1

    @property
    def records_written(self) -> int:
        return self._n


if __name__ == "__main__":                   # python -m guardrail.audit schema [path]
    if len(sys.argv) >= 2 and sys.argv[1] == "schema":
        print(write_audit_record_schema(*(sys.argv[2:3] or [SCHEMA_PATH])))
    else:
        print("usage: python -m guardrail.audit schema [path]")
        sys.exit(2)
