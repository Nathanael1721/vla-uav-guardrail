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

    ts                 ISO-8601 UTC, milliseconds
    policy_hash        the hash of the rule set the decision was made under
                       (ShieldDecision.policy_hash, taken when the tick began;
                       a hot-apply between ticks cannot restamp a decision)
    generation         that rule set's generation (bumps on every hot-apply)
    monitor_tick       the caller's tick counter (also kept as `tick`)
    raw_action         this project's world-frame Action4D: vx North, vy East,
                       vz spelled `vz_up` (+up). The grant's body-frame form,
                       keys {vx, vy, vz, yaw_rate}, is `raw_action_body`
                       (guardrail.frames.to_body at the tick's heading,
                       `body_yaw_deg`); null when the decision does not carry
                       the heading (one built outside Shield.filter)
    violations         each with `boundary_intersect_at_s` (= predicted_at_s,
                       when in the lookahead the rule breaks) and the grant's
                       `category` (geometric / envelope / time-window) as
                       `grant_category`; the record's own `category` field is
                       NOT the grant's: it is this project's finer vocabulary
                       (geofence, corridor, altitude, kinematic, clearance,
                       standoff, contract), kept under that name for the
                       readers that already use it. `time-window` is a rule
                       not in force now that the forecast breaks after a
                       window edge brings it into force (the Shield's
                       `ShieldDecision.trace["window_edge"]`). `contract` (a
                       non-finite command zeroed) is this project's
                       extension, not a grant category. `priority` when the
                       logger holds the policy
    repair_attempts    one per Repair the decision logged, as before:
                       operator, result (ok / fallback / relaxed), reason,
                       magnitude_m, axis, recovery, and the grant's
                       `iterations`: the pass of the flown run's iterated
                       repair loop in which that repair (operator AND
                       detail) was first logged (0 = before the loop; null
                       for the last resorts after it)
    repair_passes      every repair OPERATOR considered (guardrail/repair.py),
                       pass by pass, in every run of the repair stack (`run`
                       1, 2, ...: more than one when soft rules were relaxed
                       and the stack ran again; `chain_runs` counts them and
                       `flown_run` is the one whose action was flown), with
                       result ok / skip / fail / fallback / relaxed and the
                       reason for a skip. `iterations` is the number of loop
                       passes that ran, summed over the runs (measured, not
                       the configured 3; 0 when no loop ran). The grant's
                       example lists skips inside
                       repair_attempts; here they live in repair_passes so
                       repair_attempts keeps meaning "repairs made" for the
                       readers that count it
    emitted_action     what the repair stack produced (`emitted_action_body`
                       in the grant's body-frame keys)
    fsm_state_before / fsm_state_after
                       the escalation FSM's state around this tick, with its
                       edge, edge source (grant / extension), the mode it
                       requested, the setpoint it streamed and the config hash

plus `emitted_violations`, `braked`, `relaxed`, `fsm_fault`, `repair_mode`,
`flown_run` and `record_version` (2 since AuditRecord; absent, read as 1,
before). A record is written when the decision saw a violation, and also on
a tick where the FSM changed state, requested a mode or refused its input,
so a Brake -> Normal recovery on a clean tick is on record too.

ONE FILE PER EPISODE ("Python module; rotated per mission", Outputs table).
`AuditLogger.for_episode(directory, policy, episode_id)` opens
`audit_<episode_id>.jsonl` fresh; `rotate(episode_id)` starts the next one.
The plain constructor still APPENDS to the path it is given, because the rails
pair it with guardrail/replay.py's EpisodeRecord, which moves a stale
`audit.jsonl` aside before the logger opens it.
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

RECORD_VERSION = 2
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
    rule_id: str
    category: str
    detail: str = ""
    predicted_at_s: float | None = None
    boundary_intersect_at_s: float | None = None
    grant_category: str | None = None
    priority: str | None = None


class RepairRecord(_Strict):
    """One `Repair` of the decision (guardrail/shield.py)."""
    operator: str
    detail: str = ""
    magnitude_m: float | None = None
    axis: str | None = None
    recovery: bool = False


class RepairAttemptRecord(_Strict):
    operator: str
    result: Result
    reason: str = ""
    magnitude_m: float | None = None
    axis: str | None = None
    recovery: bool = False
    iterations: int | None = None


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
    """One audit line. Field order is the order written."""
    ts: str
    policy_hash: str
    generation: int | None = None
    monitor_tick: int
    tick: int
    episode_id: str | int | None = None
    raw_action: WorldAction
    violations: list[ViolationRecord] = []
    repairs: list[RepairRecord] = []
    repair_attempts: list[RepairAttemptRecord] = []
    emitted_action: WorldAction
    # None on a stored record from before the field existed: not measured,
    # which is not the same as "nothing escaped".
    emitted_violations: list[ViolationRecord] | None = None
    braked: bool = False
    relaxed: list[str] | None = None
    fsm_state_before: str | None = None
    fsm_state_after: str | None = None
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
    raw_action_body: BodyAction | None = None
    emitted_action_body: BodyAction | None = None
    repair_mode: str | None = None
    iterations: int | None = None
    chain_runs: int | None = None
    flown_run: int | None = None
    repair_passes: list[RepairPass] | None = None


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


def upgrade_record(raw: dict) -> dict:
    """A stored record of any shape this project has written, in the current
    shape. Three are on disk: July-August 2026 (no emitted_violations), late
    August to October (emitted_violations, no generation or FSM fields), and
    2026-10-07 onward (the grant's fields). What a shape did not record stays
    None - an absent `emitted_violations` is "not measured", never "none".
    `repair_attempts` of a record that has none is derived from its `repairs`
    exactly as the writer of the third shape derived it."""
    r = dict(raw)
    r.setdefault("record_version", 1)
    if r.get("monitor_tick") is None:
        r["monitor_tick"] = r.get("tick")
    if r.get("tick") is None:
        r["tick"] = r.get("monitor_tick")
    r["violations"] = [_upgrade_violation(v) for v in r.get("violations") or []]
    if r.get("emitted_violations") is not None:
        r["emitted_violations"] = [_upgrade_violation(v) for v in r["emitted_violations"]]
    if "repair_attempts" not in r:
        r["repair_attempts"] = [{
            "operator": rep.get("operator"), "result": _attempt_result(rep.get("operator")),
            "reason": rep.get("detail", ""), "magnitude_m": rep.get("magnitude_m"),
            "axis": rep.get("axis"), "recovery": bool(rep.get("recovery", False))}
            for rep in r.get("repairs") or []]
    return r


def _upgrade_violation(v: dict) -> dict:
    v = dict(v)
    if "boundary_intersect_at_s" not in v:
        v["boundary_intersect_at_s"] = v.get("predicted_at_s")
    if "grant_category" not in v:
        v["grant_category"] = GRANT_CATEGORY.get(v.get("category"), v.get("category"))
    return v


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
    def for_episode(cls, directory: str | Path, policy: "str | object",
                    episode_id: str) -> "AuditLogger":
        """A fresh `audit_<episode_id>.jsonl` in `directory`, one per episode."""
        return cls(Path(directory) / cls._episode_name(episode_id), policy,
                   episode_id=episode_id, fresh=True)

    @staticmethod
    def _episode_name(episode_id: str) -> str:
        if not isinstance(episode_id, str) or not _SAFE_ID.match(episode_id):
            raise ValueError(f"episode_id {episode_id!r} must be letters, digits, "
                             f"'.', '_' or '-' (it names a file)")
        return f"audit_{episode_id}.jsonl"

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
               fsm_record: dict | None = None) -> AuditRecord | None:
        """The validated record `log` would write, or None when nothing
        happened on this tick (no violation, no FSM event)."""
        fsm = fsm_record if fsm_record is not None else (decision.fsm_record or {})
        fsm_event = bool(fsm.get("transition") or fsm.get("set_mode")
                         or decision.set_mode or decision.fsm_fault)
        if not decision.touched and not fsm_event:
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
            return {**v.model_dump(), "boundary_intersect_at_s": v.predicted_at_s,
                    "grant_category": ("time-window" if v.rule_id in edge
                                       else GRANT_CATEGORY.get(v.category, v.category)),
                    "priority": prio.get(v.rule_id)}

        rec = {
            "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "policy_hash": decision.policy_hash or self.policy_hash,
            "generation": (decision.generation if decision.generation is not None
                           else self.generation),
            "monitor_tick": tick,
            "tick": tick,
            "episode_id": self.episode_id,
            "raw_action": decision.raw.model_dump(),
            "violations": [violation(v) for v in decision.violations],
            "repairs": [r.model_dump() for r in decision.repairs],
            "repair_attempts": [{
                "operator": r.operator, "result": _attempt_result(r.operator),
                "reason": r.detail, "magnitude_m": r.magnitude_m, "axis": r.axis,
                "recovery": r.recovery,
                "iterations": its[i] if its is not None else None}
                for i, r in enumerate(decision.repairs)],
            "emitted_action": decision.emitted.model_dump(),
            # What is still wrong with the action that was FLOWN. Empty is the
            # good case; non-empty is a P0 escape, the grant's hard KPI.
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
            "raw_action_body": _body(decision.raw, yaw),
            "emitted_action_body": _body(decision.emitted, yaw),
            "repair_mode": trace.get("mode"),
            "iterations": trace.get("iterations"),
            "chain_runs": trace.get("chain_runs"),
            "flown_run": trace.get("flown_run"),
            "repair_passes": trace.get("passes") if trace else None,
        })
        return AuditRecord.model_validate(rec)

    def log(self, tick: int, decision: ShieldDecision,
            fsm_record: dict | None = None) -> None:
        """Write a record when something happened: a violation seen, or an
        FSM transition / mode request / refused input on this tick.
        `fsm_record` is an FSMOutput.record from a rail that runs its own FSM;
        by default the decision's own (`ShieldDecision.fsm_record`) is used.
        The record is validated (AuditRecord) before it is written."""
        rec = self.record(tick, decision, fsm_record)
        if rec is None:
            return
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(rec.model_dump(by_alias=True)) + "\n")
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
