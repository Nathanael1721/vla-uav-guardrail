"""
Audit log — one JSONL record per Shield decision that touched the action.

Mirrors the grant's audit-log rule: every record carries policy_hash so any
KPI number can be traced back to the exact policy that produced it.

THE GRANT'S RECORD (Safety Shield page, "Audit log"), field by field

    ts                 ISO-8601 UTC, milliseconds
    policy_hash        the hash of the rule set the decision was made under
                       (ShieldDecision.policy_hash, taken when the tick began;
                       a hot-apply between ticks cannot restamp a decision)
    generation         that rule set's generation (bumps on every hot-apply)
    monitor_tick       the caller's tick counter (also kept as `tick`)
    raw_action         this project's Action4D: vz is spelled `vz_up` (+up)
    violations         each with `boundary_intersect_at_s` (= predicted_at_s,
                       when in the lookahead the rule breaks). The grant's
                       `category` (geometric / envelope / time-window) is
                       written as `grant_category`; the record's own
                       `category` field is NOT the grant's: it is this
                       project's finer vocabulary (geofence, corridor,
                       altitude, kinematic, clearance, standoff, contract),
                       kept under that name for the readers that already
                       use it. `contract` (a non-finite command zeroed) is
                       this project's extension, not a grant category, and
                       `time-window` never occurs: a rule out of its window
                       raises nothing here
    repair_attempts    operator, result (ok / fallback / relaxed), reason,
                       magnitude_m, axis, recovery. The grant's `iterations`
                       is not written: the repair chain iterates as a whole
                       (up to 3 passes) and does not count per operator, and
                       a number nobody measured is not written down
    emitted_action     what the repair stack produced
    fsm_state_before / fsm_state_after
                       the escalation FSM's state around this tick, with its
                       edge, edge source (grant / extension), the mode it
                       requested, the setpoint it streamed and the config hash

plus `emitted_violations`, `braked`, `relaxed` and `fsm_fault`. A record is
written when the decision saw a violation, and also on a tick where the FSM
changed state, requested a mode or refused its input, so a Brake -> Normal
recovery on a clean tick is on record too.

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
from datetime import datetime, timezone
from pathlib import Path

from .shield import ShieldDecision

# This project's violation categories in the grant's three (Safety Shield page,
# "Violation checking": geometric / envelope / time-window). A time window is
# not a violation of its own here: a rule out of its window raises nothing.
# "contract" (the Shield's own non-finite-command check) has no grant
# category and is written as itself: an extension, not one of the three.
GRANT_CATEGORY = {
    "geofence": "geometric", "corridor": "geometric",
    "altitude": "envelope", "kinematic": "envelope", "clearance": "envelope",
    "standoff": "envelope", "contract": "contract",
}

_FALLBACK_OPERATORS = ("Brake", "ClearanceEscape")
_FSM_FIELDS = ("fsm_state_before", "fsm_state_after", "edge", "edge_source",
               "set_mode", "setpoint", "reason", "fsm_config_hash")
_SAFE_ID = re.compile(r"^[A-Za-z0-9._-]+$")


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

    def log(self, tick: int, decision: ShieldDecision,
            fsm_record: dict | None = None) -> None:
        """Write a record when something happened: a violation seen, or an
        FSM transition / mode request / refused input on this tick.
        `fsm_record` is an FSMOutput.record from a rail that runs its own FSM;
        by default the decision's own (`ShieldDecision.fsm_record`) is used."""
        fsm = fsm_record if fsm_record is not None else (decision.fsm_record or {})
        fsm_event = bool(fsm.get("transition") or fsm.get("set_mode")
                         or decision.set_mode or decision.fsm_fault)
        if not decision.touched and not fsm_event:
            return
        rec = {
            "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "policy_hash": decision.policy_hash or self.policy_hash,
            "generation": (decision.generation if decision.generation is not None
                           else self.generation),
            "monitor_tick": tick,
            "tick": tick,
            "episode_id": self.episode_id,
            "raw_action": decision.raw.model_dump(),
            "violations": [{**v.model_dump(),
                            "boundary_intersect_at_s": v.predicted_at_s,
                            "grant_category": GRANT_CATEGORY.get(v.category, v.category)}
                           for v in decision.violations],
            "repairs": [r.model_dump() for r in decision.repairs],
            "repair_attempts": [{
                "operator": r.operator,
                "result": ("fallback" if r.operator in _FALLBACK_OPERATORS
                           else "relaxed" if r.operator == "SoftRelax" else "ok"),
                "reason": r.detail, "magnitude_m": r.magnitude_m, "axis": r.axis,
                "recovery": r.recovery} for r in decision.repairs],
            "emitted_action": decision.emitted.model_dump(),
            # What is still wrong with the action that was FLOWN. Empty is the
            # good case; non-empty is a P0 escape, the grant's hard KPI.
            "emitted_violations": [v.model_dump()
                                   for v in decision.emitted_violations],
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
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(rec) + "\n")
        self._n += 1

    @property
    def records_written(self) -> int:
        return self._n
