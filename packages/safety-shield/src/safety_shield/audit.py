"""Audit-log JSONL writer (safety-shield.md "Audit log").

Every monitor tick that produces a violation event writes one JSONL record
carrying the ``policy_hash`` + ``generation`` of the active bundle — the
reproducibility key. Records are aggregated into the episode bundle by the Stress
Testing harness (Phase 3).
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any, TextIO

from vlaguard_common import Action4D

from safety_shield.checker import Violation
from safety_shield.repair import RepairAttempt


class AuditLog:
    """Append-only JSONL writer. One record per violating monitor tick."""

    def __init__(self, path: str | Path, policy_hash: str, generation: int) -> None:
        self.path = Path(path)
        self.policy_hash = policy_hash
        self.generation = generation
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh: TextIO = self.path.open("a", encoding="utf-8")

    def write(
        self,
        *,
        ts: str,
        monitor_tick: int,
        raw_action: Action4D,
        violations: list[Violation],
        repair_attempts: list[RepairAttempt],
        emitted_action: Action4D,
        fsm_state_before: str,
        fsm_state_after: str,
    ) -> dict[str, Any]:
        record = {
            "ts": ts,
            "policy_hash": self.policy_hash,
            "generation": self.generation,
            "monitor_tick": monitor_tick,
            "raw_action": raw_action.model_dump(),
            "violations": [
                {
                    "rule_id": v.rule_id,
                    "category": v.category,
                    "priority": v.priority,
                    "boundary_intersect_at_s": v.first_hit_s,
                }
                for v in violations
            ],
            "repair_attempts": [asdict(a) for a in repair_attempts],
            "emitted_action": emitted_action.model_dump(),
            "fsm_state_before": fsm_state_before,
            "fsm_state_after": fsm_state_after,
        }
        self._fh.write(json.dumps(record) + "\n")
        self._fh.flush()
        return record

    def close(self) -> None:
        self._fh.close()

    def __enter__(self) -> AuditLog:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
