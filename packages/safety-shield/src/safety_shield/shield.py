"""Safety Shield orchestrator — one monitor tick (safety-shield.md node diagram).

Pure, ROS-free: ``tick()`` takes the current vehicle state + a candidate 4-D
action and returns a :class:`ShieldDecision` (the action to emit, the FSM state,
violations, repair attempts). The rclpy node in ``ros2_ws/`` wraps this; keeping
the core ROS-free is what lets it run in unit tests and CI without a ROS install.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from policy_dsl import PolicyIR
from vlaguard_common import Action4D

from safety_shield.audit import AuditLog
from safety_shield.checker import Violation, check
from safety_shield.fsm import ShieldState, next_state
from safety_shield.kinematics import VehicleState
from safety_shield.repair import RepairAttempt, RepairConfig, RepairOutcome, repair_action


@dataclass
class ShieldDecision:
    emitted_action: Action4D
    state_before: ShieldState
    state_after: ShieldState
    violations: list[Violation]
    repair_attempts: list[RepairAttempt] = field(default_factory=list)
    intercepted: bool = False


class SafetyShield:
    """Stateful Shield: holds the active IR, FSM state, repair config, audit log."""

    def __init__(
        self,
        ir: PolicyIR,
        cfg: RepairConfig | None = None,
        audit: AuditLog | None = None,
    ) -> None:
        self.ir = ir
        self.cfg = cfg or RepairConfig()
        self.audit = audit
        self.state = ShieldState.NORMAL
        self.tick_count = 0

    def tick(self, state: VehicleState, action: Action4D, ts: str = "") -> ShieldDecision:
        self.tick_count += 1
        before = self.state
        violations = check(self.ir, state, action)

        if not violations:
            self.state = ShieldState.NORMAL
            return ShieldDecision(action, before, self.state, [], intercepted=False)

        worst = violations[0]
        # Rules whose action is a direct fail-safe skip projection entirely.
        if worst.violation_action in ("RTL", "land"):
            self.state = next_state(
                before, has_violation=True, repaired=False, direct_action=worst.violation_action
            )
            decision = ShieldDecision(action, before, self.state, violations, [], intercepted=True)
            self._audit(ts, action, violations, [], action, before, self.state)
            return decision

        outcome: RepairOutcome = repair_action(self.ir, state, action, violations, self.cfg)
        self.state = next_state(
            before,
            has_violation=True,
            repaired=outcome.converged,
            direct_action=None if outcome.converged else worst.violation_action,
        )
        emitted = outcome.action if outcome.converged else _brake()
        decision = ShieldDecision(
            emitted, before, self.state, violations, outcome.attempts, intercepted=True
        )
        self._audit(ts, action, violations, outcome.attempts, emitted, before, self.state)
        return decision

    def _audit(
        self,
        ts: str,
        raw: Action4D,
        violations: list[Violation],
        attempts: list[RepairAttempt],
        emitted: Action4D,
        before: ShieldState,
        after: ShieldState,
    ) -> None:
        if self.audit is not None:
            self.audit.write(
                ts=ts,
                monitor_tick=self.tick_count,
                raw_action=raw,
                violations=violations,
                repair_attempts=attempts,
                emitted_action=emitted,
                fsm_state_before=before,
                fsm_state_after=after,
            )


def _brake() -> Action4D:
    """A full stop — emitted when repair fails but no direct fail-safe applies."""
    return Action4D(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
