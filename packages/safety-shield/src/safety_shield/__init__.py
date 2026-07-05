"""WP3 — Safety Shield pure core: monitor -> repair -> escalate (ROS-free)."""

from safety_shield.audit import AuditLog
from safety_shield.checker import Violation, check
from safety_shield.fsm import ShieldState, next_state
from safety_shield.kinematics import VehicleState, predict
from safety_shield.repair import (
    AltitudeClamp,
    LateralProjection,
    RepairConfig,
    RepairOutcome,
    repair_action,
)
from safety_shield.shield import SafetyShield, ShieldDecision

__all__ = [
    "SafetyShield",
    "ShieldDecision",
    "ShieldState",
    "next_state",
    "VehicleState",
    "predict",
    "Violation",
    "check",
    "AuditLog",
    "RepairConfig",
    "RepairOutcome",
    "repair_action",
    "AltitudeClamp",
    "LateralProjection",
]
