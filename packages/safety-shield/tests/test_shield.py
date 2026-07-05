import math
from pathlib import Path

from policy_dsl import ingest_text
from safety_shield import (
    AuditLog,
    SafetyShield,
    ShieldState,
    VehicleState,
    check,
    repair_action,
)
from vlaguard_common import Action4D

DEMO = Path(__file__).resolve().parents[3] / "bundles" / "itri-icl-2026-demo.yaml"


def _ir():
    return ingest_text(DEMO.read_text())


def _state_west_of_nfz(alt=50.0):
    # ~30 m west of the school-yard polygon edge (lon 121.5310), heading East
    # (yaw=pi/2) toward it; at 10 m/s the 5 s horizon reaches well inside.
    return VehicleState(lat=25.04245, lon=121.5307, alt_agl_m=alt, yaw_rad=math.pi / 2)


def test_clean_action_passes_through():
    ir = _ir()
    shield = SafetyShield(ir)
    # heading West (away from the NFZ which is to the East)
    state = VehicleState(lat=25.04245, lon=121.5300, alt_agl_m=50, yaw_rad=-math.pi / 2)
    decision = shield.tick(state, Action4D(vx=5, vy=0, vz=0, yaw_rate=0))
    assert decision.violations == []
    assert decision.state_after == ShieldState.NORMAL
    assert not decision.intercepted


def test_geometric_violation_detected():
    ir = _ir()
    state = _state_west_of_nfz()
    # fly East at 10 m/s toward the NFZ -> predicted trajectory enters it
    violations = check(ir, state, Action4D(vx=10, vy=0, vz=0, yaw_rate=0))
    assert any(v.category == "geometric" and v.rule_id == "nfz-school-yard" for v in violations)


def test_lateral_projection_repairs_glancing_clip():
    # glancing approach: skim just inside the west wall heading North -> shallow
    # penetration -> lateral projection nudges the trajectory back out.
    ir = _ir()
    state = VehicleState(lat=25.0420, lon=121.531012, alt_agl_m=50, yaw_rad=0.0)
    action = Action4D(vx=5, vy=0, vz=0, yaw_rate=0)
    violations = check(ir, state, action)
    assert any(v.rule_id == "nfz-school-yard" for v in violations)
    outcome = repair_action(ir, state, action, violations)
    assert outcome.converged
    assert any(a.operator == "LateralProjection" and a.result == "ok" for a in outcome.attempts)
    assert check(ir, state, outcome.action) == []


def test_deep_head_on_dive_escalates_not_projects():
    # deep penetration exceeds the lateral threshold -> repair fails -> escalate
    ir = _ir()
    state = _state_west_of_nfz()
    action = Action4D(vx=10, vy=0, vz=0, yaw_rate=0)
    outcome = repair_action(ir, state, action, check(ir, state, action))
    assert not outcome.converged


def test_envelope_violation_clamped():
    ir = _ir()
    # climbing hard from 110m toward >120m ceiling, heading West (no geo violation)
    state = VehicleState(lat=25.04245, lon=121.5290, alt_agl_m=110, yaw_rad=-math.pi / 2)
    action = Action4D(vx=2, vy=0, vz=5, yaw_rate=0)
    violations = check(ir, state, action)
    assert any(v.category == "envelope" for v in violations)
    outcome = repair_action(ir, state, action, violations)
    assert outcome.action.vz < action.vz  # climb was clamped


def test_p0_escape_rate_zero_for_demo_attack():
    # KPI gate: the Shield must not let the predicted trajectory escape into a P0 NFZ.
    ir = _ir()
    shield = SafetyShield(ir)
    state = _state_west_of_nfz()
    decision = shield.tick(state, Action4D(vx=10, vy=0, vz=0, yaw_rate=0))
    assert decision.intercepted
    # emitted action's predicted trajectory does not enter the P0 polygon
    residual = [v for v in check(ir, state, decision.emitted_action) if v.priority == "P0"]
    assert residual == []


def test_audit_log_carries_policy_hash(tmp_path):
    ir = _ir()
    audit = AuditLog(tmp_path / "audit.jsonl", ir.policy_hash, ir.generation)
    shield = SafetyShield(ir, audit=audit)
    state = _state_west_of_nfz()
    shield.tick(state, Action4D(vx=10, vy=0, vz=0, yaw_rate=0), ts="2026-06-23T00:00:00Z")
    audit.close()
    lines = (tmp_path / "audit.jsonl").read_text().strip().splitlines()
    assert len(lines) == 1
    import json

    rec = json.loads(lines[0])
    assert rec["policy_hash"] == ir.policy_hash
    assert rec["violations"]
