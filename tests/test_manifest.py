"""The WP4 determinism manifest and the four locked acceptance KPIs.

Run either way:
    pytest tests/test_manifest.py -v
    python tests/test_manifest.py

The grant allows contractual KPI numbers only from a run that carries a six-field
determinism manifest at `sim_speedup=1.0`. Our flights carried none of it, so on
the grant's own terms nothing measured here was reportable.

The most important test in this file is the one that makes the hard KPI FAIL. A
P0-escape counter that only ever sees compliant logs proves nothing: it would
report a perfect score for a broken Shield. So a log where a P0 violation is
detected and then flown anyway must come out non-zero.
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import kpi as K                                        # noqa: E402
from guardrail.manifest import (MAX_HEADING_ERR_DEG, MIN_DET_HZ,      # noqa: E402
                                TOPOLOGY_CANONICAL_HIL,
                                TOPOLOGY_PROJECTAIRSIM, build_manifest,
                                is_kpi_grade, sim_speedup_from_scene)

SCENE = ROOT / "demo" / "pas_config" / "scene_semantic.jsonc"
GOOD_METRICS = {"det_hz": 4.5, "start_heading_err_deg": 1.5}


def _man(**kw):
    base = dict(policy_hash="sha256:deadbeefdeadbeef",
                model_id="google/owlvit-base-patch32", seed=42,
                scene_path=str(SCENE))
    base.update(kw)
    return build_manifest(**base)


# ------------------------------------------------------------------ manifest

def test_the_manifest_has_exactly_the_six_grant_fields():
    m = _man()
    assert set(m) == {"code_revision", "vla_model_hash", "policy_hash",
                      "random_seed", "sim_speedup", "topology"}, sorted(m)


def test_the_same_inputs_give_a_byte_identical_manifest():
    a, b = _man(), _man()
    assert json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)


def test_changing_any_field_changes_the_manifest():
    base = json.dumps(_man(), sort_keys=True)
    assert json.dumps(_man(seed=43), sort_keys=True) != base
    assert json.dumps(_man(policy_hash="sha256:0"), sort_keys=True) != base
    assert json.dumps(_man(model_id="other/model"), sort_keys=True) != base


def test_sim_speedup_is_derived_from_the_scene_not_asserted():
    """A hardcoded 1.0 would defeat the requirement it exists to enforce."""
    assert sim_speedup_from_scene(SCENE) == 1.0


def test_a_fast_clock_is_detected_and_disqualifies_the_run(tmp=Path("_t_fastclock.jsonc")):
    """`sim_speedup=1.0` is mandatory for a KPI-bearing run, so a scene running
    faster than real time has to be caught from the config."""
    try:
        tmp.write_text(json.dumps({
            "id": "fast", "clock": {"type": "steppable", "step-ns": 3000000,
                                    "real-time-update-rate": 12000000}}),
            encoding="utf-8")
        assert sim_speedup_from_scene(tmp) == 4.0
        ok, why = is_kpi_grade(_man(scene_path=str(tmp)), GOOD_METRICS)
        assert not ok and any("sim_speedup" in r for r in why), why
    finally:
        tmp.unlink(missing_ok=True)


def test_an_unreadable_scene_is_unresolved_not_assumed_real_time():
    m = _man(scene_path="does/not/exist.jsonc")
    assert m["sim_speedup"] == "unresolved"
    ok, why = is_kpi_grade(m, GOOD_METRICS)
    assert not ok and any("sim_speedup" in r for r in why)


def test_our_rail_may_not_claim_the_canonical_hil_topology():
    """The field exists so the difference cannot be blurred. Our flights are
    Project AirSim; the grant's KPI topology is ArduPilot SITL + MAVROS."""
    try:
        _man(topology=TOPOLOGY_CANONICAL_HIL)
    except ValueError as e:
        assert "canonical-hil" in str(e)
    else:
        raise AssertionError("a Project AirSim run was allowed to claim canonical-hil")


# --------------------------------------------------------------- kpi grading

def test_the_project_airsim_rail_is_never_kpi_grade():
    """Honest ceiling on everything this repo can currently claim."""
    ok, why = is_kpi_grade(_man(), GOOD_METRICS)
    assert not ok
    assert any(TOPOLOGY_PROJECTAIRSIM in r for r in why), why


def test_a_stalled_detector_disqualifies_the_run():
    """The real failure: 29 inferences at 0.52 Hz reported det_hit_rate 1.000 on a
    flight that tracked for 13.6% of ticks."""
    ok, why = is_kpi_grade(_man(), {"det_hz": 0.52, "start_heading_err_deg": 1.0})
    assert not ok and any("Hz" in r for r in why), why


def test_a_missed_start_heading_disqualifies_the_run():
    """The real failure: the heading was never commanded, and swung the traffic hit
    rate 0.740 -> 0.331 on identical configuration."""
    ok, why = is_kpi_grade(_man(), {"det_hz": 4.5, "start_heading_err_deg": 52.1})
    assert not ok and any("heading" in r for r in why), why
    assert MAX_HEADING_ERR_DEG < 52.1 and MIN_DET_HZ > 0.52


def test_an_unresolved_field_disqualifies_the_run():
    ok, why = is_kpi_grade(_man(policy_hash=""), GOOD_METRICS)
    assert not ok and any("policy_hash" in r for r in why), why


# ------------------------------------------------------------- the hard KPI

def _tick(rule="nfz-route", repaired=False, braked=False, moved=False):
    raw = {"vx": 1.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}
    em = dict(raw)
    if moved:
        em["vy"] = 0.7
    return {"violations": [{"rule_id": rule, "category": "geofence"}],
            "repairs": ([{"operator": "GeofenceProject"}] if repaired else []),
            "braked": braked, "raw": raw, "emitted": em}


PRIOS = {"nfz-route": "P0", "kin-caps": "P1"}


def test_a_p0_flown_anyway_IS_counted_as_an_escape():
    """THE TEST THAT MATTERS. A counter that only sees compliant logs would report
    a perfect score for a broken Shield."""
    k = K.compute([_tick()] * 10, PRIOS)
    assert k["p0_escapes"] == 10
    assert k["p0_violation_escape_rate"] == 1.0
    assert k["outcome"] == "fail" and k["mission_success"] is False


def test_a_p0_that_was_repaired_is_not_an_escape():
    """Detected-and-repaired is the Shield working, and must not count against it."""
    k = K.compute([_tick(repaired=True, moved=True)] * 10, PRIOS)
    assert k["p0_escapes"] == 0 and k["p0_violation_escape_rate"] == 0.0
    assert k["repair_count"] == 10
    assert k["failsafe_trigger_correctness"] == 1.0


def test_braking_also_counts_as_acting_on_a_p0():
    k = K.compute([_tick(braked=True)] * 5, PRIOS)
    assert k["p0_escapes"] == 0


def test_a_p1_violation_is_not_a_p0_escape():
    k = K.compute([_tick(rule="kin-caps")] * 8, PRIOS)
    assert k["p0_escapes"] == 0
    assert k["violations_by_risk_level"] == {"P1": 8}


def test_an_unknown_rule_is_treated_as_p0():
    """Conservative direction on purpose: a rule the policy does not name must not
    silently downgrade the hard KPI."""
    k = K.compute([_tick(rule="who-knows")] * 3, PRIOS)
    assert k["p0_escapes"] == 3


def test_a_clean_flight_scores_zero_escapes_and_succeeds():
    rows = [{"violations": [], "repairs": [], "raw": {}, "emitted": {}}] * 50
    k = K.compute(rows, PRIOS, {"nfz_s": 0.0, "alt_violation_s": 0.0,
                                "frac_within_30m": 1.0})
    assert k["p0_violation_escape_rate"] == 0.0
    assert k["outcome"] == "success" and k["mission_success"] is True
    assert k["failsafe_trigger_correctness"] is None      # nothing to trigger on


def test_time_inside_a_fence_fails_the_mission_even_with_no_escape():
    rows = [{"violations": [], "repairs": [], "raw": {}, "emitted": {}}] * 20
    k = K.compute(rows, PRIOS, {"nfz_s": 3.2, "alt_violation_s": 0.0})
    assert k["outcome"] == "fail" and k["mission_success"] is False


def test_priorities_come_from_the_policy_constraints_list():
    """An earlier version guessed per-type attribute names, silently returned {},
    and defaulted every rule to P0."""
    from guardrail import load_policy
    prios = K.rule_priorities(load_policy(ROOT / "policies" / "follow_car_nfz.yaml"))
    assert prios, "no priorities resolved from the policy"
    assert set(prios.values()) <= {"P0", "P1", "P2"}
    assert "P0" in prios.values()


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
