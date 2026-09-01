"""The scenario sweep harness, and the altitude defect it found.

Run either way:
    pytest tests/test_sweep.py -v
    python tests/test_sweep.py

WHY THIS FILE EXISTS

A test harness needs its own tests more than most code does, because its failure
mode is silence: a sweep that runs nothing and prints "11 passed" is worse than
no sweep at all. So the tests below are mostly about the harness being HONEST -
that a skipped scenario says skipped, that a gate on an unmeasured field fails
rather than passes, that a known failure is never reported as a pass, and that
the unshielded control arm can still score a real violation.

The last group covers the defect the sweep found on its first run.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import yaml                                                       # noqa: E402

import sweep_scenarios as S                                       # noqa: E402
from guardrail import load_policy                                 # noqa: E402
from guardrail.models import Action4D, State                      # noqa: E402
from guardrail.shield import Shield                               # noqa: E402

LIB = yaml.safe_load((ROOT / "experiments" / "scenarios.yaml")
                     .read_text(encoding="utf-8"))
DEFAULTS = LIB.get("defaults", {})
BY_ID = {s["id"]: s for s in LIB["scenarios"]}


def _run(sid):
    return S.run_headless(BY_ID[sid], DEFAULTS)


# --------------------------------------------------------------------------- #
# the harness tells the truth
# --------------------------------------------------------------------------- #

def test_a_gate_on_an_unmeasured_field_fails():
    """The quiet way a harness stops testing anything.

    If a field is missing - renamed, never computed, spelled wrong - the gate
    must FAIL. Treating absent as satisfied would let a typo silently retire a
    safety check while the sweep kept printing green.
    """
    bad = S.check_gates({"no_such_field": {"max": 0.0}}, {"other": 1})
    assert bad and "not measured" in bad[0], bad


def test_gates_compare_in_the_direction_they_say():
    assert not S.check_gates({"v": {"max": 1.0}}, {"v": 1.0})
    assert S.check_gates({"v": {"max": 1.0}}, {"v": 1.1})
    assert not S.check_gates({"v": {"min": 1.0}}, {"v": 1.0})
    assert S.check_gates({"v": {"min": 1.0}}, {"v": 0.9})
    assert S.check_gates({"v": {"equals": True}}, {"v": False})


def test_every_scenario_has_a_reason_and_at_least_one_gate():
    """A scenario with no gate is a demonstration, not a test."""
    for s in LIB["scenarios"]:
        assert s.get("why", "").strip(), f"{s['id']} has no `why`"
        assert s.get("gates"), f"{s['id']} asserts nothing"
        assert (ROOT / s["policy"]).is_file(), f"{s['id']}: missing policy"


def test_the_sitl_backend_reports_unavailable_rather_than_pretending():
    """Returning None means SKIP; the runner prints it and counts it apart.

    The alternative - a backend that silently falls back to headless - would
    report ArduPilot results this host never produced.
    """
    if S.sitl_available():
        print("      SKIP: this host does have the ArduPilot rail")
        return
    assert S.run_sitl(BY_ID["nfz-head-on"], ROOT / "demo" / "out" / "sweep" / "x") is None


# --------------------------------------------------------------------------- #
# the control arm can still fail
# --------------------------------------------------------------------------- #

def test_the_unshielded_control_arm_scores_a_real_escape():
    """If an unguarded flight into a no-fly zone scores clean, every other pass
    in the sweep is worthless."""
    from guardrail import kpi as K
    sc = BY_ID["nfz-head-on-control"]
    rows, extra = _run("nfz-head-on-control")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc["policy"])), {})
    assert res["p0_violation_escape_rate"] > 0.0, res
    assert res["time_to_safe_episodes"] >= 1, res
    assert res["p0_ticks_not_measurable"] == 0, "the control arm must be MEASURED"


def test_the_shielded_arm_of_the_same_flight_is_clean():
    from guardrail import kpi as K
    sc = BY_ID["nfz-head-on"]
    rows, _ = _run("nfz-head-on")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc["policy"])), {})
    assert res["p0_violation_escape_rate"] == 0.0, res
    assert res["time_to_safe_episodes"] == 0, "the shielded arm was never unsafe"


def test_a_nonfinite_command_never_reaches_the_vehicle():
    rows, extra = _run("nonfinite-action")
    assert extra["all_actions_finite"] is True


def test_the_speed_cap_holds_over_the_whole_flight():
    _, extra = _run("speed-cap")
    assert extra["max_speed_flown"] <= 4.001, extra


def test_the_corridor_run_stays_inside_its_half_width():
    _, extra = _run("corridor-drift-out")
    assert extra["max_offset_from_corridor"] <= 20.001, extra


def test_a_clean_corridor_run_is_not_touched_at_all():
    rows, extra = _run("corridor-along")
    assert sum(len(r["repairs"]) for r in rows) == 0, "a legal flight was repaired"
    assert extra["reached_goal"] is True


def test_the_curfew_changes_the_outcome_of_the_same_flight():
    """In hours the fence binds; out of hours it is not in force at all."""
    _, in_hours = _run("corridor-curfew-in-hours")
    _, off_hours = _run("corridor-curfew-out-of-hours")
    assert off_hours["reached_goal"] is True
    assert in_hours["final"] != off_hours["final"], (
        "the schedule made no difference - the gating is not reaching the run")


# --------------------------------------------------------------------------- #
# the wedge, kept visible
# --------------------------------------------------------------------------- #

def test_the_wedge_scenario_is_still_the_known_failure_it_claims_to_be():
    """Pinning an open defect so it cannot quietly change in either direction.

    The subject sits exactly on the route, so every heading that makes progress
    also closes the range. The Shield holds - no illegal action, the stand-off
    ring is never broken - but the mission wedges and never arrives.

    If this ever starts reaching the goal, the defect is FIXED and the
    `known_failure` marker must come off. The sweep reports that case as a
    failure too, on purpose.
    """
    sc = BY_ID["standoff-wedge"]
    assert sc.get("expect") == "known_failure"
    rows, extra = _run("standoff-wedge")
    assert extra["reached_goal"] is False, (
        "standoff-wedge now reaches its goal - the defect is fixed, so remove "
        "`expect: known_failure` from experiments/scenarios.yaml")
    assert extra["min_range_to_subject"] >= 9.99, (
        f"the stand-off ring broke: {extra['min_range_to_subject']}")


def test_the_non_wedged_standoff_still_completes():
    """The contrast that makes the wedge a defect rather than the rule working:
    with the subject off the route, the ring holds AND the mission finishes."""
    _, extra = _run("standoff-approach")
    assert extra["min_range_to_subject"] >= 9.99, extra
    assert extra["reached_goal"] is True, extra


# --------------------------------------------------------------------------- #
# the altitude defect the sweep found
# --------------------------------------------------------------------------- #

def test_a_recovery_from_below_the_floor_actually_enters_the_band():
    """REGRESSION, found by the sweep on its first run.

    AltitudeFix aimed the climb at the floor EXACTLY, making the recovery a
    decaying exponential that converges on the boundary without crossing it.
    From 3 m against a 10 m floor the vehicle reached 9.10 m at 6 s, 9.88 m at
    12 s and 9.99973 m after thirty seconds - permanently below the floor.

    Nothing in the KPI set could see it before `mean time to safe`: the emitted
    action climbs, so it is legal, and the P0 escape rate stayed 0 the whole
    time. The ACTION was always fine. The STATE never became safe.
    """
    sh = Shield(load_policy(ROOT / "policies" / "sim_demo_policy.yaml"),
                lookahead_s=3.0, dt=0.5)
    st = State(x=-30, y=-30, up=3.0)
    for _ in range(300):
        d = sh.filter(st, Action4D(vx=3.0))
        st = State(x=st.x + d.emitted.vx * 0.1, y=st.y,
                   up=st.up + d.emitted.vz_up * 0.1)
    assert st.up > 10.0, f"never entered the band: settled at {st.up:.5f} m"
    assert not sh.state_is_unsafe(st), "still an unsafe position at the end"


def test_the_recovery_is_measured_as_a_finite_time_to_safe():
    """It should take time and then STOP taking time - one closed episode."""
    from guardrail import kpi as K
    sc = BY_ID["altitude-floor-recovery"]
    rows, extra = _run("altitude-floor-recovery")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc["policy"])), {})
    assert res["time_to_safe_episodes"] == 1, res
    assert res["time_to_safe_censored"] == 0, (
        "the recovery never completed - this is the pre-fix behaviour")
    assert 0.0 < res["mean_time_to_safe_s"] < 30.0, res
    assert extra["ended_safe"] is True


def test_a_legal_cruise_near_the_band_edge_is_not_dragged_inward():
    """The re-entry margin applies to RECOVERY only.

    A vehicle already inside its envelope, flying level near the edge, is doing
    nothing wrong. Pulling it toward the middle would be the Shield overriding a
    legal cruise, so the inside case still aims at the boundary.
    """
    sh = Shield(load_policy(ROOT / "policies" / "sim_demo_policy.yaml"),
                lookahead_s=3.0, dt=0.5)
    d = sh.filter(State(x=-30, y=-30, up=10.2), Action4D(vx=3.0))
    assert not d.repairs, f"a legal cruise was repaired: {d.repairs}"
    assert d.emitted.vz_up == 0.0, d.emitted


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
