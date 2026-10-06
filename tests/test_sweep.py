"""The scenario sweep harness, the altitude defect it found, and the stress
harness it became.

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

The stress-harness groups (2026-10-06) hold the same line for the new parts: a
stressor that never reaches the Shield, a window switch that never happens, a
manifest with a seventh field, a SITL command line the node would reject, and a
pinned defect that quietly started passing must each be caught.

The review round of the same day added the ones a reviewer's mutations got
past: gusts replaced by zero, `reached_goal` not passed into compute(), the
oracle sharing the flying Shield, labels or a whole-scenario known_failure
ignored by main(), false and missed fail-safes swapped, wind not turned with
the mission, and every way a run could select nothing and still exit 0.
"""
import ast
import copy
import json
import os
import statistics
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import yaml                                                       # noqa: E402

import sweep_scenarios as S                                       # noqa: E402
from guardrail import load_policy                                 # noqa: E402
from guardrail.models import Action4D, State                      # noqa: E402
from guardrail.scenario_spec import Library, ScenarioSpec         # noqa: E402
from guardrail.shield import Shield                               # noqa: E402

LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")
DEFAULTS = LIBRARY.defaults
FAMILIES = LIBRARY.by_id()
BY_ID = {sid: fam.first.spec for sid, fam in FAMILIES.items()}


def _run(sid, seed=0):
    return S.run_headless(BY_ID[sid], DEFAULTS, seed)


def _cell(sid, **params):
    """The cell of a family whose parameters include `params`."""
    for c in FAMILIES[sid].cells(samples=3):
        if all(c.params.get(k) == v for k, v in params.items()):
            return c.spec
    raise KeyError(f"{sid}: no cell with {params}")


def _with(spec: ScenarioSpec, **changes) -> ScenarioSpec:
    """A copy of a spec with nested fields replaced ('mission.clock_start')."""
    d = spec.model_dump(mode="json", by_alias=True)
    for path, value in changes.items():
        node = d
        keys = path.split(".")
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    return ScenarioSpec.model_validate(d)


def _kpi(spec, rows, extra, policy=None):
    return S.score(spec, rows, extra, policy or spec.policy.load(ROOT))


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


def test_a_time_to_safe_bound_is_vacuous_only_when_measured_and_never_censored():
    """The grant example's `mean_time_to_safe_s: "<=2.0"` on a flight that was
    never unsafe has no mean to compare - and that is a pass only when the
    episode WAS measured. Never recovering is a fail; not measuring is a fail."""
    g = {"mean_time_to_safe_s": {"max": 2.0}}
    clean = {"mean_time_to_safe_s": None, "time_to_safe_not_measurable": False,
             "time_to_safe_episodes": 0, "time_to_safe_censored": 0}
    assert not S.check_gates(g, clean)
    assert "never recovered" in S.check_gates(g, {**clean, "time_to_safe_censored": 1})[0]
    assert "not measured" in S.check_gates(
        g, {**clean, "time_to_safe_not_measurable": True})[0]


def test_the_sitl_backend_reports_unavailable_rather_than_pretending():
    """Returning None means SKIP; the runner prints it and counts it apart.

    The alternative - a backend that silently falls back to headless - would
    report ArduPilot results this host never produced.
    """
    if S.sitl_available():
        print("      SKIP: this host does have the ArduPilot rail")
        return
    assert S.run_sitl(BY_ID["nfz-head-on"], "sweep_x") is None


# --------------------------------------------------------------------------- #
# the SITL command line
# --------------------------------------------------------------------------- #

def _argparse_flags(script: Path) -> set[str]:
    """Every option string the script's argparse declares, read from its source
    (the node imports rclpy, which this machine need not have)."""
    flags = set()
    for node in ast.walk(ast.parse(script.read_text(encoding="utf-8"))):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "add_argument"):
            for a in node.args:
                if isinstance(a, ast.Constant) and str(a.value).startswith("--"):
                    flags.add(a.value)
    return flags


def test_every_sitl_flag_is_one_the_node_accepts():
    """REGRESSION (audit card WP4-04). The backend passed `--out` to
    sitl/run_sitl_demo.py, which has no such argument, so `--backend sitl` could
    only ever die in argparse. It now drives run_ros2_demo.sh, which forwards
    $3.. to ros2_shield_node.py - checked here flag by flag."""
    node_flags = _argparse_flags(ROOT / S.SITL_NODE)
    assert "--out" not in node_flags, "the check would not catch the old bug"
    assert "--out" not in _argparse_flags(ROOT / "sitl" / "run_sitl_demo.py")
    script = (ROOT / S.SITL_SCRIPT).read_text(encoding="utf-8")
    assert '"${@:3}"' in script, "run_ros2_demo.sh no longer forwards $3.. to the node"
    for sid in ("nfz-head-on", "nfz-head-on-control", "standoff-approach"):
        cmd = S.build_sitl_cmd(BY_ID[sid], "sweep_t")
        assert cmd[:3] == ["wsl", "bash", S.SITL_SCRIPT], cmd
        assert cmd[3] in ("on", "off") and cmd[4] in ("", "--dynamic"), cmd
        passed = {a for a in cmd[5:] if a.startswith("--")}
        assert passed <= node_flags, f"{sid}: node rejects {passed - node_flags}"
    assert "--subject" in S.build_sitl_cmd(BY_ID["standoff-approach"], "t")


def test_the_sitl_rail_refuses_what_it_cannot_reproduce():
    """It flies its own mission with no clock, stressors or event schedule."""
    assert "scenario clock" in S.sitl_refusal(BY_ID["corridor-curfew-in-hours"])
    assert "events" in S.sitl_refusal(BY_ID["dyn-nfz-spawn-ahead"])
    assert "stressors" in S.sitl_refusal(BY_ID["gps-noise-within-margin"])
    assert "stressors" in S.sitl_refusal(BY_ID["gps-dropout"])
    assert "relabel" in S.sitl_refusal(BY_ID["standoff-reclassified"])


def test_the_sitl_rail_refuses_a_mission_that_is_not_its_own():
    """REGRESSION (review 2026-10-06). The first version refused only events,
    stressors, clocks and relabels, so nfz-head-on, speed-cap and the corridor
    scenarios would have been flown as the rail's fixed mission and scored
    against their own gates under their own ids - a pass describing a
    different flight. Every library scenario is now refused, with what
    differs."""
    why = S.sitl_refusal(BY_ID["nfz-head-on"])
    assert why and "fixed one" in why, why
    assert "start (-20, 15)" in why and "target (40, 15)" in why, why
    assert "pilot goto" in why, why
    for sid, spec in BY_ID.items():
        assert S.sitl_refusal(spec), f"{sid} would be flown as the rail's mission"
    # A scenario on the rail's start and target is still refused on the pilot:
    # StubVLA is not the headless goto law.
    same = _with(BY_ID["nfz-head-on"], **{"mission.start_pose.x": 0.0,
                                         "mission.start_pose.y": 0.0,
                                         "mission.target.x": 30.0,
                                         "mission.target.y": 30.0})
    diff = S._mission_mismatch(same)
    assert len(diff) == 1 and diff[0].startswith("pilot goto"), diff
    assert len(S._mission_mismatch(BY_ID["nfz-head-on"])) == 3


def test_the_rail_mission_constants_are_the_nodes_own():
    """SITL_RAIL_* must describe what the node flies, or the refusal above
    compares scenarios against a mission nobody flies."""
    from guardrail.compiler import PLACES
    node = (ROOT / S.SITL_NODE).read_text(encoding="utf-8")
    stub = (ROOT / "sitl" / "ros2_vla_stub_node.py").read_text(encoding="utf-8")
    assert f'parse_command("{S.SITL_RAIL_COMMAND}")' in node
    assert S.SITL_RAIL_COMMAND in stub and "StubVLA" in stub
    assert PLACES["northeast pad"] == S.SITL_RAIL_TARGET


def test_a_bundle_policy_goes_to_the_nodes_bundle_flag():
    """`--policy` takes a YAML; a signed bundle passed there would be refused
    by the node (or worse, read as YAML). bundle_path maps to --bundle."""
    spec = _with(BY_ID["nfz-head-on"],
                 policy={"bundle_path": "demo/out/x/policy_bundle.tar.gz"})
    cmd = S.build_sitl_cmd(spec, "t")
    assert cmd[cmd.index("--bundle") + 1] == "demo/out/x/policy_bundle.tar.gz"
    assert "--policy" not in cmd
    assert {a for a in cmd[5:] if a.startswith("--")} <= _argparse_flags(ROOT / S.SITL_NODE)


def test_a_stale_rail_log_is_refused_not_scored():
    """The tag is deterministic, so a launch that exits 0 without writing a
    log would otherwise score the PREVIOUS run's flight_log.jsonl."""
    with tempfile.TemporaryDirectory() as td:
        log = Path(td) / "flight_log.jsonl"
        try:
            S.read_fresh_log(log, time.time())
        except RuntimeError as e:
            assert "wrote no" in str(e)
        else:
            raise AssertionError("a missing log was accepted")
        log.write_text('{"t": 0}\n', encoding="utf-8")
        old = time.time() - 3600
        os.utime(log, (old, old))
        try:
            S.read_fresh_log(log, time.time())
        except RuntimeError as e:
            assert "predates this launch" in str(e)
        else:
            raise AssertionError("an hour-old log was scored as this launch's")
        launched = time.time()
        log.write_text('{"t": 0}\n{"t": 0.1}\n', encoding="utf-8")
        assert len(S.read_fresh_log(log, launched)) == 2


# --------------------------------------------------------------------------- #
# the control arm can still fail
# --------------------------------------------------------------------------- #

def test_the_unshielded_control_arm_scores_a_real_escape():
    """If an unguarded flight into a no-fly zone scores clean, every other pass
    in the sweep is worthless."""
    from guardrail import kpi as K
    sc = BY_ID["nfz-head-on-control"]
    rows, extra = _run("nfz-head-on-control")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc.policy.path)), {})
    assert res["p0_violation_escape_rate"] > 0.0, res
    assert res["time_to_safe_episodes"] >= 1, res
    assert res["p0_ticks_not_measurable"] == 0, "the control arm must be MEASURED"
    assert extra["breaches"] >= 1, "it flew itself into the zone"


def test_the_shielded_arm_of_the_same_flight_is_clean():
    from guardrail import kpi as K
    sc = BY_ID["nfz-head-on"]
    rows, extra = _run("nfz-head-on")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc.policy.path)), {})
    assert res["p0_violation_escape_rate"] == 0.0, res
    assert res["time_to_safe_episodes"] == 0, "the shielded arm was never unsafe"
    assert extra["breaches"] == 0


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
# the migration changed nothing
# --------------------------------------------------------------------------- #

# Produced by the PRE-schema sweep (git HEAD 1d09786, plain-dict loader) on
# 2026-10-06: where each of the 13 original scenarios ends and how many repairs
# it took. If this fails after a guardrail/shield.py change, the Shield moved,
# not the schema - read the diff before editing the numbers.
LEGACY = {
    "nfz-head-on": ((-5.6, 15.0, 15.0), 346),
    "nfz-head-on-control": ((37.2, 15.0, 15.0), 52),
    "altitude-floor-recovery": ((60.0, -30.0, 10.01), 73),
    "altitude-ceiling-hold": ((30.0, -30.0, 20.0), 300),
    "speed-cap": ((80.0, -40.0, 15.0), 300),
    "nonfinite-action": ((-40.0, 20.0, 15.0), 300),
    "corridor-along": ((52.4, 0.0, 15.0), 0),
    "corridor-drift-out": ((30.0, 8.4, 15.0), 279),
    "corridor-curfew-in-hours": ((60.0, 0.0, 15.0), 412),
    "corridor-curfew-out-of-hours": ((60.0, 52.4, 15.0), 0),
    "standoff-approach": ((37.2, 0.0, 15.0), 0),
    "standoff-reclassified": ((22.0, -4.03, 15.0), 360),
    "standoff-wedge": ((0.0, 0.0, 15.0), 300),
}


def test_the_13_original_scenarios_fly_exactly_as_before_the_schema():
    """Migration must be a change of FORM. Rows and extras were compared one to
    one against the old code before this commit; this pins the end state."""
    moved = []
    for sid, (final, reps) in LEGACY.items():
        rows, extra = _run(sid)
        got = (extra["final"]["x"], extra["final"]["y"], extra["final"]["up"])
        n = sum(len(r["repairs"]) for r in rows)
        if got != final or n != reps:
            moved.append(f"{sid}: final {got} (was {final}), repairs {n} (was {reps})")
    assert not moved, "\n      ".join(moved)


def test_seeds_change_only_episodes_that_draw_from_them():
    """A deterministic scenario on another seed is the same flight; a noisy one
    is not. The sweep reports which is which instead of calling repeats spread."""
    a, _ = _run("nfz-head-on", seed=1001)
    b, _ = _run("nfz-head-on", seed=1002)
    assert a == b
    spec = BY_ID["gps-noise-within-margin"]
    r1, _ = S.run_headless(spec, DEFAULTS, 1001)
    r1b, _ = S.run_headless(spec, DEFAULTS, 1001)
    r2, _ = S.run_headless(spec, DEFAULTS, 1002)
    assert r1 == r1b, "the same seed must reproduce the same flight"
    assert r1 != r2, "a different seed drew the same noise"
    assert spec.stress.consumes_seed and not BY_ID["nfz-head-on"].stress.consumes_seed


# --------------------------------------------------------------------------- #
# re-labelling the subject: the answer to the 2026-09-02 review
# --------------------------------------------------------------------------- #

def test_a_scenario_without_reclassify_behaves_exactly_as_before():
    """The schedule is optional and must change nothing when absent."""
    plain = BY_ID["standoff-approach"]
    assert not plain.mission.subject.reclassify
    rows, extra = S.run_headless(plain, DEFAULTS)
    ref_rows, ref_extra = _run("standoff-approach")
    assert extra["min_range_to_subject"] == ref_extra["min_range_to_subject"]
    assert extra["range_at_reclassify"] is None
    assert extra["subject_class_final"] == plain.mission.subject.class_


def test_the_enforced_ring_changes_with_the_label_and_nothing_else():
    """One object, one policy, one word changed - and the ring moves 5 m to 10 m."""
    rows, extra = _run("standoff-reclassified")
    assert extra["min_range_as_car"] < 10.0, (
        f"never got inside the 10 m ring while it was legal to: {extra}")
    assert extra["range_at_reclassify"] < 10.0, (
        "the label flipped while the aircraft was already clear, so the new "
        "rule never had to do anything - the scenario proves nothing")
    assert extra["final_range_to_subject"] >= 9.99, (
        f"did not recover to the pedestrian ring: {extra}")
    assert extra["subject_class_final"] == "pedestrian"


def test_the_recovery_is_the_shield_not_the_mission_wandering_off():
    from guardrail import kpi as K
    sc = BY_ID["standoff-reclassified"]
    rows, extra = _run("standoff-reclassified")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc.policy.path)), {})
    assert res["repair_count"] > 0, "the Shield did nothing"
    assert res["p0_violation_escape_rate"] == 0.0, (
        f"an illegal action flew during the recovery: {res}")


def test_removing_the_reclassification_makes_the_scenario_fail():
    """The negative control. A scenario that cannot fail is not a test."""
    sc = BY_ID["standoff-reclassified"]
    broken = _with(sc, **{"mission.subject.reclassify": []})
    rows, extra = S.run_headless(broken, DEFAULTS)
    res = _kpi(sc, rows, extra)
    bad = S.check_gates(sc.gates, res)
    assert len(bad) >= 3, f"the gates barely noticed: {bad}"


def test_a_relabel_schedule_is_applied_in_time_order_not_file_order():
    sc = _with(BY_ID["standoff-reclassified"], **{"mission.subject.reclassify": [
        {"at_s": 20.0, "class": "car"}, {"at_s": 10.0, "class": "pedestrian"}]})
    _, extra = S.run_headless(sc, DEFAULTS)
    assert extra["subject_class_final"] == "car", (
        "the later entry must win at the end regardless of file order")


# --------------------------------------------------------------------------- #
# the wedge, kept visible
# --------------------------------------------------------------------------- #

def test_the_wedge_scenario_is_still_the_known_failure_it_claims_to_be():
    """Pinning an open defect so it cannot quietly change in either direction."""
    sc = BY_ID["standoff-wedge"]
    assert sc.expect == "known_failure"
    rows, extra = _run("standoff-wedge")
    assert extra["reached_goal"] is False, (
        "standoff-wedge now reaches its goal - the defect is fixed, so remove "
        "`expect: known_failure` from experiments/scenarios.yaml")
    assert extra["min_range_to_subject"] >= 9.99, (
        f"the stand-off ring broke: {extra['min_range_to_subject']}")


def test_the_non_wedged_standoff_still_completes():
    _, extra = _run("standoff-approach")
    assert extra["min_range_to_subject"] >= 9.99, extra
    assert extra["reached_goal"] is True, extra


# --------------------------------------------------------------------------- #
# the altitude defect the sweep found
# --------------------------------------------------------------------------- #

def test_a_recovery_from_below_the_floor_actually_enters_the_band():
    """REGRESSION, found by the sweep on its first run (AltitudeFix aimed at the
    floor exactly and converged on it from below for ever)."""
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
    from guardrail import kpi as K
    sc = BY_ID["altitude-floor-recovery"]
    rows, extra = _run("altitude-floor-recovery")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc.policy.path)), {})
    assert res["time_to_safe_episodes"] == 1, res
    assert res["time_to_safe_censored"] == 0, (
        "the recovery never completed - this is the pre-fix behaviour")
    assert 0.0 < res["mean_time_to_safe_s"] < 30.0, res
    assert extra["ended_safe"] is True
    assert extra["breaches"] == 0, "starting unsafe is imposed, not a breach"


def test_a_legal_cruise_near_the_band_edge_is_not_dragged_inward():
    sh = Shield(load_policy(ROOT / "policies" / "sim_demo_policy.yaml"),
                lookahead_s=3.0, dt=0.5)
    d = sh.filter(State(x=-30, y=-30, up=10.2), Action4D(vx=3.0))
    assert not d.repairs, f"a legal cruise was repaired: {d.repairs}"
    assert d.emitted.vz_up == 0.0, d.emitted


# --------------------------------------------------------------------------- #
# stress: hot-applied no-fly zones
# --------------------------------------------------------------------------- #

def test_a_spawned_zone_bumps_the_generation_and_lands_where_it_says():
    """Shield.hot_apply: generation 0 -> 1, a new policy_hash, and the near edge
    `ahead_m` in front of the aircraft on its heading at the spawn tick."""
    spec = _cell("dyn-nfz-spawn-ahead", vehicle_speed_mps=4, ahead_m=12, variant=0)
    run = S.run_episode(spec, DEFAULTS, 0)
    hot = [e for e in run.events if e["type"] == "hot_apply"]
    assert len(hot) == 1 and hot[0]["generation"] == 1, hot
    assert hot[0]["policy_hash"] != run.policy_hash_start
    assert run.extra["policy_generation_final"] == 1
    from shapely.geometry import Point, Polygon
    at = [r for r in run.rows if abs(r["t"] - hot[0]["t"]) < 1e-9][0]
    d = Polygon(hot[0]["vertices"]).exterior.distance(Point(at["x"], at["y"]))
    assert abs(d - 12.0) < 0.6, f"near edge {d:.2f} m ahead, asked for 12"


def test_a_zone_spawned_on_top_puts_the_aircraft_inside_and_it_gets_out():
    spec = _cell("dyn-nfz-spawn-on-top", size_m=40, variant=0)
    run = S.run_episode(spec, DEFAULTS, 0)
    hot = [e for e in run.events if e["type"] == "hot_apply"][0]
    at = [r for r in run.rows if abs(r["t"] - hot["t"]) < 1e-9][0]
    assert at["unsafe"], "the spawn did not land on the aircraft"
    assert run.extra["breaches"] == 0, "an imposed entry was counted as a breach"
    res = _kpi(spec, run.rows, run.extra, run.policy)
    assert res["time_to_safe_episodes"] >= 1 and res["time_to_safe_censored"] == 0


def test_an_event_the_shield_has_no_api_for_is_refused_not_skipped():
    spec = _with(BY_ID["dyn-nfz-spawn-ahead"], events=[
        {"at_sim_t": 5.0, "type": "translate_polygon_fence",
         "payload": {"mps": 5.0, "direction": "perpendicular_left"}}])
    try:
        S.run_episode(spec, DEFAULTS, 0)
    except S.UnsupportedScenario as e:
        assert "translate_polygon_fence" in str(e)
        return
    raise AssertionError("an unsupported event ran as if it had happened")


def test_an_event_after_the_run_ends_is_refused():
    spec = _with(BY_ID["dyn-nfz-spawn-ahead"], **{"events": [
        {"at_sim_t": 999.0, "type": "spawn_polygon_fence",
         "payload": {"width_m": 10, "height_m": 10, "ahead_of_vehicle_m": 5}}]})
    try:
        S.run_episode(spec, DEFAULTS, 0)
    except ValueError as e:
        assert "never fires" in str(e)
        return
    raise AssertionError("an event that can never fire was accepted")


# --------------------------------------------------------------------------- #
# stress: the time-window switch instant
# --------------------------------------------------------------------------- #

def test_the_window_closing_mid_run_is_seen_and_splits_the_flight():
    spec = _cell("time-window-closes-mid-run",
                 clock="2026-09-07T17:30:50", vehicle_speed_mps=4)
    run = S.run_episode(spec, DEFAULTS, 0)
    sw = [e for e in run.events if e["type"] == "time_window_switch"]
    assert len(sw) == 1 and sw[0]["deactivated"] == ["nfz-school"], sw
    assert abs(sw[0]["t"] - 10.0) < 1e-6, sw[0]["t"]
    assert run.extra["phase0_violation_ticks_nfz-school"] > 0
    assert run.extra["phase1_violation_ticks_nfz-school"] == 0
    assert run.extra["reached_goal"] is True


def test_with_no_switch_the_phase_gate_cannot_pass():
    """Negative control: the same flight on a clock that never crosses the edge
    has no second phase, so the gate on it is "not measured", never a pass."""
    spec = _cell("time-window-closes-mid-run",
                 clock="2026-09-07T17:30:50", vehicle_speed_mps=4)
    still = _with(spec, **{"mission.clock_start": "2026-09-07T09:00:00"})
    run = S.run_episode(still, DEFAULTS, 0)
    assert not [e for e in run.events if e["type"] == "time_window_switch"]
    bad = S.check_gates(spec.gates, _kpi(still, run.rows, run.extra, run.policy))
    assert any("phase1_violation_ticks_nfz-school: not measured" in b for b in bad), bad


# --------------------------------------------------------------------------- #
# stress: GPS noise, latency, wind
# --------------------------------------------------------------------------- #

def test_gps_noise_reaches_the_shield_and_not_the_vehicle():
    """Latency 0, so ONLY the noise can make the perceived pose differ (the
    first version used a latency-5 cell, which passed with the noise removed:
    the stale pose alone differs from the true one)."""
    spec = _with(_cell("gps-noise-beyond-margin", pass_y=23.6, sigma=2.0, latency=5),
                 **{"stress.latency_ticks": 0})
    run = S.run_episode(spec, DEFAULTS, 1001)
    diffs = [abs(r["perceived"]["x"] - r["x"]) + abs(r["perceived"]["y"] - r["y"])
             for r in run.rows]
    assert max(diffs) > 1.0, "the Shield saw the true position: no noise applied"
    assert sum(d > 0 for d in diffs) >= 0.95 * len(diffs), "noise on too few ticks"
    assert "true_violations" in run.rows[0]
    quiet = _with(spec, **{"stress.gps_noise_m": 0.0, "stress.latency_ticks": 1})
    q = S.run_episode(quiet, DEFAULTS, 1001).rows
    assert all(abs(r["perceived"]["y"] - q[max(0, i - 1)]["y"]) < 1e-12
               for i, r in enumerate(q)), "with no noise the pose must be exact"


def test_the_oracle_never_writes_into_the_flying_shields_window():
    """The ground-truth oracle is a SECOND Shield. Re-using the flying one
    would leave the emitted actions unchanged (history is a record, not an
    input) but would fill the window a rail's audit writer reads with
    true-state queries the flying Shield never made. The window must hold
    exactly the poses that Shield saw, one per tick."""
    spec = _with(_cell("gps-noise-beyond-margin", pass_y=23.6, sigma=2.0, latency=5),
                 **{"stress.latency_ticks": 0})
    run = S.run_episode(spec, DEFAULTS, 1001)
    win = run.shield_window
    assert len(win) == 50, len(win)
    tail = run.rows[-len(win):]
    for rec, row in zip(win, tail):
        assert (rec.state.x, rec.state.y) == (row["perceived"]["x"], row["perceived"]["y"]), (
            "the flying Shield's window holds a pose it was never shown")
        assert rec.decision.emitted.model_dump() == row["emitted"]


def test_latency_hands_the_shield_the_state_from_n_ticks_ago():
    spec = _with(_cell("gps-noise-within-margin", pass_y=23.6, sigma=0.2, latency=2),
                 **{"stress.gps_noise_m": 0.0, "stress.latency_ticks": 5})
    rows = S.run_episode(spec, DEFAULTS, 0).rows
    for i in (10, 50, 120):
        assert rows[i]["perceived"]["x"] == rows[i - 5]["x"], i


def test_true_state_violations_are_counted_apart_from_the_shields_own():
    """Under noise the Shield's log can be clean while the flown action was
    illegal where the aircraft really was. Both are reported; neither replaces
    the other."""
    spec = _cell("gps-noise-beyond-margin", pass_y=23.6, sigma=2.0, latency=10)
    run = S.run_episode(spec, DEFAULTS, 1001)
    res = _kpi(spec, run.rows, run.extra, run.policy)
    assert res["p0_escapes"] == 0, "the Shield's own re-check should be clean"
    assert res["true_p0_flown_ticks"] > 0, "noise had no true-state effect at all"


def test_gps_dropout_freezes_the_pose_the_shield_sees_and_not_the_vehicle():
    """GPS denial (audit card X-14): for the window the Shield is handed the
    last fix while the aircraft keeps moving; outside it, the live pose."""
    spec = _cell("gps-dropout", lane_y=15.0, window=[6.0, 11.0])
    run = S.run_episode(spec, DEFAULTS, 0)
    inside = [r for r in run.rows if 6.0 - 1e-9 <= r["t"] < 11.0 - 1e-9]
    assert len(inside) == 50 == run.extra["gps_dropout_ticks"], run.extra["gps_dropout_ticks"]
    held = {(r["perceived"]["x"], r["perceived"]["y"]) for r in inside}
    assert len(held) == 1, "the pose the Shield saw moved during the dropout"
    assert inside[-1]["x"] - inside[0]["x"] > 5.0, "the vehicle stopped too: nothing was stale"
    after = [r for r in run.rows if r["t"] >= 11.0 - 1e-9]
    assert all(r["perceived"]["x"] == r["x"] for r in after), "the fix never came back"
    kinds = [e["type"] for e in run.events]
    assert kinds.count("gps_dropout_start") == 1 and kinds.count("gps_dropout_end") == 1


def test_gps_dropout_on_the_head_on_lane_is_a_measured_polygon_entry():
    """The finding the family is TRACKED for, pinned so it cannot change
    silently: the Shield trusts a frozen pose and lets the aircraft into the
    zone, while its own re-check stays clean (it repaired against the pose it
    was given). If this stops holding, the description in scenarios.yaml is
    wrong and must be re-measured."""
    spec = _cell("gps-dropout", lane_y=15.0, window=[6.0, 11.0])
    run = S.run_episode(spec, DEFAULTS, 0)
    res = _kpi(spec, run.rows, run.extra, run.policy)
    assert res["p0_escapes"] == 0, "the Shield's own re-check should be clean"
    assert run.extra["ticks_inside_fence_polygon"] > 0
    assert run.extra["min_true_dist_to_fence_m"] < -5.0, run.extra["min_true_dist_to_fence_m"]
    short = _cell("gps-dropout", lane_y=15.0, window=[2.0, 4.0])
    e2 = S.run_episode(short, DEFAULTS, 0).extra
    assert e2["ticks_inside_fence_polygon"] == 0 and e2["breaches"] == 0, e2


def test_a_dropout_that_froze_nothing_fails_its_gate_and_one_after_the_run_is_refused():
    """Silence reads as success: with no frozen tick, the family's
    `gps_dropout_ticks` gate must fail rather than score a clean run."""
    spec = _cell("gps-dropout", lane_y=15.0, window=[6.0, 11.0])
    assert S.check_gates(spec.gates, {"gps_dropout_ticks": 0,
                                      "p0_violation_escape_rate": 0,
                                      "all_actions_finite": True})
    plain = _kpi(BY_ID["nfz-head-on"], *_run("nfz-head-on"))
    assert plain["gps_dropout_ticks"] is None, "no dropout declared must read None"
    late = _with(spec, **{"stress.gps_dropout_s": [99.0, 120.0]})
    try:
        S.run_episode(late, DEFAULTS, 0)
    except ValueError as e:
        assert "never happens" in str(e)
    else:
        raise AssertionError("a dropout after the last tick was accepted")


def test_wind_moves_the_aircraft_and_the_shield_never_sees_it():
    still = BY_ID["speed-cap"]
    windy = _with(still, **{"stress.wind_speed_mps": 2.0, "stress.wind_dir_deg": 90.0})
    r0, e0 = S.run_headless(still, DEFAULTS)
    r1, e1 = S.run_headless(windy, DEFAULTS)
    assert [r["emitted"] for r in r0] == [r["emitted"] for r in r1], (
        "the Shield reacted to wind it cannot see")
    assert abs((e1["final"]["y"] - e0["final"]["y"]) - 2.0 * 30.0) < 0.5, (e0, e1)


def test_a_declared_wind_direction_turns_with_the_mission():
    """wind_dir_deg is in the MISSION's frame: a family flown from another side
    keeps "tailwind into the zone" a tailwind. Rotating the mission 90 deg must
    rotate the wind vector 90 deg; a drawn direction is absolute instead."""
    base = _with(BY_ID["nfz-head-on"], **{"stress.wind_speed_mps": 5.0,
                                          "stress.wind_dir_deg": 0.0})
    w0 = S.run_episode(base, DEFAULTS, 0).events[0]["wind_mps"]
    turned = _with(base, **{"mission.rotate_deg": 90.0})
    w90 = S.run_episode(turned, DEFAULTS, 0).events[0]["wind_mps"]
    assert abs(w0[0] - 5.0) < 1e-6 and abs(w0[1]) < 1e-6, w0
    assert abs(w90[0]) < 1e-6 and abs(w90[1] - 5.0) < 1e-6, (
        f"wind did not turn with the mission: {w90}")


def test_gusts_are_a_velocity_disturbance_the_shield_never_commands():
    """REGRESSION (review 2026-10-06: replacing the gust draw with 0 passed
    every test and the smoke run). The per-tick residual between where the
    aircraft went and what was commanded plus the steady wind IS the gust: a
    0.5 m/s sigma must show up there, and nowhere when gust_mps is 0."""
    spec = _cell("wind-gusts", lane_y=15.0, wind_speed_mps=2)
    assert spec.stress.gust_mps == 0.5

    def residuals(s):
        run = S.run_episode(s, DEFAULTS, 1001)
        w = run.events[0]["wind_mps"]
        dt = DEFAULTS.dt
        rs = run.rows
        return [((rs[i + 1]["x"] - rs[i]["x"]) / dt - rs[i]["emitted"]["vx"] - w[0],
                 (rs[i + 1]["y"] - rs[i]["y"]) / dt - rs[i]["emitted"]["vy"] - w[1])
                for i in range(len(rs) - 1)]

    gusty = residuals(spec)
    sx = statistics.pstdev([r[0] for r in gusty])
    sy = statistics.pstdev([r[1] for r in gusty])
    assert 0.4 < sx < 0.6 and 0.4 < sy < 0.6, (sx, sy)
    calm = residuals(_with(spec, **{"stress.gust_mps": 0.0}))
    assert max(abs(v) for r in calm for v in r) < 0.01, "a residual with no gust"


def test_start_jitter_draws_a_different_start_per_seed_within_its_bound():
    spec = _cell("wind-gusts", lane_y=15.0, wind_speed_mps=2)
    assert spec.stress.start_jitter_m == 1.0
    a = S.run_episode(spec, DEFAULTS, 1001).rows[0]
    b = S.run_episode(spec, DEFAULTS, 1002).rows[0]
    assert (a["x"], a["y"]) != (b["x"], b["y"]), "two seeds, one start"
    for r in (a, b):
        assert abs(r["x"] + 30.0) <= 1.0 and abs(r["y"] - 15.0) <= 1.0, r


def test_reached_goal_goes_into_compute_so_a_goal_less_success_fails():
    """REGRESSION (WP4-08; review 2026-10-06: dropping the pass-through passed
    every test). nfz-head-on is held at the zone edge and never arrives; with
    `reached_goal` in the metrics compute() scores that as a failed mission."""
    spec = BY_ID["nfz-head-on"]
    rows, extra = _run("nfz-head-on")
    assert extra["reached_goal"] is False
    res = _kpi(spec, rows, extra)
    assert res["outcome"] == "fail", res["outcome"]
    assert res["mission_success"] is False
    assert res["mission_goal_status"] == "missed"
    ok = _kpi(BY_ID["corridor-along"], *_run("corridor-along"))
    assert ok["outcome"] == "success" and ok["mission_goal_status"] == "reached"


# --------------------------------------------------------------------------- #
# labels, pins, bundles, manifests
# --------------------------------------------------------------------------- #

def test_a_fail_safe_label_that_disagrees_with_the_flight_fails_it():
    spec = _with(BY_ID["speed-cap"], expected_failsafe=True)
    rows, extra = S.run_headless(spec, DEFAULTS)
    res = _kpi(spec, rows, extra)
    assert res["failsafe_triggered"] is False
    assert res["failsafe_matches_label"] == 0.0
    assert any("fail-safe" in b for b in S.label_failures(spec, res))
    ok = _with(spec, expected_failsafe=False)
    assert not S.label_failures(ok, _kpi(ok, rows, extra))


def test_episode_manifest_is_exactly_the_six_fields_and_never_kpi_grade():
    from guardrail.manifest import is_kpi_grade
    base = {"code_revision": "abc123", "vla_model_hash": "m@src:1",
            "topology": S.TOPOLOGY_HEADLESS}
    m = S.episode_manifest(base, "sha256:x", 1001, sim_s=30.0, wall_s=0.1)
    assert tuple(m) == ("code_revision", "vla_model_hash", "policy_hash",
                        "random_seed", "sim_speedup", "topology"), tuple(m)
    assert m["random_seed"] == 1001 and m["sim_speedup"] == 300.0
    grade, why = is_kpi_grade(m, {})
    assert not grade
    assert any("sim_speedup" in w for w in why) and any("topology" in w for w in why)


def _sweep(argv):
    return S.main(argv)


def test_a_sweep_writes_one_bundle_per_episode_and_survives_an_out_outside_the_repo():
    """REGRESSION: `out.relative_to(ROOT)` raised AFTER the results were written
    whenever --out was outside the repository, so a good sweep exited with a
    traceback. The bundle is the grant's: manifest.json (six fields),
    kpi.json, harness_events.jsonl, in an `episode-<UTC>--<id>--seed<n>` folder."""
    with tempfile.TemporaryDirectory() as td:
        out, runs = Path(td) / "s.json", Path(td) / "runs"
        rc = _sweep(["--only", "nfz-head-on", "--out", str(out), "--runs", str(runs),
                     "--seed", "1001"])
        assert rc == 0 and out.is_file()
        doc = json.loads(out.read_text(encoding="utf-8"))
        assert doc["_counts"]["pass"] == 1, doc["_counts"]
        dirs = list(runs.iterdir())
        assert len(dirs) == 1 and dirs[0].name.endswith("--nfz-head-on--seed1001"), dirs
        man = json.loads((dirs[0] / "manifest.json").read_text(encoding="utf-8"))
        assert len(man) == 6 and man["random_seed"] == 1001
        assert man["topology"] == S.TOPOLOGY_HEADLESS
        kpi = json.loads((dirs[0] / "kpi.json").read_text(encoding="utf-8"))
        assert kpi["kpi_grade"] is False and kpi["status"] == "pass"
        ev = (dirs[0] / "harness_events.jsonl").read_text(encoding="utf-8").splitlines()
        assert json.loads(ev[0])["type"] == "episode_start"
        assert json.loads(ev[-1])["type"] == "episode_end"


def test_a_pinned_cell_that_starts_passing_is_reported_fixed_not_quietly_passed():
    """The per-cell `known_failures` marker behaves like the scenario-wide one."""
    raw = yaml.safe_load((ROOT / "experiments" / "scenarios.yaml").read_text(encoding="utf-8"))
    fam = copy.deepcopy([s for s in raw["scenarios"]
                         if s["scenario_id"] == "altitude-recovery-depth"][0])
    fam["known_failures"] = [{"params": {"start_up": 9.5}, "description": "pinned on purpose"}]
    fam["parameter_sweep"] = {"start_up": [9.5, 20.5]}
    with tempfile.TemporaryDirectory() as td:
        lib = Path(td) / "lib.yaml"
        lib.write_text(yaml.safe_dump({"defaults": raw["defaults"], "scenarios": [fam]}),
                       encoding="utf-8")
        out = Path(td) / "s.json"
        rc = _sweep(["--library", str(lib), "--out", str(out), "--no-bundles"])
        doc = json.loads(out.read_text(encoding="utf-8"))
    status = {r["id"]: r["status"] for r in doc["results"]}
    assert status["altitude-recovery-depth@start_up=9.5"] == "unexpected_pass", status
    assert status["altitude-recovery-depth@start_up=20.5"] == "pass", status
    assert rc == 1, "a fixed defect must turn the sweep red until the pin comes off"


# --------------------------------------------------------------------------- #
# main(): statuses, labels, refusals (review 2026-10-06)
# --------------------------------------------------------------------------- #

RAW = yaml.safe_load((ROOT / "experiments" / "scenarios.yaml").read_text(encoding="utf-8"))


def _raw(sid, **changes):
    fam = copy.deepcopy([s for s in RAW["scenarios"] if s["scenario_id"] == sid][0])
    fam.update(changes)
    return fam


def _sweep_lib(families, *extra):
    """Run main() on a temporary library; (rc, doc or None)."""
    with tempfile.TemporaryDirectory() as td:
        lib = Path(td) / "lib.yaml"
        lib.write_text(yaml.safe_dump({"defaults": RAW["defaults"], "scenarios": families}),
                       encoding="utf-8")
        out = Path(td) / "s.json"
        rc = _sweep(["--library", str(lib), "--out", str(out), "--no-bundles", *extra])
        doc = json.loads(out.read_text(encoding="utf-8")) if out.is_file() else None
    return rc, doc


def test_a_label_the_flight_contradicts_fails_the_scenario_in_main():
    """label_failures must be APPLIED by main(), not only computed: speed-cap
    labelled "fail-safe expected" never brakes, so it fails, and the sweep is
    red."""
    rc, doc = _sweep_lib([_raw("speed-cap", expected_failsafe=True)])
    r = doc["results"][0]
    assert r["status"] == "fail" and rc == 1, (r["status"], rc)
    assert any("fail-safe" in g for g in r["gates_failed"]), r["gates_failed"]
    rc, doc = _sweep_lib([_raw("speed-cap")])
    assert doc["results"][0]["status"] == "pass" and rc == 0


def test_a_whole_scenario_known_failure_is_reported_known_and_does_not_fail_the_run():
    rc, doc = _sweep_lib([_raw("standoff-wedge")])
    assert doc["results"][0]["status"] == "known_failure", doc["results"][0]["status"]
    assert doc["_counts"]["known_failure"] == 1 and rc == 0, (doc["_counts"], rc)
    unmarked = _raw("standoff-wedge")
    del unmarked["expect"]
    rc, doc = _sweep_lib([unmarked])
    assert doc["results"][0]["status"] == "fail" and rc == 1, "the marker did nothing"


def test_a_tracked_episode_is_never_counted_as_a_pass():
    """gps-dropout on the head-on lane passes its gates (the Shield's own log
    is clean) while the aircraft flies 7 m into the zone. It must be reported
    "tracked", counted apart, with the incursion in the true-state summary."""
    fam = _raw("gps-dropout", parameter_sweep={"lane_y": [15.0], "window": [[6.0, 11.0]]})
    rc, doc = _sweep_lib([fam])
    r = doc["results"][0]
    assert r["status"] == "tracked" and rc == 0, (r["status"], rc)
    assert doc["_counts"]["pass"] == 0 and doc["_counts"]["tracked"] == 1, doc["_counts"]
    ts = doc["_true_state"]
    assert ts["episodes_entering_polygon"] == 1, ts
    assert ts["episodes_entering_polygon_by_status"] == {"tracked": 1}, ts
    assert ts["min_true_dist_to_fence_m_of_entries"] < -5.0, ts
    assert doc["_rollup"]["_all_shield_on"]["true_state"] == ts
    del fam["expect"]
    rc, doc = _sweep_lib([fam])
    assert doc["results"][0]["status"] == "pass", "without the marker it hides as a pass"


def test_an_imposed_polygon_is_not_an_entry_and_a_flown_one_is():
    """A zone spawned on the aircraft puts it inside through no action of its
    own (time_to_safe's business); a frozen fix lets it fly in (an entry)."""
    on_top = S.run_episode(_cell("dyn-nfz-spawn-on-top", size_m=40, variant=0),
                           DEFAULTS, 0).extra
    assert on_top["ticks_inside_fence_polygon"] > 0 and on_top["polygon_entries"] == 0, on_top
    flown = S.run_episode(_cell("gps-dropout", lane_y=15.0, window=[6.0, 11.0]),
                          DEFAULTS, 0).extra
    assert flown["polygon_entries"] >= 1, flown
    ctrl = _run("nfz-head-on-control")[1]
    assert ctrl["polygon_entries"] == 1, ctrl
    assert _run("nfz-head-on")[1]["polygon_entries"] == 0


def _item(triggered, match, status="pass"):
    return {"status": status, "kpi": {"failsafe_triggered": triggered,
                                      "failsafe_matches_label": match}}


def test_failsafe_correctness_is_reported_with_both_nulls_and_the_grant_target():
    """Every score needs its null (CONTRIBUTING rule 4). Labels: 2 expected,
    4 not expected; the Shield got 3 right, 2 false triggers, 1 miss."""
    items = [_item(False, 1.0), _item(True, 0.0), _item(False, 0.0),
             _item(True, 1.0), _item(False, 1.0), _item(True, 0.0),
             _item(False, None)]
    s = S.failsafe_summary(items)
    assert (s["labelled"], s["unlabelled"]) == (6, 1), s
    assert (s["labelled_expected"], s["labelled_not_expected"]) == (2, 4), s
    assert (s["correct"], s["false_triggers"], s["missed_triggers"]) == (3, 2, 1), s
    assert s["fail_safe_correctness"] == 0.5
    assert s["null_never_trigger"] == round(4 / 6, 6)
    assert s["null_always_trigger"] == round(2 / 6, 6)
    assert s["beats_never_trigger"] is False and s["meets_grant_target"] is False
    assert s["grant_target"] == 0.99 and s["discriminating"] is True
    empty = S.failsafe_summary([_item(False, None)])
    assert empty["fail_safe_correctness"] is None and empty["null_never_trigger"] is None


def test_an_only_name_that_matches_nothing_refuses_the_run():
    """Silence reads as success: `--only nfz-head-onn` used to run 0 episodes
    and exit 0."""
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "s.json"
        rc = _sweep(["--only", "nfz-head-onn", "--out", str(out), "--no-bundles"])
        assert rc == 2 and not out.exists(), rc


def test_an_only_cell_id_is_matched_against_the_profiles_own_cells():
    """The pinned variant-2 cell exists only when the nightly profile samples
    three variants. The first version matched --only against the defaults
    (one variant), selected nothing, and exited 0."""
    cell = "three-simultaneous-violations@ask_mps=8,start_up=3,variant=2"
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "s.json"
        rc = _sweep(["--profile", "nightly", "--only", cell, "--out", str(out),
                     "--no-bundles"])
        doc = json.loads(out.read_text(encoding="utf-8"))
    assert rc == 0, rc
    assert [r["id"] for r in doc["results"]] == [cell] * 3, [r["id"] for r in doc["results"]]
    assert {r["status"] for r in doc["results"]} == {"known_failure"}


def test_two_cells_that_fly_the_same_episode_refuse_the_run():
    """Padding at run time, not only in the unit test of the committed
    profiles: a copy of a family under a new id and a new description is the
    same flight."""
    copy_ = _raw("speed-cap", scenario_id="speed-cap-again",
                 description="the same flight, described differently")
    rc, doc = _sweep_lib([_raw("speed-cap"), copy_])
    assert rc == 2 and doc is None, rc


def test_a_profile_outside_its_declared_count_refuses_unless_narrowed():
    with tempfile.TemporaryDirectory() as td:
        prof = Path(td) / "tiny.yaml"
        prof.write_text(yaml.safe_dump({
            "profile": "tiny", "cadence": "test", "grant_scope": "~2 scenarios",
            "grant_sim_speedup": "n/a", "seeds": [1001], "expected_cells": [1, 2]}),
            encoding="utf-8")
        out = Path(td) / "s.json"
        assert _sweep(["--profile", str(prof), "--out", str(out), "--no-bundles"]) == 2
        assert not out.exists()
        rc = _sweep(["--profile", str(prof), "--only", "speed-cap", "--out", str(out),
                     "--no-bundles"])
        assert rc == 0 and out.is_file(), rc


def test_a_run_that_scores_nothing_does_not_exit_zero():
    """Every episode skipped (an unsupported event here) is a run that tested
    nothing: exit 3, never 0."""
    fam = _raw("dyn-nfz-spawn-ahead")
    fam["events"] = [{"at_sim_t": 5.0, "type": "translate_polygon_fence",
                      "payload": {"mps": 5.0}}]
    fam["parameter_sweep"] = {"vehicle_speed_mps": [4], "ahead_m": [12]}
    fam["mission"]["pilot"]["speed"] = "$vehicle_speed_mps"
    fam["events"][0]["payload"]["ahead_m"] = "$ahead_m"
    rc, doc = _sweep_lib([fam])
    assert doc["_counts"]["skipped"] == 1 and rc == 3, (doc["_counts"], rc)


def test_a_plain_run_does_not_overwrite_the_published_sweep():
    """docs/data/scenario_sweep.json is read by the evaluation data, the deck
    and tests/test_kpi_magnitudes.py. Only --publish writes it, and --publish
    refuses a partial run."""
    assert S.results_path(None, False, None) == ROOT / "demo" / "out" / "sweep" / "sweep.json"
    assert S.results_path(None, False, "smoke").parent.name == "stress_smoke"
    assert S.results_path(None, True, None) == S.PUBLISHED_SWEEP
    assert S.PUBLISHED_SWEEP == ROOT / "docs" / "data" / "scenario_sweep.json"
    before = S.PUBLISHED_SWEEP.stat().st_mtime if S.PUBLISHED_SWEEP.exists() else None
    assert _sweep(["--publish", "--only", "speed-cap"]) == 2
    after = S.PUBLISHED_SWEEP.stat().st_mtime if S.PUBLISHED_SWEEP.exists() else None
    assert before == after, "a refused --publish touched the published file"


def test_the_sweep_reports_how_many_cells_behaved_differently():
    """A count of cells is not a count of behaviours; the distinct KPI
    signatures sit beside it. Two cells with identical KPI tables share one."""
    a = {"status": "pass", "kpi": {"repair_count": 3, "min_true_dist_to_fence_m": 1.0012}}
    b = {"status": "pass", "kpi": {"repair_count": 3, "min_true_dist_to_fence_m": 1.0049}}
    c = {"status": "pass", "kpi": {"repair_count": 4, "min_true_dist_to_fence_m": 1.0}}
    assert S.kpi_signature(a) == S.kpi_signature(b) != S.kpi_signature(c)
    rc, doc = _sweep_lib([_raw("speed-cap"), _raw("nfz-head-on")])
    assert doc["_distinct_kpi_signatures"]["distinct"] == 2
    assert all("distinct_kpi_signatures" in f for f in doc["_families"].values())


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
