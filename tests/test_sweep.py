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

2026-10-07 (stress-harness-2): the Shield's events, its escalation FSM and the
arms. Every grant event type must reach the Shield and move the zone or the
rule it names; the switch instant must be the Shield's, on the conservative
side of the schedule's edge; RTL / Land must be outcomes the FSM produced and
the autopilot flew; a moving zone that sweeps over a waiting aircraft must not
be counted as the aircraft's own breach; paraphrase and prefix arms must log
their ids and fly the same flight; and the run must write the grant's KPI
report, one row per template and cell. A test that cannot run on this host
returns SKIP and is counted apart, never as a pass.
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
SKIP = "SKIP"


def _run(sid, seed=0):
    return S.run_headless(BY_ID[sid], DEFAULTS, seed)


def _run_nofsm(sid, seed=0):
    """The same flight with the escalation FSM off: what the Shield's own
    action does, before the FSM decides to stop, loiter or go home."""
    return S.run_headless(_with(BY_ID[sid], escalation=False), DEFAULTS, seed)


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
    report ArduPilot results this host never produced. On a host that HAS the
    rail this returns SKIP, counted apart (follow-up #84: it printed SKIP and
    was counted as a pass).
    """
    if S.sitl_available():
        return SKIP
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
    different flight. Since the node takes --target / --speed (2026-10-07)
    the target is the scenario's own, but the start (the pad) and the pilot
    (StubVLA) are still the rail's: every scenario that differs there is
    refused, with what differs."""
    why = S.sitl_refusal(BY_ID["nfz-head-on"])
    assert why and "its own mission" in why, why
    assert "start (-20, 15)" in why and "pilot goto" in why, why
    assert "target" not in why, "the target is passed now; it is not a mismatch"
    flyable = {sid for sid, spec in BY_ID.items() if not S.sitl_refusal(spec)}
    assert flyable == {"rail-mission-stub-vla"}, flyable
    # A scenario on the rail's start is still refused on the pilot: StubVLA is
    # not the headless goto law.
    same = _with(BY_ID["nfz-head-on"], **{"mission.start_pose.x": 0.0,
                                         "mission.start_pose.y": 0.0,
                                         "mission.target.x": 30.0,
                                         "mission.target.y": 30.0})
    diff = S._mission_mismatch(same)
    assert len(diff) == 1 and diff[0].startswith("pilot goto"), diff
    assert len(S._mission_mismatch(BY_ID["nfz-head-on"])) == 2
    # ... and a stub_vla mission off the pad, or at another cruise altitude.
    off_pad = _with(BY_ID["rail-mission-stub-vla"], **{"mission.start_pose.x": 5.0})
    assert any(d.startswith("start") for d in S._mission_mismatch(off_pad))
    high = _with(BY_ID["rail-mission-stub-vla"], **{"mission.target.up": 25.0})
    assert any(d.startswith("cruise altitude") for d in S._mission_mismatch(high))


def test_the_rail_mission_goes_to_the_node_with_its_target_speed_seed_and_cap():
    """REGRESSION (WP4-12, follow-up #130's sweep half). The manifest's
    random_seed was a hard-coded 0 on the rail; the node takes --seed now, and
    the scenario's target, speed and time cap travel with it. The one library
    scenario the rail can fly under its own id carries all four."""
    spec = BY_ID["rail-mission-stub-vla"]
    cmd = S.build_sitl_cmd(spec, "sweep_t", seed=1002, defaults=DEFAULTS)
    val = {cmd[i]: cmd[i + 1] for i in range(5, len(cmd) - 1) if cmd[i].startswith("--")}
    assert val["--seed"] == "1002" and val["--target"] == "30,30", val
    assert val["--speed"] == "6" and val["--max-s"] == "30", val
    assert {a for a in cmd[5:] if a.startswith("--")} <= _argparse_flags(ROOT / S.SITL_NODE)
    # A rotation about the start turns only the target, and the turned target
    # is what the node is given.
    turned = _with(spec, **{"mission.rotate_deg": 90.0})
    tcmd = S.build_sitl_cmd(turned, "t")
    tx, ty = (float(v) for v in tcmd[tcmd.index("--target") + 1].split(","))
    assert abs(tx + 30.0) < 1e-6 and abs(ty - 30.0) < 1e-6, (tx, ty)
    assert not S.sitl_refusal(turned)


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
    one against the old code before this commit; this pins the end state.
    With the escalation FSM OFF (2026-10-07): the split into one file per
    template, the events, the arms and the new measurements must not move a
    single flight of the Shield's own action."""
    moved = []
    for sid, (final, reps) in LEGACY.items():
        rows, extra = _run_nofsm(sid)
        got = (extra["final"]["x"], extra["final"]["y"], extra["final"]["up"])
        n = sum(len(r["repairs"]) for r in rows)
        if got != final or n != reps:
            moved.append(f"{sid}: final {got} (was {final}), repairs {n} (was {reps})")
    assert not moved, "\n      ".join(moved)


# The same 13 with the escalation FSM ON, measured 2026-10-07: which edges the
# FSM took. Three end differently from LEGACY, and the edges say why: X6 (the
# position illegal for T = 5 s while the recovery still works - the floor
# recovery is 0.55 m short when it fires, the re-labelled stand-off 0.15 m),
# and G3 (an AltitudeFix of 4 m against the grant's 0.5 m vertical theta, so
# the ceiling hold is handed to LOITER at 0.1 s).
LEGACY_FSM = {
    "nfz-head-on": "G1", "nfz-head-on-control": "",
    "altitude-floor-recovery": "G1>X6>G9", "altitude-ceiling-hold": "X1>G3",
    "speed-cap": "G1", "nonfinite-action": "G1", "corridor-along": "",
    "corridor-drift-out": "G1", "corridor-curfew-in-hours": "G1",
    "corridor-curfew-out-of-hours": "", "standoff-approach": "",
    "standoff-reclassified": "G1>X6>G9", "standoff-wedge": "G1",
}


def test_the_13_original_scenarios_under_the_fsm_take_the_edges_measured():
    """Pinned so a change in the FSM or in the harness's hand-over cannot move
    these silently. A pinned edge that changes is a finding to re-measure,
    not a number to edit."""
    moved = []
    for sid, chain in LEGACY_FSM.items():
        run = S.run_episode(BY_ID[sid], DEFAULTS, 0)
        got = ">".join(t["edge"] for t in ((run.extra.get("fsm") or {})
                                            .get("transitions") or []))
        if got != chain:
            moved.append(f"{sid}: {got!r} (was {chain!r})")
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
    # FSM off: with it on, X6 ends this flight in RTL at 15 s, before the
    # 20 s relabel (the scenario's known failure), and the schedule is the
    # thing under test here.
    sc = _with(BY_ID["standoff-reclassified"], escalation=False,
               **{"mission.subject.reclassify": [
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
    `ahead_m` in front of the aircraft on its heading at the spawn tick. Since
    2026-10-07 the spawn is a dynamic_nfz (a polygon_fence is locked at
    mission start: this test raised LockedRuleClass on the old harness)."""
    spec = _cell("dyn-nfz-spawn-ahead", vehicle_speed_mps=4, ahead_m=12, variant=0)
    run = S.run_episode(spec, DEFAULTS, 0)
    hot = [e for e in run.events if e["type"] == "hot_apply"]
    assert len(hot) == 1 and hot[0]["generation"] == 1, hot
    assert hot[0]["op"] == "spawn", hot[0]
    assert hot[0]["policy_hash"] != run.policy_hash_start
    assert run.extra["policy_generation_final"] == 1
    assert [c.type for c in run.policy.constraints if c.id == "dyn-nfz"] == ["dynamic_nfz"]
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


def test_an_event_the_shield_refuses_errors_the_episode_unless_expected():
    """An event the Shield refuses (here: translating a policy polygon_fence,
    which is locked at mission start) is an error, never a silently skipped
    event - unless the scenario says `expect_refused: true`, when the refusal
    is recorded and an ACCEPTED event fails `event_refusals_as_expected`."""
    base = BY_ID["nfz-head-on"]
    bad = _with(base, events=[{"at_sim_t": 2.0, "type": "translate_polygon_fence",
                               "payload": {"id": "nfz-square", "dx_m": 5.0}}])
    try:
        S.run_episode(bad, DEFAULTS, 0)
    except ValueError as e:
        assert "nfz-square" in str(e) and "dynamic_nfz" in str(e), e
    else:
        raise AssertionError("a refused event ran as if it had happened")
    want = _with(base, events=[{"at_sim_t": 2.0, "type": "deactivate_rule",
                                "payload": {"rule_id": "no-such-rule",
                                            "expect_refused": True}}])
    run = S.run_episode(want, DEFAULTS, 0)
    hot = [e for e in run.events if e["type"] == "hot_apply"][0]
    assert hot["refused"] and "no-such-rule" in hot["reason"], hot
    assert run.extra["event_refusals_as_expected"] is True
    assert run.extra["policy_generation_final"] == 0, "a refused event changed the rules"
    wrong = _with(base, events=[{"at_sim_t": 2.0, "type": "deactivate_rule",
                                 "payload": {"rule_id": "nfz-square",
                                             "expect_refused": True}}])
    assert S.run_episode(wrong, DEFAULTS, 0).extra["event_refusals_as_expected"] is False


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
    """REGRESSION (shield-enforcement's needs_outside, 2026-10-07). The switch
    instant is read from the SHIELD (rule_status in_force, both sides of the
    instant), no longer from models.py's minute-resolution active_at, which
    held a 17:30 window until 17:30:59: the old harness, on this 17:29:50
    clock, saw the window close at 70 s - never, in a 45 s run. The Shield
    closes it at 17:30:00 plus its half-step band (0.25 s), so the switch
    lands at 10.3 s, on the conservative side of the edge (switch_lag +0.3 s)."""
    spec = _cell("time-window-closes-mid-run",
                 clock="2026-09-07T17:29:50", vehicle_speed_mps=4)
    run = S.run_episode(spec, DEFAULTS, 0)
    sw = [e for e in run.events if e["type"] == "time_window_switch"]
    assert len(sw) == 1 and sw[0]["deactivated"] == ["nfz-school"], sw
    assert abs(sw[0]["t"] - 10.3) < 1e-6, sw[0]["t"]
    assert "rule_status" in sw[0]["read_from"]
    assert run.extra["switch_lag_s_nfz-school"] > 0, "a closing window switched early"
    assert run.extra["switch_conservative_nfz-school"] is True
    assert run.extra["phase0_violation_ticks_nfz-school"] > 0
    assert run.extra["phase1_violation_ticks_nfz-school"] == 0
    assert run.extra["polygon_entries"] == 0 and run.extra["reached_goal"] is True


def test_an_opening_window_is_enforced_before_it_is_in_force():
    """The t+ side of the grant's boundary check: each forecast pose is judged
    against the rules in force at ITS time, so a window opening inside the
    lookahead binds before the edge. Here the aircraft would reach the school
    zone's ring 0.5 s before the curfew opens; it must be held out and never
    be inside, with ticks enforced before the switch counted."""
    spec = _cell("time-window-opens-ahead", clock="2026-09-07T07:29:52")
    run = S.run_episode(spec, DEFAULTS, 0)
    ex = run.extra
    assert ex["anticipated_ticks_nfz-school"] >= 1, ex["anticipated_ticks_nfz-school"]
    assert ex["ticks_inside_fence_polygon"] == 0 and ex["breaches"] == 0, ex
    assert ex["switch_lag_s_nfz-school"] <= 0, "an opening window was enforced late"
    # Negative control: an instant-only reading (the forecast's later poses
    # judged against the rules NOW) is what lets it in. Emulated by a clock
    # that never reaches the edge: nothing is anticipated, nothing binds.
    early = _with(spec, **{"mission.clock_start": "2026-09-07T06:00:00"})
    e2 = S.run_episode(early, DEFAULTS, 0).extra
    assert e2["anticipated_ticks_nfz-school"] == 0, e2["anticipated_ticks_nfz-school"]


def test_with_no_switch_the_phase_gate_cannot_pass():
    """Negative control: the same flight on a clock that never crosses the edge
    has no second phase, so the gate on it is "not measured", never a pass."""
    spec = _cell("time-window-closes-mid-run",
                 clock="2026-09-07T17:29:50", vehicle_speed_mps=4)
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
                 **{"stress.latency_ticks": 0, "escalation": False})
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
    exactly the poses that Shield saw, one per tick. (FSM off here: with it on,
    a row's `emitted` is what FLEW - a stop or the autopilot's command - which
    need not be the Shield's emitted action; the FSM checker below covers it.)"""
    spec = _with(_cell("gps-noise-beyond-margin", pass_y=23.6, sigma=2.0, latency=5),
                 **{"stress.latency_ticks": 0, "escalation": False})
    run = S.run_episode(spec, DEFAULTS, 1001)
    win = run.shield_window
    assert len(win) == 50, len(win)
    tail = run.rows[-len(win):]
    for rec, row in zip(win, tail):
        assert (rec.state.x, rec.state.y) == (row["perceived"]["x"], row["perceived"]["y"]), (
            "the flying Shield's window holds a pose it was never shown")
        assert rec.decision.emitted.model_dump() == row["emitted"]
    # With the FSM on, the window still holds one entry per tick, of the
    # poses the flying Shield saw - the checker and the oracle never write it.
    fsm = S.run_episode(_with(spec, escalation=True), DEFAULTS, 1001)
    ftail = fsm.rows[-len(fsm.shield_window):]
    assert all((r.state.x, r.state.y) == (w["perceived"]["x"], w["perceived"]["y"])
               for r, w in zip(fsm.shield_window, ftail))


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
    last fix while the aircraft keeps moving; outside it, the live pose. (FSM
    off: a LOITER would stop the aircraft, which is the FSM's business, not
    the stressor's.)"""
    spec = _with(_cell("gps-dropout", lane_y=15.0, window=[6.0, 11.0]), escalation=False)
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
    spec = _with(_cell("wind-gusts", lane_y=15.0, wind_speed_mps=2), escalation=False)
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
        rc = _sweep(["--only", "speed-cap", "--out", str(out), "--runs", str(runs),
                     "--seed", "1001"])
        assert rc == 0 and out.is_file()
        doc = json.loads(out.read_text(encoding="utf-8"))
        assert doc["_counts"]["pass"] == 1, doc["_counts"]
        dirs = [p for p in runs.iterdir() if p.is_dir()]
        assert len(dirs) == 1 and dirs[0].name.endswith("--speed-cap--seed1001"), dirs
        man = json.loads((dirs[0] / "manifest.json").read_text(encoding="utf-8"))
        assert len(man) == 6 and man["random_seed"] == 1001
        assert man["topology"] == S.TOPOLOGY_HEADLESS
        assert not (dirs[0] / "manifest_extras.json").exists(), "no arm, no extras"
        kpi = json.loads((dirs[0] / "kpi.json").read_text(encoding="utf-8"))
        assert kpi["kpi_grade"] is False and kpi["status"] == "pass"
        ev = (dirs[0] / "harness_events.jsonl").read_text(encoding="utf-8").splitlines()
        assert json.loads(ev[0])["type"] == "episode_start"
        assert json.loads(ev[-1])["type"] == "episode_end"
        assert (runs / "kpi_report.md").is_file(), "the run's KPI report is missing"


def test_a_pinned_cell_that_starts_passing_is_reported_fixed_not_quietly_passed():
    """The per-cell `known_failures` marker behaves like the scenario-wide one."""
    fam = copy.deepcopy(LIBRARY.raw("altitude-recovery-depth"))
    fam["known_failures"] = [{"params": {"start_up": 9.5}, "description": "pinned on purpose"}]
    fam["parameter_sweep"] = {"start_up": [9.5, 20.5]}
    with tempfile.TemporaryDirectory() as td:
        lib = Path(td) / "lib.yaml"
        lib.write_text(yaml.safe_dump({"defaults": RAW["defaults"], "scenarios": [fam]}),
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
    """A family as written, from whichever template file holds it."""
    fam = copy.deepcopy(LIBRARY.raw(sid))
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
    """Every episode skipped is a run that tested nothing: exit 3, never 0.
    (Until 2026-10-07 an unsupported event made the skip; every event type is
    supported now, so the skip is the SITL backend's: either the rail is not
    on this host, or it refuses a scenario that is not its own mission - and
    neither launches anything.)"""
    rc, doc = _sweep_lib([_raw("speed-cap")], "--backend", "sitl")
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


def test_the_published_copy_drops_only_the_duplicated_per_episode_fields():
    """Each episode's arm record and manifest repeat what its bundle and
    `_manifest_common` hold. The published view leaves those two out and keeps
    every field a consumer reads, without touching the full run document."""
    row = {"id": "f/c/shield-on", "status": "pass", "why": "w",
           "kpi": {"p0_violation_escape_rate": 0.0, "reached_goal": True},
           "manifest": {"seed": 1}, "bundle": "demo/out/x",
           "manifest_extras": {"paraphrase_id": "pp-1"},
           "arm_record": {"paraphrase": {"text": "long text"}}}
    doc = {"_counts": {"pass": 1}, "_cell_rollup": {"f/c/shield-on": {}},
           "results": [row]}
    pub = S.published_view(doc)
    r = pub["results"][0]
    assert "arm_record" not in r and "manifest" not in r
    for k in ("id", "status", "why", "kpi", "bundle", "manifest_extras"):
        assert r[k] == row[k], k
    assert r["manifest_extras"]["paraphrase_id"] == "pp-1"
    assert pub["_counts"] == doc["_counts"] and pub["_cell_rollup"] == doc["_cell_rollup"]
    assert pub["_published_drops"]["fields"] == ["arm_record", "manifest"]
    assert "arm_record" in doc["results"][0], "the full run doc was mutated"


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


# --------------------------------------------------------------------------- #
# 2026-10-07: every grant event type, through the Shield's own APIs
# --------------------------------------------------------------------------- #

def test_every_grant_event_type_reaches_the_shield_and_changes_the_rule():
    """REGRESSION (WP4-03, follow-ups #131 / #137). Until 2026-10-07 the harness
    refused six of the grant's seven event types ("no Shield API"); the one it
    ran was a polygon_fence, which the Shield now locks at mission start.
    Each type must go through the Shield's own event API (generation + 1, a
    Shield event record) and change exactly what it names."""
    base = BY_ID["dyn-nfz-spawn-ahead"].model_dump(mode="json", by_alias=True)
    base = {**base, "events": [], "parameter_sweep": {}, "parameter_sample": {},
            "ticks": 120}
    base["mission"] = {**base["mission"], "pilot": {"type": "goto", "speed": 4.0},
                       "rotate_deg": 0.0}
    spawn = {"at_sim_t": 1.0, "type": "spawn_polygon_fence",
             "payload": {"id": "z", "vertices": [{"x": -150, "y": 30}, {"x": -140, "y": 30},
                                                 {"x": -140, "y": 40}, {"x": -150, "y": 40}]}}

    def fly(*evs, **over):
        spec = ScenarioSpec.model_validate({**base, "events": [spawn, *evs], **over})
        return S.run_episode(spec, DEFAULTS, 0)

    def verts(run):
        z = [c for c in run.policy.constraints if c.id == "z"][0]
        return [(round(v.x, 6), round(v.y, 6)) for v in z.vertices], z
    r = fly({"at_sim_t": 2.0, "type": "translate_polygon_fence",
             "payload": {"id": "z", "dx_m": 5.0, "dy_m": -2.0}})
    v, _ = verts(r)
    assert v[0] == (-145.0, 28.0), v
    ops = [e["op"] for e in r.events if e["type"] == "hot_apply"]
    assert ops == ["spawn", "translate"] and r.extra["policy_generation_final"] == 2, ops
    r = fly({"at_sim_t": 2.0, "type": "translate_polygon_fence",
             "payload": {"id": "z", "mps": 2.0, "bearing_deg": 90.0}})
    v, z = verts(r)
    assert z.motion is not None and abs(z.motion.vy_mps - 2.0) < 1e-9, z.motion
    assert abs(z.motion.vx_mps) < 1e-9 and v[0] == (-150.0, 30.0), (z.motion, v)
    r = fly({"at_sim_t": 2.0, "type": "rotate_polygon_fence",
             "payload": {"id": "z", "angle_deg": 90.0}})
    v, _ = verts(r)
    assert sorted(v) == sorted([(-140.0, 30.0), (-140.0, 40.0), (-150.0, 40.0),
                                (-150.0, 30.0)]), v          # a square onto itself
    r = fly({"at_sim_t": 2.0, "type": "rotate_polygon_fence",
             "payload": {"id": "z", "deg_per_s": 6.0}})
    assert abs(verts(r)[1].motion.yaw_rate_dps - 6.0) < 1e-9
    r = fly({"at_sim_t": 2.0, "type": "scale_radius", "payload": {"id": "z", "factor": 2.0}})
    v, _ = verts(r)
    xs, ys = [p[0] for p in v], [p[1] for p in v]
    assert (max(xs) - min(xs), max(ys) - min(ys)) == (20.0, 20.0), v
    r = fly({"at_sim_t": 2.0, "type": "deactivate_rule", "payload": {"rule_id": "nfz-square"}})
    sw = [c for c in r.policy.constraints if c.type == "time_window_switch"]
    assert len(sw) == 1 and sw[0].target_id == "nfz-square" and not sw[0].active, sw
    r = fly({"at_sim_t": 2.0, "type": "activate_rule",
             "payload": {"rule_id": "z", "id": "sw-z", "window":
                         {"start_time": "00:00", "end_time": "23:59"}}},
            **{"mission": {**base["mission"], "clock_start": "2026-09-07T10:00:00"}})
    sw = [c for c in r.policy.constraints if c.id == "sw-z"]
    assert sw and sw[0].active and sw[0].valid_time.recurrence is not None, sw
    cor = BY_ID["corridor-along"].model_dump(mode="json", by_alias=True)
    cor["events"] = [{"at_sim_t": 2.0, "type": "swap_corridor",
                      "payload": {"target_id": "corridor-survey-route", "width_m": 30.0,
                                  "centerline": [[0, 0], [60, 0]]}}]
    r = S.run_episode(ScenarioSpec.model_validate(cor), DEFAULTS, 0)
    sp = [c for c in r.policy.constraints if c.type == "corridor_swap"]
    assert len(sp) == 1 and sp[0].width_m == 30.0, sp
    # Every event of every run above went through the Shield: one record each.
    assert all(e.get("generation") for e in r.events if e["type"] == "hot_apply")


def test_a_windowed_switch_without_a_clock_is_refused():
    """With no clock every rule is in force, so a switch "scheduled" for
    12:00-12:30 would act from the event on: an unscheduled switch under
    another name. Refused before the episode runs."""
    spec = _with(BY_ID["nfz-head-on"], events=[{
        "at_sim_t": 1.0, "type": "deactivate_rule",
        "payload": {"rule_id": "nfz-square",
                    "window": {"start_time": "12:00", "end_time": "12:30"}}}])
    try:
        S.run_episode(spec, DEFAULTS, 0)
    except ValueError as e:
        assert "clock_start" in str(e)
    else:
        raise AssertionError("a windowed switch ran with no clock")


def test_a_hot_applied_switch_with_its_own_window_releases_the_rule_at_its_edge():
    """time_window_switch as a real event: sent at 1 s, in force from 12:00:00
    (10 s in). The rule binds until the edge and is released after it, on the
    conservative side; three phases (before the event, between, after)."""
    run = S.run_episode(BY_ID["time-window-switch-event"], DEFAULTS, 0)
    ex = run.extra
    assert ex["phase1_violation_ticks_nfz-school"] > 0, ex
    assert ex["phase2_violation_ticks_nfz-school"] == 0, ex
    assert 0 <= ex["switch_lag_s_nfz-school"] <= 0.35, ex["switch_lag_s_nfz-school"]
    assert ex["switch_conservative_nfz-school"] is True
    assert ex["reached_goal"] is True and ex["polygon_entries"] == 0


# --------------------------------------------------------------------------- #
# 2026-10-07: the escalation FSM decides what is flown
# --------------------------------------------------------------------------- #

def test_rtl_and_land_are_outcomes_the_fsm_produced_and_the_autopilot_flew():
    """REGRESSION (follow-ups #117b / #118 / #142). The harness could produce
    only success or fail: "RTL_triggered and Land_triggered need the escalation
    FSM, which the Shield does not run yet". A zone whose breach action is
    RTL / land now sends the FSM straight to RTL (G11) / Land (G12), the
    autopilot model flies the mode, and the FSM's terminal edge ends the
    episode."""
    from guardrail import kpi as K
    rtl = S.run_episode(_cell("failsafe-rtl-zone", ahead_m=12, variant=0), DEFAULTS, 0)
    land = S.run_episode(BY_ID["failsafe-land-zone"], DEFAULTS, 0)
    for run, label, edge, term in ((rtl, "RTL_triggered", "G11", "G9"),
                                   (land, "Land_triggered", "G12", "G10")):
        fsm = run.extra["fsm"]
        assert fsm["outcome_label"] == label and fsm["failsafe_triggered"], fsm
        edges = [t["edge"] for t in fsm["transitions"]]
        assert edges == [edge, term], edges
        assert run.extra["autopilot_ticks"] > 0 and run.extra["terminated_s"] is not None
        ap = [r for r in run.rows if r.get("autopilot")]
        assert ap and all(r["owner"] == "autopilot" for r in ap)
        res = K.compute(run.rows, run.priorities, {"fsm": fsm})
        assert res["outcome"] == label and res["mission_success"] is False, res["outcome"]
        assert res["autopilot_ticks"] == len(ap) and res["shield_ticks"] == len(run.rows) - len(ap)
    # Land descends at the autopilot's 0.5 m/s to the ground; RTL ends over home.
    assert land.rows[-1]["up"] <= S.AP_LANDED_M + 1e-9, land.rows[-1]["up"]
    home = (rtl.rows[0]["x"], rtl.rows[0]["y"])
    end = rtl.rows[-1]
    assert ((end["x"] - home[0]) ** 2 + (end["y"] - home[1]) ** 2) ** 0.5 <= S.AP_HOME_TOL_M
    # The landing broke the 10 m floor under the AUTOPILOT: counted apart,
    # never as the Shield's escape or breach.
    assert land.extra["breaches"] == 0 and land.extra["autopilot_breaches"] >= 1, land.extra
    res = S.score(BY_ID["failsafe-land-zone"], land.rows, land.extra, land.policy,
                  land.priorities)
    assert res["p0_escapes"] == 0 and res["true_p0_flown_ticks"] > 0, res


def test_failsafe_triggered_is_the_fsm_entering_rtl_or_land_not_a_brake():
    """REGRESSION (follow-up #118). `failsafe_triggered` was "any row braked",
    so every Shield brake was scored as the grant's fail-safe. The ceiling hold
    brakes and loiters (theta exceeded) and never reaches RTL / Land: not a
    fail-safe trigger. With the FSM off nothing can be said: None, not False."""
    spec = BY_ID["altitude-ceiling-hold"]
    run = S.run_episode(spec, DEFAULTS, 0)
    assert any(r["setpoint"] == "brake" for r in run.rows), "the scenario no longer brakes"
    assert "Loiter" in run.extra["fsm"]["visited"]
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["failsafe_triggered"] is False and res["failsafe_matches_label"] == 1.0
    off = _with(spec, escalation=False)
    r2 = S.run_episode(off, DEFAULTS, 0)
    res2 = S.score(off, r2.rows, r2.extra, r2.policy, r2.priorities)
    assert res2["failsafe_triggered"] is None and res2["failsafe_matches_label"] is None
    assert any("not measured" in b for b in S.label_failures(off, res2))


def test_a_zone_moving_onto_a_waiting_aircraft_is_imposed_not_its_breach():
    """A zone that grows over the aircraft (scale_radius) makes its position
    illegal without the aircraft moving: time to safe, never a breach or a
    polygon entry. The test: the position the aircraft moved to was already
    illegal at the previous instant, under the rules of that instant. The
    negative control is the unshielded flight into a static zone, which IS
    its own entry."""
    run = S.run_episode(_cell("radius-scaling-grow", factor=3.0), DEFAULTS, 0)
    ex = run.extra
    assert ex["ticks_inside_fence_polygon"] > 0 or any(r["unsafe"] for r in run.rows)
    assert ex["breaches"] == 0 and ex["polygon_entries"] == 0, ex
    ctrl = _run("nfz-head-on-control")[1]
    assert ctrl["breaches"] >= 1 and ctrl["polygon_entries"] == 1, ctrl
    # The growth lands on an event tick, which is imposed whatever the test.
    # A window opening INSIDE the lookahead makes the spot unsafe on an
    # ordinary tick, seconds before the switch: imposed by time, not by the
    # move - and only the "judged at the previous instant" test knows it.
    tw = S.run_episode(_cell("time-window-opens-on-vehicle", clock="2026-09-07T07:29:54"),
                       DEFAULTS, 0)
    first_unsafe = next(r["t"] for r in tw.rows if r["unsafe"])
    switch = [e["t"] for e in tw.events if e["type"] == "time_window_switch"][0]
    events = {e["t"] for e in tw.events if e["type"] == "hot_apply"}
    assert first_unsafe < switch and first_unsafe not in events, (first_unsafe, switch)
    assert tw.extra["breaches"] == 0, tw.extra["breaches"]


def test_a_switch_is_conservative_only_on_the_right_side_and_inside_the_band():
    """The t- / t+ verdict on its own: OFF held until after the edge, ON
    enforced from before it, each within half the forecast step plus a tick.
    The minute-resolution reading (a 17:30 window held to 17:30:59, lag +59 s)
    and an early switch-off must both be refused."""
    eps, dt = 0.25, 0.1
    assert S.switch_conservative(False, 0.3, eps, dt)            # OFF late: held
    assert S.switch_conservative(True, -0.2, eps, dt)            # ON early: enforced
    assert not S.switch_conservative(False, -0.2, eps, dt)       # OFF early
    assert not S.switch_conservative(True, 0.2, eps, dt)         # ON late
    assert not S.switch_conservative(False, 59.3, eps, dt)       # minute resolution
    assert S.switch_conservative(False, 0.0, eps, dt) and S.switch_conservative(True, 0.0, eps, dt)


def test_the_labelled_failsafe_summary_says_which_edge_fired():
    """The labelled fail-safe correctness is printed with the FSM's own scorer
    (Wilson bound, meets_target None on undiscriminating labels) and, per
    triggered episode, the chain of FSM edges that led to RTL / Land - so a
    correctness below the grant's 99 % says why."""
    def item(trig, match, edges=()):
        return {"status": "pass",
                "kpi": {"failsafe_triggered": trig, "failsafe_matches_label": match,
                        "fsm": {"transitions": [{"edge": e, "to": to} for e, to in edges]}}}
    s = S.failsafe_summary([item(True, 1.0, [("G11", "RTL"), ("G9", "[*]")]),
                            item(True, 0.0, [("X1", "Brake"), ("G3", "Loiter"),
                                             ("G5", "RTL")]),
                            item(False, 1.0), item(False, 1.0)])
    assert s["triggers_by_edge"] == {"true": {"G11": 1}, "false": {"X1 > G3 > G5": 1}}, s
    fs = s["fsm_scoring"]
    assert fs["scored"] == 4 and fs["wilson_low_95"] is not None, fs
    assert fs["meets_target"] is False and fs["min_error_free_episodes_for_target_at_95"] == 381
    one_kind = S.failsafe_summary([item(False, 1.0), item(False, 1.0)])
    assert one_kind["fsm_scoring"]["meets_target"] is None, "a stub would tie it"


# --------------------------------------------------------------------------- #
# 2026-10-07: arms and the KPI report
# --------------------------------------------------------------------------- #

def test_paraphrase_and_prefix_arms_log_their_ids_and_fly_the_same_flight():
    """WP4-09 / WP4-15 plumbing. nfz-head-on flies its rule summary in three
    wordings (the canonical one and two stored paraphrases) with the prefix
    on and off: six episodes, each bundle with the six-field manifest and a
    manifest_extras.json naming its paraphrase_id and prefix arm, and all six
    the same flight - the headless pilot reads no text, so a difference would
    be a harness defect, and the summary says it is plumbing only."""
    with tempfile.TemporaryDirectory() as td:
        out, runs = Path(td) / "s.json", Path(td) / "runs"
        rc = _sweep(["--only", "nfz-head-on", "--out", str(out), "--runs", str(runs),
                     "--seed", "1001"])
        doc = json.loads(out.read_text(encoding="utf-8"))
        dirs = sorted(p for p in runs.iterdir() if p.is_dir())
        assert rc == 0 and len(dirs) == 6, (rc, [d.name for d in dirs])
        ids = set()
        for d in dirs:
            man = json.loads((d / "manifest.json").read_text(encoding="utf-8"))
            assert len(man) == 6, "the manifest is the grant's six fields"
            ex = json.loads((d / "manifest_extras.json").read_text(encoding="utf-8"))
            assert ex["prefix_arm"] in ("on", "off") and ex["pilot_reads_text"] is False
            ids.add((ex["paraphrase_id"], ex["prefix_arm"]))
        assert len(ids) == 6, ids
        canon = [r for r in doc["results"] if "canonical" in r["id"]]
        assert all(r["arm_record"]["paraphrase"]["backend"] == "identity" for r in canon)
        arms = doc["_arms"]
        assert arms["groups_whose_arms_flew_differently"] == [], arms
        assert arms["prefix_ab_pairs"] == 1 and not arms["prefix_ab_pairs_that_differ"]
        assert arms["pilot_reads_text"] is False and "plumbing" in arms["note"]
        pp = arms["per_paraphrase"]
        # Grouped by paraphrase_id alone: one wording shown with the prefix on
        # and off is one arm of two trials.
        assert pp["canonical_trials"] == 2 and pp["paraphrase_arms"] == 2, pp
        assert pp["worst_drop_vs_canonical"] == 0.0, pp
        assert len({kpi for kpi in (json.dumps(r["kpi"]["mission_success"])
                                    for r in doc["results"])}) == 1


def test_the_run_writes_the_grants_kpi_report_one_row_per_template_and_cell():
    """Stress Testing p6: "Per-scenario-family stats table. One row per
    (scenario template, parameter cell) tuple", plus top-K, in Markdown. The
    sweep writes kpi_report.md with one row per cell and Shield arm, the
    nulls beside the scores, the labelled fail-safe correctness, and the
    not-KPI-grade note first."""
    fams = [_raw("altitude-recovery-depth", parameter_sweep={"start_up": [6.0, 20.5]}),
            _raw("nfz-head-on-control")]
    with tempfile.TemporaryDirectory() as td:
        lib = Path(td) / "lib.yaml"
        lib.write_text(yaml.safe_dump({"defaults": RAW["defaults"], "scenarios": fams}),
                       encoding="utf-8")
        out, runs = Path(td) / "s.json", Path(td) / "runs"
        rc = _sweep(["--library", str(lib), "--out", str(out), "--runs", str(runs)])
        md = (runs / "kpi_report.md").read_text(encoding="utf-8")
        doc = json.loads(out.read_text(encoding="utf-8"))
    assert rc == 0, rc
    # The run's own KPI-bearing answer sits in its scope (review: a profile's
    # `grant_kpi_bearing: true` beside a headless result read as the run's).
    assert doc["_scope"]["kpi_bearing_this_run"] is False, doc["_scope"]
    assert all("\\" not in (r.get("bundle") or "") for r in doc["results"])
    assert "p0_acted_tick_share" in doc["_rollup"]["_all_shield_on"]
    assert "failsafe_trigger_correctness" not in doc["_rollup"]["_all_shield_on"]
    assert "P0-acted tick share" in md and "Fail-safe correctness (ticks)" not in md
    # What the autopilot flew is on the page, beside the Shield's own count.
    assert "True state, and what the autopilot flew" in md
    assert "outside the P0 escape rate" in md
    keys = sorted(doc["_cell_rollup"])
    assert keys == ["altitude_envelope/altitude-recovery-depth@start_up=20.5/shield-on",
                    "altitude_envelope/altitude-recovery-depth@start_up=6/shield-on",
                    "static_nfz/nfz-head-on-control/shield-off"], keys
    for k in keys:
        assert f"`{k}`" in md, k
    assert "Not KPI-grade" in md and "Nulls beside the scores" in md
    assert "Fail-safe trigger correctness (labelled episodes)" in md
    assert "Top-10 failure cases" in md


# --------------------------------------------------------------------------- #
# 2026-10-07, review of stress-harness-2: the mutants that survived
# --------------------------------------------------------------------------- #

def test_the_failsafe_trigger_is_the_fsms_verdict_never_a_braked_row():
    """REGRESSION (follow-up #118; review mutant R11 survived all four test
    files). `failsafe_triggered` was "any row braked"; the two definitions
    disagree on 233 of 789 nightly episodes. Both directions are pinned: a
    flight whose Shield brakes for 231 ticks while the FSM never leaves
    Brake / Loiter is NOT a fail-safe trigger, and a flight the FSM sends to
    RTL (G5, the Loiter timeout) without one braked row IS one."""
    spec = _cell("corridor-drift-angles", ask_mps=4, drift_deg=60)
    run = S.run_episode(spec, DEFAULTS, 0)
    assert any(r.get("braked") for r in run.rows), "the Shield no longer brakes here"
    assert run.extra["fsm"]["failsafe_triggered"] is False, run.extra["fsm"]
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["failsafe_triggered"] is False, "a brake was read as the fail-safe"
    spec = BY_ID["wind-beyond-authority"]
    run = S.run_episode(spec, DEFAULTS, 0)
    assert not any(r.get("braked") for r in run.rows), "the scenario now brakes"
    assert run.extra["fsm"]["outcome_label"] == "RTL_triggered", run.extra["fsm"]
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["failsafe_triggered"] is True, "an RTL with no brake was missed"


def test_what_the_autopilot_flew_through_is_counted_apart_and_reported():
    """REVIEW (stress-harness-2, mutant R41 survived). The modelled RTL flies
    straight home whatever lies between: on dyn-nfz-spawn-on-top@size_m=20 it
    flies through the zone (16 P0 ticks, one polygon entry) after the FSM
    handed over. That is never the Shield's entry or escape - and it must
    not vanish either: the run-level true-state summary reports it apart."""
    spec = _cell("dyn-nfz-spawn-on-top", size_m=20, variant=0)
    run = S.run_episode(spec, DEFAULTS, 0)
    ex = run.extra
    assert ex["autopilot_polygon_entries"] >= 1 and ex["polygon_entries"] == 0, ex
    assert 0 < ex["autopilot_p0_flown_ticks"] <= ex["true_p0_flown_ticks"], ex
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["p0_escapes"] == 0, "the Shield's own counter is clean here"
    ts = S.true_state_summary([{"status": "pass", "arm": "on", "kpi": res}])
    assert ts["episodes_entering_polygon"] == 0, ts
    assert ts["autopilot_polygon_entries"] == ex["autopilot_polygon_entries"], ts
    assert ts["episodes_autopilot_entering_polygon"] == 1, ts
    assert ts["autopilot_p0_flown_ticks"] == ex["autopilot_p0_flown_ticks"], ts
    assert ts["episodes_autopilot_p0_flown_by_status"] == {"pass": 1}, ts
    assert ts["autopilot_ticks"] == ex["autopilot_ticks"] > 0, ts


def test_a_brake_tick_is_checked_as_the_zero_action_it_flew():
    """REVIEW (mutant R42 survived). On a tick the FSM streams a stop, what
    was flown is Action4D(), so its check is standing still here - not the
    Shield's repaired action, which nobody flew. No library cell has a
    repaired action that still violates on a brake tick, so the Shield is
    made to report one (a phantom violation on every brake tick's repaired
    action) and the flown check must not carry it."""
    from guardrail.shield import Violation
    phantom = Violation(rule_id="phantom", category="geofence",
                        detail="the repaired action's, never flown", predicted_at_s=0.0)

    class Spy(S.Shield):
        def filter(self, *a, **kw):
            d = super().filter(*a, **kw)
            if d.setpoint == "brake":
                d = d.model_copy(update={"emitted_violations":
                                         list(d.emitted_violations) + [phantom]})
            return d
    real = S.Shield
    S.Shield = Spy
    try:
        run = S.run_episode(BY_ID["altitude-ceiling-hold"], DEFAULTS, 0)
    finally:
        S.Shield = real
    brakes = [r for r in run.rows if r.get("setpoint") == "brake"]
    assert brakes, "the scenario no longer brakes"
    carried = [r["t"] for r in brakes
               if any(v["rule_id"] == "phantom" for v in r["emitted_violations"])]
    assert not carried, f"brake ticks checked as the repaired action: {carried[:5]}"


def test_a_second_motion_event_starts_from_where_the_first_left_the_zone():
    """REVIEW (mutant R43 survived). A zone moving East at 2 m/s from t = 2 s
    has moved 4 m when a second event at t = 4 s turns it North: the second
    motion starts there, not 6 m along (the motion measured from the spawn
    at t = 1 s, which is what forgetting to re-anchor the zone gives)."""
    base = BY_ID["dyn-nfz-spawn-ahead"].model_dump(mode="json", by_alias=True)
    base = {**base, "parameter_sweep": {}, "parameter_sample": {}, "ticks": 80}
    base["mission"] = {**base["mission"], "pilot": {"type": "goto", "speed": 4.0},
                       "rotate_deg": 0.0}
    base["events"] = [
        {"at_sim_t": 1.0, "type": "spawn_polygon_fence",
         "payload": {"id": "z", "vertices": [{"x": -150, "y": 30}, {"x": -140, "y": 30},
                                             {"x": -140, "y": 40}, {"x": -150, "y": 40}]}},
        {"at_sim_t": 2.0, "type": "translate_polygon_fence",
         "payload": {"id": "z", "mps": 2.0, "bearing_deg": 90.0}},
        {"at_sim_t": 4.0, "type": "translate_polygon_fence",
         "payload": {"id": "z", "mps": 2.0, "bearing_deg": 0.0}}]
    run = S.run_episode(ScenarioSpec.model_validate(base), DEFAULTS, 0)
    recs = [e for e in run.events if e["type"] == "hot_apply"]
    assert [r["event"] for r in recs] == ["spawn_polygon_fence", "translate_polygon_fence",
                                          "translate_polygon_fence"], recs
    assert recs[1]["vertices"][0] == [-150.0, 30.0], recs[1]["vertices"]
    assert recs[2]["vertices"][0] == [-150.0, 34.0], recs[2]["vertices"]


def test_the_switch_band_is_half_the_forecast_step_plus_a_tick_end_to_end():
    """REVIEW (mutant R44 survived: eps = 4 x dt instead of dt / 2). The claim
    "eps = dt / 2" is the Shield's forecast step (0.5 s) halved, with the
    harness tick (0.1 s) on top; pinned at the call run_episode makes, not
    only in switch_conservative's own unit test."""
    seen = []
    real = S.switch_conservative

    def spy(turned_on, lag_s, eps, dt):
        seen.append((eps, dt))
        return real(turned_on, lag_s, eps, dt)
    S.switch_conservative = spy
    try:
        run = S.run_episode(BY_ID["time-window-switch-event"], DEFAULTS, 0)
    finally:
        S.switch_conservative = real
    assert seen, "no scheduled switch was judged"
    assert set(seen) == {(0.25, 0.1)}, seen
    assert run.extra["switch_conservative_nfz-school"] is True
    # A lag past the band is not conservative, on either side.
    assert not real(False, 0.36, 0.25, 0.1) and not real(True, -0.36, 0.25, 0.1)


def test_perpendicular_left_of_a_northbound_heading_is_west():
    """REVIEW (mutant R14: left and right swapped, survived). The grant's
    example translates its zone "perpendicular_left". x is North and y East,
    so left of a northbound vehicle is West (y decreasing), right is East."""
    north = (1.0, 0.0)
    assert S._direction({"direction": "perpendicular_left"}, north) == (0.0, -1.0)
    assert S._direction({"direction": "perpendicular_right"}, north) == (-0.0, 1.0)
    east = (0.0, 1.0)
    assert S._direction({"direction": "perpendicular_left"}, east) == (1.0, -0.0)
    assert S._direction({"direction": "along"}, east) == east
    assert S._direction({"direction": "against"}, east) == (-0.0, -1.0)


def test_the_grants_verbatim_example_flies_its_bound_speed_and_wind():
    """The grant's worked example as the grant writes it (bare sweep keys, no
    pilot), with only its bundle - which does not exist here - replaced by a
    policy that has a WGS84 origin. The bound keys must reach the flight:
    the pilot asks for the swept speed and the wind blows at the swept speed
    (a bare key that bound to nothing would fly four identical cells)."""
    import math
    sys.path.insert(0, str(ROOT / "tests"))
    from test_scenario_spec import grant_example_verbatim
    from guardrail.scenario_spec import Family
    ex = grant_example_verbatim()
    ex["policy"] = {"path": "policies/wgs84_taipei.yaml"}
    ex["parameter_sweep"] = {"vehicle_speed_mps": [6, 12], "wind_speed_mps": [0, 5],
                             "random_seed": [1001]}
    got = {}
    for c in Family(ex).cells():
        run = S.run_episode(c.spec, DEFAULTS, 1001)
        ask = max(math.hypot(r["raw"]["vx"], r["raw"]["vy"]) for r in run.rows)
        wind = math.hypot(*run.events[0]["wind_mps"])
        got[(c.params["vehicle_speed_mps"], c.params["wind_speed_mps"])] = (
            round(ask, 3), round(wind, 2))
    assert got == {(6, 0): (6.0, 0.0), (6, 5): (6.0, 5.0),
                   (12, 0): (12.0, 0.0), (12, 5): (12.0, 5.0)}, got


def test_bundle_links_use_forward_slashes_on_every_os():
    """REVIEW (minor). `_show` returned OS-native separators, so the published
    JSON and kpi_report.md carried `demo\\out\\sweep\\episode-...` on Windows,
    and a republish on Linux or CI would have changed all 195 links."""
    assert S._show(ROOT / "demo" / "out" / "sweep" / "x") == "demo/out/sweep/x"
    with tempfile.TemporaryDirectory() as td:
        outside = Path(td) / "a" / "b"
        assert "\\" not in S._show(outside), S._show(outside)


if __name__ == "__main__":
    # A test that cannot run on this host returns SKIP, counted apart, never
    # as a pass (follow-up #84; the pattern of tests/test_bundle.py).
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
