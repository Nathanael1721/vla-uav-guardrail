"""The fail-safe rubric (docs/RUBRIC-failsafe.md) and the labels it gives.

Run either way:
    pytest tests/test_failsafe_rubric.py -v
    python tests/test_failsafe_rubric.py

WHY THIS FILE EXISTS

Fail-safe trigger correctness (WP3-15, >= 99 %) is "triggered when expected,
not when not expected": a number made as much of the labels as of the FSM.
Until 2026-11-02 the labels were written family by family with no stated
rule, 27 library episodes ended in Loiter before the operator timeout and
scored as correct non-triggers they never showed, and the same physics was
labelled two ways (failsafe-loiter-timeout "expected", dyn-nfz-spawn-ahead
"not expected"). These tests hold the rule in place:

  * every label names its clause, and a clause cannot contradict its label;
  * per-cell labels match exactly one cell each, and never a seed;
  * an episode that ends in Loiter with its timeout still running is
    CENSORED - not scored, and a failed label gate - never a correct
    non-trigger;
  * the tool that applies the rubric (tools/failsafe_rubric.py) gives each
    clause on a cell built for it, from the flight WITHOUT a fail-safe, so a
    label cannot follow from what the FSM did.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
sys.path.insert(0, str(ROOT / "tools"))

from guardrail.fsm import EscalationFSM, FSMConfig, RuleHit  # noqa: E402
from guardrail.scenario_spec import (RUBRIC_CLAUSES, RUBRIC_EXPECTED,  # noqa: E402
                                     Family, Library, ScenarioSpec)

LIB = Library(ROOT / "experiments" / "scenarios.yaml")
FAMS = LIB.by_id()


def _refused(fn, needle=""):
    try:
        fn()
    except ValueError as e:
        assert needle in str(e), (needle, str(e))
        return
    raise AssertionError(f"accepted; expected a refusal containing {needle!r}")


def _minimal(**over):
    raw = {"scenario_id": "rub-x", "template": "dynamic_nfz_movement",
           "description": "x",
           "mission": {"start_pose": {"x": 0.0, "y": 0.0, "up": 15.0},
                       "target": {"x": 40.0, "y": 0.0, "up": 15.0},
                       "pilot": {"type": "goto", "speed": "$s"}},
           "policy": {"path": "policies/sim_demo_policy.yaml"},
           "parameter_sweep": {"s": [2, 4]},
           "expected_kpis": {"P0_escape_rate": 0},
           "expected_failsafe": False, "failsafe_clause": "N1"}
    raw.update(over)
    return raw


def test_every_library_label_names_a_clause_that_agrees_with_it():
    """A label with no reason is the state before 2026-11-02. Every labelled
    cell names an E clause (expected) or an N clause (not expected)."""
    n = 0
    for f in LIB.families:
        for c in f.cells():
            s = c.spec
            if s.expected_failsafe is None:
                continue
            n += 1
            assert s.failsafe_clause in RUBRIC_CLAUSES, (c.episode_id, s.failsafe_clause)
            assert (s.failsafe_clause in RUBRIC_EXPECTED) == s.expected_failsafe, c.episode_id
    assert n >= 150, n


def test_a_clause_that_contradicts_its_label_is_refused():
    _refused(lambda: ScenarioSpec.model_validate(
        {k: v for k, v in _minimal(failsafe_clause="E5").items() if k != "parameter_sweep"}
        | {"mission": {**_minimal()["mission"], "pilot": {"type": "goto", "speed": 2}}}),
        "says a fail-safe is expected")
    _refused(lambda: Family(_minimal(failsafe_cells=[
        {"params": {"s": 2}, "expected_failsafe": True, "clause": "N1", "why": "x"}])),
        "says a fail-safe is not expected")


def test_a_cell_label_overrides_the_family_and_names_its_clause():
    fam = Family(_minimal(failsafe_cells=[
        {"params": {"s": 4}, "expected_failsafe": True, "clause": "E5",
         "expected_outcome": "RTL_triggered", "why": "x"}]))
    by = {c.params["s"]: c.spec for c in fam.cells()}
    assert (by[2].expected_failsafe, by[2].failsafe_clause) == (False, "N1")
    assert (by[4].expected_failsafe, by[4].failsafe_clause, by[4].expected_outcome) == (
        True, "E5", "RTL_triggered")


def test_cell_labels_name_real_cells_one_each_and_never_a_seed():
    _refused(lambda: Family(_minimal(failsafe_cells=[
        {"params": {"speed": 4}, "expected_failsafe": False, "clause": "N1", "why": "x"}])),
        "could never match")
    _refused(lambda: Family(_minimal(failsafe_cells=[
        {"params": {"s": 4, "random_seed": 1001}, "expected_failsafe": False,
         "clause": "N1", "why": "x"}])), "not to a seed")
    _refused(lambda: Family(_minimal(failsafe_cells=[
        {"params": {"s": 4}, "expected_failsafe": True, "clause": "E5", "why": "x"},
        {"params": {"s": 4}, "expected_failsafe": True, "clause": "E3", "why": "y"}])),
        "match the cell")


def test_the_library_labels_one_physics_one_way():
    """REGRESSION (2026-11-02). failsafe-loiter-timeout (a `loiter` rule the
    pilot keeps violating) was labelled expected and dyn-nfz-spawn-ahead (a
    repair over theta the pilot keeps asking for) not expected: the same
    hold, the same timeout, two labels. Both are E5 now; the spawn-ahead cells
    whose forecast meets the zone within theta are N1."""
    lt = FAMS["failsafe-loiter-timeout"].cells()[0].spec
    assert (lt.expected_failsafe, lt.failsafe_clause) == (True, "E5")
    by = {(c.params["ahead_m"], c.params["vehicle_speed_mps"]): c.spec
          for c in FAMS["dyn-nfz-spawn-ahead"].cells()}
    assert (by[(3, 4)].expected_failsafe, by[(3, 4)].failsafe_clause) == (True, "E5")
    assert (by[(12, 2)].expected_failsafe, by[(12, 2)].failsafe_clause) == (False, "N1")
    assert all(not by[(30, v)].expected_failsafe for v in (2, 4, 8))


def test_an_episode_ending_in_loiter_before_the_timeout_is_censored_not_correct():
    """REGRESSION (2026-11-02). rail-mission-stub-vla and altitude-ceiling-hold
    ended at 30.0 s, IN Loiter, 0.1 s before the Loiter timeout (G5), and
    scored "correct, not triggered" - 27 library episodes in all. Now the
    FSM's summary says the timeout was pending and the sweep scores such an
    episode as censored: not in the correctness, a failed label gate."""
    import sweep_scenarios as S
    f = EscalationFSM()
    f.tick(0.0, "blocked", [RuleHit("nfz", "project_fix", "hard", "P0")])
    f.tick(0.1, "blocked", [RuleHit("nfz", "project_fix", "hard", "P0")])
    f.tick(29.9, "blocked", [RuleHit("nfz", "project_fix", "hard", "P0")])
    held = f.summary()["ended_holding"]
    assert held["timeout_pending"] and held["state"] == "Loiter", held
    f2 = EscalationFSM(FSMConfig(loiter_timeout_s=None))
    f2.tick(0.0, "blocked", [RuleHit("nfz", "project_fix", "hard", "P0")])
    f2.tick(0.1, "blocked", [RuleHit("nfz", "project_fix", "hard", "P0")])
    assert f2.summary()["ended_holding"]["timeout_pending"] is False
    spec = FAMS["rail-mission-stub-vla"].cells()[0].spec.model_copy(update={"ticks": 300})
    run = S.run_episode(spec, LIB.defaults, 0)
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["failsafe_censored"] is True and res["failsafe_matches_label"] is None, res
    keys = [k for k, _ in S.label_failures_keyed(spec, res)]
    assert keys == ["failsafe"], S.label_failures_keyed(spec, res)
    summ = S.failsafe_summary([{"id": "x", "kpi": res}])
    assert summ["labelled"] == 0 and summ["censored_in_loiter"] == 1, summ
    assert summ["fsm_scoring"]["scored"] == 0


def test_the_tool_gives_each_clause_from_the_flight_without_a_fail_safe():
    """tools/failsafe_rubric.py on cells built for each clause. E5 from the
    flight with no fail-safe: a spawn 3 m ahead keeps the pilot asking for a
    repair over theta past the timeout; one 30 m ahead does not. The rail
    mission's E5 spell is short there (the projection takes the stub round
    the zone) although on the FSM's own flight its Loiter kept the request
    deep for 30 s: the label cannot follow from what the FSM did."""
    import failsafe_rubric as R
    from guardrail.scenario_spec import expand
    timeout = FSMConfig().loiter_timeout_s
    want = {
        "dyn-nfz-spawn-ahead@ahead_m=3,variant=0,vehicle_speed_mps=4": (True, ["E5"]),
        "dyn-nfz-spawn-ahead@ahead_m=30,variant=0,vehicle_speed_mps=4": (False, []),
        "wind-beyond-authority@lane_y=15,wind_speed_mps=8": (True, ["E2"]),
        "rail-mission-stub-vla": (False, []),
        "failsafe-rtl-zone@ahead_m=6,variant=0": (True, ["E1"]),
        "gps-noise-beyond-margin@latency=5,pass_y=23.2,sigma=1": (True, ["E3"]),
    }
    eps, _ = expand(LIB, None, None, sorted({k.split("@")[0] for k in want}))
    got = {}
    for e in eps:
        if e.cell.episode_id in want and not e.comparison and e.prefix == "on" \
                and e.paraphrase_arm in (None, -1):
            j = R.judge(e, LIB, timeout, with_fsm=e.cell.episode_id == "rail-mission-stub-vla")
            got[e.cell.episode_id] = j
            assert (j["derived"], j["clauses_held"]) == want[e.cell.episode_id], (
                e.cell.episode_id, j)
            assert j["derived"] == e.cell.spec.expected_failsafe, j
    assert set(got) == set(want), sorted(set(want) - set(got))
    rail = got["rail-mission-stub-vla"]
    assert rail["e5_spell_s"] < rail["e5_need_s"] and rail["fsm_triggered"] is True, rail


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
