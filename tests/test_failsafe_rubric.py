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
    label cannot follow from what the FSM did. Since 2026-11-10 (docs/
    RUBRIC-failsafe.md section 7, adopted) E5 is read on the flight the
    grant prescribes instead - projection abandoned and the aircraft held on
    an unsafe request - which reads no FSM state either;
  * the label artefact (docs/data/failsafe_labels_rail.json, section 8,
    adopted 2026-11-10) holds four readings per cell and seed, marks what
    could not be decided instead of guessing it, names the code revision it
    was derived at, covers every cell the rails fly, and agrees with the
    templates on the reading they declare.
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
    """tools/failsafe_rubric.py on cells built for each clause. E1-E3 from the
    flight with no fail-safe. E5 (docs/RUBRIC-failsafe.md section 7, adopted
    2026-11-10) from the flight the grant prescribes - projection abandoned
    and the aircraft held on every tick whose request the Shield may not fly
    - with section 3's reading (the flight with no fail-safe) printed beside
    it. A spawn 3 m ahead keeps the
    pilot asking for a repair over theta past the timeout on both; one 30 m
    ahead on neither. The rail mission is where the two readings part: on
    the flight without a fail-safe the projection takes the stub round the
    zone (10.9 s of unsafe requests, N1); held, as the grant prescribes, its
    request stays over theta for the whole 49.9 s (E5). Neither reads what
    the FSM did."""
    import failsafe_rubric as R
    from guardrail.scenario_spec import expand
    timeout = FSMConfig().loiter_timeout_s
    want = {
        "dyn-nfz-spawn-ahead@ahead_m=3,variant=0,vehicle_speed_mps=4": (True, ["E5"]),
        "dyn-nfz-spawn-ahead@ahead_m=30,variant=0,vehicle_speed_mps=4": (False, []),
        "wind-beyond-authority@lane_y=15,wind_speed_mps=8": (True, ["E2"]),
        "rail-mission-stub-vla": (True, ["E5"]),
        "failsafe-rtl-zone@ahead_m=6,variant=0": (True, ["E1"]),
        "gps-noise-beyond-margin@latency=5,pass_y=23.2,sigma=1": (True, ["E3"]),
    }
    assert R.E5_FLIGHT == "prescribed"
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
            if e.cell.episode_id != "rail-mission-stub-vla":
                assert j["other"]["derived"] == j["derived"], j       # no flip
    assert set(got) == set(want), sorted(set(want) - set(got))
    rail = got["rail-mission-stub-vla"]
    assert rail["e5_flight"] == "prescribed" and rail["e5_spell_s"] >= rail["e5_need_s"], rail
    assert rail["other"] == {"e5_flight": "no-failsafe", "e5_spell_s": 10.9,
                             "e5_censored": False, "derived": False,
                             "clauses_held": []}, rail
    assert rail["fsm_triggered"] is True, rail


def test_the_prescribed_flight_holds_on_an_unsafe_request_and_flies_the_rest():
    """The flight E5 is read on (section 7, adopted 2026-11-10): on a
    tick whose request the Shield may not fly, and where a stop is legal, the
    aircraft is held (BRAKE on the ideal vehicle: it does not move); on every
    other tick the Shield's own action flies. The rail mission's request is
    over theta from the pad for the whole episode, so the held flight never
    leaves the pad, while the flight without a fail-safe flies the
    projection round the zone. A spawn 30 m ahead never asks for an unsafe
    repair: there both flights are the same flight."""
    import failsafe_rubric as R
    from guardrail.scenario_spec import expand
    eps, _ = expand(LIB, None, None, ["rail-mission-stub-vla", "dyn-nfz-spawn-ahead"])
    by = {e.cell.episode_id: e for e in eps if not e.comparison and e.prefix == "on"
          and e.paraphrase_arm in (None, -1)}
    rail = by["rail-mission-stub-vla"]
    held, hlog = R.prescribed(rail.cell.spec, LIB, rail.seed)
    free, flog = R.counterfactual(rail.cell.spec, LIB, rail.seed)
    assert hlog and all(R.unsafe_request(r) for r in hlog)
    x0, y0 = held.rows[0]["x"], held.rows[0]["y"]
    assert max(abs(r["x"] - x0) + abs(r["y"] - y0) for r in held.rows) < 1e-6
    assert max(abs(r["x"] - x0) + abs(r["y"] - y0) for r in free.rows) > 10.0
    far = by["dyn-nfz-spawn-ahead@ahead_m=30,variant=0,vehicle_speed_mps=4"]
    h2, l2 = R.prescribed(far.cell.spec, LIB, far.seed)
    f2, _ = R.counterfactual(far.cell.spec, LIB, far.seed)
    assert not any(R.unsafe_request(r) for r in l2)
    assert [(r["x"], r["y"]) for r in h2.rows] == [(r["x"], r["y"]) for r in f2.rows]


def test_labels_can_be_derived_on_the_rail_vehicle_and_flown_longer():
    """docs/RUBRIC-failsafe.md section 8 (drafted 2026-11-07 on the
    Copter-4.5.7 fit, `--response ardupilot`, which this test flies; adopted
    2026-11-10 on the rig's own fit, `--response rail`, the test below).
    The hil rail flies ArduCopter 4.5.7 in GUIDED. nfz-head-on on
    the ideal vehicle: the repair stays within theta and nothing persists
    (N1); the FSM brakes and is never censored. On the rail vehicle the
    aircraft carries on while it brakes, the head-on request's forecast ends
    deeper than theta (as the hil shakedown shk03_001 flew: G1 at 2.9 s, G3
    at 3.4 s, censored in Loiter at 30 s) and stays there on BOTH readings
    of E5: at the scenario's 30 s the spell is still running (undecided) and
    the FSM ends censored in Loiter; flown 35 s longer, E5 holds on both
    readings and the FSM's operator timeout (G5) returns the aircraft."""
    import failsafe_rubric as R
    from guardrail.scenario_spec import expand
    timeout = FSMConfig().loiter_timeout_s
    eps, _ = expand(LIB, None, None, ["nfz-head-on"])
    e = next(x for x in eps if not x.comparison and x.prefix == "on"
             and x.paraphrase_arm in (None, -1))
    ideal = R.judge(e, LIB, timeout, response="ideal")
    assert ideal["response"] == "ideal" and not ideal["derived"], ideal
    assert not ideal["other"]["derived"] and not ideal["e5_censored"], ideal
    assert ideal["fsm_triggered"] is False and ideal["fsm_censored"] is False, ideal
    rail = R.judge(e, LIB, timeout, response="ardupilot")
    assert rail["response"] == "ardupilot" and not rail["derived"], rail
    assert rail["e5_censored"] and rail["other"]["e5_censored"], rail
    assert rail["fsm_censored"] is True and rail["fsm_edges"].startswith("G1 > G3"), rail
    longer = R.judge(e, LIB, timeout, response="ardupilot", extend_s=35.0)
    assert longer["episode_s"] == 65.0 and longer["extend_s"] == 35.0, longer
    assert longer["clauses_held"] == ["E5"], longer
    assert longer["other"]["clauses_held"] == ["E5"], longer
    assert longer["fsm_triggered"] is True and longer["fsm_censored"] is False, longer
    assert "G5" in longer["fsm_edges"].split(" > "), longer


def _row(ep, seed, veh, *, pres, free, prof="smoke", rev="abc123", extend=0.0):
    """One --json row of tools/failsafe_rubric.py: `pres` / `free` are the
    (derived, e5_censored, clauses, spell) of the prescribed and the
    no-fail-safe reading."""
    def rec(flight, v):
        return {"e5_flight": flight, "derived": v[0], "e5_censored": v[1],
                "clauses_held": list(v[2]), "e5_spell_s": v[3]}
    return {**rec("prescribed", pres), "other": rec("no-failsafe", free), "episode": ep,
            "seed": seed, "response": veh, "extend_s": extend, "profile": prof,
            "code_revision": rev, "e5_need_s": 30.1}


def _labels_from(tmp, files):
    import json
    import failsafe_rubric as R
    paths = []
    for name, rows in files.items():
        q = tmp / name
        q.write_text(json.dumps(rows), encoding="utf-8")
        paths.append(q)
    return R.build_labels(paths)


def test_the_label_artefact_holds_four_readings_and_marks_the_undecided():
    """tools/failsafe_rubric.py --build-labels: per cell and seed the ideal
    vehicle's two readings (a: section 3, a2: prescribed) and the rail
    vehicle's (r, r2 - the adopted KPI reading), each with its clauses and
    its E5 spell; a cell no clause holds for while an E5 spell was still
    running at the scenario's end is UNDECIDED (expected null), never "not
    expected"; a clause that holds decides it whatever is still running."""
    import tempfile
    import failsafe_rubric as R
    no, e5 = (False, False, [], 0.0), (True, False, ["E5"], 49.9)
    running, e2 = (False, True, [], 26.6), (True, True, ["E2"], 8.4)
    ext = R.rail_extend_s()
    assert ext == 35.0, ext
    with tempfile.TemporaryDirectory() as td:
        doc = _labels_from(Path(td), {
            "smoke_ideal.json": [_row("cell-a", 1001, "ideal", pres=e5, free=no),
                                 _row("cell-b", 1001, "ideal", pres=no, free=no),
                                 _row("cell-c", 1001, "ideal", pres=e2, free=e2)],
            "smoke_rail.json": [_row("cell-a", 1001, "rail", pres=e5, free=e5),
                                _row("cell-b", 1001, "rail", pres=running, free=running),
                                _row("cell-c", 1001, "rail", pres=e2, free=e2)],
            "nightly_rail.json": [_row("cell-a", 1001, "rail", pres=e5, free=e5, prof="nightly"),
                                  _row("cell-a", 1002, "rail", pres=e5, free=e5, prof="nightly")],
            "nightly_ideal.json": [_row("cell-a", 1002, "ideal", pres=e5, free=no,
                                        prof="nightly", rev="def456")],
            # The rail flies 35 s longer: what was undecided is read there;
            # what was decided stays, and a difference is listed, not applied.
            "smoke_rail_x35.json": [
                _row("cell-b", 1001, "rail", pres=e5, free=running, extend=ext),
                _row("cell-a", 1001, "rail", pres=no, free=e5, extend=ext)]})
    assert doc["schema"] == R.LABELS_SCHEMA and doc["adopted_kpi_reading"] == "r2"
    assert "not a pre-registered reading" in doc["adopted"], doc["adopted"]
    assert doc["built"]["code_revisions"] == ["abc123", "def456"]
    assert doc["built"]["e5_need_s"] == 30.1 and doc["built"]["rail_extend_s"] == 35.0
    assert {(r["profile"], r["vehicle"], r["extend_s"]) for r in doc["built"]["runs"]} == {
        ("smoke", "ideal", 0.0), ("smoke", "rail", 0.0), ("nightly", "ideal", 0.0),
        ("nightly", "rail", 0.0), ("smoke", "rail", 35.0)}
    assert all("--response" in r["command"] for r in doc["built"]["runs"])
    a = doc["labels"]["cell-a"]["1001"]
    assert set(a) == set(R.READINGS) == {"a", "a2", "r", "r2"}
    assert [a[k]["expected"] for k in ("a", "a2", "r", "r2")] == [False, True, True, True]
    assert a["a2"]["clauses"] == ["E5"] and a["a2"]["e5_spell_s"] == 49.9
    assert a["r2"]["length"] == "scenario" and doc["built"]["differ_flown_longer"] == [{
        "cell": "cell-a", "seed": 1001, "reading": "r2", "scenario": True,
        "flown_longer": False, "clauses_flown_longer": [], "e5_spell_s_flown_longer": 0.0}]
    b = doc["labels"]["cell-b"]["1001"]
    assert b["a"]["expected"] is False and not b["a"]["undecided"]
    # r2 resolved by the longer flight; r still running there: still undecided.
    assert b["r2"]["expected"] is True and b["r2"]["length"] == "rail" and not b["r2"]["undecided"]
    assert b["r"]["expected"] is None and b["r"]["undecided"] and b["r"]["length"] == "scenario"
    assert doc["built"]["resolved_flown_longer"] == 1
    c = doc["labels"]["cell-c"]["1001"]["r2"]                # E2 holds: decided
    assert c["expected"] is True and not c["undecided"] and c["clauses"] == ["E2"]
    assert sorted(doc["labels"]["cell-a"]) == ["1001", "1002"]


def test_the_label_artefact_refuses_what_it_cannot_stand_behind():
    """A clean code revision per input, the scenario's own length (or, on
    the rail vehicle alone, the rail's own 35 s more), both vehicles for
    every cell and seed, and one answer per reading: anything else is
    refused, not merged."""
    import tempfile
    no, e5 = (False, False, [], 0.0), (True, False, ["E5"], 49.9)
    ideal = [_row("cell-a", 1001, "ideal", pres=no, free=no)]
    rail = [_row("cell-a", 1001, "rail", pres=e5, free=e5)]
    with tempfile.TemporaryDirectory() as td:
        t = Path(td)
        _refused(lambda: _labels_from(t, {"i.json": ideal, "r.json": [
            _row("cell-a", 1001, "rail", pres=e5, free=e5, rev="abc123-dirty")]}),
            "one clean code")
        _refused(lambda: _labels_from(t, {"i.json": ideal, "r.json": [
            _row("cell-a", 1001, "rail", pres=e5, free=e5),
            _row("cell-b", 1001, "rail", pres=e5, free=e5, rev="def456")]}), "one clean code")
        _refused(lambda: _labels_from(t, {"i.json": ideal, "r.json": rail, "x.json": [
            _row("cell-a", 1001, "rail", pres=e5, free=e5, extend=10.0)]}), "flown longer")
        _refused(lambda: _labels_from(t, {"r.json": rail, "i.json": [
            _row("cell-a", 1001, "ideal", pres=no, free=no, extend=35.0)]}), "flown longer")
        _refused(lambda: _labels_from(t, {"i.json": ideal, "r.json": rail, "x.json": [
            _row("cell-z", 1001, "rail", pres=e5, free=e5, extend=35.0)]}),
            "not at the scenario's length")
        _refused(lambda: _labels_from(t, {"r.json": rail}), "lack a reading")
        _refused(lambda: _labels_from(t, {"i.json": ideal, "r.json": [
            _row("cell-a", 1001, "ardupilot", pres=e5, free=e5)]}), "ideal or the rail")
        _refused(lambda: _labels_from(t, {"i.json": ideal, "r.json": rail, "r2.json": [
            _row("cell-a", 1001, "rail", pres=no, free=no, prof="nightly")]}), "in one input")
        _refused(lambda: _labels_from(t, {"i.json": ideal, "r.json": []}), "no cells")


def test_the_committed_artefact_covers_what_the_rails_fly_and_agrees_with_the_templates():
    """docs/data/failsafe_labels_rail.json as committed: every Shield-on,
    FSM-on cell and seed of the smoke, grant_example and nightly profiles
    (what the hil campaigns fly) has all four readings; `a2` - the ideal
    vehicle on the prescribed flight - is the label the templates declare,
    cell for cell; `a` differs from it only on the two cells section 7
    moves; no reading is expected with no clause or decided while marked
    undecided; and it names a clean code revision and the vehicles."""
    import json
    import failsafe_rubric as R
    import sweep_scenarios as SW
    from guardrail.scenario_spec import expand
    doc = json.loads(R.LABELS_PATH.read_text(encoding="utf-8"))
    assert doc["schema"] == R.LABELS_SCHEMA and doc["adopted_kpi_reading"] == "r2"
    revs = doc["built"]["code_revisions"]
    assert revs and all(len(r) == 40 for r in revs), revs
    assert doc["vehicles"]["rail"]["arm"] == "rail" and doc["vehicles"]["ideal"]["arm"] == "ideal"
    seen, moved = 0, set()
    for name in ("smoke", "grant_example", "nightly"):
        prof = SW._resolve_profile(name)
        eps, _ = expand(LIB, prof, None, None)
        for e in eps:
            if (e.comparison or e.prefix != "on" or e.paraphrase_arm not in (None, -1)
                    or not (e.cell.spec.shield and e.cell.spec.escalation)):
                continue
            lab = (doc["labels"].get(e.cell.episode_id) or {}).get(str(e.seed))
            assert lab is not None and set(lab) == set(R.READINGS), (name, e.cell.episode_id,
                                                                     e.seed)
            seen += 1
            declared = e.cell.spec.expected_failsafe
            assert lab["a2"]["expected"] == declared, (e.cell.episode_id, e.seed, declared,
                                                       lab["a2"])
            if lab["a"]["expected"] != lab["a2"]["expected"]:
                moved.add(e.cell.episode_id.split("@")[0])
            for k, v in lab.items():
                assert v["undecided"] == (v["expected"] is None), (e.cell.episode_id, k, v)
                assert v["length"] == "scenario" or k in ("r", "r2"), (e.cell.episode_id, k, v)
                assert bool(v["clauses"]) == (v["expected"] is True), (e.cell.episode_id, k, v)
    assert seen >= 700, seen
    assert moved == {"rail-mission-stub-vla", "standoff-approach-angles"}, moved


def test_an_episode_is_flown_longer_by_whole_ticks_and_never_shorter():
    """`--extend-s` (failsafe_rubric.longer): whole ticks, rounded up; 0
    leaves the spec as it is; a negative or non-finite extension is
    refused rather than read as none."""
    import math
    import failsafe_rubric as R
    spec = FAMS["nfz-head-on"].cells()[0].spec
    ticks = int(spec.ticks or LIB.defaults.ticks)
    assert R.longer(spec, LIB, 0.0) is spec
    assert R.longer(spec, LIB, 35.0).ticks == ticks + 350
    assert R.longer(spec, LIB, 0.05).ticks == ticks + 1
    for bad in (-1.0, math.nan, math.inf):
        try:
            R.longer(spec, LIB, bad)
        except ValueError:
            continue
        raise AssertionError(f"extend_s {bad} accepted")


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
