"""The stress harness's scenario schema: guardrail/scenario_spec.py.

Run either way:
    pytest tests/test_scenario_spec.py -v
    python tests/test_scenario_spec.py

WHY THIS FILE EXISTS

The grant says scenarios are "declarative YAML, validated through Pydantic at
load time" (Stress Testing, "Scenario authoring"). A validator's failure mode
is accepting what it should refuse, so most tests below feed it something wrong
- a misspelt key, a parameter nothing reads, a padded sweep, a pin that can
never match, a policy whose hash moved - and require a refusal. The rest pin
the contract to the grant's own text: its field names and order, its seven
event types, and its worked example expanding to exactly 36 episodes.
"""
import copy
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import yaml                                                        # noqa: E402

from guardrail.scenario_spec import (GRANT_EVENT_TYPES,            # noqa: E402
                                     GRANT_TEMPLATES, Family, Library,
                                     PolicyRef, Profile, ScenarioEvent,
                                     ScenarioSpec, duplicate_cells, expand,
                                     load_profile, normalise_expectation)

LIB = ROOT / "experiments" / "scenarios.yaml"
PROFILES = ROOT / "experiments" / "profiles"


def _minimal(**over):
    spec = {
        "scenario_id": "t-min", "template": "static_nfz",
        "description": "a minimal valid scenario",
        "mission": {"start_pose": {"x": 0.0, "y": 0.0, "up": 15.0},
                    "target": {"x": 10.0, "y": 0.0, "up": 15.0},
                    "pilot": {"type": "goto", "speed": 4.0}},
        "policy": {"path": "policies/sim_demo_policy.yaml"},
        "expected_kpis": {"P0_escape_rate": 0},
    }
    spec.update(over)
    return spec


def _refused(fn, *needles):
    try:
        fn()
    except Exception as e:                                       # noqa: BLE001
        msg = str(e)
        for n in needles:
            assert n in msg, f"refused, but the message does not say {n!r}: {msg}"
        return msg
    raise AssertionError("accepted something it should have refused")


# --------------------------------------------------------------------------- #
# the contract is the grant's
# --------------------------------------------------------------------------- #

def test_the_grants_fields_come_first_and_in_the_grants_order():
    """Stress Testing p1-2, the Pydantic surface, copied field for field."""
    grant = ["scenario_id", "template", "description", "mission", "policy",
             "events", "parameter_sweep", "expected_kpis"]
    assert list(ScenarioSpec.model_fields)[:8] == grant, list(ScenarioSpec.model_fields)
    assert list(ScenarioEvent.model_fields) == ["at_sim_t", "type", "payload"]


def test_the_grants_four_templates_and_seven_event_types_are_all_accepted():
    assert GRANT_TEMPLATES == ("dynamic_nfz_movement", "time_window_switch",
                               "radius_scaling", "corridor_swap")
    assert set(GRANT_EVENT_TYPES) == {
        "spawn_polygon_fence", "translate_polygon_fence", "rotate_polygon_fence",
        "activate_rule", "deactivate_rule", "swap_corridor", "scale_radius"}
    for t in GRANT_TEMPLATES:
        ScenarioSpec.model_validate(_minimal(template=t))
    for e in GRANT_EVENT_TYPES:
        ScenarioEvent.model_validate({"at_sim_t": 1.0, "type": e, "payload": {}})
    _refused(lambda: ScenarioEvent.model_validate(
        {"at_sim_t": 1.0, "type": "teleport", "payload": {}}), "type")


def _grant_example():
    """The grant's worked example as this loader accepts it. It is NOT written
    exactly as the grant writes it; the differences, each needed:
      * "$vehicle_speed_mps" / "$wind_speed_mps" where the swept values go (the
        grant's bare sweep keys address fields its Mission never defines -
        see test_the_grants_literal_sweep_form_is_refused_and_says_why);
      * `mission.pilot` (a headless run has no VLA) and a `stress` block (the
        grant's model has no home for wind);
      * policies/wgs84_taipei.yaml, which has a WGS84 origin to project the
        lat/lon poses about, in place of the grant's `bundle_path`.
    Its poses, payload names and `random_seed` inside the sweep are the
    grant's."""
    return {
        "scenario_id": "dyn-nfz-translate-001",
        "template": "dynamic_nfz_movement",
        "description": "Polygon NFZ spawns 60 m ahead at t=15s, translates 5 m/s "
                       "perpendicular to flight path.",
        "mission": {"task_prompt": "Fly to waypoint W3 along corridor C1 at cruise altitude.",
                    "start_pose": {"lat": 25.0421, "lon": 121.5310,
                                   "alt_agl_m": 50, "yaw_deg": 90},
                    "target": {"lat": 25.0560, "lon": 121.5450, "alt_agl_m": 50},
                    "pilot": {"type": "goto", "speed": "$vehicle_speed_mps"}},
        "policy": {"path": "policies/wgs84_taipei.yaml"},
        "stress": {"wind_speed_mps": "$wind_speed_mps"},
        "events": [
            {"at_sim_t": 15.0, "type": "spawn_polygon_fence",
             "payload": {"width_m": 40, "height_m": 60, "ahead_of_vehicle_m": 60}},
            {"at_sim_t": 15.0, "type": "translate_polygon_fence",
             "payload": {"mps": 5.0, "direction": "perpendicular_left"}}],
        "parameter_sweep": {"vehicle_speed_mps": [6, 8, 10, 12],
                            "wind_speed_mps": [0, 2, 5],
                            "random_seed": [1001, 1002, 1003]},
        "expected_kpis": {"P0_escape_rate": 0, "fail_safe_correctness": ">=0.99",
                          "mean_time_to_safe_s": "<=2.0"},
    }


def test_the_grants_worked_example_expands_to_36_episodes():
    """"A single ScenarioSpec with the sweep above expands to 4 x 3 x 3 = 36
    episode runs." (Stress Testing p2), in the form _grant_example() states."""
    ex = _grant_example()
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "lib.yaml"
        p.write_text(yaml.safe_dump({"scenarios": [ex]}), encoding="utf-8")
        eps, info = expand(Library(p))
    assert info["cells"] == 12 and len(eps) == 36, info
    assert sorted({e.seed for e in eps}) == [1001, 1002, 1003]
    g = eps[0].cell.spec.gates
    assert g["p0_violation_escape_rate"] == {"equals": 0}
    assert g["failsafe_matches_label"] == {"min": 0.99}
    assert g["mean_time_to_safe_s"] == {"max": 2.0}
    assert not info["duplicate_cells"], info["duplicate_cells"]


def test_the_grants_literal_sweep_form_is_refused_and_says_why():
    """Written with the grant's bare sweep keys and nothing in the body reading
    them, the example would expand to 36 IDENTICAL episodes: neither
    `vehicle_speed_mps` nor `wind_speed_mps` is a field of anything the grant
    defines. The loader refuses it rather than count one flight 36 times."""
    ex = _grant_example()
    ex["mission"]["pilot"]["speed"] = 8.0
    del ex["stress"]
    msg = _refused(lambda: Family(ex), "nothing in the scenario reads them")
    assert "vehicle_speed_mps" in msg and "wind_speed_mps" in msg, msg


def test_the_grants_fail_safe_name_does_not_map_onto_the_escape_rate_restated():
    """guardrail.kpi's failsafe_trigger_correctness counts P0 ticks acted on -
    the escape rate in another form (audit card WP3-15). The grant's name must
    point at the label-based field instead."""
    s = ScenarioSpec.model_validate(_minimal(expected_kpis={"fail_safe_correctness": ">=0.99"}))
    assert "failsafe_trigger_correctness" not in s.gates
    assert "failsafe_matches_label" in s.gates


# --------------------------------------------------------------------------- #
# refusals
# --------------------------------------------------------------------------- #

def test_a_misspelt_key_stops_the_load_at_any_depth():
    """yaml.safe_load into a dict - what the sweep did until 2026-10-06 - takes
    `expected_kpi` as a field nobody reads, and the scenario asserts nothing."""
    _refused(lambda: ScenarioSpec.model_validate(
        {**_minimal(), "expected_kpi": {"x": 0}}), "expected_kpi")
    bad = _minimal()
    bad["mission"] = {**bad["mission"], "start_poze": {"x": 0, "y": 0, "up": 1}}
    _refused(lambda: ScenarioSpec.model_validate(bad), "start_poze")
    bad = _minimal(stress={"gps_noise": 1.0})
    _refused(lambda: ScenarioSpec.model_validate(bad), "gps_noise")


def test_a_gps_dropout_window_must_be_forward_in_time():
    """[start, end) with start < end: an empty or reversed window would declare
    GPS denial and deny nothing."""
    ok = ScenarioSpec.model_validate(_minimal(stress={"gps_dropout_s": [2.0, 5.0]}))
    assert ok.stress.degrades_perception and not ok.stress.consumes_seed
    for bad in ([5.0, 5.0], [5.0, 2.0], [-1.0, 2.0]):
        _refused(lambda b=bad: ScenarioSpec.model_validate(
            _minimal(stress={"gps_dropout_s": b})), "gps_dropout_s")


def test_a_swept_window_gets_a_shell_safe_episode_id():
    raw = _minimal(stress={"gps_dropout_s": "$w"},
                   parameter_sweep={"w": [[6.0, 11.0], [2.0, 4.5]]})
    ids = [c.episode_id for c in Family(raw).cells()]
    assert ids == ["t-min@w=6-11", "t-min@w=2-4.5"], ids


def test_a_scenario_with_no_gate_is_refused():
    _refused(lambda: ScenarioSpec.model_validate(_minimal(expected_kpis={})),
             "demonstration")


def test_a_goto_pilot_without_a_target_is_refused():
    bad = _minimal()
    bad["mission"] = {k: v for k, v in bad["mission"].items() if k != "target"}
    _refused(lambda: ScenarioSpec.model_validate(bad), "target")


def test_a_swept_parameter_nothing_reads_is_refused():
    """The padding failure: every cell would be the same episode."""
    raw = _minimal(parameter_sweep={"vehicle_speed_mps": [2, 4]})
    _refused(lambda: Family(raw), "nothing in the scenario reads")


def test_a_parameter_read_only_by_prose_gates_or_labels_is_refused():
    """REGRESSION (review 2026-10-06): a family sweeping a label and a task
    prompt gave 9 identical episodes and was accepted with 0 duplicates,
    because the prose differed. Nothing headless executes the description,
    the task prompt, a gate or a label."""
    raw = _minimal(description="$label", parameter_sweep={
        "label": ["a", "b", "c"], "prompt": ["x", "y", "z"]})
    raw["mission"]["task_prompt"] = "$prompt"
    msg = _refused(lambda: Family(raw), "nothing in the scenario reads")
    assert "label" in msg and "prompt" in msg and "description" in msg, msg
    gate = _minimal(parameter_sweep={"bound": [1, 2]},
                    expected_kpis={"repair_count": {"max": "$bound"}})
    _refused(lambda: Family(gate), "nothing in the scenario reads")
    # ... while a placeholder that IS flown is fine, prose beside it or not.
    ok = _minimal(description="$cls", parameter_sweep={"cls": ["pedestrian", "car"]})
    ok["mission"]["subject"] = {"x": 5.0, "y": 3.0, "class": "$cls"}
    assert [c.spec.description for c in Family(ok).cells()] == ["pedestrian", "car"]


def test_a_placeholder_with_no_value_is_refused():
    raw = _minimal()
    raw["mission"]["pilot"] = {"type": "goto", "speed": "$speed"}
    _refused(lambda: Family(raw), "$speed")


def test_expectations_read_the_grants_strings_and_refuse_strict_inequalities():
    assert normalise_expectation("a", ">=0.99") == {"min": 0.99}
    assert normalise_expectation("a", "<=2.0") == {"max": 2.0}
    assert normalise_expectation("a", 0) == {"equals": 0}
    assert normalise_expectation("a", True) == {"equals": True}
    assert normalise_expectation("a", "pedestrian") == {"equals": "pedestrian"}
    assert normalise_expectation("a", {"max": 1}) == {"max": 1}
    _refused(lambda: normalise_expectation("a", ">0.99"), "strict")
    _refused(lambda: normalise_expectation("a", {"maximum": 1}), "max/min/equals")


def test_a_pinned_policy_hash_that_moved_stops_the_load():
    ref = PolicyRef(path="policies/sim_demo_policy.yaml")
    real = ref.load(ROOT).policy_hash
    PolicyRef(path=ref.path, expected_hash=real).load(ROOT)          # matches: fine
    _refused(lambda: PolicyRef(path=ref.path, expected_hash="sha256:" + "0" * 64)
             .load(ROOT), "changed after the scenario was written")


def test_a_wgs84_pose_needs_a_policy_origin():
    from guardrail.scenario_spec import Pose
    pose = Pose(lat=25.0421, lon=121.5310, alt_agl_m=50)
    no_origin = PolicyRef(path="policies/sim_demo_policy.yaml").load(ROOT)
    _refused(lambda: pose.local(no_origin), "no origin")
    with_origin = PolicyRef(path="policies/wgs84_taipei.yaml").load(ROOT)
    x, y = pose.local(with_origin)
    assert abs(x) < 1e5 and abs(y) < 1e5, (x, y)


def test_a_known_failure_pin_that_can_never_match_is_refused():
    raw = _minimal(parameter_sweep={"s": [2, 4]},
                   known_failures=[{"params": {"speed": 2}, "description": "x"}])
    raw["mission"]["pilot"] = {"type": "goto", "speed": "$s"}
    _refused(lambda: Family(raw), "could never match")
    raw["known_failures"] = [{"params": {"s": 2}, "description": "pinned"}]
    cells = Family(raw).cells()
    pinned = [c.episode_id for c in cells if c.spec.known_failure_for(c.params)]
    assert pinned == ["t-min@s=2"], pinned


# --------------------------------------------------------------------------- #
# expansion, seeds, profiles
# --------------------------------------------------------------------------- #

def test_seed_precedence_is_flag_then_profile_then_spec_then_zero():
    raw = _minimal(parameter_sweep={"random_seed": [7, 8]})
    plain = _minimal(scenario_id="t-plain")
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "lib.yaml"
        p.write_text(yaml.safe_dump({"scenarios": [raw, plain]}), encoding="utf-8")
        lib = Library(p)
        prof = Profile(profile="p", cadence="c", grant_scope="g", grant_sim_speedup="s",
                       seeds=[1001, 1002], expected_cells=[1, 5])
        assert [e.seed for e in expand(lib)[0]] == [7, 8, 0]
        assert [e.seed for e in expand(lib, prof)[0]] == [1001, 1002, 1001, 1002]
        assert [e.seed for e in expand(lib, prof, seed=5)[0]] == [5, 5]


def test_profile_overrides_are_checked_against_the_family():
    lib = Library(LIB)
    base = dict(profile="p", cadence="c", grant_scope="g", grant_sim_speedup="s",
                seeds=[1], expected_cells=[1, 500])
    _refused(lambda: expand(lib, Profile(**base, sweep_overrides={
        "high-speed-near-edge": {"no_such_param": [1]}})), "does not sweep")
    _refused(lambda: expand(lib, Profile(**base, sweep_overrides={
        "no-such-family": {"x": [1]}})), "no-such-family")


def test_sampled_variants_are_the_same_geometry_in_every_run_and_profile():
    fam = Library(LIB).by_id()["three-simultaneous-violations"]
    a = [c.params for c in fam.cells(samples=3)]
    b = [c.params for c in fam.cells(samples=3)]
    one = [c.params for c in fam.cells(samples=1)]
    assert a == b
    v0 = [p for p in a if p["variant"] == 0]
    assert [p["approach_deg"] for p in v0] == [p["approach_deg"] for p in one]
    assert len({p["approach_deg"] for p in a}) == 3, "three variants, three angles"


def test_a_padded_sweep_is_caught():
    """Two identical values make two identical cells - counted once, flagged."""
    raw = _minimal(parameter_sweep={"s": [4, 4, 2]})
    raw["mission"]["pilot"] = {"type": "goto", "speed": "$s"}
    dups = duplicate_cells(Family(raw).cells())
    assert len(dups) == 1, dups


def test_cells_that_differ_only_in_prose_gates_labels_or_defaults_are_duplicates():
    """The same flight under another id, family name, description, gate, label
    or an explicit `ticks: 300` (the default) is still one flight."""
    a = Family(_minimal()).cells()[0]
    b = Family(_minimal(scenario_id="t-other", template="kinematic_envelope",
                        description="another story",
                        expected_kpis={"repair_count": {"max": 3}},
                        expected_outcome="success", expected_failsafe=False,
                        ticks=300)).cells()[0]
    assert duplicate_cells([a, b]) == [("t-min", "t-other")]
    c = Family(_minimal(scenario_id="t-flown", ticks=301)).cells()[0]
    assert duplicate_cells([a, c]) == [], "a different flight was called a copy"


def test_expand_reports_duplicates_across_families_and_unmatched_only_names():
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "lib.yaml"
        p.write_text(yaml.safe_dump({"scenarios": [
            _minimal(), _minimal(scenario_id="t-copy", description="same flight")]}),
            encoding="utf-8")
        lib = Library(p)
        _, info = expand(lib)
        assert info["duplicate_cells"] == [["t-min", "t-copy"]], info
        _, info = expand(lib, only=["t-min", "t-mni"])
        assert info["only_unmatched"] == ["t-mni"] and info["cells"] == 1, info


def test_only_matches_the_cells_this_profile_expands():
    """REGRESSION (review 2026-10-06): variant=2 exists only with three samples
    per family. Matched against the defaults it selected nothing (0 episodes,
    exit 0)."""
    lib = Library(LIB)
    nightly = load_profile(PROFILES / "nightly.yaml")
    cell = "three-simultaneous-violations@ask_mps=8,start_up=3,variant=2"
    eps, info = expand(lib, nightly, only=[cell])
    assert [e.episode_id for e in eps] == [cell] * 3 and not info["only_unmatched"], info
    eps, info = expand(lib, None, only=[cell])
    assert not eps and info["only_unmatched"] == [cell], (
        "with one sample there is no variant 2, and that must be SAID")


def test_the_repo_profiles_land_in_the_profiles_expected_ranges_without_padding():
    """The ranges are ours - [40, 60] and [170, 240] - written for the grant's
    "~50 scenarios x 1 seed" (smoke) and "~200 scenarios x 3 seeds" (nightly)."""
    lib = Library(LIB)
    for name, seeds in (("smoke", 1), ("nightly", 3)):
        prof = load_profile(PROFILES / f"{name}.yaml")
        eps, info = expand(lib, prof)
        assert info["cells_within_expected"], (name, info["cells"], info["expected_cells"])
        assert len(prof.seeds) == seeds
        assert info["episodes"] == info["cells"] * seeds
        # One entry per cell, NOT de-duplicated by id: identical parameters give
        # identical ids, and collapsing them first would hide exactly the
        # padding this checks for.
        cells = [e.cell for e in eps if e.seed == prof.seeds[0]]
        assert len(cells) == info["cells"]
        assert not duplicate_cells(cells, lib.defaults), (name, duplicate_cells(cells)[:3])
        assert not info["duplicate_cells"], (name, info["duplicate_cells"][:3])


def test_a_tracked_family_is_a_valid_expectation_and_not_a_known_failure():
    s = ScenarioSpec.model_validate(_minimal(expect="tracked"))
    assert s.expect == "tracked" and s.known_failure_for({}) is None
    _refused(lambda: ScenarioSpec.model_validate(_minimal(expect="tracked-ish")), "expect")
    tracked = {f.scenario_id for f in Library(LIB).families if f.first.spec.expect == "tracked"}
    assert tracked == {"gps-noise-beyond-margin", "gps-dropout", "wind-gusts"}, tracked


def test_the_smoke_profile_runs_every_stressor_of_the_shield_matrix():
    """Safety Shield, Test matrix: "Every release runs this matrix in Stress
    Testing's smoke set"."""
    eps, _ = expand(Library(LIB), load_profile(PROFILES / "smoke.yaml"))
    templates = {e.cell.spec.template for e in eps}
    matrix = {"high_speed_near_edge", "dynamic_nfz_movement", "time_window_switch",
              "three_simultaneous_violations", "sensor_degradation"}
    assert matrix <= templates, f"missing from smoke: {matrix - templates}"
    spawned = [e for e in eps if any(ev.type == "spawn_polygon_fence"
                                     for ev in e.cell.spec.events)]
    noisy = [e for e in eps if e.cell.spec.stress.degrades_perception]
    assert spawned and noisy


def test_every_library_scenario_has_a_reason_a_gate_and_a_real_policy():
    for fam in Library(LIB).families:
        s = fam.first.spec
        assert s.description.strip(), s.scenario_id
        assert s.expected_kpis, s.scenario_id
        assert (ROOT / s.policy.source).is_file(), s.scenario_id


def test_a_library_with_a_duplicate_id_is_refused():
    raw = _minimal()
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "lib.yaml"
        p.write_text(yaml.safe_dump({"scenarios": [raw, copy.deepcopy(raw)]}),
                     encoding="utf-8")
        _refused(lambda: Library(p), "duplicate")


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
