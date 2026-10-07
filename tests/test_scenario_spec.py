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


# One valid payload per grant event type (the grant gives only `payload: dict
# # event-specific`; these are the keys EVENT_PAYLOADS accepts).
EVENT_EXAMPLES = {
    "spawn_polygon_fence": {"width_m": 40, "height_m": 60, "ahead_of_vehicle_m": 60},
    "translate_polygon_fence": {"mps": 5.0, "direction": "perpendicular_left"},
    "rotate_polygon_fence": {"angle_deg": 90.0},
    "activate_rule": {"rule_id": "nfz-square"},
    "deactivate_rule": {"rule_id": "nfz-square",
                        "window": {"start_time": "12:00", "end_time": "12:30"}},
    "swap_corridor": {"target_id": "c1", "centerline": [{"x": 0, "y": 0},
                                                        {"x": 10, "y": 0}],
                      "width_m": 10},
    "scale_radius": {"factor": 1.5},
}


def test_the_grants_four_templates_and_seven_event_types_are_all_accepted():
    assert GRANT_TEMPLATES == ("dynamic_nfz_movement", "time_window_switch",
                               "radius_scaling", "corridor_swap")
    assert set(GRANT_EVENT_TYPES) == {
        "spawn_polygon_fence", "translate_polygon_fence", "rotate_polygon_fence",
        "activate_rule", "deactivate_rule", "swap_corridor", "scale_radius"}
    assert set(EVENT_EXAMPLES) == set(GRANT_EVENT_TYPES)
    for t in GRANT_TEMPLATES:
        ScenarioSpec.model_validate(_minimal(template=t))
    for e, payload in EVENT_EXAMPLES.items():
        ScenarioEvent.model_validate({"at_sim_t": 1.0, "type": e, "payload": payload})
    _refused(lambda: ScenarioEvent.model_validate(
        {"at_sim_t": 1.0, "type": "teleport", "payload": {}}), "type")


def test_every_event_payload_is_checked_at_load():
    """REGRESSION (2026-10-07). An event payload was a bare dict: a misspelt
    key, a missing one, or two ways of saying one thing loaded, and failed
    (or silently did one of the two) only when the episode ran."""
    def ev(t, p):
        return lambda: ScenarioEvent.model_validate({"at_sim_t": 1.0, "type": t,
                                                     "payload": p})
    _refused(ev("scale_radius", {}), "needs", "factor")
    _refused(ev("scale_radius", {"factor": 0}), "factor")
    _refused(ev("scale_radius", {"factor": 2, "radius": 3}), "unknown", "radius")
    _refused(ev("translate_polygon_fence", {"dx_m": 3, "mps": 2,
                                            "direction": "along"}), "either a step")
    _refused(ev("translate_polygon_fence", {"mps": 2}), "direction")
    _refused(ev("translate_polygon_fence", {"mps": 2, "direction": "sideways"}),
             "direction is one of")
    _refused(ev("rotate_polygon_fence", {"angle_deg": 9, "deg_per_s": 3}), "exactly one")
    _refused(ev("spawn_polygon_fence", {"width_m": 10}), "height_m")
    _refused(ev("spawn_polygon_fence", {"width_m": 4, "height_m": 4,
                                        "motion": {"vx": 1}}), "vx")
    _refused(ev("activate_rule", {"rule_id": "x", "window": {"start_time": "25:99"}}),
             "out of range")
    _refused(ev("activate_rule", {"rule_id": "x", "expect_refused": "yes"}),
             "expect_refused")
    ScenarioEvent.model_validate({"at_sim_t": 1.0, "type": "translate_polygon_fence",
                                  "payload": {"dx_m": 3.0}})
    ScenarioEvent.model_validate({"at_sim_t": 1.0, "type": "spawn_polygon_fence",
                                  "payload": {"width_m": 4, "height_m": 4,
                                              "violation_action": "RTL",
                                              "motion": {"vx_mps": 1.0}}})


def _grant_example():
    """The grant's worked example in the PLACEHOLDER form, the one this
    repository's library uses: "$vehicle_speed_mps" / "$wind_speed_mps" written
    where the swept values go, an explicit `goto` pilot, and
    policies/wgs84_taipei.yaml (a WGS84 origin to project the lat/lon poses
    about) in place of the grant's bundle. The grant's VERBATIM form loads
    too: grant_example_verbatim() and the test beside it."""
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


def grant_example_verbatim():
    """The grant's worked example (Stress Testing p2, `dyn-nfz-translate-001`)
    exactly as the grant writes it: its keys, its values, its bare sweep keys,
    its bundle path and its truncated hash, no `pilot` and no `stress`."""
    return {
        "scenario_id": "dyn-nfz-translate-001",
        "template": "dynamic_nfz_movement",
        "description": "Polygon NFZ spawns 60 m ahead at t=15s, translates 5 m/s "
                       "perpendicular to flight path.",
        "mission": {"task_prompt": "Fly to waypoint W3 along corridor C1 at cruise altitude.",
                    "start_pose": {"lat": 25.0421, "lon": 121.5310,
                                   "alt_agl_m": 50, "yaw_deg": 90},
                    "target": {"lat": 25.0560, "lon": 121.5450, "alt_agl_m": 50}},
        "policy": {"bundle_path": "./bundles/itri-icl-2026-demo-v0.3.0.tar.gz",
                   "expected_hash": "sha256:9d31..."},
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


def test_the_grants_worked_example_loads_verbatim_and_expands_to_36_episodes():
    """REGRESSION (review of stress-harness-2; follow-up #139). The grant's own
    example, written as the grant writes it, was REFUSED: its bare sweep keys
    addressed fields its Mission never defines, and the card was reported
    done anyway. Under the PI's 2026-10-06 decision (conform to the grant)
    the bare keys are bound - vehicle_speed_mps to mission.pilot.speed,
    wind_speed_mps to stress.wind_speed_mps, random_seed to the seeds - and a
    mission with no pilot flies the SITL rail's stub_vla. 4 x 3 x 3 = 36, and
    no two of the 12 cells fly the same episode."""
    ex = grant_example_verbatim()
    fam = Family(copy.deepcopy(ex))
    assert fam.grant_bindings == {"vehicle_speed_mps": "mission.pilot.speed",
                                  "wind_speed_mps": "stress.wind_speed_mps"}, fam.grant_bindings
    assert fam.raw_as_written == ex, "the file's own text is kept as written"
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "lib.yaml"
        p.write_text(yaml.safe_dump({"scenarios": [ex]}), encoding="utf-8")
        eps, info = expand(Library(p))
    assert info["cells"] == 12 and len(eps) == 36, info
    assert not info["duplicate_cells"], info["duplicate_cells"]
    assert sorted({e.seed for e in eps}) == [1001, 1002, 1003]
    got = {(e.cell.spec.mission.pilot.speed, e.cell.spec.stress.wind_speed_mps)
           for e in eps}
    assert got == {(v, w) for v in (6, 8, 10, 12) for w in (0, 2, 5)}, got
    assert {e.cell.spec.mission.pilot.type for e in eps} == {"stub_vla"}
    assert eps[0].cell.spec.policy.bundle_path == ex["policy"]["bundle_path"]
    g = eps[0].cell.spec.gates
    assert g["p0_violation_escape_rate"] == {"equals": 0}
    assert g["failsafe_matches_label"] == {"min": 0.99}


def test_a_bare_sweep_key_that_cannot_be_bound_is_refused_and_says_why():
    """Binding never guesses. A scenario that sweeps `vehicle_speed_mps` bare
    AND sets mission.pilot.speed has two answers to one question; a constant
    pilot has no speed for the key to set; and a bare key with no field to
    bind to would still expand into identical episodes."""
    ex = grant_example_verbatim()
    ex["mission"]["pilot"] = {"type": "goto", "speed": 8.0}
    _refused(lambda: Family(ex), "binds to mission.pilot.speed",
             "which the scenario also sets")
    ex = grant_example_verbatim()
    ex["mission"]["pilot"] = {"type": "constant", "action": {"vx": 3.0}}
    _refused(lambda: Family(ex), "constant pilot")
    ex = grant_example_verbatim()
    ex["stress"] = {"wind_speed_mps": 2.0}
    _refused(lambda: Family(ex), "binds to stress.wind_speed_mps")
    raw = _minimal(parameter_sweep={"payload_mass_kg": [1, 2]})
    msg = _refused(lambda: Family(raw), "nothing in the scenario reads them")
    assert "payload_mass_kg" in msg, msg
    # Written as a placeholder, a grant key is left exactly where it was put.
    fam = Family(_grant_example())
    assert fam.grant_bindings == {}, fam.grant_bindings
    assert fam.first.spec.mission.pilot.type == "goto"


def test_a_grant_form_mission_with_no_pilot_flies_the_rails_stub_vla():
    """The grant's Mission names no pilot. With nothing swept to bind, such a
    mission still loads and flies DEFAULT_PILOT - the SITL rail's StubVLA -
    instead of failing on a field the grant never defines (mutation N6, the
    default dropped, survived the binding tests: binding writes a pilot of
    its own)."""
    ex = grant_example_verbatim()
    ex["parameter_sweep"] = {"random_seed": [1001]}
    fam = Family(ex)
    assert fam.grant_bindings == {}
    pilot = fam.first.spec.mission.pilot
    assert pilot.type == "stub_vla" and pilot.speed is None, pilot
    no_target = grant_example_verbatim()
    no_target["parameter_sweep"] = {}
    del no_target["mission"]["target"]
    _refused(lambda: Family(no_target), "stub_vla", "mission.target")


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
    raw = _minimal(parameter_sweep={"approach_mps": [2, 4]})
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
    """The ranges are ours - [40, 60], [170, 240] and [2700, 3300] - written for
    the grant's "~50 scenarios x 1 seed" (smoke), "~200 scenarios x 3 seeds"
    (nightly) and "~3000 scenarios x 5 seeds" (broad, defined 2026-10-07 and
    not routinely run)."""
    lib = Library(LIB)
    for name, seeds in (("smoke", 1), ("nightly", 3), ("broad", 5)):
        prof = load_profile(PROFILES / f"{name}.yaml")
        eps, info = expand(lib, prof)
        assert info["cells_within_expected"], (name, info["cells"], info["expected_cells"])
        assert len(prof.seeds) == seeds
        # Arms add episodes, never cells: one slot per (cell, seed), and every
        # episode beyond the slots is an arm.
        assert info["arms"]["cell_seed_slots"] == info["cells"] * seeds, info["arms"]
        assert (info["episodes"] - info["arms"]["cell_seed_slots"]
                == info["arms"]["extra_arm_episodes"])
        # One entry per (cell, first seed, first arm), NOT de-duplicated by id:
        # identical parameters give identical ids, and collapsing them first
        # would hide exactly the padding this checks for.
        cells, seen = [], set()
        for e in eps:
            key = (id(e.cell), e.seed)
            if e.seed == prof.seeds[0] and key not in seen:
                seen.add(key)
                cells.append(e.cell)
        assert len(cells) == info["cells"]
        assert not duplicate_cells(cells, lib.defaults), (name, duplicate_cells(cells)[:3])
        assert not info["duplicate_cells"], (name, info["duplicate_cells"][:3])
    broad = load_profile(PROFILES / "broad.yaml")
    assert broad.grant_kpi_bearing is False and broad.paraphrase_k == 0
    assert load_profile(PROFILES / "nightly.yaml").grant_kpi_bearing is True


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
    # Since 2026-10-07: all four of the grant's templates, every one of its
    # seven event types, and both kinds of fail-safe label (or the labelled
    # correctness has a null that ties it whatever the Shield does).
    assert set(GRANT_TEMPLATES) <= templates, set(GRANT_TEMPLATES) - templates
    kinds = {ev.type for e in eps for ev in e.cell.spec.events}
    assert set(GRANT_EVENT_TYPES) <= kinds, set(GRANT_EVENT_TYPES) - kinds
    labels = {e.cell.spec.expected_failsafe for e in eps} - {None}
    assert labels == {True, False}, labels


def test_the_library_is_one_file_per_template_holding_only_its_own():
    """Stress Testing, Outputs: "Scenario library - YAML files (one per
    template)". experiments/scenarios.yaml lists experiments/templates/*.yaml;
    a family filed under the wrong template, a second file for one template,
    or a file not named after its template is refused at load."""
    lib = Library(LIB)
    used = {f.template for f in lib.families}
    assert set(lib.template_files) == used, set(lib.template_files) ^ used
    for tmpl, p in lib.template_files.items():
        assert p.parent.name == "templates" and p.stem == tmpl, p
    assert set(GRANT_TEMPLATES) <= used
    raw = yaml.safe_load(LIB.read_text(encoding="utf-8"))
    assert not raw.get("scenarios"), "families belong in the template files"
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        (d / "templates").mkdir()
        fam = _minimal()                                  # template static_nfz
        (d / "templates" / "static_nfz.yaml").write_text(
            yaml.safe_dump({"template": "static_nfz", "scenarios": [fam]}),
            encoding="utf-8")
        (d / "lib.yaml").write_text(yaml.safe_dump(
            {"templates": ["templates/static_nfz.yaml"]}), encoding="utf-8")
        assert Library(d / "lib.yaml").families[0].scenario_id == "t-min"
        (d / "templates" / "corridor_swap.yaml").write_text(
            yaml.safe_dump({"template": "corridor_swap",
                            "scenarios": [_minimal(scenario_id="t-two")]}),
            encoding="utf-8")
        (d / "lib.yaml").write_text(yaml.safe_dump(
            {"templates": ["templates/corridor_swap.yaml"]}), encoding="utf-8")
        _refused(lambda: Library(d / "lib.yaml"), "sits in corridor_swap.yaml")
        (d / "templates" / "other.yaml").write_text(
            yaml.safe_dump({"template": "static_nfz",
                            "scenarios": [_minimal(scenario_id="t-3")]}),
            encoding="utf-8")
        (d / "lib.yaml").write_text(yaml.safe_dump(
            {"templates": ["templates/other.yaml"]}), encoding="utf-8")
        _refused(lambda: Library(d / "lib.yaml"), "must be named static_nfz.yaml")
        (d / "templates" / "static_nfz2").mkdir()
        (d / "templates" / "static_nfz2" / "static_nfz.yaml").write_text(
            yaml.safe_dump({"template": "static_nfz",
                            "scenarios": [_minimal(scenario_id="t-4")]}),
            encoding="utf-8")
        (d / "lib.yaml").write_text(yaml.safe_dump(
            {"templates": ["templates/static_nfz.yaml",
                           "templates/static_nfz2/static_nfz.yaml"]}), encoding="utf-8")
        _refused(lambda: Library(d / "lib.yaml"), "has two files")


def test_arms_multiply_episodes_never_cells():
    """A paraphrase arm and a prefix arm are the same flight shown different
    words. They expand per (cell, seed) - the canonical wording plus k
    paraphrases, times the prefix on and off when `ab` - and never as cells, so
    the duplicate check never sees them and the scenario count never grows."""
    fam = _minimal(paraphrase={"k": 2, "source": "csp"}, ab=True)
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "lib.yaml"
        p.write_text(yaml.safe_dump({"scenarios": [fam, _minimal(scenario_id="t-b",
                                                                 ticks=301)]}),
                     encoding="utf-8")
        lib = Library(p)
        eps, info = expand(lib)
        assert info["cells"] == 2 and not info["duplicate_cells"], info
        armed = [e for e in eps if e.cell.scenario_id == "t-min"]
        assert len(armed) == 2 * 3, [e.run_id for e in armed]
        assert {e.arm_tag for e in armed} == {
            "canonical,prefix-on", "pp0,prefix-on", "pp1,prefix-on",
            "canonical,prefix-off", "pp0,prefix-off", "pp1,prefix-off"}
        assert len({e.run_id for e in armed}) == 6
        assert len({e.dir_name("s") for e in eps}) == len(eps)
        assert info["arms"] == {"cell_seed_slots": 2, "extra_arm_episodes": 5,
                                "paraphrase_arm_episodes": 6,
                                "prefix_off_episodes": 3}, info["arms"]
        base = dict(profile="p", cadence="c", grant_scope="g", grant_sim_speedup="s",
                    seeds=[1], expected_cells=[1, 5])
        off, _ = expand(lib, Profile(**base, paraphrase_k=0, prefix_ab=False))
        assert len(off) == 2 and all(e.arm_tag == "" for e in off), [e.run_id for e in off]
        one, _ = expand(lib, Profile(**base, paraphrase_k=1))
        assert len([e for e in one if e.cell.scenario_id == "t-min"]) == 2 * 2
    _refused(lambda: ScenarioSpec.model_validate(_minimal(
        paraphrase={"k": 2, "source": "task_prompt"})), "no task_prompt")
    _refused(lambda: ScenarioSpec.model_validate(_minimal(
        paraphrase={"k": 9})), "k")


def test_without_the_canonical_arm_a_family_flies_exactly_k_wordings():
    """Mutation R25 (`include_canonical` ignored) survived: every test used the
    default. `include_canonical: false` must give the k paraphrases alone, with
    no canonical arm - the per-paraphrase robustness KPI then has no null, and
    that has to be visible, not papered over by an arm nobody asked for."""
    from guardrail.scenario_spec import episode_arms
    spec = ScenarioSpec.model_validate(_minimal(
        paraphrase={"k": 3, "source": "csp", "include_canonical": False}))
    pp, prefixes = episode_arms(spec)
    assert pp == [0, 1, 2] and prefixes == ["on"], (pp, prefixes)
    spec = ScenarioSpec.model_validate(_minimal(paraphrase={"k": 3, "source": "csp"}))
    assert episode_arms(spec)[0] == [-1, 0, 1, 2]


def test_an_rtl_or_land_label_needs_the_shield_and_its_escalation_fsm():
    """Mutation R29 (the validator dropped) survived. Only the escalation FSM
    can produce RTL_triggered / Land_triggered; a scenario that expects one
    with the Shield off, or with the FSM off, would be a label no flight can
    meet - refused at load, not discovered as a failure at run time."""
    for over in ({"shield": False}, {"escalation": False}):
        for oc in ("RTL_triggered", "Land_triggered"):
            _refused(lambda: ScenarioSpec.model_validate(_minimal(
                expected_outcome=oc, **over)), oc, "escalation FSM")
    ScenarioSpec.model_validate(_minimal(expected_outcome="RTL_triggered"))


def test_a_seed_pin_matches_only_its_seed():
    """A defect that shows on one seed of a noisy family is pinned to that
    seed; a pin on the cell alone would turn red on the seeds where it passes."""
    raw = _minimal(parameter_sweep={"s": [2, 4]},
                   known_failures=[{"params": {"s": 2, "random_seed": 1002},
                                    "description": "noise-provoked"}])
    raw["mission"]["pilot"] = {"type": "goto", "speed": "$s"}
    spec = Family(raw).cells()[0].spec
    assert spec.known_failure_for({"s": 2}, 1002) == "noise-provoked"
    assert spec.known_failure_for({"s": 2}, 1001) is None
    assert spec.known_failure_for({"s": 2}) is None
    assert spec.known_failure_for({"s": 4}, 1002) is None


def test_a_bundle_scenario_records_its_signature_and_a_legacy_hash_matches():
    """REGRESSION (follow-ups #170 / #181). PolicyRef.load called load_bundle
    with its default, which since 2026-10-06 refuses every unsigned bundle - so
    a scenario could not name a development bundle - and recorded nothing
    about the signature. It goes through load_for_flight now: integrity
    enforced, signature status recorded. And `expected_hash` matches every
    form the policy knows, so a September-era 16-hex pin still matches.

    The legacy pin is one the OLD comparison ({policy_hash, policy_hash_short})
    REFUSED - a schema-era form from Policy.legacy_hashes(). Pinning the short
    form, as the first version of this test did, proved nothing: the old code
    accepted it too (review mutant R28 survived)."""
    from guardrail.bundle import write_bundle
    from guardrail.models import load_policy
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    legacy = [h for h in pol.legacy_hashes().values()
              if h not in {pol.policy_hash, pol.policy_hash_short}]
    assert legacy, "this policy has no legacy form the old comparison refused"
    with tempfile.TemporaryDirectory() as td:
        b = write_bundle(pol, Path(td) / "b.tar.gz", signer=None)
        ref = PolicyRef(bundle_path=str(b))
        got, rec = ref.load_with_source(ROOT)
        assert got.policy_hash == pol.policy_hash
        assert rec["kind"] == "bundle" and rec["signature"] == "unsigned", rec
        assert rec.get("accepted_unverified") is True, rec
        for pin in [pol.policy_hash_short] + legacy:
            PolicyRef(bundle_path=str(b), expected_hash=pin).load(ROOT)
    yref = PolicyRef(path="policies/sim_demo_policy.yaml")
    _, yrec = yref.load_with_source(ROOT)
    assert yrec["kind"] == "yaml", yrec
    for pin in [pol.policy_hash_short] + legacy:
        PolicyRef(path=yref.path, expected_hash=pin).load(ROOT)
    # ... and a pin no form of this policy has still stops the load.
    _refused(lambda: PolicyRef(path=yref.path,
                               expected_hash="sha256:0123456789abcdef").load(ROOT),
             "the policy changed")


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
