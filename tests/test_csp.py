"""The Prefix Compiler's typed Constraint Summary Pack (CSP) and its four KPIs.

Run either way:
    pytest tests/test_csp.py -v
    python tests/test_csp.py

WHY THIS FILE EXISTS

The grant's Prefix Compiler page locks four acceptance KPIs (Prefix Compiler
PDF p5): the token budget is configurable and respected; P0 coverage is 100 %
across the test fixtures; `CSPBudgetExceeded` is raised, never a silent
truncation; and `relevance_explanations[rule_id]` is populated for every rule in
the CSP. Until 2026-10-06 none of them had a test, because none of them had
code: `summary_pack()` took no arguments and returned a plain dict.

Each KPI below is tested together with the thing that would fake it:

  * coverage      - a coverage function that cannot say 0 % is not measuring,
                    so it is run on an empty CSP first;
  * budget        - "tokens used <= budget" also holds for a prompt that was
                    never long enough to need cutting, so the test proves the
                    untruncated prompt was over budget;
  * overflow      - checked at the exact boundary (budget = P0 tokens passes,
                    one token less raises), not with a budget of 1;
  * explanations  - a constant string is "populated" for every rule, so the
                    test also requires the text to change with the mission.

A fifth property is the page's own definition of the compiler (PDF p1): "a
stateless function: same policy bundle + same mission context + same
rule-selection seed -> same CSP". It is tested across two interpreter
processes with different hash seeds, because a set iterated in the compiler
would pass any in-process determinism test. It holds GIVEN A CLOCK: without
one the issue stamp comes from the wall clock, so the CSP is compared by
`csp_content_hash` (every field but the stamp), which is tested under a clock
that ticks between two compilations.

A test that needs an optional dependency (jsonschema, the OpenVLA tokenizer)
SKIPs visibly under both runners - pytest.skip under pytest, a SKIP line and a
"skipped" count under the standalone runner - and never passes in its place.
"""
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import load_policy                                  # noqa: E402
from guardrail.models import Action4D, Policy, State               # noqa: E402

# Imported defensively so that, run against a tree without the CSP module, each
# test fails on its own line with the reason instead of the whole file dying at
# import. That is how the "failing on the old code" evidence was produced.
try:
    from guardrail import csp as C                                 # noqa: E402
    from guardrail import compiler as K                            # noqa: E402
    _IMPORT_ERROR = None
except ImportError as e:                                           # noqa: BLE001
    C = K = None
    _IMPORT_ERROR = e
from guardrail.compiler import ConstraintCompiler, Mission         # noqa: E402

POLICIES = sorted((ROOT / "policies").glob("*.yaml"))
SURVEY = ROOT / "policies" / "corridor_survey.yaml"
PED = ROOT / "policies" / "sitl_pedestrian.yaml"
DEMO = ROOT / "policies" / "sim_demo_policy.yaml"

MONDAY_0800 = datetime(2026, 10, 5, 8, 0, 0)
SATURDAY_1000 = datetime(2026, 10, 10, 10, 0, 0)

SKIP = "SKIP"


def _skip(reason):
    """Skip visibly under either runner. Under pytest a returned value is a
    PASS (with a warning), which is exactly the 'skipped test reads as passed'
    failure csp.openvla_token_counter's docstring forbids, so raise there."""
    if os.environ.get("PYTEST_CURRENT_TEST"):
        import pytest
        pytest.skip(reason)
    print(f"      skipped: {reason}")
    return SKIP


def _need():
    assert C is not None and K is not None, f"guardrail.csp missing: {_IMPORT_ERROR}"


def _mission(tx=30.0, ty=30.0, alt=15.0, speed=4.0, text=None, sx=0.0, sy=0.0):
    return Mission(task_text=text or f"fly to ({tx:g}, {ty:g}) altitude {alt:g}",
                   target_x=tx, target_y=ty, cruise_alt_m=alt,
                   speed_pref_mps=speed, start_x=sx, start_y=sy)


def _sq(x0, y0, x1, y1):
    return [{"x": x0, "y": y0}, {"x": x1, "y": y0}, {"x": x1, "y": y1},
            {"x": x0, "y": y1}]


# A policy with every priority and every rule type, so the budget has
# something to cut. The shipped fixtures use only P0 and P1 (no P2), which is
# exactly why truncation could not be tested on them.
MIX = {
    "policy_id": "mix-for-budget",
    "constraints": [
        {"id": "nfz-on-path", "type": "polygon_fence", "priority": "P0",
         "vertices": _sq(40, -5, 50, 5)},
        {"id": "alt-band", "type": "altitude_envelope", "priority": "P0",
         "alt_min_m": 5, "alt_max_m": 50},
        {"id": "kin-caps", "type": "kinematic_envelope", "priority": "P1",
         "speed_max_mps": 5, "climb_rate_max_mps": 2, "yaw_rate_max_dps": 45},
        {"id": "nfz-near-p1", "type": "polygon_fence", "priority": "P1",
         "vertices": _sq(60, 8, 70, 18)},
        {"id": "nfz-far-p1", "type": "polygon_fence", "priority": "P1",
         "vertices": _sq(60, 120, 70, 130)},
        {"id": "keep-off-buildings", "type": "obstacle_clearance",
         "priority": "P2", "min_clearance_m": 3},
        {"id": "standoff-any", "type": "subject_standoff", "priority": "P2",
         "min_range_m": 5},
    ],
}
MIX_MISSION = dict(tx=100.0, ty=0.0, alt=15.0, speed=4.0)


def _mix():
    return Policy.model_validate(MIX)


# --------------------------------------------------------------------------- #
# The legacy sentences, frozen as the oracle.
#
# Copied verbatim from guardrail/compiler.py at commit 445bdbe (2026-09-01).
# The templates must reproduce these byte for byte for every rule without a
# time window, so a flight that read `natural_language_prompt` before this
# change reads the same text after it.
# --------------------------------------------------------------------------- #

def _legacy_sentences(policy):
    from guardrail.models import (AltitudeEnvelope, Corridor, KinematicEnvelope,
                                  ObstacleClearance, PolygonFence,
                                  SubjectStandoff)
    nl = []
    for f in policy.by_type(PolygonFence):
        nl.append(f"Never enter zone '{f.id}'.")
    for c in policy.by_type(Corridor):
        nl.append(f"Stay inside corridor '{c.id}', within "
                  f"{c.half_width_m:g} m of its centerline and between "
                  f"{c.altitude_floor_m:g} m and {c.altitude_ceiling_m:g} m.")
    for e in policy.by_type(AltitudeEnvelope):
        nl.append(f"Stay between {e.alt_min_m:g} m and {e.alt_max_m:g} m altitude.")
    for k in policy.by_type(KinematicEnvelope):
        nl.append(f"Keep speed at or below {k.speed_max_mps:g} m/s, climb rate "
                  f"below {k.climb_rate_max_mps:g} m/s and turn rate below "
                  f"{k.yaw_rate_max_dps:g} deg/s.")
    for o in policy.by_type(ObstacleClearance):
        nl.append(f"Keep at least {o.min_clearance_m:g} m from any building.")
    for s in policy.by_type(SubjectStandoff):
        who = "anything you are following" if s.subject_class == "*" \
            else f"any {s.subject_class}"
        nl.append(f"Keep at least {s.min_range_m:g} m away from {who}.")
    return nl


# --------------------------------------------------------------------------- #
# WP2-04: the locked schema
# --------------------------------------------------------------------------- #

LOCKED = ["csp_version", "policy_hash", "generation", "issued_at", "mission_id",
          "lookahead_s", "P0_constraints", "P1_summary", "P2_summary",
          "allowed_action_set", "forbidden_action_set", "cost_map_ref",
          "natural_language_prompt", "relevance_explanations"]


def test_the_csp_model_carries_every_locked_field():
    """Prefix Compiler PDF p1-2: the Pydantic surface, field by field."""
    _need()
    fields = set(C.CSP.model_fields)
    missing = [f for f in LOCKED if f not in fields]
    assert not missing, f"locked fields absent: {missing}"
    assert C.CSP_VERSION == "1.0"
    assert list(C.LOCKED_FIELDS) == LOCKED, C.LOCKED_FIELDS


def test_the_schema_export_requires_the_locked_fields():
    _need()
    out = Path(tempfile.mkdtemp()) / "csp.schema.json"
    C.write_csp_schema(out)
    schema = json.loads(out.read_text(encoding="utf-8"))
    req = set(schema.get("required", []))
    # P2_summary and cost_map_ref default to None in the spec; the rest are
    # mandatory there and must be mandatory here.
    for f in LOCKED:
        if f in ("P2_summary", "cost_map_ref"):
            continue
        assert f in req, f"{f} is not required by the exported schema"
    assert schema["properties"]["csp_version"].get("const") == "1.0" or \
        schema["properties"]["csp_version"].get("enum") == ["1.0"], \
        schema["properties"]["csp_version"]


def test_an_emitted_csp_validates_against_the_exported_schema_and_a_broken_one_does_not():
    _need()
    try:
        import jsonschema
    except ImportError:
        # vla-drone (3.11) has no jsonschema; vla-real does
        return _skip("jsonschema is not installed in this env")
    csp = ConstraintCompiler(load_policy(SURVEY)).compile_csp(
        _mission(), now=MONDAY_0800)
    doc = json.loads(csp.model_dump_json())
    jsonschema.validate(doc, C.csp_schema())
    broken = dict(doc)
    del broken["P0_constraints"]
    try:
        jsonschema.validate(broken, C.csp_schema())
    except jsonschema.ValidationError:
        return
    raise AssertionError("a CSP without P0_constraints passed the schema")


# Where csp.schema.json (PDF p5 output table) may be committed. Neither path is
# this unit's to write; the coordinator generates it with
# `python -m guardrail.compiler schema <path>`. Until one exists the drift test
# SKIPs visibly - it does not pass.
SCHEMA_PATHS = [ROOT / "docs" / "data" / "csp.schema.json",
                ROOT / "guardrail" / "csp.schema.json"]


def test_the_committed_schema_has_not_drifted_from_the_model():
    """A committed csp.schema.json that no longer matches guardrail.csp.CSP
    would tell a consumer the wrong contract while every other test passed."""
    _need()
    found = [p for p in SCHEMA_PATHS if p.is_file()]
    if not found:
        return _skip("no committed csp.schema.json at "
                     + " or ".join(p.relative_to(ROOT).as_posix() for p in SCHEMA_PATHS))
    for p in found:
        on_disk = json.loads(p.read_text(encoding="utf-8"))
        assert on_disk == C.csp_schema(), (
            f"{p.relative_to(ROOT).as_posix()} has drifted from guardrail.csp.CSP; "
            f"regenerate with `python -m guardrail.compiler schema {p.relative_to(ROOT).as_posix()}`")


def test_the_csp_round_trips_through_json():
    _need()
    csp = ConstraintCompiler(load_policy(PED)).compile_csp(_mission(), now=MONDAY_0800)
    again = C.CSP.model_validate_json(csp.model_dump_json())
    assert again == csp


def test_a_misspelt_field_is_refused_rather_than_dropped():
    """extra='forbid': a typo like P0_constraint would otherwise vanish quietly
    and the CSP would validate with the real field missing... or defaulted."""
    _need()
    from pydantic import ValidationError
    csp = ConstraintCompiler(load_policy(PED)).compile_csp(_mission(), now=MONDAY_0800)
    data = csp.model_dump()
    data["P0_constraint"] = data["P0_constraints"]
    try:
        C.CSP.model_validate(data)
    except ValidationError:
        return
    raise AssertionError("an unknown CSP field was accepted")


# --------------------------------------------------------------------------- #
# WP2-05: temporal and spatial filtering, risk grading
# --------------------------------------------------------------------------- #

def _row(csp, rid):
    rows = [r for r in csp.selection.rules if r.id == rid]
    assert len(rows) == 1, (rid, [r.id for r in csp.selection.rules])
    return rows[0]


def test_the_weekday_school_zone_is_left_out_on_a_saturday_and_says_why():
    """REGRESSION (audit WP2-05): nfz-school is in force Mon-Fri 07:30-17:30,
    and the old compiler told the model 'Never enter zone' at all times."""
    _need()
    assert SATURDAY_1000.strftime("%a") == "Sat"
    csp = ConstraintCompiler(load_policy(SURVEY)).compile_csp(
        _mission(), now=SATURDAY_1000)
    row = _row(csp, "nfz-school")
    assert row.decision == "filtered_inactive", row
    assert "Mon-Fri 07:30-17:30" in row.reason, row.reason
    assert "nfz-school" not in {p.id for p in csp.P0_constraints}
    assert "nfz-school" not in csp.natural_language_prompt
    assert "nfz-school" not in csp.relevance_explanations


def test_the_school_zone_is_kept_on_monday_morning_with_its_window_in_words():
    _need()
    assert MONDAY_0800.strftime("%a") == "Mon"
    csp = ConstraintCompiler(load_policy(SURVEY)).compile_csp(
        _mission(), now=MONDAY_0800)
    assert _row(csp, "nfz-school").decision == "kept"
    assert "Never enter zone 'nfz-school' (in force Mon-Fri 07:30-17:30)." \
        in csp.natural_language_prompt, csp.natural_language_prompt
    p0 = {p.id: p for p in csp.P0_constraints}
    assert p0["nfz-school"].in_force == "Mon-Fri 07:30-17:30", p0["nfz-school"]


def test_the_old_sentences_dropped_the_window():
    """The defect itself, pinned: `_sentences()` must now carry the schedule."""
    said = " ".join(ConstraintCompiler(load_policy(SURVEY))._sentences())
    assert "(in force Mon-Fri 07:30-17:30)" in said, said


def test_every_schedule_shape_reads_unambiguously():
    """Overnight windows wrap past midnight (models.Recurrence); written bare,
    '22:00-06:00' could be read as an empty window, so it is marked."""
    _need()

    def fence(days, start="00:00", end="23:59"):
        return Policy.model_validate({"policy_id": "w", "constraints": [
            {"id": "z", "type": "polygon_fence", "vertices": _sq(0, 0, 1, 1),
             "valid_time": {"recurrence": {"days": days, "start_time": start,
                                           "end_time": end}}}]}).constraints[0]
    week = ["Mon", "Tue", "Wed", "Thu", "Fri"]
    every = week + ["Sat", "Sun"]
    assert K.window_text(fence(week, "07:30", "17:30")) == "Mon-Fri 07:30-17:30"
    assert K.window_text(fence(["Sat", "Sun"])) == "Sat, Sun"
    assert K.window_text(fence(["Fri", "Mon", "Wed"])) == "Mon, Wed, Fri"
    assert K.window_text(fence(every, "22:00", "06:00")) == "every day 22:00-06:00 overnight"
    assert K.window_text(fence(every)) is None, "a schedule covering all time is 'always'"
    assert K.render_sentence(fence(every, "22:00", "06:00")) == \
        "Never enter zone 'z' (in force every day 22:00-06:00 overnight)."


def test_duplicate_rule_ids_are_refused_not_merged():
    """The CSP keys everything by rule id. Two rules sharing one would collapse
    into a single row and the other would disappear from the record."""
    _need()
    pol = Policy.model_validate({"policy_id": "dup", "constraints": [
        {"id": "a", "type": "polygon_fence", "vertices": _sq(10, 10, 20, 20)},
        {"id": "a", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 50}]})
    try:
        ConstraintCompiler(pol).compile_csp(_mission(), now=None)
    except ValueError as e:
        assert "duplicated" in str(e), e
        return
    raise AssertionError("a policy with two rules called 'a' compiled")


def test_a_rule_switching_on_inside_the_lookahead_is_kept_and_time_critical():
    _need()
    t = datetime(2026, 10, 5, 7, 29, 58)
    csp = ConstraintCompiler(load_policy(SURVEY)).compile_csp(
        _mission(), lookahead_s=5.0, now=t)
    row = _row(csp, "nfz-school")
    assert row.decision == "kept", row
    assert row.time_critical == 1.0, row
    assert "switches on" in csp.relevance_explanations["nfz-school"], \
        csp.relevance_explanations["nfz-school"]
    # ...and with a lookahead too short to reach 07:30 it is not in force.
    csp2 = ConstraintCompiler(load_policy(SURVEY)).compile_csp(
        _mission(), lookahead_s=1.0, now=t)
    assert _row(csp2, "nfz-school").decision == "filtered_inactive"


def test_a_rule_switching_off_inside_the_lookahead_is_time_critical_too():
    """PDF p4 grades time-criticality on a window that 'switches on or off'
    inside the lookahead. Recurrence's end is inclusive (17:30 means through
    17:30:59), so 17:30:58 + 5 s crosses the switch-off; mid-morning does not
    (the null: a grader that always said 1.0 would pass the first half)."""
    _need()
    pol = load_policy(SURVEY)
    t = datetime(2026, 10, 5, 17, 30, 58)
    csp = ConstraintCompiler(pol).compile_csp(_mission(), lookahead_s=5.0, now=t)
    row = _row(csp, "nfz-school")
    assert row.decision == "kept" and row.time_critical == 1.0, row
    assert "switches off" in csp.relevance_explanations["nfz-school"], \
        csp.relevance_explanations["nfz-school"]
    mid = ConstraintCompiler(pol).compile_csp(_mission(), lookahead_s=5.0,
                                              now=datetime(2026, 10, 5, 11, 0, 0))
    assert _row(mid, "nfz-school").time_critical == 0.0, _row(mid, "nfz-school")
    assert "in force at issue time" in mid.relevance_explanations["nfz-school"]


def test_the_region_filter_measures_to_the_margin_ring():
    """The Shield enforces a zone's margin ring, so a zone whose vertices lie
    just outside the mission region but whose ring reaches into it is in
    scope. Mission (0,0)->(30,30) at 4 m/s, no speed cap: reach 20 m, region
    x <= 30 + 200 + 20 = 250. The zone starts at x = 260."""
    _need()

    def pol(margin):
        return Policy.model_validate({"policy_id": "ring", "constraints": [
            {"id": "z", "type": "polygon_fence", "vertices": _sq(260, 0, 270, 10),
             "margin_m": margin}]})
    wide = ConstraintCompiler(pol(15.0)).compile_csp(_mission(), now=None)
    assert wide.selection.region["x_max"] == 250.0, wide.selection.region
    assert _row(wide, "z").decision == "kept", _row(wide, "z")
    thin = ConstraintCompiler(pol(5.0)).compile_csp(_mission(), now=None)
    assert _row(thin, "z").decision == "filtered_out_of_region", _row(thin, "z")


def test_reach_uses_the_tightest_hard_speed_cap_else_the_requested_speed():
    """reach = speed bound x lookahead widens the region and picks the
    geometry scale. A soft cap does not bound what the vehicle can do."""
    _need()

    def pol(*caps):
        cs = [{"id": f"k{i}", "type": "kinematic_envelope", "priority": "P1",
               "constraint_type": ct, "speed_max_mps": v, "climb_rate_max_mps": 1,
               "yaw_rate_max_dps": 30} for i, (v, ct) in enumerate(caps)]
        return Policy.model_validate({"policy_id": "reach", "constraints": cs})
    m = _mission(speed=4.0)
    assert ConstraintCompiler(pol((2.0, "hard"), (3.0, "hard"))).compile_csp(
        m, now=None).selection.reach_m == 10.0
    assert ConstraintCompiler(pol((2.0, "soft"))).compile_csp(
        m, now=None).selection.reach_m == 20.0
    assert ConstraintCompiler(Policy.model_validate({"policy_id": "none", "constraints": [
        {"id": "z", "type": "polygon_fence", "vertices": _sq(0, 0, 1, 1)}]})).compile_csp(
        m, now=None).selection.reach_m == 20.0


def test_a_soft_rule_is_graded_below_the_same_hard_rule():
    """'hard >> soft' (PDF p4): soft severity is halved (SOFT_FACTOR)."""
    _need()
    pol = Policy.model_validate({"policy_id": "soft", "constraints": [
        {"id": "hard-p1", "type": "polygon_fence", "priority": "P1",
         "vertices": _sq(10, 10, 12, 12)},
        {"id": "soft-p1", "type": "polygon_fence", "priority": "P1",
         "constraint_type": "soft", "vertices": _sq(10, 10, 12, 12)},
        {"id": "soft-p0", "type": "polygon_fence", "constraint_type": "soft",
         "vertices": _sq(10, 10, 12, 12)}]})
    csp = ConstraintCompiler(pol).compile_csp(_mission(), now=None)
    assert _row(csp, "hard-p1").severity == 0.6
    assert _row(csp, "soft-p1").severity == 0.3
    assert _row(csp, "soft-p0").severity == 0.5
    assert _row(csp, "hard-p1").risk > _row(csp, "soft-p1").risk


def test_a_windowed_p1_rule_keeps_its_window_in_the_summary():
    """The summary is a second rendering of the rule; dropping the schedule
    there would repeat the defect the sentences had."""
    _need()
    pol = Policy.model_validate({"policy_id": "p1win", "constraints": [
        {"id": "zq", "type": "polygon_fence", "priority": "P1",
         "vertices": _sq(10, 10, 12, 12),
         "valid_time": {"recurrence": {"days": ["Mon", "Tue", "Wed", "Thu", "Fri"],
                                       "start_time": "07:30", "end_time": "17:30"}}}]})
    for now in (None, MONDAY_0800):
        csp = ConstraintCompiler(pol).compile_csp(_mission(), now=now)
        assert "no entry into 'zq' (Mon-Fri 07:30-17:30)" in csp.P1_summary, csp.P1_summary


def test_without_a_clock_every_rule_is_treated_as_in_force():
    """Absent means active (models.py ConstraintBase.active_at): a P0 rule must
    never switch itself off because nobody passed the time."""
    _need()
    csp = ConstraintCompiler(load_policy(SURVEY)).compile_csp(_mission(), now=None)
    assert _row(csp, "nfz-school").decision == "kept"
    assert "no clock" in csp.relevance_explanations["nfz-school"]
    assert csp.selection.time_filter is False


def test_a_far_fence_is_filtered_by_region_but_a_far_corridor_never_is():
    """A keep-in corridor binds everywhere: outside it is forbidden. Filtering
    it as 'far away' would hide exactly the case where the mission is outside."""
    _need()
    pol = Policy.model_validate({"policy_id": "far", "constraints": [
        {"id": "nfz-far", "type": "polygon_fence", "vertices": _sq(5000, 5000, 5010, 5010)},
        {"id": "corr-far", "type": "corridor",
         "centerline": [{"x": 4000, "y": 0}, {"x": 4100, "y": 0}], "width_m": 20},
    ]})
    csp = ConstraintCompiler(pol).compile_csp(_mission(), now=None)
    assert _row(csp, "nfz-far").decision == "filtered_out_of_region"
    assert _row(csp, "corr-far").decision == "kept"
    assert "outside" in csp.relevance_explanations["corr-far"]
    # and the region filter can be switched off entirely
    csp_all = ConstraintCompiler(pol).compile_csp(_mission(), now=None,
                                                  region_margin_m=None)
    assert _row(csp_all, "nfz-far").decision == "kept"


def test_risk_is_the_grant_weighted_sum_and_the_weights_are_configurable():
    """Prefix Compiler PDF p4: 0.5*sev + 0.3*prox + 0.2*tcrit."""
    _need()
    csp = ConstraintCompiler(_mix()).compile_csp(_mission(**MIX_MISSION), now=None)
    for r in csp.selection.rules:
        want = 0.5 * r.severity + 0.3 * r.proximity + 0.2 * r.time_critical
        assert abs(r.risk - want) < 1e-3, r
    assert _row(csp, "nfz-on-path").severity == 1.0
    assert _row(csp, "kin-caps").severity == 0.6
    assert _row(csp, "keep-off-buildings").severity == 0.3
    w = C.RiskWeights(severity=0.2, proximity=0.7, time_critical=0.1)
    csp_w = ConstraintCompiler(_mix()).compile_csp(_mission(**MIX_MISSION),
                                                   now=None, weights=w)
    r = _row(csp_w, "nfz-far-p1")
    assert abs(r.risk - (0.2 * r.severity + 0.7 * r.proximity)) < 1e-3, r
    assert csp_w.selection.weights == w
    try:
        C.RiskWeights(severity=0.5, proximity=0.5, time_critical=0.5)
    except ValueError:
        pass
    else:
        raise AssertionError("weights summing to 1.5 were accepted")


def test_a_fence_on_the_path_outranks_the_same_fence_far_away():
    _need()
    csp = ConstraintCompiler(_mix()).compile_csp(_mission(**MIX_MISSION), now=None)
    near, far = _row(csp, "nfz-near-p1"), _row(csp, "nfz-far-p1")
    assert near.proximity > far.proximity, (near, far)
    assert near.risk > far.risk, (near, far)
    assert _row(csp, "nfz-on-path").proximity == 1.0


# --------------------------------------------------------------------------- #
# KPI 1 and KPI 3 (WP2-02): the budget, and the loud failure
# --------------------------------------------------------------------------- #

def test_kpi_budget_is_respected_on_every_fixture_and_recounts_exactly():
    """KPI: 'CSP token / length budget is configurable and respected.'

    The stored count is re-derived from the stored text, so a CSP that wrote
    a stale or invented number would fail here, not just one that ran long."""
    _need()
    counter = C.TableTokenCounter()
    worst = 0
    for path in POLICIES:
        csp = ConstraintCompiler(load_policy(path)).compile_csp(_mission(), now=None)
        sel = csp.selection
        assert sel.budget_tokens == K.DEFAULT_BUDGET_TOKENS
        assert sel.tokens_used == counter.count(csp.natural_language_prompt), path.name
        assert sel.tokens_used <= sel.budget_tokens, (path.name, sel.tokens_used)
        assert sel.token_counter == counter.name
        worst = max(worst, sel.tokens_used)
    print(f"      max tokens used over {len(POLICIES)} fixtures: {worst} "
          f"of {K.DEFAULT_BUDGET_TOKENS} ({counter.name})")


def _order_without_budget(pol, mission):
    big = ConstraintCompiler(pol).compile_csp(mission, now=None, budget_tokens=100000)
    rows = [r for r in big.selection.rules if r.decision == "kept"]
    p0 = sorted([r for r in rows if r.priority == "P0"], key=lambda r: (-r.risk, r.id))
    rest = sorted([r for r in rows if r.priority != "P0"], key=lambda r: (-r.risk, r.id))
    return big, p0, rest


def test_kpi_truncation_drops_lowest_risk_first_and_lists_what_it_dropped():
    """The untruncated prompt must be over budget, or 'respected' proves nothing."""
    _need()
    pol, m = _mix(), _mission(**MIX_MISSION)
    counter = C.TableTokenCounter()
    big, p0, rest = _order_without_budget(pol, m)
    assert len(rest) >= 4, rest
    keep_n = 2
    text = " ".join([r.sentence for r in p0] + [r.sentence for r in rest[:keep_n]])
    budget = counter.count(text)
    full = counter.count(big.natural_language_prompt)
    assert full > budget, "the null: without truncation the prompt would fit anyway"
    csp = ConstraintCompiler(pol).compile_csp(m, now=None, budget_tokens=budget)
    kept = {r.id for r in csp.selection.rules if r.decision == "kept"}
    dropped = {r.id for r in csp.selection.rules if r.decision == "dropped_budget"}
    assert kept == {r.id for r in p0} | {r.id for r in rest[:keep_n]}, (kept, dropped)
    assert dropped == {r.id for r in rest[keep_n:]}, dropped
    # ...and pinned from the geometry, not from the compiler's own scores: a
    # P1 cap and the P1 zone 7 m off the path outrank both P2 rules and the P1
    # zone 120 m away. (With every risk equal, an id-ordered cut would keep
    # 'keep-off-buildings' instead - the mutation this line exists to catch.)
    assert kept - {r.id for r in p0} == {"kin-caps", "nfz-near-p1"}, kept
    assert csp.selection.tokens_used <= budget
    lo_kept = min(_row(csp, i).risk for i in kept if _row(csp, i).priority != "P0")
    hi_drop = max(_row(csp, i).risk for i in dropped)
    assert lo_kept >= hi_drop, (lo_kept, hi_drop)
    for i in dropped:
        assert "budget" in _row(csp, i).reason, _row(csp, i).reason
        # dropped from the TEXT, not from the record: still explained...
        assert i in csp.relevance_explanations
    # ...and still bounding the action sets, which are built from every
    # in-scope rule, not only the kept ones (nfz-far-p1 is dropped here)
    assert "nfz-far-p1" in dropped
    assert "nfz-far-p1" in csp.forbidden_action_set.no_translate_into_polygons
    # the summaries say what was cut; a P2 summary with no P2 rule kept is None
    assert "omitted for budget: nfz-far-p1" in csp.P1_summary, csp.P1_summary
    assert csp.P2_summary is None, csp.P2_summary
    assert ">= 3 m from buildings" in big.P2_summary, big.P2_summary


def test_truncation_is_a_strict_prefix_by_risk_not_skip_and_continue():
    """Once a rule does not fit, no LOWER-risk rule is admitted after it, even
    one short enough to fit. Otherwise a short low-risk clause could take the
    place of a long high-risk one and 'every kept rule outranks every dropped
    one' would be false. Built so the two readings disagree: the speed cap
    (risk 0.6, long sentence) does not fit; the P2 zone (risk 0.45, short) would."""
    _need()
    pol = Policy.model_validate({"policy_id": "prefix", "constraints": [
        {"id": "p0z", "type": "polygon_fence", "vertices": _sq(40, -5, 50, 5)},
        {"id": "kin", "type": "kinematic_envelope", "priority": "P1",
         "speed_max_mps": 5, "climb_rate_max_mps": 2, "yaw_rate_max_dps": 45},
        {"id": "zq", "type": "polygon_fence", "priority": "P2",
         "vertices": _sq(20, -2, 22, 2)}]})
    m = _mission(**MIX_MISSION)
    big, p0, rest = _order_without_budget(pol, m)
    assert [r.id for r in rest] == ["kin", "zq"], [(r.id, r.risk) for r in rest]
    counter = C.TableTokenCounter()
    assert counter.count(rest[0].sentence) > counter.count(rest[1].sentence)
    budget = counter.count(" ".join([p0[0].sentence, rest[1].sentence]))
    csp = ConstraintCompiler(pol).compile_csp(m, now=None, budget_tokens=budget)
    assert _row(csp, "kin").decision == "dropped_budget"
    assert _row(csp, "zq").decision == "dropped_budget", \
        "a lower-risk rule was let in after a higher-risk one was cut"
    assert csp.natural_language_prompt == p0[0].sentence


def test_kpi_p0_overflow_raises_at_the_exact_boundary_never_truncates():
    """KPI: 'CSPBudgetExceeded raised, never silently truncated.'"""
    _need()
    pol, m = _mix(), _mission(**MIX_MISSION)
    counter = C.TableTokenCounter()
    _, p0, _ = _order_without_budget(pol, m)
    p0_tokens = counter.count(" ".join(r.sentence for r in p0))
    ok = ConstraintCompiler(pol).compile_csp(m, now=None, budget_tokens=p0_tokens)
    assert ok.selection.tokens_used == p0_tokens
    assert {p.id for p in ok.P0_constraints} == {r.id for r in p0}
    # every P1/P2 rule is cut from the text here, and the P1 speed cap still
    # bounds the allowed action set (design doc section 2)
    assert _row(ok, "kin-caps").decision == "dropped_budget"
    assert ok.allowed_action_set.horizontal_speed_max_mps == 5.0, ok.allowed_action_set
    assert "kin-caps" in ok.allowed_action_set.source_rules
    try:
        ConstraintCompiler(pol).compile_csp(m, now=None, budget_tokens=p0_tokens - 1)
    except C.CSPBudgetExceeded as e:
        assert e.p0_tokens == p0_tokens and e.budget_tokens == p0_tokens - 1, e
        assert set(e.p0_ids) == {r.id for r in p0}, e.p0_ids
        return
    raise AssertionError("P0 text one token over budget did not raise")


def test_the_budget_is_configurable():
    _need()
    pol, m = _mix(), _mission(**MIX_MISSION)
    a = ConstraintCompiler(pol).compile_csp(m, now=None, budget_tokens=1000)
    _, p0, _ = _order_without_budget(pol, m)
    tight = C.TableTokenCounter().count(" ".join(r.sentence for r in p0))
    b = ConstraintCompiler(pol).compile_csp(m, now=None, budget_tokens=tight)
    assert a.selection.budget_tokens == 1000 and b.selection.budget_tokens == tight
    assert len(b.natural_language_prompt) < len(a.natural_language_prompt)
    try:
        ConstraintCompiler(pol).compile_csp(m, now=None, budget_tokens=0)
    except ValueError:
        return
    raise AssertionError("a zero budget was accepted")


def test_the_table_counter_is_an_upper_bound_on_the_openvla_tokenizer():
    """The default counter needs no model files, so it must never UNDER-count
    what OpenVLA's Llama-2 tokenizer would see - otherwise 'budget respected'
    under the default would not mean respected for the model.

    Needs the tokenizer (vla-real env + D:/models/openvla-7b). Without it this
    test SKIPs visibly rather than passing."""
    _need()
    exact = C.openvla_token_counter()
    if exact is None:
        return _skip("no OpenVLA tokenizer here (needs `tokenizers` and "
                     "D:/models/openvla-7b/tokenizer.json); the frozen-snapshot "
                     "test covers the bound without it")
    # the frozen snapshot must itself be right, or the tokenizer-free test
    # checks against wrong numbers
    n_snap = 0
    for t in _snapshot_texts():
        k = _snap_key(t)
        if k in _EXACT_SNAPSHOT:
            assert _EXACT_SNAPSHOT[k] == exact.count(t), (t, _EXACT_SNAPSHOT[k], exact.count(t))
            n_snap += 1
    assert n_snap > 0, "no snapshot text was produced"
    for w, n in _EXACT_CHUNKS.items():
        assert n == exact.count(w), (w, n, exact.count(w))
    # The table is MEASURED, so each entry must equal its chunk's exact count.
    # A whole-text check cannot see an entry one too low next to an
    # over-counted rule id; this can.
    off = [(k, v, exact.count(k)) for k, v in C._OPENVLA_TABLE.items() if v != exact.count(k)]
    assert not off, f"table entries that are not the exact count (chunk, table, exact): {off}"
    table = C.TableTokenCounter()
    ratios = []
    for path in POLICIES:
        for now in (None, MONDAY_0800):
            csp = ConstraintCompiler(load_policy(path)).compile_csp(_mission(), now=now)
            texts = [csp.natural_language_prompt, csp.P1_summary] + [
                r.sentence for r in csp.selection.rules if r.sentence]
            for t in texts:
                if not t:
                    continue
                e, h = exact.count(t), table.count(t)
                assert h >= e, (path.name, t, h, e)
                # the chunk-independence the table relies on
                assert e == sum(exact.count(w) for w in t.split(" ")), t
                ratios.append(h / e)
    print(f"      table/exact over {len(ratios)} texts: min {min(ratios):.3f} "
          f"max {max(ratios):.3f}")


def _snapshot_texts():
    """The texts the frozen exact counts below cover: on every fixture, without
    a clock and at Monday 08:00, the prompt, the P1/P2 summaries and every rule
    sentence. Sentences are included because a whole prompt has slack (rule
    ids are over-counted), and a table entry set too low can hide in it; a
    sentence made of tabled words has none."""
    for path in POLICIES:
        pol = load_policy(path)
        for now in (None, MONDAY_0800):
            csp = ConstraintCompiler(pol).compile_csp(_mission(), now=now)
            yield csp.natural_language_prompt
            yield from (t for t in (csp.P1_summary, csp.P2_summary) if t)
            yield from (r.sentence for r in csp.selection.rules if r.sentence)


def _snap_key(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:16]


def _snapshot_chunks():
    """Every table key, and every space-separated chunk of the snapshot texts
    (rule ids and numbers included). The table counter sums chunk counts, so
    a chunk-level bound is the one a whole-text check cannot hide: on a
    sentence like "Never enter zone 'nfz-school'." the over-counted id leaves
    enough slack to absorb 'Never' or 'zone' tabled one token too low (the
    2026-10-06 mutation run: both survived every whole-text check)."""
    chunks = set(C._OPENVLA_TABLE)
    for t in _snapshot_texts():
        chunks.update(w for w in t.split(" ") if w)
    return sorted(chunks)


def _print_snapshot():
    exact = C.openvla_token_counter()
    if exact is None:
        print("needs the OpenVLA tokenizer (vla-real env, D:/models/openvla-7b)")
        return 2
    snap = {}
    for t in _snapshot_texts():
        snap[_snap_key(t)] = exact.count(t)
    print(f"_EXACT_SNAPSHOT = {{  # {len(snap)} texts, {exact.name}")
    items = sorted(snap.items())
    for i in range(0, len(items), 4):
        print("    " + " ".join(f'"{k}": {v},' for k, v in items[i:i + 4]))
    print("}")
    chunks = {w: exact.count(w) for w in _snapshot_chunks()}
    print(f"_EXACT_CHUNKS = {{  # {len(chunks)} chunks, {exact.name}")
    line = "   "
    for k, v in chunks.items():
        item = f" {k!r}: {v},"
        if len(line) + len(item) > 88:
            print(line)
            line = "   "
        line += item
    print(line)
    print("}")
    return 0


# Exact OpenVLA token counts, frozen so the upper-bound property is checked
# where the tokenizer is NOT present (vla-drone, CI); before this, a table entry
# set too low could only be caught on the lab PC. Measured 2026-10-06 with
# openvla-llama2-sp@8f5e2869e180 (tokenizers 0.19.1, vla-real). Regenerate both
# with `python tests/test_csp.py --snapshot` where the tokenizer exists (the
# tokenizer test checks every entry against it).
#   _EXACT_SNAPSHOT - the texts _snapshot_texts() yields, keyed by the first 16
#                     hex digits of each text's SHA-256;
#   _EXACT_CHUNKS   - every table key and every chunk of those texts, by chunk.
_EXACT_SNAPSHOT = {  # 73 texts, openvla-llama2-sp@8f5e2869e180
    "00f0bc55f8e7e602": 32, "07dfe4c6b88b45fc": 93, "0d8e188fd6a44aaf": 28, "0e0b0f6dd4600c4b": 94,
    "0ff469db4fccc24d": 31, "10a0a73445aec881": 54, "1817e31911532b64": 106, "1b2e6233b5a077f3": 33,
    "1c47753acc805898": 12, "21e9c2c212492747": 14, "22364a7729029ba5": 12, "248267872c3323d3": 12,
    "2ddd5ceccd202711": 14, "307c9dad9c9089e9": 65, "31d09d892be6ca2b": 66, "355c71360c05a745": 97,
    "3cb3841c2e2d1e5f": 29, "42273de80c704f29": 26, "4ac481f20743839f": 66, "513e994913cc392f": 10,
    "51a1ed0309f7dea8": 77, "5da3b9f0218eade6": 12, "6034b88977c25e5c": 60, "63f083b961472f59": 27,
    "6437bbdc7e9a4b27": 60, "6bbc6d30dfac13d4": 82, "6e599eb9ecbb3d4b": 13, "7834a0637f580737": 12,
    "78f23d7fe5502e42": 31, "8571981278a49de6": 70, "88f5b6edad4deef0": 106, "8a6e0d4e8080d4d3": 10,
    "8c5e41a0f0d11bbf": 10, "8f2d55b21417ab3b": 15, "91d73077ee5d7874": 12, "94f3989888e0ab1e": 56,
    "96445328391204ca": 82, "9ade48c9f1ca145d": 10, "9be2e457976c1cea": 37, "a2c06ed23a592cbe": 15,
    "a34f9263b670f7ef": 13, "a4a7819f699866b3": 14, "a63cb1d0f2bd5688": 94, "a6f7e7b20d9cbe5e": 55,
    "ab781f859395a345": 15, "acf90447e164a468": 56, "aee5fcae31363176": 12, "af4349d8007afd8f": 15,
    "afa57e9bd3c64f48": 12, "b6d98424892678e9": 94, "c56fe1b9d5246335": 10, "cbc38f1d94571196": 104,
    "cbcb101aa522919b": 12, "d4351cd85dc3cbb3": 82, "d939981f3ca2beca": 82, "daf79727dbfcb192": 106,
    "ddadac0b29572443": 73, "de1de94b9893d501": 10, "deaeb5a9ad1a886d": 11, "df20041ab35a440c": 15,
    "df884d83495db743": 94, "e0b347f1e5d34cb3": 15, "e1b6aa1cacddb76d": 26, "e1ea555c78175cee": 12,
    "e421e618da0d16bc": 31, "e5b9ce15e13a07aa": 66, "e8dc1222a830346e": 84, "ea970a4920d30e17": 12,
    "ecdc28a3234b85b2": 56, "ed84f4c1b683687f": 14, "f24d5f0bdfca3751": 14, "f72a2141a3df8825": 10,
    "fcc856b9d97cec0f": 12,
}
_EXACT_CHUNKS = {  # 129 chunks, openvla-llama2-sp@8f5e2869e180
    "'corridor-survey-route',": 10, "'gui-nfz-1'.": 9, "'gui-nfz-2'.": 9,
    "'gui-nfz-3'.": 9, "'nfz-edge-1'.": 9, "'nfz-edge-2'.": 9, "'nfz-edge-3'.": 9,
    "'nfz-edge-4'.": 9, "'nfz-frontage'.": 8, "'nfz-leg-1'.": 9, "'nfz-leg-2'.": 9,
    "'nfz-leg-3'.": 9, "'nfz-leg-4'.": 9, "'nfz-leg-5'.": 9, "'nfz-partial'.": 7,
    "'nfz-residential-block'.": 11, "'nfz-route'.": 7, "'nfz-school'": 7,
    "'nfz-school-yard'": 9, "'nfz-square'.": 7, "'nfz-target'.": 7, "'tri-nfz'.": 7,
    '(Mon,': 3, '(Mon-Fri': 4, '(Mon-Fri)': 5, '(Sat,': 4, '(Sun': 3, '(Sun)': 4,
    '(Tue-Thu': 6, '(Tue-Thu)': 7, '(every': 2, '(in': 2, '(soft': 2, '(soft)': 3,
    '07:30-17:30).': 13, '1.5': 4, '10': 3, '12': 3, '14': 3, '15': 3, '17': 3, '18': 3,
    '2': 2, '20': 3, '25': 3, '28': 3, '3': 2, '30': 3, '35': 3, '4': 2, '45': 3,
    '5': 2, '55': 3, '6': 2, '60': 3, '80': 3, '<=': 1, '>=': 1, 'Fri': 1, 'Fri)': 2,
    'Fri).': 2, 'Keep': 1, 'Mon,': 2, 'Mon-Fri': 3, 'Mon-Fri)': 4, 'Mon-Fri).': 4,
    'Never': 1, 'Sat,': 2, 'Stay': 2, 'Sun': 1, 'Sun)': 2, 'Sun).': 2, 'Tue-Thu': 5,
    'Tue-Thu)': 6, 'Tue-Thu).': 6, 'Wed,': 2, 'altitude': 2, 'altitude.': 3, 'and': 1,
    'any': 1, 'anything': 1, 'are': 1, 'at': 1, 'away': 1, 'below': 1, 'between': 1,
    'building': 1, 'building.': 2, 'buildings': 1, 'centerline': 2, 'climb': 2,
    'corridor': 3, 'day': 1, 'deg/s': 3, 'deg/s.': 4, 'enter': 1, 'entry': 1,
    'every': 1, 'followed': 1, 'following': 1, 'following.': 2, 'force': 1, 'from': 1,
    'in': 1, 'inside': 1, 'into': 1, 'its': 1, 'least': 1, 'limit)': 2, 'limit).': 2,
    'm': 1, 'm.': 2, 'm/s': 3, 'm/s,': 4, 'no': 1, 'of': 1, 'or': 1, 'overnight)': 3,
    'overnight).': 3, 'pedestrian': 3, 'pedestrian.': 4, 'rate': 1, 'speed': 1,
    'stay': 1, 'subject': 1, 'turn': 1, 'within': 1, 'you': 1, 'zone': 1,
}


def test_the_table_counter_bounds_the_frozen_exact_counts_without_the_tokenizer():
    """The tokenizer-free half of the upper-bound check. A text whose hash is
    not in the snapshot (a changed template, a new fixture) is not checked,
    and is counted; if fewer than 75 % of the snapshot's texts are still
    produced, the snapshot is stale and the test fails rather than passing on
    whatever is left."""
    _need()
    table = C.TableTokenCounter()
    seen, unseen = set(), set()
    for t in _snapshot_texts():
        k = _snap_key(t)
        if k in _EXACT_SNAPSHOT:
            assert table.count(t) >= _EXACT_SNAPSHOT[k], (t, table.count(t), _EXACT_SNAPSHOT[k])
            seen.add(k)
        else:
            unseen.add(k)
    assert _EXACT_SNAPSHOT, "the snapshot is empty"
    assert len(seen) >= 0.75 * len(_EXACT_SNAPSHOT), (
        f"only {len(seen)} of {len(_EXACT_SNAPSHOT)} snapshot texts are still produced: "
        f"regenerate with `python tests/test_csp.py --snapshot` (needs the tokenizer)")
    print(f"      {len(seen)} texts checked against frozen exact counts; "
          f"{len(unseen)} produced texts not in the snapshot")


def test_no_table_entry_is_below_its_frozen_exact_count():
    """The chunk-level bound, tokenizer-free. Every table entry must have been
    measured (be in _EXACT_CHUNKS) and be at or above that measurement, and
    every chunk the fixtures produce - ids and numbers through the byte bound
    included - must count at or above its own."""
    _need()
    table = C.TableTokenCounter()
    unmeasured = sorted(k for k in C._OPENVLA_TABLE if k not in _EXACT_CHUNKS)
    assert not unmeasured, f"table entries never measured: {unmeasured[:5]}"
    low = [(k, v, _EXACT_CHUNKS[k]) for k, v in C._OPENVLA_TABLE.items()
           if v < _EXACT_CHUNKS[k]]
    assert not low, f"table entries below the exact count (chunk, table, exact): {low}"
    n = 0
    for t in _snapshot_texts():
        for w in t.split(" "):
            if w in _EXACT_CHUNKS:
                assert table.count(w) >= _EXACT_CHUNKS[w], (w, table.count(w), _EXACT_CHUNKS[w])
                n += 1
    assert n > 0, "no fixture chunk was checked"
    print(f"      {len(C._OPENVLA_TABLE)} table entries and {n} fixture chunks "
          f"at or above their frozen exact counts")


def test_the_table_counter_bounds_unknown_text_by_its_bytes():
    _need()
    t = C.TableTokenCounter()
    assert t.count("zz-qq-17") == len("zz-qq-17") + 1
    assert t.count("\u00e9t\u00e9") == len("\u00e9t\u00e9".encode("utf-8")) + 1
    assert t.count("") == 0


# --------------------------------------------------------------------------- #
# KPI 2 (WP2-03): P0 coverage, and nothing vanishes
# --------------------------------------------------------------------------- #

def test_the_coverage_function_can_say_zero():
    """The null: a coverage scorer that reads 100 % on an empty CSP is not one."""
    _need()
    pol = load_policy(PED)
    csp = ConstraintCompiler(pol).compile_csp(_mission(), now=None)
    empty = csp.model_copy(update={"P0_constraints": [], "natural_language_prompt": ""})
    got = K.p0_coverage(empty, pol)
    assert got["coverage"] == 0.0, got
    # Each half of the definition on its own. A P0 rule is covered only if the
    # structured record AND the text both carry it, so emptying either one
    # must also read 0 % - otherwise one half of the scorer is decoration.
    assert K.p0_coverage(csp, pol)["coverage"] == 1.0
    no_struct = csp.model_copy(update={"P0_constraints": []})
    assert K.p0_coverage(no_struct, pol)["coverage"] == 0.0, "the scorer ignores P0_constraints"
    no_text = csp.model_copy(update={"natural_language_prompt": ""})
    assert K.p0_coverage(no_text, pol)["coverage"] == 0.0, "the scorer ignores the prompt text"
    none = Policy.model_validate({"policy_id": "p1-only", "constraints": [
        {"id": "k", "type": "kinematic_envelope", "priority": "P1",
         "speed_max_mps": 3, "climb_rate_max_mps": 1, "yaw_rate_max_dps": 30}]})
    got = K.p0_coverage(ConstraintCompiler(none).compile_csp(_mission(), now=None), none)
    assert got["coverage"] is None and got["n_p0_in_scope"] == 0, \
        "no P0 rules is 'undefined', not 100 % and not 0 %"


def test_kpi_p0_coverage_is_100_percent_across_every_fixture():
    """KPI: 'P0 coverage = 100% across the test fixtures.'

    Strict reading: no clock and no region filter, so EVERY P0 rule in every
    policy file is in scope, and each must be both in P0_constraints and in
    the text the model reads."""
    _need()
    n = covered = 0
    for path in POLICIES:
        pol = load_policy(path)
        csp = ConstraintCompiler(pol).compile_csp(_mission(), now=None,
                                                  region_margin_m=None)
        got = K.p0_coverage(csp, pol)
        n_p0 = sum(1 for c in pol.constraints if c.priority == "P0")
        assert got["n_p0_in_scope"] == n_p0, (path.name, got)
        assert got["coverage"] == 1.0, (path.name, got)
        n += got["n_p0_in_scope"]
        covered += got["n_p0_covered"]
    assert n > 0
    print(f"      P0 coverage {covered}/{n} = {covered / n:.1%} over "
          f"{len(POLICIES)} fixtures; null (empty CSP) = 0.0%")


def test_rule_coverage_shows_p1_rules_cut_for_budget_and_keeps_them_recorded():
    """Audit WP2-03 asked for coverage of ALL rules, text and structured. The
    budget may cut P1/P2 text (PDF p4); the cut must show as text < 100 %
    while the record still carries every rule."""
    _need()
    pol, m = _mix(), _mission(**MIX_MISSION)
    _, p0, rest = _order_without_budget(pol, m)
    budget = C.TableTokenCounter().count(" ".join(r.sentence for r in p0 + rest[:2]))
    csp = ConstraintCompiler(pol).compile_csp(m, now=None, budget_tokens=budget)
    got = K.rule_coverage(csp, pol)
    assert got["P0"]["text"] == 1.0 and got["P0"]["recorded"] == 1.0, got["P0"]
    n_rest = len(rest)
    assert got["all"]["n_in_text"] == len(p0) + 2, got["all"]
    assert got["all"]["n_in_scope"] == len(p0) + n_rest, got["all"]
    assert got["all"]["recorded"] == 1.0, got["all"]
    assert got["P2"]["text"] == 0.0 and got["P2"]["n_in_scope"] == 2, got["P2"]
    # the null: the same scorer on an emptied CSP says 0, and a priority with
    # no rule in scope says None
    empty = csp.model_copy(update={"natural_language_prompt": "",
                                   "relevance_explanations": {}})
    z = K.rule_coverage(empty, pol)["all"]
    assert z["text"] == 0.0 and z["recorded"] == 0.0, z
    ped = load_policy(PED)
    assert K.rule_coverage(ConstraintCompiler(ped).compile_csp(_mission(), now=None),
                           ped)["P2"]["text"] is None


def _report_json(budget):
    import contextlib
    import io
    out = Path(tempfile.mkdtemp()) / "csp_kpis.json"
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = K._main(["report", "--policies", str(ROOT / "policies"),
                      "--budget", str(budget), "--json", str(out)])
    assert rc == 0, buf.getvalue()[-2000:]
    return json.loads(out.read_text(encoding="utf-8")), buf.getvalue()


def test_the_kpi_report_measures_its_null_and_traces_every_number():
    """WP2-02/03: one command computes coverage and token use over every
    fixture, writes it with the policy hashes, the code revision and the
    command that reproduces it (CONTRIBUTING.md item 5), and measures its null
    with the same scorer rather than printing a constant."""
    _need()
    rep, text = _report_json(K.DEFAULT_BUDGET_TOKENS)
    t = rep["totals"]
    assert t["n_policies"] == len(POLICIES) == t["n_compiled"], t
    assert t["p0_coverage"] == 1.0 and t["null_p0_coverage"] == 0.0, t
    assert t["n_p0_in_scope"] == sum(
        1 for p in POLICIES for c in load_policy(p).constraints if c.priority == "P0")
    assert t["n_rules_explained"] == t["n_rules_in_csp"] > 0, t
    assert t["max_tokens_used"] <= rep["budget_tokens"], t
    by_file = {r["file"]: r for r in rep["rows"]}
    for p in POLICIES:
        row = by_file[p.relative_to(ROOT).as_posix()]
        assert row["policy_hash"] == load_policy(p).policy_hash, p.name
    assert rep["command"].startswith("python -m guardrail.compiler report"), rep["command"]
    assert rep["code_revision"], rep
    assert "reproduce: " in text
    # the no-skill baseline is reported beside the KPI, and says whether the
    # KPI told skill from none at this budget
    assert t["baseline_n_p0_covered"] <= t["n_p0_covered"], t
    assert t["kpi_discriminates"] == (t["baseline_n_p0_covered"] < t["n_p0_covered"]), t
    assert "baseline (old by-type text" in text
    assert all("csp_content_hash" in r and "csp_hash" not in r for r in rep["rows"]), \
        "report rows must be keyed by the wall-clock-free hash"
    print(f"      baseline {t['baseline_n_p0_covered']}/{t['n_p0_in_scope']} at "
          f"{rep['budget_tokens']} tokens; kpi_discriminates={t['kpi_discriminates']}")


def test_the_naive_baseline_loses_p0_rules_the_compiler_keeps():
    """KPI 2 against a no-skill compiler, at the budget where it matters. The
    old by-type text puts the three zones first, so cut to the P0 budget it
    runs out before the P0 altitude band; the compiler keeps both P0 rules.
    At a budget everything fits, the two agree - which is why the KPI alone
    cannot show skill at the 256-token default."""
    _need()
    pol, m = _mix(), _mission(**MIX_MISSION)
    _, p0, _ = _order_without_budget(pol, m)
    tight = C.TableTokenCounter().count(" ".join(r.sentence for r in p0))
    naive = K.naive_prefix_baseline(pol, tight)
    ours = K.p0_coverage(ConstraintCompiler(pol).compile_csp(m, now=None,
                                                             budget_tokens=tight), pol)
    assert ours["coverage"] == 1.0, ours
    assert naive["coverage"] == 0.5 and naive["missing"] == ["alt-band"], naive
    assert naive["tokens_used"] <= tight, naive
    loose = K.naive_prefix_baseline(pol, 100000)
    assert loose["coverage"] == 1.0 and loose["n_rules_in_text"] == len(pol.constraints)
    # ...and the report says which of the two cases it is in
    import yaml
    d = Path(tempfile.mkdtemp())
    (d / "mix.yaml").write_text(yaml.safe_dump(MIX, sort_keys=False), encoding="utf-8")
    at_tight = K.coverage_report(d, budget_tokens=tight)["totals"]
    assert (at_tight["n_p0_covered"], at_tight["baseline_n_p0_covered"]) == (2, 1), at_tight
    assert at_tight["kpi_discriminates"] is True, at_tight
    at_loose = K.coverage_report(d, budget_tokens=100000)["totals"]
    assert at_loose["kpi_discriminates"] is False, at_loose


def test_the_kpi_report_never_reads_100_percent_when_nothing_compiled():
    """At a budget no P0 set fits, every policy raises CSPBudgetExceeded (KPI 3).
    Then nothing was measured: coverage must be None, not 100 % and not 0 %,
    and every raised file must be listed with its P0 token need."""
    _need()
    rep, text = _report_json(5)
    t = rep["totals"]
    assert t["n_compiled"] == 0 and t["n_raised_budget_exceeded"] == len(POLICIES), t
    assert t["p0_coverage"] is None and t["rule_text_coverage"] is None, t
    assert t["baseline_p0_coverage"] is None and t["kpi_discriminates"] is None, t
    assert all(r["p0_tokens"] > 5 for r in rep["raised"]), rep["raised"][:2]
    assert "CSPBudgetExceeded" in text
    # where the compiler raises, the naive cut would have carried nothing and
    # said nothing: the report counts it
    assert t["raised_baseline_n_p0"] == sum(
        1 for p in POLICIES for c in load_policy(p).constraints if c.priority == "P0")
    assert t["raised_baseline_n_p0_covered"] == 0, t


def test_every_rule_is_accounted_for_on_every_fixture():
    """kept + dropped + filtered = all rules, by id and in policy order."""
    _need()
    ok = {"kept", "dropped_budget", "filtered_inactive", "filtered_out_of_region"}
    for path in POLICIES:
        pol = load_policy(path)
        for now in (None, MONDAY_0800, SATURDAY_1000):
            csp = ConstraintCompiler(pol).compile_csp(_mission(), now=now)
            assert [r.id for r in csp.selection.rules] == [c.id for c in pol.constraints]
            assert all(r.decision in ok for r in csp.selection.rules)
            for r in csp.selection.rules:
                assert r.reason.strip(), (path.name, r)


# --------------------------------------------------------------------------- #
# KPI 4 (WP2-06): relevance explanations
# --------------------------------------------------------------------------- #

def test_kpi_every_rule_in_the_csp_has_a_relevance_explanation():
    """KPI: 'relevance_explanations[rule_id] is populated for every rule in the CSP.'"""
    _need()
    n = 0
    for path in POLICIES:
        csp = ConstraintCompiler(load_policy(path)).compile_csp(_mission(), now=MONDAY_0800)
        in_csp = {r.id for r in csp.selection.rules
                  if r.decision in ("kept", "dropped_budget")}
        assert set(csp.relevance_explanations) == in_csp, path.name
        for rid, why in csp.relevance_explanations.items():
            assert why.strip(), (path.name, rid)
        n += len(in_csp)
    print(f"      {n} rules explained across {len(POLICIES)} fixtures")


def test_explanations_change_with_the_mission_so_a_constant_would_fail():
    _need()
    pol = load_policy(DEMO)
    fence = next(c.id for c in pol.constraints if c.type == "polygon_fence")
    alt = next(c.id for c in pol.constraints if c.type == "altitude_envelope")
    a = ConstraintCompiler(pol).compile_csp(_mission(30, 30, alt=15), now=None)
    b = ConstraintCompiler(pol).compile_csp(_mission(-60, 10, alt=45), now=None)
    assert a.relevance_explanations[fence] != b.relevance_explanations[fence]
    assert "above" in b.relevance_explanations[alt], b.relevance_explanations[alt]
    assert "Shield will repair" in b.relevance_explanations[alt]
    kin = next(c.id for c in pol.constraints if c.type == "kinematic_envelope")
    fast = ConstraintCompiler(pol).compile_csp(_mission(speed=50.0), now=None)
    assert "above" in fast.relevance_explanations[kin], fast.relevance_explanations[kin]


def test_corridor_clearance_and_standoff_explanations_change_with_the_mission():
    """The other three rule types. Without this, an explainer that always said
    'start inside' or 'asks to follow' passed (2026-10-06 review: both
    mutants survived), so the mission-dependence claim held for three of six
    rule types only."""
    _need()
    survey = ConstraintCompiler(load_policy(SURVEY))
    inside = survey.compile_csp(_mission(30, 0, alt=15), now=None)
    outside = survey.compile_csp(_mission(30, 0, alt=15, sx=0, sy=100), now=None)
    rid = "corridor-survey-route"
    assert "start is inside" in inside.relevance_explanations[rid], inside.relevance_explanations[rid]
    assert "start is outside" in outside.relevance_explanations[rid], outside.relevance_explanations[rid]
    high = survey.compile_csp(_mission(30, 0, alt=45), now=None)
    assert "45 m is outside its 10-20 m band" in high.relevance_explanations[rid]
    mix = ConstraintCompiler(_mix())
    short = mix.compile_csp(_mission(30, 40), now=None)
    long_ = mix.compile_csp(_mission(300, 400), now=None)
    assert "whole 50 m path" in short.relevance_explanations["keep-off-buildings"]
    assert "whole 500 m path" in long_.relevance_explanations["keep-off-buildings"]
    follow = mix.compile_csp(_mission(text="follow the person to (30, 30)"), now=None)
    fly = mix.compile_csp(_mission(text="fly to (30, 30)"), now=None)
    assert "asks to follow" in follow.relevance_explanations["standoff-any"]
    assert "does not ask to follow" in fly.relevance_explanations["standoff-any"]


# --------------------------------------------------------------------------- #
# WP2-08: Jinja2 templates, and backward compatibility
# --------------------------------------------------------------------------- #

def test_every_rule_type_has_its_jinja2_templates():
    """Every type the Shield ENFORCES (models.RUNTIME_TYPES) resolves to a
    template of each kind, its own or (for a subclass such as circle_fence)
    its base type's. A type with none would make compile_csp raise on any
    policy that uses it. The declarable-only types are refused by name before
    rendering (next test), so they need none; a type moved into RUNTIME_TYPES
    without templates fails here."""
    _need()
    from guardrail.models import RUNTIME_TYPES, Constraint
    missing, runtime = [], set()
    for cls in Constraint.__origin__.__args__:
        name = cls.model_fields["type"].annotation.__args__[0]
        if name not in RUNTIME_TYPES:
            continue
        runtime.add(name)
        for kind in ("sentence", "summary", "reason"):
            t = K.template_type(kind, cls)
            if not (K.TEMPLATE_DIR / kind / f"{t}.j2").is_file():
                missing.append(f"{kind}/{t}.j2")
    assert runtime == set(RUNTIME_TYPES), (runtime, RUNTIME_TYPES)
    assert not missing, f"rule types the compiler cannot render: {missing}"


def test_a_rule_the_shield_does_not_enforce_is_refused_by_name():
    """Review, 2026-10-06: a policy declaring distance_envelope, dynamic_nfz,
    time_window_switch or corridor_swap made compile_csp raise a bare
    TypeError from the relevance step (marked "pragma: no cover") or
    TemplateNotFound. Telling the model an unenforced rule over-promises;
    dropping it hides a hard rule. The compiler now refuses the policy with
    CSPUnenforcedRules naming every such rule, as the flight loaders do.

    Since 2026-10-08 the three hot-apply types render (they are enforced, and
    have templates) and only distance_envelope is left - plus an MSL height,
    named the same way: a Policy built directly reached the compiler without
    the loaders' MSL refusal, and compiled."""
    _need()
    from guardrail.csp import CSPUnenforcedRules
    from guardrail.models import GRANT_TYPES, RUNTIME_TYPES
    assert set(GRANT_TYPES) - RUNTIME_TYPES == {"distance_envelope"}, \
        "a type joined or left RUNTIME_TYPES: update this test"
    base = [{"id": "fence", "type": "polygon_fence",
             "vertices": [{"x": 10, "y": 10}, {"x": 20, "y": 10}, {"x": 20, "y": 20}],
             "altitude_floor_m": 0, "altitude_ceiling_m": 100}]
    cases = {
        "distance_envelope": ({"id": "r-de", "type": "distance_envelope",
                               "object_class": "people", "min_distance_m": 5.0},
                              {"r-de": "distance_envelope"}),
        "MSL": ({"id": "r-msl", "type": "altitude_envelope", "alt_min_m": 5,
                 "alt_max_m": 90, "altitude_ref": "MSL"},
                {"r-msl": "altitude_envelope with altitude_ref MSL"}),
    }
    for name, (rule, want) in cases.items():
        pol = Policy.model_validate({"policy_id": f"p-{name}", "constraints": base + [rule]})
        try:
            ConstraintCompiler(pol).compile_csp(_mission(), now=None)
        except CSPUnenforcedRules as e:
            assert e.rules == want, e.rules
            rid, what = next(iter(want.items()))
            assert f"{rid} ({what})" in str(e) and "unenforced_rules" in str(e), str(e)
        else:
            raise AssertionError(f"{name}: compile_csp accepted a rule the Shield does "
                                 f"not enforce")
        assert pol.unenforced_rules(), "the flight loaders' check disagrees"
    # not a ValueError: a caller's generic `except ValueError` must not swallow it
    assert not issubclass(CSPUnenforcedRules, ValueError)


def test_the_kpi_report_lists_a_file_with_an_unenforced_rule_and_carries_on():
    """2026-10-07: one policy file declaring a declarable-only rule type in the
    folder stopped coverage_report outright (load_policy refuses it for
    flight). It is now listed by name with its reasons, left out of every
    share, and the other files are scored exactly as without it."""
    _need()
    import yaml
    d = Path(tempfile.mkdtemp())
    (d / "mix.yaml").write_text(yaml.safe_dump(MIX, sort_keys=False), encoding="utf-8")
    alone = K.coverage_report(d)["totals"]
    # A time_window_switch was the example until 2026-10-08, when it became
    # flyable; distance_envelope is the one declarable-only type left.
    people = {"policy_id": "p-people", "version": "0.1.0", "constraints": [
        {"id": "fence", "type": "polygon_fence", "priority": "P0",
         "vertices": [{"x": 10, "y": 10}, {"x": 20, "y": 10}, {"x": 20, "y": 20}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 100},
        {"id": "people", "type": "distance_envelope", "object_class": "people",
         "min_distance_m": 5.0}]}
    (d / "people.yaml").write_text(yaml.safe_dump(people, sort_keys=False), encoding="utf-8")
    rep = K.coverage_report(d)
    t = rep["totals"]
    assert t["n_policies"] == 2 and t["n_compiled"] == 1, t
    assert t["n_not_flyable"] == 1, t
    bad = rep["not_flyable"][0]
    assert bad["policy_id"] == "p-people" and len(bad["problems"]) == 1, bad
    assert bad["problems"][0].startswith("people: distance_envelope"), bad
    for k in ("n_p0_in_scope", "n_p0_covered", "p0_coverage", "baseline_n_p0_covered"):
        assert t[k] == alone[k], (k, t[k], alone[k])


def test_a_circle_fence_compiles_as_the_keep_out_zone_it_is():
    """CircleFence subclasses PolygonFence (its vertices are the derived
    32-gon). Until 2026-10-06 the template lookup used `rule.type` alone, so a
    policy with one circle could not compile a CSP (TemplateNotFound) while the
    Shield enforced it. Now it renders, is covered, and is forbidden.

    Since 2026-10-08 it has its own templates (the grant's "Centre + radius"),
    so the text says it is a circle and how big; it read as a bare polygon
    zone before. No shipped policy has a circle, so no stored text moved."""
    _need()
    from guardrail.models import CircleFence
    pol = Policy.model_validate({"policy_id": "circle", "constraints": [
        {"id": "disc", "type": "circle_fence", "center": {"x": 15.0, "y": 15.0},
         "radius_m": 4.0, "altitude_floor_m": 0, "altitude_ceiling_m": 100}]})
    rule = pol.constraints[0]
    assert isinstance(rule, CircleFence), type(rule)
    for kind in ("sentence", "summary", "reason"):
        assert K.template_type(kind, rule) == "circle_fence", kind
    assert not any(isinstance(c, CircleFence) for p in POLICIES
                   for c in load_policy(p).constraints), "a fixture has a circle now"
    csp = ConstraintCompiler(pol).compile_csp(_mission(), now=None)
    assert "Never enter zone 'disc', a circle of radius 4 m." in csp.natural_language_prompt, \
        csp.natural_language_prompt
    assert K.render_summary(rule) == "no entry into 'disc' (radius 4 m)"
    assert "circular zone (radius 4 m)" in csp.relevance_explanations["disc"]
    assert K.p0_coverage(csp, pol)["coverage"] == 1.0
    assert "disc" in csp.forbidden_action_set.no_translate_into_polygons
    st = State(x=15.0, y=8.0, up=15.0)            # 7 m south of the centre
    into = K.action_violations(Action4D(vy=0.0, vx=0.0), csp, state=st, policy=pol)
    assert "polygon:disc" not in into, into
    hit = K.action_violations(Action4D(vy=3.5), csp, state=st, policy=pol, dt=1.0)
    assert "polygon:disc" in hit, hit
    assert csp.relevance_explanations.get("disc"), "no reason rendered for the circle"


def test_sentences_without_a_window_are_byte_identical_to_the_old_ones():
    """Every HARD fixture rule with no valid_time renders exactly the
    2026-09-01 text, so `build_prompt` reproduces for every flight that read
    it. Windowed and soft rules get a qualifier before the full stop."""
    n_same = n_win = n_soft = 0
    for path in POLICIES:
        pol = load_policy(path)
        new = ConstraintCompiler(pol)._sentences()
        old = _legacy_sentences(pol)
        assert len(new) == len(old), path.name
        for c_new, c_old, rule in zip(new, old, _ordered(pol)):
            if rule.valid_time is None and rule.constraint_type == "hard":
                assert c_new == c_old, (path.name, c_new, c_old)
                n_same += 1
            else:
                assert c_new.startswith(c_old[:-1] + " ("), (c_new, c_old)
                n_win += rule.valid_time is not None
                n_soft += rule.constraint_type == "soft"
    assert n_same > 100, n_same
    print(f"      {n_same} hard window-less sentences byte-identical; "
          f"{n_win} windowed, {n_soft} soft carry a qualifier")


def test_a_soft_rule_says_so_which_the_old_text_did_not():
    """A deliberate change, pinned so it stays deliberate: the 2026-09-01 text
    gave a soft rule the same 'Never enter zone' as a hard one. No shipped
    fixture has a soft rule, so build_prompt changed for none of them; it
    does change for any soft-rule policy."""
    _need()
    pol = Policy.model_validate({"policy_id": "soft", "constraints": [
        {"id": "z", "type": "polygon_fence", "constraint_type": "soft",
         "vertices": _sq(0, 0, 1, 1)},
        {"id": "k", "type": "kinematic_envelope", "constraint_type": "soft",
         "speed_max_mps": 3, "climb_rate_max_mps": 1, "yaw_rate_max_dps": 30}]})
    z, k = pol.constraints
    assert K.render_sentence(z) == "Never enter zone 'z' (soft limit)."
    assert K.render_sentence(k) == ("Keep speed at or below 3 m/s, climb rate below "
                                    "1 m/s and turn rate below 30 deg/s (soft limit).")
    assert _legacy_sentences(pol)[0] == "Never enter zone 'z'."
    assert K.render_summary(z) == "no entry into 'z' (soft)"
    n_soft = sum(1 for p in POLICIES for c in load_policy(p).constraints
                 if c.constraint_type == "soft")
    print(f"      soft rules in the {len(POLICIES)} fixtures: {n_soft} "
          f"(build_prompt text changed for each one)")


def _ordered(pol):
    from guardrail.models import (AltitudeEnvelope, Corridor, KinematicEnvelope,
                                  ObstacleClearance, PolygonFence,
                                  SubjectStandoff)
    out = []
    for cls in (PolygonFence, Corridor, AltitudeEnvelope, KinematicEnvelope,
                ObstacleClearance, SubjectStandoff):
        out += pol.by_type(cls)
    return out


def test_build_prompt_keeps_its_keys():
    import yaml
    c = ConstraintCompiler(load_policy(DEMO))
    doc = yaml.safe_load(c.build_prompt(c.parse_command("go to (30, 30) altitude 15")))
    assert list(doc) == ["mission", "constraints", "natural_language_prompt"], list(doc)
    assert list(doc["mission"]) == ["task", "target", "cruise_alt_m"], doc["mission"]
    assert list(doc["constraints"]) == ["policy_id", "policy_hash", "no_fly_zones",
                                        "altitude_band_m", "speed_max_mps",
                                        "summary_pack"], list(doc["constraints"])
    assert doc["natural_language_prompt"] == " ".join(
        _legacy_sentences(load_policy(DEMO)))


def test_a_template_with_a_missing_variable_raises_instead_of_printing_blank():
    """StrictUndefined: 'Keep at least  m away' is a silent failure."""
    _need()
    try:
        K.render_template("reason/polygon_fence.j2", {})
    except Exception as e:                          # noqa: BLE001
        assert "undefined" in type(e).__name__.lower() or "undefined" in str(e).lower(), e
        return
    raise AssertionError("a template rendered with no variables")


# --------------------------------------------------------------------------- #
# WP2-16: multi-scale geometry and geometry_ref
# --------------------------------------------------------------------------- #

def test_coarse_geometry_always_contains_the_fine_polygon():
    """A coarse keep-out zone that is SMALLER than the real one would tell the
    model a smaller zone than the Shield enforces. Checked with shapely here,
    independently of the pure-Python hull in the compiler."""
    _need()
    from shapely.geometry import Polygon as SP
    ring12 = [{"x": 50 + 20 * math.cos(2 * math.pi * k / 12),
               "y": 20 * math.sin(2 * math.pi * k / 12)} for k in range(12)]
    ell = [{"x": 0, "y": 0}, {"x": 30, "y": 0}, {"x": 30, "y": 10},
           {"x": 10, "y": 10}, {"x": 10, "y": 30}, {"x": 0, "y": 30}]
    extra = Policy.model_validate({"policy_id": "shapes", "constraints": [
        {"id": "ring12", "type": "polygon_fence", "vertices": ring12},
        {"id": "ell", "type": "polygon_fence", "vertices": ell}]})
    n = 0
    for pol in [load_policy(p) for p in POLICIES] + [extra]:
        geo = K.multiscale_geometry(pol)
        for c in pol.constraints:
            if c.type != "polygon_fence":
                continue
            fine, coarse = SP(geo[c.id]["fine"]), SP(geo[c.id]["coarse"])
            assert coarse.buffer(1e-6).covers(fine), (pol.policy_id, c.id)
            assert len(geo[c.id]["coarse"]) <= K.COARSE_MAX_VERTICES, c.id
            n += 1
    assert len(K.multiscale_geometry(extra)["ring12"]["coarse"]) == 4
    assert n > 20, n


def test_geometry_ref_resolves_and_refuses_a_stale_generation():
    _need()
    pol = load_policy(SURVEY)
    csp = ConstraintCompiler(pol).compile_csp(_mission(), now=MONDAY_0800)
    p0 = {p.id: p for p in csp.P0_constraints}
    ref = p0["nfz-school"].geometry_ref
    assert ref in ("coarse:nfz-school@v0.1.0/g0", "fine:nfz-school@v0.1.0/g0"), ref
    pts = K.resolve_geometry_ref(pol, ref)
    assert len(pts) >= 3
    assert p0["corridor-survey-route"].geometry_ref.startswith("fine:"), \
        "a keep-in corridor must never be coarsened outward"
    bumped = pol.model_copy(update={"generation": 1})
    try:
        K.resolve_geometry_ref(bumped, ref)
    except ValueError:
        return
    raise AssertionError("a g0 reference resolved against a generation-1 policy")


def test_scale_is_fine_near_the_path_and_coarse_far_from_it():
    _need()
    csp = ConstraintCompiler(_mix()).compile_csp(_mission(**MIX_MISSION), now=None,
                                                 budget_tokens=1000)
    refs = {p.id: p.geometry_ref for p in csp.P0_constraints}
    assert refs["nfz-on-path"].startswith("fine:"), refs
    assert refs["alt-band"].startswith("param:"), refs
    far = Policy.model_validate({"policy_id": "far-p0", "constraints": [
        {"id": "nfz-far", "type": "polygon_fence", "vertices": _sq(30, 150, 40, 160)}]})
    csp2 = ConstraintCompiler(far).compile_csp(_mission(**MIX_MISSION), now=None)
    assert csp2.P0_constraints[0].geometry_ref.startswith("coarse:"), csp2.P0_constraints


# --------------------------------------------------------------------------- #
# WP2-07: the action-set adapter, onto Action4D
# --------------------------------------------------------------------------- #

def test_the_action_set_matches_the_policy_caps_in_the_spec_shape():
    _need()
    csp = ConstraintCompiler(load_policy(PED)).compile_csp(_mission(), now=None)
    a = csp.allowed_action_set
    assert a.frame == "body"
    assert list(a.vx_range_mps) == [-4.0, 4.0] and list(a.vy_range_mps) == [-4.0, 4.0]
    assert list(a.vz_range_mps) == [-2.0, 2.0]
    assert a.yaw_rate_max_dps == 45.0 and a.horizontal_speed_max_mps == 4.0
    assert list(a.altitude_band_m_agl) == [10.0, 20.0]
    flat = K.action_set_adapter(csp)
    for k in ("vx_range_mps", "vy_range_mps", "vz_range_mps", "yaw_rate_max_dps",
              "altitude_band_m_agl", "no_translate_into_polygons"):
        assert k in flat, k
    d = ConstraintCompiler(load_policy(DEMO)).compile_csp(_mission(), now=None)
    fences = [c.id for c in load_policy(DEMO).constraints if c.type == "polygon_fence"]
    assert d.forbidden_action_set.no_translate_into_polygons == fences


def test_a_missing_cap_is_none_never_zero():
    """A zero speed range would read as 'hover only'; no rule means no bound."""
    _need()
    pol = Policy.model_validate({"policy_id": "fence-only", "constraints": [
        {"id": "z", "type": "polygon_fence", "vertices": _sq(10, 10, 20, 20)}]})
    a = ConstraintCompiler(pol).compile_csp(_mission(), now=None).allowed_action_set
    assert a.vx_range_mps is None and a.vz_range_mps is None, a
    assert a.yaw_rate_max_dps is None and a.altitude_band_m_agl is None, a


def test_a_corridor_altitude_band_bounds_the_action_set():
    """REVIEW FIX (major). corridor_survey has a P0 corridor at 10-20 m and no
    altitude envelope. The Shield holds 10-20 m everywhere (CorridorAltitudeFix),
    but the action set only intersected envelopes and reported no band -
    'unbounded' by AllowedActionSet's own definition."""
    _need()
    csp = ConstraintCompiler(load_policy(SURVEY)).compile_csp(_mission(), now=None)
    a = csp.allowed_action_set
    assert a.altitude_band_m_agl == (10.0, 20.0), a
    assert "corridor-survey-route" in a.source_rules, a.source_rules
    v = K.action_violations(Action4D(vz_up=5.0), csp, state=State(x=0, y=0, up=19.0))
    assert "altitude" in v, v
    # The spec's worked example (prefix-compiler.md): a 5-120 m envelope and a
    # 30-80 m corridor give altitude_band_m_agl [30, 80].
    river = {"id": "corridor-river-east", "type": "corridor", "altitude_floor_m": 30,
             "altitude_ceiling_m": 80, "width_m": 40,
             "centerline": [{"x": 0, "y": 0}, {"x": 100, "y": 0}]}
    spec = Policy.model_validate({"policy_id": "spec-example", "constraints": [
        river,
        {"id": "alt", "type": "altitude_envelope", "priority": "P1",
         "alt_min_m": 5, "alt_max_m": 120}]})
    band = ConstraintCompiler(spec).compile_csp(_mission(), now=None).allowed_action_set
    assert band.altitude_band_m_agl == (30.0, 80.0), band
    assert set(band.source_rules) == {"corridor-river-east", "alt"}, band.source_rules
    # bands that cannot all hold are a policy conflict, said out loud
    clash = Policy.model_validate({"policy_id": "clash", "constraints": [
        river,
        {"id": "low", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 20}]})
    try:
        ConstraintCompiler(clash).compile_csp(_mission(), now=None)
    except ValueError as e:
        assert "corridor-river-east" in str(e) and "low" in str(e), e
    else:
        raise AssertionError("a 30-80 m corridor and a 5-20 m envelope compiled")


def test_several_envelopes_intersect_to_the_tightest_bounds():
    """Every hard envelope binds at once, so the action set is the
    intersection: the smallest of each cap and the overlap of the bands."""
    _need()
    pol = Policy.model_validate({"policy_id": "two-each", "constraints": [
        {"id": "k1", "type": "kinematic_envelope", "speed_max_mps": 5,
         "climb_rate_max_mps": 3, "yaw_rate_max_dps": 30},
        {"id": "k2", "type": "kinematic_envelope", "speed_max_mps": 8,
         "climb_rate_max_mps": 1, "yaw_rate_max_dps": 60},
        {"id": "a1", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 50},
        {"id": "a2", "type": "altitude_envelope", "alt_min_m": 10, "alt_max_m": 80}]})
    a = ConstraintCompiler(pol).compile_csp(_mission(), now=None).allowed_action_set
    assert a.vx_range_mps == (-5.0, 5.0) and a.horizontal_speed_max_mps == 5.0, a
    assert a.vz_range_mps == (-1.0, 1.0) and a.yaw_rate_max_dps == 30.0, a
    assert a.altitude_band_m_agl == (10.0, 50.0), a


def test_action_violations_flag_leaving_a_keep_in_corridor():
    """The corridor half of the forbidden set: a step that ends outside the
    half-width is flagged; the same step from nearer the centerline is not."""
    _need()
    pol = load_policy(SURVEY)
    csp = ConstraintCompiler(pol).compile_csp(_mission(), now=None)
    step = Action4D(vy=4.0)
    out = K.action_violations(step, csp, state=State(x=30, y=18, up=15), policy=pol)
    assert "corridor:corridor-survey-route" in out, out
    ok = K.action_violations(step, csp, state=State(x=30, y=10, up=15), policy=pol)
    assert not any(v.startswith("corridor:") for v in ok), ok


def test_action_violations_convert_the_yaw_cap_to_the_contract_units():
    """Action4D.yaw_rate is rad/s; the CSP cap is deg/s. Comparing them raw
    would admit anything below 45 rad/s (2578 deg/s) - the same unit confusion
    as docs/FINDING-the-contract-disagreed-with-itself-about-yaw.md."""
    _need()
    csp = ConstraintCompiler(load_policy(PED)).compile_csp(_mission(), now=None)
    ok = Action4D(yaw_rate=math.radians(44.0))
    assert K.action_violations(ok, csp) == []
    assert "yaw_rate" in K.action_violations(Action4D(yaw_rate=math.radians(46.0)), csp)
    assert "yaw_rate" in K.action_violations(Action4D(yaw_rate=1.0), csp)
    assert "speed" in K.action_violations(Action4D(vx=3.5, vy=3.5), csp), \
        "inside the vx/vy box but 4.95 m/s against a 4 m/s disc"
    assert "vz" in K.action_violations(Action4D(vz_up=2.5), csp)
    st = State(x=0, y=0, up=19.5)
    assert "altitude" in K.action_violations(Action4D(vz_up=1.0), csp, state=st, dt=1.0)


def test_action_violations_predict_entry_into_a_forbidden_polygon():
    _need()
    pol = _mix()
    csp = ConstraintCompiler(pol).compile_csp(_mission(**MIX_MISSION), now=None)
    st = State(x=37.0, y=0.0, up=15.0)
    bad = K.action_violations(Action4D(vx=4.0), csp, state=st, policy=pol, dt=1.0)
    assert "polygon:nfz-on-path" in bad, bad
    away = K.action_violations(Action4D(vx=-4.0), csp, state=st, policy=pol, dt=1.0)
    assert not any(v.startswith("polygon:") for v in away), away


# --------------------------------------------------------------------------- #
# Determinism (Prefix Compiler PDF p1: "a stateless function")
# --------------------------------------------------------------------------- #

def test_the_same_inputs_give_the_same_csp_and_hash():
    _need()
    pol = load_policy(SURVEY)
    a = ConstraintCompiler(pol).compile_csp(_mission(), now=MONDAY_0800)
    b = ConstraintCompiler(load_policy(SURVEY)).compile_csp(_mission(), now=MONDAY_0800)
    assert a.model_dump_json() == b.model_dump_json()
    assert C.csp_hash(a) == C.csp_hash(b)
    assert a.issued_at == "2026-10-05T08:00:00", a.issued_at


_HASH_SNIPPET = """
import sys; sys.path.insert(0, {root!r})
from datetime import datetime
from guardrail import load_policy
from guardrail.compiler import ConstraintCompiler, Mission
from guardrail.csp import csp_hash
m = Mission(task_text="t", target_x=30, target_y=30, cruise_alt_m=15, speed_pref_mps=4)
out = []
for p in {paths!r}:
    out.append(csp_hash(ConstraintCompiler(load_policy(p)).compile_csp(
        m, now=datetime(2026, 10, 5, 8, 0))))
print("|".join(out))
"""


def test_the_csp_hash_is_stable_across_processes_and_hash_seeds():
    _need()
    paths = [str(p) for p in POLICIES]
    code = _HASH_SNIPPET.format(root=str(ROOT), paths=paths)
    got = []
    for seed in ("1", "2"):
        env = dict(os.environ, PYTHONHASHSEED=seed)
        r = subprocess.run([sys.executable, "-c", code], capture_output=True,
                           text=True, env=env, timeout=300)
        assert r.returncode == 0, r.stderr[-2000:]
        got.append(r.stdout.strip())
    assert got[0] == got[1], "CSP hash depends on PYTHONHASHSEED"
    assert len(set(got[0].split("|"))) == len(paths), "two policies hashed the same"


class _TickingClock(datetime):
    """Patched in as guardrail.compiler.datetime: every now() is one second
    after the last, so 'two runs across a second boundary' needs no sleep and
    cannot land inside one second by luck."""
    calls = 0

    @classmethod
    def now(cls, tz=None):
        cls.calls += 1
        t = datetime(2026, 10, 6, 12, 0, 0) + timedelta(seconds=cls.calls)
        return t.replace(tzinfo=tz) if tz is not None else t


def test_without_a_clock_the_kpi_report_and_the_content_hash_do_not_move():
    """REVIEW FIX (major). With no clock the compiler stamps issued_at from the
    wall clock, and csp_hash covers issued_at, so two KPI reports 1.1 s apart
    differed in 29/29 row hashes while the comment above REPORT_MISSION said
    they 'differ only where the code or the policies do'. The rows now carry
    csp_content_hash. Fails on the code before the fix: its rows' csp_hash
    moves with the clock."""
    _need()
    real = K.datetime
    _TickingClock.calls = 0
    K.datetime = _TickingClock
    try:
        rep1 = K.coverage_report(ROOT / "policies")
        rep2 = K.coverage_report(ROOT / "policies")
        pol = load_policy(DEMO)
        a = ConstraintCompiler(pol).compile_csp(_mission(), now=None)
        b = ConstraintCompiler(pol).compile_csp(_mission(), now=None)
    finally:
        K.datetime = real
    assert _TickingClock.calls >= 2, "the wall clock was never read; this test proved nothing"
    assert a.issued_at != b.issued_at, "the clock did not tick between the two compiles"

    def hashes(rep):
        return {r["file"]: r.get("csp_content_hash", r.get("csp_hash")) for r in rep["rows"]}
    h1, h2 = hashes(rep1), hashes(rep2)
    moved = sorted(f for f in h1 if h1[f] != h2.get(f))
    assert len(h1) == len(POLICIES) and not moved, \
        f"{len(moved)}/{len(h1)} report rows changed hash between two runs: {moved[:3]}"
    assert C.csp_hash(a) != C.csp_hash(b), "csp_hash names one ISSUED CSP and covers issued_at"
    assert C.csp_content_hash(a) == C.csp_content_hash(b)


def test_issued_at_has_one_convention_and_records_where_it_came_from():
    """Before: with a clock, naive local time ('2026-10-05T08:00:00'); without
    one, UTC with '+00:00'. Now the stamp is written the way the clock is given
    (naive local, the Shield's `now` convention) and the wall-clock fallback
    follows it; an explicit issued_at is written as given."""
    _need()
    c = ConstraintCompiler(load_policy(PED))
    clocked = c.compile_csp(_mission(), now=MONDAY_0800)
    wall = c.compile_csp(_mission(), now=None)
    assert clocked.issued_at == "2026-10-05T08:00:00", clocked.issued_at
    assert datetime.fromisoformat(wall.issued_at).tzinfo is None, \
        f"wall-clock stamp {wall.issued_at} does not follow the clock's naive convention"
    assert clocked.selection.issued_at_source == "clock"
    assert wall.selection.issued_at_source == "wall_clock"
    given = c.compile_csp(_mission(), now=None, issued_at="2026-04-28T09:01:12Z")
    assert given.issued_at == "2026-04-28T09:01:12Z", given.issued_at
    assert given.selection.issued_at_source == "argument"
    aware = c.compile_csp(_mission(), now=None,
                          issued_at=datetime(2026, 10, 6, 9, 0, 0, tzinfo=timezone.utc))
    assert aware.issued_at == "2026-10-06T09:00:00+00:00", aware.issued_at
    # The content hash ignores only the stamp: the same content issued twice
    # agrees, while the clock (an input to the time filter) and the mission
    # still move it - a constant content hash would pass the line above.
    assert C.csp_content_hash(given) == C.csp_content_hash(wall)
    assert C.csp_hash(given) != C.csp_hash(wall)
    assert C.csp_content_hash(clocked) != C.csp_content_hash(wall)
    other = c.compile_csp(_mission(tx=-20.0), now=None, issued_at="2026-04-28T09:01:12Z")
    assert C.csp_content_hash(other) != C.csp_content_hash(given)
    try:
        c.compile_csp(_mission(), now=None, issued_at="yesterday")
    except ValueError as e:
        assert "ISO 8601" in str(e), e
    else:
        raise AssertionError("issued_at='yesterday' was accepted")


def test_the_csp_hash_moves_when_the_policy_does():
    """The null for the two tests above: a constant hash passes both."""
    _need()
    pol = load_policy(SURVEY)
    a = ConstraintCompiler(pol).compile_csp(_mission(), now=MONDAY_0800)
    b = ConstraintCompiler(pol.model_copy(update={"generation": 1})).compile_csp(
        _mission(), now=MONDAY_0800)
    assert a.policy_hash != b.policy_hash and C.csp_hash(a) != C.csp_hash(b)


# --------------------------------------------------------------------------- #
# Existing callers keep working
# --------------------------------------------------------------------------- #

def test_summary_pack_without_a_mission_is_the_legacy_dict():
    _need()
    c = ConstraintCompiler(load_policy(PED))
    legacy = {"policy_id", "version", "generation", "policy_hash", "origin",
              "n_rules", "rules_by_type", "rules"}
    assert set(c.summary_pack()) == legacy
    withm = c.summary_pack(_mission(), now=MONDAY_0800)
    assert set(withm) == legacy | {"csp"}
    assert withm["csp"]["csp_version"] == "1.0"
    assert withm["n_rules"] == 4


def test_the_mission_extensions_are_optional():
    m = Mission(task_text="x", target_x=1, target_y=2, cruise_alt_m=3, speed_pref_mps=4)
    assert (m.start_x, m.start_y, m.mission_id) == (0.0, 0.0, None)
    p = ConstraintCompiler(load_policy(DEMO)).parse_command("go to (30, 30) altitude 15")
    assert (p.target_x, p.target_y, p.cruise_alt_m) == (30.0, 30.0, 15.0)


# (text, target_x, target_y, cruise_alt_m, speed_pref_mps). The defaults are
# parse_command's own: 15 m, 6 m/s.
_COMMANDS = [
    # speed is parsed and stripped BEFORE altitude (compiler.py's comment: an
    # "... 6 m/s altitude 20" command once parsed as alt = 6)
    ("fly to the northeast pad at 6 m/s altitude 20", 30.0, 30.0, 20.0, 6.0),
    ("fly to (30, 30) altitude 20 at 6 m/s", 30.0, 30.0, 20.0, 6.0),
    ("fly to (-12.5, 40) at 3 m/s, 25 m altitude", -12.5, 40.0, 25.0, 3.0),
    ("at 4.5 m/s fly to 10, 20 altitude 12", 10.0, 20.0, 12.0, 4.5),
    ("fly to (5, 5) at 6m/s", 5.0, 5.0, 15.0, 6.0),
    ("fly to (30,30) height: 22", 30.0, 30.0, 22.0, 6.0),
    ("terbang ke (10, 20) tinggi 30", 10.0, 20.0, 30.0, 6.0),
    ("Fly To The East Pad ALTITUDE 18", 0.0, 35.0, 18.0, 6.0),
    # "north pad" must not be shadowed by, nor shadow, "northeast pad"
    ("fly to the north pad", 35.0, 0.0, 15.0, 6.0),
    ("go home", 0.0, 0.0, 15.0, 6.0),
]


def test_parse_command_edge_cases():
    """Audit WP2-14: 'no test for ... parse_command edge cases'. The mission
    the CSP is compiled for comes from here, so a mis-parsed altitude would
    change every relevance explanation and the altitude band check."""
    c = ConstraintCompiler(load_policy(DEMO))
    for text, tx, ty, alt, v in _COMMANDS:
        m = c.parse_command(text)
        got = (m.target_x, m.target_y, m.cruise_alt_m, m.speed_pref_mps)
        assert got == (tx, ty, alt, v), (text, got)
        assert m.task_text == text


def test_parse_command_refuses_a_command_with_no_target():
    """No target is an error naming the known places, never a default of (0, 0)."""
    c = ConstraintCompiler(load_policy(DEMO))
    try:
        c.parse_command("fly somewhere nice at 5 m/s")
    except ValueError as e:
        assert "known places" in str(e), e
        return
    raise AssertionError("a command with no target parsed")


def test_a_csp_stored_under_the_old_16_hex_hash_still_checks_against_its_policy():
    """A CSP written before 2026-10-06 carries the 16-hex legacy policy hash.
    p0_coverage / rule_coverage / action_violations compared hashes with `!=`,
    so such a CSP was refused against the very policy it was compiled from.
    They now use Policy.matches_hash, which knows every recorded form. A CSP
    for ANOTHER policy must still be refused, under either form."""
    _need()
    pol = load_policy(PED)
    csp = ConstraintCompiler(pol).compile_csp(_mission(), now=MONDAY_0800)
    legacy = sorted(pol.legacy_hashes().values())[0]
    assert len(legacy) < len(pol.policy_hash), (legacy, pol.policy_hash)
    old = csp.model_copy(update={"policy_hash": legacy})
    assert K.p0_coverage(old, pol)["coverage"] == K.p0_coverage(csp, pol)["coverage"]
    assert K.rule_coverage(old, pol)["all"] == K.rule_coverage(csp, pol)["all"]
    assert K.action_violations(Action4D(), old, state=State(x=0, y=0, up=15),
                               policy=pol) == []
    other = load_policy(DEMO)
    for bad in (old, csp):
        for fn in (lambda c: K.p0_coverage(c, other),
                   lambda c: K.rule_coverage(c, other),
                   lambda c: K.action_violations(Action4D(), c, state=State(x=0, y=0, up=15),
                                                 policy=other)):
            try:
                fn(bad)
            except ValueError:
                continue
            raise AssertionError(f"a CSP for {bad.policy_hash} was accepted against "
                                 f"{other.policy_hash}")


def test_action_violations_give_the_same_answer_with_prebuilt_rings():
    """`ir=` reuses the margin rings guardrail.ir compiled once (for a replay
    that calls this per frame). Same verdicts at every probe point, with and
    without; an IR compiled from a DIFFERENT policy is ignored, not trusted."""
    _need()
    from guardrail.ir import PolicyIR
    pol = _mix()
    csp = ConstraintCompiler(pol).compile_csp(_mission(**MIX_MISSION), now=None)
    ir = PolicyIR.from_policy(pol)
    assert ir.fences, "fixture has no fence, so the ring path is not exercised"
    n_flagged = 0
    for x in range(20, 61, 4):
        for y in range(-20, 21, 4):
            st = State(x=float(x), y=float(y), up=15.0)
            a = K.action_violations(Action4D(vx=4.0), csp, state=st, policy=pol)
            b = K.action_violations(Action4D(vx=4.0), csp, state=st, policy=pol, ir=ir)
            assert a == b, (x, y, a, b)
            n_flagged += any(v.startswith("polygon:") for v in a)
    assert n_flagged > 0, "no probe point reached a fence; the comparison proved nothing"
    # A stale IR must not be used: same answer as none. It is built from the
    # SAME policy with the on-path fence grown (same fence id, so a lookup by
    # id would find its ring), and the probe point is inside the stale ring
    # but outside the real one - so trusting it WOULD change the answer.
    # (Until the 6 Oct review the stale IR came from another policy whose
    # fence ids differ, and the check passed whether or not staleness was
    # tested.)
    from guardrail.geometry import point_in_fence
    grown = json.loads(json.dumps(MIX))
    grown["constraints"][0]["vertices"] = _sq(30, -15, 60, 15)
    stale = PolicyIR.from_policy(Policy.model_validate(grown))
    assert not stale.is_current_for(pol)
    st = State(x=28.0, y=0.0, up=15.0)                      # +4 m/s x 1 s -> x = 32
    fence = {c.id: c for c in pol.constraints}["nfz-on-path"]
    stale_ring = {r.rule.id: r.buffered for r in stale.fences}["nfz-on-path"]
    assert point_in_fence(32.0, 0.0, 15.0, fence, buffered=stale_ring), "probe not in stale ring"
    assert not point_in_fence(32.0, 0.0, 15.0, fence), "probe inside the real fence"
    fresh = K.action_violations(Action4D(vx=4.0), csp, state=st, policy=pol)
    assert "polygon:nfz-on-path" not in fresh, fresh
    assert K.action_violations(Action4D(vx=4.0), csp, state=st, policy=pol, ir=stale) == fresh


def test_a_csp_can_be_written_with_a_flight():
    _need()
    out = Path(tempfile.mkdtemp()) / "csp_g0.json"
    ConstraintCompiler(load_policy(PED)).write_csp(out, _mission(), now=MONDAY_0800)
    got = C.CSP.model_validate_json(out.read_text(encoding="utf-8"))
    assert got.policy_hash == load_policy(PED).policy_hash


# --------------------------------------------------------------------------- #
# 2026-10-08: the hot-apply types fly (WP1-15 / WP1-09), the scale is the IR's
# (WP2-16), and the cost-map adapter (WP2-07, deviation 4)
# --------------------------------------------------------------------------- #

def _weekdays(start, end, days=("Mon", "Tue", "Wed", "Thu", "Fri")):
    return {"recurrence": {"days": list(days), "start_time": start, "end_time": end}}


# Every hot-apply type, each with an edge the clock can sit on: fence-a is in
# force on weekdays 07:30-17:30 and switched off Mondays 12:00-13:00; fence-b
# is switched off Mondays 08:00-08:30; the main corridor is swapped for a
# narrower, lower one Mondays 09:00-10:00; one zone moves at 2 m/s; one does
# not move and lies far from every mission.
HOT = {
    "policy_id": "hot-apply-mix",
    "constraints": [
        {"id": "fence-a", "type": "polygon_fence", "priority": "P0",
         "vertices": _sq(40, -25, 50, -15), "valid_time": _weekdays("07:30", "17:30")},
        {"id": "fence-b", "type": "polygon_fence", "priority": "P0",
         "vertices": _sq(60, 20, 70, 30)},
        {"id": "corr-main", "type": "corridor", "priority": "P0",
         "centerline": [{"x": 0, "y": 0}, {"x": 100, "y": 0}], "width_m": 40,
         "altitude_floor_m": 5, "altitude_ceiling_m": 60},
        {"id": "dyn", "type": "dynamic_nfz", "priority": "P0",
         "vertices": _sq(20, 30, 30, 40),
         "motion": {"vx_mps": 2.0, "vy_mps": 0.0, "yaw_rate_dps": 0.0}},
        {"id": "dyn-still", "type": "dynamic_nfz", "priority": "P1",
         "vertices": _sq(-400, -400, -390, -390)},
        {"id": "sw-a-off", "type": "time_window_switch", "priority": "P0",
         "target_id": "fence-a", "active": False,
         "valid_time": _weekdays("12:00", "13:00", ("Mon",))},
        {"id": "sw-b-off", "type": "time_window_switch", "priority": "P1",
         "target_id": "fence-b", "active": False,
         "valid_time": _weekdays("08:00", "08:30", ("Mon",))},
        {"id": "swap-main", "type": "corridor_swap", "priority": "P0",
         "target_id": "corr-main",
         "centerline": [{"x": 0, "y": 0}, {"x": 100, "y": 10}], "width_m": 30,
         "altitude_floor_m": 10, "altitude_ceiling_m": 50,
         "valid_time": _weekdays("09:00", "10:00", ("Mon",))},
    ],
}
HOT_MISSION = dict(tx=100.0, ty=0.0, alt=15.0, speed=4.0)
MON = datetime(2026, 10, 5)                     # a Monday


def _hot():
    """HOT as a declaration (DSL lint applied). Not for flight: its two
    dynamic_nfz are held back from policy files (models.FLIGHT_HELD_TYPES)."""
    from guardrail.models import policy_from_raw
    return policy_from_raw(json.loads(json.dumps(HOT)), source="HOT", runtime=False)


def test_the_hot_apply_types_compile_and_a_switch_or_swap_flies():
    """The three hot-apply classes, which the Shield has enforced since
    2026-10-07, now have their templates and are in RUNTIME_TYPES, so a
    policy or a hot-applied generation holding them compiles a CSP. Shown
    failing on 10451bc: `policy_from_raw` refused HOT ("cannot enforce ...
    dynamic_nfz, time_window_switch, corridor_swap") and compile_csp raised
    CSPUnenforcedRules on the same rules.

    A policy FILE with a switch or a swap loads for flight. One declaring a
    dynamic_nfz does not (models.FLIGHT_HELD_TYPES, found in review on
    2026-10-08): the Shield enforces the zone, but the flight rails' dwell
    metric (nfz_s) counts polygon and circle fences only and would score it
    0 s. The refusal names each zone and that reason."""
    _need()
    from guardrail.models import FLIGHT_HELD_TYPES, RUNTIME_TYPES, policy_from_raw
    assert {"dynamic_nfz", "time_window_switch", "corridor_swap"} <= RUNTIME_TYPES
    assert set(FLIGHT_HELD_TYPES) == {"dynamic_nfz"}
    pol = _hot()
    held = pol.flight_problems()
    assert sorted(p.split(":")[0] for p in held) == ["dyn", "dyn-still"], held
    assert all("nfz_s" in p for p in held), held
    try:
        policy_from_raw(json.loads(json.dumps(HOT)), source="HOT")   # the flight gate
    except ValueError as e:
        assert "dyn:" in str(e) and "dyn-still:" in str(e) and "nfz_s" in str(e), str(e)
    else:
        raise AssertionError("a policy file declaring a dynamic_nfz loaded for flight")
    no_zones = {**HOT, "constraints": [c for c in HOT["constraints"]
                                       if c["type"] != "dynamic_nfz"]}
    flies = policy_from_raw(json.loads(json.dumps(no_zones)), source="HOT-no-zones")
    assert flies.flight_problems() == []
    csp = ConstraintCompiler(pol).compile_csp(_mission(**HOT_MISSION),
                                              now=MON.replace(hour=8, minute=15),
                                              budget_tokens=1000)
    dec = {r.id: r for r in csp.selection.rules}
    nl = csp.natural_language_prompt
    assert "Never enter zone 'dyn', which moves at 2 m/s." in nl, nl
    assert "Rule 'fence-b' is switched off Mon 08:00-08:30." in csp.P1_summary + nl \
        or "rule 'fence-b' switched off (Mon 08:00-08:30)" in csp.P1_summary, csp.P1_summary
    assert dec["fence-b"].decision == "filtered_inactive", dec["fence-b"]
    assert "held off by sw-b-off" in dec["fence-b"].reason, dec["fence-b"].reason
    assert dec["dyn-still"].decision == "filtered_out_of_region", dec["dyn-still"]
    assert dec["swap-main"].decision == "filtered_inactive"
    assert K.p0_coverage(csp, pol)["coverage"] == 1.0
    assert K.rule_coverage(csp, pol)["all"]["recorded"] == 1.0
    f = csp.forbidden_action_set
    assert set(f.no_translate_into_polygons) == {"fence-a", "dyn"}, f
    assert f.no_translate_out_of_corridors == ["corr-main"], f
    assert csp.allowed_action_set.altitude_band_m_agl == (5.0, 60.0)
    # 09:30 Monday: the swap holds, so the main corridor is replaced.
    swapped = ConstraintCompiler(pol).compile_csp(_mission(**HOT_MISSION),
                                                  now=MON.replace(hour=9, minute=30),
                                                  budget_tokens=1000)
    d2 = {r.id: r for r in swapped.selection.rules}
    assert d2["corr-main"].decision == "filtered_inactive"
    assert "held off by swap-main" in d2["corr-main"].reason, d2["corr-main"].reason
    assert swapped.forbidden_action_set.no_translate_out_of_corridors == ["swap-main"]
    assert swapped.allowed_action_set.altitude_band_m_agl == (10.0, 50.0)
    assert ("Stay inside corridor 'swap-main', which replaces 'corr-main', within 15 m "
            "of its centerline and between 10 m and 50 m (in force Mon 09:00-10:00)."
            ) in swapped.natural_language_prompt, swapped.natural_language_prompt
    refs = {p.id: p.geometry_ref for p in swapped.P0_constraints}
    assert refs["dyn"] == "fine:dyn@v0.1.0/g0" and refs["swap-main"].startswith("fine:")
    assert K.resolve_geometry_ref(pol, refs["dyn"])[0] == (20.0, 30.0)


def _shield_in_force(pol, when):
    from guardrail.shield import Shield
    sh = Shield(pol, now=(lambda: when) if when is not None else None, escalation=False)
    return {r["id"]: r["in_force"] for r in sh.rule_status(State(x=-1e5, y=-1e5, up=20.0))}


def test_the_csp_times_switches_and_swaps_exactly_as_the_shield_does():
    """The CSP must never leave out a rule the Shield enforces. A switch that
    turns a rule OFF is where the compiler's old minute-grained clock was not
    conservative: models.Recurrence.active_at holds a window ending 08:30 open
    until 08:30:59, while the Shield closes it at 08:30:00.000 - so the target
    would be missing from the CSP for most of a minute while the Shield
    enforced it. Checked at instants on both sides of every edge of HOT,
    against the Shield's own `rule_status()["in_force"]`, and the minute
    reading is run as the null: it fails the same check, so the check can."""
    _need()
    pol = _hot()
    clock = K._RuleClock(pol)

    class _Minute(K._RuleClock):                 # the null: the old clock
        @staticmethod
        def _base(rule, x):
            vt = rule.valid_time
            if vt is None or vt.recurrence is None:
                return True
            return None if x is None else vt.recurrence.active_at(x)
    minute = _Minute(pol)
    edges = [(7, 30), (8, 0), (8, 30), (9, 0), (10, 0), (12, 0), (13, 0), (17, 30)]
    offsets = (-61, -20, -1, -0.05, -1e-6, 0, 1e-6, 0.05, 1, 20, 61)
    n, exact, null_misses = 0, 0, []
    for hh, mm in edges:
        for off in offsets:
            when = MON.replace(hour=hh, minute=mm) + timedelta(seconds=off)
            shield = _shield_in_force(pol, when)
            csp = ConstraintCompiler(pol).compile_csp(_mission(**HOT_MISSION), now=when,
                                                      lookahead_s=5.0, region_margin_m=None,
                                                      budget_tokens=1000)
            dec = {r.id: r.decision for r in csp.selection.rules}
            for c in pol.constraints:
                if c.type == "time_window_switch":
                    continue                     # an event: the Shield never "enforces" it
                n += 1
                if shield[c.id]:
                    assert dec[c.id] != "filtered_inactive", (when, c.id)
                    if not minute.activity(c, when, 5.0)["any"]:
                        null_misses.append((when.time().isoformat(), c.id))
                # Exact agreement with the Shield's both-sides reading.
                eps = timedelta(seconds=0.05)
                if clock.involved(c):
                    assert (clock.on(c, when - eps) or clock.on(c, when + eps)) == shield[c.id], \
                        (when, c.id)
                    exact += 1
    assert n > 400 and exact > 200, (n, exact)
    assert null_misses, "the minute-grained null never left out an enforced rule; the check proves nothing"
    print(f"      {n} rule-instants, {exact} timed by switches/swaps agree exactly with "
          f"the Shield; the minute-grained null leaves out {len(null_misses)} enforced "
          f"rule-instants, e.g. {null_misses[0]}")


def test_without_a_clock_a_switch_applies_only_where_the_shield_applies_it():
    """No clock: a switch with a window cannot be read, so it never turns a
    rule off (the Shield's rule); a switch with no window always applies."""
    _need()
    raw = json.loads(json.dumps(HOT))
    raw["constraints"].append({"id": "sw-a-always", "type": "time_window_switch",
                               "target_id": "fence-a", "active": False})
    from guardrail.models import policy_from_raw
    pol = policy_from_raw(raw, runtime=False)
    csp = ConstraintCompiler(pol).compile_csp(_mission(**HOT_MISSION), now=None,
                                              budget_tokens=1000)
    dec = {r.id: r for r in csp.selection.rules}
    sh = _shield_in_force(pol, None)
    assert not sh["fence-a"] and dec["fence-a"].decision == "filtered_inactive"
    assert "held off by sw-a-always" in dec["fence-a"].reason, dec["fence-a"].reason
    assert sh["fence-b"] and dec["fence-b"].decision == "kept", dec["fence-b"]
    assert sh["corr-main"] and dec["corr-main"].decision == "kept"
    assert "Rule 'fence-a' is switched off." in csp.natural_language_prompt


def test_build_prompt_carries_every_runtime_type():
    """build_prompt's by-type order stopped at stand-off until 2026-10-08: a
    dynamic_nfz, corridor_swap or time_window_switch would have been dropped
    from its text WITHOUT A WORD the day the loaders accepted them. Shown
    failing on 10451bc: _sentences() returned 5 sentences for HOT's 8 rules."""
    _need()
    pol = _hot()
    sents = ConstraintCompiler(pol)._sentences()
    assert len(sents) == len(pol.constraints), (len(sents), len(pol.constraints))
    text = " ".join(sents)
    for rid in ("dyn", "dyn-still", "swap-main", "fence-b"):
        assert f"'{rid}'" in text, rid
    base = K.naive_prefix_baseline(pol, 1000)
    assert base["n_rules_in_text"] == len(pol.constraints), base


def test_the_compiler_reads_the_multiscale_geometry_the_ir_carries():
    """WP2-16: "the Prefix Compiler picks the scale, never re-computes". Until
    2026-10-08 guardrail/compiler.py computed its own hull and oriented
    rectangle beside the IR's (the same rule twice, kept equal only by a
    test). Now the compiler's multi-scale cache IS the IR's object, and a
    zone or tube type the IR knows (dynamic_nfz, corridor_swap) is there too.
    Shown failing on 10451bc: no `compiled_ir`, and the compiler's cache had
    no entry for `dyn` or `swap-main`."""
    _need()
    pol = _hot()
    ir = K.compiled_ir(pol)
    assert K.multiscale_geometry(pol) is ir.multiscale
    assert not hasattr(K, "_convex_hull") and not hasattr(K, "_min_area_rect")
    assert set(ir.multiscale["dyn"]) == {"fine", "coarse", "bbox"}
    assert set(ir.multiscale["swap-main"]) == {"fine"}, "a keep-in tube is never coarsened"
    for p in POLICIES[:6]:
        q = load_policy(p)
        csp = ConstraintCompiler(q).compile_csp(_mission(), now=None)
        for e in csp.P0_constraints:
            if not e.geometry_ref.startswith("param:"):
                assert K.resolve_geometry_ref(q, e.geometry_ref) == \
                    K.compiled_ir(q).resolve_geometry_ref(e.geometry_ref)
    bumped = pol.model_copy(update={"generation": 1})
    assert K.compiled_ir(bumped) is not ir and K.compiled_ir(bumped).generation == 1


def test_the_new_sentences_are_bounded_by_the_table_counter():
    """The table counter has no entries for the words the new templates use,
    so they are counted by their bytes - an upper bound by construction.
    Checked against the exact tokenizer where it exists."""
    _need()
    pol = _hot()
    disc = Policy.model_validate({"policy_id": "c", "constraints": [
        {"id": "disc", "type": "circle_fence", "center": {"x": 1, "y": 1},
         "radius_m": 4.5}]})
    texts = [K.render_sentence(c) for c in pol.constraints + disc.constraints]
    texts += [K.render_summary(c) for c in pol.constraints + disc.constraints]
    table = C.TableTokenCounter()
    exact = C.openvla_token_counter()
    if exact is None:
        return _skip("no OpenVLA tokenizer here; the byte bound holds by construction")
    for t in texts:
        assert table.count(t) >= exact.count(t), (t, table.count(t), exact.count(t))


# ---- the cost-map adapter -------------------------------------------------- #

WGS84 = ROOT / "policies" / "wgs84_taipei.yaml"


def test_the_cost_map_follows_the_grants_conventions():
    """Prefix Compiler page: resolution_m 5, an origin {lat, lon}, layers
    nfz_hard (cost inf), corridor_pref (0), corridor_outside (50)."""
    _need()
    geo = K.rasterise_cost_map(load_policy(WGS84), now=MONDAY_0800)
    assert set(geo.header["cost_map"]["origin"]) == {"lat", "lon"}
    o = geo.header["cost_map"]["origin"]
    assert abs(o["lat"] - 25.04) < 0.01 and abs(o["lon"] - 121.53) < 0.01, o
    pol = load_policy(SURVEY)
    cm = K.rasterise_cost_map(pol, now=MONDAY_0800)
    h = cm.header
    assert h["cost_map"]["resolution_m"] == 5.0
    names = {(L["name"], L["cost"]) for L in h["cost_map"]["layers"]}
    assert ("nfz_hard", "inf") in names and ("corridor_pref", 0.0) in names \
        and ("corridor_outside", 50.0) in names, names
    assert cm.cost.dtype == np.float32 and np.isinf(cm.cost).any()
    assert set(np.unique(cm.cost[np.isfinite(cm.cost)]).tolist()) <= {0.0, 50.0}
    assert h["policy_hash"] == pol.policy_hash and h["generation"] == pol.generation
    assert cm.ref.startswith(f"costmap:{pol.policy_id}@v{pol.version}/g0#sha256:")
    metre = K.rasterise_cost_map(load_policy(DEMO))
    assert metre.header["cost_map"]["origin"] is None, "a metre policy has no WGS84 origin"
    # What has no footprint is listed with why, never dropped in silence.
    whys = {d["id"]: d["why"] for d in metre.header["not_drawn"]}
    assert any("speed" in w for w in whys.values()), whys


def test_the_cost_map_never_frees_a_place_the_shield_forbids_and_beats_both_nulls():
    """The score and its two nulls (cost_map_agreement): the map must miss
    NO point the Shield forbids (an all-free grid misses all of them), and
    must over-cost far fewer allowed points than an all-inf grid (which
    over-costs every one). A map with one zone's cells freed is caught."""
    _need()
    cases = [(load_policy(DEMO), None), (load_policy(SURVEY), MONDAY_0800),
             (_mix(), None), (_hot(), MON.replace(hour=8, minute=15)),
             (_hot(), MON.replace(hour=9, minute=30))]
    for pol, now in cases:
        cm = K.rasterise_cost_map(pol, now=now, altitude_m=15.0)
        ag = K.cost_map_agreement(cm, pol, now=now, n_points=600)
        assert ag["n_forbidden"] > 0 and ag["n_allowed"] > 0, (pol.policy_id, ag)
        assert ag["map"]["missed"] == 0, (pol.policy_id, now, ag)
        assert ag["null_all_free"]["missed"] == ag["n_forbidden"], ag
        assert ag["null_all_inf"]["over_cost"] == ag["n_allowed"], ag
        assert ag["map"]["over_cost"] < 0.5 * ag["n_allowed"], (pol.policy_id, ag)
    pol = _mix()
    cm = K.rasterise_cost_map(pol, altitude_m=15.0)
    k = next(i for i, L in enumerate(cm.header["cost_map"]["layers"])
             if L["source"] == "nfz-on-path")
    holed = K.CostMap(cost=np.where(cm.layers[k], np.float32(0), cm.cost),
                      layers=cm.layers, header=cm.header)
    ag = K.cost_map_agreement(holed, pol, n_points=3000)
    assert ag["map"]["missed"] > 0, "a map with a zone freed passed the scorer"


def test_a_moving_zone_is_drawn_where_the_shield_has_it():
    """A dynamic_nfz moving at 2 m/s, 10 s after it appeared, is 20 m on - in
    the map and in the Shield (ticked to t = 10 s). The map drawn at age 0 and
    scored at age 10 misses points: the scorer sees a stale position."""
    _need()
    pol = _hot()
    now = MON.replace(hour=8, minute=15)
    m0 = K.rasterise_cost_map(pol, now=now, altitude_m=15.0, zone_age_s=0.0,
                              bounds=(-20, -60, 140, 80))
    m10 = K.rasterise_cost_map(pol, now=now, altitude_m=15.0, zone_age_s=10.0,
                               bounds=(-20, -60, 140, 80))
    k0 = next(i for i, L in enumerate(m0.header["cost_map"]["layers"]) if L["source"] == "dyn")
    k10 = next(i for i, L in enumerate(m10.header["cost_map"]["layers"]) if L["source"] == "dyn")
    shift = np.argwhere(m10.layers[k10]).min(0) - np.argwhere(m0.layers[k0]).min(0)
    assert tuple(shift) == (4, 0), shift                     # 20 m north at 5 m cells
    assert m10.header["zone_age_s"] == {"dyn": 10.0}
    assert K.cost_map_agreement(m10, pol, now=now, n_points=1500)["map"]["missed"] == 0
    stale = K.CostMap(cost=m0.cost, layers=m0.layers, header=m10.header)
    assert K.cost_map_agreement(stale, pol, now=now, n_points=1500)["map"]["missed"] > 0


def test_standoff_and_clearance_are_drawn_only_with_their_inputs():
    """Both rules are inert in the Shield without a subject / an obstacle map;
    the map says so by name. With them, the map agrees with the Shield's own
    stand-off test and distance field."""
    _need()
    pol = Policy.model_validate({"policy_id": "so", "constraints": [
        {"id": "fence", "type": "polygon_fence", "vertices": _sq(30, 30, 40, 40)},
        {"id": "keep-off", "type": "obstacle_clearance", "min_clearance_m": 4.0},
        {"id": "standoff", "type": "subject_standoff", "min_range_m": 10.0},
        {"id": "ped-only", "type": "subject_standoff", "subject_class": "pedestrian",
         "min_range_m": 20.0}]})
    bare = K.rasterise_cost_map(pol)
    whys = {d["id"]: d["why"] for d in bare.header["not_drawn"]}
    assert "no obstacle map" in whys["keep-off"] and "no subject" in whys["standoff"], whys
    occ = np.zeros((30, 30), dtype=np.uint8)
    occ[5:8, 20:24] = 1
    omap = {"occ": occ, "res": 2.0, "ox": 0.0, "oy": 0.0}
    cm = K.rasterise_cost_map(pol, altitude_m=15.0, subject=(10.0, 10.0, "car"),
                              obstacle_map=omap, resolution_m=2.0)
    names = {L["name"]: L for L in cm.header["cost_map"]["layers"]}
    assert names["standoff_hard"]["source"] == "standoff" and names["standoff_hard"]["cells"] > 0
    assert names["clearance_hard"]["cells"] > 0
    assert "binds 'pedestrian'" in {d["id"]: d["why"] for d in cm.header["not_drawn"]}["ped-only"]
    ag = K.cost_map_agreement(cm, pol, obstacle_map=omap, n_points=3000)
    assert ag["n_forbidden"] > 100 and ag["map"]["missed"] == 0, ag
    assert ag["map"]["over_cost"] < 0.5 * ag["n_allowed"], ag


def test_the_whole_grid_is_refused_at_a_height_the_envelope_forbids():
    _need()
    pol = _mix()                                   # alt-band 5-50 m, hard
    cm = K.rasterise_cost_map(pol, altitude_m=70.0)
    assert np.isinf(cm.cost).all()
    assert any(L["name"] == "altitude_hard" for L in cm.header["cost_map"]["layers"])
    assert K.cost_map_agreement(cm, pol, n_points=300)["map"]["missed"] == 0
    ok = K.rasterise_cost_map(pol, altitude_m=20.0)
    assert not np.isinf(ok.cost).all()
    low = K.rasterise_cost_map(_hot(), now=MON.replace(hour=8, minute=15), altitude_m=2.0)
    out = next(L for L in low.header["cost_map"]["layers"] if L["name"] == "corridor_outside")
    assert out["cells"] == low.cost.size, "below the corridor band, everywhere is outside it"
    assert "fence-b" in low.header["not_in_force"]


def test_the_cost_map_export_round_trips_and_refuses_tampering():
    _need()
    import zipfile
    pol = _mix()
    cm = K.rasterise_cost_map(pol, altitude_m=15.0)
    d = Path(tempfile.mkdtemp())
    npz, js = cm.save(d / "mix")
    assert npz.name == "mix.npz" and js.name == "mix.json"
    assert b"\r\n" not in js.read_bytes()
    # The grant's Outputs table says the adapter "emits YAML": the JSON header
    # is a YAML document, and a YAML reader gets the same values from it.
    import yaml as _yaml
    text = js.read_text(encoding="utf-8")
    assert _yaml.safe_load(text) == json.loads(text)
    back = K.load_cost_map(js, pol)
    assert back.ref == cm.ref and np.array_equal(back.cost, cm.cost)
    assert all(np.array_equal(a, b) for a, b in zip(back.layers, cm.layers))
    doc = json.loads(js.read_text(encoding="utf-8"))
    doc["cost_map"]["resolution_m"] = 4.0
    (d / "edited.json").write_text(json.dumps(doc), encoding="utf-8")
    try:
        K.load_cost_map(d / "edited.json")
    except ValueError as e:
        assert "edited" in str(e), e
    else:
        raise AssertionError("an edited header loaded")
    with np.load(npz) as z:
        arrays = {k: z[k] for k in z.files}
    arrays["cost"] = np.zeros_like(arrays["cost"])
    np.savez_compressed(d / "mix.npz", **arrays)
    try:
        K.load_cost_map(js)
    except ValueError as e:
        assert "not the one" in str(e), e
    else:
        raise AssertionError("a zeroed grid loaded under the old header")
    cm.save(d / "mix")
    try:
        K.load_cost_map(js, pol.model_copy(update={"generation": 1}))
    except ValueError as e:
        assert "g0" in str(e), e
    else:
        raise AssertionError("a generation-0 map loaded for generation 1")
    assert zipfile.is_zipfile(npz)


def test_cost_map_ref_reaches_the_csp_only_when_asked():
    """The grant: `cost_map_ref: str | None = None  # for path planners;
    lazy-loaded artefact`. Without a map the CSP is what it was (no stored CSP
    moved); with one, its ref; a map of another policy is refused."""
    _need()
    pol = _mix()
    comp = ConstraintCompiler(pol)
    plain = comp.compile_csp(_mission(**MIX_MISSION), now=None, issued_at="2026-10-08T00:00:00")
    assert plain.cost_map_ref is None
    cm = comp.cost_map(altitude_m=15.0)
    with_map = comp.compile_csp(_mission(**MIX_MISSION), now=None,
                                issued_at="2026-10-08T00:00:00", cost_map=cm)
    assert with_map.cost_map_ref == cm.ref
    assert with_map.model_copy(update={"cost_map_ref": None}) == plain
    other = K.rasterise_cost_map(load_policy(DEMO))
    try:
        comp.compile_csp(_mission(**MIX_MISSION), now=None, cost_map=other)
    except ValueError as e:
        assert "rasterised from" in str(e), e
    else:
        raise AssertionError("a cost map of another policy was referenced")


def test_write_csp_exports_the_cost_map_it_references():
    """A CSP written with a cost map carries the map beside it, so its
    cost_map_ref names a file that travels with the CSP: written as
    costmap_<stem>.npz/.json (outside the replay bundle's csp_g*.json
    pattern), resolved by load_cost_map with the CSP's own ref, and refused
    under another map's ref. A map of another policy is refused before
    anything is written. Shown failing on the first 2026-10-08 version:
    write_csp set the ref but wrote no map, and load_cost_map took no ref."""
    _need()
    pol = _mix()
    comp = ConstraintCompiler(pol)
    cm = comp.cost_map(altitude_m=15.0)
    d = Path(tempfile.mkdtemp())
    p = comp.write_csp(d / "csp_g0.json", _mission(**MIX_MISSION), now=None,
                       issued_at="2026-10-08T00:00:00", cost_map=cm)
    csp = json.loads(p.read_text(encoding="utf-8"))
    assert csp["cost_map_ref"] == cm.ref
    js = d / "costmap_csp_g0.json"
    assert js.is_file() and (d / "costmap_csp_g0.npz").is_file(), sorted(os.listdir(d))
    assert sorted(x.name for x in d.glob("csp_g*.json")) == ["csp_g0.json"]
    back = K.load_cost_map(js, pol, ref=csp["cost_map_ref"])
    assert np.array_equal(back.cost, cm.cost)
    high = comp.cost_map(altitude_m=40.0)          # same policy and generation
    assert high.ref != cm.ref
    try:
        K.load_cost_map(js, pol, ref=high.ref)
    except ValueError as e:
        assert "not the map" in str(e), e
    else:
        raise AssertionError("a cost map loaded under another map's ref")
    d2 = Path(tempfile.mkdtemp())
    try:
        comp.write_csp(d2 / "csp_g0.json", _mission(**MIX_MISSION), now=None,
                       cost_map=K.rasterise_cost_map(load_policy(DEMO)))
    except ValueError:
        pass
    else:
        raise AssertionError("write_csp wrote a CSP naming another policy's cost map")
    assert not any(d2.iterdir()), sorted(os.listdir(d2))
    # Without a map, write_csp writes the CSP alone, as before.
    d3 = Path(tempfile.mkdtemp())
    comp.write_csp(d3 / "csp_g0.json", _mission(**MIX_MISSION), now=None)
    assert sorted(os.listdir(d3)) == ["csp_g0.json"]


def test_a_dynamic_zone_names_only_the_motion_it_has():
    """A zone that only turns is not said to travel, one that only
    translates is not said to turn at 0 deg/s, and a non-zero rate never
    prints as 0. Shown failing on the first 2026-10-08 templates: the reason
    read "this zone moves (2 m/s, turning 0 deg/s)" for a zone that does not
    turn, "(0 m/s, turning 7.5 deg/s), up to 0 m" for one that only turns,
    and the sentence "which moves at 0 m/s" for a zone creeping at 0.03 m/s."""
    _need()
    from guardrail.models import policy_from_raw

    def zone(zid, x0, motion):
        r = {"id": zid, "type": "dynamic_nfz", "vertices": _sq(x0, 50, x0 + 10, 60)}
        if motion is not None:
            r["motion"] = motion
        return r
    pol = policy_from_raw({"policy_id": "motions", "constraints": [
        zone("go", 0, {"vx_mps": 2.0, "vy_mps": 0.0, "yaw_rate_dps": 0.0}),
        zone("spin", 20, {"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rate_dps": -7.5}),
        zone("both", 40, {"vx_mps": 0.0, "vy_mps": 1.5, "yaw_rate_dps": 3.0}),
        zone("creep", 60, {"vx_mps": 0.03, "vy_mps": 0.0, "yaw_rate_dps": 0.0}),
        zone("still", 80, None)]}, runtime=False)
    by = {c.id: c for c in pol.constraints}
    want_sentence = {
        "go": "Never enter zone 'go', which moves at 2 m/s.",
        "spin": "Never enter zone 'spin', which turns at 7.5 deg/s.",
        "both": "Never enter zone 'both', which moves at 1.5 m/s and turns at 3 deg/s.",
        "creep": "Never enter zone 'creep', which moves at 0.03 m/s.",
        "still": "Never enter zone 'still'.",
    }
    want_summary = {
        "go": "no entry into 'go' (moving 2 m/s)",
        "spin": "no entry into 'spin' (turning 7.5 deg/s)",
        "both": "no entry into 'both' (moving 1.5 m/s, turning 3 deg/s)",
        "creep": "no entry into 'creep' (moving 0.03 m/s)",
        "still": "no entry into 'still'",
    }
    for zid, c in by.items():
        assert K.render_sentence(c) == want_sentence[zid], (zid, K.render_sentence(c))
        assert K.render_summary(c) == want_summary[zid], (zid, K.render_summary(c))
    csp = ConstraintCompiler(pol).compile_csp(_mission(tx=100.0, ty=55.0), now=None,
                                              budget_tokens=1000)
    rel = csp.relevance_explanations
    assert rel["go"].startswith("this zone moves at 2 m/s, up to 10 m in the 5 s "
                                "lookahead; "), rel["go"]
    assert rel["spin"].startswith("this zone turns at 7.5 deg/s; "), rel["spin"]
    assert rel["both"].startswith("this zone moves at 1.5 m/s, up to 7.5 m in the 5 s "
                                  "lookahead and turns at 3 deg/s; "), rel["both"]
    assert rel["creep"].startswith("this zone moves at 0.03 m/s"), rel["creep"]
    assert rel["still"].startswith(("this dynamic zone is", "the straight path")), rel["still"]
    for zid, text in rel.items():
        assert "0 deg/s" not in text and "up to 0 m" not in text, (zid, text)


# ---- review fixes, 2026-10-08 (late) --------------------------------------- #

def test_compile_csp_refuses_a_cost_map_of_another_instant_or_height():
    """Two maps of one generation differ by clock and height, so matching the
    policy hash and generation is not enough. Shown failing on the first
    2026-10-08 version: a CSP issued Mon 08:15 for a 15 m cruise accepted a
    map rasterised at Mon 12:30 (fence-a, which the Shield enforces at 08:15,
    not drawn; fence-b, which a switch holds off, drawn at inf) and a map
    drawn at 200 m; write_csp then exported that map and named it."""
    _need()
    pol = _hot()
    comp = ConstraintCompiler(pol)
    at = MON.replace(hour=8, minute=15)
    mission = _mission(**HOT_MISSION)                     # cruises at 15 m
    good = comp.cost_map(now=at, altitude_m=15.0)
    assert comp.compile_csp(mission, now=at, budget_tokens=1000,
                            cost_map=good).cost_map_ref == good.ref
    late = comp.cost_map(now=MON.replace(hour=12, minute=30), altitude_m=15.0)
    # What the wrong instant gets wrong, so the refusal is about something:
    assert "fence-a" in late.header["not_in_force"] and "fence-a" not in good.header["not_in_force"]
    assert "fence-b" in good.header["not_in_force"] and "fence-b" not in late.header["not_in_force"]
    cases = [(late, "clock 2026-10-05T12:30:00"),
             (comp.cost_map(altitude_m=15.0), "clock None"),
             (comp.cost_map(now=at, altitude_m=200.0), "altitude_m=200"),
             (comp.cost_map(now=at), "altitude_m=None")]
    for bad, word in cases:
        try:
            comp.compile_csp(mission, now=at, budget_tokens=1000, cost_map=bad)
        except ValueError as e:
            assert word in str(e), (word, str(e))
        else:
            raise AssertionError(f"compile_csp accepted a map of another instant or height ({word})")
        d = Path(tempfile.mkdtemp())
        try:
            comp.write_csp(d / "csp_g0.json", mission, now=at, budget_tokens=1000, cost_map=bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"write_csp wrote a CSP naming a map of another instant ({word})")
        assert not any(d.iterdir()), (word, sorted(os.listdir(d)))
    # Without a clock, the map drawn without one at the cruise height is the one.
    flat = comp.cost_map(altitude_m=15.0)
    assert comp.compile_csp(mission, now=None, budget_tokens=1000,
                            cost_map=flat).cost_map_ref == flat.ref


def test_the_cost_map_reports_hard_corridor_points_it_prices_finite():
    """The headline "misses 0 forbidden points" holds under the grant's
    corridor_outside = 50, which prices the outside of a HARD corridor too:
    the Shield refuses those points, the map lets a planner in at 50. The
    scorer counts them beside `missed` (hard_rule_priced_finite), with both
    nulls, so the headline is never read without them. Shown failing on the
    first 2026-10-08 version: no such count (reviewer's rerun with hard
    corridors requiring inf: 602 of 4181 shipped points)."""
    _need()
    pol = load_policy(SURVEY)                        # hard corridor, hard fence
    cm = K.rasterise_cost_map(pol, now=MONDAY_0800, altitude_m=15.0)
    ag = K.cost_map_agreement(cm, pol, now=MONDAY_0800, n_points=1500)
    m = ag["map"]
    assert m["missed"] == 0, ag
    assert m["hard_rule_priced_finite"] > 0, "the survey corridor is hard; its outside is priced 50"
    assert ag["n_forbidden_by_hard"] >= m["hard_rule_priced_finite"]
    assert ag["null_all_free"]["hard_rule_priced_finite"] == ag["n_forbidden_by_hard"], ag
    assert ag["null_all_inf"]["hard_rule_priced_finite"] == 0, ag
    # Each such point sits in a corridor_outside cell at 50, nowhere else:
    # every other hard layer is inf.
    finite_hard = {L["name"] for L in cm.header["cost_map"]["layers"]
                   if L["cost"] != "inf" and L["name"].endswith("_hard")}
    assert finite_hard == set(), finite_hard
    soft = Policy.model_validate({**json.loads(json.dumps(
        pol.model_dump(mode="json", exclude_none=True))), "constraints": [
        {**c.model_dump(mode="json", exclude_none=True), "constraint_type": "soft"}
        if c.type == "corridor" else c.model_dump(mode="json", exclude_none=True)
        for c in pol.constraints]})
    sag = K.cost_map_agreement(K.rasterise_cost_map(soft, now=MONDAY_0800, altitude_m=15.0),
                               soft, now=MONDAY_0800, n_points=1500)
    assert sag["map"]["hard_rule_priced_finite"] == 0, "a soft corridor priced 50 is what it says"


def test_a_keep_out_zone_is_drawn_with_its_margin_ring():
    """The Shield forbids a hard zone's margin ring as it forbids the zone. A
    map drawing the bare polygon (the review's mutation: rec.buffered ->
    rec.polygon) passed every other test here, because they sample too few
    points in the ring; this one samples only the ring."""
    _need()
    from shapely.geometry import Point
    from shapely.geometry import Polygon as SPoly
    from guardrail.shield import Shield
    pol = Policy.model_validate({"policy_id": "ring", "constraints": [
        {"id": "z", "type": "polygon_fence", "vertices": _sq(0, 0, 20, 20), "margin_m": 8.0}]})
    cm = K.rasterise_cost_map(pol, altitude_m=15.0, resolution_m=5.0)
    sh = Shield(pol, escalation=False)
    poly = SPoly([(0, 0), (20, 0), (20, 20), (0, 20)])
    rng = np.random.default_rng(1)
    n = 0
    while n < 400:
        x, y = (float(v) for v in rng.uniform(-8.5, 28.5, 2))
        d = poly.distance(Point(x, y))
        if not 0.0 < d < 7.9:
            continue
        row = next(r for r in sh.rule_status(State(x=x, y=y, up=15.0)) if r["id"] == "z")
        assert row["in_margin"] and row["breach"], (x, y, row)
        assert cm.cost_at(x, y) == math.inf, (x, y, d, cm.cost_at(x, y))
        n += 1


def test_the_shipped_policies_cost_map_report_misses_nothing():
    """The shipped-policy report (python -m guardrail.compiler costmap-report)
    as a test: 0 missed over every rasterisable policy, the qualifier
    counted, both nulls at their extremes. It catches the margin-ring
    mutation on its own (38 missed at 1000 points in review)."""
    _need()
    rep = K.cost_map_report(ROOT / "policies", n_points=1000)
    t = rep["totals"]
    assert t["n_policies_scored"] >= 20, t
    assert t["map"]["missed"] == 0, {r["file"]: r["map"]["missed"] for r in rep["rows"]
                                     if r["map"]["missed"]}
    assert t["null_all_free"]["missed"] == t["n_forbidden"] > 0
    assert t["null_all_inf"]["over_cost"] == t["n_allowed"] > 0
    assert t["map"]["over_cost"] < 0.5 * t["n_allowed"], t
    assert t["map"]["hard_rule_priced_finite"] == sum(
        t["hard_rule_priced_finite_by_policy"].values())
    print(f"      {t['n_points']} points over {t['n_policies_scored']} policies: missed "
          f"{t['map']['missed']} of {t['n_forbidden']}; hard-forbidden but priced finite "
          f"{t['map']['hard_rule_priced_finite']} of {t['n_forbidden_by_hard']} "
          f"({t['hard_rule_priced_finite_by_policy']}); over-cost {t['map']['over_cost']} "
          f"of {t['n_allowed']}")


def _two_switches(first_active: bool):
    """fence-s switched by two overlapping switches of opposite sense; the
    one LATER in policy order decides while both hold (Shield._Schedule)."""
    from guardrail.models import policy_from_raw
    a = {"id": "sw-wide", "type": "time_window_switch", "target_id": "fence-s",
         "active": first_active, "valid_time": _weekdays("08:00", "09:00", ("Mon",))}
    b = {"id": "sw-narrow", "type": "time_window_switch", "target_id": "fence-s",
         "active": not first_active, "valid_time": _weekdays("08:30", "08:45", ("Mon",))}
    return policy_from_raw({"policy_id": "two-switches", "constraints": [
        {"id": "fence-s", "type": "polygon_fence", "priority": "P0",
         "vertices": _sq(40, -5, 50, 5)}, a, b]}, runtime=False)


def test_the_last_switch_in_policy_order_decides_as_in_the_shield():
    """Of the switches on one rule that hold at an instant, the last in
    policy order decides (shield._Schedule.on). A clock where the FIRST
    decided passed every other test (the review's mutation M02): no policy
    there has two switches on one target. Both orders, every edge, against
    the Shield's own rule_status."""
    _need()
    for first_active in (False, True):
        pol = _two_switches(first_active)
        clock = K._RuleClock(pol)
        fence = next(c for c in pol.constraints if c.id == "fence-s")
        n = 0
        for hh, mm in ((8, 0), (8, 30), (8, 37), (8, 45), (9, 0)):
            for off in (-1, -0.05, 0, 0.05, 1, 30):
                when = MON.replace(hour=hh, minute=mm) + timedelta(seconds=off)
                shield = _shield_in_force(pol, when)["fence-s"]
                eps = timedelta(seconds=0.05)
                assert (clock.on(fence, when - eps) or clock.on(fence, when + eps)) == shield, \
                    (first_active, when, shield)
                dec = {r.id: r.decision for r in ConstraintCompiler(pol).compile_csp(
                    _mission(**HOT_MISSION), now=when, lookahead_s=0.05,
                    budget_tokens=1000).selection.rules}
                if shield:
                    assert dec["fence-s"] != "filtered_inactive", (first_active, when)
                n += 1
        # 08:37, inside both windows: the narrow switch (listed last) decides.
        mid = MON.replace(hour=8, minute=37)
        assert clock.on(fence, mid) == (not first_active), (first_active, clock.on(fence, mid))
        assert _shield_in_force(pol, mid)["fence-s"] == (not first_active)
        assert n == 30


def test_a_moving_zone_far_from_the_path_is_never_filtered_by_region():
    """A moving dynamic_nfz has no fixed place the compiler knows, so it is
    never filtered by region; the same zone standing still IS filtered. HOT's
    moving zone lies near the path, so filtering it again passed every other
    test (the review's mutation M11)."""
    _need()
    from guardrail.models import policy_from_raw
    far = _sq(500, 500, 510, 510)
    pol = policy_from_raw({"policy_id": "far-zones", "constraints": [
        {"id": "far-moving", "type": "dynamic_nfz", "vertices": far,
         "motion": {"vx_mps": 2.0, "vy_mps": 0.0, "yaw_rate_dps": 0.0}},
        {"id": "far-still", "type": "dynamic_nfz", "vertices": far}]}, runtime=False)
    csp = ConstraintCompiler(pol).compile_csp(_mission(tx=100.0, ty=0.0), now=None,
                                              budget_tokens=1000)
    dec = {r.id: r.decision for r in csp.selection.rules}
    assert dec["far-moving"] == "kept", dec
    assert dec["far-still"] == "filtered_out_of_region", dec


def test_a_runtime_type_with_no_place_in_the_by_type_order_raises():
    """build_prompt and the naive baseline walk the by-type order. A runtime
    type missing from it would vanish from their text without a word, so
    _by_type_order raises (the review's mutation M10, removing the raise,
    passed every other test: today every runtime type has its place)."""
    _need()
    from guardrail.models import RUNTIME_TYPES
    pol = Policy.model_validate({"policy_id": "stub", "constraints": [
        {"id": "f", "type": "polygon_fence", "vertices": _sq(0, 0, 5, 5)},
        {"id": "people", "type": "distance_envelope", "object_class": "people",
         "min_distance_m": 10.0}]})
    saved = K.RUNTIME_TYPES
    K.RUNTIME_TYPES = frozenset(RUNTIME_TYPES | {"distance_envelope"})
    try:
        ConstraintCompiler(pol)._sentences()
    except RuntimeError as e:
        assert "people" in str(e), str(e)
    else:
        raise AssertionError("a runtime type with no place in the order was dropped silently")
    finally:
        K.RUNTIME_TYPES = saved
    assert len(ConstraintCompiler(_hot())._sentences()) == len(HOT["constraints"])


def test_a_switch_target_names_its_switches_in_p0_in_force():
    """P0Entry.in_force read only the rule's own window: fence-b read
    "always" at 08:45 although sw-b-off turns it off Mondays 08:00-08:30.
    Now the switches and swaps acting on a rule follow its window. A P0
    switch is itself a P0 entry (param ref, its own window): its sentence is
    what tells the model when the target is off."""
    _need()
    pol = _hot()
    comp = ConstraintCompiler(pol)
    p0 = {e.id: e for e in comp.compile_csp(_mission(**HOT_MISSION),
                                            now=MON.replace(hour=8, minute=45),
                                            budget_tokens=1000).P0_constraints}
    assert p0["fence-b"].in_force == "always; switched off by sw-b-off (Mon 08:00-08:30)", \
        p0["fence-b"].in_force
    assert p0["fence-a"].in_force == ("Mon-Fri 07:30-17:30; switched off by sw-a-off "
                                      "(Mon 12:00-13:00)"), p0["fence-a"].in_force
    assert p0["corr-main"].in_force == "always; replaced by swap-main (Mon 09:00-10:00)", \
        p0["corr-main"].in_force
    assert p0["dyn"].in_force == "always"
    noon = {e.id: e for e in comp.compile_csp(_mission(**HOT_MISSION),
                                              now=MON.replace(hour=12, minute=30),
                                              budget_tokens=1000).P0_constraints}
    assert "fence-a" not in noon, "held off by sw-a-off at 12:30"
    assert noon["sw-a-off"].in_force == "Mon 12:00-13:00", noon["sw-a-off"]
    assert noon["sw-a-off"].geometry_ref.startswith("param:sw-a-off@")
    survey = ConstraintCompiler(load_policy(SURVEY)).compile_csp(_mission(), now=MONDAY_0800)
    assert {e.id: e.in_force for e in survey.P0_constraints}["nfz-school"] == \
        "Mon-Fri 07:30-17:30", "a rule nothing switches reads as before"


if __name__ == "__main__":
    if "--snapshot" in sys.argv[1:]:
        sys.exit(_print_snapshot())
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
