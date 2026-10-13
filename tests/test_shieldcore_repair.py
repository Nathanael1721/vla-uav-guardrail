"""The repair operator library (guardrail/repair.py) and its new modes.

Run:  python tests/test_shieldcore_repair.py

  * the default chain is the old chain, decision for decision, on every
    policy in policies/ (the old `_repair_chain` is copied below, verbatim);
  * every operator the chain considers is on the decision's ledger, with a
    result and, for a skip, the reason;
  * an operator written to the grant's three-argument signature registers,
    and `repair_order` puts a registered name to work;
  * repair_mode="first_feasible" applies the grant's selection: the
    earliest violation is the target, an operator whose repair is over
    theta (2.0 m lateral, 0.5 m vertical) does not win, a recovery is
    exempt, and an "ok" that leaves the target violated is a fail;
  * path_repair=True frees the standoff-wedge scenario (experiments/
    templates/subject_standoff.yaml, the "known failure" since 2026-08-31)
    with no P0 escape, detours round a zone projection stalls at, and never
    takes a detour that breaks another rule;
  * segment_check=True catches a zone thinner than one forecast step at
    12 m/s (in force at either end of the step, within its altitude band),
    and a corridor U-turn a forecast jumps across, without flagging a
    forecast that stays inside the corridor's round join.

Every new-mode test fails on 4418ae0 (the flags did not exist: TypeError),
and each states what the Shield does without its flag, measured.
"""
import functools
import math
import os
import random
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "tests", ROOT / "experiments"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np                                                      # noqa: E402

from guardrail import load_policy                                       # noqa: E402
from guardrail import repair as R                                       # noqa: E402
from guardrail.models import (Action4D, ObstacleClearance, Policy,      # noqa: E402
                              State, SubjectStandoff)
from guardrail.shield import Shield                                     # noqa: E402

FIX_MAP = ROOT / "tests" / "fixtures" / "citylife_occ_flightband_6to14.npz"


def _map():
    with np.load(FIX_MAP) as z:
        return {"occ": z["occ"], "res": float(z["res"]), "ox": float(z["ox"]),
                "oy": float(z["oy"])}


class _OldChain(Shield):
    """Shield._repair_chain as it stood at 4418ae0, verbatim but for the
    fourth return value the new signature carries."""

    def _repair_chain(self, tick, state, raw, skip, repairs, raw_violations=()):
        prev, tick.skip = tick.skip, frozenset(skip)
        try:
            fixed = self._repair_kinematic(raw, repairs)
            fixed = self._repair_altitude(state, fixed, repairs)
            for _ in range(3):
                fixed = self._repair_clearance(state, fixed, repairs)
                fixed = self._repair_standoff(state, fixed, repairs)
                fixed = self._repair_corridor(state, fixed, repairs)
                fixed = self._repair_geofence(state, fixed, repairs)
                fixed = self._repair_kinematic(fixed, repairs)
                if not self._check(state, fixed):
                    break
            blocking = self._check(state, fixed)
        finally:
            tick.skip = prev
        return fixed, repairs, blocking, []


# ------------------------------------------------------------------ the library

def test_the_registry_holds_every_operator_behind_one_protocol():
    names = ("KinematicClamp", "AltitudeClamp", "ClearanceProjection", "StandoffProjection",
             "CorridorProjection", "LateralProjection", "PathRepair")
    assert tuple(R.REGISTRY) == names, tuple(R.REGISTRY)
    for op in R.REGISTRY.values():
        assert isinstance(op, R.RepairOperator), op
        assert op.categories and isinstance(op.name, str)
    sh = Shield(load_policy(ROOT / "policies" / "demo_policy.yaml"))
    assert sh.repair_operators == R.REGISTRY and sh.repair_operators is not R.REGISTRY
    try:
        R.register(R.REGISTRY["AltitudeClamp"])
        raise AssertionError("a second AltitudeClamp was registered")
    except ValueError:
        pass
    try:
        R.register(object())
        raise AssertionError("an object with no repair() was registered")
    except TypeError:
        pass
    for bad in ("pipeline", ""):
        try:
            Shield(load_policy(ROOT / "policies" / "demo_policy.yaml"), repair_mode=bad)
            raise AssertionError(f"repair_mode {bad!r} accepted")
        except ValueError:
            pass


def _samples_for(pol, n, rng):
    out = []
    has_so = bool(pol.by_type(SubjectStandoff))
    for i in range(n):
        subj = None
        if has_so and i % 2 == 0:
            sx, sy = rng.uniform(-60, 60), rng.uniform(-60, 60)
            st = State(x=sx + rng.uniform(-14, 14), y=sy + rng.uniform(-14, 14),
                       up=rng.uniform(0, 30), yaw_deg=rng.uniform(-180, 180))
            subj = (sx, sy, rng.choice(["pedestrian", "car", "thing"]))
        else:
            st = State(x=rng.uniform(-70, 70), y=rng.uniform(-70, 70), up=rng.uniform(0, 30),
                       yaw_deg=rng.uniform(-180, 180))
        a = Action4D(vx=rng.uniform(-8, 8), vy=rng.uniform(-8, 8),
                     vz_up=rng.uniform(-4, 4), yaw_rate=rng.uniform(-3, 3))
        out.append((st, a, subj))
    return out


def test_the_default_chain_decides_as_the_old_chain_on_every_policy():
    """60 seeded ticks per policy in policies/ (the 28 test_check_contract
    hashes), stand-off subjects placed near the aircraft, the CityLife map
    under every clearance rule; plus 300 ticks of the 50-rule load near and
    among its fences. Whole decisions compared, FSM verdict included."""
    import json
    import bench_shield_50rules as B
    names = sorted(json.loads((ROOT / "tests" / "fixtures" / "check_baseline_digests.json")
                              .read_text(encoding="utf-8"))["digests"])
    smap = _map()
    rng = random.Random(20261012)
    n_touched = 0
    cases = [(name, load_policy(ROOT / "policies" / name)) for name in names]
    cases.append(("bench-50", B.fifty_rule_policy()))
    for name, pol in cases:
        kw = {"obstacle_map": smap} if pol.by_type(ObstacleClearance) else {}
        look = {} if name == "bench-50" else {"lookahead_s": 3.0, "dt": 0.5}
        new = Shield(pol.model_copy(deep=True), **look, **kw)
        old = _OldChain(pol.model_copy(deep=True), **look, **kw)
        pairs = ([(s, a, None) for s, a in B.samples("near", 150, 5) + B.samples("grid", 150, 5)]
                 if name == "bench-50" else _samples_for(pol, 60, rng))
        for k, (st, a, subj) in enumerate(pairs):
            for sh in (new, old):
                sh.set_subject(*subj) if subj else sh.set_subject(None)
            dn, do = new.filter(st, a, t=0.1 * k), old.filter(st, a, t=0.1 * k)
            assert dn.model_dump() == do.model_dump(), (name, k, dn.repairs, do.repairs)
            n_touched += bool(dn.repairs)
    assert n_touched > 500, f"only {n_touched} repaired ticks: the comparison saw too little"


def test_every_operator_considered_is_on_the_ledger_with_a_reason():
    """A tick heading into demo_policy's square: pass 0 runs the two clamps,
    pass 1 the five loop operators, LateralProjection is the one that acts,
    every skip says why, and the iteration count is what ran (1, not 3)."""
    sh = Shield(load_policy(ROOT / "policies" / "demo_policy.yaml"))
    d = sh.filter(State(x=0, y=15, up=4, yaw_deg=30.0), Action4D(vx=1.3))
    tr = d.trace
    assert tr["mode"] == "chain" and tr["iterations"] == 1 and tr["chain_runs"] == 1
    assert [p["pass"] for p in tr["passes"]] == [0, 1]
    assert [a["operator"] for a in tr["passes"][0]["attempts"]] == list(R.CHAIN_PRE)
    assert [a["operator"] for a in tr["passes"][1]["attempts"]] == list(R.CHAIN_LOOP)
    oks = [a for p in tr["passes"] for a in p["attempts"] if a["result"] == "ok"]
    assert [a["operator"] for a in oks] == ["LateralProjection"]
    assert oks[0]["repairs"] == [r.operator for r in d.repairs] == ["GeofenceSlide"]
    skips = [a for p in tr["passes"] for a in p["attempts"] if a["result"] == "skip"]
    assert len(skips) == 6 and all(a["reason"] for a in skips), skips
    why = {a["operator"]: a["reason"] for a in skips}
    assert why["StandoffProjection"].startswith("no subject set")
    assert "no altitude violation open on the raw action" == why["AltitudeClamp"]
    # A tick the chain cannot fix in one pass records each pass it ran, and
    # a brake fallback lands in the last one.
    sh2 = Shield(load_policy(ROOT / "policies" / "demo_policy.yaml"))
    d2 = sh2.filter(State(x=0, y=15, up=4), Action4D(vx=float("nan")))
    first = d2.trace["passes"][0]["attempts"][0]
    assert (first["operator"], first["result"], first["iterations"]) == ("Sanitise", "ok", 0)


def test_a_hook_that_changes_the_action_without_logging_is_ok():
    """A `_repair_*` override that changes the action but logs no Repair is
    still the operator that acted: "ok" on the ledger, not "skip"."""
    class Quiet(Shield):
        def _repair_geofence(self, state, a, repairs):
            return Action4D(vx=0.0, vy=a.vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
    d = Quiet(load_policy(ROOT / "policies" / "demo_policy.yaml")).filter(
        State(x=0, y=15, up=4), Action4D(vx=1.3))
    lat = [a for p in d.trace["passes"] for a in p["attempts"]
           if a["operator"] == "LateralProjection"]
    assert lat[0]["result"] == "ok" and d.emitted.vx == 0.0, lat


def test_path_repair_sits_right_after_the_lateral_projection():
    assert R.with_path_repair(R.CHAIN_LOOP) == (
        "ClearanceProjection", "StandoffProjection", "CorridorProjection",
        "LateralProjection", "PathRepair", "KinematicClamp")
    assert R.with_path_repair(R.FIRST_FEASIBLE) == (
        "KinematicClamp", "AltitudeClamp", "LateralProjection", "PathRepair",
        "CorridorProjection", "StandoffProjection", "ClearanceProjection")
    d = Shield(_square_policy(), escalation=False, path_repair=True).filter(
        State(x=3.0, y=0.5, up=10.0), Action4D(vx=2.0))
    assert [a["operator"] for a in d.trace["passes"][1]["attempts"]] == \
        list(R.with_path_repair(R.CHAIN_LOOP))


class _Slower:
    """An operator written to the grant's signature exactly: three arguments,
    no `ctx`, no `categories`. It takes 10 % off the horizontal speed, which
    does not clear a head-on approach."""
    name = "Slower"

    def repair(self, action, violation, ir):
        return Action4D(vx=0.9 * action.vx, vy=0.9 * action.vy, vz_up=action.vz_up,
                        yaw_rate=action.yaw_rate)


def test_an_operator_with_the_grants_signature_registers_and_runs_where_named():
    """`register` wraps a three-argument operator (it covers every category,
    and the change it makes is logged in its name); `repair_order` runs a
    registered name in either mode, and an unregistered name is refused."""
    op = R.register(_Slower())
    try:
        assert isinstance(op, R.GrantOperator) and op.categories == R.CATEGORIES
        assert isinstance(op, R.RepairOperator)
        st, a = State(x=3.0, y=0.0, up=10.0), Action4D(vx=2.0)
        d = Shield(_square_policy(), escalation=False,
                   repair_order=("Slower", "KinematicClamp")).filter(st, a)
        p1 = d.trace["passes"][1]["attempts"]
        assert [(x["operator"], x["result"]) for x in p1[:2]] == [("Slower", "ok"),
                                                                   ("KinematicClamp", "skip")]
        # 10 % slower is still head-on: every pass runs it again (the same
        # detail each time, one Repair), and the chain ends in a stop.
        assert [x["iterations"] for q in d.trace["passes"] for x in q["attempts"]
                if x["operator"] == "Slower"] == [1, 2, 3]
        assert [r.operator for r in d.repairs] == ["Slower", "Brake"], d.repairs
        assert d.trace["repair_iterations"] == [1, None]
        try:
            Shield(_square_policy(), repair_order=("Slower", "NoSuchOperator"))
            raise AssertionError("an unregistered name was accepted")
        except ValueError:
            pass
    finally:
        del R.REGISTRY["Slower"]
    # The default Shield never runs it: the stacks name it nowhere.
    assert "Slower" not in Shield(_square_policy()).repair_operators


def test_a_subclass_override_of_a_repair_hook_is_the_one_that_runs():
    calls = []

    class Spy(Shield):
        def _repair_geofence(self, state, a, repairs):
            calls.append(a)
            return super()._repair_geofence(state, a, repairs)
    d = Spy(load_policy(ROOT / "policies" / "demo_policy.yaml")).filter(
        State(x=0, y=15, up=4), Action4D(vx=1.3))
    assert calls and [r.operator for r in d.repairs] == ["GeofenceSlide"]


# ------------------------------------------------------------- first feasible

def _square_policy(margin=1.0, extra=()):
    return Policy.model_validate({"policy_id": "sq", "constraints": [
        {"id": "kin", "type": "kinematic_envelope", "speed_max_mps": 5.0,
         "climb_rate_max_mps": 3.0, "yaw_rate_max_dps": 90.0},
        {"id": "sq", "type": "polygon_fence", "margin_m": margin,
         "vertices": [{"x": 10, "y": -10}, {"x": 30, "y": -10}, {"x": 30, "y": 10},
                      {"x": 10, "y": 10}]}, *extra]})


def test_first_feasible_will_not_apply_a_repair_over_theta():
    """Head-on at the square from 6 m out of its margin at 4 m/s: the raw
    forecast goes 13 m into the ring. The chain slides (GeofenceSlide, 13 m,
    left for the FSM to judge). The grant's selection refuses a repair whose
    magnitude is over theta (2.0 m lateral): LateralProjection fails, no
    operator wins, and the fallback stops (a stop is legal here)."""
    st, a = State(x=3.0, y=0.0, up=10.0), Action4D(vx=4.0)
    chain = Shield(_square_policy(), escalation=False).filter(st, a)
    assert [r.operator for r in chain.repairs] == ["GeofenceSlide"] and not chain.braked
    assert chain.repairs[0].magnitude_m > 2.0
    ff = Shield(_square_policy(), escalation=False, repair_mode="first_feasible").filter(st, a)
    assert ff.braked and ff.emitted == Action4D(), ff
    lat = [x for p in ff.trace["passes"] for x in p["attempts"]
           if x["operator"] == "LateralProjection"]
    assert lat and lat[0]["result"] == "fail" and "over theta" in lat[0]["reason"], lat
    assert not ff.emitted_violations


def test_first_feasible_applies_the_first_operator_that_clears_a_shallow_clip():
    """A forecast that only clips the ring (0.5 m deep): the same slide the
    chain makes is feasible, and wins in iteration 1."""
    pol = load_policy(ROOT / "policies" / "demo_policy.yaml")
    st, a = State(x=0, y=15, up=4), Action4D(vx=1.3)
    chain = Shield(pol.model_copy(deep=True), escalation=False).filter(st, a)
    ff = Shield(pol.model_copy(deep=True), escalation=False,
                repair_mode="first_feasible").filter(st, a)
    assert ff.emitted == chain.emitted and [r.operator for r in ff.repairs] == ["GeofenceSlide"]
    p1 = ff.trace["passes"][-1]
    assert p1["pass"] == 1 and p1["attempts"][-1]["operator"] == "LateralProjection"
    assert p1["attempts"][-1]["result"] == "ok"
    # The clamps are skipped because they cannot repair a geofence
    # violation, not because they found nothing of their own to do.
    assert [(x["result"], x["reason"]) for x in p1["attempts"][:-1]] == \
        [("skip", "not a geofence violation (nfz-square)")] * 2, p1["attempts"]


_ALT = {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5.0, "alt_max_m": 50.0}


def test_first_feasible_targets_the_earliest_violation_first():
    """Sinking toward the 5 m floor (breaks it at t + 1.1 s, 0.4 m deep by
    t + 5 s) while drifting into the square's ring (t + 2.6 s, 0.5 m deep):
    iteration 1 targets the floor and AltitudeClamp wins; iteration 2
    targets the zone and LateralProjection wins. Both repairs are within
    theta, and the result re-checks clean."""
    pol = _square_policy(extra=[_ALT])
    st, a = State(x=8.5, y=0.0, up=5.1), Action4D(vx=0.2, vy=2.0, vz_up=-0.1)
    sh = Shield(pol, escalation=False, repair_mode="first_feasible")
    assert [(v.rule_id, round(v.predicted_at_s, 1)) for v in sh._check(st, a)] == \
        [("alt", 1.1), ("sq", 2.6)]
    d = sh.filter(st, a)
    wins = [(p["pass"], x["operator"]) for p in d.trace["passes"] for x in p["attempts"]
            if x["result"] == "ok"]
    assert wins == [(1, "AltitudeClamp"), (2, "LateralProjection")], wins
    assert [r.operator for r in d.repairs] == ["AltitudeFix", "GeofenceSlide"]
    assert not d.braked and not d.emitted_violations


def test_first_feasible_holds_a_vertical_repair_to_the_vertical_theta():
    """Sinking at 0.4 m/s from 5.1 m: the forecast ends 1.9 m under the 5 m
    floor. The chain applies the AltitudeFix; first-feasible holds it to the
    vertical theta (0.5 m, not the lateral 2.0 m), refuses it, and the
    fallback stops (a hover at 5.1 m is legal)."""
    st, a = State(x=0.0, y=0.0, up=5.1), Action4D(vx=0.5, vz_up=-0.4)
    chain = Shield(_square_policy(extra=[_ALT]), escalation=False).filter(st, a)
    assert [(r.operator, r.axis) for r in chain.repairs] == [("AltitudeFix", "vertical")]
    assert 0.5 < chain.repairs[0].magnitude_m < 2.0 and not chain.braked
    ff = Shield(_square_policy(extra=[_ALT]), escalation=False,
                repair_mode="first_feasible").filter(st, a)
    alt = [x for p in ff.trace["passes"] for x in p["attempts"]
           if x["operator"] == "AltitudeClamp"]
    assert alt[0]["result"] == "fail" and "over theta (vertical)" in alt[0]["reason"], alt
    assert ff.braked and [r.operator for r in ff.repairs] == ["Brake"]


def test_first_feasible_exempts_a_recovery_from_theta():
    """Inside the square, 6 m from its ring: GeofenceEscape's magnitude (the
    6 m depth) is three times theta, but a recovery is exempt (the
    reference's "brake while inside = deadlock"), so it wins."""
    st, a = State(x=15.0, y=0.0, up=10.0), Action4D(vx=0.5)
    ff = Shield(_square_policy(), escalation=False, repair_mode="first_feasible").filter(st, a)
    assert [(r.operator, r.recovery) for r in ff.repairs] == [("GeofenceEscape", True)]
    assert ff.repairs[0].magnitude_m > 2.0 and not ff.braked and not ff.emitted_violations


def test_first_feasible_fails_an_ok_that_leaves_the_target_violated():
    """An operator that says "ok" but whose action still breaks the target
    (10 % slower, still head-on into the square) is a fail and is not
    applied: nothing else can repair, so the fallback stops, and the
    decision carries the Brake alone."""
    R.register(_Slower())
    try:
        st, a = State(x=3.0, y=0.0, up=10.0), Action4D(vx=2.0)
        d = Shield(_square_policy(), escalation=False, repair_mode="first_feasible",
                   repair_order=("Slower",)).filter(st, a)
    finally:
        del R.REGISTRY["Slower"]
    att = d.trace["passes"][1]["attempts"][0]
    assert (att["operator"], att["result"]) == ("Slower", "fail"), att
    assert "sq still violated" in att["reason"]
    assert d.braked and [r.operator for r in d.repairs] == ["Brake"], d.repairs


# ------------------------------------------------------------------ PathRepair

def _wedge(flags):
    import sweep_scenarios as S
    import test_sweep as TS
    orig = S.Shield
    S.Shield = functools.partial(Shield, **flags)
    try:
        spec = TS._with(TS.BY_ID["standoff-wedge"], escalation=False)
        rows, extra = S.run_headless(spec, TS.DEFAULTS, 0)
    finally:
        S.Shield = orig
    pol = load_policy(ROOT / "policies" / "sitl_pedestrian.yaml")
    p0 = {c.id for c in pol.constraints if c.priority == "P0"}
    escapes = sum(1 for r in rows if any(
        (v["rule_id"] if isinstance(v, dict) else v) in p0
        for v in r.get("emitted_violations") or []))
    ops = {}
    for r in rows:
        for rp in r["repairs"]:
            k = rp["operator"] if isinstance(rp, dict) else rp
            ops[k] = ops.get(k, 0) + 1
    return extra, escapes, ops


def test_path_repair_frees_the_standoff_wedge_with_no_p0_escape():
    """The subject stands on the route 20 m ahead; StandoffHold removes the
    whole command and the aircraft hovers at (0, 0) for the 150 s episode,
    300 repairs, never reaching the goal (the scenario's known failure). With
    PathRepair the projected hover is replaced by a detour round a 12 m
    circle (ring 10 m + its 2 m soft margin), the goal is reached, the range
    never goes under 9.99 m, and no tick flies a P0 violation. FSM off, as
    tests/test_sweep.py flies the LEGACY set: with the FSM on, the detour's
    magnitude (the 10 m the raw forecast went into the ring) is over theta
    and the FSM goes X1 > G3 - theta's call, not the operator's."""
    extra, escapes, ops = _wedge({})
    assert (extra["final"]["x"], extra["final"]["y"]) == (0.0, 0.0)
    assert extra.get("reached_goal") is False and ops == {"StandoffHold": 300}, (extra, ops)
    extra, escapes, ops = _wedge({"path_repair": True})
    assert extra.get("reached_goal") is True, extra
    assert escapes == 0, escapes
    assert extra["min_range_to_subject"] >= 9.99, extra["min_range_to_subject"]
    assert ops.get("PathRepair", 0) > 0, ops


def test_path_repair_detours_round_a_zone_projection_slides_along():
    """Head-on at the square's middle, 6 m outside its margin: the slide
    turns the command 90 degrees (zero progress toward where the pilot is
    going). PathRepair aims for a corner of the margin ring's coarse ring,
    pushed 1 m out, at the commanded speed: 0.80 of its 2 m/s northward
    instead of none. The detour re-checks clean, and its waypoint is logged
    in body frame."""
    st, a = State(x=3.0, y=0.5, up=10.0, yaw_deg=0.0), Action4D(vx=2.0)
    base = Shield(_square_policy(), escalation=False).filter(st, a)
    assert [r.operator for r in base.repairs] == ["GeofenceSlide"]
    assert base.emitted.vx < 1e-9                       # no progress north
    d = Shield(_square_policy(), escalation=False, path_repair=True).filter(st, a)
    pr = [r for r in d.repairs if r.operator == "PathRepair"]
    assert pr and "body frame" in pr[0].detail and pr[0].axis == "lateral"
    assert d.emitted.vx > 0.75 and not d.emitted_violations, d
    assert abs(math.hypot(d.emitted.vx, d.emitted.vy) - 2.0) < 1e-9


def test_path_repair_offers_only_corners_it_can_see():
    """The square's detour corners are its margin ring's coarse ring pushed
    1 m out: (8, +-12) and (32, +-12). From (3, 0.5) the far two lie behind
    the zone (the straight leg would cross it), so only the near two are
    offered."""
    sh = Shield(_square_policy(), escalation=False, path_repair=True)
    pr = R.PathRepair()
    assert sorted(pr._zone_corners(sh, sh.ir.fences[0])) == \
        [(8.0, -12.0), (8.0, 12.0), (32.0, -12.0), (32.0, 12.0)]
    wps, depth = pr._zone_waypoints(sh, State(x=3.0, y=0.5, up=10.0), Action4D(vx=2.0), "sq")
    assert sorted(wps) == [(8.0, -12.0), (8.0, 12.0)], wps
    assert abs(depth - 4.0) < 1e-9


def test_path_repair_never_takes_a_detour_that_breaks_another_rule():
    """Two more zones flank the approach, one on each detour leg: flying at
    either near corner of the square would enter one within the horizon.
    PathRepair checks each candidate against every rule and takes none;
    the decision is the one made without it."""
    flank = [{"id": f"flank-{n}", "type": "polygon_fence", "margin_m": 1.0,
              "vertices": [{"x": 4, "y": lo}, {"x": 8, "y": lo}, {"x": 8, "y": hi},
                           {"x": 4, "y": hi}]} for n, lo, hi in (("n", 3, 8), ("s", -8, -3))]
    st, a = State(x=3.0, y=0.0, up=10.0), Action4D(vx=2.0)
    base = Shield(_square_policy(extra=flank), escalation=False).filter(st, a)
    d = Shield(_square_policy(extra=flank), escalation=False, path_repair=True).filter(st, a)
    pr = [x for p in d.trace["passes"] for x in p["attempts"] if x["operator"] == "PathRepair"]
    assert pr and all(x["result"] == "skip" for x in pr), pr
    assert pr[0]["reason"] == "no detour waypoint re-checks clean", pr[0]
    assert "PathRepair" not in [r.operator for r in d.repairs]
    assert d.model_dump() == base.model_dump()


def test_path_repair_passes_a_ring_at_the_tangent_of_its_soft_margin():
    """Subject 20 m ahead, ring 10 m, soft margin 2 m: the two waypoints are
    the tangent points of the 12 m circle, (12.8, +-9.6), each 12 m from the
    subject. With a soft margin under 1 m the circle is the ring + 1 m."""
    def wps(soft):
        pol = Policy.model_validate({"policy_id": "so", "constraints": [
            {"id": "so", "type": "subject_standoff", "min_range_m": 10.0,
             "soft_margin_m": soft, "subject_class": "*"}]})
        sh = Shield(pol, escalation=False)
        sh.set_subject(20.0, 0.0, "car")
        return R.PathRepair()._standoff_waypoints(sh, State(x=0, y=0, up=10), Action4D(vx=2.0))[0]
    got = wps(2.0)
    assert [(round(x, 9), round(y, 9)) for x, y in got] == [(12.8, 9.6), (12.8, -9.6)], got
    assert all(abs(math.hypot(x - 20.0, y) - 12.0) < 1e-9 for x, y in got)
    assert all(abs(math.hypot(x - 20.0, y) - 11.0) < 1e-9 for x, y in wps(0.5))


# --------------------------------------------------------------- segment check

def _wall_policy():
    """A wall 0.1 m thick with a 0.2 m margin: its ring is 0.5 m across,
    less than the 1.2 m a 12 m/s forecast covers per 0.1 s step."""
    return Policy.model_validate({"policy_id": "wall", "constraints": [
        {"id": "wall", "type": "polygon_fence", "margin_m": 0.2,
         "vertices": [{"x": 4.05, "y": -50}, {"x": 4.15, "y": -50}, {"x": 4.15, "y": 50},
                      {"x": 4.05, "y": 50}]}]})


def test_the_segment_check_catches_a_zone_thinner_than_one_step():
    st, a = State(x=0.0, y=0.0, up=10.0), Action4D(vx=12.0)
    plain = Shield(_wall_policy(), escalation=False)
    assert plain._check(st, a) == [], "the poses at x = 3.6 and 4.8 straddle the ring"
    seg = Shield(_wall_policy(), escalation=False, segment_check=True)
    vs = seg._check(st, a)
    assert [v.rule_id for v in vs] == ["wall"], vs
    assert abs(vs[0].predicted_at_s - 3.85 / 12.0) < 1e-9, vs[0]
    assert "segment" in vs[0].detail
    d = seg.filter(st, a)
    assert d.repairs and d.repairs[0].operator == "GeofenceSlide"
    # Sized by the segment's own depth: the inside part runs 3.85 -> 4.35 m,
    # and its midpoint is 0.25 m inside the ring.
    assert abs(d.repairs[0].magnitude_m - 0.25) < 1e-9, d.repairs[0]
    assert not d.emitted_violations and seg._check(st, d.emitted) == []


def _wall(**extra):
    return Policy.model_validate({"policy_id": "wall", "constraints": [
        {"id": "wall", "type": "polygon_fence", "margin_m": 0.2, **extra,
         "vertices": [{"x": 4.05, "y": -50}, {"x": 4.15, "y": -50}, {"x": 4.15, "y": 50},
                      {"x": 4.05, "y": 50}]}]})


def test_the_segment_check_reads_the_zones_altitude_band():
    """The same thin wall, 5 m tall: a segment 10 m up passes over it; one
    4 m up goes through."""
    a = Action4D(vx=12.0)
    seg = Shield(_wall(altitude_ceiling_m=5.0), escalation=False, segment_check=True)
    assert seg._check(State(x=0.0, y=0.0, up=10.0), a) == []
    assert [v.rule_id for v in seg._check(State(x=0.0, y=0.0, up=4.0), a)] == ["wall"]


def test_the_segment_check_judges_a_step_in_which_the_zone_comes_into_force():
    """The wall comes into force at 07:30:00, the clock at 07:29:59.63: it is
    off at the pose before the ring (t + 0.3 s, both sides of the instant)
    and on at the pose after it (t + 0.4 s). A segment is judged when the
    rule is in force at EITHER end, so the step is caught - a violation the
    window edge caused (time-window). The pose checks alone see nothing."""
    st, a = State(x=0.0, y=0.0, up=10.0), Action4D(vx=12.0)
    now = datetime(2026, 10, 12, 7, 29, 59, 630000)
    pol = _wall(valid_time={"recurrence": {"start_time": "07:30", "end_time": "17:30"}})
    assert Shield(pol, escalation=False, now=lambda: now).filter(st, a).violations == []
    d = Shield(pol.model_copy(deep=True), escalation=False, segment_check=True,
               now=lambda: now).filter(st, a)
    assert [(v.rule_id, round(v.predicted_at_s, 9)) for v in d.violations] == \
        [("wall", round(3.85 / 12.0, 9))], d.violations
    assert d.trace["window_edge"] == ["wall"]


def test_the_segment_check_does_not_flag_a_forecast_inside_a_round_join():
    """At the L corridor's outer corner the true tube's edge is a circle
    (radius = half-width) round the bend. GEOS builds that join from chords
    whose middles lie 0.12 % inside the circle; a forecast at 99.95 % of the
    half-width, in the middle of a chord, is inside the corridor but outside
    that polygon. The tube the segment check reads is buffered by
    half-width / cos(pi/64) so it contains the true tube: no violation."""
    from shapely.geometry import LineString, Point
    pol = Policy.model_validate({"policy_id": "L", "constraints": [
        {"id": "L", "type": "corridor", "width_m": 2.0, "altitude_floor_m": 0,
         "altitude_ceiling_m": 50,
         "centerline": [{"x": 0, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 10}]}]})
    th = math.radians(-90.0 + 90.0 / 32.0)             # the middle of the first chord
    x, y = 20.0 + 0.9995 * math.cos(th), 0.9995 * math.sin(th)
    line = LineString([(0, 0), (20, 0), (20, 10)])
    assert line.distance(Point(x, y)) < 1.0                          # in the corridor
    assert not line.buffer(1.0, quad_segs=16).covers(Point(x, y))    # not in GEOS's join
    st = State(x=x, y=y, up=10.0)
    a = Action4D(vx=-0.001 * math.sin(th), vy=0.001 * math.cos(th))  # along the edge
    assert Shield(pol, escalation=False)._check(st, a) == []
    assert Shield(pol, escalation=False, segment_check=True)._check(st, a) == []


def test_the_segment_check_catches_a_forecast_jumping_a_corridor_u_turn():
    """Two forecast poses on the two arms of a U, each inside its half-width;
    the straight line between them crosses 9 m of open air."""
    pol = Policy.model_validate({"policy_id": "u", "constraints": [
        {"id": "u", "type": "corridor", "width_m": 2.0, "altitude_floor_m": 0,
         "altitude_ceiling_m": 50,
         "centerline": [{"x": 0, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 10},
                        {"x": 0, "y": 10}]}]})
    st, a = State(x=10.0, y=0.0, up=10.0), Action4D(vy=20.0)
    assert Shield(pol, lookahead_s=0.5, dt=0.5, escalation=False)._check(st, a) == []
    vs = Shield(pol, lookahead_s=0.5, dt=0.5, escalation=False,
                segment_check=True)._check(st, a)
    assert [v.rule_id for v in vs] == ["u"] and "leaves the corridor" in vs[0].detail, vs
    assert 0.0 < vs[0].predicted_at_s < 0.5


def _mode_timing(pol, flags, pairs):
    """(_check median, filter() p99) in ms on a fresh Shield."""
    sh = Shield(pol, **flags)
    ms_c, ms_f = [], []
    for st, a in pairs:
        t0 = time.perf_counter()
        sh._check(st, a)
        ms_c.append((time.perf_counter() - t0) * 1e3)
    for k, (st, a) in enumerate(pairs[:100]):
        t0 = time.perf_counter()
        sh.filter(st, a, t=0.1 * k)
        ms_f.append((time.perf_counter() - t0) * 1e3)
    return statistics.median(ms_c), sorted(ms_f)[98]


def test_every_new_mode_keeps_the_50_rule_budget():
    """Median _check <= 5 ms and p99 filter() <= 100 ms on the 50-rule load,
    static and 48 moving zones, with each new mode on and all three at once
    (the bounds test_shield.py applies to the default, with its 15 ms
    `_check` bound on a shared CI runner). A p99 over 100 ticks is one
    sample in a hundred, which a burst of load on a shared machine can
    supply on its own (a reviewer's loaded desktop read 79 ms for all three
    modes on the moving zones, against 24 ms quiet), so a configuration
    over a bound is measured once more and judged on the better run; one
    that is slow both times fails. The figures are printed."""
    import bench_shield_50rules as B
    from test_shield import _fifty_variant
    pairs = B.samples("near", 120, seed=3)
    check_bound = 15.0 if os.environ.get("GITHUB_ACTIONS") == "true" else 5.0
    modes = ({"repair_mode": "first_feasible"}, {"path_repair": True},
             {"segment_check": True},
             {"repair_mode": "first_feasible", "path_repair": True, "segment_check": True})
    for flags in modes:
        for label, pol in (("static", B.fifty_rule_policy()),
                           ("moving", _fifty_variant("moving"))):
            med, p99 = _mode_timing(pol, flags, pairs)
            if med > check_bound or p99 > 100.0:
                med2, p992 = _mode_timing(pol, flags, pairs)
                med, p99 = min(med, med2), min(p99, p992)
            print(f"      {'+'.join(f'{k}={v}' for k, v in flags.items())} {label}: "
                  f"_check median {med:.2f} ms, filter() p99 {p99:.1f} ms")
            assert med <= check_bound, \
                f"{flags} {label}: _check median {med:.2f} ms > {check_bound:g} ms"
            assert p99 <= 100.0, f"{flags} {label}: filter() p99 {p99:.1f} ms > 100 ms"


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
