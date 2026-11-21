"""The repair operator library (guardrail/repair.py) and its new modes.

Run:  python tests/test_shieldcore_repair.py

  * the default chain is the old chain, decision for decision, on every
    policy in policies/ (the old `_repair_chain` is copied below, verbatim);
  * every operator the chain considers is on the decision's ledger, with a
    result and, for a skip, the reason;
  * an operator written to the grant's three-argument signature registers,
    and `repair_order` puts a registered name to work;
  * every operator is handed, and hands back, the BODY-frame action at the
    tick's heading (WP3-04, since 2026-11-21), in chain and first-feasible
    mode, PathRepair included; a grant-signature operator acts on the body
    frame; the interface's rotation pair is lossless on its own results;
  * repair_mode="first_feasible" applies the grant's selection: the
    earliest violation is the target, an operator whose repair is over
    theta (2.0 m lateral, 0.5 m vertical; summed per axis with the tick's
    other repairs, as the FSM sums them) does not win, a recovery is
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

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

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
        # PathRepair joined the default chain on 2026-11-17 (the grant's
        # priority order); the chain without it is still the old chain.
        new = Shield(pol.model_copy(deep=True), path_repair=False, **look, **kw)
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
    # PathRepair in the default loop since 2026-11-17, right after the
    # lateral projection; here it skips (the slide keeps the progress).
    assert [a["operator"] for a in tr["passes"][1]["attempts"]] ==         list(R.with_path_repair(R.CHAIN_LOOP))
    oks = [a for p in tr["passes"] for a in p["attempts"] if a["result"] == "ok"]
    assert [a["operator"] for a in oks] == ["LateralProjection"]
    assert oks[0]["repairs"] == [r.operator for r in d.repairs] == ["GeofenceSlide"]
    skips = [a for p in tr["passes"] for a in p["attempts"] if a["result"] == "skip"]
    assert len(skips) == 7 and all(a["reason"] for a in skips), skips
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



def _zones(sh):
    """`sh` with zone detours in chain mode as well (PathRepair detours round
    stand-off rings only by default, since 2026-11-17)."""
    sh.repair_operators["PathRepair"] = R.PathRepair(chain_categories=("geofence", "standoff"))
    return sh

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


def test_path_repair_is_in_the_default_chain_in_the_grants_order():
    """Since 2026-11-17 a Shield built with no flag runs the grant's priority
    order (Safety Shield p3: AltitudeClamp -> LateralProjection -> PathRepair):
    PathRepair right after the lateral projection in every pass, and the
    audit's flags say so. path_repair=False is the chain flown before."""
    sh = Shield(_square_policy(), escalation=False)
    assert sh.path_repair is True
    d = sh.filter(State(x=3.0, y=0.5, up=10.0), Action4D(vx=2.0))
    ops = [a["operator"] for a in d.trace["passes"][1]["attempts"]]
    assert ops == list(R.with_path_repair(R.CHAIN_LOOP)), ops
    assert ops.index("LateralProjection") + 1 == ops.index("PathRepair")
    assert R.CHAIN_PRE.index("AltitudeClamp") == 1          # before the loop
    off = Shield(_square_policy(), escalation=False, path_repair=False).filter(
        State(x=3.0, y=0.5, up=10.0), Action4D(vx=2.0))
    assert "PathRepair" not in [a["operator"] for a in off.trace["passes"][1]["attempts"]]
    assert isinstance(R.REGISTRY["PathRepair"], R.RepairOperator)   # the Protocol
    # In chain mode it detours round stand-off rings only; a zone is left to
    # the projection (zone detours: PathRepair(chain_categories=...)).
    assert R.REGISTRY["PathRepair"].chain_categories == ("standoff",)
    d = Shield(_square_policy(), escalation=False).filter(
        State(x=5.0, y=9.0, up=10.0), Action4D(vx=2.0))
    pr = [x for p in d.trace["passes"] for x in p["attempts"] if x["operator"] == "PathRepair"]
    assert pr and "standoff only" in pr[0]["reason"], pr
    assert [r.operator for r in d.repairs] == ["GeofenceSlide"], d.repairs


def test_path_repair_magnitude_is_on_thetas_decided_depth_basis():
    """magnitude_m is on theta's DECIDED basis, reading A (2026-11-02,
    fsm.DEFAULT_THETA_HORIZON_S None): a forecast depth in metres - the depth
    the raw forecast reached into the ring the detour avoids, or the summed
    magnitudes of the projections it supersedes, whichever is larger. So
    theta never reads less on a detour tick than on the projection tick it
    replaces, and a raw forecast deeper than theta is capped as a projection
    would be. The one-period |emitted - raw| x 0.1 s (the reading fsm.py
    rejects as the default: at the shipped caps it never reaches theta) is
    only reported in the detail."""
    from guardrail import fsm as F
    assert F.DEFAULT_THETA_HORIZON_S is None
    assert R.PathRepair.PERIOD_S == F.THETA_HORIZON_S
    raw, em = Action4D(vx=20.0), Action4D(vx=3.0, vy=4.0)       # cap 5 m/s
    # the one-period helper: raw at 5 m/s is (5, 0), |(3, 4) - (5, 0)| x 0.1 s
    assert abs(R.PathRepair.magnitude(em, raw) - math.sqrt(20.0) * 0.1) < 1e-12
    assert R.PathRepair.magnitude(Action4D(vx=2.0), Action4D(vx=2.0)) == 0.0

    class _R:
        def __init__(self, m):
            self.magnitude_m = m
    assert R.PathRepair.depth_magnitude(0.8, [_R(0.5)]) == 0.8
    assert R.PathRepair.depth_magnitude(0.3, [_R(0.5), _R(0.4)]) == 0.9
    assert R.PathRepair.depth_magnitude(2.5, []) == 2.5          # over theta: capped
    st, a = State(x=5.0, y=9.0, up=10.0, yaw_deg=0.0), Action4D(vx=2.0)
    sh = _zones(Shield(_square_policy(), escalation=False))
    d = sh.filter(st, a)
    pr = [r for r in d.repairs if r.operator == "PathRepair"]
    assert len(pr) == 1, d.repairs
    base = Shield(_square_policy(), escalation=False, path_repair=False).filter(st, a)
    slide = sum(r.magnitude_m for r in base.repairs if r.operator == "GeofenceSlide")
    zone = next(v.rule_id for v in base.violations if v.category == "geofence")
    depth = R.PathRepair()._zone_waypoints(sh, st, a, zone)[1]
    assert abs(pr[0].magnitude_m - max(depth, slide)) < 1e-9, (pr[0], depth, slide)
    assert pr[0].magnitude_m >= slide - 1e-12                    # theta reads no less
    assert "one-period change" in pr[0].detail, pr[0].detail
    assert "supersedes GeofenceSlide" in pr[0].detail, pr[0].detail
    assert [r.operator for r in d.repairs] == ["PathRepair"], d.repairs
    led = [x for p in d.trace["passes"] for x in p["attempts"]
           if x["operator"] == "LateralProjection"]
    assert led and led[0]["result"] == "ok"                      # still on the ledger
    assert not d.emitted_violations and d.emitted.vx > 1.0, d


def test_path_repair_takes_no_detour_without_a_position_fix_or_during_a_recovery():
    """The detour of test_path_repair_detours_round_a_zone_projection_slides_
    along is not taken when the state says the position fix is lost (fix_ok
    False: a GPS denial, the position frozen or dead-reckoned) - the slide's
    push away keeps the margin for the drift - nor while a recovery is under
    way (the aircraft already inside a rule)."""
    st, a = State(x=5.0, y=9.0, up=10.0, yaw_deg=0.0), Action4D(vx=2.0)
    base = Shield(_square_policy(), escalation=False, path_repair=False)
    on = _zones(Shield(_square_policy(), escalation=False))
    assert "PathRepair" in [r.operator for r in on.filter(st, a).repairs]
    lost = st.model_copy(update={"fix_ok": False})
    d = _zones(Shield(_square_policy(), escalation=False)).filter(lost, a)
    pr = [x for p in d.trace["passes"] for x in p["attempts"] if x["operator"] == "PathRepair"]
    assert pr and pr[0]["result"] == "skip" and "position fix" in pr[0]["reason"], pr
    assert d.emitted == base.filter(lost, a).emitted
    # Inside the square's margin ring: GeofenceEscape is a recovery.
    inside = State(x=9.5, y=0.0, up=10.0)
    d = _zones(Shield(_square_policy(), escalation=False)).filter(inside, Action4D(vx=2.0))
    pr = [x for p in d.trace["passes"] for x in p["attempts"] if x["operator"] == "PathRepair"]
    assert all(x["result"] == "skip" for x in pr), pr
    assert "PathRepair" not in [r.operator for r in d.repairs], d.repairs


def test_path_repair_does_not_overrule_theta_or_lead_deeper():
    """Chain mode takes a detour only (1) when the tick's projection is within
    theta - abandoning projection for the fail-safe is theta's call - and
    (2) where one control period of it does not take the aircraft where the
    request's forecast goes deeper into the ring. Head-on at the square's
    middle either corner detour would (4.0 m -> 4.07 / 4.08 m): it is
    refused and the decision is the one made without PathRepair. A Shield
    with no FSM of its own (a rail's) reads `theta_config` - the ROS 2 node
    sets its FSM's - else the grant's defaults; the slide's 4.0 m is over
    those, so the theta gate is lifted (50 m) to reach the depth gate."""
    from guardrail import fsm as F
    st, a = State(x=3.0, y=0.5, up=10.0, yaw_deg=0.0), Action4D(vx=2.0)
    base = Shield(_square_policy(), escalation=False, path_repair=False).filter(st, a)
    rail = _zones(Shield(_square_policy(), escalation=False))
    d = rail.filter(st, a)
    pr = [x for p in d.trace["passes"] for x in p["attempts"] if x["operator"] == "PathRepair"]
    assert pr and "2.00 m over theta" in pr[0]["reason"], pr       # 4.0 m vs 2.0 m
    assert d.emitted == base.emitted and d.repairs == base.repairs
    rail.theta_config = F.FSMConfig(theta_lateral_m=50.0)
    d = rail.filter(st, a)
    pr = [x for p in d.trace["passes"] for x in p["attempts"] if x["operator"] == "PathRepair"]
    assert pr and pr[0]["result"] == "skip" and "deeper" in pr[0]["reason"], pr
    assert d.emitted == base.emitted and d.repairs == base.repairs
    # 10 m into a 10 m stand-off ring: StandoffHold's 10 m is over theta.
    pol = Policy.model_validate({"policy_id": "so", "constraints": [
        {"id": "so", "type": "subject_standoff", **HARD_P0, "min_range_m": 10.0,
         "soft_margin_m": 2.0, "subject_class": "*"}]})
    sh = _zones(Shield(pol))
    sh.set_subject(20.0, 0.0, "car")
    d = sh.filter(State(x=0.0, y=0.0, up=10.0), Action4D(vx=4.0))
    pr = [x for p in d.trace["passes"] for x in p["attempts"] if x["operator"] == "PathRepair"]
    assert pr and pr[0]["result"] == "skip" and "over theta" in pr[0]["reason"], pr
    assert [r.operator for r in d.repairs] == ["StandoffHold"], d.repairs


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


# ------------------------------------------- the frame at the operator interface
#
# WP3-04 (since 2026-11-21): every repair operator is handed the BODY-frame
# action at the tick's heading and hands one back (guardrail/repair.py, THE
# FRAME). Each test below fails at 71aabb3, where the operators were handed
# the world action and RepairContext had no heading.

class _Spy:
    """Wraps a registered operator and records what it was handed and what
    it handed back, with the context's heading."""

    def __init__(self, op, log):
        self.op, self.log = op, log
        self.name, self.categories = op.name, op.categories

    def repair(self, action, violation, ir, *, ctx):
        att = self.op.repair(action, violation, ir, ctx=ctx)
        self.log.append((self.name, action, att, ctx.heading_deg))
        return att


def _spied(sh):
    log = []
    sh.repair_operators = {k: _Spy(op, log) for k, op in sh.repair_operators.items()}
    return sh, log


def _close(a, b, tol=1e-12):
    return all(abs(x - y) <= tol for x, y in zip((a.vx, a.vy, a.vz_up, a.yaw_rate),
                                                 (b.vx, b.vy, b.vz_up, b.yaw_rate)))


def test_every_operator_is_handed_and_hands_back_the_body_action():
    """demo_policy's square ahead of the nose at 37.3 degrees, the action in
    the body frame (filter_body): the first operator is handed the body
    action that came in (to the round trip's ulp, not the world action,
    which is 0.8 m/s away), every operator is told the tick's heading, and
    the body action the last one hands back is the Shield's emitted body
    action, to the bit. The `_repair_*` method LateralProjection wraps is
    handed the WORLD action (HookOperator turns it, frames.from_body)."""
    from guardrail.frames import from_body
    seen = []

    class Hooked(Shield):
        def _repair_geofence(self, state, a, repairs):
            seen.append(a)
            return super()._repair_geofence(state, a, repairs)
    h = 37.3
    w_in = Action4D(vx=1.3)
    st = State(x=0, y=15, up=4, yaw_deg=h)
    from guardrail.frames import body_action
    body = body_action(w_in, h)
    sh, log = _spied(Hooked(load_policy(ROOT / "policies" / "demo_policy.yaml"),
                            escalation=False))
    d, emitted_body = sh.filter_body(st, body)
    assert [r.operator for r in d.repairs] == ["GeofenceSlide"], d.repairs
    assert log and all(hd == h for _, _, _, hd in log), log
    first_in = log[0][1]
    assert _close(first_in, body) and not _close(first_in, w_in, 0.5), (first_in, body)
    assert log[-1][2].action == emitted_body, (log[-1][2].action, emitted_body)
    lat = [(a, att) for n, a, att, _ in log if n == "LateralProjection"]
    assert lat and lat[0][1].result == "ok"
    assert seen and _close(seen[0], from_body(lat[0][0].vx, lat[0][0].vy, lat[0][0].vz_up,
                                              lat[0][0].yaw_rate, h)), seen
    assert not _close(seen[0], lat[0][0], 0.5)          # the hook's is the world one


class _NoSideways:
    """An operator written to the grant's signature: it removes the
    sideways (body right) speed of the action it is handed."""
    name = "NoSideways"

    def repair(self, action, violation, ir):
        return Action4D(vx=action.vx, vy=0.0, vz_up=action.vz_up, yaw_rate=action.yaw_rate)


def test_a_grant_signature_operator_acts_in_the_body_frame():
    """Over the 3 m/s cap at 30 degrees, forward 3 and right 2 m/s in the
    body frame: KinematicClamp scales it to 3 m/s, then NoSideways (in the
    loop via repair_order) removes the right component OF THE BODY ACTION,
    so the emitted body action has no right component and keeps the clamped
    forward speed. Handed the world action (71aabb3) it removed the East
    component instead, and at 30 degrees the emitted body action kept
    0.66 m/s of sideways speed (measured at 71aabb3)."""
    pol = Policy.model_validate({"policy_id": "k", "constraints": [
        {"id": "kin", "type": "kinematic_envelope", **HARD_P0, "speed_max_mps": 3.0,
         "climb_rate_max_mps": 3.0, "yaw_rate_max_dps": 90.0}]})
    R.register(_NoSideways())
    try:
        for h in (30.0, 90.0, -123.4):
            sh = Shield(pol.model_copy(deep=True), escalation=False,
                        repair_order=("NoSideways", "KinematicClamp"))
            d, eb = sh.filter_body(State(x=0, y=0, up=10, yaw_deg=h),
                                   Action4D(vx=3.0, vy=2.0))
            assert [r.operator for r in d.repairs][:2] == ["SpeedClamp", "NoSideways"], d.repairs
            k = 3.0 / math.hypot(3.0, 2.0)
            assert abs(eb.vy) <= 1e-12 and abs(eb.vx - 3.0 * k) <= 1e-12, (h, eb)
    finally:
        del R.REGISTRY["NoSideways"]


def test_path_repair_and_first_feasible_hand_back_the_body_action():
    """PathRepair's zone detour at 30 degrees (the scene of
    test_path_repair_detours_round_a_zone_projection_slides_along, turned
    by nothing but the heading): the attempt's action is the emitted action
    in the BODY frame at 30 degrees, not the world one. First-feasible mode
    hands LateralProjection the body action and its win is the emitted
    action in the body frame too."""
    from guardrail.frames import body_action
    h = 30.0
    st, a = State(x=5.0, y=9.0, up=10.0, yaw_deg=h), Action4D(vx=2.0)
    sh, log = _spied(_zones(Shield(_square_policy(), escalation=False, path_repair=True)))
    d = sh.filter(st, a)
    pr = [att for n, _, att, _ in log if n == "PathRepair" and att.result == "ok"]
    assert pr and [r.operator for r in d.repairs if r.operator == "PathRepair"], d.repairs
    want = body_action(d.emitted, h)
    assert pr[0].action == want and not _close(pr[0].action, d.emitted, 0.5), (pr, d.emitted)
    sh, log = _spied(Shield(load_policy(ROOT / "policies" / "demo_policy.yaml"),
                            escalation=False, repair_mode="first_feasible",
                            path_repair=False))
    st2 = State(x=0, y=15, up=4, yaw_deg=h)
    d = sh.filter(st2, Action4D(vx=1.3))
    lat = [(x, att) for n, x, att, _ in log if n == "LateralProjection" and att.result == "ok"]
    assert lat, log
    assert _close(lat[0][0], body_action(Action4D(vx=1.3), h)), lat[0][0]
    assert lat[0][1].action == body_action(d.emitted, h), (lat[0][1].action, d.emitted)


def test_the_interface_rotation_pair_is_lossless_on_its_own_results():
    """RepairContext.to_body / to_world at a heading whose rotation is not
    exact: a body action the pair made converts back to the very world
    action it was made from (a non-finite channel included, which a
    rotation would spread to both horizontal channels); an equal body
    action an operator built itself, or the pair's own one changed in place,
    is rotated (frames.from_body). The heading defaults to the state's."""
    from guardrail.frames import from_body
    sh = Shield(load_policy(ROOT / "policies" / "demo_policy.yaml"))
    ctx = R.RepairContext(sh, State(x=0, y=0, up=5, yaw_deg=37.3), [], Action4D(vx=1.0))
    assert ctx.heading_deg == 37.3
    rng = random.Random(20261121)
    n_lossy = 0
    for _ in range(200):
        w = Action4D(vx=rng.uniform(-8, 8), vy=rng.uniform(-8, 8), vz_up=1.0, yaw_rate=0.2)
        b = ctx.to_body(w)
        assert ctx.to_world(b) is w and ctx.to_body(w) is b
        own = Action4D(vx=b.vx, vy=b.vy, vz_up=b.vz_up, yaw_rate=b.yaw_rate)
        back = ctx.to_world(own)
        assert back is not w and _close(back, w, 1e-13)
        n_lossy += (back.vx, back.vy) != (w.vx, w.vy)
    assert n_lossy > 20, n_lossy        # the plain round trip is NOT exact here
    w = Action4D(vx=float("nan"), vy=1.0)
    assert ctx.to_world(ctx.to_body(w)) is w
    b = ctx.to_body(Action4D(vx=2.0, vy=0.5))
    b.vx = 0.0
    assert ctx.to_world(b) == from_body(0.0, b.vy, b.vz_up, b.yaw_rate, 37.3)
    assert ctx.raw_body == ctx.to_body(ctx.raw)


# ------------------------------------------------------------- first feasible

def _square_policy(margin=1.0, extra=()):
    return Policy.model_validate({"policy_id": "sq", "constraints": [
        {"id": "kin", "type": "kinematic_envelope", **HARD_P0, "speed_max_mps": 5.0,
         "climb_rate_max_mps": 3.0, "yaw_rate_max_dps": 90.0},
        {"id": "sq", "type": "polygon_fence", **HARD_P0, "margin_m": margin,
         "vertices": [{"x": 10, "y": -10}, {"x": 30, "y": -10}, {"x": 30, "y": 10},
                      {"x": 10, "y": 10}]}, *extra]})


def test_first_feasible_will_not_apply_a_repair_over_theta():
    """Head-on at the square from 6 m out of its margin at 4 m/s: the raw
    forecast goes 13 m into the ring. The chain slides (GeofenceSlide, 13 m,
    left for the FSM to judge). The grant's selection refuses a repair whose
    magnitude is over theta (2.0 m lateral): LateralProjection fails, no
    operator wins, and the fallback stops (a stop is legal here)."""
    st, a = State(x=3.0, y=0.0, up=10.0), Action4D(vx=4.0)
    chain = Shield(_square_policy(), escalation=False, path_repair=False).filter(st, a)
    assert [r.operator for r in chain.repairs] == ["GeofenceSlide"] and not chain.braked
    assert chain.repairs[0].magnitude_m > 2.0
    ff = Shield(_square_policy(), escalation=False, repair_mode="first_feasible",
                path_repair=False).filter(st, a)
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
                repair_mode="first_feasible", path_repair=False).filter(st, a)
    assert ff.emitted == chain.emitted and [r.operator for r in ff.repairs] == ["GeofenceSlide"]
    p1 = ff.trace["passes"][-1]
    assert p1["pass"] == 1 and p1["attempts"][-1]["operator"] == "LateralProjection"
    assert p1["attempts"][-1]["result"] == "ok"
    # The clamps are skipped because they cannot repair a geofence
    # violation, not because they found nothing of their own to do.
    assert [(x["result"], x["reason"]) for x in p1["attempts"][:-1]] == \
        [("skip", "not a geofence violation (nfz-square)")] * 2, p1["attempts"]


_ALT = {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5.0, "alt_max_m": 50.0}


def test_first_feasible_targets_the_earliest_violation_first():
    """Sinking toward the 5 m floor (breaks it at t + 1.1 s, 0.4 m deep by
    t + 5 s) while drifting into the square's ring (t + 2.6 s, 0.5 m deep):
    iteration 1 targets the floor and AltitudeClamp wins; iteration 2
    targets the zone and LateralProjection wins. Both repairs are within
    theta, and the result re-checks clean."""
    pol = _square_policy(extra=[_ALT])
    st, a = State(x=8.5, y=0.0, up=5.1), Action4D(vx=0.2, vy=2.0, vz_up=-0.1)
    sh = Shield(pol, escalation=False, repair_mode="first_feasible", path_repair=False)
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
                repair_mode="first_feasible", path_repair=False).filter(st, a)
    alt = [x for p in ff.trace["passes"] for x in p["attempts"]
           if x["operator"] == "AltitudeClamp"]
    assert alt[0]["result"] == "fail" and "over theta (vertical)" in alt[0]["reason"], alt
    assert ff.braked and [r.operator for r in ff.repairs] == ["Brake"]


_SQ2 = {"id": "sq2", "type": "polygon_fence", **HARD_P0, "margin_m": 1.0,
        "vertices": [{"x": -10, "y": 10}, {"x": 10, "y": 10}, {"x": 10, "y": 30},
                     {"x": -10, "y": 30}]}


def test_first_feasible_reads_theta_as_the_fsm_sums_it():
    """REGRESSION (2026-10-19). The FSM sums magnitude_m per axis over the
    tick's repairs (fsm.magnitude_from_repairs); the selection compared each
    Repair alone, so it accepted what the FSM then called over theta. Heading
    into the corner between two squares, one LateralProjection logs two
    slides, 0.6 m and 1.5 m: 2.1 m in all, so it is no longer applied (on
    4418ae0..1b95383 it was, unbraked). Two slides of 1.0 m (2.0 m, theta is
    inclusive) still are. Across iterations: a 1.2 m slide off the square,
    then a 1.8 m StandoffHold, 3.0 m in all - each was "ok" alone, and with
    the FSM flying the tick was X1, "repair over theta", on an action the
    selection had accepted."""
    from guardrail import fsm as F
    sh = Shield(_square_policy(extra=[_SQ2]), escalation=False, repair_mode="first_feasible",
                path_repair=False)
    a = Action4D(vx=2.6, vy=2.0)
    d = sh.filter(State(x=-3.0, y=0.5, up=10.0), a)
    lat = F.magnitude_from_repairs([r for r in d.repairs if r.operator != "Brake"]).lateral_m
    assert d.braked and lat <= 2.0 + 1e-9, (d.repairs, lat)
    att = [x for p in d.trace["passes"] for x in p["attempts"]
           if x["operator"] == "LateralProjection"][0]
    assert att["result"] == "fail" and "over theta (lateral)" in att["reason"], att
    d = sh.filter(State(x=-3.0, y=0.0, up=10.0), a)                 # the boundary
    assert not d.braked and [r.operator for r in d.repairs] == ["GeofenceSlide"] * 2
    assert F.magnitude_from_repairs(d.repairs).lateral_m <= 2.0 + 1e-9
    ring = {"id": "ring", "type": "subject_standoff", **HARD_P0, "subject_class": "*", "min_range_m": 5.0}
    st, a = State(x=4.9, y=3.3, up=10.0), Action4D(vx=1.9, vy=2.3)
    for esc in (False, True):
        sh = Shield(_square_policy(extra=[ring]), escalation=esc,
                    repair_mode="first_feasible", path_repair=False)
        sh.set_subject(1.7, 11.5, "vehicle")
        d = sh.filter(st, a)
        wins = [(p["pass"], x["operator"]) for p in d.trace["passes"]
                for x in p["attempts"] if x["result"] == "ok"]
        assert wins == [(1, "LateralProjection")], wins
        hold = [x for p in d.trace["passes"] if p["pass"] == 2 for x in p["attempts"]
                if x["operator"] == "StandoffProjection"]
        assert hold and hold[0]["result"] == "fail" and "summed" in hold[0]["reason"], hold
        assert d.braked and not d.emitted_violations, d.repairs
        if esc:
            assert "repair over theta" not in d.fsm_record["reason"], d.fsm_record["reason"]


def test_first_feasible_exempts_a_recovery_from_theta():
    """Inside the square, 6 m from its ring: GeofenceEscape's magnitude (the
    6 m depth) is three times theta, but a recovery is exempt (the
    reference's "brake while inside = deadlock"), so it wins."""
    st, a = State(x=15.0, y=0.0, up=10.0), Action4D(vx=0.5)
    ff = Shield(_square_policy(), escalation=False, repair_mode="first_feasible",
                path_repair=False).filter(st, a)
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
                   path_repair=False, repair_order=("Slower",)).filter(st, a)
    finally:
        del R.REGISTRY["Slower"]
    att = d.trace["passes"][1]["attempts"][0]
    assert (att["operator"], att["result"]) == ("Slower", "fail"), att
    assert "sq still violated" in att["reason"]
    assert d.braked and [r.operator for r in d.repairs] == ["Brake"], d.repairs


# ------------------------------------------------------------------ PathRepair

def _wedge(flags, theta_m=None):
    import sweep_scenarios as S
    import test_sweep as TS
    from guardrail import fsm as F
    orig = S.Shield

    def make(*a, **k):
        sh = Shield(*a, **{**k, **flags})
        if theta_m is not None:            # the theta PathRepair defers to
            sh.theta_config = F.FSMConfig(theta_lateral_m=theta_m)
        return sh
    S.Shield = make
    try:
        # At its own 30 s (the library flies it 50 s since 2026-11-02).
        spec = TS._with(TS.BY_ID["standoff-wedge"], escalation=False,
                        ticks=TS.LEGACY_TICKS["standoff-wedge"])
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
    tests/test_sweep.py flies the LEGACY set, and with a theta (50 m) the
    10 m projection is within: at the grant's 2.0 m, StandoffHold's 10 m is
    over theta and PathRepair stands aside (with the FSM on, X1 > G3 -
    theta's call, not the operator's)."""
    extra, escapes, ops = _wedge({"path_repair": False})
    assert (extra["final"]["x"], extra["final"]["y"]) == (0.0, 0.0)
    assert extra.get("reached_goal") is False and ops == {"StandoffHold": 300}, (extra, ops)
    # No FSM here, so PathRepair reads Shield.theta_config (else the grant's
    # 2.0 m): StandoffHold's 10 m is over it, and the detour stands aside -
    # the hover again. With a theta the projection is within, it is freed.
    extra, escapes, ops = _wedge({"path_repair": True})
    assert extra.get("reached_goal") is False and ops == {"StandoffHold": 300}, (extra, ops)
    extra, escapes, ops = _wedge({"path_repair": True}, theta_m=50.0)
    assert extra.get("reached_goal") is True, extra
    assert escapes == 0, escapes
    assert extra["min_range_to_subject"] >= 9.99, extra["min_range_to_subject"]
    assert ops.get("PathRepair", 0) > 0, ops


def test_path_repair_detours_round_a_zone_projection_slides_along():
    """4 m west of the square's margin ring, 2 m inside its north edge's
    line, heading north: the slide turns the command 90 degrees (zero
    progress toward where the pilot is going). PathRepair aims for the
    corner of the margin ring's coarse ring, pushed 1 m out, (8, 12), at the
    commanded speed: 0.71 of its 2 m/s northward instead of none. The
    detour re-checks clean, leaves the request no deeper in the ring, and
    its waypoint is logged in body frame. (Head-on at the square's middle
    the corner detour would lead deeper and is refused:
    test_path_repair_does_not_overrule_theta_or_lead_deeper.)"""
    st, a = State(x=5.0, y=9.0, up=10.0, yaw_deg=0.0), Action4D(vx=2.0)
    base = Shield(_square_policy(), escalation=False, path_repair=False).filter(st, a)
    assert [r.operator for r in base.repairs] == ["GeofenceSlide"]
    assert base.emitted.vx < 1e-9                       # no progress north
    d = _zones(Shield(_square_policy(), escalation=False, path_repair=True)).filter(st, a)
    pr = [r for r in d.repairs if r.operator == "PathRepair"]
    assert pr and "body frame" in pr[0].detail and pr[0].axis == "lateral"
    assert d.emitted.vx > 0.7 and not d.emitted_violations, d
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
    """A small zone sits on the detour leg of
    test_path_repair_detours_round_a_zone_projection_slides_along (from
    (5, 9) to the corner (8, 12)), off the request's own path: flying the
    detour would enter it within the horizon. PathRepair checks each
    candidate against every rule and takes none; the decision is the one
    made without it."""
    flank = [{"id": "flank", "type": "polygon_fence", **HARD_P0, "margin_m": 0.2,
              "vertices": [{"x": 7.0, "y": 10.5}, {"x": 7.5, "y": 10.5},
                           {"x": 7.5, "y": 11.0}, {"x": 7.0, "y": 11.0}]}]
    st, a = State(x=5.0, y=9.0, up=10.0), Action4D(vx=2.0)
    base = Shield(_square_policy(extra=flank), escalation=False,
                  path_repair=False).filter(st, a)
    d = _zones(Shield(_square_policy(extra=flank), escalation=False,
                      path_repair=True)).filter(st, a)
    pr = [x for p in d.trace["passes"] for x in p["attempts"] if x["operator"] == "PathRepair"]
    assert pr and all(x["result"] == "skip" for x in pr), pr
    # (8, -12) would also lead deeper (2.0 -> 2.20 m); (8, 12) re-checks into
    # the flank zone.
    assert pr[0]["reason"].endswith("no other re-checks clean"), pr[0]
    assert "PathRepair" not in [r.operator for r in d.repairs]
    assert d.model_dump() == base.model_dump()


def test_path_repair_passes_a_ring_at_the_tangent_of_its_soft_margin():
    """Subject 20 m ahead, ring 10 m, soft margin 2 m: the two waypoints are
    the tangent points of the 12 m circle, (12.8, +-9.6), each 12 m from the
    subject. With a soft margin under 1 m the circle is the ring + 1 m."""
    def wps(soft):
        pol = Policy.model_validate({"policy_id": "so", "constraints": [
            {"id": "so", "type": "subject_standoff", **HARD_P0, "min_range_m": 10.0,
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
        {"id": "wall", "type": "polygon_fence", **HARD_P0, "margin_m": 0.2,
         "vertices": [{"x": 4.05, "y": -50}, {"x": 4.15, "y": -50}, {"x": 4.15, "y": 50},
                      {"x": 4.05, "y": 50}]}]})


def test_the_segment_check_catches_a_zone_thinner_than_one_step():
    st, a = State(x=0.0, y=0.0, up=10.0), Action4D(vx=12.0)
    plain = Shield(_wall_policy(), escalation=False)
    assert plain._check(st, a) == [], "the poses at x = 3.6 and 4.8 straddle the ring"
    seg = Shield(_wall_policy(), escalation=False, segment_check=True, path_repair=False)
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
        {"id": "wall", "type": "polygon_fence", **HARD_P0, "margin_m": 0.2, **extra,
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
        {"id": "L", "type": "corridor", **HARD_P0, "width_m": 2.0, "altitude_floor_m": 0,
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
        {"id": "u", "type": "corridor", **HARD_P0, "width_m": 2.0, "altitude_floor_m": 0,
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
