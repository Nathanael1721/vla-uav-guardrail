"""
Mid-flight events: the grant's three hot-applicable classes, applied to a live
Shield (Policy DSL page, "Mid-flight update model"), and the audit record that
stamps each decision with the rule set it was made under.

    python tests/test_shield_events.py      (no pytest needed)

What each event must do, and what is checked here:

  dynamic_nfz         spawn / move / translate / rotate / scale / expire; a zone
                      with `motion` is enforced where it WILL be at each pose of
                      the lookahead, not where it was spawned
  time_window_switch  hold a rule on or off (by hand, or on its own schedule)
  corridor_swap       replace a corridor
  every event         generation + 1, a new policy_hash, the compiled rules
                      swapped in one step (a tick already running keeps the old
                      set), and an entry in Shield.events

Shown failing first: run against guardrail/shield.py, ir.py and audit.py as of
401305a, every test below fails or errors (docs/DESIGN-shield-enforcement.md).
"""
from __future__ import annotations

import json
import math
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

from guardrail import Action4D, AuditLogger, Shield, State, load_policy  # noqa: E402
from guardrail.models import (CorridorSwap, DynamicNFZ, Policy,         # noqa: E402
                              TimeWindowSwitch)
from guardrail.shield import (HotApplyRefused, LayerRelaxation,        # noqa: E402
                              LockedRuleClass, RuleIdConflict, UnknownRule,
                              as_dynamic_nfz)

DEMO = ROOT / "policies" / "demo_policy.yaml"


def _sq(cx, cy, half):
    return [{"x": cx - half, "y": cy - half}, {"x": cx + half, "y": cy - half},
            {"x": cx + half, "y": cy + half}, {"x": cx - half, "y": cy + half}]


def _zone(zid="dyn", cx=-12.0, cy=-20.0, half=4.0, margin=0.0, motion=None, **kw):
    data = {"id": zid, "type": "dynamic_nfz", "vertices": _sq(cx, cy, half),
            "margin_m": margin, **kw}
    if motion is not None:
        data["motion"] = motion
    return DynamicNFZ.model_validate(data)


def _raises(exc, fn, *a, **k):
    try:
        fn(*a, **k)
    except exc as e:
        return e
    raise AssertionError(f"{fn.__name__} did not raise {exc.__name__}")


def _ring(sh, zid, st=State(x=0.0, y=0.0, up=4.0)):
    return next(r for r in sh.rule_status(st) if r["id"] == zid)["vertices"]


def _close(a, b, tol=1e-9):
    return len(a) == len(b) and all(abs(p[0] - q[0]) < tol and abs(p[1] - q[1]) < tol
                                    for p, q in zip(a, b))


# ------------------------------------------------------------------ dynamic_nfz

def test_a_spawned_zone_is_enforced_on_the_next_tick_and_restamps():
    sh = Shield(load_policy(DEMO))
    st, a = State(x=-20, y=-20, up=4), Action4D(vx=3.0)
    d0 = sh.filter(st, a)
    assert not d0.touched and d0.generation == 0
    h0 = sh.policy.policy_hash
    ev = sh.hot_apply(_zone())
    assert ev["op"] == "spawn" and ev["generation"] == 1
    assert ev["policy_hash"] == sh.policy.policy_hash != h0
    assert sh.ir.is_current_for(sh.policy)
    d1 = sh.filter(st, a)
    assert [v.rule_id for v in d1.violations] == ["dyn"], d1.violations
    assert (d1.generation, d1.policy_hash) == (1, sh.policy.policy_hash)
    assert not sh._check(st, d1.emitted)
    assert d0.policy_hash == h0                    # a decision is never restamped


def test_a_spawn_with_an_existing_id_or_a_negative_margin_is_refused():
    sh = Shield(load_policy(DEMO))
    sh.filter(State(x=-20, y=-20, up=4), Action4D())
    sh.hot_apply(_zone())
    _raises(RuleIdConflict, sh.hot_apply, _zone())
    _raises(RuleIdConflict, sh.hot_apply, _zone(zid="nfz-square"))
    _raises(HotApplyRefused, sh.hot_apply, _zone(zid="neg", margin=-1.0))
    assert sh.policy.generation == 1               # nothing half-applied


def test_translate_rotate_scale_and_move_reshape_the_enforced_zone():
    sh = Shield(load_policy(DEMO))
    sh.filter(State(x=-20, y=-20, up=4), Action4D())
    sh.hot_apply(_zone(cx=-40.0, cy=-40.0, half=2.0))
    base = [(-42.0, -42.0), (-38.0, -42.0), (-38.0, -38.0), (-42.0, -38.0)]
    assert _close(_ring(sh, "dyn"), base)
    sh.translate_nfz("dyn", 5.0, -1.0)
    assert _close(_ring(sh, "dyn"), [(x + 5.0, y - 1.0) for x, y in base])
    sh.rotate_nfz("dyn", 90.0)                     # about the vertex mean (-35, -41)
    got = _ring(sh, "dyn")
    want = [(-35.0 - (y + 41.0), -41.0 + (x + 35.0))
            for x, y in [(x + 5.0, y - 1.0) for x, y in base]]
    assert _close(got, want, 1e-9), (got, want)
    sh.scale_nfz("dyn", 2.0)
    assert _close(_ring(sh, "dyn"), [(-35.0 + 2 * (x + 35.0), -41.0 + 2 * (y + 41.0))
                                     for x, y in want])
    sh.move_nfz("dyn", [(0.0, 30.0), (4.0, 30.0), (4.0, 34.0)])
    assert _close(_ring(sh, "dyn"), [(0.0, 30.0), (4.0, 30.0), (4.0, 34.0)])
    assert sh.policy.generation == 5
    assert [e["op"] for e in sh.events] == ["spawn", "translate", "rotate", "scale", "move"]
    # The Shield enforces the moved zone, not the old one.
    v = sh._check(State(x=2.0, y=25.0, up=4), Action4D(vy=3.0))
    assert [x.rule_id for x in v] == ["dyn"], v
    _raises(HotApplyRefused, sh.scale_nfz, "dyn", 0.0)
    _raises(UnknownRule, sh.translate_nfz, "nope", 1.0, 1.0)
    _raises(LockedRuleClass, sh.translate_nfz, "nfz-square", 1.0, 1.0)


def test_expire_removes_the_zone():
    sh = Shield(load_policy(DEMO))
    st, a = State(x=-20, y=-20, up=4), Action4D(vx=3.0)
    sh.filter(st, Action4D())
    sh.hot_apply(_zone())
    assert sh.filter(st, a).touched
    ev = sh.expire_nfz("dyn")
    assert ev["op"] == "expire" and ev["generation"] == 2
    assert not sh.filter(st, a).touched
    assert all(c.id != "dyn" for c in sh.policy.constraints)
    assert sh.ir.is_current_for(sh.policy)


def test_a_moving_zone_is_enforced_where_it_will_be():
    """A 10 m zone (0.5 m margin) spawned with its edge 30 m north of a
    hovering aircraft, drifting south at 2 m/s: its margin ring reaches the
    aircraft when 29.5 - 2 t < 0, i.e. after 14.75 s. With the grant's 5 s
    lookahead at 0.1 s the Shield must first see it at t = 9.8 s, predicted at
    t + 5.0 s (the first pose past 4.95 s), and at t = 12 s predict t + 2.8 s;
    it repairs by moving away at the zone's own speed (into-zone motion is
    measured relative to the zone). A Shield that froze the zone where it was
    spawned (the 401305a hot_apply did) never sees it."""
    sh = Shield(Policy(policy_id="m", constraints=[]))
    st = State(x=0.0, y=0.0, up=10.0)
    sh.hot_apply(_zone(cx=35.0, cy=0.0, half=5.0, margin=0.5,
                       motion={"vx_mps": -2.0}), t=0.0)
    first = at12 = None
    for k in range(141):
        t = round(0.1 * k, 9)
        d = sh.filter(st, Action4D(), t=t)
        if d.violations and first is None:
            first = (t, d.violations[0].predicted_at_s, d)
        if abs(t - 12.0) < 1e-9:
            at12 = d
    assert first is not None, "the drifting zone was never seen"
    t0, pred, d = first
    assert abs(t0 - 9.8) < 1e-6 and abs(pred - 5.0) < 1e-6, (t0, pred)
    assert abs(at12.violations[0].predicted_at_s - 2.8) < 1e-6, at12.violations
    assert [r.operator for r in d.repairs] == ["GeofenceSlide"], d.repairs
    assert abs(d.emitted.vx - (-2.0)) < 1e-9 and d.emitted_violations == [], d
    ring = _ring(sh, "dyn", st)
    assert abs(min(p[0] for p in ring) - (30.0 - 2.0 * 14.0)) < 1e-6, ring


def test_a_moving_zone_follows_ring_at_from_its_anchor():
    """The zone the Shield enforces at tick time t is DynamicNFZ.ring_at(t -
    spawn time): the model's own motion, translation and rotation."""
    z = _zone(cx=0.0, cy=50.0, half=3.0, motion={"vx_mps": 1.0, "vy_mps": -0.5,
                                                 "yaw_rate_dps": 10.0})
    sh = Shield(Policy(policy_id="m", constraints=[]))
    sh.hot_apply(z, t=2.0)
    for t in (2.0, 3.0, 4.5):
        sh.filter(State(x=-100.0, y=-100.0, up=5.0), Action4D(), t=t)
        assert _close(_ring(sh, "dyn"), z.ring_at(t - 2.0), 1e-6), t


def test_editing_a_moving_zone_reanchors_it_where_it_is_now():
    z = _zone(cx=0.0, cy=50.0, half=3.0, motion={"vx_mps": 1.0})
    sh = Shield(Policy(policy_id="m", constraints=[]))
    sh.hot_apply(z, t=0.0)
    sh.filter(State(x=-100.0, y=-100.0, up=5.0), Action4D(), t=4.0)
    sh.translate_nfz("dyn", 0.0, 10.0)             # at t = 4 the zone is 4 m north
    sh.filter(State(x=-100.0, y=-100.0, up=5.0), Action4D(), t=6.0)
    want = [(x + 6.0, y + 10.0) for x, y in z.ring_at(0.0)]
    assert _close(_ring(sh, "dyn"), want, 1e-6), (_ring(sh, "dyn"), want)


def test_the_lookahead_frame_of_a_turning_zone_is_ring_at():
    """Inside one tick the Shield does not rebuild a moving zone per pose: it
    moves each forecast pose into the zone's frame (`_to_zone_frame`) and
    tests it against the zone as it stands now. That must be the same
    question as "is the pose inside DynamicNFZ.ring_at(age + tau)", the
    model's own motion, for a zone that translates AND turns."""
    import random
    from shapely.geometry import Point, Polygon
    from guardrail.shield import _to_zone_frame
    z = _zone(cx=0.0, cy=0.0, half=6.0, motion={"vx_mps": 1.5, "vy_mps": -0.7,
                                               "yaw_rate_dps": 25.0})
    sh = Shield(Policy(policy_id="m", constraints=[]))
    sh.hot_apply(z, t=1.0)
    sh.filter(State(x=-500.0, y=-500.0, up=5.0), Action4D(), t=3.0)     # age 2 s
    with sh._scope(sh._new_tick(3.0)):
        mv = sh._motion_of(z)
    now = Polygon(z.ring_at(2.0))
    rng = random.Random(11)
    agree = 0
    for _ in range(2000):
        tau = rng.uniform(0.0, 5.0)
        p = State(x=rng.uniform(-15, 20), y=rng.uniform(-20, 15), up=5.0)
        truth = Polygon(z.ring_at(2.0 + tau)).contains(Point(p.x, p.y))
        qx, qy = _to_zone_frame(p, tau, mv)
        assert now.contains(Point(qx, qy)) == truth, (tau, p)
        agree += truth
    assert agree > 200, agree            # the zone was actually hit, not missed


def test_a_turning_zone_is_seen_where_it_turns_to():
    """A 40 m bar (x -1..1, y -20..20) turning at 30 deg/s about its centre;
    a hovering aircraft at (15, 0) is outside the bar's box now, and inside the
    bar about 3 s from now, when it has turned 90 deg. The swept box must
    cover the disc the bar turns in, not only where it is."""
    bar = DynamicNFZ.model_validate({
        "id": "bar", "type": "dynamic_nfz", "margin_m": 0.0,
        "vertices": [{"x": -1, "y": -20}, {"x": 1, "y": -20},
                     {"x": 1, "y": 20}, {"x": -1, "y": 20}],
        "motion": {"yaw_rate_dps": 30.0}})
    sh = Shield(Policy(policy_id="r", constraints=[bar]))
    d = sh.filter(State(x=15.0, y=0.0, up=5.0), Action4D(), t=0.0)
    assert [(v.rule_id, round(v.predicted_at_s, 6)) for v in d.violations] == [("bar", 2.9)], \
        d.violations


def test_escaping_a_moving_zone_is_judged_relative_to_it():
    """Inside a zone that moves north at 2 m/s, flying north at 1 m/s is NOT
    leaving it. The Shield says so, and its escape outpaces the zone."""
    z = _zone(zid="mz", cx=0.0, cy=0.0, half=5.0, motion={"vx_mps": 2.0})
    sh = Shield(Policy(policy_id="m", constraints=[z]))
    d = sh.filter(State(x=4.0, y=0.0, up=5.0), Action4D(vx=1.0), t=0.0)
    assert [(v.rule_id, v.detail) for v in d.violations] == [
        ("mz", "currently INSIDE zone and not escaping")], d.violations
    assert [r.operator for r in d.repairs] == ["GeofenceEscape"] and d.emitted.vx > 2.0


def _count_zone_tests(sh, st, a, zone_id):
    """How many point-in-zone tests one filter() tick spends on `zone_id`."""
    import guardrail.shield as S
    real, n = S.point_in_fence, [0]

    def counting(x, y, up, f, *rest):
        n[0] += f.id == zone_id
        return real(x, y, up, f, *rest)
    S.point_in_fence = counting
    try:
        d = sh.filter(st, a)
    finally:
        S.point_in_fence = real
    return n[0], d


def test_a_moving_zone_out_of_reach_costs_no_point_test():
    """A moving zone cannot go in the STRtree (it will be elsewhere), and the
    first version then tested it at every forecast pose of every check: 48
    drifting zones on the 50-rule load cost 31 ms per _check, six times the
    grant's 5 ms. A zone 400 m away drifting at 1 m/s cannot reach a 5 s
    forecast; its swept box over the horizon says so, and it is skipped."""
    far = _zone(zid="far", cx=400.0, cy=0.0, half=5.0, motion={"vx_mps": -1.0})
    near = _zone(zid="near", cx=20.0, cy=0.0, half=3.0, motion={"vx_mps": -1.0})
    sh = Shield(Policy(policy_id="m", constraints=[]))
    sh.hot_apply(far, t=0.0)
    sh.hot_apply(near, t=0.0)
    st, a = State(x=0.0, y=0.0, up=5.0), Action4D(vx=3.0)
    n_far, d = _count_zone_tests(sh, st, a, "far")
    assert n_far == 0, n_far
    n_near, _ = _count_zone_tests(sh, st, a, "near")
    assert n_near > 0 and [v.rule_id for v in d.violations] == ["near"], d.violations


def test_the_moving_zone_prefilter_never_changes_an_answer():
    """The swept-box skip may only change what is LOOKED AT: on 600 seeded
    states near translating and turning zones (each zone spawned at t = 0, so
    the states follow them), every decision equals the one from a Shield that
    tests every moving zone at every pose."""
    import random

    class NoSkip(Shield):
        def _may_reach(self, rec, box, horizon):
            return True

    rng = random.Random(23)
    zones = [_zone(zid=f"z{i}", cx=rng.uniform(-30, 30), cy=rng.uniform(-30, 30),
                   half=rng.uniform(2, 6), margin=rng.uniform(0.0, 1.5),
                   motion={"vx_mps": rng.uniform(-3, 3), "vy_mps": rng.uniform(-3, 3),
                           "yaw_rate_dps": rng.choice([0.0, 0.0, rng.uniform(-40, 40)])})
             for i in range(8)]
    pol = Policy(policy_id="m", constraints=zones)
    a_sh, b_sh = Shield(pol.model_copy(deep=True)), NoSkip(pol.model_copy(deep=True))
    hits = 0
    for k in range(600):
        t = 0.1 * k
        ring = rng.choice(zones).ring_at(t)            # near where a zone is now
        cx, cy = (sum(p[0] for p in ring) / len(ring), sum(p[1] for p in ring) / len(ring))
        st = State(x=cx + rng.uniform(-18, 18), y=cy + rng.uniform(-18, 18), up=5.0)
        a = Action4D(vx=rng.uniform(-5, 5), vy=rng.uniform(-5, 5))
        da, db = a_sh.filter(st, a, t=t), b_sh.filter(st, a, t=t)
        assert da.model_dump() == db.model_dump(), k
        hits += bool(da.violations)
    assert hits > 60, hits


# ------------------------------------------------------------ time_window_switch

def test_a_switch_turns_a_rule_off_and_back_on():
    sh = Shield(load_policy(DEMO))
    st, a = State(x=0, y=15, up=4), Action4D(vx=1.3)
    assert sh.filter(st, a).touched
    ev = sh.set_rule_active("nfz-square", False)
    assert ev["op"] == "switch_off" and ev["generation"] == 1
    assert not sh.filter(st, a).touched
    row = next(r for r in sh.rule_status(st) if r["id"] == "nfz-square")
    assert row["in_force"] is False
    sh.set_rule_active("nfz-square", True)
    assert sh.filter(st, a).touched
    standing = [c for c in sh.policy.constraints if c.type == "time_window_switch"]
    assert len(standing) == 1 and standing[0].active is True, standing
    _raises(UnknownRule, sh.set_rule_active, "nope", False)
    _raises(HotApplyRefused, sh.hot_apply, TimeWindowSwitch(
        id="sw2", type="time_window_switch", target_id=standing[0].id, active=False))


def test_a_scheduled_switch_follows_the_clock_and_fails_safe_without_one():
    """A switch that turns the zone off 12:00-13:00: off at 12:30, on at 13:30.
    With no clock the window cannot be read, and nothing that cannot establish
    "this rule is off now" may switch it off - so the zone stays on."""
    sw = TimeWindowSwitch.model_validate({
        "id": "lunch", "type": "time_window_switch", "target_id": "nfz-square",
        "active": False, "valid_time": {"recurrence": {
            "start_time": "12:00", "end_time": "13:00"}}})
    st, a = State(x=0, y=15, up=4), Action4D(vx=1.3)
    for hhmm, touched in (("12:30", False), ("13:30", True)):
        when = datetime(2026, 10, 7, int(hhmm[:2]), int(hhmm[3:]))
        sh = Shield(load_policy(DEMO), now=lambda w=when: w)
        sh.hot_apply(sw)
        assert sh.filter(st, a).touched is touched, hhmm
    sh = Shield(load_policy(DEMO))
    sh.hot_apply(sw)
    assert sh.filter(st, a).touched


# ----------------------------------------------------------------- corridor_swap

def _lanes():
    return Policy.model_validate({"policy_id": "lanes", "constraints": [
        {"id": "lane-a", "type": "corridor", "width_m": 10,
         "centerline": [{"x": -100, "y": 0}, {"x": 100, "y": 0}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 50}]})


def _swap(sid, cy):
    return CorridorSwap.model_validate({
        "id": sid, "type": "corridor_swap", "target_id": "lane-a", "width_m": 10,
        "centerline": [{"x": -100, "y": cy}, {"x": 100, "y": cy}],
        "altitude_floor_m": 0, "altitude_ceiling_m": 50})


def test_a_corridor_swap_replaces_the_corridor():
    sh = Shield(_lanes())
    on_a = State(x=0, y=0, up=10)
    assert not sh.filter(on_a, Action4D(vx=2.0)).touched
    sh.hot_apply(_swap("lane-b", 40.0))
    d = sh.filter(on_a, Action4D(vx=2.0))
    assert [v.rule_id for v in d.violations] == ["lane-b"], d.violations
    on_b = State(x=0, y=40, up=10)
    assert not sh.filter(on_b, Action4D(vx=2.0)).touched     # lane-a no longer binds
    sh.hot_apply(_swap("lane-c", -40.0))                      # supersedes lane-b
    assert {c.id for c in sh.policy.constraints} == {"lane-a", "lane-c"}
    assert [v.rule_id for v in sh.filter(on_b, Action4D(vx=2.0)).violations] == ["lane-c"]
    fence = Shield(load_policy(DEMO))
    bad = CorridorSwap.model_validate({**_swap("x", 0.0).model_dump(), "target_id": "nfz-square"})
    _raises(HotApplyRefused, fence.hot_apply, bad)


def test_declared_events_are_enforced_from_the_policy():
    """dynamic_nfz, time_window_switch and corridor_swap written IN the policy
    are enforced from the first tick (the zone moves from the first tick)."""
    pol = Policy.model_validate({"policy_id": "decl", "constraints": [
        {"id": "lane-a", "type": "corridor", "width_m": 10,
         "centerline": [{"x": -100, "y": 0}, {"x": 100, "y": 0}]},
        {"id": "nfz", "type": "polygon_fence", "margin_m": 0, "vertices": _sq(50, 0, 3)},
        {"id": "off", "type": "time_window_switch", "target_id": "nfz", "active": False},
        {"id": "drift", "type": "dynamic_nfz", "margin_m": 0, "vertices": _sq(0, 3, 1),
         "motion": {"vx_mps": 1.0}}]})
    sh = Shield(pol)
    d = sh.filter(State(x=44, y=0, up=5), Action4D(vx=3.0), t=0.0)
    assert "nfz" not in [v.rule_id for v in d.violations]    # switched off
    sh.filter(State(x=-50, y=0, up=5), Action4D(), t=0.0)
    ring = _ring(sh, "drift")
    assert abs(min(p[0] for p in ring) - (-1.0)) < 1e-9
    sh.filter(State(x=-50, y=0, up=5), Action4D(), t=2.0)
    assert abs(min(p[0] for p in _ring(sh, "drift")) - 1.0) < 1e-9


# ------------------------------------------------------------- patch validation

def _relint(sh):
    """The live policy, re-read the way a per-generation snapshot is."""
    snap = Policy.model_validate(sh.policy.model_dump(mode="json", exclude_none=True))
    return snap.lint()


def test_an_event_that_would_leave_the_policy_inconsistent_is_refused():
    """The DSL's own consistency lint (Policy.lint, which every loader runs)
    is the patch validator: an event may not produce a policy the loaders
    would refuse, or a generation's snapshot could not be loaded again.
    Expiring a zone a switch still targets would leave the switch dangling."""
    sh = Shield(load_policy(DEMO))
    sh.filter(State(x=-20, y=-20, up=4), Action4D())
    sh.hot_apply(_zone())
    sh.set_rule_active("dyn", False, switch_id="dyn-off")
    assert _relint(sh) == []
    e = _raises(HotApplyRefused, sh.expire_nfz, "dyn")
    assert "dyn-off" in str(e) and "names no rule" in str(e), e
    assert sh.policy.generation == 2 and _relint(sh) == []
    sh.set_rule_active("dyn", True, switch_id="dyn-on")       # supersedes dyn-off
    assert _relint(sh) == []


def test_every_generation_an_event_sequence_produces_lints_clean():
    sh = Shield(_lanes())
    sh.filter(State(x=0, y=0, up=10), Action4D())
    steps = [lambda: sh.hot_apply(_zone(cx=50.0, cy=50.0)),
             lambda: sh.translate_nfz("dyn", 3.0, 0.0),
             lambda: sh.hot_apply(_swap("lane-b", 40.0)),
             lambda: sh.set_rule_active("dyn", False),
             lambda: sh.set_rule_active("dyn", True),
             lambda: sh.hot_apply(_swap("lane-c", -40.0)),
             lambda: sh.scale_nfz("dyn", 1.5)]
    for k, step in enumerate(steps, 1):
        step()
        assert sh.policy.generation == k and _relint(sh) == [], (k, _relint(sh))


# --------------------------------------------------------------------- atomicity

def test_an_event_never_changes_a_tick_already_running():
    """An event that lands while filter() is running (the REST thread) must not
    be half-seen: the running tick keeps the rule set it pinned, the next one
    has the new set."""
    class EventMidTick(Shield):
        fired = False

        def _repair_geofence(self, state, a, repairs):
            if not self.fired:
                self.fired = True
                self.hot_apply(_zone(zid="late", cx=0.0, cy=15.0, half=3.0))
            return super()._repair_geofence(state, a, repairs)

    sh = EventMidTick(load_policy(DEMO))
    st, a = State(x=0, y=15, up=4), Action4D(vx=1.3)
    d = sh.filter(st, a)
    assert sh.policy.generation == 1               # the event did land...
    assert d.generation == 0                       # ...after this tick pinned its rules
    assert "late" not in [v.rule_id for v in d.violations + d.emitted_violations]
    # Not even the repairs saw it: the tick decides exactly as a Shield the
    # event never reached (the aircraft is inside "late", whose escape would
    # otherwise rewrite the action).
    ref = Shield(load_policy(DEMO)).filter(st, a)
    assert d.emitted == ref.emitted, (d.emitted, ref.emitted)
    assert [r.detail for r in d.repairs] == [r.detail for r in ref.repairs], d.repairs
    d2 = sh.filter(State(x=0, y=15, up=4), Action4D())   # inside the new zone
    assert d2.generation == 1 and "late" in [v.rule_id for v in d2.violations]


def test_hot_apply_fits_the_grants_50_ms_at_50_rules():
    """Policy DSL page, acceptance KPIs: "Hot-apply latency (dynamic_nfz spawn)
    <= 50 ms". Median of 30 spawns on the 50-rule load, desktop."""
    import statistics
    from bench_shield_50rules import fifty_rule_policy
    ms = []
    for i in range(30):
        sh = Shield(fifty_rule_policy())
        sh.filter(State(x=-50, y=-50, up=20), Action4D())
        t0 = time.perf_counter()
        sh.hot_apply(_zone(zid=f"hot-{i}", cx=-25.0, cy=-25.0, half=5.0))
        ms.append((time.perf_counter() - t0) * 1e3)
    assert statistics.median(ms) <= 50.0, statistics.median(ms)


# -------------------------------------------------------------------------- audit

def test_the_audit_record_carries_generation_fsm_state_and_the_grants_fields():
    tmp = Path(tempfile.mkdtemp())
    pol = load_policy(DEMO)
    sh = Shield(pol)
    al = AuditLogger(tmp / "audit.jsonl", pol)
    st, a = State(x=0, y=15, up=4), Action4D(vx=1.3)
    al.log(1, sh.filter(st, a))
    sh.hot_apply(_zone())
    al.log(2, sh.filter(State(x=-20, y=-20, up=4), Action4D(vx=3.0)))
    recs = [json.loads(ln) for ln in (tmp / "audit.jsonl").read_text(encoding="utf-8").splitlines()]
    assert [r["generation"] for r in recs] == [0, 1]
    assert recs[1]["policy_hash"] == pol.policy_hash != recs[0]["policy_hash"]
    r = recs[0]
    for k in ("ts", "policy_hash", "generation", "monitor_tick", "raw_action",
              "violations", "repair_attempts", "emitted_action",
              "fsm_state_before", "fsm_state_after"):
        assert k in r, k
    assert (r["fsm_state_before"], r["fsm_state_after"], r["fsm_edge"]) == ("Normal", "Brake", "G1")
    att = r["repair_attempts"][0]
    assert att["operator"] == "GeofenceSlide" and att["axis"] == "lateral"
    assert abs(att["magnitude_m"] - 0.5) < 1e-6 and att["result"] == "ok"
    v = r["violations"][0]
    assert v["boundary_intersect_at_s"] == v["predicted_at_s"] and v["grant_category"] == "geometric"
    assert r["fsm_config_hash"].startswith("sha256:")


def test_a_record_written_after_an_event_keeps_the_rules_it_was_decided_under():
    """The rail logs a decision after filter() returns; an event can land in
    between (the REST thread). The record must carry the generation and hash
    the decision was made under, not the live policy's."""
    tmp = Path(tempfile.mkdtemp())
    pol = load_policy(DEMO)
    sh = Shield(pol)
    al = AuditLogger(tmp / "audit.jsonl", pol)
    d = sh.filter(State(x=0, y=15, up=4), Action4D(vx=1.3))
    h0 = pol.policy_hash
    sh.hot_apply(_zone())                          # lands before the log call
    al.log(1, d)
    rec = json.loads((tmp / "audit.jsonl").read_text(encoding="utf-8"))
    assert (rec["generation"], rec["policy_hash"]) == (0, h0) != (1, pol.policy_hash)


def test_a_recovery_on_a_clean_tick_is_on_record():
    """Brake -> Normal (G7) happens on a tick with no violation; the audit
    used to write only touched ticks, so the recovery was invisible."""
    tmp = Path(tempfile.mkdtemp())
    pol = load_policy(DEMO)
    sh = Shield(pol)
    al = AuditLogger(tmp / "audit.jsonl", pol)
    al.log(0, sh.filter(State(x=0, y=15, up=4), Action4D(vx=1.3)))
    for k in range(1, 30):
        al.log(k, sh.filter(State(x=-20, y=-20, up=4), Action4D(vx=2.0)))
    recs = [json.loads(ln) for ln in (tmp / "audit.jsonl").read_text(encoding="utf-8").splitlines()]
    assert [r["fsm_edge"] for r in recs] == ["G1", "G7"], [r["fsm_edge"] for r in recs]
    assert recs[1]["violations"] == []


def test_one_audit_file_per_episode():
    tmp = Path(tempfile.mkdtemp())
    pol = load_policy(DEMO)
    al = AuditLogger.for_episode(tmp, pol, "ep-001")
    sh = Shield(pol)
    al.log(1, sh.filter(State(x=0, y=15, up=4), Action4D(vx=1.3)))
    assert (tmp / "audit_ep-001.jsonl").is_file()
    again = AuditLogger.for_episode(tmp, pol, "ep-001")        # fresh, not appended
    assert again.records_written == 0
    assert (tmp / "audit_ep-001.jsonl").read_text(encoding="utf-8") == ""
    again.log(2, sh.filter(State(x=0, y=15, up=4), Action4D(vx=1.3)))
    p2 = again.rotate("ep-002")
    assert p2.name == "audit_ep-002.jsonl" and p2.read_text(encoding="utf-8") == ""
    rec = json.loads((tmp / "audit_ep-001.jsonl").read_text(encoding="utf-8"))
    assert rec["episode_id"] == "ep-001"
    _raises(ValueError, AuditLogger.for_episode, tmp, pol, "../x")


# ------------------------------------------------------------- the layer rule
#
# Policy DSL page, "Layered authoring model": a mission-layer rule cannot relax
# a higher-priority hard constraint from regulation. models.merge_layers
# refuses that at ingest; the 2026-10-07 review showed every event could still
# do it mid-flight (a mission switch turned a hard regulation zone off and a
# 4 m/s command flew into it unrepaired). Each test below fails on the code of
# that review (scratchpad copy, docs/DESIGN-shield-enforcement.md).

_SQ = [{"x": 7, "y": 7}, {"x": 23, "y": 7}, {"x": 23, "y": 23}, {"x": 7, "y": 23}]


def _layered(extra_reg=(), extra_mis=()):
    """A regulation layer (hard airport zone, soft park zone) merged with a
    mission layer (altitude band, hard mission zone). Built fresh each call:
    the Shield edits the policy it is given."""
    from guardrail.models import merge_layers
    reg = Policy.model_validate({"policy_id": "reg", "version": "0.1.0", "constraints": [
        {"id": "nfz-airport", "type": "polygon_fence", "layer": "regulation",
         "vertices": _SQ},
        {"id": "nfz-park", "type": "polygon_fence", "layer": "regulation",
         "constraint_type": "soft", "priority": "P2",
         "vertices": [{"x": -40, "y": 7}, {"x": -30, "y": 7}, {"x": -30, "y": 23},
                      {"x": -40, "y": 23}]},
        *extra_reg]})
    mis = Policy.model_validate({"policy_id": "mis", "version": "0.1.0", "constraints": [
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 2, "alt_max_m": 40},
        {"id": "nfz-mission", "type": "polygon_fence",
         "vertices": [{"x": 50, "y": 7}, {"x": 60, "y": 7}, {"x": 60, "y": 23},
                      {"x": 50, "y": 23}]},
        *extra_mis]})
    return merge_layers([reg, mis])


def test_a_switch_may_not_turn_off_a_hard_rule_of_another_layer():
    sh = Shield(_layered())
    st, a = State(x=0, y=15, up=4), Action4D(vx=4.0)
    sh.filter(st, a)
    g = sh.policy.generation
    e = _raises(LayerRelaxation, sh.set_rule_active, "nfz-airport", False)
    assert "hard regulation-layer rule" in str(e)
    _raises(LayerRelaxation, sh.hot_apply, TimeWindowSwitch(
        id="sw", type="time_window_switch", target_id="nfz-airport", active=False))
    assert sh.policy.generation == g and sh.events == []          # nothing applied
    d = sh.filter(st, a)
    assert [v.rule_id for v in d.violations] == ["nfz-airport"]   # still enforced
    assert d.repairs and d.emitted != d.raw
    # What the layer rule allows: tightening, a soft rule of any layer, a rule
    # of the switch's own layer, and the regulation layer switching its own.
    assert sh.set_rule_active("nfz-airport", True)["op"] == "switch_on"
    assert sh.set_rule_active("nfz-park", False)["op"] == "switch_off"
    assert sh.set_rule_active("nfz-mission", False)["op"] == "switch_off"
    reg = Shield(_layered())
    assert reg.set_rule_active("nfz-airport", False, layer="regulation")["op"] == "switch_off"


def test_a_switch_may_not_override_another_layers_switch_holding_a_rule_on():
    """The last switch in force decides, so appending a mission switch-off
    after a regulation switch that holds the rule on would overrule it."""
    sh = Shield(_layered(extra_reg=[{"id": "hold", "type": "time_window_switch",
                                     "layer": "regulation", "target_id": "nfz-mission",
                                     "active": True}]))
    e = _raises(LayerRelaxation, sh.set_rule_active, "nfz-mission", False)
    assert "held on by ['hold']" in str(e)


def test_a_swap_may_not_loosen_a_hard_corridor_of_another_layer():
    """A mission swap of a hard regulation corridor must be provably no looser
    (models._not_looser, as merge_layers builds it): a 2 km wide, 1 km tall,
    monitor_only replacement is refused; a narrower one inside the old tube is
    accepted and enforced."""
    from guardrail.models import merge_layers
    reg = Policy.model_validate({"policy_id": "reg", "version": "0.1.0", "constraints": [
        {"id": "lane", "type": "corridor", "layer": "regulation", "width_m": 20,
         "centerline": [{"x": -100, "y": 0}, {"x": 100, "y": 0}],
         "altitude_floor_m": 5, "altitude_ceiling_m": 30}]})
    mis = Policy.model_validate({"policy_id": "mis", "version": "0.1.0", "constraints": [
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 2, "alt_max_m": 40}]})
    sh = Shield(merge_layers([reg, mis]))
    sh.filter(State(x=0, y=0, up=10), Action4D())
    wide = CorridorSwap.model_validate({
        "id": "wide", "type": "corridor_swap", "target_id": "lane", "width_m": 2000,
        "centerline": [{"x": -100, "y": 0}, {"x": 100, "y": 0}],
        "altitude_floor_m": 0, "altitude_ceiling_m": 1000, "constraint_type": "soft",
        "priority": "P2", "violation_action": "monitor_only"})
    e = _raises(LayerRelaxation, sh.hot_apply, wide)
    assert "hard regulation-layer corridor 'lane'" in str(e)
    d = sh.filter(State(x=0, y=300, up=10), Action4D(vy=1.0))
    assert "lane" in {v.rule_id for v in d.violations}           # the original holds
    narrow = CorridorSwap.model_validate({
        "id": "narrow", "type": "corridor_swap", "target_id": "lane", "width_m": 10,
        "centerline": [{"x": -50, "y": 0}, {"x": 50, "y": 0}],
        "altitude_floor_m": 8, "altitude_ceiling_m": 25})
    assert sh.hot_apply(narrow)["op"] == "swap"
    d = sh.filter(State(x=0, y=8, up=10), Action4D(vy=1.0))
    assert [v.rule_id for v in d.violations] == ["narrow"]


def test_a_zone_of_another_layer_cannot_be_moved_reshaped_or_expired():
    """A hard regulation-layer dynamic_nfz (a temporary flight restriction):
    a mission-layer translate, scale or expire is refused; the same edit from
    the regulation layer (Python API, layer=) goes through."""
    from guardrail.models import merge_layers
    reg = Policy.model_validate({"policy_id": "reg", "version": "0.1.0", "constraints": [
        {"id": "tfr", "type": "dynamic_nfz", "layer": "regulation", "vertices": _SQ}]})
    mis = Policy.model_validate({"policy_id": "mis", "version": "0.1.0", "constraints": [
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 2, "alt_max_m": 40}]})
    sh = Shield(merge_layers([reg, mis]))
    sh.filter(State(x=0, y=15, up=4), Action4D())
    for fn, args in ((sh.translate_nfz, ("tfr", 100.0, 0.0)), (sh.scale_nfz, ("tfr", 0.1)),
                     (sh.rotate_nfz, ("tfr", 45.0)), (sh.expire_nfz, ("tfr",)),
                     (sh.move_nfz, ("tfr", [(200, 200), (201, 200), (201, 201)]))):
        _raises(LayerRelaxation, fn, *args)
    assert sh.policy.generation == 0
    assert sh.translate_nfz("tfr", 1.0, 0.0, layer="regulation")["op"] == "translate"


def test_the_last_switch_in_force_decides():
    """Two scheduled switches on one zone, overlapping 11:00-12:00: the later
    one in policy order decides while both are in force. Off-then-on leaves
    the zone on at 11:30, on-then-off leaves it off; at 10:30 only the first
    is in force."""
    def sw(sid, active, start):
        return TimeWindowSwitch.model_validate({
            "id": sid, "type": "time_window_switch", "target_id": "nfz-square",
            "active": active, "valid_time": {"recurrence": {
                "start_time": start, "end_time": "12:00"}}})
    st = State(x=15, y=15, up=4)
    for first, then in ((False, True), (True, False)):
        for hhmm, want in (("10:30", first), ("11:30", then)):
            when = datetime(2026, 10, 7, int(hhmm[:2]), int(hhmm[3:]))
            sh = Shield(load_policy(DEMO), now=lambda w=when: w)
            sh.hot_apply(sw("s1", first, "10:00"))
            sh.hot_apply(sw("s2", then, "11:00"))
            row = next(r for r in sh.rule_status(st) if r["id"] == "nfz-square")
            assert row["in_force"] is want, (first, then, hhmm, row["in_force"])


# --------------------------------------------------------------- reset_episode

def test_a_moving_zone_carries_on_across_reset_episode():
    """The review's case: a 2 m/s zone spawned 120 s into an episode on the
    counted clock. Before the fix, reset_episode() sent it back to its spawn
    position and froze it until the new episode's clock passed 120 s. Now it
    continues from where it was: 5 s old at the reset, 14.9 s old after 100
    more ticks. On a clock that runs on through the reset (t passed in), the
    zone keeps moving in real time, gap included."""
    st = State(x=-40, y=-40, up=4)
    z = _zone("mv", cx=-100.0, cy=-40.0, half=3.0, motion={"vx_mps": 2.0})
    sh = Shield(load_policy(DEMO))
    for _ in range(1200):
        sh.filter(st, Action4D())
    sh.spawn_nfz(z)
    for _ in range(50):
        sh.filter(st, Action4D())
    assert _close(_ring(sh, "mv", st), z.ring_at(5.0), 1e-6)
    sh.reset_episode()
    assert _close(_ring(sh, "mv", st), z.ring_at(5.0), 1e-6)    # between episodes
    for _ in range(100):
        sh.filter(st, Action4D())
    assert _close(_ring(sh, "mv", st), z.ring_at(14.9), 1e-6), _ring(sh, "mv", st)
    # A monotonic clock that continues: spawned at t = 10, reset at t = 12,
    # next episode starts at t = 20 -> 10 s old.
    sh = Shield(load_policy(DEMO))
    sh.filter(st, Action4D(), t=9.0)
    sh.spawn_nfz(z, t=10.0)
    sh.filter(st, Action4D(), t=12.0)
    sh.reset_episode()
    sh.filter(st, Action4D(), t=20.0)
    assert _close(_ring(sh, "mv", st), z.ring_at(10.0), 1e-6), _ring(sh, "mv", st)


def test_reset_episode_lifts_the_mission_start_lock():
    """Mission start is the first tick of an EPISODE: after reset_episode() a
    polygon_fence is part of the next mission's start policy again."""
    from guardrail.models import PolygonFence
    pf = PolygonFence.model_validate({"id": "pf", "type": "polygon_fence",
                                      "vertices": _sq(-60.0, -60.0, 3.0)})
    sh = Shield(load_policy(DEMO))
    sh.filter(State(x=-20, y=-20, up=4), Action4D())
    assert sh.mission_started
    _raises(LockedRuleClass, sh.hot_apply, pf)
    sh.reset_episode()
    assert not sh.mission_started
    assert sh.hot_apply(pf)["op"] == "add_before_start"


def test_the_migration_keeps_the_zones_breach_action_type_and_priority():
    """MIGRATION_NOTE promises as_dynamic_nfz keeps id, vertices, band,
    margin, priority and breach action; the sweep's migration relies on it."""
    from guardrail.models import PolygonFence
    pf = PolygonFence.model_validate({
        "id": "pf", "type": "polygon_fence", "violation_action": "RTL",
        "constraint_type": "soft", "priority": "P1", "margin_m": 2.0,
        "altitude_floor_m": 3.0, "altitude_ceiling_m": 50.0,
        "vertices": [{"x": 0, "y": 0}, {"x": 4, "y": 0}, {"x": 4, "y": 4}]})
    z = as_dynamic_nfz(pf)
    assert (z.id, z.type, z.violation_action, z.constraint_type, z.priority) == \
        ("pf", "dynamic_nfz", "RTL", "soft", "P1")
    assert (z.margin_m, z.altitude_floor_m, z.altitude_ceiling_m, z.motion) == \
        (2.0, 3.0, 50.0, None)
    assert [(v.x, v.y) for v in z.vertices] == [(v.x, v.y) for v in pf.vertices]


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}  {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}  {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
