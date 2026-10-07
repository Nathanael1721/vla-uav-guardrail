"""The compiled fence IR, and the claim that indexing it changed no decision.

Run either way:
    pytest tests/test_ir.py -v
    python tests/test_ir.py

WHAT IS BEING CLAIMED

guardrail/ir.py says the STRtree only decides which fences the Shield LOOKS AT,
never what it finds: a fence the index leaves out could not have contained any
forecast pose. If that is true, the indexed Shield and the old walk over every
fence must agree exactly - same violations in the same order with the same
prose, same repairs, same emitted action - on any input whatsoever.

So the main test here does not test the index against a hand-picked case. It
runs the new Shield and a verbatim copy of the old fence loop
(tests/legacy_fence_loop.py) side by side over thousands of seeded states and
actions, across every policy in policies/, a 50-rule policy at the grant's
design load, overlapping fences, a clock that ticks between calls, a fence
hot-applied mid-stream, and non-finite actions, and requires every answer to be
identical.

AND THAT THE COMPARISON CAN FAIL

An equivalence test whose samples never come near a fence would pass on a
Shield that ignored fences altogether. Two guards:

  * coverage counts - the samples must actually raise geofence violations,
    GeofenceEscape and GeofenceSlide repairs, and the index must actually have
    left fences out (otherwise it was never exercised);
  * a deliberately broken index (built on the authored polygons, blind to the
    margin ring) must be CAUGHT by the same harness.
"""
from __future__ import annotations

import math
import random
import sys
from datetime import datetime, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "demo"))
sys.path.insert(0, str(ROOT / "experiments"))

from shapely.geometry import box as _box                            # noqa: E402

import guardrail.ir as IRmod                                        # noqa: E402
from guardrail import load_policy                                   # noqa: E402
from guardrail.geometry import fence_polygon                        # noqa: E402
from guardrail.ir import FenceRecord, PolicyIR, compile_fence       # noqa: E402
from guardrail.models import (Action4D, AltitudeEnvelope,            # noqa: E402
                              KinematicEnvelope, ObstacleClearance,
                              Policy, PolygonFence, State,
                              SubjectStandoff)
from guardrail.shield import Shield, as_dynamic_nfz                 # noqa: E402
from legacy_fence_loop import LegacyFenceShield                     # noqa: E402

POLICIES = sorted((ROOT / "policies").glob("*.yaml"))
CITYMAP = ROOT / "demo" / "out" / "citymap" / "occ_day.npz"


def _obstacle_map():
    """Same source as tests/test_check_contract.py; None where the map is not
    on this machine (clearance rules are then inert on BOTH sides)."""
    if not CITYMAP.is_file():
        return None
    import city_planner
    cm = city_planner.load_occ(str(CITYMAP))
    return {"occ": cm["occ"], "res": cm["res"], "ox": cm["ox"], "oy": cm["oy"]}


def _square(fid, x0, y0, side, margin=1.0, floor=0.0, ceil=120.0, **kw):
    return PolygonFence(id=fid, type="polygon_fence",
                        vertices=[{"x": x0, "y": y0}, {"x": x0 + side, "y": y0},
                                  {"x": x0 + side, "y": y0 + side},
                                  {"x": x0, "y": y0 + side}],
                        altitude_floor_m=floor, altitude_ceiling_m=ceil,
                        margin_m=margin, **kw)


def _caps(speed=4.0):
    return [KinematicEnvelope(id="kin", type="kinematic_envelope", speed_max_mps=speed,
                              climb_rate_max_mps=2.0, yaw_rate_max_dps=60.0),
            AltitudeEnvelope(id="alt", type="altitude_envelope",
                             alt_min_m=2.0, alt_max_m=40.0)]


def _overlap_policy() -> Policy:
    """Fences that overlap, touch and nest, so a repair against one can aim the
    vehicle at the next - the case the per-iteration re-query exists for."""
    return Policy(policy_id="overlap", constraints=[
        *_caps(),
        _square("a", 0, 0, 20, margin=2.0),
        _square("b", 15, 5, 20, margin=1.0),          # overlaps a
        _square("c", 36, 0, 10, margin=0.5),          # touches b's ring
        _square("d", 5, 5, 4, margin=0.0),            # nested in a
        _square("e", -30, -30, 8, margin=3.0, floor=10.0, ceil=20.0),
        _square("f", -12, 4, 3, margin=-2.0),         # ring erased by the margin
    ])


# ------------------------------------------------------------------ sampling

def _extent(pol: Policy):
    pts = [(v.x, v.y) for f in pol.by_type(PolygonFence) for v in f.vertices]
    for c in pol.constraints:
        if hasattr(c, "points") and callable(c.points):
            pts += list(c.points())
    if not pts:
        return (-60.0, -60.0, 60.0, 60.0)
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return (min(xs) - 30, min(ys) - 30, max(xs) + 30, max(ys) + 30)


def _pairs(pol: Policy, n: int, seed: str):
    """(state, action, subject) triples, three quarters of them aimed at a
    fence: near its ring, ON its ring (to the nanometre), or inside it."""
    rng = random.Random(seed)
    fences = pol.by_type(PolygonFence)
    rings = [fence_polygon(f).buffer(f.margin_m) for f in fences]
    has_so = bool(pol.by_type(SubjectStandoff))
    ex = _extent(pol)
    out = []
    for i in range(n):
        mode = i % 4
        k = rng.randrange(len(fences)) if fences else -1
        if fences and mode < 3 and not rings[k].is_empty:
            f, ring = fences[k], rings[k]
            minx, miny, maxx, maxy = ring.bounds
            if mode == 0:
                x, y = rng.uniform(minx - 15, maxx + 15), rng.uniform(miny - 15, maxy + 15)
            elif mode == 1:
                p = ring.exterior.interpolate(rng.random(), normalized=True)
                j = rng.choice([0.0, 1e-9, -1e-9, rng.uniform(-0.6, 0.6)])
                x, y = p.x + j, p.y + rng.choice([0.0, j, -j])
            else:
                x, y = rng.uniform(minx, maxx), rng.uniform(miny, maxy)
            up = rng.choice([rng.uniform(-1.0, 35.0), rng.uniform(-1.0, 35.0),
                             f.altitude_floor_m, f.altitude_ceiling_m])
        else:
            x, y = rng.uniform(ex[0], ex[2]), rng.uniform(ex[1], ex[3])
            up = rng.uniform(-1.0, 35.0)
        roll = rng.random()
        if roll < 0.08:
            a = Action4D()
        elif roll < 0.16:
            hd = rng.uniform(0, 2 * math.pi)
            a = Action4D(vx=15 * math.cos(hd), vy=15 * math.sin(hd),
                         vz_up=rng.uniform(-6, 6), yaw_rate=rng.uniform(-3, 3))
        else:
            a = Action4D(vx=rng.uniform(-6, 6), vy=rng.uniform(-6, 6),
                         vz_up=rng.uniform(-3, 3), yaw_rate=rng.uniform(-1.5, 1.5))
        subj = None
        if has_so and rng.random() < 0.5:
            subj = (x + rng.uniform(-14, 14), y + rng.uniform(-14, 14),
                    rng.choice(["pedestrian", "car", "thing"]))
        out.append((State(x=x, y=y, up=up), a, subj))
    return out


# ------------------------------------------------------------------ harness

class Tally:
    def __init__(self):
        self.compared = self.mismatched = 0
        self.geofence = self.escape = self.slide = self.braked = self.raised = 0
        self.skipped_fences = self.fence_slots = 0
        self.first_mismatch = None

    def note(self, where, a, b):
        self.compared += 1
        if a != b:
            self.mismatched += 1
            if self.first_mismatch is None:
                self.first_mismatch = (where, a, b)


def _vdump(vs):
    return [v.model_dump() for v in vs]


# Repair fields that are RECORD, not behaviour: the size of the correction an
# operator reports for the escalation FSM's theta (added 2026-10-07). The
# frozen old loop (tests/legacy_fence_loop.py) cannot report them, so the
# equivalence compares everything else - the emitted action, every violation,
# every operator and its detail, braked, the re-check.
_RECORD_ONLY = ("magnitude_m", "axis", "recovery")


def _ddump(d):
    out = d.model_dump()
    out["repairs"] = [{k: v for k, v in r.items() if k not in _RECORD_ONLY}
                      for r in out["repairs"]]
    return out


def _call(fn, *args):
    """(result, None) or (None, "ExcType: message"). A crash is an answer too:
    `_check` called directly with a NaN action raises inside the clearance
    distance lookup on both sides (filter() sanitises first, so flights never
    reach it). The two Shields must crash identically, not just agree when
    neither crashes."""
    try:
        return fn(*args), None
    except Exception as e:                           # noqa: BLE001
        return None, f"{type(e).__name__}: {e}"


def compare(new: Shield, old: Shield, pairs, tally: Tally, label: str,
            hot_apply_at: int | None = None, hot_fence: PolygonFence | None = None,
            nonfinite_every: int = 0, tick_clock=None):
    for i, (st, a, subj) in enumerate(pairs):
        if hot_apply_at is not None and i == hot_apply_at:
            # Mid-run, so the grant's hot-applicable form (a polygon_fence is
            # locked once the mission has started; see test_shield.py).
            new.hot_apply(as_dynamic_nfz(hot_fence))
            old.hot_apply(as_dynamic_nfz(hot_fence))
        if tick_clock is not None:
            tick_clock.advance()
        bad = bool(nonfinite_every) and i % nonfinite_every == nonfinite_every - 1
        if bad:
            a = Action4D(vx=float("nan") if i % 2 else float("inf"), vy=a.vy,
                         vz_up=a.vz_up, yaw_rate=a.yaw_rate)
        for sh in (new, old):
            sh.set_subject(*subj) if subj else sh.set_subject(None)
        # filter()'s decision already carries two _check answers (`violations`
        # on the raw action, `emitted_violations` on what it emits), so the
        # monitor is compared directly only on every third sample - and on
        # every non-finite one, which filter() would sanitise before checking.
        # The old loop is ~100x slower; this keeps the run near a minute.
        if bad or i % 3 == 0:
            (cn, en), (co, eo) = _call(new._check, st, a), _call(old._check, st, a)
            tally.note(f"{label}#{i} _check", (_vdump(cn) if en is None else en),
                       (_vdump(co) if eo is None else eo))
            tally.raised += en is not None
        (dn, en), (do, eo) = _call(new.filter, st, a), _call(old.filter, st, a)
        tally.note(f"{label}#{i} filter",
                   (_ddump(dn) if en is None else en),
                   (_ddump(do) if eo is None else eo))
        tally.raised += en is not None
        if dn is not None:
            tally.geofence += any(v.category == "geofence" for v in dn.violations)
            tally.escape += any(r.operator == "GeofenceEscape" for r in dn.repairs)
            tally.slide += any(r.operator == "GeofenceSlide" for r in dn.repairs)
            tally.braked += dn.braked
        if all(math.isfinite(v) for v in (a.vx, a.vy)):
            n_all = len(new.ir.fences)
            near = new._fence_candidates(new.ir, st, new.forecast(st, a))
            tally.fence_slots += n_all
            tally.skipped_fences += n_all - len(near)


def _shields(pol: Policy, smap, cls_new=Shield, now_new=None, now_old=None,
             lookahead_s=3.0, dt=0.5):
    # No escalation FSM on either side: this file is about which fences are
    # looked at and what the repairs do (the FSM is tests/test_fsm.py and
    # tests/test_shield.py).
    m = smap if pol.by_type(ObstacleClearance) else None
    return (cls_new(pol.model_copy(deep=True), lookahead_s=lookahead_s, dt=dt,
                    obstacle_map=m, now=now_new, escalation=False),
            LegacyFenceShield(pol.model_copy(deep=True), lookahead_s=lookahead_s,
                              dt=dt, obstacle_map=m, now=now_old, escalation=False))


class _TickClock:
    """A clock that advances once per SAMPLE (the harness calls advance()),
    not per read.

    Until 2026-10-07 this clock advanced on every read, to catch a Shield that
    read the clock a different number of times than the old loop. The Shield
    now reads it once per tick on purpose (guardrail/shield.py, _Schedule), so
    read counts are no longer equal by design; what must still agree is the
    rule set at each tick, which a per-sample clock compares."""

    def __init__(self, start: datetime, step: timedelta):
        self.t, self.step, self._first = start, step, True

    def advance(self):
        if self._first:
            self._first = False
        else:
            self.t = self.t + self.step

    def __call__(self):
        return self.t


# ------------------------------------------------------------------ IR units

def test_margin_ring_is_built_once_and_is_the_same_ring():
    """The cached ring must be the ring the old per-test rebuild produced -
    coordinate for coordinate, not merely 'close'."""
    n = 0
    for path in POLICIES:
        pol = load_policy(path)
        ir = PolicyIR.from_policy(pol)
        assert len(ir.fences) == len(pol.by_type(PolygonFence)), path.name
        for rec, f in zip(ir.fences, pol.by_type(PolygonFence)):
            assert rec.rule is f
            want = fence_polygon(f).buffer(f.margin_m)
            assert rec.buffered.wkb == want.wkb, f"{path.name}:{f.id} ring differs"
            assert rec.polygon.wkb == fence_polygon(f).wkb
            n += 1
    assert n >= 40, f"only {n} fences across policies/ - the check covers too little"


def test_box_query_is_a_superset_of_exact_intersection_in_policy_order():
    """Every fence whose ring actually meets the box must come back - the
    index may over-include (envelopes), never under-include."""
    pol = _overlap_policy()
    from bench_shield_50rules import fifty_rule_policy
    rng = random.Random(7)
    for p in (pol, fifty_rule_policy()):
        ir = PolicyIR.from_policy(p)
        assert ir.indexed
        for _ in range(3000):
            x, y = rng.uniform(-60, 320), rng.uniform(-60, 240)
            w, h = rng.choice([0.0, rng.uniform(0, 3), rng.uniform(0, 60)]), rng.uniform(0, 40)
            got = ir.fences_in_box(x, y, x + w, y + h)
            idx = [r.index for r in got]
            assert idx == sorted(idx), "not in policy order"
            exact = {r.index for r in ir.fences
                     if not r.buffered.is_empty and r.buffered.intersects(_box(x, y, x + w, y + h))}
            assert exact <= set(idx), f"index dropped {exact - set(idx)}"


def test_tree_and_brute_force_return_the_same_fences():
    """The fallback for shapely 1.x must be the same filter, not a looser one."""
    from bench_shield_50rules import fifty_rule_policy
    pol = fifty_rule_policy()
    tree = PolicyIR.from_policy(pol)
    saved = IRmod._STRtree
    try:
        IRmod._STRtree = None
        brute = PolicyIR.from_policy(pol)
    finally:
        IRmod._STRtree = saved
    assert tree.indexed and not brute.indexed
    rng = random.Random(11)
    for _ in range(3000):
        x, y = rng.uniform(-60, 330), rng.uniform(-60, 250)
        w, h = rng.uniform(0, 50), rng.uniform(0, 50)
        a = [r.index for r in tree.fences_in_box(x, y, x + w, y + h)]
        b = [r.index for r in brute.fences_in_box(x, y, x + w, y + h)]
        assert a == b, (x, y, w, h, a, b)


def test_non_finite_query_returns_every_fence():
    """NaN compares false against every envelope; a NaN box must not quietly
    drop the fence the vehicle is already inside."""
    ir = PolicyIR.from_policy(_overlap_policy())
    for bad in (float("nan"), float("inf"), -float("inf")):
        assert len(ir.fences_in_box(bad, 0, 1, 1)) == len(ir.fences)
        assert len(ir.fences_for_points([0.0, bad], [0.0, 1.0])) == len(ir.fences)
    assert len(ir.fences_for_points([], [])) == len(ir.fences)


def test_an_erased_ring_is_never_returned_and_never_fires():
    """A negative margin can erase a small fence; empty geometry has NaN bounds.
    Both the tree and the fallback must leave it out, and the Shield must agree
    with the old loop that it forbids nothing."""
    pol = _overlap_policy()
    ir = PolicyIR.from_policy(pol)
    f = next(r for r in ir.fences if r.rule.id == "f")
    assert f.buffered.is_empty
    assert f not in ir.fences_in_box(-100, -100, 100, 100)
    sh = Shield(pol.model_copy(deep=True))
    old = LegacyFenceShield(pol.model_copy(deep=True))
    st = State(x=-10.5, y=5.5, up=5.0)         # inside the authored polygon of f
    assert sh._check(st, Action4D()) == old._check(st, Action4D())
    assert not any(v.rule_id == "f" for v in sh._check(st, Action4D()))


def test_empty_policy_has_an_empty_index():
    ir = PolicyIR.from_policy(Policy(policy_id="none", constraints=_caps()))
    assert ir.fences == [] and not ir.indexed
    assert ir.fences_in_box(-1, -1, 1, 1) == []
    assert ir.polygons_near(0, 0, 1e6) == []


def test_polygons_near_matches_the_reference_meaning():
    """Reference policy_dsl/ir.py: fences whose envelope is within radius_m."""
    ir = PolicyIR.from_policy(_overlap_policy())
    ids = {r.rule.id for r in ir.polygons_near(-26.0, -26.0, 1.0)}
    assert ids == {"e"}, ids
    assert {r.rule.id for r in ir.polygons_near(100.0, 100.0, 5.0)} == set()
    assert {r.rule.id for r in ir.polygons_near(100.0, 100.0, 200.0)} >= {"a", "b", "c", "d", "e"}


def test_with_fence_reuses_records_and_leaves_the_old_ir_alone():
    pol = _overlap_policy()
    ir0 = PolicyIR.from_policy(pol)
    new = _square("hot", 100, 100, 5)
    pol.constraints.append(new)
    pol.generation += 1
    ir1 = ir0.with_fence(new, pol)
    assert len(ir0.fences) == 6 and len(ir1.fences) == 7      # old IR untouched
    assert all(a is b for a, b in zip(ir0.fences, ir1.fences))  # nothing re-buffered
    assert ir1.fences[-1].index == 6 and ir1.fences[-1].rule is new
    assert [r.rule.id for r in ir1.fences_in_box(101, 101, 102, 102)] == ["hot"]
    assert ir0.fences_in_box(101, 101, 102, 102) == []
    assert ir1.generation == pol.generation and ir1.policy_hash == pol.policy_hash
    assert ir1.is_current_for(pol) and not ir0.is_current_for(pol)


def test_shield_rebuilds_the_ir_on_hot_apply():
    """A tree that was not rebuilt would never return the new fence, and the
    zone would be enforced by nothing (test_shield.py's hot-apply test is the
    behavioural half of this)."""
    pol = _overlap_policy()
    sh = Shield(pol)
    assert sh.ir.is_current_for(sh.policy)
    sh.hot_apply(_square("hot", 100, 100, 5))
    assert sh.ir.is_current_for(sh.policy)
    st, a = State(x=90.0, y=102.0, up=5.0), Action4D(vx=4.0)
    assert any(v.rule_id == "hot" for v in sh._check(st, a))


def test_compile_fence_records_the_ring_envelope():
    f = _square("z", 10, 20, 4, margin=1.5)
    rec = compile_fence(3, f)
    assert isinstance(rec, FenceRecord) and rec.index == 3
    assert rec.bounds == (8.5, 18.5, 15.5, 25.5)


# ------------------------------------------------------- the equivalence run

def test_indexed_shield_decides_exactly_as_the_old_fence_loop():
    """Every policy in policies/, plus the 50-rule load, overlapping fences, a
    ticking clock, a mid-stream hot-apply and non-finite actions."""
    from bench_shield_50rules import fifty_rule_policy
    smap = _obstacle_map()
    t = Tally()

    for path in POLICIES:
        pol = load_policy(path)
        new, old = _shields(pol, smap)
        compare(new, old, _pairs(pol, 100, path.name), t, path.name,
                nonfinite_every=40)

    pol = _overlap_policy()
    new, old = _shields(pol, smap)
    compare(new, old, _pairs(pol, 400, "overlap"), t, "overlap", nonfinite_every=50,
            hot_apply_at=200, hot_fence=_square("hot", 22, -12, 10, margin=1.0))

    pol = fifty_rule_policy()
    new, old = _shields(pol, smap)
    compare(new, old, _pairs(pol, 60, "fifty"), t, "fifty")

    # The grant's horizon (5 s at 0.1 s). This unit does not switch the
    # flights to it, but the equivalence must not depend on the pose count.
    pol = _overlap_policy()
    new, old = _shields(pol, smap, lookahead_s=5.0, dt=0.1)
    compare(new, old, _pairs(pol, 120, "overlap-5s"), t, "overlap-5s")

    # A clock that advances 7 minutes per SAMPLE, from a Friday afternoon
    # across the end of corridor_survey's 07:30-17:30 school window. No sample
    # falls in the minute 17:30:00-17:30:59, where the old whole-minute window
    # and the Shield's to-the-second one disagree on purpose (test_shield.py).
    for name in ("corridor_survey.yaml",):
        pol = load_policy(ROOT / "policies" / name)
        start, step = datetime(2026, 10, 9, 16, 0), timedelta(minutes=7)
        clk = _TickClock(start, step)
        new, old = _shields(pol, smap, now_new=clk, now_old=clk)
        compare(new, old, _pairs(pol, 200, name + "@clock"), t, name + "@clock",
                tick_clock=clk)

    assert t.mismatched == 0, (
        f"{t.mismatched}/{t.compared} answers differ; first: {t.first_mismatch}")
    # Coverage: the run must have gone where the refactor changed things.
    assert t.compared >= 4000, t.compared
    assert t.geofence >= 500, f"only {t.geofence} samples raised a geofence violation"
    assert t.escape >= 50, f"only {t.escape} GeofenceEscape repairs exercised"
    assert t.slide >= 50, f"only {t.slide} GeofenceSlide repairs exercised"
    assert t.skipped_fences >= t.fence_slots // 4, (
        f"the index left out only {t.skipped_fences}/{t.fence_slots} fences - "
        f"it was barely exercised")
    # Crashes are compared like answers, but a run that mostly crashed would
    # compare error strings and little else.
    assert t.raised <= t.compared // 50, f"{t.raised} of {t.compared} calls raised"
    print(f"      compared {t.compared} answers, 0 differ ({t.raised} identical "
          f"crashes); geofence violations {t.geofence}, escapes {t.escape}, "
          f"slides {t.slide}, brakes {t.braked}; index skipped "
          f"{t.skipped_fences}/{t.fence_slots} fence tests")


def test_the_harness_catches_an_index_blind_to_the_margin():
    """Break the index on purpose - build it on the AUTHORED polygons, as if the
    margin ring did not exist - and the same harness must report differences.
    If it did not, its zero above would mean nothing."""

    class MarginBlind(Shield):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self._blind = PolicyIR([FenceRecord(r.index, r.rule, r.polygon, r.polygon,
                                                tuple(r.polygon.bounds))
                                    for r in self._ir.fences])

        def _fence_candidates(self, ir, state, poses):
            return Shield._fence_candidates(self._blind, state, poses)

    pol = load_policy(ROOT / "policies" / "demo_policy.yaml")
    new, old = _shields(pol, None, cls_new=MarginBlind)
    t = Tally()
    compare(new, old, _pairs(pol, 400, "blind"), t, "blind")
    assert t.mismatched > 0, "a margin-blind index went unnoticed"


def test_the_current_position_is_always_in_the_query_box():
    """The already-inside test reads the state, not a forecast pose. Today the
    forecast starts at t = 0 so the two coincide; make it start one step ahead
    (on both sides) and a box built from poses alone would drop the fence the
    vehicle is sitting in while it flies fast away from it."""

    def ahead(self, state, action, horizon_s=None):
        it = Shield._predict(self, state, action, horizon_s)
        next(it)                                     # drop the t = 0 sample
        yield from it

    class NewAhead(Shield):
        _predict = ahead

    class OldAhead(LegacyFenceShield):
        _predict = ahead

    # nfz-square is 7..23 m with a 1 m margin. 3 m inside its y = 7 edge,
    # bolting the OTHER way (through the far side) at 40 m/s: not escaping by
    # the trend test, and every forecast pose is already past the ring.
    pol = load_policy(ROOT / "policies" / "demo_policy.yaml")
    st, a = State(x=15.0, y=10.0, up=4.0), Action4D(vx=0.0, vy=40.0)
    # 3 s at 0.5 s: the first pose (dropped above) is then 0.5 s ahead, past the
    # zone's far side, which is what this test needs (the default is now the
    # grant's 5 s at 0.1 s, whose first pose is still inside).
    new = NewAhead(pol.model_copy(deep=True), lookahead_s=3.0, dt=0.5, escalation=False)
    old = OldAhead(pol.model_copy(deep=True), lookahead_s=3.0, dt=0.5, escalation=False)
    poses = new.forecast(st, a)
    assert poses[0][0] > 0 and all(p.y > 24.0 for _, p in poses)
    want = _vdump(old._check(st, a))
    assert any(v["detail"].startswith("currently INSIDE") for v in want), want
    assert _vdump(new._check(st, a)) == want
    assert _ddump(new.filter(st, a)) == _ddump(old.filter(st, a))


def test_a_slide_that_turns_into_a_second_fence_is_still_repaired():
    """The raw forecast reaches only fence A; the anti-stall slide off A points
    at fence B, which the raw forecast's box never touched. The old loop
    repaired against B; so must the indexed one - which is why the candidate
    set is re-queried whenever the velocity changes inside the repair loop."""
    pol = Policy(policy_id="two", constraints=[
        *_caps(),
        PolygonFence(id="A", type="polygon_fence",
                     vertices=[{"x": 6, "y": -5}, {"x": 20, "y": -5},
                               {"x": 20, "y": 5}, {"x": 6, "y": 5}], margin_m=1.0),
        PolygonFence(id="B", type="polygon_fence",
                     vertices=[{"x": -3, "y": -16}, {"x": 3, "y": -16},
                               {"x": 3, "y": -10}, {"x": -3, "y": -10}], margin_m=1.0),
    ])
    new, old = _shields(pol, None)
    st, a = State(x=0.0, y=0.0, up=5.0), Action4D(vx=4.0)
    near = new._fence_candidates(new.ir, st, new.forecast(st, a))
    assert [new.ir.fences[i].rule.id for i in sorted(near)] == ["A"], near
    dn, do = new.filter(st, a), old.filter(st, a)
    assert _ddump(dn) == _ddump(do)
    assert any(r.detail.startswith("B:") for r in dn.repairs), dn.repairs


# ------------------------------------------------- multi-scale geometry (WP2-16)

def test_every_zone_carries_a_coarse_ring_that_contains_it():
    """The IR's coarse scale must CONTAIN the authored polygon (a coarse
    keep-out zone that cut a corner would describe a smaller zone than the
    Shield enforces), have at most COARSE_MAX_VERTICES vertices, and be the
    one guardrail/compiler.py's multiscale_geometry computes for itself, so
    the compiler can read it from the IR instead."""
    from shapely.geometry import Polygon as _P
    from guardrail.compiler import multiscale_geometry
    from guardrail.ir import COARSE_MAX_VERTICES
    n = 0
    for path in POLICIES:
        pol = load_policy(path)
        ir = PolicyIR.from_policy(pol)
        comp = multiscale_geometry(pol)
        for rec in ir.fences:
            geo = ir.multiscale[rec.rule.id]
            fine, coarse = _P(geo["fine"]), _P(geo["coarse"])
            assert len(geo["coarse"]) <= COARSE_MAX_VERTICES, (path.name, rec.rule.id)
            assert coarse.buffer(1e-6).contains(fine), (path.name, rec.rule.id)
            theirs = _P(comp[rec.rule.id]["coarse"])
            assert coarse.symmetric_difference(theirs).area <= 1e-6 * max(fine.area, 1.0), \
                (path.name, rec.rule.id, geo["coarse"], comp[rec.rule.id]["coarse"])
            minx, miny, maxx, maxy = fine.bounds
            assert geo["bbox"] == [(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)]
            n += 1
        for c in pol.constraints:
            if c.type == "corridor":
                assert ir.multiscale[c.id] == {"fine": [(float(x), float(y))
                                                        for x, y in c.points()]}
    assert n >= 40, n


def test_a_geometry_ref_names_one_scale_of_one_generation():
    pol = load_policy(ROOT / "policies" / "demo_policy.yaml")
    ir = PolicyIR.from_policy(pol)
    ref = ir.geometry_ref("coarse", "nfz-square")
    assert ref == f"coarse:nfz-square@v{pol.version}/g{pol.generation}"
    assert ir.resolve_geometry_ref(ref) == ir.multiscale["nfz-square"]["coarse"]
    for bad in ("coarse:nfz-square@v9.9.9/g0", "coarse:nfz-square@v0.1.0/g7",
                "nope", "medium:nfz-square@v0.1.0/g0"):
        try:
            ir.resolve_geometry_ref(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"{bad!r} resolved")
    sh = Shield(pol)
    sh.filter(State(x=-20.0, y=-20.0, up=4.0), Action4D())
    sh.hot_apply(as_dynamic_nfz(_square("hot", 100, 100, 5)))
    try:
        sh.ir.resolve_geometry_ref(ref)       # minted for generation 0
    except ValueError:
        pass
    else:
        raise AssertionError("a generation-0 ref resolved against generation 1")
    assert sh.ir.resolve_geometry_ref(sh.ir.geometry_ref("fine", "hot"))[0] == (100.0, 100.0)


def test_a_circle_fence_compiles_to_a_ring_that_holds_the_whole_disc():
    from guardrail.models import CircleFence
    c = CircleFence.model_validate({"id": "disc", "type": "circle_fence",
                                    "center": {"x": 10.0, "y": -5.0}, "radius_m": 20.0,
                                    "margin_m": 0.0})
    ir = PolicyIR.from_policy(Policy(policy_id="c", constraints=[c]))
    rec = ir.fences[0]
    from shapely.geometry import Point
    for k in range(720):
        ang = 2 * math.pi * k / 720
        p = Point(10.0 + 19.999 * math.cos(ang), -5.0 + 19.999 * math.sin(ang))
        assert rec.buffered.contains(p), k
    sh = Shield(Policy(policy_id="c", constraints=[c]))
    assert sh.state_is_unsafe(State(x=10.0 + 19.9, y=-5.0, up=10.0))
    assert not sh.state_is_unsafe(State(x=10.0 + 20.2, y=-5.0, up=10.0))


def test_a_moving_zone_is_left_out_of_the_tree_and_always_returned():
    """The tree holds where a zone was compiled; a moving zone will be
    elsewhere, so a box query must never drop it."""
    from guardrail.models import DynamicNFZ
    z = DynamicNFZ.model_validate({"id": "mv", "type": "dynamic_nfz",
                                   "vertices": [{"x": 0, "y": 0}, {"x": 4, "y": 0},
                                                {"x": 4, "y": 4}],
                                   "motion": {"vx_mps": 1.0}})
    ir = PolicyIR.from_policy(Policy(policy_id="m", constraints=[_square("s", 50, 50, 5), z]))
    assert [r.rule.id for r in ir.fences if r.moving] == ["mv"]
    assert [r.rule.id for r in ir.fences_in_box(-500, -500, -499, -499)] == ["mv"]
    assert [r.rule.id for r in ir.fences_in_box(51, 51, 52, 52)] == ["s", "mv"]


def test_an_event_recompiles_only_the_zone_it_changed():
    """The grant's hot path re-derives "the affected spatial-index entries":
    after a spawn every other zone keeps its compiled record (no re-buffer)."""
    pol = _overlap_policy()
    sh = Shield(pol)
    before = {r.rule.id: r.buffered for r in sh.ir.fences}
    sh.hot_apply(as_dynamic_nfz(_square("hot", 100, 100, 5)))
    after = {r.rule.id: r.buffered for r in sh.ir.fences}
    assert set(after) == set(before) | {"hot"}
    assert all(after[k] is before[k] for k in before)


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
