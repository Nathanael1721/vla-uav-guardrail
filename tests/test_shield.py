"""
Shield test scenarios — the "golden cases" idea from the grant, in miniature.

Run either way:
    pytest tests/ -v                      (nice output)
    python tests/test_shield.py           (no pytest needed)

Every test states its story in one line. The key invariant everywhere:
whatever the Shield emits must itself pass a re-check (P0 escape = 0).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from guardrail import Action4D, Shield, State, load_policy

POLICY_PATH = Path(__file__).resolve().parents[1] / "policies" / "demo_policy.yaml"


def make_shield() -> Shield:
    # Fresh policy per test: hot_apply mutates the policy object, so sharing
    # one module-level instance would leak state between tests.
    return Shield(load_policy(POLICY_PATH), lookahead_s=3.0, dt=0.5)


# --------------------------------------------------------------- legal cases

def test_legal_cruise_untouched():
    """Far from NFZ, legal speed/alt -> action must pass through UNCHANGED."""
    s = make_shield()
    d = s.filter(State(x=-20, y=-20, up=4), Action4D(vx=2, vy=0, vz_up=0))
    assert not d.touched and not d.braked
    assert d.emitted == d.raw


def test_zero_action_untouched():
    """Hovering is always legal (inside the alt band, outside NFZ)."""
    s = make_shield()
    d = s.filter(State(x=-20, y=-20, up=4), Action4D())
    assert not d.touched
    assert d.emitted == d.raw


def test_tangent_near_miss_not_overblocked():
    """Flying PAST the NFZ (not into it) must not be blocked — no over-blocking."""
    s = make_shield()
    # NFZ+margin spans y in [6, 24]; fly north along y=30: clear the whole window.
    d = s.filter(State(x=0, y=30, up=4), Action4D(vx=3, vy=0, vz_up=0))
    assert not d.touched
    assert d.emitted == d.raw


# --------------------------------------------------------------- kinematic

def test_overspeed_clamped():
    s = make_shield()
    d = s.filter(State(x=-20, y=-20, up=4), Action4D(vx=8, vy=0, vz_up=0))
    h = (d.emitted.vx ** 2 + d.emitted.vy ** 2) ** 0.5
    assert d.touched and not d.braked
    assert abs(h - 4.0) < 1e-6                    # clamped exactly to cap
    assert d.emitted.vy == 0 and d.emitted.vx > 0  # direction preserved


def test_yaw_rate_clamped():
    """The cap is in DEGREES per second; the action carries RADIANS per second.

    This assertion used to read `abs(d.emitted.yaw_rate) <= 45.0`, comparing a
    rad/s quantity against a degree bound. The Shield emits 0.785 rad/s here,
    so the test passed - and would have gone on passing for anything up to
    45 rad/s, which is 2578 deg/s. It was the only test named for the yaw clamp
    and it was blind to exactly the 57.3x units defect this project fixed on
    2026-08-17 (see tests/test_guardrail_coverage.py).

    So: convert, then compare, and pin the value rather than just bounding it.
    """
    s = make_shield()
    d = s.filter(State(x=-20, y=-20, up=4), Action4D(yaw_rate=90))
    assert d.touched
    dps = math.degrees(d.emitted.yaw_rate)
    assert abs(dps) <= 45.0 + 1e-9, f"emitted {dps} deg/s, cap is 45"
    assert abs(dps - 45.0) < 1e-6, f"should clamp exactly to the cap, got {dps}"


# --------------------------------------------------------------- altitude

def test_climb_above_cap_clamped():
    """At 5.5 m climbing 2 m/s -> would blow through 6 m cap. vz must shrink."""
    s = make_shield()
    d = s.filter(State(x=-20, y=-20, up=5.5), Action4D(vz_up=2))
    assert d.touched and not d.braked
    end_alt = 5.5 + d.emitted.vz_up * 3.0
    assert end_alt <= 6.0 + 1e-6


def test_descend_below_floor_clamped():
    s = make_shield()
    d = s.filter(State(x=-20, y=-20, up=2.5), Action4D(vz_up=-2))
    assert d.touched and not d.braked
    end_alt = 2.5 + d.emitted.vz_up * 3.0
    assert end_alt >= 2.0 - 1e-6


# --------------------------------------------------------------- geofence

def test_heading_into_nfz_repaired_safe():
    """Aimed straight at the NFZ from the west -> slide repair; emitted action
    must predict NO entry (the core promise)."""
    s = make_shield()
    st = State(x=15, y=0, up=4)                    # west of zone, aimed east
    d = s.filter(st, Action4D(vx=0, vy=3, vz_up=0))
    assert d.touched
    assert not s._check(st, d.emitted)             # re-check clean = no escape


def test_diagonal_into_nfz_repaired_safe():
    s = make_shield()
    st = State(x=2, y=2, up=4)                     # SW corner, aimed NE at zone
    d = s.filter(st, Action4D(vx=2.5, vy=2.5, vz_up=0))
    assert d.touched
    assert not s._check(st, d.emitted)


def test_already_inside_nfz_escapes():
    """Spawned illegally inside the zone: freezing would lock the violation in
    place forever. Shield must emit a RECOVERY action that flies OUT."""
    s = make_shield()
    d = s.filter(State(x=15, y=15, up=4), Action4D(vx=1, vy=0, vz_up=0))
    assert d.touched and not d.braked
    h = (d.emitted.vx ** 2 + d.emitted.vy ** 2) ** 0.5
    assert h > 0.5                                  # actually moving...
    assert not s._check(State(x=15, y=15, up=4), d.emitted)   # ...and legally


def test_grounded_below_floor_recovers():
    """The bug that deadlocked lesson 05: sitting at 0.2 m (below the 2 m
    floor). Old design braked forever. New design must CLIMB."""
    s = make_shield()
    st = State(x=-20, y=-20, up=0.2)
    d = s.filter(st, Action4D())                    # hovering, not climbing
    assert d.touched and not d.braked
    assert d.emitted.vz_up > 0                      # recovery = climb
    assert not s._check(st, d.emitted)


def test_hover_above_ceiling_recovers():
    s = make_shield()
    st = State(x=-20, y=-20, up=8.0)
    d = s.filter(st, Action4D())
    assert d.touched and not d.braked
    assert d.emitted.vz_up < 0                      # recovery = descend
    assert not s._check(st, d.emitted)


def test_head_on_approach_does_not_stall():
    """Head-on at the zone edge: naive slide cancels ALL velocity -> drone
    stalls forever. Anti-stall bias must keep it moving along the edge."""
    s = make_shield()
    st = State(x=15, y=2, up=4)                     # west of zone, aimed dead east
    d = s.filter(st, Action4D(vx=0, vy=3, vz_up=0))
    assert d.touched and not d.braked
    h = (d.emitted.vx ** 2 + d.emitted.vy ** 2) ** 0.5
    assert h > 0.5                                  # still moving
    assert not s._check(st, d.emitted)


# --------------------------------------------------------------- combined

def test_overspeed_and_climb_both_fixed():
    s = make_shield()
    d = s.filter(State(x=-20, y=-20, up=5.5), Action4D(vx=9, vy=0, vz_up=3))
    h = (d.emitted.vx ** 2 + d.emitted.vy ** 2) ** 0.5
    assert d.touched and not d.braked
    assert h <= 4.0 + 1e-6
    assert 5.5 + d.emitted.vz_up * 3.0 <= 6.0 + 1e-6
    ops = {r.operator for r in d.repairs}
    assert "SpeedClamp" in ops and ("ClimbClamp" in ops or "AltitudeClamp" in ops)


def test_lesson4_style_nfz_dead_center_climb():
    """The lesson-04 story re-told through the real Shield: aimed at NFZ center,
    climbing too fast. Everything must come out safe."""
    s = make_shield()
    st = State(x=0, y=15, up=4)                    # south of zone, aimed north at center
    d = s.filter(st, Action4D(vx=4, vy=0, vz_up=3))
    assert d.touched
    assert not s._check(st, d.emitted)             # zero escape, always


# --------------------------------------------------------------- hot-apply

def test_hot_apply_dynamic_nfz():
    """Grant's dynamic_nfz: a zone injected mid-flight must be enforced on the
    very next tick, and the policy generation/hash must change (audit trail)."""
    from guardrail.models import PolygonFence, XY

    s = make_shield()
    st = State(x=-20, y=-20, up=4)
    act = Action4D(vx=3, vy=0, vz_up=0)          # north, legal before hot-apply
    assert not s.filter(st, act).touched

    gen0, hash0 = s.policy.generation, s.policy.policy_hash
    s.hot_apply(PolygonFence(
        id="nfz-dynamic", type="polygon_fence",
        vertices=[XY(x=-15, y=-24), XY(x=-8, y=-24), XY(x=-8, y=-16), XY(x=-15, y=-16)],
    ))
    assert s.policy.generation == gen0 + 1
    assert s.policy.policy_hash != hash0          # hash tracks the change

    d = s.filter(st, act)                         # same action now heads into it
    assert d.touched                              # enforced immediately
    assert not s._check(st, d.emitted)            # and repaired safely


# --------------------------------------------------------------- runner

# ------------------------------------------------- subject standoff (WP1)

def _standoff_shield(min_range_m=10.0, subject_class="pedestrian"):
    """A policy holding a stand-off from the tracked subject.

    Asked for at the 2026-08-19 review: "hold 10 m from a person, and different
    policies for different objects". Before this the stand-off was --want-range,
    a controller flag - not hashed into policy_hash, not audited, not enforced.
    """
    from guardrail.models import KinematicEnvelope, Policy, SubjectStandoff
    pol = Policy(policy_id="standoff", version="0.1.0", constraints=[
        SubjectStandoff(id="ped-standoff", type="subject_standoff",
                        subject_class=subject_class, min_range_m=min_range_m),
        KinematicEnvelope(id="kin", type="kinematic_envelope", priority="P1",
                          speed_max_mps=5.0, climb_rate_max_mps=2.0,
                          yaw_rate_max_dps=45.0),
    ])
    return Shield(pol, lookahead_s=3.0, dt=0.5)


def _radial(dec, subject=(30.0, 0.0), pos=None):
    """Emitted speed along the line to the subject. Negative = opening."""
    import math
    sx, sy = subject
    px, py = pos
    d = math.hypot(sx - px, sy - py)
    ux, uy = (sx - px) / d, (sy - py) / d
    return dec.emitted.vx * ux + dec.emitted.vy * uy


def test_standoff_is_inert_with_no_subject():
    """A stand-off rule with nothing to stand off from has no opinion. Inventing
    one would be worse than silence."""
    sh = _standoff_shield()
    dec = sh.filter(State(x=29.0, y=0.0, up=9.0), Action4D(vx=4.0))
    assert not dec.touched and dec.emitted.vx == 4.0


def test_standoff_binds_only_the_named_class():
    """This is what makes 'different policies for different objects' real."""
    sh = _standoff_shield(subject_class="pedestrian")
    sh.set_subject(30.0, 0.0, "car")
    dec = sh.filter(State(x=24.0, y=0.0, up=9.0), Action4D(vx=4.0))
    assert not dec.touched, "a pedestrian rule must not bind a car"


def test_a_predicted_breach_is_stopped_before_it_happens():
    """At 4 m/s over a 3 s horizon the aircraft commits 12 m ahead of itself, so
    the rule has to fire well outside the ring."""
    sh = _standoff_shield(min_range_m=10.0)
    sh.set_subject(30.0, 0.0, "pedestrian")
    pos = (30.0 - 15.0, 0.0)
    dec = sh.filter(State(x=pos[0], y=pos[1], up=9.0), Action4D(vx=4.0))
    assert dec.touched
    assert _radial(dec, pos=pos) <= 1e-6, "must not still be closing"


def test_inside_the_ring_it_opens_rather_than_freezing():
    """Removing the closing component leaves the range where it is, which the
    check reads as 'not opening' - so the violation would persist every tick and
    the aircraft would be frozen at a distance the policy forbids."""
    sh = _standoff_shield(min_range_m=10.0)
    sh.set_subject(30.0, 0.0, "pedestrian")
    pos = (30.0 - 6.0, 0.0)
    dec = sh.filter(State(x=pos[0], y=pos[1], up=9.0), Action4D(vx=4.0))
    assert _radial(dec, pos=pos) < -1e-6, "must open the range, not hold or close"


def test_it_never_accelerates_toward_the_subject():
    """The failure this replaced: with the repair's trigger not matching the
    check's, nothing repaired the violation and the rescue search emitted
    +5 m/s straight at the subject."""
    sh = _standoff_shield(min_range_m=10.0)
    sh.set_subject(30.0, 0.0, "pedestrian")
    for d in (11.0, 9.0, 6.0, 3.0, 1.0):
        pos = (30.0 - d, 0.0)
        dec = sh.filter(State(x=pos[0], y=pos[1], up=9.0), Action4D(vx=4.0))
        r = _radial(dec, pos=pos)
        assert r <= 1e-6, f"at {d} m the Shield emitted {r:+.2f} m/s TOWARD the subject"


def test_orbiting_at_the_ring_is_allowed():
    """The rule objects to one direction of travel, not to the mission. Killing
    the whole velocity would stop the aircraft to satisfy it."""
    sh = _standoff_shield(min_range_m=10.0)
    sh.set_subject(30.0, 0.0, "pedestrian")
    dec = sh.filter(State(x=22.0, y=0.0, up=9.0), Action4D(vy=3.0))
    assert abs(dec.emitted.vy - 3.0) < 1e-6, "tangential motion must survive"


def test_standoff_violations_are_repaired_not_escaped():
    """The hard KPI, on the new rule: seen and then flown anyway is an escape.

    `dec.repairs or dec.braked` asserts only that something was ATTEMPTED, so a
    repair operator that runs and leaves the violation in place still passed.
    That is not what an escape rate of zero means. Every other test in this file
    uses the strong form - re-check the EMITTED action - so this one now does
    too.
    """
    sh = _standoff_shield(min_range_m=10.0)
    sh.set_subject(30.0, 0.0, "pedestrian")
    for d in (15.0, 11.0, 9.0, 6.0):
        st = State(x=30.0 - d, y=0.0, up=9.0)
        dec = sh.filter(st, Action4D(vx=4.0))
        if dec.violations:
            assert dec.repairs or dec.braked, f"P0 escape at {d} m: nothing attempted"
            assert not sh._check(st, dec.emitted), (
                f"at {d} m the repair ran but the emitted action still violates")


def test_a_lost_subject_must_clear_the_stale_position():
    """A stale position has the Shield enforcing a stand-off from where the
    subject used to be - wrong, and unfalsifiable from the logs."""
    sh = _standoff_shield(min_range_m=10.0)
    sh.set_subject(30.0, 0.0, "pedestrian")
    assert sh.filter(State(x=24.0, y=0.0, up=9.0), Action4D(vx=4.0)).touched
    sh.set_subject(None)
    assert not sh.filter(State(x=24.0, y=0.0, up=9.0), Action4D(vx=4.0)).touched





# ---------------------------------------------------------------------------
# A repair must FLOOR an escape, never cap it.
#
# Fixed 2026-08-26. The monitors are trend-aware; three repair operators judged
# position only. So once any unrelated violation dragged an action into the
# repair loop, a position-only repair clobbered an action that was already
# correctly escaping:
#
#   StandoffRecover   3.00 m/s -> 3.000   |   3.01 m/s -> 0.500   (6x slower)
#   GeofenceEscape    4.00 m/s -> 4.000   |   4.01 m/s -> 2.000   (2x slower)
#   ClearanceFix      5.00 m/s -> 5.000   |   5.01 m/s -> 3.573 + sideways drift
#
# Why every test below adds an unrelated yaw breach: the existing coverage
# (tests/test_clearance.py::test_escaping_the_ring_is_still_not_flagged) only
# ever exercised escape with NOTHING else wrong, which takes filter()'s
# untouched-passthrough branch and never enters the repair loop at all. That is
# the precise gap that let this survive. yaw_rate is P1 and does not touch
# horizontal motion, so it opens the loop without disturbing what is measured.
#
# The fix is a FLOOR and not a decline, which matters in the other direction:
# the monitor forgives any opening above 0.1 m/s, so a repair that simply stood
# aside would leave the aircraft crawling out of a zone it must not be in.
# ---------------------------------------------------------------------------

YAW_BREACH = math.radians(60)        # P1, horizontal-motion-neutral


def _standoff_ring_shield(min_range_m=10.0):
    from guardrail import load_policy as _lp
    sh = Shield(_lp(Path(__file__).resolve().parents[1] / "policies"
                    / "follow_pedestrian.yaml"), lookahead_s=3.0, dt=0.5)
    sh.set_subject(0.0, 0.0, "pedestrian")
    return sh


def test_standoff_recovery_is_never_slowed_by_an_unrelated_violation():
    sh = _standoff_ring_shield()
    st = State(x=9.0, y=0.0, up=9.0)          # inside a 10 m ring, flying out
    clean = sh.filter(st, Action4D(vx=3.0)).emitted.vx
    over = sh.filter(st, Action4D(vx=3.01)).emitted.vx
    assert clean > 2.9, clean
    assert over >= clean - 1e-6, (
        f"a 0.3 % overspeed cut the escape from {clean:.3f} to {over:.3f} m/s")


def test_standoff_recovery_raises_a_crawl_rather_than_accepting_it():
    """The monitor forgives anything over 0.1 m/s. The repair must not."""
    sh = _standoff_ring_shield()
    d = sh.filter(State(x=9.0, y=0.0, up=9.0),
                  Action4D(vx=0.15, yaw_rate=YAW_BREACH))
    assert d.emitted.vx >= 0.5 - 1e-6, (
        f"left the aircraft crawling out of a standoff ring at {d.emitted.vx:.3f} m/s")


def test_geofence_escape_is_never_slowed_by_an_unrelated_violation():
    s = make_shield()
    st = State(x=22.0, y=15.0, up=4.0)        # inside the zone, 1 m from the edge
    clean = s.filter(st, Action4D(vx=4.0)).emitted.vx
    over = s.filter(st, Action4D(vx=4.01)).emitted.vx
    assert clean > 3.9, clean
    assert over >= clean - 1e-6, (
        f"a 0.25 % overspeed cut the NFZ escape from {clean:.3f} to {over:.3f} m/s")


def test_geofence_escape_raises_a_crawl_to_the_recovery_speed():
    """2.0 m/s is the reference implementation's escape_speed_mps. It is a
    FLOOR, not a ceiling, and not a reason to decline."""
    s = make_shield()
    d = s.filter(State(x=22.0, y=15.0, up=4.0),
                 Action4D(vx=0.15, yaw_rate=YAW_BREACH))
    assert d.emitted.vx >= 2.0 - 1e-6, (
        f"left the aircraft crawling out of a no-fly zone at {d.emitted.vx:.3f} m/s")


def test_a_command_into_the_zone_is_still_turned_around():
    """The floor must not have turned the recovery into a no-op."""
    s = make_shield()
    d = s.filter(State(x=22.0, y=15.0, up=4.0), Action4D(vx=-4.0))
    assert d.emitted.vx >= 2.0 - 1e-6, d.emitted
    assert not s._check(State(x=22.0, y=15.0, up=4.0), d.emitted)


def test_the_escape_survives_the_speed_cap_and_the_mission_pays():
    """When the cap binds, it must eat the TANGENTIAL component.

    Scaling the whole vector shrinks the very component getting the aircraft
    out: measured, a 3.00 m/s standoff recovery came out at 2.12 while 2.12 m/s
    of mission motion was preserved untouched.
    """
    sh = _standoff_ring_shield()
    d = sh.filter(State(x=9.0, y=0.0, up=9.0),
                  Action4D(vx=0.0, vy=5.0, yaw_rate=YAW_BREACH))
    assert d.emitted.vx >= 0.5 - 1e-6, f"escape was sacrificed: {d.emitted}"
    assert abs(d.emitted.vy) < 5.0, "the tangential component should have paid"


def test_escape_speed_is_monotonic_in_the_commanded_speed():
    """Faster in must never mean slower out. This is the property the whole
    defect violated, and it holds only while the repair loop is engaged."""
    for label, sh, st, axis in (
            ("standoff", _standoff_ring_shield(), State(x=9.0, y=0.0, up=9.0), "vx"),
            ("geofence", make_shield(), State(x=22.0, y=15.0, up=4.0), "vx")):
        prev = -9.9
        for i in range(0, 61):
            cmd = i * 0.1
            got = getattr(sh.filter(st, Action4D(vx=cmd, yaw_rate=YAW_BREACH)).emitted,
                          axis)
            assert got >= prev - 1e-9, (
                f"{label}: commanding {cmd:.1f} m/s emitted {got:.3f}, "
                f"less than the {prev:.3f} emitted by a slower command")
            prev = got


# ------------------------------- 50-rule speed and the sliding window (WP3-23)
#
# The grant's monitor budget is <= 5 ms per query with 50 active rules inside a
# 100 ms tick (Policy DSL page, acceptance KPIs; Safety Shield page, "Violation
# checking"). Measured before guardrail/ir.py on the 50-rule bench policy:
# `_check` 14-33 ms in clear sky and ~33 ms among the fences, `filter()` median
# ~170 ms near a fence (experiments/bench_shield_50rules.py). The first two
# tests pin the two CAUSES structurally, so they cannot pass by luck on a fast
# machine; the third pins the budget itself. Every test from here down fails
# on the pre-IR shield.py (63351 rings rebuilt in 40 ticks, 19200 fence tests
# in 50 clear-sky checks, median _check 34 ms, and no `history`/`forecast`).

def _fifty():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))
    import bench_shield_50rules as B
    return B


def _count_calls(owner, name):
    """Wrap owner.name with a call counter; returns (counter, restore)."""
    orig = getattr(owner, name)
    box = {"n": 0}

    def wrapped(*a, **kw):
        box["n"] += 1
        return orig(*a, **kw)
    setattr(owner, name, wrapped)
    return box, lambda: setattr(owner, name, orig)


def test_a_50_rule_tick_never_rebuilds_a_margin_ring():
    """geometry.point_in_fence used to call poly.buffer(margin) on every point
    test - 30-45 us each, up to 384 per check. The ring is now built once per
    fence, at compile time, so a whole filter() near the fences builds none."""
    from shapely.geometry.base import BaseGeometry
    B = _fifty()
    sh = Shield(B.fifty_rule_policy(), lookahead_s=3.0, dt=0.5)
    pairs = B.samples("near", 40, seed=1)
    box, restore = _count_calls(BaseGeometry, "buffer")
    try:
        for st, a in pairs:
            sh.filter(st, a)
    finally:
        restore()
    assert box["n"] == 0, f"{box['n']} margin rings rebuilt during 40 ticks"


def test_fences_out_of_the_forecasts_reach_are_not_tested():
    """In clear sky no fence can be reached, so no point-in-fence test should
    run at all. The old loop ran 48 x (1 + 7) of them per check regardless."""
    import guardrail.shield as S
    B = _fifty()
    sh = Shield(B.fifty_rule_policy(), lookahead_s=3.0, dt=0.5)
    box, restore = _count_calls(S, "point_in_fence")
    try:
        for st, a in B.samples("clear", 50, seed=2):
            sh._check(st, a)
        clear = box["n"]
        box["n"] = 0
        for st, a in B.samples("near", 50, seed=2):
            sh._check(st, a)
        near = box["n"]
    finally:
        restore()
    assert clear == 0, f"{clear} fence tests in clear sky"
    # ...and the index must not have switched the fences OFF: near them, the
    # tests still run (the equivalence with the old loop is tests/test_ir.py).
    assert near > 0, "no fence was tested beside a fence"


def test_the_50_rule_monitor_fits_the_grant_budget():
    """Median _check <= 5 ms and p99 filter() <= 100 ms at the grant's load, on
    the samples where both used to fail worst (1-8 m outside a fence's margin,
    flying at it). Desktop numbers; the margin is ~40x, so a loaded machine
    still passes and a return of the per-fence loop does not."""
    import statistics
    import time
    B = _fifty()
    sh = Shield(B.fifty_rule_policy(), lookahead_s=3.0, dt=0.5)
    pairs = B.samples("near", 200, seed=3)
    ms_check, ms_filter = [], []
    for st, a in pairs:
        t0 = time.perf_counter()
        sh._check(st, a)
        ms_check.append((time.perf_counter() - t0) * 1e3)
    for st, a in pairs[:100]:
        t0 = time.perf_counter()
        sh.filter(st, a)
        ms_filter.append((time.perf_counter() - t0) * 1e3)
    med = statistics.median(ms_check)
    p99 = sorted(ms_filter)[98]
    assert med <= 5.0, f"_check median {med:.2f} ms > 5 ms at 50 rules"
    assert p99 <= 100.0, f"filter() p99 {p99:.1f} ms > the 100 ms tick at 50 rules"


def test_history_keeps_the_last_50_ticks_oldest_first():
    """The grant's node design: 'Sliding-window buffer - last 50 actions'."""
    from guardrail.shield import HISTORY_LEN
    assert HISTORY_LEN == 50
    s = make_shield()
    assert s.history == ()
    for i in range(60):
        s.filter(State(x=-40.0 + i * 0.1, y=-20, up=4), Action4D(vx=2.0))
    h = s.history
    assert len(h) == 50
    assert abs(h[0].state.x - (-40.0 + 10 * 0.1)) < 1e-9      # ticks 10..59 kept
    assert abs(h[-1].state.x - (-40.0 + 59 * 0.1)) < 1e-9


def test_history_records_the_brake_and_rescue_branches_too():
    """filter() has four return paths. A window filled only by the passthrough
    would go silent exactly on the ticks worth looking back at."""
    from guardrail.models import KinematicEnvelope, Policy, PolygonFence
    caps = KinematicEnvelope(id="kin", type="kinematic_envelope", speed_max_mps=4.0,
                             climb_rate_max_mps=2.0, yaw_rate_max_dps=60.0)
    two = Policy(policy_id="two", constraints=[
        caps,
        PolygonFence(id="A", type="polygon_fence", margin_m=1.0, vertices=[
            {"x": 6, "y": -5}, {"x": 20, "y": -5}, {"x": 20, "y": 5}, {"x": 6, "y": 5}]),
        PolygonFence(id="B", type="polygon_fence", margin_m=1.0, vertices=[
            {"x": -3, "y": -16}, {"x": 3, "y": -16}, {"x": 3, "y": -10}, {"x": -3, "y": -10}]),
    ])
    s = Shield(two)
    d = s.filter(State(x=0, y=0, up=5), Action4D(vx=4.0))     # slide A -> B -> brake
    assert d.braked and s.history[-1].decision is d
    overlap = Policy(policy_id="ov", constraints=[
        caps,
        PolygonFence(id="a", type="polygon_fence", margin_m=2.0, vertices=[
            {"x": 0, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 20}, {"x": 0, "y": 20}]),
        PolygonFence(id="b", type="polygon_fence", margin_m=1.0, vertices=[
            {"x": 15, "y": 5}, {"x": 35, "y": 5}, {"x": 35, "y": 25}, {"x": 15, "y": 25}]),
    ])
    s = Shield(overlap)
    d = s.filter(State(x=16, y=10, up=10), Action4D())         # inside both
    assert any(r.operator == "ClearanceEscape" for r in d.repairs), d.repairs
    assert s.history[-1].decision is d and len(s.history) == 1


def test_history_is_a_record_not_an_alias():
    """The window keeps a COPY of the state: a caller reusing one State object
    for the next tick must not rewrite what the window says happened."""
    s = make_shield()
    st = State(x=-20, y=-20, up=4)
    d = s.filter(st, Action4D(vx=1.0))
    st.x = 999.0
    assert s.history[-1].state.x == -20 and s.history[-1].decision is d


def test_forecast_is_the_trajectory_the_rules_judge():
    s = make_shield()
    poses = s.forecast(State(x=0, y=0, up=4), Action4D(vx=2.0))
    assert [t for t, _ in poses] == [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    assert poses[-1][1].x == 6.0


def test_every_tick_in_the_window_carries_its_own_time():
    """WP3-13: no flight rail logged the Shield's share of the 100 ms tick.
    Each window entry now carries it. Zero is refused - it would read the same
    as "never timed" - and each figure must fit inside a stopwatch held around
    the same call, so it is that call's time and not a stale or shared one."""
    import time
    s = make_shield()
    cases = [(State(x=-20, y=-20, up=4), Action4D(vx=2.0)),          # passthrough
             (State(x=-20, y=-20, up=4), Action4D(vx=9.0)),          # speed clamp
             (State(x=4.0, y=15, up=4), Action4D(vx=3.0)),           # fence slide
             (State(x=15.0, y=15.0, up=4), Action4D())]              # inside NFZ
    for st, a in cases:
        t0 = time.perf_counter()
        s.filter(st, a)
        outer = (time.perf_counter() - t0) * 1e3
        ms = s.history[-1].elapsed_ms
        assert math.isfinite(ms) and 0.0 < ms <= outer, (ms, outer)
    assert len(s.history) == len(cases)


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
            # Any crash is a fail, not the end of the run. Without this arm
            # an ImportError or a numpy error in one test propagates out of
            # the loop, every remaining test is silently skipped, and the
            # summary line never prints - so the file looks like it ran
            # clean when most of it never executed. pytest runs them all, so
            # the two invocation modes disagreed about coverage.
            failed += 1
            print(f"ERROR {fn.__name__}  {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
