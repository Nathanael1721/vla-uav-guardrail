"""
Shield test scenarios — the "golden cases" idea from the grant, in miniature.

Run either way:
    pytest tests/ -v                      (nice output)
    python tests/test_shield.py           (no pytest needed)

Every test states its story in one line. The key invariant everywhere:
whatever the Shield emits must itself pass a re-check (P0 escape = 0).
"""
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
    s = make_shield()
    d = s.filter(State(x=-20, y=-20, up=4), Action4D(yaw_rate=90))
    assert d.touched
    assert abs(d.emitted.yaw_rate) <= 45.0 + 1e-9


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
    """The hard KPI, on the new rule: seen and then flown anyway is an escape."""
    sh = _standoff_shield(min_range_m=10.0)
    sh.set_subject(30.0, 0.0, "pedestrian")
    for d in (15.0, 11.0, 9.0, 6.0):
        dec = sh.filter(State(x=30.0 - d, y=0.0, up=9.0), Action4D(vx=4.0))
        if dec.violations:
            assert dec.repairs or dec.braked, f"P0 escape at {d} m"


def test_a_lost_subject_must_clear_the_stale_position():
    """A stale position has the Shield enforcing a stand-off from where the
    subject used to be - wrong, and unfalsifiable from the logs."""
    sh = _standoff_shield(min_range_m=10.0)
    sh.set_subject(30.0, 0.0, "pedestrian")
    assert sh.filter(State(x=24.0, y=0.0, up=9.0), Action4D(vx=4.0)).touched
    sh.set_subject(None)
    assert not sh.filter(State(x=24.0, y=0.0, up=9.0), Action4D(vx=4.0)).touched


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
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
