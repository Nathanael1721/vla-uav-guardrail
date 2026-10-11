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
    from guardrail.models import DynamicNFZ, XY

    s = make_shield()
    st = State(x=-20, y=-20, up=4)
    act = Action4D(vx=3, vy=0, vz_up=0)          # north, legal before hot-apply
    assert not s.filter(st, act).touched

    gen0, hash0 = s.policy.generation, s.policy.policy_hash
    s.hot_apply(DynamicNFZ(
        id="nfz-dynamic", type="dynamic_nfz",
        vertices=[XY(x=-15, y=-24), XY(x=-8, y=-24), XY(x=-8, y=-16), XY(x=-15, y=-16)],
    ))
    assert s.policy.generation == gen0 + 1
    assert s.policy.policy_hash != hash0          # hash tracks the change

    d = s.filter(st, act)                         # same action now heads into it
    assert d.touched                              # enforced immediately
    assert not s._check(st, d.emitted)            # and repaired safely
    assert d.generation == gen0 + 1 and d.policy_hash == s.policy.policy_hash


def test_a_polygon_fence_is_locked_once_the_mission_starts():
    """Policy DSL page: only dynamic_nfz, time_window_switch and corridor_swap
    may be hot-applied; "all other classes are locked at mission start".
    hot_apply took a PolygonFence mid-flight until 2026-10-07. Before the first
    tick a zone is still part of the mission-start policy; after it, the call
    is refused with the migration, and as_dynamic_nfz is that migration."""
    from guardrail.models import PolygonFence, XY
    from guardrail.shield import LockedRuleClass, as_dynamic_nfz

    def fence():
        return PolygonFence(id="nfz-late", type="polygon_fence",
                            vertices=[XY(x=-15, y=-24), XY(x=-8, y=-24),
                                      XY(x=-8, y=-16), XY(x=-15, y=-16)])
    s = make_shield()
    s.hot_apply(fence())                          # before take-off: allowed
    assert s.policy.generation == 1 and not s.mission_started
    s = make_shield()
    s.filter(State(x=-20, y=-20, up=4), Action4D())
    gen, n = s.policy.generation, len(s.policy.constraints)
    try:
        s.hot_apply(fence())
    except LockedRuleClass as e:
        assert "locked at mission start" in str(e) and "as_dynamic_nfz" in str(e)
    else:
        raise AssertionError("a polygon_fence was hot-applied after take-off")
    assert (s.policy.generation, len(s.policy.constraints)) == (gen, n)
    z = as_dynamic_nfz(fence())
    assert z.type == "dynamic_nfz" and z.margin_m == fence().margin_m
    assert [(v.x, v.y) for v in z.vertices] == [(v.x, v.y) for v in fence().vertices]
    s.hot_apply(z)
    assert s.policy.generation == gen + 1


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


# The five P0 escape ticks of the 8 Oct red-car follow (demo/out/
# pas_ap_20261008_224440, flight_log.jsonl): the logged state and raw action,
# and the subject where the perception node had put it - exactly under the
# aircraft (a depth return nearer than the camera's height clamped to a
# horizontal range of 0; the log rounds it to 1 cm). Policy follow-car-
# citylife: standoff-any 5 m, hard P0; kin-caps 5 m/s. Tick 1100 also had a
# clearance repair; the occupancy map it needs is not in the repository.
FLIGHT_20261008_ESCAPES = (
    # tick, x, y, up, yaw_deg, raw vx, raw vy, raw vz_up, raw yaw_rate
    (857, 36.847, -48.38, 7.962, -99.78, -0.2329274635260671, -1.3506142592864019,
     0.022800000000000153, 0.024335576298841224),
    (927, 36.78, -49.618, 7.656, -122.72, -1.351323832833803, -2.1032865154849554,
     0.20640000000000017, 0.6132928977960882),
    (928, 36.758, -49.655, 7.683, -122.28, -1.3350221786371677, -2.113671225605429,
     0.1902000000000001, 0.6132928977960882),
    (999, 35.956, -50.251, 7.745, -136.49, -1.6663927657452855, -1.5819485365379262,
     0.15299999999999994, 0.6522187346889204),
    (1100, 34.854, -51.237, 7.986, -112.77, -1.1267271260310219, -2.684560428475885,
     0.00840000000000014, 0.10296043387283836),
)


def _follow_car_policy():
    from guardrail.models import Policy
    return Policy.model_validate({"policy_id": "follow-car-citylife", "version": "0.1.0",
                                  "constraints": [
        {"id": "standoff-pedestrian", "type": "subject_standoff", "subject_class": "pedestrian",
         "min_range_m": 10.0, "soft_margin_m": 2.0, "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair"},
        {"id": "standoff-any", "type": "subject_standoff", "subject_class": "*",
         "min_range_m": 5.0, "soft_margin_m": 1.0, "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair"},
        {"id": "alt-band", "type": "altitude_envelope", "alt_min_m": 6.0, "alt_max_m": 14.0,
         "constraint_type": "hard", "priority": "P0", "violation_action": "repair"},
        {"id": "kin-caps", "type": "kinematic_envelope", "speed_max_mps": 5.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0, "constraint_type": "hard",
         "priority": "P1", "violation_action": "repair"}]})


def test_the_8_oct_escapes_with_the_subject_directly_below_are_gone():
    """REGRESSION (pas_ap_20261008_224440, 5 P0 escape ticks). With the
    subject at range 0 the check's "opening" was the action's component along
    a zero vector - 0 for every action - so nothing could pass; the repair
    returned the action untouched ("no defined line"); the P0 guard flew a
    best-effort ClearanceEscape with standoff-any still violated. On the old
    shield.py all five ticks emit standoff-any. From range 0 every horizontal
    motion opens the range at its full speed, which is what the check now
    reads."""
    sh = Shield(_follow_car_policy(), escalation=False)
    for tick, x, y, up, yaw, vx, vy, vz, wz in FLIGHT_20261008_ESCAPES:
        st = State(x=x, y=y, up=up, yaw_deg=yaw)
        sh.set_subject(x, y, "vehicle")                  # range 0, as the node had it
        d = sh.filter(st, Action4D(vx=vx, vy=vy, vz_up=vz, yaw_rate=wz))
        p0 = [v.rule_id for v in d.emitted_violations if v.rule_id != "kin-caps"]
        assert not p0, f"tick {tick}: P0 still flown: {p0} ({d.repairs})"
        assert math.hypot(d.emitted.vx, d.emitted.vy) > 0.1, f"tick {tick}: not leaving"


def test_a_hover_directly_over_the_subject_backs_away_from_the_heading():
    """No commanded motion and no line to the subject: StandoffRecover backs
    off along the reverse of the heading (which brings the subject back in
    front of a forward camera) at the ring's recovery speed, flagged as a
    recovery. A hover there is still a violation - stopping keeps the
    aircraft at the centre of the ring - so the check does flag it."""
    sh = Shield(_follow_car_policy(), escalation=False)
    st = State(x=10.0, y=-5.0, up=8.0, yaw_deg=90.0)        # facing East (+y)
    sh.set_subject(10.0, -5.0, "vehicle")
    assert [v.rule_id for v in sh._check(st, Action4D())] == ["standoff-any"]
    d = sh.filter(st, Action4D())
    rec = [r for r in d.repairs if r.operator == "StandoffRecover"]
    assert rec and rec[0].recovery and rec[0].axis == "lateral", d.repairs
    assert abs(rec[0].magnitude_m - 5.0) < 1e-9
    assert d.emitted.vy < -0.49 and abs(d.emitted.vx) < 1e-9, d.emitted   # backing West
    assert not d.emitted_violations, d.emitted_violations
    # Any motion at more than 0.1 m/s already opens the range from 0 and
    # passes. Once something else opens the repair loop (an unrelated yaw
    # breach, as in the escape-floor tests below), a slow drift is kept in its
    # direction and floored at the recovery speed (5 m ring / 5 s lookahead =
    # 1.0 m/s), never lowered.
    assert not sh.filter(st, Action4D(vx=0.2)).violations
    d = sh.filter(st, Action4D(vx=0.2, yaw_rate=YAW_BREACH))
    assert abs(d.emitted.vx - 1.0) < 1e-9 and abs(d.emitted.vy) < 1e-9, d.emitted
    d = sh.filter(st, Action4D(vx=3.0, yaw_rate=YAW_BREACH))
    assert abs(d.emitted.vx - 3.0) < 1e-9, d.emitted


def test_a_stand_off_holds_magnitude_is_a_depth_the_ring_can_have():
    """REGRESSION (8 Oct flight: StandoffHold reported 8.1-11.0 m into a 5 m
    ring). It extrapolated the range in a straight line THROUGH the subject:
    ring - (d - closing x lookahead). The deepest a forecast can go into a
    ring is its radius; the field is now the forecast's closest pose. Tick
    201 of the flight: 8.3 m out, closing at 2.27 m/s with a tangential
    component, closest forecast pose 4.2 m from the car - 0.8 m deep (the
    old code said 8.08 m)."""
    sh = Shield(_follow_car_policy(), escalation=False)
    st = State(x=35.11, y=-19.674, up=8.001, yaw_deg=2.14)
    sh.set_subject(42.38, -23.63, "vehicle")
    a = Action4D(vx=2.639389733893821, vy=0.09851453808726945, vz_up=-0.0006,
                 yaw_rate=-0.6427205700542464)
    d = sh.filter(st, a)
    hold = [r for r in d.repairs if r.operator == "StandoffHold"]
    assert hold, d.repairs
    closest = min(math.hypot(p.x - 42.38, p.y + 23.63) for _, p in sh.forecast(st, a))
    assert abs(closest - 4.22) < 0.01, closest
    assert abs(hold[0].magnitude_m - (5.0 - closest)) < 1e-9, (hold[0], closest)
    assert hold[0].magnitude_m < 2.0, "the old code reported 8.08 m here"


def test_unsafe_depth_measures_how_far_the_position_is_from_legal():
    """The escalation FSM's X6 / X7 read this trend. Each hard rule a stop
    here breaks, measured its own way and summed; 0 where a stop is legal;
    soft and monitor_only rules do not count."""
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "depth", "constraints": [
        {"id": "nfz", "type": "polygon_fence", "margin_m": 1.0,
         "vertices": [{"x": 0, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 20},
                      {"x": 0, "y": 20}]},
        {"id": "band", "type": "altitude_envelope", "alt_min_m": 10.0, "alt_max_m": 30.0},
        {"id": "soft-band", "type": "altitude_envelope", "alt_min_m": 12.0,
         "alt_max_m": 30.0, "constraint_type": "soft", "priority": "P2"},
        {"id": "ring", "type": "subject_standoff", "subject_class": "*",
         "min_range_m": 5.0}]})
    sh = Shield(pol)
    assert sh.unsafe_depth(State(x=-30, y=-30, up=15)) == 0.0
    assert abs(sh.unsafe_depth(State(x=-30, y=-30, up=7)) - 3.0) < 1e-9     # 3 m under
    inside = sh.unsafe_depth(State(x=10, y=4, up=15))                          # ring at -1
    assert abs(inside - 5.0) < 1e-6, inside
    sh.set_subject(-30.0, -28.0, "vehicle")
    both = sh.unsafe_depth(State(x=-30, y=-30, up=7))                          # 3 m + 3 m
    assert abs(both - 6.0) < 1e-9, both


def test_applied_caps_leave_out_only_the_soft_caps_the_chain_gave_up():
    """REVIEW (2026-10-09). Shield.applied_caps is what a rail's FSM measures
    theta's |dv| from (one-period reading). A first version left out EVERY
    soft speed cap once any soft rule was given up (SoftRelax). Here a soft
    P2 band is given up and a soft P1 speed cap is still clamped to (4 -> 2
    m/s) while a zone is slid along: that version handed the FSM no caps, and
    on the one-period reading the FSM refused the tick ("needs the policy's
    caps") - a fault, LOITER, nothing streamed. Only the soft rules the chain
    actually gave up are left out now; a soft P2 speed cap given up with the
    band is."""
    from guardrail.models import Policy

    def pol(speed_prio):
        return Policy.model_validate({"policy_id": "soft", "constraints": [
            {"id": "hard-band", "type": "altitude_envelope", "alt_min_m": 0,
             "alt_max_m": 10},
            {"id": "soft-high", "type": "altitude_envelope", "alt_min_m": 20,
             "alt_max_m": 30, "constraint_type": "soft", "priority": "P2"},
            {"id": "soft-speed", "type": "kinematic_envelope", "speed_max_mps": 2.0,
             "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0,
             "constraint_type": "soft", "priority": speed_prio},
            {"id": "nfz", "type": "polygon_fence", "margin_m": 1.0,
             "vertices": [{"x": 7, "y": 7}, {"x": 23, "y": 7}, {"x": 23, "y": 23},
                          {"x": 7, "y": 23}]}]})
    st, fast = State(x=0, y=15, up=8), Action4D(vx=4.0)
    kept = Shield(pol("P1"), theta_horizon_s=0.1)
    d = kept.filter(st, fast)
    assert d.relaxed == ["soft-high"], d.relaxed
    assert {"SpeedClamp", "GeofenceSlide", "SoftRelax"} <= {r.operator for r in d.repairs}
    assert kept.applied_caps.to_dict() == {"speed_max_mps": 2.0, "climb_rate_max_mps": 2.0}
    assert d.fsm_fault is None, d.fsm_fault
    given = Shield(pol("P2"), theta_horizon_s=0.1)
    d = given.filter(st, fast)
    assert "soft-speed" in d.relaxed and "SpeedClamp" not in {r.operator for r in d.repairs}
    assert given.applied_caps is None, given.applied_caps
    assert d.fsm_fault is None, d.fsm_fault


def test_unsafe_depth_off_a_corridor_is_the_excess_over_its_half_width():
    """A corridor's term of Shield.unsafe_depth (review 2026-10-09: dropping
    it passed every test): the lateral excess over the half-width, plus the
    altitude error outside its band."""
    from guardrail.models import Policy
    sh = Shield(Policy.model_validate({"policy_id": "c", "constraints": [
        {"id": "lane", "type": "corridor", "width_m": 10,
         "centerline": [{"x": -50, "y": 0}, {"x": 50, "y": 0}],
         "altitude_floor_m": 5, "altitude_ceiling_m": 30}]}))
    assert sh.unsafe_depth(State(x=0, y=4.0, up=10)) == 0.0
    assert abs(sh.unsafe_depth(State(x=0, y=8.0, up=10)) - 3.0) < 1e-9
    assert abs(sh.unsafe_depth(State(x=0, y=-9.5, up=10)) - 4.5) < 1e-9
    assert abs(sh.unsafe_depth(State(x=0, y=8.0, up=3)) - 5.0) < 1e-9


def test_a_best_effort_recovery_never_deepens_the_ring_it_is_inside():
    """REGRESSION (2026-10-08). Inside a zone 1.5 m from its north face and
    inside a 10 m stand-off ring whose subject is 5.5 m further north: the
    zone's exit and the ring's exit point opposite ways, nothing re-checks
    clean, a stop is illegal, and the Shield flies its best effort. The old
    shield.py ranked that best effort by clearance (none mapped) and then by
    the commanded heading, so a pilot asking to go north got 4 m/s north:
    straight at the subject whose P0 ring it was already inside. The best
    effort is now chosen among headings that go deeper into neither, here
    along the zone's face."""
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "conflict", "constraints": [
        {"id": "nfz", "type": "polygon_fence", "margin_m": 1.0,
         "vertices": [{"x": 0, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 20},
                      {"x": 0, "y": 20}]},
        {"id": "ring", "type": "subject_standoff", "subject_class": "*",
         "min_range_m": 10.0},
        {"id": "kin", "type": "kinematic_envelope", "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}]})
    sh = Shield(pol, escalation=False)
    st = State(x=18.5, y=10.0, up=10.0)          # x North: the zone's north face is x = 20
    sh.set_subject(24.0, 10.0, "vehicle")         # 5.5 m north, inside the 10 m ring
    d = sh.filter(st, Action4D(vx=2.0))
    assert any(r.operator == "ClearanceEscape" for r in d.repairs), d.repairs
    assert d.emitted_violations, "a clean answer exists: the test proves nothing"
    closing = d.emitted.vx                        # the subject is due North
    assert closing <= 1e-9, f"best effort closes on the subject at {closing:.2f} m/s"
    assert d.emitted.vx >= -1e-9, "best effort heads deeper into the zone"
    assert math.hypot(d.emitted.vx, d.emitted.vy) > 1.0, d.emitted   # still moving


_SQ20 = [{"x": 0, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 20}, {"x": 0, "y": 20}]


def test_a_best_effort_recovery_never_heads_deeper_into_the_zone_it_is_inside():
    """The zone half of the rule above (review 2026-10-09: removing the zone
    term of _deepens passed every test, because the commanded heading there
    already pointed out of the zone). Same zone and ring, but the pilot asks
    to go SOUTH, deeper into the zone and away from the subject. Ranked by
    the ring and the commanded heading alone, the best effort was due south
    at 4 m/s; it now slides along the zone's face."""
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "conflict", "constraints": [
        {"id": "nfz", "type": "polygon_fence", "margin_m": 1.0, "vertices": _SQ20},
        {"id": "ring", "type": "subject_standoff", "subject_class": "*",
         "min_range_m": 10.0},
        {"id": "kin", "type": "kinematic_envelope", "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}]})
    st = State(x=18.5, y=10.0, up=10.0)          # the zone's north face is x = 20
    for raw in (Action4D(vx=-2.0), Action4D(vx=-3.0, vy=-1.0), Action4D(vx=-2.0, vy=0.5)):
        sh = Shield(pol, escalation=False)
        sh.set_subject(24.0, 10.0, "vehicle")
        d = sh.filter(st, raw)
        assert d.repairs[-1].operator == "ClearanceEscape", d.repairs
        assert d.emitted_violations, "a clean answer exists: the test proves nothing"
        # push-out of the north face is +x: no component into the zone
        assert d.emitted.vx >= -1e-9, (raw, d.emitted)
        assert math.hypot(d.emitted.vx, d.emitted.vy) > 1.0, d.emitted


def test_deepens_counts_hard_enforced_rules_and_a_moving_zone_relative_to_it():
    """_deepens, rule by rule: a hard zone the aircraft is inside counts a
    heading into it; a monitor_only or soft zone does not (the rescue must
    not be steered by a rule it may break); a zone moving north at 3 m/s is
    entered by flying north at 2 m/s (1 m/s south relative to it) and left
    at 4 m/s."""
    from guardrail.models import Policy

    def zone(**kw):
        z = {"id": "nfz", "type": "polygon_fence", "margin_m": 1.0, "vertices": _SQ20}
        z.update(kw)
        return Policy.model_validate({"policy_id": "z", "constraints": [z]})
    st = State(x=18.5, y=10.0, up=10.0)
    hard = Shield(zone(), escalation=False)
    assert [hard._deepens(st, Action4D(vx=v)) for v in (-2.0, 0.0, 2.0)] == [1, 0, 0]
    assert [hard._deepens(st, Action4D(vy=v)) for v in (-2.0, 2.0)] == [0, 0]
    for kw in ({"violation_action": "monitor_only"},
               {"constraint_type": "soft", "priority": "P2"}):
        sh = Shield(zone(**kw), escalation=False)
        assert sh._deepens(st, Action4D(vx=-2.0)) == 0, kw
    moving = Shield(Policy.model_validate({"policy_id": "mv", "constraints": [
        {"id": "dyn", "type": "dynamic_nfz", "margin_m": 1.0, "vertices": _SQ20,
         "motion": {"vx_mps": 3.0}}]}), escalation=False)
    assert [moving._deepens(st, Action4D(vx=v)) for v in (-2.0, 2.0, 4.0)] == [1, 1, 0]
    # Not inside: nothing to deepen.
    assert hard._deepens(State(x=-30.0, y=-30.0, up=10.0), Action4D(vx=4.0)) == 0





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


def _fifty_variant(kind):
    """The bench's 50-rule policy on one of the Shield's slow paths: "edge",
    every rule with a weekly window whose end falls inside the lookahead (the
    schedule is then read at both sides of every pose's instant); "moving",
    every polygon fence a dynamic_nfz drifting at 1 m/s (materialised per
    tick, no STRtree)."""
    from guardrail.models import Policy
    data = _fifty().fifty_rule_policy().model_dump(mode="json", exclude_none=True)
    for c in data["constraints"]:
        if kind == "edge":
            c["valid_time"] = {"recurrence": {"start_time": "07:30", "end_time": "17:30"}}
        elif kind == "moving" and c["type"] == "polygon_fence":
            c.update(type="dynamic_nfz", motion={"vx_mps": 1.0})
    return Policy.model_validate(data)


def test_the_50_rule_monitor_fits_the_grant_budget():
    """Median _check <= 5 ms and p99 filter() <= 100 ms at the grant's load
    (50 rules, the grant's 5 s lookahead at 0.1 s), on the samples where both
    used to fail worst (1-8 m outside a fence's margin, flying at it). Also on
    the two slow paths: windows with an edge inside the lookahead, and 48
    moving zones (31 ms per _check before the swept-box skip). Desktop
    numbers; the margin is >= 10x, so a loaded machine still passes and a
    return of the per-fence loop does not."""
    import statistics
    import time
    from datetime import datetime
    B = _fifty()
    pairs = B.samples("near", 200, seed=3)
    edge = datetime(2026, 10, 7, 17, 29, 57)            # the 17:30 edge is 3 s ahead
    for label, pol, now in (("static", B.fifty_rule_policy(), None),
                            ("edge", _fifty_variant("edge"), lambda: edge),
                            ("moving", _fifty_variant("moving"), None)):
        sh = Shield(pol, now=now)
        assert (sh.lookahead_s, sh.dt) == (5.0, 0.1)
        ms_check, ms_filter = [], []
        for st, a in pairs:
            t0 = time.perf_counter()
            sh._check(st, a)
            ms_check.append((time.perf_counter() - t0) * 1e3)
        for k, (st, a) in enumerate(pairs[:100]):
            t0 = time.perf_counter()
            sh.filter(st, a, t=0.1 * k)
            ms_filter.append((time.perf_counter() - t0) * 1e3)
        med = statistics.median(ms_check)
        p99 = sorted(ms_filter)[98]
        assert med <= 5.0, f"{label}: _check median {med:.2f} ms > 5 ms at 50 rules"
        assert p99 <= 100.0, f"{label}: filter() p99 {p99:.1f} ms > the 100 ms tick"


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

# ------------------------------------------- enforcement (2026-10-07, WP3-07/06/08)
#
# Each test below was run against guardrail/shield.py as of 401305a and failed
# there (see docs/DESIGN-shield-enforcement.md, "Shown failing first").

def _zone_policy(**kw):
    """demo_policy's square zone (7..23 m, 1 m margin) plus a speed cap, with
    the zone's fields overridden by `kw`."""
    from guardrail.models import Policy
    zone = {"id": "nfz", "type": "polygon_fence", "margin_m": 1.0,
            "vertices": [{"x": 7, "y": 7}, {"x": 23, "y": 7},
                         {"x": 23, "y": 23}, {"x": 7, "y": 23}]}
    zone.update(kw)
    return Policy.model_validate({"policy_id": "zone", "constraints": [
        zone, {"id": "kin", "type": "kinematic_envelope", "priority": "P1",
               "speed_max_mps": 4.0, "climb_rate_max_mps": 2.0,
               "yaw_rate_max_dps": 45.0}]})


def test_the_default_lookahead_is_the_grants_5_s_at_10_hz():
    """Safety Shield page: "Lookahead horizon: 5 s of predicted trajectory at
    10 Hz = 50 future poses". The default was 3 s at 0.5 s (7 poses); a rail
    that passes its own horizon keeps it."""
    s = Shield(load_policy(POLICY_PATH))
    assert (s.lookahead_s, s.dt) == (5.0, 0.1)
    poses = s.forecast(State(x=0, y=0, up=4), Action4D(vx=1.0))
    assert len(poses) == 51 and abs(poses[-1][0] - 5.0) < 1e-9
    assert len(make_shield().forecast(State(x=0, y=0, up=4), Action4D(vx=1.0))) == 7


def test_every_tick_feeds_the_escalation_fsm():
    """A clean tick stays Normal and streams the action; a small trusted repair
    enters Brake by G1 and still streams the repair; 2 s clean recovers by G7
    (T_recover), not a tick earlier."""
    s = Shield(load_policy(POLICY_PATH))
    far = State(x=-20, y=-20, up=4)
    d = s.filter(far, Action4D(vx=2.0))
    assert (d.fsm_state_before, d.fsm_state_after, d.setpoint) == ("Normal", "Normal", "pass")
    assert d.command == d.emitted == d.raw and d.set_mode is None
    assert d.generation == 0 and d.policy_hash == s.policy.policy_hash
    d = s.filter(State(x=0, y=15, up=4), Action4D(vx=1.3))     # 0.5 m deep forecast
    assert [r.operator for r in d.repairs] == ["GeofenceSlide"], d.repairs
    assert (d.fsm_edge, d.fsm_state_after, d.setpoint) == ("G1", "Brake", "pass")
    assert d.command == d.emitted != d.raw
    edges = [s.filter(far, Action4D(vx=2.0)).fsm_edge for _ in range(21)]
    assert edges[:20] == [None] * 20 and edges[20] == "G7", edges


def test_a_repair_reports_its_size_on_the_grants_axis():
    """The grant's audit field `magnitude_m` per repair (Safety Shield p5):
    the forecast's penetration depth for a lateral projection, the altitude
    error for a vertical one; a repair of a position already outside the rule
    is a recovery (exempt from theta)."""
    from shapely.geometry import Point, Polygon
    s = Shield(load_policy(POLICY_PATH))
    st, a = State(x=0, y=15, up=4), Action4D(vx=3.0)
    d = s.filter(st, a)
    slide = [r for r in d.repairs if r.operator == "GeofenceSlide"]
    ring = Polygon([(7, 7), (23, 7), (23, 23), (7, 23)]).buffer(1.0)
    deepest = max(ring.boundary.distance(Point(p.x, p.y))
                  for _, p in s.forecast(st, a) if ring.contains(Point(p.x, p.y)))
    assert len(slide) == 1 and slide[0].axis == "lateral" and not slide[0].recovery
    assert abs(slide[0].magnitude_m - deepest) < 1e-9, (slide[0].magnitude_m, deepest)
    up = Shield(load_policy(POLICY_PATH)).filter(State(x=-20, y=-20, up=5),
                                                 Action4D(vz_up=1.0))
    fix = [r for r in up.repairs if r.operator == "AltitudeFix"][0]
    assert (fix.axis, fix.recovery) == ("vertical", False)
    assert abs(fix.magnitude_m - 4.0) < 1e-9          # 5 + 1 x 5 s against a 6 m ceiling
    out = Shield(load_policy(POLICY_PATH)).filter(State(x=-20, y=-20, up=8), Action4D())
    assert [r.recovery for r in out.repairs if r.operator == "AltitudeFix"] == [True]


def test_theta_defaults_to_the_operators_depth_and_one_period_is_opt_in():
    """REVIEW (2026-10-09). A builder switched the Shield's default theta
    basis to the setpoint change over one 0.1 s period and called it the
    grant's. The grant's own audit example logs LateralProjection with
    magnitude_m 0.7 for a 1.53 m/s change (Safety Shield p5) - not |dv| x
    0.1 s - and the PI reference escalates a deep head-on dive "rather than
    projected". Under the shipped 4-5 m/s caps the one-period reading can
    never exceed theta, so making it the default waived the cap before the
    PI answered. This slide's forecast goes 9 m into the zone's ring: on the
    default basis it is escalated (X1, a stop, then LOITER); the one-period
    reading, opted into, trusts it (0.3 m). Failed on the builder's code,
    whose default Shield trusted it (G1)."""
    s = Shield(load_policy(POLICY_PATH))
    assert s.theta_horizon_s is None
    st, a = State(x=0, y=15, up=4), Action4D(vx=3.0)
    d = s.filter(st, a)
    assert d.repairs[0].magnitude_m > 2.0               # the forecast depth, reported
    assert d.fsm_record["theta_exceeded"] is True
    assert abs(d.fsm_record["magnitude"]["lateral_m"] - d.repairs[0].magnitude_m) < 1e-6
    assert (d.fsm_edge, d.setpoint, d.command) == ("X1", "brake", Action4D())
    d = s.filter(st, a)
    assert (d.fsm_edge, d.fsm_state_after, d.set_mode) == ("G3", "Loiter", "LOITER")
    # The one-period reading is selectable, and trusts the same repair.
    one = Shield(load_policy(POLICY_PATH), theta_horizon_s=0.1)
    d = one.filter(st, a)
    dv = math.hypot(d.emitted.vx - d.raw.vx, d.emitted.vy - d.raw.vy)
    mag = d.fsm_record["magnitude"]
    assert abs(mag["lateral_m"] - dv * 0.1) < 1e-6 and mag["lateral_m"] < 2.0, mag
    assert d.fsm_record["theta_exceeded"] is False
    assert (d.fsm_edge, d.setpoint, d.command) == ("G1", "pass", d.emitted)
    for _ in range(5):
        d = one.filter(st, a)
        assert d.fsm_state_after == "Brake" and d.set_mode is None, d.fsm_record["reason"]


def test_theta_is_measured_from_the_caps_the_chain_applied_this_tick():
    """REGRESSION (2026-10-09, found while trying theta on one control
    period). The one-period measure starts from the raw action clamped to the
    speed cap. The FSM read the caps off the whole policy, so a cap the chain
    did not apply - `monitor_only`, or switched off by a time_window_switch -
    made a 6 m/s pass with a slide "a decision made under other caps": the
    FSM refused the tick, the Shield requested LOITER and streamed nothing
    more (fsm_fault). The Shield now hands the FSM the caps it clamped
    against on that tick. (The default basis, each repair's magnitude_m,
    needs no caps; it is checked here too.)"""
    from guardrail.models import Policy
    zone = {"id": "nfz", "type": "polygon_fence", "margin_m": 1.0,
            "vertices": [{"x": 7, "y": 7}, {"x": 23, "y": 7},
                         {"x": 23, "y": 23}, {"x": 7, "y": 23}]}
    kin = {"id": "kin", "type": "kinematic_envelope", "speed_max_mps": 4.0,
           "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}
    st, fast = State(x=0, y=15, up=4), Action4D(vx=6.0)
    watched = Policy.model_validate({"policy_id": "mo", "constraints": [
        zone, dict(kin, violation_action="monitor_only", constraint_type="soft",
                   priority="P2")]})
    assert Shield(watched).filter(st, fast).fsm_fault is None
    sh = Shield(watched, theta_horizon_s=0.1)
    d = sh.filter(st, fast)
    assert d.fsm_fault is None and d.set_mode is None, d.fsm_fault
    assert sh.applied_caps is None, "a watched-only cap clamps nothing"
    assert [r.operator for r in d.repairs] == ["GeofenceSlide"], d.repairs
    dv = math.hypot(d.emitted.vx - fast.vx, d.emitted.vy - fast.vy)
    assert abs(d.fsm_record["magnitude"]["lateral_m"] - dv * 0.1) < 1e-6
    switched = Shield(Policy.model_validate({"policy_id": "sw", "constraints": [zone, kin]}),
                      theta_horizon_s=0.1)
    switched.filter(State(x=-40, y=-40, up=4), Action4D(vx=1.0), t=0.0)
    switched.set_rule_active("kin", False, t=0.1)
    d = switched.filter(st, fast, t=0.2)
    assert d.fsm_fault is None and d.fsm_edge == "G1", (d.fsm_fault, d.fsm_edge)
    assert switched.applied_caps is None        # what a rail's own FSM is handed
    # With the cap applied, the measure still starts from the clamped action.
    capped = Shield(Policy.model_validate({"policy_id": "cap", "constraints": [zone, kin]}),
                    theta_horizon_s=0.1)
    d = capped.filter(st, fast)
    assert "SpeedClamp" in [r.operator for r in d.repairs] and d.fsm_fault is None
    assert capped.applied_caps.to_dict() == {"speed_max_mps": 4.0, "climb_rate_max_mps": 2.0}
    dv = math.hypot(d.emitted.vx - 4.0, d.emitted.vy)
    assert abs(d.fsm_record["magnitude"]["lateral_m"] - dv * 0.1) < 1e-6, d.fsm_record


def test_theta_still_escalates_a_setpoint_change_beyond_it():
    """On the one-period reading theta can still fire, but only without a
    speed cap: a 30 m/s command into a zone with no speed cap is slid to a
    4 m/s tangent, a 30.3 m/s change of setpoint, 3.0 m over one period
    (theta 2.0 m): X1 with a stop, then LOITER (G3). The emitted repair is
    unchanged. (Under a 5 m/s / 2 m/s envelope a clamped command can change
    by at most 1.0 m / 0.4 m per period, so there it cannot fire, which is
    why it is not the default.) The default basis escalates it too."""
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "zone-uncapped", "constraints": [
        {"id": "nfz", "type": "polygon_fence", "margin_m": 1.0,
         "vertices": [{"x": 7, "y": 7}, {"x": 23, "y": 7},
                      {"x": 23, "y": 23}, {"x": 7, "y": 23}]}]})
    assert Shield(pol).filter(State(x=0, y=15, up=4), Action4D(vx=30.0)).fsm_edge == "X1"
    s = Shield(pol, theta_horizon_s=0.1)
    st, a = State(x=0, y=15, up=4), Action4D(vx=30.0)
    d = s.filter(st, a)
    assert [r.operator for r in d.repairs] == ["GeofenceSlide"], d.repairs
    assert d.fsm_record["magnitude"]["lateral_m"] > 2.0, d.fsm_record["magnitude"]
    assert d.fsm_record["theta_exceeded"] is True
    assert (d.fsm_edge, d.setpoint, d.command) == ("X1", "brake", Action4D())
    assert d.emitted != Action4D() and not d.braked     # the repair, as before
    d = s.filter(st, a)
    assert (d.fsm_edge, d.fsm_state_after, d.set_mode) == ("G3", "Loiter", "LOITER")
    assert d.setpoint == "brake"                        # the hand-over tick
    d = s.filter(st, a)
    assert (d.setpoint, d.command, d.set_mode) == ("none", None, None)


def test_monitor_only_is_recorded_and_never_repaired():
    s = Shield(_zone_policy(violation_action="monitor_only"))
    d = s.filter(State(x=0, y=15, up=4), Action4D(vx=3.0))
    assert [v.rule_id for v in d.violations] == ["nfz"]
    assert d.repairs == [] and not d.braked and d.emitted == d.raw
    assert [v.rule_id for v in d.emitted_violations] == ["nfz"]   # flown, and said so
    assert (d.fsm_state_after, d.setpoint) == ("Normal", "pass")


def test_monitor_only_is_left_alone_when_another_rule_is_repaired():
    """A tick that repairs a speed cap must not, while it is at it, steer away
    from a monitor_only zone: only the cap is enforced, and the zone is still
    on record as flown into."""
    s = Shield(_zone_policy(violation_action="monitor_only"))
    d = s.filter(State(x=0, y=15, up=4), Action4D(vx=6.0))
    assert sorted(v.rule_id for v in d.violations) == ["kin", "nfz"]
    assert [r.operator for r in d.repairs] == ["SpeedClamp"], d.repairs
    assert d.emitted == Action4D(vx=4.0)
    assert [v.rule_id for v in d.emitted_violations] == ["nfz"]


def test_a_brake_rule_skips_projection_and_stops():
    s = Shield(_zone_policy(violation_action="brake"))
    d = s.filter(State(x=0, y=15, up=4), Action4D(vx=3.0))
    assert d.braked and d.emitted == Action4D()
    assert [r.operator for r in d.repairs] == ["Brake"]
    assert "projection skipped" in d.repairs[0].detail
    assert (d.fsm_edge, d.setpoint) == ("X1", "brake")


def test_direct_failsafe_rules_request_their_mode():
    """violation_action RTL / land / loiter: projection skipped, the mode
    requested on the transition, a stop on the hand-over tick, nothing after."""
    for act, edge, mode in (("RTL", "G11", "RTL"), ("land", "G12", "LAND"),
                            ("loiter", "X4", "LOITER")):
        s = Shield(_zone_policy(violation_action=act))
        d = s.filter(State(x=0, y=15, up=4), Action4D(vx=3.0))
        assert d.braked and (d.fsm_edge, d.set_mode, d.setpoint) == (edge, mode, "brake"), act
        d = s.filter(State(x=0, y=15, up=4), Action4D(vx=3.0))
        assert (d.set_mode, d.setpoint, d.command) == (None, "none", None), act


def test_the_autopilots_report_ends_the_shields_failsafe():
    """The FSM's terminal edges need what only the autopilot knows: home
    reached (G9), landed (G10), RTL failed (G6). filter() takes them, so a
    caller that streams `command` and requests `set_mode` from the Shield's
    own FSM can finish an episode instead of holding RTL forever."""
    st, a = State(x=0, y=15, up=4), Action4D(vx=3.0)
    s = Shield(_zone_policy(violation_action="RTL"))
    assert s.filter(st, a, t=0.0).fsm_edge == "G11"
    d = s.filter(st, a, t=0.1, home_reached=True)
    assert (d.fsm_edge, d.fsm_record["terminal"], d.command) == ("G9", True, None)
    s = Shield(_zone_policy(violation_action="RTL"))
    s.filter(st, a, t=0.0)
    d = s.filter(st, a, t=0.1, rtl_failed=True)
    assert (d.fsm_edge, d.set_mode, d.fsm_state_after) == ("G6", "LAND", "Land")
    d = s.filter(st, a, t=5.0, landed=True)
    assert (d.fsm_edge, d.fsm_record["terminal"]) == ("G10", True)


def test_an_rtl_rule_inside_its_zone_recovers_rather_than_stopping():
    """Where a stop is illegal (inside the zone) the Shield still recovers and
    the stop is withheld, while the mode change goes through."""
    s = Shield(_zone_policy(violation_action="RTL"))
    d = s.filter(State(x=10, y=15, up=4), Action4D(vx=0.5))
    assert not d.braked and [r.operator for r in d.repairs] == ["GeofenceEscape"]
    assert (d.fsm_edge, d.set_mode, d.setpoint) == ("G11", "RTL", "pass")
    assert d.command == d.emitted and d.emitted.vx < 0          # flying out
    assert d.fsm_record["stop_withheld"] is True


def test_a_soft_rule_is_capped_at_brake():
    """A soft rule written with RTL stops the aircraft but never changes the
    flight mode (fsm.RuleHit.effective_action), and the record says it was
    capped."""
    s = Shield(_zone_policy(violation_action="RTL", constraint_type="soft"))
    d = s.filter(State(x=0, y=15, up=4), Action4D(vx=3.0))
    assert d.braked and d.set_mode is None and d.fsm_state_after == "Brake"
    gov = d.fsm_record["governing_rule"]
    assert gov["capped"] is True and gov["effective_action"] == "brake"


def _soft_conflict(p_high="P2", p_mid="P1"):
    from guardrail.models import Policy
    return Policy.model_validate({"policy_id": "soft", "constraints": [
        {"id": "hard-band", "type": "altitude_envelope", "alt_min_m": 0, "alt_max_m": 10},
        {"id": "soft-high", "type": "altitude_envelope", "alt_min_m": 20,
         "alt_max_m": 30, "constraint_type": "soft", "priority": p_high},
        {"id": "soft-mid", "type": "altitude_envelope", "alt_min_m": 3,
         "alt_max_m": 8, "constraint_type": "soft", "priority": p_mid}]})


def test_the_repair_chain_gives_up_a_soft_rule_lowest_priority_first():
    """Hard band 0-10 m, soft P2 band 20-30 m, soft P1 band 3-8 m, aircraft
    at 8 m. No action satisfies all three. The P2 rule is given up, the P1 and
    hard rules are kept, and the command is left alone horizontally. Before,
    the chain failed, the stop counted as illegal (the soft rule), and the
    rescue search flew a 4 m/s heading nobody asked for."""
    d = Shield(_soft_conflict()).filter(State(x=0, y=0, up=8), Action4D(vx=1.0))
    assert d.relaxed == ["soft-high"], d.relaxed
    assert [r.operator for r in d.repairs] == ["SoftRelax"]
    assert d.emitted == Action4D(vx=1.0) and not d.braked
    assert [v.rule_id for v in d.emitted_violations] == ["soft-high"]
    assert (d.fsm_edge, d.setpoint) == ("G1", "pass")
    # Priorities swapped: the P2 rule (soft-mid) goes first and does not help,
    # so the P1 rule is given up too; only a rule actually broken is listed.
    d = Shield(_soft_conflict("P1", "P2")).filter(State(x=0, y=0, up=8), Action4D(vx=1.0))
    assert d.relaxed == ["soft-high"] and d.emitted == Action4D(vx=1.0)


def test_the_lower_priority_soft_rule_goes_first_when_either_would_do():
    """Two soft bands that cannot both hold (0-5 m and 7-10 m) inside a hard
    0-10 m band, aircraft at 6 m. Giving up EITHER one would satisfy the rest;
    the grant's order says the lower priority goes. Swap the priorities and
    the other one goes, and the aircraft moves the other way."""
    from guardrail.models import Policy

    def pol(p_low, p_up):
        return Policy.model_validate({"policy_id": "s", "constraints": [
            {"id": "hard-band", "type": "altitude_envelope", "alt_min_m": 0, "alt_max_m": 10},
            {"id": "soft-low", "type": "altitude_envelope", "alt_min_m": 0, "alt_max_m": 5,
             "constraint_type": "soft", "priority": p_low},
            {"id": "soft-up", "type": "altitude_envelope", "alt_min_m": 7, "alt_max_m": 10,
             "constraint_type": "soft", "priority": p_up}]})
    st, a = State(x=0, y=0, up=6), Action4D(vx=1.0)
    d = Shield(pol("P2", "P1")).filter(st, a)
    assert d.relaxed == ["soft-low"] and d.emitted.vz_up > 0, (d.relaxed, d.emitted)
    d = Shield(pol("P1", "P2")).filter(st, a)
    assert d.relaxed == ["soft-up"] and d.emitted.vz_up < 0, (d.relaxed, d.emitted)


def test_a_stop_that_breaks_only_a_soft_rule_is_allowed():
    """Inside a SOFT zone whose action is brake: the stop leaves the aircraft
    inside a rule that allows a stop as its response, so the Shield stops
    rather than recovering. (Only a HARD rule makes a stop illegal.)"""
    s = Shield(_zone_policy(violation_action="brake", constraint_type="soft"))
    d = s.filter(State(x=10, y=15, up=4), Action4D(vx=0.5))
    assert d.braked and d.emitted == Action4D(), d
    assert [r.operator for r in d.repairs] == ["Brake"]
    hard = Shield(_zone_policy(violation_action="brake")).filter(
        State(x=10, y=15, up=4), Action4D(vx=0.5))
    assert not hard.braked and [r.operator for r in hard.repairs] == ["GeofenceEscape"]


def test_a_hard_rule_is_never_given_up():
    """Two hard rules that cannot both hold (a 0-10 m band and a corridor whose
    floor is 20 m, aircraft at 5 m inside the corridor's width): nothing is
    relaxed, the fallback keeps both rules on record as still broken, and the
    FSM escalates (X1). A stop is illegal under the corridor floor, so the
    Shield's best-effort recovery is streamed, not a stop."""
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "hard", "constraints": [
        {"id": "low", "type": "altitude_envelope", "alt_min_m": 0, "alt_max_m": 10},
        {"id": "lane", "type": "corridor", "width_m": 20,
         "centerline": [{"x": -50, "y": 0}, {"x": 50, "y": 0}],
         "altitude_floor_m": 20, "altitude_ceiling_m": 30}]})
    d = Shield(pol).filter(State(x=0, y=0, up=5), Action4D(vx=1.0))
    assert d.relaxed == [] and not any(r.operator == "SoftRelax" for r in d.repairs)
    assert [v.rule_id for v in d.emitted_violations] == ["low"]
    assert (d.fsm_edge, d.setpoint) == ("X1", "pass")
    assert d.fsm_record["stop_withheld"] is True


def test_an_fsm_contract_break_requests_loiter_once_and_stops_streaming():
    """A position operator that reports no magnitude_m cannot be judged by
    theta on the per-operator basis (theta_horizon_s=None, selectable since
    the grant's one-period basis became the default on 2026-10-08). The FSM
    refuses the tick; the Shield must not crash the loop: it records the
    fault, requests LOITER on that tick only, and streams nothing until
    reset_episode()."""
    class NoMagnitude(Shield):
        def _repair_geofence(self, state, a, repairs):
            out = super()._repair_geofence(state, a, repairs)
            for i, r in enumerate(repairs):
                if r.operator == "GeofenceSlide":
                    repairs[i] = r.model_copy(update={"magnitude_m": None, "axis": None})
            return out
    s = NoMagnitude(load_policy(POLICY_PATH), theta_horizon_s=None)
    d = s.filter(State(x=0, y=15, up=4), Action4D(vx=1.3))
    assert d.fsm_fault and "magnitude_m" in d.fsm_fault
    assert (d.set_mode, d.setpoint, d.command) == ("LOITER", "none", None)
    assert d.emitted != Action4D()                  # the repair itself is untouched
    d = s.filter(State(x=-20, y=-20, up=4), Action4D(vx=2.0))
    assert d.fsm_fault and d.set_mode is None and d.command is None
    s.reset_episode()
    d = s.filter(State(x=-20, y=-20, up=4), Action4D(vx=2.0))
    assert d.fsm_fault is None and d.command == d.emitted


def test_escalation_off_keeps_the_decision_as_it_was():
    s = Shield(load_policy(POLICY_PATH), escalation=False)
    d = s.filter(State(x=0, y=15, up=4), Action4D(vx=3.0))
    assert s.fsm is None and d.fsm_state_after is None and d.setpoint is None
    assert d.command == d.emitted and d.set_mode is None


def _windowed(end="17:30", start="07:30"):
    from guardrail.models import Policy
    return Policy.model_validate({"policy_id": "w", "constraints": [{
        "id": "yard", "type": "polygon_fence", "margin_m": 0.0,
        "vertices": [{"x": -5, "y": -5}, {"x": 5, "y": -5}, {"x": 5, "y": 5},
                     {"x": -5, "y": 5}],
        "valid_time": {"recurrence": {"days": ["Mon", "Tue", "Wed", "Thu", "Fri"],
                                      "start_time": start, "end_time": end}}}]})


def _at(hhmmss: str):
    from datetime import datetime
    fmt = "%Y-%m-%d %H:%M:%S.%f" if "." in hhmmss else "%Y-%m-%d %H:%M:%S"
    when = datetime.strptime("2026-10-07 " + hhmmss, fmt)   # a Wednesday
    return lambda: when


def test_a_window_ending_17_30_ends_at_17_30_00():
    """models.Recurrence compares whole minutes, so the school-yard zone of the
    grant's worked example stayed in force until 17:30:59. The Shield reads the
    window to the microsecond and checks both sides of the instant (eps =
    dt / 2 = 50 ms): in force at 17:30:00.000 and 17:30:00.040, off at
    17:30:00.060 and at 17:30:30."""
    inside = State(x=0, y=0, up=5)
    for hhmmss, on in (("17:29:59", True), ("17:30:00", True), ("17:30:00.040", True),
                       ("17:30:00.060", False), ("17:30:30", False)):
        s = Shield(_windowed(), now=_at(hhmmss))
        assert bool(s.state_is_unsafe(inside)) is on, hhmmss


def test_the_lookahead_reads_the_rules_in_force_at_each_poses_time():
    """At 07:29:57, hovering inside the yard that switches on at 07:30:00: the
    pose 3 s ahead is in the zone while it is in force, so the violation is
    predicted at t + 3.0 s (both sides of 07:30 put the 50 ms before it in
    force too, so the 2.9 s pose is not). Before, the forecast ignored the
    switch-on entirely."""
    s = Shield(_windowed(), now=_at("07:29:57"))
    v = s._check(State(x=0, y=0, up=5), Action4D())
    assert [(x.rule_id, round(x.predicted_at_s, 6)) for x in v] == [("yard", 3.0)], v
    s = Shield(_windowed(), now=_at("07:29:59.960"))
    assert [round(x.predicted_at_s, 6) for x in s._check(State(x=0, y=0, up=5), Action4D())] == [0.0]


def test_the_default_end_of_day_is_the_end_of_the_day():
    """`end_time` defaults to 23:59. Read as the instant 23:59:00 it would
    switch every all-day schedule off for the last minute of each day."""
    s = Shield(_windowed(end="23:59", start="00:00"), now=_at("23:59:30"))
    assert s.state_is_unsafe(State(x=0, y=0, up=5))


def test_rules_the_shield_cannot_enforce_are_refused_at_construction():
    """A Policy built directly skips the flight loaders' gate; the Shield
    must not then fly it with the rule silently ignored."""
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "x", "constraints": [
        {"id": "people", "type": "distance_envelope", "object_class": "people",
         "min_distance_m": 10}]})
    try:
        Shield(pol)
    except ValueError as e:
        assert "people" in str(e) and "distance_envelope" in str(e)
    else:
        raise AssertionError("a distance_envelope policy was accepted")
    msl = Policy.model_validate({"policy_id": "x", "constraints": [
        {"id": "band", "type": "altitude_envelope", "alt_min_m": 0, "alt_max_m": 50,
         "altitude_ref": "MSL"}]})
    try:
        Shield(msl)
    except ValueError as e:
        assert "MSL" in str(e)
    else:
        raise AssertionError("an MSL band was accepted")


# ---------------------------------------------------------------------------
# Added after the 2026-10-07 review: behaviours its mutation run changed
# without any test noticing (each assertion below fails on that mutation).

def test_the_last_instant_of_a_window_is_still_in_force():
    """The window is CLOSED: 17:30:00.000 is its last instant, 17:30:00.000001
    is outside. A tick at 17:30:00.050 reads 17:30:00.000 as its t - eps side,
    so the zone is still seen. (An open end, `start <= t < end`, passed every
    other window test: their t - eps side always fell before 17:30.)"""
    from datetime import datetime
    from guardrail.shield import recurrence_active
    rec = _windowed().constraints[0].valid_time.recurrence
    last = datetime(2026, 10, 7, 17, 30, 0)
    assert recurrence_active(rec, last)
    assert not recurrence_active(rec, last.replace(microsecond=1))
    s = Shield(_windowed(), now=_at("17:30:00.050"))
    assert [v.rule_id for v in s.state_is_unsafe(State(x=0, y=0, up=5))] == ["yard"]


def test_a_window_past_midnight_ends_at_midnight_when_the_next_day_is_not_listed():
    """Mon 22:00-06:00, Monday only (models.py's day attribution: an instant is
    in force only if its own weekday is listed). At Mon 23:59:58 the aircraft
    reaches the zone at Tue 00:00:01 at 3 m/s - the zone is off by then - and
    at Mon 23:59:59.3 at 8 m/s, while it is on. The fast path needs midnight
    as an edge: without it the tick reads the zone once, as on, for the whole
    horizon."""
    from datetime import datetime
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "n", "constraints": [
        {"id": "nfz-night", "type": "polygon_fence", "margin_m": 0.0,
         "vertices": [{"x": 10, "y": -5}, {"x": 20, "y": -5}, {"x": 20, "y": 5},
                      {"x": 10, "y": 5}],
         "valid_time": {"recurrence": {"days": ["Mon"], "start_time": "22:00",
                                       "end_time": "06:00"}}}]})
    mon = datetime(2026, 10, 5, 23, 59, 58)
    s = Shield(pol, now=lambda: mon, escalation=False)
    assert s._check(State(x=0, y=0, up=4), Action4D(vx=3.0)) == []
    v = s._check(State(x=0, y=0, up=4), Action4D(vx=8.0))
    assert [(x.rule_id, round(x.predicted_at_s, 6)) for x in v] == [("nfz-night", 1.3)], v


def test_a_cap_that_comes_into_force_later_is_broken_then_not_now():
    """A speed cap in force from 12:00, checked at 11:59:58: the 3 m/s command
    breaks it 2 s ahead (the first pose whose t + eps side is 12:00), not at
    t = 0. Inside the window it is broken now."""
    from datetime import datetime
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "k", "constraints": [
        {"id": "kin-noon", "type": "kinematic_envelope", "speed_max_mps": 2.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0,
         "valid_time": {"recurrence": {"start_time": "12:00", "end_time": "13:00"}}}]})
    for when, at in ((datetime(2026, 10, 7, 11, 59, 58), 2.0),
                     (datetime(2026, 10, 7, 12, 0, 1), 0.0)):
        s = Shield(pol, now=lambda w=when: w, escalation=False)
        v = s._check(State(x=0, y=0, up=5), Action4D(vx=3.0))
        assert [(x.rule_id, round(x.predicted_at_s, 6)) for x in v] == [("kin-noon", at)], v


def test_relaxed_lists_only_the_soft_rules_the_flown_action_breaks():
    """Hard speed cap 1.5 m/s and band 0-10 m; soft P2 cap 2 m/s; soft P1 band
    20-30 m; aircraft at 8 m commanding 3 m/s. Both soft rules are given up
    (the P1 band cannot hold beside the hard band), but the hard clamp to
    1.5 m/s satisfies the soft cap anyway: `relaxed` names only the band, the
    one rule the flown action still breaks."""
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "r", "constraints": [
        {"id": "hard-band", "type": "altitude_envelope", "alt_min_m": 0, "alt_max_m": 10},
        {"id": "hard-kin", "type": "kinematic_envelope", "speed_max_mps": 1.5,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0},
        {"id": "soft-kin", "type": "kinematic_envelope", "speed_max_mps": 2.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0,
         "constraint_type": "soft", "priority": "P2"},
        {"id": "soft-high", "type": "altitude_envelope", "alt_min_m": 20,
         "alt_max_m": 30, "constraint_type": "soft", "priority": "P1"}]})
    d = Shield(pol).filter(State(x=0, y=0, up=8), Action4D(vx=3.0))
    assert sorted(v.rule_id for v in d.violations) == ["hard-kin", "soft-high", "soft-kin"]
    assert d.relaxed == ["soft-high"], d.relaxed
    assert d.emitted == Action4D(vx=1.5)
    assert [v.rule_id for v in d.emitted_violations] == ["soft-high"]


def test_rule_status_reports_no_breach_for_a_rule_out_of_its_window():
    """Hovering inside the school-yard zone at 18:00 (out of its window): the
    row says inside, not in force, and no breach. At noon it is a breach."""
    from datetime import datetime
    inside = State(x=0, y=0, up=5)
    row = Shield(_windowed(), now=lambda: datetime(2026, 10, 7, 18, 0)).rule_status(inside)[0]
    assert (row["inside"], row["in_force"], row["breach"]) == (True, False, False), row
    row = Shield(_windowed(), now=lambda: datetime(2026, 10, 7, 12, 0)).rule_status(inside)[0]
    assert (row["inside"], row["in_force"], row["breach"]) == (True, True, True), row


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
