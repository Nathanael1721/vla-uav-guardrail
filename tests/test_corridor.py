"""The keep-IN corridor rule and the `valid_time` schedule.

Run either way:
    pytest tests/test_corridor.py -v
    python tests/test_corridor.py

WHY THIS FILE EXISTS

`corridor` and time windows are both named in the grant's DSL taxonomy and were
implemented in NEITHER repository - not here, and not in Prof. Lai's reference,
whose `policy_dsl/models.py` carries only PolygonFence and AltitudeEnvelope. So
there was no prior art to match and no existing test to extend, which is exactly
when a new rule is most likely to be subtly wrong.

Three of the tests below are regressions on defects found while writing the
rule, each of which passed a casual eye:

  * distance measured to the centerline VERTICES instead of its segments;
  * the corridor's altitude band checked but never repaired, so the Shield
    raised a violation it had no operator for and emitted a still-illegal
    action - a P0 escape manufactured by the new rule;
  * a vehicle exactly ON the centerline having no defined "way back", so a
    sideways departure got no repair and the P0 guard stopped the mission.
"""
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.geometry import nearest_on_polyline                 # noqa: E402
from guardrail.models import (Action4D, AltitudeEnvelope, Corridor,  # noqa: E402
                              KinematicEnvelope, Policy, PolygonFence,
                              Recurrence, State, ValidTime, XY)
from guardrail.shield import Shield                                # noqa: E402

# The three ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02),
# at the values they defaulted to before (repair = project_fix).
HARD_P0 = {"constraint_type": "hard", "priority": "P0", "violation_action": "project_fix"}

KIN = KinematicEnvelope(id="k", type="kinematic_envelope", **HARD_P0, speed_max_mps=4.0,
                        climb_rate_max_mps=2.0, yaw_rate_max_dps=45.0)


def _corridor(**kw):
    base = dict(id="c1", type="corridor", **HARD_P0,
                centerline=[XY(x=0, y=0), XY(x=100, y=0)],
                width_m=20.0, altitude_floor_m=5.0, altitude_ceiling_m=25.0)
    base.update(kw)
    return Corridor(**base)


def _shield(*constraints, now=None, **kw):
    return Shield(Policy(policy_id="t", constraints=list(constraints)),
                  lookahead_s=3.0, dt=0.5, now=now, **kw)


# --------------------------------------------------------------------------- #
# geometry
# --------------------------------------------------------------------------- #

def test_distance_is_to_the_segments_not_the_vertices():
    """The mistake that emptied the first populated flight, now safety-critical.

    A point 5 m off the middle of a 100 m straight is 5 m from the corridor. Its
    nearest VERTEX is 50 m away. A corridor rule built on vertex distance would
    call a vehicle 5 m off-centre "50 m out" and repair a drift that never
    happened - or, on the far side of the same error, miss a real departure.
    """
    d, nx, ny = nearest_on_polyline(50.0, 5.0, [(0.0, 0.0), (100.0, 0.0)])
    assert abs(d - 5.0) < 1e-9, d
    assert abs(nx - 50.0) < 1e-9 and abs(ny) < 1e-9, (nx, ny)


def test_distance_past_the_end_clamps_to_the_endpoint():
    d, nx, ny = nearest_on_polyline(130.0, 0.0, [(0.0, 0.0), (100.0, 0.0)])
    assert abs(d - 30.0) < 1e-9, d
    assert abs(nx - 100.0) < 1e-9, nx


def test_a_bent_corridor_measures_to_the_nearer_leg():
    pts = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]
    d, _, _ = nearest_on_polyline(95.0, 50.0, pts)
    assert abs(d - 5.0) < 1e-9, d


# --------------------------------------------------------------------------- #
# the model
# --------------------------------------------------------------------------- #

def test_a_corridor_needs_two_points_and_a_sane_band():
    for bad in (dict(centerline=[XY(x=0, y=0)]),
                dict(altitude_floor_m=30.0, altitude_ceiling_m=10.0)):
        try:
            _corridor(**bad)
        except Exception:
            continue
        raise AssertionError(f"accepted an invalid corridor: {bad}")


def test_adding_a_corridor_changes_the_policy_hash():
    """Otherwise the rule is not covered by the signed policy.

    The same argument that made SubjectStandoff a rule rather than a
    command-line flag: what is not in the hash is not in the audit trail.
    """
    a = Policy(policy_id="p", constraints=[KIN])
    b = Policy(policy_id="p", constraints=[KIN, _corridor()])
    assert a.policy_hash != b.policy_hash


# --------------------------------------------------------------------------- #
# the check
# --------------------------------------------------------------------------- #

def test_flying_along_the_corridor_is_untouched():
    sh = _shield(_corridor(), KIN)
    d = sh.filter(State(x=50, y=0, up=15), Action4D(vx=4))
    assert not d.violations and not d.repairs, d


def test_drifting_out_of_the_corridor_is_a_violation():
    sh = _shield(_corridor(), KIN)
    d = sh.filter(State(x=50, y=0, up=15), Action4D(vy=4))
    assert any(v.category == "corridor" for v in d.violations), d


def test_already_outside_but_returning_passes():
    """Trend-aware, same rule as the fence and the clearance ring.

    Raising a violation on the action that is fixing the problem is what freezes
    an aircraft in the state the policy forbids.

    No violation - but since 2026-10-19 not untouched either. 15 m outside the
    band, returning at 2 m/s is slower than CorridorReturn's floor (close the
    gap within one lookahead, capped at speed_max), which until then applied
    only when some unrelated rule had opened the repair loop. The return is
    FLOORED now, as it always was with a yaw breach beside it
    (docs/FINDING-repairs-must-floor-the-escape.md), the along-corridor motion
    kept; a return already at the cap still passes untouched.

    Since 2026-11-06 the default recovery law ("cap_all") returns at the
    speed cap (docs/DESIGN-shield-enforcement.md, "Every recovery at its
    cap"): from 6 m out at a 4 m/s cap that leaves nothing of the speed for
    the along-corridor motion, which the "cap" law keeps. Both are flown.
    """
    for law in ("cap", "cap_all"):
        sh = _shield(_corridor(), KIN, recovery_law=law)
        d = sh.filter(State(x=50, y=25, up=15), Action4D(vy=-2))
        assert not d.violations, (law, d)
        assert d.emitted.vy < -2.0, "a slow return is floored, never slowed"
        assert [(r.operator, r.recovery) for r in d.repairs] == [("CorridorReturn", True)]
        yawed = sh.filter(State(x=50, y=25, up=15), Action4D(vy=-2, yaw_rate=1.5))
        assert yawed.violations and yawed.emitted.vy == d.emitted.vy, (law, yawed)
        d = sh.filter(State(x=50, y=16, up=15), Action4D(vx=1.0, vy=-0.5))   # 6 m out
        if law == "cap":
            assert not d.violations and d.emitted.vx == 1.0 and d.emitted.vy <= -2.0 + 1e-9, d
        else:
            assert not d.violations and abs(d.emitted.vy + KIN.speed_max_mps) < 1e-9 \
                and abs(d.emitted.vx) < 1e-9, d
        d = sh.filter(State(x=50, y=25, up=15), Action4D(vy=-KIN.speed_max_mps))
        assert not d.violations and not d.repairs and d.emitted == d.raw, (law, d)


def test_the_floor_never_flies_a_projection_theta_did_not_judge():
    """REGRESSION (review of 2026-10-19). The escape floor ran every operator
    of the rules the position breaks, and the corridor's operators have
    projection branches as well as floors. Outside the corridor laterally,
    the band forecast is not checked (the return is what matters), yet
    CorridorAltitudeFix still cut a legal 1.79 m/s climb to 0.28 - a 4.55 m
    vertical projection, over theta's 0.5 m - on a tick with no violation,
    which the FSM read as clean: theta never judged it, while the same
    action beside a yaw breach went Normal -> Brake [X1] on it. A second
    tick carried a non-recovery CorridorReturn of 3.33 m that zeroed a legal
    vy. A floored decision now carries only repairs theta does not govern
    (recovery floors, clamps); otherwise the raw action flies, the trace
    says why and `n_floor_refused` counts it. Beside the yaw breach the
    projection is still made - and judged."""
    from guardrail import fsm
    for st, a, op in ((State(x=-13.44, y=34.18, up=24.17),
                       Action4D(vx=0.46, vy=-0.08, vz_up=1.79), "CorridorAltitudeFix"),
                      (State(x=12.85, y=-4.66, up=30.73),
                       Action4D(vx=-1.19, vy=-2.89, vz_up=-0.87), "CorridorReturn")):
        sh = _shield(_corridor(), KIN)
        assert sh.state_is_unsafe(st) and not sh._check(st, a), st
        d = sh.filter(st, a)
        assert not d.violations and not d.repairs and d.emitted == a, d.repairs
        assert (d.fsm_state_after, d.setpoint) == ("Normal", "pass")
        assert op in d.trace["floor_refused"] and sh.n_floor_refused == 1, d.trace
        sh.reset_episode()
        yawed = sh.filter(st, a.model_copy(update={"yaw_rate": 1.5}))
        assert op in [r.operator for r in yawed.repairs], yawed.repairs
        assert yawed.fsm_edge == "X1" and "repair over theta" in yawed.fsm_record["reason"]

    # Over a seeded scan, no tick that violates nothing carries a repair theta
    # governs, and the floors (recoveries only) still fly.
    import random
    rng = random.Random(5)
    sh = _shield(_corridor(), KIN)
    floored = 0
    for _ in range(600):
        st = State(x=rng.uniform(-40, 60), y=rng.uniform(-40, 40), up=rng.uniform(0, 35))
        a = Action4D(vx=rng.uniform(-3, 3), vy=rng.uniform(-3, 3), vz_up=rng.uniform(-1.5, 1.5))
        d = sh.filter(st, a)
        if not d.violations and d.repairs:
            floored += 1
            assert not [r.operator for r in d.repairs
                        if fsm.theta_governs(r, stop_illegal=True)], (st, a, d.repairs)
            assert not d.emitted_violations and not sh._check(st, d.emitted)
    assert floored > 100, floored


def test_already_outside_and_going_further_is_a_violation():
    sh = _shield(_corridor(), KIN)
    d = sh.filter(State(x=50, y=25, up=15), Action4D(vy=2))
    assert any(v.category == "corridor" for v in d.violations), d


def test_the_altitude_band_belongs_to_the_corridor():
    sh = _shield(_corridor(), KIN)
    assert sh.filter(State(x=50, y=0, up=2), Action4D(vx=4)).violations
    assert sh.filter(State(x=50, y=0, up=30), Action4D(vx=4, vz_up=2)).violations


# --------------------------------------------------------------------------- #
# the repair
# --------------------------------------------------------------------------- #

def test_the_repair_keeps_the_along_corridor_motion():
    """A corridor objects to sideways drift, not to progress.

    Scaling the whole horizontal vector would satisfy the rule by cancelling the
    mission. 2 m/s along must survive; 3 m/s sideways must not.
    """
    sh = _shield(_corridor(), KIN)
    d = sh.filter(State(x=50, y=8, up=15), Action4D(vx=2, vy=3))
    assert abs(d.emitted.vx - 2.0) < 1e-6, d.emitted
    assert d.emitted.vy <= 1e-6, d.emitted
    assert not d.emitted_violations, d


def test_the_corridor_altitude_band_is_actually_repaired():
    """REGRESSION: a check with no repair operator is a manufactured P0 escape.

    The first version delegated altitude to _repair_altitude, which only knows
    `AltitudeEnvelope`. With a corridor and no envelope in the policy, the
    violation was raised, no operator would act, the rescue search found nothing
    and the Shield emitted an action that STILL violated.
    """
    sh = _shield(_corridor(), KIN)
    d = sh.filter(State(x=50, y=0, up=2), Action4D(vx=4))
    assert d.emitted.vz_up > 0, f"must climb back into the band: {d.emitted}"
    assert not d.emitted_violations, (
        f"the emitted action still violates: {d.emitted_violations}")


def test_leaving_from_exactly_on_the_centerline_is_steered_not_stopped():
    """REGRESSION: no defined "way back" from distance zero.

    The first version skipped the repair when the vehicle sat exactly on the
    line, so a straight sideways departure reached the P0 guard and braked. Safe
    but wrong: the drift is steerable, and a full stop mid-mission is the
    outcome the repair operators exist to avoid.
    """
    sh = _shield(_corridor(), KIN)
    d = sh.filter(State(x=50, y=0, up=15), Action4D(vy=4))
    assert not d.braked, "a steerable drift must not end in a brake"
    assert any(r.operator == "CorridorReturn" for r in d.repairs), d.repairs
    assert not d.emitted_violations, d


def test_a_deep_recovery_is_floored_not_capped():
    """The bug found on the standoff rule: a repair acting as a CEILING.

    An action already returning faster than the repair would command must not be
    slowed down by it.
    """
    sh = _shield(_corridor(), KIN)
    d = sh.filter(State(x=50, y=25, up=15), Action4D(vy=-3.5))
    assert d.emitted.vy <= -3.5 + 1e-6, (
        f"a legal fast return was throttled to {d.emitted.vy}")


# --------------------------------------------------------------------------- #
# unsafe POSITION
# --------------------------------------------------------------------------- #

def test_being_outside_the_corridor_is_an_unsafe_position():
    sh = _shield(_corridor(), KIN)
    assert not sh.state_is_unsafe(State(x=50, y=0, up=15))
    assert sh.state_is_unsafe(State(x=50, y=25, up=15))
    assert sh.state_is_unsafe(State(x=50, y=0, up=2))


# --------------------------------------------------------------------------- #
# valid_time
# --------------------------------------------------------------------------- #

MON_0900 = datetime(2026, 9, 7, 9, 0)      # a Monday
SAT_0900 = datetime(2026, 9, 5, 9, 0)      # a Saturday
MON_2300 = datetime(2026, 9, 7, 23, 0)


def _weekday_fence():
    return PolygonFence(
        id="nfz", type="polygon_fence", **HARD_P0,
        vertices=[XY(x=-5, y=-5), XY(x=5, y=-5), XY(x=5, y=5), XY(x=-5, y=5)],
        margin_m=1.0,
        valid_time=ValidTime(recurrence=Recurrence(
            days=["Mon", "Tue", "Wed", "Thu", "Fri"],
            start_time="07:30", end_time="17:30")))


def test_a_rule_with_no_valid_time_is_always_in_force():
    sh = _shield(_weekday_fence().model_copy(update={"valid_time": None}),
                 now=lambda: SAT_0900)
    assert sh.state_is_unsafe(State(x=0, y=0, up=10))


def test_a_scheduled_rule_binds_inside_its_window():
    sh = _shield(_weekday_fence(), now=lambda: MON_0900)
    assert sh.state_is_unsafe(State(x=0, y=0, up=10))


def test_a_scheduled_rule_is_inert_outside_its_window():
    for when in (SAT_0900, MON_2300):
        sh = _shield(_weekday_fence(), now=lambda: when)
        assert not sh.state_is_unsafe(State(x=0, y=0, up=10)), when


def test_with_no_clock_every_rule_is_in_force():
    """Absent means ACTIVE, everywhere, on purpose.

    A P0 rule that switches itself off because nobody supplied a clock is the
    worst failure this feature could introduce - the Shield would report a clean
    flight while enforcing nothing.
    """
    sh = _shield(_weekday_fence(), now=None)
    assert sh.state_is_unsafe(State(x=0, y=0, up=10))


def test_gating_reaches_the_REPAIR_operators_not_only_the_checks():
    """The split this design exists to prevent.

    If only the checks were gated, an out-of-hours rule would raise no violation
    while its repair operator still bent the action away from it - the aircraft
    steering around a fence the policy says is not in force. The rule lists are
    properties so every consumer sees the same set.
    """
    st, act = State(x=-12, y=0, up=10), Action4D(vx=4)
    inside = _shield(_weekday_fence(), KIN, now=lambda: MON_0900).filter(st, act)
    outside = _shield(_weekday_fence(), KIN, now=lambda: SAT_0900).filter(st, act)
    assert inside.repairs, "in hours the fence must bend the action"
    assert not outside.repairs, (
        f"out of hours nothing should touch the action: {outside.repairs}")
    assert outside.emitted.vx == 4.0, outside.emitted


def test_a_window_may_wrap_past_midnight():
    """A night restriction is otherwise inexpressible."""
    r = Recurrence(days=["Mon"], start_time="22:00", end_time="06:00")
    assert r.active_at(datetime(2026, 9, 7, 23, 30))
    assert r.active_at(datetime(2026, 9, 7, 2, 0))
    assert not r.active_at(datetime(2026, 9, 7, 12, 0))


def test_a_malformed_time_is_rejected_at_load_not_in_flight():
    """Same rule as the rest of the DSL: schema errors raise before take-off."""
    try:
        Recurrence(days=["Mon"], start_time="not-a-time", end_time="06:00")
    except Exception:
        return
    raise AssertionError("accepted a malformed time")


def test_valid_time_is_covered_by_the_policy_hash():
    a = Policy(policy_id="p", constraints=[_weekday_fence()])
    b = Policy(policy_id="p", constraints=[
        _weekday_fence().model_copy(update={"valid_time": None})])
    assert a.policy_hash != b.policy_hash, (
        "a schedule that is not hashed is not in the audit trail")


# --------------------------------------------------------------------------- #
# coexistence
# --------------------------------------------------------------------------- #

def test_a_corridor_and_an_envelope_do_not_fight():
    """Both constrain altitude; the emitted action must satisfy both."""
    env = AltitudeEnvelope(id="alt", type="altitude_envelope", **HARD_P0,
                           alt_min_m=10.0, alt_max_m=20.0)
    sh = _shield(_corridor(), env, KIN)
    d = sh.filter(State(x=50, y=0, up=8), Action4D(vx=3))
    assert not d.emitted_violations, d.emitted_violations


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
