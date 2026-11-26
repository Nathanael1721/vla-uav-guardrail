"""Where the escalation FSM's Loiter holds and its Land lands (guardrail/fsm.py
HoldPoint, guardrail/holdpoint.py, Shield.looking_ahead), and the harness's
modelled autopilot flying it (experiments/sweep_scenarios.py).

Run either way:
    pytest tests/test_hold_points.py -v
    python tests/test_hold_points.py

WHY THIS FILE EXISTS

Until 2026-10-19 the FSM's hold point was wherever the aircraft happened to
be. In the library's three time-window-opens-on-vehicle cells G3 handed over
to a LOITER that ArduPilot holds at (60, 21.8), inside the school zone whose
window opened 1.0-2.0 s later; the modelled autopilot flew 28-38 P0 ticks of
LOITER per cell and the aircraft sat 31-41 ticks inside the polygon
(CHANGELOG 2026-10-13, "Not fixed"). G6 likewise "lands in place, which can
be inside a zone". These tests pin:

  * the FSM contract: every Loiter / Land entry asks for a point; an in-place
    Loiter called legal on a tick whose stop_illegal says otherwise is
    refused; a held spot that turns illegal asks again; no edge, guard or
    setpoint moved;
  * the search, on the Shield's own rules: out of a zone about to open, up
    into the altitude band, never across another zone, honest when nothing is
    legal, and a landing site off a zone's footprint over the whole descent;
  * the look-ahead view the search reads (Shield.looking_ahead): the same
    answer a Shield whose clock really is that much later gives;
  * the harness: a hold below the floor is moved up into the band, the
    Shield's own escape count is untouched (nothing moved out of the KPI), and
    a cell whose hold was legal flies exactly as it did. (The opens-on-vehicle
    cell was the harness's moved hold until 2026-10-21; since C2 it never
    leaves Normal, so it needs none: Shield._opening_imposed,
    tests/test_window_opens_on.py.)
"""
import functools
import math
import sys
from datetime import datetime, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

from guardrail import load_policy  # noqa: E402
from guardrail import holdpoint as HP  # noqa: E402
from guardrail.fsm import (HOLD_ARRIVAL_SLACK_S, HOLD_MIN_REPLAN_S,  # noqa: E402
                           HOLD_REPLAN_S, EscalationFSM, FSMState, HoldPoint, RuleHit,
                           TickInput)
from guardrail.models import Action4D, DynamicNFZ, Policy, State  # noqa: E402
from guardrail.shield import Shield  # noqa: E402
from wgs84_fixture import geo, geo_ring              # noqa: E402

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

S = FSMState
SURVEY = ROOT / "policies" / "corridor_survey.yaml"
DEMO = ROOT / "policies" / "sim_demo_policy.yaml"


def _hit(rid="nfz", action="project_fix"):
    return RuleHit(rid, action, "hard", "P0")


def _to_loiter(f: EscalationFSM, stop_illegal: bool = False, t0: float = 0.0):
    """Normal -X1-> Brake -G3-> Loiter on two blocked ticks."""
    f.step(TickInput(t=t0, outcome="blocked", violations=(_hit(),),
                     stop_illegal=stop_illegal))
    o = f.step(TickInput(t=t0 + 0.1, outcome="blocked", violations=(_hit(),),
                         stop_illegal=stop_illegal))
    assert o.state is S.LOITER and o.edge == "G3", o.reason
    return o


def _in_place(legal=True, mode="LOITER", breaks=()):
    return HoldPoint(mode, "in_place", 1.0, 2.0, 15.0, legal=legal, breaks=breaks)


def _moved(travel=2.5):
    return HoldPoint("LOITER", "moved", 1.0, 12.0, 15.0, legal=True, travel_s=travel,
                     speed_mps=4.0)


# --------------------------------------------------------------------------- #
# The FSM contract (pure)
# --------------------------------------------------------------------------- #

def test_loiter_entry_asks_for_a_hold_point_and_records_it():
    f = EscalationFSM()
    o0 = f.step(TickInput(t=0.0, outcome="blocked", violations=(_hit(),)))
    assert "hold_point" not in o0.record and not f.needs_hold     # Brake: no hold
    o = _to_loiter(EscalationFSM())
    f = EscalationFSM()
    o = _to_loiter(f)
    assert f.needs_hold and o.record["hold_point"] is None and o.record["hold_needed"]
    p = f.hold_at(_in_place())
    assert p.t_s == 0.1 and f.hold is p and not f.needs_hold
    o2 = f.step(TickInput(t=0.2, outcome="blocked", violations=(_hit(),)))
    assert o2.record["hold_point"]["kind"] == "in_place" and not o2.record["hold_needed"]
    s = f.summary()
    assert s["holds"] and s["holds"][0]["t_s"] == 0.1
    assert s["holds_moved"] == 0 and s["holds_illegal"] == 0


def test_an_in_place_loiter_called_legal_where_a_stop_is_illegal_is_refused():
    """The 13 Oct hold, refused at the FSM: standing still here breaks a hard
    rule within the lookahead (stop_illegal), so 'hold here, legal' is a lie."""
    f = EscalationFSM()
    _to_loiter(f, stop_illegal=True)
    try:
        f.hold_at(_in_place(legal=True))
    except ValueError as e:
        assert "stop_illegal" in str(e)
    else:
        raise AssertionError("an in-place hold called legal where a stop is illegal was taken")
    # The honest answers are taken: a moved point, or in place marked illegal.
    assert f.hold_at(_moved()).kind == "moved"
    g = EscalationFSM()
    _to_loiter(g, stop_illegal=True)
    assert not g.hold_at(_in_place(legal=False, breaks=("nfz",))).legal


def test_a_held_spot_that_turns_illegal_asks_again_and_an_illegal_one_once_a_second():
    f = EscalationFSM()
    _to_loiter(f)
    f.hold_at(_in_place())                          # taken at 0.1 s
    vio = (_hit(),)
    f.step(TickInput(t=0.2, outcome="blocked", violations=vio, stop_illegal=False))
    assert not f.needs_hold
    # A zone sweeping onto the hold, a window about to open over it: as soon
    # as the plan is HOLD_MIN_REPLAN_S old (0.6 s here), not on every tick.
    f.step(TickInput(t=0.3, outcome="blocked", violations=vio, stop_illegal=True))
    assert not f.needs_hold, "re-planned 0.2 s after the last plan"
    f.step(TickInput(t=round(0.1 + HOLD_MIN_REPLAN_S, 6), outcome="blocked",
                     violations=vio, stop_illegal=True))
    assert f.needs_hold
    t1 = round(0.1 + HOLD_MIN_REPLAN_S, 6)
    f.hold_at(_in_place(legal=False, breaks=("nfz",)))
    t = t1
    while t < t1 + HOLD_REPLAN_S - 0.05:
        t = round(t + 0.1, 6)
        f.step(TickInput(t=t, outcome="blocked", violations=vio, stop_illegal=True))
        if t < t1 + HOLD_REPLAN_S - 1e-9:
            assert not f.needs_hold, t
    f.step(TickInput(t=round(t1 + HOLD_REPLAN_S, 6), outcome="blocked", violations=vio,
                     stop_illegal=True))
    assert f.needs_hold


def test_a_world_that_keeps_invalidating_points_is_searched_twice_a_second():
    """Review of T1c, P6: only an illegal plan was rate-limited, so a LEGAL
    point that turned illegal asked for a new search on the very next tick.
    Where the world keeps invalidating the points (a walking subject's
    stand-off ring in the city follow, which plans holds by default) the
    search - 28-125 ms on the desktop - ran on consecutive ticks of the
    100 ms monitor loop. Now no plan is replaced sooner than
    HOLD_MIN_REPLAN_S after it was taken: here every point the integration
    picks is judged illegal on the next tick, for 10 s (the aircraft's own
    stop legal, so X6 stays out of it)."""
    f = EscalationFSM()
    _to_loiter(f, stop_illegal=True)
    plans = [f.hold_at(_moved()).t_s]
    for k in range(2, 102):
        t = round(0.1 * k, 6)
        f.step(TickInput(t=t, outcome="blocked", violations=(_hit(),), stop_illegal=False,
                         hold_legal=False))
        assert f.state is S.LOITER, (t, f.state)
        if f.needs_hold:
            plans.append(f.hold_at(_moved()).t_s)
    gaps = [round(b - a, 6) for a, b in zip(plans, plans[1:])]
    assert gaps and min(gaps) >= HOLD_MIN_REPLAN_S - 1e-9, gaps
    assert len(plans) <= 1 + 10.0 / HOLD_MIN_REPLAN_S + 1, len(plans)
    # ...and still re-planned, every HOLD_MIN_REPLAN_S: it is a floor, not a lock.
    assert len(plans) >= 10.0 / HOLD_MIN_REPLAN_S - 1, plans


def test_a_moved_hold_is_judged_after_its_arrival_not_during_the_transit():
    f = EscalationFSM()
    _to_loiter(f, stop_illegal=True)
    f.hold_at(_moved(travel=2.5))
    due = 0.1 + 2.5 + HOLD_ARRIVAL_SLACK_S
    t = 0.1
    while round(t + 0.1, 6) < due - 1e-9:
        t = round(t + 0.1, 6)
        f.step(TickInput(t=t, outcome="blocked", violations=(_hit(),), stop_illegal=True))
        assert not f.needs_hold, f"re-planned during the transit at {t}"
    f.step(TickInput(t=round(due + 0.05, 6), outcome="blocked", violations=(_hit(),),
                     stop_illegal=True))
    assert f.needs_hold, "arrived and still illegal: must choose again"


def test_the_point_is_judged_when_the_integration_reports_it_not_the_aircraft():
    """hold_legal (the POINT's legality) decides once given: an aircraft
    pushed off a legal hold (stop_illegal where it is) keeps its point - X6
    and X7 judge that by the illegal depth - and a point that turns illegal
    is replaced at once, even during the transit."""
    f = EscalationFSM()
    _to_loiter(f)
    f.hold_at(_in_place())
    for k in range(2, 40):
        f.step(TickInput(t=k * 0.1, outcome="blocked", violations=(_hit(),),
                         stop_illegal=True, hold_legal=True))
        assert not f.needs_hold, k
    f.step(TickInput(t=4.0, outcome="blocked", violations=(_hit(),),
                     stop_illegal=False, hold_legal=False))
    assert f.needs_hold
    g = EscalationFSM()
    _to_loiter(g, stop_illegal=True)
    g.hold_at(_moved(travel=2.5))                   # taken at 0.1 s
    g.step(TickInput(t=0.5, outcome="blocked", violations=(_hit(),), stop_illegal=True,
                     hold_legal=False))
    assert not g.needs_hold, "replaced sooner than HOLD_MIN_REPLAN_S after its plan"
    g.step(TickInput(t=0.6, outcome="blocked", violations=(_hit(),), stop_illegal=True,
                     hold_legal=False))
    assert g.needs_hold, "a moved point that turns illegal mid-transit is replaced"
    rec = g.step(TickInput(t=0.7, outcome="blocked", violations=(_hit(),),
                           stop_illegal=True, hold_legal=True)).record
    assert rec["hold_legal"] is True
    try:
        TickInput(t=0.0, outcome="clean", hold_legal="yes")
    except ValueError:
        pass
    else:
        raise AssertionError("hold_legal took a string")


def test_land_is_planned_once_on_entry():
    f = EscalationFSM()
    o = f.step(TickInput(t=0.0, outcome="blocked", violations=(_hit("z", "land"),)))
    assert o.state is S.LAND and f.needs_hold
    try:
        f.hold_at(_in_place(mode="LOITER"))
    except ValueError:
        pass
    else:
        raise AssertionError("a LOITER point was taken for a Land")
    f.hold_at(_in_place(mode="LAND"))
    for k in range(1, 30):
        f.step(TickInput(t=k * 0.1, outcome="clean", stop_illegal=True))
        assert not f.needs_hold
    o = f.step(TickInput(t=3.0, outcome="clean", landed=True))
    assert o.terminal and not f.needs_hold and "hold_point" not in o.record


def test_hold_point_validation_and_hold_at_outside_loiter():
    for bad in (dict(kind="moved", legal=False, travel_s=1.0, speed_mps=4.0),
                dict(kind="moved", legal=True, travel_s=0.0, speed_mps=4.0),
                dict(kind="moved", legal=True, travel_s=1.0),
                dict(kind="in_place", legal=True, breaks=("nfz",)),
                dict(kind="in_place", legal=True, travel_s=1.0),
                dict(kind="hover", legal=True)):
        try:
            HoldPoint("LOITER", x=0.0, y=0.0, up=1.0, **bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")
    try:
        HoldPoint("LOITER", "in_place", math.nan, 0.0, 1.0, legal=True)
    except ValueError:
        pass
    else:
        raise AssertionError("accepted a NaN hold point")
    f = EscalationFSM()
    try:
        f.hold_at(_in_place())
    except ValueError:
        pass
    else:
        raise AssertionError("hold_at was taken in Normal")


def test_hold_points_change_no_edge_guard_or_setpoint():
    """The same inputs, with and without hold_at: the same states, edges,
    set_mode and setpoints. The FSM's edges are the grant's; only the place
    a hold is flown moved."""
    seq = [TickInput(t=round(0.1 * k, 6), outcome="blocked", violations=(_hit(),),
                     stop_illegal=(k % 7 < 3)) for k in range(80)]
    seq += [TickInput(t=round(8.0 + 0.1 * k, 6), outcome="clean") for k in range(40)]
    a, b = EscalationFSM(), EscalationFSM()
    for inp in seq:
        oa, ob = a.step(inp), b.step(inp)
        if b.needs_hold:
            b.hold_at(_moved() if inp.stop_illegal else _in_place())
        assert (oa.state, oa.edge, oa.set_mode, oa.setpoint) == \
               (ob.state, ob.edge, ob.set_mode, ob.setpoint), inp.t
    assert a.summary()["transitions"] == b.summary()["transitions"]


# --------------------------------------------------------------------------- #
# The look-ahead view the search reads
# --------------------------------------------------------------------------- #

def test_a_look_ahead_view_answers_as_a_shield_whose_clock_is_later():
    pol = load_policy(SURVEY)
    t0 = datetime(2026, 9, 7, 7, 29, 57)
    now = Shield(pol, now=lambda: t0, escalation=False)
    p = State(x=60.0, y=21.0, up=15.0)
    for ahead in (0.5, 1.5, 2.5, 3.0, 4.0):
        later = Shield(pol, now=lambda a=ahead: t0 + timedelta(seconds=a), escalation=False)
        got = [(v.rule_id, round(v.predicted_at_s, 6)) for v in
               now.state_is_unsafe(p, ahead_s=ahead)]
        want = [(v.rule_id, round(v.predicted_at_s, 6)) for v in later.state_is_unsafe(p)]
        assert got == want, (ahead, got, want)
        rs_a = {r["id"]: (r["in_force"], r["breach"]) for r in now.rule_status(p, ahead_s=ahead)}
        rs_b = {r["id"]: (r["in_force"], r["breach"]) for r in later.rule_status(p)}
        assert rs_a == rs_b, (ahead, rs_a, rs_b)
    # 0 is exactly the call it always was.
    assert now.state_is_unsafe(p, ahead_s=0.0) == now.state_is_unsafe(p)


def test_a_look_ahead_view_ages_moving_zones_and_advances_a_moving_subject():
    pol = load_policy(DEMO)
    zone = DynamicNFZ(id="mover", type="dynamic_nfz", **HARD_P0,
                      vertices=[{"x": 40, "y": -2}, {"x": 44, "y": -2},
                                            {"x": 44, "y": 2}, {"x": 40, "y": 2}],
                      altitude_floor_m=0, altitude_ceiling_m=100, margin_m=0.5,
                      motion={"vx_mps": -2.0, "vy_mps": 0.0})
    sh = Shield(pol, escalation=False)
    sh.spawn_nfz(geo(zone, sh), t=0.0)
    sh.filter(State(x=-50.0, y=0.0, up=15.0), Action4D(), t=0.0)
    p = State(x=30.0, y=0.0, up=15.0)
    # The zone's near edge (40 - 0.5) reaches x = 30 at t = 4.75 s.
    with sh.looking_ahead(5.0):
        inside_then = any(r["breach"] for r in sh.rule_status(p) if r["id"] == "mover")
    inside_now = any(r["breach"] for r in sh.rule_status(p) if r["id"] == "mover")
    assert inside_then and not inside_now
    so_pol = Policy.model_validate({"policy_id": "so", "constraints": [
        {"id": "ring", "type": "subject_standoff", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "min_range_m": 5.0}]})
    sh2 = Shield(so_pol, escalation=False)
    sh2.set_subject(0.0, 0.0, "pedestrian", velocity=(1.0, 0.0))
    q = State(x=10.0, y=0.0, up=15.0)
    assert not sh2.hard_violations(sh2.state_is_unsafe(q))      # 10 m, closing at 1 m/s
    assert sh2.hard_violations(sh2.state_is_unsafe(q, ahead_s=3.0))   # 7 m then, 2 m by +5 s


# --------------------------------------------------------------------------- #
# The search, on the Shield's own rules
# --------------------------------------------------------------------------- #

def _survey_shield(clock: datetime, **kw) -> Shield:
    return Shield(load_policy(SURVEY), now=lambda: clock, escalation=False, **kw)


def test_out_of_a_zone_about_to_open_to_the_nearest_legal_point():
    sh = _survey_shield(datetime(2026, 9, 7, 7, 29, 58))
    st = State(x=60.0, y=21.4, up=15.0)
    p = HP.choose_hold_point(sh, st, heading=(0.0, 4.0))
    assert p.kind == "moved" and p.legal, p
    # Out of the ring (y 31) with the 0.5 m margin, refined to within 0.5 m.
    assert abs(p.x - 60.0) < 1e-6 and 31.5 <= p.y <= 32.0, (p.x, p.y)
    assert abs(p.travel_s - (p.y - st.y) / 4.0) < 1e-9 and p.speed_mps == 4.0
    for q in [State(x=p.x, y=p.y, up=p.up)] + HP._neighbours(State(x=p.x, y=p.y, up=p.up)):
        assert not sh.hard_violations(sh.state_is_unsafe(q, ahead_s=p.travel_s)), q
    assert "nfz-school" in p.reason and "4 m/s" in p.reason
    # Heading south, the tie (both ring edges 10 m off) goes south, and a
    # point exactly ON the ring (60, 11.0) is not taken.
    q = HP.choose_hold_point(sh, State(x=60.0, y=21.0, up=15.0), heading=(0.0, -4.0))
    assert q.kind == "moved" and q.y <= 10.5, q


def test_a_legal_spot_is_held_where_it_is():
    sh = _survey_shield(datetime(2026, 9, 7, 9, 0, 0))
    st = State(x=60.0, y=40.0, up=15.0)      # north of the zone, in the corridor
    p = HP.choose_hold_point(sh, st)
    assert (p.kind, p.legal, p.x, p.y, p.up) == ("in_place", True, 60.0, 40.0, 15.0), p
    off = HP.choose_hold_point(sh, State(x=60.0, y=21.0, up=15.0), search=False)
    assert off.kind == "in_place" and not off.legal and off.breaks == ("nfz-school",)


def test_a_hold_below_the_floor_climbs_into_the_band():
    sh = Shield(load_policy(DEMO), escalation=False)
    st = State(x=-20.0, y=-20.0, up=3.0)
    p = HP.choose_hold_point(sh, st)
    assert p.kind == "moved" and p.legal and 10.0 <= p.up <= 20.0, p
    assert (p.x, p.y) == (st.x, st.y), "straight up is the nearest legal hold"
    assert abs(p.travel_s - (p.up - 3.0) / 2.0) < 1e-9


def _two_zone_policy(window_start: str) -> Policy:
    """Zone A (timed) around (0, 0), 20 x 20 m; zone B, always in force, right
    against A's north side - so the nearest exit north lands in B."""
    return Policy.model_validate({"policy_id": "two-zones", "constraints": [
        {"id": "a-timed", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "vertices": [{"x": -10, "y": -10}, {"x": 10, "y": -10}, {"x": 10, "y": 10},
                      {"x": -10, "y": 10}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 100,
         "valid_time": {"recurrence": {"days": ["Mon", "Tue", "Wed", "Thu", "Fri"],
                                       "start_time": window_start, "end_time": "17:00"}}},
        {"id": "b-always", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "vertices": [{"x": -30, "y": 11}, {"x": 30, "y": 11}, {"x": 30, "y": 40},
                      {"x": -30, "y": 40}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 100}]})


def test_a_transit_never_enters_another_zone():
    pol = _two_zone_policy("08:00")
    sh = Shield(pol, now=lambda: datetime(2026, 9, 7, 7, 59, 58), escalation=False, legacy_metres=True)
    st = State(x=0.0, y=6.0, up=15.0)            # nearer A's north edge, under B
    p = HP.choose_hold_point(sh, st, heading=(0.0, 1.0))
    assert p.kind == "moved" and p.legal, p
    assert p.y < 10.0, f"went north into b-always's ring: {p}"
    assert abs(p.x) >= 11.5, f"not out of a-timed's ring: {p}"   # east or west exit
    assert not HP.transit_breaks(sh, st, State(x=p.x, y=p.y, up=p.up), p.travel_s,
                                 p.speed_mps, p.climb_mps, ("a-timed",))
    # The straight line north would breach b-always on the way: the check sees it.
    north = State(x=0.0, y=12.5, up=15.0)
    assert "b-always" in HP.transit_breaks(sh, st, north, 6.5 / 4.0, 4.0, 2.0, ("a-timed",))


def _closing_bar(half_len: float, speed: float) -> Shield:
    """Review of T1c, P2: the demo policy, the aircraft at (0, 0, 15) - legal
    now - and a 2 m-wide DynamicNFZ bar at x 3-5, `half_len` m either side
    of y = 0, closing from the east at `speed` m/s (margin 0.5)."""
    bar = DynamicNFZ(id="bar", type="dynamic_nfz", **HARD_P0,
                     vertices=[{"x": 3, "y": -half_len}, {"x": 5, "y": -half_len},
                               {"x": 5, "y": half_len}, {"x": 3, "y": half_len}],
                     altitude_floor_m=0, altitude_ceiling_m=100, margin_m=0.5,
                     motion={"vx_mps": -speed, "vy_mps": 0.0})
    sh = Shield(load_policy(DEMO), escalation=False)
    sh.spawn_nfz(geo(bar, sh), t=0.0)
    sh.filter(State(x=0.0, y=0.0, up=15.0), Action4D(), t=0.0)
    return sh


def _poses_in(sh: Shield, rule: str, start: State, p: HoldPoint, n: int = 200) -> int:
    """Poses of the plan's transit, sampled n times at their own time, that
    are inside `rule` (rule_status 'breach')."""
    end = State(x=p.x, y=p.y, up=p.up)
    inside = 0
    for i in range(1, n + 1):
        tau = p.travel_s * i / n
        q = HP._pose_at(start, end, tau, p.speed_mps, p.climb_mps)
        inside += any(r["id"] == rule and r["breach"]
                      for r in sh.rule_status(q, ahead_s=tau))
    return inside


def test_a_transit_never_crosses_a_zone_closing_on_the_aircraft():
    """Review of T1c, P2. A stop at (0, 0) is illegal within the lookahead
    (the bar reaches it at 0.83 s), so the bar was an "escaping" rule, which
    transit_breaks excused at every pose: the plan flew east THROUGH the bar
    to (3, 0) - 27 of 50 transit samples inside its ring, 0.3 s inside the
    polygon, the point itself inside the ring on arrival. Excused only while
    a stop at the start is in it too, the plan went north along the bar and
    was swept by it (23 samples). Now a rule in force that the aircraft is
    outside of is never entered: a clean escape, or none (in place, marked
    illegal, searched again in a second).

    Re-review P3: the 20 m and 60 m bars at 3 m/s were pinned "in_place",
    but a clean escape exists - (-21.2, 21.2) round the 20 m bar's end in
    7.5 s, (-52, 0) ahead of the 60 m one in 13 s - and the search stopped
    only because its budget ran out. Whether the budget reaches an escape
    that far is a question of cost, not of legality: every stop query
    against a moving zone costs about 0.2-0.3 ms, and holdpoint.MAX_QUERIES
    is sized so the fruitless search stays inside the 100 ms tick (about
    60 ms of the desktop's time; "THE SEARCH'S BUDGET").
    So for those three the test asks for a clean escape or an in-place plan
    whose reason says the budget stopped it."""
    st = State(x=0.0, y=0.0, up=15.0)
    for half_len, speed, want in ((3.0, 3.0, "moved"),       # round its end
                                  (20.0, 2.0, None),         # outrun it west, or
                                  (20.0, 3.0, None),         # out of budget
                                  (60.0, 3.0, None)):
        sh = _closing_bar(half_len, speed)
        assert sh.hard_violations(sh.state_is_unsafe(st)), "the start must be illegal"
        p = HP.choose_hold_point(sh, st)
        assert want is None or p.kind == want, (half_len, speed, p)
        if p.kind == "moved":
            assert p.legal and _poses_in(sh, "bar", st, p) == 0, (half_len, speed, p)
            assert not (p.x > 0.0 and abs(p.y) < half_len), f"crossed the bar: {p}"
        else:
            assert not p.legal and p.breaks == ("bar",), p
            assert HP.BUDGET_NOTE in p.reason, p.reason


def test_a_transit_never_closes_on_the_person_it_must_keep_clear_of():
    """A transit LEAVING a rule the aircraft is in must get no deeper into it
    than standing still would be. A pedestrian at (0, 0) with a 10 m
    stand-off, the aircraft 2 m from it, and always-on zones walling off the
    ring's north, south and east: every way out runs past the person. The
    first version (an escaping rule excused at every pose) flew to
    (-6.5, 8.5), passing 1.41 m from the pedestrian. Crossing would be
    quicker out of the ring, but it is deeper in it - closer to the person
    the rule protects - so the honest answer is no legal transit."""
    def wall(zid, x0, y0, x1, y1):
        return {"id": zid, "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
                "priority": "P0", "violation_action": "repair", "margin_m": 0.5,
                "vertices": [{"x": x0, "y": y0}, {"x": x1, "y": y0}, {"x": x1, "y": y1},
                             {"x": x0, "y": y1}],
                "altitude_floor_m": 0, "altitude_ceiling_m": 100}
    pol = Policy.model_validate({"policy_id": "walled", "constraints": [
        {"id": "ring", "type": "subject_standoff", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "min_range_m": 10.0},
        wall("north", -3, 9, 40, 40), wall("south", -3, -40, 40, -9),
        wall("east", 9, -40, 40, 40)]})
    sh = Shield(pol, escalation=False, legacy_metres=True)
    sh.set_subject(0.0, 0.0, "pedestrian")
    st = State(x=2.0, y=0.0, up=15.0)
    p = HP.choose_hold_point(sh, st, heading=(-1.0, 0.0))
    assert p.kind == "in_place" and not p.legal and p.breaks == ("ring",), p
    # The way out west, straight past the person, is refused by name...
    assert HP.transit_breaks(sh, st, State(x=-11.0, y=0.0, up=15.0), 13.0 / 4.0, 4.0, 2.0,
                             ("ring",)) == ["ring"]
    # ...and so is one into a wall the aircraft is outside of.
    assert "north" in HP.transit_breaks(sh, st, State(x=2.0, y=11.0, up=15.0), 11.0 / 4.0,
                                        4.0, 2.0, ("ring",))


def test_no_legal_point_holds_in_place_marked_illegal():
    """A zone opening over the aircraft that reaches past the 60 m search: the
    plan says so instead of inventing a legal hold."""
    big = Policy.model_validate({"policy_id": "big", "constraints": [
        {"id": "huge", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "vertices": [{"x": -200, "y": -200}, {"x": 200, "y": -200},
                      {"x": 200, "y": 200}, {"x": -200, "y": 200}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 100,
         "valid_time": {"recurrence": {"days": ["Mon"], "start_time": "08:00",
                                       "end_time": "09:00"}}}]})
    sh = Shield(big, now=lambda: datetime(2026, 9, 7, 7, 59, 58), escalation=False, legacy_metres=True)
    p = HP.choose_hold_point(sh, State(x=0.0, y=0.0, up=15.0))
    assert p.kind == "in_place" and not p.legal and p.breaks == ("huge",), p
    assert "no point within" in p.reason


def test_a_landing_site_leaves_a_zone_the_descent_would_enter():
    """Hovering above a low zone is legal; landing through it is not."""
    pol = Policy.model_validate({"policy_id": "low", "constraints": [
        {"id": "low-zone", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "vertices": [{"x": -5, "y": -5}, {"x": 5, "y": -5}, {"x": 5, "y": 5},
                      {"x": -5, "y": 5}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 8}]})
    sh = Shield(pol, escalation=False, legacy_metres=True)
    st = State(x=0.0, y=0.0, up=15.0)
    assert not sh.hard_violations(sh.state_is_unsafe(st))       # a hover is legal
    p = HP.choose_landing_site(sh, st, heading=(1.0, 0.0))
    assert p.mode == "LAND" and p.kind == "moved" and p.legal, p
    assert max(abs(p.x), abs(p.y)) >= 6.5, f"site on or inside the ring: {p}"
    assert not HP.landing_breaks(sh, p.x, p.y, 15.0, p.travel_s)
    off = HP.choose_landing_site(sh, st, search=False)
    assert off.kind == "in_place" and not off.legal and off.breaks == ("low-zone",)


def test_a_landing_names_the_floor_it_must_break_and_honours_a_map():
    sh = Shield(load_policy(DEMO), escalation=False)
    st = State(x=-60.0, y=0.0, up=15.0)
    p = HP.choose_landing_site(sh, st)
    assert (p.kind, p.legal) == ("in_place", True) and "alt-band" in p.reason, p
    # A map that refuses x < -55 moves the site east, still off the zone.
    q = HP.choose_landing_site(sh, st, site_ok=lambda x, y: x >= -55.0)
    assert q.kind == "moved" and q.x >= -55.0 and q.legal, q


def test_a_map_that_answers_ok_and_reasons_is_read_by_its_ok():
    """demo/landing.is_landable answers (ok, reasons). The first version read
    the map's answer with bool(), and bool((False, ['not_street'])) is True:
    the city map, passed as functools.partial(is_landable, maps), would have
    approved every site it refuses - the in-place site here among them."""
    sh = Shield(load_policy(DEMO), escalation=False)
    st = State(x=-60.0, y=0.0, up=15.0)
    city = (lambda x, y: (True, []) if x >= -55.0
            else (False, ["not_street: refused by the test map"]))
    q = HP.choose_landing_site(sh, st, site_ok=city)
    assert q.kind == "moved" and q.x >= -55.0 and q.legal, q
    assert "landing map refuses" in q.reason, q.reason
    # The Shield passes its land_site_ok through unchanged.
    sh2 = Shield(load_policy(DEMO), escalation=False, land_site_ok=city)
    assert sh2.land_site_ok is city


def _survey_loiter_policy():
    """corridor_survey with nfz-school's violation_action set to loiter, so a
    violation of it takes the FSM straight to Loiter (X4) and a hold is
    planned on that tick.

    Until 2026-10-21 these tests flew the pilot's 4 m/s north from the
    zone's middle at 07:29:58 (X1 at 0.0 s, G3 at 0.1 s). Since C2 that
    escape is not a violation (Shield._opening_imposed;
    tests/test_window_opens_on.py), so it never leaves Normal; a hover there
    still breaks the opening rule, and the loiter action makes the FSM ask
    for a hold at once."""
    pol = load_policy(SURVEY)
    return pol.model_copy(update={"constraints": [
        c.model_copy(update={"violation_action": "loiter"}) if c.id == "nfz-school" else c
        for c in pol.constraints]})


def test_the_shield_wires_the_plan_into_the_decision():
    pol = _survey_loiter_policy()
    t0 = datetime(2026, 9, 7, 7, 29, 58)
    clock = [0.0]
    sh = Shield(pol, now=lambda: t0 + timedelta(seconds=clock[0]))
    st = State(x=60.0, y=21.0, up=15.0)
    d0 = sh.filter(st, Action4D(), t=0.0)          # a hover where the window opens
    assert d0.fsm_edge == "X4", d0.fsm_edge
    # The plan is taken on the tick its search ends (holdpoint.py, THE PER-TICK
    # BOUND, since 2026-11-07): this one spends more than one tick's units, so
    # until then no point is held and the record says the search is running.
    d, k = d0, 0
    while d.hold_point is None:
        assert (d.fsm_record.get("hold_search") or {}).get("running"), d.fsm_record
        k += 1
        assert k < 10, "the search must end within MAX_QUERIES / TICK_QUERIES ticks"
        clock[0] = 0.1 * k
        d = sh.filter(st, Action4D(), t=0.1 * k)
    assert k >= 1 and d.fsm_record["hold_search"]["ticks"] == k + 1, d.fsm_record
    assert d.hold_point["kind"] == "moved", d.hold_point
    # GeofenceEscape leaves south (the way out of the middle's tie), and the
    # hold is the nearest legal point that way, out of the ring (y 11).
    assert d.hold_point["legal"] and d.hold_point["y"] < 11.0, d.hold_point
    assert d.fsm_record["hold_chosen"] and d.fsm_record["hold_point"] == d.hold_point
    assert "hold_point" not in d.model_dump(), "dumped decisions must not change"
    clock[0] = 0.1 * (k + 1)
    d1 = sh.filter(State(x=60.0, y=20.8, up=15.0), Action4D(), t=0.1 * (k + 1))
    assert d1.fsm_state_after == "Loiter" and d1.hold_point == d.hold_point, d1.hold_point
    # Planned within the tick (HoldPlanner(units=None)): on the entry tick, the
    # same point, as before the bound.
    whole = Shield(pol, now=lambda: t0 + timedelta(seconds=clock[0]))
    whole._holds().units = None
    clock[0] = 0.0
    w0 = whole.filter(st, Action4D(), t=0.0)
    assert w0.fsm_edge == "X4" and w0.hold_point is not None, w0.hold_point
    assert {k2: w0.hold_point[k2] for k2 in ("kind", "x", "y", "up", "legal", "travel_s")} == \
        {k2: d.hold_point[k2] for k2 in ("kind", "x", "y", "up", "legal", "travel_s")}
    # Off: the old in-place hold, recorded as illegal.
    off = Shield(pol, now=lambda: t0 + timedelta(seconds=clock[0]), hold_points=False)
    clock[0] = 0.0
    d = off.filter(st, Action4D(), t=0.0)
    assert d.hold_point["kind"] == "in_place" and not d.hold_point["legal"]


# --------------------------------------------------------------------------- #
# A State with a measured velocity (the own-velocity forecast, merged in on
# 2026-10-21: State.velocity, guardrail/response.py)
# --------------------------------------------------------------------------- #

def test_the_search_is_about_positions_whatever_velocity_the_state_carries():
    """The hold-point search judges POINTS (a stop at each, a transit between
    them); the measured velocity of the aircraft is not one of its inputs.
    choose_hold_point, choose_landing_site and transit_breaks answer the
    same for the state with and without a velocity."""
    sh = _survey_shield(datetime(2026, 9, 7, 7, 29, 58))
    pos = State(x=60.0, y=21.4, up=15.0)
    fast = pos.model_copy(update={"velocity": (0.0, 5.0, 0.0)})
    for fn in (HP.choose_hold_point, HP.choose_landing_site):
        a, b = fn(sh, pos, heading=(0.0, 4.0)), fn(sh, fast, heading=(0.0, 4.0))
        assert a == b and a.kind == "moved", (fn.__name__, a, b)
    end = State(x=60.0, y=32.0, up=15.0)
    assert HP.transit_breaks(sh, fast, end, 10.6 / 4.0, 4.0, 2.0, ("nfz-school",)) == \
        HP.transit_breaks(sh, pos, end, 10.6 / 4.0, 4.0, 2.0, ("nfz-school",)) == []


def test_with_a_measured_velocity_the_hold_agrees_with_the_fsms_own_stop_check():
    """The escalation FSM's stop_illegal is about the POSITION (fsm.py), and
    with a measured velocity the Shield asks it again without the velocity
    (_escalate), because tick.unsafe is then where a stop would END (the
    momentum the aircraft carries, Shield._committed). The hold point must be
    planned against the same answer the FSM was given: at the merge of the
    two branches (2026-10-21) _plan_hold still read tick.unsafe, and with a
    velocity the plan relocated holds the FSM called legal - 24 of 337
    seeded Loiter entries on seven policies, the hold moved while the reason
    said "standing still at (x, y, z) breaks" a rule that standing still
    there does not break.

    Seeded Loiter entries (two blocked ticks from one state) on the 14 Oct
    policy and its NFZ variant, each flown by a Shield given the velocity
    and one not: no FSM fault; an in-place hold is legal exactly when the
    FSM's stop_illegal is false, and an illegal one names what a stop at the
    position breaks; a Loiter is moved only when that stop is illegal; and
    where the position is illegal (the velocity is set aside, Shield.
    _decision_state) the plan is the one the Shield without a velocity
    chose."""
    import random

    import numpy as np
    with np.load(ROOT / "tests" / "fixtures" / "citylife_occ_flightband_6to14.npz") as z:
        smap = {"occ": z["occ"], "res": float(z["res"]), "ox": float(z["ox"]),
                "oy": float(z["oy"])}
    seen = {"chosen": 0, "in_place": 0, "moved": 0, "set_aside": 0}
    bad = []
    for name in ("follow_car_citylife.yaml", "follow_car_citylife_nfz.yaml"):
        pol = load_policy(ROOT / "policies" / name)
        with_v, without = Shield(pol, obstacle_map=smap), Shield(pol, obstacle_map=smap)
        rng = random.Random(f"hold-velocity:{name}")
        t = 0.0
        for i in range(150):
            pos = State(x=rng.uniform(-60.0, 120.0), y=rng.uniform(-55.0, 120.0),
                        up=rng.uniform(3.0, 22.0))
            vel = (rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-1.5, 1.5))
            a = Action4D(vx=rng.uniform(-6, 6), vy=rng.uniform(-6, 6),
                         vz_up=rng.uniform(-2.5, 2.5))
            out = []
            for sh, st in ((with_v, pos.model_copy(update={"velocity": vel})),
                           (without, pos)):
                sh.reset_episode()
                sh.filter(st, a, t=t)
                out.append(sh.filter(st, a, t=t + 0.1))
            t += 0.2
            d, d0 = out
            assert not d.fsm_fault, (name, i, d.fsm_fault)
            rec = d.fsm_record or {}
            if not rec.get("hold_chosen"):
                continue
            seen["chosen"] += 1
            hp, illegal = rec["hold_point"], rec["flags"]["stop_illegal"]
            here = sorted({v.rule_id for v in with_v.hard_violations(
                with_v.state_is_unsafe(pos))})
            assert bool(here) == bool(illegal), (name, i, here, illegal)
            seen[hp["kind"]] += 1
            if hp["mode"] == "LOITER":
                if hp["kind"] == "in_place" and (hp["legal"] == bool(illegal) or (
                        not hp["legal"] and list(hp["breaks"]) != here)):
                    bad.append((name, i, "in_place", hp["legal"], hp["breaks"], here))
                if hp["kind"] == "moved" and not illegal:
                    bad.append((name, i, "moved while the position is legal"))
            if here:
                seen["set_aside"] += 1
                assert (d0.fsm_record or {}).get("hold_point") == hp, (name, i)
    assert not bad, (len(bad), bad[:4])
    assert seen["chosen"] >= 40 and seen["in_place"] >= 10 and seen["moved"] >= 10, seen
    assert seen["set_aside"] >= 10, seen


# --------------------------------------------------------------------------- #
# The harness
# --------------------------------------------------------------------------- #

def _family(sid):
    """(library, first cell) of a library family, as the sweep flies it."""
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    yield lib, lib.by_id()[sid].first


def test_the_opens_on_vehicle_cell_no_longer_needs_a_hold_at_all():
    """Measured on c77c77d (the cell at 07:29:58, grant forecast): X1 then G3
    into a LOITER held at (60, 21.8), 38 autopilot ticks flew a P0-violating
    action, 41 ticks inside the polygon. From 2026-10-19 the hold was moved
    out to (60, 31.9): 18 autopilot P0 ticks of transit, 3 inside, and the
    Shield's own counter still scored 2 escapes (0.0 and 0.1 s). Since C2
    (2026-10-21; Shield._opening_imposed, tests/test_window_opens_on.py) the
    pilot's 4 m/s north is the escape from a window opening on the aircraft,
    not a violation: the FSM never leaves Normal, no hold is asked for, no
    autopilot tick is flown, and the Shield counts no escape - with the
    aircraft on the same path (3 ticks inside, the window still opening on
    it). Flown with either hold model, the flight is the same row for row."""
    import sweep_scenarios as SW
    lib, cell = next(_family("time-window-opens-on-vehicle"))
    run = SW.run_episode(cell.spec, lib.defaults, 0)
    ex = run.extra
    assert ex["hold_points"] == [] and ex["autopilot_ticks"] == 0, ex["hold_points"]
    assert ex["fsm"]["transitions"] == [], ex["fsm"]["transitions"]
    assert ex["ticks_inside_fence_polygon"] == 3, ex["ticks_inside_fence_polygon"]
    shield_esc = sum(1 for r in run.rows if r.get("owner") == "shield"
                     and r.get("emitted_violations"))
    assert shield_esc == 0, shield_esc
    here = SW.run_episode(cell.spec, lib.defaults, 0, autopilot_hold="in_place")
    assert _row_digest(here.rows) == _row_digest(run.rows)


def test_a_hold_below_the_floor_is_moved_up_into_the_band():
    """three-simultaneous-violations (the first cell, grant forecast): X1
    then G3 with the aircraft 7 m under the 10 m floor, where a stop is
    illegal at every horizontal position. The hold is moved straight up to
    10.5 m (0.5 m inside the band) and the modelled autopilot climbs there:
    19 autopilot P0 ticks where the in-place hold flew 78
    (test_with_the_search_off_the_cell_flies_the_old_hold_below_the_floor).
    The Shield's own counter is untouched: no escape either way (no tick
    moved out of the KPI's population). Since C2 this family, not
    time-window-opens-on-vehicle, is the library's moved hold."""
    import sweep_scenarios as SW
    lib, cell = next(_family("three-simultaneous-violations"))
    run = SW.run_episode(cell.spec, lib.defaults, 0)
    ex = run.extra
    assert ex["hold_points_moved"] == 1 and ex["hold_points_illegal"] == 0, ex["hold_points"]
    assert ex["autopilot_p0_flown_ticks"] <= 20, ex["autopilot_p0_flown_ticks"]
    shield_esc = sum(1 for r in run.rows if r.get("owner") == "shield"
                     and r.get("emitted_violations"))
    assert shield_esc == 0, shield_esc
    hp = ex["hold_points"][0]
    st0 = cell.spec.mission.start_pose
    assert hp["kind"] == "moved" and hp["legal"] and hp["up"] == 10.5, hp
    assert hp["up"] > st0.up and "alt-band" in next(
        e["reason"] for e in run.events if e.get("type") == "hold_point"), hp
    assert any(e.get("type") == "hold_point" for e in run.events)


def test_with_the_search_off_the_cell_flies_the_old_hold_below_the_floor():
    """Shield(hold_points=False) - the harness's `autopilot_hold="in_place"`,
    what every flight rail flies - is the comparison arm: the hold where the
    aircraft is, recorded as illegal, and the flight c77c77d flew, number for
    number (78 autopilot P0 ticks, X6 then home; every row of the affected
    library cells: test_in_place_holds_fly_c77c77ds_rows_exactly). Until
    2026-10-21 this was shown on time-window-opens-on-vehicle (38 autopilot
    P0 ticks, 41 inside), which since C2 never holds at all."""
    import sweep_scenarios as SW
    lib, cell = next(_family("three-simultaneous-violations"))
    orig = SW.Shield
    try:
        # c77c77d's recovery law too ("lookahead"; the default since
        # 2026-10-29 recovers at the climb cap).
        SW.Shield = functools.partial(orig, hold_points=False, recovery_law="lookahead")
        ex = SW.run_episode(cell.spec, lib.defaults, 0).extra
    finally:
        SW.Shield = orig
    assert ex["autopilot_hold_model"] == "in_place", ex["autopilot_hold_model"]
    assert ex == SW.run_episode(cell.spec, lib.defaults, 0, autopilot_hold="in_place",
                                recovery="lookahead").extra
    assert ex["autopilot_p0_flown_ticks"] == 78, ex["autopilot_p0_flown_ticks"]
    assert ex["hold_points_moved"] == 0 and ex["hold_points_illegal"] >= 1, ex["hold_points"]
    assert ex["hold_points"][0]["breaks"] == ["alt-band"]


def test_a_cell_whose_hold_was_legal_flies_exactly_as_before():
    """failsafe-loiter-timeout enters Loiter by its rule's own action (X4)
    with the aircraft held outside the zone: the hold is legal in place, so
    the flight is the same row for row with the search on or off."""
    import sweep_scenarios as SW
    lib, cell = next(_family("failsafe-loiter-timeout"))
    on = SW.run_episode(cell.spec, lib.defaults, 0)
    orig = SW.Shield
    try:
        SW.Shield = functools.partial(orig, hold_points=False)
        off = SW.run_episode(cell.spec, lib.defaults, 0)
    finally:
        SW.Shield = orig
    strip = (lambda rows: [{k: v for k, v in r.items() if k != "hold_point"} for r in rows])
    assert strip(on.rows) == strip(off.rows)
    assert on.extra["hold_points"] and all(h["kind"] == "in_place" and h["legal"]
                                           for h in on.extra["hold_points"])


# Row digests of the cells the hold points change (three-simultaneous-
# violations, dyn-nfz-rotate-spin, wind-beyond-authority,
# failsafe-rtl-blocked: every library cell, seed 0, grant forecast), flown IN
# PLACE: sha256 of the rows with the hold keys stripped and floats to 6
# decimals (_row_digest), first 16 hex digits, and the row count. Taken from
# c77c77d's own run_episode on 2026-10-20 (git archive c77c77d, which has no
# hold points), so `autopilot_hold="in_place"` / Shield(hold_points=False)
# must be the flight before them, row for row. The three-simultaneous cells
# fly 400 ticks since 2026-10-20 where c77c77d flew 300; in place they end at
# 7.4-9.7 s (X6, then home reached), so the rows are the same flight.
#
# The three time-window-opens-on-vehicle cells were on this list until
# 2026-10-21 (0290a3777d5d6b8c, cd11af1bb6082e8d, 3badfbc20e0b57f0, 300 rows
# each). C2 changed the SHIELD's own decisions there (the escape from a
# window opening on the aircraft is not a violation: Shield._opening_imposed),
# so no hold is asked for and the cells are no longer c77c77d's flight in
# either hold mode; test_the_opens_on_vehicle_cell_no_longer_needs_a_hold_at_all
# pins that both modes fly them identically.
#
# dyn-nfz-rotate-spin@deg_per_s=9 re-pinned 2026-11-17 (was 89cf8aa54cdee7d4,
# 164 rows): since then the Shield enforces a hot-applied zone at the
# micrometre view of its stored WGS84 (Shield._in_frame binds it to the
# frame), so the rotated ring is up to 0.6 um from the metres the rotation
# computed. The flight is c77c77d's: positions and emitted actions agree to
# 1e-14, every FSM edge at the same time. Only the repair RECORD changed on 8
# Loiter ticks (t 11.9-13.2 s, emitted 0 either way): a second GeofenceSlide
# pass that removes a 0.00 m/s into-zone component now appears or no longer
# appears beside the ClearanceEscape record. docs/data/
# wgs84_bind_default_20261117/README.md.
C77_IN_PLACE_DIGESTS = {
    "three-simultaneous-violations@ask_mps=8,start_up=3,variant=0": ("63646138a2a62da5", 98),
    "three-simultaneous-violations@ask_mps=11,start_up=3,variant=0": ("c898fcf25f59ac80", 98),
    "three-simultaneous-violations@ask_mps=8,start_up=6,variant=0": ("2eb8628ee9e671d0", 86),
    "three-simultaneous-violations@ask_mps=11,start_up=6,variant=0": ("5e42dd3b157d662c", 86),
    "three-simultaneous-violations@ask_mps=8,start_up=9,variant=0": ("70143375409a14f4", 75),
    "three-simultaneous-violations@ask_mps=11,start_up=9,variant=0": ("2e681687ac0d0c13", 75),
    "dyn-nfz-rotate-spin@deg_per_s=3": ("92460e8154bb6297", 400),
    "dyn-nfz-rotate-spin@deg_per_s=6": ("c033359d0407a456", 400),
    "dyn-nfz-rotate-spin@deg_per_s=9": ("1cac6a6572420378", 164),
    "wind-beyond-authority@lane_y=15,wind_speed_mps=8": ("8284db4b0c8f5e0d", 284),
    "wind-beyond-authority@lane_y=15,wind_speed_mps=12": ("ed8c4ebf4ab6d1bb", 500),
    "wind-beyond-authority@lane_y=20,wind_speed_mps=8": ("825e114b339a7d7e", 284),
    "wind-beyond-authority@lane_y=20,wind_speed_mps=12": ("19161a818d052d77", 412),
    "failsafe-rtl-blocked": ("7b996b76340b3e28", 351),
}


def _row_digest(rows) -> str:
    import hashlib
    import json

    def canon(v):
        if isinstance(v, float):
            return round(v, 6) + 0.0
        if isinstance(v, dict):
            return {k: canon(x) for k, x in v.items() if k != "hold_point"}
        if isinstance(v, (list, tuple)):
            return [canon(x) for x in v]
        return v
    blob = json.dumps([canon(r) for r in rows], sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]


def test_in_place_holds_fly_c77c77ds_rows_exactly():
    """Review of T1c, P8: "identical when off" rested on two counters of one
    cell. Every row of every cell the hold points change, flown in place,
    must be c77c77d's - the flight every rail still flies - so a drift in the
    off mode fails here, not in a later comparison. On c77c77d's recovery
    law ("lookahead"): since 2026-10-29 the default recovers at the climb
    cap, a change of the Shield's own (WP3-17). And on c77c77d's Loiter law
    (FSMConfig loiter_clear "violation_free", loiter_x67 "request"): since
    2026-11-02 Loiter hands back a request the Shield would fly and X6 / X7
    judge the held aircraft's position, which changes three of these cells
    (dyn-nfz-rotate-spin at 3 and 6 deg/s, wind-beyond-authority lane 15 at
    12 m/s) - a change of the FSM's own. And on c77c77d's reading of an
    untrusted repair (untrusted_owner "projected", until 2026-11-07). And
    with no X9 (hold_kept "off", until 2026-11-11): the in-place hold of
    every wind-beyond-authority cell is carried away by the excess wind, and
    X9 sends it home before X7 would."""
    import sweep_scenarios as SW
    from guardrail.fsm import FSMConfig
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    fams = lib.by_id()
    got = {}
    orig = SW.Shield
    old_law = FSMConfig(loiter_clear="violation_free", loiter_x67="request",
                        untrusted_owner="projected", hold_horizon="lookahead",
                        hold_kept="off")

    def shield_on_the_old_loiter_law(*a, escalation=True, **k):
        return orig(*a, escalation=old_law if escalation is True else escalation, **k)

    SW.Shield = shield_on_the_old_loiter_law
    try:
        for sid in sorted({k.split("@")[0] for k in C77_IN_PLACE_DIGESTS}):
            for cell in fams[sid].cells():
                run = SW.run_episode(cell.spec, lib.defaults, 0, autopilot_hold="in_place",
                                     recovery="lookahead")
                got[cell.episode_id] = (_row_digest(run.rows), len(run.rows))
    finally:
        SW.Shield = orig
    assert set(got) == set(C77_IN_PLACE_DIGESTS), sorted(set(got) ^ set(C77_IN_PLACE_DIGESTS))
    bad = {k: (got[k], v) for k, v in C77_IN_PLACE_DIGESTS.items() if got[k] != v}
    assert not bad, bad


def test_the_autopilot_hold_model_is_recorded_and_reflown():
    """Review of T1c, P3: the modelled autopilot flies moved holds that no
    flight rail flies, so every episode says which model its autopilot
    numbers come from (`autopilot_hold_model`, also in episode_start), and a
    bundle's extras re-fly the same one. (Shown on the library's moved hold:
    time-window-opens-on-vehicle until C2, three-simultaneous-violations
    since; test_a_hold_below_the_floor_is_moved_up_into_the_band.)"""
    import sweep_scenarios as SW
    lib, cell = next(_family("three-simultaneous-violations"))
    moved = SW.run_episode(cell.spec, lib.defaults, 0)
    here = SW.run_episode(cell.spec, lib.defaults, 0, autopilot_hold="in_place")
    assert moved.extra["autopilot_hold_model"] == "moved_points"
    assert here.extra["autopilot_hold_model"] == "in_place"
    assert here.events[0]["autopilot_hold_model"] == "in_place"
    assert moved.extra["autopilot_p0_flown_ticks"] < here.extra["autopilot_p0_flown_ticks"]
    ex = SW.episode_extras(cell.spec, lib.defaults, 0, here, autopilot_hold="in_place")
    assert ex["arm"]["autopilot_hold"] == "in_place"
    again = SW.rerun_from_extras(ex)
    assert _row_digest(again.rows) == _row_digest(here.rows)
    assert again.extra["autopilot_hold_model"] == "in_place"
    _, control = next(_family("nfz-head-on-control"))      # Shield off: no FSM
    assert SW.run_episode(control.spec, lib.defaults, 0).extra["autopilot_hold_model"] is None
    try:
        SW.run_episode(cell.spec, lib.defaults, 0, autopilot_hold="hover")
    except ValueError:
        pass
    else:
        raise AssertionError("an unknown hold model was flown")


def _fly_hold(SW, hold: dict, shaped, v0=(4.0, 0.0, 0.0), ticks: int = 120):
    """A moved Loiter hold flown by the harness's modelled autopilot through
    the measured ArduPilot vehicle response (the arm's VehicleResponse), as
    _episode flies it: the transit until hold_reached, then the hold. Returns
    (final x, largest x, ticks to reach)."""
    veh = SW.VehicleResponse(SW.AP_RESPONSE, v0=v0)
    x = y = 0.0
    up = 15.0
    done_at, xmax = None, 0.0
    for k in range(ticks):
        st = State(x=x, y=y, up=up)
        if done_at is None and SW.hold_reached(st, hold):
            done_at = k
        cmd = SW._autopilot("LOITER", st, (0.0, 0.0), 0.1, shaped,
                            hold=None if done_at is not None else hold)
        vx, vy, vz = veh.step((cmd.vx, cmd.vy, cmd.vz_up), 0.1)
        x, y, up = x + vx * 0.1, y + vy * 0.1, up + vz * 0.1
        xmax = max(xmax, x)
    return x, xmax, done_at


def test_on_the_response_arm_a_moved_hold_is_flown_as_a_position_target():
    """ArduCopter flies a moved hold as a GUIDED position target, closed by
    its position controller, so the transit ends stopped on the point
    (holdpoint.py "SPEED ASSUMPTION"; on the ideal vehicle the harness
    commands min(speed, distance / dt), which stops on it exactly). The
    vehicle-response arm (merged 2026-10-21) flies the RTL that way
    (_autopilot `shaped`) but at the merge still flew the moved hold's
    min(speed, distance / dt) through a vehicle that carries its speed: from
    4 m/s, a hold 10 m east and 1 m up was passed by 4.4 m and still not
    reached after 12 s (the braking distance at 4 m/s and 2.4 m/s^2 is
    3.3 m). Under the arm the hold is now the sqrt controller of the
    remaining distance, at the plan's speed and climb, as the RTL is: it is
    reached, passed by under a metre (the arm's RTL passes home by
    0.78-0.88 m from 0-4 m/s in the same loop) - the airframe's own lag and
    jerk limit, and the coast after the hold is latched (the modelled
    LOITER commands zero; "the response itself carries the vehicle into the
    hold", _autopilot). The ideal vehicle's command is unchanged."""
    import sweep_scenarios as SW
    hold = {"kind": "moved", "mode": "LOITER", "x": 10.0, "y": 0.0, "up": 16.0,
            "speed_mps": 4.0, "climb_mps": 2.0}
    st = State(x=0.0, y=0.0, up=15.0)
    assert SW._autopilot("LOITER", st, (0.0, 0.0), 0.1, hold=hold) == \
        Action4D(vx=4.0, vz_up=2.0)
    assert SW._autopilot("LOITER", st, (0.0, 0.0), 0.1, None, None, hold) == \
        SW._autopilot("LOITER", st, (0.0, 0.0), 0.1, hold=hold)
    near = SW._autopilot("LOITER", State(x=9.0, y=0.0, up=16.0), (0.0, 0.0), 0.1,
                         SW.AP_RESPONSE, hold=hold)
    assert 0.0 < near.vx < 4.0 and near.vz_up == 0.0, near
    for v0 in ((4.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 4.0, 0.0)):
        x, xmax, done_at = _fly_hold(SW, hold, SW.AP_RESPONSE, v0=v0)
        assert done_at is not None, (v0, "the arm's transit never reached the hold")
        assert xmax - 10.0 < 1.0 and abs(x - 10.0) < 1.0, (v0, x, xmax)
    # What the arm flew at the merge: the ideal vehicle's command through it.
    x, xmax, done_at = _fly_hold(SW, hold, None)
    assert done_at is None and xmax - 10.0 > 3.0, (x, xmax, done_at)


def test_a_run_whose_autopilot_holds_in_place_writes_outside_the_report_folders():
    """`--autopilot-hold in_place` flies a configuration other than the
    headline one (the moved hold points), so, like the 3 s forecast, the
    open-loop wind and the ArduPilot response, its run defaults to its own
    demo/out/cmp_<profile>_<label> folder (sweep_scenarios.run_folder),
    outside the demo/out/stress_* bundles tools/kpi_report.py reads by
    default. At the merge of the two branches (2026-10-21) comparison_tag
    knew the other three and not the hold model, so an in-place run wrote
    into stress_<profile> beside the headline bundles."""
    import tempfile

    import sweep_scenarios as SW
    assert SW.comparison_tag(["grant"], ["ideal"], SW.DEFAULT_WIND_MODEL,
                             SW.DEFAULT_AUTOPILOT_HOLD) is None
    assert SW.comparison_tag(["grant"], ["ideal"], SW.DEFAULT_WIND_MODEL,
                             "in_place") == "hold-in_place"
    assert SW.comparison_tag(["grant"], ["ideal", "ardupilot"], SW.DEFAULT_WIND_MODEL,
                             "in_place") == "response-ardupilot_hold-in_place"
    seen = []
    real_rf, real_rp = SW.run_folder, SW.results_path
    with tempfile.TemporaryDirectory() as td:
        def rf(tag, comparison=None):
            seen.append(("runs", tag, comparison))
            return Path(td) / "runs"

        def rp(out, publish, tag, comparison=None):
            seen.append(("out", tag, comparison))
            return Path(td) / "s.json"
        SW.run_folder, SW.results_path = rf, rp
        try:
            assert SW.main(["--only", "nfz-head-on-control", "--seed", "1001",
                            "--no-bundles", "--autopilot-hold", "in_place"]) == 0
        finally:
            SW.run_folder, SW.results_path = real_rf, real_rp
    assert seen == [("out", None, "hold-in_place"), ("runs", None, "hold-in_place")], seen


def test_a_bundle_from_before_the_hold_points_is_reflown_in_place():
    """Re-review of T1c, P2: every bundle written before the hold points has
    extras schema /1 and no `arm.autopilot_hold`, and its autopilot held in
    place. rerun_from_extras read the missing key as today's default
    (moved_points): the opens-on-vehicle cell re-flew with 18 autopilot P0
    ticks against the 38 flown, and tools/rerun_episode.py called a sound
    bundle "differs". Now such a bundle flies in place, row for row; a /1
    bundle that names its hold flies that one; the schema written is /3
    since the merge with the vehicle-response arm (2026-10-21; /2 on this
    branch before it), and a /2 or /3 bundle that does not say which hold
    it flew is refused rather than guessed. Since 2026-10-29 the schema is
    /4, which also names the recovery law (arm.recovery); every older
    bundle re-flies on "lookahead", the law it flew. Shown since C2 (2026-10-21) on
    three-simultaneous-violations, the library's moved hold (78 autopilot
    P0 ticks in place, 19 moved): the opens-on-vehicle cell no longer holds."""
    import copy
    import json
    import sweep_scenarios as SW
    lib, cell = next(_family("three-simultaneous-violations"))
    # Every /1-/3 bundle flew the recovery law of its day, "lookahead"
    # (arm.recovery and the /4 schema since 2026-10-29, WP3-17).
    here = SW.run_episode(cell.spec, lib.defaults, 0, autopilot_hold="in_place",
                          recovery="lookahead")
    ex = json.loads(json.dumps(SW.episode_extras(cell.spec, lib.defaults, 0, here,
                                                 autopilot_hold="in_place")))
    # /5 since 2026-11-06: arm.dr_drift_mps and the stress's fix report.
    assert ex["extras_schema"] == SW.EXTRAS_SCHEMA == "sweep-episode-extras/5"
    assert ex["arm"]["recovery"] == "lookahead", ex["arm"]
    assert ex["arm"]["dr_drift_mps"] == SW.DR_DRIFT_MPS, ex["arm"]
    ex["arm"].pop("dr_drift_mps")          # every older schema below predates it
    old = copy.deepcopy(ex)
    old["extras_schema"] = "sweep-episode-extras/1"
    del old["arm"]["autopilot_hold"], old["arm"]["recovery"]
    again = SW.rerun_from_extras(old)
    assert _row_digest(again.rows) == _row_digest(here.rows)
    assert again.extra["autopilot_hold_model"] == "in_place"
    assert again.extra["autopilot_p0_flown_ticks"] == 78, again.extra["autopilot_p0_flown_ticks"]
    named = copy.deepcopy(old)
    named["arm"]["autopilot_hold"] = "moved_points"
    assert SW.rerun_from_extras(named).extra["autopilot_hold_model"] == "moved_points"
    # This branch's own /2 bundles named their hold and no vehicle: read so.
    ours = copy.deepcopy(ex)
    ours["extras_schema"] = "sweep-episode-extras/2"
    del ours["arm"]["response"], ours["arm"]["recovery"]
    assert _row_digest(SW.rerun_from_extras(ours).rows) == _row_digest(here.rows)
    merged = copy.deepcopy(ex)
    merged["extras_schema"] = "sweep-episode-extras/3"
    del merged["arm"]["recovery"]
    assert _row_digest(SW.rerun_from_extras(merged).rows) == _row_digest(here.rows)
    # A /4 bundle that does not say which law it flew, and an older one that
    # names a law no harness of its schema wrote, are refused.
    for schema, drop in (("sweep-episode-extras/4", True), ("sweep-episode-extras/3", False)):
        odd = copy.deepcopy(ex)
        odd["extras_schema"] = schema
        if drop:
            del odd["arm"]["recovery"]
        try:
            SW.rerun_from_extras(odd)
        except ValueError as e:
            assert "recovery" in str(e), e
        else:
            raise AssertionError(f"a {schema} bundle was re-flown on a guessed recovery law")
    for schema in ("sweep-episode-extras/2", "sweep-episode-extras/3",
                   "sweep-episode-extras/4"):
        unnamed = copy.deepcopy(ex)
        unnamed["extras_schema"] = schema
        del unnamed["arm"]["autopilot_hold"]
        if schema not in ("sweep-episode-extras/4", SW.EXTRAS_SCHEMA):
            del unnamed["arm"]["recovery"]
        try:
            SW.rerun_from_extras(unnamed)
        except ValueError as e:
            assert "autopilot_hold" in str(e), e
        else:
            raise AssertionError(f"a {schema} bundle without its hold model was "
                                 f"re-flown on a guess")


def test_three_simultaneous_climbs_out_and_is_sent_home_by_the_loiter_timeout():
    """Review of T1c, P1. What is robust about the family since the hold
    points: the hold is moved straight up into the band and X6 never fires.
    What is not: the pilot keeps asking for the illegal action, the Loiter
    never clears, and 30 s after G3 the Loiter timeout (G5) sends it home -
    a fail-safe the label does not expect, which the 300-tick run hid by
    ending 0.2 s before it. The family flies 400 ticks and is pinned on its
    two labels; if G5 moved or stopped firing, this test (and the pin's
    FIXED or fail verdict) says so.

    Since 2026-11-02 the fail-safe rubric (docs/RUBRIC-failsafe.md) answers
    the question the pin asked: the pilot asks for repairs over theta for the
    whole run (E5), so the timeout's RTL is the fail-safe the label now
    expects, the outcome label is RTL_triggered, and the pin came off. The
    flight itself is unchanged and still pinned here."""
    import sweep_scenarios as SW
    from guardrail.fsm import FSMConfig
    lib, cell = next(_family("three-simultaneous-violations"))
    assert cell.spec.ticks == 400, cell.spec.ticks
    assert cell.spec.expect == "pass", cell.spec.expect
    assert (cell.spec.expected_failsafe, cell.spec.failsafe_clause,
            cell.spec.expected_outcome) == (True, "E5", "RTL_triggered")
    run = SW.run_episode(cell.spec, lib.defaults, 0)
    ex = run.extra
    tr = ex["fsm"]["transitions"]
    assert [x["edge"] for x in tr] == ["X1", "G3", "G5", "G9"], [x["edge"] for x in tr]
    t = {x["edge"]: x["t_s"] for x in tr}
    timeout = FSMConfig().loiter_timeout_s
    assert t["G5"] - t["G3"] >= timeout - 1e-6, t
    # 300 ticks end at 29.9 s: before G5, which is why that run read as a pass.
    assert t["G5"] > (300 - 1) * 0.1, t
    hp = ex["hold_points"][0]
    at = next(r for r in run.rows if abs(r["t"] - hp["t_s"]) < 1e-9)   # the G3 tick
    assert (hp["kind"], hp["legal"]) == ("moved", True), hp
    assert abs(hp["x"] - at["x"]) < 1e-5 and abs(hp["y"] - at["y"]) < 1e-5, (hp, at)
    assert abs(hp["up"] - 10.5) < 1e-6, hp                  # 0.5 m inside the band
    climb = [r for r in run.rows if r.get("autopilot") == "LOITER" and r.get("emitted_violations")]
    assert len(climb) <= 2, len(climb)
    assert ex["ended_safe"] and ex["breaches"] == 0, ex


# --------------------------------------------------------------------------- #
# The hold kept (guardrail/hold_flight.py HoldWatch; the FSM's X9, 2026-11-11)
# --------------------------------------------------------------------------- #

def test_the_hold_watch_reach_is_the_rails_stop_plus_the_slack():
    """A kept hold carries the aircraft no farther from its point than its
    distance when the point was set, plus the braking distance from its
    speed then on the rail Shield's own response model (0.45 s, 1.75
    m/s^2), plus 3 m. The reach is set once per point; a new point sets it
    from its own distance; a point given with no speed waits one tick for
    the speed two positions give; no point is where the Loiter began."""
    from guardrail.hold_flight import HOLD_KEPT_SLACK_M, HoldWatch, hold_reach
    from guardrail.response import ARDUCOPTER_GUIDED as R
    assert HOLD_KEPT_SLACK_M == 3.0
    assert abs(hold_reach(0.0, 3.67) - (3.67 * R.reaction_s + 3.67 ** 2 / (2 * R.accel_mps2)
                                        + 3.0)) < 1e-9
    # sm1111_048 at entry: 3.67 m/s (and 0.4 m from the point on the first
    # Loiter tick, so its reach was 8.9 m).
    assert abs(hold_reach(0.0, 3.67) - 8.4998) < 1e-3
    w = HoldWatch()
    assert w.update((0.0, 0.0), 0.0, 0.0, 0.0) is None
    off, reach = w.update((0.0, 0.0), 0.3, 0.0, 0.1)          # 3 m/s from two positions
    assert abs(off - 0.3) < 1e-9 and abs(reach - hold_reach(0.3, 3.0)) < 1e-9, (off, reach)
    assert w.update((0.0, 0.0), 5.0, 0.0, 0.2, speed_mps=10.0) == (5.0, reach)
    off, reach = w.update((20.0, 0.0), 5.0, 0.0, 0.3, speed_mps=1.0)
    assert off == 15.0 and abs(reach - hold_reach(15.0, 1.0)) < 1e-9, reach
    w.reset()
    assert w.update(None, 7.0, 7.0, 0.0, speed_mps=0.0) == (0.0, 3.0)
    assert w.update(None, 8.0, 7.0, 0.1, speed_mps=0.0) == (1.0, 3.0)


def test_the_hold_watch_judges_the_point_the_hold_flies_to():
    """judge() reads the GUIDED hold's own target while the companion holds
    the Loiter (HoldFlight.point), else the FSM's plan, else where the
    Loiter began - and nothing outside Loiter, where it starts over."""
    from guardrail.hold_flight import HoldFlight, HoldWatch
    f = EscalationFSM()
    w = HoldWatch()
    assert w.judge(f, None, 1.0, 1.0, 0.0, 2.0) == (None, None)
    _to_loiter(f)
    hf = HoldFlight()
    off, reach = w.judge(f, hf, 1.0, 1.0, 0.2, 2.0)           # no plan, no target
    assert off == 0.0 and abs(reach - 3.0 - 2.0 * 0.45 - 4.0 / 3.5) < 1e-9, reach
    plan = f.hold_at(HoldPoint("LOITER", "in_place", 4.0, 5.0, 15.0, legal=True))
    assert w.judge(f, hf, 1.0, 1.0, 0.3, 0.0) == (5.0, 8.0)   # the plan's point
    hf.tick(f, None, 1.0, 1.0, 15.0, 0.3, 0.1)                # the companion holds it
    assert hf.point == (plan.x, plan.y, plan.up)
    hf._target = type(hf._target)("LOITER", 1.0, 2.0, 15.0, 4.0, 2.0)
    off, reach = w.judge(f, hf, 1.0, 1.0, 0.4, 0.0)           # HoldFlight's own target
    assert (off, reach) == (1.0, 4.0), (off, reach)
    hf.force_brake("a gap in the stream")
    hf.tick(f, None, 1.0, 1.0, 15.0, 0.5, 0.1)
    assert hf.point is None and w.judge(f, hf, 1.0, 1.0, 0.5, 0.0) == (5.0, 8.0)
    f.tick(0.6, "blocked", [_hit(action="RTL")])
    assert w.judge(f, hf, 1.0, 1.0, 0.7, 0.0) == (None, None) and w.point is None


def _replay_rig_loiter(d: dict, cfg=None):
    """One of the rig's Loiters (tests/fixtures/hil_sm1111_wind_loiters.json)
    through HoldWatch and the FSM, held in Loiter by a `loiter` rule with no
    operator timeout, so X9 is the only edge that can leave it. Returns the
    first edge as (t, edge, record) or None."""
    from guardrail.fsm import FSMConfig
    from guardrail.hold_flight import HoldWatch
    rows, pts = d["rows"], d["points"]
    f = EscalationFSM(cfg or FSMConfig(loiter_timeout_s=None))
    loiter = (_hit("hold", "loiter"),)
    f.step(TickInput(t=rows[0][0] - 0.2, outcome="blocked", violations=loiter))
    w = HoldWatch()
    for t, x, y, vx, vy in rows:
        here = [p for p in pts if p[0] < t - 1e-6]
        point = (here[-1][1], here[-1][2]) if here else None
        got = w.update(point, x, y, t, None if vx is None else math.hypot(vx, vy))
        kw = {} if got is None else {"hold_off_m": got[0], "hold_reach_m": got[1]}
        o = f.step(TickInput(t=t, outcome="blocked", violations=loiter, **kw))
        if o.edge:
            return round(t, 1), o.edge, o.record
    return None


def test_the_rigs_wind_loiters_are_lost_at_12_mps_only():
    """REGRESSION (KPI-grade hil smoke sm1111, 22d434b: wind-beyond-authority
    lane 15 at 12 m/s, sm1111_048, the one missed fail-safe of the smoke).
    The rig's own Loiters, row for row, through HoldWatch and the FSM.
    12 m/s: the companion's GUIDED hold held the point (-11.8, 15.0) from
    5.8 s, the aircraft receded at 2-4 m/s, 8.9 m was its reach (0.4 m
    from the point and 3.67 m/s on the first Loiter tick), and the rig
    handed it back on the far side of nfz-square at 15.3 s (G8); X9 sends
    it home at 11.3 s, 13.6 m out and receding at 2.2 m/s - 1.8 s before its
    closest pass of the zone (2.4 m, 13.1 s). (The points are the node's
    notes, rounded to 0.1 m.)
    8 m/s (sm1111_047): the hold overshot 3.6 m and came back - never beyond
    its reach, so the Loiter runs on (the rig: G5 at 35.8 s). Within
    authority (sm1111_045, 5 m/s, lane 15): the same."""
    import json
    fx = json.loads((ROOT / "tests" / "fixtures" / "hil_sm1111_wind_loiters.json")
                    .read_text(encoding="utf-8"))["runs"]
    lost = _replay_rig_loiter(fx["sm1111_048_wind-beyond-authority"])
    assert fx["sm1111_048_wind-beyond-authority"]["rig_edge_out"] == ["G8", 15.302]
    assert lost is not None and lost[:2] == (11.3, "X9"), lost
    rec = lost[2]
    assert 13.0 < rec["hold_off_m"] < 14.0 and 8.8 < rec["hold_reach_m"] < 9.0, rec
    assert rec["x9_unkept"]["slope_mps"] > 2.0, rec["x9_unkept"]
    for run, edge in (("sm1111_047_wind-beyond-authority", "G5"),
                      ("sm1111_045_wind-within-authority", "G5")):
        assert fx[run]["rig_edge_out"][0] == edge, fx[run]["rig_edge_out"]
        assert len(fx[run]["rows"]) == 300
        assert _replay_rig_loiter(fx[run]) is None, run
    assert fx["sm1111_046_wind-within-authority"]["loiter"] is None
    # hold_kept "off": the FSM as it flew on the rig, nothing leaves the Loiter.
    from guardrail.fsm import FSMConfig
    assert _replay_rig_loiter(fx["sm1111_048_wind-beyond-authority"],
                              FSMConfig(loiter_timeout_s=None, hold_kept="off")) is None


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
