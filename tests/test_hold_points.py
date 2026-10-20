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
  * the harness: the opens-on-vehicle cell no longer holds inside, the
    Shield's own escape count is untouched (nothing moved out of the KPI), and
    a cell whose hold was legal flies exactly as it did.
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
from guardrail.fsm import (HOLD_ARRIVAL_SLACK_S, HOLD_REPLAN_S, EscalationFSM,  # noqa: E402
                           FSMState, HoldPoint, RuleHit, TickInput)
from guardrail.models import Action4D, DynamicNFZ, Policy, State  # noqa: E402
from guardrail.shield import Shield  # noqa: E402

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
    f.hold_at(_in_place())
    vio = (_hit(),)
    f.step(TickInput(t=0.2, outcome="blocked", violations=vio, stop_illegal=False))
    assert not f.needs_hold
    # A zone sweeping onto the hold, a window about to open over it: at once.
    f.step(TickInput(t=0.3, outcome="blocked", violations=vio, stop_illegal=True))
    assert f.needs_hold
    f.hold_at(_in_place(legal=False, breaks=("nfz",)))
    t = 0.3
    while t < 0.3 + HOLD_REPLAN_S - 0.05:
        t = round(t + 0.1, 6)
        f.step(TickInput(t=t, outcome="blocked", violations=vio, stop_illegal=True))
        if t < 0.3 + HOLD_REPLAN_S - 1e-9:
            assert not f.needs_hold, t
    f.step(TickInput(t=round(0.3 + HOLD_REPLAN_S, 6), outcome="blocked", violations=vio,
                     stop_illegal=True))
    assert f.needs_hold


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
    g.hold_at(_moved(travel=2.5))
    g.step(TickInput(t=0.5, outcome="blocked", violations=(_hit(),), stop_illegal=True,
                     hold_legal=False))
    assert g.needs_hold, "a moved point that turns illegal mid-transit is replaced"
    rec = g.step(TickInput(t=0.6, outcome="blocked", violations=(_hit(),),
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
    zone = DynamicNFZ(id="mover", type="dynamic_nfz",
                      vertices=[{"x": 40, "y": -2}, {"x": 44, "y": -2},
                                            {"x": 44, "y": 2}, {"x": 40, "y": 2}],
                      altitude_floor_m=0, altitude_ceiling_m=100, margin_m=0.5,
                      motion={"vx_mps": -2.0, "vy_mps": 0.0})
    sh = Shield(pol, escalation=False)
    sh.spawn_nfz(zone, t=0.0)
    sh.filter(State(x=-50.0, y=0.0, up=15.0), Action4D(), t=0.0)
    p = State(x=30.0, y=0.0, up=15.0)
    # The zone's near edge (40 - 0.5) reaches x = 30 at t = 4.75 s.
    with sh.looking_ahead(5.0):
        inside_then = any(r["breach"] for r in sh.rule_status(p) if r["id"] == "mover")
    inside_now = any(r["breach"] for r in sh.rule_status(p) if r["id"] == "mover")
    assert inside_then and not inside_now
    so_pol = Policy.model_validate({"policy_id": "so", "constraints": [
        {"id": "ring", "type": "subject_standoff", "constraint_type": "hard",
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
        {"id": "a-timed", "type": "polygon_fence", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "vertices": [{"x": -10, "y": -10}, {"x": 10, "y": -10}, {"x": 10, "y": 10},
                      {"x": -10, "y": 10}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 100,
         "valid_time": {"recurrence": {"days": ["Mon", "Tue", "Wed", "Thu", "Fri"],
                                       "start_time": window_start, "end_time": "17:00"}}},
        {"id": "b-always", "type": "polygon_fence", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "vertices": [{"x": -30, "y": 11}, {"x": 30, "y": 11}, {"x": 30, "y": 40},
                      {"x": -30, "y": 40}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 100}]})


def test_a_transit_never_enters_another_zone():
    pol = _two_zone_policy("08:00")
    sh = Shield(pol, now=lambda: datetime(2026, 9, 7, 7, 59, 58), escalation=False)
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


def test_no_legal_point_holds_in_place_marked_illegal():
    """A zone opening over the aircraft that reaches past the 60 m search: the
    plan says so instead of inventing a legal hold."""
    big = Policy.model_validate({"policy_id": "big", "constraints": [
        {"id": "huge", "type": "polygon_fence", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "vertices": [{"x": -200, "y": -200}, {"x": 200, "y": -200},
                      {"x": 200, "y": 200}, {"x": -200, "y": 200}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 100,
         "valid_time": {"recurrence": {"days": ["Mon"], "start_time": "08:00",
                                       "end_time": "09:00"}}}]})
    sh = Shield(big, now=lambda: datetime(2026, 9, 7, 7, 59, 58), escalation=False)
    p = HP.choose_hold_point(sh, State(x=0.0, y=0.0, up=15.0))
    assert p.kind == "in_place" and not p.legal and p.breaks == ("huge",), p
    assert "no point within" in p.reason


def test_a_landing_site_leaves_a_zone_the_descent_would_enter():
    """Hovering above a low zone is legal; landing through it is not."""
    pol = Policy.model_validate({"policy_id": "low", "constraints": [
        {"id": "low-zone", "type": "polygon_fence", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "vertices": [{"x": -5, "y": -5}, {"x": 5, "y": -5}, {"x": 5, "y": 5},
                      {"x": -5, "y": 5}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 8}]})
    sh = Shield(pol, escalation=False)
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


def test_the_shield_wires_the_plan_into_the_decision():
    pol = load_policy(SURVEY)
    t0 = datetime(2026, 9, 7, 7, 29, 58)
    clock = [0.0]
    sh = Shield(pol, now=lambda: t0 + timedelta(seconds=clock[0]))
    st = State(x=60.0, y=21.0, up=15.0)
    d0 = sh.filter(st, Action4D(vy=4.0), t=0.0)
    assert d0.fsm_edge == "X1" and d0.hold_point is None
    clock[0] = 0.1
    d1 = sh.filter(State(x=60.0, y=21.4, up=15.0), Action4D(vy=4.0), t=0.1)
    assert d1.fsm_edge == "G3" and d1.hold_point["kind"] == "moved", d1.hold_point
    assert d1.fsm_record["hold_chosen"] and d1.fsm_record["hold_point"] == d1.hold_point
    assert "hold_point" not in d1.model_dump(), "dumped decisions must not change"
    # Off: the old in-place hold, recorded as illegal.
    off = Shield(pol, now=lambda: t0 + timedelta(seconds=clock[0]), hold_points=False)
    clock[0] = 0.0
    off.filter(st, Action4D(vy=4.0), t=0.0)
    clock[0] = 0.1
    d = off.filter(State(x=60.0, y=21.4, up=15.0), Action4D(vy=4.0), t=0.1)
    assert d.hold_point["kind"] == "in_place" and not d.hold_point["legal"]


# --------------------------------------------------------------------------- #
# The harness
# --------------------------------------------------------------------------- #

def _family(sid):
    """(library, first cell) of a library family, as the sweep flies it."""
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    yield lib, lib.by_id()[sid].first


def test_the_opens_on_vehicle_cell_no_longer_holds_inside_the_zone():
    """Measured on c77c77d (the cell at 07:29:58, grant forecast): the LOITER
    held at (60, 21.8), 38 autopilot ticks flew a P0-violating action, 41
    ticks inside the polygon. Now the hold is moved out: 18 autopilot P0 ticks
    (the transit until the window opens - no action clears the forecast
    then), 3 inside. The Shield's own counter is untouched: 2 escape ticks
    before and after (no tick moved out of the KPI's population)."""
    import sweep_scenarios as SW
    lib, cell = next(_family("time-window-opens-on-vehicle"))
    run = SW.run_episode(cell.spec, lib.defaults, 0)
    ex = run.extra
    assert ex["hold_points_moved"] == 1 and ex["hold_points_illegal"] == 0, ex["hold_points"]
    assert ex["autopilot_p0_flown_ticks"] <= 20, ex["autopilot_p0_flown_ticks"]
    assert ex["ticks_inside_fence_polygon"] <= 5, ex["ticks_inside_fence_polygon"]
    assert ex["ticks_inside_fence_polygon"] >= 1, "the window must still open on it"
    shield_esc = sum(1 for r in run.rows if r.get("owner") == "shield"
                     and r.get("emitted_violations"))
    assert shield_esc == 2, shield_esc
    hp = ex["hold_points"][0]
    assert hp["kind"] == "moved" and hp["y"] > 31.0, hp
    assert any(e.get("type") == "hold_point" for e in run.events)


def test_with_the_search_off_the_cell_flies_the_old_hold_inside_the_zone():
    """Shield(hold_points=False) is the comparison arm: the hold where the
    aircraft is, recorded as illegal - and the flight c77c77d flew, number for
    number (38 autopilot P0 ticks, 41 inside; on 2026-10-19 every row of 17
    affected library cells matched the base run's flight logs this way)."""
    import sweep_scenarios as SW
    lib, cell = next(_family("time-window-opens-on-vehicle"))
    orig = SW.Shield
    try:
        SW.Shield = functools.partial(orig, hold_points=False)
        ex = SW.run_episode(cell.spec, lib.defaults, 0).extra
    finally:
        SW.Shield = orig
    assert (ex["autopilot_p0_flown_ticks"], ex["ticks_inside_fence_polygon"]) == (38, 41), ex
    assert ex["hold_points_moved"] == 0 and ex["hold_points_illegal"] >= 1, ex["hold_points"]
    assert ex["hold_points"][0]["breaks"] == ["nfz-school"]


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
