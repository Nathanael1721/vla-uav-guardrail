"""A time window that opens on the aircraft sooner than it can leave
(guardrail/shield.py Shield._opening_imposed; open item C2, 2026-10-21).

Run either way:
    pytest tests/test_window_opens_on.py -v
    python tests/test_window_opens_on.py

WHY THIS FILE EXISTS

The library's whole non-zero P0 escape headline (6 Shield-flown ticks,
0.000132-0.000133 over 175 Shield-on episodes) came from the three
time-window-opens-on-vehicle cells: the aircraft at the school zone's middle,
(60, 21), 10 m from its margin ring (2.5 s at the 4 m/s cap), the window
opening 1.0-2.0 s in. Until the opening the zone is not in force NOW, so the
check walked the forecast, found the pose at the opening inside the ring
whatever the action, and called the fastest way out a violation; the Shield
flew it anyway (4 m/s north, its best effort) and its own re-check counted 2
P0 escapes per cell. The same exit 0.1 s after the opening, with the zone in
force and the aircraft inside it, was legal by the inside rule ("currently
INSIDE zone and not escaping" is the only violation there). The fix applies
the inside rule from the opening when the opening is PROVED imposed (an ideal
vehicle at the hard caps cannot be out of the ring by then); nothing in the
KPI's definition or the FSM moved. These tests pin:

  * the escape is not a violation, a stop or a hover there is the very
    violation it was (same rule, time and words), and the decision's trace
    names the zone (`window_opens_on`);
  * the way out is read where the zone opens, so the escape floor raises a
    crawl along the way it is going instead of reversing it at the tie;
  * it is never claimed where an action could leave in time (a brute force
    over headings at the cap, on seeded random cases), with no hard cap,
    with a monitor_only cap, or for a moving zone or a measured velocity;
  * the harness cells: 0 Shield P0 escapes, no FSM edge, the rule still seen
    coming (`anticipated_ticks_nfz-school`) and the window still opening on
    the aircraft (ticks inside the polygon, time to safe never censored).

REVIEW OF 3660383 (2026-10-22). C2 must not make the KPI look better by
letting the aircraft stay longer in a zone that opened on it, and it did: a
1.5 m/s pilot's crawl became legal and was floored only to the in-force
2.0 m/s, so it took 5.1 s to safe (26 / 31 / 36 ticks inside) where 7c9c9e6,
every tick there escalated and the hold transit at the 4 m/s cap, took 2.6 s
(3 / 8 / 13) at any pilot speed. Now:

  * the escape from a window opening on the aircraft flies at the cap, the
    tangential given up first, and the Shield remembers the zone
    (`window_opened_on`) so the cap holds after the opening until the
    aircraft is out of the ring; the family sweeps `vehicle_speed_mps:
    [4, 1.5]` with upper bounds on time to safe and ticks inside, and the
    1.5 m/s cells fly 7c9c9e6's counts exactly (P1);
  * a committed stop point inside the ring is judged by the opening rule from
    there, and the trace, the anticipated count and the memory name the zone
    as the check applies it (P8);
  * the anticipated ticks counted only through the trace are their own field,
    and the sweep's headline puts ticks inside and time to safe beside the
    P0 rate (P2).

Run on 3660383's code (these tests and the family overlaid), 6 of the 13 fail
or error: the crawl, the floor, the memory, the stop point, the library cells
at 1.5 m/s (26 ticks inside) and the headline field. On a pristine copy of
7c9c9e6, 12 of 13 fail or error; the stop-and-hover test pins what must NOT
change and passes on both, as it should.

REVIEW OF b1d3aca (2026-10-22, P1). The cap was all or nothing: with another
hard zone within the cap's 20 m forecast of the exit, the cap's escape broke
it, the floor was refused, and the pilot's crawl flew unfloored - legal, so
no violation, no FSM edge and P0 escape 0 while a 0.5 m/s pilot stayed 104
ticks inside the polygon (14.4 s) and a 0.2 m/s one never got out. Now:

  * the escape is re-run slower, 0.25 m/s at a time down to the in-force
    floor, and the fastest that re-checks clean flies (`cap_refused`);
  * if none does, the crawl is the zone's violation, reported, and the FSM
    escalates (X1, then G3 into a Loiter), as 7c9c9e6 did there;
  * the pilot's own escape that reaches the other zone is slowed until it
    re-checks clean instead of being slid along that zone's edge (a slide
    whose side flipped every tick: the aircraft stood for 1.7 s);
  * the harness cell with the second zone 10, 13 and 15 m past the exit at
    1.5 and 0.5 m/s, both arms, holds bounds on ticks inside and time to
    safe and is never censored.

Run on b1d3aca's code, those 4 new tests fail and the other 13 pass.
"""
import math
import random
import sys
from datetime import datetime, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "experiments"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import load_policy                                   # noqa: E402
from guardrail.models import Action4D, Policy, State                # noqa: E402
from guardrail.shield import BRAKE, Shield                          # noqa: E402

SURVEY = ROOT / "policies" / "corridor_survey.yaml"
MON_0729_58 = datetime(2026, 9, 7, 7, 29, 58)        # a Monday, 2 s before 07:30
MID = State(x=60.0, y=21.0, up=15.0)                 # the cell's start: the zone's middle
NORTH_4 = Action4D(vy=4.0)                           # the goto pilot's command there


def _shield(now=MON_0729_58, policy=None, **kw):
    pol = policy if policy is not None else load_policy(SURVEY)
    return Shield(pol, now=lambda: now, **kw)


def _school(**over):
    z = {"id": "school", "type": "polygon_fence", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "altitude_floor_m": 0.0, "altitude_ceiling_m": 100.0,
         "vertices": [{"x": 50, "y": 12}, {"x": 70, "y": 12}, {"x": 70, "y": 30},
                      {"x": 50, "y": 30}],
         "valid_time": {"recurrence": {"days": ["Mon", "Tue", "Wed", "Thu", "Fri"],
                                       "start_time": "07:30", "end_time": "17:30"}}}
    z.update(over)
    return z


def _caps(**over):
    c = {"id": "caps", "type": "kinematic_envelope", "constraint_type": "hard",
         "priority": "P1", "violation_action": "repair", "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}
    c.update(over)
    return c


def _pol(*rules):
    return Policy.model_validate({"policy_id": "opens-on", "constraints": list(rules)})


def _north(y0):
    """An always-on hard P0 zone north of the school zone, y0..y0+6, its 1 m
    ring from y0 - 1: in reach of the cap's 5 s forecast (4 m/s x 5 s = 20 m)
    from the school zone's exit (ring at y = 31) when y0 - 1 < 51."""
    return {"id": "nfz-north", "type": "polygon_fence", "constraint_type": "hard",
            "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
            "altitude_floor_m": 0.0, "altitude_ceiling_m": 100.0,
            "vertices": [{"x": 50, "y": y0}, {"x": 70, "y": y0}, {"x": 70, "y": y0 + 6},
                         {"x": 50, "y": y0 + 6}]}


def _survey_with_north(y0):
    """policies/corridor_survey.yaml (the library cells' policy) with _north."""
    import yaml
    src = yaml.safe_load(SURVEY.read_text(encoding="utf-8"))
    src["constraints"].append(_north(y0))
    return Policy.model_validate(src)


# --------------------------------------------------------------------------- #
# The check
# --------------------------------------------------------------------------- #

def test_the_escape_from_a_window_opening_on_the_aircraft_is_not_a_violation():
    """7c9c9e6: the pilot's 4 m/s north from (60, 21) at 07:29:58 broke
    nfz-school ("predicted pos (60.0,29.0) inside NFZ at t+2.0s") and the
    emitted action re-checked with it - one P0 escape."""
    d = _shield().filter(MID, NORTH_4)
    assert d.violations == [], d.violations
    assert d.emitted_violations == [], d.emitted_violations
    assert d.emitted == NORTH_4, d.emitted
    assert d.trace["window_opens_on"] == ["nfz-school"], d.trace


def test_a_stop_or_a_hover_there_is_the_same_violation_word_for_word():
    """Only the escape stops being a violation: a stop's (the FSM's
    stop_illegal, the time-to-safe KPI's unsafe position) and a hover's
    verdict are the pose walk's, rule, time and words."""
    sh = _shield()
    unsafe = sh.state_is_unsafe(MID)
    assert [(v.rule_id, round(v.predicted_at_s, 9), v.detail) for v in unsafe] == [
        ("nfz-school", 2.0, "predicted pos (60.0,21.0) inside NFZ at t+2.0s")], unsafe
    d = _shield().filter(MID, Action4D(vx=0.3))          # a drift east, not leaving
    assert [(v.rule_id, round(v.predicted_at_s, 9), v.detail) for v in d.violations] == [
        ("nfz-school", 2.0, "predicted pos (60.6,21.0) inside NFZ at t+2.0s")], d.violations
    # Turning back south from 4 m north of the middle: the way out at the
    # opening is north (y = 25 + ... is nearer y = 30), so heading south is not
    # leaving.
    d = _shield().filter(State(x=60.0, y=25.0, up=15.0), Action4D(vy=-1.0))
    assert [v.rule_id for v in d.violations] == ["nfz-school"], d.violations


def test_the_escape_floor_reads_the_way_out_where_the_zone_opens():
    """A crawl north (0.3 m/s) from the zone's middle. The current position's
    way out is a tie (9 m to the north and the south edges) that shapely
    breaks to the SOUTH; read there, the floor reversed the crawl into 2 m/s
    south. Read at the pose where the zone opens it is north: the crawl is
    raised along its own way - to the 4 m/s cap, not the 2.0 m/s in-force
    floor, since the review of 3660383 (the next test says why) - and
    re-checks clean."""
    d = _shield().filter(MID, Action4D(vy=0.3))
    assert d.violations == [], d.violations
    assert abs(d.emitted.vx) < 1e-9 and abs(d.emitted.vy - 4.0) < 1e-9, d.emitted
    assert d.emitted_violations == []
    assert [r.operator for r in d.repairs] == ["GeofenceEscape"], d.repairs
    assert d.repairs[0].recovery is True


def test_a_crawl_out_of_a_window_opening_on_the_aircraft_is_raised_to_the_cap():
    """Review of 3660383 (P1). Until C2 the check called every action here a
    violation, so the FSM took over (X1, G3) and the modelled autopilot's
    hold transit left at the 4 m/s cap whatever the pilot asked; time to safe
    2.6 s, 3 / 8 / 13 ticks inside the polygon at 1.5, 1.0 or 0.5 m/s alike.
    C2 made any escape above 0.1 m/s legal and the floor raised it only to
    the in-force 2.0 m/s: a 1.5 m/s pilot - the StepTracker's cruise - took
    5.1 s to safe with 26 / 31 / 36 ticks inside, and the KPI read the longer
    stay as an improvement. Now the escape from a window that opens on the
    aircraft flies at the cap: a 1.5 m/s crawl north is raised to 4.0, and a
    tangential part is given up to the cap first, so the way out is flown at
    the full cap rather than SpeedClamp scaling both (1.5 north + 1.0 east
    would leave at 3.3 m/s). On 3660383 both came out at 2.0 m/s north."""
    for raw in (Action4D(vy=1.5), Action4D(vx=1.0, vy=1.5)):
        d = _shield().filter(MID, raw)
        assert d.violations == [] and d.emitted_violations == [], (raw, d.violations)
        assert abs(d.emitted.vx) < 1e-9 and abs(d.emitted.vy - 4.0) < 1e-9, (raw, d.emitted)
        assert [r.operator for r in d.repairs] == ["GeofenceEscape"], (raw, d.repairs)
        assert d.repairs[0].recovery is True and "cap" in d.repairs[0].detail, d.repairs
    # The pilot's own 4 m/s north is already the cap: passed untouched.
    d = _shield().filter(MID, NORTH_4)
    assert d.emitted == NORTH_4 and d.repairs == [], (d.emitted, d.repairs)


def test_the_cap_holds_after_the_opening_until_the_aircraft_is_out_of_the_ring():
    """After the opening the zone is in force and _opening_imposed no longer
    applies; the inside rule's in-force floor (2.0 m/s) would take over, and
    the crawl would stay inside 3 more ticks per cell than 7c9c9e6 (the
    review's prototype of the first half alone: 26 -> 6 against 3). So the
    Shield remembers the zones whose window opened on the aircraft
    (`window_opened_on` in the trace) and keeps their escape at the cap until
    the aircraft is out of the ring - and only those: a fresh Shield at the
    same pose, which never saw the window open on it, floors at 2.0 as it
    always did, and so does the same Shield once the aircraft has left and
    flies back in on its own. reset_episode() forgets."""
    clock = [MON_0729_58]
    sh = Shield(load_policy(SURVEY), now=lambda: clock[0])
    crawl = Action4D(vy=1.5)
    d = sh.filter(MID, crawl, t=0.0)
    assert abs(d.emitted.vy - 4.0) < 1e-9, d.emitted
    assert d.trace["window_opens_on"] == ["nfz-school"], d.trace
    # 2.5 s later: the window opened 0.5 s ago, the aircraft 3 m short of the
    # polygon's north edge (y = 30), the zone in force over it.
    clock[0] = MON_0729_58 + timedelta(seconds=2.5)
    inside = State(x=60.0, y=27.0, up=15.0)
    d = sh.filter(inside, crawl, t=2.5)
    assert d.trace["window_opens_on"] == [] and d.trace["window_opened_on"] == ["nfz-school"], \
        d.trace
    assert abs(d.emitted.vx) < 1e-9 and abs(d.emitted.vy - 4.0) < 1e-9, d.emitted
    assert [r.operator for r in d.repairs] == ["GeofenceEscape"], d.repairs
    fresh = Shield(load_policy(SURVEY), now=lambda: clock[0])
    df = fresh.filter(inside, crawl, t=2.5)
    assert df.trace["window_opened_on"] == [], df.trace
    assert abs(df.emitted.vy - 2.0) < 1e-9, df.emitted          # the in-force floor
    # Out of the ring (y > 31): forgotten. Back in on its own: the 2.0 floor.
    clock[0] = MON_0729_58 + timedelta(seconds=3.5)
    d = sh.filter(State(x=60.0, y=31.5, up=15.0), crawl, t=3.5)
    assert d.trace["window_opened_on"] == [] and d.emitted == crawl, (d.trace, d.emitted)
    clock[0] = MON_0729_58 + timedelta(seconds=3.6)
    d = sh.filter(State(x=60.0, y=29.0, up=15.0), crawl, t=3.6)
    assert d.trace["window_opened_on"] == [], d.trace
    assert abs(d.emitted.vy - 2.0) < 1e-9, d.emitted
    # reset_episode forgets it as it forgets the FSM.
    clock[0] = MON_0729_58
    sh2 = Shield(load_policy(SURVEY), now=lambda: clock[0])
    sh2.filter(MID, crawl, t=0.0)
    sh2.reset_episode()
    clock[0] = MON_0729_58 + timedelta(seconds=2.5)
    d = sh2.filter(inside, crawl, t=0.0)
    assert d.trace["window_opened_on"] == [] and abs(d.emitted.vy - 2.0) < 1e-9, \
        (d.trace, d.emitted)


def test_a_refused_cap_escape_flies_the_fastest_speed_that_re_checks_clean():
    """Review of b1d3aca (P1). The cap's escape forecasts 20 m ahead; with
    another hard zone in that reach the floored action broke it, the floor
    was refused, and the crawl flew unfloored ("raw flown: the floored action
    breaks nfz-north"), legal by the inside rule: no violation, FSM Normal,
    the KPI's 0. One tick of it: the Shield that remembered the opening
    emitted the 0.2 m/s crawl at (60, 27) where a fresh Shield emitted the
    2.0 floor. Now the run is made again slower, 0.25 m/s at a time, and the
    fastest that re-checks clean flies: 3.25 m/s at (60, 27) with the zone's
    ring at y = 44 (27 + 3.5 x 5 = 44.5 would be inside it), 3.5 m/s from the
    middle before the opening with the ring at y = 39; the trace says what
    the cap broke. A fresh Shield, which never saw the window open, still
    floors at the in-force 2.0. On b1d3aca: 0.2 and 0.2."""
    crawl = Action4D(vy=0.2)
    clock = [MON_0729_58]
    sh = Shield(_survey_with_north(45.0), now=lambda: clock[0])
    d = sh.filter(MID, crawl, t=0.0)
    assert abs(d.emitted.vy - 4.0) < 1e-9 and "cap_refused" not in d.trace, (d.emitted, d.trace)
    clock[0] = MON_0729_58 + timedelta(seconds=2.5)
    inside = State(x=60.0, y=27.0, up=15.0)
    d = sh.filter(inside, crawl, t=2.5)
    assert d.trace["window_opened_on"] == ["nfz-school"], d.trace
    assert d.violations == [] and d.emitted_violations == [], (d.violations, d.emitted_violations)
    assert abs(d.emitted.vx) < 1e-9 and abs(d.emitted.vy - 3.25) < 1e-9, d.emitted
    assert d.trace["cap_refused"] == ("flown at 3.25 m/s; at the cap the floored action "
                                      "breaks nfz-north"), d.trace
    assert "floor_refused" not in d.trace and sh.n_floor_refused == 0, d.trace
    assert [r.operator for r in d.repairs] == ["GeofenceEscape"], d.repairs
    assert d.repairs[0].recovery is True and "under the 4.0 m/s cap" in d.repairs[0].detail
    assert d.fsm_state_after == "Normal", d.fsm_state_after
    fresh = Shield(_survey_with_north(45.0), now=lambda: clock[0])
    assert abs(fresh.filter(inside, crawl, t=2.5).emitted.vy - 2.0) < 1e-9
    # Before the opening, the zone's ring 18 m north of the middle: 4 m/s
    # reaches y = 41, 3.75 m/s y = 39.75; 3.5 m/s stops short at 38.5.
    d = _shield(policy=_survey_with_north(40.0)).filter(MID, crawl)
    assert d.trace["window_opens_on"] == ["nfz-school"], d.trace
    assert d.violations == [] and abs(d.emitted.vy - 3.5) < 1e-9, (d.violations, d.emitted)
    assert d.trace["cap_refused"].startswith("flown at 3.50 m/s"), d.trace


def test_with_no_floored_escape_left_the_crawl_is_reported_and_the_fsm_escalates():
    """Review of b1d3aca (P1). When not even the in-force floor (2.0 m/s,
    straight or with the tangential kept) re-checks clean, the crawl must not
    pass as a clean tick: it is the zone's violation ("under the 2.0 m/s
    escape floor"), in `violations` and in `emitted_violations`, so the FSM
    escalates - Normal -> Brake [X1], the stop withheld because a stop there
    is illegal, the Shield's own action (the crawl, which breaks no other
    rule) flown - as 7c9c9e6 escalated every tick there, and the KPI counts
    it as the P0 escape it is. At (60, 29.5), 1.5 m from the zone's ring,
    the other zone's ring at y = 39: 2.0 m/s for 5 s reaches 39.5. A pilot
    already leaving at the floor is the floor: it flies, nothing reported.
    On b1d3aca the crawl flew with no violation and the FSM stayed Normal."""
    crawl = Action4D(vy=0.2)
    clock = [MON_0729_58]
    sh = Shield(_survey_with_north(40.0), now=lambda: clock[0])
    sh.filter(MID, crawl, t=0.0)
    clock[0] = MON_0729_58 + timedelta(seconds=2.5)
    d = sh.filter(State(x=60.0, y=29.5, up=15.0), crawl, t=2.5)
    assert d.trace["window_opened_on"] == ["nfz-school"], d.trace
    assert [v.rule_id for v in d.violations] == ["nfz-school"], d.violations
    assert "under the 2.0 m/s escape floor" in d.violations[0].detail, d.violations
    assert [v.rule_id for v in d.emitted_violations] == ["nfz-school"], d.emitted_violations
    assert d.emitted == crawl, d.emitted
    assert (d.fsm_state_after, d.fsm_edge) == ("Brake", "X1"), (d.fsm_state_after, d.fsm_edge)
    assert d.command == crawl, d.command                  # the stop is withheld
    assert d.trace["floor_refused"] == ("raw flown, reported to the FSM: the floored "
                                        "action breaks nfz-north"), d.trace
    assert sh.n_floor_refused == 1
    sh = Shield(_survey_with_north(40.0), now=lambda: clock[0])
    clock[0] = MON_0729_58
    sh.filter(MID, crawl, t=0.0)
    clock[0] = MON_0729_58 + timedelta(seconds=2.5)
    at_floor = Action4D(vy=2.0)
    d = sh.filter(State(x=60.0, y=28.5, up=15.0), at_floor, t=2.5)
    assert d.violations == [] and d.emitted == at_floor, (d.violations, d.emitted)
    assert d.fsm_state_after == "Normal", d.fsm_state_after


def test_escaping_towards_another_zone_slows_down_instead_of_sliding_off():
    """Review of b1d3aca (P1, the pilot's own escape). The goto pilot's 4 m/s
    north from the middle, the other zone's ring 18 m north: the forecast
    reaches it, so nfz-north is violated, and the chain's slide cancelled
    the whole northward part and pushed 4 m/s along that zone's edge - east
    or west by the side of the pilot's tiny lateral correction, which
    flipped every tick: in the harness the aircraft stood for 1.7 s and left
    south, 20 ticks inside the polygon and 4.3 s to safe (3 / 2.6 s on
    7c9c9e6). Now the escape it was flying is slowed until it re-checks
    clean - 3.5 m/s north, its lateral part kept on its own side - a
    recovery the FSM reads as repaired (Normal -> Brake [G1]). On b1d3aca:
    4 m/s east and 4 m/s west."""
    pol = _survey_with_north(40.0)
    for vx in (0.05, -0.05):
        raw = Action4D(vx=vx, vy=math.sqrt(16.0 - vx * vx))
        d = _shield(policy=pol).filter(MID, raw)
        assert [v.rule_id for v in d.violations] == ["nfz-north"], (vx, d.violations)
        assert d.emitted_violations == [], (vx, d.emitted_violations)
        assert abs(d.emitted.vy - 3.5) < 1e-9 and abs(d.emitted.vx - vx) < 1e-9, (vx, d.emitted)
        assert [r.operator for r in d.repairs] == ["GeofenceEscape"], (vx, d.repairs)
        assert d.repairs[0].recovery is True and "nfz-north" in d.repairs[0].detail, d.repairs
        assert (d.fsm_state_after, d.fsm_edge) == ("Brake", "G1"), (vx, d.fsm_edge)


# --------------------------------------------------------------------------- #
# Never claimed where it is not proved
# --------------------------------------------------------------------------- #

def test_not_claimed_when_an_action_could_still_leave_in_time():
    """At 07:29:57 the window opens 3.0 s in and the ring is 2.5 s away: the
    old pose walk decides. 4 m/s north is out by the opening (clean), 2 m/s
    north is not (the violation the walk always gave)."""
    early = datetime(2026, 9, 7, 7, 29, 57)
    d = _shield(early).filter(MID, NORTH_4)
    assert d.violations == [] and d.trace["window_opens_on"] == [], (d.violations, d.trace)
    d = _shield(early).filter(MID, Action4D(vy=2.0))
    assert [(v.rule_id, round(v.predicted_at_s, 9)) for v in d.violations] == [
        ("nfz-school", 3.0)], d.violations
    assert d.trace["window_opens_on"] == []


def test_not_claimed_without_a_hard_enforced_cap_that_holds_over_the_tick():
    """No cap, a soft cap, a monitor_only cap and a cap with a window of its
    own prove nothing: the ideal vehicle has no speed to be measured against,
    or the Shield may not hold it to one. The old judgement stands."""
    for caps in ((), (_caps(constraint_type="soft"),),
                 (_caps(violation_action="monitor_only"),),
                 (_caps(valid_time={"recurrence": {"start_time": "07:00",
                                                   "end_time": "07:30"}}),)):
        sh = _shield(policy=_pol(_school(), *caps))
        d = sh.filter(MID, NORTH_4)
        assert [v.rule_id for v in d.violations if v.rule_id == "school"] == ["school"], \
            (caps, d.violations)
        assert d.trace["window_opens_on"] == [], (caps, d.trace)
    sh = _shield(policy=_pol(_school(), _caps()))
    assert sh.filter(MID, NORTH_4).violations == []


def test_a_measured_velocity_is_set_aside_there_as_for_any_illegal_position():
    """An imposed opening makes a stop at the position illegal (the zone
    comes into force over it within the lookahead), and the Shield sets a
    measured velocity aside wherever that is so: for the decision
    (Shield._decision_state, counted in n_velocity_set_aside) and per rule
    in a direct check (Shield._rule_state judges a rule the position breaks
    from the position without its velocity). So with a velocity the zone is
    judged from the commanded forecast, exactly as without one, and the
    escape passes there too. _opening_imposed's own guard against a state
    that still carries a velocity (two forecast paths, which it does not
    model) is therefore never the one that decides here."""
    sh = _shield(policy=_pol(_school(), _caps()))
    st = State(x=60.0, y=21.0, up=15.0, velocity=(0.0, 4.0, 0.0))
    d = sh.filter(st, NORTH_4)
    assert sh.n_velocity_set_aside == 1, sh.n_velocity_set_aside
    assert d.violations == [] and d.trace["window_opens_on"] == ["school"], \
        (d.violations, d.trace)
    sh = _shield(policy=_pol(_school(), _caps()))
    assert sh._check(st, NORTH_4) == [] and sh._check(MID, NORTH_4) == []
    hover = sh._check(st, BRAKE)
    assert [v.rule_id for v in hover] == ["school"], hover
    # The guard itself: a state that still carries its velocity is not proved.
    with sh._scope(sh._new_tick(None)):
        recs, _ = sh._fence_anchors(sh._ir)
        assert sh._opening_imposed(recs[0].rule, recs[0].buffered, st) is None
        assert round(sh._opening_imposed(recs[0].rule, recs[0].buffered, MID), 9) == 2.0


def test_a_stop_point_inside_the_ring_is_judged_by_the_opening_rule_from_there():
    """Review of 3660383 (P8). _check_fence judges a zone from _rule_state's
    state, and for a zone the aircraft's momentum carries it into that is the
    GUIDED stop point (_committed), with no velocity - so _opening_imposed is
    reached there although the aircraft itself still carries one. The case:
    3 m short of the school zone's ring (y = 11) at 4 m/s north, the window
    opening 0.3 s ahead; the stop ends at (60, 14.4), 3.4 m inside the ring,
    which an ideal vehicle at the 4 m/s cap cannot leave in 0.3 s. What the
    Shield must do, and does: judge it as _committed judges a zone in force
    now - as if the aircraft were where the stop ends. Carrying on north and
    a stop are the pose walk's violation from the stop point (rule, time and
    words), repaired straight back out at the cap; reversing at the cap
    passes untouched, and a 1 m/s reverse is raised to it. The trace, the
    harness's anticipated count and the cap's memory name the zone, as the
    check applies the rule (on 3660383 the trace was empty here, because
    openings_imposed asked the state with its velocity, and the 1 m/s
    reverse left at 2.0). A stop point the ideal vehicle could still leave
    by the opening (2.4 m in 0.6 s from 2.37 m deep) is not claimed: the pose
    walk judges it from there as before."""
    now = datetime(2026, 9, 7, 7, 30) - timedelta(seconds=0.3)
    st = State(x=60.0, y=8.0, up=15.0, velocity=(0.0, 4.0, 0.0))
    sh = _shield(now)
    with sh._scope(sh._new_tick(None)):
        recs, _ = sh._fence_anchors(sh._ir)
        rec = next(r for r in recs if r.rule.id == "nfz-school")
        stop = sh._rule_state(rec.rule, st)
        assert stop.velocity is None and abs(stop.y - 14.37) < 0.01, stop
        assert sh._opening_imposed(rec.rule, rec.buffered, st) is None     # own guard
        assert round(sh._opening_imposed(rec.rule, rec.buffered, stop), 9) == 0.3
    for raw, words in ((NORTH_4, "predicted pos (60.0,15.6) inside NFZ at t+0.3s"),
                       (BRAKE, "predicted pos (60.0,14.4) inside NFZ at t+0.3s")):
        sh = _shield(now)
        d = sh.filter(st, raw, t=0.0)
        assert [(v.rule_id, round(v.predicted_at_s, 9), v.detail) for v in d.violations] == [
            ("nfz-school", 0.3, words)], d.violations
        assert abs(d.emitted.vx) < 1e-9 and abs(d.emitted.vy + 4.0) < 1e-9, d.emitted
        assert d.emitted_violations == [] and d.fsm_state_after == "Brake", d
        assert d.trace["window_opens_on"] == ["nfz-school"], d.trace
        assert sh._opened_on == {"nfz-school"}, sh._opened_on
    sh = _shield(now)
    d = sh.filter(st, Action4D(vy=-4.0), t=0.0)
    assert d.violations == [] and d.repairs == [] and d.fsm_state_after == "Normal", d
    sh = _shield(now)
    d = sh.filter(st, Action4D(vy=-1.0), t=0.0)
    assert d.violations == [] and abs(d.emitted.vy + 4.0) < 1e-9, (d.violations, d.emitted)
    assert [r.operator for r in d.repairs] == ["GeofenceEscape"], d.repairs
    # Not claimed: 1 m further back, 0.6 s ahead, the stop point is 2.37 m deep.
    early = datetime(2026, 9, 7, 7, 30) - timedelta(seconds=0.6)
    back = State(x=60.0, y=7.0, up=15.0, velocity=(0.0, 4.0, 0.0))
    sh = _shield(early)
    assert sh.openings_imposed(back) == []
    assert sh.filter(back, Action4D(vy=-4.0), t=0.0).violations == []
    d = _shield(early).filter(back, Action4D(vy=-1.0), t=0.0)
    assert [(v.rule_id, round(v.predicted_at_s, 9)) for v in d.violations] == [
        ("nfz-school", 0.6)], d.violations


def test_not_claimed_for_a_moving_zone():
    """A moving zone's escape is judged relative to the zone, which
    _opening_imposed does not model: the old judgement stands."""
    from guardrail.ir import is_moving
    moving = _school(type="dynamic_nfz", motion={"vx_mps": 0.5})
    pol = _pol(moving, _caps())
    assert is_moving(pol.constraints[0]), pol.constraints[0]
    d = _shield(policy=pol).filter(MID, NORTH_4)
    assert d.trace["window_opens_on"] == [], d.trace


def test_imposed_only_where_no_heading_at_the_cap_is_out_by_the_opening():
    """Soundness, by brute force: 300 seeded random positions in and around
    the ring, openings 0.2-4.8 s ahead. Wherever the Shield calls the opening
    imposed, no straight flight at the 4 m/s cap (720 headings, and straight
    up and down at the 2 m/s climb cap) ends outside the ring or band at the
    opening; and wherever the zone is not yet in force and an exit exists
    with 0.3 m to spare, it never calls it imposed."""
    from shapely.geometry import Point, Polygon
    ring = Polygon([(50, 12), (70, 12), (70, 30), (50, 30)]).buffer(1.0)
    rng = random.Random(20261021)
    pol = _pol(_school(), _caps())
    claimed = exits_seen = 0
    for _ in range(300):
        x, y = rng.uniform(47.0, 73.0), rng.uniform(9.0, 33.0)
        up = rng.choice([15.0, 15.0, 1.5, 98.5])
        lead = rng.uniform(0.2, 4.8)
        now = datetime(2026, 9, 7, 7, 30) - timedelta(seconds=lead)
        sh = _shield(now, policy=pol)
        st = State(x=x, y=y, up=up)
        imposed = sh.openings_imposed(st) == ["school"]
        if not imposed:
            # Nothing to prove; but an exit with room to spare must not be
            # called imposed (checked below via the same brute force).
            pass
        grid = [t for t in sh._grid()]
        t_open = next((t for t in grid if t >= lead - sh.dt / 2.0 - 1e-9), None)
        if t_open is None:
            assert not imposed
            continue
        inside_now = ring.contains(Point(x, y)) and 0.0 <= up <= 100.0
        best = max(ring.boundary.distance(Point(x, y)) if inside_now else 0.0, 0.0)
        can_leave_h = any(not ring.contains(Point(x + 4.0 * t_open * math.cos(a),
                                                  y + 4.0 * t_open * math.sin(a)))
                          for a in (k * math.pi / 360.0 for k in range(720)))
        can_leave_v = (up + 2.0 * t_open > 100.0) or (up - 2.0 * t_open < 0.0)
        if imposed:
            claimed += 1
            assert inside_now, (x, y, up, lead)
            assert not can_leave_h and not can_leave_v, (x, y, up, lead, t_open, best)
        elif inside_now and not (can_leave_h or can_leave_v):
            # Not claimed although no exit: allowed only within the eps band
            # of the opening (the check's own first pose) - never a silent
            # widening the other way.
            assert best - 4.0 * t_open < 0.5, (x, y, up, lead, best, t_open)
        if inside_now and 4.0 * t_open > best + 0.3:
            exits_seen += 1
            assert not imposed, (x, y, up, lead)
    assert claimed >= 20 and exits_seen >= 20, (claimed, exits_seen)


# --------------------------------------------------------------------------- #
# The harness cells
# --------------------------------------------------------------------------- #

def test_the_library_cells_fly_no_p0_escape_and_still_see_the_rule_coming():
    """7c9c9e6: 2 Shield escapes per cell (0.0 and 0.1 s), X1 then G3 into a
    moved LOITER, 8-18 autopilot P0 ticks of transit. Now: no Shield escape,
    the FSM stays Normal (no autopilot tick at all), the rule is counted as
    anticipated on every tick before it opens, the window still opens on the
    aircraft (ticks inside the polygon) and it gets out (goal reached, safe at
    the end), on both forecast arms. The anticipated and inside counts are
    the ones 7c9c9e6 flew (the same aircraft path to the decimetre: the
    pilot's 4 m/s north then, the Shield's best effort and the hold transit
    at the same 4 m/s north before); the 3 s arm's half-step eps (0.25 s)
    calls the zone in force two ticks earlier.

    And at the pilot speed the family sweeps beside the goto pilot's 4 m/s
    (review of 3660383, P1: `vehicle_speed_mps: [4.0, 1.5]`, the 1.5 m/s
    StepTracker cruise): the same counts and the same time to safe, 2.6 s,
    as 7c9c9e6 flew at either speed. On 3660383 the 1.5 m/s cells stayed
    inside the polygon 26 / 31 / 36 ticks (28 / 33 / 38 on the 3 s arm) and
    took 5.1 s to safe, with P0 escape 0 and no FSM edge."""
    import sweep_scenarios as SW
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    fam = lib.by_id()["time-window-opens-on-vehicle"]
    want = {("2026-09-07T07:29:58", "grant"): (20, 3), ("2026-09-07T07:29:58", "3s"): (18, 5),
            ("2026-09-07T07:29:58.5", "grant"): (15, 8), ("2026-09-07T07:29:58.5", "3s"): (13, 10),
            ("2026-09-07T07:29:59", "grant"): (10, 13), ("2026-09-07T07:29:59", "3s"): (8, 15)}
    seen = set()
    for cell in fam.cells():
        clock, speed = cell.params["clock"], cell.params["vehicle_speed_mps"]
        for arm in SW.FORECAST_ARMS:
            run = SW.run_episode(cell.spec, lib.defaults, 0, forecast=arm)
            ex = run.extra
            esc = sum(1 for r in run.rows if r.get("owner") == "shield"
                      and r.get("emitted_violations"))
            key = (clock, speed, arm)
            assert esc == 0, (key, esc)
            assert ex["autopilot_ticks"] == 0, (key, ex["autopilot_ticks"])
            assert all(r.get("fsm_state_after") == "Normal" for r in run.rows), key
            got = (ex["anticipated_ticks_nfz-school"], ex["ticks_inside_fence_polygon"])
            assert got == want[(clock, arm)], (key, got)
            res = SW.score(cell.spec, run.rows, ex, run.policy, run.priorities)
            assert (res["time_to_safe_episodes"], res["time_to_safe_censored"],
                    res["max_time_to_safe_s"]) == (1, 0, 2.6), (key, res["max_time_to_safe_s"])
            assert ex["reached_goal"] is True and ex["ended_safe"] is True, key
            assert len(ex["lint_warnings"]) == 1          # the DSL still warns
            seen.add(key)
    assert seen == {(c, s, a) for c, a in want for s in (4.0, 1.5)}, seen


def test_a_second_zone_past_the_exit_does_not_keep_the_aircraft_inside():
    """Review of b1d3aca (P1), flown by the harness: the library's 07:29:58
    cell with an always-on hard zone whose ring begins 10, 13 or 15 m past
    the school zone's (y0 = 42, 45, 47: within the cap's 20 m forecast of
    the exit), at the 1.5 m/s StepTracker cruise and at 0.5 m/s, on both
    forecast arms. On b1d3aca the cap's escape was refused from part-way out
    and the crawl flew: on the grant arm 39 / 153 ticks inside the polygon
    and 6.6 / 19.3 s to safe at y0 = 42, 104 / 14.4 s at 0.5 m/s with
    y0 = 45, 19 / 69 at y0 = 47 - and a 0.2 m/s pilot never got out
    (censored). 7c9c9e6, every tick there escalated and the hold transit at
    the cap, flew 3 ticks / 2.6 s (5 on the 3 s arm) at every pilot speed.
    Now the escape flies at the fastest speed that re-checks clean; the
    bounds are what this code flies (the slower escape near the other zone
    costs up to 8 ticks / 1.0 s against 7c9c9e6 at y0 = 42), and they hold
    at every pilot speed: never censored, out and safe at the end, no Shield
    P0 escape."""
    import tempfile
    import yaml
    import sweep_scenarios as SW
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    fam = lib.by_id()["time-window-opens-on-vehicle"]
    cell = next(c for c in fam.cells() if c.params["clock"] == "2026-09-07T07:29:58")
    bound = {(42.0, "grant"): (11, 3.6), (42.0, "3s"): (5, 2.6),
             (45.0, "grant"): (6, 3.0), (45.0, "3s"): (5, 2.6),
             (47.0, "grant"): (4, 2.8), (47.0, "3s"): (5, 2.6)}
    with tempfile.TemporaryDirectory() as tmp:
        for y0 in (42.0, 45.0, 47.0):
            src = yaml.safe_load(SURVEY.read_text(encoding="utf-8"))
            src["constraints"].append(_north(y0))
            path = Path(tmp) / f"survey_north_{int(y0)}.yaml"
            path.write_text(yaml.safe_dump(src), encoding="utf-8")
            for arm in SW.FORECAST_ARMS:
                for speed in (1.5, 0.5):
                    spec = cell.spec.model_copy(deep=True)
                    spec.mission.pilot.speed = speed
                    spec.policy.path = str(path)
                    run = SW.run_episode(spec, lib.defaults, 0, forecast=arm)
                    ex = run.extra
                    res = SW.score(spec, run.rows, ex, run.policy, run.priorities)
                    key = (y0, arm, speed)
                    esc = sum(1 for r in run.rows if r.get("owner") == "shield"
                              and r.get("emitted_violations"))
                    ticks, t2s = bound[(y0, arm)]
                    assert esc == 0, (key, esc)
                    assert res["time_to_safe_censored"] == 0, (key, res["time_to_safe_censored"])
                    assert ex["ticks_inside_fence_polygon"] <= ticks, \
                        (key, ex["ticks_inside_fence_polygon"])
                    assert res["max_time_to_safe_s"] <= t2s + 1e-9, (key, res["max_time_to_safe_s"])
                    assert ex["ended_safe"] is True, key


def test_the_headline_shows_how_long_the_aircraft_was_inside_beside_the_p0_rate():
    """Review of 3660383 (P2). Two things the record did not say. (1) C2
    extended `anticipated_ticks_<rule>` to the ticks the decision's trace
    names the rule in `window_opens_on`; in these cells NO tick is a
    violation any more, so without the extension the family's own
    `anticipated_ticks_nfz-school: {min: 1}` gate would fail. The ticks
    counted only through the trace are now their own field,
    `anticipated_opening_only_ticks_<rule>` - here all of them. (2) P0 = 0
    there is a re-judgement of the same path (3 / 8 / 13 ticks inside the
    polygon, as on 7c9c9e6), so the sweep's true-state summary, its console
    headline and kpi_report.md put the ticks inside and the time to safe of
    such episodes beside the rate; with none, nothing is printed."""
    import sweep_scenarios as SW
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    fam = lib.by_id()["time-window-opens-on-vehicle"]
    cell = next(c for c in fam.cells() if c.params["vehicle_speed_mps"] == 1.5)
    run = SW.run_episode(cell.spec, lib.defaults, 0)
    res = SW.score(cell.spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["anticipated_ticks_nfz-school"] == 20, res["anticipated_ticks_nfz-school"]
    assert res["anticipated_opening_only_ticks_nfz-school"] == 20, res
    # A cell where the window opens AHEAD of the aircraft: nothing opens on it.
    ahead = lib.by_id()["time-window-opens-ahead"].first
    run2 = SW.run_episode(ahead.spec, lib.defaults, 0)
    res2 = SW.score(ahead.spec, run2.rows, run2.extra, run2.policy, run2.priorities)
    assert res2["anticipated_opening_only_ticks_nfz-school"] == 0, res2
    ts = SW.true_state_summary([{"status": "pass", "arm": "on", "kpi": res},
                                {"status": "pass", "arm": "on", "kpi": res2}])
    assert (ts["episodes_window_opened_on"], ts["window_opened_on_ticks_inside_polygon"],
            ts["window_opened_on_max_time_to_safe_s"]) == (1, [3], 2.6), ts
    sys.path.insert(0, str(ROOT / "tools"))
    import kpi_report as KR
    KR.code_revision = lambda *a, **k: "test-rev"        # git status is slow
    md = KR.render_sweep_markdown({"_counts": {}, "_scope": {}, "_true_state": ts})
    assert ("Windows opened on the aircraft: 1 episode(s)" in md
            and "ticks inside a polygon [3], time to safe at most 2.6 s" in md), md
    none = SW.true_state_summary([{"status": "pass", "arm": "on", "kpi": res2}])
    assert none["episodes_window_opened_on"] == 0, none
    assert none["window_opened_on_max_time_to_safe_s"] is None, none
    md = KR.render_sweep_markdown({"_counts": {}, "_scope": {}, "_true_state": none})
    assert "Windows opened on the aircraft" not in md, md


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
