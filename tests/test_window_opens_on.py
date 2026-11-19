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

The same review (P4): the sweep summary's "windows opened on the aircraft"
listed only episodes where some tick's action was already the escape; an
episode now records the ticks whose decision named the window at all
(`window_opened_on_ticks`), and a window that opened on a hovering pilot is
listed. That test fails on b1d3aca's sweep code.

REVIEW OF 69c27bd (2026-10-22, P2/P3). The cap was still refused in the
library's OWN geometry: the cap's 5 s forecast (20 m) was judged as if the
aircraft kept the cap for the whole lookahead, and the corridor-survey-route
edge at x = 80 is only 9 m past the school zone's ring at x = 71, so a
sideways pilot (1-3 m/s east, 1-2 m/s west) got the cap refused, the escape
slowed to the floor and below, and the aircraft stayed 16 ticks inside the
polygon (4.0 s to safe, Normal > Brake > Loiter) against 5-6 / 2.8 s on
7c9c9e6. A zone or a corridor end 4-9 m past the north exit cost 12-14 ticks
against 3. Now:

  * an action escaping a zone that opened on the aircraft is forecast as it
    will be flown - out of the ring, then a stop at the commanded forecast's
    braking model (one forecast step, then 3 m/s^2, as the clearance rule's
    forecast) - so a rule beyond the stopping point no longer refuses it;
    only a rule inside the stop does, and then the fastest speed that
    re-checks clean flies, else the tick is reported and the FSM escalates;
  * the harness flies the sideways pilot and the near zone / corridor end
    under bounds tied to 7c9c9e6 (+1 / +2 ticks), never censored;
  * the sweep counts each episode's `cap_refused` / `floor_refused` ticks and
    puts them beside the P0 headline, so a slowed escape cannot hide behind
    "0 escapes" again.

Run on 69c27bd's code, 8 of the 22 fail or error: the 4 new tests (the
sweep one errors: no such field) and the 4 re-pinned to geometries where a
rule lies inside the escape's stop (the fastest clean retry, the
escalation and the at-floor pilot, the pilot's own escape slowed, and the
619638f harness bounds tightened to 7c9c9e6's 3 / 2.6 s); the other 14 pass.
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
    z = {"id": "school", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "altitude_floor_m": 0.0, "altitude_ceiling_m": 100.0,
         "vertices": [{"x": 50, "y": 12}, {"x": 70, "y": 12}, {"x": 70, "y": 30},
                      {"x": 50, "y": 30}],
         "valid_time": {"recurrence": {"days": ["Mon", "Tue", "Wed", "Thu", "Fri"],
                                       "start_time": "07:30", "end_time": "17:30"}}}
    z.update(over)
    return z


def _caps(**over):
    c = {"id": "caps", "type": "kinematic_envelope", "scope": "mission", "constraint_type": "hard",
         "priority": "P1", "violation_action": "repair", "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}
    c.update(over)
    return c


def _pol(*rules):
    return Policy.model_validate({"policy_id": "opens-on", "constraints": list(rules)})


def _north(y0):
    """An always-on hard P0 zone north of the school zone, y0..y0+6, its 1 m
    ring from y0 - 1. Until the review of 69c27bd the cap's escape north was
    judged on its straight 5 s forecast (4 m/s x 5 s = 20 m), which reaches
    this ring whenever y0 - 1 < 51; since then on the escape as flown, out of
    the school zone's ring (y = 31) and then a stop, which reaches it only
    when y0 - 1 < 34.07 (31 + 4 x 0.1 + 4^2 / (2 x 3), the grant arm's 0.1 s
    step; 31 + 4 x 0.5 + 2.67 = 35.67 on the 3 s arm's 0.5 s step)."""
    return {"id": "nfz-north", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
            "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
            "altitude_floor_m": 0.0, "altitude_ceiling_m": 100.0,
            "vertices": [{"x": 50, "y": y0}, {"x": 70, "y": y0}, {"x": 70, "y": y0 + 6},
                         {"x": 50, "y": y0 + 6}]}


def _survey_src(north=None, corridor_end=None):
    """policies/corridor_survey.yaml (the library cells' policy) as a dict:
    with _north(north) appended, and/or the corridor's last centerline point
    moved to (60, corridor_end - 20), so its round end lies at y =
    corridor_end on the zone's axis (probe2.py's `corr:<yend>`)."""
    import yaml
    src = yaml.safe_load(SURVEY.read_text(encoding="utf-8"))
    if north is not None:
        rule = _north(north)
        if src.get("origin"):
            # Written in lat/lon about the policy's origin, as every writer
            # stores a zone since 2026-11-17: a metre zone beside lat/lon
            # points is the legacy mixed form, which the flight loader (and
            # so the headless runner) refuses since 2026-11-18 (WP1-14). The
            # metres are on the micrometre grid: the same view.
            from guardrail.projection import raw_points_in_wgs84
            rule = raw_points_in_wgs84(rule, (src["origin"]["lat"], src["origin"]["lon"]))
        src["constraints"].append(rule)
    if corridor_end is not None:
        for c in src["constraints"]:
            if c["id"] == "corridor-survey-route":
                pt = {"x": 60.0, "y": corridor_end - 20.0}
                if src.get("origin"):
                    # WGS84 since 2026-11-03 (WP1-14): one rule cannot mix
                    # frames, so the moved point is written in lat/lon about
                    # the policy's origin; its view is the same metres.
                    from guardrail.projection import LocalProjection
                    lat, lon = LocalProjection(src["origin"]["lat"], src["origin"]["lon"]) \
                        .to_latlon(pt["x"], pt["y"])
                    pt = {"lat": lat, "lon": lon}
                c["centerline"][-1] = pt
    return src


def _survey_with_north(y0):
    """policies/corridor_survey.yaml (the library cells' policy) with _north."""
    return Policy.model_validate(_survey_src(north=y0))


def _fly(src, pilot, arm, clock="2026-09-07T07:29:58"):
    """The library's opens-on cell at `clock` flown by run_episode with the
    policy `src` (a dict, as _survey_src) and `pilot` - ("goto", speed) or
    ("constant", vx, vy) - on forecast arm `arm`: (run, score, Shield P0
    escapes). The review's probes (probe2.py) fly exactly this."""
    import tempfile
    import yaml
    import sweep_scenarios as SW
    from guardrail.scenario_spec import Library, PilotSpec
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    fam = lib.by_id()["time-window-opens-on-vehicle"]
    cell = next(c for c in fam.cells() if c.params["clock"] == clock)
    spec = cell.spec.model_copy(deep=True)
    if pilot[0] == "goto":
        spec.mission.pilot.speed = float(pilot[1])
    else:
        spec.mission.pilot = PilotSpec(type="constant",
                                       action={"vx": float(pilot[1]), "vy": float(pilot[2])})
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "policy.yaml"
        path.write_text(yaml.safe_dump(src), encoding="utf-8")
        spec.policy.path = str(path)
        run = SW.run_episode(spec, lib.defaults, 0, forecast=arm)
    res = SW.score(spec, run.rows, run.extra, run.policy, run.priorities)
    esc = sum(1 for r in run.rows if r.get("owner") == "shield" and r.get("emitted_violations"))
    return run, res, esc


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
    flies back in on its own. reset_episode() forgets.

    That distinction is recovery_law "cap" (2026-10-29 to 2026-11-06), pinned
    on it below. Since 2026-11-06 the default "cap_all" leaves EVERY static
    zone the aircraft is in at the cap, remembered or not (`escape_at_cap`
    in the trace): the fresh Shield and a flight back in leave at 4 m/s too
    (the end of this test)."""
    clock = [MON_0729_58]
    sh = Shield(load_policy(SURVEY), now=lambda: clock[0], recovery_law="cap")
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
    fresh = Shield(load_policy(SURVEY), now=lambda: clock[0], recovery_law="cap")
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
    sh2 = Shield(load_policy(SURVEY), now=lambda: clock[0], recovery_law="cap")
    sh2.filter(MID, crawl, t=0.0)
    sh2.reset_episode()
    clock[0] = MON_0729_58 + timedelta(seconds=2.5)
    d = sh2.filter(inside, crawl, t=0.0)
    assert d.trace["window_opened_on"] == [] and abs(d.emitted.vy - 2.0) < 1e-9, \
        (d.trace, d.emitted)
    # The default "cap_all": a fresh Shield, and a flight back in on its own,
    # leave at the cap too.
    fresh = Shield(load_policy(SURVEY), now=lambda: clock[0])
    df = fresh.filter(inside, crawl, t=2.5)
    assert df.trace["window_opened_on"] == [] and df.trace["escape_at_cap"] == ["nfz-school"], \
        df.trace
    assert abs(df.emitted.vy - 4.0) < 1e-9, df.emitted
    clock[0] = MON_0729_58 + timedelta(seconds=3.6)
    d = fresh.filter(State(x=60.0, y=29.0, up=15.0), crawl, t=3.6)
    assert abs(d.emitted.vy - 4.0) < 1e-9, d.emitted


def test_a_refused_cap_escape_flies_the_fastest_speed_that_re_checks_clean():
    """Review of b1d3aca (P1). The cap's escape forecasts 20 m ahead; with
    another hard zone in that reach the floored action broke it, the floor
    was refused, and the crawl flew unfloored ("raw flown: the floored action
    breaks nfz-north"), legal by the inside rule: no violation, FSM Normal,
    the KPI's 0. One tick of it: the Shield that remembered the opening
    emitted the 0.2 m/s crawl at (60, 27) where a fresh Shield emitted the
    2.0 floor. Now the run is made again slower, 0.25 m/s at a time, and the
    fastest that re-checks clean flies, and the trace says what the cap broke.

    WHICH RULE REFUSES THE CAP (review of 69c27bd, P2). That fix judged the
    cap's escape on its straight 5 s forecast, as if the aircraft kept 4 m/s
    for 20 m, so any rule within 20 m of the exit refused it - the library's
    own corridor edge, 9 m past the school zone's ring, among them (the next
    test). The escape is now judged as it is flown: straight out of the
    ring, then a stop (one 0.1 s forecast step, then 3 m/s^2, the commanded
    forecast's braking model, as the clearance rule's forecast). Only a rule
    inside that stop refuses the cap. With the other zone's ring 2 m past the
    exit (y = 33) the cap stops at 34.07 (31 + 0.4 + 2.67), 3.75 / 3.5 /
    3.25 m/s at 33.72 / 33.39 / 33.09, and 3.0 m/s at 32.8: flown at 3.0,
    from the middle before the opening and from (60, 27) after it alike; with
    the ring 2.5 m past (y = 33.5), at 3.5. On 69c27bd the same ticks flew
    2.25 (2.5) m/s from the middle, and at (60, 27) no speed's straight
    forecast was clean, so the crawl was reported and the FSM took over
    (Brake [X1])."""
    crawl = Action4D(vy=0.2)
    for y0, want in ((34.0, 3.0), (34.5, 3.5)):
        clock = [MON_0729_58]
        sh = Shield(_survey_with_north(y0), now=lambda: clock[0])
        d = sh.filter(MID, crawl, t=0.0)
        assert d.trace["window_opens_on"] == ["nfz-school"], d.trace
        assert d.violations == [] and abs(d.emitted.vy - want) < 1e-9, (y0, d.violations, d.emitted)
        assert d.trace["cap_refused"] == (f"flown at {want:.2f} m/s; at the cap the floored "
                                          f"action breaks nfz-north"), (y0, d.trace)
        clock[0] = MON_0729_58 + timedelta(seconds=2.5)
        inside = State(x=60.0, y=27.0, up=15.0)
        d = sh.filter(inside, crawl, t=2.5)
        assert d.trace["window_opened_on"] == ["nfz-school"], d.trace
        assert d.violations == [] and d.emitted_violations == [], \
            (y0, d.violations, d.emitted_violations)
        assert abs(d.emitted.vx) < 1e-9 and abs(d.emitted.vy - want) < 1e-9, (y0, d.emitted)
        assert d.trace["cap_refused"] == (f"flown at {want:.2f} m/s; at the cap the floored "
                                          f"action breaks nfz-north"), (y0, d.trace)
        assert "floor_refused" not in d.trace and sh.n_floor_refused == 0, d.trace
        assert [r.operator for r in d.repairs] == ["GeofenceEscape"], d.repairs
        assert d.repairs[0].recovery is True and "under the 4.0 m/s cap" in d.repairs[0].detail
        assert d.fsm_state_after == "Normal", d.fsm_state_after
    # With the ring 4 m past the exit (y0 = 36, ring at y = 35) the cap's
    # stop, 34.07, is short of it: the cap flies, nothing refused. (On
    # 69c27bd: 2.75 m/s.)
    d = _shield(policy=_survey_with_north(36.0)).filter(MID, crawl)
    assert abs(d.emitted.vy - 4.0) < 1e-9 and "cap_refused" not in d.trace, (d.emitted, d.trace)


def test_the_cap_escape_is_judged_out_of_the_ring_then_braked_to_a_stop():
    """Review of 69c27bd (P2), in the library's own geometry. The
    corridor-survey-route edge at x = 80 is 9 m past the school zone's ring
    at x = 71. A pilot flying 2 m/s east at the opening: the cap's straight
    5 s forecast from (61.6, 21) ends at x = 81.6, outside the corridor, so
    69c27bd refused the cap and flew 3.5 m/s, slower each tick (the forecast
    of the speed that keeps x + 5 v <= 80), until at (70.5, 21) the pilot's
    own 2 m/s (80.5) broke the corridor and the FSM went Normal -> Brake
    [X1], then to a Loiter: 16 ticks inside the polygon, 4.0 s to safe,
    against 5 / 2.8 s on 7c9c9e6. Judged as flown - out of the ring, then a
    stop (one 0.1 s step at the speed, then 3 m/s^2) - the cap stops at
    74.07 and the pilot's 2 m/s at 71.87: the cap flies on every tick, and
    the FSM stays Normal.

    Only for a zone that opened on the aircraft: a Shield that never saw it
    open (here, one built after the opening: as far as it knows the aircraft
    flew in) judges the straight forecast as before, and the same 4 m/s
    breaks the corridor there. A stop is never shaped (nothing escapes)."""
    clock = [MON_0729_58]
    sh = Shield(load_policy(SURVEY), now=lambda: clock[0])
    east = Action4D(vx=2.0)
    d = sh.filter(MID, east, t=0.0)
    assert abs(d.emitted.vx - 4.0) < 1e-9 and abs(d.emitted.vy) < 1e-9, d.emitted
    clock[0] = MON_0729_58 + timedelta(seconds=0.4)
    d = sh.filter(State(x=61.6, y=21.0, up=15.0), east, t=0.4)
    assert d.trace["window_opens_on"] == ["nfz-school"], d.trace
    assert abs(d.emitted.vx - 4.0) < 1e-9 and abs(d.emitted.vy) < 1e-9, d.emitted   # was 3.50
    assert "cap_refused" not in d.trace, d.trace
    assert d.violations == [] and d.emitted_violations == [], (d.violations, d.emitted_violations)
    clock[0] = MON_0729_58 + timedelta(seconds=2.6)
    near = State(x=70.5, y=21.0, up=15.0)
    d = sh.filter(near, east, t=2.6)
    assert d.trace["window_opened_on"] == ["nfz-school"], d.trace
    assert d.violations == [] and d.emitted_violations == [], (d.violations, d.emitted_violations)
    assert abs(d.emitted.vx - 4.0) < 1e-9 and abs(d.emitted.vy) < 1e-9, d.emitted
    assert d.fsm_state_after == "Normal", (d.fsm_state_after, d.fsm_edge)          # was Brake [X1]
    assert sh._check(near, Action4D(vx=4.0)) == []
    assert sh._check(near, Action4D(vx=2.0)) == []
    fresh = Shield(load_policy(SURVEY), now=lambda: clock[0], recovery_law="cap")
    assert [v.rule_id for v in fresh._check(near, Action4D(vx=4.0))] == ["corridor-survey-route"]
    # Since 2026-11-06 the default law ("cap_all") flies every escape from a
    # static hard zone's ring at the cap, judged as flown, whether or not
    # the Shield saw the zone open: the fresh Shield's 4 m/s stops at 74.07
    # as well, and the corridor holds.
    fresh_all = Shield(load_policy(SURVEY), now=lambda: clock[0])
    assert fresh_all.recovery_law == "cap_all"
    assert fresh_all._check(near, Action4D(vx=4.0)) == []
    with sh._scope(sh._new_tick(None)):
        assert sh._shaped_escape(near, BRAKE) is None
        assert sh._shaped_escape(near, Action4D(vx=-1.0)) is None      # not leaving east
        poses = sh._shaped_escape(near, Action4D(vx=4.0))
        assert poses is not None and [t for t, _ in poses] == list(sh._grid()), poses
        # Straight at 4 m/s until 0.1 s after the ring (x = 71 at 0.125 s),
        # then 3 m/s^2 to a stop at 71.4 + 16 / 6 = 74.07, held to the horizon.
        assert abs(max(p.x for _, p in poses) - (71.4 + 16.0 / 6.0)) < 1e-6, poses[-1]
        assert abs(poses[-1][1].x - (71.4 + 16.0 / 6.0)) < 1e-6 and poses[-1][1].y == 21.0


def test_the_cap_escape_brakes_as_the_clearance_response_does():
    """Under a clear_response (the ArduPilot rails since 2026-11-19) the
    shaped escape brakes with that response, as the clearance horizon
    does (_clear_horizon_s): one 0.1 s step plus the 0.45 s reaction at the
    speed after the ring, then 1.75 m/s^2. The same opening as above: out of
    the ring at 0.125 s, so straight to 70.5 + 4 * 0.675 = 73.2, then a stop
    at 73.2 + 16 / 3.5 = 77.77 (the instant vehicle's 74.07), still short of
    the corridor's edge at x = 80, so the cap still flies."""
    from guardrail.response import ARDUCOPTER_GUIDED as R
    clock = [MON_0729_58]
    sh = Shield(load_policy(SURVEY), now=lambda: clock[0], clear_response=R)
    east = Action4D(vx=2.0)
    sh.filter(MID, east, t=0.0)
    clock[0] = MON_0729_58 + timedelta(seconds=0.4)
    d = sh.filter(State(x=61.6, y=21.0, up=15.0), east, t=0.4)
    assert d.trace["window_opens_on"] == ["nfz-school"], d.trace
    clock[0] = MON_0729_58 + timedelta(seconds=2.6)
    near = State(x=70.5, y=21.0, up=15.0)
    d = sh.filter(near, east, t=2.6)
    assert abs(d.emitted.vx - 4.0) < 1e-9 and d.emitted_violations == [], d.emitted
    stop = 70.5 + 4.0 * (0.125 + 0.1 + R.reaction_s) + 16.0 / (2.0 * R.accel_mps2)
    with sh._scope(sh._new_tick(None)):
        poses = sh._shaped_escape(near, Action4D(vx=4.0))
        assert poses is not None and [t for t, _ in poses] == list(sh._grid()), poses
        assert abs(max(p.x for _, p in poses) - stop) < 1e-6, (poses[-1], stop)
        assert abs(poses[-1][1].x - stop) < 1e-6 and poses[-1][1].y == 21.0


def test_with_no_floored_escape_left_the_crawl_is_reported_and_the_fsm_escalates():
    """Review of b1d3aca (P1). When not even the in-force floor (2.0 m/s,
    straight or with the tangential kept) re-checks clean, the crawl must not
    pass as a clean tick: it is the zone's violation ("under the 2.0 m/s
    escape floor"), in `violations` and in `emitted_violations`, so the FSM
    escalates - Normal -> Brake [X1], the stop withheld because a stop there
    is illegal, the Shield's own action (the crawl, which breaks no other
    rule) flown - as 7c9c9e6 escalated every tick there, and the KPI counts
    it as the P0 escape it is. A pilot already leaving at the floor is the
    floor: it flies, nothing reported. On b1d3aca the crawl flew with no
    violation and the FSM stayed Normal.

    Since the review of 69c27bd (P2) every speed is judged as flown, out of
    the ring and then a stop (one 0.1 s step, then 3 m/s^2): the floor's 2.0
    m/s stops 0.87 m past the exit, 2.25 m/s 1.07 m. So the other zone's
    ring must lie within that for nothing to be left: 0.5 m past the exit
    (y0 = 32.5) for the crawl at (60, 29.5); 0.95 m past it (y0 = 32.95) for
    the pilot at the floor, which flies (2.0 stops at 31.87, short of 31.95)
    while every faster try is refused - the trace says so. With the ring at
    y = 39, as these cases had it on 69c27bd, the crawl now leaves at the cap
    (it stops at 34.07) and the 2.0 m/s pilot is raised to it."""
    crawl = Action4D(vy=0.2)
    clock = [MON_0729_58]
    sh = Shield(_survey_with_north(32.5), now=lambda: clock[0])
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
    sh = Shield(_survey_with_north(32.95), now=lambda: clock[0])
    clock[0] = MON_0729_58
    sh.filter(MID, crawl, t=0.0)
    clock[0] = MON_0729_58 + timedelta(seconds=2.5)
    at_floor = Action4D(vy=2.0)
    d = sh.filter(State(x=60.0, y=28.5, up=15.0), at_floor, t=2.5)
    assert d.violations == [] and d.emitted == at_floor, (d.violations, d.emitted)
    assert d.fsm_state_after == "Normal", d.fsm_state_after
    assert d.trace["floor_refused"] == "raw flown: the floored action breaks nfz-north", d.trace
    # The ring 8 m past the exit, as before: the cap's stop is short of it.
    for raw, at in ((crawl, State(x=60.0, y=29.5, up=15.0)), (at_floor, State(x=60.0, y=28.5, up=15.0))):
        clock[0] = MON_0729_58
        sh = Shield(_survey_with_north(40.0), now=lambda: clock[0])
        sh.filter(MID, crawl, t=0.0)
        clock[0] = MON_0729_58 + timedelta(seconds=2.5)
        d = sh.filter(at, raw, t=2.5)
        assert d.violations == [] and abs(d.emitted.vy - 4.0) < 1e-9, (raw, d.violations, d.emitted)
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
    clean, its lateral part kept on its own side - a recovery the FSM reads
    as repaired (Normal -> Brake [G1]). On b1d3aca: 4 m/s east and 4 m/s
    west.

    Since the review of 69c27bd (P2) the pilot's escape is judged as flown
    too, out of the ring and then a stop: with the ring 18 m north (y0 = 40)
    its 4 m/s stops at 34.07, 5 m short, and it flies at the cap, nothing
    violated (69c27bd slowed it to 3.5; the floor straightens the 0.05 m/s
    across, as the cap escape does). With the ring 2 m past the exit (y0 = 34) the 4 m/s
    stop breaks it and the escape is slowed to 3.0 m/s (32.8); 69c27bd, on
    the straight forecast, slowed it to 2.25."""
    for vx in (0.05, -0.05):
        raw = Action4D(vx=vx, vy=math.sqrt(16.0 - vx * vx))
        d = _shield(policy=_survey_with_north(34.0)).filter(MID, raw)
        assert [v.rule_id for v in d.violations] == ["nfz-north"], (vx, d.violations)
        assert d.emitted_violations == [], (vx, d.emitted_violations)
        assert abs(d.emitted.vy - 3.0) < 1e-9 and abs(d.emitted.vx - vx) < 1e-9, (vx, d.emitted)
        assert [r.operator for r in d.repairs] == ["GeofenceEscape"], (vx, d.repairs)
        assert d.repairs[0].recovery is True and "nfz-north" in d.repairs[0].detail, d.repairs
        assert (d.fsm_state_after, d.fsm_edge) == ("Brake", "G1"), (vx, d.fsm_edge)
        d = _shield(policy=_survey_with_north(40.0)).filter(MID, raw)
        assert d.violations == [] and d.emitted_violations == [], (vx, d.violations)
        assert abs(d.emitted.vy - 4.0) < 1e-9 and abs(d.emitted.vx) < 1e-9, (vx, d.emitted)
        assert d.fsm_state_after == "Normal", (vx, d.fsm_edge)


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
    for caps in ((), (_caps(scope="mission", constraint_type="soft"),),
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
    Then (69c27bd) the escape flew at the fastest speed whose STRAIGHT
    forecast re-checked clean: up to 11 ticks / 3.6 s at y0 = 42 on the
    grant arm. Since the review of 69c27bd (P2) it is judged as flown, out
    of the ring and then a stop at 34.07, short of every one of these rings
    (35-46), so the cap flies and the cells fly 7c9c9e6's numbers exactly -
    3 ticks / 2.6 s (5 / 2.6 s on the 3 s arm) at 0.2, 0.5, 1.5 and 4 m/s
    alike, measured - now pinned as the bounds: never censored, out and safe
    at the end, no Shield P0 escape."""
    import tempfile
    import yaml
    import sweep_scenarios as SW
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    fam = lib.by_id()["time-window-opens-on-vehicle"]
    cell = next(c for c in fam.cells() if c.params["clock"] == "2026-09-07T07:29:58")
    bound = {(y0, arm): ((3, 2.6) if arm == "grant" else (5, 2.6))
             for y0 in (42.0, 45.0, 47.0) for arm in ("grant", "3s")}
    with tempfile.TemporaryDirectory() as tmp:
        for y0 in (42.0, 45.0, 47.0):
            src = _survey_src(north=y0)            # the zone in lat/lon (WP1-14)
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


def test_a_sideways_pilot_in_the_library_geometry_leaves_at_the_cap():
    """Review of 69c27bd (P2), flown by the harness in the PLAIN library
    geometry (no extra zone): the 07:29:58 cell with a constant pilot 2.0 or
    1.0 m/s east, or 1.0 m/s west, at the opening. The escape goes east (or
    west), and the corridor-survey-route edge at x = 80 (40) is 9 m past the
    school zone's ring at x = 71 (49): on 69c27bd the cap was refused on its
    straight 5 s forecast, the escape crawled down to the floor and below,
    and the aircraft stayed 16 ticks inside the polygon on the grant arm,
    4.0 s to safe, the FSM Normal > Brake > Loiter (the pilot's own forecast
    reaching the corridor edge from 10 m short of it). 7c9c9e6, every tick
    escalated and the hold transit at the cap: 5 / 5 / 6 ticks (3 s arm 7 /
    7 / 8), 2.8 s. Bounds: 7c9c9e6's ticks + 1, time to safe at most 3.0 s,
    never censored, no Shield P0 escape, no refused cap or floor, and the
    FSM Normal on every tick the aircraft is in the ring (no slowed escape
    escalated into a Loiter)."""
    base = {(2.0, "grant"): 5, (1.0, "grant"): 5, (-1.0, "grant"): 6,
            (2.0, "3s"): 7, (1.0, "3s"): 7, (-1.0, "3s"): 8}
    for (vx, arm), b in base.items():
        run, res, esc = _fly(_survey_src(), ("constant", vx, 0.0), arm)
        ex, key = run.extra, (vx, arm)
        assert esc == 0, (key, esc)
        assert ex["ticks_inside_fence_polygon"] <= b + 1, (key, ex["ticks_inside_fence_polygon"])
        assert res["max_time_to_safe_s"] <= 3.0 + 1e-9, (key, res["max_time_to_safe_s"])
        assert res["time_to_safe_censored"] == 0, key
        assert (ex["cap_refused_ticks"], ex["floor_refused_ticks"]) == (0, 0), \
            (key, ex["cap_refused_ticks"], ex["floor_refused_ticks"])
        ring = [r for r in run.rows if r["unsafe"]]
        assert ring and all(r["owner"] == "shield" and r["fsm_state_after"] == "Normal"
                            for r in ring), (key, [(r["t"], r["fsm_state_after"]) for r in ring])
        assert ex["ended_safe"] is True, key


def test_a_zone_or_corridor_end_just_past_the_exit_costs_at_most_two_ticks():
    """Review of 69c27bd (P2): the 07:29:58 cell flown north by the goto
    pilot at 1.5 and 0.5 m/s with a hard rule a few metres past the north
    exit (the school zone's ring at y = 31): an always-on zone at y 36-42
    (ring from y = 35, probe2.py `north:36`) or the corridor's round end
    moved to y = 40 (`corr:40`). On 69c27bd the cap's straight forecast
    reached both, the escape slowed, and the aircraft stayed 12-14 ticks
    inside the polygon on the grant arm (3.5-3.8 s, the FSM to a Loiter),
    13 on the 3 s arm at north:36, against 3 (5) on 7c9c9e6. Judged as
    flown, the cap stops at 34.07 on the grant arm (0.1 s step), short of both;
    on the 3 s arm (0.5 s step) it stops at 35.67, inside the zone's ring,
    so 3.5-3.75 m/s flies there. Bounds: 7c9c9e6's ticks + 2, never censored, no
    Shield P0 escape, out and safe at the end."""
    for name, src in (("north:36", _survey_src(north=36.0)),
                      ("corr:40", _survey_src(corridor_end=40.0))):
        for arm, b in (("grant", 3), ("3s", 5)):
            for speed in (1.5, 0.5):
                run, res, esc = _fly(src, ("goto", speed), arm)
                ex, key = run.extra, (name, arm, speed)
                assert esc == 0, (key, esc)
                assert ex["ticks_inside_fence_polygon"] <= b + 2, \
                    (key, ex["ticks_inside_fence_polygon"])
                assert res["time_to_safe_censored"] == 0, key
                assert ex["ended_safe"] is True, key


def test_the_sweep_counts_refused_caps_and_floors_beside_the_p0_headline():
    """Review of 69c27bd (P3). A slowed escape scores no P0 escape - it is
    legal - so "0 escapes" said nothing about it. Each episode now records
    its ticks whose decision refused the cap (`cap_refused_ticks`: a slower
    escape flown) or the floor (`floor_refused_ticks`: the pilot's action
    flown unfloored, or reported to the FSM), each row says which
    (`cap_refused` / `floor_refused`), and the true-state summary, the
    console headline and kpi_report.md put the totals beside the P0 rate.
    The case: the zone's ring 2 m past the exit (y0 = 34), the 1.5 m/s goto
    pilot - the cap is refused on every tick until the aircraft is out of
    the ring, flown at 3.0 m/s. The plain library cell refuses none."""
    import sweep_scenarios as SW
    run, res, _ = _fly(_survey_src(north=34.0), ("goto", 1.5), "grant")
    n = sum(1 for r in run.rows if r.get("cap_refused"))
    assert n > 0 and res["cap_refused_ticks"] == n, (n, res["cap_refused_ticks"])
    assert res["floor_refused_ticks"] == sum(1 for r in run.rows if r.get("floor_refused"))
    assert all(r["cap_refused"].startswith("flown at 3.00 m/s")
               for r in run.rows if r.get("cap_refused")), \
        [r["cap_refused"] for r in run.rows if r.get("cap_refused")][:3]
    run2, res2, _ = _fly(_survey_src(), ("goto", 1.5), "grant")
    assert (res2["cap_refused_ticks"], res2["floor_refused_ticks"]) == (0, 0), res2
    ts = SW.true_state_summary([{"status": "pass", "arm": "on", "kpi": res},
                                {"status": "pass", "arm": "on", "kpi": res2}])
    assert (ts["cap_refused_ticks"], ts["floor_refused_ticks"], ts["episodes_escape_slowed"]) == \
        (n, res["floor_refused_ticks"], 1), ts
    sys.path.insert(0, str(ROOT / "tools"))
    import kpi_report as KR
    KR.code_revision = lambda *a, **k: "test-rev"        # git status is slow
    md = KR.render_sweep_markdown({"_counts": {}, "_scope": {}, "_true_state": ts})
    assert f"cap refused on {n} tick(s)" in md and "in 1 episode(s)" in md, md
    none = SW.true_state_summary([{"status": "pass", "arm": "on", "kpi": res2}])
    assert (none["cap_refused_ticks"], none["floor_refused_ticks"],
            none["episodes_escape_slowed"]) == (0, 0, 0), none
    md = KR.render_sweep_markdown({"_counts": {}, "_scope": {}, "_true_state": none})
    assert "Escapes slowed: none" in md, md


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


def test_a_window_that_opened_on_a_hovering_pilot_is_listed_too():
    """Review of b1d3aca (P4). The summary's "windows opened on the aircraft"
    listed only episodes with an `anticipated_opening_only_ticks_*` tick - a
    tick whose action was ALREADY the escape. The library's 07:29:58 cell
    flown by a hover (or a 0.3 m/s drift east): every tick before the opening
    is a violation, then repaired, so 20 anticipated ticks, 0 opening-only -
    and the episode, 3 ticks inside the polygon, was not listed (0 episodes,
    []). Now each episode records `window_opened_on_ticks`: the ticks whose
    decision named the window in `window_opens_on` or `window_opened_on`,
    whatever was flown - 26 in each of these runs (20 before the opening, 6
    after it until the ring is left), and the summary lists any episode with
    one. A window opening ahead of the aircraft names none."""
    import sweep_scenarios as SW
    from guardrail.scenario_spec import Library, PilotSpec
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    fam = lib.by_id()["time-window-opens-on-vehicle"]
    cell = next(c for c in fam.cells() if c.params["clock"] == "2026-09-07T07:29:58"
                and c.params["vehicle_speed_mps"] == 4.0)
    recs = []
    for act in ({"vx": 0.0, "vy": 0.0}, {"vx": 0.3}, None):
        spec = cell.spec.model_copy(deep=True)
        if act is not None:
            spec.mission.pilot = PilotSpec(type="constant", action=act)
        run = SW.run_episode(spec, lib.defaults, 0)
        res = SW.score(spec, run.rows, run.extra, run.policy, run.priorities)
        only = 0 if act is not None else 20
        assert (res["anticipated_ticks_nfz-school"],
                res["anticipated_opening_only_ticks_nfz-school"],
                res["window_opened_on_ticks"], res["ticks_inside_fence_polygon"]) == \
            (20, only, 26, 3), (act, res["anticipated_opening_only_ticks_nfz-school"],
                                res["window_opened_on_ticks"])
        ts = SW.true_state_summary([{"status": "pass", "arm": "on", "kpi": res}])
        assert (ts["episodes_window_opened_on"],
                ts["window_opened_on_ticks_inside_polygon"]) == (1, [3]), (act, ts)
        recs.append({"status": "pass", "arm": "on", "kpi": res})
    ahead = lib.by_id()["time-window-opens-ahead"].first
    run = SW.run_episode(ahead.spec, lib.defaults, 0)
    res = SW.score(ahead.spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["window_opened_on_ticks"] == 0, res["window_opened_on_ticks"]
    ts = SW.true_state_summary(recs + [{"status": "pass", "arm": "on", "kpi": res}])
    assert ts["episodes_window_opened_on"] == 3, ts


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
