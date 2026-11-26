"""The landing descent goes through the Shield (demo/follow_vlm.py, 2026-11-20).

Run either way:
    pytest tests/test_landing_descent.py -v
    python tests/test_landing_descent.py

The flight to the landing site was filtered by the Shield, but the vertical
descent to 1.2 m and land_async went straight to the simulator ("Shield not
in this loop" on the HUD). On 19 Nov two of seven AGX flights listed a level
pedestrian among their after-mission contacts, at a site where pedestrians
walked within a few metres of the descending aircraft
(docs/data/onepath_agx_20261119/diagnosis_20261119.md section 3).

These tests fly _descend_through_shield / _land against a kinematic stand-in
for the simulator (no simulator, no network) and a real flight Shield, and
pin: every descent command is a Shield decision; only an altitude floor is set
aside, labelled and counted; any other objection holds the descent (nothing
goes down on a refused tick) and gives it up; a refused column is not
descended; _land re-plans away from a refused place and lands only on a
cleared last metre; persons near the column are recorded, never decided on.
"""
import asyncio
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import follow_vlm as F                                           # noqa: E402
from guardrail import State                                      # noqa: E402
from guardrail.models import DynamicNFZ, Policy, mark_legacy_read  # noqa: E402
from wgs84_fixture import geo, geo_ring              # noqa: E402

HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

TICK_S = 0.1


def _policy(low_zone=True, climb=2.0):
    """The follow policies' altitude band (P0, 6-14 m) and kinematic caps,
    plus - where the column test must fail - a P0 zone from the ground to
    4 m round (0, 0): a place the aircraft may fly over but not land in."""
    cons = [
        {"id": "alt-band", "type": "altitude_envelope", **HARD_P0,
         "alt_min_m": 6, "alt_max_m": 14},
        {"id": "kin-caps", "type": "kinematic_envelope", **HARD_P0, "priority": "P1",
         "speed_max_mps": 5.0, "climb_rate_max_mps": climb, "yaw_rate_max_dps": 45.0},
    ]
    if low_zone:
        cons.append({"id": "low-plaza", "type": "polygon_fence", **HARD_P0,
                     "vertices": [{"x": -5, "y": -5}, {"x": 5, "y": -5},
                                  {"x": 5, "y": 5}, {"x": -5, "y": 5}],
                     "altitude_floor_m": 0, "altitude_ceiling_m": 4, "margin_m": 1.0})
    # An origin, so a zone can be hot-applied mid-flight (WP1-14: a policy
    # states its WGS84 frame); the follow policies' home-geo-point.
    # Laid out in local metres about it: a declared legacy read, so the
    # flight Shield may be built on it (Shield.__init__, WP1-14).
    return mark_legacy_read(Policy.model_validate({"policy_id": "landing-descent-test",
                                                   "origin": {"lat": 35.6895, "lon": 139.6917},
                                                   "constraints": cons}))


class FakeDrone:
    """The simulator calls the landing makes, integrated kinematically: each
    velocity command is flown for one tick. Every command is kept."""

    def __init__(self, x, y, up, on_move=None):
        self.x, self.y, self.z = float(x), float(y), -float(up)
        self.cmds = []
        self.ups = []                 # the height each command was sent at
        self.on_move = on_move
        self.landed = False

    @property
    def up(self):
        return -self.z

    def get_ground_truth_kinematics(self):
        return {"pose": {"position": {"x": self.x, "y": self.y, "z": self.z},
                         "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}}

    async def move_by_velocity_async(self, vn, ve, vd, duration=0.3, **kw):
        self.cmds.append((float(vn), float(ve), float(vd)))
        self.ups.append(self.up)
        self.x += vn * TICK_S
        self.y += ve * TICK_S
        self.z += vd * TICK_S
        if self.on_move is not None:
            self.on_move(self)


class _Args:
    cruise_alt = 8.0
    alt_gain = 0.6
    climb_max = 1.8
    citymap = "demo/out/citymap_citylife/occ_day.npz"


def _shield(pol):
    sh = F.flight_shield(pol, None)
    calls = []
    real = sh.filter_body

    def counted(*a, **k):
        calls.append(a)
        return real(*a, **k)
    sh.filter_body = counted
    return sh, calls


def _descend(drone, sh, **kw):
    return asyncio.run(F._descend_through_shield(drone, sh, _Args(), sleep_s=0.0, **kw))


def test_a_clear_column_is_descended_through_the_shield_and_cleared():
    sh, calls = _shield(_policy(low_zone=False))
    drone = FakeDrone(30.0, 0.0, 8.0)
    rec = _descend(drone, sh)
    assert rec["outcome"] == "cleared", rec
    assert drone.up <= F.LAND_TOUCHDOWN_M + 1e-9, drone.up
    # every descent command was a Shield decision, one per command flown
    assert rec["ticks"] == len(calls) == len(drone.cmds) > 0, (rec["ticks"], len(calls))
    # the floor is crossed - set aside, labelled and counted - nothing refused
    assert rec["floor_set_aside"] == ["alt-band"], rec["floor_set_aside"]
    assert rec["verdicts"]["floor"] > 0 and rec["verdicts"]["refused"] == 0, rec["verdicts"]
    assert rec["column_at_start"] == [] and rec["column_at_last_metre"] == []
    assert all(c[2] > 0.0 for c in drone.cmds), "a cleared descent goes down"


def test_a_column_the_shield_refuses_is_not_descended():
    sh, calls = _shield(_policy())
    drone = FakeDrone(0.0, 0.0, 8.0)                    # above the low zone
    rec = _descend(drone, sh)
    assert rec["outcome"] == "column_blocked", rec
    assert rec["column_at_start"] == ["low-plaza"], rec
    assert all(c[2] <= 0.0 for c in drone.cmds), drone.cmds
    assert abs(drone.up - 8.0) < 1e-9


def test_a_refusal_mid_descent_holds_and_gives_the_descent_up():
    """A zone hot-applied under the aircraft mid-descent (its top 3 m up):
    from that tick no command goes down, and after LAND_REFUSE_TICKS refused
    ticks the descent is given up with the zone named."""
    sh, _calls = _shield(_policy(low_zone=False))
    state = {"applied_at": None}

    def spawn(d):
        if state["applied_at"] is None and d.up < 7.0:
            sh.hot_apply(geo(DynamicNFZ.model_validate({
                "id": "spawned-plaza", "type": "dynamic_nfz", **HARD_P0, "margin_m": 1.0,
                "altitude_floor_m": 0.0, "altitude_ceiling_m": 3.0,
                "vertices": [{"x": 25, "y": -5}, {"x": 35, "y": -5},
                             {"x": 35, "y": 5}, {"x": 25, "y": 5}]}), sh))
            state["applied_at"] = len(d.cmds)

    drone = FakeDrone(30.0, 0.0, 8.0, on_move=spawn)
    rec = _descend(drone, sh)
    assert state["applied_at"] is not None
    assert rec["outcome"] == "refused", rec
    assert "spawned-plaza" in rec["refused_by"], rec["refused_by"]
    assert rec["verdicts"]["refused"] >= F.LAND_REFUSE_TICKS
    after = drone.cmds[state["applied_at"]:]
    assert after and all(c[2] <= 0.0 for c in after), after
    assert drone.up > 3.0, drone.up


def test_land_replans_away_from_a_refused_place_and_lands_only_when_cleared():
    sh, _calls = _shield(_policy())
    drone = FakeDrone(0.0, 0.0, 8.0)
    asked = []

    async def fly_to_site(exclude):
        asked.append(list(exclude))
        drone.x, drone.y = 30.0, 0.0                    # the transit, through the Shield
        return {"site": [30.0, 0.0], "reached": True}

    rec, cleared = asyncio.run(F._land(drone, sh, _Args(), fly_to_site=fly_to_site,
                                       sleep_s=0.0))
    assert cleared and rec["outcome"] == "cleared", rec
    assert asked == [[(0.0, 0.0)]], asked
    assert [a["outcome"] for a in rec["attempts"]] == ["column_blocked", "cleared"]
    assert rec["verdicts"]["floor"] > 0 and rec["refused_at"] == [[0.0, 0.0]]


def test_land_never_lands_where_no_site_is_cleared():
    """Every place refused, then no other site: not cleared (the caller does
    not call land_async), the record says why, and no more than
    LAND_MAX_REPLANS re-plans are flown."""
    sh, _calls = _shield(_policy())
    drone = FakeDrone(0.0, 0.0, 8.0)
    asked = []

    async def fly_to_site(exclude):
        asked.append(list(exclude))
        drone.x, drone.y = 1.0 * len(asked), 0.0       # still over the low zone
        return {"site": [drone.x, 0.0], "reached": True}

    rec, cleared = asyncio.run(F._land(drone, sh, _Args(), fly_to_site=fly_to_site,
                                       sleep_s=0.0))
    assert not cleared and rec["outcome"] == "column_blocked", rec
    assert len(asked) == F.LAND_MAX_REPLANS and len(rec["attempts"]) == F.LAND_MAX_REPLANS + 1
    assert all(c[2] <= 0.0 for c in drone.cmds)
    # no --land-site: no re-plan, reported, not landed
    drone2 = FakeDrone(0.0, 0.0, 8.0)
    rec2, cleared2 = asyncio.run(F._land(drone2, sh, _Args(), fly_to_site=None, sleep_s=0.0))
    assert not cleared2 and len(rec2["attempts"]) == 1
    # and a re-plan that finds no site stops there
    drone3 = FakeDrone(0.0, 0.0, 8.0)

    async def none_left(exclude):
        return {"site": None}
    rec3, cleared3 = asyncio.run(F._land(drone3, sh, _Args(), fly_to_site=none_left,
                                         sleep_s=0.0))
    assert not cleared3 and len(rec3["attempts"]) == 1


def test_a_shield_that_cannot_judge_the_descent_holds_it():
    sh, _calls = _shield(_policy(low_zone=False))

    def broken(*a, **k):
        raise RuntimeError("no verdict")
    sh.filter_body = broken
    drone = FakeDrone(30.0, 0.0, 8.0)
    rec = _descend(drone, sh)
    assert rec["outcome"] == "refused", rec
    assert all(c[2] <= 0.0 for c in drone.cmds) and abs(drone.up - 8.0) < 1e-9


def test_the_floor_is_the_only_rule_set_aside():
    """descent_verdict: an altitude-floor objection is 'floor'; the same
    descent into a zone is 'refused', naming the zone; a descent that never
    reaches the floor in the forecast is 'pass'."""
    from guardrail import Action4D
    sh = F.flight_shield(_policy(), None)
    st = State(x=30.0, y=0.0, up=7.0, yaw_deg=0.0)
    d = sh.filter(st, Action4D(vz_up=-1.0))
    assert F.descent_verdict(sh, d, st) == ("floor", [])
    st2 = State(x=0.0, y=0.0, up=5.0, yaw_deg=0.0)       # above the low zone
    d2 = sh.filter(st2, Action4D(vz_up=-1.0))
    verdict, why = F.descent_verdict(sh, d2, st2)
    assert verdict == "refused" and "low-plaza" in why, (verdict, why)
    st3 = State(x=30.0, y=0.0, up=13.0, yaw_deg=0.0)
    d3 = sh.filter(st3, Action4D(vz_up=-0.6))            # 3 m in 5 s: still >= 6 m
    assert F.descent_verdict(sh, d3, st3) == ("pass", [])


class _Figure:
    def __init__(self, x, y):
        self.x, self.y, self.seen, self.stale = x, y, True, 0


class _Crowd:
    """A level crowd as LevelActors duck-types it: polled by refresh()."""

    def __init__(self, pts):
        self.figures = [_Figure(x, y) for x, y in pts]
        self.n_polls = 0

    def refresh(self):
        self.n_polls += 1


def test_persons_near_the_column_are_recorded_and_decide_nothing():
    sh, _calls = _shield(_policy(low_zone=False))
    crowd = _Crowd([(31.0, 0.5), (30.0, 9.0), (50.0, 0.0)])
    drone = FakeDrone(30.0, 0.0, 8.0)
    rec = _descend(drone, sh, people=crowd)
    assert rec["outcome"] == "cleared"                   # no people rule: no decision
    p = rec["persons_near_column"]
    assert p["record_only"] is True and p["decision"] is None
    assert [q[1:] for q in p["at_descent_start"]] == [[31.0, 0.5], [30.0, 9.0]], p
    assert p["at_last_metre"] is not None and crowd.n_polls == 2
    # without a feed the record says so
    rec2 = _descend(FakeDrone(30.0, 0.0, 8.0), sh)
    assert rec2["persons_near_column"]["at_descent_start"] is None
    assert rec2["persons_near_column"]["source"] is None


def test_the_descent_banner_says_what_the_shield_made_of_it():
    """descent_hud paints every 5th descent tick, with the Shield's verdict in
    the banner (it read "Shield not in this loop" before 2026-11-20)."""
    painted = []

    class Rec:
        def set_hud(self, hud, box):
            painted.append(hud)

    class Ind:
        def ended(self, st, text, yaw, t=None):
            return {"banner": {"text": text}}

    sh, _calls = _shield(_policy(low_zone=False))
    rec = _descend(FakeDrone(30.0, 0.0, 8.0), sh, hud=F.descent_hud(Rec(), Ind(), 0.0, "car"))
    assert rec["outcome"] == "cleared"
    assert len(painted) == (rec["ticks"] + 4) // 5, (len(painted), rec["ticks"])
    texts = {h["policy"]["banner"]["text"] for h in painted}
    assert texts <= {"MISSION ENDED - DESCENDING TO LAND (SHIELD CLEARS IT)",
                     "MISSION ENDED - DESCENDING TO LAND (SHIELD CLEARS IT, "
                     "ALTITUDE FLOOR SET ASIDE)"}, texts
    assert all(h["shield"] is False and h["mode"] == "land" for h in painted)


# ------------------------------------------- review of f2ea619 (2026-11-20) --

def _with_verdicts(pattern, fn):
    """Run fn() with descent_verdict answering `pattern` in turn (cycled):
    a refusal that comes and goes, which the Shield's geometry alone is hard
    to make on cue."""
    real = F.descent_verdict
    k = [0]

    def fake(shield, dec, state):
        v = pattern[k[0] % len(pattern)]
        k[0] += 1
        return (v, ["flicker"] if v == "refused" else [])
    F.descent_verdict = fake
    try:
        return fn()
    finally:
        F.descent_verdict = real


def test_a_descent_out_of_ticks_is_timed_out_never_cleared():
    """It ran out of ticks above the last metre: "timed_out", a stop, and
    _land does not clear it - land_async never starts from altitude. (At
    f2ea619 the column test ran from there and could say "cleared".)"""
    sh, _calls = _shield(_policy(low_zone=False))
    drone = FakeDrone(30.0, 0.0, 8.0)
    rec = _with_verdicts(["refused", "floor"], lambda: asyncio.run(
        F._descend_through_shield(drone, sh, _Args(), sleep_s=0.0, max_ticks=8)))
    assert rec["outcome"] == "timed_out" and rec["at"][2] > F.LAND_TOUCHDOWN_M, rec
    assert "column_at_last_metre" not in rec
    assert drone.cmds[-1] == (0.0, 0.0, 0.0)

    async def no_site(exclude):
        return {"site": None}

    async def land():
        real = F._descend_through_shield

        async def short(*a, **k):
            return await real(*a, **{**k, "max_ticks": 8})
        F._descend_through_shield = short
        try:
            return await F._land(FakeDrone(30.0, 0.0, 8.0), sh, _Args(),
                                 fly_to_site=no_site, sleep_s=0.0)
        finally:
            F._descend_through_shield = real
    rec2, cleared = _with_verdicts(["refused", "floor"], lambda: asyncio.run(land()))
    assert not cleared and rec2["outcome"] == "timed_out", rec2


def test_a_refusal_that_comes_and_goes_still_ends_the_descent():
    """4 refused, 1 floor, over and over: never 5 in a row, but 10 of the
    last 20 ends it."""
    sh, _calls = _shield(_policy(low_zone=False))
    drone = FakeDrone(30.0, 0.0, 8.0)
    rec = _with_verdicts(["refused"] * 4 + ["floor"], lambda: _descend(drone, sh))
    assert rec["outcome"] == "refused", rec
    assert rec["verdicts"]["refused"] == F.LAND_REFUSE_IN_WINDOW, rec["verdicts"]
    assert rec["ticks"] < 20


def test_the_descent_asks_no_more_than_the_shields_climb_cap():
    """A 1.0 m/s climb cap and a descent from 12 m (the law asks 1.8): the
    ask is clamped to the cap, so the kinematic rule never refuses it and
    the aircraft lands. At f2ea619 every tick was a kinematic refusal."""
    sh = F.flight_shield(_policy(low_zone=False, climb=1.0), None)
    assert sh.kinematic_caps().climb_rate_max_mps == 1.0
    drone = FakeDrone(30.0, 0.0, 12.0)
    rec = _descend(drone, sh)
    assert rec["outcome"] == "cleared" and rec["verdicts"]["refused"] == 0, rec
    assert max(c[2] for c in drone.cmds) <= 1.0 + 1e-9, max(c[2] for c in drone.cmds)
    assert F.descent_ask(sh, 12.0).vz_up == -1.0


def _corridor_policy():
    return mark_legacy_read(Policy.model_validate({
        "policy_id": "landing-corridor-test", "origin": {"lat": 35.6895, "lon": 139.6917},
        "constraints": [
            {"id": "route", "type": "corridor", **HARD_P0,
             "centerline": [{"x": -50, "y": 0}, {"x": 50, "y": 0}],
             "width_m": 20.0, "altitude_floor_m": 10.0, "altitude_ceiling_m": 20.0},
            {"id": "kin-caps", "type": "kinematic_envelope", **HARD_P0, "priority": "P1",
             "speed_max_mps": 5.0, "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}]}))


def test_a_corridor_floor_is_set_aside_but_not_its_sides():
    """A corridor's floor is one a landing must cross (holdpoint._floor_rules),
    but the Shield reports every corridor breach as category "corridor": at
    f2ea619 the floor refused the descent. Inside the footprint it is now a
    floor; off the footprint it still refuses."""
    from guardrail import Action4D
    sh = F.flight_shield(_corridor_policy(), None)
    drone = FakeDrone(0.0, 0.0, 15.0)
    rec = _descend(drone, sh)
    assert rec["floor_set_aside"] == ["route"], rec["floor_set_aside"]
    assert rec["outcome"] == "cleared" and rec["verdicts"]["floor"] > 0, rec
    assert rec["verdicts"]["refused"] == 0, rec["refused_by"]
    st = State(x=0.0, y=0.0, up=11.0, yaw_deg=0.0)                # inside
    assert F.descent_verdict(sh, sh.filter(st, Action4D(vz_up=-1.5)), st)[0] == "floor"
    st2 = State(x=0.0, y=14.0, up=15.0, yaw_deg=0.0)              # 4 m off its side
    verdict, why = F.descent_verdict(sh, sh.filter(st2, Action4D(vz_up=-1.5)), st2)
    assert verdict == "refused" and "route" in why, (verdict, why)


def test_a_replan_that_did_not_reach_its_site_is_not_descended():
    """The re-plan transit timed out (reached False) beside the refused
    place: no descent there; that place is excluded and the next site is
    tried. A first transit that did not arrive is treated the same."""
    sh, _calls = _shield(_policy())
    drone = FakeDrone(0.0, 0.0, 8.0)
    asked = []

    async def fly_to_site(exclude):
        asked.append(list(exclude))
        if len(asked) == 1:                              # held near the refused spot
            drone.x, drone.y = 1.5, 0.0
            return {"site": [30.0, 0.0], "reached": False, "final": [1.5, 0.0],
                    "final_landable": True}
        drone.x, drone.y = 30.0, 0.0
        return {"site": [30.0, 0.0], "reached": True, "final": [30.0, 0.0],
                "final_landable": True}

    n0 = len(drone.cmds)
    rec, cleared = asyncio.run(F._land(drone, sh, _Args(), fly_to_site=fly_to_site,
                                       sleep_s=0.0))
    assert cleared, rec
    assert [a["outcome"] for a in rec["attempts"]] == \
        ["column_blocked", "site_unusable", "cleared"], rec["attempts"]
    assert asked == [[(0.0, 0.0)], [(0.0, 0.0), (1.5, 0.0)]], asked
    assert len(drone.cmds) > n0
    # a stop beside a refused place, even when "reached", is not descended
    assert F._site_unusable({"site": [1, 0], "reached": True, "final": [1.0, 0.0]},
                            [(0.0, 0.0)])
    assert F._site_unusable({"site": [1, 0], "reached": True, "final": [9.0, 0.0],
                             "final_landable": False, "final_reasons": ["car_lane"]}, [])
    assert F._site_unusable({"site": [9, 0], "reached": True, "final": [9.0, 0.0]},
                            [(0.0, 0.0)]) is None
    # the first transit, unreached: not descended either
    d2 = FakeDrone(5.0, 0.0, 8.0)
    rec2, cleared2 = asyncio.run(F._land(
        d2, sh, _Args(), first_site={"site": [30.0, 0.0], "reached": False,
                                     "final": [5.0, 0.0]},
        fly_to_site=None, sleep_s=0.0))
    assert not cleared2 and rec2["attempts"][0]["outcome"] == "site_unusable"
    assert d2.cmds == []


def test_the_transit_climbs_to_cruise_before_it_moves_sideways():
    """A re-plan starts where a refused descent stopped, as low as the last
    metre: the transit to the next site now climbs straight up through the
    Shield first, and moves sideways only within LAND_CLIMB_TOL_M of
    cruise. (The real _fly_to_landing_site, its map reads stubbed.)"""
    import landing as L
    saved = (L.load_landing_maps, L.choose_site, L.is_landable)
    L.load_landing_maps = lambda *a, **k: object()
    L.choose_site = lambda maps, x0, y0, **k: {
        "site": (x0 + 10.0, y0), "here_landable": False, "reasons_rejected_here": [],
        "path_m": 10.0, "crosses_building": False, "lanes_checked": 0, "excluded": []}
    L.is_landable = lambda maps, x, y: (True, [])
    try:
        sh, _calls = _shield(_policy(low_zone=False))
        drone = FakeDrone(30.0, 0.0, 1.2)
        info = asyncio.run(F._fly_to_landing_site(drone, sh, _Args(), exclude=[(30.0, 0.0)],
                                                  sleep_s=0.0))
    finally:
        L.load_landing_maps, L.choose_site, L.is_landable = saved
    assert info["reached"] and info["climb_ticks"] > 0, info
    low = _Args.cruise_alt - F.LAND_CLIMB_TOL_M
    sideways = [(c, u) for c, u in zip(drone.cmds, drone.ups) if math.hypot(c[0], c[1]) > 1e-9]
    assert sideways and all(u >= low - 1e-9 for _c, u in sideways), sideways[:3]
    first = drone.cmds[0]
    assert first[0] == 0.0 and first[1] == 0.0 and first[2] < 0.0, first   # up first


class _Walker:
    """Client figures (pedestrians.Pedestrians duck-typed): update(t) moves
    them; here one walks under the landing column on its third update."""

    def __init__(self):
        self.figures = [_Figure(60.0, 0.0)]
        self.figures[0]._failed = False
        self.n = 0

    def update(self, t):
        self.n += 1
        if self.n >= 3:
            self.figures[0].x, self.figures[0].y = 30.0, 0.5


def test_the_people_rule_judges_people_fed_during_the_descent():
    """A distance_envelope over people (5 m) and a pedestrian who walks
    under the column after the descent began. The Shield is fed the
    mission's feed on every descent tick (LandingPeopleFeed), so it refuses;
    at f2ea619 it judged the last mission tick's list and never saw them."""
    import time
    from guardrail import people as P
    pol = _policy(low_zone=False)
    pol = Policy.model_validate({**pol.model_dump(mode="json"), "constraints": [
        *[c.model_dump(mode="json") for c in pol.constraints],
        {"id": "de-people", "type": "distance_envelope", **HARD_P0,
         "object_class": "people", "min_distance_m": 5.0}]})
    mark_legacy_read(pol)                        # laid out in local metres (WP1-14)
    sh = F.flight_shield(pol, None, people_source="test: a walker")
    walker = _Walker()
    feed = F.LandingPeopleFeed(sh, walker, P.PeopleMemory(), 50.0, time.time())
    drone = FakeDrone(30.0, 0.0, 8.0)
    rec = _descend(drone, sh, feed_people=feed)
    assert rec["column_at_start"] == [], rec                      # nobody near yet
    assert rec["outcome"] == "refused" and "de-people" in rec["refused_by"], rec
    # (the jump reads as a second person to the memory until the first goes stale)
    assert feed.stats()["ticks_fed"] >= 3 and feed.stats()["max_people_given"] >= 1
    assert drone.up > F.LAND_TOUCHDOWN_M


def test_the_flight_loop_lands_only_through_land():
    """fly() reaches land_async only when _land cleared the last metre: the
    old open-loop descent (straight to move_by_velocity_async) is gone."""
    src = (ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8")
    tail = src[src.index("async def fly(args)"):]
    assert "await _land(" in tail
    i = tail.index("await (await drone.land_async())")
    assert "if _cleared:" in tail[i - 200:i], tail[i - 200:i]
    assert "max(0.6, min(2.0, up * 0.15))" not in tail
    assert "Shield not in this loop)" not in tail


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
