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
from guardrail.models import DynamicNFZ, Policy                  # noqa: E402

HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

TICK_S = 0.1


def _policy(low_zone=True):
    """The follow policies' altitude band (P0, 6-14 m) and kinematic caps,
    plus - where the column test must fail - a P0 zone from the ground to
    4 m round (0, 0): a place the aircraft may fly over but not land in."""
    cons = [
        {"id": "alt-band", "type": "altitude_envelope", **HARD_P0,
         "alt_min_m": 6, "alt_max_m": 14},
        {"id": "kin-caps", "type": "kinematic_envelope", **HARD_P0, "priority": "P1",
         "speed_max_mps": 5.0, "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0},
    ]
    if low_zone:
        cons.append({"id": "low-plaza", "type": "polygon_fence", **HARD_P0,
                     "vertices": [{"x": -5, "y": -5}, {"x": 5, "y": -5},
                                  {"x": 5, "y": 5}, {"x": -5, "y": 5}],
                     "altitude_floor_m": 0, "altitude_ceiling_m": 4, "margin_m": 1.0})
    # An origin, so a zone can be hot-applied mid-flight (WP1-14: a policy
    # states its WGS84 frame); the follow policies' home-geo-point.
    return Policy.model_validate({"policy_id": "landing-descent-test",
                                  "origin": {"lat": 35.6895, "lon": 139.6917},
                                  "constraints": cons})


class FakeDrone:
    """The simulator calls the landing makes, integrated kinematically: each
    velocity command is flown for one tick. Every command is kept."""

    def __init__(self, x, y, up, on_move=None):
        self.x, self.y, self.z = float(x), float(y), -float(up)
        self.cmds = []
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
        self.x += vn * TICK_S
        self.y += ve * TICK_S
        self.z += vd * TICK_S
        if self.on_move is not None:
            self.on_move(self)


class _Args:
    cruise_alt = 8.0


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
            sh.hot_apply(DynamicNFZ.model_validate({
                "id": "spawned-plaza", "type": "dynamic_nfz", **HARD_P0, "margin_m": 1.0,
                "altitude_floor_m": 0.0, "altitude_ceiling_m": 3.0,
                "vertices": [{"x": 25, "y": -5}, {"x": 35, "y": -5},
                             {"x": 35, "y": 5}, {"x": 25, "y": 5}]}))
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
