"""The hand-over to the autopilot, made answerable to P0: guardrail/rtl_path.py
and the harness's RTL -> Land edge (G6).

    python tests/test_harnesstruth_rtl.py      (no pytest needed)

WHY THIS FILE EXISTS

The escalation FSM's RTL -> Land edge, "RTL fails (battery / blocked path)"
(Safety Shield p4, G6), takes `rtl_failed` from the autopilot side. Until
2026-10-12 nothing computed it: the headless harness passed `home_reached`
and `landed` only, so G6 could never fire and ArduPilot's straight-line RTL
was the end of every fail-safe, whatever lay between the aircraft and home.
The published library sweep of 2026-10-12 counted 680 P0-violating ticks
flown by the modelled autopilot in 10 episodes, with no per-mode label, so a
reader could not tell a LOITER hold from an RTL leg from a landing.

Shown failing first: every test below fails or errors when this file is run
from a pristine copy of HEAD 4418ae0 (`git archive HEAD | tar -x`, this file
copied into its tests/): guardrail/rtl_path.py does not exist there, the
harness never passes rtl_failed, Stress has no battery hook, and no episode
reports its autopilot P0 ticks per mode.

The review of 2026-10-12 added the leg-traversal regression (an RTL across
the zone it starts in read clear), and tests for every branch its mutants
got past: the no-progress half of G6 through the harness, a stall after
progress, the climb counted as progress, reset(), the true position under a
frozen fix, the altitude band through the harness, the climb against a
zone's band, and the rule filters (P1, soft, monitor_only, out of force,
the tightest band).
"""
from __future__ import annotations

import copy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

SKIP = "SKIP"


def _rp():
    from guardrail import rtl_path
    return rtl_path


def _square(x0, y0, x1, y1):
    from shapely.geometry import box
    return box(x0, y0, x1, y1)


def _S():
    import sweep_scenarios as S
    return S


def _lib():
    from guardrail.scenario_spec import Library
    return Library(ROOT / "experiments" / "scenarios.yaml")


# A blocked RTL, written inline so the test does not lean on the template it
# guards: the aircraft flies north from (-200, 0); at 4 s a zone appears
# BEHIND it, between it and home; at 5 s a zone whose breach action is RTL
# appears ahead, so the FSM goes straight to RTL (G11) with the way home
# blocked.
BLOCKED = {
    "scenario_id": "rtl-blocked-test", "template": "failsafe_escalation",
    "description": "test", "ticks": 420,
    "mission": {"start_pose": {"x": -200.0, "y": 0.0, "up": 15.0},
                "target": {"x": -80.0, "y": 0.0, "up": 15.0},
                "pilot": {"type": "goto", "speed": 4.0}},
    "policy": {"path": "policies/sim_demo_policy.yaml"},
    "events": [
        {"at_sim_t": 4.0, "type": "spawn_polygon_fence",
         "payload": {"id": "home-side", "vertices": [
             {"x": -196.0, "y": -10.0}, {"x": -190.0, "y": -10.0},
             {"x": -190.0, "y": 10.0}, {"x": -196.0, "y": 10.0}]}},
        {"at_sim_t": 5.0, "type": "spawn_polygon_fence",
         "payload": {"id": "rtl-zone", "width_m": 30, "height_m": 20,
                     "ahead_of_vehicle_m": 12, "violation_action": "RTL"}}],
    "expected_kpis": {"P0_escape_rate": 0},
}


def _edges(run) -> str:
    return ">".join(t["edge"] for t in ((run.extra.get("fsm") or {})
                                        .get("transitions") or []))


# --------------------------------------------------------------------------- #
# guardrail/rtl_path.py on its own
# --------------------------------------------------------------------------- #

def test_a_zone_between_the_aircraft_and_home_blocks_the_leg():
    RP = _rp()
    z = RP.Zone("z", _square(-10, -5, -5, 5), 0.0, 100.0)
    blocked = RP.check_leg(0.0, 0.0, 15.0, (-20.0, 0.0), [z])
    assert not blocked.clear and blocked.blocking == ("z",), blocked
    assert "crosses z" in blocked.reasons[0], blocked.reasons
    # The null: the same zone off the line home blocks nothing.
    side = RP.Zone("z", _square(-10, 6, -5, 10), 0.0, 100.0)
    assert RP.check_leg(0.0, 0.0, 15.0, (-20.0, 0.0), [side]).clear


def test_the_leg_is_judged_at_rtl_altitude_and_at_the_current_one_above_it():
    """ArduCopter climbs to RTL_ALT (15 m) and returns there, or returns at its
    current altitude when above RTL_ALT. A zone banded 0-12 m is under a 15 m
    leg; a zone banded 20-40 m is crossed by a leg flown from 25 m."""
    RP = _rp()
    low = RP.Zone("low", _square(-10, -5, -5, 5), 0.0, 12.0)
    assert RP.check_leg(0.0, 0.0, 5.0, (-20.0, 0.0), [low]).clear
    high = RP.Zone("high", _square(-10, -5, -5, 5), 20.0, 40.0)
    assert RP.check_leg(0.0, 0.0, 15.0, (-20.0, 0.0), [high]).clear
    leg = RP.check_leg(0.0, 0.0, 25.0, (-20.0, 0.0), [high])
    assert not leg.clear and abs(leg.rtl_alt_m - 25.0) < 1e-9, leg
    assert abs(RP.rtl_altitude(3.0) - RP.RTL_ALT_M) < 1e-9 and RP.RTL_ALT_M == 15.0


def test_the_climb_into_a_zone_above_blocks_and_leaving_one_does_not():
    RP = _rp()
    above = RP.Zone("above", _square(-2, -2, 2, 2), 10.0, 30.0)
    climb = RP.check_leg(0.0, 0.0, 3.0, (-20.0, 0.0), [above])
    assert not climb.clear and "climb" in climb.reasons[0], climb
    # Inside a zone already: the leg out is the exit, not a block.
    around = RP.Zone("around", _square(-3, -3, 3, 3), 0.0, 100.0)
    out = RP.check_leg(0.0, 0.0, 15.0, (-20.0, 0.0), [around])
    assert out.clear and out.starts_inside == ("around",), out
    # A U-shaped zone open to the west: from just inside the base's west
    # face the line home leaves at once and runs out between the arms.
    from shapely.geometry import Polygon
    u = Polygon([(-30, -5), (3, -5), (3, 5), (-30, 5), (-30, 3), (-12, 3),
                 (-12, -3), (-30, -3)]).buffer(0)
    gap = RP.check_leg(-11.0, 0.0, 15.0, (-40.0, 0.0), [RP.Zone("u", u, 0.0, 100.0)])
    assert gap.starts_inside == ("u",) and gap.clear, gap
    # From (0, 0), 3 m from the base's east face, the same line runs 12 m
    # through the base, deeper first (to 5 m from every edge): not an exit.
    # (Until the review of 2026-10-12 any piece starting at the aircraft was
    # exempt, and this read clear.)
    deep = RP.check_leg(0.0, 0.0, 15.0, (-40.0, 0.0), [RP.Zone("u", u, 0.0, 100.0)])
    assert not deep.clear and "crosses u (12.0 m" in deep.reasons[0], deep
    # ... and a leg that leaves a zone and comes back in is blocked too.
    blocked_u = Polygon([(-30, -5), (3, -5), (3, 5), (-30, 5)])
    hole = blocked_u.difference(_square(-20, -2, -10, 2))
    # From (-9, 0), 1 m east of the hole, the leg leaves into the hole at
    # once and re-enters the zone at its west edge: 10 m inside, blocked.
    again = RP.check_leg(-9.0, 0.0, 15.0, (-40.0, 0.0), [RP.Zone("h", hole, 0.0, 100.0)])
    assert again.starts_inside == ("h",) and not again.clear, again
    assert "crosses h (10.0 m" in again.reasons[0], again


def test_a_leg_that_crosses_the_zone_it_starts_in_is_blocked():
    """REGRESSION (review of 2026-10-12). check_leg exempted the whole first
    piece of a leg that starts inside a zone's ring, however far it ran
    through the zone, so an RTL across a 100 m zone from just inside its far
    face read clear. Only an EXIT is exempt now: the piece from the aircraft
    to the zone's edge, never deeper than the aircraft already is, and not
    ending at a home inside the zone. Fails on the builder's check_leg
    (95 m traversal: clear) and on HEAD (no module)."""
    RP = _rp()
    big = [RP.Zone("big", _square(-50, -50, 50, 50), 0.0, 100.0)]
    across = RP.check_leg(45.0, 0.0, 15.0, (-100.0, 0.0), big)
    assert not across.clear and across.starts_inside == ("big",), across
    assert "crosses big (95.0 m" in across.reasons[0], across.reasons
    # The null: the real exit, 5 m out through the near face, and an
    # oblique one through the same face, are clear.
    assert RP.check_leg(45.0, 0.0, 15.0, (100.0, 0.0), big).clear
    assert RP.check_leg(45.0, 0.0, 15.0, (100.0, 80.0), big).clear
    # From the centre every straight line is an exit (depth falls at once).
    assert RP.check_leg(0.0, 0.0, 15.0, (-100.0, 0.0), big).clear
    # A home inside the zone is never reached by leaving it: blocked even
    # though the depth falls all the way (45 m -> 47 m, 5 m -> 3 m deep).
    inside = RP.check_leg(45.0, 0.0, 15.0, (47.0, 0.0), big)
    assert not inside.clear and "home is inside it" in inside.reasons[0], inside


def test_the_climb_is_judged_against_the_zones_altitude_band():
    """The climb to RTL_ALT is a vertical segment: a zone over the aircraft
    blocks it only where the segment meets the zone's band. Banded 20-40 m,
    a climb from 3 m to 15 m stays under it; banded 0-2 m, the aircraft at
    3 m is already above it. Banded 10-30 m, the climb enters it."""
    RP = _rp()
    over = _square(-2, -2, 2, 2)
    for floor, ceil, clear in ((20.0, 40.0, True), (0.0, 2.0, True), (10.0, 30.0, False)):
        leg = RP.check_leg(0.0, 0.0, 3.0, (-20.0, 0.0), [RP.Zone("z", over, floor, ceil)])
        assert leg.clear is clear, (floor, ceil, leg)


def test_remaining_distance_counts_the_climb_still_to_do():
    """`remaining` is the climb to the return altitude plus the horizontal
    distance home, so a climb out from low altitude is progress (without the
    climb an RTL climbing for 5 s from the ground would read 'no progress')."""
    RP = _rp()
    assert RP.remaining_to_home(3.0, 4.0, 5.0, (0.0, 0.0)) == 15.0     # 10 up + 5 across
    assert RP.remaining_to_home(3.0, 4.0, 25.0, (0.0, 0.0)) == 5.0     # above RTL_ALT: none
    w = RP.RTLWatch(home=(0.0, 0.0))
    # Climbing at 2.5 m/s straight up over a point 30 m out: never stuck.
    got = [w.assess(0.1 * k, 30.0, 0.0, min(15.0, 0.25 * k), []).no_progress
           for k in range(61)]
    assert not any(got), got.index(True)


def test_progress_that_stalls_fails_one_window_after_the_stall():
    """The window slides: 10 s of progress home and then a stall fails 5 s
    after the stall (at 15.0 s), not never (a window anchored at the RTL's
    first tick would still see the early progress). And the threshold is
    strict: exactly the minimum rate is enough."""
    RP = _rp()
    mon = RP.ProgressMonitor(window_s=5.0, min_rate_mps=0.1)
    first = None
    for k in range(201):
        t = 0.1 * k
        rem = 100.0 - 5.0 * min(t, 10.0)
        failed, _ = mon.update(t, rem)
        if failed and first is None:
            first = round(t, 1)
    assert first is not None and abs(first - 15.0) <= 0.1 + 1e-9, first
    # Exactly 0.25 m/s over 4 s (binary-exact numbers) is not a failure.
    mon = RP.ProgressMonitor(window_s=4.0, min_rate_mps=0.25)
    got = [mon.update(0.5 * k, 10.0 - 0.125 * k)[0] for k in range(9)]
    assert got == [False] * 9, got
    got = mon.update(4.5, 9.0)                     # and then no progress at all
    assert got[0], got


def test_rtl_watch_reset_restarts_the_progress_window():
    """reset() forgets the window, so a later RTL is judged from its own
    first tick. (The harness calls it on every tick out of RTL; the FSM never
    leaves RTL for a non-terminal state today, so there it is defensive.)"""
    RP = _rp()
    w = RP.RTLWatch(home=(0.0, 0.0))
    for k in range(50):
        w.assess(0.1 * k, 40.0, 0.0, 15.0, [])
    assert w.assess(5.0, 40.0, 0.0, 15.0, []).no_progress
    w.reset()
    assert not w.assess(5.1, 40.0, 0.0, 15.0, []).no_progress
    assert not w.assess(10.0, 40.0, 0.0, 15.0, []).no_progress
    assert w.assess(10.1, 40.0, 0.0, 15.0, []).no_progress


_POLICY_MIXED = """\
policy_id: rtl-path-test
version: 0.1.0
constraints:
  - id: hard-p0
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: repair
    vertices: [{x: 10, y: -5}, {x: 20, y: -5}, {x: 20, y: 5}, {x: 10, y: 5}]
    altitude_floor_m: 0.0
    altitude_ceiling_m: 100.0
    margin_m: 1.0
  - id: p1-zone
    type: polygon_fence
    constraint_type: hard
    priority: P1
    violation_action: repair
    vertices: [{x: 30, y: -5}, {x: 40, y: -5}, {x: 40, y: 5}, {x: 30, y: 5}]
    altitude_floor_m: 0.0
    altitude_ceiling_m: 100.0
  - id: soft-zone
    type: polygon_fence
    constraint_type: soft
    priority: P0
    violation_action: monitor_only
    vertices: [{x: 50, y: -5}, {x: 60, y: -5}, {x: 60, y: 5}, {x: 50, y: 5}]
    altitude_floor_m: 0.0
    altitude_ceiling_m: 100.0
  - id: watched-zone
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: monitor_only
    vertices: [{x: 70, y: -5}, {x: 80, y: -5}, {x: 80, y: 5}, {x: 70, y: 5}]
    altitude_floor_m: 0.0
    altitude_ceiling_m: 100.0
  - id: band-night
    type: altitude_envelope
    constraint_type: hard
    priority: P0
    violation_action: repair
    alt_min_m: 12.0
    alt_max_m: 14.0
    valid_time:
      recurrence:
        days: [Mon, Tue, Wed, Thu, Fri, Sat, Sun]
        start_time: "22:00"
        end_time: "23:00"
  - id: band-tight
    type: altitude_envelope
    constraint_type: hard
    priority: P0
    violation_action: repair
    alt_min_m: 10.0
    alt_max_m: 20.0
  - id: band-wide
    type: altitude_envelope
    constraint_type: hard
    priority: P0
    violation_action: repair
    alt_min_m: 5.0
    alt_max_m: 30.0
"""


def test_zones_and_band_skip_what_the_shield_does_not_enforce():
    """Only hard, enforced P0 rules can block the leg home: a P1 zone, a soft
    one and a monitor_only one cannot. The band is the TIGHTEST of the P0
    bands in force (their intersection, whatever their order), and a band
    out of its window does not count."""
    import tempfile
    from datetime import datetime
    RP = _rp()
    from guardrail import Shield, State, load_policy
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "p.yaml"
        p.write_text(_POLICY_MIXED, encoding="utf-8")
        pol = load_policy(p)
    noon = Shield(pol, now=lambda: datetime(2026, 9, 7, 12, 0, 0))
    assert [z.rule_id for z in RP.zones_from_shield(noon)] == ["hard-p0"]
    band = RP.band_from_shield(noon, State(x=-30, y=0, up=15.0))
    assert (band.alt_min_m, band.alt_max_m) == (10.0, 20.0), band
    assert set(band.rule_id.split("+")) == {"band-wide", "band-tight"}, band
    night = Shield(pol, now=lambda: datetime(2026, 9, 7, 22, 30, 0))
    band = RP.band_from_shield(night, State(x=-30, y=0, up=13.0))
    assert (band.alt_min_m, band.alt_max_m) == (12.0, 14.0), band


def _spec(d):
    from guardrail.scenario_spec import ScenarioSpec
    return ScenarioSpec.model_validate(d)


def test_an_rtl_into_a_wind_beyond_authority_fails_for_no_progress():
    """G6's second input, through the harness. The RTL zone sends the
    aircraft home into a 12 m/s headwind, which the closed loop cannot hold
    (5 m/s commanded home, 5.6 m/s of excess wind back): RTL fails one
    window after it starts, for no progress (G11 at 5.0 s, G6 at 10.1 s),
    and it lands. The nulls: in 8 m/s it gets home (G9), and on the
    open-loop model, where the modelled autopilot rejected every wind, the
    same 12 m/s RTL gets home too."""
    S = _S()
    lib = _lib()
    d = copy.deepcopy(BLOCKED)
    d["events"] = [d["events"][1]]
    d["stress"] = {"wind_speed_mps": 12.0, "wind_dir_deg": 0.0}
    run = S.run_episode(_spec(d), lib.defaults, 0)
    tr = [(t["t_s"], t["edge"]) for t in run.extra["fsm"]["transitions"]]
    assert [e for _, e in tr] == ["G11", "G6", "G10"], tr
    assert tr[1][0] - tr[0][0] <= RP_WINDOW + 0.15, tr
    why = " ".join(run.extra["rtl_failures"][0]["reasons"])
    assert "no progress home" in why and "blocked" not in why, why
    d8 = copy.deepcopy(d)
    d8["stress"]["wind_speed_mps"] = 8.0
    assert _edges(S.run_episode(_spec(d8), lib.defaults, 0)) == "G11>G9"
    ol = S.run_episode(_spec(d), lib.defaults, 0, wind_model="open_loop")
    assert _edges(ol) == "G11>G9", _edges(ol)


RP_WINDOW = 5.0


def test_rtl_is_judged_on_the_true_position_not_the_frozen_fix():
    """The RTL watch reads the autopilot's own position (the true one here:
    degraded perception is what the SHIELD is handed). A GPS dropout that
    freezes the Shield's fix through a 12 s RTL must not read as 'no
    progress home': the RTL reaches home (G9). Judged on the frozen fix it
    would fail one window into the dropout."""
    S = _S()
    lib = _lib()
    d = copy.deepcopy(BLOCKED)
    d["events"] = [dict(d["events"][1], at_sim_t=15.0)]
    d["stress"] = {"gps_dropout_s": [16.0, 40.0]}
    run = S.run_episode(_spec(d), lib.defaults, 0)
    assert run.extra["gps_dropout_ticks"] > 0
    tr = [(t["t_s"], t["edge"]) for t in run.extra["fsm"]["transitions"]]
    assert [e for _, e in tr] == ["G11", "G9"], tr
    assert tr[1][0] - 16.0 > RP_WINDOW + 0.5, tr        # long enough to have failed
    assert run.extra["rtl_failed_ticks"] == 0


def test_an_rtl_above_the_ceiling_is_refused_by_the_band():
    """ArduCopter's RTL entered above RTL_ALT returns at its current
    altitude: from 30 m that is above sim_demo's 10-20 m band, so the leg is
    refused (G6) with the band named, on the RTL's first tick."""
    S = _S()
    lib = _lib()
    d = copy.deepcopy(BLOCKED)
    d["mission"]["start_pose"]["up"] = 30.0
    d["mission"]["target"]["up"] = 30.0
    d["events"] = [dict(d["events"][1], at_sim_t=0.5)]
    run = S.run_episode(_spec(d), lib.defaults, 0)
    tr = [(t["t_s"], t["edge"], t["to"]) for t in run.extra["fsm"]["transitions"]]
    rtl_in = [t for t, _, to in tr if to == "RTL"][0]
    g6 = [t for t, e, _ in tr if e == "G6"]
    assert g6 and abs(g6[0] - rtl_in - 0.1) < 1e-9, tr
    why = " ".join(run.extra["rtl_failures"][0]["reasons"])
    assert "alt-band" in why and "above RTL_ALT" in why, why


def test_a_return_altitude_outside_the_band_blocks():
    """The 30 m cell of altitude-recovery-depth: ArduCopter's RTL entered above
    RTL_ALT returns at its current altitude, above a 20 m ceiling."""
    RP = _rp()
    band = RP.Band("alt", 10.0, 20.0)
    assert RP.check_leg(0.0, 0.0, 15.0, (-20.0, 0.0), [], band).clear
    assert RP.check_leg(0.0, 0.0, 3.0, (-20.0, 0.0), [], band).clear, \
        "climbing out from under the floor to RTL_ALT is the recovery, not a block"
    hi = RP.check_leg(0.0, 0.0, 30.0, (-20.0, 0.0), [], band)
    assert not hi.clear and "above RTL_ALT" in hi.reasons[0], hi
    low_ceiling = RP.Band("alt", 2.0, 12.0)
    assert not RP.check_leg(0.0, 0.0, 5.0, (-20.0, 0.0), [], low_ceiling).clear


def test_progress_home_is_judged_over_a_full_window_only():
    RP = _rp()
    mon = RP.ProgressMonitor(window_s=5.0, min_rate_mps=0.1)
    # Standing still for 4.9 s is not yet a failure; at 5.0 s it is.
    for k in range(50):
        failed, _ = mon.update(0.1 * k, 40.0)
        assert not failed, k
    failed, why = mon.update(5.0, 40.0)
    assert failed and "no progress" in why, why
    # Flying home at 5 m/s never fails.
    mon = RP.ProgressMonitor()
    assert not any(mon.update(0.1 * k, 60.0 - 0.5 * k)[0] for k in range(100))
    # Drifting away fails.
    mon = RP.ProgressMonitor()
    got = [mon.update(0.1 * k, 20.0 + 0.2 * k)[0] for k in range(60)]
    assert got[49] is False and got[50] is True, got[45:55]


def test_the_battery_hook_reports_critical_from_its_time_or_its_reader():
    RP = _rp()
    b = RP.BatteryHook(critical_at_s=8.0)
    assert not b.critical(7.9) and b.critical(8.0) and b.critical(30.0)
    assert not RP.BatteryHook().critical(1e9)
    assert RP.BatteryHook(reader=lambda t: t > 2.0).critical(2.5)
    w = RP.RTLWatch(home=(0.0, 0.0), battery=RP.BatteryHook(critical_at_s=1.0))
    assert not w.assess(0.5, 10.0, 0.0, 15.0, []).failed
    v = w.assess(1.0, 9.5, 0.0, 15.0, [])
    assert v.failed and v.battery_critical and "battery" in v.note, v


def test_zones_and_band_come_from_the_shields_own_rules():
    """One rule-evaluation code path: the leg is judged on the Shield's own
    compiled zones (margin ring included) and the P0 band in force."""
    RP = _rp()
    from guardrail import Shield, State, load_policy
    sh = Shield(load_policy(ROOT / "policies" / "sim_demo_policy.yaml"))
    zones = RP.zones_from_shield(sh)
    assert [z.rule_id for z in zones] == ["nfz-square"], zones
    # The forbidden shape is the margin ring: 1 m outside the 7..23 square.
    assert zones[0].shape.contains(_square(6.5, 6.5, 6.6, 6.6))
    band = RP.band_from_shield(sh, State(x=-30, y=-30, up=15.0))
    assert band is not None and (band.alt_min_m, band.alt_max_m) == (10.0, 20.0), band


# --------------------------------------------------------------------------- #
# the harness: G6 fires, and the autopilot's P0 ticks are labelled by mode
# --------------------------------------------------------------------------- #

def test_a_blocked_rtl_goes_to_land_instead_of_flying_through_the_zone():
    """REGRESSION (G6 had no input). With the way home blocked, the FSM takes
    G11 (RTL), then G6 (RTL fails: blocked path), then G10 (landed), and the
    autopilot never enters the zone behind it. The null: with the RTL watch
    silenced the same episode's RTL flies straight through that zone."""
    S = _S()
    from guardrail.scenario_spec import ScenarioSpec
    lib = _lib()
    spec = ScenarioSpec.model_validate(copy.deepcopy(BLOCKED))
    run = S.run_episode(spec, lib.defaults, 0)
    assert _edges(run) == "G11>G6>G10", _edges(run)
    ex = run.extra
    assert ex["rtl_failed_ticks"] >= 1 and ex["rtl_failures"], ex["rtl_failures"]
    assert "home-side" in " ".join(ex["rtl_failures"][0]["reasons"]), ex["rtl_failures"]
    assert ex["autopilot_polygon_entries"] == 0, ex["autopilot_polygon_entries"]
    assert ex["fsm"]["outcome_label"] == "Land_triggered"
    rows = [r for r in run.rows if r.get("rtl_failed")]
    assert rows and "blocked path" in rows[0]["rtl_failed"], rows[:1]

    RP = _rp()
    real = RP.RTLWatch.assess

    def silent(self, *a, **k):
        v = real(self, *a, **k)
        return RP.RTLVerdict(False, (), v.leg, False, False)
    RP.RTLWatch.assess = silent
    try:
        null = S.run_episode(spec, lib.defaults, 0)
    finally:
        RP.RTLWatch.assess = real
    assert _edges(null) == "G11>G9", _edges(null)
    assert null.extra["autopilot_polygon_entries"] >= 1, null.extra["autopilot_polygon_entries"]
    assert null.extra["autopilot_p0_flown_ticks_by_mode"].get("RTL", 0) > 0, null.extra


def test_a_critical_battery_during_rtl_goes_to_land():
    """The battery fail-safe hook: `stress.battery_critical_at_s` lands an RTL
    in progress (G6). Before the battery goes critical the RTL flies; on a
    run where it never does, RTL reaches home (G9)."""
    S = _S()
    from guardrail.scenario_spec import ScenarioSpec
    lib = _lib()
    base = copy.deepcopy(BLOCKED)
    base["events"] = [base["events"][1]]
    base["events"][0]["at_sim_t"] = 10.0
    base["ticks"] = 500
    home = S.run_episode(ScenarioSpec.model_validate(base), lib.defaults, 0)
    assert _edges(home) == "G11>G9", _edges(home)
    t_rtl = [t["t_s"] for t in home.extra["fsm"]["transitions"] if t["edge"] == "G11"][0]
    t_home = [t["t_s"] for t in home.extra["fsm"]["transitions"] if t["edge"] == "G9"][0]
    mid = round((t_rtl + t_home) / 2, 1)
    low = copy.deepcopy(base)
    low["stress"] = {"battery_critical_at_s": mid}
    run = S.run_episode(ScenarioSpec.model_validate(low), lib.defaults, 0)
    assert _edges(run) == "G11>G6>G10", _edges(run)
    g6 = [t["t_s"] for t in run.extra["fsm"]["transitions"] if t["edge"] == "G6"][0]
    assert abs(g6 - mid) <= 0.1 + 1e-9, (g6, mid)
    assert "battery" in " ".join(run.extra["rtl_failures"][0]["reasons"])
    # A battery that would go critical after the run is refused, not ignored.
    late = copy.deepcopy(base)
    late["stress"] = {"battery_critical_at_s": 999.0}
    try:
        S.run_episode(ScenarioSpec.model_validate(late), lib.defaults, 0)
    except ValueError as e:
        assert "never happens" in str(e), e
    else:
        raise AssertionError("a battery that never goes critical was accepted")


def test_autopilot_p0_ticks_are_reported_per_mode_and_sum_to_the_total():
    """failsafe-land-zone's P0 ticks are the landing's (LAND); the RTL-blocked
    test episode's are LAND too, once its RTL is refused. The per-mode counts
    sum to autopilot_p0_flown_ticks, reach the run's true-state summary and
    the cell rollup, and a mode that flew no P0 tick is absent, not 0."""
    S = _S()
    lib = _lib()
    land = S.run_episode(lib.by_id()["failsafe-land-zone"].first.spec, lib.defaults, 0)
    ex = land.extra
    assert set(ex["autopilot_p0_flown_ticks_by_mode"]) == {"LAND"}, ex
    assert sum(ex["autopilot_p0_flown_ticks_by_mode"].values()) == ex["autopilot_p0_flown_ticks"] > 0
    item = {"status": "pass", "arm": "on",
            "kpi": S.score(lib.by_id()["failsafe-land-zone"].first.spec, land.rows, ex,
                           land.policy, land.priorities)}
    ts = S.true_state_summary([item, item])
    assert ts["autopilot_p0_flown_ticks_by_mode"] == {"LAND": 2 * ex["autopilot_p0_flown_ticks"]}
    assert ts["autopilot_ticks_by_mode"] == {"LAND": 2 * ex["autopilot_mode_ticks"]["LAND"]}
    row = S.cell_extras([dict(item["kpi"], forecast_arm="grant")])
    assert row["autopilot_p0_flown_ticks_by_mode"] == ex["autopilot_p0_flown_ticks_by_mode"]
    assert row["autopilot_p0_flown_ticks"] == ex["autopilot_p0_flown_ticks"]


WHY_MARKER = "WHY ITS LABEL ACCEPTS P0 TICKS UNDER THE AUTOPILOT"


def test_every_failsafe_cell_that_accepts_autopilot_p0_ticks_says_why():
    """A cell whose labelled outcome is accepted (pass or tracked) while the
    AUTOPILOT flew P0-violating actions (a landing through the altitude
    floor, a hold carried into a zone) must say why in its description:
    under the explicit marker, naming every autopilot mode that flew those
    ticks (LOITER, RTL, LAND). Until the review of 2026-10-12 the words
    "P0" and "autopilot" anywhere in the text were enough. Read from the
    published sweep, so it holds for what was flown."""
    import json
    S = _S()
    p = S.PUBLISHED_SWEEP
    if not p.is_file():
        print(f"      SKIP: {p} absent")
        return SKIP
    doc = json.loads(p.read_text(encoding="utf-8"))
    lib = _lib().by_id()
    silent = []
    for r in doc["results"]:
        k = r.get("kpi") or {}
        if (k.get("autopilot_p0_flown_ticks") or 0) == 0 or r["status"] not in ("pass", "tracked"):
            continue
        desc = lib[r["scenario_id"]].first.spec.description
        why = " ".join(desc.split(WHY_MARKER, 1)[1].split()) if WHY_MARKER in desc else ""
        modes = set(k.get("autopilot_p0_flown_ticks_by_mode") or {})
        unnamed = sorted(m for m in modes if m not in why)
        if not why or unnamed:
            silent.append(f"{r['id']} ({k['autopilot_p0_flown_ticks']} ticks, "
                          f"{k.get('autopilot_p0_flown_ticks_by_mode')}; "
                          f"{'no marker' if not why else 'unnamed ' + str(unnamed)})")
    assert not silent, silent
    assert any((r.get("kpi") or {}).get("autopilot_p0_flown_ticks_by_mode") is not None
               for r in doc["results"]), "the published sweep predates the per-mode count"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
