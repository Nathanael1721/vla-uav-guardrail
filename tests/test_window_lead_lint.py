"""The DSL's first lint WARNING class (guardrail/compiler.py,
time_window_lead_warnings): a time window that can open on an aircraft that
cannot leave before it does.

Run either way:
    pytest tests/test_window_lead_lint.py -v
    python tests/test_window_lead_lint.py

WHY THIS FILE EXISTS

The library's only P0 escape ticks until 2026-10-21 (6, in the three
time-window-opens-on-vehicle cells) were a window opening over an aircraft
10 m from the school zone's ring - 2.5 s at the 4 m/s cap - 1.0-2.0 s after
the mission starts. No action clears the forecast on the first tick, so the
Shield flew the way out with the violation on and its counter scored an
escape; no repair and no hold point could remove that tick. Since C2 the
Shield judges that way out as the recovery it is when it can prove the
opening imposed (Shield._opening_imposed, tests/test_window_opens_on.py),
but the position is still made illegal under the aircraft, which only the
policy or the mission's timing can prevent, so the DSL says so up front - as
a warning, never a refusal (the grant leaves
"Lint severity classes. Errors only, or warnings vs errors?" open; Policy.lint()
stays errors-only). These tests pin both levels of the test, the speed
assumption every warning states, the keep-in corridor and the always-on zones
that bound the way out, and that nothing is refused.
"""
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

from guardrail import load_policy  # noqa: E402
from guardrail.compiler import (LINT_SPEED_MPS, ConstraintCompiler,  # noqa: E402
                                Mission, time_window_lead_warnings)
from guardrail.models import Policy  # noqa: E402

SURVEY = ROOT / "policies" / "corridor_survey.yaml"
MON_0729_58 = datetime(2026, 9, 7, 7, 29, 58)     # a Monday, 2 s before 07:30


def _zone(zid, half_w, half_h, *, window=True, cx=0.0, cy=0.0, margin=1.0):
    z = {"id": zid, "type": "polygon_fence", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": margin,
         "vertices": [{"x": cx - half_w, "y": cy - half_h}, {"x": cx + half_w, "y": cy - half_h},
                      {"x": cx + half_w, "y": cy + half_h}, {"x": cx - half_w, "y": cy + half_h}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 100}
    if window:
        z["valid_time"] = {"recurrence": {"days": ["Mon", "Tue", "Wed", "Thu", "Fri"],
                                          "start_time": "08:00", "end_time": "17:00"}}
    return z


def _cap(speed):
    return {"id": "caps", "type": "kinematic_envelope", "constraint_type": "hard",
            "priority": "P1", "violation_action": "repair", "speed_max_mps": speed,
            "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}


def _pol(*rules):
    return Policy.model_validate({"policy_id": "lint-test", "constraints": list(rules)})


def test_the_survey_policy_warns_only_at_a_lookahead_shorter_than_its_fly_out():
    """nfz-school's ring is 22 x 20 m: 10 m from the middle to the nearest
    exit, 2.5 s at the policy's 4 m/s cap. The grant's 5 s forecast sees an
    opening in time; a 2 s one would not."""
    pol = load_policy(SURVEY)
    assert time_window_lead_warnings(pol) == []
    ws = time_window_lead_warnings(pol, lookahead_s=2.0)
    assert len(ws) == 1 and ws[0].startswith("nfz-school:"), ws
    assert "4 m/s, the policy's hard speed cap" in ws[0] and "2 s" in ws[0], ws[0]
    assert "a warning, not a refusal" in ws[0]


def test_the_opens_on_vehicle_start_is_warned_at_compile_time():
    """The library cell, as the compiler sees it at mission start: inside the
    ring at (60, 21), the window 2.0 s away, 2.5 s to leave."""
    pol = load_policy(SURVEY)
    ws = time_window_lead_warnings(pol, start=(60.0, 21.0, 15.0), now=MON_0729_58)
    assert len(ws) == 1 and "2.00 s before its time window opens" in ws[0], ws
    assert "takes 2.5 s (10.0 m at 4 m/s" in ws[0], ws[0]
    # Ten seconds of warning is enough; so is a start outside the ring; a
    # start inside a zone already in force is start_conflicts' case.
    assert time_window_lead_warnings(pol, start=(60.0, 21.0, 15.0),
                                     now=datetime(2026, 9, 7, 7, 29, 50)) == []
    assert time_window_lead_warnings(pol, start=(60.0, 40.0, 15.0), now=MON_0729_58) == []
    assert time_window_lead_warnings(pol, start=(60.0, 21.0, 15.0),
                                     now=datetime(2026, 9, 7, 9, 0, 0)) == []
    # The compiler's own entry point, from a Mission (start at cruise height).
    m = Mission(task_text="survey", target_x=60.0, target_y=55.0, cruise_alt_m=15.0,
                speed_pref_mps=4.0, start_x=60.0, start_y=21.0)
    assert ConstraintCompiler(pol).lint_warnings(m, MON_0729_58) == ws


def test_every_warning_states_its_speed_assumption():
    big = _zone("big", 25.0, 25.0)              # 52 m ring: 26 m to leave
    assert time_window_lead_warnings(_pol(big, _cap(4.0)))[0].count("4 m/s, the policy's") == 1
    w = time_window_lead_warnings(_pol(big))[0]
    assert f"{LINT_SPEED_MPS:g} m/s assumed: the policy has no hard speed cap" in w, w
    w = time_window_lead_warnings(_pol(big, _cap(4.0)), speed_mps=2.0)[0]
    took = float(w.split("leaving takes ")[1].split(" s")[0])
    assert "2 m/s as given" in w and 12.9 <= took <= 13.0, w
    # Fast enough, no warning: 26 m at 6 m/s is 4.3 s < 5 s.
    assert time_window_lead_warnings(_pol(big, _cap(4.0)), speed_mps=6.0) == []
    try:
        time_window_lead_warnings(_pol(big), speed_mps=0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("a zero speed was accepted")


def test_an_untimed_zone_is_not_linted():
    assert time_window_lead_warnings(_pol(_zone("static", 40.0, 40.0, window=False))) == []


def test_a_keep_in_corridor_bounds_where_the_aircraft_can_be_and_go():
    """A 60 m square zone across a 20 m wide corridor: the aircraft can only
    be inside the corridor, and can only leave along it, 31 m (ring edge at
    x = +/-31) - never by a side exit into the airspace the corridor forbids."""
    corridor = {"id": "lane", "type": "corridor", "constraint_type": "hard",
                "priority": "P0", "violation_action": "repair",
                "centerline": [{"x": -100.0, "y": 0.0}, {"x": 100.0, "y": 0.0}],
                "width_m": 20.0, "altitude_floor_m": 0.0, "altitude_ceiling_m": 100.0}
    zone = _zone("wide", 30.0, 30.0)
    ws = time_window_lead_warnings(_pol(zone, corridor, _cap(4.0)))
    assert len(ws) == 1, ws
    depth = float(ws[0].split("nearest legal exit is ")[1].split(" m")[0])
    assert 30.5 <= depth <= 31.0, ws[0]
    y = float(ws[0].split("aircraft at (")[1].split(")")[0].split(",")[1])
    assert abs(y) <= 10.0, f"the deepest point is outside the corridor: {ws[0]}"
    # Narrow the zone ALONG the corridor to 6 m: 4 m to leave along it, even
    # though its side is 31 m away - inside the 5 s.
    thin = _zone("thin", 3.0, 30.0)
    assert time_window_lead_warnings(_pol(thin, corridor, _cap(4.0))) == []


def test_a_zone_that_is_always_in_force_is_no_way_out():
    """Leaving the timed zone into an always-on one is not leaving. A strip
    6 m wide (ring) and 22 m long: 3 m to leave sideways, inside a 4 s
    lookahead at 1 m/s. An always-on wall along its whole east side leaves
    only the west side (6 m from the east edge) or the ends (11 m): over 4 s."""
    timed = _zone("timed", 2.0, 10.0)
    wall = _zone("wall", 3.0, 40.0, window=False, cx=7.0)
    assert time_window_lead_warnings(_pol(timed, _cap(1.0)), lookahead_s=4.0) == []
    walled = time_window_lead_warnings(_pol(timed, wall, _cap(1.0)), lookahead_s=4.0)
    assert len(walled) == 1, walled
    depth = float(walled[0].split("nearest legal exit is ")[1].split(" m")[0])
    assert 5.5 <= depth <= 6.0, walled[0]


def test_a_zone_with_an_empty_valid_time_is_a_wall_too():
    """models.ValidTime documents `recurrence` absent as "always", so a zone
    written `valid_time: {}` is always in force. Until 2026-10-20 only an
    absent `valid_time` counted as a wall (review of T1c, P5): the strip of
    the test above, walled by such a zone, read 3 m to leave (east, into the
    wall) and raised no warning."""
    timed = _zone("timed", 2.0, 10.0)
    wall = _zone("wall", 3.0, 40.0, window=False, cx=7.0)
    wall["valid_time"] = {}
    pol = _pol(timed, wall, _cap(1.0))
    assert pol.constraints[1].valid_time is not None
    assert pol.constraints[1].valid_time.recurrence is None
    walled = time_window_lead_warnings(pol, lookahead_s=4.0)
    assert len(walled) == 1, walled
    depth = float(walled[0].split("nearest legal exit is ")[1].split(" m")[0])
    assert 5.5 <= depth <= 6.0, walled[0]


def test_a_start_with_no_legal_exit_is_said_so_without_stepping_a_week():
    """Review of T1c, P4: a 100 x 100 m zone in force Mondays 08:00-17:00,
    a keep-in corridor wholly inside its ring, the mission starting at
    (0, 0, 15) on a Tuesday at 09:30. There is no legal exit, so the old
    mission-level test stepped the clock every 0.05 s for up to a week
    (10.26 million reads, 12.6 s for this one call, which the sweep makes
    per episode) and reported the opening 513000 s later as if it were a
    lead-time problem. Now: a 'no legal exit' warning, and the next opening
    found by reading the clock at minute boundaries only."""
    from guardrail import compiler as C
    zone = _zone("monday", 50.0, 50.0)
    zone["valid_time"] = {"recurrence": {"days": ["Mon"], "start_time": "08:00",
                                         "end_time": "17:00"}}
    corridor = {"id": "lane", "type": "corridor", "constraint_type": "hard",
                "priority": "P0", "violation_action": "repair",
                "centerline": [{"x": -20.0, "y": 0.0}, {"x": 20.0, "y": 0.0}],
                "width_m": 10.0, "altitude_floor_m": 0.0, "altitude_ceiling_m": 100.0}
    pol = _pol(zone, corridor)
    reads = [0]
    orig = C._RuleClock.on

    def counted(self, rule, x):
        reads[0] += 1
        if reads[0] > 50_000:
            raise AssertionError("the mission-level test is stepping the clock "
                                 "(over 50000 reads)")
        return orig(self, rule, x)
    C._RuleClock.on = counted
    try:
        ws = time_window_lead_warnings(pol, start=(0.0, 0.0, 15.0),
                                       now=datetime(2026, 9, 8, 9, 30))   # a Tuesday
    finally:
        C._RuleClock.on = orig
    mission = [w for w in ws if "the mission starts at" in w]
    assert len(mission) == 1, ws
    assert "no legal exit from there at all" in mission[0], mission[0]
    assert "before its time window opens" not in mission[0], mission[0]
    assert "next in 513000 s, at 2026-09-14T08:00" in mission[0], mission[0]
    # The policy-level warning gives no "inf s" either.
    policy = [w for w in ws if "its time window can open" in w]
    assert len(policy) == 1 and "inf s" not in policy[0], policy
    assert "no legal exit at all" in policy[0], policy[0]
    # Re-review of T1c, P4: the speed clause read "it cannot leave at 4 m/s
    # assumed: the policy has no hard speed cap or any other speed".
    assert ("it cannot leave at any speed (planned at 4 m/s assumed: the policy "
            "has no hard speed cap), so no forecast") in policy[0], policy[0]
    assert "or any other speed" not in policy[0], policy[0]


def test_the_next_opening_is_exact_at_the_minute_it_opens():
    """The minute-boundary search gives the same leads the 0.05 s grid gave
    on the library cell (2.0, 1.5 and 1.0 s), exactly, and nothing past the
    time it takes to leave (2.5 s from (60, 21))."""
    pol = load_policy(SURVEY)
    for sec, lead in ((58, "2.00"), (58.5, "1.50"), (59, "1.00")):
        now = datetime(2026, 9, 7, 7, 29, int(sec), int(round((sec % 1) * 1e6)))
        ws = time_window_lead_warnings(pol, start=(60.0, 21.0, 15.0), now=now)
        assert len(ws) == 1 and f"{lead} s before its time window opens" in ws[0], ws
    assert time_window_lead_warnings(pol, start=(60.0, 21.0, 15.0),
                                     now=datetime(2026, 9, 7, 7, 29, 57, 400000)) == []


def test_the_warning_says_what_the_shield_makes_of_the_opening():
    """Since C2 (2026-10-21) the Shield judges the way out of a window
    opening on the aircraft as the recovery it is when it can prove the
    opening imposed (Shield._opening_imposed: a hard, enforced cap with no
    window of its own, a zone that does not move); the warning said "its
    counter scores those ticks as P0 escapes" in every case. It now says
    which: with the survey policy's 4 m/s cap, no P0 escape; with no hard
    cap (or a moving zone), the escapes the counter still scores."""
    pol = load_policy(SURVEY)
    ws = time_window_lead_warnings(pol, start=(60.0, 21.0, 15.0), now=MON_0729_58)
    assert len(ws) == 1 and "not as a P0 escape" in ws[0], ws
    assert "scores those ticks as P0 escapes" not in ws[0], ws[0]
    big = _zone("big", 40.0, 40.0)
    for caps in ((), (dict(_cap(4.0), constraint_type="soft"),)):
        w = time_window_lead_warnings(_pol(big, *caps))[0]
        assert "its counter scores those ticks as P0 escapes" in w, (caps, w)
        assert "cannot prove" in w, (caps, w)
    w = time_window_lead_warnings(_pol(big, _cap(4.0)))[0]
    assert "not as a P0 escape" in w and "cannot prove" not in w, w
    moving = dict(big, type="dynamic_nfz", motion={"vx_mps": 0.5})
    w = time_window_lead_warnings(_pol(moving, _cap(4.0)))[0]
    assert "a moving zone" in w and "scores those ticks as P0 escapes" in w, w


def test_warnings_never_refuse():
    pol = load_policy(SURVEY)
    assert pol.lint() == []                     # the error lint is untouched
    out = subprocess.run([sys.executable, "-m", "guardrail.compiler", "lint", str(SURVEY),
                          "--start", "60,21,15", "--now", "2026-09-07T07:29:58"],
                         cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr
    assert "WARNING nfz-school" in out.stdout, out.stdout
    out = subprocess.run([sys.executable, "-m", "guardrail.compiler", "lint", str(SURVEY)],
                         cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert out.returncode == 0 and "no lint warnings" in out.stdout, out.stdout


def test_the_harness_records_the_warning_on_the_cell_that_carries_it():
    import sweep_scenarios as SW
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    fams = lib.by_id()
    on = SW.run_episode(fams["time-window-opens-on-vehicle"].first.spec, lib.defaults, 0)
    ahead = SW.run_episode(fams["time-window-opens-ahead"].first.spec, lib.defaults, 0)
    assert len(on.extra["lint_warnings"]) == 1, on.extra["lint_warnings"]
    assert on.events[0]["lint_warnings"] == on.extra["lint_warnings"]
    assert ahead.extra["lint_warnings"] == []     # starts outside the ring


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
