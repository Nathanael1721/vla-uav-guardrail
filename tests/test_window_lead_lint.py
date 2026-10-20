"""The DSL's first lint WARNING class (guardrail/compiler.py,
time_window_lead_warnings): a time window that can open on an aircraft that
cannot leave before it does.

Run either way:
    pytest tests/test_window_lead_lint.py -v
    python tests/test_window_lead_lint.py

WHY THIS FILE EXISTS

The library's only P0 escape ticks (6, in the three time-window-opens-on-
vehicle cells) are a window opening over an aircraft 10 m from the school
zone's ring - 2.5 s at the 4 m/s cap - 1.0-2.0 s after the mission starts. No
action clears the forecast on the first tick, so the Shield flies the way out
with the violation on and its counter scores an escape; no repair and no hold
point can remove that tick. Only the policy or the mission's timing can, so
the DSL says so up front - as a warning, never a refusal (the grant leaves
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
