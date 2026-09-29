"""Following where the subject drove: demo/trail.py.

Run either way:
    pytest tests/test_trail.py -v
    python tests/test_trail.py

The follow controller used to fly every command along the nose. At a corner
that cut toward a car which had already turned, and once the car was out of
sight it carried the aircraft straight past the junction. These tests pin the
geometry the controller now uses instead.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))

from trail import (Trail, approach_speed, direction, heading_to,   # noqa: E402
                   lookout)


def _l_trail(step=1.0):
    """East along y from (0, 0) to (0, 30), then north (+x) to (20, 30):
    a car that turned LEFT at (0, 30) in this level's X = north frame."""
    t = Trail(spacing_m=1.0)
    for k in range(0, 31):
        t.add(0.0, float(k))
    for k in range(1, 21):
        t.add(float(k), 30.0)
    return t


def test_on_a_straight_trail_the_carrot_is_straight_ahead():
    t = Trail(spacing_m=1.0)
    for k in range(40):
        t.add(0.0, float(k))
    cx, cy = t.carrot(0.0, 10.0, 6.0)
    assert abs(cx) < 1e-9 and abs(cy - 16.0) < 1e-9


def test_at_a_corner_the_carrot_turns_where_the_subject_turned():
    """8 m before the corner with a 12 m look-ahead the carrot is 4 m up the
    cross street - the aircraft turns with the road instead of cutting across
    the corner block toward where the car is now."""
    t = _l_trail()
    cx, cy = t.carrot(0.0, 22.0, 12.0)
    assert abs(cx - 4.0) < 1e-6 and abs(cy - 30.0) < 1e-6, (cx, cy)


def test_the_carrot_never_runs_past_the_end():
    t = _l_trail()
    assert t.carrot(15.0, 30.0, 50.0) == (20.0, 30.0)


def test_remaining_counts_along_the_trail_not_as_the_crow_flies():
    t = _l_trail()
    assert abs(t.remaining(0.0, 20.0) - (10.0 + 20.0)) < 1e-6


def test_the_end_heading_is_the_last_direction_of_travel():
    t = _l_trail()
    assert abs(t.end_heading() - 0.0) < 1e-9          # north: atan2(0, +dx) = 0
    straight = Trail(spacing_m=1.0)
    for k in range(10):
        straight.add(0.0, float(k))
    assert abs(straight.end_heading() - math.pi / 2) < 1e-9   # east


def test_breadcrumbs_are_spaced_and_the_trail_is_bounded():
    t = Trail(spacing_m=1.5, max_len_m=20.0)
    assert t.add(0.0, 0.0) and not t.add(0.0, 1.0) and t.add(0.0, 1.6)
    for k in range(2, 60):
        t.add(0.0, 1.6 * k)
    length = sum(math.hypot(t.pts[i + 1][0] - t.pts[i][0], t.pts[i + 1][1] - t.pts[i][1])
                 for i in range(len(t.pts) - 1))
    assert length <= 20.0 + 1e-6 and t.end() == (0.0, 1.6 * 59)


def test_projection_does_not_jump_back_along_a_trail_that_doubles_back():
    """A U-shaped trail passes close to itself; having projected onto the far
    leg, the aircraft must not snap back to the near one."""
    t = Trail(spacing_m=1.0)
    for k in range(0, 21):
        t.add(0.0, float(k))
    for k in range(1, 5):
        t.add(float(k), 20.0)
    for k in range(19, -1, -1):
        t.add(4.0, float(k))
    t.carrot(4.0, 10.0, 2.0)               # on the return leg
    cx, cy = t.carrot(2.0, 10.0, 2.0)      # now equidistant from both legs
    assert abs(cx - 4.0) < 1e-6 and cy < 10.0, (cx, cy)


def test_a_straight_trail_flies_the_same_direction_as_the_nose():
    """On a straight street the aircraft, nose on the car, was already flying
    along the trail: following it must change nothing there."""
    t = Trail(spacing_m=1.0)
    for k in range(30):
        t.add(0.0, 20.0 + k)                      # the car, ahead along +y
    ux, uy = direction(t, 0.0, 5.0, 10.0)         # aircraft behind, nose +y
    assert abs(ux) < 1e-9 and abs(uy - 1.0) < 1e-9


def test_at_the_corner_the_direction_turns_with_the_street():
    t = _l_trail()
    ux, uy = direction(t, 0.0, 28.0, 10.0)        # 2 m before the corner
    assert ux > 0.6 and uy > 0.0, (ux, uy)        # already swinging north


def test_past_the_end_the_direction_is_the_last_direction_of_travel():
    """At, and well PAST, the last sighting: keep going the way the subject
    went. It used to point back at the end from more than a metre beyond it,
    so a search creep oscillated on the spot (review, 2026-09-24)."""
    t = _l_trail()                                # ends (20, 30) heading north
    for px in (19.8, 20.5, 21.5, 25.0, 40.0):
        ux, uy = direction(t, px, 30.0, 10.0)
        assert abs(ux - 1.0) < 1e-9 and abs(uy) < 1e-9, (px, ux, uy)
    ux, uy = direction(t, 22.0, 33.0, 10.0)       # past it and off to one side
    assert abs(ux - 1.0) < 1e-9 and abs(uy) < 1e-9


def test_a_creep_past_the_end_carries_on_instead_of_oscillating():
    t = _l_trail()
    x, y = 15.0, 30.0
    xs = []
    for _ in range(100):                          # 10 s at 1.5 m/s
        ux, uy = direction(t, x, y, 10.0)
        x, y = x + 0.15 * ux, y + 0.15 * uy
        xs.append(x)
    assert xs[-1] > 28.0 and all(b >= a - 1e-9 for a, b in zip(xs, xs[1:]))


def test_remaining_counts_the_street_to_the_first_breadcrumb():
    """An aircraft behind a short trail still has the gap to its start to fly;
    without it the coast approach crawled with the sighting 20 m away."""
    t = Trail(spacing_m=1.0)
    t.add(0.0, 0.0)
    t.add(1.5, 0.0)
    assert abs(t.remaining(-20.0, 0.0) - 21.5) < 1e-9
    assert approach_speed(t.remaining(-20.0, 0.0), 4.0) == 4.0


def test_the_search_looks_down_the_street_the_subject_turned_into():
    """Lost at the corner with the nose still pointing east (+y), the lookout
    bearing points north-ish, past the last sighting - not along the nose."""
    t = Trail(spacing_m=1.0)
    for k in range(0, 31):
        t.add(0.0, float(k))
    for k in range(1, 6):
        t.add(float(k), 30.0)                     # seen 5 m into the turn
    yaw_east = math.pi / 2
    b = lookout(t, 0.0, 15.0, yaw_east, ahead_m=8.0)
    # To the LEFT of the nose (north of east), which is where the car went -
    # and not behind it. The end heading is taken over the last 6 m, so a car
    # seen only 5 m into its turn gives a heading between the two streets.
    assert -math.pi / 2 < b < -0.2, b


def test_the_lookout_never_turns_round_once_past_the_look_point():
    """Creeping on past the last sighting, the nose keeps looking the way the
    subject went; aiming at the (overflown) look point turned it backward."""
    t = Trail(spacing_m=1.0)
    for k in range(31):
        t.add(0.0, float(k))                      # east, ends (0, 30)
    yaw_east = math.pi / 2
    for py in (25.0, 36.0, 41.2, 50.0, 80.0):
        b = lookout(t, 0.0, py, yaw_east, ahead_m=8.0)
        assert abs(b) < 1e-9, (py, b)


def test_coast_arrives_at_the_last_sighting_and_stops():
    assert approach_speed(40.0, 4.0) == 4.0
    assert abs(approach_speed(3.0, 4.0) - 1.5) < 1e-9
    assert approach_speed(0.0, 4.0) == 0.0


def test_a_jump_restarts_the_trail():
    t = Trail(spacing_m=1.0, max_jump_m=30.0)
    for k in range(10):
        t.add(0.0, float(k))
    t.add(50.0, 9.0)                              # re-acquired a block away
    assert t.pts == [(50.0, 9.0)] and t.n_restarts == 1


def test_a_short_trail_steers_nothing():
    """Three noisy breadcrumbs from 40 m do not make a direction: until the
    trail is MIN_TRAIL_M long the controller keeps its own logic."""
    t = Trail(spacing_m=1.5)
    for x, y in ((0.0, 0.0), (1.2, 1.5), (-0.8, 3.1)):
        t.add(x, y)
    assert t.length() < 8.0
    assert direction(t, -10.0, 0.0, 10.0) is None
    assert lookout(t, -10.0, 0.0, 0.0) is None
    for k in range(2, 8):
        t.add(-0.8, 3.1 + 1.5 * k)
    assert t.length() >= 8.0 and direction(t, -10.0, 0.0, 10.0) is not None


def test_heading_to_needs_a_metre():
    assert heading_to(0.0, 0.0, (0.5, 0.0)) is None
    ux, uy = heading_to(0.0, 0.0, (3.0, 4.0))
    assert abs(ux - 0.6) < 1e-9 and abs(uy - 0.8) < 1e-9


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
