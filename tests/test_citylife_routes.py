"""The CityLife driving paths: lanes on the right side, arcs that fit, speeds
that a car could actually hold.

Run either way:
    pytest tests/test_citylife_routes.py -v
    python tests/test_citylife_routes.py

The paths are written into a gitignored Unreal level, so these checks are the
only record that they are sane. What they guard against is the kind of mistake
that already happened once: lanes computed with the axes swapped (X is NORTH in
this level), documented as "keeping left" while the geometry kept right.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import citylife_routes as R                                 # noqa: E402

EAST, NORTH, WEST, SOUTH = (0.0, 1.0), (1.0, 0.0), (0.0, -1.0), (-1.0, 0.0)


def test_right_of_a_heading_in_a_north_x_east_y_frame():
    assert R.right_of(EAST) == (-1.0, 0.0)        # facing east, right is south
    assert R.right_of(NORTH) == (-0.0, 1.0)       # facing north, right is east
    assert R.left_of(EAST) == (1.0, -0.0)         # facing east, left is north


def test_turn_direction_in_this_frame():
    assert R.turns_left(EAST, NORTH)              # east then north: a left turn
    assert not R.turns_left(EAST, SOUTH)          # east then south: right
    assert R.turns_left(NORTH, WEST)


def test_the_lanes_reproduce_what_the_level_already_drives():
    """With drive_side='right' the generator lands on the lane centres built on
    2026-09-22 for loops A and B, so switching to the generated paths moves no
    car to a different side of the road."""
    a = R.lane_corners(R.LOOPS["A"], "right")
    assert a[1] == (3750.0, 12650.0) and a[2] == (20850.0, 12650.0), a
    b = R.lane_corners(R.LOOPS["B"], "right")
    assert b[0] == (4450.0, -3750.0) and b[1] == (11950.0, -3750.0), b


def test_left_hand_traffic_puts_every_lane_on_the_other_side():
    r = R.lane_corners(R.LOOPS["A"], "right")
    l = R.lane_corners(R.LOOPS["A"], "left")
    for (rx, ry), (lx, ly), j in zip(r, l, R.LOOPS["A"]):
        assert (rx + lx) / 2 == j[0] and (ry + ly) / 2 == j[1]


def test_every_corner_is_a_fillet_of_the_right_radius():
    for side in ("right", "left"):
        for name, p in R.build_all(side).items():
            kinds = [k for k, _ in p.radii]
            assert len(kinds) == 4, (name, kinds)
            for kind, r in p.radii:
                assert r == (R.R_NEAR_CM if kind == "near" else R.R_FAR_CM)


def test_the_path_is_smooth_no_heading_jump_bigger_than_an_arc_step():
    """A turn is a sequence of small heading changes, never one 90 deg step."""
    for name, p in R.build_all("right").items():
        n = len(p.pts)
        max_step = 0.0
        for i in range(n):
            a, b, c = p.pts[i - 1], p.pts[i], p.pts[(i + 1) % n]
            h1 = math.atan2(b[1] - a[1], b[0] - a[0])
            h2 = math.atan2(c[1] - b[1], c[0] - b[0])
            d = abs((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)
            max_step = max(max_step, d)
        # 150 cm on a 7 m arc is 12.3 deg; allow the seam a little more
        assert math.degrees(max_step) < 16.0, (name, math.degrees(max_step))


def test_points_are_evenly_spaced():
    for name, p in R.build_all("right").items():
        n = len(p.pts)
        gaps = [math.hypot(p.pts[(i + 1) % n][0] - p.pts[i][0],
                           p.pts[(i + 1) % n][1] - p.pts[i][1]) for i in range(n)]
        assert min(gaps) > 60.0 and max(gaps) < 200.0, (name, min(gaps), max(gaps))


def test_the_car_stays_inside_the_carriageway():
    """Every point, widened by half a car, is within the kerb of the street it
    is on. Inside a junction box the nearer centre line is used."""
    limit = R.CARRIAGEWAY_HALF_CM - R.CAR_HALF_WIDTH_CM
    for side in ("right", "left"):
        for name, p in R.build_all(side).items():
            worst = max(R.lateral_from_centre(pt, R.LOOPS[name]) for pt in p.pts)
            assert worst <= limit, (side, name, worst, limit)


def test_arcs_start_and_end_inside_the_junction_box():
    for side in ("right", "left"):
        for name, p in R.build_all(side).items():
            for pt, k in zip(p.pts, p.curvature):
                if not k:
                    continue
                nearest = min(R.LOOPS[name],
                              key=lambda j: math.hypot(pt[0] - j[0], pt[1] - j[1]))
                assert abs(pt[0] - nearest[0]) <= R.JUNCTION_HALF_CM
                assert abs(pt[1] - nearest[1]) <= R.JUNCTION_HALF_CM


def test_corner_speed_respects_lateral_acceleration():
    for name, p in R.build_all("right").items():
        for v, k in zip(p.speed, p.curvature):
            if k:
                assert v * v * abs(k) <= R.A_LAT_CMS2 + 1e-6, (name, v, k)


def test_the_braking_ramp_is_achievable():
    """Between consecutive points the limit never drops faster than the car
    can brake: v_i^2 <= v_{i+1}^2 + 2 a d."""
    for name, p in R.build_all("right").items():
        n = len(p.pts)
        for i in range(n):
            j = (i + 1) % n
            d = math.hypot(p.pts[j][0] - p.pts[i][0], p.pts[j][1] - p.pts[i][1])
            assert p.speed[i] ** 2 <= p.speed[j] ** 2 + 2 * R.A_BRAKE_CMS2 * d + 1e-3


def test_the_loops_do_not_cross_each_other():
    """Nothing in the car model yields at a junction yet, so two loops that
    cross would drive through each other. A and B share streets in opposite
    directions; C is a block west. None of them may meet."""
    paths = R.build_all("right")
    names = sorted(paths)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            hits = R.segments_cross(paths[a], paths[b], tol=150.0)
            assert not hits, (a, b, hits[:3])


def test_loop_lengths_are_what_the_level_expects():
    paths = R.build_all("right")
    # A is the long demo loop, about 684 m on its corners; fillets shorten it.
    assert 640_00 < paths["A"].length_cm < 684_00, paths["A"].length_cm
    assert 270_00 < paths["B"].length_cm < 300_00, paths["B"].length_cm


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
