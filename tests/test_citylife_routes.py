"""The CityLife driving paths: lanes on the side the road markings show (left),
arcs that fit, speeds that a car could actually hold.

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


def test_the_default_side_is_the_one_the_markings_show():
    """The lane arrows put eastbound traffic on the NORTH half (+X), i.e. the
    driver's left. Loop A's first leg runs east along x = 4100, so its lane is
    at x = 4450 - not 3750, where the 2026-09-22 cars drove against the paint."""
    assert R.DRIVE_SIDE == "left"
    a = R.lane_corners(R.LOOPS["A"])
    assert a[1] == (4450.0, 11950.0), a       # east on x=4450, north on y=11950


def test_right_hand_geometry_reproduces_what_the_level_used_to_drive():
    """drive_side='right' lands exactly on the lane centres built on 2026-09-22,
    which is how those cars can be shown to have been on the wrong side."""
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
    for name, p in R.build_all().items():
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
    for name, p in R.build_all().items():
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
    for name, p in R.build_all().items():
        for v, k in zip(p.speed, p.curvature):
            if k:
                assert v * v * abs(k) <= R.A_LAT_CMS2 + 1e-6, (name, v, k)


def test_the_braking_ramp_is_achievable():
    """Between consecutive points the limit never drops faster than the car
    can brake: v_i^2 <= v_{i+1}^2 + 2 a d."""
    for name, p in R.build_all().items():
        n = len(p.pts)
        for i in range(n):
            j = (i + 1) % n
            d = math.hypot(p.pts[j][0] - p.pts[i][0], p.pts[j][1] - p.pts[i][1])
            assert p.speed[i] ** 2 <= p.speed[j] ** 2 + 2 * R.A_BRAKE_CMS2 * d + 1e-3


def test_loops_meet_only_inside_a_junction_they_share():
    """Keeping left, B's two corners on A's streets are wide turns across the
    oncoming lane - A's - so the paths DO cross, at (4100, 4100) and
    (12300, -4100). That is ordinary traffic and the car model yields for it at
    the junction; what must never happen is two paths meeting anywhere a
    junction rule cannot see, i.e. outside a box both loops pass through."""
    paths = R.build_all()
    names = sorted(paths)
    met = set()
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            shared = set(R.junctions_on(R.LOOPS[a])) & set(R.junctions_on(R.LOOPS[b]))
            for pt in R.segments_cross(paths[a], paths[b], tol=150.0):
                box = [j for j in shared
                       if abs(pt[0] - j[0]) <= R.JUNCTION_HALF_CM
                       and abs(pt[1] - j[1]) <= R.JUNCTION_HALF_CM]
                assert box, (a, b, pt, "crossing outside any shared junction")
                met.add((a, b, box[0]))
    assert met == {("A", "B", (4100.0, 4100.0)), ("A", "B", (12300.0, -4100.0))}, met


def test_junctions_on_a_loop_include_the_ones_it_drives_straight_through():
    js = R.junctions_on(R.LOOPS["A"])
    assert len(js) == 8, js                     # a 2 x 2 block loop: 8 on its ring
    assert (4100.0, 4100.0) in js               # passed straight over, not a corner
    assert js[0] == R.LOOPS["A"][0]


def test_loop_lengths_match_the_lane_rectangle_less_the_fillets():
    """Each loop is its lane rectangle with four corners cut: a 90 deg fillet of
    radius r replaces 2r of straight with (pi/2) r of arc. A turns left at every
    corner, so keeping left it runs INSIDE its block on 7 m arcs; B turns right,
    so it runs outside on 13 m arcs."""
    paths = R.build_all()
    for name, p in paths.items():
        c = R.lane_corners(R.LOOPS[name])
        rect = sum(math.hypot(c[(i + 1) % 4][0] - c[i][0], c[(i + 1) % 4][1] - c[i][1])
                   for i in range(4))
        cut = sum(r * (2.0 - math.pi / 2.0) for _, r in p.radii)
        assert abs(p.length_cm - (rect - cut)) < 0.005 * rect, (name, p.length_cm, rect - cut)
    assert [k for k, _ in paths["A"].radii] == ["near"] * 4
    assert [k for k, _ in paths["B"].radii] == ["far"] * 4


def test_a_car_watches_the_crossings_on_its_path_not_on_its_heading():
    """Loop A comes south down y = -4100 and turns left at (4100, -4100). It
    drives over the crossing round the corner at (4100, -3000) and the one
    before the corner at (5200, -4100); it never reaches (3000, -4100), which
    lies straight ahead of it as it approaches the turn."""
    paths = R.build_all()
    on_a = {(x, y) for x, y, _ in R.crossings_on(paths["A"])}
    assert (4100.0, -3000.0) in on_a and (5200.0, -4100.0) in on_a, on_a
    assert (3000.0, -4100.0) not in on_a, on_a
    for x, y, i in R.crossings_on(paths["A"]):
        px, py = paths["A"].pts[i]
        assert math.hypot(px - x, py - y) <= 450.0


def test_only_real_crossings_are_give_way_junctions():
    """B turns across the oncoming lane at all four corners, but its path meets
    A's at two of them. Only those two may make it wait, and A - driving
    straight through - gives way nowhere."""
    gw = R.give_way_junctions(R.build_all())
    assert gw["B"] == {(4100.0, 4100.0), (12300.0, -4100.0)}, gw
    assert gw["A"] == set() and gw["C"] == set(), gw


def _drive(path, dt, laps=1.3, start=0, offset=(0.0, 0.0), yaw_err=0.0, frame=None):
    """Run the reference follower; worst deviation and lateral accel after it
    has settled (the first 30 % of a lap is start-up). `frame(i)` gives the
    i-th frame time and switches to `follow`, the sub-stepping frame loop."""
    a, b = path.pts[start], path.pts[start + 1]
    s = R.CarState(a[0] + offset[0], a[1] + offset[1],
                   math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) + yaw_err,
                   0.0, start + 1)
    dist, dev, alat, step = 0.0, 0.0, 0.0, 0
    while dist < laps * path.length_cm:
        if frame is None:
            k = R.follow_step(path, s, dt)
        else:
            dt = frame(step)
            k = R.follow(path, s, dt, v_cap=450.0)
        dist += s.v * dt
        step += 1
        if dist > 0.3 * path.length_cm and step % 3 == 0:
            dev = max(dev, R.distance_to_path(path, s.x, s.y))
            alat = max(alat, s.v * s.v * abs(k))
    return dev, alat


def test_the_follower_holds_the_lane_through_every_corner():
    """At 60 Hz the car stays within 10 cm of its lane centre, arcs included;
    at 10 Hz - a simulator busy rendering for a flight - within 25 cm."""
    for name, p in R.build_all().items():
        for dt, limit in ((1 / 60, 10.0), (1 / 10, 25.0)):
            dev, _ = _drive(p, dt)
            assert dev < limit, (name, dt, dev)


def test_the_follower_turns_at_v_over_r_not_harder():
    """Lateral acceleration stays at the profile's 1.8 m/s^2 plus the small
    correction, under 2.0 - the feed-forward is what keeps it there."""
    for name, p in R.build_all().items():
        _, alat = _drive(p, 1 / 30)
        assert alat <= 200.0, (name, alat)


def test_a_long_frame_does_not_throw_the_car_off_its_arc():
    """Every 20th frame takes 0.5 s, as editor hitches do. Stepped whole, such a
    frame drives 1.8 m on the wrong curvature at an arc's end; cut into 50 ms
    sub-steps the car holds its lane as if the frames were even."""
    spiky = lambda i: 0.5 if i % 20 == 7 else 0.1          # noqa: E731
    for name, p in R.build_all().items():
        dev, alat = _drive(p, None, frame=spiky)
        assert dev < 15.0 and alat <= 200.0, (name, dev, alat)


def test_the_follower_recovers_from_a_bad_start():
    """Placed 1.5 m off the lane and 10 deg askew, as a car moved by hand in the
    editor would be, it converges and then holds the lane."""
    dev, _ = _drive(R.build_all()["A"], 1 / 30, offset=(150.0, 0.0), yaw_err=10.0)
    assert dev < 15.0, dev


def test_the_follower_stops_where_it_is_told():
    p = R.build_all()["A"]
    a, b = p.pts[200], p.pts[201]
    s = R.CarState(a[0], a[1], math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])), 800.0, 201)
    x0, y0 = s.x, s.y
    for _ in range(600):
        stop = 3000.0 - math.hypot(s.x - x0, s.y - y0)
        R.follow_step(p, s, 1 / 30, stop_cm=stop)
    travelled = math.hypot(s.x - x0, s.y - y0)
    assert s.v < 5.0 and 2800.0 < travelled <= 3005.0, (s.v, travelled)


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
