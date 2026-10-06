"""The CityLife signal plan: tools/citylife_signals.py.

Run either way:
    pytest tests/test_citylife_signals.py -v
    python tests/test_citylife_signals.py

The signals are a pure function of game time that every car, and later the
lamp materials, evaluate on their own. Nothing in the level can show a timing
mistake except a car driving into a crossing car, so these checks pin the plan:
the phases, the all-red at every change, pedestrians only while the traffic
they cross is held, and offsets that the Blueprint's C-style % reproduces.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import citylife_signals as S          # noqa: E402
from tools import citylife_routes as R           # noqa: E402

# Every junction the three loops drive through, plus one far off the corridor.
LEVEL = sorted({j for k in R.LOOPS for j in R.junctions_on(R.LOOPS[k])}) + [(-12300.0, -4100.0)]
STEP = 0.05


def _times(cycles=2):
    return [k * STEP for k in range(int(cycles * S.CYCLE_S / STEP))]


def test_cycle_is_green_yellow_all_red_per_axis():
    assert (S.GREEN_S, S.YELLOW_S, S.ALL_RED_S, S.CYCLE_S) == (20.0, 3.0, 2.0, 50.0)
    x, y = 4100.0, 4100.0                                   # (0, 0): offset 0
    assert S.offset(x, y) == 0.0
    for t, ns, ew in ((0.0, "green", "red"), (19.9, "green", "red"), (20.0, "yellow", "red"),
                      (22.9, "yellow", "red"), (23.0, "red", "red"), (24.9, "red", "red"),
                      (25.0, "red", "green"), (44.9, "red", "green"), (45.0, "red", "yellow"),
                      (48.0, "red", "red"), (49.9, "red", "red"), (50.0, "green", "red")):
        assert S.state(t, x, y, S.NS) == ns, (t, "NS", S.state(t, x, y, S.NS))
        assert S.state(t, x, y, S.EW) == ew, (t, "EW", S.state(t, x, y, S.EW))


def test_the_two_axes_are_never_both_moving_and_all_red_is_two_seconds_twice():
    for jx, jy in LEVEL:
        all_red = 0.0
        for t in _times():
            ns, ew = S.state(t, jx, jy, S.NS), S.state(t, jx, jy, S.EW)
            assert ns == "red" or ew == "red", (jx, jy, t, ns, ew)
            assert not (ns == "green" and ew == "green")
            if ns == "red" and ew == "red":
                all_red += STEP
        assert abs(all_red - 2 * 2 * S.ALL_RED_S) < 2 * STEP, (jx, jy, all_red)   # two cycles


def test_every_axis_gets_its_green_once_per_cycle():
    for jx, jy in LEVEL:
        for axis in S.AXES:
            green = sum(STEP for t in _times(1) if S.state(t, jx, jy, axis) == "green")
            assert abs(green - S.GREEN_S) < 2 * STEP, (jx, jy, axis, green)


def test_walk_only_while_the_crossed_traffic_is_red_and_the_parallel_traffic_green():
    for jx, jy in LEVEL:
        for road in S.AXES:
            walk_s = 0.0
            for t in _times():
                if S.walk(t, jx, jy, road):
                    walk_s += STEP
                    assert S.state(t, jx, jy, road) == "red", (jx, jy, road, t)
                    assert S.state(t, jx, jy, S.other(road)) == "green", (jx, jy, road, t)
                    assert S.ped_phase(t, jx, jy, road) == "walk"
            assert abs(walk_s - 2 * S.WALK_S) < 2 * STEP, (jx, jy, road, walk_s)


def test_flash_ends_with_the_parallel_yellow_and_nobody_starts_in_it():
    x, y = 4100.0, -4100.0                                  # offset 27
    off = S.offset(x, y)
    road = S.NS                                             # walkers cross the NS road with EW green
    ew_start = off + S.HALF_CYCLE_S
    assert S.ped_phase(ew_start + 7.9, x, y, road) == "walk"
    assert S.ped_phase(ew_start + 8.1, x, y, road) == "flash"
    assert S.ped_phase(ew_start + 22.9, x, y, road) == "flash"
    assert S.ped_phase(ew_start + 23.1, x, y, road) == "dont"          # the all-red
    assert not S.walk(ew_start + 8.1, x, y, road)


def test_a_walker_who_starts_last_clears_before_its_road_turns_green():
    """The last walk instant leaves G - WALK + Y + all-red = 17 s; kerb to kerb
    is 1800 cm, so any crosser at >= 106 cm/s is off the road in time."""
    clear_s = S.GREEN_S - S.WALK_S + S.YELLOW_S + S.ALL_RED_S
    assert clear_s == 17.0
    assert 1800.0 / clear_s < 106.0
    x, y = 4100.0, 4100.0
    for road in S.AXES:
        last = S.next_walk_start(0.0, x, y, road) + S.WALK_S - 1e-6
        assert S.walk(last, x, y, road)
        t = last
        while S.state(t, x, y, road) != "green":
            t += STEP
        assert abs((t - last) - clear_s) < 2 * STEP, (road, t - last)


def test_next_walk_start_is_the_first_walk_instant():
    x, y = 12300.0, -4100.0
    for road in S.AXES:
        for t0 in (0.0, 3.3, 17.0, 31.7, 49.9):
            ts = S.next_walk_start(t0, x, y, road)
            assert ts >= t0 and S.walk(ts + 1e-9, x, y, road)
            k = t0
            while k < ts - STEP:
                assert not S.walk(k, x, y, road), (road, t0, k, ts)
                k += STEP


def test_offsets_are_deterministic_spread_and_what_the_dsl_computes():
    assert S.offset(4100.0, -4100.0) == 27.0               # (0, -1): -23 mod 50
    assert S.offset(4100.0, 4100.0) == 0.0
    assert S.offset(12300.0, -4100.0) == 38.0              # (1, -1): 11 - 23
    assert S.offset(-12300.0, -4100.0) == 5.0              # (-2, -1): -45 mod 50
    loop_a = [S.offset(x, y) for x, y in R.junctions_on(R.LOOPS["A"])]
    assert len(set(loop_a)) == len(loop_a)                  # no two of A's 8 switch in step
    for jx, jy in LEVEL:
        assert 0.0 <= S.offset(jx, jy) < S.CYCLE_S
        assert S.offset(jx, jy) == S.offset(jx, jy)
        for axis in S.AXES:
            for t in (0.0, 0.4, 7.25, 24.99, 25.0, 49.0, 123.4, 1799.95):
                a, b = S.phase_time(t, jx, jy, axis), S.dsl_phase(t, jx, jy, axis)
                assert abs(a - b) < 1e-9 or abs(abs(a - b) - S.CYCLE_S) < 1e-9, (jx, jy, axis, t, a, b)


def test_grid_membership_exceptions_and_axes():
    assert S.grid_index(20500.0, -4100.0) == (2, -1)
    try:
        S.grid_index(4000.0, 4100.0)
        assert False, "an off-grid point got an offset"
    except ValueError:
        pass
    assert S.signalled(4100.0, 4100.0)
    assert not S.signalled(4100.0, 4100.0, exceptions={(0, 0)})
    assert not S.signalled(4000.0, 4100.0)
    assert S.axis_of(-1.0, 0.02) == S.NS and S.axis_of(0.1, 1.0) == S.EW
    assert S.crossing_road_axis(5200.0, -4100.0, 4100.0, -4100.0) == S.NS   # north of the junction
    assert S.crossing_road_axis(4100.0, -3000.0, 4100.0, -4100.0) == S.EW   # east of it
    for bad in ("N", "ns", ""):
        try:
            S.state(0.0, 4100.0, 4100.0, bad)
            assert False, bad
        except ValueError:
            pass


def test_dsl_constants_are_the_module_constants():
    c = S.dsl_constants()
    assert c["CYCLE_S"] == S.CYCLE_S == 2 * c["HALF_CYCLE_S"]
    assert c["HALF_CYCLE_S"] == c["GREEN_S"] + c["YELLOW_S"] + c["ALL_RED_S"]
    assert (c["OFFSET_I"], c["OFFSET_J"], c["WALK_S"]) == (11.0, 23.0, 8.0)


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
