"""Searching for a lost car on its streets: demo/search.py.

Run either way:
    pytest tests/test_search.py -v
    python tests/test_search.py

After losing the red car, the follow used to end in SCAN: yaw in place, zero
forward speed, forever - on the reference flight it rotated against a building
corner. These tests pin what replaces it: go to the next junction the car was
driving toward, look down each exit it could have taken, then hold over that
junction; never overfly a car last seen stopped; never aim off the street.

Frames: NED metres, x = North, y = East, heading 0 = North, +pi/2 = East.
"""
import math
import random
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))

from build_street_mask import is_street, load_street          # noqa: E402
from camera_model import bearing_to_cx                         # noqa: E402
from search import (DWELL, DWELL_PER_EXIT_S, HOLD, HOLD_RATE_RAD_S,   # noqa: E402
                    HOLD_SWEEP_DEG, PURSUE, PURSUE_SWEEP_DEG, STANDOFF,
                    STANDOFF_SWEEP_DEG, SearchPlanner, camera_blind_m,
                    junctions_from_routes, snap_heading)

N, E, S, W_ = 0.0, math.pi / 2, math.pi, -math.pi / 2
IMG_W, HFOV = 768, 90.0
REAL_MASK = ROOT / "demo" / "out" / "citymap_citylife" / "street.npz"


def _ang(a, b):
    """Smallest absolute difference between two headings."""
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def _close(p, q, tol=1e-6):
    return math.hypot(p[0] - q[0], p[1] - q[1]) <= tol


def _lattice_mask(holes=(), x0=-151.0, x1=229.0, y0=-71.0, y1=151.0,
                  res=2.0, half=8.0):
    """Streets 16 m wide on the level's 82 m grid (junctions at 41 + 82 k),
    junction centres exactly on cell centres, and single-cell holes where
    asked - as the real mask has at (41, -41)."""
    xs = x0 + res * np.arange(int(round((x1 - x0) / res)) + 1)
    ys = y0 + res * np.arange(int(round((y1 - y0) / res)) + 1)
    X, Y = np.meshgrid(xs, ys, indexing="ij")

    def off_line(v):
        r = (v - 41.0) % 82.0
        return np.minimum(r, 82.0 - r)

    street = ((off_line(X) <= half) | (off_line(Y) <= half)).astype(np.uint8)
    for hx, hy in holes:
        street[int(round((hx - x0) / res)), int(round((hy - y0) / res))] = 0
    return {"street": street, "res": res, "ox": x0, "oy": y0}


def _fly(planner, anchor, start, t_end=60.0, dt=0.25):
    """Fly straight at each tick's target at its speed cap; the rows seen."""
    x, y = start
    rows, t = [], 0.0
    while t <= t_end + 1e-9:
        cmd = planner.step(t, (x, y), anchor)
        rows.append((t, (x, y), cmd))
        tx, ty = cmd["target_xy"]
        d = math.hypot(tx - x, ty - y)
        step = min(d, cmd["speed_cap_mps"] * dt)
        if d > 1e-9:
            x, y = x + (tx - x) / d * step, y + (ty - y) / d * step
        t += dt
    return rows


def _northbound(**kw):
    """Lost northbound on the N-S street y = 41, in its keep-left lane
    (the left of north is west, so y = 41 - 3.5)."""
    a = {"p": (90.0, 37.5), "heading": 0.08, "speed": 4.0, "stopped": False}
    a.update(kw)
    return a


# ------------------------------------------------------------------ helpers

def test_snap_heading_picks_the_nearest_street_axis():
    assert snap_heading(0.3) == N and snap_heading(-0.3) == N
    assert snap_heading(1.3) == E and snap_heading(-1.2) == W_
    assert snap_heading(2.9) == S and snap_heading(-2.9) == S
    assert snap_heading(2 * math.pi + 0.1) == N


def test_junctions_from_routes_are_the_loops_junctions_in_metres():
    js = junctions_from_routes()
    assert len(js) == 13 and len(set(js)) == 13, js
    for x, y in js:                           # all on the 41 + 82 k lattice
        assert (x - 41.0) % 82.0 == 0.0 and (y - 41.0) % 82.0 == 0.0, (x, y)
    assert (41.0, -41.0) in js                # a corner of loops A and B
    assert (41.0, 41.0) in js                 # crossed straight over by A
    assert (205.0, 123.0) in js and (-123.0, -41.0) in js


# ------------------------------------------------------------------ pursue

def test_a_car_lost_northbound_is_pursued_to_the_next_junction_north():
    """The junction it was driving toward - not the nearest one, and not a
    point straight ahead off the street."""
    cmd = SearchPlanner().step(0.0, (80.0, 41.0), _northbound())
    assert cmd["mode"] == PURSUE
    assert _close(cmd["target_xy"], (123.0, 41.0)) and cmd["junction"] == (123.0, 41.0)
    assert _ang(cmd["look_heading_rad"], N) < 1e-9       # the snapped street heading
    assert cmd["sweep_deg"] == PURSUE_SWEEP_DEG == 35.0
    assert cmd["speed_cap_mps"] == 4.0
    # Lost 9 m past (41, 41), still northbound: that junction is the nearest,
    # and it is behind the car. The next one north is 73 m on.
    cmd = SearchPlanner().step(0.0, (35.0, 41.0), _northbound(p=(50.0, 37.5)))
    assert cmd["mode"] == PURSUE and _close(cmd["target_xy"], (123.0, 41.0)), cmd


def test_the_pursue_speed_is_the_cars_within_1_5_to_5():
    for v, cap in ((8.0, 5.0), (0.4, 1.5), (None, 1.5), (3.2, 3.2)):
        cmd = SearchPlanner().step(0.0, (80.0, 41.0), _northbound(speed=v))
        assert cmd["speed_cap_mps"] == cap, (v, cmd["speed_cap_mps"])


def test_a_car_heading_west_is_pursued_west():
    """On the E-W street x = 41, westbound (its lane is south of the crown)."""
    a = {"p": (37.5, 20.0), "heading": -1.5, "speed": 3.0, "stopped": False}
    cmd = SearchPlanner().step(0.0, (38.0, 35.0), a)
    assert cmd["mode"] == PURSUE and _close(cmd["target_xy"], (41.0, -41.0))
    assert _ang(cmd["look_heading_rad"], W_) < 1e-9


# ------------------------------------------------------------------ dwell/hold

def test_dwell_looks_down_left_straight_right_and_never_back():
    rows = _fly(SearchPlanner(), _northbound(), (80.0, 41.0), t_end=40.0)
    modes = [c["mode"] for _, _, c in rows]
    order = {PURSUE: 0, DWELL: 1, HOLD: 2}
    assert all(order[a] <= order[b] for a, b in zip(modes, modes[1:])), modes
    dwell = [(t, c) for t, _, c in rows if c["mode"] == DWELL]
    names = []
    for _, c in dwell:
        if not names or names[-1] != c["exit"]:
            names.append(c["exit"])
    assert names == ["left", "straight", "right"], names
    want = {"left": W_, "straight": N, "right": E}   # arriving northbound
    for t, c in dwell:
        assert _ang(c["look_heading_rad"], want[c["exit"]]) < 1e-9, (t, c)
        assert _ang(c["look_heading_rad"], S) > 1.0     # never back down its street
        assert c["sweep_deg"] == 0.0
    for name in names:                                  # 2 s each, 0.25 s ticks
        n = sum(1 for _, c in dwell if c["exit"] == name)
        assert n * 0.25 == DWELL_PER_EXIT_S, (name, n)
    t_first = dwell[0][0]
    pos = [p for t, p, _ in rows if t == t_first][0]
    assert math.hypot(pos[0] - 123.0, pos[1] - 41.0) <= 4.0 + 1e-9


def test_keep_left_geometry_eastbound_left_is_north():
    """X north, Y east is left-handed from above: facing east, left is NORTH.
    Getting that backwards is how the lanes were once documented wrong."""
    a = {"p": (44.5, 0.0), "heading": E, "speed": 3.0, "stopped": False}
    p = SearchPlanner()
    exits = p.exits((41.0, 41.0), (0.0, 1.0))
    assert [n for n, _ in exits] == ["left", "straight", "right"]
    assert _ang(exits[0][1], N) < 1e-9 and _ang(exits[2][1], S) < 1e-9
    cmd = p.step(0.0, (44.0, -10.0), a)
    assert cmd["mode"] == PURSUE and _close(cmd["target_xy"], (41.0, 41.0))


def test_after_the_dwell_the_hold_turns_slowly_over_the_junction():
    rows = _fly(SearchPlanner(), _northbound(), (80.0, 41.0), t_end=29.75)
    hold = [(t, c) for t, _, c in rows if c["mode"] == HOLD]
    assert len(hold) > 20
    assert _ang(hold[0][1]["look_heading_rad"], E) < HOLD_RATE_RAD_S * 0.25 + 1e-9
    for (t0, a), (t1, b) in zip(hold, hold[1:]):
        step = (b["look_heading_rad"] - a["look_heading_rad"]) % (2 * math.pi)
        assert abs(step - HOLD_RATE_RAD_S * (t1 - t0)) < 1e-9
        assert _close(b["target_xy"], (123.0, 41.0))
        assert b["sweep_deg"] == HOLD_SWEEP_DEG and b["rotate_rad_s"] == HOLD_RATE_RAD_S
    assert HOLD_RATE_RAD_S < 0.25                # slow: a turn per ~30 s


def test_hold_persists_after_the_pursue_timeout():
    """An aircraft that never reaches the junction (held back by the fence,
    say) stops pursuing at 30 s and holds there - for good."""
    p = SearchPlanner()
    far = (60.0, 41.0)
    for t in (0.0, 10.0, 29.9):
        assert p.step(t, far, _northbound())["mode"] == PURSUE
    for t in (30.0, 45.0, 300.0):
        c = p.step(t, far, _northbound())
        assert c["mode"] == HOLD and _close(c["target_xy"], (123.0, 41.0)), (t, c)
    # A dwell cut short by the timeout: the hold starts from the exit it was on.
    p = SearchPlanner()
    for t in np.arange(0.0, 29.0, 0.5):
        p.step(t, far, _northbound())
    assert p.step(29.0, (122.0, 41.0), _northbound())["exit"] == "left"
    c = p.step(30.0, (122.0, 41.0), _northbound())
    assert c["mode"] == HOLD and _ang(c["look_heading_rad"], W_) < 1e-9


# ------------------------------------------------------------------ stopped

def test_a_stopped_car_is_not_overflown():
    """Stand off 15.8 m short of it on the aircraft's side, on the crown of
    its street, looking at it - from either side, with or without a heading."""
    car = (90.0, 37.5)
    stopped = {"p": car, "heading": N, "speed": 0.2, "stopped": True}
    cases = ((stopped, (60.0, 41.0), -1.0),
             (stopped, (112.0, 40.0), +1.0),               # it had overflown
             (dict(stopped, heading=None), (60.0, 41.0), -1.0))
    for anchor, start, side in cases:
        rows = _fly(SearchPlanner(), anchor, start, t_end=120.0, dt=0.5)
        for t, (x, y), c in rows:
            assert c["mode"] == STANDOFF, (anchor, t, c["mode"])
            tx, ty = c["target_xy"]
            assert _close((tx, ty), (car[0] + side * 15.8, 41.0)), (tx, ty)
            assert side * (x - car[0]) >= 15.8 - 1e-6, (t, x)   # never nearer
            assert c["sweep_deg"] == STANDOFF_SWEEP_DEG
            look_at_car = math.atan2(car[1] - ty, car[0] - tx)
            assert _ang(c["look_heading_rad"], look_at_car) < 1e-9
        assert side * (rows[-1][1][0] - car[0]) >= 15.8 - 1e-6   # arrived, short of it
    # Heading None and stopped is NOT "hold at the nearest junction": that is
    # (123, 41), beyond the car from an aircraft coming up from the south.
    c = SearchPlanner().step(0.0, (60.0, 41.0), dict(stopped, heading=None))
    assert c["mode"] == STANDOFF and c["target_xy"][0] < car[0]


def test_a_standoff_at_the_map_edge_stays_short_of_the_car_and_the_blind_floor():
    """The street behind the car runs off the mask 6 m past it. The old
    nearest-cell snap put the stand-off 7 m from the car, inside the 8.87 m
    floor. It must stay on the street, on the aircraft's side, outside it."""
    xs = -100.0 + 2.0 * np.arange(54)                   # x from -100 to 6
    ys = -20.0 + 2.0 * np.arange(21)                    # y from -20 to 20
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    mask = {"street": (np.abs(Y) <= 8.0).astype(np.uint8), "res": 2.0,
            "ox": -100.0, "oy": -20.0}
    car = (0.0, -3.5)
    p = SearchPlanner(junctions=[(-82.0, 0.0), (0.0, 0.0)], street=mask)
    c = p.step(0.0, (5.0, 0.0), {"p": car, "heading": N, "speed": 0.0,
                                 "stopped": True})
    tx, ty = c["target_xy"]
    assert c["mode"] == STANDOFF and c["on_street"] is True, c
    assert tx - car[0] >= 0.0, c                        # the aircraft's side
    assert math.hypot(tx - car[0], ty - car[1]) >= p.floor_m - 1e-9, c
    assert _ang(c["look_heading_rad"], math.atan2(car[1] - ty, car[0] - tx)) < 1e-9


def test_non_finite_or_missing_inputs_do_not_crash_or_restart():
    """round(nan) raises, and a NaN in the plan key never equals itself - a
    plan that restarted every tick would never leave its first second."""
    a = {"p": (100.0, 50.0), "heading": float("nan"), "speed": float("nan"),
         "stopped": False}
    p = SearchPlanner()
    c0 = p.step(0.0, (90.0, 45.0), a)
    assert c0["mode"] == HOLD and c0["junction"] == (123.0, 41.0)
    assert c0["speed_cap_mps"] == 1.5
    c1 = p.step(10.0, (90.0, 45.0), dict(a, heading=float("nan")))
    turned = (c1["look_heading_rad"] - c0["look_heading_rad"]) % (2 * math.pi)
    assert abs(turned - HOLD_RATE_RAD_S * 10.0) < 1e-9     # same plan, 10 s on
    c = SearchPlanner().step(0.0, (80.0, 41.0), _northbound(heading=float("inf")))
    assert c["mode"] == HOLD
    for stopped in (False, True):                      # no sighting at all
        c = SearchPlanner().step(0.0, (130.0, 45.0),
                                 {"p": None, "heading": N, "speed": 3.0,
                                  "stopped": stopped})
        assert c["mode"] == HOLD and c["junction"] == (123.0, 41.0), c


def test_the_standoff_is_never_inside_the_camera_blind_spot():
    """The bottom image row meets the ground 6.87 m out at 8 m - follow_vlm's
    0.86 x altitude - and no stand-off is allowed inside that plus 2 m."""
    assert abs(camera_blind_m(8.0) - 0.86 * 8.0) < 0.05
    p = SearchPlanner(standoff_m=3.0)
    assert p.standoff_m >= camera_blind_m(8.0) + 2.0 - 1e-9
    c = p.step(0.0, (60.0, 41.0), {"p": (90.0, 41.0), "heading": N, "speed": 0.0,
                                   "stopped": True})
    assert 90.0 - c["target_xy"][0] >= camera_blind_m(8.0) + 2.0 - 1e-9


# ------------------------------------------------------------------ no heading

def test_heading_none_holds_at_the_nearest_junction():
    a = {"p": (100.0, 50.0), "heading": None, "speed": 3.0, "stopped": False}
    p = SearchPlanner()
    c = p.step(0.0, (90.0, 45.0), a)
    assert c["mode"] == HOLD and c["junction"] == (123.0, 41.0)
    assert _close(c["target_xy"], (123.0, 41.0))
    assert _ang(c["look_heading_rad"], math.atan2(50.0 - 41.0, 100.0 - 123.0)) < 1e-9
    assert c["rotate_rad_s"] > 0.0 and p.step(50.0, (123.0, 41.0), a)["mode"] == HOLD


# ------------------------------------------------------------------ the mask

def test_a_missing_exit_is_not_dwelt_on():
    """A T-junction: the E-W street only runs WEST of (0, 0). Arriving
    northbound there is a left and a straight, and no right to look down."""
    x0 = y0 = -100.0
    xs = x0 + 2.0 * np.arange(101)
    X, Y = np.meshgrid(xs, xs, indexing="ij")
    street = ((np.abs(Y) <= 8.0) | ((np.abs(X) <= 8.0) & (Y <= 8.0))).astype(np.uint8)
    mask = {"street": street, "res": 2.0, "ox": x0, "oy": y0}
    p = SearchPlanner(junctions=[(-82.0, 0.0), (0.0, 0.0), (82.0, 0.0)], street=mask)
    assert [n for n, _ in p.exits((0.0, 0.0), (1.0, 0.0))] == ["left", "straight"]
    a = {"p": (-30.0, -3.5), "heading": N, "speed": 3.0, "stopped": False}
    rows = _fly(p, a, (-45.0, 0.0), t_end=30.0)
    seen = [c["exit"] for _, _, c in rows if c["mode"] == DWELL]
    assert seen and set(seen) == {"left", "straight"}, seen
    assert rows[-1][2]["mode"] == HOLD


def test_a_junction_centre_off_the_mask_is_moved_onto_the_street():
    """The real mask has a 2-4 m obstacle cell in the middle of (41, -41)."""
    mask = _lattice_mask(holes=[(123.0, 41.0)])
    assert not is_street(mask, 123.0, 41.0)
    c = SearchPlanner(street=mask).step(0.0, (80.0, 41.0), _northbound())
    assert c["on_street"] is True and is_street(mask, *c["target_xy"])
    assert 0.0 < math.hypot(c["target_xy"][0] - 123.0, c["target_xy"][1] - 41.0) <= 3.0
    c = SearchPlanner().step(0.0, (80.0, 41.0), _northbound())
    assert c["target_xy"] == (123.0, 41.0) and c["on_street"] is None


def _sweep_targets(planner_factory, mask, rng, n_anchors):
    """Lose a car at random places on the grid's streets, fly each episode,
    and return every (target, mode) the planner asked for."""
    out = []
    lines = [41.0 + 82.0 * k for k in range(-3, 4)]
    tried = 0
    while len(out) == 0 or tried < n_anchors:
        tried += 1
        on_ns = rng.random() < 0.5                 # N-S street: y fixed
        c = rng.choice(lines)
        s = rng.uniform(-140.0, 220.0) if on_ns else rng.uniform(-60.0, 140.0)
        lane = rng.choice((-3.5, 3.5))
        p = (s, c + lane) if on_ns else (c + lane, s)
        if not is_street(mask, *p):
            continue
        base = rng.choice((N, S)) if on_ns else rng.choice((E, W_))
        heading = None if rng.random() < 0.15 else base + rng.uniform(-0.3, 0.3)
        anchor = {"p": p, "heading": heading,
                  "speed": None if rng.random() < 0.1 else rng.uniform(0.0, 7.0),
                  "stopped": rng.random() < 0.25}
        back = base + math.pi
        start = (p[0] + 20.0 * math.cos(back), p[1] + 20.0 * math.sin(back))
        for _, _, cmd in _fly(planner_factory(), anchor, start, t_end=45.0, dt=0.5):
            out.append((cmd["target_xy"], cmd["mode"], anchor))
    return out


def test_the_planner_never_targets_off_the_street_mask():
    holes = [(41.0, -41.0), (123.0, 41.0), (-41.0, 41.0)]
    rng = random.Random(7)
    for _ in range(60):                             # holes on lane centres too
        holes.append((rng.choice((37.5, 44.5, 119.5)), rng.uniform(-60.0, 140.0)))
    mask = _lattice_mask(holes=holes)
    rows = _sweep_targets(lambda: SearchPlanner(street=mask), mask,
                          random.Random(11), 80)
    assert len(rows) > 1000
    bad = [(t, m, a) for t, m, a in rows if not is_street(mask, *t)]
    assert not bad, bad[:3]
    assert {m for _, m, _ in rows} == {PURSUE, DWELL, HOLD, STANDOFF}
    if REAL_MASK.is_file():                         # gitignored; checked when built
        real = load_street(REAL_MASK)
        rows = _sweep_targets(lambda: SearchPlanner(street=real), real,
                              random.Random(13), 80)
        bad = [(t, m, a) for t, m, a in rows if not is_street(real, *t)]
        assert not bad, bad[:3]
        print(f"      real mask: {len(rows)} targets, all on the street")
    else:
        print(f"      real mask absent ({REAL_MASK.name}): synthetic mask only")


# ------------------------------------------------------------------ plumbing

def test_a_new_anchor_or_time_going_back_restarts_the_plan():
    p = SearchPlanner()
    at = (123.0, 41.0)
    p.step(0.0, at, _northbound())
    assert p.step(1.0, at, _northbound())["mode"] == DWELL
    other = {"p": (37.5, 20.0), "heading": W_, "speed": 3.0, "stopped": False}
    c = p.step(2.0, (38.0, 35.0), other)
    assert c["mode"] == PURSUE and _close(c["target_xy"], (41.0, -41.0))
    p.step(5.0, (60.0, 41.0), _northbound())
    p.step(40.0, (60.0, 41.0), _northbound())       # timed out: HOLD
    assert p.step(0.0, (60.0, 41.0), _northbound())["mode"] == PURSUE


def test_the_sweeps_keep_the_street_in_frame():
    """A half-amplitude inside the 45 deg half-HFOV keeps the heading being
    searched on the image at the sweep's extremes (pinhole, not linear)."""
    for sweep in (PURSUE_SWEEP_DEG, STANDOFF_SWEEP_DEG):
        for sgn in (-1.0, 1.0):
            cx = bearing_to_cx(sgn * math.radians(sweep), IMG_W, HFOV)
            assert 0.0 < cx < IMG_W, (sweep, cx)


def test_the_plan_is_deterministic():
    a = _fly(SearchPlanner(), _northbound(), (80.0, 41.0), t_end=40.0)
    b = _fly(SearchPlanner(), _northbound(), (80.0, 41.0), t_end=40.0)
    assert a == b


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
