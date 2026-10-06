"""Choosing where to land after the mission: demo/landing.py.

Run either way:
    pytest tests/test_landing.py -v
    python tests/test_landing.py

Two flights came down on top of something. citylife_redcar_trail2 landed on a
hedge (2,525 contact events), and citylife_ped_final landed on a car in loop
B's lane (40 contact events). These tests pin that both end points are refused,
each for the right reason, and that a site is found near each of them.

Frames: NED metres, x = North, y = East. Map cell [i][j] is centred at
origin + index * res.

The real-map tests need demo/out/citymap_citylife/ and the two flight logs,
and all of those are gitignored. When they are missing, those tests print SKIP
rather than PASS. The synthetic-map tests always run.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))
sys.path.insert(0, str(ROOT / "tools"))

import citylife_routes as R                                    # noqa: E402
from landing import (CAR_HALF_WIDTH_M, CARRIAGEWAY_HALF_M,       # noqa: E402
                     CITYMAP_DIR, CRUISE_CLEAR_M, LANE_CLEAR_M, SITE_MARGIN_M,
                     LandingMaps,
                     carriageway_distance, choose_site, crosses_building,
                     is_landable, lane_paths_from_routes, load_landing_maps,
                     on_carriageway, plan_transit, reason_keys,
                     road_grid_from_routes)

SKIP = "SKIP"
RUN_DIR = ROOT / "demo" / "out"
HEDGE_RUN = "citylife_redcar_trail2"
CAR_RUN = "citylife_ped_final"
HEDGE_TOUCHDOWN = (17.9, 25.4)       # where the simulator put it on the hedge


def _flat(n=40, m=40, lanes=None, grid=None):
    """A 40 x 40 grid of 2 m street cells, origin (0, 0): x, y in [-1, 79]."""
    z = np.zeros((n, m), np.uint8)
    return LandingMaps(street=np.ones((n, m), np.uint8), low=z.copy(),
                       cruise=z.copy(), tall=z.copy(), res=2.0,
                       origin_x=0.0, origin_y=0.0, lane_paths=lanes or {},
                       road_grid=grid)


_REAL = {}


def _real_maps():
    if "maps" not in _REAL:
        try:
            _REAL["maps"] = load_landing_maps(CITYMAP_DIR)
        except FileNotFoundError:
            _REAL["maps"] = None
    return _REAL["maps"]


def _last_xy(run):
    log = RUN_DIR / run / "flight_log.jsonl"
    if not log.is_file():
        return None
    rows = [l for l in log.read_text(encoding="utf-8").splitlines() if l.strip()]
    r = json.loads(rows[-1])
    return float(r["x"]), float(r["y"])


# ------------------------------------------------------------ synthetic maps

def test_off_map_is_rejected():
    mp = _flat()
    for p in ((-50.0, 40.0), (40.0, 80.0), (-2.0, 40.0)):
        ok, why = is_landable(mp, *p)
        assert not ok and reason_keys(why) == ["off_map"], (p, why)


def test_the_two_outer_rings_of_cells_are_refused():
    """Beyond the grid nothing is known, so the border cell and the one inside
    it are refused and the third is not."""
    mp = _flat()
    for i, ok_expected in ((0, False), (1, False), (2, True), (37, True),
                           (38, False), (39, False)):
        ok, why = is_landable(mp, *mp.centre(i, 20))
        assert ok is ok_expected, (i, why)
        if not ok_expected:
            assert "map_edge" in reason_keys(why), why


def test_clearance_is_measured_to_the_cell_edge_not_its_centre():
    """An occupied cell says only that something is somewhere in its 2 x 2 m.
    A point 2.9 m from its centre can be 1.9 m from the thing."""
    mp = _flat()
    mp.low[20, 20] = True                   # centre (40, 40), square 39..41
    ok, why = is_landable(mp, 40.0, 42.9)
    assert not ok and reason_keys(why) == ["clutter_2to4"], why
    assert is_landable(mp, 40.0, 43.1)[0]
    mp = _flat()
    mp.cruise[20, 20] = True
    ok, why = is_landable(mp, 40.0, 43.9)
    assert not ok and reason_keys(why) == ["obstacle_6to14"], why
    assert is_landable(mp, 40.0, 44.1)[0]
    ok, why = is_landable(mp, 42.9, 42.9)   # diagonal: 1.9 m to the corner x 2
    assert not ok and "obstacle_6to14" in reason_keys(why), why


def test_a_cell_off_the_street_mask_is_refused():
    mp = _flat()
    mp.street[20, 20] = False
    mp.tall[20, 20] = True
    ok, why = is_landable(mp, 40.0, 40.0)
    assert not ok and reason_keys(why) == ["not_street"], why
    assert "building" in why[0], why


def test_a_lane_centre_is_refused_and_the_pavement_beside_it_is_not():
    """At least 1.2 m + 0.9 m (car half-width) from the lane centre."""
    mp = _flat(lanes={"L": [(40.0, -10.0), (40.0, 90.0)]})
    ok, why = is_landable(mp, 40.0, 30.0)
    assert not ok and reason_keys(why) == ["car_lane"], why
    assert not is_landable(mp, 42.0, 30.0)[0]                  # 2.0 m
    assert is_landable(mp, 40.0 + LANE_CLEAR_M, 30.0)[0]
    assert is_landable(mp, 44.0, 30.0)[0]
    # An explicit lane list overrides the lanes stored in the maps.
    assert is_landable(mp, 40.0, 30.0, lane_paths={})[0]
    assert not is_landable(_flat(), 20.0, 30.0,
                           lane_paths=[[(20.0, 0.0), (20.0, 60.0)]])[0]


def test_choose_site_stays_put_when_it_can():
    r = choose_site(_flat(), 40.0, 40.0)
    assert r["site"] == (40.0, 40.0) and r["path_m"] == 0.0, r
    assert r["here_landable"] and r["reasons_rejected_here"] == []


def test_a_site_is_chosen_with_room_for_drift_and_only_then_without():
    """citylife_redcar_id3 reached the nearest cell that passed, touched down
    0.67 m off it and ended 2.94 m from a 6-14 m obstacle that needs 3.0 m.
    A site keeps SITE_MARGIN_M more than each touchdown clearance; with no
    such cell in reach the search falls back to the bare rules."""
    mp = _flat()
    mp.cruise[20, 20] = True                    # a 6-14 m obstacle at (40, 40)
    mp.street[:, :21] = False                   # street only north of it...
    mp.street[:, 21:] = True
    mp.street[20:22, :] = False                 # ...but not on the aircraft's row
    mp.__post_init__()
    x0, y0 = 40.0, 40.0                         # on the obstacle: a site is needed
    r = choose_site(mp, x0, y0)
    assert not r["here_landable"] and r["path_m"] > 0.0, r
    assert r["margin_m"] == SITE_MARGIN_M, r
    assert r["clearance"]["obstacle_6to14_m"] >= CRUISE_CLEAR_M + SITE_MARGIN_M, r
    # the bare rules would have taken the first cell that passes
    r0 = choose_site(mp, x0, y0, margin_m=0.0)
    assert r0["margin_m"] == 0.0 and r0["path_m"] < r["path_m"], (r0, r)
    assert r0["clearance"]["obstacle_6to14_m"] < CRUISE_CLEAR_M + SITE_MARGIN_M, r0
    # nowhere with the margin within reach: the bare rules decide
    r1 = choose_site(mp, x0, y0, max_search_m=r0["path_m"] + 0.01)
    assert r1["site"] == r0["site"] and r1["margin_m"] == 0.0, r1


def test_staying_put_needs_the_touchdown_rules_alone():
    """No transit, no transit drift: a hover point that passes the rules is
    kept even without the margin (citylife_ped_id was sent 25 m through a
    crowd from such a point, and the Shield held it after 2 m)."""
    mp = _flat()
    mp.cruise[20, 20] = True
    mp.__post_init__()
    x0, y0 = 40.0, 44.3                         # 3.3 m from the obstacle's square
    assert is_landable(mp, x0, y0)[0]
    r = choose_site(mp, x0, y0)
    assert r["site"] == (x0, y0) and r["path_m"] == 0.0, r


def test_choose_site_skips_a_site_behind_a_building():
    """Start in a lane at (40, 40) with only the north side open and a
    building wall at x = 42. The nearest landable cell, (44, 40), is behind the
    wall, so the chosen site must be one the straight line reaches without
    crossing it."""
    mp = _flat(lanes={"L": [(40.0, -10.0), (40.0, 90.0)]})
    mp.street[:20, :] = False
    mp.tall[21, 17:24] = True
    mp.street[21, 17:24] = False
    mp.__post_init__()                          # refresh the building index
    assert is_landable(mp, 44.0, 40.0)[0]
    assert crosses_building(mp, 40.0, 40.0, 44.0, 40.0)
    r = choose_site(mp, 40.0, 40.0)
    assert r["site"] is not None and r["crosses_building"] is False, r
    assert r["path_m"] > 4.0, r
    assert is_landable(mp, *r["site"])[0]
    assert not crosses_building(mp, 40.0, 40.0, *r["site"])


def test_choose_site_crosses_a_building_only_when_it_must_and_says_so():
    """A courtyard of clutter walled by a building ring: every site is outside
    the ring, so the site is returned with crosses_building = True."""
    mp = _flat()
    mp.street[17:24, 17:24] = False
    mp.tall[17:24, 17:24] = True
    mp.tall[18:23, 18:23] = False
    mp.low[18:23, 18:23] = True
    mp.__post_init__()
    r = choose_site(mp, 40.0, 40.0)
    assert r["site"] is not None and r["crosses_building"] is True, r
    assert is_landable(mp, *r["site"])[0]


def test_choose_site_returns_no_site_when_there_is_none():
    mp = _flat()
    mp.street[:, :] = False
    r = choose_site(mp, 40.0, 40.0, max_search_m=10.0)
    assert r["site"] is None and r["path_m"] is None, r
    assert r["n_examined"] > 0 and r["rejected"] == {"not_street": r["n_examined"]}, r


def test_choose_site_says_when_the_lane_rule_never_ran():
    """An empty lane set passes every point, which also describes a map with
    no cars. The result has to say which of the two it was."""
    assert choose_site(_flat(), 40.0, 40.0)["lanes_checked"] == 0
    mp = _flat(lanes={"L": [(40.0, -10.0), (40.0, 90.0)]})
    assert choose_site(mp, 40.0, 40.0)["lanes_checked"] == 1


def test_pavement_preference_moves_off_the_carriageway():
    """Two lanes 3.5 m either side of a road centre line at x = 40. The centre
    strip is 3.5 m from both lanes, so it passes the lane rule. That is legal,
    and nobody would land there."""
    grid = {"period_m": 1000.0, "x_m": 40.0, "y_m": -400.0, "half_width_m": 8.0}
    mp = _flat(lanes={"E": [(43.5, -10.0), (43.5, 90.0)],
                      "W": [(36.5, 90.0), (36.5, -10.0)]}, grid=grid)
    r = choose_site(mp, 40.0, 40.0)
    assert r["site"] == (40.0, 40.0) and r["on_carriageway"] is True, r
    r = choose_site(mp, 40.0, 40.0, prefer="pavement")
    assert r["on_carriageway"] is False, r
    assert carriageway_distance(grid, *r["site"]) >= 8.0 + 1.2, r
    assert abs(r["path_m"] - 10.0) < 1e-9, r
    try:
        choose_site(_flat(), 40.0, 40.0, prefer="pavement")
        raise AssertionError("pavement without a road grid must refuse")
    except ValueError:
        pass


def test_non_finite_positions_and_empty_lanes_do_not_crash_or_pass_silently():
    """A NaN pose (state lost after a crash) is refused as off_map and the
    search returns no site rather than raising. max_search_m = inf searches
    the whole grid. A lane with no points is not counted as a checked lane,
    because its rule never ran."""
    mp = _flat()
    for p in ((math.nan, 40.0), (40.0, math.inf)):
        ok, why = is_landable(mp, *p)
        assert not ok and reason_keys(why) == ["off_map"], (p, why)
        r = choose_site(mp, *p)
        assert r["site"] is None and r["n_examined"] == 0, r
    mp = _flat()
    mp.street[:, :] = False
    mp.street[35, 35] = True                     # the only street cell, (70, 70)
    r = choose_site(mp, 4.0, 4.0, max_search_m=math.inf)
    assert r["site"] == (70.0, 70.0), r
    assert choose_site(_flat(lanes={"E": []}), 40.0, 40.0)["lanes_checked"] == 0
    try:
        plan_transit(0.0, 0.0, None)
        raise AssertionError("plan_transit must refuse a missing site")
    except ValueError:
        pass


def test_plan_transit_is_a_straight_line_in_short_steps():
    wps = plan_transit(0.0, 0.0, (10.0, 0.0), step_m=2.0)
    assert len(wps) == 5 and wps[-1] == (10.0, 0.0), wps
    pts = [(0.0, 0.0)] + wps
    steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    assert max(steps) <= 2.0 + 1e-9, steps
    wps = plan_transit(1.0, 2.0, (4.0, 6.0), step_m=2.0)       # 5 m -> 3 steps
    assert len(wps) == 3 and wps[-1] == (4.0, 6.0)
    assert all(abs((y - 2.0) * 3.0 - (x - 1.0) * 4.0) < 1e-9 for x, y in wps)
    assert plan_transit(3.0, 3.0, (3.0, 3.0)) == [(3.0, 3.0)]


# ------------------------------------------------------------ routes and grid

def test_lanes_from_routes_are_closed_metre_polylines():
    lanes = lane_paths_from_routes()
    paths = R.build_all()
    assert sorted(lanes) == sorted(paths)
    for k, pts in lanes.items():
        assert pts[0] == pts[-1], k
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        assert abs(length - paths[k].length_cm / 100.0) < 0.01, (k, length)
    # Loop B drives west along x = 37.5 (keep left on the x = 41 street), and
    # ped_final ended there. Loop A drives east along x = 44.5.
    b = np.array(lanes["B"])
    on = b[(np.abs(b[:, 1] + 21.0) < 1.0) & (b[:, 0] < 60.0)]   # not its x=126.5 leg
    assert len(on) and np.all(np.abs(on[:, 0] - 37.5) < 0.01), on
    a = np.array(lanes["A"])
    assert np.any((np.abs(a[:, 0] - 44.5) < 0.01) & (np.abs(a[:, 1] - 80.0) < 1.0))


def test_constants_agree_with_the_routes():
    assert CAR_HALF_WIDTH_M == R.CAR_HALF_WIDTH_CM / 100.0
    assert CARRIAGEWAY_HALF_M == R.CARRIAGEWAY_HALF_CM / 100.0
    g = road_grid_from_routes()
    assert g == {"period_m": 82.0, "x_m": 41.0, "y_m": 41.0, "half_width_m": 8.0}, g
    assert abs(carriageway_distance(g, 37.5, -21.0) - 3.5) < 1e-9
    assert abs(carriageway_distance(g, 30.0, 0.0) - 11.0) < 1e-9
    assert on_carriageway(g, 40.0, -20.0) and not on_carriageway(g, 30.0, -20.0)


# ------------------------------------------------------------ the real maps

def test_every_street_grid_line_is_a_street():
    """What prefer="pavement" rests on: the level's roads are the 82 m grid.
    Measured 2026-09-29 on the street mask: every grid line is 0.96-1.00
    street along its length."""
    mp = _real_maps()
    if mp is None:
        return SKIP
    g = mp.road_grid
    n, m = mp.shape
    x_lo, y_lo = mp.centre(0, 0)
    x_hi, y_hi = mp.centre(n - 1, m - 1)
    lines = 0
    for k in range(-5, 6):
        v = g["x_m"] + k * g["period_m"]
        if x_lo <= v <= x_hi:
            i = int(round((v - mp.origin_x) / mp.res))
            assert mp.street[i, :].mean() >= 0.9, ("x", v, mp.street[i, :].mean())
            lines += 1
        v = g["y_m"] + k * g["period_m"]
        if y_lo <= v <= y_hi:
            j = int(round((v - mp.origin_y) / mp.res))
            assert mp.street[:, j].mean() >= 0.9, ("y", v, mp.street[:, j].mean())
            lines += 1
    assert lines >= 6, lines


def test_the_hedge_end_of_redcar_trail2_is_not_landable():
    """It came down on SM_jctHedgeC_137. The cell under it is 2-4 m clutter,
    and a 6-14 m obstacle is 2.26 m away."""
    mp = _real_maps()
    xy = _last_xy(HEDGE_RUN)
    if mp is None or xy is None:
        return SKIP
    for p in (xy, HEDGE_TOUCHDOWN):
        ok, why = is_landable(mp, *p)
        keys = reason_keys(why)
        assert not ok and "not_street" in keys and "clutter_2to4" in keys, (p, why)
        assert "car_lane" not in keys, why


def test_the_car_end_of_ped_final_is_not_landable_and_only_the_lanes_know():
    """It came down on BP_CityCar_C_8 in loop B's lane. By every occupancy
    map that point is a clean street cell. The lanes are the only thing that
    refuses it, which is why a lane-free landing check would have approved it."""
    mp = _real_maps()
    xy = _last_xy(CAR_RUN)
    if mp is None or xy is None:
        return SKIP
    ok, why = is_landable(mp, *xy)
    assert not ok and reason_keys(why) == ["car_lane"], why
    assert "lane B" in why[0], why
    assert is_landable(mp, *xy, lane_paths={})[0]


def test_choose_site_finds_a_landable_site_near_each_flight_end():
    mp = _real_maps()
    ends = {run: _last_xy(run) for run in (HEDGE_RUN, CAR_RUN)}
    if mp is None or None in ends.values():
        return SKIP
    for run, xy in ends.items():
        for prefer in (None, "pavement"):
            r = choose_site(mp, *xy, max_search_m=60.0, prefer=prefer)
            assert r["site"] is not None and r["path_m"] <= 60.0, (run, prefer, r)
            assert not r["here_landable"] and r["reasons_rejected_here"], r
            assert is_landable(mp, *r["site"])[0], (run, prefer, r)
            assert r["crosses_building"] is False, (run, prefer, r)
            assert r["lanes_checked"] == 3, r
            print(f"      {run:24s} prefer={str(prefer):8s} -> site {r['site']} "
                  f"{r['path_m']:.2f} m, carriageway={r['on_carriageway']}")


def test_ped_final_nearest_site_is_on_the_carriageway_until_pavement_is_preferred():
    """The lane rule leaves strips of a two-way street open: for ped_final the
    nearest site is on the carriageway, clear of both lanes by the touchdown
    rule plus the site margin (it was 1 m off the centre line, cars 1.6 m
    away, before SITE_MARGIN_M). prefer="pavement" moves it off the
    carriageway."""
    mp = _real_maps()
    xy = _last_xy(CAR_RUN)
    if mp is None or xy is None:
        return SKIP
    r = choose_site(mp, *xy)
    assert r["on_carriageway"] is True and r["margin_m"] == SITE_MARGIN_M, r
    assert r["clearance"]["lane_m"] >= LANE_CLEAR_M + SITE_MARGIN_M, r
    p = choose_site(mp, *xy, prefer="pavement")
    assert p["on_carriageway"] is False, p
    assert carriageway_distance(mp.road_grid, *p["site"]) >= 9.2, p


def test_a_real_lane_centre_is_rejected():
    mp = _real_maps()
    if mp is None:
        return SKIP
    for p in ((37.5, 0.0), (44.5, 80.0)):       # loop B westbound, loop A east
        ok, why = is_landable(mp, *p)
        assert not ok and "car_lane" in reason_keys(why), (p, why)


def test_a_pavement_point_beside_a_building_free_kerb_is_accepted():
    """(30, -20) is on the southern pavement of the x = 41 street, 11 m from
    its centre line. The kerb is at x = 33, and the cells between the point and
    the kerb are street, with no building."""
    mp = _real_maps()
    if mp is None:
        return SKIP
    x, y = 30.0, -20.0
    assert 8.0 < carriageway_distance(mp.road_grid, x, y) < 16.0
    j = mp.cell_of(x, y)[1]
    for i in range(mp.cell_of(x, y)[0], mp.cell_of(33.0, y)[0] + 1):
        assert mp.street[i, j] and not mp.tall[i, j], (i, j)
    ok, why = is_landable(mp, x, y)
    assert ok, why


def test_off_map_is_rejected_on_the_real_map():
    mp = _real_maps()
    if mp is None:
        return SKIP
    for p in ((-500.0, 0.0), (0.0, 500.0), (300.0, 0.0)):
        ok, why = is_landable(mp, *p)
        assert not ok and reason_keys(why) == ["off_map"], (p, why)


if __name__ == "__main__":
    # A test that returns early on a missing fixture must NOT print PASS: on a
    # clean clone demo/out/ is gitignored and those tests assert nothing.
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (fixture missing)")
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
