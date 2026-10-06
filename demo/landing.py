"""Where to put the aircraft down when the mission ends.

WHY

After the follow mission, demo/follow_vlm.py descends wherever the aircraft
happens to be. Twice that was on top of something:

    citylife_redcar_trail2   last logged position (18.16, 25.26) NED. It came
                             down on the hedge SM_jctHedgeC_137, with 2,525
                             contact events after the mission. The cell under
                             it is 2-4 m clutter, and a 6-14 m obstacle is
                             2.26 m away.
    citylife_ped_final       last logged position (37.51, -20.98). It came down
                             on the car BP_CityCar_C_8, with 40 contact events.
                             The point is 0.01 m from loop B's westbound lane
                             centre (x = 37.5), and by every map it is a CLEAN
                             street cell: the nearest 2-4 m clutter and the
                             nearest 6-14 m obstacle are both 4.71 m away. The
                             maps would have approved it. Only the lanes reject
                             it, because a moving car is in no occupancy map.

A landing site therefore has to be checked against two kinds of thing: what is
built (the maps) and where the cars drive (the lane centres they follow).

THE RULES (is_landable)

    1. On the grid, and not in the border ring of cells or the ring just inside
       it. Nothing is known beyond the grid, and a clearance disc near the edge
       would run off it.
    2. A STREET cell (demo/build_street_mask.py: free of 15-55 m buildings and
       of 2-4 m clutter). "Street" covers pavements and plazas as well as roads.
    3. No 2-4 m clutter within 2.0 m and no 6-14 m obstacle within 3.0 m. Each
       distance is to the nearest point of the occupied cell's SQUARE, not to
       its centre. A voxel column only says that something is somewhere inside
       those 2 x 2 m, so the near edge is the honest distance. Measuring to
       centres would pass a point 1.9 m from a cell edge as "2.9 m clear". A
       cell outside the grid counts as occupied, because unknown is not clear.
    4. At least 1.2 m + 0.9 m (car half-width) = 2.1 m from every lane-centre
       polyline, so not in a car lane. Pavements are fine.

CHOOSING (choose_site)

This returns the nearest landable cell by straight-line distance. The aircraft
transits at cruise altitude through the Shield, which handles clearance from
buildings. Even so, a site that the straight line reaches without crossing a
15-55 m building cell is preferred: if the line to the nearest site crosses
one, the next site is tried. A site that can only be reached across a building
is returned only when no other site exists within the search radius, and the
result then carries crosses_building = True.

WHAT THE RULES ALLOW THAT A PERSON WOULD NOT

Lane centres are 3.5 m either side of a two-way street's centre line
(tools/citylife_routes.py LANE_OFFSET_CM). That leaves a strip down the middle
of the road where a point is more than 2.1 m from both lanes, so it passes rule
4. For citylife_ped_final, the nearest site under the rules is (40.0, -20.0):
1 m off the centre line and 2.5 m from both lanes, so a passing car's edge is
1.6 m away on either side. It is legal under the rules, and nobody would land
there. With prefer="pavement", sites off the carriageway are ranked first
(further than 8.0 m + 1.2 m from every road centre line of the level's 82 m
street grid), and a carriageway site is returned only when no other site
exists. For ped_final that choice is (30.0, -20.0), 7.6 m away, on the southern
pavement 11 m from the centre line. For redcar_trail2 both modes choose
(22.0, 26.0), 3.9 m away, in the plaza beside the hedge.

Frames: NED metres, x = North, y = East. Map cell [i][j] (i north, j east) is
CENTRED at (origin_x + i*res, origin_y + j*res). That is the convention that
build_voxel_map.py writes and build_street_mask.is_street documents (round,
not int). Lane paths come from tools/citylife_routes.py in Unreal centimetres
with X north and Y east, so NED metres = cm / 100.

This module is pure apart from load_landing_maps (which reads files) and the
route import. Tests: tests/test_landing.py.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CITYMAP_DIR = ROOT / "demo" / "out" / "citymap_citylife"

Vec = Tuple[float, float]
LanePaths = Union[Dict[str, Sequence[Vec]], Sequence[Sequence[Vec]]]

LOW_CLEAR_M = 2.0             # no 2-4 m clutter nearer than this
CRUISE_CLEAR_M = 3.0          # no 6-14 m obstacle nearer than this
LANE_MARGIN_M = 1.2           # aircraft half-span plus drift, beyond a car's side
CAR_HALF_WIDTH_M = 0.9        # tools/citylife_routes.py CAR_HALF_WIDTH_CM / 100
LANE_CLEAR_M = LANE_MARGIN_M + CAR_HALF_WIDTH_M
EDGE_CELLS = 2                # valid cell index is in [2, n-3] on both axes
MAX_SEARCH_M = 60.0
# A SITE to fly to is chosen with every clearance this much wider than the
# touchdown rules above: the transit stops within 0.8 m of it and drifts on the
# way down. Staying put needs the rules alone - no transit, and a transit asked
# of a hovering aircraft in a crowd can be held by the Shield for good
# (citylife_ped_id: 25 m asked, 2 m flown, 387 Shield touches).
# citylife_redcar_id3 (2026-09-30) reached the nearest cell that passed and
# touched down 0.67 m off it, 2.94 m from a 6-14 m obstacle that needs 3.0 m.
# When no cell passes with the margin, the search runs again without it.
SITE_MARGIN_M = 1.0
CARRIAGEWAY_HALF_M = 8.0      # tools/citylife_routes.py CARRIAGEWAY_HALF_CM / 100

MAP_FILES = {
    "street": ("street.npz", "street"),
    "low": ("ground_2to4.npz", "occ"),
    "cruise": ("occ_day_flightband_6to14.npz", "occ"),
    "tall": ("occ_day_highband_15to55.npz", "occ"),
}


# ------------------------------------------------------------------- maps

@dataclass
class LandingMaps:
    """The four grids a landing decision reads, the lanes the cars drive, and
    the street grid. The arrays are bool [i = north][j = east], and cell [i][j]
    is centred at (origin_x + i*res, origin_y + j*res). The building index and
    the lane segments are cached at construction, so call __post_init__()
    again after editing an array or lane_paths in place."""
    street: np.ndarray
    low: np.ndarray               # 2-4 m clutter
    cruise: np.ndarray            # 6-14 m obstacles, the band flown at cruise
    tall: np.ndarray              # 15-55 m: only buildings reach it
    res: float
    origin_x: float
    origin_y: float
    lane_paths: Dict[str, List[Vec]] = field(default_factory=dict)
    road_grid: Optional[dict] = None     # see road_grid_from_routes
    source: str = ""

    def __post_init__(self):
        self.street = np.asarray(self.street).astype(bool)
        self.low = np.asarray(self.low).astype(bool)
        self.cruise = np.asarray(self.cruise).astype(bool)
        self.tall = np.asarray(self.tall).astype(bool)
        shapes = {a.shape for a in (self.street, self.low, self.cruise, self.tall)}
        if len(shapes) != 1:
            raise ValueError(f"grids disagree in shape: {sorted(shapes)}")
        self.res = float(self.res)
        self.origin_x = float(self.origin_x)
        self.origin_y = float(self.origin_y)
        self.lane_paths = _normalise_lanes(self.lane_paths)
        ti, tj = np.nonzero(self.tall)
        self._tall_x = self.origin_x + ti * self.res
        self._tall_y = self.origin_y + tj * self.res
        self._lanes = _LaneSet(self.lane_paths)

    @property
    def shape(self) -> Tuple[int, int]:
        return self.street.shape

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        """The cell whose square contains (x, y). Uses round, not int()."""
        return (int(round((x - self.origin_x) / self.res)),
                int(round((y - self.origin_y) / self.res)))

    def centre(self, i: int, j: int) -> Vec:
        return (self.origin_x + i * self.res, self.origin_y + j * self.res)

    def on_grid(self, i: int, j: int) -> bool:
        n, m = self.shape
        return 0 <= i < n and 0 <= j < m


def load_landing_maps(map_dir: Union[str, Path] = CITYMAP_DIR,
                      lane_paths: Optional[LanePaths] = None,
                      road_grid: Optional[dict] = None) -> LandingMaps:
    """Read the street mask and the three occupancy bands from `map_dir`.

    If `lane_paths` is None, the lanes are the CityLife car loops
    (lane_paths_from_routes). Pass {} to leave them out, but a site is then
    approved in the middle of a car lane, which is how ped_final ended. If
    `road_grid` is None, it is the level's street grid (road_grid_from_routes).
    Pass {} to leave it out, which disables prefer="pavement".
    """
    d = Path(map_dir)
    grids, meta = {}, None
    for name, (fname, key) in MAP_FILES.items():
        p = d / fname
        if not p.is_file():
            raise FileNotFoundError(
                f"missing {p}. Build the bands with demo/build_voxel_map.py and "
                f"the mask with: python demo/build_street_mask.py --dir {d}")
        z = np.load(p)
        grids[name] = z[key]
        here = (float(z["res"]), float(z["origin_x"]), float(z["origin_y"]))
        if meta is None:
            meta = here
        elif here != meta:
            raise ValueError(f"{fname} has res/origin {here}, the others {meta}")
    if lane_paths is None:
        lane_paths = lane_paths_from_routes()
    if road_grid is None:
        road_grid = road_grid_from_routes()
    return LandingMaps(street=grids["street"], low=grids["low"],
                       cruise=grids["cruise"], tall=grids["tall"],
                       res=meta[0], origin_x=meta[1], origin_y=meta[2],
                       lane_paths=lane_paths, road_grid=road_grid or None,
                       source=str(d))


# ------------------------------------------------------------------ routes

def _routes_module():
    tools = str(ROOT / "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import citylife_routes                                    # noqa: E402
    return citylife_routes


def lane_paths_from_routes(paths=None) -> Dict[str, List[Vec]]:
    """The CityLife car loops as closed NED-metre polylines {name: [(x, y)]}.

    `paths` is what citylife_routes.build_all() returns (Unreal cm, X north,
    Y east). If it is None, build_all() is called. The loops are closed (a car
    drives from the last point back to the first), so the first point is
    repeated at the end. That way a consumer treating the lane as an ordinary
    polyline still includes the closing segment."""
    if paths is None:
        paths = _routes_module().build_all()
    out = {}
    for name, p in paths.items():
        pts = [(float(x) / 100.0, float(y) / 100.0) for x, y in p.pts]
        if pts and pts[0] != pts[-1]:
            pts.append(pts[0])
        out[name] = pts
    return out


def road_grid_from_routes() -> dict:
    """The level's street grid: road centre lines at x = x_m + k*period_m
    (east-west streets) and y = y_m + k*period_m (north-south streets), in
    NED metres, each carriageway half_width_m either side.

    Read off citylife_routes: GRID_CM and the junctions of LOOPS, which must
    all sit on one grid. Every grid line inside the CityLife map is a street:
    tests/test_landing.py checks that at least 90 % of the cells along each
    line are on the street mask."""
    R = _routes_module()
    period = R.GRID_CM / 100.0
    xs = {round((jx / 100.0) % period, 6) for loop in R.LOOPS.values() for jx, _ in loop}
    ys = {round((jy / 100.0) % period, 6) for loop in R.LOOPS.values() for _, jy in loop}
    if len(xs) != 1 or len(ys) != 1:
        raise ValueError(f"junctions are not on one {period} m grid: x {xs}, y {ys}")
    return {"period_m": period, "x_m": xs.pop(), "y_m": ys.pop(),
            "half_width_m": R.CARRIAGEWAY_HALF_CM / 100.0}


def carriageway_distance(grid: dict, x: float, y: float) -> float:
    """Distance (m) from (x, y) to the nearest road centre line of the grid."""
    p = grid["period_m"]
    dx = abs((x - grid["x_m"] + p / 2.0) % p - p / 2.0)
    dy = abs((y - grid["y_m"] + p / 2.0) % p - p / 2.0)
    return min(dx, dy)


def on_carriageway(grid: dict, x: float, y: float) -> bool:
    """On the road, or within LANE_MARGIN_M of its kerb."""
    return carriageway_distance(grid, x, y) < grid["half_width_m"] + LANE_MARGIN_M


# ------------------------------------------------------------------- lanes

def _normalise_lanes(lane_paths: Optional[LanePaths]) -> Dict[str, List[Vec]]:
    if not lane_paths:
        return {}
    if isinstance(lane_paths, dict):
        items = lane_paths.items()
    else:
        items = ((str(k), v) for k, v in enumerate(lane_paths))
    return {str(k): [(float(p[0]), float(p[1])) for p in v] for k, v in items}


class _LaneSet:
    """All lane segments stacked, for one vectorised distance query."""

    def __init__(self, lanes: Dict[str, List[Vec]]):
        # A lane with no points contributes no segment, so it must not count
        # towards lanes_checked: that would report a rule that never ran.
        self.names = sorted(k for k, v in lanes.items() if len(v))
        p, q, owner = [], [], []
        for k, name in enumerate(self.names):
            pts = lanes[name]
            if len(pts) == 1:
                pts = [pts[0], pts[0]]
            for a, b in zip(pts[:-1], pts[1:]):
                p.append(a)
                q.append(b)
                owner.append(k)
        self.p = np.array(p, float).reshape(-1, 2)
        self.d = np.array(q, float).reshape(-1, 2) - self.p
        self.l2 = np.maximum((self.d ** 2).sum(axis=1), 1e-12)
        self.owner = np.array(owner, int)

    def __len__(self):
        return len(self.names)

    def nearest(self, x: float, y: float) -> Tuple[float, Optional[str]]:
        if not len(self.p):
            return math.inf, None
        u = ((x - self.p[:, 0]) * self.d[:, 0] + (y - self.p[:, 1]) * self.d[:, 1]) / self.l2
        u = np.clip(u, 0.0, 1.0)
        dist = np.hypot(x - self.p[:, 0] - u * self.d[:, 0],
                        y - self.p[:, 1] - u * self.d[:, 1])
        k = int(np.argmin(dist))
        return float(dist[k]), self.names[self.owner[k]]


def _lanes_for(maps: LandingMaps, lane_paths: Optional[LanePaths]) -> _LaneSet:
    return maps._lanes if lane_paths is None else _LaneSet(_normalise_lanes(lane_paths))


def lane_distance(maps: LandingMaps, x: float, y: float,
                  lane_paths: Optional[LanePaths] = None) -> Tuple[float, Optional[str]]:
    """(distance in m to the nearest lane-centre polyline, that lane's name).
    Returns (inf, None) when there are no lanes."""
    return _lanes_for(maps, lane_paths).nearest(x, y)


# --------------------------------------------------------------- clearance

def nearest_occupied_m(maps: LandingMaps, occ: np.ndarray, x: float, y: float,
                       radius: float) -> float:
    """Distance (m) from (x, y) to the nearest point of any occupied cell's
    square, if that is less than `radius`. Otherwise inf. Cells outside the
    grid count as occupied."""
    res = maps.res
    ci, cj = maps.cell_of(x, y)
    k = int(math.ceil(radius / res)) + 1
    ii = np.arange(ci - k, ci + k + 1)
    jj = np.arange(cj - k, cj + k + 1)
    I, J = np.meshgrid(ii, jj, indexing="ij")
    n, m = occ.shape
    inside = (I >= 0) & (I < n) & (J >= 0) & (J < m)
    hit = np.ones(I.shape, bool)
    hit[inside] = occ[I[inside], J[inside]]
    if not hit.any():
        return math.inf
    dx = np.maximum(np.abs(x - (maps.origin_x + I * res)) - res / 2.0, 0.0)
    dy = np.maximum(np.abs(y - (maps.origin_y + J * res)) - res / 2.0, 0.0)
    d = np.hypot(dx, dy)[hit]
    d = d[d < radius]
    return float(d.min()) if d.size else math.inf


def crosses_building(maps: LandingMaps, x0: float, y0: float,
                     x1: float, y1: float) -> bool:
    """Does the straight segment (x0, y0) -> (x1, y1) touch the square of any
    15-55 m building cell? This is an exact slab test against each square,
    not a sampled line, so a segment that only clips a corner still counts."""
    h = maps.res / 2.0
    lo_x, hi_x = min(x0, x1) - h, max(x0, x1) + h
    lo_y, hi_y = min(y0, y1) - h, max(y0, y1) + h
    sel = ((maps._tall_x >= lo_x) & (maps._tall_x <= hi_x)
           & (maps._tall_y >= lo_y) & (maps._tall_y <= hi_y))
    if not sel.any():
        return False
    cx, cy = maps._tall_x[sel], maps._tall_y[sel]
    t_lo = np.zeros(cx.shape)
    t_hi = np.ones(cx.shape)
    for a0, da, c in ((x0, x1 - x0, cx), (y0, y1 - y0, cy)):
        if abs(da) < 1e-12:
            inside = (a0 >= c - h) & (a0 <= c + h)
            t_hi = np.where(inside, t_hi, -1.0)
            continue
        ta, tb = (c - h - a0) / da, (c + h - a0) / da
        t_lo = np.maximum(t_lo, np.minimum(ta, tb))
        t_hi = np.minimum(t_hi, np.maximum(ta, tb))
    return bool((t_lo <= t_hi).any())


# ----------------------------------------------------------------- the rule

def _assess(maps: LandingMaps, x: float, y: float, lanes: _LaneSet,
            full: bool, margin: float = 0.0) -> Tuple[bool, List[str]]:
    """Rules 1-4 in order, cheapest first. With full=False, the check stops at
    the first failure (the search only needs a yes or no, plus a key to count).
    Every reason string starts with a stable key followed by a colon.
    `margin` widens the three clearances (choose_site's SITE_MARGIN_M)."""
    low_clear, cru_clear, lane_clear = (LOW_CLEAR_M + margin, CRUISE_CLEAR_M + margin,
                                        LANE_CLEAR_M + margin)
    reasons: List[str] = []
    if not (math.isfinite(x) and math.isfinite(y)):
        # A NaN pose (lost state after a crash) has no cell; it is not a site.
        return False, [f"off_map: ({x}, {y}) is not a finite position"]
    i, j = maps.cell_of(x, y)
    n, m = maps.shape
    if not maps.on_grid(i, j):
        x_lo, y_lo = maps.centre(0, 0)
        x_hi, y_hi = maps.centre(n - 1, m - 1)
        h = maps.res / 2.0
        return False, [f"off_map: ({x:.1f}, {y:.1f}) is outside the grid "
                       f"(x {x_lo - h:.0f}..{x_hi + h:.0f}, y {y_lo - h:.0f}..{y_hi + h:.0f})"]
    if min(i, j) < EDGE_CELLS or i > n - 1 - EDGE_CELLS or j > m - 1 - EDGE_CELLS:
        reasons.append(f"map_edge: cell ({i}, {j}) is within 1 cell of the grid's edge")
        if not full:
            return False, reasons
    if not maps.street[i, j]:
        what = ("a 15-55 m building footprint" if maps.tall[i, j]
                else "2-4 m clutter" if maps.low[i, j] else "off the street mask")
        reasons.append(f"not_street: cell ({i}, {j}) is {what}")
        if not full:
            return False, reasons
    d_low = nearest_occupied_m(maps, maps.low, x, y, low_clear)
    if d_low < low_clear:
        reasons.append(f"clutter_2to4: 2-4 m clutter {d_low:.2f} m away, "
                       f"needs {low_clear:.1f} m")
        if not full:
            return False, reasons
    d_cru = nearest_occupied_m(maps, maps.cruise, x, y, cru_clear)
    if d_cru < cru_clear:
        reasons.append(f"obstacle_6to14: 6-14 m obstacle {d_cru:.2f} m away, "
                       f"needs {cru_clear:.1f} m")
        if not full:
            return False, reasons
    d_lane, name = lanes.nearest(x, y)
    if d_lane < lane_clear:
        extra = f" + {margin:g} m site margin" if margin else ""
        reasons.append(f"car_lane: {d_lane:.2f} m from lane {name}'s centre, needs "
                       f"{lane_clear:.1f} m ({LANE_MARGIN_M} m + {CAR_HALF_WIDTH_M} m "
                       f"car half-width{extra})")
    return not reasons, reasons


def is_landable(maps: LandingMaps, x: float, y: float,
                lane_paths: Optional[LanePaths] = None) -> Tuple[bool, List[str]]:
    """(True, []) if the aircraft may touch down at (x, y). Otherwise
    (False, reasons), with every failed rule listed and each reason starting
    with its key: off_map, map_edge, not_street, clutter_2to4, obstacle_6to14,
    car_lane. `lane_paths` None means the lanes stored in `maps`."""
    return _assess(maps, x, y, _lanes_for(maps, lane_paths), full=True)


def reason_keys(reasons: Sequence[str]) -> List[str]:
    return [r.split(":", 1)[0] for r in reasons]


# --------------------------------------------------------------- the choice

CLEARANCE_REPORT_M = 10.0     # clearances are reported up to this; beyond: None


def _clearances(maps: LandingMaps, x: float, y: float, lanes: _LaneSet) -> dict:
    """Measured distances at a site, for the log. A value of None for the
    clutter or obstacle distance means none within CLEARANCE_REPORT_M. For the
    lane, None means no lanes were loaded."""
    far = CLEARANCE_REPORT_M

    def cap(v):
        return None if math.isinf(v) else round(v, 2)

    d_lane, name = lanes.nearest(x, y)
    return {"clutter_2to4_m": cap(nearest_occupied_m(maps, maps.low, x, y, far)),
            "obstacle_6to14_m": cap(nearest_occupied_m(maps, maps.cruise, x, y, far)),
            "lane_m": cap(d_lane), "lane": name}


def choose_site(maps: LandingMaps, x0: float, y0: float,
                lane_paths: Optional[LanePaths] = None,
                max_search_m: float = MAX_SEARCH_M,
                prefer: Optional[str] = None,
                margin_m: float = SITE_MARGIN_M) -> dict:
    """Pick a landing site for an aircraft now at (x0, y0) NED.

    The result is a dict with these keys:
        site                   (x, y), or None if no site exists within max_search_m
        path_m                 straight-line distance to it
        reasons_rejected_here  why (x0, y0) itself is not landable ([] if it is)
        here_landable          bool
        crosses_building       whether the straight line crosses a 15-55 m cell
        on_carriageway         site within the street grid's carriageway + margin
                               (None without a road grid)
        cell, clearance        the site's cell and its measured clearances
        n_examined, rejected   candidates tried, and a count by first failed rule
        lanes_checked          number of lane polylines. 0 means rule 4 never
                               ran, and that is not the same as no car nearby.
        margin_m               the clearance margin the site was chosen with:
                               margin_m, or 0.0 when no cell passed with it

    here_landable and reasons_rejected_here are the touchdown rules as they
    stand, and they alone decide whether to stay put; only a site to fly to
    needs the margin.

    The ranking is: (1) not crossing a building, then (2) with prefer="pavement",
    off the carriageway, then (3) nearest. When (x0, y0) is itself landable (and,
    with prefer="pavement", off the carriageway), it is returned with path 0.
    Candidates are cell centres within max_search_m, taken in order of
    distance, with ties broken by cell index so that the result is
    deterministic.
    """
    if prefer not in (None, "pavement"):
        raise ValueError(f"prefer must be None or 'pavement', not {prefer!r}")
    grid = maps.road_grid
    if prefer == "pavement" and not grid:
        raise ValueError("prefer='pavement' needs maps.road_grid (road_grid_from_routes)")
    lanes = _lanes_for(maps, lane_paths)
    here_ok, here_reasons = _assess(maps, x0, y0, lanes, full=True)

    def carriage(x, y):
        return on_carriageway(grid, x, y) if grid else None

    def tier(cross, carr):
        return 2 * int(cross) + int(bool(carr) and prefer == "pavement")

    for margin in ((margin_m, 0.0) if margin_m > 0.0 else (0.0,)):
        best, rejected, n_examined = _search(maps, x0, y0, lanes, max_search_m, here_ok,
                                             margin, carriage, tier)
        if best:
            break

    out = {"site": None, "path_m": None, "reasons_rejected_here": here_reasons,
           "here_landable": here_ok, "crosses_building": None,
           "on_carriageway": None, "cell": None, "clearance": None,
           "n_examined": n_examined, "rejected": rejected,
           "lanes_checked": len(lanes), "prefer": prefer,
           "max_search_m": max_search_m, "margin_m": margin if best else None}
    if best:
        d, x, y, cross = best[min(best)]
        out.update(site=(x, y), path_m=round(d, 2), crosses_building=cross,
                   on_carriageway=carriage(x, y), cell=maps.cell_of(x, y),
                   clearance=_clearances(maps, x, y, lanes))
    return out


def _search(maps, x0, y0, lanes, max_search_m, here_ok, margin, carriage, tier):
    """choose_site's search at one clearance margin: ({tier: (d, x, y,
    crosses)}, rejected-by-rule counts, cells examined)."""
    best: Dict[int, tuple] = {}
    if here_ok:
        best[tier(False, carriage(x0, y0))] = (0.0, float(x0), float(y0), False)
    rejected: Dict[str, int] = {}
    n_examined = 0
    if 0 not in best and math.isfinite(x0) and math.isfinite(y0):
        n, m = maps.shape
        res = maps.res
        r = float(max_search_m)
        # np.floor/np.ceil keep +-inf as floats, so max_search_m = inf means
        # "the whole grid" instead of an OverflowError from int(-inf).
        i_lo = int(max(0.0, np.floor((x0 - r - maps.origin_x) / res)))
        i_hi = int(min(n - 1.0, np.ceil((x0 + r - maps.origin_x) / res)))
        j_lo = int(max(0.0, np.floor((y0 - r - maps.origin_y) / res)))
        j_hi = int(min(m - 1.0, np.ceil((y0 + r - maps.origin_y) / res)))
        cands = []
        if i_lo <= i_hi and j_lo <= j_hi:
            I, J = np.meshgrid(np.arange(i_lo, i_hi + 1), np.arange(j_lo, j_hi + 1),
                               indexing="ij")
            X = maps.origin_x + I * res
            Y = maps.origin_y + J * res
            D = np.hypot(X - x0, Y - y0)
            keep = D <= r
            order = np.lexsort((J[keep], I[keep], D[keep]))
            cands = list(zip(D[keep][order], X[keep][order], Y[keep][order]))
        for d, x, y in cands:
            n_examined += 1
            ok, why = _assess(maps, float(x), float(y), lanes, full=False, margin=margin)
            if not ok:
                key = reason_keys(why)[0]
                rejected[key] = rejected.get(key, 0) + 1
                continue
            cross = crosses_building(maps, x0, y0, float(x), float(y))
            t = tier(cross, carriage(float(x), float(y)))
            if t not in best:
                best[t] = (float(d), float(x), float(y), cross)
            if t == 0:
                break
    return best, rejected, n_examined


def plan_transit(x0: float, y0: float, site: Vec, step_m: float = 2.0) -> List[Vec]:
    """Waypoints along the straight line from (x0, y0) to `site`, no more than
    step_m apart. The start is excluded and the site is the last point. They
    are horizontal only: the transit is flown at cruise altitude, through the
    Shield, and the descent starts at the last waypoint."""
    if site is None:
        raise ValueError("no site: choose_site found none within max_search_m")
    if not step_m > 0:
        raise ValueError("step_m must be positive")
    dx, dy = site[0] - x0, site[1] - y0
    d = math.hypot(dx, dy)
    k = max(1, int(math.ceil(d / step_m - 1e-9)))
    pts = [(x0 + dx * s / k, y0 + dy * s / k) for s in range(1, k)]
    pts.append((float(site[0]), float(site[1])))
    return pts


if __name__ == "__main__":
    import json
    mp = load_landing_maps()
    for label, xy in (("redcar_trail2 hedge", (18.16, 25.26)),
                      ("ped_final car", (37.51, -20.98))):
        for pref in (None, "pavement"):
            r = choose_site(mp, *xy, prefer=pref)
            print(label, pref, json.dumps(r, default=str))
