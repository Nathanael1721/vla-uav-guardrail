"""Where the CityLife level's pedestrians and cars go.

The level is a copy of JapaneseCity/Demo_day with actors added to it, so the
geometry the demo already surveyed is the geometry these actors stand on. That
survey is `demo/out/citymap/street.npz` (an 80x80 street mask at 2 m, origin
-80,-80 m) plus `occ_day_highband_15to55.npz` (building footprints in the same
frame), and reusing it is the point: placements derived from the same grid the
existing tests assert against cannot disagree with them.

Pedestrians use the rule `demo/pedestrians.py::pavement_spots` already settled
on - a cell that is street AND touches a building is a pavement, not a
carriageway - because a figure standing in the lane is the one placement
mistake that costs the demo its lock.

Cars follow the circuit `demo/city_traffic.py` already validated: legs at
x = 34 and x = 44, y from -16 to 66.

NED metres -> Unreal centimetres is `NedToUnrealLinear` in
Plugins/ProjectAirSim/.../UnrealTransforms.h:25, which carries no origin
offset:

    UE_X = ned_x * 100      UE_Y = ned_y * 100      UE_Z = -ned_z * 100

Z is NOT computed here. The caller traces the world downward at each (x, y),
because the road surface is what the level says it is, not what this file
guesses.
"""
from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
CITYMAP = REPO / "demo" / "out" / "citymap"
OUT = REPO / "demo" / "out" / "citylife" / "placement.json"

# demo/city_traffic.py: the corridor the demo actually flies.
TARGET_LANE_X = 38.0
ROUTE_Y0, ROUTE_Y1 = -8.0, 58.0
CIRCUIT_X = (34.0, 44.0)
CIRCUIT_Y0, CIRCUIT_Y1 = ROUTE_Y0 - 8.0, ROUTE_Y1 + 8.0

# demo/pedestrians.py: a band along the route, not a scatter over the map.
AVOID_RADIUS_M = 7.0
NEAR_RADIUS_M = 20.0

PAINTS = ["Black", "Blue", "Cyan", "Green", "Orange", "Red", "White", "Yellow"]

M_TO_CM = 100.0


def load_masks():
    st = np.load(CITYMAP / "street.npz")
    oc = np.load(CITYMAP / "occ_day_highband_15to55.npz")
    assert float(st["res"]) == float(oc["res"])
    assert float(st["origin_x"]) == float(oc["origin_x"])
    assert float(st["origin_y"]) == float(oc["origin_y"])
    return (st["street"], oc["occ"], float(st["res"]),
            float(st["origin_x"]), float(st["origin_y"]))


def _dist_to_segment(px, py, ax, ay, bx, by) -> float:
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    if L2 < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def _dist_to_route(x, y, route) -> float:
    return min(_dist_to_segment(x, y, a[0], a[1], b[0], b[1])
               for a, b in zip(route, route[1:]))


def pavement_spots(street, buildings, res, ox, oy, route):
    """Street cells that touch a building, banded along the route.

    Same rule and same indexing as demo/pedestrians.py: cell i is CENTRED at
    ox + i*res, with no half-cell added.
    """
    ni, nj = street.shape

    def touches_building(i, j):
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < ni and 0 <= b < nj and buildings[a, b]:
                return True
        return False

    spots = []
    for i in range(ni):
        for j in range(nj):
            if not street[i, j] or not touches_building(i, j):
                continue
            x = ox + i * res
            y = oy + j * res
            d = _dist_to_route(x, y, route)
            if d < AVOID_RADIUS_M or d > NEAR_RADIUS_M:
                continue
            spots.append((x, y))
    return spots


def circuit_loop():
    """The car circuit as a closed polyline, in NED metres."""
    x0, x1 = CIRCUIT_X
    return [(x0, CIRCUIT_Y0), (x0, CIRCUIT_Y1), (x1, CIRCUIT_Y1), (x1, CIRCUIT_Y0)]


def _loop_point(loop, frac):
    """A point at `frac` (0..1) of the way around a closed polyline, and the
    index of the vertex it is heading towards."""
    segs = list(zip(loop, loop[1:] + loop[:1]))
    lengths = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in segs]
    total = sum(lengths)
    want = (frac % 1.0) * total
    for k, ((a, b), L) in enumerate(zip(segs, lengths)):
        if want <= L or k == len(segs) - 1:
            t = want / L if L > 1e-9 else 0.0
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t), (k + 1) % len(loop)
        want -= L
    raise AssertionError("unreachable")


def to_ue(pt):
    return {"x": pt[0] * M_TO_CM, "y": pt[1] * M_TO_CM}


def build(n_peds: int, n_cars: int, seed: int):
    street, buildings, res, ox, oy = load_masks()
    route = [(TARGET_LANE_X, ROUTE_Y0), (TARGET_LANE_X, ROUTE_Y1)]

    rng = random.Random(seed)
    spots = pavement_spots(street, buildings, res, ox, oy, route)
    rng.shuffle(spots)
    chosen = spots[:n_peds]
    if len(chosen) < n_peds:
        raise SystemExit(
            f"only {len(chosen)} pavement cells in the band, wanted {n_peds}")

    peds = []
    for k, (x, y) in enumerate(chosen):
        peds.append({
            "name": f"Ped_{k:02d}",
            "mesh": "Manny" if k % 2 == 0 else "Quinn",
            "ue": to_ue((x, y)),
            "yaw": rng.choice([0.0, 90.0, 180.0, 270.0]),
            "ned": {"x": x, "y": y},
        })

    loop = circuit_loop()
    loop_ue = [to_ue(p) for p in loop]
    cars = []
    for k in range(n_cars):
        start, idx = _loop_point(loop, k / float(n_cars))
        cars.append({
            "name": f"Car_{k:02d}",
            "ue": to_ue(start),
            "idx": idx,
            "route_ue": loop_ue,
            "paint": PAINTS[k % len(PAINTS)],
            "ned": {"x": start[0], "y": start[1]},
        })

    return {
        "frame": "UE centimetres; UE_X = ned_x*100, UE_Y = ned_y*100 "
                 "(NedToUnrealLinear, no origin offset). Z comes from a "
                 "downward world trace at spawn time.",
        "source": {
            "street": "demo/out/citymap/street.npz",
            "buildings": "demo/out/citymap/occ_day_highband_15to55.npz",
            "res_m": res, "origin_m": [ox, oy],
        },
        "seed": seed,
        "pavement_cells_available": len(spots),
        "peds": peds,
        "cars": cars,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--peds", type=int, default=16)
    ap.add_argument("--cars", type=int, default=8)
    ap.add_argument("--seed", type=int, default=20260910)
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()

    data = build(args.peds, args.cars, args.seed)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=2), encoding="utf-8", newline="\n")
    print(f"[citylife] {len(data['peds'])} peds, {len(data['cars'])} cars "
          f"from {data['pavement_cells_available']} pavement cells -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
