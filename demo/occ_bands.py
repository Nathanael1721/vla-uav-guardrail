"""Which occupancy map does a policy's altitude band actually need?

    sel = select_for_band(CITYMAP_DIR, alt_min_m=4.0, alt_max_m=10.0)
    shield_map = sel["map"]          # merged, or None
    for line in sel["report"]: print(line)

WHY THIS EXISTS

The obstacle map is a **2-D** grid. `build_voxel_map.py` asks the simulator for a
voxel cube and collapses everything inside one altitude band down to a plane, so
a map is only meaningful for the band it was built over. Four exist:

    ground_0to2                0-2 m
    ground_2to4                2-4 m
    occ_day_flightband_6to14   6-14 m      (occ_day.npz is a copy of this one)
    occ_day_highband_15to55    15-55 m

Every flight loads `occ_day.npz` because that is the default, whatever the policy
permits. `policies/follow_pedestrian.yaml` permits descent to **4 m**, and the
checklist has recorded for a week that "ground_2to4.npz exists and nothing
chooses it" — as a tidiness item.

It is not a tidiness item. Measured across the two maps:

    cells occupied in 6-14 m                   2015
    cells occupied in 2-4 m                    1391
    occupied in 2-4 m and NOT in 6-14 m         300

Three hundred cells of low structure that an aircraft descending to 4 m can hit
and the loaded map cannot see. Neither map is a superset of the other: the 6-14 m
map holds the building at grid cell (64, 40) — the one in the documented 9 m
collision — which is open ground at 2-4 m.

AND THE HOLE NOBODY HAD LOOKED FOR

The four bands are not contiguous. There is **no map at all between 4 m and 6 m**,
which is inside the band `follow_pedestrian.yaml` permits. A selector that
quietly returned the nearest map would hide that; this one names it.

THE GROUND IS NOT AN OBSTACLE MAP

`ground_0to2.npz` has all 6400 of its cells occupied — it is the ground plane
seen from above, not a map of things to avoid. Merging it would mark the whole
world blocked, and a Shield that refuses every action looks exactly like a Shield
that is working very hard. Degenerate bands are rejected with a reason rather
than merged.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

# Altitude band each map file was built over, in metres AGL. The names encode it
# and the builder takes --band-lo/--band-hi, but nothing ever read it back.
BANDS: dict[str, tuple[float, float]] = {
    "ground_0to2": (0.0, 2.0),
    "ground_2to4": (2.0, 4.0),
    "occ_day_flightband_6to14": (6.0, 14.0),
    "occ_day_highband_15to55": (15.0, 55.0),
}

# A band map whose occupancy exceeds this is not describing obstacles. Measured:
# the real maps sit at 22-35 %, and ground_0to2 sits at 100 %.
DEGENERATE_FRACTION = 0.90


def available(map_dir: str | Path) -> list[dict]:
    """The band maps present on disk, lowest band first."""
    d = Path(map_dir)
    out = []
    for stem, (lo, hi) in sorted(BANDS.items(), key=lambda kv: kv[1]):
        p = d / f"{stem}.npz"
        if p.is_file():
            out.append({"stem": stem, "lo": lo, "hi": hi, "path": p})
    return out


def gaps(alt_min: float, alt_max: float, bands: list[dict]) -> list[tuple]:
    """Slices of [alt_min, alt_max] no band map covers.

    Returned as (lo, hi) pairs. An empty list means full coverage. Zero-width
    slices are dropped: bands that merely touch (2-4 then 4-6) leave no hole.
    """
    covering = sorted(((b["lo"], b["hi"]) for b in bands
                       if b["hi"] > alt_min and b["lo"] < alt_max))
    holes, cursor = [], float(alt_min)
    for lo, hi in covering:
        if lo > cursor:
            holes.append((cursor, lo))
        cursor = max(cursor, hi)
    if cursor < alt_max:
        holes.append((cursor, float(alt_max)))
    return [(a, b) for a, b in holes if b - a > 1e-9]


def _load(path: Path) -> dict:
    d = np.load(path, allow_pickle=False)
    return {"occ": np.asarray(d["occ"]).astype(np.uint8),
            "res": float(d["res"]),
            "ox": float(d["origin_x"]), "oy": float(d["origin_y"])}


def select_for_band(map_dir: str | Path, alt_min: float, alt_max: float,
                    strict: bool = False) -> dict:
    """Merge every band map covering [alt_min, alt_max] into one 2-D map.

    Returns `{"map", "used", "rejected", "gaps", "report"}`. `map` is the merged
    grid in `city_planner.load_occ` shape, or None when nothing usable covers the
    band. `report` is a list of lines meant to be printed by the caller — the
    point of this module is that the answer is never silent.

    `strict=True` raises when the band is not fully covered. Off by default,
    because the honest response to a partial map is to fly with it and say so,
    not to refuse to fly.

    Merging is a union: a cell blocked in ANY covering band is blocked. That is
    conservative in the safe direction and it is the only sound reading of a set
    of 2-D projections — a wall present at 3 m and absent at 8 m is still a wall
    the aircraft can hit somewhere inside a band it is allowed to occupy.
    """
    bands = available(map_dir)
    report: list[str] = []
    used, rejected = [], []

    covering = [b for b in bands if b["hi"] > alt_min and b["lo"] < alt_max]
    if not covering:
        report.append(f"[occ] no band map covers {alt_min:g}-{alt_max:g} m; "
                      f"available: " + ", ".join(f"{b['lo']:g}-{b['hi']:g}"
                                                 for b in bands))
        return {"map": None, "used": [], "rejected": [], "gaps": [(alt_min, alt_max)],
                "report": report}

    merged: Optional[np.ndarray] = None
    res = ox = oy = None
    for b in covering:
        m = _load(b["path"])
        frac = float(m["occ"].astype(bool).mean())
        if frac > DEGENERATE_FRACTION:
            rejected.append({**b, "why": f"{frac*100:.0f}% of cells occupied - "
                                         f"this is the ground plane, not obstacles"})
            report.append(f"[occ] rejected {b['stem']} ({b['lo']:g}-{b['hi']:g} m): "
                          f"{frac*100:.0f}% occupied, that is the ground")
            continue
        if merged is None:
            merged, res, ox, oy = m["occ"].astype(bool), m["res"], m["ox"], m["oy"]
        else:
            # Two maps on different grids cannot be unioned by array OR, and
            # doing it anyway would silently shift obstacles - the exact class
            # of error the registration work already cost this project once.
            if (m["occ"].shape != merged.shape or m["res"] != res
                    or m["ox"] != ox or m["oy"] != oy):
                raise ValueError(
                    f"{b['stem']} is on a different grid "
                    f"({m['occ'].shape} @ {m['res']} m, origin {m['ox']},{m['oy']}) "
                    f"than the maps already merged ({merged.shape} @ {res} m, "
                    f"origin {ox},{oy}). Refusing to union mis-registered maps.")
            before = int(merged.sum())
            merged = merged | m["occ"].astype(bool)
            report.append(f"[occ] + {b['stem']} ({b['lo']:g}-{b['hi']:g} m) "
                          f"adds {int(merged.sum()) - before} cells")
        used.append(b)

    if merged is None:
        report.append("[occ] every covering band was rejected; no obstacle map")
        return {"map": None, "used": [], "rejected": rejected,
                "gaps": gaps(alt_min, alt_max, bands), "report": report}

    if len(used) == 1:
        report.append(f"[occ] {used[0]['stem']} ({used[0]['lo']:g}-"
                      f"{used[0]['hi']:g} m), {int(merged.sum())} cells")
    else:
        report.append(f"[occ] merged {len(used)} bands -> {int(merged.sum())} cells "
                      f"for the {alt_min:g}-{alt_max:g} m policy band")

    holes = gaps(alt_min, alt_max, [b for b in bands if b not in rejected])
    for lo, hi in holes:
        report.append(f"[occ] *** {lo:g}-{hi:g} m of the permitted band has NO map. "
                      f"Obstacles there are invisible to ObstacleClearance.")
    if holes and strict:
        raise ValueError(
            f"altitude band {alt_min:g}-{alt_max:g} m is not covered by any "
            f"occupancy map at " + ", ".join(f"{a:g}-{b:g} m" for a, b in holes))

    N = int(merged.shape[0])
    return {
        "map": {"occ": merged.astype(np.uint8),
                "height": np.zeros(merged.shape, dtype=np.float32),
                "res": res, "ox": ox, "oy": oy, "N": N},
        "used": used, "rejected": rejected, "gaps": holes, "report": report,
    }
