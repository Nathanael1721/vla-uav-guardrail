"""
Build an ACCURATE occupancy map from Project AirSim's ground-truth geometry.

Unlike the camera survey (survey_city.py, which had holes + registration error),
this asks the sim itself for a voxel grid via world.create_voxel_grid — so the
map is 1:1 with the actual buildings. Collapses the voxels inside the flight
altitude band to a 2-D occupancy grid in our planner format.

Output: demo/out/citymap/occ.npz  (occ uint8 NxM, res, origin_x, origin_y)
        demo/out/citymap/occ_preview.png
Then copy to occ_<map>.npz for the world you surveyed.

Run (server on the target world):  python demo/build_voxel_map.py --out occ_day

ANY RECTANGLE, SEVERAL BANDS AT ONCE (2026-09-24)
-------------------------------------------------
Every map in demo/out/citymap/ is the 160 m cube around the origin this script
was hard-wired to, surveyed on Demo_day. CityLife's car loops run 30-120 m past
it, and off the map the Shield extrapolated the border into a wall that grew
deeper the further one flew - a follow flight stalled for up to 187 s at a
junction entrance that is open street. So:

    python demo/build_voxel_map.py --x-min -140 --x-max 220 --y-min -60 --y-max 140 \
        --out-dir demo/out/citymap_citylife \
        --bands ground_0to2:0:2,ground_2to4:2:4,occ_day_flightband_6to14:6:14,occ_day_highband_15to55:15:55 \
        --alias occ_day=occ_day_flightband_6to14 --free-check 35,-20

asks the simulator ONCE for a (nx, ny, nz) grid over that rectangle and cuts
every band out of it, saving the raw voxels beside them (`voxels.npz`) so a band
can be re-cut later without the simulator (`--from-voxels`).

The simulator's indexing, from WorldSimApi::createVoxelGrid:

    idx = i + nx * (k + nz * j)       i along UE X (North), j along UE Y (East),
                                      k along UE Z (up)
    cell centre = centre + (i - nx // 2) * res      (per axis)

so the flat array reshapes to (ny, nz, nx), and a map whose cells are centred
at `origin + i * res` has origin = centre - (n // 2) * res. With the defaults
(the +-80 m cube) this is exactly what the script always did.

WHOLE LEVEL (2026-11-04)
------------------------
The 360 x 200 m rectangle was still not CityLife: the level's buildings reach
x -916..1008 m and y -802..834 m, and off a map the Shield assumes "every
obstacle lives inside the map" (shield.py `_distance_at`). Two AGX flights
searched past its edge; one pressed against BP_jctBldSetK33_6 at (64.5,
145.5), 6.5 m north of it, 1919 contact reports with the P0 escape rate at
0.0. CityLife's maps now cover the whole level with 40-62 m of empty margin:

    python demo/build_voxel_map.py --x-min -960 --x-max 1060 --y-min -860 --y-max 880 \
        --tile-m 400 --out-dir demo/out/citymap_citylife \
        --bands ground_0to2:0:2,ground_2to4:2:4,occ_day_flightband_6to14:6:14,occ_day_highband_15to55:15:55 \
        --alias occ_day=occ_day_flightband_6to14 --free-check 35,-20

`--tile-m` asks for the 70 million voxels in sixteen requests of at most
200 x 200 x 80 and stitches them (160 s); one request for the old rectangle
took 8 s. `--keep-from` keeps an earlier build's cells inside its own
rectangle: the flight bands came back identical there (0 of 18000 cells),
but the 2-4 m band differed in 362 cells where cars and pedestrians stood
at scan time, and the street mask, pedestrian crossings and landing tests
rest on the earlier scan. Re-cut from the stitched voxels with

    python demo/build_voxel_map.py --from-voxels demo/out/citymap_citylife/voxels.npz \
        --out-dir demo/out/citymap_citylife --keep-from demo/out/citymap_citylife_20260924 \
        --bands (as above) --alias occ_day=occ_day_flightband_6to14 --free-check 35,-20

Each build writes `map_build.json` beside the maps (the folder is gitignored).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_guardrail.jsonc"

GRID_HALF = 80.0     # default: world spans [-80, 80] m in x(North) and y(East)
RES = 2.0            # meters per cell (matches the planner contract)


def grid_geometry(x_min: float, x_max: float, y_min: float, y_max: float,
                  z_size: float, res: float) -> dict:
    """Centre, sizes and cell counts for a rectangle, and the origin of the
    cell-centred map the simulator will return for it.

    The request is in whole metres (the RPC takes ints) and must be a whole
    number of cells, or the simulator's integer division shifts the grid by a
    fraction of a cell."""
    xs, ys = x_max - x_min, y_max - y_min
    for name, v in (("x", xs), ("y", ys), ("z", z_size)):
        if v <= 0 or abs(v - round(v)) > 1e-9 or abs(v / res - round(v / res)) > 1e-9:
            raise ValueError(f"{name} extent {v} m must be a positive whole number "
                             f"of metres and of {res} m cells")
    nx, ny, nz = int(round(xs / res)), int(round(ys / res)), int(round(z_size / res))
    # Centre chosen so that cell 0 is centred on x_min: centre - (n//2)*res.
    cx = x_min + (nx // 2) * res
    cy = y_min + (ny // 2) * res
    return {"cx": cx, "cy": cy, "x_size": int(round(xs)), "y_size": int(round(ys)),
            "z_size": int(round(z_size)), "nx": nx, "ny": ny, "nz": nz,
            "origin_x": x_min, "origin_y": y_min, "res": res}


def reshape_voxels(flat, nx: int, ny: int, nz: int) -> np.ndarray:
    """The simulator's flat array -> vox[j, k, i] (East, up, North)."""
    arr = np.asarray(flat, dtype=bool)
    if arr.size != nx * ny * nz:
        raise ValueError(f"got {arr.size} voxels, expected {nx}*{ny}*{nz}")
    return arr.reshape(ny, nz, nx)


def ground_index(vox: np.ndarray) -> int:
    """The GROUND z-slice: the densest-occupied horizontal slice."""
    return int(np.argmax(vox.sum(axis=(0, 2))))


def band_occ(vox: np.ndarray, gz: int, lo_m: float, hi_m: float,
             res: float) -> np.ndarray:
    """2-D occupancy [i=North][j=East] of everything within [lo, hi] m of the
    ground slice.

    Buildings sit ABOVE ground. We don't know the z sign (NED vs NEU), so take
    the band on BOTH sides of the ground slice and OR them — the empty (sky)
    side contributes nothing, so this is sign-agnostic and correct."""
    ny, nz, nx = vox.shape
    lo_c = int(round(lo_m / res))
    hi_c = int(round(hi_m / res))
    band_mask = np.zeros((ny, nx), dtype=bool)  # [y][x]
    for a, b in ((gz + lo_c, gz + hi_c), (gz - hi_c, gz - lo_c)):
        a2, b2 = max(0, min(a, nz)), max(0, min(b + 1, nz))
        if b2 > a2:
            band_mask |= vox[:, a2:b2, :].any(axis=1)
    # our planner grid is occ[i=North(x)][j=East(y)]; here axis0=y, axis2=x
    return band_mask.T.astype(np.uint8)


def cell_of(x: float, y: float, geo: dict):
    return (int(round((x - geo["origin_x"]) / geo["res"])),
            int(round((y - geo["origin_y"]) / geo["res"])))


def parse_bands(spec: str):
    out = []
    for item in spec.split(","):
        name, lo, hi = item.split(":")
        out.append((name.strip(), float(lo), float(hi)))
    return out


def keep_earlier(occ: np.ndarray, geo: dict, path: Path):
    """`occ` with an earlier map's cells inside that map's rectangle, and how
    many of them this scan had otherwise (None when there is no such map).
    The earlier map must lie on this grid: the same cell size, an origin a
    whole number of cells away, inside the new rectangle."""
    if not path.is_file():
        return occ, None
    with np.load(path) as z:
        old = z["occ"].astype(occ.dtype)
        res, ox, oy = float(z["res"]), float(z["origin_x"]), float(z["origin_y"])
    di, dj = (ox - geo["origin_x"]) / geo["res"], (oy - geo["origin_y"]) / geo["res"]
    if (res != geo["res"] or abs(di - round(di)) > 1e-9 or abs(dj - round(dj)) > 1e-9):
        raise ValueError(f"{path}: not on this grid (res {res}, origin {ox}, {oy})")
    di, dj = int(round(di)), int(round(dj))
    n, m = old.shape
    if di < 0 or dj < 0 or di + n > occ.shape[0] or dj + m > occ.shape[1]:
        raise ValueError(f"{path}: its rectangle is not inside the new one")
    out = occ.copy()
    changed = int((out[di:di + n, dj:dj + m] != old).sum())
    out[di:di + n, dj:dj + m] = old
    return out, {"cells": int(old.size), "cells_changed": changed,
                 "origin": [ox, oy], "shape": [n, m]}


def tile_rects(x_min: float, x_max: float, y_min: float, y_max: float,
               tile_m: float, res: float) -> list[tuple[float, float, float, float]]:
    """The rectangle cut into tiles of at most `tile_m` a side, each starting
    on a cell of the whole grid (x_min + k*res), so the tiles' cell centres
    are the whole grid's and stitch without a seam."""
    if tile_m <= 0 or abs(tile_m / res - round(tile_m / res)) > 1e-9:
        raise ValueError(f"tile {tile_m} m must be a positive whole number of {res} m cells")

    def edges(lo, hi):
        out, a = [], lo
        while a < hi - 1e-9:
            out.append((a, min(a + tile_m, hi)))
            a += tile_m
        return out
    return [(x0, x1, y0, y1) for x0, x1 in edges(x_min, x_max)
            for y0, y1 in edges(y_min, y_max)]


def fetch_voxels(geo: dict, tile_m: float | None = None, request=None):
    """vox[j, k, i] for the whole rectangle: one createVoxelGrid request, or
    with `tile_m` one per tile, stitched (WHOLE LEVEL, module docstring).
    `request(tile_geo) -> flat` stands in for the simulator in the tests."""
    x0, y0 = geo["origin_x"], geo["origin_y"]
    x1, y1 = x0 + geo["nx"] * geo["res"], y0 + geo["ny"] * geo["res"]
    rects = ([(x0, x1, y0, y1)] if tile_m is None
             else tile_rects(x0, x1, y0, y1, tile_m, geo["res"]))
    client = None
    if request is None:
        from projectairsim import Drone, ProjectAirSimClient, World
        from projectairsim.types import Pose, Quaternion, Vector3

        client = ProjectAirSimClient()
        client.connect()
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        Drone(client, world, "Drone1")           # ensure the scene is live

        def request(g):
            center = Pose({
                "translation": Vector3({"x": g["cx"], "y": g["cy"], "z": 0.0}),
                "rotation": Quaternion({"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}),
            })
            return world.create_voxel_grid(center, g["x_size"], g["y_size"],
                                           g["z_size"], int(g["res"]))
    try:
        vox = np.zeros((geo["ny"], geo["nz"], geo["nx"]), dtype=bool)
        for n, (a, b, c, d) in enumerate(rects, 1):
            g = grid_geometry(a, b, c, d, geo["z_size"], geo["res"])
            print(f"[voxel] requesting {g['nx']}x{g['ny']}x{g['nz']} grid @ "
                  f"{g['res']} m centred on NED ({g['cx']:g}, {g['cy']:g})"
                  + (f" (tile {n} of {len(rects)})" if len(rects) > 1 else "") + " ...")
            part = reshape_voxels(request(g), g["nx"], g["ny"], g["nz"])
            i0 = int(round((a - x0) / geo["res"]))
            j0 = int(round((c - y0) / geo["res"]))
            vox[j0:j0 + g["ny"], :, i0:i0 + g["nx"]] = part
    finally:
        if client is not None:
            client.disconnect()
    return vox


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="occ", help="output basename (occ_day, …)")
    ap.add_argument("--band-lo", type=float, default=15.0, help="flight band low (m AGL)")
    ap.add_argument("--band-hi", type=float, default=55.0, help="flight band high (m AGL)")
    ap.add_argument("--bands", default=None,
                    help="NAME:LO:HI[,NAME:LO:HI...] - cut several bands from "
                         "one voxel query. Replaces --out/--band-lo/--band-hi.")
    ap.add_argument("--alias", action="append", default=[],
                    help="NEW=EXISTING: also save band EXISTING as NEW.npz "
                         "(e.g. occ_day=occ_day_flightband_6to14, the name a "
                         "flight loads by default)")
    ap.add_argument("--x-min", type=float, default=-GRID_HALF)
    ap.add_argument("--x-max", type=float, default=GRID_HALF)
    ap.add_argument("--y-min", type=float, default=-GRID_HALF)
    ap.add_argument("--y-max", type=float, default=GRID_HALF)
    ap.add_argument("--z-size", type=float, default=2 * GRID_HALF,
                    help="vertical extent, metres, centred on NED z = 0")
    ap.add_argument("--tile-m", type=float, default=None,
                    help="query in square tiles of this side (metres, whole cells) "
                         "and stitch them: a whole level in one request is tens of "
                         "millions of voxels in one RPC (WHOLE LEVEL)")
    ap.add_argument("--out-dir", default=None,
                    help="where to write the maps. Default demo/out/citymap (the "
                         "Demo_day maps), or with --from-voxels the voxels file's "
                         "own folder - re-cutting CityLife voxels must not land on "
                         "Demo_day's maps by default")
    ap.add_argument("--from-voxels", default=None,
                    help="re-cut bands from a saved voxels.npz; no simulator")
    ap.add_argument("--keep-from", default=None,
                    help="a folder of earlier maps on the same grid: inside each "
                         "one's rectangle its cells are kept, so the area earlier "
                         "flights and tests rest on does not change with the "
                         "actors that stood there at scan time (WHOLE LEVEL)")
    ap.add_argument("--free-check", action="append", default=None,
                    help="X,Y (NED m) that must be FREE in every band above "
                         "2 m, e.g. the spawn. Default 35,-20 (Demo_day spawn).")
    args = ap.parse_args()

    if args.out_dir is not None:
        out = Path(args.out_dir)
    elif args.from_voxels:
        out = Path(args.from_voxels).resolve().parent
    else:
        out = ROOT / "demo" / "out" / "citymap"
    out.mkdir(parents=True, exist_ok=True)

    if args.from_voxels:
        d = np.load(args.from_voxels)
        vox = d["vox"].astype(bool)
        geo = {k: float(d[k]) for k in ("origin_x", "origin_y", "res", "cx", "cy")}
        ny, nz, nx = vox.shape
        geo.update({"nx": nx, "ny": ny, "nz": nz})
        print(f"[voxel] loaded {args.from_voxels}: {nx}x{ny}x{nz}")
    else:
        geo = grid_geometry(args.x_min, args.x_max, args.y_min, args.y_max,
                            args.z_size, RES)
        vox = fetch_voxels(geo, tile_m=args.tile_m)
        np.savez_compressed(out / "voxels.npz", vox=vox,
                            origin_x=geo["origin_x"], origin_y=geo["origin_y"],
                            res=geo["res"], cx=geo["cx"], cy=geo["cy"])
        print(f"[voxel] raw grid saved -> {out / 'voxels.npz'}")
    print(f"[voxel] {vox.size} voxels, occupied {int(vox.sum())} "
          f"({100 * vox.mean():.1f}%)")

    gz = ground_index(vox)
    occ_per_z = vox.sum(axis=(0, 2))
    print(f"[voxel] ground z-index = {gz} (occ {int(occ_per_z[gz])}); "
          f"per-z occ head/tail: {occ_per_z[:6]} ... {occ_per_z[-6:]}")

    bands = (parse_bands(args.bands) if args.bands
             else [(args.out, args.band_lo, args.band_hi)])
    checks = [tuple(float(v) for v in c.split(",")) for c in
              (args.free_check if args.free_check is not None else ["35,-20"])]
    saved = {}
    kept = {}
    for name, lo, hi in bands:
        occ = band_occ(vox, gz, lo, hi, geo["res"])
        if args.keep_from:
            occ, kept[name] = keep_earlier(occ, geo, Path(args.keep_from) / f"{name}.npz")
            if kept[name] is not None:
                print(f"[voxel] {name}: kept {args.keep_from}'s rectangle; "
                      f"{kept[name]['cells_changed']} of its {kept[name]['cells']} cells "
                      "differed in this scan")
        np.savez(out / f"{name}.npz", occ=occ, res=geo["res"],
                 origin_x=geo["origin_x"], origin_y=geo["origin_y"])
        saved[name] = occ
        print(f"[voxel] {name}: {lo:g}-{hi:g} m AGL, {int(occ.sum())} / {occ.size} "
              f"cells occupied ({100 * occ.mean():.1f}%) -> {out / (name + '.npz')}")
        if lo >= 2.0:
            for (cx_, cy_) in checks:
                i, j = cell_of(cx_, cy_, geo)
                inside = 0 <= i < occ.shape[0] and 0 <= j < occ.shape[1]
                state = (bool(occ[i, j]) if inside else "off the map")
                print(f"[voxel]   ({cx_:g}, {cy_:g}) -> cell ({i},{j}) occupied = "
                      f"{state} (expect False)")
    if not args.bands:
        # the old single-band run also wrote the generic name the GUI defaults to
        np.savez(out / "occ.npz", occ=saved[args.out], res=geo["res"],
                 origin_x=geo["origin_x"], origin_y=geo["origin_y"])
    for a in args.alias:
        new, old = a.split("=")
        np.savez(out / f"{new}.npz", occ=saved[old], res=geo["res"],
                 origin_x=geo["origin_x"], origin_y=geo["origin_y"])
        print(f"[voxel] {new}.npz = {old}")
    # How these maps were made, beside them: the folder is gitignored, so this
    # is the only record a reader of a flight's map path can find.
    import json
    import datetime
    n_x, n_y = saved[bands[0][0]].shape
    (out / "map_build.json").write_text(json.dumps({
        "built": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "argv": sys.argv[1:],
        "res": geo["res"], "origin_x": geo["origin_x"], "origin_y": geo["origin_y"],
        "cells": [n_x, n_y],
        "extent_m": {"x": [geo["origin_x"] - geo["res"] / 2, geo["origin_x"] + (n_x - 0.5) * geo["res"]],
                     "y": [geo["origin_y"] - geo["res"] / 2, geo["origin_y"] + (n_y - 0.5) * geo["res"]]},
        "ground_z_index": gz, "bands": {n: [lo, hi] for n, lo, hi in bands},
        "from_voxels": args.from_voxels,
        "kept_from": args.keep_from, "kept": kept or None,
    }, indent=1) + "\n", encoding="utf-8")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        occ = saved[bands[-1][0]] if len(bands) == 1 else saved.get(
            "occ_day_flightband_6to14", saved[bands[-1][0]])
        n, m = occ.shape
        ext = [geo["origin_x"] - geo["res"] / 2, geo["origin_x"] + (n - 0.5) * geo["res"],
               geo["origin_y"] - geo["res"] / 2, geo["origin_y"] + (m - 0.5) * geo["res"]]
        fig, ax = plt.subplots(figsize=(7, 7 * m / n))
        disp = np.where(occ.T == 1, 1.0, np.nan)
        ax.imshow(disp, origin="lower", extent=ext, cmap="autumn")
        for (cx_, cy_) in checks:
            ax.plot(cx_, cy_, "g*", markersize=14)
        ax.set_xlabel("North x (m)"); ax.set_ylabel("East y (m)")
        ax.set_title(f"Ground-truth voxel occupancy — {int(occ.sum())} cells")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / "occ_preview.png", dpi=130)
        print(f"[voxel] preview -> {out / 'occ_preview.png'}")
    except Exception as e:
        print(f"[voxel] preview skipped: {e}")


if __name__ == "__main__":
    main()
