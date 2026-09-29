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


def fetch_voxels(geo: dict):
    from projectairsim import Drone, ProjectAirSimClient, World
    from projectairsim.types import Pose, Quaternion, Vector3

    client = ProjectAirSimClient()
    client.connect()
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        Drone(client, world, "Drone1")           # ensure the scene is live
        center = Pose({
            "translation": Vector3({"x": geo["cx"], "y": geo["cy"], "z": 0.0}),
            "rotation": Quaternion({"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}),
        })
        print(f"[voxel] requesting {geo['nx']}x{geo['ny']}x{geo['nz']} grid @ "
              f"{geo['res']} m centred on NED ({geo['cx']:g}, {geo['cy']:g}) ...")
        flat = world.create_voxel_grid(center, geo["x_size"], geo["y_size"],
                                       geo["z_size"], int(geo["res"]))
    finally:
        client.disconnect()
    return reshape_voxels(flat, geo["nx"], geo["ny"], geo["nz"])


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
    ap.add_argument("--out-dir", default=None,
                    help="where to write the maps. Default demo/out/citymap (the "
                         "Demo_day maps), or with --from-voxels the voxels file's "
                         "own folder - re-cutting CityLife voxels must not land on "
                         "Demo_day's maps by default")
    ap.add_argument("--from-voxels", default=None,
                    help="re-cut bands from a saved voxels.npz; no simulator")
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
        vox = fetch_voxels(geo)
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
    for name, lo, hi in bands:
        occ = band_occ(vox, gz, lo, hi, geo["res"])
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
