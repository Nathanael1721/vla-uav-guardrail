"""Ground elevation: what an MSL height is above the ground (X-11, WP1-07).

WHY THIS EXISTS

The grant's DSL lets a geometry carry `altitude_ref: MSL` (mean sea level) as
an opt-in beside the default AGL (above ground level). The Shield measures
height above ground (models.State.up), so until 2026-11-03 an MSL rule was
declarable and refused for flight: the Shield had no ground-elevation source
to turn one height into the other, and judging a 120-300 m MSL zone as metres
above ground over 100 m ground lets an aircraft at 50 m AGL fly through it
with no violation (CHANGELOG 2026-10-19).

This module is that source. The Shield takes one (Shield(..., ground=...))
and enforces an MSL band as the AGL band it is over the ground:

    floor_agl = floor_msl - elevation_msl(x, y)

WHAT IS IMPLEMENTED, AND WHAT IS NOT

Two grounds, both opt-in (a Shield or compiler given no ground refuses
every MSL rule, as before 2026-11-03):

1. `FlatGround` (2026-11-03): one stated elevation over the whole mission,
   per rail. That is exact where the ground is flat - every simulated world
   here is - and the simulators state it themselves, as the altitude of
   their home point:

  * ArduPilot SITL rails: --home -35.363261,149.165230,584,0 (CMAC;
    sitl/start_sitl.sh, deploy/docker/sitl-entrypoint.sh): 584 m.
  * Project AirSim scenes: home-geo-point altitude 40.0 m (demo/pas_config/
    *.jsonc, demo/pas_ardupilot/scene_citylife_ardupilot.jsonc).

2. `GridGround` (2026-11-17, X-11): a ground whose elevation varies with
   position, a gridded digital elevation model - a small JSON grid (no
   GeoTIFF), bilinear between its nodes. The Shield converts an MSL rule at
   the QUERY POINT: each tick it reads the lowest and highest ground within
   `reach_m` of the aircraft (the nodes of every cell the reach box touches,
   which bound the bilinear surface there) and enforces the band that holds
   over ALL of that ground:

    keep-in band  (altitude_envelope, corridor, corridor_swap):
        floor_agl   = floor_msl   - lowest,   ceiling_agl = ceiling_msl - highest
    keep-out band (polygon_fence, dynamic_nfz, circle_fence):
        floor_agl   = floor_msl   - highest,  ceiling_agl = ceiling_msl - lowest

HEIGHT REFERENCE. A DEM conversion is right only when State.up is the height
above the DEM surface under the aircraft (terrain-relative). The rails' up
is NOT that: the ROS 2 rail sets up = MAVROS local z, height above the EKF
origin (home), which matches a FlatGround at the home elevation but not a
DEM - the error would be g(x, y) - g(home), either sign. So a Shield refuses
a GridGround unless its caller declares up_ref="terrain"
(Shield(ground=dem, up_ref="terrain")), and a rail that measures above home
converts first with `terrain_relative_up`. Since 2026-11-18 two rails do,
opt-in: the headless harness (a scenario's `ground`; its vehicle holds its
MSL height and its `up` is kept above the DEM) and the ROS 2 Shield node
(--ground, --ground-home-msl: `TerrainReference`). Neither has flown on
SITL, hil or PAS. THE FORECAST OVER A DEM (2026-11-18, wf20): until then
the Shield's forecast held the ground under the aircraft flat over its
lookahead, so an AGL band over a DEM let a level command over rising ground
down onto its floor before the violation was seen (the first SITL flight,
msl_dem_sitl_01: 2.0 s under agl-band's floor, at most 0.49 m; headless,
189 oracle ticks). Since then each forecast pose's up is the predicted MSL
height less the ground under THAT pose (Shield._over_ground) and the
altitude repair sizes its climb on those poses, so the violation is
predicted ahead and the climb starts in time (docs/data/msl_dem_rail.json,
finding_agl_over_dem). An MSL band keeps the flat forecast (the predicted
MSL height less the ground under the aircraft, against the band converted
over the reach): it is judged in MSL, as before.

A DEM as published is a WGS84 lat/lon grid: `Wgs84Grid` / `load_ground`
(below) read one and resample it into the policy's frame.

FRAME. A GridGround's x0, y0 are in a policy's local metre frame. It carries
the WGS84 origin (lat, lon) of that frame (`origin`, None for a metre-only
world) and a Shield, a hot-apply and a flight loader refuse it under a
policy whose frame origin differs (`origin_problem`).

so a pose anywhere within the reach is judged against a band at least as
strict as the true one, whichever way the aircraft holds its height (MSL or
AGL). The extremes are rounded outward to `quantum_m` (0.5 m), so the
Shield's per-band runtime is reused across ticks (Shield._local_runtime).
The price is headroom: about slope x reach_m on a slope. Outside the grid
the edge nodes stand in (the grid must cover the mission). On a flat ground
lowest = highest = the elevation, and both lines are the one above, exactly.
Any other non-flat object is still refused, not approximated. No rail in
this repository has flown an MSL rule.

The ground is NOT part of the policy (it is the world's, not the rule's), so
it never enters policy_hash; a flight records it beside its results
(`FlatGround.record`).
"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class FlatGround:
    """The ground at one stated elevation, metres above mean sea level,
    everywhere. `source` says where the number comes from - it is recorded
    with every flight that uses it."""

    flat = True
    __slots__ = ("elevation_msl_m", "source")

    def __init__(self, elevation_msl_m: float, source: str):
        e = float(elevation_msl_m)
        if not math.isfinite(e):
            raise ValueError(f"ground elevation {elevation_msl_m!r} is not finite")
        if not source:
            raise ValueError("a ground elevation must say where it comes from (source)")
        self.elevation_msl_m = e
        self.source = str(source)

    def elevation_at(self, x: float | None = None, y: float | None = None) -> float:
        """The ground's elevation (m MSL) under (x, y) metres of the policy
        frame: the stated one, wherever."""
        return self.elevation_msl_m

    def record(self) -> dict:
        return {"kind": "flat", "elevation_msl_m": self.elevation_msl_m,
                "source": self.source}

    def __repr__(self) -> str:
        return f"FlatGround({self.elevation_msl_m:g} m MSL, {self.source!r})"


# The rails' stated ground: the simulators' own home altitude.
SITL_CMAC = FlatGround(584.0, "ArduPilot SITL --home altitude (CMAC, sitl/start_sitl.sh)")
PAS_SCENE = FlatGround(40.0, "Project AirSim scenes' home-geo-point altitude "
                             "(demo/pas_config/*.jsonc)")
RAIL_GROUND: dict[str, FlatGround] = {"ardupilot-sitl": SITL_CMAC,
                                      "projectairsim": PAS_SCENE}


def scene_ground(scene: dict | str | Path) -> FlatGround:
    """The flat ground of a Project AirSim scene: its home-geo-point
    altitude (a scene dict, or the path of a .jsonc scene file)."""
    src = "a Project AirSim scene"
    if not isinstance(scene, dict):
        p = Path(scene)
        src = str(p)
        text = re.sub(r"^\s*//.*$", "", p.read_text(encoding="utf-8"), flags=re.M)
        scene = json.loads(text)
    home = scene.get("home-geo-point") or {}
    if "altitude" not in home:
        raise ValueError(f"{src}: no home-geo-point altitude, so no ground elevation")
    return FlatGround(float(home["altitude"]), f"home-geo-point altitude of {src}")


class GridGround:
    """A gridded digital elevation model (X-11, 2026-11-17): elevations in
    metres MSL at the nodes x0 + i*cell_m, y0 + j*cell_m of the policy's
    local frame (`elev[j][i]`, rows along y), bilinear between them, the edge
    nodes beyond the grid. `reach_m`: how far from the query point the
    Shield's forecast can carry the aircraft (lookahead x the fastest
    horizontal speed it can fly); `band_at` covers that much ground.
    `origin`: the WGS84 (lat, lon) of the local frame x0, y0 are in (None:
    a metre-only world); a Shield refuses it under a policy whose frame
    origin differs (module docstring, FRAME).

    HEIGHT REFERENCE: the conversion assumes State.up is the height above
    THIS surface under the aircraft. A rail whose up is height above home
    (the ROS 2 rail's MAVROS local z) must convert first
    (`terrain_relative_up`), and the Shield refuses a GridGround unless the
    caller declares up_ref="terrain".

    Pure and deterministic: the nodes are fixed at construction (a copy), so
    every answer is a function of the arguments; `band_at` is cached."""

    flat = False

    def __init__(self, x0: float, y0: float, cell_m: float, elev, source: str,
                 reach_m: float = 20.0, quantum_m: float = 0.5,
                 origin: tuple[float, float] | None = None):
        import numpy as np
        z = np.array(elev, dtype=float)
        if z.ndim != 2 or z.shape[0] < 2 or z.shape[1] < 2:
            raise ValueError(f"a grid ground needs at least 2 x 2 nodes, got shape {z.shape}")
        if not np.isfinite(z).all():
            raise ValueError("a grid ground's elevations must all be finite")
        for name, v in (("x0", x0), ("y0", y0), ("cell_m", cell_m),
                        ("reach_m", reach_m), ("quantum_m", quantum_m)):
            if not math.isfinite(float(v)):
                raise ValueError(f"grid ground {name} {v!r} is not finite")
        if cell_m <= 0 or reach_m < 0 or quantum_m <= 0:
            raise ValueError("grid ground needs cell_m > 0, reach_m >= 0, quantum_m > 0")
        if not source:
            raise ValueError("a ground elevation must say where it comes from (source)")
        z.setflags(write=False)
        self.z = z
        self.x0, self.y0, self.cell_m = float(x0), float(y0), float(cell_m)
        self.reach_m, self.quantum_m = float(reach_m), float(quantum_m)
        self.source = str(source)
        if origin is not None:
            lat, lon = float(origin[0]), float(origin[1])
            if not (math.isfinite(lat) and math.isfinite(lon)
                    and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                raise ValueError(f"grid ground origin {origin!r} is not a WGS84 (lat, lon)")
            origin = (lat, lon)
        self.origin = origin
        self._bounds = self._outward(float(z.min()), float(z.max()))
        self._cache: dict = {}
        # The nodes as Python floats for elevation_at: the same doubles and
        # the same arithmetic as indexing `z`, without a NumPy scalar per
        # read (the Shield reads one per forecast pose).
        self._zl = z.tolist()

    # -- reading the grid
    def _fidx(self, x: float, y: float) -> tuple[float, float]:
        nj, ni = self.z.shape
        fi = min(max((float(x) - self.x0) / self.cell_m, 0.0), ni - 1.0)
        fj = min(max((float(y) - self.y0) / self.cell_m, 0.0), nj - 1.0)
        return fi, fj

    def elevation_at(self, x: float, y: float) -> float:
        """Bilinear elevation (m MSL) at (x, y) of the policy frame."""
        fi, fj = self._fidx(x, y)
        nj, ni = self.z.shape
        i, j = min(int(fi), ni - 2), min(int(fj), nj - 2)
        u, v = fi - i, fj - j
        r0, r1 = self._zl[j], self._zl[j + 1]
        return float((1 - u) * (1 - v) * r0[i] + u * (1 - v) * r0[i + 1]
                     + (1 - u) * v * r1[i] + u * v * r1[i + 1])

    def _outward(self, lo: float, hi: float) -> tuple[float, float]:
        q = self.quantum_m
        return (math.floor(lo / q + 1e-9) * q, math.ceil(hi / q - 1e-9) * q)

    def bounds(self) -> tuple[float, float]:
        """(lowest, highest) ground of the whole grid, rounded outward."""
        return self._bounds

    def band_box(self, xmin: float, ymin: float, xmax: float, ymax: float
                 ) -> tuple[float, float]:
        """(lowest, highest) ground over the box [xmin, xmax] x [ymin, ymax]
        (no reach added): the nodes of every cell it touches, rounded
        outward. The CSP's mission region (compiler.compile_csp)."""
        nj, ni = self.z.shape
        c = self.cell_m
        i0 = int(math.floor(min(max((xmin - self.x0) / c, 0.0), ni - 1.0)))
        i1 = int(math.ceil(min(max((xmax - self.x0) / c, 0.0), ni - 1.0)))
        j0 = int(math.floor(min(max((ymin - self.y0) / c, 0.0), nj - 1.0)))
        j1 = int(math.ceil(min(max((ymax - self.y0) / c, 0.0), nj - 1.0)))
        key = (i0, i1, j0, j1)
        hit = self._cache.get(key)
        if hit is None:
            sub = self.z[j0:j1 + 1, i0:i1 + 1]
            hit = self._outward(float(sub.min()), float(sub.max()))
            if len(self._cache) >= 4096:
                self._cache.clear()
            self._cache[key] = hit
        return hit

    def band_at(self, x: float, y: float, reach_m: float | None = None
                ) -> tuple[float, float]:
        """(lowest, highest) ground within `reach_m` (default the ground's)
        of (x, y): the nodes of every cell the box [x +- r] x [y +- r]
        touches - a bilinear surface stays within its cell's nodes - rounded
        outward to `quantum_m`."""
        r = self.reach_m if reach_m is None else float(reach_m)
        return self.band_box(x - r, y - r, x + r, y + r)

    # -- provenance and file form
    def record(self) -> dict:
        import hashlib
        return {"kind": "grid", "x0": self.x0, "y0": self.y0, "cell_m": self.cell_m,
                "shape": list(self.z.shape), "min_msl_m": float(self.z.min()),
                "max_msl_m": float(self.z.max()), "reach_m": self.reach_m,
                "quantum_m": self.quantum_m,
                "origin": None if self.origin is None else list(self.origin),
                "sha256": hashlib.sha256(self.z.astype("<f8").tobytes()).hexdigest(),
                "source": self.source}

    def to_dict(self) -> dict:
        return {"kind": "grid", "x0": self.x0, "y0": self.y0, "cell_m": self.cell_m,
                "reach_m": self.reach_m, "quantum_m": self.quantum_m,
                "source": self.source,
                "origin": None if self.origin is None else list(self.origin),
                "elev_msl_m": self.z.tolist()}

    @classmethod
    def from_dict(cls, d: dict) -> "GridGround":
        if d.get("kind") != "grid":
            raise ValueError(f"not a grid ground (kind {d.get('kind')!r})")
        return cls(d["x0"], d["y0"], d["cell_m"], d["elev_msl_m"], d["source"],
                   reach_m=d.get("reach_m", 20.0), quantum_m=d.get("quantum_m", 0.5),
                   origin=d.get("origin"))

    @classmethod
    def load(cls, path: str | Path) -> "GridGround":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def __repr__(self) -> str:
        lo, hi = float(self.z.min()), float(self.z.max())
        return (f"GridGround({self.z.shape[1]}x{self.z.shape[0]} @ {self.cell_m:g} m, "
                f"{lo:g}-{hi:g} m MSL, {self.source!r})")


def is_dem(ground) -> bool:
    """A ground that varies with position and says how (GridGround's
    protocol: `bounds()` and `band_at(x, y)`)."""
    return (ground is not None and not getattr(ground, "flat", False)
            and callable(getattr(ground, "band_at", None))
            and callable(getattr(ground, "bounds", None)))


def ground_band(ground) -> tuple[float, float]:
    """(lowest, highest) elevation of `ground` over everything it covers:
    a flat ground's one elevation twice, a DEM's extremes."""
    if is_dem(ground):
        return ground.bounds()
    e = float(ground.elevation_at())
    return (e, e)


def terrain_relative_up(z_above_home: float, home_elev_msl_m: float, dem,
                        x: float, y: float) -> float:
    """The height above the DEM surface under (x, y) - what a Shield given a
    GridGround (up_ref="terrain") must be told - from a rail's height above
    its home / EKF origin (the ROS 2 rail's MAVROS local z) and the home's
    elevation MSL: z + home - g(x, y)."""
    return float(z_above_home) + float(home_elev_msl_m) - float(dem.elevation_at(x, y))


def origin_problem(ground, frame_origin) -> str | None:
    """Why `ground` cannot be read in a policy frame whose WGS84 origin is
    `frame_origin` ((lat, lon) or None for a metre-only policy), or None.
    Only a DEM is placed in a frame; its origin must equal the policy's
    (both None: a metre-only world and policy)."""
    if not is_dem(ground):
        return None
    go = getattr(ground, "origin", None)
    if go is None and frame_origin is None:
        return None
    if go is not None and frame_origin is not None and             abs(go[0] - frame_origin[0]) < 1e-9 and abs(go[1] - frame_origin[1]) < 1e-9:
        return None
    return (f"the ground {ground!r} is in the frame of origin {go}, the policy's "
            f"frame origin is {frame_origin}: a DEM is read only in its own frame")


# ---------------------------------------------------------------------------
# A DEM as published: a WGS84 latitude / longitude grid (X-11, 2026-11-18)
# ---------------------------------------------------------------------------
#
# Elevation models are published on a geographic grid (SRTM, Copernicus GLO-30
# and the like: one elevation per node of a regular lat/lon lattice), not in a
# policy's local metres. `Wgs84Grid` reads such a grid from a small JSON or NPZ
# file (no GeoTIFF: no reader for it is in the flight environments) and
# `to_ground` places it in a policy's frame as the GridGround the Shield reads:
#
#   {"kind": "wgs84_grid", "source": "...",
#    "lat0": -35.3678, "lon0": 149.1597,      # the SOUTH-WEST node
#    "dlat_deg": 0.0001, "dlon_deg": 0.0001,  # node spacing, north / east
#    "elev_msl_m": [[...], ...]}              # rows south -> north, columns
#                                             # west -> east, metres MSL
#
# (an .npz holds the same keys as arrays; `source` as a 0-d string array).
#
# RESAMPLING. The policy frame is this repository's equirectangular projection
# about the policy's origin (guardrail/projection.py): north = R * dlat,
# east = R * cos(lat_origin) * dlon, so the grid's nodes fall on an
# axis-aligned lattice of spacings s_n (north) and s_e (east), which differ.
# A GridGround has one square cell, so the grid is resampled: a node of the
# square lattice (cell_m, default min(s_n, s_e)) takes the source's bilinear
# elevation at its lat/lon. The resampled surface differs from the source's
# by at most L * cell_m * sqrt(2), L the source surface's steepest slope
# (`resample_bound_m`, recorded; a resampled bilinear value is a convex
# combination of source values at nodes within cell_m * sqrt(2)).
#
# WHAT THAT ERROR CAN AND CANNOT MOVE. An MSL rule is judged at the rail's own
# MSL height: a rail's up is z_above_home + home_msl - g(x, y) (g the
# resampled surface; `terrain_relative_up`), and the Shield converts the MSL
# band over the lowest / highest g within reach, which bound g(x, y) itself -
# so g cancels and an MSL rule is enforced at least as strictly as declared
# whatever g's error (a ceiling: z + home_msl <= C - (hi - g(x, y)) <= C).
# What g's error moves is the terrain-relative height itself, i.e. how AGL
# rules over that DEM read (by up to `resample_bound_m`), and how much
# headroom the conversion takes.

WGS84_GRID_KIND = "wgs84_grid"
# A rail's home elevation and the DEM's elevation at home may disagree by
# this much before the rail refuses the DEM (TerrainReference): more, and an
# aircraft on its pad would read as above or below the ground the DEM puts
# under it.
GROUND_HOME_TOL_M = 2.0


def _file_sha256(path: Path) -> str:
    import hashlib
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Wgs84Grid:
    """A DEM on a WGS84 lat/lon grid (section comment above): `elev[r][c]`
    metres MSL at (lat0 + r * dlat_deg, lon0 + c * dlon_deg), bilinear
    between nodes, the edge nodes beyond. Read-only; `path` / `sha256` name
    the file it came from (None for one built in memory)."""

    def __init__(self, lat0: float, lon0: float, dlat_deg: float, dlon_deg: float,
                 elev, source: str, path: str | None = None, sha256: str | None = None):
        import numpy as np
        z = np.array(elev, dtype=float)
        if z.ndim != 2 or z.shape[0] < 2 or z.shape[1] < 2:
            raise ValueError(f"a WGS84 grid needs at least 2 x 2 nodes, got shape {z.shape}")
        if not np.isfinite(z).all():
            raise ValueError("a WGS84 grid's elevations must all be finite")
        vals = {"lat0": lat0, "lon0": lon0, "dlat_deg": dlat_deg, "dlon_deg": dlon_deg}
        for k, v in vals.items():
            if not isinstance(v, (int, float)) or not math.isfinite(float(v)):
                raise ValueError(f"WGS84 grid {k} {v!r} is not a finite number")
        if not (dlat_deg > 0 and dlon_deg > 0):
            raise ValueError("a WGS84 grid needs dlat_deg > 0 and dlon_deg > 0")
        lat1 = lat0 + (z.shape[0] - 1) * dlat_deg
        lon1 = lon0 + (z.shape[1] - 1) * dlon_deg
        if not (-90.0 <= lat0 and lat1 <= 90.0 and -180.0 <= lon0 and lon1 <= 180.0):
            raise ValueError(f"WGS84 grid spans lat {lat0}..{lat1}, lon {lon0}..{lon1}: "
                             f"not within -90..90 / -180..180")
        if not source:
            raise ValueError("a ground elevation must say where it comes from (source)")
        z.setflags(write=False)
        self.z = z
        self.lat0, self.lon0 = float(lat0), float(lon0)
        self.dlat, self.dlon = float(dlat_deg), float(dlon_deg)
        self.source, self.path, self.sha256 = str(source), path, sha256

    @property
    def shape(self) -> tuple[int, int]:
        return tuple(self.z.shape)

    def elevation_at_latlon(self, lat, lon):
        """Bilinear elevation (m MSL) at (lat, lon); scalars or numpy arrays."""
        import numpy as np
        nr, nc = self.z.shape
        fr = np.clip((np.asarray(lat, dtype=float) - self.lat0) / self.dlat, 0.0, nr - 1.0)
        fc = np.clip((np.asarray(lon, dtype=float) - self.lon0) / self.dlon, 0.0, nc - 1.0)
        r = np.minimum(np.floor(fr).astype(int), nr - 2)
        c = np.minimum(np.floor(fc).astype(int), nc - 2)
        v, u = fr - r, fc - c
        z = self.z
        out = ((1 - u) * (1 - v) * z[r, c] + u * (1 - v) * z[r, c + 1]
               + (1 - u) * v * z[r + 1, c] + u * v * z[r + 1, c + 1])
        return float(out) if np.ndim(out) == 0 else out

    def spacing_m(self, frame_origin: tuple[float, float]) -> tuple[float, float]:
        """(s_n, s_e): the node spacings in metres north / east in the
        equirectangular frame of `frame_origin` (guardrail/projection.py)."""
        from .projection import projection_for
        p = projection_for(float(frame_origin[0]), float(frame_origin[1]))
        n1, _ = p.to_local(p.origin_lat + self.dlat, p.origin_lon)
        _, e1 = p.to_local(p.origin_lat, p.origin_lon + self.dlon)
        return abs(n1), abs(e1)

    def max_slope(self, frame_origin: tuple[float, float]) -> float:
        """L: an upper bound on the source surface's slope (m/m) in that
        frame - the steepest node-to-node rise along north, along east,
        combined: a bilinear cell's gradient is bounded by its edges'."""
        import numpy as np
        sn, se = self.spacing_m(frame_origin)
        gn = float(np.abs(np.diff(self.z, axis=0)).max()) / sn
        ge = float(np.abs(np.diff(self.z, axis=1)).max()) / se
        return math.hypot(gn, ge)

    def to_ground(self, frame_origin, cell_m: float | None = None,
                  reach_m: float = 20.0, quantum_m: float = 0.5) -> "GeoGridGround":
        """This grid as the GridGround of a policy whose frame origin is
        `frame_origin` (lat, lon): resampled onto a square lattice of
        `cell_m` (default the smaller node spacing) covering the grid's
        extent (section comment above, RESAMPLING)."""
        import numpy as np
        from .projection import _EARTH_R, projection_for
        if frame_origin is None:
            raise ValueError("a WGS84 DEM is placed in a policy's frame, and this policy "
                             "has none (written in metres only): give a metre-framed grid "
                             "(GridGround) or a policy with a WGS84 origin")
        o = (float(frame_origin[0]), float(frame_origin[1]))
        p = projection_for(o[0], o[1])
        sn, se = self.spacing_m(o)
        c = float(cell_m) if cell_m is not None else min(sn, se)
        if not (math.isfinite(c) and c > 0):
            raise ValueError(f"cell_m {cell_m!r} must be a positive number of metres")
        nr, nc = self.z.shape
        x0, y0 = p.to_local(self.lat0, self.lon0)
        x1, y1 = p.to_local(self.lat0 + (nr - 1) * self.dlat,
                            self.lon0 + (nc - 1) * self.dlon)
        ni = int(math.ceil((x1 - x0) / c - 1e-9)) + 1
        nj = int(math.ceil((y1 - y0) / c - 1e-9)) + 1
        if ni * nj > 4_000_000:
            raise ValueError(f"resampling at {c:g} m gives {ni} x {nj} nodes: "
                             f"give a larger cell_m")
        xs = x0 + c * np.arange(ni)
        ys = y0 + c * np.arange(nj)
        X, Y = np.meshgrid(xs, ys)                        # rows along y (east)
        lat = p.origin_lat + np.degrees(X / _EARTH_R)
        lon = p.origin_lon + np.degrees(Y / (_EARTH_R * math.cos(math.radians(p.origin_lat))))
        z = self.elevation_at_latlon(lat, lon)
        L = self.max_slope(o)
        bound = L * c * math.sqrt(2.0)
        # The bound, checked where it is loosest: at the resampled cells'
        # centres and edge midpoints (recorded beside it, not a proof).
        mx = (xs[:-1] + xs[1:]) / 2.0
        my = (ys[:-1] + ys[1:]) / 2.0
        sampled = 0.0
        for qx, qy in ((mx, my), (mx, ys[:-1]), (xs[:-1], my)):
            QX, QY = np.meshgrid(qx, qy)
            qlat = p.origin_lat + np.degrees(QX / _EARTH_R)
            qlon = p.origin_lon + np.degrees(
                QY / (_EARTH_R * math.cos(math.radians(p.origin_lat))))
            src = self.elevation_at_latlon(qlat, qlon)
            fi, fj = (QX - x0) / c, (QY - y0) / c
            i, j = np.floor(fi).astype(int), np.floor(fj).astype(int)
            u, v = fi - i, fj - j
            res = ((1 - u) * (1 - v) * z[j, i] + u * (1 - v) * z[j, i + 1]
                   + (1 - u) * v * z[j + 1, i] + u * v * z[j + 1, i + 1])
            sampled = max(sampled, float(np.abs(src - res).max()))
        geo = {"kind": WGS84_GRID_KIND, "path": self.path, "sha256": self.sha256,
               "lat0": self.lat0, "lon0": self.lon0, "dlat_deg": self.dlat,
               "dlon_deg": self.dlon, "shape": list(self.z.shape),
               "spacing_m": [round(sn, 6), round(se, 6)],
               "resample_cell_m": round(c, 6),
               "max_slope": round(L, 6),
               "resample_bound_m": round(bound, 6),
               "resample_max_err_sampled_m": round(sampled, 6)}
        return GeoGridGround(x0, y0, c, z, self.source, reach_m=reach_m,
                             quantum_m=quantum_m, origin=o, geo=geo)

    def to_dict(self) -> dict:
        return {"kind": WGS84_GRID_KIND, "source": self.source, "lat0": self.lat0,
                "lon0": self.lon0, "dlat_deg": self.dlat, "dlon_deg": self.dlon,
                "elev_msl_m": self.z.tolist()}

    def __repr__(self) -> str:
        return (f"Wgs84Grid({self.z.shape[0]}x{self.z.shape[1]} from "
                f"({self.lat0:g}, {self.lon0:g}) every ({self.dlat:g}, {self.dlon:g}) deg, "
                f"{float(self.z.min()):g}-{float(self.z.max()):g} m MSL, {self.source!r})")


def load_wgs84_grid(path: str | Path) -> Wgs84Grid:
    """Read a WGS84 grid from a JSON or NPZ file (section comment above)."""
    p = Path(path)
    if p.suffix.lower() == ".npz":
        import numpy as np
        with np.load(p, allow_pickle=False) as f:
            d = {k: f[k] for k in f.files}
        missing = {"lat0", "lon0", "dlat_deg", "dlon_deg", "elev_msl_m", "source"} - set(d)
        if missing:
            raise ValueError(f"{p}: not a WGS84 grid (missing {sorted(missing)})")
        kind = str(d["kind"]) if "kind" in d else WGS84_GRID_KIND
        if kind != WGS84_GRID_KIND:
            raise ValueError(f"{p}: kind {kind!r}, not {WGS84_GRID_KIND!r}")
        return Wgs84Grid(float(d["lat0"]), float(d["lon0"]), float(d["dlat_deg"]),
                         float(d["dlon_deg"]), d["elev_msl_m"], str(d["source"]),
                         path=str(path), sha256=_file_sha256(p))
    d = json.loads(p.read_text(encoding="utf-8"))
    if not isinstance(d, dict) or d.get("kind") != WGS84_GRID_KIND:
        raise ValueError(f"{p}: not a WGS84 grid (kind "
                         f"{d.get('kind') if isinstance(d, dict) else None!r})")
    unknown = set(d) - {"kind", "source", "description", "lat0", "lon0", "dlat_deg",
                        "dlon_deg", "elev_msl_m"}
    if unknown:
        raise ValueError(f"{p}: unknown key(s) {sorted(unknown)}")
    return Wgs84Grid(d["lat0"], d["lon0"], d["dlat_deg"], d["dlon_deg"], d["elev_msl_m"],
                     d.get("source", ""), path=str(path), sha256=_file_sha256(p))


class GeoGridGround(GridGround):
    """A GridGround resampled from a WGS84 grid (Wgs84Grid.to_ground): the
    same surface to the Shield, with the source file and the resampling's
    error bound in its record."""

    def __init__(self, *a, geo: dict, **kw):
        super().__init__(*a, **kw)
        self.geo = dict(geo)

    def record(self) -> dict:
        rec = super().record()
        rec["wgs84_dem"] = dict(self.geo)
        return rec


def load_ground(path: str | Path, frame_origin=None, *, root: str | Path | None = None,
                reach_m: float | None = None):
    """A ground from a file, in the frame of a policy whose WGS84 origin is
    `frame_origin` ((lat, lon), or None for a metre-only policy):

      * a WGS84 grid (.npz, or JSON `kind: wgs84_grid`) is resampled into that
        frame (Wgs84Grid.to_ground), which it needs;
      * a metre grid (JSON `kind: grid`, GridGround.to_dict) is read as it is
        and must be in that frame (origin_problem);
      * a flat ground (JSON `{"kind": "flat", "elevation_msl_m": ...,
        "source": ...}`) is a FlatGround.

    `root`: what a relative `path` is relative to (the repository root, as a
    scenario or a rail flag names it). `reach_m`: override the grid's reach
    (a WGS84 grid's default is GridGround's 20 m)."""
    p = Path(path)
    full = p if p.is_absolute() or root is None else Path(root) / p
    if full.suffix.lower() == ".npz":
        kind = WGS84_GRID_KIND
        d = None
    else:
        d = json.loads(full.read_text(encoding="utf-8"))
        kind = d.get("kind") if isinstance(d, dict) else None
    if kind == WGS84_GRID_KIND:
        grid = load_wgs84_grid(full)
        grid.path = str(path)
        g = grid.to_ground(frame_origin, **({} if reach_m is None else {"reach_m": reach_m}))
        g.geo["path"] = str(path)
        return g
    if kind == "grid":
        if reach_m is not None:
            d = dict(d, reach_m=reach_m)
        g = GridGround.from_dict(d)
        why = origin_problem(g, None if frame_origin is None else
                             (float(frame_origin[0]), float(frame_origin[1])))
        if why:
            raise ValueError(f"{path}: {why}")
        return g
    if kind == "flat":
        return FlatGround(d["elevation_msl_m"], d.get("source") or str(path))
    raise ValueError(f"{path}: not a ground file (kind {kind!r}; one of "
                     f"{WGS84_GRID_KIND!r}, 'grid', 'flat')")


def policy_file_frame_origin(bundle=None, yaml_path=None):
    """The WGS84 frame origin of the policy a rail is about to fly (its
    bundle or YAML), read WITHOUT the flight gate - which refuses an MSL rule
    until it is given the ground this origin places. The flight loader
    (bundle.load_for_flight) then reads the same file with that ground and
    applies every check."""
    if bundle:
        from .bundle import check_bundle
        chk = check_bundle(bundle)
        pol = chk.flight_policy if chk.flight_policy is not None else chk.policy
        return pol.frame_origin
    if yaml_path:
        from .models import load_policy
        return load_policy(yaml_path, runtime=False).frame_origin
    raise ValueError("no policy given: a bundle or a YAML file")


def ground_for_flight(path, bundle=None, yaml_path=None, *, root=None, sha256=None):
    """`load_ground(path)` in the frame of the policy a rail flies (its
    bundle or YAML, `policy_file_frame_origin`). `sha256` (the readers'
    --ground-sha256, 2026-11-18): the file must hash to it, checked before
    it is read (`ground_file`); None reads it unpinned, as every rail did
    before that date."""
    if sha256 is not None:
        ground_file(path, root=root, sha256=sha256)
    return load_ground(path, policy_file_frame_origin(bundle, yaml_path), root=root)


# ---------------------------------------------------------------------------
# Naming a ground: its file and the file's hash (2026-11-18, X-11)
# ---------------------------------------------------------------------------
#
# A ground is the world's, not the policy's, so it never enters policy_hash
# and nothing in a signed bundle pins it. Every reader that flies or scores a
# ground is therefore told it by NAME: the repo-relative path of its file and
# the file's sha256 (`--ground FILE --ground-sha256 HEX` on the rails' nodes,
# the GCS step, bundle_origin and bag_custom_data; the stress harness sends
# both, computed from its own checkout). A reader never guesses one - not
# from a bundle's frame origin, not from a rail's home altitude, not from a
# default - and a file that does not hash to the name it was given is refused
# before it is read. The flight's records carry the same pair
# (`ground_file`), which is what a replay bundle packs and checks
# (guardrail/replay.py, "THE GROUND A RUN FLEW OVER").

GROUND_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def _ground_path(path, root=None) -> Path:
    p = Path(path)
    return p if p.is_absolute() or root is None else Path(root) / p


def ground_file_sha256(path, root=None) -> str:
    """The sha256 (hex) of a ground file's bytes, as the file is on disk."""
    return _file_sha256(_ground_path(path, root))


def ground_file(path, *, root=None, sha256: str | None = None) -> dict:
    """`{"path", "sha256"}` of a declared ground file: `path` as it was named
    (repo-relative on every rail) and the hash of its bytes. With `sha256`
    the file must hash to it (lower-case hex, 64 digits), else ValueError:
    the reader was named another ground than the one on its disk."""
    if sha256 is not None and not GROUND_SHA256_RE.match(str(sha256)):
        raise ValueError(f"ground sha256 {sha256!r} is not 64 lower-case hex digits")
    got = ground_file_sha256(path, root)
    if sha256 is not None and got != sha256:
        raise ValueError(f"ground {path}: the file hashes to {got}, the ground was "
                         f"named with {sha256}: another file than the one named")
    return {"path": str(path), "sha256": got}


def ground_from_args(args, bundle=None, yaml_path=None, *, root=None):
    """(ground, file record) a reader's parsed arguments name: `args.ground`
    (a path) and `args.ground_sha256` (its hash, optional), the ground placed
    in the frame of the policy the reader flies (`ground_for_flight`), or
    (None, None) without --ground. --ground-sha256 without --ground is
    refused: a hash names no file."""
    path = getattr(args, "ground", None)
    sha = getattr(args, "ground_sha256", None)
    if not path:
        if sha:
            raise ValueError("--ground-sha256 is the hash of a --ground file, and no "
                             "--ground was given")
        return None, None
    rec = ground_file(path, root=root, sha256=sha)
    rec["pinned"] = sha is not None
    return load_ground(path, policy_file_frame_origin(bundle, yaml_path), root=root), rec


class TerrainReference:
    """What a rail that measures height above its HOME tells a Shield given a
    DEM (Shield(ground=dem, up_ref="terrain")): the height above the DEM
    surface under the aircraft, `terrain_relative_up(z, home_msl_m, dem, x,
    y)`.

    `home_msl_m`: the elevation (m MSL) of the rail's home / EKF origin - the
    autopilot's own home altitude (the SITL rails': 584 m, CMAC). The MSL
    height a rule is judged at is z + home_msl_m, so this is the number that
    must be right; the DEM's elevation at home (`home_xy`, policy frame) must
    agree with it within `tol_m` (GROUND_HOME_TOL_M) or the rail is refused
    before take-off: an aircraft on its pad would otherwise read as above or
    below the ground the DEM puts under it. None: the DEM's own elevation at
    home (a rail that takes off from the DEM's surface, as the headless
    harness does)."""

    def __init__(self, ground, home_msl_m: float | None = None,
                 home_xy: tuple[float, float] = (0.0, 0.0), source: str = "",
                 tol_m: float = GROUND_HOME_TOL_M):
        if not is_dem(ground):
            raise ValueError(f"{ground!r} is not a DEM: a flat ground needs no "
                             f"terrain-relative height (the rails' height above home "
                             f"is already its height above a flat ground at home)")
        self.ground = ground
        self.home_xy = (float(home_xy[0]), float(home_xy[1]))
        g_home = float(ground.elevation_at(*self.home_xy))
        if home_msl_m is None:
            self.home_msl_m, self.home_source = g_home, "the DEM's elevation at home"
        else:
            h = float(home_msl_m)
            if not math.isfinite(h):
                raise ValueError(f"home elevation {home_msl_m!r} is not finite")
            if abs(h - g_home) > tol_m:
                raise ValueError(
                    f"the home elevation {h:g} m MSL ({source or 'given'}) and the DEM's "
                    f"elevation at home ({self.home_xy[0]:g}, {self.home_xy[1]:g}), "
                    f"{g_home:.2f} m MSL, differ by {abs(h - g_home):.2f} m (over "
                    f"{tol_m:g} m): the aircraft on its pad would read as "
                    f"{'above' if h > g_home else 'below'} the DEM's ground")
            self.home_msl_m, self.home_source = h, source or "given"
        self.dem_at_home_msl_m = g_home

    def up(self, z_above_home: float, x: float, y: float) -> float:
        """Height above the DEM at (x, y) of an aircraft z metres above home."""
        return terrain_relative_up(z_above_home, self.home_msl_m, self.ground, x, y)

    def msl(self, up_terrain: float, x: float, y: float) -> float:
        """The MSL height of an aircraft `up_terrain` above the DEM at (x, y)."""
        return float(up_terrain) + float(self.ground.elevation_at(x, y))

    def state(self, st):
        """`st` (a models.State whose up is above home) with up above the DEM."""
        return st.model_copy(update={"up": self.up(st.up, st.x, st.y)})

    def record(self) -> dict:
        return {"ground": self.ground.record(), "up_ref": "terrain",
                "home_msl_m": self.home_msl_m, "home_source": self.home_source,
                "home_xy": list(self.home_xy),
                "dem_at_home_msl_m": round(self.dem_at_home_msl_m, 6)}


# Rule fields that hold a height, per altitude-banded rule type.
_BANDS = {"altitude_envelope": ("alt_min_m", "alt_max_m")}
_GEOM_BAND = ("altitude_floor_m", "altitude_ceiling_m")
# Bands that forbid what is INSIDE them (a zone between its floor and
# ceiling); every other band is a keep-in (altitude_envelope, the corridors).
KEEP_OUT_TYPES = frozenset({"polygon_fence", "dynamic_nfz", "circle_fence"})


def band_fields(rule) -> tuple[str, str]:
    """(floor field, ceiling field) of an altitude-banded rule."""
    return _BANDS.get(rule.type, _GEOM_BAND)


def is_msl(rule) -> bool:
    return getattr(rule, "altitude_ref", None) == "MSL"


def ground_problem(ground) -> str | None:
    """Why `ground` cannot convert an MSL height for the Shield, or None."""
    if ground is None:
        return ("the Shield measures height above ground and was given no "
                "ground-elevation source to convert mean-sea-level heights "
                "(Shield(ground=...), guardrail/terrain.py)")
    if is_dem(ground):
        return None
    if not getattr(ground, "flat", False) or not callable(getattr(ground, "elevation_at", None)):
        return (f"{ground!r} is not a flat ground: only a stated elevation per rail "
                f"(terrain.FlatGround) or a gridded elevation model "
                f"(terrain.GridGround) is implemented; any other ground that varies "
                f"with position is refused, not approximated")
    return None


def agl_rule(rule, ground, band: tuple[float, float] | None = None):
    """`rule` with its MSL band as the AGL band over `ground` (a copy; the
    rule itself is unchanged), or `rule` when it is AGL. Same id, same
    everything else: the Shield enforces the copy under the declared rule's
    name. `band`: the (lowest, highest) ground to convert over (the Shield's
    query-point band of a DEM); default the whole ground's (`ground_band`).
    A keep-in band shrinks by the ground's spread, a keep-out band grows by
    it (module docstring); on a flat ground both are the plain shift."""
    if not is_msl(rule):
        return rule
    why = ground_problem(ground)
    if why:
        raise ValueError(f"{rule.id}: altitude_ref MSL - {why}")
    lo, hi = ground_band(ground) if band is None else (float(band[0]), float(band[1]))
    f_lo, f_hi = _BANDS.get(rule.type, _GEOM_BAND)
    if rule.type in KEEP_OUT_TYPES:
        upd = {f_lo: float(getattr(rule, f_lo)) - hi, f_hi: float(getattr(rule, f_hi)) - lo}
    else:
        upd = {f_lo: float(getattr(rule, f_lo)) - lo, f_hi: float(getattr(rule, f_hi)) - hi}
    upd["altitude_ref"] = None
    return rule.model_copy(update=upd)


def msl_note(rule, ground, band: tuple[float, float] | None = None) -> str | None:
    """The words a CSP adds to an MSL rule's sentence: the declared MSL band
    and the ground it was converted over (None for an AGL rule)."""
    if not is_msl(rule):
        return None
    lo, hi = ground_band(ground) if band is None else band
    f_lo, f_hi = _BANDS.get(rule.type, _GEOM_BAND)
    g = f"{lo:g} m MSL" if lo == hi else f"{lo:g}-{hi:g} m MSL"
    return (f"heights above ground: declared {float(getattr(rule, f_lo)):g}-"
            f"{float(getattr(rule, f_hi)):g} m MSL over ground at {g}")
