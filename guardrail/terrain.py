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
converts first with `terrain_relative_up`. No rail does either yet.

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
        z = self.z
        return float((1 - u) * (1 - v) * z[j, i] + u * (1 - v) * z[j, i + 1]
                     + (1 - u) * v * z[j + 1, i] + u * v * z[j + 1, i + 1])

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
