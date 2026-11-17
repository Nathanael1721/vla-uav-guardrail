"""A declared world for the distance_envelope (WP1-07): buildings and roads
as rectangles, rasterised into the maps the Shield measures them on.

WHY THIS EXISTS

A distance_envelope over buildings or roads is measured on a map the Shield
is given at construction (Shield(obstacle_map=..., road_map=...); models.
DistanceEnvelope), and a Shield is refused over one without its map. The
city rails have maps (the CityLife occupancy grid and its street mask,
demo/out/citymap_citylife); the ArduPilot SITL rails - the desktop's and the
hil rig's, which fly the stress harness's scenarios - fly an empty world and
had none, so no scenario there could carry the rule at all (2026-11-17,
WP1-07: "no flight has carried a distance_envelope").

A world file states where the buildings and roads ARE, in the policy's own
frame (metres, x North, y East about the policy's origin - the frame every
scenario's start and target are written in), as axis-aligned rectangles;
experiments/worlds/de_block.yaml, the one shipped world, reads (its
comments and description left out):

    world: de-block-v1
    res_m: 1.0
    extent: {x: [-60.0, 120.0], y: [-50.0, 50.0]}
    buildings:
      - {id: block-a, x: [20.0, 40.0], y: [2.0, 22.0]}
    roads:
      - {id: road-w, x: [10.0, 60.0], y: [-18.0, -12.0]}

It is a small text file in the repository, so both hosts of the hil rig
(the desktop that plans and the Orin that flies) read the same world at the
commit they run, and its sha256 goes into the flight's record.

WHAT IT IS, AND WHAT IT IS NOT

The world is DECLARED, like a no-fly polygon: SITL's world has no such
building, so nothing physical is there to hit, and the rule is judged
against the rectangle the file names. It is the Shield's map exactly as the
CityLife occupancy grid is on the city rails: a cell is 1 (blocked / road)
when its CENTRE lies inside a rectangle (cell i is centred at ox + i*res,
demo/city_planner.py's convention, which the Shield reads), every building
is a column of unbounded height (the clearance runtime is 2-D), and off the
map is read as the Shield reads any map's outside (Shield._distance_at: a
lower bound from the map edge). A rectangle must lie inside the extent,
so a building never reaches past the map's edge."""
from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

# The map classes a world file may declare, and the Shield keyword each one
# becomes (models.DistanceEnvelope.object_class -> Shield(...)).
WORLD_CLASSES = {"buildings": "obstacle_map", "roads": "road_map"}
# A cell size below this would build a field of millions of cells per map.
MIN_RES_M = 0.25
MAX_CELLS = 4_000_000


@dataclass(frozen=True)
class World:
    """A loaded world: its maps (None for a class the file does not declare)
    and where it came from."""
    name: str
    path: str
    sha256: str
    res: float
    ox: float
    oy: float
    shape: tuple[int, int]
    rects: dict[str, list[dict]] = field(default_factory=dict)
    maps: dict[str, dict | None] = field(default_factory=dict)

    def shield_kwargs(self) -> dict[str, dict]:
        """{"obstacle_map": ..., "road_map": ...}: the maps this world has,
        as Shield(...) keyword arguments (a class it does not declare is left
        out, so the Shield's default - no map - stands)."""
        return {WORLD_CLASSES[c]: m for c, m in self.maps.items() if m is not None}

    def record(self) -> dict[str, Any]:
        """What a flight's record keeps of the world it was judged in."""
        return {"world": self.name, "path": self.path, "sha256": self.sha256,
                "res_m": self.res, "origin": [self.ox, self.oy],
                "shape": list(self.shape),
                "classes": {c: [r["id"] for r in self.rects.get(c, [])]
                            for c in WORLD_CLASSES if self.maps.get(c) is not None}}


def _span(v, what: str) -> tuple[float, float]:
    if (not isinstance(v, (list, tuple)) or len(v) != 2
            or not all(isinstance(a, (int, float)) and np.isfinite(a) for a in v)
            or not float(v[0]) < float(v[1])):
        raise ValueError(f"{what}: [min, max] with min < max, got {v!r}")
    return float(v[0]), float(v[1])


def load_world(path: str | Path, root: str | Path | None = None) -> World:
    """Read and rasterise a world file (module docstring). `root`: what a
    relative `path` is relative to (the repository root, as a scenario names
    it); the record keeps the path as given. Raises ValueError for a file
    that is not a world."""
    import yaml
    p = Path(path)
    full = p if p.is_absolute() or root is None else Path(root) / p
    raw_bytes = full.read_bytes()
    try:
        raw = yaml.safe_load(raw_bytes.decode("utf-8"))
    except (yaml.YAMLError, UnicodeDecodeError) as exc:
        raise ValueError(f"{path}: not a YAML world file ({exc})") from exc
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: a world file is a mapping")
    unknown = set(raw) - {"world", "description", "res_m", "extent", *WORLD_CLASSES}
    if unknown:
        raise ValueError(f"{path}: unknown key(s) {sorted(unknown)}")
    name = raw.get("world")
    if not isinstance(name, str) or not name:
        raise ValueError(f"{path}: `world` (a name) is required")
    res = raw.get("res_m")
    if not isinstance(res, (int, float)) or not res >= MIN_RES_M:
        raise ValueError(f"{path}: res_m must be a number >= {MIN_RES_M:g}, got {res!r}")
    res = float(res)
    ext = raw.get("extent") or {}
    if not isinstance(ext, dict) or set(ext) != {"x", "y"}:
        raise ValueError(f"{path}: extent is {{x: [min, max], y: [min, max]}}")
    x0, x1 = _span(ext["x"], f"{path}: extent.x")
    y0, y1 = _span(ext["y"], f"{path}: extent.y")
    n = int(round((x1 - x0) / res)) + 1
    m = int(round((y1 - y0) / res)) + 1
    if n * m > MAX_CELLS:
        raise ValueError(f"{path}: {n} x {m} cells is more than {MAX_CELLS}")
    xs = x0 + np.arange(n) * res
    ys = y0 + np.arange(m) * res
    rects: dict[str, list[dict]] = {}
    maps: dict[str, dict | None] = {}
    if not any(raw.get(c) for c in WORLD_CLASSES):
        raise ValueError(f"{path}: declares neither buildings nor roads")
    for cls in WORLD_CLASSES:
        items = raw.get(cls)
        if not items:
            maps[cls] = None
            continue
        if not isinstance(items, list):
            raise ValueError(f"{path}: {cls} is a list of rectangles")
        occ = np.zeros((n, m), dtype=np.uint8)
        seen: set[str] = set()
        out = []
        for k, r in enumerate(items):
            if not isinstance(r, dict) or set(r) != {"id", "x", "y"}:
                raise ValueError(f"{path}: {cls}[{k}] is {{id, x: [min, max], "
                                 f"y: [min, max]}}, got {r!r}")
            rid = str(r["id"])
            if rid in seen:
                raise ValueError(f"{path}: {cls} id {rid!r} is given twice")
            seen.add(rid)
            rx0, rx1 = _span(r["x"], f"{path}: {cls} {rid} x")
            ry0, ry1 = _span(r["y"], f"{path}: {cls} {rid} y")
            if rx0 < x0 or rx1 > x1 or ry0 < y0 or ry1 > y1:
                raise ValueError(f"{path}: {cls} {rid} reaches outside the extent "
                                 f"(a map's outside is read as clear past its edge)")
            ii = (xs >= rx0) & (xs <= rx1)
            jj = (ys >= ry0) & (ys <= ry1)
            if not ii.any() or not jj.any():
                raise ValueError(f"{path}: {cls} {rid} covers no cell centre at "
                                 f"res_m {res:g}")
            occ[np.ix_(ii, jj)] = 1
            out.append({"id": rid, "x": [rx0, rx1], "y": [ry0, ry1]})
        rects[cls] = out
        maps[cls] = {"occ": occ, "res": res, "ox": x0, "oy": y0}
    return World(name=name, path=str(path).replace("\\", "/"),
                 sha256=hashlib.sha256(raw_bytes).hexdigest(), res=res, ox=x0, oy=y0,
                 shape=(n, m), rects=rects, maps=maps)


def missing_maps(policy, world: World | None) -> list[str]:
    """The distance_envelope rules of `policy` whose map `world` does not
    have (all of them over buildings or roads for no world): what the Shield
    would refuse at construction, said before it is built."""
    from .models import DistanceEnvelope
    have = set() if world is None else {c for c, m in world.maps.items() if m is not None}
    return [f"{c.id}: a distance_envelope over {c.object_class} and "
            + ("no world file" if world is None else f"world {world.name} declares no "
                                                      f"{c.object_class}")
            for c in policy.by_type(DistanceEnvelope)
            if c.object_class in WORLD_CLASSES and c.object_class not in have]
