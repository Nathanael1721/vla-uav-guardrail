"""Purposeful routes for the CityLife pedestrians, and the reference model of how
a pedestrian should use a crossing.

WHY THIS EXISTS

The 40 `BP_CityPed` figures in CityLife_Day roam. A 1 s timer (`Roam`) pauses
them - 70 % a beat of 0.2-1 s, 30 % a real stop of 2-6 s - then sends them 14 m
within 30 deg of where they face (80 %) or anywhere (20 %), and home once they
are more than 20 m from their spawn (docs/FINDING-citylife-level.md, "Roam
paced, never paused, and mostly failed"). Nothing in that has a destination, so
from the air a figure walks a few metres, turns, and walks back. And because
the crossings are just navmesh (seven 600 cm gaps cut in the no-walk bands,
tools/citylife_mcp/crossings.py), a random goal can land ON a zebra: figures
stop there and idle 2-6 s, and one froze a junction for 33 s. They also never
look. In a 4-minute Simulate run cars logged 5 ticks on a zebra at > 50 cm/s
with a MOVING pedestrian on it, and the likely cause written down then was
exactly that: a figure stepping out in front of a car already inside its
braking distance (docs/FINDING-crowd-pedestrians-and-traffic.md, "Corners" and
"The zebra").

What replaces it here, in plain Python where it can be tested before any of it
is pushed into the Blueprint:

  1. A PAVEMENT GRAPH over the pedestrian area: nodes on both pavements of every
     street, KERB nodes at each crossing, and the ONLY edges that cross a
     carriageway are the crossings themselves.
  2. ROUTES on it: per pedestrian a closed tour to a sequence of destinations,
     no immediate backtracking, crossings included, so the Blueprint can loop
     it the way `BP_CityCar` loops its `Route`.
  3. PedModel, a WALK / WAIT / CROSS state machine: wait at the kerb for the
     walk phase AND a 4 s gap, then cross without stopping.

Why 4 s. The level's car stops 6.5 m (centre to crossing) short of a crossing
with a figure on it - its nose 1 m short of the zebra - on a 2.5 m/s^2 ramp.
From the fastest car speed, 5.8 m/s, that stop takes 6.7 m of braking, so
7.7 m of road in all. A 4 s gap puts that car 23 m from the zebra when the
figure leaves the kerb node; the figure, starting from rest at 2 m/s^2,
reaches the carriageway edge 200 cm and 1.56-1.66 s later (measured with
PedModel over the 126-154 cm/s speed range), when the car is still 13.6 m
out - 5.9 m more than it needs. At 2 s the same car would be 2 m from the
zebra when the figure stepped on it, well inside its braking distance.

WHAT THE LEVEL ALLOWS, AND WHAT THAT MEANS FOR THE GRAPH

The pedestrian area is the 200 x 185 m nav bounds: UE X 1500..21500, Y
-5000..13500, the extents crossings.py cut its bands to. Streets run on the
82 m grid through UE (4100 + 8200 i, 4100 + 8200 j). Every carriageway in the
area is a NavArea_Null band, so a pavement is walkable only around its own
block, and blocks join ONLY at the seven crossings, which sit around two
junctions. Hence, measured on the default build (see `summary()`):

  * the graph is NOT one component. The crossing network around junction
    (4100, 4100) is one; the other blocks are islands, and routes on an island
    contain no crossing. That is a fact about the level, not about this code:
    add a crossing to citylife_routes.CROSSINGS (and cut its gap) and the
    islands it touches join the network with no change here;
  * two of the seven crossings are unusable: (3000, -4100) and (5200, -4100)
    cross the Y = -4100 street, whose west kerb (Y = -5100) lies outside the
    nav bounds (Y >= -5000). A figure that crossed there would reach a sliver
    under a metre wide and have nowhere to go.

Measured 2026-09-29 on the default build, with the street mask: 88 nodes, 86
edges, 10 kerb nodes, 14 dead ends (pavement running into the edge of the nav
bounds); 8 nodes snapped onto a street cell, none dropped. Components: the
crossing network, 46 nodes / 796 m / 4 blocks / 5 crossings; two closed block
rings of 252 m; four open 63 m runs and one 16 m stub along the far edges of
the area. The 40 routes: 24 on the network (each crosses at least twice, all
five crossings used), 8 on each ring, none on the 63 m runs or the stub - a
figure on a path with a dead end at both ends could only pace, which is the
old roaming look, so `plan_tours` places nobody on a component without a
cycle; 40-69 legs. On the network a route turns round only at a dead end, 25
times in 1159 legs; on the rings never. Without the mask the graph has the
same size but no node is snapped, so the ROUTES differ: generate them where
demo/out/citymap_citylife/street.npz exists (it is gitignored).

FRAMES

UE centimetres, X = NORTH, Y = EAST (NED metres x 100, no offset), the same
as tools/citylife_routes.py. A street "with centre line X = c" therefore runs
EAST-WEST, and a pedestrian crossing it walks along X. `Crossing.axis` is the
axis the pedestrian walks along: 0 = X, 1 = Y. Yaw is Unreal's: 0 = +X
(north), 90 = +Y (east), so a heading is atan2(dy, dx).

NODE KINDS in a route (the z of each point): 0 pavement, 1 kerb-wait (the next
leg is a crossing: WAIT here), 2 crossing-exit (the previous leg was one). A
kerb node that a route walks past without crossing is encoded 0.
"""
from __future__ import annotations

import bisect
import heapq
import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Tuple, Union

try:                                    # tests and `python tools/citylife_peds.py`
    import citylife_routes as R
except ImportError:                     # `python -m tools.citylife_peds` from the repo root
    from tools import citylife_routes as R

ROOT = Path(__file__).resolve().parents[1]
MASK_PATH = ROOT / "demo" / "out" / "citymap_citylife" / "street.npz"

Vec = Tuple[float, float]

# ---------------------------------------------------------------- geometry

AREA = (1500.0, 21500.0, -5000.0, 13500.0)   # X_LO, X_HI, Y_LO, Y_HI: the nav bounds
GRID_ORIGIN_CM = 4100.0                      # a junction centre on the 82 m grid
PAVEMENT_OFF_CM = 950.0                      # pavement node line from the road centre
KERB_OFF_CM = 1000.0                         # kerb-wait node from the road centre
NODE_SPACING_CM = 2000.0                     # about one pavement node per 20 m
ZEBRA_HALF_CM = 300.0                        # half width of a crossing gap (crossings.py GAP)
AGENT_RADIUS_CM = 35.0                       # the navmesh is eroded by this at its edges
PAVEMENT_MIN_CM = 900.0                      # nothing but a crossing comes nearer a centre line
MIN_STUB_CM = 1000.0                         # a dead-end pavement shorter than this is not kept
MIN_LEG_CM = 400.0                           # a pavement node this near a kerb node is dropped

KIND_PAVEMENT, KIND_KERB_WAIT, KIND_CROSS_EXIT = 0, 1, 2

# ---------------------------------------------------------------- behaviour

ARRIVE_CM = 120.0              # a node counts as reached inside this radius
PAUSE_P = 0.10                 # chance of a pause at a pavement node
PAUSE_S = (2.0, 6.0)
GAP_S = 4.0                    # no car may reach the zebra sooner than this
CROSS_SPEED_FACTOR = 1.25
CROSS_SPEED_MAX_CMS = 180.0
WALK_SPEED_CMS = (126.0, 154.0)   # the level's per-figure MaxWalkSpeed range
PED_ACCEL_CMS2 = 200.0         # BP_CityPed MaxAcceleration
PED_DECEL_CMS2 = 260.0         # BP_CityPed BrakingDecelerationWalking
MOVING_CAR_CMS = 50.0          # the PedViol threshold the car findings use
STILL_PED_CMS = 10.0           # Roam's own "has stopped" test
CAR_LEN_CM, CAR_WID_CM = 500.0, 250.0
STRAIGHT_WINDOW_S = 60.0

WALK, WAIT, CROSS = "WALK", "WAIT", "CROSS"

SEED = 20260929


def grid_lines(lo: float, hi: float, grid: float = R.GRID_CM,
               origin: float = GRID_ORIGIN_CM) -> List[float]:
    """Street centre lines of the 82 m grid that fall inside [lo, hi]."""
    k0 = math.ceil((lo - origin) / grid)
    k1 = math.floor((hi - origin) / grid)
    return [origin + k * grid for k in range(k0, k1 + 1)]


def _on_grid(v: float, grid: float = R.GRID_CM, origin: float = GRID_ORIGIN_CM) -> bool:
    k = (v - origin) / grid
    return abs(k - round(k)) * grid < 1.0


def in_carriageway(x: float, y: float, xs: Sequence[float], ys: Sequence[float],
                   half: float = R.CARRIAGEWAY_HALF_CM) -> bool:
    """Inside any carriageway band: |X - c| < half for a street with centre line
    X = c (it runs east-west), or |Y - c| < half for one with Y = c."""
    return any(abs(x - c) < half for c in xs) or any(abs(y - c) < half for c in ys)


def in_area(x: float, y: float, area=AREA, margin: float = AGENT_RADIUS_CM) -> bool:
    return (area[0] + margin <= x <= area[1] - margin
            and area[2] + margin <= y <= area[3] - margin)


# ---------------------------------------------------------------- street mask

def load_mask(path: Union[str, Path] = MASK_PATH):
    """The CityLife street mask (NED metres, 2 m cells), or None if not built.

    Reuses demo/build_street_mask.py rather than re-reading the grid here: its
    `is_street` rounds to the nearest cell centre, and truncating instead was
    measured to disagree on 9.4 % of points."""
    p = Path(path)
    if not p.is_file():
        return None
    _demo_on_path()
    from build_street_mask import load_street
    return load_street(p)


def _demo_on_path() -> None:
    demo = str(ROOT / "demo")
    if demo not in sys.path:
        sys.path.insert(0, demo)


def on_street(mask, x_cm: float, y_cm: float) -> bool:
    """UE cm -> NED metres (X = north = x, Y = east = y, / 100) -> the mask."""
    _demo_on_path()                     # a mask loaded elsewhere may be passed in
    from build_street_mask import is_street
    return is_street(mask, x_cm / 100.0, y_cm / 100.0)


# ---------------------------------------------------------------- the graph

@dataclass(frozen=True)
class Crossing:
    """One usable crossing. `id` is its index in the list build_graph was given
    (citylife_routes.CROSSINGS by default), so signals can key on it."""
    id: int
    x: float
    y: float
    axis: int                   # 0: the pedestrian walks along X; 1: along Y
    kerb_a: Vec                 # kerb node on the low side of the road
    kerb_b: Vec                 # kerb node on the high side

    def local(self, px: float, py: float) -> Vec:
        """(across, along): across = along the walking direction, from the
        road centre line; along = along the road, from the crossing centre."""
        if self.axis == 0:
            return px - self.x, py - self.y
        return py - self.y, px - self.x

    def in_zebra(self, px: float, py: float, pad: float = 0.0) -> bool:
        """On the painted part: the carriageway, inside the crossing corridor."""
        a, b = self.local(px, py)
        return abs(a) <= R.CARRIAGEWAY_HALF_CM + pad and abs(b) <= ZEBRA_HALF_CM + pad

    def walk_dir(self) -> Vec:
        return (1.0, 0.0) if self.axis == 0 else (0.0, 1.0)


@dataclass
class PedGraph:
    pos: List[Vec]                         # UE cm
    xing_at: List[int]                     # crossing id at a kerb node, -1 elsewhere
    adj: List[List[int]]
    crossings: List[Crossing]              # the usable ones
    block: List[int]                       # sidewalk island (pavement edges only)
    comp: List[int]                        # connected component (with crossings)
    xs: Tuple[float, ...]                  # centre lines X = c of the streets in the area
    ys: Tuple[float, ...]                  # centre lines Y = c
    area: Tuple[float, float, float, float]
    report: Dict[str, object] = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.pos)

    def kind(self, i: int) -> int:
        return KIND_KERB_WAIT if self.xing_at[i] >= 0 else KIND_PAVEMENT

    def is_crossing_edge(self, i: int, j: int) -> bool:
        return self.xing_at[i] >= 0 and self.xing_at[i] == self.xing_at[j] and i != j

    def length(self, i: int, j: int) -> float:
        return math.hypot(self.pos[j][0] - self.pos[i][0], self.pos[j][1] - self.pos[i][1])

    def edges(self) -> List[Tuple[int, int]]:
        return [(i, j) for i in range(len(self.pos)) for j in self.adj[i] if i < j]

    def components(self) -> Dict[int, List[int]]:
        out: Dict[int, List[int]] = {}
        for i, c in enumerate(self.comp):
            out.setdefault(c, []).append(i)
        return out

    def crossing(self, xid: int) -> Crossing:
        for c in self.crossings:
            if c.id == xid:
                return c
        raise KeyError(xid)

    def crossing_components(self) -> List[int]:
        return sorted({self.comp[i] for i in range(len(self.pos)) if self.xing_at[i] >= 0})


def _pt(axis: int, off: float, a: float) -> Vec:
    """A point on a pavement line. axis 0: the line is X = off and runs along Y."""
    return (off, a) if axis == 0 else (a, off)


def _key(p: Vec) -> Tuple[int, int]:
    return (int(round(p[0])), int(round(p[1])))


def _pieces(area, xs, ys):
    """Straight pavement runs: (axis, off, street centre, a0, a1, open0, open1).

    A run goes from one block corner (950 cm from both centre lines) to the
    next, or to the edge of the area, which is an OPEN end - a dead end in the
    graph. Open runs shorter than MIN_STUB_CM are dropped."""
    x_lo, x_hi, y_lo, y_hi = area
    m = AGENT_RADIUS_CM
    out = []
    for axis, centres, cross, (flo, fhi), (alo, ahi) in (
            (0, xs, ys, (x_lo, x_hi), (y_lo, y_hi)),
            (1, ys, xs, (y_lo, y_hi), (x_lo, x_hi))):
        for c in centres:
            for side in (-1, 1):
                off = c + side * PAVEMENT_OFF_CM
                if not (flo + m <= off <= fhi - m):
                    continue
                starts = [(alo + m, True)] + [(cc + PAVEMENT_OFF_CM, False) for cc in cross]
                ends = [(cc - PAVEMENT_OFF_CM, False) for cc in cross] + [(ahi - m, True)]
                for (a0, o0), (a1, o1) in zip(starts, ends):
                    length = a1 - a0
                    if length <= 0 or ((o0 or o1) and length < MIN_STUB_CM):
                        continue
                    out.append((axis, off, c, a0, a1, o0, o1))
    return out


def _snap(p: Vec, role: str, meta: dict, mask, xs, ys, area) -> Optional[Vec]:
    """Nearest acceptable point to p, or None.

    Acceptable: inside the area, at least PAVEMENT_MIN_CM from every centre
    line (a kerb node's own road excepted, which it is 1000 cm from anyway), and
    a STREET cell in the mask. Candidates depend on the role: a pavement node
    slides along its line and a little away from the road, a corner moves in
    both axes, a kerb node stays in its crossing's corridor."""
    def ok(q: Vec) -> bool:
        if not in_area(q[0], q[1], area):
            return False
        if any(abs(q[0] - c) < PAVEMENT_MIN_CM for c in xs) or \
           any(abs(q[1] - c) < PAVEMENT_MIN_CM for c in ys):
            return False
        return mask is None or on_street(mask, q[0], q[1])

    cands: List[Tuple[float, Vec]] = []
    if role == "kerb":
        axis, sgn = meta["axis"], meta["side"]
        for da in (0.0, 50.0, 100.0, -50.0):            # further from the road first
            for db in (0.0, 100.0, -100.0):
                q = (p[0] + sgn * da, p[1] + db) if axis == 0 else (p[0] + db, p[1] + sgn * da)
                cands.append((math.hypot(da, db), q))
    elif role == "regular":
        axis, sgn = meta["axis"], meta["side"]
        for dl in (0.0, 50.0, 100.0, -50.0):
            for da in (0.0, 100.0, -100.0, 200.0, -200.0, 300.0, -300.0):
                q = (p[0] + sgn * dl, p[1] + da) if axis == 0 else (p[0] + da, p[1] + sgn * dl)
                cands.append((math.hypot(dl, da), q))
    else:                                                # corner or open end
        for dx in (0.0, 100.0, -100.0, 200.0, -200.0):
            for dy in (0.0, 100.0, -100.0, 200.0, -200.0):
                cands.append((math.hypot(dx, dy), (p[0] + dx, p[1] + dy)))
    cands.sort(key=lambda c: c[0])
    for _, q in cands:
        if ok(q):
            return q
    return None


def build_graph(area=AREA, crossings: Sequence[Vec] = None, mask="default",
                spacing: float = NODE_SPACING_CM) -> PedGraph:
    """The pavement graph. `mask="default"` loads the CityLife street mask if it
    has been built; pass None to skip the mask check."""
    crossings = list(R.CROSSINGS if crossings is None else crossings)
    if isinstance(mask, str) and mask == "default":
        mask = load_mask()
    xs = tuple(grid_lines(area[0], area[1]))
    ys = tuple(grid_lines(area[2], area[3]))
    pieces = _pieces(area, xs, ys)

    # Raw nodes, keyed by position so that a corner shared by two runs is one node.
    raw: Dict[Tuple[int, int], dict] = {}
    runs: List[List[Tuple[float, Tuple[int, int]]]] = []   # per piece: (along, key)
    for axis, off, c, a0, a1, o0, o1 in pieces:
        side = 1.0 if off > c else -1.0
        n = max(1, int(round((a1 - a0) / spacing)))
        run = []
        for k in range(n + 1):
            a = a0 + (a1 - a0) * k / n
            p = _pt(axis, off, a)
            role = "regular"
            if k == 0:
                role = "end" if o0 else "corner"
            elif k == n:
                role = "end" if o1 else "corner"
            key = _key(p)
            node = raw.setdefault(key, {"p": p, "role": role, "axis": axis, "side": side,
                                        "xing": -1})
            if role != "regular":
                node["role"] = "corner" if node["role"] == "corner" or role == "corner" else role
            run.append((a, key))
        runs.append(run)

    unusable: Dict[int, str] = {}
    kerbs: Dict[int, Tuple[Tuple[int, int], Tuple[int, int], int]] = {}
    for xid, (cx, cy) in enumerate(crossings):
        gx, gy = _on_grid(cx), _on_grid(cy)
        if gx == gy:
            unusable[xid] = "not on exactly one street centre line"
            continue
        axis = 0 if gx else 1
        c, along = (cx, cy) if axis == 0 else (cy, cx)
        found = []
        for side in (-1.0, 1.0):
            kp = _pt(axis, c + side * KERB_OFF_CM, along)
            if not in_area(kp[0], kp[1], area):
                found.append(None)
                unusable[xid] = f"kerb {kp} is outside the pedestrian area"
                continue
            want = c + side * PAVEMENT_OFF_CM
            hit = None
            for pi, (pax, off, _, a0, a1, _, _) in enumerate(pieces):
                if pax == axis and abs(off - want) < 1.0 and a0 - 1.0 <= along <= a1 + 1.0:
                    hit = pi
                    break
            if hit is None:
                unusable[xid] = f"no pavement at kerb {kp}"
            found.append((kp, hit, side) if hit is not None else None)
        if None in found:
            continue
        keys = []
        for kp, pi, side in found:
            key = _key(kp)
            raw[key] = {"p": kp, "role": "kerb", "axis": axis, "side": side, "xing": xid}
            runs[pi].append((along, key))
            keys.append(key)
        kerbs[xid] = (keys[0], keys[1], axis)

    # A pavement node crowding a kerb node only makes a short, pointless leg.
    for run in runs:
        kerb_as = [a for a, k in run if raw[k]["role"] == "kerb"]
        run[:] = sorted((a, k) for a, k in run
                        if raw[k]["role"] != "regular"
                        or all(abs(a - ka) >= MIN_LEG_CM for ka in kerb_as))

    # Snap every node onto a pavement cell of the mask, or drop it.
    snapped, dropped = 0, []
    final: Dict[Tuple[int, int], Vec] = {}
    for key in sorted(raw):
        node = raw[key]
        q = _snap(node["p"], node["role"], node, mask, xs, ys, area)
        if q is None:
            dropped.append((node["role"], node["p"]))
            continue
        if q != node["p"]:
            snapped += 1
        final[key] = q
    for xid, (ka, kb, _) in list(kerbs.items()):
        if ka not in final or kb not in final:
            unusable[xid] = "a kerb node is not on a pavement cell of the street mask"
            del kerbs[xid]
            for k in (ka, kb):
                if k in raw:
                    raw[k]["xing"] = -1

    # Edges: consecutive survivors along each run, plus the crossings.
    nbr: Dict[Tuple[int, int], set] = {k: set() for k in final}
    for run in runs:
        alive = [k for _, k in run if k in final]
        for a, b in zip(alive, alive[1:]):
            if a != b:
                nbr[a].add(b)
                nbr[b].add(a)
    for xid, (ka, kb, _) in kerbs.items():
        nbr[ka].add(kb)
        nbr[kb].add(ka)
    # A kerb with no pavement behind it would make its crossing a trap.
    for xid, (ka, kb, _) in list(kerbs.items()):
        if len(nbr[ka]) < 2 or len(nbr[kb]) < 2:
            unusable[xid] = "a kerb node has no pavement neighbour"
            nbr[ka].discard(kb)
            nbr[kb].discard(ka)
            raw[ka]["xing"] = raw[kb]["xing"] = -1
            del kerbs[xid]
    isolated = [k for k in nbr if not nbr[k]]
    keys = sorted((k for k in nbr if nbr[k]), key=lambda k: (final[k][0], final[k][1]))
    index = {k: i for i, k in enumerate(keys)}
    pos = [final[k] for k in keys]
    xing_at = [raw[k]["xing"] if raw[k]["xing"] in kerbs else -1 for k in keys]
    adj = [sorted(index[n] for n in nbr[k]) for k in keys]

    xings = []
    for xid in sorted(kerbs):
        ka, kb, axis = kerbs[xid]
        pa, pb = final[ka], final[kb]
        if (pa[axis] > pb[axis]):
            pa, pb = pb, pa
        cx, cy = crossings[xid]
        xings.append(Crossing(xid, cx, cy, axis, pa, pb))

    def label(use_crossings: bool) -> List[int]:
        lab = [-1] * len(pos)
        nxt = 0
        for s in range(len(pos)):
            if lab[s] >= 0:
                continue
            lab[s] = nxt
            stack = [s]
            while stack:
                u = stack.pop()
                for v in adj[u]:
                    if lab[v] < 0 and (use_crossings or not (
                            xing_at[u] >= 0 and xing_at[u] == xing_at[v])):
                        lab[v] = nxt
                        stack.append(v)
            nxt += 1
        return lab

    g = PedGraph(pos=pos, xing_at=xing_at, adj=adj, crossings=xings, block=label(False),
                 comp=label(True), xs=xs, ys=ys, area=tuple(area))
    g.report = {
        "mask": mask is not None,
        "pieces": len(pieces),
        "snapped": snapped,
        "dropped": dropped,
        "isolated_removed": len(isolated),
        "unusable_crossings": {crossings[k]: v for k, v in sorted(unusable.items())},
    }
    return g


def summary(g: PedGraph) -> Dict[str, object]:
    comps = g.components()
    main = g.crossing_components()
    return {
        "nodes": len(g), "edges": len(g.edges()),
        "kerb_nodes": sum(1 for k in g.xing_at if k >= 0),
        "dead_ends": sum(1 for a in g.adj if len(a) == 1),
        "crossings_usable": [(c.id, (c.x, c.y)) for c in g.crossings],
        "crossings_unusable": {str(k): v for k, v in
                               g.report.get("unusable_crossings", {}).items()},
        "components": {c: {"nodes": len(v), "blocks": len({g.block[i] for i in v}),
                           "length_m": round(sum(g.length(i, j) for i in v for j in g.adj[i]
                                                 if i < j) / 100.0, 1),
                           "crossings": sum(1 for i in v if g.xing_at[i] >= 0) // 2}
                       for c, v in sorted(comps.items())},
        "crossing_network": main,
        "mask": g.report.get("mask"), "snapped": g.report.get("snapped"),
        "dropped": len(g.report.get("dropped", [])),
    }


# ---------------------------------------------------------------- routes

def _nb_path(g: PedGraph, prev: int, cur: int, goal: Callable[[int, int], bool]
             ) -> Optional[List[int]]:
    """Shortest walk from `cur` (arrived from `prev`, -1 for none) with no
    immediate backtracking - except at a dead end, where turning round is the
    only move - to the first state (p, c) with goal(p, c). Returns the nodes
    after `cur`, or None. Dijkstra over directed edges, so it is exact."""
    start = (prev, cur)
    dist = {start: 0.0}
    parent: Dict[Tuple[int, int], Tuple[int, int]] = {}
    heap = [(0.0, 0, prev, cur)]
    tick = 1
    while heap:
        d, _, p, c = heapq.heappop(heap)
        if d > dist.get((p, c), math.inf):
            continue
        if (p, c) != start and goal(p, c):
            out, s = [], (p, c)
            while s != start:
                out.append(s[1])
                s = parent[s]
            return out[::-1]
        for n in g.adj[c]:
            if n == p and len(g.adj[c]) > 1:
                continue
            nd = d + g.length(c, n)
            if nd < dist.get((c, n), math.inf):
                dist[(c, n)] = nd
                parent[(c, n)] = (p, c)
                heapq.heappush(heap, (nd, tick, c, n))
                tick += 1
    return None


def _allocate(n: int, weights: Dict[int, float]) -> Dict[int, int]:
    """Largest-remainder split of n over the keys, proportional to weight."""
    tot = sum(weights.values())
    if tot <= 0 or n <= 0:
        return {k: 0 for k in weights}
    exact = {k: n * w / tot for k, w in weights.items()}
    out = {k: int(math.floor(v)) for k, v in exact.items()}
    rest = sorted(weights, key=lambda k: (-(exact[k] - out[k]), k))
    for k in rest[:n - sum(out.values())]:
        out[k] += 1
    return out


def plan_tours(g: PedGraph, n_peds: int = 40, legs: int = 40, seed: int = SEED
               ) -> List[List[int]]:
    """Per pedestrian a closed tour of node ids (the last node leads back to
    the first), at least `legs` legs long.

    Pedestrians are shared out over the components in proportion to their
    pavement length - but only over components that contain a CYCLE. A
    component that is a bare path (the 63 m runs along the far edges of the
    area, dead ends at both ends) admits no tour without turning round every
    few legs: a figure there can only pace to and fro, which is exactly the
    old Roam's look and the "no immediate backtracking" rule forbids. Those
    pavements are left empty; everyone is placed where a tour can flow.

    A tour is a chain of DESTINATIONS, each reached by the shortest walk with
    no immediate backtracking; on the crossing network the first destination
    is always in another block, so every tour there crosses at least once, and
    later ones are in another block 70 % of the time. Returns fewer than
    `n_peds` tours only if no component has a cycle (then: none)."""
    rng = random.Random(seed)
    comps = g.components()
    weights: Dict[int, float] = {}
    pools: Dict[int, List[int]] = {}
    for c, nodes in comps.items():
        n_edges = sum(len(g.adj[i]) for i in nodes) // 2
        pool = [i for i in nodes if g.xing_at[i] < 0 and len(g.adj[i]) >= 2]
        if n_edges >= len(nodes) and len(pool) >= 2:        # connected + a cycle
            weights[c] = sum(g.length(i, j) for i in nodes for j in g.adj[i] if i < j)
            pools[c] = pool
    share = _allocate(n_peds, weights)
    tours: List[List[int]] = []
    for c in sorted(share):
        nodes = comps[c]
        pool = pools[c]
        blocks = sorted({g.block[i] for i in nodes})
        starts = pool[:]
        rng.shuffle(starts)
        for k in range(share[c]):
            v0 = starts[k % len(starts)]
            tours.append(_one_tour(g, v0, pool, len(blocks) > 1, legs, rng))
    return tours


def _one_tour(g: PedGraph, v0: int, pool: List[int], multi_block: bool, legs: int,
              rng: random.Random) -> List[int]:
    walk = [v0]
    prev = -1
    first = True
    for _ in range(10 * legs):
        cur = walk[-1]
        if len(walk) > 2:
            v1 = walk[1]
            back = _nb_path(g, prev, cur, lambda p, c: c == v0 and (
                p != v1 or len(g.adj[v0]) == 1))
            if back is not None and len(walk) - 1 + len(back) >= legs:
                tour = walk + back
                return tour[:-1]
        other = multi_block and (first or rng.random() < 0.7)
        cands = [i for i in pool if i != cur and (g.block[i] != g.block[cur]) == other]
        far = [i for i in cands if g.length(cur, i) >= 3000.0]
        dest = rng.choice(far or cands or [i for i in pool if i != cur])
        seg = _nb_path(g, prev, cur, lambda p, c, d=dest: c == d)
        if seg is None:
            continue
        first = False
        walk.extend(seg)
        prev = walk[-2]
    raise RuntimeError(f"no closed tour from node {v0} within {10 * legs} destinations")


def encode_tour(g: PedGraph, tour: Sequence[int]) -> List[Tuple[float, float, float]]:
    """Node ids -> (x, y, kind) in UE cm, cyclic: the leg after the last point
    goes back to the first."""
    n = len(tour)
    out = []
    for i, v in enumerate(tour):
        nxt, prv = tour[(i + 1) % n], tour[i - 1]
        if g.is_crossing_edge(v, nxt):
            k = KIND_KERB_WAIT
        elif g.is_crossing_edge(prv, v):
            k = KIND_CROSS_EXIT
        else:
            k = KIND_PAVEMENT
        out.append((float(g.pos[v][0]), float(g.pos[v][1]), float(k)))
    return out


def make_routes(n_peds: int = 40, legs: int = 40, seed: int = SEED,
                graph: PedGraph = None) -> List[List[Tuple[float, float, float]]]:
    """Per pedestrian, a closed tour as [(x, y, kind)] in UE cm. Deterministic
    for a given seed (and graph)."""
    g = graph if graph is not None else build_graph()
    return [encode_tour(g, t) for t in plan_tours(g, n_peds, legs, seed)]


def plan(n_peds: int = 40, legs: int = 40, seed: int = SEED, graph: PedGraph = None
         ) -> Dict[str, object]:
    """Everything a level writer needs, JSON-able: the routes (a figure starts
    at its route's first point, which is always a pavement node) and the
    crossings whose zebra box and kerbs the Blueprint's WAIT check uses."""
    g = graph if graph is not None else build_graph()
    return {
        "frame": "UE cm, X = north, Y = east; route z = kind: 0 pavement, "
                 "1 kerb-wait, 2 crossing-exit; a route is closed (last -> first)",
        "seed": seed,
        "crossings": [{"id": c.id, "centre": [c.x, c.y], "walk_axis": "X" if c.axis == 0 else "Y",
                       "kerb_a": list(c.kerb_a), "kerb_b": list(c.kerb_b),
                       "zebra_half": [R.CARRIAGEWAY_HALF_CM, ZEBRA_HALF_CM]}
                      for c in g.crossings],
        "routes": [[list(p) for p in r] for r in make_routes(n_peds, legs, seed, graph=g)],
    }


# ---------------------------------------------------------------- cars

@dataclass
class CarView:
    """What a pedestrian can know about a car. `path`/`s` are optional: with
    them the time to the zebra is measured ALONG the car's path (a car about to
    turn is judged on the arc it will drive), without them along its heading."""
    x: float
    y: float
    yaw_deg: float              # Unreal yaw: 0 = +X (north), 90 = +Y (east)
    v: float                    # cm/s
    path: object = None         # a citylife_routes.Path
    s: Optional[float] = None   # arc length of the car's centre along `path`, cm


def _obb_overlap(c1: Vec, u1: Vec, h1: Vec, c2: Vec, u2: Vec, h2: Vec) -> bool:
    """Two oriented rectangles (centre, unit long axis, half extents) overlap."""
    d = (c2[0] - c1[0], c2[1] - c1[1])
    p1, p2 = (-u1[1], u1[0]), (-u2[1], u2[0])
    for ax in (u1, p1, u2, p2):
        r1 = h1[0] * abs(u1[0] * ax[0] + u1[1] * ax[1]) + h1[1] * abs(p1[0] * ax[0] + p1[1] * ax[1])
        r2 = h2[0] * abs(u2[0] * ax[0] + u2[1] * ax[1]) + h2[1] * abs(p2[0] * ax[0] + p2[1] * ax[1])
        if abs(d[0] * ax[0] + d[1] * ax[1]) > r1 + r2:
            return False
    return True


def _car_axes(car: CarView) -> Vec:
    y = math.radians(car.yaw_deg)
    return (math.cos(y), math.sin(y))


def car_on_zebra(car: CarView, xing: Crossing) -> bool:
    """The car's 500 x 250 cm footprint overlaps the zebra box."""
    return _obb_overlap((car.x, car.y), _car_axes(car), (CAR_LEN_CM / 2, CAR_WID_CM / 2),
                        (xing.x, xing.y), xing.walk_dir(),
                        (R.CARRIAGEWAY_HALF_CM, ZEBRA_HALF_CM))


def ped_in_car(car: CarView, px: float, py: float) -> bool:
    f = _car_axes(car)
    dx, dy = px - car.x, py - car.y
    return (abs(dx * f[0] + dy * f[1]) <= CAR_LEN_CM / 2
            and abs(-dx * f[1] + dy * f[0]) <= CAR_WID_CM / 2)


def path_arc(path) -> Tuple[List[float], float]:
    """Cumulative arc length at each point of a closed path, and its length."""
    cached = getattr(path, "_ped_arc", None)
    if cached is not None:
        return cached
    pts = path.pts
    cum = [0.0]
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    out = (cum[:-1], cum[-1])
    try:
        path._ped_arc = out
    except AttributeError:
        pass
    return out


def path_point(path, s: float) -> Tuple[float, float, float]:
    """(x, y, yaw_deg) at arc length s along a closed path."""
    cum, total = path_arc(path)
    s %= total
    i = max(0, bisect.bisect_right(cum, s) - 1)
    a, b = path.pts[i], path.pts[(i + 1) % len(path.pts)]
    seg = (cum[i + 1] if i + 1 < len(cum) else total) - cum[i]
    f = (s - cum[i]) / seg if seg > 0 else 0.0
    return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f,
            math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])))


def _project_s(path, x: float, y: float) -> float:
    cum, _ = path_arc(path)
    pts, n = path.pts, len(path.pts)
    best, s_best = math.inf, 0.0
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / L2))
        d = math.hypot(x - a[0] - u * dx, y - a[1] - u * dy)
        if d < best:
            best, s_best = d, cum[i] + u * math.sqrt(L2)
    return s_best


def crossing_s(path, xing: Crossing) -> Optional[float]:
    """Arc length along `path` where it drives over the crossing's centre line,
    or None if the path never does (citylife_routes.crossings_on decides)."""
    cache = getattr(path, "_ped_xs", None)
    if cache is None:
        cache = {}
        try:
            path._ped_xs = cache
        except AttributeError:
            pass
    key = (xing.x, xing.y)
    if key not in cache:
        hit = R.crossings_on(path, [key])
        cache[key] = _project_s(path, xing.x, xing.y) if hit else None
    return cache[key]


def time_to_zebra(car: CarView, xing: Crossing, horizon_s: float = 2 * GAP_S) -> float:
    """Seconds until the car's body reaches the zebra box: 0 while a moving car
    is on it, inf for one that will not get there within `horizon_s` (beyond
    that the answer changes no decision).

    A car under MOVING_CAR_CMS is STANDING and never arrives, even one standing
    on the zebra: the rule is "no MOVING car in the box", and in the level a
    car stands on a zebra only for its 3 s emergency stop or in a queue, while
    a car that yields stops 6.5 m short, outside the box. Holding every figure
    for a parked body would also make the figure and the car wait for each
    other."""
    if car.v < MOVING_CAR_CMS:
        return math.inf
    reach = ZEBRA_HALF_CM + CAR_LEN_CM / 2
    if car.path is not None:
        s_x = crossing_s(car.path, xing)
        if s_x is None:
            return 0.0 if car_on_zebra(car, xing) else math.inf
        _, total = path_arc(car.path)
        s_car = car.s if car.s is not None else _project_s(car.path, car.x, car.y)
        d = (s_x - s_car) % total                     # forward, centre to crossing
        if d <= reach or d >= total - reach:
            return 0.0
        t = (d - reach) / car.v
        return t if t <= horizon_s else math.inf
    # No path: sweep the footprint along the heading.
    if car_on_zebra(car, xing):
        return 0.0
    if math.hypot(xing.x - car.x, xing.y - car.y) - (reach + R.CARRIAGEWAY_HALF_CM) > car.v * horizon_s:
        return math.inf
    f = _car_axes(car)
    step = 50.0
    s = step
    while s <= car.v * horizon_s:
        probe = CarView(car.x + f[0] * s, car.y + f[1] * s, car.yaw_deg, car.v)
        if car_on_zebra(probe, xing):
            return s / car.v
        s += step
    return math.inf


# ---------------------------------------------------------------- counters

@dataclass
class PedCounters:
    """What a pedestrian did that a reviewer would call wrong, plus how it walked.

    Position-based wherever it can be (zebra idling, car overlap, straightness),
    so the same class can score a pedestrian recorded in the engine; the two
    decision counters are fed at the moment a crossing starts."""
    walk_viol: int = 0              # started to cross on don't-walk
    gap_viol: int = 0               # started with a car < GAP_S from the zebra
    zebra_idle_s: float = 0.0       # seconds stopped (< 10 cm/s) on a zebra
    car_overlap: int = 0            # times the ped's centre entered a car footprint
    car_overlap_s: float = 0.0
    kerb_wait_max_s: float = 0.0
    crossings: int = 0
    waits: List[float] = field(default_factory=list)   # each completed kerb wait, s
    track: List[Tuple[float, float, float, float]] = field(default_factory=list)
    _in_car: bool = False
    _travelled: float = 0.0
    _last: Optional[Vec] = None

    def on_cross_start(self, walk_now: bool, min_tta: float) -> None:
        self.crossings += 1
        if not walk_now:
            self.walk_viol += 1
        if min_tta < GAP_S:
            self.gap_viol += 1

    def on_wait(self, waited_s: float, done: bool = False) -> None:
        self.kerb_wait_max_s = max(self.kerb_wait_max_s, waited_s)
        if done:
            self.waits.append(waited_s)

    def observe(self, t: float, dt: float, x: float, y: float, speed: float,
                cars: Sequence[CarView], xings: Sequence[Crossing]) -> None:
        if self._last is not None:
            self._travelled += math.hypot(x - self._last[0], y - self._last[1])
        self._last = (x, y)
        if speed < STILL_PED_CMS and any(c.in_zebra(x, y) for c in xings):
            self.zebra_idle_s += dt
        inside = any(abs(c.x - x) < 300.0 and abs(c.y - y) < 300.0 and ped_in_car(c, x, y)
                     for c in cars)
        if inside:
            self.car_overlap_s += dt
            if not self._in_car:
                self.car_overlap += 1
        self._in_car = inside
        if not self.track or t - self.track[-1][0] >= 1.0 - 1e-9:
            self.track.append((t, x, y, self._travelled))

    def straightness(self, window_s: float = STRAIGHT_WINDOW_S) -> List[float]:
        """Net displacement / distance walked, per consecutive window. A window
        in which the figure walked under 1 m says nothing and is skipped."""
        out = []
        if len(self.track) < 2:
            return out
        ts = [r[0] for r in self.track]
        t0 = ts[0]
        while t0 + window_s <= ts[-1] + 1e-9:
            i = bisect.bisect_left(ts, t0 - 1e-9)
            j = bisect.bisect_left(ts, t0 + window_s - 1e-9)
            j = min(j, len(ts) - 1)
            a, b = self.track[i], self.track[j]
            walked = b[3] - a[3]
            if walked >= 100.0:
                out.append(math.hypot(b[1] - a[1], b[2] - a[2]) / walked)
            t0 += window_s
        return out


def merge_counters(cs: Sequence[PedCounters]) -> Dict[str, object]:
    st = [r for c in cs for r in c.straightness()]
    waits = sorted(w for c in cs for w in c.waits)

    def pct(q):
        return round(waits[min(len(waits) - 1, int(q * len(waits)))], 1) if waits else None

    return {
        "WalkViol": sum(c.walk_viol for c in cs),
        "GapViol": sum(c.gap_viol for c in cs),
        "ZebraIdleS": round(sum(c.zebra_idle_s for c in cs), 2),
        "CarOverlap": sum(c.car_overlap for c in cs),
        "CarOverlapS": round(sum(c.car_overlap_s for c in cs), 2),
        "KerbWaitMax": round(max((c.kerb_wait_max_s for c in cs), default=0.0), 1),
        "KerbWait_p50_p90": [pct(0.5), pct(0.9)],
        "Crossings": sum(c.crossings for c in cs),
        "Straightness": {"windows": len(st),
                         "mean": round(sum(st) / len(st), 3) if st else None,
                         "min": round(min(st), 3) if st else None},
    }


# ---------------------------------------------------------------- the model

WalkOk = Union[None, Callable[[float], bool], Mapping[object, Callable[[float], bool]]]


def walk_fn(walk_ok: WalkOk, xing: Crossing) -> Callable[[float], bool]:
    """Resolve the signal for one crossing. `walk_ok` is None (always walk), one
    callable walk_ok(t) for every crossing, or a mapping from crossing id - or
    the crossing's (x, y) centre - to such a callable. The signal module
    (tools/citylife_signals.py) is written separately; wrap it to fit."""
    if walk_ok is None:
        return lambda t: True
    if isinstance(walk_ok, Mapping):
        f = walk_ok.get(xing.id)
        if f is None:
            f = walk_ok.get((xing.x, xing.y))
        if f is None:
            raise KeyError(f"no signal for crossing {xing.id} at {(xing.x, xing.y)}")
        return f
    return walk_ok


def fixed_time_signals(crossings: Sequence[Crossing], cycle_s: float = 60.0,
                       walk_s: float = 20.0) -> Dict[int, Callable[[float], bool]]:
    """A stand-in until tools/citylife_signals.py exists: crossings walked along
    X get the first `walk_s` of each cycle, those walked along Y the same span
    half a cycle later, so the two directions at a junction never walk at once."""
    out = {}
    for c in crossings:
        off = 0.0 if c.axis == 0 else cycle_s / 2
        out[c.id] = (lambda t, off=off: ((t - off) % cycle_s) < walk_s)
    return out


def signal_walk_ok(crossings: Sequence[Crossing]) -> Dict[int, Callable[[float], bool]]:
    """walk_ok per crossing from tools/citylife_signals.py: its `walk(t, jx, jy,
    road_axis)` for the crossing's nearest junction, where `road_axis` is the
    direction the crossed road's traffic runs ('NS' along X, 'EW' along Y).
    A crossing walked along X (axis 0) cuts a road whose traffic runs along Y."""
    try:
        import citylife_signals as S
    except ImportError:
        from tools import citylife_signals as S
    out = {}
    for c in crossings:
        jx, jy = S.nearest_junction(c.x, c.y)
        road = S.crossing_road_axis(c.x, c.y, jx, jy)
        assert road == ("EW" if c.axis == 0 else "NS"), (c, road)
        out[c.id] = (lambda t, jx=jx, jy=jy, road=road: S.walk(t, jx, jy, road))
    return out


class PedModel:
    """WALK / WAIT / CROSS along one route of (x, y, kind) points.

    WALK   to the next point; a point is reached inside ARRIVE_CM. At a pavement
           point, pause 2-6 s with 10 % chance - always where the route turns
           round (a dead end), so a U-turn reads as having arrived somewhere.
    WAIT   at a kerb-wait point until the crossing's walk phase is on, no car
           will reach the zebra within GAP_S, and no moving car is on it.
    CROSS  to the crossing-exit point at 1.25 x walking speed (max 180 cm/s),
           never stopping, whatever a car does.

    Speed changes at the level's MaxAcceleration / BrakingDecelerationWalking,
    so pauses and starts take the stride or two they take in the engine."""

    def __init__(self, route: Sequence[Tuple[float, float, float]],
                 crossings: Sequence[Crossing], walk_ok: WalkOk = None,
                 speed_cms: float = 140.0, seed: int = 0, start: int = 0,
                 checks: bool = True):
        """`checks=False` crosses the moment the kerb is reached, as the old
        Roam did - kept so the counters can be shown to fire, not for use."""
        if len(route) < 2:
            raise ValueError("a route needs at least two points")
        self.checks = checks
        self.route = [(float(x), float(y), int(round(k))) for x, y, k in route]
        self.n = len(self.route)
        self.crossings = list(crossings)
        self.walk_speed = float(speed_cms)
        self.cross_speed = min(CROSS_SPEED_FACTOR * self.walk_speed, CROSS_SPEED_MAX_CMS)
        self.rng = random.Random(seed)
        self.counters = PedCounters()
        self.cross_log: List[Tuple[float, int, bool, float]] = []   # t, id, walk, min tta
        self._xing: Dict[int, Crossing] = {}
        self._walk: Dict[int, Callable[[float], bool]] = {}
        for i, (x, y, k) in enumerate(self.route):
            if k != KIND_KERB_WAIT:
                continue
            nx, ny, nk = self.route[(i + 1) % self.n]
            if nk != KIND_CROSS_EXIT:
                raise ValueError(f"route point {i} is kerb-wait but the next is not a crossing exit")
            xing = self._match(x, y, nx, ny)
            self._xing[i] = xing
            self._walk[xing.id] = walk_fn(walk_ok, xing)
        self.x, self.y = self.route[start][0], self.route[start][1]
        self.v = 0.0
        self.at = start                  # index of the last point reached
        self.target = (start + 1) % self.n
        self.pause_left = 0.0
        self.wait_s = 0.0
        self.state = WAIT if self.route[start][2] == KIND_KERB_WAIT else WALK
        if self.state == WAIT:
            self.target = start

    def _match(self, x, y, nx, ny) -> Crossing:
        for c in self.crossings:
            for a, b in ((c.kerb_a, c.kerb_b), (c.kerb_b, c.kerb_a)):
                if math.hypot(a[0] - x, a[1] - y) < 1.0 and math.hypot(b[0] - nx, b[1] - ny) < 1.0:
                    return c
        raise ValueError(f"no crossing joins ({x}, {y}) and ({nx}, {ny})")

    # -- decisions

    def clear_to_cross(self, t: float, xing: Crossing, cars: Sequence[CarView]
                       ) -> Tuple[bool, bool, float, bool]:
        """(go, walk phase on, smallest time to the zebra, moving car on it)."""
        walk = bool(self._walk[xing.id](t))
        near = [c for c in cars if abs(c.x - xing.x) < 6000.0 and abs(c.y - xing.y) < 6000.0]
        tta = min((time_to_zebra(c, xing) for c in near), default=math.inf)
        busy = any(c.v > MOVING_CAR_CMS and car_on_zebra(c, xing) for c in near)
        return walk and tta >= GAP_S and not busy, walk, tta, busy

    # -- stepping

    def step(self, t: float, dt: float, cars: Sequence[CarView] = ()) -> None:
        if self.state == WAIT:
            xing = self._xing[self.target]
            go, walk, tta, _ = self.clear_to_cross(t, xing, cars)
            if go or not self.checks:
                self.counters.on_wait(self.wait_s, done=True)
                self.counters.on_cross_start(walk, tta)
                self.cross_log.append((t, xing.id, walk, tta))
                self.state = CROSS
                self.at = self.target
                self.target = (self.target + 1) % self.n
            else:
                self.wait_s += dt
                self.counters.on_wait(self.wait_s)
        elif self.pause_left > 0.0:
            self.pause_left = max(0.0, self.pause_left - dt)

        stopping = self.state == WAIT or self.pause_left > 0.0
        if stopping:
            gx, gy = self.route[self.at if self.state != WAIT else self.target][:2]
            v_goal = 0.0
        else:
            gx, gy = self.route[self.target][:2]
            v_goal = self.cross_speed if self.state == CROSS else self.walk_speed
        self.v += max(-PED_DECEL_CMS2 * dt, min(PED_ACCEL_CMS2 * dt, v_goal - self.v))
        dx, dy = gx - self.x, gy - self.y
        d = math.hypot(dx, dy)
        move = min(self.v * dt, d)
        if d > 0:
            self.x += dx / d * move
            self.y += dy / d * move
        if not stopping and math.hypot(gx - self.x, gy - self.y) <= ARRIVE_CM:
            self._arrive()
        self.counters.observe(t, dt, self.x, self.y, move / dt if dt > 0 else 0.0,
                              cars, self.crossings)

    def _arrive(self) -> None:
        i = self.target
        kind = self.route[i][2]
        if kind == KIND_KERB_WAIT:
            self.state = WAIT
            self.wait_s = 0.0
            self.at = i
            return                       # target stays the kerb until CROSS starts
        self.state = WALK
        self.at = i
        self.target = (i + 1) % self.n
        if kind == KIND_PAVEMENT:
            prv, nxt = self.route[i - 1][:2], self.route[self.target][:2]
            u_turn = math.hypot(prv[0] - nxt[0], prv[1] - nxt[1]) < 1.0
            if u_turn or self.rng.random() < PAUSE_P:
                self.pause_left = self.rng.uniform(*PAUSE_S)


# ---------------------------------------------------------------- synthetic traffic

class SyntheticTraffic:
    """Cars on the level's own loop paths, moving by arc length, doing what
    BP_CityCar does about pedestrians and nothing more: stop 6.5 m (centre to
    crossing) short of a crossing on their path, within 25 m, while a figure is
    on its zebra; keep 6.5 m behind the car ahead; brake at most 4 m/s^2. No
    signals - the level's cars have none - so the gap rule is all that keeps a
    figure out of a car's way."""

    STOP_CM = 650.0
    SCAN_CM = 2500.0

    def __init__(self, paths: Dict[str, object] = None, counts: Dict[str, int] = None,
                 seed: int = SEED, speed_range: Tuple[float, float] = (390.0, 580.0),
                 subject_speed: float = 320.0):
        rng = random.Random(seed + 1)
        self.paths = paths if paths is not None else R.build_all()
        counts = counts if counts is not None else {"A": 13, "B": 5}
        self.cars = []
        for loop in sorted(counts):
            p = self.paths[loop]
            _, total = path_arc(p)
            for k in range(counts[loop]):
                cruise = rng.uniform(*speed_range)
                if loop == "A" and k == 0:
                    cruise = subject_speed           # Car_10, the red subject car
                self.cars.append({"loop": loop, "path": p, "s": total * k / counts[loop],
                                  "v": cruise, "cruise": cruise})
        self.xs_on = {}
        for loop, p in self.paths.items():
            self.xs_on[loop] = [(x, y) for x, y, _ in R.crossings_on(p)]

    def views(self) -> List[CarView]:
        out = []
        for c in self.cars:
            x, y, yaw = path_point(c["path"], c["s"])
            out.append(CarView(x, y, yaw, c["v"], c["path"], c["s"]))
        return out

    def step(self, dt: float, xings: Sequence[Crossing], peds: Sequence[Vec]) -> None:
        by_loop: Dict[str, list] = {}
        for c in self.cars:
            by_loop.setdefault(c["loop"], []).append(c)
        busy = {x.id: any(x.in_zebra(px, py, pad=100.0) for px, py in peds) for x in xings}
        for loop, cs in by_loop.items():
            p = cs[0]["path"]
            cum, total = path_arc(p)
            cs.sort(key=lambda c: c["s"] % total)
            for k, c in enumerate(cs):
                ahead = cs[(k + 1) % len(cs)]
                gap = (ahead["s"] - c["s"]) % total if len(cs) > 1 else math.inf
                target = min(c["cruise"], R.V_STRAIGHT_CMS)
                i = max(0, bisect.bisect_right(cum, c["s"] % total) - 1)
                target = min(target, p.speed[i], p.speed[(i + 2) % len(p.speed)])
                target = min(target, math.sqrt(2 * R.STOP_DECEL_CMS2 * max(0.0, gap - self.STOP_CM)))
                for x in xings:
                    if not busy[x.id] or (x.x, x.y) not in self.xs_on[loop]:
                        continue
                    s_x = crossing_s(p, x)
                    d = (s_x - c["s"]) % total
                    if d <= self.SCAN_CM:
                        target = min(target, math.sqrt(2 * R.STOP_DECEL_CMS2 * max(0.0, d - self.STOP_CM)))
                c["v"] += max(-R.DECEL_CMS2 * dt, min(R.ACCEL_CMS2 * dt, target - c["v"]))
                c["v"] = max(0.0, c["v"])
            for c in cs:
                c["s"] = (c["s"] + c["v"] * dt) % total


def simulate(seconds: float = 180.0, dt: float = 0.1, n_peds: int = 40, legs: int = 40,
             seed: int = SEED, walk_ok: object = "fixed", graph: PedGraph = None,
             counts: Dict[str, int] = None, checks: bool = True) -> Dict[str, object]:
    """Every route walked by a PedModel among a synthetic car stream.
    `walk_ok="fixed"` uses fixed_time_signals, "signals" the level's plan in
    tools/citylife_signals.py; None means always walk. The synthetic cars
    ignore signals either way, so kerb waits here are an upper bound.
    `checks=False` runs the same figures crossing blind, for contrast."""
    g = graph if graph is not None else build_graph()
    routes = make_routes(n_peds, legs, seed, graph=g)
    if isinstance(walk_ok, str) and walk_ok == "fixed":
        walk_ok = fixed_time_signals(g.crossings)
    elif isinstance(walk_ok, str) and walk_ok == "signals":
        walk_ok = signal_walk_ok(g.crossings)
    rng = random.Random(seed + 2)
    peds = [PedModel(r, g.crossings, walk_ok, rng.uniform(*WALK_SPEED_CMS), seed=seed + 10 + k,
                     checks=checks)
            for k, r in enumerate(routes)]
    traffic = SyntheticTraffic(counts=counts, seed=seed)
    t = 0.0
    states: Dict[str, int] = {WALK: 0, WAIT: 0, CROSS: 0}
    for _ in range(int(round(seconds / dt))):
        t += dt
        cars = traffic.views()
        for p in peds:
            p.step(t, dt, cars)
            states[p.state] += 1
        traffic.step(dt, g.crossings, [(p.x, p.y) for p in peds])
    ticks = max(1, int(round(seconds / dt)) * len(peds))
    used = sorted({xid for p in peds for _, xid, _, _ in p.cross_log})
    net = [p for p in peds if p._xing]
    isl = [p for p in peds if not p._xing]

    def straight(ps):
        st = [r for p in ps for r in p.counters.straightness()]
        return {"peds": len(ps), "windows": len(st),
                "mean": round(sum(st) / len(st), 3) if st else None}

    return {"seconds": seconds, "peds": len(peds), "cars": len(traffic.cars),
            "checks": checks,
            "counters": merge_counters([p.counters for p in peds]),
            "straightness_by_route": {"with_crossings": straight(net),
                                      "islands": straight(isl)},
            "state_share": {k: round(v / ticks, 3) for k, v in states.items()},
            "crossings_walked": used}


def route_stats(g: PedGraph, tours: Sequence[Sequence[int]]) -> Dict[str, object]:
    used: Dict[int, int] = {}
    uturns = 0
    for tr in tours:
        n = len(tr)
        for i in range(n):
            a, b = tr[i], tr[(i + 1) % n]
            if g.is_crossing_edge(a, b):
                used[g.xing_at[a]] = used.get(g.xing_at[a], 0) + 1
            if tr[i - 1] == b:
                uturns += 1
    lens = [len(t) for t in tours] or [0]
    per_comp: Dict[int, int] = {}
    for tr in tours:
        per_comp[g.comp[tr[0]]] = per_comp.get(g.comp[tr[0]], 0) + 1
    return {"routes": len(tours), "legs_min": min(lens), "legs_max": max(lens),
            "legs_mean": round(sum(lens) / len(lens), 1),
            "crossing_legs_by_id": dict(sorted(used.items())),
            "dead_end_uturns": uturns, "routes_per_component": dict(sorted(per_comp.items()))}


if __name__ == "__main__":
    # python tools/citylife_peds.py [seconds] [--blind] [--json out.json]
    import json
    args = sys.argv[1:]
    g = build_graph()
    print(json.dumps(summary(g), indent=1, default=str))
    tours = plan_tours(g)
    print(json.dumps(route_stats(g, tours), indent=1))
    nums = [a for a in args if a.replace(".", "", 1).isdigit()]
    secs = float(nums[0]) if nums else 180.0
    print(json.dumps(simulate(secs, graph=g), indent=1))
    if "--blind" in args:
        print(json.dumps(simulate(secs, graph=g, checks=False), indent=1))
    if "--json" in args:
        out = Path(args[args.index("--json") + 1])
        with open(out, "w", newline="\n", encoding="utf-8") as f:
            json.dump(plan(graph=g), f, indent=1)
        print(f"wrote {out}")
