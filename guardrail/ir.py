"""
Compiled policy IR: the fence geometry the Shield queries, built once.

The grant's Policy DSL page describes ingest as "build runtime IR: R-tree / grid
spatial index / multi-scale polygons / bounding boxes", and the Safety Shield
page says "geometry queries hit the IR's R-tree spatial index. With ~50 active
rules and 50 future-pose checks per tick, total query budget is <= 5 ms". The
mid-term delivery (2026-07-20) was "Policy DSL + IR". Until this file, the IR in
this repository was the Pydantic `Policy` alone, and the Shield answered every
geometry question by walking every fence:

    for f, poly in self._fences:            # all 48 of them, every check
        ... poly.buffer(f.margin_m).contains(Point(x, y))   # per pose, per fence

At the grant's 50-rule load one `_check` took 14.3 ms in clear sky (audit,
2026-10-05) to 33 ms (experiments/bench_shield_50rules.py on 2026-10-06, on a
desktop shared with other jobs), against 5 ms; one `filter()` near a fence took
a median of 170 ms (p99 255 ms in the 2026-10-06 run), longer than the 100 ms
tick it has to fit inside. Two costs, two fixes, both here:

  1. The margin ring was rebuilt on EVERY point test (`geometry.py`'s
     `poly.buffer(margin)`, 30-45 us each), against the "pre-build once" comment
     sitting right above the fence list in shield.py. Each fence is now buffered
     once, at compile time, and kept.
  2. Every fence was tested whether or not the forecast could reach it. A shapely
     STRtree over the buffered polygons now returns only the fences whose
     bounding box meets the forecast's bounding box.

WHICH RULES ARE FENCES

Every keep-out polygon: `polygon_fence`, `circle_fence` (a PolygonFence whose
32-gon is derived from centre and radius, models.CircleFence) and `dynamic_nfz`
(the grant's hot-applicable "polygon with motion"). A dynamic zone that MOVES is
compiled at its anchor position and flagged `moving`: the STRtree cannot know
where it will be, so a moving record is left out of the tree and returned by
every query (the Shield materialises its current polygon once per tick and
transforms each forecast pose into the zone's frame; guardrail/shield.py).

WHERE THE LOOKAHEAD TRAVEL GOES

The reference (kuanting-vla-uav-guardrail/.../policy_dsl/ir.py, `polygons_near`)
queries with a disc around the vehicle whose radius is the forecast's reach
(safety_shield/checker.py: the distance to the last forecast pose, plus 1 m). That
is the same idea with the travel carried by the QUERY rather than by the tree,
and this file keeps it on the query side on purpose: the Shield's job is to catch
the raw action, and a raw action has no speed bound until the speed clamp has
run. A tree whose fences were padded for "the cap times the horizon" would miss
the fence an 8 m/s raw command is about to enter - exactly the action the monitor
exists for. The query box is the bounding box of the actual forecast poses, so it
is exact at any speed. `polygons_near(x, y, radius_m)` is kept with the
reference's meaning for callers that think in reach.

EXACTNESS, NOT APPROXIMATION

The index only decides which fences are LOOKED AT. A fence it drops cannot
contain any pose of the forecast: every pose lies inside the query box, and a
point strictly inside a polygon lies inside that polygon's envelope, so the two
envelopes intersect and the fence is returned. Everything returned is then judged
by the same `contains` the old loop used, on a buffered polygon built by the
same `buffer` call. So the Shield's answers do not change - and
tests/test_ir.py checks that claim against a verbatim copy of the old loop
(tests/legacy_fence_loop.py) over every policy in policies/ rather than
trusting this paragraph.

Two inputs fall back to "every fence", because a box cannot be trusted there:
a non-finite coordinate (NaN compares false against every envelope, so a NaN box
would silently drop the fence the vehicle is already inside), and a shapely too
old to have the index-returning STRtree (a brute-force envelope test is used
instead, which is the same filter, only slower).

MULTI-SCALE GEOMETRY (card WP2-16, the IR half)

Each zone carries three scales, computed once at compile time:

  fine    the vertices as authored (the Shield adds `margin_m` itself);
  coarse  a polygon that CONTAINS the fine one: its convex hull or, when the
          hull has more than COARSE_MAX_VERTICES vertices, the minimum-area
          oriented rectangle around it - the rule guardrail/compiler.py's
          `multiscale_geometry` uses, so the two agree (tests/test_ir.py checks
          it on every shipped policy). Containment is the point: a coarse
          keep-out zone that cut a corner would describe a smaller zone than
          the Shield enforces;
  bbox    the axis-aligned bounding rectangle of the fine polygon.

Corridors carry "fine" only (their centerline): coarsening a keep-IN shape
outward would widen where a reader believes it may fly. `geometry_ref(scale,
rule_id)` names one scale of one rule of THIS generation, in the compiler's
format ("coarse:<id>@v<version>/g<generation>"), and `resolve_geometry_ref`
refuses a reference minted for another version or generation.

NOT HERE YET (honest list)

  * Stand-off rings and the obstacle distance field are not indexed: a stand-off
    moves with its subject every tick, and the distance field is already a
    constant-time grid lookup. Corridors are listed (multi-scale "fine") but not
    indexed: a keep-IN rule must be checked at every pose anyway.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Iterable

import shapely
from shapely.geometry import Polygon, box

from .geometry import fence_polygon
from .models import Policy, PolygonFence

try:                                    # shapely >= 2.0: query() returns indices
    from shapely import STRtree as _STRtree
except ImportError:                     # pragma: no cover - shapely 1.x
    _STRtree = None

# Rule types compiled as keep-out zones.
FENCE_TYPES = ("polygon_fence", "circle_fence", "dynamic_nfz")
# Rule types listed with a "fine" scale only (keep-IN tubes).
TUBE_TYPES = ("corridor", "corridor_swap")
# Same limit as guardrail/compiler.py COARSE_MAX_VERTICES.
COARSE_MAX_VERTICES = 8
SCALES = ("fine", "coarse", "bbox")

Pt = tuple[float, float]


def is_fence(rule) -> bool:
    return getattr(rule, "type", None) in FENCE_TYPES


def is_moving(rule) -> bool:
    """A dynamic_nfz whose `motion` is not zero on every channel."""
    m = getattr(rule, "motion", None)
    return m is not None and any(abs(v) > 0.0 for v in
                                 (m.vx_mps, m.vy_mps, m.yaw_rate_dps))


def coarse_ring(points) -> list[Pt]:
    """A polygon containing `points`' polygon (or a shapely Polygon) with at
    most COARSE_MAX_VERTICES vertices: the convex hull, else the minimum-area
    oriented rectangle."""
    shape = points if isinstance(points, Polygon) else Polygon(list(points))
    hull = shape.convex_hull
    if hull.geom_type != "Polygon":              # degenerate input: keep its box
        hull = box(*hull.bounds)
    ring = list(hull.exterior.coords)[:-1]
    if len(ring) > COARSE_MAX_VERTICES:
        env = (shapely.oriented_envelope(hull) if hasattr(shapely, "oriented_envelope")
               else hull.minimum_rotated_rectangle)
        ring = list(env.exterior.coords)[:-1]
    return [(float(x), float(y)) for x, y in ring]


@dataclass(frozen=True)
class FenceRecord:
    """One compiled no-fly zone.

    `polygon` is the authored shape and `buffered` the shape the rule actually
    forbids (authored plus `margin_m`). Both are kept because they answer
    different questions: "am I inside?" is asked of `buffered`, while the
    push-out direction is measured to the AUTHORED boundary, as it always has
    been - see geometry.push_out_direction.

    `coarse` is the multi-scale coarse ring (see the module docstring); empty
    for a record materialised per tick. `moving` marks a dynamic_nfz with
    motion: such a record is never left out by a box query.
    """
    index: int                                  # order among the policy's fences
    rule: PolygonFence
    polygon: Polygon
    buffered: Polygon
    bounds: tuple[float, float, float, float]   # envelope of `buffered`
    coarse: tuple = ()
    moving: bool = False


def compile_fence(index: int, fence, ring: list[Pt] | None = None,
                  multiscale: bool = True) -> FenceRecord:
    """Compile one zone. `ring` overrides the authored vertices (a moving
    dynamic_nfz at its current position); `multiscale=False` skips the coarse
    ring for such a per-tick record."""
    poly = fence_polygon(fence) if ring is None else Polygon(ring)
    # The same call geometry.point_in_fence makes, made once. buffer() is a
    # pure function of (polygon, distance), so the cached ring is the ring the
    # old per-test rebuild produced, coordinate for coordinate.
    buffered = poly.buffer(fence.margin_m)
    return FenceRecord(index=index, rule=fence, polygon=poly, buffered=buffered,
                       bounds=tuple(buffered.bounds),
                       coarse=tuple(coarse_ring(poly)) if multiscale else (),
                       moving=is_moving(fence))


class PolicyIR:
    """Indexed, compiled form of a policy's fences. Immutable once built:
    `with_fence` and `from_policy(..., reuse=...)` return a new IR, so a reader
    holding the old one mid-tick never sees half an update (events arrive on
    the REST thread, filter() runs on the control loop - guardrail/api.py)."""

    def __init__(self, records: list[FenceRecord], policy_id: str = "",
                 generation: int = 0, policy_hash: str = "", version: str = "",
                 tubes: Iterable = ()):
        self.fences: list[FenceRecord] = list(records)
        self.policy_id = policy_id
        self.generation = generation
        self.version = version
        # The reproducibility key of the policy this IR was compiled FROM, taken
        # at compile time. The reference IR carries it for the same reason: an
        # index that silently outlived a rule change would answer for rules that
        # are no longer the ones on record.
        self.policy_hash = policy_hash
        self._static = [r for r in self.fences if not r.moving]
        self._moving = [r for r in self.fences if r.moving]
        self._tree = (_STRtree([r.buffered for r in self._static])
                      if _STRtree is not None and self._static else None)
        self.tubes = tuple(tubes)
        self.multiscale: dict[str, dict[str, list[Pt]]] = {}
        for r in self.fences:
            fine = [(float(v.x), float(v.y)) for v in r.rule.vertices]
            minx, miny, maxx, maxy = r.polygon.bounds if not r.polygon.is_empty \
                else (math.nan,) * 4
            self.multiscale[r.rule.id] = {
                "fine": fine,
                "coarse": list(r.coarse) if r.coarse else coarse_ring(fine),
                "bbox": [(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)]}
        for c in self.tubes:
            self.multiscale[c.id] = {"fine": [(float(x), float(y)) for x, y in c.points()]}

    # ------------------------------------------------------------ building

    @classmethod
    def from_policy(cls, policy: Policy, reuse: "PolicyIR | None" = None) -> "PolicyIR":
        """Compile every zone of `policy`. With `reuse`, a record whose rule
        object is unchanged is kept (re-indexed if a zone before it was
        removed) instead of re-buffered: the grant's hot path "re-derives the
        affected spatial-index entries", not the whole bundle."""
        old = {id(r.rule): r for r in reuse.fences} if reuse is not None else {}
        recs = []
        for i, f in enumerate(c for c in policy.constraints if is_fence(c)):
            r = old.get(id(f))
            if r is not None and r.rule is f:
                recs.append(r if r.index == i else replace(r, index=i))
            else:
                recs.append(compile_fence(i, f))
        return cls(recs, policy_id=policy.policy_id, generation=policy.generation,
                   policy_hash=policy.policy_hash, version=policy.version,
                   tubes=[c for c in policy.constraints
                          if getattr(c, "type", None) in TUBE_TYPES])

    @classmethod
    def from_fences(cls, fences: Iterable[PolygonFence]) -> "PolicyIR":
        return cls([compile_fence(i, f) for i, f in enumerate(fences)])

    def with_fence(self, fence: PolygonFence, policy: Policy | None = None
                   ) -> "PolicyIR":
        """Append one zone: the existing records are reused (no fence is
        re-buffered), the new one is compiled and the tree rebuilt - shapely's
        STRtree is immutable, and rebuilding 50 envelopes costs microseconds
        against the grant's 50 ms hot-apply budget."""
        recs = self.fences + [compile_fence(len(self.fences), fence)]
        if policy is None:
            return PolicyIR(recs, self.policy_id, self.generation, self.policy_hash,
                            self.version, self.tubes)
        return PolicyIR(recs, policy.policy_id, policy.generation, policy.policy_hash,
                        policy.version, [c for c in policy.constraints
                                         if getattr(c, "type", None) in TUBE_TYPES])

    def is_current_for(self, policy: Policy) -> bool:
        """Was this IR compiled from `policy` as it stands now?"""
        return (self.generation == policy.generation
                and len(self.fences) == sum(1 for c in policy.constraints if is_fence(c))
                and self.policy_hash == policy.policy_hash)

    # ------------------------------------------------------------ queries

    @property
    def indexed(self) -> bool:
        """True when queries go through the STRtree, False when they fall
        back to the brute-force envelope test (no static fences, or old
        shapely)."""
        return self._tree is not None

    def fences_in_box(self, minx: float, miny: float, maxx: float, maxy: float
                      ) -> list[FenceRecord]:
        """Every fence whose buffered polygon's envelope meets the box,
        boundaries included, plus every MOVING zone, in POLICY order.

        Policy order is load-bearing: the Shield lists violations, and applies
        repairs, fence by fence in that order (overlapping fences repair against
        what the previous one left behind), and the STRtree returns indices in
        tree order. Sorting restores it.
        """
        if not all(math.isfinite(v) for v in (minx, miny, maxx, maxy)):
            return list(self.fences)          # see module docstring
        if self._tree is not None:
            idx = self._tree.query(box(minx, miny, maxx, maxy))
            hit = [self._static[int(i)] for i in idx]
        else:
            # Written as four positive tests so an EMPTY buffered polygon (a
            # negative margin can erase a small fence; its bounds are all NaN)
            # is excluded, as the STRtree excludes it. Either way it could
            # contain nothing.
            hit = [r for r in self._static
                   if r.bounds[0] <= maxx and r.bounds[2] >= minx
                   and r.bounds[1] <= maxy and r.bounds[3] >= miny]
        if self._moving:
            hit = hit + self._moving
        return sorted(hit, key=lambda r: r.index)

    def fences_for_points(self, xs: Iterable[float], ys: Iterable[float]
                          ) -> list[FenceRecord]:
        """Fences that could contain any of the points. Non-finite input means
        every fence - a NaN would otherwise make min() order-dependent and the
        box arbitrary."""
        xs, ys = list(xs), list(ys)
        if not xs or not all(math.isfinite(v) for v in xs + ys):
            return list(self.fences)
        return self.fences_in_box(min(xs), min(ys), max(xs), max(ys))

    def polygons_near(self, x: float, y: float, radius_m: float
                      ) -> list[FenceRecord]:
        """The reference IR's query, in local metres: fences whose envelope is
        within `radius_m` of (x, y) along each axis. A caller using it for a
        forecast must pass the full reach (speed x horizon + 1 m, as the
        reference checker does) - see 'Where the lookahead travel goes'."""
        r = abs(radius_m)
        return self.fences_in_box(x - r, y - r, x + r, y + r)

    # ------------------------------------------------------------ multi-scale

    def geometry_ref(self, scale: str, rule_id: str) -> str:
        """The name of one scale of one rule in THIS generation, in the
        compiler's format. Refuses a scale the rule does not carry."""
        geo = self.multiscale.get(rule_id)
        if geo is None or scale not in geo:
            raise ValueError(f"no {scale} geometry for {rule_id!r} in {self.policy_id}")
        return f"{scale}:{rule_id}@v{self.version}/g{self.generation}"

    def resolve_geometry_ref(self, ref: str) -> list[Pt]:
        """The vertices a `geometry_ref` names. A reference minted for another
        version or generation is refused: after a hot-apply bumps the
        generation, a CSP from before must not quietly resolve against geometry
        it never described."""
        scale, _, rest = ref.partition(":")
        rid, at, tail = rest.rpartition("@v")
        ver, slash, gen = tail.partition("/g")
        if not (scale and at and slash and gen.isdigit()):
            raise ValueError(f"not a geometry ref: {ref!r}")
        if ver != self.version or int(gen) != self.generation:
            raise ValueError(f"{ref!r} is for v{ver}/g{gen}; this IR is "
                             f"v{self.version}/g{self.generation}")
        geo = self.multiscale.get(rid)
        if geo is None or scale not in geo:
            raise ValueError(f"no {scale} geometry for {rid!r} in {self.policy_id}")
        return list(geo[scale])
