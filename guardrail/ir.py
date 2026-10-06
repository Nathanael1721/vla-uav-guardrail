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

NOT HERE YET (honest list, cards WP1-03 / WP2-16)

  * Multi-scale geometry (a coarse polygon beside the fine one, and a
    `geometry_ref` to it in the CSP). The reference IR has none either.
  * Corridors, stand-off rings and the obstacle distance field are not indexed:
    a corridor is a keep-IN rule (every pose must be checked against it
    anyway), a stand-off moves with its subject every tick, and the distance
    field is already a constant-time grid lookup.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from shapely.geometry import Polygon, box

from .geometry import fence_polygon
from .models import Policy, PolygonFence

try:                                    # shapely >= 2.0: query() returns indices
    from shapely import STRtree as _STRtree
except ImportError:                     # pragma: no cover - shapely 1.x
    _STRtree = None


@dataclass(frozen=True)
class FenceRecord:
    """One compiled no-fly zone.

    `polygon` is the authored shape and `buffered` the shape the rule actually
    forbids (authored plus `margin_m`). Both are kept because they answer
    different questions: "am I inside?" is asked of `buffered`, while the
    push-out direction is measured to the AUTHORED boundary, as it always has
    been - see geometry.push_out_direction.
    """
    index: int                                  # order among the policy's fences
    rule: PolygonFence
    polygon: Polygon
    buffered: Polygon
    bounds: tuple[float, float, float, float]   # envelope of `buffered`


def compile_fence(index: int, fence: PolygonFence) -> FenceRecord:
    poly = fence_polygon(fence)
    # The same call geometry.point_in_fence makes, made once. buffer() is a
    # pure function of (polygon, distance), so the cached ring is the ring the
    # old per-test rebuild produced, coordinate for coordinate.
    buffered = poly.buffer(fence.margin_m)
    return FenceRecord(index=index, rule=fence, polygon=poly, buffered=buffered,
                       bounds=tuple(buffered.bounds))


class PolicyIR:
    """Indexed, compiled form of a policy's fences. Immutable once built:
    `with_fence` returns a new IR, so a reader holding the old one mid-tick
    never sees half an update (hot_apply runs on the REST thread, filter() on
    the control loop - guardrail/api.py)."""

    def __init__(self, records: list[FenceRecord], policy_id: str = "",
                 generation: int = 0, policy_hash: str = ""):
        self.fences: list[FenceRecord] = list(records)
        self.policy_id = policy_id
        self.generation = generation
        # The reproducibility key of the policy this IR was compiled FROM, taken
        # at compile time. The reference IR carries it for the same reason: an
        # index that silently outlived a rule change would answer for rules that
        # are no longer the ones on record.
        self.policy_hash = policy_hash
        self._tree = (_STRtree([r.buffered for r in self.fences])
                      if _STRtree is not None and self.fences else None)

    # ------------------------------------------------------------ building

    @classmethod
    def from_policy(cls, policy: Policy) -> "PolicyIR":
        return cls([compile_fence(i, f)
                    for i, f in enumerate(policy.by_type(PolygonFence))],
                   policy_id=policy.policy_id, generation=policy.generation,
                   policy_hash=policy.policy_hash)

    @classmethod
    def from_fences(cls, fences: Iterable[PolygonFence]) -> "PolicyIR":
        return cls([compile_fence(i, f) for i, f in enumerate(fences)])

    def with_fence(self, fence: PolygonFence, policy: Policy | None = None
                   ) -> "PolicyIR":
        """The hot-apply path: the existing records are reused (no fence is
        re-buffered), the new one is compiled and the tree rebuilt - shapely's
        STRtree is immutable, and rebuilding 50 envelopes costs microseconds
        against the grant's 50 ms hot-apply budget."""
        recs = self.fences + [compile_fence(len(self.fences), fence)]
        if policy is None:
            return PolicyIR(recs, self.policy_id, self.generation, self.policy_hash)
        return PolicyIR(recs, policy.policy_id, policy.generation, policy.policy_hash)

    def is_current_for(self, policy: Policy) -> bool:
        """Was this IR compiled from `policy` as it stands now?"""
        return (self.generation == policy.generation
                and len(self.fences) == len(policy.by_type(PolygonFence))
                and self.policy_hash == policy.policy_hash)

    # ------------------------------------------------------------ queries

    @property
    def indexed(self) -> bool:
        """True when queries go through the STRtree, False when they fall
        back to the brute-force envelope test (no fences, or old shapely)."""
        return self._tree is not None

    def fences_in_box(self, minx: float, miny: float, maxx: float, maxy: float
                      ) -> list[FenceRecord]:
        """Every fence whose buffered polygon's envelope meets the box,
        boundaries included, in POLICY order.

        Policy order is load-bearing: the Shield lists violations, and applies
        repairs, fence by fence in that order (overlapping fences repair against
        what the previous one left behind), and the STRtree returns indices in
        tree order. Sorting restores it.
        """
        if not all(math.isfinite(v) for v in (minx, miny, maxx, maxy)):
            return list(self.fences)          # see module docstring
        if self._tree is not None:
            idx = self._tree.query(box(minx, miny, maxx, maxy))
            return [self.fences[i] for i in sorted(int(i) for i in idx)]
        # Written as four positive tests so an EMPTY buffered polygon (a negative
        # margin can erase a small fence; its bounds are all NaN) is excluded,
        # as the STRtree excludes it. Either way it could contain nothing.
        return [r for r in self.fences
                if r.bounds[0] <= maxx and r.bounds[2] >= minx
                and r.bounds[1] <= maxy and r.bounds[3] >= miny]

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
