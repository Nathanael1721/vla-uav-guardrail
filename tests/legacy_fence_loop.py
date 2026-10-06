"""The Shield's fence handling as it stood BEFORE guardrail/ir.py (2026-10-06).

Not a test module (no `test_` prefix, so neither runner collects it). It is the
"old code" that tests/test_ir.py and experiments/bench_shield_50rules.py compare
the indexed Shield against.

Why a frozen copy rather than "the same Shield with the index switched off": an
equivalence test is only worth something if the reference side cannot drift
with the code under test. A switch inside shield.py would share every helper
the refactor touched - the cached ring, the shared forecast, the new
`_check_fence` signature - so a bug in any of them would sit on BOTH sides of
the comparison and cancel. Here the three methods that walked the fences are
copied from guardrail/shield.py at commit 1d09786 - code verbatim, the long
explanatory comments trimmed - (the uncommitted
diff on top of it then was 17 lines of read-only HUD accessors that touch no
fence), together with the `point_in_fence` they called, which rebuilt the
margin ring on every test. Everything else is inherited, because nothing else
in the Shield was changed by the IR work; tests/test_check_contract.py pins
`_check` independently of this file.

If the Shield's fence BEHAVIOUR is ever changed on purpose, this file is the
record of what it used to be: update it in the same commit and say so, rather
than let the equivalence test go quietly stale.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shapely.geometry import Point, Polygon                         # noqa: E402

from guardrail.geometry import fence_polygon, push_out_direction    # noqa: E402
from guardrail.models import Action4D, PolygonFence, State          # noqa: E402
from guardrail.shield import Repair, Shield, Violation              # noqa: E402


def legacy_point_in_fence(x: float, y: float, up: float, fence: PolygonFence,
                          poly: Polygon | None = None) -> bool:
    """guardrail/geometry.py point_in_fence at 1d09786, verbatim."""
    if not (fence.altitude_floor_m <= up <= fence.altitude_ceiling_m):
        return False
    poly = poly if poly is not None else fence_polygon(fence)
    return poly.buffer(fence.margin_m).contains(Point(x, y))


point_in_fence = legacy_point_in_fence


class LegacyFenceShield(Shield):
    """Shield whose fence loops are the pre-IR ones: every fence, every pose,
    a fresh buffer() per point test."""

    def __init__(self, policy, *args, **kwargs):
        super().__init__(policy, *args, **kwargs)
        # Pre-build shapely polygons once (grant: pre-compute at ingest,
        # never rebuild per tick).
        self._legacy_all_fences = [(f, fence_polygon(f))
                                   for f in policy.by_type(PolygonFence)]

    @property
    def _fences(self) -> list:
        """Same gating, but the entries are (rule, prebuilt polygon) pairs."""
        if self._now is None:
            return self._legacy_all_fences
        when = self._now()
        return [(f, p) for f, p in self._legacy_all_fences if f.active_at(when)]

    def hot_apply(self, fence: PolygonFence) -> None:
        super().hot_apply(fence)
        self._legacy_all_fences.append((fence, fence_polygon(fence)))

    # ---- code verbatim from shield.py@1d09786 below (comments trimmed),
    # ---- renamed only where the new Shield reuses a name with a different
    # ---- signature.

    def _legacy_check_fence(self, f, poly, state: State, action: Action4D) -> list[Violation]:
        """Trend-aware for the already-inside case."""
        if point_in_fence(state.x, state.y, state.up, f, poly):
            ox, oy = push_out_direction(state.x, state.y, poly)
            escaping = (action.vx * ox + action.vy * oy) > 0.1
            if not escaping:
                return [Violation(
                    rule_id=f.id, category="geofence", predicted_at_s=0.0,
                    detail="currently INSIDE zone and not escaping")]
            return []      # while inside, predictive entry checks are moot
        for t, p in self._predict(state, action):
            if point_in_fence(p.x, p.y, p.up, f, poly):
                return [Violation(
                    rule_id=f.id, category="geofence", predicted_at_s=t,
                    detail=f"predicted pos ({p.x:.1f},{p.y:.1f}) inside NFZ at t+{t:.1f}s")]
        return []

    def _check(self, state: State, action: Action4D) -> list[Violation]:
        found: list[Violation] = []
        for k in self._kins:
            found += self._check_kinematic(k, action)
        for env in self._alts:
            found += self._check_altitude(env, state, action)
        if self._subject is not None:
            for so in self._standoffs:
                found += self._check_standoff(so, state, action)
        for f, poly in self._fences:
            found += self._legacy_check_fence(f, poly, state, action)
        for c in self._corridors:
            found += self._check_corridor(c, state, action)
        if self._dist is not None and self._clear:
            # One forecast shared across every clearance rule, as before.
            steps, dists = self._clear_forecast(state, action)
            for c in self._clear:
                found += self._check_clearance(c, state, action, steps, dists)

        # dedupe by (rule, category), keep earliest
        seen: dict[tuple, Violation] = {}
        for v in found:
            key = (v.rule_id, v.category)
            if key not in seen or v.predicted_at_s < seen[key].predicted_at_s:
                seen[key] = v
        return list(seen.values())

    def _repair_geofence(self, state: State, a: Action4D, repairs: list[Repair]) -> Action4D:
        vx, vy = a.vx, a.vy
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)

        for f, poly in self._fences:
            if point_in_fence(state.x, state.y, state.up, f, poly):
                ox, oy = push_out_direction(state.x, state.y, poly)
                spd = min(2.0, cap)
                out_now = vx * ox + vy * oy          # outward speed commanded
                probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                if not self._legacy_check_fence(f, poly, state, probe):
                    if out_now < spd:
                        vx += ox * (spd - out_now)
                        vy += oy * (spd - out_now)
                        repairs.append(Repair(
                            operator="GeofenceEscape",
                            detail=(f"{f.id}: inside and leaving at "
                                    f"{out_now:.2f} m/s -> raised to {spd:.1f}")))
                    continue
                keep = min(cap, max(out_now, spd))
                vx, vy = ox * keep, oy * keep
                repairs.append(Repair(operator="GeofenceEscape",
                                      detail=(f"{f.id}: inside -> exit at {keep:.1f} m/s "
                                              f"(commanded {out_now:+.2f})")))
                continue

            probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
            hit = any(point_in_fence(p.x, p.y, p.up, f, poly)
                      for _, p in self._predict(state, probe))
            if not hit:
                continue

            ox, oy = push_out_direction(state.x, state.y, poly)   # unit "away from zone"
            into = -(vx * ox + vy * oy)                           # speed INTO the zone
            if into > 0:
                vx += ox * into                                   # cancel it
                vy += oy * into
                h = (vx ** 2 + vy ** 2) ** 0.5
                if h < 0.5:
                    tx, ty = -oy, ox                              # tangent to the edge
                    if a.vx * tx + a.vy * ty < 0:                 # keep the raw action's turn side
                        tx, ty = -tx, -ty
                    spd = min(max(into, 1.0), cap)
                    if self._dist is not None and self._clear:
                        hard = min(c.min_clearance_m for c in self._clear)
                        pro = Action4D(vx=tx * spd, vy=ty * spd,
                                       vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                        if self._clear_min_dist(state, pro) < hard:
                            con = Action4D(vx=-tx * spd, vy=-ty * spd,
                                           vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                            if self._clear_min_dist(state, con) >= hard:
                                tx, ty = -tx, -ty
                    vx, vy = tx * spd, ty * spd
                repairs.append(Repair(operator="GeofenceSlide",
                                      detail=f"{f.id}: removed {into:.2f} m/s into-zone component"))
        return Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
