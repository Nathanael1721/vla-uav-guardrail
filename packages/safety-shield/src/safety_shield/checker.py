"""Violation checker — the single rule-evaluation path in the system.

Reuses the Policy DSL IR (safety-shield.md: "there is exactly one rule-evaluation
code path"). For the Phase-1 slice it runs two of the three categories:

* **Geometric** — does any predicted pose fall inside a polygon fence?
* **Envelope** — does any predicted altitude leave an altitude envelope?

Time-window checking is Phase 2 (needs the hot-apply classes).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from policy_dsl import PolicyIR
from vlaguard_common import Action4D

from safety_shield.kinematics import PredictedPose, VehicleState, predict

Category = Literal["geometric", "envelope"]


@dataclass(frozen=True)
class Violation:
    rule_id: str
    category: Category
    priority: str
    violation_action: str
    first_hit_s: float
    # offending predicted position (lat/lon for geometric, alt for envelope)
    hit_lat: float
    hit_lon: float
    hit_alt_m: float


def check(
    ir: PolicyIR, state: VehicleState, action: Action4D, horizon_s: float = 5.0
) -> list[Violation]:
    """Return all violations the predicted trajectory would incur, earliest-hit first."""
    origin_xy = ir.projection.to_xy(state.lat, state.lon)
    poses = predict(state, action, origin_xy, horizon_s=horizon_s)

    # spatial pre-filter: only polygons within reach over the horizon
    reach_m = _reach(poses, origin_xy)
    candidates = ir.polygons_near(state.lat, state.lon, radius_m=reach_m + 1.0)

    violations: list[Violation] = []

    for rec in candidates:
        for pose in poses:
            lat, lon = ir.projection.to_latlon(pose.east_m, pose.north_m)
            if ir.signed_distance(rec, lat, lon) < 0:
                violations.append(
                    Violation(
                        rule_id=rec.id,
                        category="geometric",
                        priority=rec.priority,
                        violation_action=rec.violation_action,
                        first_hit_s=pose.t_s,
                        hit_lat=lat,
                        hit_lon=lon,
                        hit_alt_m=pose.alt_agl_m,
                    )
                )
                break  # earliest hit for this rule is enough

    for env in ir.envelopes:
        for pose in poses:
            if pose.alt_agl_m < env.altitude_min_m or pose.alt_agl_m > env.altitude_max_m:
                lat, lon = ir.projection.to_latlon(pose.east_m, pose.north_m)
                violations.append(
                    Violation(
                        rule_id=env.id,
                        category="envelope",
                        priority=env.priority,
                        violation_action=env.violation_action,
                        first_hit_s=pose.t_s,
                        hit_lat=lat,
                        hit_lon=lon,
                        hit_alt_m=pose.alt_agl_m,
                    )
                )
                break

    violations.sort(key=lambda v: v.first_hit_s)
    return violations


def _reach(poses: list[PredictedPose], origin_xy: tuple[float, float]) -> float:
    e0, n0 = origin_xy
    if not poses:
        return 0.0
    last = poses[-1]
    return float(((last.east_m - e0) ** 2 + (last.north_m - n0) ** 2) ** 0.5)
