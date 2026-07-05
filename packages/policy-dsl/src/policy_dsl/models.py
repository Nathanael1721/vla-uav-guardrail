"""DSL surface — the hand-authored, Pydantic-validated form (policy-dsl.md).

This is the Phase-1 slice: only the two taxonomy classes the mid-term demo needs
(``polygon_fence`` and ``altitude_envelope``). The remaining classes (corridor,
kinematic/distance envelopes, and the three hot-apply classes) land in Phase 2.

Constraints are a discriminated union on the ``type`` field. The base fields
mirror ``ConstraintBase`` in the design doc.
"""

from __future__ import annotations

from typing import Annotated, Any, Literal

from pydantic import BaseModel, Field, field_validator

ConstraintType = Literal["hard", "soft"]
Scope = Literal["global", "mission", "segment", "waypoint"]
Priority = Literal["P0", "P1", "P2"]
Layer = Literal["regulation", "site", "mission"]
ViolationAction = Literal["monitor_only", "project_fix", "brake", "loiter", "RTL", "land"]
AltitudeRef = Literal["AGL", "MSL"]


class LatLon(BaseModel):
    model_config = {"frozen": True}
    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)


class ConstraintBase(BaseModel):
    id: str
    constraint_type: ConstraintType
    scope: Scope = "mission"
    priority: Priority = "P1"
    layer: Layer = "mission"
    violation_action: ViolationAction


class PolygonGeometry(BaseModel):
    vertices: list[LatLon] = Field(min_length=3)
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 200.0
    altitude_ref: AltitudeRef = "AGL"


class PolygonFence(ConstraintBase):
    type: Literal["polygon_fence"] = "polygon_fence"
    geometry: PolygonGeometry


class AltitudeEnvelope(ConstraintBase):
    type: Literal["altitude_envelope"] = "altitude_envelope"
    altitude_min_m: float
    altitude_max_m: float
    altitude_ref: AltitudeRef = "AGL"


Constraint = Annotated[PolygonFence | AltitudeEnvelope, Field(discriminator="type")]


class PolicyDoc(BaseModel):
    """Top-level authored document — what an operator writes as YAML/JSON."""

    policy_id: str
    version: str
    generation: int = 0
    issued_at: str | None = None
    constraints: list[Constraint] = Field(min_length=1)

    @field_validator("issued_at", mode="before")
    @classmethod
    def _coerce_timestamp(cls, v: Any) -> Any:
        # PyYAML resolves ISO timestamps to datetime; keep the canonical IR as text.
        if v is not None and not isinstance(v, str):
            return v.isoformat()
        return v
