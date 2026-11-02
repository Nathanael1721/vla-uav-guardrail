"""Convert a policy to the PI's reference implementation's DSL - what can be.

WP1-05 (grant: the signed bundle must be usable by the reference
implementation, kuanting-vla-uav-guardrail packages/policy-dsl). Measured on
2026-10-29: the reference's Phase-1 DSL can represent 0 of the shipped
policies as a whole - it has two rule types (polygon_fence, altitude_envelope),
WGS84 vertices only, no margin ring and no schedule - so the whole-policy form
(`Policy.reference_ir`, `write_bundle(..., ir_form="reference")`) converts
nothing this project flies. This module converts the REPRESENTABLE SUBSET and
says exactly what it left out and why, rule by rule.

THE TWO-HASH CONTRACT (docs/DESIGN-policy-dsl-grant-form.md, section 7)

  * this project's hash, `sha256-canonical-v3` (`Policy.policy_hash`): the
    identity of the WHOLE policy. Every run, CSP and audit record carries it.
  * the reference's hash, `reference-policy-dsl-0.1`: the reference's own
    `policy_hash(PolicyDoc.model_dump(mode="json"))` of the document this
    converter writes. When every rule converts exactly (`complete`) it is the
    same policy in the reference's form (it equals `Policy.reference_hash()`).
    Otherwise it identifies the PROJECTION - a weaker policy, the rules the
    reference can enforce - and never stands for the whole: the record names
    every rule left out.

A bundle manifest carries both (`hashes`, and since 2026-11-02
`reference_projection` with the projection's hash and coverage); check_bundle
recomputes and verifies the projection record from the policy it rebuilds.

WHAT CONVERTS, RULE BY RULE

  exact          altitude_envelope; polygon_fence with WGS84 vertices, or
                 metre vertices and a WGS84 anchor for the frame's (0, 0)
                 (the policy's `origin`, else the `origin` argument), and no
                 margin ring; circle_fence the same way, as the 32-gon this
                 project derives and enforces (models.CircleFence).
  conservative   only with `conservative=True`, and recorded as such: a
                 margin ring folded into the vertices (the polygon grown by
                 the margin with mitred corners, which covers the round ring
                 the Shield enforces - checked); a schedule dropped (the rule
                 is then always in force in the reference: stricter, never
                 looser, for both convertible types).
  omitted        every other type (corridor, kinematic_envelope,
                 distance_envelope, dynamic_nfz, time_window_switch,
                 corridor_swap, obstacle_clearance, subject_standoff: not in
                 the reference's DSL), metre vertices with no anchor, and
                 - without `conservative` - a margin ring or a schedule.

Policy-level fields the reference has no place for are reported, not
converted: `origin` (WGS84 points need no frame; the reference derives its
own from the first polygon), `layers_merged` (not a PolicyDoc field), and a
rule's `locked_by` (layer-merge provenance; the reference has no hot-apply to
guard). A reference document needs at least one rule; when none converts the
document is None and no reference hash exists - none is invented.

What the reference's loader does with the document is checked with its own
code, in its own 3.11 environment, where that environment exists
(tests/test_policy_dsl_grant_form.py; it SKIPs elsewhere).
"""
from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from typing import Any

from .fsm import canonical_action
from .models import (AltitudeEnvelope, CircleFence, Policy, PolygonFence,
                     canonical_json)
from .projection import LocalProjection

# What the reference's Phase-1 DSL holds (packages/policy-dsl/.../models.py).
REFERENCE_TYPES = ("polygon_fence", "altitude_envelope")


@dataclass
class Conversion:
    """The reference-form document of what converts, and the account of what
    did not. `doc` is None when no rule converts."""
    policy_id: str
    doc: dict | None
    converted: list[str] = field(default_factory=list)       # exact
    conservative: dict[str, str] = field(default_factory=dict)   # id -> how
    omitted: dict[str, str] = field(default_factory=dict)        # id -> why
    policy_fields: dict[str, str] = field(default_factory=dict)  # field -> why
    notes: dict[str, str] = field(default_factory=dict)          # exact, reworded

    @property
    def complete(self) -> bool:
        """Every rule converted exactly and nothing reworded or dropped: the
        document is this policy in the reference's form (it equals
        `Policy.reference_ir()`, and its hash `Policy.reference_hash()`)."""
        return (self.doc is not None and not self.omitted and not self.conservative
                and not self.policy_fields and not self.notes)

    @property
    def reference_hash(self) -> str | None:
        """The reference's policy_hash of `doc` (its canonicalize(): sorted
        keys, no whitespace, ASCII - models.canonical_json), or None."""
        if self.doc is None:
            return None
        return "sha256:" + hashlib.sha256(canonical_json(self.doc)).hexdigest()

    def record(self) -> dict[str, Any]:
        """The manifest's `reference_projection` (see the module docstring)."""
        return {"policy_hash": self.reference_hash,
                "rules": [r["id"] for r in (self.doc or {}).get("constraints", [])],
                "conservative": dict(sorted(self.conservative.items())),
                "reworded": dict(sorted(self.notes.items())),
                "omitted": dict(sorted(self.omitted.items())),
                "not_carried": dict(sorted(self.policy_fields.items())),
                "complete": self.complete}


def _band(c) -> dict:
    return {"altitude_floor_m": c.altitude_floor_m,
            "altitude_ceiling_m": c.altitude_ceiling_m,
            "altitude_ref": c.altitude_ref or "AGL"}


def _base(c) -> dict:
    # The reference writes scope and layer out; absent here means "mission",
    # as there (models.DEFAULT_SCOPE / DEFAULT_LAYER).
    return {"id": c.id, "type": "polygon_fence" if isinstance(c, PolygonFence)
            else c.type, "constraint_type": c.constraint_type,
            "scope": c.effective_scope, "priority": c.priority,
            "layer": c.effective_layer,
            "violation_action": canonical_action(c.violation_action)}


def to_reference(policy: Policy, *, origin: tuple[float, float] | None = None,
                 conservative: bool = False) -> Conversion:
    """`policy` in the reference implementation's PolicyDoc form, as far as it
    goes (see the module docstring). `origin` anchors a metre policy that
    states none: the WGS84 position of its frame's (0, 0)."""
    out = Conversion(policy_id=policy.policy_id, doc=None)
    anchor = (policy.origin.lat, policy.origin.lon) if policy.origin is not None \
        else (None if origin is None else (float(origin[0]), float(origin[1])))
    proj = LocalProjection(*anchor) if anchor is not None else None
    if policy.origin is not None:
        out.policy_fields["origin"] = ("not a PolicyDoc field; WGS84 vertices need no "
                                       "frame and the reference derives its own")
    if policy.layers_merged is not None:
        out.policy_fields["layers_merged"] = "not a PolicyDoc field (it would be ignored)"
    rules = []
    for c in policy.constraints:
        if c.locked_by is not None:
            out.policy_fields[f"{c.id}.locked_by"] = (
                "layer-merge provenance; the reference has no hot-apply to guard")
        if c.type not in ("polygon_fence", "circle_fence", "altitude_envelope"):
            out.omitted[c.id] = (f"{c.type} is not in the reference's DSL "
                                 f"({', '.join(REFERENCE_TYPES)})")
            continue
        how: list[str] = []
        if c.valid_time is not None:
            if not conservative:
                out.omitted[c.id] = ("valid_time: the reference has no schedule and "
                                     "would drop it without a word")
                continue
            how.append("schedule dropped: always in force in the reference (stricter)")
        r = _base(c)
        if isinstance(c, AltitudeEnvelope):
            r.update(altitude_min_m=c.alt_min_m, altitude_max_m=c.alt_max_m,
                     altitude_ref=c.altitude_ref or "AGL")
        else:
            pts = [(v.lat, v.lon) if v.geographic else None for v in c.vertices]
            if any(p is None for p in pts):
                if proj is None:
                    out.omitted[c.id] = ("metre vertices and no WGS84 anchor (the "
                                         "policy states no origin; pass origin=)")
                    continue
                pts = [(v.lat, v.lon) if v.geographic else proj.to_latlon(v.x, v.y)
                       for v in c.vertices]
            if c.margin_m != 0.0:
                if not conservative:
                    out.omitted[c.id] = (f"margin_m {c.margin_m:g}: the reference has "
                                         f"no margin ring")
                    continue
                pts, grew = _grown(c, proj, policy)
                how.append(grew)
            r["geometry"] = {"vertices": [{"lat": la, "lon": lo} for la, lo in pts],
                             **_band(c)}
            if isinstance(c, CircleFence):
                out.notes[c.id] = (f"circle_fence written as the polygon_fence of the "
                                   f"{CircleFence.SEGMENTS}-gon this project derives "
                                   f"and enforces")
        if how:
            out.conservative[c.id] = "; ".join(how)
        else:
            out.converted.append(c.id)
        rules.append(r)
    if rules:
        out.doc = {"policy_id": policy.policy_id, "version": policy.version,
                   "generation": policy.generation, "issued_at": policy.issued_at,
                   "constraints": rules}
    return out


def _grown(c, proj, policy) -> tuple[list[tuple[float, float]], str]:
    """The fence grown by its margin, mitred, in WGS84, and a sentence saying
    how much larger it is. Checked: the grown polygon covers the round margin
    ring the Shield enforces (geometry.keepout_area)."""
    from shapely.geometry import Polygon             # lazy, as models.py does
    from .geometry import keepout_area
    poly = Polygon([(v.x, v.y) for v in c.vertices])
    grown = poly.buffer(c.margin_m, join_style="mitre", mitre_limit=10.0)
    enforced = keepout_area(c)
    if not grown.buffer(1e-9).covers(enforced):      # pragma: no cover - a GEOS defect
        raise ValueError(f"{c.id}: the mitred ring does not cover the enforced one")
    if proj is None:
        proj = policy.projection()
    ring = list(grown.exterior.coords)[:-1]
    pts = [proj.to_latlon(x, y) for x, y in ring]
    extra = grown.area - enforced.area
    return pts, (f"margin_m {c.margin_m:g} folded into the vertices (mitred; covers "
                 f"the ring the Shield enforces, {extra:.1f} m2 larger)")
