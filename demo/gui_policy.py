"""The policy document the Mission Control GUI writes (demo/gui_mission_control.py).

    doc, notes = gui_policy_doc(rects, polys, high=False)

Kept apart from the GUI (no tkinter, no Unreal lookup) so it can be tested
headless: tests/test_wgs84_ingest_writers.py.

WP1-14 (2026-11-18). The grant: "WGS84 latitude / longitude is canonical ...
the DSL itself never carries projected coordinates". Since 2026-11-03 every
shipped policy, policies/gui_policy.yaml included, is stored in WGS84 about a
stated `origin` (tools/convert_policies_wgs84.py), and since 2026-11-17 every
event the Shield applies to a policy with a frame writes WGS84 too
(models.rule_in_wgs84). The GUI was the last default writer still producing
the metre form (models.Policy.point_form lists the opt-in and legacy-base
ones that remain): it rewrote gui_policy.yaml
from the zones drawn on its map as `{x, y}` metres with no `origin`, so one
click on "Fly" turned the converted file back into the legacy form. It now
converts at ingest: the zones are drawn in the local frame (x North, y East,
metres) and written as {lat, lon} about the base policy's frame origin
(read_base; projection.raw_points_in_wgs84, the conversion every other writer uses), and
the document states that origin. The geometry the Shield enforces is the
lazy local view of the stored lat/lon, which for the GUI's 0.1 m values is
exactly the metres drawn (projection.metres_to_wgs84: exact on the
micrometre grid).
"""
from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
BASE_POLICY = ROOT / "policies" / "urban_demo_policy.yaml"

# The Project AirSim scenes' home-geo-point (tools/convert_policies_wgs84.py
# PAS_HOME): the origin policies/urban_demo_policy.yaml states, used only when
# the base policy cannot be read or states none.
PAS_HOME = (35.6895, 139.6917)

# Fallback constraints if the base policy cannot be read (same structure as
# policies/urban_demo_policy.yaml).
FALLBACK_KEEP = [
    {"id": "alt-band-urban", "type": "altitude_envelope",
     "constraint_type": "hard", "scope": "mission", "priority": "P0",
     "violation_action": "repair", "alt_min_m": 15, "alt_max_m": 25},
    {"id": "kin-caps", "type": "kinematic_envelope",
     "constraint_type": "hard", "scope": "mission", "priority": "P1",
     "violation_action": "repair", "speed_max_mps": 4.0, "climb_rate_max_mps": 2.0,
     "yaw_rate_max_dps": 45.0},
]


def _fence(fid: str, ring) -> dict[str, Any]:
    return {"id": fid, "type": "polygon_fence", "constraint_type": "hard",
            "scope": "mission", "priority": "P0", "violation_action": "repair",
            "vertices": [{"x": round(px, 1), "y": round(py, 1)} for px, py in ring],
            "altitude_floor_m": 0, "altitude_ceiling_m": 60, "margin_m": 1.0}


def metre_constraints(rects, polys, keep: list[dict], high: bool) -> list[dict]:
    """The rules as drawn, in local metres (x North, y East): one
    polygon_fence per rectangle (xmin, xmax, ymin, ymax) and per closed
    polygon (>= 3 (x, y) vertices), then `keep` (a copy) with its altitude
    band set from the GUI selector. What the GUI wrote, unconverted, until
    2026-11-18."""
    fences = []
    for i, (xmin, xmax, ymin, ymax) in enumerate(rects):
        fences.append(_fence(f"gui-nfz-{i + 1}", [(xmin, ymin), (xmax, ymin),
                                                  (xmax, ymax), (xmin, ymax)]))
    for i, poly in enumerate(polys):
        if len(poly) < 3:
            continue
        fences.append(_fence(f"gui-poly-{i + 1}", poly))
    keep = copy.deepcopy(keep)
    # altitude band from the GUI selector (High = fly over the city roofs)
    amin, amax = (35, 55) if high else (15, 25)
    for c in keep:
        if c.get("type") == "altitude_envelope":
            c["alt_min_m"], c["alt_max_m"] = amin, amax
    return fences + keep


def read_base(base_path: str | Path = BASE_POLICY):
    """(rules kept from the base policy - every one but its polygon fences,
    which the drawn zones replace -, its version, its frame origin (lat, lon),
    a note or None).

    The origin is the base's frame origin (models.Policy.frame_origin): its
    stated `origin`, else the one its geographic rules derive (the mean of
    its first polygon fence, then its first geographic rule), so the drawn
    zones and any lat/lon rule the base keeps share one frame. PAS_HOME only
    when the base has no frame at all (written only in metres, or no point),
    with a note. Falls back to FALLBACK_KEEP, "0.1.0" and PAS_HOME when the
    base cannot be read."""
    try:
        import yaml
        base = yaml.safe_load(Path(base_path).read_text(encoding="utf-8"))
        keep = [c for c in base.get("constraints", []) if c.get("type") != "polygon_fence"]
        version = base.get("version", "0.1.0")
    except Exception as e:                                      # noqa: BLE001
        return (copy.deepcopy(FALLBACK_KEEP), "0.1.0", PAS_HOME,
                f"base policy read failed ({e}) - using built-in envelope constraints")
    try:
        from guardrail.models import load_policy
        origin = load_policy(base_path, runtime=False).frame_origin
    except Exception as e:                                      # noqa: BLE001
        return (keep, version, PAS_HOME,
                f"base policy {Path(base_path).name} did not validate ({e}); "
                f"zones written about {PAS_HOME}")
    if origin is None:
        return keep, version, PAS_HOME, (f"base policy {Path(base_path).name} has no "
                                         f"frame origin; zones written about {PAS_HOME}")
    return keep, version, (float(origin[0]), float(origin[1])), None


def gui_policy_doc(rects, polys, high: bool, base_path: str | Path = BASE_POLICY
                   ) -> tuple[dict[str, Any], list[str]]:
    """(the policy document the GUI writes, notes for its log).

    Every point is WGS84 about the stated `origin` (the base policy's), the
    canonical form; the metres drawn are converted at ingest
    (projection.raw_points_in_wgs84). Any metre point a kept base rule still
    carries is converted the same way, so the document holds no projected
    coordinate at all."""
    from guardrail.projection import raw_points_in_wgs84
    keep, version, origin, note = read_base(base_path)
    rules = raw_points_in_wgs84(metre_constraints(rects, polys, keep, high), origin)
    doc = {"policy_id": "gui-mission", "version": version,
           "origin": {"lat": origin[0], "lon": origin[1]},
           "constraints": rules}
    return doc, ([note] if note else [])
