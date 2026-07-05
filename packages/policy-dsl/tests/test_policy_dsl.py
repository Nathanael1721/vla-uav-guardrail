from pathlib import Path

import pytest
from policy_dsl import build_ir, ingest_text, load_bundle, write_bundle
from policy_dsl.models import PolicyDoc
from pydantic import ValidationError

DEMO = Path(__file__).resolve().parents[3] / "bundles" / "itri-icl-2026-demo.yaml"


def _demo_text() -> str:
    return DEMO.read_text()


def test_ingest_builds_indexed_ir():
    ir = ingest_text(_demo_text())
    assert ir.policy_id == "itri-icl-2026-demo"
    assert len(ir.polygons) == 1
    assert len(ir.envelopes) == 1
    assert ir.policy_hash.startswith("sha256:")


def test_policy_hash_is_deterministic():
    a = ingest_text(_demo_text()).policy_hash
    b = ingest_text(_demo_text()).policy_hash
    assert a == b


def test_signed_distance_inside_is_negative():
    ir = ingest_text(_demo_text())
    rec = ir.polygons[0]
    # a point near the centroid of the school-yard polygon is inside
    inside = ir.signed_distance(rec, 25.04245, 121.5314)
    assert inside < 0
    # a point well to the west is outside
    outside = ir.signed_distance(rec, 25.04245, 121.5290)
    assert outside > 0


def test_nearest_exterior_pulls_point_out():
    ir = ingest_text(_demo_text())
    rec = ir.polygons[0]
    lat, lon = 25.04245, 121.5314  # inside
    out_lat, out_lon = ir.nearest_exterior_latlon(rec, lat, lon, margin_m=2.0)
    assert ir.signed_distance(rec, out_lat, out_lon) > 0  # now outside


def test_bundle_roundtrip_and_hash_check(tmp_path):
    ir = ingest_text(_demo_text())
    bundle = write_bundle(ir, tmp_path / "demo.tar.gz")
    loaded = load_bundle(bundle)
    assert loaded.policy_hash == ir.policy_hash
    assert len(loaded.polygons) == 1


def test_polygons_near_uses_spatial_index():
    ir = ingest_text(_demo_text())
    near = ir.polygons_near(25.04245, 121.5314, radius_m=50)
    assert len(near) == 1
    far = ir.polygons_near(25.20, 121.80, radius_m=50)
    assert far == []


def test_invalid_doc_rejected():
    with pytest.raises(ValidationError):
        PolicyDoc.model_validate({"policy_id": "x", "version": "1", "constraints": []})


def test_polygon_needs_three_vertices():
    bad = {
        "policy_id": "x",
        "version": "1",
        "constraints": [
            {
                "id": "p",
                "type": "polygon_fence",
                "constraint_type": "hard",
                "violation_action": "RTL",
                "geometry": {"vertices": [{"lat": 1, "lon": 1}, {"lat": 2, "lon": 2}]},
            }
        ],
    }
    with pytest.raises(ValidationError):
        build_ir(PolicyDoc.model_validate(bad))
