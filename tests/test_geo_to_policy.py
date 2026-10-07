"""tools/geo_to_policy.py: GeoJSON / KML -> polygon_fence, and the ingest tool.

Run either way:
    pytest tests/test_geo_to_policy.py -v
    python tests/test_geo_to_policy.py

A converter that produces SOMETHING for every input cannot fail, so most of
these tests are about refusal (holes, lines, a foreign CRS, duplicate ids, a
zone with no frame to land in) and about the one error that never refuses: the
longitude/latitude swap, which moves a Taipei zone to the Southern Ocean and
still validates. The positive tests go all the way to the Shield: a converted
zone must keep a vehicle out, or the conversion only produced text.
"""
import json
import sys
import tempfile
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import geo_to_policy as G                                         # noqa: E402
from guardrail.bundle import check_bundle                          # noqa: E402
from guardrail.models import Policy, State, load_policy            # noqa: E402
from guardrail.shield import Shield                                # noqa: E402

DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
WGS84 = ROOT / "policies" / "wgs84_taipei.yaml"
SKIP = "SKIP"

# A school yard near NTUT, 80 m east-west by 22 m north-south, in RFC 7946
# order [lon, lat]. Longer east-west ON PURPOSE: a swapped axis would make it
# longer north-south, so the shape itself detects the swap.
YARD = [[121.5310, 25.0421], [121.5318, 25.0421], [121.5318, 25.0423],
        [121.5310, 25.0423], [121.5310, 25.0421]]


def _fc(*features):
    return {"type": "FeatureCollection", "features": list(features)}


def _feature(ring, **props):
    return {"type": "Feature", "properties": props,
            "geometry": {"type": "Polygon", "coordinates": [ring]}}


def _write(obj, name):
    p = Path(tempfile.mkdtemp(prefix="geo_")) / name
    p.write_text(obj if isinstance(obj, str) else json.dumps(obj), encoding="utf-8")
    return p


def _refused(fn, *words):
    try:
        fn()
    except ValueError as e:
        for w in words:
            assert w in str(e), (w, str(e))
        return
    raise AssertionError(f"accepted; expected a refusal mentioning {words}")


# --------------------------------------------------------------------------- #
# convert
# --------------------------------------------------------------------------- #

def test_a_geojson_zone_becomes_a_grant_form_fence_in_lat_lon_order():
    doc = G.convert(_write(_fc(_feature(YARD, name="School Yard")), "z.geojson"),
                    "zones", defaults={"layer": "site"})
    (r,) = doc["constraints"]
    assert r["id"] == "school-yard" and r["type"] == "polygon_fence", r
    assert r["layer"] == "site" and r["violation_action"] == "project_fix", r
    v = r["geometry"]["vertices"]
    assert len(v) == 4, "the repeated closing vertex must be dropped"
    assert v[0] == {"lat": 25.0421, "lon": 121.5310}, v[0]
    assert "origin" not in doc, "the grant's form derives the frame; no origin"


def test_the_axis_order_survives_into_the_shield_frame():
    """The swap the converter exists to get right. Projected, the yard must be
    wider in y (East) than in x (North): ~80 m by ~22 m."""
    pol = Policy.model_validate(G.convert(_write(_fc(_feature(YARD)), "z.geojson"), "z"))
    xs = [p.x for p in pol.constraints[0].vertices]
    ys = [p.y for p in pol.constraints[0].vertices]
    dx, dy = max(xs) - min(xs), max(ys) - min(ys)
    assert 70.0 < dy < 90.0 and 15.0 < dx < 30.0, (dx, dy)


def test_a_converted_zone_is_enforced_by_the_shield():
    pol = Policy.model_validate(G.convert(
        _write(_fc(_feature(YARD, altitude_ceiling_m=60)), "z.geojson"), "z"))
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    assert sh.state_is_unsafe(State(x=0.0, y=0.0, up=20.0)), \
        "the frame origin is the yard's centroid, which must be unsafe"
    assert not sh.state_is_unsafe(State(x=0.0, y=0.0, up=80.0)), "above the ceiling"
    assert not sh.state_is_unsafe(State(x=200.0, y=0.0, up=20.0))


def test_dsl_properties_are_used_and_others_ignored():
    doc = G.convert(_write(_fc(_feature(YARD, priority="P1", margin_m=2.5,
                                        altitude_floor_m=10, fill="#ff0000",
                                        name="a")), "z.geojson"), "z")
    r = doc["constraints"][0]
    assert r["priority"] == "P1" and r["margin_m"] == 2.5, r
    assert r["geometry"]["altitude_floor_m"] == 10.0
    assert "fill" not in json.dumps(r)


def test_a_multipolygon_gives_one_rule_per_polygon():
    east = [[x + 0.001, y] for x, y in YARD]
    mp = {"type": "Feature", "id": "pair", "properties": {},
          "geometry": {"type": "MultiPolygon", "coordinates": [[YARD], [east]]}}
    doc = G.convert(_write(_fc(mp), "z.geojson"), "z")
    assert [r["id"] for r in doc["constraints"]] == ["pair-1", "pair-2"]


def test_a_hole_is_refused_not_dropped():
    inner = [[121.5312, 25.04215], [121.5314, 25.04215], [121.5314, 25.04225],
             [121.5312, 25.04221], [121.5312, 25.04215]]
    holed = {"type": "Feature", "properties": {},
             "geometry": {"type": "Polygon", "coordinates": [YARD, inner]}}
    _refused(lambda: G.convert(_write(_fc(holed), "z.geojson"), "z"), "hole")


def test_non_polygons_are_refused():
    line = {"type": "Feature", "properties": {},
            "geometry": {"type": "LineString", "coordinates": YARD[:2]}}
    _refused(lambda: G.convert(_write(_fc(line), "z.geojson"), "z"), "LineString")


def test_a_foreign_crs_is_refused():
    doc = _fc(_feature(YARD))
    doc["crs"] = {"type": "name", "properties": {"name": "EPSG:3826"}}   # TWD97
    _refused(lambda: G.convert(_write(doc, "z.geojson"), "z"), "EPSG:3826")


def test_lat_lon_written_in_the_wrong_order_is_refused_when_it_can_be_seen():
    """[lat, lon] instead of [lon, lat]: for Taipei, lon 121 read as a latitude
    is out of range - the one swap that CAN be caught by range."""
    swapped = [[lat, lon] for lon, lat in YARD]
    _refused(lambda: G.convert(_write(_fc(_feature(swapped)), "z.geojson"), "z"),
             "LONGITUDE first")


def test_duplicate_feature_names_are_refused():
    east = [[x + 0.001, y] for x, y in YARD]
    _refused(lambda: G.convert(_write(_fc(_feature(YARD, name="nfz"),
                                          _feature(east, name="nfz")), "z.geojson"), "z"),
             "unique")


def test_kml_gives_the_same_rule_as_the_equivalent_geojson():
    coords = " ".join(f"{lon},{lat},0" for lon, lat in YARD)
    kml = (f'<?xml version="1.0"?><kml xmlns="http://www.opengis.net/kml/2.2">'
           f'<Document><Placemark><name>School Yard</name><Polygon><outerBoundaryIs>'
           f'<LinearRing><coordinates>{coords}</coordinates></LinearRing>'
           f'</outerBoundaryIs></Polygon></Placemark></Document></kml>')
    a = G.convert(_write(kml, "z.kml"), "z")
    b = G.convert(_write(_fc(_feature(YARD, name="School Yard")), "z.geojson"), "z")
    assert a == b, (a, b)


def test_a_kml_hole_is_refused():
    coords = " ".join(f"{lon},{lat}" for lon, lat in YARD)
    kml = (f'<kml xmlns="http://www.opengis.net/kml/2.2"><Placemark><name>h</name>'
           f'<Polygon><outerBoundaryIs><LinearRing><coordinates>{coords}</coordinates>'
           f'</LinearRing></outerBoundaryIs><innerBoundaryIs><LinearRing><coordinates>'
           f'{coords}</coordinates></LinearRing></innerBoundaryIs></Polygon>'
           f'</Placemark></kml>')
    _refused(lambda: G.convert(_write(kml, "z.kml"), "z"), "hole")


def test_the_convert_cli_writes_a_file_the_loader_accepts():
    out = Path(tempfile.mkdtemp(prefix="geo_")) / "zones.yaml"
    src = _write(_fc(_feature(YARD, name="yard")), "z.geojson")
    assert G.main(["convert", str(src), "-o", str(out), "--policy-id", "zones",
                   "--ceiling", "90"]) == 0
    pol = load_policy(out, runtime=False)
    assert pol.constraints[0].altitude_ceiling_m == 90.0
    assert b"\r\n" not in out.read_bytes(), "LF, as every other policy file"
    try:
        load_policy(out)
    except ValueError as e:
        assert "derived from the geometry" in str(e), e
    else:
        raise AssertionError("a derived frame was accepted for flight")
    # A take-off point ~120 m south of the yard. (This test used the yard's
    # own centre until 2026-10-07: the policy "became flyable" with the
    # vehicle starting inside the zone.)
    assert G.main(["convert", str(src), "-o", str(out), "--policy-id", "zones",
                   "--origin", "25.0410,121.5314"]) == 0
    pol = load_policy(out)
    assert pol.origin is not None, "--origin makes it flyable"
    assert pol.start_conflicts() == []
    assert not Shield(pol).state_is_unsafe(State(x=0.0, y=0.0, up=20.0)), \
        "the stated take-off point must be a safe start"


def test_a_take_off_point_inside_a_converted_zone_is_refused():
    """--origin states where the vehicle starts. Inside a zone this command
    creates, the policy would fly a start in breach. Shown failing before
    2026-10-07: the yard's own centre was accepted as the take-off point."""
    src = _write(_fc(_feature(YARD, name="yard")), "z.geojson")
    _refused(lambda: G.convert(src, "zones", origin=(25.0422, 121.5314)),
             "inside a zone being converted", "yard")
    assert G.main(["convert", str(src), "--policy-id", "zones",
                   "--origin", "25.0422,121.5314"]) == 2


def test_a_doubled_vertex_is_read_once_not_refused():
    """GIS exports often repeat a vertex. Shapely calls such a ring valid, and
    so must the converter. Shown failing before 2026-10-07: refused as "a
    zero-length edge"."""
    doubled = YARD[:2] + [YARD[1]] + YARD[2:]
    doc = G.convert(_write(_fc(_feature(doubled, name="yard")), "z.geojson"), "z")
    assert len(doc["constraints"][0]["geometry"]["vertices"]) == 4
    from shapely.geometry import Polygon as SPoly
    assert SPoly([(c[0], c[1]) for c in doubled]).is_valid


def test_an_unstated_band_takes_the_dsl_default_not_a_third_one():
    """No band in the feature or on the command line: the DSL's geometry-block
    default (0-200 m, the reference's) applies. Shown failing before
    2026-10-07: the converter wrote its own undocumented 120 m ceiling."""
    doc = G.convert(_write(_fc(_feature(YARD)), "z.geojson"), "z")
    g = doc["constraints"][0]["geometry"]
    assert "altitude_ceiling_m" not in g and "altitude_floor_m" not in g, g
    f = Policy.model_validate(doc).constraints[0]
    assert (f.altitude_floor_m, f.altitude_ceiling_m) == (0.0, 200.0)


# --------------------------------------------------------------------------- #
# ingest
# --------------------------------------------------------------------------- #

def test_a_post_nfz_payload_is_hot_applied_onto_its_base():
    """The body guardrail/api.py's POST /nfz takes, ingested the way the API
    itself changes the live policy: rule appended, generation bumped, hash
    restamped - the same hash. Shown failing on the 2026-10-07 06:30 tool,
    which appended a polygon_fence while the API (since that day) spawns a
    dynamic_nfz, the grant's only hot-applicable zone class."""
    from guardrail import api
    base = load_policy(DEMO)
    payload = {"id": "nfz-landslide", "vertices": [{"x": 23, "y": 6}, {"x": 31, "y": 6},
                                                    {"x": 31, "y": 14}],
               "altitude_floor_m": 0, "altitude_ceiling_m": 100, "margin_m": 1.0}
    pol = G.ingest(payload, base)
    assert pol.generation == base.generation + 1
    assert pol.policy_hash != base.policy_hash
    assert (pol.constraints[-1].id, pol.constraints[-1].type) == ("nfz-landslide",
                                                                  "dynamic_nfz")
    for route in (api.apply_legacy_nfz, api.apply_dynamic_nfz):     # POST /nfz, /dynamic_nfz
        live = Shield(load_policy(DEMO))
        route(live, dict(payload))
        assert live.policy.policy_hash == pol.policy_hash, (route.__name__,
                                                             "ingest and the API disagree")
    moving = G.ingest({**payload, "motion": {"vx_mps": 1.0}}, base)
    assert moving.constraints[-1].motion.vx_mps == 1.0, "a POST /dynamic_nfz body"


def test_a_get_policy_body_ingests_to_the_same_policy():
    for f in (DEMO, WGS84):
        pol = load_policy(f)
        body = json.loads(json.dumps(pol.model_dump()))       # what GET /policy sends
        assert G.ingest(body).policy_hash == pol.policy_hash, f.name


def _serve(routes: dict):
    """A loopback HTTP server answering GET <path> with routes[path] (bytes),
    404 otherwise. Returns (base_url, stop). Nothing leaves 127.0.0.1."""
    import threading
    from http.server import BaseHTTPRequestHandler, HTTPServer

    class H(BaseHTTPRequestHandler):
        def do_GET(self):                                     # noqa: N802
            body = routes.get(self.path)
            self.send_response(200 if body is not None else 404)
            self.end_headers()
            self.wfile.write(body if body is not None else b"not found")

        def log_message(self, *a):                            # quiet
            pass
    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return f"http://127.0.0.1:{srv.server_address[1]}", srv.shutdown


def test_ingest_reads_the_rest_endpoint_itself():
    """The grant's Outputs table: "Ingest tool - CLI; reads files + REST
    endpoint, emits bundle". Until 2026-10-07 the endpoint could only be piped
    in (curl ... | ingest -). Now a URL is fetched: GET /policy's body becomes
    the same policy, a POST /nfz-shaped body served at a URL is appended to its
    base, and an endpoint that fails is refused by name, never read as empty."""
    pol = load_policy(DEMO)
    nfz = {"id": "nfz-landslide", "vertices": [{"x": 23, "y": 6}, {"x": 31, "y": 6},
                                                {"x": 31, "y": 14}],
           "altitude_floor_m": 0, "altitude_ceiling_m": 100, "margin_m": 1.0}
    url, stop = _serve({"/policy": json.dumps(pol.model_dump()).encode(),
                        "/nfz-body": json.dumps(nfz).encode(), "/empty": b""})
    try:
        assert G.ingest(url + "/policy").policy_hash == pol.policy_hash
        added = G.ingest(url + "/nfz-body", load_policy(DEMO))
        assert added.constraints[-1].id == "nfz-landslide"
        assert added.generation == pol.generation + 1
        _refused(lambda: G.ingest(url + "/missing"), "could not read the endpoint")
        _refused(lambda: G.ingest(url + "/empty"), "empty body")
        out = Path(tempfile.mkdtemp(prefix="geo_")) / "rest.tar.gz"
        assert G.main(["ingest", url + "/policy", "-o", str(out), "--unsigned"]) == 0
        assert check_bundle(out).policy.policy_hash == pol.policy_hash
    finally:
        stop()


def test_a_payload_without_a_base_is_refused():
    _refused(lambda: G.ingest({"id": "z", "vertices": [{"x": 0, "y": 0}]}), "--base")


def test_an_unknown_key_in_a_post_nfz_payload_is_refused():
    _refused(lambda: G.ingest({"id": "z", "vertices": [{"x": 0, "y": 0}, {"x": 1, "y": 0},
                                                       {"x": 1, "y": 1}],
                               "altitude_cieling_m": 40}, load_policy(DEMO)),
             "unknown keys")


def test_a_duplicate_id_against_the_base_is_refused():
    base = load_policy(DEMO)
    dup = {"id": base.constraints[0].id,
           "vertices": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]}
    _refused(lambda: G.ingest(dup, base), "used by 2 rules")


def test_geographic_zones_are_refused_onto_a_metre_base_with_no_origin():
    """sim_demo_policy is in metres with no origin: there is no telling where
    in its frame a lat/lon zone lies. Appending anyway would misplace it."""
    _refused(lambda: G.ingest(_fc(_feature(YARD)), load_policy(DEMO)), "origin")


def test_geographic_zones_land_in_a_geographic_base():
    base = load_policy(WGS84)
    east = [[x + 0.003, y] for x, y in YARD]
    pol = G.ingest(_fc(_feature(east, name="east-yard")), base)
    when = datetime(2026, 9, 7, 9, 0)
    sh = Shield(pol, lookahead_s=3.0, dt=0.5, now=lambda: when)
    new = pol.constraints[-1]
    cx = sum(v.x for v in new.vertices) / len(new.vertices)
    cy = sum(v.y for v in new.vertices) / len(new.vertices)
    assert 250.0 < cy < 350.0 and abs(cx) < 30.0, (cx, cy)   # ~300 m east
    assert sh.state_is_unsafe(State(x=cx, y=cy, up=20.0))


def test_the_ingest_cli_emits_a_bundle_that_reloads():
    out = Path(tempfile.mkdtemp(prefix="geo_")) / "b.tar.gz"
    src = _write({"id": "nfz-x", "vertices": [{"x": 40, "y": 40}, {"x": 45, "y": 40},
                                              {"x": 45, "y": 45}]}, "nfz.json")
    assert G.main(["ingest", str(src), "--base", str(DEMO), "-o", str(out),
                   "--unsigned"]) == 0
    chk = check_bundle(out)
    assert chk.policy.constraints[-1].id == "nfz-x" and chk.policy.generation == 1
    bad = _write({"id": "nfz-x", "vertices": [{"x": 0, "y": 0}]}, "bad.json")
    assert G.main(["ingest", str(bad), "--base", str(DEMO), "-o", str(out),
                   "--unsigned"]) == 2


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
