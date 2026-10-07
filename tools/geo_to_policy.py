"""GeoJSON / KML -> polygon_fence rules, and the DSL's ingest tool (WP1-16).

    python tools/geo_to_policy.py convert zones.geojson -o zones.yaml --policy-id zones
    python tools/geo_to_policy.py convert zones.kml -o zones.yaml --policy-id zones --layer site
    python tools/geo_to_policy.py convert zones.geojson --policy-id zones --origin 25.0424,121.5314
    python tools/geo_to_policy.py ingest zones.geojson --base policies/x.yaml -o out.tar.gz
    python tools/geo_to_policy.py ingest nfz_payload.json --base policies/x.yaml -o out.tar.gz
    python tools/geo_to_policy.py ingest http://127.0.0.1:8071/policy -o out.tar.gz
    curl -s http://127.0.0.1:8071/policy | python tools/geo_to_policy.py ingest - -o out.tar.gz

WHAT THE GRANT ASKS FOR

Policy DSL page, "Locked design choices": "v1 ingest accepts hand-authored YAML /
JSON (canonical), KML / GeoJSON file imports (for GIS-tool reuse), and a live
REST endpoint for cloud-pushed updates". Its Outputs table names a "KML /
GeoJSON converter" and an "Ingest tool - CLI; reads files + REST endpoint,
emits bundle". Until 2026-10-06 neither existed (audit card WP1-16).

CONVERT

Every Polygon / MultiPolygon becomes one polygon_fence in the grant's own form:
WGS84 vertices in a `geometry:` block, no origin (the DSL derives the frame).
Such a document loads as a declaration; to FLY it, state the take-off point
with `--origin LAT,LON` (the ArduPilot SITL rail puts the vehicle's start,
its home, at the frame's origin - see Policy.frame_problem).

  * Axis order. GeoJSON (RFC 7946) and KML write [lon, lat]; the DSL writes
    {lat, lon}. Swapping them is the classic GIS bug, and it does not fail: a
    zone near Taipei lands in the Southern Ocean and validates. The test pins
    the order with a zone longer east-west than north-south.
  * Refused, not approximated: a polygon with holes (a polygon_fence has none,
    and dropping the hole would forbid the area the hole exempts - or keeping
    only the hole would forbid the wrong area); a non-polygon geometry; a
    GeoJSON `crs` other than WGS84; a feature with fewer than three distinct
    vertices; two features with the same id.
  * The closing vertex GeoJSON and KML repeat is dropped, and so is a vertex
    written twice in a row (common in GIS exports; it adds no edge and no
    area). A third coordinate (altitude) is ignored: the band comes from
    `altitude_floor_m` / `altitude_ceiling_m` in the feature's properties,
    else `--floor` / `--ceiling`, else it is left out of the rule and the
    DSL's geometry-block default applies: 0-200 m AGL, as in the reference.
  * `--origin` inside a converted zone is refused: it states the take-off
    point, and a flight would start in breach (Policy.start_conflicts).
  * Feature properties that are DSL fields (priority, constraint_type,
    violation_action, layer, scope, margin_m, altitude_floor_m,
    altitude_ceiling_m, altitude_ref) are used; every other property (names,
    styling) is ignored.

The result is validated by the DSL itself (`policy_from_raw`) before anything is
written: the converter never emits a file the loader would refuse.

INGEST

Reads one source and emits a signed bundle, through the same DSL entry point:

  * a policy file (.yaml / .json);
  * GeoJSON / KML (converted as above);
  * a REST payload, from a file, stdin ("-") or the endpoint itself (an
    http:// or https:// URL, fetched with GET; added 2026-10-07, so the tool
    "reads files + REST endpoint" as the grant's Outputs table says), in
    the shapes guardrail/api.py speaks: a zone body of POST /nfz or POST
    /dynamic_nfz ({id, vertices: [{x, y}], altitude_floor_m,
    altitude_ceiling_m, margin_m, motion?}), ingested as the dynamic_nfz the
    API spawns from it, or the body GET /policy returns (a whole policy). A
    single DSL rule (with `type`, the POST /hot_apply body) is accepted as a
    patch too.

With `--base`, new rules are APPENDED to the base policy and its generation is
bumped once per rule - exactly what the API's hot-apply does to a live policy,
so the bundle describes the policy that would be in force. A bundle holding a
dynamic_nfz is a declaration until the flight loaders accept that type
(guardrail/models.py, RUNTIME_TYPES). Geographic zones
cannot be appended to a base written in local metres without an origin: their
position in that frame is unknown, and guessing it would misplace every zone.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import yaml                                                         # noqa: E402

from guardrail.models import (DynamicNFZ, Policy, load_policy,      # noqa: E402
                              parse_policy_text, policy_from_raw)
from guardrail.projection import geo_points                         # noqa: E402

DSL_PROPERTIES = ("priority", "constraint_type", "violation_action", "layer",
                  "scope", "margin_m", "altitude_floor_m", "altitude_ceiling_m",
                  "altitude_ref")
_WGS84_CRS = {"urn:ogc:def:crs:OGC:1.3:CRS84", "urn:ogc:def:crs:OGC::CRS84",
              "EPSG:4326", "urn:ogc:def:crs:EPSG::4326"}


class ConversionError(ValueError):
    """The input cannot be turned into rules without guessing."""


# --------------------------------------------------------------------------- #
# geometry -> rings
# --------------------------------------------------------------------------- #

def _ring(coords, where: str) -> list[dict]:
    """[[lon, lat(, alt)], ...] -> [{lat, lon}, ...], closing vertex dropped."""
    pts = []
    for c in coords:
        if not isinstance(c, (list, tuple)) or len(c) < 2:
            raise ConversionError(f"{where}: coordinate {c!r} is not [lon, lat]")
        lon, lat = float(c[0]), float(c[1])
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ConversionError(
                f"{where}: [{c[0]}, {c[1]}] is out of WGS84 range; GeoJSON and KML "
                f"write LONGITUDE first - check the axis order")
        if pts and pts[-1] == {"lat": lat, "lon": lon}:
            continue        # a doubled vertex (common in GIS exports): one vertex
        pts.append({"lat": lat, "lon": lon})
    while len(pts) >= 2 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len({(p["lat"], p["lon"]) for p in pts}) < 3:
        raise ConversionError(f"{where}: fewer than three distinct vertices")
    return pts


def _polygon_rings(geom: dict, where: str) -> list[list[dict]]:
    """A GeoJSON geometry -> one outer ring per polygon. Holes are refused."""
    t = geom.get("type") if isinstance(geom, dict) else None
    if t == "Polygon":
        polys = [geom.get("coordinates") or []]
    elif t == "MultiPolygon":
        polys = geom.get("coordinates") or []
    elif t == "GeometryCollection":
        out = []
        for i, g in enumerate(geom.get("geometries") or []):
            out += _polygon_rings(g, f"{where}.geometries[{i}]")
        return out
    else:
        raise ConversionError(
            f"{where}: geometry type {t!r} is not a polygon; only Polygon and "
            f"MultiPolygon become polygon_fence rules")
    rings = []
    for k, rings_k in enumerate(polys):
        if not rings_k:
            raise ConversionError(f"{where}: an empty polygon")
        if len(rings_k) > 1:
            raise ConversionError(
                f"{where}: polygon {k} has {len(rings_k) - 1} hole(s). A "
                f"polygon_fence has none; split the zone into hole-free polygons "
                f"rather than let the converter decide which area is forbidden")
        rings.append(_ring(rings_k[0], f"{where} polygon {k}"))
    return rings


def _slug(text: str) -> str:
    s = re.sub(r"[^A-Za-z0-9]+", "-", str(text)).strip("-").lower()
    return s or "zone"


# --------------------------------------------------------------------------- #
# readers
# --------------------------------------------------------------------------- #

def zones_from_geojson(doc: dict) -> list[dict]:
    """[{id, vertices, properties}] for every polygon in a GeoJSON document."""
    if not isinstance(doc, dict):
        raise ConversionError("GeoJSON must be a JSON object")
    crs = ((doc.get("crs") or {}).get("properties") or {}).get("name")
    if crs is not None and crs not in _WGS84_CRS:
        raise ConversionError(
            f"GeoJSON crs {crs!r}: only WGS84 longitude/latitude is accepted "
            f"(RFC 7946); reproject before converting")
    t = doc.get("type")
    if t == "FeatureCollection":
        feats = doc.get("features") or []
    elif t == "Feature":
        feats = [doc]
    elif t in ("Polygon", "MultiPolygon", "GeometryCollection"):
        feats = [{"type": "Feature", "geometry": doc, "properties": {}}]
    else:
        raise ConversionError(f"not GeoJSON: top-level type {t!r}")
    zones = []
    for i, f in enumerate(feats):
        props = f.get("properties") or {}
        base = f.get("id") or props.get("id") or props.get("name") or f"zone-{i + 1}"
        rings = _polygon_rings(f.get("geometry") or {}, f"feature {i} ({base})")
        for k, ring in enumerate(rings):
            zid = _slug(base) if len(rings) == 1 else f"{_slug(base)}-{k + 1}"
            zones.append({"id": zid, "vertices": ring, "properties": props})
    return zones


def _local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def zones_from_kml(text: str) -> list[dict]:
    """[{id, vertices, properties}] for every Polygon in every Placemark."""
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        raise ConversionError(f"not KML: {exc}") from None
    zones = []
    marks = [e for e in root.iter() if _local(e.tag) == "Placemark"]
    for i, pm in enumerate(marks):
        name = next((e.text for e in pm if _local(e.tag) == "name" and e.text), None)
        base = name or f"zone-{i + 1}"
        polys = [e for e in pm.iter() if _local(e.tag) == "Polygon"]
        others = [_local(e.tag) for e in pm.iter()
                  if _local(e.tag) in ("Point", "LineString", "Track")]
        if not polys:
            raise ConversionError(
                f"placemark {base!r} holds {others or 'no geometry'}, not a "
                f"Polygon; only polygons become polygon_fence rules")
        for k, poly in enumerate(polys):
            if any(_local(e.tag) == "innerBoundaryIs" for e in poly.iter()):
                raise ConversionError(
                    f"placemark {base!r}: polygon {k} has a hole (innerBoundaryIs). "
                    f"A polygon_fence has none; split the zone into hole-free polygons")
            outer = [e for e in poly.iter() if _local(e.tag) == "outerBoundaryIs"]
            coords = [e for o in outer for e in o.iter() if _local(e.tag) == "coordinates"]
            if not coords or not (coords[0].text or "").strip():
                raise ConversionError(f"placemark {base!r}: polygon {k} has no coordinates")
            tuples = [t.split(",") for t in coords[0].text.split()]
            ring = _ring(tuples, f"placemark {base!r} polygon {k}")
            zid = _slug(base) if len(polys) == 1 else f"{_slug(base)}-{k + 1}"
            zones.append({"id": zid, "vertices": ring, "properties": {}})
    if not zones:
        raise ConversionError("the KML holds no Placemark with a Polygon")
    return zones


# --------------------------------------------------------------------------- #
# zones -> rules -> policy
# --------------------------------------------------------------------------- #

def rules_from_zones(zones: list[dict], defaults: dict) -> list[dict]:
    """Grant-form polygon_fence rules. Feature properties override defaults."""
    seen: dict[str, int] = {}
    rules = []
    for z in zones:
        if z["id"] in seen:
            raise ConversionError(
                f"two features are both named {z['id']!r}; rule ids must be unique "
                f"(audit records and hot-apply address rules by id)")
        seen[z["id"]] = 1
        p = {**defaults, **{k: v for k, v in z["properties"].items()
                            if k in DSL_PROPERTIES}}
        # A band not given by the feature or the command line is left out,
        # so the DSL's geometry-block default applies (0-200 m, the
        # reference's) - not a third default invented here. Until 2026-10-07
        # this wrote 120 m, undocumented, and an imported zone ended there.
        geometry = {"vertices": z["vertices"]}
        for k in ("altitude_floor_m", "altitude_ceiling_m"):
            if k in p:
                geometry[k] = float(p.pop(k))
        geometry["altitude_ref"] = p.pop("altitude_ref", "AGL")
        rule = {"id": z["id"], "type": "polygon_fence",
                "constraint_type": p.pop("constraint_type", "hard"),
                "scope": p.pop("scope", "global"),
                "priority": p.pop("priority", "P0"),
                "layer": p.pop("layer", "site"),
                "violation_action": p.pop("violation_action", "project_fix"),
                "geometry": geometry}
        if "margin_m" in p:
            rule["margin_m"] = float(p.pop("margin_m"))
        rules.append(rule)
    return rules


def read_zones(path: Path) -> list[dict]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".kml":
        return zones_from_kml(text)
    return zones_from_geojson(json.loads(text))


def convert(path: str | Path, policy_id: str, version: str = "0.1.0",
            defaults: dict | None = None,
            origin: tuple[float, float] | None = None) -> dict:
    """A GeoJSON / KML file -> a grant-form policy document, validated.
    `origin` (lat, lon) states the take-off point, which flight needs."""
    p = Path(path)
    rules = rules_from_zones(read_zones(p), dict(defaults or {}))
    doc = {"policy_id": policy_id, "version": version, "generation": 0,
           "constraints": rules}
    if origin is not None:
        doc["origin"] = {"lat": float(origin[0]), "lon": float(origin[1])}
    pol = policy_from_raw(doc, source=str(p), runtime=False)   # never emit what will not load
    if origin is not None and pol.start_conflicts():
        # `--origin` states where the vehicle takes off (the SITL rail puts
        # its start, home, at the frame origin). A take-off point inside a hard zone this
        # command creates would fly a policy that starts in breach.
        raise ConversionError(
            f"--origin {origin[0]},{origin[1]} is inside a zone being converted: "
            + "; ".join(pol.start_conflicts())
            + ". State the take-off point, outside every keep-out zone")
    return doc


# --------------------------------------------------------------------------- #
# ingest
# --------------------------------------------------------------------------- #

def _is_geojson(obj) -> bool:
    return isinstance(obj, dict) and obj.get("type") in (
        "FeatureCollection", "Feature", "Polygon", "MultiPolygon", "GeometryCollection")


def _is_nfz_payload(obj) -> bool:
    """The body of guardrail/api.py's POST /nfz (NfzRequest) or POST
    /dynamic_nfz: a zone with an id and vertices, and no `type`."""
    return (isinstance(obj, dict) and "vertices" in obj and "id" in obj
            and "type" not in obj and "constraints" not in obj)


# What a zone body may carry: a dynamic_nfz's own fields (POST /nfz's five
# are a subset).
_ZONE_PAYLOAD_KEYS = frozenset(DynamicNFZ.model_fields) - {"type"}


def rules_from_payload(obj, defaults: dict | None = None) -> list[dict] | None:
    """New rules carried by a source, or None when the source is a whole policy.

    A zone body (POST /nfz or POST /dynamic_nfz) becomes a dynamic_nfz. Since
    2026-10-07 guardrail/api.py spawns POST /nfz as one, because the grant's
    mid-flight update model locks polygon_fence at mission start; until then
    this tool appended a polygon_fence, which is no longer what the live
    policy holds after that call."""
    if _is_geojson(obj):
        return rules_from_zones(zones_from_geojson(obj), dict(defaults or {}))
    if _is_nfz_payload(obj):
        extra = set(obj) - _ZONE_PAYLOAD_KEYS
        if extra:
            raise ConversionError(f"zone payload (POST /nfz, POST /dynamic_nfz) has "
                                  f"unknown keys {sorted(extra)}")
        return [{"type": "dynamic_nfz", **obj}]
    if isinstance(obj, dict) and "type" in obj and "constraints" not in obj:
        return [obj]                                    # one DSL rule (a patch)
    if isinstance(obj, list):
        return [r for o in obj for r in (rules_from_payload(o, defaults) or [])]
    return None


def fetch(url: str, timeout_s: float = 10.0):
    """GET a REST endpoint and parse its body: JSON / YAML, or KML (a body that
    starts with "<"). An HTTP error or an unreadable body raises
    ConversionError naming the URL - never an empty policy."""
    import urllib.error
    import urllib.request
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as resp:
            text = resp.read().decode("utf-8")
    except (urllib.error.URLError, OSError, UnicodeDecodeError) as exc:
        raise ConversionError(f"{url}: could not read the endpoint ({exc})") from None
    if text.lstrip().startswith("<"):
        return {"__kml__": text}
    obj = parse_policy_text(text, url)
    if obj is None:
        raise ConversionError(f"{url}: the endpoint returned an empty body")
    return obj


def ingest(source, base: Policy | None = None, defaults: dict | None = None,
           runtime: bool = False) -> Policy:
    """One source -> a validated Policy (see the module docstring).

    `source` is a path, "-" for stdin, an http(s) URL, or an already-parsed
    payload."""
    if isinstance(source, str) and source.startswith(("http://", "https://")):
        obj, where = fetch(source), source
    elif isinstance(source, (str, Path)) and str(source) != "-":
        p = Path(source)
        suffix = p.suffix.lower()
        if suffix == ".kml":
            obj = {"__kml__": p.read_text(encoding="utf-8")}
        elif suffix in (".yaml", ".yml"):
            obj = parse_policy_text(p.read_text(encoding="utf-8"), str(p))
        else:
            obj = json.loads(p.read_text(encoding="utf-8"))
        where = str(p)
    elif str(source) == "-":
        obj, where = parse_policy_text(sys.stdin.read(), "<stdin>"), "<stdin>"
    else:
        obj, where = source, "<payload>"

    if isinstance(obj, dict) and "__kml__" in obj:
        new = rules_from_zones(zones_from_kml(obj["__kml__"]), dict(defaults or {}))
    else:
        new = rules_from_payload(obj, defaults)

    if new is None:                                     # a whole policy
        if base is not None:
            raise ConversionError(f"{where} is a whole policy; --base is for "
                                  f"rules to append, not for two policies")
        return policy_from_raw(obj, source=where, runtime=runtime)
    if base is None:
        raise ConversionError(
            f"{where} carries {len(new)} rule(s) but no policy to add them to; "
            f"pass --base (a payload alone has no policy_id, version or frame)")
    geo_new = bool(geo_points(new))
    if geo_new and base.origin is None and not base.is_geographic and any(
            getattr(c, "vertices", None) or getattr(c, "centerline", None)
            for c in base.constraints):
        raise ConversionError(
            f"{where}: geographic zones cannot be placed in {base.policy_id}'s "
            f"frame - it is written in local metres and states no origin. Add "
            f"`origin: {{lat, lon}}` to the base policy first")
    raw = base.canonical_ir()
    raw["constraints"] = raw["constraints"] + new
    raw["generation"] = base.generation + len(new)      # one bump per rule, as hot_apply
    return policy_from_raw(raw, source=f"{base.policy_id} + {where}", runtime=runtime)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def _defaults(a) -> dict:
    d = {}
    for k in ("priority", "layer", "scope", "constraint_type", "violation_action"):
        if getattr(a, k, None) is not None:
            d[k] = getattr(a, k)
    if a.floor is not None:
        d["altitude_floor_m"] = a.floor
    if a.ceiling is not None:
        d["altitude_ceiling_m"] = a.ceiling
    return d


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python tools/geo_to_policy.py")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("convert", "ingest"):
        sp = sub.add_parser(name)
        sp.add_argument("source")
        sp.add_argument("-o", "--out", required=(name == "ingest"))
        sp.add_argument("--priority", choices=("P0", "P1", "P2"))
        sp.add_argument("--layer", choices=("regulation", "site", "mission"))
        sp.add_argument("--scope", choices=("global", "mission", "segment", "waypoint"))
        sp.add_argument("--constraint-type", dest="constraint_type",
                        choices=("hard", "soft"))
        sp.add_argument("--action", dest="violation_action")
        sp.add_argument("--floor", type=float, default=None)
        sp.add_argument("--ceiling", type=float, default=None)
    c = sub.choices["convert"]
    c.add_argument("--policy-id", required=True)
    c.add_argument("--version", default="0.1.0")
    c.add_argument("--origin", default=None,
                   help="LAT,LON of the take-off point (needed to fly the result)")
    i = sub.choices["ingest"]
    i.add_argument("--base", default=None)
    i.add_argument("-m", "--changelog", default="ingest")
    i.add_argument("--unsigned", action="store_true")
    a = ap.parse_args(argv)

    try:
        if a.cmd == "convert":
            org = tuple(float(v) for v in a.origin.split(",")) if a.origin else None
            doc = convert(a.source, a.policy_id, a.version, _defaults(a), origin=org)
            text = yaml.safe_dump(doc, sort_keys=False)
            if a.out:
                Path(a.out).write_text(text, encoding="utf-8", newline="\n")
                print(f"{a.out}: {len(doc['constraints'])} polygon_fence rule(s) "
                      f"from {a.source}")
            else:
                print(text, end="")
            return 0
        from guardrail.bundle import check_bundle, write_bundle
        base = load_policy(a.base, runtime=False) if a.base else None
        pol = ingest(a.source, base, _defaults(a))
        out = write_bundle(pol, a.out, a.changelog,
                           signer=None if a.unsigned else "auto")
        chk = check_bundle(out)
        print(f"{out}: {pol.policy_id} v{pol.version} gen {pol.generation}, "
              f"{len(pol.constraints)} rules")
        print(f"  policy_hash {pol.policy_hash}")
        print(f"  signature   {chk.signature}")
        gaps = pol.flight_problems()
        if gaps:
            print(f"  NOT FLYABLE: {'; '.join(gaps)}")
        return 0
    except (ValueError, OSError) as exc:
        print(f"REFUSED  {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
