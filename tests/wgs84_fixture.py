"""The tool-side conversion the test fixtures make before they hand a rule
to the Shield (WP1-14, 2026-11-26).

Since that date every Shield event entry (hot_apply, spawn_nfz, move_nfz,
a pre-start fence) takes an absolute position in WGS84 lat/lon only and
refuses one in local metres, naming models.wgs84_from_metres. The fixtures
are laid out in metres, as the stress library's events are, so each converts
its rule about the policy's frame origin first, exactly as our own metre
callers do (guardrail/scenario_events.py `in_wgs84`, guardrail/replay.py
EpisodeRecord.apply_zone, guardrail/harness_service.py rest_body_of_event).
The conversion is projection.metres_to_wgs84, the one the Shield applied at
ingest until then, so every zone, hash and decision below is what it was.

A Shield with no frame origin, or with its comparison switch wgs84_writes
off, gets the rule unchanged: the Shield then refuses (NoFrameOrigin) or
stores it exactly as before, which is what those tests check.

    from wgs84_fixture import geo, geo_ring
    sh.hot_apply(geo(zone, sh))
    sh.move_nfz("z", geo_ring([(0, 30), (4, 30), (4, 34)], sh))

The same date, models.load_policy / policy_from_raw / load_layered refuse a
DOCUMENT whose points are in metres (ProjectedCoordinatesRefused). A fixture
document laid out in metres goes through `wgs84_doc` first, which writes its
points in WGS84 about its frame - its stated `origin`, else the frame it
derived from its lat/lon points, else TEST_ORIGIN - and states that origin,
as tools/convert_policies_wgs84.py did for every shipped policy. The fixtures
use metres on the micrometre grid, so the policy's view of every point is
exactly the metres written and every decision is unchanged; a fixture whose
subject IS the legacy metre form reads it with models.legacy_metre_policy_from_raw.

    pol = policy_from_raw(wgs84_doc(raw))
    path.write_text(wgs84_text(yaml_text))

`rest(api.apply_x, shield, body)` calls a REST handler as a client does since
that date: the body's metres converted tool side first.
"""
from __future__ import annotations

import copy

from guardrail.models import wgs84_from_metres
from guardrail.projection import geo_points, metre_point_count, raw_frame_origin

# The frame a metre-only fixture is placed in: the Project AirSim
# home-geo-point that policies/demo_policy.yaml states.
TEST_ORIGIN = (35.6895, 139.6917)


def _points_before_20261126(shield, pts):
    """guardrail.api._points as it was until 2026-11-26: it passed {x, y}
    (and [x, y]) through, and gave a lat/lon point its frame's metres. Used
    by `rest` for the comparison Shield only (module docstring)."""
    from guardrail.api import ApiError
    from guardrail.projection import local_metres
    out = []
    for p in pts:
        if isinstance(p, (list, tuple)) and len(p) == 2:
            p = {"x": p[0], "y": p[1]}
        if "x" in p and "y" in p:
            out.append({k: v for k, v in p.items() if k in ("x", "y", "lat", "lon")})
            continue
        proj = shield.policy.projection()
        if proj is None:
            raise ApiError(422, "no frame for a lat/lon point; send x/y")
        lat, lon = float(p["lat"]), float(p["lon"])
        x, y = local_metres((proj.origin_lat, proj.origin_lon), lat, lon)
        out.append({"x": x, "y": y, "lat": lat, "lon": lon})
    return out


def rest(fn, shield, *args):
    """Call a guardrail.api handler (`fn(shield, *args)`) as a client does
    since 2026-11-26 (WP1-14): every body / point list in metres is first
    converted tool side about the policy's frame origin (wgs84_from_metres).

    A policy with no frame origin gets the metres unchanged (the endpoint
    refuses them: NoFrameOrigin). The comparison Shield (wgs84_writes off:
    the metre writes of before 2026-11-17) gets them through the REST layer
    as it was before 2026-11-26 - the only way metres still reach a Shield
    over REST - so the WGS84 writes can be compared with what was stored
    then."""
    origin = _origin(shield)
    if origin is not None:
        return fn(shield, *[wgs84_from_metres(a, origin) if isinstance(a, (dict, list))
                            else a for a in args])
    if getattr(shield, "wgs84_writes", True):
        return fn(shield, *args)
    import guardrail.api as api
    real = api._points
    api._points = _points_before_20261126
    try:
        return fn(shield, *args)
    finally:
        api._points = real


def wgs84_text(text: str, origin: tuple[float, float] | None = None) -> str:
    """wgs84_doc for a fixture written as YAML text: the YAML of the
    converted document (unchanged text when it has no metre point)."""
    import yaml
    raw = yaml.safe_load(text)
    doc = wgs84_doc(raw, origin)
    return text if doc is raw else yaml.safe_dump(doc, sort_keys=False)


def wgs84_doc(raw, origin: tuple[float, float] | None = None):
    """A raw policy document with its metre points written as WGS84 lat/lon
    and its frame origin stated (module docstring). A document with no metre
    point comes back as it was; so does anything that is not a mapping."""
    if not isinstance(raw, dict) or not metre_point_count(raw.get("constraints")):
        return raw
    out = copy.deepcopy(raw)
    org = out.get("origin")
    if isinstance(org, dict) and "lat" in org and "lon" in org:
        o = (float(org["lat"]), float(org["lon"]))
    elif geo_points(out.get("constraints")):
        o = raw_frame_origin(out)               # the frame it derived: stated now
    else:
        o = origin or TEST_ORIGIN
    out["origin"] = {"lat": o[0], "lon": o[1]}
    out["constraints"] = wgs84_from_metres(out.get("constraints") or [], o)
    return out


def _origin(shield):
    pol = getattr(shield, "policy", None)
    origin = getattr(pol, "frame_origin", None)
    if origin is None or not getattr(shield, "wgs84_writes", True):
        return None
    return origin


def geo(obj, shield):
    """`obj` (a rule, a REST body or point dicts) with its metre points in
    WGS84 about `shield`'s policy frame origin; unchanged when there is none."""
    origin = _origin(shield)
    return obj if origin is None else wgs84_from_metres(obj, origin)


def geo_ring(ring, shield):
    """A move's ring - (x, y) pairs or {x, y} dicts in metres - as {lat, lon}
    points about `shield`'s policy frame origin; unchanged when there is
    none (the Shield then refuses it, or, with wgs84_writes off, takes the
    metres as before)."""
    if _origin(shield) is None:
        return ring
    pts = [dict(v) if isinstance(v, dict) else {"x": float(v[0]), "y": float(v[1])}
           for v in ring]
    return geo(pts, shield)
