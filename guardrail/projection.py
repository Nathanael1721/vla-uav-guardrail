"""WGS84 lat/lon -> the local metre frame the Shield works in.

WHY THIS EXISTS

The grant (Policy DSL page, "Locked design choices") is explicit: "WGS84 latitude
/ longitude is canonical ... ENU / NED projection is computed lazily by the
Prefix Compiler and Safety Shield from the WGS84 + AGL fields -- the DSL itself
never carries projected coordinates."

Until 2026-10-06 this module projected a geographic policy ONCE at load and the
policy kept only the metres, so the stored IR and its hash held projected
coordinates - the opposite of the rule above. From then until 2026-11-03 a
geographic point kept its lat/lon (what the canonical IR stores and hashes) and
was given x/y HERE, AT LOAD, for every point of the policy. Since 2026-11-03
(IR schema 3.1.0, WP1-14) nothing is projected at load: a WGS84 point is bound
to its policy's frame origin when the policy validates, and its metres are
computed the first time the Shield or the compiler reads `point.x` / `point.y`
(models.XY), by `local_metres` below, which caches them per origin. Metre-
authored policies are untouched: they are the legacy form and still load and
hash exactly as before; since 2026-11-03 no shipped policy is written in it
(tools/convert_policies_wgs84.py), and since 2026-11-17 a metre point an
event writes into a policy with a frame (REST hot-apply, the Shield's zone
edits, the harness's spawned zones, a lock recipe rebuilt by bundle.derive,
tools/geo_to_policy.py ingest) is converted at ingest (`metres_to_wgs84`,
models.rule_in_wgs84, `raw_points_in_wgs84`), so in a policy with a frame the
metre form is not STORED, and the converted point is bound to the frame,
so the live Shield enforces the micrometre view of its stored lat/lon (the
metres given, on the micrometre grid; else within 0.6 um), the same floats
a reload of the snapshot reads (models.rule_in_wgs84). Since 2026-11-18 the
Mission Control GUI's policy writer (demo/gui_policy.py) converts the zones
drawn on its map the same way, about the base policy's origin; until then it
rewrote policies/gui_policy.yaml in metres with no origin. The legacy form is
labelled where it is read: models.Policy.point_form, and the flight loader's
record (bundle.load_for_flight `point_form` / `point_note`);
tests/test_wgs84_ingest_writers.py checks every writer.

THE LOCAL VIEW IS ROUNDED TO THE MICROMETRE

`local_metres` rounds each coordinate to LOCAL_QUANTUM_M (1 um). The
equirectangular projection is sub-metre over a mission's area, so the rounding
is far below anything it can resolve; what it buys is that the view is a
deterministic, platform-independent function of (lat, lon, origin), and that a
policy converted from metres (lat/lon written from x/y about a stated origin)
gives back EXACTLY the metres it was written in - the Shield's geometry is then
the same floats as before the conversion, not floats a few 1e-10 m away.
`LocalProjection.to_local` itself is not rounded (the rails convert vehicle
positions with it); the two differ by at most half a micrometre. The hash forms
of earlier schemes (v2 and the 16-hex eras) hashed the UNROUNDED metres of
their day, and are reproduced with `LocalProjection.to_local` (models, the
"era" dump).

THE FRAME ORIGIN

A policy may state `origin: {lat, lon}`. The grant's own form has no such field,
and the reference implementation (kuanting-vla-uav-guardrail, policy_dsl/ir.py,
`_choose_origin`) derives the local plane's origin instead: the mean of the FIRST
polygon_fence's vertices. `derive_frame_origin` does exactly that, with the
same float arithmetic, so the two implementations put a given lat/lon at the
same metres: `LocalProjection.to_local` to the last bit, the policy's rounded
view (above) to within half a micrometre. Where the reference falls back to (0, 0) - a policy with no polygon
- this module uses the first rule that has any geographic point instead: (0, 0)
would place the frame in the Gulf of Guinea, thousands of kilometres from every
rule, and the equirectangular projection is only good near its origin.

The frame matters at the Shield's interface: `State` is metres north/east of the
frame origin. A rail flying a geographic policy must express the vehicle's
position in `Policy.frame_origin`'s frame (LocalProjection(*policy.frame_origin)).

THE AXIS ORDER, WHICH IS THE EASY THING TO GET WRONG

The reference implementation's `LocalProjection.to_xy` returns **(east, north)**.
This project's frame is **x = North, y = East** (`models.py`, `State`). The two
are transposed, so a straight copy of the reference call would put every
constraint at ninety degrees to where the policy author meant it - a no-fly zone
neatly rotated off the thing it was meant to protect, with nothing in the schema
to complain. `to_local()` below returns (x_north, y_east) and says so in its
name and its test.

The projection is equirectangular about the frame origin: sub-metre over the
few-hundred-metre area of one mission, which is the scale everything here works
at. It is not a survey-grade projection and is not offered as one.
"""
from __future__ import annotations

import math
from functools import lru_cache

# WGS84 equatorial radius in metres, matching the reference implementation so
# the two agree on where a given lat/lon lands.
_EARTH_R = 6_378_137.0

# A point that carries BOTH lat/lon and x/y (a model_dump being validated again)
# must agree with itself to this many metres, or it is refused: it would be a
# point in two places at once.
CONSISTENCY_TOL_M = 1e-6

# The policy's local view is the projection rounded to this (see the module
# docstring): 6 decimal places of a metre.
LOCAL_QUANTUM_DIGITS = 6
LOCAL_QUANTUM_M = 10.0 ** -LOCAL_QUANTUM_DIGITS


class LocalProjection:
    """Equirectangular projector anchored at (origin_lat, origin_lon)."""

    __slots__ = ("origin_lat", "origin_lon", "_cos_lat")

    def __init__(self, origin_lat: float, origin_lon: float) -> None:
        if not -90.0 <= origin_lat <= 90.0:
            raise ValueError(f"origin lat {origin_lat} out of range")
        if not -180.0 <= origin_lon <= 180.0:
            raise ValueError(f"origin lon {origin_lon} out of range")
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self._cos_lat = math.cos(math.radians(origin_lat))

    def to_local(self, lat: float, lon: float) -> tuple[float, float]:
        """(x_north_m, y_east_m) - THIS project's axis order, not the reference's."""
        north = math.radians(lat - self.origin_lat) * _EARTH_R
        east = math.radians(lon - self.origin_lon) * _EARTH_R * self._cos_lat
        return (north, east)

    def to_latlon(self, x_north_m: float, y_east_m: float) -> tuple[float, float]:
        lat = self.origin_lat + math.degrees(x_north_m / _EARTH_R)
        lon = self.origin_lon + math.degrees(
            y_east_m / (_EARTH_R * self._cos_lat))
        return (lat, lon)


@lru_cache(maxsize=256)
def projection_for(origin_lat: float, origin_lon: float) -> LocalProjection:
    """The projector of one frame origin, built once per origin."""
    return LocalProjection(float(origin_lat), float(origin_lon))


def quantize_m(v: float) -> float:
    """`v` metres rounded to LOCAL_QUANTUM_M; -0.0 comes back as 0.0, so a
    point on an axis serialises the same whichever side it was rounded from."""
    return round(v, LOCAL_QUANTUM_DIGITS) + 0.0


@lru_cache(maxsize=65536)
def local_metres(origin: tuple[float, float], lat: float,
                 lon: float) -> tuple[float, float]:
    """(x_north_m, y_east_m) of a WGS84 point in the local frame of `origin`
    ((lat, lon)), rounded to LOCAL_QUANTUM_M: the policy's local view. Computed
    on first use and cached per (origin, point) - the lazy, per-origin view the
    grant asks for (models.XY reads it for `x` / `y`)."""
    n, e = projection_for(origin[0], origin[1]).to_local(lat, lon)
    return (quantize_m(n), quantize_m(e))


# A metre point converted to WGS84 at ingest (metres_to_wgs84) is written to
# 12 decimal places of a degree (~0.1 um of latitude), as
# tools/convert_policies_wgs84.py writes the stored policies.
WGS84_DECIMALS = 12


def metres_to_wgs84(origin: tuple[float, float], x_north_m: float,
                    y_east_m: float) -> tuple[float, float]:
    """(lat, lon) of the point (x North, y East) metres from `origin` ((lat,
    lon)), rounded to WGS84_DECIMALS: the inverse of `local_metres`. For a
    point on the micrometre grid (every metre value written with at most six
    decimals) `local_metres` gives back exactly (x, y); for any other the
    view is within 0.6 um of (x, y): half the micrometre, plus at most
    0.06 um from writing lat/lon to 12 decimals. Since 2026-11-17 (WP1-14) the
    Shield writes every metre point an event adds this way (models.
    rule_in_wgs84), so a policy with a frame is stored in WGS84 only."""
    lat, lon = projection_for(origin[0], origin[1]).to_latlon(x_north_m, y_east_m)
    return (round(lat, WGS84_DECIMALS), round(lon, WGS84_DECIMALS))


def check_ring(points) -> list[tuple[float, float]]:
    """A ring of points (model points or point dicts) as metres FOR A VALIDITY
    CHECK ONLY (self-intersection, area): metre points as written; a ring of
    WGS84 points projected about its own first vertex, unrounded. Simplicity
    and the sign of the area do not depend on the origin (the projection
    scales each axis by a positive factor), so a ring can be checked before it
    is placed in any frame. Raises on a ring that mixes the two."""
    def ll(p):
        if isinstance(p, dict):
            return (p.get("lat"), p.get("lon"))
        return (getattr(p, "lat", None), getattr(p, "lon", None))
    geo = [ll(p) for p in points]
    kinds = {g[0] is not None and g[1] is not None for g in geo}
    if kinds == {True}:
        proj = LocalProjection(float(geo[0][0]), float(geo[0][1]))
        return [proj.to_local(float(a), float(b)) for a, b in geo]
    if len(kinds) > 1:
        raise ValueError("the ring mixes lat/lon and x/y points; one rule cannot "
                         "mix frames")
    out = []
    for p in points:
        if isinstance(p, dict):
            out.append((float(p["x"]), float(p["y"])))
        else:
            out.append((float(p.x), float(p.y)))
    return out


def _is_geo_point(node) -> bool:
    """A point written in WGS84: a mapping with a non-null lat AND lon.

    Non-null, not merely present: a model_dump() of a metre point carries
    `lat: None, lon: None`, and that point is not geographic."""
    return (isinstance(node, dict) and node.get("lat") is not None
            and node.get("lon") is not None)


def _is_metric_point(node) -> bool:
    return (isinstance(node, dict) and not _is_geo_point(node)
            and "x" in node and "y" in node)


def metre_point_count(node) -> int:
    """How many points under `node` are written in local metres ({x, y}
    without lat/lon), depth first: the projected coordinates the grant's DSL
    never carries (WP1-14). For a writer that must refuse or convert them."""
    if _is_metric_point(node):
        return 1
    if isinstance(node, dict):
        return sum(metre_point_count(v) for v in node.values())
    if isinstance(node, list):
        return sum(metre_point_count(v) for v in node)
    return 0


def xy_beside_latlon_count(node) -> int:
    """How many points under `node` are written in WGS84 AND carry an `x` or
    `y` key beside lat/lon (whatever its value), depth first: a projected
    coordinate written into the document, which the DSL never carries
    (WP1-14, since 2026-11-26). Counted on the RAW document because
    project_raw drops those keys before validation (after checking them
    against the frame), so the validated policy no longer shows them."""
    if _is_geo_point(node) and ("x" in node or "y" in node):
        return 1
    if isinstance(node, dict):
        return sum(xy_beside_latlon_count(v) for v in node.values())
    if isinstance(node, list):
        return sum(xy_beside_latlon_count(v) for v in node)
    return 0


def geo_points(node) -> list[tuple[float, float]]:
    """Every (lat, lon) under `node`, depth first, in document order."""
    out: list[tuple[float, float]] = []

    def walk(n):
        if _is_geo_point(n):
            out.append((float(n["lat"]), float(n["lon"])))
        elif isinstance(n, dict):
            for v in n.values():
                walk(v)
        elif isinstance(n, list):
            for v in n:
                walk(v)
    walk(node)
    return out


def _ordered_sum(xs) -> float:
    """Left-to-right float addition, as the built-in sum() did before Python
    3.12. From 3.12 sum() of floats is compensated (Neumaier, gh-100425) and
    can differ in the last bit: on the reference demo's first polygon the mean
    longitude is 121.53139999999999 on 3.10 and 121.5314 on 3.12, so the
    frame origin - and every projected metre - depended on the interpreter
    (the desktop runs 3.10, the Orin 3.12; found 2026-11-06)."""
    total = 0.0
    for x in xs:
        total += x
    return total


def _mean(pts: list[tuple[float, float]]) -> tuple[float, float]:
    # The reference's arithmetic, term for term, on every Python version:
    #   sum(v.lat for v in verts) / len(verts)   (pre-3.12 summation)
    # A different summation order can differ in the last bit, which moves every
    # projected metre by a nanometre and breaks bit-for-bit agreement.
    return (_ordered_sum(p[0] for p in pts) / len(pts),
            _ordered_sum(p[1] for p in pts) / len(pts))


def ring_order(keys: list[tuple[float, float]]) -> list[int]:
    """The index order that puts a ring of stored coordinate pairs into its
    canonical order (models.canonical_ring_order, IR schema 3.0.0): positive
    signed area over the pairs, starting at the lexicographically smallest
    pair (on ties, the smallest rotation). A ring of fewer than three
    points, or with no area, keeps its order."""
    n = len(keys)
    idx = list(range(n))
    if n < 3:
        return idx
    area2 = sum(keys[i][0] * keys[(i + 1) % n][1] - keys[(i + 1) % n][0] * keys[i][1]
                for i in range(n))
    if area2 == 0:
        return idx
    if area2 < 0:
        idx = idx[::-1]
    ks = [keys[i] for i in idx]
    low = min(ks)
    best = min((i for i, k in enumerate(ks) if k == low), key=lambda i: ks[i:] + ks[:i])
    return idx[best:] + idx[:best]


def derive_frame_origin(rules) -> tuple[float, float] | None:
    """The local frame's origin for a policy that does not state one.

    The reference rule first: the mean of the first polygon_fence's geographic
    vertices. Then the first rule with any geographic point (a corridor, a
    circle's centre), where the reference would use (0, 0). None for a policy
    with no geographic point at all - it is in metres and needs no frame.

    `rules` is the constraints list, authored (raw YAML, `geometry:` blocks and
    all) or dumped from the model; both give the same answer, because each rule
    type has exactly one point collection.

    The fence's vertices are averaged in their CANONICAL ring order (ring_order;
    since IR schema 3.0.0 the IR stores that order), so the origin - to the
    last bit - does not depend on the order the ring was written in, and a
    policy reloaded from its bundle lands on the same metres. For a ring
    already in canonical order, which every reference document is, this is
    the reference's own sum, term for term."""
    if not isinstance(rules, list):
        return None
    for r in rules:
        if isinstance(r, dict) and r.get("type") == "polygon_fence":
            pts = geo_points(r)
            if pts:
                return _mean([pts[i] for i in ring_order(pts)])
    for r in rules:
        pts = geo_points(r)
        if pts:
            return _mean(pts)
    return None


def raw_frame_origin(raw: dict) -> tuple[float, float] | None:
    """The frame origin of a RAW policy dict - authored YAML or a policy dump
    (policy_g*.json) - without validating it: its stated `origin`, else the
    one `derive_frame_origin` gives; None for a policy written in metres.
    For tools that draw or measure a policy's geometry (2026-11-04)."""
    o = raw.get("origin") if isinstance(raw, dict) else None
    if isinstance(o, dict) and o.get("lat") is not None and o.get("lon") is not None:
        return (_num(o["lat"], "origin lat"), _num(o["lon"], "origin lon"))
    return derive_frame_origin(raw.get("constraints", []) if isinstance(raw, dict) else [])


def raw_point_metres(point: dict, origin) -> tuple[float, float]:
    """A raw point's (x_north_m, y_east_m) in the frame of `origin`: as
    written when it carries x/y (a metre rule, e.g. one hot-applied into a
    WGS84 policy), else its lat/lon through `local_metres` - the Shield's
    own view of it. Since IR schema 3.1.0 a WGS84 point is dumped as lat/lon
    only, so a tool that read point["x"] stopped with KeyError on it."""
    if point.get("x") is not None and point.get("y") is not None:
        return _num(point["x"], "x"), _num(point["y"], "y")
    if origin is None or point.get("lat") is None or point.get("lon") is None:
        raise ValueError(f"a point with neither x/y nor a placeable lat/lon: {point}")
    return local_metres((float(origin[0]), float(origin[1])),
                        _num(point["lat"], "lat"), _num(point["lon"], "lon"))


def raw_points_in_wgs84(node, origin: tuple[float, float]):
    """A copy of raw rule(s) `node` with every metre point {x, y} written as
    {lat, lon} about `origin` (metres_to_wgs84); everything else as it was.
    The raw-dict twin of models.rule_in_wgs84, for a tool that appends rules
    to a policy with a frame (tools/geo_to_policy.py ingest, 2026-11-17)."""
    if isinstance(node, list):
        return [raw_points_in_wgs84(v, origin) for v in node]
    if not isinstance(node, dict):
        return node
    if _is_metric_point(node):
        lat, lon = metres_to_wgs84(origin, _num(node["x"], "x"), _num(node["y"], "y"))
        return {**{k: v for k, v in node.items() if k not in ("x", "y", "lat", "lon")},
                "lat": lat, "lon": lon}
    return {k: raw_points_in_wgs84(v, origin) for k, v in node.items()}


def _num(v, what: str) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"{what} {v!r} is not a number") from None
    if not math.isfinite(f):
        raise ValueError(f"{what} {v!r} is not finite")
    return f


def project_raw(raw: dict) -> dict:
    """Check the frames of a raw policy dict before validation; project nothing.

    Runs BEFORE pydantic validation (models.Policy calls it from a before-
    validator). Until 2026-11-03 it gave every {lat, lon} point its {x, y}
    metres here, at load; since then a WGS84 point reaches the model as
    lat/lon only and its metres are derived on first use (models.XY,
    `local_metres`). A point that arrives with BOTH (a model_dump of an earlier
    version, a REST body) is checked against this policy's frame and its x/y
    dropped: they are a view, not part of the point.

    Refused, never guessed:
      * one rule whose points mix frames (some lat/lon, some x/y): a silently
        half-projected fence is a fence in the wrong place;
      * a point carrying lat/lon AND x/y that disagree (more than 1 um);
      * an `origin` that is not {lat, lon} or is out of range.

    Allowed: different rules in different frames. A geographic policy with a
    metre fence hot-applied mid-flight is exactly that, and its bundle has to
    load again. The metre rule is then in the policy's frame (`frame_origin`).
    """
    if not isinstance(raw, dict):
        return raw
    cons = raw.get("constraints", [])
    if not geo_points(cons):
        return raw                       # pure metre policy: untouched

    org = raw.get("origin")
    if org is None:
        lat0, lon0 = derive_frame_origin(cons) or (None, None)
        if lat0 is None:                 # unreachable: geo_points was non-empty
            raise ValueError("geographic points but no frame origin could be derived")
    else:
        if not isinstance(org, dict) or "lat" not in org or "lon" not in org:
            raise ValueError("`origin` must be a mapping {lat, lon}")
        lat0, lon0 = _num(org["lat"], "origin lat"), _num(org["lon"], "origin lon")
    proj = LocalProjection(lat0, lon0)

    if isinstance(cons, list):
        for i, rule in enumerate(cons):
            kinds = set()

            def kind_walk(n):
                if _is_geo_point(n):
                    kinds.add("lat/lon")
                elif _is_metric_point(n):
                    kinds.add("x/y")
                elif isinstance(n, dict):
                    for v in n.values():
                        kind_walk(v)
                elif isinstance(n, list):
                    for v in n:
                        kind_walk(v)
            kind_walk(rule)
            if len(kinds) > 1:
                rid = rule.get("id", f"#{i}") if isinstance(rule, dict) else f"#{i}"
                raise ValueError(
                    f"rule {rid!r} mixes lat/lon and x/y points; one rule cannot "
                    f"mix frames")

    def _walk(node):
        if isinstance(node, list):
            return [_walk(v) for v in node]
        if not isinstance(node, dict):
            return node
        if _is_geo_point(node):
            lat = _num(node["lat"], "lat")
            lon = _num(node["lon"], "lon")
            if node.get("x") is not None or node.get("y") is not None:
                if node.get("x") is None or node.get("y") is None:
                    raise ValueError(f"point {node} gives one of x/y beside lat/lon; "
                                     f"one point cannot mix frames")
                # The one projection made here: of a point that came WITH
                # metres, to check they are this frame's.
                x, y = proj.to_local(lat, lon)
                dx = abs(_num(node["x"], "x") - x)
                dy = abs(_num(node["y"], "y") - y)
                if dx > CONSISTENCY_TOL_M or dy > CONSISTENCY_TOL_M:
                    raise ValueError(
                        f"point {node} carries lat/lon and x/y that disagree by "
                        f"({dx:.3g}, {dy:.3g}) m in this policy's frame; one "
                        f"point cannot mix frames")
            return {k: v for k, v in node.items() if k not in ("x", "y")}
        return {k: _walk(v) for k, v in node.items()}

    out = dict(raw)
    out["constraints"] = _walk(cons)
    return out
