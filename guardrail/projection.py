"""WGS84 lat/lon -> the local metre frame the Shield works in.

WHY THIS EXISTS

The grant (Policy DSL page, "Locked design choices") is explicit: "WGS84 latitude
/ longitude is canonical ... ENU / NED projection is computed lazily by the
Prefix Compiler and Safety Shield from the WGS84 + AGL fields -- the DSL itself
never carries projected coordinates."

Until 2026-10-06 this module projected a geographic policy ONCE at load and the
policy kept only the metres, so the stored IR and its hash held projected
coordinates - the opposite of the rule above. Since then a geographic point keeps
its lat/lon (that is what the canonical IR stores and hashes) and gains x/y here,
at load, for the Shield. Metre-authored policies are untouched: they are the
legacy form and still load and hash exactly as before.

THE FRAME ORIGIN

A policy may state `origin: {lat, lon}`. The grant's own form has no such field,
and the reference implementation (kuanting-vla-uav-guardrail, policy_dsl/ir.py,
`_choose_origin`) derives the local plane's origin instead: the mean of the FIRST
polygon_fence's vertices. `derive_frame_origin` does exactly that, with the
same float arithmetic, so the two implementations put a given lat/lon at the
same metres. Where the reference falls back to (0, 0) - a policy with no polygon
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

# WGS84 equatorial radius in metres, matching the reference implementation so
# the two agree on where a given lat/lon lands.
_EARTH_R = 6_378_137.0

# A point that carries BOTH lat/lon and x/y (a model_dump being validated again)
# must agree with itself to this many metres, or it is refused: it would be a
# point in two places at once.
CONSISTENCY_TOL_M = 1e-6


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


def _is_geo_point(node) -> bool:
    """A point written in WGS84: a mapping with a non-null lat AND lon.

    Non-null, not merely present: a model_dump() of a metre point carries
    `lat: None, lon: None`, and that point is not geographic."""
    return (isinstance(node, dict) and node.get("lat") is not None
            and node.get("lon") is not None)


def _is_metric_point(node) -> bool:
    return (isinstance(node, dict) and not _is_geo_point(node)
            and "x" in node and "y" in node)


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


def _mean(pts: list[tuple[float, float]]) -> tuple[float, float]:
    # The reference's arithmetic, term for term:
    #   sum(v.lat for v in verts) / len(verts)
    # A different summation order can differ in the last bit, which moves every
    # projected metre by a nanometre and breaks bit-for-bit agreement.
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def derive_frame_origin(rules) -> tuple[float, float] | None:
    """The local frame's origin for a policy that does not state one.

    The reference rule first: the mean of the first polygon_fence's geographic
    vertices. Then the first rule with any geographic point (a corridor, a
    circle's centre), where the reference would use (0, 0). None for a policy
    with no geographic point at all - it is in metres and needs no frame.

    `rules` is the constraints list, authored (raw YAML, `geometry:` blocks and
    all) or dumped from the model; both give the same answer, because each rule
    type has exactly one point collection and its order is kept."""
    if not isinstance(rules, list):
        return None
    for r in rules:
        if isinstance(r, dict) and r.get("type") == "polygon_fence":
            pts = geo_points(r)
            if pts:
                return _mean(pts)
    for r in rules:
        pts = geo_points(r)
        if pts:
            return _mean(pts)
    return None


def _num(v, what: str) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"{what} {v!r} is not a number") from None
    if not math.isfinite(f):
        raise ValueError(f"{what} {v!r} is not finite")
    return f


def project_raw(raw: dict) -> dict:
    """Give every {lat, lon} point in a raw policy dict its {x, y} metres.

    Runs BEFORE pydantic validation (models.Policy calls it from a before-
    validator), so the constraint models see x/y for every point while lat/lon
    stay on the point - they are what the canonical IR stores.

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
            x, y = proj.to_local(lat, lon)
            if node.get("x") is not None or node.get("y") is not None:
                if node.get("x") is None or node.get("y") is None:
                    raise ValueError(f"point {node} gives one of x/y beside lat/lon; "
                                     f"one point cannot mix frames")
                dx = abs(_num(node["x"], "x") - x)
                dy = abs(_num(node["y"], "y") - y)
                if dx > CONSISTENCY_TOL_M or dy > CONSISTENCY_TOL_M:
                    raise ValueError(
                        f"point {node} carries lat/lon and x/y that disagree by "
                        f"({dx:.3g}, {dy:.3g}) m in this policy's frame; one "
                        f"point cannot mix frames")
            return {**node, "x": x, "y": y}
        return {k: _walk(v) for k, v in node.items()}

    out = dict(raw)
    out["constraints"] = _walk(cons)
    return out
