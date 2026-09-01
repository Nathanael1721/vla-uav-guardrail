"""WGS84 lat/lon -> the local metre frame the Shield works in.

WHY THIS EXISTS

`policy-dsl.md` is explicit that the DSL's canonical frame is WGS84: "latitude /
longitude is canonical... the DSL itself never carries projected coordinates."
Every policy in this repository is written in local metres instead, and
`models.py` has carried a note admitting it since the first version: "The real
grant DSL uses WGS84 lat/lon; local meters keeps the prototype simple... Swap
later = loader change only."

This is that loader change, and it is ADDITIVE. Existing metre-based policies
are untouched and keep working; a policy may now write geographic coordinates
instead, and they are projected once at load time. Rewriting the project to be
lat/lon internally would invalidate every delivered artefact, every policy file
and every recorded flight, in exchange for no new evidence.

THE AXIS ORDER, WHICH IS THE EASY THING TO GET WRONG

The reference implementation's `LocalProjection.to_xy` returns **(east, north)**.
This project's frame is **x = North, y = East** (`models.py`, `State`). The two
are transposed, so a straight copy of the reference call would put every
constraint at ninety degrees to where the policy author meant it - a no-fly zone
neatly rotated off the thing it was meant to protect, with nothing in the schema
to complain. `to_local()` below returns (x_north, y_east) and says so in its
name and its test.

The projection is equirectangular about a per-policy origin: sub-metre over the
few-hundred-metre area of one mission, which is the scale everything here works
at. It is not a survey-grade projection and is not offered as one.
"""
from __future__ import annotations

import math

# WGS84 equatorial radius in metres, matching the reference implementation so
# the two agree on where a given lat/lon lands.
_EARTH_R = 6_378_137.0


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


def project_raw(raw: dict) -> dict:
    """Rewrite any {lat, lon} points in a raw policy dict into {x, y} metres.

    Runs BEFORE pydantic validation, so the models never learn about geographic
    coordinates and the discriminated union stays exactly as it was. A point is
    geographic if it carries `lat`; mixing the two forms inside one policy is
    rejected rather than guessed at, because a silently half-projected fence is
    a fence in the wrong place.

    The origin comes from `origin: {lat, lon}` at the top of the policy. Without
    it, geographic coordinates are refused - defaulting to (0, 0) would put a
    Taipei no-fly zone in the Gulf of Guinea, and it would validate cleanly.
    """
    if not isinstance(raw, dict):
        return raw

    def _has_latlon(node) -> bool:
        if isinstance(node, dict):
            if "lat" in node and "lon" in node:
                return True
            return any(_has_latlon(v) for v in node.values())
        if isinstance(node, list):
            return any(_has_latlon(v) for v in node)
        return False

    if not _has_latlon(raw.get("constraints", [])):
        return raw                       # pure metre policy: untouched

    org = raw.get("origin")
    if not isinstance(org, dict) or "lat" not in org or "lon" not in org:
        raise ValueError(
            "policy uses lat/lon coordinates but declares no `origin: {lat, lon}`. "
            "Refusing to guess: an assumed origin places every rule somewhere "
            "plausible-looking and completely wrong.")
    proj = LocalProjection(float(org["lat"]), float(org["lon"]))

    def _walk(node):
        if isinstance(node, list):
            return [_walk(v) for v in node]
        if not isinstance(node, dict):
            return node
        if "lat" in node and "lon" in node:
            if "x" in node or "y" in node:
                raise ValueError(
                    f"point {node} carries both lat/lon and x/y; one policy "
                    f"cannot mix frames")
            x, y = proj.to_local(float(node["lat"]), float(node["lon"]))
            keep = {k: v for k, v in node.items() if k not in ("lat", "lon")}
            return {**keep, "x": x, "y": y}
        return {k: _walk(v) for k, v in node.items()}

    out = dict(raw)
    out["constraints"] = _walk(raw.get("constraints", []))
    return out
