"""Lazy WGS84 -> local ENU (meters) projection.

The DSL never carries projected coordinates (policy-dsl.md, "Coordinate frame");
ENU/NED projection is computed lazily by the consumers. For the Phase-1 slice we
use an equirectangular approximation around a per-bundle origin — accurate to well
under a metre over the few-hundred-metre operating area of a single mission, which
is all the Shield's lateral projection needs.
"""

from __future__ import annotations

import math

_EARTH_R = 6_378_137.0  # WGS84 equatorial radius, metres


class LocalProjection:
    """Equirectangular projector anchored at ``(origin_lat, origin_lon)``."""

    __slots__ = ("origin_lat", "origin_lon", "_cos_lat")

    def __init__(self, origin_lat: float, origin_lon: float) -> None:
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self._cos_lat = math.cos(math.radians(origin_lat))

    def to_xy(self, lat: float, lon: float) -> tuple[float, float]:
        """Return ``(east_m, north_m)`` relative to the origin."""
        east = math.radians(lon - self.origin_lon) * _EARTH_R * self._cos_lat
        north = math.radians(lat - self.origin_lat) * _EARTH_R
        return (east, north)

    def to_latlon(self, east_m: float, north_m: float) -> tuple[float, float]:
        lat = self.origin_lat + math.degrees(north_m / _EARTH_R)
        lon = self.origin_lon + math.degrees(east_m / (_EARTH_R * self._cos_lat))
        return (lat, lon)
