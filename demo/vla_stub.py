"""In-house VLA stub (architecture-constraints.md allows in-house stubs).

Emits a 4-D body-frame action that steers straight at a target waypoint at a
fixed cruise speed. It is deliberately *unsafe*: it knows nothing about the
policy, so a target placed across/along an NFZ makes the stub drive into it —
exactly what the mid-term demo needs the Safety Shield to intercept. No model
weights are involved; the determinism manifest records a fixed stub hash.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from safety_shield import VehicleState
from vlaguard_common import Action4D

STUB_MODEL_HASH = "sha256:vla-stub-v1"


@dataclass(frozen=True)
class Target:
    lat: float
    lon: float
    alt_agl_m: float


def bearing_rad(state: VehicleState, target: Target) -> float:
    """NED heading (0 = North, clockwise positive) from the vehicle to the target."""
    # equirectangular deltas are fine for a heading over a short baseline
    d_north = target.lat - state.lat
    d_east = (target.lon - state.lon) * math.cos(math.radians(state.lat))
    return math.atan2(d_east, d_north)


def stub_action(state: VehicleState, target: Target, cruise_mps: float = 5.0) -> Action4D:
    """Body-frame action heading straight at the target at cruise speed.

    ``vx`` is forward (the vehicle is yawed toward the target by the caller), and
    ``vz`` closes the altitude gap within ~1 s.
    """
    vz = _clamp(target.alt_agl_m - state.alt_agl_m, 3.0)
    return Action4D(vx=cruise_mps, vy=0.0, vz=vz, yaw_rate=0.0)


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))
