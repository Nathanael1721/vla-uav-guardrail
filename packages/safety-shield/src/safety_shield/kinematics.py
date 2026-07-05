"""Trajectory prediction + body<->ENU velocity conversion for the monitor.

The Shield predicts a short look-ahead trajectory from the candidate 4-D action
(safety-shield.md: 5 s horizon at 10 Hz = 50 future poses) and checks it against
the IR. Prediction happens in the IR's local ENU metre plane so geometry queries
are cheap.

``yaw_rad`` is an NED heading (0 = North, clockwise positive), matching
``vlaguard_common.frames.body_to_local_ned``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from vlaguard_common import Action4D


@dataclass(frozen=True)
class VehicleState:
    lat: float
    lon: float
    alt_agl_m: float
    yaw_rad: float


@dataclass(frozen=True)
class PredictedPose:
    t_s: float
    east_m: float
    north_m: float
    alt_agl_m: float


def body_to_enu(action: Action4D, yaw_rad: float) -> tuple[float, float, float]:
    """Body-frame velocity -> ``(v_east, v_north, v_up)`` in m/s."""
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    v_north = action.vx * cos_y - action.vy * sin_y
    v_east = action.vx * sin_y + action.vy * cos_y
    return (v_east, v_north, action.vz)


def enu_to_body(v_east: float, v_north: float, v_up: float, yaw_rad: float) -> Action4D:
    """Inverse of :func:`body_to_enu` (yaw_rate must be supplied separately)."""
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    vx = v_north * cos_y + v_east * sin_y
    vy = -v_north * sin_y + v_east * cos_y
    return Action4D(vx=vx, vy=vy, vz=v_up, yaw_rate=0.0)


def predict(
    state: VehicleState,
    action: Action4D,
    origin_xy: tuple[float, float],
    horizon_s: float = 5.0,
    rate_hz: float = 10.0,
) -> list[PredictedPose]:
    """Constant-velocity forward prediction in the local ENU plane.

    ``origin_xy`` is the current vehicle position already projected to ``(east, north)``
    by the caller (it owns the IR's projection).
    """
    ve, vn, vu = body_to_enu(action, state.yaw_rad)
    e0, n0 = origin_xy
    n_steps = int(round(horizon_s * rate_hz))
    dt = 1.0 / rate_hz
    poses = []
    for k in range(1, n_steps + 1):
        t = k * dt
        poses.append(
            PredictedPose(
                t_s=t,
                east_m=e0 + ve * t,
                north_m=n0 + vn * t,
                alt_agl_m=state.alt_agl_m + vu * t,
            )
        )
    return poses
