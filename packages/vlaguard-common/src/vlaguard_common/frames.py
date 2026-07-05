"""The 4-D action contract and the single body-frame -> local-NED boundary.

From ``docs/01-context/architecture-constraints.md`` the action space is::

    a = (vx, vy, vz, yaw_rate)    # body-frame velocities + yaw rate

This is the Safety Shield's *input* contract. Repair operators act on this body
frame; the conversion to local-NED happens at exactly one place — the MAVLink
adapter — so projection operators never see NED. This module is that single
boundary.
"""

from __future__ import annotations

import math

from pydantic import BaseModel


class Action4D(BaseModel):
    """A body-frame velocity setpoint plus yaw rate.

    Units: linear m/s, ``yaw_rate`` rad/s. ``vx`` is forward, ``vy`` is right,
    ``vz`` is down-positive only inside the adapter; in the body contract here
    ``vz`` is up-positive (the conversion negates it for NED).
    """

    model_config = {"frozen": True}

    vx: float
    vy: float
    vz: float
    yaw_rate: float


def body_to_local_ned(action: Action4D, yaw_rad: float) -> tuple[float, float, float, float]:
    """Rotate a body-frame velocity into local-NED using the current heading.

    ``yaw_rad`` is the vehicle's current heading (0 = North, CW positive, ENU
    yaw measured from East would differ — this assumes NED heading). Returns
    ``(v_north, v_east, v_down, yaw_rate)`` ready for ``SET_POSITION_TARGET_LOCAL_NED``.

    This is the *only* function in the codebase that crosses the body/NED
    boundary; keeping it in one place is a hard invariant from the docs.
    """
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)
    # body x = forward, body y = right -> NED north/east
    v_north = action.vx * cos_y - action.vy * sin_y
    v_east = action.vx * sin_y + action.vy * cos_y
    # body vz is up-positive; NED down is positive
    v_down = -action.vz
    return (v_north, v_east, v_down, action.yaw_rate)
