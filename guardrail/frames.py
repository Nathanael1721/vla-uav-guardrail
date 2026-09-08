"""The one place a body-frame action becomes a world-frame one, and back.

WHY THIS EXISTS

`docs/CHECKLIST-remaining-work.md` item 9 records a contract mismatch with the
reference implementation and adds: *"Both cannot be right, and no test compares
them."* This module is that comparison, made executable.

    ours       guardrail.models.Action4D    vx +North, vy +East, vz_up +up
    reference  vlaguard_common.Action4D     vx forward, vy right, vz +up

Both are internally consistent, and the conversion between them is exact, so the
mismatch is a boundary to be crossed once rather than a defect in either. Ours is
world-frame because every constraint the Shield checks is world geometry — a
fence is a polygon on the ground, a stand-off is a distance to a point, an
altitude band is an altitude. Checking those against a body-frame action would
mean rotating on every rule evaluation instead of once at the boundary. The
reference is body-frame because its repair operators act where the airframe acts,
and it converts once in its MAVLink adapter.

**`yaw_rate` is rad/s on BOTH sides**, and `vz`/`vz_up` is up-positive on both.
Only the frame differs.

WHAT WRITING THIS FILE FOUND

The first version of this module asserted the opposite — that our `yaw_rate` was
deg/s and the reference's rad/s, a factor of 57.3 — because `guardrail/models.py`
said so in a comment. The comment was wrong, and it had been wrong long enough
for two of the four adapters to believe it and convert a radian value to radians
a second time. `shield.py` enforces the cap in radians at three sites, and the
only live producer (`demo/follow_vlm.py`'s servo) emits a radian-scale value —
unclipped on both live tracking paths, and `retarget_demo` recorded 2.375 rad/s
(136 °/s), well past the 45 °/s cap. See
`docs/FINDING-the-contract-disagreed-with-itself-about-yaw.md`.

So this module deliberately does NOT convert the yaw rate. The conversion that
looked like the careful thing to do was the bug.

WHO CALLS THIS

Nothing in production, today. `from_body` and `to_body` are imported only by
`tests/test_frame_contract.py`; `to_local_ned` documents what
`sitl/run_sitl_demo.py` does without being called by it. That is deliberate and
worth stating rather than leaving a reader to assume otherwise: the module
exists so that when a body-frame producer IS wired in, it crosses here once,
with tests, instead of a conversion appearing at the call site. It is not
exported from `guardrail/__init__.py`, which lists only the core contract types
- `bundle`, `kpi`, `manifest`, `compiler`, `projection` and `audit` are all
absent from `__all__` too, so this follows the package's convention rather than
departing from it.
"""
from __future__ import annotations

import math

from .models import Action4D

# Stated here so a reader never has to infer it from a call site, and pinned by
# tests/test_frame_contract.py against what shield.py actually enforces.
YAW_RATE_UNITS = "rad/s"
REFERENCE_YAW_RATE_UNITS = "rad/s"


def from_body(vx_fwd: float, vy_right: float, vz_up: float,
              yaw_rate_rad_s: float, yaw_deg: float) -> Action4D:
    """A reference body-frame action, in our world-frame contract.

    `yaw_deg` is the vehicle's heading, 0 = North, clockwise positive — the same
    convention `State.yaw_deg` carries, so the caller passes the state it
    already has. It is the only quantity here measured in degrees, and it is a
    HEADING, not a rate.
    """
    psi = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(psi), math.sin(psi)
    return Action4D(
        # Forward is (cos, sin) in (North, East); right is 90 deg clockwise of
        # it, which is (-sin, cos). Matches vlaguard_common.body_to_local_ned.
        vx=vx_fwd * cos_y - vy_right * sin_y,
        vy=vx_fwd * sin_y + vy_right * cos_y,
        vz_up=vz_up,                       # both contracts are up-positive
        yaw_rate=yaw_rate_rad_s,           # both contracts are rad/s
    )


def to_body(action: Action4D, yaw_deg: float) -> tuple:
    """Our world-frame action as the reference's body-frame tuple.

    Returns `(vx_forward, vy_right, vz_up, yaw_rate_rad_s)`.
    """
    psi = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(psi), math.sin(psi)
    return (
        action.vx * cos_y + action.vy * sin_y,
        -action.vx * sin_y + action.vy * cos_y,
        action.vz_up,
        action.yaw_rate,
    )


def to_local_ned(action: Action4D) -> tuple:
    """`(v_north, v_east, v_down, yaw_rate_rad_s)` for a MAVLink adapter.

    A world-frame action needs no rotation to reach local-NED; only the vertical
    sign flips. This is the whole of our side of the boundary, and it is why the
    Shield never sees NED.

    The tuple is exactly what `SET_POSITION_TARGET_LOCAL_NED` wants, units
    included — that field is rad/s — so `sitl/run_sitl_demo.py` passes it
    straight through.
    """
    return (action.vx, action.vy, -action.vz_up, action.yaw_rate)
