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

The flight rails, at their boundary with a body-frame producer. (Until
2026-10-12 this paragraph said nothing in production did; that stopped being
true once the body-frame producers were wired in, and the paragraph was not
updated.) As of that date:

  * `guardrail/shield.py` (`Shield.filter_body`, since 2026-11-04): the
    Shield's own body-frame boundary (WP3-04, ARCH-04). `/vla/action`
    arrives in body frame at `sitl/ros2_shield_node.py`, which hands it to
    `filter_body` as it is; there it crosses to world frame once with
    `from_body` at the heading of the same pose, goes through
    `Shield.filter()`, and the emitted action crosses back with `to_body` at
    that heading, to be published on /shield/setpoint. Until 2026-11-04 the
    node made those two crossings itself around `Shield.filter()`. With
    `arc=True` (2026-11-17, opt-in) its forecast also turns the action's
    velocity with the heading at the yaw rate, judged beside the straight
    line (`Shield._predict`, "the arc only adds"), so the world-frame action
    it judges is the body command's first instant only. The repairs still
    act in the world frame there: this is not the grant's single
    body -> local-NED boundary.
  * `guardrail/hold_flight.py` (`body_command`, 2026-11-07): `to_body` of
    the hold's own leg - the GUIDED transit to a moved hold point and the
    Loiter held in GUIDED, a command the companion computes in world frame,
    never a VLA action - for `sitl/ros2_shield_node.py`, whose setpoints are
    body frame. The node streams the leg only on ticks whose FSM setpoint is
    "none" (no action flown), so it is that leg's one rotation and no VLA
    action is rotated twice; the node itself imports nothing from here
    (from 2026-11-07's moved hold until that night it called `to_body` for
    the leg itself, and tests/test_frame_contract.py failed on it).
  * `sitl/mavlink_adapter_node.py`: the body -> local-NED crossing of
    /shield/setpoint, `from_body` then `to_local_ned`.
  * `sitl/ros2_vla_stub_node.py`: `to_body`, so the stub publishes
    /vla/action in the body frame the grant's VLA contract names.
  * `sitl/ros2_vla_bridge_node.py` (2026-10-23): `to_body` on the step
    tracker's world-frame action, at the heading of the latest pose, as the
    stub does, so AerialVLA's /vla/action_4d is body frame too.
  * `demo/pas_ardupilot/autopilot.py` (`body_to_world`, used by
    `perception_node.py`): `from_body` on the way in, `to_local_ned` for the
    MAVLink setpoint on the way out.
  * `filter_as_body` (2026-11-18), at the boundary of the four dev rails
    whose producers speak the world frame: `demo/follow_vlm.py` (the city
    follow rail, the mission loop and the leg to the landing site),
    `sitl/run_sitl_demo.py` (pymavlink SITL), `demo/pas_ardupilot/
    perception_node.py` (PAS ArduPilot, --action-frame world, the default)
    and `experiments/sweep_scenarios.py` (the headless harness, run_episode
    action_frame "world", the default). Each turns its world action into
    the body frame with `to_body` at the heading the Shield is given and
    hands the Shield that BODY action (`Shield.filter_body`, the arc off);
    the Shield's own `from_body` turns it back, and the rail flies the
    emitted world action as before (tests/test_rail_body_frame.py: emitted
    within 1e-9 of Shield.filter's, headings at +-180 included). The city
    follow and pymavlink SITL rails give the Shield no heading (State.yaw_deg
    0), so there both turns are the identity (exact; only a zero's sign can
    change, -0.0 + 0.0 being +0.0). A world action
    with a non-finite horizontal channel is still handed to `Shield.filter`
    as it is (no body form keeps its finite channel).
  * `demo/pas_ardupilot/perception_node.py` (`VLAPilot.step`, --pilot vla,
    2026-10-23): `to_body` of the step tracker's world-frame action, for the
    row's `body` and MAVROS's record copy; by default the Shield is handed
    the scene-frame action through `filter_as_body` (above). With --action-frame
    body (2026-11-17, opt-in) the BODY action enters the Shield
    (`Shield.filter_body`, the arc on) and the link is sent the emitted body
    action (`to_body` of a stop or a stop's answer at the tick's heading).
  * `demo/pas_ardupilot/autopilot.py` `PymavlinkLink.send(body=...)` and
    `demo/pas_ardupilot/mavros_link.py` `MavrosLink.send(body=...)`
    (2026-11-17): the emitted body action turned into local NED after the
    projection, `from_body` at the link's heading (the adapter route
    publishes it as it is, for `sitl/mavlink_adapter_node.py`). These are
    further crossings beside the Shield's own from_body / to_body round
    trip, not the grant's single adapter boundary.
  * `experiments/sweep_scenarios.py` (run_episode `action_frame="body"`,
    2026-11-17, opt-in): the headless harness hands the Shield `to_body` of
    the pilot's action and flies `from_body` of what it emits.
  * `demo/pas_ardupilot/mavros_link.py` (`--setpoint-via adapter`,
    2026-11-15): `to_body` of the Shield's emitted world-frame action, once,
    at the pose's heading, published on /shield/setpoint for
    `sitl/mavlink_adapter_node.py` to cross to local NED; the default route
    publishes its own PositionTarget and rotates nothing here.
  * `demo/real_vla_demo.py`: `from_body` for the OpenVLA action.
  * `guardrail/audit.py`: `to_body` writes each audit record's raw and
    emitted actions in the grant's body-frame keys, at the tick's heading.
  * `guardrail/repair.py`: `to_body` logs PathRepair's detour waypoint in
    body frame (metres forward / right of the heading).

`sitl/run_sitl_demo.py` does what `to_local_ned` documents without calling it.
The other demo loops that call `Shield.filter` on a world-frame action
(demo/aerialvla_demo.py, aerialvla_pas_demo.py, real_vla_demo.py,
semantic_demo.py, semantic_seek.py, vla_fly.py, vla_mission.py, run_demo.py)
are not among the dev rails above and still hand the Shield the world frame.
`tests/test_frame_contract.py` pins the conversions themselves. The module is
not exported from `guardrail/__init__.py`, which lists only the core contract
types - `bundle`, `kpi`, `manifest`, `compiler`, `projection` and `audit` are
all absent from `__all__` too, so this follows the package's convention rather
than departing from it.
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


def body_action(action: Action4D, yaw_deg: float) -> Action4D:
    """`to_body` as an Action4D: vx forward, vy right, vz_up up, yaw_rate rad/s
    - the form `Shield.filter_body` and /vla/action carry a body action in."""
    fwd, right, up, yaw_rate = to_body(action, yaw_deg)
    return Action4D(vx=fwd, vy=right, vz_up=up, yaw_rate=yaw_rate)


def filter_as_body(shield, state, world: Action4D, t: float | None = None, **kw):
    """A dev rail's world-frame action handed to the Shield in the BODY frame
    (WP3-04 / ARCH-04, 2026-11-18). Returns the tick's ShieldDecision, whose
    raw and emitted actions are in the world frame, as `Shield.filter`'s are.

    The rail's producer speaks the world frame (a world-frame controller, the
    step tracker, the stub VLA), so the rail turns the action into the body
    frame here, at its boundary, at the heading the Shield itself turns it
    back with (`state.yaw_deg`), and hands the Shield that BODY action
    (`Shield.filter_body`). The Shield's one body -> world turn does the
    rest, so the rail flies `decision.emitted` as it always has.

    WHAT STAYS AS IT WAS. The arc forecast is OFF here (`arc=False`, whatever
    the Shield's own `arc_forecast` default): `Shield.filter` never arcs, and
    these rails fly as they did. The two rotations are exact up to rounding,
    so the decision is `filter(state, world)`'s to rounding (the rails'
    emitted actions within 1e-9: tests/test_rail_body_frame.py); at a heading
    of 0 - the city follow and pymavlink SITL rails give the Shield none,
    State.yaw_deg 0 - the rotations are the identity: the decision equals
    filter()'s exactly (==; only a zero's sign can change).

    THE ONE TICK HANDED IN THE WORLD FRAME. A world action with a non-finite
    horizontal channel has no body-frame form that keeps the finite one: the
    rotation mixes vx and vy, and NaN * 0 is NaN, so a NaN North speed would
    arrive as NaN North AND East, and the Shield's _sanitise would zero both
    where it zeroes the North channel alone today. So that tick (and one
    whose rotation does not come out finite: a non-finite heading, an
    overflow) is handed as it is, to `Shield.filter` - the same decision as
    before, to the bit. It is a contract fault the Shield reports
    (`contract`), never a command that flies as sent.

    Keyword arguments are `Shield.filter`'s (rtl_failed, home_reached,
    landed, hold_off_m, hold_reach_m)."""
    body = body_action(world, state.yaw_deg)
    if not all(math.isfinite(c) for c in (world.vx, world.vy, body.vx, body.vy)):
        return shield.filter(state, world, t, **kw)
    decision, _ = shield.filter_body(state, body, t, arc=False, **kw)
    return decision
