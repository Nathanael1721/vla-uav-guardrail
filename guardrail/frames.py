"""The one place a body-frame action becomes a world-frame one, and back, and
the one body -> local-NED function every rail's adapter calls.

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

THE CONTRACT (WP3-04 / ARCH-04, since wf21)

The grant (docs/02-implementation/safety-shield.md): "Action input frame:
body frame (vx, vy, vz, yaw_rate) ... The body->local-NED conversion happens in
the MAVLink adapter, *after* projection", and "Body-frame -> local-NED
conversion lives here, applied to every emitted setpoint ... the conversion is
a single boundary in the codebase." In this codebase:

  * BODY IN, BODY OUT at the Shield. `Shield.filter_body` (and `advise_body`,
    `check_body`) take the action in the body frame and hand back the emitted
    action in the body frame. Inside, the projection computes in the world
    frame its rules are written in: `from_body` at the state's heading, the
    repair, `to_body` at the same heading - the projection's own internal
    frame, never seen on either side of its interface. The round trip is
    exact at a whole number of quarter turns (`_quarter_turns`: the rotation
    is done as a permutation there) and within a few ulp elsewhere
    (tests/test_frame_boundary.py: |to_body(from_body(b)) - b| <= 4e-15 m/s
    over seeded actions at 0, 90, 179.9, -90 and random headings).
  * BODY IN, BODY OUT at every REPAIR OPERATOR (since 2026-11-21). The
    grant's "keeps repair operators acting on the same frame the VLA emits
    in" is honoured at the operator interface (guardrail/repair.py
    RepairOperator): each operator is handed the body-frame Action4D at the
    tick's heading (`RepairContext.heading_deg`, the decision state's
    yaw_deg) and hands one back. This is how the grant's own reference
    implementation does it: its operators take and return the body-frame
    Action4D and, for their geometry, turn it into the IR's ENU plane and
    back inside the operator (kuanting-vla-uav-guardrail packages/
    safety-shield/src/safety_shield/repair.py LateralProjection and
    GeofenceEscape: `body_to_enu`, the projection in ENU, `enu_to_body`;
    checker.py and kinematics.py `predict` forecast in ENU the same way).
    Ours: `RepairContext.to_world` (`from_body`) and `RepairContext.to_body`
    (`to_body`) at the tick's heading - HookOperator around each Shield
    `_repair_*` call, PathRepair around its detour - while the Shield's
    chain keeps the world action its re-checks read and converts once each
    way per operator call. The pair is lossless on its own results (a body
    action `to_body` made converts back to the world action it was made
    from, not to a re-rotation of it), so the decisions are those of the
    world-frame stack bit for bit, at every heading. Until 2026-11-21 the
    operators were handed the world action and the Shield turned once
    around the whole stack (`filter_body`). Since 2026-11-23 (WP3-04b) the
    Shield's two last resorts act through the same pair: ClearanceEscape
    (`Shield._rescue`, the heading flown when the chain did not converge)
    and the slowed GeofenceEscape from a zone that opened on the aircraft
    (`Shield._slower_opened_escape`) are operators (repair.ClearanceEscape,
    repair.OpenedEscape) handed the body action and handing one back; until
    then `Shield._decide` called both on the world action outside the
    interface, with the same decisions. So every repair that can change the
    flown action acts on the frame the VLA emits in. Neither side sees local NED
    (ours is North / East / up, the reference's East / North / up). That the
    repair is the body-frame one - the same body action in a scene that
    turns with the aircraft gets the same emitted body action - is pinned
    for each rule family in tests/test_frame_boundary.py: zone, kinematic
    and altitude envelopes, corridor, stand-off (and its overhead
    recovery), PathRepair's stand-off detour at any heading, the clearance
    rule on a raster map at the four quarter turns (a raster turned by any
    other angle is not the same map).
  * THE ONE BODY -> NED CROSSING is `body_to_local_ned` (who calls it: the
    first entry below).

WHO CALLS THIS

  * THE ONE BODY -> NED CROSSING is `body_to_local_ned`, called by each rail's
    adapter after the projection, at the heading the adapter reads:
      - `sitl/mavlink_adapter_node.py` (`body_to_ned`, the ROS 2 rail and
        `sitl/ap_dds_adapter_node.py` through it; /shield/setpoint ->
        SET_POSITION_TARGET_LOCAL_NED via MAVROS);
      - `demo/pas_ardupilot/autopilot.py` (`ned_setpoint`,
        `enu_position_target`: PymavlinkLink.send and
        `demo/pas_ardupilot/mavros_link.py` MavrosLink.send on the PAS
        ArduPilot rail);
      - `sitl/run_sitl_demo.py` (`MavlinkAdapter.send_body`, pymavlink SITL);
      - `experiments/sweep_scenarios.py` (the headless harness: the NED
        setpoint is what its integrator flies, `local_ned_to_world`);
      - the simulator loops' NED velocity command: `demo/follow_vlm.py` (the
        city follow rail and its leg to the landing site),
        `demo/vla_fly.py` (with `demo/vla_mission.py` MissionLoop),
        `demo/aerialvla_demo.py`, `demo/aerialvla_pas_demo.py`,
        `demo/real_vla_demo.py`, `demo/semantic_demo.py`,
        `demo/semantic_seek.py`, `demo/run_demo.py`.
    No other code on a command path turns a body action into the world or
    NED frame; tests/test_frame_boundary.py fails on a new crossing (a call
    of `from_body` / `to_local_ned` outside this module and the Shield's
    projection, a call of `body_to_local_ned` from a module not listed
    above, or a hand-written cos / sin rotation of a velocity into an
    Action4D or a velocity command).

And besides those adapters:

  * `guardrail/shield.py` (`Shield.filter_body`, `advise_body`, `check_body`):
    `from_body` and `to_body`, the projection's internal world frame. The ROS
    2 Shield node (`sitl/ros2_shield_node.py`) hands it /vla/action as it
    arrives and publishes the emitted body action on /shield/setpoint; it
    rotates nothing itself.
  * WORLD -> BODY at a producer whose controller speaks the world frame (the
    producer is upstream of the Shield's input contract; this is not a
    body -> NED crossing): `filter_world_as_body` (`to_body` at the heading the
    Shield is given, every tick) in `experiments/sweep_scenarios.py`,
    `sitl/run_sitl_demo.py`, `demo/follow_vlm.py`, `demo/vla_mission.py`,
    `demo/vla_fly.py` and `demo/run_demo.py`; `body_action` for a world
    guidance vector blended into a body action in `demo/aerialvla_demo.py`,
    `demo/aerialvla_pas_demo.py` and the Shield-off arm of `demo/run_demo.py`;
    `to_body` in `sitl/ros2_vla_stub_node.py` and
    `sitl/ros2_vla_bridge_node.py` (the stub's and the step tracker's
    /vla/action in the grant's body frame), in
    `demo/pas_ardupilot/perception_node.py` (VLAPilot.step: the step
    tracker's action in the body frame the Shield takes) and in
    `guardrail/hold_flight.py` (`body_command`: the hold's own world-frame
    leg, never a VLA action, for a rail whose setpoints are body frame).
    `BODY_STOP` and `streamed_body` say what a rail streams on a stop.
    The step tracker (`guardrail/vla_link.py` StepTracker, behind vla_fly,
    vla_mission, the PAS VLA pilot and the ROS 2 VLA bridge) is such a
    world-frame controller between the VLA's decision and the Shield's
    input: its position steps fly to world points fixed at the capture, its
    carry continues the last world velocity, and its acceleration limits
    scale the world velocity's change as a vector - so its rate steps, too,
    compose the step's (forward, right) speed at the heading in the world
    frame, and the rail hands the Shield their body reading.
  * PRODUCER-SIDE SMOOTHING, before the Shield's input (or a jerk shaping the
    Shield re-checks, `demo/run_demo.py`): `limit_in_world` in the demo
    loops' rate limiters (`demo/aerialvla_demo.py` RateLimiter, imported by
    `demo/aerialvla_pas_demo.py`, `demo/semantic_demo.py`,
    `demo/semantic_seek.py`; `demo/real_vla_demo.py` and `demo/run_demo.py`
    their own) caps the change of the action's world reading, as these
    loops did before wf21, and hands back the body frame; `rebase_body`
    turns `demo/real_vla_demo.py`'s action from its frame's capture heading
    to the control tick's (a body-frame turn).
  * RECORDS, not commands: `guardrail/audit.py` (`to_body` writes each audit
    record's raw and emitted actions in the grant's body-frame keys),
    `guardrail/repair.py` (`to_body` logs PathRepair's detour waypoint in
    body frame; and, not a record, `RepairContext.to_world` / `to_body`:
    the repair-operator interface's rotation pair inside the projection,
    since 2026-11-21), `demo/real_vla_demo.py` (`from_body` in
    `openvla_raw_to_action`: the step log's `world_action` and its offline
    replay; the flight hands the Shield the body action).

Until wf21 (2026-11-18 and before) the dev rails handed the Shield the body
frame only where the round trip was exact to the bit (`filter_as_body`, else
`Shield.filter` on the world action) and flew the decision's world action;
the city follow and pymavlink SITL rails gave the Shield heading 0; the PAS
links, the harness's body arm and the ROS 2 adapter each turned the body
action with their own `from_body` call; and the other demo loops called
`Shield.filter` on a world action they had rotated themselves. `tests/
test_frame_contract.py` pins the conversions themselves. The module is not
exported from `guardrail/__init__.py`, which lists only the core contract
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


def _quarter_turns(yaw_deg: float) -> int | None:
    """0-3 when `yaw_deg` is an exact multiple of 90 degrees (0 = North, 1 =
    East, 2 = South, 3 = West), else None. math.cos(math.radians(90)) is
    6.1e-17, not 0, and a rotation through 0 * x turns -0.0 into 0.0 and a
    NaN in one channel into NaN in both; at these four headings the rotation
    is done as the permutation it is instead: exact, so to_body and from_body
    undo each other bit for bit there."""
    try:
        y = float(yaw_deg)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(y):
        return None
    turns, rest = divmod(y, 90.0)
    return int(turns) % 4 if rest == 0.0 else None


def from_body(vx_fwd: float, vy_right: float, vz_up: float,
              yaw_rate_rad_s: float, yaw_deg: float) -> Action4D:
    """A reference body-frame action, in our world-frame contract.

    `yaw_deg` is the vehicle's heading, 0 = North, clockwise positive — the same
    convention `State.yaw_deg` carries, so the caller passes the state it
    already has. It is the only quantity here measured in degrees, and it is a
    HEADING, not a rate.
    """
    q = _quarter_turns(yaw_deg)
    if q is not None:
        # A whole number of quarter turns: the rotation is a permutation and
        # sign flips, exact - so to_body / from_body round-trip bit for bit,
        # signed zeros and a non-finite channel included (a NaN stays in its
        # own channel instead of spreading to both through 0 * NaN).
        vx, vy = ((vx_fwd, vy_right), (-vy_right, vx_fwd),
                  (-vx_fwd, -vy_right), (vy_right, -vx_fwd))[q]
        return Action4D(vx=vx, vy=vy, vz_up=vz_up, yaw_rate=yaw_rate_rad_s)
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
    q = _quarter_turns(yaw_deg)
    if q is not None:
        # Exact, as in from_body: the inverse permutation.
        vx, vy = action.vx, action.vy
        fwd, right = ((vx, vy), (vy, -vx), (-vx, -vy), (-vy, vx))[q]
        return (fwd, right, action.vz_up, action.yaw_rate)
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


def _channels(body) -> tuple:
    """(forward, right, up, yaw_rate) of a body action given as an Action4D
    (vx forward, vy right) or as a 4-sequence."""
    if isinstance(body, Action4D):
        return (body.vx, body.vy, body.vz_up, body.yaw_rate)
    fwd, right, up, yaw_rate = body
    return (float(fwd), float(right), float(up), float(yaw_rate))


def rebase_body(body: Action4D, from_yaw_deg: float, to_yaw_deg: float) -> Action4D:
    """A body action given at heading `from_yaw_deg`, as the same velocity
    seen from heading `to_yaw_deg`: turned by the heading change, in the body
    frame (no world frame in between). Returns an Action4D (vx forward, vy
    right at the new heading).

    For a producer whose action was made from a view captured at one heading
    and is flown at a later control tick's heading (demo/real_vla_demo.py:
    OpenVLA's ~1 s inference): the velocity keeps the direction the model
    meant relative to what it SAW, so the aircraft's turn since the capture
    does not swing it. A turn by d degrees clockwise is `to_body` at d."""
    return body_action(body, float(to_yaw_deg) - float(from_yaw_deg))


def limit_in_world(limit, body, yaw_deg: float) -> Action4D:
    """A per-tick rate limiter run on the WORLD reading of a body action, the
    result handed back in the body frame at the same heading.

    `limit` takes and returns a world-frame Action4D (a demo loop's
    RateLimiter: a per-axis cap on the change since the last tick). It is a
    smoother on the PRODUCER's side of the Shield (before its input, or, in
    demo/run_demo.py, a jerk shaping that the Shield re-checks), not a repair
    operator and not a body -> NED crossing toward an autopilot; nothing it
    returns leaves the body frame.

    Why the world reading: the cap is on the change of the velocity the
    aircraft flies. A per-axis clip does not commute with a rotation, and a
    body-frame clip does not see the change a turn makes - a constant forward
    speed while the heading turns 90 degrees is a world velocity that swings
    through 90 degrees, which the world-frame cap limits and a body-frame cap
    passes unchanged. Every demo loop limited the world reading before wf21;
    this keeps that behaviour (tests/test_frame_boundary.py pins it through a
    turn)."""
    fwd, right, up, yaw_rate = _channels(body)
    return body_action(limit(from_body(fwd, right, up, yaw_rate, yaw_deg)), yaw_deg)


def body_to_local_ned(body, yaw_deg: float) -> tuple:
    """THE body -> local-NED crossing (WP3-04, ARCH-04): the MAVLink adapter
    boundary of every rail.

    `body` is the action a rail streams, in the grant's body frame (forward,
    right, up, yaw rate in rad/s: an Action4D with vx forward and vy right,
    or a 4-sequence); `yaw_deg` is the aircraft's heading (0 = North,
    clockwise positive) as the adapter reads it. Returns `(v_north, v_east,
    v_down, yaw_rate_rad_s)`: the velocity and yaw-rate fields of
    SET_POSITION_TARGET_LOCAL_NED, which is also the frame of a simulator's
    NED velocity command (Project AirSim's move_by_velocity_async, AirSim's
    moveByVelocityAsync) and of the headless harness's integrator
    (`local_ned_to_world`).

    The grant (docs/02-implementation/safety-shield.md, "MAVLink adapter"):
    "Body-frame -> local-NED conversion lives here, applied to every emitted
    setpoint. Repair operators above never see local-NED; the conversion is a
    single boundary in the codebase." Every rail's adapter calls this one
    function, after the projection. Apart from the projection's own internal
    world frame (`Shield.filter_body`: `from_body`, filter, `to_body`; and
    the repair-operator interface inside it, guardrail/repair.py
    `RepairContext.to_world` / `to_body`), nothing
    else in the codebase turns a body action into the world or NED frame;
    tests/test_frame_boundary.py fails on a new crossing."""
    fwd, right, up, yaw_rate = _channels(body)
    return to_local_ned(from_body(fwd, right, up, yaw_rate, yaw_deg))


def local_ned_to_world(ned) -> Action4D:
    """A local-NED velocity setpoint `(v_north, v_east, v_down, yaw_rate)` as
    the (North, East, up) velocity the headless harness's integrator flies.
    Not a rotation (local NED and the world frame share their horizontal
    axes): only the vertical sign, and -(-x) == x, so it is exact."""
    vn, ve, vd, yaw_rate = ned
    return Action4D(vx=vn, vy=ve, vz_up=-vd, yaw_rate=yaw_rate)


def filter_world_as_body(shield, state, world: Action4D, t: float | None = None,
                         *, arc: bool | None = False, **kw):
    """A rail whose producer speaks the world frame hands the Shield the
    action in the BODY frame, on every tick (WP3-04 / ARCH-04). Returns
    `(decision, body_in, emitted_body)`.

    The producer (a world-frame controller, the step tracker, a scripted
    pilot) is upstream of the Shield's input contract. The rail turns its
    action into the body frame here (`to_body`), at the heading the Shield
    is given (`state.yaw_deg`: the aircraft's own heading on every rail),
    and hands the Shield that body action (`Shield.filter_body`). The rail
    streams the emitted body action (a stop on the FSM's brake, its own body
    action with the Shield off; `streamed_body`) to its adapter, which
    turns it into local NED with `body_to_local_ned` - after the
    projection, as the grant says.

    Until this change (`filter_as_body`, 2026-11-18) a tick was handed over
    in the body frame only where the rotations' round trip gave the world
    action back bit for bit, else to `Shield.filter` in the world frame, and
    the rails flew the decision's world action. Now no tick is handed over in
    the world frame. The decision's `raw` / `emitted` (world frame, for the
    logs, the audit record and the KPI re-check) are the Shield's own
    `from_body` and projection results, so at a heading whose cosine and
    sine are not both exact they can differ from the producer's world action
    in the last bits (bounded in tests/test_frame_boundary.py).

    `arc` defaults OFF here: the Shield's opt-in forecast of a body command's
    yaw-rate arc is unflown on these rails. None takes the Shield's own
    `arc_forecast`. Keyword arguments are `Shield.filter`'s (rtl_failed,
    home_reached, landed, hold_off_m, hold_reach_m)."""
    body = body_action(world, state.yaw_deg)
    decision, emitted_body = shield.filter_body(state, body, t, arc=arc, **kw)
    return decision, body, emitted_body


# A stop, in the body frame (and in every other: guardrail/shield.py BRAKE).
BODY_STOP = Action4D(vx=0.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)


def streamed_body(decision, emitted_body: Action4D | None, *, shield_on: bool = True,
                  body_in: Action4D | None = None) -> Action4D | None:
    """What a rail streams to its adapter for one tick, in the BODY frame.

    With the Shield on: its emitted body action when no escalation FSM runs
    or the FSM's setpoint is "pass"; a stop on "brake" (ShieldDecision.command
    is shield.BRAKE there); None on "none" (LOITER / RTL / LAND own the
    aircraft and nothing is streamed). With the Shield OFF: the producer's
    body action as it was handed in (`body_in`), so the control arm flies
    the raw action. The adapter turns the result into local NED with
    `body_to_local_ned`."""
    if not shield_on:
        return body_in
    sp = getattr(decision, "setpoint", None)
    if sp is None or sp == "pass":
        return emitted_body
    if sp == "brake":
        return BODY_STOP
    return None
