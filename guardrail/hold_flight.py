"""How a flight rail flies the escalation FSM's hold point (since 2026-11-07).

WHY THIS FILE EXISTS

The grant's escalation FSM names the modes and never the place (Safety Shield
p4-p5: "SET_MODE GUIDED / LOITER / RTL / LAND"). Since 2026-10-19 the FSM asks
for a place (`EscalationFSM.needs_hold` / `hold_at`), and the hold-point search
(guardrail/holdpoint.py) answers from the Shield's own rules - in place when a
stop where the aircraft is stays legal, otherwise a MOVED point found legal,
since 2026-11-07 legal for as long as the Loiter can last. Until today only
the stress harness flew a moved point (experiments/sweep_scenarios.py
`_autopilot`, `--autopilot-hold moved_points`); every flight rail held where
the aircraft was, so on the rig the hold the search moves out of a spinning
bar's path was not flown, and dyn-nfz-rotate-spin went home by X6. This module
is the one place a rail turns the plan into what it sends, so the pymavlink
rail (sitl/run_sitl_demo.py) and the ROS 2 Shield node (sitl/ros2_shield_node.py,
the hil path) fly the same thing, and the harness's `rail` arm models it.

THE MODE MAPPING (what each rail sends; the FSM's words in the record)

Since 2026-11-07 (night) a Loiter is HELD IN GUIDED by the companion
(`guided_hold`, the default): the FSM's LOITER is not sent to ArduCopter, and
the companion streams the TRANSIT law (below) toward the hold's point on every
tick - to a moved point and then on it, or on an in-place plan's point (where
the aircraft was when it was planned), or, while the plan is still being
searched for, on where the aircraft was when the Loiter began. ArduCopter's
BRAKE is asked for only when the companion cannot stream: a stale input (the
rails' own stale holds) or a gap in its stream (force_brake; guardrail/
response.py HOLD_STREAM_GAP_S), and then for the rest of that Loiter.

  FSM state  plan      ArduCopter                  companion streams
  Loiter     any       stays in GUIDED             the TRANSIT law toward the
                       (BRAKE on a stream fault)   hold's point, every tick
  Land       in_place  LAND at once                nothing
  Land       moved     GUIDED to the site level,   the transit, then nothing
                       then LAND

WHY NOT BRAKE (the hil shakedowns hs03 and hs04, 7 Nov). Every hand-over from
GUIDED to ArduCopter 4.5.7's BRAKE carried the aircraft on in the direction of
the last GUIDED command: hs04_006 handed over at 10.66 s at -0.10 m/s while
GUIDED was accelerating it toward the repaired request (2.1, -2.0) m/s, and
under BRAKE it accelerated to 0.93 m/s that way, peaking 1.1 s later, and
crept on - 1.7 m in 2.2 s, 0.45 m/s still at 2.7 s - away from the in-place
point the search had checked, until the bar's sweep reached it (X6 at
18.8 s); hs03_004 the same backward (1.0 m). The recorded velocities fit
v(t) = (v0 + (a0 - v0) t) e^(-t), the critically damped response of a
velocity PI loop (ArduCopter's default PSC_VELXY_P 2, PSC_VELXY_I 1) whose
integrator starts at GUIDED's last acceleration demand a0 (hs04_006: a0 =
(1.85, -1.74) m/s^2, the airframe's 2.5 m/s^2 toward the command) and is bled
off at the loop's own pace: it carries the aircraft about |a0| x 1 s^2 =
2.5 m. A hold that hands over to BRAKE lands wherever that carry ends, not
on the point the search checked; one held in GUIDED never hands over.

The old flow - the hold mode on arrival, at once for an in-place plan - is
kept as `HoldFlight(guided_hold=False)`, what the harness's rail arm flies
to model the rig's flights before this change (experiments/
sweep_scenarios.py RAIL_HOLD_MODEL "brake", with BRAKE's carry modelled).

Every other state passes the FSM's own mode request through untouched. On the
FSM's entry tick its own setpoint is the hand-over stop, which a rail streams
as it always did (a `command` is flown only where the FSM's setpoint is
"none"); the hold's law starts on the next tick, as the harness flies it.

TRANSIT. The point is a POSITION target, flown as ArduCopter flies one in
GUIDED: each axis at the sqrt controller of its remaining distance
(guardrail.vla_stub.sqrt_controller, AC_PosControl's shape), with the
position gain TRANSIT_P and the airframe's acceleration (the rail Shield's own
response model, guardrail/response.py ARDUCOPTER_GUIDED, 1.75 m/s^2: braking
onto the point no harder than the Shield forecasts a stop), capped at the
plan's speed and climb (the policy's caps, holdpoint._speeds) - what the
harness's `rail` arm flies too, through this class (RAIL_HOLD_MODEL). It is
streamed by the companion as GUIDED velocity setpoints, closed at the rail's
10 Hz on the measured position; a stale pose stops it (the rails hold then,
as for any stale input). "Reached": within REACHED_M horizontally (and
vertically for a Loiter).

GIVING UP (TransitWatch). A transit not reached is given up - the hold mode
sent wherever the aircraft is, the FSM's X6 / X7 judging what follows, as for
any hold - once the plan's travel time plus fsm.HOLD_ARRIVAL_SLACK_S has
passed AND the aircraft's distance to the point has stayed within
TRANSIT_PROGRESS_M of where it last moved by that much, either way, for
TRANSIT_STALL_S: the aircraft is stuck relative to its point. An aircraft
still carried away from it - the overshoot of the hand-over's stop, or of the
point itself, braked out by the transit's own command - is moving, not
stuck, and is not given up. Whatever it does, a transit is given up
TRANSIT_MAX_S after its travel time. The harness's `rail` arm gives up by
the same TransitWatch (experiments/sweep_scenarios.py,
TRANSIT_GIVE_UP_RESPONSES), so a headless rail-arm number predicts the rig.

A plan arrives on the tick its search ends (guardrail/holdpoint.py, THE
PER-TICK BOUND, since 2026-11-07): until then the FSM's hold is None on entry,
flown as in place, and on a re-plan an in-place plan, so the aircraft holds
where it is while the search runs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .fsm import HOLD_ARRIVAL_SLACK_S, FSMState, HoldPoint
from .response import ARDUCOPTER_GUIDED
from .vla_stub import sqrt_controller

# The position gain of the transit's sqrt controller (ArduCopter's
# PSC_POSXY_P default, 1.0; the harness's AP_POSXY_P).
TRANSIT_P = 1.0
# The transit's braking acceleration onto the point (the rail Shield's own
# response model) and vertically (half of it: the airframe's vertical
# authority is the weaker, and a hold's altitude change is small).
TRANSIT_ACCEL_MPS2 = ARDUCOPTER_GUIDED.accel_mps2
TRANSIT_ACCEL_Z_MPS2 = ARDUCOPTER_GUIDED.accel_mps2 / 2.0
# A moved point counts as reached within this distance (a real airframe, not
# the harness's 5 cm): BRAKE then stops within its own braking distance of a
# transit that is already slowing onto the point.
REACHED_M = 0.5
# When a transit that has not reached its point is given up (module docstring,
# GIVING UP; TransitWatch). History, all on dyn-nfz-rotate-spin@3 on the hil
# rig:
#   until 2026-11-07 (evening) the deadline alone (travel_s +
#     HOLD_ARRIVAL_SLACK_S): hs02 (d186077) gave up a 0.43 m move 1.25 s after
#     its plan - the plan's travel time is the straight line at the speed cap,
#     and neither the sqrt controller's last metre (gain TRANSIT_P: a 1 s time
#     constant) nor the rig's lag nor the hand-over's drift is in it;
#   then (199cfdf) the deadline and 1 s without closing 5 cm, the clock running
#     from the transit's start: hs03 (199cfdf) planned the point at 4.5 s with
#     the aircraft still braking from 3.0 m/s, so the distance GREW - 2.28 m at
#     4.5 s, 3.61 at 5.0, 4.42 at 5.5, 4.63 at 5.8 - while the transit's own
#     command braked it at the airframe's 2.0-2.3 m/s^2; the overshoot counted
#     as "not closing", the transit was given up at 5.8 s (4.63 m off), one tick
#     before the velocity crossed zero (5.9 s), and BRAKE held the aircraft
#     about 4.3 m forward of the point, where the bar's sweep reached it: X6 at
#     31.1 s. hs02's G8 there was luck.
#   since: the clock restarts whenever the distance has changed by
#     TRANSIT_PROGRESS_M, either way, since it last did - closing, or still
#     carried away - so it runs only while the aircraft is stuck relative to
#     the point, and it runs TRANSIT_STALL_S = 2 s: in hs03 the distance last
#     grew by 5 cm at 5.8 s, the overshoot turned at 5.9-6.0 s (4.66 m, the
#     farthest), and the transit's own command would have closed 5 cm well
#     inside 2 s (from rest at 2 m/s^2, 5 cm takes about 0.25 s). The harness's
#     rail arm, flown under 199cfdf's rule, gives the same transit up at the
#     same 5.8 s ("not closing for 1.3 s") and X6 sends it home, as hs03 did;
#     under this rule it holds at the point. A transit that truly cannot
#     close is still given up: held at one distance (a wind it cannot beat),
#     TRANSIT_STALL_S after its deadline; carried away steadily, TRANSIT_MAX_S
#     after its travel time.
TRANSIT_STALL_S = 2.0
TRANSIT_PROGRESS_M = 0.05
TRANSIT_MAX_S = 10.0

_HOLD_STATES = (FSMState.LOITER, FSMState.LAND)
# The speed and climb a GUIDED hold closes on a point with when its plan names
# none (an in-place plan, or no plan yet): the hold-point search's own
# defaults (guardrail/holdpoint.py DEFAULT_SPEED_MPS / DEFAULT_CLIMB_MPS).
HOLD_SPEED_MPS = 4.0
HOLD_CLIMB_MPS = 2.0


@dataclass(frozen=True)
class _Target:
    """Where a GUIDED hold holds: what transit_velocity reads of a plan."""
    mode: str
    x: float
    y: float
    up: float
    speed_mps: float
    climb_mps: float


class TransitWatch:
    """When to give up one moved plan's transit (module docstring, GIVING UP).

    `t0` is the plan's time (its t_s), `travel_s` its travel time. Call
    update(dist, t) on every tick the transit is flown, with the aircraft's
    distance to the point (horizontal for a Land, else 3-D); it returns True
    on the tick the transit is given up, and `why` then says why. One class
    for the rails (HoldFlight) and the harness's `rail` arm (experiments/
    sweep_scenarios.py), so both give up on the same ticks for the same
    distances."""

    def __init__(self, t0: float, travel_s: float):
        self.t0, self.travel_s = float(t0), float(travel_s)
        self.ref: float | None = None   # the distance when it last moved PROGRESS
        self.since: float | None = None  # and when
        self.why: str | None = None

    def update(self, dist: float, t: float) -> bool:
        if self.ref is None or abs(dist - self.ref) >= TRANSIT_PROGRESS_M - 1e-9:
            # Moving relative to the point - closing, or still carried away
            # (an overshoot the transit's own command is braking out): the
            # clock starts again.
            self.ref, self.since = dist, t
        due = self.t0 + self.travel_s + HOLD_ARRIVAL_SLACK_S
        if t >= due - 1e-9 and t - self.since >= TRANSIT_STALL_S - 1e-9:
            self.why = (f"{dist:.2f} m off; neither closing nor carried away for "
                        f"{t - self.since:.1f} s")
            return True
        if t >= self.t0 + self.travel_s + TRANSIT_MAX_S - 1e-9:
            self.why = f"{dist:.2f} m off; {TRANSIT_MAX_S:g} s past its travel time"
            return True
        return False


def transit_distance(x: float, y: float, up: float, hold) -> float:
    """The distance TransitWatch judges: horizontal for a Land (it transits
    level, then descends), 3-D for a Loiter. `hold` is a HoldPoint or its
    dict (the harness's ShieldDecision.hold_point)."""
    get = hold.get if isinstance(hold, dict) else (lambda k: getattr(hold, k))
    dz = 0.0 if get("mode") == "LAND" else get("up") - up
    return math.hypot(get("x") - x, get("y") - y, dz)


def transit_velocity(x: float, y: float, up: float, hold: HoldPoint, dt: float,
                     accel_mps2: float = TRANSIT_ACCEL_MPS2,
                     accel_z_mps2: float = TRANSIT_ACCEL_Z_MPS2
                     ) -> tuple[float, float, float]:
    """The world-frame velocity (north, east, up) that flies the moved `hold`
    from (x, y, up) this tick (module docstring, TRANSIT): a Land's site is
    reached level, then landed by the autopilot."""
    dx, dy = hold.x - x, hold.y - y
    d = math.hypot(dx, dy)
    speed = float(hold.speed_mps or 0.0)
    v = min(speed, sqrt_controller(d, TRANSIT_P, accel_mps2, dt)) if d > 1e-9 else 0.0
    vx, vy = ((v * dx / d, v * dy / d) if d > 1e-9 else (0.0, 0.0))
    vz = 0.0
    if hold.mode == "LOITER":
        dz = hold.up - up
        climb = float(hold.climb_mps or 0.0)
        vz = (math.copysign(min(climb, sqrt_controller(abs(dz), TRANSIT_P, accel_z_mps2, dt)),
                            dz) if abs(dz) > 1e-9 else 0.0)
    return vx, vy, vz


def reached(x: float, y: float, up: float, hold: HoldPoint,
            tol_m: float = REACHED_M) -> bool:
    """Is the aircraft at the moved `hold` (a Land: horizontally)?"""
    if math.hypot(hold.x - x, hold.y - y) > tol_m:
        return False
    return hold.mode == "LAND" or abs(hold.up - up) <= tol_m


@dataclass(frozen=True)
class HoldTick:
    """What a rail sends on one tick for the FSM's hold.

    mode      the mode to request now, in the FSM's words (GUIDED, LOITER,
              LAND, RTL; the rail maps LOITER to its hold mode), or None.
    command   the world-frame velocity (north, east, up) to stream in GUIDED
              (a transit), or None: stream what the FSM's setpoint says.
    transit   a transit is being flown (the moved plan, not reached yet).
    note      why, for the rail's event log (None: nothing new)."""
    mode: str | None
    command: tuple[float, float, float] | None
    transit: bool
    note: str | None = None


class HoldFlight:
    """One episode's flight of the FSM's hold points (module docstring).

    Call `tick(fsm, set_mode, x, y, up, t, dt)` once per rail tick AFTER the
    FSM stepped and any plan it asked for was given to it (`hold_at`), with
    the FSM's own mode request of the tick (`FSMOutput.set_mode`). Send the
    returned `mode` (if any) instead of the FSM's, and stream `command` (if
    any) instead of the FSM's setpoint."""

    def __init__(self, accel_mps2: float = TRANSIT_ACCEL_MPS2,
                 accel_z_mps2: float = TRANSIT_ACCEL_Z_MPS2,
                 reached_m: float = REACHED_M, guided_hold: bool = True) -> None:
        self.accel_mps2 = float(accel_mps2)
        self.accel_z_mps2 = float(accel_z_mps2)
        self.reached_m = float(reached_m)
        self.guided_hold = bool(guided_hold)
        self.flying: HoldPoint | None = None     # the moved plan in flight
        self.held = False                        # the hold mode was sent for the current plan
        self.transits = 0                        # moved plans flown this episode
        self.watch: TransitWatch | None = None   # when to give up the transit
        # A GUIDED hold (guided_hold): the plan it holds, where it holds it,
        # whether the moved point was reached (or given up), and a forced
        # hand-over to BRAKE (force_brake) for the rest of the Loiter.
        self.guiding = False
        self._plan_seen: object = None
        self._target: _Target | None = None
        self._arrived = False
        self.forced: str | None = None
        self.forced_count = 0

    def force_brake(self, why: str) -> bool:
        """The companion cannot stream the GUIDED hold (a gap in its stream):
        hand the hold to ArduCopter's BRAKE for the rest of this Loiter. True
        if a GUIDED hold was being flown."""
        if not self.guiding or self.forced:
            return False
        self.forced = why
        self.forced_count += 1
        return True

    def tick(self, fsm, set_mode: str | None, x: float, y: float, up: float,
             t: float, dt: float) -> HoldTick:
        st = fsm.state
        if fsm.terminal or st not in _HOLD_STATES:
            self.flying, self.held = None, False
            self.guiding, self._plan_seen, self._target = False, None, None
            self._arrived, self.forced = False, None
            return HoldTick(set_mode, None, False)
        plan = fsm.hold
        hold_mode = "LOITER" if st is FSMState.LOITER else "LAND"
        if st is FSMState.LOITER and self.guided_hold:
            return self._guided(plan, set_mode, x, y, up, t, dt)
        if plan is None or plan.kind != "moved":
            if self.flying is not None and not self.held:
                # The plan went back in place while a transit flew: hold now.
                self.flying, self.held = None, True
                return HoldTick(hold_mode, None, False,
                                "hold back in place: the autopilot holds here")
            if set_mode is not None:
                self.held = True
            return HoldTick(set_mode, None, False)
        mode = None
        note = None
        if plan is not self.flying:
            # A new moved plan: fly it in GUIDED. On the entry tick the
            # aircraft is still in GUIDED (Brake / Normal), so the FSM's hold
            # request is simply held back; a re-plan while the autopilot holds
            # asks GUIDED again.
            mode = "GUIDED" if (self.held and set_mode is None) else None
            self.flying, self.held = plan, False
            self.transits += 1
            self.watch = TransitWatch(plan.t_s if plan.t_s is not None else t,
                                      plan.travel_s)
            note = (f"{hold_mode} at a moved point: GUIDED transit to "
                    f"({plan.x:.1f}, {plan.y:.1f}, {plan.up:.1f}), the hold on arrival")
        elif self.held:
            return HoldTick(None, None, False)
        at = reached(x, y, up, plan, self.reached_m)
        if at or self.watch.update(transit_distance(x, y, up, plan), t):
            self.held = True
            why = ("reached" if at else
                   f"not reached by {t:.1f} s ({self.watch.why}): holding where it is")
            return HoldTick(hold_mode, None, False,
                            note or f"moved hold {why}: the autopilot holds")
        return HoldTick(mode, transit_velocity(x, y, up, plan, dt, self.accel_mps2,
                                               self.accel_z_mps2), True, note)

    def _guided(self, plan, set_mode, x, y, up, t, dt) -> HoldTick:
        """A Loiter held in GUIDED (module docstring, THE MODE MAPPING)."""
        if self.forced:
            if self.guiding:
                # The hand-over the stream fault forces: BRAKE, once.
                self.guiding, self.held = False, True
                return HoldTick("LOITER", None, False,
                                f"GUIDED hold handed to the autopilot: {self.forced}")
            return HoldTick(None, None, False)
        note = None
        mode = None
        if not self.guiding:
            # The Loiter's first tick (or the first after the autopilot held):
            # the FSM's LOITER is not sent; the companion holds in GUIDED.
            self.guiding = True
            if self.held:
                mode, self.held = "GUIDED", False
        if plan is not self._plan_seen or self._target is None:
            self._plan_seen = plan
            self._arrived = False
            self.watch = None
            if plan is None:
                self._target = _Target("LOITER", x, y, up, HOLD_SPEED_MPS, HOLD_CLIMB_MPS)
                what = "no plan yet: where the Loiter began"
            else:
                self._target = _Target(
                    "LOITER", plan.x, plan.y, plan.up,
                    float(plan.speed_mps or HOLD_SPEED_MPS),
                    float(plan.climb_mps or HOLD_CLIMB_MPS))
                what = f"its {plan.kind} point"
                if plan.kind == "moved":
                    self.transits += 1
                    self.watch = TransitWatch(plan.t_s if plan.t_s is not None else t,
                                              plan.travel_s)
            tg = self._target
            note = (f"LOITER held in GUIDED by the companion at ({tg.x:.1f}, {tg.y:.1f}, "
                    f"{tg.up:.1f}), {what}")
        tg = self._target
        moving = plan is not None and plan.kind == "moved" and not self._arrived
        if moving:
            if reached(x, y, up, tg, self.reached_m):
                self._arrived, moving = True, False
                note = note or "moved hold reached: held there in GUIDED"
            elif self.watch is not None and self.watch.update(
                    transit_distance(x, y, up, tg), t):
                # Given up: held where the aircraft is, still in GUIDED.
                self._arrived, moving = True, False
                self._target = _Target("LOITER", x, y, up, tg.speed_mps, tg.climb_mps)
                note = (f"moved hold not reached by {t:.1f} s ({self.watch.why}): held "
                        f"where it is, in GUIDED")
        return HoldTick(mode, transit_velocity(x, y, up, self._target, dt, self.accel_mps2,
                                               self.accel_z_mps2), moving, note)
