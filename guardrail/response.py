"""How the aircraft answers a velocity setpoint: the Shield's response model.

WHY THIS FILE EXISTS (2026-10-19)

The Shield's forecast started every pose from the COMMANDED velocity: State
had no velocity, and Shield._predict advanced the candidate action at constant
velocity from t = 0, as if the aircraft took up any commanded velocity
instantly. ArduCopter does not. On the 14 Oct ArduPilot close approach
(demo/out/pas_ap_video_on_20261014) the Shield first flagged the building at
tick 364, 4.6 m from the wall at 3.2 m/s, commanded 0.5-0.8 m/s away, and
judged that command to be leaving the margin; the aircraft flew on at near
full speed for ~0.35 s, braked at ~2 m/s^2 and ended 1.0 m inside the P0
margin. On the recorded flights the old forecast puts a braking aircraft
1.5 m short of where it went (median along-track error at 1.5 s, 197
decelerating windows; docs/DESIGN-own-velocity-forecast.md, "The fit").

THE MODEL

Horizontal only (see VERTICAL). Each velocity setpoint the autopilot is sent
is flown from `reaction_s` after it went out; the aircraft's velocity moves
straight towards the setpoint in force, at most `accel_mps2` per second (a
vector rate limit), and holds it once there. The Shield forecasts one
candidate action that way from the MEASURED velocity: for the first
`reaction_s` the aircraft flies the setpoints already sent (`pipe_segments`,
from Shield.note_flown; with no record of them it keeps its measured
velocity), then the candidate. Positions are the exact integral (`integrate`,
closed form per segment), so a forecast pose costs a few float operations.
The clearance check's "fly this for one control period, then stop" and a
hold's stop path (`stop_path`) are the same rule over a longer command list.

THE NUMBERS ARE MEASURED, and chosen by a written rule that is never
optimistic about a stop. tools/fit_velocity_response.py fits the physical
form - every setpoint actually sent, each from reaction_s on - to every
recorded ArduPilot SITL flight on the Project AirSim CityLife quad (six
pas_ap_* runs, 4417 windows, 550 with the command more than 1 m/s from the
measured velocity), against where the aircraft really went 1.0, 1.5 and
2.0 s later. Its minimum, the identified aircraft, is 0.35 s and 2.0 m/s^2
(RMS 0.31 / 0.49 / 0.64 m on the step windows, where the old forecast's is
1.41 / 1.81 / 2.34 m). But in the form the Shield flies (it cannot know the
next command, so it flies the pipe and then holds the candidate) those
numbers are SHORT on braking: on the 197 decelerating windows the forecast
falls a median 0.25 m short of where the aircraft went at 1.5 s (p25
0.51 m short). The Shield's numbers are therefore the most accurate grid
point whose median braking error in the Shield's own form is not short
(2026-10-20 review): 0.45 s and 1.75 m/s^2, median +0.03 m, with the same
accuracy in the Shield's form as the identified numbers (step RMS 0.13 /
0.36 / 0.71 m at 0.5 / 1.0 / 1.5 s, against 0.12 / 0.34 / 0.70 m) - they
describe a slightly slower aircraft than the one identified, the safe side
for a stop. A slower model is only optimistic about an aircraft speeding UP
towards a hazard, and that is the commanded forecast's case, which the
Shield judges beside this one (shield.py `_predict`, "THE MEASURED VELOCITY
ONLY ADDS"). tests/test_own_velocity_forecast.py re-runs the fit on the
committed slim copy of those flights, checks that it lands on these
constants, and pins the sign of the braking bias.

VERTICAL. The vertical channel is NOT modelled: on the 73 windows whose
commanded climb rate differed from the measured one by more than 0.3 m/s the
response model forecast altitude worse than the old commanded forecast (RMS
0.28 against 0.15 m at 1 s), and 95 % of the recorded ticks command under
0.25 m/s of climb. The forecast keeps the commanded climb rate, as before.

BRAKE. ArduCopter's BRAKE mode (the rail's flight mode for the escalation
FSM's Loiter) brakes far more gently on this airframe than a GUIDED zero
setpoint: on both recorded BRAKE entries the aircraft travelled 2.5-2.9 m
and was still moving 2.3 s later. ARDUCOPTER_BRAKE is the envelope of those
two entries after the GUIDED pipe flown with ARDUCOPTER_GUIDED (it never
forecasts less travel than either showed by the end): no further coast,
then at most 0.95 m/s^2. With two samples it is an envelope, not a fit,
and one of the two is not a clean deceleration (pas_ap_close_on tick 387:
the speed rises again from 0.57 to 1.06 m/s inside BRAKE, ticks 400-407,
while the aircraft was 2.2-3.2 m into the 3 m margin - at the building's
face, most likely in contact with it); the design note says so.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class VelocityResponse:
    """Reaction time and acceleration limit of the aircraft's horizontal
    velocity response to a setpoint. `source` says where the numbers came
    from; it is recorded with every flight that uses them."""
    reaction_s: float
    accel_mps2: float
    source: str

    def __post_init__(self) -> None:
        if not (math.isfinite(self.reaction_s) and self.reaction_s >= 0.0):
            raise ValueError(f"reaction_s must be finite and >= 0, got {self.reaction_s}")
        if not (math.isfinite(self.accel_mps2) and self.accel_mps2 > 0.0):
            raise ValueError(f"accel_mps2 must be finite and > 0, got {self.accel_mps2}")

    def to_dict(self) -> dict:
        return asdict(self)

    # ---- the closed form ------------------------------------------------

    def displacement(self, v0x: float, v0y: float, cx: float, cy: float,
                     tau: float) -> tuple[float, float]:
        """Horizontal displacement (dx, dy) `tau` seconds from now, the
        aircraft at measured velocity (v0x, v0y), keeping it for the reaction
        time (no record of what was sent before) and commanded (cx, cy) from
        now on: the Shield's forecast of one candidate action without a pipe."""
        lag = self.reaction_s
        if tau <= lag:
            return v0x * tau, v0y * tau
        dx, dy = v0x * lag, v0y * lag
        ex, ey, _, _ = _approach(v0x, v0y, cx, cy, self.accel_mps2, tau - lag)
        return dx + ex, dy + ey

    def velocity(self, v0x: float, v0y: float, cx: float, cy: float,
                 tau: float) -> tuple[float, float]:
        """The forecast horizontal velocity at `tau` (same assumptions)."""
        if tau <= self.reaction_s:
            return v0x, v0y
        _, _, vx, vy = _approach(v0x, v0y, cx, cy, self.accel_mps2, tau - self.reaction_s)
        return vx, vy

    def stop_time(self, speed: float) -> float:
        """Seconds until an aircraft at `speed` (m/s), commanded to stop now,
        is stationary: the reaction plus the braking."""
        return self.reaction_s + max(0.0, speed) / self.accel_mps2

    def stop_distance(self, speed: float) -> float:
        """Metres it covers meanwhile: v L + v^2 / (2 a)."""
        s = max(0.0, speed)
        return s * self.reaction_s + s * s / (2.0 * self.accel_mps2)

    def reaction(self, pipe=None) -> list:
        """The segments that cover the reaction time [0, reaction_s): the
        `pipe` when given (`pipe_segments`: the setpoints already sent, each
        acting reaction_s after it went out), else one coast segment - the
        aircraft keeps its measured velocity, the form a caller without the
        setpoint history gets."""
        if pipe:
            return list(pipe)
        return [(self.reaction_s, None, None)]

    def trajectory(self, v0x: float, v0y: float, commands, times, pipe=None) -> list:
        """Horizontal displacements and velocities at each of `times` (sorted,
        >= 0) for a sequence of commands [(cx, cy, duration_s), ...] issued
        from now on, each when the one before it ends (the last one's
        duration may be math.inf). They act from `reaction_s` on; until then
        the aircraft flies the `pipe` (or keeps v0: `reaction`).

        Returns [(dx, dy, vx, vy), ...], one per time."""
        segs = self.reaction(pipe) + [(dur, cx, cy) for cx, cy, dur in commands]
        return integrate(v0x, v0y, segs, times, self.accel_mps2)


def integrate(v0x: float, v0y: float, segs, times, accel: float) -> list:
    """Integrate the rate-limited response through `segs` [(duration, cx, cy),
    ...] in order from now (cx None: coast, the velocity held), sampling
    (dx, dy, vx, vy) at each of `times` (sorted, >= 0). Exact."""
    out = []
    k, seg_t, dx, dy, vx, vy = 0, 0.0, 0.0, 0.0, v0x, v0y
    # (dx, dy, vx, vy) at the START of segment k, which starts at seg_t. A
    # segment may carry its own acceleration limit as a fourth element (a
    # BRAKE stop after the GUIDED pipe: `stop_path`).
    for tau in times:
        while k < len(segs) and tau > seg_t + segs[k][0]:
            dur, cx, cy = segs[k][:3]
            if cx is None:
                dx, dy = dx + vx * dur, dy + vy * dur
            else:
                a = segs[k][3] if len(segs[k]) > 3 else accel
                ex, ey, vx, vy = _approach(vx, vy, cx, cy, a, dur)
                dx, dy = dx + ex, dy + ey
            seg_t += dur
            k += 1
        s = tau - seg_t
        if k >= len(segs) or s <= 0.0:
            out.append((dx + vx * max(s, 0.0), dy + vy * max(s, 0.0), vx, vy))
            continue
        _, cx, cy = segs[k][:3]
        if cx is None:
            out.append((dx + vx * s, dy + vy * s, vx, vy))
        else:
            a = segs[k][3] if len(segs[k]) > 3 else accel
            ex, ey, wx, wy = _approach(vx, vy, cx, cy, a, s)
            out.append((dx + ex, dy + ey, wx, wy))
    return out


def pipe_segments(flown, t_now: float, reaction_s: float) -> list:
    """The setpoints already sent that still act during the next
    `reaction_s`, as reaction segments [(duration, cx, cy), ...] covering
    [0, reaction_s) exactly.

    `flown` is [(t_sent, vx, vy), ...] in send order, vx None for a tick on
    which nothing was sent (the autopilot held the aircraft): a setpoint sent
    at t acts from t + reaction_s until the next one acts. Forecast time 0 is
    `t_now`; the command decided now acts from reaction_s, so anything sent
    at or after t_now is not in the pipe. Before the first known setpoint the
    aircraft coasts (None), the form a caller with no history gets."""
    acting = None
    starts = []
    for t, cx, cy in flown:
        if t >= t_now - 1e-9:
            break
        tau = t + reaction_s - t_now
        c = None if cx is None else (float(cx), float(cy))
        if tau <= 1e-12:
            acting = c
        else:
            starts.append((tau, c))
    segs, t0, cur = [], 0.0, acting
    for tau, c in starts:
        if tau > t0 + 1e-12:
            segs.append((tau - t0, *(cur if cur is not None else (None, None))))
            t0 = tau
        cur = c
    if reaction_s > t0 + 1e-12:
        segs.append((reaction_s - t0, *(cur if cur is not None else (None, None))))
    return segs


def _approach(vx: float, vy: float, cx: float, cy: float, a: float,
              s: float) -> tuple[float, float, float, float]:
    """Move velocity (vx, vy) towards (cx, cy) at most `a` m/s^2 for `s`
    seconds: (dx, dy, vx_end, vy_end), exact."""
    ddx, ddy = cx - vx, cy - vy
    dmag = math.hypot(ddx, ddy)
    if dmag <= 1e-12 or s <= 0.0:
        return vx * s, vy * s, (cx if dmag <= 1e-12 else vx), (cy if dmag <= 1e-12 else vy)
    ux, uy = ddx / dmag, ddy / dmag
    t_conv = dmag / a
    if s < t_conv:
        h = 0.5 * a * s * s
        return (vx * s + ux * h, vy * s + uy * h, vx + ux * a * s, vy + uy * a * s)
    h = 0.5 * a * t_conv * t_conv
    rest = s - t_conv
    return (vx * t_conv + ux * h + cx * rest, vy * t_conv + uy * h + cy * rest, cx, cy)


# --------------------------------------------------------------------------- #
# The measured models (tools/fit_velocity_response.py; the tables in
# docs/DESIGN-own-velocity-forecast.md). The fixture test re-derives both from
# tests/fixtures/pas_ap_response_id.jsonl.gz.
# --------------------------------------------------------------------------- #

# The Shield's numbers: the most accurate grid point whose braking forecast,
# in the form the Shield flies (the pipe, then the candidate), is not short
# of a braking aircraft (median along-track error at 1.5 s >= 0 on the 197
# decelerating windows: +0.03 m). The identified aircraft (the physical
# form's minimum, 0.35 s / 2.0 m/s^2) is 0.25 m short in that form and is
# NOT what the Shield forecasts with; it is the closed loop's plant
# (tools/own_velocity_closed_loop.py PLANT_GUIDED).
ARDUCOPTER_GUIDED = VelocityResponse(
    reaction_s=0.45, accel_mps2=1.75,
    source=("ArduCopter V4.5.7 SITL, GUIDED velocity setpoints, Project AirSim "
            "CityLife quad (citylife-quad.param): each setpoint flown from "
            "0.45 s after it is sent, at most 1.75 m/s^2; 6 recorded pas_ap_* "
            "flights, 550 step windows, chosen as the most accurate grid point "
            "whose braking forecast in the Shield's form is not short "
            "(tools/fit_velocity_response.py; identified aircraft 0.35 s, "
            "2.0 m/s^2)"))

# BRAKE: after the GUIDED setpoints already in the pipe, a stop at no more
# than 0.95 m/s^2 - about half of what a GUIDED zero setpoint brakes at on
# the same airframe.
ARDUCOPTER_BRAKE = VelocityResponse(
    reaction_s=0.0, accel_mps2=0.95,
    source=("ArduCopter V4.5.7 SITL BRAKE mode, Project AirSim CityLife quad: "
            "envelope of the 2 recorded BRAKE entries (pas_ap_close_on ticks "
            "280 and 387) after the GUIDED pipe of ARDUCOPTER_GUIDED "
            "(tools/fit_velocity_response.py)"))


def stop_path(guided: VelocityResponse, brake: VelocityResponse | None, x: float, y: float,
              vx: float, vy: float, pipe=None, dt: float = 0.1) -> list:
    """Where an aircraft at (x, y) moving at (vx, vy) goes once told to stop:
    [(t, x, y), ...] on a `dt` grid from now to the stop (the last sample
    exactly at it).

    The setpoints already sent (`pipe`, `pipe_segments`) are flown first, at
    the GUIDED response's acceleration - without a pipe the aircraft keeps
    its velocity for the GUIDED reaction time. Then either a GUIDED stop
    (`brake` None: a zero setpoint at the GUIDED limit) or ArduCopter BRAKE
    (`brake` the ARDUCOPTER_BRAKE envelope: a further `reaction_s` of coast,
    then zero at its own acceleration limit)."""
    segs = guided.reaction(pipe)
    if brake is None:
        tail = [(math.inf, 0.0, 0.0, guided.accel_mps2)]
        a_end, lag_end = guided.accel_mps2, 0.0
    else:
        tail = [(brake.reaction_s, None, None), (math.inf, 0.0, 0.0, brake.accel_mps2)]
        a_end, lag_end = brake.accel_mps2, brake.reaction_s
    t_pipe = sum(s[0] for s in segs)
    _, _, ex, ey = integrate(vx, vy, segs + tail, [t_pipe], guided.accel_mps2)[0]
    t_stop = t_pipe + lag_end + math.hypot(ex, ey) / a_end
    times, t = [], 0.0
    while True:
        times.append(t)
        if t >= t_stop - 1e-9:
            break
        t = min(t + dt, t_stop)
    return [(tt, x + dx, y + dy) for tt, (dx, dy, _, _) in
            zip(times, integrate(vx, vy, segs + tail, times, guided.accel_mps2))]


def hold_plan(shield, state, *, brake: VelocityResponse | None = None,
              handover_mps: float | None = None) -> dict:
    """THE HOLD THAT STOPS CLEAR: should the escalation FSM's Loiter be flown
    as ArduCopter BRAKE now, or as a GUIDED stop through the Shield until
    BRAKE's own stop would end legal?

    On this rail the FSM's Loiter is flown as BRAKE (sitl/mavlink_adapter_node
    DEFAULT_HOLD_MODE: ArduCopter's LOITER sinks with no pilot), and BRAKE on
    the Project AirSim quad brakes far more gently than a GUIDED zero
    setpoint: at pas_ap_close_on tick 387 the FSM went Brake -> Loiter at
    2.7 m/s with the position still legal, BRAKE carried the aircraft 2.9 m
    on - into the P0 building margin, 3.4 m deep by the X7 RTL two seconds
    later - and it was still moving at 1 m/s.

    The plan predicts BRAKE's stop path from the measured velocity and the
    setpoints already in the pipe (`stop_path` with the ARDUCOPTER_BRAKE
    envelope; the pipe is the Shield's record of what the rail flew,
    `Shield.note_flown`) and asks the Shield, pose by pose, whether a stop
    there breaks a hard, enforced rule (`Shield.stop_breaks`, the FSM's
    stop_illegal reading). Clear: "brake", fly BRAKE as before. Not clear:
    "guided" - stay in GUIDED and stream the Shield's answer to a stop
    (`Shield.advise(state, zero)`), which is a stop where the GUIDED stop
    ends legal and the recovery its stop point needs where it does not; the
    rail asks again every tick and hands over to BRAKE once BRAKE's stop
    would end clear. The FSM's states and edges are untouched: this is how
    the rail FLIES its Loiter, as --hold-mode already chooses between BRAKE
    and LOITER.

    BRAKE is also not handed the aircraft while it is still faster than
    `handover_mps` (HOLD_HANDOVER_MPS): the BRAKE envelope rests on two
    entries (one of them not a clean deceleration), and in the first build's
    closed loop a stop path predicted clear at 1.4 m/s ended 1.6 cm inside
    the margin, where BRAKE then held it. With the current numbers the gate
    changes nothing on the two recorded approaches (the same ticks and
    clearances with it set to 99 m/s); it is kept as a margin on an envelope
    of two samples: a GUIDED stop (1.75 m/s^2, against BRAKE's 0.95) brings
    the aircraft under that speed within about a second, and the
    prediction's error shrinks with the speed.

    A deferred hold is flown by the rail's stream, so the rail hands it to
    BRAKE whatever the plan says on a stale link, a gap in the stream or the
    deferral cap (hold_forced_handover, below).

    `state` must carry a measured velocity; without one there is nothing to
    predict from and the plan is "brake" (the old hold), said so in `why`."""
    from .models import State                        # deferred: models <- shield <- here
    brake = ARDUCOPTER_BRAKE if brake is None else brake
    guided = getattr(shield, "velocity_response", None) or ARDUCOPTER_GUIDED
    vel = getattr(state, "velocity", None)
    if vel is None or not (math.isfinite(vel[0]) and math.isfinite(vel[1])):
        return {"choice": "brake", "why": "no measured velocity: BRAKE as before",
                "stop_xy": None, "stop_s": None, "breaks": [], "illegal_at": None}
    speed = math.hypot(vel[0], vel[1])
    handover_mps = HOLD_HANDOVER_MPS if handover_mps is None else handover_mps
    pipe = shield.pipe() if hasattr(shield, "pipe") else None
    path = stop_path(guided, brake, state.x, state.y, vel[0], vel[1], pipe=pipe,
                     dt=shield.dt)
    breaks, at = [], None
    for t, x, y in path:
        b = shield.stop_breaks(State(x=x, y=y, up=state.up, yaw_deg=state.yaw_deg))
        if b:
            breaks, at = b, (round(t, 3), round(x, 3), round(y, 3))
            break
    t_end, x_end, y_end = path[-1]
    fast = speed > handover_mps
    out = {"choice": "guided" if (breaks or fast) else "brake",
           "stop_xy": (round(x_end, 3), round(y_end, 3)), "stop_s": round(t_end, 3),
           "breaks": breaks, "illegal_at": at, "measured_speed_mps": round(speed, 3)}
    if breaks:
        out["why"] = (f"BRAKE's predicted stop path breaks {', '.join(breaks)} at "
                      f"t+{at[0]:.1f} s: held in GUIDED through the Shield until it "
                      f"would not")
    elif fast:
        out["why"] = (f"BRAKE's predicted stop ({t_end:.1f} s) ends legal, but at "
                      f"{speed:.2f} m/s (over {handover_mps:g}) the GUIDED stop brakes "
                      f"first")
    else:
        out["why"] = f"BRAKE's predicted stop ({t_end:.1f} s) ends legal"
    return out


# The speed under which the hold is handed to ArduCopter BRAKE (hold_plan).
HOLD_HANDOVER_MPS = 0.5

# A DEFERRED HOLD IS FLOWN BY THE RAIL'S STREAM (2026-10-20 review). While the
# Loiter is held in GUIDED, ArduCopter has not been asked for BRAKE: if the
# companion then stops streaming - a stale link, a stalled loop - the
# autopilot keeps flying the last GUIDED setpoint until its own GUID_TIMEOUT
# (3 s), where under the old flow BRAKE held the aircraft with no companion
# involvement. So the hold is handed to BRAKE whatever the plan says when
#   * the stream has a gap longer than HOLD_STREAM_GAP_S (three nominal
#     periods; the rail's ticks are 0.109 s median, and the 14 Oct log has a
#     0.73 s gap), or the link goes stale (the rail's stale branch);
#   * the deferral has lasted HOLD_DEFER_MAX_S: GUID_TIMEOUT itself, and
#     longer than any deferral the closed loop needed (1.84 s at most, every
#     recorded pas_ap flight whole, on the identified aircraft, on the
#     Shield's own model and on a slower one; tools/own_velocity_closed_loop.py).
HOLD_STREAM_GAP_S = 0.3
HOLD_DEFER_MAX_S = 3.0


def hold_forced_handover(*, deferred_s: float, stream_gap_s: float | None,
                         defer_max_s: float = HOLD_DEFER_MAX_S,
                         gap_max_s: float = HOLD_STREAM_GAP_S) -> tuple[str | None, str | None]:
    """(reason key, why) when a deferred hold must go to BRAKE whatever the
    plan says (HOLD_STREAM_GAP_S, HOLD_DEFER_MAX_S), else (None, None).
    `deferred_s`: how long the hold has been deferred; `stream_gap_s`: the
    time since the rail last sent a setpoint (None: unknown)."""
    if stream_gap_s is not None and stream_gap_s > gap_max_s:
        return "stream_gap", (f"no setpoint for {stream_gap_s:.2f} s (over {gap_max_s:g} s): "
                              f"the stream that flies the deferred hold faltered")
    if deferred_s >= defer_max_s:
        return "deferral_cap", (f"held in GUIDED for {deferred_s:.1f} s (cap "
                                f"{defer_max_s:g} s, ArduCopter's GUID_TIMEOUT)")
    return None, None


def hold_banner_reason(plan: dict) -> str:
    """The on-screen reason a hold is deferred, short enough for the video
    banner: the rule BRAKE's predicted stop would end in, or the speed BRAKE
    is waiting for."""
    if plan.get("breaks"):
        return "BRAKE WOULD END IN " + ", ".join(plan["breaks"])
    speed = plan.get("measured_speed_mps")
    if speed is not None:
        return f"BRAKE AT UNDER {HOLD_HANDOVER_MPS:g} M/S, NOW {speed:.1f}"
    return "BRAKE DEFERRED"
