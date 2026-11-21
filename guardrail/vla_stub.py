"""
Stub VLA — stands in the VLA Backend slot until a real model arrives.

Interface contract (the part that must NOT change when the real VLA lands):

    observation (State [+ camera frame])  ->  Action4D at 10 Hz

The stub is a naive proportional controller that flies straight at the target.
It is DELIBERATELY rule-ignorant and a bit reckless (asks for more speed than
the policy allows): its job is to behave like an untrusted model so the
Guardrail has something real to catch.

THE STRESS HARNESS'S PILOTS (2026-11-06)

The scenario library (experiments/scenarios.yaml) flies three pilot laws:
`stub_vla` (StubVLA below), `goto` and `constant`. The last two lived in
experiments/sweep_scenarios.py `_pilot` until the hil rail had to fly them
too (sitl/ros2_vla_stub_node.py --pilot): they live here now, and both the
harness and the rail's VLA stub call `goto_action` / `constant_action`, so a
scenario is flown by one law on every backend. Here, not in the harness,
because the rail's VLA identity names its pilot by a dotted id that
guardrail.manifest.model_hash pins by this file's source (`MODEL_IDS`):
"guardrail.vla_stub.GotoPilot@src:<sha256 of this file>". All three laws
answer in the WORLD frame (x North, y East, up); the rail's stub turns the
answer into the grant's body frame (guardrail.frames.to_body).

THE GOTO PILOT OVER A DECLARED GROUND (2026-11-21, X-11)

`goto_action(..., terrain=TerrainFeed(dem, ...))` flies the goto law over a
declared ground (guardrail/terrain.py, a scenario's or bundle's DEM): the
target's altitude is read as a height above the ground under the aircraft,
and the climb is that height's error plus a FEED-FORWARD of the ground's
rise along the commanded horizontal velocity (TERRAIN_FEED_HORIZON_S ahead),
clamped at GOTO_CLIMB_MPS - the climb cap of every msl_dem policy
(kin-caps climb_rate_max_mps 2.0), so the command stays inside the kinematic
envelope. Why: on the sitl / hil rails the stub's pose is a height above
HOME (MAVROS local z), so before this the goto law held its target height
above home - level in MSL - and over the msl-dem-zone-ridge crest (602.5 m
MSL, home 583.8 m) the Shield's AGL-band repair asked for a climb whose
magnitude (|dvz| x the 5 s lookahead) was over the FSM's vertical theta
0.5 m on every tick: projection abandoned, Loiter, its operator timeout,
RTL - the target never reached (msl_dem_sitl_01..03, the 19 Nov sitl / hil2
sweeps). Tracking the ground, the law asks the Shield for no repair over a
ridge it can climb. Opt-in only: `terrain=None` (every caller before this
date, and every cell without a declared ground) is the law exactly as it
was, bit for bit. Theta and the FSM are untouched.
"""
from __future__ import annotations

import math

from .compiler import Mission
from .models import Action4D, State

# The stress harness's goal tolerance (experiments/sweep_scenarios.py
# GOAL_TOL_M): the goto pilot stops commanding a horizontal velocity inside
# it, and a goto or stub_vla scenario counts its goal reached inside it.
GOAL_TOL_M = 3.0
# ArduCopter's horizontal position gain (PSC_POSXY_P, AC_PosControl at the
# Copter-4.5.7 pin), the gain of the goto pilot's approach law; equal to
# experiments/sweep_scenarios.py AP_POSXY_P (tests/test_sweep.py pins it).
APPROACH_P = 1.0
# The measured ArduPilot response's horizontal acceleration (sweep_scenarios
# AP_RESPONSE.accel_mps2, fitted to the recorded Copter-4.5.7 flights): a
# goto pilot flown on a vehicle that carries its speed slows into its goal at
# it. The hil rail's stub flies goto with it (GotoPilot's default): the ideal
# law commands full speed until GOAL_TOL_M and zero inside it, which a real
# airframe overshoots by its braking distance.
APPROACH_ACCEL_MPS2 = 2.4
# The goto pilot's climb clamp toward its target's altitude, m/s.
GOTO_CLIMB_MPS = 2.0
# How far ahead (seconds of the commanded horizontal velocity) the terrain
# feed-forward reads the ground's rise (TerrainFeed.rise_rate, 2026-11-21):
# one second, ten ticks, about one DEM cell at the goto pilot's 3 m/s on
# experiments/worlds/dem_cmac_wgs84.json (9.08 m cells).
TERRAIN_FEED_HORIZON_S = 1.0
# The dotted ids the rail's VLA stub announces for each law (its
# /vla/identity `model_id`): code-only pilots, each pinned by this module's
# source (guardrail.manifest._source_of resolves the module, not the symbol).
MODEL_IDS = {"stub_vla": "guardrail.vla_stub.StubVLA",
             "goto": "guardrail.vla_stub.GotoPilot",
             "constant": "guardrail.vla_stub.ConstantPilot"}
CONSTANT_CHANNELS = ("vx", "vy", "vz_up", "yaw_rate")


class StubVLA:
    def __init__(self, mission: Mission):
        self.mission = mission

    def act(self, state: State) -> Action4D:
        dx = self.mission.target_x - state.x
        dy = self.mission.target_y - state.y
        dist = (dx * dx + dy * dy) ** 0.5
        if dist < 1e-6:
            return Action4D()
        speed = min(self.mission.speed_pref_mps, dist)   # slow near target
        vz = 0.8 * (self.mission.cruise_alt_m - state.up)  # hold cruise altitude
        return Action4D(vx=dx / dist * speed, vy=dy / dist * speed, vz_up=vz)


def sqrt_controller(error: float, p: float, second_ord_lim: float, dt: float) -> float:
    """The correction rate a non-negative `error` asks for: linear (p * error)
    near zero, the rate of a constant-`second_ord_lim` approach
    (sqrt(2 * lim * (error - linear / 2))) beyond, and never more than closes
    the error within `dt`. The shape of ArduPilot's AP_Math sqrt_controller
    (control.cpp), which AC_PosControl's velocity shaping is built on; written
    here for a vector's length, so for error >= 0 only. (The stress harness's
    vehicle response is built on it too: experiments/sweep_scenarios.py
    `_sqrt_controller` is this function.)"""
    if not (second_ord_lim > 0.0) or math.isinf(second_ord_lim):
        rate = error * p if math.isfinite(p) else math.inf
    elif p <= 0.0:
        rate = math.sqrt(2.0 * second_ord_lim * error)
    else:
        linear = second_ord_lim / (p * p)
        rate = (math.sqrt(2.0 * second_ord_lim * (error - linear / 2.0))
                if error > linear else error * p)
    return min(rate, error / dt) if dt > 0.0 else rate


class TerrainFeed:
    """The goto pilot's view of a declared ground (2026-11-21, X-11; module
    docstring, THE GOTO PILOT OVER A DECLARED GROUND): how high the aircraft
    is above the ground under it (`agl`) and how fast that ground rises
    along a horizontal velocity (`rise_rate`).

    `ground`: a DEM (guardrail.terrain.is_dem: a GridGround, placed in the
    policy's frame). `home_msl_m` says what State.up the pilot is handed is:
    None, the height above the ground under the aircraft already (the
    headless harness, whose integrator keeps `up` so); a number, the height
    above the autopilot's HOME at that elevation MSL, standing at `home_xy`
    in the policy frame (the ROS 2 rails' MAVROS local z;
    sitl/ros2_vla_stub_node.py --terrain-feed on), converted by
    guardrail.terrain.TerrainReference - which refuses a home elevation the
    DEM disagrees with by more than GROUND_HOME_TOL_M, as the Shield node
    does. A flat ground is refused: its feed-forward is zero and a rail's
    height above home is already its height above it.

    Pure and deterministic: every answer is a function of the arguments."""

    def __init__(self, ground, home_msl_m: float | None = None,
                 home_xy: tuple[float, float] = (0.0, 0.0),
                 horizon_s: float = TERRAIN_FEED_HORIZON_S) -> None:
        from .terrain import TerrainReference, is_dem
        if not is_dem(ground):
            raise ValueError(f"{ground!r} is not a DEM: the goto pilot's terrain feed "
                             f"needs a ground that varies with position")
        h = float(horizon_s)
        if not (math.isfinite(h) and h > 0.0):
            raise ValueError(f"a terrain feed horizon must be a positive number, "
                             f"got {horizon_s!r}")
        self.ground = ground
        self.horizon_s = h
        self.ref = (None if home_msl_m is None else
                    TerrainReference(ground, home_msl_m, home_xy=home_xy,
                                     source="the rail's home elevation"))

    def agl(self, st: State) -> float:
        """The height (m) above the ground under the aircraft in `st`."""
        if self.ref is None:
            return float(st.up)
        return self.ref.up(st.up, st.x, st.y)

    def rise_rate(self, st: State, vx: float, vy: float) -> float:
        """How fast (m/s) the ground under an aircraft at `st` flying the
        horizontal velocity (vx, vy) rises: its rise over the next
        `horizon_s` of that velocity, divided by `horizon_s` (negative where
        it falls). 0.0 for no horizontal velocity."""
        if vx == 0.0 and vy == 0.0:
            return 0.0
        h = self.horizon_s
        g = self.ground.elevation_at
        return (g(st.x + vx * h, st.y + vy * h) - g(st.x, st.y)) / h

    def record(self) -> dict:
        """What a run record says the pilot flew over."""
        rec = {"horizon_s": self.horizon_s, "climb_clamp_mps": GOTO_CLIMB_MPS,
               "ground_sha256": self.ground.record().get("sha256"),
               "up_ref": "terrain" if self.ref is None else "home"}
        if self.ref is not None:
            rec.update(home_msl_m=self.ref.home_msl_m, home_xy=list(self.ref.home_xy))
        return rec


def goto_action(target, st: State, speed: float, approach_accel: float | None = None,
                goal_tol_m: float = GOAL_TOL_M,
                terrain: TerrainFeed | None = None) -> Action4D:
    """The `goto` pilot: straight at `target` (x, y, up or None) at `speed`,
    blind to every rule; nothing horizontal inside `goal_tol_m`, and a climb
    toward the target's altitude clamped at GOTO_CLIMB_MPS (none when the
    target names no altitude).

    `approach_accel` None is the bang-bang law the headless harness flies on
    its ideal vehicle. A number slows the approach to the rate a constant
    deceleration of that size allows (sqrt_controller of the remaining
    distance, APPROACH_P): the harness's vehicle-response arm, and the flight
    rails, whose vehicle carries its speed (APPROACH_ACCEL_MPS2).

    `terrain` (a TerrainFeed, 2026-11-21, X-11; None by default and for
    every cell without a declared ground, which is then the law above to the
    bit): the target's altitude is a height above the ground under the
    aircraft (terrain.agl), and the climb adds the ground's rise along the
    commanded horizontal velocity (terrain.rise_rate, a feed-forward), the
    sum clamped at GOTO_CLIMB_MPS. Without a target altitude there is no
    climb either way. Still blind to every rule: a ground is not a rule."""
    gx, gy, gup = target
    dx, dy = gx - st.x, gy - st.y
    d = math.hypot(dx, dy)
    v = speed
    if approach_accel is not None:
        v = min(v, sqrt_controller(d, APPROACH_P, approach_accel, 0.0))
    vx, vy = ((0.0, 0.0) if d < 1e-6 else
              (v * dx / d, v * dy / d))
    if d < goal_tol_m:
        vx = vy = 0.0
    vz = 0.0
    if gup is not None and terrain is None:
        vz = max(-GOTO_CLIMB_MPS, min(GOTO_CLIMB_MPS, float(gup) - st.up))
    elif gup is not None:
        want = terrain.rise_rate(st, vx, vy) + (float(gup) - terrain.agl(st))
        vz = max(-GOTO_CLIMB_MPS, min(GOTO_CLIMB_MPS, want))
    return Action4D(vx=vx, vy=vy, vz_up=vz)


def constant_action(action: dict | None) -> Action4D:
    """The `constant` pilot: the same world-frame action every tick, whatever
    the state (a non-finite channel included: the action-contract family asks
    for one on purpose)."""
    return Action4D(**(action or {}))


class GotoPilot:
    """`goto_action` as a pilot object (`act(state)`), for the rail's stub.
    `approach_accel` defaults to APPROACH_ACCEL_MPS2: on a flight rail the
    vehicle carries its speed (see `goto_action`). `terrain`: a TerrainFeed
    (2026-11-21, X-11) the law flies over, None (the default) for none."""

    def __init__(self, target, speed: float,
                 approach_accel: float | None = APPROACH_ACCEL_MPS2,
                 goal_tol_m: float = GOAL_TOL_M,
                 terrain: TerrainFeed | None = None) -> None:
        t = tuple(target)
        if len(t) not in (2, 3):
            raise ValueError(f"a goto target is (x, y) or (x, y, up), got {target!r}")
        up = t[2] if len(t) == 3 else None
        self.target = (float(t[0]), float(t[1]), None if up is None else float(up))
        if not all(math.isfinite(v) for v in self.target if v is not None):
            raise ValueError(f"a goto target must be finite, got {target!r}")
        self.speed = float(speed)
        if not (math.isfinite(self.speed) and self.speed > 0.0):
            raise ValueError(f"a goto speed must be a positive number, got {speed!r}")
        self.approach_accel = approach_accel
        self.goal_tol_m = float(goal_tol_m)
        self.terrain = terrain

    def act(self, state: State) -> Action4D:
        return goto_action(self.target, state, self.speed, self.approach_accel,
                           self.goal_tol_m, terrain=self.terrain)


class ConstantPilot:
    """`constant_action` as a pilot object, for the rail's stub. A flight
    rail refuses a non-finite action (the Shield node will not fly or count
    one), so this one is refused here, before anything flies."""

    def __init__(self, action: dict) -> None:
        bad = set(action) - set(CONSTANT_CHANNELS)
        if bad:
            raise ValueError(f"unknown action channel(s) {sorted(bad)}; "
                             f"one of {list(CONSTANT_CHANNELS)}")
        vals = {k: float(v) for k, v in action.items()}
        if not all(math.isfinite(v) for v in vals.values()):
            raise ValueError(f"a constant action must be finite on a flight rail, "
                             f"got {action!r}")
        self.action = vals

    def act(self, state: State) -> Action4D:
        return constant_action(self.action)
