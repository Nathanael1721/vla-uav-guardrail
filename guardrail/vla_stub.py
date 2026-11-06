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


def goto_action(target, st: State, speed: float, approach_accel: float | None = None,
                goal_tol_m: float = GOAL_TOL_M) -> Action4D:
    """The `goto` pilot: straight at `target` (x, y, up or None) at `speed`,
    blind to every rule; nothing horizontal inside `goal_tol_m`, and a climb
    toward the target's altitude clamped at GOTO_CLIMB_MPS (none when the
    target names no altitude).

    `approach_accel` None is the bang-bang law the headless harness flies on
    its ideal vehicle. A number slows the approach to the rate a constant
    deceleration of that size allows (sqrt_controller of the remaining
    distance, APPROACH_P): the harness's vehicle-response arm, and the flight
    rails, whose vehicle carries its speed (APPROACH_ACCEL_MPS2)."""
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
    if gup is not None:
        vz = max(-GOTO_CLIMB_MPS, min(GOTO_CLIMB_MPS, float(gup) - st.up))
    return Action4D(vx=vx, vy=vy, vz_up=vz)


def constant_action(action: dict | None) -> Action4D:
    """The `constant` pilot: the same world-frame action every tick, whatever
    the state (a non-finite channel included: the action-contract family asks
    for one on purpose)."""
    return Action4D(**(action or {}))


class GotoPilot:
    """`goto_action` as a pilot object (`act(state)`), for the rail's stub.
    `approach_accel` defaults to APPROACH_ACCEL_MPS2: on a flight rail the
    vehicle carries its speed (see `goto_action`)."""

    def __init__(self, target, speed: float,
                 approach_accel: float | None = APPROACH_ACCEL_MPS2,
                 goal_tol_m: float = GOAL_TOL_M) -> None:
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

    def act(self, state: State) -> Action4D:
        return goto_action(self.target, state, self.speed, self.approach_accel,
                           self.goal_tol_m)


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
