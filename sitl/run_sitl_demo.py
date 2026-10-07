"""
Fase 4 — the SAME Guardrail pipeline, now through a REAL autopilot.

    User Command -> Constraint Compiler -> YAML Prompt -> VLA (stub)
                 -> Safety Shield -> MAVLink SET_POSITION_TARGET_LOCAL_NED
                 -> ArduPilot SITL (real flight code) -> simulated copter

What changed vs demo/run_demo.py: ONLY the bottom adapter. AirSim's toy
velocity API is replaced by pymavlink talking to ArduPilot's GUIDED mode —
exactly the grant's "MAVLink adapter" box. Guardrail code is untouched,
which is itself the point: the Shield doesn't care what flies under it.

Run inside WSL (SITL listening on tcp:127.0.0.1:5760):

    ~/venv-ap/bin/python run_sitl_demo.py --shield on
    ~/venv-ap/bin/python run_sitl_demo.py --shield off
    ~/venv-ap/bin/python run_sitl_demo.py --shield on --dynamic
    ~/venv-ap/bin/python run_sitl_demo.py --bundle ../bundles/fase3-sim-demo-v0.1.1.tar.gz

Policy note: `--bundle` flies the signed policy bundle (guardrail/bundle.py) and
refuses one whose signature does not verify, unless `--allow-unverified-bundle`
says to fly it recorded as unverified. This venv has no `cryptography`; the
signature is checked by the pure-Python RFC 8032 verifier in guardrail/bundle.py
instead, so "verified" means the same here as on Windows. `--policy`
(default policies/sim_demo_policy.yaml) is the YAML fallback, recorded as
unsigned. Either way the source lands in metrics.json, kpi.json and the replay
bundle.

Frames note: SITL has no camera — that stays AirSim's job (perception rail vs
functional rail, same split the grant makes).

Records (since 2026-10-06, the same as the ROS 2 rail): every row carries
`unsafe` / `unsafe_rules`, policy_hash + generation, the Shield's own time
(shield_ms) and the declared subject; every policy generation and its CSP is
written (guardrail.replay.EpisodeRecord), the audit log is fresh per episode,
the autopilot's own version (AUTOPILOT_VERSION) goes into metrics.json, the
seed is an argument, and with the Shield on the escalation FSM requests its
modes through pymavlink. This rail stays world-frame in-process (StubVLA and
the Shield share one process); the grant's body-frame ROS interface is
sitl/ros2_shield_node.py.

Since 2026-10-07, as on the ROS 2 rail: the Shield looks 5 s ahead at 0.1 s
(the grant's 50 poses); every row carries the episode id and `flown`; the
episode ends with a mission_end event; the GeoFence parameters are read back
from the autopilot (FENCE_ALT_MAX raised above the policy's ceiling if
needed); a KPI-grade run whose replay bundle is not signed is demoted; and
this rail runs the one escalation FSM, its Shield built without one
(guardrail.replay.rail_shield), with each audit record carrying the rail's FSM
verdict and the episode id (guardrail.replay.audit_tick).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sitl"))

from guardrail import AuditLogger, State                               # noqa: E402
from guardrail.bundle import load_for_flight                           # noqa: E402
from guardrail.compiler import ConstraintCompiler                      # noqa: E402
from guardrail.fsm import (FAILSAFE_STATES, EscalationFSM, FSMConfig,  # noqa: E402
                           tick_input_from_decision)
from guardrail.geometry import fence_polygon                           # noqa: E402
from guardrail.kpi import compute_from_dir                             # noqa: E402
from guardrail.manifest import (TOPOLOGY_ARDUPILOT_SITL,               # noqa: E402
                                ardupilot_version_from_mavlink,
                                build_manifest, is_kpi_grade,
                                sim_speedup_from_mavlink)
from guardrail.models import (AltitudeEnvelope, PolygonFence,
                              SubjectStandoff, XY)        # noqa: E402
from guardrail.models import Action4D                                  # noqa: E402
from guardrail.replay import (EpisodeRecord, audit_tick,               # noqa: E402
                              episode_row_fields, flown_fields,
                              kpi_evidence_grade, rail_shield,
                              verify_replay, write_replay)
from guardrail.vla_stub import StubVLA                                 # noqa: E402
from mavlink_adapter_node import (fence_alt_plan,                      # noqa: E402
                                  policy_ceiling_m)

from shapely.geometry import Point                                     # noqa: E402

TICK = 0.1
MAX_S = 120
REACH_M = 2.0
# The grant's Safety Shield horizon: 5 s at 10 Hz, 50 future poses (3 s at
# 0.5 s until 2026-10-07), as on the ROS 2 rail.
LOOKAHEAD_S = 5.0
LOOKAHEAD_DT_S = 0.1
FENCE_PARAMS = ("FENCE_ENABLE", "FENCE_ALT_MAX", "FENCE_ACTION", "FENCE_TYPE",
                "AVOID_ENABLE")
# The FSM's theta horizon on this rail: the velocity proxy, as on the ROS 2
# rail, until shield.py reports a per-operator magnitude (see
# docs/DESIGN-escalation-fsm.md). At 0.1 s theta cannot fire here.
PROXY_HORIZON_S = 0.1
# After the FSM entered RTL / Land, how long to keep recording.
FAILSAFE_GRACE_S = 40.0

DYNAMIC_AT_S = 8.0
DYNAMIC_FENCE = PolygonFence(
    id="nfz-dynamic", type="polygon_fence",
    vertices=[XY(x=23, y=6), XY(x=31, y=6), XY(x=31, y=14), XY(x=23, y=14)],
    margin_m=1.0,
)

# type_mask: use velocity + yaw_rate, ignore position/accel/force/yaw.
# bits(1=ignore): x y z | vx vy vz | ax ay az | force yaw yaw_rate
VEL_YAWRATE_MASK = 0b0101_1100_0111  # 1479


class MavlinkAdapter:
    """The single body/up-positive <-> NED boundary (grant rule: conversion
    lives in the adapter, never in the Shield)."""

    def __init__(self, url: str):
        print(f"[mavlink]  connecting {url} ...")
        self.m = mavutil.mavlink_connection(url)
        self.m.wait_heartbeat()
        print(f"[mavlink]  heartbeat from sys {self.m.target_system}")
        # Without a GCS attached, SITL streams nothing by default — ask for
        # everything at 10 Hz or state() would block forever.
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    def read_param(self, name: str, timeout: float = 3.0) -> float | None:
        """One autopilot parameter, as the autopilot holds it, or None."""
        self.m.mav.param_request_read_send(self.m.target_system,
                                           self.m.target_component,
                                           name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True,
                                    timeout=timeout)
            if msg is not None and msg.param_id.strip("\x00") == name:
                return float(msg.param_value)
        return None

    def set_param(self, name: str, value: float) -> float | None:
        """Set a REAL32 parameter and return the value read back."""
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                  name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.5)
        return self.read_param(name)

    def autopilot_fence(self, policy, cruise_alt_m: float) -> dict:
        """The GeoFence backstop as the autopilot holds it; FENCE_ALT_MAX is
        raised above the policy's ceiling if it sits below it (the same rule
        as the ROS 2 rail, sitl/mavlink_adapter_node.fence_alt_plan)."""
        vals = {n: self.read_param(n) for n in FENCE_PARAMS}
        ceiling, src = policy_ceiling_m(policy, cruise_alt_m)
        plan = fence_alt_plan(ceiling, vals["FENCE_ALT_MAX"])
        raised_from = None
        # SITL only (SIM_SPEEDUP answers): a hardware autopilot's GeoFence is
        # GCS-set and a change would persist; there it is recorded, not fixed.
        is_sitl = (self.read_param("SIM_SPEEDUP") or 0.0) > 0.0
        if plan["raise_to"] is not None and vals["FENCE_ALT_MAX"] is not None \
                and is_sitl:
            got = self.set_param("FENCE_ALT_MAX", plan["raise_to"])
            if got is not None:
                raised_from, vals["FENCE_ALT_MAX"] = vals["FENCE_ALT_MAX"], got
        enabled = vals["FENCE_ENABLE"]
        return {**vals, "source": "PARAM_VALUE (pymavlink)",
                "policy_ceiling_m": ceiling, "policy_ceiling_source": src,
                "fence_alt_max_raised_from": raised_from,
                "raise_allowed": is_sitl,
                "backstop_active": None if enabled is None else enabled >= 1.0,
                "stricter_than_policy": (
                    None if vals["FENCE_ALT_MAX"] is None or ceiling is None
                    else not fence_alt_plan(ceiling, vals["FENCE_ALT_MAX"])["ok"]),
                "note": "this rail uploads no polygons; only the altitude "
                        "fence applies (the ROS 2 rail uploads the zones)"}

    def _ack(self, cmd: int, timeout: float = 3.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
            if msg and msg.command == cmd:
                return msg.result == 0
        return False

    def prepare(self, alt_m: float) -> None:
        """State-machine bring-up: verify GUIDED via heartbeat, keep (re)arming,
        retry takeoff — whatever the EKF timing, converge or time out."""
        mav = self.m.mav
        mode_id = self.m.mode_mapping()["GUIDED"]
        deadline = time.time() + 120
        while time.time() < deadline:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
            if hb is None:
                continue
            if hb.custom_mode != mode_id:              # 1. ensure GUIDED (verified)
                self.m.set_mode(mode_id)
                time.sleep(1)
                continue
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:                              # 2. ensure armed (EKF may refuse)
                mav.command_long_send(self.m.target_system, self.m.target_component,
                                      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                      0, 1, 0, 0, 0, 0, 0, 0)
                time.sleep(2)
                continue
            mav.command_long_send(self.m.target_system, self.m.target_component,
                                  mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                  0, 0, 0, 0, 0, 0, 0, alt_m)  # 3. takeoff
            if self._ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
                print(f"[mavlink]  armed + takeoff accepted, climbing to {alt_m} m ...")
                break
            time.sleep(2)
        else:
            raise RuntimeError("bring-up timed out (mode/arm/takeoff)")

        t0 = time.time()
        last_up = 0.0
        while time.time() - t0 < 60:
            st = self.state(timeout=2)
            if st:
                last_up = st.up
                if st.up >= alt_m - 1.0:
                    print(f"[mavlink]  at {st.up:.1f} m")
                    return
        raise RuntimeError(f"takeoff did not reach altitude (last {last_up:.1f} m)")

    def state(self, timeout: float = 1.0) -> State | None:
        msg = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                                timeout=timeout)
        if msg is None:
            return None
        return State(x=msg.x, y=msg.y, up=-msg.z)      # NED z-down -> up-positive

    def send_velocity(self, vx: float, vy: float, vz_up: float,
                      yaw_rate_rad_s: float) -> None:
        """Emit one velocity setpoint. `yaw_rate_rad_s` is the Action4D field.

        It was named `yaw_rate_dps` and converted with math.radians() until
        2026-09-07, on the strength of a comment in models.py that said the
        contract carried degrees. It carries radians - shield.py enforces the
        cap in radians - so the conversion divided every commanded yaw by 57.3.
        Latent rather than harmful: the stub pilot this rail flies has never
        commanded a non-zero yaw rate on ANY of the 12 runs under demo/out/
        whose topology is ardupilot-sitl-pymavlink or (stored as canonical-hil,
        read as dev since 2026-10-06) the MAVROS rail - 2850
        ticks, max |yaw_rate| exactly 0.0 - so every stored KPI figure is
        unchanged.

        MAVLink's SET_POSITION_TARGET_LOCAL_NED yaw_rate field is rad/s, which
        is what makes pass-through correct here rather than merely simpler.
        """
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, VEL_YAWRATE_MASK,
            0, 0, 0,                       # position (ignored)
            vx, vy, -vz_up,                # velocity, NED
            0, 0, 0,                       # accel (ignored)
            0, yaw_rate_rad_s)

    def land_disarm(self) -> None:
        self.m.set_mode(self.m.mode_mapping()["LAND"])
        print("[mavlink]  LAND")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--command", default="fly to the northeast pad at 6 m/s")
    ap.add_argument("--policy", default=None,
                    help="policy YAML (default policies/sim_demo_policy.yaml); "
                         "recorded as UNSIGNED")
    ap.add_argument("--bundle", default=None,
                    help="signed policy bundle; refused unless its signature "
                         "verifies")
    ap.add_argument("--allow-unverified-bundle", action="store_true",
                    help="fly a bundle whose signature is not verified (unsigned, "
                         "or a key the trust store does not list), recorded as "
                         "unverified; a bad or stripped signature is refused "
                         "regardless")
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--dynamic", action="store_true")
    ap.add_argument("--subject", default=None, metavar="X,Y",
                    help="declare a subject (a person) at this NED position, so "
                         "subject_standoff rules bind. The position is DECLARED, "
                         "not perceived: this rail has no camera, and the point "
                         "is to measure the SHIELD - which the 2026-08-19 review "
                         "confirmed is the deliverable, with the pilot a "
                         "swappable input. Without it the standoff rules are "
                         "inert, which is what they silently were.")
    ap.add_argument("--subject-class", default="pedestrian",
                    help="which subject_standoff rule binds (default pedestrian)")
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--seed", type=int, default=0,
                    help="recorded in the manifest (WP4-12). StubVLA draws no "
                         "random numbers.")
    ap.add_argument("--fsm-config", default=None,
                    help="escalation FSM thresholds YAML (guardrail.fsm.FSMConfig)")
    ap.add_argument("--no-fsm", action="store_true",
                    help="do not run the escalation FSM (the pre-2026-10-06 rail)")
    ap.add_argument("--tag", default=None)
    args = ap.parse_args()

    shield_on = args.shield == "on"
    tag = args.tag or (f"sitl_shield_{args.shield}" + ("_dynamic" if args.dynamic else ""))

    # The signed bundle when one is given, else the YAML (recorded unsigned).
    # Both at once is refused: a run with two candidate policies has none.
    # Loaded BEFORE the output folder is touched, so a refused run leaves an
    # earlier run of the same tag exactly as it was.
    policy, policy_source = load_for_flight(
        args.bundle,
        args.policy or (None if args.bundle
                        else ROOT / "policies" / "sim_demo_policy.yaml"),
        allow_unverified=args.allow_unverified_bundle)
    out = ROOT / "demo" / "out" / tag
    out.mkdir(parents=True, exist_ok=True)
    compiler = ConstraintCompiler(policy)
    mission = compiler.parse_command(args.command)
    # The episode record FIRST: it moves an earlier run's audit and
    # generation files aside, and writes generation 0's policy and CSP.
    rec = EpisodeRecord(out, policy, mission, lookahead_s=LOOKAHEAD_S)
    try:
        (out / "prompt.yaml").write_text(compiler.build_prompt(mission),
                                         encoding="utf-8")
    except ImportError as exc:
        # ~/venv-ap has no jinja2 (sitl/setup_sitl.sh pins it); say so in the
        # record instead of dying before take-off.
        rec.event("prompt_failed", error=f"{type(exc).__name__}: {exc}")
        print(f"[compiler] prompt.yaml not written: {exc}")
    print(f"[compiler] target=({mission.target_x:.0f},{mission.target_y:.0f}) "
          f"alt={mission.cruise_alt_m:.0f}m speed_pref={mission.speed_pref_mps:.0f}m/s")
    print(f"[policy]   {policy.policy_id}  {policy.policy_hash}")
    print(f"[policy]   from {policy_source['kind']} {policy_source['path']} - "
          f"signature {policy_source['signature']}")

    vla = StubVLA(mission)
    # No escalation FSM inside the Shield: this rail runs THE FSM below, fed
    # with the autopilot's state (guardrail.replay.rail_shield).
    shield = rail_shield(policy, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
    # The POLICY, not a snapshot of its hash. This rail hot-applies a fence
    # mid-flight, and AuditLogger reads the hash live so records written after
    # that carry the generation that was actually in force.
    audit = AuditLogger(out / "audit.jsonl", policy,
                        episode_id=str(rec.episode_id))

    subject = None
    if args.subject:
        sx, sy = (float(v) for v in args.subject.split(","))
        subject = (sx, sy)
        shield.set_subject(sx, sy, args.subject_class)
        rings = [(c.id, c.min_range_m) for c in policy.by_type(SubjectStandoff)
                 if c.binds(args.subject_class)]
        print(f"[subject]  {args.subject_class} DECLARED at ({sx:.1f}, {sy:.1f}) "
              f"- position is ground truth, not perception")
        if rings:
            print(f"[subject]  binding rules: " +
                  ", ".join(f"{i} {r:.0f}m" for i, r in rings))
        else:
            # A standoff rule that binds nothing is the failure this whole
            # exercise exists to expose, so it is said out loud.
            print(f"[subject]  WARNING no subject_standoff rule binds "
                  f"'{args.subject_class}' in this policy - the ring is inert")

    link = MavlinkAdapter(args.url)
    # The autopilot's own name for itself (ARCH-31), read before take-off.
    autopilot = ardupilot_version_from_mavlink(link.m)
    print(f"[mavlink]  autopilot: "
          f"{autopilot['ardupilot_version'] if autopilot else 'NOT READ'}")
    rec.event("autopilot_version", **(autopilot or {"ardupilot_version": None}))
    ap_fence = link.autopilot_fence(policy, mission.cruise_alt_m)
    rec.event("autopilot_fence", **ap_fence)
    print(f"[fence]    ENABLE {ap_fence['FENCE_ENABLE']}  ALT_MAX "
          f"{ap_fence['FENCE_ALT_MAX']}"
          + (f" (raised from {ap_fence['fence_alt_max_raised_from']})"
             if ap_fence["fence_alt_max_raised_from"] is not None else "")
          + f"  AVOID_ENABLE {ap_fence['AVOID_ENABLE']}")
    fsm = (EscalationFSM(FSMConfig.from_yaml(args.fsm_config) if args.fsm_config
                         else FSMConfig())
           if shield_on and not args.no_fsm else None)
    fsm_fault = None
    failsafe_t = None
    link.prepare(mission.cruise_alt_m)
    print(f"[flight]   shield={'ON' if shield_on else 'OFF'} — mission start")

    traj: list[dict] = []
    rows: list[dict] = []
    n_touched = n_braked = 0
    spawn_t = spawn_pos = None
    t0 = time.time()
    tick = 0
    reached = False
    end_why = "time cap"
    while time.time() - t0 < MAX_S:
        tick += 1
        now = time.time() - t0
        state = link.state()
        if state is None:
            continue

        if args.dynamic and spawn_t is None and now >= DYNAMIC_AT_S:
            # As a dynamic_nfz where the Shield enforces the grant's
            # mid-flight update model (guardrail.replay.EpisodeRecord.apply_zone).
            rec.apply_zone(shield, DYNAMIC_FENCE, t=now)
            # No restamp needed: AuditLogger holds the policy and reads the hash
            # at log time. Assigning here raised AttributeError once policy_hash
            # became a read-only property, which killed every --dynamic run at
            # exactly this line.
            spawn_t, spawn_pos = now, (state.x, state.y)
            print(f"[dynamic]  t={now:.1f}s '{DYNAMIC_FENCE.id}' applied "
                  f"-> generation {policy.generation}")

        dist = math.hypot(mission.target_x - state.x, mission.target_y - state.y)
        if dist < REACH_M:
            reached = True
            end_why = "target reached"
            print(f"[flight]   target reached at tick {tick} (t={now:.1f}s)")
            break

        raw = vla.act(state)

        # The Shield ALWAYS evaluates; only enforcement is conditional.
        #
        # It used to run only when the shield was on, which left the control run
        # with no record of which rules the flown action broke. guardrail/kpi.py
        # counts a P0 escape from the violations seen on a tick, so a
        # guardrail-free flight with no violations logged scores as perfectly
        # clean - the opposite of what the A/B is for.
        decision = shield.filter(state, raw)
        emitted = decision.emitted if shield_on else raw
        extra = episode_row_fields(shield, state, policy, rec.episode_id)

        # Escalation FSM (Shield on only); modes through pymavlink.
        fsm_out, setpoint = None, "pass"
        if fsm is not None and fsm_fault is None:
            try:
                in_failsafe = fsm.state in FAILSAFE_STATES
                landed = in_failsafe and not link.m.motors_armed()
                fsm_out = fsm.step(tick_input_from_decision(
                    now, decision, policy, horizon_s=PROXY_HORIZON_S,
                    home_reached=(in_failsafe
                                  and math.hypot(state.x, state.y) < REACH_M),
                    landed=landed,
                    stop_illegal=shield.state_is_unsafe(state)))
                setpoint = fsm_out.setpoint
                if fsm_out.transition:
                    rec.event("fsm_transition", t=now,
                              **{k: fsm_out.record[k] for k in (
                                  "fsm_state_before", "fsm_state_after",
                                  "transition", "edge", "reason")})
                if fsm_out.set_mode:
                    link.m.set_mode(link.m.mode_mapping()[fsm_out.set_mode])
                    rec.event("mode_request", t=now, mode=fsm_out.set_mode,
                              why=fsm_out.reason)
                if fsm_out.state in FAILSAFE_STATES and failsafe_t is None:
                    failsafe_t = now
            except ValueError as exc:
                fsm_fault = str(exc)
                rec.event("fsm_fault", t=now, error=fsm_fault)
                link.m.set_mode(link.m.mode_mapping()["LOITER"])
                print(f"[fsm]      input refused, LOITER: {exc}")
        if fsm_fault is not None:
            setpoint = "none"
        # After the FSM stepped, with its verdict: one state machine on record.
        audit_tick(audit, tick, decision, fsm_out=fsm_out, fsm_fault=fsm_fault)
        if shield_on and decision.touched:
            n_touched += 1
            n_braked += int(decision.braked)

        touched_now = bool(shield_on and decision.touched)
        traj.append({"t": round(now, 2), "x": state.x, "y": state.y, "up": state.up,
                     "touched": touched_now})

        # Per-tick row in the shape guardrail/kpi.py reads. NOT audit.jsonl:
        # AuditLogger writes `raw_action`/`emitted_action` and only for touched
        # ticks, so feeding it to compute() would make _emitted_differs() false
        # on every row and report every P0 tick as an escape.
        #
        # repairs and braked record what ACTUALLY happened. With the shield off
        # nothing was repaired, so they stay empty even though the monitor saw
        # the violation - which is precisely how that run earns a non-zero
        # escape rate.
        rows.append({
            "t": round(now, 3), "tick": tick,
            "x": round(state.x, 3), "y": round(state.y, 3), "up": round(state.up, 3),
            "raw": raw.model_dump(),
            "emitted": emitted.model_dump(),
            "violations": [v.model_dump() for v in decision.violations],
            # `flown` and `emitted_violations`: what was FLOWN and the check
            # on it - what guardrail/kpi.py counts a P0 escape from. Without
            # it every P0 tick scores "not measurable" and the rate falls back
            # to an inference that cannot return non-zero.
            #
            # guardrail.replay.flown_fields decides, the same on both rails:
            #   shield ON  -> `decision.emitted` flew, so its re-check applies.
            #   shield OFF -> `raw` flew unmodified, so the violations already
            #                 found on `raw` ARE the flown action's violations.
            #                 That is what earns the control run its escape rate
            #                 rather than scoring it clean.
            #   Brake      -> the zero action flew; standing still is checked.
            #   none       -> nothing was sent (a fault): nothing escaped.
            **flown_fields(shield, state, decision, shield_on=shield_on,
                           setpoint=setpoint),
            "repairs": [r.model_dump() for r in decision.repairs] if shield_on else [],
            "braked": bool(shield_on and decision.braked),
            "touched": touched_now,
            "setpoint": setpoint,
            **extra,
            **({"tgt_x": subject[0], "tgt_y": subject[1]} if subject else {}),
            **({"fsm_state_before": fsm_out.before.value,
                "fsm_state_after": fsm_out.state.value,
                "fsm_edge": fsm_out.edge, "set_mode": fsm_out.set_mode,
                "failsafe": bool(fsm_out.transition
                                 and fsm_out.state in FAILSAFE_STATES)}
               if fsm_out is not None else {}),
        })
        flown = {"pass": emitted, "brake": Action4D(), "none": None}[setpoint]
        if flown is not None:
            link.send_velocity(flown.vx, flown.vy, flown.vz_up, flown.yaw_rate)
        if fsm is not None and (fsm.terminal or (
                failsafe_t is not None and now - failsafe_t > FAILSAFE_GRACE_S)):
            end_why = "escalation FSM ended the mission"
            print(f"[fsm]      {fsm.state.value}: the autopilot has the aircraft")
            break
        time.sleep(TICK)

    # The episode is complete only with this event (guardrail.replay refuses
    # an episode without it as unfinished).
    rec.end(end_why, t=time.time() - t0)
    if fsm is None or not (fsm.terminal or fsm.state in FAILSAFE_STATES):
        link.land_disarm()      # mission end, not a fail-safe

    # ---- KPI ----
    # Every keep-out zone with vertices, incl. the dynamic_nfz the hot-applied
    # zone becomes where the Shield enforces the grant's update model.
    fences = [(f, fence_polygon(f)) for f in policy.constraints
              if getattr(f, "type", "") in ("polygon_fence", "circle_fence",
                                            "dynamic_nfz")
              and getattr(f, "vertices", None)]

    def _active(f, p) -> bool:
        if f.id == DYNAMIC_FENCE.id:
            return spawn_t is not None and p["t"] >= spawn_t
        return True

    inside = sum(1 for p in traj for f, poly in fences
                 if _active(f, p)
                 and f.altitude_floor_m <= p["up"] <= f.altitude_ceiling_m
                 and poly.contains(Point(p["x"], p["y"])))
    nfz_seconds = inside * TICK

    # Seconds outside the altitude envelope, on the same footing as NFZ time so
    # both rails report the same quantities under the same names.
    envelopes = policy.by_type(AltitudeEnvelope)
    alt_bad = sum(1 for p in traj for e in envelopes
                  if not (e.alt_min_m <= p["up"] <= e.alt_max_m))
    alt_seconds = alt_bad * TICK

    # Seconds spent inside a declared subject's stand-off ring, reported the
    # same way as NFZ time so the A/B reads identically. Horizontal range, which
    # is the rule's own definition - and the reason this demonstrates "must not
    # fly OVER a person": at 15 m altitude a straight run passes 0 m away.
    standoff_seconds = 0.0
    standoff_min_m = None
    if subject is not None:
        rings = [c.min_range_m for c in policy.by_type(SubjectStandoff)
                 if c.binds(args.subject_class)]
        ring = max(rings) if rings else 0.0
        d = [math.hypot(p["x"] - subject[0], p["y"] - subject[1]) for p in traj]
        if d:
            standoff_min_m = round(min(d), 2)
        standoff_seconds = sum(1 for v in d if v < ring) * TICK

    # metrics.json first, because compute_from_dir() reads it to decide the
    # mission outcome.
    #
    # frac_within_30m is the follow rails' proximity measure, and compute() uses
    # it as the "did the mission actually happen" signal - below 0.05 the run
    # fails regardless of rule compliance. A waypoint mission has no such
    # fraction, so `reached` is mapped onto it rather than left absent: omitting
    # it would let an aircraft that never left the pad score mission_success,
    # since sitting still breaks no rules. Same contract, one rail's answer
    # expressed in the other's units.
    metrics = {
        "tag": tag,
        "episode_id": rec.episode_id,
        "topology": TOPOLOGY_ARDUPILOT_SITL,
        "ticks": len(rows),
        "ticks_flown": sum(1 for r in rows if r.get("flown")),
        "ticks_not_flown": sum(1 for r in rows if r.get("flown") is False),
        "shield_lookahead": {"horizon_s": LOOKAHEAD_S, "dt_s": LOOKAHEAD_DT_S,
                             "poses": int(round(LOOKAHEAD_S / LOOKAHEAD_DT_S))},
        "autopilot_fence": ap_fence,
        "shield": args.shield,
        "reached": reached,
        "frac_within_30m": 1.0 if reached else 0.0,
        "nfz_s": round(nfz_seconds, 2),
        "standoff_s": round(standoff_seconds, 2),
        "standoff_min_range_m": standoff_min_m,
        "subject": ({"class": args.subject_class, "x": subject[0], "y": subject[1],
                     "position_source": "declared"} if subject else None),
        "alt_violation_s": round(alt_seconds, 2),
        "interventions": n_touched,
        "brakes": n_braked,
        "seed": args.seed,
        "shield_ms_max": max((r["shield_ms"] for r in rows
                              if isinstance(r.get("shield_ms"), (int, float))),
                             default=None),
        "fsm": (dict(fsm.summary(), fault=fsm_fault,
                     theta_basis={"source": "velocity proxy",
                                  "horizon_s": PROXY_HORIZON_S})
                if fsm is not None else None),
        "generations": rec.generations,
        # The autopilot as it named itself, beside the manifest (ARCH-31).
        "autopilot": autopilot,
        "ardupilot_version": autopilot["ardupilot_version"] if autopilot else None,
        # Beside the manifest, which is the grant's six fields and stays so.
        "policy_source": policy_source,
    }
    (out / "flight_log.jsonl").write_text(
        "".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    # The determinism manifest. sim_speedup is READ from the autopilot rather
    # than assumed: an unread speedup must surface as "unresolved", because a
    # hand-written 1.0 is exactly what a fast-forwarded run would also carry.
    manifest = build_manifest(
        policy_hash=policy.policy_hash,
        model_id="guardrail.vla_stub.StubVLA",
        seed=args.seed,
        scene_path=None,
        topology=TOPOLOGY_ARDUPILOT_SITL,
        sim_speedup=sim_speedup_from_mavlink(link.m),
    )
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    kpi = compute_from_dir(out, policy)
    # The grade belongs IN the artefact, not only in the prose report. A reader
    # holding kpi.json could not previously tell whether its numbers were
    # quotable as contractual figures - which is the single thing the grade
    # exists to say. demo/follow_vlm.py has recorded it this way all along.
    graded, why = is_kpi_grade(manifest, metrics)
    kpi["kpi_grade"] = graded
    kpi["kpi_grade_reasons"] = why
    kpi["manifest"] = manifest
    kpi["policy_source"] = policy_source
    kpi["episode_id"] = rec.episode_id
    kpi_path = out / "kpi.json"
    kpi_path.write_text(json.dumps(kpi, indent=2), encoding="utf-8")
    kpi_ok = kpi["p0_violation_escape_rate"] == 0.0

    # These are the runs whose numbers are contractually reportable, so these
    # are the ones that most need to stay re-derivable. Packaged here, while the
    # policy that governed the flight is still the object in memory.
    rb_path = out / f"{out.name}.replay.tar.gz"
    try:
        rb = write_replay(out, policy, rb_path, changelog=f"sitl {out.name}",
                          policy_source=policy_source)
        # NOT `why`: that name holds the KPI-grade reasons, which report.md
        # and the [kpi] line print below. Reusing it (as this block did until
        # 2026-10-06) printed the replay notes as the reasons a run was not
        # KPI-grade, while kpi.json said something else.
        ok, replay_notes = verify_replay(rb)
        print(f"[replay]   {rb.name} "
              f"{'re-derives its own KPIs' if ok else 'FAILED verification'}")
        for w in replay_notes:
            print(f"[replay]     - {w}")
        # KPI evidence needs a SIGNED bundle; a graded run with a keyless one
        # is demoted and kpi.json + the bundle rewritten to say so.
        graded2, why2 = kpi_evidence_grade(graded, why, rb)
        if graded2 != graded:
            graded, why = graded2, why2
            kpi["kpi_grade"], kpi["kpi_grade_reasons"] = graded, why
            kpi_path.write_text(json.dumps(kpi, indent=2), encoding="utf-8")
            write_replay(out, policy, rb_path, changelog=f"sitl {out.name}",
                         policy_source=policy_source)
    except Exception as exc:                                    # noqa: BLE001
        print(f"[replay]   not written: {type(exc).__name__}: {exc}")
        if graded:
            graded, why = False, why + [f"no replay bundle: {exc}"]
            kpi["kpi_grade"], kpi["kpi_grade_reasons"] = graded, why
            kpi_path.write_text(json.dumps(kpi, indent=2), encoding="utf-8")

    # ---- plot (if matplotlib present) ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 7))
        for f, poly in fences:
            dyn = f.id == DYNAMIC_FENCE.id
            color = "purple" if dyn else "red"
            xs, ys = poly.exterior.xy
            ax.fill(ys, xs, alpha=0.25, color=color,
                    label=("dynamic NFZ" if dyn else f"NFZ {f.id}"))
        if spawn_pos:
            ax.plot(spawn_pos[1], spawn_pos[0], "X", color="purple", markersize=12)
        ax.plot([p["y"] for p in traj], [p["x"] for p in traj], "-",
                color="tab:blue", linewidth=2, label="flight path")
        tx = [p["y"] for p in traj if p["touched"]]
        if tx:
            ax.plot(tx, [p["x"] for p in traj if p["touched"]], ".",
                    color="orange", markersize=4, label="shield active")
        ax.plot(traj[0]["y"], traj[0]["x"], "go", markersize=10, label="start")
        ax.plot(mission.target_y, mission.target_x, "k*", markersize=16, label="target")
        ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
        ax.set_title(f"SITL (real ArduPilot) — shield {'ON' if shield_on else 'OFF'}\n"
                     f"NFZ time: {nfz_seconds:.1f}s | {'reached' if reached else 'NOT reached'}")
        ax.legend(loc="upper left", fontsize=9); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(out / "trajectory.png", dpi=130)
        print(f"[plot]     {out / 'trajectory.png'}")
    except ImportError:
        print("[plot]     matplotlib missing — skipped")

    (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")
    (out / "report.md").write_text(f"""# SITL Guardrail report — shield {'ON' if shield_on else 'OFF'}

| Item | Value |
|------|-------|
| Autopilot | ArduPilot SITL (real flight code, GUIDED mode) |
| Adapter | MAVLink SET_POSITION_TARGET_LOCAL_NED @ 10 Hz |
| Command | `{args.command}` |
| Policy | `{policy.policy_id}` `{policy.policy_hash}` |
| Policy source | {policy_source['kind']} `{policy_source['path']}` - signature **{policy_source['signature']}** |
| Target reached | {'yes' if reached else 'NO'} |
| Ticks | {len(traj)} |
| Shield interventions | {n_touched} |
| Brakes | {n_braked} |
| Dynamic NFZ | {f'hot-applied t={spawn_t:.1f}s, generation {policy.generation}' if spawn_t else 'not used'} |
| Topology | `{TOPOLOGY_ARDUPILOT_SITL}` |
| Code revision | `{manifest['code_revision']}` |
| Sim speedup | `{manifest['sim_speedup']}` |
| **Time inside NFZ** | **{nfz_seconds:.1f} s** |
| Time outside altitude envelope | {alt_seconds:.1f} s |
| **P0 violation escape rate** | **{kpi['p0_violation_escape_rate']}** |
| Repairs | {kpi['repair_count']} |
| Mission outcome | {kpi['outcome']} |
| **KPI-grade** | **{'yes' if graded else 'no'}** |

{'' if graded else 'Not KPI-grade because:' + chr(10) + chr(10) + chr(10).join('- ' + r for r in why)}
""", encoding="utf-8")
    print(f"[report]   NFZ {nfz_seconds:.1f}s | alt {alt_seconds:.1f}s "
          f"| P0 escape rate {kpi['p0_violation_escape_rate']} "
          f"-> {'PASS' if kpi_ok else 'FAIL'} | touched {n_touched} | braked {n_braked}")
    print(f"[kpi]      grade={'yes' if graded else 'no'}"
          + ("" if graded else "  (" + "; ".join(why) + ")"))
    print(f"[out]      {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
