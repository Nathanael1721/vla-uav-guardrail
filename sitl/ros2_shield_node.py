"""
ROS 2 node: the Safety Shield, wired the way the grant's interface locks it.

    [vla node] --/vla/action_4d--> [THIS NODE] --/shield/setpoint--> [mavlink_adapter] --/mavros/setpoint_raw/local--> MAVROS 2 --> ArduPilot
                Float32MultiArray                Float32MultiArray                      mavros_msgs/PositionTarget
                BODY [vx_fwd, vy_right,          BODY, post-Shield                      FRAME_LOCAL_NED
                vz_up, yaw_rate] 10 Hz

THE FRAME (WP3-04, ARCH-04, WP2-17). The grant locks the VLA action as
"a = (vx, vy, vz, yaw_rate) # body-frame velocities + yaw rate" and puts the
NED conversion in the MAVLink adapter after the Shield. This node takes the
body-frame action, rotates it ONCE into the world frame the Shield's geometry
is written in (guardrail.frames.from_body, with the heading of the same pose
the adapter reads), lets the Shield check and repair it, rotates the result
back to body frame (frames.to_body) and publishes that on /shield/setpoint.
The Shield therefore repairs the body-frame action it was given and returns a
body-frame action; the rotation is exact and is undone with the same heading.
sitl/mavlink_adapter_node.py turns /shield/setpoint into NED - the single
body -> local-NED crossing - and owns every MAVROS command after take-off
(SET_MODE, fence upload). Until 2026-10-06 this node published a Twist on
/mavros/setpoint_velocity/cmd_vel_unstamped and did the ENU conversion itself
(audit cards WP3-03, WP3-09, ARCH-05).

WHAT EVERY RUN RECORDS (WP2-11, WP2-15, WP1-24, WP4-13, WP4-20, WP3-11, WP3-17)

  flight_log.jsonl  one row per tick: position, heading, the action in both
                    frames, violations, repairs, `unsafe` + `unsafe_rules`
                    (is the POSITION illegal), policy_hash + generation,
                    shield_ms (the Shield's own time), the declared subject
                    (tgt_x/tgt_y), the FSM state, the autopilot's mode, the
                    episode id, and `flown`: whether anything was sent on
                    that tick (false while the autopilot holds the aircraft,
                    e.g. a GeoFence RTL - such a tick cannot be an escape).
  policy_g<N>.json, csp_g<N>.json
                    the policy and the typed CSP of every generation, written
                    at take-off and after each hot-apply (guardrail.replay.
                    EpisodeRecord).
  audit.jsonl       this episode's Shield audit only; a stale one is moved to
                    _previous/, never appended to.
  events.jsonl      episode start, generations, hot-applies, mode requests and
                    changes, fence uploads, FSM transitions, autopilot text.
  metrics.json      incl. hil_evidence (the MAVROS chain, the autopilot's
                    own version, the host, the link) and the pilot record.
  manifest.json     the grant's six fields; topology from the evidence.
  <tag>.replay.tar.gz  written here, and re-packed by run_ros2_demo.sh once
                    the rosbag is closed so it carries bag/ as well.

TOPOLOGY. One desktop running ArduPilot SITL + MAVROS 2 + this node is the
grant's `dev`. On a Jetson Orin, with ArduPilot SITL on the desktop, the same
node gathers the Orin's evidence (device tree, L4T / JetPack, the processes
running on the Orin, the VLA node's own host from /vla/identity, MAVROS's
fcu_url) and its manifest says `hil`, whether MAVROS runs on the desktop (the
grant's table) or on the Orin (docs/RUNBOOK-orin-hil.md); with a hardware
autopilot it says `flight`. The label comes from that evidence
(guardrail.manifest.detect_topology), never from a flag, so moving the stack
from the desktop to the Orin to the drone changes no code.

GEOFENCE BACKSTOP. At take-off the node reads FENCE_ENABLE, FENCE_ALT_MAX,
AVOID_ENABLE and the rest back from the autopilot, raises FENCE_ALT_MAX above
the policy's ceiling when it sits below it, and publishes each generation's
keep-out zones to the adapter - except a zone the aircraft is in or near,
which is held back until the aircraft is clear, so a zone hot-applied on top
of the aircraft is the Shield's to resolve, not an instant autopilot RTL.

SHIELD HORIZON. 5 s at 0.1 s steps, 50 future poses, as the grant's Safety
Shield page locks it (3 s at 0.5 s until 2026-10-07).

ESCALATION (WP3-07 wiring). With the Shield on, every decision goes through
guardrail.fsm.EscalationFSM (docs/DESIGN-escalation-fsm.md, "Integration").
Its mode requests go to the adapter on /shield/mode_request. Theta is applied
to the Shield's per-operator Repair.magnitude_m (the default basis,
guardrail.fsm.DEFAULT_THETA_HORIZON_S); `--theta-horizon-s 0.1` selects the
one-period reading (the setpoint change over one 0.1 s control period, a PI
question, docs/DESIGN-escalation-fsm.md). The source is recorded in
metrics.json as `fsm.theta_basis`. Flown with magnitude_m on
2026-10-07, the default mission left Normal at the first tick (a forecast
penetration of 8.1 m against theta 2.0 m), and ArduCopter's LOITER, with no
RC input on SITL, descended to the ground (docs/DESIGN-ros2-interface.md,
"Escalation FSM wiring"); the FSM's LOITER is now flown as ArduCopter's
BRAKE (the adapter's hold mode, mavlink_adapter_node.DEFAULT_HOLD_MODE). The
FSM also gets the position's illegal depth (Shield.unsafe_depth) for X6's
progress and X7's losing-ground tests, logged as `unsafe_depth_m`, and the
caps the Shield clamped against (Shield.applied_caps). Mode requests are
recorded in the FSM's words and ArduCopter's (`mode_requests`,
`autopilot_mode_requests`; mavlink_adapter_node.ModeRequests). This node
runs the one FSM: its Shield is built without an FSM of its own
(guardrail.replay.rail_shield), because only the node sees the autopilot, and
each audit record carries the node's FSM verdict and its log row's tick
number (guardrail.replay.audit_tick).

ZONE DWELL AND COST MAP (2026-10-11). `nfz_s` is the count every rail shares
(guardrail.replay.zone_dwell, read from the Shield's rule checker): polygon
and circle fences and every dynamic_nfz that does not move, the hot-applied
zone from when it went live; a moving zone is named in
`nfz_moving_not_counted` instead of being scored where it was declared.
`--cost-map` gives each generation's CSP its cost map (EpisodeRecord).

Run (after `source /opt/ros/jazzy/setup.bash`; normally via run_ros2_demo.sh
or the launch file in sitl/ros2_ws):

    ~/venv-ros/bin/python sitl/ros2_shield_node.py --shield on [--dynamic]
    ~/venv-ros/bin/python sitl/ros2_shield_node.py --bundle bundles/fase3-sim-demo-v0.1.1.tar.gz
    ~/venv-ros/bin/python sitl/ros2_shield_node.py --policy P.yaml --dev-yaml

POLICY. `--bundle` flies the signed bundle (guardrail/bundle.py): the policy
the run used is then the issued artefact, not whatever the YAML says today.
Since 2026-10-12 a YAML policy flies only with `--dev-yaml`, recorded in the
policy source (`dev_yaml: true`, in metrics.json, kpi.json and the replay
index) and as a reason the run is not KPI-grade; with neither option the node
flies the signed bundle of policies/sim_demo_policy.yaml when it is in
bundles/ (tools/build_flight_bundles.py), and refuses otherwise
(guardrail.manifest.flight_policy_choice). The VLA node announces its own
identity and policy hash on /vla/identity; a mismatch with this node's policy
is logged and recorded.

RULE CLOCK (2026-10-12). Until this date the node gave neither its Shield nor
its CSPs a clock, so every scheduled rule was in force and every
time_window_switch inert in flight. Now one clock (guardrail.replay.
rule_clock) goes to both: `--clock-start ISO8601` scripts it (it advances
with the mission time, from the start); without it the wall clock in the
mission's time zone (`--clock-tz`, default Taiwan's +08:00), read as the
policy's naive local time. Which clock a run used is in the episode_start
event and metrics.json `rule_clock`.

HOT-APPLY ON THE FLYING SHIELD (2026-10-12). `--hotapply-port` serves the REST
hot-apply API (guardrail.api.FlightHotApply), started on the pad so a listener
that cannot start stops the run before arming: 127.0.0.1 by default, mutual
TLS with the harness CA (`--hotapply-pki`, the folder
`python -m guardrail.harness_service gen-pki` writes) for any other address.
The Shield's rules are swapped under the lock the control tick holds (the
REST server runs on its own thread beside the rclpy executor); each accepted
event writes its generation's policy and CSP, re-publishes the fences
(holding back a zone the aircraft is near) and records its receipt-to-swap
time against the grant's 50 ms target. After the episode ends it answers 503.

STALE ACTIONS (2026-10-12). Until this date a pilot whose action stream
stopped left the node flying its last action until the time cap. If no
/vla/action_4d arrives for `--action-timeout-s` (default 0.5 s: five of the
grant's 10 Hz ticks), the node holds the aircraft the way the escalation FSM
holds it: LOITER through the adapter, flown as ArduCopter BRAKE, after one
tick of the Shield's own answer to a stop; the FSM does not step while held,
and the pilot's next action returns the aircraft to GUIDED. Every spell is in
events.jsonl and metrics.json `action_watchdog`; rows carry `action_stale`.

TICK LATENCY (2026-10-12). Each row records `e2e_ms` (an action's receipt to
the publication of the first setpoint flown from it), `action_age_ms` (the
age of the action a published setpoint came from) and `tick_ms` (the tick
callback's own time to publication); metrics.json `tick_latency` gives p50 /
p99 / max against the grant's 100 ms monitor tick budget. The node ticks on
its own 10 Hz timer, so e2e includes the wait for the next tick: up to one
tick period on top of the processing, by construction.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import Action4D, AuditLogger, Shield, State             # noqa: E402
from guardrail.bundle import load_for_flight                              # noqa: E402
from guardrail.compiler import ConstraintCompiler                         # noqa: E402
from guardrail.frames import from_body, to_body                           # noqa: E402
from guardrail.fsm import (DEFAULT_THETA_HORIZON_S, FAILSAFE_STATES,     # noqa: E402
                           MODE_FOR, THETA_HORIZON_S, EscalationFSM, FSMConfig,
                           tick_input_from_decision)
from guardrail.geometry import fence_polygon                              # noqa: E402
from guardrail.kpi import compute_from_dir                                # noqa: E402
from guardrail.manifest import (DEV_YAML_FLAG, TOPOLOGY_ARDUPILOT_SITL,   # noqa: E402
                                UNRESOLVED, build_manifest,
                                collect_host_evidence, decode_custom_version,
                                describe_autopilot_version, detect_topology,
                                flight_bundle_path, flight_policy_choice,
                                is_kpi_grade, mavros_location,
                                network_link_evidence, pilot_record,
                                scan_local_processes)
from guardrail.models import (AltitudeEnvelope, PolygonFence,             # noqa: E402
                              SubjectStandoff, XY)
from guardrail.replay import (EpisodeRecord, audit_tick,                  # noqa: E402
                              episode_row_fields, flown_fields,
                              kpi_evidence_grade, rail_shield, rule_clock,
                              verify_replay, write_replay, zone_dwell)
from mavlink_adapter_node import (DEFAULT_HOLD_MODE, FENCE_MARGIN_M,      # noqa: E402
                                  HOLD_MODES, TOPIC_FENCES, TOPIC_MODE_REQUEST,
                                  TOPIC_SETPOINT, TOPIC_STATUS, ModeRequests,
                                  fence_alt_plan,
                                  heading_deg_from_enu_quaternion,
                                  policy_ceiling_m, policy_fences, read_parm,
                                  zones_near)

TICK = 0.1
MAX_S = 120
REACH_M = 2.0
DYNAMIC_AT_S = 8.0
DYNAMIC_FENCE = PolygonFence(
    id="nfz-dynamic", type="polygon_fence",
    vertices=[XY(x=23, y=6), XY(x=31, y=6), XY(x=31, y=14), XY(x=23, y=14)],
    margin_m=1.0,
)
# `--dynamic-zone on-aircraft`: the grant's spawn_polygon_fence perturbation at
# its hardest, a zone hot-applied ON TOP of the aircraft, so the run starts
# unsafe and time to safe is measured with the Shield on (WP3-17).
ON_AIRCRAFT_HALF_M = 5.0


def keepout_zones(policy) -> list:
    """Every keep-out zone with vertices: polygon and circle fences, and the
    dynamic_nfz a mid-flight zone becomes where the Shield enforces the
    grant's update model. (A moving dynamic_nfz is drawn where it was
    spawned; zone dwell is guardrail.replay.zone_dwell, which names a moving
    zone instead of scoring it.)"""
    return [c for c in getattr(policy, "constraints", [])
            if getattr(c, "type", "") in ("polygon_fence", "circle_fence",
                                          "dynamic_nfz")
            and getattr(c, "vertices", None)]


def zone_around(x: float, y: float, half_m: float = ON_AIRCRAFT_HALF_M,
                zone_id: str = "nfz-on-aircraft") -> PolygonFence:
    """A square keep-out zone centred on (x North, y East)."""
    return PolygonFence(
        id=zone_id, type="polygon_fence", margin_m=1.0,
        vertices=[XY(x=x - half_m, y=y - half_m), XY(x=x + half_m, y=y - half_m),
                  XY(x=x + half_m, y=y + half_m), XY(x=x - half_m, y=y + half_m)])
FENCE_PARM = ROOT / "sitl" / "fence" / "guardrail_fence.parm"
# The grant's Safety Shield horizon: "5 s of predicted trajectory at 10 Hz =
# 50 future poses". Until 2026-10-07 both SITL rails ran 3 s at 0.5 s steps
# (6 poses).
LOOKAHEAD_S = 5.0
LOOKAHEAD_DT_S = 0.1
# The autopilot parameters that make the GeoFence a backstop, read back at
# take-off and recorded (hil_evidence.autopilot_fence).
FENCE_PARAMS = ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX",
                "FENCE_MARGIN", "AVOID_ENABLE")
# The one-period reading of theta (guardrail.fsm.THETA_HORIZON_S, 0.1 s):
# selectable with --theta-horizon-s 0.1, not the default (the per-operator
# magnitude_m is). The name is kept for the tests and tools that read it; the
# basis a run used is written into metrics.json.
PROXY_HORIZON_S = THETA_HORIZON_S
# After the autopilot takes the aircraft away from the Shield (a GeoFence
# breach RTL, a pilot on the GCS), how long to keep recording before ending
# the episode if it has not landed.
TAKEOVER_GRACE_S = 25.0
# How long after the FSM or the autopilot entered RTL / Land to wait for the
# landing before the episode is ended anyway.
FAILSAFE_GRACE_S = 40.0
# The stale-action watchdog's default: this many missed ticks of the grant's
# 10 Hz action rate (Safety Shield page: "Monitor rate: 10 Hz, matching the
# VLA action rate"). A pilot slower than 10 Hz (OpenVLA-7B makes about one
# decision per second) needs a longer --action-timeout-s, which is recorded.
ACTION_TIMEOUT_TICKS = 5
ACTION_TIMEOUT_S = ACTION_TIMEOUT_TICKS * TICK
# The grant's monitor tick budget, end to end (Safety Shield page,
# "Performance targets": <= 100 ms (10 Hz) end-to-end).
TICK_BUDGET_MS = 100.0
# The policy flown when neither --bundle nor --policy is given: its signed
# bundle when bundles/ holds it, this YAML only under --dev-yaml.
DEFAULT_POLICY_YAML = ROOT / "policies" / "sim_demo_policy.yaml"
DEFAULT_HOTAPPLY_HOST = "127.0.0.1"


# --------------------------------------------------------------------------- #
# Pure helpers (tests/test_sitl_rails.py imports these without ROS)
# --------------------------------------------------------------------------- #

def parse_xy(text: str | None) -> tuple[float, float] | None:
    if not text:
        return None
    parts = [float(v) for v in text.split(",")]
    if len(parts) != 2 or not all(math.isfinite(v) for v in parts):
        raise ValueError(f"expected X,Y, got {text!r}")
    return parts[0], parts[1]


def default_tag(shield: str, dynamic: bool) -> str:
    """Kept identical to run_ros2_demo.sh's rule, which records the bag there."""
    return f"ros2_shield_{shield}" + ("_dynamic" if dynamic else "")


def shield_step(shield: Shield, state: State, body: Action4D, shield_on: bool):
    """One tick through the Shield in the grant's body frame.

    Returns `(decision, raw_world, emitted_world, emitted_body)`. The Shield
    always evaluates (the control arm needs its violations); with it off the
    body action passes through untouched, not rotated there and back.
    """
    raw_world = from_body(body.vx, body.vy, body.vz_up, body.yaw_rate,
                          state.yaw_deg)
    decision = shield.filter(state, raw_world)
    if not shield_on:
        return decision, decision.raw, decision.raw, body
    em = decision.emitted
    vx, vy, vz, wz = to_body(em, state.yaw_deg)
    return decision, decision.raw, em, Action4D(vx=vx, vy=vy, vz_up=vz,
                                                yaw_rate=wz)


def build_row(*, now: float, tick: int, state: State, body: Action4D,
              decision, raw_world: Action4D, emitted_world: Action4D,
              emitted_body: Action4D | None, shield_on: bool, extra: dict,
              subject: tuple | None, fsm_out=None, stop_illegal=None,
              mode: str | None = None, setpoint: str = "pass",
              shield: Shield | None = None,
              unsafe_depth: float | None = None) -> dict:
    """The per-tick row, in the shape guardrail/kpi.py reads.

    raw / emitted stay in the world frame the Shield's checks use (kpi.py's
    repair magnitudes are frame-invariant anyway); raw_body / emitted_body are
    what the grant's interface carried. `flown` and `emitted_violations` say
    what was FLOWN and the check on it (guardrail.replay.flown_fields): the
    re-check with the Shield on, the violations of `raw` with it off - which
    is what earns the control arm its escapes - the zero action's on a Brake
    tick, and nothing (`flown: false`) on a tick where the rail sent nothing
    because the autopilot or a fault held the aircraft. `shield` is needed
    for a Brake tick, whose check is the Shield's own.
    """
    if setpoint == "brake" and shield is None:
        raise ValueError("a Brake row needs the Shield to check the zero action")
    flown = flown_fields(shield, state, decision, shield_on=shield_on,
                         setpoint=setpoint)
    row = {
        "t": round(now, 3), "tick": tick,
        "x": round(state.x, 3), "y": round(state.y, 3), "up": round(state.up, 3),
        "yaw_deg": round(state.yaw_deg, 2),
        "frame": "body",
        "raw_body": body.model_dump(),
        "emitted_body": None if emitted_body is None else emitted_body.model_dump(),
        "raw": raw_world.model_dump(),
        "emitted": emitted_world.model_dump(),
        "violations": [v.model_dump() for v in decision.violations],
        **flown,
        "repairs": ([r.model_dump() for r in decision.repairs]
                    if shield_on else []),
        "braked": bool(shield_on and decision.braked),
        "touched": bool(shield_on and decision.touched),
        "setpoint": setpoint,
        "mode": mode,
        **extra,
    }
    if subject is not None:
        row["tgt_x"], row["tgt_y"] = subject[0], subject[1]
    if fsm_out is not None:
        row.update({"fsm_state_before": fsm_out.before.value,
                    "fsm_state_after": fsm_out.state.value,
                    "fsm_edge": fsm_out.edge, "set_mode": fsm_out.set_mode,
                    "stop_illegal": bool(stop_illegal),
                    "unsafe_depth_m": (None if unsafe_depth is None
                                       else round(unsafe_depth, 4)),
                    "failsafe": bool(fsm_out.transition
                                     and fsm_out.state in FAILSAFE_STATES)})
    elif shield_on is False:
        row["fsm_state_after"] = None
    return row


def theta_basis(horizon_s: float | None) -> dict:
    """How theta's repair magnitude is measured on this run."""
    if horizon_s is None:
        return {"source": "per-operator magnitude_m", "horizon_s": None}
    one = abs(horizon_s - THETA_HORIZON_S) < 1e-9
    return {"source": ("setpoint change |dv| over one control period, the one-period "
                       "reading (guardrail.fsm.THETA_HORIZON_S; selectable, pending "
                       "the PI)" if one else
                       "velocity proxy |dv| * h (guardrail.fsm.magnitude_from_actions)"),
            "horizon_s": horizon_s,
            "note": ("under a 4-5 m/s / 2 m/s envelope a clamped command changes by "
                     "at most 1.0 / 0.4 m per period, so the theta cap cannot fire "
                     "on these flights (docs/DESIGN-escalation-fsm.md); 'no operator "
                     "converged', N-in-T, X6, X7 and direct RTL / Land / Loiter "
                     "actions are live")}


def default_theta_horizon() -> float | None:
    """The basis a run uses unless --theta-horizon-s says otherwise:
    guardrail.fsm.DEFAULT_THETA_HORIZON_S, None - each repair's own
    magnitude_m, which shield.py reports since 2026-10-07. (On 2026-10-08 a
    builder made the one-period reading the default and called it the
    grant's; reverted the next day: the grant's p5 audit example does not
    support it, and under the shipped caps it can never exceed theta.)"""
    return DEFAULT_THETA_HORIZON_S


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    k = min(len(s) - 1, max(0, int(math.ceil(q / 100.0 * len(s))) - 1))
    return round(s[k], 3)


def check_expected_hash(policy, expected: str | None) -> dict:
    """The `--expected-hash` pin: `{}` when none was given, else the record
    for the policy source. Raises ValueError, before take-off, when the
    policy that loaded is not the one the caller pinned - the stress
    harness passes the hash it expects with every bundle it sends
    (experiments/sweep_scenarios.py build_sitl_cmd), and until 2026-10-13
    this node did not declare the flag, so parse_known_args dropped it and a
    different bundle would have flown unchallenged."""
    if not expected:
        return {}
    got = policy.policy_hash
    want = expected if expected.startswith("sha256:") else f"sha256:{expected}"
    if got != want:
        raise ValueError(f"--expected-hash {want} but the policy that loaded is {got}: "
                         f"refusing to fly a policy other than the one pinned")
    return {"expected_hash": want, "expected_hash_ok": True}


def resolve_policy(args) -> tuple[str | None, str | None, dict]:
    """`(bundle, yaml, record)` for load_for_flight: --bundle, or --policy
    with --dev-yaml, or the default policy's signed bundle
    (guardrail.manifest.flight_policy_choice). Raises ValueError, before
    anything flies, for a YAML policy without --dev-yaml."""
    return flight_policy_choice(
        args.bundle, args.policy, dev_yaml=getattr(args, "dev_yaml", False),
        default_yaml=DEFAULT_POLICY_YAML,
        default_bundle=flight_bundle_path(DEFAULT_POLICY_YAML))


class ActionWatchdog:
    """Is the pilot's action stream alive?

    `received(t)` on every valid action; `check(t)` once per tick returns
    "stale" on the tick the stream is declared quiet (no action for longer
    than `timeout_s`, counted from the last action, or from `start(t)` when
    none has arrived since), "resumed" on the first tick after an action
    arrives during a stale spell, and None otherwise. `t` is seconds on one
    monotonic clock. Each spell is kept in `spells`."""

    def __init__(self, timeout_s: float = ACTION_TIMEOUT_S) -> None:
        timeout_s = float(timeout_s)
        if not (timeout_s > 0.0 and math.isfinite(timeout_s)):
            raise ValueError(f"the action timeout must be a positive number of "
                             f"seconds, got {timeout_s!r}")
        self.timeout_s = timeout_s
        self.t0: float | None = None
        self.last_rx: float | None = None
        self.stale = False
        self._rx_in_spell = False
        self.spells: list[dict] = []

    def start(self, t: float) -> None:
        self.t0 = float(t)

    def received(self, t: float) -> None:
        self.last_rx = float(t)
        if self.stale:
            self._rx_in_spell = True

    def age(self, t: float) -> float | None:
        ref = self.last_rx if self.last_rx is not None else self.t0
        return None if ref is None else float(t) - ref

    def check(self, t: float) -> str | None:
        if self.stale:
            if self._rx_in_spell:
                self.stale = self._rx_in_spell = False
                self.spells[-1]["t_end"] = float(t)
                return "resumed"
            return None
        age = self.age(t)
        if age is not None and age > self.timeout_s:
            self.stale = True
            self.spells.append({"t_start": float(t), "age_s": round(age, 3),
                                "t_end": None})
            return "stale"
        return None

    def summary(self, to_mission_t=None) -> dict:
        conv = to_mission_t or (lambda v: v)
        spells = [{"t_start": round(conv(s["t_start"]), 3),
                   "t_end": (None if s["t_end"] is None
                             else round(conv(s["t_end"]), 3)),
                   "quiet_s_at_onset": s["age_s"]} for s in self.spells]
        return {"timeout_s": self.timeout_s, "spells": spells,
                "n_spells": len(spells),
                "stale_at_end": self.stale}


def latency_fields(tick_ns: int, pub_ns: int | None, rx_ns: int | None,
                   fresh: bool) -> dict:
    """One row's latency, in ms on time.monotonic_ns(): `tick_ms` (the tick
    callback's entry to the setpoint's publication), `action_age_ms` (the
    receipt of the action the setpoint came from to its publication) and
    `e2e_ms` (the same, on the first tick that flies a newly received
    action; None on a tick that re-flies one, publishes nothing, or holds)."""
    if pub_ns is None:
        return {"tick_ms": None, "action_age_ms": None, "e2e_ms": None}
    age = None if rx_ns is None else round((pub_ns - rx_ns) / 1e6, 3)
    return {"tick_ms": round((pub_ns - tick_ns) / 1e6, 3),
            "action_age_ms": age, "e2e_ms": age if fresh else None}


def latency_summary(rows: list[dict], budget_ms: float = TICK_BUDGET_MS) -> dict:
    """p50 / p99 / max of the rows' e2e_ms, action_age_ms and tick_ms, and the
    ticks over the grant's budget. The null is a node doing nothing per tick:
    with a pilot on its own 10 Hz clock, e2e then sits anywhere in 0-100 ms,
    the phase between the two clocks."""
    out: dict = {"budget_ms": budget_ms}
    for key in ("e2e_ms", "action_age_ms", "tick_ms"):
        vals = [r[key] for r in rows if isinstance(r.get(key), (int, float))]
        out[key] = {"n": len(vals), "p50": percentile(vals, 50),
                    "p99": percentile(vals, 99),
                    "max": round(max(vals), 3) if vals else None,
                    "over_budget": sum(1 for v in vals if v > budget_ms)}
    out["note"] = ("e2e_ms: an action's receipt to the first setpoint flown "
                   "from it; the node ticks on its own 10 Hz timer, so up to "
                   "one tick period of it is the wait for the next tick")
    return out


# --------------------------------------------------------------------------- #
# The node (needs rclpy + mavros_msgs)
# --------------------------------------------------------------------------- #

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float32MultiArray, String
    from mavros_msgs.msg import ExtendedState, State as MavState, StatusText
    from mavros_msgs.srv import (CommandBool, CommandTOL, SetMode, StreamRate,
                                 VehicleInfoGet)
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

if HAVE_ROS:
    LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    LANDED_STATE_ON_GROUND = 1

    class ShieldNode(Node):
        def __init__(self, args, out: Path, subject=None) -> None:
            super().__init__("safety_shield")
            self.args = args
            self.shield_on = args.shield == "on"
            self.dynamic = args.dynamic
            self.out = out
            self.max_s = float(args.max_s)

            # The signed bundle (given, or the default policy's); a YAML only
            # under --dev-yaml, recorded so. Refuses before take-off, never
            # mid-air.
            bundle, yaml_path, choice = resolve_policy(args)
            self.policy, self.policy_source = load_for_flight(
                bundle, yaml_path,
                allow_unverified=args.allow_unverified_bundle)
            self.policy_source.update(choice)
            self.policy_source.update(check_expected_hash(
                self.policy, getattr(args, "expected_hash", None)))
            self.get_logger().info(
                f"policy {self.policy.policy_id} v{self.policy.version} "
                f"{self.policy.policy_hash} from {self.policy_source['kind']} "
                f"{self.policy_source['path']} - signature "
                f"{self.policy_source['signature']}")
            compiler = ConstraintCompiler(self.policy)
            # The default mission is written out literally: the stress
            # harness (experiments/sweep_scenarios.py SITL_RAIL_COMMAND) and
            # its test compare scenarios against exactly this text.
            self.mission = (compiler.parse_command(args.command) if args.command
                            else compiler.parse_command("fly to the northeast pad at 6 m/s"))
            tgt = parse_xy(args.target)
            if tgt is not None:
                self.mission = self.mission.model_copy(
                    update={"target_x": tgt[0], "target_y": tgt[1]})
            if args.speed is not None:
                self.mission = self.mission.model_copy(
                    update={"speed_pref_mps": float(args.speed)})

            # The rule clock, one callable for the Shield and every
            # generation's CSP (guardrail.replay.rule_clock): scripted from
            # --clock-start with the mission time, else the wall clock in the
            # mission's time zone. Mission time is None until mission start.
            self.mission_t0: float | None = None
            self.clock, self.clock_record = rule_clock(
                getattr(args, "clock_start", None), elapsed=self._t,
                tz=getattr(args, "clock_tz", None))
            # Episode record FIRST: it moves an earlier flight's audit and
            # generation files aside, so the logger below starts a fresh file.
            self.rec = EpisodeRecord(out, self.policy, self.mission,
                                     lookahead_s=LOOKAHEAD_S,
                                     cost_map=getattr(args, "cost_map", False),
                                     clock=self.clock,
                                     clock_record=self.clock_record)
            (out / "prompt.yaml").write_text(compiler.build_prompt(self.mission),
                                             encoding="utf-8")
            # No escalation FSM inside the Shield: this node runs THE FSM,
            # because only it sees the autopilot (guardrail.replay.rail_shield).
            self.shield = rail_shield(self.policy, lookahead_s=LOOKAHEAD_S,
                                      dt=LOOKAHEAD_DT_S, clock=self.clock)
            # Held by every control tick and by the REST hot-apply thread
            # while it swaps the Shield's rules and records the generation.
            self._lock = threading.RLock()
            self.hot = None             # guardrail.api.FlightHotApply, if served
            self.audit = AuditLogger(out / "audit.jsonl", self.policy,
                                     episode_id=str(self.rec.episode_id))

            # Escalation FSM, Shield-on arm only (the control arm records
            # fsm_state_after: null).
            self.theta_horizon_s = (default_theta_horizon()
                                    if args.theta_horizon_s == "auto"
                                    else (None if args.theta_horizon_s == "none"
                                          else float(args.theta_horizon_s)))
            cfg = FSMConfig.from_yaml(args.fsm_config) if args.fsm_config else FSMConfig()
            self.fsm = EscalationFSM(cfg) if self.shield_on else None
            self.fsm_fault: str | None = None
            self.hold_mode = getattr(args, "hold_mode", DEFAULT_HOLD_MODE)
            # What was asked of the autopilot, in the FSM's words and
            # ArduCopter's, and which request explains a mode change.
            self._asks = ModeRequests(self.hold_mode)

            # A DECLARED subject, so subject_standoff rules bind on a rail with
            # no camera. Ground truth, not perception.
            self.subject = subject
            self.subject_class = args.subject_class
            if subject is not None:
                self.shield.set_subject(subject[0], subject[1], self.subject_class)
                rings = [(c.id, c.min_range_m)
                         for c in self.policy.by_type(SubjectStandoff)
                         if c.binds(self.subject_class)]
                self.get_logger().info(
                    f"{self.subject_class} DECLARED at ({subject[0]:.1f}, "
                    f"{subject[1]:.1f}); "
                    + (", ".join(f"{i} {r:.0f}m" for i, r in rings) if rings
                       else "WARNING no standoff rule binds - the ring is inert"))

            self.state: State | None = None
            self.raw_body = Action4D()
            self.n_actions = 0
            # The action stream: its watchdog, and the receipt of the latest
            # action (sequence number and monotonic ns) for the tick latency.
            self.watchdog = ActionWatchdog(getattr(args, "action_timeout_s",
                                                   ACTION_TIMEOUT_S))
            self._stale_hold = False
            self._action_seq = 0
            self._action_rx_ns: int | None = None
            self._flown_seq = 0
            self.mav_connected = False
            self.mode: str | None = None
            self.landed: bool | None = None
            self.traj: list[dict] = []
            self.rows: list[dict] = []
            # Evidence of the topology, gathered from the live system rather
            # than asserted (guardrail.manifest.check_topology_evidence).
            self.hil_evidence: dict = {
                "ros_distro": os.environ.get("ROS_DISTRO"),
                "mavros_node": None,
                "fcu_connected": None,
                "ardupilot_version": None,
                "ardupilot_version_source": None,
                # The VLA node's own host, from /vla/identity; None until (and
                # unless) it reports one. The grant puts it on the Orin.
                "vla_host": None,
                **collect_host_evidence(),
            }
            self.vla_identity: dict | None = None
            self.fence_status: list[dict] = []
            # The autopilot's fence parameters as read back at take-off, and
            # the zones held back because the aircraft was in or near them.
            self.autopilot_fence: dict = {}
            self.fence_column_m: float | None = None
            self.fence_margin_m: float = FENCE_MARGIN_M
            self.deferred_fences: list[str] = []
            self.zones_over_home: list[str] = []
            # The zones the autopilot holds (the adapter confirmed the
            # upload): never held back again, since each upload replaces the
            # list (guardrail.fence_export.defer_near). And the last list
            # sent, which the adapter's confirmation is matched against.
            self.fences_enforced: dict = {}
            self._fences_sent: dict | None = None
            self.sim_speedup: float | None = None
            self.n_touched = self.n_braked = 0
            self.spawn_t = self.spawn_pos = None
            self.dynamic_fence_id: str | None = None
            self.done = False
            self.reached = False
            self.takeover: dict | None = None
            self.failsafe_t: float | None = None
            self.n_pose = 0
            self._last_rate_req = 0.0

            self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                     self._on_pose, qos_profile_sensor_data)
            self.create_subscription(MavState, "/mavros/state", self._on_state, 10)
            self.create_subscription(ExtendedState, "/mavros/extended_state",
                                     self._on_ext, qos_profile_sensor_data)
            self.create_subscription(StatusText, "/mavros/statustext/recv",
                                     self._on_text, qos_profile_sensor_data)
            self.create_subscription(Float32MultiArray, "/vla/action_4d",
                                     self._on_action, 10)
            self.create_subscription(String, "/vla/identity", self._on_identity,
                                     LATCHED)
            self.create_subscription(String, TOPIC_STATUS, self._on_adapter,
                                     LATCHED)
            self.pub = self.create_publisher(Float32MultiArray, TOPIC_SETPOINT, 10)
            self.pub_mode = self.create_publisher(String, TOPIC_MODE_REQUEST, 10)
            self.pub_fences = self.create_publisher(String, TOPIC_FENCES, LATCHED)
            self.pub_intercept = self.create_publisher(String, "/shield/intercept", 10)

            self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
            self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
            self.cli_tol = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
            self.cli_rate = self.create_client(StreamRate, "/mavros/set_stream_rate")
            self.cli_param = self.create_client(GetParameters,
                                                "/mavros/param/get_parameters")
            self.cli_mavros_param = self.create_client(GetParameters,
                                                       "/mavros/get_parameters")
            self.cli_info = self.create_client(VehicleInfoGet,
                                               "/mavros/vehicle_info_get")
            self.cli_param_set = self.create_client(
                SetParameters, "/mavros/param/set_parameters")
            # The fences are published in bring_up(), once the autopilot's
            # fence parameters and the aircraft's position are known.

        # ---------------- subscriptions ---------------- #

        def _t(self) -> float | None:
            return None if self.mission_t0 is None else time.monotonic() - self.mission_t0

        def _on_pose(self, msg: PoseStamped) -> None:
            q = msg.pose.orientation
            # MAVROS ENU -> our up-positive frame (x = N, y = E), heading
            # clockwise from North. The adapter reads the same topic.
            self.state = State(x=msg.pose.position.y, y=msg.pose.position.x,
                               up=msg.pose.position.z,
                               yaw_deg=heading_deg_from_enu_quaternion(
                                   q.x, q.y, q.z, q.w))
            if self.mission_t0 is not None:
                self.n_pose += 1

        def _on_state(self, msg: MavState) -> None:
            self.mav_connected = msg.connected
            self.hil_evidence["mavros_node"] = "/mavros"
            self.hil_evidence["fcu_connected"] = bool(msg.connected)
            if msg.mode != self.mode:
                self.rec.event("mode", t=self._t(), mode=msg.mode,
                               previous=self.mode, armed=bool(msg.armed))
                if msg.mode == "GUIDED":
                    self._asks.entered_guided()
                if (self.mission_t0 is not None and not self.done
                        and self.mode == "GUIDED" and msg.mode != "GUIDED"
                        and self.takeover is None
                        and not self._fsm_asked(msg.mode)):
                    # The autopilot left GUIDED without the Shield asking: a
                    # GeoFence breach, a failsafe, or a pilot on the GCS.
                    self.takeover = {"t": self._t(), "mode": msg.mode}
                    self.get_logger().warn(
                        f"autopilot took the aircraft: GUIDED -> {msg.mode}")
            self.mode = msg.mode

        def _fsm_asked(self, mode: str) -> bool:
            """Did this node ask for the mode the autopilot just entered, since
            the aircraft last came back to GUIDED? A LOITER request sent through
            the adapter is flown as the adapter's hold mode (BRAKE by default,
            LOITER with its --hold-mode LOITER), so either counts for it
            (ModeRequests.explains)."""
            return self._asks.explains(mode)

        def _on_ext(self, msg: ExtendedState) -> None:
            self.landed = msg.landed_state == LANDED_STATE_ON_GROUND

        def _on_text(self, msg: StatusText) -> None:
            self.rec.event("autopilot_text", t=self._t(), severity=int(msg.severity),
                           text=msg.text)

        def _on_action(self, msg: Float32MultiArray) -> None:
            d = list(msg.data)
            if len(d) >= 4 and all(math.isfinite(v) for v in d[:4]):
                self.raw_body = Action4D(vx=d[0], vy=d[1], vz_up=d[2], yaw_rate=d[3])
                self.n_actions += 1
                self._action_rx_ns = time.monotonic_ns()
                self._action_seq += 1
                self.watchdog.received(self._action_rx_ns / 1e9)

        def _on_identity(self, msg: String) -> None:
            try:
                self.vla_identity = json.loads(msg.data)
            except ValueError:
                return
            self.rec.event("vla_identity", identity=self.vla_identity)
            host = self.vla_identity.get("host")
            # None stays None: a VLA node that does not say where it runs
            # cannot be shown to be on the Orin (manifest._vla_host_missing).
            self.hil_evidence["vla_host"] = host if isinstance(host, dict) else None
            vh = self.vla_identity.get("policy_hash")
            with self._lock:            # not mid-way through a REST swap
                same = (not vh) or self.policy.matches_hash(vh)
            if not same:
                # In the ros2_ped_* runs the two nodes read different files and
                # nothing showed it (audit card WP1-23).
                self.get_logger().warn(
                    f"the VLA node flies policy {vh}, this node "
                    f"{self.policy.policy_hash}")
                self.rec.event("policy_mismatch", vla=vh,
                               shield=self.policy.policy_hash)

        def _on_adapter(self, msg: String) -> None:
            try:
                rec = json.loads(msg.data)
            except ValueError:
                return
            if rec.get("kind") == "fence_upload":
                self.fence_status.append(rec)
                with self._lock:
                    sent = self._fences_sent
                    if (rec.get("success") is True and sent is not None
                            and rec.get("generation") == sent["generation"]
                            and sorted(rec.get("fence_ids") or []) == sent["ids"]):
                        self.fences_enforced = {f["id"]: f for f in sent["fences"]}
            self.rec.event(f"adapter_{rec.get('kind', 'status')}", t=self._t(),
                           **{k: v for k, v in rec.items() if k not in ("kind", "t")})

        # ---------------- outputs ---------------- #

        def _publish_fences(self, reason: str) -> None:
            """This generation's keep-out zones to the adapter, under the lock
            the REST hot-apply thread also publishes them under."""
            with self._lock:
                self._publish_fences_locked(reason)

        def _publish_fences_locked(self, reason: str) -> None:
            """This generation's keep-out zones to the adapter, minus any the
            aircraft is in or near (`deferred_fences`, re-sent from _tick once
            clear). The column is the autopilot's own FENCE_ALT_MAX as read
            back at take-off; unknown, nothing is exported."""
            st = self.state
            # No pose yet means the aircraft is still on the pad, at the
            # local origin. "Unknown, so upload everything" armed a zone that
            # covers home under the aircraft on a 2026-10-07 flight: the
            # GeoFence held it on the pad and take-off never completed.
            if st is not None:
                pos, pos_src = (st.x, st.y), "pose"
            else:
                pos, pos_src = (0.0, 0.0), "assumed home: no pose yet (on the pad)"
            fences, skipped = policy_fences(
                self.policy, column_m=self.fence_column_m, position=pos,
                margin_m=self.fence_margin_m, enforced=self.fences_enforced)
            self.deferred_fences = sorted(s["id"] for s in skipped
                                          if s.get("deferred"))
            # Zones over home (the local origin in SITL): an RTL flies back
            # into them. Recorded and warned, not withheld.
            every, _ = policy_fences(self.policy, column_m=self.fence_column_m)
            over_home = zones_near(every, 0.0, 0.0, self.fence_margin_m)
            if over_home and over_home != self.zones_over_home:
                self.get_logger().warn(
                    f"zone(s) {', '.join(over_home)} cover home: an RTL lands "
                    f"in them and the GeoFence will report a breach there")
            self.zones_over_home = over_home
            # A geographic policy's frame origin (lat, lon), or None for local
            # metres about home; the adapter refuses one far from home.
            origin = getattr(self.policy, "frame_origin", None)
            doc = {"generation": self.policy.generation,
                   "policy_hash": self.policy.policy_hash,
                   "fences": fences, "skipped": skipped, "reason": reason,
                   "fence_alt_max_m": self.fence_column_m,
                   "position": [round(pos[0], 2), round(pos[1], 2)],
                   "position_source": pos_src,
                   "origin": (None if origin is None else
                              {"lat": origin[0], "lon": origin[1]}),
                   "backstop_active": self.autopilot_fence.get("backstop_active")}
            self._fences_sent = {"generation": doc["generation"],
                                 "ids": sorted(f["id"] for f in fences),
                                 "fences": fences}
            self.pub_fences.publish(String(data=json.dumps(doc)))
            self.rec.event("fences_published", t=self._t(), reason=reason,
                           generation=doc["generation"],
                           fence_ids=[f["id"] for f in fences], skipped=skipped,
                           deferred=self.deferred_fences,
                           zones_over_home=self.zones_over_home)

        def _recheck_deferred(self, st: State, unsafe_rules: list[str]) -> None:
            """Upload a held-back zone once the aircraft is clear of it: out
            of the zone and its margin, and the Shield no longer finds the
            position unsafe under that rule."""
            if not self.deferred_fences:
                return
            _, skipped = policy_fences(self.policy, column_m=self.fence_column_m,
                                       position=(st.x, st.y),
                                       margin_m=self.fence_margin_m)
            still = {s["id"] for s in skipped if s.get("deferred")}
            cleared = [i for i in self.deferred_fences
                       if i not in still and i not in unsafe_rules]
            if cleared:
                self._publish_fences(f"deferred zone(s) {', '.join(cleared)} "
                                     f"now clear")

        def _request_mode(self, mode: str, why: str) -> None:
            """Through the adapter (WP3-09); straight to MAVROS only when no
            adapter is listening, and the record says which."""
            via = "adapter"
            to_adapter = self.count_subscribers(TOPIC_MODE_REQUEST) > 0
            ap = self._asks.ask(mode, via_adapter=to_adapter)
            if to_adapter:
                # The FSM's vocabulary: the adapter maps LOITER to its hold mode.
                self.pub_mode.publish(String(data=mode))
            elif self.cli_mode.service_is_ready():
                self.cli_mode.call_async(SetMode.Request(custom_mode=ap))
                via = f"mavros (no adapter subscribed) as {ap}"
            else:
                via = "nobody: set_mode not ready"
            self.rec.event("mode_request", t=self._t(), mode=mode, autopilot_mode=ap,
                           why=why, via=via)
            self.get_logger().info(f"{mode} requested ({why}) via {via}")

        def _publish_body(self, a: Action4D) -> None:
            self.pub.publish(Float32MultiArray(
                data=[float(a.vx), float(a.vy), float(a.vz_up), float(a.yaw_rate)]))

        # ---------------- helpers ---------------- #

        def _call(self, client, req, timeout: float = 5.0):
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            return fut.result()

        def _get_params(self, client, names: list[str]) -> dict:
            try:
                res = self._call(client, GetParameters.Request(names=names), 5.0)
            except Exception:                                 # noqa: BLE001
                return {}
            out = {}
            if res is not None:
                for n, v in zip(names, res.values):
                    # type 0 = PARAMETER_NOT_SET. MAVROS sometimes reports
                    # SIM_SPEEDUP as INTEGER with the value in double_value,
                    # so take whichever field carries one.
                    if v.type == 0:
                        out[n] = None
                    elif v.type == 4:
                        out[n] = v.string_value
                    else:
                        out[n] = float(v.double_value) or float(v.integer_value)
            return out

        def read_sim_speedup(self) -> float | None:
            """SIM_SPEEDUP from the autopilot, or None - and whether the
            autopilot is SITL at all.

            MAVROS 2 surfaces autopilot parameters as node parameters of
            /mavros/param (rcl_interfaces/GetParameters); it populates them
            itself on FCU connect, and a forced pull raced the refill and
            returned 0.0, so this polls and accepts the first positive answer.
            A hardware autopilot has no SIM_ parameters: SYSID_THISMAV present
            and SIM_SPEEDUP absent marks it `hardware`. Never assumed.
            """
            deadline = time.time() + 20.0
            vals: dict = {}
            while time.time() < deadline:
                vals = self._get_params(self.cli_param, ["SIM_SPEEDUP",
                                                         "SYSID_THISMAV"])
                sp = vals.get("SIM_SPEEDUP")
                if isinstance(sp, float) and sp > 0.0:
                    self.hil_evidence["autopilot_kind"] = "sitl"
                    return round(sp, 4)
                rclpy.spin_once(self, timeout_sec=0.5)
            # Only after the whole wait: while MAVROS is still loading the
            # parameter set, SYSID_THISMAV can be present before SIM_* is.
            hardware = bool(vals.get("SYSID_THISMAV")) and \
                "SIM_SPEEDUP" in vals and vals["SIM_SPEEDUP"] is None
            self.hil_evidence["autopilot_kind"] = "hardware" if hardware else None
            if not hardware:
                self.get_logger().warn("[speedup] SIM_SPEEDUP never resolved")
            return None

        def read_autopilot_version(self) -> None:
            """AUTOPILOT_VERSION as MAVROS received it (ARCH-31); the setup
            script's banner only as a labelled fallback."""
            deadline = time.time() + 15.0
            while time.time() < deadline:
                try:
                    res = self._call(self.cli_info, VehicleInfoGet.Request(
                        sysid=0, compid=0, get_all=False), 5.0)
                except Exception:                             # noqa: BLE001
                    res = None
                if res is not None and res.success and res.vehicles:
                    v = res.vehicles[0]
                    if v.available_info & 2:      # HAVE_INFO_AUTOPILOT_VERSION
                        custom = decode_custom_version(v.flight_custom_version)
                        self.hil_evidence.update({
                            "ardupilot_version": describe_autopilot_version(
                                v.flight_sw_version, custom, v.type, v.autopilot),
                            "ardupilot_version_source":
                                "AUTOPILOT_VERSION via /mavros/vehicle_info_get",
                            "flight_sw_version": f"{int(v.flight_sw_version):08x}",
                            "flight_custom_version_raw": v.flight_custom_version,
                        })
                        return
                rclpy.spin_once(self, timeout_sec=0.5)
            env = os.environ.get("ARDUPILOT_VERSION")
            if env:
                self.hil_evidence.update({
                    "ardupilot_version": env,
                    "ardupilot_version_source":
                        "sitl/setup_sitl.sh --verify binary banner (the "
                        "autopilot itself did not answer)",
                    "ardupilot_pin_ok": os.environ.get("ARDUPILOT_PIN_OK") == "1"})
            else:
                self.get_logger().warn("autopilot version not read")

        def read_link(self) -> None:
            """The MAVLink link MAVROS flies over: its own fcu_url parameter,
            else the URL the launch script started it with (labelled)."""
            url = self._get_params(self.cli_mavros_param, ["fcu_url"]).get("fcu_url")
            src = "/mavros fcu_url parameter"
            if not url:
                url, src = os.environ.get("MAVROS_FCU_URL"), "MAVROS_FCU_URL (launch script)"
            self.hil_evidence["network_link"] = {**network_link_evidence(url),
                                                 "source": src if url else None}
            self.hil_evidence["mavlink_router"] = os.environ.get("MAVLINK_ROUTER")
            # Which MAVLink actors run on THIS host (None off Linux): where
            # SITL and MAVROS are decides dev vs hil, whatever the URL says.
            procs = scan_local_processes()
            self.hil_evidence["local_processes"] = procs
            self.hil_evidence["mavros_on"] = mavros_location(self.hil_evidence)

        def _set_param(self, name: str, value: float) -> bool:
            """One autopilot parameter through MAVROS 2 (/mavros/param is a
            node whose parameters are the autopilot's)."""
            p = Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
            try:
                res = self._call(self.cli_param_set,
                                 SetParameters.Request(parameters=[p]), 5.0)
            except Exception:                                 # noqa: BLE001
                return False
            return bool(res and res.results and res.results[0].successful)

        def read_autopilot_fence(self) -> None:
            """The GeoFence parameters as the AUTOPILOT holds them, not as the
            .parm file says (GUARDRAIL_FENCE=0 starts SITL without it, and the
            node flies any --policy). FENCE_ALT_MAX is raised above the
            policy's ceiling when it sits below it, so the backstop is never
            stricter than the policy; if that fails the record says so."""
            vals: dict = {}
            deadline = time.time() + 20.0
            while time.time() < deadline:
                vals = self._get_params(self.cli_param,
                                        list(FENCE_PARAMS) + ["SIM_SPEEDUP"])
                # ALL of them: MAVROS fills /mavros/param while it pulls the
                # parameter list, and a 2026-10-07 flight read FENCE_ENABLE
                # before FENCE_ALT_MAX existed, so no zone was exported.
                if all(vals.get(k) is not None for k in FENCE_PARAMS):
                    break
                rclpy.spin_once(self, timeout_sec=0.5)
            ceiling, ceiling_src = policy_ceiling_m(self.policy,
                                                    self.mission.cruise_alt_m)
            alt_max = vals.get("FENCE_ALT_MAX")
            plan = fence_alt_plan(ceiling, alt_max)
            raised_from = None
            # Only SITL's parameters are changed by this node. On a hardware
            # autopilot the grant has the GeoFence parameters "GCS-set", and a
            # change would persist in its EEPROM: there a fence below the
            # policy's ceiling is recorded (stricter_than_policy), not fixed.
            is_sitl = isinstance(vals.get("SIM_SPEEDUP"), float) \
                and vals["SIM_SPEEDUP"] > 0.0
            if plan["raise_to"] is not None and alt_max is not None and is_sitl:
                if self._set_param("FENCE_ALT_MAX", plan["raise_to"]):
                    raised_from = alt_max
                    alt_max = self._get_params(
                        self.cli_param, ["FENCE_ALT_MAX"]).get("FENCE_ALT_MAX")
            enabled = vals.get("FENCE_ENABLE")
            self.autopilot_fence = {
                **{k: vals.get(k) for k in FENCE_PARAMS},
                "FENCE_ALT_MAX": alt_max,
                "source": ("/mavros/param (read from the autopilot)"
                           if enabled is not None else None),
                "parm_file": {k: v for k, v in read_parm(FENCE_PARM).items()
                              if k in FENCE_PARAMS},
                "policy_ceiling_m": ceiling, "policy_ceiling_source": ceiling_src,
                "fence_alt_max_raised_from": raised_from,
                "raise_allowed": is_sitl,
                "backstop_active": (None if enabled is None else enabled >= 1.0),
                "stricter_than_policy": (
                    None if alt_max is None or ceiling is None
                    else not fence_alt_plan(ceiling, alt_max)["ok"]),
                "plan_why": plan["why"],
            }
            self.hil_evidence["autopilot_fence"] = self.autopilot_fence
            self.fence_column_m = alt_max
            if vals.get("FENCE_MARGIN") is not None:
                self.fence_margin_m = float(vals["FENCE_MARGIN"])
            self.rec.event("autopilot_fence", **self.autopilot_fence)
            self.get_logger().info(
                f"autopilot fence: ENABLE {enabled}, ALT_MAX {alt_max}"
                + (f" (raised from {raised_from})" if raised_from is not None else "")
                + f", AVOID_ENABLE {vals.get('AVOID_ENABLE')}, policy ceiling "
                f"{ceiling} m")
            if self.autopilot_fence["backstop_active"] is False:
                self.get_logger().warn("FENCE_ENABLE is 0: no GeoFence backstop "
                                       "behind the Shield on this run")
            if self.autopilot_fence["stricter_than_policy"]:
                self.get_logger().warn(f"backstop stricter than the policy: "
                                       f"{plan['why']}")

        def bring_up(self) -> None:
            log = self.get_logger()
            self.serve_hot_apply()
            t_end = time.time() + 180
            while time.time() < t_end and not self.mav_connected:
                rclpy.spin_once(self, timeout_sec=0.5)
            if not self.mav_connected:
                raise RuntimeError("mavros never connected to FCU")
            log.info("FCU connected")
            self.read_autopilot_version()
            log.info(f"autopilot: {self.hil_evidence.get('ardupilot_version')} "
                     f"({self.hil_evidence.get('ardupilot_version_source')})")
            self.read_link()

            # ArduPilot streams nothing until asked.
            self._call(self.cli_rate,
                       StreamRate.Request(stream_id=0, message_rate=10, on_off=True))
            log.info("stream rate 10 Hz requested")

            # The backstop as the autopilot holds it, then this generation's
            # zones - once the aircraft's position is known, so a zone over
            # the pad is held back rather than armed under the aircraft.
            self.read_autopilot_fence()
            pose_deadline = time.time() + 10.0
            while self.state is None and time.time() < pose_deadline:
                rclpy.spin_once(self, timeout_sec=0.5)
            self._publish_fences("take-off")

            # Give the adapter a moment to upload the fences before arming, so
            # the autopilot's backstop is in place from take-off.
            fence_deadline = time.time() + 15.0
            while time.time() < fence_deadline and not self.fence_status:
                rclpy.spin_once(self, timeout_sec=0.5)
            log.info("autopilot fence: " + (json.dumps(self.fence_status[-1])
                                            if self.fence_status else
                                            "no upload reported by the adapter"))

            while time.time() < t_end:
                r = self._call(self.cli_mode, SetMode.Request(custom_mode="GUIDED"))
                if r and r.mode_sent:
                    break
                time.sleep(1)
            log.info("GUIDED requested")

            while time.time() < t_end:
                r = self._call(self.cli_arm, CommandBool.Request(value=True))
                if r and r.success:
                    log.info("armed")
                    break
                time.sleep(2)
                rclpy.spin_once(self, timeout_sec=0.1)
            else:
                raise RuntimeError("arming timed out (EKF)")

            alt = self.mission.cruise_alt_m
            while time.time() < t_end:
                r = self._call(self.cli_tol, CommandTOL.Request(altitude=float(alt)))
                if r and r.success:
                    log.info(f"takeoff accepted -> {alt} m")
                    break
                time.sleep(2)
            else:
                raise RuntimeError("takeoff never accepted")

            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.state and self.state.up >= alt - 1.0:
                    # ArduPilot may set home only when the EKF is ready (at
                    # arming), and the adapter uploads the fence once home is
                    # known: give it a bounded wait before the mission.
                    wait_end = time.time() + 10.0
                    while not self.fence_status and time.time() < wait_end:
                        rclpy.spin_once(self, timeout_sec=0.5)
                    # FENCE_ALT_MAX still unread (MAVROS slow to fill its
                    # parameters): read again now, and re-send the zones,
                    # which were all withheld without a column.
                    if self.fence_column_m is None:
                        self.read_autopilot_fence()
                        if self.fence_column_m is not None:
                            self._publish_fences("fence parameters read late")
                            late_end = time.time() + 5.0
                            while time.time() < late_end:
                                rclpy.spin_once(self, timeout_sec=0.5)
                    self.rec.event("fence_before_mission",
                                   uploaded=bool(self.fence_status and
                                                 self.fence_status[-1].get("success")),
                                   fence_alt_max_m=self.fence_column_m)
                    log.info(f"at {self.state.up:.1f} m - mission start")
                    # Read here, not in _finish(): a service call from a timer
                    # callback is a nested spin and never completes.
                    self.sim_speedup = self.read_sim_speedup()
                    log.info(f"SIM_SPEEDUP read from autopilot: {self.sim_speedup}")
                    return
            raise RuntimeError("never reached takeoff altitude")

        # ---------------- mission tick ---------------- #

        def start_mission(self) -> None:
            # The first mission tick took 116 ms on a 2026-10-07 flight (33 ms
            # on the pymavlink rail; every later tick <= 3.2 ms), over the
            # grant's 100 ms tick budget. One Shield call before the mission,
            # on a throwaway Shield (the real one's history stays the
            # flight's), and its time recorded: on the two flights since, the
            # first mission tick took 1.2 ms. The spike's cause is not
            # established; the record keeps it visible if it recurs.
            if self.state is not None:
                warm = rail_shield(self.policy, lookahead_s=LOOKAHEAD_S,
                                   dt=LOOKAHEAD_DT_S, clock=self.clock)
                b = self.raw_body
                warm.filter(self.state, from_body(b.vx, b.vy, b.vz_up,
                                                  b.yaw_rate, self.state.yaw_deg))
                self.rec.event("shield_warmup", t=None,
                               ms=round(warm.history[-1].elapsed_ms, 3))
            self.mission_t0 = time.monotonic()
            self.watchdog.start(self.mission_t0)
            self.rec.event("mission_start", t=0.0,
                           target=[self.mission.target_x, self.mission.target_y],
                           speed_pref_mps=self.mission.speed_pref_mps,
                           shield=self.args.shield,
                           rule_clock_reading=self.clock().isoformat(
                               timespec="seconds"),
                           action_timeout_s=self.watchdog.timeout_s)
            self.create_timer(TICK, self._tick)

        def serve_hot_apply(self) -> None:
            """The REST hot-apply API on this Shield (--hotapply-port), started
            on the pad by bring_up(): a listener that cannot start (FastAPI
            missing from the venv, the port taken, the PKI absent) stops the
            run before arming, never mid-air. Events before mission start
            change the mission-start policy (Shield.hot_apply)."""
            port = getattr(self.args, "hotapply_port", None)
            if not port:
                return
            from guardrail.api import FlightHotApply
            self.hot = FlightHotApply(self.shield, self.rec, self._lock,
                                      mission_t=self._t,
                                      after=self._after_hot_apply,
                                      log_fn=self.get_logger().info)
            th = self.hot.start(self.args.hotapply_host, int(port),
                                pki_dir=self.args.hotapply_pki,
                                server_name=self.args.hotapply_server_cert,
                                allowed_clients=self.args.hotapply_client)
            self.get_logger().info(f"hot-apply API at {th.url}")

        def _after_hot_apply(self, result: dict, t: float | None) -> None:
            """A REST hot-apply was accepted (under the tick's lock): the new
            generation's zones go to the adapter, a zone the aircraft is in
            or near held back as for every other generation."""
            self._publish_fences(f"hot_apply {result['applied']} "
                                 f"({result['op']} {result['type']}) over REST")

        def _tick(self) -> None:
            tick_ns = time.monotonic_ns()
            # The REST hot-apply thread swaps the Shield's rules under the
            # same lock (guardrail.api.FlightHotApply): a tick never sees half
            # a swap, and a swap never lands between a decision and its row.
            with self._lock:
                self._tick_locked(tick_ns)

        def _streaming(self) -> bool:
            """Is this node flying the aircraft with its setpoints (no
            autopilot takeover, no FSM fault, the FSM in a GUIDED state)?"""
            return (self.takeover is None and self.fsm_fault is None
                    and (self.fsm is None
                         or MODE_FOR[self.fsm.state] == "GUIDED"))

        def _watch_actions(self, now: float) -> tuple[bool, bool]:
            """The stale-action watchdog, once per tick: `(stale, hold_onset)`.
            A stream quiet for longer than the timeout is held the way the
            FSM holds the aircraft (LOITER through the adapter, flown as
            BRAKE) while this node is the one flying it; the next action
            returns it to GUIDED."""
            edge = self.watchdog.check(time.monotonic())
            age = self.watchdog.age(time.monotonic())
            if edge == "stale":
                self.get_logger().warn(
                    f"no /vla/action_4d for {age:.2f} s (timeout "
                    f"{self.watchdog.timeout_s:g} s): holding")
                self.rec.event("action_stale", t=now, quiet_s=round(age, 3),
                               timeout_s=self.watchdog.timeout_s,
                               actions_received=self.n_actions)
            elif edge == "resumed":
                self.get_logger().info("/vla/action_4d resumed")
                self.rec.event("action_resumed", t=now,
                               held=self._stale_hold)
                if self._stale_hold:
                    self._stale_hold = False
                    self._request_mode("GUIDED", "action stream resumed")
            onset = False
            if self.watchdog.stale and not self._stale_hold and self._streaming():
                self._stale_hold = onset = True
                self._request_mode("LOITER", f"action stream stale: no "
                                   f"/vla/action_4d for {age:.2f} s")
            return self.watchdog.stale, onset

        def _tick_locked(self, tick_ns: int) -> None:
            if self.done or self.state is None:
                return
            now = time.monotonic() - self.mission_t0
            if now > self.max_s:
                self.get_logger().warn("mission time cap")
                self._finish("time cap")
                return
            st = self.state

            # Re-assert 10 Hz streams: a ground station on the same router
            # link (Mission Planner) sets its own rates for the whole link.
            if now - self._last_rate_req > 5.0 and self.cli_rate.service_is_ready():
                self.cli_rate.call_async(StreamRate.Request(
                    stream_id=0, message_rate=10, on_off=True))
                self._last_rate_req = now

            if self.dynamic and self.spawn_t is None and now >= DYNAMIC_AT_S:
                fence = (DYNAMIC_FENCE if self.args.dynamic_zone == "route"
                         else zone_around(st.x, st.y))
                self.dynamic_fence_id = fence.id
                # As a dynamic_nfz where the Shield enforces the grant's
                # mid-flight update model (EpisodeRecord.apply_zone).
                applied = self.rec.apply_zone(self.shield, fence, t=now)
                self._publish_fences(f"hot_apply {applied.id} ({applied.type})")
                self.spawn_t, self.spawn_pos = now, (st.x, st.y)
                self.get_logger().info(
                    f"dynamic NFZ applied t={now:.1f}s -> generation "
                    f"{self.policy.generation}")

            dist = math.hypot(self.mission.target_x - st.x,
                              self.mission.target_y - st.y)
            if dist < REACH_M and self.takeover is None:
                self.reached = True
                self.get_logger().info(f"target reached t={now:.1f}s")
                self._finish("target reached")
                return

            # A quiet action stream is not flown: the Shield checks a stop
            # instead, and the node holds (stale-action watchdog).
            stale, hold_onset = self._watch_actions(now)
            body = Action4D() if stale else self.raw_body
            seq, rx_ns = self._action_seq, self._action_rx_ns
            # This tick's row number (1-based), bound once: the audit record
            # and the log row of one decision carry the same tick.
            tick_no = len(self.traj) + 1
            decision, raw_w, em_w, em_body = shield_step(
                self.shield, st, body, self.shield_on)
            extra = episode_row_fields(self.shield, st, self.policy,
                                       self.rec.episode_id)
            self._recheck_deferred(st, extra["unsafe_rules"])

            fsm_out = None
            stop_illegal = None
            unsafe_depth = None
            setpoint = "pass"
            # Once the autopilot has taken the aircraft (a GeoFence RTL), the
            # FSM no longer steps: its mode requests would fight the autopilot.
            # Nor while this node holds it for a quiet action stream.
            if self.fsm is not None and self.fsm_fault is None \
                    and self.takeover is None and not self._stale_hold:
                try:
                    stop_illegal = self.shield.state_is_unsafe(st)
                    # How far from legal: X6's progress and X7's losing-ground
                    # tests read its trend.
                    unsafe_depth = self.shield.unsafe_depth(st, stop_illegal)
                    inp = tick_input_from_decision(
                        now, decision, self.policy, horizon_s=self.theta_horizon_s,
                        rtl_failed=False,
                        home_reached=(self.mode == "RTL"
                                      and math.hypot(st.x, st.y) < REACH_M),
                        landed=bool(self.landed) and self.mode in ("RTL", "LAND"),
                        stop_illegal=stop_illegal, unsafe_depth_m=unsafe_depth,
                        # The caps the Shield clamped against on this tick,
                        # not every cap the policy lists (Shield.applied_caps).
                        caps=self.shield.applied_caps or False)
                    fsm_out = self.fsm.step(inp)
                except ValueError as e:
                    # A Shield/FSM contract break mid-flight: hold, say why,
                    # stop streaming. The run is not KPI-grade.
                    self.fsm_fault = str(e)
                    self.get_logger().error(f"FSM input refused: {e}")
                    self.rec.event("fsm_fault", t=now, error=str(e))
                    self._request_mode("LOITER", "FSM input refused")
                    self.failsafe_t = now       # end the episode after the grace
            if fsm_out is not None:
                setpoint = fsm_out.setpoint
                if fsm_out.transition:
                    self.rec.event("fsm_transition", t=now,
                                   **{k: fsm_out.record[k] for k in (
                                       "fsm_state_before", "fsm_state_after",
                                       "transition", "edge", "edge_source",
                                       "reason", "fsm_config_hash")})
                if fsm_out.set_mode:
                    self._request_mode(fsm_out.set_mode, fsm_out.reason)
                if fsm_out.state in FAILSAFE_STATES and self.failsafe_t is None:
                    self.failsafe_t = now
            if self.fsm_fault is not None or self.takeover is not None:
                setpoint = "none"
            elif self._stale_hold:
                # The hold's first tick flies the Shield's answer to a stop
                # (a stop where it is legal) while SET_MODE goes through, as
                # the FSM's own hand-over does; then the autopilot holds.
                setpoint = "pass" if hold_onset else "none"
            # After the FSM stepped, with ITS verdict (not a Shield-internal
            # FSM's): the audit and the flight log describe one state machine.
            audit_tick(self.audit, tick_no, decision, fsm_out=fsm_out,
                       fsm_fault=self.fsm_fault)

            if decision.touched and self.shield_on:
                self.n_touched += 1
                self.n_braked += int(decision.braked)
                self.pub_intercept.publish(String(data=json.dumps({
                    "t": round(now, 3),
                    "violations": [v.rule_id for v in decision.violations],
                    "repairs": [r.operator for r in decision.repairs],
                    "fsm": fsm_out.state.value if fsm_out else None})))

            flown = {"pass": em_body, "brake": Action4D(), "none": None}[setpoint]
            pub_ns = None
            if flown is not None:
                self._publish_body(flown)
                pub_ns = time.monotonic_ns()
            lat = latency_fields(tick_ns, pub_ns, rx_ns,
                                 fresh=(not stale and seq != self._flown_seq))
            if pub_ns is not None and not stale:
                self._flown_seq = seq

            # The rule clock's reading rides with each point, so the zone
            # dwell applies the windows the Shield flew under.
            self.traj.append({"t": round(now, 2), "x": st.x, "y": st.y,
                              "up": st.up,
                              "touched": bool(self.shield_on and decision.touched),
                              "clock": self.clock().isoformat()})
            self.rows.append(build_row(
                now=now, tick=tick_no, state=st, body=body,
                decision=decision, raw_world=raw_w, emitted_world=em_w,
                emitted_body=flown, shield_on=self.shield_on, extra=extra,
                subject=self.subject, fsm_out=fsm_out,
                stop_illegal=stop_illegal, mode=self.mode, setpoint=setpoint,
                shield=self.shield, unsafe_depth=unsafe_depth))
            self.rows[-1].update(lat, action_seq=seq, action_stale=stale,
                                 stale_hold=self._stale_hold)

            # The episode ends when the autopilot has the aircraft and is done
            # with it: landed, or a bounded wait so a run stays short.
            if self.takeover is not None and (
                    self.landed or now - self.takeover["t"] > TAKEOVER_GRACE_S):
                self._finish(f"autopilot took over ({self.takeover['mode']})")
            elif self.fsm is not None and (self.fsm.terminal or (
                    self.failsafe_t is not None
                    and now - self.failsafe_t > FAILSAFE_GRACE_S)):
                self._finish("escalation FSM ended the mission")

        # ---------------- wrap-up ---------------- #

        def _standoff_metrics(self) -> dict:
            if self.subject is None:
                return {"standoff_s": None, "standoff_min_range_m": None,
                        "subject": None}
            rings = [c.min_range_m for c in self.policy.by_type(SubjectStandoff)
                     if c.binds(self.subject_class)]
            ring = max(rings) if rings else 0.0
            d = [math.hypot(p["x"] - self.subject[0], p["y"] - self.subject[1])
                 for p in self.traj]
            return {
                "standoff_s": round(sum(1 for v in d if v < ring) * TICK, 2),
                "standoff_min_range_m": (round(min(d), 2) if d else None),
                "subject": {"class": self.subject_class, "x": self.subject[0],
                            "y": self.subject[1], "position_source": "declared"},
            }

        def _finish(self, why: str) -> None:
            self.done = True
            if self.hot is not None:
                # Later events would land after this episode's records.
                self.hot.close(f"the episode ended ({why})")
            self.rec.end(why, t=self._t())
            # The mission-end LAND is not a fail-safe. If the FSM or the
            # autopilot already has the aircraft in RTL / LAND, leave it be.
            fsm_owns = self.fsm is not None and (
                self.fsm.terminal or self.fsm.state in FAILSAFE_STATES)
            if not fsm_owns and self.mode not in ("RTL", "LAND"):
                self._request_mode("LAND", "mission end")
            self._report()

        def _report(self) -> None:
            # Drawn in the plot below.
            fences = [(f, fence_polygon(f)) for f in keepout_zones(self.policy)]
            # Zone dwell from the Shield's own rule checker, the count every
            # rail shares (guardrail.replay.zone_dwell); the hot-applied zone
            # counts from when it went live, and a moving zone is named, not
            # scored where it was declared (as it was until 2026-10-11). The
            # spawn time on the trajectory's 0.01 s grid (traj "t" is rounded
            # so), or the spawn tick is dropped when the clock read past it.
            # A zone spawned over REST counts from its arrival too (its mission
            # time, on the same grid; guardrail.api.FlightHotApply.live_from).
            live = dict(self.hot.live_from) if self.hot is not None else {}
            if self.dynamic_fence_id and self.spawn_t is not None:
                live[self.dynamic_fence_id] = round(self.spawn_t, 2)
            self.dwell = zone_dwell(self.shield, self.traj, TICK,
                                    live_from=live or None, clock_key="clock")
            nfz_s = self.dwell["nfz_s"]
            if self.dwell["nfz_moving_not_counted"]:
                self.get_logger().warn(
                    f"moving zone(s) {self.dwell['nfz_moving_not_counted']} are NOT "
                    f"in nfz_s: the end-of-flight count has no per-tick position")
            kpi = "PASS" if nfz_s == 0 else "FAIL"

            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                fig, ax = plt.subplots(figsize=(7, 7))
                for f, poly in fences:
                    dyn = f.id == self.dynamic_fence_id
                    xs, ys = poly.exterior.xy
                    ax.fill(ys, xs, alpha=0.25, color="purple" if dyn else "red",
                            label="dynamic NFZ" if dyn else f"NFZ {f.id}")
                if self.spawn_pos:
                    ax.plot(self.spawn_pos[1], self.spawn_pos[0], "X",
                            color="purple", markersize=12)
                if self.traj:
                    ax.plot([p["y"] for p in self.traj], [p["x"] for p in self.traj],
                            "-", color="tab:blue", linewidth=2, label="flight path")
                    tx = [p["y"] for p in self.traj if p["touched"]]
                    if tx:
                        ax.plot(tx, [p["x"] for p in self.traj if p["touched"]], ".",
                                color="orange", markersize=4, label="shield active")
                    ax.plot(self.traj[0]["y"], self.traj[0]["x"], "go",
                            markersize=10, label="start")
                ax.plot(self.mission.target_y, self.mission.target_x, "k*",
                        markersize=16, label="target")
                ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
                ax.set_title(f"ROS 2 + MAVROS rail - shield "
                             f"{'ON' if self.shield_on else 'OFF'}\n"
                             f"NFZ time: {nfz_s:.1f}s | "
                             f"{'reached' if self.reached else 'NOT reached'}")
                ax.legend(loc="upper left", fontsize=9)
                ax.set_aspect("equal"); ax.grid(alpha=0.3)
                fig.tight_layout(); fig.savefig(self.out / "trajectory.png", dpi=130)
            except ImportError:
                pass

            (self.out / "trajectory.json").write_text(json.dumps(self.traj),
                                                      encoding="utf-8")

            envelopes = self.policy.by_type(AltitudeEnvelope)
            alt_bad = sum(1 for p in self.traj for e in envelopes
                          if not (e.alt_min_m <= p["up"] <= e.alt_max_m))
            alt_s = alt_bad * TICK
            shield_ms = [r["shield_ms"] for r in self.rows
                         if isinstance(r.get("shield_ms"), (int, float))]
            dur = self._t() or 0.0
            ident = self.vla_identity or {}
            model_id = ident.get("model_id") or "unresolved:no-vla-identity"
            pilot = None
            if ident.get("model_id") and ident.get("kind") in ("vla", "stub", "none"):
                pilot = pilot_record(ident["model_id"], ident["kind"],
                                     yaw_rate=ident.get("yaw_rate"),
                                     frame=ident.get("frame"))

            metrics = {
                "tag": self.out.name,
                "episode_id": self.rec.episode_id,
                "ticks": len(self.rows),
                # Ticks on which the VLA's (or the Shield's) action was sent,
                # and those on which the autopilot or a fault held the
                # aircraft and nothing was (flown: false in the row).
                "ticks_flown": sum(1 for r in self.rows if r.get("flown")),
                "ticks_not_flown": sum(1 for r in self.rows
                                       if r.get("flown") is False),
                "shield": "on" if self.shield_on else "off",
                "shield_lookahead": {"horizon_s": LOOKAHEAD_S,
                                     "dt_s": LOOKAHEAD_DT_S,
                                     "poses": int(round(LOOKAHEAD_S
                                                        / LOOKAHEAD_DT_S))},
                "reached": self.reached,
                # compute() reads this as "did the mission happen"; a waypoint
                # mission expresses reaching the target in its units.
                "frac_within_30m": 1.0 if self.reached else 0.0,
                "nfz_s": round(nfz_s, 2),
                "nfz_by_zone_s": self.dwell["nfz_by_zone_s"],
                "nfz_moving_not_counted": self.dwell["nfz_moving_not_counted"],
                "nfz_off_by_clock_s": self.dwell["nfz_off_by_clock_s"],
                **self._standoff_metrics(),
                "alt_violation_s": round(alt_s, 2),
                "interventions": self.n_touched,
                "brakes": self.n_braked,
                "frame": "body (grant Architecture constraints p2)",
                "seed": self.args.seed,
                "target": [self.mission.target_x, self.mission.target_y],
                "speed_pref_mps": self.mission.speed_pref_mps,
                "pilot": pilot,
                "vla_identity": self.vla_identity,
                "vla_actions_received": self.n_actions,
                "pose_hz": (round(self.n_pose / dur, 2) if dur > 0 else None),
                "shield_ms_p50": percentile(shield_ms, 50),
                "shield_ms_p99": percentile(shield_ms, 99),
                "shield_ms_max": (round(max(shield_ms), 3) if shield_ms else None),
                "fsm": ({**self.fsm.summary(),
                         "theta_basis": theta_basis(self.theta_horizon_s),
                         "fault": self.fsm_fault} if self.fsm else None),
                "autopilot_took_over": self.takeover,
                # Both vocabularies (mavlink_adapter_node.ModeRequests): the
                # FSM's words, and what ArduCopter was asked for (a request
                # through the adapter is flown as the adapter's hold mode).
                "mode_requests": list(self._asks.fsm),
                "autopilot_mode_requests": list(self._asks.autopilot),
                "fence_upload": (self.fence_status[-1] if self.fence_status else None),
                "fence_uploads": self.fence_status,
                "fences_still_deferred": self.deferred_fences,
                "zones_over_home": self.zones_over_home,
                "autopilot_fence": self.autopilot_fence or None,
                "generations": self.rec.generations,
                # Which clock the rules were evaluated on (scripted or wall).
                "rule_clock": self.clock_record,
                "hot_apply": (self.hot.summary() if self.hot is not None
                              else None),
                "action_watchdog": self.watchdog.summary(
                    lambda v: v - self.mission_t0),
                "tick_latency": latency_summary(self.rows),
                # Beside the manifest, which is exactly the grant's six fields.
                "hil_evidence": self.hil_evidence,
                "policy_source": self.policy_source,
            }
            (self.out / "flight_log.jsonl").write_text(
                "".join(json.dumps(r) + "\n" for r in self.rows), encoding="utf-8")

            speedup = self.sim_speedup
            topology = detect_topology(self.hil_evidence)
            try:
                manifest = build_manifest(
                    policy_hash=self.policy.policy_hash, model_id=model_id,
                    seed=self.args.seed, scene_path=None, sim_speedup=speedup,
                    topology=topology, hil_evidence=self.hil_evidence)
            except ValueError as e:
                self.get_logger().warn(f"not the {topology} topology: {e}")
                metrics["topology_refused"] = str(e)
                manifest = build_manifest(
                    policy_hash=self.policy.policy_hash, model_id=model_id,
                    seed=self.args.seed, scene_path=None, sim_speedup=speedup,
                    topology=TOPOLOGY_ARDUPILOT_SITL)
            metrics["topology"] = manifest["topology"]
            (self.out / "metrics.json").write_text(json.dumps(metrics, indent=2),
                                                   encoding="utf-8")
            (self.out / "manifest.json").write_text(json.dumps(manifest, indent=2),
                                                    encoding="utf-8")

            kpi_res = compute_from_dir(self.out, self.policy)
            graded, why = is_kpi_grade(manifest, {**metrics, "det_hz": None,
                                                  "start_heading_err_deg": None})
            if self.fsm_fault:
                graded, why = False, why + [f"FSM fault: {self.fsm_fault}"]
            if self.policy_source.get("dev_yaml"):
                graded, why = False, why + [
                    f"the policy was flown from YAML under {DEV_YAML_FLAG}: "
                    f"unsigned, not its issued bundle"]
            kpi_res["kpi_grade"] = graded
            kpi_res["kpi_grade_reasons"] = why
            kpi_res["manifest"] = manifest
            kpi_res["policy_source"] = self.policy_source
            kpi_res["episode_id"] = self.rec.episode_id
            kpi_path = self.out / "kpi.json"
            kpi_path.write_text(json.dumps(kpi_res, indent=2), encoding="utf-8")

            # The bundle, while the policy that governed the flight is still in
            # memory. The rosbag is still open here, so it is left out;
            # run_ros2_demo.sh re-packs (python -m guardrail.replay pack) once
            # the bag is closed.
            rb_path = self.out / f"{self.out.name}.replay.tar.gz"
            try:
                rb = write_replay(self.out, self.policy, rb_path,
                                  changelog=f"ros2 {self.out.name}",
                                  policy_source=self.policy_source)
                ok, notes = verify_replay(rb)
                self.get_logger().info(
                    f"[replay] {rb.name} "
                    f"{'re-derives its own KPIs' if ok else 'FAILED verification'}")
                for n_ in notes:
                    self.get_logger().info(f"[replay]   - {n_}")
                # KPI evidence needs a SIGNED bundle: a graded run whose
                # bundle is keyless is demoted, and kpi.json and the bundle
                # are rewritten to say so.
                graded2, why2 = kpi_evidence_grade(graded, why, rb)
                if graded2 != graded:
                    graded, why = graded2, why2
                    kpi_res["kpi_grade"], kpi_res["kpi_grade_reasons"] = graded, why
                    kpi_path.write_text(json.dumps(kpi_res, indent=2),
                                        encoding="utf-8")
                    write_replay(self.out, self.policy, rb_path,
                                 changelog=f"ros2 {self.out.name}",
                                 policy_source=self.policy_source)
            except Exception as exc:                            # noqa: BLE001
                self.get_logger().warn(
                    f"[replay] not written: {type(exc).__name__}: {exc}")
                if graded:
                    graded, why = False, why + [f"no replay bundle: {exc}"]
                    kpi_res["kpi_grade"], kpi_res["kpi_grade_reasons"] = graded, why
                    kpi_path.write_text(json.dumps(kpi_res, indent=2),
                                        encoding="utf-8")
            self.get_logger().info(
                f"P0 escape rate {kpi_res['p0_violation_escape_rate']} | "
                f"NFZ {nfz_s:.1f}s | alt {alt_s:.1f}s | interventions "
                f"{self.n_touched} | topology {manifest['topology']}")
            self.get_logger().info(
                f"KPI-GRADE: {'YES' if graded else 'no'}"
                + ("" if graded else "  (" + "; ".join(why) + ")"))
            fsm_line = (f"{self.fsm.summary()['max_state']} (max state)"
                        if self.fsm else "not run (control arm)")
            (self.out / "report.md").write_text(
                f"""# ROS 2 rail report - shield {'ON' if self.shield_on else 'OFF'}

| Item | Value |
|------|-------|
| Path | vla node -> /vla/action_4d (body) -> safety_shield -> /shield/setpoint (body) -> mavlink_adapter -> /mavros/setpoint_raw/local -> MAVROS 2 -> ArduPilot |
| Autopilot | {self.hil_evidence.get('ardupilot_version')} ({self.hil_evidence.get('ardupilot_version_source')}) |
| Pilot | `{model_id}` |
| Policy | `{self.policy.policy_id}` `{self.policy.policy_hash}` |
| Policy source | {self.policy_source['kind']} `{self.policy_source['path']}` - signature **{self.policy_source['signature']}**{' (--dev-yaml)' if self.policy_source.get('dev_yaml') else ''} |
| Rule clock | {self.clock_record.get('kind')}: {self.clock_record.get('start') or self.clock_record.get('tz')} |
| Hot-apply API | {f"{metrics['hot_apply']['n_applied']} event(s) at {metrics['hot_apply']['endpoint']['url']}, receipt to swap max {metrics['hot_apply']['receipt_to_swap_ms_max']} ms" if self.hot is not None else 'not served'} |
| Action stream | {self.n_actions} received; {self.watchdog.summary()['n_spells']} stale spell(s) (timeout {self.watchdog.timeout_s:g} s) |
| Tick latency (e2e p50 / p99 / max) | {metrics['tick_latency']['e2e_ms']['p50']} / {metrics['tick_latency']['e2e_ms']['p99']} / {metrics['tick_latency']['e2e_ms']['max']} ms, budget {TICK_BUDGET_MS:g} ms |
| Topology | `{manifest['topology']}` |
| Target reached | {'yes' if self.reached else 'NO'} |
| Ticks | {len(self.traj)} ({metrics['ticks_flown']} flown, {metrics['ticks_not_flown']} held by the autopilot or a fault) |
| Shield lookahead | {LOOKAHEAD_S:g} s at {LOOKAHEAD_DT_S:g} s ({metrics['shield_lookahead']['poses']} poses) |
| Shield interventions | {self.n_touched} |
| Escalation FSM | {fsm_line} |
| Autopilot took over | {self.takeover or 'no'} |
| Autopilot fence parameters | {self.autopilot_fence or 'not read'} |
| Autopilot fence upload | {(self.fence_status[-1] if self.fence_status else 'no upload reported')} |
| Dynamic NFZ | {f'hot-applied t={self.spawn_t:.1f}s, generation {self.policy.generation}' if self.spawn_t else 'not used'} |
| **Time inside NFZ** | **{nfz_s:.1f} s** |
| **P0 KPI** | **{kpi}** |
""", encoding="utf-8")
            print(f"[report] NFZ {nfz_s:.1f}s -> {kpi} | touched {self.n_touched} "
                  f"| braked {self.n_braked} | ticks {len(self.traj)}")


def make_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--dynamic", action="store_true")
    ap.add_argument("--dynamic-zone", choices=["route", "on-aircraft"],
                    default="route",
                    help="with --dynamic: 'route' hot-applies the 8 x 8 m zone "
                         "across the route at 8 s; 'on-aircraft' a 10 x 10 m "
                         "zone centred on the aircraft, so the run starts "
                         "unsafe and time to safe is measured")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--policy", default=None,
                    help="policy YAML; flown only with --dev-yaml, recorded as "
                         "UNSIGNED. Without --policy and --bundle the signed "
                         "bundle of policies/sim_demo_policy.yaml flies")
    ap.add_argument("--bundle", default=None,
                    help="signed policy bundle. Refused unless its signature "
                         "verifies.")
    ap.add_argument("--allow-unverified-bundle", action="store_true")
    ap.add_argument("--expected-hash", default=None, metavar="SHA256",
                    help="refuse to fly unless the loaded policy has this hash "
                         "(the stress harness pins every bundle it sends)")
    ap.add_argument(DEV_YAML_FLAG, dest="dev_yaml", action="store_true",
                    help="fly a YAML policy (--policy, or the default YAML): a "
                         "development run, recorded as such and never "
                         "KPI-grade")
    ap.add_argument("--clock-start", default=None, metavar="ISO8601",
                    help="script the rule clock: it reads this at mission start "
                         "and advances with the mission time. Without it the "
                         "wall clock in the mission's time zone (--clock-tz)")
    ap.add_argument("--clock-tz", default=None,
                    help="the mission's time zone for the wall rule clock: a UTC "
                         "offset (+08:00, the default: Taiwan) or an IANA name")
    ap.add_argument("--hotapply-port", type=int, default=None,
                    help="serve the REST hot-apply API (guardrail/api.py) on "
                         "this port, started on the pad before arming")
    ap.add_argument("--hotapply-host", default=DEFAULT_HOTAPPLY_HOST,
                    help="address for --hotapply-port (default 127.0.0.1; any "
                         "other needs --hotapply-pki)")
    ap.add_argument("--hotapply-pki", default=None, metavar="DIR",
                    help="mutual TLS with the harness CA: a folder written by "
                         "python -m guardrail.harness_service gen-pki")
    ap.add_argument("--hotapply-server-cert", default=None, metavar="NAME",
                    help="the server certificate's name in --hotapply-pki "
                         "(default harness-server)")
    ap.add_argument("--hotapply-client", action="append", default=None,
                    metavar="CN",
                    help="client certificate common name allowed in (repeat; "
                         "default harness-client)")
    ap.add_argument("--action-timeout-s", type=float, default=ACTION_TIMEOUT_S,
                    help="hold the aircraft once /vla/action_4d has been quiet "
                         f"this long (default {ACTION_TIMEOUT_S:g} s: "
                         f"{ACTION_TIMEOUT_TICKS} ticks at 10 Hz)")
    ap.add_argument("--command", default=None,
                    help="mission text (default: fly to the northeast pad at 6 m/s)")
    ap.add_argument("--target", default=None, metavar="X,Y",
                    help="override the mission target (local NED metres), so "
                         "the rail can fly a stress scenario's mission")
    ap.add_argument("--speed", type=float, default=None,
                    help="override the pilot's preferred speed, m/s")
    ap.add_argument("--seed", type=int, default=0,
                    help="recorded in the manifest. StubVLA itself draws no "
                         "random numbers.")
    ap.add_argument("--max-s", type=float, default=MAX_S,
                    help="mission time cap, seconds")
    ap.add_argument("--subject", default=None, metavar="X,Y",
                    help="declare a subject at this NED position so "
                         "subject_standoff rules bind (declared, not perceived)")
    ap.add_argument("--subject-class", default="pedestrian")
    ap.add_argument("--fsm-config", default=None,
                    help="escalation FSM thresholds YAML (guardrail.fsm.FSMConfig)")
    ap.add_argument("--theta-horizon-s", default="auto",
                    help="'auto' (the default basis, guardrail.fsm."
                         "DEFAULT_THETA_HORIZON_S: per-operator magnitude_m, the "
                         "forecast depth), 'none' (the same, explicitly), or seconds "
                         "(0.1 = the one-period reading: the setpoint change over one "
                         "control period; pending the PI)")
    ap.add_argument("--hold-mode", choices=list(HOLD_MODES), default=DEFAULT_HOLD_MODE,
                    help="ArduCopter mode the FSM's Loiter is flown as when no adapter "
                         "is subscribed (the adapter's own --hold-mode otherwise)")
    ap.add_argument("--cost-map", action="store_true",
                    help="give every generation's CSP its cost map (costmap_csp_g<N>"
                         ".npz/.json, named in cost_map_ref; guardrail.replay."
                         "EpisodeRecord)")
    return ap


def main(argv: list[str] | None = None) -> None:
    args, ros_args = make_parser().parse_known_args(argv)
    # Refused before ROS starts and before anything flies: a YAML policy
    # without --dev-yaml, no policy and no signed default bundle, a clock
    # start or zone that cannot be read, a timeout that is not one.
    try:
        resolve_policy(args)
        rule_clock(args.clock_start, elapsed=lambda: None, tz=args.clock_tz)
        ActionWatchdog(args.action_timeout_s)
    except ValueError as exc:
        raise SystemExit(f"[safety_shield] REFUSED: {exc}")
    if not HAVE_ROS:
        raise SystemExit("ros2_shield_node needs rclpy and mavros_msgs: source "
                         "/opt/ros/jazzy/setup.bash and use ~/venv-ros/bin/python")
    tag = args.tag or default_tag(args.shield, args.dynamic)
    out = ROOT / "demo" / "out" / tag
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = ShieldNode(args, out, subject=parse_xy(args.subject))
    try:
        node.bring_up()
        node.start_mission()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
        # keep spinning briefly so the LAND request goes out
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 3:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
