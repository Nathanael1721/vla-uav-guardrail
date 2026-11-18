"""
ROS 2 node: the Safety Shield for MISSION-STYLE flight - a pre-planned
waypoint mission that ArduPilot flies in AUTO, guarded waypoint by waypoint.

    [this node] --/shield/mission (JSON)--------> [mavlink_adapter] --/mavros/mission/push--> MAVROS 2 --MISSION_ITEM_INT--> ArduPilot (AUTO)
                --/shield/mission_update (JSON)-> [mavlink_adapter] --push, the mission with that item replaced--> MAVROS 2 --MISSION_COUNT + MISSION_ITEM_INT-->
                --/shield/mode_request--------->  [mavlink_adapter] --/mavros/set_mode-->

WHAT THE GRANT ASKS FOR (WP3-09)

Safety Shield page, the MAVLink adapter table, second row: "Pre-planned
waypoint mission | MISSION_ITEM_INT | For mission-style flight; replaces or
updates the next waypoint." The first row (live setpoints in GUIDED,
SET_POSITION_TARGET_LOCAL_NED) is sitl/ros2_shield_node.py; until 2026-11-03
nothing on any rail wrote a mission item (independent review, 29 Oct).

WHAT THIS NODE DOES

  * PRE-EXECUTION. The plan (--waypoints "x,y;x,y;...", local metres x North,
    y East, at the mission's cruise altitude; default: the mission target)
    is checked leg by leg with the Shield's own rule code before anything is
    uploaded: each leg is walked in LEG_STEP_M steps and every step asked
    `Shield.state_is_unsafe` (would a stop there break a hard, enforced rule
    - the zone with its margin, the altitude band, a corridor, ...). A leg
    that reaches an illegal step has its waypoint REPLACED by the last legal
    point short of it, held there (NAV_WAYPOINT hold time HOLD_S): the
    mission-style answer to "stop short", the same answer the GUIDED Shield
    gives a forecast that enters a zone. The plan's speed is clamped to the
    policy's speed cap (Shield.kinematic_caps), which becomes the mission's
    DO_CHANGE_SPEED. The checked plan goes to the adapter on /shield/mission;
    the adapter writes it as MISSION_ITEM_INT and reads it back.
  * IN FLIGHT, at 10 Hz. The NEXT waypoint (the item the autopilot is flying
    to, /mavros/mission/waypoints current_seq) is re-checked from the
    aircraft's position under the rules in force now - a REST hot-apply
    (--hotapply-port, guardrail.api.FlightHotApply) or a time window may
    have changed them. If its leg has become illegal, that one item is
    REPLACED in the autopilot (/shield/mission_update) by the stop-short
    point; once the original leg has been legal for T_RECOVER_S, the
    original waypoint is written back the same way (UPDATED). This is the
    grant's "replaces or updates the next waypoint": one item changed, in
    place, while ArduPilot flies AUTO; ArduCopter restarts the item it is
    flying when it changes. The adapter writes it as the whole mission with
    that item replaced (since 2026-11-15; until then a partial write, which
    MAVROS 2.14 sends one item too long, so each update ended in ArduPilot's
    8 s upload timeout: sitl/mavlink_adapter_node.py, MISSION_UPDATE_WRITES),
    and its read-back of the items is the update's verdict.
  * The position itself illegal (a zone hot-applied over the aircraft): the
    FSM's LOITER is asked of the adapter (flown as BRAKE) and recorded; AUTO
    is asked again once the position is legal. This node runs no escalation
    FSM: its whole response is the waypoint and that hold.

WHAT IT RECORDS (demo/out/<tag>/), beside the adapter's adapter_log.jsonl

  flight_log.jsonl  one row per tick: position, mode, the autopilot's
                    current item, the next waypoint as planned and as the
                    autopilot holds it, the leg's verdict and the rules it
                    breaks (`violations`), a replacement's size (`repairs`,
                    operator waypoint_replace), policy hash and generation.
  mission.json      the plan as given, the pre-execution check and repair,
                    the items uploaded and read back, every in-flight
                    replacement / update with its timings and read-back.
  policy_g<N>.json, csp_g<N>.json, events.jsonl  (guardrail.replay.EpisodeRecord)
  metrics.json      the mission's outcome and counts, the hot-apply summary,
                    the policy hash at take-off and the final one, the seed,
                    `hil_evidence` (the MAVROS chain, the autopilot's own
                    version, the host, the link) and `topology` (with
                    `topology_refused` when the evidence did not hold).
  manifest.json     the grant's six fields (WP4-12), as every other rail
                    writes them (since 2026-11-06; mission_wp_20261103d and
                    mission_wp_20261104 have none): code_revision;
                    vla_model_hash NO_VLA (no VLA flies this style);
                    policy_hash, the hash in force at the end (the last
                    generation after a hot-apply, which the replay pack binds
                    to; the take-off hash is metrics' and policy_g0.json's);
                    random_seed from --seed; sim_speedup read from the
                    autopilot over MAVROS's parameter service, never a
                    literal; topology from the evidence
                    (guardrail.manifest.detect_topology), the same evidence
                    the GUIDED Shield node gathers (sitl/mavros_evidence.py).

WHAT IT IS NOT

  * Not the VLA's pilot slot: the plan is given on the command line, as a
    pre-planned mission is. No VLA flies this mission style on this rail.
  * Not a replacement for the GUIDED Shield node: no setpoint is sent, no
    escalation FSM runs, and the forecast is the straight leg, not ArduCopter's
    S-curve. The autopilot's own GeoFence (GCS-set, run_ros2_demo.sh
    GCS_BACKSTOP=1) stays the backstop.

Run (normally through sitl/run_ros2_demo.sh with STYLE=mission):

    ~/venv-ros/bin/python sitl/ros2_mission_node.py --tag T --waypoints "0,12;0,30" \\
        [--bundle B] [--speed 3] [--hotapply-port 8071] [--max-s 120] [--seed 0]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import State                                              # noqa: E402
from guardrail.compiler import ConstraintCompiler                        # noqa: E402
from guardrail.manifest import NO_VLA                                    # noqa: E402
from guardrail.replay import EpisodeRecord, rail_shield, rule_clock      # noqa: E402
from mavlink_adapter_node import (FIRST_WAYPOINT_SEQ, TOPIC_MISSION,     # noqa: E402
                                  TOPIC_MISSION_UPDATE, TOPIC_MODE_REQUEST,
                                  TOPIC_STATUS, XYArgumentParser)
from mavros_evidence import (get_params, initial_evidence,              # noqa: E402
                             note_mavros_state, poll_autopilot_version,
                             poll_sim_speedup, record_link, topology_manifest,
                             write_stack_revisions)

TICK_S = 0.1
MAX_S = 180.0
REACH_M = 2.0
# Steps along a leg at which the Shield is asked about a stop.
LEG_STEP_M = 0.5
# How far short of the first illegal step a replacement waypoint sits, on
# top of the zone's own margin (the check includes it): ArduCopter stops on a
# held waypoint to well under a metre; this keeps the stop clear of it.
STANDOFF_M = 1.5
# The hold time of a replacement waypoint: long enough that the mission does
# not move on past it; the original waypoint written back ends the hold.
HOLD_S = 600.0
# The original leg must be legal this long before its waypoint is written
# back (the escalation FSM's T_recover, the grant's 2 s).
T_RECOVER_S = 2.0
# A replacement is re-written only if the stop-short point moved this much.
REPLAN_M = 2.0
# After the mission ends, how long to keep spinning for the LAND to go out.
END_GRACE_S = 3.0


def parse_waypoints(text: str | None) -> list[tuple[float, float]]:
    """"x,y;x,y" -> [(x, y), ...] (local metres, x North, y East)."""
    if not text:
        return []
    out = []
    for part in str(text).replace(" ", "").split(";"):
        if not part:
            continue
        x, y = (float(v) for v in part.split(","))
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"waypoint {part!r} is not finite")
        out.append((x, y))
    return out


def leg_check(shield, start: tuple[float, float, float], end: tuple[float, float, float],
              step_m: float = LEG_STEP_M, terrain=None) -> dict:
    """Walk the straight leg start -> end (x, y, up) and ask the Shield, at
    every step, whether a stop there breaks a HARD, enforced rule
    (`state_is_unsafe` filtered by `hard_violations`, the escalation FSM's
    stop_illegal reading). The start itself is not judged here.

    `terrain` (X-11, 2026-11-18; a guardrail.terrain.TerrainReference, the
    node's --ground grid): the leg's heights are above HOME (a waypoint's
    altitude, as AUTO flies it), so each step is handed to the Shield at its
    height above the DEM under THAT step (TerrainReference.up): a leg level
    above home over rising ground is judged against the ground along it.
    None (every flight without a --ground grid): the heights as given.

    Returns {"legal": True} or {"legal": False, "rules": [...], "at_m":
    distance of the first illegal step, "first_illegal": (x, y, up),
    "last_legal": (x, y, up) the step before it (the start if none)}; the
    points are always above home, the reference the waypoints are in."""
    sx, sy, su = start
    ex, ey, eu = end
    length = math.hypot(ex - sx, ey - sy)
    n = max(1, int(math.ceil(length / step_m)))
    last = (sx, sy, su)
    for i in range(1, n + 1):
        f = i / n
        p = (sx + f * (ex - sx), sy + f * (ey - sy), su + f * (eu - su))
        up = p[2] if terrain is None else terrain.up(p[2], p[0], p[1])
        bad = shield.hard_violations(shield.state_is_unsafe(State(x=p[0], y=p[1], up=up)))
        if bad:
            return {"legal": False, "rules": sorted({v.rule_id for v in bad}),
                    "at_m": round(f * length, 2), "first_illegal": p, "last_legal": last}
        last = p
    return {"legal": True}


def stop_short(start: tuple[float, float, float], verdict: dict,
               standoff_m: float = STANDOFF_M) -> tuple[float, float, float]:
    """The replacement waypoint for an illegal leg: `standoff_m` back from
    the last legal step, toward the start, never behind the start."""
    lx, ly, lu = verdict["last_legal"]
    sx, sy, su = start
    d = math.hypot(lx - sx, ly - sy)
    if d <= standoff_m or d == 0.0:
        return (sx, sy, lu)
    f = (d - standoff_m) / d
    return (sx + f * (lx - sx), sy + f * (ly - sy), lu)


def check_plan(shield, home: tuple[float, float, float],
               plan: list[tuple[float, float, float]], terrain=None
               ) -> tuple[list[dict], list[dict]]:
    """Pre-execution: every leg of the plan, from `home` at cruise altitude.
    Returns (waypoints to upload, one record per leg). An illegal leg's
    waypoint becomes its stop-short point, held (HOLD_S): the mission then
    stops there, as the GUIDED Shield stops short of a zone. `terrain`: as
    leg_check (None without a --ground grid)."""
    out, legs = [], []
    prev = home
    for i, wp in enumerate(plan):
        v = leg_check(shield, prev, wp) if terrain is None else \
            leg_check(shield, prev, wp, terrain=terrain)
        rec = {"index": i, "seq": FIRST_WAYPOINT_SEQ + i, "from": list(prev),
               "to": list(wp), "legal": v["legal"]}
        if v["legal"]:
            out.append({"x": wp[0], "y": wp[1], "up": wp[2], "hold_s": 0.0})
            prev = wp
        else:
            p = stop_short(prev, v)
            rec.update({"rules": v["rules"], "at_m": v["at_m"],
                        "replaced_by": [round(c, 3) for c in p], "hold_s": HOLD_S,
                        "replacement_m": round(math.hypot(p[0] - wp[0], p[1] - wp[1]), 3)})
            out.append({"x": p[0], "y": p[1], "up": p[2], "hold_s": HOLD_S})
            legs.append(rec)
            # The rest of the plan is never reached past a held stop.
            for j in range(i + 1, len(plan)):
                legs.append({"index": j, "seq": FIRST_WAYPOINT_SEQ + j, "legal": None,
                             "to": list(plan[j]), "why": "after a held stop"})
                q = plan[j]
                out.append({"x": q[0], "y": q[1], "up": q[2], "hold_s": 0.0})
            return out, legs
        legs.append(rec)
    return out, legs


def mission_ground(args, bundle, yaml_path):
    """(ground, file record, TerrainReference or None, Shield keywords) the
    mission node flies over (X-11, 2026-11-18): --ground FILE named with
    --ground-sha256 HEX (checked before it is read, terrain.ground_file) and,
    over a DEM, --ground-home-msl M (the autopilot's home elevation, checked
    against the DEM at home, the EKF origin: TerrainReference). As the GUIDED
    Shield node takes them; a flat ground is the home's own and converts
    nothing. (None, None, None, {}) without --ground: every flight before."""
    from guardrail.terrain import TerrainReference, ground_from_args, is_dem
    hmsl = getattr(args, "ground_home_msl", None)
    if hmsl is not None and not getattr(args, "ground", None):
        raise ValueError("--ground-home-msl is the home elevation of a --ground")
    ground, gfile = ground_from_args(args, bundle, yaml_path, root=ROOT)
    if ground is None:
        return None, None, None, {}
    if is_dem(ground):
        terrain = TerrainReference(ground, hmsl, home_xy=(0.0, 0.0),
                                   source="--ground-home-msl")
        return ground, gfile, terrain, {"ground": ground, "up_ref": "terrain"}
    e = float(ground.elevation_at())
    if hmsl is not None and abs(float(hmsl) - e) > 1e-6:
        raise ValueError(f"REFUSED before take-off: --ground-home-msl {float(hmsl):g} and "
                         f"the flat ground {ground!r} differ")
    return ground, gfile, None, {"ground": ground}


def write_run_records(out: Path, metrics: dict, *, policy_hash: str, seed: int,
                      sim_speedup: float | None, evidence: dict,
                      warn=None) -> dict:
    """The mission's end-of-run records: manifest.json - the grant's six
    fields (WP4-12) - beside metrics.json, and `topology` in metrics (with
    `topology_refused` when the evidence did not hold). Returns the manifest.

    The manifest is the GUIDED Shield node's (mavros_evidence.topology_manifest:
    the topology the evidence supports, else the rail's own label and the
    refusal recorded), with NO_VLA as the model: the mission style has no VLA,
    the plan is given on the command line. `policy_hash` is the hash in force
    at the end (the replay pack binds the bundle to it); `sim_speedup` is the
    autopilot's own SIM_SPEEDUP as read (None -> "unresolved"), never a
    literal; `seed` is --seed, recorded (nothing in this style draws a random
    number)."""
    manifest, refused = topology_manifest(
        policy_hash=policy_hash, model_id=NO_VLA, seed=seed,
        sim_speedup=sim_speedup, evidence=evidence, warn=warn)
    if refused is not None:
        metrics["topology_refused"] = refused
    metrics["topology"] = manifest["topology"]
    (out / "metrics.json").write_text(json.dumps(metrics, indent=1, default=str) + "\n",
                                      encoding="utf-8")
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                       encoding="utf-8")
    # Beside the six fields: this host's revision, the autopilot's firmware
    # and the simulator build (ARCH-31, guardrail.manifest STACK_FILE).
    write_stack_revisions(out, manifest, evidence, warn=warn)
    return manifest


try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from mavros_msgs.msg import State as MavState
    from mavros_msgs.msg import StatusText, WaypointList, WaypointReached
    from mavros_msgs.srv import (CommandBool, CommandTOL, SetMode, StreamRate,
                                 VehicleInfoGet)
    from rcl_interfaces.srv import GetParameters
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


if HAVE_ROS:
    LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class MissionShieldNode(Node):
        def __init__(self, args, out: Path) -> None:
            super().__init__("mission_shield")
            from ros2_shield_node import (LOOKAHEAD_DT_S, LOOKAHEAD_S, check_expected_hash,
                                          resolve_policy)
            from guardrail.bundle import load_for_flight
            self.args, self.out = args, out
            bundle, yaml_path, choice = resolve_policy(args)
            # The declared ground (--ground / --ground-sha256 /
            # --ground-home-msl, X-11, 2026-11-18), as the GUIDED Shield node
            # takes it (mission_ground). Without --ground: none, as before.
            self.ground, self.ground_file, self.terrain, ground_kw = mission_ground(
                args, bundle, yaml_path)
            self.policy, self.policy_source = load_for_flight(
                bundle, yaml_path, allow_unverified=args.allow_unverified_bundle,
                **({} if self.ground is None else {"ground": self.ground}))
            self.policy_source.update(choice)
            self.policy_source.update(check_expected_hash(self.policy, args.expected_hash))
            comp = (ConstraintCompiler(self.policy) if self.ground is None
                    else ConstraintCompiler(self.policy, ground=self.ground))
            self.mission = comp.parse_command(args.command or "fly to the northeast pad at 6 m/s")
            wps = parse_waypoints(args.waypoints)
            if args.target:
                tx, ty = (float(v) for v in args.target.split(","))
                self.mission = self.mission.model_copy(update={"target_x": tx, "target_y": ty})
            if not wps:
                wps = [(self.mission.target_x, self.mission.target_y)]
            else:
                self.mission = self.mission.model_copy(
                    update={"target_x": wps[-1][0], "target_y": wps[-1][1]})
            if args.speed is not None:
                self.mission = self.mission.model_copy(update={"speed_pref_mps": float(args.speed)})
            self.alt = float(self.mission.cruise_alt_m)
            self.plan = [(x, y, self.alt) for x, y in wps]
            self.mission_t0: float | None = None
            self.clock, self.clock_record = rule_clock(args.clock_start, elapsed=self._t,
                                                       tz=args.clock_tz)
            self.rec = EpisodeRecord(out, self.policy, self.mission, lookahead_s=LOOKAHEAD_S,
                                     clock=self.clock, clock_record=self.clock_record)
            (out / "prompt.yaml").write_text(comp.build_prompt(self.mission), encoding="utf-8")
            self.shield = rail_shield(self.policy, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S,
                                      clock=self.clock, **ground_kw)
            self._lock = threading.RLock()
            self.hot = None
            self.state: State | None = None
            self.mode: str | None = None
            self.armed = False
            self.connected = False
            self.current_seq: int | None = None
            self.n_items: int | None = None
            self.reached_seqs: list[dict] = []
            self.adapter: list[dict] = []
            self.uploaded = None             # the waypoints as last uploaded
            self.autopilot_wps: list[dict] = []   # what the autopilot should hold, per wp index
            self.replaced: dict[int, dict] = {}   # wp index -> active replacement
            self.legal_since: dict[int, float] = {}
            self.updates: list[dict] = []
            self.asked: list[dict] = []
            self.takeover: dict | None = None
            self.held_unsafe = False
            self.done = False
            self.end_why: str | None = None
            self.tick_no = 0
            self._log_fh = open(out / "flight_log.jsonl", "w", encoding="utf-8", newline="\n")
            self.texts: list[dict] = []
            self.pre: dict = {}
            # The topology's evidence, gathered from the live system as the
            # GUIDED Shield node gathers it (sitl/mavros_evidence.py); no
            # separate VLA node flies this style, so no vla_host.
            self.hil_evidence: dict = initial_evidence(vla_node=False)
            security = self.hil_evidence.get("ros2_security")
            if security is not None:
                self.get_logger().info(
                    f"SROS 2 {security['strategy']}: enclave {security['enclave']}, "
                    f"identity CA {security['identity_ca_sha256']}")
            # SIM_SPEEDUP as the autopilot reports it (read in bring_up, before
            # the tick timer: a service call from a timer callback never
            # completes), and the policy in force when take-off was accepted.
            self.sim_speedup: float | None = None
            self.takeoff_policy: dict | None = None

            self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_pose,
                                     qos_profile_sensor_data)
            self.create_subscription(MavState, "/mavros/state", self._on_state, 10)
            self.create_subscription(StatusText, "/mavros/statustext/recv", self._on_text,
                                     qos_profile_sensor_data)
            self.create_subscription(String, TOPIC_STATUS, self._on_adapter, LATCHED)
            self.create_subscription(WaypointList, "/mavros/mission/waypoints",
                                     self._on_waypoints, LATCHED)
            self.create_subscription(WaypointReached, "/mavros/mission/reached",
                                     self._on_reached, 10)
            self.pub_mission = self.create_publisher(String, TOPIC_MISSION, LATCHED)
            self.pub_update = self.create_publisher(String, TOPIC_MISSION_UPDATE, 10)
            self.pub_mode = self.create_publisher(String, TOPIC_MODE_REQUEST, 10)
            self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
            self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
            self.cli_tol = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
            self.cli_rate = self.create_client(StreamRate, "/mavros/set_stream_rate")
            self.cli_param = self.create_client(GetParameters,
                                                "/mavros/param/get_parameters")
            self.cli_mavros_param = self.create_client(GetParameters,
                                                       "/mavros/get_parameters")
            self.cli_info = self.create_client(VehicleInfoGet, "/mavros/vehicle_info_get")
            self.get_logger().info(
                f"mission-style Shield: policy {self.policy.policy_id} "
                f"{self.policy.policy_hash}, {len(self.plan)} waypoint(s) at {self.alt:g} m")

        # ---------------- inputs ---------------- #

        def _t(self) -> float | None:
            return None if self.mission_t0 is None else time.monotonic() - self.mission_t0

        def _on_pose(self, msg: PoseStamped) -> None:
            p = msg.pose.position
            self.state = State(x=p.y, y=p.x, up=p.z)

        def _on_state(self, msg) -> None:
            self.connected = bool(msg.connected)
            self.armed = bool(msg.armed)
            note_mavros_state(self.hil_evidence, msg.connected)
            if msg.mode != self.mode:
                self.rec.event("mode", t=self._t(), mode=msg.mode, previous=self.mode,
                               armed=self.armed)
                asked = {a["mode"] for a in self.asked[-3:]}
                if (self.mission_t0 is not None and not self.done and self.takeover is None
                        and msg.mode not in asked and msg.mode not in ("AUTO", "GUIDED")
                        and not (msg.mode == "BRAKE" and "LOITER" in asked)):
                    self.takeover = {"t": self._t(), "mode": msg.mode, "previous": self.mode}
                    self.get_logger().warn(f"autopilot took the aircraft: {self.mode} -> "
                                           f"{msg.mode}")
                    self.rec.event("takeover", **self.takeover)
            self.mode = msg.mode

        def _on_text(self, msg) -> None:
            rec = {"t": self._t(), "severity": int(msg.severity), "text": msg.text}
            self.texts.append(rec)
            self.rec.event("autopilot_text", **rec)

        def _on_adapter(self, msg: String) -> None:
            try:
                rec = json.loads(msg.data)
            except ValueError:
                return
            if not isinstance(rec, dict):
                return
            rec["_rx_t"] = self._t()
            rec["_rx_mono"] = time.monotonic()
            self.adapter.append(rec)
            self.rec.event(f"adapter_{rec.get('kind', 'status')}", t=self._t(),
                           **{k: v for k, v in rec.items()
                              if k not in ("kind", "t", "_rx_t", "_rx_mono")})
            if rec.get("kind") == "mission_upload" and rec.get("success"):
                self.n_items = rec.get("items")
            if rec.get("kind") == "mission_update":
                for u in self.updates:
                    if u.get("update_id") == rec.get("update_id"):
                        u["adapter"] = {k: v for k, v in rec.items() if not k.startswith("_")}
                        u["adapter_t"] = rec["_rx_t"]
                        u["sent_to_status_ms"] = round(
                            (rec["_rx_mono"] - u["_sent_mono"]) * 1e3, 1)

        def _on_waypoints(self, msg) -> None:
            seq = int(msg.current_seq)
            if seq != self.current_seq:
                self.rec.event("mission_current", t=self._t(), seq=seq,
                               items=len(msg.waypoints))
            self.current_seq = seq

        def _on_reached(self, msg) -> None:
            r = {"t": self._t(), "seq": int(msg.wp_seq)}
            self.reached_seqs.append(r)
            self.rec.event("mission_item_reached", **r)

        # ---------------- helpers ---------------- #

        def _call(self, client, req, timeout: float = 5.0):
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            return fut.result()

        # The MAVROS chain's evidence: sitl/mavros_evidence.py's readers, the
        # GUIDED Shield node's, handed this node's service calls, spin and
        # clock.

        def _get_params(self, client, names: list[str]) -> dict:
            return get_params(self._call, client, GetParameters.Request, names)

        def _spin_evidence(self) -> None:
            rclpy.spin_once(self, timeout_sec=0.5)

        def read_sim_speedup(self) -> float | None:
            """SIM_SPEEDUP from the autopilot over MAVROS's parameter service,
            or None - never assumed (mavros_evidence.poll_sim_speedup)."""
            return poll_sim_speedup(
                lambda names: self._get_params(self.cli_param, names),
                self._spin_evidence, self.hil_evidence, now=time.time,
                warn=self.get_logger().warn)

        def read_autopilot_version(self) -> None:
            """AUTOPILOT_VERSION as MAVROS received it; the setup script's
            banner only as a labelled fallback
            (mavros_evidence.poll_autopilot_version)."""
            poll_autopilot_version(
                lambda: self._call(self.cli_info, VehicleInfoGet.Request(
                    sysid=0, compid=0, get_all=False), 5.0),
                self._spin_evidence, self.hil_evidence, now=time.time,
                warn=self.get_logger().warn)

        def read_link(self) -> None:
            """MAVROS's fcu_url and where the MAVLink actors run
            (mavros_evidence.record_link)."""
            record_link(self._get_params(self.cli_mavros_param, ["fcu_url"]).get("fcu_url"),
                        self.hil_evidence)

        def _set_mode_direct(self, mode: str, why: str) -> bool:
            """Bring-up modes (GUIDED, AUTO) straight to MAVROS: the adapter
            forwards only the escalation FSM's modes."""
            self.asked.append({"t": self._t(), "mode": mode, "via": "mavros", "why": why})
            r = self._call(self.cli_mode, SetMode.Request(custom_mode=mode))
            ok = bool(r and r.mode_sent)
            self.rec.event("mode_request", t=self._t(), mode=mode, via="/mavros/set_mode",
                           why=why, mode_sent=ok)
            return ok

        def _request_mode(self, mode: str, why: str) -> None:
            """The FSM's modes (LOITER, RTL, LAND) through the adapter, which
            flies LOITER as BRAKE."""
            self.asked.append({"t": self._t(), "mode": mode, "via": "adapter", "why": why})
            if mode == "LOITER":
                self.asked.append({"t": self._t(), "mode": "BRAKE", "via": "adapter",
                                   "why": "the adapter's hold mode for LOITER"})
            self.pub_mode.publish(String(data=mode))
            self.rec.event("mode_request", t=self._t(), mode=mode, via="adapter", why=why)

        def _wp_index(self) -> int | None:
            """The plan index of the next waypoint (the item the autopilot
            flies to; before AUTO, the first waypoint)."""
            if self.current_seq is None:
                return 0
            i = max(self.current_seq, FIRST_WAYPOINT_SEQ) - FIRST_WAYPOINT_SEQ
            return i if 0 <= i < len(self.plan) else None

        def _send_update(self, idx: int, wp: dict, kind: str, why: str,
                         verdict: dict | None) -> dict:
            seq = FIRST_WAYPOINT_SEQ + idx
            uid = f"u{len(self.updates) + 1}"
            # ArduCopter applies a written item at once and restarts it when
            # it is the item being flown ("Auto mission changed, restarted
            # command"): nothing else is sent for the new waypoint to fly.
            doc = {"seq": seq, "waypoint": wp, "reason": f"{kind}: {why}",
                   "generation": self.policy.generation,
                   "policy_hash": self.policy.policy_hash, "update_id": uid}
            u = {"update_id": uid, "t": self._t(), "kind": kind, "seq": seq, "index": idx,
                 "waypoint": wp, "planned": list(self.plan[idx]), "why": why,
                 "generation": self.policy.generation,
                 "policy_hash": self.policy.policy_hash,
                 "verdict": None if verdict is None else
                 {k: v for k, v in verdict.items() if k != "last_legal"},
                 "_sent_mono": time.monotonic()}
            self.updates.append(u)
            self.pub_update.publish(String(data=json.dumps(doc)))
            # u carries its own mission time as "t".
            self.rec.event("mission_update_sent",
                           **{k: v for k, v in u.items() if not k.startswith("_")})
            self.autopilot_wps[idx] = wp
            self.get_logger().info(f"{kind} item {seq}: ({wp['x']:.1f}, {wp['y']:.1f}) "
                                   f"hold {wp['hold_s']:g} s - {why}")
            return u

        # ---------------- bring-up ---------------- #

        def bring_up(self) -> None:
            log = self.get_logger()
            if self.args.hotapply_port:
                from guardrail.api import FlightHotApply
                self.hot = FlightHotApply(self.shield, self.rec, self._lock,
                                          mission_t=self._t, after=self._after_hot,
                                          log_fn=log.info)
                th = self.hot.start(self.args.hotapply_host, int(self.args.hotapply_port))
                log.info(f"hot-apply API at {th.url}")
            t_end = time.time() + 180.0
            while time.time() < t_end and not self.connected:
                rclpy.spin_once(self, timeout_sec=0.5)
            if not self.connected:
                raise RuntimeError("mavros never connected to FCU")
            # The autopilot's own name and the link, as the GUIDED Shield
            # node reads them after connect: the manifest's topology is
            # judged on them.
            self.read_autopilot_version()
            log.info(f"autopilot: {self.hil_evidence.get('ardupilot_version')} "
                     f"({self.hil_evidence.get('ardupilot_version_source')})")
            self.read_link()
            self._call(self.cli_rate, StreamRate.Request(stream_id=0, message_rate=10,
                                                         on_off=True))
            # Pre-execution: the plan against the policy, the speed against
            # its cap, before anything reaches the autopilot.
            caps = self.shield.kinematic_caps()
            speed = float(self.mission.speed_pref_mps)
            cap = None if caps is None else caps.speed_max_mps
            speed_flown = min(speed, cap) if cap else speed
            wps, legs = check_plan(self.shield, (0.0, 0.0, self.alt), self.plan,
                                   terrain=self.terrain)
            self.autopilot_wps = [dict(w) for w in wps]
            for leg in legs:
                if leg.get("legal") is False:
                    self.replaced[leg["index"]] = {"t": None, "pre_execution": True,
                                                   "waypoint": wps[leg["index"]]}
            self.pre = {"plan": [list(p) for p in self.plan], "legs": legs,
                        "speed_requested_mps": speed, "speed_cap_mps": cap,
                        "speed_flown_mps": speed_flown,
                        "speed_repaired": cap is not None and speed > cap,
                        "waypoints_uploaded": wps,
                        "generation": self.policy.generation,
                        "policy_hash": self.policy.policy_hash}
            self.rec.event("mission_pre_execution", **self.pre)
            doc = {"waypoints": wps, "speed_mps": speed_flown,
                   "generation": self.policy.generation,
                   "policy_hash": self.policy.policy_hash,
                   "reason": "pre-execution checked plan"}
            self.uploaded = doc
            self.pub_mission.publish(String(data=json.dumps(doc)))
            log.info(f"plan checked ({sum(1 for g in legs if g.get('legal') is False)} "
                     f"leg(s) repaired, speed {speed:g} -> {speed_flown:g} m/s); published")
            while time.time() < t_end:
                if self._set_mode_direct("GUIDED", "bring-up"):
                    break
                time.sleep(1)
            while time.time() < t_end:
                r = self._call(self.cli_arm, CommandBool.Request(value=True))
                if r and r.success:
                    log.info("armed")
                    break
                time.sleep(2)
                rclpy.spin_once(self, timeout_sec=0.1)
            else:
                raise RuntimeError("arming refused within the bring-up budget")
            while time.time() < t_end:
                r = self._call(self.cli_tol, CommandTOL.Request(altitude=self.alt))
                if r and r.success:
                    # The policy at take-off (metrics.json), under the lock
                    # the REST hot-apply thread swaps the rules under.
                    with self._lock:
                        self.takeoff_policy = {"policy_hash": self.policy.policy_hash,
                                               "generation": self.policy.generation}
                    break
                time.sleep(2)
            else:
                raise RuntimeError("takeoff never accepted")
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.state and self.state.up >= self.alt - 1.0:
                    break
            else:
                raise RuntimeError("never reached take-off altitude")
            # The adapter's upload and read-back, before AUTO. ArduPilot sets
            # its home at arming, and the adapter converts the plan about
            # home: the plan published on the pad is pushed once home is
            # known (a 2026-11-03 flight that waited for it before arming
            # waited for ever: mission_wp_20261103).
            up_end = time.time() + 60.0
            while time.time() < up_end:
                rclpy.spin_once(self, timeout_sec=0.5)
                ups = [a for a in self.adapter if a.get("kind") == "mission_upload"]
                if ups:
                    break
            ups = [a for a in self.adapter if a.get("kind") == "mission_upload"]
            if not ups or not ups[-1].get("success") or not ups[-1].get("match"):
                raise RuntimeError(f"the mission was not uploaded and read back: "
                                   f"{ups[-1] if ups else 'no status from the adapter'}")
            log.info(f"mission uploaded: {ups[-1].get('items')} items, read back "
                     f"{ups[-1].get('read_back')}, match {ups[-1].get('match')}")
            # Read here, not in _finish(): _finish runs from the tick timer,
            # and a service call from a timer callback is a nested spin that
            # never completes (the GUIDED Shield node reads it at take-off
            # altitude for the same reason).
            self.sim_speedup = self.read_sim_speedup()
            log.info(f"SIM_SPEEDUP read from autopilot: {self.sim_speedup}")

        def start(self) -> None:
            self.mission_t0 = time.monotonic()
            self.rec.event("mission_start", t=0.0,
                           target=[self.mission.target_x, self.mission.target_y],
                           waypoints=[list(p) for p in self.plan],
                           speed_pref_mps=self.mission.speed_pref_mps, style="mission",
                           rule_clock_reading=self.clock().isoformat(timespec="seconds"))
            if not self._set_mode_direct("AUTO", "mission start: the autopilot flies the "
                                                 "uploaded mission"):
                self.get_logger().error("AUTO refused")
            self.create_timer(TICK_S, self._tick)

        def _after_hot(self, result: dict, t: float | None) -> None:
            self.get_logger().info(f"generation {self.policy.generation} after "
                                   f"{result.get('op')} {result.get('type')}")

        # ---------------- the tick ---------------- #

        def _tick(self) -> None:
            with self._lock:
                self._tick_locked()

        def _tick_locked(self) -> None:
            if self.done or self.state is None:
                return
            now = self._t()
            st = self.state
            self.tick_no += 1
            if now > self.args.max_s:
                self._finish("time cap")
                return
            t0 = time.perf_counter()
            # Over a --ground grid the Shield is told the height above the
            # DEM under the aircraft (X-11); `st` itself stays above home,
            # the reference the waypoints and the legs are in.
            told = st if self.terrain is None else self.terrain.state(st)
            unsafe = self.shield.hard_violations(self.shield.state_is_unsafe(told))
            unsafe_rules = sorted({v.rule_id for v in unsafe})
            idx = self._wp_index()
            verdict, repair = None, None
            flying = self.takeover is None and self.mode in ("AUTO", "BRAKE")
            if unsafe and flying and not self.held_unsafe:
                self.held_unsafe = True
                self._request_mode("LOITER", f"position illegal: {', '.join(unsafe_rules)}")
            elif not unsafe and self.held_unsafe and self.takeover is None:
                self.held_unsafe = False
                self._set_mode_direct("AUTO", "position legal again")
            if idx is not None and flying and not self.held_unsafe:
                planned = self.plan[idx]
                pos = (st.x, st.y, st.up)
                verdict = (leg_check(self.shield, pos, planned) if self.terrain is None
                           else leg_check(self.shield, pos, planned, terrain=self.terrain))
                active = self.replaced.get(idx)
                if not verdict["legal"]:
                    self.legal_since.pop(idx, None)
                    p = stop_short(pos, verdict)
                    wp = {"x": round(p[0], 3), "y": round(p[1], 3), "up": round(p[2], 3),
                          "hold_s": HOLD_S}
                    cur = active["waypoint"] if active else None
                    if cur is None or math.hypot(cur["x"] - wp["x"],
                                                 cur["y"] - wp["y"]) > REPLAN_M:
                        why = (f"leg to ({planned[0]:g}, {planned[1]:g}) breaks "
                               f"{', '.join(verdict['rules'])} at {verdict['at_m']:g} m")
                        self._send_update(idx, wp, "replace", why, verdict)
                        self.replaced[idx] = {"t": now, "waypoint": wp}
                else:
                    if active is not None:
                        since = self.legal_since.setdefault(idx, now)
                        if now - since >= T_RECOVER_S:
                            wp = {"x": planned[0], "y": planned[1], "up": planned[2],
                                  "hold_s": 0.0}
                            self._send_update(idx, wp, "restore",
                                              f"leg legal for {now - since:.1f} s >= "
                                              f"{T_RECOVER_S:g} s", verdict)
                            self.replaced.pop(idx, None)
                            self.legal_since.pop(idx, None)
                act = self.replaced.get(idx)
                if act is not None:
                    d = math.hypot(act["waypoint"]["x"] - planned[0],
                                   act["waypoint"]["y"] - planned[1])
                    repair = [{"operator": "waypoint_replace", "magnitude_m": round(d, 3),
                               "rule_id": ",".join(verdict.get("rules", []))
                               if verdict and not verdict["legal"] else None}]
            ms = (time.perf_counter() - t0) * 1e3
            tgt = self.autopilot_wps[idx] if idx is not None and idx < len(self.autopilot_wps) \
                else None
            row = {"t": round(now, 3), "tick": self.tick_no,
                   "x": round(st.x, 3), "y": round(st.y, 3), "up": round(st.up, 3),
                   "mode": self.mode, "mission_seq": self.current_seq,
                   "next_wp_index": idx,
                   "next_wp_planned": None if idx is None else list(self.plan[idx]),
                   "next_wp_autopilot": None if tgt is None else [tgt["x"], tgt["y"], tgt["up"]],
                   "wp_replaced": idx in self.replaced if idx is not None else False,
                   "leg_legal": None if verdict is None else verdict["legal"],
                   "violations": ([{"rule_id": r} for r in verdict["rules"]]
                                  if verdict is not None and not verdict["legal"] else []),
                   "repairs": repair or [],
                   "unsafe": bool(unsafe), "unsafe_rules": unsafe_rules,
                   "raw": {}, "emitted": {}, "flown": False, "setpoint": "mission",
                   "autopilot_set_mode": self.mode,
                   "generation": self.policy.generation,
                   "policy_hash": self.policy.policy_hash,
                   "episode_id": self.rec.episode_id, "shield_ms": round(ms, 3)}
            if self.terrain is not None:
                # As the GUIDED node writes them (X-11): `up` is what the
                # Shield judged (above the DEM), beside the pose's height
                # above home and its MSL height. Absent without a grid.
                row.update(up=round(told.up, 3), up_home=round(st.up, 3),
                           msl_m=round(self.terrain.msl(told.up, st.x, st.y), 3),
                           up_ref="terrain")
            self._log_fh.write(json.dumps(row) + "\n")
            self._log_fh.flush()
            # The end: the last waypoint reached, or the autopilot took it.
            last = len(self.plan) - 1
            if idx == last and self.takeover is None and last not in self.replaced:
                fx, fy, _ = self.plan[last]
                if math.hypot(fx - st.x, fy - st.y) < REACH_M:
                    self._finish("last waypoint reached")
                    return
            if self.takeover is not None and now - (self.takeover["t"] or 0.0) > 30.0:
                self._finish(f"autopilot took the aircraft ({self.takeover['mode']})")

        # ---------------- the end ---------------- #

        def _finish(self, why: str) -> None:
            if self.done:
                return
            self.done = True
            self.end_why = why
            now = self._t()
            land = self.takeover is None
            if land:
                self._request_mode("LAND", f"mission end: {why}")
            self.rec.end(why, t=now, mission_end_land=land,
                         land_skipped_because=None if land else "autopilot has the aircraft")
            if self.hot is not None:
                self.hot.close(f"episode ended: {why}")
            self._log_fh.close()
            reached = why == "last waypoint reached"
            ups = [a for a in self.adapter if a.get("kind") == "mission_upload"]
            upd = [{k: v for k, v in u.items() if not k.startswith("_")} for u in self.updates]
            mission = {"style": "mission", "plan": self.pre.get("plan"),
                       "pre_execution": self.pre, "uploads": [
                           {k: v for k, v in a.items() if not k.startswith("_")} for a in ups],
                       "updates": upd, "reached_items": self.reached_seqs,
                       "first_waypoint_seq": FIRST_WAYPOINT_SEQ}
            (self.out / "mission.json").write_text(json.dumps(mission, indent=1, default=str)
                                                   + "\n", encoding="utf-8")
            ok_updates = [u for u in upd if (u.get("adapter") or {}).get("success")
                          and (u.get("adapter") or {}).get("match")]
            metrics = {
                "rail": "ros2_mavros_mission", "style": "mission",
                # The episode the log's rows belong to: `python -m
                # guardrail.replay pack` refuses metrics without it.
                "episode_id": self.rec.episode_id,
                "mavlink_message": "MISSION_ITEM_INT (through /mavros/mission/push)",
                "outcome": why, "reached_last_waypoint": reached,
                "mission_s": round(now or 0.0, 3), "ticks": self.tick_no,
                "policy_id": self.policy.policy_id,
                # The hash in force when take-off was accepted (None: the run
                # never took off) and at the end; the manifest carries the
                # final one (the last generation, which the replay pack binds
                # to). Until 2026-11-06 the "take-off" hash was the loaded one.
                "policy_hash_at_takeoff": (self.takeoff_policy or {}).get("policy_hash"),
                "generation_at_takeoff": (self.takeoff_policy or {}).get("generation"),
                "policy_hash_final": self.policy.policy_hash,
                "generation_final": self.policy.generation, "policy_source": self.policy_source,
                "seed": self.args.seed,
                "rule_clock": self.clock_record,
                "waypoints_planned": len(self.plan),
                "pre_execution_repairs": sum(1 for g in self.pre.get("legs", [])
                                             if g.get("legal") is False),
                "speed": {k: self.pre.get(k) for k in ("speed_requested_mps", "speed_cap_mps",
                                                       "speed_flown_mps", "speed_repaired")},
                "mission_uploads": len(ups),
                "mission_upload_ok": bool(ups and ups[0].get("success") and ups[0].get("match")),
                "in_flight_updates": len(upd),
                "in_flight_updates_confirmed": len(ok_updates),
                "replacements": sum(1 for u in upd if u["kind"] == "replace"),
                "restores": sum(1 for u in upd if u["kind"] == "restore"),
                "takeover": self.takeover, "mode_requests": self.asked,
                "hot_apply": None if self.hot is None else {
                    "events": self.hot.applied, "endpoint": self.hot.endpoint},
                "position_unsafe_ticks": None,
                # Beside the manifest, which is exactly the grant's six fields
                # (the host's own evidence is part of it since 2026-11-06; it
                # was metrics' `host` before).
                "hil_evidence": self.hil_evidence,
            }
            if self.ground is not None:
                # The ground the Shield converted MSL rules over, named by
                # its file and hash, as the GUIDED node records it (X-11).
                metrics["ground"] = (self.terrain.record() if self.terrain is not None
                                     else {"ground": self.ground.record(),
                                           "up_ref": "home"})
                metrics["ground"]["file"] = dict(self.ground_file)
            with open(self.out / "flight_log.jsonl", encoding="utf-8") as fh:
                rows = [json.loads(line) for line in fh if line.strip()]
            metrics["position_unsafe_ticks"] = sum(1 for r in rows if r.get("unsafe"))
            metrics["shield_ms_max"] = max((r["shield_ms"] for r in rows), default=None)
            # manifest.json beside metrics.json, and the topology in metrics.
            manifest = write_run_records(
                self.out, metrics, policy_hash=self.policy.policy_hash, seed=self.args.seed,
                sim_speedup=self.sim_speedup, evidence=self.hil_evidence,
                warn=self.get_logger().warn)
            self.get_logger().info(f"mission end: {why}; {len(upd)} in-flight update(s), "
                                   f"{len(ok_updates)} confirmed; manifest: topology "
                                   f"{manifest['topology']}, sim_speedup "
                                   f"{manifest['sim_speedup']}, code {manifest['code_revision']}")


def make_parser() -> argparse.ArgumentParser:
    ap = XYArgumentParser(description="the Safety Shield for mission-style flight")
    ap.add_argument("--tag", default="ros2_mission")
    ap.add_argument("--shield", choices=["on"], default="on",
                    help="always on: a mission-style flight without the Shield is "
                         "the autopilot's own mission")
    ap.add_argument("--policy", default=None)
    ap.add_argument("--bundle", default=None)
    ap.add_argument("--dev-yaml", dest="dev_yaml", action="store_true")
    ap.add_argument("--allow-unverified-bundle", action="store_true")
    ap.add_argument("--expected-hash", default=None)
    ap.add_argument("--waypoints", default=None, metavar="X,Y;X,Y",
                    help="the pre-planned mission (local metres, x North, y East); "
                         "default: the mission target")
    ap.add_argument("--target", default=None, metavar="X,Y")
    ap.add_argument("--command", default=None)
    ap.add_argument("--speed", type=float, default=None)
    ap.add_argument("--seed", type=int, default=0,
                    help="recorded in the manifest (random_seed), as the GUIDED "
                         "Shield node's; nothing in the mission style draws a "
                         "random number")
    ap.add_argument("--max-s", type=float, default=MAX_S)
    ap.add_argument("--clock-start", default=None)
    ap.add_argument("--clock-tz", default=None)
    ap.add_argument("--hotapply-port", type=int, default=None)
    ap.add_argument("--hotapply-host", default="127.0.0.1")
    ap.add_argument("--ground", default=None, metavar="FILE",
                    help="the declared ground (guardrail/terrain.py load_ground, "
                         "repo-relative; X-11), as the GUIDED Shield node takes it: an "
                         "MSL rule is flyable over it, and over a grid every leg step "
                         "is judged at its height above the grid. Default: none")
    ap.add_argument("--ground-sha256", default=None, metavar="HEX",
                    help="the sha256 of the --ground file; it must hash to it")
    ap.add_argument("--ground-home-msl", type=float, default=None, metavar="M",
                    help="with a --ground grid: the autopilot's home elevation, m MSL "
                         "(checked against the grid at home). Default: the grid's own")
    return ap


def main(argv: list[str] | None = None) -> None:
    args, ros_args = make_parser().parse_known_args(argv)
    try:
        parse_waypoints(args.waypoints)
        if args.ground or args.ground_sha256 or args.ground_home_msl is not None:
            # Read before ROS starts, as the GUIDED node does: a bad file, a
            # file that is not the one named, or a home off the grid is
            # refused with no node built.
            from ros2_shield_node import resolve_policy
            b, y, _ = resolve_policy(args)
            mission_ground(args, b, y)
    except (ValueError, OSError) as e:
        raise SystemExit(f"[mission_shield] REFUSED: {e}")
    if not HAVE_ROS:
        raise SystemExit("ros2_mission_node needs rclpy and mavros_msgs: source "
                         "/opt/ros/jazzy/setup.bash and use ~/venv-ros/bin/python")
    out = ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros_args)
    node = MissionShieldNode(args, out)
    try:
        node.bring_up()
        node.start()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < END_GRACE_S:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    except Exception as e:                                   # noqa: BLE001
        node.get_logger().error(f"{type(e).__name__}: {e}")
        node.rec.event("mission_refused", error=f"{type(e).__name__}: {e}")
        if not node.done:
            node._finish(f"refused: {type(e).__name__}: {e}")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
