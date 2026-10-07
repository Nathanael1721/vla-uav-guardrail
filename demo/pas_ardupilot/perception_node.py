"""One process: Project AirSim camera -> detector -> pilot -> Shield -> ArduPilot.

The perception-rail integration the grant's final delivery names: the camera
image, the language-grounded detector, the 4-D action, the Safety Shield and
the autopilot in ONE loop, with ArduPilot - not Project AirSim's simple_flight -
flying the aircraft, so Mission Planner sees the flight through mavlink-router.

PER 10 Hz TICK

  1. state     ArduPilot's EKF (GLOBAL_POSITION_INT, or MAVROS's global
               position), projected into the scene frame (autopilot.FrameAnchor).
  2. frame     FrontCamera from Project AirSim, or a ROS 2 image topic on the
               real drone, buffered by demo/semantic_demo.SemanticObs.
  3. detector  demo/follow_vlm.Grounder - OWL-ViT in its own thread, with the
               colour prior and the continuity gate - unchanged.
  4. pilot     demo/follow_vlm.servo(): box -> yaw rate and forward speed, with
               the same altitude hold; follow_vlm.search_sweep_rate() while the
               box is stale. The 4-D action leaves the pilot in the BODY frame.
  5. Shield    body -> scene frame once (autopilot.body_to_world, which is
               guardrail.frames.from_body), then Shield.filter() with the
               grant's 5 s / 10 Hz lookahead and no FSM of its own
               (guardrail.replay.rail_shield). Stand-off rules get the
               subject's position from depth range (or apparent width) and
               bearing - never ground truth.
  6. FSM       with the Shield on, guardrail.fsm.EscalationFSM (the grant's
               Brake -> Loiter -> RTL -> Land) steps on the Shield's decision,
               as sitl/run_sitl_demo.py and sitl/ros2_shield_node.py do. Its
               mode requests go to ArduPilot without blocking the loop
               (link.request_mode), and its setpoint choice (pass / brake /
               none) decides what is sent.
  7. command   ONLY the Shield's emitted action (or the FSM's brake) goes to
               ArduPilot: SET_POSITION_TARGET_LOCAL_NED (pymavlink) or
               PositionTarget on /mavros/setpoint_raw/local (MAVROS 2).

WHAT THE AUTOPILOT DOES ON ITS OWN IS RECORDED. Every row carries the
autopilot's mode and arm state; metrics.json carries its mode changes, arm
changes, STATUSTEXT, FENCE_STATUS breaches and the FSM's mode requests. If
ArduPilot leaves GUIDED without this node asking (its fence fired RTL, a
failsafe, a pilot in Mission Planner), the node stops commanding at once,
records the takeover, and ends the episode once the aircraft has landed or
after TAKEOVER_GRACE_S - it never keeps streaming setpoints the autopilot is
ignoring.

ROWS have the fields the KPIs need (guardrail.replay.episode_row_fields:
unsafe, unsafe_rules, policy_hash, generation, shield_ms, episode_id), what
was flown and the Shield's check on it (guardrail.replay.flown_fields: a tick
nothing was sent on cannot escape), and the episode directory is written
through guardrail.replay.EpisodeRecord, so time to safe is measurable, the
replay bundle carries every policy generation and binds its files by the
episode id. audit.jsonl carries this node's FSM verdict (audit_tick), the
same one the flight log does. These are the contracts both SITL rails follow.

WHAT THIS IS NOT. follow_vlm.fly()'s further layers - the target estimator,
identity tiers, re-acquisition, trail following, the fence guard, the landing
site - live inside that 2,500-line function and are not reused here; moving
them needs a refactor of follow_vlm.py. Until then this node tracks with the
servo law on the detector's box, its runs carry no tracking score (no subject
ground truth is read), and its numbers are not comparable with the
simple_flight CityLife flights.

TOPOLOGY. topology.classify() decides the label the evidence supports. The
grant label is OFFERED to build_manifest only when the rail is qualified: gate
G1 passed (--g1-verdict) and ArduPilot ran at real time (its own clock against
the wall clock, within RT_FACTOR_BAND). Otherwise the rail's own label is
stamped and the reason recorded. With --link pymavlink (what runs on Windows
today) the run is "projectairsim-ardupilot-pymavlink" whatever else holds.

SCENE MODES. --scene-mode load (the default, on the desktop) loads the scene
and then, with --start-sitl, starts ArduPilot - in that order, because a
reload resets the simulator clock under a running ArduPilot. --scene-mode
attach (the hil node on the Jetson Orin) never loads: the desktop has already
run `-Step scene` and `-Step sitl`, and the node only subscribes. --scene-only
loads the scene and exits, which is what `-Step scene` runs.

RUN
    # check everything that can be checked without a simulator
    python demo/pas_ardupilot/perception_node.py --dry-run --network nat \
        --windows-ip 172.x.x.1 --wsl-ip 172.x.x.y
    # fly (the launcher fills in the addresses): scripts/run_pas_ardupilot.ps1 -Step node
"""
from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import math
import random
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for _p in (ROOT, ROOT / "demo", HERE):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
# sitl/ LAST, so nothing there shadows a module above: only for the GeoFence
# rule both SITL rails share (mavlink_adapter_node.fence_alt_plan).
if str(ROOT / "sitl") not in sys.path:
    sys.path.append(str(ROOT / "sitl"))

from guardrail import AuditLogger, State                             # noqa: E402
from guardrail.bundle import load_for_flight                          # noqa: E402
from guardrail.fsm import (FAILSAFE_STATES, EscalationFSM, FSMConfig,  # noqa: E402
                           tick_input_from_decision)
from guardrail.geometry import fence_polygon                          # noqa: E402
from guardrail.kpi import compute_from_dir                            # noqa: E402
from guardrail.manifest import (NO_VLA, TOPOLOGY_DEV, TOPOLOGY_HIL,   # noqa: E402
                                build_manifest, is_kpi_grade)
from guardrail.models import (Action4D, AltitudeEnvelope,             # noqa: E402
                              ObstacleClearance, PolygonFence)
from guardrail.replay import (EpisodeRecord, audit_tick,               # noqa: E402
                              episode_row_fields, flown_fields,
                              kpi_evidence_grade, rail_shield,
                              verify_replay, write_replay)
from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S, Repair       # noqa: E402
from mavlink_adapter_node import fence_alt_plan, policy_ceiling_m      # noqa: E402

import rail                                                           # noqa: E402
import topology as topo                                               # noqa: E402
from autopilot import FrameAnchor, PymavlinkLink, body_to_world       # noqa: E402

TICK = 0.1
# Architecture constraints: "Camera / IMU / GPS - 10 Hz observation". Every
# run reports its detector rate against this, beside the zero-skill null.
GRANT_OBS_HZ = 10.0
DEFAULT_POLICY = ROOT / "policies" / "follow_car_citylife.yaml"
DEFAULT_CITYMAP = ROOT / "demo" / "out" / "citymap_citylife" / "occ_day.npz"
SUBJECT_UP_M = 0.75          # centre height of a car or a person, as follow_vlm
# The FSM's theta needs a per-operator magnitude (Repair.magnitude_m). Until
# shield.py reports one, the velocity proxy with this horizon is used, as on
# the two SITL rails, and the basis is written into metrics.json.
PROXY_HORIZON_S = 0.1
# After the FSM entered RTL / Land, how long to keep recording.
FAILSAFE_GRACE_S = 40.0
# After the autopilot took the aircraft (fence RTL, failsafe, a pilot in
# Mission Planner), how long to keep recording if it has not landed.
TAKEOVER_GRACE_S = 25.0
# home_reached for the FSM's RTL edge: within this of the arming point.
REACH_M = 2.0
LANDED_STATE_ON_GROUND = 1
# ArduPilot's clock against the wall clock. The grant makes sim_speedup 1.0
# mandatory for a KPI-bearing run; the scene DECLARES 1.0 (step-ns equals
# real-time-update-rate), and this is what the run MEASURED. Outside the band
# the run is not KPI-grade and the grant label is not offered.
RT_FACTOR_BAND = (0.95, 1.05)


def theta_horizon() -> float | None:
    """None (per-operator magnitude) once Repair carries magnitude_m, else
    the velocity proxy's horizon - the rule sitl/ros2_shield_node.py uses."""
    return None if "magnitude_m" in getattr(Repair, "model_fields", {}) else PROXY_HORIZON_S


def _fv():
    """demo/follow_vlm.py, imported on first use (1.3 s; no GPU, no sim)."""
    return importlib.import_module("follow_vlm")


# --------------------------------------------------------------------------- #
# Pilot
# --------------------------------------------------------------------------- #

class FollowPilot:
    """follow_vlm's servo law, unchanged, on the Grounder's latest box.

    `detections` is anything with follow_vlm.Grounder's latest() shape:
    {"det": (cx, cy, w, h, score, W, H, colour) | None, "t_det", "seq",
     "n_seen", "n_miss"}.
    """

    def __init__(self, detections, hfov_deg: float, yaw_gain: float = 1.2,
                 want_w_frac: float = 0.10, speed_max: float = 4.0,
                 cruise_alt: float = 8.0, alt_gain: float = 0.6,
                 climb_max: float = 1.8, stale_s: float = 1.0,
                 sweep_deg: float = 25.0, sweep_period_s: float = 4.0,
                 servo=None, sweep=None):
        fv = None if (servo and sweep) else _fv()
        self.servo = servo or fv.servo
        self.sweep = sweep or fv.search_sweep_rate
        self.detections = detections
        self.hfov = hfov_deg
        self.yaw_gain, self.want_w_frac, self.speed_max = yaw_gain, want_w_frac, speed_max
        self.cruise_alt, self.alt_gain, self.climb_max = cruise_alt, alt_gain, climb_max
        self.stale_s = stale_s
        self.sweep_deg, self.sweep_period_s = sweep_deg, sweep_period_s
        self._t_lost = None
        self.n_track = self.n_search = 0

    def step(self, now: float, up: float) -> tuple[tuple, dict]:
        lat = self.detections.latest()
        det, t_det = lat.get("det"), lat.get("t_det") or 0.0
        fresh = det is not None and t_det > 0 and (now - t_det) <= self.stale_s
        vz = float(np.clip((self.cruise_alt - up) * self.alt_gain,
                           -self.climb_max, self.climb_max))
        if fresh:
            yaw_rate, fwd, bearing = self.servo(det, int(det[5]), self.yaw_gain,
                                                self.want_w_frac, self.speed_max,
                                                self.hfov)
            self._t_lost = None
            mode = "track"
            self.n_track += 1
        else:
            if self._t_lost is None:
                self._t_lost = now
            yaw_rate = self.sweep(now - self._t_lost, self.sweep_deg, self.sweep_period_s)
            fwd, bearing, mode = 0.0, None, "search"
            self.n_search += 1
        return ((float(fwd), 0.0, vz, float(yaw_rate)),
                {"mode": mode, "fresh": bool(fresh), "seq": lat.get("seq"),
                 "bearing": None if bearing is None else float(bearing),
                 "det": det})


def subject_estimate(det, depth, state: State, yaw_rad: float, hfov_deg: float,
                     width_m: float, fv=None):
    """(x, y, source) of the detected subject in the scene frame, or None.

    Range from the depth image inside the box (follow_vlm.range_from_depth),
    else from apparent width and the subject's real width; bearing from the box
    column. Slant range becomes horizontal range with the height difference.
    No ground truth enters: this is what lets the Shield's stand-off rules bind
    on the real drone too.
    """
    if det is None:
        return None
    fv = fv or _fv()
    b = fv.box_bearing(float(det[0]), float(det[5]), hfov_deg)
    r, src = (fv.range_from_depth(depth, det), "depth") if depth is not None else (None, None)
    if r is None:
        r, src = fv.implied_range_from_width(det, width_m, hfov_deg), "width"
    if r is None or not math.isfinite(r) or r <= 0:
        return None
    dh = max(0.0, state.up - SUBJECT_UP_M)
    horiz = math.sqrt(max(0.0, r * r - dh * dh))
    ang = yaw_rad + b
    return (state.x + horiz * math.cos(ang), state.y + horiz * math.sin(ang), src)


# --------------------------------------------------------------------------- #
# Camera sources
# --------------------------------------------------------------------------- #

def inline_scene(sim_cfg: dict) -> dict:
    """The scene dict with its robot config inlined, the shape
    projectairsim.utils.load_scene_config_as_dict returns and Drone() reads."""
    d = Path(sim_cfg["dir"])
    scene = rail.load_jsonc(d / sim_cfg["scene"])
    for a in scene.get("actors", []):
        if a.get("type") == "robot" and isinstance(a.get("robot-config"), str):
            a["robot-config"] = rail.load_jsonc(d / a["robot-config"])
    return scene


class ProjectAirSimCamera:
    """Loads (or attaches to) the scene, subscribes FrontCamera (scene + depth)
    and the collision topic, and answers ground-truth queries for the RECORD
    only - the Shield and the pilot never read them.

    attach=False  load the rendered scene (World(client, scene, ...)). Desktop
                  only, and only BEFORE ArduPilot starts.
    attach=True   never load. The World is built without its constructor:
                  World.__init__ with no scene still sends an
                  ImportNEDTrajectory request to the default scene id
                  (/Sim/SceneBasicDrone), which does not exist while CityLife
                  is loaded. The node then checks the simulator really serves
                  this scene's robot topics and refuses if it does not, so a
                  node on the Orin cannot attach to the wrong scene in silence.
    """

    def __init__(self, obs, sim_cfg: dict, address: str = "127.0.0.1",
                 projectairsim=None, view_switch: int = 1,
                 robot: str = rail.ROBOT_NAME, attach: bool = False):
        self.obs, self.cfg, self.address = obs, sim_cfg, address
        self.pas = projectairsim
        self.view_switch, self.robot, self.attach = view_switch, robot, attach
        self.client = self.world = self.drone = None
        self.collisions: list[dict] = []
        self.collisions_subscribed = False
        self.have_depth = False
        self.truth_source = None
        self.scene_record: dict = {"mode": "attach" if attach else "load",
                                   "scene_id": sim_cfg.get("scene_id")}

    def _attach_world(self, pas):
        data = inline_scene(self.cfg)
        world = pas.World.__new__(pas.World)
        world.client = self.client
        world.sim_config_path = self.cfg["dir"]
        world.sim_instance_idx = -1
        world.sim_config = data
        world.parent_topic = f"/Sim/{data['id']}"
        world.home_geo_point = data.get("home-geo-point", {})
        get_info = getattr(self.client, "get_topic_info", None)
        if callable(get_info):
            get_info()
        topics = getattr(self.client, "topics", None)
        prefix = f"{world.parent_topic}/robots/{self.robot}"
        live = None if topics is None else [t for t in topics if str(t).startswith(prefix)]
        self.scene_record.update(parent_topic=world.parent_topic,
                                 robot_topics_live=None if live is None else len(live))
        if live is not None and not live:
            raise RuntimeError(
                f"attach: the simulator at {self.address} serves no topic under "
                f"{prefix}, so scene {data['id']!r} is not the one loaded there. "
                f"Load it on the desktop first (run_pas_ardupilot.ps1 -Step scene), "
                f"start ArduPilot after it (-Step sitl), then start this node")
        return world

    def start(self) -> None:
        pas = self.pas or importlib.import_module("projectairsim")
        self.client = pas.ProjectAirSimClient(address=self.address)
        self.client.connect()
        if self.attach:
            self.world = self._attach_world(pas)
        else:
            self.world = pas.World(self.client, self.cfg["scene"], delay_after_load_sec=2,
                                   sim_config_path=self.cfg["dir"])
            self.scene_record["loaded_wall"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            # The first streaming camera in this robot is the FrontCamera depth
            # stream; one switch puts the Chase view in the window (follow_vlm.py
            # explains the blank-white window this avoids).
            for _ in range(max(0, self.view_switch)):
                try:
                    self.world.switch_streaming_view()
                except Exception as exc:                      # noqa: BLE001
                    print(f"[view] could not switch the main view: {exc}")
                    break
        self.drone = pas.Drone(self.client, self.world, self.robot)
        t0 = time.time()

        def _on_collision(_, m):
            if isinstance(m, dict) and m.get("has_collided", True):
                self.collisions.append({"t": round(time.time() - t0, 2),
                                        "object": m.get("object_name")})
        try:
            self.client.subscribe(self.drone.robot_info["collision_info"], _on_collision)
            self.collisions_subscribed = True
        except Exception as exc:                              # noqa: BLE001
            print(f"[collide] *** not subscribed ({exc}); contacts will NOT be counted")
        self.client.subscribe(self.drone.sensors["FrontCamera"]["scene_camera"],
                              lambda _, m: self.obs.put_front(m))
        try:
            self.client.subscribe(self.drone.sensors["FrontCamera"]["depth_camera"],
                                  lambda _, m: self.obs.put_depth(m))
            self.have_depth = True
        except Exception as exc:                              # noqa: BLE001
            print(f"[range] no depth stream ({exc}); range from apparent width")

    def truth_geo(self) -> dict | None:
        """{lat, lon, alt} of the simulated vehicle, or None."""
        try:
            g = self.drone.get_ground_truth_geo_location()
            self.truth_source = "GetGroundTruthGeoLocation"
            return {"lat": float(g["latitude"]), "lon": float(g["longitude"]),
                    "alt": float(g["altitude"])}
        except Exception:                                     # noqa: BLE001
            return None

    def stop(self) -> None:
        try:
            if self.client is not None:
                self.client.disconnect()
        except Exception:                                     # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #

def load_obstacle_map(policy, citymap: Path):
    """The CityLife occupancy map for the policy's altitude band, as follow_vlm
    loads it; None (ObstacleClearance INERT, said out loud) when unavailable."""
    if not policy.by_type(ObstacleClearance):
        return None, "policy has no obstacle_clearance rule"
    if not Path(citymap).is_file():
        return None, f"no map at {citymap}: obstacle_clearance is INERT this flight"
    try:
        import city_planner
        import occ_bands
    except Exception as exc:                                  # noqa: BLE001
        return None, f"map loaders unavailable ({exc}): obstacle_clearance is INERT"
    cmap = city_planner.load_occ(str(citymap))
    alt = policy.by_type(AltitudeEnvelope)
    if alt:
        sel = occ_bands.select_for_band(Path(citymap).parent, alt[0].alt_min_m,
                                        alt[0].alt_max_m)
        if sel["map"] is None:
            return None, "no map covers the policy's altitude band: obstacle_clearance is INERT"
        cmap = sel["map"]
    return ({"occ": cmap["occ"], "res": cmap["res"], "ox": cmap["ox"], "oy": cmap["oy"]},
            f"loaded {citymap}")


G1_GATE_NAME = "G1 (fallback-gates.md)"


def g1_status(path: str | None) -> dict:
    """The rail's G1 qualification, read from a verdict g1_check.summarize
    wrote. Anything else - no file, an unreadable one, a JSON that is not a
    G1 verdict - is None (unknown), never a pass."""
    if not path:
        return {"passed": None, "verdict": None,
                "why": "no G1 verdict was given (--g1-verdict)"}
    try:
        v = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return {"passed": None, "verdict": str(path), "error": str(exc)}
    if not isinstance(v, dict) or v.get("gate") != G1_GATE_NAME \
            or not isinstance(v.get("g1_passed"), bool):
        return {"passed": None, "verdict": str(path),
                "error": "not a G1 verdict written by g1_check.summarize"}
    return {"passed": v["g1_passed"], "status": v.get("status"), "verdict": str(path),
            "tag": v.get("tag"), "expected_runs": v.get("expected_runs")}


def rt_factor_ok(rt: float | None) -> bool:
    return rt is not None and RT_FACTOR_BAND[0] <= rt <= RT_FACTOR_BAND[1]


def autopilot_fence_record(link, policy, cruise_alt_m: float) -> dict | None:
    """The autopilot's GeoFence as it holds it, judged by the rule both SITL
    rails apply (sitl/mavlink_adapter_node.fence_alt_plan): its altitude
    fence must sit above every altitude the policy allows, or the backstop
    fires on a flight the Shield considers legal. Recorded only - this node
    never changes an autopilot parameter. None when the link cannot read
    parameters; a value the autopilot did not answer stays None, so an
    unread fence is never reported as off or as fine."""
    if not hasattr(link, "autopilot_fence"):
        return None
    vals = link.autopilot_fence()
    ceiling, ceiling_src = policy_ceiling_m(policy, cruise_alt_m)
    alt_max, enabled = vals.get("FENCE_ALT_MAX"), vals.get("FENCE_ENABLE")
    plan = fence_alt_plan(ceiling, alt_max)
    return {**vals, "policy_ceiling_m": ceiling, "policy_ceiling_source": ceiling_src,
            "backstop_active": None if enabled is None else enabled >= 1.0,
            "stricter_than_policy": (None if alt_max is None or ceiling is None
                                     else not plan["ok"]),
            "why": plan["why"], "changed_by_node": False}


def module_available(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def percentile(vals, q: float):
    return None if not vals else round(float(np.percentile(np.asarray(vals), q)), 3)


# --------------------------------------------------------------------------- #
# The node
# --------------------------------------------------------------------------- #

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--object", default="a red car")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--out-root", default=str(ROOT / "demo" / "out"))
    ap.add_argument("--policy", default=None)
    ap.add_argument("--bundle", default=None)
    ap.add_argument("--allow-unverified-bundle", action="store_true")
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--fsm-config", default=None,
                    help="escalation FSM thresholds YAML (guardrail.fsm.FSMConfig)")
    ap.add_argument("--no-fsm", action="store_true",
                    help="do not run the escalation FSM (not for KPI runs)")
    ap.add_argument("--seed", type=int, default=20261006)
    ap.add_argument("--max-s", type=float, default=120.0)
    # where things are
    ap.add_argument("--camera", choices=["projectairsim", "ros2"], default="projectairsim")
    ap.add_argument("--scene-mode", choices=["load", "attach"], default="load",
                    help="load the scene (desktop), or attach to the one the "
                         "desktop loaded (the hil node on the Orin)")
    ap.add_argument("--scene-only", action="store_true",
                    help="load the scene and exit (run_pas_ardupilot.ps1 -Step scene)")
    ap.add_argument("--image-topic", default="/camera/image_raw")
    ap.add_argument("--sim-host", default="127.0.0.1",
                    help="Project AirSim server address (the desktop's LAN IP "
                         "when this node runs on the Jetson Orin)")
    ap.add_argument("--network", choices=["nat", "mirrored"], default="nat")
    ap.add_argument("--windows-ip", default=None)
    ap.add_argument("--wsl-ip", default=None)
    ap.add_argument("--link", choices=["pymavlink", "mavros"], default="pymavlink")
    ap.add_argument("--mavlink-url", default=f"tcp:127.0.0.1:{rail.ROUTER_TCP_PORT}")
    ap.add_argument("--start-sitl", action="store_true",
                    help="after the scene loads, start ArduPilot SITL + the router "
                         "(+ MAVROS 2 with --link mavros) in WSL "
                         "(start_ardupilot.sh), and stop them at the end")
    ap.add_argument("--wsl-distro", default="Ubuntu")
    ap.add_argument("--g1-verdict", default=None)
    ap.add_argument("--citymap", default=str(DEFAULT_CITYMAP))
    ap.add_argument("--hfov", type=float, default=None)
    # pilot (follow_vlm's defaults; cruise 8 m as the CityLife launcher flies)
    ap.add_argument("--cruise-alt", type=float, default=8.0)
    ap.add_argument("--yaw-gain", type=float, default=1.2)
    ap.add_argument("--want-width", type=float, default=0.10)
    ap.add_argument("--speed-max", type=float, default=4.0)
    ap.add_argument("--alt-gain", type=float, default=0.6)
    ap.add_argument("--climb-max", type=float, default=1.8)
    ap.add_argument("--det-thresh", type=float, default=0.02)
    ap.add_argument("--colour-min", type=float, default=0.10)
    ap.add_argument("--dry-run", action="store_true",
                    help="render configs, print the plan and the commands, exit")
    return ap


def check_args(args) -> None:
    """Combinations that cannot work, refused before anything starts."""
    if args.camera == "ros2" and args.link != "mavros":
        raise SystemExit("--camera ros2 needs --link mavros (one ROS 2 node)")
    if args.scene_mode == "attach" and args.start_sitl:
        raise SystemExit("--scene-mode attach means the desktop already loaded the "
                         "scene and started ArduPilot after it (-Step scene, then "
                         "-Step sitl); --start-sitl here would start a second one")
    if args.scene_only and (args.camera != "projectairsim" or args.scene_mode != "load"):
        raise SystemExit("--scene-only loads the Project AirSim scene; it needs "
                         "--camera projectairsim --scene-mode load")


def sim_configs(args, out: Path):
    """(sim_cfg, plan) for this run: rendered with today's addresses when
    this node loads the scene, the template marked "attach" when it does not."""
    if args.camera != "projectairsim":
        return None, None
    if args.scene_mode == "attach":
        return rail.write_attach_configs(out / "pas_config"), None
    plan = rail.plan_network(args.network, args.windows_ip, args.wsl_ip)
    return rail.write_sim_configs(out / "pas_config", plan), plan


def dry_run(args, out: Path) -> dict:
    """Everything checkable without a simulator, an autopilot or a GPU."""
    report = {"ok": True, "problems": [], "scene_mode": args.scene_mode}
    cfg, plan = sim_configs(args, out)
    if cfg is not None:
        scene = rail.load_jsonc(Path(cfg["dir"]) / cfg["scene"])
        report["sim_config"] = cfg
        if plan is not None:
            report["sitl_args"] = rail.sitl_args(scene, plan,
                                                 [rail.to_wsl_path(rail.PARAM_FILE)])
            report["sim_vehicle_equivalent"] = rail.sim_vehicle_equivalent(
                scene, plan, rail.to_wsl_path(rail.PARAM_FILE))
            report["router_conf"] = rail.router_conf(plan.gcs_address, plan.windows_ip)
            report["wsl_start"] = rail.wsl_start_command(
                args.wsl_distro, plan, mavros=(args.link == "mavros"))
    deps = {"pymavlink": module_available("pymavlink"),
            "projectairsim": module_available("projectairsim"),
            "torch": module_available("torch"),
            "transformers": module_available("transformers"),
            "rclpy": module_available("rclpy")}
    report["dependencies"] = deps
    need = ["torch", "transformers"]
    need += ["projectairsim"] if args.camera == "projectairsim" else ["rclpy"]
    need += ["pymavlink"] if args.link == "pymavlink" else ["rclpy"]
    for n in sorted(set(need)):
        if not deps[n]:
            report["ok"] = False
            report["problems"].append(f"{n} is not importable in {sys.executable}")
    return report


class SceneLoader:
    """Loads the rendered scene and nothing else (no camera subscriptions):
    what `-Step scene` needs before ArduPilot is started."""

    def __init__(self, sim_cfg: dict, address: str = "127.0.0.1", projectairsim=None):
        self.cfg, self.address, self.pas = sim_cfg, address, projectairsim
        self.client = None

    def start(self) -> None:
        pas = self.pas or importlib.import_module("projectairsim")
        self.client = pas.ProjectAirSimClient(address=self.address)
        self.client.connect()
        world = pas.World(self.client, self.cfg["scene"], delay_after_load_sec=2,
                          sim_config_path=self.cfg["dir"])
        try:
            world.switch_streaming_view()
        except Exception:                                     # noqa: BLE001
            pass

    def stop(self) -> None:
        try:
            if self.client is not None:
                self.client.disconnect()
        except Exception:                                     # noqa: BLE001
            pass


def load_scene_only(args, out: Path, camera=None) -> int:
    """`-Step scene`: render, load, record, exit. The scene stays loaded in
    the simulator for ArduPilot (-Step sitl) and an attaching node."""
    sim_cfg, plan = sim_configs(args, out)
    camera = camera or SceneLoader(sim_cfg, address=args.sim_host)
    try:
        camera.start()
        rec = {"scene": sim_cfg, "loaded_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "next": "start ArduPilot now (run_pas_ardupilot.ps1 -Step sitl); "
                       "reloading this scene while it runs drives its clock backwards"}
        (out / "scene_load.json").write_text(json.dumps(rec, indent=2), encoding="utf-8")
        print(f"[scene]  {sim_cfg['scene_id']} loaded from {sim_cfg['dir']}; "
              f"start ArduPilot now, never before a reload")
        return 0
    finally:
        camera.stop()


def run(args, *, link=None, camera=None, grounder=None, obs=None,
        clock=time.time, sleep=time.sleep, popen=None, run_cmd=None) -> int:
    """Fly one run. Every collaborator is injectable so the loop can be tested
    without a simulator, an autopilot or a GPU (tests/test_pas_ardupilot.py)."""
    popen = popen or subprocess.Popen
    run_cmd = run_cmd or subprocess.run
    tag = args.tag or time.strftime("pas_ap_%Y%m%d_%H%M%S")
    out = Path(args.out_root) / tag
    out.mkdir(parents=True, exist_ok=True)
    check_args(args)

    if args.dry_run:
        rep = dry_run(args, out)
        print(json.dumps(rep, indent=2))
        return 0 if rep["ok"] else 2
    if args.scene_only:
        return load_scene_only(args, out, camera)

    random.seed(args.seed)
    np.random.seed(args.seed % (2 ** 32))

    policy, policy_source = load_for_flight(
        args.bundle, args.policy or (None if args.bundle else DEFAULT_POLICY),
        allow_unverified=args.allow_unverified_bundle)
    smap, map_note = load_obstacle_map(policy, Path(args.citymap))
    print(f"[policy] {policy.policy_id} {policy.policy_hash} from "
          f"{policy_source['kind']} - signature {policy_source['signature']}")
    print(f"[map]    {map_note}")
    # The grant's lookahead (guardrail.shield: 5 s at 10 Hz, 50 future poses),
    # as on both SITL rails, and NO escalation FSM inside the Shield: this
    # node runs THE FSM below, fed with what only it sees (the autopilot's
    # mode, home, landed, a takeover). With the Shield's own FSM on as well,
    # audit.jsonl would record one state machine and flight_log.jsonl another
    # (guardrail.replay, "ONE ESCALATION FSM PER RAIL").
    shield = rail_shield(policy, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S,
                         obstacle_map=smap)
    # The episode record before the audit log: it moves an earlier run's files
    # aside and writes generation 0. A follow mission has no compiler target,
    # so its CSP is recorded as not compiled (an event), not skipped.
    rec = EpisodeRecord(out, policy, None, lookahead_s=LOOKAHEAD_S)
    audit = AuditLogger(out / "audit.jsonl", policy)
    shield_on = args.shield == "on"
    fsm = (EscalationFSM(FSMConfig.from_yaml(args.fsm_config) if args.fsm_config
                         else FSMConfig())
           if shield_on and not args.no_fsm else None)
    horizon = theta_horizon()

    # ---- simulator configs and camera -------------------------------------
    sim_cfg, plan = sim_configs(args, out)
    scene, scene_path = None, None
    if sim_cfg is not None:
        scene_path = Path(sim_cfg["dir"]) / sim_cfg["scene"]
        scene = rail.load_jsonc(scene_path)
        if plan is not None:
            print(f"[sim]    configs rendered to {sim_cfg['dir']} "
                  f"({plan.mode}: sim binds {plan.sim_bind_ip}:{rail.SIM_SERVO_PORT}, "
                  f"sends to {plan.ardupilot_ip}:{rail.SITL_SENSOR_PORT})")
        else:
            print(f"[sim]    attaching to scene {sim_cfg['scene_id']} on {args.sim_host}")
    if obs is None:
        from semantic_demo import SemanticObs
        obs = SemanticObs()
    sitl_proc = sitl_log = None
    if camera is None and args.camera == "projectairsim":
        camera = ProjectAirSimCamera(obs, sim_cfg, address=args.sim_host,
                                     attach=args.scene_mode == "attach")
    try:
        if camera is not None:
            camera.start()                     # loads the scene: BEFORE SITL starts
        if args.start_sitl and plan is not None:
            cmd = rail.wsl_start_command(args.wsl_distro, plan,
                                         mavros=(args.link == "mavros"))
            print("[sitl]   " + " ".join(cmd))
            sitl_log = open(out / "sitl_wsl.log", "w", encoding="utf-8")
            sitl_proc = popen(cmd, stdout=sitl_log, stderr=subprocess.STDOUT)
            rec.event("sitl_started", command=cmd)

        # ---- autopilot -------------------------------------------------------
        if link is None:
            if args.link == "pymavlink":
                link = PymavlinkLink(args.mavlink_url)
                link.connect(timeout=180.0)
            else:
                from mavros_link import MavrosLink, Ros2ImageSource
                link = MavrosLink()
                if args.camera == "ros2":
                    Ros2ImageSource(link.node, obs, args.image_topic)
        print(f"[link]   {link.name}: bring-up to {args.cruise_alt} m")
        link.bring_up(args.cruise_alt)
        # Read from the autopilot during bring-up (identify()); None if unreadable,
        # which the manifest then records as unresolved - never an assumed 1.0.
        speedup = getattr(link, "sim_speedup", None)
        anchor = FrameAnchor.choose(policy, rail.home_geo_point(scene) if scene else None)
        if anchor is None:
            s = link.snap
            anchor = FrameAnchor(s.lat, s.lon, s.alt_msl, "ekf.first-fix (no policy origin, no scene)")
        print(f"[frame]  scene frame anchored at {anchor.to_dict()}")
        # The autopilot's own GeoFence, the grant's backstop, as the autopilot
        # holds it: recorded, never changed by this node.
        ap_fence = autopilot_fence_record(link, policy, args.cruise_alt)
        if ap_fence is not None:
            rec.event("autopilot_fence", **ap_fence)
            print(f"[fence]  ENABLE {ap_fence['FENCE_ENABLE']}  ALT_MAX "
                  f"{ap_fence['FENCE_ALT_MAX']}  RADIUS {ap_fence['FENCE_RADIUS']}  "
                  f"ACTION {ap_fence['FENCE_ACTION']}")
            if ap_fence["stricter_than_policy"]:
                print(f"[fence]  *** {ap_fence['why']}")
        # The arming point, for the FSM's "home reached" (RTL): the vehicle
        # took off vertically from it.
        link.pump()
        st_home = link.state(anchor)
        home_xy = None if st_home is None else (st_home.x, st_home.y)

        # ---- detector and pilot --------------------------------------------------
        fv = None
        hfov = args.hfov
        if grounder is None or hfov is None:
            fv = _fv()
        if hfov is None:
            hfov = (fv.camera_hfov_deg(config_path=Path(sim_cfg["dir"]) / sim_cfg["robot"])
                    if sim_cfg else fv.CAMERA_HFOV_DEG)
        if grounder is None:
            grounder = fv.Grounder(obs, args.object, thresh=args.det_thresh,
                                   colour_min=args.colour_min,
                                   log_path=out / "detections.jsonl")
            grounder.start()
        width_m, subject_class = (fv or _fv()).subject_width(args.object)
        pilot = FollowPilot(grounder, hfov, yaw_gain=args.yaw_gain,
                            want_w_frac=args.want_width, speed_max=args.speed_max,
                            cruise_alt=args.cruise_alt, alt_gain=args.alt_gain,
                            climb_max=args.climb_max)

        # ---- loop ----------------------------------------------------------------
        rows, traj, ekf_err = [], [], []
        n_touched = n_braked = n_escaped = n_no_state = n_subject = 0
        subject_src: dict = {}
        requested_modes: list[str] = []
        takeover = None
        fsm_fault = None
        failsafe_t = None
        end_reason = "max_s reached"
        prev_mode = link.snap.mode
        fence0 = link.snap.fence_breach_count
        g0 = grounder.latest()
        det_n0 = (g0.get("n_seen") or 0) + (g0.get("n_miss") or 0)
        # The FCU clock is paired with the wall time it ARRIVED at, not with
        # "now": SYSTEM_TIME comes at 2 Hz, and pairing a stale FCU stamp with
        # the current wall time read a perfect real-time run as 0.97x.
        tb0 = (link.snap.time_boot_ms, getattr(link.snap, "t_time_boot", None))
        t0 = clock()
        rec.event("mission_start", t=0.0, autopilot_mode=prev_mode,
                  fsm=None if fsm is None else fsm.config.to_dict())
        t_truth = 0.0
        try:
            while clock() - t0 < args.max_s:
                t_tick = clock()
                link.pump()
                st = link.state(anchor)
                if st is None:
                    n_no_state += 1
                    sleep(TICK)
                    continue
                now = t_tick - t0
                s = link.snap
                mode = s.mode
                if fence0 is None:
                    fence0 = s.fence_breach_count
                # The autopilot left GUIDED without this node asking: its fence
                # fired RTL, a failsafe, or a pilot in Mission Planner. From here
                # it has the aircraft, and nothing more is commanded.
                if (takeover is None and prev_mode == "GUIDED" and mode != "GUIDED"
                        and mode not in requested_modes):
                    takeover = {"t": round(now, 3), "mode": mode, "previous": prev_mode,
                                "fence_breach_count": s.fence_breach_count,
                                "statustext": s.statustext[-3:]}
                    rec.event("autopilot_took_over", t=now, mode=mode, previous=prev_mode)
                    print(f"[link]   *** the autopilot took the aircraft: GUIDED -> "
                          f"{mode}; commanding stops")
                prev_mode = mode
                yaw = math.radians(st.yaw_deg)
                obs.put_pose_full(t_tick, st.x, st.y, st.up, yaw, s.pitch or 0.0, s.roll or 0.0)
                body, info = pilot.step(t_tick, st.up)
                raw = body_to_world(*body, yaw)
                subj = None
                if info["fresh"]:
                    depth = obs.get_depth() if hasattr(obs, "get_depth") else None
                    subj = subject_estimate(info["det"], depth, st, yaw, hfov, width_m, fv)
                if subj is not None and subject_class is not None:
                    shield.set_subject(subj[0], subj[1], subject_class)
                    n_subject += 1
                    subject_src[subj[2]] = subject_src.get(subj[2], 0) + 1
                else:
                    shield.set_subject(None)
                d = shield.filter(st, raw)
                tick = len(rows) + 1
                # The episode id binds every row to this episode's events
                # (guardrail.replay: "ONE EPISODE, NOT TWO").
                extra = episode_row_fields(shield, st, policy, rec.episode_id)
                emitted = d.emitted if shield_on else raw

                # ---- escalation FSM (Shield on only) -------------------------
                fsm_out, setpoint, stop_illegal = None, "pass", None
                if fsm is not None and fsm_fault is None and takeover is None:
                    try:
                        in_failsafe = fsm.state in FAILSAFE_STATES
                        stop_illegal = shield.state_is_unsafe(st)
                        on_ground = (s.armed is False
                                     or s.landed_state == LANDED_STATE_ON_GROUND)
                        fsm_out = fsm.step(tick_input_from_decision(
                            now, d, policy, horizon_s=horizon,
                            home_reached=bool(in_failsafe and mode == "RTL"
                                              and home_xy is not None
                                              and math.hypot(st.x - home_xy[0],
                                                             st.y - home_xy[1]) < REACH_M),
                            landed=bool(in_failsafe and mode in ("RTL", "LAND")
                                        and on_ground),
                            stop_illegal=stop_illegal))
                        setpoint = fsm_out.setpoint
                        if fsm_out.transition:
                            rec.event("fsm_transition", t=now, **{k: fsm_out.record[k] for k in (
                                "fsm_state_before", "fsm_state_after", "transition",
                                "edge", "edge_source", "reason", "fsm_config_hash")})
                        if fsm_out.set_mode:
                            requested_modes.append(fsm_out.set_mode)
                            sent = link.request_mode(fsm_out.set_mode)
                            rec.event("mode_request", t=now, mode=fsm_out.set_mode,
                                      why=fsm_out.reason, sent=bool(sent))
                            print(f"[fsm]    {fsm_out.set_mode} requested: {fsm_out.reason}")
                        if fsm_out.state in FAILSAFE_STATES and failsafe_t is None:
                            failsafe_t = now
                    except ValueError as exc:
                        # A Shield/FSM contract break mid-flight: hold, say why,
                        # stop streaming. The run is not KPI-grade.
                        fsm_fault = str(exc)
                        rec.event("fsm_fault", t=now, error=fsm_fault)
                        requested_modes.append("LOITER")
                        link.request_mode("LOITER")
                        failsafe_t = now
                        print(f"[fsm]    input refused, LOITER: {exc}")
                if fsm_fault is not None or takeover is not None:
                    setpoint = "none"
                # After the FSM stepped, with ITS verdict: one state machine on
                # record, in audit.jsonl and flight_log.jsonl alike.
                audit_tick(audit, tick, d, fsm_out=fsm_out, fsm_fault=fsm_fault)
                flown = {"pass": emitted, "brake": Action4D(), "none": None}[setpoint]
                if flown is not None:
                    link.send(flown, raw_body=body)

                touched = bool(shield_on and d.touched)
                n_touched += touched
                n_braked += bool(shield_on and d.braked)
                # What was FLOWN, and the Shield's check on it, decided by the
                # rule both SITL rails share (guardrail.replay.flown_fields):
                # the re-check of the Shield's output (on) or the raw action's
                # violations (off); standing still in Brake; and NOTHING on a
                # tick nothing was sent (the autopilot or a fault held the
                # aircraft) - no action flew, so none can have escaped.
                ff = flown_fields(shield, st, d, shield_on=shield_on, setpoint=setpoint)
                n_escaped += bool(ff["emitted_violations"])
                traj.append({"t": round(now, 2), "x": st.x, "y": st.y, "up": st.up,
                             "touched": touched})
                row = {
                    "t": round(now, 3), "tick": tick,
                    "x": round(st.x, 3), "y": round(st.y, 3), "up": round(st.up, 3),
                    "yaw_deg": round(st.yaw_deg, 2),
                    "body": [round(v, 4) for v in body],
                    "raw": raw.model_dump(), "emitted": emitted.model_dump(),
                    # The action sent to the autopilot (None: nothing sent);
                    # "flown" (a bool) and "emitted_violations" come from
                    # flown_fields, the shape guardrail/kpi.py reads.
                    "flown_action": None if flown is None else flown.model_dump(),
                    **ff,
                    "setpoint": setpoint,
                    "violations": [v.model_dump() for v in d.violations],
                    "repairs": [r.model_dump() for r in d.repairs] if shield_on else [],
                    "braked": bool(shield_on and d.braked), "touched": touched,
                    "mode": info["mode"], "det_seq": info["seq"],
                    # What the autopilot was doing on this tick, by its own report.
                    "ap_mode": mode, "armed": s.armed,
                    "subject": None if subj is None else [round(subj[0], 2), round(subj[1], 2), subj[2]],
                    **extra,
                }
                if fsm_out is not None:
                    r_ = fsm_out.record
                    row.update({
                        "fsm_state_before": fsm_out.before.value,
                        "fsm_state_after": fsm_out.state.value,
                        "fsm_edge": fsm_out.edge, "set_mode": fsm_out.set_mode,
                        "stop_illegal": bool(stop_illegal),
                        "failsafe": bool(fsm_out.transition
                                         and fsm_out.state in FAILSAFE_STATES),
                        "fsm": {k: r_.get(k) for k in (
                            "outcome", "theta_exceeded", "n_in_t", "reason",
                            "risk_level", "fsm_config_hash")}})
                elif not shield_on:
                    row["fsm_state_after"] = None
                rows.append(row)
                # EKF against the simulator, every 0.5 s: the frame alignment's own
                # check. RECORD ONLY - nothing above read it.
                if camera is not None and hasattr(camera, "truth_geo") and t_tick - t_truth >= 0.5:
                    t_truth = t_tick
                    g = camera.truth_geo()
                    if g is not None and s.lat is not None:
                        tx, ty = anchor.to_scene(g["lat"], g["lon"])
                        ex, ey = anchor.to_scene(s.lat, s.lon)
                        ekf_err.append(math.hypot(tx - ex, ty - ey))
                landed = s.armed is False or s.landed_state == LANDED_STATE_ON_GROUND
                if takeover is not None and (landed or now - takeover["t"] > TAKEOVER_GRACE_S):
                    end_reason = f"autopilot took over ({takeover['mode']})"
                    break
                if fsm is not None and (fsm.terminal or (
                        failsafe_t is not None and now - failsafe_t > FAILSAFE_GRACE_S)):
                    end_reason = f"escalation FSM ended the mission ({fsm.state.value})"
                    break
                sleep(max(0.0, TICK - (clock() - t_tick)))
        except KeyboardInterrupt:
            end_reason = "interrupted"
            print("[node]   interrupted")
        except Exception as exc:
            # The episode still ends on the record, with the real reason - not
            # the "max_s reached" it was initialised with - and the error
            # propagates (no metrics or KPI table is written for it).
            end_reason = f"crashed: {type(exc).__name__}: {exc}"
            raise
        finally:
            # A mission end lands. A fail-safe the FSM asked for, or a mode the
            # autopilot chose itself, is left to the autopilot to finish.
            autopilot_owns = takeover is not None or (
                fsm is not None and (fsm.terminal or fsm.state in FAILSAFE_STATES))
            if not autopilot_owns:
                try:
                    requested_modes.append("LAND")
                    link.land()
                except Exception as exc:                          # noqa: BLE001
                    print(f"[link]   LAND failed: {exc}")
            # EpisodeRecord.end(): the one mission_end write_replay requires
            # before it packs an episode.
            rec.end(end_reason, t=round(clock() - t0, 3))
            if hasattr(grounder, "stop"):
                grounder.stop()

        t_end = clock()
        mission_s = max(1e-6, t_end - t0)
        g1 = grounder.latest()
        det_n = (g1.get("n_seen") or 0) + (g1.get("n_miss") or 0) - det_n0
        tb1 = (link.snap.time_boot_ms, getattr(link.snap, "t_time_boot", None))
        rt_factor = (round((tb1[0] - tb0[0]) / 1000.0 / (tb1[1] - tb0[1]), 3)
                     if None not in (tb0[0], tb0[1], tb1[0], tb1[1])
                     and tb1[1] - tb0[1] >= 1.0 else None)
        fence1 = link.snap.fence_breach_count
        fence_breaches = (None if fence0 is None or fence1 is None else fence1 - fence0)

        # ---- dwell, from the flown trajectory -------------------------------------
        fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
        from shapely.geometry import Point
        nfz_ticks = sum(1 for p in traj for f, poly in fences
                        if f.altitude_floor_m <= p["up"] <= f.altitude_ceiling_m
                        and poly.contains(Point(p["x"], p["y"])))
        env = policy.by_type(AltitudeEnvelope)
        alt_ticks = sum(1 for p in traj for e in env if not (e.alt_min_m <= p["up"] <= e.alt_max_m))

        link_ev = link.evidence()
        ev = {"camera_source": args.camera,
              "controller_type": sim_cfg["controller_type"] if sim_cfg else None,
              "sim_same_desktop": (topo.sim_on_this_desktop(args.sim_host)
                                   if args.camera == "projectairsim" else None),
              "link": link.name,
              "autopilot": {k: link_ev.get(k) for k in ("heartbeat", "is_ardupilot")},
              "topology_evidence": link_ev.get("topology_evidence") or {},
              "g1": g1_status(args.g1_verdict)}
        cls = topo.classify(ev)
        simulated = args.camera == "projectairsim"
        qualified = cls["rail_qualified_by_g1"] is True
        rt_ok = rt_factor_ok(rt_factor)
        withheld = []
        if cls["grant"] in (TOPOLOGY_DEV, TOPOLOGY_HIL):
            if not qualified:
                withheld.append("gate G1 has not passed for this rail "
                                f"({ev['g1'].get('why') or ev['g1'].get('error') or ev['g1'].get('status') or 'no verdict'})")
            if not rt_ok:
                withheld.append(f"ArduPilot ran at {rt_factor}x real time (measured "
                                f"from its own clock); the band is {RT_FACTOR_BAND}")
        offered = cls["grant"] if (cls["grant"] and not withheld) else None
        autopilot_events = [{**e, "t": round(e["t"] - t0, 3)} for e in link.events]

        metrics = {
            "tag": tag, "rail": "projectairsim-ardupilot",
            "episode_id": rec.episode_id,
            "topology": cls["stamp"], "grant_topology": cls["grant"],
            "grant_topology_reasons": cls["reasons"],
            "grant_topology_offered": offered,
            "grant_topology_withheld": withheld,
            "rail_qualified_by_g1": cls["rail_qualified_by_g1"],
            "topology_evidence": ev, "link_evidence": link_ev,
            # The key guardrail.manifest.is_kpi_grade re-checks a hil label on.
            "hil_evidence": ev["topology_evidence"],
            "scene": None if camera is None else getattr(camera, "scene_record", None),
            "shield": args.shield, "ticks": len(rows), "ticks_without_state": n_no_state,
            "ticks_flown": sum(1 for r in rows if r.get("flown")),
            "ticks_not_flown": sum(1 for r in rows if r.get("flown") is False),
            "shield_lookahead": {"horizon_s": LOOKAHEAD_S, "dt_s": LOOKAHEAD_DT_S,
                                 "poses": int(round(LOOKAHEAD_S / LOOKAHEAD_DT_S))},
            "mission_s": round(mission_s, 2), "end_reason": end_reason,
            "loop_hz": round(len(rows) / mission_s, 3),
            # The observation rate against the grant's 10 Hz, beside its null: a
            # detector that never ran scores 0.0 here.
            "det_hz": round(det_n / mission_s, 3), "det_hz_grant_target": GRANT_OBS_HZ,
            "det_hz_null": 0.0,
            "track_ticks": pilot.n_track, "search_ticks": pilot.n_search,
            "subject_known_ticks": n_subject, "subject_source": subject_src,
            "subject_class": subject_class,
            "interventions": n_touched, "brakes": n_braked, "escape_ticks": n_escaped,
            "nfz_s": round(nfz_ticks * TICK, 2), "alt_violation_s": round(alt_ticks * TICK, 2),
            # No subject ground truth is read on this rail, so there is no tracking
            # score and no stand-off dwell to report. None, never 0.
            "frac_within_30m": None, "standoff_s": None, "follow_scored": False,
            "fsm": (dict(fsm.summary(), fault=fsm_fault,
                         theta_basis=({"source": "per-operator magnitude_m"} if horizon is None
                                      else {"source": "velocity proxy |dv| * h",
                                            "horizon_s": horizon}))
                    if fsm is not None else None),
            "mode_requests": requested_modes,
            # What the autopilot did on its own, by its own report: mode and arm
            # changes, STATUSTEXT, fence breaches. Relative to mission start
            # (bring-up events are negative).
            "autopilot_events": autopilot_events,
            "autopilot_took_over": takeover,
            # FENCE_STATUS over the mission; None when it never arrived (the
            # MAVROS link has no such topic), never a silent 0.
            "fence_breach_count": fence_breaches,
            # The GeoFence as the autopilot held it (None: the link reads no
            # parameters), judged against the policy's ceiling.
            "autopilot_fence": ap_fence,
            "generations": rec.generations,
            # Every contact the simulator reported, take-off and landing included
            # (g1_check.airborne_collisions separates those); None = not subscribed.
            "sim_contacts_all": (None if camera is None
                                 or not getattr(camera, "collisions_subscribed", False)
                                 else len(camera.collisions)),
            "ekf_vs_sim_horiz_m": {"n": len(ekf_err), "median": percentile(ekf_err, 50),
                                   "p95": percentile(ekf_err, 95),
                                   "max": None if not ekf_err else round(max(ekf_err), 3)},
            "ardupilot_rt_factor": rt_factor, "rt_factor_band": list(RT_FACTOR_BAND),
            "fcu_clock_source": getattr(link.snap, "fcu_clock_source", None),
            "sim_speedup_param": speedup,
            "frame_anchor": anchor.to_dict(),
            "obstacle_map": map_note,
            "sim_config": sim_cfg,
            "policy_source": policy_source,
            "seed": args.seed,
            "params": {k: getattr(args, k) for k in ("object", "cruise_alt", "yaw_gain",
                                                     "want_width", "speed_max", "alt_gain",
                                                     "climb_max", "det_thresh", "colour_min",
                                                     "seed", "link", "camera", "scene_mode")},
            "detector": "google/owlvit-base-patch32 (demo/follow_vlm.DETECTOR_ID)",
        }
        (out / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                              encoding="utf-8")
        (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")
        (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")

        # The grant label is OFFERED to build_manifest only when the rail is
        # qualified (G1 passed, real time measured); the manifest's guard then
        # judges it on the evidence and the scene. On refusal, or when the
        # label was withheld, the rail's own label is stamped and the reason
        # recorded. Today manifest.py refuses dev/hil beside any Project
        # AirSim scene, so this rail stamps its own label; the guard stays the
        # only judge of the grant label.
        common = dict(policy_hash=policy.policy_hash, model_id=NO_VLA, seed=args.seed,
                      scene_path=scene_path, sim_speedup=None if scene_path else speedup)
        refusal = None
        if offered:
            try:
                manifest = build_manifest(**common, topology=offered,
                                          hil_evidence=ev["topology_evidence"])
            except ValueError as exc:
                refusal = str(exc)
        if offered is None or refusal is not None:
            manifest = build_manifest(**common, topology=cls["stamp"])
        metrics["manifest_refusal"] = refusal
        metrics["topology"] = manifest["topology"]
        (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        kpi = compute_from_dir(out, policy)
        graded, why = is_kpi_grade(manifest, metrics)
        # What is_kpi_grade cannot see from the manifest: the scene DECLARES
        # sim_speedup 1.0, and only this run measured whether ArduPilot kept it;
        # and a simulated rail must be qualified by G1.
        if simulated and not rt_ok:
            why.append(f"ArduPilot's measured real-time factor is {rt_factor} "
                       f"(band {RT_FACTOR_BAND}); the scene's declared 1.0 is not "
                       f"what flew")
        if simulated and not qualified:
            why.append("the Project AirSim + ArduPilot rail is not qualified by gate G1")
        if fsm_fault is not None:
            why.append(f"the escalation FSM refused an input mid-flight: {fsm_fault}")
        graded = graded and not why
        kpi.update(kpi_grade=graded, kpi_grade_reasons=why, manifest=manifest,
                   policy_source=policy_source, episode_id=rec.episode_id)
        kpi_path = out / "kpi.json"
        kpi_path.write_text(json.dumps(kpi, indent=2), encoding="utf-8")
        rb_path = out / f"{tag}.replay.tar.gz"
        try:
            rb = write_replay(out, policy, rb_path,
                              changelog=f"pas-ardupilot {tag}", policy_source=policy_source)
            ok, notes = verify_replay(rb)
            print(f"[replay] {rb.name}: {'re-derives its KPIs' if ok else 'FAILED'}")
            for n_ in notes:
                print(f"[replay]   - {n_}")
            # KPI evidence needs a bundle that verifies with a TRUSTED
            # signature (guardrail.replay.kpi_evidence_grade); a graded run
            # with a keyless or failing bundle is demoted, and kpi.json and
            # the bundle are rewritten to say so - as on both SITL rails.
            graded2, why2 = kpi_evidence_grade(graded, why, rb)
            if graded2 != graded:
                graded, why = graded2, why2
                kpi.update(kpi_grade=graded, kpi_grade_reasons=why)
                kpi_path.write_text(json.dumps(kpi, indent=2), encoding="utf-8")
                write_replay(out, policy, rb_path, changelog=f"pas-ardupilot {tag}",
                             policy_source=policy_source)
        except Exception as exc:                                  # noqa: BLE001
            print(f"[replay] not written: {type(exc).__name__}: {exc}")
            if graded:
                graded, why = False, why + [f"no replay bundle: {exc}"]
                kpi.update(kpi_grade=graded, kpi_grade_reasons=why)
                kpi_path.write_text(json.dumps(kpi, indent=2), encoding="utf-8")
        print(f"[report] ticks {len(rows)} at {metrics['loop_hz']} Hz | detector "
              f"{metrics['det_hz']} Hz (grant {GRANT_OBS_HZ}, null 0.0) | interventions "
              f"{n_touched} | escape ticks {n_escaped} | P0 escape rate "
              f"{kpi['p0_violation_escape_rate']} | ended: {end_reason}")
        print(f"[report] ArduPilot real-time factor {rt_factor} | fence breaches "
              f"{fence_breaches} | mode requests {requested_modes} | autopilot took "
              f"over: {takeover['mode'] if takeover else 'no'}")
        print(f"[report] topology stamped {manifest['topology']!r}; grant label "
              f"{cls['grant']!r}: " + "; ".join(cls["reasons"] + withheld))
        if refusal:
            print(f"[report] build_manifest refused {offered!r}: {refusal}")
        print(f"[report] KPI-grade: {'YES' if graded else 'no'}"
              + ("" if graded else " (" + "; ".join(why) + ")"))
        return 0
    finally:
        # Whatever happened above - a refused arm, an EKF that never
        # converged, Ctrl+C - the simulator connection and the SITL this
        # run started do not outlive it.
        if camera is not None and hasattr(camera, "stop"):
            camera.stop()
        if sitl_proc is not None:
            run_cmd(rail.wsl_stop_command(args.wsl_distro), check=False)
            sitl_proc.wait(timeout=30)
        if sitl_log is not None:
            sitl_log.close()


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
