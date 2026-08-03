"""
SEMANTIC mission demo — the case a classical path planner cannot do at all.

Every other demo in this repo is a COORDINATE mission ("fly to (40,40)"), and on
those an A* planner is strictly better than a neural pilot — which is exactly why
the VLA only contributes ~10% of the motion there. This script removes the
coordinates entirely:

    * NO target coordinates anywhere in this file
    * NO global planner, NO path following, NO mission-direction assist
    * the ONLY thing steering the drone is the VLA reading a CAMERA and a
      natural-language INSTRUCTION

...and the Guardrail still enforces every rule on every action. That is the
point of the grant's "Semantic-Spatial Translation and Safety-Constrained VLA":
the pilot may be semantic and unpredictable, the safety envelope is not.

What this demonstrates (and what it does not):
    DOES  - a language-only mission is flyable, and the guardrail holds
            (no-fly zones, altitude band, speed caps, building clearance)
    DOES  - the KPI (P0 violation escape rate = 0) does not depend on a plan
    NOT   - it does not claim the VLA navigates well; base/fine-tuned AerialVLA
            reaches a specific place only ~20% of the time unaided. The honest
            claim is about the SAFETY layer, not the pilot's competence.

Run (server on a Project AirSim world):
    python demo/semantic_demo.py --instruction "fly toward the tall tower and keep clear of the buildings"
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import AuditLogger, Shield, State, load_policy       # noqa: E402
from guardrail.geometry import fence_polygon                        # noqa: E402
from guardrail.models import (                                      # noqa: E402
    Action4D, ObstacleClearance, PolygonFence,
)
from shapely.geometry import Point                                  # noqa: E402

from aerialvla_demo import AerialVLABackend, RateLimiter            # noqa: E402
import city_planner                                                 # noqa: E402

TICK = 0.1
SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_guardrail.jsonc"
CRUISE_M = 45.0


def quat_yaw(q: dict) -> float:
    w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class SemanticObs:
    """Camera + pose feed for the VLA (its own sim connection: thread safety)."""

    def __init__(self):
        import threading
        self.lock = threading.Lock()
        self.front = None
        self.down = None
        self.depth = None
        self.pose = (0.0, 0.0, 0.0)

    def _img(self, msg):
        import cv2
        from PIL import Image
        if not msg or "data" not in msg or not len(msg["data"]):
            return None
        buf = np.frombuffer(msg["data"], dtype=np.uint8)
        arr = (cv2.imdecode(buf, cv2.IMREAD_COLOR) if msg.get("encoding") == "PNG"
               else buf.reshape(msg["height"], msg["width"], 3))
        if arr is None:
            return None
        return Image.fromarray(cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)).resize(
            (224, 224), resample=Image.BICUBIC)

    def put_front(self, msg):
        im = self._img(msg)
        if im is not None:
            with self.lock:
                self.front = im

    def put_down(self, msg):
        im = self._img(msg)
        if im is not None:
            with self.lock:
                self.down = im

    def put_depth(self, msg):
        try:
            data = msg["data"]
            raw = (np.array(data, dtype="B") if isinstance(data, list)
                   else np.frombuffer(data, dtype=np.uint8))
            h, w = msg["height"], msg["width"]
            if msg.get("encoding") == "16UC1" or raw.size == h * w * 2:
                arr = raw.view(np.uint16).reshape(h, w).astype(np.float32)
            else:
                arr = raw.view(np.float32).reshape(h, w).astype(np.float32)
        except Exception:
            return
        with self.lock:
            self.depth = arr

    def put_pose(self, x, y, yaw):
        with self.lock:
            self.pose = (x, y, yaw)

    def get_obs(self):
        with self.lock:
            if self.front is None or self.down is None:
                return None
            return self.front, self.down, self.pose

    def get_depth(self):
        with self.lock:
            return self.depth


async def fly(args) -> int:
    from projectairsim import Drone, ProjectAirSimClient, World

    out = ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)

    policy = load_policy(args.policy)
    cmap = city_planner.load_occ(args.citymap)
    shield_map = None
    if policy.by_type(ObstacleClearance) and cmap is not None:
        shield_map = {"occ": cmap["occ"], "res": cmap["res"],
                      "ox": cmap["ox"], "oy": cmap["oy"]}
    shield = Shield(policy, lookahead_s=3.0, dt=0.5, obstacle_map=shield_map)
    audit = AuditLogger(out / "audit.jsonl", policy.policy_hash)

    print(f"[policy]    {policy.policy_id} {policy.policy_hash}")
    print(f"[semantic]  instruction = {args.instruction!r}")
    print("[semantic]  NO target coordinates, NO planner, NO path following — "
          "the VLA steers from camera + language only")

    obs = SemanticObs()
    # target_xy is only used for the VLA's own direction-hint text; in semantic
    # mode we give it nothing to aim at, so the hint stays empty and the model
    # must decide from the image + instruction alone.
    vla = AerialVLABackend(args.instruction, (0.0, 0.0),
                           obs_factory=lambda: obs.get_obs, lora_id=args.adapter)

    traj, n_touched = [], 0
    client = ProjectAirSimClient()
    client.connect()
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        drone = Drone(client, world, "Drone1")
        client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                         lambda _, m: obs.put_front(m))
        client.subscribe(drone.sensors["DownCamera"]["scene_camera"],
                         lambda _, m: obs.put_down(m))
        try:
            client.subscribe(drone.sensors["FrontCamera"]["depth_camera"],
                             lambda _, m: obs.put_depth(m))
        except Exception as e:
            print(f"[warn] no depth stream ({type(e).__name__}) — reactive layer off")

        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        for _ in range(200):
            kin = drone.get_ground_truth_kinematics()
            if -kin["pose"]["position"]["z"] >= CRUISE_M - 0.5:
                break
            await drone.move_by_velocity_async(0.0, 0.0, -2.0, duration=0.2)
            await asyncio.sleep(0.1)
        vla.start()
        print("[flight] cruise reached — VLA has control (guardrail ON)")

        limiter = RateLimiter(args.dv_h, args.dv_z)
        t0, tick = time.time(), 0
        while time.time() - t0 < args.max_s:
            tick += 1
            kin = drone.get_ground_truth_kinematics()
            p = kin["pose"]["position"]
            yaw = quat_yaw(kin["pose"]["orientation"])
            state = State(x=p["x"], y=p["y"], up=-p["z"])
            obs.put_pose(p["x"], p["y"], yaw)

            # ---- the ONLY steering input: the VLA's reading of the scene ----
            fwd, dwn, yr, _land = vla.latest()
            raw = Action4D(vx=fwd * math.cos(yaw), vy=fwd * math.sin(yaw),
                           vz_up=-dwn, yaw_rate=yr * args.yaw_gain)

            d = shield.filter(state, limiter(raw))
            audit.log(tick, d)
            if d.touched:
                n_touched += 1
            traj.append({"x": state.x, "y": state.y, "up": state.up,
                         "touched": d.touched})
            e = d.emitted
            await drone.move_by_velocity_async(
                e.vx, e.vy, -e.vz_up, duration=0.3,
                yaw_is_rate=True, yaw=e.yaw_rate)
            if tick % 50 == 0:
                print(f"  tick {tick}: pos=({state.x:6.1f},{state.y:6.1f},"
                      f"{state.up:4.1f}) vla_fwd={fwd:.2f} "
                      f"shield={'HIT' if d.touched else '-'}")
            await asyncio.sleep(TICK)

        vla.stop()
        for _ in range(600):
            kin = drone.get_ground_truth_kinematics()
            up = -kin["pose"]["position"]["z"]
            if up <= 1.2:
                break
            await drone.move_by_velocity_async(
                0.0, 0.0, max(0.6, min(2.0, up * 0.15)), duration=0.3)
            await asyncio.sleep(0.1)
        await (await drone.land_async())
        drone.disarm()
        drone.disable_api_control()
    except Exception as e:
        print(f"[warn] flight aborted early: {type(e).__name__}: {e}")
    finally:
        client.disconnect()

    # ---- report: the KPI must hold even with nothing planning the route ----
    fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
    inside = sum(1 for pt in traj for f, poly in fences
                 if f.altitude_floor_m <= pt["up"] <= f.altitude_ceiling_m
                 and poly.contains(Point(pt["x"], pt["y"])))
    nfz_s = inside * TICK
    dist = sum(math.hypot(traj[i]["x"] - traj[i - 1]["x"],
                          traj[i]["y"] - traj[i - 1]["y"])
               for i in range(1, len(traj)))
    band = [c for c in policy.constraints if c.type == "altitude_envelope"]
    alt_ok = all(band[0].alt_min_m - 0.5 <= pt["up"] <= band[0].alt_max_m + 0.5
                 for pt in traj) if band and traj else True
    min_bld = None
    if cmap is not None and traj:
        df = city_planner.distance_field(cmap["occ"], cmap["res"])
        ds = []
        for pt in traj:
            i = int(round((pt["x"] - cmap["ox"]) / cmap["res"]))
            j = int(round((pt["y"] - cmap["oy"]) / cmap["res"]))
            if 0 <= i < df.shape[0] and 0 <= j < df.shape[1]:
                ds.append(float(df[i, j]))
        min_bld = min(ds) if ds else None

    (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7.5, 7))
        if cmap is not None:
            occ, res, ox, oy = cmap["occ"], cmap["res"], cmap["ox"], cmap["oy"]
            ys, xs = [], []
            for i in range(occ.shape[0]):
                for j in range(occ.shape[1]):
                    if occ[i, j]:
                        xs.append(ox + i * res); ys.append(oy + j * res)
            ax.scatter(ys, xs, s=6, c="#c8ccd0", marker="s", label="buildings")
        for f, poly in fences:
            px, py = poly.exterior.xy
            ax.fill(py, px, alpha=0.25, color="red", label=f"NFZ {f.id}")
        ax.plot([p["y"] for p in traj], [p["x"] for p in traj], "-",
                color="tab:blue", linewidth=2, label="flight path (VLA only)")
        tx = [p["y"] for p in traj if p["touched"]]
        ty = [p["x"] for p in traj if p["touched"]]
        if tx:
            ax.plot(tx, ty, ".", color="orange", markersize=5, label="shield active")
        if traj:
            ax.plot(traj[0]["y"], traj[0]["x"], "go", markersize=10, label="start")
        ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
        ax.set_title(f"SEMANTIC mission — no coordinates, no planner\n"
                     f"{args.instruction!r}\n"
                     f"NFZ {nfz_s:.1f}s {'PASS' if nfz_s == 0 else 'FAIL'} | "
                     f"interventions {n_touched} | travelled {dist:.0f} m")
        ax.legend(loc="upper left", fontsize=8)
        ax.set_aspect("equal"); ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / "trajectory.png", dpi=130)
        print(f"[plot] {out / 'trajectory.png'}")
    except ImportError:
        pass

    print(f"\n[report] SEMANTIC mission | ticks {len(traj)} | travelled {dist:.0f} m | "
          f"shield interventions {n_touched} | NFZ {nfz_s:.1f}s -> "
          f"{'PASS' if nfz_s == 0 else 'FAIL'} | altitude band "
          f"{'held' if alt_ok else 'BROKEN'}"
          + (f" | min building distance {min_bld:.1f} m" if min_bld is not None else ""))
    print("[note] The pilot had NO coordinates and NO plan — only the camera and "
          "the sentence. The guardrail is what kept the flight legal.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--instruction", required=True,
                    help="natural-language mission, e.g. 'fly toward the tall tower'")
    ap.add_argument("--policy", default=str(ROOT / "policies" / "urban_clearance.yaml"))
    ap.add_argument("--citymap",
                    default=str(ROOT / "demo" / "out" / "citymap" / "occ_day.npz"))
    ap.add_argument("--adapter", default="D:/models/aerialvla-ft/run2/epoch1")
    ap.add_argument("--tag", default="semantic")
    ap.add_argument("--max-s", type=float, default=90.0)
    ap.add_argument("--yaw-gain", type=float, default=0.4)
    ap.add_argument("--dv-h", type=float, default=0.3)
    ap.add_argument("--dv-z", type=float, default=0.15)
    return asyncio.run(fly(ap.parse_args()))


if __name__ == "__main__":
    sys.exit(main())
