"""
Follow a named object with vision and language, through the guardrail.

You type "a car". A vision-language model finds that thing in the camera image.
A servo loop keeps it centred and at a chosen distance. The Shield still checks
every action. Nothing in the steering path knows the target's coordinates.

Why this and not AerialVLA
--------------------------
AerialVLA was measured, over 108 controlled forward passes and several flights,
to ignore its object-description slot entirely: correct and wrong colour words
produce indistinguishable actions, and with no coordinate-derived bearing phrase
it emits stop-and-land on 11 of 18 real frames. Flown against a real car mesh it
never moved at all — 12 of 12 inferences returned the same token triple. See
docs/FINDING-what-drives-aerialvla.md.

So the language grounding is done by a model that actually does it. OWL-ViT is an
open-vocabulary detector: text in, boxes out. Measured here at 34-56 ms per frame
against AerialVLA's 11-12 s, which also removes the latency problem that made
closed-loop tracking impossible.

This is still vision-language-action — the words choose the target, the camera
finds it, the controller acts — with the model swapped for one whose language
input reaches the output.

What the controller does
------------------------
    horizontal box offset  -> yaw rate      (turn to centre the target)
    box width vs desired   -> forward speed (hold a standoff distance)
    altitude error         -> climb rate    (hold the cruise height)

The altitude term is deliberate: the earlier scripts had none and leaned on the
Shield, which is a constraint filter making minimal corrections, not a
controller. Flights sagged tens of seconds below the floor as a result.

Run:
    python demo/follow_vlm.py --object "a car" --tag vlmfollow
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import AuditLogger, Shield, State, load_policy       # noqa: E402
from guardrail.geometry import fence_polygon                        # noqa: E402
from guardrail.models import (                                      # noqa: E402
    Action4D, AltitudeEnvelope, ObstacleClearance, PolygonFence,
)
from shapely.geometry import Point                                  # noqa: E402

import city_planner                                                 # noqa: E402
import city_traffic
import moving_car                                                   # noqa: E402
from aerialvla_demo import RateLimiter                              # noqa: E402
from semantic_demo import SemanticObs, quat_yaw                     # noqa: E402

TICK = 0.1
SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_semantic.jsonc"
DETECTOR_ID = "google/owlvit-base-patch32"


# Hue ranges in OpenCV's 0-179 scale, plus how saturated a pixel must be to count
# as that colour at all. Grey road and pale concrete have low saturation, so the
# saturation floor does most of the rejecting.
COLOUR_HUE = {
    "red": [(0, 10), (170, 179)], "orange": [(8, 24)], "yellow": [(22, 35)],
    "green": [(36, 85)], "blue": [(90, 130)], "purple": [(130, 160)],
}
COLOUR_ACHROMATIC = {"white": "white", "black": "black", "grey": "grey", "gray": "grey"}


def colour_word(query: str):
    q = query.lower()
    for w in list(COLOUR_HUE) + list(COLOUR_ACHROMATIC):
        if w in q:
            return w
    return None


def colour_match(img, box, word) -> float:
    """Fraction of pixels inside the box that really are the named colour.

    The detector grounds the noun; this grounds the adjective. Without it the
    colour word does nothing measurable — "a blue car" and "an orange car"
    scored identically against the same orange car — and a confident detection
    on the wrong object is indistinguishable from the right one. A false lock
    cost a whole flight: the aircraft centred a city object at the right apparent
    size and held station on it 60 m from the actual car.
    """
    import cv2
    if word is None:
        return 1.0
    x0, y0, x1, y1 = [int(max(0, v)) for v in box]
    if x1 - x0 < 2 or y1 - y0 < 2:
        return 0.0
    crop = np.asarray(img)[y0:y1, x0:x1]
    if crop.size == 0:
        return 0.0
    hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    if word in COLOUR_ACHROMATIC:
        kind = COLOUR_ACHROMATIC[word]
        if kind == "white":
            m = (s < 60) & (v > 170)
        elif kind == "black":
            m = v < 60
        else:
            m = (s < 60) & (v >= 60) & (v <= 170)
    else:
        m = np.zeros(h.shape, bool)
        for lo, hi in COLOUR_HUE[word]:
            m |= (h >= lo) & (h <= hi)
        m &= (s > 90) & (v > 50)
    return float(m.mean())


class FenceGuard:
    """Lets the controller see the no-fly zones, so it can stop before them.

    Without this the servo commands "go to the car" every tick and the Shield
    refuses it every tick. Neither changes its mind, so the aircraft chatters
    against the boundary — measured at 659 corrections in 795 ticks, and it looks
    exactly as unsafe as it is.

    A real aircraft knows its own geofence; that is mission data, not target
    data. Nothing here reveals where the car is — only where the aircraft may
    not go. It brakes smoothly on approach and holds at a standoff, while yaw
    keeps tracking so the target stays in view.
    """

    def __init__(self, policy, brake_m: float = 12.0, stand_off_m: float = 3.0):
        from guardrail.geometry import fence_polygon
        from guardrail.models import PolygonFence
        self.polys = [fence_polygon(f).buffer(f.margin_m)
                      for f in policy.by_type(PolygonFence)]
        self.brake_m = brake_m
        self.stand_off_m = stand_off_m

    def gate(self, x: float, y: float, vx: float, vy: float):
        """Scale a commanded velocity down as it closes on a fence.

        Returns (scale, distance_to_fence, blocked). `scale` is 1 when clear and
        0 at the stand-off, so the approach is a smooth deceleration rather than
        a wall.
        """
        from shapely.geometry import Point
        if not self.polys:
            return 1.0, None, False
        p = Point(x, y)
        d = min(poly.distance(p) for poly in self.polys)
        speed = math.hypot(vx, vy)
        if speed < 1e-3:
            return 1.0, d, d <= self.stand_off_m
        # only brake for motion that actually closes on the fence
        ahead = Point(x + vx / speed * 2.0, y + vy / speed * 2.0)
        d_ahead = min(poly.distance(ahead) for poly in self.polys)
        if d_ahead >= d:
            return 1.0, d, False
        if d <= self.stand_off_m:
            return 0.0, d, True
        if d >= self.brake_m:
            return 1.0, d, False
        k = (d - self.stand_off_m) / (self.brake_m - self.stand_off_m)
        return float(np.clip(k, 0.0, 1.0)), d, k < 0.35

    def slide(self, x: float, y: float, vx: float, vy: float, probe_m: float = 8.0):
        """Which way to sidestep when the direct line is blocked.

        Braking alone is safe but passive: the aircraft stops at the boundary and
        the target drives away. If the fence does not span the whole corridor
        there is a way past, and this finds it by probing left and right
        perpendicular to the blocked heading and going whichever way the fence
        falls further behind.

        Returns a unit vector, or (0, 0) when neither side is better — in which
        case stopping really is the only legal answer.
        """
        from shapely.geometry import Point
        speed = math.hypot(vx, vy)
        if not self.polys or speed < 1e-3:
            return 0.0, 0.0
        ux, uy = vx / speed, vy / speed
        lx, ly = -uy, ux                      # left of the commanded heading
        best, bx, by = None, 0.0, 0.0
        for sgn in (1.0, -1.0):
            px = x + lx * sgn * probe_m + ux * probe_m * 0.5
            py = y + ly * sgn * probe_m + uy * probe_m * 0.5
            d = min(poly.distance(Point(px, py)) for poly in self.polys)
            inside = any(poly.contains(Point(px, py)) for poly in self.polys)
            score = -1.0 if inside else d
            if best is None or score > best:
                best, bx, by = score, lx * sgn, ly * sgn
        if best is None or best <= self.stand_off_m:
            return 0.0, 0.0
        return bx, by


def annotate(img, det, hud: dict):
    """Draw what the drone is actually seeing and deciding, for the demo.

    A raw camera frame proves nothing to a viewer — the whole claim is that a
    word picked the box and the box drove the aircraft, so both have to be on
    screen at once.
    """
    from PIL import ImageDraw
    im = img.copy()
    d = ImageDraw.Draw(im)
    W, H = im.size
    if det is not None:
        cx, cy, bw, bh = det[0], det[1], det[2], det[3]
        x0, y0, x1, y1 = cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2
        d.rectangle([x0, y0, x1, y1], outline=(0, 255, 0), width=2)
        d.line([(cx, 0), (cx, H)], fill=(0, 255, 0, 90))
        tag = f"{hud['query']}  p={det[4]:.3f}"
        if len(det) > 7:
            tag += f"  colour={det[7]:.2f}"
        d.text((max(2, x0), max(2, y0 - 11)), tag, fill=(0, 255, 0))
    d.line([(W / 2, H / 2 - 8), (W / 2, H / 2 + 8)], fill=(255, 255, 255))
    d.line([(W / 2 - 8, H / 2), (W / 2 + 8, H / 2)], fill=(255, 255, 255))
    lines = [
        f"t {hud['t']:5.1f}s   alt {hud['alt']:4.1f} m",
        f"bearing {hud['brg']:+6.1f}deg   fwd {hud['fwd']:+4.1f} m/s",
        (f"separation {hud['sep']:5.1f} m" if hud.get("sep") is not None
         else "separation n/a"),
        ("NFZ - SKIRTING AROUND" if hud.get("fence_mode") == "skirt"
         else "NFZ AHEAD - HOLDING" if hud.get("fence_hold")
         else (f"no-fly zone {hud['fence_d']:.0f} m ahead"
               if hud.get("fence_d") is not None and hud["fence_d"] < 20
               else "GUARDRAIL: correcting" if hud["shield"] else "GUARDRAIL: clear")),
        {"track": "TARGET LOCKED", "coast": "COASTING on last motion",
         "search": "SEARCHING ...", "scan": "SCANNING for target"}.get(
            hud.get("mode"), "SCANNING for target"),
    ]
    for i, ln in enumerate(lines):
        d.text((6, 6 + 11 * i), ln,
               fill=((255, 90, 90) if ("HOLDING" in ln or "correcting" in ln)
                     else (120, 220, 255) if "SKIRTING" in ln
                     else (255, 200, 90) if "no-fly zone" in ln
                     else (255, 255, 255)))
    return im


class Grounder:
    """Open-vocabulary detector in a background thread: text in, box out.

    Publishes the latest detection; the control loop reads it without blocking.
    Detection scores for a small distant object are low in absolute terms
    (0.03-0.07 measured at 22 m), so the box is accepted on a low threshold and
    filtered by continuity instead: a detection far from the last accepted one is
    rejected unless nothing has been seen for a while. That rejects the
    occasional confident tree without needing a confident car.
    """

    def __init__(self, obs: SemanticObs, query: str, thresh: float = 0.02,
                 jump_frac: float = 0.35, log_path: Path | None = None,
                 colour_min: float = 0.10):
        self.obs = obs
        self.query = query
        self.thresh = thresh
        self.jump_frac = jump_frac
        self.colour = colour_word(query)
        self.colour_min = colour_min
        if self.colour:
            print(f"[grounder] colour prior: {self.colour!r} "
                  f"(a box must be >={colour_min:.0%} that colour to qualify)")
        self.log_path = log_path
        self._lock = threading.Lock()
        self._stop = False
        self._seq = 0
        self._det = None          # (cx, cy, w, h, score, W, H, colour)
        self._t = 0.0             # last time the detector RAN
        self._t_det = 0.0         # last time it actually FOUND the target
        self.drop_after = 8.0     # forget the box entirely after this long
        self._infer_ms = 0.0
        self._n_seen = 0
        self._n_miss = 0
        self._thread = threading.Thread(target=self._worker, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True

    def _worker(self):
        import torch
        from transformers import OwlViTProcessor, OwlViTForObjectDetection
        t0 = time.time()
        proc = OwlViTProcessor.from_pretrained(DETECTOR_ID)
        model = OwlViTForObjectDetection.from_pretrained(DETECTOR_ID).to("cuda").eval()
        print(f"[grounder] {DETECTOR_ID} loaded in {time.time()-t0:.0f}s "
              f"(VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB)", flush=True)
        queries = [[self.query]]
        last = None
        while not self._stop:
            img = self.obs.get_front_native()
            if img is None:
                time.sleep(0.05)
                continue
            W, H = img.size
            t = time.time()
            inputs = proc(text=queries, images=img, return_tensors="pt").to("cuda")
            with torch.no_grad():
                out = model(**inputs)
            torch.cuda.synchronize()
            ms = (time.time() - t) * 1000
            res = proc.post_process_object_detection(
                out, threshold=0.0,
                target_sizes=torch.tensor([[H, W]]).to("cuda"))[0]
            sc, bx = res["scores"], res["boxes"]
            det, best = None, -1.0
            if len(sc):
                # Score every plausible box, do not just take the detector's top
                # one. Ranking by detector score alone is what let a city object
                # win and hold the aircraft 60 m from the car.
                for i in sc.argsort(descending=True)[:12]:
                    s = float(sc[i])
                    if s < self.thresh:
                        break
                    x0, y0, x1, y1 = [float(v) for v in bx[i].tolist()]
                    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                    if last is not None and (time.time() - last[1]) < 1.5:
                        if abs(cx - last[0]) > self.jump_frac * W:
                            continue          # too far from where it just was
                    cm = colour_match(img, (x0, y0, x1, y1), self.colour)
                    if self.colour is not None and cm < self.colour_min:
                        continue              # right shape, wrong colour
                    combined = s * (0.25 + 0.75 * cm)
                    if combined > best:
                        best = combined
                        det = (cx, cy, x1 - x0, y1 - y0, s, W, H, cm)
            with self._lock:
                self._seq += 1
                self._infer_ms = ms
                # `_t` is when the detector last RAN. `_t_det` is when it last
                # actually FOUND something. Conflating the two was a real bug:
                # the control loop aged the detection against `_t`, which
                # refreshes on every frame including misses, so a stale box was
                # treated as fresh forever. The aircraft chased a box that was no
                # longer there and the HUD kept reporting TARGET LOCKED.
                self._t = time.time()
                if det is not None:
                    self._det = det
                    self._t_det = time.time()
                    self._n_seen += 1
                    last = (det[0], time.time())
                else:
                    self._n_miss += 1
                    # drop the box once it is unusably old, so nothing downstream
                    # can accidentally act on it
                    if self._t_det and time.time() - self._t_det > self.drop_after:
                        self._det = None
            if self.log_path is not None:
                rec = {"seq": self._seq, "t": self._t, "infer_ms": round(ms, 1),
                       "query": self.query,
                       "det": (None if det is None else
                               {"cx": round(det[0], 1), "cy": round(det[1], 1),
                                "w": round(det[2], 1), "h": round(det[3], 1),
                                "score": round(det[4], 4), "colour": round(det[7], 3),
                                "img_w": det[5], "img_h": det[6]})}
                with self.log_path.open("a", encoding="utf-8") as fh:
                    fh.write(json.dumps(rec) + "\n")

    def latest(self) -> dict:
        with self._lock:
            return {"seq": self._seq, "t": self._t, "t_det": self._t_det,
                    "det": self._det, "infer_ms": self._infer_ms,
                    "n_seen": self._n_seen, "n_miss": self._n_miss}


def servo(det, img_w: int, yaw_gain: float, want_w_frac: float,
          speed_max: float, hfov_deg: float = 90.0):
    """Box in the image -> (yaw rate rad/s, forward speed m/s, bearing error rad).

    Bearing comes from the horizontal offset scaled by the real horizontal FOV,
    so the yaw command is in true angular units rather than arbitrary pixels.
    Forward speed closes on a target apparent width: too small means too far, so
    accelerate; too large means too close, so back off.
    """
    cx, _cy, bw, _bh, _s, W, _H = det[:7]
    off = (cx - W / 2) / (W / 2)                 # -1 left .. +1 right
    bearing = math.radians(hfov_deg / 2.0) * off
    yaw_rate = yaw_gain * bearing
    w_frac = bw / W
    err = (want_w_frac - w_frac) / max(1e-3, want_w_frac)
    fwd = float(np.clip(err * speed_max, -0.4 * speed_max, speed_max))
    # do not charge forward while the target is far off to one side
    fwd *= max(0.0, math.cos(bearing))
    return yaw_rate, fwd, bearing


async def fly(args) -> int:
    from projectairsim import Drone, ProjectAirSimClient, World

    out = ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)
    for f in ("flight_log.jsonl", "detections.jsonl"):
        if (out / f).exists():
            (out / f).unlink()

    policy = load_policy(args.policy)
    cmap = city_planner.load_occ(args.citymap)
    smap = None
    if policy.by_type(ObstacleClearance) and cmap is not None:
        smap = {"occ": cmap["occ"], "res": cmap["res"],
                "ox": cmap["ox"], "oy": cmap["oy"]}
    shield = Shield(policy, lookahead_s=3.0, dt=0.5, obstacle_map=smap)
    fence = FenceGuard(policy, brake_m=args.fence_brake, stand_off_m=args.fence_standoff)
    audit = AuditLogger(out / "audit.jsonl", policy.policy_hash)
    if fence.polys:
        print(f"[fence] {len(fence.polys)} no-fly zone(s) known to the controller: "
              f"brake from {args.fence_brake:.0f} m, hold at {args.fence_standoff:.0f} m")

    print(f"[policy]  {policy.policy_id} {policy.policy_hash}")
    print(f"[follow]  query = {args.object!r}")
    print("[follow]  the ONLY steering input is where the detector puts the box; "
          "no target coordinates reach the controller")

    obs = SemanticObs()
    rows, traj, n_touched = [], [], 0
    car = None
    traffic = None
    client = ProjectAirSimClient()
    client.connect()
    grounder = None
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        drone = Drone(client, world, "Drone1")
        client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                         lambda _, m: obs.put_front(m))
        client.subscribe(drone.sensors["DownCamera"]["scene_camera"],
                         lambda _, m: obs.put_down(m))
        view_dir = None
        if args.save_view:
            try:
                client.subscribe(drone.sensors["Chase"]["scene_camera"],
                                 lambda _, m: obs.put_chase(m))
                view_dir = out / "view"
                (view_dir / "tps").mkdir(parents=True, exist_ok=True)
                (view_dir / "fpv").mkdir(parents=True, exist_ok=True)
                print(f"[view] third-person + annotated frames -> {view_dir}")
            except Exception as exc:
                print(f"[view] no Chase camera in this config ({type(exc).__name__})")

        if not args.no_car:
            stops = ([(0.30, args.car_stop_s), (0.62, args.car_stop_s)]
                     if args.straight and args.car_stop_s > 0 else None)
            if args.traffic > 0:
                # Several vehicles, same mesh, only the target painted. `car`
                # stays a MovingCar (the fleet's target), so every downstream
                # use — ground-truth separation, the HUD, the metrics — is
                # unchanged and the traffic is purely additive.
                traffic = city_traffic.Traffic(world, fleet=city_traffic.default_fleet(
                    n_background=args.traffic, speed=args.car_speed,
                    target_stops=stops, bg_every=args.traffic_every))
                traffic.spawn()
                car = traffic.target
            else:
                car = moving_car.MovingCar(
                    world, speed_mps=args.car_speed,
                    route=(moving_car.STRAIGHT_ROUTE if args.straight else None),
                    one_shot=args.straight,
                    phase_s=(0.0 if args.straight else 10.0),
                    stops=stops)
                car.spawn()
            # let the actor settle before the first teleport; it is briefly
            # not movable straight after spawning
            for _ in range(10):
                await asyncio.sleep(0.4)
                before = car.pos
                car.update(0.5)
                if car.pos != before or getattr(car, "_fail_streak", 0) == 0:
                    break
            car.update(0.0)

        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        for _ in range(400):
            kin = drone.get_ground_truth_kinematics()
            if -kin["pose"]["position"]["z"] >= args.cruise_alt - 0.5:
                break
            await drone.move_by_velocity_async(0.0, 0.0, -2.0, duration=0.2)
            await asyncio.sleep(0.1)

        # face the car's street before handing over, so the first frames contain
        # something to ground. This is a starting attitude, not steering: no
        # target position is used after this point.
        if car is not None:
            kin = drone.get_ground_truth_kinematics()
            p0 = kin["pose"]["position"]
            psi0 = math.atan2(car.pos[1] - p0["y"], car.pos[0] - p0["x"])
            for _ in range(120):
                kin = drone.get_ground_truth_kinematics()
                if abs((psi0 - quat_yaw(kin["pose"]["orientation"]) + math.pi)
                       % (2 * math.pi) - math.pi) < math.radians(4):
                    break
                await drone.move_by_velocity_async(0.0, 0.0, 0.0, duration=0.3,
                                                   yaw_is_rate=False, yaw=psi0)
                await asyncio.sleep(0.1)

        grounder = Grounder(obs, args.object, thresh=args.det_thresh,
                            log_path=out / "detections.jsonl",
                            colour_min=args.colour_min)
        grounder.start()
        for _ in range(600):                      # wait for the detector to load
            if grounder.latest()["seq"] > 0:
                break
            await asyncio.sleep(0.5)
        print(f"[flight] cruise {args.cruise_alt:.0f} m — following {args.object!r}")

        limiter = RateLimiter(args.dv_h, args.dv_z)
        t0, tick, last_seen, nfz_hold_ticks = time.time(), 0, 0.0, 0
        last_bearing, brg_rate, last_cmd, mode = 0.0, 0.0, (0.0, 0.0), "hold"
        while time.time() - t0 < args.max_s:
            tick += 1
            kin = drone.get_ground_truth_kinematics()
            p = kin["pose"]["position"]
            yaw = quat_yaw(kin["pose"]["orientation"])
            state = State(x=p["x"], y=p["y"], up=-p["z"])
            obs.put_pose(p["x"], p["y"], yaw)
            if traffic is not None:
                traffic.update(time.time() - t0, tick)
            elif car is not None and tick % 2 == 0:
                car.update(time.time() - t0)

            g = grounder.latest()
            # age against the last DETECTION, not the last inference
            det = g["det"]
            age = (time.time() - g["t_det"]) if g["t_det"] else 1e9
            if det is not None and age < args.det_max_age:
                yaw_rate, fwd, bearing = servo(
                    det, det[5], args.yaw_gain, args.want_width, args.speed_max)
                # remember what it was doing, so a gap can be coasted through
                if last_seen:
                    dt = max(1e-3, time.time() - last_seen)
                    brg_rate = 0.6 * brg_rate + 0.4 * ((bearing - last_bearing) / dt)
                last_seen, last_bearing = time.time(), bearing
                last_cmd = (yaw_rate, fwd)
                mode, seen = "track", True
            else:
                # Detection drops on roughly a quarter of ticks — the car leaves
                # frame on a corner, or the detector simply misses. Freezing on
                # every gap meant the drone stopped dead and fell behind, so it
                # coasts on what the target was doing, then searches, and only
                # then gives up.
                seen = False
                if not last_seen:
                    # Never acquired yet. Sweep — do not sit still waiting for a
                    # target to wander into frame. This was the failure mode the
                    # first time: the aircraft held its launch heading for the
                    # whole flight while the car drove a lap behind it.
                    yaw_rate, fwd, bearing = args.search_rate, 0.0, 0.0
                    mode = "search"
                    lost = 0.0
                elif (lost := time.time() - last_seen) < args.coast_s:
                    # keep turning the way the target was moving, decaying
                    k = 1.0 - lost / args.coast_s
                    bearing = last_bearing + brg_rate * lost
                    yaw_rate = float(np.clip(args.yaw_gain * bearing, -1.1, 1.1))
                    fwd = last_cmd[1] * k
                    mode = "coast"
                elif lost < args.coast_s + args.search_s:
                    # sweep back toward the side it was last seen on, slowly
                    side = 1.0 if last_bearing >= 0 else -1.0
                    yaw_rate, fwd, bearing = side * args.search_rate, 0.0, 0.0
                    mode = "search"
                else:
                    # Keep sweeping, slowly, instead of freezing. The target is
                    # driving a circuit and will come back round; a drone that
                    # gives up stays pointed at nothing for the rest of the
                    # flight, which is what it used to do.
                    side = 1.0 if last_bearing >= 0 else -1.0
                    yaw_rate = side * args.search_rate * 0.6
                    fwd, bearing = 0.0, 0.0
                    mode = "scan"

            # altitude hold, in the controller where it belongs — the Shield is a
            # constraint filter, not a regulator
            vz_up = float(np.clip((args.cruise_alt - state.up) * args.alt_gain,
                                  -args.climb_max, args.climb_max))
            cvx, cvy = fwd * math.cos(yaw), fwd * math.sin(yaw)
            fscale, fdist, fblocked = fence.gate(state.x, state.y, cvx, cvy)
            gvx, gvy = cvx * fscale, cvy * fscale
            if fblocked:
                nfz_hold_ticks += 1
                # blocked ahead: try to slide around rather than just stop
                sx, sy = fence.slide(state.x, state.y, cvx, cvy)
                if sx or sy:
                    lat = args.slide_speed * (1.0 - fscale)
                    gvx += sx * lat
                    gvy += sy * lat
                    fmode = "skirt"
                else:
                    fmode = "hold"
            else:
                fmode = "clear" if fdist is None or fdist > 20 else "near"
            raw = Action4D(vx=gvx, vy=gvy, vz_up=vz_up, yaw_rate=yaw_rate)
            smooth = limiter(raw)
            d = shield.filter(state, smooth)
            audit.log(tick, d)
            if d.touched:
                n_touched += 1
            e = d.emitted

            traj.append({"x": state.x, "y": state.y, "up": state.up,
                         "touched": d.touched})
            rows.append({
                "t": round(time.time() - t0, 3), "tick": tick,
                "x": state.x, "y": state.y, "up": state.up, "psi": yaw,
                "det_seq": g["seq"], "det_age_s": round(min(age, 99), 3),
                "seen": seen, "mode": mode,
                "fence_d": (round(fdist, 2) if fdist is not None else None),
                "fence_scale": round(fscale, 3), "fence_hold": fblocked,
                "fence_mode": fmode,
                "infer_ms": round(g["infer_ms"], 1),
                "det": (None if det is None else
                        {"cx": round(det[0], 1), "cy": round(det[1], 1),
                         "w": round(det[2], 1), "score": round(det[4], 4),
                         "colour": round(det[7], 3)}),
                "bearing_deg": round(math.degrees(bearing), 2),
                "raw": raw.model_dump(), "smooth": smooth.model_dump(),
                "emitted": e.model_dump(),
                "touched": d.touched, "braked": d.braked,
                "violations": [v.model_dump() for v in d.violations],
                "repairs": [r.model_dump() for r in d.repairs],
                "tgt_x": (car.pos[0] if car else None),
                "tgt_y": (car.pos[1] if car else None),
            })

            await drone.move_by_velocity_async(
                e.vx, e.vy, -e.vz_up, duration=0.3,
                yaw_is_rate=True, yaw=e.yaw_rate)
            if view_dir is not None and tick % args.view_every == 0:
                v = obs.get_chase_native()
                if v is not None:
                    v.save(view_dir / "tps" / f"{tick:05d}.jpg", quality=85)
                f = obs.get_front_native()
                if f is not None:
                    sep = (math.hypot(state.x - car.pos[0], state.y - car.pos[1])
                           if car else None)
                    annotate(f, det if seen else None, {
                        "t": time.time() - t0, "sep": sep,
                        "brg": math.degrees(bearing), "fwd": fwd,
                        "alt": state.up, "shield": d.touched,
                        "query": args.object, "mode": mode,
                        "fence_d": fdist, "fence_hold": fblocked,
                        "fence_mode": fmode,
                    }).save(view_dir / "fpv" / f"{tick:05d}.jpg", quality=85)
            if tick % 50 == 0:
                sep = (math.hypot(state.x - car.pos[0], state.y - car.pos[1])
                       if car else float("nan"))
                print(f"  tick {tick}: pos=({state.x:6.1f},{state.y:6.1f},"
                      f"{state.up:4.1f}) {mode.upper():6} "
                      f"brg={math.degrees(bearing):+5.1f} fwd={fwd:4.1f} "
                      f"sep={sep:5.1f}m shield={'HIT' if d.touched else '-'}")
            await asyncio.sleep(TICK)

        grounder.stop()
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
    except Exception as exc:
        print(f"[warn] flight aborted: {type(exc).__name__}: {exc}")
    finally:
        if grounder is not None:
            grounder.stop()
        try:
            if traffic is not None:
                traffic.destroy()
            elif car is not None:
                car.destroy()
        except Exception:
            pass
        client.disconnect()

    with (out / "flight_log.jsonl").open("w", encoding="utf-8") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")

    fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
    inside = [pt for pt in traj for f, poly in fences
              if f.altitude_floor_m <= pt["up"] <= f.altitude_ceiling_m
              and poly.contains(Point(pt["x"], pt["y"]))]
    band = policy.by_type(AltitudeEnvelope)
    alt_bad = (sum(1 for pt in traj
                   if pt["up"] < band[0].alt_min_m or pt["up"] > band[0].alt_max_m)
               if band else 0)
    seps = [math.hypot(r["x"] - r["tgt_x"], r["y"] - r["tgt_y"])
            for r in rows if r["tgt_x"] is not None]
    g = grounder.latest() if grounder else {"n_seen": 0, "n_miss": 0}
    metrics = {
        "tag": args.tag, "ticks": len(traj), "object": args.object,
        "detector": DETECTOR_ID,
        "det_seen": g["n_seen"], "det_missed": g["n_miss"],
        "det_hit_rate": round(g["n_seen"] / max(1, g["n_seen"] + g["n_miss"]), 3),
        "det_hz": round((g["n_seen"] + g["n_miss"]) / max(1e-6, len(traj) * TICK), 2),
        "frac_ticks_seen": round(sum(1 for r in rows if r["seen"]) / max(1, len(rows)), 3),
        "mode_frac": {m: round(sum(1 for r in rows if r.get("mode") == m) / max(1, len(rows)), 3)
                      for m in ("track", "coast", "search", "scan")},
        "sep_min_m": round(min(seps), 1) if seps else None,
        "sep_mean_m": round(float(np.mean(seps)), 1) if seps else None,
        "sep_end_m": round(seps[-1], 1) if seps else None,
        "frac_within_30m": round(float(np.mean([s <= 30 for s in seps])), 3) if seps else None,
        "nfz_s": round(len(inside) * TICK, 2), "nfz_entered": bool(inside),
        "alt_violation_s": round(alt_bad * TICK, 1),
        "interventions": n_touched,
        "nfz_hold_ticks": nfz_hold_ticks,
        "params": {"yaw_gain": args.yaw_gain, "want_width": args.want_width,
                   "speed_max": args.speed_max, "cruise_alt": args.cruise_alt,
                   "alt_gain": args.alt_gain, "det_thresh": args.det_thresh},
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=1), encoding="utf-8")
    (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")

    print(f"\n[report] ticks {len(traj)} | detector {metrics['det_hz']} Hz, "
          f"hit rate {metrics['det_hit_rate']} | target visible on "
          f"{metrics['frac_ticks_seen']*100:.0f}% of ticks")
    print(f"[report] separation: min {metrics['sep_min_m']} m, "
          f"mean {metrics['sep_mean_m']} m, end {metrics['sep_end_m']} m, "
          f"within 30 m {metrics['frac_within_30m']}")
    print(f"[report] guardrail: NFZ {metrics['nfz_s']}s, altitude escape "
          f"{metrics['alt_violation_s']}s, interventions {n_touched}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", default="a white car",
                    help="what to follow, in words. This is the only thing that "
                         "tells the drone what its target is.")
    ap.add_argument("--policy", default=str(ROOT / "policies" / "follow_car.yaml"))
    ap.add_argument("--citymap", default=str(ROOT / "demo" / "out" / "citymap" / "occ_day.npz"))
    ap.add_argument("--tag", default="vlmfollow")
    ap.add_argument("--max-s", type=float, default=120.0)
    ap.add_argument("--cruise-alt", type=float, default=9.0)
    ap.add_argument("--car-speed", type=float, default=3.0)
    ap.add_argument("--traffic", type=int, default=0,
                    help="number of BACKGROUND vehicles besides the target. "
                         "They are the same mesh and unpainted, so the noun "
                         "cannot separate them and only the colour test can - "
                         "which turns 'the noun does most of the work' from a "
                         "stated limitation into a measurement. Capped at the "
                         "number of lanes (4).")
    ap.add_argument("--traffic-every", type=int, default=2,
                    help="ticks between teleports for BACKGROUND vehicles. The "
                         "target always updates every tick. At 2 m/s and every "
                         "2nd tick a vehicle moves 0.4 m between updates.")
    ap.add_argument("--car-stop-s", type=float, default=6.0,
                    help="how long the car pauses at each of two points along "
                         "the straight route. A follower has to stop too, so "
                         "this is the clearest evidence the drone is tracking "
                         "the car and not just flying down the same street. "
                         "0 disables the stops.")
    ap.add_argument("--straight", action="store_true",
                    help="drive the car in a straight line and park it at the "
                         "end, instead of looping. A circuit looks natural but "
                         "its turns swing the target through the aircraft's "
                         "blind spot — the front camera cannot see closer than "
                         "0.86 x altitude — and every lost lock costs tracking.")
    ap.add_argument("--no-car", action="store_true",
                    help="control condition: fly the same mission with no car "
                         "in the scene")
    ap.add_argument("--yaw-gain", type=float, default=1.2)
    ap.add_argument("--want-width", type=float, default=0.10,
                    help="target apparent width as a fraction of the image; "
                         "sets the standoff distance")
    ap.add_argument("--speed-max", type=float, default=4.0)
    ap.add_argument("--alt-gain", type=float, default=0.6)
    ap.add_argument("--climb-max", type=float, default=1.8)
    ap.add_argument("--det-thresh", type=float, default=0.02)
    ap.add_argument("--colour-min", type=float, default=0.10,
                    help="minimum fraction of the box that must actually be the "
                         "named colour. 0 disables the colour check.")
    ap.add_argument("--det-max-age", type=float, default=1.0)
    ap.add_argument("--coast-s", type=float, default=2.0,
                    help="after losing the target, keep following its last "
                         "known motion for this long before searching")
    ap.add_argument("--search-s", type=float, default=5.0,
                    help="how long to sweep looking for it before holding still")
    ap.add_argument("--search-rate", type=float, default=0.35,
                    help="yaw rate of the search sweep, rad/s")
    ap.add_argument("--save-view", action="store_true",
                    help="record the third-person (Chase) camera to "
                         "demo/out/<tag>/tps/ for the demo video")
    ap.add_argument("--view-every", type=int, default=3,
                    help="save a third-person frame every N control ticks "
                         "(3 = ~3.3 fps)")
    ap.add_argument("--slide-speed", type=float, default=2.5,
                    help="lateral speed used to skirt round a no-fly zone when "
                         "the direct line is blocked but a gap exists")
    ap.add_argument("--fence-brake", type=float, default=12.0,
                    help="start slowing this far from a no-fly zone")
    ap.add_argument("--fence-standoff", type=float, default=3.0,
                    help="hold this far outside a no-fly zone")
    ap.add_argument("--dv-h", type=float, default=0.4)
    ap.add_argument("--dv-z", type=float, default=0.2)
    return asyncio.run(fly(ap.parse_args()))


if __name__ == "__main__":
    sys.exit(main())
