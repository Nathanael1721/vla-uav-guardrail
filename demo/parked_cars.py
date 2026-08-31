"""Vehicles at the kerb, as scenery.

WHY PARKED RATHER THAN MORE TRAFFIC

A real street carries far more parked cars than moving ones, and here that
happens to also be the cheap direction. A moving vehicle costs one
`set_object_pose` every tick - about 9 RPC/s each at the ~8.7 Hz control loop,
and `--traffic-every` defaults to 1 so every background vehicle pays it. Six of
them is ~52 RPC/s inside the same loop that runs OWL-ViT, whose `det_hz` only
just clears its 4.0 Hz gate.

A parked car is spawned once and never touched again. It costs nothing per tick,
so the street can be filled without spending any of that budget. Same reasoning
as the standing pedestrians in `demo/pedestrians.py`, and the same code is
reused to place them.

WHERE THEY MAY STAND

`pedestrians.pavement_spots()` already finds the cells that are street AND
adjacent to a building - the kerb. Parked cars want the same cells pedestrians
want, so the two are allocated from one draw and never share a spot.

They must also stay out of the way of anything that actually drives, which is
two polylines: the target's route up x = 38, and the background circuit whose
legs run at x = 34 and x = 44. `pedestrians._dist_to_route()` measures against a
polyline rather than its corners, which is what makes "3 m clear of the road"
mean what it says.

THE ONE MODEL THAT IS EXCLUDED

`taxi.glb`. It is the target's mesh and the standing query is "a yellow car", so
a parked taxi would not be a distractor - it would be a second correct answer,
and `det_hit_rate` would fall for a reason no amount of tuning could fix. The
palette below is checked against that by a test, not by care.
"""
from __future__ import annotations

import math
import os
import pathlib
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

from pedestrians import _dist_to_route, pavement_spots

# Kenney Car Kit, CC0, the same pack the moving fleet draws from. Deliberately
# NOT taxi.glb - see the module docstring. Nothing here is yellow.
PARKED_PALETTE: List[str] = [
    "sedan.glb", "suv.glb", "van.glb", "delivery.glb", "truck.glb",
    "hatchback-sports.glb", "suv-luxury.glb", "sedan-sports.glb",
    "truck-flat.glb", "delivery-flat.glb", "ambulance.glb", "police.glb",
]

# The model that must never appear parked, named once so the test can import it.
TARGET_MODEL = "taxi.glb"

# Matching demo/moving_car.py's CarSpec defaults: the pack is authored small and
# faces +Y, so the same scale and yaw offset apply to a parked car as a driving
# one. Getting these wrong gives either toy cars or cars parked across the road.
GLB_SCALE = 2.0
GLB_YAW_OFFSET_DEG = 270.0
GROUND_Z = 0.0

# How far a parked car must stay from anything that drives. A lane is 4 m, the
# models are ~2 m wide at this scale, so 3 m of clearance from the centre line
# puts them at the kerb rather than in the carriageway.
ROAD_CLEARANCE_M = 3.0


def glb_dir(explicit: Optional[str] = None) -> Optional[pathlib.Path]:
    """Where the vehicle models live, or None if they were never fetched.

    Same contract as `city_traffic.glb_dir()`: None rather than an exception, so
    a machine without the third-party pack still flies.
    """
    import city_traffic
    return city_traffic.glb_dir(explicit)


@dataclass
class Parked:
    name: str
    glb: pathlib.Path
    x: float
    y: float
    heading: float
    actual_name: Optional[str] = None


def kerb_spots(street_mask, buildings, n: int, rng: random.Random,
               route: List[Tuple[float, float]],
               circuit: List[Tuple[float, float]],
               taken: List[Tuple[float, float]] = (),
               apart_m: float = 5.0,
               near_radius_m: float = 22.0) -> List[Tuple[float, float]]:
    """Kerbside cells clear of both driving lines and of anything already placed.

    `taken` is where the pedestrians went. Two objects in one cell is the one
    placement fault that looks like a bug rather than a scene, so the spots are
    drawn once and partitioned instead of being sampled twice independently.
    """
    # Ask for far more than needed: most candidates are rejected below, and
    # pavement_spots shuffles, so over-drawing keeps the spread.
    pool = pavement_spots(street_mask, buildings, n * 25, rng, avoid=route,
                          avoid_radius_m=ROAD_CLEARANCE_M,
                          near_radius_m=near_radius_m)
    out: List[Tuple[float, float]] = []
    for x, y in pool:
        if _dist_to_route(x, y, list(route)) < ROAD_CLEARANCE_M:
            continue                       # in the target's lane
        if circuit and _dist_to_route(x, y, list(circuit)) < ROAD_CLEARANCE_M:
            continue                       # in a background vehicle's path
        if any(math.hypot(x - px, y - py) < apart_m
               for px, py in list(taken) + out):
            continue                       # on top of a pedestrian, or another car
        out.append((x, y))
        if len(out) >= n:
            break
    return out


def _pose(x: float, y: float, h: float):
    """Typed Pose, as the spawn API requires.

    A plain dict of the same shape is refused with `ERROR code: 2.0, message:
    Unknown exception`, which reads like a bad mesh and is not one. See
    demo/pedestrians.py::_pose - this cost two rounds of chasing the wrong thing.
    """
    from projectairsim.types import Pose, Quaternion, Vector3
    from projectairsim.utils import rpy_to_quaternion
    w, qx, qy, qz = rpy_to_quaternion(
        0.0, 0.0, h + math.radians(GLB_YAW_OFFSET_DEG))
    return Pose({
        "translation": Vector3({"x": float(x), "y": float(y), "z": GROUND_Z}),
        "rotation": Quaternion({"w": w, "x": qx, "y": qy, "z": qz}),
        "frame_id": "DEFAULT_ID",
    })


class ParkedCars:
    """Spawn a row of parked vehicles and then forget about them entirely."""

    def __init__(self, world, street_mask, buildings, count: int = 0,
                 seed: int = 20260828, models_dir: Optional[str] = None,
                 route: List[Tuple[float, float]] = (),
                 circuit: List[Tuple[float, float]] = (),
                 taken: List[Tuple[float, float]] = ()):
        self.world = world
        self.rng = random.Random(seed)
        self.dir = glb_dir(models_dir)
        self.count = int(count)
        self.street_mask = street_mask
        self.buildings = buildings
        self.route = list(route)
        self.circuit = list(circuit)
        self.taken = list(taken)
        self.cars: List[Parked] = []

    def available(self) -> bool:
        return self.dir is not None

    def spawn(self) -> None:
        if self.count <= 0:
            return
        if self.dir is None:
            print("[parked] no vehicle models installed; the kerb stays empty")
            return
        models = [self.dir / f for f in PARKED_PALETTE if (self.dir / f).is_file()]
        if not models:
            print(f"[parked] {self.dir} holds none of the parked palette")
            return

        spots = kerb_spots(self.street_mask, self.buildings, self.count,
                           self.rng, self.route, self.circuit, self.taken)
        if len(spots) < self.count:
            print(f"[parked] only {len(spots)} kerb spots clear of the roads; "
                  f"asked for {self.count}")

        n_fail = 0
        for k, (x, y) in enumerate(spots):
            glb = models[k % len(models)]
            # Park along the kerb, not at a random angle: face up or down the
            # nearest road direction. The route runs north-south here, so the
            # two legal headings are 0 and pi.
            h = 0.0 if (k % 2 == 0) else math.pi
            car = Parked(name=f"ParkedCar{k}", glb=glb, x=x, y=y, heading=h)
            try:
                car.actual_name = self.world.spawn_object_from_file(
                    car.name, "gltf", glb.read_bytes(), True, _pose(x, y, h),
                    [GLB_SCALE, GLB_SCALE, GLB_SCALE], False)
                self.cars.append(car)
            except Exception as exc:                          # noqa: BLE001
                n_fail += 1
                if n_fail == 1:
                    print(f"[parked] spawn failed ({type(exc).__name__}: {exc})")

        kinds = len({c.glb.name for c in self.cars})
        print(f"[parked] {len(self.cars)} vehicle(s) at the kerb, {kinds} model(s), "
              f"costing nothing per tick")

    def destroy(self) -> None:
        for car in self.cars:
            if car.actual_name:
                try:
                    self.world.destroy_object(car.actual_name)
                except Exception:                             # noqa: BLE001
                    pass
