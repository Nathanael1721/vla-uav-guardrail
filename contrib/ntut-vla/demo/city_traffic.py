"""Several vehicles on the street at once, so "follow the white car" becomes a
choice rather than a lookup.

WHY THIS EXISTS

The follow demo's honest limitation number one is that the noun does most of the
work. With a single car in the scene, "a white car" and "a car" pick the same
object, and the colour gate never has to discriminate anything -- it only has to
avoid rejecting the only candidate. That is a weaker claim than it sounds, and it
is the one a reviewer goes for first.

With several cars on the same street the query has to select. Every distractor
scores well as "a car" -- they are the same mesh -- so the noun cannot separate
them and only the colour test can. That turns limitation one into a measurement.

THE DESIGN CHOICE THAT MATTERS: SAME MESH, DIFFERENT COLOUR

An earlier plan mixed meshes (a sports car against the old offroad buggy). That
would confound shape with colour: if the drone locked onto the right vehicle we
could not say whether it did so because of the paint or because the other one
looks like a roll cage. Here every vehicle is SKM_SportsCar, so silhouette,
apparent size and detector affinity are held constant and colour is the only free
variable.

WHAT COLOURS ARE ACTUALLY AVAILABLE

Two, and not by choice. `list_assets('.*')` returns 2109 names and not one is a
material, so material paths cannot be discovered from the client, and every path
tried except `/Game/Geometry/Materials/M_Orange` was refused by the sim. On
SKM_SportsCar that material renders WHITE (a material is a shader, not a colour --
the same one renders orange on the offroad mesh). So:

    target      SKM_SportsCar + M_Orange   reads white   OWL "a white car" 0.1178
    distractor  SKM_SportsCar default      reads blue-grey

See docs/FINDING-vehicle-mesh-and-materials.md for the measurements.

The risk this creates is stated up front rather than discovered later: the
default mesh measured a white fraction of 0.117 at the near pose against a colour
gate of 0.10. That is close. If the distractors pass the white gate, the
experiment shows nothing and the fix is a higher `--colour-min`, not a quieter
report. `summary()` returns the per-vehicle ground truth needed to check it.

THE RPC BUDGET

Motion is client-side teleport: one `set_object_pose` per vehicle per update,
issued from the same 10 Hz asyncio loop that runs the detector. One car costs
10 RPC/s. Six cars at full rate would cost 60, inside the loop that also has to
service inference, and that is the wrong place to spend a tick.

So updates are staggered. The target updates every tick because the aircraft is
closing on it and the ground truth is scored against it. Background vehicles
update every `bg_every` ticks; at 2 m/s and every 2nd tick a vehicle moves 0.4 m
between updates, which at 9 m altitude and 400x225 is well under a pixel of
apparent motion error. Position is a pure function of elapsed time
(`MovingCar.pose_at`), so a skipped update introduces no drift -- the vehicle
simply teleports to exactly where it should be, slightly later.

Default six vehicles: 10 RPC/s for the target plus 25 for five background
vehicles at every 2nd tick = 35 RPC/s, against 10 for the single-car demo.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from moving_car import CarSpec, MovingCar

# The corridor is free for x in [30, 50] at >= 5 m building clearance, and the
# single-car demo drives the middle of it at x = 38. Lanes are placed either side
# of that, 4 m apart, which is a real lane width and keeps every vehicle inside
# the clearance envelope. Nothing is placed outside [32, 46]: the obstacle map
# holds BUILDINGS ONLY, and a flight that drifted to x = 48 hit street furniture
# the map cannot see.
TARGET_LANE_X = 38.0
# 3 m lane pitch either side of the target, nearest lanes first so a small fleet
# clusters around it rather than hiding at the kerb. Four lanes is the hard limit
# and default_fleet() clamps to it: a fifth vehicle would have to share a lane,
# and the first version of this file did exactly that -- two cars in lane 34
# driving opposite ways met head-on at 0.0 m separation at t=27.8 s, which the
# fleet-geometry test caught before it ever reached the simulator.
BG_LANE_X = (35.0, 41.0, 32.0, 44.0)

# Same start and end as STRAIGHT_ROUTE, so every vehicle covers the same ground
# and the target is not the only one that happens to pass through frame.
ROUTE_Y0, ROUTE_Y1 = -8.0, 58.0


@dataclass
class VehicleSpec:
    """One vehicle's identity, route and update rate."""

    name: str
    lane_x: float
    speed_mps: float
    phase_s: float
    is_target: bool
    painted: bool                    # True -> M_Orange -> reads white
    reverse: bool = False            # drive the lane the other way
    update_every: int = 1
    stops: List[Tuple[float, float]] = field(default_factory=list)

    def route(self) -> List[Tuple[float, float]]:
        y0, y1 = (ROUTE_Y1, ROUTE_Y0) if self.reverse else (ROUTE_Y0, ROUTE_Y1)
        return [(self.lane_x, y0), (self.lane_x, y1)]

    def car_spec(self) -> CarSpec:
        """Painted vehicles get M_Orange (renders white on this mesh); the rest
        get an empty material list so the mesh keeps its own blue-grey."""
        s = CarSpec()
        if not self.painted:
            s.materials = []
            s.desc_match = "a car"
        return s


def default_fleet(n_background: int = 5, speed: float = 2.0,
                  target_stops: Optional[List[Tuple[float, float]]] = None,
                  bg_every: int = 2) -> List[VehicleSpec]:
    """Target plus `n_background` distractors, spread in lane and in phase.

    Phase offsets matter more than they look. If every vehicle started at the
    same point the aircraft would see one clump and the selection problem would
    be trivial -- and worse, the distractors would occlude each other. Spreading
    the phase over the lap puts vehicles at different ranges, so the target has
    to be picked out of a scene where other cars are both nearer and further.
    """
    fleet = [VehicleSpec(name="TargetCar", lane_x=TARGET_LANE_X, speed_mps=speed,
                         phase_s=10.0, is_target=True, painted=True,
                         update_every=1,
                         stops=list(target_stops or []))]
    # One vehicle per lane, never two. Sharing a lane is not a cosmetic problem:
    # two cars in the same lane going opposite ways pass through each other,
    # because physics is off and they are teleported.
    n_background = int(n_background)
    if n_background > len(BG_LANE_X):
        print(f"[traffic] {n_background} background vehicles requested but only "
              f"{len(BG_LANE_X)} lanes exist - using {len(BG_LANE_X)}")
        n_background = len(BG_LANE_X)
    for i in range(n_background):
        lane = BG_LANE_X[i]
        fleet.append(VehicleSpec(
            name=f"BgCar{i + 1}",
            lane_x=lane,
            # Not all the same speed: identical speeds make the whole scene move
            # as one rigid body, which reads as a texture rather than as traffic.
            speed_mps=speed * (0.75 + 0.15 * (i % 3)),
            phase_s=10.0 + 7.0 * (i + 1),
            is_target=False,
            painted=False,
            # Alternate direction so the street has oncoming traffic.
            reverse=bool(i % 2),
            update_every=max(1, int(bg_every)),
        ))
    return fleet


class Traffic:
    """Spawns and drives a fleet. One `MovingCar` per vehicle, reused as-is.

    `MovingCar` already takes name, route, spec, speed, phase and stops, so this
    class adds only the fleet-level concerns: staggered updates, spawn/destroy
    lifecycle, and reporting which vehicles ended up with the colour they were
    supposed to have.
    """

    def __init__(self, world, fleet: Optional[List[VehicleSpec]] = None,
                 corner_r: float = 6.0):
        self.world = world
        self.fleet = list(fleet or default_fleet())
        self.cars: List[MovingCar] = []
        self._specs: List[VehicleSpec] = []
        for vs in self.fleet:
            self.cars.append(MovingCar(
                world, speed_mps=vs.speed_mps, route=vs.route(), name=vs.name,
                spec=vs.car_spec(), corner_r=corner_r, phase_s=vs.phase_s,
                one_shot=True, stops=list(vs.stops)))
            self._specs.append(vs)
        self.n_updates = 0
        self.n_skipped = 0

    @property
    def target(self) -> MovingCar:
        for vs, car in zip(self._specs, self.cars):
            if vs.is_target:
                return car
        return self.cars[0]

    def spawn(self) -> None:
        """Spawn every vehicle, and be loud about any that lost its colour.

        A distractor that silently keeps the target's paint, or a target that
        silently loses it, does not crash anything -- it just quietly turns the
        discrimination experiment into a coin flip. So the check is explicit.
        """
        for vs, car in zip(self._specs, self.cars):
            car.spawn()
            if vs.painted and car.material_used is None:
                print(f"[traffic] *** {vs.name}: PAINT FAILED - the target is not "
                      f"white, the colour query cannot select it ***")
            if not vs.painted and car.material_used is not None:
                print(f"[traffic] *** {vs.name}: a distractor got painted "
                      f"({car.material_used}) - it will compete with the target ***")
        print(f"[traffic] {len(self.cars)} vehicles "
              f"({sum(1 for v in self._specs if v.painted)} painted), "
              f"{self.rpc_per_second():.0f} RPC/s at 10 Hz")

    def update(self, t: float, tick: int) -> None:
        """Move the fleet. Only vehicles due this tick pay an RPC."""
        for vs, car in zip(self._specs, self.cars):
            if tick % vs.update_every:
                # Keep the ground truth current even when the sim is not told:
                # pose_at is a pure function of time, so scoring stays exact.
                car.pos = car.pose_at(t)[:2]
                self.n_skipped += 1
                continue
            car.update(t)
            self.n_updates += 1

    def rpc_per_second(self, loop_hz: float = 10.0) -> float:
        return sum(loop_hz / vs.update_every for vs in self._specs)

    def destroy(self) -> None:
        for car in self.cars:
            car.destroy()

    def positions(self, t: float) -> List[Tuple[str, float, float]]:
        return [(vs.name,) + tuple(car.pose_at(t)[:2])
                for vs, car in zip(self._specs, self.cars)]

    def min_separation(self, t: float) -> float:
        """Closest pair of vehicles. Physics is off so they cannot collide, but
        two meshes in the same place look like one object to the detector and
        would make the selection ambiguous for the wrong reason."""
        pts = [p[1:] for p in self.positions(t)]
        best = float("inf")
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                best = min(best, math.dist(pts[i], pts[j]))
        return best

    def summary(self) -> dict:
        return {
            "n_vehicles": len(self.cars),
            "rpc_per_s_at_10hz": round(self.rpc_per_second(), 1),
            "updates_sent": self.n_updates,
            "updates_skipped": self.n_skipped,
            "vehicles": [
                {"name": vs.name, "is_target": vs.is_target,
                 "painted": vs.painted, "material": car.material_used,
                 "lane_x": vs.lane_x, "speed_mps": round(vs.speed_mps, 2),
                 "phase_s": vs.phase_s, "reverse": vs.reverse,
                 "update_every": vs.update_every,
                 "asset": car.spec.asset}
                for vs, car in zip(self._specs, self.cars)
            ],
        }
