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

# ONE shared circuit for all background traffic, driven at different phases -
# cars circulating a block, which is what a street actually looks like.
#
# The previous design gave each distractor its own thin loop, 1.0 m wide with a
# 0.6 m corner radius, so it could keep moving without leaving its lane. Measured,
# that turned the heading 10.9 degrees per 0.1 s tick: 109 deg/s, against the
# ~30 deg/s a real car manages. The vehicles snapped round rather than turning,
# and that is what read as "kasar" on the video.
#
# A 10 m x 66 m circuit with a 4 m corner radius gives v/r = 0.5 rad/s at 2 m/s,
# which is 2.9 deg per tick - inside what a car does.
#
# The legs sit at x = 34 and x = 44, so they bracket the target lane at 38
# without ever entering it: 4 m clear on one side, 6 m on the other. Both stay
# inside the [32, 46] envelope, because the obstacle map holds buildings only and
# a flight at x = 48 hit street furniture it cannot see.
CIRCUIT_X = (34.0, 44.0)
ROUTE_Y0, ROUTE_Y1 = -8.0, 58.0
# The circuit runs 8 m PAST each end of the target's route. Its cross-legs are
# the only place a distractor is at x = 38, and putting them outside -8..58 keeps
# that off the stretch of road the target actually drives - otherwise a
# distractor sweeps through the target's lane at exactly the moment the aircraft
# is looking there.
CIRCUIT_Y0, CIRCUIT_Y1 = ROUTE_Y0 - 8.0, ROUTE_Y1 + 8.0
CIRCUIT_CORNER_R = 4.0


def circuit() -> List[Tuple[float, float]]:
    """The closed lap every background vehicle drives, anticlockwise."""
    x0, x1 = CIRCUIT_X
    return [(x0, CIRCUIT_Y0), (x0, CIRCUIT_Y1),
            (x1, CIRCUIT_Y1), (x1, CIRCUIT_Y0)]


# Materials that EXIST on disk under PASBlocks/Content/Geometry/Materials.
#
# The earlier conclusion that "the sim refuses every material except M_Orange"
# was wrong, and wrong in an embarrassing way: M_Red, M_Green, M_White and
# M_Black were guesses and no such assets exist, so of course they were refused.
# The real set is short and oddly named - some with the M_ prefix, some without.
# Which of these actually RENDER as their name on SKM_SportsCar is measured by
# experiments/survey_materials.py, not assumed.
TARGET_MATERIAL = "/Game/Geometry/Materials/M_Orange"
TARGET_ASSET = "SKM_SportsCar"
TARGET_COLOUR_WORD = "white"        # M_Orange renders WHITE on the sports car

# Distractors differ by MESH as well as material, because in this build colour
# alone cannot be varied. Three routes were tried and measured:
#
#   set_object_material   accepts exactly ONE material, M_Orange. Blue.uasset,
#                         Green.uasset and Yellow.uasset provably exist on disk
#                         and are still refused - only cooked materials load.
#   set_object_texture    accepts EVERYTHING and changes nothing: hue 208, S 72,
#                         V 161 on every candidate, identical to the untouched
#                         mesh. "Returns True" and "renders differently" are not
#                         the same claim.
#   spawn_object_from_file  glTF with an embedded baseColorTexture would work and
#                         is the real answer, but needs a model downloaded first.
#
# So variety comes from the two vehicle meshes that exist. M_Orange renders
# genuinely ORANGE on the offroad body (measured 0.118 orange coverage) and WHITE
# on the sports car, which gives three distinguishable appearances.
#
# NOTE THE TRADE. Mixing meshes makes the scene easier to read and easier to
# track, which is what a DEMO wants. It also confounds shape with colour, so a
# correct lock could be the paint or could be the silhouette. For the controlled
# discrimination experiment use --traffic-mode experiment, which holds the mesh
# constant and accepts that the distractors are then only distinguishable from
# the target and not from each other.
BG_PALETTE_DEMO: List[Tuple[Optional[str], str, str]] = [
    ("/Game/Geometry/Materials/M_Orange", "orange", "SM_Offroad_Body"),
    (None, "", "SM_Offroad_Body"),
    (None, "", "SKM_SportsCar"),
]

# Same mesh as the target; only the paint differs. Weaker visually, stronger as
# evidence.
BG_PALETTE_EXPERIMENT: List[Tuple[Optional[str], str, str]] = [
    (None, "", "SKM_SportsCar"),
    (None, "", "SKM_SportsCar"),
    (None, "", "SKM_SportsCar"),
]


@dataclass
class VehicleSpec:
    """One vehicle's identity, route, colour and update rate."""

    name: str
    speed_mps: float
    # Fraction of the lap, not a number of seconds. Absolute offsets do not
    # survive vehicles having different speeds: phase_s = 31 s against a 32.7 s
    # route once left a distractor parked 1.7 s into a 60 s flight.
    phase_frac: float
    is_target: bool
    # Material asset path, or None to leave the mesh in its own colours.
    material: Optional[str]
    # The colour word this vehicle should answer to, for the ground-truth record.
    colour_word: str
    asset: str = TARGET_ASSET
    update_every: int = 1
    stops: List[Tuple[float, float]] = field(default_factory=list)

    def route(self) -> List[Tuple[float, float]]:
        """The target drives a straight one-shot run and parks; every distractor
        drives the shared closed circuit."""
        if self.is_target:
            return [(TARGET_LANE_X, ROUTE_Y0), (TARGET_LANE_X, ROUTE_Y1)]
        return circuit()

    def car_spec(self) -> CarSpec:
        sp = CarSpec()
        sp.asset = self.asset
        if self.asset == "SM_Offroad_Body":
            # The offroad body is a different size and it is NOT a car body - it
            # is an open roll cage. Recorded here so the ground truth is honest.
            sp.length_m, sp.width_m, sp.height_m = 3.7, 1.8, 1.2
        sp.materials = [self.material] if self.material else []
        sp.desc_match = f"a {self.colour_word} car" if self.colour_word else "a car"
        return sp


def default_fleet(n_background: int = 3, speed: float = 2.0,
                  target_stops: Optional[List[Tuple[float, float]]] = None,
                  bg_every: int = 1, mode: str = "demo",
                  palette=None) -> List[VehicleSpec]:
    """Target plus `n_background` distractors sharing one circuit.

    Distractors are spread evenly around the lap rather than given separate
    lanes. On a 152 m circuit three vehicles sit ~50 m apart, which is traffic
    rather than a convoy, and it removes the thin per-vehicle loops whose 0.6 m
    corners were turning the heading at 109 deg/s.

    `palette` is (material path, colour word) per distractor. Different colours
    are the point: with every car the same colour the noun cannot separate them
    AND neither can the colour gate, so the tracker has nothing to hold on to and
    wanders between them. That is what it was doing.
    """
    fleet = [VehicleSpec(name="TargetCar", speed_mps=speed, phase_frac=0.15,
                         is_target=True, material=TARGET_MATERIAL,
                         colour_word=TARGET_COLOUR_WORD, asset=TARGET_ASSET,
                         update_every=1, stops=list(target_stops or []))]
    pal = list(palette or (BG_PALETTE_EXPERIMENT if mode == "experiment"
                           else BG_PALETTE_DEMO))
    n_background = int(n_background)
    if n_background > len(pal):
        print(f"[traffic] {n_background} distractors requested but only "
              f"{len(pal)} distinct appearances are available - using {len(pal)}")
        n_background = len(pal)
    for i in range(n_background):
        mat, word, asset = pal[i]
        fleet.append(VehicleSpec(
            name=f"BgCar{i + 1}",
            # SAME speed for every distractor, and that is deliberate. They
            # share one circuit, so unequal speeds mean a faster car eventually
            # catches a slower one and drives through it - measured, min
            # separation hit 0.0 m. At equal speed the phase gap is constant and
            # they never meet. The scene still does not read as a rigid body,
            # because they are spread round the lap and the two long legs run in
            # opposite directions.
            speed_mps=speed,
            # Evenly around the lap, offset from the target's phase.
            phase_frac=(0.15 + (i + 1) / (n_background + 1)) % 1.0,
            is_target=False,
            material=mat,
            colour_word=word,
            asset=asset,
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
            # Only the TARGET runs one_shot. Background vehicles loop.
            #
            # A one_shot vehicle parks at the end of its route and stops being a
            # distractor: 66 m at 1.5 m/s is 44 s, so on a 90 s flight half the
            # traffic is stationary scenery by the midpoint, and the selection
            # problem quietly gets easier exactly when it should not. Looping
            # reverses their heading at each end, which is the artefact the
            # straight route was introduced to avoid -- but that artefact only
            # matters for the TRACKED target, whose continuity filter it breaks.
            # On a distractor it costs a little realism and buys a distractor
            # that is still there at the end of the flight.
            car = MovingCar(
                world, speed_mps=vs.speed_mps, route=vs.route(), name=vs.name,
                spec=vs.car_spec(),
                corner_r=(corner_r if vs.is_target else CIRCUIT_CORNER_R),
                phase_s=0.0,
                one_shot=vs.is_target, stops=list(vs.stops))
            # lap_time is only known once the speed profile is built, so the
            # fraction is converted here rather than guessed by the caller.
            car.phase_s = vs.phase_frac * car.lap_time
            car.pos = car.pose_at(0.0)[:2]
            self.cars.append(car)
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
            if vs.material and car.material_used is None:
                print(f"[traffic] *** {vs.name}: PAINT FAILED - the target is not "
                      f"white, the colour query cannot select it ***")
            if not vs.material and car.material_used is not None:
                print(f"[traffic] *** {vs.name}: a distractor got painted "
                      f"({car.material_used}) - it will compete with the target ***")
        print(f"[traffic] {len(self.cars)} vehicles "
              f"({sum(1 for v in self._specs if v.material)} painted), "
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
                 "material_applied": car.material_used,
                 "material": vs.material, "colour_word": vs.colour_word,
                 "asset": vs.asset,
                 "speed_mps": round(vs.speed_mps, 2),
                 "phase_frac": round(vs.phase_frac, 3),
                 "phase_s": round(car.phase_s, 1),
                 "lap_s": round(car.lap_time, 1),
                 "parked_ticks": car._parked_ticks, 
                 "update_every": vs.update_every,
                 "asset": car.spec.asset}
                for vs, car in zip(self._specs, self.cars)
            ],
        }
