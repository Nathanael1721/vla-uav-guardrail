"""Ground truth for the figures and cars that live IN the level.

WHY THIS EXISTS

`CityLife_Day` carries its own 16 walking pedestrians and 8 driving cars. The
client never placed them, so it does not know where they are, and
`follow_vlm.subject_truth_pts` answers `[]` for every tick of a flight that
follows one - which `track_truth.score_rows` counts as UNSCORABLE, not as
wrong. A flight in the better-looking level would therefore have produced a
video and no tracking number at all, and nothing in the run would have said so.

WHAT IT ASKS

`World.get_object_poses(names)` is a service the simulator already exposes
(`WorldSimApi::getObjectPose` -> `UnrealHelpers::FindActor` ->
`UnrealTransform::ToGlobalNed`). It returns where the actor IS, in the same NED
metres the flight log uses. That is strictly better evidence than the truth the
client keeps for its own spawned figures, which is the position it COMMANDED -
`demo/pedestrians.py` had to count refused pose updates precisely because the
two can differ.

THE NAME IS NOT THE LABEL

`FindActor` matches an actor whose `GetName()` CONTAINS the string, or which
carries it as a TAG. A Blueprint placed in a level is named after its class -
`BP_CityPed_M1_C_1`, `BP_CityCar_C_6` - and `Ped_07` is only the editor LABEL,
which does not exist in a `-game` build at all. So the level tags each figure
with its intended name and this module looks actors up by that tag. A miss is
not silent: `getObjectPose` returns NaN for an actor it cannot find, and every
NaN is counted and reported.

COST

One RPC per poll, but the simulator loops the names on the game thread, so 16
names is 16 game-thread round trips. `min_period_s` throttles polling below the
control tick; `stats()` reports the calls actually spent, the way
`Pedestrians.stats()` reports its pose updates.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class LevelFigure:
    """One actor that the LEVEL owns. `x`, `y` are NED metres, like `Figure`."""
    name: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    heading: float = 0.0
    seen: bool = False          # has ever resolved
    stale: int = 0              # consecutive polls that came back NaN


def _yaw_from_quat(q) -> float:
    """Yaw in radians from a quaternion dict/attr-dict, NED convention."""
    w, x, y, z = (float(q["w"]), float(q["x"]), float(q["y"]), float(q["z"])) \
        if isinstance(q, dict) else (float(q.w), float(q.x), float(q.y), float(q.z))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LevelActors:
    """Poll the simulator for the poses of actors the level owns.

    Duck-types `demo.pedestrians.Pedestrians` closely enough for
    `follow_vlm`: `.figures` with `.x` / `.y`, plus `update`, `stats`,
    `destroy`. `spawn()` is `resolve()` here, because nothing is spawned.
    """

    def __init__(self, world, names: List[str], kind: str = "pedestrian",
                 min_period_s: float = 0.1):
        self.world = world
        self.kind = kind
        self.min_period_s = float(min_period_s)
        self.figures: List[LevelFigure] = [LevelFigure(n) for n in names]
        self.n_calls = 0
        self.n_nan = 0
        self._last_t: Optional[float] = None

    # -- one poll -----------------------------------------------------------
    def _poll(self) -> int:
        """Ask for every name once. Returns how many came back finite."""
        names = [f.name for f in self.figures]
        if not names:
            return 0
        poses = self.world.get_object_poses(names)
        self.n_calls += 1
        good = 0
        for fig, pose in zip(self.figures, poses):
            t = pose["translation"] if isinstance(pose, dict) else pose.translation
            px = float(t["x"] if isinstance(t, dict) else t.x)
            py = float(t["y"] if isinstance(t, dict) else t.y)
            pz = float(t["z"] if isinstance(t, dict) else t.z)
            if math.isnan(px) or math.isnan(py):
                # A NaN is the simulator saying "no actor by that name". The
                # last known position is KEPT rather than zeroed: (0, 0) is a
                # real place on this map, and a truth that teleports to the
                # origin scores the detector wrong instead of unscorable.
                fig.stale += 1
                self.n_nan += 1
                continue
            r = pose["rotation"] if isinstance(pose, dict) else pose.rotation
            fig.x, fig.y, fig.z = px, py, pz
            fig.heading = _yaw_from_quat(r)
            fig.seen = True
            fig.stale = 0
            good += 1
        return good

    def resolve(self) -> int:
        """First poll. Refuses a flight whose truth would be empty."""
        good = self._poll()
        missing = [f.name for f in self.figures if not f.seen]
        if good == 0:
            raise SystemExit(
                f"[level] none of the {len(self.figures)} {self.kind} name(s) "
                f"resolved in the simulator: {', '.join(missing[:8])}"
                f"{' ...' if len(missing) > 8 else ''}. The level tags each "
                f"actor with its name; without those tags FindActor matches "
                f"the Blueprint's own name (BP_CityPed_M1_C_1) instead, and "
                f"every tick of this flight would log an empty truth.")
        if missing:
            print(f"[level] {len(missing)} of {len(self.figures)} {self.kind} "
                  f"name(s) never resolved and are dropped: "
                  f"{', '.join(missing)}")
            self.figures = [f for f in self.figures if f.seen]
        print(f"[level] tracking {len(self.figures)} {self.kind}(s) from the "
              f"level, polled at most every {self.min_period_s:.2f} s")
        # The first poll counts as the poll for t = 0, so `update` honours the
        # period from the start instead of granting a free extra call.
        self._last_t = 0.0
        return len(self.figures)

    # `Pedestrians` calls this `spawn`; nothing is spawned, but the caller
    # should not have to care which kind of source it holds.
    spawn = resolve

    def update(self, t: float) -> None:
        """Throttled re-poll. `t` is seconds since the flight started."""
        if self._last_t is not None and (t - self._last_t) < self.min_period_s:
            return
        self._last_t = t
        self._poll()

    def refresh(self) -> None:
        """Poll now, outside the flight clock, without moving the throttle.

        For reads taken before the mission clock starts (the start gate's "was
        that the subject?" check). Going through `update(t)` there would record
        a pre-mission time as the last poll, and the first ticks of the mission
        - whose t restarts near zero - would all be skipped as too soon.
        """
        self._poll()

    def stats(self) -> dict:
        return {"source": "level", "kind": self.kind,
                "figures": len(self.figures),
                "rpc_polls": self.n_calls,
                "nan_reads": self.n_nan,
                "stale_now": sum(1 for f in self.figures if f.stale)}

    def destroy(self) -> None:
        """The level owns these actors; destroying them is not ours to do."""
        print(f"[level] {self.n_calls} pose poll(s), {self.n_nan} NaN read(s)")


class LevelCar:
    """One car the LEVEL drives, standing in for the client's scripted car.

    `follow_vlm` reads a subject car through a handful of attributes - `.pos` for
    the truth row and the live separation, `.update(t)` once a tick, `.destroy()`
    at the end, `.spec.desc_match` for the colour sanity check - and none of
    them care who moves the car. So a car on a CityLife loop becomes the subject
    by polling ONE tag, and the flight is scored against that one instance: a
    crowd of look-alikes cannot saturate the null, because the truth list has a
    single entry.

    `actual_name` is None for the same reason the env-actor car's is: there is
    nothing here for the client to teleport or destroy.
    """

    def __init__(self, world, tag: str, desc: Optional[str] = None,
                 min_period_s: float = 0.1):
        self.tag = tag
        self.source = LevelActors(world, [tag], kind="car",
                                  min_period_s=min_period_s)
        self.pos = (0.0, 0.0)
        self.heading = 0.0
        self.actual_name = None
        self.spec = _Spec(desc or tag)

    def _sync(self) -> None:
        if self.source.figures:
            f = self.source.figures[0]
            self.pos = (f.x, f.y)
            self.heading = f.heading

    def spawn(self) -> int:
        """Resolve the tag. Refuses the flight if it does not resolve."""
        n = self.source.resolve()
        self._sync()
        return n

    def update(self, t: float) -> None:
        self.source.update(t)
        self._sync()

    def refresh(self) -> None:
        self.source.refresh()
        self._sync()

    def stats(self) -> dict:
        return dict(self.source.stats(), tag=self.tag)

    def destroy(self) -> None:
        self.source.destroy()


@dataclass
class _Spec:
    """The one field of `moving_car.CarSpec` the flight reads for a level car."""
    desc_match: str


def names(prefix: str, count: int, start: int = 0) -> List[str]:
    """`Ped_00 … Ped_15`, the tag convention the level uses."""
    return [f"{prefix}{i:02d}" for i in range(start, start + count)]
