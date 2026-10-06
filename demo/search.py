"""After the subject is lost: where to fly and where to look, on its streets.

WHY

demo/follow_vlm.py, once the red car leaves the frame, predicts it for 3 s,
coasts 2 s to a lookout on its trail (demo/trail.py) and searches 5 s: a
+-25 deg sweep about the car's last direction of travel, creeping on along it.
After that it SCANNED - 0.21 rad/s of yaw in place with zero forward speed,
for the rest of the flight. On the reference flight (citylife_redcar_trail.mp4)
that parked the aircraft against a building corner, rotating, while the car it
had lost drove on. Rotating in place throws away the one thing the aircraft
still knows: which way the car was going, on a map where it cannot go far.

A car in this level drives fixed keep-left loops on an 82 m street grid
(tools/citylife_routes.py). One last seen northbound on a N-S street is, some
seconds later, still on that street, or at the next junction north, or in one
of that junction's three other exits. So instead of rotating in place:

  PURSUE    fly to the next junction AHEAD of the last sighting along its
            street (the heading snapped to N-S or E-W), at about the car's
            measured speed (1.5-5 m/s), looking down that street, +-35 deg.
  DWELL     within 4 m of it: 2 s looking down each exit the car could have
            taken - left, straight, right - never back the way it came. Which
            is left is read with citylife_routes.left_of, the function the
            lanes themselves were built with (X north, Y east: left-handed
            from above, so the left of heading (dx, dy) is (dy, -dx)).
  HOLD      then stay over that junction, turning slowly (a full turn per
            30 s): a junction sees four streets at once, and a car on a loop
            comes back round. After the pursue timeout (30 s from the start of
            PURSUE) the plan stops wherever it is and HOLDs at that junction.
  STANDOFF  a car last seen STOPPED has not gone anywhere: stop 15.8 m short of
            it on its street (the follow's own stand-off), look at it, sweep
            +-25 deg, and never pass it. Approaching a stopped car to 5 m put
            it in the camera's 6.9 m blind spot under the nose, and
            citylife_redcar_trail2 never saw it again (follow_vlm.py, coast).

A car last seen moving with no heading (a trail too short to give one) HOLDs at
the junction nearest the last sighting: the one place with a view of every
street it could be on.

Every target is put ON THE STREET MASK when one is given
(demo/out/citymap_citylife/street.npz). Junction centres are not always street
there: (41, -41) is a low-clutter cell at 2-4 m (a pole or a sign), so the
junction target is moved to the nearest street cell, 2 m away.

INPUTS are what follow_vlm already has for the coast - the estimator's last
accepted position, the trail's end heading, the MEASURED speed and "stopped"
flag - plus the map: junction positions and the street mask. Never ground
truth, never which loop the car is on.

FRAMES: NED metres, x = North, y = East. Headings are atan2(dy, dx): 0 = North,
+pi/2 = East (clockwise from above). Unreal cm / 100 = NED m in this level.

Pure, no simulator: tests/test_search.py.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE.parent):              # demo/ for siblings, root for tools/
    if str(_p) not in sys.path:
        sys.path.append(str(_p))

import camera_model as cam                                    # noqa: E402
from build_street_mask import is_street, load_street          # noqa: E402
from tools import citylife_routes as routes                   # noqa: E402

Vec = Tuple[float, float]

PURSUE, DWELL, HOLD, STANDOFF = "pursue", "dwell", "hold", "standoff"

GRID_M = routes.GRID_CM / 100.0                 # 82 m between junctions
LANE_OFFSET_M = routes.LANE_OFFSET_CM / 100.0   # 3.5 m lane centre off the crown

# FrontCamera (robot_semantic_quad.jsonc) and the follow's cruise altitude.
IMG_W, IMG_H, HFOV_DEG = 768, 432, 90.0
CRUISE_ALT_M = 8.0

STANDOFF_M = 15.8               # the follow's stand-off for a 4 m car (0.16 width)
PURSUE_TIMEOUT_S = 30.0         # from the start of PURSUE; HOLD after it
ARRIVE_M = 4.0                  # "at the junction"
DWELL_PER_EXIT_S = 2.0
PURSUE_SWEEP_DEG = 35.0         # half-amplitudes, inside the 45 deg half-HFOV:
STANDOFF_SWEEP_DEG = 25.0       # the street / the sighting never leaves frame
HOLD_SWEEP_DEG = 180.0          # the whole circle - by rotation, not oscillation
HOLD_RATE_RAD_S = 2.0 * math.pi / 30.0     # one turn per 30 s (~ the old scan)
SPEED_MIN_MPS, SPEED_MAX_MPS = 1.5, 5.0
DWELL_SPEED_MPS = 1.0           # settle over the junction while looking

# An exit exists when the street mask has road down it: at 14-30 m from the
# junction centre (past the 11 m junction box), across the crown and both
# lane centres. Most, not all, samples: the real mask has single-cell holes
# on lane centres (56 of loop A's 408 path points read off-street).
EXIT_PROBE_M = (14.0, 18.0, 22.0, 26.0, 30.0)
EXIT_PROBE_LATERAL_M = (-LANE_OFFSET_M, 0.0, LANE_OFFSET_M)
EXIT_MIN_HITS = 3

_AXES = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))
_AXIS_HEADINGS = (0.0, math.pi / 2, math.pi, -math.pi / 2)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _finite(v) -> Optional[float]:
    """float(v), or None for None / NaN / inf: an estimator that lost its
    velocity must not take the flight loop down (round(nan) raises)."""
    if v is None:
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def _finite_xy(p) -> Optional[Vec]:
    if p is None:
        return None
    x, y = _finite(p[0]), _finite(p[1])
    return None if x is None or y is None else (x, y)


def _quadrant(heading: float) -> int:
    return int(round(heading / (math.pi / 2))) % 4


def snap_heading(heading: float) -> float:
    """The street axis nearest a heading: 0 (N), pi/2 (E), pi (S) or -pi/2 (W).
    Exactly 45 deg off both rounds half-to-even - deterministic, not clever."""
    return _AXIS_HEADINGS[_quadrant(heading)]


def axis_unit(heading: float) -> Vec:
    """Exact unit vector (north, east) of the street axis nearest `heading`."""
    return _AXES[_quadrant(heading)]


def junctions_from_routes(loops: Optional[Dict[str, Sequence[Vec]]] = None
                          ) -> List[Vec]:
    """Every junction the car loops drive through, NED metres, sorted: loop
    corners AND the junctions a loop crosses straight over
    (citylife_routes.junctions_on) - a car can be lost before either kind."""
    seen = set()
    for corners in (loops or routes.LOOPS).values():
        for x, y in routes.junctions_on(corners):
            seen.add((round(x / 100.0, 6), round(y / 100.0, 6)))
    return sorted(seen)


def camera_blind_m(alt_m: float = CRUISE_ALT_M) -> float:
    """Ground distance under the nose the FrontCamera cannot see: where the
    bottom image row meets flat ground (6.87 m at 8 m, level body)."""
    ray = cam.ray_world(IMG_W / 2.0, IMG_H, IMG_W, IMG_H, HFOV_DEG, yaw=0.0)
    hit = cam.ground_hit(ray, alt_m)
    return 0.0 if hit is None else math.hypot(hit[0], hit[1])


class SearchPlanner:
    """Where to fly and look while the subject is lost, after coast + search.

        planner = SearchPlanner(junctions_from_routes(), load_street(path))
        cmd = planner.step(t_lost_s, (x, y), anchor)

    `t_lost_s` counts from the start of PURSUE (the end of follow_vlm's
    coast + search). `anchor` = dict(p=(x, y) last sighting, heading=rad or
    None, speed=m/s measured or None, stopped=bool). NaN / inf read as None;
    p None (no usable sighting) HOLDs at the junction nearest the aircraft.
    The plan is fixed per anchor: a different anchor, or `t_lost_s` going
    backwards, starts a new one (so does reset()). Returns

        mode            "pursue" | "dwell" | "hold" | "standoff"
        target_xy       where to fly (NED m), on the street mask when given
        look_heading_rad  where to point the nose (NED yaw, wrapped)
        sweep_deg       half-amplitude of an oscillating sweep about it
        speed_cap_mps   horizontal speed limit toward target_xy
        rotate_rad_s    >0 only in HOLD: look_heading_rad already advances at
                        this rate - track it, add no oscillation (sweep_deg
                        180 says the rotation covers the circle)
        junction        the junction centre the plan is about (None: standoff)
        exit            "left" | "straight" | "right" while dwelling, else None
        on_street       is_street(target_xy), or None without a mask
    """

    def __init__(self, junctions: Optional[Sequence[Vec]] = None,
                 street=None, *, standoff_m: float = STANDOFF_M,
                 pursue_timeout_s: float = PURSUE_TIMEOUT_S,
                 overshoot_m: float = 0.0, cruise_alt_m: float = CRUISE_ALT_M):
        js = junctions if junctions is not None else junctions_from_routes()
        self.junctions: List[Vec] = [(float(j[0]), float(j[1])) for j in js]
        if not self.junctions:
            raise ValueError("no junctions")
        if isinstance(street, (str, Path)):
            street = load_street(street)
        self.street = street
        self._cells = None
        if street is not None:
            ij = np.argwhere(np.asarray(street["street"]) != 0)
            if len(ij) == 0:
                raise ValueError("street mask has no street cells")
            self._cells = np.stack([street["ox"] + ij[:, 0] * street["res"],
                                    street["oy"] + ij[:, 1] * street["res"]],
                                   axis=1)
        # Never stand off inside the blind spot plus 2 m - the floor the
        # follow's coast uses (follow_vlm.py, camera_blind_m + 2).
        self.floor_m = camera_blind_m(cruise_alt_m) + 2.0
        self.standoff_m = max(float(standoff_m), self.floor_m)
        self.pursue_timeout_s = float(pursue_timeout_s)
        self.overshoot_m = float(overshoot_m)
        self.reset()

    def reset(self) -> None:
        self._key = None
        self._plan: Optional[dict] = None
        self._t_prev: Optional[float] = None
        self._t_arrive: Optional[float] = None

    # ------------------------------------------------------------ map helpers

    def on_street(self, q: Vec) -> Vec:
        """q itself when it is street (or there is no mask); else the centre
        of the nearest street cell (first in row-major order on a tie)."""
        q = (float(q[0]), float(q[1]))
        if self.street is None or is_street(self.street, q[0], q[1]):
            return q
        d2 = (self._cells[:, 0] - q[0]) ** 2 + (self._cells[:, 1] - q[1]) ** 2
        k = int(np.argmin(d2))
        return (float(self._cells[k, 0]), float(self._cells[k, 1]))

    def nearest_junction(self, p: Vec) -> Vec:
        return min(self.junctions,
                   key=lambda j: (math.hypot(j[0] - p[0], j[1] - p[1]), j))

    def junction_ahead(self, p: Vec, u: Vec) -> Optional[Vec]:
        """The first junction centre ahead of p along axis u, on p's street
        (within half a block of the line through p)."""
        best = None
        for j in self.junctions:
            dx, dy = j[0] - p[0], j[1] - p[1]
            along = dx * u[0] + dy * u[1]
            lateral = abs(dx * u[1] - dy * u[0])
            if along > 0.0 and lateral < GRID_M / 2.0:
                cand = (along, lateral, j)
                if best is None or cand < best:
                    best = cand
        return None if best is None else best[2]

    def has_exit(self, j: Vec, d: Vec) -> bool:
        """Is there road leaving junction j along axis d? Always, without a mask."""
        if self.street is None:
            return True
        n = routes.right_of(d)
        hits = 0
        for s in EXIT_PROBE_M:
            if any(is_street(self.street, j[0] + d[0] * s + n[0] * o,
                             j[1] + d[1] * s + n[1] * o)
                   for o in EXIT_PROBE_LATERAL_M):
                hits += 1
        return hits >= EXIT_MIN_HITS

    def exits(self, j: Vec, u: Vec) -> List[Tuple[str, float]]:
        """(name, heading) of the exits of j for a car arriving along u, in
        the order they are looked at: left, straight, right."""
        out = []
        for name, d in (("left", routes.left_of(u)), ("straight", u),
                        ("right", routes.right_of(u))):
            if self.has_exit(j, d):
                out.append((name, math.atan2(d[1], d[0])))
        return out

    def _street_axis_at(self, p: Vec, aircraft: Vec) -> Vec:
        """Axis of the street a sighting with no heading is on: the one it is
        displaced along from the nearest junction, or - inside the junction
        box, where both fit - the one the aircraft is approaching along."""
        jn = self.nearest_junction(p)
        dx, dy = p[0] - jn[0], p[1] - jn[1]
        if max(abs(dx), abs(dy)) <= routes.JUNCTION_HALF_CM / 100.0:
            dx, dy = aircraft[0] - p[0], aircraft[1] - p[1]
        return (1.0, 0.0) if abs(dx) >= abs(dy) else (0.0, 1.0)

    # ------------------------------------------------------------ the plans

    def _make_plan(self, anchor: dict, aircraft: Vec) -> dict:
        p = _finite_xy(anchor.get("p"))
        h = _finite(anchor.get("heading"))
        v = _finite(anchor.get("speed"))
        cap = min(SPEED_MAX_MPS, max(SPEED_MIN_MPS, v if v is not None else 0.0))
        if p is None:                     # no usable sighting: watch the junction
            return self._hold_plan(aircraft, cap)       # nearest the aircraft
        if anchor.get("stopped"):
            return self._standoff_plan(p, h, aircraft, cap)
        if h is None:
            return self._hold_plan(p, cap)
        hs = snap_heading(h)
        u = axis_unit(hs)
        j = self.junction_ahead(p, u)
        if j is None:                     # off the known network: watch the nearest
            return self._hold_plan(p, cap)
        target = self.on_street((j[0] + u[0] * self.overshoot_m,
                                 j[1] + u[1] * self.overshoot_m))
        return {"mode": PURSUE, "junction": j, "target": target, "heading": hs,
                "exits": self.exits(j, u), "cap": cap}

    def _hold_plan(self, p: Vec, cap: float) -> dict:
        j = self.nearest_junction(p)
        h0 = (math.atan2(p[1] - j[1], p[0] - j[0])
              if math.hypot(p[0] - j[0], p[1] - j[1]) > 1.0 else 0.0)
        return {"mode": HOLD, "junction": j, "target": self.on_street(j),
                "h0": h0, "cap": cap}

    def _standoff_plan(self, p: Vec, h, aircraft: Vec, cap: float) -> dict:
        u = axis_unit(h) if h is not None else self._street_axis_at(p, aircraft)
        # On the street's crown - the line through the nearest junction along
        # u - which is the farthest the aircraft can be from the kerbs.
        jn = self.nearest_junction(p)
        n = (-u[1], u[0])
        lat = (p[0] - jn[0]) * n[0] + (p[1] - jn[1]) * n[1]
        c = (p[0] - lat * n[0], p[1] - lat * n[1])
        # On the aircraft's side of the sighting, so reaching the target never
        # means passing it; level with it, behind the car's direction.
        side = (aircraft[0] - p[0]) * u[0] + (aircraft[1] - p[1]) * u[1]
        s = 1.0 if side > 1.0 else -1.0
        target = None
        if self.street is None:
            target = (c[0] + s * self.standoff_m * u[0], c[1] + s * self.standoff_m * u[1])
        else:
            # Further back along the street, never nearer, until it is road.
            for extra in np.arange(0.0, 20.0 + 1e-9, 0.5):
                q = (c[0] + s * (self.standoff_m + extra) * u[0],
                     c[1] + s * (self.standoff_m + extra) * u[1])
                if is_street(self.street, q[0], q[1]):
                    target = (float(q[0]), float(q[1]))
                    break
            if target is None:
                # The street runs off the mask behind the car (a map edge):
                # the street cell nearest the ideal point that is still on
                # the aircraft's side and outside the blind floor. A plain
                # nearest-cell snap put it 8.5 m from a car stopped 10 m from
                # the real mask's east edge - inside the 8.87 m floor.
                ideal = (c[0] + s * self.standoff_m * u[0],
                         c[1] + s * self.standoff_m * u[1])
                cells = self._cells
                rel_x, rel_y = cells[:, 0] - p[0], cells[:, 1] - p[1]
                ok = ((s * (rel_x * u[0] + rel_y * u[1]) >= 0.0)
                      & (np.hypot(rel_x, rel_y) >= self.floor_m))
                if ok.any():
                    cand = cells[ok]
                    k = int(np.argmin((cand[:, 0] - ideal[0]) ** 2
                                      + (cand[:, 1] - ideal[1]) ** 2))
                    target = (float(cand[k, 0]), float(cand[k, 1]))
                else:
                    target = self.on_street(ideal)
        look = math.atan2(p[1] - target[1], p[0] - target[0])
        return {"mode": STANDOFF, "target": target, "look": look, "cap": cap}

    # ------------------------------------------------------------ the step

    @staticmethod
    def _anchor_key(anchor: dict) -> tuple:
        def r(v):                         # NaN != NaN would restart every tick
            v = _finite(v)
            return None if v is None else round(v, 3)
        p = _finite_xy(anchor.get("p"))
        return (None if p is None else (r(p[0]), r(p[1])), r(anchor.get("heading")),
                r(anchor.get("speed")), bool(anchor.get("stopped")))

    def _out(self, mode: str, target: Vec, look: float, sweep: float, cap: float,
             junction: Optional[Vec] = None, exit_name: Optional[str] = None,
             rotate: float = 0.0) -> dict:
        return {"mode": mode, "target_xy": target, "look_heading_rad": _wrap(look),
                "sweep_deg": sweep, "speed_cap_mps": cap, "rotate_rad_s": rotate,
                "junction": junction, "exit": exit_name,
                "on_street": (None if self.street is None
                              else is_street(self.street, target[0], target[1]))}

    def _hold(self, plan: dict, t: float, t0: float, h0: float) -> dict:
        look = h0 + HOLD_RATE_RAD_S * max(0.0, t - t0)
        return self._out(HOLD, plan["target"], look, HOLD_SWEEP_DEG, plan["cap"],
                         junction=plan["junction"], rotate=HOLD_RATE_RAD_S)

    def step(self, t_lost_s: float, aircraft_xy: Vec, anchor: dict) -> dict:
        t = float(t_lost_s)
        aircraft = (float(aircraft_xy[0]), float(aircraft_xy[1]))
        key = self._anchor_key(anchor)
        if key != self._key or (self._t_prev is not None and t < self._t_prev):
            self.reset()
            self._key = key
            self._plan = self._make_plan(anchor, aircraft)
        self._t_prev = t
        plan = self._plan

        if plan["mode"] == STANDOFF:
            return self._out(STANDOFF, plan["target"], plan["look"],
                             STANDOFF_SWEEP_DEG, plan["cap"])
        if plan["mode"] == HOLD:
            return self._hold(plan, t, 0.0, plan["h0"])

        T = self.pursue_timeout_s
        tgt = plan["target"]
        if (self._t_arrive is None and t < T
                and math.hypot(tgt[0] - aircraft[0], tgt[1] - aircraft[1]) <= ARRIVE_M):
            self._t_arrive = t
        if self._t_arrive is None:
            if t < T:
                return self._out(PURSUE, tgt, plan["heading"], PURSUE_SWEEP_DEG,
                                 plan["cap"], junction=plan["junction"])
            return self._hold(plan, t, T, plan["heading"])      # never got there

        exits, ta = plan["exits"], self._t_arrive
        k = int((t - ta) // DWELL_PER_EXIT_S)
        if t < T and k < len(exits):
            name, heading = exits[k]
            return self._out(DWELL, tgt, heading, 0.0, DWELL_SPEED_MPS,
                             junction=plan["junction"], exit_name=name)
        dwell_end = ta + DWELL_PER_EXIT_S * len(exits)
        if not exits:
            t0, h0 = ta, plan["heading"]
        elif dwell_end <= T:                  # every exit looked at
            t0, h0 = dwell_end, exits[-1][1]
        else:                                 # the timeout cut the dwell short
            t0 = T
            h0 = exits[min(len(exits) - 1, int((T - ta) // DWELL_PER_EXIT_S))][1]
        return self._hold(plan, t, t0, h0)
