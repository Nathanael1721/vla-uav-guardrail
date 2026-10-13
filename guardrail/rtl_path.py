"""The hand-over to ArduPilot's RTL, made answerable to P0.

WHY THIS FILE EXISTS

The escalation FSM's RTL -> Land edge, "RTL fails (battery / blocked path)"
(Safety Shield p4, edge G6 in guardrail/fsm.py), takes `rtl_failed` from the
autopilot side, and until 2026-10-12 nothing in the repository computed it: the
headless harness passed `home_reached` and `landed` only, so G6 could never
fire, and ArduPilot's RTL - a straight line home at RTL_ALT that knows nothing
of the Shield's zones - was the end of every fail-safe whatever lay on the way.
The library sweep published on 2026-10-12 counted 680 ticks on which the
modelled autopilot flew a P0-violating action (10 episodes); most were LOITER
holds and one landing, not RTL legs, but nothing would have stopped an RTL leg
through a zone either.

WHAT IT ANSWERS (pure geometry and arithmetic: no ROS, no MAVLink, no clock)

  check_leg        Is the straight RTL path from here clear? ArduCopter's RTL
                   climbs to RTL_ALT (1500 cm by default, ArduCopter/config.h
                   at Copter-4.5.7) when below it, then flies straight home at
                   that altitude, or at the current one when above RTL_ALT.
                   Both pieces are tested against every P0 zone in force (its
                   margin ring, the shape the Shield forbids), and the return
                   altitude against the P0 altitude band. A zone the aircraft
                   is already inside does not block the leg by itself when the
                   leg is its EXIT: the stretch from the aircraft to the zone's
                   edge never goes deeper than the aircraft already is (the
                   Shield's own test of an aircraft inside a zone is that it
                   is escaping, guardrail/shield.py _check_fence). A leg that
                   crosses the zone from just inside its far edge, re-enters
                   it, or ends at a home inside it is blocked.
  ProgressMonitor  Is RTL getting the aircraft home? `remaining` is the climb
                   still to do plus the horizontal distance to home; RTL has
                   failed when, over the last `window_s`, it fell by less than
                   `min_rate_mps` per second (a wind beyond the airframe's
                   authority, an obstacle the autopilot is pressed against).
  BatteryHook      The battery fail-safe hook: critical from `critical_at_s`,
                   or whatever a reader callable reports (a rail would read
                   BATTERY_STATUS / SYS_STATUS).
  RTLWatch         All three per tick -> RTLVerdict, whose `failed` is the
                   flag Shield.filter(rtl_failed=...) and fsm.TickInput take.

`zones_from_shield` and `band_from_shield` read the rules from a
guardrail.shield.Shield, so the leg is judged on the Shield's own compiled
geometry and schedule ("there is exactly one rule-evaluation code path",
Safety Shield p2).

THE NUMBERS AND WHERE THEY COME FROM

  RTL_ALT_M 15.0          ArduCopter's RTL_ALT default (config.h: 1500 cm).
  PROGRESS_WINDOW_S 5.0   the grant's T (FSM defaults, Safety Shield p4): an
                          RTL gets the same 5 s the grant gives N-in-T.
  MIN_PROGRESS_MPS 0.1    the trend threshold X6 and X7 use
                          (fsm.FSMConfig.x6_min_escape_mps).

WHAT IT DOES NOT DECIDE

What the FSM does after G6 (Land, where the aircraft is, which may still be
over a zone), and which hold it picks before RTL, belong to guardrail/fsm.py.
A blocked leg is reported as soon as it is seen, so an RTL entered with a
zone between the aircraft and home fails on its first tick; whether the FSM
should then hold (BRAKE) and wait instead of landing is a question for it,
not for this module.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, NamedTuple

RTL_ALT_M = 15.0
PROGRESS_WINDOW_S = 5.0
MIN_PROGRESS_MPS = 0.1
_EPS_M = 1e-6


class Zone(NamedTuple):
    """One keep-out zone as the leg check needs it: `shape` is the forbidden
    shape (a shapely Polygon: the authored zone plus its margin)."""
    rule_id: str
    shape: Any
    floor_m: float
    ceiling_m: float


class Band(NamedTuple):
    """The P0 altitude band in force (the tightest of all in force)."""
    rule_id: str
    alt_min_m: float
    alt_max_m: float


@dataclass(frozen=True)
class LegCheck:
    clear: bool
    rtl_alt_m: float
    blocking: tuple[str, ...] = ()        # rule ids that block the leg
    reasons: tuple[str, ...] = ()
    starts_inside: tuple[str, ...] = ()   # zones the aircraft is inside now


def rtl_altitude(up: float, rtl_alt_m: float = RTL_ALT_M) -> float:
    """ArduCopter's return altitude: RTL_ALT, or the current altitude when the
    aircraft is already above it."""
    return max(float(up), float(rtl_alt_m))


def remaining_to_home(x: float, y: float, up: float, home: tuple[float, float],
                      rtl_alt_m: float = RTL_ALT_M) -> float:
    """Metres of RTL still to fly: the climb to the return altitude plus the
    horizontal distance home (the final landing at home is not counted)."""
    return max(0.0, rtl_alt_m - up) + math.hypot(home[0] - x, home[1] - y)


def _pieces(geom) -> list:
    if geom is None or geom.is_empty:
        return []
    if hasattr(geom, "geoms"):
        out = []
        for g in geom.geoms:
            out.extend(_pieces(g))
        return out
    return [geom] if geom.length > _EPS_M else []


# How finely an exit piece is sampled for its depth profile, and the slack a
# sample may sit deeper than the one before it (float noise, not geometry).
_EXIT_STEP_M = 0.25
_EXIT_TOL_M = 1e-6


def _is_exit(piece, shape, p, home) -> bool:
    """Is `piece` (a stretch of the leg inside `shape` that starts at the
    aircraft `p`) the way OUT? It must end on the zone's edge, not at a home
    inside the zone, and its depth - the distance to the shape's boundary -
    must never grow along it. For a convex zone the depth along a straight
    line is concave, so one deeper sample anywhere means the leg crosses the
    zone instead of leaving it; for a non-convex one the samples are
    `_EXIT_STEP_M` apart."""
    from shapely.geometry import Point
    a, b = Point(piece.coords[0]), Point(piece.coords[-1])
    if a.distance(p) > b.distance(p):
        a, b = b, a
    if b.distance(Point(home)) <= 1e-6:
        return False                       # home itself is inside the zone
    n = min(4000, max(2, int(math.ceil(piece.length / _EXIT_STEP_M))))
    edge = shape.boundary
    prev = edge.distance(a)
    for k in range(1, n + 1):
        q = Point(a.x + (b.x - a.x) * k / n, a.y + (b.y - a.y) * k / n)
        d = edge.distance(q)
        if d > prev + _EXIT_TOL_M:
            return False
        prev = d
    return True


def check_leg(x: float, y: float, up: float, home: tuple[float, float],
              zones: Iterable[Zone], band: Band | None = None,
              rtl_alt_m: float = RTL_ALT_M) -> LegCheck:
    """Is the straight RTL path from (x, y, up) to `home` clear of every zone
    in `zones` and inside `band`? (Module docstring.)"""
    from shapely.geometry import LineString, Point
    h = rtl_altitude(up, rtl_alt_m)
    p = Point(x, y)
    blocking: list[str] = []
    reasons: list[str] = []
    inside: list[str] = []
    seg = (LineString([(x, y), (home[0], home[1])])
           if math.hypot(home[0] - x, home[1] - y) > _EPS_M else None)
    for z in zones:
        in_footprint = z.shape.contains(p)
        if in_footprint and z.floor_m <= up <= z.ceiling_m:
            inside.append(z.rule_id)
        already = z.rule_id in inside
        # The climb: a vertical segment at (x, y) from `up` to `h`.
        if (h > up + _EPS_M and in_footprint and not already
                and z.floor_m <= h and z.ceiling_m >= up):
            blocking.append(z.rule_id)
            reasons.append(f"the climb to {h:g} m enters {z.rule_id} "
                           f"({z.floor_m:g}-{z.ceiling_m:g} m)")
            continue
        # The return leg at h.
        if seg is None or not (z.floor_m <= h <= z.ceiling_m):
            continue
        parts = _pieces(seg.intersection(z.shape))
        if already:
            # Leaving a zone it is inside is the exit, not a block - but only
            # the piece that starts at the aircraft can be the exit, and only
            # if it leaves (_is_exit); any other piece re-enters the zone.
            parts = [g for g in parts
                     if not ((Point(g.coords[0]).distance(p) <= 1e-6
                              or Point(g.coords[-1]).distance(p) <= 1e-6)
                             and _is_exit(g, z.shape, p, home))]
        if parts:
            blocking.append(z.rule_id)
            ends = " and ends there: home is inside it" if z.shape.contains(
                Point(home[0], home[1])) else ""
            reasons.append(f"the leg home at {h:g} m crosses {z.rule_id} "
                           f"({sum(g.length for g in parts):.1f} m inside its ring{ends})")
    if band is not None and not (band.alt_min_m - 1e-9 <= h <= band.alt_max_m + 1e-9):
        blocking.append(band.rule_id)
        why = ("RTL keeps the current altitude above RTL_ALT" if up > rtl_alt_m
               else "RTL_ALT")
        reasons.append(f"the return altitude {h:g} m ({why}) is outside "
                       f"{band.rule_id} [{band.alt_min_m:g}, {band.alt_max_m:g}] m")
    return LegCheck(clear=not blocking, rtl_alt_m=h, blocking=tuple(blocking),
                    reasons=tuple(reasons), starts_inside=tuple(inside))


class ProgressMonitor:
    """Is RTL getting the aircraft home? Feed (t, remaining_m) on every RTL
    tick; `update` returns (failed, why). Nothing is judged before a full
    window has been seen, so the first `window_s` of an RTL never fails here."""

    def __init__(self, window_s: float = PROGRESS_WINDOW_S,
                 min_rate_mps: float = MIN_PROGRESS_MPS):
        if not (window_s > 0 and min_rate_mps >= 0):
            raise ValueError("window_s must be > 0 and min_rate_mps >= 0")
        self.window_s = float(window_s)
        self.min_rate_mps = float(min_rate_mps)
        self._s: deque[tuple[float, float]] = deque()

    def reset(self) -> None:
        self._s.clear()

    def update(self, t: float, remaining_m: float) -> tuple[bool, str]:
        s = self._s
        s.append((float(t), float(remaining_m)))
        # Keep the latest sample at or before t - window as the window start.
        while len(s) >= 2 and s[1][0] <= t - self.window_s + 1e-9:
            s.popleft()
        t0, r0 = s[0]
        span = t - t0
        if span < self.window_s - 1e-9:
            return False, ""
        made = r0 - remaining_m
        if made < self.min_rate_mps * span:
            return True, (f"no progress home: {made:+.2f} m in {span:.1f} s "
                          f"(needs {self.min_rate_mps * span:.2f} m)")
        return False, ""


@dataclass
class BatteryHook:
    """The battery fail-safe hook. `critical_at_s`: the battery reports
    critical from this time on (None: never). `reader`, when given, is called
    with t and returns True while the battery is critical; a rail would read
    the autopilot's battery state there."""
    critical_at_s: float | None = None
    reader: Callable[[float], bool] | None = None

    def critical(self, t: float) -> bool:
        if self.reader is not None:
            return bool(self.reader(t))
        return self.critical_at_s is not None and t >= self.critical_at_s - 1e-9


@dataclass(frozen=True)
class RTLVerdict:
    failed: bool
    reasons: tuple[str, ...]
    leg: LegCheck
    battery_critical: bool
    no_progress: bool

    @property
    def note(self) -> str:
        return "; ".join(self.reasons)


@dataclass
class RTLWatch:
    """Per-tick RTL assessment for a caller that flies the FSM's verdict:
    call `assess` on every tick the autopilot is in RTL and pass `failed` as
    `rtl_failed`; call `reset` when RTL is left."""
    home: tuple[float, float]
    rtl_alt_m: float = RTL_ALT_M
    battery: BatteryHook = field(default_factory=BatteryHook)
    window_s: float = PROGRESS_WINDOW_S
    min_rate_mps: float = MIN_PROGRESS_MPS

    def __post_init__(self) -> None:
        self._progress = ProgressMonitor(self.window_s, self.min_rate_mps)

    def reset(self) -> None:
        self._progress.reset()

    def assess(self, t: float, x: float, y: float, up: float,
               zones: Iterable[Zone], band: Band | None = None) -> RTLVerdict:
        leg = check_leg(x, y, up, self.home, zones, band, self.rtl_alt_m)
        stuck, why = self._progress.update(
            t, remaining_to_home(x, y, up, self.home, self.rtl_alt_m))
        batt = self.battery.critical(t)
        reasons = [f"blocked path: {r}" for r in leg.reasons]
        if stuck:
            reasons.append(why)
        if batt:
            reasons.append(f"battery critical (from {self.battery.critical_at_s:g} s)"
                           if self.battery.critical_at_s is not None
                           else "battery critical")
        return RTLVerdict(failed=bool(reasons), reasons=tuple(reasons), leg=leg,
                          battery_critical=batt, no_progress=stuck)


# --------------------------------------------------------------------------- #
# reading the rules from a Shield
# --------------------------------------------------------------------------- #

def _enforced_p0(rule) -> bool:
    return (getattr(rule, "priority", "P0") == "P0"
            and getattr(rule, "constraint_type", "hard") == "hard"
            and getattr(rule, "violation_action", "repair") != "monitor_only")


def zones_from_shield(shield) -> list[Zone]:
    """Every hard, enforced P0 zone the Shield has in force now (moving zones
    where they are now), as the forbidden shape: polygon plus margin."""
    return [Zone(r.rule.id, r.buffered, float(r.rule.altitude_floor_m),
                 float(r.rule.altitude_ceiling_m))
            for r in shield.zones_now(None) if _enforced_p0(r.rule)]


def band_from_shield(shield, state) -> Band | None:
    """The tightest hard, enforced P0 altitude band in force now, or None."""
    lo, hi, ids = -math.inf, math.inf, []
    for row in shield.rule_status(state):
        rule = row.get("rule")
        if row.get("type") != "altitude_envelope" or not row.get("in_force") \
                or rule is None or not _enforced_p0(rule):
            continue
        lo, hi = max(lo, float(rule.alt_min_m)), min(hi, float(rule.alt_max_m))
        ids.append(rule.id)
    return Band("+".join(ids), lo, hi) if ids else None
