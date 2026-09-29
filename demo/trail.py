"""Where the subject has been: breadcrumbs, and a carrot to follow along them.

WHY

A car that turns a corner is followed by flying where it DROVE, not where it
is. The follow controller used to point every horizontal command along the
aircraft's nose, and the nose at the subject, so at a junction the aircraft cut
the corner toward a car that had already turned - and, once the car was out of
sight, the estimator predicted it straight on for 3 s and coast and search
carried the aircraft straight past the junction (citylife_redcar_ground and its
siblings lost the car at the first corner every time).

Here the breadcrumbs are the ESTIMATOR's positions after each accepted
detection - built from the box, the depth range and the aircraft's own pose,
never from ground truth - so following them keeps the demo's claim that the
only steering input is where the detector puts the box.

    trail.add(x, y)                      after each accepted estimator update
    trail.carrot(px, py, lookahead)      a point `lookahead` m along the trail,
                                         ahead of where the aircraft projects
    trail.remaining(px, py)              metres of trail still ahead
    trail.end_heading()                  the subject's last direction of travel

Pure geometry, no simulator: tested in tests/test_trail.py.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

Vec = Tuple[float, float]


class Trail:
    def __init__(self, spacing_m: float = 1.5, max_len_m: float = 120.0,
                 max_jump_m: Optional[float] = 30.0):
        self.spacing_m = spacing_m
        self.max_len_m = max_len_m
        # A breadcrumb this far from the last one is not the subject having
        # driven there: it is a re-acquisition somewhere else (or a different
        # subject), and the straight segment between the two could run through
        # a building. The trail restarts instead.
        self.max_jump_m = max_jump_m
        self.pts: List[Vec] = []
        self._proj_i = 0          # segment the aircraft last projected onto
        self.n_restarts = 0
        self.n_added = 0          # breadcrumbs laid, over the whole flight

    def __len__(self) -> int:
        return len(self.pts)

    def clear(self) -> None:
        self.pts.clear()
        self._proj_i = 0

    def add(self, x: float, y: float) -> bool:
        """Append a breadcrumb if it is at least `spacing_m` from the last one.
        Returns True when a point was added."""
        if self.pts:
            lx, ly = self.pts[-1]
            gap = math.hypot(x - lx, y - ly)
            if gap < self.spacing_m:
                return False
            if self.max_jump_m is not None and gap > self.max_jump_m:
                self.clear()
                self.n_restarts += 1
        self.pts.append((float(x), float(y)))
        self.n_added += 1
        self._trim()
        return True

    def _trim(self) -> None:
        total = 0.0
        for i in range(len(self.pts) - 1, 0, -1):
            a, b = self.pts[i - 1], self.pts[i]
            total += math.hypot(b[0] - a[0], b[1] - a[1])
            if total > self.max_len_m:
                drop = i                  # keep pts[i:], at most max_len_m long
                self.pts = self.pts[drop:]
                self._proj_i = max(0, self._proj_i - drop)
                return

    def _project(self, px: float, py: float) -> Tuple[int, float]:
        """(segment index, fraction along it) of the point on the trail nearest
        (px, py). Searches from the last projection onward, so a trail that
        doubles back near itself cannot make the aircraft jump backwards."""
        n = len(self.pts)
        best = (self._proj_i, 0.0, float("inf"))
        for i in range(self._proj_i, n - 1):
            a, b = self.pts[i], self.pts[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            L2 = dx * dx + dy * dy
            u = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / L2))
            d = math.hypot(px - a[0] - u * dx, py - a[1] - u * dy)
            if d < best[2] - 1e-9:
                best = (i, u, d)
        self._proj_i = best[0]
        return best[0], best[1]

    def carrot(self, px: float, py: float, lookahead_m: float) -> Optional[Vec]:
        """The point `lookahead_m` along the trail past the aircraft's
        projection; the trail's end if that is closer. None without a trail."""
        if len(self.pts) < 2:
            return None
        i, u = self._project(px, py)
        a, b = self.pts[i], self.pts[i + 1]
        cx, cy = a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])
        left = lookahead_m
        while True:
            nx, ny = self.pts[i + 1]
            seg = math.hypot(nx - cx, ny - cy)
            if seg >= left:
                f = left / seg if seg > 0 else 0.0
                return (cx + f * (nx - cx), cy + f * (ny - cy))
            left -= seg
            cx, cy = nx, ny
            i += 1
            if i >= len(self.pts) - 1:
                return (cx, cy)

    def remaining(self, px: float, py: float) -> float:
        """Metres of trail between the aircraft's projection and the end.

        An aircraft BEHIND the first breadcrumb projects onto its start (u = 0
        on segment 0), and the gap from it to that breadcrumb counts too - it
        is street still to fly. Left out, a trail shorter than the stand-off
        (every trail at the start of a follow, and after every restart) read as
        a metre or two to go, and the coast approach crawled at 0.5 m/s with
        the last sighting 20 m away (found in review, 2026-09-24)."""
        if len(self.pts) < 2:
            return 0.0
        i, u = self._project(px, py)
        a, b = self.pts[i], self.pts[i + 1]
        total = math.hypot(b[0] - a[0], b[1] - a[1]) * (1.0 - u)
        if i == 0 and u <= 0.0:
            total += math.hypot(px - a[0], py - a[1])
        for j in range(i + 1, len(self.pts) - 1):
            p, q = self.pts[j], self.pts[j + 1]
            total += math.hypot(q[0] - p[0], q[1] - p[1])
        return total

    def end(self) -> Optional[Vec]:
        return self.pts[-1] if self.pts else None

    def length(self) -> float:
        """Metres of trail from its first breadcrumb to its last."""
        return sum(math.hypot(self.pts[i + 1][0] - self.pts[i][0],
                              self.pts[i + 1][1] - self.pts[i][1])
                   for i in range(len(self.pts) - 1))

    def end_heading(self, span_m: float = 6.0) -> Optional[float]:
        """Heading (rad, atan2(dy, dx) in the trail's frame) of the last
        `span_m` of trail: the subject's direction of travel when last seen.
        None when the trail is shorter than that."""
        if len(self.pts) < 2:
            return None
        ex, ey = self.pts[-1]
        for i in range(len(self.pts) - 2, -1, -1):
            sx, sy = self.pts[i]
            if math.hypot(ex - sx, ey - sy) >= span_m:
                return math.atan2(ey - sy, ex - sx)
        return None


def heading_to(px: float, py: float, target: Vec) -> Optional[Vec]:
    """Unit vector from (px, py) toward `target`, or None when it is under a
    metre away (too close to define a direction worth flying)."""
    dx, dy = target[0] - px, target[1] - py
    d = math.hypot(dx, dy)
    if d < 1.0:
        return None
    return (dx / d, dy / d)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------- the policy --
#
# What the follow controller does with a trail. Kept here, next to the geometry,
# so the tests exercise the same functions the flight calls.
#
# A trail steers nothing until it is MIN_TRAIL_M long. Its first breadcrumbs
# are the estimator's first positions, often from 30-45 m where depth is
# quantised to a metre and the box is 10-15 px; a direction from three of them
# can point anywhere. On citylife_redcar_trail3 a four-point trail turned the
# coast's lookout 119 deg off the car and the aircraft flew away from it.
MIN_TRAIL_M = 8.0

def direction(trail: Trail, px: float, py: float,
              lookahead_m: float) -> Optional[Vec]:
    """Unit horizontal direction to fly: toward the carrot while there is
    trail ahead; at or past the trail's end (under a metre of it left, measured
    ALONG the trail) the subject's last direction of travel. None when there is
    no usable trail - the caller keeps flying along the nose.

    "At the end" used to mean "the carrot is within a metre", and past the end
    the carrot IS the end: more than a metre beyond it, this pointed straight
    back at the last sighting, so a search creep that was meant to carry on
    down the street the car took oscillated on the spot instead (found in
    review, 2026-09-24)."""
    if len(trail) < 2 or trail.length() < MIN_TRAIL_M:
        return None
    if trail.remaining(px, py) < 1.0:
        h = trail.end_heading()
        if h is None:
            return None
        return (math.cos(h), math.sin(h))
    c = trail.carrot(px, py, lookahead_m)
    u = heading_to(px, py, c)
    if u is None:
        h = trail.end_heading()
        if h is None:
            return None
        u = (math.cos(h), math.sin(h))
    return u


def lookout(trail: Trail, px: float, py: float, yaw: float,
            ahead_m: float = 8.0) -> Optional[float]:
    """Bearing (rad, relative to the nose) to point the camera while the
    subject is out of sight: a little past where it was last seen, along the
    direction it was going. That is where a car that turned a corner reappears,
    so the search sweep is centred there rather than wherever the nose was."""
    end = trail.end()
    if end is None or len(trail) < 2 or trail.length() < MIN_TRAIL_M:
        return None
    h = trail.end_heading()
    lx, ly = end
    if h is not None:
        lx, ly = lx + ahead_m * math.cos(h), ly + ahead_m * math.sin(h)
        # Within 3 m of the look point, or anywhere past it along the
        # subject's direction: look that way. Aiming at the point itself from
        # beyond it swung the nose round to look BACKWARD while the creep
        # carried the aircraft on (found in review, 2026-09-24).
        if (px - lx) * math.cos(h) + (py - ly) * math.sin(h) > -3.0:
            return _wrap(h - yaw)
    dx, dy = lx - px, ly - py
    if math.hypot(dx, dy) < 3.0:
        if h is None:
            return None
        return _wrap(h - yaw)
    return _wrap(math.atan2(dy, dx) - yaw)


def approach_speed(remaining_m: float, cap_mps: float,
                   gain: float = 0.5) -> float:
    """Speed toward the end of the trail: `gain` m/s per metre left, capped,
    so the aircraft arrives at the last sighting and stops there instead of
    overflying it."""
    return max(0.0, min(cap_mps, gain * remaining_m))
