"""On-screen policy indicators: the rules in force, how close the aircraft is to
each limit, and a banner whenever the Safety Shield acts.

Asked for at the 2026-09-30 progress meeting: a viewer of a demo video could
not tell which policy was being enforced, or how. The old HUD had one line -
"GUARDRAIL: clear", "GUARDRAIL: correcting" on a tick the Shield acted, or a
hold line - and never said which rule, why, or how close the aircraft was to
any limit; a one-tick correction was gone before a viewer could read it.

The grant's Safety Shield already keeps a per-decision audit trail
(guardrail/audit.py: what was violated, which repair operator ran, whether it
braked). This module is that same record made visible, tick by tick. It
decides nothing. It reads

  - the ShieldDecision the control loop already holds (violations, repairs,
    braked, emitted_violations),
  - what the Shield itself measured: the subject position it was given (the
    estimator's, never ground truth), its obstacle distance field and whether
    the aircraft is off that field,
  - the controller's FenceGuard verdict (holding at a fence or obstacle, or
    routing around one, and which of the two),

and turns them into rows, a banner and a few running counts. The drawing is a
separate function so the per-tick part stays pure and testable, and so the
recorder thread, which does the drawing, never touches the Shield.

WHAT THE STATUSES MEAN (shown per rule), lowest to highest rank:

  idle    the rule is in the policy but cannot bind right now: a stand-off with
          no subject being tracked, one written for pedestrians while the
          subject is a car, or a zone whose altitude band the aircraft is not
          in. Shown, not hidden, because "the 10 m pedestrian rule did nothing
          on a car flight" is a fact a reviewer should see.
  OK      bound and satisfied with room to spare.
  NEAR    inside the rule's own soft margin (`soft_margin_m` in the policy),
          within the controller's brake distance of a no-fly zone, or off the
          obstacle map (clearance unknown, not clear).
  AVOID   the controller is steering around a no-fly zone or obstacle on its
          own (FenceGuard mirrors the policy), so the Shield does not have to.
  HOLD    the controller stopped short of a no-fly zone or obstacle.
  ACTING  the Shield repaired the command for this rule: the rule was violated
          by the command, or a repair operator for this kind of rule ran.
  BREACH  the aircraft is PAST the limit right now (inside the zone or its
          margin, nearer than the clearance or stand-off, outside the band) -
          the Shield forgives that only while the motion is recovering - or a
          P0 rule is still violated by the command actually sent.
  BRAKE   the repair did not converge and the Shield stopped the aircraft.

ACTING outranks HOLD: a Shield repair during a controller hold is the more
important thing to see. AVOID and ACTING are kept apart on purpose: on the
30 Sept red-car flight citylife_redcar_id4 the Shield repaired the command on
ONE tick of 2,399 while the controller steered around mapped obstacles on 554.
On those flights most of the policy's work is done by a hand-written controller
that knows the rules (FenceGuard, our addition - in the grant, upstream
compliance comes from the Prefix Compiler's prompt to the VLA, and the backstop
behind the Shield is ArduPilot's own GeoFence). A HUD that showed only Shield
repairs would call that flight "nothing happened".

THE BANNER shows the most important event of the last `hold_s` seconds, by
event, not by colour: a P0 still violated by the sent command, then a brake,
then a breach, then any Shield repair, then a controller hold, a controller
detour, being off the map, approaching a zone. A one-tick repair at 10 Hz is
invisible in a video, so rows and banner are LATCHED for `hold_s` seconds. The
counts are not latched; they are the flight's running totals, and the P0
escape count uses guardrail/kpi.py's definition exactly.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

# Row status rank; the latch keeps the highest seen within hold_s.
SEVERITY = {"idle": 0, "ok": 1, "near": 2, "avoid": 3, "hold": 4, "act": 5,
            "breach": 6, "brake": 7}
STATUS_TEXT = {"idle": "idle", "ok": "OK", "near": "NEAR", "avoid": "AVOID",
               "hold": "HOLD", "act": "ACTING", "breach": "BREACH",
               "brake": "BRAKE"}

# RGB per status, shared by the rows, the dots and the banner.
COLOURS = {
    "idle": (150, 150, 150),
    "ok": (70, 205, 100),
    "near": (255, 205, 60),
    "avoid": (90, 180, 255),
    "hold": (240, 70, 60),
    "act": (255, 145, 30),
    "breach": (255, 60, 160),
    "brake": (230, 30, 30),
}

# Banner events, most important first. The latch compares these ranks, not the
# colours: a Shield repair (orange) must not be hidden by a controller hold
# (red) that happens to be drawn in a stronger colour.
EVENTS = ("escape", "brake", "breach", "shield", "hold", "avoid", "offmap",
          "approach")

# Default for when the zone turns NEAR. The control loop passes FenceGuard's
# own brake distance, which is where the aircraft starts slowing for a zone.
FENCE_NEAR_M = 12.0

# Operator names in guardrail/shield.py, in words a viewer can read, and the
# kind of rule each one acts for.
OPERATOR_WORDS = {
    "SpeedClamp": "speed capped",
    "ClimbClamp": "climb rate capped",
    "YawClamp": "turn rate capped",
    "AltitudeFix": "altitude corrected",
    "ClearanceFix": "pushed away from obstacle",
    "ClearanceEscape": "recovery heading away from obstacle",
    "GeofenceEscape": "steered out of no-fly zone",
    "GeofenceSlide": "slid along no-fly zone edge",
    "StandoffRecover": "opening range to target",
    "StandoffHold": "holding stand-off from target",
    "CorridorReturn": "steered back into corridor",
    "CorridorAltitudeFix": "corridor altitude corrected",
    "Sanitise": "invalid command replaced",
    "Brake": "stopped",
}
OPERATOR_KIND = {
    "SpeedClamp": "kinematic_envelope", "ClimbClamp": "kinematic_envelope",
    "YawClamp": "kinematic_envelope", "AltitudeFix": "altitude_envelope",
    "ClearanceFix": "obstacle_clearance", "ClearanceEscape": "obstacle_clearance",
    "GeofenceEscape": "polygon_fence", "GeofenceSlide": "polygon_fence",
    "StandoffRecover": "subject_standoff", "StandoffHold": "subject_standoff",
    "CorridorReturn": "corridor", "CorridorAltitudeFix": "corridor",
}


@dataclass(frozen=True)
class RuleView:
    """One policy rule as the HUD shows it."""
    id: str
    kind: str            # the constraint's `type`
    label: str           # what it is, in words
    limit: str           # the number it holds the aircraft to
    priority: str        # P0 / P1 / P2, as in the policy
    soft_m: float        # the rule's own soft margin, 0 when it has none


def _num(v: float) -> str:
    return f"{v:g}"


def rule_view(c) -> RuleView:
    """Describe one constraint from the policy file, without interpreting it."""
    kind = getattr(c, "type", type(c).__name__)
    soft = float(getattr(c, "soft_margin_m", 0.0) or 0.0)
    if kind == "subject_standoff":
        who = ("target" if c.subject_class in ("*", "", None)
               else "person" if c.subject_class == "pedestrian" else c.subject_class)
        label, limit = f"Stand-off from {who}", f">= {_num(c.min_range_m)} m"
    elif kind == "obstacle_clearance":
        label, limit = "Obstacle clearance", f">= {_num(c.min_clearance_m)} m"
    elif kind == "altitude_envelope":
        label, limit = "Altitude band", f"{_num(c.alt_min_m)}-{_num(c.alt_max_m)} m"
    elif kind == "kinematic_envelope":
        label, limit = "Speed limit", f"<= {_num(c.speed_max_mps)} m/s"
    elif kind == "polygon_fence":
        label, limit = f"NFZ {c.id}", "keep out"
    elif kind == "corridor":
        label, limit = f"Corridor {c.id}", f"+/-{_num(c.width_m / 2.0)} m"
    else:
        label, limit = c.id, kind
    return RuleView(id=c.id, kind=kind, label=label, limit=limit,
                    priority=getattr(c, "priority", "P0"), soft_m=soft)


def _poly_distance(px: float, py: float, pts) -> tuple[float, bool]:
    """(distance to the polygon's boundary, inside?) without shapely, so the
    module stays importable where shapely is not installed."""
    inside = False
    n = len(pts)
    best = float("inf")
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        if (ay > py) != (by > py):
            xc = ax + (py - ay) * (bx - ax) / (by - ay)
            if px < xc:
                inside = not inside
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best, inside


def _polyline_distance(px: float, py: float, pts) -> float:
    best = float("inf")
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


class PolicyIndicator:
    """Per-tick policy state for the HUD. Call `update()` once per control tick
    with what the loop already has; hand the returned dict to the recorder."""

    def __init__(self, policy, hold_s: float = 1.5, trail_n: int = 400,
                 base_map=None, fence_near_m: float = FENCE_NEAR_M):
        self.policy = policy
        self.hold_s = float(hold_s)
        self.fence_near_m = float(fence_near_m)
        self.rules = [rule_view(c) for c in policy.constraints]
        # Rows are keyed by POSITION, not id: a policy with a reused id (a
        # copy-pasted block) would otherwise pair a row with the wrong rule
        # and raise inside the control loop. Ids are still shown.
        self._pairs = list(zip(self.rules, policy.constraints))
        self._by_id = {c.id: c for c in policy.constraints}
        self._prio = {c.id: getattr(c, "priority", "P0") for c in policy.constraints}
        self.fences = [(c.id, [(v.x, v.y) for v in c.vertices], float(c.margin_m))
                       for c in policy.constraints if getattr(c, "type", "") == "polygon_fence"]
        self.corridors = [(c.id, [(p.x, p.y) for p in c.centerline], float(c.width_m))
                          for c in policy.constraints if getattr(c, "type", "") == "corridor"]
        self._latch: dict[int, tuple[str, float, str]] = {}
        self._banner: tuple[int, str, str, float] | None = None
        self.counts = {"ticks": 0, "repaired": 0, "braked": 0, "held": 0,
                       "avoided": 0, "p0_escapes": 0}
        self.banner_ticks = {k: 0 for k in SEVERITY if k != "idle"}
        self.trail: deque = deque(maxlen=trail_n)
        self.base_map = base_map          # see render_base_map()

    def _p(self, rid: str) -> str:
        # An unknown rule is treated as P0, as guardrail/kpi.py does.
        return self._prio.get(rid, "P0")

    # ------------------------------------------------------------------ #

    def _latched(self, rid: int, status: str, value: str, t: float) -> tuple[str, str]:
        """Keep the most severe status seen within hold_s, with the value read
        at that moment, so an ACTING row does not show a value from after the
        repair already took effect."""
        cur = self._latch.get(rid)
        if (cur is None or t >= cur[1]
                or SEVERITY[status] >= SEVERITY[cur[0]]):
            until = t + self.hold_s if SEVERITY[status] >= SEVERITY["near"] else t
            self._latch[rid] = (status, until, value)
            return status, value
        return cur[0], cur[2]

    def update(self, t: float, decision, state, *, subject_xy=None,
               subject_class: str | None = None, clearance_m: float | None = None,
               off_map: bool = False, fence_cause: str | None = None,
               fence_mode: str | None = None, est_xy=None,
               yaw_rad: float | None = None) -> dict:
        """One tick. `decision` is the ShieldDecision for this tick; `state` the
        aircraft State (x north, y east, up); `fence_mode` the controller's
        "clear" / "near" / "skirt" / "hold" and `fence_cause` which hazard set
        it (FenceGuard.last_cause; None = not known). Returns the HUD snapshot."""
        viol = {v.rule_id for v in decision.violations}
        ops = [r.operator for r in decision.repairs]
        acted_kinds = {OPERATOR_KIND[o] for o in ops if o in OPERATOR_KIND}
        braked = bool(decision.braked)
        holding = fence_mode == "hold"
        skirting = fence_mode == "skirt"
        # guardrail/kpi.py: an escape is a tick whose RAW command violated a P0
        # rule and whose SENT command still violates one.
        p0_raw = any(self._p(v.rule_id) == "P0" for v in decision.violations)
        escaped_ids = sorted({v.rule_id for v in decision.emitted_violations
                              if self._p(v.rule_id) == "P0"})
        escaped = p0_raw and bool(escaped_ids)

        self.counts["ticks"] += 1
        if decision.violations:
            self.counts["repaired"] += 1
        if braked:
            self.counts["braked"] += 1
        if holding:
            self.counts["held"] += 1
        if skirting:
            self.counts["avoided"] += 1
        if escaped:
            self.counts["p0_escapes"] += 1

        # Zones as the Shield enforces them: the polygon plus its margin, and
        # only inside the zone's altitude band.
        fence_d = {}
        for fid, pts, margin in self.fences:
            rule = self._by_id[fid]
            d, inside = _poly_distance(state.x, state.y, pts)
            in_band = rule.altitude_floor_m <= state.up <= rule.altitude_ceiling_m
            in_ring = inside or d <= margin
            fence_d[fid] = (0.0 if in_ring else d - margin, in_ring, in_band, inside)
        live = [k for k in fence_d if fence_d[k][2]]
        nearest_fence = min(live, key=lambda k: fence_d[k][0]) if live else None

        rows, breaches = [], []
        for k, (rv, rule) in enumerate(self._pairs):
            status, value, breach = "ok", "", False
            bound = True
            if rv.kind == "subject_standoff":
                if subject_xy is None:
                    status, value, bound = "idle", "no target", False
                elif not rule.binds(subject_class):
                    # Same test the Shield uses (SubjectStandoff.binds).
                    status, value, bound = "idle", f"n/a ({subject_class})", False
                else:
                    sep = math.hypot(state.x - subject_xy[0], state.y - subject_xy[1])
                    value = f"{sep:.0f} m" if sep >= 10 else f"{sep:.1f} m"
                    breach = sep < rule.min_range_m
                    if sep - rule.min_range_m < rv.soft_m:
                        status = "near"
            elif rv.kind == "obstacle_clearance":
                if off_map:
                    # The clearance rule cannot see anything here: unknown,
                    # which is not the same as clear.
                    status, value = "near", "off map"
                elif clearance_m is None or not math.isfinite(clearance_m):
                    status, value, bound = "idle", "no map", False
                else:
                    value = f"{clearance_m:.1f} m"
                    breach = clearance_m < rule.min_clearance_m
                    if clearance_m - rule.min_clearance_m < rv.soft_m:
                        status = "near"
                if bound and fence_cause == "obstacle" and (holding or skirting):
                    status = "hold" if holding else "avoid"
            elif rv.kind == "altitude_envelope":
                value = f"{state.up:.1f} m"
                room = min(state.up - rule.alt_min_m, rule.alt_max_m - state.up)
                breach = room < 0
                if room < max(rv.soft_m, 1.0):
                    status = "near"
            elif rv.kind == "kinematic_envelope":
                e = decision.emitted
                spd = math.hypot(e.vx, e.vy)
                value = f"{spd:.1f} m/s"
                breach = spd > rule.speed_max_mps + 1e-6
            elif rv.kind == "polygon_fence":
                d, in_ring, in_band, inside = fence_d.get(
                    rv.id, (float("inf"), False, False, False))
                if not in_band:
                    status, bound = "idle", False
                    value = "above zone" if state.up > rule.altitude_ceiling_m else "below zone"
                else:
                    # INSIDE only inside the polygon itself, which is what
                    # nfz_entered in metrics.json measures; IN MARGIN for the
                    # ring the Shield also keeps the aircraft out of.
                    value = ("INSIDE" if inside else "IN MARGIN" if in_ring
                             else f"{d:.0f} m away")
                    breach = in_ring
                    if d < self.fence_near_m:
                        status = "near"
                    if rv.id == nearest_fence and fence_cause == "fence" and (holding or skirting):
                        status = "hold" if holding else "avoid"
            elif rv.kind == "corridor":
                off = _polyline_distance(state.x, state.y,
                                         [(p.x, p.y) for p in rule.centerline])
                value = f"{off:.1f} m off"
                breach = (off > rule.width_m / 2.0
                          or not rule.altitude_floor_m <= state.up <= rule.altitude_ceiling_m)
                if rule.width_m / 2.0 - off < max(rv.soft_m, 1.0):
                    status = "near"
            # The Shield's work for this rule: violated by the command, or a
            # repair operator of this rule's kind ran (a building push made
            # while repairing a stand-off is still the clearance rule acting).
            # For zones only the live nearest one is credited, for stand-offs
            # only a binding one.
            if bound and (rv.id in viol or (rv.kind in acted_kinds and (
                    rv.kind != "polygon_fence" or rv.id == nearest_fence))):
                status = "act"
            if bound and (breach or rv.id in escaped_ids):
                status = "breach"
                breaches.append(rv)
            if braked and rv.id in viol:
                status = "brake"
            status, value = self._latched(k, status, value, t)
            rows.append({"id": rv.id, "label": rv.label, "limit": rv.limit,
                         "value": value, "status": status, "priority": rv.priority})

        self._update_banner(t, viol, ops, braked, escaped_ids if escaped else [],
                            breaches, holding, skirting, fence_cause, off_map,
                            fence_d, nearest_fence)

        if not self.trail or math.hypot(state.x - self.trail[-1][0],
                                        state.y - self.trail[-1][1]) > 0.5:
            self.trail.append((state.x, state.y))
        level, text = ("ok", "GUARDRAIL ACTIVE - all rules satisfied")
        if self._banner is not None and t < self._banner[3]:
            level, text = self._banner[1], self._banner[2]
        self.banner_ticks[level] += 1
        return {
            "t": t,
            "policy": f"{self.policy.policy_id} v{self.policy.version}",
            "rows": rows,
            "banner": {"level": level, "text": text},
            "counts": dict(self.counts),
            "map": {"drone": (state.x, state.y), "yaw": yaw_rad,
                    "est": None if est_xy is None else (float(est_xy[0]), float(est_xy[1])),
                    "trail": list(self.trail)[-160:],
                    "fences": self.fences, "corridors": self.corridors,
                    "base": self.base_map},
        }

    def ended(self, state, text: str, yaw_rad: float | None = None,
              t: float | None = None) -> dict:
        """A snapshot for after the mission: every row idle and one plain
        banner. The vertical descent and the landing are flown outside the
        Shield and below the policy's altitude band, so the rows must not keep
        showing the last mission tick's verdicts over them. Counts unchanged."""
        return {
            "t": t,
            "policy": f"{self.policy.policy_id} v{self.policy.version}",
            "rows": [{"id": rv.id, "label": rv.label, "limit": rv.limit,
                      "value": "", "status": "idle", "priority": rv.priority}
                     for rv in self.rules],
            "banner": {"level": "idle", "text": text},
            "counts": dict(self.counts),
            "map": {"drone": (state.x, state.y), "yaw": yaw_rad, "est": None,
                    "trail": list(self.trail)[-160:], "fences": self.fences,
                    "corridors": self.corridors, "base": self.base_map},
        }

    def _update_banner(self, t, viol, ops, braked, escaped_ids, breaches,
                       holding, skirting, fence_cause, off_map, fence_d,
                       nearest_fence) -> None:
        """Pick this tick's most important event and latch it for hold_s."""
        def kind_of(rid):
            return getattr(self._by_id.get(rid), "type", "")

        # The LAST operator is the one whose result was flown: repairs are
        # appended in the order they ran, and a fall-back (ClearanceEscape,
        # Brake) comes after the operators it overrode.
        # ...and only an operator acting for one of the VIOLATED rules' kinds,
        # so the words and the rule ids on the banner describe the same thing.
        vkinds = {kind_of(r) for r in viol}
        last = next((o for o in reversed(ops)
                     if o in OPERATOR_WORDS and OPERATOR_KIND.get(o) in vkinds), None)
        words = OPERATOR_WORDS.get(last, "corrected")
        cand = None
        if escaped_ids:
            cand = ("escape", "brake",
                    f"P0 RULE STILL BROKEN AFTER SHIELD ({', '.join(escaped_ids)})")
        elif braked:
            cand = ("brake", "brake",
                    f"SHIELD BRAKE - aircraft stopped ({', '.join(sorted(viol))})")
        elif breaches:
            rv = breaches[0]
            how = "SHIELD CORRECTING" if (rv.id in viol or rv.kind in
                                          {OPERATOR_KIND.get(o) for o in ops}) else "RECOVERING"
            in_poly = rv.kind == "polygon_fence" and fence_d.get(rv.id, (0, 0, 0, False))[3]
            what = {"polygon_fence": (f"INSIDE NO-FLY ZONE {rv.id}" if in_poly
                                      else f"INSIDE NO-FLY ZONE MARGIN {rv.id}"),
                    "obstacle_clearance": "CLOSER THAN CLEARANCE TO OBSTACLE",
                    "subject_standoff": "CLOSER THAN STAND-OFF TO TARGET",
                    "altitude_envelope": "OUTSIDE ALTITUDE BAND",
                    "corridor": f"OUTSIDE CORRIDOR {rv.id}",
                    "kinematic_envelope": "OVER SPEED LIMIT"}.get(rv.kind, rv.id)
            cand = ("breach", "breach", f"{what} - {how}")
        elif viol:
            rids = sorted(viol)
            kinds = {kind_of(r) for r in rids}
            if kinds == {"subject_standoff"}:
                text = f"TOO CLOSE TO TARGET - SHIELD CORRECTED COMMAND ({words})"
            elif "polygon_fence" in kinds:
                text = f"NO-FLY ZONE - SHIELD CORRECTED COMMAND ({words})"
            else:
                text = f"SHIELD CORRECTED COMMAND - {words} ({', '.join(rids)})"
            cand = ("shield", "act", text)
        elif holding:
            text = {"fence": f"NO-FLY ZONE AHEAD - HOLDING AT THE BOUNDARY ({nearest_fence})",
                    "obstacle": "OBSTACLE AHEAD - HOLDING"}.get(
                        fence_cause, "CONTROLLER HOLDING (cause not logged)")
            cand = ("hold", "hold", text)
        elif skirting:
            text = {"fence": f"NO-FLY ZONE AHEAD - ROUTING AROUND IT ({nearest_fence})",
                    "obstacle": "OBSTACLE CLOSE - STEERING AROUND IT"}.get(
                        fence_cause, "CONTROLLER STEERING AROUND A HAZARD (cause not logged)")
            cand = ("avoid", "avoid", text)
        elif off_map:
            cand = ("offmap", "near", "OFF THE OBSTACLE MAP - CLEARANCE UNKNOWN")
        elif nearest_fence is not None and fence_d[nearest_fence][0] < self.fence_near_m:
            cand = ("approach", "near",
                    f"APPROACHING NO-FLY ZONE {nearest_fence} - {fence_d[nearest_fence][0]:.0f} m")
        if cand is None:
            return
        rank = EVENTS.index(cand[0])
        cur = self._banner
        if cur is None or t >= cur[3] or rank <= cur[0]:
            self._banner = (rank, cand[1], cand[2], t + self.hold_s)


# ---------------------------------------------------------------------- #
# Drawing (runs on the recorder thread)
# ---------------------------------------------------------------------- #

_FONTS: dict = {}


def font(size: int, bold: bool = False):
    """A legible monospace font, cached. Falls back to PIL's scalable default."""
    from PIL import ImageFont
    key = (size, bold)
    if key not in _FONTS:
        f = None
        for name in (("consolab.ttf", "arialbd.ttf", "DejaVuSansMono-Bold.ttf") if bold
                     else ("consola.ttf", "arial.ttf", "DejaVuSansMono.ttf")):
            try:
                f = ImageFont.truetype(name, size)
                break
            except OSError:
                continue
        if f is None:
            try:
                f = ImageFont.load_default(size=size)
            except TypeError:                      # Pillow < 10.1
                f = ImageFont.load_default()
        _FONTS[key] = f
    return _FONTS[key]


def render_base_map(occ_map: dict | None, street_mask: dict | None = None):
    """Pre-render the obstacle map once, north up, one pixel per cell.

    Buildings and mapped furniture dark, roads mid-grey, the rest light.
    Cell (i, j) is centred at (ox + i*res, oy + j*res); the image row for i is
    flipped so north is up. Returns (PIL image, meta) or None.
    """
    if not occ_map:
        return None
    import numpy as np
    from PIL import Image
    occ = np.asarray(occ_map["occ"]).astype(bool)
    n, m = occ.shape
    rgb = np.full((n, m, 3), 95, dtype=np.uint8)
    if street_mask is not None and np.asarray(street_mask["street"]).shape == occ.shape:
        rgb[np.asarray(street_mask["street"]).astype(bool)] = (150, 150, 150)
    rgb[occ] = (35, 35, 42)
    img = Image.fromarray(rgb[::-1].copy(), "RGB")
    return img, {"res": float(occ_map["res"]), "ox": float(occ_map["ox"]),
                 "oy": float(occ_map["oy"]), "n": n, "m": m}


def draw_minimap(im, snap_map: dict, x0: int, y0: int, size: int,
                 half_m: float = 40.0, scale: float = 1.0) -> None:
    """Top-down inset, north up, centred on the aircraft.

    Drawn on its own tile and pasted, so the trail, a fence or the target dot
    beyond the window is clipped at the map's edge instead of being painted
    across the camera view.
    """
    from PIL import Image, ImageDraw
    cx, cy = snap_map["drone"]

    def to_px(x, y):
        return ((y - (cy - half_m)) / (2 * half_m) * size,
                ((cx + half_m) - x) / (2 * half_m) * size)

    base = snap_map.get("base")
    if base is not None:
        img, meta = base
        res, ox, oy, n = meta["res"], meta["ox"], meta["oy"], meta["n"]
        # Cell centres sit on the grid points; offset half a cell to edges.
        c0 = (cy - half_m - oy) / res + 0.5
        c1 = (cy + half_m - oy) / res + 0.5
        r0 = n - ((cx + half_m - ox) / res + 0.5)
        r1 = n - ((cx - half_m - ox) / res + 0.5)
        # EXTENT takes the fractional source box as is. Rounding it to whole
        # source pixels first shifted the buildings by up to half a cell (1 m
        # on CityLife) against the zone, trail and aircraft drawn over them.
        tile = img.transform((size, size), Image.EXTENT, (c0, r0, c1, r1),
                             Image.NEAREST, fillcolor=(0, 0, 0))
    else:
        tile = Image.new("RGB", (size, size), (40, 40, 40))
    d = ImageDraw.Draw(tile, "RGBA")

    for _fid, pts, margin in snap_map.get("fences", []):
        poly = [to_px(x, y) for x, y in pts]
        d.polygon(poly, fill=(230, 40, 40, 120), outline=(255, 60, 60, 255))
    for _cid, pts, width in snap_map.get("corridors", []):
        line = [to_px(x, y) for x, y in pts]
        d.line(line, fill=(80, 170, 255, 120),
               width=max(2, int(width / (2 * half_m) * size)))
    tr = snap_map.get("trail") or []
    if len(tr) > 1:
        d.line([to_px(x, y) for x, y in tr], fill=(120, 220, 255, 255),
               width=max(1, int(2 * scale)))
    est = snap_map.get("est")
    if est is not None:
        ex, ey = to_px(*est)
        r = 5 * scale
        d.ellipse([ex - r, ey - r, ex + r, ey + r], fill=(255, 60, 60, 255),
                  outline=(255, 255, 255, 255))
    px, py = to_px(cx, cy)
    yaw = snap_map.get("yaw")
    r = 8 * scale
    if yaw is not None:
        fx, fy = math.sin(yaw), -math.cos(yaw)      # north up, east right
        lx, ly = -fy, fx
        d.polygon([(px + fx * r * 1.4, py + fy * r * 1.4),
                   (px - fx * r + lx * r * 0.8, py - fy * r + ly * r * 0.8),
                   (px - fx * r - lx * r * 0.8, py - fy * r - ly * r * 0.8)],
                  fill=(255, 255, 255, 255), outline=(0, 0, 0, 255))
    else:
        d.ellipse([px - r / 2, py - r / 2, px + r / 2, py + r / 2],
                  fill=(255, 255, 255, 255))
    d.rectangle([0, 0, size - 1, size - 1], outline=(255, 255, 255, 160), width=1)
    f = font(int(12 * scale))
    d.rectangle([0, 0, size, 18 * scale], fill=(0, 0, 0, 140))
    d.text((4, 2), f"MAP  north up  {2 * half_m:.0f} m", font=f, fill=(255, 255, 255, 235))
    legend = "red dot = target estimate" if est is not None else "no target estimate"
    d.rectangle([0, size - 18 * scale, size, size], fill=(0, 0, 0, 140))
    d.text((4, size - 16 * scale), legend, font=f, fill=(255, 200, 200, 235))
    im.paste(tile, (x0, y0))


class OverlayCache:
    """Draw the status box and the policy overlay at most `hz` times a second,
    and paste the drawn tiles onto every frame in between.

    Drawing it on every recorder frame cost the detector. Flown A/B on
    2026-10-03 under the same conditions (10 m cruise, the same machine load):
    the detector ran at 2.89 Hz during the mission with the old one-line HUD
    and 2.43 Hz with the overlay drawn at the recorder's 20 Hz
    (citylife_redcar_alt10_nohud vs citylife_redcar_alt10). The recorder
    thread's PIL calls hold the GIL the detector thread needs between its GPU
    waits. The panel's content changes at the control rate at most, and the
    banner is latched for 1.5 s, so 5 Hz loses nothing a viewer can read.

    A banner change, or a new frame size, redraws at once, so an event appears
    on the first frame after it. Time is the SNAPSHOT's ("t"), not the wall
    clock, so an offline re-render that runs faster than real time still
    redraws per flight-time period. Pasting the cached tiles costs well under
    a millisecond. Single caller (the recorder thread) - not thread-safe.
    """

    def __init__(self, hz: float = 5.0):
        self.period = 1.0 / float(hz)
        self.renders = 0
        self._key = None
        self._t = None
        self._tiles: list = []

    def draw(self, im, snap: dict, lines: list[str] | None = None) -> None:
        from PIL import Image
        t = snap.get("t")
        key = (im.size, snap["banner"]["level"], snap["banner"]["text"])
        stale = (key != self._key or t is None or self._t is None
                 or t - self._t >= self.period or t < self._t)
        if stale:
            layer = Image.new("RGBA", im.size, (0, 0, 0, 0))
            if lines:
                draw_status_box(layer, lines)
            draw_policy_overlay(layer, snap)
            W, H = im.size
            # Three regions: the status box, banner + rule panel, the map.
            regions = ((0, 0, int(W * 0.30), int(H * 0.25)),
                       (int(W * 0.22), 0, W, int(H * 0.55)),
                       (int(W * 0.70), int(H * 0.55), W, H))
            tiles = []
            for x0, y0, x1, y1 in regions:
                sub = layer.crop((x0, y0, x1, y1))
                bb = sub.getchannel("A").getbbox()
                if bb:
                    tile = sub.crop(bb)
                    tiles.append((tile.convert("RGB"), tile.getchannel("A"),
                                  (x0 + bb[0], y0 + bb[1])))
            self._tiles, self._key, self._t = tiles, key, t
            self.renders += 1
        for rgb, alpha, xy in self._tiles:
            im.paste(rgb, xy, alpha)


def draw_status_box(im, lines: list[str]) -> None:
    """The flight lines (time/altitude, bearing/speed, separation, tracking
    state), top left, at a size a video viewer can read. The last line is the
    tracking state and is coloured: green when locked, amber otherwise."""
    from PIL import ImageDraw
    d = ImageDraw.Draw(im, "RGBA")
    s = im.size[0] / 1280.0
    f = font(int(15 * s))
    d.rounded_rectangle([8 * s, 8 * s, 330 * s, 8 * s + 21 * s * len(lines) + 10 * s],
                        radius=6 * s, fill=(10, 10, 15, 200))
    for i, ln in enumerate(lines):
        last = i == len(lines) - 1
        d.text((16 * s, 13 * s + 21 * s * i), ln, font=f,
               fill=((120, 255, 140) if last and "LOCKED" in ln
                     else (255, 220, 120) if last else (255, 255, 255)))


def draw_policy_overlay(im, snap: dict) -> None:
    """Draw the banner, the rule panel, the counts and the map onto `im`."""
    from PIL import ImageDraw
    d = ImageDraw.Draw(im, "RGBA")
    W, H = im.size
    s = W / 1280.0
    f_row, f_bold = font(int(15 * s)), font(int(15 * s), bold=True)
    f_head, f_ban = font(int(14 * s), bold=True), font(int(20 * s), bold=True)

    # Banner, top centre.
    ban = snap["banner"]
    col = COLOURS[ban["level"]]
    tw = d.textlength(ban["text"], font=f_ban)
    bx0 = (W - tw) / 2 - 14 * s
    big = ban["level"] not in ("ok", "idle")
    pad = 8 * s if big else 5 * s
    bh = (20 * s + 2 * pad)
    d.rounded_rectangle([bx0, 10 * s, bx0 + tw + 28 * s, 10 * s + bh], radius=6 * s,
                        fill=col + ((215,) if big else (150,)))
    d.text((bx0 + 14 * s, 10 * s + pad - 1), ban["text"], font=f_ban,
           fill=(20, 20, 20) if ban["level"] in ("ok", "near", "idle") else (255, 255, 255))

    # Rule panel, top right.
    rows = snap["rows"]
    pw = 470 * s
    px0 = W - pw - 10 * s
    py0 = 10 * s + bh + 8 * s
    rh = 21 * s
    ph = (rows and len(rows) or 1) * rh + 50 * s
    d.rounded_rectangle([px0, py0, px0 + pw, py0 + ph], radius=6 * s, fill=(10, 10, 15, 175))
    d.text((px0 + 10 * s, py0 + 6 * s), f"GUARDRAIL POLICY  {snap['policy']}",
           font=f_head, fill=(255, 255, 255))
    y = py0 + 28 * s
    for r in rows:
        c = COLOURS[r["status"]]
        d.ellipse([px0 + 10 * s, y + 4 * s, px0 + 22 * s, y + 16 * s], fill=c + (255,))
        d.text((px0 + 30 * s, y), r["label"][:22], font=f_row, fill=(235, 235, 235))
        d.text((px0 + 232 * s, y), r["limit"], font=f_row, fill=(180, 180, 180))
        d.text((px0 + 318 * s, y), r["value"][:12], font=f_row, fill=(235, 235, 235))
        d.text((px0 + 404 * s, y), STATUS_TEXT[r["status"]], font=f_bold, fill=c)
        y += rh
    k = snap["counts"]
    d.text((px0 + 10 * s, y + 4 * s),
           f"Shield corrections {k['repaired']}  brakes {k['braked']}  "
           f"holds {k['held']}  P0 escapes {k['p0_escapes']}",
           font=f_row, fill=(255, 255, 255))

    # Map, bottom right.
    m = snap.get("map")
    if m is not None:
        size = int(220 * s)
        draw_minimap(im, m, int(W - size - 10 * s), int(H - size - 10 * s), size,
                     scale=s)
