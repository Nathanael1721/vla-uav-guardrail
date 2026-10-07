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
decides nothing, and since 2026-10-07 it measures nothing either. It reads

  - the ShieldDecision the control loop already holds (violations, repairs,
    braked, emitted_violations),
  - the Shield's own rule checker, `Shield.rule_status(state)`: each rule's
    distance, whether it is in force, binds and is breached, measured with the
    geometry the Shield enforces (the zones' margin rings, the corridor's
    segments, its obstacle distance field). The grant allows "exactly one
    rule-evaluation code path in the system" (Safety Shield page); this module
    used to keep a second one (`_poly_distance`, `_polyline_distance` and its
    own breach tests), and a parity test was all that held the two together,
  - what the Shield was given: the subject position (the estimator's, never
    ground truth) and whether the aircraft is off its obstacle map,
  - the controller's FenceGuard verdict (holding at a fence or obstacle, or
    routing around one, and which of the two),

and turns them into rows, a banner and a few running counts. The drawing is a
separate function so the per-tick part stays pure and testable, and so the
recorder thread, which does the drawing, never touches the Shield.

WHAT THE STATUSES MEAN (shown per rule), lowest to highest rank:

  idle    the rule is in the policy but cannot bind right now: a stand-off with
          no subject being tracked, one written for pedestrians while the
          subject is a car, a zone whose altitude band the aircraft is not
          in, or a rule outside its time window (or switched off by a
          time_window_switch). Shown, not hidden, because "the 10 m pedestrian
          rule did nothing on a car flight" is a fact a reviewer should see.
  OK      bound and satisfied with room to spare.
  NEAR    inside the rule's own soft margin (`soft_margin_m` in the policy),
          within the controller's brake distance of a no-fly zone, or off the
          obstacle map (clearance unknown, not clear).
  AVOID   the controller is steering around a no-fly zone or obstacle on its
          own (FenceGuard mirrors the policy), so the Shield does not have to.
  HOLD    the controller stopped short of a no-fly zone or obstacle.
  WATCH   the command breaks this rule and the Shield did NOT correct it: the
          rule's breach action is monitor_only (recorded, never repaired), or
          the decision carries no repair at all (a Shield-off flight's log).
  ACTING  the Shield repaired the command for this rule: the rule was violated
          by the command, or a repair operator for this kind of rule ran. Never
          shown for a monitor_only rule, and never without a repair or brake
          in the decision.
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
then a breach, then any Shield repair, then a rule broken and not corrected
(WATCH), then a controller hold, a controller detour, being off the map,
approaching a zone. "SHIELD CORRECTED COMMAND" and the "Shield corrections"
count need a repair or a brake in the decision: before 2026-10-07 (review) a
violated monitor_only rule, which the Shield flies uncorrected by design, was
shown as ACTING with that banner, and so was every violation in the log of a
flight flown with the Shield off. A one-tick repair at 10 Hz is
invisible in a video, so rows and banner are LATCHED for `hold_s` seconds. The
counts are not latched; they are the flight's running totals, and the P0
escape count uses guardrail/kpi.py's definition exactly.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

# Row status rank; the latch keeps the highest seen within hold_s.
SEVERITY = {"idle": 0, "ok": 1, "near": 2, "avoid": 3, "hold": 4, "watch": 5,
            "act": 6, "breach": 7, "brake": 8}
STATUS_TEXT = {"idle": "idle", "ok": "OK", "near": "NEAR", "avoid": "AVOID",
               "hold": "HOLD", "watch": "WATCH", "act": "ACTING", "breach": "BREACH",
               "brake": "BRAKE"}

# RGB per status, shared by the rows, the dots and the banner.
COLOURS = {
    "idle": (150, 150, 150),
    "ok": (70, 205, 100),
    "near": (255, 205, 60),
    "avoid": (90, 180, 255),
    "hold": (240, 70, 60),
    "watch": (190, 140, 255),
    "act": (255, 145, 30),
    "breach": (255, 60, 160),
    "brake": (230, 30, 30),
}

# Banner events, most important first. The latch compares these ranks, not the
# colours: a Shield repair (orange) must not be hidden by a controller hold
# (red) that happens to be drawn in a stronger colour.
EVENTS = ("escape", "brake", "breach", "shield", "watch", "hold", "avoid",
          "offmap", "approach")

# Default for when the zone turns NEAR. The control loop passes FenceGuard's
# own brake distance, which is where the aircraft starts slowing for a zone.
FENCE_NEAR_M = 12.0


class _Origin:
    """A position for reading the rules' geometry before the first tick."""
    x = 0.0
    y = 0.0
    up = 0.0


_ORIGIN = _Origin()

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
    "SoftRelax": "soft rule given up",
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

# Every keep-out zone class is drawn and judged as a zone; both corridor
# classes as a corridor. The Shield enforces circle_fence (a derived 32-gon)
# and dynamic_nfz (it can move) exactly as it enforces polygon_fence.
ZONE_KINDS = ("polygon_fence", "circle_fence", "dynamic_nfz")
TUBE_KINDS = ("corridor", "corridor_swap")


def _family(kind: str) -> str:
    """polygon_fence for any zone class, corridor for either corridor class:
    the kind the repair operators are filed under (OPERATOR_KIND)."""
    return ("polygon_fence" if kind in ZONE_KINDS
            else "corridor" if kind in TUBE_KINDS else kind)


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
    elif kind in ZONE_KINDS:
        label, limit = f"NFZ {c.id}", "keep out"
    elif kind in TUBE_KINDS:
        label, limit = f"Corridor {c.id}", f"+/-{_num(c.width_m / 2.0)} m"
    elif kind == "time_window_switch":
        label, limit = f"Switch {c.target_id}", "on" if c.active else "off"
    else:
        label, limit = c.id, kind
    return RuleView(id=c.id, kind=kind, label=label, limit=limit,
                    priority=getattr(c, "priority", "P0"), soft_m=soft)


class PolicyIndicator:
    """Per-tick policy state for the HUD. Call `update()` once per control tick
    with what the loop already has; hand the returned dict to the recorder.

    `shield` is the flight's own Shield, whose rule checker the panel reads
    (and whose hot-applied rules it shows as they arrive). Without one the
    indicator builds a Shield of its own over `policy`, with no escalation
    FSM and no obstacle map - the rule geometry is still the Shield's, and an
    offline caller passes the clearance it measured (`clearance_m`)."""

    def __init__(self, policy, hold_s: float = 1.5, trail_n: int = 400,
                 base_map=None, fence_near_m: float = FENCE_NEAR_M, shield=None):
        if shield is None:
            # Imported here, not at the top: the drawing half of this module
            # (OverlayCache, draw_*) runs on the recorder thread and needs only
            # PIL.
            from guardrail.shield import Shield
            shield = Shield(policy, escalation=False)
        self.shield = shield
        self.policy = shield.policy
        self.hold_s = float(hold_s)
        self.fence_near_m = float(fence_near_m)
        self._gen = None
        self.fences: list = []
        self.corridors: list = []
        first = self.shield.rule_status(_ORIGIN, None)
        self._sync(first)
        self._map_rules(first)
        self._latch: dict[tuple, tuple[str, float, str]] = {}
        self._banner: tuple[int, str, str, float] | None = None
        self.counts = {"ticks": 0, "repaired": 0, "braked": 0, "held": 0,
                       "avoided": 0, "p0_escapes": 0}
        self.banner_ticks = {k: 0 for k in SEVERITY if k != "idle"}
        self.trail: deque = deque(maxlen=trail_n)
        self.base_map = base_map          # see render_base_map()

    def _sync(self, status: list[dict]) -> None:
        """Rows follow the Shield's rule set: a hot-applied dynamic zone, switch
        or swap gets a row at the generation it arrives, an expired zone loses
        it. The rules are taken from the SAME rule_status snapshot as the row
        values (each row carries its `rule`), so an event landing on the REST
        thread mid-update cannot pair a row with another rule's status."""
        cons = [r["rule"] for r in status]
        key = tuple(id(c) for c in cons)
        if self._gen == key:
            return
        self._gen = key
        self.rules = [rule_view(c) for c in cons]
        # Rows are keyed by POSITION (and id), not id alone: a policy with a
        # reused id (a copy-pasted block) would otherwise pair a row with the
        # wrong rule and raise inside the control loop. Ids are still shown.
        self._pairs = list(zip(self.rules, cons))
        self._by_id = {c.id: c for c in cons}
        self._prio = {c.id: getattr(c, "priority", "P0") for c in cons}

    def _map_rules(self, status: list[dict]) -> None:
        """The zones and corridors for the inset map, as the Shield holds them
        now (a moving zone where it is this tick)."""
        self.fences = [(r["id"], r["vertices"], r["margin_m"]) for r in status
                       if r["type"] in ZONE_KINDS and r["in_force"]]
        self.corridors = [(r["id"], r["centerline"], r["width_m"]) for r in status
                          if r["type"] in TUBE_KINDS and r["in_force"]]

    def _p(self, rid: str) -> str:
        # An unknown rule is treated as P0, as guardrail/kpi.py does.
        return self._prio.get(rid, "P0")

    # ------------------------------------------------------------------ #

    def _latched(self, rid: tuple, status: str, value: str, t: float) -> tuple[str, str]:
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
        ops = [r.operator for r in decision.repairs]
        acted_kinds = {OPERATOR_KIND[o] for o in ops if o in OPERATOR_KIND}
        # The Shield's own reading of every rule at this position (see the
        # module docstring). The subject is the one this loop was given.
        status = self.shield.rule_status(
            state, decision.emitted, subject=subject_xy, subject_class=subject_class,
            use_live_subject=False, clearance_m=clearance_m)
        self._sync(status)
        self._map_rules(status)
        braked = bool(decision.braked)
        # Which violations the Shield corrected and which it did not. A
        # monitor_only rule is never corrected (its row's effective action,
        # read from the Shield); a decision with no repair and no brake
        # corrected nothing at all.
        monitored = {r["id"] for r in status if r["effective_action"] == "monitor_only"}
        acted = bool(decision.repairs) or braked
        broken = {v.rule_id for v in decision.violations}
        watch = broken if not acted else broken & monitored
        viol = broken - watch
        holding = fence_mode == "hold"
        skirting = fence_mode == "skirt"
        # guardrail/kpi.py: an escape is a tick whose RAW command violated a P0
        # rule and whose SENT command still violates one.
        p0_raw = any(self._p(v.rule_id) == "P0" for v in decision.violations)
        escaped_ids = sorted({v.rule_id for v in decision.emitted_violations
                              if self._p(v.rule_id) == "P0"})
        escaped = p0_raw and bool(escaped_ids)

        self.counts["ticks"] += 1
        if acted:
            self.counts["repaired"] += 1
        if braked:
            self.counts["braked"] += 1
        if holding:
            self.counts["held"] += 1
        if skirting:
            self.counts["avoided"] += 1
        if escaped:
            self.counts["p0_escapes"] += 1

        # Zones as the Shield enforces them (rule_status): the distance to the
        # margin ring, and only inside the zone's altitude band and window.
        fence_d = {}
        for r in status:
            if r["type"] in ZONE_KINDS and r["in_force"]:
                fence_d[r["id"]] = (r["distance_m"], r["inside"] or r["in_margin"],
                                    r["in_band"], r["inside"])
        live = [k for k in fence_d if fence_d[k][2]]
        nearest_fence = min(live, key=lambda k: fence_d[k][0]) if live else None

        rows, breaches = [], []
        for k, (rv, rule) in enumerate(self._pairs):
            st = status[k]
            state_, value, breach = "ok", "", False
            bound = True
            fam = _family(rv.kind)
            if not st["in_force"]:
                # Outside its time window, or held off by a switch / replaced
                # by a swap: the Shield does not enforce it right now.
                state_, value, bound = "idle", "not in force", False
            elif rv.kind == "subject_standoff":
                if subject_xy is None:
                    state_, value, bound = "idle", "no target", False
                elif st["value"] is None:
                    # The rule does not bind this subject (SubjectStandoff.binds).
                    state_, value, bound = "idle", f"n/a ({subject_class})", False
                else:
                    sep = st["value"]
                    value = f"{sep:.0f} m" if sep >= 10 else f"{sep:.1f} m"
                    breach = st["breach"]
                    if st["distance_m"] < rv.soft_m:
                        state_ = "near"
            elif rv.kind == "obstacle_clearance":
                if off_map:
                    # The clearance rule cannot see anything here: unknown,
                    # which is not the same as clear.
                    state_, value = "near", "off map"
                elif st["value"] is None:
                    state_, value, bound = "idle", "no map", False
                else:
                    value = f"{st['value']:.1f} m"
                    breach = st["breach"]
                    if st["distance_m"] < rv.soft_m:
                        state_ = "near"
                if bound and fence_cause == "obstacle" and (holding or skirting):
                    state_ = "hold" if holding else "avoid"
            elif rv.kind == "altitude_envelope":
                value = f"{state.up:.1f} m"
                breach = st["breach"]
                if st["distance_m"] < max(rv.soft_m, 1.0):
                    state_ = "near"
            elif rv.kind == "kinematic_envelope":
                value = f"{st['value']:.1f} m/s"
                breach = st["breach"]
            elif rv.kind in ZONE_KINDS:
                d, in_ring, in_band, inside = fence_d.get(
                    rv.id, (float("inf"), False, False, False))
                if not in_band:
                    state_, bound = "idle", False
                    value = "above zone" if state.up > rule.altitude_ceiling_m else "below zone"
                else:
                    # INSIDE only inside the polygon itself, which is what
                    # nfz_entered in metrics.json measures; IN MARGIN for the
                    # ring the Shield also keeps the aircraft out of.
                    value = ("INSIDE" if inside else "IN MARGIN" if in_ring
                             else f"{d:.0f} m away")
                    breach = st["breach"]
                    if d < self.fence_near_m:
                        state_ = "near"
                    if rv.id == nearest_fence and fence_cause == "fence" and (holding or skirting):
                        state_ = "hold" if holding else "avoid"
            elif rv.kind in TUBE_KINDS:
                value = f"{st['value']:.1f} m off"
                breach = st["breach"]
                if st["distance_m"] < max(rv.soft_m, 1.0):
                    state_ = "near"
            elif rv.kind == "time_window_switch":
                state_, value, bound = "idle", "event", False
            # The Shield's work for this rule: violated by the command, or a
            # repair operator of this rule's kind ran (a building push made
            # while repairing a stand-off is still the clearance rule acting).
            # For zones only the live nearest one is credited, for stand-offs
            # only a binding one. A rule the command breaks and the Shield did
            # not correct is WATCH, and a monitor_only rule is never ACTING.
            if bound and rv.id in watch:
                state_ = "watch"
            elif bound and st["effective_action"] != "monitor_only" and (
                    rv.id in viol or (fam in acted_kinds and (
                        fam != "polygon_fence" or rv.id == nearest_fence))):
                state_ = "act"
            if bound and (breach or rv.id in escaped_ids):
                state_ = "breach"
                breaches.append(rv)
            if braked and rv.id in viol:
                state_ = "brake"
            state_, value = self._latched((k, rv.id), state_, value, t)
            rows.append({"id": rv.id, "label": rv.label, "limit": rv.limit,
                         "value": value, "status": state_, "priority": rv.priority})

        self._update_banner(t, viol, ops, braked, escaped_ids if escaped else [],
                            breaches, holding, skirting, fence_cause, off_map,
                            fence_d, nearest_fence, watch)

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
                       nearest_fence, watch=frozenset()) -> None:
        """Pick this tick's most important event and latch it for hold_s.
        `viol`: the violated rules the Shield corrected; `watch`: the ones the
        command breaks and the Shield did not correct."""
        def kind_of(rid):
            return _family(getattr(self._by_id.get(rid), "type", ""))

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
            fam = _family(rv.kind)
            how = ("MONITORED, NOT CORRECTED" if rv.id in watch
                   else "SHIELD CORRECTING" if (rv.id in viol or fam in
                                                {OPERATOR_KIND.get(o) for o in ops})
                   else "RECOVERING")
            in_poly = fam == "polygon_fence" and fence_d.get(rv.id, (0, 0, 0, False))[3]
            what = {"polygon_fence": (f"INSIDE NO-FLY ZONE {rv.id}" if in_poly
                                      else f"INSIDE NO-FLY ZONE MARGIN {rv.id}"),
                    "obstacle_clearance": "CLOSER THAN CLEARANCE TO OBSTACLE",
                    "subject_standoff": "CLOSER THAN STAND-OFF TO TARGET",
                    "altitude_envelope": "OUTSIDE ALTITUDE BAND",
                    "corridor": f"OUTSIDE CORRIDOR {rv.id}",
                    "kinematic_envelope": "OVER SPEED LIMIT"}.get(fam, rv.id)
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
        elif watch:
            cand = ("watch", "watch",
                    f"RULE BROKEN - MONITORED, NOT CORRECTED ({', '.join(sorted(watch))})")
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
