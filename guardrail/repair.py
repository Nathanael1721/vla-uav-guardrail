"""
Repair operators as a pluggable library.

The grant (Safety Shield page, "Repair operators" and the Outputs table:
"Repair operator library | Python module; pluggable Protocol") describes the
repair stack as a list of operators behind one Protocol:

    class RepairOperator(Protocol):
        name: str
        def repair(self, action, violation, ir) -> RepairAttempt: ...

    # Priority 1  AltitudeClamp
    # Priority 2  LateralProjection
    # Priority 3  PathRepair     ("insert a detour waypoint in body frame")

and selects among them "first operator that produces a feasible action with
magnitude <= threshold wins", inside "a bounded loop (default 3 iterations)".

Until this file the Shield's operators were six private methods called in a
fixed order (guardrail/shield.py `_repair_chain`). They still are, and they
stay where they are: each operator below WRAPS one of them, calling it by name
on the Shield (`getattr(shield, "_repair_geofence")`), so a subclass that
overrides a `_repair_*` method (tests/test_shield.py does) is the one that
runs. The default chain calls them in exactly the old order with exactly the
old arguments, so its decisions are byte-identical; what the library adds is
a record of each call (`RepairAttempt`), a registry a caller can extend, and
two modes the grant describes that the chain did not have.

THE PROTOCOL, AS THIS PROJECT CAN HONOUR IT

    repair(action, violation, ir, *, ctx) -> RepairAttempt

The grant's three arguments are kept, positionally. `violation` is None when
the operator is run as a chain stage (it then acts on every rule of its class,
as the methods always have) and the target violation in first-feasible mode.
`ctx` (RepairContext) carries what the grant's signature leaves implicit and
every real operator needs: the vehicle state, the Shield (rule lists, the
monitor, the distance field), the raw action and its violations, and the list
the chain logs its `Repair` records into. The reference implementation
(kuanting-vla-uav-guardrail/.../safety_shield/repair.py) made the same
extension, with `state` and `cfg` as extra positional arguments.

An operator written to the grant's signature exactly - `repair(action,
violation, ir)`, no `ctx`, no `categories` - is accepted too: `register`
wraps it (`GrantOperator`; it then covers every category, and a change it
makes without logging a Repair is logged in its name). A registered
operator runs where a stack names it: under a name the stacks below already
use (replacing that stage), or listed in a Shield's `repair_order`, which
replaces the iterated loop's stack (chain mode) or the priority order
(first-feasible mode).

THE OPERATORS

    name                 wraps / does                      repair records
    KinematicClamp       Shield._repair_kinematic          SpeedClamp ClimbClamp YawClamp
    AltitudeClamp        Shield._repair_altitude           AltitudeFix
    ClearanceProjection  Shield._repair_clearance          ClearanceFix
    StandoffProjection   Shield._repair_standoff           StandoffHold StandoffRecover
    CorridorProjection   Shield._repair_corridor           CorridorReturn CorridorAltitudeFix
    LateralProjection    Shield._repair_geofence           GeofenceEscape GeofenceSlide
    PathRepair           a body-frame detour waypoint      PathRepair      (flag, default off)

`repair_attempts` in the audit record still lists the REPAIR records (the
right-hand column) as it always has; `repair_passes` lists these operators,
pass by pass, with result ok / skip / fail and a reason for every skip.

THE MODES (guardrail/shield.py, Shield(...) keyword arguments)

  repair_mode="chain"           (default) the iterated chain: KinematicClamp,
                                AltitudeClamp, then up to 3 passes of
                                Clearance, Standoff, Corridor, Lateral
                                (, PathRepair), KinematicClamp, until the
                                monitor finds nothing.
  repair_mode="first_feasible"  the grant's selection: per iteration (up to 3)
                                the earliest open violation is the target;
                                operators are tried in FIRST_FEASIBLE order,
                                the first whose action clears the target with
                                every non-recovery repair's magnitude_m within
                                theta wins. No winner ends the loop, and the
                                Shield's rescue / brake fallback decides.
  path_repair=True              PathRepair joins either mode (after
                                LateralProjection).
  repair_order=(name, ...)      the caller's stack for either mode, in
                                place of CHAIN_LOOP / FIRST_FEASIBLE.

Both new behaviours are off by default; tests/test_shieldcore_repair.py pins
that the default chain's decisions do not move.
"""
from __future__ import annotations

import inspect
import math
from dataclasses import dataclass, replace
from typing import Any, Protocol, runtime_checkable

from shapely.geometry import LineString, Point, Polygon

from .frames import to_body
from .ir import coarse_ring
from .models import Action4D, State

# What an attempt can come to. "fallback" and "relaxed" are the Shield's own
# last resorts (Brake / ClearanceEscape, SoftRelax), recorded in the same
# ledger so one list says everything the repair stack tried on a tick.
RESULTS = ("ok", "skip", "fail", "fallback", "relaxed")


@dataclass
class RepairContext:
    """What an operator may read, beyond the grant's (action, violation, ir).

    `repairs` is the list the chain logs its Repair records into: an operator
    that acts appends to it (that is how the decision's `repairs` is built).
    `open_violations` are the violations open at the start of the pass, as
    the monitor last saw them (`open_label` says when: "on the raw action" or
    "after pass k"); they only feed the reason given for a skip."""
    shield: Any
    state: State
    repairs: list
    raw: Action4D
    raw_violations: tuple = ()
    open_violations: tuple = ()
    open_label: str = "on the raw action"
    pass_no: int = 0


@dataclass
class RepairAttempt:
    """One operator, one call. `action` is the action it handed on (unchanged
    on a skip), `repairs` the Repair records it logged, `iterations` the pass
    it ran in (0 = before the iterated loop, 1..3 inside it)."""
    operator: str
    result: str
    action: Action4D
    reason: str = ""
    repairs: tuple = ()
    iterations: int = 0

    def record(self) -> dict:
        """The audit form (guardrail/audit.py `repair_passes`)."""
        out = {"operator": self.operator, "result": self.result,
               "reason": self.reason, "iterations": self.iterations}
        if self.repairs:
            mags = [r.magnitude_m for r in self.repairs if r.magnitude_m is not None]
            axes = sorted({r.axis for r in self.repairs if r.axis})
            out["repairs"] = [r.operator for r in self.repairs]
            out["magnitude_m"] = max(mags) if mags else None
            out["axis"] = "+".join(axes) if axes else None
        return out


@runtime_checkable
class RepairOperator(Protocol):
    name: str
    categories: tuple

    def repair(self, action: Action4D, violation, ir, *,
               ctx: RepairContext) -> RepairAttempt: ...


class HookOperator:
    """An operator that is one of the Shield's `_repair_*` methods.

    The method is looked up on the Shield at call time, so an override in a
    Shield subclass is honoured. It is "ok" when it logged a Repair or changed
    the action, "skip" otherwise, with the reason: the rule class is not in
    force (or cannot act: no map, no subject), no violation of its class was
    open, or one was and it changed nothing."""

    def __init__(self, name: str, hook: str, categories: tuple, *,
                 takes_state: bool = True):
        self.name = name
        self.hook = hook
        self.categories = tuple(categories)
        self.takes_state = takes_state

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.name!r} -> Shield.{self.hook})"

    def repair(self, action, violation, ir, *, ctx: RepairContext) -> RepairAttempt:
        n0 = len(ctx.repairs)
        fn = getattr(ctx.shield, self.hook)
        out = (fn(ctx.state, action, ctx.repairs) if self.takes_state
               else fn(action, ctx.repairs))
        new = tuple(ctx.repairs[n0:])
        if new or out != action:
            return RepairAttempt(self.name, "ok", out,
                                 "; ".join(r.detail for r in new), new, ctx.pass_no)
        return RepairAttempt(self.name, "skip", out, self.skip_reason(ctx), (),
                             ctx.pass_no)

    def inert(self, sh) -> str | None:
        """Why this operator could not act at all on this tick, or None."""
        return None

    def skip_reason(self, ctx: RepairContext) -> str:
        why = self.inert(ctx.shield)
        if why:
            return why
        cats = self.categories
        open_ = sorted({v.rule_id for v in ctx.open_violations if v.category in cats})
        if not open_:
            return f"no {'/'.join(cats)} violation open {ctx.open_label}"
        return (f"{', '.join(open_)} open {ctx.open_label}; the operator left the "
                f"action unchanged")


class _Kinematic(HookOperator):
    def inert(self, sh):
        return None if sh._kins else "no kinematic_envelope in force"


class _Altitude(HookOperator):
    def inert(self, sh):
        return None if sh._alts else "no altitude_envelope in force"


class _Clearance(HookOperator):
    def inert(self, sh):
        if not sh._clear:
            return "no obstacle_clearance rule in force"
        if not sh.has_obstacle_map:
            return "no obstacle map: obstacle_clearance is inert"
        return None


class _Standoff(HookOperator):
    def inert(self, sh):
        if sh.subject is None:
            return "no subject set: subject_standoff is inert"
        if not any(so.binds(sh.subject_class) for so in sh._standoffs):
            return f"no subject_standoff in force binds class {sh.subject_class!r}"
        return None


class _Corridor(HookOperator):
    def inert(self, sh):
        return None if sh._corridors else "no corridor in force"


class _Lateral(HookOperator):
    def inert(self, sh):
        return None if sh._fence_anchors(sh._ir)[0] else "no keep-out zone in force"


class PathRepair:
    """The grant's priority-3 operator: "If a predicted segment crosses a
    boundary, insert a detour waypoint in body frame. Simplified path repair -
    full path planning is out of scope for the Shield."

    Projection keeps a forecast out of a ring by removing the velocity
    component that enters it. Head-on, that is all of it: the subject of the
    `standoff-wedge` scenario sits on the route, StandoffHold removes the
    whole command, and the aircraft hovers 20 m short of a ring it never
    enters, for the rest of the mission (a legal action, never a P0 escape,
    and no progress). PathRepair steps in where the projected action has
    lost at least half of the commanded progress (chain mode), or where the
    operators before it failed (first-feasible mode):

      * a stand-off ring (the largest one binding the subject): the tangent
        point of a circle `max(soft_margin_m, CLEARANCE_M)` outside it, on
        the side nearer the commanded heading (the right, seen from the
        aircraft, on a tie); inside that circle, the tangent at the current
        range.
      * a static keep-out zone: a corner of the zone's margin ring's coarse
        ring (at most 8 vertices) pushed `CLEARANCE_M` out (mitre), seen from
        the aircraft without crossing the ring. A moving zone has no static
        detour and is left to projection.

    The detour flies toward the waypoint at the commanded speed (capped),
    with the vertical speed and yaw rate already repaired. It is taken only
    if the monitor finds NOTHING wrong with it, under every rule the chain is
    enforcing, so it cannot add an escape. The waypoint is logged in body
    frame (forward / right of the heading, frames.to_body). Its magnitude_m
    is the depth the forecast it replaced reached into the ring, the same
    "forecast it corrected" basis every lateral operator reports."""

    name = "PathRepair"
    categories = ("geofence", "standoff")
    MIN_PROGRESS = 0.5        # of the commanded horizontal speed, chain mode
    CLEARANCE_M = 1.0         # waypoint clearance outside the ring
    MIN_SPEED_MPS = 0.5       # below this there is no progress to keep

    def __repr__(self) -> str:
        return "PathRepair()"

    def repair(self, action, violation, ir, *, ctx: RepairContext) -> RepairAttempt:
        sh, st, raw = ctx.shield, ctx.state, ctx.raw

        def skip(why: str) -> RepairAttempt:
            return RepairAttempt(self.name, "skip", action, why, (), ctx.pass_no)

        spd = math.hypot(raw.vx, raw.vy)
        if spd < self.MIN_SPEED_MPS:
            return skip("no commanded horizontal motion to keep")
        ux, uy = raw.vx / spd, raw.vy / spd
        prog = action.vx * ux + action.vy * uy
        if violation is None and prog >= self.MIN_PROGRESS * spd - 1e-9:
            return skip(f"the action keeps {max(0.0, prog) / spd:.0%} of the commanded "
                        f"progress")
        if violation is not None:
            targets = [violation] if violation.category in self.categories else []
        else:
            targets = sorted((v for v in ctx.raw_violations if v.category in self.categories),
                             key=lambda v: v.predicted_at_s)
        if not targets:
            return skip("the raw forecast crosses no keep-out zone or stand-off ring")
        cap = min((k.speed_max_mps for k in sh._kins), default=4.0)
        speed = min(spd, cap)
        from .shield import Repair                    # shield imports this module
        for v in targets:
            if v.category == "standoff":
                wps, depth = self._standoff_waypoints(sh, st, raw)
            else:
                wps, depth = self._zone_waypoints(sh, st, raw, v.rule_id)
            for wx, wy in self._by_preference(st, wps, ux, uy):
                dx, dy = wx - st.x, wy - st.y
                n = math.hypot(dx, dy)
                if n < 1e-6:
                    continue
                cand = Action4D(vx=dx / n * speed, vy=dy / n * speed,
                                vz_up=action.vz_up, yaw_rate=action.yaw_rate)
                if violation is None and cand.vx * ux + cand.vy * uy <= prog + 1e-9:
                    continue                          # no better than projection
                if sh._check(st, cand):
                    continue                          # never trade one breach for another
                fwd, right, _, _ = to_body(Action4D(vx=dx, vy=dy, vz_up=0.0, yaw_rate=0.0),
                                           st.yaw_deg)
                detail = (f"{v.rule_id}: detour waypoint {fwd:+.1f} m forward, "
                          f"{right:+.1f} m right (body frame) at {speed:.2f} m/s; "
                          f"the action handed in kept {max(0.0, prog) / spd:.0%} of the "
                          f"commanded progress")
                rep = Repair(operator="PathRepair", detail=detail, magnitude_m=depth,
                             axis="lateral", recovery=False)
                ctx.repairs.append(rep)
                return RepairAttempt(self.name, "ok", cand, detail, (rep,), ctx.pass_no)
        return skip("no detour waypoint re-checks clean")

    @staticmethod
    def _by_preference(st, wps, ux, uy):
        """Waypoints nearest the commanded heading first; on a tie, the one to
        the right of it (clockwise, North towards East)."""
        def key(w):
            dx, dy = w[0] - st.x, w[1] - st.y
            n = math.hypot(dx, dy) or 1.0
            along = round((dx * ux + dy * uy) / n, 9)
            right = (dy * ux - dx * uy) / n        # > 0: clockwise of the heading
            return (-along, -right)
        return sorted(wps, key=key)

    def _standoff_waypoints(self, sh, st, raw):
        subj = sh.subject
        if subj is None:
            return [], 0.0
        binding = [so for so in sh._standoffs if so.binds(sh.subject_class)]
        if not binding:
            return [], 0.0
        so = max(binding, key=lambda s: s.min_range_m)
        sx, sy = subj
        dmin = min(math.hypot(p.x - sx, p.y - sy) for _, p in sh._predict(st, raw))
        depth = max(0.0, so.min_range_m - dmin)
        r_wp = so.min_range_m + max(self.CLEARANCE_M, float(so.soft_margin_m or 0.0))
        d = math.hypot(sx - st.x, sy - st.y)
        if d <= 1e-6:
            return [], depth                       # directly over it: StandoffRecover's
        bx, by = (sx - st.x) / d, (sy - st.y) / d
        if d > r_wp:
            a = math.asin(r_wp / d)
            reach = math.sqrt(d * d - r_wp * r_wp)
        else:
            a, reach = math.pi / 2.0, r_wp         # the tangent at the current range
        out = []
        for s in (1.0, -1.0):                      # clockwise, then anticlockwise
            c, sn = math.cos(s * a), math.sin(s * a)
            out.append((st.x + reach * (bx * c - by * sn), st.y + reach * (bx * sn + by * c)))
        return out, depth

    def _zone_waypoints(self, sh, st, raw, rule_id):
        anchors, _ = sh._fence_anchors(sh._ir)
        rec = next((r for r in anchors if r.rule.id == rule_id), None)
        if rec is None or rec.moving or rec.buffered.is_empty:
            return [], 0.0
        ring = rec.buffered
        depth = 0.0
        for _, p in sh._predict(st, raw):
            q = Point(p.x, p.y)
            if ring.contains(q):
                depth = max(depth, ring.boundary.distance(q))
        out = []
        for wx, wy in self._zone_corners(sh, rec):
            seg = LineString([(st.x, st.y), (wx, wy)])
            if seg.length < 1e-6 or ring.relate_pattern(seg, "T********"):
                continue                           # the leg itself would cross the zone
            out.append((wx, wy))
        return out, depth

    def _zone_corners(self, sh, rec):
        """A static zone's detour corners: its margin ring's coarse ring (the
        convex hull, or the oriented rectangle round it - at most 8 vertices,
        guardrail/ir.py coarse_ring) pushed CLEARANCE_M out with mitred joins.
        Built once per compiled zone and kept on the Shield. (The first
        version took every vertex of the ring's hull - 68 for a buffered
        square, each a visibility test and possibly a monitor check - and
        tripled a 50-rule filter()'s median.)"""
        cache = sh.__dict__.setdefault("_path_repair_corners", {})
        key = (rec.rule.id, id(rec.buffered))
        hit = cache.get(key)
        if hit is None or hit[0] is not rec.buffered:
            grown = Polygon(coarse_ring(rec.buffered)).buffer(self.CLEARANCE_M, join_style=2)
            hit = cache[key] = (rec.buffered,
                                [(float(x), float(y)) for x, y in list(grown.exterior.coords)[:-1]])
        return hit[1]


def default_operators() -> dict:
    """A fresh registry: every operator above, by name. Each Shield takes its
    own copy (`Shield.repair_operators`), so replacing one there changes that
    Shield only."""
    return {op.name: op for op in (
        _Kinematic("KinematicClamp", "_repair_kinematic", ("kinematic",), takes_state=False),
        _Altitude("AltitudeClamp", "_repair_altitude", ("altitude",)),
        _Clearance("ClearanceProjection", "_repair_clearance", ("clearance",)),
        _Standoff("StandoffProjection", "_repair_standoff", ("standoff",)),
        _Corridor("CorridorProjection", "_repair_corridor", ("corridor",)),
        _Lateral("LateralProjection", "_repair_geofence", ("geofence",)),
        PathRepair(),
    )}


# The module-level registry. `register` adds an operator to it; a Shield built
# afterwards copies it. Names, not objects, define the stacks below, so a
# registered replacement for "LateralProjection" takes that stage.
REGISTRY: dict = default_operators()


# Every violation category the Shield raises (guardrail/shield.py Violation):
# what an operator that declares no `categories` is taken to cover.
CATEGORIES = ("kinematic", "altitude", "clearance", "standoff", "corridor", "geofence",
              "contract")


class GrantOperator:
    """An operator written to the grant's own signature,

        repair(action, violation, ir) -> RepairAttempt   (or -> Action4D)

    made a RepairOperator. It is called without `ctx`; with no `categories`
    it covers every category. What it returns becomes this pass's attempt;
    when it says "ok" and changed the action but logged no Repair, one is
    logged in its name (detail: its reason), so the decision's `repairs`
    and the audit record never fly a change nobody owns. An operator that
    takes `ctx` but declares no categories is wrapped too, and gets `ctx`."""

    def __init__(self, op, *, pass_ctx: bool = False):
        self.op = op
        self.name = op.name
        self.categories = tuple(getattr(op, "categories", None) or CATEGORIES)
        self.pass_ctx = pass_ctx

    def __repr__(self) -> str:
        return f"GrantOperator({self.op!r})"

    def repair(self, action, violation, ir, *, ctx: RepairContext) -> RepairAttempt:
        out = (self.op.repair(action, violation, ir, ctx=ctx) if self.pass_ctx
               else self.op.repair(action, violation, ir))
        if isinstance(out, RepairAttempt):
            att = out
        elif isinstance(out, Action4D):
            att = (RepairAttempt(self.name, "ok", out) if out != action else
                   RepairAttempt(self.name, "skip", out, "the operator left the action unchanged"))
        else:
            raise TypeError(f"{self.name}.repair returned {type(out).__name__}, not a "
                            f"RepairAttempt or an Action4D")
        if att.result not in RESULTS:
            raise ValueError(f"{self.name}: result {att.result!r} is not one of {RESULTS}")
        reps = tuple(att.repairs)
        if att.result == "ok":
            if not reps and att.action != action:
                from .shield import Repair            # shield imports this module
                reps = (Repair(operator=self.name,
                               detail=att.reason or f"{self.name} changed the action"),)
            ctx.repairs.extend(reps)
        return replace(att, operator=self.name, repairs=reps, iterations=ctx.pass_no)


def adapt(op):
    """`op` as a RepairOperator: itself when its `repair` takes `ctx` and it
    declares `categories`, else wrapped in GrantOperator. Refuses anything
    without a string `name` and a callable `repair`."""
    rep = getattr(op, "repair", None)
    if not isinstance(getattr(op, "name", None), str) or not callable(rep):
        raise TypeError(f"{op!r} is not a repair operator (it needs `name` and `repair`)")
    try:
        params = inspect.signature(rep).parameters.values()
    except (TypeError, ValueError):                   # a builtin: call it the grant's way
        params = ()
    takes_ctx = any(p.name == "ctx" or p.kind is p.VAR_KEYWORD for p in params)
    if takes_ctx and isinstance(getattr(op, "categories", None), tuple):
        return op
    return GrantOperator(op, pass_ctx=takes_ctx)


def register(op, *, replace_existing: bool = False):
    """Add `op` to the registry and return what was registered: `op` itself,
    or `op` wrapped (`adapt`) when it is written to the grant's three-
    argument signature or declares no categories. Refuses a name already
    taken unless `replace_existing`. A registered name runs where a stack
    names it: replacing a stage of CHAIN_LOOP / FIRST_FEASIBLE, or listed in
    a Shield's `repair_order`."""
    op = adapt(op)
    if op.name in REGISTRY and not replace_existing:
        raise ValueError(f"a repair operator named {op.name!r} is already registered")
    REGISTRY[op.name] = op
    return op


# The default chain (guardrail/shield.py `_repair_chain`): run once before the
# iterated loop, then each pass of the loop. KinematicClamp closes every pass
# because AltitudeFix can size a climb past the cap on a deep recovery.
CHAIN_PRE = ("KinematicClamp", "AltitudeClamp")
CHAIN_LOOP = ("ClearanceProjection", "StandoffProjection", "CorridorProjection",
              "LateralProjection", "KinematicClamp")
# First-feasible priority. The grant names three (AltitudeClamp,
# LateralProjection, PathRepair); this project's other classes slot in after
# the lateral projection, the cheap clamps first.
FIRST_FEASIBLE = ("KinematicClamp", "AltitudeClamp", "LateralProjection",
                  "CorridorProjection", "StandoffProjection", "ClearanceProjection")
REPAIR_MODES = ("chain", "first_feasible")


def with_path_repair(stack: tuple) -> tuple:
    """`stack` with PathRepair right after LateralProjection."""
    out = list(stack)
    i = out.index("LateralProjection") + 1 if "LateralProjection" in out else len(out)
    out.insert(i, "PathRepair")
    return tuple(out)


def ledger_record(passes: list) -> list:
    """The audit form of a chain run: one entry per pass, every attempt."""
    return [{"pass": k, "attempts": [a.record() for a in atts]}
            for k, atts in passes]


__all__ = ["CATEGORIES", "CHAIN_LOOP", "CHAIN_PRE", "FIRST_FEASIBLE", "GrantOperator",
           "HookOperator", "PathRepair", "REGISTRY", "REPAIR_MODES", "RESULTS",
           "RepairAttempt", "RepairContext", "RepairOperator", "adapt", "default_operators",
           "ledger_record", "register", "with_path_repair"]
