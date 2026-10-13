"""Which of a policy's keep-out zones a clockless autopilot fence may hold.

    zones, skipped = policy_zones(policy, column_m=30.0)
    zones, skipped = policy_zones(policy, now=clock(), column_m=30.0)

WHY THIS IS ITS OWN MODULE

ArduPilot's GeoFence is the grant's backstop behind the Shield (Architecture
constraints p1: it is to "fire only if the Shield itself crashes or fails to
emit"). A backstop that is STRICTER than the policy fires first, on a flight
the Shield allows, and an RTL at the fence edge then measures the autopilot,
not the Shield. So the zones uploaded to it are only those it can hold exactly
as the policy does, at every altitude and at every instant of the flight.

Until 2026-10-12 that selection lived inside
sitl/mavlink_adapter_node.policy_fences, beside the ROS-specific deferral of a
zone the aircraft is near. The perception rail (demo/pas_ardupilot) needs the
same selection without ROS, so it lives here; the adapter imports it and
behaves exactly as before (tests/test_mavlink_adapter.py).

THE RULES (each one an observed or a reasoned way to be stricter than the
policy)

  * Keep-out zones only: polygon_fence, circle_fence, and a dynamic_nfz
    without motion (what a mid-flight zone becomes where the Shield enforces
    the grant's update model). A corridor is a keep-in rule: an inclusion
    fence around it would breach on the take-off pad.
  * No zone the Shield only watches (2026-10-12, perception rail): a
    monitor_only zone is recorded when entered and never repaired, and an
    autopilot fence around it would RTL a flight the policy lets through.
    No shipped policy has one, so neither rail's export changed.
  * No zone a CLOCK can turn off: one with a time window (`valid_time` with a
    recurrence), and one a time_window_switch can switch off (`active: false`
    naming it). The autopilot fence has no clock, so it would hold such a
    zone while the policy has it off. A switch that only holds a zone ON
    adds no off-time and does not count.
  * No moving dynamic_nfz: the autopilot fence is static.
  * With the fence column unknown (the autopilot's FENCE_ALT_MAX unread),
    nothing: ArduPilot's polygon and circle fences are 2-D and apply at
    every altitude, so a zone is exported only if its altitude band covers
    the whole column 0 .. FENCE_ALT_MAX.

`now` NEVER CHANGES THE EXPORT. The backstop has no clock, so the selection
is the same at every instant ("clock-proof"). With `now` (the rule clock's
reading, as Shield(now=...) takes it) every keep-out entry, exported or
skipped, also carries `in_force_now`: whether the Shield holds the zone at
that instant, read with the compiler's exact copy of the Shield's switch
logic (guardrail.compiler._RuleClock). That is for a display or a record
(which zones the Shield alone is holding right now); it is not a reason to
upload anything.

`defer_near` is the other half of what the ROS adapter does: a zone the
aircraft is inside, or within the fence margin of, is held back until it is
clear, because uploading it would breach the autopilot fence on arrival - but
never a zone the autopilot already holds (`enforced`): each upload replaces
the whole list, so holding that one back would remove it.
"""
from __future__ import annotations

import math
from typing import Any, Iterator

# The keep-out zone types an autopilot exclusion fence can represent.
ZONE_TYPES = ("polygon_fence", "circle_fence", "dynamic_nfz")


def zone_xy(p) -> tuple[float, float]:
    """(x North, y East) of a vertex given as a dict, a pair, or an XY."""
    if isinstance(p, dict):
        return float(p["x"]), float(p["y"])
    if isinstance(p, (list, tuple)):
        return float(p[0]), float(p[1])
    return float(p.x), float(p.y)


def time_controls(rule, off_switches=()) -> list[str]:
    """What can switch `rule` off by the clock: its own time window
    (`valid_time` with a recurrence; absent, or without one, means always)
    and the ids of the time_window_switches that turn it off. [] = in force
    for the whole flight."""
    out = []
    vt = getattr(rule, "valid_time", None)
    if vt is not None and getattr(vt, "recurrence", None) is not None:
        out.append("it has a time window (valid_time)")
    if off_switches:
        out.append(f"time_window_switch {', '.join(sorted(off_switches))} can "
                   f"switch it off")
    return out


def _off_switches(constraints) -> dict[str, list[str]]:
    """Rule id -> the switches that can turn it OFF. A switch that holds its
    target ON adds no off-time to it."""
    out: dict[str, list[str]] = {}
    for c in constraints:
        if getattr(c, "type", "") == "time_window_switch" \
                and getattr(c, "active", True) is False:
            out.setdefault(c.target_id, []).append(c.id)
    return out


def _in_force(policy, now):
    """rule -> bool at `now`, the Shield's own answer (compiler._RuleClock is
    held to Shield.rule_status by tests/test_csp.py). Only for a real Policy;
    a duck-typed stand-in has no schedule to read, and counts as in force."""
    try:
        from .compiler import _RuleClock
        clock = _RuleClock(policy)
    except Exception:                                   # noqa: BLE001
        return lambda rule: True

    def on(rule) -> bool:
        try:
            return bool(clock.on(rule, now))
        except Exception:                               # noqa: BLE001
            return True                 # absent means active (models.active_at)
    return on


def iter_zones(policy, now=None, column_m: float | None = None
               ) -> Iterator[tuple[str, dict[str, Any]]]:
    """("zone", zone) or ("skipped", entry) for each rule, in policy order.

    A zone is {"id", "kind": "polygon", "vertices": [[x, y], ...]} or
    {"id", "kind": "circle", "center": [x, y], "radius_m"}; a skipped entry is
    {"id", "why"} plus `time_controlled: True` for a zone a clock can turn
    off. `policy_zones` collects these; the ROS adapter walks them to hold a
    zone back in place, so its skip list keeps the policy's order."""
    constraints = list(getattr(policy, "constraints", []))
    switched_off = _off_switches(constraints)
    in_force = _in_force(policy, now) if now is not None else None

    def tag(entry: dict, rule) -> dict:
        if in_force is not None:
            entry["in_force_now"] = in_force(rule)
        return entry

    for c in constraints:
        ctype = getattr(c, "type", "")
        if ctype not in ZONE_TYPES:
            if ctype == "corridor":
                yield "skipped", {"id": c.id, "why": "keep-in corridor: not "
                                  "exported as an autopilot inclusion fence"}
            continue
        if getattr(c, "violation_action", None) == "monitor_only":
            yield "skipped", tag({"id": c.id, "why": (
                "monitor_only: the Shield records an entry and never repairs "
                "one, and an autopilot fence would act on it - stricter than "
                "the policy")}, c)
            continue
        clock = time_controls(c, switched_off.get(c.id, ()))
        if clock:
            yield "skipped", tag({"id": c.id, "time_controlled": True, "why": (
                f"{'; '.join(clock)}: the autopilot fence has no clock and "
                f"would enforce the zone while the policy has it off - "
                f"stricter than the policy, so the Shield alone enforces it")}, c)
            continue
        if ctype == "dynamic_nfz" and getattr(c, "motion", None) is not None:
            yield "skipped", tag({"id": c.id, "why": "moving dynamic_nfz: the "
                                  "autopilot fence is static, so the Shield "
                                  "alone enforces it"}, c)
            continue
        if column_m is None:
            yield "skipped", tag({"id": c.id, "why": "the autopilot's "
                                  "FENCE_ALT_MAX is unknown, so whether a 2-D "
                                  "fence would be stricter than the policy "
                                  "cannot be decided"}, c)
            continue
        lo = float(getattr(c, "altitude_floor_m", 0.0) or 0.0)
        hi = getattr(c, "altitude_ceiling_m", None)
        hi = float("inf") if hi is None else float(hi)
        if lo > 0.0 or hi < column_m:
            yield "skipped", tag({"id": c.id, "why": (
                f"altitude band {lo:g}-{hi:g} m does not cover the fence "
                f"column 0-{column_m:g} m; a 2-D autopilot fence would be "
                f"stricter than the policy")}, c)
            continue
        if ctype in ("polygon_fence", "dynamic_nfz"):
            f = {"id": c.id, "kind": "polygon",
                 "vertices": [list(zone_xy(v)) for v in c.vertices]}
        else:
            centre = getattr(c, "center", None) or getattr(c, "centre", None)
            radius = getattr(c, "radius_m", None) or getattr(c, "radius", None)
            if centre is None or not radius:
                yield "skipped", tag({"id": c.id, "why": "circle_fence without "
                                      "a centre and radius this exporter can "
                                      "read"}, c)
                continue
            f = {"id": c.id, "kind": "circle", "center": list(zone_xy(centre)),
                 "radius_m": float(radius)}
        yield "zone", tag(f, c)


def policy_zones(policy, now=None, column_m: float | None = None
                 ) -> tuple[list[dict], list[dict]]:
    """The policy's keep-out zones a clockless 2-D autopilot fence can hold
    exactly as the policy does: `(zones, skipped)`, each in policy order.

    `column_m` is the autopilot's FENCE_ALT_MAX (None: unknown, nothing is
    exported). `now`, when given, only adds `in_force_now` to each keep-out
    entry (module docstring); the selection does not depend on it. The
    aircraft's position plays no part here: see `defer_near`."""
    zones, skipped = [], []
    for kind, entry in iter_zones(policy, now=now, column_m=column_m):
        (zones if kind == "zone" else skipped).append(entry)
    return zones, skipped


# --------------------------------------------------------------------------- #
# Where the aircraft is: hold back a zone it is in or near
# --------------------------------------------------------------------------- #

def _point_in_polygon(x: float, y: float, verts: list) -> bool:
    inside = False
    n = len(verts)
    for i in range(n):
        (x1, y1), (x2, y2) = verts[i], verts[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


def _segment_distance(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def zone_clearance_m(fence: dict, x: float, y: float) -> float:
    """Signed horizontal distance from (x, y) to an exported zone's edge:
    positive outside, negative inside (the depth)."""
    if fence["kind"] == "circle":
        cx, cy = fence["center"]
        return math.hypot(x - cx, y - cy) - fence["radius_m"]
    verts = [tuple(v) for v in fence["vertices"]]
    d = min(_segment_distance(x, y, *verts[i], *verts[(i + 1) % len(verts)])
            for i in range(len(verts)))
    return -d if _point_in_polygon(x, y, verts) else d


def deferral(zone: dict, position, margin_m: float) -> dict | None:
    """The skipped entry that holds `zone` back while the aircraft at
    `position` (x North, y East) is inside it or within `margin_m` of it, or
    None when it may be uploaded now."""
    clr = zone_clearance_m(zone, float(position[0]), float(position[1]))
    if clr >= margin_m:
        return None
    return {"id": zone["id"], "deferred": True, "clearance_m": round(clr, 2),
            "why": (f"aircraft {'inside' if clr < 0 else 'within'}"
                    f" {abs(clr):.1f} m of the zone (margin "
                    f"{margin_m:g} m): deferred until clear, "
                    f"so the upload cannot breach the "
                    f"autopilot fence on arrival")}


def zone_geometry(zone: dict) -> tuple:
    """What an autopilot fence holds of an exported zone: its id, kind and
    shape (not the `in_force_now` a record adds)."""
    verts = zone.get("vertices")
    centre = zone.get("center")
    return (zone["id"], zone["kind"],
            None if verts is None else tuple(tuple(map(float, v)) for v in verts),
            None if centre is None else tuple(map(float, centre)),
            zone.get("radius_m"))


def is_enforced(zone: dict, enforced) -> bool:
    """Whether the autopilot already holds `zone`, with this shape: `enforced`
    maps id -> the zone as last uploaded with success (None: nothing)."""
    held = (enforced or {}).get(zone["id"])
    return held is not None and zone_geometry(held) == zone_geometry(zone)


def defer_near(zones: list[dict], position, margin_m: float, enforced=None
               ) -> tuple[list[dict], list[dict]]:
    """`(upload_now, deferred)`: the zones the aircraft at `position` is clear
    of, and a `deferred: True` skipped entry for each one it is not. With no
    position every zone is uploaded (the caller decides what an unknown
    position means: the ROS node assumes the pad).

    `enforced` (id -> zone, the last upload that succeeded): a zone the
    autopilot already holds with this shape is never held back. It cannot
    breach on arrival - it is already on the fence - and a re-upload (the
    list is replaced whole) that held it back would take it OFF the fence
    exactly while the aircraft is next to it, the moment the backstop exists
    for (review 2026-10-13: a held-back pad zone clearing next to an enforced
    zone dropped that zone)."""
    if position is None:
        return list(zones), []
    now, held = [], []
    for z in zones:
        d = None if is_enforced(z, enforced) else deferral(z, position, margin_m)
        (now if d is None else held).append(z if d is None else d)
    return now, held


def zones_near(fences: list[dict], x: float, y: float, margin_m: float) -> list[str]:
    """Ids of exported zones that (x, y) is inside or within `margin_m` of."""
    return [f["id"] for f in fences
            if zone_clearance_m(f, float(x), float(y)) < margin_m]
