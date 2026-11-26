"""The stress library's mid-flight events, applied to a Shield.

    es = EventState(shields=[shield])
    rec = apply_event(ev, es, state, heading, t)               # headless harness
    rec = apply_event(ev, es, state, heading, t, shield_t=None)  # a flight rail

Moved here from experiments/sweep_scenarios.py on 2026-11-06, so the hil rail
(sitl/ros2_shield_node.py --events) applies a scenario's ScenarioEvents with
the code the headless harness has applied them with since 2026-10-07: the
grant's seven event types through the Shield's own event APIs
(guardrail/scenario_spec.py ScenarioEvent says which API each one drives).
The harness keeps its old names (`_spawn_zone`, `_direction`, `_window`,
`_EventState`, `_zone`, `apply_event`) as aliases and wrappers.

TWO CLOCKS. `t` is the time the RECORD carries (the harness's simulated time,
a rail's mission time). `shield_t` is the time handed to the Shield's event
APIs: the same `t` by default, as the headless harness always passed it; a
rail passes None, as the REST hot-apply path does (guardrail/api.py), so the
Shield stamps the event with its own last tick (Shield._event_time) and a
moving zone's motion runs on the clock its filter() ticks run on.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable

from .models import CorridorSwap, DynamicNFZ, State, TimeWindowSwitch, wgs84_from_metres


class UnsupportedEvent(RuntimeError):
    """An event type this module does not apply (none of the grant's seven:
    scenario_spec.ScenarioEvent refuses any other at load)."""


_SAME = object()


def in_wgs84(obj, shield):
    """`obj` (a rule, or raw point dicts) with its absolute metre points
    written as WGS84 lat/lon about `shield`'s policy frame origin: the
    tool-side conversion (models.wgs84_from_metres) a scenario event goes
    through before it reaches the Shield, which since 2026-11-26 takes an
    absolute position in lat/lon only (WP1-14). The stress library writes
    its events in local metres (the grant's scenario payloads are metres
    relative to the vehicle: width_m, ahead_of_vehicle_m), so the harness
    converts them here, with the same metres_to_wgs84 the Shield applied at
    ingest until then: the stored lat/lon, the hash and the enforced zone
    are unchanged. A policy with no frame origin is left to the Shield,
    which refuses the event as before (NoFrameOrigin), as is a Shield with
    its comparison switch wgs84_writes off (it stores what it is given)."""
    origin = shield.policy.frame_origin
    if origin is None or not getattr(shield, "wgs84_writes", True):
        return obj
    return wgs84_from_metres(obj, origin)


def spawn_zone(payload: dict, k: int, st: State, heading: tuple[float, float]
               ) -> DynamicNFZ:
    """spawn_polygon_fence, with the grant example's payload, as the dynamic_nfz
    the Shield accepts mid-flight.

    `width_m` x `height_m` (across x along track), its near edge
    `ahead_of_vehicle_m` ahead of the vehicle on its current heading - the
    worked example's "Polygon NFZ spawns 60 m ahead". `anchor: center` puts the
    CENTRE that far ahead instead, so `ahead_of_vehicle_m: 0, anchor: center`
    drops the zone on top of the aircraft. `vertices` gives it absolutely:
    in local metres ({x, y}, the library's frame; converted to WGS84 before
    the Shield sees them, `in_wgs84`) or, since 2026-11-26, in WGS84
    ({lat, lon}, passed as given; WP1-14).
    `motion` ({vx_mps, vy_mps, yaw_rate_dps}) makes it move from the spawn on;
    `violation_action` (default repair) is the breach action, so a zone can ask
    for a brake, a loiter, RTL or a landing.

    Until 2026-10-07 this built a polygon_fence, which the Shield now locks at
    mission start (guardrail/shield.py MIGRATION_NOTE): a mid-flight polygon
    zone IS a dynamic_nfz, which keeps the id, vertices, band, margin and
    priority (shield.as_dynamic_nfz).
    """
    if "vertices" in payload:
        verts = [_point(v) for v in payload["vertices"]]
    else:
        w, h = float(payload["width_m"]), float(payload["height_m"])
        a = float(payload.get("ahead_of_vehicle_m", 0.0))
        fx, fy = heading
        lx, ly = -fy, fx
        along = a + h / 2 if payload.get("anchor", "near_edge") == "near_edge" else a
        cx, cy = st.x + fx * along, st.y + fy * along
        verts = [{"x": cx + fx * sa * h / 2 + lx * sb * w / 2,
                  "y": cy + fy * sa * h / 2 + ly * sb * w / 2}
                 for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    data = {"id": payload.get("id", f"dyn-nfz-{k}"), "type": "dynamic_nfz",
            "priority": payload.get("priority", "P0"), "vertices": verts,
            "constraint_type": payload.get("constraint_type", "hard"),
            "violation_action": payload.get("violation_action", "repair"),
            "scope": payload.get("scope", "mission"),   # required since IR 4.0.0
            "altitude_floor_m": float(payload.get("altitude_floor_m", 0.0)),
            "altitude_ceiling_m": float(payload.get("altitude_ceiling_m", 1000.0)),
            "margin_m": float(payload.get("margin_m", 1.0))}
    if payload.get("motion") is not None:
        data["motion"] = dict(payload["motion"])
    return DynamicNFZ.model_validate(data)


def _point(v) -> dict:
    """An absolute point of an event payload: {lat, lon} (WGS84) as given, or
    {x, y} local metres (a dict or an [x, y] pair), as floats."""
    if isinstance(v, dict) and v.get("lat") is not None and v.get("lon") is not None:
        return {"lat": float(v["lat"]), "lon": float(v["lon"])}
    if isinstance(v, dict):
        return {"x": float(v["x"]), "y": float(v["y"])}
    return {"x": float(v[0]), "y": float(v[1])}


def _record_ring(shield, zone) -> list[list[float]]:
    """A zone's ring for the event record: [x, y] metres rounded to the mm,
    from the zone as given when it is in metres (as recorded since
    2026-10-07), else as the Shield stored it (its frame's view)."""
    if all(not v.geographic for v in zone.vertices):
        return [[round(v.x, 3), round(v.y, 3)] for v in zone.vertices]
    return [[round(v.x, 3), round(v.y, 3)] for v in zone_of(shield, zone.id).vertices]


def direction(payload: dict, heading: tuple[float, float]) -> tuple[float, float]:
    """A unit vector for translate_polygon_fence's motion: relative to the
    vehicle's heading (x North, y East; left of a northbound heading is West)
    or an absolute bearing, degrees clockwise from North."""
    if "bearing_deg" in payload:
        b = math.radians(float(payload["bearing_deg"]))
        return math.cos(b), math.sin(b)
    fx, fy = heading
    return {"along": (fx, fy), "against": (-fx, -fy),
            "perpendicular_left": (fy, -fx),
            "perpendicular_right": (-fy, fx)}[payload["direction"]]


def window(payload: dict) -> dict | None:
    w = payload.get("window")
    return None if w is None else {"recurrence": dict(w)}


@dataclass
class EventState:
    """What the event runner keeps between events of one episode."""
    shields: list                          # every Shield kept in step
    n_spawn: int = 0
    last_zone: str | None = None
    refused: list = field(default_factory=list)   # (type, reason, expected)


def zone_of(shield, zid: str):
    for c in shield.policy.constraints:
        if c.id == zid:
            if not isinstance(c, DynamicNFZ):
                raise ValueError(f"{zid!r} is a {c.type}; only a dynamic_nfz moves, "
                                 f"rotates or scales mid-flight")
            return c
    raise ValueError(f"no zone {zid!r} to edit (spawn one first)")


def start_heading(start: tuple[float, float], target: tuple[float, float] | None,
                  yaw_deg: float) -> tuple[float, float]:
    """The heading an episode's events see before the vehicle has moved: from
    the start toward the target, else along the start yaw (degrees clockwise
    from North) - the stress harness's rule since 2026-10-07."""
    if target is not None:
        hx, hy = target[0] - start[0], target[1] - start[1]
        n = math.hypot(hx, hy)
        if n > 1e-6:
            return hx / n, hy / n
        return 1.0, 0.0
    r = math.radians(yaw_deg)
    return math.cos(r), math.sin(r)


def rules_switch_with_clock(policy) -> bool:
    """Can a rule's in-force state change during the run with no event? A
    weekly window, a time_window_switch or a corridor_swap (each may switch
    with the clock or the event that added it). The stress harness's
    `_timed`."""
    return any((c.valid_time is not None and c.valid_time.recurrence is not None)
               or isinstance(c, (TimeWindowSwitch, CorridorSwap))
               for c in policy.constraints)


def has_moving_zone(policy) -> bool:
    """Does the policy hold a dynamic_nfz with a motion (a zone that moves
    with no event)?"""
    return any(isinstance(c, DynamicNFZ) and c.motion is not None
               for c in policy.constraints)


def apply_event(ev, es: EventState, st: State, heading: tuple[float, float],
                t: float, *, shield_t: Any = _SAME,
                spawn: Callable | None = None) -> dict:
    """Apply one ScenarioEvent to every Shield in `es.shields`, identically, and
    return the record harness_events.jsonl (or a rail's events.jsonl) keeps.

    A Shield refusal (LockedRuleClass, LayerRelaxation, UnknownRule, a lint
    finding: all ValueError) is raised - the episode errors - unless the
    payload says `expect_refused: true`; then it is recorded, and an event
    expected to be refused that was ACCEPTED is recorded as that, and fails
    the scenario's `event_refusals_as_expected` gate.

    `t` goes into the record; `shield_t` (default: `t`) is what the Shield's
    event APIs are given (module docstring, TWO CLOCKS). `spawn` builds the
    zone of a spawn_polygon_fence (default `spawn_zone`): the harness passes
    its own name, which its tests replace."""
    p = ev.payload
    expect_refused = bool(p.get("expect_refused", False))
    flying = es.shields[0]
    when = t if shield_t is _SAME else shield_t
    rec: dict[str, Any] = {"t": round(t, 4), "type": "hot_apply", "event": ev.type,
                           "at_sim_t": ev.at_sim_t}

    def run_all(fn) -> dict:
        try:
            out = fn(flying)
        except ValueError as e:
            if not expect_refused:
                raise
            es.refused.append((ev.type, f"{type(e).__name__}: {e}", True))
            rec.update(refused=True, reason=f"{type(e).__name__}: {e}",
                       expected=True, generation=flying.policy.generation,
                       policy_hash=flying.policy.policy_hash)
            return rec
        for s in es.shields[1:]:
            fn(s)
        if expect_refused:
            es.refused.append((ev.type, "accepted", False))
            rec.update(accepted_unexpectedly=True)
        rec.update(op=out.get("op"), rule_id=out.get("rule_id"),
                   generation=flying.policy.generation,
                   policy_hash=flying.policy.policy_hash)
        return rec

    if ev.type == "spawn_polygon_fence":
        zone = (spawn or spawn_zone)(p, es.n_spawn, st, heading)
        es.n_spawn += 1
        if any(c.id == zone.id for c in flying.policy.constraints):
            # Same refusal as the hot-apply REST endpoint (guardrail/api.py).
            raise ValueError(f"spawn_polygon_fence: id {zone.id!r} already exists")
        out = run_all(lambda s: s.hot_apply(in_wgs84(zone, s), when))
        if not out.get("refused"):
            es.last_zone = zone.id
            out["vertices"] = _record_ring(flying, zone)
            out["violation_action"] = zone.violation_action
            if zone.motion is not None:
                out["motion"] = zone.motion.model_dump()
        return out

    if ev.type in ("translate_polygon_fence", "rotate_polygon_fence", "scale_radius"):
        zid = p.get("id") or es.last_zone
        if zid is None:
            raise ValueError(f"{ev.type}: no zone named and none spawned yet")
        zone = zone_of(flying, zid)
        motion = zone.motion.model_dump() if zone.motion is not None else \
            {"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rate_dps": 0.0}
        # A motion swap keeps the zone where the FLYING Shield has it now
        # (Shield.zone_ring_at), and that one ring goes to every Shield so
        # they stay in step. Until 2026-10-19 the harness kept its own anchor
        # per zone and defaulted it to the event time for a zone the policy
        # declared, so ring_at(0) put a moving policy zone back at its
        # authored vertices - 10 m back for a 1 m/s zone edited at t = 10 s
        # (latent: the flight loader refuses a policy file with a
        # dynamic_nfz today; review, harness-tools/ring-now-wrong-anchor-
        # for-policy-zones).
        if ev.type == "translate_polygon_fence" and "mps" in p:
            ux, uy = direction(p, heading)
            motion.update(vx_mps=float(p["mps"]) * ux, vy_mps=float(p["mps"]) * uy)
            ring = [{"x": x, "y": y} for x, y in flying.zone_ring_at(zid, when)]
            out = run_all(lambda s: s.move_nfz(zid, in_wgs84(ring, s), when,
                                               motion=dict(motion)))
            out["motion"] = motion
        elif ev.type == "rotate_polygon_fence" and "deg_per_s" in p:
            motion["yaw_rate_dps"] = float(p["deg_per_s"])
            ring = [{"x": x, "y": y} for x, y in flying.zone_ring_at(zid, when)]
            out = run_all(lambda s: s.move_nfz(zid, in_wgs84(ring, s), when,
                                               motion=dict(motion)))
            out["motion"] = motion
        elif ev.type == "translate_polygon_fence":
            dx, dy = float(p.get("dx_m", 0.0)), float(p.get("dy_m", 0.0))
            out = run_all(lambda s: s.translate_nfz(zid, dx, dy, when))
        elif ev.type == "rotate_polygon_fence":
            ang = float(p["angle_deg"])
            out = run_all(lambda s: s.rotate_nfz(zid, ang, when))
        else:
            f = float(p["factor"])
            out = run_all(lambda s: s.scale_nfz(zid, f, when))
        if not out.get("refused"):
            z = zone_of(flying, zid)
            out["vertices"] = [[round(v.x, 3), round(v.y, 3)] for v in z.vertices]
        out["zone"] = zid
        return out

    if ev.type in ("activate_rule", "deactivate_rule"):
        active = ev.type == "activate_rule"
        rid = p["rule_id"]
        win = window(p)
        if win is None:
            sid = p.get("id")
            out = run_all(lambda s: s.set_rule_active(rid, active, when, switch_id=sid))
        else:
            sw = TimeWindowSwitch.model_validate({
                "id": p.get("id") or f"sw-{rid}-{'on' if active else 'off'}",
                "type": "time_window_switch", "target_id": rid, "active": active,
                # required since IR schema 3.0.0; the values they defaulted to
                "constraint_type": "hard", "priority": "P0",
                "violation_action": "project_fix", "scope": "mission",
                "valid_time": win})
            out = run_all(lambda s: s.hot_apply(sw, when))
            out["window"] = win["recurrence"]
        out.update(target=rid, active=active)
        return out

    if ev.type == "swap_corridor":
        cl = [_point(v) for v in p["centerline"]]
        data = {"id": p.get("id") or f"swap-{p['target_id']}", "type": "corridor_swap",
                "target_id": p["target_id"], "centerline": cl,
                "width_m": float(p["width_m"]),
                # required since IR schema 3.0.0; the values they defaulted to
                "constraint_type": "hard", "priority": "P0",
                "violation_action": "project_fix", "scope": "mission"}
        for k in ("altitude_floor_m", "altitude_ceiling_m"):
            if k in p:
                data[k] = float(p[k])
        if window(p) is not None:
            data["valid_time"] = window(p)
        sp = CorridorSwap.model_validate(data)
        out = run_all(lambda s: s.hot_apply(in_wgs84(sp, s), when))
        out.update(target=p["target_id"],
                   centerline=[[v["x"], v["y"]] if "x" in v else [v["lat"], v["lon"]]
                               for v in cl],
                   width_m=data["width_m"])
        return out

    raise UnsupportedEvent(f"event type {ev.type!r}")
