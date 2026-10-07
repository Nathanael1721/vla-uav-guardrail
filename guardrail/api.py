"""
REST hot-apply endpoint — the external interface other teams asked for
(2026-07-07 meeting: "provide an API so other modules can create obstacles /
no-fly zones dynamically in the simulation"), and the grant's hot-apply
channel (Policy DSL page: "a live REST endpoint for cloud-pushed updates (the
channel hot-applies use)"; Outputs: "REST hot-apply endpoint - FastAPI service,
deployed alongside the harness").

Wraps a live Shield in a small FastAPI app. Only the grant's three
hot-applicable classes can change in flight (Policy DSL page, "Mid-flight
update model"); every route below is one of them:

    GET    /health                  liveness, policy id / hash / generation
    GET    /policy                  the full active policy
    GET    /events                  every event applied so far, in order
    POST   /dynamic_nfz             spawn a dynamic no-fly zone ("add")
    PATCH  /dynamic_nfz/{id}        move / translate / rotate / scale it
    DELETE /dynamic_nfz/{id}        expire it
    POST   /time_window_switch      hold a rule on or off
    POST   /corridor_swap           replace a corridor
    POST   /hot_apply               any of the three, by "type"
    POST   /nfz                     the 2026-07 body, kept for its callers: it
                                    now spawns a dynamic_nfz (see below)

POST /dynamic_nfz body (metres in the policy frame, or lat/lon for a policy
with a frame; the DSL's own field names):

    {"id": "nfz-landslide-007",
     "vertices": [{"x": 23, "y": 6}, {"x": 31, "y": 6}, {"x": 31, "y": 14}],
     "altitude_floor_m": 0, "altitude_ceiling_m": 100, "margin_m": 1.0,
     "motion": {"vx_mps": 0.5, "vy_mps": 0.0, "yaw_rate_dps": 0.0}}

PATCH /dynamic_nfz/{id}: {"op": "move", "vertices": [...]},
{"op": "translate", "dx": 5, "dy": 0}, {"op": "rotate", "angle_deg": 30} or
{"op": "scale", "factor": 1.5}.

POST /time_window_switch: {"target_id": "nfz-school-yard", "active": false}
(an `id` and a `valid_time` are optional). POST /corridor_swap: a corridor's
fields plus "target_id" and an "id".

WHAT CHANGED ON 2026-10-07. POST /nfz built a PolygonFence and appended it -
a class the grant locks at mission start. It now spawns a dynamic_nfz with the
same id, vertices, band and margin, and its response says so ("type":
"dynamic_nfz", "deprecated": ...). A request for any locked class through
/hot_apply is refused with 422 and the migration note. Every accepted event
bumps the policy generation and re-derives the policy hash; the Shield swaps
its compiled rules in one assignment, so the 10 Hz loop sees the change on its
next tick and never half of it.

THE LAYER RULE. Every event over REST is a MISSION-layer event (the DSL's
default layer). A body that names another layer ("layer": "regulation" or
"site") is refused with 422: nothing on this channel establishes that the
sender speaks for the regulator or the site, and a higher layer's event could
relax that layer's own hard rules. A mission-layer event may relax a rule of
its own layer (that is what a switch-off is for) and any soft rule, but never
a HARD rule of another layer: no switch-off, no swap that is not provably no
looser, no edit or expiry of the zone (422, the tests models.merge_layers runs
at ingest; Policy DSL page, "Layered authoring model"). Before this rule
(2026-10-07 review) a mission switch turned a hard regulation zone off and the
aircraft flew into it unrepaired.

Status codes: 409 an id that already exists, 404 an id that does not, 422 any
other refusal (a locked class, a bad polygon, a corridor_swap aimed at a
fence, a layer above mission, a relaxation across layers). The request
handling is plain functions (`apply_*` below) so it is testable where FastAPI
is not installed; `build_app` only routes to them.
"""
from __future__ import annotations

import threading
from typing import Any

from pydantic import BaseModel, Field, ValidationError

from .models import DEFAULT_LAYER, XY, CorridorSwap, DynamicNFZ, TimeWindowSwitch
from .shield import (HOT_APPLICABLE, HotApplyRefused, LockedRuleClass, RuleIdConflict,
                     Shield, UnknownRule)


class NfzRequest(BaseModel):
    """The 2026-07 POST /nfz body."""
    id: str
    vertices: list[XY] = Field(min_length=3)
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 1000.0
    margin_m: float = 1.0


class ApiError(Exception):
    """A refused request: HTTP status and the reason, in words."""

    def __init__(self, status: int, detail: str):
        super().__init__(detail)
        self.status = status
        self.detail = detail


def _status(e: Exception) -> int:
    if isinstance(e, RuleIdConflict):
        return 409
    if isinstance(e, UnknownRule):
        return 404
    return 422


def _refuse(e: Exception) -> ApiError:
    return ApiError(_status(e), str(e))


def _points(shield: Shield, pts: Any) -> list[dict]:
    """Vertices as the DSL's point dicts. A lat/lon point is placed in the
    policy's own frame (its stated or derived origin); a policy written only in
    metres has no frame for it, and the request is refused."""
    if not isinstance(pts, list):
        raise ApiError(422, "vertices / centerline must be a list of points")
    out = []
    proj = None
    for p in pts:
        if isinstance(p, (list, tuple)) and len(p) == 2:
            p = {"x": p[0], "y": p[1]}
        if not isinstance(p, dict):
            raise ApiError(422, f"point {p!r} is neither {{x, y}} nor {{lat, lon}}")
        if "x" in p and "y" in p:
            out.append({k: v for k, v in p.items() if k in ("x", "y", "lat", "lon")})
            continue
        if "lat" in p and "lon" in p:
            if proj is None:
                proj = shield.policy.projection()
                if proj is None:
                    raise ApiError(422, f"{shield.policy.policy_id} is written in metres "
                                        f"and has no frame for a lat/lon point; send x/y")
            x, y = proj.to_local(float(p["lat"]), float(p["lon"]))
            out.append({"x": x, "y": y, "lat": float(p["lat"]), "lon": float(p["lon"])})
            continue
        raise ApiError(422, f"point {p!r} is neither {{x, y}} nor {{lat, lon}}")
    return out


def _result(ev: dict, **extra) -> dict:
    return {"applied": ev["rule_id"], "op": ev["op"], "type": ev["type"],
            "generation": ev["generation"], "policy_hash": ev["policy_hash"], **extra}


def _build(model, data: dict):
    try:
        return model.model_validate(data)
    except ValidationError as e:
        raise ApiError(422, str(e)) from None


def _mission_layer_only(body: dict) -> None:
    """Refuse a body that raises its own layer above mission (module docstring,
    "THE LAYER RULE")."""
    layer = body.get("layer")
    if layer is not None and layer != DEFAULT_LAYER:
        raise ApiError(422, f"layer {layer!r} refused: an event over REST is a "
                            f"{DEFAULT_LAYER}-layer event, and nothing on this channel "
                            f"establishes that the sender speaks for the {layer} layer. "
                            f"Omit 'layer' (or send {DEFAULT_LAYER!r})")


# ------------------------------------------------------------------ handlers

def health(shield: Shield) -> dict:
    return {"status": "ok", "policy_id": shield.policy.policy_id,
            "policy_hash": shield.policy.policy_hash,
            "generation": shield.policy.generation,
            "mission_started": shield.mission_started,
            "fsm_state": shield.fsm.state.value if shield.fsm is not None else None,
            "hot_applicable": list(HOT_APPLICABLE)}


def apply_dynamic_nfz(shield: Shield, body: dict) -> dict:
    """dynamic_nfz "add"."""
    if not isinstance(body, dict):
        raise ApiError(422, "body must be a JSON object")
    _mission_layer_only(body)
    data = {**body, "type": "dynamic_nfz"}
    if "vertices" in data:
        data["vertices"] = _points(shield, data["vertices"])
    zone = _build(DynamicNFZ, data)
    try:
        return _result(shield.spawn_nfz(zone))
    except ValueError as e:
        raise _refuse(e) from None


def apply_nfz_edit(shield: Shield, zone_id: str, body: dict) -> dict:
    """dynamic_nfz "move" / "translate" / "rotate" / "scale"."""
    if not isinstance(body, dict):
        raise ApiError(422, "body must be a JSON object")
    _mission_layer_only(body)
    op = body.get("op")
    try:
        if op == "move":
            ev = shield.move_nfz(zone_id, _points(shield, body.get("vertices")),
                                 motion=body.get("motion", False))
        elif op == "translate":
            ev = shield.translate_nfz(zone_id, float(body.get("dx", 0.0)),
                                      float(body.get("dy", 0.0)))
        elif op == "rotate":
            ev = shield.rotate_nfz(zone_id, float(body["angle_deg"]))
        elif op == "scale":
            ev = shield.scale_nfz(zone_id, float(body["factor"]))
        else:
            raise ApiError(422, f"op {op!r}: one of move, translate, rotate, scale "
                                f"(DELETE expires a zone)")
    except (KeyError, TypeError) as e:
        raise ApiError(422, f"{op}: missing or malformed field {e}") from None
    except ApiError:
        raise
    except ValueError as e:
        raise _refuse(e) from None
    return _result(ev)


def apply_nfz_expire(shield: Shield, zone_id: str) -> dict:
    try:
        return _result(shield.expire_nfz(zone_id))
    except ValueError as e:
        raise _refuse(e) from None


def apply_time_window_switch(shield: Shield, body: dict) -> dict:
    if not isinstance(body, dict):
        raise ApiError(422, "body must be a JSON object")
    _mission_layer_only(body)
    data = {**body, "type": "time_window_switch"}
    if "id" not in data and "target_id" in data:
        data["id"] = f"switch-{data['target_id']}-g{shield.policy.generation + 1}"
    sw = _build(TimeWindowSwitch, data)
    try:
        return _result(shield.hot_apply(sw))
    except ValueError as e:
        raise _refuse(e) from None


def apply_corridor_swap(shield: Shield, body: dict) -> dict:
    if not isinstance(body, dict):
        raise ApiError(422, "body must be a JSON object")
    _mission_layer_only(body)
    data = {**body, "type": "corridor_swap"}
    if "centerline" in data:
        data["centerline"] = _points(shield, data["centerline"])
    sp = _build(CorridorSwap, data)
    try:
        return _result(shield.hot_apply(sp))
    except ValueError as e:
        raise _refuse(e) from None


def apply_event(shield: Shield, body: dict) -> dict:
    """Any hot-applicable event, by its DSL `type`. A locked class is refused
    (422) with the migration note."""
    if not isinstance(body, dict):
        raise ApiError(422, "body must be a JSON object")
    kind = body.get("type")
    if kind == "dynamic_nfz":
        return apply_dynamic_nfz(shield, body)
    if kind == "time_window_switch":
        return apply_time_window_switch(shield, body)
    if kind == "corridor_swap":
        return apply_corridor_swap(shield, body)
    if kind in ("polygon_fence", "circle_fence"):
        from .shield import MIGRATION_NOTE
        raise ApiError(422, f"{kind} cannot be hot-applied: {MIGRATION_NOTE}")
    raise ApiError(422, f"type {kind!r} cannot be hot-applied; the grant allows "
                        f"{', '.join(HOT_APPLICABLE)} (every other class is locked at "
                        f"mission start)")


def apply_legacy_nfz(shield: Shield, body: dict) -> dict:
    """POST /nfz, the 2026-07 body: now a dynamic_nfz spawn (same zone)."""
    try:
        req = NfzRequest.model_validate(body)
    except ValidationError as e:
        raise ApiError(422, str(e)) from None
    data = {"id": req.id, "vertices": [v.model_dump(exclude_none=True) for v in req.vertices],
            "altitude_floor_m": req.altitude_floor_m,
            "altitude_ceiling_m": req.altitude_ceiling_m, "margin_m": req.margin_m}
    res = apply_dynamic_nfz(shield, data)
    res["deprecated"] = ("POST /nfz now spawns a dynamic_nfz: polygon_fence is locked "
                         "at mission start. Use POST /dynamic_nfz.")
    return res


# ------------------------------------------------------------------ the app

def build_app(shield: Shield):
    from fastapi import Body, FastAPI, HTTPException

    app = FastAPI(title="Guardrail hot-apply API", version="0.2")
    # Events are serialised by the Shield's own lock; this one also orders the
    # read of hash + generation in /health against them.
    lock = threading.Lock()

    def call(fn, *args):
        with lock:
            try:
                return fn(shield, *args)
            except ApiError as e:
                raise HTTPException(e.status, e.detail) from None

    @app.get("/health")
    def get_health():
        return call(health)

    @app.get("/policy")
    def get_policy():
        with lock:
            return shield.policy.model_dump(mode="json")

    @app.get("/events")
    def get_events():
        with lock:
            return list(shield.events)

    @app.post("/dynamic_nfz")
    def post_dynamic_nfz(body: dict = Body(...)):
        return call(apply_dynamic_nfz, body)

    @app.patch("/dynamic_nfz/{zone_id}")
    def patch_dynamic_nfz(zone_id: str, body: dict = Body(...)):
        return call(apply_nfz_edit, zone_id, body)

    @app.delete("/dynamic_nfz/{zone_id}")
    def delete_dynamic_nfz(zone_id: str):
        return call(apply_nfz_expire, zone_id)

    @app.post("/time_window_switch")
    def post_switch(body: dict = Body(...)):
        return call(apply_time_window_switch, body)

    @app.post("/corridor_swap")
    def post_swap(body: dict = Body(...)):
        return call(apply_corridor_swap, body)

    @app.post("/hot_apply")
    def post_hot_apply(body: dict = Body(...)):
        return call(apply_event, body)

    @app.post("/nfz")
    def add_nfz(body: dict = Body(...)):
        return call(apply_legacy_nfz, body)

    return app


def serve_in_background(shield: Shield, host: str = "127.0.0.1",
                        port: int = 8071) -> threading.Thread:
    """Start uvicorn in a daemon thread so the 10 Hz mission loop stays the
    main thread. Returns the thread (daemon: dies with the mission)."""
    import uvicorn

    app = build_app(shield)
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    t = threading.Thread(target=server.run, daemon=True, name="guardrail-api")
    t.start()
    return t
