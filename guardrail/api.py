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

ON A FLYING NODE (2026-10-12). Until this date only demo/run_demo.py served
this API; no SITL rail did, so the grant's hot-apply channel never reached an
aircraft flown by ArduPilot. `FlightHotApply` puts it on a flying Shield
(sitl/ros2_shield_node.py and sitl/run_sitl_demo.py, `--hotapply-port`):

  * the routes and bodies above are unchanged; `build_app` takes three
    optional hooks. `lock` is the rail's own lock (an RLock), held by its
    control tick, so a swap and a tick never interleave; `on_applied(result,
    received_ns=, applied_ns=)` runs under it after each accepted event (the
    rail records the generation and re-publishes its fences); `refuse_when()`
    names why the episode takes no more events (it ended), answered 503
    before the Shield is touched;
  * the listener binds 127.0.0.1 by default. Any other address needs mutual
    TLS with the stress harness's lab CA (guardrail/harness_service.py
    gen-pki): the server presents <server_name>.cert.pem, requires a client
    certificate signed by ca.cert.pem, and drops a connection whose common
    name is not on the allow-list (default harness-client), as the gRPC
    harness does - the same CA will sign every SROS 2 node's identity, so a
    valid chain alone is not enough;
  * each accepted event's receipt-to-swap time (handler entry, before the
    lock, to the Shield's swap) is recorded beside the grant's 50 ms target
    for a dynamic_nfz spawn (Policy DSL page, "Hot-apply latency").
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable

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

log = logging.getLogger("guardrail.api")


def build_app(shield: Shield, *, lock=None,
              on_applied: Callable[..., Any] | None = None,
              refuse_when: Callable[[], str | None] | None = None):
    """The FastAPI app. The three keywords are for a flying node (module
    docstring, "On a flying node"); without them it is the app it always was.

    `lock` serialises every route against the caller's own use of the Shield
    (pass an RLock when `on_applied` takes it again). `on_applied(result,
    received_ns=, applied_ns=)` runs under it after an accepted event, with
    time.monotonic_ns() at handler entry and right after the swap; an
    exception it raises is logged, never turned into a refusal, because the
    Shield has already swapped. `refuse_when()` returning a reason refuses a
    mutating request with 503 before the Shield is touched."""
    from fastapi import Body, FastAPI, HTTPException

    app = FastAPI(title="Guardrail hot-apply API", version="0.2")
    # Events are serialised by the Shield's own lock; this one also orders the
    # read of hash + generation in /health against them.
    lock = lock if lock is not None else threading.Lock()

    def call(fn, *args, mutating: bool = False):
        received = time.monotonic_ns()
        with lock:
            if mutating and refuse_when is not None:
                why = refuse_when()
                if why:
                    raise HTTPException(503, why)
            try:
                res = fn(shield, *args)
            except ApiError as e:
                raise HTTPException(e.status, e.detail) from None
            if mutating and on_applied is not None:
                applied = time.monotonic_ns()
                try:
                    on_applied(res, received_ns=received, applied_ns=applied)
                except Exception:                          # noqa: BLE001
                    log.exception("on_applied failed after %s was applied",
                                  res.get("applied"))
            return res

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
        return call(apply_dynamic_nfz, body, mutating=True)

    @app.patch("/dynamic_nfz/{zone_id}")
    def patch_dynamic_nfz(zone_id: str, body: dict = Body(...)):
        return call(apply_nfz_edit, zone_id, body, mutating=True)

    @app.delete("/dynamic_nfz/{zone_id}")
    def delete_dynamic_nfz(zone_id: str):
        return call(apply_nfz_expire, zone_id, mutating=True)

    @app.post("/time_window_switch")
    def post_switch(body: dict = Body(...)):
        return call(apply_time_window_switch, body, mutating=True)

    @app.post("/corridor_swap")
    def post_swap(body: dict = Body(...)):
        return call(apply_corridor_swap, body, mutating=True)

    @app.post("/hot_apply")
    def post_hot_apply(body: dict = Body(...)):
        return call(apply_event, body, mutating=True)

    @app.post("/nfz")
    def add_nfz(body: dict = Body(...)):
        return call(apply_legacy_nfz, body, mutating=True)

    return app


def peer_common_name(cert: dict | None) -> str | None:
    """The subject common name of a peer certificate as ssl.getpeercert()
    returns it, or None."""
    for rdn in (cert or {}).get("subject", ()):
        for key, value in rdn:
            if key == "commonName":
                return value
    return None


def _common_name_gate(allowed: set[str]):
    """uvicorn's own HTTP protocol, closing a TLS connection whose client
    certificate's common name is not in `allowed` (the CA's chain was already
    verified by the handshake: ssl.CERT_REQUIRED)."""
    from uvicorn.config import HTTP_PROTOCOLS
    from uvicorn.importer import import_from_string
    base = import_from_string(HTTP_PROTOCOLS["auto"])

    class CommonNameGate(base):
        def connection_made(self, transport):
            super().connection_made(transport)
            cn = peer_common_name(transport.get_extra_info("peercert"))
            if cn not in allowed:
                log.warning("hot-apply: client certificate %r is not on the "
                            "allow-list %s; connection closed", cn, sorted(allowed))
                transport.close()
    return CommonNameGate


def serve_in_background(shield: Shield, host: str = "127.0.0.1",
                        port: int = 8071, *, lock=None,
                        on_applied: Callable[..., Any] | None = None,
                        refuse_when: Callable[[], str | None] | None = None,
                        pki_dir=None, server_name: str | None = None,
                        allowed_clients=None,
                        wait_s: float = 5.0) -> threading.Thread:
    """Start uvicorn in a daemon thread so the 10 Hz mission loop stays the
    main thread. Returns the thread (daemon: dies with the mission) once the
    server listens (RuntimeError if it has not within `wait_s`); its
    `server` attribute is the uvicorn.Server (`server.should_exit = True`
    stops it) and `url` the address it serves.

    Plaintext on a loopback address only. `pki_dir` (a folder written by
    `python -m guardrail.harness_service gen-pki`) serves mutual TLS:
    <server_name>.cert.pem (default harness-server), a client certificate
    signed by its ca.cert.pem required, and its common name required to be
    in `allowed_clients` (default harness-client)."""
    import ssl as _ssl

    import uvicorn

    from .harness_service import (DEFAULT_CLIENT_CN, SERVER_CN, is_loopback,
                                  pki_paths)
    if pki_dir is None and not is_loopback(host):
        raise ValueError(f"refusing a plaintext hot-apply listener on {host!r}: "
                         f"plaintext is for loopback only; give pki_dir (the "
                         f"harness CA, guardrail.harness_service gen-pki) for "
                         f"mutual TLS")
    app = build_app(shield, lock=lock, on_applied=on_applied,
                    refuse_when=refuse_when)
    kw: dict[str, Any] = {}
    if pki_dir is not None:
        allowed = set(allowed_clients or (DEFAULT_CLIENT_CN,))
        ca, cert, key = pki_paths(pki_dir, server_name or SERVER_CN)
        for f in (ca, cert, key):
            if not f.is_file():
                raise FileNotFoundError(f"{f} is missing: write the lab PKI with "
                                        f"python -m guardrail.harness_service gen-pki")
        kw = {"ssl_certfile": str(cert), "ssl_keyfile": str(key),
              "ssl_ca_certs": str(ca), "ssl_cert_reqs": _ssl.CERT_REQUIRED,
              "http": _common_name_gate(allowed)}
    config = uvicorn.Config(app, host=host, port=port, log_level="warning", **kw)
    server = uvicorn.Server(config)
    t = threading.Thread(target=server.run, daemon=True, name="guardrail-api")
    t.server = server
    shown = f"[{host}]" if ":" in host else host
    t.url = f"{'https' if pki_dir is not None else 'http'}://{shown}:{port}"
    t.start()
    # uvicorn binds on its own thread and only logs a failure there (a taken
    # port, a bad certificate). Wait for it, so a listener that is not
    # listening is an error here and not a silent absence.
    deadline = time.monotonic() + wait_s
    while not server.started and t.is_alive() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not server.started:
        server.should_exit = True
        raise RuntimeError(f"the hot-apply listener did not start on {t.url} "
                           f"within {wait_s:g} s (port taken, or a TLS file "
                           f"refused; uvicorn's log says which)")
    return t


# ------------------------------------------------------- on a flying node

# The grant's hot-apply latency target for a dynamic_nfz spawn (Policy DSL
# page, "Performance targets": <= 50 ms).
HOT_APPLY_TARGET_MS = 50.0


class FlightHotApply:
    """The REST hot-apply listener on a FLYING Shield: one per episode.

        hot = FlightHotApply(shield, rec, lock, mission_t=node_t, after=refence)
        hot.start("127.0.0.1", 8071)                    # or pki_dir=... for mTLS
        ...                                             # ticks take `lock`
        hot.close("episode ended")                      # later events: 503
        metrics["hot_apply"] = hot.summary()

    `record` is the rail's guardrail.replay.EpisodeRecord: each accepted event
    writes the new generation's policy_g<N>.json and csp_g<N>.json
    (`hot_applied`) and a `hot_apply_rest` event with its receipt-to-swap
    time. `after(result, t)` runs next, under the lock (the ROS node
    re-publishes its fences there, keeping a zone the aircraft is near held
    back). `mission_t()` is the rail's mission time, or None before take-off.
    A dynamic_nfz spawned this way is named in `live_from` with its mission
    time on the trajectory's 0.01 s grid, for guardrail.replay.zone_dwell.
    """

    def __init__(self, shield: Shield, record, lock, *,
                 mission_t: Callable[[], float | None],
                 after: Callable[[dict, float | None], None] | None = None,
                 log_fn: Callable[[str], None] | None = None) -> None:
        self.shield = shield
        self.rec = record
        self.lock = lock
        self.mission_t = mission_t
        self.after = after
        self.log = log_fn or (lambda msg: log.info("%s", msg))
        self.applied: list[dict] = []
        self.live_from: dict[str, float] = {}
        self.closed: str | None = None
        self.thread: threading.Thread | None = None
        self.endpoint: dict | None = None

    def refuse_when(self) -> str | None:
        return self.closed

    def on_applied(self, result: dict, *, received_ns: int,
                   applied_ns: int) -> dict:
        t = self.mission_t()
        entry = {"op": result["op"], "type": result["type"],
                 "rule_id": result["applied"],
                 "generation": result["generation"],
                 "policy_hash": result["policy_hash"],
                 "receipt_to_swap_ms": round((applied_ns - received_ns) / 1e6, 3)}
        self.rec.hot_applied(result["applied"], t=t)
        if entry["op"] == "spawn" and entry["type"] == "dynamic_nfz" \
                and t is not None:
            self.live_from[entry["rule_id"]] = round(t, 2)
        self.applied.append({"t": None if t is None else round(t, 3), **entry})
        self.rec.event("hot_apply_rest", t=t, **entry)
        if self.after is not None:
            self.after(result, t)
        self.log(f"hot-apply over REST: {entry['op']} {entry['type']} "
                 f"{entry['rule_id']!r} -> generation {entry['generation']}, "
                 f"receipt to swap {entry['receipt_to_swap_ms']:.2f} ms")
        return entry

    def start(self, host: str = "127.0.0.1", port: int = 8071, *,
              pki_dir=None, server_name: str | None = None,
              allowed_clients=None) -> threading.Thread:
        self.thread = serve_in_background(
            self.shield, host, port, lock=self.lock, on_applied=self.on_applied,
            refuse_when=self.refuse_when, pki_dir=pki_dir,
            server_name=server_name, allowed_clients=allowed_clients)
        self.endpoint = {"url": self.thread.url, "mutual_tls": pki_dir is not None,
                         "pki_dir": None if pki_dir is None else str(pki_dir),
                         "allowed_clients": (sorted(allowed_clients)
                                             if pki_dir is not None and allowed_clients
                                             else None)}
        self.rec.event("hot_apply_listener", **self.endpoint)
        return self.thread

    def close(self, why: str) -> None:
        """No more events (they would land after the episode's records are
        written): later requests get 503 with `why`, and the server stops."""
        with self.lock:
            self.closed = why
        if self.thread is not None:
            self.thread.server.should_exit = True

    def summary(self) -> dict:
        lat = sorted(e["receipt_to_swap_ms"] for e in self.applied)

        def pct(q):                     # nearest rank, as the rails' percentile
            if not lat:
                return None
            return lat[min(len(lat) - 1, max(0, math.ceil(q / 100 * len(lat)) - 1))]
        spawns = [e["receipt_to_swap_ms"] for e in self.applied
                  if e["op"] == "spawn"]
        return {"endpoint": self.endpoint, "n_applied": len(self.applied),
                "events": list(self.applied),
                "receipt_to_swap_ms_p50": pct(50),
                "receipt_to_swap_ms_max": lat[-1] if lat else None,
                "spawn_target_ms": HOT_APPLY_TARGET_MS,
                "spawns_over_target": sum(1 for v in spawns
                                          if v > HOT_APPLY_TARGET_MS),
                "measured": "handler entry (before the rail's lock) to the "
                            "Shield's swap; the generation's policy and CSP "
                            "are written after it, still under the lock"}
