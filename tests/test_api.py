"""
REST hot-apply endpoint (guardrail/api.py): the grant's three hot-applicable
classes over HTTP, and the refusal of everything else.

    python tests/test_api.py      (no pytest needed)

Two layers, tested separately:

  * the request handling (`apply_*`), plain functions that need no FastAPI -
    these run on every interpreter;
  * the routes, through FastAPI's TestClient. FastAPI is installed in the
    3.11 env (vla-drone) and not in the 3.10 one (vla-real); there each route
    test prints SKIP, never PASS.

No test file imported guardrail.api before 2026-10-07 (audit card WP1-15):
POST /nfz had never run under a test. Shown failing first: against the
401305a api.py and shield.py, every test here fails or errors
(docs/DESIGN-shield-enforcement.md).
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import Action4D, Shield, State, load_policy  # noqa: E402
from guardrail import api as A                             # noqa: E402

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

DEMO = ROOT / "policies" / "demo_policy.yaml"
WGS = ROOT / "policies" / "wgs84_taipei.yaml"
SKIP = "SKIP"
ZONE = {"id": "nfz-landslide", **HARD_P0,
        "vertices": [{"x": -15, "y": -24}, {"x": -8, "y": -24},
                                            {"x": -8, "y": -16}, {"x": -15, "y": -16}],
        "altitude_floor_m": 0, "altitude_ceiling_m": 100, "margin_m": 1.0}


def _flying(path=DEMO) -> Shield:
    sh = Shield(load_policy(path))
    sh.filter(State(x=-20, y=-20, up=4), Action4D())        # the mission has started
    return sh


def _err(fn, *a):
    try:
        fn(*a)
    except A.ApiError as e:
        return e
    raise AssertionError(f"{fn.__name__} accepted {a[1:]}")


def _client(sh):
    try:
        from fastapi.testclient import TestClient
    except ImportError:
        return None
    return TestClient(A.build_app(sh))


# ------------------------------------------------------------ request handling

def test_a_dynamic_nfz_spawns_restamps_and_is_enforced():
    sh = _flying()
    h0 = sh.policy.policy_hash
    res = A.apply_dynamic_nfz(sh, dict(ZONE))
    assert res["type"] == "dynamic_nfz" and res["op"] == "spawn"
    assert res["generation"] == 1 and res["policy_hash"] == sh.policy.policy_hash != h0
    d = sh.filter(State(x=-20, y=-20, up=4), Action4D(vx=3.0))
    assert [v.rule_id for v in d.violations] == ["nfz-landslide"]


def test_the_legacy_nfz_body_now_spawns_a_dynamic_nfz():
    """POST /nfz built a polygon_fence - a class the grant locks at mission
    start. The same body now spawns the dynamic_nfz it stands for, and the
    response says so."""
    sh = _flying()
    res = A.apply_legacy_nfz(sh, dict(ZONE))
    assert res["type"] == "dynamic_nfz" and "deprecated" in res
    assert sh.policy.constraints[-1].type == "dynamic_nfz"
    assert [(v.x, v.y) for v in sh.policy.constraints[-1].vertices] == \
        [(p["x"], p["y"]) for p in ZONE["vertices"]]


def test_a_rule_body_must_state_the_four_constraintbase_fields():
    """IR schema 3.0.0 (2026-11-02): a REST body that is a DSL rule states
    constraint_type, priority and violation_action, as every rule must; until
    then a body without them spawned a hard P0 zone (failed then: each body
    below was accepted). IR schema 4.0.0 (2026-11-06): and scope, which an
    absent body field meant "mission" until then (failed then: the body
    without it was accepted). The deprecated POST /nfz keeps its fixed
    2026-07 body, which never had them, and spawns what it always meant:
    hard, P0, project_fix, for this mission."""
    sh = _flying()
    for field in ("constraint_type", "scope", "priority", "violation_action"):
        body = {k: v for k, v in ZONE.items() if k != field}
        e = _err(A.apply_dynamic_nfz, sh, body)
        assert e.status == 422 and field in e.detail, (field, e.status, e.detail)
        e = _err(A.apply_event, sh, {"type": "time_window_switch", "id": "sw",
                                     "target_id": "nfz-square", "active": False,
                                     **{k: v for k, v in HARD_P0.items() if k != field}})
        assert e.status == 422 and field in e.detail, (field, e.status, e.detail)
    assert sh.policy.generation == 0, "a refused body changed the policy"
    legacy = {k: ZONE[k] for k in ("id", "vertices", "altitude_floor_m",
                                   "altitude_ceiling_m", "margin_m")}
    res = A.apply_legacy_nfz(sh, legacy)
    z = sh.policy.constraints[-1]
    assert (z.constraint_type, z.priority, z.violation_action, z.scope) == \
        ("hard", "P0", "project_fix", "mission")
    assert "scope" in res["deprecated"] and "this mission" in res["deprecated"], res
    # A null scope is no scope.
    e = _err(A.apply_dynamic_nfz, sh, {**ZONE, "id": "null-scope", "scope": None})
    assert e.status == 422 and "scope" in e.detail, (e.status, e.detail)
    # `repair` is still read, as the grant's project_fix.
    A.apply_dynamic_nfz(sh, {**ZONE, "id": "old-spelling", "violation_action": "repair",
                             "vertices": [{"x": 40, "y": 40}, {"x": 45, "y": 40},
                                          {"x": 45, "y": 45}]})
    assert sh.policy.constraints[-1].violation_action == "project_fix"


def test_refusals_map_to_their_status():
    sh = _flying()
    A.apply_dynamic_nfz(sh, dict(ZONE))
    assert _err(A.apply_dynamic_nfz, sh, dict(ZONE)).status == 409           # id exists
    assert _err(A.apply_nfz_expire, sh, "nope").status == 404                # no such id
    bow = {**ZONE, "id": "bow", "vertices": [{"x": 0, "y": 0}, {"x": 4, "y": 4},
                                             {"x": 4, "y": 0}, {"x": 0, "y": 4}]}
    assert _err(A.apply_dynamic_nfz, sh, bow).status == 422                  # bow-tie
    e = _err(A.apply_event, sh, {**ZONE, "id": "pf", "type": "polygon_fence"})
    assert e.status == 422 and "locked at mission start" in e.detail
    assert _err(A.apply_event, sh, {"type": "altitude_envelope"}).status == 422
    assert _err(A.apply_nfz_edit, sh, "nfz-landslide", {"op": "wobble"}).status == 422
    assert _err(A.apply_nfz_edit, sh, "nfz-square", {"op": "translate", "dx": 1}).status == 422
    assert sh.policy.generation == 1                                         # nothing applied


def test_edits_switches_and_swaps_go_through_the_shield():
    sh = _flying()
    A.apply_dynamic_nfz(sh, dict(ZONE))
    assert A.apply_nfz_edit(sh, "nfz-landslide", {"op": "translate", "dx": 2})["op"] == "translate"
    assert A.apply_nfz_edit(sh, "nfz-landslide", {"op": "rotate", "angle_deg": 15})["op"] == "rotate"
    assert A.apply_nfz_edit(sh, "nfz-landslide", {"op": "scale", "factor": 0.5})["op"] == "scale"
    assert A.apply_nfz_edit(sh, "nfz-landslide", {
        "op": "move", "vertices": [[0, 40], [4, 40], [4, 44]]})["op"] == "move"
    assert A.apply_nfz_expire(sh, "nfz-landslide")["op"] == "expire"
    res = A.apply_time_window_switch(sh, {**HARD_P0, "target_id": "nfz-square", "active": False})
    assert res["op"] == "switch_off" and res["applied"].startswith("switch-nfz-square")
    assert not sh.filter(State(x=0, y=15, up=4), Action4D(vx=1.3)).touched
    assert sh.policy.generation == 7
    assert [e["op"] for e in sh.events] == ["spawn", "translate", "rotate", "scale",
                                            "move", "expire", "switch_off"]


def test_a_lat_lon_zone_lands_in_the_policys_own_frame():
    sh = _flying(WGS)
    o = sh.policy.frame_origin
    res = A.apply_dynamic_nfz(sh, {**HARD_P0, "id": "geo", "margin_m": 0.0, "vertices": [
        {"lat": o[0] + 0.001, "lon": o[1] + 0.001}, {"lat": o[0] + 0.002, "lon": o[1] + 0.001},
        {"lat": o[0] + 0.002, "lon": o[1] + 0.002}]})
    z = sh.policy.constraints[-1]
    assert res["applied"] == "geo" and z.vertices[0].lat == o[0] + 0.001
    assert 100 < z.vertices[0].x < 125 and 90 < z.vertices[0].y < 115     # ~111 m N, ~101 m E
    # A policy written in metres with no origin has no frame for a lat/lon
    # point. Every shipped file states an origin since 2026-11-03 (WP1-14);
    # demo_policy's metre form is the policy as it was.
    metres = Shield(load_policy(DEMO).metre_form())
    metres.filter(State(x=-20, y=-20, up=4), Action4D())
    assert _err(A.apply_dynamic_nfz, metres, {**HARD_P0, "id": "geo", "vertices": [
        {"lat": 25.0, "lon": 121.5}, {"lat": 25.001, "lon": 121.5},
        {"lat": 25.001, "lon": 121.501}]}).status == 422


BIG = "1" + "0" * 400                    # a JSON integer no float can hold


def _geo_square(**first) -> list[dict]:
    """Three WGS-84 vertices, the first overridden with `first`."""
    return [{"lat": 25.04, "lon": 121.56, **first}, {"lat": 25.041, "lon": 121.56},
            {"lat": 25.041, "lon": 121.561}]


def test_a_lat_lon_that_is_not_a_number_is_a_422_not_a_crash():
    """2026-10-19 bug hunt: on a policy with a frame, float() on a lat of
    "abc" or null (or an integer too big for a float) raised out of the
    handler, so FastAPI answered 500, the harness dry run INTERNAL and a
    forwarded body a retryable UNAVAILABLE - for a client's mistake. Shown
    failing on 1b95383 (raw ValueError / TypeError / OverflowError)."""
    sh = _flying(WGS)
    for bad in ("abc", None, int(BIG)):
        e = _err(A.apply_dynamic_nfz, sh, {**HARD_P0, "id": "g", "vertices": _geo_square(lat=bad)})
        assert e.status == 422 and "lat/lon must be numbers" in e.detail, e.detail
        e = _err(A.apply_event, sh, {"type": "dynamic_nfz", **HARD_P0, "id": "g",
                                     "vertices": _geo_square(lon=bad)})
        assert e.status == 422, e.detail
        e = _err(A.apply_corridor_swap, sh, {**HARD_P0, "id": "c", "target_id": "x", "width_m": 10,
                                             "centerline": _geo_square(lat=bad)[:2]})
        assert e.status == 422, e.detail
        e = _err(A.apply_nfz_edit, sh, "g", {"op": "move", "vertices": _geo_square(lat=bad)})
        assert e.status == 422, e.detail
    A.apply_dynamic_nfz(sh, {**HARD_P0, "id": "g", "margin_m": 0.0, "vertices": _geo_square()})
    for edit in ({"op": "translate", "dx": int(BIG)}, {"op": "rotate", "angle_deg": int(BIG)},
                 {"op": "scale", "factor": int(BIG)}):
        assert _err(A.apply_nfz_edit, sh, "g", edit).status == 422, edit
    assert sh.policy.generation == 1                     # only the good spawn


def test_routes_answer_422_for_a_lat_lon_that_is_not_a_number():
    sh = _flying(WGS)
    c = _client(sh)
    if c is None:
        return SKIP
    body = {**HARD_P0, "id": "g", "vertices": _geo_square(lat="abc")}
    for path, b in (("/dynamic_nfz", body), ("/hot_apply", {**body, "type": "dynamic_nfz"}),
                    ("/corridor_swap", {"id": "c", "target_id": "x", "width_m": 10,
                                        "centerline": _geo_square(lat=None)[:2]}),
                    ("/hot_apply", {"type": "corridor_swap", **HARD_P0, "id": "c", "target_id": "x",
                                    "width_m": 10, "centerline": _geo_square(lat=None)[:2]})):
        r = c.post(path, json=b)
        assert r.status_code == 422, (path, r.status_code, r.text)
    # json= cannot carry a 400-digit integer; the raw body can.
    raw = ('{"id": "g", "vertices": [{"lat": %s, "lon": 121.56}, {"lat": 25.041, '
           '"lon": 121.56}, {"lat": 25.041, "lon": 121.561}]}' % BIG)
    r = c.post("/dynamic_nfz", content=raw, headers={"Content-Type": "application/json"})
    assert r.status_code == 422, (r.status_code, r.text[:200])
    assert c.post("/dynamic_nfz", json={**HARD_P0, "id": "g", "margin_m": 0.0,
                                        "vertices": _geo_square()}).status_code == 200
    r = c.patch("/dynamic_nfz/g", content='{"op": "translate", "dx": %s}' % BIG,
                headers={"Content-Type": "application/json"})
    assert r.status_code == 422, (r.status_code, r.text[:200])
    assert c.get("/health").json()["generation"] == 1


def _layered_shield(tighten: bool = False) -> Shield:
    """A hard regulation-layer zone merged with a mission layer, flying. With
    `tighten` the mission layer also re-declares the zone, bigger (the
    tightening the grant allows), so the merged rule is the mission's copy."""
    from guardrail.models import Policy, merge_layers
    reg = Policy.model_validate({"policy_id": "reg", "version": "0.1.0", "constraints": [
        {"id": "nfz-airport", "type": "polygon_fence", **HARD_P0, "layer": "regulation",
         "vertices": [{"x": 7, "y": 7}, {"x": 23, "y": 7}, {"x": 23, "y": 23},
                      {"x": 7, "y": 23}]}]})
    mis_rules = [{"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 2, "alt_max_m": 6}]
    if tighten:
        mis_rules.append({"id": "nfz-airport", "type": "polygon_fence", **HARD_P0,
                          "vertices": [{"x": 6, "y": 6}, {"x": 24, "y": 6},
                                       {"x": 24, "y": 24}, {"x": 6, "y": 24}]})
    mis = Policy.model_validate({"policy_id": "mis", "version": "0.1.0",
                                 "constraints": mis_rules})
    sh = Shield(merge_layers([reg, mis]))
    sh.filter(State(x=0, y=15, up=4), Action4D(vx=4.0))
    return sh


def test_a_rest_event_cannot_relax_another_layers_hard_rule_or_claim_a_layer():
    """The review's probe: over REST, a switch-off of a hard regulation zone
    was accepted twice (once with "layer": "mission", once without) and a
    4 m/s command then flew into the zone unrepaired. Every REST event is a
    mission-layer event; the switch-off is refused (422), and so is a body
    that claims the regulation layer for itself. Nothing is applied and the
    zone is still enforced."""
    sh = _layered_shield()
    for body in ({**HARD_P0, "target_id": "nfz-airport", "active": False},
                 {**HARD_P0, "target_id": "nfz-airport", "active": False, "layer": "mission"},
                 {**HARD_P0, "target_id": "nfz-airport", "active": False, "layer": "regulation"}):
        e = _err(A.apply_time_window_switch, sh, dict(body))
        assert e.status == 422, (body, e.status, e.detail)
    e = _err(A.apply_event, sh, {"type": "time_window_switch", **HARD_P0, "target_id": "nfz-airport",
                                 "active": False, "layer": "regulation"})
    assert e.status == 422 and "layer 'regulation' refused" in e.detail
    e = _err(A.apply_dynamic_nfz, sh, {**ZONE, "layer": "site"})
    assert e.status == 422 and "layer 'site' refused" in e.detail
    assert sh.policy.generation == 0 and sh.events == []
    d = sh.filter(State(x=0, y=15, up=4), Action4D(vx=4.0))
    assert [v.rule_id for v in d.violations] == ["nfz-airport"] and d.repairs


def test_a_rest_switch_off_of_a_tightened_regulation_zone_is_refused():
    """Found 2026-10-18: once the mission layer had tightened the regulation
    zone (a legitimate merge), the merged rule was labelled "mission" and a
    plain REST switch-off was ACCEPTED (generation 1, switch_off); the 4 m/s
    command then flew in unrepaired. The merged rule now remembers the
    regulation copy it overrode (locked_by), so REST refuses with 422.
    Shown failing on 1b95383."""
    sh = _layered_shield(tighten=True)
    e = _err(A.apply_time_window_switch, sh, {**HARD_P0, "target_id": "nfz-airport", "active": False})
    assert e.status == 422 and "regulation-layer" in e.detail, (e.status, e.detail)
    assert sh.policy.generation == 0 and sh.events == []
    d = sh.filter(State(x=0, y=15, up=4), Action4D(vx=4.0))
    assert [v.rule_id for v in d.violations] == ["nfz-airport"] and d.repairs


def test_health_reports_the_generation_and_the_fsm():
    sh = _flying()
    h = A.health(sh)
    assert h["generation"] == 0 and h["mission_started"] is True
    assert h["fsm_state"] == "Normal" and h["hot_applicable"] == [
        "dynamic_nfz", "time_window_switch", "corridor_swap"]


# ---------------------------------------------------------------------- routes

def test_routes_spawn_edit_expire_and_refuse_over_http():
    sh = _flying()
    c = _client(sh)
    if c is None:
        return SKIP
    r = c.post("/dynamic_nfz", json=ZONE)
    assert r.status_code == 200 and r.json()["generation"] == 1, r.text
    assert c.post("/dynamic_nfz", json=ZONE).status_code == 409
    r = c.patch("/dynamic_nfz/nfz-landslide", json={"op": "translate", "dx": 1, "dy": 0})
    assert r.status_code == 200 and r.json()["generation"] == 2
    assert c.delete("/dynamic_nfz/nfz-landslide").status_code == 200
    assert c.delete("/dynamic_nfz/nfz-landslide").status_code == 404
    r = c.post("/hot_apply", json={**ZONE, "id": "pf", "type": "polygon_fence"})
    assert r.status_code == 422 and "locked at mission start" in r.json()["detail"]
    assert c.get("/health").json()["generation"] == 3
    assert [e["op"] for e in c.get("/events").json()] == ["spawn", "translate", "expire"]


def test_routes_switch_swap_and_the_legacy_body():
    from guardrail.models import Policy
    # A frame origin stated: since 2026-11-18 (WP1-14) a swap's metres are
    # written as WGS84 about it, and a policy with no frame refuses them (422).
    pol = Policy.model_validate({"policy_id": "lanes",
                                 "origin": {"lat": 35.6895, "lon": 139.6917},
                                 "constraints": [
        {"id": "lane-a", "type": "corridor", **HARD_P0, "width_m": 10,
         "centerline": [{"x": -100, "y": 0}, {"x": 100, "y": 0}]}]})
    sh = Shield(pol)
    c = _client(sh)
    if c is None:
        return SKIP
    r = c.post("/corridor_swap", json={**HARD_P0, "id": "lane-b", "target_id": "lane-a", "width_m": 10,
                                       "centerline": [{"x": -100, "y": 40}, {"x": 100, "y": 40}]})
    assert r.status_code == 200, r.text
    d = sh.filter(State(x=0, y=0, up=10), Action4D(vx=2.0))
    assert [v.rule_id for v in d.violations] == ["lane-b"]
    r = c.post("/time_window_switch", json={**HARD_P0, "target_id": "lane-b", "active": False})
    assert r.status_code == 422                     # a swap is an event, not a rule
    r = c.post("/nfz", json={**ZONE, "id": "old-style"})
    assert r.status_code == 200 and r.json()["type"] == "dynamic_nfz"
    assert c.get("/policy").json()["constraints"][-1]["type"] == "dynamic_nfz"


def test_routes_refuse_a_relaxation_across_layers():
    sh = _layered_shield()
    c = _client(sh)
    if c is None:
        return SKIP
    r = c.post("/time_window_switch", json={**HARD_P0, "target_id": "nfz-airport", "active": False})
    assert r.status_code == 422 and "hard regulation-layer rule" in r.json()["detail"], r.text
    r = c.post("/hot_apply", json={"type": "time_window_switch", **HARD_P0, "target_id": "nfz-airport",
                                   "active": False, "layer": "regulation"})
    assert r.status_code == 422, r.text
    assert c.get("/health").json()["generation"] == 0


def test_routes_refuse_a_zone_the_shield_would_not_enforce_or_build():
    """REGRESSION (2026-10-19). An altitude_ref MSL zone was accepted (200,
    generation 1) and then enforced as metres above ground; a margin_m of
    1e308 passed validation, failed to compile inside the Shield after the
    policy had already been changed, and answered 500 with /health already
    on generation 1. Both are 422 now, and nothing is applied."""
    sh = _flying()
    c = _client(sh)
    if c is None:
        return SKIP
    r = c.post("/dynamic_nfz", json={**ZONE, "altitude_ref": "MSL"})
    assert r.status_code == 422 and "MSL" in r.json()["detail"], r.text
    r = c.post("/dynamic_nfz", json={**ZONE, "margin_m": 1e308})
    assert r.status_code == 422 and "could not compile" in r.json()["detail"], r.text
    assert c.get("/health").json()["generation"] == 0
    assert c.post("/dynamic_nfz", json=ZONE).status_code == 200


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs fastapi: run it in the 3.11 env)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}  {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}  {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
