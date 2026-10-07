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

DEMO = ROOT / "policies" / "demo_policy.yaml"
WGS = ROOT / "policies" / "wgs84_taipei.yaml"
SKIP = "SKIP"
ZONE = {"id": "nfz-landslide", "vertices": [{"x": -15, "y": -24}, {"x": -8, "y": -24},
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
    res = A.apply_time_window_switch(sh, {"target_id": "nfz-square", "active": False})
    assert res["op"] == "switch_off" and res["applied"].startswith("switch-nfz-square")
    assert not sh.filter(State(x=0, y=15, up=4), Action4D(vx=1.3)).touched
    assert sh.policy.generation == 7
    assert [e["op"] for e in sh.events] == ["spawn", "translate", "rotate", "scale",
                                            "move", "expire", "switch_off"]


def test_a_lat_lon_zone_lands_in_the_policys_own_frame():
    sh = _flying(WGS)
    o = sh.policy.frame_origin
    res = A.apply_dynamic_nfz(sh, {"id": "geo", "margin_m": 0.0, "vertices": [
        {"lat": o[0] + 0.001, "lon": o[1] + 0.001}, {"lat": o[0] + 0.002, "lon": o[1] + 0.001},
        {"lat": o[0] + 0.002, "lon": o[1] + 0.002}]})
    z = sh.policy.constraints[-1]
    assert res["applied"] == "geo" and z.vertices[0].lat == o[0] + 0.001
    assert 100 < z.vertices[0].x < 125 and 90 < z.vertices[0].y < 115     # ~111 m N, ~101 m E
    metres = _flying(DEMO)
    assert _err(A.apply_dynamic_nfz, metres, {"id": "geo", "vertices": [
        {"lat": 25.0, "lon": 121.5}, {"lat": 25.001, "lon": 121.5},
        {"lat": 25.001, "lon": 121.501}]}).status == 422


def _layered_shield() -> Shield:
    """A hard regulation-layer zone merged with a mission layer, flying."""
    from guardrail.models import Policy, merge_layers
    reg = Policy.model_validate({"policy_id": "reg", "version": "0.1.0", "constraints": [
        {"id": "nfz-airport", "type": "polygon_fence", "layer": "regulation",
         "vertices": [{"x": 7, "y": 7}, {"x": 23, "y": 7}, {"x": 23, "y": 23},
                      {"x": 7, "y": 23}]}]})
    mis = Policy.model_validate({"policy_id": "mis", "version": "0.1.0", "constraints": [
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 2, "alt_max_m": 6}]})
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
    for body in ({"target_id": "nfz-airport", "active": False},
                 {"target_id": "nfz-airport", "active": False, "layer": "mission"},
                 {"target_id": "nfz-airport", "active": False, "layer": "regulation"}):
        e = _err(A.apply_time_window_switch, sh, dict(body))
        assert e.status == 422, (body, e.status, e.detail)
    e = _err(A.apply_event, sh, {"type": "time_window_switch", "target_id": "nfz-airport",
                                 "active": False, "layer": "regulation"})
    assert e.status == 422 and "layer 'regulation' refused" in e.detail
    e = _err(A.apply_dynamic_nfz, sh, {**ZONE, "layer": "site"})
    assert e.status == 422 and "layer 'site' refused" in e.detail
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
    pol = Policy.model_validate({"policy_id": "lanes", "constraints": [
        {"id": "lane-a", "type": "corridor", "width_m": 10,
         "centerline": [{"x": -100, "y": 0}, {"x": 100, "y": 0}]}]})
    sh = Shield(pol)
    c = _client(sh)
    if c is None:
        return SKIP
    r = c.post("/corridor_swap", json={"id": "lane-b", "target_id": "lane-a", "width_m": 10,
                                       "centerline": [{"x": -100, "y": 40}, {"x": 100, "y": 40}]})
    assert r.status_code == 200, r.text
    d = sh.filter(State(x=0, y=0, up=10), Action4D(vx=2.0))
    assert [v.rule_id for v in d.violations] == ["lane-b"]
    r = c.post("/time_window_switch", json={"target_id": "lane-b", "active": False})
    assert r.status_code == 422                     # a swap is an event, not a rule
    r = c.post("/nfz", json={**ZONE, "id": "old-style"})
    assert r.status_code == 200 and r.json()["type"] == "dynamic_nfz"
    assert c.get("/policy").json()["constraints"][-1]["type"] == "dynamic_nfz"


def test_routes_refuse_a_relaxation_across_layers():
    sh = _layered_shield()
    c = _client(sh)
    if c is None:
        return SKIP
    r = c.post("/time_window_switch", json={"target_id": "nfz-airport", "active": False})
    assert r.status_code == 422 and "hard regulation-layer rule" in r.json()["detail"], r.text
    r = c.post("/hot_apply", json={"type": "time_window_switch", "target_id": "nfz-airport",
                                   "active": False, "layer": "regulation"})
    assert r.status_code == 422, r.text
    assert c.get("/health").json()["generation"] == 0


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
