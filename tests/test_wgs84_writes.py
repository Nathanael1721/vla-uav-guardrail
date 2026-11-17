"""
WP1-14: what an event WRITES into a policy with a frame is WGS84 (2026-11-17).

    python tests/test_wgs84_writes.py      (no pytest needed)

Since 2026-11-03 every stored policy is WGS84 with a stated origin, but the
REST hot-apply endpoint (guardrail/api.py), the Shield's own zone edits and
the harness's spawned zones (guardrail/scenario_events.py) still wrote metre
points into the live policy, so a generation's snapshot - stored, hashed and
signed - carried projected coordinates, against "the DSL itself never carries
projected coordinates" (grant, Policy DSL page). The Shield now writes every
rule an event adds or rewrites in WGS84 about the policy's frame origin
(Shield._in_frame, models.rule_in_wgs84). What is checked here:

  * every write path stores WGS84 only: REST POST /dynamic_nfz, PATCH
    move / translate / rotate / scale, /corridor_swap, a pre-start
    polygon_fence / circle_fence, the harness's spawn / translate / rotate /
    scale / swap events; the snapshot reloads to the same hash and metres;
  * the round trip metres -> WGS84 -> metres is within 1 mm (in fact under a
    micrometre), and exact for metres on the micrometre grid;
  * the enforcement geometry is what it was: the live Shield compiles the
    same zone rings, float for float, builds the same CSP and takes the same
    decisions as with the metre writes of before (`wgs84_writes = False`) -
    computed, off-grid rings included; the snapshot reloads to the same hash
    and the same metres (exact on the micrometre grid, else within 0.6 um);
  * a policy written only in metres (the legacy form) keeps metre writes, and
    no stored policy's hash moves;
  * a derived frame (no stated origin) is never moved by an event;
  * the declared deviation of the default mode (live view = the metres
    given, snapshot view = the micrometre view of the stored lat/lon, up to
    0.6 um apart under one hash) and the opt-in that closes it
    (Shield.wgs84_bind_frame, default off: live view = snapshot view, bit
    for bit);
  * bundle.derive, tools/geo_to_policy.py ingest and EpisodeRecord.apply_zone
    agree with the Shield's writes; the 1-um consistency guard fires.

The REST checks call the guardrail.api functions the routes call
(api.apply_dynamic_nfz, apply_nfz_edit, apply_corridor_swap); the FastAPI
routes themselves are exercised by tests/test_api.py where FastAPI is
installed.
"""
from __future__ import annotations

import copy
import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import api                                          # noqa: E402
from guardrail.compiler import ConstraintCompiler, Mission         # noqa: E402
from guardrail.models import (Action4D, DynamicNFZ, Policy, State,  # noqa: E402
                              _rule_points, load_policy, rule_in_wgs84)
from guardrail.projection import (LocalProjection, local_metres,  # noqa: E402
                                  metres_to_wgs84)
from guardrail.scenario_events import EventState, apply_event      # noqa: E402
from guardrail.scenario_spec import ScenarioEvent                  # noqa: E402
from guardrail.shield import HotApplyRefused, Shield               # noqa: E402

SITL = ROOT / "policies" / "sim_demo_policy.yaml"      # CMAC origin, SITL rails
SURVEY = ROOT / "policies" / "corridor_survey.yaml"    # a corridor, PAS origin

RULE = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
        "violation_action": "project_fix"}
ZONE = {"id": "nfz-landslide-007", **RULE,
        "vertices": [{"x": 23, "y": 6}, {"x": 31, "y": 6}, {"x": 31, "y": 14}],
        "altitude_floor_m": 0, "altitude_ceiling_m": 100, "margin_m": 1.0}


def _pair(path=SITL):
    """Two Shields over the same stored policy: the WGS84 writes (default)
    and the metre writes of before 2026-11-17. Both in the default
    (unbound) mode whatever VLAGUARD_WGS84_BIND says: these tests are about
    that mode; the bound mode has its own below."""
    new = Shield(load_policy(path))
    old = Shield(load_policy(path))
    new.wgs84_bind_frame = old.wgs84_bind_frame = False
    old.wgs84_writes = False
    return new, old


def _stored(policy: Policy) -> dict:
    return policy.canonical_ir()


def _has_xy(node) -> bool:
    if isinstance(node, dict):
        return ("x" in node or "y" in node) or any(_has_xy(v) for v in node.values())
    if isinstance(node, list):
        return any(_has_xy(v) for v in node)
    return False


def _all_wgs84(policy: Policy) -> None:
    """No metre point anywhere in the policy, and none in what it hashes."""
    metre = [v for pts in _rule_points(policy, all_points=True) for v in pts
             if not v.geographic]
    assert not metre, f"metre points stored: {metre}"
    assert not _has_xy(_stored(policy)["constraints"]), "x/y in the canonical IR"


def _reload_same(policy: Policy) -> None:
    """The generation's snapshot (a run's policy_g<N>.json) loads back to the
    same hash, in the same frame, with the same metres: exactly where the
    live metres are on the micrometre grid, else within 0.6 um (the reload
    reads the policy's micrometre view of lat/lon written to 12 decimals)."""
    back = Policy.model_validate(json.loads(policy.model_dump_json()))
    assert back.policy_hash == policy.policy_hash
    assert back.frame_origin == policy.frame_origin
    for a, b in zip(_rule_points(back, all_points=True),
                    _rule_points(policy, all_points=True)):
        for u, v in zip(a, b):
            for p, q in ((u.x, v.x), (u.y, v.y)):
                if round(q, 6) == q:
                    assert p == q, (p, q)
                else:
                    assert abs(p - q) <= 0.6e-6, (p, q)


def _rings(shield: Shield) -> list:
    """Every compiled zone ring (polygon and margin ring) the Shield enforces."""
    out = []
    for r in shield.ir.fences:
        out.append((r.rule.id, list(r.polygon.exterior.coords),
                    list(r.buffered.exterior.coords)))
    return out


def _close(a, b, tol) -> bool:
    if isinstance(a, (list, tuple)):
        return len(a) == len(b) and all(_close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, float) or isinstance(b, float):
        return abs(a - b) <= tol
    return a == b


# ---------------------------------------------------------------- round trip

def test_roundtrip_metres_wgs84_metres():
    rng = random.Random(1117)
    worst = 0.0
    for origin in ((-35.363261, 149.165230), (35.6895, 139.6917), (25.0330, 121.5654)):
        proj = LocalProjection(*origin)
        for _ in range(2000):
            x, y = rng.uniform(-2000, 2000), rng.uniform(-2000, 2000)
            lat, lon = metres_to_wgs84(origin, x, y)
            n, e = proj.to_local(lat, lon)
            worst = max(worst, abs(n - x), abs(e - y))
            # the policy's view: exact on the micrometre grid ...
            gx, gy = round(x, 6) + 0.0, round(y, 6) + 0.0
            assert local_metres(origin, *metres_to_wgs84(origin, gx, gy)) == (gx, gy)
            # ... and within the micrometre anywhere
            vx, vy = local_metres(origin, lat, lon)
            assert abs(vx - x) <= 1e-6 and abs(vy - y) <= 1e-6, (x, y, vx, vy)
    assert worst < 1e-3, worst            # the row's tolerance: 1 mm
    assert worst < 1e-6, worst            # what it is: under a micrometre


# ---------------------------------------------------------------- REST paths

def test_rest_spawn_stores_wgs84_same_zone():
    new, old = _pair()
    h0 = new.policy.policy_hash
    api.apply_dynamic_nfz(new, copy.deepcopy(ZONE))
    api.apply_dynamic_nfz(old, copy.deepcopy(ZONE))
    z = new.policy.constraints[-1]
    assert all(v.geographic for v in z.vertices)
    assert [(v.x, v.y) for v in z.vertices] == \
        [(v.x, v.y) for v in old.policy.constraints[-1].vertices]
    _all_wgs84(new.policy)
    _reload_same(new.policy)
    assert new.policy.policy_hash != h0
    # the metre writes of before put x/y into the hash
    assert _has_xy(_stored(old.policy)["constraints"])
    assert _rings(new) == _rings(old)


def test_rest_patch_edits_store_wgs84():
    new, old = _pair()
    for sh in (new, old):
        api.apply_dynamic_nfz(sh, copy.deepcopy(ZONE))
    edits = [{"op": "translate", "dx": 5, "dy": -2.5},
             {"op": "move", "vertices": [{"x": 40, "y": 0}, {"x": 48, "y": 0},
                                         {"x": 48, "y": 9.25}, {"x": 40, "y": 9.25}]},
             {"op": "scale", "factor": 1.5},
             {"op": "rotate", "angle_deg": 30}]
    for body in edits:
        for sh in (new, old):
            api.apply_nfz_edit(sh, ZONE["id"], dict(body))
        _all_wgs84(new.policy)
        _reload_same(new.policy)
        # the live rings: the same floats, the computed (rotated) ring too
        assert _rings(new) == _rings(old), body
    a = [(v.x, v.y) for v in new.policy.constraints[-1].given_vertices()]
    b = [(v.x, v.y) for v in old.policy.constraints[-1].given_vertices()]
    assert a == b, (a, b)
    assert any(round(c, 6) != c for xy in a for c in xy), "rotate wrote no off-grid metre"


def test_rest_lat_lon_spawn_unchanged():
    """A body already in lat/lon is stored as before (lat/lon only)."""
    new, old = _pair()
    o = new.policy.frame_origin
    verts = [dict(zip(("lat", "lon"), metres_to_wgs84(o, x, y)))
             for x, y in ((10, 10), (18, 10), (18, 17))]
    for sh in (new, old):
        api.apply_dynamic_nfz(sh, {**ZONE, "id": "ll", "vertices": verts})
    assert _stored(new.policy) == _stored(old.policy)
    assert new.policy.policy_hash == old.policy.policy_hash
    _all_wgs84(new.policy)


def test_rest_corridor_swap_stores_wgs84():
    new, old = _pair(SURVEY)
    body = {"id": "swap-1", "target_id": "corridor-survey-route", **RULE,
            "centerline": [{"x": 0, "y": 0}, {"x": 60, "y": 0}, {"x": 60, "y": 75.5}],
            "width_m": 30, "altitude_floor_m": 10, "altitude_ceiling_m": 60}
    for sh in (new, old):
        api.apply_corridor_swap(sh, copy.deepcopy(body))
    sp = new.policy.constraints[-1]
    assert sp.type == "corridor_swap" and all(v.geographic for v in sp.centerline)
    assert list(sp.points()) == list(old.policy.constraints[-1].points())
    _all_wgs84(new.policy)
    _reload_same(new.policy)


def test_prestart_fences_store_wgs84():
    new, old = _pair()
    fence = {"id": "pre-1", "type": "polygon_fence", **RULE,
             "vertices": [{"x": -30, "y": -30}, {"x": -20, "y": -30}, {"x": -20, "y": -21}]}
    circle = {"id": "pre-2", "type": "circle_fence", **RULE,
              "center": {"x": -40.5, "y": 12}, "radius_m": 6}
    from guardrail.models import CircleFence, PolygonFence
    for sh in (new, old):
        sh.hot_apply(PolygonFence.model_validate(fence))
        sh.hot_apply(CircleFence.model_validate(circle))
    _all_wgs84(new.policy)
    _reload_same(new.policy)
    c_new, c_old = new.policy.constraints[-1], old.policy.constraints[-1]
    assert (c_new.center.x, c_new.center.y) == (c_old.center.x, c_old.center.y)
    # the circle's 32-gon is derived from the same centre view
    assert [(v.x, v.y) for v in c_new.vertices] == [(v.x, v.y) for v in c_old.vertices]
    assert _rings(new) == _rings(old)


# ------------------------------------------------------------- harness paths

def _events():
    return [ScenarioEvent(at_sim_t=1.0, type="spawn_polygon_fence",
                        payload={"width_m": 12, "height_m": 8, "ahead_of_vehicle_m": 25}),
          ScenarioEvent(at_sim_t=2.0, type="translate_polygon_fence",
                        payload={"dx_m": 3, "dy_m": 1}),
          ScenarioEvent(at_sim_t=3.0, type="rotate_polygon_fence",
                        payload={"angle_deg": 20}),
          ScenarioEvent(at_sim_t=4.0, type="scale_radius", payload={"factor": 1.25})]


def test_harness_events_store_wgs84_same_geometry():
    new, old = _pair()
    es_new, es_old = EventState(shields=[new]), EventState(shields=[old])
    # an off-grid vehicle position, as a flight gives one
    st = State(x=3.141592653, y=-2.718281828, up=20.0)
    heading = (math.cos(0.3), math.sin(0.3))
    for ev in _events():
        r_new = apply_event(ev, es_new, st, heading, ev.at_sim_t)
        r_old = apply_event(ev, es_old, st, heading, ev.at_sim_t)
        _all_wgs84(new.policy)
        _reload_same(new.policy)
        # the record keeps its metres (rounded to the mm) and they agree
        assert r_new["vertices"] == r_old["vertices"], (ev.type, r_new, r_old)
        assert _rings(new) == _rings(old), ev.type


def test_harness_swap_corridor_stores_wgs84():
    new, old = _pair(SURVEY)
    ev = ScenarioEvent(at_sim_t=1.0, type="swap_corridor",
                       payload={"target_id": "corridor-survey-route", "width_m": 25,
                                "centerline": [[0, 0], [55, 0], [55, 60]]})
    st = State(x=0.0, y=0.0, up=20.0)
    for sh in (new, old):
        apply_event(ev, EventState(shields=[sh]), st, (1.0, 0.0), 1.0)
    _all_wgs84(new.policy)
    _reload_same(new.policy)
    assert list(new.policy.constraints[-1].points()) == \
        list(old.policy.constraints[-1].points())


# ------------------------------------------------- enforcement is unchanged

def test_csp_and_decisions_unchanged():
    """The same CSP (but for the hashes that name the new IR, and issued_at:
    the wall clock to the second, which made this test flaky when the two
    compiles straddled a second) and the same Shield decisions, tick for
    tick, past a spawned and edited zone."""
    new, old = _pair()
    for sh in (new, old):
        api.apply_dynamic_nfz(sh, copy.deepcopy(ZONE))
        api.apply_nfz_edit(sh, ZONE["id"], {"op": "translate", "dx": -2, "dy": 1})
    m = Mission(task_text="fly to (60, 10) altitude 20", target_x=60, target_y=10,
                cruise_alt_m=20, speed_pref_mps=4)

    def scrub(node):
        if isinstance(node, dict):
            return {k: scrub(v) for k, v in node.items()
                    if "hash" not in k
                    and k not in ("csp_id", "lat", "lon", "issued_at")}
        if isinstance(node, list):
            return [scrub(v) for v in node]
        return node
    a = scrub(ConstraintCompiler(new.policy).compile_csp(m).model_dump(mode="json"))
    b = scrub(ConstraintCompiler(old.policy).compile_csp(m).model_dump(mode="json"))
    assert a == b
    # fly both at the zone along y = 10, straight north
    s_new = s_old = State(x=0.0, y=10.0, up=20.0)
    acted = 0
    for k in range(120):
        act = Action4D(vx=4.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)
        d_new = new.filter(s_new, act, t=k * 0.1)
        d_old = old.filter(s_old, act, t=k * 0.1)
        assert d_new.emitted == d_old.emitted, (k, d_new.emitted, d_old.emitted)
        assert [v.rule_id for v in d_new.violations] == \
            [v.rule_id for v in d_old.violations], k
        acted += bool(d_new.violations)
        e = d_new.emitted
        s_new = s_old = State(x=s_new.x + e.vx * 0.1, y=s_new.y + e.vy * 0.1, up=20.0)
    assert acted, "the flight never met the zone; the comparison proved nothing"


# ------------------------------------------------------ the legacy and frames

def test_metre_policy_keeps_metre_writes():
    """A policy written only in metres has no WGS84 position for a metre
    point: its events write metres, as before, to the same hash."""
    doc = {"policy_id": "legacy-metres", "version": "0.1.0",
           "constraints": [{"id": "speed", "type": "kinematic_envelope", **RULE,
                            "speed_max_mps": 6,
                            "climb_rate_max_mps": 3, "yaw_rate_max_dps": 90}]}
    new, old = Shield(Policy.model_validate(doc)), Shield(Policy.model_validate(doc))
    old.wgs84_writes = False
    for sh in (new, old):
        api.apply_dynamic_nfz(sh, copy.deepcopy(ZONE))
        api.apply_nfz_edit(sh, ZONE["id"], {"op": "rotate", "angle_deg": 15})
    assert new.policy.policy_hash == old.policy.policy_hash
    assert not any(v.geographic for v in new.policy.constraints[-1].vertices)
    assert rule_in_wgs84(new.policy.constraints[-1], None) is new.policy.constraints[-1]


def test_stored_policy_hashes_unchanged():
    """Nothing on the load path moved: every stored policy hashes to its lock
    entry."""
    lock = json.loads((ROOT / "policies" / "policy.lock.json").read_text(encoding="utf-8"))
    text = json.dumps(lock)
    n = 0
    for f in sorted((ROOT / "policies").glob("*.yaml")):
        pol = load_policy(f, runtime=False)
        assert pol.policy_hash in text, f"{f.name}: {pol.policy_hash} not in the lock"
        n += 1
    assert n >= 32, n


def test_derived_frame_is_never_moved():
    """A WGS84 policy with no stated origin derives its frame from the first
    polygon_fence (else the first WGS84 rule). An event that would move it is
    refused: the live rules would stay in the old frame while the snapshot,
    reloaded, put every point in the new one."""
    o = (35.6895, 139.6917)
    cl = [dict(zip(("lat", "lon"), metres_to_wgs84(o, x, y))) for x, y in ((0, 0), (80, 0))]
    doc = {"policy_id": "derived", "version": "0.1.0",
           "constraints": [{"id": "route", "type": "corridor", **RULE, "centerline": cl,
                            "width_m": 20}]}
    sh = Shield(Policy.model_validate(doc))
    fo = sh.policy.frame_origin
    from guardrail.models import PolygonFence
    fence = PolygonFence.model_validate(
        {"id": "pre", "type": "polygon_fence", **RULE,
         "vertices": [{"x": 30, "y": 15}, {"x": 40, "y": 15}, {"x": 40, "y": 25}]})
    try:
        sh.hot_apply(fence)
    except HotApplyRefused as e:
        assert "derived frame origin" in str(e), e
    else:
        raise AssertionError("a pre-start fence that moves the derived frame was accepted")
    assert sh.policy.frame_origin == fo and sh.policy.generation == 0
    # a zone does not take part in the derivation: converted and accepted
    api.apply_dynamic_nfz(sh, copy.deepcopy(ZONE))
    assert sh.policy.frame_origin == fo
    _all_wgs84(sh.policy)
    _reload_same(sh.policy)


def test_a_point_beyond_the_lat_lon_range_is_refused():
    """A metre point 20 000 km out converts to a latitude of about 144 deg,
    which XY's own lat/lon range check (le=90) refuses: a 422, never stored.
    (This is the range check, not rule_in_wgs84's 1-um consistency guard;
    that guard has its own test below.)"""
    sh = Shield(load_policy(SITL))
    far = {**ZONE, "id": "far", "vertices": [{"x": 2e7, "y": 0}, {"x": 2e7 + 8, "y": 0},
                                             {"x": 2e7 + 8, "y": 8}]}
    try:
        api.apply_dynamic_nfz(sh, far)
    except api.ApiError as e:
        assert e.status == 422, e
    else:
        raise AssertionError("a zone 20 000 km out was stored")
    assert sh.policy.generation == 0



def test_the_consistency_guard_refuses_a_bad_conversion():
    """rule_in_wgs84 refuses lat/lon that do not project back within 1 um of
    the metres given. metres_to_wgs84 cannot produce such a pair for a point
    inside the lat/lon range (the 12-decimal round trip is far inside the
    tolerance), so the guard is exercised with a conversion broken on
    purpose: about 1 mm of latitude too far north."""
    import guardrail.models as M
    good = M.metres_to_wgs84
    M.metres_to_wgs84 = lambda o, x, y: (good(o, x, y)[0] + 1e-8, good(o, x, y)[1])
    try:
        sh = Shield(load_policy(SITL))
        try:
            api.apply_dynamic_nfz(sh, copy.deepcopy(ZONE))
        except api.ApiError as e:
            assert e.status == 422 and "projects back" in str(e), e
        else:
            raise AssertionError("a conversion 1 mm off was stored")
        assert sh.policy.generation == 0
    finally:
        M.metres_to_wgs84 = good


# -------------------------------------- opt-in: the frame binds the live view

def _bound_pair(path=SITL):
    """The default Shield (given metres enforced live) and one with
    wgs84_bind_frame (the live view is the projection of the stored lat/lon)."""
    dflt = Shield(load_policy(path))
    bound = Shield(load_policy(path))
    dflt.wgs84_bind_frame = False
    bound.wgs84_bind_frame = True
    return dflt, bound


def _snapshot(policy: Policy) -> Policy:
    return Policy.model_validate(json.loads(policy.model_dump_json()))


def _views(policy: Policy) -> list:
    return [[(v.x, v.y) for v in pts] for pts in _rule_points(policy, all_points=True)]


def test_the_bind_flag_is_off_by_default():
    """Default-off: it moves computed rings by up to 0.6 um, a flown change,
    so it waits for a declared re-baseline (VLAGUARD_WGS84_BIND=1)."""
    import os
    if os.environ.get("VLAGUARD_WGS84_BIND", "").strip() != "1":
        assert Shield.wgs84_bind_frame is False
        assert Shield(load_policy(SITL)).wgs84_bind_frame is False


def test_bound_live_view_is_the_snapshot_view_bit_for_bit():
    """With wgs84_bind_frame the geometry the live Shield enforces is the
    projection of what is stored: the signed snapshot, reloaded, reads the
    same floats for every point - computed off-grid rings included - and the
    same compiled rings. A FIRST write stores and hashes the same in both
    modes; an edit computes its new ring from the current view, so after one
    the modes part by up to 0.6 um (that is the flown change). In the
    default mode the same events leave at least one point where live and
    snapshot differ (the declared deviation, at most 0.6 um), which shows
    the flag matters."""
    dflt, bound = _bound_pair()
    for sh in (dflt, bound):
        api.apply_dynamic_nfz(sh, copy.deepcopy(ZONE))
    assert bound.policy.policy_hash == dflt.policy.policy_hash
    assert _stored(bound.policy) == _stored(dflt.policy)
    for sh in (dflt, bound):
        api.apply_nfz_edit(sh, ZONE["id"], {"op": "rotate", "angle_deg": 30})
    es_d, es_b = EventState(shields=[dflt]), EventState(shields=[bound])
    st = State(x=3.141592653, y=-2.718281828, up=20.0)
    heading = (math.cos(0.3), math.sin(0.3))
    for ev in _events():
        apply_event(ev, es_d, st, heading, ev.at_sim_t)
        apply_event(ev, es_b, st, heading, ev.at_sim_t)
        _all_wgs84(bound.policy)
        back = _snapshot(bound.policy)
        assert back.policy_hash == bound.policy.policy_hash
        assert _views(back) == _views(bound.policy), ev.type
        # the bound live rings are the rings a Shield over the snapshot compiles
        assert _rings(bound) == _rings(Shield(back)), ev.type
    live_d, snap_d = _views(dflt.policy), _views(_snapshot(dflt.policy))
    assert live_d != snap_d, "the default mode showed no live/snapshot gap here"
    for a, b in zip(live_d, snap_d):
        for (p, q), (u, v) in zip(a, b):
            assert abs(p - u) <= 0.6e-6 and abs(q - v) <= 0.6e-6, ((p, q), (u, v))


def test_bound_mode_binds_a_lat_lon_body_too():
    """A REST body in lat/lon with metres beside them: bound, the metres are
    checked against the frame and dropped, so the live view is the frame's."""
    _, bound = _bound_pair()
    o = bound.policy.frame_origin
    verts = []
    for x, y in ((10.1234567, 10), (18, 10.7654321), (18, 17)):
        lat, lon = metres_to_wgs84(o, x, y)
        verts.append({"lat": lat, "lon": lon, "x": x, "y": y})
    api.apply_dynamic_nfz(bound, {**ZONE, "id": "ll", "vertices": verts})
    z = bound.policy.constraints[-1]
    assert [(v.x, v.y) for v in z.vertices] == \
        [local_metres(o, v.lat, v.lon) for v in z.vertices]
    assert _views(_snapshot(bound.policy)) == _views(bound.policy)


# ---------------------------------------------- the other write paths agree

OFF_GRID = [{"x": 23.1234567, "y": 6}, {"x": 31, "y": 6.7654321}, {"x": 31, "y": 14}]


def test_derive_on_a_framed_base_matches_hot_apply():
    """bundle.derive rebuilds a lock recipe as Shield.hot_apply applies it: on
    a base with a frame, metre points are written as WGS84, so the derived
    hash is the live hash. `wgs84_writes=False` (a recipe for a run flown on
    a framed base before 2026-11-17) gives the metre-write hash."""
    from guardrail.bundle import derive
    rule = {**ZONE, "type": "dynamic_nfz", "vertices": OFF_GRID}
    new, old = _pair()
    api.apply_dynamic_nfz(new, copy.deepcopy(rule))
    api.apply_dynamic_nfz(old, copy.deepcopy(rule))
    got = derive(load_policy(SITL), [copy.deepcopy(rule)], 1, ir_schema_version="4.0.0")
    assert got.policy_hash == new.policy.policy_hash
    _all_wgs84(got)
    was = derive(load_policy(SITL), [copy.deepcopy(rule)], 1, ir_schema_version="4.0.0",
                 wgs84_writes=False)
    assert was.policy_hash == old.policy.policy_hash != got.policy_hash


def _geo_tool():
    sys.path.insert(0, str(ROOT / "tools"))
    import geo_to_policy
    return geo_to_policy


def test_geo_to_policy_ingest_matches_the_live_hash():
    """tools/geo_to_policy.py ingest of a POST /dynamic_nfz metre body onto
    a framed base: stored in WGS84 (no x/y in the canonical IR) and the same
    hash as api.apply_dynamic_nfz gives the live policy - off-grid metres
    included."""
    G = _geo_tool()
    body = {**ZONE, "vertices": OFF_GRID}
    pol = G.ingest(copy.deepcopy(body), load_policy(SITL))
    live = Shield(load_policy(SITL))
    api.apply_dynamic_nfz(live, copy.deepcopy(body))
    assert pol.policy_hash == live.policy.policy_hash
    assert not _has_xy(pol.canonical_ir()["constraints"])
    _all_wgs84(pol)


def test_geo_to_policy_refuses_to_move_a_derived_origin():
    """A base with no stated origin derives its frame from its first
    polygon_fence. Appending one that would become that fence would move
    every WGS84 point: refused, for a metre rule and - a change from master
    47b3fde, which accepted it - for a pure lat/lon GeoJSON zone."""
    G = _geo_tool()
    o = (35.6895, 139.6917)
    cl = [dict(zip(("lat", "lon"), metres_to_wgs84(o, x, y))) for x, y in ((0, 0), (80, 0))]
    base = Policy.model_validate(
        {"policy_id": "derived", "version": "0.1.0",
         "constraints": [{"id": "route", "type": "corridor", **RULE, "centerline": cl,
                          "width_m": 20}]})
    assert base.origin is None and base.frame_origin is not None
    fence = {"id": "pre", "type": "polygon_fence", **RULE,
             "vertices": [{"x": 30, "y": 15}, {"x": 40, "y": 15}, {"x": 40, "y": 25}]}
    ring = [list(reversed(metres_to_wgs84(o, x, y)))
            for x, y in ((30, 15), (40, 15), (40, 25), (30, 15))]
    geo = {"type": "FeatureCollection", "features": [
        {"type": "Feature", "properties": {"name": "yard", **RULE},
         "geometry": {"type": "Polygon", "coordinates": [ring]}}]}
    for src in (fence, geo):
        try:
            G.ingest(copy.deepcopy(src), base)
        except G.ConversionError as e:
            assert "derived frame origin" in str(e), e
        else:
            raise AssertionError(f"moved the derived origin: {src}")


def test_apply_zone_returns_the_rule_as_stored():
    """EpisodeRecord.apply_zone returns the rule the live policy holds: on a
    framed policy that is the WGS84 rule, not the metre fence handed in."""
    import tempfile
    from guardrail.models import XY, PolygonFence
    from guardrail.replay import EpisodeRecord
    policy = load_policy(SITL)
    rec = EpisodeRecord(Path(tempfile.mkdtemp(prefix="wgs84_")) / "ep", policy, None)
    sh = Shield(policy)
    sh.filter(State(x=0.0, y=0.0, up=15.0), Action4D(vx=1.0))     # mission started
    fence = PolygonFence(id="nfz-late", type="polygon_fence", **RULE, margin_m=1.0,
                         vertices=[XY(x=40, y=-5), XY(x=50, y=-5),
                                   XY(x=50, y=5), XY(x=40, y=5)])
    got = rec.apply_zone(sh, fence, t=1.0)
    assert got is sh.policy.constraints[-1]
    assert all(v.geographic for v in got.vertices)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}  {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}  {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
