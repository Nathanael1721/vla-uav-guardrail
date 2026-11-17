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
  * what the live Shield enforces is the lazy projection of what is stored
    (since 2026-11-17 evening the only mode: the rule is bound to the
    policy's frame as it is written): the signed snapshot, reloaded, reads
    the same floats and compiles the same rings, bit for bit, computed
    off-grid rings included;
  * against the metre writes of before (`wgs84_writes = False`): the same
    zone rings, CSP and decisions, float for float, for metres on the
    micrometre grid (every REST body, every hand-written scenario); a
    computed off-grid ring (rotate, a spawn at an off-grid vehicle position)
    moves by at most 0.6 um;
  * a policy written only in metres (the legacy form) keeps metre writes, and
    no stored policy's hash moves;
  * a derived frame (no stated origin) is never moved by an event;
  * no unbound mode is left (no Shield.wgs84_bind_frame, no
    VLAGUARD_WGS84_BIND); a lat/lon body whose x/y disagree with the frame
    is refused;
  * an exact tie between a bound dynamic zone's edges (a zone spawned
    centred on the aircraft) is judged at the geometry's resolution
    (shield.tie_tolerance_m), from what is stored, and every sampled
    dyn-nfz-spawn-on-top cell flies as the metre writes flew it;
  * the evidence in docs/data/wgs84_bind_default_20261117 says what its
    README says, and its diff tool drops only wall-clock keys;
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
    """Two Shields over the same stored policy: the WGS84 writes (default,
    bound to the frame) and the metre writes of before 2026-11-17."""
    new = Shield(load_policy(path))
    old = Shield(load_policy(path))
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


# A computed off-grid metre point, bound to the frame, is enforced at the
# micrometre view of its stored lat/lon: within 0.6 um of the metres the
# edit computed. Compiled rings (buffered by the margin) carry it through.
OFF_GRID_TOL_M = 0.6e-6
RING_TOL_M = 1e-6
# Over a chain of edits each computed from the previous bound view.
CHAIN_TOL_M = 3e-6


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
        # the live rings: the same floats while every point is on the
        # micrometre grid; the computed (rotated) ring within 1 um
        given = [c for v in old.policy.constraints[-1].given_vertices() for c in (v.x, v.y)]
        if all(round(c, 6) == c for c in given):
            assert _rings(new) == _rings(old), body
        else:
            assert _close(_rings(new), _rings(old), RING_TOL_M), body
    a = [(v.x, v.y) for v in new.policy.constraints[-1].given_vertices()]
    b = [(v.x, v.y) for v in old.policy.constraints[-1].given_vertices()]
    assert _close(a, b, OFF_GRID_TOL_M), (a, b)
    assert any(round(c, 6) != c for xy in b for c in xy), "rotate wrote no off-grid metre"
    # bound: the live view is the micrometre view of the stored lat/lon
    assert all(round(c, 6) == c for xy in a for c in xy), a


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
        # the record keeps its metres (rounded to the mm) and they agree;
        # the computed off-grid rings within 1 um
        assert r_new["vertices"] == r_old["vertices"], (ev.type, r_new, r_old)
        assert _close(_rings(new), _rings(old), RING_TOL_M), ev.type


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


# ------------------------- the live view is the projection of what is stored

def _snapshot(policy: Policy) -> Policy:
    return Policy.model_validate(json.loads(policy.model_dump_json()))


def _views(policy: Policy) -> list:
    return [[(v.x, v.y) for v in pts] for pts in _rule_points(policy, all_points=True)]


def test_no_unbound_mode_is_left():
    """Until the evening of 2026-11-17 (integrate/1117) the Shield by default
    enforced the metres an event gave, not the projection of the lat/lon it
    stored (opt-in Shield.wgs84_bind_frame / VLAGUARD_WGS84_BIND=1, read once
    when guardrail.shield was imported, closed the gap). Binding is now the
    only mode: the flag is gone (nothing reads the variable any more, so
    setting it here could prove nothing), rule_in_wgs84 takes no `bind`, and
    an off-grid spawn's live view is its snapshot's."""
    import inspect
    assert not hasattr(Shield, "wgs84_bind_frame")
    assert "bind" not in inspect.signature(rule_in_wgs84).parameters
    sh = Shield(load_policy(SITL))
    api.apply_dynamic_nfz(sh, {**ZONE, "vertices": OFF_GRID})
    assert _views(_snapshot(sh.policy)) == _views(sh.policy)


def test_live_view_is_the_snapshot_view_bit_for_bit():
    """The geometry the live Shield enforces is the projection of what is
    stored: the signed snapshot, reloaded, reads the same floats for every
    point - computed off-grid rings included - and compiles the same rings,
    after every write path (REST spawn and edits, the harness's events).
    Against the metre writes of before, at least one computed point is not
    the same float (that is the flown change this binding made, WP4-11's
    price for bit-for-bit replay) and every point is within a few
    micrometres: each write is within 0.6 um of what it computed, but an
    edit computes its ring from the current (bound) view, so over a chain
    of six edits (a rotate, a scale by 1.25, ...) the offsets compound - the
    worst here is about 0.87 um."""
    sh, old = _pair()
    for s in (sh, old):
        api.apply_dynamic_nfz(s, copy.deepcopy(ZONE))
        api.apply_nfz_edit(s, ZONE["id"], {"op": "rotate", "angle_deg": 30})
    back = _snapshot(sh.policy)
    assert back.policy_hash == sh.policy.policy_hash
    assert _views(back) == _views(sh.policy)
    assert _rings(sh) == _rings(Shield(back))
    es, es_old = EventState(shields=[sh]), EventState(shields=[old])
    st = State(x=3.141592653, y=-2.718281828, up=20.0)
    heading = (math.cos(0.3), math.sin(0.3))
    for ev in _events():
        apply_event(ev, es, st, heading, ev.at_sim_t)
        apply_event(ev, es_old, st, heading, ev.at_sim_t)
        _all_wgs84(sh.policy)
        back = _snapshot(sh.policy)
        assert back.policy_hash == sh.policy.policy_hash
        assert _views(back) == _views(sh.policy), ev.type
        assert _rings(sh) == _rings(Shield(back)), ev.type
    live, metres = _views(sh.policy), _views(old.policy)
    assert live != metres, "no computed point moved; the comparison proved nothing"
    for a, b in zip(live, metres):
        for (p, q), (u, v) in zip(a, b):
            assert abs(p - u) <= CHAIN_TOL_M and abs(q - v) <= CHAIN_TOL_M, \
                ((p, q), (u, v))


def test_a_zone_centred_on_the_aircraft_escapes_toward_the_heading_once_bound():
    """A square zone written centred on an off-grid aircraft position is an
    exact tie between its edges as written (the escape then follows the
    heading: shield.escape_direction). Bound to the frame, its vertices move
    by up to 0.6 um and the edge distances by up to about 1.2 um - above
    shield._TIE_M (1e-9 m), at which the tie broke by geometry (library cells
    dyn-nfz-spawn-on-top@size_m=20 x3 and @size_m=40 variants 1, 2;
    docs/data/wgs84_bind_default_20261117/). A dynamic_nfz stored in WGS84
    is therefore judged at the geometry's resolution
    (shield.tie_tolerance_m: _TIE_BOUND_M, 2e-6 m). Pinned: with it the
    bound zone follows the heading every time, as the metre writes of before
    do, and the escape is straight out through one edge (never a diagonal).
    How many of these ties 1e-9 m would break away from the heading is
    printed, not asserted: it is the reason for the tolerance, not a spec."""
    from shapely.geometry import Polygon as _Poly
    from guardrail import shield as SH
    sh = Shield(load_policy(SITL))
    rng = random.Random(20261117)
    off_heading = at_1e9 = 0
    for _ in range(40):
        cx, cy = rng.uniform(-200, 200), rng.uniform(-200, 200)
        h = rng.uniform(5, 25)
        ang = rng.uniform(0, 2 * math.pi)
        c, s = math.cos(ang), math.sin(ang)
        corners = [(cx + c * u - s * v, cy + s * u + c * v)
                   for u, v in ((-h, -h), (h, -h), (h, h), (-h, h))]
        z = DynamicNFZ.model_validate({**ZONE, "id": "on-top", "type": "dynamic_nfz",
                                       "vertices": [{"x": x, "y": y} for x, y in corners]})
        bz = rule_in_wgs84(z, sh.policy.frame_origin)
        assert SH.tie_tolerance_m(bz) == SH._TIE_BOUND_M == 2e-6
        assert SH.tie_tolerance_m(z) == SH._TIE_M          # metres: no frame, no rounding
        bound = _Poly([(v.x, v.y) for v in bz.vertices])
        metres = _Poly(corners)
        normals = [(math.cos(ang + k * math.pi / 2), math.sin(ang + k * math.pi / 2))
                   for k in range(4)]
        for hx, hy in normals:             # heading straight at each edge
            ux, uy = SH.escape_direction(cx, cy, metres, hx, hy)
            assert ux * hx + uy * hy > 0.999999, ("metres", (ux, uy), (hx, hy))
            ux, uy = SH.escape_direction(cx, cy, bound, hx, hy, SH.tie_tolerance_m(bz))
            assert max(ux * nx + uy * ny for nx, ny in normals) > 0.999999, (ux, uy)
            off_heading += ux * hx + uy * hy < 0.999
            ux, uy = SH.escape_direction(cx, cy, bound, hx, hy)        # 1e-9 m
            at_1e9 += ux * hx + uy * hy < 0.999
    assert off_heading == 0, f"{off_heading} of 160 bound ties broke away from the heading"
    print(f"      (at 1e-9 m, {at_1e9} of 160 bound ties would break away from the heading)")


def test_the_tie_tolerance_is_a_property_of_what_is_stored():
    """_TIE_BOUND_M applies to a dynamic_nfz stored in WGS84 and to nothing
    else, and a snapshot reloaded (as tools/replay_q3.py installs a
    generation, without Shield._in_frame) gives the same answer: every fence
    - the static zones of gps-noise-within-margin and wind-beyond-authority,
    which a global 2e-6 m changed - keeps 1e-9 m, as does a dynamic_nfz in a
    policy written only in metres."""
    from guardrail import shield as SH
    sh = Shield(load_policy(SITL))
    api.apply_dynamic_nfz(sh, {**ZONE, "vertices": OFF_GRID})
    live = {c.id: SH.tie_tolerance_m(c) for c in sh.policy.constraints}
    assert live[ZONE["id"]] == SH._TIE_BOUND_M
    assert all(v == SH._TIE_M for k, v in live.items() if k != ZONE["id"]), live
    assert any(c.type == "polygon_fence" for c in sh.policy.constraints)
    back = _snapshot(sh.policy)
    assert {c.id: SH.tie_tolerance_m(c) for c in back.constraints} == live
    legacy = Shield(Policy.model_validate(
        {"policy_id": "metres", "version": "0.1.0",
         "constraints": [{"id": "f", "type": "polygon_fence", **RULE,
                          "vertices": [{"x": 0, "y": 0}, {"x": 9, "y": 0}, {"x": 9, "y": 9}],
                          "altitude_floor_m": 0, "altitude_ceiling_m": 100}]}))
    assert legacy.policy.frame_origin is None
    api.apply_dynamic_nfz(legacy, {**ZONE, "vertices": OFF_GRID})
    assert all(SH.tie_tolerance_m(c) == SH._TIE_M for c in legacy.policy.constraints)


def _harness():
    sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S
    from guardrail.scenario_spec import Library
    return S, Library(ROOT / "experiments" / "scenarios.yaml")


def _end(ep) -> tuple[float, float]:
    r = ep.rows[-1]
    return float(r["x"]), float(r["y"])


def test_centred_spawns_fly_as_the_metre_writes_did():
    """Every sampled cell of dyn-nfz-spawn-on-top (size 20 and 40 m, the
    three approach variants the nightly profile flies): bound to the frame
    the zone is flown as the metre writes of before flew it (no FSM edge,
    the same end position to 1 mm). At _TIE_M (1e-9 m) for every zone five
    of these six cells changed: size 20 x3 went G1 Brake 5.0 s -> G7 11.9 s
    and missed the goal, size 40 variants 1 and 2 went G1 Brake -> G3
    Loiter and ended holding at the zone edge (review of wf17b-bind,
    2026-11-17)."""
    from guardrail import shield as SH
    S, L = _harness()
    cells = L.by_id()["dyn-nfz-spawn-on-top"].cells(samples=3)
    assert len(cells) == 6, [c.params for c in cells]
    for c in cells:
        bound = S.run_episode(c.spec, L.defaults, 0)
        SH.Shield.wgs84_writes = False
        try:
            metres = S.run_episode(c.spec, L.defaults, 0)
        finally:
            SH.Shield.wgs84_writes = True
        for ep in (bound, metres):
            tr = (ep.extra.get("fsm") or {}).get("transitions") or []
            assert tr == [], (c.params, tr)
        (bx, by), (mx, my) = _end(bound), _end(metres)
        assert math.hypot(bx - mx, by - my) < 1e-3, (c.params, (bx, by), (mx, my))


def test_a_lat_lon_body_with_inconsistent_metres_is_refused():
    """Since binding is the only mode, x/y given beside lat/lon are checked
    against the frame: more than 1 um off and the body is refused (422 on
    the REST endpoint). Before (integrate/1117 default) such a rule came back
    unchanged and the live Shield enforced the given x/y under the lat/lon's
    hash."""
    sh = Shield(load_policy(SITL))
    o = sh.policy.frame_origin
    verts = []
    for x, y in ((10.0, 10.0), (18.0, 10.0), (18.0, 17.0)):
        lat, lon = metres_to_wgs84(o, x, y)
        verts.append({"lat": lat, "lon": lon, "x": x + 5e-6, "y": y})
    try:
        api.apply_dynamic_nfz(sh, {**ZONE, "id": "ll-off", "vertices": verts})
    except (api.ApiError, HotApplyRefused) as e:
        assert getattr(e, "status", 422) == 422, e
    else:
        raise AssertionError("x/y 5 um off their lat/lon were accepted")
    assert sh.policy.generation == 0


def test_a_lat_lon_body_is_bound_too():
    """A REST body in lat/lon with metres beside them: the metres are checked
    against the frame and dropped, so the live view is the frame's."""
    sh = Shield(load_policy(SITL))
    o = sh.policy.frame_origin
    verts = []
    for x, y in ((10.1234567, 10), (18, 10.7654321), (18, 17)):
        lat, lon = metres_to_wgs84(o, x, y)
        verts.append({"lat": lat, "lon": lon, "x": x, "y": y})
    api.apply_dynamic_nfz(sh, {**ZONE, "id": "ll", "vertices": verts})
    z = sh.policy.constraints[-1]
    assert [(v.x, v.y) for v in z.vertices] == \
        [local_metres(o, v.lat, v.lon) for v in z.vertices]
    assert _views(_snapshot(sh.policy)) == _views(sh.policy)


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


# ------------------------------- tools/wgs84_bind_compare.py (the evidence)

def _compare_tool():
    sys.path.insert(0, str(ROOT / "tools"))
    import wgs84_bind_compare
    return wgs84_bind_compare


def test_the_compare_tool_classifies_and_aligns():
    """The before/after tool of docs/data/wgs84_bind_default_20261117: a
    float within 1e-6 is um_level, an int change a count, wall-clock fields
    are ignored; audit logs align by monitor tick, a negated action reads
    `mirrored`, a tick logged on one side only is named with its violation."""
    import tempfile
    W = _compare_tool()
    rows = {f["field"]: f for f in W.diff_dicts(
        {"a": 1.0, "n": 3, "runtime_s": 1, "same": 2},
        {"a": 1.0000004, "n": 4, "runtime_s": 9, "same": 2})}
    assert set(rows) == {"a", "n"}, rows
    assert rows["a"]["kind"] == "um_level" and rows["n"]["kind"] == "count"
    # Only wall-clock / path / revision keys are dropped, matched on the whole
    # key: until 2026-11-17 evening an endswith() on 'ts', 'dir', 'path' and
    # 'wall' also dropped these flown-behaviour fields (review of wf17b-bind).
    flown = {"kpi": {"hold_points": [1], "harness_events": 1, "collision_contacts": 0,
                     "collision_objects": [], "p0_escapes": 0},
             "_failsafe_labels": {"fail_safe_correctness_distinct_flights": 1.0,
                                  "correct_distinct_flights": 3,
                                  "labelled_distinct_flights": 3, "cell_seed_slots": 3},
             "_rollup": {"wind-gusts": 1},
             "wall": 1, "ts": 1, "bundle": "x", "dir": "d", "path": "p", "runtime_s": 1,
             "flight_s_wall": 1, "code_revision": "a", "issued_at": "t"}
    moved = json.loads(json.dumps(flown).replace(": 1", ": 2").replace(": 3", ": 4")
                       .replace("[1]", "[2]").replace(": 0", ": 5").replace("[]", "[0]")
                       .replace('"x"', '"y"').replace('"d"', '"e"').replace('"p"', '"q"')
                       .replace('"a"', '"b"').replace('"t"', '"u"'))
    got = {f["field"] for f in W.diff_dicts(flown, moved)}
    assert got == {"kpi.hold_points", "kpi.harness_events", "kpi.collision_contacts",
                   "kpi.collision_objects", "kpi.p0_escapes",
                   "_failsafe_labels.fail_safe_correctness_distinct_flights",
                   "_failsafe_labels.correct_distinct_flights",
                   "_failsafe_labels.labelled_distinct_flights",
                   "_failsafe_labels.cell_seed_slots", "_rollup.wind-gusts"}, got
    # The headline carries the grant's KPI 1, mission success, Shield-on and
    # per family: a fallen success rate is not "headline identical".
    sw = {"results": [], "_rollup": {"_all_shield_on": {"mission_success_rate": 0.5},
                                     "fam": {"mission_success_rate": 0.4}},
          "_outcome_labels": {"outcomes": {"success": 1}}}
    sw2 = json.loads(json.dumps(sw).replace("0.4", "0.3"))
    h1, h2 = W.headline(sw), W.headline(sw2)
    assert h1["mission_shield_on"]["mission_success_rate"] == 0.5
    assert h1 != h2 and h1["mission_by_family"]["fam"]["mission_success_rate"] == 0.4
    tmp = Path(tempfile.mkdtemp(prefix="bindcmp_"))

    def log(d, recs):
        d.mkdir()
        (d / "shield_audit.jsonl").write_text(
            "".join(json.dumps(r) + "\n" for r in recs), encoding="utf-8")

    def rec(t, vx, vy, viol=()):
        return {"monitor_tick": t, "emitted_action_world": {"vx": vx, "vy": vy},
                "violations": [{"rule_id": v, "category": "geometric",
                                "detail": "d", "predicted_at_s": 5.0} for v in viol]}
    log(tmp / "b", [rec(1, 1.0, 0.0), rec(2, 3.0, -1.0), rec(3, 0.0, 0.0, ["z"])])
    log(tmp / "a", [rec(1, 1.0 + 4e-8, 0.0), rec(2, -3.0, 1.0)])
    got = W.audit_diff(tmp / "b", tmp / "a")
    assert got["first_differing_tick"] == 1, got
    assert got["ticks_emitted_differs_um_level"] == 1
    assert got["ticks_emitted_differs_large"] == 1 and got["large"][0]["mirrored"]
    one = got["ticks_audited_in_one_only"]
    assert one == [{"tick": 3, "audited_in": "before", "fsm": [None, None],
                    "violations": [{"rule_id": "z", "detail": "d", "predicted_at_s": 5.0}]}], one


def test_the_bind_evidence_matches_its_readme():
    """The committed diffs say what the README says (the numbers are read
    from the JSON, the README quotes them): the headline, mission success
    included, is the same before and after in the smoke profile, the library
    and the nightly profile's sampled cells on one seed; no episode's FSM
    edge sequence, mission outcome, status or hold points changed; the
    episodes that differ are counted."""
    d = ROOT / "docs" / "data" / "wgs84_bind_default_20261117"
    want = {"smoke_diff.json": (64, 8), "library_diff.json": (180, 14),
            "nightly_seed1001_diff.json": (248, 29)}
    times = {"smoke_diff.json": 0, "library_diff.json": 0, "nightly_seed1001_diff.json": 5}
    success = {"smoke_diff.json": (0.460317, 29, 63), "library_diff.json": (0.536313, 96, 179),
               "nightly_seed1001_diff.json": (0.469636, 116, 247)}
    for name, (rate, n, of) in success.items():
        got = json.loads((d / name).read_text(encoding="utf-8"))
        for side in ("headline_before", "headline_after"):
            m = got[side]["mission_shield_on"]
            assert (m["mission_success_rate"], m["mission_successes"],
                    m["episodes_mission_scored"]) == (rate, n, of), (name, side, m)
    for name, counts in want.items():
        got = json.loads((d / name).read_text(encoding="utf-8"))
        assert got["headline_identical"], (name, got["headline_fields_differing"])
        assert got["headline_fields_differing"] == [], name
        hb = got["headline_before"]
        assert hb["mission_shield_on"]["mission_success_rate"] is not None, name
        assert got["episodes_fsm_outcome_status_or_hold_changed"] == [], name
        for e in got["episodes"]:
            f = e["flown"]
            assert [x[:3] for x in f["fsm_edges_before"]] == \
                [x[:3] for x in f["fsm_edges_after"]], (name, e["id"])
        assert (got["episodes_compared"], got["episodes_differing"]) == counts, \
            (name, got["episodes_compared"], got["episodes_differing"])
        # the same edges, some 0.1-0.2 s apart (tangent-phase swaps, README)
        assert len(got["episodes_fsm_edge_times_moved"]) == times[name], name
    smoke = json.loads((d / "smoke_diff.json").read_text(encoding="utf-8"))
    assert smoke["headline_after"]["fail_safe_correctness"] == 1.0
    cost = json.loads((d / "tick_cost.json").read_text(encoding="utf-8"))
    assert all(v["view_reads"] == 0 for v in cost["projection_reads_during_ticks"].values())


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
