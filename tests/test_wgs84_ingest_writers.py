"""
WP1-14: every writer of policy / zone geometry stores WGS84 at ingest (2026-11-18).

    python tests/test_wgs84_ingest_writers.py      (no pytest needed)

The grant (Policy DSL page): "WGS84 latitude / longitude is canonical ... the
DSL itself never carries projected coordinates". The row's gap said a metre
document was still what "the REST API, the Shield's own zone edits and the
harness's spawned zones write". Since 2026-11-17 the Shield writes every rule
an event adds in WGS84 about the policy's frame origin (Shield._in_frame,
models.rule_in_wgs84; tests/test_wgs84_writes.py proves it on two policies).
This file checks the WRITERS as an inventory, over every shipped policy, and
closes the one writer that still produced the metre form:

  * REST: POST /dynamic_nfz, POST /hot_apply (dynamic_nfz), the deprecated
    POST /nfz, PATCH move / translate - metre bodies at off-grid values, on
    every shipped policy the Shield can fly - store lat/lon only, and the
    stored point's local view is within 1 mm of the metres sent (it is
    within a micrometre); POST /corridor_swap on every shipped corridor (one
    policy has one); PATCH rotate / scale are in tests/test_wgs84_writes.py
    (two policies);
  * the harness: spawn_polygon_fence (a zone computed ahead of an off-grid
    vehicle position, and given vertices) on every shipped policy; and the
    gRPC harness's forward path (a REST body validated on a dry-run Shield,
    turned into the runner's ScenarioEvent, applied to another Shield) on
    three policies (sim_demo, wgs84_taipei, gui_policy) stores the same zone
    within 1 mm;
  * the Mission Control GUI (demo/gui_mission_control.py, via
    demo/gui_policy.py): the zones drawn in metres are written as WGS84
    about the base policy's frame origin. Redrawing the zones of the shipped
    policies/gui_policy.yaml reproduces its policy_hash exactly (the
    gui-mission@0.1.3 lock entry); the metre writer of before produced a
    document with no origin and another hash. On seeded synthetic drawings
    the Shield enforces the same rings and takes the same decisions from
    either document. A base with lat/lon points and no stated origin gives
    its derived frame origin, not PAS_HOME;
  * the legacy metre READ form is labelled: Policy.point_form, and
    bundle.load_for_flight records `point_form` / `point_note` for it; a
    WGS84 policy's flight record is unchanged (no key added); since
    2026-11-18 nothing writes the legacy form (an event on a policy with no
    frame origin, a new derivation on a metre base and gen_random_scenario
    --metres are refused; tests/test_wgs84_frameless_writers.py), and the
    lock's derived recipes on a metre base are only read back;
  * no stored policy hash moves: every policies/*.yaml hashes to its lock
    entry and check_lock() is clean.
"""
from __future__ import annotations

import copy
import json
import math
import random
import sys
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import api                                          # noqa: E402
from wgs84_fixture import rest                                  # noqa: E402
from guardrail.bundle import (LEGACY_POINT_NOTE, check_lock,       # noqa: E402
                              LegacyPointFormRefused,
                              load_for_flight, load_lock, policy_candidates)
from guardrail.harness_service import scenario_event_from_rule     # noqa: E402
from guardrail.models import (LEGACY_POINT_FORMS,                  # noqa: E402
                              POINTS_LEGACY_METRES, POINTS_LEGACY_MIXED, POINTS_NONE, POINTS_WGS84, Action4D, Policy,
                              State, _rule_points, load_policy)
from guardrail.projection import LocalProjection                   # noqa: E402
from guardrail.scenario_events import EventState, apply_event      # noqa: E402
from guardrail.scenario_spec import ScenarioEvent                  # noqa: E402
from guardrail.shield import Shield                                # noqa: E402

import gui_policy                                                  # noqa: E402

POLICIES = sorted((ROOT / "policies").glob("*.yaml"))
RULE = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
        "violation_action": "project_fix"}
TOL_M = 1e-3            # the row's tolerance: 1 mm
MICRO_M = 1e-6          # what the conversion gives: within a micrometre


def _has_xy(node) -> bool:
    if isinstance(node, dict):
        return ("x" in node or "y" in node) or any(_has_xy(v) for v in node.values())
    if isinstance(node, list):
        return any(_has_xy(v) for v in node)
    return False


def _assert_wgs84(policy: Policy, where: str) -> None:
    metre = [v for pts in _rule_points(policy, all_points=True) for v in pts
             if not v.geographic]
    assert not metre, f"{where}: metre points stored: {metre[:3]}"
    assert not _has_xy(policy.canonical_ir()["constraints"]), f"{where}: x/y in the IR"
    assert policy.point_form in (POINTS_WGS84, POINTS_NONE), (where, policy.point_form)


def _true_view(policy: Policy, rule) -> list[tuple[float, float]]:
    """The local metres of `rule`'s stored lat/lon about the policy's frame
    origin, UNROUNDED (the projection itself, not the micrometre view)."""
    proj = LocalProjection(*policy.frame_origin)
    pts = getattr(rule, "vertices", None) or rule.centerline
    return [proj.to_local(v.lat, v.lon) for v in pts]


def _err(policy: Policy, rule, sent) -> float:
    """The largest per-axis distance between each point sent and the stored
    point it became. A ring is stored in its canonical order (IR schema
    3.0.0: counter-clockwise, from a fixed vertex), so points are matched
    by nearest neighbour, and the match must be one to one."""
    got = _true_view(policy, rule)
    assert len(got) == len(sent), (rule.id, got, sent)
    worst, used = 0.0, set()
    for x, y in sent:
        d, k = min((max(abs(a - x), abs(b - y)), k) for k, (a, b) in enumerate(got))
        assert k not in used, (rule.id, got, sent)
        used.add(k)
        worst = max(worst, d)
    return worst


def _rule(policy: Policy, rid: str):
    return next(c for c in policy.constraints if c.id == rid)


def _flyable():
    """(path, Shield) for every shipped policy the Shield flies as loaded
    (distance_envelope_sitl.yaml needs a ground source for its MSL rule)."""
    out = []
    for p in POLICIES:
        try:
            out.append((p, Shield(load_policy(p))))
        except ValueError:
            continue
    return out


def _tri(rng, cx, cy, r=8.0):
    """A triangle of off-grid metre points (many decimals) about (cx, cy)."""
    return [(cx + r * math.cos(a) + rng.uniform(-1, 1) * 1e-3,
             cy + r * math.sin(a) + rng.uniform(-1, 1) * 1e-3)
            for a in (0.1, 2.2, 4.3)]


# ------------------------------------------------------------------ REST

def test_rest_writers_store_wgs84_on_every_shipped_policy():
    rng = random.Random(1118)
    flyable = _flyable()
    assert len(flyable) == len(POLICIES) - 1, [p.name for p, _ in flyable]
    worst = 0.0
    n = 0
    for p, sh in flyable:
        h0 = sh.policy.policy_hash
        cx, cy = rng.uniform(300, 400), rng.uniform(300, 400)
        # POST /dynamic_nfz
        t1 = _tri(rng, cx, cy)
        rest(api.apply_dynamic_nfz, sh, {"id": "rest-dyn", **RULE, "margin_m": 1.0,
                                   "vertices": [{"x": x, "y": y} for x, y in t1]})
        worst = max(worst, _err(sh.policy, _rule(sh.policy, "rest-dyn"), t1))
        # POST /hot_apply (dynamic_nfz); an [x, y] pair is refused since
        # 2026-11-26 (WP1-14), so the client sends {lat, lon} points
        t2 = _tri(rng, cx + 40, cy)
        try:
            api.apply_event(sh, {"type": "dynamic_nfz", "id": "rest-hot", **RULE,
                                 "margin_m": 1.0, "vertices": [[x, y] for x, y in t2]})
        except api.ApiError as e:
            assert e.status == 422 and "WP1-14" in e.detail, e.detail
        else:
            raise AssertionError(f"{p.name}: an [x, y] pair was accepted")
        rest(api.apply_event, sh, {"type": "dynamic_nfz", "id": "rest-hot", **RULE,
                             "margin_m": 1.0, "vertices": [{"x": x, "y": y} for x, y in t2]})
        worst = max(worst, _err(sh.policy, _rule(sh.policy, "rest-hot"), t2))
        # the deprecated POST /nfz (the 2026-07 body)
        t3 = _tri(rng, cx, cy + 40)
        rest(api.apply_legacy_nfz, sh, {"id": "rest-legacy",
                                  "vertices": [{"x": x, "y": y} for x, y in t3]})
        worst = max(worst, _err(sh.policy, _rule(sh.policy, "rest-legacy"), t3))
        # PATCH move, then translate by an off-grid offset
        t4 = _tri(rng, cx - 40, cy)
        rest(api.apply_nfz_edit, sh, "rest-dyn", {"op": "move",
                                            "vertices": [{"x": x, "y": y} for x, y in t4]})
        worst = max(worst, _err(sh.policy, _rule(sh.policy, "rest-dyn"), t4))
        before = [(v.x, v.y) for v in _rule(sh.policy, "rest-dyn").vertices]
        dx, dy = rng.uniform(-3, 3), rng.uniform(-3, 3)
        rest(api.apply_nfz_edit, sh, "rest-dyn", {"op": "translate", "dx": dx, "dy": dy})
        worst = max(worst, _err(sh.policy, _rule(sh.policy, "rest-dyn"),
                                [(x + dx, y + dy) for x, y in before]))
        _assert_wgs84(sh.policy, p.name)
        assert sh.policy.policy_hash != h0 and sh.policy.generation == 5, p.name
        # the generation's snapshot reloads to the same hash, still WGS84
        back = Policy.model_validate(json.loads(sh.policy.model_dump_json()))
        assert back.policy_hash == sh.policy.policy_hash, p.name
        n += 1
    assert n == 32, n
    assert worst <= TOL_M, worst
    assert worst <= MICRO_M, worst
    print(f"    REST: {n} shipped policies x 5 writes, worst round trip {worst:.3e} m")


def test_rest_corridor_swap_stores_wgs84():
    rng = random.Random(7)
    worst = 0.0
    n = 0
    for p, sh in _flyable():
        cors = [c for c in sh.policy.constraints if c.type == "corridor"]
        for c in cors:
            line = [(x + rng.uniform(-0.5, 0.5), y + rng.uniform(-0.5, 0.5))
                    for x, y in ((v.x, v.y) for v in c.centerline)]
            rest(api.apply_corridor_swap, sh, {"id": f"swap-{c.id}", "target_id": c.id,
                                         **RULE, "width_m": float(c.width_m),
                                         "altitude_floor_m": float(c.altitude_floor_m),
                                         "altitude_ceiling_m": float(c.altitude_ceiling_m),
                                         "centerline": [{"x": x, "y": y} for x, y in line]})
            worst = max(worst, _err(sh.policy, _rule(sh.policy, f"swap-{c.id}"), line))
            n += 1
        _assert_wgs84(sh.policy, p.name)
    assert n >= 1, "no shipped corridor to swap"
    assert worst <= MICRO_M, worst
    print(f"    REST corridor_swap: {n} corridor(s), worst {worst:.3e} m")


# ------------------------------------------------------------------ harness

def test_harness_spawns_store_wgs84_on_every_shipped_policy():
    from guardrail.scenario_events import spawn_zone
    rng = random.Random(3)
    worst = 0.0
    n = 0
    for p, sh in _flyable():
        es = EventState(shields=[sh])
        # an off-grid vehicle position and heading, as a flight gives them
        st = State(x=300 + rng.random(), y=300 + rng.random(), up=20.0)
        a = rng.uniform(0, 2 * math.pi)
        heading = (math.cos(a), math.sin(a))
        ahead = {"width_m": 12.3, "height_m": 7.7, "ahead_of_vehicle_m": 25}
        # what spawn_zone computes in metres, before the Shield's ingest
        want = [(v.x, v.y) for v in spawn_zone(ahead, 1, st, heading).vertices]
        apply_event(ScenarioEvent(at_sim_t=1.0, type="spawn_polygon_fence",
                                  payload=dict(ahead)), es, st, heading, 1.0)
        worst = max(worst, _err(sh.policy, _rule(sh.policy, es.last_zone), want))
        given = _tri(rng, st.x - 60, st.y)
        apply_event(ScenarioEvent(at_sim_t=2.0, type="spawn_polygon_fence",
                                  payload={"vertices": [{"x": x, "y": y} for x, y in given]}),
                    es, st, heading, 2.0)
        worst = max(worst, _err(sh.policy, _rule(sh.policy, es.last_zone), given))
        _assert_wgs84(sh.policy, p.name)
        n += 1
    assert n == 32, n
    assert worst <= MICRO_M, worst
    print(f"    harness spawns: {n} shipped policies x 2, worst {worst:.3e} m")


def test_grpc_harness_forward_path_keeps_the_zone():
    """HotApply on the gRPC harness: the body is validated on a dry-run
    Shield through the REST handler (stored WGS84), turned into the
    runner's ScenarioEvent from the stored rule's local view (metres), and
    applied by the runner to the flying Shield, which converts it at
    ingest again. The zone flown is the zone sent, within a micrometre."""
    rng = random.Random(5)
    worst = 0.0
    for name in ("sim_demo_policy.yaml", "wgs84_taipei.yaml", "gui_policy.yaml"):
        p = ROOT / "policies" / name
        dry, flying = Shield(load_policy(p)), Shield(load_policy(p))
        sent = _tri(rng, 250.0, -120.0)
        out = rest(api.apply_event, dry, {"type": "dynamic_nfz", "id": "grpc-zone", **RULE,
                                    "margin_m": 1.0,
                                    "vertices": [{"x": x, "y": y} for x, y in sent]})
        rule = _rule(dry.policy, out["applied"])
        ev = ScenarioEvent.model_validate(scenario_event_from_rule(rule, 1.0))
        es = EventState(shields=[flying])
        apply_event(ev, es, State(x=0.0, y=0.0, up=20.0), (1.0, 0.0), 1.0)
        flown = _rule(flying.policy, "grpc-zone")
        _assert_wgs84(dry.policy, name)
        _assert_wgs84(flying.policy, name)
        worst = max(worst, _err(flying.policy, flown, sent))
        # the micrometre views agree exactly
        assert [(v.x, v.y) for v in flown.vertices] == [(v.x, v.y) for v in rule.vertices]
    assert worst <= MICRO_M, worst


# ------------------------------------------------------------------ the GUI

def _shipped_gui_rects():
    st = load_policy(ROOT / "policies" / "gui_policy.yaml", runtime=False)
    rects = []
    for c in st.constraints:
        if c.type == "polygon_fence":
            xs, ys = [v.x for v in c.vertices], [v.y for v in c.vertices]
            rects.append((min(xs), max(xs), min(ys), max(ys)))
    return st, rects


def test_gui_writer_reproduces_the_shipped_gui_policy():
    """Redrawing the three zones of policies/gui_policy.yaml (High band)
    writes that very policy: same hash as the file and its lock entry. The
    metre writer of before wrote no origin and {x, y} points: the legacy
    form, under another hash, so one click on Fly broke the lock."""
    st, rects = _shipped_gui_rects()
    assert len(rects) == 3
    doc, notes = gui_policy.gui_policy_doc(rects, [], True)
    assert notes == []
    # what the GUI puts on disk, read back as the flight reads it
    with tempfile.TemporaryDirectory() as d:
        f = Path(d) / "gui_policy.yaml"
        f.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
        back = load_policy(f)
    assert back.policy_hash == st.policy_hash
    assert back.point_form == POINTS_WGS84 and back.frame_origin == st.frame_origin
    lock = load_lock()["policies"]
    assert lock[f"{st.policy_id}@{st.version}"]["policy_hash"] == back.policy_hash
    # the writer of before
    old = Policy.model_validate({"policy_id": "gui-mission", "version": st.version,
                                 "constraints": gui_policy.metre_constraints(
                                     rects, [], gui_policy.read_base()[0], True)})
    assert old.point_form == POINTS_LEGACY_METRES and old.origin is None
    assert old.policy_hash != st.policy_hash


def _rings(sh):
    return [(r.rule.id, list(r.polygon.exterior.coords), list(r.buffered.exterior.coords))
            for r in sh.ir.fences]


def test_gui_writer_round_trip_and_same_enforcement():
    rng = random.Random(42)
    rects = []
    for _ in range(4):
        x0, y0 = rng.uniform(-70, 50), rng.uniform(-70, 50)
        rects.append((x0, x0 + rng.uniform(4, 20), y0, y0 + rng.uniform(4, 20)))
    polys = [[(rng.uniform(-80, 80), rng.uniform(-80, 80))]]      # < 3 points: skipped
    polys.append([(10.04, 60.0), (30.0, 62.51), (22.2, 75.0), (12.0, 71.0)])
    for high in (False, True):
        doc, _ = gui_policy.gui_policy_doc(rects, polys, high)
        new = Policy.model_validate(doc)
        _assert_wgs84(new, "gui")
        metres = gui_policy.metre_constraints(rects, polys, gui_policy.read_base()[0], high)
        sent = [[(v["x"], v["y"]) for v in c["vertices"]]
                for c in metres if c["type"] == "polygon_fence"]
        got = [c for c in new.constraints if c.type == "polygon_fence"]
        assert len(sent) == len(got) == 5
        worst = 0.0
        for ring, c in zip(sent, got):
            # the GUI's 0.1 m values: the policy's view is exact ...
            assert [(v.x, v.y) for v in c.vertices] == ring, (c.id, ring)
            # ... and the projection itself within a micrometre
            worst = max(worst, _err(new, c, ring))
        assert worst <= MICRO_M, worst
        old = Policy.model_validate({"policy_id": "gui-mission", "version": doc["version"],
                                     "constraints": metres})
        a, b = Shield(new, legacy_metres=True), Shield(old, legacy_metres=True)
        assert _rings(a) == _rings(b)
        # fly both through the first zone, tick for tick
        x0, x1, y0, y1 = rects[0]
        s = State(x=x0 - 30.0, y=round((y0 + y1) / 2, 1), up=45.0 if high else 20.0)
        acted = 0
        for k in range(150):
            act = Action4D(vx=4.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)
            da, db = a.filter(s, act, t=k * 0.1), b.filter(s, act, t=k * 0.1)
            assert da.emitted == db.emitted, (k, da.emitted, db.emitted)
            assert [v.rule_id for v in da.violations] == [v.rule_id for v in db.violations]
            acted += bool(da.violations)
            e = da.emitted
            s = State(x=s.x + e.vx * 0.1, y=s.y + e.vy * 0.1, up=s.up)
        assert acted, "the flight never met the zone; the comparison proved nothing"


def test_gui_writer_without_a_readable_base():
    doc, notes = gui_policy.gui_policy_doc([(0.0, 10.0, 0.0, 10.0)], [], False,
                                           ROOT / "policies" / "no_such_base.yaml")
    assert notes and "base policy read failed" in notes[0]
    pol = Policy.model_validate(doc)
    _assert_wgs84(pol, "fallback")
    assert pol.frame_origin == gui_policy.PAS_HOME
    assert [(v.x, v.y) for v in pol.constraints[0].vertices] == \
        [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


TAIPEI_CORRIDOR = {"id": "kept-corridor", "type": "corridor", **RULE,
                   "centerline": [{"lat": 25.0420, "lon": 121.5305},
                                  {"lat": 25.0430, "lon": 121.5305}],
                   "width_m": 20.0, "altitude_floor_m": 5.0, "altitude_ceiling_m": 60.0}


def test_gui_writer_takes_the_base_frame_origin():
    """A base that states no `origin` but has lat/lon points has a frame
    (Policy.frame_origin: the mean of its first polygon fence). The drawn
    zones are written about THAT origin, not PAS_HOME, so they share one
    frame with the lat/lon rules the base keeps (here a corridor)."""
    base = yaml.safe_load((ROOT / "policies" / "wgs84_taipei.yaml").read_text(encoding="utf-8"))
    base.pop("origin")
    base["constraints"].append(copy.deepcopy(TAIPEI_CORRIDOR))
    with tempfile.TemporaryDirectory() as d:
        f = Path(d) / "base.yaml"
        f.write_text(yaml.safe_dump(base, sort_keys=False), encoding="utf-8")
        frame = load_policy(f, runtime=False).frame_origin
        doc, notes = gui_policy.gui_policy_doc([(-20.0, -10.0, 5.0, 15.0)], [], False, f)
    assert frame is not None and frame != gui_policy.PAS_HOME, frame
    assert notes == [], notes
    pol = Policy.model_validate(doc)
    _assert_wgs84(pol, "taipei base, no stated origin")
    assert pol.frame_origin == frame
    zone = _rule(pol, "gui-nfz-1")
    for got, want in zip(_true_view(pol, zone), [(-20.0, 5.0), (-10.0, 5.0),
                                                 (-10.0, 15.0), (-20.0, 15.0)]):
        assert math.dist(got, want) <= MICRO_M, (got, want)
    # the kept corridor is the base's own lat/lon, untouched
    kept = [c for c in doc["constraints"] if c["id"] == "kept-corridor"]
    assert kept and kept[0]["centerline"] == TAIPEI_CORRIDOR["centerline"], kept
    # a metre-only base has no frame: PAS_HOME, said in a note
    with tempfile.TemporaryDirectory() as d:
        f = Path(d) / "metre_base.yaml"
        f.write_text(yaml.safe_dump(METRE_DOC, sort_keys=False), encoding="utf-8")
        doc, notes = gui_policy.gui_policy_doc([(0.0, 10.0, 0.0, 10.0)], [], False, f)
    assert notes and "has no frame origin" in notes[0], notes
    assert Policy.model_validate(doc).frame_origin == gui_policy.PAS_HOME


# ------------------------------------------------------ the legacy read form

METRE_DOC = {"policy_id": "legacy-metre", "version": "0.0.1", "constraints": [
    {"id": "nfz-a", "type": "polygon_fence", **RULE, "margin_m": 1.0,
     "vertices": [{"x": 10, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 10}]}]}


def test_point_form_labels_every_form():
    forms = {p.name: load_policy(p, runtime=False).point_form for p in POLICIES}
    assert set(forms.values()) <= {POINTS_WGS84, POINTS_NONE}, forms
    # 27 with points (all lat/lon), 6 with none (stand-off, envelopes)
    assert sum(f == POINTS_WGS84 for f in forms.values()) == 27, forms
    assert sum(f == POINTS_NONE for f in forms.values()) == 6, forms
    assert Policy.model_validate(copy.deepcopy(METRE_DOC)).point_form == POINTS_LEGACY_METRES
    # a snapshot written by the metre writes of before 2026-11-17
    sh = Shield(load_policy(ROOT / "policies" / "sim_demo_policy.yaml"))
    sh.wgs84_writes = False
    rest(api.apply_dynamic_nfz, sh, {"id": "old-write", **RULE, "margin_m": 1.0,
                               "vertices": [{"x": 23, "y": 6}, {"x": 31, "y": 6},
                                            {"x": 31, "y": 14}]})
    snap = Policy.model_validate(json.loads(sh.policy.model_dump_json()))
    assert snap.point_form == POINTS_LEGACY_MIXED
    # an event on a metre-only policy is refused since 2026-11-18 (no WGS84
    # frame to write it in; tests/test_wgs84_frameless_writers.py), so the
    # policy stays as it was loaded
    leg = Shield(Policy.model_validate(copy.deepcopy(METRE_DOC)), legacy_metres=True)
    try:
        rest(api.apply_dynamic_nfz, leg, {"id": "z", **RULE, "margin_m": 1.0,
                                    "vertices": [{"x": 50, "y": 0}, {"x": 60, "y": 0},
                                                 {"x": 60, "y": 9}]})
    except api.ApiError as e:
        assert e.status == 422 and "no WGS84 frame origin" in e.detail, e.detail
    else:
        raise AssertionError("an event on a metre-only policy wrote metres")
    assert leg.policy.point_form == POINTS_LEGACY_METRES and leg.policy.generation == 0


def test_the_legacy_form_is_only_read_back():
    """The inventory's other side. Until 2026-11-18 three writers could still
    produce the metre form; now none does (tests/test_wgs84_frameless_writers.py
    proves each refusal). What is left is READING: the lock's derived recipes
    on a metre base are rebuilt as the recorded runs flew them (bundle.derive
    legacy_read, which policy_candidates passes), in the legacy form; no
    recipe records `wgs84_writes: false` today; demo/gen_random_scenario.py
    has no metre writer (--metres is refused by name)."""
    lock = load_lock()
    derived = lock.get("derived") or []
    assert not any(d.get("wgs84_writes") is False for d in derived), derived
    by_name = dict(policy_candidates())
    forms = {d["name"]: by_name[d["name"]].point_form for d in derived if d["name"] in by_name}
    assert len(forms) == len(derived), (forms, [d["name"] for d in derived])
    assert all(f in LEGACY_POINT_FORMS for f in forms.values()), forms
    print(f"  derived lock recipes (read back): {forms}")
    src = (ROOT / "demo" / "gen_random_scenario.py").read_text(encoding="utf-8")
    assert "METRES_REMOVED" in src and "DEPRECATED (WP1-14)" not in src
    assert "write = policy_yaml" not in src


def test_flight_refuses_the_legacy_form_and_labels_its_read():
    """Since 2026-11-18 (review of WP1-14) the flight loader refuses a metre-only
    document; until then it flew it, labelled. Every policies/*.yaml flies
    with no label; a refusal of one of them for its point form is a failure,
    not a skip. The legacy form loads under legacy_read only (a re-run of a
    recorded flight), and its record says so."""
    n = 0
    for p in POLICIES:
        try:
            pol, rec = load_for_flight(yaml_path=p)
        except LegacyPointFormRefused as e:
            raise AssertionError(f"{p.name}: {e}")
        except ValueError:
            continue
        assert "point_form" not in rec and "point_note" not in rec, (p.name, rec)
        assert "legacy_read" not in rec, (p.name, rec)
        n += 1
    assert n >= 32, n
    with tempfile.TemporaryDirectory() as d:
        f = Path(d) / "legacy.yaml"
        f.write_text(yaml.safe_dump(METRE_DOC, sort_keys=False), encoding="utf-8")
        try:
            load_for_flight(yaml_path=f)
        except LegacyPointFormRefused as e:
            assert POINTS_LEGACY_METRES in str(e) and "VLAGUARD_LEGACY_READ" in str(e), e
        else:
            raise AssertionError("a metre-only document was flown")
        pol, rec = load_for_flight(yaml_path=f, legacy_read=True)
    assert rec["legacy_read"] is True
    assert rec["point_form"] == POINTS_LEGACY_METRES == pol.point_form
    assert rec["point_note"] == LEGACY_POINT_NOTE and "legacy read form" in LEGACY_POINT_NOTE


# ------------------------------------------------------------- the hashes

def test_no_stored_policy_hash_moves():
    lock = load_lock()["policies"]
    for p in POLICIES:
        pol = load_policy(p, runtime=False)
        assert lock[f"{pol.policy_id}@{pol.version}"]["policy_hash"] == pol.policy_hash, p.name
    assert check_lock() == []


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


