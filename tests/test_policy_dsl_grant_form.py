"""The Policy DSL in the grant's own form: names, WGS84, strictness, layers,
the nine rule types, and cross-loading with the reference implementation.

Run either way:
    pytest tests/test_policy_dsl_grant_form.py -v
    python tests/test_policy_dsl_grant_form.py

WHY THIS FILE EXISTS

On 2026-10-05 the loader refused the grant's own example. The reference
repository's bundles/itri-icl-2026-demo.yaml - the policy the grant's Policy DSL
page prints - failed with "`issued_at` is not a policy field", and with that
removed it failed four ways more (project_fix, geometry.vertices,
altitude_min_m, altitude_max_m). Its bundle failed with the same four. Meanwhile
eleven of twenty-five deliberately broken policies loaded without a word: a
misspelt ceiling fell back to 1000 m, a bow-tie fence, an infinite speed cap,
a NaN altitude band, duplicate rule ids, `version: banana`.

The PI's ruling of 2026-10-06 was to conform to the grant rather than ask for
waivers. So the tests below are mostly the grant's text turned into assertions,
each paired with the refusal that keeps it honest. "Shown failing on the old
code" notes refer to the 401305a versions of guardrail/models.py, bundle.py and
projection.py; the evidence is in the unit report.
"""
import copy
import json
import math
import random
import sys
import tarfile
import tempfile
import io
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import guardrail.bundle as B                                       # noqa: E402
from guardrail.models import (GRANT_TYPES, HASH_SCHEME,             # noqa: E402
                              MODE_ACTIONS, PRIOR_SCHEME_V2, REFERENCE_FORM,
                              RUNTIME_TYPES, SHIELD_ACTS_ON, Action4D, CircleFence,
                              Constraint, DynamicNFZ, Policy, State,
                              load_layered, load_policy, merge_layers,
                              policy_from_raw, polygon_problem)
from guardrail.shield import Shield                                # noqa: E402

REF_REPO = ROOT / "kuanting-vla-uav-guardrail"
REF_DEMO = REF_REPO / "bundles" / "itri-icl-2026-demo.yaml"
REF_BUNDLE = REF_REPO / "bundles" / "itri-icl-2026-demo-v0.3.0.tar.gz"
DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
WGS84 = ROOT / "policies" / "wgs84_taipei.yaml"
SKIP = "SKIP"

# Printed by the reference implementation's own loader (its .venv, Python
# 3.11.15) on 2026-10-06: `ingest_file(p).policy_hash`, and the frame origin of
# the demo (`ir.projection.origin_lat / origin_lon`).
REF_HASHES = {
    "bundles/itri-icl-2026-demo.yaml":
        "sha256:31cafbb3dbe138c2ee63c5fcec078ad1d0bde98da96440c0cb4ffc8be30f840b",
    "bundles/projectairsim-demo.yaml":
        "sha256:290e81e27906ced5f48dead99cbf244e5325ba3ff05c7cf5f757c6f584896563",
    "bundles/sitl-flight-demo.yaml":
        "sha256:3bcbba49f0f7f6d865e26e976f1a907583339ddcec5eaa9e7e38969f56d86a07",
}
REF_ORIGIN = (25.042450000000002, 121.53139999999999)

# The grant's worked example (Policy DSL page, "Worked DSL example"), verbatim.
WORKED_EXAMPLE = """\
policy_id: itri-icl-2026-demo
version: 0.3.0
generation: 0
issued_at: 2026-04-28T09:00:00Z
layers_merged: [regulation, site, mission]

constraints:
  - id: nfz-school-yard
    type: polygon_fence
    constraint_type: hard
    scope: global
    priority: P0
    layer: site
    geometry:
      vertices:
        - { lat: 25.0421, lon: 121.5310 }
        - { lat: 25.0428, lon: 121.5310 }
        - { lat: 25.0428, lon: 121.5318 }
        - { lat: 25.0421, lon: 121.5318 }
      altitude_floor_m: 0
      altitude_ceiling_m: 200
      altitude_ref: AGL
    valid_time:
      recurrence:
        days: [Mon, Tue, Wed, Thu, Fri]
        start_time: "07:30"
        end_time: "17:30"
    violation_action: RTL

  - id: corridor-river-east
    type: corridor
    constraint_type: hard
    scope: mission
    priority: P0
    layer: mission
    geometry:
      centerline:
        - { lat: 25.0500, lon: 121.5400 }
        - { lat: 25.0560, lon: 121.5450 }
      width_m: 80
      altitude_floor_m: 30
      altitude_ceiling_m: 80
      altitude_ref: AGL
    violation_action: project_fix

  - id: envelope-default
    type: altitude_envelope
    constraint_type: soft
    scope: global
    priority: P1
    layer: regulation
    altitude_min_m: 5
    altitude_max_m: 120
    altitude_ref: AGL
    violation_action: brake
"""


def _tmp(name: str) -> Path:
    return Path(tempfile.mkdtemp(prefix="dslgf_")) / name


def _yaml(text: str, name: str = "p.yaml") -> Path:
    p = _tmp(name)
    p.write_text(text, encoding="utf-8")
    return p


def _raw(**over):
    """A minimal valid metre policy as a dict, with overrides."""
    doc = {"policy_id": "t", "version": "0.1.0", "constraints": [
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 30}]}
    doc.update(over)
    return doc


def _refused(fn, *words):
    try:
        fn()
    except ValueError as e:
        for w in words:
            assert w in str(e), (w, str(e)[:500])
        return str(e)
    raise AssertionError(f"accepted; expected a refusal mentioning {words}")


def _square(fid, x0, y0, side, **kw):
    return {"id": fid, "type": "polygon_fence",
            "vertices": [{"x": x0, "y": y0}, {"x": x0 + side, "y": y0},
                         {"x": x0 + side, "y": y0 + side}, {"x": x0, "y": y0 + side}],
            **kw}


def _reference_available() -> bool:
    import wp1_roundtrip_kpi as W
    return W.REFERENCE_PY.is_file()


# --------------------------------------------------------------------------- #
# the grant's own documents
# --------------------------------------------------------------------------- #

def test_the_grant_demo_document_loads_exactly_as_written():
    """Shown failing on the old code: ValueError "`issued_at` is not a policy
    field", and four ValidationErrors once that line was removed."""
    if not REF_DEMO.is_file():
        return SKIP
    pol = load_policy(REF_DEMO, runtime=False)
    fence, env = pol.constraints
    assert (fence.type, fence.scope, fence.layer, fence.violation_action,
            fence.altitude_ref) == ("polygon_fence", "global", "site", "project_fix", "AGL")
    assert all(v.geographic for v in fence.vertices)
    assert (fence.altitude_floor_m, fence.altitude_ceiling_m, fence.margin_m) == (0.0, 200.0, 0.0)
    assert (env.alt_min_m, env.alt_max_m, env.constraint_type, env.priority,
            env.violation_action, env.layer) == (5.0, 120.0, "soft", "P1", "brake", "regulation")
    assert pol.issued_at == "2026-04-28T09:00:00+00:00", pol.issued_at
    assert pol.origin is None


def test_the_grant_worked_example_loads_verbatim():
    """The Policy DSL page's own example: layers_merged, a schedule on the
    fence, a corridor with a geometry block, RTL. Not one key changed."""
    pol = load_policy(_yaml(WORKED_EXAMPLE), runtime=False)
    assert pol.layers_merged == ["regulation", "site", "mission"]
    fence, corr, env = pol.constraints
    assert fence.violation_action == "RTL" and fence.valid_time is not None
    assert corr.type == "corridor" and corr.width_m == 80.0
    assert (corr.altitude_floor_m, corr.altitude_ceiling_m) == (30.0, 80.0)
    assert env.alt_max_m == 120.0
    # RTL is requested by the Shield, flown only by an autopilot - and says so.
    assert any("RTL" in n for n in pol.runtime_notes()), pol.runtime_notes()


def test_every_reference_document_loads_and_hashes_as_the_reference_does():
    """Not just "loads": the reference's own hash of each document, reproduced
    from our model. A loader that read a field wrong would get another hash."""
    if not REF_REPO.is_dir():
        return SKIP
    for rel, want in REF_HASHES.items():
        pol = load_policy(REF_REPO / rel, runtime=False)
        assert pol.reference_hash() == want, (rel, pol.reference_hash())
        assert pol.hash_form(want) == REFERENCE_FORM, rel


def test_the_frame_is_derived_the_way_the_reference_derives_it():
    """No origin in the grant's form: the reference takes the mean of the first
    polygon's vertices. Same float, to the last bit, so the same lat/lon lands
    on the same metres - transposed, as this project's frame is (north, east)."""
    if not REF_DEMO.is_file():
        return SKIP
    pol = load_policy(REF_DEMO, runtime=False)
    assert pol.frame_origin == REF_ORIGIN, pol.frame_origin
    v = pol.constraints[0].vertices[2]
    east = math.radians(v.lon - REF_ORIGIN[1]) * 6_378_137.0 * math.cos(math.radians(REF_ORIGIN[0]))
    north = math.radians(v.lat - REF_ORIGIN[0]) * 6_378_137.0
    assert (v.x, v.y) == (north, east), ((v.x, v.y), (north, east))


def test_the_reference_bundle_loads_under_a_named_form():
    """Shown failing on the old code: 4 ValidationErrors (project_fix,
    vertices, alt_min_m, alt_max_m)."""
    if not REF_BUNDLE.is_file():
        return SKIP
    chk = B.check_bundle(REF_BUNDLE)
    assert chk.hash_form == REFERENCE_FORM, chk.hash_form
    assert chk.signature == B.SIG_UNSIGNED, "the reference signs with a placeholder"
    assert chk.policy.reference_hash() == REF_HASHES["bundles/itri-icl-2026-demo.yaml"]


def test_an_edited_reference_bundle_is_refused():
    """The reference's own loader re-validates and re-hashes; ours must refuse
    an IR edited after the fact just as it does its own bundles."""
    if not REF_BUNDLE.is_file():
        return SKIP
    with tarfile.open(REF_BUNDLE, "r:gz") as tar:
        m = {x.name: tar.extractfile(x).read() for x in tar.getmembers()}
    ir = json.loads(m["ir.json"])
    ir["constraints"][0]["geometry"]["altitude_ceiling_m"] = 20.0
    m["ir.json"] = json.dumps(ir, sort_keys=True, separators=(",", ":")).encode()
    bad = _tmp("edited.tar.gz")
    with tarfile.open(bad, "w:gz") as tar:
        for name, data in m.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    _refused(lambda: B.check_bundle(bad), "policy_hash mismatch")


def test_a_reference_form_bundle_is_read_by_the_reference_loader():
    """The other direction (WP1-05): written here with ir_form="reference",
    read by the reference implementation's own code in its own 3.11 env.
    Without that env the local half still runs; the cross half says SKIP."""
    if not REF_DEMO.is_file():
        return SKIP
    pol = load_policy(REF_DEMO, runtime=False)
    out = B.write_bundle(pol, _tmp("ref.tar.gz"), signer=None, ir_form="reference")
    chk = B.check_bundle(out)
    assert chk.hash_form == REFERENCE_FORM and chk.policy.policy_hash == pol.policy_hash
    if not _reference_available():
        return SKIP
    import wp1_roundtrip_kpi as W
    got = W.run_reference(
        "import json; from policy_dsl import load_bundle; "
        f"print(json.dumps({{'h': load_bundle({str(out)!r}).policy_hash}}))")
    assert got.get("ok"), got
    assert got["h"] == pol.reference_hash(), got


def test_a_policy_the_reference_cannot_represent_is_not_written_in_its_form():
    pol = load_policy(WGS84)         # origin, margin ring, schedule, kinematics
    why = _refused(lambda: B.write_bundle(pol, _tmp("x.tar.gz"), signer=None,
                                          ir_form="reference"), "cannot represent")
    for w in ("origin", "margin_m", "valid_time", "kinematic_envelope"):
        assert w in why, (w, why)


# --------------------------------------------------------------------------- #
# names and shapes
# --------------------------------------------------------------------------- #

GRANT_FENCE = {"id": "f", "type": "polygon_fence", "margin_m": 1.0, "geometry": {
    "vertices": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 10}],
    "altitude_floor_m": 0, "altitude_ceiling_m": 1000}}
OUR_FENCE = {"id": "f", "type": "polygon_fence", "margin_m": 1.0,
             "vertices": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 10}],
             "altitude_floor_m": 0, "altitude_ceiling_m": 1000}


def test_the_grant_shape_and_ours_are_one_policy():
    a = Policy.model_validate(_raw(constraints=[
        GRANT_FENCE, {"id": "alt", "type": "altitude_envelope",
                      "altitude_min_m": 5, "altitude_max_m": 30},
        {"id": "kin", "type": "kinematic_envelope", "speed_max": 5,
         "climb_rate_max": 2, "turn_rate_max": 45}]))
    b = Policy.model_validate(_raw(constraints=[
        OUR_FENCE, {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 30},
        {"id": "kin", "type": "kinematic_envelope", "speed_max_mps": 5,
         "climb_rate_max_mps": 2, "yaw_rate_max_dps": 45}]))
    assert a.policy_hash == b.policy_hash
    assert a.canonical_ir() == b.canonical_ir()


def test_a_grant_shaped_fence_gets_the_references_defaults_not_ours():
    """The same geometry block means the same zone to both implementations:
    0-200 m and no margin ring. Our flat form keeps ours (0-1000 m, 1 m)."""
    g = dict(GRANT_FENCE)
    g["geometry"] = {"vertices": GRANT_FENCE["geometry"]["vertices"]}
    g.pop("margin_m")
    f = Policy.model_validate(_raw(constraints=[g])).constraints[0]
    assert (f.altitude_floor_m, f.altitude_ceiling_m, f.margin_m) == (0.0, 200.0, 0.0)
    o = dict(OUR_FENCE)
    for k in ("margin_m", "altitude_floor_m", "altitude_ceiling_m"):
        o.pop(k)
    f = Policy.model_validate(_raw(constraints=[o])).constraints[0]
    assert (f.altitude_floor_m, f.altitude_ceiling_m, f.margin_m) == (0.0, 1000.0, 1.0)


def test_two_spellings_of_one_field_are_refused_not_resolved():
    _refused(lambda: Policy.model_validate(_raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5,
         "altitude_max_m": 30, "alt_max_m": 300}])), "name the same field")
    both = copy.deepcopy(GRANT_FENCE)
    both["altitude_ceiling_m"] = 400
    _refused(lambda: Policy.model_validate(_raw(constraints=[both])),
             "both inside `geometry` and beside it")
    odd = copy.deepcopy(GRANT_FENCE)
    odd["geometry"]["colour"] = "red"
    _refused(lambda: Policy.model_validate(_raw(constraints=[odd])), "unknown key 'colour'")
    _refused(lambda: Policy.model_validate(_raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", "geometry": {"vertices": []},
         "alt_min_m": 5, "alt_max_m": 30}])), "takes no `geometry`")


def test_the_grant_breach_actions_load_and_repair_is_stored_as_written():
    """Shown failing on the old code: only repair|brake were accepted."""
    for act in ("monitor_only", "project_fix", "brake", "loiter", "RTL", "land", "repair"):
        r = _raw()
        r["constraints"][0]["violation_action"] = act
        assert Policy.model_validate(r).constraints[0].violation_action == act
    r = _raw()
    r["constraints"][0]["violation_action"] = "rtl_now"
    _refused(lambda: Policy.model_validate(r), "violation_action")
    # `repair` stays `repair` in the IR, so no stored hash moved...
    assert load_policy(DEMO).constraints[0].violation_action == "repair"
    # ...and the reference form spells it the grant's way.
    pol = Policy.model_validate({"policy_id": "p", "constraints": [
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 30}]})
    assert pol.reference_ir()["constraints"][0]["violation_action"] == "project_fix"


def test_every_grant_rule_type_is_declarable():
    """All nine classes of the grant's taxonomy table validate and hash."""
    import typing
    members = typing.get_args(typing.get_args(Constraint)[0])
    names = {typing.get_args(m.model_fields["type"].annotation)[0] for m in members}
    assert set(GRANT_TYPES) <= names, set(GRANT_TYPES) - names
    pol = policy_from_raw(_all_nine(), runtime=False)
    assert sorted({c.type for c in pol.constraints}) == sorted(GRANT_TYPES)
    assert pol.policy_hash.startswith("sha256:")


def _all_nine() -> dict:
    sq = [{"lat": 25.0421, "lon": 121.5310}, {"lat": 25.0428, "lon": 121.5310},
          {"lat": 25.0428, "lon": 121.5318}]
    line = [{"lat": 25.0430, "lon": 121.5300}, {"lat": 25.0440, "lon": 121.5320}]
    return {"policy_id": "nine", "version": "0.1.0", "constraints": [
        {"id": "poly", "type": "polygon_fence", "geometry": {"vertices": sq}},
        {"id": "disc", "type": "circle_fence",
         "geometry": {"center": {"lat": 25.0450, "lon": 121.5330}, "radius_m": 20}},
        {"id": "tube", "type": "corridor",
         "geometry": {"centerline": line, "width_m": 40, "altitude_floor_m": 10,
                      "altitude_ceiling_m": 60}},
        {"id": "alt", "type": "altitude_envelope", "altitude_min_m": 5, "altitude_max_m": 120},
        {"id": "kin", "type": "kinematic_envelope", "speed_max": 8,
         "climb_rate_max": 3, "turn_rate_max": 60},
        {"id": "people", "type": "distance_envelope", "object_class": "people",
         "min_distance_m": 10},
        {"id": "moving", "type": "dynamic_nfz", "geometry": {"vertices": sq},
         "motion": {"vx_mps": 1.0, "vy_mps": 0.0, "yaw_rate_dps": 0.0}},
        {"id": "curfew-off", "type": "time_window_switch", "target_id": "poly",
         "active": False},
        {"id": "swap", "type": "corridor_swap", "target_id": "tube",
         "geometry": {"centerline": line, "width_m": 20, "altitude_floor_m": 10,
                      "altitude_ceiling_m": 60}},
    ]}


# --------------------------------------------------------------------------- #
# WGS84 is canonical; metres are legacy
# --------------------------------------------------------------------------- #

def test_a_geographic_policy_stores_lat_lon_and_never_metres():
    pol = load_policy(_yaml(WORKED_EXAMPLE), runtime=False)
    ir = json.dumps(pol.canonical_ir())
    assert '"x"' not in ir and '"y"' not in ir, "projected coordinates in the IR"
    assert '"lat": 25.0421' in ir
    # ...while the Shield still has metres.
    assert all(abs(v.x) < 2000 for v in pol.constraints[0].vertices)


def test_reprojection_is_deterministic_through_a_bundle():
    pol = load_policy(_yaml(WORKED_EXAMPLE), runtime=False)
    back = B.check_bundle(B.write_bundle(pol, _tmp("w.tar.gz"), signer=None)).policy
    assert [(v.x, v.y) for v in back.constraints[0].vertices] == \
        [(v.x, v.y) for v in pol.constraints[0].vertices]
    assert back.policy_hash == pol.policy_hash


def test_every_metre_policy_hashes_exactly_as_before():
    """The legacy form keeps its identity: every policy written in metres has
    the hash the lock pinned before this change."""
    lock = B.load_lock()["policies"]
    n = 0
    for f in sorted((ROOT / "policies").glob("*.yaml")):
        pol = load_policy(f)
        if pol.is_geographic:
            continue
        ent = lock[B.lock_key(pol)]
        ent = ent["shared_by_files"][f.name] if "shared_by_files" in ent else ent
        assert ent["policy_hash"] == pol.policy_hash, f.name
        n += 1
    assert n >= 27, n


def test_the_geographic_policy_keeps_its_old_hash_as_a_prior_form():
    """wgs84_taipei's v2 hash covered projected metres. It is still its hash
    under the named prior scheme; the lock keeps it and records the v3 form
    beside it, never instead of it."""
    pol = load_policy(WGS84)
    ent = B.load_lock()["policies"][B.lock_key(pol)]
    assert pol.prior_hashes()[PRIOR_SCHEME_V2] == ent["policy_hash"]
    assert pol.hash_form(ent["policy_hash"]) == PRIOR_SCHEME_V2
    assert ent["hashes"][HASH_SCHEME] == pol.policy_hash
    assert pol.policy_hash != ent["policy_hash"]


def test_a_point_in_two_frames_that_disagree_is_refused():
    raw = {"policy_id": "p", "origin": {"lat": 25.0, "lon": 121.0}, "constraints": [
        {"id": "f", "type": "polygon_fence", "vertices": [
            {"lat": 25.0, "lon": 121.0, "x": 1.0, "y": 2.0},
            {"lat": 25.001, "lon": 121.0}, {"lat": 25.001, "lon": 121.001}]}]}
    _refused(lambda: Policy.model_validate(raw), "mix frames")


def test_a_hot_applied_metre_fence_on_a_geographic_policy_still_bundles():
    """Mixing frames ACROSS rules is what a hot-apply produces; the bundle of
    the policy in force must load again, in the same frame."""
    from guardrail.models import PolygonFence
    pol = load_policy(_yaml(WORKED_EXAMPLE), runtime=False)
    Shield(pol).hot_apply(PolygonFence(**_square("nfz-hot", 500, 500, 20)))
    back = B.check_bundle(B.write_bundle(pol, _tmp("h.tar.gz"), signer=None)).policy
    assert back.policy_hash == pol.policy_hash and back.generation == 1
    assert back.constraints[-1].vertices[0].x == 500


# --------------------------------------------------------------------------- #
# strictness
# --------------------------------------------------------------------------- #

def test_the_strict_checks_each_refuse_their_defect():
    """One refusal per check (the negative corpus holds one file each too)."""
    cases = {
        "extra_forbidden": _raw(constraints=[{**OUR_FENCE, "altitude_cieling_m": 40}]),
        "finite": _raw(constraints=[{"id": "k", "type": "kinematic_envelope",
                                     "speed_max_mps": float("inf"),
                                     "climb_rate_max_mps": 2, "yaw_rate_max_dps": 45}]),
        "should match pattern": _raw(version="banana"),
        "no rules": _raw(constraints=[]),
        "used by 2 rules": _raw(constraints=[OUR_FENCE, {**OUR_FENCE}]),
        "cross": _raw(constraints=[{"id": "b", "type": "polygon_fence", "vertices": [
            {"x": 0, "y": 0}, {"x": 10, "y": 10}, {"x": 10, "y": 0}, {"x": 0, "y": 10}]}]),
        "no area": _raw(constraints=[{"id": "c", "type": "polygon_fence", "vertices": [
            {"x": 0, "y": 0}, {"x": 5, "y": 0}, {"x": 10, "y": 0}]}]),
        "shrinks the zone": _raw(constraints=[{**OUR_FENCE, "margin_m": -20}]),
        "no height in common": _raw(constraints=[
            {"id": "lo", "type": "altitude_envelope", "alt_min_m": 10, "alt_max_m": 20},
            {"id": "hi", "type": "altitude_envelope", "alt_min_m": 30, "alt_max_m": 40}]),
    }
    for word, raw in cases.items():
        _refused(lambda raw=raw: policy_from_raw(raw), word)


def test_the_conflict_lint_is_not_over_eager():
    """Disjoint bands are fine when the rules can never be in force together,
    or when one of them is soft. A lint that refused these would push authors
    to delete schedules - a worse policy."""
    day = {"recurrence": {"days": ["Mon"], "start_time": "08:00", "end_time": "17:00"}}
    night = {"recurrence": {"days": ["Mon"], "start_time": "18:00", "end_time": "06:00"}}
    ok = [
        [{"id": "lo", "type": "altitude_envelope", "alt_min_m": 10, "alt_max_m": 20,
          "valid_time": day},
         {"id": "hi", "type": "altitude_envelope", "alt_min_m": 30, "alt_max_m": 40,
          "valid_time": night}],
        [{"id": "lo", "type": "altitude_envelope", "alt_min_m": 10, "alt_max_m": 20},
         {"id": "hi", "type": "altitude_envelope", "alt_min_m": 30, "alt_max_m": 40,
          "constraint_type": "soft"}],
    ]
    for cons in ok:
        policy_from_raw(_raw(constraints=cons))
    # ...but a night window that wraps INTO the day window does conflict.
    wrap = {"recurrence": {"days": ["Sun"], "start_time": "22:00", "end_time": "09:00"}}
    _refused(lambda: policy_from_raw(_raw(constraints=[
        {**ok[0][0]}, {**ok[0][1], "valid_time": wrap}])), "no height in common")


def test_the_polygon_check_agrees_with_shapely_on_the_rings_it_was_tested_on():
    """polygon_problem is pure Python so models.py need not import shapely.
    It must say the same as shapely's is_valid (plus non-zero area) on random
    rings, on the same rings with one vertex doubled (a GIS export's habit),
    and on every fence this repository holds. That is the whole claim: the
    rings below, not every ring.

    Shown failing before 2026-10-07 on the doubled rings: a repeated
    consecutive vertex was refused as "a zero-length edge", which shapely
    accepts."""
    from shapely.geometry import Polygon as SPoly
    rng = random.Random(20261006)
    rings = []
    for _ in range(300):
        n = rng.randint(3, 8)
        rings.append([(rng.uniform(0, 10), rng.uniform(0, 10)) for _ in range(n)])
    doubled = []
    for r in rings[:150]:
        k = rng.randrange(len(r))
        doubled.append(r[:k + 1] + [r[k]] + r[k + 1:])
    rings += doubled
    for f in sorted((ROOT / "policies").glob("*.yaml")):
        for c in load_policy(f, runtime=False).constraints:
            if hasattr(c, "ring"):
                rings.append(c.ring())
    disagree = []
    for r in rings:
        ours = polygon_problem(r) is None
        sp = SPoly(r)
        theirs = sp.is_valid and sp.area > 1e-6
        if ours != theirs:
            disagree.append((r, polygon_problem(r), sp.is_valid))
    assert not disagree, disagree[:3]
    assert sum(polygon_problem(r) is not None for r in rings) > 50, \
        "the random rings must include invalid ones, or this compared nothing"
    assert sum(polygon_problem(r) is None for r in doubled) > 20, \
        "valid doubled rings must be among them, or the repeat case compared nothing"


def test_a_fence_with_a_doubled_vertex_loads():
    pol = policy_from_raw(_raw(constraints=[{"id": "d", "type": "polygon_fence",
        "vertices": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 0},
                     {"x": 10, "y": 10}, {"x": 0, "y": 10}]}]))
    assert len(pol.constraints[0].vertices) == 5, "kept as written"


def test_the_polygon_check_is_not_quadratic():
    """The first version compared every pair of edges: 0.73 s at 1000 vertices
    and 3.5 s at 2000, per validation, against the grant's 1 s cold start (and
    the merge validates several times). A 2000-vertex ring must take well
    under that budget now."""
    import time
    for n in (1000, 2000):
        ring = [(100 * math.cos(2 * math.pi * k / n), 100 * math.sin(2 * math.pi * k / n))
                for k in range(n)]
        t = time.perf_counter()
        assert polygon_problem(ring) is None
        dt = time.perf_counter() - t
        assert dt < 0.25, f"{n} vertices took {dt:.3f} s"
    # ...and it still finds a crossing in a big ring.
    bad = list(ring)
    bad[10], bad[1500] = bad[1500], bad[10]
    assert "cross" in (polygon_problem(bad) or "")


def test_duplicate_yaml_keys_are_refused():
    text = "policy_id: d\nconstraints:\n  - id: alt\n    type: altitude_envelope\n" \
           "    alt_min_m: 5\n    alt_max_m: 30\n    alt_max_m: 300\n"
    _refused(lambda: load_policy(_yaml(text)), "written twice")


# --------------------------------------------------------------------------- #
# layers
# --------------------------------------------------------------------------- #

def _layer(name, *rules, pid="p"):
    return Policy.model_validate({"policy_id": pid, "version": "0.1.0",
                                  "constraints": [{**r, "layer": name} for r in rules]})


REG_ALT = {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 120}
REG_FENCE = _square("airport", 0, 0, 50, margin_m=2.0, altitude_ceiling_m=300)


def test_a_mission_layer_may_tighten_and_add():
    reg = _layer("regulation", REG_ALT, REG_FENCE)
    mis = _layer("mission", {**REG_ALT, "alt_min_m": 10, "alt_max_m": 60},
                 _square("airport", -5, -5, 60, margin_m=2.0, altitude_ceiling_m=400),
                 _square("crane", 100, 100, 10), pid="delivery-7")
    m = merge_layers([reg, mis])
    assert m.policy_id == "delivery-7" and m.layers_merged == ["regulation", "mission"]
    alt = next(c for c in m.constraints if c.id == "alt")
    assert (alt.alt_min_m, alt.alt_max_m, alt.layer) == (10.0, 60.0, "mission")
    assert [c.id for c in m.constraints].count("airport") == 1
    assert any(c.id == "crane" for c in m.constraints)


TUBE_LINE = [{"x": 0, "y": 0}, {"x": 100, "y": 0}]
REG_TUBE = {"id": "tube", "type": "corridor", "width_m": 20, "centerline": TUBE_LINE}
REG_BAND = {"id": "band-tube", "type": "corridor", "width_m": 20,
            "altitude_floor_m": 10, "altitude_ceiling_m": 60,
            "centerline": [{"x": 0, "y": 50}, {"x": 100, "y": 50}]}
REG_KIN = {"id": "kin", "type": "kinematic_envelope", "speed_max_mps": 5,
           "climb_rate_max_mps": 2, "yaw_rate_max_dps": 45}
REG_CLEAR = {"id": "clear", "type": "obstacle_clearance", "min_clearance_m": 5}
MON_EVE = {"recurrence": {"days": ["Mon"], "start_time": "18:00", "end_time": "23:00"}}
REG_NIGHT = {"id": "night-alt", "type": "altitude_envelope", "alt_min_m": 5,
             "alt_max_m": 120, "valid_time": MON_EVE}
REG_DYN = {**_square("moving", 300, 300, 10), "type": "dynamic_nfz",
           "motion": {"vx_mps": 1.0}}
REG_SWITCH = {"id": "sw", "type": "time_window_switch", "target_id": "alt", "active": True}


def test_a_mission_layer_may_not_relax_a_hard_regulation_rule():
    """Each way a rule can be loosened, refused by name. The grant: "A
    mission-layer rule cannot relax a higher-priority hard constraint from
    regulation, but it can add stricter ones."

    The second half of the list was added on 2026-10-07: a review mutated
    each of those checks away and no test noticed (corridor_swap widening,
    schedule change, altitude reference, kinematic raise, clearance cut, the
    margin ring, corridor band, and the identical-only rule for the event
    types)."""
    reg = _layer("regulation", REG_ALT, REG_FENCE, REG_TUBE, REG_BAND, REG_KIN,
                 REG_CLEAR, REG_NIGHT, REG_DYN, REG_SWITCH)
    loosened = [
        ("widens the altitude band", {**REG_ALT, "alt_max_m": 150}),
        ("shrinks the keep-out area", _square("airport", 0, 0, 40, margin_m=2.0,
                                              altitude_ceiling_m=300)),
        ("narrows the fence's altitude band", {**REG_FENCE, "altitude_ceiling_m": 100}),
        ("turns a hard rule soft", {**REG_ALT, "constraint_type": "soft"}),
        ("lowers its priority", {**REG_ALT, "priority": "P2"}),
        ("weakens its breach action", {**REG_FENCE, "violation_action": "monitor_only"}),
        ("adds a schedule", {**REG_ALT, "valid_time": {"recurrence": {"days": ["Mon"]}}}),
        ("widens the corridor", {**REG_TUBE, "width_m": 30}),
        ("changes the rule's type", {"id": "alt", "type": "kinematic_envelope",
                                     "speed_max_mps": 5, "climb_rate_max_mps": 2,
                                     "yaw_rate_max_dps": 45}),
        ("turns off", {"id": "lift-alt", "type": "time_window_switch",
                       "target_id": "alt", "active": False}),
        # added 2026-10-07
        ("the swap widens the corridor", {"id": "wide-tube", "type": "corridor_swap",
                                          "target_id": "tube", "width_m": 60,
                                          "centerline": TUBE_LINE}),
        ("changes its schedule", {**REG_NIGHT, "valid_time": {"recurrence": {
            "days": ["Tue"], "start_time": "18:00", "end_time": "23:00"}}}),
        ("changes its altitude reference", {**REG_ALT, "altitude_ref": "MSL"}),
        ("raises speed_max_mps", {**REG_KIN, "speed_max_mps": 8}),
        ("reduces min_clearance_m", {**REG_CLEAR, "min_clearance_m": 3}),
        ("shrinks the keep-out area", {**REG_FENCE, "margin_m": 0.0}),
        ("widens the corridor's altitude band", {**REG_BAND, "altitude_floor_m": 5}),
        ("changes a dynamic_nfz rule", {**REG_DYN, "motion": {"vx_mps": 2.0}}),
        ("changes a time_window_switch rule", {**REG_SWITCH, "active": False}),
    ]
    for word, rule in loosened:
        mis = _layer("mission", rule)
        _refused(lambda mis=mis: merge_layers([reg, mis]), "layer merge refused", word)
    # The same overrides made STRICTER are accepted - so the refusals above
    # are about loosening, not about overriding at all.
    tighter = [{**REG_KIN, "speed_max_mps": 4}, {**REG_CLEAR, "min_clearance_m": 8},
               {**REG_FENCE, "margin_m": 3.0}, {**REG_BAND, "altitude_floor_m": 20},
               {**REG_TUBE, "width_m": 10}, {**REG_NIGHT}, {**REG_DYN}, {**REG_SWITCH},
               {"id": "narrow-tube", "type": "corridor_swap", "target_id": "tube",
                "width_m": 10, "centerline": TUBE_LINE}]
    for rule in tighter:
        merge_layers([reg, _layer("mission", rule)])


def test_layers_must_share_one_frame():
    """Different stated origins put metres in two frames; refused. And layers
    that mix metre and lat/lon points with NO stated origin are refused too:
    the metre rules sit in a frame derived from the first fence, and the merge
    re-orders the fences. Shown failing before 2026-10-07: the second case
    merged, re-anchoring the metre rule without a word."""
    a = Policy.model_validate({"policy_id": "a", "origin": {"lat": 25.0, "lon": 121.0},
                               "constraints": [{**REG_ALT, "layer": "regulation"}]})
    b = Policy.model_validate({"policy_id": "b", "origin": {"lat": 25.1, "lon": 121.0},
                               "constraints": [{**REG_KIN, "layer": "mission"}]})
    _refused(lambda: merge_layers([a, b]), "different origins")
    geo = Policy.model_validate({"policy_id": "g", "constraints": [
        {"id": "yard", "type": "polygon_fence", "layer": "site",
         "geometry": {"vertices": [{"lat": 25.0421, "lon": 121.5310},
                                   {"lat": 25.0428, "lon": 121.5310},
                                   {"lat": 25.0428, "lon": 121.5318}]}}]})
    metre = _layer("mission", _square("crane", 100, 100, 10))
    _refused(lambda: merge_layers([geo, metre]), "mix metre and lat/lon", "origin")


def test_a_soft_lower_rule_may_be_relaxed_and_a_same_layer_duplicate_may_not():
    reg = _layer("regulation", {**REG_ALT, "constraint_type": "soft"})
    m = merge_layers([reg, _layer("mission", {**REG_ALT, "alt_max_m": 200})])
    assert m.constraints[0].alt_max_m == 200.0
    _refused(lambda: merge_layers([_layer("site", REG_ALT), _layer("site", REG_ALT)]),
             "written twice in layer 'site'")


def test_a_merge_hashes_post_merge_and_the_cli_writes_its_bundle():
    reg = _yaml("policy_id: reg\nversion: 1.0.0\nconstraints:\n"
                "  - {id: alt, type: altitude_envelope, layer: regulation, "
                "alt_min_m: 5, alt_max_m: 120}\n", "reg.yaml")
    mis = _yaml("policy_id: m\nversion: 0.2.0\nconstraints:\n"
                "  - {id: alt, type: altitude_envelope, layer: mission, "
                "alt_min_m: 10, alt_max_m: 50}\n", "mis.yaml")
    merged = load_layered([reg, mis])
    assert merged.policy_id == "m" and merged.version == "0.2.0"
    out = _tmp("merged.tar.gz")
    assert B._main(["merge", str(reg), str(mis), "-o", str(out), "--unsigned"]) == 0
    assert B.check_bundle(out).policy.policy_hash == merged.policy_hash
    loose = _yaml("policy_id: m\nversion: 0.2.0\nconstraints:\n"
                  "  - {id: alt, type: altitude_envelope, layer: mission, "
                  "alt_min_m: 1, alt_max_m: 50}\n", "loose.yaml")
    assert B._main(["merge", str(reg), str(loose), "-o", str(out), "--unsigned"]) == 2


# --------------------------------------------------------------------------- #
# the new rule types and what the runtime does with them
# --------------------------------------------------------------------------- #

def test_a_circle_fence_is_enforced_and_stored_as_centre_and_radius():
    pol = policy_from_raw({"policy_id": "c", "constraints": [
        {"id": "disc", "type": "circle_fence", "center": {"x": 50, "y": 0},
         "radius_m": 10, "margin_m": 0.0}]})
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    assert sh.state_is_unsafe(State(x=50.0, y=9.9, up=10.0)), "inside the disc"
    assert not sh.state_is_unsafe(State(x=50.0, y=10.2, up=10.0)), \
        "0.48 % beyond the radius is the most the 32-gon may add"
    ir = pol.canonical_ir()["constraints"][0]
    assert "vertices" not in ir and ir["radius_m"] == 10.0, ir
    disc = pol.constraints[0]
    assert all(math.hypot(v.x - 50, v.y) >= 10.0 - 1e-9 for v in disc.vertices), \
        "the derived polygon must contain the circle (circumscribed)"
    # Circumscribed, not inscribed: in the direction of an edge's midpoint an
    # inscribed 32-gon reaches only r cos(pi/32) = 0.9952 r, so a point at
    # 0.997 r there is outside it and inside the circle. The vertex test above
    # cannot tell the two apart (an inscribed polygon's vertices lie ON the
    # circle); this one can. Added 2026-10-07 after a review swapped the two
    # and every test still passed.
    th = math.pi / CircleFence.SEGMENTS
    p = State(x=50.0 + 0.997 * 10 * math.cos(th), y=0.997 * 10 * math.sin(th), up=10.0)
    assert sh.state_is_unsafe(p), "a point inside the circle, between two vertices"
    _refused(lambda: Policy.model_validate({"policy_id": "c", "constraints": [
        {"id": "d", "type": "circle_fence", "center": {"x": 0, "y": 0}, "radius_m": 5,
         "vertices": [{"x": 0, "y": 0}]}]}), "derived from center")


def test_the_circle_polygon_resolution_is_pinned():
    """SEGMENTS decides the enforced zone of every circle_fence and is in
    neither the hash nor the IR (only centre and radius are). Changing it would
    change enforcement under an unchanged policy_hash, so it is pinned here and
    a change needs an IR_SCHEMA_VERSION bump (models.IR_SCHEMA_VERSION)."""
    assert CircleFence.SEGMENTS == 32
    pol = policy_from_raw({"policy_id": "c", "constraints": [
        {"id": "disc", "type": "circle_fence", "center": {"x": 0, "y": 0}, "radius_m": 10}]})
    assert len(pol.constraints[0].vertices) == 32


def test_declarable_only_rules_are_refused_for_flight_and_loaded_as_declarations():
    """A rule the Shield silently ignored would report a clean flight while
    enforcing nothing. So a flight loader refuses it, and says which and why;
    the DSL still validates, hashes and bundles it. Which types are
    declarable-only is RUNTIME_TYPES' business (it shrinks as the runtime
    grows); distance_envelope stays in that set for the foreseeable future."""
    p = _yaml(json.dumps(_all_nine()), "nine.json")
    msg = _refused(lambda: load_policy(p), "cannot enforce")
    declarable_only = set(GRANT_TYPES) - RUNTIME_TYPES
    assert "distance_envelope" in declarable_only
    for t in declarable_only:
        assert t in msg, t
    pol = load_policy(p, runtime=False)
    _refused(lambda: B.load_for_flight(None, p), "cannot enforce")
    b = B.write_bundle(pol, _tmp("nine.tar.gz"), signer=None)
    _refused(lambda: B.load_for_flight(b, allow_unverified=True), "cannot enforce")


def test_msl_is_declarable_and_refused_for_flight():
    raw = _raw()
    raw["constraints"][0]["altitude_ref"] = "MSL"
    pol = policy_from_raw(raw, runtime=False)
    assert pol.constraints[0].altitude_ref == "MSL"
    _refused(lambda: policy_from_raw(raw), "MSL")


# A take-off point ~100 m south of the school yard's centroid, outside every
# zone of the grant's demo and worked example. Until 2026-10-07 this was the
# centroid itself (25.04245, 121.5314) - so the test that "made the policy
# flyable" started the vehicle inside nfz-school-yard, the very failure the
# frame gate exists to prevent.
TAKE_OFF = "origin: {lat: 25.0415, lon: 121.5314}\n"
CENTROID = "origin: {lat: 25.04245, lon: 121.5314}\n"


def test_a_derived_frame_is_refused_for_flight_until_the_take_off_point_is_stated():
    """The SITL rail puts the vehicle's start (home) at the frame's (0, 0). A
    grant-form policy derives its frame from its first fence's centroid, so flying it as
    written would place every zone relative to the wrong point - with nothing
    failing. The DSL loads it; a flight loader says what is missing.

    Shown failing without the gate (frame_problem disabled): the grant demo
    flew with the vehicle's start placed INSIDE nfz-school-yard."""
    if not REF_DEMO.is_file():
        return SKIP
    msg = _refused(lambda: load_policy(REF_DEMO), "derived from the geometry", "origin")
    assert "25.0424500" in msg, msg
    _refused(lambda: B.load_for_flight(None, REF_DEMO), "derived from the geometry")
    stated = _yaml(REF_DEMO.read_text(encoding="utf-8") + "\n" + TAKE_OFF)
    pol = load_policy(stated)                         # flyable once stated
    assert pol.frame_problem() is None and pol.origin is not None
    # ...and the stated point really is a safe start, not merely a stated one.
    assert pol.start_conflicts() == []
    assert not Shield(pol).state_is_unsafe(State(x=0.0, y=0.0, up=20.0))
    assert load_policy(DEMO).frame_problem() is None, "metre policies are unaffected"
    assert load_policy(WGS84).frame_problem() is None, "an explicit origin is unaffected"


def test_the_frame_gate_guarantees_a_stated_frame_not_a_correct_one():
    """An origin written INSIDE a keep-out zone passes the frame gate: the gate
    cannot know where the vehicle is. Such a start is named
    (`start_conflicts`) and every flight loaded through load_for_flight records
    it. It is not refused: the frame origin is not the start on every rail
    (the CityLife scenarios spawn at (35, -20)), six generated CityLife
    policies have a zone over (0, 0), and wgs84_taipei.yaml (used by
    experiments/scenarios.yaml) states the yard's centroid.

    Shown failing before 2026-10-07: nothing named the start-inside case, and
    this file's own take-off point was the centroid."""
    if not REF_DEMO.is_file():
        return SKIP
    inside = _yaml(REF_DEMO.read_text(encoding="utf-8") + "\n" + CENTROID)
    pol = load_policy(inside)                         # passes the frame gate...
    assert pol.frame_problem() is None
    assert Shield(pol).state_is_unsafe(State(x=0.0, y=0.0, up=20.0)), \
        "...and the vehicle starts inside the school yard"
    (why,) = pol.start_conflicts()
    assert why.startswith("nfz-school-yard:") and "frame origin" in why, why
    _, src = B.load_for_flight(None, inside)
    assert any(n.startswith("nfz-school-yard:") for n in src["runtime_notes"]), src
    assert load_policy(WGS84).start_conflicts(), "wgs84_taipei states the centroid too"
    # A zone whose margin ring alone reaches the origin counts as well.
    ring = policy_from_raw(_raw(constraints=[_square("near", 0.5, 0.5, 10, margin_m=1.0)]))
    assert ring.start_conflicts()
    clear = policy_from_raw(_raw(constraints=[_square("far", 5, 5, 10, margin_m=1.0)]))
    assert clear.start_conflicts() == []


def test_a_flight_records_the_breach_actions_the_shield_does_not_perform():
    p = _yaml(WORKED_EXAMPLE + TAKE_OFF)
    _, src = B.load_for_flight(None, p)
    assert any("RTL" in n and "set_mode" in n for n in src["runtime_notes"]), src
    _, src = B.load_for_flight(None, DEMO)
    assert "runtime_notes" not in src, "a repair-only policy has nothing to note"


def test_the_breach_action_notes_say_what_the_shield_now_does():
    """Since 2026-10-07 the Shield dispatches on violation_action
    (guardrail/shield.py, ENFORCEMENT). Shown failing on the note of that
    morning, which said of every action but repair that "the Shield repairs
    then brakes for every rule and does not perform it itself": wrong for
    brake and monitor_only, which the Shield now performs, and for RTL, which
    it stops for and requests. Each note is checked against the Shield's own
    behaviour, so the two cannot drift apart unnoticed."""
    def env(action, kind="hard"):
        return policy_from_raw(_raw(constraints=[
            {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 60,
             "constraint_type": kind, "violation_action": action}]))
    for act in sorted(SHIELD_ACTS_ON):
        assert env(act).runtime_notes() == [], act
    for act in sorted(MODE_ACTIONS):
        (hard,) = env(act).runtime_notes()
        assert f"requests {act}" in hard and "set_mode" in hard, hard
        (soft,) = env(act, "soft").runtime_notes()
        assert "caps a soft rule's response at brake" in soft, soft
    assert set(SHIELD_ACTS_ON) | set(MODE_ACTIONS) >= {"repair", "project_fix",
                                                       "monitor_only", "brake",
                                                       "loiter", "RTL", "land"}
    # The Shield really does what the notes say. 58 m climbing at 3 m/s breaks
    # the 60 m ceiling inside the lookahead; standing still at 58 m is legal.
    state, climb = State(x=0.0, y=0.0, up=58.0), Action4D(vz_up=3.0)
    for act in ("brake", "RTL"):
        d = Shield(env(act)).filter(state, climb)
        assert d.braked and any("projection skipped" in r.detail for r in d.repairs), \
            (act, d.repairs)
    d = Shield(env("monitor_only")).filter(state, climb)
    assert d.violations and d.emitted == climb and not d.repairs, "recorded, not repaired"
    d = Shield(env("project_fix")).filter(state, climb)
    assert d.violations and not d.braked and d.emitted.vz_up < climb.vz_up, "repaired"


def test_a_dynamic_nfz_moves_as_declared():
    pol = policy_from_raw({"policy_id": "d", "constraints": [
        {"id": "m", "type": "dynamic_nfz",
         "vertices": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 10}, {"x": 0, "y": 10}],
         "motion": {"vx_mps": 2.0, "vy_mps": -1.0, "yaw_rate_dps": 90.0}}]}, runtime=False)
    z = pol.constraints[0]
    assert isinstance(z, DynamicNFZ)
    assert z.ring_at(0) == [(0, 0), (10, 0), (10, 10), (0, 10)]
    r = z.ring_at(1.0)                    # centre (5,5) -> (7,4); quarter turn N->E
    assert abs(r[0][0] - 12.0) < 1e-9 and abs(r[0][1] - (-1.0)) < 1e-9, r[0]


# --------------------------------------------------------------------------- #
# issued_at and producing the grant's form
# --------------------------------------------------------------------------- #

def test_issued_at_is_kept_hashed_and_carried_to_the_manifest():
    """Shown failing on the old code: the file was refused outright. Kept as
    the grant's document has it; the bundle may not contradict it."""
    pol = load_policy(_yaml(WORKED_EXAMPLE), runtime=False)
    later = pol.model_copy(update={"issued_at": "2026-05-01T00:00:00+00:00"})
    assert later.policy_hash != pol.policy_hash, "issued_at is part of the document"
    out = B.write_bundle(pol, _tmp("i.tar.gz"), signer=None)
    assert B.check_bundle(out).manifest["issued_at"] == "2026-04-28T09:00:00Z"
    _refused(lambda: B.write_bundle(pol, _tmp("j.tar.gz"), signer=None,
                                    issued_at="2026-10-06T00:00:00Z"), "contradict")
    B.write_bundle(pol, _tmp("k.tar.gz"), signer=None, issued_at="2026-04-28T09:00:00Z")


def test_the_grant_form_is_produced_and_reloads_to_the_same_rules():
    for src in (_yaml(WORKED_EXAMPLE), WGS84):
        pol = load_policy(src, runtime=False)
        g = pol.to_grant_form()
        back = policy_from_raw(json.loads(json.dumps(g)), runtime=False)
        assert back.to_grant_form() == g
        for a, b in zip(pol.constraints, back.constraints):
            pa = [(v.lat, v.lon) for v in getattr(a, "vertices", None) or []]
            pb = [(v.lat, v.lon) for v in getattr(b, "vertices", None) or []]
            assert pa == pb, a.id
    g = load_policy(_yaml(WORKED_EXAMPLE), runtime=False).to_grant_form()
    assert g["constraints"][0]["geometry"]["vertices"][0] == {"lat": 25.0421, "lon": 121.531}
    assert "origin" not in g


def test_a_metre_policy_needs_an_anchor_for_the_grant_form():
    pol = load_policy(DEMO)
    _refused(lambda: pol.to_grant_form(), "Pass origin=")
    g = pol.to_grant_form(origin=(25.04245, 121.5314))
    back = policy_from_raw(json.loads(json.dumps(g)))
    for a, b in zip(pol.constraints, back.constraints):
        for va, vb in zip(getattr(a, "vertices", []) or [], getattr(b, "vertices", []) or []):
            assert abs(va.x - vb.x) < 1e-6 and abs(va.y - vb.y) < 1e-6, (va, vb)
    assert back.constraints[0].violation_action == "project_fix", "grant spelling"
    kin = next(c for c in g["constraints"] if c["type"] == "kinematic_envelope")
    assert {"speed_max", "climb_rate_max", "turn_rate_max"} <= set(kin), kin


def test_the_export_cli_writes_the_grant_form():
    out = _tmp("g.yaml")
    assert B._main(["export", str(WGS84), "-o", str(out)]) == 0
    assert load_policy(out).to_grant_form() == load_policy(WGS84).to_grant_form()
    assert B._main(["export", str(DEMO)]) == 2, "a metre policy with no anchor"


# --------------------------------------------------------------------------- #
# review of 2026-10-07: every path below was mutated away in a review and no
# test noticed. Each test names what the pre-fix code did.
# --------------------------------------------------------------------------- #

# A rule type neither the flight loaders nor the Shield accept. Until
# 2026-10-07 these tests used a dynamic_nfz, which the Shield of that morning
# ignored; the Shield has enforced dynamic_nfz since (guardrail/shield.py), so
# the case that a loader must not hand the Shield a rule it cannot enforce is
# now made with distance_envelope.
UNENF_RAW = {"policy_id": "dist", "constraints": [
    {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 60},
    {"id": "people", "type": "distance_envelope", "object_class": "people",
     "min_distance_m": 10.0}]}


def test_load_bundle_applies_the_flight_gate():
    """Shown failing before 2026-10-07: load_bundle returned the policy of a
    bundle holding a rule every other flight loader refuses (then a 20 m
    dynamic_nfz; the Shield of that day reported the zone's centre as safe -
    the rule silently ignored)."""
    pol = policy_from_raw(UNENF_RAW, runtime=False)
    b = B.write_bundle(pol, _tmp("dist.tar.gz"), signer=None)
    assert Shield.unenforceable(pol), "the reason for the gate: the Shield cannot enforce it"
    _refused(lambda: B.load_bundle(b, require_signature=False), "cannot enforce", "people")
    assert B.load_bundle(b, require_signature=False, runtime=False).policy_hash == pol.policy_hash


def test_a_scenario_cannot_fly_a_bundle_the_shield_would_ignore():
    """guardrail/scenario_spec.py loads `bundle_path` scenarios through
    load_bundle - the stress-harness path the grant's scenarios take."""
    from guardrail.scenario_spec import PolicyRef
    pol = policy_from_raw(UNENF_RAW, runtime=False)
    b = B.write_bundle(pol, _tmp("dist.tar.gz"), signer=None)
    _refused(lambda: PolicyRef(bundle_path=str(b)).load(ROOT), "cannot enforce")


def test_the_loaders_never_hand_the_shield_a_rule_it_does_not_enforce():
    """RUNTIME_TYPES (what a flight may carry) must stay inside the Shield's
    own ENFORCED_TYPES, and both must refuse MSL. Every type the loaders turn
    away is either refused by the Shield too or enforced by it - the second is
    an over-refusal (safe), printed so it is not forgotten: since 2026-10-07
    the Shield enforces the three hot-apply classes, which wait for compiler
    templates before they join RUNTIME_TYPES (see its comment)."""
    from guardrail import shield as S
    assert RUNTIME_TYPES <= S.ENFORCED_TYPES, RUNTIME_TYPES - S.ENFORCED_TYPES
    nine = _all_nine()
    for rule in nine["constraints"]:
        if rule["type"] in RUNTIME_TYPES:
            continue
        # The switch and swap targets ("poly", "tube") are runtime types, so
        # they are kept and the lint still resolves them.
        rules = [r for r in nine["constraints"] if r["type"] in RUNTIME_TYPES] + [rule]
        pol = policy_from_raw({**nine, "origin": {"lat": 25.0410, "lon": 121.5310},
                               "constraints": rules}, runtime=False)
        assert pol.unenforced_rules(), rule["type"]
        if rule["type"] not in S.ENFORCED_TYPES:
            _refused(lambda: Shield(pol), "does not enforce", rule["id"])
    msl = _raw()
    msl["constraints"][0]["altitude_ref"] = "MSL"
    pol = policy_from_raw(msl, runtime=False)
    assert pol.unenforced_rules() and Shield.unenforceable(pol)
    print(f"      over-refused (the Shield enforces them, the loaders wait for the "
          f"compiler): {sorted(S.ENFORCED_TYPES - RUNTIME_TYPES)}")


def test_a_reference_form_bundle_is_an_exchange_format_not_a_flight_artefact():
    """Written with ir_form="reference", the IR writes every default out and
    spells `repair` as `project_fix`, so the policy rebuilt from it has a THIRD
    hash - neither the manifest's nor the source's. Shown failing before
    2026-10-07: load_for_flight flew it and recorded that third hash, which
    resolves to no policy. Now refused for flight; readable; and the source
    policy still recognises the manifest's hash."""
    pol = policy_from_raw(_raw(), runtime=True)       # no scope/layer, `repair`
    b = B.write_bundle(pol, _tmp("x.tar.gz"), signer=None, ir_form="reference")
    chk = B.check_bundle(b)
    declared = chk.manifest["policy_hash"]
    assert chk.hash_form == REFERENCE_FORM
    assert len({pol.policy_hash, declared, chk.policy.policy_hash}) == 3, \
        "three hashes: the reason it cannot be a flight artefact"
    assert pol.hash_form(declared) == REFERENCE_FORM, "the source still knows it"
    assert chk.source_record()["declared_policy_hash"] == declared
    _refused(lambda: B.load_for_flight(b, allow_unverified=True), "exchange format")
    _refused(lambda: B.load_bundle(b, require_signature=False), "exchange format")
    assert B.load_bundle(b, require_signature=False, runtime=False).policy_id == "t"
    # A bundle in this project's form carries the same hash in both places.
    own = B.check_bundle(B.write_bundle(pol, _tmp("o.tar.gz"), signer=None))
    rec = own.source_record()
    assert rec["declared_policy_hash"] == rec["loaded_policy_hash"] == pol.policy_hash


def test_the_lint_compares_every_pair_of_hard_height_limits():
    """A hard envelope against a hard corridor band; two hard corridors (the
    Shield requires the vehicle inside every active corridor at once - until
    2026-10-07 corridor pairs were skipped as "different places"); and AGL
    against MSL, which cannot be compared and is not."""
    env_vs_tube = _raw(constraints=[
        {"id": "env", "type": "altitude_envelope", "alt_min_m": 100, "alt_max_m": 120},
        {"id": "tube", "type": "corridor", "width_m": 20, "altitude_floor_m": 30,
         "altitude_ceiling_m": 80, "centerline": TUBE_LINE}])
    _refused(lambda: policy_from_raw(env_vs_tube), "env", "tube", "no height in common")
    two_tubes = _raw(constraints=[
        {"id": "low", "type": "corridor", "width_m": 20, "altitude_floor_m": 10,
         "altitude_ceiling_m": 20, "centerline": TUBE_LINE},
        {"id": "high", "type": "corridor", "width_m": 20, "altitude_floor_m": 30,
         "altitude_ceiling_m": 40, "centerline": TUBE_LINE}])
    _refused(lambda: policy_from_raw(two_tubes), "low", "high", "no height in common")
    overlapping = copy.deepcopy(two_tubes)
    overlapping["constraints"][1]["altitude_floor_m"] = 15
    policy_from_raw(overlapping)
    agl_msl = _raw(constraints=[
        {"id": "agl", "type": "altitude_envelope", "alt_min_m": 10, "alt_max_m": 20},
        {"id": "msl", "type": "altitude_envelope", "alt_min_m": 30, "alt_max_m": 40,
         "altitude_ref": "MSL"}])
    policy_from_raw(agl_msl, runtime=False)          # not compared: not a conflict


def test_the_lint_checks_what_a_switch_or_swap_points_at():
    swap_env = _raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 30},
        {"id": "s", "type": "corridor_swap", "target_id": "alt", "width_m": 20,
         "centerline": TUBE_LINE}])
    _refused(lambda: policy_from_raw(swap_env, runtime=False), "not a corridor")
    self_target = _raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 30},
        {"id": "sw", "type": "time_window_switch", "target_id": "sw", "active": False}])
    _refused(lambda: policy_from_raw(self_target, runtime=False), "cannot target itself")
    at_event = _raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 30},
        {"id": "a", "type": "time_window_switch", "target_id": "alt", "active": True},
        {"id": "b", "type": "time_window_switch", "target_id": "a", "active": False}])
    _refused(lambda: policy_from_raw(at_event, runtime=False), "not an event")


def test_check_bundle_runs_the_lint():
    """A bundle is a release of a DSL-valid policy. An IR with two rules of
    one id hashes correctly (write_bundle does not lint) and must still be
    refused on read."""
    dup = Policy.model_validate(_raw(constraints=[OUR_FENCE, {**OUR_FENCE}]))
    b = B.write_bundle(dup, _tmp("dup.tar.gz"), signer=None)
    _refused(lambda: B.check_bundle(b), "does not pass the DSL lint", "used by 2 rules")


def test_load_layered_applies_the_flight_gate():
    reg = _yaml("policy_id: reg\nconstraints:\n  - {id: alt, type: altitude_envelope, "
                "layer: regulation, alt_min_m: 5, alt_max_m: 120}\n", "reg.yaml")
    mis = _yaml("policy_id: m\nconstraints:\n  - {id: people, type: distance_envelope, "
                "layer: mission, object_class: people, min_distance_m: 10}\n", "mis.yaml")
    _refused(lambda: load_layered([reg, mis]), "cannot enforce", "people")
    assert len(load_layered([reg, mis], runtime=False).constraints) == 2


def test_a_grant_form_corridor_gets_the_references_band():
    """A corridor's geometry block, like a fence's, takes the reference's
    0-200 m band; the flat form keeps this project's 0-1000 m."""
    g = policy_from_raw(_raw(constraints=[{"id": "t", "type": "corridor", "geometry": {
        "centerline": TUBE_LINE, "width_m": 20}}])).constraints[0]
    assert (g.altitude_floor_m, g.altitude_ceiling_m) == (0.0, 200.0)
    f = policy_from_raw(_raw(constraints=[REG_TUBE])).constraints[0]
    assert (f.altitude_floor_m, f.altitude_ceiling_m) == (0.0, 1000.0)


def test_the_derived_frame_prefers_the_first_polygon_fence_then_any_geographic_rule():
    """The reference's rule is the first POLYGON_FENCE's mean, whatever comes
    before it. With no polygon this code takes the first geographic rule
    (the reference would take (0, 0), the Gulf of Guinea)."""
    line = [{"lat": 25.0500, "lon": 121.5400}, {"lat": 25.0560, "lon": 121.5450}]
    sq = [{"lat": 25.0421, "lon": 121.5310}, {"lat": 25.0428, "lon": 121.5310},
          {"lat": 25.0428, "lon": 121.5318}]
    tube = {"id": "tube", "type": "corridor", "geometry": {"centerline": line, "width_m": 80}}
    fence = {"id": "yard", "type": "polygon_fence", "geometry": {"vertices": sq}}
    both = Policy.model_validate({"policy_id": "p", "constraints": [tube, fence]})
    assert both.frame_origin == (sum(p["lat"] for p in sq) / 3, sum(p["lon"] for p in sq) / 3)
    only_tube = Policy.model_validate({"policy_id": "p", "constraints": [tube]})
    assert only_tube.frame_origin == (sum(p["lat"] for p in line) / 2,
                                      sum(p["lon"] for p in line) / 2)


def _json_doc(text: str):
    import yaml
    sys.path.insert(0, str(ROOT / "tools"))
    import wp1_roundtrip_kpi as W
    return W.json_compatible(yaml.safe_load(text))


def test_the_published_schema_describes_the_authoring_form():
    """The grant: the DSL is the hand-authored form and its JSON Schema export
    is "the published contract for external tooling". Shown failing before
    2026-10-07: policy_dsl.schema.json described this project's IR and
    rejected the grant's own documents (0/3 of the reference's, and the
    worked example)."""
    try:
        import jsonschema
    except ImportError:
        return SKIP
    schema = json.loads(B.SCHEMA_PATH.read_text(encoding="utf-8"))
    root = jsonschema.Draft202012Validator(schema)
    canon = jsonschema.Draft202012Validator({**schema, "$ref": "#/$defs/CanonicalIR"})
    docs = [_json_doc(WORKED_EXAMPLE)]
    if REF_REPO.is_dir():
        docs += [_json_doc(p.read_text(encoding="utf-8"))
                 for p in sorted((REF_REPO / "bundles").glob("*.yaml"))]
    for d in docs:
        errs = list(root.iter_errors(d))
        assert not errs, (d["policy_id"], errs[0].message)
        # The canonical IR is the stricter contract: the grant's names are
        # not in it, so the document as written is not an IR...
        assert list(canon.iter_errors(d)), d["policy_id"]
        # ...while the IR the loader makes of it is.
        ir = policy_from_raw(d, runtime=False).canonical_ir()
        assert not list(canon.iter_errors(ir)), d["policy_id"]
    for f in sorted((ROOT / "policies").glob("*.yaml")):
        errs = list(root.iter_errors(_json_doc(f.read_text(encoding="utf-8"))))
        assert not errs, (f.name, errs[0].message)
    # It says no, the way the loader does.
    base = _json_doc(WORKED_EXAMPLE)
    env = base["constraints"][2]
    bad = {
        "two names for one field": {**env, "alt_max_m": 300},
        "inside and beside geometry": {**base["constraints"][0], "vertices":
                                       base["constraints"][0]["geometry"]["vertices"]},
        "an unknown key in geometry": {**base["constraints"][0], "geometry": {
            **base["constraints"][0]["geometry"], "colour": "red"}},
        "a required field missing": {k: v for k, v in env.items() if k != "altitude_max_m"},
    }
    for name, rule in bad.items():
        d = copy.deepcopy(base)
        d["constraints"][2 if "field" in name else 0] = rule
        assert list(root.iter_errors(d)), f"the schema accepted {name}"
    assert list(root.iter_errors({**base, "constraints": []})), "no rules"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (reference repository, its env, or "
                      f"jsonschema missing)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
