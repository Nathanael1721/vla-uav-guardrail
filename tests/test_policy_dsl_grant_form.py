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

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}
# The same three in a YAML flow mapping, for the inline documents below.
_HP0_YAML = ", ".join(f"{k}: {v}" for k, v in HARD_P0.items())

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
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 30}]}
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
    return {"id": fid, "type": "polygon_fence", **HARD_P0,
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
    polygon's vertices. Same float, to the last bit. The projector is the
    reference's arithmetic as well, so `LocalProjection.to_local` puts the same
    lat/lon on the same metres, to the last bit - transposed, as this project's
    frame is (north, east). The policy's local view (`point.x` / `point.y`) is
    that projection rounded to the micrometre (IR schema 3.1.0, projection.py),
    so it is the reference's metres to within half a micrometre.

    Until 2026-11-04 this asserted that the rounded view itself was
    bit-identical. Schema 3.1.0 rounds it on purpose, and the old assertion
    failed in the lab checkout - (38.961822, 40.341935) against the reference's
    (38.9618217773572, 40.341935165238766) - while the branch's own test run
    skipped it, having no reference checkout."""
    if not REF_DEMO.is_file():
        return SKIP
    from guardrail.projection import LOCAL_QUANTUM_M, projection_for, quantize_m
    pol = load_policy(REF_DEMO, runtime=False)
    assert pol.frame_origin == REF_ORIGIN, pol.frame_origin
    v = pol.constraints[0].vertices[2]
    east = math.radians(v.lon - REF_ORIGIN[1]) * 6_378_137.0 * math.cos(math.radians(REF_ORIGIN[0]))
    north = math.radians(v.lat - REF_ORIGIN[0]) * 6_378_137.0
    assert projection_for(*REF_ORIGIN).to_local(v.lat, v.lon) == (north, east)
    assert (v.x, v.y) == (quantize_m(north), quantize_m(east)), ((v.x, v.y), (north, east))
    assert max(abs(v.x - north), abs(v.y - east)) <= LOCAL_QUANTUM_M / 2


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


def test_the_converter_writes_what_the_reference_can_hold_and_says_what_it_cannot():
    """WP1-05, the two-hash contract. The reference's Phase-1 DSL holds 0 of
    the shipped policies whole, so the whole-policy form converts nothing this
    project flies. The converter writes the representable subset and accounts
    for every rule: converted exactly, or left out with a reason. Its hash is
    the reference's hash of THAT document - never of the whole policy unless
    nothing was left out (`complete`), in which case it is the whole-policy
    reference form, byte for byte."""
    from guardrail.reference_convert import to_reference
    shipped = sorted((ROOT / "policies").glob("*.yaml"))
    with_projection = 0
    for f in shipped:
        pol = load_policy(f, runtime=False)
        conv = to_reference(pol)
        rec = conv.record()
        accounted = set(rec["rules"]) | set(rec["omitted"])
        assert accounted == {c.id for c in pol.constraints}, (f.name, accounted)
        assert all(rec["omitted"].values()), f.name           # every omission says why
        assert not conv.complete and pol.reference_hash() is None, f.name
        assert pol.policy_hash != conv.reference_hash, "the two hashes are different forms"
        with_projection += conv.doc is not None
        for r in (conv.doc or {}).get("constraints", []):
            assert r["type"] in ("polygon_fence", "altitude_envelope"), r
    assert with_projection == len(shipped) - 1, with_projection   # corridor_survey: none
    # A policy the reference can hold whole: the converter's document IS the
    # whole-policy reference form, and its hash the reference hash.
    sq = [{"lat": 25.0421, "lon": 121.5310}, {"lat": 25.0428, "lon": 121.5310},
          {"lat": 25.0428, "lon": 121.5318}]
    whole = policy_from_raw({"policy_id": "geo", "version": "0.1.0", "constraints": [
        {"id": "f", "type": "polygon_fence", **HARD_P0, "vertices": sq, "margin_m": 0.0},
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5,
         "alt_max_m": 90}]}, runtime=False)
    conv = to_reference(whole)
    assert conv.complete and conv.doc == whole.reference_ir()
    assert conv.reference_hash == whole.reference_hash() is not None
    # Nothing convertible: no document and no hash, never an invented one.
    kin = policy_from_raw(_raw(constraints=[{**REG_KIN}]))
    none = to_reference(kin)
    assert none.doc is None and none.reference_hash is None and "kin" in none.omitted
    # A metre fence needs a WGS84 anchor; a margin ring and a schedule are
    # left out unless asked for, and then written stricter and recorded.
    metre = policy_from_raw(_raw(constraints=[
        _square("nfz", 10, 10, 20, margin_m=2.0),
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5,
         "alt_max_m": 60, "valid_time": {"recurrence": {"days": ["Mon"]}}}]))
    plain = to_reference(metre)
    assert "no WGS84 anchor" in plain.omitted["nfz"] and "valid_time" in plain.omitted["alt"]
    anchored = to_reference(metre, origin=(25.0, 121.5))
    assert "margin_m 2" in anchored.omitted["nfz"]
    loose = to_reference(metre, origin=(25.0, 121.5), conservative=True)
    assert set(loose.conservative) == {"nfz", "alt"} and not loose.omitted
    assert "always in force" in loose.conservative["alt"]
    # The grown zone covers the zone the Shield enforces (polygon + margin).
    from shapely.geometry import Polygon
    from guardrail.geometry import keepout_area
    from guardrail.projection import LocalProjection
    pr = LocalProjection(25.0, 121.5)
    ring = [pr.to_local(v["lat"], v["lon"])
            for v in loose.doc["constraints"][0]["geometry"]["vertices"]]
    assert Polygon(ring).buffer(1e-6).covers(keepout_area(metre.constraints[0]))
    # The reference's own loader agrees on the hash, where its env exists.
    if not _reference_available():
        return SKIP
    import wp1_roundtrip_kpi as W
    import yaml
    docs = {}
    for f in shipped[:6] + [WGS84]:
        c = to_reference(load_policy(f, runtime=False), origin=(25.0424, 121.5314),
                         conservative=True)
        if c.doc is not None:
            p = _yaml(yaml.safe_dump(c.doc, sort_keys=False), f"{f.stem}.ref.yaml")
            docs[str(p)] = c.reference_hash
    got = W.run_reference(
        "import json; from policy_dsl import ingest_file; "
        f"print(json.dumps({{p: ingest_file(p).policy_hash for p in {list(docs)!r}}}))")
    assert got.get("ok"), got
    assert {p: got[p] for p in docs} == docs, got


def test_every_manifest_carries_the_reference_projection_and_it_is_verified():
    """The two-hash contract in the bundle: `hashes` names the whole policy
    under both forms (the reference's null where it cannot hold it), and
    since 2026-11-02 `reference_projection` names the reference's hash of
    what it CAN hold, with the rules left out and why. check_bundle
    recomputes it; a manifest that claims more is refused."""
    from guardrail.reference_convert import to_reference
    pol = load_policy(WGS84)
    b = B.write_bundle(pol, _tmp("p.tar.gz"), signer=None)
    chk = B.check_bundle(b)
    rec = chk.manifest["reference_projection"]
    assert rec == to_reference(pol).record()
    assert rec["rules"] == ["envelope-default"] and not rec["complete"]
    assert chk.manifest["hashes"][REFERENCE_FORM] is None
    with tarfile.open(b, "r:gz") as tar:
        m = {x.name: tar.extractfile(x).read() for x in tar.getmembers()}
    for name, edit in (("more rules", lambda r: r["rules"].append("kin-caps")),
                       ("another hash", lambda r: r.__setitem__("policy_hash", "sha256:" + "0" * 64)),
                       ("complete", lambda r: r.__setitem__("complete", True)),
                       ("a reason dropped", lambda r: r["omitted"].popitem())):
        man = json.loads(m[B.MANIFEST_NAME])
        edit(man["reference_projection"])
        out = _tmp("f.tar.gz")
        with tarfile.open(out, "w:gz") as tar:
            for n, data in {**m, B.MANIFEST_NAME: json.dumps(
                    man, indent=2, sort_keys=True).encode()}.items():
                info = tarfile.TarInfo(n)
                info.size = len(data)
                tar.addfile(info, io.BytesIO(data))
        _refused(lambda out=out: B.check_bundle(out), "reference_projection")


# --------------------------------------------------------------------------- #
# names and shapes
# --------------------------------------------------------------------------- #

GRANT_FENCE = {"id": "f", "type": "polygon_fence", **HARD_P0, "margin_m": 1.0, "geometry": {
    "vertices": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 10}],
    "altitude_floor_m": 0, "altitude_ceiling_m": 1000}}
OUR_FENCE = {"id": "f", "type": "polygon_fence", **HARD_P0, "margin_m": 1.0,
             "vertices": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 10}],
             "altitude_floor_m": 0, "altitude_ceiling_m": 1000}


def test_the_grant_shape_and_ours_are_one_policy():
    a = Policy.model_validate(_raw(constraints=[
        GRANT_FENCE, {"id": "alt", "type": "altitude_envelope", **HARD_P0,
                      "altitude_min_m": 5, "altitude_max_m": 30},
        {"id": "kin", "type": "kinematic_envelope", **HARD_P0, "speed_max": 5,
         "climb_rate_max": 2, "turn_rate_max": 45}]))
    b = Policy.model_validate(_raw(constraints=[
        OUR_FENCE, {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 30},
        {"id": "kin", "type": "kinematic_envelope", **HARD_P0, "speed_max_mps": 5,
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
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5,
         "altitude_max_m": 30, "alt_max_m": 300}])), "name the same field")
    both = copy.deepcopy(GRANT_FENCE)
    both["altitude_ceiling_m"] = 400
    _refused(lambda: Policy.model_validate(_raw(constraints=[both])),
             "both inside `geometry` and beside it")
    odd = copy.deepcopy(GRANT_FENCE)
    odd["geometry"]["colour"] = "red"
    _refused(lambda: Policy.model_validate(_raw(constraints=[odd])), "unknown key 'colour'")
    _refused(lambda: Policy.model_validate(_raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "geometry": {"vertices": []},
         "alt_min_m": 5, "alt_max_m": 30}])), "takes no `geometry`")


def test_the_grant_breach_actions_load_and_repair_is_read_as_project_fix():
    """Shown failing on the old code: only repair|brake were accepted. Since
    IR schema 3.0.0 (2026-11-02) the IR holds the grant's six only: `repair`,
    this repository's spelling until then, is read as project_fix at the
    models boundary (failed before: it was stored as written, a seventh
    value), so the escalation FSM never meets it from a policy. Nothing else
    is mapped: a near spelling is refused, never guessed."""
    for act in ("monitor_only", "project_fix", "brake", "loiter", "RTL", "land"):
        r = _raw()
        r["constraints"][0]["violation_action"] = act
        assert Policy.model_validate(r).constraints[0].violation_action == act
    r = _raw()
    r["constraints"][0]["violation_action"] = "repair"
    pol = Policy.model_validate(r)
    assert pol.constraints[0].violation_action == "project_fix"
    assert b"repair" not in pol.canonical_bytes()
    for bad in ("rtl_now", "Repair", "rtl", "project-fix", ""):
        r = _raw()
        r["constraints"][0]["violation_action"] = bad
        _refused(lambda r=r: Policy.model_validate(r), "violation_action")
    assert all(c.violation_action == "project_fix" for c in load_policy(DEMO).constraints)
    # The schema export offers the grant's six and nothing else.
    import typing
    from guardrail.models import ViolationAction
    assert typing.get_args(ViolationAction) == ("monitor_only", "project_fix", "brake",
                                                "loiter", "RTL", "land")


def test_a_ring_hashes_the_same_whatever_order_it_is_written_in():
    """WP1-02: the grant's geometry normalisation names "ring direction".
    Until IR schema 3.0.0 the direction was checked but kept as written, so
    the same zone written clockwise, or from another vertex, had another
    policy_hash (failed then: 8 distinct hashes below). Now every spelling
    stores one canonical ring - positive winding over the stored (north,
    east) / (lat, lon) pairs, smallest pair first - while the order it was
    given is kept outside the IR for the IR 2.x hash forms."""
    from guardrail.models import IR2_FORM
    sq = [{"x": 10, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 15}, {"x": 10, "y": 15}]
    spellings = [sq[k:] + sq[:k] for k in range(4)]
    spellings += [list(reversed(s)) for s in spellings]
    pols = [policy_from_raw(_raw(constraints=[_square("z", 0, 0, 1) | {"vertices": s}]))
            for s in spellings]
    assert len({p.policy_hash for p in pols}) == 1, "the hash depends on vertex order"
    stored = [(v.x, v.y) for v in pols[0].constraints[0].vertices]
    assert stored == [(10.0, 0.0), (20.0, 0.0), (20.0, 15.0), (10.0, 15.0)], stored
    assert all([(v.x, v.y) for v in p.constraints[0].vertices] == stored for p in pols)
    # The order it was given survives outside the IR, so a hash recorded
    # under IR 2.x (which kept it) is still reproduced...
    given = [(v.x, v.y) for v in pols[5].constraints[0].given_vertices()]
    assert given == [(v["x"], v["y"]) for v in spellings[5]]
    assert len({p.ir2_hashes()[IR2_FORM] for p in pols}) == 8
    # ...and the frame a WGS84 ring derives is the same, to the last bit.
    geo = [{"lat": 25.0421, "lon": 121.5310}, {"lat": 25.0428, "lon": 121.5310},
           {"lat": 25.0428, "lon": 121.5318}, {"lat": 25.0421, "lon": 121.5318}]
    orig = {Policy.model_validate({"policy_id": "g", "constraints": [
        {"id": "f", "type": "polygon_fence", **HARD_P0, "vertices": r}]}).frame_origin
        for r in (geo, geo[2:] + geo[:2], list(reversed(geo)))}
    assert len(orig) == 1, orig
    # A dynamic_nfz is normalised too, and still moves about the same centre.
    z = DynamicNFZ.model_validate({"id": "d", "type": "dynamic_nfz", **HARD_P0,
                                   "vertices": list(reversed(sq)),
                                   "motion": {"yaw_rate_dps": 90.0}})
    assert [(v.x, v.y) for v in z.vertices] == stored
    assert sorted(z.ring_at(1.0)) == sorted(DynamicNFZ.model_validate(
        {"id": "d", "type": "dynamic_nfz", **HARD_P0, "vertices": sq,
         "motion": {"yaw_rate_dps": 90.0}}).ring_at(1.0))
    # The grant's worked example and the reference's documents are already in
    # canonical order, so their reference hashes did not move.
    pol = load_policy(_yaml(WORKED_EXAMPLE), runtime=False)
    f = pol.constraints[0]
    assert [(v.lat, v.lon) for v in f.vertices] == \
        [(v.lat, v.lon) for v in f.given_vertices()]
    # Through a bundle the canonical ring comes back as it went in.
    p = _yaml("policy_id: cw\nversion: 0.1.0\nconstraints:\n  - id: z\n"
              "    type: polygon_fence\n    constraint_type: hard\n    scope: mission\n"
              "    priority: P0\n    violation_action: project_fix\n    vertices:\n"
              + "".join(f"      - {{x: {v['x']}, y: {v['y']}}}\n" for v in spellings[6]))
    src = load_policy(p)
    back = B.check_bundle(B.write_bundle(src, _tmp("cw.tar.gz"), signer=None,
                                          allow_legacy_metres=True)).policy
    assert back.policy_hash == src.policy_hash
    assert [(v.x, v.y) for v in back.constraints[0].vertices] == stored


def test_a_bundle_written_under_ir_2_still_verifies():
    """Bundles written before IR schema 3.0.0 spell project_fix `repair` and
    keep rings as written: the policy rebuilt from one has the 3.0.0 hash,
    and its manifest the 2.x one. It is read under the named 2.x form - the
    archived bytes are exactly the 2.x view of the rebuilt policy - and an
    edit to it is still refused."""
    from guardrail.models import IR2_FORM, canonical_json
    import hashlib
    raw = _raw(constraints=[_square("z", 0, 0, 10) | {
        "vertices": [{"x": 0, "y": 10}, {"x": 10, "y": 10}, {"x": 10, "y": 0},
                     {"x": 0, "y": 0}], "violation_action": "repair"}])
    pol = policy_from_raw(raw)
    old = pol._ir2_dump(mode="json", exclude_none=True)
    assert old["constraints"][0]["violation_action"] == "repair"
    ir = canonical_json(old)
    digest = "sha256:" + hashlib.sha256(ir).hexdigest()
    assert pol.hash_form(digest) == IR2_FORM and digest != pol.policy_hash
    man = {"policy_id": "t", "version": "0.1.0", "generation": 0, "policy_hash": digest,
           "hash_scheme": HASH_SCHEME, "ir_schema_version": "2.2.0",
           "hashes": {HASH_SCHEME: digest, REFERENCE_FORM: None},
           "signed_by": B.UNSIGNED, "changelog": "initial", "issued_at": "2026-10-30T00:00:00Z"}
    man_b = json.dumps(man, indent=2, sort_keys=True).encode()

    def bundle(ir_bytes, name):
        out = _tmp(name)
        with tarfile.open(out, "w:gz") as tar:
            for n, data in ((B.IR_NAME, ir_bytes), (B.MANIFEST_NAME, man_b),
                            (B.SIGNATURE_NAME, f"{digest} {B.UNSIGNED}\n".encode())):
                info = tarfile.TarInfo(n)
                info.size = len(data)
                tar.addfile(info, io.BytesIO(data))
        return out
    chk = B.check_bundle(bundle(ir, "ir2.tar.gz"))
    assert chk.hash_form == IR2_FORM, chk.hash_form
    assert chk.policy.policy_hash == pol.policy_hash
    assert chk.policy.constraints[0].violation_action == "project_fix"
    edited = json.loads(ir)
    edited["constraints"][0]["altitude_ceiling_m"] = 5.0
    _refused(lambda: B.check_bundle(bundle(canonical_json(edited), "e.tar.gz")),
             "policy_hash mismatch")


def test_every_rule_must_state_its_type_priority_and_breach_action():
    """WP1-01: the grant's ConstraintBase gives constraint_type, priority and
    violation_action no default. Until IR schema 3.0.0 they defaulted to
    hard / P0 / repair here (failed then: each document below loaded, as the
    strictest rule with a projection repair - a guess its author never
    made; the reference guesses priority P1). Each is refused by name, at
    every DSL entry: a file, a raw document, a bundle IR."""
    for field in ("constraint_type", "priority", "violation_action"):
        r = _raw()
        del r["constraints"][0][field]
        _refused(lambda r=r: policy_from_raw(r), field, "Field required")
        text = (f"policy_id: t\nversion: 0.1.0\nconstraints:\n  - id: alt\n"
                f"    type: altitude_envelope\n    alt_min_m: 5\n    alt_max_m: 30\n"
                + "".join(f"    {k}: {v}\n" for k, v in HARD_P0.items() if k != field))
        _refused(lambda text=text: load_policy(_yaml(text)), field, "Field required")
    pub = json.loads((ROOT / "policies" / "policy_dsl.schema.json").read_text(encoding="utf-8"))
    for model in ("PolygonFence", "AltitudeEnvelope", "Corridor", "IR.PolygonFence"):
        d = pub["$defs"][model]
        req = set(d.get("required", [])) | {x for r in d.get("allOf", [])
                                             for x in r.get("required", [])}
        assert {"constraint_type", "priority", "violation_action"} <= req, (model, req)
    # The negative corpus holds the case too.
    _refused(lambda: load_policy(ROOT / "policies" / "negative" /
                                 "missing_violation_action.yaml"), "violation_action")


def test_every_rule_must_state_its_scope():
    """WP1-01, the fourth field (IR schema 4.0.0, 2026-11-06): the grant's
    ConstraintBase gives `scope` no default either. Until this date it was
    optional here and an absent scope was read as "mission" (failed then:
    every document below that leaves it out loaded, and the published
    schema offered null). It is refused by name at every DSL entry - a
    file, a raw document, a rule built in code, a 4.0.0 recipe - and so is a
    null; the published schema requires it and offers the grant's four
    values only; every shipped rule states it."""
    from guardrail.models import era_context, ir_schema_defaults
    r = _raw()
    del r["constraints"][0]["scope"]
    _refused(lambda: policy_from_raw(r), "scope", "Field required")
    text = ("policy_id: t\nversion: 0.1.0\nconstraints:\n  - id: alt\n"
            "    type: altitude_envelope\n    alt_min_m: 5\n    alt_max_m: 30\n"
            + "".join(f"    {k}: {v}\n" for k, v in HARD_P0.items() if k != "scope"))
    _refused(lambda: load_policy(_yaml(text)), "scope", "Field required")
    nul = _raw()
    nul["constraints"][0]["scope"] = None
    _refused(lambda: policy_from_raw(nul), "scope is required")
    for bad in ("everywhere", "Mission", "", 1):
        x = _raw()
        x["constraints"][0]["scope"] = bad
        _refused(lambda x=x: policy_from_raw(x), "scope")
    for good in ("global", "mission", "segment", "waypoint"):
        x = _raw()
        x["constraints"][0]["scope"] = good
        assert policy_from_raw(x).constraints[0].scope == good
    # A rule built in code, with no document behind it, states it too.
    _refused(lambda: CircleFence(id="c", type="circle_fence", constraint_type="hard",
                                 priority="P0", violation_action="project_fix",
                                 center={"x": 0.0, "y": 0.0}, radius_m=5.0),
             "scope", "Field required")
    # An era says otherwise only for a document of an earlier one: an
    # unreadable or later version is today's.
    for era in ("4.0.0", "4.1.0", "5.0.0", "banana", "3.1"):
        _refused(lambda era=era: Policy.model_validate(r, context=era_context(era)), "scope")
    for era in (None, "2.2.0", "3.0.0", "3.1.0"):
        assert Policy.model_validate(r, context=era_context(era)).constraints[0].scope is None
    # The schema pin and the published contract.
    pins = ir_schema_defaults()
    assert {m for m, f in pins.items() if f.get("scope") == "<required>"} == {
        "PolygonFence", "CircleFence", "AltitudeEnvelope", "KinematicEnvelope",
        "ObstacleClearance", "SubjectStandoff", "Corridor", "DistanceEnvelope",
        "DynamicNFZ", "TimeWindowSwitch", "CorridorSwap"}, pins
    pub = json.loads((ROOT / "policies" / "policy_dsl.schema.json").read_text(encoding="utf-8"))
    for model in ("PolygonFence", "AltitudeEnvelope", "Corridor", "TimeWindowSwitch",
                  "IR.PolygonFence", "IR.DynamicNFZ", "IR.CorridorSwap"):
        d = pub["$defs"][model]
        req = set(d.get("required", [])) | {x for r_ in d.get("allOf", [])
                                             for x in r_.get("required", [])}
        assert "scope" in req, (model, req)
        sc = d["properties"]["scope"]
        assert sc["enum"] == ["global", "mission", "segment", "waypoint"] and "anyOf" not in sc, sc
    _refused(lambda: load_policy(ROOT / "policies" / "negative" / "missing_scope.yaml"),
             "scope", "Field required")
    n = 0
    for f in sorted((ROOT / "policies").glob("*.yaml")):
        pol = load_policy(f, runtime=False)
        assert {c.scope for c in pol.constraints} == {"mission"}, f.name
        assert pol.ir_schema_version == "4.0.0", f.name
        n += len(pol.constraints)
    assert n >= 154, n


def test_a_document_from_before_ir_4_reads_and_hashes_as_stored():
    """IR schema 4.0.0 made `scope` required, and no stored document before it
    wrote scope. Each must still load and re-derive the hash it recorded,
    rewritten nowhere: it is read in its own era (models.era_context), scope
    absent, and `effective_scope` reads that as "mission", what it meant.
    Checked over the stored artefacts in this repository (the bundles of
    every earlier flight live in the lab checkout's gitignored bundles/ and
    are covered by test_every_bundle_on_disk_reads_in_its_era):

      * every tracked replay bundle (bundles/*.replay.tar.gz): the four of
        before 2026-10-06 (16-hex hash) and the KPI evidence of 6 Nov
        (hil_smoke_03, the AIOT-VLA city flights), all written under IR
        schema 3.1.0 or earlier: read and verified as before, every rule's
        scope absent; the same ir.json read as a document of today is
        refused;
      * a run's policy_g0.json (tests/fixtures/pas_ap_20261008_policy_g0.json,
        flown 8 Oct): read by the run readers as written, it resolves to the
        lock's rewound follow-car-citylife@0.1.0;
      * a bundle issued under 3.1.0 for a shipped policy's previous version
        (its content rebuilt from the lock chain, so its ir.json is the bytes
        such a bundle holds): check_bundle reads it in its manifest's era
        with the hash the manifest records; the same bytes under a 4.0.0
        manifest are refused, and a policy of the earlier era re-bundled
        today records 3.1.0, the schema its IR is a document of."""
    import subprocess
    from guardrail.manifest import resolve_run_policy
    from guardrail.replay import POLICY, generation_table, read_replay, verify_replay
    # The tracked ones: bundles/ also holds untracked build products.
    r = subprocess.run(["git", "ls-files", "bundles/*.replay.tar.gz"], cwd=ROOT,
                       capture_output=True, text=True)
    tracked = sorted(ROOT / n for n in r.stdout.split()) if r.returncode == 0 else []
    if not tracked:
        return SKIP                     # not a git checkout
    legacy = {"retarget_demo", "retarget_demo2", "retarget_fixed", "retarget_smooth"}
    assert legacy <= {b.name.split(".")[0] for b in tracked}, tracked
    for b in tracked:
        got = read_replay(b)
        assert got["hash_form"] is not None, b.name
        if b.name.split(".")[0] in legacy:
            assert got["hash_form"].startswith("legacy16-"), (b.name, got["hash_form"])
        assert got["policy"].constraints and \
            all(c.scope is None for c in got["policy"].constraints), b.name
        assert {c.effective_scope for c in got["policy"].constraints} == {"mission"}
        assert got["policy"].ir_schema_version == "3.1.0", b.name
        ok, why = verify_replay(b)
        assert ok, (b.name, why)
        with tarfile.open(b, "r:gz") as tar:
            ir = json.loads(tar.extractfile(POLICY + "ir.json").read())
        _refused(lambda ir=ir: Policy.model_validate(ir), "scope", "Field required")
    fx = ROOT / "tests" / "fixtures" / "pas_ap_20261008_policy_g0.json"
    _, pols, probs = generation_table({"policy_g0.json": fx.read_bytes()})
    assert probs == [] and [c.scope for c in pols[0].constraints] == [None] * 5, probs
    pol, form, label = resolve_run_policy({"policy_hash": pols[0].policy_hash},
                                          B.policy_candidates())
    assert label == "follow_car_citylife.yaml as follow-car-citylife@0.1.0", label
    assert form == HASH_SCHEME and pol.policy_hash == pols[0].policy_hash
    _refused(lambda: Policy.model_validate(json.loads(fx.read_text(encoding="utf-8"))), "scope")
    # A 3.1.0 bundle of follow_car.yaml's previous version (0.1.2).
    lock = B.load_lock()["policies"]
    now = load_policy(ROOT / "policies" / "follow_car.yaml", runtime=False)
    prev_key = lock[B.lock_key(now)]["migrated_from"]
    prev = B.as_version(now.ir3_form(), prev_key.rpartition("@")[2])
    assert prev.policy_hash == lock[prev_key]["policy_hash"], "not the issued 0.1.2"
    assert prev.ir_schema_version == "3.1.0"
    path = B.write_bundle(prev, _tmp("prev.tar.gz"), signer=None,
                          issued_at="2026-11-04T09:32:15Z")
    chk = B.check_bundle(path)
    assert chk.manifest["ir_schema_version"] == "3.1.0", chk.manifest["ir_schema_version"]
    assert chk.hash_form == HASH_SCHEME and chk.manifest["policy_hash"] == lock[prev_key]["policy_hash"]
    assert all(c.scope is None for c in chk.policy.constraints)
    with tarfile.open(path, "r:gz") as tar:
        members = {m.name: tar.extractfile(m).read() for m in tar.getmembers()}
    for era in ("3.0.0", None):
        man = json.loads(members[B.MANIFEST_NAME])
        man.pop("ir_schema_version") if era is None else man.update(ir_schema_version=era)
        assert B.check_bundle(_bundle_with(members, {B.MANIFEST_NAME: man})).hash_form == \
            HASH_SCHEME, era
    man = json.loads(members[B.MANIFEST_NAME])
    man["ir_schema_version"] = "4.0.0"
    _refused(lambda: B.check_bundle(_bundle_with(members, {B.MANIFEST_NAME: man})),
             "cannot represent", "scope")


def _bundle_with(members: dict, replace: dict) -> Path:
    """A copy of a bundle's members with some replaced (a dict is written as
    the manifest's JSON), as a new keyless tar.gz."""
    out = _tmp("edited.tar.gz")
    with tarfile.open(out, "w:gz") as tar:
        for n, data in members.items():
            if n in replace:
                data = json.dumps(replace[n], indent=2, sort_keys=True).encode()
            info = tarfile.TarInfo(n)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return out


def test_every_bundle_on_disk_reads_in_its_era():
    """Every policy bundle in bundles/ (flight bundles are gitignored build
    products: in the lab checkout they go back to 2026-10-06, each written
    under the IR schema its manifest names) still opens: integrity, its
    recorded hash, and every rule's scope as its era wrote it - absent
    before 4.0.0 (read as "mission"), stated since. SKIP where there is no
    bundle but the replay ones (a fresh clone)."""
    from guardrail.models import _semver_tuple
    found = sorted(p for p in (ROOT / "bundles").glob("*.tar.gz")
                   if not p.name.endswith(".replay.tar.gz"))
    if not found:
        return SKIP
    eras = {}
    for p in found:
        chk = B.check_bundle(p)
        era = chk.manifest.get("ir_schema_version")
        eras[p.name] = era
        scopes = {c.scope for c in chk.policy.constraints}
        if era is not None and _semver_tuple(era) >= (4, 0, 0):
            assert None not in scopes, (p.name, scopes)
        assert chk.signature in (B.SIG_VERIFIED, B.SIG_UNSIGNED), (p.name, chk.signature)
        assert {c.effective_scope for c in chk.policy.constraints} <= {
            "global", "mission", "segment", "waypoint"}
    print(f"      {len(found)} bundles: " + ", ".join(
        f"{v or 'unrecorded'} x{sum(1 for e in eras.values() if e == v)}"
        for v in sorted(set(eras.values()), key=lambda v: _semver_tuple(v) if v else ())))


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
        {"id": "poly", "type": "polygon_fence", **HARD_P0, "geometry": {"vertices": sq}},
        {"id": "disc", "type": "circle_fence", **HARD_P0,
         "geometry": {"center": {"lat": 25.0450, "lon": 121.5330}, "radius_m": 20}},
        {"id": "tube", "type": "corridor", **HARD_P0,
         "geometry": {"centerline": line, "width_m": 40, "altitude_floor_m": 10,
                      "altitude_ceiling_m": 60}},
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "altitude_min_m": 5, "altitude_max_m": 120},
        {"id": "kin", "type": "kinematic_envelope", **HARD_P0, "speed_max": 8,
         "climb_rate_max": 3, "turn_rate_max": 60},
        {"id": "people", "type": "distance_envelope", **HARD_P0, "object_class": "people",
         "min_distance_m": 10},
        {"id": "moving", "type": "dynamic_nfz", **HARD_P0, "geometry": {"vertices": sq},
         "motion": {"vx_mps": 1.0, "vy_mps": 0.0, "yaw_rate_dps": 0.0}},
        {"id": "curfew-off", "type": "time_window_switch", **HARD_P0, "target_id": "poly",
         "active": False},
        {"id": "swap", "type": "corridor_swap", **HARD_P0, "target_id": "tube",
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
    the hash the lock pinned before this change.

    Since 2026-11-03 no shipped file is written in metres (WP1-14,
    tools/convert_policies_wgs84.py): each converted file's metre form, at
    the version it had before, is the metre policy, and it hashes exactly as
    the lock pinned it then. Since IR schema 4.0.0 (2026-11-06) the file
    states scope: the WGS84 version it migrated from is the file with scope
    left out (Policy.ir3_form, bundle.IR3_FORM), and its metre form is that
    metre policy."""
    lock = B.load_lock()["policies"]
    n = 0
    for f in sorted((ROOT / "policies").glob("*.yaml")):
        pol = load_policy(f)
        ent = lock[B.lock_key(pol)]
        if ent.get("migration", {}).get("old_hash_form") == B.IR3_FORM:
            prev = ent["migrated_from"]
            pol, ent = B.as_version(pol.ir3_form(), prev.rpartition("@")[2]), lock[prev]
        if not ent.get("migration", {}).get("old_hash_form", "").startswith(B.FRAME_FORM):
            continue
        old_key = ent["migrated_from"]
        metre = B.as_version(pol.metre_form(), old_key.rpartition("@")[2])
        assert not metre.is_geographic and metre.origin is None, f.name
        old = lock[old_key]
        old = old["shared_by_files"][f.name] if "shared_by_files" in old else old
        assert old["policy_hash"] == metre.policy_hash, f.name
        n += 1
    assert n == 31, n


def test_the_geographic_policy_keeps_its_old_hash_as_a_prior_form():
    """wgs84_taipei's v2 hash covered projected metres. It is still its hash
    under the named prior scheme; the lock keeps it and records the v3 form
    beside it, never instead of it. Since IR schema 3.0.0 both belong to
    wgs84-taipei-demo@0.1.0, the file's version before the migration; the
    file, put back at 0.1.0, reproduces both. Since IR schema 4.0.0 the file
    (0.1.2) states scope; it reproduces them with scope left out, as 0.1.1
    and 0.1.0 were written (Policy.ir3_form)."""
    from guardrail.models import IR2_FORM
    pol = load_policy(WGS84)
    lock = B.load_lock()["policies"]
    ent = lock[B.lock_key(pol)]
    assert ent["migration"]["old_hash_form"] == B.IR3_FORM, ent
    assert B.as_version(pol, "0.1.0").prior_hashes()[PRIOR_SCHEME_V2] != \
        lock["wgs84-taipei-demo@0.1.0"]["policy_hash"], "a scope-stating 0.1.0 is not the old one"
    pol = B.as_version(pol.ir3_form(), ent["migrated_from"].rpartition("@")[2])
    old_key = lock[B.lock_key(pol)]["migrated_from"]
    ent = lock[old_key]
    old = B.as_version(pol, old_key.rpartition("@")[2])
    assert old.prior_hashes()[PRIOR_SCHEME_V2] == ent["policy_hash"]
    assert old.hash_form(ent["policy_hash"]) == PRIOR_SCHEME_V2
    assert old.hash_form(ent["hashes"][HASH_SCHEME]) == IR2_FORM
    assert pol.policy_hash not in (ent["policy_hash"], ent["hashes"][HASH_SCHEME])


def test_a_point_in_two_frames_that_disagree_is_refused():
    raw = {"policy_id": "p", "origin": {"lat": 25.0, "lon": 121.0}, "constraints": [
        {"id": "f", "type": "polygon_fence", **HARD_P0, "vertices": [
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
        "finite": _raw(constraints=[{"id": "k", "type": "kinematic_envelope", **HARD_P0,
                                     "speed_max_mps": float("inf"),
                                     "climb_rate_max_mps": 2, "yaw_rate_max_dps": 45}]),
        "should match pattern": _raw(version="banana"),
        "no rules": _raw(constraints=[]),
        "used by 2 rules": _raw(constraints=[OUR_FENCE, {**OUR_FENCE}]),
        "cross": _raw(constraints=[{"id": "b", "type": "polygon_fence", **HARD_P0, "vertices": [
            {"x": 0, "y": 0}, {"x": 10, "y": 10}, {"x": 10, "y": 0}, {"x": 0, "y": 10}]}]),
        "no area": _raw(constraints=[{"id": "c", "type": "polygon_fence", **HARD_P0, "vertices": [
            {"x": 0, "y": 0}, {"x": 5, "y": 0}, {"x": 10, "y": 0}]}]),
        "shrinks the zone": _raw(constraints=[{**OUR_FENCE, "margin_m": -20}]),
        "no height in common": _raw(constraints=[
            {"id": "lo", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 10, "alt_max_m": 20},
            {"id": "hi", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 30, "alt_max_m": 40}]),
    }
    for word, raw in cases.items():
        _refused(lambda raw=raw: policy_from_raw(raw), word)


def test_a_schedule_must_name_a_day_and_write_its_clock_as_hh_mm():
    """`days: []` loaded, hashed and flew a P0 fence that was never in force
    (Shield rule_status in_force False, CSP "(in force no day)"), and the
    clock was range-checked on the total only, so "10:75" meant 11:15 and was
    printed verbatim into the CSP. Shown failing before 2026-10-19: every
    case below loaded."""
    def with_window(rec):
        return _raw(constraints=[{**OUR_FENCE, "valid_time": {"recurrence": rec}}])
    _refused(lambda: policy_from_raw(with_window({"days": []})), "names no day")
    for t in ("10:75", "23:60", "24:00"):
        _refused(lambda t=t: policy_from_raw(with_window(
            {"days": ["Mon"], "start_time": t, "end_time": "23:00"})), "out of range")
    _refused(lambda: policy_from_raw(with_window(
        {"days": ["Mon"], "start_time": "10:00", "end_time": "22:99"})), "out of range")
    for t in ("7:30", " 7:30", "+7:30", "10:-5", "1_0:30", "07:3", "0730",
              "١٠:30"):            # Arabic-Indic digits pass int()
        _refused(lambda t=t: policy_from_raw(with_window(
            {"days": ["Mon"], "start_time": t, "end_time": "23:00"})), "is not HH:MM")
    # The forms the DSL has always meant still load: "24:00" ends the day,
    # a window may wrap past midnight, and no `days` key means every day.
    for rec in ({"days": ["Mon"], "start_time": "22:00", "end_time": "24:00"},
                {"days": ["Sun"], "start_time": "22:00", "end_time": "06:00"},
                {"start_time": "00:00", "end_time": "23:59"}):
        policy_from_raw(with_window(rec))


def test_the_conflict_lint_is_not_over_eager():
    """Disjoint bands are fine when the rules can never be in force together,
    or when one of them is soft. A lint that refused these would push authors
    to delete schedules - a worse policy."""
    day = {"recurrence": {"days": ["Mon"], "start_time": "08:00", "end_time": "17:00"}}
    night = {"recurrence": {"days": ["Mon"], "start_time": "18:00", "end_time": "06:00"}}
    ok = [
        [{"id": "lo", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 10, "alt_max_m": 20,
          "valid_time": day},
         {"id": "hi", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 30, "alt_max_m": 40,
          "valid_time": night}],
        [{"id": "lo", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 10, "alt_max_m": 20},
         {"id": "hi", "type": "altitude_envelope", "scope": "mission",
          "priority": "P0", "violation_action": "project_fix", "alt_min_m": 30, "alt_max_m": 40,
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
    pol = policy_from_raw(_raw(constraints=[{"id": "d", "type": "polygon_fence", **HARD_P0,
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


REG_ALT = {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 120}
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
REG_TUBE = {"id": "tube", "type": "corridor", **HARD_P0, "width_m": 20, "centerline": TUBE_LINE}
# Crosses REG_TUBE at x = 50. Until 2026-11-02 it ran parallel at y = 50,
# disjoint from REG_TUBE: two hard corridors no flight can be inside at once,
# which the lint has refused since that date (_geometry_conflicts).
REG_BAND = {"id": "band-tube", "type": "corridor", **HARD_P0, "width_m": 20,
            "altitude_floor_m": 10, "altitude_ceiling_m": 60,
            "centerline": [{"x": 50, "y": -50}, {"x": 50, "y": 50}]}
REG_KIN = {"id": "kin", "type": "kinematic_envelope", **HARD_P0, "speed_max_mps": 5,
           "climb_rate_max_mps": 2, "yaw_rate_max_dps": 45}
REG_CLEAR = {"id": "clear", "type": "obstacle_clearance", **HARD_P0, "min_clearance_m": 5}
MON_EVE = {"recurrence": {"days": ["Mon"], "start_time": "18:00", "end_time": "23:00"}}
REG_NIGHT = {"id": "night-alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5,
             "alt_max_m": 120, "valid_time": MON_EVE}
REG_DYN = {**_square("moving", 300, 300, 10), "type": "dynamic_nfz",
           "motion": {"vx_mps": 1.0}}
REG_SWITCH = {"id": "sw", "type": "time_window_switch", **HARD_P0, "target_id": "alt", "active": True}


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
        ("changes the rule's type", {"id": "alt", "type": "kinematic_envelope", **HARD_P0,
                                     "speed_max_mps": 5, "climb_rate_max_mps": 2,
                                     "yaw_rate_max_dps": 45}),
        ("turns off", {"id": "lift-alt", "type": "time_window_switch", **HARD_P0,
                       "target_id": "alt", "active": False}),
        # added 2026-10-07
        ("the swap widens the corridor", {"id": "wide-tube", "type": "corridor_swap", **HARD_P0,
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
               {"id": "narrow-tube", "type": "corridor_swap", **HARD_P0, "target_id": "tube",
                "width_m": 10, "centerline": TUBE_LINE}]
    for rule in tighter:
        merge_layers([reg, _layer("mission", rule)])


def test_a_redeclared_lower_rule_stays_locked_to_its_lower_layer():
    """Found 2026-10-18: the switch and swap checks compared an event's layer
    with the WINNING copy's only. A mission layer that re-declared a hard
    regulation rule - identically, or tighter, both of which _not_looser
    passes - made it a mission rule, and a mission switch-off of it, or a
    500 m wide swap of a 20 m regulation corridor, then merged without a
    word. Shown failing on 1b95383: all three merged. The winner now records
    the overridden layer (`locked_by`), which survives the merge and a bundle
    round trip, so the Shield's runtime guard sees it too
    (tests/test_shield_events.py)."""
    reg = _layer("regulation", REG_FENCE, REG_TUBE)
    off = {"id": "lift", "type": "time_window_switch", **HARD_P0, "target_id": "airport",
           "active": False}
    for again in (REG_FENCE, {**REG_FENCE, "margin_m": 3.0}):     # identical, tighter
        _refused(lambda again=again: merge_layers([reg, _layer("mission", again, off)]),
                 "layer merge refused", "turns off 'airport'", "regulation layer")
    wide = {"id": "wide", "type": "corridor_swap", **HARD_P0, "target_id": "tube", "width_m": 500,
            "centerline": TUBE_LINE, "altitude_floor_m": 0, "altitude_ceiling_m": 300}
    _refused(lambda: merge_layers([reg, _layer("mission", REG_TUBE, wide)]),
             "layer merge refused", "widens the corridor", "regulation-layer corridor")
    # The provenance is written where a hard copy was overridden, and only there.
    m = merge_layers([reg, _layer("mission", {**REG_FENCE, "margin_m": 3.0},
                                  _square("crane", 100, 100, 10))])
    by = {c.id: c for c in m.constraints}
    assert (by["airport"].layer, by["airport"].locked_by) == ("mission", ["regulation"])
    assert by["tube"].locked_by is None and by["crane"].locked_by is None
    back = B.check_bundle(B.write_bundle(m, _tmp("locked.tar.gz"), signer=None,
                                          allow_legacy_metres=True)).policy
    assert {c.id: c.locked_by for c in back.constraints} == \
        {c.id: c.locked_by for c in m.constraints}
    assert back.policy_hash == m.policy_hash
    # An unmerged policy is untouched: no field appears, no hash moves.
    assert all(c.locked_by is None for c in load_policy(DEMO).constraints)
    # What stays allowed: a layer may still relax its OWN tightening. A
    # mission swap wider than the mission's 10 m copy but inside regulation's
    # 20 m tube merges; regulation's tube is what it is held to.
    mid = {"id": "mid", "type": "corridor_swap", **HARD_P0, "target_id": "tube", "width_m": 15,
           "centerline": TUBE_LINE}
    merge_layers([reg, _layer("mission", {**REG_TUBE, "width_m": 10}, mid)])
    # A lock carried in from an earlier merge (its lower copy gone) still
    # binds a later one: re-merging the result with a mission switch-off.
    _refused(lambda: merge_layers([m, _layer("mission", off)]), "turns off 'airport'")


def test_a_hand_written_locked_by_is_refused_and_a_merged_one_is_a_listed_extension():
    """`locked_by` is authorable (policy_dsl.schema.json lists it) but it is
    merge provenance. On an unmerged policy a hand-written value moved
    policy_hash and locked the rule against every layer's event, its own
    included, while the Shield's refusal named "a hard regulation-layer copy"
    no file holds; `locked_by: []` moved the hash and did nothing. And
    to_grant_form writes a merged rule's locked_by while
    grant_form_extensions(), which lists what that form keeps beyond the
    grant's, left it out. Shown failing before the 2026-10-19 review fix:
    both hand-written forms passed the DSL entry point, and the extension
    list had no locked_by entry."""
    hand = _raw(constraints=[{**REG_FENCE, "locked_by": ["regulation"]}])
    _refused(lambda: policy_from_raw(hand), "does not pass the DSL lint",
             "airport: locked_by ['regulation']", "no layers_merged")
    bare = {**_raw(constraints=[{**REG_FENCE, "layer": "mission", "locked_by": []}]),
            "layers_merged": ["regulation", "mission"]}
    _refused(lambda: policy_from_raw(bare), "airport: locked_by [] says nothing")
    # What merge_layers writes still passes every entry point, reloads from
    # the grant form, and is listed as a grant-form extension.
    m = merge_layers([_layer("regulation", REG_FENCE),
                      _layer("mission", {**REG_FENCE, "margin_m": 3.0})])
    assert m.lint() == [] and m.constraints[0].locked_by == ["regulation"]
    assert "airport.locked_by" in m.grant_form_extensions(), m.grant_form_extensions()
    g = m.to_grant_form(origin=(25.0, 121.5))
    assert g["constraints"][0]["locked_by"] == ["regulation"], g["constraints"][0]
    back = policy_from_raw(json.loads(json.dumps(g)), runtime=False)
    assert back.constraints[0].locked_by == ["regulation"]
    assert "airport.locked_by" in back.grant_form_extensions()
    # The null: a policy nobody merged has no such field and no such entry.
    assert not any("locked_by" in e for e in load_policy(DEMO).grant_form_extensions())


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
        {"id": "yard", "type": "polygon_fence", **HARD_P0, "layer": "site",
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
                f"{_HP0_YAML}, alt_min_m: 5, alt_max_m: 120}}\n", "reg.yaml")
    mis = _yaml("policy_id: m\nversion: 0.2.0\nconstraints:\n"
                "  - {id: alt, type: altitude_envelope, layer: mission, "
                f"{_HP0_YAML}, alt_min_m: 10, alt_max_m: 50}}\n", "mis.yaml")
    merged = load_layered([reg, mis])
    assert merged.policy_id == "m" and merged.version == "0.2.0"
    out = _tmp("merged.tar.gz")
    assert B._main(["merge", str(reg), str(mis), "-o", str(out), "--unsigned"]) == 0
    assert B.check_bundle(out).policy.policy_hash == merged.policy_hash
    loose = _yaml("policy_id: m\nversion: 0.2.0\nconstraints:\n"
                  "  - {id: alt, type: altitude_envelope, layer: mission, "
                  f"{_HP0_YAML}, alt_min_m: 1, alt_max_m: 50}}\n", "loose.yaml")
    assert B._main(["merge", str(reg), str(loose), "-o", str(out), "--unsigned"]) == 2


# --------------------------------------------------------------------------- #
# the new rule types and what the runtime does with them
# --------------------------------------------------------------------------- #

def test_a_circle_fence_is_enforced_and_stored_as_centre_and_radius():
    pol = policy_from_raw({"policy_id": "c", "constraints": [
        {"id": "disc", "type": "circle_fence", **HARD_P0, "center": {"x": 50, "y": 0},
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
        {"id": "d", "type": "circle_fence", **HARD_P0, "center": {"x": 0, "y": 0}, "radius_m": 5,
         "vertices": [{"x": 0, "y": 0}]}]}), "derived from center")


def test_the_circle_polygon_resolution_is_pinned():
    """SEGMENTS decides the enforced zone of every circle_fence and is in
    neither the hash nor the IR (only centre and radius are). Changing it would
    change enforcement under an unchanged policy_hash, so it is pinned here and
    a change needs an IR_SCHEMA_VERSION bump (models.IR_SCHEMA_VERSION)."""
    assert CircleFence.SEGMENTS == 32
    pol = policy_from_raw({"policy_id": "c", "constraints": [
        {"id": "disc", "type": "circle_fence", **HARD_P0, "center": {"x": 0, "y": 0}, "radius_m": 10}]})
    assert len(pol.constraints[0].vertices) == 32


def test_declarable_only_rules_are_refused_for_flight_and_loaded_as_declarations():
    """A rule the Shield silently ignored would report a clean flight while
    enforcing nothing. So a flight loader refuses it, and says which and why;
    the DSL still validates, hashes and bundles it. Which types are
    declarable-only is RUNTIME_TYPES' business (it shrinks as the runtime
    grows). Since 2026-11-03 (WP1-07) it is empty: distance_envelope, the last
    one, is enforced. The nine still do not fly from a FILE: the dynamic_nfz
    is held (FLIGHT_HELD_TYPES), and that refusal is what is checked here."""
    p = _yaml(json.dumps(_all_nine()), "nine.json")
    msg = _refused(lambda: load_policy(p), "cannot enforce")
    declarable_only = set(GRANT_TYPES) - RUNTIME_TYPES
    assert declarable_only == set(), declarable_only
    assert "dynamic_nfz" in msg and "distance_envelope" not in msg, msg
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
            {"id": "alt", "type": "altitude_envelope", "scope": "mission",
             "priority": "P0", "alt_min_m": 5, "alt_max_m": 60,
             "constraint_type": kind, "violation_action": action}]))
    for act in sorted(SHIELD_ACTS_ON):
        assert env(act).runtime_notes() == [], act
    for act in sorted(MODE_ACTIONS):
        (hard,) = env(act).runtime_notes()
        assert f"requests {act}" in hard and "set_mode" in hard, hard
        (soft,) = env(act, "soft").runtime_notes()
        assert "caps a soft rule's response at brake" in soft, soft
    assert set(SHIELD_ACTS_ON) | set(MODE_ACTIONS) == {"project_fix", "monitor_only",
                                                       "brake", "loiter", "RTL", "land"}
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
        {"id": "m", "type": "dynamic_nfz", **HARD_P0,
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


def _metre_demo() -> Policy:
    """sim_demo_policy.yaml as it was written until 2026-11-03: in metres, no
    origin (the shipped file is WGS84 since; Policy.metre_form)."""
    return load_policy(DEMO).metre_form()


def test_a_metre_policy_needs_an_anchor_for_the_grant_form():
    pol = _metre_demo()
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
    metre = _tmp("metre.json")
    metre.write_text(_metre_demo().canonical_bytes().decode(), encoding="utf-8")
    assert B._main(["export", str(metre)]) == 2, "a metre policy with no anchor"
    # The shipped file states its origin (WGS84 since 2026-11-03): it exports.
    assert B._main(["export", str(DEMO), "-o", str(_tmp("demo_g.yaml"))]) == 0


# --------------------------------------------------------------------------- #
# review of 2026-10-07: every path below was mutated away in a review and no
# test noticed. Each test names what the pre-fix code did.
# --------------------------------------------------------------------------- #

# A rule type neither the flight loaders nor the Shield accept. Until
# 2026-10-07 these tests used a dynamic_nfz, which the Shield of that morning
# ignored; the Shield has enforced dynamic_nfz since (guardrail/shield.py), so
# the case that a loader must not hand the Shield a rule it cannot enforce was
# made with distance_envelope - enforced since 2026-11-03 (WP1-07), and no
# declarable-only type is left. It is now made with an MSL height and no
# ground-elevation source, which the loaders and the Shield still refuse (X-11:
# MSL is an opt-in that names its ground). The rule keeps the id "people" so
# every needle below still names it.
UNENF_RAW = {"policy_id": "dist", "constraints": [
    {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 60},
    {"id": "people", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 600,
     "alt_max_m": 700, "altitude_ref": "MSL"}]}


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
    """RUNTIME_TYPES (what the runtime handles) must stay inside the Shield's
    own ENFORCED_TYPES, and both must refuse MSL. Every type the loaders turn
    away is either refused by the Shield too or enforced by it - the second is
    an over-refusal (safe), printed so it is not forgotten. From 2026-10-07 to
    2026-10-08 the three hot-apply classes were over-refused while they waited
    for compiler templates; since then RUNTIME_TYPES equals ENFORCED_TYPES,
    and the one over-refusal left is FLIGHT_HELD_TYPES (a MOVING dynamic_nfz
    in a policy FILE, since 2026-11-15; until then any dynamic_nfz), held for
    a stated reason: the rails' dwell metric."""
    from guardrail import shield as S
    from guardrail.models import FLIGHT_HELD_TYPES
    assert RUNTIME_TYPES <= S.ENFORCED_TYPES, RUNTIME_TYPES - S.ENFORCED_TYPES
    assert S.ENFORCED_TYPES - RUNTIME_TYPES == set(), \
        f"over-refused again: {sorted(S.ENFORCED_TYPES - RUNTIME_TYPES)}"
    assert set(FLIGHT_HELD_TYPES) <= RUNTIME_TYPES, "a held type must be one the runtime handles"
    assert set(FLIGHT_HELD_TYPES) == {"dynamic_nfz"}, \
        f"the held set changed - update this test and the docs: {sorted(FLIGHT_HELD_TYPES)}"
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
          f"compiler): {sorted(S.ENFORCED_TYPES - RUNTIME_TYPES)}; held from policy "
          f"files until the rails measure them: {sorted(FLIGHT_HELD_TYPES)} (moving only)")


def _strip_manifest_keys(src, dst, *keys):
    """Rewrite a bundle with keys removed from its manifest (unsigned)."""
    with tarfile.open(src, "r:gz") as tar:
        m = {x.name: tar.extractfile(x).read() for x in tar.getmembers()}
    man = json.loads(m["manifest.json"])
    for k in keys:
        man.pop(k, None)
    m["manifest.json"] = json.dumps(man, indent=2, sort_keys=True).encode()
    with tarfile.open(dst, "w:gz") as tar:
        for name, data in m.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return dst


def test_a_reference_form_bundle_is_an_exchange_format_not_a_flight_artefact():
    """Written with ir_form="reference", the IR writes every default out and
    spells `repair` as `project_fix`, so the policy rebuilt from it has a THIRD
    hash - neither the manifest's nor the source's. Shown failing before
    2026-10-07: load_for_flight flew it and recorded that third hash, which
    resolves to no policy. Refused for flight since.

    Since 2026-10-08 the manifest carries both hashes and the
    `reference_delta` that rebuilds the source; check_bundle verifies both,
    and the flight loaders fly the SOURCE policy under the hash the manifest
    records - the third hash never reaches a run. A reference-form bundle
    without the delta (the reference's own; or one with it stripped) is still
    refused, for the old reason."""
    pol = policy_from_raw(_raw(), runtime=True)       # no scope/layer, `repair`
    b = B.write_bundle(pol, _tmp("x.tar.gz"), signer=None, ir_form="reference")
    chk = B.check_bundle(b)
    declared = chk.manifest["policy_hash"]
    assert chk.hash_form == REFERENCE_FORM
    assert len({pol.policy_hash, declared, chk.policy.policy_hash}) == 3, \
        "three hashes: the document rebuilt from the reference form is not the source"
    assert pol.hash_form(declared) == REFERENCE_FORM, "the source still knows it"
    assert chk.hashes == {HASH_SCHEME: pol.policy_hash, REFERENCE_FORM: declared}
    assert chk.source_policy is not None and chk.source_policy.policy_hash == pol.policy_hash
    flown, rec = B.load_for_flight(b, allow_unverified=True)
    assert flown.policy_hash == pol.policy_hash
    assert rec["declared_policy_hash"] == declared
    assert rec["loaded_policy_hash"] == pol.policy_hash, "the run records the source's hash"
    assert B.load_bundle(b, require_signature=False).policy_hash == pol.policy_hash
    assert B.load_bundle(b, require_signature=False, runtime=False).policy_hash == \
        chk.policy.policy_hash, "runtime=False returns the archived document"
    # Without its delta: the old exchange format, refused for flight.
    bare = _strip_manifest_keys(b, _tmp("bare.tar.gz"), "hashes", "reference_delta",
                                "reference_problems")
    assert B.check_bundle(bare).source_policy is None
    _refused(lambda: B.load_for_flight(bare, allow_unverified=True), "exchange format")
    _refused(lambda: B.load_bundle(bare, require_signature=False), "exchange format")
    # A bundle in this project's form carries the same hash in both places.
    own = B.check_bundle(B.write_bundle(pol, _tmp("o.tar.gz"), signer=None))
    rec = own.source_record()
    assert rec["declared_policy_hash"] == rec["loaded_policy_hash"] == pol.policy_hash


def test_every_manifest_names_both_hashes_and_both_are_verified():
    """Deviation 10 (the PI's decision of 6 Oct, conform to the grant): every
    bundle names its policy under this project's canonical form AND the
    reference implementation's, and the loader checks both. Where the
    reference cannot hold the policy, its entry is null with the reasons -
    and the null is checked too, so a forged reference hash on such a policy
    is refused. An edited or dropped entry is refused. No stored hash moved:
    `policy_hash` of a bundle in this project's form is what it was.

    Shown failing on 10451bc: no manifest had `hashes`, and an edited
    reference hash could not be noticed because none was recorded.

    The representable half needs the reference repository's demo document,
    which is gitignored: on a clean clone (CI) that half is left out, the
    metre and reference-form-bundle halves still run, and the test ends in
    SKIP rather than PASS, so a partial run never reads as a full one. Until
    2026-10-19 the unguarded load raised FileNotFoundError there instead."""
    metre = _metre_demo()                                  # not representable
    cases = [(metre, None)]
    if REF_DEMO.is_file():
        ref_pol = load_policy(REF_DEMO, runtime=False)    # representable
        cases.insert(0, (ref_pol, ref_pol.reference_hash()))
    for pol, want_ref in cases:
        b = B.write_bundle(pol, _tmp("h.tar.gz"), signer=None,
                           allow_legacy_metres=want_ref is None)   # the metre case
        chk = B.check_bundle(b)
        assert chk.manifest["policy_hash"] == pol.policy_hash, "our hash did not move"
        assert chk.hashes == {HASH_SCHEME: pol.policy_hash, REFERENCE_FORM: want_ref}
        assert chk.manifest["hashes"] == chk.hashes
        if want_ref is None:
            assert any("metre vertices" in w for w in chk.manifest["reference_problems"])
        else:
            assert want_ref == REF_HASHES["bundles/itri-icl-2026-demo.yaml"]
        # tamper each entry, and drop one
        for edit in ("ours", "ref", "drop"):
            with tarfile.open(b, "r:gz") as tar:
                m = {x.name: tar.extractfile(x).read() for x in tar.getmembers()}
            man = json.loads(m["manifest.json"])
            if edit == "ours":
                man["hashes"][HASH_SCHEME] = "sha256:" + "0" * 64
            elif edit == "ref":
                man["hashes"][REFERENCE_FORM] = "sha256:" + "1" * 64
            else:
                man["hashes"].pop(REFERENCE_FORM)
            m["manifest.json"] = json.dumps(man, indent=2, sort_keys=True).encode()
            bad = _tmp(f"{edit}.tar.gz")
            with tarfile.open(bad, "w:gz") as tar:
                for name, data in m.items():
                    info = tarfile.TarInfo(name)
                    info.size = len(data)
                    tar.addfile(info, io.BytesIO(data))
            _refused(lambda: B.check_bundle(bad), "hashes")
    # A reference-form bundle: its own (reference) hash is the declared one,
    # and this project's entry is checked against the source policy the
    # reference_delta rebuilds. Forging that entry alone passed every test
    # until the review's mutation M18 removed the check; it is refused here.
    src = policy_from_raw(_raw(), runtime=True)
    rb = B.write_bundle(src, _tmp("rf.tar.gz"), signer=None, ir_form="reference")
    assert B.check_bundle(rb).hashes[HASH_SCHEME] == src.policy_hash
    with tarfile.open(rb, "r:gz") as tar:
        m = {x.name: tar.extractfile(x).read() for x in tar.getmembers()}
    man = json.loads(m["manifest.json"])
    man["hashes"][HASH_SCHEME] = "sha256:" + "2" * 64
    m["manifest.json"] = json.dumps(man, indent=2, sort_keys=True).encode()
    forged = _tmp("rf-forged.tar.gz")
    with tarfile.open(forged, "w:gz") as tar:
        for name, data in m.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    _refused(lambda: B.check_bundle(forged), f"hashes[{HASH_SCHEME}]",
             "rebuilt from the reference document")
    _refused(lambda: B.load_for_flight(forged, allow_unverified=True), f"hashes[{HASH_SCHEME}]")
    # The lock still pins every shipped policy under the hash it always had.
    assert B.check_lock() == [], B.check_lock()
    if not REF_DEMO.is_file():
        print(f"      (the representable half needs {REF_DEMO.relative_to(ROOT)})")
        return SKIP


def test_a_reference_form_bundle_is_verified_by_both_loaders():
    """The grant's demo document, written here in the reference's form: the
    PI's own loader (its 3.11 env, read-only) reads it and reproduces its
    `policy_hash`; this loader verifies that hash and this project's hash of
    the source it rebuilds. Without the reference env the local half runs and
    the cross half says SKIP."""
    if not REF_DEMO.is_file():
        return SKIP
    pol = load_policy(REF_DEMO, runtime=False)
    out = B.write_bundle(pol, _tmp("both.tar.gz"), signer=None, ir_form="reference")
    chk = B.check_bundle(out)
    assert chk.hashes == {HASH_SCHEME: pol.policy_hash, REFERENCE_FORM: pol.reference_hash()}
    assert chk.source_policy.policy_hash == pol.policy_hash
    if not _reference_available():
        return SKIP
    import wp1_roundtrip_kpi as W
    got = W.run_reference(
        "import json; from policy_dsl import load_bundle; "
        f"print(json.dumps({{'h': load_bundle({str(out)!r}).policy_hash}}))")
    assert got.get("ok"), got
    assert got["h"] == chk.hashes[REFERENCE_FORM], got


def test_a_dynamic_zone_in_a_policy_file_is_held_until_the_rails_measure_it():
    """dynamic_nfz joined RUNTIME_TYPES on 2026-10-08 (the Shield enforces it,
    the compiler renders it), but a policy FILE declaring one was refused for
    flight (models.FLIGHT_HELD_TYPES): every flight rail but follow_vlm
    computed its zone-dwell metric nfz_s - which guardrail/kpi.py reads for
    the mission outcome - from policy.by_type(PolygonFence), so the zone
    would score 0 s however long the aircraft spent in it. Found in review
    the day it joined; the refusal names the zone and that reason, by every
    flight loader (YAML and bundle). The declaration still loads, bundles and
    records a start conflict (until 2026-10-08 start_conflicts looked at
    polygon fences only). A policy file with a switch and a swap flies.

    Since 2026-11-15 (C-16) the hold is on a MOVING zone only: every rail
    counts a zone without motion (zone_dwell) and every offline reader takes
    it with the polygon fences (Policy.static_keepout_zones), so the same
    zone declared without motion flies, while one with motion is refused as
    before - no rail records the Shield's clock to place it per point."""
    still = _raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 30},
        {"id": "dz", "type": "dynamic_nfz", **HARD_P0,
         "vertices": [{"x": -5, "y": -5}, {"x": 5, "y": -5}, {"x": 5, "y": 5}]}])
    ps = _yaml(json.dumps(still), "dz_still.json")
    flies = load_policy(ps)
    assert flies.flight_problems() == [] and [c.id for c in flies.static_keepout_zones()] == ["dz"]
    assert B.load_for_flight(None, ps)[0].policy_hash == flies.policy_hash
    raw = _raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 30},
        {"id": "dz", "type": "dynamic_nfz", **HARD_P0,
         "vertices": [{"x": -5, "y": -5}, {"x": 5, "y": -5}, {"x": 5, "y": 5}],
         "motion": {"vx_mps": 1.0}}])
    p = _yaml(json.dumps(raw), "dz.json")
    _refused(lambda: load_policy(p), "dz: dynamic_nfz is enforced by the Shield", "nfz_s",
             "moves")
    _refused(lambda: B.load_for_flight(None, p), "nfz_s")
    pol = load_policy(p, runtime=False)
    assert [g.split(":")[0] for g in pol.flight_problems()] == ["dz"]
    assert pol.unenforced_rules() == [], "the Shield enforces it; the gap is the rails'"
    assert any(n.startswith("dz: the frame origin") for n in pol.runtime_notes()), \
        pol.runtime_notes()
    assert Shield(pol).state_is_unsafe(State(x=1.0, y=-3.0, up=10.0))
    b = B.write_bundle(pol, _tmp("dz.tar.gz"), signer=None,
                       allow_legacy_metres=True)          # a metre fixture
    _refused(lambda: B.load_for_flight(b, allow_unverified=True), "nfz_s")
    _refused(lambda: B.load_bundle(b, require_signature=False), "nfz_s")
    assert B.load_bundle(b, require_signature=False, runtime=False).policy_hash == pol.policy_hash
    # Switches and swaps are measured as before (the Shield applies them; no
    # rail scores corridor dwell), so a file declaring them flies.
    raw = _raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 30},
        _square("nfz", 40, 40, 10),
        {"id": "off", "type": "time_window_switch", **HARD_P0, "target_id": "nfz", "active": False,
         "valid_time": {"recurrence": {"days": ["Sat"], "start_time": "08:00",
                                       "end_time": "09:00"}}},
        {"id": "lane", "type": "corridor", **HARD_P0, "width_m": 40,
         "centerline": [{"x": 0, "y": 0}, {"x": 100, "y": 0}]},
        {"id": "swap", "type": "corridor_swap", **HARD_P0, "target_id": "lane", "width_m": 30,
         "centerline": [{"x": 0, "y": 0}, {"x": 100, "y": 10}],
         "valid_time": {"recurrence": {"days": ["Sun"], "start_time": "08:00",
                                       "end_time": "09:00"}}}])
    p2 = _yaml(json.dumps(raw), "sw.json")
    flies = load_policy(p2)
    assert flies.flight_problems() == []
    flown, _ = B.load_for_flight(B.write_bundle(flies, _tmp("sw.tar.gz"), signer=None,
                                                allow_legacy_metres=True),
                                 allow_unverified=True)
    assert flown.policy_hash == flies.policy_hash


def test_every_reader_of_keep_out_zones_takes_a_dynamic_zone_without_motion():
    """C-16, lifted for a zone without motion (2026-11-15): the readers the
    hold named - experiments/analyze_semantic_ab.py's legal-approach bound,
    dwell and overlay, training/features.make_fences (the BC pilots'
    features) and the prompt's no_fly_zones - take the polygon and circle
    fences AND every dynamic_nfz without motion (Policy.static_keepout_zones),
    never a moving one. Shown failing on 478dcc7: they read
    by_type(PolygonFence), the static zone was not there (make_fences [],
    no_fly_zones without it, the approach bound 0.0 m because load_policy
    refused the file)."""
    from guardrail.compiler import ConstraintCompiler
    from guardrail.models import DynamicNFZ
    raw = _raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 30},
        _square("poly", 40, 40, 10),
        {"id": "disc", "type": "circle_fence", **HARD_P0, "center": {"x": -40, "y": 0},
         "radius_m": 5},
        {"id": "still", "type": "dynamic_nfz", **HARD_P0, "margin_m": 1.0,
         "vertices": [{"x": 15, "y": -5}, {"x": 25, "y": -5}, {"x": 25, "y": 5},
                      {"x": 15, "y": 5}]},
        {"id": "zero", "type": "dynamic_nfz", **HARD_P0,
         "vertices": [{"x": 60, "y": -5}, {"x": 70, "y": -5}, {"x": 70, "y": 5}],
         "motion": {"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rate_dps": 0.0}}])
    p = _yaml(json.dumps(raw), "still.json")
    pol = load_policy(p)                              # flies: nothing held
    assert [c.id for c in pol.static_keepout_zones()] == ["poly", "disc", "still", "zero"]
    moving = DynamicNFZ.model_validate({**raw["constraints"][3], "id": "mov",
                                        "motion": {"vx_mps": 1.0}})
    assert moving.moves and not pol.constraints[3].moves and not pol.constraints[4].moves
    assert "mov" not in [c.id for c in pol.model_copy(update={
        "constraints": pol.constraints + [moving]}).static_keepout_zones()]
    # The BC pilots' features.
    sys.path.insert(0, str(ROOT))
    from training.features import encode, make_fences
    fences = make_fences(pol)
    assert [f.id for f, _ in fences] == ["poly", "disc", "still", "zero"]
    assert encode(State(x=20.0, y=0.0, up=10.0), 100.0, 0.0, 10.0, fences)[6] == 1.0, \
        "inside the static dynamic_nfz is 'inside an NFZ'"
    # The prompt's shorthand.
    from guardrail.compiler import Mission
    mission = Mission(task_text="fly to the pad", target_x=100.0, target_y=0.0,
                      cruise_alt_m=10.0, speed_pref_mps=3.0)
    doc = __import__("yaml").safe_load(ConstraintCompiler(pol).build_prompt(mission))
    assert [z["id"] for z in doc["constraints"]["no_fly_zones"]] == \
        ["poly", "disc", "still", "zero"], doc["constraints"]["no_fly_zones"]
    # Offline analysis: the approach bound for a target inside the static zone.
    sys.path.insert(0, str(ROOT / "experiments"))
    import analyze_semantic_ab as S
    got = S.legal_approach([{"x": 0.0, "y": 0.0}, {"x": 8.0, "y": 0.0}], 20.0, 0.0, p,
                           ROOT / "no_map.npz")
    assert got["d_bound"] >= 5.0, got
    src = (ROOT / "experiments" / "analyze_semantic_ab.py").read_text(encoding="utf-8")
    assert "by_type(PolygonFence)" not in src and src.count("static_keepout_zones()") == 3


def test_the_lint_compares_every_pair_of_hard_height_limits():
    """A hard envelope against a hard corridor band; two hard corridors (the
    Shield requires the vehicle inside every active corridor at once - until
    2026-10-07 corridor pairs were skipped as "different places"); and AGL
    against MSL, which cannot be compared and is not."""
    env_vs_tube = _raw(constraints=[
        {"id": "env", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 100, "alt_max_m": 120},
        {"id": "tube", "type": "corridor", **HARD_P0, "width_m": 20, "altitude_floor_m": 30,
         "altitude_ceiling_m": 80, "centerline": TUBE_LINE}])
    _refused(lambda: policy_from_raw(env_vs_tube), "env", "tube", "no height in common")
    two_tubes = _raw(constraints=[
        {"id": "low", "type": "corridor", **HARD_P0, "width_m": 20, "altitude_floor_m": 10,
         "altitude_ceiling_m": 20, "centerline": TUBE_LINE},
        {"id": "high", "type": "corridor", **HARD_P0, "width_m": 20, "altitude_floor_m": 30,
         "altitude_ceiling_m": 40, "centerline": TUBE_LINE}])
    _refused(lambda: policy_from_raw(two_tubes), "low", "high", "no height in common")
    overlapping = copy.deepcopy(two_tubes)
    overlapping["constraints"][1]["altitude_floor_m"] = 15
    policy_from_raw(overlapping)
    agl_msl = _raw(constraints=[
        {"id": "agl", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 10, "alt_max_m": 20},
        {"id": "msl", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 30, "alt_max_m": 40,
         "altitude_ref": "MSL"}])
    policy_from_raw(agl_msl, runtime=False)          # not compared: not a conflict


def test_the_lint_refuses_a_corridor_no_flight_can_be_inside():
    """WP1-02 (2026-11-02): the conflict lint covered altitude bands only, so
    a hard corridor inside a hard no-fly zone, and two hard corridors with no
    point in common, loaded clean (failed then: both policies below loaded).
    Each refusal is paired with the near miss that must still load, so the
    check is about an EMPTY allowed set, not about overlap at all."""
    tube = {"id": "tube", "type": "corridor", **HARD_P0, "width_m": 20, "altitude_floor_m": 10,
            "altitude_ceiling_m": 20, "centerline": TUBE_LINE}       # y in [-10, 10]
    nfz = _square("nfz", -20, -20, 140, altitude_ceiling_m=100, margin_m=0.0)
    buried = _raw(constraints=[tube, nfz])
    _refused(lambda: policy_from_raw(buried), "tube", "nfz", "lies wholly inside")
    loads = {
        "a soft zone": [tube, {**nfz, "constraint_type": "soft"}],
        "a zone band that leaves heights free": [tube, {**nfz, "altitude_floor_m": 15}],
        "a zone that leaves part of the tube free": [tube, _square("nfz", -20, -20, 110)],
        "schedules that never meet": [
            {**tube, "valid_time": {"recurrence": {"days": ["Mon"]}}},
            {**nfz, "valid_time": {"recurrence": {"days": ["Tue"]}}}],
        "a zone a switch may turn off": [tube, nfz, {
            "id": "sw", "type": "time_window_switch", **HARD_P0, "target_id": "nfz",
            "active": False, "valid_time": {"recurrence": {"days": ["Mon"]}}}],
        "a moving zone": [tube, {**nfz, "type": "dynamic_nfz", "motion": {"vx_mps": 1.0}}],
    }
    for name, cons in loads.items():
        try:
            policy_from_raw(_raw(constraints=cons), runtime=False)
        except ValueError as e:
            raise AssertionError(f"{name}: refused - {str(e)[:300]}") from None
    # The margin ring counts (the Shield forbids polygon + margin)...
    _refused(lambda: policy_from_raw(_raw(constraints=[
        tube, _square("nfz", -9, -9, 118, margin_m=2.0)])), "lies wholly inside")
    # ...a tube that only touches the zone's boundary keeps its touching
    # points, so it is not reported...
    policy_from_raw(_raw(constraints=[tube, {
        "id": "edge", "type": "polygon_fence", **HARD_P0, "margin_m": 0.0, "vertices": [
            {"x": -10, "y": -10}, {"x": 110, "y": -10}, {"x": 110, "y": 10},
            {"x": -10, "y": 10}]}]), runtime=False)
    # ...and two always-on zones that cover the tube together are a conflict.
    halves = [tube, _square("west", -20, -20, 70, margin_m=0.0),
              _square("east", 45, -20, 80, margin_m=0.0)]
    _refused(lambda: policy_from_raw(_raw(constraints=halves)), "west", "east",
             "lies wholly inside")
    # Two hard corridors with no point in common; crossing ones load.
    far = {"id": "far", "type": "corridor", **HARD_P0, "width_m": 20,
           "centerline": [{"x": 0, "y": 100}, {"x": 100, "y": 100}]}
    _refused(lambda: policy_from_raw(_raw(constraints=[tube, far])), "tube", "far",
             "no point in common")
    policy_from_raw(_raw(constraints=[tube, {**far, "centerline": [
        {"x": 50, "y": -50}, {"x": 50, "y": 100}]}]))
    policy_from_raw(_raw(constraints=[tube, {**far, "constraint_type": "soft"}]))
    # The negative corpus holds one file of each.
    for name in ("corridor_inside_nfz.yaml", "disjoint_hard_corridors.yaml"):
        _refused(lambda name=name: load_policy(ROOT / "policies" / "negative" / name),
                 "no flight can satisfy")


def test_the_lint_checks_what_a_switch_or_swap_points_at():
    swap_env = _raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 30},
        {"id": "s", "type": "corridor_swap", **HARD_P0, "target_id": "alt", "width_m": 20,
         "centerline": TUBE_LINE}])
    _refused(lambda: policy_from_raw(swap_env, runtime=False), "not a corridor")
    self_target = _raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 30},
        {"id": "sw", "type": "time_window_switch", **HARD_P0, "target_id": "sw", "active": False}])
    _refused(lambda: policy_from_raw(self_target, runtime=False), "cannot target itself")
    at_event = _raw(constraints=[
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 5, "alt_max_m": 30},
        {"id": "a", "type": "time_window_switch", **HARD_P0, "target_id": "alt", "active": True},
        {"id": "b", "type": "time_window_switch", **HARD_P0, "target_id": "a", "active": False}])
    _refused(lambda: policy_from_raw(at_event, runtime=False), "not an event")


def test_check_bundle_runs_the_lint():
    """A bundle is a release of a DSL-valid policy. An IR with two rules of
    one id hashes correctly (write_bundle does not lint) and must still be
    refused on read."""
    dup = Policy.model_validate(_raw(constraints=[OUR_FENCE, {**OUR_FENCE}]))
    b = B.write_bundle(dup, _tmp("dup.tar.gz"), signer=None, allow_legacy_metres=True)
    _refused(lambda: B.check_bundle(b), "does not pass the DSL lint", "used by 2 rules")


def test_load_layered_applies_the_flight_gate():
    reg = _yaml("policy_id: reg\nconstraints:\n  - {id: alt, type: altitude_envelope, "
                f"layer: regulation, {_HP0_YAML}, alt_min_m: 5, alt_max_m: 120}}\n",
                "reg.yaml")
    # An MSL height with no ground (UNENF_RAW says why: no declarable-only type
    # is left since 2026-11-03).
    mis = _yaml("policy_id: m\nconstraints:\n  - {id: people, type: altitude_envelope, "
                f"layer: mission, {_HP0_YAML}, alt_min_m: 600, alt_max_m: 700, "
                "altitude_ref: MSL}\n", "mis.yaml")
    _refused(lambda: load_layered([reg, mis]), "cannot enforce", "people")
    assert len(load_layered([reg, mis], runtime=False).constraints) == 2


def test_a_grant_form_corridor_gets_the_references_band():
    """A corridor's geometry block, like a fence's, takes the reference's
    0-200 m band; the flat form keeps this project's 0-1000 m."""
    g = policy_from_raw(_raw(constraints=[{"id": "t", "type": "corridor", **HARD_P0, "geometry": {
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
    tube = {"id": "tube", "type": "corridor", **HARD_P0, "geometry": {"centerline": line, "width_m": 80}}
    fence = {"id": "yard", "type": "polygon_fence", **HARD_P0, "geometry": {"vertices": sq}}
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
