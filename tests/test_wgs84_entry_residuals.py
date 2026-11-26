"""
WP1-14: the four residuals of the WGS84-only entry (review 2026-11-26).

    python tests/test_wgs84_entry_residuals.py      (no pytest needed)

The grant (Policy DSL page): "WGS84 latitude / longitude is canonical ... the
DSL itself never carries projected coordinates". On 2026-11-26 (909804b,
305ffdf, fa4287e) every entry was made to refuse a point given in local
metres; an independent review found four places where projected coordinates
still got in. Each is closed here, and valid lat/lon input is unchanged:

  1. the published JSON Schema (policies/policy_dsl.schema.json) offers a
     point as WGS84 {lat, lon} only - no {x, y} form, no x/y beside lat/lon -
     in the authoring root and in $defs/CanonicalIR alike;
  2. a DOCUMENT point that gives lat/lon with an x or y key beside it is
     refused by policy_from_raw / load_policy / load_layered
     (ProjectedCoordinatesRefused, naming models.wgs84_from_metres), as is
     an in-process Shield event carrying one (HotApplyRefused); the legacy
     metre read (legacy_metre_policy_from_raw) still reads it, and a rule
     converted tool side (wgs84_from_metres) still goes in;
  3. a REST body in the grant's nested form, geometry.vertices /
     geometry.centerline, goes through the same point check as the top-level
     form (422), on POST /dynamic_nfz, /corridor_swap and /hot_apply, and so
     on the gRPC HotApply, which runs api.apply_event; a nested lat/lon body
     is applied exactly as the top-level one;
  4. a Shield is not built on a policy that stores local metres
     (Policy.point_form a legacy form) unless the caller declares the
     legacy read (Shield(..., legacy_metres=True), or a policy that came
     from models.load_legacy_metre_policy / legacy_metre_policy_from_raw).

The REST checks call the guardrail.api handlers the routes call; the FastAPI
routes are exercised by tests/test_api.py where FastAPI is installed.
"""
from __future__ import annotations

import copy
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from guardrail import api                                          # noqa: E402
from guardrail import bundle as B                                  # noqa: E402
from guardrail.models import (TOOL_SIDE_CONVERSION,                # noqa: E402
                              CorridorSwap, DynamicNFZ, PolygonFence,
                              ProjectedCoordinatesRefused, legacy_metre_policy_from_raw,
                              load_layered, load_policy, point_gives_metres,
                              policy_from_raw, refuse_projected_policy,
                              wgs84_from_metres)
from guardrail.projection import metres_to_wgs84, xy_beside_latlon_count  # noqa: E402
from guardrail.shield import HotApplyRefused, Shield               # noqa: E402

SITL = ROOT / "policies" / "sim_demo_policy.yaml"      # CMAC origin, SITL rails
SURVEY = ROOT / "policies" / "corridor_survey.yaml"    # a corridor, PAS origin

RULE = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
        "violation_action": "project_fix"}


def _refused(fn, exc=ProjectedCoordinatesRefused, *, names_conversion=True) -> str:
    try:
        fn()
    except exc as e:
        msg = str(e)
        if names_conversion:
            assert TOOL_SIDE_CONVERSION in msg, msg
        return msg
    raise AssertionError(f"{getattr(fn, '__name__', fn)} was not refused")


def _ll(sh: Shield, x: float, y: float) -> dict:
    lat, lon = metres_to_wgs84(sh.policy.frame_origin, x, y)
    return {"lat": lat, "lon": lon}


def _first_rule_point(raw: dict) -> dict:
    """The first vertex / centre / centerline point of a document's rules."""
    for c in raw["constraints"]:
        for key in ("vertices", "centerline"):
            if c.get(key):
                return c[key][0]
        if c.get("center"):
            return c["center"]
    raise AssertionError("no rule with points")


# --------------------------------------------------------------------------- #
# 1. the published schema
# --------------------------------------------------------------------------- #

def test_schema_point_is_lat_lon_only():
    """The published contract offers WGS84 {lat, lon} only, in both forms."""
    schema = B.policy_json_schema()
    for name in ("XY", B.IR_DEF_PREFIX + "XY"):
        xy = schema["$defs"][name]
        assert set(xy["properties"]) == {"lat", "lon"}, (name, sorted(xy["properties"]))
        assert xy["required"] == ["lat", "lon"], name
        assert "anyOf" not in xy and xy["additionalProperties"] is False, name
        for k, lim in (("lat", 90.0), ("lon", 180.0)):
            p = xy["properties"][k]
            assert p["type"] == "number" and p["maximum"] == lim and p["minimum"] == -lim, p
        assert "{x, y} metres" not in xy["description"], xy["description"]
    assert "metre point" not in schema["$defs"]["CanonicalIR"]["description"]
    # Nowhere in the published file is a property named x or y.
    def walk(n):
        if isinstance(n, dict):
            props = n.get("properties")
            if isinstance(props, dict):
                assert not {"x", "y"} & set(props), sorted(props)
            for v in n.values():
                walk(v)
        elif isinstance(n, list):
            for v in n:
                walk(v)
    walk(schema)


def test_schema_file_is_regenerated():
    """The committed file is what the generator writes (as
    tests/test_policy_hash.py checks it), and the generator's CLI writes it
    byte for byte."""
    have = json.loads(B.SCHEMA_PATH.read_text(encoding="utf-8"))
    assert have == B.policy_json_schema(), (
        "policy_dsl.schema.json is stale - run python -m guardrail.bundle schema")
    with tempfile.TemporaryDirectory() as tmp:
        out = B.write_schema(Path(tmp) / "policy_dsl.schema.json")
        assert out.read_bytes() == B.SCHEMA_PATH.read_bytes()


def test_schema_validates_lat_lon_and_refuses_metres():
    try:
        import jsonschema
    except ImportError:
        print("      (jsonschema not installed: the schema's verdicts were not run)")
        return
    schema = json.loads(B.SCHEMA_PATH.read_text(encoding="utf-8"))
    root = jsonschema.Draft202012Validator(schema)
    canon = jsonschema.Draft202012Validator({**schema, "$ref": "#/$defs/CanonicalIR"})
    ir = load_policy(SITL).canonical_ir()
    assert not list(root.iter_errors(ir)) and not list(canon.iter_errors(ir))
    p = load_policy(SITL).projection()
    for name, mutate in (
            ("x/y beside lat/lon", lambda pt: pt.update(dict(zip("xy", p.to_local(pt["lat"], pt["lon"]))))),
            ("an x key alone beside lat/lon", lambda pt: pt.update(x=None)),
            ("{x, y} metres", lambda pt: (pt.clear(), pt.update(x=1.0, y=2.0)))):
        bad = copy.deepcopy(ir)
        mutate(_first_rule_point(bad))
        for label, v in (("authoring root", root), ("CanonicalIR", canon)):
            assert list(v.iter_errors(bad)), f"the {label} accepted {name}"


# --------------------------------------------------------------------------- #
# 2. a document point with x/y beside lat/lon; the in-process Shield entry
# --------------------------------------------------------------------------- #

def _doc_with_xy_beside(path=SITL, *, value_none: bool = False) -> tuple[dict, dict]:
    pol = load_policy(path)
    raw = pol.canonical_ir()
    bad = copy.deepcopy(raw)
    pt = _first_rule_point(bad)
    if value_none:
        pt["x"] = None
    else:
        # The point's own metres in its frame: consistent to the micrometre,
        # so until 2026-11-26 they were checked and silently dropped.
        pt["x"], pt["y"] = pol.projection().to_local(pt["lat"], pt["lon"])
    return raw, bad


def test_policy_from_raw_refuses_xy_beside_lat_lon():
    raw, bad = _doc_with_xy_beside()
    assert xy_beside_latlon_count(bad) == 1 and xy_beside_latlon_count(raw) == 0
    msg = _refused(lambda: policy_from_raw(bad, source="probe"))
    assert "1 point(s) carry x/y beside lat/lon" in msg, msg
    # An x key with no value is refused too, as REST refuses it (api._points).
    _, bad_none = _doc_with_xy_beside(value_none=True)
    _refused(lambda: policy_from_raw(bad_none))
    # Unchanged for lat/lon input: same policy, same hash.
    assert policy_from_raw(raw).policy_hash == load_policy(SITL).policy_hash
    # The declared legacy read still reads a recorded document carrying
    # them (checked against the frame and dropped, as before).
    assert legacy_metre_policy_from_raw(bad).policy_hash == load_policy(SITL).policy_hash


def test_load_policy_and_load_layered_refuse_xy_beside_lat_lon():
    raw, bad = _doc_with_xy_beside()
    with tempfile.TemporaryDirectory() as tmp:
        f = Path(tmp) / "xy_beside.json"
        f.write_text(json.dumps(bad), encoding="utf-8")
        ok = Path(tmp) / "clean.json"
        ok.write_text(json.dumps(raw), encoding="utf-8")
        _refused(lambda: load_policy(f))
        _refused(lambda: load_layered([f], runtime=False))
        assert load_policy(ok).policy_hash == load_policy(SITL).policy_hash
        assert load_layered([ok], runtime=False).frame_origin == load_policy(SITL).frame_origin


def test_policy_built_from_point_models_is_checked_too():
    """refuse_projected_policy also reads the model's own points (a Policy
    built from point models keeps what its author wrote: model_fields_set)."""
    pol = load_policy(SITL)
    lat, lon = pol.frame_origin
    fence = PolygonFence.model_validate({
        "id": "f-xy", "type": "polygon_fence", **RULE,
        "vertices": [{"lat": lat, "lon": lon, "x": 0.0, "y": 0.0},
                     {"lat": lat + 1e-4, "lon": lon}, {"lat": lat + 1e-4, "lon": lon + 1e-4}]})
    assert point_gives_metres(fence.vertices[0])
    assert not point_gives_metres(fence.vertices[1])
    doc = pol.model_copy(update={"constraints": list(pol.constraints) + [fence]})
    _refused(lambda: refuse_projected_policy(doc, "probe"))
    refuse_projected_policy(pol, "probe")                 # lat/lon only: no refusal
    # A loaded point's lazily computed view never counts as given metres.
    assert not any(point_gives_metres(v) for c in pol.constraints
                   for v in (getattr(c, "vertices", None) or getattr(c, "centerline", None) or []))


def _zone(sh: Shield, zid: str, *, xy_beside: bool) -> DynamicNFZ:
    verts = [_ll(sh, x, y) for x, y in ((23, 6), (31, 6), (31, 14))]
    if xy_beside:
        verts[0] = {**verts[0], "x": 23.0, "y": 6.0}
    return DynamicNFZ.model_validate({"id": zid, "type": "dynamic_nfz", **RULE,
                                      "vertices": verts, "altitude_floor_m": 0,
                                      "altitude_ceiling_m": 100, "margin_m": 1.0})


def test_shield_events_refuse_xy_beside_lat_lon():
    sh = Shield(load_policy(SITL))
    gen = sh.policy.generation
    msg = _refused(lambda: sh.spawn_nfz(_zone(sh, "z-xy", xy_beside=True)), HotApplyRefused)
    assert "carry x/y beside lat/lon" in msg, msg
    _refused(lambda: sh.hot_apply(_zone(sh, "z-xy", xy_beside=True)), HotApplyRefused)
    fence = PolygonFence.model_validate({
        "id": "pre-xy", "type": "polygon_fence", **RULE,
        "vertices": [{**_ll(sh, -30, -30), "x": -30.0, "y": -30.0},
                     _ll(sh, -20, -30), _ll(sh, -20, -21)]})
    _refused(lambda: sh.hot_apply(fence), HotApplyRefused)
    # move: any x/y key on a given vertex, a null one included
    sh.spawn_nfz(_zone(sh, "z-ok", xy_beside=False))
    ring = [_ll(sh, x, y) for x, y in ((40, 6), (48, 6), (48, 14))]
    _refused(lambda: sh.move_nfz("z-ok", [{**ring[0], "x": None}] + ring[1:]), HotApplyRefused)
    assert sh.policy.generation == gen + 1                # only the clean spawn
    # swap
    sv = Shield(load_policy(SURVEY))
    swap = {"id": "swap-xy", "type": "corridor_swap", "target_id": "corridor-survey-route",
            **RULE, "width_m": 30, "altitude_floor_m": 10, "altitude_ceiling_m": 60,
            "centerline": [{**_ll(sv, 0, 0), "x": 0.0, "y": 0.0}, _ll(sv, 60, 0),
                           _ll(sv, 60, 75.5)]}
    _refused(lambda: sv.hot_apply(CorridorSwap.model_validate(swap)), HotApplyRefused)


def test_tool_side_conversion_still_goes_in():
    """wgs84_from_metres writes lat/lon only (no x/y in what it hands on), so
    the Shield takes its rule; the stored zone is the one the 26 Nov
    conversion stored."""
    sh = Shield(load_policy(SITL))
    metre = DynamicNFZ.model_validate({"id": "z-m", "type": "dynamic_nfz", **RULE,
                                       "vertices": [{"x": 23, "y": 6}, {"x": 31, "y": 6},
                                                    {"x": 31, "y": 14}],
                                       "altitude_floor_m": 0, "altitude_ceiling_m": 100})
    conv = wgs84_from_metres(metre, sh.policy.frame_origin)
    assert all(v.model_fields_set == {"lat", "lon"} for v in conv.vertices)
    assert [(v.x, v.y) for v in conv.vertices] == [(23.0, 6.0), (31.0, 6.0), (31.0, 14.0)]
    sh.spawn_nfz(conv)
    stored = sh.policy.constraints[-1]
    assert [(v.lat, v.lon) for v in stored.vertices] == \
        [tuple(metres_to_wgs84(sh.policy.frame_origin, x, y)) for x, y in ((23, 6), (31, 6), (31, 14))]
    # A rule already in lat/lon converts to itself, given x/y checked and dropped.
    beside = _zone(sh, "z-b", xy_beside=True)
    again = wgs84_from_metres(beside, sh.policy.frame_origin)
    assert not any(point_gives_metres(v) for v in again.vertices)
    assert [(v.lat, v.lon) for v in again.vertices] == [(v.lat, v.lon) for v in beside.vertices]
    # ...and x/y that disagree with the frame are still refused there.
    off = _zone(sh, "z-off", xy_beside=True)
    off.vertices[0].__dict__["x"] = 99.0
    try:
        wgs84_from_metres(off, sh.policy.frame_origin)
    except ValueError as e:
        assert "cannot mix frames" in str(e), e
    else:
        raise AssertionError("disagreeing x/y were converted")


def test_derive_refuses_xy_beside_lat_lon():
    base = load_policy(SITL)
    sh = Shield(base)
    rule = _zone(sh, "z-derive", xy_beside=True).model_dump(mode="json", exclude_none=True)
    rule["vertices"][0].update(x=23.0, y=6.0)
    _refused(lambda: B.derive(base, [rule]))
    clean = copy.deepcopy(rule)
    for v in clean["vertices"]:
        v.pop("x", None), v.pop("y", None)
    assert B.derive(base, [clean]).generation == base.generation + 1


# --------------------------------------------------------------------------- #
# 3. REST: the grant's nested geometry block
# --------------------------------------------------------------------------- #

def _nested_nfz(sh: Shield, zid: str, *, xy_beside: bool) -> dict:
    verts = [_ll(sh, x, y) for x, y in ((23, 6), (31, 6), (31, 14))]
    if xy_beside:
        verts[0] = {**verts[0], "x": 23.0, "y": 6.0}
    return {"id": zid, **RULE, "margin_m": 1.0,
            "geometry": {"vertices": verts, "altitude_floor_m": 0, "altitude_ceiling_m": 100}}


def test_rest_nested_geometry_vertices_refused():
    sh = Shield(load_policy(SITL))
    gen, h = sh.policy.generation, sh.policy.policy_hash
    for fn, body in ((api.apply_dynamic_nfz, _nested_nfz(sh, "n1", xy_beside=True)),
                     (api.apply_event, {**_nested_nfz(sh, "n1", xy_beside=True),
                                        "type": "dynamic_nfz"})):
        sent = copy.deepcopy(body)
        try:
            fn(sh, body)
        except api.ApiError as e:
            assert e.status == 422 and TOOL_SIDE_CONVERSION in e.detail, (e.status, e.detail)
        else:
            raise AssertionError(f"{fn.__name__} applied a nested x/y body")
        assert body == sent, "the client's body was changed"
    # [x, y] pairs and {x, y} metres nested are refused the same way
    for verts in ([[23, 6], [31, 6], [31, 14]], [{"x": 23, "y": 6}, {"x": 31, "y": 6},
                                                 {"x": 31, "y": 14}]):
        b = _nested_nfz(sh, "n2", xy_beside=False)
        b["geometry"]["vertices"] = verts
        try:
            api.apply_dynamic_nfz(sh, b)
        except api.ApiError as e:
            assert e.status == 422, e.status
        else:
            raise AssertionError(f"nested {verts!r} applied")
    assert (sh.policy.generation, sh.policy.policy_hash) == (gen, h), "something was applied"


def test_rest_nested_geometry_centerline_refused():
    sh = Shield(load_policy(SURVEY))
    gen = sh.policy.generation
    body = {"id": "swap-n", "target_id": "corridor-survey-route", **RULE,
            "geometry": {"centerline": [{**_ll(sh, 0, 0), "x": 0.0, "y": 0.0},
                                        _ll(sh, 60, 0), _ll(sh, 60, 75.5)],
                         "width_m": 30, "altitude_floor_m": 10, "altitude_ceiling_m": 60}}
    for fn, b in ((api.apply_corridor_swap, body),
                  (api.apply_event, {**body, "type": "corridor_swap"})):
        try:
            fn(sh, copy.deepcopy(b))
        except api.ApiError as e:
            assert e.status == 422 and TOOL_SIDE_CONVERSION in e.detail, (e.status, e.detail)
        else:
            raise AssertionError(f"{fn.__name__} applied a nested x/y centerline")
    assert sh.policy.generation == gen


def test_rest_nested_lat_lon_body_applies_as_the_top_level_form():
    """No behaviour change for valid input: the nested lat/lon body stores
    exactly what the same body at the top level stores."""
    a, b = Shield(load_policy(SITL)), Shield(load_policy(SITL))
    nested = _nested_nfz(a, "n-ok", xy_beside=False)
    flat = {k: v for k, v in nested.items() if k != "geometry"}
    flat.update(nested["geometry"])
    ra = api.apply_dynamic_nfz(a, copy.deepcopy(nested))
    rb = api.apply_dynamic_nfz(b, copy.deepcopy(flat))
    assert ra["policy_hash"] == rb["policy_hash"] and ra["generation"] == 1, (ra, rb)
    sa, sb = Shield(load_policy(SURVEY)), Shield(load_policy(SURVEY))
    geom = {"centerline": [_ll(sa, 0, 0), _ll(sa, 60, 0), _ll(sa, 60, 75.5)],
            "width_m": 30, "altitude_floor_m": 10, "altitude_ceiling_m": 60}
    swap = {"id": "swap-ok", "target_id": "corridor-survey-route", **RULE}
    ra = api.apply_corridor_swap(sa, {**swap, "geometry": copy.deepcopy(geom)})
    rb = api.apply_corridor_swap(sb, {**swap, **copy.deepcopy(geom)})
    assert ra["policy_hash"] == rb["policy_hash"], (ra, rb)


# --------------------------------------------------------------------------- #
# 4. no Shield on a policy stored in local metres without a declared read
# --------------------------------------------------------------------------- #

METRE_DOC = {"policy_id": "metre-only", "version": "0.1.0", "constraints": [
    {"id": "nfz", "type": "polygon_fence", **RULE,
     "vertices": [{"x": 20, "y": -5}, {"x": 30, "y": -5}, {"x": 30, "y": 5}, {"x": 20, "y": 5}]},
    {"id": "band", "type": "altitude_envelope", **RULE, "alt_min_m": 2, "alt_max_m": 60}]}


def test_shield_refuses_a_metre_policy_without_a_declared_read():
    from guardrail.models import POINTS_LEGACY_METRES, Policy
    pol = Policy.model_validate(copy.deepcopy(METRE_DOC))
    assert pol.point_form == POINTS_LEGACY_METRES and not pol.legacy_read
    msg = _refused(lambda: Shield(pol))
    assert "legacy_metres=True" in msg and "load_legacy_metre_policy" in msg, msg
    # A mixed policy (a WGS84 policy with a metre rule appended) too.
    wgs = load_policy(SITL)
    mixed = wgs.model_copy(update={"constraints": list(wgs.constraints) + [
        PolygonFence.model_validate({"id": "m", "type": "polygon_fence", **RULE,
                                     "vertices": [{"x": 0, "y": 0}, {"x": 5, "y": 0},
                                                  {"x": 5, "y": 5}]})]})
    _refused(lambda: Shield(mixed))
    # WGS84 and point-free policies are untouched.
    Shield(wgs)
    Shield(Policy.model_validate({"policy_id": "band-only", "constraints": [
        METRE_DOC["constraints"][1]]}))


def test_declared_legacy_reads_still_build_a_shield():
    from guardrail.models import Policy, load_legacy_metre_policy, mark_legacy_read
    pol = Policy.model_validate(copy.deepcopy(METRE_DOC))
    # the caller declares it ...
    a = Shield(pol, legacy_metres=True)
    # ... or the policy came from a declared legacy read
    b = Shield(legacy_metre_policy_from_raw(copy.deepcopy(METRE_DOC), runtime=False))
    with tempfile.TemporaryDirectory() as tmp:
        f = Path(tmp) / "metre.json"
        f.write_text(json.dumps(METRE_DOC), encoding="utf-8")
        c = Shield(load_legacy_metre_policy(f, runtime=False))
        pol2, rec = B.load_for_flight(yaml_path=f, legacy_read=True)
        assert rec["legacy_read"] is True and pol2.legacy_read
        Shield(pol2)
    d = Shield(mark_legacy_read(Policy.model_validate(copy.deepcopy(METRE_DOC))))
    # the same policy, the same enforcement
    from guardrail.models import Action4D, State
    st, act = State(x=10.0, y=0.0, up=10.0), Action4D(vx=4.0)
    outs = {repr(sh.filter(st, act).emitted) for sh in (a, b, c, d)}
    assert len(outs) == 1, outs
    # the label survives a hot-applied generation (model_copy) and is never hashed
    assert pol.model_copy(deep=True).legacy_read is False
    marked = mark_legacy_read(Policy.model_validate(copy.deepcopy(METRE_DOC)))
    assert marked.model_copy(deep=True).legacy_read
    assert marked.policy_hash == pol.policy_hash


def test_a_lock_rebuild_and_the_comparison_switch_are_declared_reads():
    from guardrail.models import Policy
    base = Policy.model_validate(copy.deepcopy(METRE_DOC))
    zone = {"id": "z", "type": "dynamic_nfz", **RULE,
            "vertices": [{"x": 40, "y": 0}, {"x": 45, "y": 0}, {"x": 45, "y": 5}],
            "altitude_floor_m": 0, "altitude_ceiling_m": 100}
    rebuilt = B.derive(base, [zone], legacy_read=True)
    assert rebuilt.legacy_read
    Shield(rebuilt)
    # wgs84_writes=False stores metres in a WGS84 policy: its generations are
    # the legacy form, declared as such, so a Shield can be built on them.
    sh = Shield(load_policy(SITL))
    sh.wgs84_writes = False
    sh.spawn_nfz(DynamicNFZ.model_validate(zone))
    assert sh.policy.point_form != "wgs84" and sh.policy.legacy_read
    Shield(sh.policy)


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
