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

A second review of that fix (2026-11-26) found more, closed here too: a
bundle hashed before 2026-10-06 whose archived ir.json writes x/y beside
lat/lon flew through load_for_flight / InjectPolicy; the legacy-read label
followed the Policy object, not its content; as_dynamic_nfz dumped the x/y
view and so its own migration was refused; POST /nfz ignored a `geometry:`
block; and the lock's rewound metre versions (bundle.policy_candidates)
could no longer build a Shield.

A third review (2026-11-26, second independent re-review of WP1-14) found:
bundle.load_bundle(runtime=True), documented as applying load_for_flight's
gate, applied neither point-form refusal; POST /nfz stored a vertex whose x
was null (NfzRequest's dump dropped it) and every route dropped any other
unknown point key; and Shield.move_nfz refused a models.XY given {lat, lon}
as "given in local metres". Closed here too. (The Ingest CLI's metre
conversion, the other finding, is tests/test_geo_to_policy.py.)

The independent review of that fix (2026-11-26) found that a declared
legacy read (VLAGUARD_LEGACY_READ=1) flew a NEW policy in local metres
through load_for_flight; a declared read now takes a policy the lock records
only (bundle.recorded_as; tests/recorded_legacy.py gives the recorded ones).
It also found that PATCH /dynamic_nfz move ignored a `geometry:` block; a
move now refuses any key it does not take.

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
from recorded_legacy import (recorded_metre_policy,                # noqa: E402
                             recorded_metre_versions, write_recorded_metre_bundle,
                             write_recorded_metre_yaml)

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
        # the flight loader's declared read: of a recorded policy only
        _refused(lambda: B.load_for_flight(yaml_path=f, legacy_read=True),
                 B.LegacyPointFormRefused)
        _, recorded = write_recorded_metre_yaml(Path(tmp))
        pol2, rec = B.load_for_flight(yaml_path=recorded, legacy_read=True)
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


# --------------------------------------------------------------------------- #
# The second review (2026-11-26): the bundle entry, the label's scope, the
# migration helper, POST /nfz, and the readers of recorded artefacts.
# --------------------------------------------------------------------------- #

def _legacy16_xy_bundles(pol, folder: Path) -> list[Path]:
    """Bundles as written before 2026-10-06: a 16-hex legacy hash over the
    rebuilt object, the archived ir.json a plain model_dump - which writes
    each WGS84 point's local x/y view beside its lat/lon. Unsigned."""
    import gzip
    import io
    import tarfile
    from guardrail.models import _prune_nulls
    good = B.write_bundle(pol, folder / "good.tar.gz", signer=None,
                          issued_at="2026-11-26T00:00:00Z")
    with tarfile.open(good, "r:gz") as t:
        man = json.loads(t.extractfile(B.MANIFEST_NAME).read())
    for k in ("hashes", "reference_projection", "multiscale_sha256"):
        man.pop(k, None)
    ir_bytes = json.dumps(_prune_nulls(pol.model_dump(mode="json"), frozenset()),
                          sort_keys=True).encode()
    out = []
    for name, h in sorted(set(pol.legacy_hashes().items())):
        mb = json.dumps(dict(man, policy_hash=h), indent=2, sort_keys=True).encode()
        p = folder / f"xy_{name}.tar.gz"
        with open(p, "wb") as fh, \
                gzip.GzipFile(fileobj=fh, mode="wb", mtime=0, filename="") as gz, \
                tarfile.open(fileobj=gz, mode="w") as tar:
            for nm, data in ((B.IR_NAME, ir_bytes), (B.MANIFEST_NAME, mb),
                             (B.SIGNATURE_NAME, f"{h} {B.UNSIGNED}\n".encode())):
                ti = tarfile.TarInfo(nm)
                ti.size, ti.mtime = len(data), 0
                tar.addfile(ti, io.BytesIO(data))
        out.append(p)
    return out


def test_a_legacy16_bundle_writing_xy_beside_latlon_is_a_read_only():
    """The archived ir.json of a bundle hashed before 2026-10-06 is a plain
    model_dump, which writes x/y beside lat/lon. The model checks those x/y
    to 1 um and drops them, so until this fix such a bundle flew through
    load_for_flight - a rail's --bundle, the harness's InjectPolicy archive
    (allow_unverified) - as a WGS84 policy, with no declared read (failed
    then: all three legacy16 forms ACCEPTED). Now it is refused for flight
    as a metre bundle is (LegacyPointFormRefused), read only under
    legacy_read, and the record says so."""
    import os
    os.environ.pop(B.LEGACY_READ_ENV, None)
    pol = load_policy(SITL)
    n_points = sum(len(c.vertices) for c in pol.constraints if hasattr(c, "vertices"))
    with tempfile.TemporaryDirectory() as tmp:
        bundles = _legacy16_xy_bundles(pol, Path(tmp))
        assert bundles
        assert B.check_bundle(Path(tmp) / "good.tar.gz").projected_points == 0
        for b in bundles:
            chk = B.check_bundle(b)
            assert chk.hash_form.startswith("legacy16"), chk.hash_form
            assert chk.projected_points == n_points > 0, chk.projected_points
            assert chk.policy.point_form == "wgs84"      # the x/y were dropped
            msg = _refused(lambda: B.load_for_flight(bundle=b, allow_unverified=True),
                           B.LegacyPointFormRefused, names_conversion=False)
            assert "x/y beside lat/lon" in msg and B.LEGACY_READ_ENV in msg, msg
            got, rec = B.load_for_flight(bundle=b, allow_unverified=True,
                                         legacy_read=True)
            assert rec["legacy_read"] is True and rec["projected_points"] == n_points
            assert rec["point_note"] == B.XY_BESIDE_NOTE and got.legacy_read
            assert got.policy_hash == pol.policy_hash
        # The harness's InjectPolicy, uploaded archive bytes: refused.
        from guardrail import harness_service as H
        h = H.Harness(runs_dir=Path(tmp) / "runs")
        s = h.start_session()["session_id"]
        try:
            h.inject_policy(s, bundle_bytes=bundles[0].read_bytes(),
                            expected_hash=pol.policy_hash)
        except H.HarnessError as e:
            assert e.code == "INVALID_ARGUMENT" and "LegacyPointFormRefused" in e.message, e
        else:
            raise AssertionError("InjectPolicy flew a bundle with x/y beside lat/lon")
        # A bundle written today (lat/lon only) is injected as before.
        ack = h.inject_policy(s, bundle_bytes=(Path(tmp) / "good.tar.gz").read_bytes(),
                              expected_hash=pol.policy_hash)
        assert ack["policy_hash"] == pol.policy_hash, ack


def test_the_legacy_read_label_covers_the_content_it_was_made_on():
    """The declared read is bound to the metre points the policy held when
    it was declared (Policy.legacy_read_covers), not to the Policy object.
    Failed before (probe A4): a WGS84 policy read through a legacy reader,
    then given metre constraints with model_copy, still built a Shield."""
    from guardrail.models import Policy, mark_legacy_read, read_recorded_policy
    wgs = legacy_metre_policy_from_raw(
        json.loads(json.dumps(load_policy(SITL).model_dump(
            mode="json", exclude_none=True, context={"points": "stored"}))))
    assert wgs.legacy_read and wgs.point_form == "wgs84"
    metre_rule = PolygonFence.model_validate(
        {"id": "late", "type": "polygon_fence", **RULE,
         "vertices": [{"x": 0, "y": 0}, {"x": 5, "y": 0}, {"x": 5, "y": 5}]})
    grown = wgs.model_copy(update={"constraints": list(wgs.constraints) + [metre_rule]})
    assert grown.legacy_read and not grown.legacy_read_covers()
    msg = _refused(lambda: Shield(grown))
    assert "put in after" in msg, msg
    # A marked metre policy given ANOTHER metre rule: not covered either ...
    metre = mark_legacy_read(Policy.model_validate(copy.deepcopy(METRE_DOC)))
    more = metre.model_copy(update={"constraints": list(metre.constraints) + [metre_rule]})
    _refused(lambda: Shield(more))
    # ... nor a metre point moved in place; dropping a rule keeps it covered.
    moved = metre.model_copy(deep=True)
    v0 = moved.constraints[0].vertices
    moved.constraints[0].vertices = [type(v0[0]).model_validate({"x": v0[0].x + 1.0,
                                                                 "y": v0[0].y})] + v0[1:]
    assert moved.legacy_read
    _refused(lambda: Shield(moved))
    Shield(metre.model_copy(update={"constraints": metre.constraints[1:]}))
    Shield(metre)
    # The label does not survive a dump / model_validate round trip: a
    # re-read declares again (read_recorded_policy).
    again = Policy.model_validate(metre.model_dump(mode="json"))
    assert not again.legacy_read
    _refused(lambda: Shield(again))
    Shield(read_recorded_policy(metre.model_dump(mode="json")))
    # The Shield's own events keep the declaration over the covered content
    # they leave: a mixed generation (a metre zone stored by the comparison
    # switch), then WGS84 events on it, re-read by a new Shield each time.
    zone = {"id": "z", "type": "dynamic_nfz", **RULE,
            "vertices": [{"x": 40, "y": 0}, {"x": 45, "y": 0}, {"x": 45, "y": 5}],
            "altitude_floor_m": 0, "altitude_ceiling_m": 100}
    sh = Shield(load_policy(SITL))
    sh.wgs84_writes = False
    sh.spawn_nfz(DynamicNFZ.model_validate(zone))
    sh.wgs84_writes = True
    assert sh.policy.point_form != "wgs84"
    sh.spawn_nfz(DynamicNFZ.model_validate(
        {**zone, "id": "z2", "vertices": [_ll(sh, 60, 0), _ll(sh, 65, 0), _ll(sh, 65, 5)]}))
    Shield(sh.policy)
    sh.translate_nfz("z", 2.0, 1.0)      # written back in WGS84 (the frame exists)
    Shield(sh.policy)


def test_as_dynamic_nfz_migrates_a_wgs84_fence():
    """MIGRATION_NOTE tells a caller to hot-apply as_dynamic_nfz(fence). Its
    vertices were a plain model_dump, which for a WGS84 point bound to a
    frame writes the local x/y beside lat/lon, so the Shield refused the
    rule (failed before: HotApplyRefused, '4 point(s) carry x/y beside
    lat/lon'). Now the vertices are copied as stored, lat/lon only."""
    from guardrail.shield import as_dynamic_nfz
    pol = load_policy(SURVEY)
    sh = Shield(pol)
    fence = next(c for c in pol.constraints if isinstance(c, PolygonFence))
    dyn = as_dynamic_nfz(fence.model_copy(update={"id": fence.id + "-dyn"}))
    assert all(set(v.model_fields_set) == {"lat", "lon"} for v in dyn.vertices)
    ev = sh.hot_apply(dyn)
    assert ev["rule_id"] == fence.id + "-dyn", ev
    got = sh.policy.constraints[-1]
    assert [(v.lat, v.lon) for v in got.vertices] == [(v.lat, v.lon) for v in fence.vertices]
    # A legacy metre fence still migrates in the form it is stored in.
    m = PolygonFence.model_validate({"id": "m", "type": "polygon_fence", **RULE,
                                     "vertices": [{"x": 0, "y": 0}, {"x": 5, "y": 0},
                                                  {"x": 5, "y": 5}]})
    assert [(v.x, v.y) for v in as_dynamic_nfz(m).vertices] == [(0, 0), (5, 0), (5, 5)]


def test_post_nfz_refuses_a_geometry_block():
    """POST /nfz (the 2026-07 body) ignored any key NfzRequest does not
    define, so a `geometry:` block with x/y beside lat/lon was dropped
    without a word while /dynamic_nfz and /hot_apply answered 422 (failed
    before: ACCEPTED, the zone applied from the top-level vertices)."""
    sh = Shield(load_policy(SITL))
    top = [_ll(sh, 20, -5), _ll(sh, 30, -5), _ll(sh, 30, 5)]
    beside = [{**p, "x": 0.0, "y": 0.0} for p in top]
    for geometry, conversion in (({"vertices": beside}, True), ({"vertices": top}, False)):
        try:
            api.apply_legacy_nfz(sh, {"id": "old", "vertices": top, "geometry": geometry})
        except api.ApiError as e:
            assert e.status == 422, e.status
            assert (TOOL_SIDE_CONVERSION in e.detail) == conversion, e.detail
        else:
            raise AssertionError("POST /nfz ignored a geometry block")
    assert sh.policy.generation == 0
    # The 2026-07 body itself is applied as before.
    res = api.apply_legacy_nfz(sh, {"id": "old", "vertices": top})
    assert res["type"] == "dynamic_nfz" and sh.policy.generation == 1


def test_load_bundle_runtime_applies_the_point_form_refusals():
    """bundle.load_bundle(runtime=True) says it applies load_for_flight's
    flight gate, but a legacy16 bundle whose ir.json writes x/y beside
    lat/lon and a bundle stored in local metres both loaded, and a Shield
    built on the first (failed before: ACCEPTED, second re-review 26 Nov).
    Now both are refused (LegacyPointFormRefused) unless the read is
    declared (legacy_read=True, or VLAGUARD_LEGACY_READ=1), which labels the
    policy; runtime=False (the declaration) and a WGS84 bundle load as
    before; write_bundle's `previous` (a recorded bundle, the one caller)
    declares the read, so an older metre version still gives its diff.
    Since the third fix of 2026-11-26 the declared read takes a recorded
    policy only, so the metre bundle here is a version the lock records
    (tests/recorded_legacy.py); a new one is
    test_a_declared_legacy_read_reads_a_recorded_policy_only."""
    import os
    os.environ.pop(B.LEGACY_READ_ENV, None)
    pol = load_policy(SITL)
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        xy = _legacy16_xy_bundles(pol, d)
        metre = write_recorded_metre_bundle(d)[1]
        good = d / "good.tar.gz"
        assert B.load_bundle(good, require_signature=False).policy_hash == pol.policy_hash
        for b, words in [(x, ("x/y beside lat/lon",)) for x in xy] +                 [(metre, ("local-metres",))]:
            msg = _refused(lambda: B.load_bundle(b, require_signature=False),
                           B.LegacyPointFormRefused, names_conversion=False)
            for w in (*words, B.LEGACY_READ_ENV):
                assert w in msg, (b.name, w, msg)
            got = B.load_bundle(b, require_signature=False, legacy_read=True)
            assert got.legacy_read, b.name
            Shield(got)                                   # a declared read builds one
            assert not B.load_bundle(b, require_signature=False,
                                     runtime=False).legacy_read, b.name
            os.environ[B.LEGACY_READ_ENV] = "1"
            try:
                assert B.load_bundle(b, require_signature=False).legacy_read, b.name
            finally:
                os.environ.pop(B.LEGACY_READ_ENV, None)
        # write_bundle(previous=<an older metre bundle>): the diff is a read.
        # (_previous_policy also requires a verified signature, which these
        # unsigned fixtures lack; only that check is relaxed here.)
        real = B.load_bundle
        B.load_bundle = lambda *a, **k: real(*a, **{**k, "require_signature": False})
        try:
            new = B.write_bundle(pol, d / "next.tar.gz", signer=None, previous=metre)
        finally:
            B.load_bundle = real
        assert B.check_bundle(new).manifest.get("changelog_diff") is not None


def test_a_declared_legacy_read_reads_a_recorded_policy_only():
    """Third independent review of WP1-14 (2026-11-26): with
    VLAGUARD_LEGACY_READ=1 (or legacy_read=True), load_for_flight,
    load_yaml_for_flight and load_bundle(runtime=True) read a NEW policy in
    local metres - a polygon_fence in {x, y} with no origin, never
    recorded - and a Shield was built on it (failed before: ACCEPTED). The
    declared read now takes a recorded artefact only: one form of its
    policy_hash, as the era of its document computes it, is in
    policies/policy.lock.json (bundle.recorded_as). Anything else - a new
    metre YAML, a new metre bundle, a legacy16 bundle writing x/y beside
    lat/lon of a policy no version of which is locked - is refused
    (LegacyPointFormRefused, naming models.wgs84_from_metres) with the
    variable set. The lock's own metre version, written as the file was
    (scope left out) and bundled as the lab's bundles of the time hold it,
    is still read, labelled, and says which record it matched."""
    import os
    from guardrail.models import Policy
    os.environ.pop(B.LEGACY_READ_ENV, None)
    new_doc = {"policy_id": "new-metre", "version": "0.0.1", "constraints": [
        {"id": "fence", "type": "polygon_fence", **RULE, "margin_m": 1.0,
         "vertices": [{"x": 10, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 10}]}]}
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        new_yaml = d / "new_metre.yaml"
        new_yaml.write_text(json.dumps(new_doc), encoding="utf-8")
        new_bundle = B.write_bundle(Policy.model_validate(copy.deepcopy(new_doc)),
                                    d / "new-metre-v0.0.1.tar.gz", signer=None,
                                    allow_legacy_metres=True)
        (d / "xy").mkdir()
        unlocked = load_policy(SITL).model_copy(update={"policy_id": "new-wgs84"})
        new_xy = _legacy16_xy_bundles(unlocked, d / "xy")
        key, old_yaml = write_recorded_metre_yaml(d)
        _, old_bundle = write_recorded_metre_bundle(d)
        yaml_loads = {
            "load_for_flight(yaml_path)": lambda p, **k: B.load_for_flight(yaml_path=p, **k),
            "load_yaml_for_flight": lambda p, **k: B.load_yaml_for_flight(
                p, say=lambda s: None)}
        bundle_loads = {
            "load_for_flight(bundle)": lambda p, **k: B.load_for_flight(
                p, allow_unverified=True, **k),
            "load_bundle(runtime=True)": lambda p, **k: B.load_bundle(
                p, require_signature=False, **k)}
        os.environ[B.LEGACY_READ_ENV] = "1"
        try:
            for name, load in yaml_loads.items():
                msg = _refused(lambda: load(new_yaml), B.LegacyPointFormRefused)
                assert "not a recorded policy" in msg and "local-metres" in msg, (name, msg)
            for name, load in bundle_loads.items():
                for b in (new_bundle, *new_xy):
                    msg = _refused(lambda: load(b), B.LegacyPointFormRefused)
                    assert "not a recorded policy" in msg, (name, b.name, msg)
            # The recorded version: read, labelled, a Shield built on it.
            for name, load in yaml_loads.items():
                pol, rec = load(old_yaml)
                assert pol.legacy_read and rec["legacy_read"] is True, name
                assert rec["recorded_as"] == key and rec["point_form"] == pol.point_form
                Shield(pol)
            pol, rec = bundle_loads["load_for_flight(bundle)"](old_bundle)
            assert pol.legacy_read and rec["recorded_as"] == key, rec
            got = bundle_loads["load_bundle(runtime=True)"](old_bundle)
            assert got.legacy_read and got.policy_hash == pol.policy_hash
            Shield(got)
        finally:
            os.environ.pop(B.LEGACY_READ_ENV, None)
        # The argument is the same declaration, and gated the same way.
        _refused(lambda: B.load_for_flight(yaml_path=new_yaml, legacy_read=True),
                 B.LegacyPointFormRefused)
        _refused(lambda: B.load_bundle(new_bundle, require_signature=False,
                                       legacy_read=True), B.LegacyPointFormRefused)
        assert B.load_for_flight(yaml_path=old_yaml, legacy_read=True)[1]["recorded_as"] == key
    # Every metre version the lock records is recognised as itself; a new
    # document is not; the lock is read once.
    for k, pol in recorded_metre_versions():
        assert B.recorded_as(pol) == k, k
    assert B.recorded_as(Policy.model_validate(copy.deepcopy(new_doc))) is None
    assert B.recorded_as(recorded_metre_policy()[1]) == key
    assert B.recorded_policy_hashes() is B.recorded_policy_hashes()


def test_rest_refuses_any_point_key_but_lat_lon():
    """The published schema's point is {lat, lon} with additionalProperties
    false, and a document's point refuses another key (extra_forbidden), but
    REST dropped it (`X`, `alt`) and stored the zone; POST /nfz also stored
    a vertex whose `x` was null, because NfzRequest's dump dropped it before
    the point check (failed before: both ACCEPTED, 200). Now 422 on every
    route, nothing stored; a clean {lat, lon} body is applied as before."""
    sh = Shield(load_policy(SITL))
    top = [_ll(sh, 20, -5), _ll(sh, 30, -5), _ll(sh, 30, 5)]
    bad_points = [("X", [{**top[0], "X": 1.0}] + top[1:]),
                  ("alt", [{**top[0], "alt": 30.0}] + top[1:]),
                  ("x null", [{**top[0], "x": None}] + top[1:])]
    for name, verts in bad_points:
        for route, body in ((api.apply_legacy_nfz, {"id": "z", "vertices": verts}),
                            (api.apply_dynamic_nfz, {"id": "z", **RULE, "vertices": verts}),
                            (api.apply_event,                     # /hot_apply, gRPC
                             {"id": "z", "type": "dynamic_nfz", **RULE,
                              "geometry": {"vertices": verts}})):
            try:
                route(sh, copy.deepcopy(body))
            except api.ApiError as e:
                assert e.status == 422, (name, route.__name__, e.status)
            else:
                raise AssertionError(f"{route.__name__} stored a point with {name}")
            assert sh.policy.generation == 0, (name, route.__name__)
    assert api.apply_legacy_nfz(sh, {"id": "z", "vertices": top})["type"] == "dynamic_nfz"


def test_move_takes_an_xy_given_lat_lon():
    """Shield.move_nfz with models.XY points: one given {lat, lon} alone is
    the WGS84 form and is applied as the dict form is (failed before:
    refused as "an absolute position is given in local metres"); one given
    {x, y} metres is refused, naming the conversion."""
    from guardrail.models import XY
    sh = Shield(load_policy(SITL))
    sh.spawn_nfz(DynamicNFZ.model_validate(
        {"id": "z", "type": "dynamic_nfz", **RULE,
         "vertices": [_ll(sh, 20, -5), _ll(sh, 30, -5), _ll(sh, 30, 5)]}))
    ring = [_ll(sh, 40, -5), _ll(sh, 50, -5), _ll(sh, 50, 5)]
    twin = Shield(load_policy(SITL))
    twin.spawn_nfz(DynamicNFZ.model_validate(
        {"id": "z", "type": "dynamic_nfz", **RULE,
         "vertices": [_ll(sh, 20, -5), _ll(sh, 30, -5), _ll(sh, 30, 5)]}))
    sh.move_nfz("z", [XY.model_validate(v) for v in ring])
    twin.move_nfz("z", ring)
    assert sh.policy.policy_hash == twin.policy.policy_hash
    msg = _refused(lambda: sh.move_nfz("z", [XY(x=0.0, y=0.0), XY(x=5.0, y=0.0),
                                            XY(x=5.0, y=5.0)]), HotApplyRefused)
    assert "local metres" in msg, msg


def test_patch_move_refuses_a_geometry_block_and_unknown_keys():
    """PATCH /dynamic_nfz/{id} {op: move} read `vertices` and ignored every
    other key, so {op: move, vertices: [lat/lon], geometry: {vertices:
    [{x, y}]}} answered 200, dropped the block and bumped the generation 1
    -> 2 (third independent review of WP1-14, 2026-11-26; failed before:
    200). Now 422, nothing applied: a geometry block in metres is refused
    for its points (naming the conversion), any other unknown key for what
    it is. A clean move, with `motion` or `layer: mission`, is applied as
    before."""
    sh = Shield(load_policy(SITL))
    api.apply_dynamic_nfz(sh, {"id": "z", **RULE, "vertices": [
        _ll(sh, 20, -5), _ll(sh, 30, -5), _ll(sh, 30, 5)]})
    assert sh.policy.generation == 1
    ring = [_ll(sh, 40, -5), _ll(sh, 50, -5), _ll(sh, 50, 5)]
    metre_block = {"vertices": [{"x": 0, "y": 0}, {"x": 5, "y": 0}, {"x": 5, "y": 5}]}
    for body, words in (
            ({"op": "move", "vertices": ring, "geometry": metre_block},
             (TOOL_SIDE_CONVERSION,)),
            ({"op": "move", "vertices": ring, "geometry": {"vertices": ring}},
             ("geometry",)),
            ({"op": "move", "vertices": ring, "X": 1.0}, ("unknown key", "X"))):
        try:
            api.apply_nfz_edit(sh, "z", copy.deepcopy(body))
        except api.ApiError as e:
            assert e.status == 422, (body, e.status)
            for w in words:
                assert w in e.detail, (w, e.detail)
        else:
            raise AssertionError(f"PATCH move applied {sorted(body)}")
        assert sh.policy.generation == 1, body
    assert api.apply_nfz_edit(sh, "z", {"op": "move", "vertices": ring,
                                        "layer": "mission"})["generation"] == 2
    assert api.apply_nfz_edit(sh, "z", {"op": "move", "vertices": ring,
                                        "motion": None})["generation"] == 3
    # translate carries no point; its body is as it was
    assert api.apply_nfz_edit(sh, "z", {"op": "translate", "dx": 1.0})["generation"] == 4


def test_every_run_resolution_candidate_builds_a_shield():
    """bundle.policy_candidates offers each policy a stored run may have
    flown under, the old metre versions the lock rewinds to included
    (Policy.metre_form). Resolving a stored run is a declared read, so a
    Shield can be built on every candidate (failed before: Shield refused
    the 52 rewound metre versions, ProjectedCoordinatesRefused)."""
    from guardrail.models import LEGACY_POINT_FORMS
    metre = 0
    for label, pol in B.policy_candidates():
        if pol.point_form in LEGACY_POINT_FORMS:
            metre += 1
        try:
            Shield(pol)
        except ProjectedCoordinatesRefused as e:
            raise AssertionError(f"{label}: {e}") from None
        except ValueError:
            pass                 # a rule needing a ground / map source: not WP1-14
    assert metre > 0, "no metre candidate: the test checks nothing"
    assert load_policy(SITL).metre_form().legacy_read


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
