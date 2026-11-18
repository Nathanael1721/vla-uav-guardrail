"""
WP1-14: no writer produces the legacy metre form any more (2026-11-18).

    python tests/test_wgs84_frameless_writers.py      (no pytest needed)

The grant (Policy DSL page): "WGS84 latitude / longitude is canonical ... the
DSL itself never carries projected coordinates". Since 2026-11-17 every writer
into a policy WITH a frame origin converts metre input to WGS84 at ingest
(tests/test_wgs84_writes.py, tests/test_wgs84_ingest_writers.py). The row's
residual named the writers that could still store metres, all of them on a
policy with NO frame origin (written only in metres, the legacy read form, or
with no point and no stated origin), where a metre point has no WGS84
position. Each is now refused with one clear message (models.no_frame_message)
and leaves the policy as it was; this file checks every one:

  * the Shield: spawn, a pre-start polygon_fence / circle_fence, a
    corridor_swap, move / translate / rotate / scale of a zone the policy
    declares, on a metre-only policy and on a point-free policy without an
    origin; a lat/lon zone is refused as well (it would silently become the
    frame the policy's metres are read in); a time_window_switch (no points)
    and an expire are not affected;
  * REST (guardrail/api.py, the handlers the routes call): POST /dynamic_nfz,
    /hot_apply, /nfz, /corridor_swap and PATCH answer 422 with the message;
  * the harness's ScenarioEvents (guardrail/scenario_events.py): a spawn is
    refused, and recorded as refused when the scenario expects it;
  * the gRPC harness's HotApply: its dry run (Harness._dry_shield, the REST
    handler) refuses the body with INVALID_ARGUMENT, so no ScenarioEvent is
    built for the runner;
  * bundle.derive: a new derivation on a metre base is refused; the lock's
    recorded recipe is read back with legacy_read (as it flew);
  * bundle.write_bundle and `python -m guardrail.bundle`: a policy whose points
    are metres is not bundled (allow_legacy_metres only for rebuilding a
    recorded bundle or a fixture standing for one);
  * tools/geo_to_policy.py ingest: metre rules onto a frameless base and a
    whole metre document are refused; metre rules onto a base with a frame
    are converted (as before);
  * demo/gen_random_scenario.py --metres is refused by name, writing nothing;
  * the same writes on a policy WITH a frame still succeed and store WGS84.
"""
from __future__ import annotations

import copy
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import api                                          # noqa: E402
from guardrail.bundle import derive, load_lock, write_bundle       # noqa: E402
from guardrail.harness_service import Harness, HarnessError, _HotBody  # noqa: E402
from guardrail.models import (POINTS_LEGACY_METRES, POINTS_NONE,   # noqa: E402
                              Action4D,
                              POINTS_WGS84, CircleFence, CorridorSwap,
                              DynamicNFZ, NoFrameOrigin, Policy, PolygonFence,
                              State, TimeWindowSwitch, load_policy, no_frame_message,
                              rule_in_wgs84)
from guardrail.scenario_events import EventState, apply_event      # noqa: E402
from guardrail.scenario_spec import ScenarioEvent                  # noqa: E402
from guardrail.shield import HotApplyRefused, Shield               # noqa: E402

RULE = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
        "violation_action": "project_fix"}
SQUARE = [{"x": 40, "y": -5}, {"x": 50, "y": -5}, {"x": 50, "y": 5}, {"x": 40, "y": 5}]
WHY = "no WGS84 frame origin"
FRAME = {"lat": 35.6895, "lon": 139.6917}

# A policy written only in metres (the legacy read form of the pre-3-Nov
# bundles), with a zone it declares, a corridor and a switchable fence.
METRE_DOC = {"policy_id": "legacy-metre", "version": "0.0.1", "constraints": [
    {"id": "fence", "type": "polygon_fence", **RULE, "margin_m": 1.0,
     "vertices": [{"x": 10, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 10}]},
    {"id": "lane", "type": "corridor", **RULE, "width_m": 30,
     "centerline": [{"x": -100, "y": 0}, {"x": 200, "y": 0}],
     "altitude_floor_m": 0, "altitude_ceiling_m": 120},
    {"id": "zone", "type": "dynamic_nfz", **RULE, "margin_m": 0.0,
     "vertices": [{"x": -30, "y": 20}, {"x": -20, "y": 20}, {"x": -20, "y": 30},
                  {"x": -30, "y": 30}]}]}
# No point and no stated origin: no frame either.
POINTFREE_DOC = {"policy_id": "caps-only", "version": "0.0.1", "constraints": [
    {"id": "alt", "type": "altitude_envelope", **RULE, "alt_min_m": 2, "alt_max_m": 60}]}


def _metre():
    return Policy.model_validate(copy.deepcopy(METRE_DOC))


def _pointfree():
    return Policy.model_validate(copy.deepcopy(POINTFREE_DOC))


def _framed():
    return Policy.model_validate({**copy.deepcopy(POINTFREE_DOC), "origin": FRAME})


def _zone(zid="z-new", verts=None):
    return DynamicNFZ.model_validate({"id": zid, "type": "dynamic_nfz", **RULE,
                                      "margin_m": 1.0, "vertices": verts or SQUARE})


def _refused(fn, exc=ValueError, *words):
    try:
        fn()
    except exc as e:
        for w in (WHY, *words):
            assert w in str(e), (w, str(e))
        return e
    raise AssertionError(f"{fn} wrote into a policy with no frame origin")


def _unchanged(sh, before):
    assert (sh.policy.policy_hash, sh.policy.generation) == before, \
        (sh.policy.policy_hash, sh.policy.generation, before)


# ------------------------------------------------------------- the Shield

def test_the_shield_refuses_every_write_with_points_on_a_frameless_policy():
    for make in (_metre, _pointfree):
        pol = make()
        assert pol.frame_origin is None
        sh = Shield(pol)
        before = (sh.policy.policy_hash, sh.policy.generation)
        form = sh.policy.point_form
        assert form in (POINTS_LEGACY_METRES, POINTS_NONE), form
        # pre-start fences (before the first tick)
        _refused(lambda: sh.hot_apply(PolygonFence.model_validate(
            {"id": "pre", "type": "polygon_fence", **RULE, "vertices": SQUARE})),
            HotApplyRefused, "add_before_start", "'pre'")
        _refused(lambda: sh.hot_apply(CircleFence.model_validate(
            {"id": "disc", "type": "circle_fence", **RULE, "radius_m": 5.0,
             "center": {"x": 60, "y": 0}})), HotApplyRefused, "'disc'")
        _unchanged(sh, before)
        sh.filter(State(x=0.0, y=0.0, up=10.0), Action4D())     # mission started
        _refused(lambda: sh.spawn_nfz(_zone()), HotApplyRefused, "spawn", "'z-new'")
        # a lat/lon zone too: it would become the frame the metres are read in
        o = (FRAME["lat"], FRAME["lon"])
        geo = [{"lat": o[0] + d[0], "lon": o[1] + d[1]}
               for d in ((0, 0), (1e-4, 0), (1e-4, 1e-4), (0, 1e-4))]
        _refused(lambda: sh.spawn_nfz(_zone("z-geo", geo)), HotApplyRefused, "'z-geo'")
        _unchanged(sh, before)
        assert sh.policy.point_form == form
    # the metre policy's own zone and corridor: edits and a swap are refused
    sh = Shield(_metre())
    sh.filter(State(x=0.0, y=0.0, up=10.0), Action4D())
    before = (sh.policy.policy_hash, sh.policy.generation)
    for fn in (lambda: sh.translate_nfz("zone", 3.0, 0.0),
               lambda: sh.rotate_nfz("zone", 15.0),
               lambda: sh.scale_nfz("zone", 1.5),
               lambda: sh.move_nfz("zone", [(-30, 40), (-20, 40), (-20, 50), (-30, 50)]),
               lambda: sh.hot_apply(CorridorSwap.model_validate(
                   {"id": "lane-b", "type": "corridor_swap", **RULE, "target_id": "lane",
                    "width_m": 30, "centerline": [{"x": -100, "y": 20}, {"x": 200, "y": 20}],
                    "altitude_floor_m": 0, "altitude_ceiling_m": 120}))):
        _refused(fn, HotApplyRefused)
    _unchanged(sh, before)
    # what writes no point is not affected
    sh.hot_apply(TimeWindowSwitch.model_validate(
        {"id": "off", "type": "time_window_switch", **RULE, "target_id": "fence",
         "active": False}))
    sh.expire_nfz("zone")
    assert sh.policy.generation == before[1] + 2
    assert sh.policy.point_form == POINTS_LEGACY_METRES


def test_the_converter_names_the_rule_and_passes_a_rule_without_points():
    z = _zone("named")
    e = _refused(lambda: rule_in_wgs84(z, None), NoFrameOrigin, "dynamic_nfz 'named'")
    assert str(e) == no_frame_message("dynamic_nfz 'named'")
    sw = TimeWindowSwitch.model_validate({"id": "s", "type": "time_window_switch", **RULE,
                                          "target_id": "x", "active": False})
    assert rule_in_wgs84(sw, None) is sw


def test_with_a_frame_the_same_writes_store_wgs84():
    sh = Shield(_framed())
    sh.filter(State(x=0.0, y=0.0, up=10.0), Action4D())
    sh.spawn_nfz(_zone())
    sh.rotate_nfz("z-new", 15.0)
    assert sh.policy.point_form == POINTS_WGS84
    assert all(v.geographic for v in sh.policy.constraints[-1].vertices)


# ------------------------------------------------------------------ REST

def test_rest_answers_422_naming_the_missing_origin():
    for make in (_metre, _pointfree):
        sh = Shield(make())
        before = (sh.policy.policy_hash, sh.policy.generation)
        body = {"id": "rest-z", **RULE, "margin_m": 1.0, "vertices": SQUARE}
        calls = [lambda: api.apply_dynamic_nfz(sh, copy.deepcopy(body)),
                 lambda: api.apply_event(sh, {"type": "dynamic_nfz", **copy.deepcopy(body)}),
                 lambda: api.apply_legacy_nfz(sh, {"id": "legacy-z", "vertices": SQUARE,
                                                   "altitude_floor_m": 0,
                                                   "altitude_ceiling_m": 100})]
        if make is _metre:
            calls += [lambda: api.apply_corridor_swap(sh, {
                          "id": "lane-b", **RULE, "target_id": "lane", "width_m": 30,
                          "centerline": [{"x": -100, "y": 20}, {"x": 200, "y": 20}],
                          "altitude_floor_m": 0, "altitude_ceiling_m": 120}),
                      lambda: api.apply_nfz_edit(sh, "zone", {"op": "translate", "dx": 2.0}),
                      lambda: api.apply_nfz_edit(sh, "zone", {"op": "rotate",
                                                              "angle_deg": 10})]
        for call in calls:
            e = _refused(call, api.ApiError)
            assert e.status == 422, (e.status, e.detail)
        _unchanged(sh, before)


# ------------------------------------------------------- harness and gRPC

def test_a_scenario_event_is_refused_or_recorded_as_refused():
    st = State(x=0.0, y=0.0, up=15.0)
    for make in (_metre, _pointfree):
        sh = Shield(make())
        before = (sh.policy.policy_hash, sh.policy.generation)
        ev = ScenarioEvent(at_sim_t=1.0, type="spawn_polygon_fence",
                           payload={"vertices": SQUARE, "id": "harness-z"})
        _refused(lambda: apply_event(ev, EventState(shields=[sh]), st, (1.0, 0.0), 1.0),
                 HotApplyRefused, "'harness-z'")
        _unchanged(sh, before)
    # on the metre policy: a translate of its zone is refused, and a corridor
    # swap a scenario expects to be refused is recorded as refused, for this
    sh = Shield(_metre())
    before = (sh.policy.policy_hash, sh.policy.generation)
    tr = ScenarioEvent(at_sim_t=1.0, type="translate_polygon_fence",
                       payload={"id": "zone", "dx_m": 2.0, "dy_m": 0.0})
    _refused(lambda: apply_event(tr, EventState(shields=[sh]), st, (1.0, 0.0), 1.0),
             HotApplyRefused)
    expected = ScenarioEvent(at_sim_t=1.0, type="swap_corridor",
                             payload={"target_id": "lane", "width_m": 30.0,
                                      "centerline": [[-100, 20], [200, 20]],
                                      "expect_refused": True})
    es = EventState(shields=[sh])
    rec = apply_event(expected, es, st, (1.0, 0.0), 1.0)
    assert rec.get("refused") is True and WHY in rec["reason"], rec
    assert es.refused and es.refused[0][2] is True, es.refused
    _unchanged(sh, before)


def test_the_grpc_hot_apply_dry_run_refuses_the_body():
    body = {"type": "dynamic_nfz", "id": "grpc-z", **RULE, "margin_m": 1.0,
            "vertices": SQUARE}
    for make in (_metre, _pointfree):
        e = SimpleNamespace(policy=make())
        new = _HotBody(at=1.0, index=0, body=dict(body))
        try:
            Harness._dry_shield(None, e, [], new)
        except HarnessError as err:
            assert err.code == "INVALID_ARGUMENT" and WHY in err.message, err
        else:
            raise AssertionError("the gRPC dry run accepted a metre body with no frame")
    # with a frame the dry run accepts it and hands the runner a rule
    sh, res, rule = Harness._dry_shield(None, SimpleNamespace(policy=_framed()), [],
                                        _HotBody(at=1.0, index=0, body=dict(body)))
    assert rule.id == "grpc-z" and all(v.geographic for v in rule.vertices)


# ------------------------------------------------------------ bundles

def test_derive_writes_no_metres_and_reads_the_recorded_recipe_back():
    (d,) = [x for x in load_lock()["derived"] if x["base"] == "fase3-sim-demo@0.1.1"]
    from guardrail.bundle import as_version
    base = as_version(load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
                      .ir3_form().metre_form(), "0.1.1")
    assert base.frame_origin is None
    _refused(lambda: derive(base, d["hot_apply"], d["generations"]), ValueError)
    back = derive(base, d["hot_apply"], d["generations"], legacy_read=True)
    assert back.point_form == POINTS_LEGACY_METRES
    assert back.hash_form(d["policy_hash"]) is not None, "the recorded recipe no longer rebuilds"


def test_no_bundle_is_written_in_the_metre_form():
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "legacy.tar.gz"
        try:
            write_bundle(_metre(), out, signer=None)
        except ValueError as e:
            assert "canonical WGS84 form only" in str(e), e
        else:
            raise AssertionError("a metre-only policy was bundled")
        assert not out.exists()
        # the CLI says REFUSED and writes nothing
        f = Path(td) / "legacy.yaml"
        import yaml
        f.write_text(yaml.safe_dump(METRE_DOC, sort_keys=False), encoding="utf-8")
        cli = Path(td) / "cli.tar.gz"
        p = subprocess.run([sys.executable, "-X", "utf8", "-m", "guardrail.bundle", str(f),
                            "-o", str(cli), "--unsigned"], cwd=ROOT,
                           capture_output=True, text=True, encoding="utf-8")
        assert p.returncode == 2 and "REFUSED" in p.stdout, (p.returncode, p.stdout, p.stderr)
        assert not cli.exists()
        # a fixture standing for a recorded bundle says so explicitly
        write_bundle(_metre(), out, signer=None, allow_legacy_metres=True)
        assert out.exists()
        # a point-free policy without origin carries no projected coordinate
        write_bundle(_pointfree(), Path(td) / "caps.tar.gz", signer=None)


def _geo_tool():
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        import geo_to_policy
    finally:
        sys.path.remove(str(ROOT / "tools"))
    return geo_to_policy


def test_the_ingest_tool_refuses_metres_with_no_frame_and_converts_with_one():
    G = _geo_tool()
    payload = {"id": "ingest-z", **RULE, "margin_m": 1.0, "vertices": SQUARE,
               "altitude_floor_m": 0, "altitude_ceiling_m": 100}
    for base in (_metre(), _pointfree()):
        _refused(lambda: G.ingest(copy.deepcopy(payload), base), G.ConversionError,
                 "rule(s)", base.policy_id)
    try:
        G.ingest(copy.deepcopy(METRE_DOC))
    except G.ConversionError as e:
        assert "local metres" in str(e) and "convert_policies_wgs84" in str(e), e
    else:
        raise AssertionError("a whole metre document was ingested")
    pol = G.ingest(copy.deepcopy(payload), _framed())
    assert pol.point_form == POINTS_WGS84 and pol.generation == 1


# --------------------------------------------------------- the generator

def test_the_generator_has_no_metre_writer():
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "gen.yaml"
        p = subprocess.run([sys.executable, "-X", "utf8", str(ROOT / "demo" /
                                                              "gen_random_scenario.py"),
                            "--metres", "--out", str(out)], cwd=ROOT,
                           capture_output=True, text=True, encoding="utf-8")
        assert p.returncode == 2 and "--metres was removed" in p.stderr, (p.returncode,
                                                                          p.stderr)
        assert not out.exists()
    src = (ROOT / "demo" / "gen_random_scenario.py").read_text(encoding="utf-8")
    assert "wgs84_policy_yaml(policy_id" in src and "write = policy_yaml" not in src


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
