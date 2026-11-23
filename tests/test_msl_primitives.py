"""X-11b (2026-11-23): MSL on every altitude-bearing primitive, and the
readers that measure height above their home take a declared ground.

The grant's DSL makes MSL "an opt-in field on geometry primitives"
(02-implementation/policy-dsl.md). The review of 23 Nov found two gaps.

WHAT IS PINNED

  (a) Every primitive that carries `altitude_ref` (guardrail/models.py:
      PolygonFence, CircleFence, AltitudeEnvelope, Corridor, DynamicNFZ,
      CorridorSwap) is refused without a ground at the loader and at the
      Shield, and over a declared flat ground judges its MSL band as the AGL
      band it is: a keep-out zone breaches inside its converted band and not
      above it, a keep-in band breaches outside it. The same numbers read as
      AGL decide otherwise, so the conversion is what decided. A DynamicNFZ
      and a CorridorSwap hot-applied with MSL are refused by a Shield with no
      ground and judged in MSL by one with a ground. Over a DEM each type
      converts over the ground's band the safe way (keep-out widens, keep-in
      narrows; terrain.agl_rule).
  (b) The readers that called bundle.load_for_flight with no ground -
      demo/pas_ardupilot/perception_node.py, demo/follow_vlm.py,
      demo/vla_fly.py (Project AirSim) and sitl/run_sitl_demo.py (the
      in-process SITL rail sitl/run_gazebo_demo.sh flies) - now declare one
      explicitly (terrain.declared_home_ground): `--ground scene` on the PAS
      rails (the scene's home-geo-point altitude, terrain.scene_ground),
      `--ground sitl|FILE` on the SITL rail. Without --ground an MSL policy
      is refused before take-off, as on the ROS 2 rail; with it the policy
      loads over the ground, the Shield / CSP / fence ceiling / altitude
      count read it above that ground, and metrics.json records `ground` in
      the ROS 2 node's shape. A DEM, a file on a scene rail and a ground off
      the rail's home are refused. An AGL policy without --ground loads
      exactly as before.

Import-level only: no simulator, no autopilot, no flight.

Run: python tests/test_msl_primitives.py
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "demo" / "pas_ardupilot", ROOT / "sitl",
           ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import terrain                                           # noqa: E402
from guardrail.models import (CorridorSwap, DynamicNFZ, State,          # noqa: E402
                              policy_from_raw)
from guardrail.projection import raw_points_in_wgs84                    # noqa: E402
from guardrail.shield import HotApplyRefused, Shield                    # noqa: E402

H = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
     "violation_action": "project_fix"}
# The Project AirSim scenes' ground (home-geo-point altitude) and frame.
G40 = terrain.FlatGround(40.0, "test: the PAS scenes' 40 m")
FRAME = (35.6895, 139.6917)
SQ = [{"x": 20, "y": -5}, {"x": 30, "y": -5}, {"x": 30, "y": 5}, {"x": 20, "y": 5}]
LINE = [{"x": -50, "y": 40}, {"x": 50, "y": 40}]
DEM = "experiments/worlds/dem_cmac_wgs84.json"
SITL_POLICY = ROOT / "experiments" / "policies" / "msl_dem_sitl.yaml"


def _refused(fn, *needles):
    try:
        fn()
    except (ValueError, SystemExit, HotApplyRefused) as e:
        msg = str(e)
        for n in needles:
            assert n in msg, (n, msg)
        return msg
    raise AssertionError(f"not refused: {needles}")


def _breach(sh, rid, x, y, up) -> bool:
    rows = {r["id"]: r for r in sh.rule_status(State(x=float(x), y=float(y), up=float(up)))}
    return bool(rows[rid]["breach"])


def _agl(rule: dict) -> dict:
    return {k: v for k, v in rule.items() if k != "altitude_ref"}


# One rule per primitive, each with points where its MSL band over 40 m
# ground decides: (x, y, up above ground, breach?). `extra`: rules it needs
# beside it (a corridor_swap's target corridor, declared AGL and wide).
CASES = {
    # 50-160 m MSL over 40 m: a keep-in band of 10-120 m above ground.
    "altitude_envelope": (
        {"id": "r", "type": "altitude_envelope", **H, "alt_min_m": 50.0,
         "alt_max_m": 160.0, "altitude_ref": "MSL"}, [],
        [(0, 0, 5, True), (0, 0, 60, False), (0, 0, 125, True)]),
    # 40-100 m MSL keep-out over 40 m: 0-60 m above ground.
    "polygon_fence": (
        {"id": "r", "type": "polygon_fence", **H, "vertices": SQ,
         "altitude_floor_m": 40.0, "altitude_ceiling_m": 100.0,
         "altitude_ref": "MSL"}, [],
        [(25, 0, 30, True), (25, 0, 70, False)]),
    "circle_fence": (
        {"id": "r", "type": "circle_fence", **H, "center": {"x": -25, "y": 0},
         "radius_m": 5.0, "altitude_floor_m": 40.0, "altitude_ceiling_m": 100.0,
         "altitude_ref": "MSL"}, [],
        [(-25, 0, 30, True), (-25, 0, 70, False)]),
    # 50-100 m MSL keep-in tube over 40 m: 10-60 m above ground.
    "corridor": (
        {"id": "r", "type": "corridor", **H, "centerline": LINE, "width_m": 10.0,
         "altitude_floor_m": 50.0, "altitude_ceiling_m": 100.0,
         "altitude_ref": "MSL"}, [],
        [(0, 40, 30, False), (0, 40, 70, True), (0, 40, 5, True)]),
    # 150-200 m MSL keep-out over 40 m: 110-160 m above ground.
    "dynamic_nfz": (
        {"id": "r", "type": "dynamic_nfz", **H, "vertices": SQ,
         "altitude_floor_m": 150.0, "altitude_ceiling_m": 200.0,
         "altitude_ref": "MSL"}, [],
        [(25, 0, 130, True), (25, 0, 170, False), (25, 0, 100, False)]),
    # 140-200 m MSL replacing tube over 40 m: 100-160 m above ground.
    "corridor_swap": (
        {"id": "r", "type": "corridor_swap", **H, "target_id": "base",
         "centerline": LINE, "width_m": 10.0, "altitude_floor_m": 140.0,
         "altitude_ceiling_m": 200.0, "altitude_ref": "MSL"},
        [{"id": "base", "type": "corridor", **H, "centerline": LINE,
          "width_m": 10.0, "altitude_floor_m": 0.0, "altitude_ceiling_m": 500.0}],
        [(0, 40, 90, True), (0, 40, 110, False), (0, 40, 165, True)]),
}
MSL_TYPES = set(CASES)


def _raw(rule: dict, extra: list, pid: str = "p") -> dict:
    return {"policy_id": pid, "constraints": [*extra, rule]}


def test_every_primitive_with_altitude_ref_is_covered():
    """The cases are every geometry class that declares altitude_ref: a new
    one fails here until it is added."""
    from guardrail import models
    declared = set()
    for name in dir(models):
        cls = getattr(models, name)
        fields = getattr(cls, "model_fields", None)
        if isinstance(fields, dict) and "altitude_ref" in fields and "type" in fields:
            ann = fields["type"].annotation
            for lit in getattr(ann, "__args__", ()):
                if isinstance(lit, str):
                    declared.add(lit)
    assert declared == MSL_TYPES, (declared, MSL_TYPES)


def test_each_primitive_is_refused_without_a_ground():
    for t, (rule, extra, _) in CASES.items():
        raw = _raw(rule, extra)
        _refused(lambda: policy_from_raw(raw), "r: altitude_ref MSL", "ground")
        pol = policy_from_raw(raw, runtime=False)
        _refused(lambda: Shield(pol), "altitude_ref MSL")


def test_each_primitive_judges_its_msl_band_over_a_flat_ground():
    n = 0
    for t, (rule, extra, pts) in CASES.items():
        sh = Shield(policy_from_raw(_raw(rule, extra), ground=G40), ground=G40,
                    escalation=False)
        for x, y, up, want in pts:
            assert _breach(sh, "r", x, y, up) is want, (t, x, y, up, want)
            n += 1
        # The same numbers as AGL are another rule: at least one point flips.
        agl = Shield(policy_from_raw(_raw(_agl(rule), extra)), escalation=False)
        assert any(_breach(agl, "r", x, y, up) is not want for x, y, up, want in pts), t
    assert n == 16, n


def test_the_shield_repairs_an_msl_keep_out_breach_at_its_agl_band():
    """Not only the status row: the decision. A climb into a 150-200 m MSL
    dynamic_nfz over 40 m ground (110-160 m above it) is repaired; the same
    climb under the band is not touched."""
    from guardrail.models import Action4D
    rule, extra, _ = CASES["dynamic_nfz"]
    sh = Shield(policy_from_raw(_raw(rule, extra), ground=G40), ground=G40,
                escalation=False)
    into = sh.filter(State(x=25.0, y=0.0, up=105.0), Action4D(vz_up=3.0))
    assert into.touched and "r" in [v.rule_id for v in into.violations], into
    below = sh.filter(State(x=25.0, y=0.0, up=40.0), Action4D(vz_up=1.0))
    assert not below.touched, below


def test_msl_dynamic_nfz_and_corridor_swap_hot_apply_over_the_shields_ground():
    base_raw = {"policy_id": "hb", "origin": {"lat": 25.0421, "lon": 121.531},
                "constraints": [{"id": "base", "type": "corridor", **H,
                                 "centerline": LINE, "width_m": 10.0,
                                 "altitude_floor_m": 0.0, "altitude_ceiling_m": 500.0}]}
    dz = DynamicNFZ(**{**CASES["dynamic_nfz"][0], "id": "dz",
                       "vertices": [{"x": 20, "y": 35}, {"x": 30, "y": 35},
                                    {"x": 30, "y": 45}, {"x": 20, "y": 45}]})
    sw = CorridorSwap(**{**CASES["corridor_swap"][0], "id": "sw"})
    for ev in (dz, sw):
        _refused(lambda: Shield(policy_from_raw(base_raw)).hot_apply(ev), "MSL")
    sh = Shield(policy_from_raw(base_raw), ground=G40, escalation=False)
    sh.hot_apply(dz)
    assert _breach(sh, "dz", 25, 40, 130) and not _breach(sh, "dz", 25, 40, 170)
    sh.hot_apply(sw)
    assert _breach(sh, "sw", 0, 40, 90) and not _breach(sh, "sw", 0, 40, 110)
    assert _breach(sh, "sw", 0, 40, 165)


def test_each_primitive_converts_over_a_dem_band_the_safe_way():
    """terrain.agl_rule over a (lowest, highest) ground of (30, 50) m MSL:
    a keep-out band grows by the spread, a keep-in band shrinks by it."""
    keep_out = {"polygon_fence", "circle_fence", "dynamic_nfz"}
    for t, (rule, extra, _) in CASES.items():
        pol = policy_from_raw(_raw(rule, extra), runtime=False)
        r = pol.constraints[-1]
        lo_f, hi_f = terrain.band_fields(r)
        got = terrain.agl_rule(r, G40, band=(30.0, 50.0))
        assert got.altitude_ref is None and got.id == "r" and r.altitude_ref == "MSL"
        lo, hi = float(getattr(r, lo_f)), float(getattr(r, hi_f))
        want = (lo - 50.0, hi - 30.0) if t in keep_out else (lo - 30.0, hi - 50.0)
        assert (getattr(got, lo_f), getattr(got, hi_f)) == want, (t, want)
        assert (t in keep_out) == (t in terrain.KEEP_OUT_TYPES), t


# --------------------------------------------------------------------------- #
# The readers (b)
# --------------------------------------------------------------------------- #

def _msl_yaml(d: Path) -> Path:
    """A flight-form MSL policy (WGS84 points about a stated origin): a
    50-160 m MSL envelope and a 40-100 m MSL zone."""
    raw = {"policy_id": "msl-reader", "version": "0.1.0",
           "origin": {"lat": FRAME[0], "lon": FRAME[1]},
           "constraints": raw_points_in_wgs84([CASES["altitude_envelope"][0],
                                               {**CASES["polygon_fence"][0], "id": "z"}],
                                              FRAME)}
    p = d / "msl_reader.yaml"
    p.write_text(json.dumps(raw), encoding="utf-8")
    return p


def _agl_yaml(d: Path) -> Path:
    raw = {"policy_id": "agl-reader", "version": "0.1.0",
           "origin": {"lat": FRAME[0], "lon": FRAME[1]},
           "constraints": raw_points_in_wgs84(
               [_agl({**CASES["altitude_envelope"][0], "alt_min_m": 10.0,
                      "alt_max_m": 120.0})], FRAME)}
    p = d / "agl_reader.yaml"
    p.write_text(json.dumps(raw), encoding="utf-8")
    return p


def _flat_file(d: Path, elev: float) -> Path:
    p = d / f"flat_{elev:g}.json"
    p.write_text(json.dumps({"kind": "flat", "elevation_msl_m": elev,
                             "source": "test flat ground"}), encoding="utf-8")
    return p


def test_declared_home_ground_is_explicit_and_fails_closed():
    D = terrain.declared_home_ground
    scene = ROOT / "demo" / "pas_ardupilot" / "scene_citylife_ardupilot.jsonc"
    assert D(None, scene=scene) == (None, None) and D("", scene=None) == (None, None)
    _refused(lambda: D(None, sha256="0" * 64), "no --ground")
    g, rec = D("scene", scene=scene, root=ROOT)
    assert g.elevation_msl_m == 40.0 and rec is None
    assert g.source == ("home-geo-point altitude of "
                        "demo/pas_ardupilot/scene_citylife_ardupilot.jsonc"), g.source
    _refused(lambda: D("experiments/worlds/flat.json", scene=scene), "scene's own")
    _refused(lambda: D("scene", scene=scene, sha256="0" * 64), "names a ground FILE")
    _refused(lambda: D("scene", scene=None), "loads no Project AirSim scene")
    assert D("sitl", home_msl_m=584.0)[0] is terrain.SITL_CMAC
    _refused(lambda: D("sitl", home_msl_m=40.0), "584 m MSL", "40 m MSL")
    d = Path(tempfile.mkdtemp(prefix="msl_prim_"))
    f584 = _flat_file(d, 584.0)
    sha = terrain.ground_file_sha256(f584)
    g, rec = D(str(f584), sha256=sha, home_msl_m=584.0, yaml_path=SITL_POLICY)
    assert g.elevation_msl_m == 584.0 and rec == {"path": str(f584), "sha256": sha,
                                                   "pinned": True}, rec
    _refused(lambda: D(str(f584), sha256="1" * 64, home_msl_m=584.0,
                       yaml_path=SITL_POLICY), "hashes to")
    _refused(lambda: D(str(_flat_file(d, 100.0)), home_msl_m=584.0,
                       yaml_path=SITL_POLICY), "100 m MSL")
    _refused(lambda: D(DEM, yaml_path=SITL_POLICY, root=ROOT), "is a DEM",
             "ros2_shield_node.py")


def test_the_ground_record_reads_as_the_ros2_nodes_and_the_replay_reads_it():
    from guardrail.replay import run_ground_ref
    assert terrain.home_ground_record(None) is None
    rec = terrain.home_ground_record(G40, None, "--ground scene")
    assert rec == {"ground": G40.record(), "up_ref": "home",
                   "declared": "--ground scene"}, rec
    assert run_ground_ref({"ground": rec}) == {"path": None, "sha256": None, "kind": "flat"}
    f = {"path": "x/flat.json", "sha256": "a" * 64, "pinned": False}
    rec = terrain.home_ground_record(terrain.SITL_CMAC, f, "--ground x/flat.json")
    assert run_ground_ref({"ground": rec}) == {"path": "x/flat.json", "sha256": "a" * 64}


def test_agl_rules_is_the_identity_without_msl_and_fails_closed_without_a_ground():
    pol = policy_from_raw(_raw(_agl(CASES["altitude_envelope"][0]), []))
    rules = pol.constraints
    out = terrain.agl_rules(rules, None)
    assert all(a is b for a, b in zip(out, rules)) and len(out) == len(rules)
    msl = policy_from_raw(_raw(CASES["altitude_envelope"][0], []), runtime=False)
    _refused(lambda: terrain.agl_rules(msl.constraints, None), "altitude_ref MSL")
    got = terrain.agl_rules(msl.constraints, G40)[0]
    assert (got.alt_min_m, got.alt_max_m) == (10.0, 120.0)


def _pas_args(policy: Path, ground=None, **kw):
    return SimpleNamespace(bundle=None, policy=str(policy), dev_yaml=True,
                           allow_unverified_bundle=False, ground=ground, **kw)


def test_the_perception_node_declares_the_scene_ground():
    import perception_node as pn
    d = Path(tempfile.mkdtemp(prefix="msl_prim_pn_"))
    msl = _msl_yaml(d)
    _refused(lambda: pn.flight_policy(_pas_args(msl)), "altitude_ref MSL")
    pol, src, g, rec = pn.flight_policy(_pas_args(msl, "scene"))
    assert g.elevation_msl_m == 40.0 and src["ground"] == g.record(), src
    assert rec["declared"] == "--ground scene" and rec["up_ref"] == "home", rec
    assert "scene_citylife_ardupilot.jsonc" in rec["ground"]["source"]
    _refused(lambda: pn.flight_policy(_pas_args(msl, DEM)), "--ground scene")
    # The fence ceiling is read above home: 160 m MSL over 40 m is 120 m.

    class Link:
        def autopilot_fence(self):
            return {"FENCE_ALT_MAX": 150.0, "FENCE_ENABLE": 1.0, "FENCE_TYPE": 7.0}
    assert pn.autopilot_fence_record(Link(), pol, 8.0, g)["policy_ceiling_m"] == 120.0
    assert pn.fence_ground_kw(None) == {} and pn.fence_ground_kw(g)["home_msl_m"] == 40.0
    # The node's parser declares the option; an AGL policy without it loads
    # exactly as the flight loader loads it.
    assert pn.build_parser().parse_args(["--ground", "scene"]).ground == "scene"
    from guardrail.bundle import load_for_flight
    agl = _agl_yaml(d)
    pol2, src2, g2, rec2 = pn.flight_policy(_pas_args(agl))
    want, want_src = load_for_flight(None, agl)
    assert g2 is None and rec2 is None and "ground" not in src2
    assert pol2.policy_hash == want.policy_hash


def test_the_city_follow_rail_declares_the_scene_ground():
    import follow_vlm as fv
    d = Path(tempfile.mkdtemp(prefix="msl_prim_fv_"))
    msl = _msl_yaml(d)
    a = dict(record_vla=None, vla_hierarchy=None)
    _refused(lambda: fv.city_flight_policy(_pas_args(msl, **a)), "altitude_ref MSL")
    pol, src, g, rec = fv.city_flight_policy(_pas_args(msl, "scene", **a))
    assert g.elevation_msl_m == 40.0 and "scene_semantic.jsonc" in g.source, g
    assert rec["ground"] == g.record() and src["ground"] == g.record()
    _, _, g_vla, _ = fv.city_flight_policy(_pas_args(msl, "scene", record_vla="x",
                                                     vla_hierarchy=None))
    assert "scene_semantic_vla.jsonc" in g_vla.source, g_vla
    _refused(lambda: fv.city_flight_policy(_pas_args(msl, "sitl", **a)), "--ground scene")
    # The flight Shield holds the MSL policy over the ground (and refuses it
    # without), and every forecast arm passes it on.
    _refused(lambda: fv.flight_shield(pol, None), "altitude_ref MSL")
    for kw in ({}, {"grant_forecast": True}, {"legacy_forecast": True}):
        sh = fv.flight_shield(pol, None, ground=g, **kw)
        assert sh.ground is g, kw
        assert _breach(sh, "r", 0, 0, 5) and not _breach(sh, "r", 0, 0, 60)
    # The episode's CSP compiles the MSL rules over the same ground.
    rec_ep = fv.city_episode(d / "ep", pol, 5.0, (0.0, 0.0), "a red car", 30.0, 5.0,
                             ground=g)
    assert rec_ep.ground is g and rec_ep.generations[0]["csp"], rec_ep.generations[0]
    assert fv.build_arg_parser().parse_args(["--ground", "scene"]).ground == "scene"


def test_the_vla_flight_rail_declares_the_scene_ground():
    import vla_fly
    d = Path(tempfile.mkdtemp(prefix="msl_prim_vf_"))
    msl = _msl_yaml(d)
    _refused(lambda: vla_fly.vla_flight_policy(_pas_args(msl, mission="goto")),
             "altitude_ref MSL")
    for mission, scene in (("goto", "scene_vla.jsonc"), ("follow", "scene_vla_det.jsonc")):
        pol, src, g, rec = vla_fly.vla_flight_policy(_pas_args(msl, "scene",
                                                               mission=mission))
        assert g.elevation_msl_m == 40.0 and scene in g.source, (mission, g)
        assert rec["declared"] == "--ground scene" and src["dev_yaml"] is True
    _refused(lambda: vla_fly.vla_flight_policy(_pas_args(msl, DEM, mission="goto")),
             "--ground scene")
    assert vla_fly.build_arg_parser().parse_args(["--ground", "scene"]).ground == "scene"


def test_the_sitl_rail_declares_a_ground_at_its_home():
    import run_sitl_demo as rs
    from guardrail.bundle import load_for_flight
    d = Path(tempfile.mkdtemp(prefix="msl_prim_rs_"))
    msl = _msl_yaml(d)

    def args(ground=None, **kw):
        return SimpleNamespace(ground=ground, ground_sha256=kw.get("sha"),
                               ground_home_msl=kw.get("home"))
    assert rs.flight_ground(args(), None, msl) == (None, None)
    g, rec = rs.flight_ground(args("sitl"), None, msl)
    assert g is terrain.SITL_CMAC and rec["declared"] == "--ground sitl"
    pol, src = load_for_flight(None, msl, ground=g)
    assert src["ground"]["elevation_msl_m"] == 584.0
    f = _flat_file(d, 584.0)
    g, rec = rs.flight_ground(args(str(f), sha=terrain.ground_file_sha256(f)), None, msl)
    assert rec["file"]["pinned"] is True and g.elevation_msl_m == 584.0
    _refused(lambda: rs.flight_ground(args("scene"), None, msl), "loads no Project AirSim")
    _refused(lambda: rs.flight_ground(args(str(_flat_file(d, 40.0))), None, msl),
             "40 m MSL", "584 m MSL")
    g, _ = rs.flight_ground(args(str(_flat_file(d, 40.0)), home=40.0), None, msl)
    assert g.elevation_msl_m == 40.0
    _refused(lambda: rs.flight_ground(args(DEM), None, SITL_POLICY), "is a DEM")
    assert rs.SITL_HOME_MSL_M == 584.0


def test_every_reader_records_its_ground_and_wires_it_to_the_shield():
    """A source check of what a flight cannot show here: each reader passes
    its declared ground to the Shield and writes it into metrics.json."""
    for rel, shield_kw in (("demo/pas_ardupilot/perception_node.py", "**gkw, **({}"),
                           ("demo/follow_vlm.py", "ground=ground, people_source=people_source)"),
                           ("demo/vla_fly.py", '**({} if ground is None else {"ground": ground}),'),
                           ("sitl/run_sitl_demo.py", "clear_response=ARDUCOPTER_GUIDED, **gkw,")):
        src = (ROOT / rel).read_text(encoding="utf-8")
        assert 'metrics["ground"] = ground_rec' in src, rel
        assert shield_kw in src, rel
        assert "agl_rules(policy.by_type(AltitudeEnvelope), ground)" in src, rel
        assert '"--ground"' in src, rel


if __name__ == "__main__":
    import traceback
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    ok = 0
    for name, fn in fns:
        try:
            fn()
            print(f"PASS  {name}")
            ok += 1
        except Exception as e:                                # noqa: BLE001
            print(f"FAIL  {name}\n      {type(e).__name__}: {e}")
            traceback.print_exc()
    print(f"\n{ok}/{len(fns)} passed")
    sys.exit(0 if ok == len(fns) else 1)
