"""X-11 (2026-11-18, wf21): every reader takes a declared ground by name.

The grant's DSL makes MSL "an opt-in field on geometry primitives"
(02-implementation/policy-dsl.md). An MSL rule is enforced over a declared
ground (guardrail/terrain.py); until this date the stress harness's rail
backends, the mission-style node, the GCS backstop step, bundle_origin and
the bag's custom data refused one, a replay bundle named its DEM without
carrying it, the RTL watch judged the leg home at the ground under the
aircraft only, and the autopilot's FENCE_ALT_MAX sat 46 m over the MSL rule.

WHAT IS PINNED

  (a) A ground is NAMED: terrain.ground_file / ground_from_args - its path
      and the sha256 of its file; a file that does not hash to the name is
      refused before it is read; a hash without a file is refused.
  (b) The stress harness flies a scenario's ground on its rails:
      rail_node_args names it (--ground, --ground-sha256, --ground-home-msl
      = the DEM at the start, where sitl_home boots SITL); the Shield node
      and the VLA stub declare every flag; the hil backend sends the same
      for a signed bundle; the post-flight judges are built over the ground
      (an MSL policy cannot be judged without it); the run's own record is
      checked against the declared file. A rotated mission flies over the
      same ground.
  (c) The mission-style node, the GCS step, bundle_origin and
      bag_custom_data each take --ground (and --ground-sha256); the mission
      node judges every leg step at its height above the DEM.
  (d) The replay bundle carries the DEM, checks its sha256 and re-derives
      every terrain row's height from it; a row that disagrees fails it.
  (e) The RTL leg is judged along the ground (rtl_path, THE LEG OVER A DEM).
  (f) FENCE_ALT_MAX is capped by a hard MSL ceiling in force all flight; the
      fence column test reads an MSL zone above home.
  (g) experiments/profiles/msl_dem.yaml: three MSL families (polygon_fence,
      corridor, altitude_envelope) over the DEM, flown headless here, every
      cell passing its gates and labels, and planned on the sitl backend.

Throwaway signing keys only; no rig step runs.

Run: python tests/test_msl_readers.py
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "experiments", ROOT / "tools", ROOT / "tests", ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import terrain                                           # noqa: E402
from guardrail.models import State                                      # noqa: E402
from guardrail.scenario_spec import Library, ScenarioSpec               # noqa: E402

DEM = "experiments/worlds/dem_cmac_wgs84.json"
SITL_POLICY = ROOT / "experiments" / "policies" / "msl_dem_sitl.yaml"
ZONE_POLICY = ROOT / "experiments" / "policies" / "msl_dem_zone.yaml"
CMAC = (-35.363261, 149.16523)
LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")


def _tmp() -> Path:
    return Path(tempfile.mkdtemp(prefix="msl_readers_"))


def _sha() -> str:
    return terrain.ground_file(DEM, root=ROOT)["sha256"]


def _dem():
    return terrain.load_ground(DEM, CMAC, root=ROOT)


def _signer(k: Path):
    from guardrail import bundle as B
    trust = k / "trust.json"
    if B.HAVE_CRYPTO:
        signer = B.generate_signing_key(k / "k.pem", authority="test-lab", trust_store=trust)
    else:
        import os
        signer = B.Signer(authority="test-lab",
                          private_key=B._PyEd25519TestKey(os.urandom(32)))
        B.register_public_key(signer, trust)
    return signer, trust


def _bundle(policy: Path) -> tuple[Path, Path]:
    """(a throwaway-signed bundle of `policy` built over the DEM, its trust store)."""
    import build_flight_bundles as T
    k = _tmp()
    signer, trust = _signer(k)
    out = _tmp()
    rows = T.build([str(policy)], out, signer=signer, trust_store=trust,
                   issued_at="2026-11-18T00:00:00Z", ground=DEM)
    assert rows[0]["ok"], rows
    return Path(rows[0]["bundle"]), trust


# ------------------------------------------------------------------ (a)

def test_a_ground_is_named_by_its_file_and_hash():
    sha = _sha()
    assert sha == terrain.ground_file_sha256(ROOT / DEM)
    rec = terrain.ground_file(DEM, root=ROOT, sha256=sha)
    assert rec == {"path": DEM, "sha256": sha}
    for bad in ("0" * 64, "XYZ"):
        try:
            terrain.ground_file(DEM, root=ROOT, sha256=bad)
        except ValueError as e:
            assert ("hashes to" in str(e)) or ("hex digits" in str(e)), e
        else:
            raise AssertionError(f"a ground named {bad!r} was read")
    ns = argparse.Namespace(ground=DEM, ground_sha256=sha)
    g, f = terrain.ground_from_args(ns, None, SITL_POLICY, root=ROOT)
    assert terrain.is_dem(g) and f == {"path": DEM, "sha256": sha, "pinned": True}
    g2, f2 = terrain.ground_from_args(argparse.Namespace(ground=DEM), None, SITL_POLICY,
                                      root=ROOT)
    assert f2["pinned"] is False and g2.record() == g.record()
    assert terrain.ground_from_args(argparse.Namespace(), None, SITL_POLICY) == (None, None)
    try:
        terrain.ground_from_args(argparse.Namespace(ground=None, ground_sha256=sha), None,
                                 SITL_POLICY, root=ROOT)
    except ValueError as e:
        assert "no --ground" in str(e), e
    else:
        raise AssertionError("a hash with no ground file was accepted")
    # ground_for_flight checks a pin before it reads the file.
    try:
        terrain.ground_for_flight(DEM, yaml_path=SITL_POLICY, root=ROOT, sha256="1" * 64)
    except ValueError as e:
        assert "another file" in str(e), e
    else:
        raise AssertionError("ground_for_flight read a file another hash named")


# ------------------------------------------------------------------ (b)

def _family_cells(fid: str) -> list:
    return list(LIBRARY.by_id()[fid].cells())


def test_the_harness_names_the_ground_to_both_rail_nodes():
    import sweep_scenarios as S
    from guardrail.models import load_policy  # noqa: F401
    import test_railinputs_node as RN
    N = RN._node_module()
    import importlib.util
    spec_v = importlib.util.spec_from_file_location("stub_readers",
                                                    ROOT / "sitl" / "ros2_vla_stub_node.py")
    V = importlib.util.module_from_spec(spec_v)
    spec_v.loader.exec_module(V)
    sha = _sha()
    dem = _dem()
    for cell in _family_cells("msl-dem-zone-ridge") + _family_cells("msl-dem-ceiling-climb"):
        spec = cell.spec
        pol = spec.policy.load(ROOT)
        args = S.rail_node_args(spec, 1001, LIBRARY.defaults, backend="sitl")
        rm = S.rail_mission(spec, pol, LIBRARY.defaults, 1001)
        home = round(dem.elevation_at(rm["start"][0], rm["start"][1]), 3)
        assert f"--ground={DEM}" in args and f"--ground-sha256={sha}" in args, args
        assert f"--ground-home-msl={S._num(home)}" in args, (args, home)
        assert S.rail_home_msl(spec, pol, LIBRARY.defaults, 1001) == home
        # SITL boots on the DEM at the start: the home the nodes are told.
        sh = S.sitl_home(spec, pol, LIBRARY.defaults, 1001)
        assert sh.split(",")[2] == S._num(home), (sh, home)
        # Both nodes read every flag they are sent (none falls to rclpy).
        ns, rest = N.make_parser().parse_known_args(args)
        assert ns.ground == DEM and ns.ground_sha256 == sha
        assert abs(ns.ground_home_msl - home) < 1e-9
        assert not [a for a in rest if a.startswith("--ground")], rest
        vs, _ = V.make_parser().parse_known_args(args)
        assert vs.ground == DEM and vs.ground_sha256 == sha
        pol_v, src = V.stub_policy(vs)
        assert pol_v.policy_hash == pol.policy_hash and "ground" in src
    # A scenario without a ground: no ground flag, SITL at 584 m as before.
    plain = S.rail_node_args(_family_cells("altitude-ceiling-hold")[0].spec, 1001,
                             LIBRARY.defaults, backend="sitl")
    assert not [a for a in plain if a.startswith("--ground")], plain


def test_the_hil_backend_sends_the_ground_with_the_signed_bundle():
    import sweep_scenarios as S
    from guardrail import bundle as B  # noqa: F401
    path, _trust = _bundle(ZONE_POLICY)
    bdir = path.parent
    cell = _family_cells("msl-dem-zone-ridge")[0]
    args = S.rail_node_args(cell.spec, 1001, LIBRARY.defaults, backend="hil",
                            bundles_dir=bdir)
    assert "--bundle" in args and f"bundles/{path.name}" in args, args
    assert f"--ground={DEM}" in args and f"--ground-sha256={_sha()}" in args, args


def _rows_of(spec):
    import sweep_scenarios as S
    r = S.run_episode(spec, LIBRARY.defaults, 1001)
    return r


def test_the_post_flight_judges_score_over_the_ground():
    """The harness's rows of a ground episode (the rail's row shape, `up`
    above the DEM) scored by sitl_extra: every judge is built over the
    ground, and an MSL policy cannot be judged without it."""
    import sweep_scenarios as S
    cell = next(c for c in _family_cells("msl-dem-corridor-west") if c.params["vz"] == 1.0)
    spec = cell.spec
    r = _rows_of(spec)
    metrics = {"ground": {"file": terrain.ground_file(DEM, root=ROOT)}}
    extra = S.sitl_extra(spec, r.rows, metrics, r.policy, None, max_s=None, dt=0.1)
    assert extra["ground_check"]["match"] is True, extra["ground_check"]
    assert extra.get("breaches") == 0, extra.get("breaches")
    assert "rail_measurements_not_measured" not in extra
    # The rule judges were built over the ground and read every row: the
    # corridor offset is theirs (Shield.rule_status on the MSL corridor).
    assert "rail_rule_basis" in extra and extra["max_offset_from_corridor"] is not None
    assert extra["max_offset_from_corridor"] <= 15.0, extra["max_offset_from_corridor"]
    assert "MSL" not in json.dumps(extra.get("rail_not_measured") or {})
    # Without the ground the judges cannot even be built: the MSL rule.
    try:
        S.rail_rule_measurements(r.rows, r.policy, None)
    except ValueError as e:
        assert "MSL" in str(e), e
    else:
        raise AssertionError("an MSL policy was judged with no ground")
    # A run whose record names another file is flagged, never matched.
    other = {"ground": {"file": {"path": DEM, "sha256": "0" * 64}}}
    assert S.rail_ground_check(spec, other)["match"] is False
    assert S.rail_ground_check(spec, {})["flown"] is None


def test_a_rotated_mission_flies_over_the_same_ground():
    import sweep_scenarios as S
    cell = next(c for c in _family_cells("msl-dem-ceiling-climb") if c.params["vy"] == -3.0)
    raw = cell.spec.model_dump(mode="json", by_alias=True)
    raw["mission"] = {**raw["mission"], "rotate_deg": 90.0}
    rot = ScenarioSpec.model_validate(raw)
    assert rot.ground == DEM and rot.policy._ground == DEM
    r = S.run_episode(rot, LIBRARY.defaults, 1001)
    sc = S.score(rot, r.rows, r.extra, r.policy, r.priorities)
    assert sc["p0_violation_escape_rate"] == 0.0 and sc["ended_safe"] is True, sc
    # Turned 90 deg about the start, the climb heads north, not west.
    far = max(r.rows, key=lambda x: math.hypot(x["x"], x["y"]))
    assert abs(far["x"]) > abs(far["y"]), far
    assert max(x["msl_m"] for x in r.rows) <= 624.0 + 1e-6


# ------------------------------------------------------------------ (c)

def test_the_mission_node_judges_each_leg_step_over_the_dem():
    import test_sitl_rails as TSR
    RN, M = TSR._mission_standin()
    sha = _sha()
    args, _ = M.make_parser().parse_known_args(
        ["--policy", str(SITL_POLICY), "--dev-yaml", "--waypoints", "0,90",
         "--ground", DEM, "--ground-sha256", sha, "--ground-home-msl", "584"])
    out = _tmp()
    node = M.MissionShieldNode(args, out)
    assert node.terrain is not None and node.shield.ground is node.ground
    assert node.ground_file == {"path": DEM, "sha256": sha, "pinned": True}
    assert node.policy.policy_id == "msl-dem-sitl"
    # 15 m above home east over the rising ground: under the AGL floor at
    # about 60 m east (595.6 m MSL there: 3.4 m above it).
    v = M.leg_check(node.shield, (0.0, 0.0, 15.0), (0.0, 90.0, 15.0), terrain=node.terrain)
    assert v["legal"] is False and "agl-band" in v["rules"], v
    assert 30.0 < v["at_m"] < 60.0, v
    # Without the conversion the same heights read as above the ground at
    # each step: the leg looks legal (what the node did before wf21).
    assert M.leg_check(node.shield, (0.0, 0.0, 15.0), (0.0, 90.0, 15.0))["legal"] is True
    wps, legs = M.check_plan(node.shield, (0.0, 0.0, 15.0), [(0.0, 90.0, 15.0)],
                             terrain=node.terrain)
    assert legs[0]["legal"] is False and wps[0]["hold_s"] == M.HOLD_S
    # Refused: a hash that does not name the file, a home off the DEM.
    for bad in (["--ground-sha256", "0" * 64], ["--ground-home-msl", "590"]):
        a, _ = M.make_parser().parse_known_args(
            ["--policy", str(SITL_POLICY), "--dev-yaml", "--ground", DEM] + bad)
        try:
            M.mission_ground(a, None, SITL_POLICY)
        except ValueError:
            pass
        else:
            raise AssertionError(f"the mission node took {bad}")
    a, _ = M.make_parser().parse_known_args(["--policy", str(SITL_POLICY), "--dev-yaml"])
    assert M.mission_ground(a, None, SITL_POLICY) == (None, None, None, {})


def test_the_gcs_takes_the_ground_and_checks_home():
    import gcs_backstop as G
    import mavlink_adapter_node as A
    path, trust = _bundle(SITL_POLICY)
    sha = _sha()
    ns = argparse.Namespace(bundle=str(path), ground=DEM, ground_sha256=sha,
                            ground_home_msl=584.0)
    ground, rec, gkw = G.gcs_ground(ns)
    assert terrain.is_dem(ground) and rec["file"]["sha256"] == sha
    assert gkw == {"ground": ground, "home_msl_m": 584.0}
    from guardrail.bundle import load_for_flight
    pol, _ = load_for_flight(str(path), None, ground=ground, allow_unverified=True)
    parm = A.read_parm(ROOT / "sitl" / "fence" / "guardrail_fence.parm")
    vals, plan = G.desired_params(parm, pol, 15.0, gkw)
    # The MSL ceiling caps the column: 624 - 584 = 40 m above home, + 5.
    assert plan["policy_ceiling_m"] == 40.0 and vals["FENCE_ALT_MAX"] == 45.0, plan
    assert G.home_ground_problem(ground, 584.0, {"alt_m": 584.3}, (0.0, 0.0)) is None
    assert "home altitude" in G.home_ground_problem(ground, 584.0, {"alt_m": 590.0},
                                                    (0.0, 0.0))
    assert "DEM's elevation at home" in G.home_ground_problem(
        ground, 584.0, {"alt_m": 584.0}, (0.0, 100.0))
    try:
        G.gcs_ground(argparse.Namespace(bundle=str(path), ground=DEM, ground_sha256=sha,
                                        ground_home_msl=None))
    except ValueError as e:
        assert "--ground-home-msl" in str(e), e
    else:
        raise AssertionError("the GCS took a DEM home elevation it was not given")
    assert G.gcs_ground(argparse.Namespace(bundle=str(path), ground=None,
                                           ground_home_msl=None)) == (None, None, {})
    _ = trust


def test_bundle_origin_reads_an_msl_bundle_only_over_its_named_ground():
    import bundle_origin as BO
    path, _ = _bundle(SITL_POLICY)
    # Without a ground the bundle is refused (exit 2), as before.
    rc_none = BO.main([str(path)])
    assert rc_none == 2
    # The throwaway key is not in the lab trust store, so load_for_flight
    # would refuse the bundle as unverified whatever its ground: the gate is
    # read leniently on that one point, for this test only.
    from guardrail import bundle as B
    saved = B.load_for_flight

    def lenient(bundle, yaml_path=None, **kw):
        kw.setdefault("allow_unverified", True)
        return saved(bundle, yaml_path, **kw)
    B.load_for_flight = lenient
    try:
        try:
            BO.origin_of(str(path))
        except ValueError as e:
            assert "MSL" in str(e), e
        else:
            raise AssertionError("bundle_origin read an MSL bundle with no ground")
        o = BO.origin_of(str(path), DEM, _sha())
        assert o == f"{CMAC[0]!r},{CMAC[1]!r}", o
        assert BO.main([str(path), "--ground", DEM, "--ground-sha256", _sha()]) == 0
        assert BO.main([str(path), "--ground", DEM, "--ground-sha256", "0" * 64]) == 2
    finally:
        B.load_for_flight = saved
    assert BO.main([]) == 64 and BO.main(["a", "b"]) == 64


def test_the_bag_custom_data_loads_the_policy_over_the_named_ground():
    import bag_custom_data as CD
    sha = _sha()
    base = ["--shield", "on", "--seed", "3", "--policy", str(SITL_POLICY), "--dev-yaml"]
    fields, notes = CD.launch_fields(base + ["--ground", DEM, "--ground-sha256", sha])
    assert notes == [], notes
    from guardrail.bundle import load_for_flight
    g = terrain.ground_for_flight(DEM, yaml_path=SITL_POLICY, root=ROOT)
    assert fields[CD.POLICY_HASH_KEY] == load_for_flight(None, SITL_POLICY, ground=g)[0].policy_hash
    assert fields[CD.GROUND_SHA256_KEY] == sha
    # Before wf21 (no ground read): the hash was left out with a note.
    f0, n0 = CD.launch_fields(base)
    assert CD.POLICY_HASH_KEY not in f0 and any("MSL" in n for n in n0), (f0, n0)
    # A wrong hash: refused, said, never recorded.
    f1, n1 = CD.launch_fields(base + ["--ground", DEM, "--ground-sha256", "0" * 64])
    assert CD.POLICY_HASH_KEY not in f1 and CD.GROUND_SHA256_KEY not in f1, f1
    assert CD.GROUND_SHA256_KEY not in CD.KEYS


# ------------------------------------------------------------------ (d)

def _flown_run():
    """A hand-ticked GUIDED flight of the SITL MSL profile on the stand-in
    ROS stack (test_msl_rail's), with --ground named by its hash: its run
    folder, with the replay bundle the node wrote at the end."""
    import test_msl_rail as TM
    RN, N, node, clk, out = TM._node([f"--ground={DEM}", f"--ground-sha256={_sha()}",
                                      "--ground-home-msl=584", "--start=0,0,20",
                                      "--no-target", "--max-s", "0.45"])
    pose = State(x=0.0, y=0.0, up=20.0)
    node._on_pose(RN._pose_msg(N, pose))
    node.start_mission()
    for _ in range(5):
        node._on_pose(RN._pose_msg(N, pose))
        RN._act(N, node, vx=0.0)
        clk.t += 0.1
        node._tick()
    assert node.done
    return node, out


def test_the_replay_bundle_carries_the_dem_and_re_derives_from_it():
    from guardrail.replay import read_replay, replay_ground, verify_replay, write_replay
    node, out = _flown_run()
    m = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert m["ground"]["file"] == {"path": DEM, "sha256": _sha(), "pinned": True}
    rb = out / f"{out.name}.replay.tar.gz"
    assert rb.is_file()
    got = read_replay(rb)
    idx = got["index"]["ground"]
    assert idx == {"carried": True, "path": DEM, "sha256": _sha(),
                   "member": "ground/dem_cmac_wgs84.json"}, idx
    assert got["ground_bytes"] == (ROOT / DEM).read_bytes()
    g = replay_ground(got)
    assert g.record()["wgs84_dem"]["sha256"] == _sha()
    ok, notes = verify_replay(rb)
    assert ok, notes
    assert any("row height(s) above it re-derived" in n for n in notes), notes
    # A row whose height is not its MSL height less the carried DEM fails.
    bad = _tmp() / "run"
    shutil.copytree(out, bad, ignore=shutil.ignore_patterns("*.replay.tar.gz", "_previous"))
    rows = [json.loads(x) for x in (bad / "flight_log.jsonl").read_text(
        encoding="utf-8").splitlines() if x.strip()]
    rows[1]["up"] = round(rows[1]["up"] + 0.5, 3)
    (bad / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    rb2 = write_replay(bad, node.policy, bad / "x.replay.tar.gz", signer=None)
    ok2, why2 = verify_replay(rb2)
    assert not ok2 and "does not re-derive" in why2[0], why2
    # A run whose named ground is missing or another file: no bundle.
    m2 = dict(m, ground=dict(m["ground"], file={"path": DEM, "sha256": "0" * 64}))
    (bad / "metrics.json").write_text(json.dumps(m2), encoding="utf-8")
    try:
        write_replay(bad, node.policy, bad / "y.replay.tar.gz", signer=None)
    except ValueError as e:
        assert "not the ground the run flew over" in str(e), e
    else:
        raise AssertionError("a bundle was written around another ground")
    try:
        write_replay(bad, node.policy, bad / "z.replay.tar.gz", signer=None,
                     ground_root=_tmp())
    except ValueError as e:
        assert "must carry the ground" in str(e), e
    else:
        raise AssertionError("a bundle was written without its ground")


def test_a_run_without_a_ground_packs_as_before():
    from guardrail.replay import _ground_member, run_ground_ref
    assert run_ground_ref({}) is None and run_ground_ref(None) is None
    assert _ground_member({"episode_id": 1}, ROOT) == (None, {})
    # A run flown before the file record (msl_dem_sitl_0x): the DEM's own.
    old = {"ground": {"ground": {"wgs84_dem": {"path": DEM, "sha256": _sha()}},
                      "up_ref": "terrain"}}
    assert run_ground_ref(old) == {"path": DEM, "sha256": _sha()}
    flat = {"ground": {"ground": {"kind": "flat", "elevation_msl_m": 584.0, "source": "x"},
                       "up_ref": "home"}}
    entry, mem = _ground_member(flat, ROOT)
    assert entry["carried"] is False and mem == {}


def test_the_stored_sitl_rows_re_derive_from_the_dem():
    """msl_dem_sitl_01's committed rows: every height re-derives from the
    DEM within the replay's tolerance (what verify_replay checks)."""
    from guardrail.replay import GROUND_ROW_TOL_M, _ground_rows_problem
    with gzip.open(ROOT / "tests" / "fixtures" / "msl_dem_sitl_01_rows.slim.jsonl.gz",
                   "rt", encoding="utf-8") as fh:
        rows = [dict(json.loads(x), up_ref="terrain") for x in fh if x.strip()]
    n, bad = _ground_rows_problem(rows, _dem())
    assert bad is None and n == len(rows) == 170, (n, bad)
    assert GROUND_ROW_TOL_M == 2.5e-3


# ------------------------------------------------------------------ (e)

def test_the_rtl_leg_is_judged_along_the_ground():
    from guardrail import rtl_path as RP
    from guardrail.replay import rail_shield
    from guardrail.bundle import load_for_flight
    g = _dem()
    pol, _ = load_for_flight(None, SITL_POLICY, ground=g)
    sh = rail_shield(pol, lookahead_s=5.0, dt=0.1, ground=g, up_ref="terrain")
    # Beyond the ridge, 170 m east, 30 m above home (614 m MSL): 25 m above
    # the ground there. The leg home at 614 m MSL crosses the crest at
    # 602.75 m: 11.25 m above it, inside agl-band - clear.
    x, y = 0.0, 170.0
    up = 614.0 - g.elevation_at(x, y)
    st = State(x=x, y=y, up=up)
    zones, bands, over = RP.leg_rules_from_shield(sh, st)
    assert over is not None and set(over.msl) == {"msl-ceiling"}
    assert over.msl["msl-ceiling"] == (590.0, 624.0)
    assert {b.rule_id for b in bands} == {"msl-ceiling", "agl-band"}
    leg = RP.check_leg(x, y, up, (0.0, 0.0), zones, bands, 15.0 + up - 30.0, over=over)
    assert leg.clear, leg.reasons
    # 22 m above home (606 m MSL) the same leg passes the crest 3.25 m above
    # it: under the 8 m floor there, though 19 m above the ground at the
    # aircraft. The check at the aircraft alone (before wf21) calls it clear.
    up2 = 606.0 - g.elevation_at(x, y)
    st2 = State(x=x, y=y, up=up2)
    z2, b2, o2 = RP.leg_rules_from_shield(sh, st2)
    leg2 = RP.check_leg(x, y, up2, (0.0, 0.0), z2, b2, 15.0 + up2 - 22.0, over=o2)
    assert not leg2.clear and leg2.blocking == ("agl-band",), leg2
    assert "above the ground at" in leg2.reasons[0], leg2.reasons
    old = RP.check_leg(x, y, up2, (0.0, 0.0), RP.zones_from_shield(sh),
                       RP.band_from_shield(sh, st2), 15.0 + up2 - 22.0)
    assert old.clear, old.reasons
    # The MSL band is judged in MSL at the leg's level: a return altitude
    # above 624 m MSL is blocked by msl-ceiling, wherever the ground.
    up3 = 625.0 - g.elevation_at(x, y)
    z3, b3, o3 = RP.leg_rules_from_shield(sh, State(x=x, y=y, up=up3))
    leg3 = RP.check_leg(x, y, up3, (0.0, 0.0), z3, b3, 0.0, over=o3)
    assert "msl-ceiling" in leg3.blocking and "m MSL is outside" in " ".join(leg3.reasons)
    # The RTLWatch hands `over` through; without a DEM nothing changes.
    w = RP.RTLWatch(home=(0.0, 0.0))
    v = w.assess(0.0, x, y, up2, z2, b2, up_home=22.0, over=o2)
    assert v.failed and v.leg.blocking == ("agl-band",)
    from guardrail.models import load_policy
    flat = rail_shield(load_policy(ROOT / "policies" / "sim_demo_policy.yaml"),
                       lookahead_s=5.0, dt=0.1)
    s0 = State(x=1.0, y=2.0, up=15.0)
    zf, bf, of = RP.leg_rules_from_shield(flat, s0)
    assert of is None and bf == RP.band_from_shield(flat, s0)
    assert [z.rule_id for z in zf] == [z.rule_id for z in RP.zones_from_shield(flat)]


def test_an_msl_zone_crossed_by_the_leg_is_judged_in_msl():
    from guardrail import rtl_path as RP
    from guardrail.replay import rail_shield
    from guardrail.bundle import load_for_flight
    g = _dem()
    pol, _ = load_for_flight(None, ZONE_POLICY, ground=g)
    sh = rail_shield(pol, lookahead_s=5.0, dt=0.1, ground=g, up_ref="terrain")
    # 180 m east, home behind the zone: a leg at 610 m MSL crosses
    # msl-ridge-nfz (606-700 m MSL), one at 603 m MSL passes under it but
    # under the AGL floor over the crest.
    for msl, want in ((610.0, "msl-ridge-nfz"), (603.0, "agl-band")):
        x, y = 0.0, 180.0
        up = msl - g.elevation_at(x, y)
        z, b, o = RP.leg_rules_from_shield(sh, State(x=x, y=y, up=up))
        leg = RP.check_leg(x, y, up, (0.0, 0.0), z, b, 0.0, over=o)
        assert want in leg.blocking, (msl, leg)


# ------------------------------------------------------------------ (f)

def test_the_backstop_is_capped_by_an_msl_ceiling_in_force_all_flight():
    import mavlink_adapter_node as A
    from guardrail.bundle import load_for_flight
    g = _dem()
    pol, _ = load_for_flight(None, SITL_POLICY, ground=g)
    c, src = A.policy_ceiling_m(pol, 15.0, ground=g, home_msl_m=584.0)
    assert c == 40.0 and "capped by msl-ceiling" in src
    assert A.fence_alt_plan(c, 30.0)["raise_to"] == 45.0
    # A soft MSL ceiling, or one a clock can switch off, does not cap it.
    top = terrain.ground_band(g)[1]
    for upd in ({"constraint_type": "soft"},
                {"valid_time": {"recurrence": {"days": ["Mon"], "start_time": "07:00",
                                               "end_time": "08:00"}}}):
        raw = pol.model_dump(mode="json", by_alias=True, exclude_none=True)
        for cc in raw["constraints"]:
            if cc["id"] == "msl-ceiling":
                cc.update(upd)
        from guardrail.models import policy_from_raw
        p2 = policy_from_raw(raw, source="t", runtime=False)
        c2, src2 = A.policy_ceiling_m(p2, 15.0, ground=g, home_msl_m=584.0)
        assert abs(c2 - (60.0 + top - 584.0)) < 1e-9 and "capped" not in src2, (upd, c2)
    # Without a ground: as before.
    assert A.policy_ceiling_m(pol, 15.0) == (624.0, "altitude_envelope")


def test_the_fence_column_reads_an_msl_zone_above_home():
    import mavlink_adapter_node as A
    from guardrail.fence_export import policy_zones
    from guardrail.bundle import load_for_flight
    g = _dem()
    pol, _ = load_for_flight(None, ZONE_POLICY, ground=g)
    zones, skipped = policy_zones(pol, column_m=45.0, ground=g, home_msl_m=584.0)
    assert zones == [] and skipped[0]["id"] == "msl-ridge-nfz"
    assert "22-116 m above home" in skipped[0]["why"], skipped
    # An MSL zone from below the ground to above the column is exported.
    raw = pol.model_dump(mode="json", by_alias=True, exclude_none=True)
    for cc in raw["constraints"]:
        if cc["id"] == "msl-ridge-nfz":
            cc.update(altitude_floor_m=500, altitude_ceiling_m=700)
    from guardrail.models import policy_from_raw
    p2 = policy_from_raw(raw, source="t", runtime=False)
    f2, _ = A.policy_fences(p2, column_m=45.0, ground=g, home_msl_m=584.0)
    assert [z["id"] for z in f2] == ["msl-ridge-nfz"]
    # Read without the ground the same 500-700 band is MSL metres compared
    # with a column above home: 500 > 0, never exported (the old reading).
    assert A.policy_fences(p2, column_m=45.0)[0] == []
    # Over a flat ground at home's elevation an AGL zone reads as before.
    from guardrail.models import load_policy
    sim = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    flat = terrain.FlatGround(584.0, "t")
    assert policy_zones(sim, column_m=30.0, ground=flat, home_msl_m=584.0)[0] == \
        policy_zones(sim, column_m=30.0)[0]


# ------------------------------------------------------------------ (g)

def test_the_msl_dem_profile_flies_three_msl_geometries_over_the_dem():
    import sweep_scenarios as S
    from guardrail.scenario_spec import expand, load_profile
    prof = load_profile(ROOT / "experiments" / "profiles" / "msl_dem.yaml")
    eps, _how = expand(LIBRARY, prof)
    assert sorted(e.cell.episode_id for e in eps) == sorted(
        c.episode_id for f in ("msl-dem-zone-ridge", "msl-dem-corridor-west",
                               "msl-dem-ceiling-climb") for c in _family_cells(f))
    assert len(eps) == 6
    # Left out of every other run (the families name their profile).
    assert not [e for e in expand(LIBRARY)[0] if e.cell.spec.scenario_id.startswith("msl-dem-")]
    fams = ("msl-dem-zone-ridge", "msl-dem-corridor-west", "msl-dem-ceiling-climb")
    types = {}
    for fid in fams:
        for cell in _family_cells(fid):
            spec = cell.spec
            assert spec.ground == DEM and spec.profiles == ["msl_dem"], fid
            pol = spec.policy.load(ROOT)
            msl = [c for c in pol.constraints if terrain.is_msl(c)]
            assert msl, fid
            types.setdefault(fid, set()).update(c.type for c in msl)
            r = S.run_episode(spec, LIBRARY.defaults, 1001)
            sc = S.score(spec, r.rows, r.extra, r.policy, r.priorities)
            assert sc["p0_violation_escape_rate"] == 0.0 and sc["ended_safe"] is True, \
                (cell.episode_id, sc)
            assert sc.get("failsafe_matches_label") in (1.0, None), (cell.episode_id, sc)
            assert sc.get("outcome_matches_label") is True, (cell.episode_id,
                                                             sc.get("mission_outcome"))
    assert types == {"msl-dem-zone-ridge": {"polygon_fence"},
                     "msl-dem-corridor-west": {"corridor"},
                     "msl-dem-ceiling-climb": {"altitude_envelope"}}, types
    # Every mission is planned on the sitl backend (nothing refused).
    for e in eps:
        S.rail_node_args(e.cell.spec, e.seed, LIBRARY.defaults, backend="sitl")
    text = (ROOT / "experiments" / "profiles" / "msl_dem.yaml").read_text(encoding="utf-8")
    assert "families: [msl-dem-zone-ridge, msl-dem-corridor-west, msl-dem-ceiling-climb]" \
        in text and "expected_cells: [6, 6]" in text


TESTS = [v for k, v in sorted(globals().items()) if k.startswith("test_")]

if __name__ == "__main__":
    ok = 0
    for t in TESTS:
        try:
            t()
        except Exception as e:                                          # noqa: BLE001
            import traceback
            traceback.print_exc()
            print(f"FAIL  {t.__name__}: {type(e).__name__}: {e}")
        else:
            ok += 1
            print(f"PASS  {t.__name__}")
    print(f"{ok}/{len(TESTS)} passed")
    sys.exit(0 if ok == len(TESTS) else 1)
