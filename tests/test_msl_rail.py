"""X-11 (2026-11-18): an MSL rule on a rail - a WGS84 DEM, the headless
harness's opt-in ground, and the ROS 2 Shield node's --ground.

WHAT IS PINNED

  (a) guardrail/terrain.py reads a DEM on a WGS84 lat/lon grid (JSON or NPZ,
      Wgs84Grid) and places it in a policy's frame (to_ground): the resampled
      surface agrees with the source within the bound it records, its origin
      is the policy's, and a grid with no frame to go to is refused.
      TerrainReference turns a height above home into a height above the DEM
      and refuses a home elevation the DEM disagrees with.
  (b) The headless harness (experiments/sweep_scenarios.py) takes a scenario's
      `ground`: the policy loads with it (an MSL rule passes the flight
      gate), every Shield gets it with up_ref="terrain", the vehicle holds
      its MSL height. Over a FLAT DEM an AGL scenario flies the same rows as
      without a ground (the plumbing is neutral); without a ground nothing
      is added to the rows, events or record. Over the CMAC DEM the SITL
      profile's MSL rule binds and no tick breaks it (oracle without a
      Shield). The SITL / hil rails refuse a scenario ground (their
      post-flight scoring has none).
  (c) sitl/ros2_shield_node.py --ground on the stand-in ROS stack: the MSL
      policy is refused without it, flown with it (the Shield told the
      height above the DEM, the row carrying the pose's height above home
      and the MSL height, metrics.json the ground); a home elevation the DEM
      disagrees with is refused; without --ground the Shield is handed the
      pose object itself. The VLA stub and tools/build_flight_bundles.py
      take the same ground.

  (d) After review (2026-11-18): with --ground the node's FENCE_ALT_MAX
      ceiling is a height above HOME (an MSL ceiling less home, an AGL one
      plus the ground's highest less home; 624 m MSL was being handed on as
      624 m above home, raise to 629), metrics alt_violation_s judges an MSL
      envelope in MSL (a clean flight is 0, not every tick), and the
      AerialVLA bridge takes --ground as the stub does.

Byte-identity of every flight without a ground (141 library episodes of the
profiles smoke, distance_envelope, fsm_edges, grant_example and prefix_ab,
base 7b0f50c and this branch, both digest sets stored) is
docs/data/msl_rail_identity_20261118.json; this file pins the mechanisms. Throwaway signing keys only; no rig step runs.

Run: python tests/test_msl_rail.py
"""
from __future__ import annotations

import json
import math
import random
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "experiments", ROOT / "tools", ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import terrain                                           # noqa: E402
from guardrail.models import State, load_policy                         # noqa: E402
from guardrail.projection import projection_for                         # noqa: E402
from guardrail.scenario_spec import Library, ScenarioSpec, expand       # noqa: E402

DEM = "experiments/worlds/dem_cmac_wgs84.json"
POLICY = ROOT / "experiments" / "policies" / "msl_dem_sitl.yaml"
CMAC = (-35.363261, 149.16523)
LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")


def _tmp() -> Path:
    return Path(tempfile.mkdtemp(prefix="msl_rail_"))


# ------------------------------------------------------------------ (a)

def test_the_cmac_dem_file_is_the_tool_s_ridge():
    import msl_dem_rail as M
    grid = terrain.load_wgs84_grid(ROOT / DEM)
    assert grid.shape == (M.NR, M.NC) and grid.dlat == grid.dlon == M.DDEG
    assert grid.to_dict()["elev_msl_m"] == M.cmac_grid().to_dict()["elev_msl_m"]
    assert abs(grid.elevation_at_latlon(*CMAC) - 584.0) < 0.01


def test_a_wgs84_grid_is_placed_in_the_policy_frame_within_its_bound():
    g = terrain.load_ground(DEM, CMAC, root=ROOT)
    grid = terrain.load_wgs84_grid(ROOT / DEM)
    rec = g.record()
    geo = rec["wgs84_dem"]
    assert g.origin == CMAC and rec["kind"] == "grid" and geo["kind"] == "wgs84_grid"
    assert geo["path"] == DEM and len(geo["sha256"]) == 64
    sn, se = geo["spacing_m"]
    assert abs(sn - 11.131949) < 1e-5 and abs(se - 9.078094) < 1e-5, geo
    assert geo["resample_cell_m"] == round(min(sn, se), 6)
    bound = geo["resample_bound_m"]
    assert 0 < geo["resample_max_err_sampled_m"] <= bound
    p = projection_for(*CMAC)
    rng = random.Random(20261118)
    worst = 0.0
    for _ in range(3000):
        x, y = rng.uniform(-495.0, 495.0), rng.uniform(-495.0, 495.0)
        lat, lon = p.to_latlon(x, y)
        worst = max(worst, abs(g.elevation_at(x, y) - grid.elevation_at_latlon(lat, lon)))
    assert worst <= bound, (worst, bound)
    print(f"  resampled vs source: worst {worst:.4f} m over 3000 points, bound {bound:.3f} m")
    # The ground goes to the policy's frame: the Shield's origin check passes.
    assert terrain.origin_problem(g, CMAC) is None
    assert terrain.origin_problem(g, (25.0, 121.5)) is not None


def test_npz_and_json_read_the_same_grid_and_bad_files_are_refused():
    import numpy as np
    grid = terrain.load_wgs84_grid(ROOT / DEM)
    d = _tmp()
    np.savez(d / "g.npz", lat0=grid.lat0, lon0=grid.lon0, dlat_deg=grid.dlat,
             dlon_deg=grid.dlon, elev_msl_m=grid.z, source=np.array(grid.source))
    g2 = terrain.load_wgs84_grid(d / "g.npz")
    assert (g2.z == grid.z).all() and (g2.lat0, g2.lon0) == (grid.lat0, grid.lon0)
    a = terrain.load_ground(d / "g.npz", CMAC)
    b = terrain.load_ground(DEM, CMAC, root=ROOT)
    assert (a.z == b.z).all() and (a.x0, a.y0, a.cell_m) == (b.x0, b.y0, b.cell_m)
    # No frame to place it in (a metre-only policy): refused, not guessed.
    try:
        terrain.load_ground(DEM, None, root=ROOT)
    except ValueError as e:
        assert "frame" in str(e), e
    else:
        raise AssertionError("a WGS84 grid was placed with no frame origin")
    bad = d / "bad.json"
    for doc, why in (({"kind": "wgs84_grid", "lat0": 0, "lon0": 0, "dlat_deg": 0,
                       "dlon_deg": 0.1, "elev_msl_m": [[1, 2], [3, 4]], "source": "s"}, "dlat"),
                     ({"kind": "wgs84_grid", "lat0": 0, "lon0": 0, "dlat_deg": 0.1,
                       "dlon_deg": 0.1, "elev_msl_m": [[1, 2], [3, 4]], "source": "s",
                       "zone": 1}, "unknown"),
                     ({"kind": "tif"}, "not a ground file")):
        bad.write_text(json.dumps(doc), encoding="utf-8")
        try:
            terrain.load_ground(bad, CMAC)
        except ValueError as e:
            assert why in str(e), (why, e)
        else:
            raise AssertionError(f"accepted a bad ground file ({why})")
    # A metre grid (the existing synthetic ridge, no origin) reads as it is
    # for a metre-only policy and is refused under a framed one.
    ridge = "experiments/worlds/dem_ridge.json"
    assert terrain.load_ground(ridge, None, root=ROOT).origin is None
    try:
        terrain.load_ground(ridge, CMAC, root=ROOT)
    except ValueError as e:
        assert "frame" in str(e), e
    else:
        raise AssertionError("a metre-only DEM was read under a framed policy")


def test_terrain_reference_converts_and_checks_home():
    g = terrain.load_ground(DEM, CMAC, root=ROOT)
    tr = terrain.TerrainReference(g, 584.0, (0.0, 0.0), "SITL --home")
    assert abs(tr.dem_at_home_msl_m - 584.0) < 0.01
    # 20 m above home at the crest: 604 m MSL over ground near 605 m.
    crest = g.elevation_at(0.0, 100.0)
    assert abs(tr.up(20.0, 0.0, 100.0) - (20.0 + 584.0 - crest)) < 1e-12
    assert abs(tr.msl(tr.up(20.0, 0.0, 100.0), 0.0, 100.0) - 604.0) < 1e-9
    st = tr.state(State(x=0.0, y=100.0, up=20.0))
    assert st.up == tr.up(20.0, 0.0, 100.0) and (st.x, st.y) == (0.0, 100.0)
    # Home's elevation 590 against the DEM's 584: refused (6 m over 2 m).
    try:
        terrain.TerrainReference(g, 590.0, (0.0, 0.0), "test")
    except ValueError as e:
        assert "differ by 6.00 m" in str(e), e
    else:
        raise AssertionError("a home elevation 6 m off the DEM was accepted")
    # None: the DEM's own elevation at home (the headless harness's take-off).
    assert terrain.TerrainReference(g, None, (0.0, 100.0)).home_msl_m == g.elevation_at(0, 100)
    try:
        terrain.TerrainReference(terrain.SITL_CMAC)
    except ValueError as e:
        assert "not a DEM" in str(e)
    else:
        raise AssertionError("a flat ground was taken as a DEM")


# ------------------------------------------------------------------ (b)

def _spec(**upd) -> ScenarioSpec:
    eps, _ = expand(LIBRARY, None, 0, ["altitude-ceiling-hold"])
    raw = eps[0].cell.spec.model_dump(mode="json", by_alias=True, exclude_none=True)
    raw.update(upd)
    return ScenarioSpec.model_validate(raw)


def test_a_scenario_ground_loads_the_policy_with_it_and_stays_its_own():
    import msl_dem_rail as M
    spec = M.spec_for("level-cross", 0.0, M.POLICY_REL)
    assert spec.ground == DEM and spec.policy._ground == DEM
    pol = spec.policy.load(ROOT)                     # the MSL rule passes the gate
    assert {c.id for c in pol.constraints} == {"msl-ceiling", "agl-band", "kin-caps"}
    # Without the ground the same policy is refused, as it always was.
    try:
        load_policy(POLICY)
    except ValueError as e:
        assert "MSL" in str(e), e
    else:
        raise AssertionError("the MSL policy loaded with no ground")
    # The reference is the spec's own copy: a spec without `ground` built
    # from the same PolicyRef carries no ground.
    plain = _spec()
    assert plain.ground is None and plain.policy._ground is None
    # A rotated mission with a ground is refused (the relief would not turn).
    try:
        ScenarioSpec.model_validate({**spec.model_dump(mode="json", by_alias=True),
                                     "mission": {**spec.mission.model_dump(mode="json"),
                                                 "rotate_deg": 30.0}})
    except ValueError as e:
        assert "rotated mission" in str(e), e
    else:
        raise AssertionError("a ground with a rotated mission was accepted")


def test_without_a_ground_nothing_is_added_to_the_record():
    import sweep_scenarios as S
    spec = _spec()
    run = S.run_episode(spec, LIBRARY.defaults, 1001)
    assert not any("msl_m" in r for r in run.rows)
    assert "ground" not in run.events[0] and "up_ref" not in run.events[0]
    ex = S.episode_extras(spec, LIBRARY.defaults, 1001, run)
    assert "ground" not in ex and "ground" not in ex["scenario_spec"]
    # rerun_from_extras reads the record back without the key.
    again = S.rerun_from_extras(json.loads(json.dumps(ex)))
    assert [r["up"] for r in again.rows] == [r["up"] for r in run.rows]


def test_a_flat_dem_flies_an_agl_scenario_exactly_as_no_ground():
    """The plumbing is neutral: the library's altitude-ceiling-hold (AGL
    only) over a DEM that is 584 m everywhere flies the same positions and
    commands, to the bit, as with no ground; only `msl_m` is added."""
    import sweep_scenarios as S
    real = terrain.load_ground

    def flat(path, frame_origin=None, *, root=None, reach_m=None):
        return terrain.GridGround(-1000.0, -1000.0, 50.0, [[584.0] * 41] * 41,
                                  "flat test DEM", origin=frame_origin)
    terrain.load_ground = flat
    try:
        grounded = _spec(ground=DEM)
        a = S.run_episode(grounded, LIBRARY.defaults, 1001)
    finally:
        terrain.load_ground = real
    b = S.run_episode(_spec(), LIBRARY.defaults, 1001)
    keys = ("x", "y", "up", "raw", "emitted", "violations", "unsafe", "fsm_state_after")
    assert len(a.rows) == len(b.rows) and len(a.rows) > 100
    for ra, rb in zip(a.rows, b.rows):
        assert {k: ra.get(k) for k in keys} == {k: rb.get(k) for k in keys}
        assert ra["msl_m"] == ra["up"] + 584.0
    assert a.extra.get("true_p0_flown_ticks") == b.extra.get("true_p0_flown_ticks")
    assert a.events[0]["ground"]["source"] == "flat test DEM"
    assert a.events[0]["up_ref"] == "terrain"


def test_the_msl_rule_binds_over_the_cmac_dem_and_holds():
    """One cell of tools/msl_dem_rail.py (docs/data/msl_dem_rail.json): a
    terrain-following goto pilot asks for 634 m MSL over the crest; the
    MSL ceiling (624 m) binds and the oracle (no Shield: the row's true MSL
    height against the declared band) sees no tick above it."""
    import msl_dem_rail as M
    import sweep_scenarios as S
    grid = terrain.load_wgs84_grid(ROOT / DEM)
    run = S.run_episode(M.spec_for("terrain-follow", 0.0, M.POLICY_REL), LIBRARY.defaults, 1001)
    named = sum(1 for r in run.rows if any(v["rule_id"] == "msl-ceiling"
                                           for v in r["violations"]))
    o = M.oracle(run.rows, grid)
    assert named > 0, "the MSL rule never bound"
    assert o["msl_ticks"] == 0 and o["max_msl_m"] <= M.MSL_BAND[1], o
    assert run.extra["true_p0_flown_ticks"] == 0
    # The vehicle holds MSL: msl_m - up is the ground under it.
    for r in run.rows[:50]:
        assert abs(r["msl_m"] - r["up"] - run_ground(r)) < 1e-9
    # The null: shield off, the same pilot goes over the ceiling.
    off = S.run_episode(M.spec_for("terrain-follow", 0.0, M.POLICY_REL, shield=False),
                        LIBRARY.defaults, 1001)
    assert M.oracle(off.rows, grid)["msl_ticks"] > 0


def run_ground(r):
    return terrain.load_ground(DEM, CMAC, root=ROOT).elevation_at(r["x"], r["y"])


def test_the_evidence_file_says_what_the_tool_measured():
    d = json.loads((ROOT / "docs" / "data" / "msl_dem_rail.json").read_text(encoding="utf-8"))
    s = d["summary"]
    v = d["verdict"]
    assert all(v["gated"].values()), v
    # The verdict is scoped to the MSL rule by name, and the oracle's P0
    # count over the whole policy (agl-band included) sits beside it, never
    # folded into the pass.
    assert "msl-ceiling" in v["scope"] and "agl-band" in v["scope"]
    ng = v["not_gated"]
    assert ng["oracle_p0_ticks"] == s["on_oracle_p0_ticks"]
    assert ng["oracle_p0_zero"] is (s["on_oracle_p0_ticks"] == 0)
    assert ng["oracle_p0_ticks_by_rule"] == {"msl-ceiling": s["on_oracle_msl_ticks"],
                                             "agl-band": s["on_oracle_agl_ticks"]}
    assert (f"harness true-P0 {s['on_harness_true_p0_ticks']}; independent oracle "
            f"{s['on_oracle_p0_ticks']} P0 ticks") in d["p0_counts"]
    assert s["on_oracle_msl_ticks"] == 0 and s["on_harness_true_p0_ticks"] == 0
    assert s["on_requested_ticks_msl_ceiling"] > 0
    assert s["null_off_oracle_p0_ticks"] > 0 and s["null_msl_as_agl_oracle_msl_ticks"] > 0
    # The AGL finding is reported, never hidden in a pass.
    f = d["finding_agl_over_dem"]
    assert f["oracle_agl_ticks"] == s["on_oracle_agl_ticks"]
    assert set(f["by_pilot"]) == {"level-cross", "climb-cross", "terrain-follow", "descend"}
    assert d["ground"]["wgs84_dem"]["path"] == DEM


def test_the_identity_evidence_and_its_compare_tool():
    """docs/data/msl_rail_identity_20261118.json: 141 library episodes
    without a ground, base 7b0f50c vs this branch, every digest equal; the
    compare tool names a differing field and an error."""
    import episode_digest as E
    d = json.loads((ROOT / "docs" / "data" / "msl_rail_identity_20261118.json")
                   .read_text(encoding="utf-8"))
    c = d["compare"]
    assert c["same_episodes"] and c["differing"] == [] and c["errors"] == []
    assert c["identical_episodes"] == c["episodes_before"] == 141 == len(d["digests_after"])
    # Both digest sets are stored, so the compare can be redone from the file.
    assert E.compare(d["digests_before"], d["digests_after"]) == c
    one = {f: "x" for f in E.FIELDS}
    r = E.compare({"a": one, "b": one}, {"a": dict(one, rows="y"), "b": {"error": "e"}})
    assert ["a", "rows"] in r["differing"] and r["errors"] == ["b"]
    assert r["identical_episodes"] == 0


def test_the_rails_refuse_a_scenario_ground():
    import msl_dem_rail as M
    import sweep_scenarios as S
    spec = M.spec_for("level-cross", 0.0, M.POLICY_REL)
    try:
        S.rail_node_args(spec, 1001, LIBRARY.defaults, backend="sitl")
    except S.UnsupportedScenario as e:
        assert "flown headless only" in str(e), e
    else:
        raise AssertionError("a scenario ground was sent to a rail whose scoring has none")


# ------------------------------------------------------------------ (c)

def _node(argv, policy=POLICY):
    import test_railinputs_node as RN
    N = RN._node_module()
    clk = RN._Clock()
    N.time = clk
    pre = ["--policy", str(policy), "--dev-yaml"] if policy is not None else []
    args, _ = N.make_parser().parse_known_args(pre + RN.HAND_TICKED + list(argv))
    out = _tmp()
    return RN, N, N.ShieldNode(args, out), clk, out


def test_the_ros2_node_flies_an_msl_rule_over_the_dem():
    g = terrain.load_ground(DEM, CMAC, root=ROOT)
    RN, N, node, clk, out = _node([f"--ground={DEM}", "--ground-home-msl=584",
                                   "--start=0,0,20", "--no-target", "--max-s", "0.35"])
    sh = node.shield
    assert node.terrain is not None and sh.ground is not None
    assert sh.ground.record()["wgs84_dem"]["sha256"] == g.record()["wgs84_dem"]["sha256"]
    assert node.policy_source["ground"]["wgs84_dem"]["path"] == DEM
    # 20 m above home at the crest (100 m east): 604 m MSL, the Shield told
    # its height above the DEM there (about -1.2 m: below the AGL floor).
    pose = State(x=0.0, y=100.0, up=20.0)
    told = node._shield_pose(pose)
    assert abs(told.up - (20.0 + 584.0 - g.elevation_at(0.0, 100.0))) < 1e-12
    assert {v.rule_id for v in sh.state_is_unsafe(told)} == {"agl-band"}
    # A short hand-ticked mission at home: the rows carry both heights.
    home = State(x=0.0, y=0.0, up=20.0)
    node._on_pose(RN._pose_msg(N, home))
    node.start_mission()
    for _ in range(4):
        node._on_pose(RN._pose_msg(N, home))
        RN._act(N, node, vx=0.0)
        clk.t += 0.1
        node._tick()
    assert node.rows
    r = node.rows[0]
    assert r["up_ref"] == "terrain" and r["up_home"] == 20.0
    assert abs(r["msl_m"] - 604.0) < 1e-3 and abs(r["up"] - 20.0) < 0.01
    assert node.done
    m = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert m["ground"]["home_msl_m"] == 584.0 and m["ground"]["up_ref"] == "terrain"
    assert m["ground"]["ground"]["wgs84_dem"]["path"] == DEM
    # The clean flight (604 m MSL, 20 m above the DEM: inside both bands) has
    # no altitude dwell. Until the review every tick counted (0.3 s for 3
    # ticks: 604 m MSL judged against `up`), and guardrail/kpi.py fails a run
    # on any alt_violation_s above 0.
    assert m["ticks"] > 0 and m["alt_violation_s"] == 0, m["alt_violation_s"]
    p = node.traj[0]
    assert abs(p["msl_m"] - 604.0) < 1e-3 and abs(p["up"] - 20.0) < 0.01


def test_alt_dwell_judges_each_envelope_in_its_own_reference():
    _, N, node, _, _ = _node([f"--ground={DEM}", "--ground-home-msl=584"])
    from guardrail.models import AltitudeEnvelope
    envs = node.policy.by_type(AltitudeEnvelope)
    ok = {"up": 20.0, "msl_m": 604.0}
    high = {"up": 20.0, "msl_m": 630.0}          # above the MSL ceiling only
    low = {"up": 5.0, "msl_m": 600.0}            # under the AGL floor only
    assert N.alt_dwell_ticks([ok], envs) == 0
    assert N.alt_dwell_ticks([high], envs) == 1 and N.alt_dwell_ticks([low], envs) == 1
    # Without msl_m (every flight without --ground) every envelope is judged
    # against `up`, as before: the MSL band 590-624 fails 20 m.
    assert N.alt_dwell_ticks([{"up": 20.0}], envs) == 1
    agl = [e for e in envs if e.altitude_ref is None]
    assert N.alt_dwell_ticks([{"up": 20.0}, {"up": 70.0}], agl) == 1


def test_the_fence_ceiling_is_above_home_with_a_ground():
    """FENCE_ALT_MAX is measured above home. Without the fix the MSL profile's
    ceiling was 624 (its MSL alt_max read as metres above home) and SITL's
    fence was raised to 629 m, switching the altitude backstop off."""
    g = terrain.load_ground(DEM, CMAC, root=ROOT)
    RN, N, node, clk, out = _node([f"--ground={DEM}", "--ground-home-msl=584",
                                   "--start=0,0,20", "--no-target"])
    kw = node._fence_ground_kw()
    assert kw["ground"] is node.ground and kw["home_msl_m"] == 584.0
    ceiling, src = N.policy_ceiling_m(node.policy, 20.0, **kw)
    top = terrain.ground_band(g)[1]
    # msl-ceiling: 624 - 584 = 40 m above home; agl-band: 60 m over the
    # DEM's highest ground, top - 584 above home. The higher of the two, as
    # without a ground (an envelope may be scoped or windowed).
    assert abs(ceiling - max(624.0 - 584.0, 60.0 + top - 584.0)) < 1e-9, (ceiling, top)
    assert ceiling < 100.0 and "above home" in src
    if str(ROOT / "sitl") not in sys.path:
        sys.path.insert(0, str(ROOT / "sitl"))
    from mavlink_adapter_node import FENCE_ALT_HEADROOM_M
    plan = N.fence_alt_plan(ceiling, 30.0)
    assert abs(plan["raise_to"] - (ceiling + FENCE_ALT_HEADROOM_M)) < 1e-9
    assert plan["raise_to"] < 100.0, plan
    # The MSL rule alone: 40 m above home + the headroom.
    only_msl = node.policy.model_copy(update={"constraints": [
        c for c in node.policy.constraints if c.id != "agl-band"]})
    c2, _ = N.policy_ceiling_m(only_msl, 20.0, **kw)
    assert abs(c2 - 40.0) < 1e-9
    # Without a ground: the figure every flight before 2026-11-18 got.
    assert N.policy_ceiling_m(node.policy, 20.0) == (624.0, "altitude_envelope")
    sim = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    assert N.policy_ceiling_m(sim, 15.0) == N.policy_ceiling_m(sim, 15.0, **{})
    # A flat ground at home's elevation changes no AGL ceiling.
    flat = terrain.FlatGround(584.0, "test")
    assert N.policy_ceiling_m(sim, 15.0, ground=flat, home_msl_m=584.0)[0] == \
        N.policy_ceiling_m(sim, 15.0)[0]


def test_the_ros2_node_refuses_msl_without_a_ground_or_with_a_wrong_home():
    try:
        _node([])
    except ValueError as e:
        assert "MSL" in str(e), e
    else:
        raise AssertionError("the node flew an MSL policy with no ground")
    try:
        _node([f"--ground={DEM}", "--ground-home-msl=590"])
    except ValueError as e:
        assert "differ by 6.00 m" in str(e), e
    else:
        raise AssertionError("the node took a home elevation 6 m off the DEM")


def test_without_ground_the_node_hands_the_shield_the_pose_itself():
    _, _, node, _, out = _node([], policy=ROOT / "policies" / "sim_demo_policy.yaml")
    assert node.ground is None and node.terrain is None and node.ground_kw == {}
    assert node.shield.ground is None and "ground" not in node.policy_source
    st = State(x=1.0, y=2.0, up=15.0)
    assert node._shield_pose(st) is st
    src = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "ground_for_flight(args.ground, b, y, root=ROOT)" in src   # main() reads it first


def test_the_stub_and_the_bundle_builder_take_the_ground():
    import importlib.util
    spec = importlib.util.spec_from_file_location("stub_msl", ROOT / "sitl" / "ros2_vla_stub_node.py")
    V = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(V)
    base = ["--policy", str(POLICY)]
    pol, src = V.stub_policy(V.make_parser().parse_known_args(base + [f"--ground={DEM}"])[0])
    assert pol.policy_id == "msl-dem-sitl" and src["ground"]["wgs84_dem"]["path"] == DEM
    try:
        V.stub_policy(V.make_parser().parse_known_args(base)[0])
    except ValueError as e:
        assert "MSL" in str(e), e
    else:
        raise AssertionError("the stub loaded the MSL policy with no ground")
    # The AerialVLA bridge (the rail's real-VLA slot) the same way; resolve()
    # connects to nothing.
    if str(ROOT / "sitl") not in sys.path:
        sys.path.insert(0, str(ROOT / "sitl"))
    import ros2_vla_bridge_node as Br
    bbase = ["--vla-server", "127.0.0.1:5999", "--policy", str(POLICY)]
    _, plan = Br.resolve(Br.make_parser().parse_known_args(bbase + [f"--ground={DEM}"])[0])
    assert plan["policy"].policy_id == "msl-dem-sitl"
    assert plan["policy_source"]["ground"]["wgs84_dem"]["path"] == DEM
    try:
        Br.resolve(Br.make_parser().parse_known_args(bbase)[0])
    except ValueError as e:
        assert "MSL" in str(e), e
    else:
        raise AssertionError("the bridge loaded the MSL policy with no ground")
    # The signed bundle recipe (docs/RUNBOOK-orin-hil.md): built and verified
    # over the ground, refused without it.
    import build_flight_bundles as T
    from guardrail import bundle as B
    k = _tmp()
    trust = k / "trust.json"
    if B.HAVE_CRYPTO:
        signer = B.generate_signing_key(k / "k.pem", authority="test-lab", trust_store=trust)
    else:
        import os
        signer = B.Signer(authority="test-lab",
                          private_key=B._PyEd25519TestKey(os.urandom(32)))
        B.register_public_key(signer, trust)
    out = _tmp()
    rows = T.build([str(POLICY)], out, signer=signer, trust_store=trust,
                   issued_at="2026-11-18T00:00:00Z", ground=DEM)
    assert rows[0]["ok"] and Path(rows[0]["bundle"]).name == "msl-dem-sitl-v0.1.0.tar.gz", rows
    assert T.check_one(str(POLICY), out, trust, ground=DEM)["ok"]
    try:
        T.build([str(POLICY)], _tmp(), signer=signer, trust_store=trust,
                issued_at="2026-11-18T00:00:00Z")
    except ValueError as e:
        assert "not flyable" in str(e) and "MSL" in str(e), e
    else:
        raise AssertionError("an MSL bundle was built with no ground")
    # The node flies that bundle with the ground.
    _, _, node, _, _ = _node([f"--ground={DEM}", "--ground-home-msl=584", "--bundle",
                              rows[0]["bundle"], "--allow-unverified-bundle"],
                             policy=None)
    assert node.policy.policy_id == "msl-dem-sitl"


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
