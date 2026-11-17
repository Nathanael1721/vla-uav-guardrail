"""distance_envelope on the rails (WP1-07, 2026-11-17): a declared world for
the SITL rails, the road map on the city rails, and the people limit.

Run either way:
    pytest tests/test_world_map.py -v
    python tests/test_world_map.py

WHAT IS PINNED

  (a) A scenario can name a declared world (ScenarioSpec.world,
      guardrail/world_map.py): rectangles in the policy frame, rasterised on
      the city map's cell convention. The harness gives every Shield of the
      episode its maps; the hil / desktop rail sends it to the Shield node as
      --world (declared there, checked by check_rail_flags), and the node
      builds both of its Shields with it. The distance_envelope profile's two
      cells are accepted by the hil dry-run plan (not refused) and, headless,
      the Shield acts on de-buildings in one and de-roads in the other with
      no breach. Before: no scenario could carry the rule (the Shield refused
      the policy without a map, and no rail had one).
  (b) The city rails (vla_fly, the PAS rail) build a road map from the
      street mask for a distance_envelope over roads, and load the obstacle
      map for one over buildings; for every policy without such a rule the
      helper returns None, so their Shields are the ones they were. Before:
      a roads envelope was refused on both.
  (c) People: the rule binds the ONE tracked subject; a person beside a
      tracked car is outside it (the recorded limit, models.DistanceEnvelope).

Throwaway signing keys only (guardrail.bundle.generate_signing_key into a
temporary trust store); no rig step runs.
"""
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "experiments", ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np                                                      # noqa: E402

from guardrail.models import (DistanceEnvelope, State, load_policy,     # noqa: E402
                              policy_from_raw)
from guardrail.scenario_spec import (Library, ScenarioSpec, expand,     # noqa: E402
                                     load_profile)
from guardrail.shield import Shield                                     # noqa: E402
from guardrail.world_map import load_world, missing_maps                # noqa: E402

WORLD = "experiments/worlds/de_block.yaml"
POLICY = ROOT / "policies" / "distance_envelope_sitl.yaml"
PROFILE = ROOT / "experiments" / "profiles" / "distance_envelope.yaml"
LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")
SKIP = "SKIP"


def _tmp_world(text: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix="world_"))
    p = d / "w.yaml"
    p.write_text(text, encoding="utf-8")
    return p


# ---------------------------------------------------------------- the world

def test_the_world_is_rasterised_on_the_city_maps_cell_convention():
    w = load_world(WORLD, ROOT)
    occ, road = w.maps["buildings"], w.maps["roads"]
    assert (occ["res"], occ["ox"], occ["oy"]) == (1.0, -60.0, -50.0)
    assert occ["occ"].shape == w.shape == (181, 101)

    def cell(m, x, y):
        return int(m["occ"][int(round((x - m["ox"]) / m["res"])),
                            int(round((y - m["oy"]) / m["res"]))])
    # block-a x 20..40, y 2..22: cell centres on its faces are in, outside not.
    assert cell(occ, 20, 2) == cell(occ, 40, 22) == cell(occ, 30, 10) == 1
    assert cell(occ, 19, 2) == cell(occ, 30, 1) == cell(occ, 30, -12) == 0
    assert int(occ["occ"].sum()) == 21 * 21
    # road-w x 10..60, y -18..-12.
    assert cell(road, 30, -12) == cell(road, 10, -18) == 1 and cell(road, 30, -11) == 0
    assert int(road["occ"].sum()) == 51 * 7
    rec = w.record()
    assert rec["world"] == "de-block-v1" and rec["path"] == WORLD
    assert rec["classes"] == {"buildings": ["block-a"], "roads": ["road-w"]}
    assert len(rec["sha256"]) == 64
    assert set(w.shield_kwargs()) == {"obstacle_map", "road_map"}
    # The module docstring's example IS the shipped world (it once gave road-w
    # 2 m off, which put lane -11 inside the road; review of wf17-distenv).
    import yaml
    import guardrail.world_map as W
    blk = W.__doc__.split("left out):\n\n", 1)[1].split("\n\n", 1)[0]
    example = yaml.safe_load("\n".join(ln[4:] for ln in blk.splitlines()))
    shipped = yaml.safe_load((ROOT / WORLD).read_text(encoding="utf-8"))
    shipped.pop("description", None)
    assert example == shipped, (example, shipped)


def test_a_bad_world_file_is_refused():
    base = "world: w\nres_m: 1.0\nextent: {x: [0, 10], y: [0, 10]}\n"
    bad = {
        "outside the extent": base + "buildings:\n  - {id: a, x: [5, 12], y: [0, 2]}\n",
        "twice": base + "buildings:\n  - {id: a, x: [1, 2], y: [1, 2]}\n"
                        "  - {id: a, x: [3, 4], y: [3, 4]}\n",
        "unknown key": base + "trees: []\nbuildings:\n  - {id: a, x: [1, 2], y: [1, 2]}\n",
        "neither": base,
        "no cell": base + "roads:\n  - {id: r, x: [1.2, 1.4], y: [1.2, 1.4]}\n",
        "not yaml": "world: [unclosed\n",
        "tiny cells": "world: w\nres_m: 0.01\nextent: {x: [0, 10], y: [0, 10]}\n"
                      "buildings:\n  - {id: a, x: [1, 2], y: [1, 2]}\n",
    }
    for why, text in bad.items():
        try:
            load_world(_tmp_world(text))
        except ValueError:
            continue
        raise AssertionError(f"accepted a world file with {why}")


def test_the_policy_is_refused_without_its_maps_and_enforced_with_them():
    pol = load_policy(POLICY)
    assert {c.object_class for c in pol.by_type(DistanceEnvelope)} == {"buildings", "roads"}
    assert pol.flight_problems() == []
    # Before 2026-11-17 no SITL rail had a map: the Shield refuses it so.
    try:
        Shield(pol)
    except ValueError as e:
        assert "obstacle_map" in str(e) and "road_map" in str(e)
    else:
        raise AssertionError("a distance_envelope policy built a Shield with no map")
    assert len(missing_maps(pol, None)) == 2
    w = load_world(WORLD, ROOT)
    assert missing_maps(pol, w) == []
    only_b = load_world(_tmp_world(
        "world: b\nres_m: 1.0\nextent: {x: [0, 10], y: [0, 10]}\n"
        "buildings:\n  - {id: a, x: [1, 2], y: [1, 2]}\n"))
    assert [m.split(":")[0] for m in missing_maps(pol, only_b)] == ["de-roads"]
    sh = Shield(pol, **w.shield_kwargs())

    def rules(y):
        return {v.rule_id for v in sh.state_is_unsafe(State(x=30.0, y=y, up=15.0))}
    assert rules(0.0) == {"de-buildings"}          # 2 m from block-a's face
    assert rules(-6.0) == set()                    # between the two
    assert rules(-11.0) == {"de-roads"}            # 1 m from road-w's edge


# ------------------------------------------------- (a) the scenario and rails

def test_the_family_is_flown_by_its_profile_alone_and_moves_no_other_count():
    prof = load_profile(PROFILE)
    assert prof.grant_kpi_bearing is False
    eps, info = expand(LIBRARY, prof)
    assert info["cells"] == 2 and info["cells_within_expected"] and not info["duplicate_cells"]
    assert sorted(e.cell.params["lane_y"] for e in eps) == [-11.0, -1.0]
    assert all(e.cell.spec.world == WORLD for e in eps)
    for name in ("smoke", "nightly", "broad"):
        _, other = expand(LIBRARY, load_profile(ROOT / "experiments" / "profiles"
                                                / f"{name}.yaml"))
        assert "distance-envelope-lanes" not in other["cells_per_family"], name
    _, lib = expand(LIBRARY)
    assert "distance-envelope-lanes" not in lib["cells_per_family"]
    # No other family names a world: every other scenario builds its Shields
    # exactly as before (no map kwargs; sweep_scenarios.episode_world None).
    for fam in LIBRARY.families:
        if fam.scenario_id != "distance-envelope-lanes":
            assert fam.raw.get("world") is None, fam.scenario_id


def test_a_rotated_mission_with_a_world_is_refused():
    raw = dict(scenario_id="w-rot", template="distance_envelope", description="x",
               world=WORLD, policy={"path": "policies/distance_envelope_sitl.yaml"},
               mission={"start_pose": {"x": 0, "y": 0, "up": 15},
                        "target": {"x": 10, "y": 0, "up": 15},
                        "pilot": {"type": "goto"}, "rotate_deg": 90.0},
               expected_kpis={"P0_escape_rate": 0})
    try:
        ScenarioSpec.model_validate(raw)
    except ValueError as e:
        assert "rotated" in str(e)
    else:
        raise AssertionError("a rotated mission with a world validated")
    raw["mission"]["rotate_deg"] = 0.0
    assert ScenarioSpec.model_validate(raw).world == WORLD


def test_headless_the_shield_acts_on_each_envelope_and_lets_nothing_in():
    """Measured 2026-11-17, seed 1001, ideal vehicle: lane -1 is pushed out
    to y = -3.68 past block-a (de-buildings named, never de-roads), lane -11
    to y = -8.09 past road-w (de-roads only); both reach the target with no
    breach and no fail-safe."""
    import sweep_scenarios as S
    eps, _ = expand(LIBRARY, load_profile(PROFILE))
    for e in eps:
        lane = e.cell.params["lane_y"]
        run = S.run_episode(e.cell.spec, LIBRARY.defaults, e.seed)
        named = {v["rule_id"] for r in run.rows for v in r["violations"]}
        assert named == ({"de-buildings"} if lane == -1.0 else {"de-roads"}), (lane, named)
        assert not any(r["unsafe"] for r in run.rows), lane
        assert run.extra["reached_goal"] is True and run.extra["breaches"] == 0, lane
        ys = [r["y"] for r in run.rows]
        if lane == -1.0:
            assert min(ys) < -3.0          # held >= 5 m from the face at y = 2
        else:
            assert max(ys) > -9.0          # held >= 3 m from the edge at y = -12
        # The world is on the episode's record, pinned by its content hash.
        extras = S.episode_extras(e.cell.spec, LIBRARY.defaults, e.seed, run, episode=e)
        assert extras["world"]["sha256"] == load_world(WORLD, ROOT).sha256
        assert extras["scenario_spec"]["world"] == WORLD


def test_without_its_world_the_family_cannot_fly():
    import sweep_scenarios as S
    eps, _ = expand(LIBRARY, load_profile(PROFILE))
    spec = eps[0].cell.spec.model_copy(update={"world": None})
    try:
        S.run_episode(spec, LIBRARY.defaults, eps[0].seed)
    except ValueError as e:
        assert "obstacle_map" in str(e)
    else:
        raise AssertionError("the family flew with no world")
    spec = eps[0].cell.spec.model_copy(update={"world": "experiments/worlds/nope.yaml"})
    try:
        S.episode_world(spec)
    except S.UnsupportedScenario:
        pass
    else:
        raise AssertionError("a missing world file was accepted")
    # A policy that cannot be loaded is refused up front too (review of
    # wf17-distenv: it was swallowed, and the world check skipped).
    bad = eps[0].cell.spec.model_copy(update={"policy": eps[0].cell.spec.policy.model_copy(
        update={"path": "policies/nope_distance_envelope.yaml"})})
    try:
        S.episode_world(bad)
    except S.UnsupportedScenario as e:
        assert "policy cannot be loaded" in str(e), e
    else:
        raise AssertionError("a world was accepted against a policy that cannot be loaded")


def test_the_rail_sends_the_world_and_the_node_declares_it():
    import sweep_scenarios as S
    assert "--world" in S.sitl_node_flags(ROOT / S.SITL_NODE)
    assert "--world" in S.RAIL_SHIELD_FLAGS
    eps, _ = expand(LIBRARY, load_profile(PROFILE))
    args = S.rail_node_args(eps[0].cell.spec, eps[0].seed, LIBRARY.defaults, backend="sitl")
    assert args[-1] == f"--world={WORLD}", args
    S.check_rail_flags(args)                       # raises if a node would drop it
    # A scenario without a world sends the list it sent before.
    other, _ = expand(LIBRARY, None, 0, ["nfz-head-on"])
    assert not any(a.startswith("--world") for a in
                   S.rail_node_args(other[0].cell.spec, 0, LIBRARY.defaults))
    # main() reads the file before ROS starts (a bad path is refused with no
    # node built); what the node does with it is flown below
    # (test_the_ros2_node_flies_the_world_it_is_given).
    src = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "load_world(args.world, ROOT)" in src


def _de_node(argv, policy=POLICY):
    """sitl/ros2_shield_node.py's ShieldNode on the stand-in ROS stack
    (tests/test_railinputs_node.py, as tests/test_sitl_rails.py's
    _scenario_node), built from its own parser with `policy` and `argv` on a
    hand-moved clock: (RN, N, node, clk, out)."""
    if str(ROOT / "tests") not in sys.path:
        sys.path.insert(0, str(ROOT / "tests"))
    import test_railinputs_node as RN
    N = RN._node_module()
    clk = RN._Clock()
    N.time = clk
    args, _ = N.make_parser().parse_known_args(
        ["--policy", str(policy), "--dev-yaml"] + RN.HAND_TICKED + list(argv))
    out = Path(tempfile.mkdtemp(prefix="de_node_"))
    return RN, N, N.ShieldNode(args, out), clk, out


def test_the_ros2_node_flies_the_world_it_is_given():
    """The hil rail's own node, built and ticked (review of wf17-distenv:
    the wiring was pinned by source strings only). With --world the flying
    Shield has both maps and names each rule where the world puts it; the
    warm-up Shield (start_mission) is built with them too, or it would raise;
    metrics.json pins the world by its hash. Without --world the node refuses
    the policy before take-off, naming both rules. The null: a policy with no
    distance_envelope gets no map from the same --world."""
    world = load_world(WORLD, ROOT)
    RN, N, node, clk, out = _de_node([f"--world={WORLD}", "--start=0,-6,15",
                                      "--no-target", "--max-s", "0.35"])
    sh = node.shield
    assert node.world is not None and node.world.sha256 == world.sha256
    assert sh.has_obstacle_map and sh._road is not None
    named = lambda y: {v.rule_id for v in sh.state_is_unsafe(State(x=30.0, y=y, up=15.0))}
    assert named(-1.0) == {"de-buildings"}, named(-1.0)     # 3 m from block-a's face
    assert named(-11.0) == {"de-roads"}, named(-11.0)       # 1 m from road-w's edge
    assert named(-6.0) == set(), named(-6.0)                # the legal band
    # One short hand-ticked mission in the legal band: the warm-up and every
    # tick run on the world's Shield, and the record names the world.
    pose = State(x=0.0, y=0.0, up=15.0)                     # EKF frame: the --start
    node._on_pose(RN._pose_msg(N, pose))
    node.start_mission()
    assert any(e["kind"] == "shield_warmup" for e in RN._events(out))
    for _ in range(6):
        node._on_pose(RN._pose_msg(N, pose))
        RN._act(N, node, vx=0.0)
        clk.t += 0.1
        node._tick()
    assert node.done and node.rows, (node.done, len(node.rows))
    assert not any(r.get("unsafe") for r in node.rows)
    import json
    m = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert m["world"]["sha256"] == world.sha256 and m["world"]["path"] == WORLD, m["world"]
    assert m["world"]["classes"] == {"buildings": ["block-a"], "roads": ["road-w"]}
    # Without --world: refused before take-off, both rules named.
    try:
        _de_node([])
    except ValueError as e:
        msg = str(e)
        assert msg.startswith("REFUSED before take-off"), msg
        assert "de-buildings" in msg and "de-roads" in msg, msg
    else:
        raise AssertionError("the node built a Shield over distance_envelope_sitl with no world")
    # The null: sim_demo_policy (no distance_envelope) with the same --world
    # flies the Shield it flew before: no map at all.
    _, _, plain, _, _ = _de_node([f"--world={WORLD}"],
                                 policy=ROOT / "policies" / "sim_demo_policy.yaml")
    assert not plain.shield.has_obstacle_map and plain.shield._road is None


def test_the_hil_dry_run_plans_both_missions_and_refuses_none():
    import sweep_scenarios as S
    import build_flight_bundles as T
    from guardrail import bundle as B
    k = Path(tempfile.mkdtemp(prefix="throwaway_key_"))
    trust = k / "trust.json"
    if B.HAVE_CRYPTO:
        signer = B.generate_signing_key(k / "k.pem", authority="test-lab", trust_store=trust)
    else:
        import os
        signer = B.Signer(authority="test-lab",
                          private_key=B._PyEd25519TestKey(os.urandom(32)))
        B.register_public_key(signer, trust)
    out = Path(tempfile.mkdtemp(prefix="de_bundles_"))
    rows = T.build(["distance_envelope_sitl"], out, signer=signer, trust_store=trust,
                   issued_at="2026-11-17T00:00:00Z")
    assert rows[0]["ok"], rows
    assert "distance_envelope_sitl" in T.FLIGHT_POLICIES
    eps, _ = expand(LIBRARY, load_profile(PROFILE))
    plan = S.hil_plan(eps, LIBRARY.defaults, S.HilConfig(bundles_dir=str(out)), "detest")
    assert [p["fly"] for p in plan] == [True, True], plan
    for p in plan:
        assert f"--world={WORLD}" in p["args"]
        assert "--bundle" in p["args"] and "bundles/distance-envelope-sitl-v0.1.0.tar.gz" in p["args"]
        assert p["shield"] == "on" and p["sitl_home"]
    # Without the bundle the plan refuses it for the bundle, never for the rule.
    empty = Path(tempfile.mkdtemp(prefix="de_nobundle_"))
    plan = S.hil_plan(eps, LIBRARY.defaults, S.HilConfig(bundles_dir=str(empty)), "detest")
    assert all(not p["fly"] and "signed bundle" in p["refused"] for p in plan), plan


def test_rail_scoring_reads_a_distance_envelope_log_without_its_map():
    """sitl_extra's judges read rule status and zones (no map changes them);
    the envelope's breaches are the flying Shield's own `unsafe`. Before
    this branch Shield(policy) there raised for a distance_envelope."""
    import sweep_scenarios as S
    pol = load_policy(POLICY)
    rows = [{"t": 0.1 * i, "x": -20.0 + 0.4 * i, "y": -1.0, "up": 15.0, "generation": 0,
             "unsafe": False, "violations": []} for i in range(5)]
    judges, pin, why = S._rail_judges(rows, pol, None, 5.0, 0.1)
    assert why is None and set(judges) == {0}
    S.rail_measurements(rows, pol, None)          # must not raise
    # The rebuilt per-step judgement refuses (it would be blind), never guesses.
    gens = {0: pol, 1: pol}
    rows2 = [dict(r, generation=i % 2) for i, r in enumerate(rows)]
    out, why = S._judged_steps(rows2, pol, gens)
    assert out is None and "distance_envelope" in why


# ------------------------------------------------------ (b) the city rails

def _street_dir():
    d = Path(tempfile.mkdtemp(prefix="citymap_"))
    s = np.zeros((40, 40), dtype=np.uint8)
    s[:, 20:24] = 1
    np.savez(d / "street.npz", street=s, res=np.float64(2.0),
             origin_x=np.float64(-40.0), origin_y=np.float64(-40.0))
    return d


def _roads_policy():
    return policy_from_raw({
        "policy_id": "de-roads-test", "version": "0.1.0",
        "constraints": [{"id": "de-roads", "type": "distance_envelope",
                         "constraint_type": "hard", "scope": "mission",
                         "priority": "P1", "violation_action": "project_fix",
                         "object_class": "roads", "min_distance_m": 3.0}]})


def test_the_city_rails_road_map_helper():
    from build_street_mask import road_map_for
    d = _street_dir()
    pol = _roads_policy()
    rmap, note = road_map_for(pol, d)
    assert rmap is not None and rmap["res"] == 2.0 and int(rmap["occ"].sum()) == 160, note
    sh = Shield(pol, road_map=rmap)
    # Road cells j 20..23 -> y 0..6 m; x = 0 at y = 7.5 is 1.5 m off it.
    assert {v.rule_id for v in sh.state_is_unsafe(State(x=0.0, y=7.5, up=10.0))} == {"de-roads"}
    assert sh.state_is_unsafe(State(x=0.0, y=14.0, up=10.0)) == []
    none, why = road_map_for(pol, Path(tempfile.mkdtemp(prefix="nomask_")))
    assert none is None and "no street mask" in why
    # Every policy in the repository without a roads envelope gets None: the
    # Shield those rails build for it is the one they built before.
    for f in sorted((ROOT / "policies").glob("*.yaml")):
        p = load_policy(f, runtime=False)
        if not any(c.object_class == "roads" for c in p.by_type(DistanceEnvelope)):
            assert road_map_for(p, d)[0] is None, f.name


def test_the_real_citylife_street_mask_serves_as_a_road_map():
    """The whole-level CityLife maps (rebuilt 4 Nov) live in the lab
    checkout's gitignored demo/out; skipped where they are not."""
    from build_street_mask import road_map_for
    for base in (ROOT, ROOT.parents[2] if len(ROOT.parents) > 2 else ROOT):
        d = base / "demo" / "out" / "citymap_citylife"
        if (d / "street.npz").is_file():
            break
    else:
        print("      SKIP: no demo/out/citymap_citylife/street.npz here")
        return SKIP
    rmap, note = road_map_for(_roads_policy(), d)
    assert rmap is not None and int(rmap["occ"].sum()) > 0, note
    Shield(_roads_policy(), road_map=rmap)


def test_vla_fly_and_the_pas_rail_pass_the_road_map_to_their_shield():
    vf = (ROOT / "demo" / "vla_fly.py").read_text(encoding="utf-8")
    assert "rmap, road_note = road_map_for(policy, Path(args.citymap).parent)" in vf
    assert "obstacle_map=smap,\n                         road_map=rmap)" in vf
    assert 'c.object_class == "buildings" for c in policy.by_type(DistanceEnvelope)' in vf
    pn = (ROOT / "demo" / "pas_ardupilot" / "perception_node.py").read_text(encoding="utf-8")
    assert "rmap, road_note = road_map_for(policy, Path(args.citymap).parent)" in pn
    assert "obstacle_map=smap, road_map=rmap, clock=rule_now," in pn
    assert '"road_map": road_note,' in pn


def test_the_pas_rail_loads_the_obstacle_map_for_a_buildings_envelope():
    sys.path.insert(0, str(ROOT / "demo" / "pas_ardupilot"))
    import perception_node as pn
    pol = policy_from_raw({
        "policy_id": "de-b-test", "version": "0.1.0",
        "constraints": [{"id": "de-b", "type": "distance_envelope",
                         "constraint_type": "hard", "scope": "mission",
                         "priority": "P0", "violation_action": "project_fix",
                         "object_class": "buildings", "min_distance_m": 4.0}]})
    d = Path(tempfile.mkdtemp(prefix="occ_"))
    occ = np.zeros((20, 20), dtype=np.uint8)
    occ[8:12, 8:12] = 1
    np.savez(d / "occ_day.npz", occ=occ, res=np.float64(2.0),
             origin_x=np.float64(-20.0), origin_y=np.float64(-20.0))
    smap, note = pn.load_obstacle_map(pol, d / "occ_day.npz")
    assert smap is not None and int(smap["occ"].sum()) == 16, note
    Shield(pol, obstacle_map=smap)
    # Without the file the note says what the Shield will do with a buildings
    # envelope (refuse it), not "INERT" (review of wf17-distenv)...
    none, why = pn.load_obstacle_map(pol, d / "missing.npz")
    assert none is None and "INERT" not in why, why
    assert "de-b" in why and "Shield refuses this policy" in why, why
    # ...and an obstacle_clearance-only policy still runs inert, said so.
    oc = policy_from_raw({"policy_id": "oc", "version": "0.1.0", "constraints": [
        {"id": "oc", "type": "obstacle_clearance", "constraint_type": "hard",
         "scope": "mission", "priority": "P0", "violation_action": "project_fix",
         "min_clearance_m": 4.0}]})
    assert "obstacle_clearance is INERT" in pn.load_obstacle_map(oc, d / "missing.npz")[1]
    # A policy with neither rule: no map, as before.
    plain = policy_from_raw({"policy_id": "p", "version": "0.1.0", "constraints": [
        {"id": "a", "type": "altitude_envelope", "constraint_type": "hard",
         "scope": "mission", "priority": "P0", "violation_action": "project_fix",
         "alt_min_m": 5, "alt_max_m": 30}]})
    assert pn.load_obstacle_map(plain, d / "occ_day.npz")[0] is None


# --------------------------------------------------------- (c) the people limit

def test_people_binds_the_tracked_subject_only_the_recorded_limit():
    pol = policy_from_raw({
        "policy_id": "de-p-test", "version": "0.1.0",
        "constraints": [{"id": "de-people", "type": "distance_envelope",
                         "constraint_type": "hard", "scope": "mission",
                         "priority": "P0", "violation_action": "project_fix",
                         "object_class": "people", "min_distance_m": 10.0}]})
    sh = Shield(pol)
    st = State(x=0.0, y=0.0, up=10.0)
    # A tracked car 5 m away: a person standing beside it is not passed to
    # the Shield (set_subject holds one subject), so nothing binds.
    sh.set_subject(5.0, 0.0, "car")
    assert sh.state_is_unsafe(st) == []
    # The same position tracked AS a person binds.
    sh.set_subject(5.0, 0.0, "pedestrian")
    assert {v.rule_id for v in sh.state_is_unsafe(st)} == {"de-people"}
    doc = DistanceEnvelope.__doc__
    assert "PEOPLE: THE RECORDED LIMIT" in doc and "40 pedestrians" in doc


if __name__ == "__main__":
    import traceback
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = skipped = 0
    for name, fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {name}")
                continue
            print(f"PASS  {name}")
        except Exception as e:                                # noqa: BLE001
            failed += 1
            print(f"FAIL  {name}\n      {type(e).__name__}: {e}")
            traceback.print_exc()
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
