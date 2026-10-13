"""Self-contained episode bundles, and the policy references the SITL rail is
handed.

    python tests/test_harnesstruth_bundles.py      (no pytest needed)

WHY THIS FILE EXISTS

An episode bundle carried the grant's six-field manifest, the KPI table and
the harness events, and a manifest_extras.json only when the episode had a
paraphrase or prefix arm. The cell it flew (the family with every parameter
bound), the library defaults it relied on, the forecast, the wind model and
the policy text were nowhere in it, so nothing outside this checkout could
re-fly the episode. Since 2026-10-12 every headless bundle's
manifest_extras.json holds them (sweep_scenarios.episode_extras), and
rerun_from_extras() re-flies the episode from that file alone.

The SITL half: a launch flies a signed bundle unless told otherwise, so a
YAML policy needs `--dev-yaml` beside `--policy`, and a bundle goes with the
hash it must hold - or, while the node does not declare the hash flag, is
refused (the node would drop the flag and fly the bundle unpinned; review of
2026-10-12). The same review added the rerun keys its mutants got past:
the library defaults, the wind model, the arm record, the events, refused
hot-applies, and a signed bundle carried in the extras.

Shown failing first: every test below fails or errors when this file is run
from a pristine copy of HEAD 4418ae0 (`git archive HEAD | tar -x`, this file
copied into its tests/).
"""
from __future__ import annotations

import ast
import copy
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import yaml                                                       # noqa: E402

import sweep_scenarios as S                                       # noqa: E402
from guardrail.scenario_spec import Library, ScenarioSpec         # noqa: E402

LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")
FAMILIES = LIBRARY.by_id()
SKIP = "SKIP"

# The keys episode_extras() writes (the interface the WP4 outputs tools read).
EXTRAS_KEYS = {"extras_schema", "scenario_id", "template", "random_seed", "arm",
               "forecast", "wind_model", "library_defaults", "scenario_spec",
               "scenario_events", "policy", "generations", "rerun",
               "episode_id", "run_id", "params", "arm_tag"}


def _main(argv):
    """sweep_scenarios.main, with an argparse exit (an unknown flag on an older
    harness) reported as this test's failure rather than ending the run."""
    try:
        return S.main(argv)
    except SystemExit as e:
        raise AssertionError(f"main() exited {e.code} on {argv}") from None


def _bundles(families, *extra):
    """main() on a temporary library with flight logs kept; (rc, doc,
    {bundle dir name: (extras, flight log rows)})."""
    with tempfile.TemporaryDirectory() as td:
        raw = yaml.safe_load((ROOT / "experiments" / "scenarios.yaml").read_text(encoding="utf-8"))
        lib = Path(td) / "lib.yaml"
        lib.write_text(yaml.safe_dump({"defaults": raw["defaults"], "scenarios": families}),
                       encoding="utf-8")
        out, runs = Path(td) / "s.json", Path(td) / "runs"
        rc = _main(["--library", str(lib), "--out", str(out), "--runs", str(runs),
                     "--keep-logs", *extra])
        doc = json.loads(out.read_text(encoding="utf-8"))
        got = {}
        for d in sorted(p for p in runs.iterdir() if p.is_dir()):
            ex = json.loads((d / "manifest_extras.json").read_text(encoding="utf-8"))
            rows = [json.loads(x) for x in
                    (d / "flight_log.jsonl").read_text(encoding="utf-8").splitlines() if x]
            got[d.name] = (ex, rows)
    return rc, doc, got


def _norm(rows):
    return json.loads(json.dumps(S._json_safe(rows), allow_nan=False))


def test_every_headless_bundle_reruns_from_its_manifest_extras_alone():
    """Three kinds of episode: a seeded GPS-noise cell (the seed matters), a
    hot-applied zone (an event and a second policy generation), and a
    paraphrase-armed cell on the 3 s comparison forecast too (the arm and the
    forecast travel). Each bundle's manifest_extras.json re-flies its
    flight_log.jsonl row for row, with no reference to the library file."""
    noise = copy.deepcopy(LIBRARY.raw("gps-noise-within-margin"))
    noise["parameter_sweep"] = {"pass_y": [23.6], "sigma": [0.3], "latency": [2]}
    spawn = copy.deepcopy(LIBRARY.raw("dyn-nfz-spawn-on-top"))
    spawn["parameter_sweep"] = {"size_m": [40]}
    head = copy.deepcopy(LIBRARY.raw("nfz-head-on"))
    head["ab"] = False
    head["paraphrase"] = dict(head["paraphrase"], k=1)
    rc, doc, got = _bundles([noise, spawn, head], "--seed", "1001",
                            "--forecast-arms", "grant", "3s")
    assert rc == 0, rc
    assert len(got) == 2 * (1 + 1 + 2), sorted(got)
    for name, (ex, rows) in got.items():
        missing = EXTRAS_KEYS - set(ex)
        assert not missing, (name, missing)
        assert ex["extras_schema"] == S.EXTRAS_SCHEMA and ex["random_seed"] == 1001
        assert ex["policy"]["yaml_text"] and ex["policy"]["policy_hash"].startswith("sha256:")
        assert ex["generations"][0]["generation"] == 0
        assert ("forecast-3s" in name) == (ex["arm"]["forecast"] == "3s"), name
        again = S.rerun_from_extras(json.loads(json.dumps(ex)))
        assert _norm(again.rows) == rows, f"{name}: the re-run flew differently"
    spawned = [ex for n, (ex, _) in got.items() if "spawn-on-top" in n]
    assert all(len(ex["generations"]) == 2 and ex["generations"][1]["event"] ==
               "spawn_polygon_fence" for ex in spawned), spawned[0]["generations"]
    armed = [ex for n, (ex, _) in got.items() if "nfz-head-on" in n]
    assert all(ex["arm"]["record"]["paraphrase"]["paraphrase_id"] == ex["paraphrase_id"]
               for ex in armed)


def test_a_rerun_refuses_a_policy_that_is_not_the_one_flown():
    spec = FAMILIES["speed-cap"].first.spec
    run = S.run_episode(spec, LIBRARY.defaults, 0)
    ex = json.loads(json.dumps(S.episode_extras(spec, LIBRARY.defaults, 0, run)))
    assert S.rerun_from_extras(ex).rows == run.rows
    bad = copy.deepcopy(ex)
    bad["policy"]["yaml_text"] = bad["policy"]["yaml_text"].replace(
        "speed_max_mps: 4.0", "speed_max_mps: 9.0")
    assert bad["policy"]["yaml_text"] != ex["policy"]["yaml_text"], "the edit missed"
    try:
        S.rerun_from_extras(bad)
    except ValueError as e:
        assert "policy changed" in str(e), e
    else:
        raise AssertionError("a re-run flew a policy that is not the one recorded")
    wrong = dict(ex, extras_schema="sweep-episode-extras/0")
    try:
        S.rerun_from_extras(wrong)
    except ValueError as e:
        assert "schema" in str(e)
    else:
        raise AssertionError("an unknown extras schema was read")


def _node_flags() -> set[str]:
    """The node's argparse flags, read from its source; a flag given as a
    name imported from guardrail (DEV_YAML_FLAG, guardrail.manifest) or a
    module-level constant is resolved."""
    import importlib
    tree = ast.parse((ROOT / S.SITL_NODE).read_text(encoding="utf-8"))
    consts: dict[str, str] = {}
    for node in tree.body:
        if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant)
                and isinstance(node.value.value, str)):
            consts.update({t.id: node.value.value for t in node.targets
                           if isinstance(t, ast.Name)})
        elif (isinstance(node, ast.ImportFrom) and node.module
              and node.module.split(".")[0] == "guardrail"):
            for alias in node.names:
                try:
                    val = getattr(importlib.import_module(node.module), alias.name, None)
                except Exception:                            # noqa: BLE001
                    val = None
                if isinstance(val, str):
                    consts[alias.asname or alias.name] = val
    flags = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "add_argument"):
            for a in node.args:
                v = (a.value if isinstance(a, ast.Constant)
                     else consts.get(a.id) if isinstance(a, ast.Name) else None)
                if isinstance(v, str) and v.startswith("--"):
                    flags.add(v)
    return flags


def test_a_yaml_policy_goes_to_the_rail_with_dev_yaml():
    cmd = S.build_sitl_cmd(FAMILIES["rail-mission-stub-vla"].first.spec, "t")
    i = cmd.index("--policy")
    assert cmd[i + 1].endswith(".yaml") and cmd[i + 2] == S.SITL_DEV_YAML_FLAG == "--dev-yaml", cmd
    assert "--bundle" not in cmd


def test_a_bundle_goes_to_the_rail_with_the_hash_it_must_hold():
    """With a node that declares the hash flag, a bundle goes as `--bundle
    <path> --expected-hash <hash>`: the spec's expected_hash, or, with none,
    the command is refused when the bundle cannot be read here. With a node
    that does not declare it, every bundle is refused: the node parses with
    parse_known_args, so the flag would be dropped and the bundle flown
    unpinned (review of 2026-10-12). Both nodes are simulated here, so the
    test does not depend on which one the tree holds."""
    spec = FAMILIES["nfz-head-on"].first.spec.model_copy(deep=True)
    d = spec.model_dump(mode="json", by_alias=True)
    d["policy"] = {"bundle_path": "demo/out/x/policy_bundle.tar.gz",
                   "expected_hash": "sha256:" + "ab" * 32}
    real = S.sitl_node_flags
    try:
        S.sitl_node_flags = lambda script=None: real(script) | {S.SITL_EXPECTED_HASH_FLAG}
        cmd = S.build_sitl_cmd(ScenarioSpec.model_validate(d), "t")
        i = cmd.index("--bundle")
        assert cmd[i + 1] == "demo/out/x/policy_bundle.tar.gz", cmd
        assert cmd[i + 2] == S.SITL_EXPECTED_HASH_FLAG and cmd[i + 3] == "sha256:" + "ab" * 32
        assert "--dev-yaml" not in cmd and "--policy" not in cmd
        # No expected_hash, and a bundle this host cannot read: refused,
        # never sent unpinned.
        nohash = dict(d, policy={"bundle_path": "demo/out/x/no_such_bundle.tar.gz"})
        try:
            S.build_sitl_cmd(ScenarioSpec.model_validate(nohash), "t")
        except S.UnsupportedScenario as e:
            assert "cannot be pinned" in str(e), e
        else:
            raise AssertionError("an unreadable bundle went to the rail with no hash")
        S.sitl_node_flags = lambda script=None: real(script) - {S.SITL_EXPECTED_HASH_FLAG}
        try:
            S.build_sitl_cmd(ScenarioSpec.model_validate(d), "t")
        except S.UnsupportedScenario as e:
            assert S.SITL_EXPECTED_HASH_FLAG in str(e) and "unpinned" in str(e), e
        else:
            raise AssertionError("a bundle went to a node that cannot pin it")
    finally:
        S.sitl_node_flags = real


def test_the_node_accepts_the_yaml_policy_command():
    """The rail's own mission (a YAML policy) goes with --dev-yaml, which the
    node declares (as guardrail.manifest's DEV_YAML_FLAG since 2026-10-12;
    the same string here). Every flag of that command must be one the node
    accepts: an undeclared one would not stop the launch (parse_known_args)
    but would be ignored."""
    flags = _node_flags()
    from guardrail import manifest
    assert manifest.DEV_YAML_FLAG == S.SITL_DEV_YAML_FLAG, manifest.DEV_YAML_FLAG
    assert S.SITL_DEV_YAML_FLAG in flags, f"{S.SITL_NODE} does not declare --dev-yaml"
    passed = {a for a in S.build_sitl_cmd(FAMILIES["rail-mission-stub-vla"].first.spec,
                                          "t")[5:] if a.startswith("--")}
    assert passed <= flags, passed - flags
    assert S.sitl_node_flags() == flags, "the harness reads the node differently"


def test_a_bundle_never_reaches_the_node_unpinned():
    """REGRESSION (review of 2026-10-12). build_sitl_cmd sent `--expected-hash`
    with every bundle while sitl/ros2_shield_node.py did not declare it, so
    the launch would fly the bundle unpinned, and test_sweep's flag check had
    been widened to let the flag through. Against the node in this tree:
    either it declares the flag and the command carries it, or the bundle
    is refused. (HEAD sent `--bundle <path>` with no hash at all.)"""
    flags = _node_flags()
    d = FAMILIES["rail-mission-stub-vla"].first.spec.model_dump(mode="json", by_alias=True)
    d["policy"] = {"bundle_path": "bundles/x.tar.gz", "expected_hash": "sha256:" + "1" * 64}
    try:
        cmd = S.build_sitl_cmd(ScenarioSpec.model_validate(d), "t")
    except S.UnsupportedScenario as e:
        assert S.SITL_EXPECTED_HASH_FLAG not in flags, "refused although the node pins"
        assert S.SITL_EXPECTED_HASH_FLAG in str(e), e
        return None
    assert S.SITL_EXPECTED_HASH_FLAG in flags, "sent a flag the node drops"
    i = cmd.index(S.SITL_EXPECTED_HASH_FLAG)
    assert cmd[i + 1] == "sha256:" + "1" * 64, cmd
    assert {a for a in cmd[5:] if a.startswith("--")} <= flags


def test_the_sitl_rail_refuses_what_only_the_headless_harness_flies():
    """A battery hook is a stressor the rail cannot inject, and the 3 s
    forecast arm is a headless comparison (the node builds its own Shield):
    both are refused before anything is launched."""
    d = FAMILIES["rail-mission-stub-vla"].first.spec.model_dump(mode="json", by_alias=True)
    assert S.sitl_refusal(ScenarioSpec.model_validate(d)) is None
    d["stress"] = {"battery_critical_at_s": 10.0}
    assert "stressors" in (S.sitl_refusal(ScenarioSpec.model_validate(d)) or "")
    with tempfile.TemporaryDirectory() as td:
        rc = _main(["--backend", "sitl", "--forecast-arms", "grant", "3s",
                    "--only", "speed-cap", "--out", str(Path(td) / "s.json"),
                    "--runs", str(Path(td) / "runs")])
        assert rc == 2 and not (Path(td) / "s.json").exists(), rc


def _hover(**stress):
    return ScenarioSpec.model_validate({
        "scenario_id": "hover-test", "template": "wind_disturbance",
        "description": "test", "ticks": 100, "escalation": False,
        "mission": {"start_pose": {"x": -100.0, "y": -100.0, "up": 15.0},
                    "pilot": {"type": "constant", "action": {"vx": 0.0}}},
        "policy": {"path": "policies/sim_demo_policy.yaml"},
        "stress": stress, "expected_kpis": {"P0_escape_rate": 0}})


def test_a_rerun_flies_the_recorded_defaults_wind_model_and_arm():
    """The extras keys a rerun cannot do without, each shown to matter: the
    LIBRARY's defaults (here 120 ticks, not the class default), the wind
    model (open-loop drifts where the default holds), the arm record (it is
    written into the re-flown episode_start), and the events
    (`scenario_events` is the cell's own list, not an empty one)."""
    from guardrail.scenario_spec import Defaults
    dfl = Defaults(ticks=120)
    # No ticks of its own, so the library's count is the one flown.
    spec = _hover(wind_speed_mps=3.0, wind_dir_deg=90.0).model_copy(update={"ticks": None})
    arm = {"arm": "paraphrase-0", "prefix": "on", "pilot_reads_text": False,
           "prompt_sha256": "sha256:" + "2" * 64}
    run = S.run_episode(spec, dfl, 7, arm=arm, wind_model="open_loop")
    assert len(run.rows) == 120 and run.extra["final"]["y"] > -90.0, run.extra["final"]
    ex = json.loads(json.dumps(S._json_safe(S.episode_extras(
        spec, dfl, 7, run, arm=arm, wind_model="open_loop"))))
    assert ex["library_defaults"]["ticks"] == 120 and ex["wind_model"] == "open_loop", ex
    again = S.rerun_from_extras(ex)
    assert _norm(again.rows) == _norm(run.rows)
    assert again.events[0]["arm"] == arm and again.events[0]["wind_model"] == "open_loop"
    spawn = FAMILIES["dyn-nfz-spawn-on-top"].first.spec
    r2 = S.run_episode(spawn, LIBRARY.defaults, 0)
    ex2 = S.episode_extras(spawn, LIBRARY.defaults, 0, r2)
    assert ex2["scenario_events"] and ex2["scenario_events"] == ex2["scenario_spec"]["events"]


def test_refused_hot_applies_make_no_generation():
    """`generations` lists the policy injected at t = 0 and every hot-apply the
    Shield ACCEPTED; an event it refused (expected so) made none."""
    base = FAMILIES["nfz-head-on"].first.spec.model_dump(mode="json", by_alias=True)
    base["events"] = [
        {"at_sim_t": 1.0, "type": "deactivate_rule",
         "payload": {"rule_id": "no-such-rule", "expect_refused": True}},
        {"at_sim_t": 2.0, "type": "spawn_polygon_fence",
         "payload": {"id": "extra", "width_m": 4, "height_m": 4,
                     "ahead_of_vehicle_m": 60}}]
    spec = ScenarioSpec.model_validate(base)
    run = S.run_episode(spec, LIBRARY.defaults, 0)
    assert any(e.get("refused") for e in run.events if e["type"] == "hot_apply")
    gens = S.episode_extras(spec, LIBRARY.defaults, 0, run)["generations"]
    assert [(g["generation"], g["event"]) for g in gens] == [
        (0, None), (1, "spawn_polygon_fence")], gens


def test_a_bundle_policy_reruns_from_the_extras_alone():
    """A signed bundle of the repository's size is inlined (base64) with its
    file hash, so the episode re-runs with no bundles/ directory at all
    (root pointed at an empty one); a tampered archive is refused."""
    import base64
    d = FAMILIES["nfz-head-on"].first.spec.model_dump(mode="json", by_alias=True)
    d.pop("paraphrase", None)
    d["ab"] = False
    d["policy"] = {"bundle_path": "bundles/fase3-sim-demo-v0.1.1.tar.gz"}
    spec = ScenarioSpec.model_validate(d)
    run = S.run_episode(spec, LIBRARY.defaults, 0)
    ex = json.loads(json.dumps(S.episode_extras(spec, LIBRARY.defaults, 0, run)))
    pol = ex["policy"]
    assert pol["kind"] == "bundle" and pol["yaml_text"] is None and pol["bundle_b64"], pol
    with tempfile.TemporaryDirectory() as empty:
        again = S.rerun_from_extras(ex, root=Path(empty))
    assert again.rows == run.rows
    bad = copy.deepcopy(ex)
    raw = bytearray(base64.b64decode(bad["policy"]["bundle_b64"]))
    raw[-1] ^= 0xFF
    bad["policy"]["bundle_b64"] = base64.b64encode(bytes(raw)).decode("ascii")
    try:
        S.rerun_from_extras(bad)
    except ValueError as e:
        assert "hashes to" in str(e), e
    else:
        raise AssertionError("a tampered inlined bundle was flown")


def test_main_flies_the_wind_model_it_is_given():
    """`--wind-model open_loop` reaches every episode main() flies: the hover
    in a 3 m/s crosswind drifts about 30 m on it and holds on the default."""
    fam = json.loads(_hover(wind_speed_mps=3.0, wind_dir_deg=90.0)
                     .model_dump_json(by_alias=True))
    got = {}
    for model in ("guided", "open_loop"):
        with tempfile.TemporaryDirectory() as td:
            raw = yaml.safe_load((ROOT / "experiments" / "scenarios.yaml")
                                 .read_text(encoding="utf-8"))
            lib = Path(td) / "lib.yaml"
            lib.write_text(yaml.safe_dump({"defaults": raw["defaults"], "scenarios": [fam]}),
                           encoding="utf-8")
            out = Path(td) / "s.json"
            _main(["--library", str(lib), "--out", str(out), "--no-bundles",
                   "--wind-model", model])
            kpi = json.loads(out.read_text(encoding="utf-8"))["results"][0]["kpi"]
            got[model] = (kpi["wind_model"], kpi["final"]["y"])
    assert got["guided"] == ("guided", -100.0), got
    assert got["open_loop"][0] == "open_loop" and got["open_loop"][1] > -71.0, got


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
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
