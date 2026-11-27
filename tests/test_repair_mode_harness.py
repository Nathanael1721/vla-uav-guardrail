"""The repair-mode selection on every harness that flies or sweeps the Shield
(WP3-06, 2026-11-27).

Run either way:
    pytest tests/test_repair_mode_harness.py -v
    python tests/test_repair_mode_harness.py

The grant's safety-shield.md says the repair operators are tried in priority
order and the first that produces a feasible action within theta wins. The
Shield has had that selection since the repair library landed
(Shield(repair_mode="first_feasible"), guardrail/repair.py THE MODES), off by
default, but no flying or library harness could select it: only the offline
bench (experiments/bench_shield_50rules.py) and tools/prefix_eval.py. So the
default ("chain") was never compared against it on the scenario library or
on a rail.

Pinned here:
  * the harnesses' named default (guardrail.repair.DEFAULT_REPAIR_MODE) is
    the Shield's own, "chain", so every harness flies what it flew before
    unless asked;
  * the ROS 2 node takes --repair-mode {chain, first_feasible} (default
    chain) into rail_shield and records it in metrics.json
    shield_forecast.repair_mode; run_sitl_demo.py likewise;
  * a rail campaign asks the node for it (--rail-repair-mode, through
    rail_extra_args) and refuses a run whose record disagrees
    (repair_mode_problems), as --rail-arc-forecast does;
  * the headless sweep's --repair-mode reaches the flying Shield (only),
    is recorded per episode (extra, episode_start) and in a bundle
    (arm.repair_mode, extras schema /6, written only off the default so a
    chain bundle stays /5), and a /6 bundle re-flies on its own mode;
  * the options are refused where they would mean nothing (a rail's flag on
    headless, the headless flag on a rail, either on --publish);
  * tools/replay_at_revision.py knows the recorded key instead of refusing
    every run flown since.
"""
import inspect
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
sys.path.insert(0, str(ROOT / "sitl"))
sys.path.insert(0, str(ROOT / "tools"))

from guardrail import repair as R                                     # noqa: E402
from guardrail.shield import Shield                                   # noqa: E402

# A library cell on which the two selections fly differently (the A/B in
# docs/data/repair_mode_ab_20261127/: 300 repairs on the chain, 245 and a
# failed time-to-safe gate on first_feasible). nfz-head-on, for one, flies
# the same either way.
DIFFERING = "time-window-opens-on-sideways-pilot"


def _lib():
    from guardrail.scenario_spec import Library
    return Library(ROOT / "experiments" / "scenarios.yaml")


def _spec(lib, sid):
    return lib.by_id()[sid].first.spec.model_copy(deep=True)


def test_the_harnesses_default_is_the_shields_own():
    assert R.DEFAULT_REPAIR_MODE == "chain" and R.DEFAULT_REPAIR_MODE in R.REPAIR_MODES
    assert inspect.signature(Shield).parameters["repair_mode"].default == R.DEFAULT_REPAIR_MODE
    assert set(R.REPAIR_MODES) == {"chain", "first_feasible"}


def test_the_ros2_node_takes_the_mode_into_its_shield_and_records_it():
    import ros2_shield_node as N
    a, _ = N.make_parser().parse_known_args([])
    assert a.repair_mode == "chain"
    a, _ = N.make_parser().parse_known_args(["--repair-mode=first_feasible"])
    assert a.repair_mode == "first_feasible"
    try:
        N.make_parser().parse_args(["--repair-mode=greedy"])
    except SystemExit:
        pass
    else:
        raise AssertionError("an unknown repair mode was parsed")
    src = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    i = src.index("self.shield = rail_shield(self.policy")
    assert "repair_mode=self.repair_mode" in src[i:i + 600]
    assert '"repair_mode": getattr(self.shield, "repair_mode", DEFAULT_REPAIR_MODE)' in src
    import sweep_scenarios as S
    assert "--repair-mode" in S.sitl_node_flags()


def test_the_pymavlink_sitl_rail_takes_the_mode_and_records_it():
    src = (ROOT / "sitl" / "run_sitl_demo.py").read_text(encoding="utf-8")
    assert 'ap.add_argument("--repair-mode", choices=["chain", "first_feasible"], ' \
           'default="chain"' in src
    i = src.index("shield = rail_shield(policy, lookahead_s=LOOKAHEAD_S")
    assert "repair_mode=repair_mode" in src[i:i + 400]
    assert '"repair_mode": getattr(shield, "repair_mode", repair_mode)' in src
    # The rail's PathRepair setting is recorded too, so a replay rebuilds
    # the chain it flew (FOLLOW_RAIL_PATH_REPAIR is off; the Shield's
    # default is on).
    assert '"path_repair": bool(getattr(shield, "path_repair", False))' in src
    # The demo's choices are the library's.
    assert ("chain", "first_feasible") == R.REPAIR_MODES


def test_a_campaign_asks_the_node_for_the_mode_and_refuses_a_run_that_did_not_fly_it():
    import sweep_scenarios as S
    ff = ["--repair-mode=first_feasible"]
    assert S.rail_extra_args(ff) == ff
    assert S.rail_extra_args(["--repair-mode=chain", "--arc-forecast=on"]) == \
        ["--repair-mode=chain", "--arc-forecast=on"]
    for bad in (["--repair-mode=greedy"], ["--repair-mode", "first_feasible"],
                ["--repair-mode=chain", "--repair-mode=first_feasible"]):
        try:
            S.rail_extra_args(bad)
        except S.UnsupportedScenario:
            pass
        else:
            raise AssertionError(f"{bad} was accepted")
    assert S.rail_repair_mode_asked(None) == "chain"
    assert S.rail_repair_mode_asked(ff) == "first_feasible"
    flew_ff = {"shield_forecast": {"mode": "commanded", "repair_mode": "first_feasible"}}
    flew_chain = {"shield_forecast": {"mode": "commanded", "repair_mode": "chain"}}
    old = {"shield_forecast": {"mode": "commanded"}}       # a node before the flag
    assert S.repair_mode_problems(flew_ff, ff) == []
    assert S.repair_mode_problems(flew_chain, None) == []
    assert S.repair_mode_problems(old, None) == [] == S.repair_mode_problems({}, None)
    for m in (flew_chain, old, {}):
        assert "not the selection" in S.repair_mode_problems(m, ff)[0], m
    assert "not the selection" in S.repair_mode_problems(flew_ff, None)[0]
    src = (ROOT / "experiments" / "sweep_scenarios.py").read_text(encoding="utf-8")
    # Both rail launches check the record (run_sitl, run_hil), and the
    # campaign's argument list carries the flag.
    assert src.count("repair_mode_problems(metrics, extra_args)") == 2
    assert src.count("rail_args = (rail_arc_args + rail_repair_args + rail_hold_args "
                     "+ rail_fsm) or None") == 1


def test_the_options_are_refused_where_they_mean_nothing():
    import sweep_scenarios as S
    # The rail's flag on the headless harness, the headless flag on a rail,
    # and either one on the published file.
    assert S.main(["--rail-repair-mode", "first_feasible", "--only", "nfz-head-on"]) == 2
    assert S.main(["--backend", "sitl", "--repair-mode", "first_feasible",
                   "--only", "nfz-head-on", "--sitl-dry-run"]) == 2
    assert S.main(["--publish", "--repair-mode", "first_feasible"]) == 2
    assert S.main(["--rescore-rail", "x", "--out", "y", "--rail-repair-mode",
                   "first_feasible"]) == 2


def test_a_first_feasible_run_gets_its_own_folder_tag():
    import sweep_scenarios as S
    assert S.comparison_tag(["grant"], ["ideal"]) is None
    assert S.comparison_tag(["grant"], ["ideal"], repair_mode="chain") is None
    assert S.comparison_tag(["grant"], ["ideal"],
                            repair_mode="first_feasible") == "repair-first_feasible"
    assert S.comparison_tag(["grant", "3s"], ["ideal"], repair_mode="first_feasible") == \
        "forecast-3s_repair-first_feasible"


def test_the_headless_mode_reaches_the_flying_shield_and_is_recorded():
    import sweep_scenarios as S
    lib = _lib()
    spec = _spec(lib, DIFFERING)
    dflt = S.run_episode(spec, lib.defaults, 1001)
    chain = S.run_episode(spec, lib.defaults, 1001, repair_mode="chain")
    ff = S.run_episode(spec, lib.defaults, 1001, repair_mode="first_feasible")
    # The default is the chain, row for row.
    assert dflt.rows == chain.rows
    assert dflt.extra["repair_mode"] == "chain" == dflt.events[0]["repair_mode"]
    assert ff.extra["repair_mode"] == "first_feasible" == ff.events[0]["repair_mode"]
    # The selection changes the flight on this cell (not a label only).
    assert [r["emitted"] for r in ff.rows] != [r["emitted"] for r in chain.rows], \
        "first_feasible flew the chain's actions: the mode never reached the Shield"
    assert sum(1 for r in ff.rows if r["repairs"]) > 10, "the Shield never acted: blind"
    try:
        S.run_episode(spec, lib.defaults, 1001, repair_mode="greedy")
    except ValueError as e:
        assert "repair mode" in str(e)
    else:
        raise AssertionError("an unknown repair mode was flown")


def test_a_first_feasible_bundle_names_its_mode_and_re_flies_on_it():
    import sweep_scenarios as S
    lib = _lib()
    spec = _spec(lib, DIFFERING)
    chain = S.run_episode(spec, lib.defaults, 1001)
    ff = S.run_episode(spec, lib.defaults, 1001, repair_mode="first_feasible")
    ex_c = json.loads(json.dumps(S.episode_extras(spec, lib.defaults, 1001, chain)))
    ex_f = json.loads(json.dumps(S.episode_extras(spec, lib.defaults, 1001, ff,
                                                  repair_mode="first_feasible")))
    # A chain bundle is /5 exactly as before: no new key.
    assert ex_c["extras_schema"] == S.EXTRAS_SCHEMA == "sweep-episode-extras/5"
    assert "repair_mode" not in ex_c["arm"]
    assert ex_f["extras_schema"] == S.EXTRAS_SCHEMA_REPAIR == "sweep-episode-extras/6"
    assert ex_f["arm"]["repair_mode"] == "first_feasible"
    # The run's own record wins over the argument (as for the recovery law).
    ex_f2 = json.loads(json.dumps(S.episode_extras(spec, lib.defaults, 1001, ff)))
    assert ex_f2["arm"]["repair_mode"] == "first_feasible"
    again = S.rerun_from_extras(ex_f)
    assert again.rows == ff.rows and again.extra["repair_mode"] == "first_feasible"
    assert S.rerun_from_extras(ex_c).rows == chain.rows

    def refused(ex):
        try:
            S.rerun_from_extras(ex)
        except ValueError as e:
            return str(e)
        raise AssertionError("re-flown on a guess")
    nameless = json.loads(json.dumps(ex_f))
    del nameless["arm"]["repair_mode"]
    assert "arm.repair_mode" in refused(nameless)
    contradicts = json.loads(json.dumps(ex_c))
    contradicts["arm"]["repair_mode"] = "first_feasible"
    assert "predates arm.repair_mode" in refused(contradicts)
    unknown = json.loads(json.dumps(ex_f))
    unknown["arm"]["repair_mode"] = "greedy"
    assert "arm.repair_mode" in refused(unknown)


def test_the_replay_at_revision_rebuilds_the_recorded_mode():
    """metrics.json shield_forecast keys outside KNOWN_FORECAST_KEYS refuse
    the episode, so without the key every rail run flown since the node
    records its mode would be refused (the end-to-end rebuild is in
    tests/test_replay_at_revision.py)."""
    import replay_at_revision as RA
    assert "repair_mode" in RA.KNOWN_FORECAST_KEYS
    src = (ROOT / "tools" / "replay_at_revision.py").read_text(encoding="utf-8")
    assert 'kw["repair_mode"] = rm' in src


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except Exception as e:                      # noqa: BLE001
            failed += 1
            print(f"FAIL  {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
