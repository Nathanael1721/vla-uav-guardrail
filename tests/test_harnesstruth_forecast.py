"""Every headless KPI from the grant's Shield configuration: the 5 s / 0.1 s
forecast, and the labelled 3 s comparison arm kept out of the headline.

    python tests/test_harnesstruth_forecast.py      (no pytest needed)

WHY THIS FILE EXISTS

The Safety Shield page locks the forecast: "5 s of predicted trajectory at
10 Hz = 50 future poses". guardrail/shield.py has defaulted to it since
2026-10-07, but the headless harness built its flying, oracle and checker
Shields with `lookahead_s=look, dt=0.5`, and the library's default `look` was
3.0: six poses. Every number the sweep published came from a Shield the grant
does not describe. The old forecast survives only as a comparison arm, which
must be labelled in every bundle and kept out of every headline number.

Shown failing first: every test below fails or errors when this file is run
from a pristine copy of HEAD 4418ae0 (`git archive HEAD | tar -x`, this file
copied into its tests/).
"""
from __future__ import annotations

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
from guardrail.scenario_spec import Defaults, Library             # noqa: E402

LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")
FAMILIES = LIBRARY.by_id()
SKIP = "SKIP"


def _main(argv):
    """sweep_scenarios.main, with an argparse exit (an unknown flag on an older
    harness) reported as this test's failure rather than ending the run."""
    try:
        return S.main(argv)
    except SystemExit as e:
        raise AssertionError(f"main() exited {e.code} on {argv}") from None


def _spy_shields(fn):
    """Run fn() with every Shield the harness builds recorded: (lookahead_s,
    dt, escalation) per construction."""
    seen = []
    real = S.Shield

    class Spy(real):
        def __init__(self, policy, *a, **k):
            super().__init__(policy, *a, **k)
            seen.append((self.lookahead_s, self.dt, k.get("escalation")))
    S.Shield = Spy
    try:
        out = fn()
    finally:
        S.Shield = real
    return seen, out


def test_the_library_defaults_are_the_grants_forecast():
    """Defaults() and the shipped library both default to guardrail.shield's
    LOOKAHEAD_S / LOOKAHEAD_DT_S (5.0 s at 0.1 s), one copy of the numbers."""
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    assert (LOOKAHEAD_S, LOOKAHEAD_DT_S) == (5.0, 0.1)
    d = Defaults()
    assert (d.lookahead_s, d.lookahead_dt_s) == (LOOKAHEAD_S, LOOKAHEAD_DT_S), d
    lib = LIBRARY.defaults
    assert (lib.lookahead_s, lib.lookahead_dt_s) == (5.0, 0.1), lib
    raw = yaml.safe_load((ROOT / "experiments" / "scenarios.yaml").read_text(encoding="utf-8"))
    assert raw["defaults"]["lookahead_s"] == 5.0 and raw["defaults"]["lookahead_dt_s"] == 0.1


def test_every_shield_of_an_episode_is_built_with_the_grants_forecast():
    """The flying Shield, the ground-truth oracle (GPS noise) and the
    autopilot checker (escalation on): all three at 5 s / 0.1 s."""
    spec = FAMILIES["gps-noise-beyond-margin"].first.spec
    assert spec.stress.degrades_perception and spec.escalation and spec.shield
    seen, run = _spy_shields(lambda: S.run_episode(spec, LIBRARY.defaults, 1001))
    assert len(seen) == 3, seen
    assert {(look, dt) for look, dt, _ in seen} == {(5.0, 0.1)}, seen
    assert run.extra["forecast"] == {"arm": "grant", "lookahead_s": 5.0,
                                     "lookahead_dt_s": 0.1, "poses": 50,
                                     "grant_config": True}, run.extra["forecast"]
    start = run.events[0]
    assert start["forecast"]["grant_config"] is True, start


def test_a_spec_may_override_the_forecast_and_is_then_not_the_grants():
    spec = FAMILIES["speed-cap"].first.spec.model_copy(
        update={"lookahead_s": 4.0, "lookahead_dt_s": 0.2})
    seen, run = _spy_shields(lambda: S.run_episode(spec, LIBRARY.defaults, 0))
    assert {(look, dt) for look, dt, _ in seen} == {(4.0, 0.2)}, seen
    assert run.extra["forecast"]["grant_config"] is False
    assert run.extra["forecast"]["poses"] == 20


def test_the_3s_arm_is_the_harness_forecast_until_2026_10_12():
    """forecast="3s" builds every Shield at 3.0 s / 0.5 s (6 poses), what the
    harness built before; tests/test_sweep.py's LEGACY pins fly it and hold
    the pre-change end states exactly."""
    spec = FAMILIES["gps-noise-beyond-margin"].first.spec
    seen, run = _spy_shields(lambda: S.run_episode(spec, LIBRARY.defaults, 1001,
                                                   forecast="3s"))
    assert {(look, dt) for look, dt, _ in seen} == {(3.0, 0.5)}, seen
    fc = run.extra["forecast"]
    assert fc["arm"] == "3s" and fc["poses"] == 6 and fc["grant_config"] is False, fc
    try:
        S.run_episode(spec, LIBRARY.defaults, 1001, forecast="2s")
    except ValueError as e:
        assert "forecast arm" in str(e), e
    else:
        raise AssertionError("an unknown forecast arm was flown")


def test_the_forecast_changes_the_flight():
    """The arm is not plumbing: nfz-head-on is held further from the zone by a
    forecast that sees 5 s ahead than by one that sees 3 s (the null that
    makes the arm worth keeping)."""
    spec = FAMILIES["nfz-head-on"].first.spec
    grant = S.run_episode(spec, LIBRARY.defaults, 0).extra
    old = S.run_episode(spec, LIBRARY.defaults, 0, forecast="3s").extra
    assert old["final"] == {"x": -5.6, "y": 15.0, "up": 15.0}, old["final"]
    assert grant["final"]["x"] < old["final"]["x"] - 1.0, (grant["final"], old["final"])


def test_the_comparison_arm_needs_the_grant_arm_beside_it():
    from guardrail.scenario_spec import check_forecast_arms, expand
    assert check_forecast_arms(["grant", "3s"]) == ["grant", "3s"]
    for bad in (["3s"], [], ["grant", "grant"], ["grant", "4s"]):
        try:
            check_forecast_arms(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted forecast arms {bad}")
    eps, info = expand(LIBRARY, only=["speed-cap"], forecast_arms=["grant", "3s"])
    assert [e.forecast for e in eps] == ["grant", "3s"], eps
    assert eps[1].arm_tag == "forecast-3s" and eps[1].run_id == "speed-cap#forecast-3s"
    assert info["arms"]["forecast_comparison_episodes"] == 1 and info["cells"] == 1
    assert info["arms"]["extra_arm_episodes"] == 0, info["arms"]
    # `episodes` is the grant forecast's count; the copies are counted apart.
    assert (info["episodes"], info["episodes_with_comparison"]) == (1, 2), info
    assert "forecast-3s" in eps[1].dir_name("T")


def _sweep(families, *extra):
    with tempfile.TemporaryDirectory() as td:
        raw = yaml.safe_load((ROOT / "experiments" / "scenarios.yaml").read_text(encoding="utf-8"))
        lib = Path(td) / "lib.yaml"
        lib.write_text(yaml.safe_dump({"defaults": raw["defaults"], "scenarios": families}),
                       encoding="utf-8")
        out, runs = Path(td) / "s.json", Path(td) / "runs"
        rc = _main(["--library", str(lib), "--out", str(out), "--runs", str(runs), *extra])
        doc = json.loads(out.read_text(encoding="utf-8")) if out.is_file() else None
        extras = {}
        if runs.is_dir():
            for d in runs.iterdir():
                p = d / "manifest_extras.json"
                if p.is_file():
                    extras[d.name] = json.loads(p.read_text(encoding="utf-8"))
    return rc, doc, extras


def test_the_comparison_arm_is_labelled_and_kept_out_of_every_headline():
    """With --forecast-arms grant 3s: the 3 s episodes are judged but never
    counted in _counts or the exit code, their cells are `/forecast-3s` rows
    marked grant_config false, their bundles say so in manifest_extras, and
    `_forecast_comparison` lists what moved (nfz-head-on is held further out
    by the 5 s forecast, so it moves)."""
    fam = copy.deepcopy(LIBRARY.raw("nfz-head-on"))
    fam.pop("paraphrase", None)
    fam["ab"] = False
    rc, doc, extras = _sweep([fam], "--forecast-arms", "grant", "3s", "--seed", "0")
    assert rc == 0, rc
    assert doc["_counts"]["pass"] == 1 and sum(doc["_counts"].values()) == 1, doc["_counts"]
    assert [r["id"] for r in doc["results"]] == ["nfz-head-on"], [r["id"] for r in doc["results"]]
    cmp_ = doc["_forecast_comparison"]
    assert cmp_["grant_config"] is False and cmp_["forecast"]["poses"] == 6, cmp_["forecast"]
    assert [r["id"] for r in cmp_["results"]] == ["nfz-head-on#forecast-3s"]
    assert cmp_["episodes_moved"] == 1 and cmp_["moved"][0]["id"] == "nfz-head-on", cmp_["moved"]
    cells = doc["_cell_rollup"]
    assert cells["static_nfz/nfz-head-on/shield-on"]["grant_config"] is True, cells
    assert cells["static_nfz/nfz-head-on/shield-on/forecast-3s"]["grant_config"] is False
    assert doc["_rollup"]["_comparison_forecast_3s_shield_on"]["grant_config"] is False
    assert doc["_rollup"]["_all_shield_on"]["episodes"] == 1
    assert doc["_true_state"]["shield_on_episodes"] == 1
    assert doc["_failsafe_labels"]["labelled"] == 1
    arms = {name: ex["forecast"] for name, ex in extras.items()}
    assert len(arms) == 2, arms
    assert {(f["arm"], f["grant_config"], f["lookahead_s"], f["lookahead_dt_s"])
            for f in arms.values()} == {("grant", True, 5.0, 0.1), ("3s", False, 3.0, 0.5)}
    assert all(("forecast-3s" in n) == (f["arm"] == "3s") for n, f in arms.items()), arms


def test_a_failing_comparison_episode_never_fails_the_run():
    """The 3 s arm is reported, never gated: a gate it alone breaks (here a
    bound on how close nfz-head-on may end to the zone, which the 3 s forecast
    lets it approach) leaves the run green and the grant arm a pass."""
    fam = copy.deepcopy(LIBRARY.raw("nfz-head-on"))
    fam.pop("paraphrase", None)
    fam["ab"] = False
    fam["expected_kpis"] = dict(fam["expected_kpis"], min_true_dist_to_fence_m={"min": 15.0})
    rc, doc, _ = _sweep([fam], "--forecast-arms", "grant", "3s", "--no-bundles")
    grant = doc["results"][0]
    old = doc["_forecast_comparison"]["results"][0]
    assert grant["status"] == "pass", (grant["status"], grant["gates_failed"])
    assert old["status"] == "fail", (old["status"], old["kpi"]["min_true_dist_to_fence_m"])
    assert doc["_forecast_comparison"]["counts"]["fail"] == 1
    assert rc == 0 and doc["_counts"]["fail"] == 0, (rc, doc["_counts"])


def test_publish_flies_both_forecasts_and_refuses_to_change_them():
    """--publish takes neither --forecast-arms nor --wind-model. Run against a
    redirected published path and a ROOT with no policies, so a regression
    of the refusal fails within seconds instead of flying the library for
    six minutes over the tracked docs/data/scenario_sweep.json (review of
    2026-10-12)."""
    assert tuple(S.PUBLISH_FORECAST_ARMS) == ("grant", "3s")
    real = (S.PUBLISHED_SWEEP, S.ROOT)
    with tempfile.TemporaryDirectory() as td:
        S.PUBLISHED_SWEEP = Path(td) / "published.json"
        S.ROOT = Path(td)
        try:
            assert _main(["--publish", "--forecast-arms", "grant"]) == 2
            assert _main(["--publish", "--wind-model", "open_loop"]) == 2
        finally:
            S.PUBLISHED_SWEEP, S.ROOT = real
        assert not (Path(td) / "published.json").exists(), "a refused --publish wrote"
        assert not (Path(td) / "demo").exists(), "a refused --publish flew episodes"


def test_the_3s_arm_keeps_a_spec_that_overrides_the_horizon():
    """The comparison arm is what the harness built before: the spec's own
    lookahead_s when it gives one, else 3.0 s, always at 0.5 s."""
    spec = FAMILIES["speed-cap"].first.spec.model_copy(update={"lookahead_s": 4.0})
    fc = S.forecast_for(spec, LIBRARY.defaults, "3s")
    assert (fc["lookahead_s"], fc["lookahead_dt_s"], fc["poses"]) == (4.0, 0.5, 8), fc
    assert fc["grant_config"] is False
    plain = S.forecast_for(FAMILIES["speed-cap"].first.spec, LIBRARY.defaults, "3s")
    assert (plain["lookahead_s"], plain["lookahead_dt_s"]) == (3.0, 0.5), plain


def test_the_forecast_knobs_are_validated_where_they_are_read():
    """A profile that flies only the comparison arm, a non-finite battery
    time, and two cells that differ only in writing the library's forecast
    step out: each refused or caught at load, never flown."""
    from guardrail.scenario_spec import Cell, Profile, Stress, duplicate_cells
    base = dict(profile="t", cadence="c", grant_scope="g", grant_sim_speedup="s",
                seeds=[1], expected_cells=[1, 2])
    assert Profile.model_validate(dict(base, forecast_arms=["grant", "3s"]))
    for arms in (["3s"], ["grant", "grant"]):
        try:
            Profile.model_validate(dict(base, forecast_arms=arms))
        except ValueError:
            continue
        raise AssertionError(f"a profile flew forecast arms {arms}")
    for bad in (float("nan"), float("inf")):
        try:
            Stress(battery_critical_at_s=bad)
        except ValueError:
            continue
        raise AssertionError(f"battery_critical_at_s={bad} was accepted")
    spec = FAMILIES["speed-cap"].first.spec
    same = spec.model_copy(update={"lookahead_dt_s": LIBRARY.defaults.lookahead_dt_s})
    other = spec.model_copy(update={"lookahead_dt_s": 0.2})
    cells = [Cell(episode_id=f"c{i}", scenario_id="speed-cap", params={}, spec=s)
             for i, s in enumerate((spec, same, other))]
    assert duplicate_cells(cells, LIBRARY.defaults) == [("c0", "c1")]


def test_the_run_summaries_count_what_they_say():
    """`episodes_rtl_failed` counts the episodes whose RTL failed, not all of
    them; a cell row SUMS the per-mode autopilot ticks of its episodes; a
    row whose episodes recorded no forecast (the SITL rail's node builds its
    own Shield) is grant_config None, not False; and the published copy
    drops the comparison arm's per-episode items but keeps its summaries."""
    def item(rtl, by_mode):
        return {"status": "pass", "arm": "on",
                "kpi": {"rtl_failed_ticks": rtl, "autopilot_p0_flown_ticks_by_mode": by_mode,
                        "autopilot_p0_flown_ticks": sum(by_mode.values()),
                        "autopilot_mode_ticks": by_mode,
                        "forecast": {"grant_config": True}}}
    a, b = item(3, {"RTL": 2, "LAND": 1}), item(0, {"RTL": 5})
    ts = S.true_state_summary([a, b, item(0, {})])
    assert ts["episodes_rtl_failed"] == 1, ts["episodes_rtl_failed"]
    row = S.cell_extras([a["kpi"], b["kpi"]])
    assert row["autopilot_p0_flown_ticks_by_mode"] == {"LAND": 1, "RTL": 7}, row
    assert row["autopilot_ticks_by_mode"] == {"LAND": 1, "RTL": 7}, row
    assert row["grant_config"] is True and row["episodes_rtl_failed"] == 1
    sitl = S.cell_extras([{k: v for k, v in a["kpi"].items() if k != "forecast"}])
    assert sitl["grant_config"] is None, sitl
    doc = {"results": [], "_forecast_comparison": {"counts": {"pass": 1}, "results": [1, 2]}}
    pub = S.published_view(doc)
    assert "results" not in pub["_forecast_comparison"], pub["_forecast_comparison"]
    assert pub["_forecast_comparison"]["counts"] == {"pass": 1}
    assert "results" in doc["_forecast_comparison"], "published_view changed its input"


def test_the_failsafe_score_reports_distinct_flights_beside_episodes():
    """Copies of one flight (cells or seeds that flew the same KPI table under
    the same label) are counted once in `labelled_distinct_flights`, so a
    padded denominator shows beside the headline (review of 2026-10-12)."""
    def item(sid, trig, match, repairs):
        return {"status": "pass", "scenario_id": sid, "template": "t",
                "kpi": {"failsafe_triggered": trig, "failsafe_matches_label": match,
                        "repair_count": repairs}}
    s = S.failsafe_summary([item("a", False, 1.0, 5), item("a", False, 1.0, 5),
                            item("a", False, 1.0, 5), item("a", True, 0.0, 7),
                            item("b", False, 1.0, 5)])
    assert (s["labelled"], s["correct"]) == (5, 4), s
    assert (s["labelled_distinct_flights"], s["correct_distinct_flights"]) == (3, 2), s
    assert s["fail_safe_correctness_distinct_flights"] == round(2 / 3, 6)


def test_the_published_sweep_came_from_the_grants_forecast():
    """docs/data/scenario_sweep.json: every headline episode flew the grant's
    forecast, and the 3 s comparison sits beside it marked non-grant."""
    p = S.PUBLISHED_SWEEP
    if not p.is_file():
        print(f"      SKIP: {p} absent")
        return SKIP
    doc = json.loads(p.read_text(encoding="utf-8"))
    fcs = [(r.get("kpi") or {}).get("forecast") for r in doc["results"] if "kpi" in r]
    assert fcs and all(f and f["arm"] == "grant" and f["grant_config"] is True for f in fcs), \
        "a headline episode did not fly the grant's forecast"
    cmp_ = doc.get("_forecast_comparison")
    assert cmp_ and cmp_["grant_config"] is False and cmp_["forecast"]["poses"] == 6, cmp_
    marked = {k: v.get("grant_config") for k, v in doc["_cell_rollup"].items()}
    assert all(v is False for k, v in marked.items() if k.endswith("/forecast-3s"))
    assert all(v is True for k, v in marked.items() if not k.endswith("/forecast-3s"))
    assert doc["_scope"]["arms"]["forecast_arms"] == ["grant", "3s"]


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
