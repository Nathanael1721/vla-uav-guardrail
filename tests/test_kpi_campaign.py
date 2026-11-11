"""The KPI report written from a hil campaign (tools/kpi_report.py, 2026-11-11).

Run either way:
    pytest tests/test_kpi_campaign.py -v
    python tests/test_kpi_campaign.py

WHAT IS BEING CLAIMED (WP4, "a KPI report from a nightly stress run, with
the P0 violation escape rate at 0", due 30 Nov)

1. --campaign reads a hil campaign's sweep.json and takes its runs from the
   campaign's store (`_scope.hil.config.store`/<campaign>_*, or the folder of
   that name beside the summary when the store moved); a run the campaign
   did not accept (an `error` naming its mission) or does not list is NOT
   KPI-grade, with the campaign's reason, while the accepted runs keep the
   gate they had (is_kpi_grade plus the signed replay bundle bound to the run).
2. --rescore-rail applies the ADOPTED fail-safe reading of a
   `experiments/sweep_scenarios.py --rescore-rail` table to every run it
   names: the family table's fail-safe column reads it, the label each run
   flew with is kept beside it, a Shield-off control or a run the table
   leaves out has no label (readings are never mixed); and every reading is
   scored over the KPI-grade Shield-on runs with the point estimate, the
   Wilson 95 % lower bound and both verdicts (guardrail/fsm.py
   score_failsafe_triggers), its excluded runs named.
3. The tick budget reads `tick_ms`, `e2e_ms` and the perception rail's
   `cam_age_ms` / `cam_to_setpoint_ms` from every row of the WHOLE flight,
   pooled per rail with p50 / p95 / p99 / max and the count over 100 ms;
   a run's metrics.json summary stands in only where its rows have none and
   is never pooled; a measure nothing carries is "not measured", never 0.
4. The KPI summary leads the report: the P0 escape rate of the Shield-on
   runs against the Shield-off control, fail-safe correctness on the adopted
   reading, the tick budget, and the tracked KPIs overall and per template.
5. On the delivered campaign sm1111 (SKIP where demo/out/hil_sm1111 is
   absent; KPI_CAMPAIGN_DEMO_OUT points the test at another checkout's
   demo/out): P0 0.0 Shield on vs 0.65 control, r2 44/45, tick_ms max
   95.984 ms, e2e_ms p99 about 106 ms.

Shown failing first: against 0df7ee8 (the tree before this change) every
test here fails - kpi_report has no load_campaign, load_rescore,
apply_rescore, latency_section or kpi_summary, tick_timing reads neither
tick_ms nor e2e_ms, and main() has no --campaign or --rescore-rail.
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "tools", ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import load_policy                                  # noqa: E402
import kpi_report as KR                                            # noqa: E402
import test_wp4outputs_kpi_report as W                             # noqa: E402

SKIP = "SKIP"
P0 = {"rule_id": "nfz-square", "category": "geofence"}


def _rows(n, *, shield=True, escapes=0, tick=(5.0,), e2e=(60.0,)):
    """n rows of a flown run: tick_ms on every tick, e2e_ms on every other
    (a tick that re-flies an action has none); a Shield-on run repairs a P0
    violation on its first three ticks (so its 0 escapes is measured, not
    vacuous), a Shield-off control flies one on its first `escapes`."""
    slide = {"operator": "GeofenceSlide", "detail": "", "magnitude_m": 1.0,
             "axis": "lateral", "recovery": False}
    out = []
    for i in range(1, n + 1):
        r = W._row(i)
        r["tick_ms"] = tick[(i - 1) % len(tick)]
        r["e2e_ms"] = e2e[((i - 1) // 2) % len(e2e)] if i % 2 else None
        if shield and i <= 3:
            r.update(violations=[P0], repairs=[slide])
        if not shield and i <= escapes:
            r.update(violations=[P0], emitted_violations=[P0])
        out.append(r)
    return out


def _run(store: Path, name: str, pol, signer, rows, shield="on") -> Path:
    from guardrail.replay import write_replay
    run = W._hil_run(store, name, pol)
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    m = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
    m["shield"] = shield
    (run / "metrics.json").write_text(json.dumps(m), encoding="utf-8")
    (run / "kpi.json").unlink()
    write_replay(run, pol, run / f"{name}.replay.tar.gz", signer=signer)
    return run


def _campaign(root: Path, store: Path, *, name="cx") -> Path:
    """demo/out/hil_cx/sweep.json: three flights accepted, cx_004 not (its
    stress was not realised), one cell refused before flight."""
    summ = root / f"hil_{name}"
    summ.mkdir(parents=True, exist_ok=True)
    res = [{"id": "a", "status": "pass", "mission_tag": f"{name}_001_a", "run": None},
           {"id": "b", "status": "fail", "mission_tag": f"{name}_002_b",
            "gates_failed": ["x"]},
           {"id": "ctrl", "status": "known_failure", "mission_tag": f"{name}_003_ctrl"},
           {"id": "d", "status": "error",
            "error": f"mission {name}_004_d: the cell's stress was not realised"},
           {"id": "skipped-cell", "status": "skipped"}]
    doc = {"_command": "python experiments/sweep_scenarios.py --backend hil ...",
           "_backend": "hil", "_runtime_s": 12.5,
           "_manifest_common": {"code_revision": "feedc0de1234"},
           "_scope": {"profile": "smoke", "grant_scope": "~50 x 1", "cadence": "per build",
                      "hil": {"campaign": name, "flyable": 4,
                              "refused": {"skipped-cell": "no"},
                              "config": {"store": store.as_posix()}}},
           "_failsafe_labels": {"labelled": 2, "correct": 1, "fail_safe_correctness": 0.5,
                                "false_triggers": 1, "missed_triggers": 0},
           "results": res}
    (summ / "sweep.json").write_text(json.dumps(doc), encoding="utf-8")
    return summ


def _rescore(path: Path, name="cx") -> Path:
    """A --rescore-rail table: cx_001 triggered and is a false trigger under
    (a), correct under the adopted (r2) (its label read on the longer
    flight); cx_002 did not trigger and is correct under both; the control
    is Shield-off; cx_004 is not accepted."""
    def run(tag, shield, trig, accepted=True):
        return {"run": tag, "cell": tag.split("_", 2)[2], "seed": 1001,
                "before": {"status": "pass"} if accepted else None,
                "after": {}, "doc": {"metrics": {"shield": shield},
                                     "kpi": {"failsafe_triggered": trig}}}
    a, b, c, d = (f"{name}_001_a", f"{name}_002_b", f"{name}_003_ctrl", f"{name}_004_d")
    tag = lambda t: {"run": t, "cell": t.split("_", 2)[2], "seed": 1001}   # noqa: E731
    empty = {k: [] for k in ("false", "missed", "censored", "undecided", "unlabelled",
                             "not_measured", "not_accepted", "label_flown_longer")}
    doc = {"schema": "rail-rescore/1", "rail_gates": "ideal",
           "runs": [run(a, "on", True), run(b, "on", False), run(c, "off", False),
                    run(d, "on", True, accepted=False)],
           "failsafe_labels": {
               "artefact": "docs/data/failsafe_labels_rail.json",
               "built": {"code_revisions": ["442397ff60a6aa", "897dac7a2d1fbb"]},
               "adopted_kpi_reading": "r2", "shield_off_runs_left_out": 1,
               "readings": {
                   "a": {**empty, "reading": "ideal labels", "adopted_kpi_reading": False,
                         "runs": 3, "scored": 2, "correct": 1,
                         "failsafe_trigger_correctness": 0.5, "wilson_low_95": 0.09,
                         "false": [dict(tag(a), label=[])],
                         "not_accepted": [tag(d)],
                         "at_scenario_length": {"scored": 2, "correct": 1}},
                   "r2": {**empty, "reading": "rail labels, prescribed flight",
                          "adopted_kpi_reading": True, "runs": 3, "scored": 2,
                          "correct": 2, "failsafe_trigger_correctness": 1.0,
                          "wilson_low_95": 0.34, "not_accepted": [tag(d)],
                          "label_flown_longer": [tag(a)],
                          "at_scenario_length": {"scored": 1, "correct": 1}}}}}
    path.write_text(json.dumps(doc), encoding="utf-8")
    return path


def _fixture():
    pol = load_policy(W.POLICY)
    root = W._tmp("kr_campaign_")
    signer, trust = W._signer(root / "keys")
    store = root / "hil_smoke"
    _run(store, "cx_001_a", pol, signer, _rows(20, tick=(5.0, 95.984), e2e=(60.0, 106.0)))
    _run(store, "cx_002_b", pol, signer, _rows(10, tick=(7.0,), e2e=(80.0,)))
    _run(store, "cx_003_ctrl", pol, signer, _rows(10, shield=False, escapes=4),
         shield="off")
    _run(store, "cx_004_d", pol, signer, _rows(10, tick=(500.0,), e2e=(900.0,)))
    _run(store, "other_001_x", pol, signer, _rows(5))       # another campaign's
    summ = _campaign(root, store)
    rs = _rescore(root / "rescore.json")
    return root, trust, summ, rs


def _report(summ, rs, trust, **kw):
    with W._trust_store(trust):
        return KR.build_report([], [], 10, False, "cmd", wp1_path=None, csp_json=None,
                               tick_profiles=None, campaign=summ, rescore=rs, **kw)


def _ep(rep, name):
    return next(e for e in rep["episodes"] if e["id"] == name)


# =========================================================================== #
# 1. the campaign: its runs, its acceptance as a gate
# =========================================================================== #

def test_the_campaign_gives_the_runs_and_demotes_what_it_did_not_accept():
    root, trust, summ, rs = _fixture()
    try:
        c = KR.load_campaign(summ)
        assert c["campaign"] == "cx" and c["profile"] == "smoke"
        assert c["runs_glob"] == [(root / "hil_smoke" / "cx_*").as_posix()], c["runs_glob"]
        assert set(c["accepted"]) == {"cx_001_a", "cx_002_b", "cx_003_ctrl"}
        assert set(c["not_accepted"]) == {"cx_004_d"} and "not realised" in \
            c["not_accepted"]["cx_004_d"]
        # The summary itself, or the folder holding it.
        assert KR.load_campaign(summ / "sweep.json")["runs_glob"] == c["runs_glob"]
        rep = _report(summ, None, trust)
        ids = sorted(e["id"] for e in rep["episodes"])
        assert ids == ["cx_001_a", "cx_002_b", "cx_003_ctrl", "cx_004_d"], ids
        for n in ("cx_001_a", "cx_002_b", "cx_003_ctrl"):
            e = _ep(rep, n)
            assert e["kpi_grade"] is True, (n, e["kpi_grade_reasons"])
        d = _ep(rep, "cx_004_d")
        assert d["kpi_grade"] is False and d["kpi_grade_manifest"] is True
        assert d["campaign_status"] == "not accepted"
        assert "did not accept" in d["kpi_grade_reasons"][-1] \
            and "not realised" in d["kpi_grade_reasons"][-1]
        assert rep["counts"]["campaign_not_accepted"] == 1
        assert rep["inputs"]["campaign"].endswith("hil_cx/sweep.json")
        # A run the campaign does not list is demoted too, by its own reason.
        e = {"id": "cx_009_z", "source": "x/cx_009_z", "kpi_grade": True,
             "kpi_grade_reasons": []}
        KR.campaign_gate(e, c)
        assert e["kpi_grade"] is False and "does not list it" in e["kpi_grade_reasons"][-1]
        md = KR.render_markdown(KR.json_safe(rep))
        assert "## Campaign" in md and "`cx_004_d`: campaign cx did not accept" in md, md
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_a_store_that_moved_is_found_beside_the_summary():
    root = W._tmp("kr_moved_")
    try:
        (root / "hil_smoke").mkdir()
        summ = _campaign(root, Path("Z:/gone/elsewhere/hil_smoke"))
        assert KR.load_campaign(summ)["runs_glob"] == [(root / "hil_smoke" / "cx_*")
                                                       .as_posix()]
        bad = root / "bad.json"
        bad.write_text("{}", encoding="utf-8")
        try:
            KR.load_campaign(bad)
        except SystemExit as e:
            assert "no `results`" in str(e)
        else:
            raise AssertionError("a file with no results read as a campaign")
    finally:
        shutil.rmtree(root, ignore_errors=True)


# =========================================================================== #
# 2. the fail-safe readings
# =========================================================================== #

def test_the_adopted_reading_is_applied_and_every_reading_scored_side_by_side():
    root, trust, summ, rs = _fixture()
    try:
        t = KR.load_rescore(rs)
        assert t["adopted"] == "r2" and t["keys"] == ["a", "r2"]
        a1 = t["runs"]["cx_001_a"]["readings"]
        assert a1["a"]["status"] == "false" and a1["a"]["expected"] is False
        assert a1["r2"]["status"] == "correct" and a1["r2"]["expected"] is True
        assert a1["r2"]["label_flown_longer"] is True
        assert t["runs"]["cx_004_d"]["readings"]["r2"]["status"] == "not_accepted"
        assert t["runs"]["cx_003_ctrl"]["readings"] == {}           # Shield off
        rep = _report(summ, rs, trust)
        e = _ep(rep, "cx_001_a")
        # The adopted label on the run; the one it flew with kept beside.
        assert (e["failsafe_label"], e["failsafe_triggered"], e["failsafe_matches_label"]) \
            == (True, True, 1.0), e
        assert e["failsafe_reading"] == "r2" and e["failsafe_reading_status"] == "correct"
        assert set(e["failsafe_readings"]) == {"a", "r2"}
        assert "failsafe_matches_label" in e["failsafe_at_flight"]
        ctrl = _ep(rep, "cx_003_ctrl")
        assert ctrl["failsafe_matches_label"] is None \
            and "Shield-off" in ctrl["failsafe_reading_status"]
        fr = rep["failsafe_readings"]
        assert fr["kpi_grade_shield_on_flights"] == 2, fr
        ra, r2 = fr["readings"]["a"], fr["readings"]["r2"]
        assert (ra["correct"], ra["scored"]) == (1, 2) and [x["run"] for x in ra["false"]] \
            == ["cx_001_a"]
        # (a) labels nothing to expect a fail-safe: not judged, never "meets".
        assert ra["discriminating"] is False and ra["meets_target"] is None
        assert (r2["correct"], r2["scored"]) == (2, 2) and r2["adopted_kpi_reading"]
        assert r2["failsafe_trigger_correctness"] == 1.0 and r2["meets_target"] is True
        assert r2["wilson_low_95"] < 0.99 and r2["meets_target_at_95"] is False
        assert r2["at_scenario_length"] == {"scored": 1, "correct": 1}
        assert [x["run"] for x in r2["label_flown_longer"]] == ["cx_001_a"]
        # cx_004 is not KPI-grade here: it is not in the contractual set at all.
        assert r2["not_accepted"] == [] and r2["agrees_with_table"] is True
        # The family column reads r2; the KPI summary's fail-safe is r2's.
        fam = rep["families"]["kpi_grade"]
        row = next(v for k, v in fam.items() if k.endswith("shield-on"))
        assert row["failsafe_labelled_episodes"]["scored"] == 2 \
            and row["failsafe_labelled_episodes"]["failsafe_trigger_correctness"] == 1.0
        fs = rep["kpi_summary"]["failsafe"]
        assert fs["basis"].startswith("adopted reading (r2)") and fs["correct"] == 2
        md = KR.render_markdown(KR.json_safe(rep))
        assert "## Fail-safe trigger correctness by label reading" in md
        assert "| (r2) **adopted** | 2/2 | 1.000 |" in md, md
        assert "false trigger: `cx_001_a`" in md
        assert "In this report the labels are the ADOPTED reading (r2)" in md
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_a_rescore_without_labels_or_of_another_kind_is_refused():
    d = W._tmp("kr_rs_bad_")
    try:
        for doc, word in (({"schema": "x", "runs": []}, "not a rail-rescore/1"),
                          ({"schema": "rail-rescore/1", "runs": []}, "no fail-safe label")):
            p = d / "r.json"
            p.write_text(json.dumps(doc), encoding="utf-8")
            try:
                KR.load_rescore(p)
            except SystemExit as e:
                assert word in str(e), (word, str(e))
            else:
                raise AssertionError(f"accepted: {doc}")
    finally:
        shutil.rmtree(d, ignore_errors=True)


# =========================================================================== #
# 3. the tick budget, per tick
# =========================================================================== #

def test_tick_timing_reads_the_end_to_end_fields_and_never_invents_a_zero():
    rows = [{"t": 0.1 * i, "shield_ms": 1.0, "tick_ms": float(i), "e2e_ms": None}
            for i in range(1, 101)]
    tt = KR.tick_timing(rows)
    assert (tt["tick_ms"]["p50"], tt["tick_ms"]["p95"], tt["tick_ms"]["p99"],
            tt["tick_ms"]["max"]) == (50.0, 95.0, 99.0, 100.0), tt["tick_ms"]
    assert tt["tick_ms"]["over_budget"] == 0 and tt["tick_ms"]["source"] == "flight_log.jsonl"
    assert "e2e_ms" not in tt and "cam_age_ms" not in tt, tt.keys()
    assert tt["shield_ms"]["p95"] == 1.0
    # A run whose rows carry none: its metrics.json summary, said so, no p95.
    m = {"camera_to_setpoint": {"cam_to_setpoint_ms": {"n": 4, "p50": 300.0, "p99": 410.0,
                                                       "max": 420.0, "over_budget": 4}},
         "tick_latency": {"e2e_ms": {"n": 0, "p50": None, "p99": None, "max": None}}}
    tt = KR.tick_timing([{"t": 0.0}, {"t": 0.1}], m)
    s = tt["cam_to_setpoint_ms"]
    assert s["source"].startswith("metrics.json camera_to_setpoint") and s["p95"] is None
    assert "e2e_ms" not in tt, "an empty summary is not a measurement"
    assert KR.tick_timing([{"x": 1}], {}) is None


def test_the_latency_section_pools_per_rail_and_says_what_was_not_measured():
    eps = [{"id": "a", "source": "a", "kind": "flight", "rail": "hil", "kpi_grade": True,
            "tick_timing": {}},
           {"id": "b", "source": "b", "kind": "flight", "rail": "hil", "kpi_grade": True,
            "tick_timing": {}},
           {"id": "c", "source": "c", "kind": "flight", "rail": "hil", "kpi_grade": False,
            "tick_timing": {}},
           {"id": "p", "source": "p", "kind": "flight", "rail": "pas", "kpi_grade": True,
            "tick_timing": {"cam_to_setpoint_ms": {"n": 3, "p50": 1.0, "p99": 2.0, "max": 3.0,
                                                   "over_budget": 0,
                                                   "source": "metrics.json"}}}]
    vals = {"a": {"tick_ms": [1.0, 2.0], "e2e_ms": [50.0]},
            "b": {"tick_ms": [3.0, 150.0], "e2e_ms": [110.0]},
            "c": {"tick_ms": [999.0]},
            "p": {"tick_ms": [4.0], "cam_age_ms": [120.0, 80.0]}}
    lat = KR.latency_section(eps, vals)
    hil = lat["groups"]["kpi_grade"]["hil"]
    assert hil["episodes"] == 2 and hil["tick_ms"]["n"] == 4 and hil["tick_ms"]["max"] == 150.0
    assert hil["tick_ms"]["over_budget"] == 1 and hil["tick_ms"]["episodes"] == 2
    assert hil["e2e_ms"]["p99"] == 110.0
    assert hil["cam_to_setpoint_ms"]["status"] == "not measured" \
        and "perception rail" in hil["cam_to_setpoint_ms"]["why"]
    # The run that is not KPI-grade is never pooled with the graded ones.
    assert lat["groups"]["not_kpi_grade"]["hil"]["tick_ms"]["max"] == 999.0
    pas = lat["groups"]["kpi_grade"]["pas"]
    assert pas["cam_age_ms"]["n"] == 2 and pas["cam_age_ms"]["over_budget"] == 1
    assert pas["cam_to_setpoint_ms"]["status"] == "summary only"
    assert pas["cam_to_setpoint_ms"]["summary_only"][0]["id"] == "p"
    md = "\n".join(KR._render_latency(lat))
    assert "| hil | `tick_ms` | 2 of 2 | 4 |" in md, md
    assert "| hil | `cam_age_ms` | - | - | not measured:" in md, md


# =========================================================================== #
# 4. the KPI summary, end to end through main()
# =========================================================================== #

def test_the_report_leads_with_the_kpi_summary_from_the_whole_flight():
    root, trust, summ, rs = _fixture()
    try:
        stem = root / "out" / "kpi"
        with W._trust_store(trust), contextlib.redirect_stdout(io.StringIO()) as buf:
            assert KR.main(["--campaign", str(summ), "--rescore-rail", str(rs),
                            "--wp1", "", "--tick-profiles", "", "--out", str(stem)]) == 0
        out = buf.getvalue()
        assert "1 not accepted by the campaign" in out, out
        assert "P0 escape rate: 0.0 Shield on (0 escapes, 2 flights)" in out, out
        rep = json.loads(stem.with_suffix(".json").read_text(encoding="utf-8"))
        assert rep["inputs"]["sweep"] == [], "the headless sweep is not read for a campaign"
        s = rep["kpi_summary"]
        assert (s["kpi_grade_episodes"], s["shield_on"], s["shield_off"]) == (3, 2, 1)
        on, off = s["p0"]["shield_on"], s["p0"]["shield_off_control"]
        assert on["p0_escapes"] == 0 and on["p0_escape_rate"] == 0.0
        assert off["p0_escapes"] == 4 and off["p0_escape_rate"] > 0
        hil = s["tick_budget"]["hil"]
        # cx_004 (not accepted) flew 500 / 900 ms ticks: none of them here.
        assert hil["tick_ms"]["max"] == 95.984 and hil["tick_ms"]["over_budget"] == 0
        # every KPI-grade run, the control too: the node ticks whatever the arm
        assert hil["tick_ms"]["n"] == 40 and hil["tick_ms"]["episodes"] == 3, hil["tick_ms"]
        assert hil["e2e_ms"]["max"] == 106.0 and hil["e2e_ms"]["n"] == 20, hil
        assert hil["cam_age_ms"]["status"] == "not measured"
        assert s["tracked"]["episodes"] == 2 and "by_template" in s
        assert not any("_latency_values" in e for e in rep["episodes"])
        md = stem.with_suffix(".md").read_text(encoding="utf-8")
        head = md.split("## Read this first")[0]
        assert head.index("## KPI summary") < head.index("## Campaign") \
            < head.index("## Fail-safe trigger correctness by label reading"), head[:400]
        assert "| P0 violation escape rate (Stress Testing p6) | 0 | 0.0000 (0 escapes" in md
        assert "Shield-off control" in md and "meets: 0 escapes" in md
        assert "Fail-safe trigger correctness, reading (r2) adopted" in md
        assert "max 95.98 ms; 0 of 40 ticks over 100 ms" in md, md
        assert "Camera to first setpoint `cam_to_setpoint_ms` (hil) | <= 100 ms end to end " \
               "| not measured:" in md
        assert "## End-to-end tick budget, per tick" in md
    finally:
        shutil.rmtree(root, ignore_errors=True)


# =========================================================================== #
# 5. the delivered campaign
# =========================================================================== #

def test_the_delivered_campaign_sm1111_reads_as_flown():
    out = Path(os.environ.get("KPI_CAMPAIGN_DEMO_OUT") or ROOT / "demo" / "out")
    summ = out / "hil_sm1111"
    if not (summ / "sweep.json").is_file() or not any((out / "hil_smoke").glob("sm1111_*")):
        print(f"      SKIP: {summ} absent")
        return SKIP
    if str(ROOT / "experiments") not in sys.path:
        sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S                                      # noqa: E402
    d = W._tmp("kr_sm1111_")
    try:
        # The light, read-only re-score (no flight, no simulator): the table
        # the report takes, written to a temp folder.
        rs = d / "rescore.json"
        with contextlib.redirect_stdout(io.StringIO()):
            rc = S.main(["--profile", "smoke", "--rescore-rail",
                         str(out / "hil_smoke" / "sm1111_*"), "--out", str(rs)])
        assert rc == 0 and rs.is_file(), rc
        rep = KR.build_report([], [], 10, False, "cmd", wp1_path=None, csp_json=None,
                              tick_profiles=None, campaign=summ, rescore=rs)
        s = rep["kpi_summary"]
        assert s["p0"]["shield_on"]["p0_escape_rate"] == 0.0
        assert s["p0"]["shield_off_control"]["p0_escape_rate"] == 0.65
        r2 = rep["failsafe_readings"]["readings"]["r2"]
        assert (r2["correct"], r2["scored"]) == (44, 45), (r2["correct"], r2["scored"])
        assert r2["agrees_with_table"] is True
        hil = s["tick_budget"]["hil"]
        assert hil["tick_ms"]["max"] == 95.984 and hil["tick_ms"]["over_budget"] == 0
        assert 105.0 < hil["e2e_ms"]["p99"] < 108.0, hil["e2e_ms"]
        assert rep["counts"]["campaign_not_accepted"] == 2
    finally:
        shutil.rmtree(d, ignore_errors=True)


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
        except BaseException as e:                   # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
