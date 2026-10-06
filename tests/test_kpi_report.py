"""The KPI rollup (tools/kpi_report.py) and the rescore fix (tools/rescore_kpis.py).

Run either way:
    pytest tests/test_kpi_report.py -v
    python tests/test_kpi_report.py

WHY THIS FILE EXISTS

Two things were missing on 2026-10-06 and both fail silently when written the
easy way.

  * The grant's KPI report (Stress Testing: per-family table + top-10
    failures, Markdown + JSON) did not exist. The traps are the ones every
    rollup has: pooling KPI-grade and non-grade runs into one number,
    printing an empty denominator as 0, and averaging ratios.
  * `ros2_ped_off/kpi.json` said `time_to_safe_episodes: 0` while its own
    metrics.json says the drone spent 2.3 s inside the 10 m pedestrian ring.
    The SITL rails declare the subject once in metrics.json; the rescore read
    it only from per-row fields those rails never write, so the stand-off rule
    was inert and the one run that breached it scored clean.

The tests marked REGRESSION fail on the code before the fix. Those dated
2026-10-06 review fail on the first version of the report itself: it
recomputed P0 figures against no policy, skipped every stress-harness bundle,
pooled parameter cells, had no parameters column, and its rescore corrected a
time to safe whenever any tick had used the declared subject.
"""
import io
import json
import sys
import tempfile
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from guardrail import kpi as K, load_policy                        # noqa: E402
import rescore_kpis as R                                           # noqa: E402

PED = ROOT / "policies" / "sitl_pedestrian.yaml"
A0 = {"vx": 0.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}


def _rows_past_a_person(n=61, step=0.5):
    """(0,0) -> (30,30) at 15 m, 0.1 s per tick. A person declared at (15,15)
    is passed within 10 m on |s - 15| < 7.07, i.e. about 2.8 s."""
    rows = []
    for i in range(n):
        s = i * step
        rows.append({"t": round(0.1 * (i + 1), 3), "tick": i + 1,
                     "x": s, "y": s, "up": 15.0,
                     "raw": dict(A0, vx=4.0, vy=4.0), "emitted": dict(A0, vx=4.0, vy=4.0),
                     "violations": [], "emitted_violations": [], "repairs": [],
                     "braked": False})
    return rows


def _make_run(root: Path, name: str, rows, *, policy_hash, metrics=None,
              stored=None, prompt=None) -> Path:
    run = root / name
    run.mkdir(parents=True)
    (run / "flight_log.jsonl").write_text(
        "".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    (run / "metrics.json").write_text(json.dumps(metrics or {}), encoding="utf-8")
    (run / "manifest.json").write_text(json.dumps({
        "code_revision": "abc", "vla_model_hash": "stub", "policy_hash": policy_hash,
        "random_seed": 0, "sim_speedup": 1.0, "topology": "canonical-hil"}),
        encoding="utf-8")
    if stored is not None:
        (run / "kpi.json").write_text(json.dumps(stored), encoding="utf-8")
    if prompt is not None:
        import yaml
        (run / "prompt.yaml").write_text(yaml.safe_dump(prompt), encoding="utf-8")
    return run


def _stale_store(rows):
    """What the 2026-09-01 rescore left behind: reconstructed, zero episodes."""
    st = K.compute([dict(r, unsafe=False) for r in rows], {}, {})
    st["time_to_safe_reconstructed"] = True
    return st


def _effective(r: dict) -> dict:
    """What the tool leaves in kpi.json: stored, plus added, plus corrected."""
    return dict(r["stored"], **r.get("added", {}), **r.get("corrected", {}))


DECLARED = {"subject": {"class": "pedestrian", "x": 15.0, "y": 15.0,
                        "position_source": "declared"},
            "standoff_s": 2.8, "nfz_s": 0.0, "alt_violation_s": 0.0, "reached": True}


# --------------------------------------------------------------------------- #
# the rescore fix
# --------------------------------------------------------------------------- #

def test_a_declared_subject_breach_is_not_scored_clean():
    """REGRESSION. Rows carry no subject; metrics.json declares one. The old
    rescore served nothing, the 10 m ring never fired, and 0 episodes stood."""
    pol = load_policy(PED)
    rows = _rows_past_a_person()
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "ped_off", rows, policy_hash=pol.policy_hash,
                        metrics=DECLARED, stored=_stale_store(rows))
        r = R.rescore(run, {pol.policy_hash: pol})
        eff = _effective(r)
    assert eff["time_to_safe_episodes"] == 1, eff
    assert abs(eff["mean_time_to_safe_s"] - 2.8) <= 0.21, eff


def test_ros2_ped_off_reports_the_standoff_breach_its_metrics_record():
    """REGRESSION, on the delivered artefact. Its metrics.json says 2.3 s inside
    the ring; its kpi.json said 0 time-to-safe episodes. The policy is passed
    keyed by the manifest hash so the test isolates the subject bug from the
    policy-hash drift, which is a separate problem (X-09).

    It asserts on the FRESH reconstruction, not on stored + corrected: the
    stored kpi.json was rewritten on 2026-10-06, so a check of the effective
    table passed on the pre-fix tool too (2026-10-06 review) - a check that
    could not fail."""
    run = ROOT / "demo" / "out" / "ros2_ped_off"
    if not (run / "flight_log.jsonl").is_file():
        print("      SKIP: demo/out/ros2_ped_off absent")
        return
    man = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
    met = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
    r = R.rescore(run, {man["policy_hash"]: load_policy(PED)})
    fresh = r["fresh"]
    assert fresh["time_to_safe_episodes"] == 1, fresh
    assert abs(fresh["mean_time_to_safe_s"] - met["standoff_s"]) <= 0.2, (
        fresh["mean_time_to_safe_s"], met["standoff_s"])
    assert r["crosscheck"]["fields"]["standoff_s"]["agree"] is True, r["crosscheck"]
    assert r["rescore_meta"]["subject_source"].get("metrics_declared"), \
        "the subject came from the rail's declaration"


def test_the_row_subject_wins_and_the_declared_one_is_only_a_fallback():
    dec = R.declared_subject(DECLARED)
    one = {"x": 0.0, "y": 0.0, "truth": {"class": "car", "pts": [[5.0, 6.0]]}}
    assert R.subject_for_row(one, dec) == (5.0, 6.0, "car", "row_truth")
    tgt = {"x": 0.0, "y": 0.0, "tgt_x": 1.0, "tgt_y": 2.0}
    assert R.subject_for_row(tgt, dec)[3] == "row_tgt"
    assert R.subject_for_row({"x": 0.0, "y": 0.0}, dec) == (15.0, 15.0, "pedestrian",
                                                           "metrics_declared")
    assert R.subject_for_row({"x": 0.0, "y": 0.0}, None) is None
    many = {"x": 0.0, "y": 0.0, "truth": {"class": "p", "pts": [[50.0, 0.0], [3.0, 0.0]]}}
    assert R.subject_for_row(many, dec)[:2] == (3.0, 0.0), "nearest, not the first"


def test_a_correction_keeps_the_values_it_replaced():
    pol = load_policy(PED)
    rows = _rows_past_a_person()
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "ped_off", rows, policy_hash=pol.policy_hash,
                        metrics=DECLARED, stored=_stale_store(rows))
        r = R.rescore(run, {pol.policy_hash: pol})
    sup = r["corrected"]["time_to_safe_superseded"]
    assert sup["values"]["time_to_safe_episodes"] == 0, sup
    assert "metrics.json" in sup["reason"], sup


def test_a_time_to_safe_the_flight_itself_wrote_is_never_overwritten():
    """Only this tool's OWN earlier reconstruction may be corrected. A table
    without `time_to_safe_reconstructed: true` came from the flight."""
    pol = load_policy(PED)
    rows = _rows_past_a_person()
    stored = _stale_store(rows)
    del stored["time_to_safe_reconstructed"]
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "flown", rows, policy_hash=pol.policy_hash,
                        metrics=DECLARED, stored=stored)
        r = R.rescore(run, {pol.policy_hash: pol})
    assert r["corrected"] == {}, r["corrected"]
    assert _effective(r)["time_to_safe_episodes"] == 0


def test_a_difference_the_subject_fix_does_not_explain_is_withheld():
    """Rows that carry their own subject (`tgt_x`) were never affected by the
    declared-subject defect. If their reconstruction still moved, Shield or IR
    code changed what 'unsafe' means since the stored rescore - found on
    retarget_demo / retarget_demo2 (0 -> 1 episode with the PRE-fix rescore
    too). That is for a human to rule on, not for this tool to adopt."""
    pol = load_policy(PED)
    rows = [dict(r, tgt_x=15.0, tgt_y=15.0) for r in _rows_past_a_person()]
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "tgt", rows, policy_hash=pol.policy_hash,
                        metrics={"nfz_s": 0.0}, stored=_stale_store(rows))
        r = R.rescore(run, {pol.policy_hash: pol})
    assert r["fresh"]["time_to_safe_episodes"] == 1, r["fresh"]
    assert r["corrected"] == {}, r["corrected"]
    assert any("needs a human" in w for w in r["withheld"]), r["withheld"]


def test_a_correction_the_subject_does_not_fully_explain_is_withheld():
    """REGRESSION (2026-10-06 review). The first gate only asked whether some
    tick was served the declared subject. Rerun WITHOUT the subject, the
    pre-fix reconstruction gives 0 episodes; a stored 5 is therefore not that
    tool's answer, and something else moved it. The pre-change rescore
    corrects it anyway."""
    pol = load_policy(PED)
    rows = _rows_past_a_person()
    stored = _stale_store(rows)
    stored["time_to_safe_episodes"] = 5
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "odd", rows, policy_hash=pol.policy_hash,
                        metrics=DECLARED, stored=stored)
        r = R.rescore(run, {pol.policy_hash: pol})
    assert r["corrected"] == {}, r["corrected"]
    assert any("needs a human" in w for w in r["withheld"]), r["withheld"]


def test_an_unresolved_policy_writes_no_priority_dependent_field():
    """Mutation M22 (the NEEDS_PRIORITIES gate removed) survived: without the
    policy every rule is P0, so these fields would be written wrong."""
    rows = [dict(r, violations=[{"rule_id": "kin-speed", "category": "kinematic"}],
                 repairs=[{"operator": "SpeedClamp", "detail": ""}],
                 emitted=dict(A0, vx=1.0))
            for r in _rows_past_a_person(10)]
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "lost", rows, policy_hash="sha256:" + "f" * 64,
                        metrics={"shield": "on"},
                        stored={"ticks": 10, "p0_escapes": 0})
        r = R.rescore(run, {}, {})
    assert r["verified"] is False
    assert not set(r["added"]) & set(R.NEEDS_PRIORITIES), set(r["added"]) & set(R.NEEDS_PRIORITIES)
    assert "repair_success_rate" in r["added"], "the priority-free fields still land"


def test_retired_fields_leave_only_this_tools_own_tables():
    """The withdrawn false-trigger fields are removed from a table this tool
    wrote on 2026-10-06, kept with their values under rescore.retired - and
    left alone in a table that never had this tool's block."""
    pol = load_policy(PED)
    rows = _rows_past_a_person()
    base = {k: v for k, v in K.compute(rows, {}, {}).items()
            if k in ("ticks", "p0_escapes", "p0_violation_ticks",
                     "p0_violation_escape_rate", "p0_ticks_not_measurable")}
    mine = dict(base, failsafe_false_triggers=0, failsafe_not_expected_ticks=61,
                rescore={"date": "2026-10-06"})
    theirs = dict(base, failsafe_false_triggers=0)
    with tempfile.TemporaryDirectory() as td:
        a = _make_run(Path(td), "mine", rows, policy_hash=pol.policy_hash,
                      metrics=DECLARED, stored=mine)
        b = _make_run(Path(td), "theirs", rows, policy_hash=pol.policy_hash,
                      metrics=DECLARED, stored=theirs)
        with redirect_stdout(io.StringIO()):
            R.main(["--roots", td])
        got_a = json.loads((a / "kpi.json").read_text(encoding="utf-8"))
        got_b = json.loads((b / "kpi.json").read_text(encoding="utf-8"))
    assert "failsafe_false_triggers" not in got_a, got_a
    assert got_a["rescore"]["retired"]["fields"]["failsafe_not_expected_ticks"] == 61
    assert "failsafe_false_triggers" in got_b, "not this tool's to remove"


def test_a_rewrite_keeps_the_files_own_line_endings():
    """`Path.write_text` on Windows turns an LF file into CRLF, a whole-file
    diff for a one-field addition (repo-line-endings memory)."""
    pol = load_policy(PED)
    rows = _rows_past_a_person()
    with tempfile.TemporaryDirectory() as td:
        lf = _make_run(Path(td), "lf", rows, policy_hash=pol.policy_hash,
                       metrics=DECLARED, stored=_stale_store(rows))
        crlf = _make_run(Path(td), "crlf", rows, policy_hash=pol.policy_hash,
                         metrics=DECLARED, stored=_stale_store(rows))
        for run, nl in ((lf, b"\n"), (crlf, b"\r\n")):
            body = json.dumps(_stale_store(rows), indent=2).encode("utf-8")
            (run / "kpi.json").write_bytes(body.replace(b"\n", nl) + nl)
        with redirect_stdout(io.StringIO()):
            R.main(["--roots", td])
        b_lf = (lf / "kpi.json").read_bytes()
        b_crlf = (crlf / "kpi.json").read_bytes()
    assert b"\r\n" not in b_lf, "an LF file must stay LF"
    assert b_crlf.count(b"\n") == b_crlf.count(b"\r\n"), "a CRLF file must stay CRLF"
    assert json.loads(b_lf)["time_to_safe_episodes"] == 1, "and it was rewritten"


def test_the_p0_figures_are_still_never_rewritten():
    pol = load_policy(PED)
    rows = _rows_past_a_person()
    stored = _stale_store(rows)
    stored["p0_escapes"] = 7                       # disagrees with the log
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "drift", rows, policy_hash=pol.policy_hash,
                        metrics=DECLARED, stored=stored)
        r = R.rescore(run, {pol.policy_hash: pol})
        assert r["drift"], "a P0 disagreement must be reported"
        with redirect_stdout(io.StringIO()):
            code = R.main(["--roots", td])
        after = json.loads((run / "kpi.json").read_text(encoding="utf-8"))
    assert code == 1
    assert after == stored, "a drifted run must not be written at all"


def test_dry_run_writes_nothing_and_a_real_run_writes_strict_json():
    pol = load_policy(PED)
    rows = _rows_past_a_person()
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "ped_off", rows, policy_hash=pol.policy_hash,
                        metrics=DECLARED, stored=_stale_store(rows))
        before = (run / "kpi.json").read_text(encoding="utf-8")
        with redirect_stdout(io.StringIO()):
            R.main(["--dry-run", "--roots", td])
        assert (run / "kpi.json").read_text(encoding="utf-8") == before
        out = io.StringIO()
        with redirect_stdout(out):
            R.main(["--roots", td])
        after = json.loads((run / "kpi.json").read_text(encoding="utf-8"),
                           parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
    assert after["time_to_safe_episodes"] == 1, after
    assert after["time_to_safe_superseded"]["values"]["time_to_safe_episodes"] == 0
    assert after["repair_success_status"] == "no_repairs_attempted", after
    assert "CORRECTED" in out.getvalue()


# --------------------------------------------------------------------------- #
# policy resolution when the hash no longer matches
# --------------------------------------------------------------------------- #

def _resolve(td, name, prompt, manifest_hash):
    run = _make_run(Path(td), name, _rows_past_a_person(5),
                    policy_hash=manifest_hash, prompt=prompt)
    loaded = R.load_policies()
    return R.resolve_policy(run, {"policy_hash": manifest_hash}, {},
                            R.policies_by_id(loaded))


def test_a_unique_policy_id_resolves_a_run_whose_hash_drifted():
    h = "sha256:0123456789abcdef"
    with tempfile.TemporaryDirectory() as td:
        pol, how, note = _resolve(td, "a", {"constraints": {
            "policy_id": "sitl-pedestrian-standoff", "policy_hash": h}}, h)
    assert how == "policy_id" and pol.policy_id == "sitl-pedestrian-standoff", note


def test_a_hot_applied_run_is_not_resolved_by_id():
    """The file on disk lacks the zone added mid-flight; reconstructing against
    it would under-report time unsafe. The dynamic runs must stay n/m."""
    with tempfile.TemporaryDirectory() as td:
        pol, how, note = _resolve(td, "dyn", {"constraints": {
            "policy_id": "fase3-sim-demo", "policy_hash": "sha256:aaaaaaaaaaaaaaaa"}},
            "sha256:bbbbbbbbbbbbbbbb")
    assert pol is None and how is None and "moved in flight" in note, note


def test_an_ambiguous_or_missing_policy_id_is_not_guessed():
    h = "sha256:0123456789abcdef"
    with tempfile.TemporaryDirectory() as td:
        pol, _, note = _resolve(td, "r", {"constraints": {
            "policy_id": "random-scenario", "policy_hash": h}}, h)
        assert pol is None and "policy files" in note, note
        pol, _, note = _resolve(td, "n", None, h)
        assert pol is None, note


def test_an_id_resolved_run_whose_dwell_disagrees_writes_no_time_to_safe():
    """The rail's own dwell is what corroborates an id match. 2.8 s
    reconstructed against a rail that measured 20 s means the file on disk
    is not the policy that flew."""
    h = "sha256:0123456789abcdef"
    rows = _rows_past_a_person()
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "bad", rows, policy_hash=h,
                        metrics=dict(DECLARED, standoff_s=20.0),
                        stored={k: v for k, v in K.compute(rows, {}, {}).items()
                                if "time_to_safe" not in k},
                        prompt={"constraints": {"policy_id": "sitl-pedestrian-standoff",
                                                "policy_hash": h}})
        loaded = R.load_policies()
        r = R.rescore(run, {}, R.policies_by_id(loaded))
    assert r["resolved_by"] == "policy_id"
    assert not any("time_to_safe" in f for f in r["added"]), r["added"]
    assert r["withheld"], "the disagreement must be reported"


def test_the_dwell_crosscheck_catches_zero_against_two_point_three():
    bad = R.dwell_crosscheck({"unsafe_s_by_category": {}}, {"standoff_s": 2.3})
    assert bad["agree"] is False, bad
    ok = R.dwell_crosscheck({"unsafe_s_by_category": {"geofence": 4.1}}, {"nfz_s": 3.7})
    assert ok["agree"] is True, "ros2_shield_off's known 0.4 s gap is tolerated"
    assert R.dwell_crosscheck({}, {})["agree"] is None, "nothing to compare is not 'agree'"


# --------------------------------------------------------------------------- #
# the report
# --------------------------------------------------------------------------- #

def _report_module():
    import kpi_report as KR
    KR.code_revision = lambda *a, **k: "test-rev"        # git status is slow
    return KR


def _three_runs(td):
    """A shielded run that always brakes, a passthrough control, one clean."""
    pol = load_policy(PED)
    base = _rows_past_a_person(20)
    brake = [dict(r, violations=[{"rule_id": "alt-band", "category": "altitude"}],
                  repairs=[{"operator": "Brake", "detail": ""}], braked=True,
                  emitted=dict(A0)) for r in base]
    off = [dict(r, violations=[{"rule_id": "alt-band", "category": "altitude"}],
                emitted_violations=[{"rule_id": "alt-band", "category": "altitude"}])
           for r in base]
    _make_run(Path(td), "graded_brake", brake, policy_hash=pol.policy_hash,
              metrics={"shield": "on", "reached": True})
    _make_run(Path(td), "control_off", off, policy_hash=pol.policy_hash,
              metrics={"shield": "off", "reached": True})
    _make_run(Path(td), "clean_on", base, policy_hash=pol.policy_hash,
              metrics={"shield": "on", "reached": False})


def test_kpi_grade_and_other_runs_are_tabulated_apart_never_pooled():
    KR = _report_module()
    real = KR.is_kpi_grade
    KR.is_kpi_grade = lambda m, me: ((True, []) if me.get("shield") == "on"
                                     and me.get("reached") else (False, ["test"]))
    try:
        with tempfile.TemporaryDirectory() as td:
            _three_runs(td)
            rep = KR.build_report([str(Path(td) / "*")], [], 10, False, "cmd")
    finally:
        KR.is_kpi_grade = real
    g, o = rep["families"]["kpi_grade"], rep["families"]["other"]
    assert sum(f["episodes"] for f in g.values()) == 1, g
    assert sum(f["episodes"] for f in o.values()) == 2, o
    # graded_brake and clean_on share a family NAME; they must still land in
    # different rows, each pooled only with its own grade.
    g_members = {m for f in g.values() for m in f["members"]}
    o_members = {m for f in o.values() for m in f["members"]}
    assert g_members == {"graded_brake"}, g_members
    assert o_members == {"control_off", "clean_on"}, o_members
    assert rep["counts"]["kpi_grade"] == 1


def test_zero_none_attempted_and_not_measurable_render_differently():
    """The always-brake run scores 0 % repair success (the null), the
    passthrough control 'none attempted'. Printing both as 0 would make the
    Shield-off arm look like a Shield that tried and failed."""
    KR = _report_module()
    with tempfile.TemporaryDirectory() as td:
        _three_runs(td)
        rep = KR.build_report([str(Path(td) / "*")], [], 10, True, "cmd")
    md = KR.render_markdown(rep)
    fams = rep["families"]["other"]
    brake = next(f for n, f in fams.items() if "graded_brake" in f["members"])
    off = next(f for n, f in fams.items() if "control_off" in f["members"])
    assert brake["repair_success_rate"] == 0.0, brake
    assert off["repair_success_rate"] is None
    assert off["repair_success_status"] == "no_repairs_attempted"
    assert "none attempted" in md and "0.0 %" in md
    # the clean run missed its goal: a failure case, not a success
    clean = next(e for e in rep["episodes"] if e["id"] == "clean_on")
    assert clean["mission_success"] is False and clean["mission_goal_status"] == "missed"
    assert "clean_on" in [t["scenario_id"] for t in rep["top_failures"]]


def test_the_cli_writes_strict_json_and_markdown_and_prints_how_to_reproduce():
    KR = _report_module()
    with tempfile.TemporaryDirectory() as td:
        _three_runs(td)
        stem = Path(td) / "out" / "rollup"
        buf = io.StringIO()
        with redirect_stdout(buf):
            KR.main(["--runs", str(Path(td) / "*"), "--no-sweep", "--out", str(stem)])
        js = json.loads(stem.with_suffix(".json").read_text(encoding="utf-8"),
                        parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
        md = stem.with_suffix(".md").read_text(encoding="utf-8")
    assert js["counts"]["episodes"] == 3, js["counts"]
    assert "Top-10 failure cases" in md and "Per-family stats" in md
    assert "reproduce: python tools/kpi_report.py --runs" in buf.getvalue()
    assert js["command"].startswith("python tools/kpi_report.py --runs")


def test_the_sweep_loader_applies_the_goal_and_drops_counterfactual_repairs():
    KR = _report_module()
    # Shaped like the delivered docs/data/scenario_sweep.json: written before
    # compute() had the repair-success fields, so they are absent.
    old = {k: v for k, v in K.compute([], {}, {}).items()
           if not k.startswith(("repair_success", "repair_attempt", "repair_outcomes",
                                "failsafe_", "repairs_to", "mission_goal"))}
    sweep = {"results": [
        {"id": "x-control", "status": "pass",
         "kpi": dict(old, repair_count=52, reached_goal=None)},
        {"id": "x-goal", "status": "pass",
         "kpi": dict(old, repair_count=3, reached_goal=False)},
        {"id": "x-skip", "status": "skipped"}]}
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "sweep.json"
        p.write_text(json.dumps(sweep), encoding="utf-8")
        eps = KR.load_sweep(p)
    ctrl, goal, skip = eps
    assert ctrl["arm"] == "off" and ctrl["repair_count"] == 0, ctrl
    assert ctrl["repair_count_counterfactual"] == 52
    assert K.mission_success_with_goal(goal) is False
    assert goal["stored_vs_recomputed"]["mission_success"] == {"stored": True,
                                                               "recomputed": False}
    assert "skipped" in skip
    assert goal["kpi_grade"] is False and goal["kpi_grade_reasons"]
    # the sweep stores no per-tick rows, so repair success cannot be claimed
    assert K.rollup([goal])["repair_success_status"] == "not_measurable"


def test_non_finite_values_survive_as_visible_strings():
    KR = _report_module()
    found = []
    out = KR.json_safe({"a": [1.0, float("nan")], "b": float("inf")}, "", found)
    assert out == {"a": [1.0, "NaN"], "b": "Infinity"}, out
    assert found == ["/a/1", "/b"], found


def test_a_hot_applied_delivered_run_stays_not_measurable():
    KR = _report_module()
    run = ROOT / "demo" / "out" / "ros2_shield_on_dynamic"
    if not (run / "flight_log.jsonl").is_file():
        print("      SKIP: demo/out/ros2_shield_on_dynamic absent")
        return
    loaded = R.load_policies()
    e = KR.load_run(run, R.policies_by_hash(loaded), R.policies_by_id(loaded))
    assert e["time_to_safe_not_measurable"] is True, e["time_to_safe_basis"]
    assert e["time_to_safe_basis"].startswith("n/m"), e["time_to_safe_basis"]


def test_an_unresolvable_policy_does_not_count_every_rule_as_p0():
    """REGRESSION (2026-10-06 review). With no policy, compute() treats every
    rule as P0, so the report recomputed ros2_shield_on_dynamic at 309 P0
    ticks against the 217 it scored in flight, and printed a passthrough null
    of 0.8234 for its family instead of 0.652. The pre-change report gives 10
    here; the flight said 0."""
    KR = _report_module()
    rows = [dict(r, violations=[{"rule_id": "kin-speed", "category": "kinematic"}],
                 repairs=[{"operator": "SpeedClamp", "detail": ""}],
                 emitted=dict(A0, vx=1.0))
            for r in _rows_past_a_person(10)]
    flown = {"ticks": 10, "p0_escapes": 0, "p0_violation_ticks": 0,
             "p0_violation_escape_rate": 0.0, "p0_ticks_not_measurable": 0,
             "failsafe_trigger_correctness": None, "outcome": "success",
             "mission_success": True}
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "dyn", rows, policy_hash="sha256:" + "e" * 64,
                        metrics={"shield": "on"}, stored=flown)
        bare = _make_run(Path(td), "bare", rows, policy_hash="sha256:" + "e" * 64,
                         metrics={"shield": "on"})
        e = KR.load_run(run, {}, {})
        b = KR.load_run(bare, {}, {})
        rep = KR.build_report([str(Path(td) / "dyn")], [], 10, False, "cmd")
    assert e["p0_violation_ticks"] == 0, e["p0_violation_ticks"]
    assert e["priority_fields_from"].startswith("stored"), e["priority_fields_from"]
    assert b["p0_violation_ticks"] is None, "no policy, no stored table: n/m, not 10"
    fam = next(iter(rep["families"]["other"].values()))
    assert fam["null_passthrough_p0_escape_rate"] == 0.0, fam
    assert e["repair_success_rate"] == 1.0, "the priority-free figure still stands"


def _bundle(root: Path, name: str, *, sid, template, params, seed, shield, kpi,
            code="abc123", policy_hash="sha256:" + "a" * 64) -> Path:
    d = root / name
    d.mkdir(parents=True)
    (d / "manifest.json").write_text(json.dumps({
        "code_revision": code, "vla_model_hash": "pilot", "policy_hash": policy_hash,
        "random_seed": seed, "sim_speedup": 300.0, "topology": "headless-kinematic"}),
        encoding="utf-8")
    (d / "kpi.json").write_text(json.dumps({
        "id": KR_cell(sid, params), "scenario_id": sid, "template": template,
        "params": params, "seed": seed, "status": "pass", "kpi_grade": False,
        "kpi": kpi}), encoding="utf-8")
    (d / "harness_events.jsonl").write_text(json.dumps(
        {"t": 0.0, "type": "episode_start", "scenario_id": sid, "seed": seed,
         "shield": shield}) + "\n", encoding="utf-8")
    return d


def KR_cell(sid, params):
    return sid + ("@" + ",".join(f"{k}={params[k]}" for k in sorted(params))
                  if params else "")


def _old_control_table():
    """A control arm's table as the pre-change kpi.py stored it: 52
    counterfactual repairs scored as a measured 0.0."""
    t = K.compute([], {}, {})
    t.update({"ticks": 300, "p0_escapes": 52, "p0_violation_ticks": 52,
              "repair_count": 52, "repair_success_rate": 0.0,
              "repair_success_status": "measured", "repair_attempt_ticks": 52,
              "repair_success_ticks": 0, "repaired_ticks": 52,
              "mean_repair_magnitude_mps": 0.0, "reached_goal": True,
              "mission_success": False})
    for f in ("repair_unmeasured_ticks", "repair_success_basis",
              "repair_theta_unchecked_ticks"):
        t.pop(f, None)
    return t


def test_stress_bundles_are_read_as_stored_tables_with_their_arm_and_cell():
    """REGRESSION (2026-10-06 review). The stress harness writes episode
    bundles with no flight log; the first report skipped all 700 of them. The
    pre-change tool reports 0 episodes here."""
    KR = _report_module()
    on = K.compute([_rep_row_conv(0.1 * i) for i in range(3)], {}, {})
    with tempfile.TemporaryDirectory() as td:
        _bundle(Path(td), "episode-T--x-control--seed1", sid="x-control",
                template="tmpl", params={}, seed=1, shield="off",
                kpi=_old_control_table())
        _bundle(Path(td), "episode-T--x@a=1--seed1", sid="x", template="tmpl",
                params={"a": 1}, seed=1, shield="on", kpi=on)
        _bundle(Path(td), "episode-T--x@a=2--seed1", sid="x", template="tmpl",
                params={"a": 2}, seed=1, shield="on", kpi=on)
        rep = KR.build_report([str(Path(td) / "episode-*")], [], 10, True, "cmd")
    eps = {e["id"]: e for e in rep["episodes"]}
    assert len(eps) == 3, (sorted(eps), rep["skipped"])
    ctrl = eps["x-control--seed1"]
    assert ctrl["kind"] == "stored-table" and ctrl["arm"] == "off", ctrl
    assert ctrl["repair_success_status"] == "shield_off", ctrl["repair_success_status"]
    assert ctrl["repair_success_rate"] is None and ctrl["repair_count"] == 0
    assert ctrl["repair_count_counterfactual"] == 52
    fams = rep["families"]["other"]
    assert "stress-bundle/tmpl/x@a=1/shield-on" in fams, sorted(fams)
    assert "stress-bundle/tmpl/x@a=2/shield-on" in fams, "two cells, two rows"
    assert fams["stress-bundle/tmpl/x-control/shield-off"]["repair_success_status"] \
        == "shield_off"


def _rep_row_conv(t):
    return {"t": t, "x": 0.0, "y": 0.0, "up": 15.0,
            "raw": dict(A0, vx=3.0), "emitted": dict(A0, vx=1.0),
            "violations": [{"rule_id": "nfz", "category": "geofence"}],
            "emitted_violations": [],
            "repairs": [{"operator": "GeofenceProject", "detail": ""}],
            "braked": False}


def test_a_bundle_flown_twice_counts_once():
    """The smoke profile re-flies the nightly profile's first seed: the same
    deterministic episode, which must not double its weight in its family."""
    KR = _report_module()
    on = K.compute([_rep_row_conv(0.0)], {}, {})
    with tempfile.TemporaryDirectory() as td:
        for stamp in ("A", "B"):
            _bundle(Path(td) / stamp, "episode-x--seed1", sid="x", template="t",
                    params={}, seed=1, shield="on", kpi=on)
        rep = KR.build_report([str(Path(td) / "*" / "episode-*")], [], 10, False, "cmd")
    assert rep["counts"]["episodes"] == 1, rep["counts"]
    assert any("duplicate of" in s["skipped"] for s in rep["skipped"]), rep["skipped"]


def test_two_cells_of_one_template_are_two_family_rows():
    """REGRESSION (2026-10-06 review). "One row per (scenario template,
    parameter cell) tuple" (Stress Testing p6). The first report keyed the
    sweep by template alone and pooled three different subject_standoff
    scenarios into one row; the pre-change tool gives one family here."""
    KR = _report_module()
    old = {k: v for k, v in K.compute([], {}, {}).items()
           if not k.startswith(("repair_success", "repair_attempt", "repair_outcomes"))}
    sweep = {"results": [
        {"id": "standoff-a", "template": "subject_standoff", "status": "pass",
         "kpi": dict(old, reached_goal=True)},
        {"id": "standoff-b", "template": "subject_standoff", "status": "pass",
         "kpi": dict(old, reached_goal=True)}]}
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "sweep.json"
        p.write_text(json.dumps(sweep), encoding="utf-8")
        rep = KR.build_report([], [str(p)], 10, False, "cmd")
    assert len(rep["families"]["other"]) == 2, sorted(rep["families"]["other"])


def test_a_hot_applied_flight_does_not_share_a_row_with_the_static_one():
    """The static ros2_shield_on and the hot-applied ros2_shield_on_dynamic
    are different stressors; the first report pooled them. A run whose
    takeoff policy hash differs from its manifest's was hot-applied."""
    KR = _report_module()
    pol = load_policy(PED)
    rows = _rows_past_a_person(10)
    with tempfile.TemporaryDirectory() as td:
        _make_run(Path(td), "static", rows, policy_hash=pol.policy_hash,
                  metrics={"shield": "on", "reached": True},
                  prompt={"constraints": {"policy_id": pol.policy_id,
                                          "policy_hash": pol.policy_hash}})
        _make_run(Path(td), "dynamic", rows, policy_hash=pol.policy_hash,
                  metrics={"shield": "on", "reached": True},
                  prompt={"constraints": {"policy_id": pol.policy_id,
                                          "policy_hash": "sha256:" + "0" * 64}})
        rep = KR.build_report([str(Path(td) / "*")], [], 10, False, "cmd")
    fams = rep["families"]["other"]
    assert len(fams) == 2, sorted(fams)
    assert any("hot_applied=True" in f for f in fams), sorted(fams)


def test_the_top_k_markdown_carries_the_parameters_column():
    """The grant's top-K columns: "scenario_id, parameters, failure category,
    raw vs repaired action" (Stress Testing p6); the first Markdown had no
    parameters column."""
    KR = _report_module()
    with tempfile.TemporaryDirectory() as td:
        _three_runs(td)
        rep = KR.build_report([str(Path(td) / "*")], [], 10, False, "cmd")
    md = KR.render_markdown(rep)
    head = next(line for line in md.splitlines() if line.startswith("| # | Scenario"))
    assert "| Parameters |" in head, head
    row = next(line for line in md.splitlines() if "`clean_on`" in line)
    assert "seed=0" in row, row


def test_a_repair_that_changed_nothing_on_an_airsim_flight_is_a_failure():
    """demo/follow_vlm.py has no Shield-off switch, so the report tells
    compute() the AirSim arm is on, and a no-op repair stays a failed one."""
    KR = _report_module()
    pol = load_policy(PED)
    rows = [dict(r, violations=[{"rule_id": "alt-band", "category": "altitude"}],
                 emitted_violations=[{"rule_id": "alt-band", "category": "altitude"}],
                 repairs=[{"operator": "AltitudeFix", "detail": ""}])
            for r in _rows_past_a_person(5)]
    with tempfile.TemporaryDirectory() as td:
        run = _make_run(Path(td), "airsim", rows, policy_hash=pol.policy_hash)
        man = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
        man["topology"] = "projectairsim-single-host"
        (run / "manifest.json").write_text(json.dumps(man), encoding="utf-8")
        e = KR.load_run(run, {pol.policy_hash: pol}, {})
    assert e["repair_success_rate"] == 0.0, (e["repair_success_status"],
                                            e["repair_not_applied_ticks"])


def test_the_delivered_ros2_runs_roll_up_with_ped_off_corrected():
    KR = _report_module()
    if not (ROOT / "demo" / "out" / "ros2_ped_off" / "flight_log.jsonl").is_file():
        print("      SKIP: demo/out/ros2_* absent")
        return
    rep = KR.build_report(["demo/out/ros2_*"], [], 10, True, "cmd")
    eps = {e["id"]: e for e in rep["episodes"]}
    assert len(eps) == 5, sorted(eps)
    assert eps["ros2_ped_off"]["time_to_safe_episodes"] == 1, eps["ros2_ped_off"]
    for on in ("ros2_shield_on", "ros2_ped_on"):
        assert eps[on]["repair_success_status"] == "measured", eps[on]
        assert eps[on]["time_to_safe_episodes"] == 0
    assert eps["ros2_shield_off"]["repair_success_status"] == "no_repairs_attempted"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
