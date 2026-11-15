"""A lengthened hil campaign (2026-11-14; contract WP3-15, docs/RUBRIC-failsafe.md
section 3): `experiments/sweep_scenarios.py --backend hil --hil-max-s-add S`.

    python tests/test_hil_fly_longer.py

NOTHING HERE FLIES. The rig is tests/test_sweep.py's fake hil rig (`_FakeRig`:
its "Orin" flies the scenario it is handed on the headless harness and writes
the run folder the Shield node would). These tests show what the harness
launches, records, scores and refuses - nothing about ArduPilot, MAVROS or the
Orin.

WHY. In the KPI nightly ntFIN2, 19 KPI-grade runs ended in Loiter with the
operator timeout still running: censored, not measured, "fly it longer". A
rail mission already flies the scenario's cap plus the 30 s Loiter timeout
plus 5 s (rail_max_s); the option flies a campaign S seconds past that, and
says so everywhere a reader could mistake it for the profile's own length.

WHAT IS BEING CLAIMED

1. `--hil-max-s-add S` lengthens every mission: the node's --max-s and the
   mission's timeout are the rail's cap + S, the scenario record says
   `max_s_add`, `rail_max_s_profile`, `flight_length` "lengthened" and its
   family ends in "/flown-longer-<S>s", while `max_s` (the scenario's own cap,
   the performance window) is unchanged; the campaign's sweep.json records it
   in `_scope.hil` and its command. Without the option nothing of this is
   written and --max-s is the rail's cap, as before.
2. The scorer reads the cell's label on the longer flight as on any other:
   the label is the cell's and seed's, the trigger the FSM's over the whole
   flight; --rescore-rail names the run `run_flown_longer` and keeps the count
   without it (`at_profile_length`). A table never mixes flight lengths.
3. A run that still ends in Loiter with its timeout pending is still
   CENSORED: not scored, a label failure saying it was already flown longer,
   named censored by the re-score with its cause. A Loiter frozen by the
   autopilot's takeover (the GeoFence RTL that ended 18 of ntFIN2's 19
   censored runs) is named as such: flying longer does not decide it.
4. tools/kpi_report.py keeps a lengthened campaign apart: its own family
   rows, its length in the Campaign section and the inputs, and a campaign,
   supplement, re-score table or run glob of another length refused.
5. The option is refused where it cannot apply, and the dry run shows it.
6. (2026-11-15, docs/RUBRIC-failsafe.md sections 1 and 9) A takeover the
   autopilot's own "Fence Breached" stands beside - RTL or LAND, the FSM
   holding in Brake or Loiter - is no longer censored: it is a fail-safe
   triggered by the GeoFence backstop (`failsafe_by` "autopilot_geofence"),
   scored against the label, counted beside the figure, with the reading
   before that rule printed beside. The takeover of claim 3 has no fence
   breach on record and stays censored.

Shown failing first: against e1d3a54 (the tree before this change) every test
here fails - main() has no --hil-max-s-add, rail_max_s takes no `add`, and
neither the scenario record nor kpi_report knows a flight length.
"""
from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "experiments", ROOT / "tools", ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import kpi_report as KR                                            # noqa: E402


def _load_test_sweep():
    """tests/test_sweep.py as a module, for its fake hil rig."""
    spec = importlib.util.spec_from_file_location("test_sweep_for_fly_longer",
                                                  ROOT / "tests" / "test_sweep.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


TS = _load_test_sweep()
S = TS.S
SKIP = "SKIP"
# A labelled cell (expected_failsafe True) the fake rig flies to the headless
# verdict, with no scenario events: 500 ticks, a 50 s cap, the rail's 85 s.
CELL = "rail-mission-stub-vla"


class _HeldRig(TS._FakeRig):
    """The fake rig, with the flight ending as ntFIN2's censored runs did: the
    FSM still in Loiter, its 30 s operator timeout pending, no fail-safe;
    `took_over` adds the autopilot's takeover (GUIDED -> RTL) the Shield
    node records."""

    took_over = False

    def run_mission(self, args, tag, shield, timeout_s):
        rc, out = super().run_mission(args, tag, shield, timeout_s)
        p = self.remote / "demo" / "out" / tag / "metrics.json"
        m = json.loads(p.read_text(encoding="utf-8"))
        fsm = dict(m.get("fsm") or {})
        fsm.update(failsafe_triggered=False, final_state="Loiter", max_state="Loiter",
                   terminal=False, outcome_label=None,
                   ended_holding={"state": "Loiter", "dwell_s": 1.6, "timeout_s": 30.0,
                                  "timeout_pending": True})
        m["fsm"] = fsm
        if self.took_over:
            m["autopilot_took_over"] = {"t": 5.78, "mode": "RTL", "previous": "GUIDED"}
        p.write_text(json.dumps(m), encoding="utf-8")
        return rc, out


class _TakenRig(_HeldRig):
    took_over = True


class _FencedRig(_TakenRig):
    """The taken rig, with what the Shield node recorded on ntFIN2's 18
    takeovers: ArduPilot's own "Fence Breached" STATUSTEXT 2 ms after the mode
    change, in events.jsonl."""

    def run_mission(self, args, tag, shield, timeout_s):
        rc, out = super().run_mission(args, tag, shield, timeout_s)
        p = self.remote / "demo" / "out" / tag / "events.jsonl"
        with open(p, "a", encoding="utf-8", newline="\n") as f:
            f.write(json.dumps({"kind": "autopilot_text", "t": 5.782, "severity": 5,
                                "text": "Fence Breached"}) + "\n")
        return rc, out


def _campaign(argv, td: Path, rig_cls=TS._FakeRig):
    """main() --backend hil on a fake rig of `rig_cls`; (rc, results doc,
    the rig, stdout)."""
    cfg = td / "hil.json"
    cfg.write_text(json.dumps({"store": str(td / "store"),
                               "bundles_dir": str(TS._signed_like_bundles(td))}),
                   encoding="utf-8")
    rigs = []
    saved = S.make_hil_transport
    S.make_hil_transport = lambda c: rigs.append(rig_cls(c)) or rigs[-1]
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            rc = S.main(["--backend", "hil", "--hil-config", str(cfg), "--seed", "1001",
                         "--out", str(td / "s.json"), "--no-bundles"] + argv)
    finally:
        S.make_hil_transport = saved
        for d in (ROOT / S.CAMPAIGN_DIR).glob("tst*"):
            shutil.rmtree(d, ignore_errors=True)
        for d in (ROOT / S.RAIL_STRESS_DIR).glob("tst*"):
            shutil.rmtree(d, ignore_errors=True)
    doc = json.loads((td / "s.json").read_text(encoding="utf-8"))
    return rc, doc, rigs[0] if rigs else None, buf.getvalue()


def _labels(path: Path, cell: str, seed: int, expected) -> Path:
    """A label artefact giving `cell` / `seed` one label under every reading."""
    one = {"expected": expected, "clauses": ["E5"] if expected else [], "e5_spell_s": 0.0,
           "undecided": expected is None}
    path.write_text(json.dumps({
        "schema": S.RAIL_LABELS_SCHEMA, "adopted_kpi_reading": "r2",
        "built": {"code_revisions": ["0" * 40]},
        "labels": {cell: {str(seed): {k: dict(one) for k in ("a", "a2", "r", "r2")}}}}),
        encoding="utf-8")
    return path


def _rescore(runs: list[Path], labels: Path, out: Path) -> tuple[int, str]:
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = S.main(["--seed", "1001", "--only", CELL, "--rescore-rail",
                     *[str(r) for r in runs], "--rail-labels", str(labels),
                     "--out", str(out)])
    return rc, buf.getvalue()


# =========================================================================== #
# 1-2. the lengthened flight, recorded; its label read; one length per table
# =========================================================================== #

def test_the_option_lengthens_every_mission_and_says_so_everywhere():
    assert S.rail_max_s(50.0) == 85.0 and S.rail_max_s(50.0, 35.0) == 120.0
    assert S.rail_max_s(0.0) == 35.0, "the rubric's rail_extend_s is unchanged"
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc, doc, rig, out = _campaign(["--only", CELL, "--hil-campaign", "tstfl1",
                                       "--hil-max-s-add", "35"], td)
        runs = [c for c in rig.calls if c[0] == "run_mission"]
        assert len(runs) == 1, rig.calls
        assert TS._val(runs[0][3], "--max-s") == "120", runs[0][3]
        assert abs(runs[0][4] - 120.0 - S.HilConfig().mission_margin_s) < 1e-6, runs[0][4]
        hil = doc["_scope"]["hil"]
        assert hil["max_s_add"] == 35.0 and hil["flight_length"]["lengthened"] is True, hil
        assert hil["flight_length"]["family_suffix"] == "/flown-longer-35s"
        assert "never merged" in hil["flight_length"]["merge"]
        assert "--hil-max-s-add 35" in doc["_command"], doc["_command"]
        assert "LENGTHENED CAMPAIGN tstfl1" in out, out[:400]
        assert "**LENGTHENED campaign:** every mission flew 35 s past the rail's cap" in \
            KR.render_sweep_markdown(doc)
        run_dir = td / "store" / "tstfl1_001_rail-mission-stub-vla"
        meta = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))["scenario"]
        assert (meta["max_s"], meta["rail_max_s"], meta["rail_max_s_profile"],
                meta["max_s_add"], meta["flight_length"]) == (50.0, 120.0, 85.0, 35.0,
                                                              "lengthened"), meta
        assert meta["family"].endswith("/shield-on/flown-longer-35s"), meta["family"]
        assert meta["cell"] == CELL, "the cell id, which the labels are keyed by, is the cell's"
        score = json.loads((run_dir / S.SCENARIO_SCORE).read_text(encoding="utf-8"))
        win = score["rail_window"]
        assert win["max_s"] == 50.0, "the performance window is the scenario's own cap"
        assert (win["max_s_add"], win["rail_max_s"], win["rail_max_s_profile"]) == (
            35.0, 120.0, 85.0), win
        # The label on the longer flight: the cell's, read against the FSM's
        # verdict over the whole flight (here the headless one, triggered).
        spec = TS.BY_ID[CELL]
        hr = S.run_episode(spec, TS.DEFAULTS, 1001)
        k = score["kpi"]
        assert k["failsafe_triggered"] is hr.extra["fsm"]["failsafe_triggered"] is True
        assert k["failsafe_label"] is True and k["failsafe_matches_label"] == 1.0, k
        assert k["failsafe_censored"] is False
        res = doc["results"][0]
        assert res["status"] == S.episode_verdict(spec, S.score(
            spec, hr.rows, hr.extra, hr.policy, hr.priorities), {}, 1001)[0], res

        # The same campaign at the profile's length: nothing of it written.
        td2 = td / "plain"
        td2.mkdir()
        rc2, doc2, rig2, out2 = _campaign(["--only", CELL, "--hil-campaign", "tstfl2"], td2)
        run2 = [c for c in rig2.calls if c[0] == "run_mission"][0]
        assert TS._val(run2[3], "--max-s") == "85", run2[3]
        assert doc2["_scope"]["hil"]["max_s_add"] == 0.0
        assert doc2["_scope"]["hil"]["flight_length"] == {
            "lengthened": False, "max_s_add": 0.0,
            "cap": "the scenario's max_s + the Loiter operator timeout (30 s) + 5 s"}
        assert "LENGTHENED" not in out2 and "LENGTHENED" not in KR.render_sweep_markdown(doc2)
        plain_dir = td2 / "store" / "tstfl2_001_rail-mission-stub-vla"
        meta2 = json.loads((plain_dir / "metrics.json").read_text(encoding="utf-8"))["scenario"]
        assert not {"max_s_add", "rail_max_s_profile", "flight_length"} & set(meta2), meta2
        assert meta2["rail_max_s"] == 85.0 and meta2["family"].endswith("/shield-on")
        win2 = json.loads((plain_dir / S.SCENARIO_SCORE).read_text(
            encoding="utf-8"))["rail_window"]
        assert "max_s_add" not in win2 and win2["max_s"] == 50.0, win2

        # --rescore-rail: the lengthened run on its own reads the cell's label
        # (keyed by cell and seed, as for any run), names the run flown longer
        # and keeps the count without it; the table says its length.
        art = _labels(td / "labels.json", CELL, 1001, True)
        tab = td / "table.json"
        rc3, out3 = _rescore([run_dir], art, tab)
        assert rc3 == 0, out3
        t = json.loads(tab.read_text(encoding="utf-8"))
        assert t["flight_length"]["max_s_add"] == 35.0 and t["flight_length"]["lengthened"]
        r2 = t["failsafe_labels"]["readings"]["r2"]
        assert (r2["correct"], r2["scored"]) == (1, 1), r2
        assert [x["run"] for x in r2["run_flown_longer"]] == [run_dir.name], r2
        assert r2["run_flown_longer"][0]["max_s_add"] == 35.0
        assert r2["at_profile_length"] == {"scored": 0, "correct": 0}, r2
        assert t["runs"][0]["doc"]["rail_window"]["max_s"] == 50.0
        assert "FLOWN LONGER than the profile (+35 s past the rail's cap" in out3, out3
        assert "FLIGHT LENGTH: a LENGTHENED campaign" in out3
        # The profile's run alone: its own length, nothing named longer.
        rc4, out4 = _rescore([plain_dir], art, tab)
        t4 = json.loads(tab.read_text(encoding="utf-8"))
        assert rc4 == 0 and t4["flight_length"] == {
            "max_s_add": 0.0, "lengthened": False, "what": t4["flight_length"]["what"]}, t4
        r2b = t4["failsafe_labels"]["readings"]["r2"]
        assert r2b["run_flown_longer"] == [] and r2b["at_profile_length"] == {
            "scored": 1, "correct": 1}, r2b
        assert "FLOWN LONGER" not in out4
        # Both in one table: refused, nothing written.
        tab.unlink()
        rc5, out5 = _rescore([run_dir, plain_dir], art, tab)
        assert rc5 == 2 and not tab.exists(), (rc5, out5)
        assert "2 flight lengths" in out5 and "+35 s past the rail cap: 1 run(s)" in out5, out5
        assert rc in (0, 1) and rc2 in (0, 1)


# =========================================================================== #
# 3. still censored after flying longer; the takeover named
# =========================================================================== #

def test_a_lengthened_run_that_still_ends_in_loiter_is_still_censored():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc, doc, _, _ = _campaign(["--only", CELL, "--hil-campaign", "tstfl3",
                                   "--hil-max-s-add", "35"], td, _HeldRig)
        res = doc["results"][0]
        assert res["status"] == "fail" and rc == 1, res
        k = res["kpi"]
        assert k["failsafe_censored"] is True and k["failsafe_matches_label"] is None, k
        msg = [g for g in res["gates_failed"] if g.startswith("fail-safe:")]
        assert msg and "censored, not measured; fly it longer (already flown 35 s past the " \
                       "rail's cap, --hil-max-s-add)" in msg[0], msg
        fl = doc["_failsafe_labels"]
        assert fl["censored_in_loiter"] == 1 and fl["labelled"] == 0, fl
        assert fl["censored_by_cause"] == {"flight_ended": [CELL], "autopilot_takeover": []}, fl
        run_dir = td / "store" / "tstfl3_001_rail-mission-stub-vla"
        art = _labels(td / "labels.json", CELL, 1001, True)
        tab = td / "table.json"
        rc2, out = _rescore([run_dir], art, tab)
        assert rc2 == 0, out
        for key, sc in json.loads(tab.read_text(encoding="utf-8"))[
                "failsafe_labels"]["readings"].items():
            assert sc["scored"] == 0 and sc["failsafe_trigger_correctness"] is None, (key, sc)
            assert sc["censored"] == [{"run": run_dir.name, "cell": CELL, "seed": 1001,
                                       "cause": "flight_ended", "max_s_add": 35.0}], sc
            assert sc["run_flown_longer"] == [], "a censored run is not scored, longer or not"
        assert f"censored   {run_dir.name}  ({CELL}, seed 1001)  - ended still holding " \
               f"after +35 s" in out, out

    # The autopilot took the aircraft: the FSM stopped stepping, and no
    # length decides it - said in the label failure, the summary and the
    # re-score, still censored, never a correct non-trigger.
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc, doc, _, _ = _campaign(["--only", CELL, "--hil-campaign", "tstfl4",
                                   "--hil-max-s-add", "35"], td, _TakenRig)
        res = doc["results"][0]
        assert res["kpi"]["failsafe_censored"] is True and res["status"] == "fail", res
        msg = [g for g in res["gates_failed"] if g.startswith("fail-safe:")][0]
        assert "the autopilot took the aircraft (RTL at 5.78 s) and the FSM stopped " \
               "stepping, so flying longer will not show it" in msg, msg
        assert res["kpi"]["rail_window"]["autopilot_took_over"] == {
            "t": 5.78, "mode": "RTL", "previous": "GUIDED"}
        assert doc["_failsafe_labels"]["censored_by_cause"] == {
            "flight_ended": [], "autopilot_takeover": [CELL]}
        run_dir = td / "store" / "tstfl4_001_rail-mission-stub-vla"
        tab = td / "table.json"
        rc2, out = _rescore([run_dir], _labels(td / "labels.json", CELL, 1001, True), tab)
        r2 = json.loads(tab.read_text(encoding="utf-8"))["failsafe_labels"]["readings"]["r2"]
        assert rc2 == 0 and r2["scored"] == 0 and r2["censored"][0]["cause"] == \
            "autopilot_takeover", r2
        assert "the autopilot took the aircraft; flying longer will not decide it" in out, out
    # A headless episode records no takeover: "fly it longer", as it always said.
    assert S.censored_remedy({"failsafe_censored": True}) == "fly it longer"
    assert S.censored_cause({"failsafe_censored": False}) is None


def test_a_geofence_takeover_of_a_held_flight_is_the_backstops_fail_safe():
    """2026-11-15 (docs/RUBRIC-failsafe.md sections 1 and 9; sweep_scenarios
    BACKSTOP_RULE). The takeover above carried no fence breach on record, so
    it stays censored. The same flight with ArduPilot's "Fence Breached" in
    its events.jsonl - what ntFIN2's 18 dyn-nfz-spawn-ahead runs recorded -
    is a fail-safe triggered by the backstop: scored against the cell's
    label by the campaign and by the re-score, named, counted beside the
    figure, with the reading before the rule (left out as censored) and the
    one that counts it as not triggered printed beside."""
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc, doc, _, out = _campaign(["--only", CELL, "--hil-campaign", "tstfl5"], td,
                                    _FencedRig)
        res = doc["results"][0]
        k = res["kpi"]
        assert (k["failsafe_triggered"], k["failsafe_by"], k["failsafe_censored"],
                k["failsafe_matches_label"]) == (True, "autopilot_geofence", False, 1.0), k
        assert k["rail_window"]["autopilot_took_over"] == {
            "t": 5.78, "mode": "RTL", "previous": "GUIDED"}
        assert k["rail_window"]["takeover_fence_breach"] == {
            "found": True, "t": 5.782, "text": "Fence Breached", "severity": 5,
            "after_takeover_s": 0.002}
        b = k["failsafe_backstop"]
        assert b["counts"] and b["fsm_state"] == "Loiter" and b["fsm_censored"] is True, b
        assert not [g for g in res["gates_failed"] if g.startswith("fail-safe:")], res
        fl = doc["_failsafe_labels"]
        assert (fl["labelled"], fl["correct"], fl["censored_in_loiter"]) == (1, 1, 0), fl
        assert fl["triggered_by"] == {"fsm": 0, "autopilot_geofence": 1}
        assert fl["backstop_triggered_episodes"] == [CELL]
        assert fl["before_backstop_rule"] == {
            "labelled": 0, "correct": 0, "fail_safe_correctness": None,
            "left_out_as_censored": 1}, fl["before_backstop_rule"]
        assert fl["backstop_as_not_triggered"] == {
            "labelled": 1, "correct": 0, "fail_safe_correctness": 0.0}
        assert [e for e in fl["triggers_by_edge"]["true"] if e.endswith("autopilot GeoFence")], \
            fl["triggers_by_edge"]
        assert "of them triggered by the autopilot's GeoFence backstop while the FSM held " \
               "(counted since 2026-11-15): 1; the same runs read as before that rule " \
               "(1 left out as censored): 0/0 = None; counted as not triggered: 0/1 = 0.0" \
            in out, out
        run_dir = td / "store" / "tstfl5_001_rail-mission-stub-vla"
        stored = json.loads((run_dir / S.SCENARIO_SCORE).read_text(encoding="utf-8"))
        assert stored["kpi"]["failsafe_by"] == "autopilot_geofence"
        tab = td / "table.json"
        rc2, out2 = _rescore([run_dir], _labels(td / "labels.json", CELL, 1001, True), tab)
        fl2 = json.loads(tab.read_text(encoding="utf-8"))["failsafe_labels"]
        assert rc2 == 0 and fl2["backstop_rule"]["since"] == "2026-11-15", out2
        for key, sc in fl2["readings"].items():
            assert (sc["correct"], sc["scored"], sc["censored"]) == (1, 1, []), (key, sc)
            assert [(t["run"], t["mode"], t["verdict"]) for t in sc["backstop"]] == [
                (run_dir.name, "RTL", "correct")], sc["backstop"]
            assert sc["before_backstop_rule"]["scored"] == 0 \
                and sc["before_backstop_rule"]["left_out_as_censored"] == 1, sc
            assert (sc["backstop_as_not_triggered"]["scored"],
                    sc["backstop_as_not_triggered"]["missed_triggers"]) == (1, 1), sc
        assert f"backstop   {run_dir.name}  ({CELL}, seed 1001)  - the autopilot's GeoFence " \
               f"took it (RTL at 5.78 s) with the FSM in Loiter: a fail-safe expected, " \
               f"correct" in out2, out2
        # The same flight on a label that expects none: a false trigger.
        rc3, out3 = _rescore([run_dir], _labels(td / "labels0.json", CELL, 1001, False), tab)
        r2 = json.loads(tab.read_text(encoding="utf-8"))["failsafe_labels"]["readings"]["r2"]
        assert (r2["correct"], r2["scored"]) == (0, 1) and [t["run"] for t in r2["false"]] == [
            run_dir.name], r2
        assert r2["backstop"][0]["verdict"] == "false" and "a false trigger" in out3, out3


# =========================================================================== #
# 4. the KPI report keeps a lengthened campaign apart
# =========================================================================== #

def _summary(root: Path, name: str, *, add=None, rev="feedc0de1234") -> Path:
    summ = root / f"hil_{name}"
    summ.mkdir(parents=True, exist_ok=True)
    hil = {"campaign": name, "flyable": 1, "refused": {},
           "config": {"store": (root / "hil_smoke").as_posix()}}
    if add is not None:
        hil.update(max_s_add=add, flight_length=S.hil_flight_length(add))
    doc = {"_command": f"python experiments/sweep_scenarios.py --backend hil "
                       f"--hil-campaign {name}", "_backend": "hil", "_runtime_s": 1.0,
           "_manifest_common": {"code_revision": rev},
           "_scope": {"profile": "nightly", "grant_scope": "x", "cadence": "nightly",
                      "hil": hil},
           "results": [{"id": "c", "id_cell": "c", "seed": 1001, "arm": "on",
                        "status": "pass", "mission_tag": f"{name}_001_c"}]}
    (summ / "sweep.json").write_text(json.dumps(doc), encoding="utf-8")
    return summ


def _table(path: Path, run: str, add: float | None, *, declared=None) -> Path:
    scen = {"run_id": run} if add is None else {"run_id": run, "max_s_add": add}
    lists = {k: [] for k in ("false", "missed", "censored", "undecided", "unlabelled",
                             "not_measured", "not_accepted", "label_flown_longer")}
    doc = {"schema": "rail-rescore/1", "rail_gates": "ideal",
           "runs": [{"run": run, "cell": "c", "seed": 1001, "before": {"status": "pass"},
                     "after": {}, "doc": {"scenario": scen, "metrics": {"shield": "on"},
                                          "kpi": {"failsafe_triggered": True}}}],
           "failsafe_labels": {"artefact": "docs/data/failsafe_labels_rail.json",
                               "built": {"code_revisions": ["0" * 12]},
                               "adopted_kpi_reading": "r2", "shield_off_runs_left_out": 0,
                               "readings": {"r2": {**lists, "reading": "r2",
                                                   "adopted_kpi_reading": True, "runs": 1,
                                                   "scored": 1, "correct": 1,
                                                   "failsafe_trigger_correctness": 1.0,
                                                   "wilson_low_95": 0.2}}}}
    if declared is not None:
        doc["flight_length"] = {"max_s_add": declared}
    path.write_text(json.dumps(doc), encoding="utf-8")
    return path


def _refused(fn, word):
    try:
        fn()
    except SystemExit as e:
        assert word in str(e), (word, str(e))
        return str(e)
    raise AssertionError(f"not refused ({word})")


def test_the_kpi_report_keeps_a_lengthened_campaign_apart():
    sc = {"backend": "hil", "template": "corridor_swap", "cell": "corridor-swap-shifted"}
    assert KR.scenario_family(sc, "on") == "hil-stress/corridor_swap/corridor-swap-shifted/" \
                                           "shield-on"
    assert KR.scenario_family(dict(sc, max_s_add=35.0), "on") == \
        "hil-stress/corridor_swap/corridor-swap-shifted/shield-on" + S.lengthened_suffix(35.0)
    assert S.lengthened_suffix(35.0) == "/flown-longer-35s" and S.lengthened_suffix(0.0) == ""
    assert KR.max_s_add({"max_s_add": "nan"}) == 0.0 == KR.max_s_add(None)
    root = Path(tempfile.mkdtemp(prefix="kr_longer_"))
    try:
        kpi, longer, longer2 = (_summary(root, "kx"), _summary(root, "lx", add=35.0),
                                _summary(root, "ly", add=35.0))
        c = KR.load_campaign(longer)
        assert c["max_s_add"] == 35.0 and c["flight_length"]["lengthened"] is True, c
        assert KR.load_campaign(kpi)["max_s_add"] == 0.0, "a campaign before the option"
        # Never merged with a campaign of another length, either way round;
        # two campaigns lengthened alike are one flight and may be.
        msg = _refused(lambda: KR.load_campaigns([kpi, longer]), "cannot supplement")
        assert "LENGTHENED, 35 s past the rail's cap" in msg and "never merged" in msg, msg
        _refused(lambda: KR.load_campaigns([longer, kpi]), "the profile's own length")
        assert KR.load_campaigns([longer, longer2])["max_s_add"] == 35.0
        md = "\n".join(KR._render_campaign(c, []))
        assert "**LENGTHENED campaign**: every mission flew 35 s past" in md, md
        assert "`.../flown-longer-35s`" in md and "never merged into its figure" in md
        assert "Flight length: the profile's own" in "\n".join(
            KR._render_campaign(KR.load_campaign(kpi), []))
        assert any("LENGTHENED campaign" in r for r in KR.EPISODE_CHOICE_RULES)
        # Re-score tables: one length each, never merged across lengths, and
        # never applied to a campaign of another length.
        t0 = _table(root / "t0.json", "kx_001_c", None)
        t35 = _table(root / "t35.json", "lx_001_c", 35.0, declared=35.0)
        assert KR.load_rescore(t0)["max_s_add"] == 0.0
        assert KR.load_rescore(t35)["max_s_add"] == 35.0
        _refused(lambda: KR.load_rescores([t0, t35]), "flight length")
        bad = _table(root / "bad.json", "lx_001_c", 35.0, declared=0.0)
        _refused(lambda: KR.load_rescore(bad), "more than one flight length")

        def report(camp, rs, runs=()):
            return KR.build_report(list(runs), [], 10, False, "cmd", wp1_path=None,
                                   csp_json=None, tick_profiles=None, campaign=camp,
                                   rescore=rs)
        _refused(lambda: report(longer, t0), "--hil-max-s-add")
        _refused(lambda: report(kpi, t35), "never with the profile's")
        rep = report(longer, t35)
        assert rep["inputs"]["campaign_max_s_add"] == 35.0
        assert rep["inputs"]["rescore_rail_max_s_add"] == 35.0
        assert report(kpi, t0)["inputs"]["campaign_max_s_add"] == 0.0
        # Rail runs of two lengths brought in by one glob: refused; the
        # lengthened run alone is filed under its own row.
        from guardrail import load_policy
        import test_wp4outputs_kpi_report as W
        pol = load_policy(W.POLICY)
        store = root / "runs"
        for name, add in (("lx_001_c", 35.0), ("kx_001_c", None)):
            run = W._hil_run(store, name, pol)
            m = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
            m["scenario"] = {"schema": "rail-scenario/1", "backend": "hil", "run_id": name,
                             "run_nonce": "n", "scenario_id": "corridor-swap-shifted",
                             "template": "corridor_swap", "cell": "corridor-swap-shifted",
                             "seed": 1001, "arm": "on",
                             **({"max_s_add": add} if add else {})}
            (run / "metrics.json").write_text(json.dumps(m), encoding="utf-8")
        _refused(lambda: report(None, None, [(store / "*").as_posix()]),
                 "more than one length")
        one = report(None, None, [(store / "lx_*").as_posix()])
        e = next(x for x in one["episodes"] if x["id"] == "lx_001_c")
        assert e["family"].endswith("/shield-on/flown-longer-35s") and e["max_s_add"] == 35.0, e
    finally:
        shutil.rmtree(root, ignore_errors=True)


# =========================================================================== #
# 5. refused where it cannot apply; the dry run shows it
# =========================================================================== #

def test_the_option_is_refused_where_it_cannot_apply_and_the_dry_run_shows_it():
    def run(argv):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = S.main(argv)
        return rc, buf.getvalue()
    rc, out = run(["--only", CELL, "--hil-max-s-add", "35"])
    assert rc == 2 and "--hil-* options are for --backend hil" in out, out
    rc, out = run(["--only", CELL, "--rescore-rail", "nowhere_*", "--out", "x.json",
                   "--hil-max-s-add", "35"])
    assert rc == 2 and "--hil-* options" in out, out
    rc, out = run(["--only", CELL, "--backend", "hil", "--rescore-rail", "nowhere_*",
                   "--out", "x.json", "--hil-max-s-add", "35"])
    assert rc == 2 and "reads each run's own length" in out, out
    for bad in ("0", "-5", "nan", "inf", "601", "long"):
        rc, out = run(["--only", CELL, "--backend", "hil", "--hil-dry-run",
                       "--hil-max-s-add", bad])
        assert rc == 2 and "REFUSED: --hil-max-s-add" in out, (bad, out)
    made = []
    saved = S.make_hil_transport
    S.make_hil_transport = lambda c: made.append(c)
    try:
        with tempfile.TemporaryDirectory() as td:
            cfg = Path(td) / "hil.json"
            cfg.write_text(json.dumps({"bundles_dir": str(TS._signed_like_bundles(Path(td)))}),
                           encoding="utf-8")
            rc, out = run(["--only", CELL, "--backend", "hil", "--hil-dry-run", "--seed",
                           "1001", "--hil-config", str(cfg), "--hil-campaign", "dry",
                           "--hil-max-s-add", "35"])
    finally:
        S.make_hil_transport = saved
    assert rc == 0 and not made, (rc, made)
    assert "LENGTHENED CAMPAIGN (--hil-max-s-add 35)" in out, out
    assert "(cap 120 s: the scenario's 50 s, the rail's 85 s, +35 s)" in out, out
    assert "--max-s=120" in out, out


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
