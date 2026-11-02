"""The scenario sweep harness, the altitude defect it found, and the stress
harness it became.

Run either way:
    pytest tests/test_sweep.py -v
    python tests/test_sweep.py

WHY THIS FILE EXISTS

A test harness needs its own tests more than most code does, because its failure
mode is silence: a sweep that runs nothing and prints "11 passed" is worse than
no sweep at all. So the tests below are mostly about the harness being HONEST -
that a skipped scenario says skipped, that a gate on an unmeasured field fails
rather than passes, that a known failure is never reported as a pass, and that
the unshielded control arm can still score a real violation.

The stress-harness groups (2026-10-06) hold the same line for the new parts: a
stressor that never reaches the Shield, a window switch that never happens, a
manifest with a seventh field, a SITL command line the node would reject, and a
pinned defect that quietly started passing must each be caught.

The review round of the same day added the ones a reviewer's mutations got
past: gusts replaced by zero, `reached_goal` not passed into compute(), the
oracle sharing the flying Shield, labels or a whole-scenario known_failure
ignored by main(), false and missed fail-safes swapped, wind not turned with
the mission, and every way a run could select nothing and still exit 0.

2026-10-07 (stress-harness-2): the Shield's events, its escalation FSM and the
arms. Every grant event type must reach the Shield and move the zone or the
rule it names; the switch instant must be the Shield's, on the conservative
side of the schedule's edge; RTL / Land must be outcomes the FSM produced and
the autopilot flew; a moving zone that sweeps over a waiting aircraft must not
be counted as the aircraft's own breach; paraphrase and prefix arms must log
their ids and fly the same flight; and the run must write the grant's KPI
report, one row per template and cell. A test that cannot run on this host
returns SKIP and is counted apart, never as a pass.
"""
import ast
import contextlib
import copy
import io
import json
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import yaml                                                       # noqa: E402

import sweep_scenarios as S                                       # noqa: E402
from guardrail import load_policy                                 # noqa: E402
from guardrail.models import Action4D, State                      # noqa: E402
from guardrail.scenario_spec import Library, ScenarioSpec         # noqa: E402
from guardrail.shield import Shield                               # noqa: E402

LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")
DEFAULTS = LIBRARY.defaults
FAMILIES = LIBRARY.by_id()
BY_ID = {sid: fam.first.spec for sid, fam in FAMILIES.items()}
SKIP = "SKIP"


def _run(sid, seed=0):
    return S.run_headless(BY_ID[sid], DEFAULTS, seed)


def _cell(sid, **params):
    """The cell of a family whose parameters include `params`."""
    for c in FAMILIES[sid].cells(samples=3):
        if all(c.params.get(k) == v for k, v in params.items()):
            return c.spec
    raise KeyError(f"{sid}: no cell with {params}")


def _with(spec: ScenarioSpec, **changes) -> ScenarioSpec:
    """A copy of a spec with nested fields replaced ('mission.clock_start').

    Since 2026-11-02 every library label names its rubric clause
    (docs/RUBRIC-failsafe.md), and a clause that contradicts its label is
    refused: a test that writes a fail-safe label by hand drops the clause
    (and the cell labels) unless it gives one, and a test that turns the
    Shield or its FSM off drops a fail-safe OUTCOME label (RTL_triggered /
    Land_triggered), which nothing could then produce."""
    changes = _relabel_safely(spec.model_dump(mode="json", by_alias=True), changes)
    d = spec.model_dump(mode="json", by_alias=True)
    for path, value in changes.items():
        node = d
        keys = path.split(".")
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    return ScenarioSpec.model_validate(d)


def _relabel_safely(current: dict, changes: dict) -> dict:
    """_with's and _raw's label rules (see _with)."""
    out = dict(changes)
    if "expected_failsafe" in out and "failsafe_clause" not in out:
        out["failsafe_clause"] = None
        out.setdefault("failsafe_cells", [])
    if (out.get("escalation") is False or out.get("shield") is False) \
            and current.get("expected_outcome") in ("RTL_triggered", "Land_triggered") \
            and "expected_outcome" not in out:
        out["expected_outcome"] = None
    return out


# The 13 original scenarios' own lengths, where they changed on 2026-11-02
# (docs/RUBRIC-failsafe.md section 3: these four now fly 50 s so the Loiter
# timeout's outcome is shown). The identity pins below are those flights.
LEGACY_TICKS = {"altitude-ceiling-hold": 300, "corridor-curfew-in-hours": 300,
                "standoff-reclassified": 400, "standoff-wedge": 300}


def _kpi(spec, rows, extra, policy=None):
    return S.score(spec, rows, extra, policy or spec.policy.load(ROOT))


# --------------------------------------------------------------------------- #
# the harness tells the truth
# --------------------------------------------------------------------------- #

def test_a_gate_on_an_unmeasured_field_fails():
    """The quiet way a harness stops testing anything.

    If a field is missing - renamed, never computed, spelled wrong - the gate
    must FAIL. Treating absent as satisfied would let a typo silently retire a
    safety check while the sweep kept printing green.
    """
    bad = S.check_gates({"no_such_field": {"max": 0.0}}, {"other": 1})
    assert bad and "not measured" in bad[0], bad


def test_gates_compare_in_the_direction_they_say():
    assert not S.check_gates({"v": {"max": 1.0}}, {"v": 1.0})
    assert S.check_gates({"v": {"max": 1.0}}, {"v": 1.1})
    assert not S.check_gates({"v": {"min": 1.0}}, {"v": 1.0})
    assert S.check_gates({"v": {"min": 1.0}}, {"v": 0.9})
    assert S.check_gates({"v": {"equals": True}}, {"v": False})


def test_a_time_to_safe_bound_is_vacuous_only_when_measured_and_never_censored():
    """The grant example's `mean_time_to_safe_s: "<=2.0"` on a flight that was
    never unsafe has no mean to compare - and that is a pass only when the
    episode WAS measured. Never recovering is a fail; not measuring is a fail."""
    g = {"mean_time_to_safe_s": {"max": 2.0}}
    clean = {"mean_time_to_safe_s": None, "time_to_safe_not_measurable": False,
             "time_to_safe_episodes": 0, "time_to_safe_censored": 0}
    assert not S.check_gates(g, clean)
    assert "never recovered" in S.check_gates(g, {**clean, "time_to_safe_censored": 1})[0]
    assert "not measured" in S.check_gates(
        g, {**clean, "time_to_safe_not_measurable": True})[0]


def test_the_sitl_backend_reports_unavailable_rather_than_pretending():
    """Returning None means SKIP; the runner prints it and counts it apart.

    The alternative - a backend that silently falls back to headless - would
    report ArduPilot results this host never produced. On a host that HAS the
    rail this returns SKIP, counted apart (follow-up #84: it printed SKIP and
    was counted as a pass).
    """
    if S.sitl_available():
        return SKIP
    assert S.run_sitl(BY_ID["nfz-head-on"], "sweep_x") is None


# --------------------------------------------------------------------------- #
# the SITL command line
# --------------------------------------------------------------------------- #

def _argparse_flags(script: Path) -> set[str]:
    """Every option string the script's argparse declares, read from its source
    (the node imports rclpy, which this machine need not have). A flag passed
    as a name (since 2026-10-12 the node declares `DEV_YAML_FLAG`, imported
    from guardrail.manifest) is resolved from the script's own module-level
    string constants or from the guardrail module it was imported from."""
    import importlib
    tree = ast.parse(script.read_text(encoding="utf-8"))
    consts: dict[str, str] = {}
    for node in tree.body:
        if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant)
                and isinstance(node.value.value, str)):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name):
                    consts[tgt.id] = node.value.value
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


def test_every_sitl_flag_is_one_the_node_accepts():
    """REGRESSION (audit card WP4-04). The backend passed `--out` to
    sitl/run_sitl_demo.py, which has no such argument, so `--backend sitl` could
    only ever die in argparse. It now drives run_ros2_demo.sh, which forwards
    $3.. to ros2_shield_node.py - checked here flag by flag."""
    # Every flag, the policy-reference ones included: until the review of
    # 2026-10-12 --dev-yaml and --expected-hash were added to the node's
    # set here, so this check could not catch a flag the node drops.
    node_flags = _argparse_flags(ROOT / S.SITL_NODE)
    assert "--out" not in node_flags, "the check would not catch the old bug"
    assert "--out" not in _argparse_flags(ROOT / "sitl" / "run_sitl_demo.py")
    script = (ROOT / S.SITL_SCRIPT).read_text(encoding="utf-8")
    assert '"${@:3}"' in script, "run_ros2_demo.sh no longer forwards $3.. to the node"
    for sid in ("nfz-head-on", "nfz-head-on-control", "standoff-approach"):
        cmd = S.build_sitl_cmd(BY_ID[sid], "sweep_t")
        assert cmd[:3] == ["wsl", "bash", S.SITL_SCRIPT], cmd
        assert cmd[3] in ("on", "off") and cmd[4] in ("", "--dynamic"), cmd
        passed = {a for a in cmd[5:] if a.startswith("--")}
        assert passed <= node_flags, f"{sid}: node rejects {passed - node_flags}"
    assert "--subject" in S.build_sitl_cmd(BY_ID["standoff-approach"], "t")


def test_the_sitl_rail_refuses_what_it_cannot_reproduce():
    """It flies its own mission with no clock, stressors or event schedule."""
    assert "scenario clock" in S.sitl_refusal(BY_ID["corridor-curfew-in-hours"])
    assert "events" in S.sitl_refusal(BY_ID["dyn-nfz-spawn-ahead"])
    assert "stressors" in S.sitl_refusal(BY_ID["gps-noise-within-margin"])
    assert "stressors" in S.sitl_refusal(BY_ID["gps-dropout"])
    assert "relabel" in S.sitl_refusal(BY_ID["standoff-reclassified"])


def test_the_sitl_rail_refuses_a_mission_that_is_not_its_own():
    """REGRESSION (review 2026-10-06). The first version refused only events,
    stressors, clocks and relabels, so nfz-head-on, speed-cap and the corridor
    scenarios would have been flown as the rail's fixed mission and scored
    against their own gates under their own ids - a pass describing a
    different flight. Since the node takes --target / --speed (2026-10-07)
    the target is the scenario's own, but the start (the pad) and the pilot
    (StubVLA) are still the rail's: every scenario that differs there is
    refused, with what differs."""
    why = S.sitl_refusal(BY_ID["nfz-head-on"])
    assert why and "its own mission" in why, why
    assert "start (-20, 15)" in why and "pilot goto" in why, why
    assert "target" not in why, "the target is passed now; it is not a mismatch"
    flyable = {sid for sid, spec in BY_ID.items() if not S.sitl_refusal(spec)}
    assert flyable == {"rail-mission-stub-vla"}, flyable
    # A scenario on the rail's start is still refused on the pilot: StubVLA is
    # not the headless goto law.
    same = _with(BY_ID["nfz-head-on"], **{"mission.start_pose.x": 0.0,
                                         "mission.start_pose.y": 0.0,
                                         "mission.target.x": 30.0,
                                         "mission.target.y": 30.0})
    diff = S._mission_mismatch(same)
    assert len(diff) == 1 and diff[0].startswith("pilot goto"), diff
    assert len(S._mission_mismatch(BY_ID["nfz-head-on"])) == 2
    # ... and a stub_vla mission off the pad, or at another cruise altitude.
    off_pad = _with(BY_ID["rail-mission-stub-vla"], **{"mission.start_pose.x": 5.0})
    assert any(d.startswith("start") for d in S._mission_mismatch(off_pad))
    high = _with(BY_ID["rail-mission-stub-vla"], **{"mission.target.up": 25.0})
    assert any(d.startswith("cruise altitude") for d in S._mission_mismatch(high))


def test_the_rail_mission_goes_to_the_node_with_its_target_speed_seed_and_cap():
    """REGRESSION (WP4-12, follow-up #130's sweep half). The manifest's
    random_seed was a hard-coded 0 on the rail; the node takes --seed now, and
    the scenario's target, speed and time cap travel with it. The one library
    scenario the rail can fly under its own id carries all four."""
    spec = BY_ID["rail-mission-stub-vla"]
    cmd = S.build_sitl_cmd(spec, "sweep_t", seed=1002, defaults=DEFAULTS)
    val = {cmd[i]: cmd[i + 1] for i in range(5, len(cmd) - 1) if cmd[i].startswith("--")}
    assert val["--seed"] == "1002" and val["--target"] == "30,30", val
    # 50 s since 2026-11-02 (it flies past the Loiter timeout).
    assert val["--speed"] == "6" and val["--max-s"] == "50", val
    assert {a for a in cmd[5:] if a.startswith("--")} <= _argparse_flags(ROOT / S.SITL_NODE)
    # A rotation about the start turns only the target, and the turned target
    # is what the node is given.
    turned = _with(spec, **{"mission.rotate_deg": 90.0})
    tcmd = S.build_sitl_cmd(turned, "t")
    tx, ty = (float(v) for v in tcmd[tcmd.index("--target") + 1].split(","))
    assert abs(tx + 30.0) < 1e-6 and abs(ty - 30.0) < 1e-6, (tx, ty)
    assert not S.sitl_refusal(turned)


def test_the_rail_mission_constants_are_the_nodes_own():
    """SITL_RAIL_* must describe what the node flies, or the refusal above
    compares scenarios against a mission nobody flies."""
    from guardrail.compiler import PLACES
    node = (ROOT / S.SITL_NODE).read_text(encoding="utf-8")
    stub = (ROOT / "sitl" / "ros2_vla_stub_node.py").read_text(encoding="utf-8")
    assert f'parse_command("{S.SITL_RAIL_COMMAND}")' in node
    assert S.SITL_RAIL_COMMAND in stub and "StubVLA" in stub
    assert PLACES["northeast pad"] == S.SITL_RAIL_TARGET


def test_a_bundle_policy_goes_to_the_nodes_bundle_flag():
    """`--policy` takes a YAML; a signed bundle passed there would be refused
    by the node (or worse, read as YAML). bundle_path maps to --bundle.
    Since 2026-10-12 a bundle travels with the hash it must hold
    (`--expected-hash`), and while the node does not declare that flag the
    bundle is refused instead: it would be dropped and the bundle flown
    unpinned (tests/test_harnesstruth_bundles.py). Re-pinned on the review
    of 2026-10-12, which removed the allow-list that had let the undeclared
    flag through this check."""
    spec = _with(BY_ID["nfz-head-on"],
                 policy={"bundle_path": "demo/out/x/policy_bundle.tar.gz",
                         "expected_hash": "sha256:" + "0" * 64})
    node = _argparse_flags(ROOT / S.SITL_NODE)
    try:
        cmd = S.build_sitl_cmd(spec, "t")
    except S.UnsupportedScenario as e:
        assert S.SITL_EXPECTED_HASH_FLAG not in node and S.SITL_EXPECTED_HASH_FLAG in str(e), e
        return None
    assert cmd[cmd.index("--bundle") + 1] == "demo/out/x/policy_bundle.tar.gz"
    assert "--policy" not in cmd
    assert {a for a in cmd[5:] if a.startswith("--")} <= node


def test_a_stale_rail_log_is_refused_not_scored():
    """The tag is deterministic, so a launch that exits 0 without writing a
    log would otherwise score the PREVIOUS run's flight_log.jsonl."""
    with tempfile.TemporaryDirectory() as td:
        log = Path(td) / "flight_log.jsonl"
        try:
            S.read_fresh_log(log, time.time())
        except RuntimeError as e:
            assert "wrote no" in str(e)
        else:
            raise AssertionError("a missing log was accepted")
        log.write_text('{"t": 0}\n', encoding="utf-8")
        old = time.time() - 3600
        os.utime(log, (old, old))
        try:
            S.read_fresh_log(log, time.time())
        except RuntimeError as e:
            assert "predates this launch" in str(e)
        else:
            raise AssertionError("an hour-old log was scored as this launch's")
        launched = time.time()
        log.write_text('{"t": 0}\n{"t": 0.1}\n', encoding="utf-8")
        assert len(S.read_fresh_log(log, launched)) == 2


def _as_rail_rows(rows):
    """Headless rows in the rail log's shape for what rail_measurements reads:
    `flown` is a bool there (sent or not), `setpoint` is "none" while the
    autopilot holds the aircraft."""
    out = []
    for r in rows:
        rr = dict(r)
        rr["flown"] = r.get("owner") != "autopilot"
        out.append(rr)
    return out


def test_the_rail_mission_scored_from_the_rail_log_gets_the_headless_verdict():
    """REVIEW 2026-10-19 (harness-tools/sitl-backend-cannot-pass). run_sitl
    returned the rail's rows with {"seed_applied": True} only, so breaches,
    polygon_entries and the FSM record were never measured and
    rail-mission-stub-vla - the one scenario the rail may fly - FAILED as
    "not measured" however the flight went. The rail log and metrics.json
    hold what is needed; scored from them, the headless flight's own rows get
    the headless verdict, and a step into the zone fails it."""
    spec = BY_ID["rail-mission-stub-vla"]
    run = S.run_episode(spec, DEFAULTS, 0)
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    headless = S.episode_verdict(spec, res, {}, 0)
    # A known failure since 2026-11-02: past the Loiter timeout the FSM sends
    # it home, a false fail-safe by the rubric (N1), pinned on the label only.
    assert headless[0] == "known_failure", headless
    rows = _as_rail_rows(run.rows)
    # what main() scored before: nothing measured, a FAIL for a clean flight
    old = S.score(spec, rows, {"seed_applied": True}, run.policy)
    assert S.episode_verdict(spec, old, {}, 0)[0] == "fail"
    metrics = {"reached": run.extra["reached_goal"], "fsm": run.extra["fsm"]}
    extra = S.sitl_extra(spec, rows, metrics, run.policy)
    for k in ("breaches", "polygon_entries", "ticks_inside_fence_polygon",
              "autopilot_breaches", "autopilot_polygon_entries"):
        assert extra[k] == run.extra[k], (k, extra[k], run.extra[k])
    assert abs(extra["min_true_dist_to_fence_m"] - run.extra["min_true_dist_to_fence_m"]) < 1e-3
    rail = S.score(spec, rows, extra, run.policy)
    assert S.episode_verdict(spec, rail, {}, 0) == headless, (rail.get("breaches"),)
    # one row steps into the demo zone after a tick the rail flew: a breach
    # and an entry, never zero. (This flight hands over to LOITER at 0.1 s,
    # so the tick before is made one the rail flew.)
    zone = S.Shield(run.policy).zones_now(15.0)[0].polygon
    c = zone.centroid
    k = len(rows) // 2
    bad = (rows[:k - 1] + [dict(rows[k - 1], flown=True, setpoint="pass")]
           + [dict(rows[k], x=c.x, y=c.y, unsafe=True)] + rows[k + 1:])
    extra = S.sitl_extra(spec, bad, metrics, run.policy)
    assert extra["breaches"] == 1 and extra["polygon_entries"] == 1, extra
    status, gates, _, _ = S.episode_verdict(spec, S.score(spec, bad, extra, run.policy), {}, 0)
    assert status == "fail" and any(g.startswith("breaches=1") for g in gates), gates
    assert any(g.startswith("polygon_entries=1") for g in gates), gates
    # ...and on a step the autopilot made, it is the autopilot's
    bad[k - 1] = dict(bad[k - 1], flown=False, setpoint="none")
    extra = S.sitl_extra(spec, bad, metrics, run.policy)
    assert (extra["breaches"], extra["autopilot_breaches"]) == (0, 1), extra
    assert (extra["polygon_entries"], extra["autopilot_polygon_entries"]) == (0, 1), extra
    # an older rail log without `unsafe`: breaches not measured, never 0
    extra = S.sitl_extra(spec, [{k2: v for k2, v in r.items() if k2 != "unsafe"}
                                for r in rows], metrics, run.policy)
    assert "breaches" not in extra
    # a Shield-on flight with no FSM record: the label is "not measured"
    assert "fsm" not in S.sitl_extra(spec, rows, {"reached": True}, run.policy)


def test_a_rail_log_under_a_clock_switched_policy_measures_no_breach():
    """REVIEW of F8-harness-tools (P4). rail_measurements left the polygon
    fields out for a policy whose rules switch with the clock (_timed: which
    zone was in force at a row's wall time is not in the log) but still
    counted breaches from the rail's `unsafe`, which the rail's own Shield
    judged at wall time - so a weekly window opening under the aircraft was
    scored as a breach the flight made. sitl_refusal refuses a scenario
    clock, not a timed policy, so nothing kept such a flight out. Breaches
    are now left out for it too ("not measured", never 0). Shown failing
    before the fix (breaches 1 under the timed policy). Flown at its old
    30 s (the library flies it 50 s since 2026-11-02), where it ends in
    Loiter with no fail-safe outcome - what the outcome checks below need."""
    spec = _with(BY_ID["rail-mission-stub-vla"], ticks=300)
    run = S.run_episode(spec, DEFAULTS, 0)
    rows = _as_rail_rows(run.rows)
    timed = load_policy(ROOT / "policies" / "corridor_survey.yaml")
    assert S._timed(timed) and not S._timed(run.policy)
    # `unsafe` turns on mid-flight after a tick the rail flew, and stays on:
    # what a curfew window coming into force under the aircraft looks like.
    k = len(rows) // 2
    switched = (rows[:k - 1] + [dict(rows[k - 1], flown=True, setpoint="pass")]
                + [dict(r, unsafe=True) for r in rows[k:]])
    # the null: under a policy that cannot switch it is the flight's breach
    assert S.rail_measurements(switched, run.policy)["breaches"] == 1
    metrics = {"reached": run.extra["reached_goal"], "fsm": run.extra["fsm"]}
    extra = S.sitl_extra(spec, switched, metrics, timed)
    for f in ("breaches", "autopilot_breaches", "polygon_entries",
              "autopilot_polygon_entries", "ticks_inside_fence_polygon",
              "min_true_dist_to_fence_m"):
        assert f not in extra, (f, extra.get(f))
    assert S.check_gates({"breaches": {"max": 0}}, extra) == ["breaches: not measured"]
    # REVIEW of the F8b merge (P1): the count left out must not read as 0 in
    # the outcome label either. score() built mission_outcome from
    # extra.get("breaches", 0), so with `unsafe` on for 5 rows mid-flight and
    # the goal reached the label went from `fail` (before e911e52) to
    # `success`, while compute() itself said unsafe_position_entered. Shown
    # failing first: 'success' at a8ae8d5. Now compute()'s reading decides,
    # for a brief stretch and for one still open at the end. (This headless
    # flight hands over to LOITER and misses its goal, which would fail the
    # label on its own, so the node is made to report it reached.)
    reached = dict(metrics, reached=True)
    brief = (rows[:k - 1] + [dict(rows[k - 1], flown=True, setpoint="pass")]
             + [dict(r, unsafe=True) for r in rows[k:k + 5]] + rows[k + 5:])
    for flight, why in ((brief, "unsafe_position_entered"),
                        (switched, "unsafe_never_recovered")):
        ex = S.sitl_extra(spec, flight, reached, timed)
        assert "breaches" not in ex and ex["reached_goal"] is True, ex
        assert not (ex.get("fsm") or {}).get("outcome_label"), ex["fsm"]
        res = S.score(spec, flight, ex, timed)
        assert why in res["mission_fail_reasons"], res["mission_fail_reasons"]
        assert res["mission_outcome"] == "fail", (why, res["mission_outcome"])
    # the null: the same flight with no unsafe row is a success under it
    assert not any(r.get("unsafe") for r in rows)
    ex = S.sitl_extra(spec, rows, reached, timed)
    assert S.score(spec, rows, ex, timed)["mission_outcome"] == "success"


def test_run_sitl_reads_this_launchs_log_and_metrics_and_scores_them():
    """run_sitl end to end, with the rail's launch replaced by one that writes
    a flight log and metrics.json (the headless flight's): the scored
    verdict is the headless one; a stale metrics.json is refused."""
    spec = BY_ID["rail-mission-stub-vla"]
    run = S.run_episode(spec, DEFAULTS, 0)
    rows = _as_rail_rows(run.rows)
    metrics = {"reached": run.extra["reached_goal"], "fsm": run.extra["fsm"]}
    saved = (S.ROOT, S.sitl_available, S.build_sitl_cmd, S.subprocess.run)
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        shutil.copytree(ROOT / "policies", root / "policies")
        stale = {"on": False}

        def fake_run(cmd, **kw):
            out = root / "demo" / "out" / cmd[1]
            out.mkdir(parents=True, exist_ok=True)
            (out / "flight_log.jsonl").write_text(
                "".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
            m = out / "metrics.json"
            m.write_text(json.dumps(metrics), encoding="utf-8")
            if stale["on"]:
                old = time.time() - 3600
                os.utime(m, (old, old))
            return subprocess.CompletedProcess(cmd, 0, "", "")
        try:
            S.ROOT, S.sitl_available = root, (lambda: True)
            S.build_sitl_cmd = lambda spec_, tag, *a, **k: ["fake", tag]
            S.subprocess.run = fake_run
            got_rows, extra = S.run_sitl(spec, "sweep_test", 0, DEFAULTS)
            res = S.score(spec, got_rows, extra, run.policy)
            # the headless verdict (a known failure since 2026-11-02)
            want = S.episode_verdict(spec, S.score(spec, run.rows, run.extra, run.policy,
                                                   run.priorities), {}, 0)[0]
            assert S.episode_verdict(spec, res, {}, 0)[0] == want, res.get("breaches")
            stale["on"] = True
            try:
                S.run_sitl(spec, "sweep_test", 0, DEFAULTS)
            except RuntimeError as e:
                assert "predates this launch" in str(e), e
            else:
                raise AssertionError("an earlier run's metrics.json was scored")
        finally:
            S.ROOT, S.sitl_available, S.build_sitl_cmd, S.subprocess.run = saved


# --------------------------------------------------------------------------- #
# the control arm can still fail
# --------------------------------------------------------------------------- #

def test_the_unshielded_control_arm_scores_a_real_escape():
    """If an unguarded flight into a no-fly zone scores clean, every other pass
    in the sweep is worthless."""
    from guardrail import kpi as K
    sc = BY_ID["nfz-head-on-control"]
    rows, extra = _run("nfz-head-on-control")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc.policy.path)), {})
    assert res["p0_violation_escape_rate"] > 0.0, res
    assert res["time_to_safe_episodes"] >= 1, res
    assert res["p0_ticks_not_measurable"] == 0, "the control arm must be MEASURED"
    assert extra["breaches"] >= 1, "it flew itself into the zone"


def test_the_shielded_arm_of_the_same_flight_is_clean():
    from guardrail import kpi as K
    sc = BY_ID["nfz-head-on"]
    rows, extra = _run("nfz-head-on")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc.policy.path)), {})
    assert res["p0_violation_escape_rate"] == 0.0, res
    assert res["time_to_safe_episodes"] == 0, "the shielded arm was never unsafe"
    assert extra["breaches"] == 0


def test_a_nonfinite_command_never_reaches_the_vehicle():
    rows, extra = _run("nonfinite-action")
    assert extra["all_actions_finite"] is True


def test_the_speed_cap_holds_over_the_whole_flight():
    _, extra = _run("speed-cap")
    assert extra["max_speed_flown"] <= 4.001, extra


def test_the_corridor_run_stays_inside_its_half_width():
    _, extra = _run("corridor-drift-out")
    assert extra["max_offset_from_corridor"] <= 20.001, extra


def test_a_clean_corridor_run_is_not_touched_at_all():
    rows, extra = _run("corridor-along")
    assert sum(len(r["repairs"]) for r in rows) == 0, "a legal flight was repaired"
    assert extra["reached_goal"] is True


def test_the_curfew_changes_the_outcome_of_the_same_flight():
    """In hours the fence binds; out of hours it is not in force at all."""
    _, in_hours = _run("corridor-curfew-in-hours")
    _, off_hours = _run("corridor-curfew-out-of-hours")
    assert off_hours["reached_goal"] is True
    assert in_hours["final"] != off_hours["final"], (
        "the schedule made no difference - the gating is not reaching the run")


# --------------------------------------------------------------------------- #
# the migration changed nothing
# --------------------------------------------------------------------------- #

# Produced by the PRE-schema sweep (git HEAD 1d09786, plain-dict loader) on
# 2026-10-06: where each of the 13 original scenarios ends and how many repairs
# it took. If this fails after a guardrail/shield.py change, the Shield moved,
# not the schema - read the diff before editing the numbers.
# Repair counts re-pinned 2026-10-12: SpeedClamp had logged a "repair" of a
# float-rounding excess (4.000000000000001 -> 4.0) on Windows and not on
# Linux, so these three counts held here and failed on GitHub. With the
# Shield's 1e-9 tolerance in the clamp they are the same on both: nfz-head-on
# 346 -> 264, corridor-curfew-in-hours 412 -> 300, standoff-reclassified
# 360 -> 300, every final position unchanged.
LEGACY = {
    "nfz-head-on": ((-5.6, 15.0, 15.0), 264),
    "nfz-head-on-control": ((37.2, 15.0, 15.0), 52),
    "altitude-floor-recovery": ((60.0, -30.0, 10.01), 73),
    "altitude-ceiling-hold": ((30.0, -30.0, 20.0), 300),
    "speed-cap": ((80.0, -40.0, 15.0), 300),
    "nonfinite-action": ((-40.0, 20.0, 15.0), 300),
    "corridor-along": ((52.4, 0.0, 15.0), 0),
    "corridor-drift-out": ((30.0, 8.4, 15.0), 279),
    "corridor-curfew-in-hours": ((60.0, 0.0, 15.0), 300),
    "corridor-curfew-out-of-hours": ((60.0, 52.4, 15.0), 0),
    "standoff-approach": ((37.2, 0.0, 15.0), 0),
    "standoff-reclassified": ((22.0, -4.03, 15.0), 300),
    "standoff-wedge": ((0.0, 0.0, 15.0), 300),
}


# The same 13 at the grant's forecast (5 s at 0.1 s), FSM off, measured
# 2026-10-12 when the harness moved to it. What moved and why: nfz-head-on is
# held 8 m further from the zone (the forecast reaches 20 m ahead at 4 m/s,
# not 12: -5.6 -> -13.6) with 284 repairs (264); its control arm's
# counterfactual repairs 52 -> 72 (a longer forecast flags more ticks);
# altitude-floor-recovery climbs more gently (AltitudeFix sizes the climb to
# one lookahead) and ends at 10.0 m with 103 repairs (10.01 m, 73);
# corridor-drift-out is stopped 0.4 m off the centreline instead of 8.4 m
# (the drift is stopped when the 5 s forecast reaches the wall), 299
# repairs (279); standoff-reclassified ends 0.01 m further (-4.04). The other
# eight end exactly where they did.
LEGACY_GRANT = {
    "nfz-head-on": ((-13.6, 15.0, 15.0), 284),
    "nfz-head-on-control": ((37.2, 15.0, 15.0), 72),
    "altitude-floor-recovery": ((60.0, -30.0, 10.0), 103),
    "altitude-ceiling-hold": ((30.0, -30.0, 20.0), 300),
    "speed-cap": ((80.0, -40.0, 15.0), 300),
    "nonfinite-action": ((-40.0, 20.0, 15.0), 300),
    "corridor-along": ((52.4, 0.0, 15.0), 0),
    "corridor-drift-out": ((30.0, 0.4, 15.0), 299),
    "corridor-curfew-in-hours": ((60.0, 0.0, 15.0), 300),
    "corridor-curfew-out-of-hours": ((60.0, 52.4, 15.0), 0),
    "standoff-approach": ((37.2, 0.0, 15.0), 0),
    "standoff-reclassified": ((22.0, -4.04, 15.0), 300),
    "standoff-wedge": ((0.0, 0.0, 15.0), 300),
}


def _moved(pins: dict, forecast: str) -> list[str]:
    # The pins are the old flights, so on the old recovery law: since
    # 2026-10-29 the default recovers at the climb cap (WP3-17), which moves
    # altitude-floor-recovery.
    moved = []
    for sid, (final, reps) in pins.items():
        run = S.run_episode(_with(BY_ID[sid], escalation=False,
                                  ticks=LEGACY_TICKS.get(sid, BY_ID[sid].ticks)),
                            DEFAULTS, 0, forecast=forecast, recovery="lookahead")
        got = (run.extra["final"]["x"], run.extra["final"]["y"], run.extra["final"]["up"])
        n = sum(len(r["repairs"]) for r in run.rows)
        if got != final or n != reps:
            moved.append(f"{sid}: final {got} (was {final}), repairs {n} (was {reps})")
    return moved


def test_the_13_original_scenarios_fly_exactly_as_before_the_schema():
    """Migration must be a change of FORM. Rows and extras were compared one to
    one against the old code before this commit; this pins the end state.
    With the escalation FSM OFF (2026-10-07): the split into one file per
    template, the events, the arms and the new measurements must not move a
    single flight of the Shield's own action. Since 2026-10-12 LEGACY is
    flown on the 3 s comparison arm, the forecast it was measured with: the
    arm must reproduce the old flights exactly (it is what makes the arm a
    comparison), and LEGACY_GRANT pins the same 13 at the grant's forecast."""
    moved = _moved(LEGACY, "3s")
    assert not moved, "the 3 s arm is not the old forecast:\n      " + "\n      ".join(moved)
    moved = _moved(LEGACY_GRANT, "grant")
    assert not moved, "\n      ".join(moved)


# The same 13 with the escalation FSM ON: which edges the FSM took. Measured
# 2026-10-07; two re-measured 2026-10-11 after X6 gained its progress test
# (2026-10-09). Until then X6 sent two working recoveries to RTL once the
# position had been illegal for T = 5 s (G1>X6>G9: the floor recovery 0.55 m
# short when it fired, the re-labelled stand-off 0.15 m). X6 now leaves a
# recovery alone while its illegal depth is shown to fall: the floor recovery
# is G1>G7 (back in the band, mission completes) and the re-labelled stand-off
# G1>G3 (out of the ring, then LOITER). G3 on the ceiling hold is an
# AltitudeFix of 4 m against the grant's 0.5 m vertical theta, so it is handed
# to LOITER at 0.1 s.
LEGACY_FSM = {
    "nfz-head-on": "G1", "nfz-head-on-control": "",
    "altitude-floor-recovery": "G1>G7", "altitude-ceiling-hold": "X1>G3",
    "speed-cap": "G1", "nonfinite-action": "G1", "corridor-along": "",
    "corridor-drift-out": "G1", "corridor-curfew-in-hours": "G1",
    "corridor-curfew-out-of-hours": "", "standoff-approach": "",
    "standoff-reclassified": "G1>G3", "standoff-wedge": "G1",
}


# ... and at the grant's forecast (measured 2026-10-12): two move from G1 to
# X1>G3. Theta reads each operator's magnitude_m, the depth of the forecast
# it corrected; a 5 s forecast runs 20 m ahead at 4 m/s, so the curfew fence
# and the wedged stand-off are first met more than 2.0 m deep and the FSM
# abandons projection on the first tick (X1), then hands the hold to LOITER
# (G3). Neither reaches a fail-safe. Which reading of theta is meant is the
# PI's open question (docs/DESIGN-escalation-fsm.md).
LEGACY_FSM_GRANT = dict(LEGACY_FSM, **{"corridor-curfew-in-hours": "X1>G3",
                                       "standoff-wedge": "X1>G3"})


def test_the_13_original_scenarios_under_the_fsm_take_the_edges_measured():
    """Pinned so a change in the FSM or in the harness's hand-over cannot move
    these silently. A pinned edge that changes is a finding to re-measure,
    not a number to edit. LEGACY_FSM on the 3 s comparison arm (it was
    measured there), LEGACY_FSM_GRANT at the grant's forecast. Flown at
    their own lengths (LEGACY_TICKS); at the library's 50 s since 2026-11-02
    the four that end in Loiter there go on to the timeout (G5, G9), which
    the sweep itself scores."""
    moved = []
    for pins, fc in ((LEGACY_FSM, "3s"), (LEGACY_FSM_GRANT, "grant")):
        for sid, chain in pins.items():
            spec = (_with(BY_ID[sid], ticks=LEGACY_TICKS[sid]) if sid in LEGACY_TICKS
                    else BY_ID[sid])
            run = S.run_episode(spec, DEFAULTS, 0, forecast=fc)
            got = ">".join(t["edge"] for t in ((run.extra.get("fsm") or {})
                                                .get("transitions") or []))
            if got != chain:
                moved.append(f"{fc} {sid}: {got!r} (was {chain!r})")
    assert not moved, "\n      ".join(moved)


def test_seeds_change_only_episodes_that_draw_from_them():
    """A deterministic scenario on another seed is the same flight; a noisy one
    is not. The sweep reports which is which instead of calling repeats spread."""
    a, _ = _run("nfz-head-on", seed=1001)
    b, _ = _run("nfz-head-on", seed=1002)
    assert a == b
    spec = BY_ID["gps-noise-within-margin"]
    r1, _ = S.run_headless(spec, DEFAULTS, 1001)
    r1b, _ = S.run_headless(spec, DEFAULTS, 1001)
    r2, _ = S.run_headless(spec, DEFAULTS, 1002)
    assert r1 == r1b, "the same seed must reproduce the same flight"
    assert r1 != r2, "a different seed drew the same noise"
    assert spec.stress.consumes_seed and not BY_ID["nfz-head-on"].stress.consumes_seed


# --------------------------------------------------------------------------- #
# re-labelling the subject: the answer to the 2026-09-02 review
# --------------------------------------------------------------------------- #

def test_a_scenario_without_reclassify_behaves_exactly_as_before():
    """The schedule is optional and must change nothing when absent."""
    plain = BY_ID["standoff-approach"]
    assert not plain.mission.subject.reclassify
    rows, extra = S.run_headless(plain, DEFAULTS)
    ref_rows, ref_extra = _run("standoff-approach")
    assert extra["min_range_to_subject"] == ref_extra["min_range_to_subject"]
    assert extra["range_at_reclassify"] is None
    assert extra["subject_class_final"] == plain.mission.subject.class_


def test_the_enforced_ring_changes_with_the_label_and_nothing_else():
    """One object, one policy, one word changed - and the ring moves 5 m to 10 m."""
    rows, extra = _run("standoff-reclassified")
    assert extra["min_range_as_car"] < 10.0, (
        f"never got inside the 10 m ring while it was legal to: {extra}")
    assert extra["range_at_reclassify"] < 10.0, (
        "the label flipped while the aircraft was already clear, so the new "
        "rule never had to do anything - the scenario proves nothing")
    assert extra["final_range_to_subject"] >= 9.99, (
        f"did not recover to the pedestrian ring: {extra}")
    assert extra["subject_class_final"] == "pedestrian"


def test_the_recovery_is_the_shield_not_the_mission_wandering_off():
    from guardrail import kpi as K
    sc = BY_ID["standoff-reclassified"]
    rows, extra = _run("standoff-reclassified")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc.policy.path)), {})
    assert res["repair_count"] > 0, "the Shield did nothing"
    assert res["p0_violation_escape_rate"] == 0.0, (
        f"an illegal action flew during the recovery: {res}")


def test_removing_the_reclassification_makes_the_scenario_fail():
    """The negative control. A scenario that cannot fail is not a test."""
    sc = BY_ID["standoff-reclassified"]
    broken = _with(sc, **{"mission.subject.reclassify": []})
    rows, extra = S.run_headless(broken, DEFAULTS)
    res = _kpi(sc, rows, extra)
    bad = S.check_gates(sc.gates, res)
    assert len(bad) >= 3, f"the gates barely noticed: {bad}"


def test_a_relabel_schedule_is_applied_in_time_order_not_file_order():
    # FSM off: with it on, X6 ends this flight in RTL at 15 s, before the
    # 20 s relabel (the scenario's known failure), and the schedule is the
    # thing under test here.
    sc = _with(BY_ID["standoff-reclassified"], escalation=False,
               **{"mission.subject.reclassify": [
                   {"at_s": 20.0, "class": "car"}, {"at_s": 10.0, "class": "pedestrian"}]})
    _, extra = S.run_headless(sc, DEFAULTS)
    assert extra["subject_class_final"] == "car", (
        "the later entry must win at the end regardless of file order")


# --------------------------------------------------------------------------- #
# the wedge, kept visible
# --------------------------------------------------------------------------- #

def test_the_wedge_scenario_is_still_the_known_failure_it_claims_to_be():
    """Pinning an open defect so it cannot quietly change in either direction."""
    sc = BY_ID["standoff-wedge"]
    assert sc.expect == "known_failure"
    rows, extra = _run("standoff-wedge")
    assert extra["reached_goal"] is False, (
        "standoff-wedge now reaches its goal - the defect is fixed, so remove "
        "`expect: known_failure` from experiments/scenarios.yaml")
    assert extra["min_range_to_subject"] >= 9.99, (
        f"the stand-off ring broke: {extra['min_range_to_subject']}")


def test_the_non_wedged_standoff_still_completes():
    _, extra = _run("standoff-approach")
    assert extra["min_range_to_subject"] >= 9.99, extra
    assert extra["reached_goal"] is True, extra


# --------------------------------------------------------------------------- #
# the altitude defect the sweep found
# --------------------------------------------------------------------------- #

def test_a_recovery_from_below_the_floor_actually_enters_the_band():
    """REGRESSION, found by the sweep on its first run (AltitudeFix aimed at the
    floor exactly and converged on it from below for ever: 9.99973 m after
    30 s, still unsafe). Since 2026-10-29 the recovery climbs at the 2 m/s
    cap (WP3-17): with this exact integrator it arrives at 9.999999999999996
    m in 3.5 s, inside the band to the monitors' 1e-9 m and legal, where the
    climb stops; under the old law it crossed at 0.3 m/s. So "entered" is
    the Shield's own band, and the recovery must be done by 4 s."""
    sh = Shield(load_policy(ROOT / "policies" / "sim_demo_policy.yaml"),
                lookahead_s=3.0, dt=0.5)
    st = State(x=-30, y=-30, up=3.0)
    t_in = None
    for k in range(300):
        d = sh.filter(st, Action4D(vx=3.0))
        st = State(x=st.x + d.emitted.vx * 0.1, y=st.y,
                   up=st.up + d.emitted.vz_up * 0.1)
        if t_in is None and not sh.state_is_unsafe(st):
            t_in = 0.1 * (k + 1)
    assert st.up >= 10.0 - 1e-9, f"never entered the band: settled at {st.up:.5f} m"
    assert not sh.state_is_unsafe(st), "still an unsafe position at the end"
    assert t_in is not None and t_in <= 4.0, f"inside the band only after {t_in} s"


def test_the_recovery_is_measured_as_a_finite_time_to_safe():
    from guardrail import kpi as K
    sc = BY_ID["altitude-floor-recovery"]
    rows, extra = _run("altitude-floor-recovery")
    res = K.compute(rows, K.rule_priorities(load_policy(ROOT / sc.policy.path)), {})
    assert res["time_to_safe_episodes"] == 1, res
    assert res["time_to_safe_censored"] == 0, (
        "the recovery never completed - this is the pre-fix behaviour")
    assert 0.0 < res["mean_time_to_safe_s"] < 30.0, res
    assert extra["ended_safe"] is True
    assert extra["breaches"] == 0, "starting unsafe is imposed, not a breach"


def test_a_legal_cruise_near_the_band_edge_is_not_dragged_inward():
    sh = Shield(load_policy(ROOT / "policies" / "sim_demo_policy.yaml"),
                lookahead_s=3.0, dt=0.5)
    d = sh.filter(State(x=-30, y=-30, up=10.2), Action4D(vx=3.0))
    assert not d.repairs, f"a legal cruise was repaired: {d.repairs}"
    assert d.emitted.vz_up == 0.0, d.emitted


# --------------------------------------------------------------------------- #
# stress: hot-applied no-fly zones
# --------------------------------------------------------------------------- #

def test_a_spawned_zone_bumps_the_generation_and_lands_where_it_says():
    """Shield.hot_apply: generation 0 -> 1, a new policy_hash, and the near edge
    `ahead_m` in front of the aircraft on its heading at the spawn tick. Since
    2026-10-07 the spawn is a dynamic_nfz (a polygon_fence is locked at
    mission start: this test raised LockedRuleClass on the old harness)."""
    spec = _cell("dyn-nfz-spawn-ahead", vehicle_speed_mps=4, ahead_m=12, variant=0)
    run = S.run_episode(spec, DEFAULTS, 0)
    hot = [e for e in run.events if e["type"] == "hot_apply"]
    assert len(hot) == 1 and hot[0]["generation"] == 1, hot
    assert hot[0]["op"] == "spawn", hot[0]
    assert hot[0]["policy_hash"] != run.policy_hash_start
    assert run.extra["policy_generation_final"] == 1
    assert [c.type for c in run.policy.constraints if c.id == "dyn-nfz"] == ["dynamic_nfz"]
    from shapely.geometry import Point, Polygon
    at = [r for r in run.rows if abs(r["t"] - hot[0]["t"]) < 1e-9][0]
    d = Polygon(hot[0]["vertices"]).exterior.distance(Point(at["x"], at["y"]))
    assert abs(d - 12.0) < 0.6, f"near edge {d:.2f} m ahead, asked for 12"


def test_a_zone_spawned_on_top_puts_the_aircraft_inside_and_it_gets_out():
    spec = _cell("dyn-nfz-spawn-on-top", size_m=40, variant=0)
    run = S.run_episode(spec, DEFAULTS, 0)
    hot = [e for e in run.events if e["type"] == "hot_apply"][0]
    at = [r for r in run.rows if abs(r["t"] - hot["t"]) < 1e-9][0]
    assert at["unsafe"], "the spawn did not land on the aircraft"
    assert run.extra["breaches"] == 0, "an imposed entry was counted as a breach"
    res = _kpi(spec, run.rows, run.extra, run.policy)
    assert res["time_to_safe_episodes"] >= 1 and res["time_to_safe_censored"] == 0


def test_an_event_the_shield_refuses_errors_the_episode_unless_expected():
    """An event the Shield refuses (here: translating a policy polygon_fence,
    which is locked at mission start) is an error, never a silently skipped
    event - unless the scenario says `expect_refused: true`, when the refusal
    is recorded and an ACCEPTED event fails `event_refusals_as_expected`."""
    base = BY_ID["nfz-head-on"]
    bad = _with(base, events=[{"at_sim_t": 2.0, "type": "translate_polygon_fence",
                               "payload": {"id": "nfz-square", "dx_m": 5.0}}])
    try:
        S.run_episode(bad, DEFAULTS, 0)
    except ValueError as e:
        assert "nfz-square" in str(e) and "dynamic_nfz" in str(e), e
    else:
        raise AssertionError("a refused event ran as if it had happened")
    want = _with(base, events=[{"at_sim_t": 2.0, "type": "deactivate_rule",
                                "payload": {"rule_id": "no-such-rule",
                                            "expect_refused": True}}])
    run = S.run_episode(want, DEFAULTS, 0)
    hot = [e for e in run.events if e["type"] == "hot_apply"][0]
    assert hot["refused"] and "no-such-rule" in hot["reason"], hot
    assert run.extra["event_refusals_as_expected"] is True
    assert run.extra["policy_generation_final"] == 0, "a refused event changed the rules"
    wrong = _with(base, events=[{"at_sim_t": 2.0, "type": "deactivate_rule",
                                 "payload": {"rule_id": "nfz-square",
                                             "expect_refused": True}}])
    assert S.run_episode(wrong, DEFAULTS, 0).extra["event_refusals_as_expected"] is False


def test_an_event_after_the_run_ends_is_refused():
    spec = _with(BY_ID["dyn-nfz-spawn-ahead"], **{"events": [
        {"at_sim_t": 999.0, "type": "spawn_polygon_fence",
         "payload": {"width_m": 10, "height_m": 10, "ahead_of_vehicle_m": 5}}]})
    try:
        S.run_episode(spec, DEFAULTS, 0)
    except ValueError as e:
        assert "never fires" in str(e)
        return
    raise AssertionError("an event that can never fire was accepted")


# --------------------------------------------------------------------------- #
# stress: the time-window switch instant
# --------------------------------------------------------------------------- #

def test_the_window_closing_mid_run_is_seen_and_splits_the_flight():
    """REGRESSION (shield-enforcement's needs_outside, 2026-10-07). The switch
    instant is read from the SHIELD (rule_status in_force, both sides of the
    instant), no longer from models.py's minute-resolution active_at, which
    held a 17:30 window until 17:30:59: the old harness, on this 17:29:50
    clock, saw the window close at 70 s - never, in a 45 s run. The Shield
    closes it at 17:30:00 plus its half-step band. Re-pinned 2026-10-12: at
    the grant's 0.1 s forecast step the band is 0.05 s, inside one 0.1 s
    tick, so the switch lands on the edge's own tick, 10.1 s (switch_lag
    0.0, still not early); at the 0.5 s step it was 0.25 s and the switch
    landed at 10.3 s (lag +0.3 s), which the 3 s comparison arm still flies."""
    spec = _cell("time-window-closes-mid-run",
                 clock="2026-09-07T17:29:50", vehicle_speed_mps=4)
    run = S.run_episode(spec, DEFAULTS, 0)
    sw = [e for e in run.events if e["type"] == "time_window_switch"]
    assert len(sw) == 1 and sw[0]["deactivated"] == ["nfz-school"], sw
    assert abs(sw[0]["t"] - 10.1) < 1e-6, sw[0]["t"]
    assert "rule_status" in sw[0]["read_from"]
    assert run.extra["switch_lag_s_nfz-school"] >= 0, "a closing window switched early"
    old = S.run_episode(spec, DEFAULTS, 0, forecast="3s")
    assert [e["t"] for e in old.events if e["type"] == "time_window_switch"] == [10.3]
    assert run.extra["switch_conservative_nfz-school"] is True
    assert run.extra["phase0_violation_ticks_nfz-school"] > 0
    assert run.extra["phase1_violation_ticks_nfz-school"] == 0
    assert run.extra["polygon_entries"] == 0 and run.extra["reached_goal"] is True


def test_an_opening_window_is_enforced_before_it_is_in_force():
    """The t+ side of the grant's boundary check: each forecast pose is judged
    against the rules in force at ITS time, so a window opening inside the
    lookahead binds before the edge. Here the aircraft would reach the school
    zone's ring 0.5 s before the curfew opens; it must be held out and never
    be inside, with ticks enforced before the switch counted."""
    spec = _cell("time-window-opens-ahead", clock="2026-09-07T07:29:52")
    run = S.run_episode(spec, DEFAULTS, 0)
    ex = run.extra
    assert ex["anticipated_ticks_nfz-school"] >= 1, ex["anticipated_ticks_nfz-school"]
    assert ex["ticks_inside_fence_polygon"] == 0 and ex["breaches"] == 0, ex
    assert ex["switch_lag_s_nfz-school"] <= 0, "an opening window was enforced late"
    # Negative control: an instant-only reading (the forecast's later poses
    # judged against the rules NOW) is what lets it in. Emulated by a clock
    # that never reaches the edge: nothing is anticipated, nothing binds.
    early = _with(spec, **{"mission.clock_start": "2026-09-07T06:00:00"})
    e2 = S.run_episode(early, DEFAULTS, 0).extra
    assert e2["anticipated_ticks_nfz-school"] == 0, e2["anticipated_ticks_nfz-school"]


def test_with_no_switch_the_phase_gate_cannot_pass():
    """Negative control: the same flight on a clock that never crosses the edge
    has no second phase, so the gate on it is "not measured", never a pass."""
    spec = _cell("time-window-closes-mid-run",
                 clock="2026-09-07T17:29:50", vehicle_speed_mps=4)
    still = _with(spec, **{"mission.clock_start": "2026-09-07T09:00:00"})
    run = S.run_episode(still, DEFAULTS, 0)
    assert not [e for e in run.events if e["type"] == "time_window_switch"]
    bad = S.check_gates(spec.gates, _kpi(still, run.rows, run.extra, run.policy))
    assert any("phase1_violation_ticks_nfz-school: not measured" in b for b in bad), bad


# --------------------------------------------------------------------------- #
# stress: GPS noise, latency, wind
# --------------------------------------------------------------------------- #

def test_gps_noise_reaches_the_shield_and_not_the_vehicle():
    """Latency 0, so ONLY the noise can make the perceived pose differ (the
    first version used a latency-5 cell, which passed with the noise removed:
    the stale pose alone differs from the true one)."""
    spec = _with(_cell("gps-noise-beyond-margin", pass_y=23.6, sigma=2.0, latency=5),
                 **{"stress.latency_ticks": 0, "escalation": False})
    run = S.run_episode(spec, DEFAULTS, 1001)
    diffs = [abs(r["perceived"]["x"] - r["x"]) + abs(r["perceived"]["y"] - r["y"])
             for r in run.rows]
    assert max(diffs) > 1.0, "the Shield saw the true position: no noise applied"
    assert sum(d > 0 for d in diffs) >= 0.95 * len(diffs), "noise on too few ticks"
    assert "true_violations" in run.rows[0]
    quiet = _with(spec, **{"stress.gps_noise_m": 0.0, "stress.latency_ticks": 1})
    q = S.run_episode(quiet, DEFAULTS, 1001).rows
    assert all(abs(r["perceived"]["y"] - q[max(0, i - 1)]["y"]) < 1e-12
               for i, r in enumerate(q)), "with no noise the pose must be exact"


def test_the_oracle_never_writes_into_the_flying_shields_window():
    """The ground-truth oracle is a SECOND Shield. Re-using the flying one
    would leave the emitted actions unchanged (history is a record, not an
    input) but would fill the window a rail's audit writer reads with
    true-state queries the flying Shield never made. The window must hold
    exactly the poses that Shield saw, one per tick. (FSM off here: with it on,
    a row's `emitted` is what FLEW - a stop or the autopilot's command - which
    need not be the Shield's emitted action; the FSM checker below covers it.)"""
    spec = _with(_cell("gps-noise-beyond-margin", pass_y=23.6, sigma=2.0, latency=5),
                 **{"stress.latency_ticks": 0, "escalation": False})
    run = S.run_episode(spec, DEFAULTS, 1001)
    win = run.shield_window
    assert len(win) == 50, len(win)
    tail = run.rows[-len(win):]
    for rec, row in zip(win, tail):
        assert (rec.state.x, rec.state.y) == (row["perceived"]["x"], row["perceived"]["y"]), (
            "the flying Shield's window holds a pose it was never shown")
        assert rec.decision.emitted.model_dump() == row["emitted"]
    # With the FSM on, the window still holds one entry per tick, of the
    # poses the flying Shield saw - the checker and the oracle never write it.
    fsm = S.run_episode(_with(spec, escalation=True), DEFAULTS, 1001)
    ftail = fsm.rows[-len(fsm.shield_window):]
    assert all((r.state.x, r.state.y) == (w["perceived"]["x"], w["perceived"]["y"])
               for r, w in zip(fsm.shield_window, ftail))


def test_latency_hands_the_shield_the_state_from_n_ticks_ago():
    spec = _with(_cell("gps-noise-within-margin", pass_y=23.6, sigma=0.2, latency=2),
                 **{"stress.gps_noise_m": 0.0, "stress.latency_ticks": 5})
    rows = S.run_episode(spec, DEFAULTS, 0).rows
    for i in (10, 50, 120):
        assert rows[i]["perceived"]["x"] == rows[i - 5]["x"], i


def test_true_state_violations_are_counted_apart_from_the_shields_own():
    """Under noise the Shield's log can be clean while the flown action was
    illegal where the aircraft really was. Both are reported; neither replaces
    the other."""
    spec = _cell("gps-noise-beyond-margin", pass_y=23.6, sigma=2.0, latency=10)
    run = S.run_episode(spec, DEFAULTS, 1001)
    res = _kpi(spec, run.rows, run.extra, run.policy)
    assert res["p0_escapes"] == 0, "the Shield's own re-check should be clean"
    assert res["true_p0_flown_ticks"] > 0, "noise had no true-state effect at all"


def test_gps_dropout_freezes_the_pose_the_shield_sees_and_not_the_vehicle():
    """GPS denial (audit card X-14): for the window the Shield is handed the
    last fix while the aircraft keeps moving; outside it, the live pose. (FSM
    off: a LOITER would stop the aircraft, which is the FSM's business, not
    the stressor's.) Re-pinned 2026-10-12: the family's windows were re-cut
    for the grant's 5 s forecast (experiments/templates/sensor_degradation.yaml)
    and the long one is now 4-11 s, 70 ticks."""
    spec = _with(_cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0]), escalation=False)
    run = S.run_episode(spec, DEFAULTS, 0)
    inside = [r for r in run.rows if 4.0 - 1e-9 <= r["t"] < 11.0 - 1e-9]
    assert len(inside) == 70 == run.extra["gps_dropout_ticks"], run.extra["gps_dropout_ticks"]
    held = {(r["perceived"]["x"], r["perceived"]["y"]) for r in inside}
    assert len(held) == 1, "the pose the Shield saw moved during the dropout"
    assert inside[-1]["x"] - inside[0]["x"] > 5.0, "the vehicle stopped too: nothing was stale"
    after = [r for r in run.rows if r["t"] >= 11.0 - 1e-9]
    assert all(r["perceived"]["x"] == r["x"] for r in after), "the fix never came back"
    kinds = [e["type"] for e in run.events]
    assert kinds.count("gps_dropout_start") == 1 and kinds.count("gps_dropout_end") == 1


def test_gps_dropout_on_the_head_on_lane_is_a_measured_polygon_entry():
    """The finding the family is TRACKED for, pinned so it cannot change
    silently: the Shield trusts a frozen pose and lets the aircraft into the
    zone, while its own re-check stays clean (it repaired against the pose it
    was given). If this stops holding, the description in scenarios.yaml is
    wrong and must be re-measured. Re-measured 2026-10-12 at the grant's 5 s
    forecast: the entry is the 4-11 s window's (frozen still at speed, 24 m
    out); the old 6-11 s window freezes a pose the Shield already holds."""
    spec = _cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0])
    run = S.run_episode(spec, DEFAULTS, 0)
    res = _kpi(spec, run.rows, run.extra, run.policy)
    assert res["p0_escapes"] == 0, "the Shield's own re-check should be clean"
    assert run.extra["ticks_inside_fence_polygon"] > 0
    assert run.extra["min_true_dist_to_fence_m"] < -5.0, run.extra["min_true_dist_to_fence_m"]
    short = _cell("gps-dropout", lane_y=15.0, window=[2.0, 4.0])
    e2 = S.run_episode(short, DEFAULTS, 0).extra
    assert e2["ticks_inside_fence_polygon"] == 0 and e2["breaches"] == 0, e2


def test_a_dropout_that_froze_nothing_fails_its_gate_and_one_after_the_run_is_refused():
    """Silence reads as success: with no frozen tick, the family's
    `gps_dropout_ticks` gate must fail rather than score a clean run."""
    spec = _cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0])
    assert S.check_gates(spec.gates, {"gps_dropout_ticks": 0,
                                      "p0_violation_escape_rate": 0,
                                      "all_actions_finite": True})
    plain = _kpi(BY_ID["nfz-head-on"], *_run("nfz-head-on"))
    assert plain["gps_dropout_ticks"] is None, "no dropout declared must read None"
    late = _with(spec, **{"stress.gps_dropout_s": [99.0, 120.0]})
    try:
        S.run_episode(late, DEFAULTS, 0)
    except ValueError as e:
        assert "never happens" in str(e)
    else:
        raise AssertionError("a dropout after the last tick was accepted")


def test_wind_moves_the_aircraft_and_the_shield_never_sees_it():
    """The Shield never sees the wind (it predicts with the commanded action
    only). Re-pinned 2026-10-12: on the open-loop model the harness flew until
    then, a 2 m/s wind carries the aircraft 60 m in 30 s; on the closed-loop
    default (ArduCopter's position controller, tests/test_harnesstruth_wind.py)
    the same wind is rejected and the aircraft ends where the calm flight
    does. Either way the Shield emits exactly what it emits in still air."""
    still = BY_ID["speed-cap"]
    windy = _with(still, **{"stress.wind_speed_mps": 2.0, "stress.wind_dir_deg": 90.0})
    r0, e0 = S.run_headless(still, DEFAULTS)
    run = S.run_episode(windy, DEFAULTS, 0, wind_model="open_loop")
    r1, e1 = run.rows, run.extra
    assert [r["emitted"] for r in r0] == [r["emitted"] for r in r1], (
        "the Shield reacted to wind it cannot see")
    assert abs((e1["final"]["y"] - e0["final"]["y"]) - 2.0 * 30.0) < 0.5, (e0, e1)
    r2, e2 = S.run_headless(windy, DEFAULTS)
    assert [r["emitted"] for r in r0] == [r["emitted"] for r in r2]
    assert e2["final"] == e0["final"], (e0["final"], e2["final"])


def test_a_declared_wind_direction_turns_with_the_mission():
    """wind_dir_deg is in the MISSION's frame: a family flown from another side
    keeps "tailwind into the zone" a tailwind. Rotating the mission 90 deg must
    rotate the wind vector 90 deg; a drawn direction is absolute instead."""
    base = _with(BY_ID["nfz-head-on"], **{"stress.wind_speed_mps": 5.0,
                                          "stress.wind_dir_deg": 0.0})
    w0 = S.run_episode(base, DEFAULTS, 0).events[0]["wind_mps"]
    turned = _with(base, **{"mission.rotate_deg": 90.0})
    w90 = S.run_episode(turned, DEFAULTS, 0).events[0]["wind_mps"]
    assert abs(w0[0] - 5.0) < 1e-6 and abs(w0[1]) < 1e-6, w0
    assert abs(w90[0]) < 1e-6 and abs(w90[1] - 5.0) < 1e-6, (
        f"wind did not turn with the mission: {w90}")


def test_gusts_are_a_velocity_disturbance_the_shield_never_commands():
    """REGRESSION (review 2026-10-06: replacing the gust draw with 0 passed
    every test and the smoke run). The per-tick residual between where the
    aircraft went and what was commanded plus the steady wind IS the gust: a
    0.5 m/s sigma must show up there, and nowhere when gust_mps is 0. On the
    open-loop model (the default until 2026-10-12): on the closed loop the
    wind is rejected and the gust pulled back, which
    tests/test_harnesstruth_wind.py measures."""
    spec = _with(_cell("wind-gusts", lane_y=15.0, wind_speed_mps=2), escalation=False)
    assert spec.stress.gust_mps == 0.5

    def residuals(s):
        run = S.run_episode(s, DEFAULTS, 1001, wind_model="open_loop")
        w = run.events[0]["wind_mps"]
        dt = DEFAULTS.dt
        rs = run.rows
        return [((rs[i + 1]["x"] - rs[i]["x"]) / dt - rs[i]["emitted"]["vx"] - w[0],
                 (rs[i + 1]["y"] - rs[i]["y"]) / dt - rs[i]["emitted"]["vy"] - w[1])
                for i in range(len(rs) - 1)]

    gusty = residuals(spec)
    sx = statistics.pstdev([r[0] for r in gusty])
    sy = statistics.pstdev([r[1] for r in gusty])
    assert 0.4 < sx < 0.6 and 0.4 < sy < 0.6, (sx, sy)
    calm = residuals(_with(spec, **{"stress.gust_mps": 0.0}))
    assert max(abs(v) for r in calm for v in r) < 0.01, "a residual with no gust"


def test_start_jitter_draws_a_different_start_per_seed_within_its_bound():
    spec = _cell("wind-gusts", lane_y=15.0, wind_speed_mps=2)
    assert spec.stress.start_jitter_m == 1.0
    a = S.run_episode(spec, DEFAULTS, 1001).rows[0]
    b = S.run_episode(spec, DEFAULTS, 1002).rows[0]
    assert (a["x"], a["y"]) != (b["x"], b["y"]), "two seeds, one start"
    for r in (a, b):
        assert abs(r["x"] + 30.0) <= 1.0 and abs(r["y"] - 15.0) <= 1.0, r


def test_reached_goal_goes_into_compute_so_a_goal_less_success_fails():
    """REGRESSION (WP4-08; review 2026-10-06: dropping the pass-through passed
    every test). nfz-head-on is held at the zone edge and never arrives; with
    `reached_goal` in the metrics compute() scores that as a failed mission."""
    spec = BY_ID["nfz-head-on"]
    rows, extra = _run("nfz-head-on")
    assert extra["reached_goal"] is False
    res = _kpi(spec, rows, extra)
    assert res["outcome"] == "fail", res["outcome"]
    assert res["mission_success"] is False
    assert res["mission_goal_status"] == "missed"
    ok = _kpi(BY_ID["corridor-along"], *_run("corridor-along"))
    assert ok["outcome"] == "success" and ok["mission_goal_status"] == "reached"


# --------------------------------------------------------------------------- #
# labels, pins, bundles, manifests
# --------------------------------------------------------------------------- #

def test_a_fail_safe_label_that_disagrees_with_the_flight_fails_it():
    spec = _with(BY_ID["speed-cap"], expected_failsafe=True)
    rows, extra = S.run_headless(spec, DEFAULTS)
    res = _kpi(spec, rows, extra)
    assert res["failsafe_triggered"] is False
    assert res["failsafe_matches_label"] == 0.0
    assert any("fail-safe" in b for b in S.label_failures(spec, res))
    ok = _with(spec, expected_failsafe=False)
    assert not S.label_failures(ok, _kpi(ok, rows, extra))


def test_episode_manifest_is_exactly_the_six_fields_and_never_kpi_grade():
    from guardrail.manifest import is_kpi_grade
    base = {"code_revision": "abc123", "vla_model_hash": "m@src:1",
            "topology": S.TOPOLOGY_HEADLESS}
    m = S.episode_manifest(base, "sha256:x", 1001, sim_s=30.0, wall_s=0.1)
    assert tuple(m) == ("code_revision", "vla_model_hash", "policy_hash",
                        "random_seed", "sim_speedup", "topology"), tuple(m)
    assert m["random_seed"] == 1001 and m["sim_speedup"] == 300.0
    grade, why = is_kpi_grade(m, {})
    assert not grade
    assert any("sim_speedup" in w for w in why) and any("topology" in w for w in why)


def _sweep(argv):
    return S.main(argv)


def test_a_sweep_writes_one_bundle_per_episode_and_survives_an_out_outside_the_repo():
    """REGRESSION: `out.relative_to(ROOT)` raised AFTER the results were written
    whenever --out was outside the repository, so a good sweep exited with a
    traceback. The bundle is the grant's: manifest.json (six fields),
    kpi.json, harness_events.jsonl, in an `episode-<UTC>--<id>--seed<n>` folder."""
    with tempfile.TemporaryDirectory() as td:
        out, runs = Path(td) / "s.json", Path(td) / "runs"
        rc = _sweep(["--only", "speed-cap", "--out", str(out), "--runs", str(runs),
                     "--seed", "1001"])
        assert rc == 0 and out.is_file()
        doc = json.loads(out.read_text(encoding="utf-8"))
        assert doc["_counts"]["pass"] == 1, doc["_counts"]
        dirs = [p for p in runs.iterdir() if p.is_dir()]
        assert len(dirs) == 1 and dirs[0].name.endswith("--speed-cap--seed1001"), dirs
        man = json.loads((dirs[0] / "manifest.json").read_text(encoding="utf-8"))
        assert len(man) == 6 and man["random_seed"] == 1001
        assert man["topology"] == S.TOPOLOGY_HEADLESS
        # Re-pinned 2026-10-12: every headless bundle now carries
        # manifest_extras.json, the keys that re-fly the episode
        # (episode_extras); until then only an armed episode had one.
        ex = json.loads((dirs[0] / "manifest_extras.json").read_text(encoding="utf-8"))
        assert ex["extras_schema"] == S.EXTRAS_SCHEMA and ex["random_seed"] == 1001
        assert ex["arm"]["record"] is None and "paraphrase_id" not in ex, "no arm"
        kpi = json.loads((dirs[0] / "kpi.json").read_text(encoding="utf-8"))
        assert kpi["kpi_grade"] is False and kpi["status"] == "pass"
        ev = (dirs[0] / "harness_events.jsonl").read_text(encoding="utf-8").splitlines()
        assert json.loads(ev[0])["type"] == "episode_start"
        assert json.loads(ev[-1])["type"] == "episode_end"
        assert (runs / "kpi_report.md").is_file(), "the run's KPI report is missing"


def test_a_pinned_cell_that_starts_passing_is_reported_fixed_not_quietly_passed():
    """The per-cell `known_failures` marker behaves like the scenario-wide one."""
    fam = copy.deepcopy(LIBRARY.raw("altitude-recovery-depth"))
    fam["known_failures"] = [{"params": {"start_up": 9.5}, "description": "pinned on purpose",
                              "fails": sorted(fam["expected_kpis"])[:1]}]
    fam["parameter_sweep"] = {"start_up": [9.5, 20.5]}
    with tempfile.TemporaryDirectory() as td:
        lib = Path(td) / "lib.yaml"
        lib.write_text(yaml.safe_dump({"defaults": RAW["defaults"], "scenarios": [fam]}),
                       encoding="utf-8")
        out = Path(td) / "s.json"
        rc = _sweep(["--library", str(lib), "--out", str(out), "--no-bundles"])
        doc = json.loads(out.read_text(encoding="utf-8"))
    status = {r["id"]: r["status"] for r in doc["results"]}
    assert status["altitude-recovery-depth@start_up=9.5"] == "unexpected_pass", status
    assert status["altitude-recovery-depth@start_up=20.5"] == "pass", status
    assert rc == 1, "a fixed defect must turn the sweep red until the pin comes off"


# --------------------------------------------------------------------------- #
# main(): statuses, labels, refusals (review 2026-10-06)
# --------------------------------------------------------------------------- #

RAW = yaml.safe_load((ROOT / "experiments" / "scenarios.yaml").read_text(encoding="utf-8"))


def _raw(sid, **changes):
    """A family as written, from whichever template file holds it."""
    fam = copy.deepcopy(LIBRARY.raw(sid))
    fam.update(_relabel_safely(fam, changes))
    return fam


def _sweep_lib(families, *extra):
    """Run main() on a temporary library; (rc, doc or None)."""
    with tempfile.TemporaryDirectory() as td:
        lib = Path(td) / "lib.yaml"
        lib.write_text(yaml.safe_dump({"defaults": RAW["defaults"], "scenarios": families}),
                       encoding="utf-8")
        out = Path(td) / "s.json"
        rc = _sweep(["--library", str(lib), "--out", str(out), "--no-bundles", *extra])
        doc = json.loads(out.read_text(encoding="utf-8")) if out.is_file() else None
    return rc, doc


def test_a_label_the_flight_contradicts_fails_the_scenario_in_main():
    """label_failures must be APPLIED by main(), not only computed: speed-cap
    labelled "fail-safe expected" never brakes, so it fails, and the sweep is
    red."""
    rc, doc = _sweep_lib([_raw("speed-cap", expected_failsafe=True)])
    r = doc["results"][0]
    assert r["status"] == "fail" and rc == 1, (r["status"], rc)
    assert any("fail-safe" in g for g in r["gates_failed"]), r["gates_failed"]
    rc, doc = _sweep_lib([_raw("speed-cap")])
    assert doc["results"][0]["status"] == "pass" and rc == 0


def test_a_whole_scenario_known_failure_is_reported_known_and_does_not_fail_the_run():
    rc, doc = _sweep_lib([_raw("standoff-wedge")])
    assert doc["results"][0]["status"] == "known_failure", doc["results"][0]["status"]
    assert doc["_counts"]["known_failure"] == 1 and rc == 0, (doc["_counts"], rc)
    unmarked = _raw("standoff-wedge")
    del unmarked["expect"], unmarked["known_failure_fails"]
    rc, doc = _sweep_lib([unmarked])
    assert doc["results"][0]["status"] == "fail" and rc == 1, "the marker did nothing"


def _scored_cell(sid, seed=1001, **params):
    """(spec, res, params) of a flown and scored cell of a library family."""
    for c in FAMILIES[sid].cells(samples=3):
        if all(c.params.get(k) == v for k, v in params.items()):
            run = S.run_episode(c.spec, DEFAULTS, seed)
            return c.spec, S.score(c.spec, run.rows, run.extra, run.policy,
                                   run.priorities), c.params
    raise KeyError(f"{sid}: no cell with {params}")


def test_a_new_failure_in_a_pinned_cell_is_a_failure_not_the_old_defect():
    """REVIEW 2026-10-19 (harness-tools/known-failure-pin-absorbs-new-
    regressions). A pinned cell was KNOWN whatever failed in it: the
    three-simultaneous-violations family, pinned for a false fail-safe (its
    two labels), stayed KNOWN with p0_violation_escape_rate 0.25, 3 breaches
    and an unsafe end injected into its real KPI table, and so did the
    dyn-nfz-spawn-ahead and standoff-wedge pins. Each pin now names what it
    fails (`fails`, measured over the nightly profile); anything else that
    fails in the cell is NEW, and the episode fails.

    The three-simultaneous-violations pin came off on 2026-10-19, when its
    cells passed at 300 ticks - only because the run ended 0.2 s before the
    Loiter timeout (G5) - and came back on 2026-10-20 at 400 ticks, where G5
    fires (review of T1c, P1). Its two cells are back among the cases. The
    two label pins of near-edge-at-12mps were further cover until
    2026-11-02, when Loiter began handing back a request the Shield would fly
    (G8) and both cells passed; their pins came off.

    Since 2026-11-02 the fail-safe rubric (docs/RUBRIC-failsafe.md) labels
    three-simultaneous-violations and dyn-nfz-spawn-ahead's theta cells as
    expecting the fail-safe they fly, and both pins came off. The cases are
    the pins the library still carries: the rail mission's false fail-safe
    (a whole-scenario pin on the label), standoff-approach-angles' 7 m
    pedestrian cell (a cell pin on the goal, outcome and label), the
    corridor-swap-shifted RTL that ends outside the swapped corridor (two
    corridor gates) and the standoff wedge (the goal)."""
    cases = (("rail-mission-stub-vla", {}),
             ("standoff-approach-angles", {"offset_m": 7, "subject_class": "pedestrian",
                                           "variant": 0}),
             ("corridor-swap-shifted", {}),
             ("standoff-wedge", {}))
    for sid, params in cases:
        spec, res, cell = _scored_cell(sid, **params)
        status, bad, _, why = S.episode_verdict(spec, res, cell, 1001)
        assert status == "known_failure" and why, (sid, status, bad)
        assert not any(b.startswith(S.NEW_IN_PINNED_CELL) for b in bad), (sid, bad)
        hit = dict(res, p0_violation_escape_rate=0.25)
        status, bad, _, _ = S.episode_verdict(spec, hit, cell, 1001)
        assert status == "fail", (sid, status, bad)
        assert any(b.startswith(S.NEW_IN_PINNED_CELL + "p0_violation_escape_rate")
                   for b in bad), (sid, bad)
        if "breaches" in spec.gates:
            status, bad, _, _ = S.episode_verdict(spec, dict(res, breaches=3), cell, 1001)
            assert status == "fail" and any("breaches=3" in b for b in bad), (sid, bad)
    # A pin that does fail a safety gate keeps it - and only it. (Since C2,
    # 2026-10-21, no library cell is pinned on a safety gate; the
    # opens-on-vehicle cell is re-pinned as it was, with 7c9c9e6's measured
    # escapes put back into its score: _repinned_on_vehicle.)
    spec, res, cell = _repinned_on_vehicle("2026-09-07T07:29:58")
    assert res["p0_violation_escape_rate"] > 0, "the re-pinned P0 failure is gone?"
    status, bad, _, _ = S.episode_verdict(spec, res, cell, 1001)
    assert status == "known_failure", (status, bad)
    status, bad, _, _ = S.episode_verdict(spec, dict(res, breaches=3), cell, 1001)
    assert status == "fail" and bad[0].startswith(S.NEW_IN_PINNED_CELL + "breaches"), bad
    # A pinned cell whose pinned gates all pass is still reported FIXED.
    spec, res, cell = _scored_cell("standoff-wedge")
    fixed = dict(res, reached_goal=True, outcome_matches_label=True,
                 mission_outcome="success")
    assert S.episode_verdict(spec, fixed, cell, 1001)[0] == "unexpected_pass"


def test_a_new_failure_in_a_pinned_cell_turns_the_sweep_red():
    """End to end through main(): the pinned standoff-wedge exits 0 as flown;
    with a P0 escape injected into its score the run exits 1, the episode is
    FAIL and nothing is counted known."""
    fam = _raw("standoff-wedge")
    rc, doc = _sweep_lib([fam])
    assert rc == 0 and doc["results"][0]["status"] == "known_failure", (rc, doc["_counts"])
    real = S.score

    def escaping(spec, *a, **k):
        res = real(spec, *a, **k)
        if spec.scenario_id == "standoff-wedge":
            res["p0_violation_escape_rate"] = 0.25
        return res
    S.score = escaping
    try:
        rc, doc = _sweep_lib([fam])
    finally:
        S.score = real
    r = doc["results"][0]
    assert rc == 1 and r["status"] == "fail", (rc, r["status"], r["gates_failed"])
    assert doc["_counts"]["fail"] == 1 and doc["_counts"]["known_failure"] == 0, doc["_counts"]
    assert r["gates_failed"][0].startswith(S.NEW_IN_PINNED_CELL), r["gates_failed"]


TW_ON_VEHICLE_CLOCKS = ("2026-09-07T07:29:58", "2026-09-07T07:29:58.5",
                        "2026-09-07T07:29:59")
# What 7c9c9e6 measured in those cells on seed 1001, grant arm: 2 Shield P0
# escapes over this many Shield-flown ticks (the rest were the modelled
# autopilot's LOITER). The pins were set from these; C2 (2026-10-21) removed
# the escapes and the pins (0 escapes over 300 Shield ticks).
TW_ON_VEHICLE_OLD_TICKS = {"2026-09-07T07:29:58": 262, "2026-09-07T07:29:58.5": 267,
                           "2026-09-07T07:29:59": 272}
TW_ON_VEHICLE_OLD_PINS = [
    {"params": {"clock": c}, "fails": [{"P0_escape_rate": {"max": 0.01}}],
     "description": "re-pinned as at 7c9c9e6 (tests/test_sweep.py) to exercise "
                    "the bounded pin on the numbers it was set from"}
    for c in TW_ON_VEHICLE_CLOCKS]


def _repinned_family(**changes):
    """time-window-opens-on-vehicle with its 7c9c9e6 pins put back."""
    return _raw("time-window-opens-on-vehicle",
                known_failures=copy.deepcopy(TW_ON_VEHICLE_OLD_PINS), **changes)


def _with_old_escapes(res: dict, clock: str) -> dict:
    """A score with 7c9c9e6's measured escapes in it (2 over the old count of
    Shield ticks), everything else as flown today."""
    n = TW_ON_VEHICLE_OLD_TICKS[clock]
    return dict(res, p0_escapes=2, shield_ticks=n, p0_violation_escape_rate=round(2 / n, 6))


def _repinned_on_vehicle(clock: str, seed: int = 1001):
    """(spec, res, params) of the re-pinned cell at `clock`, flown and scored
    today, with 7c9c9e6's escapes put back into the score."""
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "lib.yaml"
        path.write_text(yaml.safe_dump({"defaults": RAW["defaults"],
                                        "scenarios": [_repinned_family()]}),
                        encoding="utf-8")
        fam = Library(path).by_id()["time-window-opens-on-vehicle"]
        cell = next(c for c in fam.cells(samples=3) if c.params.get("clock") == clock)
    run = S.run_episode(cell.spec, DEFAULTS, seed)
    res = S.score(cell.spec, run.rows, run.extra, run.policy, run.priorities)
    return cell.spec, _with_old_escapes(res, clock), cell.params


def test_a_bounded_pin_absorbs_its_measured_magnitude_and_no_more():
    """REVIEW of F8-harness-tools (P3). A pin naming a safety gate absorbed
    it at ANY magnitude: time-window-opens-on-vehicle was pinned for its P0
    escape rate (2 escapes over 262-272 Shield ticks, 0.0074-0.0076, on the
    grant arm these tests fly; the review of the F8b merge flew the 3 s arm
    at 264-274, 0.0073-0.0076, the YAML's range over both arms), and
    0.5 injected into those cells still printed KNOWN and exited 0 - every
    other pinned P0, breach and ended_safe gate of the nightly profile
    turned its episode red under the same injection. The pins carry their
    bound, `fails: [{P0_escape_rate: {max: 0.01}}]`: the measured rate is
    KNOWN, 0.5 is a NEW failure, and so is a third escape or a rate that was
    not measured. Shown failing before the fix (0.5 was known_failure).

    Since C2 (2026-10-21) those cells fly no escape and carry no pin, and no
    library cell is pinned on a safety gate with a bound, so the mechanism
    is shown on the cell re-pinned as it was, with 7c9c9e6's measured
    escapes put back into today's score (_repinned_on_vehicle) - and the
    re-pinned cell as flown today is FIXED, which is why the pins came
    off."""
    for clock in TW_ON_VEHICLE_CLOCKS:
        spec, res, cell = _repinned_on_vehicle(clock)
        rate = res["p0_violation_escape_rate"]
        assert res["p0_escapes"] == 2 and 0.007 <= rate <= 0.008, (clock, rate, res["p0_escapes"])
        assert S.episode_verdict(spec, res, cell, 1001)[0] == "known_failure", clock
        status, bad, _, why = S.episode_verdict(
            spec, dict(res, p0_violation_escape_rate=0.5), cell, 1001)
        assert status == "fail" and why, (clock, status, bad)
        assert bad == [S.NEW_IN_PINNED_CELL + "p0_violation_escape_rate=0.5 != 0 "
                       "(pinned only within max 0.01)"], bad
        third = round(3 / res["shield_ticks"], 6)
        for worse in (third, None):
            got = S.episode_verdict(spec, dict(res, p0_violation_escape_rate=worse), cell, 1001)
            assert got[0] == "fail" and got[1][0].startswith(S.NEW_IN_PINNED_CELL), (worse, got)
        # inside the bound but past the gate is still the pinned defect
        assert S.episode_verdict(spec, dict(res, p0_violation_escape_rate=0.01),
                                 cell, 1001)[0] == "known_failure"
        # As flown today (0 escapes) every gate passes: the pin is stale.
        flown = dict(res, p0_escapes=0, p0_violation_escape_rate=0.0)
        assert S.episode_verdict(spec, flown, cell, 1001)[0] == "unexpected_pass", clock
    # End to end: the re-pinned smoke cell exits 0 with the old escapes in
    # it, 1 with 0.5, and 1 as flown today (FIXED: remove the marker).
    # (The 4 m/s pilot: the escapes 7c9c9e6 measured were flown at 4 m/s; the
    # family sweeps 1.5 m/s beside it since the review of 3660383.)
    fam = _repinned_family(parameter_sweep={"clock": [TW_ON_VEHICLE_CLOCKS[0]],
                                            "vehicle_speed_mps": [4]})
    real = S.score

    def scored_as(rate):
        def score(spec, *a, **k):
            res = real(spec, *a, **k)
            if spec.scenario_id == "time-window-opens-on-vehicle" and rate is not None:
                res = _with_old_escapes(res, TW_ON_VEHICLE_CLOCKS[0])
                if rate != "old":
                    res["p0_violation_escape_rate"] = rate
            return res
        return score
    try:
        S.score = scored_as("old")
        rc, doc = _sweep_lib([fam], "--seed", "1001")
        assert rc == 0 and doc["_counts"]["known_failure"] == 1, (rc, doc["_counts"])
        S.score = scored_as(0.5)
        rc, doc = _sweep_lib([fam], "--seed", "1001")
        r = doc["results"][0]
        assert rc == 1 and r["status"] == "fail", (rc, r["status"], r["gates_failed"])
        assert doc["_counts"]["known_failure"] == 0, doc["_counts"]
    finally:
        S.score = real
    rc, doc = _sweep_lib([fam], "--seed", "1001")
    assert rc == 1 and doc["results"][0]["status"] == "unexpected_pass", (rc, doc["_counts"])
    # And the library's own cells, unpinned, pass as flown, at both pilot
    # speeds (the 1.5 m/s one against its time-to-safe and ticks-inside
    # bounds, which 3660383 failed: 5.1 s, 26 ticks).
    rc, doc = _sweep_lib([_raw("time-window-opens-on-vehicle",
                               parameter_sweep={"clock": [TW_ON_VEHICLE_CLOCKS[0]],
                                                "vehicle_speed_mps": [4, 1.5]})],
                         "--seed", "1001")
    assert rc == 0 and [r["status"] for r in doc["results"]] == ["pass", "pass"], \
        (rc, doc["_counts"])


def test_a_tracked_episode_is_never_counted_as_a_pass():
    """gps-dropout on the head-on lane passes its gates (the Shield's own log
    is clean) while the aircraft flies 7 m into the zone. It must be reported
    "tracked", counted apart, with the incursion in the true-state summary."""
    fam = _raw("gps-dropout", parameter_sweep={"lane_y": [15.0], "window": [[4.0, 11.0]]})
    rc, doc = _sweep_lib([fam])
    r = doc["results"][0]
    assert r["status"] == "tracked" and rc == 0, (r["status"], rc)
    assert doc["_counts"]["pass"] == 0 and doc["_counts"]["tracked"] == 1, doc["_counts"]
    ts = doc["_true_state"]
    assert ts["episodes_entering_polygon"] == 1, ts
    assert ts["episodes_entering_polygon_by_status"] == {"tracked": 1}, ts
    assert ts["min_true_dist_to_fence_m_of_entries"] < -5.0, ts
    assert doc["_rollup"]["_all_shield_on"]["true_state"] == ts
    del fam["expect"]
    rc, doc = _sweep_lib([fam])
    assert doc["results"][0]["status"] == "pass", "without the marker it hides as a pass"


def test_an_imposed_polygon_is_not_an_entry_and_a_flown_one_is():
    """A zone spawned on the aircraft puts it inside through no action of its
    own (time_to_safe's business); a frozen fix lets it fly in (an entry)."""
    on_top = S.run_episode(_cell("dyn-nfz-spawn-on-top", size_m=40, variant=0),
                           DEFAULTS, 0).extra
    assert on_top["ticks_inside_fence_polygon"] > 0 and on_top["polygon_entries"] == 0, on_top
    flown = S.run_episode(_cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0]),
                          DEFAULTS, 0).extra
    assert flown["polygon_entries"] >= 1, flown
    ctrl = _run("nfz-head-on-control")[1]
    assert ctrl["polygon_entries"] == 1, ctrl
    assert _run("nfz-head-on")[1]["polygon_entries"] == 0


def _item(triggered, match, status="pass"):
    return {"status": status, "kpi": {"failsafe_triggered": triggered,
                                      "failsafe_matches_label": match}}


def test_failsafe_correctness_is_reported_with_both_nulls_and_the_grant_target():
    """Every score needs its null (CONTRIBUTING rule 4). Labels: 2 expected,
    4 not expected; the Shield got 3 right, 2 false triggers, 1 miss."""
    items = [_item(False, 1.0), _item(True, 0.0), _item(False, 0.0),
             _item(True, 1.0), _item(False, 1.0), _item(True, 0.0),
             _item(False, None)]
    s = S.failsafe_summary(items)
    assert (s["labelled"], s["unlabelled"]) == (6, 1), s
    assert (s["labelled_expected"], s["labelled_not_expected"]) == (2, 4), s
    assert (s["correct"], s["false_triggers"], s["missed_triggers"]) == (3, 2, 1), s
    assert s["fail_safe_correctness"] == 0.5
    assert s["null_never_trigger"] == round(4 / 6, 6)
    assert s["null_always_trigger"] == round(2 / 6, 6)
    assert s["beats_never_trigger"] is False and s["meets_grant_target"] is False
    assert s["grant_target"] == 0.99 and s["discriminating"] is True
    empty = S.failsafe_summary([_item(False, None)])
    assert empty["fail_safe_correctness"] is None and empty["null_never_trigger"] is None


def test_an_only_name_that_matches_nothing_refuses_the_run():
    """Silence reads as success: `--only nfz-head-onn` used to run 0 episodes
    and exit 0."""
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "s.json"
        rc = _sweep(["--only", "nfz-head-onn", "--out", str(out), "--no-bundles"])
        assert rc == 2 and not out.exists(), rc


def test_an_only_cell_id_is_matched_against_the_profiles_own_cells():
    """The variant-2 cell exists only when the nightly profile samples
    three variants. The first version matched --only against the defaults
    (one variant), selected nothing, and exited 0. (Its family pin came off
    on 2026-10-19 and back on 2026-10-20, when the family flies 400 ticks and
    the Loiter timeout G5 fires on all three seeds.)"""
    cell = "three-simultaneous-violations@ask_mps=8,start_up=3,variant=2"
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "s.json"
        rc = _sweep(["--profile", "nightly", "--only", cell, "--out", str(out),
                     "--no-bundles"])
        doc = json.loads(out.read_text(encoding="utf-8"))
    assert rc == 0, rc
    assert [r["id"] for r in doc["results"]] == [cell] * 3, [r["id"] for r in doc["results"]]
    # The family's pin came off on 2026-11-02 with the rubric's labels (E5:
    # the Loiter timeout's RTL is the expected fail-safe): every seed passes.
    assert {r["status"] for r in doc["results"]} == {"pass"}


def test_two_cells_that_fly_the_same_episode_refuse_the_run():
    """Padding at run time, not only in the unit test of the committed
    profiles: a copy of a family under a new id and a new description is the
    same flight."""
    copy_ = _raw("speed-cap", scenario_id="speed-cap-again",
                 description="the same flight, described differently")
    rc, doc = _sweep_lib([_raw("speed-cap"), copy_])
    assert rc == 2 and doc is None, rc


def test_a_profile_outside_its_declared_count_refuses_unless_narrowed():
    with tempfile.TemporaryDirectory() as td:
        prof = Path(td) / "tiny.yaml"
        prof.write_text(yaml.safe_dump({
            "profile": "tiny", "cadence": "test", "grant_scope": "~2 scenarios",
            "grant_sim_speedup": "n/a", "seeds": [1001], "expected_cells": [1, 2]}),
            encoding="utf-8")
        out = Path(td) / "s.json"
        assert _sweep(["--profile", str(prof), "--out", str(out), "--no-bundles"]) == 2
        assert not out.exists()
        rc = _sweep(["--profile", str(prof), "--only", "speed-cap", "--out", str(out),
                     "--no-bundles"])
        assert rc == 0 and out.is_file(), rc


def test_a_run_that_scores_nothing_does_not_exit_zero():
    """Every episode skipped is a run that tested nothing: exit 3, never 0.
    (Until 2026-10-07 an unsupported event made the skip; every event type is
    supported now, so the skip is the SITL backend's: either the rail is not
    on this host, or it refuses a scenario that is not its own mission - and
    neither launches anything.)"""
    rc, doc = _sweep_lib([_raw("speed-cap")], "--backend", "sitl")
    assert doc["_counts"]["skipped"] == 1 and rc == 3, (doc["_counts"], rc)


def test_a_plain_run_does_not_overwrite_the_published_sweep():
    """docs/data/scenario_sweep.json is read by the evaluation data, the deck
    and tests/test_kpi_magnitudes.py. Only --publish writes it, and --publish
    refuses a partial run."""
    assert S.results_path(None, False, None) == ROOT / "demo" / "out" / "sweep" / "sweep.json"
    assert S.results_path(None, False, "smoke").parent.name == "stress_smoke"
    assert S.results_path(None, True, None) == S.PUBLISHED_SWEEP
    assert S.PUBLISHED_SWEEP == ROOT / "docs" / "data" / "scenario_sweep.json"
    before = S.PUBLISHED_SWEEP.stat().st_mtime if S.PUBLISHED_SWEEP.exists() else None
    assert _sweep(["--publish", "--only", "speed-cap"]) == 2
    after = S.PUBLISHED_SWEEP.stat().st_mtime if S.PUBLISHED_SWEEP.exists() else None
    assert before == after, "a refused --publish touched the published file"


def test_the_published_copy_drops_only_the_duplicated_per_episode_fields():
    """Each episode's arm record and manifest repeat what its bundle and
    `_manifest_common` hold. The published view leaves those two out and keeps
    every field a consumer reads, without touching the full run document."""
    row = {"id": "f/c/shield-on", "status": "pass", "why": "w",
           "kpi": {"p0_violation_escape_rate": 0.0, "reached_goal": True},
           "manifest": {"seed": 1}, "bundle": "demo/out/x",
           "manifest_extras": {"paraphrase_id": "pp-1"},
           "arm_record": {"paraphrase": {"text": "long text"}}}
    doc = {"_counts": {"pass": 1}, "_cell_rollup": {"f/c/shield-on": {}},
           "results": [row]}
    pub = S.published_view(doc)
    r = pub["results"][0]
    assert "arm_record" not in r and "manifest" not in r
    for k in ("id", "status", "why", "kpi", "bundle", "manifest_extras"):
        assert r[k] == row[k], k
    assert r["manifest_extras"]["paraphrase_id"] == "pp-1"
    assert pub["_counts"] == doc["_counts"] and pub["_cell_rollup"] == doc["_cell_rollup"]
    assert pub["_published_drops"]["fields"] == ["arm_record", "manifest"]
    assert "arm_record" in doc["results"][0], "the full run doc was mutated"


def test_the_sweep_reports_how_many_cells_behaved_differently():
    """A count of cells is not a count of behaviours; the distinct KPI
    signatures sit beside it. Two cells with identical KPI tables share one."""
    a = {"status": "pass", "kpi": {"repair_count": 3, "min_true_dist_to_fence_m": 1.0012}}
    b = {"status": "pass", "kpi": {"repair_count": 3, "min_true_dist_to_fence_m": 1.0049}}
    c = {"status": "pass", "kpi": {"repair_count": 4, "min_true_dist_to_fence_m": 1.0}}
    assert S.kpi_signature(a) == S.kpi_signature(b) != S.kpi_signature(c)
    rc, doc = _sweep_lib([_raw("speed-cap"), _raw("nfz-head-on")])
    assert doc["_distinct_kpi_signatures"]["distinct"] == 2
    assert all("distinct_kpi_signatures" in f for f in doc["_families"].values())


# --------------------------------------------------------------------------- #
# 2026-10-07: every grant event type, through the Shield's own APIs
# --------------------------------------------------------------------------- #

def test_every_grant_event_type_reaches_the_shield_and_changes_the_rule():
    """REGRESSION (WP4-03, follow-ups #131 / #137). Until 2026-10-07 the harness
    refused six of the grant's seven event types ("no Shield API"); the one it
    ran was a polygon_fence, which the Shield now locks at mission start.
    Each type must go through the Shield's own event API (generation + 1, a
    Shield event record) and change exactly what it names."""
    base = BY_ID["dyn-nfz-spawn-ahead"].model_dump(mode="json", by_alias=True)
    base = {**base, "events": [], "parameter_sweep": {}, "parameter_sample": {},
            "ticks": 120}
    base["mission"] = {**base["mission"], "pilot": {"type": "goto", "speed": 4.0},
                       "rotate_deg": 0.0}
    spawn = {"at_sim_t": 1.0, "type": "spawn_polygon_fence",
             "payload": {"id": "z", "vertices": [{"x": -150, "y": 30}, {"x": -140, "y": 30},
                                                 {"x": -140, "y": 40}, {"x": -150, "y": 40}]}}

    def fly(*evs, **over):
        spec = ScenarioSpec.model_validate({**base, "events": [spawn, *evs], **over})
        return S.run_episode(spec, DEFAULTS, 0)

    def verts(run):
        z = [c for c in run.policy.constraints if c.id == "z"][0]
        return [(round(v.x, 6), round(v.y, 6)) for v in z.vertices], z
    r = fly({"at_sim_t": 2.0, "type": "translate_polygon_fence",
             "payload": {"id": "z", "dx_m": 5.0, "dy_m": -2.0}})
    v, _ = verts(r)
    assert v[0] == (-145.0, 28.0), v
    ops = [e["op"] for e in r.events if e["type"] == "hot_apply"]
    assert ops == ["spawn", "translate"] and r.extra["policy_generation_final"] == 2, ops
    r = fly({"at_sim_t": 2.0, "type": "translate_polygon_fence",
             "payload": {"id": "z", "mps": 2.0, "bearing_deg": 90.0}})
    v, z = verts(r)
    assert z.motion is not None and abs(z.motion.vy_mps - 2.0) < 1e-9, z.motion
    assert abs(z.motion.vx_mps) < 1e-9 and v[0] == (-150.0, 30.0), (z.motion, v)
    r = fly({"at_sim_t": 2.0, "type": "rotate_polygon_fence",
             "payload": {"id": "z", "angle_deg": 90.0}})
    v, _ = verts(r)
    assert sorted(v) == sorted([(-140.0, 30.0), (-140.0, 40.0), (-150.0, 40.0),
                                (-150.0, 30.0)]), v          # a square onto itself
    r = fly({"at_sim_t": 2.0, "type": "rotate_polygon_fence",
             "payload": {"id": "z", "deg_per_s": 6.0}})
    assert abs(verts(r)[1].motion.yaw_rate_dps - 6.0) < 1e-9
    r = fly({"at_sim_t": 2.0, "type": "scale_radius", "payload": {"id": "z", "factor": 2.0}})
    v, _ = verts(r)
    xs, ys = [p[0] for p in v], [p[1] for p in v]
    assert (max(xs) - min(xs), max(ys) - min(ys)) == (20.0, 20.0), v
    r = fly({"at_sim_t": 2.0, "type": "deactivate_rule", "payload": {"rule_id": "nfz-square"}})
    sw = [c for c in r.policy.constraints if c.type == "time_window_switch"]
    assert len(sw) == 1 and sw[0].target_id == "nfz-square" and not sw[0].active, sw
    r = fly({"at_sim_t": 2.0, "type": "activate_rule",
             "payload": {"rule_id": "z", "id": "sw-z", "window":
                         {"start_time": "00:00", "end_time": "23:59"}}},
            **{"mission": {**base["mission"], "clock_start": "2026-09-07T10:00:00"}})
    sw = [c for c in r.policy.constraints if c.id == "sw-z"]
    assert sw and sw[0].active and sw[0].valid_time.recurrence is not None, sw
    cor = BY_ID["corridor-along"].model_dump(mode="json", by_alias=True)
    cor["events"] = [{"at_sim_t": 2.0, "type": "swap_corridor",
                      "payload": {"target_id": "corridor-survey-route", "width_m": 30.0,
                                  "centerline": [[0, 0], [60, 0]]}}]
    r = S.run_episode(ScenarioSpec.model_validate(cor), DEFAULTS, 0)
    sp = [c for c in r.policy.constraints if c.type == "corridor_swap"]
    assert len(sp) == 1 and sp[0].width_m == 30.0, sp
    # Every event of every run above went through the Shield: one record each.
    assert all(e.get("generation") for e in r.events if e["type"] == "hot_apply")


def test_a_windowed_switch_without_a_clock_is_refused():
    """With no clock every rule is in force, so a switch "scheduled" for
    12:00-12:30 would act from the event on: an unscheduled switch under
    another name. Refused before the episode runs."""
    spec = _with(BY_ID["nfz-head-on"], events=[{
        "at_sim_t": 1.0, "type": "deactivate_rule",
        "payload": {"rule_id": "nfz-square",
                    "window": {"start_time": "12:00", "end_time": "12:30"}}}])
    try:
        S.run_episode(spec, DEFAULTS, 0)
    except ValueError as e:
        assert "clock_start" in str(e)
    else:
        raise AssertionError("a windowed switch ran with no clock")


def test_a_hot_applied_switch_with_its_own_window_releases_the_rule_at_its_edge():
    """time_window_switch as a real event: sent at 1 s, in force from 12:00:00
    (10 s in). The rule binds until the edge and is released after it, on the
    conservative side; three phases (before the event, between, after)."""
    run = S.run_episode(BY_ID["time-window-switch-event"], DEFAULTS, 0)
    ex = run.extra
    assert ex["phase1_violation_ticks_nfz-school"] > 0, ex
    assert ex["phase2_violation_ticks_nfz-school"] == 0, ex
    assert 0 <= ex["switch_lag_s_nfz-school"] <= 0.35, ex["switch_lag_s_nfz-school"]
    assert ex["switch_conservative_nfz-school"] is True
    assert ex["reached_goal"] is True and ex["polygon_entries"] == 0


# --------------------------------------------------------------------------- #
# 2026-10-07: the escalation FSM decides what is flown
# --------------------------------------------------------------------------- #

def test_rtl_and_land_are_outcomes_the_fsm_produced_and_the_autopilot_flew():
    """REGRESSION (follow-ups #117b / #118 / #142). The harness could produce
    only success or fail: "RTL_triggered and Land_triggered need the escalation
    FSM, which the Shield does not run yet". A zone whose breach action is
    RTL / land now sends the FSM straight to RTL (G11) / Land (G12), the
    autopilot model flies the mode, and the FSM's terminal edge ends the
    episode."""
    from guardrail import kpi as K
    rtl = S.run_episode(_cell("failsafe-rtl-zone", ahead_m=12, variant=0), DEFAULTS, 0)
    land = S.run_episode(BY_ID["failsafe-land-zone"], DEFAULTS, 0)
    for run, label, edge, term in ((rtl, "RTL_triggered", "G11", "G9"),
                                   (land, "Land_triggered", "G12", "G10")):
        fsm = run.extra["fsm"]
        assert fsm["outcome_label"] == label and fsm["failsafe_triggered"], fsm
        edges = [t["edge"] for t in fsm["transitions"]]
        assert edges == [edge, term], edges
        assert run.extra["autopilot_ticks"] > 0 and run.extra["terminated_s"] is not None
        ap = [r for r in run.rows if r.get("autopilot")]
        assert ap and all(r["owner"] == "autopilot" for r in ap)
        res = K.compute(run.rows, run.priorities, {"fsm": fsm})
        assert res["outcome"] == label and res["mission_success"] is False, res["outcome"]
        assert res["autopilot_ticks"] == len(ap) and res["shield_ticks"] == len(run.rows) - len(ap)
    # Land descends at the autopilot's 0.5 m/s to the ground; RTL ends over home.
    assert land.rows[-1]["up"] <= S.AP_LANDED_M + 1e-9, land.rows[-1]["up"]
    home = (rtl.rows[0]["x"], rtl.rows[0]["y"])
    end = rtl.rows[-1]
    assert ((end["x"] - home[0]) ** 2 + (end["y"] - home[1]) ** 2) ** 0.5 <= S.AP_HOME_TOL_M
    # The landing broke the 10 m floor under the AUTOPILOT: counted apart,
    # never as the Shield's escape or breach.
    assert land.extra["breaches"] == 0 and land.extra["autopilot_breaches"] >= 1, land.extra
    res = S.score(BY_ID["failsafe-land-zone"], land.rows, land.extra, land.policy,
                  land.priorities)
    assert res["p0_escapes"] == 0 and res["true_p0_flown_ticks"] > 0, res


def test_failsafe_triggered_is_the_fsm_entering_rtl_or_land_not_a_brake():
    """REGRESSION (follow-up #118). `failsafe_triggered` was "any row braked",
    so every Shield brake was scored as the grant's fail-safe. The ceiling hold
    brakes and loiters (theta exceeded) and never reaches RTL / Land: not a
    fail-safe trigger. With the FSM off nothing can be said: None, not False.
    (2026-11-02: flown at its old 30 s, the hold ends in Loiter 0.1 s before
    the operator timeout - not a trigger, and since then CENSORED, not a
    correct non-trigger; at the library's 50 s the timeout sends it home,
    the fail-safe the rubric expects, E5.)"""
    spec = _with(BY_ID["altitude-ceiling-hold"], ticks=300)
    run = S.run_episode(spec, DEFAULTS, 0)
    assert any(r["setpoint"] == "brake" for r in run.rows), "the scenario no longer brakes"
    assert "Loiter" in run.extra["fsm"]["visited"]
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["failsafe_triggered"] is False and res["failsafe_censored"] is True
    assert res["failsafe_matches_label"] is None
    full = BY_ID["altitude-ceiling-hold"]
    rf = S.run_episode(full, DEFAULTS, 0)
    resf = S.score(full, rf.rows, rf.extra, rf.policy, rf.priorities)
    assert resf["failsafe_triggered"] is True and resf["failsafe_matches_label"] == 1.0
    assert full.failsafe_clause == "E5"
    off = _with(spec, escalation=False)
    r2 = S.run_episode(off, DEFAULTS, 0)
    res2 = S.score(off, r2.rows, r2.extra, r2.policy, r2.priorities)
    assert res2["failsafe_triggered"] is None and res2["failsafe_matches_label"] is None
    assert any("not measured" in b for b in S.label_failures(off, res2))


def test_a_zone_moving_onto_a_waiting_aircraft_is_imposed_not_its_breach():
    """A zone that grows over the aircraft (scale_radius) makes its position
    illegal without the aircraft moving: time to safe, never a breach or a
    polygon entry. The test: the position the aircraft moved to was already
    illegal at the previous instant, under the rules of that instant. The
    negative control is the unshielded flight into a static zone, which IS
    its own entry."""
    # factor 3.8 since 2026-10-12 (the family re-cut for the 5 s forecast;
    # 3.0 was the 8.2 m-deep cell at 3 s, 3.8 is it at 5 s).
    run = S.run_episode(_cell("radius-scaling-grow", factor=3.8), DEFAULTS, 0)
    ex = run.extra
    assert ex["ticks_inside_fence_polygon"] > 0 or any(r["unsafe"] for r in run.rows)
    assert ex["breaches"] == 0 and ex["polygon_entries"] == 0, ex
    ctrl = _run("nfz-head-on-control")[1]
    assert ctrl["breaches"] >= 1 and ctrl["polygon_entries"] == 1, ctrl
    # The growth lands on an event tick, which is imposed whatever the test.
    # A window opening INSIDE the lookahead makes the spot unsafe on an
    # ordinary tick, seconds before the switch: imposed by time, not by the
    # move - and only the "judged at the previous instant" test knows it.
    # Re-pinned 2026-10-12: at the grant's 5 s forecast the family's old
    # clocks (07:29:54-56) never left the aircraft where the zone opens (the
    # Shield anticipates the window and holds it out), so the clock starts 4 s
    # earlier: from 07:29:50 the spot is unsafe from 5.0 s, five seconds
    # before the 10.0 s switch, on no event tick. The family itself was
    # re-cut the same day to start INSIDE the zone (its first unsafe tick is
    # tick 0, imposed whatever the test), so this flies the old route start,
    # (60, 0), explicitly.
    tw = S.run_episode(_with(_cell("time-window-opens-on-vehicle",
                                   clock="2026-09-07T07:29:58"),
                             **{"mission.clock_start": "2026-09-07T07:29:50",
                                "mission.start_pose.y": 0.0}),
                       DEFAULTS, 0)
    first_unsafe = next(r["t"] for r in tw.rows if r["unsafe"])
    switch = [e["t"] for e in tw.events if e["type"] == "time_window_switch"][0]
    events = {e["t"] for e in tw.events if e["type"] == "hot_apply"}
    assert first_unsafe < switch and first_unsafe not in events, (first_unsafe, switch)
    assert tw.extra["breaches"] == 0, tw.extra["breaches"]


def test_a_switch_is_conservative_only_on_the_right_side_and_inside_the_band():
    """The t- / t+ verdict on its own: OFF held until after the edge, ON
    enforced from before it, each within half the forecast step plus a tick.
    The minute-resolution reading (a 17:30 window held to 17:30:59, lag +59 s)
    and an early switch-off must both be refused."""
    eps, dt = 0.25, 0.1
    assert S.switch_conservative(False, 0.3, eps, dt)            # OFF late: held
    assert S.switch_conservative(True, -0.2, eps, dt)            # ON early: enforced
    assert not S.switch_conservative(False, -0.2, eps, dt)       # OFF early
    assert not S.switch_conservative(True, 0.2, eps, dt)         # ON late
    assert not S.switch_conservative(False, 59.3, eps, dt)       # minute resolution
    assert S.switch_conservative(False, 0.0, eps, dt) and S.switch_conservative(True, 0.0, eps, dt)


def test_the_labelled_failsafe_summary_says_which_edge_fired():
    """The labelled fail-safe correctness is printed with the FSM's own scorer
    (Wilson bound, meets_target None on undiscriminating labels) and, per
    triggered episode, the chain of FSM edges that led to RTL / Land - so a
    correctness below the grant's 99 % says why."""
    def item(trig, match, edges=()):
        return {"status": "pass",
                "kpi": {"failsafe_triggered": trig, "failsafe_matches_label": match,
                        "fsm": {"transitions": [{"edge": e, "to": to} for e, to in edges]}}}
    s = S.failsafe_summary([item(True, 1.0, [("G11", "RTL"), ("G9", "[*]")]),
                            item(True, 0.0, [("X1", "Brake"), ("G3", "Loiter"),
                                             ("G5", "RTL")]),
                            item(False, 1.0), item(False, 1.0)])
    assert s["triggers_by_edge"] == {"true": {"G11": 1}, "false": {"X1 > G3 > G5": 1}}, s
    fs = s["fsm_scoring"]
    assert fs["scored"] == 4 and fs["wilson_low_95"] is not None, fs
    assert fs["meets_target"] is False and fs["min_error_free_episodes_for_target_at_95"] == 381
    one_kind = S.failsafe_summary([item(False, 1.0), item(False, 1.0)])
    assert one_kind["fsm_scoring"]["meets_target"] is None, "a stub would tie it"


# --------------------------------------------------------------------------- #
# 2026-10-07: arms and the KPI report
# --------------------------------------------------------------------------- #

def test_paraphrase_and_prefix_arms_log_their_ids_and_fly_the_same_flight():
    """WP4-09 / WP4-15 plumbing. nfz-head-on flies its rule summary in three
    wordings (the canonical one and two stored paraphrases) with the prefix
    on and off: six episodes, each bundle with the six-field manifest and a
    manifest_extras.json naming its paraphrase_id and prefix arm, and all six
    the same flight - the headless pilot reads no text, so a difference would
    be a harness defect, and the summary says it is plumbing only."""
    with tempfile.TemporaryDirectory() as td:
        out, runs = Path(td) / "s.json", Path(td) / "runs"
        rc = _sweep(["--only", "nfz-head-on", "--out", str(out), "--runs", str(runs),
                     "--seed", "1001"])
        doc = json.loads(out.read_text(encoding="utf-8"))
        dirs = sorted(p for p in runs.iterdir() if p.is_dir())
        assert rc == 0 and len(dirs) == 6, (rc, [d.name for d in dirs])
        ids = set()
        for d in dirs:
            man = json.loads((d / "manifest.json").read_text(encoding="utf-8"))
            assert len(man) == 6, "the manifest is the grant's six fields"
            ex = json.loads((d / "manifest_extras.json").read_text(encoding="utf-8"))
            assert ex["prefix_arm"] in ("on", "off") and ex["pilot_reads_text"] is False
            ids.add((ex["paraphrase_id"], ex["prefix_arm"]))
        assert len(ids) == 6, ids
        canon = [r for r in doc["results"] if "canonical" in r["id"]]
        assert all(r["arm_record"]["paraphrase"]["backend"] == "identity" for r in canon)
        arms = doc["_arms"]
        assert arms["groups_whose_arms_flew_differently"] == [], arms
        assert arms["prefix_ab_pairs"] == 1 and not arms["prefix_ab_pairs_that_differ"]
        assert arms["pilot_reads_text"] is False and "plumbing" in arms["note"]
        pp = arms["per_paraphrase"]
        # Grouped by paraphrase_id alone: one wording shown with the prefix on
        # and off is one arm of two trials.
        assert pp["canonical_trials"] == 2 and pp["paraphrase_arms"] == 2, pp
        assert pp["worst_drop_vs_canonical"] == 0.0, pp
        assert len({kpi for kpi in (json.dumps(r["kpi"]["mission_success"])
                                    for r in doc["results"])}) == 1


def test_the_run_writes_the_grants_kpi_report_one_row_per_template_and_cell():
    """Stress Testing p6: "Per-scenario-family stats table. One row per
    (scenario template, parameter cell) tuple", plus top-K, in Markdown. The
    sweep writes kpi_report.md with one row per cell and Shield arm, the
    nulls beside the scores, the labelled fail-safe correctness, and the
    not-KPI-grade note first."""
    fams = [_raw("altitude-recovery-depth", parameter_sweep={"start_up": [6.0, 20.5]}),
            _raw("nfz-head-on-control")]
    with tempfile.TemporaryDirectory() as td:
        lib = Path(td) / "lib.yaml"
        lib.write_text(yaml.safe_dump({"defaults": RAW["defaults"], "scenarios": fams}),
                       encoding="utf-8")
        out, runs = Path(td) / "s.json", Path(td) / "runs"
        rc = _sweep(["--library", str(lib), "--out", str(out), "--runs", str(runs)])
        md = (runs / "kpi_report.md").read_text(encoding="utf-8")
        doc = json.loads(out.read_text(encoding="utf-8"))
    assert rc == 0, rc
    # The run's own KPI-bearing answer sits in its scope (review: a profile's
    # `grant_kpi_bearing: true` beside a headless result read as the run's).
    assert doc["_scope"]["kpi_bearing_this_run"] is False, doc["_scope"]
    assert all("\\" not in (r.get("bundle") or "") for r in doc["results"])
    assert "p0_acted_tick_share" in doc["_rollup"]["_all_shield_on"]
    assert "failsafe_trigger_correctness" not in doc["_rollup"]["_all_shield_on"]
    assert "P0-acted tick share" in md and "Fail-safe correctness (ticks)" not in md
    # What the autopilot flew is on the page, beside the Shield's own count.
    assert "True state, and what the autopilot flew" in md
    assert "outside the P0 escape rate" in md
    keys = sorted(doc["_cell_rollup"])
    assert keys == ["altitude_envelope/altitude-recovery-depth@start_up=20.5/shield-on",
                    "altitude_envelope/altitude-recovery-depth@start_up=6/shield-on",
                    "static_nfz/nfz-head-on-control/shield-off"], keys
    for k in keys:
        assert f"`{k}`" in md, k
    assert "Not KPI-grade" in md and "Nulls beside the scores" in md
    assert "Fail-safe trigger correctness (labelled episodes)" in md
    assert "Top-10 failure cases" in md


def _sweep_with_broken(attr, fail_on_call, extra=(), err=None):
    """main() on the KPI-report library above with guardrail.kpi.<attr>
    raising on its Nth call; (rc, doc, kpi_report.md text or None). `extra`
    is added to the command line; a list given as `err` receives what the
    run printed to stderr."""
    fams = [_raw("altitude-recovery-depth", parameter_sweep={"start_up": [6.0, 20.5]}),
            _raw("nfz-head-on-control")]
    real = getattr(S.K, attr)
    calls = {"n": 0}

    def flaky(*a, **k):
        calls["n"] += 1
        if calls["n"] == fail_on_call:
            raise KeyError("a_new_kpi_field")
        return real(*a, **k)
    setattr(S.K, attr, flaky)
    try:
        with tempfile.TemporaryDirectory() as td:
            lib = Path(td) / "lib.yaml"
            lib.write_text(yaml.safe_dump({"defaults": RAW["defaults"], "scenarios": fams}),
                           encoding="utf-8")
            out, runs = Path(td) / "s.json", Path(td) / "runs"
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                rc = _sweep(["--library", str(lib), "--out", str(out), "--runs",
                             str(runs), *extra])
            if err is not None:
                err.append(stderr.getvalue())
            else:
                sys.stderr.write(stderr.getvalue())
            md = ((runs / "kpi_report.md").read_text(encoding="utf-8")
                  if (runs / "kpi_report.md").is_file() else None)
            doc = json.loads(out.read_text(encoding="utf-8"))
    finally:
        setattr(S.K, attr, real)
    return rc, doc, md


def test_a_rollup_that_fails_part_way_writes_no_partial_report_and_exits_4():
    """REVIEW 2026-10-19 (harness-tools/rollup-failure-swallowed). Any
    exception in the rollup loop went into `_rollup_note`, which nothing
    printed or read, and the run exited 0: with the LAST cell's rollup
    raising, kpi_report.md was written with 2 of 3 rows and no word of it;
    with the first cell's, no report and a normal-looking summary. Now
    nothing partial is kept or written, and the run exits 4."""
    for nth in (6, 4, 2):     # 1 family, all-on, controls-off, then 3 cells
        rc, doc, md = _sweep_with_broken("rollup", nth)
        assert rc == 4, (nth, rc)
        assert md is None, f"call {nth}: a partial KPI report was written"
        assert doc["_cell_rollup"] is None and doc["_rollup"] is None, (nth, doc["_rollup"])
        assert "a_new_kpi_field" in doc["_rollup_note"], doc["_rollup_note"]


def test_a_top_failures_crash_is_flagged_never_shown_as_no_failure():
    rc, doc, md = _sweep_with_broken("top_failures", 1)
    assert rc == 4, rc
    assert md is not None and "INCOMPLETE" in md, md
    assert "(no failing episode)" not in md and "top failures not computed" in md
    assert doc["_cell_rollup"], "the cell rollup itself succeeded and is kept"


def test_the_rollup_failure_message_names_a_report_only_when_one_was_written():
    """REVIEW of F8-harness-tools (P6). The KPI ROLLUP FAILED line chose its
    wording on `cells` alone and said "The KPI report carries the failure at
    its top" - but kpi_report.md is written only with cells AND bundles, so
    a --no-bundles run whose top_failures crashed named a report that was
    never written. Shown failing before the fix."""
    err = []
    rc, _, md = _sweep_with_broken("top_failures", 1, extra=["--no-bundles"], err=err)
    assert rc == 4 and md is None, (rc, md)
    line = next(ln for ln in err[0].splitlines() if "KPI ROLLUP FAILED" in ln)
    assert "No KPI report was written" in line and "carries the failure" not in line, line
    # the null: with bundles the report is written and the line names it
    err = []
    rc, _, md = _sweep_with_broken("top_failures", 1, err=err)
    assert rc == 4 and md is not None
    line = next(ln for ln in err[0].splitlines() if "KPI ROLLUP FAILED" in ln)
    assert "The KPI report carries the failure at its top" in line, line


# --------------------------------------------------------------------------- #
# 2026-10-07, review of stress-harness-2: the mutants that survived
# --------------------------------------------------------------------------- #

def test_the_failsafe_trigger_is_the_fsms_verdict_never_a_braked_row():
    """REGRESSION (follow-up #118; review mutant R11 survived all four test
    files). `failsafe_triggered` was "any row braked"; the two definitions
    disagree on 233 of 789 nightly episodes. Both directions are pinned: a
    flight whose Shield brakes for 231 ticks while the FSM never leaves
    Brake / Loiter is NOT a fail-safe trigger, and a flight the FSM sends to
    RTL (G5, the Loiter timeout) without one braked row IS one."""
    # At its old 30 s (the library flies it 70 s since 2026-11-02, where the
    # Loiter timeout sends it home).
    spec = _with(_cell("corridor-drift-angles", ask_mps=4, drift_deg=60), ticks=300)
    run = S.run_episode(spec, DEFAULTS, 0)
    assert any(r.get("braked") for r in run.rows), "the Shield no longer brakes here"
    assert run.extra["fsm"]["failsafe_triggered"] is False, run.extra["fsm"]
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["failsafe_triggered"] is False, "a brake was read as the fail-safe"
    spec = BY_ID["wind-beyond-authority"]
    run = S.run_episode(spec, DEFAULTS, 0)
    assert not any(r.get("braked") for r in run.rows), "the scenario now brakes"
    assert run.extra["fsm"]["outcome_label"] == "RTL_triggered", run.extra["fsm"]
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["failsafe_triggered"] is True, "an RTL with no brake was missed"


def test_what_the_autopilot_flew_through_is_counted_apart_and_reported():
    """REVIEW (stress-harness-2, mutant R41 survived). The modelled RTL flies
    straight home whatever lies between: on dyn-nfz-spawn-on-top@size_m=20 it
    flies through the zone (16 P0 ticks, one polygon entry) after the FSM
    handed over. That is never the Shield's entry or escape - and it must
    not vanish either: the run-level true-state summary reports it apart.
    Since 2026-10-09 X6 leaves this cell's working exit alone (G1, G7), so
    the modelled RTL through the zone is recreated by flying the same cell
    with X6's progress test off (G1>X6>G9).
    Re-pinned on the review of 2026-10-12: the RTL watch (guardrail/
    rtl_path.py) now refuses that leg - it starts inside the zone and runs
    12.3 m deeper through it before it leaves, which is a traversal, not an
    exit - so G6 lands the aircraft where it is, inside the zone, and the
    autopilot's P0 ticks are LAND's (counted apart, by mode). The RTL
    through the zone this test needs is flown with the watch silenced."""
    from guardrail.fsm import FSMConfig

    class NoProgressTest(S.Shield):
        def __init__(self, policy, *a, escalation=True, **k):
            if escalation is True:
                escalation = FSMConfig(x6_min_escape_mps=1e9)
            super().__init__(policy, *a, escalation=escalation, **k)

    spec = _cell("dyn-nfz-spawn-on-top", size_m=20, variant=0)
    default = S.run_episode(spec, DEFAULTS, 0)
    got = ">".join(t["edge"] for t in (default.extra.get("fsm") or {}).get("transitions") or [])
    assert got == "G1>G7", f"the shipped X6 should leave this exit alone: {got!r}"
    real = S.Shield
    S.Shield = NoProgressTest
    try:
        watched = S.run_episode(spec, DEFAULTS, 0)
        assess = S.RP.RTLWatch.assess

        def silent(self, *a, **k):
            v = assess(self, *a, **k)
            return S.RP.RTLVerdict(False, (), v.leg, False, False)
        S.RP.RTLWatch.assess = silent
        try:
            run = S.run_episode(spec, DEFAULTS, 0)
        finally:
            S.RP.RTLWatch.assess = assess
    finally:
        S.Shield = real
    wx = watched.extra
    assert [t["edge"] for t in wx["fsm"]["transitions"]] == ["G1", "X6", "G6"], wx["fsm"]
    assert "crosses dyn-nfz" in " ".join(wx["rtl_failures"][0]["reasons"]), wx["rtl_failures"]
    assert set(wx["autopilot_p0_flown_ticks_by_mode"]) == {"LAND"}, wx
    assert wx["autopilot_polygon_entries"] == 0, wx
    edges = [t["edge"] for t in (run.extra.get("fsm") or {}).get("transitions") or []]
    assert "X6" in edges, f"the RTL this test needs did not happen: {edges}"
    ex = run.extra
    assert ex["autopilot_polygon_entries"] >= 1 and ex["polygon_entries"] == 0, ex
    assert 0 < ex["autopilot_p0_flown_ticks"] <= ex["true_p0_flown_ticks"], ex
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["p0_escapes"] == 0, "the Shield's own counter is clean here"
    ts = S.true_state_summary([{"status": "pass", "arm": "on", "kpi": res}])
    assert ts["episodes_entering_polygon"] == 0, ts
    assert ts["autopilot_polygon_entries"] == ex["autopilot_polygon_entries"], ts
    assert ts["episodes_autopilot_entering_polygon"] == 1, ts
    assert ts["autopilot_p0_flown_ticks"] == ex["autopilot_p0_flown_ticks"], ts
    assert ts["episodes_autopilot_p0_flown_by_status"] == {"pass": 1}, ts
    assert ts["autopilot_ticks"] == ex["autopilot_ticks"] > 0, ts
    # REVIEW 2026-10-19 (imposed-only-counts-autopilot-entries): the RTL
    # carried it into the zone, so it is not "inside only because a zone
    # appeared under it" (imposed_only was len(inside) - len(entered) = 1).
    assert ts["episodes_inside_polygon_autopilot_entered"] == 1, ts
    assert ts["episodes_inside_polygon_imposed_only"] == 0, ts


def test_a_motion_swap_on_a_policy_zone_keeps_it_where_it_is():
    """REVIEW 2026-10-19 (harness-tools/ring-now-wrong-anchor-for-policy-
    zones). A dynamic_nfz the POLICY declares (not one a harness event
    spawned) had no harness anchor, so an `mps` / `deg_per_s` event took its
    age as 0 and handed move_nfz its authored vertices: a zone drifting 1 m/s
    and re-tasked at t = 5 s jumped 5 m back. Latent - load_for_flight
    refuses a policy file with a dynamic_nfz - so the Shield is built on an
    in-memory policy and apply_event is called directly."""
    from guardrail.models import policy_from_raw
    from guardrail.scenario_spec import ScenarioEvent

    def drift_policy():
        # A fresh Policy per Shield: a Shield edits the Policy it was built
        # on IN PLACE (Shield._commit - the REST app and the audit logger hold
        # it), so a second Shield built on the same object would start from
        # the first one's edit.
        return policy_from_raw({"policy_id": "drift", "constraints": [
            {"id": "z", "type": "dynamic_nfz", "priority": "P0",
             "vertices": [{"x": 100, "y": 0}, {"x": 110, "y": 0},
                          {"x": 110, "y": 10}, {"x": 100, "y": 10}],
             "motion": {"vx_mps": 1.0, "vy_mps": 0.0, "yaw_rate_dps": 0.0}}]},
            runtime=False)
    sh = Shield(drift_policy())
    st = State(x=0.0, y=0.0, up=15.0, yaw_deg=0.0)
    for i in range(51):                                  # t = 0 .. 5 s
        sh.filter(st, Action4D(), t=i * 0.1)
    es = S._EventState(shields=[sh])
    for ev, want in ((ScenarioEvent(at_sim_t=5.0, type="translate_polygon_fence",
                                    payload={"id": "z", "mps": 2.0, "bearing_deg": 0}),
                      105.0),
                     (ScenarioEvent(at_sim_t=5.0, type="rotate_polygon_fence",
                                    payload={"id": "z", "deg_per_s": 3.0}), 105.0)):
        sh2 = Shield(drift_policy())
        for i in range(51):
            sh2.filter(st, Action4D(), t=i * 0.1)
        rec = S.apply_event(ev, S._EventState(shields=[sh2]), st, (1.0, 0.0), 5.0)
        assert abs(rec["vertices"][0][0] - want) < 1e-6, (ev.type, rec["vertices"])
    rec = S.apply_event(ScenarioEvent(at_sim_t=5.0, type="translate_polygon_fence",
                                      payload={"id": "z", "mps": 2.0, "bearing_deg": 0}),
                        es, st, (1.0, 0.0), 5.0)
    assert rec["vertices"][0] == [105.0, 0.0], rec["vertices"]
    assert sh.zone_ring_at("z", 5.0)[0] == (105.0, 0.0)
    # and from there it moves at the new 2 m/s
    assert abs(sh.zone_ring_at("z", 6.0)[0][0] - 107.0) < 1e-6, sh.zone_ring_at("z", 6.0)


def test_imposed_only_counts_neither_the_shields_nor_the_autopilots_entries():
    """REVIEW 2026-10-19 (harness-tools/imposed-only-counts-autopilot-entries).
    Three episodes inside a polygon: a zone imposed on one, the Shield flew
    into one, the modelled LOITER carried one in (polygon_entries 0,
    autopilot_polygon_entries 1, as the wind-beyond-authority lane does with
    no harness event). Only the first is "imposed only"; the old arithmetic
    len(inside) - len(entered) said 2. An episode both entered is counted
    once, as the Shield's."""
    def item(**kpi):
        return {"status": "pass", "arm": "on",
                "kpi": dict({"ticks_inside_fence_polygon": 5}, **kpi)}
    items = [item(polygon_entries=0, autopilot_polygon_entries=0),
             item(polygon_entries=1, autopilot_polygon_entries=0),
             item(polygon_entries=0, autopilot_polygon_entries=1),
             item(polygon_entries=1, autopilot_polygon_entries=1),
             {"status": "pass", "arm": "on", "kpi": {"ticks_inside_fence_polygon": 0}}]
    ts = S.true_state_summary(items)
    assert ts["episodes_inside_polygon"] == 4, ts
    assert ts["episodes_entering_polygon"] == 2, ts
    assert ts["episodes_inside_polygon_autopilot_entered"] == 1, ts
    assert ts["episodes_inside_polygon_imposed_only"] == 1, ts


def test_a_brake_tick_is_checked_as_the_zero_action_it_flew():
    """REVIEW (mutant R42 survived). On a tick the FSM streams a stop, what
    was flown is Action4D(), so its check is standing still here - not the
    Shield's repaired action, which nobody flew. No library cell has a
    repaired action that still violates on a brake tick, so the Shield is
    made to report one (a phantom violation on every brake tick's repaired
    action) and the flown check must not carry it."""
    from guardrail.shield import Violation
    phantom = Violation(rule_id="phantom", category="geofence",
                        detail="the repaired action's, never flown", predicted_at_s=0.0)

    class Spy(S.Shield):
        def filter(self, *a, **kw):
            d = super().filter(*a, **kw)
            if d.setpoint == "brake":
                d = d.model_copy(update={"emitted_violations":
                                         list(d.emitted_violations) + [phantom]})
            return d
    real = S.Shield
    S.Shield = Spy
    try:
        run = S.run_episode(BY_ID["altitude-ceiling-hold"], DEFAULTS, 0)
    finally:
        S.Shield = real
    brakes = [r for r in run.rows if r.get("setpoint") == "brake"]
    assert brakes, "the scenario no longer brakes"
    carried = [r["t"] for r in brakes
               if any(v["rule_id"] == "phantom" for v in r["emitted_violations"])]
    assert not carried, f"brake ticks checked as the repaired action: {carried[:5]}"


def test_a_second_motion_event_starts_from_where_the_first_left_the_zone():
    """REVIEW (mutant R43 survived). A zone moving East at 2 m/s from t = 2 s
    has moved 4 m when a second event at t = 4 s turns it North: the second
    motion starts there, not 6 m along (the motion measured from the spawn
    at t = 1 s, which is what forgetting to re-anchor the zone gives)."""
    base = BY_ID["dyn-nfz-spawn-ahead"].model_dump(mode="json", by_alias=True)
    base = {**base, "parameter_sweep": {}, "parameter_sample": {}, "ticks": 80}
    base["mission"] = {**base["mission"], "pilot": {"type": "goto", "speed": 4.0},
                       "rotate_deg": 0.0}
    base["events"] = [
        {"at_sim_t": 1.0, "type": "spawn_polygon_fence",
         "payload": {"id": "z", "vertices": [{"x": -150, "y": 30}, {"x": -140, "y": 30},
                                             {"x": -140, "y": 40}, {"x": -150, "y": 40}]}},
        {"at_sim_t": 2.0, "type": "translate_polygon_fence",
         "payload": {"id": "z", "mps": 2.0, "bearing_deg": 90.0}},
        {"at_sim_t": 4.0, "type": "translate_polygon_fence",
         "payload": {"id": "z", "mps": 2.0, "bearing_deg": 0.0}}]
    run = S.run_episode(ScenarioSpec.model_validate(base), DEFAULTS, 0)
    recs = [e for e in run.events if e["type"] == "hot_apply"]
    assert [r["event"] for r in recs] == ["spawn_polygon_fence", "translate_polygon_fence",
                                          "translate_polygon_fence"], recs
    assert recs[1]["vertices"][0] == [-150.0, 30.0], recs[1]["vertices"]
    assert recs[2]["vertices"][0] == [-150.0, 34.0], recs[2]["vertices"]


def test_the_switch_band_is_half_the_forecast_step_plus_a_tick_end_to_end():
    """REVIEW (mutant R44 survived: eps = 4 x dt instead of dt / 2). The claim
    "eps = dt / 2" is the Shield's forecast step halved, with the harness tick
    (0.1 s) on top; pinned at the call run_episode makes, not only in
    switch_conservative's own unit test. Re-pinned 2026-10-12: the grant's
    forecast step is 0.1 s, so eps is 0.05 s (0.25 s at the 0.5 s step, which
    the 3 s comparison arm still uses)."""
    seen = []
    real = S.switch_conservative

    def spy(turned_on, lag_s, eps, dt):
        seen.append((eps, dt))
        return real(turned_on, lag_s, eps, dt)
    S.switch_conservative = spy
    try:
        run = S.run_episode(BY_ID["time-window-switch-event"], DEFAULTS, 0)
    finally:
        S.switch_conservative = real
    assert seen, "no scheduled switch was judged"
    assert set(seen) == {(0.05, 0.1)}, seen
    assert run.extra["switch_conservative_nfz-school"] is True
    # A lag past the band is not conservative, on either side.
    assert not real(False, 0.36, 0.25, 0.1) and not real(True, -0.36, 0.25, 0.1)


def test_perpendicular_left_of_a_northbound_heading_is_west():
    """REVIEW (mutant R14: left and right swapped, survived). The grant's
    example translates its zone "perpendicular_left". x is North and y East,
    so left of a northbound vehicle is West (y decreasing), right is East."""
    north = (1.0, 0.0)
    assert S._direction({"direction": "perpendicular_left"}, north) == (0.0, -1.0)
    assert S._direction({"direction": "perpendicular_right"}, north) == (-0.0, 1.0)
    east = (0.0, 1.0)
    assert S._direction({"direction": "perpendicular_left"}, east) == (1.0, -0.0)
    assert S._direction({"direction": "along"}, east) == east
    assert S._direction({"direction": "against"}, east) == (-0.0, -1.0)


def test_the_grants_verbatim_example_flies_its_bound_speed_and_wind():
    """The grant's worked example as the grant writes it (bare sweep keys, no
    pilot), with only its bundle - which does not exist here - replaced by a
    policy that has a WGS84 origin. The bound keys must reach the flight:
    the pilot asks for the swept speed and the wind blows at the swept speed
    (a bare key that bound to nothing would fly four identical cells)."""
    import math
    sys.path.insert(0, str(ROOT / "tests"))
    from test_scenario_spec import grant_example_verbatim
    from guardrail.scenario_spec import Family
    ex = grant_example_verbatim()
    ex["policy"] = {"path": "policies/wgs84_taipei.yaml"}
    ex["parameter_sweep"] = {"vehicle_speed_mps": [6, 12], "wind_speed_mps": [0, 5],
                             "random_seed": [1001]}
    got = {}
    for c in Family(ex).cells():
        run = S.run_episode(c.spec, DEFAULTS, 1001)
        ask = max(math.hypot(r["raw"]["vx"], r["raw"]["vy"]) for r in run.rows)
        wind = math.hypot(*run.events[0]["wind_mps"])
        got[(c.params["vehicle_speed_mps"], c.params["wind_speed_mps"])] = (
            round(ask, 3), round(wind, 2))
    assert got == {(6, 0): (6.0, 0.0), (6, 5): (6.0, 5.0),
                   (12, 0): (12.0, 0.0), (12, 5): (12.0, 5.0)}, got


def test_bundle_links_use_forward_slashes_on_every_os():
    """REVIEW (minor). `_show` returned OS-native separators, so the published
    JSON and kpi_report.md carried `demo\\out\\sweep\\episode-...` on Windows,
    and a republish on Linux or CI would have changed all 195 links."""
    assert S._show(ROOT / "demo" / "out" / "sweep" / "x") == "demo/out/sweep/x"
    with tempfile.TemporaryDirectory() as td:
        outside = Path(td) / "a" / "b"
        assert "\\" not in S._show(outside), S._show(outside)


# --------------------------------------------------------------------------- #
# the steppable runner (2026-10-11): one integrator for run_episode and a
# live control plane
# --------------------------------------------------------------------------- #

def _same_run(a, b) -> list[str]:
    """Where two EpisodeRuns differ (wall time aside)."""
    diff = [f for f in ("rows", "extra", "events", "policy_hash_start", "sim_s",
                        "seed_consumed", "priorities")
            if getattr(a, f) != getattr(b, f)]
    if a.policy.policy_hash != b.policy.policy_hash:
        diff.append("policy")
    return diff


def test_the_stepped_runner_flies_exactly_what_run_episode_flies():
    """EpisodeRunner stepped tick by tick must be run_episode: an event cell,
    seeded GPS noise, seeded gusts, a terminal fail-safe that ends the episode
    early (failsafe-rtl-zone: G11 then G9, home reached; until 2026-10-19 this
    was three-simultaneous-violations' X1>G3>X6>G9 at 9.6 s, which now runs
    X1>G3>G5>G9 to 31.9 s), a Loiter whose hold point is moved and flown
    (time-window-opens-on-vehicle) and the Shield-off control arm."""
    early_spec = FAMILIES["failsafe-rtl-zone"].first.spec
    cells = [_cell("dyn-nfz-spawn-on-top", size_m=40, variant=0),
             FAMILIES["gps-noise-beyond-margin"].first.spec,
             FAMILIES["wind-gusts"].first.spec,
             early_spec,
             FAMILIES["time-window-opens-on-vehicle"].first.spec,
             BY_ID["nfz-head-on-control"]]
    for spec in cells:
        ref = S.run_episode(spec, DEFAULTS, 3)
        r = S.EpisodeRunner(spec, DEFAULTS, 3)
        steps, ts = 0, []
        while r.t is not None:
            ts.append(r.t)
            r.step()
            steps += 1
        assert r.done and r.step() is False and r.t is None, spec.scenario_id
        got = r.finish()
        assert steps == len(ref.rows) == len(got.rows), (spec.scenario_id, steps, len(ref.rows))
        assert all(abs(a - row["t"]) < 1e-6 for a, row in zip(ts, ref.rows)), \
            (spec.scenario_id, ts[:3])
        assert not _same_run(ref, got), (spec.scenario_id, _same_run(ref, got))
    early = S.run_episode(early_spec, DEFAULTS, 3)
    assert early.extra["terminated_s"] is not None, "the early-ending case did not end early"


def test_an_event_injected_between_ticks_flies_as_the_same_event_written_in():
    """The live hook: dyn-nfz-spawn-on-top's zone (spawned ON the aircraft at
    5 s, so where it lands depends on where the aircraft is) injected while
    the episode is paused before the 5 s tick flies exactly the rows of the
    scenario with the event written in. The null: the same episode with no
    injection flies differently, so the comparison could fail."""
    spec = _cell("dyn-nfz-spawn-on-top", size_m=40, variant=0)
    assert len(spec.events) == 1 and spec.events[0].at_sim_t == 5.0, spec.events
    written = S.run_episode(spec, DEFAULTS, 0)
    bare = spec.model_copy(update={"events": []})
    r = S.EpisodeRunner(bare, DEFAULTS, 0)
    while r.t < 5.0 - 1e-9:
        r.step()
    r.inject(spec.events[0].model_dump(mode="json"))     # a dict is validated
    live = r.finish()
    assert not _same_run(written, live), _same_run(written, live)
    assert live.extra["harness_events"] == 1, live.extra["harness_events"]
    null = S.run_episode(bare, DEFAULTS, 0)
    assert null.rows != written.rows, "the zone changed nothing: the test proves nothing"


def test_an_injected_event_joins_the_schedule_after_one_already_due_then():
    """Two events at the same instant fly in the order a scenario lists them;
    an injected one counts as listed last, so it lands after one already
    scheduled for that instant - the order run_episode would give it."""
    spec = _cell("dyn-nfz-spawn-on-top", size_m=40, variant=0)
    second = {"at_sim_t": 5.0, "type": "spawn_polygon_fence",
              "payload": {"id": "dyn-nfz-2", "width_m": 10, "height_m": 10,
                          "ahead_of_vehicle_m": 30}}
    both = spec.model_copy(update={"events": list(spec.events)
                                   + [S.ScenarioEvent.model_validate(second)]})
    written = S.run_episode(both, DEFAULTS, 0)
    r = S.EpisodeRunner(spec, DEFAULTS, 0)
    r.inject(second)                                     # before any tick flew
    live = r.finish()
    assert not _same_run(written, live), _same_run(written, live)
    ids = [e.get("rule_id") for e in live.events
           if e.get("type") == "hot_apply" and e.get("event") == "spawn_polygon_fence"]
    assert ids == ["dyn-nfz", "dyn-nfz-2"], ids


def test_the_runner_refuses_an_event_it_cannot_fly_as_written():
    """Refused like a scenario's own event: in the past (that tick flew), after
    the last tick (never fires), a window with no clock, and anything after the
    episode ended."""
    spec = _cell("dyn-nfz-spawn-on-top", size_m=40, variant=0)
    ev = {"type": "spawn_polygon_fence",
          "payload": {"id": "late", "width_m": 10, "height_m": 10, "ahead_of_vehicle_m": 30}}
    r = S.EpisodeRunner(spec, DEFAULTS, 0)
    for _ in range(10):
        r.step()
    assert abs(r.t - 10 * float(spec.dt or DEFAULTS.dt)) < 1e-9, r.t
    cases = [({**ev, "at_sim_t": r.t - 0.5}, "in the past"),
             ({**ev, "at_sim_t": r.t_end + 1.0}, "never fires"),
             ({"at_sim_t": r.t + 1.0, "type": "deactivate_rule",
               "payload": {"rule_id": "nfz-square",
                           "window": {"days": ["Mon"], "start_time": "12:00",
                                      "end_time": "12:30"}}},
              "needs mission.clock_start")]
    for bad, why in cases:
        try:
            r.inject(bad)
        except ValueError as e:
            assert why in str(e), (why, str(e))
        else:
            raise AssertionError(f"accepted an event that is {why}: {bad}")
    r.inject({**ev, "at_sim_t": r.t})                    # the tick about to fly: fine
    r.finish()
    try:
        r.inject({**ev, "at_sim_t": 1.0})
    except RuntimeError as e:
        assert "ended" in str(e), str(e)
    else:
        raise AssertionError("accepted an event after the episode ended")



def test_a_runner_whose_episode_raised_fails_fast_instead_of_hanging():
    """A tick that raises ends the episode. Before 2026-10-12 the dead
    generator left the runner `done` False for ever: step() returned True,
    inject() raised TypeError and finish() spun without end, so a live caller
    that caught the error and called finish() hung its thread. Here a second
    zone with dyn-nfz-spawn-on-top's id is injected at 6 s; the Shield refuses
    the duplicate when that tick flies (as it would refuse the same event
    written in). That step() raises the refusal; every later call raises
    RuntimeError naming it, at once (a watchdog thread bounds the wait, so a
    regression fails here instead of hanging the suite)."""
    import threading
    spec = _cell("dyn-nfz-spawn-on-top", size_m=40, variant=0)
    dup = {"at_sim_t": 6.0, "type": "spawn_polygon_fence",
           "payload": {"id": "dyn-nfz", "width_m": 10, "height_m": 10,
                       "ahead_of_vehicle_m": 30}}
    r = S.EpisodeRunner(spec, DEFAULTS, 0)
    while r.t < 6.0 - 1e-9:
        r.step()
    r.inject(dup)                       # accepted: the refusal is the Shield's
    try:
        r.step()
    except ValueError as e:
        first = str(e)
        assert "already exists" in first, first
    else:
        raise AssertionError("the duplicate zone id was not refused when its tick flew")

    later: dict[str, str] = {}

    def call_again():
        for name, fn in (("finish", r.finish), ("step", r.step),
                         ("inject", lambda: r.inject({**dup, "at_sim_t": 7.0}))):
            try:
                fn()
                later[name] = "returned"
            except RuntimeError as e:
                later[name] = f"RuntimeError: {e}"
            except Exception as e:                       # noqa: BLE001
                later[name] = f"{type(e).__name__}: {e}"

    th = threading.Thread(target=call_again, daemon=True)
    th.start()
    th.join(10.0)
    assert not th.is_alive(), f"a call after the error hung (got so far: {later})"
    for name in ("finish", "step", "inject"):
        got = later.get(name, "")
        assert got.startswith("RuntimeError: the episode failed in the tick at 6.0 s")             and "already exists" in got, (name, got)
    assert r.done and r.t is None and r.result is None, (r.done, r.t, r.result)
    assert isinstance(r.error, ValueError), repr(r.error)



def test_the_episode_bundle_carries_the_shields_audit():
    """Audit card WP4-20 (review of 29 Oct): the Stress Testing harness's
    bundles carried no Shield audit; only the flight rails packed audit.jsonl.
    The headless episode now keeps the flying Shield's audit records
    (EpisodeRun.audit, the lines a rail writes) and write_bundle packs them as
    shield_audit.jsonl. Each line validates as an AuditRecord, carries the
    episode's policy hash, and a head-on NFZ episode logs its violations."""
    from guardrail.audit import AuditRecord
    run = S.run_episode(BY_ID["nfz-head-on"], DEFAULTS, 0)
    assert run.audit, "a head-on NFZ episode logged no audit record"
    for line in run.audit:
        AuditRecord.model_validate(line)
    assert {a["policy_hash"] for a in run.audit} <= {
        run.policy_hash_start, run.policy.policy_hash}
    assert any(a["violations"] for a in run.audit)
    d = Path(tempfile.mkdtemp())
    try:
        S.write_bundle(d / "ep", {"m": 1}, {"k": 1}, run.events, None, audit=run.audit)
        lines = (d / "ep" / "shield_audit.jsonl").read_text(encoding="utf-8").splitlines()
        assert len(lines) == len(run.audit)
        assert json.loads(lines[0]) == json.loads(json.dumps(run.audit[0]))
        # no audit given: no file (a caller that has none says nothing)
        S.write_bundle(d / "ep2", {"m": 1}, {"k": 1}, run.events, None)
        assert not (d / "ep2" / "shield_audit.jsonl").exists()
    finally:
        shutil.rmtree(d, ignore_errors=True)

if __name__ == "__main__":
    # A test that cannot run on this host returns SKIP, counted apart, never
    # as a pass (follow-up #84; the pattern of tests/test_bundle.py).
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
