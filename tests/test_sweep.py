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
import re
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


def _flag_names(cmd):
    """The option names of a command line, `--flag=value` read as `--flag`."""
    return {a.split("=", 1)[0] for a in cmd if a.startswith("--")}


def _val(cmd, flag):
    """A flag's value, given as `--flag=value` or `--flag value`."""
    for i, a in enumerate(cmd):
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
        if a == flag:
            return cmd[i + 1]
    raise KeyError(flag)


def test_every_sitl_flag_is_one_the_node_accepts():
    """REGRESSION (audit card WP4-04). The backend passed `--out` to
    sitl/run_sitl_demo.py, which has no such argument, so `--backend sitl` could
    only ever die in argparse. It now drives run_ros2_demo.sh, which forwards
    $3.. to ros2_shield_node.py - and to the VLA stub, which reads the pilot's
    flags since 2026-11-06 - checked here flag by flag, each against the node
    it is for (both drop what they do not declare: parse_known_args)."""
    # Every flag, the policy-reference ones included: until the review of
    # 2026-10-12 --dev-yaml and --expected-hash were added to the node's
    # set here, so this check could not catch a flag the node drops.
    node_flags = _argparse_flags(ROOT / S.SITL_NODE)
    stub_flags = _argparse_flags(ROOT / S.SITL_STUB)
    assert "--out" not in node_flags, "the check would not catch the old bug"
    assert "--out" not in _argparse_flags(ROOT / "sitl" / "run_sitl_demo.py")
    script = (ROOT / S.SITL_SCRIPT).read_text(encoding="utf-8")
    assert '"${@:3}"' in script, "run_ros2_demo.sh no longer forwards $3.. to the node"
    assert '"$DIR/ros2_vla_stub_node.py" "${EXTRA[@]}"' in script, "... nor to the stub"
    entry = (ROOT / "deploy" / "docker" / "entrypoint.sh").read_text(encoding="utf-8")
    assert "vla_args=\"$(printf '%q ' \"${extra[@]}\")\"" in entry, "the Orin's stub"
    for sid in ("nfz-head-on", "nfz-head-on-control", "standoff-approach", "speed-cap",
                "dyn-nfz-spawn-ahead", "corridor-curfew-in-hours"):
        cmd = S.build_sitl_cmd(BY_ID[sid], "sweep_t", events="demo/out/_campaign/t/events.json",
                               meta="demo/out/_campaign/t/scenario.json")
        assert cmd[:3] == ["wsl", "bash", S.SITL_SCRIPT], cmd
        assert cmd[3] in ("on", "off") and cmd[4] in ("", "--dynamic"), cmd
        passed = _flag_names(cmd[5:])
        assert passed <= node_flags | stub_flags, \
            f"{sid}: no node takes {passed - node_flags - stub_flags}"
        assert passed & set(S.RAIL_SHIELD_FLAGS) <= node_flags, (sid, passed - node_flags)
        assert passed & set(S.RAIL_STUB_FLAGS) <= stub_flags, (sid, passed - stub_flags)
    assert "--subject" in _flag_names(S.build_sitl_cmd(BY_ID["standoff-approach"], "t"))
    assert S.sitl_node_flags() == node_flags
    assert S.sitl_node_flags(ROOT / S.SITL_STUB) == stub_flags


def test_the_rails_refuse_only_what_they_cannot_realise():
    """One refusal for both rails (2026-11-06; sitl_refusal and
    _mission_mismatch until then, which refused every scenario but the rail's
    own mission, and a scenario clock the node had taken since 2026-10-12).
    The node now takes the start, the pilot law, the clock and the events:
    what neither rail can realise is refused, with the reason, and the rest
    is flown."""
    for backend in S.RAIL_BACKENDS:
        def why(spec, **kw):
            return S.rail_refusal(spec, backend, **kw) or ""
        # Since 2026-11-21 GPS noise and state latency fly too, applied by
        # the Shield node to its Shield's view (THE PERCEPTION STRESSORS ON
        # THE COMPANION); flown blind they stay headless.
        assert not why(BY_ID["gps-noise-within-margin"]), why(BY_ID["gps-noise-within-margin"])
        assert not why(BY_ID["gps-noise-beyond-margin"]), why(BY_ID["gps-noise-beyond-margin"])
        blind = _with(BY_ID["gps-noise-within-margin"], **{"stress.fix_quality_reported": False})
        assert "fix_quality_reported false" in why(blind), why(blind)
        # Since 2026-11-07 SITL realises wind, its direction, GPS denial and
        # start jitter (tests/test_sitl_stressors.py); gusts stay refused.
        assert not why(BY_ID["gps-dropout"]), why(BY_ID["gps-dropout"])
        assert not why(BY_ID["wind-within-authority"]), why(BY_ID["wind-within-authority"])
        assert "gust_mps" in why(BY_ID["wind-gusts"])
        # Since 2026-11-14 the node's RTL watch feeds G6 and writes
        # rtl_failed per RTL row, so a gate on rtl_failed_ticks is flown;
        # since 2026-11-17 the battery hook too (the simulator side gives
        # the simulated vehicle its battery fail-safe setup), on an
        # opted-in campaign only (RAIL_BATTERY_OPT_IN).
        assert not why(BY_ID["failsafe-rtl-blocked"]), why(BY_ID["failsafe-rtl-blocked"])
        assert "battery_critical_at_s" in why(BY_ID["failsafe-low-battery"])
        assert not why(BY_ID["failsafe-low-battery"], rail_battery=True), \
            why(BY_ID["failsafe-low-battery"], rail_battery=True)
        assert "relabel" in why(BY_ID["standoff-reclassified"])
        assert "non-finite pilot action" in why(BY_ID["nonfinite-action"])
        low = _cell("altitude-recovery-depth", start_up=0.5)
        high = _cell("altitude-recovery-depth", start_up=30.0)
        assert "start altitude 0.5 m" in why(low), why(low)
        assert "start altitude 30 m" in why(high) and "FENCE_ALT_MAX 30 m" in why(high)
        assert not why(_cell("altitude-recovery-depth", start_up=24.0))
        for sid in ("nfz-head-on", "speed-cap", "corridor-curfew-in-hours",
                    "dyn-nfz-spawn-ahead", "standoff-approach", "rail-mission-stub-vla",
                    "corridor-swap-shifted", "time-window-switch-event", "nfz-head-on-control"):
            assert not why(BY_ID[sid]), (backend, sid, why(BY_ID[sid]))
        # What the headless harness refuses too: an event after the run's end.
        spawn = BY_ID["dyn-nfz-spawn-ahead"]
        late = _with(spawn, ticks=int(spawn.events[0].at_sim_t / DEFAULTS.dt))
        assert "after the run's last tick" in why(late, defaults=DEFAULTS), why(late)
    # The arms: a paraphrase or prefix-off arm would re-fly the canonical
    # arm's flight under another id; only the canonical one goes.
    eps, _ = S.expand(LIBRARY, None, 0, ["nfz-head-on"])
    reasons = {e.arm_tag: S.rail_refusal(e.cell.spec, "hil", episode=e) for e in eps}
    assert reasons.pop("canonical,prefix-on") is None and reasons, reasons
    assert all("paraphrase / prefix arm" in r for r in reasons.values()), reasons
    # sitl_refusal is the desktop rail's. (The refused example was the GPS
    # noise cell until 2026-11-21, when the Shield node began to apply it.)
    gusts = BY_ID["wind-gusts"]
    assert S.sitl_refusal(gusts) == S.rail_refusal(gusts, "sitl") and S.sitl_refusal(gusts)
    assert S.sitl_refusal(BY_ID["nfz-head-on"]) is None


def test_the_rail_flies_the_scenarios_own_mission():
    """REVIEW 2026-10-06 (a scenario scored under its id while the rail flew
    something else), kept now that the rail flies the scenario's mission:
    since 2026-11-06 the start (--start, where SITL boots), the target and its
    altitude, the pilot law (--pilot / --action), its speed (the harness's
    4 m/s when the scenario names none), the time cap, the subject TURNED
    with the mission (it was sent unturned), the clock and the events travel
    with the scenario, and SITL_HOME puts the autopilot's home at the start."""
    import random
    from guardrail.projection import LocalProjection
    spec = BY_ID["rail-mission-stub-vla"]
    cmd = S.build_sitl_cmd(spec, "sweep_t", seed=1002, defaults=DEFAULTS)
    assert _val(cmd, "--seed") == "1002" and _val(cmd, "--start") == "0,0,15", cmd
    assert _val(cmd, "--target") == "30,30,15" and _val(cmd, "--pilot") == "stub_vla"
    # 50 s since 2026-11-02 (it flies past the Loiter timeout); on a rail
    # since 2026-11-07 plus the node's 30 s Loiter timeout and 5 s
    # (rail_max_s: the lagging autopilot can enter Loiter where the ideal
    # vehicle does not).
    assert _val(cmd, "--speed") == "6" and _val(cmd, "--max-s") == "85", cmd
    assert S.rail_max_s(50.0) == 85.0 and S.rail_max_s(30.0) == 65.0
    assert _val(cmd, "--reach-m") == "3"
    head = S.build_sitl_cmd(BY_ID["nfz-head-on"], "t", defaults=DEFAULTS)
    assert (_val(head, "--start"), _val(head, "--target"), _val(head, "--pilot"),
            _val(head, "--speed")) == ("-20,15,15", "40,15,15", "goto", "4"), head
    cap = S.build_sitl_cmd(BY_ID["speed-cap"], "t", defaults=DEFAULTS)
    assert "--no-target" in cap and "--target" not in _flag_names(cap)
    assert _val(cap, "--pilot") == "constant" and _val(cap, "--action") == "20,0,0,0"
    curfew = S.build_sitl_cmd(BY_ID["corridor-curfew-in-hours"], "t")
    assert _val(curfew, "--clock-start") == "2026-09-07T09:00:00"
    # A rotation about the start turns the target, and the turned target is
    # what the node is given.
    turned = _with(spec, **{"mission.rotate_deg": 90.0})
    tx, ty, tup = (float(v) for v in _val(S.build_sitl_cmd(turned, "t"), "--target").split(","))
    assert abs(tx + 30.0) < 1e-6 and abs(ty - 30.0) < 1e-6 and tup == 15.0, (tx, ty)
    assert not S.rail_refusal(turned)
    ang = _cell("standoff-approach-angles", offset_m=12, subject_class="pedestrian",
                variant=0)
    sx, sy = (float(v) for v in _val(S.build_sitl_cmd(ang, "t"), "--subject").split(","))
    mm = S._resolve_mission(ang, ang.policy.load(ROOT), random.Random(0))
    assert abs(sx - mm.subject[0]) < 1e-5 and abs(sy - mm.subject[1]) < 1e-5
    assert (round(sx, 3), round(sy, 3)) != (ang.mission.subject.x, ang.mission.subject.y)
    ev = S.build_sitl_cmd(BY_ID["dyn-nfz-spawn-ahead"], "t", events="e.json", meta="m.json")
    assert _val(ev, "--events") == "e.json" and _val(ev, "--scenario-meta") == "m.json"
    # SITL_HOME: the policy frame origin's projection of the start, 584 m,
    # facing the start yaw (turned with the mission).
    pol = BY_ID["nfz-head-on"].policy.load(ROOT)
    lat, lon, alt, hdg = (float(v) for v in S.sitl_home(BY_ID["nfz-head-on"], pol).split(","))
    n, e = LocalProjection(*pol.frame_origin).to_local(lat, lon)
    assert abs(n + 20.0) < 0.01 and abs(e - 15.0) < 0.01 and alt == 584.0 and hdg == 0.0
    assert S.sitl_home(turned, pol).endswith(",584,90.00"), S.sitl_home(turned, pol)


def test_the_hil_arguments_parse_in_both_nodes_as_the_harness_means_them():
    """Every argument the hil rail would send for every flyable cell of the
    smoke profile, read by the Shield node's and the VLA stub's own parsers
    (sitl/ros2_shield_node.py, ros2_vla_stub_node.py, no ROS): nothing
    dropped, nothing misread. Shown failing first: the Shield node read the
    stub's `--action=20,0,0,0` as an abbreviation of --action-timeout-s and
    exited 2, so a constant-pilot mission would never have started."""
    sys.path.insert(0, str(ROOT / "sitl"))
    import ros2_shield_node as N
    import ros2_vla_stub_node as V
    from guardrail.scenario_spec import load_profile
    eps, _ = S.expand(LIBRARY, load_profile(ROOT / "experiments" / "profiles" / "smoke.yaml"))
    with tempfile.TemporaryDirectory() as td:
        bundles = _signed_like_bundles(Path(td))
        plan = S.hil_plan(eps, DEFAULTS, S.HilConfig(bundles_dir=str(bundles)), "t")
    fly = [p for p in plan if p["fly"]]
    assert len(fly) >= 35, len(fly)
    for p in fly:
        args = p["args"]
        ns, rest_n = N.make_parser().parse_known_args(args)
        vs, rest_v = V.make_parser().parse_known_args(args)
        assert _flag_names(rest_n) <= set(S.RAIL_STUB_FLAGS) - set(S.RAIL_SHIELD_FLAGS), \
            (p["id"], rest_n)
        assert _flag_names(rest_v) <= set(S.RAIL_SHIELD_FLAGS) - set(S.RAIL_STUB_FLAGS), \
            (p["id"], rest_v)
        V.check_args(vs)
        N.parse_point(ns.start, "--start")
        assert ns.start == vs.start == _val(args, "--start") and ns.seed == vs.seed
        assert (ns.target, ns.no_target) == (vs.target, vs.no_target)
        assert ns.speed == vs.speed and ns.bundle == vs.bundle == _val(args, "--bundle")
        assert ns.expected_hash and ns.scenario_meta.endswith(f"{p['tag']}/scenario.json")
        assert vs.pilot == _val(args, "--pilot") and ns.reach_m in (3.0, N.REACH_M)


def _signed_like_bundles(root: Path) -> Path:
    """bundles/<id>-v<version>.tar.gz of the library's policies in `root`, as
    tools/build_flight_bundles.py names them (written unsigned here:
    flight_bundle_status judges the policy hash, not the signature)."""
    from guardrail.bundle import write_bundle
    from guardrail.manifest import flight_bundle_path
    out = root / "bundles"
    for name in ("sim_demo_policy", "corridor_survey", "sitl_pedestrian", "wgs84_taipei"):
        pol = load_policy(ROOT / "policies" / f"{name}.yaml")
        write_bundle(pol, flight_bundle_path(pol, out), signer=None)
    return out


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
    # The rest goes to the node or to the VLA stub, which reads the pilot's
    # flags (2026-11-06): each one declared by a node that reads it.
    assert _flag_names(cmd[5:]) <= node | _argparse_flags(ROOT / S.SITL_STUB)


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
    # Since 2026-11-10 (docs/RUBRIC-failsafe.md section 7 adopted: E5 read on
    # the flight the grant prescribes) the same RTL is the fail-safe its label
    # expects (E5), the pin is off, and the headless verdict is a pass.
    assert headless[0] == "pass", headless
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
    assert status == "fail" and any("breaches=1" in g for g in gates), gates
    assert any("polygon_entries=1" in g for g in gates), gates
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


def test_the_spawn_on_top_escape_does_not_depend_on_the_rings_vertex_order():
    """dyn-nfz-spawn-on-top@size_m=40: the zone lands centred on the aircraft,
    so every edge is an exact tie for the way out. The same zone flown with
    its ring stored from each start vertex, both windings (past the model's
    canonical ordering), flies the same episode, and leaves through the edge
    ahead in the 5.3 s it took before the canonical ring order of 2026-11-02
    made the escape take a side edge and end in Loiter (fixed 2026-11-03,
    guardrail/shield.py escape_direction)."""
    spec = _cell("dyn-nfz-spawn-on-top", size_m=40, variant=0)
    orig = S._spawn_zone
    flights = []
    try:
        for rev in (False, True):
            for k in range(4):
                def spawn(payload, i, st, heading, _k=k, _rev=rev):
                    z = orig(payload, i, st, heading)
                    pts = list(z.vertices)[::-1] if _rev else list(z.vertices)
                    pts = pts[_k:] + pts[:_k]
                    object.__setattr__(z, "vertices", pts)
                    return z
                S._spawn_zone = spawn
                run = S.run_episode(spec, DEFAULTS, 0)
                flights.append(([(round(r["x"], 6), round(r["y"], 6)) for r in run.rows],
                                _kpi(spec, run.rows, run.extra, run.policy)))
    finally:
        S._spawn_zone = orig
    poses = [f[0] for f in flights]
    assert all(p == poses[0] for p in poses[1:]), "the escape depends on the ring's order"
    res = flights[0][1]
    assert res["time_to_safe_censored"] == 0 and res["breaches"] == 0, res
    assert res["max_time_to_safe_s"] <= 5.5, res["max_time_to_safe_s"]


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


# --------------------------------------------------------------------------- #
# GPS noise / latency on the rails: the Shield node's own injection (2026-11-21)
# --------------------------------------------------------------------------- #

def _node():
    sys.path.insert(0, str(ROOT / "sitl"))
    import ros2_shield_node as N
    return N


def test_the_shield_nodes_noise_and_delay_line_is_deterministic_under_a_seed():
    """sitl/ros2_shield_node.py PerceptionStress, the companion-side GPS
    noise / latency (WP3-19 / WP4-14): inactive it hands back the pose object
    itself and draws nothing (every flight before 2026-11-21); active, the
    pose latency_ticks calls ago (the first one until the line fills) plus
    random.Random(f"{seed}:gps") draws in the order x, y, up, pos_sigma_m =
    sigma, the CURRENT heading - the same sequence for the same seed, another
    for another seed. Bad values are refused."""
    import math
    import random
    N = _node()
    poses = [State(x=-30.0 + 0.4 * i, y=23.6 + 0.01 * i, up=15.0 + 0.02 * i,
                   yaw_deg=float(3 * i)) for i in range(30)]
    off = N.PerceptionStress()
    assert not off.active and all(off.apply(p) is p for p in poses)
    assert off.ticks == 0 and off.draws == 0
    ns = N.make_parser().parse_args([])
    assert ns.gps_noise_m == 0.0 and ns.latency_ticks == 0
    for sigma, lat in ((0.3, 0), (0.3, 2), (0.0, 3), (2.0, 5)):
        a, b = N.PerceptionStress(sigma, lat, 1001), N.PerceptionStress(sigma, lat, 1001)
        rng = random.Random("1001:gps")
        sa = [a.apply(p) for p in poses]
        assert [(s.x, s.y, s.up) for s in sa] == [(s.x, s.y, s.up) for s in map(b.apply, poses)]
        for i, (p, s) in enumerate(zip(poses, sa)):
            old = poses[max(0, i - lat)]
            n = [rng.gauss(0, sigma) if sigma else 0.0 for _ in range(3)]
            assert (s.x, s.y, s.up) == (old.x + n[0], old.y + n[1], old.up + n[2]), (sigma, lat, i)
            assert s.yaw_deg == p.yaw_deg, "the heading must stay the current one"
            assert s.pos_sigma_m == (sigma if sigma else None)
        assert a.ticks == len(poses) and a.draws == (3 * len(poses) if sigma else 0)
        c = N.PerceptionStress(sigma, lat, 1002)
        other = [c.apply(p) for p in poses]
        assert ([(s.x, s.y) for s in other] != [(s.x, s.y) for s in sa]) == bool(sigma)
        summ = a.summary()
        assert summ["applied"] and summ["gps_noise_m"] == sigma and summ["latency_ticks"] == lat
        assert summ["seed"] == 1001 and summ["rng"] == S.RAIL_PERCEPTION_RNG
        assert summ["ticks"] == len(poses)
        row = a.row_fields(sa[-1])
        assert row["shield_pose"] == [round(sa[-1].x, 4), round(sa[-1].y, 4), round(sa[-1].up, 4)]
    for bad in ((-0.1, 0), (math.nan, 0), (math.inf, 0), (0.3, -1), (0.3, 1.5), (0.3, True)):
        try:
            N.PerceptionStress(*bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")


def test_the_node_hands_its_shield_the_pose_the_harness_hands_its_own():
    """The labels of the GPS noise / latency cells were derived on the
    headless harness's degraded view (_episode, "what the Shield sees"): the
    node's PerceptionStress, fed the harness's true poses with the episode's
    seed, hands the Shield exactly the harness's `perceived` pose on every
    tick - noise and latency together, and each alone."""
    N = _node()
    base = _cell("gps-noise-within-margin", pass_y=23.6, sigma=0.3, latency=2)
    for over in ({}, {"stress.latency_ticks": 0}, {"stress.gps_noise_m": 0.0}):
        spec = _with(base, **over)
        rows = S.run_episode(spec, DEFAULTS, 1001).rows
        ps = N.PerceptionStress(spec.stress.gps_noise_m, spec.stress.latency_ticks, 1001)
        for i, r in enumerate(rows):
            seen = ps.apply(State(x=r["x"], y=r["y"], up=r["up"]))
            p = r["perceived"]
            assert (seen.x, seen.y, seen.up) == (p["x"], p["y"], p["up"]), (over, i)


def test_the_rails_hand_the_cells_noise_and_latency_to_the_shield_node_only():
    """rail_node_args (2026-11-21): a GPS noise / latency cell carries
    --gps-noise-m and --latency-ticks, which the Shield node reads and the
    VLA stub drops (its pilot flies the true pose, as the harness's does);
    nothing is set in SITL for them (no stress plan); every other cell's
    argument list has neither flag."""
    N = _node()
    sys.path.insert(0, str(ROOT / "sitl"))
    import ros2_vla_stub_node as V
    cell = _cell("gps-noise-within-margin", pass_y=23.6, sigma=0.3, latency=2)
    for backend in S.RAIL_BACKENDS:
        assert S.rail_refusal(cell, backend, defaults=DEFAULTS) is None
    args = S.rail_node_args(cell, 1001, DEFAULTS, backend="sitl")
    assert "--gps-noise-m=0.3" in args and "--latency-ticks=2" in args, args
    assert S.rail_perception_args(cell) == ["--gps-noise-m=0.3", "--latency-ticks=2"]
    ns, _ = N.make_parser().parse_known_args(args)
    assert ns.gps_noise_m == 0.3 and ns.latency_ticks == 2 and ns.seed == 1001
    _, rest_v = V.make_parser().parse_known_args(args)
    assert {"--gps-noise-m", "--latency-ticks"} <= _flag_names(rest_v), rest_v
    assert S.rail_stress_plan(cell, 1001, DEFAULTS) is None
    lat_only = _with(cell, **{"stress.gps_noise_m": 0.0})
    assert S.rail_perception_args(lat_only) == ["--latency-ticks=2"]
    for sid in ("nfz-head-on", "gps-dropout", "wind-within-authority"):
        plain = S.rail_node_args(BY_ID[sid], 0, DEFAULTS, backend="sitl")
        assert not {"--gps-noise-m", "--latency-ticks"} & _flag_names(plain), (sid, plain)


def _stressed_flight(sigma, lat, seed, n=40):
    """A flight log and metrics.json as the Shield node writes them under
    PerceptionStress(sigma, lat, seed): the rows' true pose (rounded as
    build_row rounds it) and each row's shield_pose."""
    N = _node()
    ps = N.PerceptionStress(sigma, lat, seed)
    rows = []
    for i in range(n):
        st = State(x=-30.0 + 0.4 * i + 1e-4 * i, y=23.6, up=15.0)
        seen = ps.apply(st)
        rows.append({"t": round(0.1 * i, 2), "x": round(st.x, 3), "y": round(st.y, 3),
                     "up": round(st.up, 3), **ps.row_fields(seen)})
    return rows, ({"perception_stress": ps.summary()} if ps.active else {})


def test_rail_stress_check_scores_only_the_cells_own_noise_and_latency():
    """Fail closed (2026-11-21): a GPS noise / latency cell is scored only
    when the Shield node's record (metrics.json perception_stress) is the
    cell's - sigma, latency, the episode's seed, the harness's generator, on
    every row - and every row's shield_pose reproduces from the flown rows
    and that seed. A node that dropped the flags, another sigma, latency or
    seed, a missing or altered shield_pose: problems. A cell without either
    stressor and no record is checked exactly as before (no problem, nothing
    in the summary); a record on such a cell is a problem."""
    cell = _cell("gps-noise-within-margin", pass_y=23.6, sigma=0.3, latency=2)

    def check(spec, rows, metrics, seed=1001):
        return S.rail_stress_check(spec, S.rail_stress_plan(spec, seed, DEFAULTS), None,
                                   metrics, rows, seed)

    rows, met = _stressed_flight(0.3, 2, 1001)
    problems, summ = check(cell, rows, met)
    assert problems == [], problems
    assert summ["perception"]["rows_checked"] == len(rows)
    assert summ["perception"]["max_error_m"] <= S.RAIL_PERCEPTION_TOL_M
    lat_only = _with(cell, **{"stress.gps_noise_m": 0.0})
    r0, m0 = _stressed_flight(0.0, 2, 1001)
    assert check(lat_only, r0, m0)[0] == []
    assert m0["perception_stress"]["pos_sigma_m"] is None

    def why(spec, rows, metrics, seed=1001):
        return " | ".join(check(spec, rows, metrics, seed)[0])

    assert "seed" in why(cell, rows, met, seed=1002)
    assert "recorded no perception_stress" in why(cell, rows, {})
    r2, m2 = _stressed_flight(0.2, 2, 1001)
    assert "gps_noise_m" in why(cell, r2, m2)
    r3, m3 = _stressed_flight(0.3, 1, 1001)
    assert "latency_ticks" in why(cell, r3, m3)
    assert "not the cell's draw" in why(cell, r0, m0 | {"perception_stress": dict(
        m0["perception_stress"], gps_noise_m=0.3, pos_sigma_m=0.3, draws=3 * len(r0))})
    tampered = copy.deepcopy(rows)
    tampered[17]["shield_pose"][1] += 0.05
    assert "not the cell's draw" in why(cell, tampered, met)
    assert "tick(s)" in why(cell, rows[:-1], met)
    unposed = copy.deepcopy(rows)
    del unposed[5]["shield_pose"]
    assert "no readable shield_pose" in why(cell, unposed, met)
    wrong_rng = {"perception_stress": dict(met["perception_stress"], rng="numpy")}
    assert "drew from" in why(cell, rows, wrong_rng)
    # A cell without either stressor: as before, unless a record says the
    # Shield saw another pose than the cell's.
    plain = BY_ID["nfz-head-on"]
    flat = [{"t": r["t"], "x": r["x"], "y": r["y"], "up": r["up"]} for r in rows]
    problems, summ = check(plain, flat, {}, seed=0)
    assert problems == [] and "perception" not in summ, (problems, summ)
    assert "sets none" in why(plain, rows, met, seed=0)


def test_true_state_violations_are_counted_apart_from_the_shields_own():
    """Under noise the Shield's log can be clean while the flown action was
    illegal where the aircraft really was. Both are reported; neither replaces
    the other. Measured blind (the rail's accuracy report withheld, as on
    every rail that reports none): with it reported, X8 returns the aircraft
    before the noise can carry it anywhere (2026-11-06)."""
    spec = _with(_cell("gps-noise-beyond-margin", pass_y=23.6, sigma=2.0, latency=10),
                 **{"stress.fix_quality_reported": False})
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
    assert "gps_dropout" not in run.events[0], "the default model's events changed"
    assert "model" not in next(e for e in run.events if e["type"] == "gps_dropout_start")


def test_on_a_rail_a_gps_dropout_hands_the_shield_the_autopilots_estimate():
    """How the rails realise a GPS denial (GPS_DROPOUT_MODELS "dead_reckoned",
    2026-11-12): SITL's GPS stops and EKF3 keeps estimating on its IMU, so the
    pose the Shield is handed keeps moving and lags the true one by the
    estimator's drift, RAIL_EKF_DRIFT_MPS x the time since the loss, along
    the ground track at the loss - measured on the rig's 30 GPS denials
    against SITL's own truth (SIMSTATE): 1.47 m at most after 6.5 s, along
    the track. The Shield is told the fix is gone (fix_ok False) as with the
    frozen pose; the fix returns exactly at the window's end. RAIL_VEHICLE
    flies it, so the rail labels (tools/failsafe_rubric.py --response rail)
    are derived on the dropout the rails fly, not on the frozen last fix. An
    unknown model is refused before anything flies."""
    import math
    assert S.RAIL_VEHICLE["gps_dropout"] == "dead_reckoned" == S.GPS_DROPOUT_MODELS[1]
    assert S.DEFAULT_GPS_DROPOUT == "frozen" and S.RAIL_EKF_DRIFT_MPS == 0.23
    spec = _with(_cell("gps-dropout", lane_y=23.6, window=[2.0, 9.0]), escalation=False)
    run = S.run_episode(spec, DEFAULTS, 0, gps_dropout="dead_reckoned")
    inside = [r for r in run.rows if 2.0 - 1e-9 <= r["t"] < 9.0 - 1e-9]
    assert len(inside) == 70 == run.extra["gps_dropout_ticks"]
    seen = [(r["perceived"]["x"], r["perceived"]["y"]) for r in inside]
    assert seen[-1][0] - seen[0][0] > 5.0, "the estimate froze: that is the other model"
    # Along the ground track at the loss (the lane: +x), never across it, even
    # after the Shield's repair turns the aircraft off the lane.
    k = run.rows.index(inside[0])
    gx, gy = run.rows[k]["x"] - run.rows[k - 1]["x"], run.rows[k]["y"] - run.rows[k - 1]["y"]
    ux, uy = gx / math.hypot(gx, gy), gy / math.hypot(gx, gy)
    assert ux > 0.99, (ux, uy)
    turned = max(abs(r["y"] - inside[0]["y"]) for r in inside)
    assert turned > 0.5, "the repair never turned the aircraft: the direction is untested"
    for r in inside:
        lag = S.RAIL_EKF_DRIFT_MPS * (r["t"] - inside[0]["t"])
        ex, ey = r["x"] - r["perceived"]["x"], r["y"] - r["perceived"]["y"]
        assert math.isclose(math.hypot(ex, ey), lag, abs_tol=1e-6), (r["t"], lag, ex, ey)
        assert abs(ex * uy - ey * ux) < 1e-6, (r["t"], ex, ey)
    assert 1.5 < math.hypot(inside[-1]["x"] - inside[-1]["perceived"]["x"],
                            inside[-1]["y"] - inside[-1]["perceived"]["y"]) < 1.6
    after = [r for r in run.rows if r["t"] >= 9.0 - 1e-9]
    assert all(r["perceived"]["x"] == r["x"] for r in after), "the fix never came back"
    start = next(e for e in run.events if e["type"] == "gps_dropout_start")
    assert start["model"] == "dead_reckoned", start
    rec = run.events[0]["gps_dropout"]
    assert rec["model"] == "dead_reckoned" and rec["drift_mps"] == S.RAIL_EKF_DRIFT_MPS, rec
    assert "SIMSTATE" in rec["source"]
    # And the autopilot's own verdict on that estimate (State.estimator_ok,
    # 2026-11-12), as the ROS 2 node reads EKF_STATUS_REPORT: True until EKF3's
    # 6.8 s plus the autopilot's 1 s failsafe rule, past every 7 s window.
    assert rec["estimator_valid_s"] == S.RAIL_EKF_POS_VALID_S == 6.8 + 1.0, rec
    assert "EKF_STATUS_REPORT" in rec["estimator_ok"]
    assert "estimator_ok" not in S.gps_dropout_for("frozen")
    assert S.gps_dropout_for("frozen")["model"] == "frozen"
    try:
        S.run_episode(spec, DEFAULTS, 0, gps_dropout="ekf")
    except ValueError as e:
        assert "gps dropout model" in str(e), e
    else:
        raise AssertionError("an unknown dropout model was flown")


def test_gps_dropout_on_the_head_on_lane_is_a_measured_polygon_entry():
    """The finding the family is TRACKED for, pinned so it cannot change
    silently: the Shield trusts a frozen pose and lets the aircraft into the
    zone, while its own re-check stays clean (it repaired against the pose it
    was given). If this stops holding, the description in scenarios.yaml is
    wrong and must be re-measured. Re-measured 2026-10-12 at the grant's 5 s
    forecast: the entry is the 4-11 s window's (frozen still at speed, 24 m
    out); the old 6-11 s window freezes a pose the Shield already holds.
    Since 2026-11-06 the modelled rail reports the stale pose by default
    (stress.fix_quality_reported) and X8 returns the aircraft before the
    zone, so this finding is measured with the report WITHHELD: the Shield
    flying blind, as on every rail that reports nothing."""
    blind = {"stress.fix_quality_reported": False}
    spec = _with(_cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0]), **blind)
    run = S.run_episode(spec, DEFAULTS, 0)
    res = _kpi(spec, run.rows, run.extra, run.policy)
    assert res["p0_escapes"] == 0, "the Shield's own re-check should be clean"
    assert run.extra["ticks_inside_fence_polygon"] > 0
    assert run.extra["min_true_dist_to_fence_m"] < -5.0, run.extra["min_true_dist_to_fence_m"]
    short = _with(_cell("gps-dropout", lane_y=15.0, window=[2.0, 4.0]), **blind)
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
        # WP1-25: the sweep names the interpreter that drove and scored it.
        import platform
        assert doc["_runner"]["python"] == platform.python_version(), doc.get("_runner")
        assert doc["_runner"]["env"] == Path(sys.prefix).name, doc["_runner"]
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
    the pins the library still carries: standoff-approach-angles' 7 m
    pedestrian cell (a cell pin on the goal), the corridor-swap-shifted RTL
    that ends outside the swapped corridor (two corridor gates) and the
    standoff wedge (the goal). Since 2026-11-10 (docs/RUBRIC-failsafe.md
    section 7 adopted: E5 read on the flight the grant prescribes) the rail
    mission's pin on its label is off - its fail-safe is the one its label
    expects - and the 7 m cell's pin keeps only the goal."""
    cases = (("standoff-approach-angles", {"offset_m": 7, "subject_class": "pedestrian",
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
    "tracked", counted apart, with the incursion in the true-state summary.
    Flown blind (the rail's fix report withheld, 2026-11-06): reported, X8
    returns the aircraft before the zone."""
    fam = _raw("gps-dropout", parameter_sweep={"lane_y": [15.0], "window": [[4.0, 11.0]]})
    fam["stress"] = {**fam["stress"], "fix_quality_reported": False}
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
    own (time_to_safe's business); a frozen fix lets it fly in (an entry) -
    flown blind, the rail's fix report withheld (2026-11-06)."""
    on_top = S.run_episode(_cell("dyn-nfz-spawn-on-top", size_m=40, variant=0),
                           DEFAULTS, 0).extra
    assert on_top["ticks_inside_fence_polygon"] > 0 and on_top["polygon_entries"] == 0, on_top
    flown = S.run_episode(_with(_cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0]),
                                **{"stress.fix_quality_reported": False}),
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
    on this host, or it refuses a scenario it cannot realise - and neither
    launches anything. Since 2026-11-06 the rail flies speed-cap, its own
    mission no longer being the only one, so the probe is an action the
    rail cannot fly: a flyable one here would start SITL in WSL.)"""
    assert S.rail_refusal(BY_ID["nonfinite-action"], "sitl"), "the probe must stay refused"
    rc, doc = _sweep_lib([_raw("nonfinite-action")], "--backend", "sitl", "--seed", "1001")
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
    through the zone this test needs is flown with the watch silenced.
    Re-pinned 2026-11-03: the cell's zone lands EXACTLY centred on the
    aircraft, and which edge its escape took was the ring's vertex order's
    choice; the Shield now breaks that tie toward the commanded heading
    (shield.escape_direction), so the pilot's forward command leaves through
    the edge ahead and no FSM edge is taken. An escape the forward command
    does not already fly, which this test was written on (the ring order's
    choice was not the edge ahead), is recreated by spawning the same zone
    0.5 m to the side: no tie, and the nearest edge is the side one.
    Re-pinned 2026-11-06: on the default recovery law ("cap_all") that side
    exit flies at the 4 m/s cap and is out in 2.6 s, inside X6's T, so no
    RTL happens at all; the RTL this test is about is flown on the law it
    was written on ("cap": the 2.0 m/s floor), which stays selectable.
    Since 2026-11-17 the spawned zone is enforced at the micrometre view of
    its stored WGS84 (Shield._in_frame binds it to the frame), its edge
    distances up to about 1.7 um apart; the tie is judged at that
    resolution (shield._TIE_BOUND_M, 2e-6 m, for a dynamic_nfz stored in
    WGS84), so the centred spawn is still a tie and the premise holds."""
    from guardrail.fsm import FSMConfig

    class NoProgressTest(S.Shield):
        def __init__(self, policy, *a, escalation=True, **k):
            if escalation is True:
                escalation = FSMConfig(x6_min_escape_mps=1e9)
            super().__init__(policy, *a, escalation=escalation, **k)

    spec = _cell("dyn-nfz-spawn-on-top", size_m=20, variant=0)
    centred = S.run_episode(spec, DEFAULTS, 0)
    assert not (centred.extra.get("fsm") or {}).get("transitions"), \
        "the centred spawn's forward command should already be the way out"
    orig_spawn = S._spawn_zone

    def beside(payload, k, st, heading):
        z = orig_spawn(payload, k, st, heading)
        sx, sy = -heading[1] * 0.5, heading[0] * 0.5          # 0.5 m to the left
        object.__setattr__(z, "vertices", [type(v)(x=v.x + sx, y=v.y + sy)
                                           for v in z.vertices])
        return z
    S._spawn_zone = beside
    try:
        default = S.run_episode(spec, DEFAULTS, 0, recovery="cap")
    except BaseException:
        S._spawn_zone = orig_spawn
        raise
    got = ">".join(t["edge"] for t in (default.extra.get("fsm") or {}).get("transitions") or [])
    assert got == "G1>G7", f"the shipped X6 should leave this exit alone: {got!r}"
    real = S.Shield
    S.Shield = NoProgressTest
    try:
        watched = S.run_episode(spec, DEFAULTS, 0, recovery="cap")
        assess = S.RP.RTLWatch.assess

        def silent(self, *a, **k):
            v = assess(self, *a, **k)
            return S.RP.RTLVerdict(False, (), v.leg, False, False)
        S.RP.RTLWatch.assess = silent
        try:
            run = S.run_episode(spec, DEFAULTS, 0, recovery="cap")
        finally:
            S.RP.RTLWatch.assess = assess
    finally:
        S.Shield = real
        S._spawn_zone = orig_spawn
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
        # the first one's edit. It states a frame origin: since 2026-11-18
        # (WP1-14) an edit's metres are written as WGS84 about it, and a
        # policy with no frame origin refuses them.
        return policy_from_raw({"policy_id": "drift",
                                "origin": {"lat": 35.6895, "lon": 139.6917},
                                "constraints": [
            {"id": "z", "type": "dynamic_nfz",
             "scope": "mission", "constraint_type": "hard", "violation_action": "project_fix", "priority": "P0",
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
    # One injected event. (harness_events counts every event of the episode;
    # since IR schema 3.0.0 stores a zone's ring in canonical order, this
    # cell's escape from the zone spawned on the aircraft - an exact tie -
    # also passes a hold point at 15.6 s, in both runs alike.)
    spawned = [e for e in live.events if e.get("type") == "hot_apply"]
    assert len(spawned) == 1, spawned
    assert live.extra["harness_events"] == written.extra["harness_events"]
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

# --------------------------------------------------------------------------- #
# the hil rail (2026-11-06): each step judged under the previous rules, the
# campaign through its rig, a run accepted by its own records
# --------------------------------------------------------------------------- #

def _step_rows(unsafe, **extra):
    """Rail rows at one place, `unsafe` per row, flown by the Shield."""
    return [dict({"t": 0.1 * i, "x": 0.0, "y": 0.0, "up": 15.0, "unsafe": u,
                  "flown": True, "setpoint": "pass"}, **extra) for i, u in enumerate(unsafe)]


def test_a_rail_step_is_judged_under_the_rules_of_the_instant_before():
    """H3: a step into an unsafe position is a breach only if the position was
    already unsafe under the previous row's rules, zones and rule clock, and
    no rule changed on the row - read from the node's own judgement
    (`unsafe_prev_rules`, `zones_in_prev`, `rule_events`), else rebuilt from
    the run's generations at the previous row's clock. Until then every
    false -> true step after the first row was the flight's own: a zone
    spawned onto the aircraft read as a breach (the null below)."""
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    rows = _step_rows([False, False, True, True])
    # The node's judgement: the step on row 2 was already unsafe before.
    made = [dict(r, unsafe_prev_rules=(["nfz-square"] if i == 2 else []), zones_in_prev=[],
                 rule_events=[], zones_in=[], zone_d_m=None) for i, r in enumerate(rows)]
    assert S.rail_measurements(made, pol)["breaches"] == 1
    # ...or the rules changed under it (a spawn on row 2): imposed.
    imposed = [dict(r, rule_events=(["scenario event 0: spawn_polygon_fence"] if i == 2 else []))
               for i, r in enumerate(made)]
    assert S.rail_measurements(imposed, pol)["breaches"] == 0
    # ...or it was safe a tick earlier (a window opening over it): imposed.
    opened = [dict(r, unsafe_prev_rules=[]) for r in made]
    assert S.rail_measurements(opened, pol)["breaches"] == 0
    # An older log, rules unchanged and nothing timed: its own reading.
    assert S.rail_measurements(rows, pol)["breaches"] == 1
    # An older log whose rules changed (two generations): rebuilt from the
    # generations - the spawned zone is not where the aircraft came from -
    # and without them, not measured (never 0).
    from guardrail.shield import as_dynamic_nfz
    from guardrail.models import PolygonFence, XY
    g1 = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    sh = Shield(g1)
    sh.filter(State(x=-50.0, y=-50.0, up=15.0), Action4D())
    sh.hot_apply(as_dynamic_nfz(PolygonFence(
        id="dyn-top", type="polygon_fence", constraint_type="hard", priority="P0",
        violation_action="project_fix", scope="mission", margin_m=1.0,
        vertices=[XY(x=-5, y=-5), XY(x=5, y=-5), XY(x=5, y=5), XY(x=-5, y=5)])))
    gens = {0: pol, 1: g1}
    old = [dict(r, generation=(1 if i >= 2 else 0)) for i, r in enumerate(rows)]
    m = S.rail_measurements(old, pol, gens)
    assert m["breaches"] == 0 and m["polygon_entries"] == 0, m
    assert m["ticks_inside_fence_polygon"] == 2, m
    unmeasured = S.rail_measurements(old, pol)
    assert "breaches" not in unmeasured and unmeasured["rail_measurements_not_measured"]
    # The null: the same log read as one generation counts the spawn.
    assert S.rail_measurements([dict(r, generation=0) for r in old], pol)["breaches"] == 1


class _FakeRig:
    """A stand-in hil rig for run_hil: no WSL, no ssh. Its "Orin" flies the
    scenario it is handed (the meta file it was pushed names the cell) on
    the headless harness, and writes the run folder the Shield node would,
    rows in the rail's shape, with the scenario record it was handed; fetch
    copies it into the store. Every call is recorded. `nonce` / `finish`
    make it write another launch's record or an unfinished episode."""

    def __init__(self, cfg, nonce=None, finish=True):
        self.cfg, self.nonce, self.finish = cfg, nonce, finish
        self.calls: list[tuple] = []
        self.remote = Path(tempfile.mkdtemp(prefix="fake_orin_"))

    def restart_sitl(self, home, stress=None):
        # This rig has no simulator side: a stress plan is recorded and
        # nothing is set (tests/test_sitl_stressors.py has a rig that does).
        self.calls.append(("restart_sitl", home))
        if stress is not None:
            self.calls.append(("stress", str(stress[0])))
        return "HIL_DESKTOP READY"

    def stop_sitl(self):
        self.calls.append(("stop_sitl",))

    def push(self, local_dir, remote_rel):
        self.calls.append(("push", remote_rel))
        shutil.copytree(local_dir, self.remote / remote_rel)

    def restart_mavros(self):
        self.calls.append(("restart_mavros",))

    def run_mission(self, args, tag, shield, timeout_s):
        self.calls.append(("run_mission", tag, shield, list(args), timeout_s))
        meta = json.loads((self.remote / _val(args, "--scenario-meta")).read_text(
            encoding="utf-8"))
        cell = next(c for c in FAMILIES[meta["scenario_id"]].cells(samples=3)
                    if c.episode_id == meta["cell"])
        run = S.run_episode(cell.spec, DEFAULTS, meta["seed"])
        out = self.remote / "demo" / "out" / tag
        out.mkdir(parents=True)
        ep = 1793968079901861321
        (out / "flight_log.jsonl").write_text("".join(
            json.dumps(S._json_safe(dict(r, flown=r.get("owner") != "autopilot",
                                         # what the rail sent: a speed is the same
                                         # in either frame
                                         emitted_body=r["emitted"], episode_id=ep)))
            + "\n" for r in run.rows),
            encoding="utf-8")
        scen = dict(meta, run_nonce=self.nonce) if self.nonce else meta
        (out / "metrics.json").write_text(json.dumps({
            "episode_id": ep, "shield": shield, "reached": run.extra["reached_goal"],
            "fsm": run.extra.get("fsm"), "scenario": scen,
            "scenario_family": meta["family"]}), encoding="utf-8")
        ev = [{"kind": "episode_start", "episode_id": ep}]
        if self.finish:
            ev.append({"kind": "mission_end", "why": "fake"})
        (out / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in ev),
                                          encoding="utf-8")
        (out / "policy_g0.json").write_bytes(run.policy.canonical_bytes())
        return 0, f"fake mission {tag}"

    def fetch(self, tag, local_parent):
        local_parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(self.remote / "demo" / "out" / tag, local_parent / tag)
        return local_parent / tag


def _hil_campaign(argv, td: Path, **rig_kw):
    """main() --backend hil on a fake rig; (rc, results doc, the rig)."""
    cfg = td / "hil.json"
    cfg.write_text(json.dumps({"store": str(td / "store"),
                               "bundles_dir": str(_signed_like_bundles(td))}),
                   encoding="utf-8")
    rigs = []
    saved = S.make_hil_transport
    S.make_hil_transport = lambda c: rigs.append(_FakeRig(c, **rig_kw)) or rigs[-1]
    try:
        rc = _sweep(["--backend", "hil", "--hil-config", str(cfg), "--seed", "1001",
                     "--out", str(td / "s.json"), "--no-bundles"] + argv)
    finally:
        S.make_hil_transport = saved
        for d in (ROOT / S.CAMPAIGN_DIR).glob("tst*"):
            shutil.rmtree(d, ignore_errors=True)
        for d in (ROOT / S.RAIL_STRESS_DIR).glob("tst*"):
            shutil.rmtree(d, ignore_errors=True)
    doc = json.loads((td / "s.json").read_text(encoding="utf-8"))
    return rc, doc, rigs[0] if rigs else None


class _RigMissingBundle(_FakeRig):
    """A rig whose Orin lacks the policy bundles (msl_dem_hil, 2026-11-19)."""

    def missing_files(self, rels):
        self.calls.append(("missing_files", list(rels)))
        return [r for r in rels if r.startswith("bundles/")]


def test_a_mission_whose_files_are_not_on_the_orin_is_refused_before_sitl_starts():
    """run_hil asks the rig which of the node's --bundle / --ground files the
    Orin lacks before anything starts: six msl_dem_hil missions (2026-11-19)
    whose bundles were never copied read 'no events.jsonl (the mission never
    started)' with ros2 launch exiting 0. Now each is an ERROR naming the
    missing file, and SITL, MAVROS and the mission are never started."""
    assert S.orin_input_files(["--bundle", "bundles/a.tar.gz", "--x", "1",
                               "--ground=experiments/worlds/g.json"]) == [
        "bundles/a.tar.gz", "experiments/worlds/g.json"]
    assert S.orin_input_files(["--bundle"]) == []
    saved = _FakeRig
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        cfg = td / "hil.json"
        cfg.write_text(json.dumps({"store": str(td / "store"),
                                   "bundles_dir": str(_signed_like_bundles(td))}),
                       encoding="utf-8")
        rigs = []
        mk = S.make_hil_transport
        S.make_hil_transport = lambda c: rigs.append(_RigMissingBundle(c)) or rigs[-1]
        try:
            _sweep(["--backend", "hil", "--hil-config", str(cfg), "--seed", "1001",
                    "--out", str(td / "s.json"), "--no-bundles", "--only", "speed-cap",
                    "--hil-campaign", "tst9"])
        finally:
            S.make_hil_transport = mk
            for d in (ROOT / S.CAMPAIGN_DIR).glob("tst9*"):
                shutil.rmtree(d, ignore_errors=True)
        doc = json.loads((td / "s.json").read_text(encoding="utf-8"))
    (r,) = [r for r in doc["results"] if r["id"] == "speed-cap"]
    assert r["status"] == "error" and "not on the Orin" in r["error"], r
    assert "bundles/" in r["error"] and "never started" not in r["error"], r
    kinds = [c[0] for c in rigs[0].calls]
    assert kinds == ["missing_files"], kinds
    assert _FakeRig is saved
    # the real transport asks one ssh and reads back the names it echoed
    tr = S.SubprocessTransport.__new__(S.SubprocessTransport)
    tr.cfg = S.HilConfig()
    sent = []
    tr._ssh = lambda cmd, t: (sent.append(cmd) or (0, "bundles/b.tar.gz\nnoise\n"))
    assert tr.missing_files(["bundles/a.tar.gz", "bundles/b.tar.gz"]) == ["bundles/b.tar.gz"]
    assert "[ -f bundles/a.tar.gz ] || echo bundles/a.tar.gz" in sent[0], sent
    assert tr.missing_files([]) == [] and len(sent) == 1


def test_the_hil_backend_flies_each_scenario_through_its_rig_and_scores_it():
    """--backend hil end to end on a fake rig: per flyable scenario SITL is
    restarted at its SITL_HOME, its files go to the Orin, MAVROS restarts,
    the mission runs with the scenario's arguments, its folder is fetched
    into the store, accepted by its own records and scored with the same
    score() / episode_verdict (the headless flight's verdict here), and
    scenario_score.json is written beside it with this launch's nonce. A
    scenario the rail cannot fly is skipped with its reason, never flown.
    Since 2026-11-07 a GPS dropout is flown, with its stress plan handed to
    the desktop side; on this rig, whose simulator side sets nothing, each
    such mission is an ERROR (the stress was not realised), never scored."""
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc, doc, rig = _hil_campaign(["--only", "rail-mission-stub-vla", "speed-cap",
                                      "gps-dropout", "--hil-campaign", "tst1"], td)
        res = {r["id"]: r for r in doc["results"]}
        assert res["speed-cap"]["status"] == "pass", res["speed-cap"]
        spec = BY_ID["rail-mission-stub-vla"]
        hr = S.run_episode(spec, DEFAULTS, 1001)
        headless = S.episode_verdict(spec, S.score(spec, hr.rows, hr.extra, hr.policy,
                                                   hr.priorities), {}, 1001)[0]
        assert res["rail-mission-stub-vla"]["status"] == headless, res["rail-mission-stub-vla"]
        drop = [r for r in res.values() if r["id"].startswith("gps-dropout")]
        assert drop and all(r["status"] == "error" and "stress was not realised" in r["error"]
                            for r in drop), drop
        assert sum(1 for c in rig.calls if c[0] == "stress") == len(drop)
        runs = [c for c in rig.calls if c[0] == "run_mission"]
        assert [c[1] for c in runs][:2] == ["tst1_001_rail-mission-stub-vla",
                                            "tst1_002_speed-cap"]
        assert all("gps-dropout" in c[1] for c in runs[2:]) and len(runs) == 2 + len(drop)
        kinds = [c[0] for c in rig.calls]
        assert kinds[:5] == ["restart_sitl", "push", "restart_mavros", "run_mission",
                             "stop_sitl"], kinds
        pol = BY_ID["speed-cap"].policy.load(ROOT)
        assert ("restart_sitl", S.sitl_home(BY_ID["speed-cap"], pol)) in rig.calls
        assert ("push", f"{S.CAMPAIGN_DIR}/tst1_002_speed-cap") in rig.calls
        args = runs[1][3]
        # The policy's own id and version name its bundle (0.1.3 until IR 4.0.0, 0.1.4 since).
        assert _val(args, "--bundle") == f"bundles/{pol.policy_id}-v{pol.version}.tar.gz"
        assert _val(args, "--expected-hash") == pol.policy_hash
        assert _val(args, "--pilot") == "constant" and "--no-target" in args
        assert runs[1][2] == "on"
        # The mission's timeout is the rail's time cap (rail_max_s: 30 s + the
        # node's 30 s Loiter timeout + 5 s, 2026-11-07) plus the margin.
        assert abs(runs[1][4] - S.rail_max_s(30.0) - S.HilConfig().mission_margin_s) < 1e-6, (
            runs[1][4])
        run_dir = td / "store" / "tst1_002_speed-cap"
        score = json.loads((run_dir / S.SCENARIO_SCORE).read_text(encoding="utf-8"))
        met = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
        assert score["run_nonce"] == met["scenario"]["run_nonce"] and score["status"] == "pass"
        assert score["scenario"]["family"] == "hil-stress/kinematic_envelope/speed-cap/shield-on"
        assert score["metrics"]["shield"] == "on" and "kpi" in score
        assert (run_dir / "mission_console.log").is_file()
        # The desktop's own revision joins the run's stack record (ARCH-31);
        # this fake Orin wrote neither a manifest nor a record of its own.
        stack = json.loads((run_dir / "stack_revisions.json").read_text(encoding="utf-8"))
        assert list(stack["hosts"]) == ["desktop"], stack
        assert stack["hosts"]["desktop"]["code_revision"] == S.desktop_revision()["code_revision"]
        # A run the harness rejected (the gps-dropout missions: stress not
        # realised) is never joined to this launch's desktop.
        rejected = [d for d in (td / "store").iterdir()
                    if d.is_dir() and "gps-dropout" in d.name]
        assert rejected and not any((d / "stack_revisions.json").exists() for d in rejected), (
            [d.name for d in rejected])
        assert res["speed-cap"]["mission_tag"] == "tst1_002_speed-cap"
        assert doc["_backend"] == "hil" and doc["_scope"]["hil"]["campaign"] == "tst1"
        assert rc in (0, 1), rc


GRANT_BUNDLE_NAMES = ("manifest.json", "policy_bundle.tar.gz", "episode.bag2",
                      "shield_audit.jsonl", "harness_events.jsonl", "kpi.json")


def _rail_run_folder(d: Path, tag: str, *, bag: bool = True) -> dict:
    """A flown rail run folder as the Shield node leaves it (its own names:
    bag/, events.jsonl, policy_g<N>.json), with one hot-applied generation.
    Returns {name: bytes} of every file in it."""
    import hashlib
    d.mkdir(parents=True)
    g0 = load_policy(ROOT / "policies" / "sim_demo_policy.yaml").canonical_bytes()
    g1 = load_policy(ROOT / "policies" / "corridor_survey.yaml").canonical_bytes()
    h0, h1 = ("sha256:" + hashlib.sha256(b).hexdigest() for b in (g0, g1))
    (d / "policy_g0.json").write_bytes(g0)
    (d / "policy_g1.json").write_bytes(g1)
    (d / "manifest.json").write_text(json.dumps({
        "code_revision": "0123456789abcdef0123456789abcdef01234567",
        "vla_model_hash": "stub", "policy_hash": h1, "random_seed": 1001,
        "sim_speedup": 1.0, "topology": "hil"}), encoding="utf-8")
    (d / "metrics.json").write_text(json.dumps({"shield": "on"}), encoding="utf-8")
    (d / "trajectory.json").write_text(json.dumps({"t": [0.0, 0.1]}), encoding="utf-8")
    (d / f"{tag}.replay.tar.gz").write_bytes(b"signed replay bundle bytes")
    (d / "shield_audit.jsonl").write_text('{"tick": 0}\n{"tick": 1}\n', encoding="utf-8")
    (d / "kpi.json").write_text('{"the node": "its own"}\n', encoding="utf-8")
    ev = [{"kind": "episode_start", "t": None, "episode_id": 7},
          {"kind": "policy_generation", "t": None, "generation": 0, "policy_hash": h0,
           "reason": "take-off"},
          {"kind": "mission_start", "t": 0.0},
          {"kind": "policy_generation", "t": 3.002, "generation": 1, "policy_hash": h1,
           "reason": "hot_apply bar"},
          {"kind": "scenario_event", "t": 3.002, "type": "hot_apply",
           "event": "spawn_polygon_fence", "at_sim_t": 3.0, "op": "spawn",
           "rule_id": "bar", "generation": 1, "policy_hash": h1},
          {"kind": "mission_end", "t": 41.5, "why": "test"}]
    (d / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in ev),
                                    encoding="utf-8")
    if bag:
        (d / "bag").mkdir()
        (d / "bag" / "bag_0.mcap").write_bytes(b"\x89MCAP0\r\n rosbag2 bytes")
        (d / "bag" / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n",
                                                 encoding="utf-8")
    return {p.relative_to(d).as_posix(): p.read_bytes()
            for p in sorted(d.rglob("*")) if p.is_file()}


def test_a_rail_run_is_bundled_under_the_grants_six_names_from_its_own_files():
    """WP4-11 (2026-11-18): a flown rail episode's bundle carries the grant's
    six names (Stress Testing, "Episode bundle") - manifest.json, a
    policy_bundle.tar.gz holding every generation's IR, episode.bag2/,
    shield_audit.jsonl, harness_events.jsonl and kpi.json - built from the
    run folder, which keeps its own names and bytes. Until then every flown
    KPI campaign (--backend hil) left bag/, events.jsonl and policy_g<N>.json
    only. A run without a bag says so; it is never made up."""
    import tarfile
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = td / "store" / "tst9_001_speed-cap"
        before = _rail_run_folder(run, run.name)
        spec = BY_ID["speed-cap"]
        ep = td / "runs" / "episode-x--speed-cap--seed1001"
        files = S.write_rail_episode_bundle(
            ep, run, spec, 1001, {"id": "speed-cap", "template": spec.template,
                                  "status": "pass", "kpi": {"ticks": 3}},
            backend="hil", tag=run.name, rc=0)
        for name in GRANT_BUNDLE_NAMES:
            assert (ep / name).exists(), name
        assert list(files) == ["manifest.json", "policy_bundle.tar.gz", "episode.bag2/",
                               "shield_audit.jsonl", "harness_events.jsonl", "kpi.json"]
        assert all(v.startswith("written") for v in files.values()), files
        # the run folder is untouched: its own names, its own bytes
        assert {p.relative_to(run).as_posix(): p.read_bytes()
                for p in sorted(run.rglob("*")) if p.is_file()} == before
        assert (ep / "manifest.json").read_bytes() == before["manifest.json"]
        assert (ep / "shield_audit.jsonl").read_bytes() == before["shield_audit.jsonl"]
        for n in ("bag_0.mcap", "metadata.yaml"):
            assert (ep / "episode.bag2" / n).read_bytes() == before[f"bag/{n}"], n
        # every hot-apply generation's IR, checked against the node's hash
        with tarfile.open(ep / "policy_bundle.tar.gz") as tar:
            names = set(tar.getnames())
            idx = json.load(tar.extractfile("policy_bundle.json"))
            assert tar.extractfile("generations/policy_g1.json").read() == \
                before["policy_g1.json"]
        assert {"generations/policy_g0.json", "generations/policy_g1.json"} <= names, names
        assert idx["complete"] is True and [g["generation"] for g in idx["generations"]] \
            == [0, 1], idx
        assert idx["generations"][1]["t"] == 3.002 and \
            idx["generations"][1]["event"] == "spawn_polygon_fence", idx["generations"]
        assert idx["injected"]["member"], idx["injected"]   # the policy the rail was handed
        # harness events in the headless bundle's shape (kpi_report load_stored)
        ev = [json.loads(x) for x in
              (ep / "harness_events.jsonl").read_text(encoding="utf-8").splitlines()]
        assert [e["type"] for e in ev] == ["episode_start", "hot_apply", "episode_end"], ev
        assert ev[0]["shield"] == "on" and ev[0]["generation"] == 0 and ev[0]["backend"] == "hil"
        assert ev[1]["rule_id"] == "bar" and "kind" not in ev[1]
        assert ev[2]["generation"] == 1 and ev[2]["t"] == 41.5 and ev[2]["mission_rc"] == 0
        # what a replay reads, beside; kpi.json is the campaign's, not the node's
        assert (ep / "tst9_001_speed-cap.replay.tar.gz").read_bytes() == \
            before["tst9_001_speed-cap.replay.tar.gz"]
        assert (ep / "trajectory.json").read_bytes() == before["trajectory.json"]
        assert not (ep / "flight_log.jsonl").exists()
        kpi = json.loads((ep / "kpi.json").read_text(encoding="utf-8"))
        assert kpi["status"] == "pass" and kpi["bundle_files"] == files
        assert isinstance(kpi["kpi_grade"], bool) and kpi["kpi_grade_reasons"] is not None
        assert kpi["rail_bundle"]["tag"] == run.name and kpi["rail_bundle"]["episode_bag"][
            "files"][0]["sha256"].startswith("sha256:")
        # no bag on the run: said, never made up
        run2 = td / "store" / "tst9_002_speed-cap"
        _rail_run_folder(run2, run2.name, bag=False)
        ep2 = td / "runs" / "episode-y--speed-cap--seed1001"
        f2 = S.write_rail_episode_bundle(ep2, run2, spec, 1001, {"kpi": {}}, backend="hil")
        assert f2["episode.bag2/"].startswith("not written") and not (ep2 / "episode.bag2").exists()


class _BagRig(_FakeRig):
    """_FakeRig whose mission also leaves what the Shield node's run folder
    holds beside the log: manifest.json, bag/, shield_audit.jsonl."""

    def run_mission(self, args, tag, shield, timeout_s):
        rc, out = super().run_mission(args, tag, shield, timeout_s)
        d = self.remote / "demo" / "out" / tag
        (d / "manifest.json").write_text(json.dumps({
            "code_revision": "0123456789abcdef0123456789abcdef01234567",
            "vla_model_hash": "stub", "policy_hash": "sha256:00", "random_seed": 1001,
            "sim_speedup": 1.0, "topology": "hil"}), encoding="utf-8")
        (d / "bag").mkdir()
        (d / "bag" / "bag_0.mcap").write_bytes(b"rosbag2")
        (d / "shield_audit.jsonl").write_text('{"tick": 0}\n', encoding="utf-8")
        return rc, out


def test_a_hil_campaign_writes_each_flown_episode_under_the_grants_six_names():
    """--backend hil without --no-bundles (WP4-11, 2026-11-18): each flown
    episode gets a bundle in --runs under the grant's six names, the sweep's
    record names it, and its kpi.json carries the campaign's verdict."""
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        cfg = td / "hil.json"
        cfg.write_text(json.dumps({"store": str(td / "store"),
                                   "bundles_dir": str(_signed_like_bundles(td))}),
                       encoding="utf-8")
        saved = S.make_hil_transport
        S.make_hil_transport = lambda c: _BagRig(c)
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                rc = _sweep(["--backend", "hil", "--hil-config", str(cfg), "--seed", "1001",
                             "--only", "speed-cap", "--hil-campaign", "tst8",
                             "--out", str(td / "s.json"), "--runs", str(td / "runs")])
        finally:
            S.make_hil_transport = saved
            for d in (ROOT / S.CAMPAIGN_DIR).glob("tst*"):
                shutil.rmtree(d, ignore_errors=True)
            for d in (ROOT / S.RAIL_STRESS_DIR).glob("tst*"):
                shutil.rmtree(d, ignore_errors=True)
        doc = json.loads((td / "s.json").read_text(encoding="utf-8"))
        r = doc["results"][0]
        assert rc in (0, 1) and r["status"] == "pass", (rc, r.get("status"), r.get("error"))
        ep = Path(r["bundle"])
        ep = ep if ep.is_absolute() else ROOT / ep
        assert ep.parent == td / "runs" and ep.name.startswith("episode-"), ep
        for name in GRANT_BUNDLE_NAMES:
            assert (ep / name).exists(), name
        kpi = json.loads((ep / "kpi.json").read_text(encoding="utf-8"))
        assert kpi["status"] == r["status"] and kpi["mission_tag"] == "tst8_001_speed-cap"
        assert kpi["kpi"] == r["kpi"]
        assert all(kpi["bundle_files"][n].startswith("written") for n in (
            "manifest.json", "policy_bundle.tar.gz", "episode.bag2/", "shield_audit.jsonl",
            "harness_events.jsonl", "kpi.json")), kpi["bundle_files"]
        run = td / "store" / "tst8_001_speed-cap"
        assert (ep / "episode.bag2" / "bag_0.mcap").read_bytes() == \
            (run / "bag" / "bag_0.mcap").read_bytes()
        assert (run / "bag").is_dir() and not (run / "episode.bag2").exists()


def test_a_hil_run_is_accepted_by_its_own_records_not_by_file_times():
    """The fetched folder is this launch's only if its records say so: this
    episode's mission_end, metrics.json and every row of the same episode,
    and the scenario record carrying this launch's nonce. Another launch's
    record, or an episode that never ended, is an error - flown, not
    scored - and never a pass."""
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc, doc, _ = _hil_campaign(["--only", "speed-cap", "--hil-campaign", "tst2"], td,
                                   nonce="another-launch")
        r = doc["results"][0]
        assert r["status"] == "error" and "not this launch's" in r["error"] and rc == 1, r
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc, doc, _ = _hil_campaign(["--only", "speed-cap", "--hil-campaign", "tst3"], td,
                                   finish=False)
        r = doc["results"][0]
        assert r["status"] == "error" and "has no mission_end" in r["error"], r
    with tempfile.TemporaryDirectory() as td:
        run = Path(td) / "x"
        run.mkdir()
        (run / "events.jsonl").write_text(json.dumps({"kind": "episode_start", "episode_id": 1})
                                          + "\n" + json.dumps({"kind": "mission_end"}) + "\n",
                                          encoding="utf-8")
        (run / "metrics.json").write_text(json.dumps(
            {"episode_id": 1, "scenario": {"run_nonce": "n"}}), encoding="utf-8")
        (run / "flight_log.jsonl").write_text(json.dumps({"episode_id": 1}) + "\n"
                                              + json.dumps({"episode_id": 2}) + "\n",
                                              encoding="utf-8")
        try:
            S.accept_hil_run(run, "n")
        except RuntimeError as e:
            assert "rows of other episodes" in str(e), e
        else:
            raise AssertionError("a log of two episodes was accepted")
        (run / "flight_log.jsonl").write_text(json.dumps({"episode_id": 1}) + "\n",
                                              encoding="utf-8")
        rows, met, ep = S.accept_hil_run(run, "n")
        assert len(rows) == 1 and ep["episode_id"] == 1
        (run / "metrics.json").write_text(json.dumps(
            {"episode_id": 9, "scenario": {"run_nonce": "n"}}), encoding="utf-8")
        try:
            S.accept_hil_run(run, "n")
        except RuntimeError as e:
            assert "metrics.json is episode 9" in str(e), e
        else:
            raise AssertionError("another episode's metrics were accepted")


def test_only_the_fsm_edges_profile_opts_in_to_the_battery_cell_on_the_rails():
    """RAIL_BATTERY_OPT_IN (2026-11-17, WP3-07): failsafe-low-battery has
    never flown on the rig and writes BATT_* to the simulated autopilot, so
    it flies on a rail campaign only from a profile with `rail_battery:
    true` - fsm_edges, which bears no KPI. Every KPI profile leaves it off
    (and may not set it), so their campaigns refuse it as before."""
    from guardrail.scenario_spec import Profile, load_profile
    profs = {p.stem: load_profile(p) for p in (ROOT / "experiments" / "profiles").glob("*.yaml")}
    assert [n for n, p in profs.items() if p.rail_battery] == ["fsm_edges"], profs.keys()
    assert profs["fsm_edges"].grant_kpi_bearing is False
    raw = yaml.safe_load((ROOT / "experiments" / "profiles" / "smoke.yaml").read_text(
        encoding="utf-8"))
    try:
        Profile.model_validate({**raw, "rail_battery": True})
    except ValueError as e:
        assert "KPI-bearing" in str(e), e
    else:
        raise AssertionError("a KPI-bearing profile opted in to the battery cell")
    made = []
    saved = S.make_hil_transport
    S.make_hil_transport = lambda c: made.append(c)
    buf = io.StringIO()
    try:
        with tempfile.TemporaryDirectory() as td:
            cfg = Path(td) / "hil.yaml"
            cfg.write_text(yaml.safe_dump({"bundles_dir": str(_signed_like_bundles(Path(td)))}),
                           encoding="utf-8")
            with contextlib.redirect_stdout(buf):
                rc = _sweep(["--profile", "fsm_edges", "--backend", "hil", "--hil-dry-run",
                             "--hil-config", str(cfg), "--hil-campaign", "fe"])
    finally:
        S.make_hil_transport = saved
    out = buf.getvalue()
    assert rc == 0 and not made, (rc, made)
    flown, _, refused = out.partition("\n  refused:")
    assert "_failsafe-low-battery  failsafe-low-battery" in flown, out[:600]
    assert "failsafe-low-battery" not in refused
    assert "SITL setup BATT_LOW_VOLT=10.5, BATT_CRT_VOLT=10.1, BATT_LOW_TIMER=1" in flown
    assert "SIM_BATT_VOLTAGE=10 at 13 s" in flown


def test_the_hil_dry_run_lists_every_mission_and_flies_nothing():
    """--hil-dry-run: every episode with its tag, SITL_HOME and commands, or
    why it is refused, and the counts; no transport is made, no file
    written, exit 0. The refused families are the ones the rail cannot
    realise, each with its reason."""
    made = []
    saved = S.make_hil_transport
    S.make_hil_transport = lambda c: made.append(c)
    before = set((ROOT / S.CAMPAIGN_DIR).glob("*")) if (ROOT / S.CAMPAIGN_DIR).is_dir() else set()
    buf = io.StringIO()
    try:
        with tempfile.TemporaryDirectory() as td:
            cfg = Path(td) / "hil.yaml"
            cfg.write_text(yaml.safe_dump({"bundles_dir": str(_signed_like_bundles(Path(td)))}),
                           encoding="utf-8")
            with contextlib.redirect_stdout(buf):
                rc = _sweep(["--profile", "smoke", "--backend", "hil", "--hil-dry-run",
                             "--hil-config", str(cfg), "--hil-campaign", "dry"])
    finally:
        S.make_hil_transport = saved
    out = buf.getvalue()
    after = set((ROOT / S.CAMPAIGN_DIR).glob("*")) if (ROOT / S.CAMPAIGN_DIR).is_dir() else set()
    assert rc == 0 and not made and after == before, (rc, made, after - before)
    # 41 / 23 until 2026-11-07, when the two GPS-dropout, two wind-within-
    # authority and two wind-beyond-authority cells began to fly; 47 / 17
    # until 2026-11-14, when failsafe-rtl-blocked did (the node's RTL watch
    # feeds G6 and writes the rtl_failed its gate counts). The battery cell
    # (2026-11-17) is opt-in on the rails (RAIL_BATTERY_OPT_IN): the smoke
    # profile does not opt in, so its campaign stayed 48 / 16 - until
    # 2026-11-21, when its three GPS noise / latency cells began to fly
    # (the Shield node applies them to its Shield's view): 51 / 13.
    assert "51 mission(s) to fly, 13 refused" in out, out[:300]
    flown, _, refused = out.partition("\n  refused:")
    assert "_failsafe-rtl-blocked  failsafe-rtl-blocked" in flown, "the blocked return flies"
    assert "failsafe-low-battery" not in flown and "SITL setup" not in flown
    assert "REF  failsafe-low-battery" in refused and "rail_battery: true" in refused
    assert "dry_001_nfz-head-on" in out and "SITL_HOME  -35.36344066,149.16539523,584,0.00" in out
    assert "--start=-20,15,15" in out and "hil_desktop.sh" in out
    # The simulator's stressors, as the desktop side will set them.
    assert "SIM_GPS_DISABLE=1 at 4 s" in out and "SIM_WIND_DIR=180" in out
    assert "--stress-plan" in out and "--stress-record" in out
    # The GPS noise / latency cells fly with the Shield node's flags.
    assert "--gps-noise-m=0.3" in flown and "--latency-ticks=2" in flown
    assert "--gps-noise-m=2" in flown and "--latency-ticks=5" in flown
    assert "gps_noise_m" not in refused and "gps-noise" not in refused
    for why in ("gust_mps", "battery_critical_at_s", "non-finite",
                "start altitude 30 m", "relabel", "paraphrase / prefix arm",
                "Shield-off control arm"):
        assert why in out, why
    # A config key it does not know is refused (exit 2), not ignored.
    with tempfile.TemporaryDirectory() as td:
        bad = Path(td) / "hil.json"
        bad.write_text(json.dumps({"ssh_targte": "x"}), encoding="utf-8")
        assert _sweep(["--only", "speed-cap", "--backend", "hil", "--hil-config", str(bad),
                       "--hil-dry-run"]) == 2
    assert _sweep(["--only", "speed-cap", "--hil-dry-run"]) == 2


def test_the_hil_transport_runs_the_rig_commands_it_names():
    """SubprocessTransport with recording stand-ins for subprocess: the WSL
    helper started with the home and the router's target and waited on for
    its ready line, the files and the run folder copied with scp relative to
    the remote home, the mission run in the hil topology's compose project
    with every argument quoted for the remote shell; a mission that times
    out stops its container."""
    calls = []

    class _Done:
        def __init__(self, rc=0, out=""):
            self.returncode, self.stdout, self.stderr = rc, out, ""

    class _Popen:
        def __init__(self, cmd, **kw):
            calls.append(("popen", cmd))
            self.stdout = iter(["hil_desktop: home: x\n", "HIL_DESKTOP READY sitl_pid=1\n"])

        def terminate(self):
            calls.append(("terminate",))

    def run(cmd, **kw):
        calls.append(("run", cmd, kw.get("cwd"), kw.get("timeout")))
        if "run --rm -T mission" in cmd[-1] and "slow" in cmd[-1]:
            raise subprocess.TimeoutExpired(cmd, kw.get("timeout"))
        return _Done(0, "ok")
    cfg = S.HilConfig(wsl_repo="/mnt/d/repo")
    t = S.SubprocessTransport(cfg, run=run, popen=_Popen)
    assert "HIL_DESKTOP READY" in t.restart_sitl("-35.1,149.2,584,90.00")
    popen = [c for c in calls if c[0] == "popen"][0][1]
    assert popen[:6] == ["wsl.exe", "-d", "Ubuntu", "--", "bash", "/mnt/d/repo/sitl/hil_desktop.sh"]
    assert popen[6] == "start" and _val(popen, "--home") == "-35.1,149.2,584,90.00"
    assert _val(popen, "--mavros-host") == "10.42.0.3"
    t.stop_sitl()
    assert any(c[0] == "run" and c[1][-1] == "stop" for c in calls) and ("terminate",) in calls
    with tempfile.TemporaryDirectory() as td:
        d = Path(td) / "tag1"
        d.mkdir()
        t.push(d, "demo/out/_campaign/tag1")
        scp = [c for c in calls if c[0] == "run" and c[1][0] == "scp"][-1]
        assert scp[1][-2:] == ["tag1", "nathan-agxiot@10.42.0.3:vla-drone/demo/out/_campaign/"]
        assert scp[2] == str(Path(td)), scp
        got = t.fetch("tag1", Path(td) / "store")
        scp = [c for c in calls if c[0] == "run" and c[1][0] == "scp"][-1]
        assert scp[1][-2:] == ["nathan-agxiot@10.42.0.3:vla-drone/demo/out/tag1", "."]
        assert got == Path(td) / "store" / "tag1"
    t.restart_mavros()
    rc, out = t.run_mission(["--start=-200,0,15", "--action=-3,0,0,0", "--tag", "it's"],
                            "tag1", "on", 99.0)
    ssh = [c for c in calls if c[0] == "run" and c[1][0] == "ssh"][-1]
    remote = ssh[1][-1]
    assert remote.startswith("cd ~/vla-drone && docker compose --env-file "
                             "deploy/topologies/hil.env -f "
                             "deploy/compose/docker-compose.orin.yml run --rm -T mission "
                             "mission --shield on --tag tag1 "), remote
    assert "--start=-200,0,15 --action=-3,0,0,0 --tag 'it'\"'\"'s'" in remote, remote
    assert "BatchMode=yes" in ssh[1] and ssh[3] == 99.0 and rc == 0
    rc, out = t.run_mission(["--slow"], "tag2", "on", 5.0)
    assert rc == 124 and "timed out" in out
    assert "xargs -r docker kill" in [c for c in calls if c[0] == "run"][-1][1][-1]


def test_the_hil_transport_hands_the_chain_requirement_to_the_mission_container():
    """Review of the bundle-ca branch (2026-11-16): VLAGUARD_REQUIRE_CHAIN=1
    exported in the desktop's shell before a hil campaign never reached the
    Orin - ssh forwards no variable, and the mission container read hil.env's
    0 - so a mission meant to fly "with the chain required" flew without it,
    while its record still read chain verified. The mission is now launched
    with `-e VLAGUARD_REQUIRE_CHAIN=<0|1>` when the hil config's
    `require_chain` or this desktop's variable asks, and with nothing added
    otherwise (the command is the one every campaign flew); and a mission
    launched with 1 whose own policy_source does not say require_chain true
    and chain verified is not accepted as that flight."""
    sent = []

    class _Done:
        returncode, stdout, stderr = 0, "ok", ""

    def run(cmd, **kw):
        sent.append(cmd)
        return _Done()
    saved = os.environ.pop("VLAGUARD_REQUIRE_CHAIN", None)
    try:
        for cfg_value, env_value, want in (
                (None, None, ""), (None, "", ""),
                (True, None, "-e VLAGUARD_REQUIRE_CHAIN=1 "),
                (None, "1", "-e VLAGUARD_REQUIRE_CHAIN=1 "),
                (None, "0", "-e VLAGUARD_REQUIRE_CHAIN=0 "),
                (False, "1", "-e VLAGUARD_REQUIRE_CHAIN=0 ")):
            if env_value is None:
                os.environ.pop("VLAGUARD_REQUIRE_CHAIN", None)
            else:
                os.environ["VLAGUARD_REQUIRE_CHAIN"] = env_value
            cfg = S.HilConfig(require_chain=cfg_value)
            S.SubprocessTransport(cfg, run=run).run_mission(["--x"], "tag", "on", 5.0)
            remote = sent[-1][-1]
            assert ("docker-compose.orin.yml run --rm -T " + want
                    + "mission mission --shield on --tag tag --x") in remote, \
                (cfg_value, env_value, remote)
            assert S.hil_config_record(cfg)["require_chain_launched"] == \
                (None if not want else want.endswith("=1 ")), (cfg_value, env_value)
    finally:
        os.environ.pop("VLAGUARD_REQUIRE_CHAIN", None)
        if saved is not None:
            os.environ["VLAGUARD_REQUIRE_CHAIN"] = saved
    # The hil config file takes the key.
    p = Path(tempfile.mkdtemp()) / "rig.json"
    p.write_text(json.dumps({"require_chain": True}), encoding="utf-8")
    assert S.HilConfig.load(p).require_chain is True
    # Accepting the run: its own record must show the requirement in force.
    good = {"policy_source": {"signature": "verified", "chain": "verified",
                              "require_chain": True}}
    assert S.hil_chain_problems(good, True) == []
    assert S.hil_chain_problems({}, None) == [] and S.hil_chain_problems({}, False) == []
    lost = {"policy_source": dict(good["policy_source"], require_chain=False)}
    assert any("did not reach the flight loader" in x for x in S.hil_chain_problems(lost, True))
    old = {"policy_source": {"signature": "verified", "chain": "verified"}}   # before the key
    assert S.hil_chain_problems(old, True), "a record without require_chain passed"
    bare = {"policy_source": dict(good["policy_source"], chain="none")}
    assert any("chain 'none'" in x for x in S.hil_chain_problems(bare, True))


# --------------------------------------------------------------------------- #
# the rail scorer (2026-11-07): the rule readings from the rail's own log, the
# scenario's window, the proposed rail gates, the offline re-score
# --------------------------------------------------------------------------- #

_RULE_KEYS = ("phase", "switch_lag_s_", "switch_conservative_", "anticipated_ticks_",
              "anticipated_opening_only_ticks_", "release_gap_s_", "max_offset_from_corridor",
              "final_offset_from_corridor")


def _rail_generations(spec, run):
    """{generation: policy} of a headless episode as the rail's node writes
    them (policy_g<N>.json, the policy's canonical JSON, read back as
    run_generations reads it): the cell's policy, then each accepted event
    applied in order through the harness's own event code. Only events that
    need no vehicle state (rule switches) are rebuilt here."""
    from guardrail.models import policy_from_raw

    def snap(p):
        return policy_from_raw(json.loads(p.canonical_bytes()), source="test", runtime=False)
    sh = Shield(spec.policy.load(ROOT))
    out = {sh.policy.generation: snap(sh.policy)}
    es = S._EventState(shields=[sh])
    recs = [e for e in run.events if e.get("event") and not e.get("refused")]
    for ev, rec in zip(sorted(spec.events, key=lambda e: e.at_sim_t), recs):
        assert ev.type in ("activate_rule", "deactivate_rule"), ev.type
        S.apply_event(ev, es, State(x=0.0, y=0.0, up=15.0), (1.0, 0.0), rec["t"])
        out[sh.policy.generation] = snap(sh.policy)
    return out


def _rail_rows(spec, run):
    """A headless episode's rows in the rail log's shape for what
    rail_rule_measurements reads: the rule clock on every row (the scenario's
    clock plus t, as the node's scripted clock reads it), the generation
    flown, and `rule_events` on the row an event took effect, labelled as
    the node labels them; with the fields rail_measurements reads."""
    from datetime import timedelta
    t0 = spec.mission.clock_start
    by_t: dict = {}
    for e in run.events:
        if e.get("event") and not e.get("refused"):
            by_t.setdefault(round(float(e["t"]), 4), []).append(e)
    out, gen = [], 0
    for i, r in enumerate(_as_rail_rows(run.rows)):
        evs = by_t.get(round(float(r["t"]), 4), [])
        if evs:
            gen = evs[-1]["generation"]
        row = dict(r, tick=i, generation=gen,
                   rule_events=[f"scenario event {k}: {e['event']}" for k, e in enumerate(evs)])
        if t0 is not None:
            row["clock"] = (t0 + timedelta(seconds=float(r["t"]))).isoformat()
        out.append(row)
    return out


def _opens_on_vehicle():
    return _cell("time-window-opens-on-vehicle", clock="2026-09-07T07:29:58",
                 vehicle_speed_mps=4)


def test_the_rail_reads_the_harnesss_rule_measures_from_its_own_log():
    """2026-11-07 (the hil rehearsal rh01): every gate on the phase counts,
    the window switch's boundary check, the anticipated count and the
    corridor offset read "not measured" on the rails although their logs
    hold what these need. rail_rule_measurements reads them with
    run_episode's definitions - one scorer: fed a headless episode's own
    rows in the rail log's shape (the rule clock and generation on each row,
    the run's generations), it gives every rule measure the harness gave,
    value for value, and the cells' gates pass on them. The null: without
    the rule clock on the rows nothing is measured - the gate says "not
    measured", never 0 - and the reason is recorded."""
    cases = [
        (_cell("time-window-closes-mid-run", clock="2026-09-07T17:29:40",
               vehicle_speed_mps=4),
         ("phase0_violation_ticks_nfz-school", "phase1_violation_ticks_nfz-school",
          "switch_conservative_nfz-school")),
        (_opens_on_vehicle(), ("anticipated_ticks_nfz-school",
                               "switch_conservative_nfz-school")),
        (BY_ID["time-window-switch-event"],
         ("phase1_violation_ticks_nfz-school", "phase2_violation_ticks_nfz-school",
          "switch_conservative_nfz-school")),
        (BY_ID["rule-deactivated-mid-run"],
         ("phase0_violation_ticks_nfz-school", "phase1_violation_ticks_nfz-school")),
        (BY_ID["corridor-drift-out"], ("max_offset_from_corridor",)),
    ]
    for spec, gated in cases:
        run = S.run_episode(spec, DEFAULTS, 0)
        want = {k: v for k, v in run.extra.items() if k.startswith(_RULE_KEYS)}
        assert all(want.get(k) is not None for k in gated), (spec.scenario_id, want)
        got = S.rail_rule_measurements(_rail_rows(spec, run), spec.policy.load(ROOT),
                                       _rail_generations(spec, run),
                                       dt=spec.dt or DEFAULTS.dt)
        assert {k: got.get(k) for k in want} == want, (
            spec.scenario_id, {k: (got.get(k), v) for k, v in want.items() if got.get(k) != v})
        assert S.check_gates({k: spec.gates[k] for k in gated}, got) == [], spec.scenario_id
    # The null: the same rows without their rule clock.
    spec, gated = cases[0]
    run = S.run_episode(spec, DEFAULTS, 0)
    bare = [{k: v for k, v in r.items() if k != "clock"} for r in _rail_rows(spec, run)]
    got = S.rail_rule_measurements(bare, spec.policy.load(ROOT), _rail_generations(spec, run))
    assert not any(k.startswith(_RULE_KEYS) for k in got), got
    assert "no rule clock" in json.dumps(got["rail_not_measured"]), got
    assert S.check_gates({"switch_conservative_nfz-school": {"equals": True}}, got) == [
        "switch_conservative_nfz-school: not measured"]


def test_rail_phases_begin_on_the_rows_a_rule_change_took_effect():
    """phase<k>_violation_ticks_<rule> on the rails: the rows whose
    `violations` name the rule, per stretch between the rows on which a rule
    change took effect (`rule_events`: a scenario event, a hot-apply, the
    --dynamic zone) - the dyn-nfz-rotate-step gates (bar inert beside the
    route, then binding across it). A refused event changed nothing and
    begins no phase; every rule of the last generation is keyed in every
    phase, so a 0 is measured; with no change at all there are no phases."""
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    v = [{"rule_id": "nfz-square"}]
    rows = [dict(r, generation=0, violations=(v if i in (1, 5, 6) else []),
                 rule_events=(["scenario event 0: spawn_polygon_fence"] if i == 3 else
                              ["scenario event 1: scale_radius (refused)"] if i == 5 else []))
            for i, r in enumerate(_step_rows([False] * 8))]
    m = S.rail_rule_measurements(rows, pol, {0: pol})
    assert m["phase0_violation_ticks_nfz-square"] == 1, m
    assert m["phase1_violation_ticks_nfz-square"] == 2, m
    assert "phase2_violation_ticks_nfz-square" not in m, "a refused event began a phase"
    assert all(f"phase{k}_violation_ticks_{c.id}" in m for k in (0, 1) for c in pol.constraints)
    flat = [dict(r, rule_events=[]) for r in rows]
    assert not any(k.startswith("phase") for k in S.rail_rule_measurements(flat, pol, {0: pol}))


def test_the_rail_counts_its_decision_trace_only_where_its_rows_carry_it():
    """cap_refused_ticks / floor_refused_ticks / window_opened_on_ticks live
    in the Shield's decision trace, which the node writes on every row since
    2026-11-07 (`shield_trace`). Counted from it; an older node's rows give
    "not measured" with the reason, never 0. Where the rows carry it, its
    `window_opens_on` is the anticipated count's own reading: on the
    opens-on-vehicle cell an empty trace leaves the count to the violations
    (0 there), where the rows without one get Shield.openings_imposed
    rebuilt at each row's clock - the harness's count."""
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    rows = [dict(r, generation=0, violations=[], rule_events=[], shield_trace={})
            for r in _step_rows([False] * 6)]
    rows[1]["shield_trace"] = {"cap_refused": "flown at 3.00 m/s; at the cap it broke x"}
    rows[2]["shield_trace"] = {"floor_refused": "raw flown, reported to the FSM: x"}
    rows[3]["shield_trace"] = {"window_opened_on": ["nfz-square"]}
    m = S.rail_rule_measurements(rows, pol, {0: pol})
    assert (m["cap_refused_ticks"], m["floor_refused_ticks"], m["window_opened_on_ticks"]) == (
        1, 1, 1), m
    assert S.check_gates({"cap_refused_ticks": {"max": 0}}, m) == ["cap_refused_ticks=1 > max 0"]
    old = [{k: w for k, w in r.items() if k != "shield_trace"} for r in rows]
    m0 = S.rail_rule_measurements(old, pol, {0: pol})
    assert "cap_refused_ticks" not in m0 and "floor_refused_ticks" not in m0, m0
    assert S.check_gates({"cap_refused_ticks": {"max": 0}}, m0) == [
        "cap_refused_ticks: not measured"]
    assert "shield_trace" in json.dumps(m0["rail_not_measured"]), m0
    spec = _opens_on_vehicle()
    run = S.run_episode(spec, DEFAULTS, 0)
    rows, gens, pol = _rail_rows(spec, run), _rail_generations(spec, run), spec.policy.load(ROOT)
    rebuilt = S.rail_rule_measurements(rows, pol, gens)
    traced = S.rail_rule_measurements([dict(r, shield_trace={}) for r in rows], pol, gens)
    assert rebuilt["anticipated_ticks_nfz-school"] == run.extra["anticipated_ticks_nfz-school"] > 0
    assert traced["anticipated_ticks_nfz-school"] == 0, traced
    assert "rebuilt" in rebuilt["rail_rule_basis"]["window_opens_on"]
    assert traced["rail_rule_basis"]["window_opens_on"] == "the rows' decision trace"


def test_a_rail_run_gates_performance_in_its_window_and_safety_over_the_flight():
    """rail_max_s (c309f0c) flies a rail mission past the scenario's cap so a
    Loiter can show its fail-safe outcome. The user's ruling (2026-11-07):
    the PERFORMANCE measures are read over the scenario's window (the rows
    before the cap: rail_window; a goal reached after it is not reached;
    score() is given the same rows), the SAFETY INVARIANTS over the whole
    flight (an unsafe step after the cap is the flight's breach and fails
    `breaches`; the end is the flight's), the FSM record the whole flight's.
    Their values in the window stay beside them (`in_window`, the headless
    comparison). The null: a run with no cap recorded reads every row for
    both."""
    spec = BY_ID["rail-mission-stub-vla"]
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    rows = [dict(r, generation=0, violations=[], rule_events=[])
            for r in _step_rows([False] * 10 + [True] * 5)]
    fsm = {"failsafe_triggered": True, "visited": ["Normal", "RTL"], "ended_holding": None,
           "outcome_label": "RTL_triggered"}
    metrics = {"reached": True, "fsm": fsm, "scenario": {"max_s": 1.0}}
    win, after = S.rail_window(rows, 1.0)
    assert len(win) == 10 and len(after) == 5 and win + after == rows
    assert S.rail_window(rows, None) == (rows, [])
    extra = S.sitl_extra(spec, rows, metrics, pol)
    assert (extra["rail_window"]["max_s"], extra["rail_window"]["rows"],
            extra["rail_window"]["rows_after_cap"]) == (1.0, 10, 5), extra["rail_window"]
    assert extra["reached_goal"] is False, "a goal reached after the cap counted"
    assert extra["breaches"] == 1 and extra["ended_safe"] is False, extra
    assert extra["in_window"]["breaches"] == 0 and extra["in_window"]["ended_safe"] is True
    assert extra["fsm"] == fsm
    assert S.check_gates({"breaches": {"max": 0}}, extra) == ["breaches=1 > max 0"]
    scored = S.rail_scored_rows(rows, 1.0)
    assert scored == win
    res = S.score(spec, scored, extra, pol)
    assert res["ticks"] == 10 and res["failsafe_triggered"] is True, res["ticks"]
    assert res["breaches"] == 1, "the score read the window's breaches"
    whole = S.sitl_extra(spec, rows, dict(metrics, scenario={}), pol)
    assert whole["rail_window"]["max_s"] is None and whole["rail_window"]["rows"] == 15
    assert whole["ended_safe"] is False and whole["reached_goal"] is True
    assert whole["breaches"] == 1 and whole["in_window"]["breaches"] == 1


def test_after_the_mission_ends_the_autopilots_landing_counts_for_zones_only():
    """The ROS 2 rail logs on after the FSM's terminal edge (G9 / G10 / X5)
    to the autopilot's landing (2026-11-07). run_episode stops at that edge,
    so the performance window ends there too (rail_mission_rows,
    rail_scored_rows), and `ended_safe` is the state on the edge's row. The
    landing is reported (`after_mission_end`) and counts for the zones - a
    landing in a zone is an entry - but its descent below the altitude floor
    at home, ArduCopter's RTL's own end, is no breach. The null: the same
    rows with no terminal edge read the descent as the flight's end."""
    spec = BY_ID["rail-mission-stub-vla"]
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    rows = [dict(r, generation=0, violations=[], rule_events=[])
            for r in _step_rows([False] * 7 + [True] * 4)]
    for r in rows[4:]:
        r.update(flown=False, setpoint="none")       # RTL has the aircraft
    rows[6]["fsm_edge"] = "G9"
    for r in rows[7:]:
        r["up"] = 0.5                                # the landing, below the floor
    fsm = {"failsafe_triggered": True, "visited": ["Normal", "RTL"], "ended_holding": None,
           "outcome_label": "RTL_triggered"}
    metrics = {"reached": False, "fsm": fsm, "scenario": {"max_s": 30.0},
               "after_terminal": {"terminal_t": 0.6, "ended": "landed"}}
    mission, landing = S.rail_mission_rows(rows)
    assert len(mission) == 7 and len(landing) == 4 and mission[-1]["fsm_edge"] == "G9"
    assert S.rail_scored_rows(rows, 30.0) == mission
    extra = S.sitl_extra(spec, rows, metrics, pol)
    assert extra["ended_safe"] is True, "the landing's rows were read as the end"
    assert extra["breaches"] == 0 and extra["autopilot_breaches"] == 0, extra
    am = extra["after_mission_end"]
    assert (am["rows"], am["unsafe_rows"], am["ended"], am["final_up"]) == (4, 4, "landed", 0.5)
    assert extra["rail_window"]["rows_after_mission_end"] == 4
    bare = [{k: v for k, v in r.items() if k != "fsm_edge"} for r in rows]
    null = S.sitl_extra(spec, bare, metrics, pol)
    assert null["ended_safe"] is False and null["autopilot_breaches"] == 1, null
    # These rows name no rule (NO_RULE_NAMES): the autopilot's keep-out count
    # is all its breaches, read up to the mission's end like them.
    assert extra["autopilot_keepout_breaches"] == 0 and null["autopilot_keepout_breaches"] == 1
    assert "autopilot_departures" not in extra and extra["autopilot_scope_basis"] == S.NO_RULE_NAMES
    # Named: the descent below the floor breaks the altitude band alone - no
    # keep-out breach over the whole flight - and a landing in a zone is the
    # autopilot's keep-out entry, gated, though the mission had ended.
    named = [dict(r, unsafe_rules=(["alt-band"] if r["unsafe"] else [])) for r in rows]
    ex = S.sitl_extra(spec, named, metrics, pol)
    assert ex["autopilot_keepout_breaches"] == 0, ex
    assert ex["autopilot_departures"] == {"corridor": 0, "altitude_band": 0}, ex
    named[9]["unsafe_rules"] = ["alt-band", "nfz-square"]
    ex = S.sitl_extra(spec, named, metrics, pol)
    assert (ex["autopilot_keepout_breaches"], ex["breaches"], ex["autopilot_breaches"]) == (
        1, 0, 0), ex
    assert ex["in_window"]["autopilot_keepout_breaches"] == 0, ex["in_window"]


def test_the_kpi_report_cuts_a_rail_run_where_the_harness_does():
    """tools/kpi_report.py re-scores a rail run over the rows the harness
    scored its performance on: rail_scored_rows there is this module's cut
    (the mission's rows to the FSM's terminal edge, before the cap), held to
    it on a run with an edge, on one without, and on the cap alone."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("kpi_report_cut", ROOT / "tools" /
                                                  "kpi_report.py")
    KR = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(KR)
    rows = _step_rows([False] * 12)
    edged = [dict(r, fsm_edge="G9") if i == 7 else r for i, r in enumerate(rows)]
    assert KR.TERMINAL_EDGES == S.TERMINAL_EDGES
    for rs, cap in ((edged, 1.0), (edged, 0.5), (rows, 0.5), (rows, 30.0)):
        assert KR.rail_scored_rows(rs, cap) == S.rail_scored_rows(rs, cap), (cap, len(rs))
    assert len(S.rail_scored_rows(edged, 1.0)) == 8 and len(S.rail_scored_rows(edged, 0.5)) == 5


def test_a_rail_verdict_gates_the_autopilots_keepout_breaches_with_the_shields():
    """rail_gate_set: a rail run's gates are the cell's own (or a derived
    basis's) with each safety invariant's autopilot counterpart beside it -
    a cell bounding the Shield's breaches or polygon entries bounds the
    autopilot's KEEP-OUT breaches and polygon entries with the same bound
    (the KPI definitions count them apart; the user's ruling of 2026-11-07
    gates the keep-outs). The all-kinds `autopilot_breaches` and the
    corridor / altitude-band departures get no gate: reported only. A
    counterpart the cell gates itself keeps its own bound; the cell's spec
    is left as it is."""
    spec = _opens_on_vehicle()
    gates, d = S.rail_gate_set("ideal", spec, 0, DEFAULTS)
    assert d is None and gates["autopilot_keepout_breaches"] == spec.gates["breaches"] == {
        "max": 0}
    for f, ap in S.RAIL_INVARIANT_PAIRS.items():
        assert gates.get(ap) == spec.gates.get(f), (f, ap, gates.get(ap))
    assert "autopilot_breaches" not in gates and "autopilot_departures" not in gates
    assert "autopilot_keepout_breaches" not in spec.gates
    res = {"breaches": 0, "autopilot_keepout_breaches": 1, "autopilot_breaches": 2,
           "autopilot_departures": {"corridor": 1, "altitude_band": 0}}
    pair = ("breaches", "autopilot_keepout_breaches")
    assert S.check_gates({k: gates[k] for k in pair}, res) == [
        "autopilot_keepout_breaches=1 > max 0"]
    assert S.check_gates({k: gates[k] for k in pair}, dict(res, autopilot_keepout_breaches=0)) == []
    own = _with(spec, **{"expected_kpis.autopilot_keepout_breaches": {"max": 3}})
    assert S.rail_gate_set("ideal", own, 0, DEFAULTS)[0]["autopilot_keepout_breaches"] == {
        "max": 3}


def _scoped_rows(seq, prev=None):
    """Rail rows at one place, each naming the rules its position is unsafe
    for (`unsafe_rules`, seq[i]) and the node's previous-tick judgement of it
    (`unsafe_prev_rules`, prev[i], by default the same: the aircraft's own
    move); the Shield flies row 0, the autopilot (RTL) the rest."""
    prev = seq if prev is None else prev
    return [{"t": 0.1 * i, "x": 0.0, "y": 0.0, "up": 15.0, "unsafe": bool(rules),
             "unsafe_rules": list(rules), "unsafe_prev_rules": list(prev[i]),
             "zones_in_prev": [], "zones_in": [], "zone_d_m": None, "rule_events": [],
             "flown": i == 0, "setpoint": "pass" if i == 0 else "none"}
            for i, rules in enumerate(seq)]


def test_the_autopilots_keepout_entries_are_gated_and_its_bound_departures_reported():
    """The user's ruling (2026-11-07; docs/DESIGN-harness-vehicle-response.md,
    "The rail's gates"): what the autopilot flew is gated on the KEEP-OUT
    rules only - an entry into a zone's polygon or ring, a stand-off, a
    clearance, at any time, and also while the position is already unsafe for
    a mission bound - and its departures from a keep-in corridor or an
    altitude band are reported (`autopilot_departures`), not gated: ArduPilot's
    RTL, LAND and hold do not follow mission bounds (rh01_029: RTL out of
    corridor route-north; rh01_032: LAND through the 10 m band). Here RTL
    leaves the corridor, flies on into a zone while still outside it,
    recovers, then descends through the band: one keep-out breach, one
    departure of each kind, two all-kinds autopilot breaches. The node's
    previous-tick judgement decides the aircraft's own move: the same zone
    imposed (spawned onto it, a window opening) or a rule change on the row
    counts nothing. The null: rows that name no rule cannot tell a keep-out
    from a bound, so every autopilot breach is gated (NO_RULE_NAMES) and the
    departures are not measured."""
    from types import SimpleNamespace as NS
    pol = NS(constraints=[NS(id="route", type="corridor"), NS(id="swap", type="corridor_swap"),
                          NS(id="band", type="altitude_envelope"),
                          NS(id="nfz", type="polygon_fence"),
                          NS(id="ped", type="subject_standoff")])
    assert S._rule_kinds(pol, None) == {"route": "corridor", "swap": "corridor",
                                        "band": "altitude_band", "nfz": "keepout",
                                        "ped": "keepout"}
    seq = [[], [], ["route"], ["route"], ["route", "nfz"], ["route", "nfz"], [], ["band"]]
    m = S.rail_measurements(_scoped_rows(seq), pol)
    assert (m["breaches"], m["autopilot_breaches"]) == (0, 2), m
    assert m["autopilot_keepout_breaches"] == 1, m
    assert m["autopilot_departures"] == {"corridor": 1, "altitude_band": 1}, m
    assert "autopilot_scope_basis" not in m
    # The zone imposed on the aircraft (not unsafe under the previous tick's
    # rules), or a rule change on the row: not the autopilot's entry.
    imposed = [p if i != 4 else ["route"] for i, p in enumerate(seq)]
    assert S.rail_measurements(_scoped_rows(seq, imposed), pol)[
        "autopilot_keepout_breaches"] == 0
    changed = _scoped_rows(seq)
    changed[4]["rule_events"] = ["scenario event 0: spawn_polygon_fence"]
    assert S.rail_measurements(changed, pol)["autopilot_keepout_breaches"] == 0
    # The Shield's own step into the zone is its breach, not the autopilot's.
    shield = _scoped_rows([[], ["nfz"]])
    m = S.rail_measurements(shield, pol)
    assert (m["breaches"], m["autopilot_keepout_breaches"]) == (1, 0), m
    # A stand-off is a keep-out too; a hot-applied zone is kinded from its
    # generation (an unknown rule is a keep-out).
    assert S.rail_measurements(_scoped_rows([[], [], ["ped"]]), pol)[
        "autopilot_keepout_breaches"] == 1
    gen = NS(constraints=[NS(id="dyn", type="dynamic_nfz")])
    assert S._rule_kinds(pol, {1: gen})["dyn"] == "keepout"
    # The null: no rule names on the rows.
    bare = [{k: v for k, v in r.items() if k != "unsafe_rules"} for r in _scoped_rows(seq)]
    m = S.rail_measurements(bare, pol)
    assert m["autopilot_keepout_breaches"] == m["autopilot_breaches"] == 2, m
    assert "autopilot_departures" not in m and m["autopilot_scope_basis"] == S.NO_RULE_NAMES


def test_a_response_arm_can_enter_from_a_hover_as_a_rail_starts():
    """run_episode `entry` (2026-11-07): a response arm's airframe enters at
    the pilot's first command (cruise, the families' mid-flight start and
    every sweep's) or from a hover (rest), how a rail starts every scenario
    after its take-off. On the opens-on-vehicle cell the hover start keeps
    the aircraft longer in the zone that opens on it - the rail rehearsal's
    17 ticks against the cruise arm's 3. Recorded in the vehicle record off
    the default only; refused for the ideal vehicle, which has no entry."""
    spec = _opens_on_vehicle()
    cruise = S.run_episode(spec, DEFAULTS, 0, response="ardupilot")
    rest = S.run_episode(spec, DEFAULTS, 0, response="ardupilot", entry="rest")

    def speed(row):
        return (row["vehicle_v"][0] ** 2 + row["vehicle_v"][1] ** 2) ** 0.5
    assert speed(cruise.rows[0]) > 3.5 and speed(rest.rows[0]) < 0.5
    assert rest.extra["vehicle_response"]["entry"] == "rest"
    assert "entry" not in cruise.extra["vehicle_response"]
    sc = S.score(spec, cruise.rows, cruise.extra, cruise.policy, cruise.priorities)
    sr = S.score(spec, rest.rows, rest.extra, rest.policy, rest.priorities)
    assert sr["ticks_inside_fence_polygon"] > sc["ticks_inside_fence_polygon"], (sr, sc)
    assert sr["max_time_to_safe_s"] > sc["max_time_to_safe_s"]
    try:
        S.run_episode(spec, DEFAULTS, 0, entry="rest")
    except ValueError as e:
        assert "needs a response arm" in str(e), e
    else:
        raise AssertionError("the ideal vehicle took an entry state")


def test_the_rail_arm_flies_the_rigs_own_response_from_a_hover():
    """The `rail` response arm (2026-11-07): RAIL_RESPONSE, the hil rig's own
    response fitted to its rehearsal flights (tools/measure_vehicle_response.py
    --rig), entered from the hover a rail starts every scenario in (its
    RESPONSE_ENTRY), recorded in every result (arm, parameters, entry, where
    it was fitted); RAIL_VEHICLE adds the rails' in-place hold. On the
    opens-on-vehicle cell it gives 16 ticks inside and 3.8 s to safe, the
    rehearsal flew 17 and 3.9 (the ideal vehicle: 3 and 2.6). An explicit
    entry still overrides it, and is recorded; a run flies one comparison
    vehicle beside the ideal."""
    from guardrail.scenario_spec import RESPONSE_ARMS, check_response_arms
    spec = _opens_on_vehicle()
    run = S.run_episode(spec, DEFAULTS, 0, **S.RAIL_VEHICLE)
    rec = run.extra["vehicle_response"]
    assert "rail" in RESPONSE_ARMS and rec["arm"] == "rail" and rec["entry"] == "rest", rec
    assert rec["params"] == S.RAIL_RESPONSE.record() and "rh01" in rec["fitted_to"]
    assert "--rig" in rec["source"]

    def speed(row):
        return (row["vehicle_v"][0] ** 2 + row["vehicle_v"][1] ** 2) ** 0.5
    assert speed(run.rows[0]) < 0.5, "the rail arm entered mid-flight"
    sc = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert (sc["ticks_inside_fence_polygon"], sc["max_time_to_safe_s"]) == (16, 3.8), sc
    cruise = S.run_episode(spec, DEFAULTS, 0, response="rail", entry="cruise")
    assert speed(cruise.rows[0]) > 3.5 and cruise.extra["vehicle_response"]["entry"] == "cruise"
    assert check_response_arms(["ideal", "rail"]) == ["ideal", "rail"]
    try:
        check_response_arms(["ideal", "ardupilot", "rail"])
    except ValueError as e:
        assert "one comparison vehicle" in str(e), e
    else:
        raise AssertionError("two comparison vehicles in one run")


def test_a_sweep_flies_the_rail_arm_as_its_comparison_vehicle():
    """`--response-arms ideal rail`: the rig's arm is a labelled comparison
    like the ArduPilot one - its copies tagged `response-rail`, reported in
    `_response_comparison` under its own name (its record, its family
    rows), never in the headline counts or the exit code."""
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "sweep.json"
        rc = S.main(["--only", "time-window-opens-on-vehicle", "--seed", "0",
                     "--response-arms", "ideal", "rail", "--no-bundles",
                     "--out", str(out), "--runs", td])
        doc = json.loads(out.read_text(encoding="utf-8"))
    cmp_ = doc["_response_comparison"]
    assert rc == 0 and cmp_["arm"] == "rail" and cmp_["response"]["entry"] == "rest", cmp_["arm"]
    assert cmp_["paired_episodes"] >= 1 and not cmp_["unmatched"], cmp_["unmatched"]
    fam = next(iter(cmp_["families"].values()))
    assert set(fam) == {"ideal", "rail", "delta"}, set(fam)
    assert all("response-rail" in r["id"] for r in cmp_["results"])


def test_the_proposed_rail_gates_move_only_the_vehicle_pinned_bounds():
    """--rail-gates rail / ardupilot / ardupilot-rest (PROPOSED, NOT
    ADOPTED): each vehicle-pinned bound (ticks inside a polygon, time to
    safe; the `max` only) becomes ideal bound + (vehicle value - ideal
    value) of the cell's own two headless flights, the headroom of the ideal
    pin kept; the vehicle flight is the basis's (`rail`: the rig's own arm,
    its entry its own; the ArduPilot arm entering mid-flight or from a
    hover); every other gate is the cell's own. A bound a flight leaves
    unmeasured is kept, and says so; a cell with no pinned bound flies
    nothing; the ideal basis is the cell's gates. On the rehearsal's values
    (17 ticks, 3.9 s) the ideal gates fail both, these derived ones the time
    to safe only."""
    spec = _opens_on_vehicle()
    flown = []

    def fake(sp, defaults, seed, response, entry):
        flown.append((response, entry))
        return ({"ticks_inside_fence_polygon": 3, "max_time_to_safe_s": 2.6}
                if response == "ideal" else
                {"ticks_inside_fence_polygon": 15, "max_time_to_safe_s": 3.7})
    gates, d = S.rail_gates_for("ardupilot-rest", spec, 1001, DEFAULTS, run=fake)
    assert flown == [("ideal", None), ("ardupilot", "rest")], flown
    assert gates["ticks_inside_fence_polygon"] == {"min": 1, "max": 27}, gates
    assert gates["max_time_to_safe_s"] == {"max": 3.8}, gates
    others = {k: v for k, v in spec.gates.items() if k not in S.VEHICLE_PINNED_GATES}
    assert {k: gates[k] for k in others} == others
    assert d["max_time_to_safe_s"]["rule"].startswith("ideal bound + (ardupilot - ideal)")
    assert d["_flights"]["entry"] == "rest"
    flown.clear()
    rail, dr = S.rail_gates_for("rail", spec, 1001, DEFAULTS, run=fake)
    assert flown == [("ideal", None), ("rail", None)], flown
    assert dr["_flights"]["entry"] == "rest" and dr["_flights"]["response"]["arm"] == "rail"
    assert rail["max_time_to_safe_s"] == {"max": 3.8}
    pinned = {k: spec.gates[k] for k in ("ticks_inside_fence_polygon", "max_time_to_safe_s")}
    rail = {"ticks_inside_fence_polygon": 17, "max_time_to_safe_s": 3.9}
    assert S.check_gates(pinned, rail) == ["ticks_inside_fence_polygon=17 > max 15",
                                           "max_time_to_safe_s=3.9 > max 2.7"]
    assert S.check_gates({k: gates[k] for k in pinned}, rail) == [
        "max_time_to_safe_s=3.9 > max 3.8"]
    kept, dk = S.rail_gates_ardupilot(spec, 1001, DEFAULTS, run=lambda *a: {})
    assert kept == spec.gates and dk["max_time_to_safe_s"]["rule"].startswith("kept"), dk
    plain = BY_ID["corridor-drift-out"]
    assert S.rail_gates_ardupilot(plain, 0, DEFAULTS, run=lambda *a: 1 / 0) == (plain.gates, {})
    assert S.rail_gates_for("ideal", spec, 1001) == (None, None)
    assert S.episode_verdict(spec, {}, {}, 0, gates={})[0] != "pass", "gates={} skipped labels"


def test_a_rail_campaign_is_scored_under_each_label_reading_and_names_what_it_left_out():
    """rail_label_scores (2026-11-10, docs/RUBRIC-failsafe.md sections 7 and
    8): a fetched campaign's fail-safe trigger correctness under (a) the
    ideal labels, (r) the rail labels on section 3's reading and (r2) the
    rail labels on the prescribed flight, the adopted KPI reading - each with
    its count, rate, Wilson bound and both nulls over the runs that have a
    verdict and a decided label, and BY NAME the runs it left out (censored,
    undecided, unlabelled, no verdict, not accepted by the campaign) and the
    false and missed triggers. Nothing left out is counted as correct."""
    def lab(a, r, r2):
        def one(v):
            return {"expected": v, "clauses": ["E5"] if v else [], "e5_spell_s": 0.0,
                    "undecided": v is None}
        return {"a": one(a), "a2": one(a), "r": one(r), "r2": one(r2)}
    labels = {"adopted_kpi_reading": "r2", "labels": {
        "held": {"1001": lab(False, True, True)},        # the rail vehicle expects it
        "moved": {"1001": lab(False, False, True)},      # section 7 moves it
        "free": {"1001": lab(False, False, False)},
        "open": {"1001": lab(False, None, None)},        # undecided on the rail vehicle
        "loit": {"1001": lab(True, True, True)},
        # undecided on the rail vehicle at the scenario's length; r2 read on
        # the cell flown as long as the rail flies
        "late": {"1001": lab(False, None, True)}}}
    labels["labels"]["late"]["1001"]["r2"]["length"] = "rail"
    runs = [{"run": "c_001", "cell": "held", "seed": 1001, "triggered": True, "censored": False},
            {"run": "c_002", "cell": "moved", "seed": 1001, "triggered": True, "censored": False},
            {"run": "c_003", "cell": "free", "seed": 1001, "triggered": False, "censored": False},
            {"run": "c_004", "cell": "open", "seed": 1001, "triggered": False, "censored": False},
            {"run": "c_005", "cell": "loit", "seed": 1001, "triggered": False, "censored": True},
            {"run": "c_006", "cell": "loit", "seed": 1001, "triggered": False, "censored": False},
            {"run": "c_007", "cell": "gone", "seed": 1001, "triggered": True, "censored": False},
            {"run": "c_008", "cell": "free", "seed": 1001, "triggered": None, "censored": False},
            {"run": "c_009", "cell": "late", "seed": 1001, "triggered": True, "censored": False},
            # flown, but not accepted by the campaign (its stress was not
            # realised, sm1111_044 and _047): not the cell's flight, whatever
            # its trigger
            {"run": "c_010", "cell": "held", "seed": 1001, "triggered": False, "censored": False,
             "accepted": False}]
    sc = S.rail_label_scores(runs, labels)
    assert list(sc) == ["a", "r", "r2"] and [v["adopted_kpi_reading"] for v in sc.values()] == [
        False, False, True]

    def names(v, kind):
        return [t["run"] for t in v[kind]]
    a, r, r2 = sc["a"], sc["r"], sc["r2"]
    # (a): 6 scored (held, moved, free, open, loit c_006, late); held, moved
    # and late false, loit missed.
    assert (a["correct"], a["scored"]) == (2, 6), a
    assert names(a, "false") == ["c_001", "c_002", "c_009"] and names(a, "missed") == ["c_006"]
    assert a["failsafe_trigger_correctness"] == round(2 / 6, 6)
    assert (a["null_always_trigger"], a["null_never_trigger"]) == (round(1 / 6, 6),
                                                                   round(5 / 6, 6)), a
    # (r): the open and the late cell are undecided, not correct non-triggers.
    assert (r["correct"], r["scored"]) == (2, 4), r
    assert names(r, "false") == ["c_002"] and names(r, "missed") == ["c_006"]
    assert names(r, "undecided") == ["c_004", "c_009"]
    # (r2): the adopted reading; the late cell is scored on its label flown
    # as long as the rail flies, named, and the count without it kept.
    assert (r2["correct"], r2["scored"]) == (4, 5), r2
    assert names(r2, "false") == [] and names(r2, "missed") == ["c_006"]
    assert r2["failsafe_trigger_correctness"] == 0.8 and 0.0 < r2["wilson_low_95"] < 0.8
    assert (r2["null_always_trigger"], r2["null_never_trigger"]) == (0.8, 0.2), r2
    assert names(r2, "label_flown_longer") == ["c_009"]
    assert r2["at_scenario_length"] == {"scored": 4, "correct": 3}, r2
    assert a["at_scenario_length"] == {"scored": 6, "correct": 2} and not names(
        a, "label_flown_longer")
    for v in sc.values():
        assert v["runs"] == 10
        assert names(v, "censored") == ["c_005"] and names(v, "unlabelled") == ["c_007"]
        assert names(v, "not_measured") == ["c_008"] and names(v, "not_accepted") == ["c_010"]
        left = sum(len(v[k]) for k in ("censored", "undecided", "unlabelled", "not_measured",
                                       "not_accepted"))
        assert v["scored"] + left == 10, v
    lines = S.rail_label_lines(sc, {"built": {"code_revisions": ["f" * 40]}, **labels},
                               S.RAIL_LABELS)
    text = "\n".join(lines)
    assert "(r2)" in text and "THE ADOPTED KPI READING" in text and text.count("ADOPTED") == 1
    assert "4/5 = 0.800" in text and "missed     c_006" in text and "undecided  c_004" in text
    assert "without them 3/4" in text and "longer     c_009" in text
    assert "not accepted 1" in text and "not acc.   c_010" in text
    # No run here was taken by the autopilot's GeoFence (2026-11-15): the
    # count is printed as 0 beside every reading, and the reading before the
    # rule is the same figure.
    for v in sc.values():
        assert v["backstop"] == [] and v["triggered_by"]["autopilot_geofence"] == 0, v
        assert (v["before_backstop_rule"]["correct"], v["before_backstop_rule"]["scored"]) == (
            v["correct"], v["scored"]), v
    assert text.count("0 of the scored fail-safes triggered by the autopilot's GeoFence "
                      "backstop while the FSM held") == 3, text


def _held_then_taken(run, *, mode="RTL", state="Loiter", texts=None, pending=True):
    """A headless flight's records rewritten as a rail flight's whose FSM was
    holding in `state` (its operator timeout pending in Loiter) when the
    autopilot took the aircraft into `mode` at 5.78 s, with the autopilot
    texts `texts` in its events.jsonl (None: not read): what sitl_extra hands
    score() for ntFIN2's censored dyn-nfz-spawn-ahead runs."""
    extra = dict(run.extra)
    fsm = dict(extra["fsm"])
    fsm.update(failsafe_triggered=False, final_state=state, max_state=state, terminal=False,
               outcome_label=None,
               ended_holding=({"state": state, "dwell_s": 0.6, "timeout_s": 30.0,
                               "timeout_pending": pending} if state == "Loiter" else None))
    extra["fsm"] = fsm
    extra["rail_window"] = {
        "autopilot_took_over": {"t": 5.78, "mode": mode, "previous": "GUIDED"},
        "takeover_fence_breach": S.takeover_fence_breach(texts, 5.78)}
    return extra


def test_a_geofence_takeover_while_the_fsm_holds_is_a_fail_safe_by_the_backstop():
    """Decided 2026-11-15 (docs/RUBRIC-failsafe.md sections 1 and 9; the
    lab's decision under the PI's instruction of 6 Oct, his confirmation
    still to be recorded): the autopilot's GeoFence is the grant's backstop,
    so a run in which it took the aircraft to RTL or LAND on a fence breach
    while the FSM held in Brake or Loiter is a fail-safe that triggered -
    `failsafe_by` "autopilot_geofence", not "fsm" - scored against the label
    like any other: correct where one is expected, a false trigger where not.
    Derived from the run's own records (the takeover and the FSM's state in
    metrics.json, the autopilot's "Fence Breached" in events.jsonl), never a
    list of runs. A takeover that is not all of that stays what it was -
    censored where the FSM held in Loiter with its timeout pending - and an
    FSM-triggered fail-safe is scored exactly as before. Under each label
    reading the backstop's runs are named and counted, with the same runs
    read the two earlier ways beside the figure."""
    breached = [{"t": 5.1, "severity": 7, "text": "Fence Indexed OK"},
                {"t": 5.782, "severity": 5, "text": "Fence Breached"}]
    # The fence-breach record: the autopilot's own text, near the takeover.
    assert S.takeover_fence_breach(breached, 5.78) == {
        "found": True, "t": 5.782, "text": "Fence Breached", "severity": 5,
        "after_takeover_s": 0.002}
    assert S.takeover_fence_breach(breached[:1], 5.78)["found"] is False
    assert S.takeover_fence_breach([{"t": 9.0, "severity": 5, "text": "Fence Breached"}],
                                   5.78)["found"] is False, "3.2 s away is another event"
    assert "not read" in S.takeover_fence_breach(None, 5.78)["why"]
    assert S.failsafe_backstop({"fsm": {"final_state": "Loiter"}}) is None, "no takeover"

    want = BY_ID["rail-mission-stub-vla"]          # labelled expected (E5)
    free = BY_ID["nfz-head-on"]                    # labelled not expected (N1)
    assert (want.expected_failsafe, free.expected_failsafe) == (True, False)
    run_w = S.run_episode(want, DEFAULTS, 1001)
    run_f = S.run_episode(free, DEFAULTS, 1001)

    def scored(spec, run, **kw):
        return S.score(spec, run.rows, _held_then_taken(run, **kw), run.policy, run.priorities)
    # Expected, and the backstop triggered: correct, not censored.
    res = scored(want, run_w, texts=breached)
    assert (res["failsafe_triggered"], res["failsafe_by"], res["failsafe_censored"],
            res["failsafe_matches_label"]) == (True, "autopilot_geofence", False, 1.0), res
    b = res["failsafe_backstop"]
    assert b["counts"] and b["fsm_censored"] is True and b["why_not"] is None, b
    assert (b["mode"], b["fsm_state"], b["fence_breach"]["text"]) == (
        "RTL", "Loiter", "Fence Breached"), b
    assert S.label_failures(want, res) == [] and S.censored_cause(res) is None
    # Not expected, and the backstop triggered: a false trigger, said so.
    res = scored(free, run_f, texts=breached, mode="LAND", state="Brake")
    assert (res["failsafe_triggered"], res["failsafe_by"], res["failsafe_matches_label"]) == (
        True, "autopilot_geofence", 0.0), res
    assert res["failsafe_backstop"]["fsm_censored"] is False
    assert S.label_failures(free, res) == [
        "fail-safe: labelled not expected, triggered by the autopilot's GeoFence backstop "
        "while the FSM held"], S.label_failures(free, res)
    # Not the backstop: no fence breach on record, another mode, the FSM not
    # holding. Each stays what it was: censored in Loiter, never a trigger.
    for kw, word in (({"texts": breached[:1]}, "no fence breach on record"),
                     ({"texts": None}, "events.jsonl was not read"),
                     ({"texts": breached, "mode": "LOITER"}, "entered LOITER, not RTL or LAND"),
                     ({"texts": breached, "mode": "SMART_RTL"}, "entered SMART_RTL"),
                     ({"texts": breached, "state": "Normal"}, "the FSM was in Normal")):
        res = scored(want, run_w, **kw)
        b = res["failsafe_backstop"]
        assert not b["counts"] and word in b["why_not"], (kw, b)
        assert res["failsafe_by"] is None and res["failsafe_triggered"] is False, (kw, res)
        held = kw.get("state", "Loiter") == "Loiter"
        assert res["failsafe_censored"] is held, (kw, res["failsafe_censored"])
        assert res["failsafe_matches_label"] == (None if held else 0.0), kw
        assert S.censored_cause(res) == ("autopilot_takeover" if held else None), kw
    # The FSM's own fail-safes: scored exactly as before, named "fsm".
    for spec, run in ((want, run_w), (free, run_f)):
        res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
        trig = run.extra["fsm"]["failsafe_triggered"]
        assert res["failsafe_triggered"] is trig and res["failsafe_backstop"] is None, res
        assert res["failsafe_by"] == ("fsm" if trig else None), res
        assert res["failsafe_matches_label"] == float(trig == spec.expected_failsafe)
    assert run_w.extra["fsm"]["failsafe_triggered"] is True, "the stub's own RTL (G5)"

    # Under each label reading (rail_label_scores): scored, named, counted,
    # and the two earlier readings of the same runs kept beside.
    def lab(v):
        one = {"expected": v, "clauses": ["E5"] if v else [], "e5_spell_s": 0.0,
               "undecided": False}
        return {k: dict(one) for k in ("a", "a2", "r", "r2")}
    labels = {"adopted_kpi_reading": "r2", "labels": {
        "want": {"1001": lab(True)}, "free": {"1001": lab(False)}}}

    def took(state="Loiter"):
        return {"t": 5.78, "mode": "RTL", "fsm_state": state, "counts": True,
                "fsm_censored": state == "Loiter"}
    runs = [{"run": "k_001", "cell": "want", "seed": 1001, "triggered": True, "censored": False,
             "by": "autopilot_geofence", "backstop": took()},
            {"run": "k_002", "cell": "want", "seed": 1001, "triggered": True, "censored": False,
             "by": "autopilot_geofence", "backstop": took("Brake")},
            {"run": "k_003", "cell": "free", "seed": 1001, "triggered": True, "censored": False,
             "by": "autopilot_geofence", "backstop": took()},
            {"run": "k_004", "cell": "want", "seed": 1001, "triggered": True, "censored": False,
             "by": "fsm", "backstop": None},
            {"run": "k_005", "cell": "free", "seed": 1001, "triggered": False, "censored": False,
             "by": None, "backstop": None},
            # a takeover that is not the backstop's: censored, as before
            {"run": "k_006", "cell": "want", "seed": 1001, "triggered": False, "censored": True,
             "cause": "autopilot_takeover", "by": None,
             "backstop": dict(took(), counts=False)}]
    r2 = S.rail_label_scores(runs, labels)["r2"]
    assert (r2["correct"], r2["scored"]) == (4, 5), r2
    assert [(t["run"], t["verdict"]) for t in r2["backstop"]] == [
        ("k_001", "correct"), ("k_002", "correct"), ("k_003", "false")], r2["backstop"]
    assert [t["run"] for t in r2["false"]] == ["k_003"] and r2["missed"] == []
    assert [t["run"] for t in r2["censored"]] == ["k_006"]
    assert r2["triggered_by"] == {"fsm": 1, "autopilot_geofence": 3}, r2["triggered_by"]
    # Before the rule: k_001 and k_003 (Loiter) left out as censored, k_002
    # (Brake) scored as not triggered - a missed trigger.
    b0 = r2["before_backstop_rule"]
    assert (b0["correct"], b0["scored"], b0["left_out_as_censored"], b0["missed_triggers"]) == (
        2, 3, 2, 1), b0
    # All three counted as not triggered: two missed, the third a correct
    # non-trigger.
    b1 = r2["backstop_as_not_triggered"]
    assert (b1["correct"], b1["scored"], b1["missed_triggers"], b1["false_triggers"]) == (
        3, 5, 2, 0), b1
    text = "\n".join(S.rail_label_lines({"r2": r2}, {"built": {}, **labels}, S.RAIL_LABELS))
    assert "4/5 = 0.800" in text, text
    assert "3 of the scored fail-safes triggered by the autopilot's GeoFence backstop while " \
           "the FSM held (failsafe_by autopilot_geofence; counted since 2026-11-15): " \
           "2 correct, 1 false" in text, text
    assert "the same runs read as before 2026-11-15 (2 left out as censored): 2/3 = 0.667" \
        in text and "counted as not triggered: 3/5 = 0.600" in text, text
    assert "backstop   k_003  (free, seed 1001)  - the autopilot's GeoFence took it (RTL at " \
           "5.78 s) with the FSM in Loiter: no fail-safe expected, a false trigger" in text
    assert "censored   k_006  (want, seed 1001)  - the autopilot took the aircraft; flying " \
           "longer will not decide it" in text, text


def test_a_fetched_rail_run_is_rescored_offline_and_left_as_it_was():
    """--rescore-rail: a fetched run folder read again with this checkout's
    scorer - the campaign's own path (sitl_extra over the scenario's window,
    score(), episode_verdict), against the expansion's cell its scenario
    record names - with the verdict on every gate basis beside its stored
    one, the table in --out and nothing written beside the run. A folder
    with no scenario record is reported and exits 1."""
    from guardrail.scenario_spec import expand
    spec_id = "time-window-opens-on-vehicle"
    eps, _ = expand(LIBRARY, None, 0, [spec_id])
    ep = next(e for e in eps if e.cell.params.get("clock") == "2026-09-07T07:29:58"
              and e.cell.params.get("vehicle_speed_mps") == 4 and e.forecast == "grant"
              and e.response == "ideal")
    spec = ep.cell.spec
    run = S.run_episode(spec, DEFAULTS, ep.seed)
    meta = {"schema": "rail-scenario/1", "backend": "hil", "run_id": "rs_001_twoov",
            "run_nonce": "n1", "scenario_id": spec_id, "template": spec.template,
            "cell": ep.episode_id, "params": ep.cell.params, "seed": ep.seed, "arm": "on",
            "arm_tag": ep.arm_tag, "max_s": 30.0, "rail_max_s": 65.0}
    with tempfile.TemporaryDirectory() as td:
        rd = Path(td) / "rs_001_twoov"
        rd.mkdir()
        (rd / "flight_log.jsonl").write_text("".join(
            json.dumps(S._json_safe(r)) + "\n" for r in _rail_rows(spec, run)), encoding="utf-8")
        (rd / "metrics.json").write_text(json.dumps(S._json_safe({
            "shield": "on", "reached": run.extra["reached_goal"], "fsm": run.extra.get("fsm"),
            "scenario": meta})), encoding="utf-8")
        (rd / "policy_g0.json").write_bytes(run.policy.canonical_bytes())
        (rd / "scenario_score.json").write_text(json.dumps({
            "status": "fail", "gates_failed": ["anticipated_ticks_nfz-school: not measured"],
            "labels_failed": []}), encoding="utf-8")
        before = {p.name: p.read_bytes() for p in rd.iterdir()}
        out = Path(td) / "table.json"
        rc = S.main(["--seed", "0", "--only", spec_id, "--rescore-rail", str(rd),
                     "--rail-gates", "ardupilot-rest", "--out", str(out)])
        assert rc == 0, rc
        assert {p.name: p.read_bytes() for p in rd.iterdir()} == before, "the run was touched"
        item = json.loads(out.read_text(encoding="utf-8"))["runs"][0]
        assert item["before"]["status"] == "fail", item["before"]
        assert {k: v["status"] for k, v in item["after"].items()} == {
            "ideal": "pass", "rail": "pass", "ardupilot": "pass",
            "ardupilot-rest": "pass"}, item["after"]
        assert item["doc"]["gates_basis"] == "ardupilot-rest"
        assert item["doc"]["other_verdicts"]["ideal"]["status"] == "pass"
        assert item["doc"]["kpi"]["anticipated_ticks_nfz-school"] > 0
        # The fail-safe label table (2026-11-10). The committed artefact
        # holds the profiles' seeds, not the library's seed 0: the run is
        # named unlabelled under every reading, never counted as correct.
        fl = json.loads(out.read_text(encoding="utf-8"))["failsafe_labels"]
        assert fl["adopted_kpi_reading"] == "r2" and list(fl["readings"]) == ["a", "r", "r2"]
        for sc in fl["readings"].values():
            assert (sc["scored"], [t["run"] for t in sc["unlabelled"]]) == (0, ["rs_001_twoov"])
            assert sc["failsafe_trigger_correctness"] is None, sc
        # With labels for it: one reading each way, and one undecided.
        trig = item["doc"]["kpi"]["failsafe_triggered"]
        assert trig in (True, False), trig

        def lab(v):
            return {"expected": v, "clauses": ["E5"] if v else [], "e5_spell_s": 0.0,
                    "undecided": v is None}
        art = Path(td) / "labels.json"
        art.write_text(json.dumps({
            "schema": S.RAIL_LABELS_SCHEMA, "adopted_kpi_reading": "r2",
            "built": {"code_revisions": ["0" * 40]},
            "labels": {ep.episode_id: {str(ep.seed): {
                "a": lab(trig), "a2": lab(trig), "r": lab(None), "r2": lab(not trig)}}}}),
            encoding="utf-8")
        rc = S.main(["--seed", "0", "--only", spec_id, "--rescore-rail", str(rd),
                     "--rail-labels", str(art), "--out", str(out)])
        assert rc == 0, rc
        rd3 = json.loads(out.read_text(encoding="utf-8"))["failsafe_labels"]["readings"]
        assert (rd3["a"]["correct"], rd3["a"]["scored"]) == (1, 1), rd3["a"]
        assert rd3["a"]["failsafe_trigger_correctness"] == 1.0 and not rd3["a"]["adopted_kpi_reading"]
        assert rd3["r"]["scored"] == 0 and [t["run"] for t in rd3["r"]["undecided"]] == [
            "rs_001_twoov"], rd3["r"]
        assert (rd3["r2"]["correct"], rd3["r2"]["scored"]) == (0, 1) and rd3["r2"][
            "adopted_kpi_reading"], rd3["r2"]
        wrong = rd3["r2"]["false" if trig else "missed"]
        assert [(t["run"], t["cell"], t["seed"]) for t in wrong] == [
            ("rs_001_twoov", ep.episode_id, ep.seed)], rd3["r2"]
        assert {p.name: p.read_bytes() for p in rd.iterdir()} == before, "the run was touched"
        # No artefact, no score: said, and the exit code is 1.
        rc = S.main(["--seed", "0", "--only", spec_id, "--rescore-rail", str(rd),
                     "--rail-labels", str(Path(td) / "none.json"), "--out", str(out)])
        assert rc == 1 and "failsafe_labels" not in json.loads(out.read_text(encoding="utf-8"))
        bad = Path(td) / "rs_002_bare"
        bad.mkdir()
        (bad / "metrics.json").write_text("{}", encoding="utf-8")
        rc = S.main(["--seed", "0", "--only", spec_id, "--rescore-rail", str(bad),
                     "--out", str(out)])
        assert rc == 1 and "no scenario record" in out.read_text(encoding="utf-8")


def test_the_fsm_edges_profile_flies_the_edges_no_rail_had_taken():
    """experiments/profiles/fsm_edges.yaml (2026-11-14, WP3-07): the KPI
    nightly ntFIN2 never took G2, G4, G6, X2 or X3. Its six cells take them
    headless on the ideal vehicle and on the rail vehicle (RAIL_VEHICLE,
    the rig's fitted response), and no rail refuses any of them - the
    blocked return home included, now that the node's RTL watch feeds G6.
    The three new families are flown by this profile alone, so no library,
    smoke or nightly number moves. fsm-n-in-t-flicker carried no fail-safe
    label until 2026-11-15 (the rubric's N1 and the grant's G4 disagreed on
    it); since then the grant's G4 governs, as the rubric's clause E6
    (docs/RUBRIC-failsafe.md section 9): expected, with no outcome label."""
    from guardrail.scenario_spec import expand, load_profile
    prof = load_profile(ROOT / "experiments" / "profiles" / "fsm_edges.yaml")
    assert prof.grant_kpi_bearing is False
    eps, info = expand(LIBRARY, prof)
    # Six since 2026-11-17: failsafe-low-battery, G6 for the battery.
    assert info["cells"] == 6 and info["cells_within_expected"] and not info["duplicate_cells"]
    new = ("failsafe-rtl-from-hold", "failsafe-land-from-hold", "fsm-n-in-t-flicker")
    for name in ("smoke", "nightly"):
        _, other = expand(LIBRARY, load_profile(ROOT / "experiments" / "profiles" / f"{name}.yaml"))
        assert not set(new) & set(other["cells_per_family"]), name
    _, lib = expand(LIBRARY)
    assert not set(new) & set(lib["cells_per_family"])
    flick = BY_ID["fsm-n-in-t-flicker"]
    assert (flick.expected_failsafe, flick.failsafe_clause, flick.expected_outcome) == (
        True, "E6", None)
    assert [(BY_ID[s].expected_failsafe, BY_ID[s].failsafe_clause, BY_ID[s].expected_outcome)
            for s in new[:2]] == [(True, "E1", "RTL_triggered"), (True, "E1", "Land_triggered")]
    want = {"failsafe-rtl-from-hold": (["X2"], "RTL_triggered"),
            "failsafe-land-from-hold": (["X3"], "Land_triggered"),
            "fsm-n-in-t-flicker": (["G2", "G4"], "RTL_triggered"),
            "failsafe-rtl-blocked": (["G11", "G6"], "Land_triggered"),
            "failsafe-low-battery": (["G11", "G6", "G10"], "Land_triggered")}
    seen = set()
    for e in eps:
        spec = e.cell.spec
        for backend in S.RAIL_BACKENDS:
            # The profile opts in to the battery cell (rail_battery: true).
            assert S.rail_refusal(spec, backend, rail_battery=prof.rail_battery) is None, (
                e.cell.episode_id, backend)
        sid = e.cell.episode_id.split("@")[0]
        if sid not in want:
            assert sid == "wind-beyond-authority" and (
                e.cell.params["lane_y"], e.cell.params["wind_speed_mps"]) == (15.0, 12.0), \
                e.cell.params
            continue
        edges, outcome = want[sid]
        for arm in ({}, dict(S.RAIL_VEHICLE)):
            run = S.run_episode(spec, DEFAULTS, e.seed, **arm)
            fsm = run.extra["fsm"]
            taken = [t["edge"] for t in fsm["transitions"]]
            it = iter(taken)
            assert all(edge in it for edge in edges), (sid, arm, taken)
            assert fsm["outcome_label"] == outcome, (sid, arm, fsm["outcome_label"])
            assert run.extra["breaches"] == 0 and run.extra["polygon_entries"] == 0, (
                sid, arm, run.extra["breaches"], run.extra["polygon_entries"])
            if sid in ("failsafe-rtl-blocked", "failsafe-low-battery"):
                assert run.extra["rtl_failed_ticks"] >= 1
        seen.add(sid)
    assert seen == set(want), seen


# --------------------------------------------------------------------------- #
# THE GAZEBO FUNCTIONAL RAIL (2026-11-17, WP4-04)
# --------------------------------------------------------------------------- #

GAZEBO_PROFILE = ROOT / "experiments" / "profiles" / "gazebo_functional.yaml"


def _rail_dirs() -> set:
    return {p for d in (S.CAMPAIGN_DIR, S.RAIL_STRESS_DIR) if (ROOT / d).is_dir()
            for p in (ROOT / d).iterdir()}


def test_the_gazebo_profile_dry_run_lists_every_mission_without_wsl_or_gazebo():
    """--profile gazebo_functional --backend sitl --sitl-dry-run: the eight
    smoke cells as missions on the Gazebo rail - "gzsweep_" tags, PHYSICS=
    gazebo in the environment, the run folder and replay bundle each will
    leave, the grant's episode name as a label that names no file - and the
    wall-time estimate from the caps in wall seconds; nothing launched
    (neither WSL nor subprocess is touched), nothing written, exit 0. A
    speedup other than 1 is refused (the Shield node runs on wall time)."""
    from guardrail.scenario_spec import load_profile
    prof = load_profile(GAZEBO_PROFILE)
    assert (prof.rail_physics, prof.rail_sim_speedup, prof.grant_kpi_bearing) == (
        "gazebo", 1.0, False)
    saved = (S.sitl_available, S.subprocess.run)

    def boom(*a, **k):
        raise AssertionError("the dry run launched something")
    before = _rail_dirs()
    buf = io.StringIO()
    try:
        S.sitl_available, S.subprocess.run = boom, boom
        with contextlib.redirect_stdout(buf):
            rc = _sweep(["--profile", "gazebo_functional", "--backend", "sitl",
                         "--sitl-dry-run"])
    finally:
        S.sitl_available, S.subprocess.run = saved
    out = buf.getvalue()
    assert rc == 0 and _rail_dirs() == before, (rc, _rail_dirs() - before)
    assert "8 mission(s) to fly, 0 refused" in out, out[:400]
    assert "FUNCTIONAL, NOT KPI-GRADE" in out
    assert out.count("env        PHYSICS=gazebo") == 8 and "SITL_SPEEDUP" not in out
    for cell in ("nfz-head-on", "nfz-head-on-control", "speed-cap", "altitude-floor-recovery",
                 "corridor-along", "time-window-switch-event"):
        assert f"FLY  gzsweep_{cell}--seed1001  {cell}  seed=1001" in out, cell
        assert re.search(rf"grant name episode-\S+Z--{re.escape(cell)}--seed1001  \(the "
                         rf"episode bundle, written after the flight\)\n", out), cell
        assert (f"run folder demo/out/gzsweep_{cell}--seed1001  (replay demo/out/gzsweep_"
                f"{cell}--seed1001/gzsweep_{cell}--seed1001.replay.tar.gz)") in out, cell
    assert "wsl bash sitl/run_ros2_demo.sh off '' --tag gzsweep_nfz-head-on-control" in out
    assert "grant names: " + S.GRANT_EPISODE_NAME_NOTE in out
    # since 2026-11-18 the rail writes the episode's bundle under that name
    assert "WP4-11's per-episode bundle naming stays open" not in out
    assert "the grant's six file names" in out
    assert "bundle     " not in out
    # The estimate: the caps are wall seconds (the node's clock), summed
    # as they are, plus the overhead per mission.
    plan = S.sitl_plan(S.expand(LIBRARY, prof)[0], DEFAULTS, "gazebo")
    caps = sum(p["rail_max_s"] for p in plan if p["fly"])
    est = (caps + S.RAIL_MISSION_OVERHEAD_S * 8) / 60
    assert (f"estimate: at most {est:.0f} min wall ({caps / 60:.1f} min of flight caps, "
            in out), out[-400:]
    assert "an estimate, not a measurement" in out and "caps at x" not in out
    # Twice the speed (or half) is refused, before anything is listed.
    for argv in (["--profile", "gazebo_functional", "--backend", "sitl", "--sitl-dry-run",
                  "--rail-sim-speedup", "2"],
                 ["--profile", "gazebo_functional", "--backend", "sitl", "--sitl-dry-run",
                  "--rail-sim-speedup", "0.5"]):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            assert _sweep(argv) == 2, argv
        assert "REFUSED" in buf.getvalue() and "wall time" in buf.getvalue(), buf.getvalue()
        assert "FLY  " not in buf.getvalue()


def test_the_gazebo_rail_refuses_what_it_cannot_fly_and_the_sitl_plan_is_unchanged():
    """sitl_plan on the smoke profile: on SITL's own physics the tags are
    "sweep_" and the environment holds SITL_HOME alone (the launch run_sitl
    always made); on Gazebo the cells with SITL's stressors (wind, GPS
    dropout) and the target-less cells with a start yaw off north are
    refused, each saying why, and every other cell flies as on SITL."""
    from guardrail.scenario_spec import load_profile
    eps, _ = S.expand(LIBRARY, load_profile(ROOT / "experiments" / "profiles" / "smoke.yaml"))
    sitl = {p["id"]: p for p in S.sitl_plan(eps, DEFAULTS)}
    gz = {p["id"]: p for p in S.sitl_plan(eps, DEFAULTS, "gazebo")}
    assert sitl.keys() == gz.keys()
    for pid, p in sitl.items():
        if p["fly"]:
            assert p["tag"].startswith("sweep_") and set(p["env"]) <= {"SITL_HOME"}, p
            assert p["cmd"][:3] == ["wsl", "bash", S.SITL_SCRIPT] and not p["functional_only"]
    newly = {pid for pid in sitl if sitl[pid]["fly"] and not gz[pid]["fly"]}
    assert all(not sitl[pid]["fly"] for pid in gz if not gz[pid]["fly"] and pid not in newly)
    stressed = {pid for pid in newly if "simulator stressors" in gz[pid]["refused"]}
    turned = {pid for pid in newly if "start yaw" in gz[pid]["refused"]}
    assert stressed | turned == newly and stressed and turned, newly
    assert {pid.split("@")[0] for pid in stressed} == {
        "gps-dropout", "wind-within-authority", "wind-beyond-authority"}, stressed
    assert all(sitl[pid]["stress"] for pid in stressed)
    for pid, p in gz.items():
        if p["fly"]:
            assert p["tag"] == "gz" + sitl[pid]["tag"] and p["functional_only"], pid
            assert p["env"] == dict(sitl[pid]["env"], PHYSICS="gazebo"), pid
            # The same mission, under its own tag.
            assert p["cmd"] == [c.replace(sitl[pid]["tag"], p["tag"])
                                for c in sitl[pid]["cmd"]], pid
            assert p["grant_episode_name"] == sitl[pid]["grant_episode_name"]


def test_a_gazebo_mission_is_marked_functional_and_never_kpi_grade():
    """rail_meta on Gazebo: `functional_only` with why, the rail and the
    speedup asked for, a "gazebo-stress/" family (tools/kpi_report.py files
    it so); on SITL's own physics the record is the one it always was. The
    node's metrics carry the record, and is_kpi_grade refuses it whatever
    the manifest reads - even under a dev waiver at speedup 1.0 - saying it
    is the lab's choice, not the grant's wording. Any speedup but 1.0 is
    refused, on Gazebo too: the Shield node runs on wall time."""
    sys.path.insert(0, str(ROOT / "tools"))
    import kpi_report
    from guardrail.manifest import is_kpi_grade
    spec = BY_ID["speed-cap"]
    plain = S.rail_meta(spec, None, 1001, "sitl", "t1", "n", None, None, DEFAULTS)
    again = S.rail_meta(spec, None, 1001, "sitl", "t1", "n", None, None, DEFAULTS,
                        physics="sitl", sim_speedup=1.0)
    assert plain == again
    assert not {"rail", "physics", "functional_only", "sim_speedup_requested"} & set(plain)
    assert plain["family"].startswith("sitl-stress/")
    assert plain["harness"] == "experiments/sweep_scenarios.py --backend sitl"
    gz = S.rail_meta(spec, None, 1001, "sitl", "t1", "n", None, None, DEFAULTS,
                     physics="gazebo", sim_speedup=1.0)
    assert gz["functional_only"] is True and gz["rail"] == "gazebo"
    assert gz["sim_speedup_requested"] == 1.0 and gz["physics"] == "gazebo-harmonic"
    assert gz["family"] == plain["family"].replace("sitl-stress/", "gazebo-stress/", 1)
    assert "--rail-physics gazebo --rail-sim-speedup 1" in gz["harness"]
    # The reason says whose choice it is and does not lean on the broad
    # sweep's sentence (which covers speed > 1 only).
    for text in (gz["functional_only_why"], S.GAZEBO_FUNCTIONAL_WHY):
        assert "the lab's choice" in text and "not the grant's wording" in text, text
        assert "dual-rail.md" in text and "Test scheduling" not in text, text
    assert kpi_report.scenario_family(gz, "on") == kpi_report.scenario_family(
        plain, "on").replace("sitl-stress/", "gazebo-stress/", 1)
    assert kpi_report.scenario_family(plain, "on").startswith("sitl-stress/")
    man = {"code_revision": "abc123", "vla_model_hash": "stub@src:1",
           "policy_hash": "sha256:1", "random_seed": 1001, "sim_speedup": 1.0,
           "topology": "dev"}
    ok, why = is_kpi_grade(man, {"scenario": plain}, dev_waiver="test waiver")
    assert ok, why
    ok, why = is_kpi_grade(man, {"scenario": gz}, dev_waiver="test waiver")
    assert not ok and len(why) == 1 and "functional rail" in why[0], why
    assert "the lab's choice" in why[0] and "not the grant's wording" in why[0], why
    assert "Test scheduling" not in why[0], why
    for bad in (("hil", "gazebo", 1.0), ("sitl", "sitl", 2.0), ("sitl", "gazebo", 0.0),
                ("sitl", "gazebo", 2.0), ("sitl", "gazebo", 1.5), ("sitl", "gazebo", 0.5),
                ("sitl", "gazebo", 1e-05), ("sitl", "gazebo", float("nan")),
                ("sitl", "gazebo", "x"), ("sitl", "gazebo", 5.0), ("sitl", "ode", 1.0)):
        try:
            S.check_rail_physics(*bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")


def test_run_sitl_on_gazebo_launches_with_physics_and_checks_what_the_rail_wrote():
    """run_sitl on Gazebo with the rail's launch replaced: PHYSICS goes to
    WSL through WSLENV, SITL_SPEEDUP never; the run is accepted only when
    THIS launch wrote physics.json saying gazebo-harmonic at the speedup
    asked for and the node's metrics carry the functional record - a stale
    physics.json or a scenario record not marked functional is an error. A
    speedup other than 1 is refused before anything launches. On SITL's own
    physics neither variable is set, as before."""
    spec = BY_ID["rail-mission-stub-vla"]
    run = S.run_episode(spec, DEFAULTS, 0)
    rows = _as_rail_rows(run.rows)
    saved = (S.ROOT, S.sitl_available, S.build_sitl_cmd, S.subprocess.run)
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        shutil.copytree(ROOT / "policies", root / "policies")
        seen: dict = {}
        physics = {"write": {"physics": "gazebo-harmonic", "sim_speedup_requested": 1},
                   "functional": True, "age_s": 0.0}

        def fake_run(cmd, **kw):
            seen.clear()
            seen.update(kw.get("env") or {})
            tag = cmd[1]
            out = root / "demo" / "out" / tag
            out.mkdir(parents=True, exist_ok=True)
            (out / "flight_log.jsonl").write_text(
                "".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
            meta = json.loads((root / S.CAMPAIGN_DIR / tag / "scenario.json").read_text(
                encoding="utf-8"))
            if not physics["functional"]:
                meta = {k: v for k, v in meta.items() if k != "functional_only"}
            (out / "metrics.json").write_text(json.dumps(
                {"reached": run.extra["reached_goal"], "fsm": run.extra["fsm"],
                 "scenario": meta}), encoding="utf-8")
            p = out / "physics.json"
            if p.exists():
                p.unlink()
            if physics["write"] is not None:
                p.write_text(json.dumps(physics["write"]), encoding="utf-8")
                if physics["age_s"]:
                    old = time.time() - physics["age_s"]
                    os.utime(p, (old, old))
            return subprocess.CompletedProcess(cmd, 0, "", "")
        try:
            S.ROOT, S.sitl_available = root, (lambda: True)
            S.build_sitl_cmd = lambda spec_, tag, *a, **k: ["fake", tag]
            S.subprocess.run = fake_run
            _, extra = S.run_sitl(spec, "gzsweep_t", 0, DEFAULTS, physics="gazebo")
            assert seen["PHYSICS"] == "gazebo" and "SITL_SPEEDUP" not in seen
            assert "PHYSICS/u" in seen["WSLENV"].split(":")
            assert "SITL_SPEEDUP/u" not in seen["WSLENV"].split(":")
            assert extra["rail_physics"]["physics"] == "gazebo-harmonic"
            good = physics["write"]
            for change, says in (({"write": {"physics": "ardupilot-sitl-builtin",
                                             "sim_speedup_requested": 1}},
                                  "not 'gazebo-harmonic'"),
                                 ({"write": {"physics": "gazebo-harmonic",
                                             "sim_speedup_requested": 2}},
                                  "sim_speedup_requested"),
                                 ({"write": None}, "no physics.json"),
                                 ({"age_s": 600.0}, "physics.json predates this launch"),
                                 ({"functional": False},
                                  "metrics.json's scenario record is not marked "
                                  "functional_only")):
                physics.update({"write": good, "functional": True, "age_s": 0.0})
                physics.update(change)
                try:
                    S.run_sitl(spec, "gzsweep_t", 0, DEFAULTS, physics="gazebo")
                except RuntimeError as e:
                    assert "not a Gazebo functional-rail flight" in str(e) and says in str(e), e
                else:
                    raise AssertionError(f"accepted {change}")
            physics.update({"write": good, "functional": True, "age_s": 0.0})
            # Faster than real time: refused before the launch, on any physics.
            for phys in ("gazebo", "sitl"):
                seen.clear()
                try:
                    S.run_sitl(spec, "gzsweep_t", 0, DEFAULTS, physics=phys, sim_speedup=2.0)
                except ValueError as e:
                    assert "wall time" in str(e) and not seen, e
                else:
                    raise AssertionError(f"flew at 2x on {phys}")
            physics["write"] = None
            _, extra = S.run_sitl(spec, "sweep_t", 0, DEFAULTS)
            assert "PHYSICS" not in seen and "SITL_SPEEDUP" not in seen
            assert "PHYSICS/u" not in seen.get("WSLENV", "") and "rail_physics" not in extra
            # A stressed cell never reaches the launch on Gazebo.
            seen.clear()
            wind = next(e for e in S.expand(LIBRARY, None)[0]
                        if e.cell.scenario_id == "wind-within-authority")
            try:
                S.run_sitl(wind.cell.spec, "gzsweep_w", wind.seed, DEFAULTS, episode=wind,
                           physics="gazebo")
            except S.UnsupportedScenario as e:
                assert "simulator stressors" in str(e) and not seen, e
            else:
                raise AssertionError("a wind cell was flown on Gazebo")
        finally:
            S.ROOT, S.sitl_available, S.build_sitl_cmd, S.subprocess.run = saved


def test_a_gazebo_sweep_records_each_missions_run_and_says_it_is_not_kpi():
    """main() on the Gazebo rail, with the flight replaced: the results say
    functional, not KPI (`_not_kpi_grade`, `kpi_bearing_this_run` false,
    `_scope.rail`), and each episode names its mission tag, run folder,
    replay bundle and - as a label, with the note that no file carries it -
    the grant's episode name. The profile refuses every backend but sitl,
    and the rail options refuse any speedup but 1."""
    calls = []
    saved = S.run_sitl

    def fake(spec, tag, seed=0, defaults=None, episode=None, **kw):
        calls.append((tag, kw))
        run = S.run_episode(spec, defaults, seed)
        return _as_rail_rows(run.rows), run.extra
    try:
        S.run_sitl = fake
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "s.json"
            with contextlib.redirect_stdout(io.StringIO()):
                rc = _sweep(["--profile", "gazebo_functional", "--backend", "sitl",
                             "--only", "speed-cap", "--out", str(out),
                             "--runs", str(Path(td) / "runs")])
            doc = json.loads(out.read_text(encoding="utf-8"))
    finally:
        S.run_sitl = saved
    assert rc in (0, 1) and calls == [("gzsweep_speed-cap--seed1001",
                                       {"physics": "gazebo", "sim_speedup": 1.0,
                                        "rail_battery": False,
                                        "extra_args": None})], calls
    assert doc["_not_kpi_grade"] == S.GAZEBO_FUNCTIONAL_WHY
    assert doc["_scope"]["kpi_bearing_this_run"] is False
    assert doc["_scope"]["rail"]["physics"] == "gazebo"
    assert doc["_scope"]["rail"]["functional_only"] is True
    (ep,) = [r for r in doc["results"] if r["id"] == "speed-cap"]
    assert ep["run"] == "demo/out/gzsweep_speed-cap--seed1001" and ep["functional_only"]
    assert re.fullmatch(r"episode-\S+Z--speed-cap--seed1001", ep["grant_episode_name"]), ep
    assert ep["grant_episode_name_note"] == S.GRANT_EPISODE_NAME_NOTE
    assert "the grant's six file names" in ep["grant_episode_name_note"]
    assert ep["replay_bundle"] == ("demo/out/gzsweep_speed-cap--seed1001/"
                                   "gzsweep_speed-cap--seed1001.replay.tar.gz")
    assert "episode_bundle_name" not in ep
    with contextlib.redirect_stdout(io.StringIO()):
        assert _sweep(["--profile", "gazebo_functional", "--only", "speed-cap"]) == 2
        assert _sweep(["--profile", "gazebo_functional", "--backend", "hil",
                       "--only", "speed-cap", "--hil-dry-run"]) == 2
        assert _sweep(["--only", "speed-cap", "--backend", "sitl", "--rail-sim-speedup",
                       "2", "--sitl-dry-run"]) == 2
        assert _sweep(["--only", "speed-cap", "--sitl-dry-run"]) == 2
        assert _sweep(["--profile", "gazebo_functional", "--backend", "sitl", "--only",
                       "speed-cap", "--rail-sim-speedup", "2", "--sitl-dry-run"]) == 2


def test_a_profile_flies_at_speedup_1_and_only_gazebo_names_a_rail():
    """Profile: rail_sim_speedup 1.0 only, on the functional rail too (the
    Shield node runs on wall time); rail_physics gazebo needs
    grant_kpi_bearing false; every shipped profile but gazebo_functional
    names no rail, and every one flies at 1.0."""
    from guardrail.scenario_spec import Profile, load_profile
    base = yaml.safe_load(GAZEBO_PROFILE.read_text(encoding="utf-8"))
    Profile.model_validate(base)
    Profile.model_validate(dict(base, rail_sim_speedup=1))
    for bad in (dict(base, rail_physics=None, rail_sim_speedup=2.0),
                dict(base, rail_sim_speedup=2.0), dict(base, rail_sim_speedup=0.5),
                dict(base, grant_kpi_bearing=True),
                dict(base, rail_sim_speedup=4.5), dict(base, rail_physics="ode")):
        try:
            Profile.model_validate(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")
    for p in sorted((ROOT / "experiments" / "profiles").glob("*.yaml")):
        prof = load_profile(p)
        assert prof.rail_sim_speedup == 1.0, p.name
        if p.stem != "gazebo_functional":
            assert prof.rail_physics is None, p.name


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
