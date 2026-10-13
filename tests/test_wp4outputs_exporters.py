"""The episode exporters (tools/export_episodes.py) and the re-run tool
(tools/rerun_episode.py), 2026-10-12.

Run either way:
    pytest tests/test_wp4outputs_exporters.py -v
    python tests/test_wp4outputs_exporters.py

WHAT EACH TEST GUARDS

  * A headless episode exports one step per tick with the grant's names, and
    a mid-flight hot-apply moves `policy_state` to generation 1 exactly at
    the tick it flew; its null (the same episode without the zone) stays at
    generation 0 and has no fence label.
  * A rail run folder: actions and policy state fall back to the audit
    record, read in the shape before record version 2 (`tick` only) and in
    version 2's (`monitor_tick`, a violation's `priority` and
    `grant_category`); `executed` follows the rail's verdict (flown_action,
    a brake, nothing streamed, the raw action on a Shield-off arm);
    risk_level comes from the run's policy generations, and an unresolved
    rule is labelled P0 and says so. A version-2 audit written by
    guardrail/audit.py's own AuditLogger, 0-based against a 1-based log as
    the ROS 2 rail writes it, is joined one tick on, and its time-window
    category and priority reach the row's violations.
  * An audit.jsonl that a reused run folder appended across flights joins
    only the flight whose raw actions are the log's; a log no flight
    matches joins nothing, and says why.
  * A replay bundle is verified before anything is exported, and a member
    changed after packing is refused.
  * JSONL is strict (a NaN is a string), Parquet is written or SKIPPED,
    never faked, and `--format parquet` alone without pyarrow writes nothing
    and exits 3; the nearest bag message is read through `mcap` when it is
    installed and SKIPPED when it is not.
  * A headless bundle without manifest_extras.json takes its generation
    timeline from harness_events.jsonl, a refused hot-apply left out; the
    outcome on every step is the stored table's; a braked or autopilot tick
    has no post-repair action.
  * rerun_episode: the re-flown log is byte-identical (from manifest_extras
    and from the library), queued gRPC events included; the null - another
    seed - differs, and a tampered log is caught at the line it was changed.
    A changed policy is "cannot compare" (exit 2), a policy file that is
    gone is "cannot rebuild" (exit 2, not a traceback read as "differs"),
    a log-less bundle's final position is compared, and a scenario that
    draws nothing from the seed says so.

All of it is new: on the 4418ae0 copy both tools are absent and every test
here fails at its import. Of the tests added on 2026-10-13, four also fail
on the 2026-10-12 version of the tools: the version-2 writer's audit and the
appended audit (it joined records by tick alone, first record wins, so
against a 0-based audit each row got the next tick's record and the last
row none, and an appended file labelled a flight with the oldest flight's
records), parquet alone (it exited 0 having
written nothing) and the policy file that is gone (a traceback, exit 1:
"differs"). The others - the events timeline, the outcome, the action edge
cases, the changed-policy guard, the final position, the unconsumed seed -
pin behaviour that was right but that a deliberate break got past. The nulls inside each test (another seed, no
hot-apply, a tampered member) are what keep them from passing a tool that
copies its input.
"""
import contextlib
import io
import json
import math
import os
import shutil
import sys
import tarfile
import tempfile
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
sys.path.insert(0, str(ROOT / "tools"))

import export_episodes as X                                        # noqa: E402
import rerun_episode as RR                                         # noqa: E402
from guardrail import harness_service as H                       # noqa: E402
from guardrail import load_policy                                  # noqa: E402

SKIP = "SKIP"
_TMP: list[Path] = []
A0 = {"vx": 0.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}
POLICY = ROOT / "policies" / "sim_demo_policy.yaml"


def _tmp(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


OPEN = """
scenario_id: wp4-open-flight
template: dynamic_nfz_movement
description: "Across open ground at 15 m; nothing in the way unless an event puts it there."
mission:
  start_pose: {x: -20.0, y: -20.0, up: 15.0}
  target: {x: 40.0, y: -20.0, up: 15.0}
  pilot: {type: goto, speed: 4.0}
policy: {path: policies/sim_demo_policy.yaml}
expected_kpis:
  P0_escape_rate: 0
"""
JITTER = """
scenario_id: wp4-jitter
template: static_nfz
description: "Head-on into the demo zone from a start drawn from the seed."
mission:
  start_pose: {x: -20.0, y: 15.0, up: 15.0}
  target: {x: 40.0, y: 15.0, up: 15.0}
  pilot: {type: goto, speed: 4.0}
policy: {path: policies/sim_demo_policy.yaml}
stress: {start_jitter_m: 3.0}
expected_kpis:
  P0_escape_rate: 0
"""
ZONE = {"type": "dynamic_nfz", "id": "nfz-hot",
        "vertices": [{"x": 10, "y": -30}, {"x": 20, "y": -30},
                     {"x": 20, "y": -10}, {"x": 10, "y": -10}]}

_H: dict = {}


def _harness() -> H.Harness:
    if "h" not in _H:
        _H["h"] = H.Harness(runs_dir=_tmp("wp4_runs_"))
    return _H["h"]


def _fly(yaml_text="", *, scenario_id="", hot=(), seed=None, keep=True) -> Path:
    h = _harness()
    ref = h.start_episode(scenario_id=scenario_id, scenario_yaml=yaml_text, seed=seed,
                          keep_flight_log=keep)
    for at, body in hot:
        h.hot_apply(ref["episode_id"], at, body)
    return Path(h.end_episode(ref["episode_id"])["bundle_path"])


def _export(path) -> tuple[list[dict], dict]:
    return X.export(X.load_episode(path))


# --------------------------------------------------------------------------- #
# the step and episode records
# --------------------------------------------------------------------------- #

def test_a_headless_episode_exports_every_tick_with_the_grants_names():
    hot = _fly(OPEN, hot=[(3.0, ZONE)])
    null = _fly(OPEN)
    steps, ep = _export(hot)
    rows = (hot / "flight_log.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(steps) == len(rows) == ep["n_steps"] > 0
    for s in steps:
        assert set(s["action"]) >= {"planner_output", "post_repair", "executed"}
        assert {"policy_hash", "generation"} <= set(s["policy_state"])
        assert s["observation"]["ref"].endswith(f":L{s['step'] + 1}")
        assert s["outcome"] in ("success", "fail", "RTL_triggered", "Land_triggered")
    before = {s["policy_state"]["generation"] for s in steps if s["t"] < 3.0 - 1e-9}
    after = {s["policy_state"]["generation"] for s in steps if s["t"] >= 3.0 - 1e-9}
    assert before == {0} and after == {1}, (before, after)
    hashes = {s["policy_state"]["policy_hash"] for s in steps}
    assert len(hashes) == 2, hashes
    fenced = [s for s in steps if "fence" in s["violation_type"]]
    assert fenced and all(s["risk_level"] == "P0" for s in fenced), fenced[:1]
    assert all(not s["risk_level_defaulted"] for s in steps)
    assert ep["labels"]["max_risk_level"] == "P0"
    # the outcome on every step is the stored table's, not a constant
    stored = json.loads((hot / "kpi.json").read_text(encoding="utf-8"))["kpi"]
    assert {s["outcome"] for s in steps} == {stored["mission_outcome"]}, stored
    assert ep["trajectory"]["generation"][0] == 0 and ep["trajectory"]["generation"][-1] == 1
    # the null: no hot-apply, one generation, nothing to label as a fence
    nsteps, nep = _export(null)
    assert {s["policy_state"]["generation"] for s in nsteps} == {0}
    assert not any("fence" in s["violation_type"] for s in nsteps)
    assert nep["labels"]["max_risk_level"] is None


def test_headless_post_repair_is_only_the_emitted_action_on_a_shield_flown_tick():
    """In a headless row `emitted` is what FLEW: the Shield-off control's raw
    action is not a post-repair action, and the export must not call it one."""
    bundle = _fly(scenario_id="nfz-head-on-control", seed=1001)
    steps, ep = _export(bundle)
    assert ep["shield"] == "off"
    # the control flies into the zone: its outcome is "fail", on every step
    stored = json.loads((bundle / "kpi.json").read_text(encoding="utf-8"))["kpi"]
    assert stored["mission_outcome"] == "fail" == ep["outcome"], stored["mission_outcome"]
    assert {s["outcome"] for s in steps} == {"fail"}
    assert all(s["action"]["post_repair"] is None for s in steps)
    assert all(s["action"]["executed"] == s["action"]["planner_output"] for s in steps)
    assert all(s["action"]["executed_source"] == "headless:pilot" for s in steps)
    on, _ = _export(_fly(scenario_id="nfz-head-on", seed=1001))
    flew = [s for s in on if s["action"]["executed_source"] == "headless:shield"
            and s["action"]["post_repair"] is not None]
    assert flew and all(s["action"]["post_repair"] == s["action"]["executed"] for s in flew)


def _rail_run(root: Path) -> Path:
    run = root / "rail"
    run.mkdir(parents=True)
    pol = load_policy(POLICY)
    (run / "policy_g0.json").write_text(pol.model_dump_json(), encoding="utf-8")
    nfz = {"rule_id": "nfz-square", "category": "geofence"}
    ghost = {"rule_id": "rule-nobody-wrote", "category": "standoff"}
    rows = [
        # 1: flown_action recorded (the pymavlink rail)
        {"t": 0.1, "tick": 1, "x": 0.0, "y": 0.0, "up": 15.0, "raw": dict(A0, vx=4.0),
         "emitted": dict(A0, vx=3.0), "flown_action": dict(A0, vx=3.0), "flown": True,
         "violations": [nfz], "repairs": [], "policy_hash": pol.policy_hash, "generation": 0},
        # 2: the FSM braked: zero streamed
        {"t": 0.2, "tick": 2, "x": 0.1, "y": 0.0, "up": 15.0, "raw": dict(A0, vx=4.0),
         "emitted": dict(A0, vx=3.0), "setpoint": "brake", "flown": True,
         "violations": [nfz], "repairs": [], "policy_hash": pol.policy_hash, "generation": 0},
        # 3: nothing streamed; the autopilot held the aircraft
        {"t": 0.3, "tick": 3, "x": 0.1, "y": 0.0, "up": 15.0, "raw": dict(A0, vx=4.0),
         "emitted": dict(A0, vx=3.0), "setpoint": "none", "flown": False,
         "violations": [], "repairs": [], "policy_hash": pol.policy_hash, "generation": 0},
        # 4: a row that logged nothing but the pose: the audit record
        #    answers, in record version 2's shape (monitor_tick, a
        #    violation's own priority and grant category)
        {"t": 0.4, "tick": 4, "x": 0.2, "y": 0.0, "up": 15.0},
        # 5: the same, answered by a record of the shape before version 2
        {"t": 0.5, "tick": 5, "x": 0.3, "y": 0.0, "up": 15.0},
    ]
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    ts = "2026-10-13T01:00:00.{:03d}+00:00"
    # Rows 1-3 carry their raw actions; their records match them, which is
    # what lets the join trust this audit for rows 4 and 5 too.
    audit = [{"ts": ts.format(100 * r["tick"]), "tick": r["tick"],
              "policy_hash": pol.policy_hash, "raw_action": r["raw"],
              "emitted_action": r["emitted"], "violations": r["violations"],
              "repairs": [], "braked": False} for r in rows[:3]]
    audit += [
        {"ts": ts.format(400), "monitor_tick": 4, "tick": 4, "episode_id": None,
         "policy_hash": pol.policy_hash, "generation": 0, "record_version": 2,
         "raw_action": {"vx": 5.0, "vy": 0.0, "vz_up": 1.0, "yaw_rate": 0.0},
         "emitted_action": {"vx": 2.0, "vy": 0.0, "vz_up": 0.5, "yaw_rate": 0.0},
         # the grant's body-frame keys: never read as world velocities
         "raw_action_body": {"vx": 9.0, "vy": 9.0, "vz": 9.0, "yaw_rate": 0.0},
         "violations": [{"rule_id": "nfz-square", "category": "geofence",
                         "grant_category": "geometric", "priority": "P0"},
                        {"rule_id": "alt-band", "category": "altitude",
                         "grant_category": "envelope", "priority": "P1"},
                        # a window edge in the lookahead, and the rule's
                        # priority copied into the record
                        {"rule_id": "kin-caps", "category": "kinematic",
                         "grant_category": "time-window", "priority": "P2"}],
         "repairs": [{"operator": "AltitudeFix"}],
         "repair_attempts": [{"operator": "AltitudeFix", "result": "ok"}]},
        {"ts": ts.format(500), "tick": 5, "policy_hash": pol.policy_hash,
         "raw_action": {"vx": 6.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0},
         "emitted_action": {"vx": 1.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0},
         "violations": [{"rule_id": "kin-caps", "category": "kinematic"}, ghost],
         "repairs": [{"operator": "SpeedClamp"}], "braked": False},
    ]
    (run / "audit.jsonl").write_text("".join(json.dumps(a) + "\n" for a in audit),
                                     encoding="utf-8")
    (run / "manifest.json").write_text(json.dumps({"policy_hash": pol.policy_hash}),
                                       encoding="utf-8")
    return run


def test_a_rail_run_reads_the_audit_in_old_and_new_keys_and_the_rails_verdict():
    run = _rail_run(_tmp("wp4_rail_"))
    steps, ep = _export(run)
    assert ep["kind"] == "run" and ep["priority_source"].startswith("the run's policy_g")
    s1, s2, s3, s4, s5 = steps
    assert s1["action"]["executed"]["vx"] == 3.0 and s1["action"]["executed_source"] == "flown_action"
    assert s2["action"]["executed"] == X.ZERO and s2["action"]["post_repair"]["vx"] == 3.0
    assert s3["action"]["executed"] is None and s3["action"]["executed_source"].startswith("not_flown")
    # record version 2: monitor_tick, world-frame actions (the body-frame
    # copy is not read), a violation's own priority and grant category
    assert s4["audit_record"] and s4["action"]["planner_output"] == {
        "vx": 5.0, "vy": 0.0, "vz_up": 1.0, "yaw_rate": 0.0}, s4["action"]
    assert s4["action"]["post_repair"]["vx"] == 2.0 and s4["action"]["executed"]["vz_up"] == 0.5
    assert s4["policy_state"]["source"] == "audit record"
    assert s4["violation_type"] == ["alt", "fence", "schedule"], s4["violation_type"]
    lv4 = {v["rule_id"]: v["risk_level"] for v in s4["violations"]}
    assert lv4["kin-caps"] == "P2", lv4     # the record's own priority wins over P1
    assert s4["repairs"] == ["AltitudeFix"]
    # the shape before version 2: tick / raw_action / emitted_action / repairs
    assert s5["audit_record"] and s5["action"]["planner_output"]["vx"] == 6.0
    assert s5["action"]["post_repair"]["vx"] == 1.0 and s5["repairs"] == ["SpeedClamp"]
    assert s5["violation_type"] == ["distance", "envelope"], s5["violation_type"]
    # the episode's worst level is the highest one present (P0 above P1, P2)
    assert ep["labels"]["max_risk_level"] == "P0", ep["labels"]
    # rows 1-3 carry raw actions their records match: the audit is this flight's
    assert ep["audit_join"]["joined"] == 5 and ep["audit_join"]["tick_offset"] == 0
    # risk level from the policy; the unresolved rule is P0, and says so
    assert s1["risk_level"] == "P0" and not s1["risk_level_defaulted"]
    lv = {v["rule_id"]: v["risk_level"] for v in s5["violations"]}
    assert lv == {"kin-caps": "P1", "rule-nobody-wrote": "P0"}, lv
    assert s5["risk_level"] == "P0" and s5["risk_level_defaulted"]


def test_a_shield_off_rail_tick_executes_the_raw_action():
    run = _rail_run(_tmp("wp4_off_"))
    (run / "metrics.json").write_text(json.dumps({"shield": "off"}), encoding="utf-8")
    rows = [json.loads(x) for x in (run / "flight_log.jsonl").read_text().splitlines()]
    rows[1]["setpoint"] = "pass"
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    steps, _ = _export(run)
    assert steps[1]["action"]["executed"]["vx"] == 4.0, steps[1]["action"]
    assert steps[1]["action"]["executed_source"].startswith("shield_off")


def test_a_replay_bundle_is_verified_first_and_a_changed_member_refused():
    from guardrail.replay import write_replay
    root = _tmp("wp4_replay_")
    run = _rail_run(root)
    for f in ("audit.jsonl",):
        (run / f).unlink()               # rows 4-5 then carry no action: drop them
    rows = [json.loads(x) for x in (run / "flight_log.jsonl").read_text().splitlines()][:3]
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    pol = load_policy(POLICY)
    out = write_replay(run, pol, root / "r.replay.tar.gz", signer=None)
    steps, ep = _export(out)
    assert ep["kind"] == "replay" and len(steps) == 3
    assert ep["signature"]["status"] == "unsigned", ep["signature"]
    # change the flight log inside the archive without updating the index
    bad = root / "bad.replay.tar.gz"
    with tarfile.open(out, "r:gz") as src, tarfile.open(bad, "w:gz") as dst:
        for m in src.getmembers():
            data = src.extractfile(m).read()
            if m.name.endswith("flight_log.jsonl"):
                data = data.replace(b'"vx": 4.0', b'"vx": 9.0', 1)
            info = tarfile.TarInfo(m.name)
            info.size = len(data)
            dst.addfile(info, io.BytesIO(data))
    try:
        X.load_episode(bad)
    except X.ExportError as e:
        assert "digest" in str(e), e
    else:
        raise AssertionError("a bundle whose log was changed was exported")
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        assert X.main([str(bad), "--out", str(root / "x"), "--format", "jsonl"]) == 2


def test_jsonl_is_strict_and_parquet_is_written_or_skipped_never_faked():
    root = _tmp("wp4_strict_")
    run = _rail_run(root)
    rows = [json.loads(x) for x in (run / "flight_log.jsonl").read_text().splitlines()]
    rows[0]["raw"]["vx"] = float("nan")
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    out = root / "out"
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        assert X.main([str(run), "--out", str(out)]) == 0
    text = (out / "steps.jsonl").read_text(encoding="utf-8")
    first = json.loads(text.splitlines()[0], parse_constant=lambda c: (_ for _ in ()).throw(
        AssertionError(f"bare {c} token")))
    assert first["action"]["planner_output"]["vx"] == "NaN"
    try:
        import pyarrow                                           # noqa: F401
        have = True
    except ImportError:
        have = False
    if have:
        assert (out / "steps.parquet").is_file() and (out / "episodes.parquet").is_file()
    else:
        assert "SKIP parquet" in buf.getvalue() and not (out / "steps.parquet").exists()
    # with pyarrow blocked the tool must say SKIP, whatever is installed
    saved = sys.modules.get("pyarrow")
    sys.modules["pyarrow"] = None
    try:
        with contextlib.redirect_stdout(io.StringIO()) as buf2:
            X.main([str(run), "--out", str(root / "out2")])
    finally:
        if saved is None:
            sys.modules.pop("pyarrow", None)
        else:
            sys.modules["pyarrow"] = saved
    assert "SKIP parquet" in buf2.getvalue()
    assert not (root / "out2" / "steps.parquet").exists()


def _with_bag(run: Path) -> None:
    bag = run / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  storage_identifier: mcap\n"
        "  starting_time: {nanoseconds_since_epoch: 1000000000}\n"
        "  duration: {nanoseconds: 2000000000}\n"
        "  relative_file_paths: [bag_0.mcap]\n"
        "  topics_with_message_count:\n"
        "    - topic_metadata: {name: /camera/image}\n      message_count: 3\n",
        encoding="utf-8")
    (bag / "bag_0.mcap").write_bytes(b"\x89MCAP0\r\n")
    (run / "events.jsonl").write_text(json.dumps(
        {"kind": "episode_start", "wall_ns": 1_000_000_000}) + "\n", encoding="utf-8")


def test_bag_observations_use_the_mcap_reader_when_present_and_skip_without():
    run = _rail_run(_tmp("wp4_bag_"))
    _with_bag(run)
    # a stand-in for the `mcap` package: three image messages at 1.10, 1.21, 1.45 s
    times = [1_100_000_000, 1_210_000_000, 1_450_000_000]
    fake = types.ModuleType("mcap")
    reader = types.ModuleType("mcap.reader")

    class _R:
        def __init__(self, fh):
            assert fh.read(5) == b"\x89MCAP"

        def iter_messages(self, topics=None):
            for t in times:
                yield (None, types.SimpleNamespace(topic="/camera/image"),
                       types.SimpleNamespace(log_time=t))
    reader.make_reader = _R
    fake.reader = reader
    saved = {k: sys.modules.get(k) for k in ("mcap", "mcap.reader")}
    sys.modules.update({"mcap": fake, "mcap.reader": reader})
    try:
        steps, ep = _export(run)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    assert ep["bag"]["topics"] == {"/camera/image": 3}
    b = steps[1]["observation"]["bag"]                # t = 0.2 s -> 1.2 s
    assert b["t_ns"] == 1_200_000_000 and b["nearest_message_ns"] == {"/camera/image": 1_210_000_000}
    # without the reader: the bag and the time, no message, and a SKIP line
    sys.modules["mcap"] = None
    try:
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            X.main([str(run), "--out", str(run.parent / "o"), "--format", "jsonl"])
        steps, _ = _export(run)
    finally:
        sys.modules.pop("mcap", None)
    assert "nearest_message_ns" not in steps[1]["observation"]["bag"]
    assert "SKIP nearest bag message" in buf.getvalue()


def _tw_policy():
    """A zone in force from 07:30 at priority P1; at 07:29:58 a Monday the
    forecast enters it after the window opens: the grant's time-window
    category (the shape tests/test_shieldcore_audit.py checks the writer
    with)."""
    from guardrail.models import Policy
    return Policy.model_validate({"policy_id": "tw", "constraints": [
        {"id": "school", "type": "polygon_fence", "margin_m": 1.0, "priority": "P1",
         "valid_time": {"recurrence": {"start_time": "07:30", "end_time": "17:30"}},
         "vertices": [{"x": 10, "y": -10}, {"x": 30, "y": -10}, {"x": 30, "y": 10},
                      {"x": 10, "y": 10}]}]})


def _writer_run(root: Path, name: str, vx_list, *, policy=None, clock=None) -> tuple[Path, list]:
    """A run folder as the ROS 2 rail writes one: the audit by
    guardrail/audit.py's AuditLogger (record version 2, monitor_tick from 0),
    the flight log's `tick` from 1, its violations without priority or
    grant category. No policy file: the record's priority is the only one."""
    from datetime import datetime
    from guardrail.audit import AuditLogger
    from guardrail.models import Action4D, State
    from guardrail.shield import Shield
    pol = policy or _tw_policy()
    edge = clock or datetime(2026, 10, 12, 7, 29, 58)
    sh = Shield(pol, now=lambda: edge, escalation=False)
    run = root / name
    run.mkdir(parents=True)
    al = AuditLogger(run / "audit.jsonl", pol)
    rows = []
    for i, vx in enumerate(vx_list):
        d = sh.filter(State(x=0.0, y=0.0, up=10.0), Action4D(vx=vx))
        al.log(i, d)                                   # 0-based, as the rail does
        rows.append({"t": round(0.1 * (i + 1), 3), "tick": i + 1, "x": 0.0, "y": 0.0,
                     "up": 10.0, "raw": d.raw.model_dump(), "emitted": d.emitted.model_dump(),
                     "violations": [{"rule_id": v.rule_id, "category": v.category}
                                    for v in d.violations],
                     "repairs": [{"operator": r.operator} for r in d.repairs]})
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    (run / "manifest.json").write_text(json.dumps({"policy_hash": pol.policy_hash}),
                                       encoding="utf-8")
    return run, rows


def test_a_version_2_audit_from_the_writer_is_joined_one_tick_on_and_labels_the_row():
    run, rows = _writer_run(_tmp("wp4_v2_"), "rail", [3.0, 3.1, 3.2, 3.3])
    recs = [json.loads(x) for x in (run / "audit.jsonl").read_text(encoding="utf-8").splitlines()]
    assert recs and all(r["record_version"] == 2 for r in recs), recs[:1]
    assert [r["monitor_tick"] for r in recs] == [0, 1, 2, 3]
    assert all(v.get("grant_category") == "time-window" and v.get("priority") == "P1"
               for r in recs for v in r["violations"])
    steps, ep = _export(run)
    for s, r in zip(steps, rows):
        assert s["audit_record"], s
        # the row's violation lacks both; the joined record lends them
        assert s["violation_type"] == ["schedule"], s["violation_type"]
        assert s["risk_level"] == "P1" and not s["risk_level_defaulted"], s
        assert s["policy_state"]["source"] == "audit record"
    assert ep["audit_join"]["tick_offset"] == 1 and ep["audit_join"]["joined"] == 4, \
        ep["audit_join"]
    # the null: the same rows with no audit have no priority to read, so the
    # unresolved rule is P0 by default and its category is the row's own
    (run / "audit.jsonl").unlink()
    steps, ep = _export(run)
    assert all(s["risk_level"] == "P0" and s["risk_level_defaulted"] for s in steps)
    assert all(s["violation_type"] == ["fence"] for s in steps)


def test_an_appended_audit_joins_only_the_flight_that_flew():
    """A reused run folder appended flights to one audit.jsonl (before
    rotate_stale_episode): the older flight's records, at the same ticks,
    must not label the newer flight."""
    from guardrail.models import Policy
    root = _tmp("wp4_appended_")
    old_pol = Policy.model_validate({"policy_id": "old", "constraints": [
        {"id": "old-zone", "type": "polygon_fence", "vertices": [
            {"x": 5, "y": -5}, {"x": 9, "y": -5}, {"x": 9, "y": 5}, {"x": 5, "y": 5}]}]})
    # the old flight's first command happens to equal the new one's: one
    # record in four matches, which is not enough to join a flight
    old_run, _ = _writer_run(root, "old", [3.0, 6.1, 6.2, 6.3], policy=old_pol)
    new_run, rows = _writer_run(root, "new", [3.0, 3.1, 3.2, 3.3])
    old_lines = (old_run / "audit.jsonl").read_text(encoding="utf-8")
    new_lines = (new_run / "audit.jsonl").read_text(encoding="utf-8")
    assert old_lines and new_lines
    (new_run / "audit.jsonl").write_text(old_lines + new_lines, encoding="utf-8")
    steps, ep = _export(new_run)
    hashes = {s["policy_state"]["policy_hash"] for s in steps}
    assert hashes == {_tw_policy().policy_hash}, hashes
    assert not any(v["rule_id"] == "old-zone" for s in steps for v in s["violations"])
    assert all(s["audit_record"] for s in steps)
    j = ep["audit_join"]
    assert j["flights_in_audit"] == 2 and j["flight"] == 2 and j["tick_offset"] == 1, j
    assert j["records_of_other_flights"] == len(old_lines.splitlines()), j
    # the old flight written AFTER the new one: still the new flight's records
    (new_run / "audit.jsonl").write_text(new_lines + old_lines, encoding="utf-8")
    steps, ep = _export(new_run)
    assert ep["audit_join"]["flight"] == 1, ep["audit_join"]
    assert {s["policy_state"]["policy_hash"] for s in steps} == {_tw_policy().policy_hash}
    # the same flight appended twice (a re-run into the same folder): the
    # later copy, the one beside today's log, is the one joined
    (new_run / "audit.jsonl").write_text(new_lines + new_lines, encoding="utf-8")
    j = _export(new_run)[1]["audit_join"]
    assert (j["flights_in_audit"], j["flight"], j["joined"]) == (2, 2, 4), j
    # a log no flight in the audit matches (1 record of 4): nothing joined
    (new_run / "audit.jsonl").write_text(old_lines, encoding="utf-8")
    steps, ep = _export(new_run)
    assert ep["audit_join"]["joined"] == 0 and "nothing joined" in ep["audit_join"]["why"]
    assert "best 1 of 4" in ep["audit_join"]["why"], ep["audit_join"]
    assert not any(s["audit_record"] for s in steps)
    # record version 2's episode_id: records of another episode are never
    # joined, even when their raw actions match
    recs = [dict(json.loads(x), episode_id="ep-a") for x in new_lines.splitlines()]
    (new_run / "audit.jsonl").write_text("".join(json.dumps(r) + "\n" for r in recs),
                                         encoding="utf-8")
    (new_run / "flight_log.jsonl").write_text(
        "".join(json.dumps(dict(r, episode_id="ep-b")) + "\n" for r in rows), encoding="utf-8")
    steps, ep = _export(new_run)
    assert ep["audit_join"]["joined"] == 0 and "episode_id" in ep["audit_join"]["why"], \
        ep["audit_join"]
    (new_run / "flight_log.jsonl").write_text(
        "".join(json.dumps(dict(r, episode_id="ep-a")) + "\n" for r in rows), encoding="utf-8")
    assert _export(new_run)[1]["audit_join"]["joined"] == 4     # the same id: joined


def test_parquet_alone_without_pyarrow_writes_nothing_and_exits_3():
    root = _tmp("wp4_pq_")
    run = _rail_run(root)
    saved = sys.modules.get("pyarrow")
    sys.modules["pyarrow"] = None
    try:
        with contextlib.redirect_stdout(io.StringIO()) as out, \
                contextlib.redirect_stderr(io.StringIO()) as err:
            rc = X.main([str(run), "--out", str(root / "pq"), "--format", "parquet"])
    finally:
        if saved is None:
            sys.modules.pop("pyarrow", None)
        else:
            sys.modules["pyarrow"] = saved
    assert rc == 3, (rc, out.getvalue(), err.getvalue())
    assert not any((root / "pq").iterdir()), list((root / "pq").iterdir())
    assert "JSONL written" not in out.getvalue() and "pyarrow" in err.getvalue()


def test_without_extras_the_generation_timeline_comes_from_the_events():
    """A headless bundle flown before 2026-10-12 has no manifest_extras.json:
    the timeline is read from harness_events.jsonl, where a refused
    hot-apply is not a generation and each accepted one starts at its t."""
    bundle = _fly(OPEN, hot=[(3.0, ZONE)])
    copy = _tmp("wp4_events_") / bundle.name
    shutil.copytree(bundle, copy)
    (copy / "manifest_extras.json").unlink()
    ev = (copy / "harness_events.jsonl").read_text(encoding="utf-8").splitlines()
    refused = {"t": 1.0, "type": "hot_apply", "refused": True, "generation": 1,
               "policy_hash": "sha256:refused-never-flew"}
    (copy / "harness_events.jsonl").write_text(
        "\n".join([ev[0], json.dumps(refused)] + ev[1:]) + "\n", encoding="utf-8")
    steps, ep = _export(copy)
    assert len(ep["generations"]) == 2, ep["generations"]
    by_t = [(s["t"], s["policy_state"]) for s in steps]
    assert all(ps["source"] == "the episode's generation timeline" for _, ps in by_t)
    assert {ps["generation"] for t, ps in by_t if t < 3.0 - 1e-9} == {0}
    assert {ps["generation"] for t, ps in by_t if t >= 3.0 - 1e-9} == {1}
    assert "sha256:refused-never-flew" not in {ps["policy_hash"] for _, ps in by_t}


def test_the_action_columns_follow_who_flew_the_tick():
    raw, em = dict(A0, vx=4.0), dict(A0, vx=0.0)
    # headless: the FSM braked, so the stop flew and there is no post-repair
    a = X._actions({"owner": "shield", "setpoint": "brake", "raw": raw, "emitted": em},
                   None, "on")
    assert a["post_repair"] is None and a["executed"] == em and "braked" in a["post_repair_source"]
    # headless: the autopilot flew (LOITER / RTL / LAND)
    a = X._actions({"owner": "autopilot", "autopilot": "LOITER", "raw": raw, "emitted": em},
                   None, "on")
    assert a["post_repair"] is None and a["executed_source"] == "headless:autopilot:LOITER"
    # rail: setpoint "none" alone means nothing was streamed
    a = X._actions({"raw": raw, "emitted": em, "setpoint": "none"}, None, "on")
    assert a["executed"] is None and a["executed_source"].startswith("not_flown"), a
    # rail: a flown_action logged on a tick the rail did not fly is not executed
    a = X._actions({"raw": raw, "emitted": em, "flown_action": em, "flown": False},
                   None, "on")
    assert a["executed"] is None and a["executed_source"].startswith("not_flown"), a
    # control: setpoint "pass" streamed the emitted action
    a = X._actions({"raw": raw, "emitted": em, "setpoint": "pass"}, None, "on")
    assert a["executed"] == em and a["executed_source"].startswith("setpoint:pass")


# --------------------------------------------------------------------------- #
# rerun_episode
# --------------------------------------------------------------------------- #

def _rerun(bundle: Path, *extra) -> tuple[int, dict]:
    with contextlib.redirect_stdout(io.StringIO()) as buf, \
            contextlib.redirect_stderr(io.StringIO()):
        rc = RR.main([str(bundle), "--json", *extra])
    out = buf.getvalue()
    return rc, (json.loads(out) if out.strip().startswith("{") else {})


def test_a_rerun_flies_the_same_log_and_another_seed_differs():
    bundle = _fly(JITTER, seed=7)
    rc, rep = _rerun(bundle)
    assert rc == 0 and rep["match"] is True, rep
    assert rep["compared"].startswith("flight_log.jsonl")
    assert rep["stored_log_sha256"] == rep["rerun_log_sha256"]
    # the null: another seed must differ, and the run did draw from the seed
    rc, rep = _rerun(bundle, "--seed", "8")
    assert rc == 1 and rep["match"] is False and rep["seed_consumed"] is True, rep
    assert rep["seed_flown"] == 8 and rep["seed_recorded"] == 7


def test_a_rerun_rebuilds_from_the_library_when_the_extras_are_absent():
    # a zone across the lane ahead of the aircraft (it starts at x -20, y 15,
    # eastbound at 4 m/s), spawned at 1 s: the flight cannot ignore it
    bundle = _fly(scenario_id="nfz-head-on", seed=1001, hot=[(1.0, {
        "type": "dynamic_nfz", "id": "nfz-early",
        "vertices": [{"x": -12, "y": 5}, {"x": -8, "y": 5}, {"x": -8, "y": 25},
                     {"x": -12, "y": 25}]})])
    copy = _tmp("wp4_lib_") / bundle.name
    shutil.copytree(bundle, copy)
    (copy / "manifest_extras.json").unlink(missing_ok=True)
    rc, rep = _rerun(copy)
    assert rc == 0 and rep["match"] is True, rep
    assert rep["rebuilt_from"].startswith("library") and "1 queued event" in rep["rebuilt_from"]
    # without the queued event the same rebuild is another flight
    doc = json.loads((copy / "kpi.json").read_text(encoding="utf-8"))
    doc["harness"]["events_injected"] = []
    (copy / "kpi.json").write_text(json.dumps(doc), encoding="utf-8")
    rc, rep = _rerun(copy)
    assert rc == 1, rep


def test_a_rerun_uses_the_self_contained_extras_when_the_bundle_has_them():
    S = RR.sweep()
    if not hasattr(S, "episode_extras"):
        print("      SKIP: this sweep_scenarios.py writes no self-contained extras")
        return SKIP
    bundle = _fly(JITTER, seed=11)
    ex = json.loads((bundle / "manifest_extras.json").read_text(encoding="utf-8"))
    assert ex.get("extras_schema") == S.EXTRAS_SCHEMA, sorted(ex)
    rc, rep = _rerun(bundle)
    assert rc == 0 and rep["rebuilt_from"] == "manifest_extras", rep
    rc, rep = _rerun(bundle, "--seed", "12")
    assert rc == 1, rep


def test_a_tampered_log_is_caught_at_the_line_it_was_changed():
    bundle = _fly(OPEN, seed=0)
    copy = _tmp("wp4_tamper_") / bundle.name
    shutil.copytree(bundle, copy)
    lines = (copy / "flight_log.jsonl").read_text(encoding="utf-8").splitlines()
    row = json.loads(lines[42])
    row["x"] += 0.001
    lines[42] = json.dumps(row)
    (copy / "flight_log.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    rc, rep = _rerun(copy)
    assert rc == 1 and rep["first_differing_tick"] == 42, rep


def test_a_bundle_without_its_log_is_compared_on_its_kpi_figures():
    bundle = _fly(scenario_id="nfz-head-on", seed=1001, keep=False)
    assert not (bundle / "flight_log.jsonl").exists()
    rc, rep = _rerun(bundle)
    assert rc == 0 and "weaker" in rep["compared"], rep
    copy = _tmp("wp4_nolog_") / bundle.name
    shutil.copytree(bundle, copy)
    doc = json.loads((copy / "kpi.json").read_text(encoding="utf-8"))
    doc["kpi"]["repair_count"] += 1
    (copy / "kpi.json").write_text(json.dumps(doc), encoding="utf-8")
    rc, rep = _rerun(copy)
    assert rc == 1 and "repair_count" in rep["kpi_differences"], rep


def test_an_episode_that_cannot_be_rebuilt_exits_2():
    d = _tmp("wp4_none_")
    (d / "manifest.json").write_text(json.dumps({"random_seed": 0}), encoding="utf-8")
    (d / "kpi.json").write_text(json.dumps({"scenario_id": "no-such-scenario",
                                            "id_cell": "no-such-scenario"}), encoding="utf-8")
    rc, _ = _rerun(d)
    assert rc == 2


def test_a_policy_that_changed_is_cannot_compare_and_exits_2():
    bundle = _fly(JITTER, seed=5)
    copy = _tmp("wp4_moved_") / bundle.name
    shutil.copytree(bundle, copy)
    man = json.loads((copy / "manifest.json").read_text(encoding="utf-8"))
    man["policy_hash"] = "sha256:" + "0" * 64
    (copy / "manifest.json").write_text(json.dumps(man), encoding="utf-8")
    rc, rep = _rerun(copy)
    assert rc == 2 and rep["verdict"] == "cannot_compare", (rc, rep)
    assert "policy changed" in rep["why"], rep
    assert _rerun(bundle)[0] == 0                       # the untouched bundle


def test_a_policy_file_that_is_gone_is_cannot_rebuild_not_differs():
    S = RR.sweep()
    if not hasattr(S, "episode_extras"):
        print("      SKIP: this sweep_scenarios.py writes no self-contained extras")
        return SKIP
    bundle = _fly(JITTER, seed=5)
    copy = _tmp("wp4_gone_") / bundle.name
    shutil.copytree(bundle, copy)
    ex = json.loads((copy / "manifest_extras.json").read_text(encoding="utf-8"))
    ex["policy"].update(source="bundles/does_not_exist.tar.gz", yaml_text=None,
                        bundle_b64=None)
    (copy / "manifest_extras.json").write_text(json.dumps(ex), encoding="utf-8")
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()) as err:
        rc = RR.main([str(copy), "--json"])
    assert rc == 2 and "cannot rebuild" in err.getvalue(), (rc, err.getvalue()[-400:])
    assert "Traceback" not in err.getvalue()


def test_a_bundle_without_its_log_is_compared_on_its_final_position_too():
    bundle = _fly(scenario_id="nfz-head-on", seed=1001, keep=False)
    copy = _tmp("wp4_final_") / bundle.name
    shutil.copytree(bundle, copy)
    lines = (copy / "harness_events.jsonl").read_text(encoding="utf-8").splitlines()
    out = []
    for line in lines:
        e = json.loads(line)
        if e.get("type") == "episode_end":
            e["final"] = dict(e["final"], x=e["final"]["x"] + 1.0)
        out.append(json.dumps(e))
    (copy / "harness_events.jsonl").write_text("\n".join(out) + "\n", encoding="utf-8")
    rc, rep = _rerun(copy)
    assert rc == 1 and list(rep["kpi_differences"]) == ["final"], rep


def test_the_tools_own_failure_exits_3_not_differs():
    """Exit 1 means "the re-run flew differently"; a defect of the tool must
    not read as that (nor as 2, "cannot rebuild")."""
    real = RR.compare

    def boom(*_a, **_k):
        raise RuntimeError("a defect, not a verdict")
    RR.compare = boom
    try:
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()) as err:
            rc = RR.main([str(_tmp("wp4_boom_")), "--json"])
    finally:
        RR.compare = real
    assert rc == 3 and "a defect, not a verdict" in err.getvalue(), (rc, err.getvalue())


def test_a_scenario_that_draws_nothing_from_the_seed_says_so():
    bundle = _fly(OPEN, seed=3)
    rc, rep = _rerun(bundle, "--seed", "4")
    # nothing in OPEN draws from the seed: another seed flies the same, and
    # the report says the seed was not consumed (so "identical" proves less)
    assert rep["seed_consumed"] is False and rc == 0 and rep["match"] is True, rep
    rc, rep = _rerun(_fly(JITTER, seed=3))
    assert rep["seed_consumed"] is True, rep


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
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    for d in _TMP:
        shutil.rmtree(d, ignore_errors=True)
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
