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
  * A tick's second record (decision_kind "stop_advised", the ArduPilot
    rail since 2026-10-20) stays in its tick's flight: the filter() record
    is joined, the stop's rides on the step as stop_advice (before, every
    such tick split the audit into another flight and most rows lost
    their record).
  * A replay bundle is verified before anything is exported, and a member
    changed after packing is refused. A flight log with no rows is neither
    packaged nor verified as a perfect run (the synthetic-episode test of
    tests/test_replay.py, run here because that file is not on CI).
  * JSONL is strict (a NaN is a string), Parquet is written or SKIPPED,
    never faked, and `--format parquet` alone without pyarrow writes nothing
    and exits 3; the nearest bag message is read through `mcap` when it is
    installed and SKIPPED when it is not.
  * (2026-11-02, WP4-13) With the run's rosbag2, `executed` is the setpoint
    MAVROS was sent on /mavros/setpoint_raw/local in each tick's window,
    back in the world frame, the flight log's verdict beside it: the bag's
    where the two differ or nothing reached MAVROS. The join is placed on
    the mission_start event's clock and refused when the bag does not line
    up with the log (another clock, another flight). Without a bag, without
    `rosbags`, with the join refused or headless, `executed` is the flight
    log's, labelled a fallback on every step. A replay bundle's bag is read
    from the verified archive. The bag's reading is a stand-in here
    (rosbags is not in vla-real); the round trip through a real rosbag2
    SKIPs without rosbags.
  * Audit record version 4: the world-frame actions a step takes from the
    record are raw_action_world / emitted_action_world, never the grant's
    body-frame raw_action; a step's violation carries both categories.
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

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

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
ZONE = {"type": "dynamic_nfz", **HARD_P0, "id": "nfz-hot",
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


def test_a_flight_log_with_no_rows_is_neither_packaged_nor_verified_as_perfect():
    """A rail that aborts before its first tick still writes an empty
    flight_log.jsonl beside kpi.json; write_replay packaged it and
    verify_replay re-derived compute([])'s 0.0 escape rate as a perfect run
    (fixed 2026-10-19). The test that shows it lives in tests/test_replay.py,
    which is off CI because most of its tests need demo/out/ fixtures; this
    one needs none (its episode is synthetic), so it is run here, where CI
    sees it, from its one copy rather than a second that could drift."""
    import test_replay as RP
    assert RP.test_an_episode_whose_log_holds_no_rows_is_refused_not_a_perfect_run() is None


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


# --------------------------------------------------------------------------- #
# executed = what MAVROS was sent, from the rosbag2 (WP4-13, 2026-11-02)
# --------------------------------------------------------------------------- #

_E_NS = 1_761_955_200_000_000_000           # the episode starts
_TAKEOFF_S = 20.0                           # the mission starts 20 s later
_M_NS = _E_NS + int(_TAKEOFF_S * 1e9)
_HEADING = 30.0


def _iso(ns: int) -> str:
    from datetime import datetime, timezone
    return datetime.fromtimestamp(ns / 1e9, tz=timezone.utc).isoformat(timespec="milliseconds")


def _f32(v: float) -> float:
    import struct
    return struct.unpack("f", struct.pack("f", float(v)))[0]


def _ros2_rail_run(root: Path, *, mission_start: bool = True) -> tuple[Path, list, list, list]:
    """A run folder as the ROS 2 rail writes one, with what its bag holds: six
    ticks at a 30 deg heading, t counted from the mission start (20 s after
    the episode start). The adapter's own conversion (body -> NED -> the ENU
    message MAVROS reads, through a float32 /shield/setpoint) gives each
    setpoint. Tick 3: MAVROS was sent something other than the log's verdict.
    Tick 4: the FSM braked (zero sent). Tick 5: the log says "pass" but
    nothing reached MAVROS (the adapter dropped it). Tick 6: nothing streamed.
    One setpoint before the mission (the take-off) lands on no tick."""
    sys.path.insert(0, str(ROOT / "sitl"))
    import mavlink_adapter_node as A
    from guardrail.frames import to_body
    from guardrail.models import Action4D
    run = root / "ros2"
    (run / "bag").mkdir(parents=True)
    (run / "bag" / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  storage_identifier: mcap\n"
        f"  starting_time: {{nanoseconds_since_epoch: {_E_NS - 5_000_000_000}}}\n"
        "  duration: {nanoseconds: 40000000000}\n"
        "  relative_file_paths: [bag_0.mcap]\n"
        "  topics_with_message_count:\n"
        f"    - topic_metadata: {{name: {X.SETPOINT_TOPIC}}}\n      message_count: 5\n"
        f"    - topic_metadata: {{name: {X.POSE_TOPIC}}}\n      message_count: 6\n",
        encoding="utf-8")
    events = [{"kind": "episode_start", "t": None, "wall": _iso(_E_NS), "wall_ns": _E_NS,
               "episode_id": _E_NS}]
    if mission_start:
        events.append({"kind": "mission_start", "t": 0.0, "wall": _iso(_M_NS)})
    (run / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in events),
                                      encoding="utf-8")
    cmd = {1: (3.0, 1.0), 2: (3.0, 0.5), 3: (2.0, 2.0), 4: (2.5, 0.0), 5: (2.5, 0.0),
           6: (0.0, 0.0)}
    sp_of = {1: "pass", 2: "pass", 3: "pass", 4: "brake", 5: "pass", 6: "none"}

    def message(a: Action4D) -> dict:
        body = [_f32(v) for v in to_body(a, _HEADING)]
        f = A.setpoint_from_body(body, _HEADING)["fields"]
        return {"coordinate_frame": f["coordinate_frame"], "type_mask": f["type_mask"],
                "velocity": [float(v) for v in f["velocity"]],
                "yaw_rate": _f32(f["yaw_rate"]), "stamp_ns": 0}
    rows, sent, poses = [], [(_M_NS - 5_000_000_000, message(Action4D(vz_up=1.0)))], []
    for tick in range(1, 7):
        t = round(0.1 * tick, 3)
        em = Action4D(vx=cmd[tick][0], vy=cmd[tick][1], vz_up=0.2, yaw_rate=0.1)
        sp = sp_of[tick]
        rows.append({"t": t, "tick": tick, "x": float(tick), "y": 0.0, "up": 15.0,
                     "yaw_deg": _HEADING, "raw": dict(em.model_dump(), vx=em.vx + 1.0),
                     "emitted": em.model_dump(), "setpoint": sp, "flown": sp != "none",
                     "violations": [], "repairs": []})
        tick_ns = _M_NS + int(t * 1e9)
        poses.append((tick_ns - 10_000_000, {"x": float(tick), "y": 0.0, "up": 15.0,
                                             "yaw_deg": _HEADING}))
        flown = {"pass": em, "brake": Action4D(), "none": None}[sp]
        if tick == 3:
            flown = Action4D(vx=1.0)
        if flown is not None and tick != 5:
            sent.append((tick_ns + 4_000_000, message(flown)))
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    return run, rows, sent, poses


def _close(a: dict | None, b: dict, tol: float = 1e-5) -> bool:
    return a is not None and all(abs(a[k] - b[k]) <= tol for k in ("vx", "vy", "vz_up",
                                                                   "yaw_rate"))


def _check_executed_from_bag(steps: list, ep: dict, rows: list) -> None:
    """What every bag test expects of the six-tick ROS 2 run above."""
    from guardrail.models import Action4D
    s = {st["tick"]: st for st in steps}
    assert all(st["action"]["executed_from"] == "bag" for st in steps), \
        [st["action"]["executed_fallback"] for st in steps]
    assert all(st["action"]["executed_fallback"] is None for st in steps)
    for tick in (1, 2):
        a = s[tick]["action"]
        assert _close(a["executed"], rows[tick - 1]["emitted"]), (tick, a["executed"])
        assert a["executed_source"] == f"bag:{X.SETPOINT_TOPIC}"
        assert a["executed_bag"]["lag_ms"] == 4.0 and a["executed_bag"]["messages_in_window"] == 1
    # tick 3: the bag's command, not the log's; the log's beside it
    a = s[3]["action"]
    assert _close(a["executed"], Action4D(vx=1.0).model_dump()), a["executed"]
    assert a["executed_flight_log"] == rows[2]["emitted"], a
    assert _close(s[4]["action"]["executed"], X.ZERO)                     # the brake
    for tick in (5, 6):                                    # nothing reached MAVROS
        a = s[tick]["action"]
        assert a["executed"] is None and "no message" in a["executed_source"], a
    assert s[5]["action"]["executed_flight_log"] == rows[4]["emitted"]     # the log said pass
    e = ep["executed"]
    assert (e["from"], e["comparable"], e["agree"], e["disagree_with_flight_log"]) == \
        ("bag", 4, 3, 1), e
    assert (e["ticks_with_message"], e["ticks_without_message"], e["messages_on_no_tick"]) \
        == (4, 2, 1), e
    assert e["clock"].startswith("mission_start"), e
    ob = s[2]["observation"]["bag"]
    assert ob["t_ns"] == _M_NS + 200_000_000 and ob["clock"].startswith("mission_start"), ob
    assert ob["state"]["x"] == 2.0 and ob["state"]["topic"] == X.POSE_TOPIC, ob
    assert ep["trajectory"]["executed"][2] == s[3]["action"]["executed"]


class _FakeBag:
    """X.read_bag replaced by one returning `data` (or None: no rosbags)."""

    def __init__(self, data):
        self.data, self.seen = data, []

    def __enter__(self):
        self.saved = X.read_bag

        def fake(d):
            self.seen.append(Path(d))
            self.files = sorted(p.name for p in Path(d).iterdir())
            return self.data
        X.read_bag = fake
        return self

    def __exit__(self, *exc):
        X.read_bag = self.saved
        return False


def test_executed_is_what_the_bag_says_mavros_was_sent():
    """WP4-13 (independent review of 29 Oct): `executed` was inferred from the
    flight log. Now, with the run's rosbag2, it is the setpoint MAVROS was
    sent on /mavros/setpoint_raw/local in each tick's window, back in the
    world frame: where the log and the bag differ (tick 3) and where the log
    says "pass" but nothing reached MAVROS (tick 5), the bag's. The bag is
    read here through a stand-in for read_bag (rosbags is not in vla-real);
    the round trip through a real bag is the next test, which needs it."""
    run, rows, sent, poses = _ros2_rail_run(_tmp("wp4_bagexec_"))
    with _FakeBag(X.BagData(reader="fake", times={X.SETPOINT_TOPIC: [t for t, _ in sent]},
                            setpoints=sent, poses=poses)) as fake:
        steps, ep = _export(run)
    assert fake.seen == [run / "bag"]
    _check_executed_from_bag(steps, ep, rows)


def test_without_a_trusted_bag_executed_is_the_flight_log_labelled_a_fallback():
    """The nulls of the test above. Each one leaves `executed` the flight
    log's verdict and says why on every step and in the episode record: no
    bag; a bag but no rosbags (and the run prints SKIP with the install
    line); a bag placed on the episode start's clock, 20 s before the
    mission's, whose setpoints then line up with no tick (the join is
    refused, not half used); a bag with no setpoint topic; a headless
    episode."""
    from guardrail.models import Action4D
    root = _tmp("wp4_bagnull_")
    run, rows, sent, poses = _ros2_rail_run(root)
    data = X.BagData(reader="fake", setpoints=sent, poses=poses)

    def labelled(steps, ep, word):
        assert ep["executed"]["from"] == "flight_log" and word in ep["executed"]["why"], \
            ep["executed"]
        for st, r in zip(steps, rows):
            a = st["action"]
            assert a["executed_from"] == "flight_log" and word in a["executed_fallback"], a
            assert "executed_flight_log" not in a
            want = {"pass": r["emitted"], "brake": X.ZERO, "none": None}[r["setpoint"]]
            assert a["executed"] == want, (r["tick"], a)
    # no rosbags
    with _FakeBag(None):
        steps, ep = _export(run)
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            X.main([str(run), "--out", str(root / "o"), "--format", "jsonl"])
    labelled(steps, ep, "rosbags")
    assert "SKIP executed from the bag" in buf.getvalue() and "pip install" in buf.getvalue()
    # the wrong clock: no mission_start, so the episode start's (20 s early)
    run2, rows2, _, _ = _ros2_rail_run(root / "b", mission_start=False)
    with _FakeBag(data):
        steps, ep = _export(run2)
    labelled(steps, ep, "do not line up")
    # a bag without the setpoint topic
    with _FakeBag(X.BagData(reader="fake", poses=poses)):
        steps, ep = _export(run)
    labelled(steps, ep, "no /mavros/setpoint_raw/local")
    # no bag at all
    shutil.rmtree(run / "bag")
    steps, ep = _export(run)
    labelled(steps, ep, "no rosbag2")
    # a headless episode never reads a bag
    hs, hep = _export(_fly(OPEN))
    assert hep["executed"]["from"] == "flight_log" and "headless" in hep["executed"]["why"]
    assert all(st["action"]["executed_from"] == "flight_log" for st in hs)
    # a setpoint in another frame is not converted (one that ignores the
    # velocity neither)
    f = dict(sent[1][1], coordinate_frame=8)
    assert X.setpoint_world(f)[0] is None and "FRAME_LOCAL_NED" in X.setpoint_world(f)[1]
    f = dict(sent[1][1], type_mask=sent[1][1]["type_mask"] | 8)
    assert X.setpoint_world(f)[0] is None
    assert _close(X.setpoint_world(sent[1][1])[0], rows[0]["emitted"])
    assert not X._agrees(Action4D(vx=3.0).model_dump(), Action4D(vx=2.0).model_dump())


def test_a_replay_bundles_bag_is_read_from_the_verified_archive():
    """A replay bundle carries the bag (guardrail/replay.py); the exporter
    reads it from the verified archive, its files flat in a folder of their
    own, and the setpoints in it give `executed` as in a run folder. The
    episode is tests/test_replay.py's synthetic one (no mission_start: the
    episode start is its clock), its bag a stand-in read by the fake."""
    import test_replay as RP
    from guardrail.replay import write_replay
    root = _tmp("wp4_replaybag_")
    run, policy, rec, rows = RP._episode(root, hot=False)
    RP._bag(run, rec.episode_id - int(20e9))
    sent = [(rec.episode_id + int(round(r["t"] * 1e9)) + 3_000_000,
             {"coordinate_frame": 1, "type_mask": 1479,
              "velocity": [r["emitted"]["vy"], r["emitted"]["vx"], r["emitted"]["vz_up"]],
              "yaw_rate": -r["emitted"]["yaw_rate"]}) for r in rows]
    out = write_replay(run, policy, root / "r.replay.tar.gz", signer=None)
    with _FakeBag(X.BagData(reader="fake", setpoints=sent)) as fake:
        steps, ep = _export(out)
    assert fake.files == ["bag_0.mcap", "metadata.yaml"], fake.files
    assert ep["kind"] == "replay" and ep["executed"]["from"] == "bag", ep["executed"]
    assert ep["executed"]["clock"] == "episode_start wall_ns"
    assert all(st["action"]["executed_from"] == "bag" for st in steps)
    assert all(_close(st["action"]["executed"], r["emitted"]) for st, r in zip(steps, rows))


def test_the_exporters_heading_formula_is_the_adapters():
    """The pose a step's observation takes from the bag is turned into a
    heading by the exporter's copy of the adapter's formula; the two must
    not drift."""
    sys.path.insert(0, str(ROOT / "sitl"))
    import mavlink_adapter_node as A
    for yaw_enu in (-3.1, -1.2, 0.0, 0.4, 1.5707963, 2.9):
        for tilt in (0.0, 0.05):
            q = (tilt, 0.0, math.sin(yaw_enu / 2), math.cos(yaw_enu / 2))
            assert abs(X.heading_deg_from_enu_quaternion(*q)
                       - A.heading_deg_from_enu_quaternion(*q)) < 1e-12, q


def _write_rosbag(bag_dir: Path, sent: list, poses: list) -> None:
    """A real rosbag2 with `rosbags` (>= 0.10): the setpoints as
    mavros_msgs/PositionTarget, the poses as PoseStamped in MAVROS's ENU."""
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    store = get_typestore(getattr(Stores, "ROS2_JAZZY", None) or Stores.LATEST)
    store.register(get_types_from_msg(X.POSITION_TARGET_MSG, X.POSITION_TARGET_TYPE))
    T = store.types
    Time, Header = T["builtin_interfaces/msg/Time"], T["std_msgs/msg/Header"]
    Point, Vector3 = T["geometry_msgs/msg/Point"], T["geometry_msgs/msg/Vector3"]
    Quaternion, Pose = T["geometry_msgs/msg/Quaternion"], T["geometry_msgs/msg/Pose"]
    PoseStamped, PT = T["geometry_msgs/msg/PoseStamped"], T[X.POSITION_TARGET_TYPE]
    writer = None
    for kw in ({"version": 9}, {"version": 8}, {}):
        try:
            writer = Writer(bag_dir, **kw)
            break
        except (TypeError, ValueError):
            continue
    assert writer is not None, "no rosbags Writer signature worked"

    def header(ts):
        return Header(stamp=Time(sec=ts // 10 ** 9, nanosec=ts % 10 ** 9), frame_id="")
    with writer as w:
        def conn(topic, typ):
            try:
                return w.add_connection(topic, typ, typestore=store)
            except TypeError:                    # rosbags < 0.10
                return w.add_connection(topic, typ)
        c_sp = conn(X.SETPOINT_TOPIC, X.POSITION_TARGET_TYPE)
        c_pose = conn(X.POSE_TOPIC, "geometry_msgs/msg/PoseStamped")
        for ts, f in sent:
            msg = PT(header=header(ts), coordinate_frame=f["coordinate_frame"],
                     type_mask=f["type_mask"], position=Point(x=0.0, y=0.0, z=0.0),
                     velocity=Vector3(x=f["velocity"][0], y=f["velocity"][1],
                                      z=f["velocity"][2]),
                     acceleration_or_force=Vector3(x=0.0, y=0.0, z=0.0), yaw=0.0,
                     yaw_rate=f["yaw_rate"])
            w.write(c_sp, ts, store.serialize_cdr(msg, X.POSITION_TARGET_TYPE))
        for ts, p in poses:
            yaw_enu = math.radians(90.0 - p["yaw_deg"])
            msg = PoseStamped(header=header(ts), pose=Pose(
                position=Point(x=p["y"], y=p["x"], z=p["up"]),
                orientation=Quaternion(x=0.0, y=0.0, z=math.sin(yaw_enu / 2),
                                       w=math.cos(yaw_enu / 2))))
            w.write(c_pose, ts, store.serialize_cdr(msg, "geometry_msgs/msg/PoseStamped"))


def test_a_real_rosbag2_is_read_with_rosbags():
    """The same six ticks through a rosbag2 written and read with `rosbags`
    (the reader the exporter uses), in a run folder and in its replay
    bundle. SKIPPED where rosbags is not installed: it is not in vla-real.
    Run it with `pip install "rosbags>=0.10"` in a scratch environment, or
    in WSL (see CHANGELOG 2026-11-02)."""
    if not X.have_rosbags():
        print(f"      SKIP: rosbags is not installed ({X.ROSBAGS_INSTALL_HINT})")
        return SKIP
    root = _tmp("wp4_rosbags_")
    run, rows, sent, poses = _ros2_rail_run(root)
    shutil.rmtree(run / "bag")
    _write_rosbag(run / "bag", sent, poses)
    data = X.read_bag(run / "bag")
    assert len(data.setpoints) == len(sent) and len(data.poses) == len(poses), data
    assert abs(data.poses[0][1]["yaw_deg"] - _HEADING) < 1e-9
    steps, ep = _export(run)
    _check_executed_from_bag(steps, ep, rows)
    assert ep["executed"]["reader"].startswith("rosbags")


def _tw_policy():
    """A zone in force from 07:30 at priority P1; at 07:29:58 a Monday the
    forecast enters it after the window opens: the grant's time-window
    category (the shape tests/test_shieldcore_audit.py checks the writer
    with)."""
    from guardrail.models import Policy
    return Policy.model_validate({"policy_id": "tw", "constraints": [
        {"id": "school", "type": "polygon_fence",
         "scope": "mission", "constraint_type": "hard", "violation_action": "project_fix", "margin_m": 1.0, "priority": "P1",
         "valid_time": {"recurrence": {"start_time": "07:30", "end_time": "17:30"}},
         "vertices": [{"x": 10, "y": -10}, {"x": 30, "y": -10}, {"x": 30, "y": 10},
                      {"x": 10, "y": 10}]}]})


def _writer_run(root: Path, name: str, vx_list, *, policy=None, clock=None,
                yaw_deg: float = 0.0) -> tuple[Path, list]:
    """A run folder as the ROS 2 rail writes one: the audit by
    guardrail/audit.py's AuditLogger (record version 2 when this was written,
    4 now; monitor_tick from 0), the flight log's `tick` from 1, its
    violations without priority or grant category. No policy file: the
    record's priority is the only one."""
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
        d = sh.filter(State(x=0.0, y=0.0, up=10.0, yaw_deg=yaw_deg), Action4D(vx=vx))
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
    # The writer's own version: 3 since 2026-10-20 (fsm_hold_point, which the
    # join does not read), 2 when this test was written.
    assert recs and all(r["record_version"] >= 2 for r in recs), recs[:1]
    assert [r["monitor_tick"] for r in recs] == [0, 1, 2, 3]
    # Record version 4: the grant's category is `category`, ours `rule_category`.
    assert all(v.get("category") == "time-window" and v.get("priority") == "P1"
               and v.get("rule_category") == "geofence"
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


def test_a_version_4_audit_lends_its_world_frame_actions_and_both_categories():
    """Audit record version 4 (2026-11-02): `raw_action` / `emitted_action`
    are the grant's body frame. Heading East (90 deg), a North command is
    vx 0, vy -3.3 in the body frame; the step that takes its action from the
    record must take the WORLD one (raw_action_world), and its violation
    carries the grant's category beside ours. Reading `raw_action` as world,
    the join compared body against world and joined nothing."""
    run, rows = _writer_run(_tmp("wp4_v4_"), "rail", [3.0, 3.1, 3.2, 3.3], yaw_deg=90.0)
    recs = [json.loads(x) for x in (run / "audit.jsonl").read_text(encoding="utf-8").splitlines()]
    assert recs[-1]["record_version"] == 4 and abs(recs[-1]["raw_action"]["vy"] + 3.3) < 1e-9
    # the last row logged only its pose: its actions come from the record
    rows[-1] = {k: rows[-1][k] for k in ("t", "tick", "x", "y", "up")}
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    steps, ep = _export(run)
    assert ep["audit_join"]["joined"] == 4, ep["audit_join"]
    last = steps[-1]
    assert last["action"]["planner_output"] == recs[-1]["raw_action_world"] == {
        "vx": 3.3, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}, last["action"]
    assert last["action"]["post_repair"] == recs[-1]["emitted_action_world"]
    v = last["violations"][0]
    assert (v["category"], v["rule_category"], v["violation_type"]) == \
        ("time-window", "geofence", "schedule"), v
    assert steps[0]["schema"] == "vlaguard.episode_export/2"


def test_an_appended_audit_joins_only_the_flight_that_flew():
    """A reused run folder appended flights to one audit.jsonl (before
    rotate_stale_episode): the older flight's records, at the same ticks,
    must not label the newer flight."""
    from guardrail.models import Policy
    root = _tmp("wp4_appended_")
    old_pol = Policy.model_validate({"policy_id": "old", "constraints": [
        {"id": "old-zone", "type": "polygon_fence", **HARD_P0, "vertices": [
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


def _advised_run(root: Path, name: str, vx_list, advised) -> tuple[Path, list]:
    """A run folder as the ArduPilot rail writes one with the own-velocity
    forecast on (demo/pas_ardupilot/perception_node.py, 2026-10-20): the
    audit's tick is the log's (offset 0); on each tick in `advised` the rail
    flew the Shield's answer to a stop (Shield.advise of a zero action)
    instead of the pilot's action, so audit.jsonl holds the tick's filter()
    record and then, AT THE SAME TICK, the stop's (decision_kind
    "stop_advised"), and the row's flown_action / stop_advice are the stop's."""
    from datetime import datetime
    from guardrail.audit import AuditLogger
    from guardrail.models import Action4D, State
    from guardrail.shield import Shield
    pol = _tw_policy()
    sh = Shield(pol, now=lambda: datetime(2026, 10, 12, 7, 29, 58), escalation=False)
    run = root / name
    run.mkdir(parents=True)
    al = AuditLogger(run / "audit.jsonl", pol)
    rows = []
    for i, vx in enumerate(vx_list, start=1):
        st = State(x=0.0, y=0.0, up=10.0)
        d = sh.filter(st, Action4D(vx=vx))
        al.log(i, d)
        row = {"t": round(0.1 * i, 3), "tick": i, "x": 0.0, "y": 0.0, "up": 10.0,
               "raw": d.raw.model_dump(), "emitted": d.emitted.model_dump(),
               "violations": [{"rule_id": v.rule_id, "category": v.category}
                              for v in d.violations],
               "repairs": [{"operator": r.operator} for r in d.repairs],
               "setpoint": "pass", "flown": True, "stop_advised": i in advised,
               "flown_action": d.emitted.model_dump(), "stop_advice": None}
        if i in advised:
            adv = sh.advise(st, Action4D())
            al.log(i, adv, kind="stop_advised")
            row["flown_action"] = adv.emitted.model_dump()
            row["stop_advice"] = {"emitted": adv.emitted.model_dump()}
        rows.append(row)
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    (run / "manifest.json").write_text(json.dumps({"policy_hash": pol.policy_hash}),
                                       encoding="utf-8")
    return run, rows


def test_a_stop_advised_record_is_its_tick_s_second_decision_not_a_new_flight():
    """THE SECOND RECORD OF A TICK (2026-10-20 re-review). Since the hold that
    stops clear, the ArduPilot rail writes a stop-advised tick's record
    (guardrail/audit.py AuditRecord.decision_kind "stop_advised") right after
    that tick's filter() record, with the SAME tick. audit_blocks started a
    new flight wherever the tick did not advance, so every such tick split
    the audit into another "flight" and join_audit kept the best single
    fragment: on the hold rail run of tests/test_own_velocity_forecast.py
    (81 records, 21 stop-advised) 22 "flights" and 40 of 60 rows joined, on
    the FSM-brake run 2 of 60 - the policy state, priorities and categories
    the steps take from the audit lost without a word but a lower `joined`.
    Now the stop's record stays in its tick's flight; the filter() record is
    the one joined (the stop's raw action is a zero, not the row's, so it is
    never compared against the row either); the stop's record rides on the
    step as `stop_advice`. An appended file still splits at the flight
    boundary, its stop records counted with their own flight."""
    root = _tmp("wp4_advised_")
    advised = {2, 3}
    run, rows = _advised_run(root, "rail", [3.0, 3.1, 3.2, 3.3], advised)
    recs = [json.loads(x) for x in (run / "audit.jsonl").read_text(encoding="utf-8").splitlines()]
    assert [(r["monitor_tick"], r.get("decision_kind")) for r in recs] == [
        (1, None), (2, None), (2, "stop_advised"), (3, None), (3, "stop_advised"),
        (4, None)], [(r["monitor_tick"], r.get("decision_kind")) for r in recs]
    assert len(X.audit_blocks(recs)) == 1, [len(b) for b in X.audit_blocks(recs)]
    steps, ep = _export(run)
    j = ep["audit_join"]
    assert (j["flights_in_audit"], j["flight"], j["tick_offset"], j["joined"]) == (1, 1, 0, 4), j
    assert j["raw_matched"] == j["raw_comparable"] == 4, j
    assert j["stop_advice_records"] == 2 and j["records_of_other_flights"] == 0, j
    by_tick = {r["monitor_tick"]: r for r in recs if r.get("decision_kind")}
    for s, r in zip(steps, rows):
        assert s["audit_record"] and s["policy_state"]["source"] == "audit record", s
        # the filter() record's labels, not the stop's
        assert s["violation_type"] == ["schedule"] and s["risk_level"] == "P1", s
        assert s["action"]["executed"] == r["flown_action"], s["action"]
        if r["tick"] in advised:
            a = s["stop_advice"]
            assert a is not None and a["source"].startswith("audit record"), a
            assert a["emitted"] == by_tick[r["tick"]]["emitted_action_world"] == \
                r["flown_action"], a
            assert "stop" in s["action"]["post_repair_source"], s["action"]
        else:
            assert s["stop_advice"] is None, s["stop_advice"]
    # Appended (a reused run folder): two such flights split at the boundary
    # only, and each flight's stop records count with it.
    other, _ = _advised_run(root, "other", [6.0, 6.1, 6.2, 6.3], advised)
    old_lines = (other / "audit.jsonl").read_text(encoding="utf-8")
    new_lines = (run / "audit.jsonl").read_text(encoding="utf-8")
    (run / "audit.jsonl").write_text(old_lines + new_lines, encoding="utf-8")
    steps, ep = _export(run)
    j = ep["audit_join"]
    assert (j["flights_in_audit"], j["flight"], j["joined"]) == (2, 2, 4), j
    assert j["records_of_other_flights"] == len(old_lines.splitlines()) == 6, j
    assert [s["stop_advice"] is not None for s in steps] == [False, True, True, False]
    # A second stop record at a tick that already has one is not the same
    # tick's second decision: it starts another flight, as before.
    dup = [recs[1], recs[2], dict(recs[2])]
    assert [len(b) for b in X.audit_blocks(dup)] == [2, 1]


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


# --------------------------------------------------------------------------- #
# Parquet (2026-10-19). The first writer had never run here (pyarrow was in
# neither env). With pyarrow 26.0.0 it dropped every key the first record
# lacked, crashed on a float column holding a NaN (JSON safety made it a mixed
# float / "NaN" column) and on a bool / int mix, and wrote JSON text a reader
# could not tell from a plain string. These tests fail on it: parquet_table,
# read_parquet and have_pyarrow did not exist, and its file had no metadata.
# --------------------------------------------------------------------------- #

def _no_pyarrow():
    try:
        import pyarrow                                           # noqa: F401
        import pyarrow.parquet                                   # noqa: F401
    except ImportError:
        print("      SKIP: pyarrow is not installed (optional; pip install .[export])")
        return True
    return False


class _BlockPyarrow:
    def __enter__(self):
        self.saved = {m: sys.modules.get(m) for m in ("pyarrow", "pyarrow.parquet")}
        for m in self.saved:
            sys.modules[m] = None
        return self

    def __exit__(self, *exc):
        for m, v in self.saved.items():
            if v is None:
                sys.modules.pop(m, None)
            else:
                sys.modules[m] = v
        return False


def test_parquet_reads_back_as_the_jsonl_with_one_type_per_column():
    """A headless episode with a hot-apply and a rail run with a NaN in a raw
    action, exported together: every Parquet record equals its JSONL line,
    and the columns are typed (not one string per cell)."""
    if _no_pyarrow():
        return SKIP
    import pyarrow.parquet as pq
    root = _tmp("wp4_pq_rt_")
    run = _rail_run(root)
    rows = [json.loads(x) for x in (run / "flight_log.jsonl").read_text().splitlines()]
    rows[0]["raw"]["vx"] = float("nan")
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    hot = _fly(OPEN, hot=[(3.0, ZONE)])
    out = root / "out"
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        assert X.main([str(hot), str(run), "--out", str(out)]) == 0, buf.getvalue()
    for name in ("steps", "episodes"):
        jsonl = [json.loads(x) for x in
                 (out / f"{name}.jsonl").read_text(encoding="utf-8").splitlines()]
        back = X.read_parquet(out / f"{name}.parquet")
        assert len(back) == len(jsonl) > 0, (name, len(back), len(jsonl))
        assert back == jsonl, next((i, a, b) for i, (a, b) in enumerate(zip(back, jsonl))
                                   if a != b)
    t = pq.read_table(out / "steps.parquet")
    types = {f.name: str(f.type) for f in t.schema}
    assert types["step"] == "int64" and types["t"] == "double", types
    assert types["outcome"] == "string" and types["risk_level_defaulted"] == "bool", types
    meta = t.schema.metadata
    assert meta[b"vlaguard.schema"].decode() == X.SCHEMA
    jc = set(json.loads(meta[b"vlaguard.json_columns"]))
    assert {"observation", "policy_state", "action", "violations"} <= jc, jc
    assert "step" not in jc and "outcome" not in jc, jc
    # The rail run's first tick carries the NaN, as the JSONL's string.
    steps = X.read_parquet(out / "steps.parquet")
    rail_first = next(s for s in steps if s["episode"] != steps[0]["episode"])
    assert rail_first["step"] == 0, rail_first["step"]
    assert rail_first["action"]["planner_output"]["vx"] == "NaN", rail_first["action"]


def test_the_parquet_schema_keeps_every_key_and_every_mixed_column():
    """The records that broke the first writer: a key the first record lacks,
    a float column with NaN and infinity, a bool / int mix, a column that is a
    dict in one row and a string in another, an int beyond int64, a column
    with no value. Each reads back as the JSONL would say it."""
    if _no_pyarrow():
        return SKIP
    import pyarrow.parquet as pq
    recs = [{"a": 1, "f": 0.5, "m": True, "d": {"x": 1}, "big": 1, "none": None},
            {"a": 2, "b": 3, "f": float("nan"), "m": 1, "d": "plain", "big": 2 ** 70,
             "none": None},
            {"a": 3, "f": float("inf"), "m": None, "d": None, "big": 3, "none": None}]
    p = _tmp("wp4_pq_mixed_") / "x.parquet"
    assert X.write_parquet(p, recs) is True
    back = X.read_parquet(p)
    want = [X.json_safe({k: r.get(k) for k in ("a", "f", "m", "d", "big", "none", "b")})
            for r in recs]
    assert [dict(sorted(r.items())) for r in back] == [dict(sorted(r.items())) for r in want]
    assert back[1]["b"] == 3 and back[0]["b"] is None, "the key the first record lacked"
    assert back[1]["f"] == "NaN" and back[2]["f"] == "Infinity", back
    assert back[1]["d"] == "plain" and back[0]["d"] == {"x": 1}, back
    t = pq.read_table(p)
    types = {f.name: str(f.type) for f in t.schema}
    assert types["f"] == "double", "NaN and infinity stay in a float column"
    assert types["a"] == "int64" and types["b"] == "int64", types
    jc = set(json.loads(t.schema.metadata[b"vlaguard.json_columns"]))
    assert jc == {"m", "d", "big"}, jc
    assert list(t.column_names) == ["a", "f", "m", "d", "big", "none", "b"], t.column_names


def test_a_parquet_file_that_does_not_read_back_is_not_published():
    """The writer compares its file with the records before it reports
    success; a mismatch deletes both files and exits 2, the JSONL kept."""
    if _no_pyarrow():
        return SKIP
    root = _tmp("wp4_pq_bad_")
    run = _rail_run(root)
    saved = X.read_parquet

    def lossy(path):
        recs = saved(path)
        return recs[:-1] if len(recs) > 1 else [{**recs[0], "step": -1}]
    X.read_parquet = lossy
    try:
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()) as err:
            rc = X.main([str(run), "--out", str(root / "out")])
    finally:
        X.read_parquet = saved
    assert rc == 2, (rc, err.getvalue())
    assert "parquet NOT written" in err.getvalue(), err.getvalue()
    assert not list((root / "out").glob("*.parquet")), list((root / "out").iterdir())
    assert (root / "out" / "steps.jsonl").is_file()
    # A Parquet file without the export's metadata (the first writer's) is
    # refused by the reader rather than decoded by guesswork.
    import pyarrow as pa
    import pyarrow.parquet as pq
    plain = root / "plain.parquet"
    pq.write_table(pa.Table.from_pylist([{"action": json.dumps({"vx": 1.0})}]), plain)
    try:
        X.read_parquet(plain)
    except X.ExportError as e:
        assert "json_columns" in str(e), e
    else:
        raise AssertionError("a Parquet file with no column metadata was decoded")


def test_a_parquet_file_missing_a_column_is_not_published():
    """Review of the T1d branch (P1): the read-back check built its reference
    from the columns the FILE had (`back[0]`), so a Parquet file that lost a
    whole column - the first writer's failure mode, a key dropped from every
    row - compared equal on the columns left and was kept. Shown failing at
    044f0fc: with parquet_table dropping column `b`, write_parquet returned
    True for [{a: 1, b: 10}, {a: 2, b: 20}] and read_parquet gave
    [{a: 1}, {a: 2}]; with it dropping `violation_type`, main() exited 0 with
    both Parquet files published. The reference is now every key of every
    record, as the JSONL holds it."""
    if _no_pyarrow():
        return SKIP
    real = X.parquet_table

    def dropping(name):
        def table(records):
            t = real(records)
            if name not in t.column_names:
                return t
            return t.drop_columns([name]).replace_schema_metadata(t.schema.metadata)
        return table
    p = _tmp("wp4_pq_dropcol_") / "x.parquet"
    X.parquet_table = dropping("b")
    try:
        X.write_parquet(p, [{"a": 1, "b": 10}, {"a": 2, "b": 20}])
    except X.ExportError as e:
        assert "missing column" in str(e) and "'b'" in str(e), e
    else:
        raise AssertionError(f"a file without column b was kept: {X.read_parquet(p)}")
    finally:
        X.parquet_table = real
    assert not p.exists(), "the file that lost a column is still on disk"
    # The first writer's own bug: columns from the first record only.
    X.parquet_table = lambda records: real([{k: r.get(k) for k in records[0]}
                                             for r in records])
    try:
        X.write_parquet(p, [{"a": 1}, {"a": 2, "b": 3}])
    except X.ExportError as e:
        assert "'b'" in str(e), e
    else:
        raise AssertionError("columns taken from the first record only were not caught")
    finally:
        X.parquet_table = real
    assert not p.exists()
    # Through the CLI: exit 2, no Parquet file at all, the JSONL kept.
    root = _tmp("wp4_pq_dropcol_cli_")
    run = _rail_run(root)
    out = root / "out"
    X.parquet_table = dropping("violation_type")
    try:
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()) as err:
            rc = X.main([str(run), "--out", str(out)])
    finally:
        X.parquet_table = real
    assert rc == 2, (rc, err.getvalue())
    assert "parquet NOT written" in err.getvalue() and "violation_type" in err.getvalue(), \
        err.getvalue()
    assert not list(out.glob("*.parquet")), list(out.iterdir())
    assert (out / "steps.jsonl").is_file() and (out / "episodes.jsonl").is_file()


def test_any_parquet_failure_leaves_no_parquet_file_and_exits_2():
    """Review of the T1d branch (P2): only ExportError was caught around the
    two writes, so a pyarrow or encoding error on episodes.parquet raised a
    traceback after steps.parquet was written, and left it beside a stale
    episodes.parquet from an earlier export. Shown failing at 044f0fc: the
    ArrowInvalid below escaped main(). Now any failure removes both files,
    says its type, and exits 2; the JSONL stays."""
    if _no_pyarrow():
        return SKIP
    import pyarrow as pa
    root = _tmp("wp4_pq_anyerr_")
    run = _rail_run(root)
    out = root / "out"
    out.mkdir()
    for name in ("steps.parquet", "episodes.parquet"):
        (out / name).write_bytes(b"an earlier export")
    real = X.parquet_table

    def fails_on_episodes(records):
        if records and "n_steps" in records[0]:
            raise pa.ArrowInvalid("a column pyarrow cannot type")
        return real(records)
    X.parquet_table = fails_on_episodes
    try:
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()) as err:
            rc = X.main([str(run), "--out", str(out)])
    finally:
        X.parquet_table = real
    assert rc == 2, (rc, err.getvalue())
    assert "parquet NOT written: ArrowInvalid: a column pyarrow cannot type" in err.getvalue(), \
        err.getvalue()
    assert not list(out.glob("*.parquet")), list(out.iterdir())
    assert (out / "steps.jsonl").is_file() and (out / "episodes.jsonl").is_file()


def test_parquet_reads_back_what_the_jsonl_encodes_not_what_json_safe_returns():
    """Review of the T1d branch (P2): the reference was json_safe(record), not
    the JSONL's encoding, so a dict with int keys (the JSONL writes "1") was
    refused although the JSONL holds it fine; and a string with a lone
    surrogate (the JSONL escapes it as \\ud800) crashed pyarrow's UTF-8
    string column with UnicodeEncodeError. Both now read back as the JSONL
    line does: the int-keyed dict as JSON text, the surrogate column as JSON
    text named in the metadata."""
    if _no_pyarrow():
        return SKIP
    recs = [{"step": 0, "counts": {1: "one", 2: "two"}, "note": "plain"},
            {"step": 1, "counts": {3: "three"}, "note": "half \ud800 pair"}]
    p = _tmp("wp4_pq_jsonenc_") / "x.parquet"
    assert X.write_parquet(p, recs) is True
    lines = [json.loads(json.dumps(X.json_safe(r), allow_nan=False)) for r in recs]
    assert X.read_parquet(p) == lines, (X.read_parquet(p), lines)
    assert lines[0]["counts"] == {"1": "one", "2": "two"}
    import pyarrow.parquet as pq
    jc = set(json.loads(pq.read_table(p).schema.metadata[b"vlaguard.json_columns"]))
    assert jc == {"counts", "note"}, jc


def test_without_pyarrow_the_hint_is_printed_and_a_stale_parquet_removed():
    """--format both without pyarrow: JSONL, a SKIP line saying how to install
    it, and no Parquet file from an earlier export left beside the new JSONL
    to disagree with it."""
    root = _tmp("wp4_pq_stale_")
    run = _rail_run(root)
    out = root / "out"
    out.mkdir()
    (out / "steps.parquet").write_bytes(b"an earlier export")
    with _BlockPyarrow():
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            assert X.main([str(run), "--out", str(out)]) == 0
    text = buf.getvalue()
    assert "SKIP parquet" in text and "pip install pyarrow" in text, text
    assert "removed the earlier export's steps.parquet" in text, text
    assert not (out / "steps.parquet").exists()
    assert (out / "steps.jsonl").is_file()


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
    # The bundle's shield_audit.jsonl (written since 2026-10-30, read by the
    # exporter since 2026-11-02) answers the ticks the Shield logged; the rest
    # take the timeline. Without the audit, every tick takes the timeline.
    for with_audit in (True, False):
        if not with_audit:
            (copy / "shield_audit.jsonl").unlink()
        steps, ep = _export(copy)
        assert len(ep["generations"]) == 2, ep["generations"]
        by_t = [(s["t"], s["policy_state"], s["audit_record"]) for s in steps]
        assert all(ps["source"] == ("audit record" if rec else
                                    "the episode's generation timeline")
                   for _, ps, rec in by_t)
        assert any(rec for *_, rec in by_t) == with_audit
        assert any(not rec for *_, rec in by_t)
        assert {ps["generation"] for t, ps, _ in by_t if t < 3.0 - 1e-9} == {0}
        assert {ps["generation"] for t, ps, _ in by_t if t >= 3.0 - 1e-9} == {1}
        assert "sha256:refused-never-flew" not in {ps["policy_hash"] for _, ps, _ in by_t}


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
        "type": "dynamic_nfz", **HARD_P0, "id": "nfz-early",
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
