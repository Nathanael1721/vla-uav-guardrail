"""tools/replay_at_revision.py: an episode replayed at the revision it flew (WP4-11).

    python tests/test_replay_at_revision.py

What is pinned here:

  * a revision a checkout cannot reproduce is refused with its reason, never
    replayed on other code: none recorded, 'unversioned', '-dirty',
    '-unknown', not a commit id, a SHA that is not a commit of this
    repository;
  * episodes flown at HEAD (made here with this checkout's code, so HEAD
    holds the code that made them) replay at HEAD in a temporary detached
    worktree, through a driver whose modules come from that worktree:
      - a rail run stored as the ROS 2 rail stores it (flight_log.jsonl,
        trajectory.json, policy_g<N>.json, a hot-applied moving zone,
        metrics.json's forecast, shield_audit.jsonl as the rail's
        AuditLogger writes it) replays `identical` open loop, every audit
        record compared;
      - a headless episode bundle (manifest_extras.json, harness_events,
        kpi.json, shield_audit.jsonl, as the sweep writes it) is re-flown
        `identical` by the revision's own rerun tool;
  * the check can fail: one audit float changed by one ulp in the world
    frame reads `differs` (class `float_last_digits`) at that tick and
    field; the same change in the body-frame action alone reads `differs` of
    class `body_frame_rotation_last_digits`; a rule id changed reads class
    `decision`; a changed headless audit record and a changed KPI figure read
    `differs`; the fields the replay does not reproduce by design (`ts`, the
    node's fsm_*) are not compared;
  * a GPS dropout and an estimator loss whose rows log the pipe the Shield
    read (`note_flown`, guardrail.replay.PipeRecorder, 2026-11-21) replay
    `identical`, the Shield's X8 verdict compared per tick; the same rows
    without it keep the old refusal word for word; a changed setpoint sent
    reads `differs` at position_lost;
  * a subject_standoff run with a DECLARED subject that metrics.json records
    replays identical (set_subject as the node set it); with no record of
    the subject's class it is not_replayable;
  * the replay bundle alone (a folder holding only <run>.replay.tar.gz)
    replays identical: it carries trajectory.json since 2026-11-18
    (guardrail.replay EPISODE_FILES), so the positions are the doubles the
    Shield read, not the log's 1 mm;
  * a sweep's headless bundle carries the grant's policy_bundle.tar.gz with
    every generation's IR, and kpi.json `bundle_files` names the six
    members, with why episode.bag2/ is not written;
  * the worktree is gone afterwards (not on disk, not in `git worktree
    list`), and a junction / symlink inside a folder is unlinked without
    deleting its target's files;
  * rows that jump two generations in one tick are installed through the
    one between, in order, each commit recorded (`installs`); a `decision`
    record names its ticks and whether the emitted action is the flown one
    (`decision_ticks`);
  * a run is dated by its own flight record (the episode folder's UTC
    stamp, else events.jsonl's first wall time), not by manifest.json's file
    time; `flown_on` carries the flown interpreter where the run recorded it;
  * the CLI writes the summary and the per-episode records, and exits 0 / 1
    / 2; --python picks the drivers' interpreter (the summary's replayed_on
    is theirs) and --dump-dir keeps every difference of a differing episode.

The end-to-end tests SKIP outside a git checkout, and when guardrail/,
experiments/, policies/ or tools/rerun_episode.py have uncommitted changes
(HEAD would not hold the code that made the episodes; the driver itself is
this checkout's tools/replay_at_revision.py, run on HEAD's modules).
"""
from __future__ import annotations

import contextlib
import gzip
import io
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "experiments"))

import replay_at_revision as RA                                   # noqa: E402

SKIP = "SKIP"
DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
PEDESTRIAN = ROOT / "policies" / "sitl_pedestrian.yaml"
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}
_TMP: list[Path] = []
_CACHE: dict = {}
# The synthetic dropout (_rail_run dropout=True): the ticks [from, to) without
# a valid fix, and those with the autopilot's estimator declared lost.
DROPOUT_FIX = (24, 52)
DROPOUT_EST = (44, 47)


def _tmp(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


def _head() -> str | None:
    try:
        r = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True,
                           text=True, timeout=30)
    except OSError:
        return None
    return r.stdout.strip() if r.returncode == 0 and len(r.stdout.strip()) == 40 else None


def _head_holds_this_code() -> str | None:
    """Why HEAD does not hold the code this process runs, or None."""
    if _head() is None:
        return "not a git checkout"
    r = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "--", "guardrail",
                        "experiments", "policies", "tools/rerun_episode.py"],
                       capture_output=True, text=True)
    if r.stdout.strip():
        return "uncommitted changes under guardrail/, experiments/ or the tools"
    return None


# --------------------------------------------------------------------------- #
# episodes flown here, at HEAD's code
# --------------------------------------------------------------------------- #

def _rail_run(run: Path, rev: str, *, ticks: int = 80, spawn_at: int = 22,
              subject: tuple | None = None, record_subject: bool = True,
              two_edits: bool = False, clear_response: dict | None = None,
              dropout: bool = False) -> None:
    """A run as the ROS 2 rail stores one, its Shield the rail's (no FSM
    inside), its audit written by AuditLogger as the rail writes it.
    `subject` (x, y, class): a DECLARED subject set once before the first
    tick, on the pedestrian policy, as the node's --subject does; metrics.json
    records it unless `record_subject` is false. `two_edits`: the spawned
    zone is moved in the same tick (a spawn and a rotate at one mission
    time, as dyn-nfz-rotate-spin flies), so the rows go from generation 0
    to 2. `clear_response`: the Shield's clearance-horizon response, as the
    ArduPilot rails pass and record it (shield_forecast) since 2026-11-19.
    `dropout` (2026-11-21, WP4-11b): the GPS fix lost over DROPOUT_FIX and
    the autopilot's estimator declared lost over DROPOUT_EST, the pose handed
    on frozen meanwhile (a stale fix) while the aircraft flies on; the rail
    sends its own setpoint after each tick (some ticks a stop, some nothing,
    the rest the Shield's answer) on a jittered real-time pipe clock read
    through guardrail.replay.PipeRecorder, and every row logs fix_ok,
    estimator_ok, position_lost and the pipe (`note_flown`), as the ROS 2
    node does."""
    from guardrail.audit import AuditLogger
    from guardrail.models import Action4D, DynamicNFZ, State, load_policy
    from guardrail.replay import rail_shield
    run.mkdir(parents=True)
    pol = load_policy(PEDESTRIAN if subject else DEMO)
    kw: dict = {}
    wall = [1000.0 + 1 / 3]
    pipe = None
    if dropout:
        from guardrail.replay import PipeRecorder
        pipe = kw["pipe_clock"] = PipeRecorder(lambda: wall[0])
    if clear_response is not None:
        from guardrail.response import VelocityResponse
        sh = rail_shield(pol, lookahead_s=5.0, dt=0.1,
                         clear_response=VelocityResponse(**clear_response), **kw)
    else:
        sh = rail_shield(pol, lookahead_s=5.0, dt=0.1, **kw)
    if subject:
        sh.set_subject(*subject)
    (run / "policy_g0.json").write_bytes(sh.policy.canonical_bytes())
    logger = AuditLogger.in_memory(sh.policy, episode_id="1234")
    rows, traj, audit = [], [], []
    x, y, up, yaw = 1 / 3, -2 / 7, 15.0 + 1 / 9, 12.5 + 1 / 7
    held = (x, y)
    for k in range(ticks):
        if k == spawn_at:
            sh.hot_apply(DynamicNFZ(
                id="bar", type="dynamic_nfz", **HARD_P0,
                vertices=[{"x": x + 9, "y": y - 6}, {"x": x + 11, "y": y - 6},
                          {"x": x + 11, "y": y + 6}, {"x": x + 9, "y": y + 6}],
                altitude_floor_m=0, altitude_ceiling_m=100, margin_m=0.5,
                motion={"vx_mps": -0.3, "vy_mps": 0.1}))
            (run / f"policy_g{sh.policy.generation}.json").write_bytes(
                sh.policy.canonical_bytes())
            if two_edits:
                sh.move_nfz("bar", [(x + 10, y - 7), (x + 12, y - 7), (x + 12, y + 7),
                                    (x + 10, y + 7)],
                            motion={"vx_mps": -0.25, "vy_mps": 0.05})
                (run / f"policy_g{sh.policy.generation}.json").write_bytes(
                    sh.policy.canonical_bytes())
        raw = Action4D(vx=4.5 + 0.01 * k, vy=0.3 * math.sin(k / 7.0),
                       vz_up=1.2 if k < 30 else -0.1, yaw_rate=0.0)
        quality: dict = {}
        hx, hy = x, y
        if dropout:
            lost = DROPOUT_FIX[0] <= k < DROPOUT_FIX[1]
            est = False if DROPOUT_EST[0] <= k < DROPOUT_EST[1] else None
            quality = {"fix_ok": not lost, "estimator_ok": est}
            if lost:
                hx, hy = held
            else:
                held = (x, y)
            wall[0] = 1000.0 + 1 / 3 + 0.1 * k + 0.013 * math.sin(k)
            pipe.begin_tick()
            wall[0] += 0.021 + 0.004 * math.cos(k)
        d = sh.filter(State(x=hx, y=hy, up=up, yaw_deg=yaw,
                            **{q: v for q, v in quality.items() if v is not None}), raw)
        if dropout:
            reads = pipe.end_tick()
            wall[0] += 0.017 + 0.003 * (k % 3)
            sent = (None if k % 9 == 4 else Action4D() if k % 7 == 3
                    else Action4D(vx=d.emitted.vx * 0.75, vy=d.emitted.vy) if k % 5 == 1
                    else d.emitted)
            quality.update(position_lost=d.position_lost[0] if d.position_lost else None,
                           note_flown=pipe.note_flown(sh, sent, reads))
        rec = logger.record(k + 1, d, fsm_record={"fsm_state_before": "Normal",
                                                   "fsm_state_after": "Normal",
                                                   "reason": "rail FSM", "transition": False})
        if rec is not None:
            line = AuditLogger.line(rec)
            line["episode_id"] = "1234"
            audit.append(line)
        rows.append({"t": round(k * 0.1, 3), "tick": k + 1, "x": round(hx, 3),
                     "y": round(hy, 3), "up": round(up, 3), "yaw_deg": round(yaw, 2),
                     "raw": d.raw.model_dump(), "emitted": d.emitted.model_dump(),
                     "violations": [v.model_dump() for v in d.violations],
                     "repairs": [r.model_dump() for r in d.repairs],
                     "touched": bool(d.touched), "braked": bool(d.braked),
                     "generation": sh.policy.generation, "policy_hash": sh.policy.policy_hash,
                     **({"tgt_x": subject[0], "tgt_y": subject[1]} if subject else {}),
                     **quality})
        traj.append({"t": round(k * 0.1, 2), "x": hx, "y": hy, "up": up})
        x, y, up = x + 0.1 * d.emitted.vx, y + 0.1 * d.emitted.vy, up + 0.1 * d.emitted.vz_up
    with (run / "flight_log.jsonl").open("w", encoding="utf-8", newline="\n") as fh:
        fh.writelines(json.dumps(r) + "\n" for r in rows)
    with (run / "shield_audit.jsonl").open("w", encoding="utf-8", newline="\n") as fh:
        fh.writelines(json.dumps(a) + "\n" for a in audit)
    (run / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")
    (run / "metrics.json").write_text(json.dumps({
        "shield": "on", "shield_lookahead": {"horizon_s": 5.0, "dt_s": 0.1, "poses": 50},
        "shield_forecast": {"mode": "commanded", "velocity_ticks": 0,
                            **({"clear_response": clear_response} if clear_response
                               else {})},
        **({"subject": {"class": subject[2], "x": subject[0], "y": subject[1],
                        "position_source": "declared"}} if subject and record_subject
           else {})}), encoding="utf-8")
    (run / "manifest.json").write_text(json.dumps({
        "code_revision": rev, "policy_hash": pol.policy_hash, "random_seed": 0,
        "sim_speedup": 1.0, "topology": "dev"}), encoding="utf-8")
    assert len(audit) > 10 and {r["generation"] for r in rows} == (
        {0, 2} if two_edits else {0, 1}), "the synthetic flight gave the Shield nothing to audit"


def _headless_bundle(ep_dir: Path, rev: str, sid: str = "nfz-head-on") -> None:
    """A headless episode bundle as the sweep writes one."""
    import sweep_scenarios as S
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    spec = lib.by_id()[sid].first.spec
    run = S.run_episode(spec, lib.defaults, 0)
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    manifest = {"code_revision": rev, "vla_model_hash": "test", "policy_hash":
                run.policy_hash_start, "random_seed": 0, "sim_speedup": 50.0,
                "topology": "headless-kinematic"}
    S.write_bundle(ep_dir, manifest, S._json_safe({"kpi": res}), run.events, None,
                   S._json_safe(S.episode_extras(spec, lib.defaults, 0, run)), audit=run.audit)
    assert run.audit, "the headless episode wrote no audit record"


def _edit_jsonl(p: Path, i: int, fn) -> None:
    lines = [json.loads(x) for x in p.read_text(encoding="utf-8").splitlines() if x.strip()]
    fn(lines[i])
    p.write_text("".join(json.dumps(x) + "\n" for x in lines), encoding="utf-8")


def _first_touched(p: Path) -> int:
    lines = [json.loads(x) for x in p.read_text(encoding="utf-8").splitlines() if x.strip()]
    return next(i for i, a in enumerate(lines) if a.get("repairs"))


def _replayed() -> dict | str:
    """Every episode below replayed in ONE call (one worktree at HEAD)."""
    if "out" in _CACHE:
        return _CACHE["out"]
    why = _head_holds_this_code()
    if why is not None:
        print(f"      (end-to-end SKIP: {why})")
        _CACHE["out"] = SKIP
        return SKIP
    head = _head()
    rev = head[:12]
    root = _tmp("rar_")
    _rail_run(root / "rail_ok", rev)
    # the same flight, one world-frame float one ulp off on a repaired tick
    shutil.copytree(root / "rail_ok", root / "rail_world_ulp")
    p = root / "rail_world_ulp" / "shield_audit.jsonl"
    i = _first_touched(p)
    _edit_jsonl(p, i, lambda a: a["emitted_action_world"].__setitem__(
        "vx", math.nextafter(a["emitted_action_world"]["vx"], math.inf)))
    _CACHE["world_tick"] = json.loads(p.read_text(encoding="utf-8").splitlines()[i])["tick"]
    # one ulp in the body-frame action only: the writer's turn at the heading
    shutil.copytree(root / "rail_ok", root / "rail_body_ulp")
    _edit_jsonl(root / "rail_body_ulp" / "shield_audit.jsonl", i,
                lambda a: a["raw_action"].__setitem__(
                    "vx", math.nextafter(a["raw_action"]["vx"], math.inf)))
    # what is not compared by design: the wall time and the node's FSM
    shutil.copytree(root / "rail_ok", root / "rail_ts_fsm")
    _edit_jsonl(root / "rail_ts_fsm" / "shield_audit.jsonl", i,
                lambda a: a.update(ts="1999-01-01T00:00:00.000+00:00",
                                   fsm_reason="another FSM", fsm_state_after="Brake"))
    # a rule id changed on one record: not a float, a decision
    shutil.copytree(root / "rail_ok", root / "rail_rule_id")
    _edit_jsonl(root / "rail_rule_id" / "shield_audit.jsonl", i,
                lambda a: a["violations"][0].__setitem__("rule_id", "another-rule"))
    # two hot applies between two ticks: the rows jump from generation 0 to 2
    _rail_run(root / "rail_two_edits", rev, two_edits=True)
    # a declared subject, recorded in metrics.json; and one that is not
    _rail_run(root / "rail_subject", rev, subject=(20.0, 1.0, "pedestrian"))
    _rail_run(root / "rail_subject_unrecorded", rev, subject=(20.0, 1.0, "pedestrian"),
              record_subject=False)
    # a rail Shield with the ArduPilot clearance response, recorded as the
    # ROS 2 node and the pymavlink SITL rail record it
    from guardrail.response import ARDUCOPTER_GUIDED
    _rail_run(root / "rail_clear_response", rev, clear_response=ARDUCOPTER_GUIDED.to_dict())
    # a GPS dropout and an estimator loss with the pipe logged per row (the
    # ROS 2 node since 2026-11-21); the same rows without it (a run flown
    # before); and one setpoint the rail sent changed on a dropout tick
    _rail_run(root / "rail_dropout", rev, dropout=True,
              clear_response=ARDUCOPTER_GUIDED.to_dict())
    shutil.copytree(root / "rail_dropout", root / "rail_dropout_unlogged")
    for i in range(80):
        _edit_jsonl(root / "rail_dropout_unlogged" / "flight_log.jsonl", i,
                    lambda r: r.pop("note_flown"))
    shutil.copytree(root / "rail_dropout", root / "rail_dropout_sent_edit")
    _edit_jsonl(root / "rail_dropout_sent_edit" / "flight_log.jsonl", DROPOUT_FIX[0] + 5,
                lambda r: r["note_flown"].__setitem__("vx", r["note_flown"]["vx"] + 2.0))
    # the replay bundle alone: a folder holding nothing but <run>.replay.tar.gz
    from guardrail.replay import last_generation_policy, write_replay
    (root / "tar_only").mkdir()
    write_replay(root / "rail_ok", last_generation_policy(root / "rail_ok"),
                 root / "tar_only" / "tar_only.replay.tar.gz", signer=None)
    _headless_bundle(root / "headless_ok", rev)
    shutil.copytree(root / "headless_ok", root / "headless_audit_edit")
    _edit_jsonl(root / "headless_audit_edit" / "shield_audit.jsonl", 0,
                lambda a: a.update(violations=a["violations"][:-1] or [{"rule_id": "x"}]))
    shutil.copytree(root / "headless_ok", root / "headless_kpi_edit")
    k = json.loads((root / "headless_kpi_edit" / "kpi.json").read_text(encoding="utf-8"))
    k["kpi"]["repair_count"] = int(k["kpi"]["repair_count"]) + 1
    (root / "headless_kpi_edit" / "kpi.json").write_text(json.dumps(k), encoding="utf-8")
    # refused before any worktree: a dirty tree and a SHA that is no commit here
    shutil.copytree(root / "rail_ok", root / "rail_dirty")
    m = json.loads((root / "rail_dirty" / "manifest.json").read_text(encoding="utf-8"))
    (root / "rail_dirty" / "manifest.json").write_text(
        json.dumps({**m, "code_revision": rev + "-dirty"}), encoding="utf-8")
    shutil.copytree(root / "rail_ok", root / "rail_foreign")
    (root / "rail_foreign" / "manifest.json").write_text(
        json.dumps({**m, "code_revision": "0123456789ab"}), encoding="utf-8")
    scratch = _tmp("rar_wt_")
    selected = RA.select(RA.find_runs(root), [], [])
    got = RA.replay(selected, repo=ROOT, root=root, scratch=scratch, n_proc=2,
                    log=lambda *_: None)
    _CACHE.update(out={r["run"]: r for r in got["records"]}, worktrees=got["worktrees"],
                  root=root, scratch=scratch, head=head)
    return _CACHE["out"]


# --------------------------------------------------------------------------- #
# the revision
# --------------------------------------------------------------------------- #

def test_a_revision_no_checkout_reproduces_is_refused_with_its_reason():
    assert RA.revision_refusal("116214f3e6ee") is None
    assert RA.revision_refusal("a" * 40) is None
    for rev, word in ((None, "no code_revision"), ("", "no code_revision"),
                      ("unversioned", "unversioned"), ("116214f3e6ee-dirty", "uncommitted"),
                      ("116214f3e6ee-unknown", "could not say"),
                      ("git-unresolved", "could not say"), ("v1.2", "not a commit id")):
        why = RA.revision_refusal(rev)
        assert why and word in why, (rev, why)
    head = _head()
    if head is None:
        return SKIP
    assert RA.resolve_commit(ROOT, head[:12]) == head
    assert RA.resolve_commit(ROOT, "0123456789ab") is None


# --------------------------------------------------------------------------- #
# the replay at HEAD
# --------------------------------------------------------------------------- #

def test_a_rail_run_flown_at_head_replays_identical_open_loop_record_by_record():
    out = _replayed()
    if out == SKIP:
        return SKIP
    r = out["rail_ok"]
    assert r["verdict"] == "identical" and r["kind"] == "open_loop", r
    a = r["audit"]
    assert a["n_stored"] > 10 and a["n_replayed"] == a["n_stored"] and a["identical"], a
    assert a["touched_ticks_without_record"] == 0 and a["records_without_row"] == 0, a
    assert a["excluded_fields"] == ["ts", "fsm_*"], a
    assert r["generations"] == [0, 1] and r["ticks"] == 80, r
    assert r["state_source"].startswith("trajectory.json"), r
    assert r["code_revision_flown"] == _CACHE["head"][:12] and r["sha"] == _CACHE["head"]


def test_the_replay_bundle_alone_is_enough_and_carries_the_positions_doubles():
    """The tarball is self-contained for the open-loop replay: manifest,
    metrics, every generation, the audit and, since 2026-11-18,
    trajectory.json (guardrail.replay EPISODE_FILES) - without it the
    replay would get the log's 1 mm positions."""
    out = _replayed()
    if out == SKIP:
        return SKIP
    t = out["tar_only"]
    assert t["verdict"] == "identical", t
    assert t["state_source"].startswith("trajectory.json"), t
    assert t["audit"]["n_stored"] == out["rail_ok"]["audit"]["n_stored"], t["audit"]
    assert "replay.tar.gz" in t["audit"]["source"] and "replay.tar.gz" in t["log"]["source"]
    assert sorted(p.name for p in (_CACHE["root"] / "tar_only").iterdir()) == [
        "tar_only.replay.tar.gz"]


def test_the_replay_bundle_packs_trajectory_json():
    from guardrail.replay import (EPISODE_FILES, STALE_PATTERNS, last_generation_policy,
                                  read_replay, verify_replay, write_replay)
    assert EPISODE_FILES["trajectory.json"] is False and "trajectory.json" in STALE_PATTERNS
    run = _tmp("rar_pack_") / "r"
    _rail_run(run, "test")
    p = write_replay(run, last_generation_policy(run), run.parent / "r.replay.tar.gz",
                     signer=None)
    got = read_replay(p)
    want = json.loads((run / "trajectory.json").read_text(encoding="utf-8"))
    assert got["trajectory"] == want and len(want) == 80
    verify_replay(p)
    (run / "trajectory.json").unlink()
    p2 = write_replay(run, last_generation_policy(run), run.parent / "r2.replay.tar.gz",
                      signer=None)
    assert read_replay(p2)["trajectory"] is None        # optional, as before


def test_a_sweep_bundle_carries_the_grants_policy_bundle_with_every_generation():
    """experiments/sweep_scenarios.py's headless bundles: policy_bundle.tar.gz
    (the policy file and each generation's IR), and kpi.json `bundle_files`
    naming all six of the grant's members and why episode.bag2/ is not one."""
    import tarfile
    import yaml
    import sweep_scenarios as S
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    raw = yaml.safe_load((ROOT / "experiments" / "scenarios.yaml").read_text(encoding="utf-8"))
    fam = lib.raw("dyn-nfz-spawn-on-top")
    d = _tmp("rar_sweep_")
    (d / "lib.yaml").write_text(yaml.safe_dump({"defaults": raw["defaults"],
                                                "scenarios": [fam]}), encoding="utf-8")
    with contextlib.redirect_stdout(io.StringIO()):
        S.main(["--library", str(d / "lib.yaml"), "--out", str(d / "s.json"),
                "--runs", str(d / "runs")])
    eps = sorted(p for p in (d / "runs").iterdir() if p.is_dir())
    assert eps, "the sweep wrote no bundle"
    most = 0
    for ep in eps:
        names = {p.name for p in ep.iterdir()}
        assert {"manifest.json", "policy_bundle.tar.gz", "shield_audit.jsonl",
                "harness_events.jsonl", "kpi.json"} <= names, (ep.name, names)
        doc = json.loads((ep / "kpi.json").read_text(encoding="utf-8"))
        files = doc["bundle_files"]
        assert set(files) == {"manifest.json", "policy_bundle.tar.gz", "episode.bag2/",
                              "shield_audit.jsonl", "harness_events.jsonl", "kpi.json"}
        assert files["policy_bundle.tar.gz"] == "written, complete", files
        assert files["episode.bag2/"].startswith("not written: the headless backend")
        with tarfile.open(ep / "policy_bundle.tar.gz") as tf:
            members = set(tf.getnames())
            index = json.loads(tf.extractfile("policy_bundle.json").read())
        gens = [g["generation"] for g in index["generations"]]
        assert index["complete"] and all(f"generations/policy_g{n}.json" in members
                                         for n in gens), (members, index)
        assert any(m.startswith("injected/") for m in members), members
        man = json.loads((ep / "manifest.json").read_text(encoding="utf-8"))
        assert index["generations"][0]["policy_hash"] == man["policy_hash"], index
        most = max(most, len(gens))
    assert most >= 2, "no hot-applied generation: the 'all generations' half is untested"


def test_the_open_loop_replay_can_fail_and_says_where():
    out = _replayed()
    if out == SKIP:
        return SKIP
    w = out["rail_world_ulp"]
    fd = w["first_difference"]
    assert w["verdict"] == "differs" and w["difference_class"] == "float_last_digits", w
    assert 0 < w["audit"]["fields_differing"]["emitted_action_world"]["max_abs"] < 1e-14, w
    assert (fd["where"], fd["tick"], fd["field"]) == (
        "shield_audit.jsonl", _CACHE["world_tick"], "emitted_action_world"), fd
    b = out["rail_body_ulp"]
    assert b["verdict"] == "differs", b
    assert b["difference_class"] == "body_frame_rotation_last_digits", b
    assert set(b["audit"]["fields_differing"]) == {"raw_action"}, b["audit"]
    r = out["rail_rule_id"]
    assert r["verdict"] == "differs" and r["difference_class"] == "decision", r
    assert r["first_difference"]["field"] == "violations", r["first_difference"]
    # where it is a decision, and whether the emitted action is still the flown one
    assert r["decision_ticks"] == [{"tick": _CACHE["world_tick"], "audit_fields": ["violations"],
                                    "log_fields": [], "emitted_bit_identical": True,
                                    "emitted_max_abs_diff": None}], r["decision_ticks"]
    assert w["decision_ticks"] is None and b["decision_ticks"] is None
    # the wall time and the node's FSM verdict are not the Shield's to replay
    assert out["rail_ts_fsm"]["verdict"] == "identical", out["rail_ts_fsm"]


def test_two_hot_applies_in_one_tick_are_installed_in_the_order_the_node_committed():
    """Rows that go from generation 0 to 2 in one tick (a spawn and a move
    between two ticks, as dyn-nfz-rotate-spin flies) are installed through
    generation 1, then 2, at the Shield's own event time (2026-11-18 review:
    the replay had committed 2 straight from 0); each commit is recorded,
    and the replay is identical."""
    out = _replayed()
    if out == SKIP:
        return SKIP
    t = out["rail_two_edits"]
    assert t["verdict"] == "identical" and t["generations"] == [0, 2], t
    (inst,) = t["installs"]
    assert (inst["from"], inst["to"]) == (0, 2), inst
    assert [c["generation"] for c in inst["commits"]] == [1, 2], inst
    assert len({c["event_time"] for c in inst["commits"]}) == 1, inst
    assert all(c["anchored"] == {"bar": c["event_time"]} for c in inst["commits"]), inst


def test_a_run_is_dated_by_its_own_flight_record_not_a_file_time():
    """The Q3 window reads the date the episode flew: the UTC stamp in an
    episode folder's name, else the first wall time of its events.jsonl;
    manifest.json's file time (which a copy, a restore or a sync changes)
    only when the run holds neither, and the record says which."""
    root = _tmp("rar_date_")
    late = 1798761600                      # 2027-01-01: a copy made long after the flight
    ep = root / "episode-2026-10-22T15-33-56Z--nfz-head-on--seed0"
    rail, bare = root / "rail_run", root / "bare_run"
    for d in (ep, rail, bare):
        d.mkdir()
        (d / "manifest.json").write_text(json.dumps({"code_revision": "a" * 12,
                                                     "topology": "dev"}), encoding="utf-8")
        os.utime(d / "manifest.json", (late, late))
    (rail / "events.jsonl").write_text(
        json.dumps({"kind": "episode_start", "t": None,
                    "wall": "2026-10-31T17:30:00.000+00:00"}) + "\n", encoding="utf-8")
    assert RA.flight_date(ep) == (RA.date(2026, 10, 22), RA.FLIGHT_DATE_BASES[0])
    assert RA.flight_date(rail) == (RA.date(2026, 10, 31), RA.FLIGHT_DATE_BASES[1])
    d, basis = RA.flight_date(bare)
    assert d.year == 2027 and basis == RA.FLIGHT_DATE_BASES[2], (d, basis)
    q3 = [(RA.date(2026, 8, 1), RA.date(2026, 10, 31))]
    picked = {r.name for r, _, _ in RA.select([ep, rail, bare], q3, [])}
    assert picked == {ep.name, rail.name}, picked
    groups, refused = RA.plan(RA.select([ep], q3, []), ROOT, root)
    rec = (refused or [j for js in groups.values() for j in js])[0]
    assert rec["flight_date"] == "2026-10-22" and \
        rec["flight_date_basis"] == RA.FLIGHT_DATE_BASES[0], rec


def test_flown_on_names_the_interpreter_where_the_run_recorded_it():
    """A last-digit difference has two candidate causes, the host's libm and
    the interpreter's float routines: flown_on carries both where the run
    recorded them (metrics.json hil_evidence, gc.python), never a guess."""
    ev = {"host_arch": "aarch64", "device_model": "Orin", "jetpack": None}
    assert RA._flown_on({"hil_evidence": ev, "gc": {"python": "3.12.3"}}) == {
        "host_arch": "aarch64", "device_model": "Orin", "python": "3.12.3"}
    assert RA._flown_on({"hil_evidence": ev}) == {"host_arch": "aarch64",
                                                  "device_model": "Orin"}
    assert RA._flown_on({"gc": {"python": "3.12.3"}}) == {"python": "3.12.3"}
    assert RA._flown_on({}) is None


def test_a_declared_subject_is_replayed_and_an_unrecorded_one_is_refused():
    out = _replayed()
    if out == SKIP:
        return SKIP
    s = out["rail_subject"]
    assert s["verdict"] == "identical", s
    rows = [json.loads(x) for x in (_CACHE["root"] / "rail_subject" / "flight_log.jsonl")
            .read_text(encoding="utf-8").splitlines()]
    assert any(v["rule_id"].startswith("standoff") for r in rows for v in r["violations"]), \
        "the subject never bound: the declared-subject replay is untested"
    u = out["rail_subject_unrecorded"]
    assert u["verdict"] == "not_replayable" and "subject_standoff" in u["why"], u


def test_a_recorded_clear_response_is_rebuilt_not_refused():
    """The rails record shield_forecast.clear_response (2026-11-19); a key
    outside KNOWN_FORECAST_KEYS refuses the episode, so without this every
    hil / SITL run flown since would be refused for replay at revision."""
    assert "clear_response" in RA.KNOWN_FORECAST_KEYS
    out = _replayed()
    if out == SKIP:
        return SKIP
    from guardrail.response import ARDUCOPTER_GUIDED
    r = out["rail_clear_response"]
    assert r["verdict"] == "identical" and r["kind"] == "open_loop", r
    assert r["shield"]["clear_response"] == ARDUCOPTER_GUIDED.to_dict(), r["shield"]
    assert out["rail_ok"]["shield"]["clear_response"] is None, out["rail_ok"]["shield"]


OLD_PIPE_REFUSAL = ("the pipe the Shield read is not logged per tick: {} row(s) with no "
                    "valid fix and {} with the estimator unhealthy, where the Shield "
                    "dead-reckons on what the rail sent (note_flown, on a real-time pipe "
                    "clock no rail logs)")


def test_a_gps_dropout_and_an_estimator_loss_replay_identical_on_the_logged_pipe():
    """WP4-11b (2026-11-21): a run whose rows log the pipe the Shield read
    (`note_flown`: what the rail sent and the pipe clock's readings, as the
    ROS 2 node logs it) replays a GPS dropout and an estimator loss open
    loop, `identical`, the Shield's X8 verdict (`position_lost`) compared
    tick by tick - both the dead reckoning's and the estimator's. The same
    rows without the field (every run flown before) keep the refusal, word
    for word; one setpoint sent changed on a dropout tick reads `differs`
    at a later tick's position_lost: the logged pipe is what the replay
    reads."""
    out = _replayed()
    if out == SKIP:
        return SKIP
    rows = [json.loads(x) for x in (_CACHE["root"] / "rail_dropout" / "flight_log.jsonl")
            .read_text(encoding="utf-8").splitlines()]
    nofix = sum(1 for r in rows if r["fix_ok"] is False)
    noest = sum(1 for r in rows if r["estimator_ok"] is False)
    assert (nofix, noest) == (DROPOUT_FIX[1] - DROPOUT_FIX[0],
                              DROPOUT_EST[1] - DROPOUT_EST[0]), (nofix, noest)
    lost = [r["position_lost"] for r in rows if r["position_lost"]]
    assert any(s.startswith("no valid fix") for s in lost), lost
    assert any(s.startswith("the autopilot reports no valid") for s in lost), lost
    assert sum(len(r["note_flown"]["pipe_reads"]) for r in rows) > 0, \
        "the Shield never read the pipe clock: the clearance pipe is untested"
    r = out["rail_dropout"]
    assert r["verdict"] == "identical" and r["kind"] == "open_loop", r
    assert r["shield"]["pipe"].startswith("fed from the rows' note_flown"), r["shield"]
    assert r["ticks"] == 80 and r["audit"]["n_replayed"] == r["audit"]["n_stored"] > 10, r
    assert out["rail_clear_response"]["shield"]["pipe"] == "none"
    u = out["rail_dropout_unlogged"]
    assert u["verdict"] == "not_replayable", u
    assert u["why"] == OLD_PIPE_REFUSAL.format(nofix, noest), u["why"]
    e = out["rail_dropout_sent_edit"]
    fd = e["first_difference"]
    assert e["verdict"] == "differs" and e["difference_class"] == "decision", e
    assert (fd["where"], fd["field"]) == ("flight_log.jsonl", "position_lost"), fd
    assert fd["tick"] > DROPOUT_FIX[0] + 6, fd


def test_the_ros2_nodes_own_dropout_run_replays_identical_on_the_pipe_it_logged():
    """sitl/ros2_shield_node.py on tests/test_railinputs_node.py's stand-in
    ROS stack (tests/test_sitl_stressors.py's harness), the GPS fix lost for
    3.5 s beside nfz-square while the pilot asks 2 m/s north: every row logs
    `note_flown`, the node's FSM brakes and holds (what was sent is not what
    the Shield emitted), and the run folder the node wrote replays open loop
    at this checkout `identical`, its X8 verdicts compared; with the field
    taken off the rows it is refused as before."""
    import test_sitl_stressors as TS
    R, N = TS._node_module()
    node, clk, out = TS._node(R, N, [])
    node.start_mission()
    for k in range(60):
        node._on_fix(N.NavSatFix(status=R._Msg(status=-1 if 10 <= k < 45 else 0)))
        node._on_pose(R._pose_msg(N, node.state))
        R._act(N, node, vx=2.0)
        clk.t += 0.1
        node._tick()
    with contextlib.redirect_stdout(io.StringIO()):
        node._finish("test over")
    _TMP.append(out)
    rows = node.rows
    assert all(RA._pipe_note(r.get("note_flown")) is not None for r in rows), rows[0]
    assert sum(1 for r in rows if r.get("fix_ok") is False) == 35
    assert sum(1 for r in rows if r.get("position_lost")) > 0, "X8 never judged"
    assert any(r["note_flown"]["pipe_reads"] for r in rows), "the pipe clock was never read"
    rec = RA._open_loop(out, ROOT)
    assert rec["verdict"] == "identical", rec["first_difference"]
    assert rec["shield"]["pipe"].startswith("fed from"), rec["shield"]
    lines = [json.loads(x) for x in (out / "flight_log.jsonl").read_text(
        encoding="utf-8").splitlines()]
    for r in lines:
        r.pop("note_flown")
    (out / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in lines),
                                          encoding="utf-8")
    try:
        RA._open_loop(out, ROOT)
    except RA.Refused as e:
        assert str(e) == OLD_PIPE_REFUSAL.format(35, 0), str(e)
    else:
        raise AssertionError("rows without note_flown were replayed, not refused")


def test_the_pipe_recorder_keeps_the_filters_readings_and_what_was_sent():
    """guardrail.replay.PipeRecorder (2026-11-21): the Shield's pipe clock
    reads as before, only the readings between begin_tick and end_tick are
    kept, and note_flown notes the setpoint at the clock's reading now and
    returns the row field the replay feeds back (_pipe_note); nothing sent
    and a non-finite setpoint are both None, as Shield.note_flown records
    them alike."""
    from guardrail.models import Action4D
    from guardrail.replay import PIPE_ROW_KEY, PipeRecorder
    assert PIPE_ROW_KEY == RA.PIPE_ROW_KEY == "note_flown"
    now = [5.0]
    rec = PipeRecorder(lambda: now[0])
    assert rec() == 5.0 and rec.end_tick() == []          # outside a tick: not kept
    rec.begin_tick()
    now[0] = 5.25
    assert rec() == 5.25 and rec() == 5.25
    assert rec.end_tick() == [5.25, 5.25] and rec.end_tick() == []

    class _Sh:
        def __init__(self):
            self.noted = []

        def note_flown(self, action, t=None):
            self.noted.append((action, t))
    sh = _Sh()
    now[0] = 5.3
    f = rec.note_flown(sh, Action4D(vx=1 / 3, vy=-2.5), [5.25])
    assert f == {"t": 5.3, "vx": 1 / 3, "vy": -2.5, "pipe_reads": [5.25]}, f
    assert sh.noted[-1][1] == 5.3
    assert rec.note_flown(sh, None, [])["vx"] is None and sh.noted[-1] == (None, 5.3)
    g = rec.note_flown(sh, Action4D(vx=float("nan"), vy=0.0), [])
    assert g["vx"] is None and g["vy"] is None, g
    assert RA._pipe_note(f) == f and RA._pipe_note(g) == g
    for bad in (None, {}, {"t": 1.0}, {"t": 1.0, "vx": 1.0, "vy": None, "pipe_reads": []},
                {"t": float("nan"), "vx": None, "vy": None, "pipe_reads": []},
                {"t": 1.0, "vx": None, "vy": None, "pipe_reads": [True]},
                {"t": "1", "vx": None, "vy": None, "pipe_reads": []}):
        assert RA._pipe_note(bad) is None, bad


def test_a_headless_episode_flown_at_head_is_reflown_identical_by_that_revisions_tool():
    out = _replayed()
    if out == SKIP:
        return SKIP
    h = out["headless_ok"]
    assert h["verdict"] == "identical" and h["kind"] == "refly", h
    assert h["rebuilt_from"] == "manifest_extras", h
    assert set(h["compared"]) == {"shield_audit.jsonl", "harness_events.jsonl",
                                  "kpi.json kpi"}, h["compared"]
    assert h["audit"]["n_stored"] == h["audit"]["n_replayed"] > 0, h["audit"]
    e = out["headless_audit_edit"]
    assert e["verdict"] == "differs" and e["first_difference"]["where"] == \
        "shield_audit.jsonl" and e["first_difference"]["field"] == "violations", e
    k = out["headless_kpi_edit"]
    assert k["verdict"] == "differs" and k["first_difference"]["where"] == "kpi.json kpi" \
        and k["first_difference"]["field"] == "repair_count", k


def test_dirty_and_foreign_revisions_are_refused_and_never_replayed():
    out = _replayed()
    if out == SKIP:
        return SKIP
    d, f = out["rail_dirty"], out["rail_foreign"]
    assert d["verdict"] == "refused" and "uncommitted" in d["why"], d
    assert f["verdict"] == "refused" and "not a commit of this repository" in f["why"], f
    assert "ticks" not in d and "ticks" not in f


def test_the_worktree_is_removed_and_pruned():
    out = _replayed()
    if out == SKIP:
        return SKIP
    wts = _CACHE["worktrees"]
    assert len(wts) == 1 and wts[0]["sha"] == _CACHE["head"], wts
    assert wts[0]["removed"] is True and wts[0]["git_remove_rc"] == 0, wts
    assert wts[0]["reparse_points_unlinked"] == 0
    assert not Path(wts[0]["path"]).exists()
    listed = subprocess.run(["git", "-C", str(ROOT), "worktree", "list", "--porcelain"],
                            capture_output=True, text=True).stdout.replace("\\", "/")
    assert wts[0]["path"].replace("\\", "/") not in listed, listed
    assert not any(Path(_CACHE["scratch"]).iterdir()), list(Path(_CACHE["scratch"]).iterdir())


def test_a_junction_inside_is_unlinked_without_touching_its_target():
    top, target = _tmp("rar_top_"), _tmp("rar_target_")
    (target / "precious.txt").write_text("keep", encoding="utf-8")
    (top / "sub").mkdir()
    link = top / "sub" / "link"
    try:
        import _winapi
        _winapi.CreateJunction(str(target), str(link))
    except (ImportError, AttributeError):
        os.symlink(target, link, target_is_directory=True)
    gone = RA.unlink_reparse_points(top)
    assert [Path(g).name for g in gone] == ["link"], gone
    assert not link.exists() and (target / "precious.txt").read_text(encoding="utf-8") == "keep"
    shutil.rmtree(top)
    assert (target / "precious.txt").is_file()


def test_records_are_compared_as_json_text_with_named_fields_dropped():
    a = [{"ts": "x", "tick": 3, "v": {"vx": 1.0, "vy": 0.1}, "fsm_reason": "a"}]
    b = [{"ts": "y", "tick": 3, "v": {"vx": 1.0, "vy": 0.1}, "fsm_reason": "b"}]
    assert RA.compare_records(a, b, keys=("ts",), prefix="fsm_")["identical"]
    c = RA.compare_records(a, b, keys=("ts",))
    assert not c["identical"] and c["first_difference"]["field"] == "fsm_reason", c
    d = [{"ts": "x", "tick": 3, "v": {"vx": 1.0, "vy": math.nextafter(0.1, 1)},
          "fsm_reason": "a"}]
    c = RA.compare_records(a, d, keys=("ts",))
    assert c["fields_differing"]["v"]["records"] == 1 and \
        0 < c["fields_differing"]["v"]["max_abs"] < 1e-16, c
    c = RA.compare_records(a, a + a)
    assert c["first_difference"]["field"] == "<count>", c
    assert RA.compare_records([{"a": 1, "b": 2}], [{"b": 2, "a": 1}])["first_difference"][
        "field"] == "<key order>"


def test_the_cli_writes_the_summary_and_exits_by_verdict():
    out = _replayed()
    if out == SKIP:
        return SKIP
    root, dst = _CACHE["root"], _tmp("rar_cli_") / "rar.json"
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        rc = RA.main([str(root / "rail_ok"), str(root / "rail_dirty"), "--out", str(dst),
                      "--scratch", str(_tmp("rar_cli_wt_"))])
    assert rc == 0, buf.getvalue()
    s = json.loads(dst.read_text(encoding="utf-8"))
    assert s["verdicts"] == {"identical": 1, "differs": 0, "not_replayable": 0,
                             "refused": 1}, s["verdicts"]
    assert s["worktrees_left_behind"] == [] and s["per_run_file"] == "rar.per_run.jsonl.gz"
    per = [json.loads(x) for x in gzip.open(dst.with_name(s["per_run_file"]), "rt",
                                            encoding="utf-8")]
    assert sorted(p["verdict"] for p in per) == ["identical", "refused"], per
    with contextlib.redirect_stdout(io.StringIO()):
        assert RA.main([str(root / "rail_world_ulp"), "--scratch",
                        str(_tmp("rar_cli_wt_"))]) == 1
        assert RA.main([str(root / "rail_dirty")]) == 2
    # --python names the interpreter the drivers run under, and the summary
    # says which ran; --dump-dir keeps every difference of a differing episode
    import platform
    dump, dst2 = _tmp("rar_dump_"), _tmp("rar_cli2_") / "rar.json"
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        assert RA.main([str(root / "rail_world_ulp"), "--python", sys.executable,
                        "--dump-dir", str(dump), "--out", str(dst2), "--scratch",
                        str(_tmp("rar_cli_wt_"))]) == 1
        assert RA.main([str(root / "rail_ok"), "--python",
                        str(root / "no_such_python.exe")]) == 2
    s2 = json.loads(dst2.read_text(encoding="utf-8"))
    assert s2["replayed_on"]["python"] == platform.python_version(), s2["replayed_on"]
    assert "<python>" in s2["command"] and "<dump-dir>" in s2["command"], s2["command"]
    (f,) = list(dump.iterdir())
    d = json.loads(f.read_text(encoding="utf-8"))
    assert [p["tick"] for p in d["audit_records_differing"]] == [_CACHE["world_tick"]], d
    assert d["audit_records_differing"][0]["fields"] == ["emitted_action_world"], d


def test_a_run_named_relative_to_the_callers_directory_is_found():
    """The drivers run inside the revision's worktree: a run folder given
    relative to the caller's directory is resolved before they start, never
    read as 'no flight log' (rel060, 2026-11-19)."""
    out = _replayed()
    if out == SKIP:
        return SKIP
    root = _CACHE["root"]
    here = os.getcwd()
    try:
        os.chdir(root)
        dst = _tmp("rar_rel_") / "rar.json"
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            rc = RA.main(["rail_ok", "--out", str(dst), "--scratch", str(_tmp("rar_rel_wt_"))])
    finally:
        os.chdir(here)
    assert rc == 0, buf.getvalue()
    s = json.loads(dst.read_text(encoding="utf-8"))
    assert s["verdicts"]["identical"] == 1 and s["verdicts"]["not_replayable"] == 0, s


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
