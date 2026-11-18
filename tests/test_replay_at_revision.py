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
    frame reads `differs` (class `decision`) at that tick and field; the same
    change in the body-frame action alone reads `differs` of class
    `body_frame_rotation_last_digits`; a changed headless audit record and a
    changed KPI figure read `differs`; a fields the replay does not
    reproduce by design (`ts`, the node's fsm_*) is not compared;
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
  * the CLI writes the summary and the per-episode records, and exits 0 / 1
    / 2.

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
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}
_TMP: list[Path] = []
_CACHE: dict = {}


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

def _rail_run(run: Path, rev: str, *, ticks: int = 80, spawn_at: int = 22) -> None:
    """A run as the ROS 2 rail stores one, its Shield the rail's (no FSM
    inside), its audit written by AuditLogger as the rail writes it."""
    from guardrail.audit import AuditLogger
    from guardrail.models import Action4D, DynamicNFZ, State, load_policy
    from guardrail.replay import rail_shield
    run.mkdir(parents=True)
    pol = load_policy(DEMO)
    sh = rail_shield(pol, lookahead_s=5.0, dt=0.1)
    (run / "policy_g0.json").write_bytes(sh.policy.canonical_bytes())
    logger = AuditLogger.in_memory(sh.policy, episode_id="1234")
    rows, traj, audit = [], [], []
    x, y, up, yaw = 1 / 3, -2 / 7, 15.0 + 1 / 9, 12.5 + 1 / 7
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
        raw = Action4D(vx=4.5 + 0.01 * k, vy=0.3 * math.sin(k / 7.0),
                       vz_up=1.2 if k < 30 else -0.1, yaw_rate=0.0)
        d = sh.filter(State(x=x, y=y, up=up, yaw_deg=yaw), raw)
        rec = logger.record(k + 1, d, fsm_record={"fsm_state_before": "Normal",
                                                   "fsm_state_after": "Normal",
                                                   "reason": "rail FSM", "transition": False})
        if rec is not None:
            line = AuditLogger.line(rec)
            line["episode_id"] = "1234"
            audit.append(line)
        rows.append({"t": round(k * 0.1, 3), "tick": k + 1, "x": round(x, 3),
                     "y": round(y, 3), "up": round(up, 3), "yaw_deg": round(yaw, 2),
                     "raw": d.raw.model_dump(), "emitted": d.emitted.model_dump(),
                     "violations": [v.model_dump() for v in d.violations],
                     "repairs": [r.model_dump() for r in d.repairs],
                     "touched": bool(d.touched), "braked": bool(d.braked),
                     "generation": sh.policy.generation, "policy_hash": sh.policy.policy_hash})
        traj.append({"t": round(k * 0.1, 2), "x": x, "y": y, "up": up})
        x, y, up = x + 0.1 * d.emitted.vx, y + 0.1 * d.emitted.vy, up + 0.1 * d.emitted.vz_up
    with (run / "flight_log.jsonl").open("w", encoding="utf-8", newline="\n") as fh:
        fh.writelines(json.dumps(r) + "\n" for r in rows)
    with (run / "shield_audit.jsonl").open("w", encoding="utf-8", newline="\n") as fh:
        fh.writelines(json.dumps(a) + "\n" for a in audit)
    (run / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")
    (run / "metrics.json").write_text(json.dumps({
        "shield": "on", "shield_lookahead": {"horizon_s": 5.0, "dt_s": 0.1, "poses": 50},
        "shield_forecast": {"mode": "commanded", "velocity_ticks": 0}}), encoding="utf-8")
    (run / "manifest.json").write_text(json.dumps({
        "code_revision": rev, "policy_hash": pol.policy_hash, "random_seed": 0,
        "sim_speedup": 1.0, "topology": "dev"}), encoding="utf-8")
    assert len(audit) > 10 and {r["generation"] for r in rows} == {0, 1}, \
        "the synthetic flight gave the Shield nothing to audit"


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
    assert w["verdict"] == "differs" and w["difference_class"] == "decision", w
    assert (fd["where"], fd["tick"], fd["field"]) == (
        "shield_audit.jsonl", _CACHE["world_tick"], "emitted_action_world"), fd
    b = out["rail_body_ulp"]
    assert b["verdict"] == "differs", b
    assert b["difference_class"] == "body_frame_rotation_last_digits", b
    assert set(b["audit"]["fields_differing"]) == {"raw_action"}, b["audit"]
    # the wall time and the node's FSM verdict are not the Shield's to replay
    assert out["rail_ts_fsm"]["verdict"] == "identical", out["rail_ts_fsm"]


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
