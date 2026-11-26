"""tools/replay_q3.py: today's Shield re-run on a stored run's logged inputs (WP4-11).

    python tests/test_replay_q3.py

What is pinned here:

  * a run flown through the rail's Shield (guardrail.replay.rail_shield) and
    stored as the ROS 2 rail stores it - flight_log.jsonl, trajectory.json
    with the position's own doubles, policy_g<N>.json per generation, a
    hot-applied moving zone mid-flight, metrics.json's forecast - replays
    `identical`, bit for bit, with both generations, and had repairs to do;
  * the same for a Shield-off arm (raw and violations compared, emitted
    not), a scheduled rule on the clock trajectory.json logged (and a
    different clock reads `differs`), a moving zone translated and rotated
    mid-flight (re-anchored at each edit, as Shield._edit_zone does), a zone
    expired, and a Shield with its own escalation FSM;
  * the check can fail: one emitted float reads `differs` at that tick with
    the decisions still identical; one repair dropped reads `differs` on
    `repairs` with the decisions not identical;
  * a row that does not log touched / braked is counted per field, not
    passed silently;
  * what a run lacks makes it `not_replayable` with the reason, never a
    filled-in guess: no flight log, a headless episode, no forecast recorded,
    no policy IR, a generation a row names that is not stored, a map rule, a
    subject rule with a subject and no class, scheduled rules and no clock,
    a row whose Shield read the unlogged pipe (shield_velocity, fix_ok
    false), a generation today's hot_apply refuses (a fence after the first
    tick);
  * the flown revision is named clean or not ('-dirty', '-unknown',
    'unversioned' are not);
  * the window is the manifest's date, found under nested folders and not
    inside a run's bag/;
  * the CLI writes summary.json, per_run.jsonl.gz and table.md to --out, and
    exits 0 / 1 / 2 for identical / differs / nothing replayed.
"""
from __future__ import annotations

import contextlib
import gzip
import io
import json
import math
import os
import shutil
import sys
import tempfile
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import replay_q3 as Q                                              # noqa: E402
from guardrail.models import Action4D, DynamicNFZ, State, load_policy   # noqa: E402
from guardrail.replay import rail_shield                           # noqa: E402
from wgs84_fixture import geo, geo_ring              # noqa: E402

DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
CITY = ROOT / "policies" / "follow_car_citylife_nfz.yaml"
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}
_TMP: list[Path] = []


def _tmp(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


def _dump(p: Path, doc) -> None:
    p.write_text(json.dumps(doc), encoding="utf-8")


def _date(p: Path, when: str) -> None:
    ts = datetime.fromisoformat(when).timestamp()
    os.utime(p, (ts, ts))


def _fly(run: Path, *, ticks: int = 90, spawn_at: int = 25, when: str = "2026-10-20T12:00",
         metrics: bool = True, policy=None, arm: str = "on", clock_at=None,
         events: dict | None = None, escalation: bool = False,
         revision: str = "test") -> list[dict]:
    """A run as the ROS 2 rail stores one: the demo policy, a moving zone
    hot-applied at `spawn_at` across the aircraft's path, the aircraft
    integrating what the Shield emitted (arm "off": the raw action, as the
    control arm flies it; the Shield still judges it).

    policy     the policy flown (default the demo policy);
    clock_at   k -> datetime: the rule clock, pinned per tick and written to
               trajectory.json as the ROS 2 rail writes it;
    events     k -> fn(shield, x, y): more mid-flight events at tick k;
    escalation the Shield's own FSM (metrics.json says so), not the rail's."""
    run.mkdir(parents=True)
    pol = policy if policy is not None else load_policy(DEMO)
    pin = [None]
    if escalation:
        from guardrail.shield import Shield
        sh = Shield(pol, lookahead_s=5.0, dt=0.1,
                    now=(lambda: pin[0]) if clock_at else None)
    else:
        sh = rail_shield(pol, lookahead_s=5.0, dt=0.1,
                         clock=(lambda: pin[0]) if clock_at else None)
    (run / "policy_g0.json").write_bytes(sh.policy.canonical_bytes())
    events = dict(events or {})
    rows, traj = [], []
    x, y, up = 1 / 3, -2 / 7, 15.0 + 1 / 9
    for k in range(ticks):
        g_before = sh.policy.generation
        if k == spawn_at:
            zone = DynamicNFZ(id="bar", type="dynamic_nfz", **HARD_P0,
                              vertices=[{"x": x + 9, "y": y - 6}, {"x": x + 11, "y": y - 6},
                                        {"x": x + 11, "y": y + 6}, {"x": x + 9, "y": y + 6}],
                              altitude_floor_m=0, altitude_ceiling_m=100, margin_m=0.5,
                              motion={"vx_mps": -0.3, "vy_mps": 0.1})
            sh.hot_apply(geo(zone, sh))
        if k in events:
            events[k](sh, x, y)
        if sh.policy.generation != g_before:
            g = sh.policy.generation
            (run / f"policy_g{g}.json").write_bytes(sh.policy.canonical_bytes())
        if clock_at:
            pin[0] = clock_at(k)
        raw = Action4D(vx=4.5 + 0.01 * k, vy=0.3 * math.sin(k / 7.0),
                       vz_up=1.2 if k < 30 else -0.1, yaw_rate=0.0)
        d = sh.filter(State(x=x, y=y, up=up, yaw_deg=12.5), raw)
        flown = d.emitted if arm != "off" else d.raw
        rows.append({
            "t": round(k * 0.1, 3), "tick": k + 1,
            "x": round(x, 3), "y": round(y, 3), "up": round(up, 3), "yaw_deg": 12.5,
            "raw": d.raw.model_dump(), "emitted": flown.model_dump(),
            "violations": [v.model_dump() for v in d.violations],
            "repairs": [r.model_dump() for r in d.repairs] if arm != "off" else [],
            "touched": bool(d.touched) if arm != "off" else False,
            "braked": bool(d.braked) if arm != "off" else False,
            "generation": sh.policy.generation, "policy_hash": sh.policy.policy_hash,
        })
        p = {"t": round(k * 0.1, 2), "x": x, "y": y, "up": up}
        if clock_at:
            p["clock"] = clock_at(k).isoformat()
        traj.append(p)
        x, y, up = x + 0.1 * flown.vx, y + 0.1 * flown.vy, up + 0.1 * flown.vz_up
    with (run / "flight_log.jsonl").open("w", encoding="utf-8", newline="\n") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")
    _dump(run / "trajectory.json", traj)
    if metrics:
        la = {"horizon_s": 5.0, "dt_s": 0.1, "poses": 50}
        if escalation:
            la["escalation_fsm"] = True
        _dump(run / "metrics.json", {"shield": arm, "shield_lookahead": la})
    _dump(run / "manifest.json", {"code_revision": revision, "policy_hash": pol.policy_hash,
                                  "random_seed": 0, "topology": "dev"})
    _date(run / "manifest.json", when)
    return rows


def _rewrite_log(run: Path, rows: list[dict]) -> None:
    with (run / "flight_log.jsonl").open("w", encoding="utf-8", newline="\n") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")


# --------------------------------------------------------------------------- #
# the replay
# --------------------------------------------------------------------------- #

def test_a_stored_rail_run_replays_bit_for_bit_with_its_hot_applied_generation():
    run = _tmp("q3_") / "ros2_like_01"
    rows = _fly(run)
    assert sum(r["touched"] for r in rows) > 10, "the synthetic flight gave the Shield nothing"
    assert {r["generation"] for r in rows} == {0, 1}
    rec = Q.replay_run(run)
    assert rec["verdict"] == "identical", rec
    assert rec["first_difference"] is None and rec["decisions_identical"]
    assert rec["generations"] == [0, 1] and rec["ticks"] == len(rows)
    assert rec["emitted_max_abs_diff"] == 0.0
    assert rec["logged_touched_ticks"] == sum(r["touched"] for r in rows)
    assert rec["state_source"].startswith("trajectory.json")
    assert rec["shield"] == {"lookahead_s": 5.0, "dt": 0.1, "escalation": False,
                             "path_repair": False,
                             "clock": "none"}
    # the moving zone is anchored where the flight anchored it: the repairs
    # after the spawn are compared, and agree
    after = [r for r in rows if r["generation"] == 1 and r["repairs"]]
    assert after, "no repair after the hot-apply: the generation switch is untested"


def test_one_changed_value_reads_differs_with_its_tick_and_field():
    run = _tmp("q3_") / "ros2_like_02"
    rows = _fly(run)
    k = next(i for i, r in enumerate(rows) if r["repairs"])
    bad = json.loads(json.dumps(rows))
    bad[k]["emitted"]["vx"] = bad[k]["emitted"]["vx"] + 1e-12
    _rewrite_log(run, bad)
    rec = Q.replay_run(run)
    fd = rec["first_difference"]
    assert rec["verdict"] == "differs" and (fd["tick"], fd["field"]) == (rows[k]["tick"],
                                                                         "emitted"), fd
    assert rec["decisions_identical"] and rec["first_decision_difference"] is None
    assert 0 < rec["emitted_max_abs_diff"] < 1e-9
    bad = json.loads(json.dumps(rows))
    bad[k]["repairs"] = bad[k]["repairs"][1:]
    _rewrite_log(run, bad)
    rec = Q.replay_run(run)
    assert rec["verdict"] == "differs" and rec["first_difference"]["field"] == "repairs"
    assert not rec["decisions_identical"]
    assert rec["first_decision_difference"]["tick"] == rows[k]["tick"]


def test_the_rounded_log_is_used_only_when_trajectory_json_does_not_fit():
    run = _tmp("q3_") / "ros2_like_03"
    rows = _fly(run)
    traj = json.loads((run / "trajectory.json").read_text(encoding="utf-8"))
    traj[5]["x"] += 0.01                   # no longer the row's point
    _dump(run / "trajectory.json", traj)
    rec = Q.replay_run(run)
    assert rec["state_source"].startswith("flight_log.jsonl")
    assert any("rounded to 1 mm" in x for x in rec["limits"])


def test_what_a_run_lacks_makes_it_not_replayable_with_the_reason():
    root = _tmp("q3_nr_")
    # no flight log
    run = root / "no_log"
    run.mkdir()
    _dump(run / "manifest.json", {"topology": "dev"})
    rec = Q.replay_run(run)
    assert rec["verdict"] == "not_replayable" and rec["why"].startswith("no flight log")
    # a headless episode
    run = root / "headless"
    run.mkdir()
    _dump(run / "manifest.json", {"topology": "headless-kinematic"})
    (run / "flight_log.jsonl").write_text("{}\n", encoding="utf-8")
    rec = Q.replay_run(run)
    assert rec["why"].startswith("a headless-kinematic episode") and \
        "rerun_episode.py" in rec["why"]
    # the forecast not recorded
    run = root / "no_metrics"
    _fly(run, ticks=5, spawn_at=99, metrics=False)
    rec = Q.replay_run(run)
    assert rec["why"].startswith("the Shield's forecast is not recorded"), rec
    # no policy IR
    run = root / "no_policy"
    _fly(run, ticks=5, spawn_at=99)
    (run / "policy_g0.json").unlink()
    rec = Q.replay_run(run)
    assert rec["why"].startswith("no policy IR stored"), rec
    # a generation a row names that is not stored
    run = root / "missing_gen"
    rows = _fly(run, ticks=30, spawn_at=10)
    (run / "policy_g1.json").unlink()
    rec = Q.replay_run(run)
    assert "generation 1" in rec["why"] and "not stored" in rec["why"], rec
    # the reasons are counted under stable keys
    for why, key in (("the policy has obstacle_clearance rule(s), which read a map; x",
                      "a map rule, and the map flown is not recorded"),
                     ("the policy has scheduled rules (valid_time) and ...",
                      "the policy has scheduled rules and no clock per tick"),
                     ("tick 4 flew generation 2, whose policy is not stored",
                      "a generation's policy is not stored")):
        assert Q._reason_key(why) == key, why


def test_a_rule_that_reads_what_the_log_lacks_is_refused_not_guessed():
    city = load_policy(CITY, runtime=False)
    assert any(c.type == "obstacle_clearance" for c in city.constraints)
    inp = Q.Inputs(rows=[{"x": 0, "y": 0, "up": 10}], policies={0: city},
                   metrics={"params": {"citymap": "D:/x/citymap_citylife/occ_day.npz"}},
                   manifest={}, traj=None)
    try:
        Q.check_world(inp)
        raise AssertionError("a map rule was replayed without its map")
    except Q.NotReplayable as e:
        assert "occ_day.npz" in str(e) and "obstacle_clearance" in str(e)
    demo = load_policy(DEMO)
    standoff = city.model_copy(update={"constraints": [
        c for c in city.constraints if c.type == "subject_standoff"]})
    inp = Q.Inputs(rows=[{"x": 0, "y": 0, "up": 10, "tgt_x": 3.0}], policies={0: standoff},
                   metrics={}, manifest={}, traj=None)
    try:
        Q.check_world(inp)
        raise AssertionError("a subject rule was replayed without the subject's class")
    except Q.NotReplayable as e:
        assert "subject_standoff" in str(e)
    # with no subject on any row the rule never bound in flight either
    inp.rows = [{"x": 0, "y": 0, "up": 10}]
    Q.check_world(inp)
    # a velocity the Shield flew on that the log does not hold
    inp = Q.Inputs(rows=[{"x": 0, "y": 0, "up": 10}], policies={0: demo},
                   metrics={"shield_velocity_source": "ground truth"}, manifest={}, traj=None)
    try:
        Q.check_world(inp)
        raise AssertionError("a velocity-fed Shield was replayed without the velocity")
    except Q.NotReplayable as e:
        assert "measured velocity" in str(e)


def _scheduled_demo(tmp: Path):
    """The demo policy with its speed cap in force weekdays 07:30-17:30 only."""
    # As stored: points {lat, lon} only (WP1-14; a plain dump adds the local view).
    doc = load_policy(DEMO).model_dump(mode="json", exclude_none=True,
                                       context={"points": "stored"})
    for c in doc["constraints"]:
        if c["id"] == "kin-caps":
            c["valid_time"] = {"recurrence": {"days": ["Mon", "Tue", "Wed", "Thu", "Fri"],
                                              "start_time": "07:30", "end_time": "17:30"}}
    doc.pop("generation", None)
    p = tmp / "scheduled_demo.yaml"
    _dump(p, doc)
    pol = load_policy(p)
    assert any(c.valid_time is not None for c in pol.constraints)
    return pol


def test_the_shield_off_arm_compares_raw_and_violations_only():
    run = _tmp("q3_") / "ros2_like_off"
    rows = _fly(run, arm="off")
    assert sum(bool(r["violations"]) for r in rows) > 10, "the off arm judged nothing"
    rec = Q.replay_run(run)
    assert rec["verdict"] == "identical" and rec["arm"] == "off", rec
    assert rec["emitted_max_abs_diff"] is None
    # what the off arm logs as emitted (the raw it flew) is not compared...
    # ...but a violation it did not log is
    k = next(i for i, r in enumerate(rows) if r["violations"])
    bad = json.loads(json.dumps(rows))
    bad[k]["violations"] = []
    _rewrite_log(run, bad)
    rec = Q.replay_run(run)
    assert rec["verdict"] == "differs" and rec["first_difference"]["field"] == "violations"


def test_scheduled_rules_replay_on_the_logged_clock_and_are_refused_without_one():
    tmp = _tmp("q3_clock_")
    pol = _scheduled_demo(tmp)
    t0 = datetime(2026, 10, 19, 17, 29, 55)                  # a Monday; cap off at 17:30
    from datetime import timedelta
    run = tmp / "ros2_like_clock"
    rows = _fly(run, policy=pol, clock_at=lambda k: t0 + timedelta(seconds=0.1 * k))
    capped = [r for r in rows if any(v["rule_id"] == "kin-caps" for v in r["violations"])]
    assert capped and len(capped) < len(rows), "the schedule never switched the cap"
    rec = Q.replay_run(run)
    assert rec["verdict"] == "identical", rec
    assert rec["shield"]["clock"] == "pinned per tick"
    # the clock is read, not ignored: one hour later the cap is never in force
    traj = json.loads((run / "trajectory.json").read_text(encoding="utf-8"))
    for i, p in enumerate(traj):
        p["clock"] = (t0 + timedelta(hours=1, seconds=0.1 * i)).isoformat()
    _dump(run / "trajectory.json", traj)
    rec = Q.replay_run(run)
    assert rec["verdict"] == "differs" and not rec["decisions_identical"], rec
    # no clock logged: refused, never replayed with every rule in force
    for p in traj:
        p.pop("clock")
    _dump(run / "trajectory.json", traj)
    rec = Q.replay_run(run)
    assert rec["verdict"] == "not_replayable" and "valid_time" in rec["why"], rec
    assert Q._reason_key(rec["why"]) == "the policy has scheduled rules and no clock per tick"


def test_a_moving_zone_edited_mid_flight_is_re_anchored_where_the_flight_did():
    # A fast zone spawned far ahead, edited twice while it is still ahead:
    # Shield._edit_zone re-anchors it at each edit, so a replay that kept
    # the spawn's anchor would place it 1 m/s x the time since the spawn
    # further along, and the repairs against it would differ.
    def spawn(sh, x, y):
        sh.hot_apply(geo(DynamicNFZ(id="mover", type="dynamic_nfz", **HARD_P0,
                                vertices=[{"x": x + 30, "y": y - 8}, {"x": x + 33, "y": y - 8},
                                          {"x": x + 33, "y": y + 8}, {"x": x + 30, "y": y + 8}],
                                altitude_floor_m=0, altitude_ceiling_m=100, margin_m=0.5,
                                motion={"vx_mps": -1.0, "vy_mps": 0.0}), sh))
    run = _tmp("q3_") / "ros2_like_edit"
    rows = _fly(run, ticks=110, spawn_at=999, events={
        5: spawn,
        25: lambda sh, x, y: sh.translate_nfz("mover", 0.0, 0.5),
        40: lambda sh, x, y: sh.rotate_nfz("mover", 10.0)})
    assert {r["generation"] for r in rows} == {0, 1, 2, 3}
    hit = [r for r in rows if r["generation"] == 3
           and any(v["rule_id"] == "mover" for v in r["violations"])]
    assert len(hit) > 5, "the edited zone was never repaired against: the re-anchor is untested"
    rec = Q.replay_run(run)
    assert rec["verdict"] == "identical", rec
    assert rec["generations"] == [0, 1, 2, 3]


def test_an_expired_zone_leaves_no_anchor_and_replays():
    run = _tmp("q3_") / "ros2_like_expire"
    _fly(run, ticks=80, events={55: lambda sh, x, y: sh.expire_nfz("bar")})
    rec = Q.replay_run(run)
    assert rec["verdict"] == "identical", rec


def test_a_generation_todays_hot_apply_refuses_is_not_replayable():
    run = _tmp("q3_") / "ros2_like_late_fence"

    def late_fence(sh, x, y):
        # What a rail before the mission-start lock did: a polygon fence
        # committed after the first tick. Today's hot_apply refuses it.
        fence = next(c for c in sh.policy.constraints if c.id == "nfz-square")
        late = fence.model_copy(update={"id": "nfz-late"})
        try:
            sh.hot_apply(geo(late, sh))
            raise AssertionError("today's hot_apply took a fence after the first tick")
        except Exception as e:                                   # noqa: BLE001
            assert type(e).__name__ == "LockedRuleClass", e
        sh._commit(list(sh.policy.constraints) + [late], "add", late.id,
                   late.type, None)
    _fly(run, ticks=60, spawn_at=999, events={30: late_fence})
    rec = Q.replay_run(run)
    assert rec["verdict"] == "not_replayable", rec
    assert "adds polygon_fence 'nfz-late' after the first tick" in rec["why"], rec
    assert Q._reason_key(rec["why"]) == \
        "a generation today's Shield.hot_apply refuses where the run installed it"


def test_the_escalation_fsm_shield_is_rebuilt_when_the_run_recorded_it():
    run = _tmp("q3_") / "ros2_like_fsm"
    _fly(run, escalation=True)
    rec = Q.replay_run(run)
    assert rec["verdict"] == "identical" and rec["shield"]["escalation"] is True, rec


def test_a_field_the_row_does_not_log_is_counted_not_skipped_silently():
    run = _tmp("q3_") / "ros2_like_nolog"
    rows = _fly(run)
    for r in rows[:20]:
        r.pop("touched")
    for r in rows[:7]:
        r.pop("braked")
    _rewrite_log(run, rows)
    rec = Q.replay_run(run)
    assert rec["verdict"] == "identical", rec
    assert rec["ticks_field_not_logged"] == {"touched": 20, "braked": 7}, rec
    assert any("not compared" in x and "touched on 20" in x for x in rec["limits"])


def test_a_run_whose_shield_read_the_pipe_is_refused():
    demo = load_policy(DEMO)
    for row, word in (({"x": 0, "y": 0, "up": 10, "shield_velocity": True}, "own-velocity"),
                      ({"x": 0, "y": 0, "up": 10, "fix_ok": False}, "no valid fix")):
        inp = Q.Inputs(rows=[{"x": 0, "y": 0, "up": 10}, row], policies={0: demo},
                       metrics={}, manifest={}, traj=None)
        try:
            Q.check_world(inp)
            raise AssertionError(f"a run with {row} was replayed with an empty pipe")
        except Q.NotReplayable as e:
            assert word in str(e) and Q._reason_key(str(e)) == \
                "the pipe the Shield read is not logged per tick", e
    # fix_ok True / absent is no pipe read
    Q.check_world(Q.Inputs(rows=[{"x": 0, "y": 0, "up": 10, "fix_ok": True}],
                           policies={0: demo}, metrics={}, manifest={}, traj=None))


def test_the_flown_revision_is_named_clean_or_not():
    assert Q.revision_clean("e5b2e51492ec")
    for rev in ("e5b2e51492ec-dirty", "401305abd019-unknown", "unversioned", None, ""):
        assert not Q.revision_clean(rev), rev
    run = _tmp("q3_") / "ros2_like_dirty"
    _fly(run, ticks=20, spawn_at=99, revision="e5b2e51492ec-dirty")
    rec = Q.replay_run(run)
    assert rec["verdict"] == "identical" and rec["flown_revision_clean"] is False
    assert any("not a clean commit" in x for x in rec["limits"])
    now = Q._code_revision_now()
    assert now is None or now == "unversioned" or len(now.split("-")[0]) == 12, now


# --------------------------------------------------------------------------- #
# the window and the CLI
# --------------------------------------------------------------------------- #

def test_the_window_is_the_manifests_date_and_a_bag_folder_is_not_a_run():
    root = _tmp("q3_win_")
    for rel, when in (("a_sep", "2026-09-15T10:00"), ("b_nov", "2026-11-10T10:00"),
                      ("group/c_aug", "2026-08-02T08:00"), ("d_jul", "2026-07-31T23:00"),
                      ("a_sep/bag", "2026-09-15T10:00")):
        d = root / rel
        d.mkdir(parents=True, exist_ok=True)
        _dump(d / "manifest.json", {})
        _date(d / "manifest.json", when)
    got = [p.relative_to(root).as_posix() for p in Q.find_runs(root, Q.Q3_FROM, Q.Q3_TO)]
    assert got == ["a_sep", "group/c_aug"], got


def _main(*argv) -> tuple[int, str]:
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = Q.main([str(a) for a in argv])
    return rc, buf.getvalue()


def test_the_cli_writes_its_table_and_exits_by_verdict():
    root = _tmp("q3_cli_")
    _fly(root / "run_ok")
    out = root / "_out"
    rc, text = _main("--root", root, "--out", out)
    assert rc == 0 and "run_ok" in text, text
    s = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert s["verdicts"] == {"identical": 1, "differs": 0, "not_replayable": 0}, s
    assert s["window"]["by"].startswith("manifest.json") and str(out) not in s["command"]
    with gzip.open(out / "per_run.jsonl.gz", "rt", encoding="utf-8") as fh:
        recs = [json.loads(line) for line in fh]
    assert [r["run"] for r in recs] == ["run_ok"]
    assert "| run_ok |" in (out / "table.md").read_text(encoding="utf-8")
    # nothing written into the run folder
    assert sorted(p.name for p in (root / "run_ok").iterdir()) == [
        "flight_log.jsonl", "manifest.json", "metrics.json", "policy_g0.json",
        "policy_g1.json", "trajectory.json"]
    # a run that differs: exit 1
    run = root / "run_bad"
    rows = _fly(run)
    rows[7]["violations"] = []
    _rewrite_log(run, rows)
    _date(run / "manifest.json", "2026-10-20T12:00")
    rc, text = _main("--root", root, "--out", out)
    assert rc == 1 and "differs" in text, text
    # nothing replayable in the window: exit 2
    rc, _ = _main("--root", root, "--since", "2026-01-01", "--until", "2026-01-31")
    assert rc == 2


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
    for d in _TMP:
        shutil.rmtree(d, ignore_errors=True)
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
