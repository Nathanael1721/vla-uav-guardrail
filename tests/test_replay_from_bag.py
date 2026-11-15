"""tools/replay_from_bag.py: an episode rebuilt from its rosbag2 alone (ARCH-22).

    python tests/test_replay_from_bag.py

What is pinned here:

  * an episode as the ROS 2 Shield rail records it - a pass, a repair, a stop
    by the FSM, the hand-over to a hold, the hold's own leg, the autopilot's
    ticks with and without an intercept - is rebuilt from the topics tick for
    tick and reads `identical` against the log, trajectory.json and the KPI
    table `guardrail.kpi.compute` wrote;
  * the check can fail: one float of one action, one pose, one KPI field, a
    dropped row, a tick the timer ran, each read `differs` with the layer,
    the field and the tick;
  * a KPI the topics cannot carry is never copied from kpi.json: editing the
    stored escape count changes nothing in the verdict, and every key
    `compute()` returns is classified (from the bag, from the bag plus the
    rule priorities, not from the bag with a reason, or not a KPI);
  * the limits are said, not hidden: the ticks after the Shield's last
    message, a Shield-off arm, a tick the FSM stopped, a pose the node took
    in the other order, the node running behind the recorder at the start;
  * the topic names are the ones the two record commands list;
  * the CLI writes its records to --out and nothing into a run folder, and
    its exit codes; a campaign's runs are its sweep.json's accepted ones;
  * a replay bundle's bag is unpacked from the verified archive.

The bag is a stand-in for `read_streams` in most tests, so they run without
`rosbags`. The round trip through a real rosbag2 written and read with
`rosbags` SKIPs where it is not installed (no CI job installs it; the lab's
vla-real environment has rosbags 0.10.11).
"""
from __future__ import annotations

import contextlib
import io
import json
import math
import re
import shutil
import struct
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests"))

import replay_from_bag as R                                        # noqa: E402
from guardrail import kpi as K                                     # noqa: E402
from guardrail.frames import from_body                             # noqa: E402

SKIP = "SKIP"
_TMP: list[Path] = []
T0 = 1_794_500_000_000_000_000          # the mission clock starts (ns)
TICK = 100_000_000
HOME = [-200.0, 5.0]
YAW = 30.0
PRIORITIES = {"nfz-a": "P0", "kin-caps": "P1"}
ZERO = [0.0, 0.0, 0.0, 0.0]
# One mission, every kind of tick the rail writes (the rebuild's cases).
PLAN = (["pass"] * 4 + ["repair"] * 3 + ["soft"] + ["pass"] * 2 + ["fsm_stop", "handover"]
        + ["hold_leg"] * 4 + ["auto_touched"] * 3 + ["auto_clean"] * 5)


def _tmp(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


def _f32(v: float) -> float:
    return struct.unpack("<f", struct.pack("<f", float(v)))[0]


def _vec(*v: float) -> list:
    return [_f32(x) for x in v]


def _episode(plan=PLAN, *, pre: int = 3, post: int = 4, shield: str = "on",
             late_start_ns: int = 0):
    """(the bag's topics as read_streams returns them, the flight log's rows,
    trajectory.json, kpi.json) of one mission flown to `plan`: one action a
    tick, the pose 30 ms before it, the setpoint 4 ms after it, the executed
    setpoint 5 ms, the intercept 8 ms; RTL from the first autopilot tick.
    `late_start_ns`: the node ticks that much after the recorder logged the
    first two actions (their Shield messages come that much later)."""
    s = R.BagStreams(reader="stand-in")
    rows, traj = [], []
    s.fences.append((T0 - 2 * TICK, {"generation": 0, "home_policy_ne": HOME}))
    s.states.append((T0 - 5 * TICK, {"mode": "GUIDED", "armed": True, "connected": True}))
    s.markers.append((T0 + 1_000_000, ("abcd1234", 0.0)))
    auto_from = next((i for i, k in enumerate(plan) if k.startswith("auto")), None)
    for i in range(-pre, len(plan) + post):
        a_ns = T0 + i * TICK
        raw = _vec(4.0, 0.01 * i, 0.001 * i, 0.0)
        north, east, up = 0.4 * i + 1 / 3, 0.1 * i, 15.0 + 0.001 * i
        s.actions.append((a_ns, raw))
        s.poses.append((a_ns - 30_000_000, {"north": north, "east": east, "up": up,
                                            "yaw_deg": YAW}))
        if not 0 <= i < len(plan):
            continue
        kind = plan[i]
        late = late_start_ns if i < 2 else 0
        on = shield == "on"
        vios, ops, fsm, pub, sp, em_w = [], [], "Normal", raw, "pass", None
        if kind == "repair":
            vios, ops, fsm, pub = ["nfz-a"], ["GeofenceSlide"], "Brake", _vec(1.0, 2.0, 0.0, 0.0)
        elif kind == "soft":
            vios, ops, fsm, pub = ["kin-caps"], ["SpeedClamp"], "Brake", _vec(3.0, 0.0, 0.0, 0.0)
        elif kind == "fsm_stop":
            vios, ops, fsm, pub, sp = ["nfz-a"], ["GeofenceSlide"], "Brake", ZERO, "brake"
            em_w = from_body(0.5, 3.0, 0.0, 0.0, YAW).model_dump()   # what the Shield repaired
        elif kind == "handover":
            vios, ops, fsm, pub, sp = ["nfz-a"], ["GeofenceSlide"], "Loiter", ZERO, "brake"
            em_w = from_body(0.5, 3.0, 0.0, 0.0, YAW).model_dump()
        elif kind == "hold_leg":
            vios, ops, fsm, pub, sp = ["nfz-a"], ["GeofenceSlide"], "Loiter", \
                _vec(0.2, -0.1, 0.0, 0.0), "none"
        elif kind == "auto_touched":
            vios, ops, fsm, pub, sp = ["nfz-a"], ["GeofenceSlide"], "RTL", None, "none"
        elif kind == "auto_clean":
            fsm, pub, sp = "RTL", None, "none"
        if not on:
            # The control arm: the raw action flies, no FSM, no intercept.
            ops, fsm = [], None
            pub, sp = (raw, "pass") if not kind.startswith("auto") else (None, "none")
        touched = bool(on and ops)
        if auto_from is not None and i == auto_from:
            s.states.append((a_ns - 20_000_000, {"mode": "RTL", "armed": True,
                                                 "connected": True}))
        mode = "RTL" if auto_from is not None and i >= auto_from else "GUIDED"
        t = round(i * TICK / 1e9 + 0.003 + late / 1e9, 3)
        if pub is not None:
            s.setpoints.append((a_ns + 4_000_000 + late, pub))
            w = from_body(*pub, YAW)
            s.executed.append((a_ns + 5_000_000 + late, {
                "coordinate_frame": 1, "type_mask": 1479,
                "velocity": [w.vy, w.vx, w.vz_up], "yaw_rate": -w.yaw_rate}))
        if touched:
            s.intercepts.append((a_ns + 8_000_000 + late, {
                "t": t, "violations": vios, "repairs": ops, "fsm": fsm}))
        row = {
            "t": t, "tick": i + 1,
            "x": round(north + HOME[0], 3), "y": round(east + HOME[1], 3), "up": round(up, 3),
            "yaw_deg": YAW, "frame": "body",
            "raw_body": dict(zip(R.KEYS, raw)),
            "emitted_body": None if pub is None else dict(zip(R.KEYS, pub)),
            "raw": from_body(*raw, YAW).model_dump(),
            "emitted": em_w or from_body(*(pub or raw), YAW).model_dump(),
            "violations": [{"rule_id": v, "category": "geofence", "detail": "x"} for v in vios],
            "flown": sp != "none", "emitted_violations": [],
            "repairs": [{"operator": o, "detail": "x", "magnitude_m": 1.0, "axis": "lateral"}
                        for o in ops],
            "braked": False, "touched": touched, "setpoint": sp, "mode": mode,
            "unsafe": False, "unsafe_rules": [], "action_seq": pre + i + 1,
            "tick_trigger": "action",
        }
        if on:
            row["fsm_state_after"] = fsm
        else:
            row["fsm_state_after"] = None
        rows.append(row)
        traj.append({"t": round(t, 2), "x": north + HOME[0], "y": east + HOME[1], "up": up})
    kpi = dict(K.compute(rows, PRIORITIES, {"shield": shield, "reached": False}),
               episode_id=T0)
    return s, rows, traj, kpi


@contextlib.contextmanager
def _bag_is(streams):
    """R.read_streams replaced by one returning `streams` (no rosbags)."""
    real, seen = R.read_streams, []

    def fake(bag_dir):
        seen.append(sorted(p.name for p in Path(bag_dir).iterdir())
                    if Path(bag_dir).is_dir() else None)
        return streams
    R.read_streams = fake
    try:
        yield seen
    finally:
        R.read_streams = real


def _replay(s, rows, traj, kpi, *, priorities=PRIORITIES, arm="on"):
    ep = R.Episode(name="synthetic", source="memory", bag_dir=Path("stand-in"), rows=rows,
                   trajectory=traj, kpi=kpi, priorities=priorities, arm=arm)
    with _bag_is(s):
        rec, ticks = R.replay(ep)
    return rec, ticks


def _field(rec: dict, name: str) -> dict:
    return rec["kpi"]["fields"][name]


# --------------------------------------------------------------------------- #
# the rebuild
# --------------------------------------------------------------------------- #

def test_an_episode_is_rebuilt_from_the_topics_and_reads_identical():
    s, rows, traj, kpi = _episode()
    rec, ticks = _replay(s, rows, traj, kpi)
    assert rec["verdict"] == "identical" and rec["first_difference"] is None, rec
    c = rec["compared_with_the_log"]
    assert c["ticks"]["verdict"] == "identical" and c["trajectory"]["verdict"] == "identical"
    assert c["log_rows"] == c["compared"] == len(PLAN) and rec["kpi"]["verdict"] == "identical"
    # who flew each tick, inferred from the topics, is the log's verdict
    assert [t["setpoint"] for t in ticks] == [r["setpoint"] for r in rows]
    assert rec["rebuilt"]["setpoint"] == {"pass": 10, "brake": 2, "none": 12}
    # the trajectory against trajectory.json, to the message's own doubles
    tr = c["trajectory"]
    assert tr["points"] == tr["exact"] == len(PLAN) and "trajectory.json" in tr["against"]
    assert all(abs(t["x"] - p["x"]) < 1e-12 for t, p in zip(ticks, traj))
    # the first tick is the action the Shield's first message answers, not
    # the first action in the bag; the home offset comes from /shield/fences
    w = rec["window"]
    assert w["first_tick"]["action_index"] == 4 and w["actions_before_the_first_tick"] == 3
    assert w["home_policy_ne"] == HOME and "/shield/fences" in w["home_source"]
    assert "/shield/intercept" in w["t0_source"]
    # executed: what MAVROS was sent, in the world frame, on a streamed tick
    assert ticks[4]["executed"] is not None and ticks[-1]["executed"] is None
    want = from_body(1.0, 2.0, 0.0, 0.0, YAW)
    assert abs(ticks[4]["executed"]["vx"] - want.vx) < 1e-6
    # the rotation is guardrail.frames.from_body's
    got = R.world_from_body([1.0, 2.0, 0.5, 0.1], 73.0)
    assert all(abs(got[k] - v) < 1e-12 for k, v in from_body(1.0, 2.0, 0.5, 0.1, 73.0)
               .model_dump().items())


def test_the_kpi_fields_the_topics_carry_equal_kpi_json_and_say_their_source():
    s, rows, traj, kpi = _episode(["pass"] * 3 + ["repair"] * 4 + ["soft"] * 2 + ["pass"] * 3)
    rec, _ = _replay(s, rows, traj, kpi)
    assert rec["verdict"] == "identical", rec["first_difference"]
    for name in R.FROM_BAG:
        f = _field(rec, name)
        assert f["equal"] is True and f["bag"] == kpi[name], (name, f)
    assert _field(rec, "repair_count") == {"source": "bag", "bag": 6, "kpi_json": 6,
                                           "equal": True}
    assert _field(rec, "p0_violation_ticks")["source"] == R.BAG_POLICY
    assert _field(rec, "p0_violation_ticks")["bag"] == 4
    assert _field(rec, "failsafe_permitted_ticks")["bag"] == 2
    assert _field(rec, "violations_by_risk_level")["bag"] == {"P0": 4, "P1": 2}
    # the magnitude is measured on the body-frame pair, which is the world's
    assert _field(rec, "mean_repair_magnitude_mps")["bag"] == kpi["mean_repair_magnitude_mps"] > 0
    assert rec["kpi"]["equal"] == len(R.FROM_BAG) and rec["kpi"]["differs"] == 0
    # without the policy generations a rule's priority is on no topic
    rec, _ = _replay(s, rows, traj, kpi, priorities=None)
    f = _field(rec, "p0_violation_ticks")
    assert "policy_g<N>.json" in f["not_on_this_run"] and "bag" not in f
    assert _field(rec, "repair_count")["equal"] is True and rec["verdict"] == "identical"


def test_a_kpi_the_topics_cannot_carry_is_reported_as_such_never_copied():
    s, rows, traj, kpi = _episode()
    forged = dict(kpi, p0_escapes=5, p0_violation_escape_rate=0.5, outcome="success",
                  mean_time_to_safe_s=1.0, repair_success_rate=1.0)
    rec, _ = _replay(s, rows, traj, forged)
    # nothing here is re-derived, so the forgery is neither caught nor
    # vouched for: the verdict covers the fields from the bag only
    assert rec["verdict"] == "identical"
    for name in ("p0_escapes", "p0_violation_escape_rate", "outcome", "mean_time_to_safe_s",
                 "repair_success_rate", "failsafe_trigger_correctness", "mission_success"):
        f = _field(rec, name)
        assert f["source"] == "not from the bag" and "bag" not in f and "equal" not in f, f
        assert len(f["why"].split()) >= 6, (name, f["why"])
    assert rec["kpi"]["not_from_the_bag"] >= 30
    # every key compute() returns is classified, so a new KPI cannot be left
    # out of the account in silence; and so is every field verify_replay checks
    from guardrail.replay import VERIFIED_KPI_FIELDS
    known = set(R.FROM_BAG) | set(R.NOT_FROM_BAG) | set(R.NOT_A_KPI)
    assert not set(K.compute(rows, PRIORITIES, {})) - known, \
        sorted(set(K.compute(rows, PRIORITIES, {})) - known)
    assert not set(VERIFIED_KPI_FIELDS) - known
    assert not set(R.FROM_BAG) & set(R.NOT_FROM_BAG)
    # the headline figures are among those the bag alone cannot give
    for name in ("p0_violation_escape_rate", "p0_escapes", "failsafe_trigger_correctness",
                 "mean_time_to_safe_s", "max_time_to_safe_s"):
        assert name in R.NOT_FROM_BAG, name


# --------------------------------------------------------------------------- #
# the check can fail
# --------------------------------------------------------------------------- #

def test_one_changed_value_reads_differs_with_the_layer_field_and_tick():
    s, rows, traj, kpi = _episode()

    def edited(fn):
        r2, t2, k2 = json.loads(json.dumps(rows)), json.loads(json.dumps(traj)), dict(kpi)
        fn(r2, t2, k2)
        return _replay(s, r2, t2, k2)[0]

    def first(rec):
        f = rec["first_difference"]
        return rec["verdict"], f["layer"], f["field"], f["tick"]

    # one float32 step of one action
    step = struct.unpack("<f", struct.pack("<I", struct.unpack("<I", struct.pack(
        "<f", rows[6]["raw_body"]["vx"]))[0] + 1))[0]
    rec = edited(lambda r, t, k: r[6]["raw_body"].__setitem__("vx", step))
    assert first(rec) == ("differs", "ticks", "raw_body", 7), rec["first_difference"]
    assert rec["compared_with_the_log"]["ticks"]["differences"] == {"raw_body": 1}
    rec = edited(lambda r, t, k: r[5]["emitted_body"].__setitem__("vy", 2.5))
    assert first(rec) == ("differs", "ticks", "emitted_body", 6)
    rec = edited(lambda r, t, k: r[12].__setitem__("setpoint", "pass"))
    assert first(rec)[:3] == ("differs", "ticks", "setpoint") and first(rec)[3] == 13
    rec = edited(lambda r, t, k: r[2].__setitem__("mode", "LAND"))
    assert first(rec) == ("differs", "ticks", "mode", 3)
    rec = edited(lambda r, t, k: r[5]["violations"][0].__setitem__("rule_id", "other"))
    assert first(rec) == ("differs", "ticks", "violations", 6)
    rec = edited(lambda r, t, k: r[5]["repairs"][0].__setitem__("operator", "Brake"))
    assert first(rec) == ("differs", "ticks", "repairs", 6)
    rec = edited(lambda r, t, k: r[5].__setitem__("fsm_state_after", "Normal"))
    assert first(rec) == ("differs", "ticks", "fsm_state_after", 6)
    # a point that is no pose message of the bag (1 mm off)
    rec = edited(lambda r, t, k: t[9].__setitem__("x", t[9]["x"] + 0.001))
    assert first(rec) == ("differs", "trajectory", "pose", 10)
    assert rec["compared_with_the_log"]["trajectory"]["differs"] == 1
    assert rec["compared_with_the_log"]["ticks"]["verdict"] == "identical"
    # a KPI field the bag carries
    rec = edited(lambda r, t, k: k.__setitem__("repair_count", k["repair_count"] + 1))
    f = rec["first_difference"]
    assert (rec["verdict"], f["layer"], f["field"]) == ("differs", "kpi", "repair_count")
    assert f["bag"] == kpi["repair_count"] and f["log"] == kpi["repair_count"] + 1
    rec = edited(lambda r, t, k: k.__setitem__("shield_ticks", 3))
    assert rec["first_difference"]["field"] == "shield_ticks"
    # a row dropped from the middle: the bag has an action no row ticked on
    rec = edited(lambda r, t, k: (r.pop(8), t.pop(8)))
    assert rec["verdict"] == "differs"
    assert rec["compared_with_the_log"]["bag_actions_no_row_ticked_on"] == 1
    # the log ends before the Shield's last message
    rec = edited(lambda r, t, k: (r.__delitem__(slice(14, None)), t.__delitem__(slice(14, None))))
    assert rec["verdict"] == "differs" and "ticks" in rec["compared_with_the_log"]["ticks"][
        "differences"]


def test_a_tick_the_timer_ran_has_no_message_and_is_said_not_rebuilt():
    s, rows, traj, kpi = _episode()
    extra = dict(rows[3], tick=99, t=rows[3]["t"] + 0.05, tick_trigger="timer")
    rows2 = rows[:4] + [extra] + rows[4:]
    rec, _ = _replay(s, rows2, None, kpi)
    f = rec["first_difference"]
    assert rec["verdict"] == "differs" and (f["layer"], f["field"], f["tick"]) == (
        "ticks", "tick", 99), f
    assert "'timer'" in f["log"] and "/vla/action_4d" in f["log"]
    assert rec["compared_with_the_log"]["rows_the_bag_marks_no_tick_for"] == 1


# --------------------------------------------------------------------------- #
# the limits, said
# --------------------------------------------------------------------------- #

def test_ticks_after_the_shields_last_message_are_counted_from_the_log_and_said():
    s, rows, traj, kpi = _episode()
    rec, ticks = _replay(s, rows, traj, kpi)
    c = rec["compared_with_the_log"]
    # the five clean autopilot ticks left no message of the Shield's
    assert c["bag_ticks_evidenced"] == len(PLAN) - 5 and c["ticks_past_bag_end"] == 5
    assert rec["rebuilt"]["evidenced_by_the_bag"] == len(PLAN) - 5
    assert rec["rebuilt"]["ticks"] == len(ticks) == len(PLAN)
    for name in ("ticks", "autopilot_ticks"):
        assert "flight_log.jsonl" in _field(rec, name)["source"], _field(rec, name)
    assert _field(rec, "shield_ticks")["source"] == "bag"
    assert any("5 tick(s) after the Shield's last message" in x for x in rec["limits"])
    # their actions and poses are still the bag's: a changed one differs
    rows2 = json.loads(json.dumps(rows))
    rows2[-1]["raw_body"]["vy"] = 9.0
    assert _replay(s, rows2, traj, kpi)[0]["first_difference"]["tick"] == len(PLAN)
    # without a log the bag's own window is all there is
    ep = R.Episode(name="x", source="memory", bag_dir=Path("stand-in"))
    with _bag_is(s):
        rec, ticks = R.replay(ep)
    assert len(ticks) == len(PLAN) - 5 and rec["compared_with_the_log"] is None
    assert rec["kpi"]["verdict"] == "no_kpi_json" and "no flight log" in rec["verdict_covers"]
    assert _field(rec, "ticks") == {"source": "bag", "bag": len(PLAN) - 5, "kpi_json": "absent"}


def test_a_tick_the_fsm_stopped_carries_no_repaired_action():
    s, rows, traj, kpi = _episode()
    rec, ticks = _replay(s, rows, traj, kpi)
    # ticks 11 and 12: the stop was published; the log's `emitted` is the
    # action the Shield repaired, which no topic carries
    assert [ticks[i]["setpoint"] for i in (10, 11)] == ["brake", "brake"]
    assert "emitted" not in ticks[10] and rows[10]["emitted"]["vy"] != 0.0
    for name in ("repaired_ticks", "mean_repair_magnitude_mps", "max_repair_magnitude_mps"):
        f = _field(rec, name)
        assert "2 tick(s) the FSM stopped" in f["not_on_this_run"] and "bag" not in f, f
        assert f["kpi_json"] == kpi[name]
    assert _field(rec, "repair_count")["equal"] is True
    assert rec["kpi"]["not_on_this_run"] == 4 and rec["verdict"] == "identical"
    # the Shield's own stop (its Brake operator) IS the repaired action
    s, rows, traj, kpi = _episode(["pass"] * 3 + ["own_brake"] + ["auto_touched"] * 2)
    for r in rows:
        if r["tick"] == 4:
            r.update(repairs=[{"operator": "Brake", "detail": "stop", "magnitude_m": None,
                               "axis": None}], braked=True, setpoint="brake",
                     emitted=dict(zip(R.KEYS, ZERO)), emitted_body=dict(zip(R.KEYS, ZERO)),
                     violations=[{"rule_id": "nfz-a", "category": "geofence"}],
                     touched=True, fsm_state_after="Land")
    s.setpoints[3] = (s.setpoints[3][0], ZERO)
    s.executed[3] = (s.executed[3][0], dict(s.executed[3][1], velocity=[0.0, 0.0, 0.0]))
    s.intercepts.insert(0, (T0 + 3 * TICK + 8_000_000, {
        "t": rows[3]["t"], "violations": ["nfz-a"], "repairs": ["Brake"], "fsm": "Land"}))
    kpi = K.compute(rows, PRIORITIES, {"shield": "on"})
    rec, ticks = _replay(s, rows, traj, kpi)
    assert rec["verdict"] == "identical", rec["first_difference"]
    assert ticks[3]["setpoint"] == "brake" and ticks[3]["emitted"]["vx"] == 0.0
    assert _field(rec, "mean_repair_magnitude_mps")["equal"] is True
    assert abs(_field(rec, "mean_repair_magnitude_mps")["bag"] - 4.0) < 1e-3


def test_a_shield_off_arm_publishes_no_intercept_and_the_record_says_so():
    s, rows, traj, kpi = _episode(["pass"] * 3 + ["repair"] * 4 + ["auto_clean"] * 3,
                                  shield="off")
    assert not s.intercepts and kpi["p0_violation_ticks"] == 4
    rec, ticks = _replay(s, rows, traj, kpi, arm="off")
    assert rec["verdict"] == "identical", rec["first_difference"]
    f = _field(rec, "p0_violation_ticks")
    assert "Shield-off arm" in f["not_on_this_run"] and f["kpi_json"] == 4 and "bag" not in f
    assert _field(rec, "shield_ticks")["equal"] is True
    assert rec["kpi"]["equal"] == 3 and rec["kpi"]["not_on_this_run"] == len(R.FROM_BAG) - 3
    # read as a Shield-on run, the same bag says 0 where the table says 4
    rec, _ = _replay(s, rows, traj, kpi, arm=None)
    assert rec["verdict"] == "differs" and rec["first_difference"]["layer"] == "kpi"


def test_a_pose_taken_in_the_other_order_is_a_neighbour_not_a_difference():
    s, rows, traj, kpi = _episode()
    # the node had already taken the next pose when tick 6 ran: the log's
    # point is the bag's NEXT message
    nxt = s.poses[3 + 5 + 1][1]
    traj[5].update(x=nxt["north"] + HOME[0], y=nxt["east"] + HOME[1], up=nxt["up"])
    rec, ticks = _replay(s, rows, traj, kpi)
    tr = rec["compared_with_the_log"]["trajectory"]
    assert rec["verdict"] == "identical" and tr["next"] == 1 and tr["exact"] == len(PLAN) - 1
    assert tr["bag_order_max_error_m"] > 0.4          # one pose period at 4 m/s
    assert any("next or previous pose message" in x for x in rec["limits"])
    # the rebuilt tick keeps the recorder's order: it is not moved to agree
    assert abs(ticks[5]["x"] - (s.poses[3 + 5][1]["north"] + HOME[0])) < 1e-12
    # against the flight log's 3 decimals when there is no trajectory.json
    rec, _ = _replay(s, rows, None, kpi)
    tr = rec["compared_with_the_log"]["trajectory"]
    assert "flight_log.jsonl" in tr["against"] and tr["exact"] == len(PLAN)
    # two messages away is a difference
    far = s.poses[3 + 5 + 2][1]
    traj[5].update(x=far["north"] + HOME[0], y=far["east"] + HOME[1], up=far["up"])
    assert _replay(s, rows, traj, kpi)[0]["first_difference"]["field"] == "pose"


def test_the_node_running_behind_the_recorder_at_the_start_is_still_aligned():
    # The node ticks on the first two actions 97 ms after the recorder logged
    # them, so its first setpoint is logged after the SECOND action.
    s, rows, traj, kpi = _episode(late_start_ns=97_000_000)
    assert s.setpoints[0][0] > s.actions[4][0]
    rec, ticks = _replay(s, rows, traj, kpi)
    assert rec["verdict"] == "identical", rec["first_difference"]
    assert rec["window"]["first_tick"]["action_index"] == 4
    assert "needs the room" in rec["window"]["first_tick"]["source"]
    assert rec["account"]["setpoints"]["logged_after_the_next_action"] == 2
    t = rec["compared_with_the_log"]["ticks"]
    assert 90 <= t["first_action_logged_before_its_tick_ms"] <= 100 and t["t_jitter_ms"] <= 100
    # place(): read from the last message back, one a tick
    ticks_ns = [0, 100, 200, 300]
    assert R.place(ticks_ns, [(105, "a"), (190, "b"), (210, "c")]) == [0, 1, 2]
    assert R.place(ticks_ns, [(10, "a"), (310, "b")]) == [0, 3]
    assert R.place(ticks_ns, [(5, "a"), (20, "b")]) == [-1, 0]
    # more than MAX_START_SHIFT messages with no tick to go to: refused
    s2, *_ = _episode(pre=0)
    s2.setpoints = [(T0 + 1_000 + i, s2.setpoints[0][1]) for i in range(4)] + s2.setpoints[1:]
    try:
        R.rebuild(s2)
        raise AssertionError("four setpoints on one tick were placed")
    except R.ReplayError as e:
        assert "do not fit one a tick" in str(e)


def test_who_flew_a_tick_is_inferred_from_the_setpoint_and_the_intercept():
    raw, other = _vec(4.0, 0.0, 0.0, 0.0), _vec(0.3, 0.1, 0.0, 0.0)

    def ic(fsm):
        return {"violations": ["z"], "repairs": ["GeofenceSlide"], "fsm": fsm}
    assert R.classify(raw, raw, None, False) == ("pass", False)
    # nothing published: the autopilot's tick, whatever the Shield thought
    assert R.classify(raw, None, None, False) == ("none", False)
    assert R.classify(raw, None, ic("RTL"), False) == ("none", True)
    # a repair the FSM trusts (Normal / Brake): the Shield's
    assert R.classify(raw, other, ic("Brake"), False) == ("pass", False)
    assert R.classify(raw, ZERO, ic("Brake"), False) == ("brake", False)
    # the hand-over tick is the Shield's; after it the topic carries the hold
    assert R.classify(raw, ZERO, ic("Loiter"), False) == ("brake", True)
    assert R.classify(raw, other, ic("Loiter"), True) == ("none", True)
    assert R.classify(raw, ZERO, ic("Land"), True) == ("none", True)
    # a setpoint that is not the action, with no intercept: the hold's leg
    assert R.classify(raw, other, None, False) == ("none", True)
    # back from the hold: an untouched tick whose setpoint is the action
    assert R.classify(raw, raw, None, True) == ("pass", False)
    # float32 on the wire: the log's float64 of the same value is the same
    assert R.same_vec([0.1, 0.2, 0.3, 0.0], [_f32(0.1), _f32(0.2), _f32(0.3), 0.0])
    assert not R.same_vec([0.1, 0.2, 0.3, 0.0], [0.1, 0.2, 0.3000001, 0.0])
    assert R.same_vec([math.nan, 0, 0, 0], {"vx": math.nan, "vy": 0, "vz_up": 0, "yaw_rate": 0})
    assert R.same_vec(None, None) and not R.same_vec(None, ZERO)


def test_a_bag_with_no_action_or_no_shield_message_is_not_replayable():
    s, rows, traj, kpi = _episode()
    empty = R.BagStreams(reader="stand-in")
    rec, ticks = _replay(empty, rows, traj, kpi)
    assert rec["verdict"] == "not_replayable" and ticks is None
    assert "/vla/action_4d" in rec["why"]
    quiet = R.BagStreams(reader="stand-in", actions=list(s.actions), poses=list(s.poses))
    rec, _ = _replay(quiet, rows, traj, kpi)
    assert rec["verdict"] == "not_replayable" and "published nothing" in rec["why"]
    ep = R.Episode(name="x", source="memory", bag_dir=None)
    rec, _ = R.replay(ep)
    assert rec["verdict"] == "not_replayable" and "no rosbag2 here" in rec["why"]
    # an empty log is nothing to compare with, not a pass
    rec, _ = _replay(s, [], traj, kpi)
    assert rec["verdict"] == "not_replayable" and "no rows" in rec["why"]


def test_the_topics_read_are_the_ones_the_rails_record():
    """The names in the tool are the names in both record commands
    (sitl/run_ros2_demo.sh, deploy/docker/entrypoint.sh) and the nodes'."""
    for rel in ("sitl/run_ros2_demo.sh", "deploy/docker/entrypoint.sh"):
        text = (ROOT / rel).read_text(encoding="utf-8")
        listed = set(re.findall(r"(?<![\w/])(/[a-z0-9_]+(?:/[a-z0-9_]+)+)", text))
        missing = [t for t in R.READ_TOPICS if t not in listed]
        assert not missing, f"{rel} does not record {missing}"
    adapter = (ROOT / "sitl" / "mavlink_adapter_node.py").read_text(encoding="utf-8")
    for name, topic in (("TOPIC_SETPOINT", R.SETPOINT_TOPIC), ("TOPIC_FENCES", R.FENCES_TOPIC),
                        ("TOPIC_MODE_REQUEST", R.MODE_REQUEST_TOPIC),
                        ("TOPIC_RAW_LOCAL", R.EXECUTED_TOPIC)):
        assert re.search(rf'^{name} = "{re.escape(topic)}"$', adapter, re.M), name
    node = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    for topic in (R.ACTION_TOPIC, R.INTERCEPT_TOPIC, R.POSE_TOPIC, R.STATE_TOPIC):
        assert f'"{topic}"' in node, topic
    # the marker text the node sends is the one read back
    from sitl_stressors import marker_text
    assert R.parse_marker(marker_text("abcd1234ffff", 1.002)) == ("abcd1234", 1.002)


# --------------------------------------------------------------------------- #
# the CLI, a campaign, a bundle
# --------------------------------------------------------------------------- #

def _run_folder(root: Path, name: str, rows, traj, kpi, *, bag: bool = True) -> Path:
    run = root / name
    run.mkdir(parents=True)
    if bag:
        (run / "bag").mkdir()
        (run / "bag" / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n",
                                                   encoding="utf-8")
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    (run / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")
    (run / "kpi.json").write_text(json.dumps(kpi), encoding="utf-8")
    (run / "metrics.json").write_text(json.dumps({"shield": "on"}), encoding="utf-8")
    return run


def _tree(d: Path) -> list:
    return sorted((str(p.relative_to(d)), p.stat().st_size) for p in d.rglob("*"))


def _main(*argv) -> tuple[int, str]:
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        try:
            rc = R.main([str(a) for a in argv])
        except SystemExit as e:
            rc = e.code
    return rc, buf.getvalue()


def test_the_cli_writes_its_records_to_out_and_nothing_into_a_run_folder():
    root = _tmp("rfb_cli_")
    s, rows, traj, kpi = _episode(["pass"] * 6)
    good = _run_folder(root, "good_001", rows, traj, kpi)
    bad_kpi = dict(kpi, ticks=kpi["ticks"] + 1)
    bad = _run_folder(root, "bad_002", rows, traj, bad_kpi)
    nobag = _run_folder(root, "nobag_003", rows, traj, kpi, bag=False)
    before = _tree(root)
    out = root / "out"
    real = R.X.have_rosbags
    R.X.have_rosbags = lambda: True
    try:
        with _bag_is(s):
            rc, text = _main(good, "--out", out, "--write-ticks")
            assert rc == 0 and "1 identical" in text, text
            assert _tree(root)[: len(before)] == before or \
                [x for x in _tree(root) if not x[0].startswith("out")] == before
            rec = json.loads((out / "good_001.replay_from_bag.json").read_text(encoding="utf-8"))
            assert rec["verdict"] == "identical" and rec["bag"]["dir"] == "bag"
            # with no policy_g<N>.json beside the bag, priorities are not there
            assert "policy_g<N>.json" in rec["kpi"]["fields"]["p0_violation_ticks"][
                "not_on_this_run"]
            ticks = [json.loads(x) for x in (out / "good_001.bag_ticks.jsonl").read_text(
                encoding="utf-8").splitlines()]
            assert len(ticks) == 6 and ticks[0]["raw_body"] == rows[0]["raw_body"]
            summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
            assert summary["runs"] == 1 and summary["verdicts"] == {"identical": 1}
            assert summary["kpi_fields"]["ticks"] == {
                "source": "bag", "equal": 1, "differs": 0, "not_on_this_run": 0,
                "not_compared": 0}
            assert summary["kpi_fields"]["p0_escapes"]["source"] == "not from the bag"
            assert "why" in summary["kpi_fields"]["p0_escapes"]
            assert "not a bit-for-bit" in summary["_what"].lower()
            # one run differs: exit 1, and the summary names the first difference
            rc, text = _main(good, bad, "--out", out)
            assert rc == 1 and "first at kpi.ticks" in text, text
            summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
            assert summary["verdicts"] == {"identical": 1, "differs": 1}
            assert summary["first_differences"] == [{
                "run": "bad_002", "layer": "kpi", "field": "ticks", "tick": None,
                "bag": 6, "log": 7}]
            assert summary["layers"]["kpi"] == {"identical": 1, "differs": 1}
            assert summary["totals"]["ticks"] == 12 and summary["totals"]["pose_exact"] == 12
            # a folder with no bag: exit 2, said
            rc, text = _main(good, nobag, "--out", out)
            assert rc == 2 and "not_replayable" in text and "no rosbag2 here" in text, text
            # the grant's name for the bag directory is found too
            (good / "bag").rename(good / "episode.bag2")
            rc, text = _main(good)
            assert rc == 0, text
            (good / "episode.bag2").rename(good / "bag")
            # --limit, and the refusals
            rc, text = _main(good, bad, "--limit", 1)
            assert rc == 0 and "1 run(s)" in text
            assert _main()[0] == 2 and _main(good, "--write-ticks")[0] == 2
        assert [x for x in _tree(root) if not x[0].startswith("out")] == before, \
            "the tool wrote into a run folder"
    finally:
        R.X.have_rosbags = real
    # without rosbags: said, exit 3, nothing replayed
    R.X.have_rosbags = lambda: False
    try:
        rc, text = _main(good)
    finally:
        R.X.have_rosbags = real
    assert rc == 3 and "pip install" in text, text


def test_a_campaigns_runs_are_the_ones_its_sweep_json_accepted():
    root = _tmp("rfb_camp_")
    store = root / "hil_smoke"
    for tag in ("c_001_a", "c_002_b", "c_004_d"):
        (store / tag).mkdir(parents=True)
    camp = root / "hil_c"
    camp.mkdir()
    elsewhere = "Z:/gone/hil_smoke"
    (camp / "sweep.json").write_text(json.dumps({"results": [
        {"run": str(store / "c_001_a"), "status": "pass"},
        {"run": f"{elsewhere}/c_002_b", "status": "fail"},        # the store moved
        {"run": None, "status": "skipped"},
        {"run": str(store / "c_003_c"), "status": "error", "error": "mission c_003_c"},
        {"run": str(store / "c_004_d"), "status": "tracked"}]}), encoding="utf-8")
    runs = R.campaign_runs(camp)
    assert [p.name for p in runs] == ["c_001_a", "c_002_b", "c_004_d"]
    assert all(p.is_dir() for p in runs), runs
    assert R.campaign_runs(camp / "sweep.json") == runs
    (camp / "other.json").write_text("{}", encoding="utf-8")
    try:
        R.campaign_runs(camp / "other.json")
        raise AssertionError("a file with no results was read as a campaign")
    except R.ReplayError as e:
        assert "no `results`" in str(e)
    rc, text = _main("--campaign", camp / "other.json")
    assert rc == 2 and "no `results`" in text


def test_a_replay_bundles_bag_is_unpacked_from_the_verified_archive():
    """The bag a replay bundle carries (guardrail/replay.py) is read from the
    archive's digested members, flat in a folder of its own; the log, the KPI
    table and the last generation's priorities are the bundle's. The episode
    is tests/test_replay.py's synthetic one, its bag a stand-in."""
    import test_replay as RP
    from guardrail.replay import write_replay
    root = _tmp("rfb_bundle_")
    run, policy, rec0, rows = RP._episode(root, hot=False)
    RP._bag(run, rec0.episode_id - int(20e9))
    out = write_replay(run, policy, root / "r.replay.tar.gz", signer=None)
    s, *_ = _episode(["pass"] * 4)
    with _bag_is(s) as seen:
        rec, ticks = R.replay_path(out)
    assert seen == [["bag_0.mcap", "metadata.yaml"]], seen
    assert rec["run"] == "r" and rec["verdict"] in ("identical", "differs")
    assert rec["compared_with_the_log"]["log_rows"] == len(rows)
    assert any("bundle's last policy generation" in x for x in rec["not_from_the_bag"])
    # a bundle that does not verify is not replayed
    bad = root / "bad.replay.tar.gz"
    bad.write_bytes(out.read_bytes()[:-200])
    rec, ticks = R.replay_path(bad)
    assert rec["verdict"] == "not_replayable" and ticks is None
    rec, _ = R.replay_path(root / "nothing-here")
    assert rec["verdict"] == "not_replayable" and "neither a run folder" in rec["why"]


# --------------------------------------------------------------------------- #
# a real rosbag2, written and read with rosbags
# --------------------------------------------------------------------------- #

def _write_bag(bag_dir: Path, s) -> None:
    """The stand-in's topics as a real rosbag2, with the message types the
    rail records (std_msgs, geometry_msgs, mavros_msgs)."""
    import numpy as np
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    store = get_typestore(getattr(Stores, "ROS2_JAZZY", None) or Stores.LATEST)
    for name, text in R.FALLBACK_MSGS.items():
        store.register(get_types_from_msg(text, name))
    T = store.types
    Time, Header = T["builtin_interfaces/msg/Time"], T["std_msgs/msg/Header"]
    Point, Vector3 = T["geometry_msgs/msg/Point"], T["geometry_msgs/msg/Vector3"]
    Quaternion, Pose = T["geometry_msgs/msg/Quaternion"], T["geometry_msgs/msg/Pose"]
    F32, Layout = T["std_msgs/msg/Float32MultiArray"], T["std_msgs/msg/MultiArrayLayout"]
    String = T["std_msgs/msg/String"]
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
        def put(topic, typ, items, build):
            try:
                c = w.add_connection(topic, typ, typestore=store)
            except TypeError:                    # rosbags < 0.10
                c = w.add_connection(topic, typ)
            for ts, v in items:
                w.write(c, ts, store.serialize_cdr(build(ts, v), typ))

        def f32(ts, v):
            return F32(layout=Layout(dim=[], data_offset=0), data=np.array(v, dtype=np.float32))
        put(R.ACTION_TOPIC, "std_msgs/msg/Float32MultiArray", s.actions, f32)
        put(R.SETPOINT_TOPIC, "std_msgs/msg/Float32MultiArray", s.setpoints, f32)
        for topic, items in ((R.INTERCEPT_TOPIC, s.intercepts), (R.FENCES_TOPIC, s.fences)):
            put(topic, "std_msgs/msg/String", items, lambda ts, v: String(data=json.dumps(v)))
        put(R.MODE_REQUEST_TOPIC, "std_msgs/msg/String", [(T0 + 99, "RTL")],
            lambda ts, v: String(data=v))

        def pose(ts, p):
            yaw_enu = math.radians(90.0 - p["yaw_deg"])
            return T["geometry_msgs/msg/PoseStamped"](header=header(ts), pose=Pose(
                position=Point(x=p["east"], y=p["north"], z=p["up"]),
                orientation=Quaternion(x=0.0, y=0.0, z=math.sin(yaw_enu / 2),
                                       w=math.cos(yaw_enu / 2))))
        put(R.POSE_TOPIC, "geometry_msgs/msg/PoseStamped", s.poses, pose)
        put(R.STATE_TOPIC, "mavros_msgs/msg/State", s.states,
            lambda ts, v: T["mavros_msgs/msg/State"](
                header=header(ts), connected=v["connected"], armed=v["armed"], guided=True,
                manual_input=False, mode=v["mode"], system_status=4))
        put(R.EXECUTED_TOPIC, R.X.POSITION_TARGET_TYPE, s.executed,
            lambda ts, f: T[R.X.POSITION_TARGET_TYPE](
                header=header(ts), coordinate_frame=f["coordinate_frame"],
                type_mask=f["type_mask"], position=Point(x=0.0, y=0.0, z=0.0),
                velocity=Vector3(x=f["velocity"][0], y=f["velocity"][1], z=f["velocity"][2]),
                acceleration_or_force=Vector3(x=0.0, y=0.0, z=0.0), yaw=0.0,
                yaw_rate=f["yaw_rate"]))

        def mavlink(ts, v):
            msgid, payload = v
            padded = payload + b"\0" * (-len(payload) % 8)
            return T["mavros_msgs/msg/Mavlink"](
                header=header(ts), framing_status=1, magic=253, len=len(payload),
                incompat_flags=0, compat_flags=0, seq=0, sysid=1, compid=191, msgid=msgid,
                checksum=0, payload64=np.frombuffer(padded, dtype="<u8").copy(),
                signature=np.array([], dtype=np.uint8))
        from sitl_stressors import marker_text
        sink = [(ns, (253, b"\x06" + marker_text(n, t).encode("ascii")))
                for ns, (n, t) in s.markers]
        # other traffic to the autopilot, and a status text that is no marker
        sink += [(T0 + 7, (84, b"\x01" * 40)), (T0 + 9, (253, b"\x06Fence Indexed OK"))]
        put(R.MAVLINK_SINK_TOPIC, "mavros_msgs/msg/Mavlink", sorted(sink), mavlink)
        # a topic the tool does not read
        put("/mavros/battery", "std_msgs/msg/String", [(T0, "x")], lambda ts, v: String(data=v))


def test_a_real_rosbag2_is_read_and_replayed_with_rosbags():
    """The whole path on a rosbag2 written with `rosbags` (the reader the tool
    uses): every topic read back as written, the run folder identical through
    the CLI, a changed log differing. SKIPPED where rosbags is not installed."""
    if not R.X.have_rosbags():
        print(f"      SKIP: rosbags is not installed ({R.X.ROSBAGS_INSTALL_HINT})")
        return SKIP
    root = _tmp("rfb_real_")
    s, rows, traj, kpi = _episode()
    run = _run_folder(root, "real_001", rows, traj, kpi, bag=False)
    _write_bag(run / "bag", s)
    got = R.read_streams(run / "bag")
    assert got.reader.startswith("rosbags")
    assert got.actions == s.actions and got.setpoints == s.setpoints
    assert got.intercepts == s.intercepts and got.fences == s.fences
    assert got.states == s.states and got.mode_requests == [(T0 + 99, "RTL")]
    assert got.markers == s.markers, got.markers            # the non-marker texts are left
    assert [ns for ns, _ in got.executed] == [ns for ns, _ in s.executed]
    assert got.executed[0][1]["velocity"] == s.executed[0][1]["velocity"]
    assert len(got.poses) == len(s.poses)
    for (_, a), (_, b) in zip(got.poses, s.poses):
        assert (a["north"], a["east"], a["up"]) == (b["north"], b["east"], b["up"])
        assert abs(a["yaw_deg"] - b["yaw_deg"]) < 1e-9
    assert got.topics["/mavros/battery"] == 1 and got.topics[R.ACTION_TOPIC] == len(s.actions)
    assert got.start_ns == min(s.states[0][0], s.actions[0][0], s.poses[0][0])
    out = root / "out"
    rc, text = _main(run, "--out", out)
    assert rc == 0 and "identical" in text, text
    rec = json.loads((out / "real_001.replay_from_bag.json").read_text(encoding="utf-8"))
    assert rec["verdict"] == "identical" and rec["bag"]["reader"].startswith("rosbags")
    assert rec["bag"]["topics_read"][R.SETPOINT_TOPIC] == len(s.setpoints)
    assert rec["window"]["markers"][0]["nonce"] == "abcd1234"
    assert rec["compared_with_the_log"]["trajectory"]["exact"] == len(PLAN)
    # the same folder with one action changed in the log
    rows[7]["raw_body"]["vy"] = 0.5
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    rc, text = _main(run)
    assert rc == 1 and "first at ticks.raw_body, tick 8" in text, text
    # a folder that is not a rosbag2
    junk = _run_folder(root, "junk_002", rows, traj, kpi)
    (junk / "bag" / "bag_0.mcap").write_bytes(b"not a bag")
    rec, _ = R.replay_path(junk)
    assert rec["verdict"] == "not_replayable" and "could not be read" in rec["why"], rec


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
