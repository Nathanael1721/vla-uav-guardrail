"""guardrail/vla_link.py: the AerialVLA pilot's client, step tracker and records.

Run either way:
    pytest tests/test_vla_link.py -v
    python tests/test_vla_link.py

WHAT IS BEING CLAIMED

1. Each AerialVLA answer is a STEP (forward metres, down metres, heading
   change radians), as upstream trained and executes it, never a velocity.
   The tracker anchors it at the pose the image was captured at, turns the
   short way across +-pi, translates only when |dpsi| < 0.25 rad, never flies
   backwards along the step, holds speed, climb and yaw rate under their caps,
   and clamps the altitude target into the policy's band.
2. A step ends by tolerance or by its time cap; a decision older than the
   timeout is never flown (hover), and neither is one captured before a rail
   hold ended. A malformed answer holds zero, counted; a server error is no
   answer (hover, counted, backed off). LAND stops the tracker and is
   reported as a flag; nothing in the module can request a flight mode.
3. Every tick names its author: vla_step, hold, hover or none.
4. The client speaks protocol v2 exactly (tests/vla_fake_server.py refuses
   what the real server refuses): one request outstanding, submit() never
   blocks and never overwrites an unread result, timeouts and dropped
   connections fail cleanly and reconnect, the worker survives anything, a
   reply that breaks the protocol is never read as a decision (parse_ok is
   re-checked against raw_text), the model identity is checked and pinned,
   and the log is written off the caller's thread and never raises.
5. Closed loop: a point vehicle that follows the tracker's actions reaches a
   goal from scripted steps (quantized to the model's 99 bins), pipelined and
   stop-and-go, with latencies from 0.4 s to 3 s.

Shown failing first: guardrail/vla_link.py did not exist at 7c9c9e6, so every
test here fails there at import. The review fixes of 2026-10-22 (the tests
that cite "Review of 7c7cc50") each failed at 7c7cc50: a server error was
accepted as a hold; a second submit overwrote an unread result; raw_text was
not checked; a nested reply killed the worker; the identity was not checked;
StepLog had no writer thread; policy_alt_band did not exist. No model, GPU or
simulator is needed; the TCP tests use a loopback fake server.
"""
from __future__ import annotations

import ast
import io
import json
import math
import sys
import time
import tokenize
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))
sys.path.insert(0, str(ROOT / "tests"))

from guardrail.models import Action4D, load_policy                  # noqa: E402
from guardrail import vla_link as L                                  # noqa: E402
from guardrail.vla_link import (AUTHORS, Capture, Decision, Hint, Pose,  # noqa: E402
                                RemoteVLA, Step, StepLog, StepTracker,
                                TrackerConfig)
import vla_fake_server as F                                          # noqa: E402

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

SKIP = "SKIP"
MODULE = ROOT / "guardrail" / "vla_link.py"
JPG = b"\xff\xd8\xff\xe0fake-jpeg\xff\xd9"


def _cap(t=0.0, pose=None, down=True):
    return Capture(t, pose or Pose(0.0, 0.0, 10.0, 0.0), JPG, JPG if down else None)


def _fields(t=0.0, pose=None, phrase="straight ahead ", src="goal"):
    return L.build_infer_fields(_cap(t, pose), Hint(phrase, src), "an orange car")


class _Vehicle:
    """A point multirotor: velocity follows the command with a first-order lag
    (tau 0.3 s), heading integrates the yaw rate. World frame, as Action4D."""

    def __init__(self, x=0.0, y=0.0, up=10.0, yaw=0.0, tau=0.3):
        self.x, self.y, self.up, self.yaw, self.tau = x, y, up, yaw, tau
        self.vx = self.vy = self.vz = 0.0

    def pose(self) -> Pose:
        return Pose(self.x, self.y, self.up, self.yaw)

    def step(self, a: Action4D, dt: float = 0.1) -> None:
        k = min(1.0, dt / self.tau)
        self.vx += k * (a.vx - self.vx)
        self.vy += k * (a.vy - self.vy)
        self.vz += k * (a.vz_up - self.vz)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.up += self.vz * dt
        self.yaw = L.wrap_pi(self.yaw + a.yaw_rate * dt)


def _fly(tracker, veh, t0, t1, dt=0.1):
    """Tick the tracker and move the vehicle from t0 to t1; the ticks."""
    out, t = [], t0
    while t < t1 - 1e-9:
        tk = tracker.tick(t, veh.pose())
        veh.step(tk.action, dt)
        out.append((t, tk))
        t = round(t + dt, 10)
    return out


# --------------------------------------------------------------------------- #
# 1. The vocabulary and the request
# --------------------------------------------------------------------------- #
def test_the_vocabulary_and_the_goal_phrase_are_the_training_ones():
    """The seven compass phrases are vla_bridge's, trailing spaces and all,
    and goal_phrase() is semantic_direction() (the phrase upstream computes
    from the goal) at every bearing: a sweep at 0.25 deg over the full circle,
    at four headings, plus the "" inside 0.5 m."""
    import vla_bridge
    from aerialvla_demo import semantic_direction
    assert set(L.DIRECTION_VOCABULARY) == set(vla_bridge.DIRECTION_VOCABULARY)
    assert L.PHRASES == set(vla_bridge.DIRECTION_VOCABULARY) | {""}
    n = 0
    for yaw in (0.0, 1.0, -2.5, 3.1):
        pose = Pose(3.0, -4.0, 10.0, yaw)
        for k in range(1440):
            a = math.radians(k * 0.25)
            goal = (3.0 + 10.0 * math.cos(a), -4.0 + 10.0 * math.sin(a))
            want = semantic_direction((pose.x, pose.y), yaw, goal)
            assert L.goal_phrase(pose, goal) == want, (yaw, k * 0.25)
            n += 1
    assert L.goal_phrase(Pose(0, 0, 10, 0), (0.3, 0.2)) == ""
    assert semantic_direction((0, 0), 0.0, (0.3, 0.2)) == ""
    assert n == 5760


def test_a_request_carries_exactly_the_protocol_fields_and_no_coordinates():
    fields, meta = _fields(1.5, Pose(1.0, 2.0, 12.0, 0.3))
    assert set(fields) | {"seq"} == set(L.INFER_FIELDS)
    assert fields["v"] == 2 and fields["op"] == "infer"
    assert fields["pose"] == {"x": 1.0, "y": 2.0, "up": 12.0, "yaw_rad": 0.3}
    assert fields["csp_mode"] == "off" and fields["csp_text"] is None
    assert meta["front_jpg_bytes"] == len(JPG) and "front_jpg_b64" not in meta
    for k in L.FORBIDDEN_FIELDS:
        bad = dict(fields, seq=1, **{k: [10.0, 20.0]})
        try:
            L.check_request_fields(bad)
        except ValueError as exc:
            assert "coordinates" in str(exc), exc
        else:
            raise AssertionError(f"a request carrying {k!r} was not refused")
    try:
        L.check_request_fields(dict(fields, seq=1, hint="x"))
    except ValueError:
        pass
    else:
        raise AssertionError("an unknown field was not refused")


def test_a_hints_kind_is_recorded_never_sent():
    """Review of d6e7123, P2-2: a follow's detector phrase comes from a
    fresh box, from the remembered sighting or from the search turn, and the
    records called all three "detector" (57-74 % of the decisions of follows
    #6, #8 and #9 were memory or search). Hint.kind says which: it goes into
    the request's record (meta `hint_kind`) and never onto the wire (the
    protocol's hint_source stays "detector"); a hint without a kind - every
    other rail's - records exactly what it did before; two hints are equal
    on phrase and source, as before."""
    plain_f, plain_m = _fields(1.0, phrase="forward-left ", src="detector")
    f, m = L.build_infer_fields(_cap(1.0), Hint("forward-left ", "detector", "memory"),
                                "an orange car")
    assert f == plain_f and "hint_kind" not in plain_m and m["hint_kind"] == "memory"
    assert {k: v for k, v in m.items() if k != "hint_kind"} == plain_m
    assert set(f) | {"seq"} == set(L.INFER_FIELDS)
    assert Hint("forward-left ", "detector", "search") == Hint("forward-left ", "detector")
    assert Hint("forward-left ", "detector").kind is None
    for bad in ("", 3, "memory "):
        try:
            Hint("forward-left ", "detector", bad)
        except ValueError:
            continue
        raise AssertionError(f"hint kind {bad!r} accepted")


def test_hints_and_csp_arms_are_refused_outside_the_protocol():
    for phrase, src in (("go to the red car ", "goal"), ("straight ahead", "goal"),
                        ("straight ahead ", "operator"), ("forward-left ", "none"),
                        ("", "detector")):
        try:
            Hint(phrase, src)
        except ValueError:
            continue
        raise AssertionError(f"Hint({phrase!r}, {src!r}) was accepted")
    assert Hint("", "goal").phrase == "" and Hint("", "none").source == "none"
    for mode, text in (("off", "rules"), ("csp", None), ("neutral", ""), ("on", "x")):
        try:
            L.build_infer_fields(_cap(), Hint("straight ahead ", "goal"), "", mode, text)
        except ValueError:
            continue
        raise AssertionError(f"csp_mode {mode!r} with {text!r} was accepted")
    f, m = L.build_infer_fields(_cap(), Hint("straight ahead ", "goal"), "", "csp", "No NFZ.")
    assert f["csp_text"] == "No NFZ." and len(m["csp_sha256"]) == 64


MODE_NAMES = {"set_mode", "request_mode", "mode_request", "asks", "arm", "disarm",
              "takeoff", "command_long", "mavutil", "pymavlink", "mavros", "rclpy",
              "loiter", "rtl", "guided", "brake", "flight_mode"}
MODE_STRINGS = {"LAND", "LOITER", "RTL", "GUIDED", "BRAKE", "ALT_HOLD", "POSHOLD",
                "AUTO", "STABILIZE", "SMART_RTL"}


def test_the_module_can_never_request_a_flight_mode():
    """LAND reaches the rail as a flag (Tick.land_requested); the escalation
    FSM or the rail lands. Checked on the source: no identifier that talks to
    an autopilot or names a mode, no MAVLink/ROS import, no mode name as a
    string constant outside docstrings - and the same for RemoteVLABackend.
    A tick carries an action, its author and the LAND flag, nothing else."""
    import dataclasses
    import inspect
    import textwrap
    from guardrail import vla_backends
    for name, src in (("vla_link", MODULE.read_text(encoding="utf-8")),
                      ("RemoteVLABackend", textwrap.dedent(
                          inspect.getsource(vla_backends.RemoteVLABackend)))):
        tree = ast.parse(src)
        names = {t.string.lower() for t in tokenize.generate_tokens(
            io.StringIO(src).readline) if t.type == tokenize.NAME}
        assert not names & MODE_NAMES, (name, sorted(names & MODE_NAMES))
        docs = {id(n.body[0].value) for n in ast.walk(tree)
                if isinstance(n, (ast.Module, ast.ClassDef, ast.FunctionDef))
                and n.body and isinstance(n.body[0], ast.Expr)
                and isinstance(n.body[0].value, ast.Constant)}
        consts = {n.value for n in ast.walk(tree) if isinstance(n, ast.Constant)
                  and isinstance(n.value, str) and id(n) not in docs}
        assert not consts & MODE_STRINGS, (name, sorted(consts & MODE_STRINGS))
        for n in ast.walk(tree):
            if isinstance(n, (ast.Import, ast.ImportFrom)):
                mods = [a.name for a in n.names] + [getattr(n, "module", "") or ""]
                assert not any(m.split(".")[0] in ("pymavlink", "rclpy", "mavros",
                                                   "dronekit", "mavsdk") for m in mods), mods
    assert {f.name for f in dataclasses.fields(L.Tick)} == {
        "action", "author", "reason", "seq", "age_s", "land_requested"}


# --------------------------------------------------------------------------- #
# 2. Step geometry
# --------------------------------------------------------------------------- #
def test_a_step_is_anchored_at_its_capture_pose():
    """Upstream: new heading = heading + dpsi, then fwd_m along it and down_m
    down. The target is fixed in the world at the CAPTURE pose; the aircraft
    has moved since (pipelining), and it flies toward that fixed point."""
    tr = StepTracker()
    cap = Pose(10.0, 5.0, 20.0, 0.5)
    d = Decision.of(7, 0.0, cap, Step(4.0, -2.0, 0.2))
    assert tr.accept(d, 0.4)
    tgt = d.record["target"]
    assert abs(tgt["psi_rad"] - 0.7) < 1e-12
    assert abs(tgt["x"] - (10.0 + 4.0 * math.cos(0.7))) < 1e-12
    assert abs(tgt["y"] - (5.0 + 4.0 * math.sin(0.7))) < 1e-12
    assert tgt["up"] == 22.0 and not tgt["rotate_only"] and not tgt["alt_clamped"]
    now = Pose(11.0, 5.5, 20.5, 0.6)              # where it is when the step lands
    tk = tr.tick(0.5, now)
    assert tk.author == "vla_step" and tk.seq == 7 and abs(tk.age_s - 0.5) < 1e-12
    dx, dy = tgt["x"] - now.x, tgt["y"] - now.y
    norm = math.hypot(dx, dy)
    speed = math.hypot(tk.action.vx, tk.action.vy)
    assert abs(speed - 1.0) < 1e-9, speed        # upstream cruise, 1.0 m/s
    assert abs(tk.action.vx / speed - dx / norm) < 1e-9
    assert abs(tk.action.vy / speed - dy / norm) < 1e-9
    assert tk.action.vz_up > 0 and tk.action.yaw_rate > 0


def test_the_heading_turns_the_short_way_across_pi():
    """psi_capture 3.0 + dpsi 0.5 = 3.5 rad, which is -2.78 after the wrap. At
    a current heading of 3.1 the short way is +0.40 rad (clockwise), not
    -5.88. Mirror case on the other side."""
    for yaw_cap, dpsi, yaw_now, sign in ((3.0, 0.5, 3.1, 1.0), (-3.0, -0.5, -3.1, -1.0),
                                        (3.0, 1.1, -2.9, 1.0)):
        tr = StepTracker()
        tr.accept(Decision.of(1, 0.0, Pose(0, 0, 10, yaw_cap), Step(0.0, 0.0, dpsi)), 0.0)
        tk = tr.tick(0.1, Pose(0, 0, 10, yaw_now))
        err = L.wrap_pi(yaw_cap + dpsi - yaw_now)
        assert abs(err) < math.pi / 2, err
        assert math.copysign(1.0, tk.action.yaw_rate) == sign, (yaw_cap, dpsi, tk)
        assert abs(tk.action.yaw_rate) <= 0.5 + 1e-12
        # Closed loop: it settles on the wrapped target without a full turn.
        veh = _Vehicle(yaw=yaw_now)
        turned = 0.0
        for t, k in _fly(tr, veh, 0.2, 6.0):
            turned += abs(k.action.yaw_rate) * 0.1
        assert abs(L.wrap_pi(veh.yaw - (yaw_cap + dpsi))) <= 0.05 + 1e-9, veh.yaw
        assert turned < abs(err) + 0.2, turned


def test_translation_only_below_the_rotate_gate():
    """|dpsi| >= 0.25 rad: rotate only (plus altitude), no horizontal move,
    whatever fwd_m says. Below it: translate along the new heading at once."""
    for dpsi in (0.25, -0.25, 0.30, 1.1):
        tr = StepTracker()
        veh = _Vehicle()
        tr.accept(Decision.of(1, 0.0, veh.pose(), Step(5.0, -1.0, dpsi)), 0.0)
        ticks = _fly(tr, veh, 0.0, 8.0)
        flown = [k for _, k in ticks if k.author == "vla_step"]
        assert flown and all(k.reason == "rotate" for k in flown), dpsi
        assert all(k.action.vx == 0.0 and k.action.vy == 0.0 for _, k in ticks), dpsi
        assert any(k.action.vz_up > 0 for k in flown), "altitude still changes"
        assert math.hypot(veh.x, veh.y) < 1e-9 and abs(veh.up - 11.0) < 0.35
        assert abs(L.wrap_pi(veh.yaw - dpsi)) <= 0.05 + 1e-9
    for dpsi in (0.249, -0.2, 0.0):
        tr = StepTracker()
        tr.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(3.0, 0.0, dpsi)), 0.0)
        tk = tr.tick(0.1, Pose(0, 0, 10, 0))
        assert tk.reason == "translate" and math.hypot(tk.action.vx, tk.action.vy) > 0.9


def test_the_follow_executor_turns_while_moving_and_holds_its_altitude():
    """TrackerConfig.turn_while_moving / hold_altitude_m (2026-10-23, the
    follow executor; both off by default, above): a big turn still flies its
    fwd_m along the new heading, slowed by cos(heading error), and the
    altitude target is the held one whatever down_m says - recorded on the
    step's target, counted.

    Review of d6e7123 (P3): the tick's reason for such a step is
    "turn_moving", not "translate" (its rotation dominates: upstream would
    only have rotated), and the record keeps the MODEL's altitude target as
    up_raw beside the held one (up_held) - it used to overwrite up_raw with
    the held altitude, so what the model asked for was lost. A new policy
    band re-clamps the held target, not the model's."""
    cfg = TrackerConfig(cruise_mps=2.5, speed_cap_mps=3.0, turn_while_moving=True,
                        hold_altitude_m=10.0)
    tr = StepTracker(cfg)
    veh = _Vehicle()
    d = Decision.of(1, 0.0, veh.pose(), Step(4.0, -3.0, 0.6))     # a 3 m climb asked
    tr.accept(d, 0.0)
    tk = tr.tick(0.1, veh.pose())
    assert tk.reason == "turn_moving" and math.hypot(tk.action.vx, tk.action.vy) > 0.5, tk
    assert math.hypot(tk.action.vx, tk.action.vy) < 2.5 * math.cos(0.6) + 1e-6
    tgt = d.record["target"]
    assert tgt["turn_while_moving"] and tgt["alt_held"] and tgt["up"] == 10.0, tgt
    assert tgt["up_raw"] == 13.0 and tgt["up_held"] == 10.0 and not tgt["rotate_only"], tgt
    _fly(tr, veh, 0.1, 10.0)
    assert math.hypot(veh.x, veh.y) > 3.0 and abs(veh.up - 10.0) < 0.35
    assert tr.counts["turn_while_moving"] == 1 and tr.counts["alt_held"] == 1
    # A small turn under the same executor still translates.
    small = StepTracker(cfg)
    small.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(4.0, 0.0, 0.1)), 0.0)
    assert small.tick(0.1, Pose(0, 0, 10, 0)).reason == "translate"
    # A hot-applied band re-clamps the HELD target (10 -> 12.5 in 12-20 m).
    held = StepTracker(cfg)
    d3 = Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(4.0, -3.0, 0.1))
    held.accept(d3, 0.0)
    held.set_alt_band((12.0, 20.0))
    assert held.tick(0.1, Pose(0, 0, 12.5, 0)).action.vz_up == 0.0
    # Off (the default): the same answer rotates in place and climbs.
    plain = StepTracker(TrackerConfig(cruise_mps=2.5, speed_cap_mps=3.0))
    d2 = Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(4.0, -3.0, 0.6))
    plain.accept(d2, 0.0)
    tk = plain.tick(0.1, Pose(0, 0, 10, 0))
    assert tk.reason == "rotate" and tk.action.vx == 0.0 and tk.action.vy == 0.0
    assert d2.record["target"]["up"] == 13.0 and not d2.record["target"]["alt_held"]
    assert "up_held" not in d2.record["target"] and d2.record["target"]["up_raw"] == 13.0
    for bad in (float("nan"), float("inf")):
        try:
            TrackerConfig(hold_altitude_m=bad)
        except ValueError:
            continue
        raise AssertionError(f"hold_altitude_m {bad} accepted")


def test_a_follow_step_can_be_anchored_where_its_answer_arrives():
    """TrackerConfig.anchor_at_answer (2026-10-23, follow #3 flew 1.54 m/s at
    a 4.5 m/s cruise): the step's displacement starts from the pose passed
    with the answer, not the capture pose; the heading target stays capture
    yaw + dpsi; recorded as anchor "answer". Off, or no pose: the capture."""
    on = StepTracker(TrackerConfig(anchor_at_answer=True))
    d = Decision.of(1, 0.0, Pose(0.0, 0.0, 10.0, 0.0), Step(4.0, 0.0, 0.1))
    on.accept(d, 0.0, pose_now=Pose(3.0, 0.5, 10.0, 0.05))
    tgt = d.record["target"]
    assert tgt["anchor"] == "answer" and abs(tgt["psi_rad"] - 0.1) < 1e-12
    assert abs(tgt["x"] - (3.0 + 4.0 * math.cos(0.1))) < 1e-9
    assert abs(tgt["y"] - (0.5 + 4.0 * math.sin(0.1))) < 1e-9
    off = StepTracker()
    d2 = Decision.of(1, 0.0, Pose(0.0, 0.0, 10.0, 0.0), Step(4.0, 0.0, 0.1))
    off.accept(d2, 0.0, pose_now=Pose(3.0, 0.5, 10.0, 0.05))
    assert d2.record["target"]["anchor"] == "capture"
    assert abs(d2.record["target"]["x"] - 4.0 * math.cos(0.1)) < 1e-9
    nopose = StepTracker(TrackerConfig(anchor_at_answer=True))
    d3 = Decision.of(1, 0.0, Pose(0.0, 0.0, 10.0, 0.0), Step(4.0, 0.0, 0.1))
    nopose.accept(d3, 0.0)
    assert d3.record["target"]["anchor"] == "capture"


def test_a_follow_holds_on_the_models_land_instead_of_landing():
    """TrackerConfig.land_as_arrival (follow #5 ended on LAND 0.08 m from the
    car after a 17 s chase): the answer is a hold (author hold, reason
    land_as_arrival - review of d6e7123, P2-6: it was "land", which read as a
    LAND the rail would fly), land_requested stays False, the next answer
    supersedes it, a silent pilot still goes to hover after the decision
    timeout. Off: LAND as before."""
    tr = StepTracker(TrackerConfig(land_as_arrival=True, decision_timeout_s=5.0))
    land = Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(0.0, 0.0, 0.0), land=True)
    assert tr.accept(land, 0.1)
    tk = tr.tick(0.2, Pose(0, 0, 10, 0))
    assert (tk.author, tk.reason, tk.land_requested) == ("hold", "land_as_arrival", False), tk
    assert tk.action.vx == 0.0 and tk.action.vy == 0.0 and not tr.land_requested
    assert land.record["land_as_arrival"] and tr.counts["land_as_arrival"] == 1
    step = Decision.of(2, 0.3, Pose(0, 0, 10, 0), Step(3.0, 0.0, 0.0))
    assert tr.accept(step, 0.4)
    assert tr.tick(0.5, Pose(0, 0, 10, 0)).reason == "translate"
    tr2 = StepTracker(TrackerConfig(land_as_arrival=True, decision_timeout_s=5.0))
    tr2.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(0.0, 0.0, 0.0), land=True), 0.1)
    assert tr2.tick(6.0, Pose(0, 0, 10, 0)).author == "hover"      # silent: hover
    plain = StepTracker()
    plain.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(0.0, 0.0, 0.0), land=True), 0.1)
    assert plain.land_requested and plain.tick(0.2, Pose(0, 0, 10, 0)).land_requested


def test_the_altitude_target_is_clamped_into_the_policy_band():
    """up_capture - down_m, clamped into the hard altitude envelope (inset
    0.5 m): the model's 5 m descent from 12 m in a 10-30 m band stops at
    10.5 m, recorded with the unclamped 7 m. The band comes from the policy
    file, and a hot-applied band re-clamps the step being flown."""
    band = L.alt_band_from_policy(load_policy(ROOT / "policies" / "follow_car_citylife.yaml"))
    assert band == (6.0, 14.0), band
    assert L.alt_band_from_policy(load_policy(ROOT / "policies" / "corridor_survey.yaml")) is None
    tr = StepTracker(TrackerConfig(alt_band=(10.0, 30.0)))
    d = Decision.of(1, 0.0, Pose(0, 0, 12.0, 0), Step(0.0, 5.0, 0.0))
    tr.accept(d, 0.0)
    assert d.record["target"]["up"] == 10.5 and d.record["target"]["up_raw"] == 7.0
    assert d.record["target"]["alt_clamped"] and tr.counts["alt_clamped"] == 1
    veh = _Vehicle(up=12.0)
    _fly(tr, veh, 0.0, 8.0)
    assert 10.5 - 0.31 <= veh.up <= 10.5 + 0.31, veh.up
    up = StepTracker(TrackerConfig(alt_band=(10.0, 30.0)))
    d2 = Decision.of(1, 0.0, Pose(0, 0, 28.0, 0), Step(0.0, -5.0, 0.0))
    up.accept(d2, 0.0)
    assert d2.record["target"]["up"] == 29.5
    free = StepTracker()
    d3 = Decision.of(1, 0.0, Pose(0, 0, 12.0, 0), Step(0.0, 5.0, 0.0))
    free.accept(d3, 0.0)
    assert d3.record["target"]["up"] == 7.0 and not d3.record["target"]["alt_clamped"]
    assert free.tick(0.1, Pose(0, 0, 12.0, 0)).action.vz_up < 0      # descending to 7
    free.set_alt_band((8.0, 20.0))                # hot-apply mid-step
    assert free._cur.up_t == 8.5 and free._cur.alt_clamped
    tk = free.tick(0.2, Pose(0, 0, 8.4, 0))
    assert tk.action.vz_up > 0, tk                # never below the new floor's inset
    thin = StepTracker(TrackerConfig(alt_band=(10.0, 11.0)))
    d4 = Decision.of(1, 0.0, Pose(0, 0, 10.5, 0), Step(0.0, 5.0, 0.0))
    thin.accept(d4, 0.0)
    assert d4.record["target"]["up"] == 10.25     # inset is a quarter of a thin band


def test_the_policy_band_skips_msl_envelopes_and_never_widens_a_configured_band():
    """Review of 7c7cc50, P8. An MSL envelope was clamped as if its heights
    were above ground (the Shield refuses to enforce MSL envelopes: there is
    no ground-elevation source, Shield.unenforceable); it is now skipped and
    named. A policy band replaced a band the caller configured instead of
    intersecting with it; combine_alt_bands keeps the tighter of the two and
    says which applied. When the two do not overlap the policy wins (the
    Shield enforces the policy band, not the configured one), said so."""
    from types import SimpleNamespace
    from guardrail.models import AltitudeEnvelope
    agl = AltitudeEnvelope(id="agl", type="altitude_envelope", **HARD_P0, alt_min_m=5, alt_max_m=40)
    msl = AltitudeEnvelope(id="msl", type="altitude_envelope", **HARD_P0, alt_min_m=120,
                           alt_max_m=300, altitude_ref="MSL")
    soft = AltitudeEnvelope(id="soft", type="altitude_envelope",
                            priority="P0", violation_action="project_fix", alt_min_m=10,
                            alt_max_m=12, scope="mission", constraint_type="soft")
    info = L.policy_alt_band(SimpleNamespace(constraints=[agl, msl, soft]))
    assert info == {"band": (5.0, 40.0), "envelopes": ["agl"], "skipped_msl": ["msl"]}, info
    assert L.alt_band_from_policy(SimpleNamespace(constraints=[agl, msl])) == (5.0, 40.0)
    only_msl = L.policy_alt_band(SimpleNamespace(constraints=[msl]))
    assert only_msl["band"] is None and only_msl["skipped_msl"] == ["msl"]
    assert L.combine_alt_bands(None, None) == (None, "none")
    assert L.combine_alt_bands((8, 20), None) == ((8.0, 20.0), "configured")
    assert L.combine_alt_bands(None, (5, 40)) == ((5.0, 40.0), "policy")
    assert L.combine_alt_bands((8, 50), (5, 40)) == ((8.0, 40.0), "intersection")
    assert L.combine_alt_bands((50, 60), (5, 40)) == ((5.0, 40.0), "policy_disjoint")


def test_speed_climb_and_yaw_caps_hold_on_every_tick():
    import random
    rng = random.Random(20261021)
    cfg = TrackerConfig(cruise_mps=1.5, speed_cap_mps=2.0)
    tr = StepTracker(cfg)
    veh = _Vehicle(up=20.0)
    t = 0.0
    for seq in range(1, 41):
        s = Step(rng.uniform(0, 5), rng.uniform(-5, 5), rng.uniform(-1.1, 1.1))
        tr.accept(Decision.of(seq, t, veh.pose(), s), t)
        for _, k in _fly(tr, veh, t, t + rng.uniform(0.3, 4.0)):
            a = k.action
            assert math.hypot(a.vx, a.vy) <= cfg.cruise_mps + 1e-9, a
            assert abs(a.vz_up) <= cfg.climb_cap_mps + 1e-9, a
            assert abs(a.yaw_rate) <= cfg.yaw_rate_cap_rad_s + 1e-9, a
            t = round(t + 0.1, 10)
    try:
        TrackerConfig(cruise_mps=3.0, speed_cap_mps=2.0)
    except ValueError:
        pass
    else:
        raise AssertionError("a cruise above the speed cap was accepted")


def test_a_step_completes_by_tolerance_or_by_its_time_cap():
    tr = StepTracker()
    veh = _Vehicle()
    d = Decision.of(1, 0.0, veh.pose(), Step(3.0, -1.0, 0.1))
    tr.accept(d, 0.0)
    ticks = _fly(tr, veh, 0.0, 9.0)
    done = [t for t, k in ticks if k.author == "hold"]
    assert done, "never completed"
    assert tr._cur.completion == "tolerance"
    assert all(k.author == "hold" and k.reason == "step_complete" for t, k in ticks
               if t >= done[0])
    assert all(k.action.vx == 0 and k.action.vy == 0 and k.action.yaw_rate == 0
               for t, k in ticks if t >= done[0])
    tgt = d.record["target"]
    assert math.hypot(veh.x - tgt["x"], veh.y - tgt["y"]) <= 0.35
    # A vehicle that cannot move: the cap ends the step, then it holds.
    stuck = StepTracker()
    d2 = Decision.of(2, 0.0, Pose(0, 0, 10, 0), Step(5.0, 0.0, 0.0))
    stuck.accept(d2, 0.0)
    cap = d2.record["target"]["time_cap_s"]
    assert abs(cap - (1.5 * 5.0 + 2.0)) < 1e-9, cap
    t = 0.0
    while stuck.tick(t, Pose(0, 0, 10, 0)).author == "vla_step":
        t = round(t + 0.1, 10)
        assert t < 20
    assert abs(t - cap) < 0.11 and stuck._cur.completion == "time_cap"


def test_the_tracker_never_flies_backwards_along_the_step():
    """A step that arrives after the aircraft already passed its endpoint (the
    image was captured 2 m back) counts as reached: AerialVLA's forward axis
    is 0..5 m, never negative, so the tracker does not invent a reverse."""
    tr = StepTracker()
    tr.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(1.0, 0.0, 0.0)), 1.0)
    tk = tr.tick(1.0, Pose(2.0, 0.1, 10.0, 0.0))
    assert tk.action.vx == 0.0 and tk.action.vy == 0.0, tk
    assert tk.author == "hold" and tr._cur.completion == "tolerance"


# --------------------------------------------------------------------------- #
# 3. LAND, malformed answers, timeouts, holds, authorship
# --------------------------------------------------------------------------- #
def test_land_stops_the_tracker_and_is_reported_as_a_flag():
    tr = StepTracker()
    tr.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(4.0, 0.0, 0.0)), 0.0)
    assert tr.tick(0.1, Pose(0, 0, 10, 0)).author == "vla_step"
    land = Decision.of(2, 0.2, Pose(0.1, 0, 10, 0), Step(0.0, 0.0, 0.0), land=True)
    assert tr.accept(land, 0.3)
    tk = tr.tick(0.4, Pose(0.1, 0, 10, 0))
    assert tk.land_requested and tk.author == "hold" and tk.reason == "land"
    assert tk.action == Action4D(), tk
    assert land.record["outcome"]["end"] == "land"
    late = Decision.of(3, 0.35, Pose(0.1, 0, 10, 0), Step(4.0, 0.0, 0.0))
    assert not tr.accept(late, 0.5) and late.record["refused"] == "after_land"
    for t in (1.0, 20.0, 60.0):                   # LAND never times out into motion
        assert tr.tick(t, Pose(0, 0, 10, 0)).action == Action4D()
    tr.clear_land()
    assert tr.accept(Decision.of(4, 60.0, Pose(0, 0, 10, 0), Step(1.0, 0.0, 0.0)), 60.0)
    assert tr.tick(60.1, Pose(0, 0, 10, 0)).author == "vla_step"


def test_a_malformed_answer_holds_zero_and_a_server_error_is_no_answer():
    """The brief's rule: parse_ok false means step null and the client HOLDS
    ZERO, counted. It replaces the step being flown: a misread answer is not
    a licence to keep flying the previous one.

    CHANGED BY DESIGN ON 2026-10-22 (review of 7c7cc50, P1): a server error
    (ok=false) was accepted the same way, as a `hold`. It is not an answer
    from the model, and every refusal reset the hold timer, so a server that
    refused every request (CUDA out of memory, or the server refusing an
    'unknown field' on every request) gave 299 `hold` ticks and 0 `hover`
    over 30 s: a dead model looked like a pilot holding on purpose. A server
    error is now refused (`refused: server_error`, counted `server_error`),
    ends the step being flown like a link failure, and the tracker hovers;
    before any decision it says `none` with the reason."""
    first = Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(4.0, -3.0, 0.0))
    tr2 = StepTracker()
    tr2.accept(first, 0.0)
    pf = Decision.of(2, 0.5, Pose(0.5, 0, 10.5, 0), kind="parse_fail")
    assert tr2.accept(pf, 0.6)
    assert first.record["outcome"]["end"] == "superseded"
    tk = tr2.tick(0.7, Pose(0.6, 0, 10.6, 0))
    assert tk.author == "hold" and tk.reason == "parse_fail" and tk.action == Action4D()
    se = Decision.of(3, 0.8, Pose(0.6, 0, 10.6, 0), kind="server_error")
    assert not tr2.accept(se, 0.9), "a server error was accepted as a decision"
    assert se.record["accepted"] is False and se.record["refused"] == "server_error"
    assert se.record["target"] is None and se.record["outcome"] is None
    assert pf.record["outcome"]["end"] == "server_error"
    for t in (1.0, 1.5, 3.0):
        tk = tr2.tick(t, Pose(0.6, 0, 10.6, 0))
        assert tk.author == "hover" and tk.reason == "server_error", tk
        assert tk.action == Action4D()
    assert tr2.counts["kind_parse_fail"] == 1 and tr2.counts["server_error"] == 1
    assert "kind_server_error" not in tr2.counts and tr2.counts["accepted"] == 2
    assert [r["seq"] for r in tr2.pop_finished()] == [1, 2, 3]
    # Before any decision: nothing flies, and the tick says why.
    fresh = StepTracker()
    assert not fresh.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), kind="server_error"), 0.1)
    tk = fresh.tick(0.2, Pose(0, 0, 10, 0))
    assert tk.author == "none" and tk.reason == "server_error", tk
    # With hover_on_link_loss off, a server error leaves the step being flown
    # alone, exactly as a link failure does (no new answer arrived).
    keep = StepTracker(TrackerConfig(hover_on_link_loss=False))
    keep.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(5.0, 0.0, 0.0)), 0.0)
    assert not keep.accept(Decision.of(2, 0.4, Pose(0, 0, 10, 0), kind="server_error"), 0.5)
    assert keep.tick(0.6, Pose(0.3, 0, 10, 0)).author == "vla_step"


def test_a_decision_past_the_timeout_is_never_flown():
    cfg = TrackerConfig(decision_timeout_s=4.0)
    # (a) refused on arrival when already too old
    tr = StepTracker(cfg)
    old = Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(4.0, 0.0, 0.0))
    assert not tr.accept(old, 4.5) and old.record["refused"] == "stale_on_arrival"
    assert tr.tick(4.6, Pose(0, 0, 10, 0)).author == "none"
    # (b) a step still being flown when its age passes the timeout: hover
    tr = StepTracker(cfg)
    d = Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(5.0, 0.0, 0.0))
    tr.accept(d, 1.0)
    assert tr.tick(3.9, Pose(0, 0, 10, 0)).author == "vla_step"   # held back: still flying
    tk = tr.tick(4.1, Pose(0, 0, 10, 0))
    assert tk.author == "hover" and tk.reason == "stale" and tk.action == Action4D()
    assert d.record["outcome"]["end"] == "stale" and not d.record["outcome"]["completed"]
    # (c) a hold lasts at most the timeout, then hover: the pilot went silent.
    # Measured from the hold's start, not the capture: in the stop-and-go
    # cadence the next image is taken only after the step completes.
    tr = StepTracker(cfg)
    veh = _Vehicle()
    d = Decision.of(1, 0.0, veh.pose(), Step(1.0, 0.0, 0.0))
    tr.accept(d, 0.0)
    ticks = _fly(tr, veh, 0.0, 10.0)
    authors = [k.author for _, k in ticks]
    t_done = min(t for t, k in ticks if k.author == "hold")
    t_hover = min(t for t, k in ticks if k.author == "hover")
    assert 4.0 < t_hover - t_done <= 4.11, (t_done, t_hover)
    assert t_hover > 4.0 + 1.0, "the capture-age rule would have hovered at 4.0 s"
    assert authors[-1] == "hover" and all(k.action == Action4D() for t, k in ticks
                                          if t >= t_hover)
    assert d.record["outcome"]["end"] == "hold_timeout" and d.record["outcome"]["completed"]
    assert ticks[-1][1].reason == "no_new_decision"
    zero = StepTracker(cfg)
    zero.accept(Decision.of(1, 0.0, veh.pose(), kind="parse_fail"), 1.0)
    assert zero.tick(4.9, veh.pose()).author == "hold"
    assert zero.tick(5.1, veh.pose()).author == "hover"
    # (d) a capture time in this clock's future is refused
    fut = Decision.of(2, 9.0, Pose(0, 0, 10, 0), Step(1.0, 0.0, 0.0))
    assert not tr.accept(fut, 8.0) and fut.record["refused"] == "future_capture"


def test_a_decision_captured_before_a_rail_hold_ended_is_thrown_away():
    """The FSM's Loiter (or a Shield hold) took the aircraft at t=2 and gave
    it back at t=5. The step being flown and any answer to an image from
    before t=5 describe a scene from before the hold: thrown away. The
    tracker hovers until an answer to a fresh image arrives."""
    tr = StepTracker()
    first = Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(5.0, 0.0, 0.0))
    tr.accept(first, 0.5)
    assert tr.tick(1.0, Pose(0.5, 0, 10, 0)).author == "vla_step"
    tr.suspend(2.0)
    assert first.record["outcome"]["end"] == "suspended"
    tk = tr.tick(2.5, Pose(1.5, 0, 10, 0))
    assert tk.author == "none" and tk.reason == "suspended" and tk.action == Action4D()
    during = Decision.of(2, 2.6, Pose(1.5, 0, 10, 0), Step(5.0, 0.0, 0.0))
    assert not tr.accept(during, 3.0) and during.record["refused"] == "suspended"
    tr.resume(5.0)
    tk = tr.tick(5.1, Pose(1.5, 0, 10, 0))
    assert tk.author == "hover" and tk.reason == "awaiting_fresh_decision"
    stale = Decision.of(3, 4.9, Pose(1.5, 0, 10, 0), Step(5.0, 0.0, 0.0))
    assert not tr.accept(stale, 5.2) and stale.record["refused"] == "invalidated"
    assert tr.tick(5.3, Pose(1.5, 0, 10, 0)).author == "hover"
    fresh = Decision.of(4, 5.15, Pose(1.5, 0, 10, 0), Step(5.0, 0.0, 0.0))
    assert tr.accept(fresh, 5.4)
    assert tr.tick(5.5, Pose(1.5, 0, 10, 0)).author == "vla_step"


def test_a_hand_over_ramps_from_the_command_the_rail_flew():
    """note_flown (demo/vla_dagger.py, 2026-10-24): the rail flew its own
    command (the expert, in a DAgger recording). With smoothing the next tick
    ramps from THAT command, not from the tracker's last output or from rest;
    the decision, its targets and timers are untouched; without smoothing it
    changes nothing. Shown failing first: StepTracker had no note_flown."""
    cfg = TrackerConfig(cruise_mps=4.5, speed_cap_mps=5.0, turn_while_moving=True,
                        smooth_accel_mps2=2.5, smooth_yaw_accel_rps2=0.6, carry_s=1.0)
    flown = Action4D(vx=4.0, vy=0.0, vz_up=0.0, yaw_rate=0.2)
    rest, handed = StepTracker(cfg), StepTracker(cfg)
    for tr in (rest, handed):
        assert tr.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(5.0, 0.0, 0.0)), 0.0)
    handed.note_flown(flown, 0.0)
    a, b = rest.tick(0.1, Pose(0, 0, 10, 0)), handed.tick(0.1, Pose(0, 0, 10, 0))
    # Toward 4.5 m/s (min(cruise, sqrt(2 a d)) = min(4.5, 5.0)) at 2.5 m/s^2:
    # from rest 0.25, from the flown 4.0 m/s 4.25; the yaw rate from 0.2 toward
    # the heading's 0 at 0.6 rad/s^2: 0.14.
    assert math.isclose(a.action.vx, 0.25) and math.isclose(b.action.vx, 4.25), (a, b)
    assert a.action.yaw_rate == 0.0 and math.isclose(b.action.yaw_rate, 0.14), b
    assert b.author == "vla_step" and handed.current_seq == 1
    assert handed.counts["rail_flown"] == 1 and "rail_flown" not in rest.counts
    plain = StepTracker(TrackerConfig())
    plain.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(5.0, 0.0, 0.0)), 0.0)
    plain.note_flown(flown, 0.0)
    ref = StepTracker(TrackerConfig())
    ref.accept(Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(5.0, 0.0, 0.0)), 0.0)
    assert plain.tick(0.1, Pose(0, 0, 10, 0)) == ref.tick(0.1, Pose(0, 0, 10, 0))
    assert "rail_flown" not in plain.counts


def test_every_tick_names_its_author():
    tr = StepTracker(TrackerConfig(decision_timeout_s=6.0))
    veh = _Vehicle()
    seen = []
    seen += [k for _, k in _fly(tr, veh, 0.0, 0.5)]              # nothing yet
    tr.accept(Decision.of(1, 0.4, veh.pose(), Step(1.0, 0.0, 0.0)), 0.5)
    seen += [k for _, k in _fly(tr, veh, 0.5, 12.0)]
    authors = [k.author for k in seen]
    assert set(authors) <= set(AUTHORS)
    order = [a for i, a in enumerate(authors) if i == 0 or authors[i - 1] != a]
    assert order == ["none", "vla_step", "hold", "hover"], order
    rows = [k.as_row() for k in seen]
    assert rows[0] == {"pilot_author": "none", "pilot_reason": "no_decision",
                       "vla_seq": None, "vla_age_s": None, "land_requested": False}
    assert tr.ticks["none"] == 5 and sum(tr.ticks.values()) == len(seen)


def test_a_lost_link_hovers_unless_told_otherwise():
    tr = StepTracker()
    tr.link_lost(0.0, "refused")
    assert tr.tick(0.1, Pose(0, 0, 10, 0)).author == "none"     # never started
    d = Decision.of(1, 0.2, Pose(0, 0, 10, 0), Step(5.0, 0.0, 0.0))
    tr.accept(d, 0.3)
    tr.link_lost(0.5, "timeout")
    tk = tr.tick(0.6, Pose(0.3, 0, 10, 0))
    assert tk.author == "hover" and tk.reason == "link_lost"
    assert d.record["outcome"]["end"] == "link_lost"
    keep = StepTracker(TrackerConfig(hover_on_link_loss=False))
    keep.accept(Decision.of(1, 0.2, Pose(0, 0, 10, 0), Step(5.0, 0.0, 0.0)), 0.3)
    keep.link_lost(0.5, "timeout")
    assert keep.tick(0.6, Pose(0.3, 0, 10, 0)).author == "vla_step"


def test_the_step_log_writes_off_the_callers_thread_and_never_raises():
    """Review of 7c7cc50, P4: vla_steps.jsonl was written and flushed on the
    10 Hz loop thread from inside act(), and an OSError went straight out of
    act() (a full disk stopped the flight loop at tick 2 in the reviewer's
    repro; D: is known to run near full). StepLog.write now only queues the
    record: a writer thread serializes and writes it, and a failure (OSError,
    or a record that is not JSON) is counted in `errors`, never raised.
    close() drains the queue first, so every line written still lands."""
    import tempfile
    import threading
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "vla_steps.jsonl"
        log = StepLog(path)
        try:
            seen = []
            real = log._fh.write

            def spy(s):
                seen.append(threading.current_thread())
                return real(s)
            log._fh.write = spy
            log.write({"seq": 1})
            assert log.flush(3.0)
            assert seen and all(t is not threading.current_thread() for t in seen), seen

            def full(s):
                raise OSError(28, "No space left on device")
            log._fh.write = full
            for i in range(5):
                log.write({"seq": 2 + i})             # never raises
            assert log.flush(3.0)
            assert log.errors == 5 and "No space left" in log.last_error, log.last_error
            log._fh.write = real
            log.write({"seq": 8, "bad": float("nan")})  # not JSON: counted, not raised
            log.write({"seq": 9})
            assert log.flush(3.0)
            assert log.errors == 6 and log.n == 2, (log.errors, log.n)
        finally:
            log.close()
        log.write({"seq": 10})                        # after close: counted, not raised
        assert log.errors == 7
        lines = path.read_bytes().decode("utf-8").splitlines()
        assert [json.loads(x)["seq"] for x in lines] == [1, 9], lines


def test_the_record_says_what_became_of_each_decision_and_the_log_is_lf_jsonl():
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        _check_records(Path(tmp) / "run" / "vla_steps.jsonl")


def _check_records(path: Path) -> None:
    """Two decisions, the first superseded mid-step, the second completed
    and still held at close; the log is one LF-terminated JSON line each."""
    tr = StepTracker()
    veh = _Vehicle()
    tr.accept(Decision.of(1, 0.0, veh.pose(), Step(4.0, 0.0, 0.0)), 0.1)
    _fly(tr, veh, 0.1, 1.0)
    tr.accept(Decision.of(2, 0.9, veh.pose(), Step(1.0, 0.0, 0.0)), 1.0)
    _fly(tr, veh, 1.0, 5.0)
    tr.close(5.0)
    log = StepLog(path)
    for rec in tr.pop_finished():
        log.write(rec)
    log.close()
    raw = path.read_bytes()
    assert b"\r\n" not in raw and raw.endswith(b"\n")
    recs = [json.loads(x) for x in raw.decode("utf-8").splitlines()]
    assert [r["seq"] for r in recs] == [1, 2]
    assert recs[0]["outcome"]["end"] == "superseded" and not recs[0]["outcome"]["completed"]
    assert recs[0]["outcome"]["ticks"]["vla_step"] == 9
    assert recs[1]["outcome"]["completed"] and recs[1]["outcome"]["completion"] == "tolerance"
    assert recs[1]["outcome"]["end"] == "closed"
    assert all(r["schema"] == L.VLA_STEPS_SCHEMA and r["accepted"] for r in recs)


# --------------------------------------------------------------------------- #
# 4. The client against a scripted server
# --------------------------------------------------------------------------- #
def _client(srv, **kw):
    kw.setdefault("response_timeout_s", 2.0)
    kw.setdefault("reconnect_backoff_s", 0.0)
    return RemoteVLA("127.0.0.1", srv.port, **kw)


def test_the_client_round_trip_with_a_scripted_server():
    with F.FakeVLAServer(lambda req: F.step_reply(req, 3.3, -1.2, 0.15)) as srv:
        c = _client(srv)
        try:
            ident = c.connect()
            assert ident["id"] == F.MODEL["id"] and srv.hellos == 1
            fields, meta = _fields(2.0, Pose(1.0, 2.0, 12.0, 0.3))
            seq = c.submit(fields)
            assert seq == 1 and c.wait_idle(3.0)
            res = c.poll()
            assert res is not None and res.failure is None and c.poll() is None
            rep = res.reply
            assert rep.kind == "step" and rep.parse_ok and not rep.land
            want = [F.dequantize(F.quantize(v, *F.RANGES[k]), *F.RANGES[k])
                    for k, v in (("fwd_m", 3.3), ("down_m", -1.2), ("dpsi_rad", 0.15))]
            assert [rep.step.fwd_m, rep.step.down_m, rep.step.dpsi_rad] == want
            dec = L.decision_from_result(meta, res)
            assert dec.kind == "step" and dec.pose == Pose(1.0, 2.0, 12.0, 0.3)
            rec = dec.record
            assert rec["phrase"] == "straight ahead " and rec["hint_source"] == "goal"
            assert rec["bins"] == F.step_reply(srv.requests[0], 3.3, -1.2, 0.15)["bins"]
            assert rec["model"]["id"] == F.MODEL["id"] and rec["latency_server_s"] == 0.01
            assert rec["latency_client_s"] == res.t_done - 2.0
            got = srv.requests[0]
            assert set(got) == set(L.INFER_FIELDS) and srv.refused == []
            assert got["seq"] == 1 and got["phrase"] == "straight ahead "
            seq2 = c.submit(_fields(2.5)[0])
            assert seq2 == 2 and c.wait_idle(3.0) and c.poll().reply.kind == "step"
            assert srv.connections == 1, "one connection carries many requests"
        finally:
            c.close()


def test_one_request_outstanding_and_submit_never_blocks():
    with F.FakeVLAServer(lambda req: F.Delay(0.6, F.step_reply(req, 1.0, 0.0, 0.0))) as srv:
        c = _client(srv)
        try:
            t0 = time.monotonic()
            assert c.submit(_fields(0.0)[0]) == 1
            assert time.monotonic() - t0 < 0.2, "submit blocked"
            assert c.busy and c.submit(_fields(0.1)[0]) is None
            assert c.poll() is None
            assert c.wait_idle(3.0)
            assert c.poll().reply.kind == "step"
            assert c.submit(_fields(0.7)[0]) == 2 and c.wait_idle(3.0)
            assert srv.overlaps == 0 and len(srv.requests) == 2
            assert c.counts["submit_busy"] == 1
        finally:
            c.close()


def test_a_finished_result_waits_until_it_is_polled():
    """Review of 7c7cc50, P2: submit() checked only that no request was in
    flight, so a second request could go out while the first one's result
    sat unread, and the worker then overwrote it: poll() returned seq 2 and
    seq 1 (perhaps a step, perhaps LAND) was gone. In act() the window is
    between _drain()'s poll() and _maybe_request()'s busy check, and it bites
    whenever the next answer comes back within one tick (an instant refusal,
    a server that dies). A finished, unread result now counts as busy."""
    with F.FakeVLAServer(lambda req: F.step_reply(req, 3.0, 0.0, 0.0)) as srv:
        c = _client(srv)
        try:
            assert c.submit(_fields(0.0)[0]) == 1 and c.wait_idle(3.0)
            assert c.busy, "a finished, unread result must count as busy"
            assert c.submit(_fields(0.1)[0]) is None
            assert c.counts["submit_unpolled"] == 1 and len(srv.requests) == 1
            res = c.poll()
            assert res is not None and res.seq == 1 and c.poll() is None, res
            assert not c.busy and c.submit(_fields(0.2)[0]) == 2 and c.wait_idle(3.0)
            assert c.poll().seq == 2 and len(srv.requests) == 2
        finally:
            c.close()


def test_the_worker_survives_any_exception_and_always_posts_a_result():
    """Review of 7c7cc50, P5: the worker caught only the link, protocol, OS
    and value errors. Anything else killed the thread with `_busy` still set:
    every later submit() returned None, no failure was recorded and the
    request got no line until close(). A reply nested 200,000 deep (a
    RecursionError in json) did it in the reviewer's repro. That reply is now
    a protocol failure, anything else unexpected is failure "internal", and
    either way the result is posted, the connection dropped and the worker
    keeps serving."""
    nested = b"[" * 200000 + b"]" * 200000 + b"\n"
    with F.FakeVLAServer(lambda req: nested) as srv:
        c = _client(srv)
        try:
            assert c.submit(_fields(0.0)[0]) == 1
            assert c.wait_idle(5.0), "the worker died with the request outstanding"
            res = c.poll()
            assert res is not None and res.failure == "protocol" and res.reply is None, res
            assert c._worker.is_alive()
            assert c.submit(_fields(1.0)[0]) == 2 and c.wait_idle(5.0)
            assert c.poll().failure == "protocol"
        finally:
            c.close()
    with F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0)) as srv:
        c = _client(srv)
        real = L.parse_infer_response

        def boom(*a, **k):
            raise RuntimeError("a bug in the reply reader")
        L.parse_infer_response = boom
        try:
            assert c.submit(_fields(0.0)[0]) == 1 and c.wait_idle(3.0)
            res = c.poll()
            assert res.failure == "internal" and "RuntimeError" in res.detail, res
            assert c.counts["fail_internal"] == 1 and c._worker.is_alive()
        finally:
            L.parse_infer_response = real
        try:
            assert c.submit(_fields(1.0)[0]) == 2 and c.wait_idle(3.0)
            assert c.poll().reply.kind == "step"
            assert srv.connections == 2, "the failure did not drop the connection"
        finally:
            c.close()


def test_a_server_error_backs_off_without_dropping_the_connection():
    """Review of 7c7cc50, P1 (the client half): an ok=false reply set no
    backoff, so a server refusing every request was asked again on every
    tick (300 requests in 30 s). It now backs off like a failure, on the
    same connection (the stream is still in step), and is counted."""
    clock = {"t": 0.0}
    with F.FakeVLAServer(lambda req: F.error_reply(req, "CUDA out of memory")) as srv:
        c = _client(srv, reconnect_backoff_s=1.0, clock=lambda: clock["t"])
        try:
            assert c.submit(_fields(0.0)[0]) == 1 and c.wait_idle(3.0)
            res = c.poll()
            assert res.failure is None and res.reply.kind == "server_error"
            assert c.counts["server_error"] == 1
            assert c.backing_off and c.submit(_fields(0.1)[0]) is None
            assert c.counts["submit_backoff"] == 1
            clock["t"] = 1.01
            assert c.submit(_fields(1.01)[0]) == 2 and c.wait_idle(3.0)
            assert c.poll().reply.kind == "server_error"
            assert srv.connections == 1 and c.counts["connects"] == 1
        finally:
            c.close()


def test_a_timeout_fails_cleanly_and_the_next_request_reconnects():
    calls = {"n": 0}

    def respond(req):
        calls["n"] += 1
        if calls["n"] == 1:
            return F.Delay(1.0, F.step_reply(req, 1.0, 0.0, 0.0))
        return F.step_reply(req, 2.0, 0.0, 0.0)
    with F.FakeVLAServer(respond) as srv:
        c = _client(srv, response_timeout_s=0.3, reconnect_backoff_s=0.0)
        try:
            c.submit(_fields(0.0)[0])
            assert c.wait_idle(3.0)
            res = c.poll()
            assert res.failure == "timeout" and res.reply is None, res
            assert c.counts["fail_timeout"] == 1
            assert c.submit(_fields(1.0)[0]) == 2 and c.wait_idle(3.0)
            res = c.poll()
            assert res.failure is None and res.reply.step.fwd_m > 1.9
            assert srv.connections == 2 and srv.hellos == 2
        finally:
            c.close()
    # Backoff: after a failure the client refuses to resubmit for a while.
    with F.FakeVLAServer(lambda req: F.CLOSE) as srv:
        clock = {"t": 0.0}
        c = _client(srv, reconnect_backoff_s=1.0, clock=lambda: clock["t"])
        try:
            c.submit(_fields(0.0)[0])
            assert c.wait_idle(3.0) and c.poll().failure == "link"
            assert c.backing_off and c.submit(_fields(0.1)[0]) is None
            clock["t"] = 1.5
            assert not c.backing_off and c.submit(_fields(1.5)[0]) == 2
            c.wait_idle(3.0)
        finally:
            c.close()


def test_a_reply_that_breaks_the_protocol_is_never_a_decision():
    def bad(req, **over):
        r = F.step_reply(req, 1.0, 0.0, 0.0)
        r.update(over)
        return r
    cases = {
        "wrong seq": lambda req: bad(req, seq=req["seq"] + 1),
        "seq null on an answer": lambda req: bad(req, seq=None),
        "seq true": lambda req: bad(req, seq=True),         # True == 1 in Python
        "seq float": lambda req: bad(req, seq=float(req["seq"])),
        "fwd out of range": lambda req: bad(req, step={"fwd_m": 6.0, "down_m": 0.0,
                                                       "dpsi_rad": 0.0}),
        "malformed but moving": lambda req: bad(req, parse_ok=False),
        "malformed LAND": lambda req: bad(req, parse_ok=False, step=None, land=True),
        "pose echo": lambda req: bad(req, pose=dict(req["pose"], x=req["pose"]["x"] + 1)),
        "t_capture echo": lambda req: bad(req, t_capture=req["t_capture"] + 0.5),
        "missing field": lambda req: {k: v for k, v in bad(req).items() if k != "image_sha256"},
        "version": lambda req: bad(req, v=1),
        "bins": lambda req: bad(req, bins=[1, 2, 99]),
        "not json": lambda req: b"00 49 49 LAND\n",
        "NaN": lambda req: json.dumps(bad(req)).replace('"latency_s": 0.01',
                                                         '"latency_s": NaN').encode() + b"\n",
        # Review of 7c7cc50, P3: the client trusted the server's parse_ok. A
        # server still running the old ints[-3:] parser (the brief's headline
        # bug) would be flown. parse_ok true now needs raw_text to BE the
        # answer: three 2-digit bins equal to `bins`, " LAND" exactly when
        # land is true, and a step within half a bin of the bins' value.
        "four numbers (ints[-3:])": lambda req: bad(req, raw_text="00 19 49 49"),
        "LAND in the text, land false": lambda req: bad(req, raw_text="19 49 49 LAND"),
        "land true, no LAND in the text": lambda req: bad(req, land=True),
        "three-digit bin": lambda req: bad(req, raw_text="19 49 049"),
        "text around the answer": lambda req: bad(req, raw_text=" 19 49 49"),
        "bins are not the text's": lambda req: bad(req, raw_text="20 49 49"),
        "step is not the bins'": lambda req: bad(req, step={"fwd_m": 4.0, "down_m": 0.0,
                                                            "dpsi_rad": 0.0}),
        "step off by a bin": lambda req: bad(req, step={"fwd_m": F.dequantize(20, 0.0, 5.0),
                                                        "down_m": 0.0, "dpsi_rad": 0.0}),
        "non-ASCII digit": lambda req: bad(req, raw_text="19 49 4٩"),
    }
    for name, respond in cases.items():
        with F.FakeVLAServer(respond) as srv:
            c = _client(srv)
            try:
                c.submit(_fields(0.0)[0])
                assert c.wait_idle(3.0), name
                res = c.poll()
                assert res.failure == "protocol" and res.reply is None, (name, res)
            finally:
                c.close()
    # The answer as the server decodes it: with special tokens, so the text
    # may end in the EOS marker; LAND exactly when the text says it.
    for name, respond, kind in (
            ("EOS", lambda req: dict(F.step_reply(req, 1.0, 0.0, 0.0),
                                     raw_text="19 49 49</s>"), "step"),
            ("LAND", lambda req: F.land_reply(req), "land"),
            ("LAND + EOS", lambda req: dict(F.land_reply(req), raw_text="00 49 49 LAND</s>"),
             "land")):
        with F.FakeVLAServer(respond) as srv:
            c = _client(srv)
            try:
                c.submit(_fields(0.0)[0])
                assert c.wait_idle(3.0), name
                res = c.poll()
                assert res.failure is None and res.reply.kind == kind, (name, res)
            finally:
                c.close()
    # The malformed answer the real server sends as such is a hold, not a failure.
    with F.FakeVLAServer(lambda req: F.parse_fail_reply(req)) as srv:
        c = _client(srv)
        try:
            c.submit(_fields(0.0)[0])
            c.wait_idle(3.0)
            res = c.poll()
            assert res.failure is None and res.reply.kind == "parse_fail"
            assert res.reply.step is None and res.reply.raw_text == "00 49 494 49"
        finally:
            c.close()
    with F.FakeVLAServer(lambda req: F.error_reply(req, "CUDA out of memory")) as srv:
        c = _client(srv)
        try:
            c.submit(_fields(0.0)[0])
            c.wait_idle(3.0)
            res = c.poll()
            assert res.failure is None and res.reply.kind == "server_error"
            assert res.reply.error == "CUDA out of memory"
        finally:
            c.close()
    # A refusal the server could not attach a seq to is still a refusal.
    with F.FakeVLAServer(lambda req: dict(F.error_reply(req, "bad request"), seq=None)) as srv:
        c = _client(srv)
        try:
            c.submit(_fields(0.0)[0])
            c.wait_idle(3.0)
            res = c.poll()
            assert res.failure is None and res.reply.kind == "server_error", res
        finally:
            c.close()


def test_a_closed_or_absent_server_is_a_link_failure():
    with F.FakeVLAServer(lambda req: F.CLOSE) as srv:
        c = _client(srv)
        try:
            c.submit(_fields(0.0)[0])
            assert c.wait_idle(3.0)
            res = c.poll()
            assert res.failure == "link" and "closed" in res.detail, res
        finally:
            c.close()
    srv = F.FakeVLAServer(lambda req: None)
    srv.start()
    port = srv.port
    srv.stop()                                    # nothing listens there now
    c = RemoteVLA("127.0.0.1", port, connect_timeout_s=0.5, reconnect_backoff_s=0.0)
    try:
        t0 = time.monotonic()
        c.submit(_fields(0.0)[0])
        assert c.wait_idle(3.0)
        res = c.poll()
        assert res.failure == "link" and "connect" in res.detail, res
        assert time.monotonic() - t0 < 2.5
    finally:
        c.close()


def test_the_model_identity_is_checked_and_may_not_change_mid_run():
    """Review of 7c7cc50, P9: hello accepted any object as the model ({}
    included) and a reply's model was never compared with it. The dashboard
    classifies VLA runs by the weights hash, so an identity without one, or
    a model swapped mid-run by a server restart, passed unnoticed. hello
    must now carry the brief's identity (id, base, weights_sha256 as 64 hex
    digits, precision nf4|bf16, attn eager|sdpa, merged true|false; extra
    serving facts are kept); every answer's model must be that identity, and
    so must the hello on any reconnect. A scripted stand-in (no weights) is
    accepted only with strict_identity=False, said out loud by the caller."""
    good = dict(F.MODEL)
    step = (lambda req: F.step_reply(req, 1.0, 0.0, 0.0))
    for name, model in (("empty", {}),
                        ("no digest", {k: v for k, v in good.items() if k != "weights_sha256"}),
                        ("short digest", dict(good, weights_sha256="abc")),
                        ("upper-case digest", dict(good, weights_sha256="A" * 64)),
                        ("precision", dict(good, precision="fp8")),
                        ("attn", dict(good, attn="flash")),
                        ("merged not a bool", dict(good, merged="no")),
                        ("empty id", dict(good, id="")),
                        ("no base", {k: v for k, v in good.items() if k != "base"})):
        with F.FakeVLAServer(step, model=model) as srv:
            c = _client(srv)
            try:
                c.submit(_fields(0.0)[0])
                assert c.wait_idle(3.0), name
                res = c.poll()
                assert res.failure == "protocol" and "identity" in res.detail, (name, res)
                assert srv.requests == [] and c.identity is None, name
            finally:
                c.close()
    # int4 (torchao, DroneCATS on the Orin Nano) is an identity's precision;
    # fp8 above is still refused.
    from guardrail.vla_link import check_identity
    check_identity(dict(good, precision="int4"))
    # fp16: AIOT-VLA's frozen OWL-ViT backbone (training/aiotvla, 29 Oct)
    check_identity(dict(good, precision="fp16"))
    # Serving facts beside the identity are kept; answers are compared on the
    # identity's own fields.
    with F.FakeVLAServer(step, model=dict(good, serving={"max_new_tokens": 12})) as srv:
        c = _client(srv)
        try:
            assert c.connect()["serving"] == {"max_new_tokens": 12}
            c.submit(_fields(0.0)[0])
            assert c.wait_idle(3.0) and c.poll().reply.kind == "step"
        finally:
            c.close()
    # A scripted stand-in, on purpose.
    fake = {"id": "fake:scripted", "base": "none", "weights_sha256": None,
            "precision": "fake", "attn": "none", "merged": False}
    with F.FakeVLAServer(lambda req: dict(step(req), model=dict(fake)), model=fake) as srv:
        c = _client(srv, strict_identity=False)
        try:
            c.submit(_fields(0.0)[0])
            assert c.wait_idle(3.0) and c.poll().reply.kind == "step"
        finally:
            c.close()
    # An answer from another model than the hello's.
    other = dict(good, weights_sha256="1" * 64)
    with F.FakeVLAServer(lambda req: dict(step(req), model=dict(other))) as srv:
        c = _client(srv)
        try:
            c.submit(_fields(0.0)[0])
            assert c.wait_idle(3.0)
            res = c.poll()
            assert res.failure == "protocol" and "model changed" in res.detail, res
            assert c.counts["model_changed"] == 1
        finally:
            c.close()
    # A restart with other weights: the reconnect's hello is refused, and
    # nothing more is sent.
    calls = {"n": 0}

    def respond(req):
        calls["n"] += 1
        return F.CLOSE if calls["n"] == 1 else step(req)
    with F.FakeVLAServer(respond) as srv:
        c = _client(srv)
        try:
            c.submit(_fields(0.0)[0])
            assert c.wait_idle(3.0) and c.poll().failure == "link"
            srv.model = dict(good, weights_sha256="2" * 64)
            c.submit(_fields(1.0)[0])
            assert c.wait_idle(3.0)
            res = c.poll()
            assert res.failure == "protocol" and "model changed" in res.detail, res
            assert len(srv.requests) == 1 and c.counts["model_changed"] == 1
            assert c.identity["weights_sha256"] == "0" * 64
        finally:
            c.close()


# --------------------------------------------------------------------------- #
# 5. Closed loop: a point vehicle reaches a goal from scripted steps
# --------------------------------------------------------------------------- #
def _scripted_step(pose: Pose, phrase: str, goal, goal_up):
    """What a perfect AerialVLA would answer from this capture
    (vla_fake_server.scripted_step), quantized to the model's 99 bins with
    int truncation as the server decodes it; None for LAND."""
    s = F.scripted_step(pose.as_wire(), phrase, goal, goal_up)
    if s is None:
        return None
    q = [F.dequantize(F.quantize(v, *F.RANGES[k]), *F.RANGES[k])
         for k, v in zip(("fwd_m", "down_m", "dpsi_rad"), s)]
    return Step(*q)


def _closed_loop(latency_s, cadence, goal=(30.0, -20.0), goal_up=18.0, t_max=150.0):
    tr = StepTracker(TrackerConfig(alt_band=(6.0, 30.0)))
    veh = _Vehicle(x=0.0, y=0.0, up=12.0, yaw=0.0)
    hint = L.goal_hint(goal)
    pending = None                                # (t_ready, decision)
    seq, t, n = 0, 0.0, 0
    ticks = []
    while t < t_max and not tr.land_requested:
        if pending is not None and t >= pending[0] - 1e-9:
            tr.accept(pending[1], t)
            pending = None
            n += 1
        idle = tr.idle_since
        want = pending is None and (cadence == "pipelined" or
                                    (idle is not None and t - idle >= L.UPSTREAM_SETTLE_S))
        if want:
            seq += 1
            cap = veh.pose()
            h = hint(Capture(t, cap, JPG))
            s = _scripted_step(cap, h.phrase, goal, goal_up)
            d = (Decision.of(seq, t, cap, Step(0.0, 0.0, 0.0), land=True) if s is None
                 else Decision.of(seq, t, cap, s))
            pending = (t + latency_s, d)
        tk = tr.tick(t, veh.pose())
        veh.step(tk.action)
        ticks.append(tk)
        t = round(t + 0.1, 10)
    return tr, veh, ticks, n, t


def test_a_point_vehicle_reaches_the_goal_from_scripted_steps():
    """36 m away, 6 m up, behind-left of the nose (bearing -33.7 deg, so the
    first steps rotate only). The vehicle follows only the tracker's 10 Hz
    actions; every step is the scripted model's, quantized to 99 bins; the
    goal phrase turns "" within 0.5 m and the scripted model answers LAND.
    Pipelined at 0.4 s and 3 s of latency, and upstream's stop-and-go."""
    goal, goal_up = (30.0, -20.0), 18.0
    for latency, cadence in ((0.4, "pipelined"), (3.0, "pipelined"),
                             (0.4, "stop_and_go"), (3.0, "stop_and_go")):
        tr, veh, ticks, n, t = _closed_loop(latency, cadence, goal, goal_up)
        assert tr.land_requested, (latency, cadence, t, veh.pose())
        miss = math.hypot(veh.x - goal[0], veh.y - goal[1])
        assert miss < 0.8, (latency, cadence, miss)
        assert abs(veh.up - goal_up) < 0.5, (latency, cadence, veh.up)
        authors = {k.author for k in ticks}
        assert authors <= {"none", "vla_step", "hold"}, (latency, cadence, authors)
        flown = sum(1 for k in ticks if k.author == "vla_step")
        assert flown / len(ticks) > 0.5, (latency, cadence, flown, len(ticks))
        assert tr.counts.get("refused_stale_on_arrival", 0) == 0
        assert not any(k.startswith("end_stale") or k == "end_hold_timeout"
                       for k in tr.counts), (latency, cadence, tr.counts)
        assert t < 120.0, (latency, cadence, t)
        recs = [r for r in tr.pop_finished() if r.get("outcome")]
        cut = [r for r in recs if r["outcome"]["end"] == "superseded"
               and not r["outcome"]["completed"]]
        if cadence == "stop_and_go":
            # Upstream's cadence: every step is flown to completion first.
            assert not cut and n == len(recs), (latency, cadence, len(cut))
        else:
            # Pipelined: newer steps take over steps still in progress.
            assert cut, (latency, cadence, tr.counts)


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
        except Exception as e:                        # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
