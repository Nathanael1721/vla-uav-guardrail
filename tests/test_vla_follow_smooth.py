"""The VLA follow's smoothness: the step tracker's opt-in smoothing
(guardrail/vla_link.TrackerConfig, SMOOTHING), the follow's close hold through
it (demo/vla_mission.MissionLoop), the attitude on vla_fly's rows, its flags,
and tools/compare_motion.py (the hand-written follow's definitions).

Run either way:
    pytest tests/test_vla_follow_smooth.py -v
    python tests/test_vla_follow_smooth.py

WHAT IS BEING CLAIMED

1. Smoothing is OFF by default: a TrackerConfig without it flies exactly as
   before (the same actions, tick for tick, on a scripted follow), and every
   mission but a follow refuses --smooth.
2. With it, the commanded horizontal velocity changes by at most
   smooth_accel_mps2 x dt per tick (as a vector, its direction kept; a late
   tick buys no burst), the yaw rate by at most smooth_yaw_accel_rps2 x dt,
   and the heading still settles on the step's target without overshoot.
3. A step flown to its end while an answer is pending carries its velocity,
   decaying linearly to zero over carry_s (author vla_step, reason carry,
   counted); with nothing pending it holds as before. A hover, a hold and
   the follow's close hold ramp down instead of stopping in one tick.
4. Closed loop (a point vehicle chasing a car that turns a corner, answers
   0.8 s after capture, pipelined, the follow executor): smoothing takes the
   commanded acceleration p99 from far above to at most 2.5 m/s^2 and the
   yaw acceleration p99 to at most 0.6 rad/s^2 (the targets set from follow
   #9 against the hand-written follow) without losing the chase.
5. vla_fly's rows carry pitch_deg / roll_deg (follow_vlm's quaternion
   reading), metrics carry "motion" on compare_motion's definitions, and
   compare_motion says when an attitude was never recorded instead of
   reading its absence as 0 tilt.

Shown failing first: at 47fccc0 TrackerConfig had no smooth_accel_mps2,
smooth_yaw_accel_rps2 or carry_s (TypeError), StepTracker.tick took no
pending / hold_horizontal, MissionLoop zeroed a close box's velocity in one
tick, vla_fly had no --smooth, attitude_fields or motion_record, and
tools/compare_motion.py did not exist. No simulator, model or GPU.
"""
from __future__ import annotations

import json
import math
import statistics
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools"):
    sys.path.insert(0, str(_p))

from guardrail import aerialvla_codec as C                           # noqa: E402
from guardrail.models import Action4D, State, load_policy            # noqa: E402
from guardrail.vla_link import (Decision, Pose, Step, StepTracker,   # noqa: E402
                                TrackerConfig, wrap_pi)

SKIP = "SKIP"
POLICY = ROOT / "policies" / "vla_goto_citylife.yaml"
SMOOTH = dict(smooth_accel_mps2=2.5, smooth_yaw_accel_rps2=0.6, carry_s=1.0)
FOLLOW = dict(cruise_mps=4.5, speed_cap_mps=5.0, turn_while_moving=True, anchor_at_answer=True,
              land_as_arrival=True, hold_altitude_m=10.0)
EPS = 1e-9


def _step_of(fwd: float, dpsi: float, down: float = 0.0) -> Step:
    """A step as the model can say it: quantized to its 99 bins (the codec's
    truncation) and decoded at the bin's lower edge, as the server does."""
    b = (C.quantize(fwd, "fwd_m"), C.quantize(down, "down_m"), C.quantize(dpsi, "dpsi_rad"))
    s = C.bins_to_step(b)
    return Step(s.fwd_m, s.down_m, s.dpsi_rad)


class _Veh:
    """A point multirotor: velocity lag 0.3 s, yaw follows the yaw rate."""

    def __init__(self, x=0.0, y=0.0, up=10.0, yaw=0.0):
        self.x, self.y, self.up, self.yaw = x, y, up, yaw
        self.v = [0.0, 0.0, 0.0]

    def pose(self) -> Pose:
        return Pose(self.x, self.y, self.up, self.yaw)

    def step(self, a: Action4D, dt: float = 0.1) -> None:
        k = min(1.0, dt / 0.3)
        for i, c in enumerate((a.vx, a.vy, a.vz_up)):
            self.v[i] += k * (c - self.v[i])
        self.x += self.v[0] * dt
        self.y += self.v[1] * dt
        self.up += self.v[2] * dt
        self.yaw = wrap_pi(self.yaw + a.yaw_rate * dt)


def _row(t: float, veh: _Veh, a: Action4D, author: str = "vla_step") -> dict:
    return {"t": round(t, 3), "x": veh.x, "y": veh.y, "up": veh.up,
            "emitted": a.model_dump(), "author": author}


# ------------------------------------------------------------------ 1. off by default

def test_smoothing_is_off_by_default_and_validated():
    cfg = TrackerConfig()
    assert not cfg.smoothing and cfg.smooth_accel_mps2 is None
    assert cfg.smooth_yaw_accel_rps2 is None and cfg.carry_s == 0.0
    assert TrackerConfig(carry_s=0.5).smoothing and TrackerConfig(smooth_accel_mps2=1).smoothing
    for bad in ({"smooth_accel_mps2": 0.0}, {"smooth_accel_mps2": -1.0},
                {"smooth_yaw_accel_rps2": 0.0}, {"smooth_accel_mps2": float("nan")},
                {"carry_s": -0.1}, {"carry_s": float("inf")}, {"smooth_tick_s": 0.0}):
        try:
            TrackerConfig(**bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")


def _scripted_follow(cfg: TrackerConfig, seconds: float = 30.0, latency: float = 0.8,
                     car=None, standoff: float = 12.0):
    """A pipelined follow of a moving car through the real StepTracker: each
    answer is computed at its capture from the car's bearing and distance,
    quantized to the model's bins, and accepted `latency` s later (the next
    capture at once). Returns (rows, tracker, distances after 5 s)."""
    car = car or _corner_car
    tr = StepTracker(cfg)
    veh = _Veh()
    rows, dists = [], []
    pending, seq, t = None, 0, 0.0
    while t < seconds:
        if pending is not None and t >= pending[0] - EPS:
            tr.accept(pending[1], t, pose_now=veh.pose())
            pending = None
        if pending is None:
            cx, cy = car(t)
            dx, dy = cx - veh.x, cy - veh.y
            dist = math.hypot(dx, dy)
            bearing = wrap_pi(math.atan2(dy, dx) - veh.yaw)
            st = _step_of(min(5.0, max(0.0, dist - standoff)), max(-1.1, min(1.1, bearing)))
            seq += 1
            pending = (t + latency, Decision.of(seq, t, veh.pose(), st, t_received=t + latency))
        tk = tr.tick(t, veh.pose(), pending=pending is not None)
        rows.append(_row(t, veh, tk.action, tk.author))
        cx, cy = car(t)
        if t >= 5.0:
            dists.append(math.hypot(cx - veh.x, cy - veh.y))
        veh.step(tk.action)
        t = round(t + 0.1, 10)
    return rows, tr, dists


def _corner_car(t: float) -> tuple[float, float]:
    """3.5 m/s north from (25, 0) for 15 s, a quarter turn to the east over
    4 s (radius ~8.9 m), then east."""
    v, t1, t2 = 3.5, 15.0, 19.0
    if t <= t1:
        return 25.0 + v * t, 0.0
    r = v * (t2 - t1) / (math.pi / 2)
    x0, y0 = 25.0 + v * t1, 0.0
    if t <= t2:
        a = (t - t1) / (t2 - t1) * math.pi / 2
        return x0 + r * math.sin(a), y0 + r * (1 - math.cos(a))
    return x0 + r, y0 + r + v * (t - t2)


def test_without_smoothing_the_tracker_flies_as_before():
    """The same scripted follow with smoothing off by default and spelled
    out as None / 0: identical actions on every tick, and no carry or limit
    counted - the change is opt-in, not a retune."""
    a, ta, _ = _scripted_follow(TrackerConfig(**FOLLOW), seconds=20.0)
    b, tb, _ = _scripted_follow(TrackerConfig(**FOLLOW, smooth_accel_mps2=None,
                                              smooth_yaw_accel_rps2=None, carry_s=0.0),
                                seconds=20.0)
    assert [r["emitted"] for r in a] == [r["emitted"] for r in b]
    for k in ("carry_ticks", "smooth_limited_h", "smooth_limited_yaw", "smooth_decay_ticks"):
        assert ta.counts.get(k, 0) == 0 and tb.counts.get(k, 0) == 0, (k, ta.counts)


def test_hold_horizontal_without_smoothing_zeroes_at_once_and_keeps_yaw_and_climb():
    tr = StepTracker(TrackerConfig(**FOLLOW))
    p = Pose(0.0, 0.0, 9.0, 0.0)
    tr.accept(Decision.of(1, 0.0, p, _step_of(4.9, 0.4)), 0.0, pose_now=p)
    a = tr.tick(0.1, p).action
    assert math.hypot(a.vx, a.vy) > 0.5 and a.yaw_rate > 0 and a.vz_up > 0
    h = tr.tick(0.2, p, hold_horizontal=True)
    assert h.action.vx == 0.0 and h.action.vy == 0.0
    assert h.action.yaw_rate == a.yaw_rate and h.action.vz_up == a.vz_up
    assert h.author == "vla_step" and tr.counts["held_horizontal"] == 1


# ------------------------------------------------------------------ 2. the limits

def test_the_velocity_changes_at_most_accel_times_dt_as_a_vector():
    cfg = TrackerConfig(**FOLLOW, smooth_accel_mps2=2.5)
    tr = StepTracker(cfg)
    p = Pose(0.0, 0.0, 10.0, 0.3)
    tr.accept(Decision.of(1, 0.0, p, _step_of(4.9, 0.0)), 0.0, pose_now=p)
    prev = Action4D()
    t = 0.0
    for k in range(30):
        a = tr.tick(t, p).action
        dv = math.hypot(a.vx - prev.vx, a.vy - prev.vy)
        assert dv <= 2.5 * 0.1 + 1e-9, (k, dv)
        if math.hypot(a.vx, a.vy) > 1e-6:
            # the direction is the step's (heading 0.3 rad), not bent per axis
            assert abs(math.atan2(a.vy, a.vx) - 0.3) < 1e-6, (k, a)
        prev = a
        t = round(t + 0.1, 10)
    assert tr.counts["smooth_limited_h"] >= 10
    # A late tick (1 s gap) buys only one tick's allowance: no burst.
    tr2 = StepTracker(cfg)
    tr2.accept(Decision.of(1, 0.0, p, _step_of(4.9, 0.0)), 0.0, pose_now=p)
    a0 = tr2.tick(0.0, p).action
    a1 = tr2.tick(1.0, p).action
    assert math.hypot(a1.vx - a0.vx, a1.vy - a0.vy) <= 2.5 * 0.1 + 1e-9


def test_the_yaw_rate_slews_and_the_heading_settles_without_overshoot():
    cfg = TrackerConfig(**FOLLOW, smooth_yaw_accel_rps2=0.6)
    tr = StepTracker(cfg)
    veh = _Veh()
    tr.accept(Decision.of(1, 0.0, veh.pose(), _step_of(0.0, 1.0)), 0.0, pose_now=veh.pose())
    target = wrap_pi(_step_of(0.0, 1.0).dpsi_rad)
    prev, t, peak = 0.0, 0.0, -1.0
    for _ in range(100):
        a = tr.tick(t, veh.pose()).action
        assert abs(a.yaw_rate - prev) <= 0.6 * 0.1 + 1e-9, (t, a.yaw_rate, prev)
        prev = a.yaw_rate
        veh.step(a)
        peak = max(peak, veh.yaw)
        t = round(t + 0.1, 10)
    assert abs(veh.yaw - target) <= cfg.yaw_tol_rad + 0.01, (veh.yaw, target)
    assert peak <= target + 0.03, f"overshoot {peak - target:.3f} rad"


# ------------------------------------------------------------------ 3. carry, ramps

def _completed(cfg: TrackerConfig, pending: bool):
    """A short step flown to its end; returns (tracker, the ticks after)."""
    tr = StepTracker(cfg)
    veh = _Veh()
    tr.accept(Decision.of(1, 0.0, veh.pose(), _step_of(3.0, 0.0)), 0.0, pose_now=veh.pose())
    t, done = 0.0, None
    out = []
    for _ in range(120):
        tk = tr.tick(t, veh.pose(), pending=pending)
        out.append((t, tk))
        if done is None and tr.counts.get("completed_tolerance", 0) + \
                tr.counts.get("completed_time_cap", 0):
            done = t
        veh.step(tk.action)
        t = round(t + 0.1, 10)
    return tr, out, done


def test_a_step_finished_while_an_answer_is_pending_carries_its_velocity():
    cfg = TrackerConfig(**FOLLOW, **SMOOTH)
    tr, ticks, done = _completed(cfg, pending=True)
    assert done is not None, "the step never completed"
    carry = [(t, k) for t, k in ticks if k.reason == "carry"]
    assert carry and all(k.author == "vla_step" for _, k in carry)
    assert tr.counts["carry_ticks"] == len(carry)
    # within carry_s of the completion, then a hold
    assert all(t - done < cfg.carry_s + 1e-9 for t, _ in carry), (done, carry[-1][0])
    speeds = [math.hypot(k.action.vx, k.action.vy) for _, k in carry]
    assert speeds[0] > 0.3 and all(b <= a + 1e-9 for a, b in zip(speeds, speeds[1:])), speeds
    # then the hold, until the hold timeout (decision_timeout_s) makes it a hover
    after = [k for t, k in ticks if carry[-1][0] < t < done + cfg.decision_timeout_s - 0.05]
    assert after and all(k.author == "hold" for k in after), {k.author for k in after}
    assert math.hypot(after[-1].action.vx, after[-1].action.vy) < 1e-6
    # Nothing pending: no carry, the hold ramps down at the limit.
    tr2, ticks2, _ = _completed(cfg, pending=False)
    assert not any(k.reason == "carry" for _, k in ticks2) and tr2.counts.get("carry_ticks", 0) == 0
    prev = Action4D()
    for _, k in ticks2:
        assert math.hypot(k.action.vx - prev.vx, k.action.vy - prev.vy) <= 0.25 + 1e-9
        prev = k.action
    assert tr2.counts["smooth_decay_ticks"] > 0


def test_stop_and_go_waits_until_the_ramp_has_stopped():
    """idle_since (stop-and-go's settle timer) under smoothing starts when the
    commanded velocity has ramped to zero, not when the step completed: the
    next image is still captured after the aircraft is told to stand still,
    as upstream's step-then-hover. Without smoothing it is t_complete, as
    before."""
    cfg = TrackerConfig(**FOLLOW, **SMOOTH)
    tr = StepTracker(cfg)
    veh = _Veh()
    assert tr.idle_since == -math.inf                  # nothing flown yet: idle
    tr.accept(Decision.of(1, 0.0, veh.pose(), _step_of(3.0, 0.0)), 0.0, pose_now=veh.pose())
    t, t_done, t_zero, seen = 0.0, None, None, []
    for _ in range(80):
        k = tr.tick(t, veh.pose(), pending=False)
        if t_done is None and tr.counts.get("completed_tolerance", 0):
            t_done = t
        if t_done is not None and t_zero is None and math.hypot(k.action.vx, k.action.vy) <= 1e-3:
            t_zero = t
        seen.append((t, tr.idle_since))
        veh.step(k.action)
        t = round(t + 0.1, 10)
    assert t_done is not None and t_zero is not None and t_zero > t_done + 0.25, (t_done, t_zero)
    assert all(i is None for tt, i in seen if tt < t_zero), "idle while still ramping down"
    assert all(i == t_zero for tt, i in seen if tt >= t_zero), [s for s in seen if s[0] >= t_zero][:3]
    plain = StepTracker(TrackerConfig(**FOLLOW))
    plain.accept(Decision.of(1, 0.0, veh.pose(), _step_of(0.0, 0.0)), 0.0, pose_now=veh.pose())
    plain.tick(0.0, veh.pose())
    assert plain.idle_since == 0.0


def test_a_hover_and_a_close_hold_ramp_down_and_a_suspend_resets():
    cfg = TrackerConfig(**FOLLOW, **SMOOTH)
    tr = StepTracker(cfg)
    p = Pose(0.0, 0.0, 10.0, 0.0)
    tr.accept(Decision.of(1, 0.0, p, _step_of(4.9, 0.0)), 0.0, pose_now=p)
    t, prev = 0.0, Action4D()
    for _ in range(20):
        prev = tr.tick(t, p).action
        t = round(t + 0.1, 10)
    v0 = math.hypot(prev.vx, prev.vy)
    assert v0 > 2.0, v0
    tr.link_lost(t, "test")
    k = tr.tick(t, p)
    assert k.author == "hover" and abs(math.hypot(k.action.vx, k.action.vy) - (v0 - 0.25)) < 1e-6
    # the close hold (StepTracker.tick(hold_horizontal=True)) ramps the same way
    tr2 = StepTracker(cfg)
    tr2.accept(Decision.of(1, 0.0, p, _step_of(4.9, 0.0)), 0.0, pose_now=p)
    t2, a = 0.0, Action4D()
    for _ in range(20):
        a = tr2.tick(t2, p).action
        t2 = round(t2 + 0.1, 10)
    h = tr2.tick(t2, p, hold_horizontal=True).action
    assert abs(math.hypot(h.vx, h.vy) - (math.hypot(a.vx, a.vy) - 0.25)) < 1e-6
    # suspend: the limiter forgets; after resume it starts from rest
    tr2.suspend(t2 + 0.1)
    assert tr2.tick(t2 + 0.2, p).author == "none"
    tr2.resume(t2 + 0.3)
    tr2.accept(Decision.of(2, t2 + 0.3, p, _step_of(4.9, 0.0)), t2 + 0.3, pose_now=p)
    r = tr2.tick(t2 + 0.4, p).action
    assert math.hypot(r.vx, r.vy) <= 0.25 + 1e-9, r


# ------------------------------------------------------------------ 4. closed loop

def test_closed_loop_follow_is_smooth_and_keeps_the_chase():
    import compare_motion as CM
    off, _, d_off = _scripted_follow(TrackerConfig(**FOLLOW), seconds=45.0)
    on, tr_on, d_on = _scripted_follow(TrackerConfig(**FOLLOW, **SMOOTH), seconds=45.0)
    m_off, m_on = CM.motion_numbers(off), CM.motion_numbers(on)
    print(f"      off: cmd accel p99 {m_off['cmd_accel_p99']:.2f} max {m_off['cmd_accel_max']:.2f}, "
          f"yaw p99 {m_off['cmd_yaw_accel_p99']:.2f}; median dist {statistics.median(d_off):.1f} m")
    print(f"      on:  cmd accel p99 {m_on['cmd_accel_p99']:.2f} max {m_on['cmd_accel_max']:.2f}, "
          f"yaw p99 {m_on['cmd_yaw_accel_p99']:.2f}; median dist {statistics.median(d_on):.1f} m; "
          f"carry ticks {tr_on.counts.get('carry_ticks', 0)}")
    assert m_on["cmd_accel_max"] <= 2.5 + 1e-6 and m_on["cmd_accel_p99"] <= 2.5 + 1e-6, m_on
    assert m_on["cmd_yaw_accel_max"] <= 0.6 + 1e-6 and m_on["cmd_yaw_accel_p99"] <= 0.6 + 1e-6
    assert m_off["cmd_accel_p99"] > 2 * m_on["cmd_accel_p99"], (m_off, m_on)
    assert m_off["cmd_yaw_accel_p99"] > 2 * m_on["cmd_yaw_accel_p99"], (m_off, m_on)
    # the chase is kept: within 30 m throughout, and no further behind than
    # the unsmoothed tracker's median by more than a quarter
    assert max(d_on) < 30.0, max(d_on)
    assert statistics.median(d_on) <= 1.25 * statistics.median(d_off), (
        statistics.median(d_on), statistics.median(d_off))


# ------------------------------------------------------------------ the rail

class _Backend:
    """A backend with a smoothing tracker: act(state, hold_horizontal=) is
    recorded and flies the tracker's own tick."""

    def __init__(self, tracker: StepTracker, clock):
        self.tracker, self.clock = tracker, clock
        self.last_link_failure = None
        self.last_tick = None
        self.holds: list[bool] = []

    def act(self, state, hold_horizontal: bool = False):
        self.holds.append(hold_horizontal)
        self.last_tick = self.tracker.tick(self.clock(), Pose.from_state(state),
                                           hold_horizontal=hold_horizontal)
        return self.last_tick.action


def test_the_follows_close_box_goes_through_the_smoothing_tracker():
    """MissionLoop with a smoothing tracker: the close box is read once,
    BEFORE act, and handed to the tracker (hold_horizontal), so the held
    velocity ramps down at the limit instead of dropping to zero in one tick
    before the Shield; the row still says close_hold / close_box."""
    import vla_mission as VM
    from guardrail.replay import rail_shield
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    from vla_bridge import VLABridge
    shield = rail_shield(load_policy(POLICY), lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
    clk = {"t": 0.0}
    tr = StepTracker(TrackerConfig(**FOLLOW, **SMOOTH))
    st = State(x=42.0, y=-18.0, up=10.0, yaw_deg=90.0)
    p = Pose.from_state(st)
    tr.accept(Decision.of(1, 0.0, p, _step_of(4.9, 0.0)), 0.0, pose_now=p)
    box = {"v": None}
    reads = {"n": 0}

    def det_source():
        reads["n"] += 1
        return box["v"]
    be = _Backend(tr, lambda: clk["t"])
    hint = VM.DetectorHint(det_source, VLABridge("a red car", hfov_deg=90.0), fresh_s=1.0)
    mon = VM.MissionMonitor(VM.MonitorConfig(mission="follow", max_s=60.0, lost_timeout_s=90.0),
                            0.0)
    loop = VM.MissionLoop(be, shield, mon, mission="follow", det_source=det_source,
                          detector_hint=hint, clock=lambda: clk["t"],
                          close_hold=VM.BoxClose(0.92, 0.25))
    far = (384.0, 200.0, 30.0, 20.0, 0.3, 768, 432, 0.5)
    low = (384.0, 380.0, 100.0, 80.0, 0.3, 768, 432, 0.5)
    rows = []
    for k in range(25):
        box["v"] = ((low if k >= 15 else far), clk["t"] - 0.2, 0.0, clk["t"] - 0.1)
        res = loop.tick(st)
        rows.append(res.row)
        clk["t"] = round(clk["t"] + 0.1, 10)
    assert reads["n"] == 25, reads
    assert be.holds[:15] == [False] * 15 and all(be.holds[15:])
    sp = [math.hypot(r["raw"]["vx"], r["raw"]["vy"]) for r in rows]
    assert sp[14] > 1.0, sp
    assert all(r["close_hold"] and r["close_box"] for r in rows[15:])
    assert loop.n_close_hold == 10
    for a, b in zip(sp[14:], sp[15:]):
        assert a - b <= 0.25 + 1e-9, (a, b)      # a ramp, not a drop
    assert sp[15] > 0.5, "the hold zeroed in one tick"


# ------------------------------------------------------------------ vla_fly

def test_vla_fly_smooths_the_follow_only_and_records_it():
    import contextlib
    import io
    import vla_fly
    ap = vla_fly.build_arg_parser()
    a = vla_fly.parse_cli(ap, ["--mission", "follow"])
    assert a.smooth is True and a.follow_executor
    rec = vla_fly.smooth_record(a)
    assert rec == {"on": True, "accel_mps2": 2.5, "yaw_accel_rps2": 0.6, "carry_s": 1.0}, rec
    cfg = vla_fly.tracker_config(a, None, TrackerConfig)
    assert cfg.smooth_accel_mps2 == 2.5 and cfg.smooth_yaw_accel_rps2 == 0.6 and cfg.carry_s == 1.0
    assert cfg.turn_while_moving and cfg.anchor_at_answer
    off = vla_fly.parse_cli(ap, ["--mission", "follow", "--no-smooth"])
    c2 = vla_fly.tracker_config(off, None, TrackerConfig)
    assert not c2.smoothing and vla_fly.smooth_record(off)["on"] is False
    g = vla_fly.parse_cli(ap, ["--mission", "goto"])
    assert g.smooth is False and not vla_fly.tracker_config(g, None, TrackerConfig).smoothing
    assert vla_fly.check_args_report(a)["smooth"]["on"] is True
    for bad in (["--mission", "goto", "--smooth"], ["--mission", "approach", "--smooth"],
                ["--mission", "follow", "--no-follow-executor", "--smooth"],
                ["--mission", "follow", "--smooth-accel", "0"],
                ["--mission", "follow", "--smooth-yaw-accel", "-1"],
                ["--mission", "follow", "--carry-s", "-1"],
                ["--mission", "follow", "--vla-adapter", "D:/x", "--vla-server", "127.0.0.1:9"],
                ["--mission", "follow", "--vla-adapter", "D:/x", "--vla-fake-output", "00 49 49"]):
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                vla_fly.parse_cli(ap, bad)
        except SystemExit as exc:
            assert exc.code == 2, bad
            continue
        raise AssertionError(f"accepted {bad}")


def test_vla_fly_starts_the_server_with_the_adapter_it_is_given():
    import vla_fly
    import vla_mission as VM
    ap = vla_fly.build_arg_parser()
    a = vla_fly.parse_cli(ap, ["--mission", "follow", "--vla-adapter",
                               "D:/models/aerialvla-ft-follow/r1/epoch2"])
    srv = vla_fly.server_plan(a, Path(tempfile.mkdtemp(prefix="smooth_")))
    i = srv.argv.index("--adapter")
    assert srv.argv[i + 1] == "D:/models/aerialvla-ft-follow/r1/epoch2"
    assert "--adapter" not in VM.server_argv("py", precision="bf16", attn="sdpa", merge=False)
    try:
        VM.server_argv("py", precision="bf16", attn="sdpa", merge=False, fake_output=["x"],
                       adapter="D:/x")
    except ValueError:
        pass
    else:
        raise AssertionError("an adapter for the scripted stand-in was accepted")


def test_vla_fly_rows_carry_the_attitude_as_follow_vlm_reads_it():
    import follow_vlm as FV
    import vla_fly
    half = math.radians(10.0) / 2.0
    # a pure 10 deg nose-up pitch about the body y axis (NED quaternion)
    kin = {"pose": {"orientation": {"w": math.cos(half), "x": 0.0, "y": math.sin(half), "z": 0.0}}}
    f = vla_fly.attitude_fields(kin, FV)
    assert abs(f["pitch_deg"] - 10.0) < 0.01 and abs(f["roll_deg"]) < 0.01, f
    rkin = {"pose": {"orientation": {"w": math.cos(half), "x": math.sin(half), "y": 0.0, "z": 0.0}}}
    g = vla_fly.attitude_fields(rkin, FV)
    assert abs(g["roll_deg"] - 10.0) < 0.01 and abs(g["pitch_deg"]) < 0.01, g
    assert vla_fly.attitude_fields({"pose": {}}, FV) == {"pitch_deg": None, "roll_deg": None}
    src = (ROOT / "demo" / "vla_fly.py").read_text(encoding="utf-8")
    assert "**attitude_fields(kin, FV)" in src, "the flight loop no longer adds the attitude"
    assert '"motion": guard("motion", lambda: motion_record(rows))' in src


# ------------------------------------------------------------------ compare_motion

def _log(rows, metrics=None) -> Path:
    d = Path(tempfile.mkdtemp(prefix="cmpmot_"))
    (d / "flight_log.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n",
                                        encoding="utf-8")
    (d / "metrics.json").write_text(json.dumps(metrics or {}), encoding="utf-8")
    return d


def test_compare_motion_uses_the_motion_definitions_and_flags_an_unmeasured_attitude():
    import compare_motion as CM
    rows = []
    for k in range(40):
        t = k * 0.1
        v = 0.0 if k < 20 else 2.0                  # one 2 m/s step at k = 20
        rows.append({"t": round(t, 3), "x": 0.0, "y": 0.0, "up": 10.0,
                     "emitted": {"vx": v, "vy": 0.0, "vz_up": 0.0,
                                 "yaw_rate": 0.5 if k >= 30 else 0.0},
                     "pitch_deg": 16.0 if 10 <= k < 13 else 0.0, "roll_deg": 0.0})
    n = CM.motion_numbers(rows)
    assert abs(n["cmd_accel_max"] - 20.0) < 1e-6, n          # 2 m/s in 0.1 s
    assert abs(n["cmd_yaw_accel_max"] - 5.0) < 1e-6, n
    assert n["tilt_gt15_episodes"] == 1 and n["tilt_gt10_episodes"] == 1
    assert abs(n["pitch_rate_max_dps"] - 160.0) < 1e-6, n
    assert n["attitude_rows"] == 40
    # dt floored at 50 ms: a burst tick 10 ms after the last does not inflate
    burst = [dict(rows[0]), dict(rows[1], t=0.01, emitted=dict(rows[1]["emitted"], vx=1.0))]
    assert abs(CM.motion_numbers(burst)["cmd_accel_max"] - 20.0) < 1e-6
    # no attitude on the rows: read as 0 (old tables reproduce) but SAID so
    bare = [{k: v for k, v in r.items() if k not in ("pitch_deg", "roll_deg")} for r in rows]
    a = CM.stats(_log(rows, {"frac_within_30m": 0.9, "params": {"tracker": {"carry_s": 1.0,
                                                                           "smooth_accel_mps2": 2.5,
                                                                           "smooth_yaw_accel_rps2": 0.6}}}))
    b = CM.stats(_log(bare))
    assert a["attitude recorded"] == "40 of 40 rows"
    assert b["attitude recorded"].startswith("0 of 40 rows") and "NOT measured" in b["attitude recorded"]
    assert b["tilt >15 deg episodes"] == 0
    assert a["tracker smoothing"] == "smooth_accel_mps2=2.5, smooth_yaw_accel_rps2=0.6, carry_s=1.0"
    assert b["tracker smoothing"] == "-"
    s = CM.motion_summary(rows)
    assert s["attitude_recorded"] is True and s["cmd_accel_max"] == 20.0
    assert CM.motion_summary(bare)["attitude_recorded"] is False
    out = CM.table(a, b, "A", "B")
    assert out.splitlines()[0].startswith("metric") and "cmd accel p95 / p99 / max m/s2" in out


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
