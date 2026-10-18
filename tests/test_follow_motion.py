"""Motion shaping for the city follow (demo/follow_vlm.py, 2026-10-19).

Run either way:
    pytest tests/test_follow_motion.py -v
    python tests/test_follow_motion.py

The user's complaint: the drone pitches and rolls abruptly when the red car
turns or pulls away. On citylife_redcar_nfz_20261013 body tilt follows
horizontal acceleration (correlation 0.977 with atan(a/g)); pitch reached
-14.9 deg and roll -12.2 deg. Four causes, each answered by a piece tested here:

  1. the stand-off law was linear up to speed_max (241 ticks at the 5 m/s cap)
     -> standoff_speed saturates the catch-up;
  2. FenceGuard's gate/slide switched on and off with no memory (raw steps of
     2.0-3.9 m/s in one tick) -> GuardSmoother, in the loop;
  3. aerialvla_demo.RateLimiter clipped 0.4 m/s per AXIS per CALL, with no
     time base, so a diagonal got 0.566 and a stalled tick followed by an
     immediate one spent two steps in milliseconds -> VecSlewLimiter;
  4. a ~14-18 deg bearing lag in every turn (a P servo on a rotating line of
     sight) -> the line-of-sight-rate feed-forward (los_rate).

Plus a closed-loop kinematic check that acceleration and jerk peaks drop
without losing the follow, and a replay of the recorded flight's commands
(skipped when the gitignored flight log is absent; FOLLOW_MOTION_LOG points
at one elsewhere).
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import follow_vlm as F                                        # noqa: E402
from aerialvla_demo import RateLimiter                        # noqa: E402
from guardrail import load_policy                             # noqa: E402
from guardrail.models import Action4D                         # noqa: E402

SKIP = "skip"
TICK = F.TICK


def _args(argv=()):
    ap = argparse.ArgumentParser()
    F.add_motion_args(ap)
    ap.add_argument("--dv-h", type=float, default=0.4)
    ap.add_argument("--dv-z", type=float, default=0.2)
    return ap.parse_args(list(argv))


def _A(vx=0.0, vy=0.0, vz=0.0, yr=0.0):
    return Action4D(vx=vx, vy=vy, vz_up=vz, yaw_rate=yr)


# ------------------------------------------------------------ the limiter --

def test_a_back_to_back_tick_gets_only_its_share_of_the_step():
    lim = F.VecSlewLimiter(accel=2.0)
    lim(_A(), 0.0)
    out = lim(_A(vx=4.0), 0.01)
    assert abs(out.vx - 2.0 * 0.01) < 1e-9, out
    # the legacy limiter gave a whole 0.4 m/s step to each of the two calls
    leg = RateLimiter(0.4, 0.2)
    leg(_A()); assert abs(leg(_A(vx=4.0)).vx - 0.4) < 1e-9


def test_a_stall_is_not_repaid_as_a_burst():
    """dt is CAPPED at one tick: after 200 ms of silence one call moves the
    command by at most accel x TICK, not accel x 0.2 (a floored or uncapped
    dt only moved the burst, review of rate-limiter-per-call-per-axis)."""
    lim = F.VecSlewLimiter(accel=2.0)
    lim(_A(), 0.0)
    out = lim(_A(vx=4.0), 0.2)
    assert abs(out.vx - 2.0 * TICK) < 1e-9, out


def test_a_diagonal_step_is_limited_as_a_vector_and_keeps_its_direction():
    lim = F.VecSlewLimiter(accel=2.0)
    lim(_A(), 0.0)
    out = lim(_A(vx=3.0, vy=3.0), TICK)
    assert abs(math.hypot(out.vx, out.vy) - 2.0 * TICK) < 1e-9, out
    assert abs(out.vx - out.vy) < 1e-12, "direction bent"
    leg = RateLimiter(0.4, 0.2)
    leg(_A()); o = leg(_A(vx=3.0, vy=3.0))
    assert abs(math.hypot(o.vx, o.vy) - 0.4 * math.sqrt(2)) < 1e-9   # the old sqrt(2)


def test_speed_up_is_gentle_and_turning_has_its_own_limit():
    lim = F.VecSlewLimiter(accel=2.0, brake=4.0, lat=2.5)
    lim.prev, lim.t = _A(vx=4.0), 0.0
    up = lim(_A(vx=5.0), TICK)
    assert abs(up.vx - 4.2) < 1e-9, up                 # 2 m/s^2
    lim.prev, lim.t = _A(vx=4.0), 0.0
    turn = lim(_A(vx=4.0, vy=2.0), TICK)
    assert abs(turn.vy - 0.25) < 1e-9 and abs(turn.vx - 4.0) < 1e-9, turn


def test_braking_is_never_later_than_the_legacy_step_whatever_the_tick_spacing():
    """'Never slower braking for holds than today': for every slowing
    command, in every tick pattern, the new speed is no higher than what the
    legacy per-axis per-call step would have left."""
    rng = np.random.default_rng(7)
    for _ in range(300):
        lim = F.VecSlewLimiter()
        p = _A(vx=float(rng.uniform(-5, 5)), vy=float(rng.uniform(-5, 5)))
        lim.prev, lim.t = p, 0.0
        sp = math.hypot(p.vx, p.vy)
        k = float(rng.uniform(0.0, 0.95))
        ang = float(rng.uniform(-0.6, 0.6))
        c, s = math.cos(ang), math.sin(ang)
        a = _A(vx=k * (c * p.vx - s * p.vy), vy=k * (s * p.vx + c * p.vy))
        dt = float(rng.choice([0.005, 0.05, 0.1, 0.2]))
        out = lim(a, dt)
        lx = p.vx + float(np.clip(a.vx - p.vx, -0.4, 0.4))
        ly = p.vy + float(np.clip(a.vy - p.vy, -0.4, 0.4))
        assert math.hypot(out.vx, out.vy) <= math.hypot(lx, ly) + 1e-9, (p, a, dt, out)
        assert math.hypot(out.vx, out.vy) <= sp + 1e-9
    # a hold from 4 m/s, back-to-back: as fast as the legacy step
    lim = F.VecSlewLimiter()
    lim.prev, lim.t = _A(vx=4.0), 0.0
    assert abs(lim(_A(), 0.005).vx - 3.6) < 1e-9


def test_the_yaw_rate_is_slewed():
    lim = F.VecSlewLimiter(yaw=2.0)
    lim(_A(), 0.0)
    out = lim(_A(yr=1.2), TICK)
    assert abs(out.yaw_rate - 0.2) < 1e-9, out
    off = F.VecSlewLimiter(yaw=0.0)
    off(_A(), 0.0)
    assert off(_A(yr=1.2), TICK).yaw_rate == 1.2


# ------------------------------------------------------ the guard smoother --

def _wall_guard():
    """follow_car.yaml (no fence) and a synthetic wall: every cell with
    y >= 5.2 is occupied, a 5 m clearance ring - the verifier's setup."""
    res, ox, oy = 0.5, -60.0, -29.8
    occ = np.zeros((400, 120), dtype=bool)
    ys = oy + np.arange(120) * res
    occ[:, ys >= 5.2 - 1e-9] = True
    pol = load_policy(ROOT / "policies" / "follow_car.yaml")
    return F.FenceGuard(pol, brake_m=12.0, stand_off_m=3.0,
                        obstacle_map={"occ": occ, "res": res, "ox": ox, "oy": oy},
                        min_clearance_m=5.0)


def _compose(fence, x, y, cvx, cvy, slide_speed=2.5):
    """One tick of the loop's guard composition (follow_vlm, before the
    limiter): gate, then slide() while blocked or approaching."""
    fscale, fdist, fblocked = fence.gate(x, y, cvx, cvy)
    lat_x = lat_y = 0.0
    approaching = fdist is not None and fdist < fence.brake_m and fscale < 1.0
    if fblocked or approaching:
        sx, sy, _ = fence.slide(x, y, cvx, cvy)
        if sx or sy:
            lat = slide_speed * float(np.clip(1.0 - fscale, 0.33, 1.0))
            lat_x, lat_y = sx * lat, sy * lat
    return fscale, fdist, fblocked, lat_x, lat_y


def test_the_guard_smoother_is_a_pass_through_while_the_guard_is_clear():
    gs = F.GuardSmoother()
    for k in range(5):
        assert gs(k * TICK, 3.0, -1.0, 1.0) == (3.0, -1.0)
    assert gs.stats()["ramped_ticks"] == 0


def test_an_urgent_brake_bites_at_once_and_its_release_is_ramped():
    gs = F.GuardSmoother(release=2.0)
    gs(0.0, 4.0, 0.0, 1.0)
    assert gs(0.1, 4.0, 0.0, 0.1, urgent=True) == (0.4, 0.0)       # at once
    gx, _ = gs(0.2, 4.0, 0.0, 1.0)
    assert abs(gx - (0.4 + 2.0 * TICK)) < 1e-9, gx                   # 2 m/s^2 back
    # a NON-urgent brake is ramped at `engage`
    gs2 = F.GuardSmoother(engage=3.0)
    gs2(0.0, 4.0, 0.0, 1.0)
    gx, _ = gs2(0.1, 4.0, 0.0, 0.1)
    assert abs(gx - (4.0 - 3.0 * TICK)) < 1e-9, gx


def test_beside_a_wall_the_guards_command_no_longer_jumps():
    """Heading walked 0 -> 6 -> 0 deg toward a wall 5.2 m away at 4 m/s,
    tick by tick. Today the guard's output steps by metres per second
    (gate 1.00 -> 0.55 -> 0.27 in 2 deg, plus a sidestep on or off); through
    GuardSmoother it moves by at most (engage or release, with the sidestep
    fade) x TICK - under 0.4 m/s a tick - except where the brake must bite at
    once (blocked). The lunge itself was the RELEASE: the guard let go, the
    command jumped back to 4 m/s and the limiter after it gained speed at its
    full step (>= 0.3 m/s a tick) for tick after tick. Through the smoother it
    never does so for more than 2 ticks running."""
    fence = _wall_guard()
    heads = list(np.arange(0.0, 6.01, 0.5)) + list(np.arange(5.5, -0.01, -0.5))
    gs = F.GuardSmoother()
    lims = {"old": RateLimiter(0.4, 0.2), "new": RateLimiter(0.4, 0.2)}
    for lim in lims.values():
        lim.prev = _A(vx=4.0)            # already flying the 4 m/s it is asked
    run = {"old": 0, "new": 0}
    longest = {"old": 0, "new": 0}
    prev_new = prev_old = None
    worst_new = worst_old = 0.0
    urgent_ticks = 0
    for k, hd in enumerate(heads):
        a = math.radians(hd)
        cvx, cvy = 4.0 * math.cos(a), 4.0 * math.sin(a)
        fs, fd, fb, lx, ly = _compose(fence, 0.0, 0.0, cvx, cvy)
        old = (cvx * fs + lx, cvy * fs + ly)
        urgent = F.guard_urgent(fence, fd, fb)
        urgent_ticks += int(urgent)
        new = gs(k * TICK, cvx, cvy, fs, lx, ly, urgent=urgent)
        if prev_old is not None:
            worst_old = max(worst_old, math.hypot(old[0] - prev_old[0], old[1] - prev_old[1]))
            if not urgent:
                worst_new = max(worst_new, math.hypot(new[0] - prev_new[0],
                                                      new[1] - prev_new[1]))
        prev_old, prev_new = old, new
        for key, cmd in (("old", old), ("new", new)):
            lim = lims[key]
            before = lim.prev
            out = lim(_A(vx=cmd[0], vy=cmd[1]))
            gain = math.hypot(out.vx, out.vy) - math.hypot(before.vx, before.vy)
            run[key] = run[key] + 1 if gain >= 0.3 else 0
            longest[key] = max(longest[key], run[key])
    assert urgent_ticks > 0, "the walk must reach a blocked heading"
    assert worst_old > 1.0, f"the setup must reproduce the jump (worst {worst_old:.2f})"
    assert worst_new <= 0.4, f"smoothed guard stepped {worst_new:.2f} m/s in a tick"
    assert longest["old"] > 2, f"the setup must reproduce the lunge ({longest})"
    assert longest["new"] <= 2, f"full-step speed-up ticks in a row: {longest}"


# --------------------------------------------------- stand-off and yaw laws --

def test_the_catch_up_saturates_and_small_errors_are_unchanged():
    lin = F.standoff_speed(17.0, 15.83, 0.25, 0.0, 5.0, 0.0)
    sat = F.standoff_speed(17.0, 15.83, 0.25, 0.0, 5.0, 2.5)
    assert abs(sat - lin) / lin < 0.02, (sat, lin)
    # a 13 m lag behind a car opening at 2 m/s: linear sprints to the cap
    assert F.standoff_speed(28.83, 15.83, 0.25, 2.0, 5.0, 0.0) == 5.0
    far = F.standoff_speed(28.83, 15.83, 0.25, 2.0, 5.0, 2.5)
    assert 2.0 + 2.0 < far < 2.0 + 2.5, far
    # too close stays linear, and 0 is exactly the old law
    assert F.standoff_speed(10.0, 15.83, 0.25, 0.0, 5.0, 2.5) == \
        F.standoff_speed(10.0, 15.83, 0.25, 0.0, 5.0, 0.0)
    for r in (5.0, 15.0, 40.0):
        assert F.standoff_speed(r, 15.83, 0.25, 1.0, 5.0, 0.0) == \
            float(np.clip((r - 15.83) * 0.25 + 1.0, -2.0, 5.0))


def test_the_line_of_sight_rate_has_the_yaw_sense_and_cancels_the_turn_lag():
    # subject 20 m north, moving east at 4 m/s: the sight line turns clockwise
    w = F.los_rate(20.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0)
    assert abs(w - 0.2) < 1e-12 and w > 0.0
    assert F.los_rate(20.0, 0.0, 4.0, 0.0, 0.0, 0.0, 4.0, 0.0) == 0.0     # in line
    assert F.los_rate(0.5, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0) == 0.0      # too close
    # a P servo on that sight line settles w/gain behind it; with the rate fed
    # forward it settles on it
    for ff, want in ((0.0, w / 1.2), (w, 0.0)):
        b = 0.0
        for _ in range(400):
            b += (w - F.yaw_command(1.2, b, ff=ff)) * 0.02
        assert abs(b - want) < 1e-3, (ff, b, want)
    assert F.yaw_command(1.2, 0.0) == 0.0
    assert F.yaw_command(1.2, 1.0, ff=0.5) == F.YAW_RATE_CAP


def test_the_motion_settings_are_recorded_and_legacy_restores_the_old_limiter():
    p = F.motion_params(_args())
    assert p["legacy"] is False and p["accel_mps2"] == 2.0 and p["catchup_mps"] == 2.5
    assert p["guard_smoother"]["release_mps2"] == 2.0 and p["yaw_los_ff"] == 0.7
    lim, gs = F.make_motion(_args())
    assert isinstance(lim, F.VecSlewLimiter) and isinstance(gs, F.GuardSmoother)
    lim, gs = F.make_motion(_args(["--legacy-motion"]))
    assert isinstance(lim, RateLimiter) and gs is None
    assert F.motion_params(_args(["--legacy-motion"]))["legacy"] is True


def test_the_help_renders_and_lists_the_motion_flags():
    """`follow_vlm.py --help` crashed (a bare '%' in --target-estimator's
    help, which argparse formats with %), so none of these flags could be
    read from the command line."""
    import subprocess
    r = subprocess.run([sys.executable, str(ROOT / "demo" / "follow_vlm.py"), "--help"],
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr[-800:]
    for flag in ("--legacy-motion", "--accel-max", "--brake-max", "--turn-accel-max",
                 "--yaw-accel-max", "--guard-release-accel", "--guard-engage-accel",
                 "--guard-lat-accel", "--catchup-mps", "--yaw-los-ff",
                 "--legacy-tick-pose", "--legacy-servo-width", "--grant-forecast"):
        assert flag in r.stdout, flag


# ----------------------------------------- closed-loop kinematics and replay --

def _car_track(T=44.0, h=0.005):
    """4 m/s east; stop between 6 and 12 s; pull away at 3 m/s^2 to 4.5 m/s;
    a left turn of radius 15 m at 22 s. (13 Oct: the car's median speed was
    3.2 m/s, p90 3.9.)"""
    out, x, y, hd, v = [], 0.0, 0.0, 0.0, 4.0
    t_turn = (math.pi / 2) * 15.0 / 4.5
    for i in range(int(T / h) + 1):
        t = i * h
        if 6.0 <= t < 8.0:
            v = max(0.0, v - 2.0 * h)
        elif 8.0 <= t < 12.0:
            v = 0.0
        elif t >= 12.0:
            v = min(4.5, v + 3.0 * h)
        hd += (v / 15.0) * h if 22.0 <= t < 22.0 + t_turn else 0.0
        x += v * math.cos(hd) * h
        y += v * math.sin(hd) * h
        out.append((x, y, v * math.cos(hd), v * math.sin(hd)))
    return out


def _aircraft(v1, v2, cmd, h, tau=(0.6, 0.25)):
    """Velocity response to a command: two cascaded lags (their sum ~ the
    ~1 s first-order lag fitted to the 13 Oct flight's poses)."""
    v1 = v1 + (cmd - v1) * (h / tau[0])
    return v1, v2 + (v1 - v2) * (h / tau[1])


def _fly(argv, pattern=(0.2, 0.005, 0.1, 0.1, 0.1), want=15.83, vmax=5.0):
    """Closed loop: the yaw servo and stand-off law of follow_vlm's
    estimator branch (truth in place of the estimate), a guard that chatters
    between 28 and 33 s (scale 1.0 / 0.25 every 0.4 s, a 1.5 m/s sidestep),
    the limiter, and the aircraft model; ticks spaced like the truth-poll
    stalls (200 ms, then 5 ms)."""
    args = _args(argv)
    legacy = args.legacy_motion
    lim, gs = F.make_motion(args)
    catch = 0.0 if legacy else args.catchup_mps
    car = _car_track()
    h = 0.005
    x, y, yaw = -want, 0.0, 0.0
    v1 = np.zeros(2); v2 = np.zeros(2); cmd = np.zeros(2); yr = 0.0
    t, t_next, k = 0.0, 0.0, 0
    vel, rngs, turn_brg = [], [], []
    while t < 44.0:
        if t >= t_next - 1e-9:
            cx, cy, cvx_, cvy_ = car[min(len(car) - 1, int(round(t / h)))]
            dx, dy = cx - x, cy - y
            rng = math.hypot(dx, dy)
            brg = math.atan2(dy, dx) - yaw
            brg = math.atan2(math.sin(brg), math.cos(brg))
            ff = (cvx_ * dx + cvy_ * dy) / max(rng, 1e-3)
            ffy = 0.0 if legacy else float(np.clip(
                args.yaw_los_ff * F.los_rate(cx, cy, cvx_, cvy_, x, y, v2[0], v2[1]),
                -args.yaw_los_ff_cap, args.yaw_los_ff_cap))
            yaw_rate = F.yaw_command(1.2, brg, ff=ffy)
            fwd = F.standoff_speed(rng, want, 0.25, ff, vmax, catch) * max(0.0, math.cos(brg))
            cvx, cvy = fwd * math.cos(yaw), fwd * math.sin(yaw)
            fs, lx, ly = 1.0, 0.0, 0.0
            if 28.0 <= t < 33.0 and int((t - 28.0) / 0.4) % 2 == 1:
                n = math.hypot(cvx, cvy) or 1.0
                fs, lx, ly = 0.25, -cvy / n * 1.5, cvx / n * 1.5
            gx, gy = cvx * fs + lx, cvy * fs + ly
            if gs is not None:
                gx, gy = gs(t, cvx, cvy, fs, lx, ly, urgent=False)
            raw = _A(vx=gx, vy=gy, yr=yaw_rate)
            out = lim(raw, t) if isinstance(lim, F.VecSlewLimiter) else lim(raw)
            cmd = np.array([out.vx, out.vy]); yr = out.yaw_rate
            rngs.append(rng)
            if 22.0 < t < 26.0:
                turn_brg.append(abs(math.degrees(brg)))
            t_next = t + pattern[k % len(pattern)]; k += 1
        v1, v2 = _aircraft(v1, v2, cmd, h)
        x += v2[0] * h; y += v2[1] * h; yaw += yr * h
        vel.append((t, v2[0], v2[1]))
        t += h
    v = np.array(vel)
    a = np.gradient(v[:, 1:3], v[:, 0], axis=0)
    sp = np.hypot(v[:, 1], v[:, 2]) + 1e-9
    a_par = (a[:, 0] * v[:, 1] + a[:, 1] * v[:, 2]) / sp
    j = np.gradient(a, v[:, 0], axis=0)
    return {"speed_up": float(a_par.max()), "brake": float(-a_par.min()),
            "accel": float(np.hypot(a[:, 0], a[:, 1]).max()),
            "jerk": float(np.hypot(j[:, 0], j[:, 1]).max()),
            "range_max": max(rngs), "range_mean": float(np.mean(rngs)),
            "turn_brg_p50": float(np.median(turn_brg))}


def test_closed_loop_acceleration_and_jerk_peaks_drop_and_the_follow_holds():
    old = _fly(["--legacy-motion"])
    new = _fly([])
    # speed-up (the pitch-down surge) and jerk drop a lot...
    assert new["speed_up"] < 0.6 * old["speed_up"], (old, new)
    assert new["jerk"] < 0.5 * old["jerk"], (old, new)
    assert new["accel"] < old["accel"], (old, new)
    # ...braking is NOT made softer (it is the hold side)...
    assert new["brake"] >= 0.95 * old["brake"], (old, new)
    # ...the turn lag shrinks, and the follow is kept
    assert new["turn_brg_p50"] < 0.5 * old["turn_brg_p50"], (old, new)
    assert new["range_max"] <= old["range_max"] + 1.0 and new["range_max"] < 30.0, (old, new)
    # the same holds at a steady 10 Hz
    old10, new10 = _fly(["--legacy-motion"], pattern=(0.1,)), _fly([], pattern=(0.1,))
    assert new10["jerk"] < 0.6 * old10["jerk"] and new10["range_max"] < 30.0, (old10, new10)


def _flight_log():
    for p in (os.environ.get("FOLLOW_MOTION_LOG"),
              ROOT / "demo" / "out" / "citylife_redcar_nfz_20261013" / "flight_log.jsonl"):
        if p and Path(p).is_file():
            return Path(p)
    return None


def test_replayed_on_the_13_oct_commands_the_new_limiter_cuts_the_peaks():
    """Open loop: the recorded raw commands at their recorded tick times,
    through the legacy limiter (which reproduces the logged `smooth`
    exactly) and through VecSlewLimiter, then the aircraft model."""
    log = _flight_log()
    if log is None:
        return SKIP
    rows = [json.loads(l) for l in log.open(encoding="utf-8")]
    t = [r["t"] for r in rows]
    raws = [_A(r["raw"]["vx"], r["raw"]["vy"], r["raw"]["vz_up"], r["raw"]["yaw_rate"])
            for r in rows]
    leg = RateLimiter(0.4, 0.2)
    old = [leg(a) for a in raws]
    err = max(abs(o.vx - r["smooth"]["vx"]) + abs(o.vy - r["smooth"]["vy"])
              for o, r in zip(old, rows))
    assert err < 1e-6, f"the legacy replay must reproduce the log ({err})"
    lim = F.make_motion(_args())[0]
    new = [lim(a, ti) for a, ti in zip(raws, t)]

    def burst(cmds):           # largest change of the command within 30 ms
        worst = 0.0
        for i in range(1, len(cmds)):
            j = i
            while j > 0 and t[i] - t[j - 1] <= 0.03:
                j -= 1
            if j < i:
                worst = max(worst, math.hypot(cmds[i].vx - cmds[j - 1].vx if j else 0,
                                              cmds[i].vy - cmds[j - 1].vy if j else 0))
        return worst

    def peaks(cmds):
        h = 0.005
        v1 = np.zeros(2); v2 = np.zeros(2); k = 0; out = []
        tt = t[0]
        while tt < t[-1]:
            while k + 1 < len(t) and t[k + 1] <= tt:
                k += 1
            v1, v2 = _aircraft(v1, v2, np.array([cmds[k].vx, cmds[k].vy]), h)
            out.append((tt, v2[0], v2[1])); tt += h
        v = np.array(out)
        a = np.gradient(v[:, 1:3], v[:, 0], axis=0)
        j = np.gradient(a, v[:, 0], axis=0)
        return float(np.hypot(a[:, 0], a[:, 1]).max()), float(np.hypot(j[:, 0], j[:, 1]).max())

    a_old, j_old = peaks(old)
    a_new, j_new = peaks(new)
    # Measured 2026-10-19: accel 3.35 -> 2.13 m/s^2, jerk 8.42 -> 6.48 m/s^3.
    # Open loop and without the guard smoother (the logged raw command is
    # already composed), so this is the limiter's share alone.
    assert burst(new) < burst(old), (burst(new), burst(old))
    assert a_new < 0.8 * a_old and j_new < 0.9 * j_old, (a_old, a_new, j_old, j_new)


if __name__ == "__main__":
    # A test that short-circuits on a missing fixture must NOT print PASS.
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (fixture missing)")
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
