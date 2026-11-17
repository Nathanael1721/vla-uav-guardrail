"""The estimator follow on the Project AirSim + ArduPilot perception node
(demo/pas_ardupilot/perception_node.py --pilot servo --mission follow
--identity --target-estimator; FAMILY=estimator in deploy/orin/
run_pas_perception.sh): the lab's hand-written follow WITH its target
estimator, the VLA backend table's C2 estimator null - checked without the
simulator, ArduPilot, the GPU or a model.

Run either way:
    pytest tests/test_pas_estimator_follow.py -v
    python tests/test_pas_estimator_follow.py

WHAT IS BEING CLAIMED (2026-11-17)

1. ONE CONTROL LAW. On a served estimate the pilot's yaw rate and forward
   speed are follow_vlm.fly()'s estimate branch, computed by follow_vlm's
   own functions (yaw_command with the line-of-sight feed-forward,
   standoff_speed with the range feed-forward and the catch-up, times
   cos(bearing)) on follow_vlm's own parser defaults, at the stand-off
   target_state.want_range_from_width gives; with a fresh box and no
   estimate it is follow_vlm.servo at servo_want_width of that stand-off.
   The stopped-subject floor, the line-of-sight rate on the last command
   (no measured velocity), the memory coast that keeps a backing aircraft
   backing, and fly()'s forward direction (--trail-follow: toward a carrot
   on the subject's trail, never away from the estimate) are fly()'s. The
   HUD claims TARGET LOCKED only on a fresh box.
2. ON A SCRIPTED CAR, CLOSED LOOP. A car driving at 3.2 m/s, hidden for
   2.5 s and later for 6 s, followed from 20 m: the estimate flies most
   ticks and every hidden tick of the short gap, the aircraft holds near
   the stand-off and within 30 m throughout, and the long gap is a lapse
   the Reacquirer takes back; the box-width servo of C2's first null
   (FollowPilot at its node defaults) on the same car falls behind. Round
   a right-angle corner the trail keeps the aircraft nearer the car's path
   than the nose does; with no subject committed the box law flies.
3. THE NODE. --target-estimator with the servo pilot builds this pilot (the
   strict lock, the SubjectTrack started at the mission's start), and a
   flight on the stand-ins records the law per tick, est_* fields, metrics
   estimator_follow and subject_track; the Shield's subject is still the
   fresh box only. Without --target-estimator the servo pilot is the box
   servo, its rows and metrics without any of it.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PAS = ROOT / "demo" / "pas_ardupilot"
for _p in (ROOT, ROOT / "tests", ROOT / "demo", PAS, ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import follow_vlm as FV                                              # noqa: E402
import perception_node as pn                                         # noqa: E402
import subject_track as ST                                           # noqa: E402
import test_subject_track as TS                                      # noqa: E402

DT = 0.1


def _pilot(track, g, **kw):
    return pn.EstimatorFollowPilot(g, 90.0, track, object_width_m=4.0, fv=FV, **kw)


# =========================================================================== #
# 1. One control law
# =========================================================================== #

def test_the_law_is_fly_s_estimate_branch_on_follow_vlm_s_own_functions():
    tr, g = TS._started()
    p = _pilot(tr, g)
    ap = FV.build_arg_parser()
    cfg = p.config()
    for k in pn.ESTIMATOR_FOLLOW_KEYS:
        assert cfg[k] == ap.get_default(k), k
    # want_width 0.16 on a 4 m car: fly()'s 15.83 m; the box servo's fraction
    # holds the same metric stand-off on the pinhole camera
    assert abs(p.want_range_m - FV.want_range_from_width(0.16, 4.0)) < 1e-9
    assert abs(p.want_range_m - 15.83) < 0.01, p.want_range_m
    assert abs(p.box_want_w_frac - FV.servo_want_width(p.want_range_m, 4.0, 90.0)) < 1e-12
    # Build an estimate: the car seen every 0.25 s for 2 s from 20 m behind.
    TS._fly(tr, g, 100.0, 102.0)
    t = 102.05
    # The aircraft off the car's line (a line-of-sight rate), turned 0.3 rad.
    x, y, yaw = TS._car(t)[0] - 18.0, 4.0, 0.3
    vel = (2.0, 0.5, 0.0)
    e = tr.estimator
    # The expected command, from follow_vlm's functions on a copy of the state.
    obs = e.observe(t, x, y, yaw)
    assert obs is not None
    b, r = obs
    los = FV.los_rate(float(e.x[0]), float(e.x[1]), float(e.x[2]), float(e.x[3]),
                      x, y, vel[0], vel[1], r_min=ap.get_default("yaw_los_ff_min_range"))
    ff_yaw = max(-0.4, min(0.4, 0.7 * los))
    want_yaw = FV.yaw_command(1.2, b, ff=ff_yaw)
    want_fwd = FV.standoff_speed(r, p.want_range_m, 0.25, e.range_rate(x, y), 5.0, 3.0) \
        * max(0.0, math.cos(b))
    body, info = p.step(t, 8.0, pose=(x, y, yaw), vel=vel)
    assert info["law"] == "estimate" and info["mode"] == "track", info
    assert abs(body[3] - want_yaw) < 1e-9 and abs(body[0] - want_fwd) < 1e-9, (body, want_yaw,
                                                                                 want_fwd)
    assert ff_yaw != 0.0 and p.n_los_ff == 1
    assert body[1] == 0.0 and abs(body[2] - 0.0) < 1e-9          # at 8 m: no climb
    # No estimate (none seeded) and a fresh box: the box servo at the same
    # metric stand-off.
    g2 = TS.FakeGrounder()
    tr2 = ST.SubjectTrack(g2, "car", object_width_m=4.0, fv=FV)
    p2 = _pilot(tr2, g2)
    pose = (0.0, 0.0, 8.0, 0.0)
    g2.infer(10.0, pose, [((20.0, 2.0), "ok")])
    body2, info2 = p2.step(10.05, 8.0, pose=(0.0, 0.0, 0.0))
    det = g2.latest()["det"]
    yr, fw, _b = FV.servo(det, int(det[5]), 1.2, p2.box_want_w_frac, 5.0, 90.0)
    assert info2["law"] == "box" and (body2[0], body2[3]) == (fw, yr), (body2, fw, yr)


def test_a_fault_in_the_track_is_counted_and_never_stops_the_pilot():
    class Boom:
        state, reacq = "committed", None
        estimator = SimpleNamespace(observe=lambda *a: (_ for _ in ()).throw(
            RuntimeError("bad estimate")))

        def step(self, *a):
            raise RuntimeError("bad step")

        def memory_bearing(self, *a):
            raise RuntimeError("bad memory")

        def row(self, *a):
            raise RuntimeError("bad row")
    g = TS.FakeGrounder()
    p = _pilot(Boom(), g)
    body, info = p.step(1.0, 8.0, pose=(0.0, 0.0, 0.0))
    assert info["law"] == "sweep" and body[0] == 0.0, info
    assert p.track_errors == 3 and "bad step" in p.track_error
    row = p.row_fields(1.0, SimpleNamespace(x=0.0, y=0.0, yaw_deg=0.0))
    assert row["est_state"] is None and row["follow_law"] == "sweep"
    s = p.summary()
    assert s["track_errors"] == 4 and s["ticks_by_law"]["sweep"] == 1


def test_a_subject_measured_stopped_is_not_closed_past_the_floor():
    """fly()'s PREDICTION IS NOT EVIDENCE: a car measured stopped, its
    estimate over 0.5 s old, is not closed nearer than the camera's blind
    range plus 2 m - the forward speed capped at half the distance left."""
    def stop(t):
        return (25.0, 0.0)

    def pose(t):
        return (0.0, 0.0, 8.0, 0.0)
    tr, g = TS._started(car=stop, pose=pose)
    TS._fly(tr, g, 100.0, 103.0, car=stop, pose=pose)
    TS._fly(tr, g, 103.0, 104.0, car=stop, pose=pose, hidden=lambda t: True)
    assert tr.last_meas_speed is not None and tr.last_meas_speed < 1.0, tr.last_meas_speed
    # a stand-off (6.2 m) inside the floor, so the law alone would close in
    p = _pilot(tr, g, want_w_frac=0.4)
    t = 104.05
    assert t - tr.estimator.t_last_update > 0.5
    x = 15.0                                      # 10 m short of the car
    b, r = tr.estimator.observe(t, x, 0.0, 0.0)
    free = FV.standoff_speed(r, p.want_range_m, 0.25, tr.estimator.range_rate(x, 0.0), 5.0,
                             3.0) * max(0.0, math.cos(b))
    floor = FV.camera_blind_m(8.0) + 2.0
    capped = max(0.0, 0.5 * (math.hypot(tr.last_acc[0] - x, tr.last_acc[1]) - floor))
    assert capped < free, (capped, free)
    body, info = p.step(t, 8.0, pose=(x, 0.0, 0.0))
    assert info["law"] == "estimate" and abs(body[0] - capped) < 1e-9, (body, capped)
    assert p.n_floor == 1 and p.summary()["stopped_floor_ticks"] == 1


def test_without_a_measured_velocity_the_los_rate_uses_the_last_command():
    """vel=None: the line-of-sight rate's aircraft velocity is the pilot's
    last commanded horizontal velocity (fly()'s last_emit_v), here the last
    forward speed along the heading (the trail is under 8 m: no carrot)."""
    tr, g = TS._started()
    p = _pilot(tr, g)
    TS._fly(tr, g, 100.0, 102.0)
    x0, y, yaw = TS._car(102.05)[0] - 18.0, 4.0, 0.3
    body1, info1 = p.step(102.05, 8.0, pose=(x0, y, yaw))
    assert info1["trail_dir"] is None and body1[0] > 0.0
    last = (body1[0] * math.cos(yaw), body1[0] * math.sin(yaw))
    assert max(abs(a - b) for a, b in zip(p._last_v, last)) < 1e-12, (p._last_v, last)
    t, x = 102.15, x0 + 0.3
    body2, info2 = p.step(t, 8.0, pose=(x, y, yaw))
    e = tr.estimator
    b, _r = e.observe(t, x, y, yaw)
    ap = FV.build_arg_parser()
    los = FV.los_rate(float(e.x[0]), float(e.x[1]), float(e.x[2]), float(e.x[3]),
                      x, y, last[0], last[1], r_min=ap.get_default("yaw_los_ff_min_range"))
    ff = max(-ap.get_default("yaw_los_ff_cap"),
             min(ap.get_default("yaw_los_ff_cap"), ap.get_default("yaw_los_ff") * los))
    assert ff != 0.0
    assert abs(body2[3] - FV.yaw_command(1.2, b, ff=ff)) < 1e-9, (body2, ff)


def test_forward_follows_the_subjects_trail_and_never_away_from_it():
    """fly()'s WHICH WAY FORWARD IS (--trail-follow, as agx_manual_20261103
    flew): with 8 m of trail the forward speed is flown toward a carrot on it
    (a body right component), the yaw still on the subject; a trail
    direction pointing away from the served estimate is refused and the nose
    wins; trail_follow=False flies along the nose."""
    import trail as trail_mod
    tr, g = TS._started()
    TS._fly(tr, g, 100.0, 105.0)
    assert tr.trail.length() >= trail_mod.MIN_TRAIL_M
    p = _pilot(tr, g)
    t = 105.05
    x, y, yaw = TS._car(t)[0] - 20.0, 4.0, 0.3
    body, info = p.step(t, 8.0, pose=(x, y, yaw))
    d = trail_mod.direction(tr.trail, x, y, FV.build_arg_parser().get_default(
        "trail_lookahead_m"))
    a, v = math.atan2(d[1], d[0]), info["fwd_speed"]
    assert info["law"] == "estimate" and v > 0.0 and abs(info["trail_dir"] - a) < 1e-12
    assert abs(body[0] - v * math.cos(a - yaw)) < 1e-9 and \
        abs(body[1] - v * math.sin(a - yaw)) < 1e-9 and body[1] < -0.5, (body, v, a)
    # the world velocity it asks for is v along the trail (from_body's frame)
    from guardrail.frames import from_body
    w = from_body(body[0], body[1], 0.0, 0.0, math.degrees(yaw))
    assert abs(w.vx - v * d[0]) < 1e-9 and abs(w.vy - v * d[1]) < 1e-9
    assert p.n_trail == 1 and p.summary()["trail_ticks"] == 1
    # ahead of the car and off to its side: the trail's end heading (+x)
    # points away from the estimate - refused, the nose wins
    car = TS._car(t + 0.1)
    x2, y2 = car[0] + 10.0, -30.0
    yaw2 = math.atan2(car[1] - y2, car[0] - x2)
    body2, info2 = p.step(t + 0.1, 8.0, pose=(x2, y2, yaw2))
    assert info2["fwd_speed"] > 0.0 and info2["trail_dir"] is None and body2[1] == 0.0, info2
    assert p.n_trail_against == 1
    assert p.summary()["trail_refused_away_from_subject"] == 1
    # trail_follow off: along the nose
    tr3, g3 = TS._started()
    TS._fly(tr3, g3, 100.0, 105.0)
    p3 = _pilot(tr3, g3, trail_follow=False)
    body3, info3 = p3.step(t, 8.0, pose=(x, y, yaw))
    assert info3["trail_dir"] is None and body3[1] == 0.0 and body3[0] == info3["fwd_speed"]
    assert p3.config()["trail_follow"] is False and p.config()["trail_follow"] is True


def test_the_memory_coast_keeps_a_backing_off_aircraft_backing():
    """fly()'s coast flies last_cmd[1] x k: an aircraft that was backing off
    keeps backing, decaying over coast_s (not clamped at 0)."""
    class Memory:
        state, reacq, trail = "committed", None, None
        estimator = SimpleNamespace(observe=lambda *a: None)

        def step(self, *a):
            return None

        def memory_bearing(self, *a):
            return (0.2, "estimate")
    g = TS.FakeGrounder()
    p = _pilot(Memory(), g)
    p._last_fwd, p._t_seen = -1.5, 9.0
    body, info = p.step(10.0, 8.0, pose=(0.0, 0.0, 0.0))
    k = 1.0 - 1.0 / float(p.law["coast_s"])
    assert info["law"] == "memory" and info["mode"] == "coast", info
    assert abs(body[0] - (-1.5 * k)) < 1e-12 and body[0] < 0.0 and body[1] == 0.0, body


def test_the_hud_claims_target_locked_only_on_a_fresh_box():
    """--save-view: an estimate tick with no fresh box reads TRACKING
    (PREDICTED) and a memory tick COASTING on last motion, as fly()'s HUD
    words them (hud_target); TARGET LOCKED only with a fresh box held. Every
    other pilot (no law) reads as before; the rerendered rows agree."""
    from guardrail.models import State
    tw = pn.track_words
    assert tw("track", True, "estimate", False) == "TRACKING (PREDICTED)"
    assert tw("track", True, "estimate", True) == "TARGET LOCKED"
    assert tw("track", True, "box", True) == "TARGET LOCKED"
    assert tw("coast", True, "memory", False) == "COASTING on last motion"
    assert tw("search", True, "sweep", False) == "SEARCHING ..."
    assert tw("track", True) == "TARGET LOCKED" and tw("track", False) == pn.TRACK_NO_LOCK
    hud = pn.RailHud(None, query="a red car", shield_on=True)
    st = State(x=0.0, y=0.0, up=8.0)

    def label(info):
        return hud.tick(0.0, decision=None, state=st, yaw_rad=0.0, info=info,
                        fsm_state=pn.FSMState.NORMAL)["mode_label"]
    assert label({"mode": "track", "law": "estimate", "fresh": False}) == \
        "TRACKING (PREDICTED) | FSM Normal"
    assert label({"mode": "track", "law": "estimate", "fresh": True}) == \
        "TARGET LOCKED | FSM Normal"
    s = hud.summary()
    assert s["track_label_ticks"] == 1 and \
        s["law_label_ticks"] == {"TRACKING (PREDICTED)": 1}, s
    import rerender_policy_hud as rr
    row = {"t": 1.0, "up": 8.0, "x": 0.0, "y": 0.0, "emitted": {"vx": 1.0, "vy": 0.0},
           "mode": "track", "follow_law": "estimate", "subject": None}
    assert rr.status_lines(row, True)[-1] == "TRACKING (PREDICTED)"
    assert rr.status_lines(dict(row, subject=[1.0, 2.0]), True)[-1] == "TARGET LOCKED"
    assert rr.status_lines({k: v for k, v in row.items() if k != "follow_law"},
                           True)[-1] == "TARGET LOCKED"


# =========================================================================== #
# 2. On a scripted car, closed loop
# =========================================================================== #

def _closed_loop(pilot, g, track=None, *, t0=100.0, seconds=60.0,
                 hidden=lambda t: False, car=TS._car):
    """The car of test_subject_track (3.2 m/s along +x from 25 m; `car`
    another), an inference every 0.25 s; the aircraft flies the pilot's body
    command (forward and right of its heading, as guardrail.frames.from_body
    turns them, and the yaw rate), from 20 m behind the car."""
    x, y, yaw = car(t0)[0] - 20.0, 0.0, 0.0
    out = []
    next_inf = t0
    for k in range(int(round(seconds / DT))):
        t = round(t0 + DT * k, 6)
        if t >= next_inf - 1e-9:
            g.infer(t, (x, y, 8.0, yaw), [] if hidden(t) else [(car(t), "ok")],
                    held=not hidden(t))
            next_inf += 0.25
        if track is None:
            body, info = pilot.step(t, 8.0)
        else:
            body, info = pilot.step(t, 8.0, pose=(x, y, yaw))
        fwd, right, _vz, yr = body
        yaw = (yaw + yr * DT + math.pi) % (2 * math.pi) - math.pi
        x += (fwd * math.cos(yaw) - right * math.sin(yaw)) * DT
        y += (fwd * math.sin(yaw) + right * math.cos(yaw)) * DT
        cx, cy = car(t)
        out.append({"t": t - t0, "law": info.get("law"), "mode": info["mode"],
                    "range": math.hypot(cx - x, cy - y), "hidden": hidden(t),
                    "x": x, "y": y, "right": right, "trail_dir": info.get("trail_dir")})
    return out


def test_on_a_scripted_car_the_estimate_holds_the_stand_off_through_the_gaps():
    def hidden(t):
        m = t - 100.0
        return 20.0 <= m < 22.5 or 35.0 <= m < 41.0
    tr, g = TS._started()
    p = _pilot(tr, g)
    rows = _closed_loop(p, g, tr, hidden=hidden)
    laws = {k: sum(1 for r in rows if r["law"] == k) for k in pn.ESTIMATOR_FOLLOW_LAWS}
    assert laws["estimate"] > 0.8 * len(rows), laws
    # the short gap (2.5 s, inside the estimator's 3 s coast): flown on the
    # estimate on every tick but those right at its end
    short = [r for r in rows if 20.0 <= r["t"] < 22.5]
    assert sum(r["law"] == "estimate" for r in short) >= len(short) - 3, short
    # the long gap (6 s, past the 3 s lapse): the Reacquirer takes it back
    s = tr.summary()
    assert s["lapses"] >= 1 and s["reacquisitions"] >= 1 and tr.state == "committed", s
    rng = [r["range"] for r in rows]
    assert max(rng) < 30.0, max(rng)
    settled = sorted(r["range"] for r in rows if 10.0 <= r["t"] < 20.0)
    med = settled[len(settled) // 2]
    assert abs(med - p.want_range_m) < 3.0, (med, p.want_range_m)
    summ = p.summary()
    assert summ["ticks"] == len(rows) and summ["estimate_frac"] > 0.8 and \
        summ["track_errors"] == 0, summ


def test_the_box_servo_null_on_the_same_car_falls_behind():
    """C2's first null (FollowPilot at the node's defaults: want_width 0.10,
    speed_max 4.0, the box only) on the same car and the same gaps: the
    reason a null with the lab's estimator was owed."""
    def hidden(t):
        m = t - 100.0
        return 20.0 <= m < 22.5 or 35.0 <= m < 41.0
    g = TS.FakeGrounder()
    g.acquiring = False
    servo = pn.FollowPilot(g, 90.0, yaw_gain=1.2, want_w_frac=0.10, speed_max=4.0)
    rows_s = _closed_loop(servo, g, hidden=hidden)
    tr, g2 = TS._started()
    rows_e = _closed_loop(_pilot(tr, g2), g2, tr, hidden=hidden)

    def within(rows):
        return sum(r["range"] <= 30.0 for r in rows) / len(rows)
    assert within(rows_e) == 1.0
    assert within(rows_s) < within(rows_e), (within(rows_s), within(rows_e))


def _turning_car(t, t0=100.0):
    """3.2 m/s along +x from 25 m for 15 s, then a right-angle turn to +y."""
    m = t - t0
    if m < 15.0:
        return (25.0 + 3.2 * m, 0.0)
    return (25.0 + 3.2 * 15.0, 3.2 * (m - 15.0))


def _off_the_cars_path(x, y, cx=73.0):
    """Distance from the turning car's L-shaped path (y = 0 to x = cx, then
    x = cx)."""
    d1 = abs(y) if x <= cx else math.hypot(x - cx, y)
    d2 = abs(x - cx) if y >= 0.0 else math.hypot(x - cx, y)
    return min(d1, d2)


def test_on_a_turning_car_the_trail_keeps_the_aircraft_on_the_cars_path():
    """The trail direction acting closed loop: round a right-angle corner the
    aircraft flies the car's path more closely than along the nose (which
    cuts the corner), within 30 m throughout, no direction refused."""
    def run(trail_follow):
        tr, g = TS._started(car=_turning_car,
                            pose=lambda t: (_turning_car(t)[0] - 20.0, 0.0, 8.0, 0.0))
        p = _pilot(tr, g, trail_follow=trail_follow)
        return p, _closed_loop(p, g, tr, seconds=40.0, car=_turning_car)
    p_on, on = run(True)
    p_off, off = run(False)
    assert p_on.n_trail > 0.8 * len(on) and p_on.n_trail_against == 0, p_on.summary()
    assert p_off.n_trail == 0 and all(r["right"] == 0.0 for r in off)
    assert max(r["range"] for r in on) < 30.0
    cut_on = max(_off_the_cars_path(r["x"], r["y"]) for r in on)
    cut_off = max(_off_the_cars_path(r["x"], r["y"]) for r in off)
    assert cut_on < 0.7 * cut_off, (cut_on, cut_off)


def test_with_no_estimate_the_box_law_flies_every_tick():
    """No subject committed (no estimate, no trail): the box law - follow_vlm.
    servo at the metric stand-off's box fraction - flies every tick, along
    the nose."""
    g = TS.FakeGrounder()
    tr = ST.SubjectTrack(g, "car", object_width_m=4.0, fv=FV)
    p = _pilot(tr, g)
    rows = _closed_loop(p, g, tr, seconds=20.0)
    assert {r["law"] for r in rows} == {"box"}, {r["law"] for r in rows}
    assert all(r["right"] == 0.0 and r["trail_dir"] is None for r in rows)
    assert p.summary()["ticks_by_law"]["box"] == len(rows) and p.n_trail == 0


# =========================================================================== #
# 3. The node
# =========================================================================== #

def _servo_run(*extra, max_s="32"):
    """One servo-pilot follow of perception_node.run on the stand-ins: the
    link whose heading follows the yaw rate (test_pas_vla.VLALink), the
    scripted Grounder of test_subject_track (a car driving North, hidden
    for 1.5 s and later 6 s) and the truth camera that reads it."""
    import autopilot as ap
    import test_pas_ardupilot as T
    import test_pas_vla as V
    out = T._tmp()
    pol = out / "policy.yaml"
    pol.write_text(T.CAP_POLICY, encoding="utf-8")
    args = pn.build_parser().parse_args([
        "--out-root", str(out), "--tag", "run", "--policy", str(pol), "--max-s", max_s,
        "--hfov", "90", "--citymap", str(out / "no_map.npz"), "--network", "nat",
        "--windows-ip", T.WIN, "--wsl-ip", T.WSL, "--shield", "on", "--dev-yaml",
        "--start-when-seen", "0", "--pilot", "servo", "--mission", "follow", *extra])
    clk = T.Clock()
    anchor = ap.FrameAnchor(*V.ANCHOR, "scene.home-geo-point")
    holder = {}
    link = V.VLALink(clk, anchor)
    holder["link"] = link
    g = TS.RunGrounder(clk, holder)
    rc = pn.run(args, link=link, camera=TS._truth_camera(link, g), grounder=g,
                obs=T.FakeObs(), clock=clk, sleep=clk.sleep)
    run = out / "run"
    rows = [json.loads(ln) for ln in (run / "flight_log.jsonl").read_text().splitlines()]
    m = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
    return rc, m, rows, g


def test_the_node_flies_the_estimator_follow_and_records_it():
    rc, m, rows, g = _servo_run("--identity", "--target-estimator", "--want-width", "0.16",
                                "--speed-max", "5")
    assert rc == 0, rc
    p = m["params"]
    assert (p["pilot"], p["mission"], p["identity"], p["target_estimator"]) == \
        ("servo", "follow", True, True), p
    ef = m["estimator_follow"]
    assert ef["on"] and ef["track_errors"] == 0 and ef["ticks"] == len(rows), ef
    assert abs(ef["config"]["want_range_m"] - 15.83) < 0.01 and ef["config"]["speed_max"] == 5.0
    assert ef["ticks_by_law"]["estimate"] > 0.5 * len(rows), ef
    assert ef["trail_ticks"] > 0 and ef["config"]["trail_follow"] is True, ef
    s = m["subject_track"]
    assert s["on"] and s["errors"] == 0 and s["started"] and s["lapses"] >= 1 \
        and s["reacquisitions"] >= 1, s
    assert s["truth"]["est_lock_frac"] > 0.5, s["truth"]
    # every row: the law and the estimator's fields, est_gap_m against the truth
    keys = ("follow_law", "est_state", "est_xy", "est_age_s", "est_memory", "lock_why",
            "reacq_streak")
    assert all(all(k in r for k in keys) for r in rows), sorted(rows[0])
    assert any(r.get("est_gap_m") is not None for r in rows)
    assert {r["follow_law"] for r in rows} <= set(pn.ESTIMATOR_FOLLOW_LAWS)
    # the Shield's subject is still the fresh box only, never the estimate
    between = [r for r in rows if r["follow_law"] == "estimate" and r["subject"] is None]
    assert between, "no tick flew on the estimate without a fresh box"
    # the Grounder was committed to each pick and leaned on the estimate
    assert any(c[0] == "commit" and c[1] is not None for c in g.calls)
    assert any(isinstance(pr, dict) for pr in g.priors)
    assert m["frac_within_30m"] is not None and m["frac_within_30m"] > 0.9, \
        m["frac_within_30m"]


def test_without_the_estimator_the_servo_pilot_is_the_box_servo_unchanged():
    rc, m, rows, g = _servo_run("--identity", max_s="8")
    assert rc == 0, rc
    assert m["params"]["target_estimator"] is False
    assert "estimator_follow" not in m and "subject_track" not in m, sorted(m)
    assert rows and not any("follow_law" in r or "est_state" in r for r in rows)
    assert not any(c[0] == "commit" for c in g.calls) and not g.priors
    # the node refuses the estimator follow without --identity, as the VLA's
    try:
        pn.check_args(pn.build_parser().parse_args(["--network", "mirrored",
                                                    "--target-estimator"]))
    except SystemExit as exc:
        assert "--identity" in str(exc)
    else:
        raise AssertionError("--target-estimator without --identity was accepted")


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
        except Exception as e:                        # noqa: BLE001
            failed += 1
            import traceback
            traceback.print_exc()
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
