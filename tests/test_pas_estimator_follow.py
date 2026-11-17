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
2. ON A SCRIPTED CAR, CLOSED LOOP. A car driving at 3.2 m/s, hidden for
   2.5 s and later for 6 s, followed from 20 m: the estimate flies most
   ticks and every hidden tick of the short gap, the aircraft holds near
   the stand-off and within 30 m throughout, and the long gap is a lapse
   the Reacquirer takes back; the box-width servo of C2's first null
   (FollowPilot at its node defaults) on the same car falls behind.
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


# =========================================================================== #
# 2. On a scripted car, closed loop
# =========================================================================== #

def _closed_loop(pilot, g, track=None, *, t0=100.0, seconds=60.0,
                 hidden=lambda t: False):
    """The car of test_subject_track (3.2 m/s along +x from 25 m), an
    inference every 0.25 s; the aircraft flies the pilot's body command
    (forward along its heading, the yaw rate), from 20 m behind the car."""
    x, y, yaw = TS._car(t0)[0] - 20.0, 0.0, 0.0
    out = []
    next_inf = t0
    for k in range(int(round(seconds / DT))):
        t = round(t0 + DT * k, 6)
        if t >= next_inf - 1e-9:
            g.infer(t, (x, y, 8.0, yaw), [] if hidden(t) else [(TS._car(t), "ok")],
                    held=not hidden(t))
            next_inf += 0.25
        if track is None:
            body, info = pilot.step(t, 8.0)
        else:
            body, info = pilot.step(t, 8.0, pose=(x, y, yaw))
        fwd, _side, _vz, yr = body
        yaw = (yaw + yr * DT + math.pi) % (2 * math.pi) - math.pi
        x += fwd * math.cos(yaw) * DT
        y += fwd * math.sin(yaw) * DT
        cx, cy = TS._car(t)
        out.append({"t": t - t0, "law": info.get("law"), "mode": info["mode"],
                    "range": math.hypot(cx - x, cy - y), "hidden": hidden(t)})
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
