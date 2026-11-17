"""demo/subject_track.SubjectTrack - follow_vlm.fly()'s target estimator,
strict-lock prior and re-acquisition as one object - and its use by the
Project AirSim + ArduPilot perception node's VLA follow (--target-estimator),
checked without the simulator, ArduPilot, the GPU or a model.

Run either way:
    pytest tests/test_subject_track.py -v
    python tests/test_subject_track.py

WHAT IS BEING CLAIMED (2026-11-15)

1. ONE SET OF RULES. The decisions fly()'s identity path makes about its
   subject - the estimator and the trail it is built with, a commit's seed,
   which SOFT box may feed the estimate, where a lapse anchors the
   re-acquisition, the strict lock's prior - are follow_vlm functions that
   fly() calls, unchanged in what they compute, and SubjectTrack calls the
   same ones (no copy of their arithmetic in either).
2. Between boxes the estimate PREDICTS the subject (a car at 3.2 m/s hidden
   for 2.5 s is served within 1.5 m of where it is), the strict lock is
   given the estimator's snapshot every tick there is one, and a pilot
   without a box is pointed at the prediction ("estimate").
3. After lapse_s without an accepted measurement the estimate is dropped,
   the Grounder re-acquires and follow_vlm.Reacquirer is anchored where the
   subject was predicted, with its measured speed; meanwhile the pilot is
   pointed along the trail ("trail") while the anchor still says where the
   subject is, and at nothing after. The car reappearing within reach is
   committed in `need` inferences and the estimate re-seeded there; a
   static OK box out of reach, or a SOFT box, never is.
4. A SOFT box only continues a live track; with no pick at the start the
   subject is found by the unanchored Reacquirer, which wants it moving.
5. vla_mission.DetectorHint's `estimate`: with no fresh box the memory hint
   aims at the predicted subject while it is given, then the sighting's
   memory and search turn toward where it was last predicted; without it
   every hint and its stats are as before.
6. The node: --target-estimator is refused where it cannot run as
   follow_vlm runs it; the Grounder holds follow_vlm's strict lock with it
   (and no lock with --identity alone, as before); VLAPilot steps the track
   before the backend acts, logs its fields and survives its faults; a VLA
   follow flown on the stand-ins records est_* fields on every row, the
   estimate against the truth, the lapses and re-acquisitions, the memory
   hints by source - and the Shield's subject is still the fresh box only.
"""
from __future__ import annotations

import ast
import inspect
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
import subject_track as ST                                           # noqa: E402
import vla_mission as VM                                             # noqa: E402
from guardrail.vla_link import Capture, Pose                          # noqa: E402
from target_state import TargetState                                 # noqa: E402

W, H = 768, 432
JPG = b"\xff\xd8\xff\xe0fake-jpeg\xff\xd9"


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _cand(xy, pose, score=0.05):
    """A detector candidate of a 4 m car at world `xy` seen from `pose`
    (x, y, up, yaw): the 8-tuple box and identity's features of it (P, the
    horizontal range, the world bearing of its centre ray)."""
    px, py, up, yaw = pose
    dx, dy = xy[0] - px, xy[1] - py
    rng = math.hypot(dx, dy)
    brg = math.atan2(dy, dx)
    rel = max(-1.4, min(1.4, _wrap(brg - yaw)))
    cx = W / 2.0 + (W / 2.0) * math.tan(rel)
    w = (W / 2.0) * 4.0 / max(rng, 1.0)
    det = (cx, H * 0.6, w, 0.4 * w, score, W, H, 0.6)
    feat = {"r": math.hypot(rng, up - 0.75), "rng_h": rng, "bearing": brg,
            "P": [float(xy[0]), float(xy[1]), 0.75]}
    return det, feat


def _pose_d(pose):
    return {"x": pose[0], "y": pose[1], "up": pose[2], "yaw": pose[3], "pitch": 0.0,
            "roll": 0.0, "src": "ring"}


class FakeGrounder:
    """follow_vlm.Grounder's identity-mode interface, scripted. infer()
    publishes one inference: every candidate (box, tier, features) with its
    capture; while committed (not acquiring) the FIRST candidate, when
    `held`, is THE box (the strict lock holding the car), else the last box
    stays published with its old t_det, as the Grounder keeps it through
    misses. begin_reacquire() / commit() / set_prior() are recorded."""

    def __init__(self):
        self.seq = 0
        self.acquiring = False
        self.calls: list = []
        self.priors: list = []
        self.lock = None
        self._g = {"seq": 0, "t": None, "det": None, "t_det": 0.0, "cands": [],
                   "cand_tiers": [], "cand_feats": [], "tier": None, "feat": None,
                   "cap": None, "lock_why": None, "inf_cap": None}

    def latest(self):
        return dict(self._g)

    def commit(self, cand, yaw=None):
        self.calls.append(("commit", cand, yaw))
        self.acquiring = False

    def begin_acquire(self):
        self.calls.append(("begin_acquire",))
        self.acquiring = True
        self._g.update(det=None, t_det=0.0)

    def begin_reacquire(self):
        """Grounder.begin_reacquire: begin_acquire with no estimator prior."""
        self.calls.append(("begin_reacquire",))
        self.priors.append(None)
        self.acquiring = True
        self._g.update(det=None, t_det=0.0)

    def set_prior(self, snap):
        self.priors.append(snap)

    def infer(self, t, pose, cands=(), held=True):
        """`cands`: [(xy, tier)], the first one the subject."""
        self.seq += 1
        boxes, tiers, feats = [], [], []
        for xy, tier in cands:
            d, f = _cand(xy, pose)
            boxes.append(d)
            tiers.append(tier)
            feats.append(f)
        cap = {"t_capture": t - 0.05, "t_cap": t - 0.05, "pose": _pose_d(pose)}
        self._g.update(seq=self.seq, t=t, cands=boxes, cand_tiers=tiers, cand_feats=feats,
                       inf_cap=dict(cap), lock_why=None)
        if not self.acquiring and boxes and held:
            self._g.update(det=boxes[0], t_det=t, tier=tiers[0], feat=feats[0],
                           cap={**cap, "range_m": feats[0]["r"], "depth_src": "capture"})
        elif not self.acquiring:
            self._g["lock_why"] = "nothing where the subject should be"
        return self._g


def _car(t, t0=100.0, x0=25.0, v=3.2):
    return (x0 + v * (t - t0), 0.0)


def _chase(t, t0=100.0):
    """The aircraft, 20 m behind the car, facing it."""
    return (_car(t, t0)[0] - 20.0, 0.0, 8.0, 0.0)


def _fly(track, g, t_from, t_to, *, car=_car, pose=_chase, hidden=lambda t: False,
         extra=lambda t: (), tier="ok", rows=None, infer_dt=0.25):
    """Ticks at 10 Hz from t_from to t_to, an inference every infer_dt: the
    car (unless hidden) and `extra(t)` candidates; each tick steps the track
    and logs its row. Returns the rows."""
    rows = [] if rows is None else rows
    n = int(round((t_to - t_from) / 0.1))
    next_inf = t_from
    for k in range(n):
        t = round(t_from + 0.1 * k, 6)
        p = pose(t)
        if t >= next_inf - 1e-9:
            cs = ([] if hidden(t) else [(car(t), tier)]) + list(extra(t))
            # Only the car is ever THE box (the strict lock holds nothing else).
            g.infer(t, p, cs, held=not hidden(t))
            next_inf += infer_dt
        track.step(g.latest(), t, (p[0], p[1], p[3]))
        rows.append({"t": t, **track.row(t, p[0], p[1], p[3])})
    return rows


def _started(t0=100.0, car=_car, pose=_chase):
    """A track committed at t0 to the start gate's pick: the car's features
    at its capture."""
    g = FakeGrounder()
    tr = ST.SubjectTrack(g, "car", object_width_m=4.0, fv=FV)
    p = pose(t0)
    _d, f = _cand(car(t0), p)
    tr.start(f, {"t_capture": t0 - 0.05, "pose": _pose_d(p)}, now=t0, t0=t0)
    return tr, g


# =========================================================================== #
# 1. One set of rules
# =========================================================================== #

RULES = ("subject_estimator", "subject_trail", "seed_estimate", "soft_may_continue",
         "lapse_anchor", "estimator_prior")


def test_fly_and_the_track_call_the_same_follow_vlm_rules():
    """fly() calls every extracted rule (and no longer carries their
    arithmetic: the SOFT ages, the seed's bearing, the prior's dict, the
    anchor's fallbacks); SubjectTrack calls them through follow_vlm, with
    the Reacquirer, estimator_measurement and measured_speed."""
    src = (ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    fly = next(n for n in tree.body if isinstance(n, ast.AsyncFunctionDef) and n.name == "fly")
    called = {n.func.id for n in ast.walk(fly)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
    assert set(RULES) <= called, sorted(set(RULES) - called)
    body = ast.get_source_segment(src, fly)
    for inline in ("est_age < 1.5", "Trail(spacing_m=1.5", "TargetState(v_max=",
                   "b0 = pf[\"bearing\"]", "\"t_upd\": estimator.t_last_update",
                   "have_est = estimator"):
        assert inline not in body, inline
    track = inspect.getsource(ST)
    for name in RULES + ("Reacquirer(", "estimator_measurement(", "measured_speed(",
                         "implied_range_from_width("):
        assert f"fv.{name.rstrip('(')}" in track or f"fv.{name}" in track, name
    # ...and its numbers are follow_vlm's parser's.
    ap = FV.build_arg_parser()
    tr = ST.SubjectTrack(FakeGrounder(), "car", fv=FV)
    assert (tr.lapse_s, tr.reacq_need, tr.reacq_max_range_m) == (
        ap.get_default("lapse_s"), ap.get_default("reacq_need"),
        ap.get_default("reacq_max_range_m")) == (3.0, 4, 45.0)
    assert tr.estimator.v_max == FV.SUBJECT_VMAX_MPS["car"]


def test_the_extracted_rules_compute_what_fly_computed():
    """seed_estimate, soft_may_continue, lapse_anchor and estimator_prior,
    each against the expression fly() carried before 2026-11-15."""
    est = TargetState()
    pose = {"x": 10.0, "y": 5.0, "yaw": 0.3}
    pf = {"rng_h": 20.0, "bearing": 0.5, "P": [99.0, 99.0, 0.75]}
    seeded, P0 = FV.seed_estimate(est, pf, pose, 50.0)
    assert seeded and abs(P0[0] - (10 + 20 * math.cos(0.5))) < 1e-6
    assert abs(P0[1] - (5 + 20 * math.sin(0.5))) < 1e-6 and est.t_last_update == 50.0
    # No range: not seeded, the sighting's own map point, the estimator reset.
    seeded, P0 = FV.seed_estimate(est, {"P": [3.0, 4.0, 0.0]}, pose, 51.0)
    assert not seeded and P0 == (3.0, 4.0) and est._xu is None
    assert FV.seed_estimate(None, pf, pose, 1.0) == (False, (99.0, 99.0))
    assert FV.seed_estimate(est, {}, None, 1.0) == (False, None)
    # SOFT: the estimate updated under 1.5 s before the box, an OK one under 3 s ago.
    assert FV.soft_may_continue(10.0, 8.6, 20.0, 17.1)
    assert not FV.soft_may_continue(10.0, 8.5, 20.0, 17.1)
    assert not FV.soft_may_continue(10.0, 8.6, 20.0, 17.0)
    assert not FV.soft_may_continue(10.0, None, 20.0, 19.9)
    # The lapse's anchor: the posterior when there is one...
    est.update(60.0, 0.0, 0.0, 0.0, 0.0, 30.0)
    a = FV.lapse_anchor(est, (1.0, 1.0), 55.0, 56.0, 3.2, 2.0)
    assert a == {"t": 60.0, "P": (30.0, 0.0), "v": 3.2}, a
    est.reset()
    # ...else the last accepted place, at its measurement's capture.
    assert FV.lapse_anchor(est, (1.0, 1.0), 55.0, 56.0, None, 2.0) == \
        {"t": 55.0, "P": (1.0, 1.0), "v": 2.0}
    assert FV.lapse_anchor(est, (1.0, 1.0), None, 56.0, None, None)["t"] == 56.0
    assert FV.lapse_anchor(None, None, None, None, None, None) is None
    # The strict lock's snapshot.
    assert FV.estimator_prior(est, 0.75) is None
    est.update(70.0, 0.0, 0.0, 0.0, 0.0, 10.0)
    assert FV.estimator_prior(est, 0.9) == {"t": 70.0, "x": 10.0, "y": 0.0, "vx": 0.0,
                                             "vy": 0.0, "t_upd": 70.0, "up": 0.9}
    t = FV.subject_trail()
    assert (t.spacing_m, t.max_len_m) == (1.5, 120.0)
    assert FV.subject_estimator("pedestrian").v_max == FV.SUBJECT_VMAX_MPS["pedestrian"]


# =========================================================================== #
# 2. Between boxes: the prediction
# =========================================================================== #

def test_between_boxes_the_estimate_predicts_the_car_and_the_pilot_is_pointed_there():
    tr, g = _started()
    rows = _fly(tr, g, 100.0, 104.0)
    hid = _fly(tr, g, 104.0, 106.5, hidden=lambda t: True)
    assert tr.state == "committed" and tr.n_events["lapse"] == 0
    assert tr.counts["accepted"] >= 14 and abs(tr.estimator.speed() - 3.2) < 0.4
    last = hid[-1]
    car = _car(last["t"])
    assert last["est_xy"] is not None, last
    err = math.hypot(last["est_xy"][0] - car[0], last["est_xy"][1] - car[1])
    assert err < 1.5, (err, last, car)
    assert last["est_memory"] == "estimate" and 2.0 < last["est_age_s"] < 3.0, last
    p = _chase(last["t"])
    b, src = tr.memory_bearing(last["t"], p[0], p[1], p[3])
    assert src == "estimate" and abs(b) < math.radians(3), math.degrees(b)
    # The strict lock leaned on the estimate every tick there was one.
    assert g.priors and all(isinstance(s, dict) and s["up"] == 0.75 for s in g.priors), \
        g.priors[:3]
    # And the car seen again is taken with no lapse in between.
    _fly(tr, g, 106.5, 108.0)
    assert tr.state == "committed" and tr.n_events == {"acquired": 1}
    assert all(r["est_state"] == "committed" for r in rows + hid)


# =========================================================================== #
# 3. The lapse and the anchored re-acquisition
# =========================================================================== #

def test_a_lapse_reacquires_where_the_car_was_predicted():
    tr, g = _started()
    _fly(tr, g, 100.0, 104.0)
    pre = (float(tr.estimator._xu[0]), float(tr.estimator._xu[1]))
    rows = _fly(tr, g, 104.0, 110.0, hidden=lambda t: True)
    lapse = next(e for e in tr.events if e["event"] == "lapse")
    # the last box consumed at 103.75, the first tick more than 3 s after it
    assert 6.75 < lapse["t"] < 6.95 and lapse["unseen_s"] > 3.0, lapse
    assert math.hypot(lapse["P"][0] - pre[0], lapse["P"][1] - pre[1]) < 0.5, (lapse, pre)
    assert abs(lapse["v"] - 3.2) < 0.4, lapse
    assert ("begin_reacquire",) in g.calls and g.priors[-1] is None
    assert tr.state == "reacquiring" and tr.estimator._xu is None
    # Meanwhile the pilot is pointed along the trail, past the last sighting.
    re = [r for r in rows if r["est_state"] == "reacquiring"]
    assert re and all(r["est_memory"] == "trail" and r["est_xy"] is None for r in re), re[:2]
    assert all(r["est_memory"] == "estimate" for r in rows if r["t"] < 106.0)
    # The car back in view, within reach: committed in `need` inferences.
    back = _fly(tr, g, 110.0, 112.0)
    ev = next(e for e in tr.events if e["event"] == "reacquired")
    assert ev["streak"] == 4 and ev["seeded"] and 10.6 < ev["t"] < 11.2, ev
    car = _car(100.0 + ev["t"])
    assert math.hypot(ev["P"][0] - car[0], ev["P"][1]) < 1.5, (ev, car)
    commits = [c for c in g.calls if c[0] == "commit"]
    assert len(commits) == 1 and commits[0][1] is not None
    assert tr.state == "committed" and back[-1]["est_xy"] is not None
    s = tr.summary()
    assert (s["lapses"], s["reacquisitions"], s["commits"]) == (1, 1, 2), s
    assert s["ticks_by_state"]["reacquiring"] == len(re) + sum(
        1 for r in back if r["est_state"] == "reacquiring")
    assert s["memory_ticks"]["trail"] >= len(re) and s["est_age_s"]["n"] > 0


def test_a_static_box_out_of_reach_or_a_soft_box_is_never_taken_back():
    """After the lapse only an OK box 70 m ahead of the anchor (out of
    reach) and a SOFT box where the car is: nothing is committed, the
    refusals are counted, and once the reach has grown past reach_cap_m
    the pilot is pointed at nothing."""
    tr, g = _started()
    _fly(tr, g, 100.0, 104.0)
    anchor_x = float(tr.estimator._xu[0])
    far = (anchor_x + 70.0, 0.0)
    _fly(tr, g, 104.0, 118.0, tier="soft", extra=lambda t: [(far, "ok")],
         hidden=lambda t: t < 107.5)
    assert not [c for c in g.calls if c[0] == "commit"] and tr.state == "reacquiring"
    s = tr.summary()
    assert s["reacq_refused"].get("not ok", 0) > 10, s["reacq_refused"]
    assert s["reacq_refused"].get("out of reach", 0) + s["reacq_refused"].get("too far", 0) \
        > 10, s["reacq_refused"]
    p = _chase(118.0)
    assert not tr.in_memory(118.0) and tr.memory_bearing(118.0, p[0], p[1], 0.0) is None


# =========================================================================== #
# 4. SOFT boxes, and the start with no pick
# =========================================================================== #

def test_a_soft_box_only_continues_a_live_track():
    tr, g = _started()
    _fly(tr, g, 100.0, 102.0)
    _fly(tr, g, 102.0, 108.0, tier="soft")
    c = tr.counts
    # Fed while an OK box was accepted under 3 s ago (the last at 101.75),
    # refused after; with nothing accepted for 3 s more, the subject lapses.
    assert c["soft_fed"] == 11 and c["soft_refused"] >= 10, dict(c)
    assert tr.n_events["lapse"] == 1 and tr.state == "reacquiring", dict(tr.n_events)


def test_with_no_pick_the_subject_is_found_only_moving():
    """start(None): re-acquiring with no anchor (fly() when its gate did
    not confirm one); a static OK box is never taken (marked static), the
    pilot is pointed at nothing; a moving car is."""
    g = FakeGrounder()
    tr = ST.SubjectTrack(g, "car", object_width_m=4.0, fv=FV)
    tr.start(None, None, now=100.0, t0=100.0)
    assert tr.state == "reacquiring" and tr.reacq.anchor is None
    assert g.calls == [("begin_reacquire",)]
    pose = lambda t: (0.0, 0.0, 8.0, 0.0)                         # noqa: E731
    rows = _fly(tr, g, 100.0, 110.0, car=lambda t: (20.0, 3.0), pose=pose)
    assert tr.state == "reacquiring" and "static" in tr.summary()["reacq_refused"]
    assert all(r["est_memory"] is None for r in rows)
    _fly(tr, g, 110.0, 116.0, car=lambda t: (15.0 + 2.5 * (t - 110.0), -6.0), pose=pose)
    assert tr.state == "committed" and tr.n_events["reacquired"] == 1, tr.events


# =========================================================================== #
# 5. The VLA's hint
# =========================================================================== #

def _hint(estimate=None):
    from vla_bridge import VLABridge
    box = {"v": (None, None, None, None)}
    dh = VM.DetectorHint(lambda: box["v"], VLABridge("a red car", hfov_deg=90.0),
                         fresh_s=1.0, memory_s=6.0, search_s=20.0, record_kind=True,
                         estimate=estimate)
    return dh, box


def _cap(t, yaw=0.0):
    return Capture(t, Pose(0.0, 0.0, 8.0, yaw), JPG)


def test_the_memory_hint_aims_at_the_estimate_and_the_sighting_follows_it():
    est = {"v": None}
    dh, box = _hint(estimate=lambda cap: est["v"])
    plain, pbox = _hint()
    det = (384.0, 260.0, 30.0, 12.0, 0.1, W, H, 0.6)            # dead ahead
    box["v"] = pbox["v"] = (det, 100.0, 0.0, 100.05)
    for h in (dh, plain):
        assert h(_cap(100.2)).kind == "fresh"
    # No fresh box: the estimate's bearing, 60 deg right, is the memory.
    est["v"] = (math.radians(60.0), "estimate")
    h, hp = dh(_cap(102.0)), plain(_cap(102.0))
    assert h.kind == hp.kind == "memory"
    assert dh.last_step.from_estimate and dh.last_memory_src == "estimate"
    assert h.phrase == dh.bridge.phrase_for_bearing(math.radians(60.0)) != hp.phrase
    assert hp.phrase == plain.bridge.phrase_for_bearing(0.0) and plain.last_memory_src == "sighting"
    assert abs(dh._seen[0] - math.radians(60.0)) < 1e-9 and dh._seen[1] == 100.0
    # The estimate gone: the sighting's memory, now toward where it was predicted.
    est["v"] = None
    h = dh(_cap(103.0))
    assert h.kind == "memory" and h.phrase == dh.bridge.phrase_for_bearing(math.radians(60.0))
    assert dh.last_memory_src == "sighting" and not dh.last_step.from_estimate
    # ...and the search past memory_s turns toward that side.
    h = dh(_cap(107.0))
    assert h.kind == "search" and dh.last_step.bearing_rad == math.pi / 2
    # The estimate holds the memory however long it is given (past memory_s).
    est["v"] = (math.radians(-20.0), "trail")
    h = dh(_cap(110.0))
    assert h.kind == "memory" and dh.last_memory_src == "trail"
    st = dh.stats()
    assert st["memory_by_source"] == {"estimate": 1, "sighting": 1, "trail": 1}, st
    assert "memory_by_source" not in plain.stats()
    assert plain.stats()["hints_by_kind"] == {"fresh": 1, "memory": 1}


def test_without_an_estimate_the_hint_rule_is_unchanged():
    """follow_hint_step with estimate=None and without the keyword give the
    same step on every branch."""
    from vla_bridge import VLABridge
    br = VLABridge("a red car", hfov_deg=90.0)
    det = (500.0, 260.0, 30.0, 12.0, 0.1, W, H, 0.6)
    seen = (0.4, 100.0)
    for box, t in (((det, 100.0, 0.1), 100.3), ((det, 100.0, 0.1), 102.0),
                   ((None, None, None), 104.0), ((None, None, None), 110.0),
                   ((None, None, None), 140.0)):
        a = VM.follow_hint_step(box, t, 0.2, seen, br, fresh_s=1.0, memory_s=6.0, search_s=20.0)
        b = VM.follow_hint_step(box, t, 0.2, seen, br, fresh_s=1.0, memory_s=6.0, search_s=20.0,
                                estimate=None)
        assert a == b and not a.from_estimate, (a, b)


# =========================================================================== #
# 6. The node
# =========================================================================== #

def _pn():
    import perception_node as pn
    return pn


def _args(*extra):
    return _pn().build_parser().parse_args(["--network", "mirrored", *extra])


VLA_FOLLOW = ("--pilot", "vla", "--mission", "follow", "--vla-server", "127.0.0.1:9")


def test_the_estimator_is_refused_where_it_cannot_run_as_follow_vlm_runs_it():
    pn = _pn()
    a = _args(*VLA_FOLLOW, "--identity", "--target-estimator")
    pn.check_args(a)
    assert a.target_estimator and a.mission == "follow"
    # The servo pilot's follow takes it too since 2026-11-17 (the estimator
    # follow, tests/test_pas_estimator_follow.py), on the same terms.
    s = _args("--identity", "--target-estimator")
    pn.check_args(s)
    assert s.pilot == "servo" and s.mission == "follow" and s.target_estimator
    for extra, word in ((VLA_FOLLOW + ("--target-estimator",), "--identity"),
                        (("--target-estimator",), "--identity"),
                        (("--pilot", "vla", "--mission", "approach", "--vla-server",
                          "127.0.0.1:9", "--identity", "--target-estimator"), "follow"),
                        (VLA_FOLLOW + ("--identity", "--target-estimator", "--no-lock-target"),
                         "--no-lock-target")):
        try:
            pn.check_args(_args(*extra))
        except SystemExit as exc:
            assert word in str(exc), (extra, str(exc))
        else:
            raise AssertionError(f"{extra} was accepted")
    # Off by default, and recorded with a VLA run's params.
    assert _args().target_estimator is False and "target_estimator" in pn.VLA_PARAMS


def test_the_grounder_holds_the_strict_lock_only_with_the_estimator():
    import tempfile
    pn = _pn()
    from test_pas_ardupilot import FakeObs
    with tempfile.TemporaryDirectory() as td:
        out = Path(td)
        base = VLA_FOLLOW + ("--identity", "--citymap", str(out / "no_map.npz"))
        g, rec = pn.build_grounder(_args(*base, "--target-estimator"), FakeObs(), out, FV, 90.0)
        assert isinstance(g.lock, FV.TargetLock) and g.identity is not None
        assert rec["lock"]["strict"] is True and "SubjectTrack" in rec["lock"]["prior"], rec
        g, rec = pn.build_grounder(_args(*base), FakeObs(), out, FV, 90.0)
        assert g.lock is None and rec["lock"].startswith("strict lock not used"), rec
        g, rec = pn.build_grounder(_args("--citymap", str(out / "no_map.npz")), FakeObs(),
                                   out, FV, 90.0)
        assert isinstance(g.lock, FV.TargetLock) and rec["lock"]["kind"] == "TargetLock"
        assert "strict" not in rec["lock"]


def test_the_vla_pilot_steps_the_track_before_the_backend_acts_and_survives_its_faults():
    pn = _pn()
    from guardrail.models import Action4D, State
    from guardrail.vla_link import Tick
    order = []

    class Track:
        def __init__(self, boom=False):
            self.boom = boom

        def step(self, g, now, pose):
            order.append(("step", g.get("seq"), round(pose[2], 3)))
            if self.boom:
                raise RuntimeError("bad estimate")

        def row(self, now, x, y, yaw):
            if self.boom:
                raise RuntimeError("bad row")
            return {"est_state": "committed", "est_xy": [1.0, 2.0]}

    class Backend:
        cadence = "pipelined"
        last_link_failure = None

        def __init__(self):
            self.tracker = SimpleNamespace(current_record=None, smoothing=False,
                                           cfg=SimpleNamespace(rate_horizon_s=None))
            self.counts = {"requests": 0, "no_hint": 0, "no_capture": 0, "request_errors": 0}
            self.last_tick = None

        def act(self, st, hold_horizontal=False):
            order.append(("act",))
            self.last_tick = Tick(Action4D(vx=1.0), "vla_step", "translate", seq=1)
            return self.last_tick.action

        def suspend(self):
            pass

        def resume(self):
            pass

    class Det:
        def latest(self):
            return {"det": None, "t_det": 0.0, "seq": 7}
    st = State(x=0.0, y=0.0, up=8.0, yaw_deg=90.0)
    dh = SimpleNamespace(fresh_s=1.0, last_kind=None, record_kind=True,
                         box_ignored=lambda t: False, last_box_age_s=None,
                         last_yaw_correction_rad=None, last_detector_latency_s=None,
                         last_memory_src="estimate")
    for boom in (False, True):
        order.clear()
        p = pn.VLAPilot(Backend(), VM.MissionMonitor(VM.MonitorConfig(mission="follow",
                                                                      lost_timeout_s=90.0), 0.0),
                        mission="follow", detections=Det(), detector_hint=dh,
                        hint_source=SimpleNamespace(last={"kind": None}), vm=VM,
                        track=Track(boom))
        raw, body, info = p.step(10.0, st)
        assert order == [("step", 7, round(math.pi / 2, 3)), ("act",)], order
        assert raw.vx == 1.0
        _end, row, _hud = p.after(10.0, st, flies=True, touched=False, shield_on=True,
                                  t_mission=10.0)
        assert row["hint_memory_src"] == "estimate"
        if boom:
            assert p.track_errors == 2 and "bad estimate" in p.track_error
            assert row["est_state"] is None
        else:
            assert p.track_errors == 0 and row["est_xy"] == [1.0, 2.0]


# ------------------------------------------------- a VLA follow on the stand-ins

class RunGrounder(FakeGrounder):
    """FakeGrounder on the flight's clock and the stand-in link's pose: an
    inference every 0.25 s of a car driving North at 1.6 m/s from 20 m
    ahead of the spawn, hidden for 1.5 s from 9 s and for 6 s from 15 s of
    the mission (counted from the track's start: its first
    begin_reacquire, or the start gate's commit)."""

    def __init__(self, clock, holder):
        super().__init__()
        self.clock, self.holder = clock, holder
        self.t_c0 = clock.t
        self.t_m0 = None
        self.next_t = None

    def car(self, t):
        return (55.0 + 1.6 * (t - self.t_c0), -20.0)

    def hidden(self, t):
        if self.t_m0 is None:
            return False
        m = t - self.t_m0
        return 9.0 <= m < 10.5 or 15.0 <= m < 21.0

    def commit(self, cand, yaw=None):
        if cand is not None and self.t_m0 is None:
            self.t_m0 = self.clock.t
        super().commit(cand, yaw)

    def begin_reacquire(self):
        if self.t_m0 is None:
            self.t_m0 = self.clock.t
        super().begin_reacquire()

    def latest(self):
        t = self.clock.t
        link = self.holder.get("link")
        if link is not None and (self.next_t is None or t >= self.next_t):
            self.next_t = t + 0.25
            pose = (link.x, link.y, link.up, link.yaw)
            self.infer(t, pose, [] if self.hidden(t) else [(self.car(t), "ok")],
                       held=not self.hidden(t))
        return super().latest()


def _truth_camera(link, grounder):
    import test_pas_ardupilot as T
    pn = _pn()

    class TruthCamera(T.FakeCamera):
        def object_xy(self, name):
            if name == pn.AIRCRAFT_ACTOR:
                return (self.link.x, self.link.y)
            return grounder.car(self.link.clock.t)
    return TruthCamera(link)


def _estimator_follow(*extra):
    import test_pas_vla as V
    import vla_fake_server as F
    holder = {}

    def mk_link(clk, anchor, **kw):
        holder["link"] = V.VLALink(clk, anchor, **kw)
        return holder["link"]
    made = {}

    def mk_grounder(clk):
        made["g"] = RunGrounder(clk, holder)
        return made["g"]

    class Cam:
        def __init__(self):
            self.cam = None

        def __getattr__(self, k):
            if self.cam is None:
                self.cam = _truth_camera(holder["link"], made["g"])
            return getattr(self.cam, k)
    r = V._follow(lambda req: F.step_reply(req, 0.8, 0.0, 0.0), grounder=mk_grounder,
                  link_cls=mk_link, camera=Cam(), max_s="32",
                  extra=("--identity", "--target-estimator") + tuple(extra))
    return r, made["g"]


def _check_estimator_follow(r, g, *, gate: bool):
    import test_pas_vla as V
    assert r.rc == 0, r.rc
    m = V._json(r.run, "metrics.json")
    rows = V._rows(r.run)
    assert m["params"]["target_estimator"] is True and m["params"]["identity"] is True
    fol = m["vla"]["follow"]["target_estimator"]
    assert fol["lapse_s"] == 3.0 and fol["reacq_need"] == 4 and fol["subject_class"] == "car"
    s = m["subject_track"]
    assert s["on"] and s["errors"] == 0 and s["started"], s
    ev = [e["event"] for e in s["events"]]
    assert ev[0] == ("acquired" if gate else "acquire"), ev
    assert "lapse" in ev and ev.count("reacquired") >= (1 if gate else 2), ev
    assert s["lapses"] >= 1 and s["reacquisitions"] >= 1 and s["state_at_end"] == "committed"
    # Every row carries the estimator's fields; the estimate is on the car.
    keys = ("est_state", "est_xy", "est_age_s", "est_speed_mps", "est_memory", "lock_why",
            "reacq_streak", "hint_memory_src")
    assert rows and all(all(k in x for k in keys) for x in rows), sorted(rows[0])
    tt = s["truth"]
    assert tt["truth_ticks"] >= len(rows) - 5 and tt["est_lock_frac"] > 0.5, tt
    assert tt["est_on_subject_frac"] > 0.95 and tt["est_gap_m"]["median"] < 2.0, tt
    # Between boxes the estimate is served, and the Shield is NOT given it.
    between = [x for x in rows if x["est_xy"] is not None
               and (x.get("det_age_s") is None or x["det_age_s"] > 1.0)]
    assert len(between) >= 10, len(between)
    assert all(x["subject"] is None for x in between)
    assert any(x["est_state"] == "reacquiring" and x["est_memory"] == "trail" for x in rows)
    # The VLA was asked with memory hints aimed at the estimate.
    dh = m["vla"]["mission"]["detector_hint"]
    assert dh["memory_by_source"].get("estimate", 0) >= 1, dh
    assert dh["hints_by_kind"].get("memory", 0) >= 1 and dh["hints_by_kind"]["fresh"] > 10
    assert any(x["hint_memory_src"] == "estimate" for x in rows)
    # The Grounder was committed to each pick and given the estimate.
    commits = [c for c in g.calls if c[0] == "commit" and c[1] is not None]
    assert len(commits) >= 2 and any(isinstance(p, dict) for p in g.priors)
    # The Shield still filtered every tick, on the unchanged path.
    assert all("emitted" in x and "violations" in x for x in rows)
    return m, rows


def test_a_vla_follow_with_the_estimator_records_it_and_keeps_the_shields_subject():
    """--start-when-seen 0: the track starts with no pick (the unanchored
    Reacquirer finds the moving car), loses it for 6 s (a lapse) and takes
    it back where it was predicted."""
    r, g = _estimator_follow()
    _check_estimator_follow(r, g, gate=False)


def test_with_the_start_gate_the_estimate_is_seeded_from_its_pick():
    r, g = _estimator_follow("--start-when-seen", "4")
    m, rows = _check_estimator_follow(r, g, gate=True)
    assert m["start_gate"]["reached"] and m["start_gate"]["identity"] is True
    first = m["subject_track"]["events"][0]
    assert first["event"] == "acquired" and first["seeded"] is True, first
    assert rows[0]["est_state"] == "committed"


# --------------------------------------------------------------------- runner

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
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
