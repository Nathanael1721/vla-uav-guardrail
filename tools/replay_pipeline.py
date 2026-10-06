"""Replay the recorded red-car flights through the OLD and the NEW pipeline.

tools/replay_identity.py answers "is each box judged right?". This answers
the question the aircraft flies on: how often is the ESTIMATE on the car,
and how often does a box that is not the car start (or re-start) it?

OPEN LOOP, AND ONLY THE BOXES THAT WERE LOGGED. The aircraft's path is the
one the old controller flew, and each inference contributes the ONE box the
old lenient lock chose (detections.jsonl logged every candidate only while
acquiring), or nothing on a miss. So this under-counts what the new pipeline
would have had - a true car the old lock passed over is not in the log - and
cannot show what the new controller would have done with a better estimate.
Both arms see exactly the same boxes, each at the tick that consumed it (the
flight_log row the box joins to, tools/replay_identity.join_detections).
Boxes the old presence gate blocked are fed to BOTH arms, so both judge the
same boxes (the new pipeline has no presence gate; identity replaces it). The
old arm can feed only those whose range the log kept. On six flights
(_carpolicy, _far, _ground, _high, _pedpolicy, _trail) it kept none (r_src
"unlogged"), so there "old" is gated exactly as flown; on the other six it is
fed 90-453 blocked boxes per flight that the flight never fed. The new arm
gets the unlogged ones as SOFT "no range". The old arm AS FLOWN (blocked boxes
not fed) is scored beside it; see AS-FLOWN CHECK.

  old  the estimator loop of demo/follow_vlm.py as flown at 1d09786, on the
       estimator as flown (tools/_target_state_1d09786.py - vendored because
       the live one has since changed how it predicts). Never reset: those
       flights had no retarget, the only reset. Each box is fed ONCE, at the
       consuming tick's time and pose, with the linear pixel bearing and the
       depth range, exactly as the flight fed the boxes it did feed (a 0.5 s
       median lag behind the capture on these flights - part of what was
       flown). observe() runs on every logged tick, which predicts IN PLACE
       while the estimate is younger than 3 s and not at all after: the first
       update after a gap then predicts the rest of it in one step, and its
       gate opens as dt^2. So the old estimate has no "seeds"; what it has
       are POST-GAP ACCEPTS - the first box it accepted after more than 3 s
       without an update (or the first ever) - which is where it
       re-attached, to whatever that box was on.
  new  demo/follow_vlm.py's identity path, with follow_vlm's own Reacquirer,
       TargetLock, measured_speed and Grounder._prior_at imported, not
       copied, so their current rules are the ones replayed. The FIRST
       acquisition is the runner's start gate (scripts/run_citylife_follow.ps1:
       --start-when-seen 5, --start-max-range-m 30; its 330 s timeout outlasts
       every log); each later one a re-acquisition (4 OK in a row, <= 45 m,
       within reach of the lapse anchor). A commit is follow_vlm's
       _take_subject: the estimator seeded from the gate's pick at ITS
       capture pose (bearing = the pick's world bearing - capture yaw, range
       = its horizontal range), the lock seeded with the capture yaw, and
       the last measured position, speed history and heading moved to the
       seed (the pre-loss speed kept only as the next anchor's fallback).
       Then: identity tiers (demo/identity_thresholds.json, on
       identity.features_for's features - bottom_clipped included - via
       replay_identity.build_samples), the strict lock's pixel and world
       gates against the prior projected from the estimator's posterior,
       OK boxes feed, SOFT ones only continue a live track (estimate
       < 1.5 s, last OK < 3 s), each at its capture time and pose. A LAPSE
       - 3 s on the flight's own clock of accepted measurements and commits,
       checked each tick before that tick's box - resets the estimator and
       the lock and starts a re-acquisition anchored on the estimator's
       posterior and the last measured speed.

Scored on a 10 Hz grid over each flight: a tick is SERVED when the estimate
is younger than 3 s, ON when it is within 6 m of the car's truth. A seed (new)
or post-gap accept (old) is FALSE when the box that did it placed the car
more than 10 m from where the car was. Counts are kept as integers per flight
and pooled as integers; fractions are for display only.

AS-FLOWN CHECK. The old arm is run once more with the presence gate
honoured (blocked boxes not fed, as flown), and its served flag compared,
row by row, with the `est.served` the flight itself logged. On the four
_final flights - logs closed 2026-09-29 19:33-20:08, from the working tree
that was committed as 1d09786 at 20:17 - it agrees on 9,586 of 9,593 rows.
The other eight flew earlier trees (manifest code_revision 37514c3-dirty on
2026-09-23; 82295b1-dirty on 09-24 and earlier on 09-29) and agree on
59-79 % of rows. Their loop fed the estimator differently: one known
difference is that it re-fed the held box on every inference, and a rough
replay fed that way lifts the five 2026-09-23 flights from 0.59-0.73 to
0.76-0.94. So on those the old arm is 1d09786's pipeline on their boxes, not
literally what they flew; the report pools all twelve, and the REPRODUCED
flights (>= REPRODUCED_MIN) apart. This as-flown arm is also SCORED
("old_as_flown"). On the reproduced flights it is what the flown estimate
did, and it differs from "old" there: the presence gate blocked boxes on the
car too, so the flown estimate was on the car less often than "old" says -
pooled over the four, on the car on 0.107 of served ticks and 0.040 of all,
against "old"'s 0.190 and 0.086 (2026-09-30).

    python tools/replay_pipeline.py     # demo/out/identity_replay/pipeline.md/.json
    python tools/replay_pipeline.py --start-need 4 --start-max-range-m 0

Needs the vla-real env (the new arm imports demo/follow_vlm.py).
Tests: tests/test_replay_pipeline.py.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))
sys.path.insert(0, str(ROOT / "tools"))

import identity                                    # noqa: E402
import replay_identity as ri                       # noqa: E402
from target_state import TargetState               # noqa: E402
from _target_state_1d09786 import TargetState as FlownTargetState   # noqa: E402

OUT = ri.REPORT_DIR
LAPSE_S = 3.0           # follow_vlm --lapse-s default, and observe()'s max_coast_s
ON_M = 6.0
FALSE_SEED_M = 10.0
TICK = 0.1
CAP_LAG_S = 0.2         # replay_identity's estimate of the image age at fetch
HFOV_DEG = ri.HFOV_DEG
# As flown at 1d09786: SUBJECT_VMAX_MPS["car"] and SUBJECT_WIDTH_M["car"].
FLOWN_CAR_VMAX = 15.0
FLOWN_CAR_WIDTH_M = 4.0
SOFT_FEED_AGE_S = 1.5   # follow_vlm: SOFT may continue an estimate this young...
SOFT_FEED_OK_S = 3.0    # ...whose last OK box is at most this old
SUBJECT_CENTRE_UP_M = 0.75  # follow_vlm's set_prior "up" for a car

# The integer counters of each arm; every rate is derived from these.
TICK_KEYS = ("n_ticks", "n_served", "n_on")
# A flight whose logged est.served the old arm reproduces on at least this
# share of rows was flown by the loop it replays (the AS-FLOWN CHECK), and is
# pooled once more on its own as a REPRODUCED flight.
REPRODUCED_MIN = 0.99


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _dist(P, truth):
    if P is None or truth is None:
        return None
    return math.hypot(P[0] - truth[0], P[1] - truth[1])


def rates(c: dict) -> dict:
    """`c` plus the display fractions, computed from its integers."""
    c = dict(c)
    n, s, o = c["n_ticks"], c["n_served"], c["n_on"]
    c["served_frac"] = s / n if n else None
    c["on_frac_of_served"] = o / s if s else None
    c["on_frac_of_all"] = o / n if n else None
    return c


def reproduced(r: dict) -> bool:
    """Did the old arm reproduce this flight's own served flag (see REPRODUCED_MIN)?"""
    ch = r.get("flown_check") or {}
    return bool(ch.get("rows")) and ch["agree"] >= REPRODUCED_MIN * ch["rows"]


def pool(per_flight: dict) -> dict:
    """Pooled counts per arm: the integers summed, the fractions recomputed.

    The pooled fractions were once rebuilt from per-flight fractions ROUNDED
    to 3 places and multiplied back by the tick count; summing the integers
    is exact and cannot drift with the display precision."""
    out = {}
    for arm in ("old", "new", "old_as_flown", "flown_check"):
        tot = {}
        for r in per_flight.values():
            for k, v in (r.get(arm) or {}).items():
                if isinstance(v, int) and not isinstance(v, bool):
                    tot[k] = tot.get(k, 0) + v
        if not tot:
            continue
        out[arm] = rates(tot) if all(k in tot for k in TICK_KEYS) else tot
    return out


# ---------------------------------------------------------------------------
# the old arm: follow_vlm's estimator loop at 1d09786
# ---------------------------------------------------------------------------
def flown_bearing(cx: float, img_w: float, hfov_deg: float = HFOV_DEG) -> float:
    """The bearing the estimator was fed at 1d09786: LINEAR in the column."""
    return math.radians(hfov_deg / 2.0) * ((cx - img_w / 2.0) / (img_w / 2.0))


def flown_width_range(w: float, img_w: float, object_width_m: float = FLOWN_CAR_WIDTH_M,
                      hfov_deg: float = HFOV_DEG):
    """implied_range_from_width at 1d09786: the fallback when depth had none."""
    if w <= 0 or img_w <= 0:
        return None
    half = math.radians(hfov_deg / 2.0) * (w / img_w)
    if half <= 1e-4:
        return None
    return (object_width_m / 2.0) / math.tan(half)


class FlownArm:
    """The estimator as the old flights ran it. See the module docstring."""

    def __init__(self, honour_gate: bool = False):
        self.est = FlownTargetState(v_max=FLOWN_CAR_VMAX)
        self.honour_gate = honour_gate
        self.post_gap = []          # (t, measured point, distance to the car)

    def row(self, row: dict, boxes, truth_at) -> bool:
        """One logged tick: feed the box(es) it consumed, then observe().
        Returns whether the estimate was served on that tick."""
        t = float(row["t"])
        for bx in boxes:
            self._feed(t, row, bx, truth_at)
        return self.est.observe(t, row["x"], row["y"], row["psi"]) is not None

    def _feed(self, t, row, bx, truth_at):
        if self.honour_gate and bx["blocked"]:
            return
        r, det, W = bx["r"], bx["box"], bx["W"]
        if r is None:
            if bx["r_src"] is not None:
                return              # blocked in flight and its range unlogged
            r = flown_width_range(float(det["w"]), W)
        if not r:
            return
        b = flown_bearing(float(det["cx"]), W)
        was = self.est.t_last_update
        if self.est.update(t, row["x"], row["y"], row["psi"], b, float(r)):
            if was is None or t - was > LAPSE_S:
                th = row["psi"] + b
                z = (row["x"] + r * math.cos(th), row["y"] + r * math.sin(th))
                self.post_gap.append((t, z, _dist(z, truth_at(t))))

    def served_at(self, t: float):
        """The served position at `t`, WITHOUT moving the in-place state (the
        flight predicted on its own ticks, which `row` replays)."""
        e = self.est
        if e.x is None or e.t_last_update is None or t - e.t_last_update > LAPSE_S:
            return None
        dt = t - getattr(e, "_t_state", e.t_last_update)
        return float(e.x[0] + e.x[2] * dt), float(e.x[1] + e.x[3] * dt)


# ---------------------------------------------------------------------------
# the new arm: follow_vlm's identity path
# ---------------------------------------------------------------------------
class NewArm:
    """demo/follow_vlm.py's identity path over one flight's logged boxes.

    Two clocks, as in the flight. The ESTIMATE is stamped with capture times
    and stops being served `max_coast_s` after the last one; the LAPSE runs
    on the flight's own clock of accepted measurements and commits
    (`t_meas_last`, the tick that consumed them), so a box captured just
    before the estimate went stale can still be fed after it."""

    def __init__(self, th: dict, cfg: dict, fv):
        self.fv = fv
        self.th = th
        self.cfg = cfg
        self.est = TargetState(v_max=fv.SUBJECT_VMAX_MPS.get("car"))
        self.lock = fv.TargetLock()
        self.reacq = self._gate("start")
        self.reacq.start(None)
        self.kind = "start"
        self.committed = False      # follow_vlm's subject_committed
        self.t_meas_last = None     # ...t_meas_last
        self.t_ok_last = 0.0        # ...t_ok_last (the flight's 0.0 epoch: long ago)
        self.acc_hist = []          # (t, x, y) of accepted estimates, for the speed
        self.last_acc = None
        self.last_meas_speed = None
        self.v_prior = None         # the speed measured before the last commit
        self.last_heading = None
        self.seeds = []             # (kind, t, P, distance to the car)
        self.n_lapses = 0

    def _gate(self, kind: str):
        c = self.cfg
        if kind == "start":
            # follow_vlm: the start gate's range, or the re-acquisition one at 0.
            rmax = (c["start_max_range_m"] if c["start_max_range_m"] > 0
                    else c["reacq_max_range_m"])
            return self.fv.Reacquirer(need=c["start_need"], max_range_m=rmax)
        return self.fv.Reacquirer(need=c["reacq_need"], max_range_m=c["reacq_max_range_m"])

    def tick(self, now: float) -> None:
        """The lapse check, run each tick before that tick's box (as flown)."""
        if (self.reacq is not None or not self.committed or self.t_meas_last is None
                or now - self.t_meas_last <= self.cfg["lapse_s"]):
            return
        xu = self.est._xu
        P_a = (float(xu[0]), float(xu[1])) if xu is not None else self.last_acc
        anchor = (None if P_a is None else
                  {"t": (self.est.t_last_update if xu is not None
                         and self.est.t_last_update is not None else self.t_meas_last),
                   "P": P_a,
                   "v": (self.last_meas_speed if self.last_meas_speed is not None
                         else self.v_prior)})
        self.est.reset()
        self.committed = False
        self.lock.reset()           # grounder.begin_reacquire()
        self.reacq = self._gate("reacq")
        self.reacq.start(anchor)
        self.kind = "reacq"
        self.n_lapses += 1

    def miss(self, t_cap: float) -> None:
        """An inference that found nothing: it breaks a gate's streak."""
        if self.reacq is not None:
            self.reacq.step([], [], [], t_cap)

    def _prior(self, pose: dict, W: int, H: int, t_cap: float):
        """The strict lock's prior exactly as the flight projects it: the
        grounder's own _prior_at on the snapshot follow_vlm hands set_prior()."""
        xu, tu = self.est._xu, self.est.t_last_update
        if xu is None:
            return None
        snap = {"t": tu, "x": float(xu[0]), "y": float(xu[1]),
                "vx": float(xu[2]), "vy": float(xu[3]), "t_upd": tu,
                "up": SUBJECT_CENTRE_UP_M}
        stub = SimpleNamespace(_prior=snap, identity={"hfov": HFOV_DEG})
        return self.fv.Grounder._prior_at(stub, pose, W, H, t_cap)

    def box(self, bx: dict, pose: dict, truth_at) -> None:
        f, t_cap, t_use = bx["f"], bx["t_cap"], bx["t_use"]
        b, W, H = bx["box"], bx["W"], bx["H"]
        cand = (b["cx"], b["cy"], b["w"], b["h"], b.get("score") or 0.0, W, H,
                b.get("colour") or 0.0)
        tier, _why = identity.classify(f, ri.NOUN, self.th)
        if self.reacq is not None:
            if self.reacq.step([cand], [tier], [f], t_cap):
                heading = self.reacq.heading() if self.kind == "reacq" else None
                self._take_subject(cand, pose, t_cap, t_use, truth_at)
                if heading is not None:
                    self.last_heading = heading
            return
        k, _ = self.lock.select_strict([cand], [tier], [f], W, pose["yaw"], t_cap,
                                       prior=self._prior(pose, W, H, t_cap))
        # A box with no range never gets here while tracking: it is SOFT "no
        # range", and select_strict refuses a SOFT box without a map point
        # whenever there is a prior - which there always is between a seed
        # and a lapse. The guard only keeps a future rule change honest.
        if k is None or f.get("rng_h") is None:
            return
        if tier != "ok":
            age = (t_cap - self.est.t_last_update
                   if self.est.t_last_update is not None else 1e9)
            if not (age < SOFT_FEED_AGE_S and t_use - self.t_ok_last < SOFT_FEED_OK_S):
                return
        bm = _wrap(f["bearing"] - pose["yaw"])
        if self.est.update(t_cap, pose["x"], pose["y"], pose["yaw"], bm, f["rng_h"]):
            if tier == "ok":
                self.t_ok_last = t_use
            self.t_meas_last = t_use
            self._accepted(t_use)

    def _take_subject(self, cand, pose, t_cap, t_use, truth_at):
        """follow_vlm._take_subject: seed the estimator from the gate's pick at
        its capture pose, seed the lock, and move the flight's memory of the
        subject - last measured position, its history, speed, heading - to
        the seed. Left on the pre-loss track, a second loss soon after
        anchored the reach on the old place (review, 2026-09-29)."""
        pf = self.reacq.pick_feat
        self.est.reset()
        seeded = False
        if pf.get("rng_h") is not None and pf.get("bearing") is not None:
            b0 = _wrap(pf["bearing"] - pose["yaw"])
            seeded = bool(self.est.update(t_cap, pose["x"], pose["y"], pose["yaw"],
                                          b0, float(pf["rng_h"])))
        if seeded:
            P0 = (float(self.est._xu[0]), float(self.est._xu[1]))
        elif pf.get("P") is not None:
            P0 = (float(pf["P"][0]), float(pf["P"][1]))
        else:
            P0 = None
        self.lock.seed(cand, pose["yaw"], t_cap)     # grounder.commit(pick, yaw=cap)
        if self.last_meas_speed is not None:
            self.v_prior = self.last_meas_speed
        self.last_meas_speed = None         # a streak's ~3 m jitter is no speed
        self.last_heading = None
        self.acc_hist.clear()
        if P0 is not None:
            self.last_acc = P0
            self.acc_hist.append((t_use, P0[0], P0[1]))
        self.t_ok_last = t_use
        self.t_meas_last = t_use
        self.committed = True
        P = (pf["P"][0], pf["P"][1])
        self.seeds.append((self.kind, t_cap, P, _dist(P, truth_at(t_cap))))
        self.reacq = None

    def _accepted(self, t_use: float) -> None:
        x = self.est.x
        self.acc_hist.append((t_use, float(x[0]), float(x[1])))
        del self.acc_hist[:-40]
        self.last_acc = (float(x[0]), float(x[1]))
        v = self.fv.measured_speed(self.acc_hist)
        if v is not None:
            self.last_meas_speed = v
        if len(self.acc_hist) >= 2 and self.acc_hist[-1][0] - self.acc_hist[0][0] >= 1.0:
            h0 = [h for h in self.acc_hist if self.acc_hist[-1][0] - h[0] <= 2.0][0]
            dx, dy = self.acc_hist[-1][1] - h0[1], self.acc_hist[-1][2] - h0[2]
            if math.hypot(dx, dy) > 1.0:
                self.last_heading = math.atan2(dy, dx)

    def served(self, t: float):
        """The estimate the controller would get at `t` (observe()'s rule)."""
        e = self.est
        if e._xu is None or e.t_last_update is None or t - e.t_last_update > e.max_coast_s:
            return None
        e.predict(t)
        return float(e.x[0]), float(e.x[1])


# ---------------------------------------------------------------------------
# one flight
# ---------------------------------------------------------------------------
def flight_events(flight_dir, street, cap_lag: float = CAP_LAG_S) -> dict:
    """The logged boxes and misses of one flight, on the flight clock: each
    box with the row that consumed it (t_use) and its capture time (t_cap)."""
    rows, dets = ri.load_flight(flight_dir)
    built = ri.build_samples(flight_dir, street, cap_lag, rows=rows, dets=dets)
    rows = sorted((r for r in rows if r.get("t") is not None), key=lambda r: r["t"])
    first_row = {}
    for r in rows:
        q = r.get("det_seq")
        if q is not None and q not in first_row:
            first_row[q] = r
    boxes = []
    for s, (rec, row) in zip(built["samples"], built["pairs"]):
        b = rec["det"]
        boxes.append({"seq": rec["seq"], "row": row, "t_use": float(row["t"]),
                      "t_cap": s["t_cap"], "box": b, "W": b.get("img_w", 768),
                      "H": b.get("img_h", 432), "f": s["f"], "r": s["r"],
                      "r_src": s["r_src"], "blocked": s["blocked"]})
    misses = []
    if built["offset"] is not None:
        for rec in dets:
            if rec.get("det") is None and rec.get("seq") in first_row:
                misses.append({"seq": rec["seq"],
                               "t_use": float(first_row[rec["seq"]]["t"]),
                               "t_cap": ri.capture_time(rec, built["offset"], cap_lag)})
    return {"rows": rows, "boxes": boxes, "misses": misses,
            "timeline": built["timeline"]}


def replay_flight(flight_dir, th, street, cfg, fv, cap_lag: float = CAP_LAG_S) -> dict:
    ev = flight_events(flight_dir, street, cap_lag)
    tl, rows = ev["timeline"], ev["rows"]
    by_row = {}
    for bx in ev["boxes"]:
        by_row.setdefault(id(bx["row"]), []).append(bx)
    stream = sorted([("box", bx) for bx in ev["boxes"]]
                    + [("miss", m) for m in ev["misses"]],
                    key=lambda e: (e[1]["t_use"], e[1]["t_cap"]))
    old, flown, new = FlownArm(), FlownArm(honour_gate=True), NewArm(th, cfg, fv)
    cnt = {a: {k: 0 for k in TICK_KEYS} for a in ("old", "new", "old_as_flown")}
    chk = {"rows": 0, "agree": 0, "arm_served": 0, "log_served": 0}
    i_row = i_ev = 0
    t0, t_end = float(tl.t[0]), float(tl.t[-1])
    k = 0
    while True:
        t = t0 + k * TICK
        if t > t_end + 1e-9:
            break
        k += 1
        while i_row < len(rows) and rows[i_row]["t"] <= t:
            r = rows[i_row]
            here = by_row.get(id(r), [])
            old.row(r, here, tl.truth)
            s = flown.row(r, here, tl.truth)
            if isinstance(r.get("est"), dict):
                logged = bool(r["est"].get("served"))
                chk["rows"] += 1
                chk["agree"] += int(s == logged)
                chk["arm_served"] += int(s)
                chk["log_served"] += int(logged)
            i_row += 1
        while i_ev < len(stream) and stream[i_ev][1]["t_use"] <= t:
            kind, e = stream[i_ev]
            new.tick(e["t_use"])
            if kind == "miss":
                new.miss(e["t_cap"])
            else:
                new.box(e, tl.pose(e["t_cap"]), tl.truth)
            i_ev += 1
        new.tick(t)
        tr = tl.truth(t)
        if tr is None:
            continue
        for name, pos in (("old", old.served_at(t)), ("new", new.served(t)),
                          ("old_as_flown", flown.served_at(t))):
            c = cnt[name]
            c["n_ticks"] += 1
            if pos is not None:
                c["n_served"] += 1
                c["n_on"] += int(_dist(pos, tr) <= ON_M)

    def flown_counts(name, arm):
        false_pg = [d for _t, _z, d in arm.post_gap if d is not None and d > FALSE_SEED_M]
        return rates({**cnt[name], "post_gap_accepts": len(arm.post_gap),
                      "false_post_gap_accepts": len(false_pg),
                      "updates": arm.est.n_updates, "gated_out": arm.est.n_rejected,
                      "resets": arm.est.n_resets,
                      "false_post_gap_m": [round(d, 1) for d in false_pg][:10]})
    false_seeds = [d for _k, _t, _P, d in new.seeds if d is not None and d > FALSE_SEED_M]
    return {
        "old": flown_counts("old", old),
        "old_as_flown": flown_counts("old_as_flown", flown),
        "new": rates({**cnt["new"], "seeds": len(new.seeds),
                      "seeds_start": sum(s[0] == "start" for s in new.seeds),
                      "seeds_reacq": sum(s[0] == "reacq" for s in new.seeds),
                      "false_seeds": len(false_seeds), "lapses": new.n_lapses,
                      "start_t": (None if not new.seeds or new.seeds[0][0] != "start"
                                  else round(new.seeds[0][1], 2)),
                      "false_seed_m": [round(d, 1) for d in false_seeds][:10]}),
        "flown_check": chk,
        "data": {"boxes": len(ev["boxes"]), "misses": len(ev["misses"]),
                 "range_unlogged": sum(b["r_src"] == "unlogged" for b in ev["boxes"]),
                 "presence_blocked": sum(bool(b["blocked"]) for b in ev["boxes"]),
                 "code_revision": code_revision(flight_dir)},
    }


def code_revision(flight_dir):
    """The tree the flight was flown from (manifest.json), or None."""
    try:
        with (Path(flight_dir) / "manifest.json").open(encoding="utf-8") as fh:
            return json.load(fh).get("code_revision")
    except (OSError, ValueError):
        return None


# ---------------------------------------------------------------------------
# report
# ---------------------------------------------------------------------------
def _f(x):
    return "-" if x is None else f"{x:.3f}"


def markdown(res: dict) -> str:
    c = res["_config"]
    L = ["# Pipeline replay: is the ESTIMATE on the red car?", "",
         "Open loop over the recorded flights, the logged boxes only (see "
         "tools/replay_pipeline.py). served = estimate younger than "
         f"{c['lapse_s']:g} s; on = within {c['on_m']:g} m of truth. Thresholds "
         f"`{res['_thresholds_hash']}`.", "",
         f"- **old**: the estimator as flown, {c['old_estimator']}, never reset, "
         "each box fed at its consuming tick's time and pose. **Post-gap accepts** "
         f"= the first box it accepted after > {c['lapse_s']:g} s without an "
         "update (or the first ever): where the flown estimate re-attached. It "
         "has no seeds.",
         "- Both arms see every logged box, presence-blocked ones included - the "
         "old arm only where the log kept the blocked box's range (none on "
         "_carpolicy, _far, _ground, _high, _pedpolicy, _trail: there it is "
         "gated as flown). The old arm AS FLOWN, gate honoured, is scored in "
         "the second table.",
         f"- **new**: start gate {c['start_need']} OK sightings in a row within "
         f"{c['start_max_range_m']:g} m (0 = the re-acquisition range); "
         f"re-acquisition {c['reacq_need']} within {c['reacq_max_range_m']:g} m; "
         f"lapse after {c['lapse_s']:g} s. **Seeds** = start + re-acquisitions.",
         f"- **false** = the box that did it put the car more than "
         f"{c['false_seed_m']:g} m from where it was.", "",
         "| flight | old served | old on/served | old on/all | old false / post-gap "
         "accepts | new served | new on/served | new on/all | new seeds (start+reacq) "
         "| new false seeds | new lapses |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]

    def line(name, o, n):
        return (f"| {name} | {_f(o['served_frac'])} | {_f(o['on_frac_of_served'])} | "
                f"{_f(o['on_frac_of_all'])} | {o['false_post_gap_accepts']}/"
                f"{o['post_gap_accepts']} | {_f(n['served_frac'])} | "
                f"{_f(n['on_frac_of_served'])} | {_f(n['on_frac_of_all'])} | "
                f"{n['seeds']} ({n['seeds_start']}+{n['seeds_reacq']}) | "
                f"{n['false_seeds']} | {n['lapses']} |")
    flights = {k: r for k, r in res.items() if not k.startswith("_")}
    for k, r in flights.items():
        L.append(line(k, r["old"], r["new"]))
    p, pf = res["_pooled"], res["_pooled_reproduced"]
    L.append(line("**pooled**", p["old"], p["new"]))
    if pf:
        n_rep = len(res["_reproduced_flights"])
        L.append(line(f"**pooled, reproduced flights** ({n_rep})", pf["old"], pf["new"]))
    L += [""]
    for name, q in (("pooled", p), ("pooled, reproduced flights", pf)):
        if q:
            L.append(f"- {name}: ticks {q['old']['n_ticks']}; old served "
                     f"{q['old']['n_served']}, on {q['old']['n_on']}; old as flown "
                     f"served {q['old_as_flown']['n_served']}, on "
                     f"{q['old_as_flown']['n_on']}; new served "
                     f"{q['new']['n_served']}, on {q['new']['n_on']}.")
    L += ["", "The fractions above are these integers, rounded for display.", "",
          "## Is the old arm the flown one?", "",
          "The old arm once more with the presence gate honoured (blocked boxes "
          "not fed, as flown), its served flag against the `est.served` the "
          "flight logged, row by row. A flight agreeing on at least "
          f"{REPRODUCED_MIN:g} of its rows was flown by the loop the old arm "
          "replays, and is pooled again as a reproduced flight; the others flew "
          "earlier trees whose loop fed the estimator differently. The last "
          "four columns score that as-flown arm like the table above: on a "
          "reproduced flight they, not the old columns above, are what the "
          "flown estimate did.", "",
          "| flight | code revision | rows | agree | arm served | flight served "
          "| as flown: served | on/served | on/all | false / post-gap accepts |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    items = list(flights.items()) + [("**pooled**", dict(p, data={}))]
    if pf:
        items.append((f"**pooled, reproduced flights** ({len(res['_reproduced_flights'])})",
                      dict(pf, data={})))
    for k, r in items:
        ch, a = r["flown_check"], r["old_as_flown"]
        frac = ch["agree"] / ch["rows"] if ch["rows"] else None
        L.append(f"| {k} | {r['data'].get('code_revision') or '-'} | {ch['rows']} | "
                 f"{ch['agree']} ({_f(frac)}) | {ch['arm_served']} | "
                 f"{ch['log_served']} | {_f(a['served_frac'])} | "
                 f"{_f(a['on_frac_of_served'])} | {_f(a['on_frac_of_all'])} | "
                 f"{a['false_post_gap_accepts']}/{a['post_gap_accepts']} |")
    return "\n".join(L) + "\n"


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--start-need", type=int, default=5,
                    help="OK sightings in a row for the FIRST acquisition "
                         "(scripts/run_citylife_follow.ps1 -StartWhenSeen)")
    ap.add_argument("--start-max-range-m", type=float, default=30.0,
                    help="the start gate's range (-StartMaxRange); 0 = "
                         "--reacq-max-range-m, as follow_vlm")
    ap.add_argument("--reacq-need", type=int, default=4,
                    help="follow_vlm --reacq-need")
    ap.add_argument("--reacq-max-range-m", type=float, default=45.0,
                    help="follow_vlm --reacq-max-range-m")
    ap.add_argument("--out", default=str(OUT))
    return ap.parse_args(argv)


def config(args) -> dict:
    return {"start_need": args.start_need, "start_max_range_m": args.start_max_range_m,
            "reacq_need": args.reacq_need, "reacq_max_range_m": args.reacq_max_range_m,
            "lapse_s": LAPSE_S, "on_m": ON_M, "false_seed_m": FALSE_SEED_M,
            "tick_s": TICK, "cap_lag_s": CAP_LAG_S,
            "old_estimator": "tools/_target_state_1d09786.py (1d09786)"}


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        import follow_vlm as fv
    except ImportError as e:
        print(f"[replay] the new arm imports demo/follow_vlm.py, which needs the "
              f"vla-real env: {e}")
        return 1
    cfg = config(args)
    th = identity.load_thresholds()
    sd = identity.StreetDistance.load()
    # Both pools from the flights alone, before any summary key is added: the
    # '_pooled' row itself could otherwise be counted as a reproduced flight
    # and double every integer (re-review, 2026-09-30).
    flights = {d.name: replay_flight(d, th, sd, cfg, fv) for d in ri.flight_dirs()}
    repro = {k: v for k, v in flights.items() if reproduced(v)}
    res = dict(flights)
    res["_pooled"] = pool(flights)
    res["_pooled_reproduced"] = pool(repro) if repro else None
    res["_reproduced_flights"] = sorted(repro)
    res["_config"] = cfg
    res["_thresholds_hash"] = identity.thresholds_hash(th)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with (out / "pipeline.json").open("w", encoding="utf-8", newline="\n") as fh:
        json.dump(res, fh, indent=1, allow_nan=False)
        fh.write("\n")
    md = markdown(res)
    with (out / "pipeline.md").open("w", encoding="utf-8", newline="\n") as fh:
        fh.write(md)
    print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
