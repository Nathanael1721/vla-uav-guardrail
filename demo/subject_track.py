"""The followed subject in the world between detections, for a rail whose pilot
is not demo/follow_vlm.py's controller.

WHY (2026-11-15)

The lab's AIOT-VLA follows the red car at 0.955-0.969 within 30 m on the
CityLife simple_flight rail and at a target lock of 0.04-0.82 on the Project
AirSim + ArduPilot rail (docs/data/pas_aiot_hil_20261114). The perception
node of the second (demo/pas_ardupilot/perception_node.py) carried none of
follow_vlm.fly()'s subject handling: no target estimate between detections,
no strict lock, no re-acquisition. With --identity it even held no lock at
all ("strict lock not used (identity without an estimator)"), because a
strict lock with nothing to lean on never takes a subject back after its 2 s
hold. In pas_hil_aiot_kpi_01 the detector's box was fresh on 942 of 4,192
ticks, and on those ticks the car was a median 97 m away: the box was on
something else.

WHAT THIS IS

follow_vlm.fly()'s identity path - the target estimator, the prior the strict
lock leans on, the lapse and the anchored re-acquisition, the trail - as one
object a control loop steps once per tick with the detector's latest answer
(follow_vlm.Grounder.latest()). Every decision is follow_vlm's own code,
imported, not copied:

  * the estimator      follow_vlm.subject_estimator (demo/target_state.
                       TargetState with the class's speed ceiling), fed once
                       per NEW held box by follow_vlm.estimator_measurement
                       (identity's range and bearing at the box's own capture
                       pose and time); a SOFT box only to continue a live track
                       (follow_vlm.soft_may_continue); no range in the box's
                       own depth frame: the subject width's
                       (follow_vlm.implied_range_from_width);
  * the commit         follow_vlm.seed_estimate from the start gate's or the
                       re-acquisition's pick at its capture, the Grounder
                       committed to the pick (its strict lock seeded), the
                       memory of the subject moved to it;
  * the strict lock    the Grounder's (built with a TargetLock and identity),
                       leaning on follow_vlm.estimator_prior every tick there
                       is an estimate, so a car hidden for more than the
                       lock's 2 s hold is taken back where it is PREDICTED;
  * the lapse          no accepted measurement for lapse_s: the estimate is
                       dropped and follow_vlm.Reacquirer starts, anchored by
                       follow_vlm.lapse_anchor (the estimator's posterior, the
                       last measured speed); its pick is committed as above;
  * the trail          follow_vlm.subject_trail, the estimator's own positions
                       after each accepted update (demo/trail.py);
  * the speed          follow_vlm.measured_speed over the accepted positions.

The defaults (lapse_s, reacq_need, reacq_max_range_m) are follow_vlm's own
parser's (follow_defaults), the flights tools/replay_pipeline.py measured at
94.5 % of served ticks on the car.

THE ORDER of one tick is fly()'s: a new held box is a detection once; while
re-acquiring every new inference's candidates go to the Reacquirer and no box
is a measurement; else a lapse may start one; then the strict lock's prior;
then the measurement.

WHAT A PILOT IS GIVEN (memory_bearing). Not a command: a direction. While
committed and the estimator serves one (TargetState.observe: at most its
max_coast_s, 3 s, after the last update) the bearing to the PREDICTED
subject ("estimate"); else, while the subject is still held in memory -
committed, or re-acquiring with an anchor whose reach is within the
Reacquirer's reach_cap_m (past it the anchor no longer says where the subject
is, the Reacquirer's own rule) - fly()'s coast look: a little past the last
sighting along the subject's last direction (trail.lookout, "trail"); else
None, and the caller keeps its own rule. The Project AirSim node turns it
into the VLA's "memory" hint (vla_mission.DetectorHint's `estimate`).

WHAT THIS IS NOT. Nothing here flies, and nothing here reaches the Shield:
the Shield's subject stays whatever the caller gives it. No ground truth
enters: only the detector's boxes, identity's features of them and the
aircraft's own pose. Stdlib, numpy and demo/'s modules; follow_vlm (passed
in as `fv`, else imported on first use) needs no simulator to import.

Tested in tests/test_subject_track.py.
"""
from __future__ import annotations

import importlib
import math
import sys
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import trail as trail_mod                                            # noqa: E402

SOURCE = ("follow_vlm.fly()'s identity path (its target estimator, the strict lock's "
          "prior, the lapse and the anchored Reacquirer, the trail), stepped per tick "
          "by demo/subject_track.SubjectTrack on follow_vlm's own functions")
STATES = ("acquiring", "committed", "reacquiring")
MEMORY_SOURCES = ("estimate", "trail")
# follow_vlm's parser options this object takes its defaults from.
DEFAULT_KEYS = ("lapse_s", "reacq_need", "reacq_max_range_m")
EVENTS_MAX = 200


def follow_defaults(fv) -> dict:
    """lapse_s, reacq_need and reacq_max_range_m as follow_vlm's own parser
    defaults them - read there, never copied here."""
    ap = fv.build_arg_parser()
    return {k: ap.get_default(k) for k in DEFAULT_KEYS}


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _spread(vals) -> dict:
    v = sorted(float(x) for x in vals if x is not None)
    if not v:
        return {"n": 0, "p50": None, "p95": None, "max": None}

    def q(p):
        return round(v[min(len(v) - 1, int(math.ceil(p * len(v))) - 1)], 3)
    return {"n": len(v), "p50": q(0.5), "p95": q(0.95), "max": round(v[-1], 3)}


class SubjectTrack:
    """The followed subject in the world (module docstring), stepped once per
    control tick.

    `grounder` is follow_vlm.Grounder (or anything with its latest() shape and
    commit / begin_reacquire / set_prior), built with identity and a
    TargetLock, so its lock is strict. `subject_class` picks the estimator's
    speed ceiling and the prior's centre height; `object_width_m` ranges a box
    with no depth range of its own (None: such a box is no measurement).

      start(pick_feat, cap, now)  after the start gate: committed to its pick
                                  (identity's features of it and its
                                  inference's capture), or - no pick - looking
                                  for the subject with the Reacquirer and no
                                  anchor, as fly() does when its gate did not
                                  confirm one.
      step(g, now, pose)          one tick: `g` this tick's latest(), `pose`
                                  the aircraft's (x, y, yaw rad) now.
      memory_bearing(t, x, y, yaw)  (world bearing rad, "estimate" | "trail")
                                  or None (module docstring).
      hint_memory(capture)        memory_bearing at a vla_link.Capture's time
                                  and pose: vla_mission.DetectorHint's
                                  `estimate`.
      row(now, x, y, yaw)         the flight-log fields of the tick (est_*),
                                  once per logged tick.
      summary()                   metrics.json's record.

    Times: `now` is the loop's clock and the Grounder's capture times the
    detector's; both are the wall clock on a flight (time.time)."""

    def __init__(self, grounder, subject_class=None, *, object_width_m: float | None = None,
                 lapse_s: float | None = None, reacq_need: int | None = None,
                 reacq_max_range_m: float | None = None, trail: bool = True, fv=None):
        self.fv = fv or importlib.import_module("follow_vlm")
        d = follow_defaults(self.fv)
        self.lapse_s = float(d["lapse_s"] if lapse_s is None else lapse_s)
        self.reacq_need = int(d["reacq_need"] if reacq_need is None else reacq_need)
        self.reacq_max_range_m = float(d["reacq_max_range_m"] if reacq_max_range_m is None
                                       else reacq_max_range_m)
        if not (self.lapse_s > 0 and self.reacq_need >= 1 and self.reacq_max_range_m > 0):
            raise ValueError("lapse_s and reacq_max_range_m must be positive, reacq_need >= 1")
        self.grounder = grounder
        self.subject_class = subject_class
        self.object_width_m = None if object_width_m is None else float(object_width_m)
        self.centre_up_m = float(self.fv.SUBJECT_CENTRE_UP_M.get(subject_class, 0.75))
        self.estimator = self.fv.subject_estimator(subject_class)
        self.trail = self.fv.subject_trail() if trail else None
        self.state = "acquiring"
        self.started = False
        self.reacq = None
        self.t0: float | None = None
        # The subject's memory since the last commit (fly()'s): when it was
        # last measured (the loop's clock) and that measurement's capture,
        # when last OK, the last accepted place and its history, the speeds.
        self.t_meas_last: float | None = None
        self.t_meas_last_cap: float | None = None
        self.t_ok_last = -math.inf
        self.last_acc: tuple | None = None
        self.acc_hist: list = []
        self.last_meas_speed: float | None = None
        self.v_prior: float | None = None
        self._last_est_seq = None
        self._last_reacq_seq = None
        self.last_lock_why = None
        # What happened, for metrics.json.
        self.events: list[dict] = []
        self.n_events = Counter()
        self.counts = Counter()
        self.meas_src = Counter()
        self.reacq_refused = Counter()
        self.reacq_outcomes = Counter()
        self.ticks_by_state = Counter()
        self.memory_ticks = Counter()
        self.n_rows = 0
        self.n_served = 0
        self._ages: list[float] = []

    # ------------------------------------------------------------- set-up --
    def config(self) -> dict:
        return {"lapse_s": self.lapse_s, "reacq_need": self.reacq_need,
                "reacq_max_range_m": self.reacq_max_range_m,
                "subject_class": self.subject_class, "object_width_m": self.object_width_m,
                "centre_up_m": self.centre_up_m, "v_max_mps": self.estimator.v_max,
                "max_coast_s": self.estimator.max_coast_s, "trail": self.trail is not None,
                "defaults_from": "follow_vlm.build_arg_parser()", "source": SOURCE}

    def _mt(self, now: float) -> float:
        return round(now - (self.t0 if self.t0 is not None else now), 3)

    def _event(self, kind: str, now: float, **data) -> None:
        self.n_events[kind] += 1
        if len(self.events) < EVENTS_MAX:
            self.events.append({"t": self._mt(now), "event": kind, **data})

    def start(self, pick_feat: dict | None = None, cap: dict | None = None,
              now: float | None = None, t0: float | None = None) -> None:
        """After the start gate (class docstring). `now` the loop's clock;
        `t0` the mission's start on it (events are in mission seconds)."""
        now = time.time() if now is None else float(now)
        self.t0 = now if t0 is None else float(t0)
        self.started = True
        if pick_feat is not None:
            seeded, P0 = self._take(pick_feat, cap, now)
            self._event("acquired", now, seeded=bool(seeded),
                        P=None if P0 is None else [round(v, 2) for v in P0])
        else:
            self._begin_reacquire(None, now)
            self._event("acquire", now, anchor=None)

    def _take(self, pf: dict, ic: dict | None, now: float) -> tuple:
        """fly()'s _take_subject: the estimator seeded from the committing
        sighting at its capture pose (follow_vlm.seed_estimate), and the
        memory of the subject moved to it - the last measured place, its
        history, the speed (kept as the next anchor's fallback) and the
        trail. Left on the pre-loss track, a second loss soon after anchored
        the reach on the old place (follow_vlm, review 2026-09-29)."""
        cp = (ic or {}).get("pose")
        t_c = (ic or {}).get("t_capture") or (ic or {}).get("t_cap") or now
        seeded, P0 = self.fv.seed_estimate(self.estimator, pf, cp, t_c)
        if self.last_meas_speed is not None:
            self.v_prior = self.last_meas_speed
        self.last_meas_speed = None             # a streak's ~3 m jitter is no speed
        self.acc_hist.clear()
        if P0 is not None:
            self.last_acc = P0
            self.acc_hist.append((now, P0[0], P0[1]))
            if self.trail is not None:          # the breadcrumbs to here are unknown
                self.trail.clear()
                self.trail.add(P0[0], P0[1])
        self.t_ok_last = now
        self.t_meas_last = now
        self.t_meas_last_cap = t_c
        self.state = "committed"
        return seeded, P0

    def _begin_reacquire(self, anchor: dict | None, now: float) -> None:
        self.reacq = self.fv.Reacquirer(need=self.reacq_need,
                                        max_range_m=self.reacq_max_range_m)
        self.reacq.start(anchor)
        self.grounder.begin_reacquire()
        self.state = "reacquiring"

    def _retire(self) -> dict:
        r = self.reacq
        for k, v in r.refused.items():
            self.reacq_refused[k] += v
        return {"refused": dict(r.refused), "judged": r.n, "best_streak": r.best_streak}

    # --------------------------------------------------------------- tick --
    def step(self, g: dict, now: float, pose) -> None:
        """One control tick (the module docstring's order). `g` is this
        tick's Grounder.latest(); `pose` the aircraft's (x, y, yaw rad)."""
        fv = self.fv
        g = g if isinstance(g, dict) else {}
        x, y, yaw = (float(v) for v in pose)
        det, t_det = g.get("det"), g.get("t_det")
        # A NEW held box is a detection once (fly(): keyed on t_det, not on
        # seq - the Grounder republishes its last box through misses).
        fresh_det = bool(det is not None and t_det and t_det != self._last_est_seq)
        if fresh_det:
            self._last_est_seq = t_det
        self.last_lock_why = g.get("lock_why")
        if not self.started:
            return
        if self.reacq is not None:
            # RE-ACQUISITION: each NEW inference's candidates; the box the
            # Grounder publishes meanwhile is nobody's (it holds none).
            det, fresh_det = None, False
            seq = g.get("seq")
            if seq != self._last_reacq_seq and g.get("inf_cap"):
                self._last_reacq_seq = seq
                ic = g["inf_cap"]
                t_ic = ic.get("t_capture") or ic.get("t_cap") or now
                got = self.reacq.step(g.get("cands") or [], g.get("cand_tiers") or [],
                                      g.get("cand_feats") or [], t_ic)
                self.reacq_outcomes["committed" if got else
                                    (self.reacq.last_outcome or "unknown")] += 1
                if got:
                    pick, pf, cp = self.reacq.pick, self.reacq.pick_feat, ic.get("pose")
                    self.grounder.commit(pick, yaw=(cp or {}).get("yaw"))
                    streak = self.reacq.streak
                    retired = self._retire()
                    seeded, _P0 = self._take(pf, ic, now)
                    last_lapse = next((e for e in reversed(self.events)
                                       if e["event"] in ("lapse", "acquire")), None)
                    self._event("reacquired", now,
                                after_s=(None if last_lapse is None
                                         else round(self._mt(now) - last_lapse["t"], 2)),
                                P=[round(float(v), 2) for v in pf["P"][:2]],
                                rng_h=(None if pf.get("rng_h") is None
                                       else round(float(pf["rng_h"]), 1)),
                                seeded=bool(seeded), streak=streak, **retired)
                    self.reacq = None
        elif (self.state == "committed" and self.t_meas_last is not None
              and now - self.t_meas_last > self.lapse_s):
            # LAPSE: no accepted measurement for lapse_s. The estimate is a
            # guess by now; nothing may re-seed it but the Reacquirer.
            anchor = fv.lapse_anchor(self.estimator, self.last_acc, self.t_meas_last_cap,
                                     self.t_meas_last, self.last_meas_speed, self.v_prior)
            unseen = now - self.t_meas_last
            self.estimator.reset()
            self._begin_reacquire(anchor, now)
            det, fresh_det = None, False
            self._event("lapse", now, unseen_s=round(unseen, 2),
                        P=None if anchor is None else [round(v, 2) for v in anchor["P"]],
                        v=(None if anchor is None or anchor["v"] is None
                           else round(anchor["v"], 2)))
        # The strict lock leans on the estimate while there is one.
        if self.reacq is None and self.estimator._xu is not None:
            self.grounder.set_prior(fv.estimator_prior(self.estimator, self.centre_up_m))
        else:
            self.grounder.set_prior(None)
        if det is not None and fresh_det:
            self._measure(det, g, now, (x, y, yaw))

    def _measure(self, det, g: dict, now: float, pose) -> None:
        """Feed the estimator ONCE per new held box (fly()'s feed, identity on)."""
        fv = self.fv
        t_meas, x_m, y_m, yaw_m, b_meas, r_meas, src = fv.estimator_measurement(
            det, g, (now, *pose), True)
        self.meas_src[src] += 1
        tier = g.get("tier")
        feed = True
        if tier != "ok":
            # SOFT: may CONTINUE a live track only.
            feed = fv.soft_may_continue(t_meas, self.estimator.t_last_update, now,
                                        self.t_ok_last)
            self.counts["soft_fed" if feed else "soft_refused"] += 1
        if r_meas is None and self.object_width_m is not None:
            r_meas = fv.implied_range_from_width(det, self.object_width_m)
        if not r_meas:
            self.counts["no_range"] += 1
            return
        if not feed:
            return
        accepted = self.estimator.update(t_meas, x_m, y_m, yaw_m, b_meas, float(r_meas))
        self.counts["accepted" if accepted else "gated_out"] += 1
        if not accepted:
            return
        if tier == "ok":
            self.t_ok_last = now
        self.t_meas_last = now
        xe = self.estimator.x
        self.acc_hist.append((now, float(xe[0]), float(xe[1])))
        del self.acc_hist[:-40]
        self.last_acc = (float(xe[0]), float(xe[1]))
        v = fv.measured_speed(self.acc_hist)
        if v is not None:
            self.last_meas_speed = v
        if self.trail is not None:
            self.trail.add(float(xe[0]), float(xe[1]))

    # ------------------------------------------------------------ readout --
    def served(self, t: float):
        """(x, y) of the subject predicted at `t` while committed and the
        estimator serves one (TargetState.observe's coast), else None."""
        e = self.estimator
        if (self.state != "committed" or e._xu is None or e.t_last_update is None
                or t - e.t_last_update > e.max_coast_s):
            return None
        e.predict(t)
        return float(e.x[0]), float(e.x[1])

    def in_memory(self, t: float) -> bool:
        """Is the subject still held in memory at `t`: committed, or
        re-acquiring with an anchor whose reach is within reach_cap_m?"""
        if self.state == "committed":
            return True
        if self.state == "reacquiring" and self.reacq is not None:
            r = self.reacq.reach_m(t)
            return r is not None and r <= self.reacq.reach_cap_m
        return False

    def memory_bearing(self, t: float, x: float, y: float, yaw: float):
        """(world bearing rad, source) toward where the subject is expected
        from the pose (x, y, yaw) at `t`, or None (module docstring)."""
        if self.state == "committed":
            obs = self.estimator.observe(t, x, y, yaw)
            if obs is not None:
                return _wrap(yaw + obs[0]), "estimate"
        if not self.in_memory(t):
            return None
        if self.trail is not None:
            look = trail_mod.lookout(self.trail, x, y, yaw)
            if look is not None:
                return _wrap(yaw + look), "trail"
        return None

    def hint_memory(self, capture):
        """vla_mission.DetectorHint's `estimate`: memory_bearing at the
        capture's own time and pose."""
        p = capture.pose
        return self.memory_bearing(float(capture.t_capture), float(p.x), float(p.y),
                                   float(p.yaw_rad))

    def row(self, now: float, x: float, y: float, yaw: float) -> dict:
        """The tick's flight-log fields, once per logged tick:
          est_state      acquiring / committed / reacquiring
          est_xy         the estimate served now, [x, y] (None: none served)
          est_age_s      since the estimator's last update (its measurement's
                         capture), None with no estimate
          est_speed_mps  the estimate's ground speed when served
          est_memory     what a pilot without a fresh box is pointed at now:
                         "estimate", "trail" or None
          lock_why       the strict lock's reason this tick held no box
          reacq_streak   the Reacquirer's streak while re-acquiring."""
        xy = self.served(now)
        mem = self.memory_bearing(now, x, y, yaw)
        age = self.estimator.age_s(now)
        self.n_rows += 1
        self.ticks_by_state[self.state] += 1
        if xy is not None:
            self.n_served += 1
            if age is not None:
                self._ages.append(float(age))
        self.memory_ticks[mem[1] if mem is not None else "none"] += 1
        return {"est_state": self.state,
                "est_xy": None if xy is None else [round(xy[0], 2), round(xy[1], 2)],
                "est_age_s": None if age is None else round(float(age), 3),
                "est_speed_mps": None if xy is None else round(self.estimator.speed(), 2),
                "est_memory": None if mem is None else mem[1],
                "lock_why": self.last_lock_why,
                "reacq_streak": None if self.reacq is None else int(self.reacq.streak)}

    def summary(self) -> dict:
        """metrics.json's record: the configuration, the estimator's own
        summary, what was measured and refused, the commits, lapses and
        re-acquisitions (mission seconds), the ticks per state, the ticks an
        estimate was served and its age, what a pilot without a box was
        pointed at, the trail and the strict lock's counters."""
        refused = Counter(self.reacq_refused)
        if self.reacq is not None:
            refused.update(self.reacq.refused)
        lock = getattr(self.grounder, "lock", None)
        tr = self.trail
        return {"on": True, "config": self.config(), "started": self.started,
                "state_at_end": self.state,
                "estimator": self.estimator.summary(),
                "measurements": {**{k: int(self.counts.get(k, 0)) for k in (
                    "accepted", "gated_out", "soft_fed", "soft_refused", "no_range")},
                    "by_source": dict(self.meas_src)},
                "commits": int(self.n_events.get("acquired", 0)
                               + self.n_events.get("reacquired", 0)),
                "lapses": int(self.n_events.get("lapse", 0)),
                "reacquisitions": int(self.n_events.get("reacquired", 0)),
                "events": list(self.events),
                "reacq_open": self.reacq is not None,
                "reacq_refused": dict(refused), "reacq_outcomes": dict(self.reacq_outcomes),
                "ticks": self.n_rows, "ticks_by_state": dict(self.ticks_by_state),
                "ticks_served": self.n_served,
                "served_frac": round(self.n_served / self.n_rows, 4) if self.n_rows else None,
                "est_age_s": _spread(self._ages),
                "memory_ticks": dict(self.memory_ticks),
                "trail": (None if tr is None else
                          {"points_added": tr.n_added, "restarts": tr.n_restarts,
                           "length_m": round(tr.length(), 1)}),
                "lock_stats": (lock.stats() if lock is not None
                               and callable(getattr(lock, "stats", None)) else None)}
