"""
The follow expert held to the VLA's decision rate: the CEILING flight for the
follow fine-tune (2026-10-24).

    .\\scripts\\run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 `
        -PolicyFile policies\\follow_car_citylife_nfz.yaml -Seconds 420 `
        -Tag ceiling_p10_d08 -DecideEveryS 1.0 -DecideDelayS 0.8
    python demo/follow_vlm.py ... --decide-every-s 1.0 --decide-delay-s 0.8

WHY

The fine-tuned AerialVLA decides once per ~0.8-1.3 s: capture to answer p50
~0.75-0.8 s in flight, and one step is the motion over the next 1.0 s. The
hand-written follow (demo/follow_vlm.py) acts every 0.1 s on OWL-ViT's ~3 Hz
boxes and is within 30 m on 0.963 of its ticks after acquisition; the VLA
flights score 0.16-0.24. Before more training, the CEILING: how well does the
expert itself follow when it may act only at the VLA's rate? If it stays near
0.9, better learning can close the gap; if it collapses, the decision rate is
the bottleneck, and no amount of cloning fixes that.

THE RULE (DecideRate.apply)

The expert's controller and its slew limiter run every tick as on any flight,
on their own state. What is EXECUTED - at the point vla_record.Perturber and
vla_dagger.VLAPerturber sit, after the limiter and before the Shield, which
still filters it - is a zero-order hold of the expert's own command:

  * decisions on the mission clock's grid k*P (k = 0, 1, ...), each taken at
    the first tick at or after its instant. Anchored to the grid, not to the
    previous decision: "last + P" would add the tick's average overshoot
    (~0.05 s at 10 Hz) to every period. A tick that stalls past more than one
    instant takes ONE decision and counts the instants it skipped;
  * a decision carries the newest command the expert computed at least D
    seconds before the tick that takes it (its own history, stamped on the
    same mission clock), so a VLA's capture-to-answer latency is in it too;
  * held, all four axes, until the next decision: a VLA step carries all
    four. Before the expert has a command D seconds old (the mission's first
    decision, at t = 0, when D > 0) the decision is a hover, as a VLA flight
    hovers until its first answer.

The age of the command flown is therefore D at a decision and up to D + P
just before the next (0.8-1.7 s for P = 1.0, D = 0.8 on a 10 Hz tick).

WHAT IT IS NOT. It does not emulate the VLA's step executor (a body-frame
step tracked on the pose; the hold is the expert's world-frame velocity), its
perception (the expert keeps OWL-ViT's boxes and its estimator, which run at
their own rates), or a latency that varies (D is fixed). The follow score is
computed as on any flight (track_truth and follow_vlm's frac_within_30m read
the rows' position and truth, which this does not touch), so it reads
against the expert's 0.963 directly.
"""
from __future__ import annotations

import math
from collections import deque

# The range a flag may take. A VLA decides every ~0.8-1.3 s; more than 10 s is
# a typo, most likely milliseconds typed as seconds ("--decide-delay-s 800").
MAX_S = 10.0
# A command stamped exactly D before a decision is old enough, and a tick
# stamped exactly on k*P takes decision k: float noise (0.3 / 0.1 is
# 2.9999999999999996) must not move either by a tick.
EPS_S = 1e-6

RULE = ("the executed command (after the expert's controller and slew limiter, before the "
        "Shield, which filters it) is a zero-order hold of the expert's own command: on the "
        "mission clock's grid k*every_s, at the first tick at or after each instant, a decision "
        "takes the newest command the expert computed at least delay_s before that tick (a "
        "hover before one exists) and holds it, all four axes, until the next decision; the "
        "controller and the limiter run every tick on their own state; times are mission "
        "seconds")


def check_args(every_s, delay_s) -> str | None:
    """Why follow_vlm should refuse --decide-every-s / --decide-delay-s, or None."""
    try:
        every_s, delay_s = float(every_s), float(delay_s)
    except (TypeError, ValueError):
        return f"--decide-every-s / --decide-delay-s must be seconds, got {every_s!r}, {delay_s!r}"
    if not (math.isfinite(every_s) and every_s > 0):
        return f"--decide-every-s must be a number of seconds above 0, got {every_s!r}"
    if every_s > MAX_S:
        return (f"--decide-every-s {every_s:g} is more than {MAX_S:g} s: it takes seconds, not "
                f"milliseconds (a VLA decides every ~0.8-1.3 s)")
    if not (math.isfinite(delay_s) and delay_s >= 0):
        return f"--decide-delay-s must be 0 or more seconds, got {delay_s!r}"
    if delay_s > MAX_S:
        return (f"--decide-delay-s {delay_s:g} is more than {MAX_S:g} s: it takes seconds, not "
                f"milliseconds (a VLA answers ~0.75-0.8 s after its capture)")
    return None


class DecideRate:
    """The sample-and-hold of the executed command (the module's docstring).
    `apply(action, s, pose=None)` is called once per tick with the expert's
    command after its slew limiter and the mission second s, and returns (the
    command to send to the Shield, the tick's record); the perturbers'
    interface, so the pose is accepted and not read."""

    def __init__(self, every_s: float, delay_s: float = 0.0):
        why = check_args(every_s, delay_s)
        if why is not None:
            raise ValueError(why)
        self.every_s, self.delay_s = float(every_s), float(delay_s)
        self._hist: deque = deque()     # (mission s, the expert's command), oldest first
        self._next_k = 0                # the grid index of the next decision
        self._held = None               # the decision in force...
        self._cmd_t = None              # ...when the expert computed it (None: a hover)
        self._taken_s = None            # ...and the tick that took it
        self.k = -1                     # the decision in force, counted from 0
        self.n_ticks = self.n_hover = self.n_missed = 0
        self._first_s = self._last_s = None       # the first and last decision's tick
        self._dec_age_sum, self._dec_age_n = 0.0, 0
        self._age_sum, self._age_n, self._age_max = 0.0, 0, None

    def describe(self) -> str:
        return (f"the expert acts at a VLA's rate: every {self.every_s:g} s on the mission "
                f"clock it takes its own command from {self.delay_s:g} s before and holds it "
                f"until the next (after the controller and its slew limiter, before the "
                f"Shield; a hover until a command is {self.delay_s:g} s old)")

    def apply(self, action, s: float, pose=None):
        s = float(s)
        self.n_ticks += 1
        self._hist.append((s, action))
        cut = s - self.delay_s + EPS_S
        # Keep the newest command old enough for this tick and everything after
        # it: a later tick's cut is later, so an older one is never taken again.
        while len(self._hist) >= 2 and self._hist[1][0] <= cut:
            self._hist.popleft()
        decide = s + EPS_S >= self._next_k * self.every_s
        if decide:
            k_now = math.floor((s + EPS_S) / self.every_s)
            self.n_missed += max(0, k_now - self._next_k)
            self._next_k = k_now + 1
            t_cmd, cmd = self._hist[0]
            if t_cmd <= cut:
                self._cmd_t, self._held = t_cmd, cmd
                self._dec_age_sum += s - t_cmd
                self._dec_age_n += 1
            else:
                self._cmd_t = None
                self._held = type(action)(vx=0.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)
                self.n_hover += 1
            self.k += 1
            self._taken_s = s
            if self._first_s is None:
                self._first_s = s
            self._last_s = s
        age = None if self._cmd_t is None else s - self._cmd_t
        if age is not None:
            self._age_sum += age
            self._age_n += 1
            self._age_max = age if self._age_max is None else max(self._age_max, age)
        return self._held, {"k": self.k, "held": not decide,
                            "since_s": round(s - self._taken_s, 3),
                            "age_s": None if age is None else round(age, 3),
                            "cmd_t": None if self._cmd_t is None else round(self._cmd_t, 3),
                            "hover": self._cmd_t is None}

    def summary(self) -> dict:
        n_dec = self.k + 1
        return {"every_s": self.every_s, "delay_s": self.delay_s,
                "ticks": self.n_ticks, "decisions": n_dec,
                "held_ticks": self.n_ticks - n_dec,
                "hover_decisions": self.n_hover,
                # Grid instants a stalled tick passed without a decision of their own.
                "missed_instants": self.n_missed,
                # Seconds from one decision to the next (P on a tick that keeps
                # up), and ticks each decision flew.
                "mean_hold_s": (round((self._last_s - self._first_s) / (n_dec - 1), 3)
                                if n_dec >= 2 else None),
                "mean_hold_ticks": round(self.n_ticks / n_dec, 2) if n_dec else None,
                # How old the expert's command was: when a decision took it,
                # and over every tick it flew (hovers excluded).
                "decision_age_s_mean": (round(self._dec_age_sum / self._dec_age_n, 3)
                                        if self._dec_age_n else None),
                "flown_age_s_mean": (round(self._age_sum / self._age_n, 3)
                                     if self._age_n else None),
                "flown_age_s_max": None if self._age_max is None else round(self._age_max, 3),
                "rule": RULE}

    def summary_line(self) -> str:
        """One console line for the flight's end."""
        st = self.summary()
        return (f"[decide] {st['decisions']} decisions every {st['every_s']:g} s, delay "
                f"{st['delay_s']:g} s, over {st['ticks']} ticks: mean hold {st['mean_hold_s']} s, "
                f"command age {st['flown_age_s_mean']} s mean / {st['flown_age_s_max']} s max, "
                f"{st['hover_decisions']} hover(s), {st['missed_instants']} instant(s) skipped")
