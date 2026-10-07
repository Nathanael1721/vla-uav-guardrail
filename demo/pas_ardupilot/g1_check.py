"""Gate G1, measured: the four closed-loop conditions, written as evidence.

The gate (kuanting-vla-uav-guardrail/docs/04-risks-and-fallbacks/
fallback-gates.md, owner: PI) passes only if ALL FOUR hold:

  1. AirSim sensor messages (HIL_GPS, HIL_SENSOR) reach AP SITL and AP's EKF
     converges on the AirSim-reported position to within 1 m.
  2. AP SITL's actuator PWM output drives AirSim's vehicle model to produce
     visible motion in the AirSim viewport.
  3. A scripted square-pattern mission flies in AirSim with AP in AUTO mode,
     completes without breach, and lands within 2 m of takeoff.
  4. The same mission is repeatable across three runs with the same random_seed.

TRANSPORT, condition 1. On this rail the sensors do not travel as MAVLink
HIL_GPS / HIL_SENSOR. Project AirSim's own ArduPilot controller sends one JSON
sensor frame per physics step to ArduPilot's AirSim backend (the interface
Colosseum, the gate's fallback, also uses). The condition's measurable content
- the sensors reach SITL, and the EKF converges on the simulator's position
within 1 m - is checked unchanged; the transport is recorded in every run file.

OPERATIONAL DEFINITIONS, stated so a verdict can be disputed on the record:

  1. EKF position (GLOBAL_POSITION_INT) against the simulator's ground-truth
     geo position, paired in time, over everything after the EKF reports a
     usable position: the 95th percentile of the 3-D error must be <= 1.0 m.
     NULL: the same EKF track against a FROZEN truth (the vehicle never moved).
     The check must beat the null by half, and the vehicle must have travelled
     >= 10 m, or it cannot tell a working bridge from a vehicle parked under a
     converged EKF - the result is then "not measured", not a pass.
  2. While disarmed (the null: no PWM) the simulated vehicle moves < 0.3 m.
     After arming, motor PWM rises above 1100 and the simulated vehicle then
     climbs >= 2 m within 30 s, with the lift-off AFTER the PWM rise. Chase
     camera frames before and during the climb are saved as the viewport
     evidence.
  3. A square (default 10 m side, 10 m up) uploaded as a mission and flown in
     AUTO: AUTO observed, take-off and every corner reported reached
     (MISSION_ITEM_REACHED), landed, the simulator's
     ground truth passing within 2 m of each corner (so a vehicle that never
     left the pad - which "lands within 2 m" trivially - fails), landing within
     2 m of take-off by ground truth, zero simulator collisions and zero
     ArduPilot fence breaches.
  4. At least three runs, all with the same seed, each passing 1-3, with
     landing points and corner passes each spread by <= 2 m across runs.

`passed` is True, False, or None (NOT MEASURED). None never counts as a pass;
G1 passes only when all four are True.

NO SURVIVOR BIAS. A run that crashes is a failed run, not a missing one.
`run` logs the attempt (g1_attempts.jsonl) before anything can fail and
writes g1_run.json from a finally block, with an `error` field when the run
did not complete. `summarize` expects runs 1..N of one invocation: a run that
started and left no record, crashed, or carries another attempt's record
counts as failing conditions 1-3, and a run folder from an earlier use of the
same tag fails condition 4. (Before this, a crashed run simply had no
g1_run.json, and three passing runs plus a crash read as PASSED.)

USAGE (scripts/run_pas_ardupilot.ps1 -Step g1 runs all of this):

    python demo/pas_ardupilot/g1_check.py run --tag g1_20261007 --run 1 \
        --seed 20261006 --network nat --windows-ip A --wsl-ip B --start-sitl
    ... runs 2 and 3 ...
    python demo/pas_ardupilot/g1_check.py summarize --tag g1_20261007 --runs 3
"""
from __future__ import annotations

import argparse
import bisect
import collections
import importlib
import json
import math
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for _p in (ROOT, HERE):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import rail                                                           # noqa: E402
from autopilot import (FrameAnchor, PymavlinkLink, ekf_ready,          # noqa: E402
                       MAV_CMD_SET_MESSAGE_INTERVAL)

# Thresholds: the gate's own numbers, and the definitions above.
EKF_TOL_M = 1.0
EKF_MIN_PAIRS = 50
EKF_SETTLE_S = 2.0
PAIR_MAX_DT_S = 0.15
MIN_TRUTH_PATH_M = 10.0
NULL_RATIO = 0.5
REST_MAX_M = 0.3
ARMED_PWM_MIN = 1100
LIFT_M = 0.5
CLIMB_MIN_M = 2.0
CLIMB_WINDOW_S = 30.0
LAND_TOL_M = 2.0
CORNER_TOL_M = 2.0
N_RUNS = 3
REPEAT_TOL_M = 2.0

TRANSPORT = ("Project AirSim ArduCopterApi JSON sensor frames over UDP 9003 -> "
             "ArduPilot SIM_AirSim; PWM back over UDP 9002 (not MAVLink HIL_*)")

# MAVLink numbers for the mission protocol.
MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_MISSION_START = 300
MAV_MISSION_ACCEPTED = 0
MAV_LANDED_STATE_ON_GROUND = 1
EXTRA_STREAMS = {"FENCE_STATUS": (162, 2.0), "MISSION_CURRENT": (42, 2.0)}


def verdict(passed, reasons=(), **measured) -> dict:
    return {"passed": passed, "reasons": list(reasons), **measured}


def _pct(vals, q):
    if not vals:
        return None
    s = sorted(vals)
    k = (len(s) - 1) * q / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return round(s[lo] + (s[hi] - s[lo]) * (k - lo), 4)


def _dist(a, b, dims=3):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(dims)))


def interp_truth(truth: list, t: float):
    """Truth [t, x, y, up] at time t, linearly interpolated between the two
    samples around it, or None if either is more than PAIR_MAX_DT_S away."""
    if not truth:
        return None
    ts = [r[0] for r in truth]
    i = bisect.bisect_left(ts, t)
    if i == 0 or i >= len(truth):
        j = 0 if i == 0 else len(truth) - 1
        return truth[j][1:] if abs(truth[j][0] - t) <= PAIR_MAX_DT_S else None
    a, b = truth[i - 1], truth[i]
    if t - a[0] > PAIR_MAX_DT_S or b[0] - t > PAIR_MAX_DT_S:
        return None
    f = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
    return [a[k] + f * (b[k] - a[k]) for k in (1, 2, 3)]


# --------------------------------------------------------------------------- #
# The four evaluators. Each takes a recorded run and never touches a live system.
# --------------------------------------------------------------------------- #

def check_ekf(rec: dict) -> dict:
    """Condition 1."""
    t_ready = rec.get("ekf_ready_t")
    base = {"transport": TRANSPORT, "tol_m": EKF_TOL_M, "statistic": "p95 3-D error"}
    if t_ready is None:
        return verdict(None, ["the EKF never reported a usable position"], **base)
    if not rec.get("sitl_clock_advanced"):
        return verdict(False, ["ArduPilot's clock did not advance: no sensor frames "
                               "reached SITL"], **base)
    pairs = []
    for e in rec.get("ekf", []):
        if e[0] < t_ready + EKF_SETTLE_S:
            continue
        tr = interp_truth(rec.get("truth", []), e[0])
        if tr is not None:
            pairs.append((e[1:4], tr))
    if len(pairs) < EKF_MIN_PAIRS:
        return verdict(None, [f"only {len(pairs)} EKF/truth pairs (need "
                              f"{EKF_MIN_PAIRS})"], n_pairs=len(pairs), **base)
    errs = [_dist(e, tr) for e, tr in pairs]
    frozen = pairs[0][1]
    null_errs = [_dist(e, frozen) for e, _ in pairs]
    trs = [tr for _, tr in pairs]
    path = sum(_dist(a, b) for a, b in zip(trs, trs[1:]))
    p95, p95_null = _pct(errs, 95), _pct(null_errs, 95)
    m = dict(n_pairs=len(pairs), p95_m=p95, median_m=_pct(errs, 50),
             max_m=round(max(errs), 4), null_frozen_p95_m=p95_null,
             truth_path_m=round(path, 2), **base)
    if path < MIN_TRUTH_PATH_M:
        return verdict(None, [f"the vehicle travelled {path:.1f} m (< "
                              f"{MIN_TRUTH_PATH_M} m): a converged EKF cannot be "
                              f"told from a frozen one"], **m)
    if p95 > EKF_TOL_M:
        return verdict(False, [f"p95 error {p95} m > {EKF_TOL_M} m"], **m)
    if not p95 < NULL_RATIO * p95_null:
        # Inside 1 m, but a vehicle that never moved would score nearly as
        # well: jitter or creep that a fixed EKF offset also stays close to.
        # The run shows nothing about convergence either way.
        return verdict(None, [f"p95 {p95} m does not beat the frozen-truth null "
                              f"{p95_null} m by half: the motion cannot tell a "
                              f"tracking EKF from a fixed one"], **m)
    return verdict(True, [], **m)


def check_pwm(rec: dict) -> dict:
    """Condition 2."""
    truth, servo = rec.get("truth", []), rec.get("servo", [])
    arm_t, rest = rec.get("arm_t"), rec.get("rest_window")
    frames = rec.get("frames", {})
    if not servo:
        return verdict(None, ["no SERVO_OUTPUT_RAW received"], frames=frames)
    if arm_t is None:
        return verdict(None, ["the vehicle was never armed"], frames=frames)
    if not rest:
        return verdict(None, ["no disarmed rest window was recorded (the null)"],
                       frames=frames)
    rest_pts = [r[1:4] for r in truth if rest[0] <= r[0] <= rest[1]]
    if len(rest_pts) < 5:
        return verdict(None, [f"{len(rest_pts)} truth samples in the rest window"],
                       frames=frames)
    rest_motion = max(_dist(p, rest_pts[0]) for p in rest_pts)
    rest_pwm = [sum(s[1:5]) / 4.0 for s in servo if rest[0] <= s[0] <= rest[1]]
    t_rise = next((s[0] for s in servo
                   if s[0] >= arm_t and sum(s[1:5]) / 4.0 >= ARMED_PWM_MIN), None)
    up0 = interp_truth(truth, arm_t)
    after = [r for r in truth if arm_t <= r[0] <= arm_t + CLIMB_WINDOW_S]
    climb = (max(r[3] for r in after) - up0[2]) if (after and up0) else None
    t_lift = (next((r[0] for r in after if r[3] - up0[2] >= LIFT_M), None)
              if up0 else None)
    m = dict(rest_motion_m=round(rest_motion, 3), rest_pwm_mean=(
        round(sum(rest_pwm) / len(rest_pwm), 1) if rest_pwm else None),
        pwm_rise_t=t_rise, lift_t=t_lift,
        climb_m=None if climb is None else round(climb, 3), frames=frames,
        null="disarmed rest window: no PWM, so no motion")
    reasons = []
    if rest_motion >= REST_MAX_M:
        reasons.append(f"the vehicle moved {rest_motion:.2f} m while disarmed: "
                       f"something other than ArduPilot's PWM moves it")
    if t_rise is None:
        reasons.append(f"motor PWM never rose to {ARMED_PWM_MIN} after arming")
    if climb is None or climb < CLIMB_MIN_M:
        reasons.append(f"climbed {climb} m in {CLIMB_WINDOW_S:.0f} s (need {CLIMB_MIN_M})")
    if t_rise is not None and t_lift is not None and t_lift < t_rise:
        reasons.append("lift-off came BEFORE the PWM rise")
    return verdict(not reasons, reasons, **m)


AIRBORNE_CONTACT_M = 1.0


def airborne_collisions(rec: dict) -> int | None:
    """Simulator contacts while more than 1 m above the take-off point.

    The spawn drop and the touchdown are ground contacts the simulator also
    reports; counting them would fail every flight. A contact in the air is a
    crash into something. None when the collision topic was not subscribed:
    an unmeasured zero is not a zero.
    """
    if not rec.get("collisions_subscribed"):
        return None
    tk = rec.get("takeoff_truth")
    if tk is None:
        return None
    n = 0
    for c in rec.get("collision_events", []):
        p = interp_truth(rec.get("truth", []), c["t"])
        if p is None or p[2] - tk[2] > AIRBORNE_CONTACT_M:
            n += 1                  # unknown height counts against the run
    return n


def check_square(rec: dict) -> dict:
    """Condition 3."""
    mi = rec.get("mission") or {}
    if not mi.get("uploaded"):
        return verdict(None, ["the mission was never accepted by ArduPilot"])
    fails, unmeasured = [], []
    modes = [e.get("mode") for e in rec.get("events", []) if e.get("kind") == "mode"]
    if "AUTO" not in modes:
        fails.append("AUTO mode was never observed")
    missing = sorted(set(mi.get("nav_seqs", [])) - set(mi.get("reached", [])))
    if missing:
        fails.append(f"waypoints never reached: {missing}")
    if rec.get("landed_t") is None:
        fails.append("never landed")
    tk, ld = rec.get("takeoff_truth"), rec.get("landing_truth")
    land_err = None if (tk is None or ld is None) else round(_dist(tk, ld, 2), 3)
    if land_err is None:
        unmeasured.append("no take-off/landing ground truth")
    elif land_err > LAND_TOL_M:
        fails.append(f"landed {land_err} m from take-off (> {LAND_TOL_M} m)")
    air = [r for r in rec.get("truth", [])
           if rec.get("arm_t") is not None and r[0] >= rec["arm_t"]]
    corner_miss = []
    for c in mi.get("corners_scene", []):
        dmin = min((math.hypot(r[1] - c[0], r[2] - c[1]) for r in air), default=None)
        corner_miss.append(None if dmin is None else round(dmin, 3))
    if not corner_miss or any(d is None or d > CORNER_TOL_M for d in corner_miss):
        fails.append(f"corners not all passed within {CORNER_TOL_M} m: {corner_miss}")
    coll, fence = airborne_collisions(rec), rec.get("fence_breach_count")
    if coll is None:
        unmeasured.append("airborne collisions not measured")
    elif coll:
        fails.append(f"{coll} simulator collision(s) while airborne")
    if fence is None:
        unmeasured.append("fence breaches not measured (no FENCE_STATUS)")
    elif fence:
        fails.append(f"{fence} ArduPilot fence breach(es)")
    passed = False if fails else (None if unmeasured else True)
    return verdict(passed, fails + unmeasured, landing_error_m=land_err,
                   corner_min_dist_m=corner_miss, airborne_collisions=coll,
                   fence_breaches=fence, side_m=mi.get("side_m"), alt_m=mi.get("alt_m"),
                   null="a vehicle that never left the pad lands 0 m from take-off; "
                        "the corner check is what it fails")


def check_repeat(runs: list[dict]) -> dict:
    """Condition 4, over the per-run summaries written by `run`."""
    if len(runs) < N_RUNS:
        return verdict(None, [f"{len(runs)} run(s); the gate needs {N_RUNS}"],
                       n_runs=len(runs))
    reasons = []
    seeds = sorted({r.get("seed") for r in runs}, key=str)
    if len(seeds) != 1:
        reasons.append(f"runs used different seeds {seeds}")
    for r in runs:
        bad = [k for k in ("1", "2", "3") if r.get("checks", {}).get(k, {}).get("passed") is not True]
        if bad:
            reasons.append(f"run {r.get('run')} did not pass condition(s) {bad}")
    lands = [r.get("landing_truth") for r in runs]
    spread_land = (max(_dist(a, b, 2) for a in lands for b in lands)
                   if all(lands) else None)
    if spread_land is None:
        reasons.append("a run has no landing ground truth")
    elif spread_land > REPEAT_TOL_M:
        reasons.append(f"landing points spread {spread_land:.2f} m (> {REPEAT_TOL_M})")
    hits = [r.get("corner_hits") or [] for r in runs]
    corner_spread = []
    if hits and all(len(h) == len(hits[0]) and h for h in hits):
        for k in range(len(hits[0])):
            pts = [h[k] for h in hits]
            corner_spread.append(round(max(_dist(a, b, 2) for a in pts for b in pts), 3))
        if any(s > REPEAT_TOL_M for s in corner_spread):
            reasons.append(f"corner passes spread {corner_spread} m (> {REPEAT_TOL_M})")
    else:
        reasons.append("corner passes missing from a run")
    return verdict(not reasons, reasons, n_runs=len(runs), seeds=seeds,
                   landing_spread_m=None if spread_land is None else round(spread_land, 3),
                   corner_spread_m=corner_spread,
                   note="the seed fixes nothing random on our side of a scripted "
                        "mission; the simulator's IMU noise is seeded inside "
                        "Project AirSim and is not controlled by it")


def overall(checks: dict) -> dict:
    vals = [checks.get(k, {}).get("passed") for k in ("1", "2", "3", "4")]
    if all(v is True for v in vals):
        status = "passed"
    elif any(v is False for v in vals):
        status = "fired"
    else:
        status = "not measured"
    return {"g1_passed": status == "passed", "status": status,
            "conditions": {k: checks.get(k, {}).get("passed") for k in ("1", "2", "3", "4")}}


def corner_hits(rec: dict) -> list:
    """For each corner, the ground-truth point of closest approach (for 4)."""
    out = []
    air = [r for r in rec.get("truth", [])
           if rec.get("arm_t") is not None and r[0] >= rec["arm_t"]]
    for c in (rec.get("mission") or {}).get("corners_scene", []):
        if not air:
            return []
        best = min(air, key=lambda r: math.hypot(r[1] - c[0], r[2] - c[1]))
        out.append([round(best[1], 3), round(best[2], 3)])
    return out


# --------------------------------------------------------------------------- #
# The square mission
# --------------------------------------------------------------------------- #

def square_mission(anchor: FrameAnchor, start_xy, side_m: float, alt_m: float):
    """(items, corners_scene): take-off, the four corners of a square north and
    east of the start point, land at the start. Item 0 is the home slot
    ArduPilot overwrites."""
    x0, y0 = start_xy
    corners = [(x0 + side_m, y0), (x0 + side_m, y0 + side_m),
               (x0, y0 + side_m), (x0, y0)]

    def ll(x, y):
        lat, lon = anchor.to_latlon(x, y)
        return int(round(lat * 1e7)), int(round(lon * 1e7))

    s_lat, s_lon = ll(x0, y0)
    items = [{"frame": MAV_FRAME_GLOBAL, "command": MAV_CMD_NAV_WAYPOINT,
              "x": s_lat, "y": s_lon, "z": 0.0, "p1": 0.0},
             {"frame": MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, "command": MAV_CMD_NAV_TAKEOFF,
              "x": s_lat, "y": s_lon, "z": float(alt_m), "p1": 0.0}]
    for cx, cy in corners:
        la, lo = ll(cx, cy)
        items.append({"frame": MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                      "command": MAV_CMD_NAV_WAYPOINT, "x": la, "y": lo,
                      "z": float(alt_m), "p1": 0.0})
    items.append({"frame": MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, "command": MAV_CMD_NAV_LAND,
                  "x": s_lat, "y": s_lon, "z": 0.0, "p1": 0.0})
    return items, [list(c) for c in corners]


def upload_mission(link, items: list[dict], inbox: collections.deque,
                   timeout: float = 20.0) -> tuple[bool, str]:
    """The MAVLink mission upload handshake, over link.pump(). `inbox` receives
    MISSION_REQUEST(_INT) / MISSION_ACK messages from the link's on_message hook."""
    m, ts, tc = link.m, link.target_system, link.target_component
    inbox.clear()
    m.mav.mission_clear_all_send(ts, tc)
    t_clear = link.clock() + 3.0
    while link.clock() < t_clear:                 # the clear's own ACK
        link.pump()
        if any(msg.get_type() == "MISSION_ACK" for msg in inbox):
            break
        time.sleep(0.02)
    inbox.clear()
    m.mav.mission_count_send(ts, tc, len(items))
    sent: set[int] = set()
    deadline = link.clock() + timeout
    while link.clock() < deadline:
        link.pump()
        while inbox:
            msg = inbox.popleft()
            kind = msg.get_type()
            if kind == "MISSION_ACK":
                ok = int(msg.type) == MAV_MISSION_ACCEPTED and len(sent) == len(items)
                return ok, f"ACK type {int(msg.type)} after {len(sent)}/{len(items)} items"
            if kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                seq = int(msg.seq)
                if not 0 <= seq < len(items):
                    return False, f"ArduPilot asked for item {seq} of {len(items)}"
                it = items[seq]
                m.mav.mission_item_int_send(ts, tc, seq, it["frame"], it["command"],
                                            0, 1, it["p1"], 0, 0, 0,
                                            it["x"], it["y"], it["z"])
                sent.add(seq)
        time.sleep(0.02)
    return False, f"timed out after {len(sent)}/{len(items)} items"


# --------------------------------------------------------------------------- #
# Live run (needs Project AirSim, ArduPilot SITL and pymavlink)
# --------------------------------------------------------------------------- #

def _save_frame(msg, path: Path) -> str | None:
    if not msg or not msg.get("data"):
        return None
    try:
        if msg.get("encoding") == "PNG":
            path.write_bytes(bytes(msg["data"]))
        else:
            import cv2
            import numpy as np
            arr = np.frombuffer(bytes(msg["data"]), dtype=np.uint8).reshape(
                msg["height"], msg["width"], 3)
            cv2.imwrite(str(path), arr)
        return path.name
    except Exception as exc:                                  # noqa: BLE001
        print(f"[frame] not saved: {exc}")
        return None


ATTEMPTS_FILE = "g1_attempts.jsonl"


def record_attempt(tag_dir: Path, run: int, seed, attempt_id: str) -> dict:
    """Log that run N of this tag was STARTED, before anything can fail.

    summarize() reads this list, so a run that crashed before it wrote its
    record - or a process killed outright - is still a run that was made, and
    counts against the gate instead of vanishing from it."""
    tag_dir.mkdir(parents=True, exist_ok=True)
    row = {"run": int(run), "seed": seed, "attempt_id": attempt_id,
           "started_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    with open(tag_dir / ATTEMPTS_FILE, "a", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(row) + "\n")
    return row


def read_attempts(tag_dir: Path) -> dict[int, dict] | None:
    """{run: latest attempt} from the attempt log, or None if there is none."""
    p = tag_dir / ATTEMPTS_FILE
    if not p.is_file():
        return None
    out: dict[int, dict] = {}
    for ln in p.read_text(encoding="utf-8").splitlines():
        if ln.strip():
            row = json.loads(ln)
            out[int(row["run"])] = row
    return out


def new_record(a, attempt_id: str | None) -> dict:
    return {"tag": a.tag, "run": a.run, "seed": a.seed, "attempt_id": attempt_id,
            "transport": TRANSPORT, "completed": False, "error": None,
            "sim_config": None, "home": None, "truth": [], "ekf": [], "servo": [],
            "events": [], "frames": {}, "mission": {}, "collision_events": [],
            "collisions_subscribed": False, "fence_breach_count": None,
            "ekf_ready_t": None, "arm_t": None, "landed_t": None,
            "rest_window": None, "statustext": [], "sitl_clock_advanced": None,
            "ardupilot_rt_factor": None, "autopilot": None}


def evaluate(rec: dict) -> dict:
    """The per-run checks, from whatever the run recorded. A crashed run is
    evaluated too (its partial evidence is kept), and summarize() counts it
    as a failed run whatever these say."""
    rec["corner_hits"] = corner_hits(rec)
    rec["checks"] = {"1": check_ekf(rec), "2": check_pwm(rec), "3": check_square(rec)}
    return rec["checks"]


def run_once(a, *, pas=None, link=None) -> dict:
    """One G1 run. g1_run.json is written whatever happens - an EKF that never
    converged, a refused arm, a connect failure, Ctrl+C - with an `error`
    field when the run did not complete; then the error is raised again.

    `pas` (the projectairsim module) and `link` (a PymavlinkLink) can be
    injected so the record-keeping is testable without a simulator."""
    tag_dir = Path(a.out_root) / a.tag
    out = tag_dir / f"run_{a.run}"
    if (out / "g1_run.json").exists() and not getattr(a, "overwrite", False):
        raise FileExistsError(
            f"{out / 'g1_run.json'} already exists: a G1 tag is one invocation. "
            f"Use a fresh --tag (the launcher does), or --overwrite to replace it")
    out.mkdir(parents=True, exist_ok=True)
    attempt_id = uuid.uuid4().hex
    record_attempt(tag_dir, a.run, a.seed, attempt_id)
    rec = new_record(a, attempt_id)
    error = None
    try:
        _fly(a, rec, out, pas=pas, link=link)
        rec["completed"] = True
    except BaseException as exc:                              # noqa: BLE001
        rec["error"] = {"type": type(exc).__name__, "message": str(exc)}
        error = exc
    finally:
        evaluate(rec)
        try:
            from guardrail.manifest import code_revision
            rec["code_revision"] = code_revision(ROOT)
        except Exception:                                     # noqa: BLE001
            rec["code_revision"] = None
        (out / "g1_run.json").write_text(json.dumps(rec, indent=1), encoding="utf-8")
    if error is not None:
        raise error
    return rec


def _fly(a, rec: dict, out: Path, *, pas=None, link=None) -> None:
    """The live part of a run; fills `rec` as it goes, so a failure at any
    step leaves everything recorded up to that step."""
    plan = rail.plan_network(a.network, a.windows_ip, a.wsl_ip)
    cfg = rail.write_sim_configs(out / "pas_config", plan)
    rec["sim_config"] = cfg
    scene = rail.load_jsonc(Path(cfg["dir"]) / cfg["scene"])
    home = rail.home_geo_point(scene)
    rec["home"] = list(home)
    anchor = FrameAnchor(home[0], home[1], home[2], "scene.home-geo-point")
    pas = pas or importlib.import_module("projectairsim")
    client = pas.ProjectAirSimClient(address=a.sim_host)
    t0 = time.time()
    collisions, chase = [], {"msg": None}
    stop = threading.Event()
    sitl = None
    try:
        client.connect()
        world = pas.World(client, cfg["scene"], delay_after_load_sec=2,
                          sim_config_path=cfg["dir"])
        try:
            world.switch_streaming_view()
        except Exception:                                     # noqa: BLE001
            pass
        drone = pas.Drone(client, world, rail.ROBOT_NAME)

        def on_collision(_, m):
            if isinstance(m, dict) and m.get("has_collided", True):
                collisions.append({"t": round(time.time() - t0, 3),
                                   "object": m.get("object_name")})
        try:
            client.subscribe(drone.robot_info["collision_info"], on_collision)
            rec["collisions_subscribed"] = True
        except Exception:                                     # noqa: BLE001
            rec["collisions_subscribed"] = False
        client.subscribe(drone.sensors["Chase"]["scene_camera"],
                         lambda _, m: chase.__setitem__("msg", m))

        def poll_truth():
            while not stop.is_set():
                try:
                    g = drone.get_ground_truth_geo_location()
                    x, y = anchor.to_scene(float(g["latitude"]), float(g["longitude"]))
                    rec["truth"].append([round(time.time() - t0, 3), x, y,
                                         float(g["altitude"]) - home[2]])
                except Exception:                             # noqa: BLE001
                    pass
                time.sleep(0.05)
        threading.Thread(target=poll_truth, daemon=True).start()

        if a.start_sitl:                  # AFTER the scene load; see the scene file
            sitl = subprocess.Popen(rail.wsl_start_command(a.wsl_distro, plan),
                                    stdout=open(out / "sitl_wsl.log", "w"),
                                    stderr=subprocess.STDOUT)
        inbox = collections.deque()
        link = link or PymavlinkLink(a.mavlink_url)

        def hook(msg, t):
            tt = round(t - t0, 3)
            k = msg.get_type()
            if k == "GLOBAL_POSITION_INT":
                x, y = anchor.to_scene(msg.lat / 1e7, msg.lon / 1e7)
                rec["ekf"].append([tt, x, y, msg.alt / 1000.0 - home[2]])
            elif k == "SERVO_OUTPUT_RAW":
                rec["servo"].append([tt, msg.servo1_raw, msg.servo2_raw,
                                     msg.servo3_raw, msg.servo4_raw])
            elif k in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"):
                inbox.append(msg)
            elif k == "MISSION_ITEM_REACHED":
                rec["mission"].setdefault("reached", []).append(int(msg.seq))
            elif k == "FENCE_STATUS":
                rec["fence_breach_count"] = int(msg.breach_count)
        link.on_message = hook
        try:
            link.connect(timeout=240.0)
            link.identify()                   # which firmware flew, and SITL or not
            for _n, (mid, hz) in EXTRA_STREAMS.items():
                link.command(MAV_CMD_SET_MESSAGE_INTERVAL, mid, int(1e6 / hz))
            tb0 = None
            if not link.wait(lambda s: ekf_ready(s.ekf_flags) and s.lat is not None, 240.0):
                raise RuntimeError(f"EKF not ready: {link.snap.statustext[-5:]}")
            rec["ekf_ready_t"] = round(time.time() - t0, 3)
            tb0 = (link.snap.time_boot_ms, link.snap.t_time_boot)
            r0 = time.time()
            link.wait(lambda s: False, a.rest_s)                  # the disarmed null window
            rec["rest_window"] = [round(r0 - t0, 3), round(time.time() - t0, 3)]
            rec["frames"]["before"] = _save_frame(chase["msg"], out / "chase_before.png")

            start = link.state(anchor)
            items, corners = square_mission(anchor, (start.x, start.y), a.side_m, a.alt_m)
            ok, why = upload_mission(link, items, inbox)
            # Take-off and the four corners must each report MISSION_ITEM_REACHED.
            # The final LAND item is judged by landing itself (landed_t and the
            # ground-truth landing point), not by a message the vehicle may still
            # be sending as it disarms.
            rec["mission"].update(uploaded=ok, upload=why, n_items=len(items),
                                  nav_seqs=list(range(1, len(items) - 1)),
                                  corners_scene=corners, side_m=a.side_m, alt_m=a.alt_m)
            if not ok:
                raise RuntimeError(f"mission upload failed: {why}")
            if not link.set_mode("GUIDED"):
                raise RuntimeError("GUIDED refused")
            if not link.arm():
                raise RuntimeError(f"arming refused: {link.snap.statustext[-5:]}")
            rec["arm_t"] = round(time.time() - t0, 3)
            tk = interp_truth(rec["truth"], rec["arm_t"])
            rec["takeoff_truth"] = None if tk is None else [round(v, 3) for v in tk]
            link.set_mode("AUTO")
            link.command(MAV_CMD_MISSION_START, 0, 0)
            airborne = False
            on_ground_since = None
            deadline = time.time() + a.timeout_s
            while time.time() < deadline:
                link.pump()
                up = rec["truth"][-1][3] if rec["truth"] else None
                if not airborne and up is not None and tk and up - tk[2] > 2.0:
                    airborne = True
                    rec["frames"]["airborne"] = _save_frame(chase["msg"],
                                                            out / "chase_airborne.png")
                if airborne:
                    if link.snap.armed is False:
                        break
                    if link.snap.landed_state == MAV_LANDED_STATE_ON_GROUND:
                        on_ground_since = on_ground_since or time.time()
                        if time.time() - on_ground_since > 3.0:
                            break
                    else:
                        on_ground_since = None
                time.sleep(0.05)
            else:
                rec["events"].append({"t": round(time.time() - t0, 3), "kind": "timeout"})
            if airborne and (link.snap.armed is False or on_ground_since):
                rec["landed_t"] = round(time.time() - t0, 3)
                time.sleep(1.0)
                last = [r[1:4] for r in rec["truth"][-20:]]
                rec["landing_truth"] = [round(sum(p[i] for p in last) / len(last), 3)
                                        for i in range(3)] if last else None
            # FCU time paired with the wall time it arrived at (SYSTEM_TIME is
            # 2 Hz; pairing it with "now" biases the factor low).
            tb1 = (link.snap.time_boot_ms, link.snap.t_time_boot)
            rec["sitl_clock_advanced"] = bool(tb0 and tb0[0] is not None
                                              and tb1[0] is not None and tb1[0] > tb0[0])
            rec["ardupilot_rt_factor"] = (
                round((tb1[0] - tb0[0]) / 1000.0 / (tb1[1] - tb0[1]), 3)
                if rec["sitl_clock_advanced"] and tb1[1] - tb0[1] >= 1.0 else None)
        finally:
            # What the autopilot said and did, kept even when a step above
            # raised: the pre-arm messages are usually the reason.
            rec["events"] += [{**e, "t": round(e["t"] - t0, 3)} for e in link.events]
            rec["statustext"] = link.snap.statustext[-50:]
            try:
                rec["autopilot"] = link.evidence()
            except Exception as exc:                          # noqa: BLE001
                rec["autopilot"] = {"error": f"{type(exc).__name__}: {exc}"}
    finally:
        stop.set()
        rec["collision_events"] = list(collisions)
        if sitl is not None:
            subprocess.run(rail.wsl_stop_command(a.wsl_distro), check=False)
        try:
            client.disconnect()
        except Exception:                                     # noqa: BLE001
            pass


def _run_entry(tag_dir: Path, run: int, attempts: dict | None) -> dict:
    """One expected run as summarize() sees it: its checks, or why it counts
    as a failed run (no record, a crash, a record from another invocation)."""
    p = tag_dir / f"run_{run}" / "g1_run.json"
    entry = {"run": run, "file": str(p), "seed": None, "landing_truth": None,
             "corner_hits": None, "checks": {}, "status": "ok"}
    if attempts is not None and run not in attempts:
        entry["status"] = "never started in this invocation (not in the attempt log)"
    elif not p.is_file():
        entry["status"] = "started but left no record (crashed or killed)"
    else:
        r = json.loads(p.read_text(encoding="utf-8"))
        entry.update(seed=r.get("seed"), landing_truth=r.get("landing_truth"),
                     corner_hits=r.get("corner_hits"), checks=r.get("checks", {}))
        if r.get("error"):
            e = r["error"]
            entry["status"] = f"did not complete: {e.get('type')}: {e.get('message')}"
        elif attempts is not None and r.get("attempt_id") != attempts[run].get("attempt_id"):
            entry["status"] = ("its record belongs to another attempt (a stale run "
                               "directory from an earlier invocation of this tag)")
        elif r.get("completed") is not True:
            entry["status"] = "the record does not say the run completed"
    if entry["status"] != "ok":
        # A run that did not complete demonstrated none of the conditions;
        # its measured values stay in its own file.
        entry["measured"] = {k: (entry["checks"].get(k) or {}).get("passed")
                             for k in ("1", "2", "3")}
        entry["checks"] = {k: verdict(False, [f"run {run} {entry['status']}"])
                           for k in ("1", "2", "3")}
    return entry


def summarize(tag_dir: Path, expect_runs: int | None = None) -> dict:
    """The gate verdict over runs 1..N of ONE invocation.

    N is `expect_runs` (the launcher passes -Runs), else the highest run in
    the attempt log. Every run 1..N must have started (attempt log), written
    a record of THIS attempt, and completed; anything else is a failed run,
    never a skipped one. A run directory outside 1..N, or not in the attempt
    log, means the tag was reused, and the verdict refuses to pass."""
    attempts = read_attempts(tag_dir)
    dirs = sorted(int(p.name[4:]) for p in tag_dir.glob("run_*")
                  if p.is_dir() and p.name[4:].isdigit())
    started = sorted(attempts) if attempts is not None else dirs
    n = expect_runs if expect_runs is not None else (max(started) if started else 0)
    expected = list(range(1, n + 1))
    problems = []
    if attempts is None and dirs:
        problems.append(f"no {ATTEMPTS_FILE}: which runs this invocation started "
                        f"cannot be told, so a crashed run could be missing")
    extra = [r for r in sorted(set(dirs) | set(attempts or {})) if r not in expected]
    if extra:
        problems.append(f"run(s) {extra} lie outside runs 1..{n} of this invocation")
    if attempts is not None:
        stale = [r for r in dirs if r not in attempts]
        if stale:
            problems.append(f"run folder(s) {stale} are not in the attempt log: "
                            f"left over from an earlier invocation of this tag")
    runs = [_run_entry(tag_dir, r, attempts) for r in expected]
    ok_runs = [r for r in runs if r["status"] == "ok"]
    first = ok_runs[0]["checks"] if ok_runs else {}
    checks = {}
    for k in ("1", "2", "3"):
        vals = [r["checks"].get(k, {}).get("passed") for r in runs]
        base = first.get(k) or verdict(None, ["no completed run recorded"])
        reasons = list(base.get("reasons", []))
        reasons += [f"run {r['run']}: {r['status']}" for r in runs if r["status"] != "ok"]
        checks[k] = {**base, "reasons": reasons,
                     # Conditions 1-3 must hold in EVERY run, not just the first.
                     "passed": (False if any(v is False for v in vals)
                                else True if vals and all(v is True for v in vals)
                                else None),
                     "per_run": vals}
    c4 = check_repeat(runs)
    if problems:
        c4 = {**c4, "passed": False, "reasons": list(c4.get("reasons", [])) + problems}
    checks["4"] = c4
    res = {"gate": "G1 (fallback-gates.md)", "tag": tag_dir.name, "expected_runs": n,
           "runs": runs, "run_set_problems": problems, "checks": checks,
           **overall(checks)}
    (tag_dir / "g1_verdict.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    lines = [f"# Gate G1 verdict: {res['status'].upper()}", "",
             f"Runs expected: {n} ({tag_dir.name}); completed: {len(ok_runs)}. "
             f"Transport: {TRANSPORT}.", "",
             "| Condition | Passed | Reasons |", "|---|---|---|"]
    for k in ("1", "2", "3", "4"):
        c = checks[k]
        lines.append(f"| {k} | {c.get('passed')} | {'; '.join(c.get('reasons', [])) or '-'} |")
    if problems:
        lines += ["", "Run-set problems: " + "; ".join(problems)]
    (tag_dir / "g1_verdict.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--tag", required=True)
    r.add_argument("--run", type=int, required=True)
    r.add_argument("--seed", type=int, default=20261006)
    r.add_argument("--out-root", default=str(ROOT / "demo" / "out"))
    r.add_argument("--network", choices=["nat", "mirrored"], default="nat")
    r.add_argument("--windows-ip", default=None)
    r.add_argument("--wsl-ip", default=None)
    r.add_argument("--sim-host", default="127.0.0.1")
    r.add_argument("--mavlink-url", default=f"tcp:127.0.0.1:{rail.ROUTER_TCP_PORT}")
    r.add_argument("--start-sitl", action="store_true")
    r.add_argument("--wsl-distro", default="Ubuntu")
    r.add_argument("--side-m", type=float, default=10.0)
    r.add_argument("--alt-m", type=float, default=10.0)
    r.add_argument("--rest-s", type=float, default=5.0)
    r.add_argument("--timeout-s", type=float, default=240.0)
    r.add_argument("--overwrite", action="store_true",
                   help="replace an existing record of this run (not for gate runs)")
    s = sub.add_parser("summarize")
    s.add_argument("--tag", required=True)
    s.add_argument("--out-root", default=str(ROOT / "demo" / "out"))
    s.add_argument("--runs", type=int, default=None,
                   help="how many runs this invocation made (the launcher's -Runs)")
    a = ap.parse_args(argv)
    if a.cmd == "run":
        try:
            rec = run_once(a)
        except FileExistsError as exc:
            print(f"[G1] refused: {exc}")
            return 2
        except Exception as exc:                              # noqa: BLE001
            print(f"[G1] run {a.run} did NOT complete: {type(exc).__name__}: {exc}")
            print(f"[G1] its record (with the error) is in "
                  f"{Path(a.out_root) / a.tag / f'run_{a.run}' / 'g1_run.json'}; "
                  f"summarize counts it as a failed run")
            return 3
        for k, c in rec["checks"].items():
            print(f"[G1] condition {k}: {c['passed']}  {'; '.join(c['reasons'])}")
        return 0
    res = summarize(Path(a.out_root) / a.tag, a.runs)
    print(f"[G1] {res['status'].upper()}: {res['conditions']}")
    for p in res["run_set_problems"]:
        print(f"[G1]   run set: {p}")
    return 0 if res["g1_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
