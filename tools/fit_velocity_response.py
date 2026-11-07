"""Measure how ArduCopter answers a velocity setpoint, from recorded flights.

guardrail/response.py's constants come from here, and
tests/test_own_velocity_forecast.py re-runs this fit on the committed slim
copy of the flights to check that they still do.

DATA. The six pas_ap_* runs with a flight log (demo/out, gitignored; read
only): the 8 and 12 Oct follows, the 13 Oct close approach with and without
the Shield, and the 14 Oct pair. A row gives the tick time, the EKF position
(GLOBAL_POSITION_INT, projected) and the action actually sent
(`flown_action`; None when nothing was sent). No run logged the velocity, so
the "measured" velocity at a tick is the least-squares slope of the position
over that tick and two either side (0.44 s, centred: a stand-in for the EKF's
velocity estimate, which the rails now read and log as `state_v`).

WINDOWS. A tick starts a window when the autopilot was in GUIDED and a
setpoint went out on every tick from 1.0 s before to 2.5 s after it, with no
gap over 0.3 s. A STEP window is one whose command differs from the measured
velocity by more than 1 m/s - the ticks where the old forecast and the new one
disagree, and the only ones where the response model can be told apart from
"the aircraft flies its command". A DECELERATING window is a step window
whose command along the current direction of travel is more than 1 m/s below
the current speed: the Shield braking for a wall is one.

FIT. The PHYSICAL form: each setpoint actually sent acts from reaction_s
after it went out, and the velocity moves towards the one in force at most
accel_mps2 (guardrail.response.integrate). Grid over reaction_s 0-0.8 s
(0.05) and accel_mps2 1-6 m/s^2 (0.25); objective: the mean over horizons
1.0, 1.5, 2.0 s of the RMS horizontal position error on the step windows.
Its minimum is the IDENTIFIED aircraft (`identified`: 0.35 s, 2.0 m/s^2 on
these flights, the only grid point within 1 % of it), the closed loop's
plant (tools/own_velocity_closed_loop.py).

THE SHIELD'S NUMBERS ARE CHOSEN BY A WRITTEN RULE, not by eye, and in the
form the Shield actually flies (SHIELD_FORM: the pipe of setpoints already
sent, then the candidate held - Shield.note_flown, as every rail that gives
the Shield a velocity calls it): among the grid points whose MEDIAN
along-track error on the decelerating windows at 1.5 s, in that form, is
>= 0 (not short, in the median at that one horizon, of where a braking
aircraft went - short is the unsafe direction for a keep-out rule), the
one with the smallest physical objective (the most accurate description of
the aircraft among those), ties to the longer stop from 5 m/s (the speed
cap of every shipped follow policy).

WHAT THE RULE DOES NOT SAY (2026-10-20 re-review). It is a median at one
horizon, not a bound: with the chosen numbers, in the Shield's form, a
quarter of the decelerating windows are still 0.33 m short or more at
1.5 s (p25 -0.33 m), and at 2.0 s the median itself is short (-0.16 m,
p25 -0.51 m). The physical form with the same numbers stays >= 0 in the
median at every horizon (+0.06 m at 2.0 s), so the later shortfall is
mostly the pilot changing its command, which no Shield form can know; the
table prints these quartiles per horizon (`decel_along_by_h_m`). An
aircraft slower than the chosen numbers still enters a margin in the
closed loop (tools/own_velocity_closed_loop.py --robust).

Why not the identified point itself (2026-10-20 review): in the Shield's
form it is short on braking, median -0.25 m at 1.5 s (p25 -0.51 m), and an
aircraft a little slower than the fit then still enters the margin. The
first version of this tool applied the bias rule in the PHYSICAL form, among
the 1 % ridge only; on these flights the ridge holds one point, whose
physical-form bias is -0.11 m, and the rule fell back to it in silence
(`ok = [...] or ridge`). The rule now reads the Shield's form, and when no
grid point meets it the report says so (`fallback`, printed as a WARNING)
instead of choosing quietly. On these flights it chooses 0.45 s, 1.75 m/s^2:
median braking bias +0.03 m in the Shield's form at 1.5 s (p25 -0.33 m;
at 2.0 s median -0.16 m, p25 -0.51 m), at no cost in the
Shield's own accuracy (step RMS 0.13 / 0.36 / 0.71 m at 0.5 / 1.0 / 1.5 s,
against 0.12 / 0.34 / 0.70 m with the identified numbers); its physical
objective is 13.6 % above the minimum - it describes a slightly slower
aircraft than the one identified, the safer side for a stop (not a safe
one: see WHAT THE RULE DOES NOT SAY).

The table then scores, with those numbers, what the Shield can actually
forecast - it cannot know the next command, so it holds the candidate:
with the setpoint history ("pipe", Shield.note_flown) and without it ("hold",
the measured velocity kept for the reaction time) - against the old
commanded forecast, and the identified numbers beside them.

BRAKE. The FSM's Loiter is flown as ArduCopter BRAKE on this rail. The two
recorded BRAKE entries (pas_ap_close_on) are fitted as an ENVELOPE after the
GUIDED setpoints already in the pipe, flown with the Shield's GUIDED numbers
(the hold plan predicts with those): the (coast, decel) pair with the
smallest RMS among those whose travel at each entry's last recorded sample,
and whose full stop, are never short of the recorded travel (0.05 m slack),
so the hold plan's predicted stop point is never short of a recorded one.

Run:
  python tools/fit_velocity_response.py                 # from demo/out (or --runs-root)
  python tools/fit_velocity_response.py --fixture tests/fixtures/pas_ap_response_id.jsonl.gz
  python tools/fit_velocity_response.py --write-fixture # refresh the committed slim copy
  ... --out report.json                                  # the full table as JSON
"""
from __future__ import annotations

import argparse
import gzip
import io
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail.response import VelocityResponse, integrate           # noqa: E402

RUNS = ("pas_ap_20261008_224440", "pas_ap_20261012_033329", "pas_ap_close_off",
        "pas_ap_close_on", "pas_ap_video_off_20261014", "pas_ap_video_on_20261014")
FIXTURE = ROOT / "tests" / "fixtures" / "pas_ap_response_id.jsonl.gz"
HIST_S = 1.0
HORIZON_S = 2.5
MAX_GAP_S = 0.3
STEP_MPS = 1.0
HORIZONS = (0.5, 1.0, 1.5, 2.0, 2.5)
OBJ_HORIZONS = (1.0, 1.5, 2.0)
GRID_L = [round(0.05 * k, 2) for k in range(0, 17)]             # 0 .. 0.80
GRID_A = [round(1.0 + 0.25 * k, 2) for k in range(0, 21)]       # 1 .. 6.00
RIDGE_TOL = 0.01
CAP_MPS = 5.0
# The form the bias rule is applied in: the one the Shield flies on every rail
# that gives it a velocity (the pipe from Shield.note_flown, then the
# candidate held). See FIT above.
SHIELD_FORM = "pipe"
BRAKE_GRID_L = [round(0.05 * k, 2) for k in range(0, 31)]       # 0 .. 1.50
BRAKE_GRID_A = [round(0.25 + 0.05 * k, 2) for k in range(0, 116)]  # 0.25 .. 6.00
BRAKE_SLACK_M = 0.05
BRAKE_SPAN_S = 2.5


# --------------------------------------------------------------------------- #
# Data
# --------------------------------------------------------------------------- #

def _slim(run: str, r: dict) -> dict:
    fa = r.get("flown_action")
    return {"run": run, "tick": r["tick"], "t": r["t"], "x": r["x"], "y": r["y"],
            "up": r["up"], "ap_mode": r.get("ap_mode"),
            "cmd": None if fa is None else [fa["vx"], fa["vy"], fa["vz_up"]],
            "autopilot_set_mode": r.get("autopilot_set_mode")}


def load_runs(root: Path) -> dict[str, list[dict]]:
    out = {}
    for run in RUNS:
        p = root / run / "flight_log.jsonl"
        if not p.is_file():
            raise SystemExit(f"missing {p}: give --runs-root (the main checkout's demo/out) "
                             f"or --fixture")
        rows = [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines() if l.strip()]
        out[run] = [_slim(run, r) for r in rows]
    return out


def _slim_rig(run: str, r: dict) -> dict:
    """_slim for a ROS 2 rail's row (sitl/ros2_shield_node.py build_row,
    2026-11-07): the command SENT - `emitted` on a "pass" tick, the zero on a
    "brake" tick (the row's `emitted` is then the Shield's unsent action),
    None when the rail sent nothing - ArduPilot's mode (`mode`), and the
    FSM's mode request (`set_mode`), whose LOITER the MAVLink adapter flies
    as BRAKE (its default hold mode)."""
    cmd = None
    if r.get("flown") is True and r.get("setpoint") == "brake":
        cmd = [0.0, 0.0, 0.0]
    elif r.get("flown") is True and r.get("setpoint") == "pass" \
            and isinstance(r.get("emitted"), dict):
        e = r["emitted"]
        cmd = [e.get("vx", 0.0), e.get("vy", 0.0), e.get("vz_up", 0.0)]
    sm = r.get("set_mode")
    return {"run": run, "tick": r["tick"], "t": r["t"], "x": r["x"], "y": r["y"],
            "up": r["up"], "ap_mode": r.get("mode"), "cmd": cmd,
            "autopilot_set_mode": "BRAKE" if sm == "LOITER" else sm}


def load_rig_runs(globs: list[str], exclude: tuple = ()) -> dict[str, list[dict]]:
    """A ROS 2 rail's run folders (globs), each read through _slim_rig; the
    folders whose name matches an `exclude` pattern (fnmatch) left out."""
    import fnmatch
    import glob as _glob
    out = {}
    for g in globs:
        for d in sorted(_glob.glob(g)):
            p = Path(d) / "flight_log.jsonl"
            if not p.is_file() or any(fnmatch.fnmatch(Path(d).name, e) for e in exclude):
                continue
            rows = [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines()
                    if l.strip()]
            out[Path(d).name] = [_slim_rig(Path(d).name, r) for r in rows
                                 if "x" in r and "t" in r]
    if not out:
        raise SystemExit(f"no rail run under {globs}")
    return out


def _order(runs: dict[str, list[dict]]) -> list[str]:
    """The runs in the order the fit reads them: RUNS first (the committed
    fixture's order, so its numbers are what they were), then any other."""
    return [r for r in RUNS if r in runs] + sorted(r for r in runs if r not in RUNS)


def load_fixture(path: Path) -> dict[str, list[dict]]:
    out: dict[str, list[dict]] = {}
    with gzip.open(path, "rt", encoding="utf-8") as fh:
        for line in fh:
            if line.strip():
                r = json.loads(line)
                out.setdefault(r["run"], []).append(r)
    return out


def write_fixture(runs: dict[str, list[dict]], path: Path) -> None:
    text = "".join(json.dumps(r, separators=(",", ":")) + "\n"
                   for run in RUNS for r in runs[run])
    buf = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=buf, mtime=0) as gz:
        gz.write(text.encode("utf-8"))
    path.write_bytes(buf.getvalue())


def _slope(ts, vs) -> float:
    n = len(ts)
    mt, mv = sum(ts) / n, sum(vs) / n
    den = sum((t - mt) ** 2 for t in ts)
    return 0.0 if den <= 0 else sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / den


def velocity_at(rows: list[dict], i: int, half: int = 2) -> tuple[float, float, float]:
    """Least-squares slope of x, y, up over rows i-half .. i+half."""
    lo, hi = max(0, i - half), min(len(rows) - 1, i + half)
    ts = [rows[k]["t"] for k in range(lo, hi + 1)]
    return tuple(_slope(ts, [rows[k][f] for k in range(lo, hi + 1)]) for f in ("x", "y", "up"))


def windows(runs: dict[str, list[dict]]) -> list[dict]:
    out = []
    for run in _order(runs):
        rows = runs[run]
        n = len(rows)
        for i in range(n):
            j0 = i
            while j0 > 0 and rows[i]["t"] - rows[j0]["t"] < HIST_S:
                j0 -= 1
            j1 = i
            while j1 < n - 1 and rows[j1]["t"] - rows[i]["t"] < HORIZON_S:
                j1 += 1
            if (rows[j1]["t"] - rows[i]["t"] < HORIZON_S - 1e-9
                    or rows[i]["t"] - rows[j0]["t"] < HIST_S - 1e-9):
                continue
            seg = rows[j0:j1 + 1]
            if any(r["cmd"] is None or r["ap_mode"] != "GUIDED" for r in seg):
                continue
            if max(b["t"] - a["t"] for a, b in zip(seg, seg[1:])) > MAX_GAP_S:
                continue
            t0, x0, y0, u0 = rows[i]["t"], rows[i]["x"], rows[i]["y"], rows[i]["up"]
            ts = [r["t"] - t0 for r in rows[i:j1 + 1]]
            actual = {}
            for h in HORIZONS:
                actual[h] = tuple(_interp(h, ts, [r[f] - o for r in rows[i:j1 + 1]])
                                  for f, o in (("x", x0), ("y", y0), ("up", u0)))
            out.append({"run": run, "tick": rows[i]["tick"], "v0": velocity_at(rows, i),
                        "c": tuple(rows[i]["cmd"]), "actual": actual,
                        # every setpoint sent from HIST_S before to the end
                        # of the window: (send time from now, (vx, vy))
                        "sent": [(r["t"] - t0, (r["cmd"][0], r["cmd"][1])) for r in seg]})
    return out


def _interp(x, xs, ys):
    for k in range(1, len(xs)):
        if xs[k] >= x:
            f = (x - xs[k - 1]) / (xs[k] - xs[k - 1])
            return ys[k - 1] + f * (ys[k] - ys[k - 1])
    return ys[-1]


def classify(ws: list[dict]) -> tuple[list[dict], list[dict]]:
    steps, dec = [], []
    for w in ws:
        (vx, vy, _), (cx, cy, _) = w["v0"], w["c"]
        if math.hypot(cx - vx, cy - vy) <= STEP_MPS:
            continue
        steps.append(w)
        sp = math.hypot(vx, vy)
        if sp > STEP_MPS and (cx * vx + cy * vy) / sp < sp - STEP_MPS:
            dec.append(w)
    return steps, dec


# --------------------------------------------------------------------------- #
# Scoring
# --------------------------------------------------------------------------- #

def _segments(w: dict, m: VelocityResponse, future: bool) -> list:
    """The rate-limited response's command segments for window `w`: each
    setpoint acts from its send time + reaction_s. future=True uses every
    setpoint actually sent (the PHYSICAL form: what the aircraft did with
    what it was told); False only those sent before now plus this tick's,
    held (the Shield's form: the pipe, then this command)."""
    L = m.reaction_s
    sent = [(t, c) for t, c in w["sent"] if future or t < 1e-9]
    if not future:
        sent = [(t, c) for t, c in sent if t < -1e-9] + [(0.0, (w["c"][0], w["c"][1]))]
    acting, starts = None, []
    for t, c in sent:
        tau = t + L
        if tau <= 1e-12:
            acting = c
        else:
            starts.append((tau, c))
    segs, t0, cur = [], 0.0, acting
    for tau, c in starts:
        if tau > t0 + 1e-12:
            segs.append((tau - t0, *(cur if cur is not None else (None, None))))
            t0 = tau
        cur = c
    segs.append((math.inf, *(cur if cur is not None else (None, None))))
    return segs


def predict(w: dict, model, h: float) -> tuple[float, float]:
    """Displacement at `h`: "commanded" (the old forecast), "measured"
    (constant measured velocity), or (VelocityResponse, form) with form
    "physical" (every setpoint sent, the identification), "pipe" (the
    Shield's form with the setpoint history: guardrail.shield with
    note_flown) or "hold" (the Shield's form without it: the measured
    velocity kept for the reaction time)."""
    (vx, vy, _), (cx, cy, _) = w["v0"], w["c"]
    if model == "commanded":
        return cx * h, cy * h
    if model == "measured":
        return vx * h, vy * h
    m, form = model
    if form == "hold":
        return m.displacement(vx, vy, cx, cy, h)
    dx, dy, _, _ = integrate(vx, vy, _segments(w, m, form == "physical"), [h],
                             m.accel_mps2)[0]
    return dx, dy


def errors(ws: list[dict], model) -> dict:
    out = {}
    for h in HORIZONS:
        es = []
        for w in ws:
            px, py = predict(w, model, h)
            ax, ay, _ = w["actual"][h]
            es.append(math.hypot(px - ax, py - ay))
        es.sort()
        out[h] = {"mean": sum(es) / len(es), "rms": math.sqrt(sum(e * e for e in es) / len(es)),
                  "p95": es[min(len(es) - 1, int(0.95 * len(es)))]}
    return out


def objective(errs: dict) -> float:
    return sum(errs[h]["rms"] for h in OBJ_HORIZONS) / len(OBJ_HORIZONS)


def along_errors(ws: list[dict], model, h: float = 1.5) -> list[float]:
    """Along-track errors (forecast minus actual, along the measured
    direction of travel) at `h`, sorted: negative = the forecast is short."""
    vals = []
    for w in ws:
        vx, vy, _ = w["v0"]
        sp = math.hypot(vx, vy)
        ux, uy = vx / sp, vy / sp
        px, py = predict(w, model, h)
        ax, ay, _ = w["actual"][h]
        vals.append((px - ax) * ux + (py - ay) * uy)
    vals.sort()
    return vals


def along_bias(ws: list[dict], model, h: float = 1.5) -> float:
    """Median along-track error at `h` (along_errors): negative = short."""
    vals = along_errors(ws, model, h)
    return vals[len(vals) // 2]


def along_quartiles(ws: list[dict], model, h: float = 1.5) -> dict:
    """p25 / median / p75 of the along-track error at `h`, m."""
    vals = along_errors(ws, model, h)
    n = len(vals)
    return {"p25": vals[n // 4], "median": vals[n // 2], "p75": vals[(3 * n) // 4]}


def vertical(ws: list[dict], reaction_s: float, accel_mps2: float) -> dict:
    """The vertical channel on windows whose commanded climb differs from the
    measured one by > 0.3 m/s: the commanded forecast (what the Shield keeps)
    against the response model applied to the climb rate with the fitted
    horizontal numbers. Also how often a climb is commanded at all."""
    vw = [w for w in ws if abs(w["c"][2] - w["v0"][2]) > 0.3]
    cz = sorted(abs(w["c"][2]) for w in ws)
    out = {"windows": len(vw), "cmd_climb_p95_mps": cz[int(0.95 * (len(cz) - 1))]}
    if not vw:
        return out
    m = VelocityResponse(reaction_s, accel_mps2, "vertical probe")

    def resp_z(w, h):
        # the same rule on one axis: displacement() with the climb as "x"
        dz, _ = m.displacement(w["v0"][2], 0.0, w["c"][2], 0.0, h)
        return dz

    def rms(fn, h):
        return math.sqrt(sum((fn(w, h) - w["actual"][h][2]) ** 2 for w in vw) / len(vw))
    out["rms_m"] = {str(h): {"commanded": rms(lambda w, hh: w["c"][2] * hh, h),
                             "response": rms(resp_z, h)} for h in (0.5, 1.0, 1.5, 2.0)}
    return out


def fit(runs: dict[str, list[dict]], current: VelocityResponse | None = None) -> dict:
    """The grid, the identified point, the rule's choice and the table (FIT).
    `current` (the --rig mode: the Shield's numbers in use) is scored beside
    them on the same windows: its objective, its braking error in the
    Shield's form at every horizon, and the BRAKE envelope behind it."""
    ws = windows(runs)
    steps, dec = classify(ws)
    grid = []
    for L in GRID_L:
        for a in GRID_A:
            m = VelocityResponse(L, a, "grid")
            grid.append({"reaction_s": L, "accel_mps2": a,
                         "objective": objective(errors(steps, (m, "physical"))),
                         "decel_bias_m": along_bias(dec, (m, "physical")),
                         # the bias in the form the Shield flies (FIT, SHIELD_FORM)
                         "shield_decel_bias_m": along_bias(dec, (m, SHIELD_FORM)),
                         "stop_from_cap_m": m.stop_distance(CAP_MPS)})
    best = min(g["objective"] for g in grid)
    ridge = [g for g in grid if g["objective"] <= best * (1 + RIDGE_TOL)]
    identified = min(grid, key=lambda g: (g["objective"], -g["stop_from_cap_m"]))
    ok = [g for g in grid if g["shield_decel_bias_m"] >= 0.0]
    fallback = None
    if ok:
        chosen = min(ok, key=lambda g: (g["objective"], -g["stop_from_cap_m"]))
    else:
        # Nothing on the grid is long enough on braking in the Shield's form:
        # take the least short point and SAY so (the first version fell back
        # to the ridge in silence).
        chosen = max(grid, key=lambda g: (g["shield_decel_bias_m"], -g["objective"]))
        fallback = (f"no grid point has a median braking bias >= 0 in the Shield's "
                    f"form ({SHIELD_FORM}); chose the least short, "
                    f"{chosen['shield_decel_bias_m']:+.3f} m")
    model = VelocityResponse(chosen["reaction_s"], chosen["accel_mps2"], "fit")
    ident = VelocityResponse(identified["reaction_s"], identified["accel_mps2"], "identified")
    table = {}
    rows_ = [("commanded (old forecast)", "commanded"),
             ("measured, constant", "measured"),
             ("Shield form, measured velocity kept (no pipe)", (model, "hold")),
             ("Shield form, with the pipe (note_flown)", (model, "pipe")),
             ("physical form, the Shield's numbers", (model, "physical")),
             ("Shield form, with the pipe, identified numbers", (ident, "pipe")),
             ("physical form, identified numbers", (ident, "physical"))]
    if current is not None:
        rows_ += [("Shield form, with the pipe, the current numbers", (current, "pipe")),
                  ("physical form, the current numbers", (current, "physical"))]
    for name, m in rows_:
        table[name] = {"all": errors(ws, m), "steps": errors(steps, m),
                       "decel": errors(dec, m), "decel_bias_m": along_bias(dec, m),
                       "decel_along_m": along_quartiles(dec, m),
                       # The same quartiles at every horizon: the bias rule
                       # reads one median at 1.5 s, and its claim must be
                       # read with the spread and the later horizons beside
                       # it (WHAT THE RULE DOES NOT SAY).
                       "decel_along_by_h_m": {h: along_quartiles(dec, m, h)
                                              for h in HORIZONS}}
    out = {"windows": len(ws), "step_windows": len(steps), "decel_windows": len(dec),
           "runs": _order(runs), "best_objective": best,
           "ridge": sorted(ridge, key=lambda g: g["objective"]),
           "identified": identified, "shield_form": SHIELD_FORM,
           "chosen": chosen, "fallback": fallback, "table": table}
    out["vertical"] = vertical(ws, chosen["reaction_s"], chosen["accel_mps2"])
    out["brake"] = fit_brake(runs, model)
    # The same envelope behind the IDENTIFIED GUIDED pipe: the closed loop's
    # BRAKE plant (tools/own_velocity_closed_loop.py), whose GUIDED plant is
    # the identified aircraft. Each envelope is consistent with its own pipe.
    out["brake_identified"] = fit_brake(runs, ident)
    if current is not None:
        cur = next((g for g in grid if abs(g["reaction_s"] - current.reaction_s) < 1e-9
                    and abs(g["accel_mps2"] - current.accel_mps2) < 1e-9), None)
        out["current"] = {
            "reaction_s": current.reaction_s, "accel_mps2": current.accel_mps2,
            "objective": objective(errors(steps, (current, "physical"))),
            "shield_decel_bias_m": along_bias(dec, (current, SHIELD_FORM)),
            "decel_bias_m": along_bias(dec, (current, "physical")),
            "stop_from_cap_m": current.stop_distance(CAP_MPS),
            "on_grid": cur is not None,
            "brake": fit_brake(runs, current)}
    return out


def brake_entries(runs: dict[str, list[dict]], guided: VelocityResponse) -> list[dict]:
    """Each tick the rail asked ArduCopter for BRAKE: the measured velocity,
    the GUIDED setpoints still in the pipe (sent within guided.reaction_s
    before the request, the request tick's own included - the FSM's handover
    tick streams a stop), and the along-track travel (along the measured
    direction) at each later tick while the autopilot was in BRAKE or still
    in GUIDED, up to BRAKE_SPAN_S. The pipe matters: at pas_ap_close_on tick
    280 the aircraft was at 1.4 m/s, accelerating towards the -3.5 m/s sent
    0.2 s earlier, and at 1.9 m/s 0.4 s into BRAKE."""
    from guardrail.response import pipe_segments
    out = []
    for run in _order(runs):
        rows = runs[run]
        for i, r in enumerate(rows):
            if r.get("autopilot_set_mode") != "BRAKE":
                continue
            vx, vy, _ = velocity_at(rows, i)
            sp = math.hypot(vx, vy)
            if sp < 0.2:
                continue
            ux, uy = vx / sp, vy / sp
            flown = [(q["t"], *(q["cmd"][:2] if q["cmd"] is not None else (None, None)))
                     for q in rows[max(0, i - 20):i + 1]]
            # Sent at the request tick too: pipe_segments takes "now" just after it.
            pipe = pipe_segments(flown, r["t"] + 1e-6, guided.reaction_s)
            samples = []
            for k in range(i, len(rows)):
                q = rows[k]
                dt = q["t"] - r["t"]
                if dt > BRAKE_SPAN_S or (k > i and q["ap_mode"] not in ("BRAKE", "GUIDED")):
                    break
                samples.append((dt, (q["x"] - r["x"]) * ux + (q["y"] - r["y"]) * uy))
            out.append({"run": run, "tick": r["tick"], "v": (vx, vy), "u": (ux, uy),
                        "measured_speed": sp, "pipe": pipe, "samples": samples})
    return out


def brake_travel(e: dict, guided: VelocityResponse, lag_b: float, a_b: float,
                 times) -> list[float]:
    """Along-track travel the BRAKE model forecasts for entry `e`: the GUIDED
    pipe at the GUIDED limit, then lag_b of coast, then zero at a_b."""
    segs = guided.reaction(e["pipe"]) + [(lag_b, None, None), (math.inf, 0.0, 0.0, a_b)]
    ux, uy = e["u"]
    return [dx * ux + dy * uy for dx, dy, _, _ in
            integrate(e["v"][0], e["v"][1], segs, times, guided.accel_mps2)]


def fit_brake(runs: dict[str, list[dict]], guided: VelocityResponse) -> dict:
    ents = brake_entries(runs, guided)
    if not ents:
        return {"entries": 0}
    # An ENVELOPE on where each entry got to: the model's travel at the last
    # recorded sample, and its full stop distance, are at least the recorded
    # travel there (0.05 m slack). Not at every sample: in the first second
    # the timing of the GUIDED pipe is the GUIDED model's (its own RMS is
    # 0.3 m at 1 s), and the hold choice reads where the stop path goes, not
    # when it passes each point. Among those, the smallest RMS over every
    # sample.
    best = None
    for L in BRAKE_GRID_L:
        for a in BRAKE_GRID_A:
            errs, feasible = [], True
            for e in ents:
                ts = [dt for dt, _ in e["samples"]] + [60.0]
                pred = brake_travel(e, guided, L, a, ts)
                obs = [s for _, s in e["samples"]]
                if (pred[-2] < obs[-1] - BRAKE_SLACK_M
                        or pred[-1] < max(obs) - BRAKE_SLACK_M):
                    feasible = False
                    break
                errs.extend(p - s for p, s in zip(pred[:-1], obs))
            if not feasible:
                continue
            rms = math.sqrt(sum(x * x for x in errs) / len(errs))
            if best is None or rms < best[0]:
                best = (rms, L, a)
    return {"entries": len(ents),
            "entry_ticks": [{"run": e["run"], "tick": e["tick"],
                             "measured_speed_mps": round(e["measured_speed"], 3),
                             "travel_m": round(max(s for _, s in e["samples"]), 3),
                             "over_s": round(e["samples"][-1][0], 3)} for e in ents],
            "chosen": None if best is None else
            {"reaction_s": best[1], "accel_mps2": best[2], "rms_m": best[0]}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs-root", default=str(ROOT / "demo" / "out"))
    ap.add_argument("--fixture", default=None)
    ap.add_argument("--write-fixture", action="store_true")
    ap.add_argument("--out", default=None)
    ap.add_argument("--rig", action="append", default=None, metavar="RUN_GLOB",
                    help="fit a ROS 2 rail's own runs instead (e.g. the hil rig's "
                         "demo/out/hil_smoke/rh01_*), by the same rule, with the "
                         "Shield's current numbers (ARDUCOPTER_GUIDED) scored beside it; "
                         "repeatable. Reports only: guardrail/response.py is not changed")
    ap.add_argument("--rig-exclude", action="append", default=None, metavar="NAME_GLOB",
                    help="with --rig: run folders left out (fnmatch); default "
                         "*near-edge-at-12mps* (commands the airframe cannot reach)")
    args = ap.parse_args(argv)
    current = None
    if args.rig:
        from guardrail.response import ARDUCOPTER_GUIDED
        current = ARDUCOPTER_GUIDED
        runs = load_rig_runs(args.rig, tuple(args.rig_exclude or ("*near-edge-at-12mps*",)))
        print(f"rig runs: {len(runs)}")
    else:
        runs = (load_fixture(Path(args.fixture)) if args.fixture
                else load_runs(Path(args.runs_root)))
    if args.write_fixture and not args.rig:
        write_fixture(runs, FIXTURE)
        print(f"wrote {FIXTURE} ({FIXTURE.stat().st_size} bytes)")
    rep = fit(runs, current)
    c, i = rep["chosen"], rep["identified"]
    print(f"windows {rep['windows']}, step {rep['step_windows']}, decelerating "
          f"{rep['decel_windows']}")
    print(f"identified (physical minimum): reaction {i['reaction_s']} s, accel "
          f"{i['accel_mps2']} m/s^2 (objective {i['objective']:.3f}, braking bias in the "
          f"Shield's form {i['shield_decel_bias_m']:+.3f} m)")
    print(f"chosen for the Shield: reaction {c['reaction_s']} s, accel {c['accel_mps2']} m/s^2 "
          f"(objective {c['objective']:.3f}, +{100 * (c['objective'] / rep['best_objective'] - 1):.1f} % "
          f"over the minimum; braking bias in the Shield's form "
          f"{c['shield_decel_bias_m']:+.3f} m, physical {c['decel_bias_m']:+.3f} m; stop from "
          f"{CAP_MPS:g} m/s {c['stop_from_cap_m']:.2f} m)")
    if rep["fallback"]:
        print(f"WARNING: the bias rule FELL BACK: {rep['fallback']}")
    for name, t in rep["table"].items():
        q = t["decel_along_m"]
        print(f"  {name:46s} step RMS " + " ".join(
            f"{h:g}s {t['steps'][h]['rms']:.2f}" for h in HORIZONS)
              + f" | decel along-track p25/median/p75 {q['p25']:+.2f} {q['median']:+.2f} "
                f"{q['p75']:+.2f} m")
    by_h = rep["table"]["Shield form, with the pipe (note_flown)"]["decel_along_by_h_m"]
    print("braking error of the Shield's numbers in the Shield's form, p25 / median by "
          "horizon (the rule reads the 1.5 s median only): " + "; ".join(
              f"{h:g}s {q['p25']:+.2f} / {q['median']:+.2f} m" for h, q in by_h.items()))
    if rep.get("current"):
        cur = rep["current"]
        cby = rep["table"]["Shield form, with the pipe, the current numbers"]["decel_along_by_h_m"]
        print(f"current Shield numbers: reaction {cur['reaction_s']} s, accel "
              f"{cur['accel_mps2']} m/s^2 (objective {cur['objective']:.3f}, "
              f"+{100 * (cur['objective'] / rep['best_objective'] - 1):.1f} % over the minimum; "
              f"braking bias in the Shield's form {cur['shield_decel_bias_m']:+.3f} m, "
              f"physical {cur['decel_bias_m']:+.3f} m; stop from {CAP_MPS:g} m/s "
              f"{cur['stop_from_cap_m']:.2f} m)")
        print("braking error of the current numbers in the Shield's form, p25 / median by "
              "horizon: " + "; ".join(f"{h:g}s {q['p25']:+.2f} / {q['median']:+.2f} m"
                                      for h, q in cby.items()))
        print(f"BRAKE (behind the current GUIDED pipe): {cur['brake'].get('chosen')}")
    print(f"vertical: {rep['vertical']}")
    print(f"BRAKE (behind the Shield's GUIDED pipe): {rep['brake']}")
    print(f"BRAKE (behind the identified GUIDED pipe, the closed loop's plant): "
          f"{rep['brake_identified']['chosen']}")
    if args.out:
        Path(args.out).write_text(json.dumps(rep, indent=2, default=str), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
