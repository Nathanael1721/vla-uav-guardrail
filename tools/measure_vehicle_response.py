"""Measure the ArduPilot vehicle response the stress harness's `ardupilot`
response arm flies, from the recorded ArduPilot flights in demo/out.

    python tools/measure_vehicle_response.py                 # print the fit
    python tools/measure_vehicle_response.py --json out.json # and keep it
    python tools/measure_vehicle_response.py --check         # exit 1 when the
                                                             # constants drift

WHAT IT FITS

experiments/sweep_scenarios.py VehicleResponse - the SAME class the harness
flies - to every recorded flight whose log carries the command it sent and
the position it read: a command acts `delay_s` after the tick whose state was
read, the velocity target moves toward it under an acceleration limit and a
jerk limit (the sqrt-controller shape of AC_PosControl's velocity shaping),
and the airframe follows the target with a first-order lag `tau_s`. The
vertical axis gets its own three. Nothing here is a second model of the
vehicle: whatever the fit says is what the arm flies (sweep_scenarios.
AP_RESPONSE holds the numbers this prints, and `--check` fails when they
drift apart).

THE DATA, AND WHAT EACH KIND CAN SAY

  pas_ap_*  Project AirSim's physics flown by ArduPilot Copter-4.5.7
            (`projectairsim-ardupilot-pymavlink`, docs/DESIGN-projectairsim-
            ardupilot.md). Positions are ArduPilot's EKF (GLOBAL_POSITION_INT
            at 20 Hz), and demo/pas_ardupilot/autopilot.py pump() drained
            every message each tick from the first version (9bdc331), so a
            row's position is at most one 50 ms message old. These runs fix
            the delay AND the shape. Rows whose command was not flown (the
            FSM handed the aircraft to LOITER / BRAKE / RTL, `flown` false,
            or ArduPilot not in GUIDED) end a segment: the autopilot's own
            modes are not the GUIDED velocity controller.

  sitl_*    ArduPilot's own SITL airframe (`ardupilot-sitl-pymavlink`).
            Every stored sitl_* log read a BACKLOGGED pose: sitl/
            run_sitl_demo.py state() consumed one queued 10 Hz
            LOCAL_POSITION_NED per tick until 2026-10-19, while the loop
            ticked at 9.3-9.6 Hz (its docstring). So row k's position is the
            stream's k-th sample - on the stream's own clock, `start + P*k`
            - not the vehicle at the row's time. The fit therefore gives
            each run its own clock (an offset and a period P near 0.1 s),
            which absorbs the delay: these runs say nothing about delay_s,
            and are fitted for the SHAPE (acceleration, jerk, lag) only.
            sitl_shield_off's hover-to-6 m/s step is the one recording that
            saturates the acceleration limit, so the arm's acceleration, jerk
            and lag come from this fit (constants_from says why).

THE LOSS

A velocity is finite-differenced over +/- 2 samples (0.4 s) from the logged
positions, and the SAME operator is applied to the model's positions at the
same instants, so the smoothing is on both sides. The first 2 s of every
segment are left out (the model starts from the measured velocity, with no
history of commands). The loss is the RMS of the vector difference.

The search is a coordinate pattern search, in pure Python, so it runs in
the harness's own environment (no scipy): from a start point it tries each
parameter up and down by a step, keeps any move that lowers the loss, and
halves the steps when none does.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import sweep_scenarios as S                                       # noqa: E402

# The recorded flights this measures, by kind (module docstring). Paths are
# under the MAIN checkout's demo/out, which is gitignored: a worktree or a CI
# runner has none, and the tool then says so instead of fitting nothing.
PAS_RUNS = ("pas_ap_20261008_224440", "pas_ap_20261012_033329", "pas_ap_close_on",
            "pas_ap_close_off", "pas_ap_video_on_20261014", "pas_ap_video_off_20261014")
# Shield-off runs are the cleanest step responses (hover to a constant 6 m/s
# command); the Shield-on ones add the repairs' direction changes. Left out:
# sitl_ped_on (120 s: its backlog grew past 6 s, and whether the socket
# buffer dropped samples on the way cannot be told from the log) and the
# sitl_ped_* Shield-off runs, which repeat sitl_shield_off's command exactly.
SITL_RUNS = ("sitl_shield_off", "sitl_shield_on", "sitl_shield_on_dynamic",
             "sitl_ped_side_on", "sitlfix_on_dyn", "sitl_if_on_dyn")
HALF = 2            # samples each side of the finite difference
BURN_S = 2.0        # seconds left out at the start of every segment
MIN_SEG = 2 * HALF + 3


def default_out_dirs() -> list[Path]:
    """Where demo/out may be: this checkout's, then the main checkout's when
    this is a worktree under <main>/.claude/worktrees/<name>."""
    out = [ROOT / "demo" / "out"]
    parts = ROOT.parts
    if ".claude" in parts:
        out.append(Path(*parts[:parts.index(".claude")]) / "demo" / "out")
    return out


def find_run(name: str, dirs: list[Path]) -> Path | None:
    for d in dirs:
        p = d / name / "flight_log.jsonl"
        if p.is_file():
            return p
    return None


# --------------------------------------------------------------------------- #
# reading a log into segments
# --------------------------------------------------------------------------- #

def _commanded(r: dict) -> tuple[float, float, float] | None:
    """The velocity this row SENT to ArduPilot's GUIDED controller, or None
    when the row sent nothing GUIDED flew (the FSM's autopilot hand-over, a
    row the node did not fly, ArduPilot in another mode)."""
    if r.get("flown") is False:
        return None
    if r.get("setpoint") not in (None, "pass", "brake"):
        return None
    if r.get("ap_mode") not in (None, "GUIDED"):
        return None
    a = r.get("flown_action") or r.get("emitted")
    if not isinstance(a, dict):
        return None
    c = (a.get("vx", 0.0), a.get("vy", 0.0), a.get("vz_up", 0.0))
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in c):
        return None
    return float(c[0]), float(c[1]), float(c[2])


def segments(path: Path, min_rows: int = MIN_SEG) -> list[dict]:
    """Maximal runs of consecutive GUIDED-flown rows: per row its time, its
    position and the command it sent."""
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()]
    rows = [r for r in rows if "t" in r and "x" in r and "y" in r and "up" in r]
    segs, cur = [], []
    for r in rows:
        c = _commanded(r)
        if c is None:
            if len(cur) >= min_rows:
                segs.append(cur)
            cur = []
            continue
        cur.append((float(r["t"]), float(r["x"]), float(r["y"]), float(r["up"]), c))
    if len(cur) >= min_rows:
        segs.append(cur)
    return [{"t": [s[0] for s in seg], "pos": [(s[1], s[2], s[3]) for s in seg],
             "cmd": [s[4] for s in seg]} for seg in segs]


def _fd(times: list[float], pos: list[tuple], k: int) -> tuple[float, float, float]:
    """The +/- HALF finite-difference velocity at sample k."""
    a, b = pos[k - HALF], pos[k + HALF]
    dt = times[k + HALF] - times[k - HALF]
    return ((b[0] - a[0]) / dt, (b[1] - a[1]) / dt, (b[2] - a[2]) / dt)


def _start_velocity(times, pos) -> tuple[float, float, float]:
    """The measured velocity at a segment's first sample (forward difference
    over 2 * HALF samples)."""
    a, b = pos[0], pos[2 * HALF]
    dt = times[2 * HALF] - times[0]
    return ((b[0] - a[0]) / dt, (b[1] - a[1]) / dt, (b[2] - a[2]) / dt)


# --------------------------------------------------------------------------- #
# the model on a segment
# --------------------------------------------------------------------------- #

def _simulate(params, seg) -> tuple[list[float], list[tuple]]:
    """Model positions at substep resolution through one segment: the
    command of row j issued at row j's time, as the rail issued it. Returns
    (times, positions) of the trace, starting at the segment's first row."""
    t0 = seg["t"][0]
    veh = S.VehicleResponse(params, v0=_start_velocity(seg["t"], seg["pos"]), t0=t0)
    ts, ps = [t0], [(0.0, 0.0, 0.0)]
    x = y = z = 0.0
    for j in range(len(seg["t"]) - 1):
        veh.command(seg["cmd"][j])
        trace: list = []
        veh.advance(seg["t"][j + 1] - seg["t"][j], trace)
        for tt, dx, dy, dz in trace:
            ts.append(tt)
            ps.append((x + dx, y + dy, z + dz))
        x, y, z = ps[-1]
    return ts, ps


def _interp(ts: list[float], ps: list[tuple], t: float) -> tuple[float, float, float]:
    """Linear interpolation of the trace at t (clamped to its ends)."""
    if t <= ts[0]:
        return ps[0]
    if t >= ts[-1]:
        return ps[-1]
    lo, hi = 0, len(ts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ts[mid] <= t:
            lo = mid
        else:
            hi = mid
    f = (t - ts[lo]) / (ts[hi] - ts[lo])
    a, b = ps[lo], ps[hi]
    return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]))


def _residuals(seg, sample_t: list[float], trace, axis: str) -> list[float]:
    """Squared residuals of the finite-difference velocity, model against the
    log, at every usable sample (module docstring, "THE LOSS")."""
    ts, ps = trace
    model = [_interp(ts, ps, t) for t in sample_t]
    out = []
    t_start = sample_t[0]
    for k in range(HALF, len(sample_t) - HALF):
        if sample_t[k - HALF] - t_start < BURN_S:
            continue
        vm = _fd(sample_t, seg["pos"], k)
        vs = _fd(sample_t, model, k)
        if axis == "xy":
            out.append((vm[0] - vs[0]) ** 2 + (vm[1] - vs[1]) ** 2)
        else:
            out.append((vm[2] - vs[2]) ** 2)
    return out


def pas_loss(params, segs: list[dict], axis: str = "xy") -> tuple[float, int]:
    """RMS over the pas_ap segments; rows are sampled at their own times."""
    sq: list[float] = []
    for seg in segs:
        sq += _residuals(seg, seg["t"], _simulate(params, seg), axis)
    return (math.sqrt(sum(sq) / len(sq)) if sq else math.inf), len(sq)


# A sitl_* log's clock (module docstring): row k of a segment sampled at
# t_first - offset + P * k. Searched per run inside every evaluation, coarse
# (0.1 s) then fine (0.02 s) on the offset, over a few periods near the
# stream's nominal 0.1 s (SITL's clock against the wall one).
SITL_OFFSETS = [round(0.1 * i, 2) for i in range(0, 31)]          # 0 .. 3 s
SITL_PERIODS = (0.100, 0.102, 0.104)


def _sitl_run_sq(segs, traces, per: float, off: float) -> list[float]:
    sq: list[float] = []
    for seg, tr in zip(segs, traces):
        t0 = seg["t"][0]
        st = [t0 - off + per * k for k in range(len(seg["t"]))]
        sq += _residuals(seg, st, tr, "xy")
    return sq


def sitl_loss(params, runs: dict[str, list[dict]]) -> tuple[float, int, dict]:
    """RMS over the sitl_* runs with each run's best clock (offset, period),
    and the clocks chosen. Delay is held at 0: the offset absorbs it."""
    p0 = S.ResponseParams(**dict(params.record(), delay_s=0.0))
    sq_all: list[float] = []
    clocks = {}
    for name, segs in runs.items():
        traces = [_simulate(p0, seg) for seg in segs]
        best = None
        for per in SITL_PERIODS:
            for off in SITL_OFFSETS:
                sq = _sitl_run_sq(segs, traces, per, off)
                if sq and (best is None or sum(sq) < best[0]):
                    best = (sum(sq), sq, per, off)
        if best is None:
            continue
        per0, off0 = best[2], best[3]
        for i in range(-5, 6):
            off = round(off0 + 0.02 * i, 2)
            if off < 0:
                continue
            sq = _sitl_run_sq(segs, traces, per0, off)
            if sq and sum(sq) < best[0]:
                best = (sum(sq), sq, per0, off)
        sq_all += best[1]
        clocks[name] = {"period_s": best[2], "offset_s": best[3]}
    return (math.sqrt(sum(sq_all) / len(sq_all)) if sq_all else math.inf), len(sq_all), clocks


# --------------------------------------------------------------------------- #
# the search
# --------------------------------------------------------------------------- #

def pattern_search(fn, x0: dict, steps: dict, lo: dict, hi: dict,
                   min_frac: float = 0.02, max_evals: int = 400) -> tuple[dict, float, int]:
    """Coordinate pattern search over the keys of x0 (module docstring)."""
    x = dict(x0)
    fx = fn(x)
    step = dict(steps)
    n = 1
    while n < max_evals:
        moved = False
        for k in list(x):
            for sgn in (+1, -1):
                y = dict(x)
                y[k] = min(hi[k], max(lo[k], x[k] + sgn * step[k]))
                if y[k] == x[k]:
                    continue
                fy = fn(y)
                n += 1
                if fy < fx - 1e-9:
                    x, fx, moved = y, fy, True
                    break
        if not moved:
            if all(step[k] <= steps[k] * min_frac for k in step):
                break
            step = {k: v / 2.0 for k, v in step.items()}
    return x, fx, n


def _params(xy: dict, z: dict | None = None) -> S.ResponseParams:
    z = z or {"accel_z_mps2": math.inf, "jerk_z_mps3": math.inf, "tau_z_s": 0.0}
    return S.ResponseParams(delay_s=xy["delay_s"], accel_mps2=xy["accel_mps2"],
                            jerk_mps3=xy["jerk_mps3"], tau_s=xy["tau_s"], **z)


def fit(dirs: list[Path], quick: bool = False, fresh: tuple[str, ...] = (),
        backlogged: tuple[str, ...] = ()) -> dict:
    """The whole fit. `fresh` adds runs whose rows read a FRESH position (as
    pas_ap_* do, and as every sitl/run_sitl_demo.py run does since
    2026-10-19, when its state() began draining the pose queue) to the set
    that fixes the delay and the shape; `backlogged` adds runs read like the
    stored sitl_* (each on its own clock, shape only)."""
    fresh_runs = tuple(PAS_RUNS) + tuple(fresh)
    back_runs = tuple(SITL_RUNS) + tuple(backlogged)
    found = {n: find_run(n, dirs) for n in fresh_runs + back_runs}
    missing = sorted(n for n, p in found.items() if p is None)
    pas = {n: segments(found[n]) for n in fresh_runs if found[n] is not None}
    sitl = {n: segments(found[n]) for n in back_runs if found[n] is not None}
    pas_segs = [s for segs in pas.values() for s in segs]
    if not pas_segs:
        raise FileNotFoundError(f"none of {list(fresh_runs)} under {[str(d) for d in dirs]}")
    out: dict = {"runs_used": {"fresh_position": sorted(pas), "backlogged": sorted(sitl)},
                 "runs_missing": missing,
                 "segments": {n: len(s) for n, s in {**pas, **sitl}.items()},
                 "samples_xy": None}
    evals = 120 if quick else 400

    # ---- the ideal vehicle, the null every fit must beat --------------------
    ideal = _params({"delay_s": 0.0, "accel_mps2": math.inf, "jerk_mps3": math.inf,
                     "tau_s": 0.0})
    out["ideal_rms_xy"], out["samples_xy"] = pas_loss(ideal, pas_segs, "xy")
    out["ideal_rms_z"], _ = pas_loss(ideal, pas_segs, "z")
    # ... and a pure delay, the best a shift alone can do.
    best_d = min(((pas_loss(_params({"delay_s": d, "accel_mps2": math.inf,
                                     "jerk_mps3": math.inf, "tau_s": 0.0}),
                            pas_segs)[0], d) for d in [0.05 * i for i in range(0, 25)]))
    out["pure_delay"] = {"delay_s": round(best_d[1], 3), "rms_xy": round(best_d[0], 4)}

    # ---- horizontal: delay, acceleration, jerk, lag on pas_ap ---------------
    def f_xy(x):
        return pas_loss(_params(x), pas_segs, "xy")[0]
    x0 = {"delay_s": 0.3, "accel_mps2": 2.5, "jerk_mps3": 5.0, "tau_s": 0.3}
    xy, rms_xy, n_xy = pattern_search(
        f_xy, x0, steps={"delay_s": 0.1, "accel_mps2": 0.5, "jerk_mps3": 2.0, "tau_s": 0.1},
        lo={"delay_s": 0.0, "accel_mps2": 0.3, "jerk_mps3": 0.5, "tau_s": 0.0},
        hi={"delay_s": 1.5, "accel_mps2": 10.0, "jerk_mps3": 60.0, "tau_s": 2.0},
        max_evals=evals)
    out["xy"] = {k: round(v, 3) for k, v in xy.items()}
    out["rms_xy"] = round(rms_xy, 4)
    out["evals_xy"] = n_xy
    # Sensitivity: how much each parameter alone moves the loss at +/- 25 %
    # (a flat direction is a parameter the data cannot pin).
    sens = {}
    for k, v in xy.items():
        row = {}
        for f in (0.75, 1.25):
            y = dict(xy, **{k: v * f})
            row[f"x{f}"] = round(f_xy(y), 4)
        sens[k] = row
    out["sensitivity_xy"] = sens

    # ---- vertical, with the horizontal delay ------------------------------
    def f_z(z):
        return pas_loss(_params(xy, z), pas_segs, "z")[0]
    z0 = {"accel_z_mps2": 1.0, "jerk_z_mps3": 5.0, "tau_z_s": 0.3}
    z, rms_z, n_z = pattern_search(
        f_z, z0, steps={"accel_z_mps2": 0.4, "jerk_z_mps3": 2.0, "tau_z_s": 0.1},
        lo={"accel_z_mps2": 0.2, "jerk_z_mps3": 0.5, "tau_z_s": 0.0},
        hi={"accel_z_mps2": 10.0, "jerk_z_mps3": 60.0, "tau_z_s": 2.0},
        max_evals=evals)
    out["z"] = {k: round(v, 3) for k, v in z.items()}
    out["rms_z"] = round(rms_z, 4)
    out["evals_z"] = n_z

    # ---- the second airframe: shape only, each run on its own clock ---------
    if sitl:
        fitted = _params(xy, z)
        rms_s, n_s, clocks = sitl_loss(fitted, sitl)
        out["sitl_with_pas_shape"] = {"rms_xy": round(rms_s, 4), "samples": n_s,
                                      "clocks": clocks}
        rms_i, _, _ = sitl_loss(ideal, sitl)
        out["sitl_ideal_rms_xy"] = round(rms_i, 4)

        def f_s(x):
            return sitl_loss(_params(dict(x, delay_s=0.0)), sitl)[0]
        s0 = {k: xy[k] for k in ("accel_mps2", "jerk_mps3", "tau_s")}
        sx, rms_sx, n_sx = pattern_search(
            f_s, s0, steps={"accel_mps2": 0.5, "jerk_mps3": 2.0, "tau_s": 0.1},
            lo={"accel_mps2": 0.3, "jerk_mps3": 0.5, "tau_s": 0.0},
            hi={"accel_mps2": 10.0, "jerk_mps3": 60.0, "tau_s": 2.0},
            max_evals=60 if quick else 150)
        out["sitl_own_shape"] = {**{k: round(v, 3) for k, v in sx.items()},
                                 "rms_xy": round(rms_sx, 4), "evals": n_sx}
    return out


def constants_from(fit_doc: dict) -> dict:
    """The AP_RESPONSE values a fit implies, rounded as the harness pins them.

    Which fit each constant comes from (docs/DESIGN-harness-vehicle-
    response.md, "Which numbers the arm flies"):

      delay_s            pas_ap: the only recordings whose position is fresh,
                         so the only ones where a delay can be told apart
                         from a backlog
      accel, jerk, tau   sitl_* (ArduPilot's own SITL airframe), when present:
                         sitl_shield_off is the one recording that saturates
                         the acceleration limit cleanly (a hover-to-6 m/s
                         step), where the pas_ap follows change their command
                         smoothly (|dv| per tick p90 0.07 m/s) and leave the
                         limit weakly pinned; and its 2.4 m/s^2 is the lower
                         of the two airframes' (pas_ap's own fit, 3.8, is
                         kept beside it): the longer braking distance, the
                         pessimistic side, as the wind model takes the lower
                         end of its airframe's authority. The two airframes
                         agree on the jerk (5.1 and 5.7 m/s^3) and on a lag
                         of a few hundredths of a second.
      *_z                pas_ap (the sitl_* logs barely move vertically)
    """
    xy, z = fit_doc["xy"], fit_doc["z"]
    shape = fit_doc.get("sitl_own_shape") or xy
    return {"delay_s": round(xy["delay_s"], 2), "accel_mps2": round(shape["accel_mps2"], 1),
            "jerk_mps3": round(shape["jerk_mps3"], 1), "tau_s": round(shape["tau_s"], 2),
            "accel_z_mps2": round(z["accel_z_mps2"], 1),
            "jerk_z_mps3": round(z["jerk_z_mps3"], 1), "tau_z_s": round(z["tau_z_s"], 2)}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out-dir", action="append", default=None,
                    help="where the recorded runs are (default: this checkout's "
                         "demo/out, then the main checkout's)")
    ap.add_argument("--json", default=None, help="write the fit here")
    ap.add_argument("--quick", action="store_true",
                    help="fewer evaluations: an indication only - the SITL shape "
                         "has a jerk / lag ridge the short search stops on "
                         "(measured: jerk 7.2, tau 0.15 against the full search's "
                         "5.7, 0.03), so --check needs the full search (~4 min)")
    ap.add_argument("--check", action="store_true",
                    help="exit 1 when sweep_scenarios.AP_RESPONSE differs from this "
                         "fit by more than the tolerance (and 3 when no run is on disk)")
    ap.add_argument("--tol", type=float, default=0.15,
                    help="--check's relative tolerance per constant (default 0.15)")
    ap.add_argument("--fresh-run", action="append", default=[],
                    help="another recorded run whose rows read a fresh position "
                         "(a sitl/run_sitl_demo.py run flown since 2026-10-19), "
                         "added to the delay-and-shape set; repeatable")
    ap.add_argument("--backlogged-run", action="append", default=[],
                    help="another run read like the stored sitl_* (shape only); "
                         "repeatable")
    args = ap.parse_args(argv)
    dirs = [Path(d) for d in args.out_dir] if args.out_dir else default_out_dirs()
    try:
        doc = fit(dirs, quick=args.quick, fresh=tuple(args.fresh_run),
                  backlogged=tuple(args.backlogged_run))
    except FileNotFoundError as e:
        print(f"NO RECORDED FLIGHTS: {e}")
        return 3
    doc["constants"] = constants_from(doc)
    doc["harness_constants"] = S.AP_RESPONSE.record()
    print(json.dumps(doc, indent=2, default=str))
    if args.json:
        Path(args.json).write_text(json.dumps(doc, indent=2, default=str) + "\n",
                                   encoding="utf-8")
    if args.check:
        bad = []
        for k, v in doc["constants"].items():
            h = getattr(S.AP_RESPONSE, k)
            if not (math.isfinite(h) and abs(h - v) <= args.tol * max(abs(v), 0.05)):
                bad.append(f"{k}: harness {h}, fit {v}")
        if bad:
            print("AP_RESPONSE DRIFTED from the recorded flights: " + "; ".join(bad))
            return 1
        print("AP_RESPONSE matches the fit within the tolerance")
    return 0


if __name__ == "__main__":
    sys.exit(main())
