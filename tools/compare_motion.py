"""Compare two follow runs on smoothness, follow quality and safety: the
hand-written follow (demo/follow_vlm.py) and the VLA follow (demo/vla_fly.py
--mission follow) on the SAME definitions.

    python tools/compare_motion.py demo/out/citylife_redcar_nfz_motion2_20261019 \\
        demo/out/vla_follow_redcar_09_20261023
    python tools/compare_motion.py RUN_A RUN_B --json        # the numbers, machine-readable

Read-only. Prints one table: metric | A | B.

THE DEFINITIONS (copied from the motion work's scratch script compare_motion.py
of 2026-10-19/23, which produced the numbers quoted for both controllers -
hand-written follow 19 Oct: cmd accel p95/p99/max 2.60/3.90/5.56 m/s^2, cmd yaw
accel p99/max 0.50/2.09 rad/s^2; VLA follow #9: 3.75/32.68/54.32 and
6.87/18.41 - so numbers printed here compare with those), per flight_log row:

  * tilt = hypot(pitch_deg, roll_deg); a tilt episode is a run of ticks over
    the threshold, ticks less than 1 s apart joined (`clusters`);
  * pitch / roll rate = |diff| / max(dt, 0.05 s) - bursty ticks (two rows a
    few ms apart) would inflate a raw difference quotient;
  * commanded acceleration = |diff of the Shield's EMITTED (vx, vy)| /
    max(dt, 0.05 s); commanded yaw acceleration likewise on emitted yaw_rate;
  * flown acceleration = from the positions: velocity by np.gradient, a
    3-tick moving average, then np.gradient again, the first and last 3
    ticks dropped.

A ZERO THAT WAS NEVER MEASURED. vla_fly's rows carried no pitch or roll until
2026-10-23 (the follow fine-tune wave); the scratch script read an absent
attitude as 0.0, so every VLA run "had" 0 tilt episodes and 0 deg/s of pitch
rate. The numbers below keep that reading (so old tables reproduce), and the
row "attitude recorded" says on how many rows it was there: "0 of N" means
the tilt and rate rows are not measurements.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def load(run) -> tuple[list[dict], dict, dict]:
    run = Path(run)
    with open(run / "flight_log.jsonl", encoding="utf-8") as fh:
        rows = [json.loads(line) for line in fh if line.strip()]
    m = json.loads((run / "metrics.json").read_text(encoding="utf-8")) \
        if (run / "metrics.json").is_file() else {}
    k = json.loads((run / "kpi.json").read_text(encoding="utf-8")) \
        if (run / "kpi.json").is_file() else {}
    return rows, m, k


def clusters(mask, t, gap: float = 1.0) -> int:
    """Runs of True in `mask`, ticks less than `gap` seconds apart joined."""
    n, last = 0, -1e9
    for on, tt in zip(mask, t):
        if on:
            if tt - last > gap:
                n += 1
            last = tt
    return n


def _pct(a, q) -> float:
    return float(np.percentile(a, q)) if len(a) else float("nan")


def _max(a) -> float:
    return float(np.max(a)) if len(a) else float("nan")


def motion_numbers(rows: list[dict]) -> dict:
    """The smoothness numbers of one flight log, as numbers (the table's
    strings are made from these). `attitude_rows`: rows that carried a pitch
    or roll at all."""
    if len(rows) < 2:
        raise ValueError(f"a flight log of {len(rows)} rows has no motion to measure")
    t = np.array([r["t"] for r in rows], dtype=float)
    dt = np.diff(t)
    pitch = np.array([r.get("pitch_deg") or 0.0 for r in rows], dtype=float)
    roll = np.array([r.get("roll_deg") or 0.0 for r in rows], dtype=float)
    att = sum(1 for r in rows if r.get("pitch_deg") is not None or r.get("roll_deg") is not None)
    tilt = np.hypot(pitch, roll)
    dts = np.maximum(dt, 0.05)
    prate = np.abs(np.diff(pitch)) / dts
    rrate = np.abs(np.diff(roll)) / dts
    ex = np.array([r["emitted"]["vx"] for r in rows], dtype=float)
    ey = np.array([r["emitted"]["vy"] for r in rows], dtype=float)
    acc_cmd = np.hypot(np.diff(ex), np.diff(ey)) / dts
    yr = np.array([r["emitted"]["yaw_rate"] for r in rows], dtype=float)
    dyr = np.abs(np.diff(yr)) / dts
    x = np.array([r["x"] for r in rows], dtype=float)
    y = np.array([r["y"] for r in rows], dtype=float)
    acc = np.array([])
    if len(rows) >= 7:
        # As the scratch script: two rows with the same rounded t give an
        # infinite gradient there, which it kept (np.errstate only quiets it).
        with np.errstate(divide="ignore", invalid="ignore"):
            vx, vy = np.gradient(x, t), np.gradient(y, t)
            kk = np.ones(3) / 3
            vxs, vys = np.convolve(vx, kk, "same"), np.convolve(vy, kk, "same")
            acc = np.hypot(np.gradient(vxs, t), np.gradient(vys, t))[3:-3]
    fm = [r.get("fence_mode") for r in rows]
    return {
        "ticks": len(rows), "duration_s": round(float(t[-1] - t[0]), 1),
        "tick_dt_p50_ms": _pct(dt, 50) * 1000, "tick_dt_p95_ms": _pct(dt, 95) * 1000,
        "ticks_lt_10ms": int((dt < 0.010).sum()),
        "ticks_150_300ms": int(((dt > 0.15) & (dt < 0.3)).sum()),
        "attitude_rows": att,
        "pitch_min_deg": float(pitch.min()), "pitch_max_deg": float(pitch.max()),
        "roll_min_deg": float(roll.min()), "roll_max_deg": float(roll.max()),
        "tilt_p95_deg": _pct(tilt, 95), "tilt_p99_deg": _pct(tilt, 99), "tilt_max_deg": _max(tilt),
        "tilt_gt10_episodes": clusters(tilt > 10, t), "tilt_gt15_episodes": clusters(tilt > 15, t),
        "pitch_rate_p95_dps": _pct(prate, 95), "pitch_rate_max_dps": _max(prate),
        "roll_rate_p95_dps": _pct(rrate, 95), "roll_rate_max_dps": _max(rrate),
        "cmd_accel_p95": _pct(acc_cmd, 95), "cmd_accel_p99": _pct(acc_cmd, 99),
        "cmd_accel_max": _max(acc_cmd),
        "cmd_yaw_accel_p99": _pct(dyr, 99), "cmd_yaw_accel_max": _max(dyr),
        "flown_accel_p95": _pct(acc, 95), "flown_accel_p99": _pct(acc, 99),
        "flown_accel_max": _max(acc),
        "fence_mode_switches": sum(1 for a, b in zip(fm, fm[1:]) if a != b),
    }


def motion_summary(rows: list[dict]) -> dict:
    """metrics.json "motion" for a flight (demo/vla_fly.py): motion_numbers
    rounded, with the definitions named."""
    n = motion_numbers(rows)
    out = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in n.items()}
    out["definitions"] = ("tools/compare_motion.py: emitted (Shield output) velocity and yaw "
                          "rate differences over max(dt, 0.05 s); tilt = hypot(pitch, roll), "
                          "episodes joined within 1 s")
    out["attitude_recorded"] = n["attitude_rows"] == n["ticks"]
    return out


def stats(run) -> dict:
    """The table's column for one run folder (the scratch script's keys and
    formats, plus "attitude recorded")."""
    rows, m, k = load(run)
    n = motion_numbers(rows)
    coll = m.get("collisions")
    return {
        "ticks": n["ticks"],
        "duration_s": n["duration_s"],
        "tick dt p50 / p95 ms": f"{n['tick_dt_p50_ms']:.0f} / {n['tick_dt_p95_ms']:.0f}",
        "ticks <10 ms apart": n["ticks_lt_10ms"],
        "ticks 150-300 ms apart": n["ticks_150_300ms"],
        "attitude recorded": f"{n['attitude_rows']} of {n['ticks']} rows"
                             + ("" if n["attitude_rows"] else " (tilt/rates below NOT measured)"),
        "pitch min / max deg": f"{n['pitch_min_deg']:.1f} / {n['pitch_max_deg']:.1f}",
        "roll min / max deg": f"{n['roll_min_deg']:.1f} / {n['roll_max_deg']:.1f}",
        "tilt p95 / p99 / max deg": f"{n['tilt_p95_deg']:.1f} / {n['tilt_p99_deg']:.1f} / "
                                    f"{n['tilt_max_deg']:.1f}",
        "tilt >10 deg episodes": n["tilt_gt10_episodes"],
        "tilt >15 deg episodes": n["tilt_gt15_episodes"],
        "pitch rate p95 / max deg/s": f"{n['pitch_rate_p95_dps']:.1f} / {n['pitch_rate_max_dps']:.1f}",
        "roll rate p95 / max deg/s": f"{n['roll_rate_p95_dps']:.1f} / {n['roll_rate_max_dps']:.1f}",
        "cmd accel p95 / p99 / max m/s2": f"{n['cmd_accel_p95']:.2f} / {n['cmd_accel_p99']:.2f} / "
                                          f"{n['cmd_accel_max']:.2f}",
        "cmd yaw accel p99 / max rad/s2": f"{n['cmd_yaw_accel_p99']:.2f} / {n['cmd_yaw_accel_max']:.2f}",
        "flown accel p95 / p99 / max m/s2": f"{n['flown_accel_p95']:.2f} / {n['flown_accel_p99']:.2f} / "
                                            f"{n['flown_accel_max']:.2f}",
        "fence_mode switches": n["fence_mode_switches"],
        "frac_within_30m": m.get("frac_within_30m"),
        "sep mean / min m": f"{m.get('sep_mean_m')} / {m.get('sep_min_m')}",
        "nfz_s": m.get("nfz_s"),
        "interventions": m.get("interventions"),
        "p0 escape rate": m.get("p0_violation_escape_rate", k.get("p0_violation_escape_rate")),
        "collisions": coll.get("n") if isinstance(coll, dict) else coll,
        "det_hz": m.get("det_hz"),
        "follow_scored": m.get("follow_scored"),
        "motion params": (json.dumps((m.get("params") or {}).get("motion"))[:120]
                          if (m.get("params") or {}).get("motion") else "(legacy)"),
        "tracker smoothing": _smoothing(m),
    }


def _smoothing(m: dict) -> str:
    """The VLA follow's TrackerConfig smoothing as flown (vla_fly params
    .tracker), or '-' for a run without one (the hand-written follow)."""
    tr = (m.get("params") or {}).get("tracker")
    if not isinstance(tr, dict):
        return "-"
    keys = ("smooth_accel_mps2", "smooth_yaw_accel_rps2", "carry_s")
    if not any(k in tr for k in keys):
        return "none (before 2026-10-23)"
    return ", ".join(f"{k}={tr.get(k)}" for k in keys)


def table(a: dict, b: dict, name_a: str, name_b: str) -> str:
    w = max(len(k) for k in a)
    lines = [f"{'metric'.ljust(w)} | {name_a} | {name_b}"]
    for key in a:
        lines.append(f"{key.ljust(w)} | {a[key]} | {b.get(key)}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Compare two follow runs' motion (read-only)")
    ap.add_argument("run_a")
    ap.add_argument("run_b")
    ap.add_argument("--json", action="store_true", help="print both runs' numbers as JSON")
    args = ap.parse_args(argv)
    if args.json:
        out = {}
        for r in (args.run_a, args.run_b):
            rows, _, _ = load(r)
            out[Path(r).name] = motion_numbers(rows)
        print(json.dumps(out, indent=1))
        return 0
    a, b = stats(args.run_a), stats(args.run_b)
    print(table(a, b, Path(args.run_a).name, Path(args.run_b).name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
