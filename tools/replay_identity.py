"""Replay the logged red-car flights through demo/identity.py: label, tune, validate.

WHY

demo/identity.py turns a detection box into physical features (width in
metres, height of its bottom edge above the road, distance from the street)
and rules on them. This tool supplies the evidence behind those rules. Every
detection the flights logged is joined to the pose it was taken from and to
where the red car really was. Each box is labelled on-car or off-car from
geometry alone. The thresholds are grid-searched and then checked
LEAVE-ONE-FLIGHT-OUT, so the numbers reported for a flight come from rules
that never saw it.

THE JOIN. detections.jsonl has one record per inference: seq, t (epoch, when
the result was published), infer_ms, and the chosen `det` box. When
acquiring it also has every candidate. flight_log.jsonl has one row per
control tick with det_seq, det_age_s, pose and the car's truth. A record
joins to the row with its seq that has the smallest det_age_s: the first
tick that used it. That row supplies rng_m, the depth range for the box.
When the presence gate blocked the box, rng_m is None, but gate_why says
"implies 0.8 m wide at 29 m" or "... it is 46 m away" and the range is
parsed from that text. The older flights (_carpolicy, _far, _ground, _high,
_pedpolicy, _trail) logged no gate_why, so a blocked box there has NO
range. That is a gap in the log, not in the sensor, so those boxes are
counted but kept out of tuning (r_src "unlogged").

POSE AT CAPTURE, NOT AT THE TICK. The frame was captured before inference,
so the tick's pose lags it by infer_ms + det_age_s + the image's own age.
Clocks are aligned per flight with median(t_rec - (t_row - det_age_s)). The
spread of that offset is 2-22 ms on these flights. Pose and truth are then
interpolated at t_rec - infer_ms - cap_lag. cap_lag is the image's age when
the detector fetched it. It is estimated by minimising the median bearing
residual of the boxes within 6 deg of the car: 0.2 s. Measured with the tick
pose instead, the median residual of those boxes is 1.5-2.7 deg. With the
capture pose it is 0.2-1.4 deg. The 3 deg on-car tolerance is meaningless
without the capture pose.

LABELS (per box, geometry only):
  on-car   the box centre ray's world azimuth is within max(3 deg,
           atan(2.5 m / range)) of the car's, AND the depth range is within
           max(4 m, 15 %) of the slant range to the car's centre (0.75 m up)
  off-car  azimuth more than 8 deg away, and (beyond the on-car tolerance
           + 3 deg, or with a range that disagrees). The brief said
           "> 8 deg". Inside ~18 m the car's own angular extent is wider
           than that, and 9-13 deg off the car's centre at 8-10 m is still
           the car.
  ambiguous  everything else, including a box on the car's bearing with no
           range. It is excluded from the rates.
The azimuth uses camera_model.ray_world with the logged pitch and roll. At
row v it is not pixel_bearing(cx): with the 20 deg nose-down mount the two
differ by up to ~7 deg at the bottom corners.

TUNING. A grid over width_min, aspect_min, score_min, bottom_max,
off_street_hard and the three far rules. Objective: maximise the off-car
not-OK rate subject to on-car not-OK <= 15 % and on-car HARD <= 3 %.
Ties go to the lower on-car not-OK rate, then to the higher off-car HARD
rate. Counting is vectorised (near and far boxes are disjoint, so
|H or S| = |H| + |S| - |H and S| per group via one matrix product), which
makes the whole grid x 12 folds a few seconds.

Usage:
    python tools/replay_identity.py
    python tools/replay_identity.py --no-write-thresholds   # report only
    python tools/replay_identity.py --pose join             # tick-pose sensitivity

Writes demo/out/identity_replay/report.json and report.md, and (unless
--no-write-thresholds) demo/identity_thresholds.json.
Tests: tests/test_replay_identity.py.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import itertools
import json
import math
import re
import statistics
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))

import camera_model as cam      # noqa: E402
import identity                 # noqa: E402

OUT_ROOT = ROOT / "demo" / "out"
REPORT_DIR = OUT_ROOT / "identity_replay"
FLIGHT_GLOB = "citylife_redcar_*"

HFOV_DEG = 90.0
NOUN = "car"
CAR_CENTRE_UP_M = 0.75          # truth pts are the car's footprint centre
ON_BEARING_MIN_DEG = 3.0
ON_BEARING_HALF_M = 2.5         # half a car length, as an angle at range
ON_RANGE_ABS_M = 4.0
ON_RANGE_FRAC = 0.15
OFF_BEARING_DEG = 8.0
OFF_MARGIN_DEG = 3.0            # beyond the on-car tolerance, see label_box
SLIVER_PX = 15.0
CAP_LAG_GRID = [round(0.025 * k, 3) for k in range(0, 17)]      # 0..0.4 s

MAX_ON_NOTOK = 0.15
MAX_ON_HARD = 0.03

GRID = {
    "width_min": [0.8, 0.9, 1.0, 1.1, 1.2],
    "aspect_min": [0.7, 0.75, 0.8, 0.85, 0.9],
    "score_min": [0.0, 0.01, 0.015],
    "bottom_max": [1.5, 1.75, 2.0, 2.25, 2.5],
    "off_street_hard": [2.0, 2.5, 3.0, 3.5, 4.0],
    "far_aspect_min": [1.0, 1.1, 1.2, 1.3, 1.4, 1.5],
    "far_h_px_max": [10.0, 12.0, 14.0, 16.0, 20.0],
    "far_off_street": [1.0, 1.5, 2.0, 2.5, 3.0],
}

_RANGE_PATTERNS = (re.compile(r"\bwide at ([0-9]+(?:\.[0-9]+)?) m"),
                   re.compile(r"\bit is ([0-9]+(?:\.[0-9]+)?) m away"))


# ---------------------------------------------------------------------------
# loading and joining
# ---------------------------------------------------------------------------
def load_jsonl(path: Path) -> list:
    out = []
    with Path(path).open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def join_detections(rows: list, dets: list) -> list:
    """[(record, row)] for every detection record with a box, paired with the
    flight-log row carrying its seq that has the smallest det_age_s. Records
    no row used (the detector warming up before the flight) are dropped."""
    best = {}
    for r in rows:
        q = r.get("det_seq")
        if q is None or r.get("det_age_s") is None:
            continue
        if q not in best or r["det_age_s"] < best[q]["det_age_s"]:
            best[q] = r
    return [(rec, best[rec["seq"]]) for rec in dets
            if rec.get("det") is not None and rec.get("seq") in best]


def clock_offset(pairs: list) -> Optional[float]:
    """Epoch minus flight clock: median(t_rec - (t_row - det_age_s))."""
    offs = [rec["t"] - (row["t"] - row["det_age_s"]) for rec, row in pairs]
    return statistics.median(offs) if offs else None


def parse_gate_range(text: Optional[str]) -> Optional[float]:
    """The range in a presence-gate reason, or None. Handles
    "implies 0.8 m wide at 29 m; ..." and "... it is 46 m away"."""
    if not text:
        return None
    for pat in _RANGE_PATTERNS:
        m = pat.search(text)
        if m:
            return float(m.group(1))
    return None


def range_for(row: dict):
    """(range m or None, source) for the joined row's box.
    depth: rng_m. gate_why: parsed from a blocked box's reason. unlogged: the
    box was blocked and the log kept no range. None: depth had nothing."""
    if row.get("rng_m") is not None:
        return float(row["rng_m"]), "depth"
    if row.get("presence_blocked"):
        r = parse_gate_range(row.get("gate_why"))
        if r is not None:
            return r, "gate_why"
        return None, "unlogged"
    return None, None


class Timeline:
    """Pose and truth of one flight, interpolated by flight-clock time.
    Yaw is unwrapped before interpolating. pitch/roll are None when the flight
    did not log them."""

    def __init__(self, rows: list):
        rows = [r for r in rows if r.get("t") is not None]
        rows.sort(key=lambda r: r["t"])
        self.t = np.array([r["t"] for r in rows], dtype=float)
        self.x = np.array([r["x"] for r in rows], dtype=float)
        self.y = np.array([r["y"] for r in rows], dtype=float)
        self.up = np.array([r["up"] for r in rows], dtype=float)
        self.psi = np.unwrap(np.array([r["psi"] for r in rows], dtype=float))
        self.has_att = all(r.get("pitch_deg") is not None for r in rows)
        self.pitch = (np.radians([r["pitch_deg"] for r in rows])
                      if self.has_att else None)
        self.roll = (np.radians([r.get("roll_deg") or 0.0 for r in rows])
                     if self.has_att else None)
        tr = []
        for r in rows:
            pts = (r.get("truth") or {}).get("pts") or []
            tr.append(pts[0] if pts else (math.nan, math.nan))
        tr = np.array(tr, dtype=float)
        self.tx, self.ty = tr[:, 0], tr[:, 1]

    def _at(self, arr, t):
        return float(np.interp(t, self.t, arr))

    def pose(self, t: float) -> dict:
        return {"x": self._at(self.x, t), "y": self._at(self.y, t),
                "up": self._at(self.up, t), "yaw": self._at(self.psi, t),
                "pitch": (self._at(self.pitch, t) if self.has_att else None),
                "roll": (self._at(self.roll, t) if self.has_att else None)}

    def truth(self, t: float):
        n, e = self._at(self.tx, t), self._at(self.ty, t)
        return None if (math.isnan(n) or math.isnan(e)) else (n, e)


def row_pose(row: dict) -> dict:
    """The pose of the join row itself: what the controller had at the tick."""
    p, r = row.get("pitch_deg"), row.get("roll_deg")
    return {"x": row["x"], "y": row["y"], "up": row["up"], "yaw": row["psi"],
            "pitch": None if p is None else math.radians(p),
            "roll": None if r is None else math.radians(r)}


# ---------------------------------------------------------------------------
# labelling
# ---------------------------------------------------------------------------
def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def box_azimuth(box: dict, pose: dict, img_w, img_h, hfov=HFOV_DEG) -> float:
    ray = cam.ray_world(box["cx"], box["cy"], img_w, img_h, hfov, pose["yaw"],
                        pose.get("pitch") or 0.0, pose.get("roll") or 0.0)
    return math.atan2(ray[1], ray[0])


def label_box(box: dict, r: Optional[float], pose: dict, truth_ne,
              img_w, img_h, hfov=HFOV_DEG):
    """("on" | "off" | "amb", info) for one box. See the module docstring."""
    if truth_ne is None:
        return "amb", {"why": "no truth"}
    dn, de = truth_ne[0] - pose["x"], truth_ne[1] - pose["y"]
    truth_h = math.hypot(dn, de)
    truth_slant = math.hypot(truth_h, pose["up"] - CAR_CENTRE_UP_M)
    err = math.degrees(_wrap(box_azimuth(box, pose, img_w, img_h, hfov)
                             - math.atan2(de, dn)))
    tol = max(ON_BEARING_MIN_DEG,
              math.degrees(math.atan2(ON_BEARING_HALF_M, max(truth_h, 1e-6))))
    info = {"bearing_err_deg": err, "tol_deg": tol, "truth_h": truth_h,
            "truth_slant": truth_slant,
            "range_err": (None if r is None else r - truth_slant)}
    range_ok = (r is not None and abs(r - truth_slant)
                <= max(ON_RANGE_ABS_M, ON_RANGE_FRAC * truth_slant))
    if abs(err) <= tol and range_ok:
        return "on", info
    # Inside ~18 m the car's own angular extent (atan(2.5/range)) is wider
    # than 8 deg, so "8 deg off the car's centre" can still be ON the car:
    # carpolicy seq 1562-1565 are 120-170 px boxes 9-13 deg off-centre at
    # 8-10 m whose range agrees to 1 m. Within tol + OFF_MARGIN_DEG a box is
    # off-car only when its range disagrees too.
    if abs(err) > OFF_BEARING_DEG and (abs(err) > tol + OFF_MARGIN_DEG
                                       or (r is not None and not range_ok)):
        return "off", info
    return "amb", info


# ---------------------------------------------------------------------------
# samples
# ---------------------------------------------------------------------------
def load_flight(flight_dir: Path):
    rows = load_jsonl(Path(flight_dir) / "flight_log.jsonl")
    dets = load_jsonl(Path(flight_dir) / "detections.jsonl")
    return rows, dets


def flown_with_identity(d: Path) -> bool:
    """Did this flight run --identity? Its logged boxes were then chosen by
    the very rules this tool tunes and replay_pipeline.py evaluates, so it
    cannot be evidence for them (citylife_redcar_id1..4, 2026-09-30, were
    swept into both by the glob)."""
    try:
        m = json.loads((Path(d) / "metrics.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False
    ident = m.get("identity")
    return isinstance(ident, dict) and bool(ident) and ident.get("enabled", True) is not False


def flight_dirs(root: Path = OUT_ROOT) -> list:
    """The recorded red-car flights flown WITHOUT --identity."""
    return sorted(d for d in Path(root).glob(FLIGHT_GLOB)
                  if (d / "flight_log.jsonl").is_file()
                  and (d / "detections.jsonl").is_file()
                  and not flown_with_identity(d))


def capture_time(rec: dict, offset: float, cap_lag: float) -> float:
    return rec["t"] - offset - float(rec.get("infer_ms") or 0.0) / 1000.0 - cap_lag


def bearing_residuals(pairs, tl: Timeline, offset: float, cap_lag: float):
    out = []
    for rec, _row in pairs:
        t = capture_time(rec, offset, cap_lag)
        tr = tl.truth(t)
        if tr is None:
            continue
        p = tl.pose(t)
        b = rec["det"]
        W, H = b.get("img_w", 768), b.get("img_h", 432)
        out.append(math.degrees(_wrap(box_azimuth(b, p, W, H)
                                      - math.atan2(tr[1] - p["y"], tr[0] - p["x"]))))
    return out


def estimate_cap_lag(flights: list, grid=CAP_LAG_GRID) -> dict:
    """The image age at fetch that best aligns boxes with the car: the lag
    minimising the pooled median |bearing residual| of boxes within 6 deg.
    `flights` is [(pairs, timeline, offset)]."""
    score = {}
    for lag in grid:
        res = []
        for pairs, tl, off in flights:
            res += [abs(e) for e in bearing_residuals(pairs, tl, off, lag)
                    if abs(e) < 6.0]
        score[lag] = (statistics.median(res) if res else math.inf, len(res))
    best = min(score, key=lambda k: score[k][0])
    return {"lag_s": best,
            "median_abs_deg": {str(k): round(v[0], 3) for k, v in score.items()},
            "n_within_6deg": {str(k): v[1] for k, v in score.items()}}


def build_samples(flight_dir, street_dist=None, cap_lag: float = 0.2,
                  hfov: float = HFOV_DEG, rows=None, dets=None) -> dict:
    """Every joined box of one flight as a sample: label (at capture), and
    features under both the capture pose ("f") and the tick pose ("fj")."""
    flight_dir = Path(flight_dir)
    if rows is None or dets is None:
        rows, dets = load_flight(flight_dir)
    pairs = join_detections(rows, dets)
    offset = clock_offset(pairs)
    tl = Timeline(rows)
    samples = []
    for rec, row in pairs:
        b = rec["det"]
        W, H = b.get("img_w", 768), b.get("img_h", 432)
        t = capture_time(rec, offset, cap_lag)
        pose = tl.pose(t)
        truth = tl.truth(t)
        r, src = range_for(row)
        lab, info = label_box(b, r, pose, truth, W, H, hfov)
        f = identity.features_for(b, r, pose, W, H, hfov, street_dist,
                                  r_src=src or "none")
        fj = identity.features_for(b, r, row_pose(row), W, H, hfov,
                                   street_dist, r_src=src or "none")
        samples.append({"flight": flight_dir.name, "seq": rec["seq"],
                        "t_cap": t, "label": lab, "r": r, "r_src": src,
                        "info": info, "f": f, "fj": fj,
                        "blocked": bool(row.get("presence_blocked"))})
    return {"flight": flight_dir.name, "samples": samples, "offset": offset,
            "n_records": len(dets),
            "n_with_box": sum(1 for d in dets if d.get("det") is not None),
            "n_joined": len(pairs), "has_attitude": tl.has_att,
            "pairs": pairs, "timeline": tl}


def usable(s: dict) -> bool:
    """In the rates and the tuning: labelled on/off and not a box whose range
    the log lost (see the module docstring)."""
    return s["label"] in ("on", "off") and s["r_src"] != "unlogged"


# ---------------------------------------------------------------------------
# rates for one threshold set (reference implementation, via classify)
# ---------------------------------------------------------------------------
def tally(samples: list, th: dict, feat_key: str = "f") -> dict:
    c = {"n_on": 0, "n_off": 0, "on_notok": 0, "on_hard": 0,
         "off_notok": 0, "off_hard": 0}
    for s in samples:
        if not usable(s):
            continue
        tier, _ = identity.classify(s[feat_key], NOUN, th)
        k = s["label"]
        c[f"n_{k}"] += 1
        c[f"{k}_notok"] += tier != "ok"
        c[f"{k}_hard"] += tier == "hard"
    return with_rates(c)


def with_rates(c: dict) -> dict:
    c = dict(c)
    for k in ("on", "off"):
        n = c[f"n_{k}"]
        c[f"{k}_notok_rate"] = (c[f"{k}_notok"] / n) if n else None
        c[f"{k}_hard_rate"] = (c[f"{k}_hard"] / n) if n else None
    return c


# ---------------------------------------------------------------------------
# vectorised grid search
# ---------------------------------------------------------------------------
def _arrays(samples: list, feat_key: str, near_far_m: float, fixed: dict):
    S = [s for s in samples if usable(s)]
    nan = math.nan

    def col(key):
        return np.array([nan if s[feat_key].get(key) is None
                         else float(s[feat_key][key]) for s in S])
    rng_h = col("rng_h")
    a = {
        "flight": [s["flight"] for s in S],
        "on": np.array([s["label"] == "on" for s in S]),
        "near": ~(rng_h > near_far_m),               # None counts as near
        "r_none": np.array([s[feat_key].get("r") is None for s in S]),
        "fixed_hard": (col("width_m") > fixed["width_max"])
        | (col("frac_w") > fixed["frame_frac_max"]),
        "bottom_h": col("bottom_h"), "off_street": col("off_street_m"),
        "width": col("width_m"), "aspect": col("aspect"),
        "score": col("score"), "h_px": col("h_px"),
    }
    return a


def _combos(keys):
    return list(itertools.product(*[GRID[k] for k in keys]))


HARD_KEYS = ("bottom_max", "off_street_hard")
NEAR_KEYS = ("width_min", "aspect_min", "score_min")
FAR_KEYS = ("far_aspect_min", "far_h_px_max", "far_off_street")


def grid_counts(samples: list, feat_key: str = "f", fixed=None) -> dict:
    """Per-flight counts over the whole grid.

    Returns {flight: {"n_on", "n_off", "on_notok"[H,N,F], "off_notok"[H,N,F],
    "on_hard"[H], "off_hard"[H]}} with H/N/F indexing the hard / near-soft /
    far-soft combos (_combos(HARD_KEYS) etc.)."""
    fixed = {**identity.DEFAULTS, **(fixed or {})}
    a = _arrays(samples, feat_key, fixed["near_far_m"], fixed)
    hc, nc, fc = _combos(HARD_KEYS), _combos(NEAR_KEYS), _combos(FAR_KEYS)
    with np.errstate(invalid="ignore"):
        Hm = np.stack([a["fixed_hard"] | (a["bottom_h"] > b) | (a["off_street"] > o)
                       for b, o in hc]).astype(np.float64)
        Nm = np.stack([a["near"] & (a["r_none"] | (a["width"] < w)
                                    | (a["aspect"] < asp) | (a["score"] < s))
                       for w, asp, s in nc]).astype(np.float64)
        Fm = np.stack([~a["near"] & ((a["aspect"] < fa) | (a["h_px"] > fh)
                                     | (a["off_street"] > fo))
                       for fa, fh, fo in fc]).astype(np.float64)
    near = a["near"].astype(np.float64)
    far = 1.0 - near
    flights = sorted(set(a["flight"]))
    fl_arr = np.array(a["flight"])
    out = {}
    for fname in flights:
        entry = {}
        for lab, mask in (("on", a["on"]), ("off", ~a["on"])):
            m = (fl_arr == fname) & mask
            H, N, F = Hm[:, m], Nm[:, m], Fm[:, m]
            nr, fr = near[m], far[m]
            h_near = H @ nr                                   # (nH,)
            h_far = H @ fr
            near_or = h_near[:, None] + N.sum(1)[None, :] - H @ N.T
            far_or = h_far[:, None] + F.sum(1)[None, :] - H @ F.T
            entry[f"n_{lab}"] = int(m.sum())
            entry[f"{lab}_notok"] = near_or[:, :, None] + far_or[:, None, :]
            entry[f"{lab}_hard"] = H.sum(1)
        out[fname] = entry
    return out


def _sum(counts: dict, flights) -> dict:
    tot = None
    for f in flights:
        e = counts[f]
        if tot is None:
            tot = {k: (v.copy() if isinstance(v, np.ndarray) else v)
                   for k, v in e.items()}
        else:
            for k, v in e.items():
                tot[k] = tot[k] + v
    return tot


def choose(tot: dict, max_on_notok=MAX_ON_NOTOK, max_on_hard=MAX_ON_HARD):
    """Best (h, n, f) indices on summed counts, or None if nothing is
    feasible. Maximise off-car not-OK; ties: lower on-car not-OK, then higher
    off-car HARD, then the combo closest to identity.DEFAULTS."""
    n_on, n_off = max(tot["n_on"], 1), max(tot["n_off"], 1)
    on_rate = tot["on_notok"] / n_on
    on_hard = (tot["on_hard"] / n_on)[:, None, None]
    feas = (on_rate <= max_on_notok + 1e-12) & (on_hard <= max_on_hard + 1e-12)
    if not feas.any():
        return None
    off_rate = tot["off_notok"] / n_off
    off_hard = (tot["off_hard"] / n_off)[:, None, None]
    shape = on_rate.shape
    dist = _default_distance(shape)
    key = np.where(feas, off_rate, -1.0)
    best = key.max()
    cand = feas & (key >= best - 1e-12)
    # lexicographic tie-break
    sec = np.where(cand, -on_rate, -np.inf)
    cand &= sec >= sec.max() - 1e-12
    ter = np.where(cand, np.broadcast_to(off_hard, shape), -np.inf)
    cand &= ter >= ter.max() - 1e-12
    qua = np.where(cand, -dist, -np.inf)
    idx = np.unravel_index(int(np.argmax(qua)), shape)
    return tuple(int(i) for i in idx)


def _default_distance(shape):
    """How far each grid combo is from identity.DEFAULTS, in grid steps."""
    def d(keys):
        out = []
        for c in _combos(keys):
            s = 0.0
            for k, v in zip(keys, c):
                vals = GRID[k]
                step = (vals[-1] - vals[0]) / max(len(vals) - 1, 1) or 1.0
                s += abs(v - identity.DEFAULTS[k]) / step
            out.append(s)
        return np.array(out)
    dh, dn, df = d(HARD_KEYS), d(NEAR_KEYS), d(FAR_KEYS)
    return dh[:, None, None] + dn[None, :, None] + df[None, None, :]


def combo_thresholds(idx) -> dict:
    h, n, f = idx
    th = {}
    for keys, i in ((HARD_KEYS, h), (NEAR_KEYS, n), (FAR_KEYS, f)):
        th.update(dict(zip(keys, _combos(keys)[i])))
    return th


def counts_at(counts_or_tot: dict, idx) -> dict:
    h, n, f = idx
    e = counts_or_tot
    return with_rates({"n_on": e["n_on"], "n_off": e["n_off"],
                       "on_notok": int(e["on_notok"][h, n, f]),
                       "on_hard": int(e["on_hard"][h]),
                       "off_notok": int(e["off_notok"][h, n, f]),
                       "off_hard": int(e["off_hard"][h])})


def leave_one_out(counts: dict) -> dict:
    flights = sorted(counts)
    per, held = {}, None
    for f in flights:
        train = _sum(counts, [g for g in flights if g != f])
        idx = choose(train)
        if idx is None:
            per[f] = {"thresholds": None, "held_out": None}
            continue
        c = counts_at(counts[f], idx)
        per[f] = {"thresholds": combo_thresholds(idx), "held_out": c,
                  "train": counts_at(train, idx)}
        if held is None:
            held = {k: 0 for k in ("n_on", "n_off", "on_notok", "on_hard",
                                   "off_notok", "off_hard")}
        for k in held:
            held[k] += c[k]
    return {"per_flight": per, "pooled_held_out": with_rates(held) if held else None}


# ---------------------------------------------------------------------------
# report pieces
# ---------------------------------------------------------------------------
def sliver_outcomes(samples: list, th: dict, feat_key="f") -> dict:
    out = {"near": {"ok": 0, "soft": 0, "hard": 0},
           "far": {"ok": 0, "soft": 0, "hard": 0}, "hard_reasons": {}}
    for s in samples:
        if not usable(s) or s["label"] != "on" or s[feat_key]["w_px"] >= SLIVER_PX:
            continue
        tier, why = identity.classify(s[feat_key], NOUN, th)
        rng_h = s[feat_key].get("rng_h")
        band = "far" if rng_h is not None and rng_h > th["near_far_m"] else "near"
        out[band][tier] += 1
        if tier == "hard":
            k = re.sub(r"[0-9.]+", "#", why[0])
            out["hard_reasons"][k] = out["hard_reasons"].get(k, 0) + 1
    return out


def range_bias(samples: list, feat_key="f") -> list:
    """rng minus the truth HORIZONTAL range (and minus the truth slant range to
    the car centre), by 10 m band of truth horizontal range, over boxes whose
    bearing agrees with the car (|err| <= tol) and that have a range."""
    bands = {}
    for s in samples:
        inf = s["info"]
        if s["r"] is None or "bearing_err_deg" not in inf:
            continue
        if abs(inf["bearing_err_deg"]) > inf["tol_deg"]:
            continue
        b = int(inf["truth_h"] // 10) * 10
        bands.setdefault(min(b, 100), []).append(
            (s["r"] - inf["truth_h"], s["r"] - inf["truth_slant"],
             s["label"] == "on"))
    out = []
    for b in sorted(bands):
        v = bands[b]
        dh = np.array([x[0] for x in v])
        ds = np.array([x[1] for x in v])
        out.append({"band_m": f"{b}-{b + 10}" if b < 100 else "100+",
                    "n": len(v), "n_on": int(sum(x[2] for x in v)),
                    "mean_minus_h": round(float(dh.mean()), 2),
                    "median_minus_h": round(float(np.median(dh)), 2),
                    "p10_minus_h": round(float(np.percentile(dh, 10)), 2),
                    "p90_minus_h": round(float(np.percentile(dh, 90)), 2),
                    "median_minus_slant": round(float(np.median(ds)), 2)})
    return out


def on_car_hard_cases(samples: list, th: dict, feat_key="f") -> list:
    """Every true car the rules HARD-reject, with the geometry behind it."""
    out = []
    for s in samples:
        if not usable(s) or s["label"] != "on":
            continue
        tier, why = identity.classify(s[feat_key], NOUN, th)
        if tier != "hard":
            continue
        f = s[feat_key]
        out.append({"flight": s["flight"], "seq": s["seq"], "why": why[0],
                    "rng_h": round(f["rng_h"], 1),
                    "dep_bottom_deg": (None if f.get("dep_bottom_deg") is None
                                       else round(f["dep_bottom_deg"], 1)),
                    "w_px": f["w_px"], "h_px": f["h_px"]})
    return out


def passing_off_car(samples: list, th: dict, feat_key="f") -> dict:
    """What the off-car boxes that pass as OK look like. Car-sized (1.2-5 m)
    on the road (<= 1.5 m off the street) means another vehicle, which no
    physical rule can tell from the red car: that is the tracker's job."""
    ok = []
    for s in samples:
        if usable(s) and s["label"] == "off":
            if identity.classify(s[feat_key], NOUN, th)[0] == "ok":
                ok.append(s[feat_key])
    carlike = [f for f in ok if f.get("width_m") is not None
               and 1.2 <= f["width_m"] <= 5.0
               and (f.get("off_street_m") or 0.0) <= 1.5]
    med = (lambda k: None if not ok else round(float(np.median(
        [f[k] for f in ok if f.get(k) is not None])), 2))
    return {"n_ok": len(ok), "n_carlike_on_road": len(carlike),
            "median_width_m": med("width_m"), "median_aspect": med("aspect"),
            "median_off_street_m": med("off_street_m"),
            "median_rng_h": med("rng_h")}


def grid_edges(th: dict) -> list:
    """Tuned keys that landed on the first or last grid value: the budget,
    not the grid, may be what stopped them."""
    return [k for k, vals in GRID.items()
            if th[k] in (vals[0], vals[-1]) and len(vals) > 1]


def _pct(x):
    return "-" if x is None else f"{100.0 * x:.1f} %"


def _row_md(name, c):
    return (f"| {name} | {c['n_off']} | {_pct(c['off_notok_rate'])} | "
            f"{_pct(c['off_hard_rate'])} | {c['n_on']} | "
            f"{_pct(c['on_notok_rate'])} | {_pct(c['on_hard_rate'])} |")


def write_markdown(rep: dict, path: Path) -> None:
    L = []
    L.append("# Target identity replay: \"follow a red car\"")
    L.append("")
    L.append(f"Generated {rep['generated']} by `tools/replay_identity.py` "
             f"({rep['runtime_s']:.1f} s). Features from the "
             f"**{rep['pose']}** pose; cap_lag {rep['cap_lag']['lag_s']} s. "
             f"Thresholds hash `{rep['thresholds_hash']}`.")
    L.append("")
    L.append("Labels are geometric (module docstring of the tool). Rates are "
             "over labelled boxes with a known range state. Not-OK means SOFT "
             "or HARD. The constraint is on-car not-OK <= 15 % and on-car "
             "HARD <= 3 %.")
    L.append("")
    L.append("## Chosen thresholds (tuned on all flights)")
    L.append("")
    L.append("| key | value |")
    L.append("|---|---|")
    for k in identity.RULE_KEYS:
        L.append(f"| {k} | {rep['thresholds'][k]:g} |")
    L.append("")
    if rep["grid_edges"]:
        L.append(f"On a grid edge: {', '.join(rep['grid_edges'])}. The grid is "
                 "the brief's; the on-car not-OK budget is what binds.")
        L.append("")
    hdr = ("| flight | off-car n | off not-OK | off HARD | on-car n | "
           "on not-OK | on HARD |")
    sep = "|---|---|---|---|---|---|---|"
    L.append("## In-sample (chosen thresholds, every flight)")
    L.append("")
    L.append(hdr)
    L.append(sep)
    for f, c in rep["per_flight"].items():
        L.append(_row_md(f, c))
    L.append(_row_md("**pooled**", rep["pooled"]))
    L.append("")
    L.append("## Leave-one-flight-out (thresholds tuned without the flight)")
    L.append("")
    L.append(hdr)
    L.append(sep)
    for f, e in rep["loo"]["per_flight"].items():
        if e["held_out"] is not None:
            L.append(_row_md(f, e["held_out"]))
    if rep["loo"]["pooled_held_out"]:
        L.append(_row_md("**pooled held-out**", rep["loo"]["pooled_held_out"]))
    L.append("")
    L.append("## Baselines on the same boxes")
    L.append("")
    L.append(hdr)
    L.append(sep)
    for name, c in rep["baselines"].items():
        L.append(_row_md(name, c))
    L.append("")
    sl = rep["slivers"]
    L.append(f"## On-car slivers (w < {SLIVER_PX:g} px)")
    L.append("")
    L.append(f"near (<= 60 m): ok {sl['near']['ok']}, soft {sl['near']['soft']}, "
             f"hard {sl['near']['hard']}; far: ok {sl['far']['ok']}, "
             f"soft {sl['far']['soft']}, hard {sl['far']['hard']}. "
             f"HARD reasons: {sl['hard_reasons'] or 'none'}")
    L.append("")
    po = rep["off_car_passing"]
    L.append("## Off-car boxes that pass as OK")
    L.append("")
    L.append(f"{po['n_ok']} pass; {po['n_carlike_on_road']} of them are "
             "car-sized (1.2-5 m) and within 1.5 m of the street, i.e. other "
             "vehicles, which physics cannot tell from the red car. Medians: "
             f"width {po['median_width_m']} m, aspect {po['median_aspect']}, "
             f"{po['median_off_street_m']} m off the street, "
             f"{po['median_rng_h']} m away.")
    L.append("")
    L.append("## True cars the rules HARD-reject")
    L.append("")
    L.append("| flight | seq | rule | rng_h m | bottom ray dep deg | w px | h px |")
    L.append("|---|---|---|---|---|---|---|")
    for c in rep["on_car_hard"]:
        L.append(f"| {c['flight']} | {c['seq']} | {c['why']} | {c['rng_h']} | "
                 f"{c['dep_bottom_deg']} | {c['w_px']:g} | {c['h_px']:g} |")
    L.append("")
    L.append("## Range bias of boxes on the car's bearing")
    L.append("")
    L.append("rng = the depth range the flight used (or the gate's rounded "
             "range). h = truth horizontal range; slant = to the car centre "
             "0.75 m up.")
    L.append("")
    L.append("| truth h band | n | on-car | mean rng-h | median rng-h | "
             "p10 | p90 | median rng-slant |")
    L.append("|---|---|---|---|---|---|---|---|")
    for b in rep["range_bias"]:
        L.append(f"| {b['band_m']} m | {b['n']} | {b['n_on']} | "
                 f"{b['mean_minus_h']:+.2f} | {b['median_minus_h']:+.2f} | "
                 f"{b['p10_minus_h']:+.2f} | {b['p90_minus_h']:+.2f} | "
                 f"{b['median_minus_slant']:+.2f} |")
    L.append("")
    L.append("## Data")
    L.append("")
    L.append("on / off / amb count every joined box. The rate tables leave out "
             "the 'unlogged range' boxes: blocked by the presence gate in a "
             "flight that did not log gate_why, so their range is lost.")
    L.append("")
    L.append("| flight | records | with box | joined | on | off | amb | "
             "unlogged range | attitude logged |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for f, d in rep["data"].items():
        L.append(f"| {f} | {d['n_records']} | {d['n_with_box']} | "
                 f"{d['n_joined']} | {d['on']} | {d['off']} | {d['amb']} | "
                 f"{d['unlogged']} | {'yes' if d['has_attitude'] else 'no'} |")
    L.append("")
    if rep.get("sensitivity"):
        L.append("## Sensitivity: the same thresholds on the TICK pose")
        L.append("")
        L.append(hdr)
        L.append(sep)
        L.append(_row_md("pooled, tick pose", rep["sensitivity"]["tick_pose"]))
        L.append("")
    Path(path).write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--root", default=str(OUT_ROOT))
    ap.add_argument("--out", default=str(REPORT_DIR))
    ap.add_argument("--street", default=str(identity.DEFAULT_STREET))
    ap.add_argument("--thresholds-out", default=str(identity.DEFAULT_THRESHOLDS))
    ap.add_argument("--no-write-thresholds", action="store_true")
    ap.add_argument("--cap-lag", default="auto",
                    help="image age at fetch, s, or 'auto' (estimated)")
    ap.add_argument("--pose", choices=("capture", "join"), default="capture",
                    help="pose the FEATURES use; labels always use capture")
    args = ap.parse_args(argv)
    t0 = time.time()

    dirs = flight_dirs(Path(args.root))
    if not dirs:
        print(f"no flights under {args.root}/{FLIGHT_GLOB}")
        return 1
    street = identity.StreetDistance.load(args.street)
    raw = {d.name: load_flight(d) for d in dirs}

    # capture lag
    if args.cap_lag == "auto":
        tmp = []
        for d in dirs:
            rows, dets = raw[d.name]
            pairs = join_detections(rows, dets)
            tmp.append((pairs, Timeline(rows), clock_offset(pairs)))
        lag = estimate_cap_lag(tmp)
    else:
        lag = {"lag_s": float(args.cap_lag)}
    print(f"[replay] {len(dirs)} flights; cap_lag {lag['lag_s']} s")

    flights = {}
    for d in dirs:
        rows, dets = raw[d.name]
        flights[d.name] = build_samples(d, street, lag["lag_s"], rows=rows,
                                        dets=dets)
    samples = [s for fl in flights.values() for s in fl["samples"]]
    fk = "f" if args.pose == "capture" else "fj"

    counts = grid_counts(samples, fk)
    if not counts:
        print("[replay] no labelled box with a known range state")
        return 2
    tot = _sum(counts, sorted(counts))
    idx = choose(tot)
    if idx is None:
        print("[replay] no threshold set meets the constraints")
        return 2
    th = {**identity.DEFAULTS, **combo_thresholds(idx)}
    loo = leave_one_out(counts)

    per_flight = {f: tally(fl["samples"], th, fk) for f, fl in flights.items()}
    pooled = tally(samples, th, fk)
    # the vectorised count and the classify() path must agree exactly
    vec = counts_at(tot, idx)
    for k in ("on_notok", "on_hard", "off_notok", "off_hard", "n_on", "n_off"):
        assert vec[k] == pooled[k], (k, vec[k], pooled[k])

    baselines = {
        "defaults (brief)": tally(samples, dict(identity.DEFAULTS), fk),
        "HARD rules only": tally(samples, {**th, "width_min": -1, "aspect_min": -1,
                                           "score_min": -1, "far_aspect_min": -1,
                                           "far_h_px_max": 1e9,
                                           "far_off_street": 1e9}, fk),
    }
    data = {}
    for f, fl in flights.items():
        labs = [s["label"] for s in fl["samples"]]
        data[f] = {"n_records": fl["n_records"], "n_with_box": fl["n_with_box"],
                   "n_joined": fl["n_joined"], "on": labs.count("on"),
                   "off": labs.count("off"), "amb": labs.count("amb"),
                   "unlogged": sum(s["r_src"] == "unlogged" for s in fl["samples"]),
                   "has_attitude": fl["has_attitude"],
                   "clock_offset_s": fl["offset"]}

    rep = {
        "generated": _dt.datetime.now().isoformat(timespec="seconds"),
        "pose": args.pose, "cap_lag": lag,
        "labels": {"on_bearing": f"max({ON_BEARING_MIN_DEG} deg, atan("
                                 f"{ON_BEARING_HALF_M}/range))",
                   "on_range": f"max({ON_RANGE_ABS_M} m, {ON_RANGE_FRAC:.0%})",
                   "off_bearing_deg": OFF_BEARING_DEG},
        "constraints": {"on_notok_max": MAX_ON_NOTOK, "on_hard_max": MAX_ON_HARD},
        "grid": GRID,
        "thresholds": {k: th[k] for k in identity.RULE_KEYS},
        "thresholds_hash": identity.thresholds_hash(th),
        "per_flight": per_flight, "pooled": pooled, "loo": loo,
        "baselines": baselines,
        "slivers": sliver_outcomes(samples, th, fk),
        "on_car_hard": on_car_hard_cases(samples, th, fk),
        "off_car_passing": passing_off_car(samples, th, fk),
        "grid_edges": grid_edges(th),
        "range_bias": range_bias(samples, fk),
        "data": data,
    }
    if args.pose == "capture":
        rep["sensitivity"] = {"tick_pose": tally(samples, th, "fj")}
    rep["runtime_s"] = time.time() - t0

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with (out / "report.json").open("w", encoding="utf-8", newline="\n") as fh:
        json.dump(rep, fh, indent=1, default=float)
        fh.write("\n")
    write_markdown(rep, out / "report.md")

    held = loo["pooled_held_out"] or with_rates(
        {k: 0 for k in ("n_on", "n_off", "on_notok", "on_hard",
                        "off_notok", "off_hard")})
    if not args.no_write_thresholds:
        doc = {k: th[k] for k in identity.RULE_KEYS}
        doc["note"] = (
            "Written by tools/replay_identity.py. Grid-searched to maximise the "
            "off-car not-OK rate subject to on-car not-OK <= 15 % and on-car "
            "HARD <= 3 %, on geometric labels from the capture-time pose. "
            f"Pooled in-sample: off not-OK {_pct(pooled['off_notok_rate'])}, "
            f"off HARD {_pct(pooled['off_hard_rate'])}, on not-OK "
            f"{_pct(pooled['on_notok_rate'])}, on HARD "
            f"{_pct(pooled['on_hard_rate'])}. Leave-one-flight-out pooled: "
            f"off not-OK {_pct(held['off_notok_rate'])}, "
            f"on not-OK {_pct(held['on_notok_rate'])}, "
            f"on HARD {_pct(held['on_hard_rate'])}. "
            "See demo/out/identity_replay/report.md.")
        doc["tuned_on"] = sorted(flights)
        doc["hash"] = identity.thresholds_hash(th)
        with Path(args.thresholds_out).open("w", encoding="utf-8",
                                            newline="\n") as fh:
            json.dump(doc, fh, indent=2)
            fh.write("\n")

    def line(name, c):
        return (f"  {name:26s} off n={c['n_off']:5d} notOK {_pct(c['off_notok_rate']):>8s}"
                f" HARD {_pct(c['off_hard_rate']):>8s} | on n={c['n_on']:4d} notOK "
                f"{_pct(c['on_notok_rate']):>8s} HARD {_pct(c['on_hard_rate']):>8s}")
    print(f"[replay] thresholds {rep['thresholds']} hash {rep['thresholds_hash']}")
    for f, c in per_flight.items():
        print(line(f, c))
    print(line("POOLED (in-sample)", pooled))
    for f, e in loo["per_flight"].items():
        if e["held_out"]:
            print(line(f"LOO {f}", e["held_out"]))
    print(line("POOLED HELD-OUT (LOO)", held))
    for name, c in baselines.items():
        print(line(name, c))
    if "sensitivity" in rep:
        print(line("tick pose, same th", rep["sensitivity"]["tick_pose"]))
    print(f"[replay] slivers {rep['slivers']}")
    print(f"[replay] wrote {out / 'report.json'} and report.md "
          f"in {rep['runtime_s']:.1f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
