"""
The follow expert's AerialVLA recording (demo/vla_record.py, follow_vlm
--record-vla) and the step-label builder (tools/build_follow_dataset.py).

Run either way:
    pytest tests/test_follow_dataset.py -v
    python tests/test_follow_dataset.py

WHAT IS AT STAKE (wave FT-D, 2026-10-23)

The hand-written follow (demo/follow_vlm.py) chases the red car within 30 m
on 0.963 of its ticks; AerialVLA's follows on 0.17-0.31 of theirs (#9, #10,
CHANGELOG 2026-10-23). The plan is to fine-tune AerialVLA on the expert's
own flights, and the July campaign showed how that goes wrong: run1
collapsed on straight-heavy data, run2 learned to turn and overshot because
the expert decided at 10 Hz and the model at 0.8 Hz, and the old trainer
(training/finetune_aerialvla.py) wrote its labels in a format the model was
never trained to emit (unpadded bins, round() instead of truncation, a
single image, its own prompt). So
every piece of a training sample is pinned here against the ONE definition
the server flies with (guardrail/aerialvla_codec.py) and the ONE rule the
flight builds its phrase with (demo/vla_mission.DetectorHint):

  1. THE LABEL is the expert's motion over the next horizon, in upstream's
     step units: dpsi = wrap(yaw(t+h) - yaw(t)) clipped to +-1.1 rad, fwd =
     the displacement projected on the NEW heading clipped to 0..5 m, down =
     up(t) - up(t+h) clipped to -5..5 m - checked on synthetic trajectories
     (straight, turn, climb, the wrap at +-pi, every clip).
  2. THE TEXT is upstream's: truncation into 99 bins, "FF DD YY" zero-padded,
     and it reads back through the codec's strict reader (the server's) as
     the same bins.
  3. THE PHRASE is DetectorHint's for the expert's box: fresh (age <= 1 s at
     the capture), its bearing rotated by the turn since the box's frame.
  4. THE SPLIT is by run: a hold-out run never reaches train.
  5. THE RECORDER is off unless asked for, never slows the loop (a writer
     thread; a tick costs a deque append), and writes what the builder
     reads: the front+down pair paired by sim stamp, the capture pose, the
     expert's box at that moment, the expert's command, the car's truth.
  6. DART (2026-10-23, after ft_expert_01's 507 samples all read "straight
     ahead "): --record-perturb knocks the executed command off on a seeded
     schedule, off by default and the flight unchanged when off; a label
     whose horizon overlaps a perturbation is dropped, the samples after it
     kept; each run reports its phrases and turn histogram; --mirror
     mirrors the train split against the codec's phrase vocabulary.

No simulator, GPU or model. The mosaic test needs Pillow and SKIPs without
it (GUARDRAIL_REQUIRE_PILLOW=1 turns that into a failure, as for the codec).
"""
from __future__ import annotations

import ast
import base64
import hashlib
import io
import json
import math
import os
import re
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import aerialvla_codec as C                         # noqa: E402

SKIP = "SKIP"
CFG = ROOT / "demo" / "pas_config"
PS1 = ROOT / "scripts" / "run_citylife_follow.ps1"
FOLLOW = ROOT / "demo" / "follow_vlm.py"
T0 = 1_000_000.0             # a wall-clock origin far from 0, as a flight's


def _B():
    import build_follow_dataset as B
    return B


def _R():
    import vla_record as R
    return R


def _pil():
    """Pillow's Image module, or None (the test SKIPs). With
    GUARDRAIL_REQUIRE_PILLOW=1 - the CI step that installs Pillow for the
    AerialVLA image tests - a missing Pillow is a broken install."""
    try:
        from PIL import Image
        return Image
    except ImportError:
        if os.environ.get("GUARDRAIL_REQUIRE_PILLOW") == "1":
            raise AssertionError("GUARDRAIL_REQUIRE_PILLOW=1 and Pillow does not import")
        return None


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# --------------------------------------------------------------------------- #
# Synthetic recordings, in the recorder's own file format
# --------------------------------------------------------------------------- #
def _det_at_bearing(deg: float, w_frac: float = 0.12, W: int = 768, H: int = 432):
    """A Grounder box (cx, cy, w, h, score, W, H, colour) whose pinhole bearing
    on the 90 deg FrontCamera is `deg` (f = W/2 / tan 45 = W/2)."""
    cx = W / 2 + (W / 2) * math.tan(math.radians(deg))
    return [cx, 0.6 * H, w_frac * W, 0.5 * w_frac * W, 0.3, W, H, 0.5]


def _write_run(root: Path, tag: str, motion, *, seconds: float = 8.0, dt: float = 0.1,
               box=None, object_text: str = "a red car", jpeg=None, mode=None,
               record_json: bool = True, perturb=None) -> Path:
    """A recording as ExpertVLARecorder writes it: track.jsonl (one line per
    loop tick), captures.jsonl (one per front+down pair, on the same 10 Hz
    grid here so the label arithmetic is exact), frames/<sha256>.jpg and
    record.json. `motion(tau) -> (x, y, up, yaw)`; `box(k, t, yaw_at) ->
    dict | None` (default: a fresh box straight ahead, 0.3 s old); `jpeg(k)
    -> (front_bytes, down_bytes)` (default: distinct fake bytes);
    `perturb(k) -> dict | None`: the tick's DART perturbation record, put
    on the track line and the capture as the recorder does (default: no
    such key at all, as an unperturbed recording)."""
    d = root / tag / "vla_record"
    (d / "frames").mkdir(parents=True, exist_ok=True)
    n = int(round(seconds / dt))

    def yaw_at(t):
        return motion(t - T0)[3]

    track, caps = [], []
    for k in range(n + 1):
        t = T0 + k * dt
        x, y, up, yaw = motion(k * dt)
        m = mode(k) if mode else "track"
        track.append({"t": t, "x": x, "y": y, "up": up, "yaw": yaw, "mode": m,
                      "author": "follow_vlm", "cmd": {"vx": 0.0, "vy": 0.0, "vz_up": 0.0,
                                                      "yaw_rate": 0.0},
                      "asked": None, "touched": False, "truth": [x + 15.0, y],
                      "object": object_text})
        if box is None:
            b = {"det": _det_at_bearing(0.0), "t_box": t - 0.3, "yaw_box": yaw_at(t - 0.3),
                 "t_published": t - 0.05, "seq": k, "tier": "ok"}
        else:
            b = box(k, t, yaw_at)
        fb, db = jpeg(k) if jpeg else (f"front-{tag}-{k}".encode(), f"down-{tag}-{k}".encode())
        hf, hd = hashlib.sha256(fb).hexdigest(), hashlib.sha256(db).hexdigest()
        (d / "frames" / f"{hf}.jpg").write_bytes(fb)
        (d / "frames" / f"{hd}.jpg").write_bytes(db)
        caps.append({"i": k, "t_capture": t, "t_mission": t - T0,
                     "pose": {"x": x, "y": y, "up": up, "yaw_rad": yaw},
                     "front_jpg_sha256": hf, "down_jpg_sha256": hd,
                     "front_wh": [256, 256], "down_wh": [256, 256], "pair_dt_s": 0.0,
                     "box": b, "tick": {"tick": k, "t": t, "mode": m}, "truth": [x + 15.0, y],
                     "object": object_text})
        if perturb is not None:
            track[-1]["perturb"] = caps[-1]["perturb"] = perturb(k)
    with (d / "track.jsonl").open("w", encoding="utf-8", newline="\n") as fh:
        for r in track:
            fh.write(json.dumps(r) + "\n")
    with (d / "captures.jsonl").open("w", encoding="utf-8", newline="\n") as fh:
        for r in caps:
            fh.write(json.dumps(r) + "\n")
    if record_json:
        (d / "record.json").write_text(json.dumps(
            {"format": "vla_record/1", "tag": tag, "object": object_text, "t0": T0}),
            encoding="utf-8")
    return d


def _straight(speed=4.0, up=8.0, yaw=0.0):
    return lambda tau: (speed * tau * math.cos(yaw), speed * tau * math.sin(yaw), up, yaw)


def _turning(speed=4.0, rate=0.3, up=8.0, yaw0=0.0):
    def m(tau):
        if abs(rate) < 1e-12:
            return speed * tau, 0.0, up, yaw0
        r = speed / rate
        x = r * (math.sin(yaw0 + rate * tau) - math.sin(yaw0))
        y = r * (math.cos(yaw0) - math.cos(yaw0 + rate * tau))
        return x, y, up, _wrap(yaw0 + rate * tau)
    return m


def _rows(motion, seconds=6.0, dt=0.1):
    out = []
    for k in range(int(round(seconds / dt)) + 1):
        x, y, up, yaw = motion(k * dt)
        out.append({"t": T0 + k * dt, "x": x, "y": y, "up": up, "yaw": yaw})
    return out


# =========================================================================== #
# 1. The label: the expert's motion over the horizon, in upstream's units
# =========================================================================== #
def test_a_straight_flight_is_a_forward_step_and_the_upstream_text():
    B = _B()
    tr = B.Track(_rows(_straight(speed=4.0)))
    lab = B.label_at(tr, T0 + 2.0, 1.0)
    assert lab["reason"] is None, lab
    assert math.isclose(lab["raw"]["fwd_m"], 4.0, abs_tol=1e-9), lab
    assert abs(lab["raw"]["down_m"]) < 1e-9 and abs(lab["raw"]["dpsi_rad"]) < 1e-9, lab
    # 4.0 m: 4/5*98 = 78.4 -> 78 (truncation); 0 m and 0 rad: bin 49.
    assert lab["bins"] == (78, 49, 49), lab
    assert C.action_text(lab["bins"]) == "78 49 49"


def test_a_turn_is_the_heading_change_and_the_chord_on_the_new_heading():
    B = _B()
    rate, v = 0.3, 4.0
    tr = B.Track(_rows(_turning(speed=v, rate=rate)))
    lab = B.label_at(tr, T0 + 2.0, 1.0)
    assert lab["reason"] is None, lab
    assert math.isclose(lab["raw"]["dpsi_rad"], rate * 1.0, abs_tol=1e-9), lab
    r = v / rate
    dx = r * (math.sin(0.9) - math.sin(0.6))
    dy = r * (math.cos(0.6) - math.cos(0.9))
    want = dx * math.cos(0.9) + dy * math.sin(0.9)       # projected on yaw(t+h)
    assert math.isclose(lab["raw"]["fwd_m"], want, abs_tol=1e-6), (lab, want)
    assert want < v, "the chord on the NEW heading is shorter than the arc"
    # Left turns are negative (yaw clockwise from North, as the codec's dpsi).
    lab_l = B.label_at(B.Track(_rows(_turning(speed=v, rate=-rate))), T0 + 2.0, 1.0)
    assert math.isclose(lab_l["raw"]["dpsi_rad"], -rate, abs_tol=1e-9), lab_l
    assert lab["bins"][2] > 49 > lab_l["bins"][2], (lab["bins"], lab_l["bins"])


def test_a_climb_is_a_negative_down_and_a_descent_a_positive_one():
    B = _B()
    climb = lambda tau: (4.0 * tau, 0.0, 8.0 + 1.0 * tau, 0.0)        # noqa: E731
    lab = B.label_at(B.Track(_rows(climb)), T0 + 2.0, 1.0)
    assert math.isclose(lab["raw"]["down_m"], -1.0, abs_tol=1e-9), lab
    assert lab["bins"][1] == 39, lab          # (-1+5)/10*98 = 39.2 -> 39
    desc = lambda tau: (4.0 * tau, 0.0, 12.0 - 0.5 * tau, 0.0)        # noqa: E731
    lab = B.label_at(B.Track(_rows(desc)), T0 + 2.0, 1.0)
    assert math.isclose(lab["raw"]["down_m"], 0.5, abs_tol=1e-9), lab
    assert lab["bins"][1] == 53, lab          # (0.5+5)/10*98 = 53.9 -> 53


def test_the_heading_wraps_at_plus_minus_pi_the_short_way():
    B = _B()
    # yaw 3.0 rad turning +0.2 rad/s crosses +pi into -pi at tau ~ 0.71 s.
    m = lambda tau: (4.0 * tau, 0.0, 8.0, _wrap(3.0 + 0.2 * tau))     # noqa: E731
    tr = B.Track(_rows(m))
    lab = B.label_at(tr, T0 + 0.5, 1.0)
    assert math.isclose(lab["raw"]["dpsi_rad"], 0.2, abs_tol=1e-9), lab
    # Interpolated BETWEEN samples on either side of the seam (0.7: 3.14,
    # 0.8: -3.123): the short way round, not through zero.
    p = tr.at(T0 + 0.75)
    assert math.isclose(_wrap(p["yaw"] - 3.15), 0.0, abs_tol=1e-9), p
    lab = B.label_at(tr, T0 + 0.75, 1.0)
    assert math.isclose(lab["raw"]["dpsi_rad"], 0.2, abs_tol=1e-9), lab
    # And the other way: -3.0 turning -0.2 rad/s.
    m2 = lambda tau: (0.0, 0.0, 8.0, _wrap(-3.0 - 0.2 * tau))         # noqa: E731
    lab = B.label_at(B.Track(_rows(m2)), T0 + 0.5, 1.0)
    assert math.isclose(lab["raw"]["dpsi_rad"], -0.2, abs_tol=1e-9), lab


def test_every_axis_is_clipped_to_upstreams_range_and_the_clip_is_counted():
    B = _B()
    cases = [
        # (motion, axis, raw expected, clipped expected, bin)
        (_straight(speed=-2.0), "fwd_m", -2.0, 0.0, 0),        # backwards
        (_straight(speed=7.0), "fwd_m", 7.0, 5.0, 98),          # faster than 5 m/step
        (lambda tau: (0, 0, 8 + 6 * tau, 0), "down_m", -6.0, -5.0, 0),
        (lambda tau: (0, 0, 20 - 6 * tau, 0), "down_m", 6.0, 5.0, 98),
        (lambda tau: (0, 0, 8, _wrap(2.0 * tau)), "dpsi_rad", 2.0, 1.1, 98),
        (lambda tau: (0, 0, 8, _wrap(-2.0 * tau)), "dpsi_rad", -2.0, -1.1, 0),
    ]
    for motion, axis, raw, clipped, b in cases:
        lab = B.label_at(B.Track(_rows(motion)), T0 + 1.0, 1.0)
        assert lab["reason"] is None, (axis, lab)
        assert math.isclose(lab["raw"][axis], raw, abs_tol=1e-9), (axis, lab)
        assert math.isclose(lab["label"][axis], clipped, abs_tol=1e-12), (axis, lab)
        assert lab["clipped"][axis] == ("low" if raw < clipped else "high"), \
            (axis, lab["clipped"])
        assert lab["bins"][C.AXES.index(axis)] == b, (axis, lab)
    lab = B.label_at(B.Track(_rows(_straight())), T0 + 1.0, 1.0)
    assert lab["clipped"] == {"fwd_m": None, "down_m": None, "dpsi_rad": None}, lab


def test_a_horizon_past_the_track_or_across_a_gap_is_no_label():
    B = _B()
    rows = _rows(_straight(), seconds=4.0)
    tr = B.Track(rows)
    assert B.label_at(tr, T0 + 3.5, 1.0)["reason"] == "no_future_pose"
    assert B.label_at(tr, T0 - 0.5, 1.0)["reason"] == "no_pose"
    gap = [r for r in rows if not (T0 + 1.4 < r["t"] < T0 + 2.6)]    # a 1.2 s stall
    tr = B.Track(gap, max_gap_s=0.5)
    assert B.label_at(tr, T0 + 1.0, 1.0)["reason"] == "track_gap"
    assert B.label_at(tr, T0 + 0.2, 1.0)["reason"] is None


# =========================================================================== #
# 2. The text: upstream's truncation and padding, read back by the server's reader
# =========================================================================== #
def test_the_label_text_is_truncated_zero_padded_and_reads_back_through_the_strict_reader():
    assert C.action_text((0, 5, 9)) == "00 05 09"
    assert C.action_text((98, 49, 0), land=True) == "98 49 00 LAND"
    for bad in ((99, 49, 49), (-1, 49, 49), (1.0, 49, 49), (True, 49, 49), (49, 49)):
        try:
            C.action_text(bad)
        except ValueError:
            continue
        raise AssertionError(f"action_text accepted {bad!r}")
    w = {a: (hi - lo) / C.MAX_BIN for a, (lo, hi) in C.RANGES.items()}
    for axis, (lo, hi) in C.RANGES.items():
        n = 997
        for i in range(n + 1):
            v = lo + (hi - lo) * i / n
            b = C.quantize(v, axis)
            edge = C.dequantize(b, axis)
            # Truncation: the value is in [edge, edge + w), never rounded up.
            assert edge <= v + 1e-12 and (b == C.MAX_BIN or v < edge + w[axis] - 1e-12), \
                (axis, v, b)
    # Just below a cell edge stays in the lower bin (round() would not).
    assert C.quantize(0.99 * w["fwd_m"], "fwd_m") == 0
    assert C.quantize(1.99 * w["fwd_m"], "fwd_m") == 1
    for fwd, down, dpsi in ((4.0, 0.0, 0.0), (0.0509, -0.003, 0.249), (5.0, 5.0, 1.1),
                            (0.0, -5.0, -1.1), (2.37, 1.23, -0.61)):
        bins = C.step_bins(fwd, down, dpsi)
        text = C.action_text(bins)
        assert re.fullmatch(r"[0-9]{2} [0-9]{2} [0-9]{2}", text), text
        p = C.parse_action(text + C.EOS_TEXT, require_eos=True)
        assert p.parse_ok and p.bins == bins and not p.land, (text, p)
        assert p.step == C.bins_to_step(bins)
    # The tiny negative 'down' of level flight goes to bin 48, as upstream's
    # labels did (aerialvla_codec docstring): the builder does not "fix" it.
    assert C.step_bins(4.0, -0.003, 0.0)[1] == 48


# =========================================================================== #
# 3. The phrase: DetectorHint's, for a fresh box, yaw-corrected to the capture
# =========================================================================== #
def _cap(t, yaw, box):
    return {"t_capture": t, "pose": {"x": 0.0, "y": 0.0, "up": 8.0, "yaw_rad": yaw}, "box": box}


def test_the_fresh_box_rule_is_detectorhints():
    B = _B()
    br = B.make_bridge()
    det = _det_at_bearing(0.0)
    ok, why, info = B.detector_phrase(_cap(T0, 0.0, {"det": det, "t_box": T0 - 1.0,
                                                     "yaw_box": 0.0}), br)
    assert ok == C.STRAIGHT_AHEAD and why is None, (ok, why)      # exactly 1.0 s: fresh
    assert math.isclose(info["age_s"], 1.0)
    ok, why, _ = B.detector_phrase(_cap(T0, 0.0, {"det": det, "t_box": T0 - 1.05,
                                                  "yaw_box": 0.0}), br)
    assert ok is None and why == "stale_box", why
    ok, why, _ = B.detector_phrase(_cap(T0, 0.0, None), br)
    assert ok is None and why == "no_box"
    ok, why, _ = B.detector_phrase(_cap(T0, 0.0, {"det": None, "t_box": T0, "yaw_box": 0.0}), br)
    assert ok is None and why == "no_box"
    ok, why, _ = B.detector_phrase(_cap(T0, 0.0, {"det": det, "t_box": None, "yaw_box": 0.0}), br)
    assert ok is None and why == "no_box"
    # A box from a frame captured just AFTER the VLA capture (the Grounder is
    # read when the capture is built, as DetectorHint reads it): fresh.
    ok, why, _ = B.detector_phrase(_cap(T0, 0.0, {"det": det, "t_box": T0 + 0.05,
                                                  "yaw_box": 0.0}), br)
    assert ok == C.STRAIGHT_AHEAD, (ok, why)


def test_the_phrase_is_the_bearing_rotated_by_the_turn_since_the_box_and_matches_detectorhint():
    B = _B()
    import vla_mission as VM
    from guardrail.vla_link import Capture, Pose
    det = _det_at_bearing(20.0)                  # 20 deg right on the box's frame
    # The aircraft turned 0.2 rad (11.5 deg) right since: 8.5 deg -> straight ahead.
    cap = _cap(T0, 0.2, {"det": det, "t_box": T0 - 0.5, "yaw_box": 0.0})
    phrase, why, info = B.detector_phrase(cap, B.make_bridge())
    assert phrase == C.STRAIGHT_AHEAD and why is None, (phrase, why)
    assert math.isclose(info["yaw_correction_deg"], math.degrees(0.2), abs_tol=1e-6), info
    assert math.isclose(info["bearing_deg"], 20.0, abs_tol=1e-6), info
    # Uncorrected it would have read forward-right.
    phrase0, _, _ = B.detector_phrase(_cap(T0, 0.0, {"det": det, "t_box": T0 - 0.5,
                                                     "yaw_box": 0.0}), B.make_bridge())
    assert phrase0 == C.FORWARD_RIGHT
    # The real DetectorHint, fed the same box and capture, says the same.
    from vla_bridge import VLABridge
    for deg, yaw_cap, yaw_box in ((20.0, 0.2, 0.0), (-30.0, -0.1, 0.3), (44.0, 0.0, -0.4),
                                  (-10.0, 3.1, -3.1), (5.0, 0.0, None)):
        d = _det_at_bearing(deg)
        hint = VM.DetectorHint(lambda d=d, yb=yaw_box: (d, T0 - 0.4, yb, T0 - 0.1),
                               VLABridge("a red car", hfov_deg=90.0), fresh_s=1.0)
        want = hint(Capture(T0, Pose(0.0, 0.0, 8.0, yaw_cap), b"\xff\xd8", None))
        got, _, _ = B.detector_phrase(_cap(T0, yaw_cap, {"det": d, "t_box": T0 - 0.4,
                                                         "yaw_box": yaw_box}), B.make_bridge())
        assert want is not None and got == want.phrase, (deg, yaw_cap, yaw_box, got, want)


def test_a_close_box_asks_nothing_unless_kept():
    B = _B()
    det = _det_at_bearing(0.0, w_frac=0.30)        # wider than BoxClose's 0.25
    cap = _cap(T0, 0.0, {"det": det, "t_box": T0 - 0.2, "yaw_box": 0.0})
    phrase, why, _ = B.detector_phrase(cap, B.make_bridge(), close=B.default_close_rule())
    assert phrase is None and why == "close_box", (phrase, why)
    phrase, why, _ = B.detector_phrase(cap, B.make_bridge(), close=None)
    assert phrase == C.STRAIGHT_AHEAD and why is None
    # The stand-off box (want-width 0.16) is not close: those samples stay.
    det = _det_at_bearing(0.0, w_frac=0.16)
    phrase, why, _ = B.detector_phrase(_cap(T0, 0.0, {"det": det, "t_box": T0 - 0.2,
                                                      "yaw_box": 0.0}),
                                       B.make_bridge(), close=B.default_close_rule())
    assert phrase == C.STRAIGHT_AHEAD, why


# =========================================================================== #
# 4. The dataset: samples, the split by run, the stats
# =========================================================================== #
def test_a_run_becomes_samples_with_the_servers_prompt_and_a_text_it_parses():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = _write_run(td, "runA", _straight(speed=4.0), seconds=8.0)
        out = td / "ds"
        st = B.build_dataset([run], out, horizon_s=1.0, stride_s=0.5, images=False)
        rows = [json.loads(ln) for ln in (out / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        assert rows and st["splits"]["train"] == len(rows), st["splits"]
        # 8 s at 10 Hz, a 1 s horizon: captures 0..7 s, one each 0.5 s.
        assert len(rows) == 15, len(rows)
        assert st["dropped"].get("no_future_pose") == 10, st["dropped"]
        assert st["dropped"].get("stride") == 81 - 15 - 10, st["dropped"]
        for r in rows:
            assert r["run"] == "runA" and r["split"] == "train"
            assert r["phrase"] == C.STRAIGHT_AHEAD and r["hint_source"] == "detector"
            assert r["object"] == "a red car"
            assert r["prompt"] == C.build_prompt(r["phrase"], r["object"])
            assert r["prompt_sha256"] == C.prompt_sha256(r["prompt"])
            p = C.parse_action(r["action_text"] + C.EOS_TEXT, require_eos=True)
            assert p.parse_ok and list(p.bins) == r["bins"], r
            assert r["bins"] == [78, 49, 49], r
            assert r["image"] is None and r["front_jpg_sha256"] and r["down_jpg_sha256"]
            # The request the server would get for this sample is valid v2.
            C.validate_request({"v": 2, "op": "infer", "seq": 1, "t_capture": r["t_capture"],
                                "pose": r["pose"], "front_jpg_b64": "eA==",
                                "down_jpg_b64": "eA==", "phrase": r["phrase"],
                                "hint_source": r["hint_source"], "object": r["object"],
                                "csp_text": None, "csp_mode": "off"})
            assert "truth_xy" in r["analysis"], "truth is kept for analysis, apart"
        ts = [r["t_capture"] for r in rows]
        assert all(b - a >= 0.5 - 1e-9 for a, b in zip(ts, ts[1:])), ts
        assert st["phrase_counts"] == {C.STRAIGHT_AHEAD: 15}
        assert st["label_hist"]["train"]["fwd_m"] == {"78": 15}, st["label_hist"]
        assert st["runs"]["runA"]["samples"] == 15


def test_hold_out_runs_never_reach_train():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        a = _write_run(td, "runA", _straight(speed=4.0))
        b = _write_run(td, "runB", _turning(speed=3.0, rate=0.2))
        c = _write_run(td, "runC", _straight(speed=2.0))
        out = td / "ds"
        st = B.build_dataset([a, b, c], out, horizon_s=1.0, stride_s=0.5,
                             holdout=["runB"], images=False)
        rows = [json.loads(ln) for ln in (out / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        train = {r["run"] for r in rows if r["split"] == "train"}
        hold = {r["run"] for r in rows if r["split"] == "holdout"}
        assert train == {"runA", "runC"} and hold == {"runB"}, (train, hold)
        assert st["splits"]["holdout"] == sum(1 for r in rows if r["run"] == "runB") > 0
        assert st["runs"]["runB"]["split"] == "holdout"
        assert st["holdout_runs"] == ["runB"]
        # A hold-out tag that names no run is refused (a typo would put the
        # run meant to be held out into train).
        for bad in (["runX"], ["runb"]):
            try:
                B.build_dataset([a, b], td / "ds2", holdout=bad, images=False)
            except B.DatasetError as exc:
                assert "runX" in str(exc) or "runb" in str(exc), exc
            else:
                raise AssertionError(f"holdout {bad} was accepted")
        # Two recordings with one tag are refused: the split is by tag.
        b2 = _write_run(td / "again", "runA", _straight())
        try:
            B.build_dataset([a, b2], td / "ds3", images=False)
        except B.DatasetError as exc:
            assert "runA" in str(exc)
        else:
            raise AssertionError("two runs tagged runA were accepted")
        # Every run held out: nothing to train on.
        try:
            B.build_dataset([a], td / "ds4", holdout=["runA"], images=False)
        except B.DatasetError as exc:
            assert "train" in str(exc)
        else:
            raise AssertionError("a dataset with no training run was built")


def test_drops_are_counted_by_reason_and_the_rotate_only_band_is_reported():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)

        def box(k, t, yaw_at):
            if 10 <= k < 20:
                return None                                           # no box
            if 20 <= k < 30:
                return {"det": _det_at_bearing(0.0), "t_box": t - 2.0, "yaw_box": yaw_at(t - 2.0)}
            if 30 <= k < 35:
                return {"det": _det_at_bearing(0.0, w_frac=0.3), "t_box": t - 0.2,
                        "yaw_box": yaw_at(t - 0.2)}                   # close
            return {"det": _det_at_bearing(0.0), "t_box": t - 0.3, "yaw_box": yaw_at(t - 0.3)}

        # A hard turn while moving: 0.4 rad per 1 s horizon and 4 m/s, a
        # label upstream's executor would only rotate on.
        run = _write_run(td, "runT", _turning(speed=4.0, rate=0.4), seconds=8.0, box=box)
        (run / "frames" / (json.loads((run / "captures.jsonl").read_text(
            encoding="utf-8").splitlines()[40])["front_jpg_sha256"] + ".jpg")).unlink()
        st = B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False)
        d = st["dropped"]
        assert d["no_box"] == 10 and d["stale_box"] == 10 and d["close_box"] == 5, d
        assert d["frame_missing"] == 1 and d["no_future_pose"] == 10, d
        n = st["splits"]["train"]
        assert n == 81 - 10 - 10 - 5 - 1 - 10, (n, d)
        band = st["rotate_only_band"]
        assert band["n"] == n and band["with_fwd"] == n, band
        assert "upstream" in band["note"] and "turns while moving" in band["note"], band
        assert st["runs"]["runT"]["dropped"] == d
        # --keep-close keeps the five close ones.
        st2 = B.build_dataset([run], td / "ds2", horizon_s=1.0, stride_s=0.0,
                              images=False, keep_close=True)
        assert st2["splits"]["train"] == n + 5 and "close_box" not in st2["dropped"]


def test_a_stand_off_with_small_steps_is_kept_and_counted():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = _write_run(td, "runS", _straight(speed=0.4), seconds=6.0)
        st = B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.5, images=False)
        assert st["splits"]["train"] > 0
        assert st["small_fwd"]["lt_1m"] == st["splits"]["train"], st["small_fwd"]
        rows = [json.loads(ln) for ln in (td / "ds" / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        assert {tuple(r["bins"]) for r in rows} == {(7, 49, 49)}, rows[0]["bins"]   # 0.4 m


def test_the_dataset_refuses_to_overwrite_and_needs_a_recording():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = _write_run(td, "runA", _straight())
        out = td / "ds"
        B.build_dataset([run], out, images=False)
        try:
            B.build_dataset([run], out, images=False)
        except B.DatasetError as exc:
            assert "--force" in str(exc)
        else:
            raise AssertionError("an existing dataset was overwritten without --force")
        B.build_dataset([run], out, images=False, force=True)
        # The flight folder works as well as its vla_record/ folder.
        st = B.build_dataset([run.parent], td / "ds_parent", images=False)
        assert "runA" in st["runs"]
        try:
            B.build_dataset([td / "nothing_here"], td / "ds5", images=False)
        except B.DatasetError as exc:
            assert "captures.jsonl" in str(exc)
        else:
            raise AssertionError("a folder with no recording was accepted")
        # Without record.json (a flight that died before the recorder
        # stopped) the tag is the flight folder's name.
        r2 = _write_run(td, "died_early", _straight(), record_json=False)
        assert "died_early" in B.build_dataset([r2], td / "ds6", images=False)["runs"]


def test_the_cli_writes_the_dataset_and_says_what_it_dropped():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        a = _write_run(td, "runA", _straight())
        b = _write_run(td, "runB", _straight(speed=3.0))
        out = td / "ds"
        rc = B.main([str(a), str(b), "--out", str(out), "--holdout", "runB",
                     "--horizon-s", "1.0", "--stride-s", "0.5", "--no-images"])
        assert rc == 0
        st = json.loads((out / "stats.json").read_text(encoding="utf-8"))
        assert st["args"]["horizon_s"] == 1.0 and st["args"]["holdout"] == ["runB"]
        assert st["format"].startswith("aerialvla-follow-steps/")
        assert st["codec"]["num_bins"] == 99
        assert B.main([str(a), "--out", str(td / "ds_bad"), "--holdout", "nope",
                       "--no-images"]) == 2


def test_the_mosaic_is_the_codecs_from_the_decoded_jpegs():
    Image = _pil()
    if Image is None:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    B = _B()

    def jpeg(k):
        out = []
        for col in ((200, 40 + k, 40), (30, 30, 180 + k % 50)):
            im = Image.new("RGB", (256, 256), col)
            for x in range(0, 256, 9):
                im.putpixel((x, (x * 3 + k) % 256), (250, 250, 250))
            buf = io.BytesIO()
            im.save(buf, format="JPEG", quality=95)
            out.append(buf.getvalue())
        return tuple(out)

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = _write_run(td, "runI", _straight(), seconds=4.0, jpeg=jpeg)
        out = td / "ds"
        st = B.build_dataset([run], out, horizon_s=1.0, stride_s=0.5, images=True)
        rows = [json.loads(ln) for ln in (out / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        assert rows and st["images"]["written"] == len({r["mosaic_sha256"] for r in rows})
        for r in rows:
            fb = (run / "frames" / f"{r['front_jpg_sha256']}.jpg").read_bytes()
            db = (run / "frames" / f"{r['down_jpg_sha256']}.jpg").read_bytes()
            # The server's own path: base64 JPEG -> decode -> mosaic.
            f = C.decode_image_b64(base64.b64encode(fb).decode("ascii"))
            d = C.decode_image_b64(base64.b64encode(db).decode("ascii"))
            m, info = C.build_mosaic(f, d, missing_down="refuse")
            assert r["mosaic_sha256"] == C.mosaic_sha256(m), r
            img = Image.open(out / r["image"])
            assert img.mode == "RGB" and img.size == C.MOSAIC_SIZE
            assert C.mosaic_sha256(img.convert("RGB")) == r["mosaic_sha256"], "PNG must be lossless"
            assert r["image"] == f"images/{r['mosaic_sha256']}.png"


# =========================================================================== #
# 5. The recorder
# =========================================================================== #
class _FakeGrounder:
    def __init__(self):
        self.box = None

    def latest(self):
        return self.box


def _msg(stamp_ns, tag):
    return {"data": f"{tag}-{stamp_ns}".encode(), "width": 256, "height": 256,
            "time_stamp": stamp_ns, "encoding": None}


def _recorder(td, obs, g, **kw):
    R = _R()
    return R.ExpertVLARecorder(td / "vla_record", obs, lambda: R.box_from_grounder(g.latest()),
                               tag="t_rec", object_text="a red car",
                               encode=kw.pop("encode", lambda m: bytes(m["data"])), **kw)


def test_the_recorder_is_off_unless_asked_and_the_expert_flight_is_unchanged():
    import follow_vlm as F
    a = F.build_arg_parser().parse_args([])
    assert a.record_vla is None
    assert F.flight_scene(a) == F.SCENE == "scene_semantic.jsonc"
    b = F.build_arg_parser().parse_args(["--record-vla", r"demo\out\t\vla_record"])
    assert b.record_vla == r"demo\out\t\vla_record"
    assert F.flight_scene(b) == "scene_semantic_vla.jsonc"
    # Inside fly(): every use of the recorder is under a guard, so with the
    # flag off nothing of it runs (no camera subscribed, no tick noted, no
    # metrics key): `vla_rec` is assigned only under `if args.record_vla`
    # and read only under `if vla_rec is not None`.
    tree = ast.parse(FOLLOW.read_text(encoding="utf-8"))
    fly = next(n for n in ast.walk(tree) if isinstance(n, ast.AsyncFunctionDef) and n.name == "fly")
    parents = {}
    for n in ast.walk(fly):
        for c in ast.iter_child_nodes(n):
            parents[c] = n

    def guarded(node, test):
        p = node
        while p in parents:
            p = parents[p]
            if isinstance(p, ast.If) and test(ast.unparse(p.test)):
                return True
        return False

    uses = [n for n in ast.walk(fly) if isinstance(n, ast.Attribute)
            and isinstance(n.value, ast.Name) and n.value.id == "vla_rec"]
    assert len(uses) >= 4, "the recorder is not wired into fly()"
    for u in uses:
        assert guarded(u, lambda s: "vla_rec is not None" in s or "args.record_vla" in s), \
            f"line {u.lineno}: vla_rec.{u.attr} outside its guard"
    made = [n for n in ast.walk(fly) if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == "vla_rec" for t in n.targets)
            and not (isinstance(n.value, ast.Constant) and n.value.value is None)]
    assert made and all(guarded(n, lambda s: "args.record_vla" in s) for n in made)
    stats = [n for n in ast.walk(fly) if isinstance(n, ast.Subscript)
             and isinstance(n.slice, ast.Constant) and n.slice.value == "vla_record"]
    assert stats and all(guarded(n, lambda s: "is not None" in s) for n in stats)
    src = FOLLOW.read_text(encoding="utf-8")
    assert "World(client, flight_scene(args)" in src, "the scene is chosen by the flag"


def test_the_record_variant_adds_exactly_aerialvlas_two_cameras():
    from guardrail.manifest import _strip_jsonc

    def j(name):
        return json.loads(_strip_jsonc((CFG / name).read_text(encoding="utf-8")))

    R = _R()
    base, var, vla = j("robot_semantic_quad.jsonc"), j(R.ROBOT_CONFIG), j("robot_vla_quad.jsonc")
    assert set(base) == set(var)
    for k in base:
        if k != "sensors":
            assert base[k] == var[k], k
    added = [s for s in var["sensors"] if s not in base["sensors"]]
    assert [s for s in var["sensors"] if s in base["sensors"]] == base["sensors"]
    want = [s for s in vla["sensors"] if s["id"] in ("VLAFront", "VLADown")]
    assert added == want and [s["id"] for s in added] == ["VLAFront", "VLADown"], added
    for s in added:                     # never streamed: --view-switch's order is unchanged
        assert not any(c.get("streaming-enabled") for c in s["capture-settings"])
    sb, sv = j("scene_semantic.jsonc"), j(R.SCENE)
    (ab,), (av,) = sb["actors"], sv["actors"]
    assert av["robot-config"] == R.ROBOT_CONFIG and ab["robot-config"] == "robot_semantic_quad.jsonc"
    assert {**av, "robot-config": None} == {**ab, "robot-config": None}
    assert {k: v for k, v in sv.items() if k not in ("actors", "id")} == \
        {k: v for k, v in sb.items() if k not in ("actors", "id")}
    import follow_vlm as F
    assert F.SCENE_VLA_RECORD == R.SCENE
    raw = (CFG / R.ROBOT_CONFIG).read_bytes()
    assert raw.count(b"\r\n") == raw.count(b"\n"), "CRLF, like robot_semantic_quad.jsonc"


def test_the_recorder_writes_paired_captures_with_the_box_the_command_and_the_track():
    from semantic_demo import SemanticObs
    R = _R()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        obs, g = SemanticObs(), _FakeGrounder()
        rec = _recorder(td, obs, g)
        # Before start(): frames are not recorded.
        rec.put_front(_msg(1, "f"))
        rec.put_down(_msg(1, "d"))
        t_start = time.time()
        rec.start(t_start)
        stamp0 = 5_000_000_000
        for k in range(30):
            t = time.time()
            st = stamp0 + k * 100_000_000
            yaw = 0.01 * k
            obs.put_pose_full(t, 1.0 * k, 0.5, 8.0, yaw, 0.0, 0.0, stamp=st)
            g.box = {"seq": k, "t": t, "t_det": t - 0.02, "tier": "ok",
                     "det": (384.0 + k, 260.0, 90.0, 45.0, 0.31, 768, 432, 0.5),
                     "cap": {"t_capture": t - 0.25, "t_cap": t - 0.2,
                             "pose": {"x": 0.0, "y": 0.0, "up": 8.0, "yaw": yaw - 0.02}}}
            rec.note_tick(t, 1.0 * k, 0.5, 8.0, yaw, mode="track",
                          cmd={"vx": 3.0, "vy": 0.1, "vz_up": 0.0, "yaw_rate": 0.1},
                          asked={"vx": 3.2, "vy": 0.1, "vz_up": 0.0, "yaw_rate": 0.1},
                          touched=(k % 7 == 0), truth=(15.0 + k, 0.5), obj="a red car")
            rec.put_down(_msg(st, "d"))
            rec.put_front(_msg(st, "f"))
            time.sleep(0.03)
        time.sleep(0.2)
        s = rec.stop()
        d = td / "vla_record"
        caps = [json.loads(ln) for ln in (d / R.CAPTURES).read_text(encoding="utf-8").splitlines()]
        track = [json.loads(ln) for ln in (d / R.TRACK).read_text(encoding="utf-8").splitlines()]
        assert len(track) == 30 and s["ticks"] == 30 and s["ticks_dropped"] == 0, s
        assert track[3]["cmd"]["vx"] == 3.0 and track[3]["mode"] == "track"
        assert track[3]["truth"] == [18.0, 0.5] and track[3]["author"] == "follow_vlm"
        assert len(caps) >= 20, (len(caps), s)
        assert s["front_frames"] == 30, s
        assert s["front_frames"] == s["capture_attempts"] + s["skipped_front_frames"], s
        assert s["captures"] == len(caps) and s["captures"] <= s["capture_attempts"]
        for c in caps:
            assert set(c["pose"]) == {"x", "y", "up", "yaw_rad"}
            assert (d / "frames" / f"{c['front_jpg_sha256']}.jpg").is_file()
            assert (d / "frames" / f"{c['down_jpg_sha256']}.jpg").is_file()
            assert c["front_jpg_sha256"] != c["down_jpg_sha256"]
            # Paired by sim stamp: the same stamp, or the next down frame when
            # the writer read between the loop's put_down and put_front.
            assert c["down_stamp_ns"] - c["stamp_ns"] in (0, 100_000_000), c
            assert c["pair_dt_s"] in (0.0, 0.1) and c["t_capture_basis"] == "stamp", c
            b = c["box"]
            assert b["det"][5:7] == [768, 432] and b["t_box"] < c["t_capture"] + 1.0, b
            assert b["yaw_box"] is not None and b["tier"] == "ok"
            assert c["tick"]["mode"] == "track" and c["tick"]["t"] <= c["t_capture"] + 1e-6
            assert c["object"] == "a red car" and c["truth"] is not None
            assert math.isclose(c["t_mission"], c["t_capture"] - t_start, abs_tol=1e-3)
        # The pose at the capture came from the loop's pose ring at the
        # frame's sim stamp: x = k at stamp k.
        for c in caps:
            k = (c["stamp_ns"] - stamp0) / 1e8
            assert math.isclose(c["pose"]["x"], k, abs_tol=1e-6), c
        r = json.loads((d / R.RECORD).read_text(encoding="utf-8"))
        assert r["format"] == R.FORMAT and r["tag"] == "t_rec" and r["object"] == "a red car"
        assert r["t0"] == t_start and r["counts"]["captures"] == len(caps)
        assert r["cameras"]["front"] == "VLAFront" and r["cameras"]["size"] == [256, 256]
        assert r["sim_config"]["robot_config"] == R.ROBOT_CONFIG


def test_a_tick_costs_a_deque_append_even_when_the_writer_is_slow():
    """The flight must not slow: the loop's side is note_tick (and the camera
    callbacks), the writer's encode/IO happens on its own thread. With an
    encoder that takes 80 ms a frame, the ticks still go in at once and none
    is lost; frames the writer could not keep up with are counted, not
    queued."""
    from semantic_demo import SemanticObs
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        obs, g = SemanticObs(), _FakeGrounder()

        def slow(m):
            time.sleep(0.08)
            return bytes(m["data"])

        rec = _recorder(td, obs, g, encode=slow)
        rec.start(time.time())
        cost = []
        stamp0 = 7_000_000_000
        for k in range(60):
            t = time.time()
            st = stamp0 + k * 50_000_000
            obs.put_pose_full(t, float(k), 0.0, 8.0, 0.0, stamp=st)
            rec.put_front(_msg(st, "f"))
            rec.put_down(_msg(st, "d"))
            t1 = time.perf_counter()
            rec.note_tick(t, float(k), 0.0, 8.0, 0.0, mode="track",
                          cmd={"vx": 1.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0})
            cost.append(time.perf_counter() - t1)
            time.sleep(0.02)
        s = rec.stop()
        assert s["ticks"] == 60 and s["ticks_dropped"] == 0, s
        assert max(cost) < 0.05 and sum(cost) / len(cost) < 0.002, (max(cost), sum(cost) / 60)
        assert s["skipped_front_frames"] > 0, s
        assert s["front_frames"] == s["capture_attempts"] + s["skipped_front_frames"], s


def test_the_flights_own_encoder_writes_jpegs_the_builder_turns_into_the_mosaic():
    """The recorder with VLAFrames' default encoder (jpeg_from_msg: OpenCV on
    the raw BGR message, as a flight runs it), then the builder's mosaic
    from those files. Lab-only: SKIPs without OpenCV and Pillow (no CI job
    installs OpenCV)."""
    Image = _pil()
    try:
        import cv2                                                   # noqa: F401
        import numpy as np
    except ImportError:
        cv2 = None
    if cv2 is None or Image is None:
        print("      SKIP: needs OpenCV and Pillow (the flight's environment has both)")
        return SKIP
    from semantic_demo import SemanticObs
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        obs, g = SemanticObs(), _FakeGrounder()
        rec = _recorder(td, obs, g, encode=None)
        rec.start(time.time())
        stamp0 = 11_000_000_000
        for k in range(30):
            t = time.time()
            st = stamp0 + k * 100_000_000
            obs.put_pose_full(t, 0.4 * k, 0.0, 8.0, 0.0, stamp=st)
            g.box = {"seq": k, "t_det": t, "det": (384.0, 260.0, 90.0, 45.0, 0.3, 768, 432, 0.5),
                     "cap": {"t_capture": t - 0.2, "pose": {"yaw": 0.0}}}
            rec.note_tick(t, 0.4 * k, 0.0, 8.0, 0.0, mode="track", cmd=None)
            for cam, base in (("d", 40), ("f", 160)):
                bgr = np.full((256, 256, 3), base, np.uint8)
                c0 = (k * 8) % 240
                bgr[:, c0:c0 + 16] = (0, 0, 255)                    # a red band, moving
                m = {"data": bgr.tobytes(), "width": 256, "height": 256, "time_stamp": st}
                (rec.put_down if cam == "d" else rec.put_front)(m)
            time.sleep(0.1)
        time.sleep(0.15)
        s = rec.stop()
        assert s["captures"] >= 10 and s["saver"]["errors"] == 0, s
        st = B.build_dataset([td / "vla_record"], td / "ds", horizon_s=1.0, stride_s=0.5,
                             images=True)
        rows = [json.loads(ln) for ln in (td / "ds" / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        assert rows and st["images"]["written"] >= 1
        r = rows[0]
        front = Image.open(td / "vla_record" / "frames" / f"{r['front_jpg_sha256']}.jpg")
        assert front.size == (256, 256) and front.format == "JPEG"
        cap = next(json.loads(ln) for ln in (td / "vla_record" / "captures.jsonl").read_text(
            encoding="utf-8").splitlines() if json.loads(ln)["i"] == r["i"])
        k = round((cap["stamp_ns"] - stamp0) / 1e8)
        # BGR in, RGB out: the band is red in the JPEG the server reads.
        px = front.convert("RGB").getpixel(((k * 8) % 240 + 8, 128))
        assert px[0] > 200 and px[1] < 60 and px[2] < 60, px
        assert Image.open(td / "ds" / r["image"]).size == C.MOSAIC_SIZE


def test_box_from_grounder_is_vla_flys_reading_of_the_grounder():
    R = _R()
    assert R.box_from_grounder(None) is None
    assert R.box_from_grounder({"det": None, "t_det": 5.0}) is None
    g = {"seq": 9, "t_det": 105.0, "tier": "soft",
         "det": (400.0, 200.0, 80.0, 40.0, 0.2, 768, 432, 0.4),
         "cap": {"t_capture": 104.6, "t_cap": 104.7, "pose": {"yaw": 0.25}}}
    b = R.box_from_grounder(g)
    assert b == {"det": [400.0, 200.0, 80.0, 40.0, 0.2, 768, 432, 0.4], "t_box": 104.6,
                 "yaw_box": 0.25, "t_published": 105.0, "seq": 9, "tier": "soft"}, b
    # vla_fly's fallbacks: t_cap, then t_det; no pose, no heading.
    g2 = dict(g, cap={"t_cap": 104.7})
    assert R.box_from_grounder(g2)["t_box"] == 104.7
    assert R.box_from_grounder(g2)["yaw_box"] is None
    assert R.box_from_grounder(dict(g, cap=None))["t_box"] == 105.0


def test_the_recording_goes_straight_into_the_builder():
    from semantic_demo import SemanticObs
    B, R = _B(), _R()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        obs, g = SemanticObs(), _FakeGrounder()
        rec = _recorder(td, obs, g)
        t_start = time.time()
        rec.start(t_start)
        stamp0 = 9_000_000_000
        for k in range(45):
            t = time.time()
            st = stamp0 + k * 100_000_000
            obs.put_pose_full(t, 0.3 * k, 0.0, 8.0, 0.0, stamp=st)
            g.box = {"seq": k, "t_det": t, "tier": "ok",
                     "det": (384.0, 260.0, 90.0, 45.0, 0.31, 768, 432, 0.5),
                     "cap": {"t_capture": t - 0.2, "pose": {"yaw": 0.0}}}
            rec.note_tick(t, 0.3 * k, 0.0, 8.0, 0.0, mode="track",
                          cmd={"vx": 3.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0},
                          truth=(20.0, 0.0), obj="a red car")
            rec.put_down(_msg(st, "d"))
            rec.put_front(_msg(st, "f"))
            time.sleep(0.1)
        time.sleep(0.15)
        rec.stop()
        st = B.build_dataset([td / "vla_record"], td / "ds", horizon_s=1.0, stride_s=0.5,
                             images=False)
        assert st["runs"]["t_rec"]["samples"] >= 5, st["runs"]
        rows = [json.loads(ln) for ln in (td / "ds" / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        for r in rows:
            # ~3 m/s along yaw 0: fwd ~3 m per 1 s horizon (wall-clock pacing).
            assert 2.0 < r["label"]["fwd_m"] < 4.0 and r["phrase"] == C.STRAIGHT_AHEAD, r
        assert st["pose_check"]["capture_vs_track_m"]["max"] < 0.5, st["pose_check"]


def test_the_launcher_switch_records_to_c_through_a_junction():
    src = PS1.read_text(encoding="utf-8")
    assert "[switch]$RecordVla" in src
    assert src.count('"--record-vla"') == 1, "the flag is added in exactly one place"
    # ...inside `if ($RecordVla) {`: every line from the guard to the flag is
    # indented deeper than the guard (PowerShell's braces, by layout).
    lines = src.splitlines()
    fi = next(n for n, ln in enumerate(lines) if '"--record-vla"' in ln)
    gi = max(n for n, ln in enumerate(lines[:fi]) if ln.lstrip().startswith("if ($RecordVla)"))
    ind = len(lines[gi]) - len(lines[gi].lstrip())
    for ln in lines[gi + 1:fi + 1]:
        assert not ln.strip() or len(ln) - len(ln.lstrip()) > ind, \
            f"--record-vla is passed outside `if ($RecordVla)`: {ln!r}"
    guard = src.index(lines[gi])
    blk = src[guard:src.index("& $Py @a")]
    assert "vla_record" in blk and "New-Item -ItemType Junction" in blk and "$OffloadRoot" in blk
    # The flight's own parser takes what the launcher passes.
    import follow_vlm as F
    a = F.build_arg_parser().parse_args(["--record-vla", r"demo\out\redcar_x\vla_record"])
    assert a.record_vla.endswith("vla_record")


# =========================================================================== #
# 6. DART: perturbing the expert, the overlap rule, the per-run report, mirror
#
# ft_expert_01 (2026-10-23, 0.953 within 30 m, 2826 captures) gave 507
# samples, every one "straight ahead ": the expert keeps the car centred, so
# the data never shows an off-centre car being brought back. The fix is
# DART-style noise: knock the executed command off for a second now and
# then, label only the expert's clean motion, keep what follows.
# =========================================================================== #
def _onsets(p, until_s=400.0):
    p.state(until_s)
    return [e for e in p.episodes if e["on_s"] <= until_s]


def test_the_perturbation_schedule_is_seeded_and_in_the_asked_ranges():
    R = _R()
    a, b = R.Perturber(7), R.Perturber(7)
    sa = [a.state(k * 0.1) for k in range(3000)]
    sb = [b.state(k * 0.1) for k in range(3000)]
    assert sa == sb, "the same seed must give the same perturbations"
    assert sa != [R.Perturber(8).state(k * 0.1) for k in range(3000)]
    eps = _onsets(a, 300.0)
    assert len(eps) >= 300 / 8.0 - 1, len(eps)
    assert 5.0 <= eps[0]["on_s"] <= 8.0, eps[0]           # never at the mission start
    for e0, e1 in zip(eps, eps[1:]):
        assert 5.0 <= e1["on_s"] - e0["on_s"] <= 8.0, (e0, e1)
    for e in eps:
        assert 1.0 <= e["off_s"] - e["on_s"] <= 1.5, e
        assert 0.3 <= abs(e["yaw_rate"]) <= 0.5, e
        assert e["speed_scale"] == 1.0, "no speed scaling unless asked"
    assert {math.copysign(1, e["yaw_rate"]) for e in eps} == {1.0, -1.0}, "both signs"
    on = sum(1 for s in sa if s["active"])
    want = sum(round((e["off_s"] - e["on_s"]) / 0.1) for e in eps if e["off_s"] <= 300.0)
    assert abs(on - want) <= len(eps) + 1, (on, want)
    for s in sa:
        assert (s["yaw_rate"] != 0.0) == s["active"] and (s["speed_scale"] == 1.0 or s["active"])
    # The speed option draws the same schedule: an A/B changes one thing.
    c = R.Perturber(7, speed_scale=(0.5, 1.0))
    ec = _onsets(c, 300.0)
    assert [(e["on_s"], e["off_s"], e["yaw_rate"]) for e in ec] == \
        [(e["on_s"], e["off_s"], e["yaw_rate"]) for e in eps]
    assert all(0.5 <= e["speed_scale"] <= 1.0 for e in ec) and len({e["speed_scale"] for e in ec}) > 3
    # Asking late first does not change the early answers.
    d = R.Perturber(7)
    d.state(250.0)
    assert [d.state(k * 0.1) for k in range(3000)] == sa


def test_a_perturbation_offsets_the_yaw_rate_scales_the_speed_and_is_a_no_op_off():
    from guardrail.models import Action4D
    R = _R()
    p = R.Perturber(7, speed_scale=(0.5, 1.0))
    e = _onsets(p)[0]
    a = Action4D(vx=3.0, vy=1.0, vz_up=0.2, yaw_rate=0.1)
    out, rec = p.apply(a, e["on_s"] - 0.5)
    assert out is a, "an inactive tick flies the expert's own command object"
    assert rec == {"active": False, "yaw_rate": 0.0, "speed_scale": 1.0, "episode": None}, rec
    out, rec = p.apply(a, e["on_s"] + 0.1)
    assert math.isclose(out.yaw_rate, 0.1 + e["yaw_rate"]) and out.vz_up == 0.2
    assert math.isclose(out.vx, 3.0 * e["speed_scale"]) and math.isclose(out.vy, e["speed_scale"])
    assert rec["active"] and rec["episode"] == e["k"] and rec["yaw_rate"] == e["yaw_rate"]
    out, rec = p.apply(a, e["off_s"] + 0.01)
    assert out is a and not rec["active"]
    s = p.summary()
    assert s["ticks"] == 3 and s["active_ticks"] == 1 and s["seed"] == 7, s
    assert s["yaw_rate_rad_s"] == [0.3, 0.5] and s["every_s"] == [5.0, 8.0], s
    assert s["dur_s"] == [1.0, 1.5] and s["speed_scale"] == [0.5, 1.0]
    assert s["episodes"][0]["on_s"] == e["on_s"] and "applied after the expert" in s["rule"]


def test_the_stronger_knobs_replace_the_experts_yaw_or_widen_the_offset():
    """--perturb-replace-yaw: during a perturbation only the offset turns the
    aircraft (the expert's own yaw-rate command is dropped), so the bearing
    drifts uncorrected; --perturb-yaw-rate LO,HI widens the offset. Both
    off by default (the spec: an offset ADDED to the expert's command)."""
    from guardrail.models import Action4D
    R = _R()
    p = R.Perturber(7, replace_yaw=True)
    e = _onsets(p)[0]
    a = Action4D(vx=3.0, vy=0.0, vz_up=0.0, yaw_rate=0.25)
    out, rec = p.apply(a, e["on_s"] + 0.1)
    assert out.yaw_rate == e["yaw_rate"] and rec["active"], (out, e)
    assert p.apply(a, e["on_s"] - 0.1)[0] is a
    assert p.summary()["replace_yaw"] is True and R.Perturber(7).summary()["replace_yaw"] is False
    w = R.Perturber(7, yaw_rate=(0.6, 0.9))
    assert all(0.6 <= abs(x["yaw_rate"]) <= 0.9 for x in _onsets(w))
    # The same schedule (onsets, durations, signs): only the magnitude moves.
    base = _onsets(R.Perturber(7))
    assert [(x["on_s"], x["off_s"], x["yaw_rate"] > 0) for x in _onsets(w)] == \
        [(x["on_s"], x["off_s"], x["yaw_rate"] > 0) for x in base]
    import follow_vlm as F
    a = F.build_arg_parser().parse_args([])
    assert a.perturb_yaw_rate == (0.3, 0.5) and a.perturb_replace_yaw is False
    b = F.build_arg_parser().parse_args(["--record-vla", "d", "--record-perturb",
                                         "--perturb-yaw-rate", "0.6,0.9", "--perturb-replace-yaw"])
    assert b.perturb_yaw_rate == (0.6, 0.9) and b.perturb_replace_yaw
    for argv in (["--record-vla", "d", "--perturb-replace-yaw"],
                 ["--record-vla", "d", "--perturb-yaw-rate", "0.6,0.9"],
                 ["--record-vla", "d", "--record-perturb", "--perturb-yaw-rate", "0.9,0.6"],
                 ["--record-vla", "d", "--record-perturb", "--perturb-yaw-rate", "a,b"]):
        try:
            F.main(argv)
        except SystemExit as exc:
            assert exc.code == 2, (argv, exc.code)
        else:
            raise AssertionError(f"{argv} was accepted")


def _point_vehicle_recording(root: Path, tag: str, perturber, seconds=300.0, replace=False):
    """The follow expert's yaw servo (follow_vlm: 1.2 x bearing, capped)
    and a stand-off speed law on a point vehicle chasing a car down a gently
    curving street, the Perturber on the executed command, written in the
    recorder's format with the box at the true bearing."""
    from guardrail.models import Action4D
    dt = 0.1
    d = root / tag / "vla_record"
    (d / "frames").mkdir(parents=True)
    x = y = yaw = 0.0
    tx, ty, th = 16.0, 0.0, 0.0
    track, caps = [], []
    for k in range(int(seconds / dt)):
        t, s = T0 + k * dt, k * dt
        th += 0.05 * math.sin(s / 7.0) * dt
        tx, ty = tx + 4.0 * math.cos(th) * dt, ty + 4.0 * math.sin(th) * dt
        b = _wrap(math.atan2(ty - y, tx - x) - yaw)
        v = max(0.0, min(5.0, 4.0 + 0.5 * (math.hypot(tx - x, ty - y) - 16.0)))
        a = Action4D(vx=v * math.cos(yaw), vy=v * math.sin(yaw), vz_up=0.0,
                     yaw_rate=max(-0.8, min(0.8, 1.2 * b)))
        row = {"t": t, "x": x, "y": y, "up": 8.0, "yaw": yaw, "mode": "track"}
        if perturber is not None:
            a, row["perturb"] = perturber.apply(a, s)
        track.append(row)
        if abs(b) < math.radians(44):
            fb, db = f"f{k}".encode(), f"d{k}".encode()
            hf, hd = hashlib.sha256(fb).hexdigest(), hashlib.sha256(db).hexdigest()
            (d / "frames" / f"{hf}.jpg").write_bytes(fb)
            (d / "frames" / f"{hd}.jpg").write_bytes(db)
            caps.append({"i": k, "t_capture": t, "pose": {"x": x, "y": y, "up": 8.0, "yaw_rad": yaw},
                         "front_jpg_sha256": hf, "down_jpg_sha256": hd,
                         "box": {"det": _det_at_bearing(math.degrees(b)), "t_box": t,
                                 "yaw_box": yaw},
                         "tick": {"mode": "track"}, "object": "a red car"})
        yaw = _wrap(yaw + a.yaw_rate * dt)
        x, y = x + a.vx * dt, y + a.vy * dt
    for name, rows in (("track.jsonl", track), ("captures.jsonl", caps)):
        with (d / name).open("w", encoding="utf-8", newline="\n") as fh:
            for r in rows:
                fh.write(json.dumps(r) + "\n")
    (d / "record.json").write_text(json.dumps({"tag": tag, "object": "a red car"}),
                                   encoding="utf-8")
    return d


def test_a_perturbed_expert_gives_turn_labels_and_the_stronger_knobs_side_phrases():
    """Closed loop through the real Perturber and the real builder, on a
    point vehicle (idealised: no yaw lag, no detector latency). The clean
    expert centres the car: every label 'straight ahead ' with |dpsi| <
    0.05 - ft_expert_01's 507 of 507. The spec's additive 0.3-0.5 rad/s
    gives turn labels but the expert's 1.2 gain holds the bearing near or
    under the 15 deg band, so few side phrases; dropping the expert's yaw
    during the perturbation gives far more of both."""
    B, R = _B(), _R()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        got = {}
        for name, p in (("clean", None), ("add", R.Perturber(101)),
                        ("replace", R.Perturber(101, replace_yaw=True))):
            run = _point_vehicle_recording(td, name, p)
            st = B.build_dataset([run], td / f"ds_{name}", images=False)
            got[name] = st["runs"][name]
        c, a, r = got["clean"], got["add"], got["replace"]
        assert c["phrase_counts"] == {C.STRAIGHT_AHEAD: c["samples"]}, c["phrase_counts"]
        assert c["abs_dpsi_hist"] == {"0.00-0.05": c["samples"]}, c["abs_dpsi_hist"]

        def turning(run):
            return sum(n for band, n in run["abs_dpsi_hist"].items() if band != "0.00-0.05")

        def side(run):
            return sum(n for ph, n in run["phrase_counts"].items() if ph != C.STRAIGHT_AHEAD)

        assert a["perturb"]["post_perturb_3s"] > 0 and turning(a) > 0.2 * a["samples"], a
        assert side(r) > 3 * max(1, side(a)), (side(a), side(r))
        assert r["phrase_counts"].get(C.FORWARD_LEFT) and r["phrase_counts"].get(C.FORWARD_RIGHT)
        assert turning(r) > turning(a)


def test_the_perturbation_is_off_unless_asked_and_the_flight_is_unchanged_when_off():
    import follow_vlm as F
    a = F.build_arg_parser().parse_args([])
    assert a.record_perturb is False and a.perturb_speed is False
    for argv in (["--record-perturb"], ["--record-vla", "x", "--perturb-speed"]):
        try:
            F.main(argv)
        except SystemExit as exc:
            assert exc.code == 2, (argv, exc.code)
        else:
            raise AssertionError(f"{argv} was accepted")
    src = FOLLOW.read_text(encoding="utf-8")
    # The Shield judges `flown_in`: the expert's `smooth`, replaced only by
    # the perturber's output when there is one.
    assert src.count("d = filter_as_body(shield, state, flown_in)") == 1, "the loop's Shield call moved"
    assert "d = filter_as_body(shield, state, smooth)" not in src
    tree = ast.parse(src)
    fly = next(n for n in ast.walk(tree) if isinstance(n, ast.AsyncFunctionDef) and n.name == "fly")
    parents = {c: n for n in ast.walk(fly) for c in ast.iter_child_nodes(n)}

    def guarded(node, test):
        p = node
        while p in parents:
            p = parents[p]
            if isinstance(p, ast.If) and test(ast.unparse(p.test)):
                return True
        return False

    flown = [n for n in ast.walk(fly) if isinstance(n, ast.Assign)
             and any(isinstance(t, ast.Name) and t.id == "flown_in" for t in n.targets)
             or isinstance(n, ast.Assign) and any(
                 isinstance(t, ast.Tuple) and any(isinstance(e, ast.Name) and e.id == "flown_in"
                                                  for e in t.elts) for t in n.targets)]
    plain = [n for n in flown if isinstance(n.value, ast.Name) and n.value.id == "smooth"]
    assert len(plain) == 1 and not guarded(plain[0], lambda s: "perturb" in s), plain
    # ...or by --decide-every-s's hold (demo/decide_rate.py), refused with a
    # perturbation and pinned in tests/test_follow_decide_rate.py, or by
    # --vla-hierarchy's intent layer (demo/vla_hierarchy.py), refused with
    # either and with a recording, pinned in tests/test_vla_hierarchy.py.
    for n in flown:
        if n not in plain:
            assert guarded(n, lambda s: "perturber is not None" in s
                           or "decider is not None" in s
                           or "hierarchy is not None" in s), f"line {n.lineno}"
    uses = [n for n in ast.walk(fly) if isinstance(n, ast.Attribute)
            and isinstance(n.value, ast.Name) and n.value.id == "perturber"]
    assert uses, "the perturber is not wired in"
    for u in uses:
        assert guarded(u, lambda s: "perturber is not None" in s or "args.record_perturb" in s), \
            f"line {u.lineno}: perturber.{u.attr} outside its guard"
    made = [n for n in ast.walk(fly) if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == "perturber" for t in n.targets)
            and not (isinstance(n.value, ast.Constant) and n.value.value is None)]
    assert made and all(guarded(n, lambda s: "args.record_perturb" in s) for n in made)
    # A row says "perturb" only on a perturbed flight.
    rows = [n for n in ast.walk(fly) if isinstance(n, ast.Subscript)
            and isinstance(n.slice, ast.Constant) and n.slice.value == "perturb"]
    assert rows and all(guarded(n, lambda s: "is not None" in s) for n in rows), rows


def test_the_recorder_keeps_the_perturbation_per_tick_and_per_capture():
    from semantic_demo import SemanticObs
    R = _R()
    for perturbed in (False, True):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            obs, g = SemanticObs(), _FakeGrounder()
            pt = R.Perturber(3) if perturbed else None
            rec = _recorder(td, obs, g, perturber=pt)
            t0 = time.time()
            rec.start(t0)
            stamp0 = 13_000_000_000
            for k in range(20):
                t = time.time()
                st = stamp0 + k * 100_000_000
                obs.put_pose_full(t, float(k), 0.0, 8.0, 0.0, stamp=st)
                pr = None
                if pt is not None:
                    pr = {"active": k >= 10, "yaw_rate": 0.4 if k >= 10 else 0.0,
                          "speed_scale": 1.0, "episode": 0 if k >= 10 else None}
                rec.note_tick(t, float(k), 0.0, 8.0, 0.0, mode="track", cmd=None, perturb=pr)
                rec.put_down(_msg(st, "d"))
                rec.put_front(_msg(st, "f"))
                time.sleep(0.03)
            time.sleep(0.15)
            s = rec.stop()
            d = td / "vla_record"
            track = [json.loads(ln) for ln in (d / R.TRACK).read_text(encoding="utf-8").splitlines()]
            caps = [json.loads(ln) for ln in (d / R.CAPTURES).read_text(encoding="utf-8").splitlines()]
            r = json.loads((d / R.RECORD).read_text(encoding="utf-8"))
            if not perturbed:
                assert all("perturb" not in x for x in track + caps), "an unperturbed recording " \
                    "has no perturb key"
                assert r.get("perturb") is None and s.get("perturb") is None
                continue
            assert [x["perturb"]["active"] for x in track] == [k >= 10 for k in range(20)]
            assert caps and all("perturb" in c for c in caps)
            for c in caps:
                assert c["perturb"] == c["tick"]["perturb"]
            assert r["perturb"]["seed"] == 3 and s["perturb"]["seed"] == 3
            assert R.record_summary(s)["perturb"]["seed"] == 3


def _perturb_window(lo_k, hi_k):
    return lambda k: ({"active": True, "yaw_rate": 0.4, "speed_scale": 1.0, "episode": 0}
                      if lo_k <= k < hi_k else
                      {"active": False, "yaw_rate": 0.0, "speed_scale": 1.0, "episode": None})


def test_a_sample_whose_horizon_overlaps_a_perturbation_is_dropped_and_after_it_kept():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        # Active on ticks 30..41: commands at t = 3.0 .. 4.1, clean again from 4.2.
        run = _write_run(td, "runP", _straight(), seconds=8.0, perturb=_perturb_window(30, 42))
        st = B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False)
        # Default margin 0.3 s after the last perturbed command (the vehicle's
        # response): dropped when t < 4.2 + 0.3 and t + 1.0 > 3.0, so
        # t = 2.1 .. 4.4 (24 captures); t = 2.0 ends exactly at the onset.
        assert st["dropped"]["perturbed"] == 24, st["dropped"]
        rows = [json.loads(ln) for ln in (td / "ds" / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        ts = sorted(round(r["t_capture"] - T0, 3) for r in rows)
        assert 2.0 in ts and 4.5 in ts and not any(2.0 < t < 4.5 for t in ts), ts
        for r in rows:
            since = r["perturb"]["since_end_s"]
            if r["t_capture"] - T0 >= 4.5 - 1e-9:
                assert math.isclose(since, r["t_capture"] - T0 - 4.2, abs_tol=1e-6), r["perturb"]
            else:
                assert since is None, r["perturb"]
        # Starting off-centre after it is the point: 4.5 .. 7.0 s (26), counted.
        assert st["perturb"]["post_perturb_3s"] == 26, st["perturb"]
        assert st["runs"]["runP"]["perturb"] == {"intervals": 1, "active_s": 1.2,
                                                 "dropped": 24, "post_perturb_3s": 26}, \
            st["runs"]["runP"]["perturb"]
        assert st["args"]["perturb_margin_s"] == 0.3
        # Margin 0: exactly the overlap rule, t = 2.1 .. 4.1 (21).
        st0 = B.build_dataset([run], td / "ds0", horizon_s=1.0, stride_s=0.0, images=False,
                              perturb_margin_s=0.0)
        assert st0["dropped"]["perturbed"] == 21, st0["dropped"]
        # An unperturbed recording loses nothing to the rule.
        clean = _write_run(td, "runC", _straight(), seconds=8.0)
        stc = B.build_dataset([clean], td / "dsc", horizon_s=1.0, stride_s=0.0, images=False)
        assert "perturbed" not in stc["dropped"]
        assert stc["runs"]["runC"]["perturb"] == {"intervals": 0, "active_s": 0.0, "dropped": 0,
                                                  "post_perturb_3s": 0}


def test_each_run_reports_its_phrases_and_its_turn_histogram():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        a = _write_run(td, "runA", _straight())

        def right(k, t, yaw_at):
            return {"det": _det_at_bearing(30.0), "t_box": t - 0.1, "yaw_box": yaw_at(t - 0.1)}

        b = _write_run(td, "runR", _turning(speed=4.0, rate=0.35), box=right)
        st = B.build_dataset([a, b], td / "ds", horizon_s=1.0, stride_s=0.5, images=False)
        ra, rr = st["runs"]["runA"], st["runs"]["runR"]
        assert ra["phrase_counts"] == {C.STRAIGHT_AHEAD: ra["samples"]}, ra["phrase_counts"]
        assert rr["phrase_counts"] == {C.FORWARD_RIGHT: rr["samples"]}, rr["phrase_counts"]
        assert ra["abs_dpsi_hist"] == {"0.00-0.05": ra["samples"]}, ra["abs_dpsi_hist"]
        assert rr["abs_dpsi_hist"] == {"0.30-0.50": rr["samples"]}, rr["abs_dpsi_hist"]
        assert list(B.DPSI_BANDS)[0] == "0.00-0.05" and list(B.DPSI_BANDS)[-1] == "0.80-1.10"
        assert st["abs_dpsi_hist"]["train"] == {"0.00-0.05": ra["samples"],
                                                "0.30-0.50": rr["samples"]}


def test_mirror_maps_the_vocabulary_onto_itself_and_matches_the_mirrored_bearing():
    B = _B()
    from vla_bridge import bearing_to_phrase
    m = B.MIRROR_PHRASE
    assert set(m) == set(C.PHRASES) and set(m.values()) == set(C.PHRASES)
    for p in C.PHRASES:
        assert m[m[p]] == p, p
    assert m[C.STRAIGHT_AHEAD] == C.STRAIGHT_AHEAD and m[C.NO_HINT] == C.NO_HINT
    assert (m[C.FORWARD_LEFT], m[C.LEFT], m[C.LEFT_REAR]) == (C.FORWARD_RIGHT, C.RIGHT, C.RIGHT_REAR)
    # Against the bridge's own band chain over the whole circle (band edges
    # and +-180 excluded: the chain wraps 180 to the left rear).
    for k in range(1440):
        deg = -179.9 + 0.25 * k
        assert B.mirror_phrase(bearing_to_phrase(math.radians(deg))) == \
            bearing_to_phrase(math.radians(-deg)), deg
    try:
        B.mirror_phrase("straight ahead")                  # no trailing space: not a phrase
    except ValueError:
        pass
    else:
        raise AssertionError("an off-vocabulary phrase was mirrored")


def test_a_mirrored_sample_negates_the_turn_mirrors_the_phrase_and_is_marked():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)

        def right(k, t, yaw_at):
            return {"det": _det_at_bearing(30.0), "t_box": t - 0.1, "yaw_box": yaw_at(t - 0.1)}

        a = _write_run(td, "runR", _turning(speed=4.0, rate=0.35), box=right)
        h = _write_run(td, "runH", _turning(speed=4.0, rate=0.35), box=right)
        st = B.build_dataset([a, h], td / "ds", horizon_s=1.0, stride_s=0.5, images=False,
                             holdout=["runH"], mirror=True)
        rows = [json.loads(ln) for ln in (td / "ds" / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        src = {r["id"]: r for r in rows if not r["mirrored"]}
        mir = [r for r in rows if r["mirrored"]]
        n_real = sum(1 for r in src.values() if r["split"] == "train")
        assert mir and len(mir) == n_real, (len(mir), n_real)
        assert all(r["split"] == "train" and r["run"] == "runR" for r in mir), "hold-out stays real"
        assert st["mirror"] == {"enabled": True, "samples": len(mir), "splits": ["train"]}
        assert st["splits"]["train"] == 2 * n_real and st["splits"]["train_real"] == n_real
        for r in mir:
            s = src[r["mirror_of"]]
            assert r["id"] == s["id"] + "/mirror"
            assert s["phrase"] == C.FORWARD_RIGHT and r["phrase"] == C.FORWARD_LEFT
            assert r["prompt"] == C.build_prompt(r["phrase"], r["object"])
            assert r["prompt_sha256"] == C.prompt_sha256(r["prompt"])
            assert r["label"]["dpsi_rad"] == -s["label"]["dpsi_rad"]
            assert r["label"]["fwd_m"] == s["label"]["fwd_m"]
            assert r["label"]["down_m"] == s["label"]["down_m"]
            want = C.step_bins(s["label"]["fwd_m"], s["label"]["down_m"], -s["label"]["dpsi_rad"])
            assert tuple(r["bins"]) == want and r["bins"][:2] == s["bins"][:2], (r["bins"], s["bins"])
            assert s["bins"][2] > 49 > r["bins"][2]
            p = C.parse_action(r["action_text"] + C.EOS_TEXT, require_eos=True)
            assert p.parse_ok and list(p.bins) == r["bins"]
            assert math.isclose(r["box"]["bearing_deg"], -s["box"]["bearing_deg"])
            assert math.isclose(r["box"]["det"][0], s["box"]["det"][5] - s["box"]["det"][0])
            assert r["mirror"] == "horizontal flip of the front-over-down mosaic"
        assert all(r["mirrored"] is False for r in src.values())
        assert st["phrase_counts"] == {C.FORWARD_RIGHT: len(src), C.FORWARD_LEFT: len(mir)}
        assert st["phrase_counts_real"] == {C.FORWARD_RIGHT: len(src)}


def test_the_mirrored_mosaic_is_the_source_flipped_left_to_right():
    Image = _pil()
    if Image is None:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    from PIL import ImageOps
    B = _B()

    def jpeg(k):
        out = []
        for col in ((200, 40, 40 + k), (30, 160, 30)):
            im = Image.new("RGB", (256, 256), col)
            for y in range(0, 256, 4):
                im.putpixel(((y * 5 + k) % 128, y), (255, 255, 0))     # left half only
            buf = io.BytesIO()
            im.save(buf, format="JPEG", quality=95)
            out.append(buf.getvalue())
        return tuple(out)

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = _write_run(td, "runM", _straight(), seconds=4.0, jpeg=jpeg)
        B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.5, images=True, mirror=True)
        rows = [json.loads(ln) for ln in (td / "ds" / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        src = {r["id"]: r for r in rows if not r["mirrored"]}
        mir = [r for r in rows if r["mirrored"]]
        assert mir
        for r in mir:
            s = src[r["mirror_of"]]
            a = Image.open(td / "ds" / s["image"]).convert("RGB")
            b = Image.open(td / "ds" / r["image"]).convert("RGB")
            assert b.size == C.MOSAIC_SIZE
            assert C.mosaic_sha256(ImageOps.mirror(a)) == r["mosaic_sha256"] == C.mosaic_sha256(b)
            assert r["mosaic_sha256"] != s["mosaic_sha256"]


def _FT():
    if str(ROOT / "training") not in sys.path:
        sys.path.insert(0, str(ROOT / "training"))
    import finetune_aerialvla_follow as FT
    return FT


def _box_right(k, t, yaw_at):
    return {"det": _det_at_bearing(30.0), "t_box": t - 0.1, "yaw_box": yaw_at(t - 0.1)}


def test_the_builders_samples_are_what_the_trainer_reads():
    """The builder (FT-D) and the trainer (FT-T) were written apart; the
    first real dataset (follow_v2, 23 Oct) was refused whole by the trainer's
    dry run: 6051 of 6051 lines 'no_target' (the builder writes the answer as
    action_text). Every row, real, mirrored and held out, must load as the
    trainer's Sample with the builder's own prompt, text and bins."""
    FT = _FT()
    B = _B()
    right = _box_right
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        a = _write_run(td, "runR", _turning(speed=4.0, rate=0.35), box=right)
        s = _write_run(td, "runS", _straight(speed=4.0))
        h = _write_run(td, "runH", _turning(speed=4.0, rate=0.35), box=right)
        B.build_dataset([a, s, h], td / "ds", horizon_s=1.0, stride_s=0.5, images=False,
                        holdout=["runH"], mirror=True)
        rows = [json.loads(ln) for ln in (td / "ds" / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        assert {r["split"] for r in rows} == {"train", "holdout"}
        assert any(r["mirrored"] for r in rows) and any(r["phrase"] != C.STRAIGHT_AHEAD
                                                         for r in rows)
        for i, r in enumerate(rows):
            r = dict(r, image="images/placeholder.png")        # images=False leaves it None
            smp = FT.check_sample(r, td / "ds", i)
            assert smp.prompt == r["prompt"] and smp.target == r["action_text"], r["id"]
            assert list(smp.bins) == r["bins"] and smp.run == r["run"], r["id"]
            assert smp.split == r["split"] and smp.phrase == r["phrase"], r["id"]


def test_a_dataset_built_with_mosaics_loads_whole_in_the_trainer():
    """The same join with the images written: the trainer's loader (which
    opens every mosaic) keeps every line the builder wrote."""
    Image = _pil()
    if Image is None:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    FT = _FT()
    B = _B()
    right = _box_right
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)

        def jpeg(k):
            out = []
            for col in ((200, 40, 40 + k % 200), (30, 160, 30)):
                buf = io.BytesIO()
                Image.new("RGB", (256, 256), col).save(buf, format="JPEG", quality=90)
                out.append(buf.getvalue())
            return tuple(out)

        runs = [_write_run(td / "img", tag, motion, box=right, jpeg=jpeg)
                for tag, motion in (("runR", _turning(speed=4.0, rate=0.35)),
                                    ("runH", _turning(speed=4.0, rate=0.35)))]
        B.build_dataset(runs, td / "ds_img", horizon_s=1.0, stride_s=0.5, images=True,
                        holdout=["runH"], mirror=True)
        kept, rep = FT.load_samples(td / "ds_img", check_images=True)
        assert rep["kept"] == rep["lines"] == len(kept) > 0 and not rep["dropped"], rep


def test_the_launcher_switch_perturbs_only_a_recording():
    src = PS1.read_text(encoding="utf-8")
    assert "[switch]$RecordPerturb" in src and "[switch]$PerturbSpeed" in src
    lines = src.splitlines()
    assert "[switch]$PerturbReplaceYaw" in src and "[string]$PerturbYawRate" in src
    for flag, guard in (('"--record-perturb"', "if ($RecordPerturb)"),
                        ('"--perturb-speed"', "if ($PerturbSpeed)"),
                        ('"--perturb-replace-yaw"', "if ($PerturbReplaceYaw)"),
                        ('"--perturb-yaw-rate"', "if ($PerturbYawRate)")):
        assert src.count(flag) == 1, flag
        fi = next(n for n, ln in enumerate(lines) if flag in ln)
        gi = max(n for n, ln in enumerate(lines[:fi]) if ln.lstrip().startswith(guard))
        vi = max(n for n, ln in enumerate(lines[:fi]) if ln.lstrip().startswith("if ($RecordVla)"))
        assert vi < gi, f"{flag} is not inside the -RecordVla block"
        for start in (gi, vi):
            ind = len(lines[start]) - len(lines[start].lstrip())
            for ln in lines[start + 1:fi + 1]:
                assert not ln.strip() or len(ln) - len(ln.lstrip()) > ind, (flag, ln)
    # Refused before the simulator starts: a perturbation needs a recording.
    i = src.index("if ($RecordPerturb -and -not $RecordVla)")
    assert i < src.index("if (-not $SkipSim) { Start-Sim }")
    assert src.index("if ($PerturbSpeed -and -not $RecordPerturb)") < \
        src.index("if (-not $SkipSim) { Start-Sim }")
    assert src.index("($PerturbReplaceYaw -or $PerturbYawRate) -and -not $RecordPerturb") < \
        src.index("if (-not $SkipSim) { Start-Sim }")
    import follow_vlm as F
    a = F.build_arg_parser().parse_args(["--record-vla", "d", "--record-perturb", "--perturb-speed"])
    assert a.record_perturb and a.perturb_speed


# --------------------------------------------------------------------------- #
# runner
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
