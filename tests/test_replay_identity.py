"""The offline identity replay: tools/replay_identity.py.

Run either way:
    pytest tests/test_replay_identity.py -v
    python tests/test_replay_identity.py

The replay's numbers are only as good as its join and its labels. These
tests write a tiny synthetic flight in the logs' own format and check four
things. The join picks the first tick that used each detection. Pose and
truth come from the moment the frame was CAPTURED; the aircraft yaws at
0.5 rad/s here, so the tick pose is 4.3 deg off at 40 m. A blocked box's
range is recovered from gate_why, and where the log lost it, the box stays
out of the tuning. Finally, the vectorised grid count agrees with
identity.classify box for box.
"""
import json
import math
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests"))

import identity                                              # noqa: E402
import replay_identity as R                                  # noqa: E402
from test_identity import W, H, box_of                       # noqa: E402

T0 = 1_790_000_000.0
INFER_S = 0.10
YAW_RATE = 0.5
CAR = (40.0 * math.cos(0.3), 40.0 * math.sin(0.3))           # 40 m, 17 deg
KINDS = ["on", "on", "on", "off", "blocked", "miss", "amb", "on", "on", "on"]
BLOCK_WHY = "implies 0.6 m wide at 29 m; a car is 1.0-8.0 m"


def _pose(t, attitude):
    p, r = (math.radians(3.0), math.radians(-2.0)) if attitude else (0.0, 0.0)
    return {"x": 0.0, "y": 0.0, "up": 8.0, "yaw": 0.1 + YAW_RATE * t,
            "pitch": p, "roll": r}


def make_flight(root, name, attitude=True, gate_log=True):
    """A hovering aircraft yawing at 0.5 rad/s, a parked red car 40 m out,
    ten detector publications 0.3 s apart (and one warm-up record no tick
    ever used), and control ticks every 0.1 s."""
    d = Path(root) / name
    d.mkdir(parents=True)
    recs = [{"seq": 1, "t": T0 - 5.0, "infer_ms": 100.0,
             "det": {"cx": 10, "cy": 10, "w": 5, "h": 5, "score": 0.1,
                     "colour": 0.3, "img_w": W, "img_h": H}}]
    pubs = []
    for k, kind in enumerate(KINDS):
        seq, tp = k + 2, 0.25 + 0.3 * k
        tcap = tp - INFER_S
        box, r = None, None
        if kind in ("on", "amb"):
            box, r = box_of(_pose(tcap, attitude), CAR[0], CAR[1], 1.8, 0.0, 1.5)
            if kind == "amb":
                r += 30.0                           # right bearing, wrong range
        elif kind == "off":
            box, r = {"cx": 700.0, "cy": 250.0, "w": 20.0, "h": 30.0,
                      "score": 0.05, "colour": 0.4}, 20.0
        elif kind == "blocked":
            box = {"cx": 100.0, "cy": 200.0, "w": 12.0, "h": 20.0,
                   "score": 0.03, "colour": 0.4}
        det = None if box is None else {**box, "img_w": W, "img_h": H}
        recs.append({"seq": seq, "t": T0 + tp, "infer_ms": 100.0,
                     "query": "a red car", "det": det, "n_cands": 1,
                     "acquiring": False, "cands": None, "runner_up": None})
        pubs.append((seq, tp, det, r, kind))

    rows, used = [], set()
    for i in range(41):
        t = 0.1 * i
        live = [p for p in pubs if p[1] <= t + 1e-9]
        row = {"t": t, "tick": i + 1, "x": 0.0, "y": 0.0, "up": 8.0,
               "psi": _pose(t, attitude)["yaw"],
               "truth": {"class": "car", "pts": [list(CAR)], "names": ["Car_10"]}}
        if attitude:
            row["pitch_deg"], row["roll_deg"] = 3.0, -2.0
        if live:
            seq, tp, det, r, kind = live[-1]
            found = [p for p in live if p[2] is not None][-1]
            row["det_seq"] = seq
            row["det_age_s"] = t - found[1]
            first = seq not in used
            used.add(seq)
            if kind == "blocked":
                row["presence_blocked"] = True
                row["rng_m"] = None
                if gate_log:
                    row["gate_why"] = BLOCK_WHY
            else:
                row["presence_blocked"] = False
                # a later tick re-reads depth for the same box: make it
                # obviously different so a wrong join shows
                row["rng_m"] = None if r is None else (r if first else r + 50.0)
        rows.append(row)
    for fname, items in (("flight_log.jsonl", rows), ("detections.jsonl", recs)):
        with (d / fname).open("w", encoding="utf-8", newline="\n") as fh:
            for it in items:
                fh.write(json.dumps(it) + "\n")
    return d


class TmpDir:
    def __enter__(self):
        self.p = Path(tempfile.mkdtemp(prefix="replay_identity_"))
        return self.p

    def __exit__(self, *a):
        shutil.rmtree(self.p, ignore_errors=True)


def by_kind(samples):
    seq_kind = {k + 2: kind for k, kind in enumerate(KINDS)}
    return [(seq_kind[s["seq"]], s) for s in samples]


def test_a_flight_flown_with_identity_is_not_evidence_for_it():
    """citylife_redcar_id1..4 (2026-09-30) matched the glob and entered both
    replays: their boxes were chosen by the rules under test."""
    with tempfile.TemporaryDirectory() as t:
        root = Path(t)
        for name, metrics in (("citylife_redcar_old", {"det_hz": 3.5}),
                              ("citylife_redcar_off", {"identity": {"enabled": False}}),
                              ("citylife_redcar_id9", {"identity": {"candidates": {"ok": 1}}}),
                              ("citylife_redcar_nometrics", None)):
            d = root / name
            d.mkdir()
            for f in ("flight_log.jsonl", "detections.jsonl"):
                (d / f).write_text("", encoding="utf-8")
            if metrics is not None:
                (d / "metrics.json").write_text(json.dumps(metrics), encoding="utf-8")
        got = [d.name for d in R.flight_dirs(root)]
        assert got == ["citylife_redcar_nometrics", "citylife_redcar_off",
                       "citylife_redcar_old"], got


def test_parse_gate_range():
    assert R.parse_gate_range(BLOCK_WHY) == 29.0
    assert R.parse_gate_range("not on the ground: its base ray meets the road "
                              "at 91 m, it is 57 m away") == 57.0
    assert R.parse_gate_range("implies 12.5 m wide at 7.5 m; a car is 1.0-8.0 m") == 7.5
    assert R.parse_gate_range("colour 0.10 < 0.15") is None
    assert R.parse_gate_range("box is 98% of frame - a wall") is None
    assert R.parse_gate_range(None) is None


def test_join_takes_the_first_tick_and_drops_unused_records():
    with TmpDir() as tmp:
        d = make_flight(tmp, "citylife_redcar_synth")
        rows, dets = R.load_flight(d)
        pairs = R.join_detections(rows, dets)
        seqs = [rec["seq"] for rec, _ in pairs]
        assert 1 not in seqs                        # warm-up, no tick used it
        assert 7 not in seqs                        # the miss has no box
        assert len(pairs) == len(KINDS) - 1
        n_multi = 0
        for rec, row in pairs:
            same = sorted((r for r in rows if r.get("det_seq") == rec["seq"]),
                          key=lambda r: r["t"])
            assert row["det_age_s"] == min(r["det_age_s"] for r in same)
            assert row is same[0]                   # the first tick that used it
            if len(same) > 1 and same[0]["rng_m"] is not None:
                assert same[1]["rng_m"] == same[0]["rng_m"] + 50.0   # fixture
                n_multi += 1
        assert n_multi >= 5
        assert abs(R.clock_offset(pairs) - T0) < 1e-3


def test_labels_on_a_synthetic_flight():
    with TmpDir() as tmp:
        d = make_flight(tmp, "citylife_redcar_synth")
        fl = R.build_samples(d, None, cap_lag=0.0)
        assert fl["has_attitude"] and abs(fl["offset"] - T0) < 1e-3
        for kind, s in by_kind(fl["samples"]):
            if kind == "on":
                assert s["label"] == "on", (s["seq"], s["info"])
                assert abs(s["info"]["bearing_err_deg"]) < 0.05, s["info"]
                assert s["r_src"] == "depth" and R.usable(s)
            elif kind == "off":
                assert s["label"] == "off", s["info"]
            elif kind == "amb":
                assert s["label"] == "amb", s["info"]
                assert not R.usable(s)
            elif kind == "blocked":
                assert s["r"] == 29.0 and s["r_src"] == "gate_why"
                assert s["label"] == "off" and R.usable(s)
                assert abs(s["f"]["width_m"] - 29.0 * identity.cam.box_angular_width(
                    100.0, 12.0, W, 90.0)) < 1e-9


def test_pose_and_truth_are_taken_at_capture_not_at_the_tick():
    """The first tick that used a box comes 0.05 s after publication, which
    is 0.15 s after capture: 4.3 deg of yaw at 0.5 rad/s. At 40 m the on-car
    tolerance is 3.6 deg, so the tick pose would not call it on-car."""
    with TmpDir() as tmp:
        d = make_flight(tmp, "citylife_redcar_synth")
        rows, dets = R.load_flight(d)
        fl = R.build_samples(d, None, cap_lag=0.0, rows=rows, dets=dets)
        pairs = {rec["seq"]: row for rec, row in R.join_detections(rows, dets)}
        n = 0
        for kind, s in by_kind(fl["samples"]):
            if kind != "on":
                continue
            db = math.degrees(s["f"]["bearing"] - s["fj"]["bearing"])
            assert abs(abs(db) - math.degrees(YAW_RATE * 0.15)) < 0.05, db
            rec = next(r for r in dets if r["seq"] == s["seq"])
            lab, _ = R.label_box(rec["det"], s["r"], R.row_pose(pairs[s["seq"]]),
                                 CAR, W, H)
            assert lab != "on"
            n += 1
        assert n == KINDS.count("on")


def test_inside_the_cars_extent_a_box_is_not_called_off_on_bearing_alone():
    """9 m away the on-car tolerance is atan(2.5/9) = 15.5 deg, far wider
    than the 8 deg off-car line. A box 10 deg off the car's centre with the
    car's range is on it; at 17 deg (past the tolerance, within 3 deg of
    it) it is ambiguous, not off-car, unless its range disagrees too."""
    pose = {"x": 0.0, "y": 0.0, "up": 8.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}
    car = (9.0, 0.0)
    b10, r = box_of(pose, 9.0, 9.0 * math.tan(math.radians(10.0)), 1.0, 0.0, 1.0)
    lab, info = R.label_box(b10, r, pose, car, W, H)
    assert abs(abs(info["bearing_err_deg"]) - 10.0) < 0.1, info
    assert abs(info["tol_deg"] - math.degrees(math.atan(2.5 / 9.0))) < 1e-9
    assert lab == "on", (lab, info)
    b17, r = box_of(pose, 9.0, 9.0 * math.tan(math.radians(17.0)), 1.0, 0.0, 1.0)
    lab, info = R.label_box(b17, r, pose, car, W, H)
    assert lab == "amb", (lab, info)
    lab, _ = R.label_box(b17, r + 30.0, pose, car, W, H)
    assert lab == "off"
    b25, r = box_of(pose, 9.0, 9.0 * math.tan(math.radians(25.0)), 1.0, 0.0, 1.0)
    assert R.label_box(b25, r, pose, car, W, H)[0] == "off"
    far = (60.0, 0.0)
    b9, r = box_of(pose, 60.0, 60.0 * math.tan(math.radians(9.0)), 1.0, 0.0, 1.0)
    assert R.label_box(b9, r, pose, far, W, H)[0] == "off"


def test_a_blocked_box_the_old_logs_lost_stays_out_of_tuning():
    with TmpDir() as tmp:
        d = make_flight(tmp, "citylife_redcar_old", attitude=False, gate_log=False)
        fl = R.build_samples(d, None, cap_lag=0.0)
        assert not fl["has_attitude"]
        for kind, s in by_kind(fl["samples"]):
            assert s["f"]["bottom_h"] is None and not s["f"]["pitch_measured"]
            if kind == "blocked":
                assert s["r"] is None and s["r_src"] == "unlogged"
                assert s["label"] == "off" and not R.usable(s)
            if kind == "on":
                assert s["label"] == "on"


def test_the_vectorised_grid_count_matches_classify():
    with TmpDir() as tmp:
        samples = []
        for name, att in (("citylife_redcar_a", True), ("citylife_redcar_b", False)):
            samples += R.build_samples(make_flight(tmp, name, attitude=att),
                                       None, cap_lag=0.0)["samples"]
        counts = R.grid_counts(samples, "f")
        tot = R._sum(counts, sorted(counts))
        shape = tot["on_notok"].shape
        for idx in ((0, 0, 0), tuple(s // 2 for s in shape),
                    tuple(s - 1 for s in shape)):
            th = {**identity.DEFAULTS, **R.combo_thresholds(idx)}
            ref = R.tally(samples, th, "f")
            vec = R.counts_at(tot, idx)
            for k in ("n_on", "n_off", "on_notok", "on_hard", "off_notok", "off_hard"):
                assert vec[k] == ref[k], (idx, k, vec[k], ref[k])


def test_the_whole_replay_runs_and_writes_its_outputs():
    with TmpDir() as tmp:
        make_flight(tmp, "citylife_redcar_a")
        make_flight(tmp, "citylife_redcar_b", attitude=False, gate_log=False)
        np.savez(tmp / "street.npz", street=np.ones((60, 60), dtype=np.uint8),
                 res=2.0, origin_x=-20.0, origin_y=-40.0)
        rc = R.main(["--root", str(tmp), "--out", str(tmp / "out"),
                     "--street", str(tmp / "street.npz"), "--cap-lag", "0",
                     "--thresholds-out", str(tmp / "th.json")])
        assert rc == 0
        rep = json.loads((tmp / "out" / "report.json").read_text(encoding="utf-8"))
        assert rep["pooled"]["n_on"] == 2 * KINDS.count("on")
        assert rep["pooled"]["on_notok"] == 0
        assert set(rep["loo"]["per_flight"]) == {"citylife_redcar_a",
                                                 "citylife_redcar_b"}
        assert (tmp / "out" / "report.md").read_text(encoding="utf-8").startswith("# ")
        th = json.loads((tmp / "th.json").read_text(encoding="utf-8"))
        assert th["tuned_on"] == ["citylife_redcar_a", "citylife_redcar_b"]
        assert identity.thresholds_hash(th) == th["hash"] == rep["thresholds_hash"]


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
