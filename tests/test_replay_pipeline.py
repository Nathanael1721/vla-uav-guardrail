"""The pipeline replay: tools/replay_pipeline.py.

Run either way:
    pytest tests/test_replay_pipeline.py -v
    python tests/test_replay_pipeline.py

Two of its numbers were wrong in review (2026-09-30), and these pin the
fixes on synthetic data. The pooled rates were rebuilt from per-flight
fractions rounded to 3 places; they are now the summed integers. The "old"
arm reset its estimator after 3 s and re-seeded on any box, which the
flown code never did: it is now the estimator as flown, never reset, and
what it reports is the first box it ACCEPTED after a gap. The tests that
need demo/follow_vlm.py (the vla-real env) or the flights under demo/out
SKIP without them rather than pass.
"""
import json
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))
sys.path.insert(0, str(ROOT / "tools"))

import replay_pipeline as rp                                 # noqa: E402

SKIP = "SKIP"
W = 768
V_CAR = 3.2                      # m/s, the level car's cruise


def _fv():
    try:
        import follow_vlm
        return follow_vlm
    except Exception:                                        # noqa: BLE001
        return None


# ---------------------------------------------------------------------------
# pooling
# ---------------------------------------------------------------------------
def _flight(ticks, served, on, **extra):
    return rp.rates({"n_ticks": ticks, "n_served": served, "n_on": on, **extra})


def test_pooling_sums_the_integers():
    """Two flights of very different length: the pool is the ratio of the
    summed counts, not a mean of the flights' fractions."""
    per = {"a": {"old": _flight(3, 1, 1, post_gap_accepts=2),
                 "old_as_flown": _flight(3, 0, 0, post_gap_accepts=1),
                 "new": _flight(3, 0, 0, seeds=0)},
           "b": {"old": _flight(7, 7, 0, post_gap_accepts=1),
                 "old_as_flown": _flight(7, 2, 2, post_gap_accepts=1),
                 "new": _flight(7, 4, 4, seeds=1)}}
    p = rp.pool(per)
    assert (p["old"]["n_ticks"], p["old"]["n_served"], p["old"]["n_on"]) == (10, 8, 1)
    assert p["old"]["served_frac"] == 8 / 10
    assert p["old"]["on_frac_of_served"] == 1 / 8
    assert p["old"]["on_frac_of_all"] == 1 / 10
    assert p["old"]["post_gap_accepts"] == 3
    assert p["new"]["on_frac_of_served"] == 1.0 and p["new"]["seeds"] == 1
    # the as-flown arm is pooled the same way, and apart from "old"
    assert (p["old_as_flown"]["n_served"], p["old_as_flown"]["post_gap_accepts"]) == (2, 2)
    assert p["old_as_flown"]["served_frac"] == 2 / 10


def test_pooling_does_not_drift_with_the_display_rounding():
    """What the old pooling did: round(frac, 3) * ticks. Over 300 flights of
    3 ticks with 1 served it pools 0.333 - the integers say 1/3 exactly."""
    per = {f"f{i}": {"old": _flight(3, 1, 1), "new": _flight(3, 1, 0)}
           for i in range(300)}
    p = rp.pool(per)
    assert p["old"]["n_served"] == 300 and p["old"]["n_ticks"] == 900
    assert p["old"]["served_frac"] == 300 / 900
    rounded = sum(round(r["old"]["served_frac"], 3) * 3 for r in per.values()) / 900
    assert rounded != p["old"]["served_frac"]                # the defect it replaces


def test_pooling_leaves_lists_floats_and_flags_alone():
    per = {"a": {"old": _flight(2, 2, 2, false_post_gap_m=[12.0], start_t=1.5),
                 "new": _flight(2, 0, 0), "flown_check": {"rows": 5, "agree": 5}},
           "b": {"old": _flight(2, 2, 2, false_post_gap_m=[20.0], start_t=None),
                 "new": _flight(2, 0, 0), "flown_check": {"rows": 5, "agree": 4}}}
    p = rp.pool(per)
    assert "false_post_gap_m" not in p["old"] and "start_t" not in p["old"]
    assert p["flown_check"] == {"rows": 10, "agree": 9}
    assert p["new"]["on_frac_of_served"] is None             # nothing served: no rate
    assert rp.reproduced(per["a"]) and not rp.reproduced(per["b"])


# ---------------------------------------------------------------------------
# the old arm
# ---------------------------------------------------------------------------
def _truth(t):
    return (30.0 + V_CAR * t, 0.0)


def _box(t, offset_m=0.0, blocked=False):
    """A box on the car (or `offset_m` to its left), seen from the origin
    facing north, as the flight fed it: the column through the LINEAR map."""
    n, e = _truth(t)
    e += offset_m
    b = math.atan2(e, n)
    cx = W / 2.0 + (b / math.radians(45.0)) * (W / 2.0)
    return {"box": {"cx": cx, "w": 40.0}, "W": W, "r": math.hypot(n, e),
            "r_src": "depth", "blocked": blocked}


def _fly(arm, box_at, t_end):
    """10 Hz ticks from 0 to t_end; `box_at(k)` gives the box consumed on
    tick k, or None. Returns the ticks on which the estimate was served."""
    served = []
    for k in range(int(round(t_end / 0.1)) + 1):
        t = round(0.1 * k, 3)
        row = {"t": t, "x": 0.0, "y": 0.0, "psi": 0.0}
        bx = box_at(k)
        if arm.row(row, [bx] if bx is not None else [], _truth):
            served.append(t)
    return served


def test_the_old_arm_counts_the_first_accept_after_a_gap_and_never_resets():
    """5 s on the car at ~3 Hz, 5 s with nothing, then a box 12 m to the
    car's left. The flown estimator has stopped predicting at 3 s, so that
    update predicts the last 2 s in one step and its gate opens to take the
    box: a FALSE post-gap accept. The first-ever accept counts too (on the
    car); none of the in-stream updates do; nothing was reset."""
    def box_at(k):
        if k <= 50 and k % 3 == 0:
            return _box(0.1 * k)
        if k == 100:
            return _box(10.0, offset_m=12.0)
        return None
    arm = rp.FlownArm()
    served = _fly(arm, box_at, 12.0)
    assert arm.est.n_resets == 0
    assert len(arm.post_gap) == 2, arm.post_gap
    (t1, _z1, d1), (t2, _z2, d2) = arm.post_gap
    assert t1 == 0.0 and d1 < 1.0, arm.post_gap
    assert t2 == 10.0 and d2 > rp.FALSE_SEED_M, arm.post_gap
    # served while younger than 3 s: 0.0-7.8 s (last on-car box at 4.8 s),
    # then again from 10.0 s
    assert 7.8 in served and 7.9 not in served and 10.0 in served, served


def test_a_box_inside_the_coast_window_is_not_a_post_gap_accept():
    def box_at(k):
        if k <= 30 and k % 3 == 0:
            return _box(0.1 * k)
        if k == 55:                                          # 2.5 s after the last
            return _box(5.5)
        return None
    arm = rp.FlownArm()
    _fly(arm, box_at, 6.0)
    assert len(arm.post_gap) == 1 and arm.est.n_updates == 12, (arm.post_gap,
                                                                arm.est.n_updates)


def test_the_presence_gate_is_honoured_only_by_the_as_flown_check():
    boxes = {k: _box(0.1 * k, blocked=(k % 6 == 0)) for k in range(0, 31, 3)}
    feed_all, as_flown = rp.FlownArm(), rp.FlownArm(honour_gate=True)
    _fly(feed_all, boxes.get, 3.0)
    _fly(as_flown, boxes.get, 3.0)
    assert feed_all.est.n_updates == 11 and as_flown.est.n_updates == 5


def test_the_old_arm_feeds_the_flown_linear_bearing_and_width_fallback():
    assert rp.flown_bearing(W / 2.0, W) == 0.0
    assert abs(rp.flown_bearing(W, W) - math.radians(45.0)) < 1e-12
    # A 4 m car 96 px wide in 768 px spans 11.25 deg: 2 / tan(5.625 deg) m.
    r = rp.flown_width_range(96.0, W)
    assert abs(r - 2.0 / math.tan(math.radians(5.625))) < 1e-9, r
    # no depth and not blocked: the flight fell back to the width
    arm = rp.FlownArm()
    bx = dict(_box(0.0), r=None, r_src=None)
    arm.row({"t": 0.0, "x": 0.0, "y": 0.0, "psi": 0.0}, [bx], _truth)
    assert arm.est.n_updates == 1
    # blocked and the range lost: the flight never fed it, and nor does this
    arm = rp.FlownArm()
    arm.row({"t": 0.0, "x": 0.0, "y": 0.0, "psi": 0.0},
            [dict(bx, r_src="unlogged")], _truth)
    assert arm.est.n_updates == 0


def test_the_old_arm_is_the_vendored_estimator_byte_for_byte():
    """In-place predict (no posterior kept), and the body is exactly the
    commit's file after the header - so it cannot drift by an edit."""
    assert not hasattr(rp.FlownTargetState(), "_xu")
    assert hasattr(rp.TargetState(), "_xu")
    try:
        blob = subprocess.run(["git", "show", "1d09786:demo/target_state.py"],
                              cwd=ROOT, capture_output=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return SKIP
    body = (ROOT / "tools" / "_target_state_1d09786.py").read_bytes()
    assert body.endswith(blob) and body[:-len(blob)].count(b"\n") == 16
    assert all(line.startswith(b"#") for line in body[:-len(blob)].splitlines())


# ---------------------------------------------------------------------------
# the new arm's gates (needs follow_vlm: vla-real)
# ---------------------------------------------------------------------------
def test_the_cli_defaults_are_the_runner_start_gate():
    cfg = rp.config(rp.parse_args([]))
    assert (cfg["start_need"], cfg["start_max_range_m"]) == (5, 30.0)
    assert (cfg["reacq_need"], cfg["reacq_max_range_m"]) == (4, 45.0)
    cfg = rp.config(rp.parse_args(["--start-need", "3", "--start-max-range-m", "0",
                                   "--reacq-need", "6", "--reacq-max-range-m", "50"]))
    assert (cfg["start_need"], cfg["start_max_range_m"],
            cfg["reacq_need"], cfg["reacq_max_range_m"]) == (3, 0.0, 6, 50.0)


def test_the_first_gate_is_the_start_gate_and_a_lapse_anchors_on_the_posterior():
    fv = _fv()
    if fv is None:
        return SKIP
    import identity
    cfg = rp.config(rp.parse_args([]))
    arm = rp.NewArm(identity.load_thresholds(), cfg, fv)
    assert isinstance(arm.reacq, fv.Reacquirer)
    assert (arm.reacq.need, arm.reacq.max_range_m) == (5, 30.0)
    # a start gate with no range of its own takes the re-acquisition range
    arm0 = rp.NewArm(identity.load_thresholds(),
                     dict(cfg, start_max_range_m=0.0), fv)
    assert arm0.reacq.max_range_m == 45.0
    # hold a subject, then let it lapse
    arm.reacq = None
    arm.est.update(10.0, 0.0, 0.0, 0.0, 0.0, 30.0)
    arm.committed, arm.t_meas_last, arm.last_acc = True, 10.4, (99.0, 99.0)
    arm.tick(13.4)
    assert arm.reacq is None                                 # 3.0 s: not yet
    arm.tick(13.5)
    assert (arm.reacq.need, arm.reacq.max_range_m) == (4, 45.0)
    assert arm.reacq.anchor["P"] == (30.0, 0.0), arm.reacq.anchor   # the posterior
    assert arm.reacq.anchor["t"] == 10.0 and arm.n_lapses == 1
    assert arm.est._xu is None and arm.served(13.5) is None


def test_a_recorded_flight_the_old_arm_reproduces():
    """The docstring's claim, on a flight flown from the tree committed as
    1d09786: the old arm with the presence gate honoured reproduces the
    flight's own est.served on >= 99 % of rows, and the report is JSON."""
    fv = _fv()
    d = ROOT / "demo" / "out" / "citylife_redcar_final1"
    if fv is None or not (d / "flight_log.jsonl").is_file():
        return SKIP
    import identity
    cfg = rp.config(rp.parse_args([]))
    r = rp.replay_flight(d, identity.load_thresholds(), identity.StreetDistance.load(),
                         cfg, fv)
    ch = r["flown_check"]
    assert rp.reproduced(r), ch
    for arm in ("old", "new", "old_as_flown"):
        c = r[arm]
        assert 0 <= c["n_on"] <= c["n_served"] <= c["n_ticks"], c
    assert r["old"]["n_ticks"] == r["new"]["n_ticks"] == r["old_as_flown"]["n_ticks"] > 0
    assert r["old"]["resets"] == r["old_as_flown"]["resets"] == 0
    assert r["new"]["seeds"] == r["new"]["seeds_start"] + r["new"]["seeds_reacq"]
    # final1 had 115 presence-blocked boxes, all with a logged range: "old"
    # feeds them and the flight did not, so the two arms must differ here
    assert r["data"]["presence_blocked"] > r["data"]["range_unlogged"]
    assert r["old"]["n_served"] != r["old_as_flown"]["n_served"], (r["old"],
                                                                   r["old_as_flown"])
    json.dumps(r, allow_nan=False)
    res = {d.name: r, "_pooled": rp.pool({d.name: r}), "_reproduced_flights": [d.name],
           "_config": cfg, "_thresholds_hash": "test"}
    res["_pooled_reproduced"] = res["_pooled"]
    md = rp.markdown(res)
    assert "as flown: served" in md and "**pooled, reproduced flights** (1)" in md


def test_main_pools_the_flights_and_never_its_own_summary_row():
    """main() once pooled into `res` and then picked the reproduced flights
    out of the same dict: when every flight reproduced, so did the
    '_pooled' row, and '_pooled_reproduced' counted every integer twice
    (re-review, 2026-09-30). Run main() on two stub flights that both
    reproduce, with the model, map and markdown stubbed out."""
    import tempfile
    import types

    per = {"f1": {"old": _flight(3, 1, 1), "old_as_flown": _flight(3, 1, 1),
                  "new": _flight(3, 1, 1, seeds=1), "flown_check": {"rows": 3, "agree": 3}},
           "f2": {"old": _flight(5, 2, 0), "old_as_flown": _flight(5, 2, 0),
                  "new": _flight(5, 0, 0, seeds=0), "flown_check": {"rows": 5, "agree": 5}}}
    saved = (rp.ri.flight_dirs, rp.replay_flight, rp.identity.load_thresholds,
             rp.identity.StreetDistance.load, rp.identity.thresholds_hash, rp.markdown,
             sys.modules.get("follow_vlm"))
    try:
        rp.ri.flight_dirs = lambda: [Path(k) for k in per]
        rp.replay_flight = lambda d, th, sd, cfg, fv: per[d.name]
        rp.identity.load_thresholds = lambda *a, **k: {}
        rp.identity.StreetDistance.load = staticmethod(lambda *a, **k: None)
        rp.identity.thresholds_hash = lambda th: "test"
        rp.markdown = lambda res: ""
        sys.modules["follow_vlm"] = types.ModuleType("follow_vlm")
        with tempfile.TemporaryDirectory() as tmp:
            assert rp.main(["--out", tmp]) == 0
            res = json.loads((Path(tmp) / "pipeline.json").read_text(encoding="utf-8"))
    finally:
        (rp.ri.flight_dirs, rp.replay_flight, rp.identity.load_thresholds,
         rp.identity.StreetDistance.load, rp.identity.thresholds_hash, rp.markdown,
         fv) = saved
        if fv is None:
            sys.modules.pop("follow_vlm", None)
        else:
            sys.modules["follow_vlm"] = fv
    assert res["_reproduced_flights"] == ["f1", "f2"], res["_reproduced_flights"]
    assert res["_pooled_reproduced"] == res["_pooled"]
    assert res["_pooled"]["old"]["n_ticks"] == 8 and res["_pooled"]["new"]["seeds"] == 1


if __name__ == "__main__":
    # A test that short-circuits on a missing fixture must NOT print PASS.
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs vla-real / demo/out)")
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
