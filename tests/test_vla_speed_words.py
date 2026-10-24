"""
The follow fine-tune's proprioception (2026-10-24): the aircraft's own
horizontal speed, as words after the object ("a red car while flying fast").

Run either way:
    pytest tests/test_vla_speed_words.py -v
    python tests/test_vla_speed_words.py

WHAT IS AT STAKE

The prev_front follows (vla_follow_ft_05 / _06, decisions every ~0.45 s)
still answered "forward 0" for ~7 s when a car stopped ahead pulled away,
then chased 30-40 m behind. The expert flies at the target's speed; the
model sees one image (two with prev_front) and does not know its OWN
speed, so "the car stopped and I hover" and "the car moves and I fly at
its speed" look alike. Speed words give it that as text.

So, pinned here (default OFF; every existing dataset, prompt and test is
unchanged):

  A. ONE RULE (guardrail/aerialvla_codec.own_speed / own_speed_words /
     object_with_speed): a backward difference over 0.5 s of the pose
     history at or before the capture - never a later pose - and four word
     classes with bounds at 0.5 / 1.5 / 3.0 m/s; words only, accepted by
     the server's object check.
  B. The dataset builder (tools/build_follow_dataset.py --speed-words) and
     the flight (vla_mission.VLAFrames over SemanticObs's pose ring) give
     the same words for the same pose history; a capture with too little
     history is no sample and no request; the mirror keeps the words;
     stats.json counts the classes in all, per split, per run and per
     label-fwd band.
  C. PROVENANCE: the trainer reads the setting, refuses a dataset that
     mixes modes, and writes it into the checkpoint's provenance; the server
     announces it; vla_fly and the DAgger recorder REFUSE before take-off
     when the adapter's setting and the flight's differ, either way (an
     adapter without the field is off); the evaluation tool sends each
     sample's prompt as it is and says when the adapter differs.
  D. The request with speed words round-trips through the codec, the
     server's parser and the trainer's prompt reader byte for byte.

No simulator, GPU or model: synthetic recordings, SemanticObs fed by hand,
a stub client, tests/vla_fake_server.py on an ephemeral loopback port
(never 8790), the server's core with injected decoders. The evaluation
through the server's real mosaic needs Pillow and the training loop torch;
they SKIP without them (GUARDRAIL_REQUIRE_PILLOW=1 turns a missing Pillow
into a failure).

Shown failing first: at 3147b68 none of aerialvla_codec.own_speed /
SpeedWords / object_with_speed, the builder's speed_words, the trainer's
dataset_speed_words, VLAFrames(speed_words=), Capture.own_speed,
SemanticObs.pose_history, vla_mission.speed_words_check, vla_fly's
--vla-speed-words or the DAgger's existed (every test below fails at an
AttributeError, a TypeError or an unknown flag).
"""
from __future__ import annotations

import contextlib
import inspect
import io
import json
import math
import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools", ROOT / "training", ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import aerialvla_codec as C                           # noqa: E402
from guardrail.models import State                                   # noqa: E402
from guardrail.vla_link import Capture, Hint, Pose, build_infer_fields  # noqa: E402

SKIP = "SKIP"
PS1 = ROOT / "scripts" / "run_citylife_follow.ps1"
NO_PORT = "127.0.0.1:59999"      # parsed only, never connected to
T0 = 1_000_000.0                 # tests/test_follow_dataset.py's wall-clock origin
ABSENT = "absent"
SW = C.SpeedWords() if hasattr(C, "SpeedWords") else None
HINT = Hint("straight ahead ", "detector")


def _B():
    import build_follow_dataset as B
    return B


def _TF():
    """tests/test_follow_dataset.py's synthetic recordings (the recorder's
    own file format)."""
    import test_follow_dataset as TF
    return TF


def _TD():
    """tests/test_vla_dagger.py's identity."""
    import test_vla_dagger as TD
    return TD


def _D():
    import vla_dagger as D
    return D


def _pil():
    try:
        from PIL import Image
        return Image
    except ImportError:
        if os.environ.get("GUARDRAIL_REQUIRE_PILLOW") == "1":
            raise AssertionError("GUARDRAIL_REQUIRE_PILLOW=1 and Pillow does not import")
        return None


def _rows(path: Path) -> list[dict]:
    return [json.loads(ln) for ln in path.read_text(encoding="utf-8").splitlines() if ln.strip()]


# A speed profile (m/s along x) with every class in it: hover, slow, cruise,
# fast, then an abrupt stop. Piecewise constant, so x(tau) is piecewise
# linear with its corners on the 10 Hz grid, and the 0.5 s backward speed at
# a grid capture is exact arithmetic - never within 0.1 m/s of a class bound.
PROFILE = ((0.0, 0.0), (2.0, 1.0), (3.0, 2.0), (4.0, 4.0), (6.0, 0.0))


def _speed_at(tau: float) -> float:
    v = 0.0
    for t, s in PROFILE:
        if tau >= t:
            v = s
    return v


def _x_at(tau: float) -> float:
    x, edges = 0.0, [t for t, _s in PROFILE] + [math.inf]
    for (t, s), nxt in zip(PROFILE, edges[1:]):
        if tau <= t:
            break
        x += s * (min(tau, nxt) - t)
    return x


def _profile_motion(tau):
    return _x_at(tau), 0.0, 8.0, 0.0


def _expected_speed(tau: float) -> float:
    """The 0.5 s backward speed at mission time tau, from the analytic track."""
    return abs(_x_at(tau) - _x_at(tau - 0.5)) / 0.5


def _sw_identity(setting=ABSENT, model: dict | None = None) -> dict:
    """A fine-tune's hello identity whose adapter provenance records the
    speed words `setting` (ABSENT: a provenance block without the key, as a
    fine-tune or a server from before 2026-10-24 announces itself)."""
    TD = _TD()
    m = json.loads(json.dumps(model or TD.FT_MODEL))
    prov = {"id_root": "aerialvla-ft-follow", "run": "r1", "epoch": 2, "epoch_dir": "epoch2"}
    if setting != ABSENT:
        prov["speed_words"] = setting
    m["adapter"]["provenance"] = prov
    return m


# =========================================================================== #
# A. The shared rule
# =========================================================================== #
def test_the_speed_classes_and_their_bounds():
    assert C.SPEED_WORDS_RULE == "own_speed_words/1"
    assert (C.SPEED_WORDS_WINDOW_S, C.SPEED_WORDS_MAX_GAP_S) == (0.5, 0.5)
    assert C.SPEED_WORDS == ("while hovering", "while flying slowly", "while flying",
                             "while flying fast")
    assert [hi for hi, _w in C.SPEED_CLASSES] == [0.5, 1.5, 3.0, math.inf]
    for v, words in ((0.0, "while hovering"), (0.4999, "while hovering"),
                     (0.5, "while flying slowly"), (1.4999, "while flying slowly"),
                     (1.5, "while flying"), (2.9999, "while flying"),
                     (3.0, "while flying fast"), (4.9, "while flying fast"),
                     (50.0, "while flying fast")):
        assert C.own_speed_words(v) == words, (v, C.own_speed_words(v))
    for bad in (-0.01, float("nan"), float("inf"), None, "2.0", True):
        try:
            C.own_speed_words(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad!r}")
    # Words only, after one space; every class passes the server's check.
    assert C.object_with_speed("a red car", 0.2) == "a red car while hovering"
    assert C.object_with_speed("a red car", 4.2) == "a red car while flying fast"
    for words in C.SPEED_WORDS:
        obj = f"a red car {words}"
        assert C.validate_object(obj) == obj and C.coordinate_text(obj) is None
        assert not any(ch.isdigit() for ch in words)
        assert C.speed_words_of(obj) == words
    assert C.speed_words_of("a red car") is None and C.speed_words_of(None) is None
    assert C.speed_words_of("a red car while flying") == "while flying"
    # Appended once; the base object must itself be a valid object.
    for bad in ("a red car while flying fast", "a red car while hovering"):
        try:
            C.object_with_speed(bad, 1.0)
        except ValueError:
            continue
        raise AssertionError(f"appended twice to {bad!r}")
    for bad in (" a red car", "a red\ncar", 7):
        try:
            C.object_with_speed(bad, 1.0)
        except (C.CodecError, TypeError, ValueError):
            continue
        raise AssertionError(f"accepted the object {bad!r}")


def test_the_speed_is_a_backward_difference_of_the_poses_at_or_before_the_capture():
    # 10 Hz poses at 2 m/s along x, then 6 m/s from t = 1.0 on.
    rows = [(k * 0.1, (2.0 * k * 0.1 if k <= 10 else 2.0 + 6.0 * (k * 0.1 - 1.0)), 0.0)
            for k in range(31)]
    h = C.speed_history(rows)
    got = C.own_speed(h, 1.0)
    assert math.isclose(got["speed_mps"], 2.0, abs_tol=1e-9) and got["words"] == "while flying"
    assert math.isclose(got["t_end"], 1.0) and math.isclose(got["t_start"], 0.5)
    assert got["window_s"] == 0.5
    # Never a later pose: every pose after the capture changed, same answer.
    wild = [(t, (x if t <= 1.0 + 1e-9 else 1e6), y) for t, x, y in rows]
    assert C.own_speed(C.speed_history(wild), 1.0) == got
    # Between ticks: the window ends at the newest pose at or before it.
    g2 = C.own_speed(h, 1.04)
    assert math.isclose(g2["t_end"], 1.0) and g2["speed_mps"] == got["speed_mps"]
    # The start is interpolated off the grid (window 0.45 s: 0.55 is between
    # ticks) - exact on a straight segment.
    g3 = C.own_speed(h, 1.0, window_s=0.45)
    assert math.isclose(g3["t_start"], 0.55) and math.isclose(g3["speed_mps"], 2.0)
    g4 = C.own_speed(h, 2.0)
    assert math.isclose(g4["speed_mps"], 6.0) and g4["words"] == "while flying fast"
    g5 = C.own_speed(h, 1.2)                        # 0.3 s at 2 + 0.2 s at 6 m/s
    assert math.isclose(g5["speed_mps"], (0.3 * 2 + 0.2 * 6) / 0.5)
    # No speed: before the first pose, a history shorter than the window,
    # the newest pose too old, a gap around the window's start.
    assert C.own_speed(h, -0.1) is None
    assert C.own_speed(h, 0.45) is None
    assert C.own_speed(h, 0.5) is not None
    assert C.own_speed(h, 3.0 + 0.51) is None, "the newest pose is 0.51 s old"
    assert C.own_speed(h, 3.0 + 0.49) is not None
    holed = C.speed_history([r for r in rows if not 0.35 < r[0] < 0.95])
    assert C.own_speed(holed, 1.1) is None, "0.3 .. 1.0 is a 0.7 s gap around 0.6"
    assert C.own_speed(holed, 1.5) is not None
    assert C.own_speed([], 1.0) is None
    # The history: finite rows, by time, a repeated time keeping the last.
    hh = C.speed_history([(0.2, 9.0, 0.0), (0.1, 0.0, 0.0), (0.2, 1.0, 0.0),
                          (0.3, float("nan"), 0.0), ("x", 1, 1), {"t": 0.4, "x": 2.0, "y": 0.0},
                          (0.0, 0.0, 0.0)])
    assert hh == [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.2, 1.0, 0.0), (0.4, 2.0, 0.0)], hh
    # Horizontal only: a climb is no speed.
    climb = C.speed_history([(k * 0.1, 0.0, 0.0) for k in range(10)])
    assert C.own_speed(climb, 0.9)["words"] == "while hovering"


def test_the_setting_record_and_the_mismatch_both_ways():
    assert C.speed_words(False) is None and C.speed_words(True) == SW
    d = SW.as_dict()
    assert d == {"rule": "own_speed_words/1", "window_s": 0.5, "max_gap_s": 0.5,
                 "classes": [[0.5, "while hovering"], [1.5, "while flying slowly"],
                             [3.0, "while flying"], [None, "while flying fast"]]}, d
    assert json.loads(json.dumps(d)) == d
    assert C.speed_words_from(None) is None and C.speed_words_from({}) is None
    assert C.speed_words_from(d) == SW
    assert C.speed_words_from({"rule": "own_speed_words/1"}) == SW, "the rule's defaults"
    assert C.speed_words_from(dict(d, max_gap_s=0.3)) == SW, "max_gap_s is slack"
    for bad in ({"rule": "own_speed_words/2"}, {"window_s": 0.5}, dict(d, window_s=0),
                dict(d, window_s=float("nan")), dict(d, classes=[[0.5]]), "on", 1):
        try:
            C.speed_words_from(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad!r}")
    other_window = C.speed_words_from(dict(d, window_s=1.0))
    other_classes = C.speed_words_from(dict(d, classes=[[1.0, "while hovering"],
                                                        [None, "while flying"]]))
    assert C.speed_words_mismatch(None, None) is None
    assert C.speed_words_mismatch(SW, SW) is None
    for trained, given in ((SW, None), (None, SW), (SW, other_window), (SW, other_classes)):
        why = C.speed_words_mismatch(trained, given)
        assert why and "speed words" in why, (trained, given, why)
    assert "no speed words" in C.speed_words_mismatch(SW, None)
    assert "1 s window" in C.speed_words_mismatch(SW, other_window)
    # What hello says: absent, announced off, announced on, malformed.
    assert C.adapter_speed_words(None) == (None, ABSENT)
    assert C.adapter_speed_words({"id": "x"}) == (None, ABSENT)
    assert C.adapter_speed_words(_sw_identity()) == (None, ABSENT)
    assert C.adapter_speed_words(_sw_identity(None)) == (None, "announced")
    assert C.adapter_speed_words(_sw_identity(d)) == (SW, "announced")
    try:
        C.adapter_speed_words(_sw_identity({"rule": "nope"}))
    except ValueError:
        pass
    else:
        raise AssertionError("a malformed announcement was read")


# =========================================================================== #
# B. The dataset builder
# =========================================================================== #
def test_the_builder_appends_the_speed_words_and_drops_a_capture_without_history():
    B, TF = _B(), _TF()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TF._write_run(td, "runS", _profile_motion, seconds=8.0)
        st = B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False,
                             speed_words=True)
        rows = {r["i"]: r for r in _rows(td / "ds" / "samples.jsonl")}
        # captures 0..4 (tau < 0.5 s) have no pose 0.5 s back; 71..80 no label
        assert st["dropped"]["no_speed"] == 5, st["dropped"]
        # judged after the label: every other reason counts as without words
        assert B.DROP_ORDER.index("track_gap") < B.DROP_ORDER.index("no_speed") < \
            B.DROP_ORDER.index("perturbed")
        assert sorted(rows) == list(range(5, 71)), sorted(rows)
        for i, r in rows.items():
            tau = round(i * 0.1, 10)
            v = _expected_speed(tau)
            sp = r["own_speed"]
            assert math.isclose(sp["speed_mps"], v, abs_tol=1e-6), (i, sp, v)
            assert sp["words"] == C.own_speed_words(v), (i, sp)
            assert sp["window_s"] == 0.5 and sp["object_base"] == "a red car"
            assert math.isclose(sp["t_end"], r["t_capture"], abs_tol=1e-6), "the capture's tick"
            assert math.isclose(sp["t_end"] - sp["t_start"], 0.5, abs_tol=1e-6)
            assert r["object"] == f"a red car {sp['words']}"
            assert r["prompt"] == C.build_prompt(r["phrase"], r["object"])
            assert r["prompt_sha256"] == C.prompt_sha256(r["prompt"])
        # Every class, where the profile puts it; the words never foresee:
        # at 2.0 s the aircraft still hovers though the label moves 1 m, at
        # 5.9 s it is fast though it stops at 6.0.
        for i, words in ((10, "while hovering"), (20, "while hovering"),
                         (25, "while flying slowly"), (36, "while flying"),
                         (50, "while flying fast"), (59, "while flying fast"),
                         (64, "while flying slowly"), (69, "while hovering")):
            assert rows[i]["own_speed"]["words"] == words, (i, rows[i]["own_speed"])
        assert rows[20]["label"]["fwd_m"] > 0.9 and rows[59]["label"]["fwd_m"] < 0.5
        # The labels, phrases, bins and images are the plain build's, sample
        # for sample; only the object words (and so the prompt) differ.
        s0 = B.build_dataset([run], td / "ds0", horizon_s=1.0, stride_s=0.0, images=False)
        plain = {r["i"]: r for r in _rows(td / "ds0" / "samples.jsonl")}
        assert "no_speed" not in s0["dropped"]
        assert {k: v for k, v in st["dropped"].items() if k != "no_speed"} == s0["dropped"]
        assert set(plain) - set(rows) == set(range(5)), "only the no-history captures"
        for i, r in rows.items():
            p = plain[i]
            assert "own_speed" not in p and p["object"] == "a red car"
            assert set(r) - set(p) == {"own_speed"}, set(r) ^ set(p)
            for key in p:
                if key not in ("object", "prompt", "prompt_sha256"):
                    assert r[key] == p[key], (i, key)


def test_the_mirror_keeps_the_words_and_the_stats_count_them():
    B, TF = _B(), _TF()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        ra = TF._write_run(td, "ra", _profile_motion, seconds=8.0)
        rb = TF._write_run(td, "rb", TF._straight(speed=2.0), seconds=4.0)
        st = B.build_dataset([ra, rb], td / "ds", horizon_s=1.0, stride_s=0.0, images=False,
                             speed_words=True, mirror=True, holdout=["rb"])
        rows = _rows(td / "ds" / "samples.jsonl")
        real = {r["id"]: r for r in rows if not r["mirrored"]}
        mirrored = [r for r in rows if r["mirrored"]]
        assert mirrored and all(m["run"] == "ra" for m in mirrored)
        for m in mirrored:
            src = real[m["mirror_of"]]
            assert m["own_speed"] == src["own_speed"] and m["object"] == src["object"]
            assert m["prompt"] == C.build_prompt(m["phrase"], src["object"])
        # The setting, and the tables.
        assert st["speed_words"] == SW.as_dict() and st["args"]["speed_words"] is True
        sp = st["speed"]
        assert sp["setting"] == SW.as_dict() and sp["no_speed"] == 10, sp["no_speed"]
        n_real = len(real)
        assert sum(sp["by_class_real"].values()) == n_real
        assert sum(sp["by_class"].values()) == len(rows)
        assert list(sp["by_class_real"]) == [w for w in C.SPEED_WORDS
                                             if w in sp["by_class_real"]], "class order"
        assert sp["by_split"]["holdout"] == {"while flying": 26}, sp["by_split"]
        assert sum(sp["by_split"]["train"].values()) == len(rows) - 26
        assert sp["by_run"]["rb"] == {"while flying": 26}
        assert sp["by_run"]["ra"] == st["runs"]["ra"]["speed_words"]
        assert set(sp["by_run"]["ra"]) == set(C.SPEED_WORDS), sp["by_run"]["ra"]
        assert set(sp["by_fwd_band"]) == set(B.FWD_BANDS)
        assert sum(sum(h.values()) for h in sp["by_fwd_band"].values()) == n_real
        # hovering with a large label: the aircraft starting after the car
        assert sp["by_fwd_band"]["0.5-1.5"].get("while hovering", 0) > 0, sp["by_fwd_band"]
        assert sp["label_by_class"]["while flying fast"]["n"] == sp["by_class_real"][
            "while flying fast"]
        assert sp["speed_mps"]["n"] == n_real and sp["speed_mps"]["max"] <= 4.0 + 1e-6
        assert any("Speed words" in n for n in st["notes"])
        # Off: nothing of it.
        s0 = B.build_dataset([ra, rb], td / "ds0", horizon_s=1.0, stride_s=0.0, images=False,
                             mirror=True, holdout=["rb"])
        assert s0["speed_words"] is None and s0["speed"] is None
        assert s0["args"]["speed_words"] is False
        assert all("speed_words" not in r for r in s0["runs"].values())
        assert not any("Speed words" in n for n in s0["notes"])


def test_the_builder_cli_takes_speed_words_and_refuses_words_already_there():
    B, TF = _B(), _TF()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TF._write_run(td, "runC", _profile_motion, seconds=8.0)
        out = io.StringIO()
        try:
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
                rc = B.main([str(run), "--out", str(td / "ds"), "--no-images", "--speed-words"])
        except SystemExit as exc:                  # argparse: an unknown flag
            raise AssertionError(f"the builder's CLI refused --speed-words ({exc.code})")
        assert rc == 0
        st = json.loads((td / "ds" / "stats.json").read_text(encoding="utf-8"))
        assert st["speed_words"] == SW.as_dict()
        said = out.getvalue()
        assert "speed words" in said and "label fwd 0.5-1.5 m" in said, said[-800:]
        with contextlib.redirect_stdout(io.StringIO()):
            assert B.main([str(run), "--out", str(td / "ds0"), "--no-images"]) == 0
        assert json.loads((td / "ds0" / "stats.json").read_text(encoding="utf-8"))[
            "speed_words"] is None
        # A recording whose object words already end in speed words.
        bad = TF._write_run(td, "runW", _profile_motion, seconds=4.0,
                            object_text="a red car while flying")
        with contextlib.redirect_stderr(io.StringIO()) as err, \
                contextlib.redirect_stdout(io.StringIO()):
            rc = B.main([str(bad), "--out", str(td / "bad"), "--no-images", "--speed-words"])
        assert rc == 2 and "speed words" in err.getvalue(), err.getvalue()
        assert not (td / "bad" / "samples.jsonl").exists()


# =========================================================================== #
# B. The flight: VLAFrames over the pose ring, the same words
# =========================================================================== #
class _NoHistoryObs:
    def pose_at(self, t, stamp=None):
        return {"x": 0.0, "y": 0.0, "up": 8.0, "yaw": 0.0, "src": "ring"}

    def wall_at_stamp(self, stamp):
        return None


def _ring_frames(rows, k, future=3, sw=SW, clock_t=None):
    """A SemanticObs fed the track rows up to capture k + `future` (poses the
    capture must not see), and a VLAFrames whose newest front+down pair is
    the one stamped at tick k: its capture time maps through the ring onto
    the tick's own wall time - the recorder's (VLAFrames') rule."""
    import vla_mission as VM
    from semantic_demo import SemanticObs
    obs = SemanticObs()
    stamp = lambda j: 5_000_000_000 + j * 100_000_000            # noqa: E731
    for j, r in enumerate(rows[:k + future + 1]):
        obs.put_pose_full(r["t"], r["x"], r["y"], r["up"], r["yaw"], 0.0, 0.0, stamp=stamp(j))
    clock = {"t": rows[min(len(rows) - 1, k + future)]["t"] if clock_t is None else clock_t}
    fr = VM.VLAFrames(obs, encode=lambda m: m["jpg"], clock=lambda: clock["t"],
                      speed_words=sw)
    msg = {"time_stamp": stamp(k), "width": 256, "height": 256}
    fr.put_front(dict(msg, jpg=f"F{k}".encode()))
    fr.put_down(dict(msg, jpg=f"D{k}".encode()))
    return fr, obs


def test_the_flight_and_the_builder_give_the_same_words_for_the_same_pose_history():
    B, TF = _B(), _TF()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TF._write_run(td, "runF", _profile_motion, seconds=8.0)
        B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False,
                        speed_words=True)
        samples = {r["i"]: r for r in _rows(td / "ds" / "samples.jsonl")}
        track = _rows(run / "track.jsonl")
        checked = 0
        for k in (5, 12, 20, 21, 23, 25, 31, 36, 42, 43, 50, 59, 61, 64, 66, 69, 70):
            fr, _obs = _ring_frames(track, k)
            cap = fr.capture()
            assert cap is not None, (k, dict(fr.counts))
            assert math.isclose(cap.t_capture, samples[k]["t_capture"], abs_tol=1e-6), k
            got, want = cap.own_speed, samples[k]["own_speed"]
            assert got["words"] == want["words"], (k, got, want)
            assert math.isclose(got["speed_mps"], want["speed_mps"], abs_tol=1e-6), (k, got)
            assert got["window_s"] == want["window_s"] == 0.5 and got["rule"] == SW.rule
            # The request's prompt is the sample's, byte for byte.
            fields, meta = build_infer_fields(cap, Hint(samples[k]["phrase"], "detector"),
                                              "a red car")
            assert fields["object"] == samples[k]["object"], (k, fields["object"])
            assert C.build_prompt(fields["phrase"], fields["object"]) == samples[k]["prompt"]
            checked += 1
        assert checked == 17
        # Too little history (the first half second): no capture, as no sample.
        fr, _ = _ring_frames(track, 4)
        assert fr.capture() is None and fr.counts["no_speed"] == 1, dict(fr.counts)
        assert 4 not in samples


def test_vlaframes_sends_nothing_without_history_and_records_the_words():
    import vla_mission as VM
    from guardrail.vla_backends import RemoteVLABackend
    rows = [{"t": T0 + 0.1 * k, "x": 0.4 * k, "y": 0.0, "up": 8.0, "yaw": 0.0}
            for k in range(30)]
    # 0.3 s of poses: nothing is sent, the backend asks get_capture first.
    fr, _ = _ring_frames(rows, 2, future=0)
    sent = []

    class Client:
        busy = backing_off = False
        counts: dict = {}
        identity = None
        late = []

        def submit(self, fields):
            sent.append(fields)
            return len(sent)

        def poll(self):
            return None

        def close(self):
            pass
    be = RemoteVLABackend(Client(), get_capture=fr.capture, get_hint=lambda cap: HINT,
                          object_text="a red car", cadence="pipelined",
                          clock=lambda: T0 + 1.0)
    be.act(State(x=0.0, y=0.0, up=10.0, yaw_deg=0.0))
    assert not sent and fr.counts["no_speed"] == 1 and fr.counts["captures"] == 0
    # Enough history: 4 m/s, "while flying fast"; the object on the wire,
    # the record and the commit log say so.
    fr, _ = _ring_frames(rows, 20)
    be = RemoteVLABackend(Client(), get_capture=fr.capture, get_hint=lambda cap: HINT,
                          object_text="a red car", cadence="pipelined",
                          on_submit=fr.commit, clock=lambda: T0 + 3.0)
    be.act(State(x=0.0, y=0.0, up=10.0, yaw_deg=0.0))
    assert len(sent) == 1 and sent[0]["object"] == "a red car while flying fast", sent
    C.validate_request(dict(sent[0], seq=1))
    meta = be._meta[1]
    assert meta["object"] == "a red car while flying fast"
    os_ = meta["own_speed"]
    assert os_["words"] == "while flying fast" and os_["object_base"] == "a red car"
    assert math.isclose(os_["speed_mps"], 4.0, abs_tol=1e-6) and os_["window_s"] == 0.5
    rec = fr.log[-1]
    assert rec["own_speed"]["words"] == "while flying fast" and rec["seq"] == 1
    st = fr.stats()
    assert st["speed_words"]["rule"] == SW.rule
    assert st["speed_words"]["words"] == {"while flying fast": 1}
    assert st["speed_words"]["speed_mps"]["n"] == 1
    # A capture built by hand with own_speed (as a replay would not have):
    # words that are not the rule's for the speed are refused.
    cap = Capture(T0, Pose(0.0, 0.0, 8.0, 0.0), b"F", b"D",
                  own_speed={"speed_mps": 4.0, "words": "while hovering", "window_s": 0.5})
    try:
        build_infer_fields(cap, HINT, "a red car")
    except ValueError:
        pass
    else:
        raise AssertionError("words that disagree with the speed were sent")
    # Speed words need the ring's history.
    try:
        VM.VLAFrames(_NoHistoryObs(), encode=lambda m: m["jpg"], speed_words=SW)
    except ValueError as exc:
        assert "pose_history" in str(exc), exc
    else:
        raise AssertionError("VLAFrames took speed words with no pose history")


def test_default_off_is_unchanged():
    import vla_mission as VM
    import vla_fly as VF
    D = _D()
    rows = [{"t": T0 + 0.1 * k, "x": 0.4 * k, "y": 0.0, "up": 8.0, "yaw": 0.0}
            for k in range(30)]
    fr, _ = _ring_frames(rows, 2, future=0, sw=None)              # 0.3 s: off needs none
    assert fr.speed_words is None
    cap = fr.capture()
    assert cap is not None and cap.own_speed is None and "no_speed" not in fr.counts
    fields, meta = build_infer_fields(cap, HINT, "a red car")
    assert fields["object"] == meta["object"] == "a red car" and "own_speed" not in meta
    fr.commit(cap, HINT, 1)
    assert "own_speed" not in fr.log[-1] and "speed_words" not in fr.stats()
    assert VM.VLAFrames(_NoHistoryObs()).speed_words is None, "off needs no history"
    a = VF.parse_cli(VF.build_arg_parser(), ["--mission", "follow"])
    assert a.vla_speed_words is False and a.speed_words is None
    assert VF.check_args_report(a)["speed_words"] is None
    vargs, argv = D.follow_args("a red car", cruise_alt=10.0)
    assert vargs.speed_words is None and "--vla-speed-words" not in argv
    assert argv == ["--mission", "follow", "--cadence", "pipelined", "--object=a red car",
                    "--cruise-mps", "4.5", "--speed-cap-mps", "5", "--cruise-alt", "10"], argv
    # The prompt template is upstream's: no words unless asked.
    assert C.build_prompt("straight ahead ", "a red car") == \
        "<image>\nFly straight ahead and find the target. a red car\nAction: "


# =========================================================================== #
# D. The request round-trips through the codec, the server and the trainer
# =========================================================================== #
def test_a_speed_words_request_round_trips_through_the_codec_and_the_server():
    import vla_server_v2 as S
    import finetune_aerialvla_follow as FT
    rows = [{"t": T0 + 0.1 * k, "x": 0.01 * k, "y": 0.02 * k, "up": 8.0, "yaw": 0.0}
            for k in range(30)]
    fr, _ = _ring_frames(rows, 20)
    cap = fr.capture()
    assert cap.own_speed["words"] == "while hovering", cap.own_speed        # 0.224 m/s
    assert math.isclose(cap.own_speed["speed_mps"], math.hypot(0.1, 0.2), abs_tol=1e-6)
    fields, _meta = build_infer_fields(cap, Hint("forward-left ", "detector"), "a red car")
    req = dict(fields, seq=7)
    C.validate_request(req)
    seen = {}

    class Model:
        precision, attn, merged, max_new_tokens = "fake", "none", False, 12

        def generate(self, mosaic, prompt):
            seen["prompt"] = prompt
            return {"raw_text": "45 49 49</s>", "model_s": 0.0}
    core = S.V2Server(Model(), dict(S.FAKE_IDENTITY), missing_down="refuse",
                      decode_frames=lambda r: ("front", "down"),
                      mosaic_fn=lambda f, d, missing_down: ("mosaic", {}),
                      hash_fn=lambda m: "0" * 64)
    resp, _stop, _extra = core.handle_request(req)
    assert resp["ok"] and resp["parse_ok"], resp
    want = "<image>\nFly forward-left and find the target. a red car while hovering\nAction: "
    assert seen["prompt"] == want, seen["prompt"]
    assert resp["prompt_sha256"] == C.prompt_sha256(want)
    # The trainer reads it back into its phrase and its full object words.
    assert FT.split_prompt(want) == ("forward-left ", "a red car while hovering")


# =========================================================================== #
# C. The trainer, the provenance, the server's hello, the evaluation tool
# =========================================================================== #
def _rewrite(path: Path, rows: list[dict]) -> None:
    path.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8",
                    newline="\n")


def _speed_dataset(td: Path, name: str = "ds", **kw) -> Path:
    """A built dataset (no images: each sample names a placeholder, which the
    trainer reads with check_images=False)."""
    B, TF = _B(), _TF()
    ra = TF._write_run(td, f"{name}_a", _profile_motion, seconds=8.0)
    rb = TF._write_run(td, f"{name}_b", TF._straight(speed=2.0), seconds=4.0)
    B.build_dataset([ra, rb], td / name, horizon_s=1.0, stride_s=0.0, images=False,
                    holdout=[f"{name}_b"], **kw)
    rows = _rows(td / name / "samples.jsonl")
    _rewrite(td / name / "samples.jsonl", [dict(r, image="images/placeholder.png")
                                           for r in rows])
    return td / name


def test_the_trainer_reads_the_speed_words_refuses_a_mix_and_writes_the_provenance():
    import finetune_aerialvla_follow as FT
    import vla_server_v2 as S
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        on = _speed_dataset(td, "on", speed_words=True)
        s_on, rep = FT.load_samples(on, check_images=False)
        assert rep["kept"] == rep["lines"] and not rep["dropped"], rep
        assert all(s.extra["own_speed"]["words"] == C.speed_words_of(s.object) for s in s_on)
        got = FT.dataset_speed_words(on, s_on)
        assert got == {"setting": SW.as_dict(), "source": "stats.json"}, got
        off = _speed_dataset(td, "off")
        s_off, _ = FT.load_samples(off, check_images=False)
        assert FT.dataset_speed_words(off, s_off) == {"setting": None, "source": "stats.json"}
        # A dataset from before (no stats key, no sample record): off.
        (off / "stats.json").unlink()
        g = FT.dataset_speed_words(off, s_off)
        assert g["setting"] is None and "absent" in g["source"], g
        # Without stats.json the samples say it.
        (on / "stats.json").rename(on / "stats.keep")
        assert FT.dataset_speed_words(on, s_on) == {"setting": SW.as_dict(),
                                                    "source": "samples"}
        (on / "stats.keep").rename(on / "stats.json")
        # Refused: a mix, words that disagree with the speed or the object,
        # words with no record, two windows, stats.json against the samples.
        base = _rows(on / "samples.jsonl")

        def refused(rows, stats=None) -> str:
            d = td / f"bad{refused.k}"
            refused.k += 1
            d.mkdir()
            _rewrite(d / "samples.jsonl", rows)
            if stats is not None:
                (d / "stats.json").write_text(json.dumps(stats), encoding="utf-8")
            s, _ = FT.load_samples(d, check_images=False)
            try:
                FT.dataset_speed_words(d, s)
            except FT.DataError as exc:
                return str(exc)
            raise AssertionError(f"accepted {d.name}")
        refused.k = 0
        plain = [dict(r, object="a red car",
                      prompt=C.build_prompt(r["phrase"], "a red car")) for r in base]
        for r in plain:
            r.pop("own_speed")
            r.pop("prompt_sha256")
        assert "mix" in refused(base[:20] + plain[20:])
        lie = [dict(r, own_speed=dict(r["own_speed"], speed_mps=10.0))
               if r["own_speed"]["words"] != "while flying fast" else r for r in base]
        assert "classes say" in refused(lie)
        swapped = [dict(r, own_speed=dict(r["own_speed"], words="while flying fast",
                                          speed_mps=4.0)) for r in base]
        assert "does not end in its speed words" in refused(
            [s for s in swapped if not s["object"].endswith("fast")][:5])
        unmarked = [dict(r) for r in base]
        for r in unmarked:
            r.pop("own_speed")
        assert "no own_speed record" in refused(unmarked)
        two = [dict(r, own_speed=dict(r["own_speed"], window_s=1.0)) if k % 2 else r
               for k, r in enumerate(base)]
        assert "mix" in refused(two)
        assert "stats.json says" in refused(base, {"speed_words": None})
        assert "stats.json says" in refused(plain, {"speed_words": SW.as_dict()})
        # The dry run says it, and refuses a mix.
        rp = td / "rep.json"
        with contextlib.redirect_stdout(io.StringIO()):
            FT.main(["--data", str(on), "--run", "x", "--dry-run", "1", "--no-image-check",
                     "--base", str(td / "no-such-base"), "--report", str(rp)])
        assert json.loads(rp.read_text(encoding="utf-8"))["speed_words"]["setting"] == \
            SW.as_dict()
        with contextlib.redirect_stdout(io.StringIO()):
            rc = FT.main(["--data", str(td / "bad0"), "--run", "x", "--dry-run", "1",
                          "--no-image-check", "--base", str(td / "no-such-base"), "--report",
                          str(rp)])
        assert rc == 1 and "speed-words" in json.loads(rp.read_text(encoding="utf-8"))["refused"]
        # The provenance: null by default, the setting when given.
        kw = dict(run="r", epoch=1, parent_adapter=FT.DEFAULT_ADAPTER, parent_sha256="ab" * 32,
                  base=FT.DEFAULT_BASE, data_report={"file": "x", "sha256": "cd" * 32},
                  split_info={"train": 4, "held_out": 2, "train_runs": ["r1"],
                              "held_out_runs": ["r3"]},
                  precision="bf16", hyper={}, metrics=None, step=3)
        assert FT.provenance(**kw)["speed_words"] is None
        prov = FT.provenance(**kw, speed_words=got["setting"])
        assert prov["speed_words"] == SW.as_dict()
        # The server announces it in hello; the codec reads it back.
        ad = td / "ft" / "r" / "epoch1"
        ad.mkdir(parents=True)
        (ad / "adapter_config.json").write_text("{}", encoding="utf-8")
        (ad / C.ADAPTER_PROVENANCE_FILE).write_text(json.dumps(prov), encoding="utf-8")
        info = S.adapter_identity(ad, "1a" * 32)
        assert info["provenance"]["speed_words"] == SW.as_dict(), info["provenance"]
        assert C.adapter_speed_words({"adapter": info}) == (SW, "announced")
        # a checkpoint from before has no key: announced as null -> off
        old = dict(prov)
        del old["speed_words"]
        (ad / C.ADAPTER_PROVENANCE_FILE).write_text(json.dumps(old), encoding="utf-8")
        info = S.adapter_identity(ad, "1a" * 32)
        assert info["provenance"]["speed_words"] is None
        assert C.adapter_speed_words({"adapter": info}) == (None, "announced")


def test_the_training_loop_writes_the_speed_words_into_the_one_provenance_file():
    Image = _pil()
    try:
        import torch                                               # noqa: F401
    except ImportError:
        torch = None
    if torch is None or Image is None:
        print("      SKIP: torch and Pillow are needed (the vla-real env has both)")
        return SKIP
    import finetune_aerialvla_follow as FT
    import test_finetune_follow as TFT
    d = TFT._synthetic_dataset(Image)
    rows = _rows(d / "samples.jsonl")
    for r in rows:
        base = r["object"] if "object" in r else FT.split_prompt(r["prompt"])[1]
        words = C.own_speed_words(2.0)
        r["object"] = f"{base} {words}"
        phrase = FT.split_prompt(r["prompt"])[0]
        r["prompt"] = C.build_prompt(phrase, r["object"])
        r.pop("prompt_sha256", None)
        r["own_speed"] = {"speed_mps": 2.0, "words": words, "window_s": 0.5,
                          "object_base": base}
    _rewrite(d / "samples.jsonl", rows)
    (d / "stats.json").write_text(json.dumps({"speed_words": SW.as_dict(),
                                              "args": {"horizon_s": 1.0}}), encoding="utf-8")
    parent = Path(tempfile.mkdtemp(prefix="sw_parent_"))
    (parent / "adapter_config.json").write_text("{}", encoding="utf-8")
    out = Path(tempfile.mkdtemp(prefix="sw_out_"))
    args = FT.build_arg_parser().parse_args(
        ["--data", str(d), "--run", "sw", "--epochs", "1", "--batch-size", "2",
         "--grad-accum", "1", "--eval-max", "0", "--device", "cpu", "--no-eval-before",
         "--adapter", str(parent), "--out-root", str(out), "--base", "unused"])
    with contextlib.redirect_stdout(io.StringIO()):
        assert FT.train(args, loader=TFT._tiny_loader("96 49 49")) == 0
    ck = out / "sw" / "epoch1"
    prov = json.loads((ck / C.ADAPTER_PROVENANCE_FILE).read_text(encoding="utf-8"))
    assert prov["speed_words"] == SW.as_dict(), prov
    rec = json.loads((out / "sw" / "run.json").read_text(encoding="utf-8"))
    assert rec["speed_words"]["setting"] == SW.as_dict()


def test_the_evaluation_tool_sends_each_prompt_and_says_when_the_adapter_differs():
    import eval_vla_adapter as EV
    import finetune_aerialvla_follow as FT
    on = _sw_identity(SW.as_dict())
    assert EV.speed_words_check(on, SW.as_dict()) == "ok"
    assert EV.speed_words_check({"id": "fake:scripted"}, None) == "ok"
    assert EV.speed_words_check(_sw_identity(), None) == "ok", "absent is off"
    why = EV.speed_words_check({"id": "fake:scripted"}, SW.as_dict())
    assert why != "ok" and "no speed words" in why, why
    why = EV.speed_words_check(on, None)
    assert why != "ok" and "speed words" in why, why
    src = (ROOT / "tools" / "eval_vla_adapter.py").read_text(encoding="utf-8")
    assert "FT.dataset_speed_words(" in src and '"speed_words_check"' in src
    # The request carries the sample's full object words: the server builds
    # the sample's prompt (no Pillow: the request is only built and checked).
    with tempfile.TemporaryDirectory() as td:
        d = _speed_dataset(Path(td), "ev", speed_words=True)
        samples, _ = FT.load_samples(d, check_images=False)
        held = [s for s in samples if s.split == "holdout"]
        assert held and all(s.object == "a red car while flying" for s in held)
        assert all(C.build_prompt(s.phrase, s.object) == s.prompt for s in held)


def test_the_evaluation_through_the_servers_mosaic_matches_every_prompt():
    Image = _pil()
    if Image is None:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    import finetune_aerialvla_follow as FT
    import eval_vla_adapter as EV
    B, TF = _B(), _TF()

    def jpeg(k):
        out = []
        for col in ((180, 20 + 3 * k, 60), (10, 90, 10)):
            im = Image.new("RGB", (256, 256), col)
            buf = io.BytesIO()
            im.save(buf, format="JPEG", quality=95)
            out.append(buf.getvalue())
        return tuple(out)

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        ra = TF._write_run(td, "ra", _profile_motion, seconds=8.0, jpeg=jpeg)
        rb = TF._write_run(td, "rb", _profile_motion, seconds=8.0, jpeg=jpeg)
        B.build_dataset([ra, rb], td / "ds", holdout=["rb"], images=True, speed_words=True)
        samples, _ = FT.load_samples(td / "ds")
        held = [s for s in samples if s.split == "holdout"]
        assert len({C.speed_words_of(s.object) for s in held}) >= 3, "several classes"
        req = EV.build_request(held[0], FT.load_mosaic(held[0].image, held[0].image_sha256), 1)
        assert req["object"] == held[0].object
        res = EV.evaluate_fake([held[0].target + "</s>"] * len(held), held)
        f = res["fidelity"]
        assert f["answered"] == len(held), f
        assert f["prompt_sha256_mismatch"] == 0 and f["image_sha256_mismatch"] == 0, f


# =========================================================================== #
# C. The refusal before take-off: vla_fly and the DAgger recorder
# =========================================================================== #
def test_vla_fly_refuses_an_adapter_whose_speed_words_differ_either_way():
    import vla_mission as VM
    import vla_fly as VF
    d = SW.as_dict()
    ok = VM.speed_words_check(_sw_identity(d), SW, is_vla=True)
    assert ok["ok"] and ok["checked"] and ok["adapter"] == d and ok["flight"] == d, ok
    assert ok["source"] == "announced"
    off = VM.speed_words_check(_TD().FT_MODEL, None, is_vla=True)
    assert off["ok"] and off["flight"] is None and off["adapter"] is None, off
    assert VM.speed_words_check(_sw_identity(None), None, is_vla=True)["ok"]
    for ident, flight in ((_sw_identity(), SW),                      # an older fine-tune
                          (_sw_identity(None), SW),                  # announced off
                          (_TD().FT_MODEL, SW),                      # no provenance at all
                          ({"id": "aerialvla-lora/aero_vla@196f2f32"}, SW),
                          (_sw_identity(d), None),                   # trained with, flown without
                          (_sw_identity(dict(d, window_s=1.0)), SW)):
        r = VM.speed_words_check(ident, flight, is_vla=True)
        assert r["checked"] and not r["ok"] and r["why"], (ident, flight, r)
    r = VM.speed_words_check(_sw_identity({"rule": "nope"}), SW, is_vla=True)
    assert not r["ok"] and "malformed" in r["why"], r
    # A scripted stand-in reads no prompt: not checked, flown (and recorded) as one.
    st = VM.speed_words_check({"id": "fake:scripted"}, SW, is_vla=False)
    assert st["ok"] and not st["checked"] and "stand-in" in st["source"], st
    # A server from before the field: its hello cannot say, the file here can.
    with tempfile.TemporaryDirectory() as td:
        ad = Path(td) / "epoch2"
        ad.mkdir()
        (ad / C.ADAPTER_PROVENANCE_FILE).write_text(json.dumps(
            {"id_root": "aerialvla-ft-follow", "run": "r1", "epoch_dir": "epoch2",
             "speed_words": d}), encoding="utf-8")
        ident = _sw_identity()
        ident["adapter"]["path"] = ad.as_posix()
        r = VM.speed_words_check(ident, SW, is_vla=True)
        assert r["ok"] and r["source"].startswith("local"), r
        assert not VM.speed_words_check(ident, None, is_vla=True)["ok"]
        assert not VM.speed_words_check(ident, SW, is_vla=True, read_local=False)["ok"]
    # The flag, recorded by --check-args.
    a = VF.parse_cli(VF.build_arg_parser(), ["--mission", "follow", "--vla-speed-words"])
    assert a.speed_words == SW and VF.check_args_report(a)["speed_words"] == d
    # _fly checks it right after hello, before the simulator and the arming,
    # exits as a refusal, builds VLAFrames with it and records the check.
    src = inspect.getsource(VF._fly)
    i = src.index("VM.speed_words_check(")
    assert src.index("server.wait_hello(") < i < src.index("sim.connect()") < \
        src.index("drone.arm()"), "the speed-words check is not before the simulator"
    block = src[i:i + 900]
    assert "raise SystemExit(EXIT_REFUSED)" in block and "server.stop()" in block, block
    assert "speed_words=args.speed_words" in src, "VLAFrames is not given the setting"
    assert '"speed_words": sw_check' in src, "params do not record the check"


def test_the_dagger_recorder_follows_vla_flys_speed_words():
    D, TD = _D(), _TD()
    import time
    import vla_fake_server as F
    from semantic_demo import SemanticObs
    vargs, argv = D.follow_args("a red car", cruise_alt=10.0, speed_words=True)
    assert vargs.speed_words == SW and argv[-1] == "--vla-speed-words"
    assert D.check_args(NO_PORT, (6.0, 10.0), (2.5, 4.0), speed_words=True) is None
    kw = dict(seed=1, box_source=lambda: None, policy=None, object_text="a red car",
              cruise_alt=8.0, record_dir=None)
    with F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0), model=TD.FT_MODEL) as srv:
        assert srv.port != 8790
        try:
            D.make_vla_perturber(f"127.0.0.1:{srv.port}", speed_words=True,
                                 obs=SemanticObs(), **kw)
        except D.DaggerRefused as exc:
            assert "speed words" in str(exc), exc
        else:
            raise AssertionError("an adapter without speed words flew speed-words windows")
        p = D.make_vla_perturber(f"127.0.0.1:{srv.port}", obs=SemanticObs(), **kw)
        try:
            assert p.frames.speed_words is None and p.config["speed_words"]["ok"]
            assert "--vla-speed-words" not in p.config["vla_fly_argv"]
        finally:
            p.stop()
    sw_model = _sw_identity(SW.as_dict())
    with F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0), model=sw_model) as srv:
        assert srv.port != 8790
        try:
            D.make_vla_perturber(f"127.0.0.1:{srv.port}", obs=SemanticObs(), **kw)
        except D.DaggerRefused as exc:
            assert "speed words" in str(exc), exc
        else:
            raise AssertionError("a speed-words adapter flew windows without them")
        # A window's request is asked exactly as vla_fly would: the frames'
        # sim stamps map onto the last 1.6 s of the wall clock, the aircraft
        # at 2 m/s ("while flying"), the expert's box fresh straight ahead.
        obs = SemanticObs()
        t0 = time.time() - 1.6
        grounder = {"det": TD._det_at_bearing(0.0), "t_det": t0 + 1.25, "seq": 1,
                    "tier": "ok", "cap": {"t_capture": t0 + 1.2, "pose": {"yaw": 0.0}}}
        p = D.make_vla_perturber(f"127.0.0.1:{srv.port}", speed_words=True,
                                 encode=lambda m: bytes(m["data"]),
                                 **dict(kw, obs=obs, box_source=lambda: grounder))
        try:
            assert p.frames.speed_words == SW and p.config["speed_words"]["adapter"] == \
                SW.as_dict()
            assert "--vla-speed-words" in p.config["vla_fly_argv"]
            assert "own speed as words" in p.describe()
            for k in range(14):
                t = t0 + 0.1 * k
                st = 9_000_000_000 + k * 100_000_000
                obs.put_pose_full(t, 2.0 * 0.1 * k, 0.0, 8.0, 0.0, 0.0, 0.0, stamp=st)
                p.put_front({"data": f"f-{k}".encode(), "width": 256, "height": 256,
                             "time_stamp": st})
                p.put_down({"data": f"d-{k}".encode(), "width": 256, "height": 256,
                            "time_stamp": st})
            assert p._request(0) == "sent", dict(p.counts)
            end = time.time() + 5.0
            while not srv.requests and time.time() < end:
                time.sleep(0.02)
            assert srv.requests, "the window's request never reached the server"
            assert srv.requests[-1]["object"] == "a red car while flying", srv.requests[-1]
            meta, _k = next(iter(p._meta.values()))
            assert meta["own_speed"]["words"] == "while flying"
            assert math.isclose(meta["own_speed"]["speed_mps"], 2.0, abs_tol=1e-6)
        finally:
            p.stop()


def test_follow_vlm_and_the_launcher_pass_speed_words_only_with_a_dagger_recording():
    import follow_vlm as F_
    a = F_.build_arg_parser().parse_args([])
    assert a.perturb_vla_speed_words is False
    b = F_.build_arg_parser().parse_args(["--record-vla", "d", "--record-perturb-vla", NO_PORT,
                                          "--perturb-vla-speed-words"])
    assert b.perturb_vla_speed_words is True
    try:
        with contextlib.redirect_stderr(io.StringIO()):
            F_.main(["--record-vla", "d", "--perturb-vla-speed-words"])
    except SystemExit as exc:
        assert exc.code == 2, exc.code
    else:
        raise AssertionError("--perturb-vla-speed-words was accepted without a DAgger recording")
    src = (ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8")
    call = src[src.index("vla_dagger.make_vla_perturber("):]
    call = call[:call.index(")\n")]
    assert "speed_words=args.perturb_vla_speed_words" in call, call
    chk = src[src.index("why = check_args(args.record_perturb_vla"):]
    assert "speed_words=args.perturb_vla_speed_words" in chk[:chk.index(")\n")]
    # The launcher: a switch, passed inside the -RecordPerturbVla block,
    # refused without it before a simulator is started.
    ps = PS1.read_text(encoding="utf-8")
    assert "[switch]$PerturbVlaSpeedWords" in ps
    flag = '"--perturb-vla-speed-words"'
    assert ps.count(flag) == 1, flag
    lines = ps.splitlines()
    fi = next(n for n, ln in enumerate(lines) if flag in ln)
    gi = max(n for n, ln in enumerate(lines[:fi])
             if ln.lstrip().startswith("if ($RecordPerturbVla)"))
    ind = len(lines[gi]) - len(lines[gi].lstrip())
    for ln in lines[gi + 1:fi + 1]:
        assert not ln.strip() or len(ln) - len(ln.lstrip()) > ind, ln
    sim = ps.index("if (-not $SkipSim) { Start-Sim }")
    assert ps.index("($PerturbVlaSpeedWords -and -not $RecordPerturbVla)") < sim


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
