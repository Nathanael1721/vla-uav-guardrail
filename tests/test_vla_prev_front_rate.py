"""
The follow fine-tune's two input/output fixes (2026-10-24): the mosaic's
lower tile as the FRONT frame from dt seconds earlier (prev_front), and a
step flown as a RATE over its label horizon.

Run either way:
    pytest tests/test_vla_prev_front_rate.py -v
    python tests/test_vla_prev_front_rate.py

WHAT IS AT STAKE

The first fine-tuned follow (demo/out/vla_follow_ft_01_20261024, adapter
follow_v2_r1/epoch4) showed two root causes:

  1. One image cannot tell a STOPPED car from one PULLING AWAY. The car
     stopped at a light ~9 m ahead, the VLA held (right), the car drove off
     and the VLA went on answering fwd 0.0 for ~6 s while the car went from
     9.8 to 27.6 m. The expert knows the car's speed from its box history;
     the model sees one frame, and the mosaic's lower tile is the DOWN
     camera (the road under the aircraft). prev_front puts the front frame
     from dt_s earlier there instead.
  2. The tracker flew each step as an ENDPOINT at cruise (4.5 m/s), braking
     into it, while the label is the expert's displacement over the next
     1.0 s: a 2 m label is the expert flying 2 m/s. --step-as-rate H flies
     fwd / H and dpsi / H until the next answer replaces them.

So, pinned here (both default OFF; every existing run, dataset and test is
unchanged):

  A. ONE TILE RULE (guardrail/aerialvla_codec.pick_prev_front): the lower
     tile is the front frame whose capture time is nearest t_capture - dt_s
     within tol_s (tol_s < dt_s, so strictly earlier); with none, no sample
     and no request - never the down frame, never black. The dataset builder
     (tools/build_follow_dataset.py --bottom-tile prev_front) and the flight
     (vla_mission.VLAFrames) both use it; the mirror still flips both tiles;
     stats.json and every sample say the mode and the measured dt.
  B. PROVENANCE: the trainer copies the dataset's tile into the checkpoint's
     one provenance file, the server announces it in hello, and vla_fly and
     the DAgger recorder REFUSE before take-off when the adapter's tile is
     not the flight's (an adapter without the field is "down"); the
     evaluation tool sends whatever lower tile the dataset holds and says
     when the adapter was trained on another.
  C. RATE EXECUTION (guardrail/vla_link.TrackerConfig.rate_horizon_s):
     speed fwd / H along the body heading (speed cap, cos(heading error)
     under turn_while_moving), yaw rate dpsi / H (yaw cap) until the heading
     reaches capture yaw + dpsi, replaced at once by the next answer, and
     past H x (1 + overrun) ended by the existing carry / hold rules; the
     smoothing applies; every tick records the commanded rates.
  D. vla_fly's --vla-bottom-tile / --vla-prev-dt-s / --step-as-rate and the
     DAgger recorder (follow_vlm --perturb-vla-bottom-tile ...,
     run_citylife_follow.ps1 -PerturbVlaBottomTile ...) follow the same
     settings through vla_fly's own parser.

No simulator, GPU or model: synthetic recordings, a fake pose ring, a stub
client on a fake clock, tests/vla_fake_server.py on an ephemeral loopback
port (never 8790). The mosaic, evaluation and training-loop tests need
Pillow (and torch) and SKIP without them (GUARDRAIL_REQUIRE_PILLOW=1 turns a
missing Pillow into a failure).

Shown failing first: at c9dc0a1 none of aerialvla_codec.BottomTile /
pick_prev_front / adapter_bottom_tile, the builder's --bottom-tile, the
trainer's dataset_bottom_tile, VLAFrames(bottom_tile=), Capture.lower,
TrackerConfig.rate_horizon_s, vla_fly's flags or the DAgger's existed
(every test below fails at an AttributeError, a TypeError or an unknown
flag).
"""
from __future__ import annotations

import hashlib
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
from guardrail.models import Action4D, State, load_policy            # noqa: E402
from guardrail.vla_link import (Capture, Decision, Hint, Pose, Step,  # noqa: E402
                                StepTracker, TrackerConfig, build_infer_fields, wrap_pi)

SKIP = "SKIP"
POLICY = ROOT / "policies" / "vla_goto_citylife.yaml"
PS1 = ROOT / "scripts" / "run_citylife_follow.ps1"
NO_PORT = "127.0.0.1:59999"      # parsed only, never connected to
PREV = C.BottomTile("prev_front", 1.0, 0.25) if hasattr(C, "BottomTile") else None
FOLLOW = dict(cruise_mps=4.5, speed_cap_mps=5.0, turn_while_moving=True, anchor_at_answer=True,
              land_as_arrival=True, hold_altitude_m=10.0)


def _B():
    import build_follow_dataset as B
    return B


def _TF():
    """tests/test_follow_dataset.py's synthetic recordings (the recorder's
    own file format)."""
    import test_follow_dataset as TF
    return TF


def _TD():
    """tests/test_vla_dagger.py's stub client, point vehicle and identity."""
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


def _prev_identity(tile: dict | None, model: dict | None = None) -> dict:
    """A fine-tune's hello identity whose adapter provenance records `tile`
    (None: a provenance block without the field, as a fine-tune from before
    2026-10-24 announces itself)."""
    TD = _TD()
    m = json.loads(json.dumps(model or TD.FT_MODEL))
    prov = {"id_root": "aerialvla-ft-follow", "run": "r1", "epoch": 2, "epoch_dir": "epoch2"}
    if tile is not None:
        prov["bottom_tile"] = tile
        prov["label_horizon_s"] = 1.0
    m["adapter"]["provenance"] = prov
    return m


# =========================================================================== #
# A. The shared tile rule
# =========================================================================== #
def test_the_tile_rule_is_the_nearest_earlier_front_within_the_tolerance():
    assert C.BOTTOM_TILE_MODES == ("down", "prev_front")
    assert (C.DEFAULT_PREV_DT_S, C.DEFAULT_PREV_TOL_S) == (1.0, 0.25)
    down = C.BottomTile()
    assert down.mode == "down" and down.dt_s is None and down.tol_s is None
    assert down.as_dict() == {"mode": "down", "dt_s": None, "tol_s": None}
    assert C.bottom_tile("prev_front") == C.BottomTile("prev_front", 1.0, 0.25)
    assert C.bottom_tile("prev_front", 0.5, 0.1).as_dict() == \
        {"mode": "prev_front", "dt_s": 0.5, "tol_s": 0.1}
    assert C.bottom_tile("down", 0.5, 0.1) == down, "a down tile has no dt"
    for bad in (("side",), ("prev_front", 0.0), ("prev_front", -1.0), ("prev_front", 1.0, 0.0),
                ("prev_front", 1.0, 1.0), ("prev_front", 0.2, 0.25),       # tol >= dt
                ("prev_front", float("nan")), ("prev_front", 1.0, float("inf"))):
        try:
            C.bottom_tile(*bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")
    # A record: absent / None / no mode is "down"; a prev_front one keeps dt.
    assert C.bottom_tile_from(None) == down and C.bottom_tile_from({}) == down
    assert C.bottom_tile_from({"mode": "down", "dt_s": None}) == down
    assert C.bottom_tile_from({"mode": "prev_front", "dt_s": 1.0, "tol_s": 0.25,
                               "dt_actual_s": 0.98}) == PREV
    assert C.bottom_tile_from({"mode": "prev_front", "dt_s": 0.5}).dt_s == 0.5
    for bad in ({"mode": "rear"}, {"mode": "prev_front"}, "prev_front", 7):
        try:
            C.bottom_tile_from(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad!r}")
    # The rule: nearest t - dt within tol, from frames strictly earlier.
    times = [10.0, 10.1, 10.2, 10.35, 10.9, 11.0]
    assert C.pick_prev_front(11.2, times, 1.0, 0.25) == 2            # 10.2 for 10.2
    assert C.pick_prev_front(11.3, times, 1.0, 0.25) == 3            # 10.35 (0.05) over 10.2
    assert C.pick_prev_front(11.15, times, 1.0, 0.25) in (1, 2)      # a tie: either neighbour
    assert C.pick_prev_front(11.59, times, 1.0, 0.25) == 3           # 0.24 off: 10.35
    assert C.pick_prev_front(11.62, times, 1.0, 0.25) is None        # 10.35 0.27, 10.9 0.28 off
    assert C.pick_prev_front(10.7, times, 1.0, 0.25) is None         # 10.0 is 0.3 late
    assert C.pick_prev_front(10.76, times, 1.0, 0.25) == 0
    assert C.pick_prev_front(10.74, times, 1.0, 0.25) is None        # 10.0 is 0.26 early
    assert C.pick_prev_front(11.0, [], 1.0, 0.25) is None
    # Never the capture's own frame or a later one, whatever the tolerance.
    assert C.pick_prev_front(11.0, [11.0, 11.1], 1.0, 0.25) is None


def test_the_flight_and_the_adapter_must_carry_the_same_tile():
    down = C.BottomTile()
    assert C.adapter_bottom_tile({"id": "x"}) == (down, "absent")
    assert C.adapter_bottom_tile(None) == (down, "absent")
    assert C.adapter_bottom_tile(_prev_identity(None)) == (down, "absent")
    tile, src = C.adapter_bottom_tile(_prev_identity(PREV.as_dict()))
    assert tile == PREV and src == "announced"
    assert C.bottom_tile_mismatch(PREV, PREV) is None
    assert C.bottom_tile_mismatch(down, down) is None
    why = C.bottom_tile_mismatch(down, PREV)
    assert why and "down" in why and "prev_front" in why, why
    why = C.bottom_tile_mismatch(PREV, down)
    assert why and "prev_front" in why and "down" in why, why
    why = C.bottom_tile_mismatch(PREV, C.bottom_tile("prev_front", 0.5, 0.1))
    assert why and "1" in why and "0.5" in why, why
    # The tolerance is sampling slack, not the input: it does not refuse.
    assert C.bottom_tile_mismatch(PREV, C.bottom_tile("prev_front", 1.0, 0.1)) is None


# =========================================================================== #
# A. The dataset builder
# =========================================================================== #
def test_the_builder_puts_the_front_frame_one_second_earlier_in_the_lower_tile():
    B, TF = _B(), _TF()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TF._write_run(td, "runP", TF._straight(speed=4.0), seconds=8.0)
        caps = {c["i"]: c for c in _rows(run / "captures.jsonl")}
        st = B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False,
                             bottom_tile="prev_front", prev_dt_s=1.0, prev_tol_s=0.25)
        rows = _rows(td / "ds" / "samples.jsonl")
        # captures 0..7 (t < 0.75 s) have no front frame 0.75-1.25 s before
        assert st["dropped"]["no_prev_front"] == 8, st["dropped"]
        assert "no_prev_front" in B.DROP_ORDER
        assert min(r["i"] for r in rows) == 8 and rows
        for r in rows:
            bt = r["bottom_tile"]
            assert (bt["mode"], bt["dt_s"], bt["tol_s"]) == ("prev_front", 1.0, 0.25), bt
            # 1 s earlier on the 10 Hz grid; captures 8 and 9 take the first
            # frame, 0.8 and 0.9 s back (within the 0.25 s tolerance)
            j = max(0, r["i"] - 10)
            assert bt["prev_i"] == j, (r["i"], bt)
            assert bt["prev_front_jpg_sha256"] == caps[j]["front_jpg_sha256"]
            assert math.isclose(bt["dt_actual_s"], 0.1 * (r["i"] - j), abs_tol=1e-6), bt
            assert math.isclose(bt["prev_t_capture"], caps[j]["t_capture"], abs_tol=1e-6)
            # the capture's own two shas stay as recorded
            assert r["front_jpg_sha256"] == caps[r["i"]]["front_jpg_sha256"]
        bts = st["bottom_tile"]
        assert (bts["mode"], bts["dt_s"], bts["tol_s"]) == ("prev_front", 1.0, 0.25), bts
        assert bts["no_prev_front"] == 8 and bts["dt_actual_s"]["n"] == len(rows)
        assert math.isclose(bts["dt_actual_s"]["p50"], 1.0, abs_tol=1e-4), bts
        assert st["args"]["bottom_tile"] == "prev_front" and st["args"]["prev_dt_s"] == 1.0
        assert st["runs"]["runP"]["bottom_tile_dt_actual_s"]["n"] == len(rows)
        # The labels, phrases and prompts are the down build's, sample for sample.
        sd = B.build_dataset([run], td / "dsd", horizon_s=1.0, stride_s=0.0, images=False)
        drows = {r["i"]: r for r in _rows(td / "dsd" / "samples.jsonl")}
        assert sd["bottom_tile"]["mode"] == "down" and sd["bottom_tile"]["dt_actual_s"] is None
        assert "no_prev_front" not in sd["dropped"]
        assert all(r["bottom_tile"] == {"mode": "down", "dt_s": None, "tol_s": None}
                   for r in drows.values())
        assert set(drows) - {r["i"] for r in rows} == set(range(8))
        for r in rows:
            d = drows[r["i"]]
            for key in ("label", "bins", "action_text", "phrase", "prompt", "label_raw"):
                assert r[key] == d[key], (r["i"], key)


def test_the_builder_takes_any_earlier_capture_and_drops_a_gap_wider_than_the_tolerance():
    """Captures on a 10 Hz grid with holes: 25..29 never written (a writer
    stall), the front JPEGs of 8 and 9 lost (FrameSaver), capture 5 with no
    box (no sample itself, still a frame), capture 40 without its down JPEG.
    Each sample's lower tile is the nearest earlier front within 0.25 s of
    t - 1 s among the frames that exist; none, no sample."""
    B, TF = _B(), _TF()

    def box(k, t, yaw_at):
        if k == 5:
            return None
        return {"det": TF._det_at_bearing(0.0), "t_box": t - 0.3, "yaw_box": yaw_at(t - 0.3),
                "t_published": t - 0.05, "seq": k, "tier": "ok"}

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TF._write_run(td, "runG", TF._straight(), seconds=6.0, box=box)
        caps = _rows(run / "captures.jsonl")
        by_i = {c["i"]: c for c in caps}
        kept = [c for c in caps if not 25 <= c["i"] <= 29]
        (run / "captures.jsonl").write_text("".join(json.dumps(c) + "\n" for c in kept),
                                            encoding="utf-8", newline="\n")
        for k in (8, 9):
            (run / "frames" / f"{by_i[k]['front_jpg_sha256']}.jpg").unlink()
        (run / "frames" / f"{by_i[40]['down_jpg_sha256']}.jpg").unlink()
        st = B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False,
                             bottom_tile="prev_front")
        rows = {r["i"]: r for r in _rows(td / "ds" / "samples.jsonl")}
        prev = {i: r["bottom_tile"]["prev_i"] for i, r in rows.items()}
        assert prev[15] == 5, "a capture with no box is still a frame of the ring"
        assert prev[17] == 7 and prev[18] == 7 and prev[19] == 10, (prev[17], prev[18], prev[19])
        assert prev[36] == 24 and prev[38] == 30, (prev[36], prev[38])
        assert math.isclose(rows[36]["bottom_tile"]["dt_actual_s"], 1.2, abs_tol=1e-6)
        assert math.isclose(rows[38]["bottom_tile"]["dt_actual_s"], 0.8, abs_tol=1e-6)
        assert 37 not in rows, "2.4 and 3.0 are both 0.3 s off 2.7"
        assert 40 in rows, "prev_front does not need the down frame"
        assert 8 not in rows and 9 not in rows, "their own front frames are missing"
        # 0..7 and 37: no earlier front; 8 and 9 frame_missing first.
        assert st["dropped"]["no_prev_front"] == 9, st["dropped"]
        assert st["dropped"]["frame_missing"] == 2, st["dropped"]
        sp = st["bottom_tile"]["dt_actual_s"]
        assert 0.75 <= sp["min"] and sp["max"] <= 1.25, sp
        # The same recording in down mode: 40 lacks its down frame.
        sd = B.build_dataset([run], td / "dsd", horizon_s=1.0, stride_s=0.0, images=False)
        assert 40 not in {r["i"] for r in _rows(td / "dsd" / "samples.jsonl")}
        assert sd["dropped"]["frame_missing"] == 3, sd["dropped"]


def test_the_builder_cli_takes_the_tile_flags_and_refuses_what_it_would_not_use():
    B, TF = _B(), _TF()
    import contextlib
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TF._write_run(td, "runC", TF._straight(), seconds=4.0)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rc = B.main([str(run), "--out", str(td / "ds"), "--no-images", "--bottom-tile",
                         "prev_front", "--prev-dt-s", "1.0", "--prev-tol-s", "0.25"])
        assert rc == 0
        st = json.loads((td / "ds" / "stats.json").read_text(encoding="utf-8"))
        assert st["bottom_tile"]["mode"] == "prev_front" and st["bottom_tile"]["dt_s"] == 1.0
        assert "prev_front" in out.getvalue(), "the console says which lower tile"
        for k, argv in enumerate((["--prev-dt-s", "1.0"],                    # with down
                                  ["--prev-tol-s", "0.2"],
                                  ["--bottom-tile", "prev_front", "--prev-dt-s", "0.2"],
                                  ["--bottom-tile", "prev_front", "--prev-dt-s", "-1"])):
            with contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
                rc = B.main([str(run), "--out", str(td / f"bad{k}"), "--no-images", *argv])
            assert rc == 2, argv
            assert not (td / f"bad{k}" / "samples.jsonl").exists(), argv


def test_the_prev_front_mosaic_is_the_codecs_and_its_mirror_flips_both_tiles():
    Image = _pil()
    if Image is None:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    from PIL import ImageOps
    B, TF = _B(), _TF()

    def jpeg(k):
        out = []
        for col in ((200, 30 + 4 * k, 40), (20, 20, 120)):
            im = Image.new("RGB", (256, 256), col)
            for y in range(0, 256, 4):
                im.putpixel(((y * 3 + 7 * k) % 128, y), (255, 255, 0))      # left half only
            buf = io.BytesIO()
            im.save(buf, format="JPEG", quality=95)
            out.append(buf.getvalue())
        return tuple(out)

    def decode(b):
        import base64
        return C.decode_image_b64(base64.b64encode(b).decode("ascii"))

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TF._write_run(td, "runM", TF._straight(), seconds=4.0, jpeg=jpeg)
        B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.5, images=True,
                        mirror=True, bottom_tile="prev_front")
        rows = _rows(td / "ds" / "samples.jsonl")
        real = {r["id"]: r for r in rows if not r["mirrored"]}
        assert real and any(r["mirrored"] for r in rows)
        w, h = C.MOSAIC_TILE
        for r in real.values():
            fr = (run / "frames" / f"{r['front_jpg_sha256']}.jpg").read_bytes()
            pv = (run / "frames" / f"{r['bottom_tile']['prev_front_jpg_sha256']}.jpg").read_bytes()
            dn = (run / "frames" / f"{r['down_jpg_sha256']}.jpg").read_bytes()
            m, _ = C.build_mosaic(decode(fr), decode(pv), missing_down="refuse")
            assert r["mosaic_sha256"] == C.mosaic_sha256(m), r["id"]
            md, _ = C.build_mosaic(decode(fr), decode(dn), missing_down="refuse")
            assert r["mosaic_sha256"] != C.mosaic_sha256(md), "the down frame is not the tile"
            img = Image.open(td / "ds" / r["image"]).convert("RGB")
            assert C.mosaic_sha256(img) == r["mosaic_sha256"]
        for r in rows:
            if not r["mirrored"]:
                continue
            s = real[r["mirror_of"]]
            assert r["bottom_tile"] == s["bottom_tile"], "the mirror keeps the tile's record"
            a = Image.open(td / "ds" / s["image"]).convert("RGB")
            b = Image.open(td / "ds" / r["image"]).convert("RGB")
            assert C.mosaic_sha256(b) == C.mosaic_sha256(ImageOps.mirror(a)) == r["mosaic_sha256"]
            for box in ((0, 0, w, h), (0, h, w, 2 * h)):                    # each tile in place
                assert ImageOps.mirror(a.crop(box)).tobytes() == b.crop(box).tobytes(), box


# =========================================================================== #
# B. The trainer, the server, the evaluation tool
# =========================================================================== #
def _dataset(d: Path, tiles, stats_tile=None, horizon=1.0) -> Path:
    """samples.jsonl (one record per entry of `tiles`: a bottom_tile record,
    or None for none) and, unless stats_tile is None, a stats.json."""
    import finetune_aerialvla_follow as FT
    d.mkdir(parents=True, exist_ok=True)
    rows = []
    for k, t in enumerate(tiles):
        r = {"image": f"images/{k}.png", "prompt": C.build_prompt("straight ahead ", "a red car"),
             "target": "45 49 49", "run": "r1" if k % 3 else "r3",
             "split": "train" if k % 3 else "holdout"}
        if t is not None:
            r["bottom_tile"] = t
        rows.append(r)
    (d / FT.samples_file(d).name).write_text("".join(json.dumps(r) + "\n" for r in rows),
                                            encoding="utf-8")
    if stats_tile is not None:
        (d / "stats.json").write_text(json.dumps({"bottom_tile": stats_tile,
                                                  "args": {"horizon_s": horizon}}),
                                      encoding="utf-8")
    return d


def test_the_trainer_reads_the_datasets_tile_and_writes_it_into_the_provenance():
    import finetune_aerialvla_follow as FT
    import vla_server_v2 as S
    pf = dict(PREV.as_dict(), dt_actual_s=1.01, prev_i=3)
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        # A dataset from before 2026-10-24: no field anywhere -> down.
        d0 = _dataset(td / "d0", [None] * 6)
        s0, _ = FT.load_samples(d0, check_images=False)
        t0 = FT.dataset_bottom_tile(d0, s0)
        assert (t0["mode"], t0["dt_s"]) == ("down", None) and "absent" in t0["source"], t0
        # stats.json and every sample: prev_front, the label horizon read too.
        d1 = _dataset(td / "d1", [pf] * 6, stats_tile=dict(PREV.as_dict(), dt_actual_s={}))
        s1, _ = FT.load_samples(d1, check_images=False)
        t1 = FT.dataset_bottom_tile(d1, s1)
        assert (t1["mode"], t1["dt_s"], t1["tol_s"]) == ("prev_front", 1.0, 0.25), t1
        assert t1["source"] == "stats.json" and t1["label_horizon_s"] == 1.0, t1
        assert FT.dataset_bottom_tile(d1 / "samples.jsonl", s1)["mode"] == "prev_front"
        # Refused: stats and samples disagree, samples mix, dt differs.
        for name, tiles, stats in (
                ("d2", [None] * 6, PREV.as_dict()),
                ("d3", [pf] * 3 + [None] * 3, None),
                ("d4", [dict(pf, dt_s=0.5)] * 6, PREV.as_dict())):
            d = _dataset(td / name, tiles, stats_tile=stats)
            s, _ = FT.load_samples(d, check_images=False)
            try:
                FT.dataset_bottom_tile(d, s)
            except FT.DataError as exc:
                assert "tile" in str(exc), exc
                continue
            raise AssertionError(f"{name} was accepted")
        # The dry run says it, and refuses a dataset that mixes tiles.
        rep = td / "rep.json"
        rc = FT.main(["--data", str(d1), "--run", "x", "--dry-run", "1", "--no-image-check",
                      "--base", str(td / "no-such-base"), "--report", str(rep)])
        assert json.loads(rep.read_text(encoding="utf-8"))["bottom_tile"]["mode"] == "prev_front"
        rc = FT.main(["--data", str(td / "d3"), "--run", "x", "--dry-run", "1",
                      "--no-image-check", "--base", str(td / "no-such-base"), "--report",
                      str(rep)])
        assert rc == 1 and "tile" in json.loads(rep.read_text(encoding="utf-8"))["refused"]
        # The provenance: the tile and the label horizon; down by default.
        kw = dict(run="r", epoch=1, parent_adapter=FT.DEFAULT_ADAPTER, parent_sha256="ab" * 32,
                  base=FT.DEFAULT_BASE, data_report={"file": "x", "sha256": "cd" * 32},
                  split_info={"train": 4, "held_out": 2, "train_runs": ["r1"],
                              "held_out_runs": ["r3"]},
                  precision="bf16", hyper={}, metrics=None, step=3)
        assert FT.provenance(**kw)["bottom_tile"] == C.BottomTile().as_dict()
        prov = FT.provenance(**kw, bottom_tile=t1, label_horizon_s=t1["label_horizon_s"])
        assert prov["bottom_tile"] == PREV.as_dict() and prov["label_horizon_s"] == 1.0
        # The server announces it in hello; the codec reads it back.
        ad = td / "ft" / "r" / "epoch1"
        ad.mkdir(parents=True)
        (ad / "adapter_config.json").write_text("{}", encoding="utf-8")
        (ad / C.ADAPTER_PROVENANCE_FILE).write_text(json.dumps(prov), encoding="utf-8")
        info = S.adapter_identity(ad, "1a" * 32)
        assert info["provenance"]["bottom_tile"] == PREV.as_dict(), info["provenance"]
        assert info["provenance"]["label_horizon_s"] == 1.0
        assert C.adapter_bottom_tile({"adapter": info}) == (PREV, "announced")
        # an older checkpoint's file has no field: announced as None -> down
        old = dict(prov)
        del old["bottom_tile"]
        (ad / C.ADAPTER_PROVENANCE_FILE).write_text(json.dumps(old), encoding="utf-8")
        info = S.adapter_identity(ad, "1a" * 32)
        assert info["provenance"]["bottom_tile"] is None
        assert C.adapter_bottom_tile({"adapter": info}) == (C.BottomTile(), "absent")


def test_the_training_loop_writes_the_tile_into_the_one_provenance_file():
    Image = _pil()
    try:
        import torch                                               # noqa: F401
    except ImportError:
        torch = None
    if torch is None or Image is None:
        print("      SKIP: torch and Pillow are needed (the vla-real env has both)")
        return SKIP
    import contextlib
    import finetune_aerialvla_follow as FT
    import test_finetune_follow as TFT
    d = TFT._synthetic_dataset(Image)
    rows = _rows(d / "samples.jsonl")
    for r in rows:
        r["bottom_tile"] = dict(PREV.as_dict(), dt_actual_s=1.0)
    (d / "samples.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                     encoding="utf-8")
    (d / "stats.json").write_text(json.dumps({"bottom_tile": PREV.as_dict(),
                                              "args": {"horizon_s": 1.0}}), encoding="utf-8")
    parent = Path(tempfile.mkdtemp(prefix="pfr_parent_"))
    (parent / "adapter_config.json").write_text("{}", encoding="utf-8")
    out = Path(tempfile.mkdtemp(prefix="pfr_out_"))
    ap = FT.build_arg_parser()
    args = ap.parse_args(["--data", str(d), "--run", "pf", "--epochs", "1", "--batch-size", "2",
                          "--grad-accum", "1", "--eval-max", "0", "--device", "cpu",
                          "--no-eval-before", "--adapter", str(parent), "--out-root", str(out),
                          "--base", "unused"])
    with contextlib.redirect_stdout(io.StringIO()):
        assert FT.train(args, loader=TFT._tiny_loader("96 49 49")) == 0
    ck = out / "pf" / "epoch1"
    assert sorted(p.name for p in ck.iterdir()) == \
        sorted(["adapter_config.json", "adapter_model.bin", C.ADAPTER_PROVENANCE_FILE])
    prov = json.loads((ck / C.ADAPTER_PROVENANCE_FILE).read_text(encoding="utf-8"))
    assert prov["bottom_tile"] == PREV.as_dict() and prov["label_horizon_s"] == 1.0, prov
    rec = json.loads((out / "pf" / "run.json").read_text(encoding="utf-8"))
    assert rec["bottom_tile"]["mode"] == "prev_front"


def test_the_evaluation_tool_says_when_the_adapter_was_trained_on_another_tile():
    import eval_vla_adapter as EV
    pf = _prev_identity(PREV.as_dict())
    assert EV.tile_check(pf, PREV.as_dict()) == "ok"
    assert EV.tile_check({"id": "fake:scripted"}, {"mode": "down"}) == "ok"
    why = EV.tile_check({"id": "fake:scripted"}, PREV.as_dict())
    assert why != "ok" and "prev_front" in why, why
    why = EV.tile_check(pf, {"mode": "down", "dt_s": None, "tol_s": None})
    assert why != "ok" and "down" in why, why
    src = (ROOT / "tools" / "eval_vla_adapter.py").read_text(encoding="utf-8")
    assert "FT.dataset_bottom_tile(" in src and '"bottom_tile_check"' in src


def test_the_evaluation_sends_the_prev_front_tile_and_the_server_rebuilds_the_mosaic():
    Image = _pil()
    if Image is None:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    import finetune_aerialvla_follow as FT
    import eval_vla_adapter as EV
    B, TF = _B(), _TF()

    def jpeg(k):
        out = []
        for col in ((180, 20 + 5 * k, 60), (10, 90, 10)):
            im = Image.new("RGB", (256, 256), col)
            buf = io.BytesIO()
            im.save(buf, format="JPEG", quality=95)
            out.append(buf.getvalue())
        return tuple(out)

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        ra = TF._write_run(td, "ra", TF._straight(), seconds=4.0, jpeg=jpeg)
        rb = TF._write_run(td, "rb", TF._straight(), seconds=4.0, jpeg=jpeg)
        B.build_dataset([ra, rb], td / "ds", holdout=["rb"], images=True,
                        bottom_tile="prev_front")
        samples, _ = FT.load_samples(td / "ds")
        held = [s for s in samples if s.split == "holdout"]
        assert held
        m = FT.load_mosaic(held[0].image, held[0].image_sha256)
        req = EV.build_request(held[0], m, 1)
        w, h = C.MOSAIC_TILE
        lower = C.decode_image_b64(req["down_jpg_b64"])
        assert lower.tobytes() == m.crop((0, h, w, 2 * h)).tobytes(), "the down slot is the tile"
        res = EV.evaluate_fake([held[0].target + "</s>"] * len(held), held)
        assert res["fidelity"]["answered"] == len(held)
        assert res["fidelity"]["image_sha256_mismatch"] == 0, res["fidelity"]


# =========================================================================== #
# A. The flight: VLAFrames
# =========================================================================== #
class _Obs:
    """A pose ring whose sim clock maps 1:1 onto wall time + 1000 s."""

    def pose_at(self, t, stamp=None):
        return {"x": 1.0, "y": 2.0, "up": 10.0, "yaw": 0.5, "src": "ring-stamp"}

    def wall_at_stamp(self, stamp):
        return None if stamp is None else 1000.0 + float(stamp) / 1e9


def _frames(tile=None, **kw):
    import vla_mission as VM
    clock = {"t": 1000.0}
    fr = VM.VLAFrames(_Obs(), encode=lambda m: m["jpg"], clock=lambda: clock["t"],
                      bottom_tile=tile, **kw)
    return fr, clock


def _put(fr, clock, ks, down=True):
    """Front frames k at sim time k x 0.1 s, arriving 30 ms later; a down
    frame beside each (prev_front must not use it)."""
    for k in ks:
        clock["t"] = 1000.0 + k * 0.1 + 0.03
        fr.put_front({"jpg": f"F{k}".encode(), "time_stamp": k * 100_000_000,
                      "width": 256, "height": 256})
        if down:
            fr.put_down({"jpg": f"D{k}".encode(), "time_stamp": k * 100_000_000,
                         "width": 256, "height": 256})


def test_vlaframes_sends_the_front_frame_one_second_earlier_in_the_down_slot():
    fr, clock = _frames(PREV)
    assert fr.tile == PREV
    _put(fr, clock, range(0, 13))
    cap = fr.capture()
    assert cap is not None and cap.front_jpg == b"F12" and cap.down_jpg == b"F2", cap
    assert math.isclose(cap.t_capture, 1001.2, abs_tol=1e-9)
    lo = cap.lower
    assert (lo["mode"], lo["dt_s"], lo["tol_s"]) == ("prev_front", 1.0, 0.25), lo
    assert math.isclose(lo["dt_actual_s"], 1.0, abs_tol=1e-6) and lo["prev_stamp_ns"] == 200_000_000
    # The request's record says so; the wire request itself is protocol v2.
    fields, meta = build_infer_fields(cap, Hint("straight ahead ", "detector"), "a red car")
    C.validate_request(dict(fields, seq=1))
    assert meta["bottom_tile"]["mode"] == "prev_front" and meta["bottom_tile"]["dt_actual_s"] == \
        lo["dt_actual_s"]
    assert meta["down_jpg_sha256"] == hashlib.sha256(b"F2").hexdigest()
    fr.commit(cap, Hint("straight ahead ", "detector"), 5)
    rec = fr.log[-1]
    assert rec["bottom_tile"]["mode"] == "prev_front" and rec["bottom_tile"]["dt_actual_s"] == \
        lo["dt_actual_s"] and rec["down_jpg_sha256"] == hashlib.sha256(b"F2").hexdigest()
    st = fr.stats()
    assert st["bottom_tile"]["mode"] == "prev_front" and st["bottom_tile"]["dt_actual_s"]["n"] == 1
    # No down camera at all: prev_front still asks.
    fr2, clock2 = _frames(PREV)
    _put(fr2, clock2, range(0, 13), down=False)
    cap2 = fr2.capture()
    assert cap2 is not None and cap2.down_jpg == b"F2" and fr2.counts["no_down"] == 0
    # A hole around t - 1 s: the nearest within the tolerance, else nothing.
    fr3, clock3 = _frames(PREV)
    _put(fr3, clock3, [0, 5, 6, 7, 8, 9, 10, 11, 12])
    assert fr3.capture().down_jpg == b"F0", "0.2 s off beats 0.3 s off"
    fr4, clock4 = _frames(C.bottom_tile("prev_front", 0.5, 0.1))
    _put(fr4, clock4, range(0, 13))
    c4 = fr4.capture()
    assert c4.down_jpg == b"F7" and math.isclose(c4.lower["dt_actual_s"], 0.5, abs_tol=1e-6)
    # The ring keeps about dt + tol + its margin of frames, not the flight.
    fr5, clock5 = _frames(PREV)
    _put(fr5, clock5, range(0, 400))
    assert len(fr5._fronts) <= 30, len(fr5._fronts)
    assert fr5.capture().down_jpg == b"F389"


def test_vlaframes_sends_nothing_without_an_earlier_front_and_never_the_down_frame():
    fr, clock = _frames(PREV)
    _put(fr, clock, range(0, 7))                     # 0.6 s of frames, down frames beside
    assert fr.capture() is None and fr.counts["no_prev_front"] == 1, dict(fr.counts)
    assert fr.counts["captures"] == 0
    fr3, clock3 = _frames(PREV)
    _put(fr3, clock3, range(5, 13))                  # nothing within 0.25 s of 1000.2
    assert fr3.capture() is None and fr3.counts["no_prev_front"] == 1
    # The backend sends nothing for such a tick (RemoteVLABackend asks
    # get_capture first): no request without the earlier front.
    from guardrail.vla_backends import RemoteVLABackend
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
    be = RemoteVLABackend(Client(), get_capture=fr.capture,
                          get_hint=lambda cap: Hint("straight ahead ", "detector"),
                          object_text="a red car", cadence="pipelined", clock=lambda: clock["t"])
    be.act(State(x=0.0, y=0.0, up=10.0, yaw_deg=0.0))
    assert not sent, "a request went out with no earlier front frame"


def test_vlaframes_down_mode_is_unchanged():
    fr, clock = _frames(None)
    assert fr.tile == C.BottomTile()
    _put(fr, clock, range(0, 13))
    cap = fr.capture()
    assert cap.down_jpg == b"D12" and cap.lower is None
    fields, meta = build_infer_fields(cap, Hint("straight ahead ", "detector"), "a red car")
    assert "bottom_tile" not in meta
    fr.commit(cap, Hint("straight ahead ", "detector"), 1)
    assert "bottom_tile" not in fr.log[-1] and "bottom_tile" not in fr.stats()
    assert len(fr._fronts) == 0, "down mode keeps no ring"
    fr2, _ = _frames(None)
    fr2.put_front({"jpg": b"F", "time_stamp": 0, "width": 256, "height": 256})
    assert fr2.capture() is None and fr2.counts["no_down"] == 1


# =========================================================================== #
# B. The refusal before take-off
# =========================================================================== #
def test_vla_fly_refuses_an_adapter_trained_on_another_tile():
    import vla_mission as VM
    import vla_fly as VF
    down = C.BottomTile()
    ok = VM.bottom_tile_check(_prev_identity(PREV.as_dict()), PREV, is_vla=True)
    assert ok["ok"] and ok["checked"] and ok["adapter"] == PREV.as_dict(), ok
    assert ok["flight"] == PREV.as_dict() and ok["source"] == "announced"
    for ident, flight in ((_prev_identity(None), PREV),           # an older fine-tune
                          (_TD().FT_MODEL, PREV),                  # no provenance at all
                          ({"id": "aerialvla-lora/aero_vla@196f2f32"}, PREV),
                          (_prev_identity(PREV.as_dict()), down),
                          (_prev_identity(PREV.as_dict()), C.bottom_tile("prev_front", 0.5, 0.1))):
        r = VM.bottom_tile_check(ident, flight, is_vla=True)
        assert r["checked"] and not r["ok"] and r["why"], (ident, flight, r)
    assert VM.bottom_tile_check(_TD().FT_MODEL, down, is_vla=True)["ok"]
    bad = _prev_identity({"mode": "sideways"})
    r = VM.bottom_tile_check(bad, PREV, is_vla=True)
    assert not r["ok"] and "malformed" in r["why"], r
    # A scripted stand-in reads no image: not checked, flown (and recorded) as one.
    st = VM.bottom_tile_check({"id": "fake:scripted"}, PREV, is_vla=False)
    assert st["ok"] and not st["checked"] and "stand-in" in st["source"], st
    # A server from before the field: its hello cannot say, the file here can.
    with tempfile.TemporaryDirectory() as td:
        ad = Path(td) / "epoch2"
        ad.mkdir()
        (ad / C.ADAPTER_PROVENANCE_FILE).write_text(json.dumps(
            {"id_root": "aerialvla-ft-follow", "run": "r1", "epoch_dir": "epoch2",
             "bottom_tile": PREV.as_dict()}), encoding="utf-8")
        ident = _prev_identity(None)
        ident["adapter"]["path"] = ad.as_posix()
        r = VM.bottom_tile_check(ident, PREV, is_vla=True)
        assert r["ok"] and r["source"].startswith("local"), r
        assert not VM.bottom_tile_check(ident, PREV, is_vla=True, read_local=False)["ok"]
    # _fly checks it right after hello, before the simulator and the arming,
    # and exits as a refusal.
    src = inspect.getsource(VF._fly)
    i = src.index("VM.bottom_tile_check(")
    assert src.index("server.wait_hello(") < i < src.index("sim.connect()") < \
        src.index("drone.arm()"), "the tile check is not before the simulator"
    block = src[i:i + 900]
    assert "raise SystemExit(EXIT_REFUSED)" in block and "server.stop()" in block, block
    assert '"bottom_tile": tile_check' in src, "params do not record the check"


def test_the_dagger_recorder_refuses_an_adapter_trained_on_another_tile():
    D, TD = _D(), _TD()
    import vla_fake_server as F
    from semantic_demo import SemanticObs
    kw = dict(seed=1, obs=SemanticObs(), box_source=lambda: None, policy=None,
              object_text="a red car", cruise_alt=8.0, record_dir=None)
    with F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0), model=TD.FT_MODEL) as srv:
        assert srv.port != 8790
        try:
            D.make_vla_perturber(f"127.0.0.1:{srv.port}", bottom_tile="prev_front", **kw)
        except D.DaggerRefused as exc:
            assert "lower tile" in str(exc) or "tile" in str(exc), exc
        else:
            raise AssertionError("a down adapter flew prev_front windows")
        p = D.make_vla_perturber(f"127.0.0.1:{srv.port}", **kw)
        try:
            assert p.frames.tile == C.BottomTile() and p.cfg.rate_horizon_s is None
            assert p.config["bottom_tile"]["ok"] and p.config["step_as_rate_s"] is None
        finally:
            p.stop()
    pm = _prev_identity(PREV.as_dict())
    with F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0), model=pm) as srv:
        assert srv.port != 8790
        try:
            D.make_vla_perturber(f"127.0.0.1:{srv.port}", **kw)
        except D.DaggerRefused as exc:
            assert "tile" in str(exc), exc
        else:
            raise AssertionError("a prev_front adapter flew down windows")
        obs = SemanticObs()
        p = D.make_vla_perturber(f"127.0.0.1:{srv.port}", bottom_tile="prev_front",
                                 prev_dt_s=1.0, step_as_rate=1.0,
                                 encode=lambda m: bytes(m["data"]), **dict(kw, obs=obs))
        try:
            assert p.frames.tile == PREV and p.cfg.rate_horizon_s == 1.0
            assert p.config["bottom_tile"]["adapter"] == PREV.as_dict()
            assert p.config["step_as_rate_s"] == 1.0
            assert "--vla-bottom-tile" in p.config["vla_fly_argv"]
            # A window's request is asked exactly as vla_fly would: the front
            # frame from 1 s earlier in the down slot. The frames' sim stamps
            # map onto the last 1.6 s of the wall clock (VLAFrames trusts a
            # stamp within 2 s of the frame's arrival, never later than it).
            import time
            t0 = time.time() - 1.6
            for k in range(14):
                t = t0 + 0.1 * k
                st = 9_000_000_000 + k * 100_000_000
                obs.put_pose_full(t, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, stamp=st)
                p.put_front({"data": f"f-{k}".encode(), "width": 256, "height": 256,
                             "time_stamp": st})
                p.put_down({"data": f"d-{k}".encode(), "width": 256, "height": 256,
                            "time_stamp": st})
            cap = p.get_capture()
            assert cap is not None and cap.front_jpg == b"f-13" and cap.down_jpg == b"f-3", cap
            assert cap.lower["mode"] == "prev_front"
        finally:
            p.stop()


# =========================================================================== #
# C. Rate execution
# =========================================================================== #
def test_rate_execution_is_off_by_default_and_validated():
    cfg = TrackerConfig()
    assert cfg.rate_horizon_s is None and cfg.rate_overrun_frac == 0.2
    assert TrackerConfig(rate_horizon_s=1.0).rate_horizon_s == 1.0
    for bad in ({"rate_horizon_s": 0.0}, {"rate_horizon_s": -1.0},
                {"rate_horizon_s": float("nan")}, {"rate_horizon_s": float("inf")},
                {"rate_overrun_frac": -0.1}, {"rate_overrun_frac": float("nan")}):
        try:
            TrackerConfig(**bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")
    # The default flies the step as an endpoint: no rate on any tick.
    tr = StepTracker(TrackerConfig(**FOLLOW))
    p = Pose(0.0, 0.0, 10.0, 0.0)
    tr.accept(Decision.of(1, 0.0, p, Step(3.0, 0.0, 0.1)), 0.0, pose_now=p)
    tk = tr.tick(0.0, p)
    assert tk.reason in ("translate", "turn_moving") and tr.last_rate is None
    assert "vla_rate" not in tk.as_row()


def test_a_step_is_flown_as_fwd_over_h_along_the_body_heading_and_dpsi_over_h():
    H = 1.0
    tr = StepTracker(TrackerConfig(**dict(FOLLOW, turn_while_moving=False), rate_horizon_s=H))
    cap = Pose(0.0, 0.0, 10.0, 0.3)
    d = Decision.of(1, 0.0, cap, Step(3.0, 0.0, 0.2))
    now = Pose(2.0, 1.0, 10.0, 0.25)                 # turned a little since the capture
    assert tr.accept(d, 0.6, pose_now=now)
    tk = tr.tick(0.6, now)
    assert tk.author == "vla_step" and tk.reason == "rate", tk
    assert math.isclose(tk.action.vx, 3.0 * math.cos(0.25)) and \
        math.isclose(tk.action.vy, 3.0 * math.sin(0.25)), tk.action
    assert math.isclose(tk.action.yaw_rate, 0.2), "dpsi / H"
    assert math.isclose(tk.action.vz_up, 0.0, abs_tol=1e-12), "the altitude held as before"
    assert tr.last_rate == {"horizon_s": H, "speed_mps": 3.0, "yaw_rate_rps": 0.2,
                            "heading_reached": False}, tr.last_rate
    tgt = d.record["target"]
    assert tgt["rate"] == {"horizon_s": H, "speed_mps": 3.0, "yaw_rate_rps": 0.2}, tgt
    assert math.isclose(tgt["psi_rad"], 0.5), "the heading target is capture yaw + dpsi"
    assert math.isclose(tgt["time_cap_s"], H * 1.2)
    # H = 0.5 doubles the rates.
    t2 = StepTracker(TrackerConfig(rate_horizon_s=0.5, speed_cap_mps=8.0, cruise_mps=1.0))
    t2.accept(Decision.of(1, 0.0, cap, Step(3.0, 0.0, 0.2)), 0.0, pose_now=cap)
    k2 = t2.tick(0.0, cap)
    assert math.isclose(math.hypot(k2.action.vx, k2.action.vy), 6.0)
    assert math.isclose(k2.action.yaw_rate, 0.4)
    # turn_while_moving keeps its cos(heading error) scaling.
    t3 = StepTracker(TrackerConfig(**FOLLOW, rate_horizon_s=H))
    t3.accept(Decision.of(1, 0.0, cap, Step(3.0, 0.0, 0.6)), 0.0, pose_now=cap)
    k3 = t3.tick(0.0, cap)
    assert math.isclose(math.hypot(k3.action.vx, k3.action.vy), 3.0 * math.cos(0.6))
    assert math.isclose(k3.action.yaw_rate, 0.5), "0.6 rad/s capped at the yaw cap"
    # Without it a rotate-only step only turns.
    t4 = StepTracker(TrackerConfig(rate_horizon_s=H, speed_cap_mps=5.0, cruise_mps=1.0))
    t4.accept(Decision.of(1, 0.0, cap, Step(3.0, 0.0, 0.4)), 0.0, pose_now=cap)
    k4 = t4.tick(0.0, cap)
    assert (k4.action.vx, k4.action.vy) == (0.0, 0.0) and math.isclose(k4.action.yaw_rate, 0.4)


def test_the_rates_are_capped_and_the_turn_stops_at_the_target_heading():
    cap = Pose(0.0, 0.0, 10.0, 0.0)
    t = StepTracker(TrackerConfig(**FOLLOW, rate_horizon_s=0.5))
    t.accept(Decision.of(1, 0.0, cap, Step(5.0, 0.0, 0.0)), 0.0, pose_now=cap)
    k = t.tick(0.0, cap)
    assert math.isclose(k.action.vx, 5.0), "10 m/s capped at speed_cap_mps 5"
    assert t.last_rate["speed_mps"] == 5.0
    # A turn: at the rate until the heading reaches capture yaw + dpsi.
    t = StepTracker(TrackerConfig(**FOLLOW, rate_horizon_s=1.0, yaw_tol_rad=0.01))
    t.accept(Decision.of(1, 0.0, cap, Step(2.0, 0.0, 0.3)), 0.0, pose_now=cap)
    assert math.isclose(t.tick(0.1, Pose(0.1, 0.0, 10.0, 0.1)).action.yaw_rate, 0.3)
    near = t.tick(0.2, Pose(0.2, 0.0, 10.0, 0.285))           # 0.015 rad to go
    assert math.isclose(near.action.yaw_rate, 0.15), "never past the target in one tick"
    at = t.tick(0.3, Pose(0.3, 0.0, 10.0, 0.295))
    assert at.action.yaw_rate == 0.0 and t.last_rate["heading_reached"], at
    past = t.tick(0.4, Pose(0.4, 0.0, 10.0, 0.36))
    assert past.action.yaw_rate == 0.0 and past.reason == "rate", "no turning back"
    assert math.hypot(past.action.vx, past.action.vy) > 1.9, "the speed flies on"
    # Pipelined: turned past the target since the capture -> no turn at all.
    t = StepTracker(TrackerConfig(**FOLLOW, rate_horizon_s=1.0))
    t.accept(Decision.of(1, 0.0, cap, Step(2.0, 0.0, 0.2)), 0.8,
             pose_now=Pose(3.0, 0.0, 10.0, 0.4))
    assert t.tick(0.8, Pose(3.0, 0.0, 10.0, 0.4)).action.yaw_rate == 0.0
    # The yaw slew's stopping profile still applies with smoothing.
    t = StepTracker(TrackerConfig(**FOLLOW, rate_horizon_s=1.0, smooth_yaw_accel_rps2=0.6))
    t.accept(Decision.of(1, 0.0, cap, Step(2.0, 0.0, 0.5)), 0.0, pose_now=cap)
    k = t.tick(0.0, Pose(0.0, 0.0, 10.0, 0.4))                # 0.1 rad to go
    assert 0.0 < t.last_rate["yaw_rate_rps"] <= math.sqrt(2 * 0.6 * 0.1) + 1e-9, t.last_rate
    assert k.action.yaw_rate <= 0.6 * 0.1 + 1e-9, "the slew limit on the output too"


def test_the_next_answer_replaces_the_rate_and_past_h_the_carry_or_hold_ends_it():
    cfg = TrackerConfig(**FOLLOW, rate_horizon_s=1.0, carry_s=1.0)
    p = Pose(0.0, 0.0, 10.0, 0.0)
    tr = StepTracker(cfg)
    d1 = Decision.of(1, 0.0, p, Step(2.0, 0.0, 0.0))
    tr.accept(d1, 0.0, pose_now=p)
    ticks = {}
    for k in range(0, 12):
        now = round(k * 0.1, 10)
        ticks[k] = tr.tick(now, p, pending=True)
    assert all(t.reason == "rate" and math.isclose(t.action.vx, 2.0) for t in ticks.values())
    # Replaced at once by the next answer (no ramp without an accel limit).
    d2 = Decision.of(2, 0.9, p, Step(4.0, 0.0, 0.0))
    tr.accept(d2, 1.15, pose_now=p)
    k = tr.tick(1.15, p, pending=True)
    assert k.seq == 2 and math.isclose(k.action.vx, 4.0) and k.reason == "rate"
    assert d1.record["outcome"]["end"] == "superseded"
    # 1.2 x H after ITS answer it is complete: the carry while one is pending...
    for now in (1.5, 2.0, 2.34):
        assert tr.tick(now, p, pending=True).reason == "rate", now
    c = tr.tick(2.35, p, pending=True)
    assert c.author == "vla_step" and c.reason == "carry" and math.isclose(c.action.vx, 4.0), c
    assert tr.last_rate is None, "a carry is no rate tick"
    c2 = tr.tick(2.85, p, pending=True)
    assert c2.reason == "carry" and math.isclose(c2.action.vx, 2.0, abs_tol=1e-9), c2
    # ...and the hold with nothing pending: no new stop rule, no silent hover.
    h = tr.tick(2.9, p, pending=False)
    assert h.author == "hold" and h.reason == "step_complete" and h.action.vx == 0.0, h
    assert tr.counts["completed_rate_horizon"] == 1 and tr.counts["rate_ticks"] >= 15
    tr.close(3.0)
    assert d2.record["outcome"]["completion"] == "rate_horizon"
    # Without a carry the step ends in the hold at once.
    t2 = StepTracker(TrackerConfig(**FOLLOW, rate_horizon_s=1.0))
    t2.accept(Decision.of(1, 0.0, p, Step(2.0, 0.0, 0.0)), 0.0, pose_now=p)
    assert t2.tick(1.19, p, pending=True).reason == "rate"
    assert t2.tick(1.2, p, pending=True).reason == "step_complete"
    # The overrun is configurable.
    t3 = StepTracker(TrackerConfig(**FOLLOW, rate_horizon_s=1.0, rate_overrun_frac=0.5))
    t3.accept(Decision.of(1, 0.0, p, Step(2.0, 0.0, 0.0)), 0.0, pose_now=p)
    assert t3.tick(1.45, p).reason == "rate" and t3.tick(1.5, p).reason == "step_complete"


def test_the_smoothing_limits_a_rate_step_as_any_other_output():
    cfg = TrackerConfig(**FOLLOW, rate_horizon_s=1.0, smooth_accel_mps2=2.5,
                        smooth_yaw_accel_rps2=0.6, carry_s=1.0)
    p = Pose(0.0, 0.0, 10.0, 0.0)
    tr = StepTracker(cfg)
    tr.accept(Decision.of(1, 0.0, p, Step(4.0, 0.0, 0.0)), 0.0, pose_now=p)
    sp = []
    for k in range(10):
        tk = tr.tick(round(k * 0.1, 10), p, pending=True)
        assert tk.reason == "rate" and tr.last_rate["speed_mps"] == 4.0
        sp.append(math.hypot(tk.action.vx, tk.action.vy))
    assert sp[0] <= 0.25 + 1e-9 and all(b - a <= 0.25 + 1e-9 for a, b in zip(sp, sp[1:])), sp
    assert math.isclose(sp[-1], 2.5, abs_tol=1e-9)


def test_the_rail_records_the_rate_on_every_tick_of_a_rate_tracker():
    import vla_mission as VM
    from guardrail.replay import rail_shield
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    shield = rail_shield(load_policy(POLICY), lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)

    class Backend:
        def __init__(self, tracker, clock):
            self.tracker, self.clock = tracker, clock
            self.last_link_failure = self.last_tick = None

        def act(self, state, hold_horizontal=False):
            self.last_tick = self.tracker.tick(self.clock(), Pose.from_state(state),
                                               hold_horizontal=hold_horizontal)
            return self.last_tick.action

    for rate in (1.0, None):
        clk = {"t": 0.0}
        tr = StepTracker(TrackerConfig(**FOLLOW, rate_horizon_s=rate))
        st = State(x=42.0, y=-18.0, up=10.0, yaw_deg=90.0)
        p = Pose.from_state(st)
        tr.accept(Decision.of(1, 0.0, p, Step(2.0, 0.0, 0.0)), 0.0, pose_now=p)
        mon = VM.MissionMonitor(VM.MonitorConfig(mission="goto", max_s=60.0), 0.0)
        loop = VM.MissionLoop(Backend(tr, lambda: clk["t"]), shield, mon, mission="goto",
                              goal_xy=(43.0, 40.0), clock=lambda: clk["t"])
        rows = []
        for _ in range(16):
            rows.append(loop.tick(st).row)
            clk["t"] = round(clk["t"] + 0.1, 10)
        if rate is None:
            assert all("vla_rate" not in r for r in rows), "a default run's rows are unchanged"
            continue
        assert all("vla_rate" in r for r in rows)
        assert rows[0]["vla_rate"]["speed_mps"] == 2.0 and rows[0]["author_reason"] == "rate"
        assert rows[-1]["vla_rate"] is None and rows[-1]["author_reason"] == "step_complete"


# =========================================================================== #
# D. vla_fly's flags and the DAgger recorder
# =========================================================================== #
def test_vla_fly_takes_the_tile_and_the_rate_and_records_both():
    import contextlib
    import vla_fly as VF
    ap = VF.build_arg_parser()
    d = VF.parse_cli(ap, ["--mission", "follow"])
    assert d.bottom_tile == C.BottomTile() and d.step_as_rate is None
    assert VF.tracker_config(d, None, TrackerConfig).rate_horizon_s is None
    a = VF.parse_cli(ap, ["--mission", "follow", "--cadence", "pipelined", "--vla-bottom-tile",
                          "prev_front", "--step-as-rate", "1.0"])
    assert a.bottom_tile == PREV and a.step_as_rate == 1.0
    cfg = VF.tracker_config(a, None, TrackerConfig)
    assert cfg.rate_horizon_s == 1.0 and cfg.turn_while_moving and cfg.smoothing
    rep = VF.check_args_report(a)
    assert rep["bottom_tile"] == PREV.as_dict() and rep["step_as_rate_s"] == 1.0
    b = VF.parse_cli(ap, ["--mission", "follow", "--vla-bottom-tile", "prev_front",
                          "--vla-prev-dt-s", "0.5"])
    assert b.bottom_tile == C.bottom_tile("prev_front", 0.5)
    # A goto with the prev_front tile is the refusal's business (the original
    # adapter is "down"); --step-as-rate is the follow fine-tune's only.
    VF.parse_cli(ap, ["--mission", "goto", "--vla-bottom-tile", "prev_front"])
    for bad in (["--mission", "goto", "--step-as-rate", "1.0"],
                ["--mission", "approach", "--step-as-rate", "1.0"],
                ["--mission", "follow", "--step-as-rate", "0"],
                ["--mission", "follow", "--step-as-rate", "-1"],
                ["--mission", "follow", "--step-as-rate", "nan"],
                ["--mission", "follow", "--vla-prev-dt-s", "1.0"],          # with down
                ["--mission", "follow", "--vla-bottom-tile", "prev_front", "--vla-prev-dt-s",
                 "0.2"],                                                       # tol >= dt
                ["--mission", "follow", "--vla-bottom-tile", "rear"]):
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                VF.parse_cli(ap, bad)
        except SystemExit as exc:
            assert exc.code == 2, bad
            continue
        raise AssertionError(f"accepted {bad}")
    src = inspect.getsource(VF._fly)
    assert "VM.VLAFrames(obs, saver=saver, bottom_tile=args.bottom_tile)" in src
    assert '"step_as_rate_s": args.step_as_rate' in src


def test_the_dagger_recorder_follows_vla_flys_tile_and_rate_tick_by_tick():
    D, TD = _D(), _TD()
    import vla_fly as VF
    vargs, argv = D.follow_args("a red car", cruise_alt=8.0, bottom_tile="prev_front",
                                prev_dt_s=1.0, step_as_rate=1.0)
    ref = VF.parse_cli(VF.build_arg_parser(), [
        "--mission", "follow", "--cadence", "pipelined", "--cruise-mps", "4.5",
        "--speed-cap-mps", "5", "--cruise-alt", "8", "--object", "a red car",
        "--vla-bottom-tile", "prev_front", "--vla-prev-dt-s", "1", "--step-as-rate", "1"])
    cfg = D.follow_tracker_config(vargs)
    assert cfg == VF.tracker_config(ref, None, TrackerConfig) and cfg.rate_horizon_s == 1.0
    assert vargs.bottom_tile == ref.bottom_tile == PREV and vargs.step_as_rate == 1.0
    plain, pargv = D.follow_args("a red car", cruise_alt=8.0)
    assert plain.bottom_tile == C.BottomTile() and plain.step_as_rate is None
    assert "--vla-bottom-tile" not in pargv and "--step-as-rate" not in pargv
    # A window flown by the rate tracker: every window tick records the rates.
    t = {"t": TD.T0}
    client = TD._Stub(lambda: t["t"])
    veh = TD._Vehicle()

    def grounder():
        return {"det": TD._det_at_bearing(0.0), "t_det": t["t"] - 0.02, "seq": 1, "tier": "ok",
                "cap": {"t_capture": t["t"] - 0.2, "pose": {"yaw": veh.yaw}}}
    p = D.VLAPerturber(7, client=client,
                       get_capture=lambda: Capture(t["t"] - 0.05, Pose(*veh.pose()), TD.JPG,
                                                   TD.JPG),
                       latest_box=D.latest_box_from(grounder), object_text="a red car",
                       tracker_config=cfg, clock=lambda: t["t"], **D.follow_hint_config(vargs))
    p.start(TD.T0, thread=False)
    e0 = p.schedule.episode(0)
    answer = TD._step(3.0, 0.0, 0.2, after=0.5)
    rows, s = [], 0.0
    while s < e0["off_s"] + 0.5:
        t["t"] = TD.T0 + s
        if client._pending is not None:
            got = answer(client._pending, t["t"] - client.t_sent)
            if got is not None:
                client.answer(t["t"], got)
        p._service_once()
        out, rec = p.apply(TD.EXPERT, s, pose=veh.pose())
        p._service_once()
        json.dumps(rec)
        rows.append((s, out, rec))
        veh.step(out)
        s = round(s + 0.1, 10)
    win = [r for r in rows if r[2]["window"]]
    flown = [r for r in win if r[2]["active"]]
    assert len(flown) >= 20, len(flown)
    assert all("vla_rate" in r[2] for r in win)
    for _s, out, rec in flown:
        assert rec["reason"] == "rate" and rec["vla_rate"]["horizon_s"] == 1.0, rec
        assert rec["vla_cmd"] == {k: getattr(out, k) for k in ("vx", "vy", "vz_up", "yaw_rate")}
    assert all("vla_rate" not in r[2] for r in rows if not r[2]["window"])
    p.stop()
    # The flags' refusals, before any server is asked.
    assert D.check_args(NO_PORT, (6.0, 10.0), (2.5, 4.0)) is None
    assert D.check_args(NO_PORT, (6.0, 10.0), (2.5, 4.0), bottom_tile="prev_front",
                        prev_dt_s=1.0, step_as_rate=1.0) is None
    for kw in ({"bottom_tile": "prev_front", "prev_dt_s": 0.1}, {"step_as_rate": 0.0},
               {"prev_dt_s": 1.0}, {"bottom_tile": "rear"}):
        assert D.check_args(NO_PORT, (6.0, 10.0), (2.5, 4.0), **kw), kw


def test_follow_vlm_and_the_launcher_pass_the_dagger_flags_only_with_a_dagger_recording():
    import follow_vlm as F_
    a = F_.build_arg_parser().parse_args([])
    assert a.perturb_vla_bottom_tile == "down" and a.perturb_vla_prev_dt_s is None
    assert a.perturb_vla_step_as_rate is None
    b = F_.build_arg_parser().parse_args(
        ["--record-vla", "d", "--record-perturb-vla", NO_PORT, "--perturb-vla-bottom-tile",
         "prev_front", "--perturb-vla-prev-dt-s", "1.0", "--perturb-vla-step-as-rate", "1.0"])
    assert (b.perturb_vla_bottom_tile, b.perturb_vla_prev_dt_s, b.perturb_vla_step_as_rate) == \
        ("prev_front", 1.0, 1.0)
    for argv in (["--record-vla", "d", "--perturb-vla-bottom-tile", "prev_front"],
                 ["--record-vla", "d", "--perturb-vla-step-as-rate", "1.0"],
                 ["--record-vla", "d", "--perturb-vla-prev-dt-s", "1.0"],
                 ["--record-vla", "d", "--record-perturb-vla", NO_PORT,
                  "--perturb-vla-step-as-rate", "0"],
                 ["--record-vla", "d", "--record-perturb-vla", NO_PORT,
                  "--perturb-vla-bottom-tile", "prev_front", "--perturb-vla-prev-dt-s", "0.1"]):
        try:
            with __import__("contextlib").redirect_stderr(io.StringIO()):
                F_.main(argv)
        except SystemExit as exc:
            assert exc.code == 2, (argv, exc.code)
        else:
            raise AssertionError(f"{argv} was accepted")
    src = (ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8")
    call = src[src.index("vla_dagger.make_vla_perturber("):]
    call = call[:call.index(")\n")]
    for kw in ("bottom_tile=args.perturb_vla_bottom_tile",
               "prev_dt_s=args.perturb_vla_prev_dt_s",
               "step_as_rate=args.perturb_vla_step_as_rate"):
        assert kw in call, kw
    # The launcher: inside the -RecordPerturbVla block, refused without it
    # before a simulator is started.
    ps = PS1.read_text(encoding="utf-8")
    for p in ("[string]$PerturbVlaBottomTile", "[double]$PerturbVlaPrevDtS",
              "[double]$PerturbVlaStepAsRate"):
        assert p in ps, p
    lines = ps.splitlines()
    for flag in ('"--perturb-vla-bottom-tile"', '"--perturb-vla-prev-dt-s"',
                 '"--perturb-vla-step-as-rate"'):
        assert ps.count(flag) == 1, flag
        fi = next(n for n, ln in enumerate(lines) if flag in ln)
        gi = max(n for n, ln in enumerate(lines[:fi])
                 if ln.lstrip().startswith("if ($RecordPerturbVla)"))
        ind = len(lines[gi]) - len(lines[gi].lstrip())
        for ln in lines[gi + 1:fi + 1]:
            assert not ln.strip() or len(ln) - len(ln.lstrip()) > ind, (flag, ln)
    sim = ps.index("if (-not $SkipSim) { Start-Sim }")
    assert ps.index("($PerturbVlaBottomTile -or $PerturbVlaPrevDtS -or $PerturbVlaStepAsRate) "
                    "-and -not $RecordPerturbVla") < sim


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
