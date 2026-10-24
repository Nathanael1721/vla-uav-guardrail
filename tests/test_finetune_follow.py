"""training/finetune_aerialvla_follow.py and tools/eval_vla_adapter.py: the
follow fine-tune trains on exactly what the VLA server feeds and reads back,
scores held-out answers the way a flight reads them, and its checkpoints are
announced and recorded under their own id.

Run either way:
    pytest tests/test_finetune_follow.py -v
    python tests/test_finetune_follow.py

WHAT IS BEING CLAIMED

1. A sample is kept only when its prompt re-builds byte for byte from its
   phrase and object with aerialvla_codec.build_prompt, its target is a
   two-digit, truncated "FF DD YY" the strict reader accepts (no EOS: the
   trainer appends the tokenizer's), its optional phrase / object / bins
   agree, and it names a run and a split; a run in both train and a held-out
   split refuses the dataset.
2. The encoding: input_ids = tok(prompt + target) + [EOS] with the prompt's
   own tokens (tok(prompt), as the server makes them) as a prefix, labels
   -100 over the prompt, the answer decoding to exactly target + "</s>". A
   tokenizer that merges the prompt's trailing space into the answer, or
   decodes with a leading space, is refused - the old trainer's defect.
3. The held-out metrics read answers as the server does (require_eos; a
   parse failure is the zero step the client holds), per axis.
4. A checkpoint's provenance file makes the server announce it as
   aerialvla-ft-follow/<run>/<epoch>@<adapter sha8> with an `adapter` block;
   the pilot side records it under its own model id, and the dashboard rule
   (tools/todo.py) counts it as a VLA, recorded as the fine-tune - never as
   upstream's adapter.
5. The evaluation tool sends what a flight sends (the mosaic's tiles as PNG:
   the server rebuilds the mosaic pixel for pixel), refuses a server whose
   adapter digest is not --adapter's, and the training loop runs end to end
   on a CPU stand-in model (checkpoints, provenance, logs, early stopping).
6. With the real tokenizer and image processor on disk (CPU; never the 7B
   model): the encoding holds on all 8 phrases, and one encoded sample
   equals the server's own preprocessing (prompt bytes, image sha256, prompt
   token ids, pixel_values).

SKIPs: the image tests need Pillow, the training-loop and collate tests
torch, the real-tokenizer test transformers and D:/models/openvla-7b (none
in CI; all present in the vla-real env).

Shown failing first: at 47fccc0 neither file existed (every test fails at
import), aerialvla_codec had no adapter_variant, vla_server_v2 had no
adapter_identity / read_adapter_provenance (a checkpoint was announced as
"<run>/epoch<N>@unknown", which tools/todo.py classed "unknown"), and
vla_mission recorded every real identity under upstream's adapter id.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools", ROOT / "training"):
    sys.path.insert(0, str(_p))

from guardrail import aerialvla_codec as C                           # noqa: E402
import finetune_aerialvla_follow as FT                               # noqa: E402

SKIP = "SKIP"
OBJ = "a red car"


def _tmp() -> Path:
    return Path(tempfile.mkdtemp(prefix="ftfollow_"))


def _rec(phrase="straight ahead ", target="96 49 49", run="r1", split="train", **kw) -> dict:
    r = {"image": "images/a.png", "prompt": C.build_prompt(phrase, OBJ), "target": target,
         "run": run, "split": split}
    r.update(kw)
    return r


def _pil():
    try:
        from PIL import Image
        return Image
    except ImportError:
        if os.environ.get("GUARDRAIL_REQUIRE_PILLOW") == "1":
            raise AssertionError("GUARDRAIL_REQUIRE_PILLOW=1 and Pillow does not import")
        return None


def _torch():
    try:
        import torch
        return torch
    except ImportError:
        return None


# ------------------------------------------------------------------ 1. the data

def test_every_server_prompt_round_trips_and_nothing_else_does():
    for ph in C.PHRASES:
        p = C.build_prompt(ph, OBJ)
        assert FT.split_prompt(p) == (ph, OBJ), ph
    bad = [C.build_prompt("straight ahead ", OBJ, "Stay above 6 m.", "csp"),   # a CSP arm
           C.build_prompt("straight ahead ", OBJ).replace("Action: ", "Action:"),
           C.build_prompt("straight ahead ", OBJ).replace("ahead ", "ahead  "),  # not a phrase
           "<image>\nFly straight ahead and find the target. a red car\nAction: 96",
           "Fly straight ahead and find the target. a red car\nAction: ",
           C.PROMPT_TEMPLATE.format(direction="straight ahead ", object="target at 38.0, 26.0"),
           None, 7]
    for p in bad:
        try:
            FT.split_prompt(p)
        except FT.SampleError as exc:
            assert exc.code == "bad_prompt", (p, exc.code)
            continue
        raise AssertionError(f"accepted {p!r}")


def test_a_sample_is_kept_only_when_every_field_agrees():
    root = _tmp()
    s = FT.check_sample(_rec(phrase="forward-left ", target="96 48 47 LAND", bins=[96, 48, 47],
                             phrase_field=None, pose={"x": 1}, t_capture=3.0), root, 4)
    assert (s.phrase, s.object, s.bins, s.land, s.run, s.split) == \
        ("forward-left ", OBJ, (96, 48, 47), True, "r1", "train")
    assert s.image == root / "images" / "a.png" and s.extra == {"pose": {"x": 1}, "t_capture": 3.0}
    # the aliases FT-D may write
    alias = {"image_path": "x.png", "prompt": C.build_prompt("", OBJ), "target_text": "00 49 49",
             "run_tag": "r9", "split": "holdout"}
    t = FT.check_sample(alias, root, 0)
    assert t.phrase == "" and t.run == "r9" and t.image == root / "x.png"
    mism = _rec(phrase="to your left ")
    mism["phrase"] = "to your right "              # the field disagrees with the prompt
    cases = {
        "bad_prompt": _rec(prompt="Fly"),
        "phrase_mismatch": mism,
        "object_mismatch": _rec(object="a blue car"),
        "target_has_eos": _rec(target="96 49 49</s>"),
        "bad_target": _rec(target="5 49 49"),
        "bins_mismatch": _rec(bins=[96, 49, 48]),
        "no_run": _rec(run=""),
        "no_split": _rec(split=None),
        "no_image": {k: v for k, v in _rec().items() if k != "image"},
        "bad_image_sha256": _rec(image_sha256="XYZ"),
        "no_target": {k: v for k, v in _rec().items() if k != "target"},
        "not_an_object": [1, 2],
    }
    for code, rec in cases.items():
        try:
            FT.check_sample(rec, root, 0)
        except FT.SampleError as exc:
            assert exc.code == code, (code, exc)
            continue
        raise AssertionError(f"{code}: accepted {rec!r}")
    # the strict reader's refusals are the trainer's: '00 49 494 49' (the dive),
    # bin 99, a fourth group, a lower-case land
    for t in ("00 49 494 49", "99 49 49", "00 49 49 49", "00 49 49 land", " 00 49 49"):
        try:
            FT.check_sample(_rec(target=t), root, 0)
        except FT.SampleError as exc:
            assert exc.code == "bad_target", (t, exc)
            continue
        raise AssertionError(f"accepted target {t!r}")


def test_targets_are_upstreams_truncated_zero_padded_bins():
    """The labels the old trainer made were rounded and unpadded; a target
    made with the codec is the truncated, two-digit text, and the trainer
    takes nothing else."""
    fwd = 2.49                                   # norm 0.498 x 98 = 48.8
    assert C.quantize(fwd, "fwd_m") == 48 and round(fwd / 5.0 * 98) == 49
    b = (C.quantize(0.1, "fwd_m"), C.quantize(-0.003, "down_m"), C.quantize(0.0, "dpsi_rad"))
    assert b == (1, 48, 49)
    text = f"{b[0]:02d} {b[1]:02d} {b[2]:02d}"
    assert text == "01 48 49" and FT.check_sample(_rec(target=text), _tmp(), 0).bins == b
    try:
        FT.check_sample(_rec(target="1 48 49"), _tmp(), 0)
    except FT.SampleError:
        pass
    else:
        raise AssertionError("an unpadded bin was accepted")


def test_the_dataset_loads_with_counted_drops_and_never_trains_a_held_out_run():
    d = _tmp()
    (d / "images").mkdir()
    (d / "images" / "a.png").write_bytes(b"x")
    lines = [json.dumps(_rec()), json.dumps(_rec(run="r2")), json.dumps(_rec(run="r3", split="holdout")),
             json.dumps(_rec(run="r4", split="other")), "{not json", json.dumps(_rec(target="5 49 49")),
             json.dumps(_rec(image="images/missing.png")), ""]
    (d / "samples.jsonl").write_text("\n".join(lines), encoding="utf-8")
    samples, rep = FT.load_samples(d)
    assert rep["lines"] == 7 and rep["kept"] == 4 and len(samples) == 4, rep
    assert rep["dropped"] == {"bad_json": 1, "bad_target": 1, "image_missing": 1}, rep
    assert rep["sha256"] == hashlib.sha256((d / "samples.jsonl").read_bytes()).hexdigest()
    tr, held, info = FT.split_samples(samples)
    assert len(tr) == 2 and len(held) == 1 and info["other_splits"] == {"other": 1}
    assert info["train_runs"] == ["r1", "r2"] and info["held_out_runs"] == ["r3"]
    leak = samples + [FT.check_sample(_rec(run="r1", split="holdout"), d, 99)]
    try:
        FT.split_samples(leak)
    except FT.DataError as exc:
        assert "r1" in str(exc)
    else:
        raise AssertionError("a run in train and held-out was accepted")
    st = FT.data_stats(samples)
    assert st["bins"]["fwd_m"] == {"96": 4} and st["phrases"] == {"straight ahead ": 4}
    assert st["rotate_only_band_with_forward"] == 0
    rot = FT.data_stats([FT.check_sample(_rec(target=f"40 49 {C.quantize(0.6, 'dpsi_rad'):02d}"),
                                         d, 0)])
    assert rot["rotate_only_band_with_forward"] == 1 and rot["turning"] == 1


# ------------------------------------------------------------------ 2. the encoding

class _CharTok:
    """A character tokenizer with Llama's special ids (BOS 1, EOS 2, pad 0):
    every character is its own token, so a prompt's tokens are always a
    prefix of prompt + target's - the property the real tokenizer has on
    every phrase (test_the_real_tokenizer_... below)."""
    eos_token_id, pad_token_id, bos_token_id = 2, 0, 1

    def _enc(self, text):
        return [1] + [3 + ord(c) for c in text]

    def __call__(self, text, return_tensors=None):
        ids = self._enc(text)
        if return_tensors == "pt":
            import torch
            t = torch.tensor([ids], dtype=torch.long)
            return {"input_ids": t, "attention_mask": torch.ones_like(t)}
        return {"input_ids": ids}

    def decode(self, ids, skip_special_tokens=False):
        ids = ids.tolist() if hasattr(ids, "tolist") else list(ids)
        out = []
        for i in ids:
            if i == 2:
                out.append("" if skip_special_tokens else "</s>")
            elif i == 1:
                out.append("" if skip_special_tokens else "<s>")
            elif i == 0:
                out.append("" if skip_special_tokens else "<PAD>")
            else:
                out.append(chr(i - 3))
        return "".join(out)


class _MergingTok(_CharTok):
    """Merges a space with the digit after it into one token (what the July
    trainer's separate tokenisation amounted to): the prompt's trailing
    space is then not a token of prompt + target."""

    def _enc(self, text):
        out, i = [1], 0
        while i < len(text):
            if text[i] == " " and i + 1 < len(text) and text[i + 1].isdigit():
                out.append(900 + int(text[i + 1]))
                i += 2
                continue
            out.append(3 + ord(text[i]))
            i += 1
        return out


class _SpaceDecodeTok(_CharTok):
    def decode(self, ids, skip_special_tokens=False):
        return " " + super().decode(ids, skip_special_tokens)


def test_the_encoding_is_the_servers_prompt_then_the_answer_and_eos():
    tok = _CharTok()
    for ph in C.PHRASES:
        p = C.build_prompt(ph, OBJ)
        for tgt in ("96 48 49", "00 49 49 LAND", "05 50 30"):
            e = FT.encode_text(tok, p, tgt)
            n = len(tok(p)["input_ids"])
            assert e["n_prompt"] == n and e["input_ids"][:n] == tok(p)["input_ids"]
            assert e["input_ids"][-1] == tok.eos_token_id
            assert e["labels"][:n] == [FT.IGNORE_INDEX] * n
            assert e["labels"][n:] == e["input_ids"][n:] and len(e["labels"]) == len(e["input_ids"])
            assert e["answer_text"] == tgt + "</s>"
            assert C.parse_action(e["answer_text"], require_eos=True).parse_ok
            assert e["input_ids"] == tok(p + tgt)["input_ids"] + [2]
    for bad in (_MergingTok(), _SpaceDecodeTok()):
        try:
            FT.encode_text(bad, C.build_prompt("straight ahead ", OBJ), "96 48 49")
        except FT.EncodeError:
            continue
        raise AssertionError(f"{type(bad).__name__} was accepted")


def test_collate_pads_on_the_right_and_masks_the_padding():
    torch = _torch()
    if torch is None:
        print("      SKIP: torch is not installed (the vla-real env has it)")
        return SKIP
    tok = _CharTok()
    a = FT.encode_text(tok, C.build_prompt("straight ahead ", OBJ), "96 48 49")
    b = FT.encode_text(tok, C.build_prompt("to your right rear ", OBJ), "00 49 49 LAND")
    a["pixel_values"] = torch.zeros(1, 6, 4, 4)
    b["pixel_values"] = torch.ones(1, 6, 4, 4)
    out = FT.collate([a, b], pad_id=0)
    w = max(len(a["input_ids"]), len(b["input_ids"]))
    assert out["input_ids"].shape == (2, w) and out["pixel_values"].shape == (2, 6, 4, 4)
    assert out["input_ids"][0, 0] == 1 and out["input_ids"][1, 0] == 1, "BOS must stay first"
    na = len(a["input_ids"])
    assert out["attention_mask"][0, :na].all() and not out["attention_mask"][0, na:].any()
    assert (out["labels"][0, na:] == FT.IGNORE_INDEX).all()
    assert out["input_ids"][0, na:].eq(0).all()


# ------------------------------------------------------------------ 3. the metrics

def test_the_held_out_metrics_read_answers_as_a_flight_does():
    pairs = [("96 49 49", "96 49 49</s>"),          # exact
             ("96 49 60", "90 49 40</s>"),          # wrong fwd and the opposite turn
             ("50 49 30", "00 49 494 49</s>"),      # the dive text: a parse failure, flown as zero
             ("50 49 49", "50 49 49")]              # no EOS: ran out of tokens, a parse failure
    m = FT.score_outputs(pairs)
    assert m["n"] == 4 and m["parse_ok"] == 2 and m["parse_ok_rate"] == 0.5
    assert m["exact_bin_rate"]["fwd_m"] == 0.25 and m["exact_bin_rate"]["all"] == 0.25
    assert m["exact_bin_rate_parsed"]["down_m"] == 1.0 and m["exact_bin_rate_parsed"]["fwd_m"] == 0.5
    f = lambda b: C.dequantize(b, "fwd_m")                                   # noqa: E731
    y = lambda b: C.dequantize(b, "dpsi_rad")                                # noqa: E731
    want_fwd = (0 + abs(f(90) - f(96)) + f(50) + f(50)) / 4
    assert math.isclose(m["step_mae"]["fwd_m"], round(want_fwd, 6), abs_tol=1e-6), m["step_mae"]
    want_y = (0 + abs(y(40) - y(60)) + abs(y(30)) + 0) / 4
    assert math.isclose(m["step_mae"]["dpsi_rad"], round(want_y, 6), abs_tol=1e-6)
    assert m["turn_sign_agree"] == 0.5 and m["turn_sign_agree_on_turns"] == 0.0
    norm = (m["step_mae"]["fwd_m"] / 5 + m["step_mae"]["down_m"] / 10 + m["step_mae"]["dpsi_rad"] / 2.2) / 3
    assert math.isclose(m["step_mae_norm"], norm, abs_tol=1e-5)
    assert sum(m["parse_fail_reasons"].values()) == 2
    land = FT.score_outputs([("00 49 49 LAND", "00 49 49 LAND</s>"), ("00 49 49", "00 49 49 LAND</s>")])
    assert land["land_target_rate"] == 0.5 and land["land_pred_rate"] == 1.0
    assert land["land_agree_rate"] == 0.5
    try:
        FT.score_outputs([("not a target", "96 49 49</s>")])
    except ValueError:
        pass
    else:
        raise AssertionError("a malformed target was scored")


def test_schedule_oversampling_subsets_and_run_names():
    assert FT.lr_at(0, 100, 10, 0.1) == 0.1 and FT.lr_at(9, 100, 10, 0.1) == 1.0
    assert math.isclose(FT.lr_at(100, 100, 10, 0.1), 0.1)
    assert math.isclose(FT.lr_at(55, 100, 10, 0.1), 0.1 + 0.9 * 0.5, rel_tol=1e-9)
    root = _tmp()
    straight = FT.check_sample(_rec(target="96 49 49"), root, 0)
    turn = FT.check_sample(_rec(target="96 49 53"), root, 1)
    assert FT.oversample([straight, turn], 1, 3) == [straight, turn]
    assert FT.oversample([straight, turn], 3, 3) == [straight, turn, turn, turn]
    many = [FT.check_sample(_rec(), root, i) for i in range(20)]
    a, b = FT.heldout_subset(many, 5, 7), FT.heldout_subset(many, 5, 7)
    assert a == b and len(a) == 5 and [s.idx for s in a] == sorted(s.idx for s in a)
    assert FT.heldout_subset(many, 0, 7) == many
    ap = FT.build_arg_parser()
    import contextlib
    import io
    for bad in ("a/b", "a@b", "a:b", "", ".x"):
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                FT.check_args(ap, ap.parse_args(["--data", "d", "--run", bad]))
        except SystemExit:
            continue
        raise AssertionError(f"run name {bad!r} accepted")


def test_speed_changes_are_oversampled_against_the_same_runs_sample_a_second_earlier():
    """Flight vla_follow_ft_05_pf_off_20261024 (24 Oct): the car pulled away
    from 8 m and the model answered forward 0.0 at 2 Hz for about 7 s. The
    samples where the expert's forward step changes by >= 1 m within 1 s (a
    car stopping or pulling away) are a few per flight; --change-oversample
    repeats them. The earlier sample is the same run's and the same mirror
    side's nearest capture to t - 1 s (within 0.2 s); none -> not a change."""
    root = _tmp()

    def smp(i, t, fwd_bin, run="r1", mirror=False):
        sid = f"{run}/{i}" + ("/mirror" if mirror else "")
        return FT.check_sample(_rec(target=f"{fwd_bin:02d} 49 49", run=run, id=sid,
                                    t_capture=t), root, i)

    a = smp(0, 10.0, 0)           # hovering behind a stopped car
    b = smp(1, 11.0, 0)           # still hovering: no change
    c = smp(2, 12.0, 40)          # pulled away: fwd 0 -> ~2 m within 1 s
    d = smp(3, 12.0, 40, mirror=True)   # its mirror: the mirror side has no 11.0 s -> no change
    e = smp(4, 12.5, 40, run="r2")      # another run: nothing a second earlier
    f = smp(5, 13.0, 41)          # steady at speed: no change
    got = FT.oversample([a, b, c, d, e, f], 1, 3, change_oversample=3)
    assert got.count(c) == 3 and got.count(f) == 1 and got.count(a) == 1
    assert got.count(d) == 1 and got.count(e) == 1 and got.count(b) == 1
    # default off: the list as recorded
    assert FT.oversample([a, b, c], 1, 3) == [a, b, c]
    # with turns as well, a sample is repeated by the larger of the two factors
    t = smp(6, 14.0, 40)
    t2 = FT.check_sample(_rec(target="40 49 55", run="r1", id="r1/7", t_capture=14.0), root, 7)
    got = FT.oversample([f, t2], 2, 3, change_oversample=4)
    assert got.count(t2) == 2 and got.count(f) == 1
    assert t is not None
    ap = FT.build_arg_parser()
    args = ap.parse_args(["--data", "d", "--run", "x", "--change-oversample", "3"])
    assert args.change_oversample == 3 and args.change_fwd_m == 1.0 and args.change_dt_s == 1.0


def test_the_dry_run_validates_without_a_model():
    d = _tmp()
    (d / "images").mkdir()
    (d / "images" / "a.png").write_bytes(b"x")
    recs = [_rec(), _rec(run="r2", split="holdout", target="40 49 60")]
    (d / "samples.jsonl").write_text("\n".join(json.dumps(r) for r in recs), encoding="utf-8")
    rep = d / "dry.json"
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        rc = FT.main(["--data", str(d), "--run", "x", "--dry-run", "2", "--no-image-check",
                      "--base", str(d / "no-such-base"), "--report", str(rep)])
    r = json.loads(rep.read_text(encoding="utf-8"))
    assert rc == 0, r
    assert "not available" in r["tokenizer"] and r["split"]["train"] == 1
    assert [e["target"] for e in r["encoded"]] == ["96 49 49", "40 49 60"]
    (d / "samples.jsonl").write_text("\n".join(json.dumps(r) for r in
                                               recs + [_rec(run="r2")]), encoding="utf-8")
    with contextlib.redirect_stdout(io.StringIO()):
        rc = FT.main(["--data", str(d), "--run", "x", "--dry-run", "2", "--no-image-check",
                      "--base", str(d / "no-such-base"), "--report", str(rep)])
    assert rc == 1 and "r2" in json.loads(rep.read_text(encoding="utf-8"))["refused"]


# ------------------------------------------------------------------ 4. identity

def _adapter_dir(run="follow_r1", epoch=3) -> Path:
    d = _tmp() / run / f"epoch{epoch}"
    d.mkdir(parents=True)
    (d / "adapter_config.json").write_text('{"r": 64}', encoding="utf-8")
    (d / "adapter_model.safetensors").write_bytes(b"\x00" * 64)
    prov = FT.provenance(run=run, epoch=epoch, parent_adapter=FT.DEFAULT_ADAPTER,
                         parent_sha256="ab" * 32, base=FT.DEFAULT_BASE,
                         data_report={"file": "D:/data/samples.jsonl", "sha256": "cd" * 32},
                         split_info={"train": 10, "held_out": 2, "train_runs": ["a"],
                                     "held_out_runs": ["b"]},
                         precision="bf16", hyper={"lr": 5e-5}, metrics=None, step=40)
    (d / C.ADAPTER_PROVENANCE_FILE).write_text(json.dumps(prov), encoding="utf-8")
    return d


def test_a_checkpoint_is_announced_and_recorded_under_its_own_id():
    import vla_mission as VM
    import vla_server_v2 as S
    import todo as T
    d = _adapter_dir()
    a_sha = "1a2b3c4d" + "e" * 56
    info = S.adapter_identity(d, a_sha)
    assert info["variant"] == "fine-tune" and info["sha256"] == a_sha
    assert info["provenance"]["run"] == "follow_r1" and info["provenance"]["epoch"] == 3
    assert info["provenance"]["parent_adapter_sha256"] == "ab" * 32
    ident = S.model_identity("D:/models/openvla-7b", d, "bf16", "sdpa", False, "ef" * 32,
                             adapter_info=info)
    assert ident["id"] == "aerialvla-ft-follow/follow_r1/epoch3@1a2b3c4d", ident["id"]
    assert ident["adapter"] is info
    # upstream's adapter keeps its id; a folder without provenance is "other"
    up = S.adapter_identity(FT.DEFAULT_ADAPTER, "00" * 32)
    assert up["variant"] == "original" and up["provenance"] is None
    plain = _tmp() / "x" / "epoch1"
    plain.mkdir(parents=True)
    assert S.adapter_identity(plain, None)["variant"] == "other"
    assert S.model_identity("b", plain, "bf16", "sdpa", False, None)["id"] == "x/epoch1@unknown"
    assert "adapter" not in S.model_identity("b", plain, "bf16", "sdpa", False, None)
    # a provenance that does not read as ours (a path in its run) is not one
    bad = _adapter_dir()
    p = json.loads((bad / C.ADAPTER_PROVENANCE_FILE).read_text(encoding="utf-8"))
    p["run"] = "../evil"
    (bad / C.ADAPTER_PROVENANCE_FILE).write_text(json.dumps(p), encoding="utf-8")
    assert S.read_adapter_provenance(bad) is None and S.adapter_identity(bad, None)["variant"] == "other"
    # the pilot side's model id names the fine-tune; upstream's stays MODEL_ID
    full = dict(ident, weights_sha256="ef" * 32)
    assert VM.model_id_for(full) == "aerialvla:openvla-7b+aerialvla-ft-follow/follow_r1/epoch3"
    assert VM.model_id_for({"id": "aerialvla-lora/aero_vla@196f2f32"}) == VM.MODEL_ID
    assert VM.model_id_for({"id": "fake:aerialvla-ft-follow/a/epoch1@x"}) == VM.MODEL_ID
    rec = VM.pilot_entry(full, {}, [], [], server={}, reported_by="127.0.0.1:8790")
    assert rec["kind"] == "vla" and rec["model_id"].endswith("aerialvla-ft-follow/follow_r1/epoch3")
    assert rec["server_identity"]["adapter"]["variant"] == "fine-tune"
    man = VM.manifest_for("a" * 64, full, 1, ROOT / "demo" / "pas_config" / "scene_vla_det.jsonc")
    h = man["vla_model_hash"]
    assert h == f"aerialvla:openvla-7b+aerialvla-ft-follow/follow_r1/epoch3@sha256:{'ef' * 32}", h
    # the dashboard rule: a VLA, recorded as the fine-tune
    for mid in (ident["id"], h):
        assert C.adapter_variant(mid) == T.adapter_variant(mid) == "fine-tune", mid
        assert T.classify_pilot(mid) == "vla", mid
    ev = {"source": "vla_steps.jsonl", "decisions": 40, "digests": ["ef" * 32],
          "model_ids": [ident["id"]], "requests": 40, "decision_share": 1.0,
          "vla_step_ticks": 300, "vla_ticks": 380, "mission_ticks": 400, "vla_tick_share": 0.95,
          "weights_sha256": "ef" * 32, "model_id": ident["id"]}
    assert T.classify_run(h, ev) == ("vla", None)
    # configured as upstream's adapter, served by the fine-tune: not that flight
    up_h = f"{VM.MODEL_ID}@sha256:{'ef' * 32}"
    cls, why = T.classify_run(up_h, ev)
    assert cls == "vla_unverified" and "original adapter, server the fine-tune" in why, why


def test_the_dashboard_and_codec_agree_on_the_adapter_prefix():
    import todo as T
    assert T.FINE_TUNE_ID_ROOTS == C.FINE_TUNE_ID_ROOTS
    assert T.ORIGINAL_ADAPTER_ID == C.ORIGINAL_ADAPTER_ID
    ids = ["aerialvla-lora/aero_vla@196f2f32", "aerialvla:openvla-7b+aero_vla@sha256:ab",
           "aerialvla-ft-follow/r1/epoch2@1a2b3c4d", "aerialvla-ft-follow/",
           "aerialvla:openvla-7b+aerialvla-ft-follow/r1/epoch2@sha256:ab",
           "fake:aerialvla-ft-follow/r1/epoch2@x", "fake:scripted", "openvla/openvla-7b@47a0",
           "run1/epoch1@unknown", "", None, 7, "aerialvla-ft-followx/r1/epoch1@1"]
    for mid in ids:
        assert C.adapter_variant(mid) == T.adapter_variant(mid), mid
    assert T.classify_pilot("fake:aerialvla-ft-follow/r1/epoch2@x") == "stub"
    assert T.classify_pilot("run1/epoch1@unknown") == "unknown"


# ------------------------------------------------------------------ 5. the eval tool

def test_the_eval_tool_refuses_a_server_serving_another_adapter():
    import eval_vla_adapter as EV
    mine = "12" * 32
    ok = {"id": "x", "adapter": {"path": "D:/a", "sha256": mine}}
    assert EV.check_served_adapter(ok, "D:/a", digest_fn=lambda a: mine) is None
    why = EV.check_served_adapter(dict(ok, adapter={"path": "D:/b", "sha256": "34" * 32}), "D:/a",
                                  digest_fn=lambda a: mine)
    assert why and "D:/b" in why
    assert "no adapter digest" in EV.check_served_adapter({"id": "fake:scripted"}, "D:/a",
                                                         digest_fn=lambda a: mine)
    assert "cannot read" in EV.check_served_adapter(ok, "D:/a", digest_fn=lambda a: None)
    a = {"parse_ok_rate": 0.9, "step_mae_norm": 0.05, "turn_sign_agree": 0.8,
         "turn_sign_agree_on_turns": 0.7, "step_mae": {"fwd_m": 0.5, "down_m": 0.1, "dpsi_rad": 0.1},
         "exact_bin_rate": {"fwd_m": 0.3, "down_m": 0.9, "dpsi_rad": 0.4}}
    b = {"parse_ok_rate": 0.8, "step_mae_norm": 0.10, "turn_sign_agree": 0.5,
         "turn_sign_agree_on_turns": 0.2, "step_mae": {"fwd_m": 1.5, "down_m": 0.1, "dpsi_rad": 0.3},
         "exact_bin_rate": {"fwd_m": 0.1, "down_m": 0.9, "dpsi_rad": 0.2}}
    dl = EV.delta(a, b)
    assert dl["step_mae_norm"] == -0.05 and dl["step_mae.fwd_m"] == -1.0
    assert dl["exact_bin_rate.dpsi_rad"] == 0.2
    s = FT.check_sample(_rec(pose={"x": 1.0, "y": "bad", "up": 8.0, "yaw": 0.5}), _tmp(), 0)
    assert EV._pose(s) == {"x": 1.0, "y": 0.0, "up": 8.0, "yaw_rad": 0.5}


def _synthetic_dataset(Image, runs=(("r1", "train"), ("r2", "train"), ("r3", "holdout")),
                       n=4) -> Path:
    import random
    d = _tmp()
    (d / "images").mkdir()
    rng = random.Random(5)
    rows = []
    for run, split in runs:
        for k in range(n):
            front = Image.new("RGB", (256, 256), (rng.randrange(256), 40, 90))
            down = Image.new("RGB", (256, 256), (10, rng.randrange(256), 200))
            for x in range(0, 256, 9):
                front.putpixel((x, (x * 7) % 256), (250, 250, 0))
            m, _ = C.build_mosaic(front, down)
            name = f"images/{run}_{k}.png"
            m.save(d / name)
            ph = C.DIRECTION_VOCABULARY[k % 7]
            b = (C.quantize(4.5 - k, "fwd_m"), 49, C.quantize(0.2 * (k - 1), "dpsi_rad"))
            rows.append(_rec(phrase=ph, target=f"{b[0]:02d} {b[1]:02d} {b[2]:02d}", run=run,
                             split=split, image=name, image_sha256=C.mosaic_sha256(m),
                             t_capture=10.0 + k))
    (d / "samples.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")
    return d


def test_the_eval_tool_sends_what_a_flight_sends_and_the_server_sees_the_dataset():
    Image = _pil()
    if Image is None:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    import eval_vla_adapter as EV
    import vla_server_v2 as S
    d = _synthetic_dataset(Image)
    samples, _ = FT.load_samples(d)
    held = [s for s in samples if s.split == "holdout"]
    m = FT.load_mosaic(held[0].image, held[0].image_sha256)
    req = EV.build_request(held[0], m, 1)
    assert set(req) == set(C.INFER_FIELDS) and req["phrase"] == held[0].phrase
    assert req["object"] == OBJ and req["csp_mode"] == "off" and req["t_capture"] == 10.0
    model = S.ScriptedModel([held[0].target + "</s>"])
    core = S.V2Server(model, dict(S.FAKE_IDENTITY), missing_down="refuse")
    met, outs, fid = EV.run_requests(held, lambda r: core.handle_request(r)[0], log=lambda s: None)
    assert fid["answered"] == len(held) and fid["refused"] == 0
    assert fid["image_sha256_mismatch"] == 0 and fid["prompt_sha256_mismatch"] == 0, fid
    seen_mosaic, seen_prompt = model.calls[0]
    assert seen_prompt == held[0].prompt and C.mosaic_sha256(seen_mosaic) == held[0].image_sha256
    assert met["n"] == len(held) and met["parse_ok_rate"] == 1.0
    assert met["exact_bin_rate"]["all"] == 0.25          # one scripted answer, four targets
    # a mosaic that is not the dataset's is refused before anything is sent
    wrong = held[0].image.parent / "wrong.png"
    Image.new("RGB", (224, 224)).save(wrong)
    try:
        FT.load_mosaic(wrong)
    except FT.EncodeError:
        pass
    else:
        raise AssertionError("a 224x224 image was taken as a mosaic")
    try:
        FT.load_mosaic(held[1].image, held[0].image_sha256)
    except FT.EncodeError:
        pass
    else:
        raise AssertionError("a sha256 mismatch was accepted")


class _Proc:
    """A stand-in image processor: pixel_values (1, 6, 4, 4) from the mosaic's mean."""

    def __call__(self, images, return_tensors="pt"):
        import torch
        b = images.tobytes()
        return {"pixel_values": torch.full((1, 6, 4, 4), sum(b[:4096]) / 4096 / 255.0)}


def _tiny_loader(answer: str):
    """load_trainable's stand-in: a tiny causal LM over _CharTok's ids that
    really trains (cross-entropy on the labels, shifted), generates a fixed
    answer, and saves a two-file 'adapter'."""
    import torch

    class Tiny(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.emb = torch.nn.Embedding(400, 16)
            self.img = torch.nn.Linear(6, 16)
            self.head = torch.nn.Linear(16, 400)

        def forward(self, input_ids, attention_mask=None, labels=None, pixel_values=None):
            h = self.emb(input_ids) + self.img(pixel_values.float().mean(dim=(2, 3)))[:, None, :]
            logits = self.head(h).float()
            loss = torch.nn.functional.cross_entropy(
                logits[:, :-1].reshape(-1, 400), labels[:, 1:].reshape(-1),
                ignore_index=FT.IGNORE_INDEX)
            return type("Out", (), {"loss": loss})()

        def generate(self, input_ids, attention_mask, pixel_values, max_new_tokens, do_sample,
                     eos_token_id, pad_token_id):
            ans = torch.tensor([[3 + ord(c) for c in answer] + [eos_token_id[0]]])
            return torch.cat([input_ids, ans], dim=1)

        def save_pretrained(self, path):
            Path(path).mkdir(parents=True, exist_ok=True)
            (Path(path) / "adapter_config.json").write_text('{"r": 64}', encoding="utf-8")
            torch.save(self.state_dict(), Path(path) / "adapter_model.bin")

    def load(base, adapter, precision, attn, device, grad_ckpt=True):
        m = Tiny()
        n = sum(p.numel() for p in m.parameters())
        return m, _CharTok(), _Proc(), {"trainable_params": n, "all_params": n}
    return load


def test_the_training_loop_writes_checkpoints_the_server_announces():
    torch, Image = _torch(), _pil()
    if torch is None or Image is None:
        print("      SKIP: torch and Pillow are needed (the vla-real env has both)")
        return SKIP
    import vla_server_v2 as S
    d = _synthetic_dataset(Image)
    parent = _tmp() / "parent"
    parent.mkdir()
    (parent / "adapter_config.json").write_text("{}", encoding="utf-8")
    out = _tmp()
    ap = FT.build_arg_parser()
    args = ap.parse_args(["--data", str(d), "--run", "cpu_t", "--epochs", "3", "--patience", "1",
                          "--batch-size", "2", "--grad-accum", "1", "--log-every", "1",
                          "--lr", "0.05", "--eval-max", "0", "--device", "cpu",
                          "--adapter", str(parent), "--out-root", str(out), "--base", "unused"])
    FT.check_args(ap, args)
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        rc = FT.train(args, loader=_tiny_loader("96 49 49"))
    assert rc == 0
    run = out / "cpu_t"
    rec = json.loads((run / "run.json").read_text(encoding="utf-8"))
    # the same answer every epoch: no improvement after epoch 1, patience 1
    assert (run / "epoch1").is_dir() and (run / "epoch2").is_dir() and not (run / "epoch3").exists()
    assert rec["stopped"].startswith("early stop") and rec["best"]["epoch"] == 1, rec["best"]
    assert [e["epoch"] for e in rec["evaluations"]] == [0, 1, 2]
    assert rec["split"]["held_out_runs"] == ["r3"] and rec["data"]["kept"] == 12
    loss = (run / "loss_log.csv").read_text(encoding="utf-8").splitlines()
    assert loss[0] == "run,epoch,step,loss,lr,tokens,elapsed_s" and len(loss) == 1 + 8, loss
    assert all(math.isfinite(float(r.split(",")[3])) for r in loss[1:])
    ev = (run / "eval_log.csv").read_text(encoding="utf-8").splitlines()
    assert ev[0].split(",")[:3] == ["run", "epoch", "step"] and len(ev) == 4
    he = json.loads((run / "heldout_eval_epoch1.json").read_text(encoding="utf-8"))
    assert he["metrics"]["n"] == 4 and he["metrics"]["parse_ok_rate"] == 1.0
    assert all(o["raw_text"] == "96 49 49</s>" for o in he["outputs"])
    # the checkpoint directory holds the adapter and its provenance only:
    # the server's digest covers every file in it
    assert sorted(p.name for p in (run / "epoch1").iterdir()) == \
        sorted(["adapter_config.json", "adapter_model.bin", C.ADAPTER_PROVENANCE_FILE])
    p1 = json.loads((run / "epoch1" / C.ADAPTER_PROVENANCE_FILE).read_text(encoding="utf-8"))
    assert p1["heldout_metrics"]["n"] == 4, "the provenance was not rewritten with the metrics"
    prov = json.loads((run / "epoch2" / C.ADAPTER_PROVENANCE_FILE).read_text(encoding="utf-8"))
    assert prov["id_root"] == "aerialvla-ft-follow" and prov["epoch_dir"] == "epoch2"
    assert prov["parent_adapter"] == parent.as_posix() and prov["parent_adapter_sha256"]
    assert prov["samples_sha256"] == rec["data"]["sha256"]
    ident = S.model_identity("D:/models/openvla-7b", run / "epoch2", "bf16", "sdpa", False,
                             "ef" * 32, adapter_info=S.adapter_identity(run / "epoch2", "9f" * 32))
    assert ident["id"] == "aerialvla-ft-follow/cpu_t/epoch2@9f9f9f9f", ident["id"]
    # a run name already used is refused, so a recorded checkpoint is never replaced
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            FT.train(args, loader=_tiny_loader("96 49 49"))
    except SystemExit as exc:
        assert "exists" in str(exc)
    else:
        raise AssertionError("an existing run was overwritten")


# ------------------------------------------------------------------ 6. the real files

def test_the_real_tokenizer_and_processor_match_the_servers_preprocessing():
    base = Path(FT.DEFAULT_BASE)
    Image = _pil()
    try:
        import transformers                                        # noqa: F401
    except ImportError:
        print("      SKIP: transformers is not installed (the vla-real env has it)")
        return SKIP
    if Image is None or not (base / "tokenizer.json").is_file():
        print(f"      SKIP: Pillow or {base} is not here (the lab machine has both)")
        return SKIP
    tok, imgproc = FT.load_tokenizer_and_processor(str(base))
    for ph in C.PHRASES:
        p = C.build_prompt(ph, OBJ)
        for tgt in ("96 48 49", "00 49 49 LAND", "45 49 62", "05 50 30"):
            e = FT.encode_text(tok, p, tgt)
            assert e["answer_text"] == tgt + "</s>", (ph, tgt)
            # upstream wrote the EOS as text; it is the same token
            assert tok(p + tgt + "</s>")["input_ids"] == e["input_ids"], (ph, tgt)
    d = _synthetic_dataset(Image, n=2)
    samples, _ = FT.load_samples(d)
    par = FT.server_parity(samples[0], tok, imgproc)
    assert par["ok"], par


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
        except Exception as e:                        # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
