"""
Fine-tune AerialVLA on the hand-written follow's demonstrations, FORMAT-EXACT:
the prompt, the image and the answer text the model is trained on are, byte
for byte and pixel for pixel, what demo/vla_server_v2.py gives it and reads
back in flight.

    # CI / no model: validate the dataset, encode N samples (the real tokenizer
    # and image processor when they are on disk - CPU only, never the 7B model)
    python training/finetune_aerialvla_follow.py --data D:/datasets/follow_v1 --run follow_r1 --dry-run 16

    # train (vla-real env, RTX 4090, NOTHING ELSE ON THE GPU - stop the VLA
    # server first): bf16 LoRA with gradient checkpointing, nf4 QLoRA fallback
    python training/finetune_aerialvla_follow.py --data D:/datasets/follow_v1 --run follow_r1 \\
        --epochs 4 --precision bf16
    python training/finetune_aerialvla_follow.py ... --precision nf4      # if bf16 does not fit

    # a 5-minute smoke run (a checkpoint after 20 optimizer steps, 32 held-out samples)
    python training/finetune_aerialvla_follow.py ... --run smoke --max-steps 20 --eval-max 32

WHY A NEW TRAINER (training/finetune_aerialvla.py, July, is kept as it was)
--------------------------------------------------------------------------
The July trainer is not what the model sees in flight, four ways:
  * its labels ROUND (upstream truncates: aerovla_dataset._quantize_action,
    int(norm * 98)) and are not zero-padded ("5 49 49" for "05 49 49");
  * it tokenises the answer apart from the prompt with add_special_tokens
    False, so SentencePiece's dummy-prefix space is added again after the
    prompt's own trailing "\u2581" - a token sequence the server never feeds;
  * it trains on ONE image where upstream (and our server) give the 224x448
    front-over-down mosaic;
  * its prompt is its own ("<image>\\n{instruction}\\nAction: " from its data).
Everything here comes from guardrail/aerialvla_codec.py instead - the prompt
(build_prompt), the bins (quantize = truncation, two digits), the mosaic
(MOSAIC_SIZE) and the strict reader (parse_action) - and one inference is
demo/vla_server_v2.generate_text, the server's own.

THE ENCODING (upstream aerovla_dataset.__getitem__ plus its collator's mask)
---------------------------------------------------------------------------
  full text  = prompt + target                 e.g. "<image>\\nFly straight ahead and
                                                find the target. a red car\\nAction: 96 48 49"
  input_ids  = tok(full text) + [EOS]           (BOS first, as tok(prompt) gives the server)
  labels     = -100 over tok(prompt)'s tokens, then the answer's tokens and EOS
The prompt's tokens must be a PREFIX of the full text's (checked on every
sample: the prompt ends in the separate token "\u2581" and the answer's digits
follow it, tokenizer probe 2026-10-21 and again 2026-10-23 on all 8 phrases),
and decode(answer tokens + EOS, skip_special_tokens=False) must be exactly
target + "</s>" - the text the server decodes and parse_action(require_eos)
accepts. Loss is on the answer tokens and EOS only. Prismatic inserts the 256
patch embeddings after BOS and masks their labels itself
(modeling_prismatic.py forward); batches are padded on the RIGHT, so position
0 stays BOS. pixel_values = the Prismatic processor on the dataset's 224x448
mosaic, as the server calls it on the mosaic it builds from the front and down
frames (the processor squashes it to 224x224).

THE DATA (tools/build_follow_dataset.py, branch FT-D)
-----------------------------------------------------
DATASET/samples.jsonl, one JSON object per sample, at least:
  image   the 224x448 mosaic, a path relative to DATASET (or absolute)
          (also read: image_path, mosaic, mosaic_path)
  prompt  the exact prompt text (must re-build from its phrase and object
          with aerialvla_codec.build_prompt; csp off)
  target  the exact answer text "FF DD YY" (or "FF DD YY LAND"), without EOS
          (also read: target_text, label_text, action_text - the builder's)
  run     the run tag (also run_tag)
  split   "train", or a held-out split ("holdout", "val", "eval", "test")
optional and checked when present: phrase, object (must equal the prompt's),
bins (must equal the target's), image_sha256 / mosaic_sha256 (the mosaic's
raw RGB bytes, aerialvla_codec.mosaic_sha256), pose, t_capture. A sample that
fails a check is dropped and counted by reason; a run found in both train and
a held-out split refuses the dataset (hold-out runs never in train).

THE LOWER TILE (2026-10-24; aerialvla_codec, THE MOSAIC'S LOWER TILE). The
builder writes the mosaic's lower tile - "down", or "prev_front" (the front
frame dt_s earlier) - into stats.json and every sample (`bottom_tile`).
dataset_bottom_tile reads it (a dataset without it is "down") and refuses a
dataset whose samples mix tiles or disagree with its stats.json; the trainer
copies it, with the label horizon (stats.json args.horizon_s), into every
checkpoint's provenance file - the file the server announces in hello, so a
flight with another tile is refused before take-off (demo/vla_mission.
bottom_tile_check).

THE RUN (outputs under --out-root, default D:/models/aerialvla-ft-follow)
-------------------------------------------------------------------------
  <run>/epoch<N>/            the adapter (save_pretrained) and the provenance
                             file aerialvla_ft_provenance.json (the server's id
                             and `adapter` block come from it, the lower tile
                             and the label horizon included) - nothing else:
                             the server's digest covers every file here, so
                             write no other file into it
  <run>/heldout_eval_epoch<N>.json   that epoch's held-out answers and metrics
  <run>/loss_log.csv         run, epoch, step, loss, lr, tokens, elapsed_s
  <run>/eval_log.csv         one row per evaluation (epoch 0 = the parent
                             adapter before training, same precision)
  <run>/run.json             arguments, data statistics, every evaluation, the
                             best epoch and why training stopped
The held-out metrics (score_outputs): parse-ok rate (the strict reader),
exact-bin accuracy per axis, step MAE in metres / radians, turn-sign
agreement, LAND agreement. Early stopping watches step_mae_norm (each axis's
MAE over its range, averaged; a parse failure counts as the zero step the
client holds) with --patience.

PRECISION. bf16 LoRA (the base in bf16 as served, the LoRA weights and the
projector held in fp32 for AdamW, forward under bf16 autocast) with gradient
checkpointing; nf4 QLoRA (the base 4-bit, as the server's --precision nf4) when
bf16 does not fit 24 GB. The per-epoch evaluation runs that training model
under autocast; tools/eval_vla_adapter.py serves the SAVED checkpoint through
the server's own core at serving precision, and is the number to quote.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import random
import re
import subprocess
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import aerialvla_codec as C                           # noqa: E402

SCHEMA = "vlaguard.aerialvla_ft_follow/1"
DEFAULT_BASE = "D:/models/openvla-7b"
DEFAULT_ADAPTER = "D:/models/aerialvla-lora/aero_vla"
ID_ROOT = "aerialvla-ft-follow"
DEFAULT_OUT_ROOT = Path("D:/models") / ID_ROOT
assert ID_ROOT in C.FINE_TUNE_ID_ROOTS, "the server would not announce this trainer's checkpoints"
IGNORE_INDEX = -100
TRAIN_SPLIT = "train"
HOLDOUT_SPLITS = ("holdout", "val", "eval", "test")
IMAGE_KEYS = ("image", "image_path", "mosaic", "mosaic_path")
# action_text: what tools/build_follow_dataset.py writes (the first real
# dataset was refused whole without it, 23 Oct).
TARGET_KEYS = ("target", "target_text", "label_text", "action_text")
RUN_KEYS = ("run", "run_tag")
SHA_KEYS = ("image_sha256", "mosaic_sha256")
RUN_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}")
# The server's prompt, read back: aerialvla_codec.PROMPT_TEMPLATE with its two
# slots (csp off). The match is only a parse; build_prompt re-builds the text
# from what it found, and the sample is kept only when the two are equal.
_PROMPT_RE = re.compile(r"<image>\nFly (?P<direction>.*?)and find the target\. "
                        r"(?P<object>[^\n]*)\nAction: \Z", re.S)
# The yaw bin of no turn: 49/98 of the way from -1.1 to 1.1 is 0.0 exactly.
NO_TURN_BIN = 49


class SampleError(ValueError):
    """A sample the trainer refuses; `code` is its short reason (counted)."""

    def __init__(self, code: str, detail: str):
        super().__init__(f"{code}: {detail}")
        self.code = code
        self.detail = detail


class DataError(ValueError):
    """The dataset as a whole cannot be trained on honestly."""


@dataclass(frozen=True)
class Sample:
    idx: int
    image: Path
    prompt: str
    target: str
    run: str
    split: str
    phrase: str
    object: str
    bins: tuple[int, int, int]
    land: bool
    image_sha256: str | None = None
    extra: dict = field(default_factory=dict, compare=False, hash=False)


# --------------------------------------------------------------------------- #
# The data
# --------------------------------------------------------------------------- #
def split_prompt(prompt: Any) -> tuple[str, str]:
    """(phrase, object) of a prompt that aerialvla_codec.build_prompt makes
    from exactly those two (csp off), or SampleError."""
    if not isinstance(prompt, str):
        raise SampleError("bad_prompt", f"prompt is {type(prompt).__name__}, not text")
    m = _PROMPT_RE.fullmatch(prompt)
    if m is None:
        raise SampleError("bad_prompt", f"not the server's prompt template: {prompt[:80]!r}")
    phrase, obj = m.group("direction"), m.group("object")
    try:
        rebuilt = C.build_prompt(phrase, obj)
    except C.CodecError as exc:
        raise SampleError("bad_prompt", f"the server would refuse it: {exc}") from None
    if rebuilt != prompt:
        raise SampleError("bad_prompt", "does not re-build byte for byte from its phrase and object")
    return phrase, obj


def _first(rec: dict, keys: tuple[str, ...]):
    for k in keys:
        if k in rec and rec[k] is not None:
            return rec[k]
    return None


def check_sample(rec: Any, root: Path, idx: int) -> Sample:
    """One samples.jsonl record -> Sample, or SampleError (module docstring,
    THE DATA, says what is checked)."""
    if not isinstance(rec, dict):
        raise SampleError("not_an_object", f"line {idx}: {type(rec).__name__}")
    img = _first(rec, IMAGE_KEYS)
    if not isinstance(img, str) or not img:
        raise SampleError("no_image", f"none of {IMAGE_KEYS}")
    path = Path(img)
    path = path if path.is_absolute() else Path(root) / path
    prompt = rec.get("prompt")
    phrase, obj = split_prompt(prompt)
    if "phrase" in rec and rec["phrase"] != phrase:
        raise SampleError("phrase_mismatch", f"phrase {rec['phrase']!r}, prompt says {phrase!r}")
    if "object" in rec and rec["object"] != obj:
        raise SampleError("object_mismatch", f"object {rec['object']!r}, prompt says {obj!r}")
    target = _first(rec, TARGET_KEYS)
    if not isinstance(target, str):
        raise SampleError("no_target", f"none of {TARGET_KEYS} is text")
    if target.endswith(C.EOS_TEXT):
        raise SampleError("target_has_eos", "the target carries '</s>': the trainer appends "
                                            "the tokenizer's EOS itself")
    p = C.parse_action(target)
    if not p.parse_ok:
        raise SampleError("bad_target", f"{target!r}: {p.reason}")
    if "bins" in rec and rec["bins"] is not None and list(rec["bins"]) != list(p.bins):
        raise SampleError("bins_mismatch", f"bins {rec['bins']!r}, target says {list(p.bins)}")
    run = _first(rec, RUN_KEYS)
    if not isinstance(run, str) or not run:
        raise SampleError("no_run", f"none of {RUN_KEYS}")
    split = rec.get("split")
    if not isinstance(split, str) or not split:
        raise SampleError("no_split", "split missing")
    sha = _first(rec, SHA_KEYS)
    if sha is not None and not (isinstance(sha, str) and re.fullmatch(r"[0-9a-f]{64}", sha)):
        raise SampleError("bad_image_sha256", f"{sha!r}")
    extra = {k: rec[k] for k in ("pose", "t_capture", "id", "sample_id", "bottom_tile")
             if k in rec}
    return Sample(idx=idx, image=path, prompt=prompt, target=target, run=run, split=split,
                  phrase=phrase, object=obj, bins=p.bins, land=p.land, image_sha256=sha,
                  extra=extra)


def samples_file(data: Path) -> Path:
    data = Path(data)
    return data if data.is_file() else data / "samples.jsonl"


def load_samples(data: Path, check_images: bool = True) -> tuple[list[Sample], dict]:
    """(samples kept, report). The report counts lines, what was dropped and
    why (with up to 5 examples per reason), and the file's sha256."""
    f = samples_file(data)
    root = f.parent
    raw = f.read_bytes()
    kept: list[Sample] = []
    dropped: Counter = Counter()
    examples: dict[str, list[str]] = {}
    lines = 0
    for i, line in enumerate(raw.decode("utf-8").splitlines()):
        if not line.strip():
            continue
        lines += 1
        try:
            rec = json.loads(line)
            s = check_sample(rec, root, i)
            if check_images and not s.image.is_file():
                raise SampleError("image_missing", str(s.image))
        except SampleError as exc:
            dropped[exc.code] += 1
            examples.setdefault(exc.code, [])
            if len(examples[exc.code]) < 5:
                examples[exc.code].append(f"line {i}: {exc.detail}"[:200])
            continue
        except ValueError as exc:
            dropped["bad_json"] += 1
            examples.setdefault("bad_json", [])
            if len(examples["bad_json"]) < 5:
                examples["bad_json"].append(f"line {i}: {exc}"[:200])
            continue
        kept.append(s)
    return kept, {"file": f.as_posix(), "sha256": hashlib.sha256(raw).hexdigest(),
                  "lines": lines, "kept": len(kept), "dropped": dict(dropped),
                  "dropped_examples": examples}


def split_samples(samples: list[Sample], holdout_splits: Iterable[str] = HOLDOUT_SPLITS
                  ) -> tuple[list[Sample], list[Sample], dict]:
    """(train, held-out, info). DataError when a run is in both: a held-out
    run never trains. Samples in any other split are left out, counted."""
    hs = set(holdout_splits)
    train = [s for s in samples if s.split == TRAIN_SPLIT]
    held = [s for s in samples if s.split in hs]
    other = Counter(s.split for s in samples if s.split != TRAIN_SPLIT and s.split not in hs)
    both = sorted({s.run for s in train} & {s.run for s in held})
    if both:
        raise DataError(f"runs in both train and a held-out split: {both} - a held-out run must "
                        f"never be trained on (split by RUN)")
    return train, held, {"train": len(train), "held_out": len(held),
                         "train_runs": sorted({s.run for s in train}),
                         "held_out_runs": sorted({s.run for s in held}),
                         "other_splits": dict(other), "held_out_splits": sorted(hs)}


def _tile_key(t: "C.BottomTile") -> tuple:
    return (t.mode, None if t.dt_s is None else round(float(t.dt_s), 6))


def dataset_bottom_tile(data: Path, samples: list[Sample]) -> dict:
    """The lower tile the dataset's mosaics carry (module docstring, THE
    LOWER TILE): {mode, dt_s, tol_s, source, label_horizon_s}. Read from
    stats.json beside samples.jsonl (the builder's) and checked against every
    sample's `bottom_tile` (a sample without one is "down"): the samples
    must agree with one another and with stats.json on the mode and dt_s, or
    DataError. Neither present: "down" (every dataset from before
    2026-10-24). `label_horizon_s`: stats.json's args.horizon_s, or None."""
    stats_p = samples_file(data).parent / "stats.json"
    st_tile = horizon = None
    if stats_p.is_file():
        try:
            st = json.loads(stats_p.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise DataError(f"{stats_p}: unreadable ({exc}), so its lower tile is unknown") \
                from None
        if isinstance(st, dict):
            if st.get("bottom_tile") is not None:
                try:
                    st_tile = C.bottom_tile_from(st["bottom_tile"])
                except ValueError as exc:
                    raise DataError(f"{stats_p}: its lower tile is malformed ({exc})") from None
            h = (st.get("args") or {}).get("horizon_s") if isinstance(st.get("args"), dict) \
                else None
            horizon = float(h) if isinstance(h, (int, float)) and not isinstance(h, bool) else None
    seen: dict[tuple, Any] = {}
    marked = False
    for s in samples:
        rec = s.extra.get("bottom_tile")
        marked |= rec is not None
        try:
            t = C.bottom_tile_from(rec)
        except ValueError as exc:
            raise DataError(f"sample {s.idx}: its lower tile is malformed ({exc})") from None
        seen.setdefault(_tile_key(t), t)
    if len(seen) > 1:
        raise DataError(f"the samples mix lower tiles {sorted(seen, key=str)}: one model learns "
                        f"one tile (tools/build_follow_dataset.py --bottom-tile)")
    s_tile = next(iter(seen.values()), None)
    if st_tile is not None and s_tile is not None and _tile_key(s_tile) != _tile_key(st_tile):
        raise DataError(f"stats.json says the lower tile is {st_tile.as_dict()}, the samples say "
                        f"{s_tile.as_dict()}: rebuild the dataset")
    tile = st_tile or s_tile or C.BottomTile()
    source = ("stats.json" if st_tile is not None else "samples" if marked else
              "absent (a dataset from before 2026-10-24): down")
    return {**tile.as_dict(), "source": source, "label_horizon_s": horizon}


def turn_class(yaw_bin: int) -> int:
    """-1 left, 0 straight, +1 right: the side of the no-turn bin (49)."""
    return (yaw_bin > NO_TURN_BIN) - (yaw_bin < NO_TURN_BIN)


def data_stats(samples: list[Sample]) -> dict:
    """Label histograms per axis (bin -> count), phrase counts, samples per
    run, LAND count and the rotate-only band (|dpsi| >= 0.25 rad) with a
    forward step - steps upstream's executor would only have rotated."""
    hist = {ax: Counter() for ax in C.AXES}
    for s in samples:
        for ax, b in zip(C.AXES, s.bins):
            hist[ax][b] += 1
    rot = sum(1 for s in samples if C.bins_to_step(s.bins).rotate_only and s.bins[0] > 0)
    return {"n": len(samples),
            "bins": {ax: {f"{b:02d}": n for b, n in sorted(h.items())} for ax, h in hist.items()},
            "phrases": dict(Counter(s.phrase for s in samples)),
            "runs": dict(Counter(s.run for s in samples)),
            "land": sum(1 for s in samples if s.land),
            "turning": sum(1 for s in samples if turn_class(s.bins[2]) != 0),
            "rotate_only_band_with_forward": rot}


# --------------------------------------------------------------------------- #
# The encoding
# --------------------------------------------------------------------------- #
class EncodeError(ValueError):
    """A sample whose tokens would not be what the server feeds and reads."""


def _ids(enc) -> list[int]:
    ids = enc["input_ids"]
    if hasattr(ids, "tolist"):
        ids = ids.tolist()
    if ids and isinstance(ids[0], list):          # a batch of one
        ids = ids[0]
    return [int(i) for i in ids]


def encode_text(tok, prompt: str, target: str) -> dict:
    """The token ids and labels of one sample (module docstring, THE
    ENCODING), with every format check: the prompt tokens as the server
    makes them (tok(prompt), its BOS added) are a prefix of tok(prompt +
    target); the answer is followed by EOS; and decoding the answer tokens
    with their special tokens gives exactly target + "</s>", which the strict
    reader accepts with require_eos - the text the server would parse."""
    p_ids = _ids(tok(prompt))
    f_ids = _ids(tok(prompt + target))
    n = len(p_ids)
    if f_ids[:n] != p_ids:
        raise EncodeError("the prompt's tokens are not a prefix of prompt+target's: the model "
                          "would be trained on a token sequence the server never feeds")
    eos = tok.eos_token_id
    if not isinstance(eos, int):
        raise EncodeError(f"the tokenizer has no EOS id ({eos!r})")
    answer = f_ids[n:] + [eos]
    if len(answer) < 2:
        raise EncodeError("no answer tokens after the prompt")
    text = tok.decode(answer, skip_special_tokens=False)
    if text != target + C.EOS_TEXT:
        raise EncodeError(f"the answer decodes to {text!r}, not {target + C.EOS_TEXT!r}")
    parsed = C.parse_action(text, require_eos=True)
    if not parsed.parse_ok:
        raise EncodeError(f"the strict reader refuses the decoded answer {text!r}: {parsed.reason}")
    return {"input_ids": p_ids + answer, "labels": [IGNORE_INDEX] * n + answer,
            "n_prompt": n, "n_answer": len(answer), "answer_text": text}


def load_mosaic(path: Path, expect_sha256: str | None = None):
    """The dataset's mosaic as the RGB image the processor is given: exactly
    aerialvla_codec.MOSAIC_SIZE (224x448), and, when the sample names one,
    its raw-RGB sha256 equal (a JPEG re-encode would change it)."""
    from PIL import Image
    with Image.open(path) as im:
        img = im.convert("RGB")
    if tuple(img.size) != C.MOSAIC_SIZE:
        raise EncodeError(f"{path}: {img.size}, not the {C.MOSAIC_SIZE} mosaic")
    if expect_sha256 is not None and C.mosaic_sha256(img) != expect_sha256:
        raise EncodeError(f"{path}: pixels differ from the sample's image_sha256")
    return img


def encode_sample(s: Sample, tok, imgproc, load_image: Callable = load_mosaic) -> dict:
    """encode_text + pixel_values: imgproc(images=mosaic, return_tensors="pt")
    exactly as the server's generate_text calls it."""
    enc = encode_text(tok, s.prompt, s.target)
    mosaic = load_image(s.image, s.image_sha256)
    enc["pixel_values"] = imgproc(images=mosaic, return_tensors="pt")["pixel_values"]
    enc["image_sha256"] = C.mosaic_sha256(mosaic) if hasattr(mosaic, "tobytes") else None
    return enc


def collate(batch: list[dict], pad_id: int) -> dict:
    """Right-padded tensors: input_ids (pad_id), attention_mask, labels
    (-100 on padding), pixel_values stacked. Right, never left: Prismatic
    puts the patches after position 0, which must be BOS."""
    import torch
    width = max(len(b["input_ids"]) for b in batch)
    ids = torch.full((len(batch), width), int(pad_id), dtype=torch.long)
    att = torch.zeros((len(batch), width), dtype=torch.long)
    lab = torch.full((len(batch), width), IGNORE_INDEX, dtype=torch.long)
    for i, b in enumerate(batch):
        n = len(b["input_ids"])
        ids[i, :n] = torch.tensor(b["input_ids"], dtype=torch.long)
        att[i, :n] = 1
        lab[i, :n] = torch.tensor(b["labels"], dtype=torch.long)
    pv = torch.cat([b["pixel_values"] for b in batch], dim=0)
    return {"input_ids": ids, "attention_mask": att, "labels": lab, "pixel_values": pv}


# --------------------------------------------------------------------------- #
# The held-out metrics (shared with tools/eval_vla_adapter.py)
# --------------------------------------------------------------------------- #
_ZERO_STEP = C.Step(0.0, 0.0, 0.0)


def score_outputs(pairs: Iterable[tuple[str, Any]]) -> dict:
    """(target text, the model's raw text) pairs -> the held-out metrics.

    The raw text is read as the server reads it (parse_action with
    require_eos); a parse failure is the ZERO step the client then holds.
      parse_ok_rate            answers the strict reader accepts
      exact_bin_rate[axis]     the answer's bin equal to the target's (a parse
                               failure counts as wrong); [all] all three
      exact_bin_rate_parsed    the same over parsed answers only
      step_mae[axis]           |decoded answer - decoded target| in metres /
                               radians (both at their bin's lower edge, as
                               flown), parse failures as the zero step
      step_mae_parsed, bin_mae_parsed   over parsed answers only
      step_mae_norm            mean over the axes of step_mae / the axis
                               range - early stopping watches this
      turn_sign_agree          left / straight / right of the yaw bin equal
                               (parsed answers); _on_turns: targets that turn
      land_*                   LAND in targets, in answers, agreement
    """
    n = parsed = land_t = land_p = land_agree = 0
    exact = Counter()
    exact_p = Counter()
    err = {ax: [] for ax in C.AXES}
    err_p = {ax: [] for ax in C.AXES}
    berr_p = {ax: [] for ax in C.AXES}
    turn_n = turn_ok = turns = turns_ok = 0
    reasons: Counter = Counter()
    for target, raw in pairs:
        t = C.parse_action(target)
        if not t.parse_ok:
            raise ValueError(f"target {target!r} is not an answer text: {t.reason}")
        p = C.parse_action(raw, require_eos=True)
        n += 1
        land_t += int(t.land)
        if p.parse_ok:
            parsed += 1
            ps = p.step
            for i, ax in enumerate(C.AXES):
                same = p.bins[i] == t.bins[i]
                exact[ax] += same
                exact_p[ax] += same
                err_p[ax].append(abs(getattr(ps, ax) - getattr(t.step, ax)))
                berr_p[ax].append(abs(p.bins[i] - t.bins[i]))
            exact["all"] += p.bins == t.bins
            exact_p["all"] += p.bins == t.bins
            tc, pc = turn_class(t.bins[2]), turn_class(p.bins[2])
            turn_n += 1
            turn_ok += tc == pc
            if tc != 0:
                turns += 1
                turns_ok += tc == pc
            land_p += int(p.land)
            land_agree += int(p.land == t.land)
        else:
            ps = _ZERO_STEP
            reasons[(p.reason or "?").split(" in ")[0][:60]] += 1
        for ax in C.AXES:
            err[ax].append(abs(getattr(ps, ax) - getattr(t.step, ax)))

    def mean(v):
        return round(sum(v) / len(v), 6) if v else None

    def rate(k, d):
        return round(k / d, 6) if d else None
    width = {ax: hi - lo for ax, (lo, hi) in C.RANGES.items()}
    mae = {ax: mean(err[ax]) for ax in C.AXES}
    return {
        "n": n, "parse_ok": parsed, "parse_ok_rate": rate(parsed, n),
        "parse_fail_reasons": dict(reasons),
        "exact_bin_rate": {k: rate(exact[k], n) for k in C.AXES + ("all",)},
        "exact_bin_rate_parsed": {k: rate(exact_p[k], parsed) for k in C.AXES + ("all",)},
        "step_mae": mae,
        "step_mae_parsed": {ax: mean(err_p[ax]) for ax in C.AXES},
        "bin_mae_parsed": {ax: mean(berr_p[ax]) for ax in C.AXES},
        "step_mae_norm": (round(sum(mae[ax] / width[ax] for ax in C.AXES) / len(C.AXES), 6)
                          if n else None),
        "turn_sign_agree": rate(turn_ok, turn_n),
        "turn_sign_agree_on_turns": rate(turns_ok, turns), "turning_targets_parsed": turns,
        "land_target_rate": rate(land_t, n), "land_pred_rate": rate(land_p, parsed),
        "land_agree_rate": rate(land_agree, parsed),
        "units": {"fwd_m": "m", "down_m": "m", "dpsi_rad": "rad"},
        "parse_failure_flown_as": "the zero step (the client holds zero)",
    }


# --------------------------------------------------------------------------- #
# The model (vla-real: torch, transformers, peft, bitsandbytes)
# --------------------------------------------------------------------------- #
def load_tokenizer_and_processor(base: str):
    """The base's tokenizer and Prismatic image processor (CPU, no model):
    the two the server loads."""
    from transformers import AutoImageProcessor, AutoTokenizer
    tok = AutoTokenizer.from_pretrained(base, trust_remote_code=True)
    imgproc = AutoImageProcessor.from_pretrained(base, trust_remote_code=True)
    return tok, imgproc


def load_trainable(base: str, adapter: str, precision: str, attn: str, device: str = "cuda:0",
                   grad_ckpt: bool = True):
    """OpenVLA-7B with the adapter attached TRAINABLE (it continues from it;
    r, alpha, targets and the projector's modules_to_save are the adapter's
    own, adapter_config.json). The base loads as the server loads it (bf16,
    or nf4 with the projector kept bf16); no resize_token_embeddings, as the
    server. Gradient checkpointing on the Llama backbone. The trainable
    weights (LoRA and the projector) are cast to fp32 for AdamW; the forward
    runs under bf16 autocast (train())."""
    import torch
    from peft import PeftModel
    from transformers import AutoModelForVision2Seq, BitsAndBytesConfig
    tok, imgproc = load_tokenizer_and_processor(base)
    kw: dict[str, Any] = dict(attn_implementation=attn, torch_dtype=torch.bfloat16,
                              device_map={"": device}, low_cpu_mem_usage=True,
                              trust_remote_code=True)
    if precision == "nf4":
        kw["quantization_config"] = BitsAndBytesConfig(
            load_in_4bit=True, bnb_4bit_compute_dtype=torch.bfloat16,
            bnb_4bit_quant_type="nf4", llm_int8_skip_modules=["projector"])
    model = AutoModelForVision2Seq.from_pretrained(base, **kw)
    if grad_ckpt:
        model.gradient_checkpointing_enable(gradient_checkpointing_kwargs={"use_reentrant": False})
        model.enable_input_require_grads()
    model = PeftModel.from_pretrained(model, adapter, is_trainable=True)
    n_train = n_all = 0
    for p in model.parameters():
        n_all += p.numel()
        if p.requires_grad:
            n_train += p.numel()
            if p.dtype != torch.float32:
                p.data = p.data.float()
    return model, tok, imgproc, {"trainable_params": n_train, "all_params": n_all}


def evaluate(model, tok, imgproc, samples: list[Sample], device: str, max_new_tokens: int,
             autocast_dtype=None, load_image: Callable = load_mosaic,
             log: Callable[[str], None] = print) -> tuple[dict, list[dict]]:
    """Generate on every held-out sample the server's way
    (vla_server_v2.generate_text) and score the answers. Returns (metrics,
    outputs: one {idx, run, target, raw_text} per sample)."""
    from vla_server_v2 import generate_text
    was_training = model.training
    model.eval()
    outs = []
    t0 = time.time()
    try:
        for k, s in enumerate(samples):
            mosaic = load_image(s.image, s.image_sha256)
            g = generate_text(model, tok, imgproc, mosaic, s.prompt, device=device,
                              max_new_tokens=max_new_tokens, eos_id=tok.eos_token_id,
                              pad_id=tok.pad_token_id, autocast_dtype=autocast_dtype)
            outs.append({"idx": s.idx, "run": s.run, "phrase": s.phrase, "target": s.target,
                         "raw_text": g["raw_text"]})
            if (k + 1) % 50 == 0:
                log(f"[eval] {k + 1}/{len(samples)} ({time.time() - t0:.0f} s)")
    finally:
        if was_training:
            model.train()
    return score_outputs((o["target"], o["raw_text"]) for o in outs), outs


# --------------------------------------------------------------------------- #
# Records
# --------------------------------------------------------------------------- #
def _utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def git_commit() -> str | None:
    try:
        r = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True,
                           text=True, timeout=10)
        return r.stdout.strip() or None
    except Exception:                                              # noqa: BLE001
        return None


def provenance(*, run: str, epoch: int, parent_adapter: str, parent_sha256: str | None,
               base: str, data_report: dict, split_info: dict, precision: str, hyper: dict,
               metrics: dict | None, step: int, bottom_tile: dict | None = None,
               label_horizon_s: float | None = None) -> dict:
    """aerialvla_codec.ADAPTER_PROVENANCE_FILE's content: what the server
    reads to announce the checkpoint (id_root, run, epoch_dir) and what the
    identity's `adapter.provenance` repeats (parent adapter and its digest,
    base, data and its sha256, precision, the lower tile and the label
    horizon). `bottom_tile`: dataset_bottom_tile's ({mode, dt_s, tol_s}
    kept); None records "down"."""
    tile = C.bottom_tile_from(bottom_tile).as_dict()
    return {"schema": SCHEMA, "id_root": ID_ROOT, "run": run, "epoch": int(epoch),
            # the mosaic's lower tile this adapter learned: a flight with
            # another is refused (demo/vla_mission.bottom_tile_check)
            "bottom_tile": tile, "label_horizon_s": label_horizon_s,
            "epoch_dir": f"epoch{int(epoch)}", "optimizer_step": int(step),
            "parent_adapter": Path(parent_adapter).as_posix(),
            "parent_adapter_sha256": parent_sha256, "base": Path(base).as_posix(),
            "data": data_report.get("file"), "samples_sha256": data_report.get("sha256"),
            "train_runs": split_info.get("train_runs"),
            "held_out_runs": split_info.get("held_out_runs"),
            "n_train": split_info.get("train"), "n_held_out": split_info.get("held_out"),
            "precision": precision, "hyperparameters": hyper, "heldout_metrics": metrics,
            "code_commit": git_commit(), "created_utc": _utc()}


EVAL_COLUMNS = ("run", "epoch", "step", "n", "parse_ok_rate", "exact_fwd", "exact_down",
                "exact_dpsi", "exact_all", "mae_fwd_m", "mae_down_m", "mae_dpsi_rad",
                "step_mae_norm", "turn_sign_agree", "turn_sign_agree_on_turns", "land_pred_rate",
                "utc")


def eval_row(run: str, epoch: int, step: int, m: dict) -> list:
    e, a = m["exact_bin_rate"], m["step_mae"]
    return [run, epoch, step, m["n"], m["parse_ok_rate"], e["fwd_m"], e["down_m"], e["dpsi_rad"],
            e["all"], a["fwd_m"], a["down_m"], a["dpsi_rad"], m["step_mae_norm"],
            m["turn_sign_agree"], m["turn_sign_agree_on_turns"], m["land_pred_rate"], _utc()]


def _append_csv(path: Path, header: tuple, row: list) -> None:
    new = not path.is_file()
    with path.open("a", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(header)
        w.writerow(row)


def _write_json(path: Path, obj) -> None:
    path.write_text(json.dumps(obj, indent=1, default=str), encoding="utf-8")


# --------------------------------------------------------------------------- #
# The run
# --------------------------------------------------------------------------- #
def oversample(train: list[Sample], turn_oversample: int, turn_bins: int) -> list[Sample]:
    """The training list with each sample whose yaw bin is at least
    `turn_bins` from the no-turn bin repeated `turn_oversample` times in all
    (1 = as recorded). The July campaign's run1 collapsed on straight-heavy
    data (unlearned turning); this is the lever, recorded, default off."""
    if turn_oversample <= 1:
        return list(train)
    out = []
    for s in train:
        out.extend([s] * (turn_oversample if abs(s.bins[2] - NO_TURN_BIN) >= turn_bins else 1))
    return out


def lr_at(step: int, total: int, warmup: int, min_frac: float) -> float:
    """The LR multiplier: linear warm-up over `warmup` steps, then cosine
    from 1 to `min_frac` at `total`."""
    if warmup > 0 and step < warmup:
        return (step + 1) / warmup
    span = max(1, total - warmup)
    prog = min(1.0, (step - warmup) / span)
    return min_frac + (1.0 - min_frac) * 0.5 * (1.0 + math.cos(math.pi * prog))


def heldout_subset(held: list[Sample], n: int, seed: int) -> list[Sample]:
    if n <= 0 or len(held) <= n:
        return list(held)
    rng = random.Random(seed)
    return sorted(rng.sample(held, n), key=lambda s: s.idx)


def train(args, loader: Callable = None) -> int:
    """The training run (module docstring, THE RUN). `loader(base, adapter,
    precision, attn, device, grad_ckpt=)` -> (model, tok, imgproc, info) is
    load_trainable; tests pass a small stand-in to run the loop on the CPU."""
    import torch
    from guardrail.manifest import weights_digest
    loader = loader or load_trainable

    out_root = Path(args.out_root)
    run_dir = out_root / args.run
    if run_dir.exists() and any(run_dir.iterdir()) and not args.overwrite:
        raise SystemExit(f"[ft] {run_dir} exists: choose another --run (or --overwrite), so no "
                         f"checkpoint is ever replaced under a name already recorded")
    samples, report = load_samples(Path(args.data))
    tr, held, sinfo = split_samples(samples, args.holdout_splits)
    try:
        tile = dataset_bottom_tile(Path(args.data), samples)
    except DataError as exc:
        raise SystemExit(f"[ft] refused: {exc}") from None
    if not tr:
        raise SystemExit("[ft] no training samples")
    if not held:
        raise SystemExit("[ft] no held-out samples: early stopping and the per-epoch "
                         "evaluation need a held-out run (tools/build_follow_dataset.py --holdout)")
    train_list = oversample(tr, args.turn_oversample, args.turn_bins)
    eval_set = heldout_subset(held, args.eval_max, args.seed)
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"[ft] data {report['file']}: {report['kept']} of {report['lines']} samples kept "
          f"(dropped {report['dropped']}); train {len(tr)} ({len(train_list)} with "
          f"oversampling) from {len(sinfo['train_runs'])} runs, held-out {len(held)} from "
          f"{sinfo['held_out_runs']}, evaluating {len(eval_set)}", flush=True)
    print(f"[ft] lower tile {tile['mode']}"
          + (f" (the front frame {tile['dt_s']:g} s earlier)" if tile["dt_s"] else "")
          + f" from {tile['source']}; label horizon {tile['label_horizon_s']} s - both go "
            f"into every checkpoint's provenance", flush=True)

    parent_sha = weights_digest([Path(args.adapter)])
    device = args.device
    model, tok, imgproc, pinfo = loader(args.base, args.adapter, args.precision,
                                        args.attn, device, grad_ckpt=not args.no_grad_ckpt)
    dev_type = str(device).split(":", 1)[0]
    print(f"[ft] {args.precision} base, adapter {args.adapter} (sha256 {str(parent_sha)[:12]}), "
          f"trainable {pinfo['trainable_params'] / 1e6:.1f} M of {pinfo['all_params'] / 1e9:.2f} B",
          flush=True)
    autocast = torch.bfloat16
    params = [p for p in model.parameters() if p.requires_grad]
    opt = torch.optim.AdamW(params, lr=args.lr, weight_decay=args.weight_decay)
    per_epoch = math.ceil(len(train_list) / (args.batch_size * args.grad_accum))
    total = per_epoch * args.epochs if args.max_steps <= 0 else min(per_epoch * args.epochs,
                                                                     args.max_steps)
    warmup = int(round(total * args.warmup_frac))
    sched = torch.optim.lr_scheduler.LambdaLR(
        opt, lambda s: lr_at(s, total, warmup, args.min_lr_frac))
    hyper = {k: getattr(args, k) for k in (
        "epochs", "lr", "min_lr_frac", "warmup_frac", "weight_decay", "batch_size", "grad_accum",
        "max_steps", "patience", "turn_oversample", "turn_bins", "eval_max", "seed",
        "max_new_tokens", "attn")}
    hyper.update({"optimizer_steps_planned": total, "warmup_steps": warmup,
                  "grad_ckpt": not args.no_grad_ckpt, "lora": "the parent adapter's own config"})
    runrec = {"schema": SCHEMA, "run": args.run, "created_utc": _utc(), "argv": sys.argv[1:],
              "data": report, "split": sinfo, "bottom_tile": tile,
              "train_stats": data_stats(tr),
              "held_out_stats": data_stats(held), "parent_adapter": args.adapter,
              "parent_adapter_sha256": parent_sha, "base": args.base,
              "precision": args.precision, "params": pinfo, "hyperparameters": hyper,
              "evaluations": [], "best": None, "stopped": None, "code_commit": git_commit()}
    _write_json(run_dir / "run.json", runrec)
    loss_csv, eval_csv = run_dir / "loss_log.csv", run_dir / "eval_log.csv"

    def do_eval(epoch: int, step: int) -> dict:
        m, outs = evaluate(model, tok, imgproc, eval_set, device, args.max_new_tokens,
                           autocast_dtype=autocast)
        _append_csv(eval_csv, EVAL_COLUMNS, eval_row(args.run, epoch, step, m))
        runrec["evaluations"].append({"epoch": epoch, "step": step, "metrics": m})
        print(f"[eval] epoch {epoch}: parse_ok {m['parse_ok_rate']} exact {m['exact_bin_rate']} "
              f"step MAE {m['step_mae']} norm {m['step_mae_norm']} turn-sign "
              f"{m['turn_sign_agree']}", flush=True)
        return {"metrics": m, "outputs": outs}

    if not args.no_eval_before:
        base_eval = do_eval(0, 0)
        _write_json(run_dir / "heldout_eval_epoch0_parent.json",
                    {"note": "the parent adapter before any training, same precision",
                     **base_eval})
    best_score, best_epoch, bad = math.inf, None, 0
    step = 0
    model.train()
    t0 = time.time()
    stop = None
    for epoch in range(1, args.epochs + 1):
        order = list(range(len(train_list)))
        random.Random(args.seed + epoch).shuffle(order)
        micro = [order[i:i + args.batch_size] for i in range(0, len(order), args.batch_size)]
        opt.zero_grad(set_to_none=True)
        run_loss, run_n, run_tok, enc_fail = 0.0, 0, 0, Counter()
        for mi, idxs in enumerate(micro):
            batch = []
            for i in idxs:
                try:
                    batch.append(encode_sample(train_list[i], tok, imgproc))
                except (EncodeError, OSError) as exc:
                    enc_fail[type(exc).__name__] += 1
                    if sum(enc_fail.values()) <= 5:
                        print(f"[ft] *** sample {train_list[i].idx} not encoded: {exc}")
            if not batch:
                continue
            b = collate(batch, tok.pad_token_id)
            with torch.autocast(device_type=dev_type, dtype=autocast):
                out = model(input_ids=b["input_ids"].to(device),
                            attention_mask=b["attention_mask"].to(device),
                            labels=b["labels"].to(device),
                            pixel_values=b["pixel_values"].to(device, dtype=torch.bfloat16))
            (out.loss / args.grad_accum).backward()
            run_loss += float(out.loss.detach())
            run_n += 1
            run_tok += int((b["labels"] != IGNORE_INDEX).sum())
            last = mi == len(micro) - 1
            if (mi + 1) % args.grad_accum == 0 or last:
                torch.nn.utils.clip_grad_norm_(params, args.max_grad_norm)
                opt.step()
                sched.step()
                opt.zero_grad(set_to_none=True)
                step += 1
                if step % args.log_every == 0 or last:
                    lr = sched.get_last_lr()[0]
                    _append_csv(loss_csv, ("run", "epoch", "step", "loss", "lr", "tokens",
                                           "elapsed_s"),
                                [args.run, epoch, step, round(run_loss / max(1, run_n), 6),
                                 f"{lr:.3e}", run_tok, round(time.time() - t0, 1)])
                    print(f"[ft] epoch {epoch} step {step}/{total} loss "
                          f"{run_loss / max(1, run_n):.4f} lr {lr:.2e} "
                          f"({(time.time() - t0) / 60:.1f} min)", flush=True)
                    run_loss, run_n, run_tok = 0.0, 0, 0
                if args.max_steps > 0 and step >= args.max_steps:
                    stop = f"max_steps {args.max_steps}"
                    break
        ck = run_dir / f"epoch{epoch}"
        ck.mkdir(parents=True, exist_ok=True)
        model.save_pretrained(str(ck))

        def _prov(metrics):
            # written with the checkpoint (a run that dies in the evaluation
            # still leaves a checkpoint the server can name), then again with
            # its held-out metrics
            _write_json(ck / C.ADAPTER_PROVENANCE_FILE, provenance(
                run=args.run, epoch=epoch, parent_adapter=args.adapter, parent_sha256=parent_sha,
                base=args.base, data_report=report, split_info=sinfo, precision=args.precision,
                hyper=hyper, metrics=metrics, step=step, bottom_tile=tile,
                label_horizon_s=tile["label_horizon_s"]))
        _prov(None)
        ev = do_eval(epoch, step)
        m = ev["metrics"]
        # beside the checkpoint, never in it: the server's digest covers
        # every file of the adapter directory (vla_server_v2.weight_paths)
        _write_json(run_dir / f"heldout_eval_epoch{epoch}.json",
                    {"note": "training precision (bf16 autocast over fp32 LoRA); "
                             "tools/eval_vla_adapter.py measures the saved checkpoint as served",
                     "checkpoint": ck.as_posix(), "encode_failures": dict(enc_fail), **ev})
        _prov(m)
        score = m["step_mae_norm"]
        if score is not None and score < best_score - args.min_delta:
            best_score, best_epoch, bad = score, epoch, 0
        else:
            bad += 1
        runrec["best"] = {"epoch": best_epoch, "step_mae_norm": best_score,
                          "checkpoint": (run_dir / f"epoch{best_epoch}").as_posix()
                          if best_epoch else None}
        _write_json(run_dir / "run.json", runrec)
        print(f"[ft] saved {ck}; best epoch {best_epoch} (step_mae_norm {best_score:.4f})",
              flush=True)
        if stop:
            break
        if args.patience > 0 and bad >= args.patience:
            stop = f"early stop: step_mae_norm did not improve for {bad} epochs"
            break
    runrec["stopped"] = stop or "all epochs"
    runrec["finished_utc"] = _utc()
    _write_json(run_dir / "run.json", runrec)
    print(f"[ft] done ({runrec['stopped']}) in {(time.time() - t0) / 60:.1f} min; best "
          f"{runrec['best']}", flush=True)
    return 0


# --------------------------------------------------------------------------- #
# The dry run (CI: no model, no GPU)
# --------------------------------------------------------------------------- #
def server_parity(s: Sample, tok, imgproc) -> dict:
    """One sample through the SERVER's preprocessing (protocol v2 request:
    the mosaic's front and down tiles as PNG, decoded, re-mosaicked and the
    prompt built by demo/vla_server_v2.V2Server.infer, captured before the
    model) against this trainer's encoding: the same prompt bytes, the same
    mosaic sha256, the same prompt token ids and the same pixel_values."""
    import torch
    from vla_server_v2 import V2Server
    sys.path.insert(0, str(ROOT / "tools"))
    import eval_vla_adapter as EV

    class _Capture:
        precision, attn, merged, max_new_tokens = "fake", "none", False, 12

        def __init__(self):
            self.seen = None

        def generate(self, mosaic, prompt):
            self.seen = (mosaic, prompt)
            return {"raw_text": s.target + C.EOS_TEXT, "model_s": 0.0}
    cap = _Capture()
    core = V2Server(cap, {"id": "fake:parity", "base": "none", "weights_sha256": None,
                          "precision": "fake", "attn": "none", "merged": False},
                    missing_down="refuse")
    mosaic = load_mosaic(s.image, s.image_sha256)
    resp, _, _ = core.handle_request(EV.build_request(s, mosaic, 1))
    if not resp.get("ok"):
        return {"ok": False, "why": resp.get("error")}
    srv_mosaic, srv_prompt = cap.seen
    enc = encode_sample(s, tok, imgproc)
    srv_ids = _ids(tok(srv_prompt))
    srv_pv = imgproc(images=srv_mosaic, return_tensors="pt")["pixel_values"]
    checks = {"prompt_bytes_equal": srv_prompt == s.prompt,
              "image_sha256_equal": resp["image_sha256"] == C.mosaic_sha256(mosaic),
              "prompt_ids_equal": srv_ids == enc["input_ids"][:enc["n_prompt"]],
              "pixel_values_equal": bool(torch.equal(srv_pv, enc["pixel_values"]))}
    return {"ok": all(checks.values()), **checks, "sample": s.idx}


def dry_run(args) -> int:
    """Validate the dataset and encode `--dry-run` samples without a model:
    the real tokenizer and image processor when transformers and the base's
    files are present (CPU), else the text checks alone, reported as such."""
    samples, report = load_samples(Path(args.data), check_images=not args.no_image_check)
    rep: dict[str, Any] = {"schema": SCHEMA + "#dry-run", "data": report}
    try:
        tr, held, sinfo = split_samples(samples, args.holdout_splits)
        rep["bottom_tile"] = dataset_bottom_tile(Path(args.data), samples)
    except DataError as exc:
        rep["refused"] = str(exc)
        text = json.dumps(rep, indent=1, default=str)
        if args.report:
            Path(args.report).write_text(text, encoding="utf-8")
        print(text)
        return 1
    rep.update({"split": sinfo, "train_stats": data_stats(tr), "held_out_stats": data_stats(held)})
    n = min(args.dry_run, len(samples))
    tok = imgproc = None
    try:
        tok, imgproc = load_tokenizer_and_processor(args.base)
        rep["tokenizer"] = f"{type(tok).__name__} from {args.base}"
    except Exception as exc:                                       # noqa: BLE001
        rep["tokenizer"] = f"not available ({type(exc).__name__}: {str(exc)[:120]}): text checks only"
    enc_rep: list[dict] = []
    bad = Counter()
    for s in samples[:n]:
        row: dict[str, Any] = {"idx": s.idx, "run": s.run, "split": s.split,
                               "phrase": s.phrase, "target": s.target}
        if tok is not None:
            try:
                e = encode_sample(s, tok, imgproc) if imgproc is not None and \
                    not args.no_image_check else encode_text(tok, s.prompt, s.target)
                row.update({"n_prompt": e["n_prompt"], "n_answer": e["n_answer"],
                            "answer_text": e["answer_text"],
                            "pixel_values": (list(e["pixel_values"].shape)
                                             if "pixel_values" in e else None)})
            except (EncodeError, OSError, ImportError) as exc:
                bad[type(exc).__name__] += 1
                row["error"] = str(exc)[:200]
        enc_rep.append(row)
    rep["encoded"] = enc_rep
    rep["encode_failures"] = dict(bad)
    if tok is not None and imgproc is not None and samples and not args.no_image_check:
        try:
            rep["server_parity"] = server_parity(samples[0], tok, imgproc)
        except Exception as exc:                                   # noqa: BLE001
            rep["server_parity"] = {"ok": False, "why": f"{type(exc).__name__}: {exc}"}
    text = json.dumps(rep, indent=1, default=str)
    if args.report:
        Path(args.report).write_text(text, encoding="utf-8")
    print(text)
    ok = (bool(samples) and not bad
          and (rep.get("server_parity") or {"ok": True}).get("ok", False))
    return 0 if ok else 1


# --------------------------------------------------------------------------- #
def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Format-exact AerialVLA follow fine-tune")
    ap.add_argument("--data", required=True, help="the dataset directory (samples.jsonl) or file")
    ap.add_argument("--run", required=True, help="the run's name: <out-root>/<run>/epoch<N>")
    ap.add_argument("--epochs", type=int, default=4)
    ap.add_argument("--precision", choices=("bf16", "nf4"), default="bf16",
                    help="bf16 LoRA (as served), or nf4 QLoRA when bf16 does not fit 24 GB")
    ap.add_argument("--attn", choices=("sdpa", "eager"), default="sdpa")
    ap.add_argument("--base", default=DEFAULT_BASE)
    ap.add_argument("--adapter", default=DEFAULT_ADAPTER, help="the adapter training continues")
    ap.add_argument("--out-root", default=str(DEFAULT_OUT_ROOT))
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--lr", type=float, default=5e-5)
    ap.add_argument("--min-lr-frac", type=float, default=0.1)
    ap.add_argument("--warmup-frac", type=float, default=0.03)
    ap.add_argument("--weight-decay", type=float, default=0.0)
    ap.add_argument("--batch-size", type=int, default=4)
    ap.add_argument("--grad-accum", type=int, default=4)
    ap.add_argument("--max-grad-norm", type=float, default=1.0)
    ap.add_argument("--max-steps", type=int, default=0, help="stop after this many optimizer "
                    "steps (0 = all epochs); a checkpoint and an evaluation are written")
    ap.add_argument("--patience", type=int, default=2,
                    help="early stop after this many evaluations without a better "
                         "step_mae_norm (0 = never)")
    ap.add_argument("--min-delta", type=float, default=0.0)
    ap.add_argument("--holdout-splits", nargs="+", default=list(HOLDOUT_SPLITS))
    ap.add_argument("--eval-max", type=int, default=400,
                    help="held-out samples evaluated per epoch (a seeded subset; 0 = all)")
    ap.add_argument("--no-eval-before", action="store_true",
                    help="skip the parent adapter's evaluation before training (epoch 0)")
    ap.add_argument("--turn-oversample", type=int, default=1)
    ap.add_argument("--turn-bins", type=int, default=3)
    ap.add_argument("--max-new-tokens", type=int, default=12)
    ap.add_argument("--log-every", type=int, default=10)
    ap.add_argument("--seed", type=int, default=20261023)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--no-grad-ckpt", action="store_true")
    ap.add_argument("--dry-run", type=int, default=0, metavar="N",
                    help="validate the data and encode N samples, no model (CI)")
    ap.add_argument("--no-image-check", action="store_true",
                    help="dry run: do not open the images (text checks only)")
    ap.add_argument("--report", default=None, help="dry run: also write the report here")
    return ap


def check_args(ap: argparse.ArgumentParser, args) -> None:
    if not RUN_NAME.fullmatch(args.run):
        ap.error(f"--run {args.run!r}: letters, digits, '.', '_', '-' (it becomes the server's "
                 f"model id {ID_ROOT}/<run>/epoch<N>)")
    for k in ("epochs", "batch_size", "grad_accum", "log_every", "turn_oversample",
              "max_new_tokens"):
        if getattr(args, k) < 1:
            ap.error(f"--{k.replace('_', '-')} must be at least 1")
    for k in ("lr",):
        if not (getattr(args, k) > 0 and math.isfinite(getattr(args, k))):
            ap.error(f"--{k} must be positive")
    if not 0 <= args.min_lr_frac <= 1 or not 0 <= args.warmup_frac < 1:
        ap.error("--min-lr-frac in [0, 1], --warmup-frac in [0, 1)")
    if TRAIN_SPLIT in args.holdout_splits:
        ap.error("'train' cannot be a held-out split")


def main(argv: list[str] | None = None) -> int:
    ap = build_arg_parser()
    args = ap.parse_args(argv)
    check_args(ap, args)
    if args.dry_run > 0:
        return dry_run(args)
    return train(args)


if __name__ == "__main__":
    sys.exit(main())
