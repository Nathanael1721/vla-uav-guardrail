"""
AerialVLA behind a socket: VLA server protocol v2, one model process, frames
from the client.

    # serve (vla-real env; loads OpenVLA-7B + the AerialVLA LoRA from D:/models).
    # The configuration has NO default: name all three flags on every run.
    python demo/vla_server_v2.py serve --precision bf16 --attn sdpa --no-merge   # fidelity
    python demo/vla_server_v2.py serve --precision nf4 --attn sdpa --merge       # low VRAM
    python demo/vla_server.py --v2 serve ...                  # the same, via the old entry point

    # a scripted stand-in for client development (no torch, no GPU, never a VLA)
    python demo/vla_server_v2.py serve --fake-output "96 48 49</s>" "00 49 49 LAND</s>"

    # latency of one configuration on the GPU, in process (no socket)
    python demo/vla_server_v2.py bench --precision bf16 --attn sdpa --merge --out bench.json

    # round trip against a running server
    python demo/vla_server_v2.py ping --port 8790 --n 20

WHAT CHANGED FROM demo/vla_server.py (the legacy file protocol, kept as it was)
------------------------------------------------------------------------------
* The CLIENT sends the frames, the pose they were captured at and the compass
  phrase. The server never loads a scene (the legacy server built
  `World(client, scene)`, which LOADS a scene and resets the simulator clock
  under a running ArduPilot: integration discovery report, 2026-10-21) and
  never sees a target coordinate: a request carrying `target`, `goal`,
  `object_position`, `xy`, `lat`, `lon` or any key containing "target" or
  "goal" is refused by name (guardrail/aerialvla_codec.validate_request). The
  legacy command file carried `"target": [x, y]` and turned it into the
  phrase - the coordinate leak.
* The output is read by the strict reader (aerialvla_codec.parse_action) and
  returned as a STEP in upstream units (metres and radians per decision) with
  `parse_ok`; a malformed output is parse_ok=false, step=null, land=false and
  the client holds zero.
* Every response carries the model's identity, including a SHA-256 over the
  weight bytes (base shards + adapter), so a flight process that cannot see
  the weights can still record what flew (guardrail/manifest.pilot_record,
  reported_digest).

THE PROTOCOL (v2; the brief BRIEF-V1 is the authority, the codec its code)
-------------------------------------------------------------------------
TCP, 127.0.0.1:8790 by default, newline-delimited UTF-8 JSON, one request ->
one response, many requests per connection.

  {"v":2,"op":"hello"}     -> {"v":2,"ok":true,"model":{identity + serving facts
                                                         + "health"}}
  {"v":2,"op":"infer", "seq", "t_capture", "pose":{"x","y","up","yaw_rad"},
   "front_jpg_b64", "down_jpg_b64"|null, "phrase" (one of the 8 strings),
   "hint_source" goal|detector|none, "object", "csp_text"|null,
   "csp_mode" off|csp|neutral}
                           -> {"v":2,"seq","ok","error","raw_text","bins","step",
                               "land","parse_ok","latency_s","t_capture","pose",
                               "prompt_sha256","image_sha256","model"}
  {"v":2,"op":"shutdown"}  -> {"v":2,"ok":true}, then the server exits

An infer response always has every key, and so does EVERY ok=false answer,
whatever was sent - a line that is not JSON, a line longer than
MAX_LINE_BYTES, a request with no `op` or not an object, a refused hello: ok
false with `error` "<code>: <detail>" (codes in aerialvla_codec, ERR_*),
raw_text "", bins/step/latency/hashes null, land and parse_ok false, seq /
t_capture / pose echoed only when they were valid, and the model identity. A
client reading resp["seq"] never meets a KeyError (review of V1a, P2: those
four used to answer {v, ok, error} only). raw_text is always a string.

EVERY REQUEST THE SERVER READ IS ANSWERED, and the connection stays open: an
exception in the model is "inference failed", any other exception while
answering (a frame decoder, the host out of memory) is "internal error", a
line too long is answered before the connection closes (the rest of that
line cannot be found again). The JSONL log never costs an answer: a write
that fails is counted (counts["log_failures"], in hello's health) and warned
about once on stderr, and serving goes on. Until the review of V1a (P1) a log
on a full or locked disk raised after the model had answered, and the client
met EOF on every request. `latency_s` is the model's own time on the
server: tokenizer + image processor + generate + decode, from the moment the
mosaic and the prompt exist to the moment the text is decoded. The whole
server-side time (JPEG decode and mosaic too) is in the JSONL log as
server_s. `image_sha256` is over the mosaic's raw RGB bytes
(aerialvla_codec.mosaic_sha256); a missing down frame is filled black and
logged as down_missing (aerialvla_codec.build_mosaic), or refused with
--missing-down refuse.

HEALTH. hello never touches the model (it does not wait for its lock), so an
answered hello proves only that the process listens: a server whose
generate() hangs or fails on every call still answers it. hello's model
object therefore carries `health` (V2Server.health): the counters,
t_last_ok_infer, t_last_inference_error and, while generate() runs,
in_flight_since. Liveness is a RECENT OK INFER, not a hello (review of V1a,
P4).

THE MODEL OPTIONS - NO DEFAULT
------------------------------
`serve` and `bench` refuse to load a model until --precision, --attn and
--merge/--no-merge are all given, and the refusal prints the measured table
(check_model_args). Decided 2026-10-22 (review of V1a): the precision
changes what the model says. On the RTX 4090 probe
(docs/data/aerialvla_serving_4090.json, 48 inputs) bf16 reproduces the
upstream-precision reference - bf16/sdpa/unmerged gives the same output as
bf16/eager/unmerged on 96 %, the merged bf16 rows on 88-90 % - while every
nf4 row agrees on only 25-33 % (4-bit quantisation, not the attention kernel
or the merge, is what moves the outputs). The CLI used to default to
nf4/sdpa/merged, the lowest-VRAM row that met the 1.0 s gate, so a run that
named nothing flew the configuration furthest from upstream. That row is
kept as the measured LOW-VRAM OPTION (p50 517 ms, about 7.9 GB of process
VRAM); bf16/sdpa/unmerged is the FIDELITY OPTION (p50 486 ms, about 17 GB).
Which one flies is decided by measuring CityLife's own VRAM beside the model
(next wave). Every response's identity carries the configuration's measured
agreement (`agreement`, measured_agreement), so the record of a flight says
how far its configuration moves the outputs.

--precision nf4   bitsandbytes 4-bit NF4, compute bf16, the projector kept in
                  bf16 (LoRA's modules_to_save replaces it) - the loader every
                  AerialVLA script here has used (aerialvla_demo.py).
--precision bf16  unquantised, upstream's own precision
                  (aerialvla_wrapper_ui.py:34-39). ~15 GB of VRAM.
--attn eager|sdpa attention implementation of the Llama backbone; Prismatic
                  delegates (modeling_prismatic.py:208-210, 249).
--merge           (or --no-merge, to keep the adapter separate)
                  PeftModel.merge_and_unload() in memory after loading: no
                  per-token LoRA matmuls. On nf4 the merge dequantises, adds
                  and re-quantises each layer, so the weights that run are not
                  bit-identical to base+adapter; the identity says merged=true
                  and the bench compares outputs with the bf16 reference.
--save-merged     (bf16 + --merge only, refused before the load otherwise)
                  write the merged model to D:/models/aerialvla-merged
                  (~15 GB) with a provenance file.
upstream calls resize_token_embeddings(len(tokenizer)) before attaching the
adapter; that only drops the 63 padding rows of the 32064-row head (the
adapter touches neither embeddings nor head) and crashes on a quantised head,
so neither precision does it here (as aerialvla_demo.py).

weights_sha256 is guardrail.manifest.weights_digest over the base's
model-*.safetensors shards and the adapter directory - the same list a flight
process passes to pilot_record(kind="vla", weight_paths=...) when it can see
the files, so the two digests agree. It is computed once and cached in a
sidecar JSON keyed by each file's path, size, mtime and file index
(--digest-sidecar, default ~/.cache/guardrail/vla_server_weights.json);
--rehash recomputes. The per-file digests themselves are cached by the
manifest module under its stronger version key.

Every request and response is appended to a JSONL log (--log), with the base64
frames replaced by their byte length and SHA-256 and the full prompt added.

Not done here (and why): no detector role, no ROS node, no status op - the
brief's protocol has three ops and the client side is guardrail/vla_link.py.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import socket
import socketserver
import statistics
import sys
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import aerialvla_codec as C                           # noqa: E402

DEFAULT_BASE = "D:/models/openvla-7b"
DEFAULT_ADAPTER = "D:/models/aerialvla-lora/aero_vla"
DEFAULT_MERGED_OUT = "D:/models/aerialvla-merged"
# The model stops at EOS after 9 tokens ("96 48 49</s>") or 11 with LAND
# ("00 49 49 LAND</s>"): tokenizer probe 2026-10-21. 12 leaves one spare and
# bounds a rambling output; a truncated one fails the strict reader (no EOS).
DEFAULT_MAX_NEW_TOKENS = 12
PRECISIONS = ("nf4", "bf16")
ATTNS = ("eager", "sdpa")
# NO DEFAULT MODEL CONFIGURATION, by decision (2026-10-22, review of V1a).
# The precision changes what the model says: on the RTX 4090 probe (48
# inputs) bf16 reproduces the upstream-precision reference - bf16/sdpa/
# unmerged agrees with bf16/eager/unmerged on 96 % - while every nf4 row
# agrees on only 25-33 %. Until then the CLI defaulted to nf4/sdpa/merged,
# the lowest-VRAM row that met the gate, so a run that named no precision
# silently flew the configuration furthest from upstream's outputs. Now
# `serve` and `bench` refuse to load the model until --precision, --attn
# and --merge/--no-merge are all given (check_model_args), and every
# response's identity carries that configuration's measured agreement
# (measured_agreement). nf4/sdpa/merged stays available as the measured
# low-VRAM option; which configuration flies is decided by measuring
# CityLife's own VRAM beside the model (next wave).
SERVING_JSON = ROOT / "docs" / "data" / "aerialvla_serving_4090.json"
# The configuration every other one is compared with: upstream's precision
# (aerialvla_wrapper_ui.py:34-39) with the adapter kept separate.
# tools/vla_backend_table.SERVING_REFERENCE is the same.
REFERENCE_CONFIG = {"precision": "bf16", "attn": "eager", "merged": False}
SIDECAR_DEFAULT = Path.home() / ".cache" / "guardrail" / "vla_server_weights.json"
BENCH_SCHEMA = "vlaguard.aerialvla_serving_bench/1"


# --------------------------------------------------------------------------- #
# Identity: revisions, the weight list, the digest
# --------------------------------------------------------------------------- #
def local_hf_revision(model_dir: str | Path) -> str | None:
    """The HuggingFace commit a `huggingface-cli download --local-dir` copy was
    taken at: the first line of every .cache/huggingface/download/*.metadata
    file that belongs to this directory (found in the directory itself or in
    a parent the download was made into). None when there is no metadata, or
    when the files disagree (a mixed copy pins nothing)."""
    d = Path(model_dir).resolve()
    for root in (d, *d.parents):
        meta = root / ".cache" / "huggingface" / "download"
        if not meta.is_dir():
            continue
        rel = d.relative_to(root)
        scope = meta / rel if str(rel) != "." else meta
        revs = set()
        for f in scope.rglob("*.metadata") if scope.is_dir() else []:
            try:
                first = f.read_text(encoding="utf-8").splitlines()[0].strip()
            except (OSError, IndexError):
                continue
            if len(first) == 40:
                revs.add(first)
        return revs.pop() if len(revs) == 1 else None
    return None


def base_weight_files(base_dir: str | Path) -> list[Path]:
    """The base checkpoint's weight shards: those named by
    model.safetensors.index.json when present, else model*.safetensors."""
    b = Path(base_dir)
    idx = b / "model.safetensors.index.json"
    if idx.is_file():
        names = sorted(set(json.loads(idx.read_text(encoding="utf-8"))["weight_map"].values()))
        return [b / n for n in names]
    return sorted(b.glob("model*.safetensors"))


def weight_paths(base_dir: str | Path, adapter_dir: str | Path | None) -> list[Path]:
    """What weights_sha256 covers: the base shards, then the adapter directory
    (adapter_config.json - r, alpha and the target modules change what runs -
    and adapter_model.safetensors)."""
    out = base_weight_files(base_dir)
    if adapter_dir:
        out.append(Path(adapter_dir))
    return out


def _stat_key(paths: list[Path]) -> str:
    rows = []
    for p in paths:
        files = sorted(f for f in p.rglob("*") if f.is_file()) if p.is_dir() else [p]
        for f in files:
            st = f.stat()
            rows.append([f.resolve().as_posix(), st.st_size, st.st_mtime_ns,
                         int(getattr(st, "st_ino", 0) or 0)])
    return hashlib.sha256(json.dumps(rows).encode()).hexdigest()


def weights_sha256(paths: list[Path], sidecar: Path = SIDECAR_DEFAULT,
                   rehash: bool = False,
                   digest_fn: Callable[[list], str | None] | None = None
                   ) -> tuple[str | None, dict]:
    """(digest, info). The digest is guardrail.manifest.weights_digest(paths):
    one SHA-256 over every weight file's own SHA-256 - None when a named file
    is missing or unreadable, never a digest of what was left.

    Cached in `sidecar` under a key built from every file's resolved path,
    size, mtime_ns and file index (stat only, no reads), so a restart costs
    a few stats instead of hashing 15 GB. That key is weaker than the manifest
    module's per-file version key (it does not sample the files' bytes): a
    checkpoint rewritten in place with its size and mtime preserved would be
    served the old digest. --rehash forces a fresh computation; the per-file
    hashes under it still come from the manifest's stronger cache."""
    if digest_fn is None:
        from guardrail.manifest import weights_digest as digest_fn
    key = _stat_key(paths)
    cache: dict = {}
    if sidecar.is_file():
        try:
            cache = json.loads(sidecar.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            cache = {}
    hit = cache.get(key) if not rehash else None
    if isinstance(hit, dict) and isinstance(hit.get("digest"), str) and len(hit["digest"]) == 64:
        return hit["digest"], {"source": "sidecar", "sidecar": str(sidecar), "key": key,
                               "computed_utc": hit.get("computed_utc")}
    t0 = time.perf_counter()
    digest = digest_fn(list(paths))
    info = {"source": "computed", "sidecar": str(sidecar), "key": key,
            "seconds": round(time.perf_counter() - t0, 1),
            "computed_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}
    if digest is not None:
        cache[key] = {"digest": digest, "computed_utc": info["computed_utc"],
                      "files": [p.as_posix() for p in paths],
                      "by": "guardrail.manifest.weights_digest"}
        try:
            sidecar.parent.mkdir(parents=True, exist_ok=True)
            tmp = sidecar.with_suffix(".tmp")
            tmp.write_text(json.dumps(cache, indent=1), encoding="utf-8")
            os.replace(tmp, sidecar)
        except OSError:
            info["sidecar_write"] = "failed"
    return digest, info


def model_identity(base: str | Path, adapter: str | Path | None, precision: str, attn: str,
                   merged: bool, digest: str | None,
                   agreement: dict | None = None) -> dict[str, Any]:
    """The `model` object every response carries (brief: id, base,
    weights_sha256, precision, attn, merged), plus `agreement`
    (measured_agreement; None when not given). Clients compare the brief's
    six fields and allow the rest (guardrail/vla_link.py check_identity)."""
    def rev8(p):
        r = local_hf_revision(p) if p and Path(p).exists() else None
        return r[:8] if r else "unknown"
    a = Path(adapter) if adapter else None
    aid = f"{a.parent.name}/{a.name}@{rev8(a)}" if a else "none"
    return {"id": aid, "base": f"{Path(base).name}@{rev8(base)}",
            "weights_sha256": digest, "precision": precision, "attn": attn,
            "merged": bool(merged), "agreement": agreement}


def _cfg_name(precision: str, attn: str, merged: bool) -> str:
    return f"{precision}/{attn}/{'merged' if merged else 'unmerged'}"


def _rel(p: Path) -> str:
    try:
        return Path(p).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return Path(p).as_posix()


def load_serving(path: Path = SERVING_JSON) -> dict | None:
    """The RTX 4090 serving measurement (tools/vla_backend_table.py serving),
    or None when it is absent or unreadable."""
    try:
        d = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return d if isinstance(d, dict) and isinstance(d.get("configs"), list) else None


def measured_agreement(precision: str, attn: str, merged: bool, weights_sha256: str | None,
                       path: Path = SERVING_JSON) -> dict[str, Any]:
    """How often this configuration's output equals the reference's
    (REFERENCE_CONFIG, bf16/eager/unmerged) on the 48-input probe of the RTX
    4090 measurement: exactly (parse_ok, bins, LAND) and closely (same
    parse_ok and LAND, every bin within 3). Carried in the identity of every
    response, so the record of what flew says how far its configuration
    moves AerialVLA's outputs, not only which weights ran.

    measured=False, with `why`, when the measurement file is absent, was made
    on other weights (its weights_sha256 differs: the probe says nothing
    about them), or has no row for this configuration."""
    ref = (REFERENCE_CONFIG["precision"], REFERENCE_CONFIG["attn"], REFERENCE_CONFIG["merged"])
    out: dict[str, Any] = {"reference": _cfg_name(*ref), "measured": False, "probe_n": None, "probe_exact": None,
                           "probe_close": None, "close_means": "same parse_ok and LAND, "
                                                               "every bin within 3",
                           "source": _rel(path), "measured_utc": None, "note": None, "why": None}
    s = load_serving(path)
    if s is None:
        out["why"] = f"no serving measurement at {_rel(path)}"
        return out
    if s.get("weights_sha256") != weights_sha256:
        out["why"] = (f"the measurement ran weights_sha256 {str(s.get('weights_sha256'))[:16]}..., "
                      f"this model is {str(weights_sha256)[:16]}...: other weights")
        return out
    row = next((c for c in s["configs"] if (c.get("precision"), c.get("attn"),
                                             bool(c.get("merged"))) ==
                (precision, attn, bool(merged))), None)
    ap = (row or {}).get("agreement_probe") or {}
    if not ap.get("n"):
        out["why"] = f"no probe row for {_cfg_name(precision, attn, merged)} in {_rel(path)}"
        return out
    out.update({"measured": True, "probe_n": ap["n"], "probe_exact": ap["exact"],
                "probe_close": ap["close"], "measured_utc": row.get("bench_created_utc")})
    if (precision, attn, bool(merged)) == ref:
        out["note"] = "this is the reference configuration (agreement 1.0 by definition)"
    return out


def serving_identity(base: str | Path, adapter: str | Path | None, precision: str, attn: str,
                     merged: bool, digest: str | None,
                     serving_json: Path = SERVING_JSON) -> dict[str, Any]:
    """model_identity with the configuration's measured agreement: what a
    real server announces (build_real)."""
    return model_identity(base, adapter, precision, attn, merged, digest,
                          agreement=measured_agreement(precision, attn, merged, digest,
                                                       serving_json))


def configuration_table(path: Path = SERVING_JSON) -> str:
    """The measured configurations as lines a person choosing one can read:
    p50, VRAM and agreement with the reference, the low-VRAM and fidelity
    options named."""
    s = load_serving(path)
    if s is None:
        return f"  (no serving measurement at {_rel(path)})"
    lines = []
    for c in sorted(s["configs"], key=lambda c: (c["precision"], c["attn"], c["merged"])):
        name = _cfg_name(c["precision"], c["attn"], c["merged"])
        ap = c.get("agreement_probe") or {}
        agree = ("the reference" if name == _cfg_name(REFERENCE_CONFIG["precision"],
                                                       REFERENCE_CONFIG["attn"],
                                                       REFERENCE_CONFIG["merged"])
                 else (f"agrees with the reference on {ap['exact']:.0%} of {ap['n']} probe "
                       f"inputs" if ap.get("n") else "agreement not measured"))
        vram, alloc = c.get("vram_process_gb"), c.get("vram_inference_peak_gb")
        vram = (f"{vram:.1f} GB process" if vram is not None else
                f"{alloc:.1f} GB torch-allocated" if alloc is not None else "VRAM n/a")
        tag = (" <- the measured low-VRAM option" if name == s.get("low_vram") else
               " <- the closest to the reference" if name == s.get("fidelity") else "")
        flags = (f"--precision {c['precision']} --attn {c['attn']} "
                 f"--{'merge' if c['merged'] else 'no-merge'}")
        lines.append(f"  {name:<20} p50 {c['p50_ms']:>4.0f} ms  {vram:<22} {agree}{tag}\n"
                     f"  {'':<20} {flags}")
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# The model (vla-real: torch, transformers, peft, bitsandbytes)
# --------------------------------------------------------------------------- #
class AerialVLAModel:
    """OpenVLA-7B + the AerialVLA LoRA, loaded once. `generate(mosaic, prompt)`
    is the whole model step; nothing here decides what the text means.

    precision, attn and merge are keyword-only with NO default, like the
    CLI (see the note at PRECISIONS): until the review of V1a (P11) this
    constructor defaulted to nf4/eager/unmerged while the CLI said
    nf4/sdpa/merged, so a programmatic caller got a third configuration."""

    def __init__(self, base: str = DEFAULT_BASE, adapter: str | None = DEFAULT_ADAPTER, *,
                 precision: str, attn: str, merge: bool,
                 max_new_tokens: int = DEFAULT_MAX_NEW_TOKENS, device: str = "cuda:0"):
        if precision not in PRECISIONS:
            raise ValueError(f"precision {precision!r} is not one of {PRECISIONS}")
        if attn not in ATTNS:
            raise ValueError(f"attn {attn!r} is not one of {ATTNS}")
        import torch
        from transformers import (AutoImageProcessor, AutoModelForVision2Seq,
                                  AutoTokenizer, BitsAndBytesConfig)
        self.torch = torch
        self.base, self.adapter = str(base), (str(adapter) if adapter else None)
        self.precision, self.attn, self.merged = precision, attn, False
        self.max_new_tokens = int(max_new_tokens)
        self.device = device
        self.timing: dict[str, float] = {}
        t0 = time.perf_counter()
        self.tok = AutoTokenizer.from_pretrained(self.base, trust_remote_code=True)
        self.imgproc = AutoImageProcessor.from_pretrained(self.base, trust_remote_code=True)
        kw: dict[str, Any] = dict(attn_implementation=attn, torch_dtype=torch.bfloat16,
                                  device_map={"": device}, low_cpu_mem_usage=True,
                                  trust_remote_code=True)
        if precision == "nf4":
            kw["quantization_config"] = BitsAndBytesConfig(
                load_in_4bit=True, bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_quant_type="nf4",
                # LoRA's modules_to_save replaces the projector wholesale: keep
                # it unquantised or PEFT cannot wrap it (aerialvla_demo.py).
                llm_int8_skip_modules=["projector"])
        model = AutoModelForVision2Seq.from_pretrained(self.base, **kw)
        self.timing["load_base_s"] = time.perf_counter() - t0
        if self.adapter:
            from peft import PeftModel
            t1 = time.perf_counter()
            model = PeftModel.from_pretrained(model, self.adapter)
            self.timing["load_adapter_s"] = time.perf_counter() - t1
            if merge:
                t2 = time.perf_counter()
                model = model.merge_and_unload()
                self.merged = True
                self.timing["merge_s"] = time.perf_counter() - t2
        self.model = model.eval()
        self.timing["load_total_s"] = time.perf_counter() - t0
        self.eos_id = self.tok.eos_token_id
        self.pad_id = self.tok.pad_token_id

    def generate(self, mosaic, prompt: str) -> dict[str, Any]:
        torch = self.torch
        t0 = time.perf_counter()
        enc = self.tok(prompt, return_tensors="pt")
        pv = self.imgproc(images=mosaic, return_tensors="pt")["pixel_values"]
        n_prompt = int(enc["input_ids"].shape[1])
        t1 = time.perf_counter()
        with torch.inference_mode():
            out = self.model.generate(
                input_ids=enc["input_ids"].to(self.device),
                attention_mask=enc["attention_mask"].to(self.device),
                pixel_values=pv.to(self.device, dtype=torch.bfloat16),
                max_new_tokens=self.max_new_tokens, do_sample=False,
                eos_token_id=[self.eos_id], pad_token_id=self.pad_id)
        torch.cuda.synchronize()
        t2 = time.perf_counter()
        gen = out[0, n_prompt:]
        raw = self.tok.decode(gen, skip_special_tokens=False)
        t3 = time.perf_counter()
        return {"raw_text": raw, "new_tokens": int(gen.shape[0]), "prompt_tokens": n_prompt,
                "pre_s": t1 - t0, "gen_s": t2 - t1, "model_s": t3 - t0}

    def save_merged(self, out_dir: str | Path, provenance: dict) -> Path:
        """Write the merged bf16 model (safetensors), its tokenizer and image
        processor, and a provenance file naming the sources and their digest.
        Refused for nf4: a re-quantised merge is not the model upstream ran."""
        if not self.merged or self.precision != "bf16":
            raise ValueError("--save-merged needs --precision bf16 --merge")
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        self.model.save_pretrained(out, safe_serialization=True)
        self.tok.save_pretrained(out)
        self.imgproc.save_pretrained(out)
        (out / "aerialvla_merged_provenance.json").write_text(
            json.dumps(provenance, indent=1), encoding="utf-8")
        return out


class ScriptedModel:
    """A stand-in that returns scripted texts in turn: for building and testing
    a client without a GPU. Its identity says "fake:scripted" with no weights
    digest, so nothing it answers can be mistaken for a VLA."""

    precision = "fake"
    attn = "none"
    merged = False

    def __init__(self, outputs: list[str], delay_s: float = 0.0):
        if not outputs:
            raise ValueError("ScriptedModel needs at least one scripted output")
        self.outputs = list(outputs)
        self.delay_s = float(delay_s)
        self.calls: list[tuple[Any, str]] = []
        self.max_new_tokens = DEFAULT_MAX_NEW_TOKENS

    def generate(self, mosaic, prompt: str) -> dict[str, Any]:
        t0 = time.perf_counter()
        if self.delay_s:
            time.sleep(self.delay_s)
        text = self.outputs[len(self.calls) % len(self.outputs)]
        self.calls.append((mosaic, prompt))
        dt = time.perf_counter() - t0
        return {"raw_text": text, "new_tokens": None, "prompt_tokens": None,
                "pre_s": 0.0, "gen_s": dt, "model_s": dt}


FAKE_IDENTITY = {"id": "fake:scripted", "base": "none", "weights_sha256": None,
                 "precision": "fake", "attn": "none", "merged": False, "agreement": None}


# --------------------------------------------------------------------------- #
# The protocol core (no socket): one request in, one response out
# --------------------------------------------------------------------------- #
def _elide(req: Any) -> Any:
    """A request as logged: base64 frames replaced by their length and the
    SHA-256 of their text, so the log stays small and still pins the bytes."""
    if not isinstance(req, dict):
        return req
    out = {}
    for k, v in req.items():
        if k in ("front_jpg_b64", "down_jpg_b64") and isinstance(v, str):
            out[k] = {"b64_chars": len(v), "sha256": hashlib.sha256(v.encode()).hexdigest()}
        else:
            out[k] = v
    return out


def _decode_frames(req: dict):
    """(front, down|None) PIL images from the request's base64 JPEGs."""
    front = C.decode_image_b64(req["front_jpg_b64"], "front_jpg_b64")
    down = (None if req["down_jpg_b64"] is None
            else C.decode_image_b64(req["down_jpg_b64"], "down_jpg_b64"))
    return front, down


class V2Server:
    """Protocol v2 without the transport. `model` has generate(mosaic, prompt)
    -> {"raw_text", "model_s", ...}; `identity` is the `model` object of every
    response; `decode_frames(req) -> (front, down)` and `mosaic_fn(front, down,
    missing_down) -> (mosaic, info)` default to the codec's Pillow path and are
    parameters so the protocol can be tested where Pillow is absent."""

    def __init__(self, model, identity: dict, log_path: str | Path | None = None,
                 missing_down: str = "black", hello_extra: dict | None = None,
                 decode_frames: Callable = _decode_frames,
                 mosaic_fn: Callable = C.build_mosaic,
                 hash_fn: Callable = C.mosaic_sha256,
                 clock: Callable[[], float] = time.perf_counter):
        if missing_down not in C.MISSING_DOWN_POLICIES:
            raise ValueError(f"missing_down {missing_down!r}")
        self.model = model
        self.identity = dict(identity)
        self.log_path = Path(log_path) if log_path else None
        self.missing_down = missing_down
        self.hello_extra = dict(hello_extra or {})
        self.decode_frames = decode_frames
        self.mosaic_fn = mosaic_fn
        self.hash_fn = hash_fn
        self.clock = clock
        self._model_lock = threading.Lock()
        self._log_lock = threading.Lock()
        self._counts_lock = threading.Lock()
        self._log_warned = False
        # refused: a request the protocol or the codec turned down (the
        # client's fault). inference_error: model.generate() raised or
        # returned no timing (the model's). internal_error: anything else
        # raised while answering (the server's). They were one counter
        # until the review of V1a (P3), so the stop summary could not tell a
        # dying model from bad requests.
        self.counts = {"requests": 0, "infer": 0, "refused": 0, "parse_fail": 0, "land": 0,
                       "inference_error": 0, "internal_error": 0, "log_failures": 0}
        # Live facts for hello (health): wall-clock times, None until it happens.
        self._t_start = time.time()
        self._t_last_ok_infer: float | None = None
        self._t_last_inference_error: float | None = None
        self._in_flight_since: float | None = None

    def _count(self, key: str, n: int = 1) -> None:
        # ThreadingTCPServer runs one thread per connection; `+=` on a dict
        # item is a read and a write.
        with self._counts_lock:
            self.counts[key] += n

    def health(self) -> dict[str, Any]:
        """What hello reports beside the identity (review of V1a, P4). hello
        never touches the model and never waits for its lock, so an answered
        hello proves only that the process listens: a server whose
        generate() hangs (holding the lock) or fails on every call still
        answers it. LIVENESS IS A RECENT OK INFER: t_last_ok_infer (wall
        clock, time.time()) and s_since_last_ok_infer; in_flight_since is
        set while generate() runs, so a hang shows as a growing
        in_flight_s."""
        now = time.time()
        with self._counts_lock:
            counts = dict(self.counts)
        last, err, infl = self._t_last_ok_infer, self._t_last_inference_error, \
            self._in_flight_since
        return {"counts": counts, "t_now": now, "uptime_s": round(now - self._t_start, 3),
                "t_last_ok_infer": last,
                "s_since_last_ok_infer": None if last is None else round(now - last, 3),
                "t_last_inference_error": err,
                "in_flight_since": infl,
                "in_flight_s": None if infl is None else round(now - infl, 3),
                "liveness": "a recent ok infer (t_last_ok_infer), not an answered hello"}

    # -- responses ------------------------------------------------------- #
    def _infer_error(self, req: Any, error: str) -> dict:
        """The ok=false answer, with every infer key, for ANY refused or
        failed request (an infer, a hello, a line that was not JSON): seq,
        t_capture and pose are echoed only when they were valid."""
        seq = req.get("seq") if isinstance(req, dict) else None
        seq = seq if isinstance(seq, int) and not isinstance(seq, bool) else None
        tcap = req.get("t_capture") if isinstance(req, dict) else None
        tcap = tcap if C._finite_number(tcap) else None
        pose = req.get("pose") if isinstance(req, dict) else None
        pose_ok = (isinstance(pose, dict) and set(pose) == set(C.POSE_FIELDS)
                   and all(C._finite_number(pose[k]) for k in C.POSE_FIELDS))
        return {"v": C.PROTOCOL_VERSION, "seq": seq, "ok": False, "error": error,
                "raw_text": "", "bins": None, "step": None, "land": False, "parse_ok": False,
                "latency_s": None, "t_capture": tcap, "pose": pose if pose_ok else None,
                "prompt_sha256": None, "image_sha256": None, "model": dict(self.identity)}

    def _log(self, t_rx: float, req: Any, resp: dict, server_s: float,
             extra: dict | None = None) -> None:
        """Append one request/response record. NEVER raises: the answer has
        been computed and the client is owed it whatever the disk does. A
        failure (a full D:, a file OneDrive holds locked, a record that does
        not serialise) is counted in counts["log_failures"] and warned about
        once on stderr."""
        if self.log_path is None:
            return
        try:
            rec = {"t_rx": t_rx, "request": _elide(req), "response": resp,
                   "server_s": server_s, **(extra or {})}
            line = json.dumps(rec, ensure_ascii=False, default=repr) + "\n"
            with self._log_lock:
                self.log_path.parent.mkdir(parents=True, exist_ok=True)
                with self.log_path.open("a", encoding="utf-8") as fh:
                    fh.write(line)
        except Exception as exc:                                  # noqa: BLE001
            self._count("log_failures")
            self._warn_log(exc)

    def _warn_log(self, exc: BaseException) -> None:
        if not self._log_warned:
            self._log_warned = True
            print(f"[vla-server] WARNING: cannot write the request log {self.log_path} "
                  f"({type(exc).__name__}: {exc}); answering every request anyway and "
                  f"counting log_failures (hello reports them)", file=sys.stderr, flush=True)

    def probe_log(self) -> bool:
        """Open the log for append once, before serving, so an unwritable
        location is reported when the server starts rather than first in the
        middle of a flight: the default is under demo/out on D:, which fills
        up, inside OneDrive, which can hold a file locked (review of V1a,
        P1). Never raises; True when there is no log or it opened."""
        if self.log_path is None:
            return True
        try:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open("a", encoding="utf-8"):
                pass
            return True
        except Exception as exc:                                  # noqa: BLE001
            self._warn_log(exc)
            return False

    # -- entry points ---------------------------------------------------- #
    def handle_line(self, line: bytes | str) -> tuple[dict, bool]:
        """One request line -> (response, stop_after). Never raises
        (Exception): a line that is not JSON - including one nested so deep
        that json.loads raises RecursionError, which is not a ValueError - is
        a bad-json answer, and the rest goes through handle_request."""
        t_rx = time.time()
        t0 = self.clock()
        try:
            text = line.decode("utf-8") if isinstance(line, bytes) else line
            req = json.loads(text)
        except (UnicodeDecodeError, ValueError, RecursionError) as exc:
            self._count("requests")
            self._count("refused")
            resp = self._infer_error(None, f"{C.ERR_BAD_JSON}: {type(exc).__name__}: {exc}")
            self._log(t_rx, None, resp, self.clock() - t0)
            return resp, False
        resp, stop, _ = self.handle_request(req, t_rx=t_rx, t0=t0)
        return resp, stop

    def handle_request(self, req: Any, t_rx: float | None = None,
                       t0: float | None = None) -> tuple[dict, bool, dict]:
        """One decoded request -> (response, stop_after, log extras), and its
        log record. Never raises (Exception): whatever `handle` raises - a
        frame decoder's own exception, MemoryError, a bug - becomes an
        ok=false "internal error" answer with every infer key, so the
        connection carrying it stays usable. The transport and the bench
        both come through here, so both write the log."""
        t_rx = time.time() if t_rx is None else t_rx
        t0 = self.clock() if t0 is None else t0
        try:
            resp, stop, extra = self.handle(req)
        except Exception as exc:                                  # noqa: BLE001
            self._count("internal_error")
            resp, stop = self._infer_error(
                req, f"{C.ERR_INTERNAL}: {type(exc).__name__}: {exc}"), False
            extra = {"internal_error": traceback.format_exc(limit=12)}
        self._log(t_rx, req, resp, self.clock() - t0, extra)
        return resp, stop, extra

    def refuse_line_too_long(self) -> dict:
        """The answer to a line longer than MAX_LINE_BYTES (the transport
        closes the connection after it: the rest of the line cannot be found
        again). Counted and logged like any refusal."""
        self._count("requests")
        self._count("refused")
        resp = self._infer_error(None, f"{C.ERR_LINE_TOO_LONG}: more than "
                                       f"{C.MAX_LINE_BYTES} bytes")
        self._log(time.time(), None, resp, 0.0)
        return resp

    def handle(self, req: Any) -> tuple[dict, bool, dict]:
        """(response, stop_after, log extras) for one decoded request. May
        raise on a server fault; handle_request is the entry point that
        answers those too."""
        self._count("requests")
        try:
            C.validate_request(req)
        except C.ProtocolError as exc:
            self._count("refused")
            return self._infer_error(req, str(exc)), False, {}
        op = req["op"]
        if op == "hello":
            return ({"v": C.PROTOCOL_VERSION, "ok": True,
                     "model": {**self.identity, **self.hello_extra, "health": self.health()}},
                    False, {})
        if op == "shutdown":
            return {"v": C.PROTOCOL_VERSION, "ok": True}, True, {}
        return self.infer(req)

    @staticmethod
    def _model_result(g: Any) -> tuple[Any, float]:
        """(raw_text as returned, model_s) from generate()'s dict, or raise:
        a result without a finite model_s has no latency to report (the
        brief types latency_s a float), so it is an inference failure, not a
        KeyError on the way out (review of V1a, P1)."""
        if not isinstance(g, dict):
            raise TypeError(f"generate() returned {type(g).__name__}, not a dict")
        if not C._finite_number(g.get("model_s")):
            raise ValueError(f"generate() returned no finite model_s ({g.get('model_s')!r})")
        return g.get("raw_text"), float(g["model_s"])

    def infer(self, req: dict) -> tuple[dict, bool, dict]:
        self._count("infer")
        try:
            front, down = self.decode_frames(req)
            mosaic, minfo = self.mosaic_fn(front, down, missing_down=self.missing_down)
            image_sha = self.hash_fn(mosaic)
            prompt = C.build_prompt(req["phrase"], req["object"], req["csp_text"],
                                    req["csp_mode"])
        except C.CodecError as exc:
            self._count("refused")
            return self._infer_error(req, f"{exc.code}: {exc.detail}"), False, {}
        try:
            with self._model_lock:
                self._in_flight_since = time.time()
                try:
                    g = self.model.generate(mosaic, prompt)
                finally:
                    self._in_flight_since = None
            raw, model_s = self._model_result(g)
        except Exception as exc:                                  # noqa: BLE001
            # A CUDA OOM or a driver error must answer the client, not drop the
            # connection: the client then holds zero and counts the failure.
            self._t_last_inference_error = time.time()
            self._count("inference_error")
            return (self._infer_error(req, f"{C.ERR_INFERENCE}: {type(exc).__name__}: {exc}"),
                    False, {"prompt": prompt, "mosaic": minfo})
        # The reader judges what the model returned (a non-string is "not
        # text", parse_ok false); the client is sent a string either way.
        self._t_last_ok_infer = time.time()
        p = C.parse_action(raw, require_eos=True)
        raw_text = raw if isinstance(raw, str) else ("" if raw is None else str(raw))
        self._count("parse_fail", 0 if p.parse_ok else 1)
        self._count("land", 1 if p.land else 0)
        resp = {"v": C.PROTOCOL_VERSION, "seq": req["seq"], "ok": True, "error": None,
                "raw_text": raw_text,
                "bins": list(p.bins) if p.bins is not None else None,
                "step": p.step.to_dict() if p.step is not None else None,
                "land": p.land, "parse_ok": p.parse_ok,
                "latency_s": round(model_s, 6),
                "t_capture": req["t_capture"], "pose": req["pose"],
                "prompt_sha256": C.prompt_sha256(prompt), "image_sha256": image_sha,
                "model": dict(self.identity)}
        extra = {"prompt": prompt, "mosaic": minfo, "parse_reason": p.reason,
                 "timing": {k: g.get(k) for k in ("pre_s", "gen_s", "model_s")},
                 "new_tokens": g.get("new_tokens"), "prompt_tokens": g.get("prompt_tokens")}
        return resp, False, extra


# --------------------------------------------------------------------------- #
# Transport
# --------------------------------------------------------------------------- #
class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        core: V2Server = self.server.core                         # type: ignore[attr-defined]
        try:
            while True:
                line = self.rfile.readline(C.MAX_LINE_BYTES + 1)
                if not line:
                    return
                if len(line) > C.MAX_LINE_BYTES and not line.endswith(b"\n"):
                    # The rest of the line cannot be found again: answer and close.
                    self._send(core.refuse_line_too_long())
                    return
                if not line.strip():
                    continue
                # handle_line answers every line it is given (it never raises
                # an Exception), so one bad request cannot cost the
                # connection the next one.
                resp, stop = core.handle_line(line)
                self._send(resp)
                if stop:
                    threading.Thread(target=self.server.shutdown, daemon=True).start()
                    return
        except (ConnectionError, OSError):
            return              # the client went away mid-read or mid-write

    def _send(self, resp: dict) -> None:
        self.wfile.write((json.dumps(resp, ensure_ascii=False) + "\n").encode("utf-8"))
        self.wfile.flush()


class V2TCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = False      # a second server on the same port must fail loudly
    daemon_threads = True

    def __init__(self, addr, core: V2Server):
        super().__init__(addr, _Handler)
        self.core = core


def make_tcp_server(core: V2Server, host: str = C.DEFAULT_HOST,
                    port: int = C.DEFAULT_PORT) -> V2TCPServer:
    return V2TCPServer((host, port), core)


class V2Client:
    """A minimal blocking client (bench `ping`, tests). The flight-side client
    with pipelining and the step tracker is guardrail/vla_link.py."""

    def __init__(self, host: str = C.DEFAULT_HOST, port: int = C.DEFAULT_PORT,
                 timeout: float = 60.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.rfile = self.sock.makefile("rb")

    def call(self, req: dict | str) -> dict:
        line = req if isinstance(req, str) else json.dumps(req)
        self.sock.sendall(line.encode("utf-8") + b"\n")
        got = self.rfile.readline()
        if not got:
            raise ConnectionError("server closed the connection")
        return json.loads(got)

    def close(self) -> None:
        try:
            self.rfile.close()
        finally:
            self.sock.close()


# --------------------------------------------------------------------------- #
# Building a real server, and the benchmark
# --------------------------------------------------------------------------- #
def library_versions() -> dict[str, Any]:
    out: dict[str, Any] = {"python": sys.version.split()[0]}
    for name in ("torch", "transformers", "peft", "bitsandbytes", "accelerate", "PIL"):
        try:
            mod = __import__(name)
            out[name] = getattr(mod, "__version__", None)
        except Exception:                                          # noqa: BLE001
            out[name] = None
    try:
        import torch
        out["cuda"] = torch.version.cuda
        out["gpu"] = torch.cuda.get_device_name(0) if torch.cuda.is_available() else None
    except Exception:                                              # noqa: BLE001
        pass
    return out


def nvidia_smi_used_mib() -> int | None:
    import subprocess
    try:
        r = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=10)
        return int(r.stdout.strip().splitlines()[0])
    except Exception:                                              # noqa: BLE001
        return None


def build_real(args) -> tuple[AerialVLAModel, dict, dict]:
    """Load the model and compute its identity (digest first: it is CPU work
    and a refusal to start is cheaper before 15 GB are on the card)."""
    paths = weight_paths(args.base, args.adapter)
    digest, dinfo = weights_sha256(paths, Path(args.digest_sidecar), rehash=args.rehash)
    if digest is None:
        raise SystemExit(f"weights_sha256 did not resolve over {[str(p) for p in paths]}: a "
                         f"named weight is missing or unreadable; refusing to serve an "
                         f"unidentified model")
    model = AerialVLAModel(args.base, args.adapter, precision=args.precision, attn=args.attn,
                           merge=args.merge, max_new_tokens=args.max_new_tokens)
    ident = serving_identity(args.base, args.adapter, args.precision, args.attn, model.merged,
                             digest)
    facts = {"max_new_tokens": model.max_new_tokens, "load": model.timing,
             "digest": dinfo, "weight_files": [p.as_posix() for p in paths],
             "libraries": library_versions(), "protocol": C.PROTOCOL_VERSION}
    try:
        from guardrail.manifest import collect_host_evidence
        facts["host"] = collect_host_evidence()
    except Exception:                                              # noqa: BLE001
        facts["host"] = None
    return model, ident, facts


def _protocol_frames(frames_dir: Path):
    from PIL import Image
    meta = json.loads((frames_dir / "frames.json").read_text(encoding="utf-8"))
    imgs = [Image.open(frames_dir / f["file"]).convert("RGB") for f in meta["frames"]]
    return imgs, meta


def _tiles(mosaic):
    return mosaic.crop((0, 0, 224, 224)), mosaic.crop((0, 224, 224, 448))


def bench_requests(frames_dir: Path, n: int, object_phrase: str = "orange barrier",
                   jpeg_quality: int = 95) -> list[dict]:
    """The R1 inputs as a client would send them: each committed protocol
    mosaic split into its front and down tiles, each tile JPEG-encoded, the
    pose of its capture, and the GOAL-derived phrase from that pose to the
    recorded target (hint_source "goal": the trained task, computed here on
    the client's side of the protocol by aerialvla_demo.semantic_direction -
    the server itself never sees the target)."""
    from aerialvla_demo import semantic_direction
    imgs, meta = _protocol_frames(frames_dir)
    target = tuple(meta["target"])
    base = []
    for img, f in zip(imgs, meta["frames"]):
        front, down = _tiles(img)
        phrase = semantic_direction((f["x"], f["y"]), f["psi"], target)
        base.append({"v": 2, "op": "infer", "seq": 0, "t_capture": 0.0,
                     "pose": {"x": f["x"], "y": f["y"], "up": f["up"], "yaw_rad": f["psi"]},
                     "front_jpg_b64": C.encode_jpeg_b64(front, jpeg_quality),
                     "down_jpg_b64": C.encode_jpeg_b64(down, jpeg_quality),
                     "phrase": phrase, "hint_source": "goal" if phrase else "none",
                     "object": object_phrase, "csp_text": None, "csp_mode": "off",
                     "_frame": f["file"]})
    out = []
    for k in range(n):
        r = dict(base[k % len(base)])
        r["seq"] = k + 1
        r["t_capture"] = float(k)
        out.append(r)
    return out


def probe_requests(frames_dir: Path, object_phrase: str = "orange barrier",
                   jpeg_quality: int = 95) -> list[dict]:
    """Every frame x every one of the 8 phrases: the fidelity probe, so two
    configurations can be compared output for output on varied inputs
    (including the empty phrase, which should answer LAND)."""
    imgs, meta = _protocol_frames(frames_dir)
    out = []
    k = 0
    for img, f in zip(imgs, meta["frames"]):
        front, down = _tiles(img)
        fj, dj = C.encode_jpeg_b64(front, jpeg_quality), C.encode_jpeg_b64(down, jpeg_quality)
        for phrase in C.PHRASES:
            k += 1
            out.append({"v": 2, "op": "infer", "seq": 10000 + k, "t_capture": float(k),
                        "pose": {"x": f["x"], "y": f["y"], "up": f["up"], "yaw_rad": f["psi"]},
                        "front_jpg_b64": fj, "down_jpg_b64": dj, "phrase": phrase,
                        "hint_source": "goal" if phrase else "none", "object": object_phrase,
                        "csp_text": None, "csp_mode": "off", "_frame": f["file"]})
    return out


def _stats(seconds: list[float]) -> dict:
    if not seconds:
        return {"n": 0}
    ms = sorted(s * 1e3 for s in seconds)

    def rank(q):
        return ms[min(len(ms) - 1, max(0, int(math.ceil(q * len(ms))) - 1))]
    p50 = statistics.median(ms)
    return {"n": len(ms), "p50_ms": round(p50, 1), "p95_ms": round(rank(0.95), 1),
            "max_ms": round(ms[-1], 1), "mean_ms": round(statistics.fmean(ms), 1),
            "hz_at_p50": round(1000.0 / p50, 3) if p50 > 0 else None}


def run_bench(core: V2Server, timed_reqs: list[dict], warmup: int,
              probe: list[dict] | None = None, cuda: Any = None,
              smi: Callable[[], int | None] = nvidia_smi_used_mib) -> dict:
    """Send `warmup` untimed then the timed requests through the protocol core
    (decode, mosaic, prompt, generate, strict read), recording the server's
    latency_s (model time), the generate time, the whole server time and each
    output. Then the probe set, if any. Every request is logged.

    VRAM: torch's allocated peak during the timed requests, which leaves out
    the allocator's cache and the CUDA context, and the reserved peak, which
    includes the cache (review of V1a, P5); the whole card by nvidia-smi
    after the timed requests. main() adds the card before the load, so the
    process's footprint is their difference. `cuda` (torch.cuda by default)
    and `smi` are parameters so the bench can be tested without a GPU."""
    def one(r):
        frame = r.get("_frame")
        req = {k: v for k, v in r.items() if not k.startswith("_")}
        t0 = time.perf_counter()
        # handle_request, not handle: it writes the JSONL record the bench
        # output names as "log" (until the review of V1a, P3, the bench
        # recorded a path to a log nothing ever wrote).
        resp, _, extra = core.handle_request(req)
        total = time.perf_counter() - t0
        if not resp.get("ok"):
            raise RuntimeError(f"bench request refused: {resp.get('error')}")
        return {"frame": frame, "phrase": req["phrase"], "raw_text": resp["raw_text"],
                "parse_ok": resp["parse_ok"], "bins": resp["bins"], "land": resp["land"],
                "latency_s": resp["latency_s"], "gen_s": extra["timing"]["gen_s"],
                "server_s": total, "new_tokens": extra.get("new_tokens"),
                "image_sha256": resp["image_sha256"], "prompt_sha256": resp["prompt_sha256"]}
    if cuda is None:
        import torch
        cuda = torch.cuda
    for r in timed_reqs[:warmup]:
        one(r)
    cuda.reset_peak_memory_stats()
    timed = [one(r) for r in timed_reqs[warmup:]]
    infer_peak = cuda.max_memory_allocated() / 1e9
    infer_reserved = cuda.max_memory_reserved() / 1e9
    smi_after = smi()
    probe_out = [one(r) for r in (probe or [])]
    return {"timed_outputs": timed, "probe_outputs": probe_out,
            "stats_latency": _stats([t["latency_s"] for t in timed]),
            "stats_gen": _stats([t["gen_s"] for t in timed]),
            "stats_server": _stats([t["server_s"] for t in timed]),
            "stats_probe_latency": _stats([t["latency_s"] for t in probe_out]),
            "new_tokens": sorted({t["new_tokens"] for t in timed if t["new_tokens"] is not None}),
            "parse_fail_timed": sum(1 for t in timed if not t["parse_ok"]),
            "parse_fail_probe": sum(1 for t in probe_out if not t["parse_ok"]),
            "land_probe": sum(1 for t in probe_out if t["land"]),
            "vram": {"inference_peak_allocated_gb": round(infer_peak, 3),
                     "inference_peak_reserved_gb": round(infer_reserved, 3),
                     "nvidia_smi_used_mib_after_timed": smi_after}}


def _add_model_args(ap: argparse.ArgumentParser) -> None:
    # No default for the three model flags (see the note at PRECISIONS):
    # check_model_args refuses a run that leaves one out and prints the
    # measured table (docs/data/aerialvla_serving_4090.json). The reference
    # configuration of the fidelity probe is --precision bf16 --attn eager
    # --no-merge; the measured low-VRAM option --precision nf4 --attn sdpa
    # --merge.
    ap.add_argument("--base", default=DEFAULT_BASE)
    ap.add_argument("--adapter", default=DEFAULT_ADAPTER)
    ap.add_argument("--precision", choices=PRECISIONS, default=None,
                    help="REQUIRED for a real model: nf4 (4-bit) or bf16 (upstream's)")
    ap.add_argument("--attn", choices=ATTNS, default=None,
                    help="REQUIRED for a real model: eager or sdpa")
    ap.add_argument("--merge", action=argparse.BooleanOptionalAction, default=None,
                    help="REQUIRED for a real model, one of the two: --merge "
                         "(merge_and_unload the LoRA in memory after loading) or "
                         "--no-merge (keep the adapter separate)")
    ap.add_argument("--save-merged", nargs="?", const=DEFAULT_MERGED_OUT, default=None,
                    help=f"(bf16 + --merge) write the merged model (default {DEFAULT_MERGED_OUT})")
    ap.add_argument("--max-new-tokens", type=int, default=DEFAULT_MAX_NEW_TOKENS)
    ap.add_argument("--digest-sidecar", default=str(SIDECAR_DEFAULT))
    ap.add_argument("--rehash", action="store_true", help="recompute weights_sha256")
    ap.add_argument("--missing-down", choices=C.MISSING_DOWN_POLICIES, default="black")


def check_model_args(ap: argparse.ArgumentParser, args) -> None:
    """Refuse (argparse's error, exit 2) a real-model run that does not name
    its whole configuration, or that asks --save-merged of anything but
    bf16 + --merge - before build_real spends two minutes loading 15 GB
    (review of V1a, P11: save_merged refused only after the load)."""
    missing = [flag for flag, v in (("--precision", args.precision), ("--attn", args.attn),
                                    ("--merge/--no-merge", args.merge)) if v is None]
    if missing:
        ap.error(f"choose the model configuration explicitly: {', '.join(missing)} missing. "
                 f"There is no default, because the precision changes what the model says. "
                 f"Measured on the RTX 4090 ({_rel(SERVING_JSON)}; agreement = identical "
                 f"output to the reference bf16/eager/unmerged on the probe):\n"
                 f"{configuration_table()}\n"
                 f"Which configuration flies is decided by measuring CityLife's own VRAM "
                 f"beside the model (next wave); every response's identity carries the "
                 f"chosen configuration's agreement.")
    if args.save_merged and (args.precision != "bf16" or not args.merge):
        ap.error("--save-merged needs --precision bf16 --merge: a re-quantised nf4 merge, or "
                 "an unmerged adapter, is not the merged bf16 model upstream ran")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="AerialVLA server, protocol v2")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p_s = sub.add_parser("serve", help="serve protocol v2 over TCP")
    _add_model_args(p_s)
    p_s.add_argument("--host", default=C.DEFAULT_HOST)
    p_s.add_argument("--port", type=int, default=C.DEFAULT_PORT)
    p_s.add_argument("--log", default=None, help="JSONL of every request and response "
                     "(default demo/out/vla_server_v2/requests_<utc>.jsonl)")
    p_s.add_argument("--fake-output", nargs="+", default=None,
                     help="serve a scripted stand-in instead of the model (no GPU; "
                          "identity fake:scripted)")
    p_s.add_argument("--fake-delay-s", type=float, default=0.0)
    p_b = sub.add_parser("bench", help="latency of one configuration, in process")
    _add_model_args(p_b)
    p_b.add_argument("--frames", default=str(ROOT / "deploy" / "evidence" / "vla_protocol_frames"))
    p_b.add_argument("--probe-frames", default=None,
                     help="directory of mosaics + frames.json for the 8-phrase fidelity probe")
    p_b.add_argument("--warmup", type=int, default=5)
    p_b.add_argument("--timed", type=int, default=50)
    p_b.add_argument("--log", default=None)
    p_b.add_argument("--out", required=True)
    p_p = sub.add_parser("ping", help="round trip against a running server")
    p_p.add_argument("--host", default=C.DEFAULT_HOST)
    p_p.add_argument("--port", type=int, default=C.DEFAULT_PORT)
    p_p.add_argument("--n", type=int, default=20)
    p_p.add_argument("--warmup", type=int, default=3)
    p_p.add_argument("--frames", default=str(ROOT / "deploy" / "evidence" / "vla_protocol_frames"))
    p_p.add_argument("--out", default=None)
    p_p.add_argument("--shutdown", action="store_true", help="send shutdown at the end")
    args = ap.parse_args(argv)

    if args.cmd == "ping":
        cli = V2Client(args.host, args.port)
        hello = cli.call({"v": 2, "op": "hello"})
        reqs = bench_requests(Path(args.frames), args.warmup + args.n)
        rtt, srv = [], []
        for k, r in enumerate(reqs):
            req = {kk: v for kk, v in r.items() if not kk.startswith("_")}
            t0 = time.perf_counter()
            resp = cli.call(req)
            dt = time.perf_counter() - t0
            if not resp.get("ok"):
                print(f"refused: {resp.get('error')}")
                return 1
            if k >= args.warmup:
                rtt.append(dt)
                srv.append(resp["latency_s"])
        if args.shutdown:
            cli.call({"v": 2, "op": "shutdown"})
        cli.close()
        res = {"hello": hello, "roundtrip": _stats(rtt), "server_latency": _stats(srv),
               "n": args.n, "warmup": args.warmup}
        print(json.dumps({k: res[k] for k in ("roundtrip", "server_latency")}, indent=1))
        if args.out:
            Path(args.out).write_text(json.dumps(res, indent=1), encoding="utf-8")
        return 0

    smi_before = None
    if args.cmd == "serve" and args.fake_output:
        model, ident, facts = ScriptedModel(args.fake_output, args.fake_delay_s), \
            dict(FAKE_IDENTITY), {"fake": True}
    else:
        check_model_args(p_s if args.cmd == "serve" else p_b, args)
        if args.cmd == "bench":
            smi_before = nvidia_smi_used_mib()      # the card before this process's model
        model, ident, facts = build_real(args)
        if args.save_merged:
            out = model.save_merged(args.save_merged, {"identity": ident, **facts})
            print(f"[vla-server] merged model written to {out}", flush=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log = args.log or str(ROOT / "demo" / "out" / "vla_server_v2" / f"{args.cmd}_{stamp}.jsonl")
    core = V2Server(model, ident, log, missing_down=args.missing_down, hello_extra=facts)

    if args.cmd == "bench":
        import torch
        load_peak = torch.cuda.max_memory_allocated() / 1e9
        load_reserved = torch.cuda.max_memory_reserved() / 1e9
        timed = bench_requests(Path(args.frames), args.warmup + args.timed)
        probe = probe_requests(Path(args.probe_frames)) if args.probe_frames else None
        res = run_bench(core, timed, args.warmup, probe)
        after = res["vram"]["nvidia_smi_used_mib_after_timed"]
        res["vram"].update({
            "load_peak_allocated_gb": round(load_peak, 3),
            "load_peak_reserved_gb": round(load_reserved, 3),
            "nvidia_smi_used_mib_before_load": smi_before,
            # The process's whole footprint (CUDA context, cache, weights), as
            # far as nothing else on the card changed during the run.
            "process_delta_mib": (after - smi_before if after is not None and
                                  smi_before is not None else None)})
        res["counts"] = dict(core.counts)
        out = {"schema": BENCH_SCHEMA,
               "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
               "config": {"precision": args.precision, "attn": args.attn,
                          "merged": bool(model.merged)},
               "identity": ident, "facts": facts, "warmup": args.warmup, "timed": args.timed,
               "frames": str(Path(args.frames).resolve().as_posix()),
               "probe_frames": args.probe_frames,
               # Written by every request (run_bench -> handle_request), unless
               # counts["log_failures"] says otherwise.
               "log": log,
               "command": "python demo/vla_server_v2.py " + " ".join(
                   argv if argv is not None else sys.argv[1:]),
               **res}
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(out, indent=1), encoding="utf-8")
        s = res["stats_latency"]
        print(f"[bench] {args.precision}/{args.attn}/{'merged' if model.merged else 'unmerged'}: "
              f"p50 {s.get('p50_ms')} ms p95 {s.get('p95_ms')} ms (n={s.get('n')}); "
              f"parse_fail {res['parse_fail_timed']}; VRAM inference peak "
              f"{res['vram']['inference_peak_allocated_gb']} GB -> {args.out}", flush=True)
        return 0

    core.probe_log()            # warns now, not at the first request, if it cannot write
    srv = make_tcp_server(core, args.host, args.port)
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        print(f"[vla-server] WARNING: listening on {args.host}, not loopback", flush=True)
    print(f"[vla-server] protocol v{C.PROTOCOL_VERSION} on {args.host}:{args.port} "
          f"model {ident['id']} ({ident['precision']}/{ident['attn']}"
          f"{'/merged' if ident['merged'] else ''}) log {log}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        print(f"[vla-server] stopped; {core.counts}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
