"""
BitVLA's real loader, once, on the AGX Orin (ARCH-06, 2026-11-22).

Loads the LIBERO-Spatial BitVLA checkpoint with demo/vla_server_foreign.py's
BitVLAModel (BitVLA's own model class and action head, its transformers
fork), then sends real frames through the server's protocol core
(ForeignServer.handle: the same path a flight's RemoteVLA request takes) with
the BitVLA family (rule bitvla_chunk_velocity4/1). Writes every response
and the measurements to --out. Flies nothing; no TCP server is started.

Run on the AGX from a copy of this repository (not the campaign checkout):
    ~/bitvla-env/bin/python docs/data/bitvla_loader_20261122/probe_agx.py \
        --weights ~/models/bitvla-libero-spatial \
        --code ~/bitvla-src/BitVLA/openvla-oft --out ~/bitvla-run/out
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import os
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import vla_server_foreign as F                                       # noqa: E402
import vla_server_v2 as S                                            # noqa: E402

# (label, repository path, crop box or None): real frames already in the
# repository, plus one synthetic grey frame
FRAMES = (
    ("protocol_present_front", "deploy/evidence/vla_protocol_frames/present_+00.png",
     (0, 0, 224, 224)),
    ("protocol_absent_front", "deploy/evidence/vla_protocol_frames/absent_+00.png",
     (0, 0, 224, 224)),
    ("citylife_fpv_before", "docs/img/progress_0930_before_fpv.jpg", None),
    ("citylife_fpv_after", "docs/img/progress_0930_after_fpv.jpg", None),
    ("synthetic_grey", None, None),
)
OBJECT = "a red car"
CALLS_PER_FRAME = 5


def mem_available_mib() -> float:
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) / 1024.0
    return float("nan")


def jpg_b64(img) -> str:
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=90)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def load_frames() -> list[tuple[str, object, str]]:
    from PIL import Image
    out = []
    for label, rel, box in FRAMES:
        if rel is None:
            out.append((label, Image.new("RGB", (640, 480), (128, 128, 128)), "synthetic"))
            continue
        img = Image.open(ROOT / rel).convert("RGB")
        if box is not None:
            img = img.crop(box)
        out.append((label, img, rel + (f" crop {list(box)}" if box else "")))
    return out


def req_for(seq: int, t: float, b64: str) -> dict:
    return {"v": 2, "op": "infer", "seq": seq, "t_capture": t,
            "pose": {"x": 0.0, "y": 0.0, "up": 10.0, "yaw_rad": 0.0},
            "front_jpg_b64": b64, "down_jpg_b64": None, "phrase": "straight ahead ",
            "hint_source": "detector", "object": OBJECT, "csp_text": None, "csp_mode": "off"}


def git_head(d: Path) -> str | None:
    try:
        return subprocess.run(["git", "-C", str(d), "rev-parse", "HEAD"], capture_output=True,
                              text=True, check=True).stdout.strip()
    except Exception:                                              # noqa: BLE001
        return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--code", required=True, help="BitVLA's openvla-oft folder")
    ap.add_argument("--out", required=True)
    ap.add_argument("--device", default="cuda:0")
    a = ap.parse_args()
    weights, code, out = (Path(os.path.expanduser(p)) for p in (a.weights, a.code, a.out))
    out.mkdir(parents=True, exist_ok=True)
    import torch
    import transformers
    import tokenizers

    rec: dict = {"date": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "host": platform.node(),
                 "machine": platform.machine(), "python": platform.python_version(),
                 "torch": torch.__version__, "cuda": torch.version.cuda,
                 "transformers": transformers.__version__,
                 "tokenizers": tokenizers.__version__,
                 "bitvla_repo_commit": git_head(code.parent),
                 "checkpoint": "hongyuw/ft-bitvla-bitsiglipL-224px-libero_spatial-bf16",
                 "checkpoint_folder": weights.name}

    # the identity digest, as serve computes it before the model is loaded
    paths = F.weight_files("bitvla", weights)
    t = time.perf_counter()
    digest, dinfo = S.weights_sha256(paths, out / "digest_sidecar.json")
    rec["weights_sha256"], rec["digest_s"] = digest, time.perf_counter() - t
    rec["weight_files"] = [p.name for p in paths]

    mem0 = mem_available_mib()
    torch.cuda.reset_peak_memory_stats()
    model = F.BitVLAModel(weights, device=a.device, code_dir=code)
    rec["load"] = dict(model.timing)
    rec["code"], rec["image_pipeline"] = model.code, model.image_pipeline
    rec["memory_after_load"] = {
        "cuda_allocated_mib": torch.cuda.memory_allocated() / 2**20,
        "cuda_peak_allocated_mib": torch.cuda.max_memory_allocated() / 2**20,
        "mem_available_drop_mib": mem0 - mem_available_mib()}

    key, _ = F.load_action_stats(weights / "dataset_statistics.json", None, dims=(F.ACTION_DIM,))
    fam = F.BitVLAFamily(key)
    ident = F.foreign_identity("bitvla", name="ft-bitvla-libero_spatial", folder=weights.name,
                               revision=S.local_hf_revision(weights), digest=digest,
                               precision="bf16", attn="sdpa", front_source="vla",
                               facts={**fam.describe(), "code": model.code,
                                      "image_pipeline": model.image_pipeline,
                                      "checkpoint": model.checkpoint})
    core = F.ForeignServer(fam, model, ident, None)
    rec["identity_id"] = ident["id"]
    rec["unnorm_key"] = key

    torch.cuda.reset_peak_memory_stats()
    responses, per_frame = [], []
    seq = 0
    for label, img, src in load_frames():
        b64 = jpg_b64(img)
        calls = []
        for i in range(CALLS_PER_FRAME):
            seq += 1
            t0 = time.perf_counter()
            resp, _, extra = core.handle(req_for(seq, 1000.0 + seq, b64))
            wall = time.perf_counter() - t0
            if not resp.get("ok"):
                raise SystemExit(f"{label} call {i}: {resp.get('error')}")
            responses.append({"frame": label, "call": i, "wall_s": wall, "response": resp})
            calls.append((wall, resp))
            rec.setdefault("prompt_text", extra.get("prompt"))
        chunks = [r["native"]["action"]["chunk"] for _, r in calls]
        r0 = calls[0][1]
        per_frame.append({
            "frame": label, "source": src, "size": list(img.size),
            "first_call_wall_s": calls[0][0],
            "wall_s_after_first": [w for w, _ in calls[1:]],
            "timing": [r["timing"] for _, r in calls],
            "deterministic": all(c == chunks[0] for c in chunks),
            "prompt_tokens": r0["timing"]["prompt_tokens"],
            "chunk": chunks[0], "chunk_shape": [len(chunks[0]), len(chunks[0][0])],
            "mapping": r0["mapping"], "step": r0["step"], "raw_text": r0["raw_text"],
            "bins": r0["bins"]})
        print(f"{label}: chunk row0 {[round(v, 4) for v in chunks[0][0]]} -> step "
              f"{r0['step']} clipped {r0['mapping']['clipped']}", flush=True)
    walls = [w for f in per_frame for w in f["wall_s_after_first"]]
    rec["frames"] = per_frame
    rec["latency_wall_s_excluding_each_frames_first_call"] = {
        "n": len(walls), "median": statistics.median(walls), "min": min(walls),
        "max": max(walls), "mean": statistics.fmean(walls)}
    rec["very_first_call_wall_s"] = per_frame[0]["first_call_wall_s"]
    rec["memory_during_inference"] = {
        "cuda_peak_allocated_mib": torch.cuda.max_memory_allocated() / 2**20,
        "mem_available_drop_mib_vs_before_load": mem0 - mem_available_mib()}
    rec["server_counts"] = getattr(core, "_counts", None)
    (out / "measurements.json").write_text(json.dumps(rec, indent=1, default=repr) + "\n",
                                           encoding="utf-8")
    with (out / "responses.jsonl").open("w", encoding="utf-8") as f:
        for r in responses:
            f.write(json.dumps(r, default=repr) + "\n")
    print(json.dumps({k: rec[k] for k in ("load", "memory_after_load",
                                          "latency_wall_s_excluding_each_frames_first_call",
                                          "memory_during_inference")}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
