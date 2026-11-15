"""Does OpenVLA-7B run unquantized (bf16) on this Orin's GPU? (decision 11)

Run in the AGX's native environment (deploy/orin/agx_vla_env.sh), after
deploy/orin/gpu_check.py --skip-bnb has passed:

    python deploy/orin/vla_bf16_check.py --model ~/models/openvla-7b \\
        > deploy/evidence/incoming/hil/vla_bf16_check.json

It loads the checkpoint in bfloat16 with no quantization (the precision
upstream publishes; the 64 GB AGX Orin holds it with room to spare), then asks
for one action on the R1 protocol's committed frame (its front half, the crop
tools/vla_backend_table.py gives OpenVLA) a few times with the call
demo/real_vla_demo.py flies: input_ids and pixel_values only, unnorm_key
bridge_orig, greedy. It prints one JSON object and exits 1 on any problem.

A gate, not a rate: R1 (5 warm-up + 50 timed inferences in the budgeted power
mode with the clocks locked, docs/RUNBOOK-orin-hil.md step 4) is
tools/vla_backend_table.py's job. What is refused here is a model that did
not land on the GPU, did not load in bf16, or answered with anything but
seven finite numbers. Whether repeated greedy answers were identical is
recorded, not judged.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FRAME = ROOT / "deploy" / "evidence" / "vla_protocol_frames" / "present_+00.png"
PROMPT = "In: What action should the robot take to fly forward and avoid restricted areas?\nOut:"
UNNORM_KEY = "bridge_orig"


def load_bf16(model_dir: str):
    """(processor, model, torch): the checkpoint in bfloat16 with no
    quantization, on the GPU. The one bf16 loader: this check's, and R1's for
    the `openvla_7b_bf16` row (tools/vla_backend_table.py measure), so the
    rate is timed on the model this check passed. It needs no bitsandbytes,
    which the AGX's native environment does not install."""
    import torch
    from transformers import AutoModelForVision2Seq, AutoProcessor
    proc = AutoProcessor.from_pretrained(model_dir, trust_remote_code=True)
    model = AutoModelForVision2Seq.from_pretrained(
        model_dir, attn_implementation="eager", torch_dtype=torch.bfloat16,
        low_cpu_mem_usage=True, trust_remote_code=True).to("cuda").eval()
    return proc, model, torch


def collect(model_dir: str, frame: Path = FRAME, repeats: int = 8) -> dict:
    """Load, ask `repeats` times, and read the facts. Every failure is
    recorded as a string, never raised, so the report is always complete."""
    facts: dict = {"model": str(model_dir), "frame": frame.name, "prompt": PROMPT,
                   "unnorm_key": UNNORM_KEY}
    try:
        import torch
        from PIL import Image
        # imported here so a stack that does not import is an import_error,
        # not a run_error of load_bf16
        from transformers import AutoModelForVision2Seq, AutoProcessor  # noqa: F401
    except Exception as e:                                   # noqa: BLE001
        facts["import_error"] = f"{type(e).__name__}: {e}"
        return facts
    facts["torch"] = torch.__version__
    facts["torch_cuda"] = torch.version.cuda
    if not torch.cuda.is_available():
        facts["cuda_available"] = False
        return facts
    facts["cuda_available"] = True
    facts["device_name"] = torch.cuda.get_device_name(0)
    try:
        t0 = time.perf_counter()
        proc, model, _ = load_bf16(model_dir)
        torch.cuda.synchronize()
        facts["load_s"] = round(time.perf_counter() - t0, 2)
        p = next(model.parameters())
        facts["param_dtype"] = str(p.dtype)
        facts["param_device"] = str(p.device)
        img = Image.open(frame).convert("RGB").crop((0, 0, 224, 224))
        lat, actions = [], []
        for _ in range(repeats):
            a = time.perf_counter()
            inputs = proc(PROMPT, img).to("cuda", dtype=torch.bfloat16)
            with torch.no_grad():
                act = model.predict_action(input_ids=inputs["input_ids"],
                                           pixel_values=inputs["pixel_values"],
                                           unnorm_key=UNNORM_KEY, do_sample=False)
            torch.cuda.synchronize()
            lat.append(round(time.perf_counter() - a, 4))
            actions.append([float(v) for v in act])
        facts["latency_s"] = lat
        facts["actions"] = actions
        facts["identical_answers"] = all(x == actions[0] for x in actions)
        facts["peak_memory_gb"] = round(torch.cuda.max_memory_allocated() / 1e9, 2)
    except Exception as e:                                   # noqa: BLE001
        facts["run_error"] = f"{type(e).__name__}: {e}"
    return facts


def problems(facts: dict) -> list[str]:
    """Why these facts do not show OpenVLA-7B answering in bf16 on the GPU.
    Empty = they do."""
    if "import_error" in facts:
        return [f"the VLA stack does not import: {facts['import_error']}"]
    if not facts.get("cuda_available"):
        return [f"torch {facts.get('torch')} sees no GPU (torch.version.cuda = "
                f"{facts.get('torch_cuda')}): run deploy/orin/gpu_check.py --skip-bnb"]
    if "run_error" in facts:
        return [f"loading or asking the model failed: {facts['run_error']}"]
    out = []
    device = str(facts.get("param_device") or "")
    if not (device == "cuda" or device.startswith("cuda:")):
        out.append(f"the weights are on {device!r}, not on a CUDA device")
    if facts.get("param_dtype") != "torch.bfloat16":
        out.append(f"the weights are {facts.get('param_dtype')!r}, not torch.bfloat16: "
                   f"this check is for the unquantized model")
    actions = facts.get("actions") or []
    if not actions:
        out.append("no action was returned")
    for k, act in enumerate(actions):
        if len(act) != 7 or not all(isinstance(v, float) and math.isfinite(v) for v in act):
            out.append(f"answer {k} is not seven finite numbers: {act!r}")
            break
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--model", default=str(Path.home() / "models" / "openvla-7b"),
                    help="the OpenVLA-7B checkpoint directory")
    ap.add_argument("--frame", default=str(FRAME), help="the input image (front half is used)")
    ap.add_argument("--repeats", type=int, default=8)
    args = ap.parse_args(argv)
    facts = collect(args.model, Path(args.frame), args.repeats)
    lat = facts.get("latency_s") or []
    if len(lat) > 3:
        warm = sorted(lat[3:])
        facts["warm_median_s"] = warm[len(warm) // 2]
    found = problems(facts)
    print(json.dumps({"facts": facts, "problems": found,
                      "verdict": "ok" if not found else "FAIL"}, indent=2))
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main())
