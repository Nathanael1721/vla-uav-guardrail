"""Does the VLA image's GPU stack work on this Jetson Orin? (runbook step 8)

Runs INSIDE the vla image (or its base, with --skip-bnb) on the Orin:

    docker compose --env-file deploy/topologies/hil.env \\
        -f deploy/compose/docker-compose.orin.yml --profile vla-bench \\
        run --rm -T --entrypoint python3 vla-bench deploy/orin/gpu_check.py \\
        > deploy/evidence/incoming/hil/gpu_check.json

It prints one JSON object (the facts it read and the problems it found) and
exits 1 when there is any problem. Importing a package proves nothing here:
on an Orin, PyPI's bitsandbytes imports fine and its first 4-bit kernel fails
("named symbol not found", no sm_87 code), and a CUDA 12.8 image whose
forward-compatibility libraries do not load still imports torch and reports
no GPU. So every claim is a computation on the GPU, compared with a
reference:

  * a float32 matrix product on the GPU against the same product on the CPU;
  * an NF4 4-bit quantize / dequantize round trip through bitsandbytes'
    CUDA kernels. Its relative error must be above 0 (a pass-through that did
    not quantize scores 0) and below 0.2 (NF4 on Gaussian data is about 0.1;
    a kernel returning zeros scores 1.0).

Each result also records the device it was computed on (`matmul_device`,
`nf4_device`, read from the result tensor before it is copied back), and a
result that was not computed on a CUDA device is refused: a CPU-only path
would otherwise match the CPU reference perfectly and pass.

On aarch64 the GPU must be compute capability 8.7 (every Jetson Orin).
Nothing here loads a model or reads weights.
"""
from __future__ import annotations

import argparse
import json
import platform
import sys

ORIN_CAPABILITY = (8, 7)
MATMUL_REL_TOL = 1e-2          # TF32 on Ampere stays well inside this
NF4_REL_ERR_RANGE = (0.0, 0.2)


def collect(skip_bnb: bool = False, size: int = 256) -> dict:
    """Read the facts by running the computations. Every failure is recorded
    as a string, never raised, so the report is always complete."""
    facts: dict = {"python": platform.python_version(), "machine": platform.machine()}
    try:
        import torch
    except Exception as e:                                   # noqa: BLE001
        facts["torch_error"] = f"{type(e).__name__}: {e}"
        return facts
    facts["torch"] = torch.__version__
    facts["torch_cuda"] = torch.version.cuda
    facts["cuda_available"] = bool(torch.cuda.is_available())
    if not facts["cuda_available"]:
        return facts
    try:
        facts["device_name"] = torch.cuda.get_device_name(0)
        facts["capability"] = list(torch.cuda.get_device_capability(0))
        facts["bf16_supported"] = bool(torch.cuda.is_bf16_supported())
        g = torch.Generator().manual_seed(0)
        a = torch.randn(size, size, generator=g)
        b = torch.randn(size, size, generator=g)
        ref = a @ b
        on_gpu = a.cuda() @ b.cuda()
        facts["matmul_device"] = str(on_gpu.device)
        got = on_gpu.cpu()
        facts["matmul_rel_err"] = float((got - ref).abs().max() / ref.abs().max())
    except Exception as e:                                   # noqa: BLE001
        facts["gpu_error"] = f"{type(e).__name__}: {e}"
        return facts
    if skip_bnb:
        facts["bitsandbytes"] = "skipped"
        return facts
    try:
        import bitsandbytes
        import bitsandbytes.functional as F
        facts["bitsandbytes"] = getattr(bitsandbytes, "__version__", "unknown")
        x = torch.randn(size, size, generator=torch.Generator().manual_seed(1)).half().cuda()
        q, state = F.quantize_4bit(x, quant_type="nf4")
        y = F.dequantize_4bit(q, state, quant_type="nf4")
        torch.cuda.synchronize()
        facts["nf4_device"] = str(y.device)
        facts["nf4_rel_err"] = float((x.float() - y.float()).norm() / x.float().norm())
    except Exception as e:                                   # noqa: BLE001
        facts["bnb_error"] = f"{type(e).__name__}: {e}"
    return facts


def problems(facts: dict, expect_capability: tuple[int, int] | None = None) -> list[str]:
    """Why these facts do not show a working GPU stack. Empty = they do."""
    out: list[str] = []
    if "torch_error" in facts:
        return [f"torch does not import: {facts['torch_error']}"]
    if not facts.get("torch_cuda"):
        out.append(f"torch {facts.get('torch')} is a CPU build (torch.version.cuda is None): "
                   f"a requirement replaced the base image's Jetson torch")
    if not facts.get("cuda_available"):
        out.append("torch sees no GPU: was the container started with the nvidia runtime, and "
                   "do the image's CUDA libraries suit this driver? (CUDA 12.8 images need "
                   "/usr/local/cuda/compat; the CUDA 12.6 fallback base is "
                   "dustynv/l4t-pytorch:r36.4.0)")
        return out
    if "gpu_error" in facts:
        out.append(f"a computation on the GPU failed: {facts['gpu_error']}")
        return out
    cap = tuple(facts.get("capability") or ())
    if expect_capability and cap != tuple(expect_capability):
        out.append(f"compute capability {cap}, expected {expect_capability} (a Jetson Orin)")
    if not _on_cuda(facts.get("matmul_device")):
        out.append(f"the matrix product meant for the GPU ran on {facts.get('matmul_device')!r}, "
                   f"not on a CUDA device: its agreement with the CPU shows nothing")
    err = facts.get("matmul_rel_err")
    if not isinstance(err, float) or not err < MATMUL_REL_TOL:
        out.append(f"the GPU matrix product differs from the CPU's by {err} (relative), "
                   f"limit {MATMUL_REL_TOL}")
    if facts.get("bitsandbytes") == "skipped":
        return out
    if "bnb_error" in facts:
        hint = ""
        if "symbol" in facts["bnb_error"].lower():
            hint = (" - the library has no code for this GPU (sm_87); rebuild the vla image, "
                    "whose install script builds bitsandbytes for it")
        out.append(f"bitsandbytes' 4-bit kernels fail on this GPU: {facts['bnb_error']}{hint}")
        return out
    if not _on_cuda(facts.get("nf4_device")):
        out.append(f"the NF4 round trip ran on {facts.get('nf4_device')!r}, not on a CUDA "
                   f"device: bitsandbytes' GPU kernels were not exercised")
    lo, hi = NF4_REL_ERR_RANGE
    nf4 = facts.get("nf4_rel_err")
    if not isinstance(nf4, float) or not lo < nf4 < hi:
        out.append(f"NF4 round-trip relative error {nf4}, expected between {lo} and {hi} "
                   f"(0 = nothing was quantized, about 1 = the kernel returned nothing)")
    return out


def _on_cuda(device) -> bool:
    """torch prints a CUDA device as 'cuda' or 'cuda:N'."""
    return isinstance(device, str) and (device == "cuda" or device.startswith("cuda:"))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--skip-bnb", action="store_true",
                    help="check torch and the GPU only (for a base image, before the build)")
    args = ap.parse_args(argv)
    facts = collect(skip_bnb=args.skip_bnb)
    expect = ORIN_CAPABILITY if facts.get("machine") in ("aarch64", "arm64") else None
    found = problems(facts, expect)
    print(json.dumps({"facts": facts, "expect_capability": expect, "problems": found,
                      "verdict": "ok" if not found else "FAIL"}, indent=2))
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main())
