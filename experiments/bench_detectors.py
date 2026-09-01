"""Which open-vocabulary detector should this project use?

Offline, on frames already on disk, so it is cheap and repeatable and needs no
simulator. Four numbers per backend, because all four can disqualify a model:

  LATENCY   `det_hz` must clear 4.0 Hz and OWL-ViT only just does (66 ms idle,
            287 ms in flight). A more accurate model that misses the gate is not
            an improvement, it is a different failure.
  TAXI      the case that works today - 0.15-0.44. A regression here would cost
            the demo the thing it currently does well.
  PERSON    the case that fails today - 0.03-0.07, four times weaker, which is
            why the 10 m stand-off had to be flown on the camera-free rail.
  VRAM      the 16 GB card also hosts Unreal.

Scores are NOT comparable across architectures in absolute terms - OWL-ViT and
Grounding DINO calibrate differently - so the useful comparison is the RATIO of
person to taxi within one model, plus where each sits against its own noise
floor. Both are reported.

Usage:
    python experiments/bench_detectors.py                    # all backends
    python experiments/bench_detectors.py --backends owlvit owlv2
"""
from __future__ import annotations

import argparse
import glob
import json
import statistics as st
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import detectors                                            # noqa: E402

# The demo's own queries, unchanged. Benchmarking on different words would
# measure a prompt, not a model.
Q_TAXI = "a yellow car"
Q_PERSON = "a person"

# Frames from the delivered populated-city flight: the taxi is in shot and so
# are twelve pedestrians, which is exactly the scene the detector has to serve.
FRAME_DIR = ROOT / "demo" / "out" / "city_locked" / "view" / "fpv"


def frames(n: int):
    fs = sorted(glob.glob(str(FRAME_DIR / "*.jpg")))
    if not fs:
        raise SystemExit(f"no frames under {FRAME_DIR} - run a flight with --save-view")
    step = max(1, len(fs) // n)
    return fs[::step][:n]


def bench(backend: str, paths, warmup: int = 3):
    from PIL import Image
    import torch

    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()
    t0 = time.time()
    d = detectors.load(backend)
    load_s = time.time() - t0

    imgs = [Image.open(p).convert("RGB") for p in paths]
    # The detector runs on the FRONT camera at its native size, not on the
    # recorded frame: benchmarking at 960x540 would measure a resolution the
    # flight never uses. 400x225 is what demo/follow_vlm.py feeds it.
    imgs = [im.resize((400, 225)) for im in imgs]

    phrases = [Q_TAXI, Q_PERSON]
    for im in imgs[:warmup]:                    # warm the kernels
        detectors.infer(d, im, phrases)

    pre, fwd, taxi, person = [], [], [], []
    for im in imgs:
        sc, bx, lb, p_ms, f_ms = detectors.infer(d, im, phrases)
        pre.append(p_ms)
        fwd.append(f_ms)
        if len(sc):
            s = sc.detach().cpu()
            l = lb.detach().cpu() if lb is not None else None
            if l is not None and len(l) == len(s):
                t = s[l == 0]
                p = s[l == 1]
                taxi.append(float(t.max()) if len(t) else 0.0)
                person.append(float(p.max()) if len(p) else 0.0)
            else:
                taxi.append(float(s.max()))
                person.append(0.0)
        else:
            taxi.append(0.0)
            person.append(0.0)

    total = [a + b for a, b in zip(pre, fwd)]
    vram = torch.cuda.max_memory_allocated() / 1e9
    del d
    torch.cuda.empty_cache()

    return {
        "backend": backend,
        "model_id": detectors.DEFAULTS[backend],
        "load_s": round(load_s, 1),
        "frames": len(imgs),
        "pre_ms_median": round(st.median(pre), 1),
        "fwd_ms_median": round(st.median(fwd), 1),
        "total_ms_median": round(st.median(total), 1),
        "total_ms_p95": round(sorted(total)[int(0.95 * (len(total) - 1))], 1),
        # What the control loop would actually see if inference were the only
        # cost. In flight it is not - measured 4.3x worse - so treat this as an
        # upper bound, not a prediction.
        "hz_ceiling": round(1000.0 / st.median(total), 2),
        "taxi_score_median": round(st.median(taxi), 4),
        "person_score_median": round(st.median(person), 4),
        "person_over_taxi": (round(st.median(person) / st.median(taxi), 3)
                             if st.median(taxi) > 0 else None),
        "vram_gb": round(vram, 2),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--backends", nargs="*", default=list(detectors.BACKENDS))
    ap.add_argument("--frames", type=int, default=24)
    ap.add_argument("--out", default=str(ROOT / "docs" / "data" / "detector_bench.json"))
    args = ap.parse_args()

    paths = frames(args.frames)
    print(f"{len(paths)} frames from {FRAME_DIR.relative_to(ROOT)}")
    print(f"queries: {Q_TAXI!r} / {Q_PERSON!r}, inference size 400x225\n")

    rows = []
    for b in args.backends:
        print(f"--- {b} ({detectors.DEFAULTS[b]}) ---", flush=True)
        try:
            r = bench(b, paths)
        except Exception as e:                                # noqa: BLE001
            # A backend that cannot run is a RESULT, recorded rather than
            # dropped: silently benchmarking three of four models and
            # presenting it as a survey would be the dishonest version.
            print(f"    FAILED {type(e).__name__}: {e}\n", flush=True)
            rows.append({"backend": b, "model_id": detectors.DEFAULTS[b],
                         "error": f"{type(e).__name__}: {e}"})
            continue
        rows.append(r)
        print(f"    {r['total_ms_median']:.0f} ms median "
              f"({r['pre_ms_median']:.0f} pre + {r['fwd_ms_median']:.0f} fwd), "
              f"ceiling {r['hz_ceiling']:.1f} Hz, {r['vram_gb']:.2f} GB")
        print(f"    taxi {r['taxi_score_median']:.4f}   "
              f"person {r['person_score_median']:.4f}   "
              f"ratio {r['person_over_taxi']}\n", flush=True)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "_why": "Which open-vocabulary detector should this project use. "
                "Scores are not comparable in absolute terms across "
                "architectures; person_over_taxi is the within-model figure.",
        "_gate": {"det_hz_min": 4.0,
                  "note": "hz_ceiling is inference alone on an idle GPU. "
                          "In flight OWL-ViT measures 4.3x worse than its "
                          "idle figure, so a ceiling near 4 Hz will not hold."},
        "_frames": str(FRAME_DIR.relative_to(ROOT)),
        "_queries": [Q_TAXI, Q_PERSON],
        "_inference_size": "400x225",
        "results": rows,
    }, indent=2), encoding="utf-8")
    print(f"wrote {out.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
