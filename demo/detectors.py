"""Open-vocabulary detector backends, behind one interface.

WHY THIS IS ALLOWED TO CHANGE

The grant says nothing about detectors. Its `Architecture constraints` lock the
flight path - hierarchical control, the 4-D action space, MAVROS 2 as the
Year-1 bridge - and its switchable-backend clause is about VLA backends:

    "The VLA backend interface is switchable (CognitiveDrone, OpenVLA generic,
     BitVLA, in-house stubs for unit tests, etc.); every backend must conform to
     this 4-D output shape, ..."

That clause does not cover this module: a detector is not a VLA backend and
emits no 4-D action. Nor does it cover the city demos' pilot. The city demos
fly a hand-written controller, not a VLA, so they test the Shield and the
tracker, not a constrained VLA. (Until 2026-10-06 this docstring argued that
the clause made swapping the detector "inside the contract"; that reading was
withdrawn, CHANGELOG.md, Retracted.) The detector is ours to choose because the
grant does not specify one.

One property must survive, by our own design choice rather than a grant term:
it has to stay OPEN-VOCABULARY, because the follow demos take their target as
a phrase ("a red car", "a person") and the stand-off rule binds by the class
that phrase resolves to - the same reason `docs/CHECKLIST-remaining-work.md`
recommends declining colour-tracking retraining.

WHY BOTHER

OWL-ViT is the measured weak link twice over. It scores our pedestrians at
0.03-0.07 against the taxi's 0.15-0.44, which is why the 10 m stand-off had to
be demonstrated on the camera-free rail. And its latency holds the in-flight
detector rate at 2.77-4.31 Hz on the five mid-evaluation flights, against a
4.0 Hz gate (mission rate, recomputed from the flight logs on 2026-10-06 by
tools/build_eval_data.py `det_hz_mission`).

WHAT THE INTERFACE PROMISES

`infer()` returns scores and boxes in IMAGE PIXELS, plus the preprocessing and
forward times kept apart. That split is not decoration: it is how this project
established that the in-flight slowdown is CPU-side contention with the recorder
rather than GPU-side contention with the renderer. A backend that reported one
total would throw that away.

Every backend here runs on the PINNED transformers 4.40.1. The environment also
carries OpenVLA's pins and is not worth disturbing to chase a detector.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

# Backend -> default checkpoint. All Apache-2.0.
DEFAULTS = {
    "owlvit": "google/owlvit-base-patch32",
    "owlv2": "google/owlv2-base-patch16-ensemble",
    "gdino": "IDEA-Research/grounding-dino-tiny",
    "gdino-base": "IDEA-Research/grounding-dino-base",
}

BACKENDS = tuple(DEFAULTS)


@dataclass
class Detector:
    backend: str
    model_id: str
    proc: object = None
    model: object = None
    # Grounding DINO wants one lowercase period-separated string rather than a
    # list of phrases, and it needs the tokenised input back at post-process
    # time. Cached per query so the cost is paid once, not per frame.
    _text: Optional[str] = None
    _phrases: List[str] = field(default_factory=list)


def load(backend: str, model_id: Optional[str] = None, device: str = "cuda") -> Detector:
    """Load a backend. Raises on an unknown name rather than guessing."""
    if backend not in DEFAULTS:
        raise ValueError(f"unknown detector backend {backend!r}; "
                         f"choose from {', '.join(BACKENDS)}")
    mid = model_id or DEFAULTS[backend]
    d = Detector(backend=backend, model_id=mid)

    if backend == "owlvit":
        from transformers import OwlViTForObjectDetection, OwlViTProcessor
        d.proc = OwlViTProcessor.from_pretrained(mid)
        d.model = OwlViTForObjectDetection.from_pretrained(mid).to(device).eval()
    elif backend == "owlv2":
        from transformers import Owlv2ForObjectDetection, Owlv2Processor
        d.proc = Owlv2Processor.from_pretrained(mid)
        d.model = Owlv2ForObjectDetection.from_pretrained(mid).to(device).eval()
    else:                                   # gdino, gdino-base
        from transformers import (AutoProcessor,
                                  AutoModelForZeroShotObjectDetection)
        d.proc = AutoProcessor.from_pretrained(mid)
        d.model = AutoModelForZeroShotObjectDetection.from_pretrained(mid).to(device).eval()
    return d


def _gdino_text(phrases: List[str]) -> str:
    """Grounding DINO's prompt format: lowercase, period-separated, trailing dot.

    Not cosmetic. The model matches text spans against boxes, and a prompt
    without the separators returns spans that straddle two phrases, so the label
    a box came back under stops being recoverable.
    """
    return " . ".join(p.strip().lower().rstrip(".") for p in phrases) + " ."


def infer(d: Detector, img, phrases: List[str], device: str = "cuda"):
    """Run one frame. Returns (scores, boxes_xyxy, labels, pre_ms, fwd_ms).

    `boxes_xyxy` are in image pixels. `labels` indexes into `phrases`.
    Timing is split preprocessing / forward, deliberately - see the module
    docstring.
    """
    import torch

    W, H = img.size
    t0 = time.time()

    if d.backend in ("owlvit", "owlv2"):
        inputs = d.proc(text=[phrases], images=img, return_tensors="pt").to(device)
        t_pre = time.time()
        with torch.no_grad():
            out = d.model(**inputs)
        torch.cuda.synchronize()
        t_fwd = time.time()
        res = d.proc.post_process_object_detection(
            out, threshold=0.0,
            target_sizes=torch.tensor([[H, W]]).to(device))[0]
        scores, boxes = res["scores"], res["boxes"]
        labels = res.get("labels")
    else:
        text = _gdino_text(phrases)
        inputs = d.proc(images=img, text=text, return_tensors="pt").to(device)
        t_pre = time.time()
        with torch.no_grad():
            out = d.model(**inputs)
        torch.cuda.synchronize()
        t_fwd = time.time()
        # box_threshold 0.0 keeps every candidate, matching the OWL path: the
        # caller does its own thresholding, colour gating and ranking, and a
        # backend that pre-filtered would silently change those decisions.
        #
        # text_threshold is NOT the same kind of knob and must NOT be 0.0. It
        # decides which text tokens a box is considered to match, so at zero
        # every box comes back labelled with the ENTIRE prompt - measured:
        # "a yellow car. a person. [SEP]" on all 115 boxes. The per-phrase label
        # is then unrecoverable, every box falls to phrase 0, and the second
        # query looks like it detected nothing.
        #
        # That is not hypothetical. It made Grounding DINO score 0.0000 on "a
        # person" in the first benchmark run, which would have been reported as
        # "cannot see pedestrians" - the exact opposite of the truth, since at
        # 0.1 the same frame yields people at 0.369 against OWL-ViT's 0.062.
        res = d.proc.post_process_grounded_object_detection(
            out, inputs["input_ids"], box_threshold=0.0, text_threshold=0.1,
            target_sizes=[(H, W)])[0]
        scores, boxes = res["scores"], res["boxes"]
        # Grounding DINO returns text labels, not indices. Map back to the
        # caller's phrase list; anything unrecognised falls to phrase 0, which
        # is the query the demo actually cares about.
        raw = res.get("labels") or res.get("text_labels") or []
        lut = {p.strip().lower().rstrip("."): i for i, p in enumerate(phrases)}
        labels = torch.tensor(
            [lut.get(str(x).strip().lower().rstrip("."), 0) for x in raw],
            device=scores.device if hasattr(scores, "device") else None,
        ) if len(raw) else None

    return scores, boxes, labels, (t_pre - t0) * 1000.0, (t_fwd - t_pre) * 1000.0
