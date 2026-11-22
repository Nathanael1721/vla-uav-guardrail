"""
Two loader choices of BitVLAModel, checked on the AGX (2026-11-22):

  attention: the repository's model class runs the LLM with use_bi_attn=True
    (OFT parallel decoding, as trained); the checkpoint's own copy of
    bitvla_for_action_prediction.py omits it, so it runs the fork's default,
    causal. Here the same loaded model answers the same frames both ways (the
    checkpoint's behaviour reproduced by forcing use_bi_attn=False).
  cameras: the frame in both of the checkpoint's two image slots (what
    BitVLAModel sends) vs in one slot only.

    ~/bitvla-env/bin/python docs/data/bitvla_loader_20261122/ablation_agx.py \
        --weights ~/models/bitvla-libero-spatial \
        --code ~/bitvla-src/BitVLA/openvla-oft --out ~/bitvla-run/out
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import probe_agx as P                                                # noqa: E402

F = P.F


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--code", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    weights, code, out = (Path(os.path.expanduser(p)) for p in (a.weights, a.code, a.out))
    from transformers import LlavaForConditionalGeneration
    model = F.BitVLAModel(weights, code_dir=code)
    key, _ = F.load_action_stats(weights / "dataset_statistics.json", None, dims=(F.ACTION_DIM,))
    fam = F.BitVLAFamily(key)
    req = P.req_for(1, 1.0, "")
    base = fam.inputs(req, None, [])
    one = {**base, "prompt": F.bitvla_user_text(base["instruction"], 1), "n_images": 1}

    orig = LlavaForConditionalGeneration.forward

    def causal(self, *args, **kw):
        kw["use_bi_attn"] = False
        return orig(self, *args, **kw)

    rows = []
    for label, img, _ in P.load_frames():
        frame = fam.prepare(img)[0]
        bi2 = model.generate(frame, base)["raw"]
        bi1 = model.generate(frame, one)["raw"]
        LlavaForConditionalGeneration.forward = causal
        try:
            causal2 = model.generate(frame, base)["raw"]
        finally:
            LlavaForConditionalGeneration.forward = orig
        again = model.generate(frame, base)["raw"]

        def dist(x, y):
            return max(abs(u - v) for r, s in zip(x, y) for u, v in zip(r, s))

        rows.append({"frame": label, "bidirectional_two_slots_row0": bi2[0],
                     "bidirectional_one_slot_row0": bi1[0], "causal_two_slots_row0": causal2[0],
                     "max_abs_diff_causal_vs_bidirectional": dist(causal2, bi2),
                     "max_abs_diff_one_vs_two_slots": dist(bi1, bi2),
                     "restored_equals_first": again == bi2})
        print(json.dumps(rows[-1]), flush=True)
    (out / "ablation.json").write_text(json.dumps(rows, indent=1) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
