"""
Re-score the 4 Aug AerialVLA ablation with the strict action reader.

    python experiments/rescore_ablation.py                       # experiments/out/ablation_*.json
    python experiments/rescore_ablation.py --src <dir with ablation_orig.json, ablation_ft.json>
    python experiments/rescore_ablation.py --check               # exit 1 if the committed file differs

WHY

experiments/ablate_image_vs_hint.py read the model's text with
`re.findall(r"\\d+", tail)[-3:]`, upstream's own parser. That takes the last
three integers anywhere in the text, so a malformed landing output such as
'00 49 494 49' became (49, 494, 49) -> forward 2.5 m and a clamped +5.0 m
"down": motion read out of a stop. docs/FINDING-what-drives-aerialvla.md
(4 Aug) built its no-hint row (+0.182 yaw, 0.56 forward, LAND 11/18) and its
"weakly positive" object-presence comparison on those numbers.

This script reads the SAME stored outputs (no model, no GPU) with
guardrail.aerialvla_codec.parse_action - exactly three two-digit bins 00..98,
single spaces, optional " LAND", then EOS - and writes every row and every
summary to docs/data/aerialvla_ablation_rescore.json (tracked; the inputs under
experiments/out/ are gitignored, so each row carries its stored text and the
old reading, and the inputs' SHA-256 is recorded).

How a stored tail is read. The 4 Aug script stored `tail.strip()` for the
rows its parser accepted and the raw `decode(full).split("Action:")[-1]` for
the rest, which keeps the prompt's own trailing space. One leading space is
removed (aerialvla_codec.generated_from_decoded's rule), never more, and EOS
is required: a tail without "</s>" ran out of max_new_tokens (20 there).

What each row becomes (the client contract, BRIEF-V1):
    land        parse_ok and " LAND"             -> stop; the rail lands
    hold        parse_ok False                   -> the client holds zero (counted)
    zero_step   parse_ok, no LAND, all axes 0    -> no motion
    move        parse_ok, no LAND, a non-zero step
"stop" = land + hold + zero_step.

UNITS. The 4 Aug tables said rad/s and m/s; upstream's semantics make every
number a STEP: dpsi_rad radians and fwd_m metres per decision
(guardrail/aerialvla_codec.py). The values are the same numbers, relabelled.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.aerialvla_codec import (generated_from_decoded,  # noqa: E402
                                       legacy_last_three_ints, parse_action)

SCHEMA = "vlaguard.aerialvla_ablation_rescore/1"
OUT = ROOT / "docs" / "data" / "aerialvla_ablation_rescore.json"
LABELS = {
    "orig": {"adapter": "D:/models/aerialvla-lora/aero_vla (original AerialVLA LoRA)",
             "command": "python experiments/ablate_image_vs_hint.py --label orig"},
    "ft": {"adapter": "D:/models/aerialvla-ft/run2/epoch1 (our QLoRA fine-tune, velocity labels)",
           "command": "python experiments/ablate_image_vs_hint.py "
                      "--adapter D:/models/aerialvla-ft/run2/epoch1 --label ft"},
}
HINT_ORDER = ["to your left", "forward-left", "straight ahead", "forward-right",
              "to your right", "(none)"]
OBJECT_ORDER = ["orange barrier", "blue barrier", "(none)"]


def _sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _r(x, nd=3):
    return None if x is None else round(float(x), nd)


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return _r(statistics.fmean(xs)) if xs else None


def _spread(xs):
    xs = [x for x in xs if x is not None]
    return (max(xs) - min(xs)) if len(xs) > 1 else 0.0


def rescore_row(r: dict) -> dict:
    tail = r.get("tail", "")
    # The stored tail is what followed "Action:" (stripped on accepted rows,
    # raw on the rest): generated_from_decoded's one-space rule applies as is.
    gen = generated_from_decoded("Action:" + tail)
    p = parse_action(gen, require_eos=True)
    legacy = legacy_last_three_ints(tail)
    if p.parse_ok:
        outcome = "land" if p.land else ("zero_step" if p.step.is_zero else "move")
    else:
        outcome = "hold"
    stored = {k: r.get(k) for k in ("ok", "bins", "fwd", "down", "yaw", "land")}
    # The legacy reading recomputed from the stored text must reproduce what
    # the 4 Aug script stored, or this file is not re-reading the same data.
    if r.get("ok"):
        assert legacy["ok"] and legacy["bins"] == list(r["bins"]), (tail, legacy, r["bins"])
    return {"file": r["file"], "scene": r["scene"], "hint": r["hint"], "object": r["object"],
            "tail": tail, "stored": stored,
            "strict": {"parse_ok": p.parse_ok, "reason": p.reason,
                       "bins": list(p.bins) if p.bins else None,
                       "step": p.step.to_dict() if p.step else None, "land": p.land},
            "outcome": outcome}


def _block(rows: list[dict]) -> dict:
    ok = [x for x in rows if x["strict"]["parse_ok"]]
    outc = {k: sum(1 for x in rows if x["outcome"] == k)
            for k in ("land", "hold", "zero_step", "move")}
    stored_ok = [x for x in rows if x["stored"]["ok"]]
    return {
        "n": len(rows), "parse_ok": len(ok), **outc,
        "stop": outc["land"] + outc["hold"] + outc["zero_step"],
        # over the rows the strict reader accepts (a LAND row is its decoded step)
        "mean_dpsi_rad": _mean([x["strict"]["step"]["dpsi_rad"] for x in ok]),
        "mean_fwd_m": _mean([x["strict"]["step"]["fwd_m"] for x in ok]),
        # as the client would fly them: a hold is the zero step
        "flown_mean_dpsi_rad": _mean([x["strict"]["step"]["dpsi_rad"] if x["strict"]["step"]
                                      else 0.0 for x in rows]),
        "flown_mean_fwd_m": _mean([x["strict"]["step"]["fwd_m"] if x["strict"]["step"]
                                   else 0.0 for x in rows]),
        "stored": {"parsed": len(stored_ok),
                   "mean_yaw": _mean([x["stored"]["yaw"] for x in stored_ok]),
                   "mean_fwd": _mean([x["stored"]["fwd"] for x in stored_ok]),
                   "land": sum(1 for x in stored_ok if x["stored"]["land"])},
    }


def summarise(rows: list[dict]) -> dict:
    ok = [x for x in rows if x["strict"]["parse_ok"]]
    files = sorted({x["file"] for x in rows})
    by_hint = {h: _block([x for x in rows if x["hint"] == h]) for h in HINT_ORDER
               if any(x["hint"] == h for x in rows)}
    presence = {s: _block([x for x in rows if x["hint"] == "(none)" and x["scene"] == s])
                for s in ("present", "absent")}
    objects = {o: _block([x for x in rows if x["scene"] == "present" and x["object"] == o])
               for o in OBJECT_ORDER}
    hinted = [x for x in rows if x["hint"] != "(none)"]

    def dpsi(x):
        return x["strict"]["step"]["dpsi_rad"]

    # The 4 Aug script's three spreads, recomputed over the accepted rows only:
    # the dpsi range when ONE factor varies and the other two are held fixed.
    sp_hint = [_spread([dpsi(x) for x in ok if x["file"] == f and x["object"] == o])
               for f in files for o in OBJECT_ORDER
               if sum(1 for x in ok if x["file"] == f and x["object"] == o) > 1]
    sp_img = [_spread([dpsi(x) for x in ok if x["hint"] == h and x["object"] == o])
              for h in HINT_ORDER for o in OBJECT_ORDER
              if sum(1 for x in ok if x["hint"] == h and x["object"] == o) > 1]
    sp_obj = [_spread([dpsi(x) for x in ok if x["file"] == f and x["hint"] == h])
              for f in files for h in HINT_ORDER
              if sum(1 for x in ok if x["file"] == f and x["hint"] == h) > 1]
    return {
        "rows": len(rows), "parse_ok": len(ok), "parse_fail": len(rows) - len(ok),
        "parse_fail_rows": [{"hint": x["hint"], "scene": x["scene"], "object": x["object"],
                             "tail": x["tail"], "reason": x["strict"]["reason"],
                             "stored_bins": x["stored"]["bins"]}
                            for x in rows if not x["strict"]["parse_ok"]],
        "by_hint": by_hint,
        "no_hint_present_vs_absent": presence,
        "object_phrase_target_present": objects,
        "hinted_land": sum(1 for x in hinted if x["outcome"] == "land"),
        "hinted_rows": len(hinted),
        "dpsi_range_when_one_factor_varies": {
            "hint": {"mean": _r(statistics.fmean(sp_hint)) if sp_hint else None,
                     "max": _r(max(sp_hint)) if sp_hint else None},
            "image": {"mean": _r(statistics.fmean(sp_img)) if sp_img else None,
                      "max": _r(max(sp_img)) if sp_img else None},
            "object": {"mean": _r(statistics.fmean(sp_obj)) if sp_obj else None,
                       "max": _r(max(sp_obj)) if sp_obj else None}},
    }


def build(src: Path) -> dict:
    out = {"schema": SCHEMA,
           "generator": "experiments/rescore_ablation.py",
           "parser": "guardrail/aerialvla_codec.py parse_action(require_eos=True); one leading "
                     "space removed (generated_from_decoded's rule)",
           "units": "fwd_m and down_m metres per step, dpsi_rad radians per step (upstream "
                    "step semantics); the 4 Aug tables' 'rad/s' and 'm/s' are these numbers",
           "outcomes": {"land": "parse_ok with LAND", "hold": "parse_ok False: the client "
                        "holds zero", "zero_step": "parse_ok, no LAND, every axis below 0.01",
                        "move": "parse_ok, no LAND, a non-zero step",
                        "stop": "land + hold + zero_step"},
           "sources": {}, "adapters": {}}
    for label, meta in LABELS.items():
        p = Path(src) / f"ablation_{label}.json"
        rows_in = json.loads(p.read_text(encoding="utf-8"))
        rows = [rescore_row(r) for r in rows_in]
        out["sources"][label] = {
            "file": f"experiments/out/ablation_{label}.json (gitignored; lab desktop)",
            "sha256": _sha256(p), "rows": len(rows_in), "adapter": meta["adapter"],
            "made_by": meta["command"], "date": "2026-08-04", "max_new_tokens": 20,
            "prompt_builder": "demo/aerialvla_demo.build_prompt (whitespace-normalised; the "
                              "'(none)' object arm ends 'target.\\nAction: ')",
            "frames": "experiments/out/frames (6 mosaics: target present/absent x 3 headings; "
                      "two are committed in deploy/evidence/vla_protocol_frames)"}
        out["adapters"][label] = {"summary": summarise(rows), "rows": rows}
    return out


def _dump(d: dict) -> str:
    return json.dumps(d, indent=1, ensure_ascii=False) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    ap.add_argument("--src", type=Path, default=ROOT / "experiments" / "out")
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--check", action="store_true",
                    help="compare with the committed file instead of writing it")
    args = ap.parse_args(argv)
    d = build(args.src)
    text = _dump(d)
    if args.check:
        same = args.out.is_file() and args.out.read_text(encoding="utf-8") == text
        print("unchanged" if same else f"DIFFERS from {args.out}")
        return 0 if same else 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    for label, a in d["adapters"].items():
        s = a["summary"]
        nh = s["by_hint"]["(none)"]
        print(f"[{label}] parsed {s['parse_ok']}/{s['rows']}; no hint: stop {nh['stop']}/{nh['n']} "
              f"(LAND {nh['land']}, hold {nh['hold']}, zero {nh['zero_step']}), move {nh['move']}")
        for h, b in s["by_hint"].items():
            print(f"   {h:15} n={b['n']:2d} ok={b['parse_ok']:2d} dpsi={b['mean_dpsi_rad']!s:>7} "
                  f"fwd={b['mean_fwd_m']!s:>6}  (stored yaw {b['stored']['mean_yaw']}, "
                  f"fwd {b['stored']['mean_fwd']}, LAND {b['stored']['land']})")
        pr = s["no_hint_present_vs_absent"]
        print(f"   no hint, present: move {pr['present']['move']}/{pr['present']['n']}, "
              f"absent: move {pr['absent']['move']}/{pr['absent']['n']}")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
