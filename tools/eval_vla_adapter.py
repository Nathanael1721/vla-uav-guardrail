"""
The held-out metrics of an AerialVLA adapter, measured through the VLA
server's own path (protocol v2: decode, mosaic, prompt, generate, the strict
reader) - against a running server or in process - and the same for the
ORIGINAL adapter as the baseline.

    # in process (vla-real env, GPU free: loads the 7B model twice in turn)
    python tools/eval_vla_adapter.py --data D:/datasets/follow_v1 \\
        --adapter D:/models/aerialvla-ft-follow/follow_r1/epoch3 \\
        --in-process --precision bf16 --attn sdpa --no-merge --out eval_r1e3.json

    # against running servers (each started with the adapter it is scored as;
    # the identity's adapter digest must match, or the run is refused)
    python tools/eval_vla_adapter.py --data ... --adapter D:/models/aerialvla-ft-follow/follow_r1/epoch3 \\
        --server 127.0.0.1:8791 --baseline-server 127.0.0.1:8792

    # the whole path without a model (the scripted stand-in; NOT a measurement)
    python tools/eval_vla_adapter.py --data ... --adapter DIR --fake-output "96 48 49</s>" --no-baseline

WHAT IS SENT. For every held-out sample (training/finetune_aerialvla_follow.py
reads the dataset; --split chooses which), the request a flight would send:
the mosaic's front tile (top 224x224) and down tile (bottom) as PNG - lossless,
and the server's bicubic resize of a 224x224 tile to 224x224 is the identity
(Pillow returns a copy), so the server rebuilds the dataset's mosaic pixel for
pixel - the sample's phrase and object words (the server builds the prompt),
hint_source "detector" ("none" for the empty phrase), csp off. Every answer
is checked: its image_sha256 must equal the dataset mosaic's and its
prompt_sha256 the sample prompt's (counted as fidelity failures otherwise:
then the server did not see what the model was trained on).

THE LOWER TILE (2026-10-24). The bottom tile is sent whatever it is: for a
prev_front dataset (tools/build_follow_dataset.py --bottom-tile prev_front)
it is the front frame dt_s earlier, in the down slot - what a prev_front
flight sends (demo/vla_mission.VLAFrames). The dataset's tile
(finetune_aerialvla_follow.dataset_bottom_tile) is recorded, and each
evaluated adapter's own (its provenance, from the identity) is checked
against it: `bottom_tile_check` is "ok" or why not, and a mismatch is
printed. Not refused: the parent adapter (upstream's, "down") on a
prev_front dataset is the baseline, labelled - its numbers are what it says
to inputs it never saw.

THE SPEED WORDS (2026-10-24). A --speed-words dataset's object words end in
the aircraft's own speed class ("a red car while flying fast"); the request
carries the sample's object as it is, so the server builds the sample's
prompt byte for byte (prompt_sha256 checked as for every sample). The
dataset's setting (finetune_aerialvla_follow.dataset_speed_words) is
recorded and each adapter's own (its provenance) checked against it
(`speed_words_check`, "ok" or why not; printed, not refused - the baseline
never saw speed words).

WHAT IS SCORED: finetune_aerialvla_follow.score_outputs - parse-ok rate,
exact-bin accuracy per axis, step MAE (m / rad), turn-sign agreement, LAND.
And the same split by the sample's hint kind (`metrics_by_hint_kind`:
"fresh", "memory", "search" - tools/build_follow_dataset.py --hint-modes;
a dataset built before 2026-10-24 records none and holds fresh hints only),
so an adapter's answers with the car out of view are scored apart.

WHICH ADAPTER ANSWERED. A running server's hello must carry the `adapter`
block (demo/vla_server_v2.adapter_identity) whose sha256 equals the digest of
--adapter's files here (guardrail.manifest.weights_digest), else the
evaluation is refused: scoring adapter X on a server that serves Y is the
mistake this check exists for. In process, the identity is built as the
server builds it.
"""
from __future__ import annotations

import argparse
import base64
import gc
import io
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "training"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import aerialvla_codec as C                           # noqa: E402
import finetune_aerialvla_follow as FT                               # noqa: E402

SCHEMA = "vlaguard.adapter_eval/1"
DEFAULT_BASELINE = FT.DEFAULT_ADAPTER
KEY_METRICS = ("parse_ok_rate", "step_mae_norm", "turn_sign_agree", "turn_sign_agree_on_turns")


def png_b64(img) -> str:
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


def _pose(s: "FT.Sample") -> dict:
    """The sample's capture pose as protocol v2's pose (echoed by the
    server, never read by the model), or zeros."""
    p = s.extra.get("pose") if isinstance(s.extra.get("pose"), dict) else {}
    out = {}
    for k, alts in (("x", ("x",)), ("y", ("y",)), ("up", ("up",)), ("yaw_rad", ("yaw_rad", "yaw"))):
        v = next((p[a] for a in alts if isinstance(p.get(a), (int, float))
                  and not isinstance(p.get(a), bool)), 0.0)
        out[k] = float(v)
    return out


def build_request(s: "FT.Sample", mosaic, seq: int) -> dict:
    """The protocol-v2 infer request for one sample (module docstring, WHAT
    IS SENT); validated against the codec before it is returned."""
    w, h = C.MOSAIC_TILE
    tc = s.extra.get("t_capture")
    req = {"v": C.PROTOCOL_VERSION, "op": "infer", "seq": int(seq),
           "t_capture": float(tc) if isinstance(tc, (int, float)) and not isinstance(tc, bool)
           else float(seq),
           "pose": _pose(s),
           "front_jpg_b64": png_b64(mosaic.crop((0, 0, w, h))),
           "down_jpg_b64": png_b64(mosaic.crop((0, h, w, 2 * h))),
           "phrase": s.phrase, "hint_source": "detector" if s.phrase else "none",
           "object": s.object, "csp_text": None, "csp_mode": "off"}
    C.validate_request(req)
    return req


def run_requests(samples: list, send: Callable[[dict], dict],
                 load_image: Callable = FT.load_mosaic, log: Callable[[str], None] = print
                 ) -> tuple[dict, list[dict], dict]:
    """Send every sample's request with `send` (a socket client's call, or
    a V2Server core's handle_request) and score the answers. Returns
    (metrics, outputs, fidelity counts)."""
    outs, fid = [], {"answered": 0, "refused": 0, "image_sha256_mismatch": 0,
                     "prompt_sha256_mismatch": 0, "errors": []}
    t0 = time.time()
    for k, s in enumerate(samples):
        mosaic = load_image(s.image, s.image_sha256)
        resp = send(build_request(s, mosaic, k + 1))
        if not resp.get("ok"):
            fid["refused"] += 1
            if len(fid["errors"]) < 10:
                fid["errors"].append(f"sample {s.idx}: {resp.get('error')}")
            raw = ""                       # no answer: scored as a parse failure (held zero)
        else:
            fid["answered"] += 1
            raw = resp.get("raw_text") or ""
            if resp.get("image_sha256") != C.mosaic_sha256(mosaic):
                fid["image_sha256_mismatch"] += 1
            if resp.get("prompt_sha256") != C.prompt_sha256(s.prompt):
                fid["prompt_sha256_mismatch"] += 1
        outs.append({"idx": s.idx, "run": s.run, "phrase": s.phrase, "target": s.target,
                     "hint_kind": hint_kind(s), "raw_text": raw})
        if (k + 1) % 50 == 0:
            log(f"[eval] {k + 1}/{len(samples)} ({time.time() - t0:.0f} s)")
    return FT.score_outputs((o["target"], o["raw_text"]) for o in outs), outs, fid


def hint_kind(s: "FT.Sample") -> str:
    """The sample's hint kind; a dataset from before --hint-modes records
    none and holds fresh hints only."""
    return s.extra.get("hint_kind") or "fresh"


def metrics_by_hint_kind(outs: list[dict]) -> dict:
    """score_outputs per hint kind ("fresh" when an output carries none),
    in the builder's order, the kinds present only."""
    groups: dict[str, list[dict]] = {}
    for o in outs:
        groups.setdefault(o.get("hint_kind") or "fresh", []).append(o)
    order = ("fresh", "memory", "search")
    kinds = [k for k in order if k in groups] + sorted(k for k in groups if k not in order)
    return {k: FT.score_outputs((o["target"], o["raw_text"]) for o in groups[k]) for k in kinds}


def local_adapter_digest(adapter: str | Path) -> str | None:
    from guardrail.manifest import weights_digest
    return weights_digest([Path(adapter)])


def check_served_adapter(identity: dict | None, adapter: str | Path,
                         digest_fn: Callable = local_adapter_digest) -> str | None:
    """Why the server's identity is NOT this adapter (None when it is): its
    `adapter.sha256` must equal the digest of `adapter`'s files here."""
    a = (identity or {}).get("adapter") if isinstance(identity, dict) else None
    if not isinstance(a, dict) or not a.get("sha256"):
        return ("the server's identity carries no adapter digest (a server from before "
                "2026-10-23, or the scripted stand-in): cannot tell which adapter answers")
    mine = digest_fn(adapter)
    if mine is None:
        return f"cannot read {adapter} here to compare its digest"
    if a["sha256"] != mine:
        return (f"the server serves adapter {a.get('path')} (sha256 {a['sha256'][:12]}...), "
                f"not {adapter} ({mine[:12]}...)")
    return None


def tile_check(identity: dict | None, dataset_tile: dict) -> str:
    """"ok", or why the adapter `identity` announces was not trained on the
    dataset's lower tile (aerialvla_codec.adapter_bottom_tile: no recorded
    tile is "down")."""
    try:
        trained, _src = C.adapter_bottom_tile(identity)
        given = C.bottom_tile_from(dataset_tile)
    except ValueError as exc:
        return f"a lower tile record is malformed ({exc})"
    why = C.bottom_tile_mismatch(trained, given)
    return "ok" if why is None else why


def speed_words_check(identity: dict | None, dataset_setting: dict | None) -> str:
    """"ok", or why the adapter `identity` announces was not trained with
    the dataset's speed words (aerialvla_codec.adapter_speed_words: no
    recorded setting is off; `dataset_setting` None is off)."""
    try:
        trained, _src = C.adapter_speed_words(identity)
        given = C.speed_words_from(dataset_setting)
    except ValueError as exc:
        return f"a speed-words record is malformed ({exc})"
    why = C.speed_words_mismatch(trained, given)
    return "ok" if why is None else why


def socket_send(addr: str):
    from vla_server_v2 import V2Client
    host, port = addr.rsplit(":", 1)
    cli = V2Client(host, int(port), timeout=120.0)
    return cli, cli.call


def evaluate_server(addr: str, adapter: str, samples: list, trust: bool = False) -> dict:
    cli, send = socket_send(addr)
    try:
        hello = send({"v": 2, "op": "hello"})
        ident = (hello or {}).get("model")
        why = check_served_adapter(ident, adapter)
        if why and not trust:
            raise SystemExit(f"[eval] refused: {addr}: {why}")
        m, outs, fid = run_requests(samples, send)
    finally:
        cli.close()
    return {"how": f"server {addr}", "identity": _ident(ident), "adapter_check": why or "ok",
            "metrics": m, "metrics_by_hint_kind": metrics_by_hint_kind(outs), "fidelity": fid,
            "outputs": outs}


def evaluate_in_process(adapter: str, samples: list, *, base: str, precision: str, attn: str,
                        merge: bool, max_new_tokens: int) -> dict:
    """Load base + adapter as demo/vla_server_v2 build_real does, serve the
    requests through V2Server's core (no socket), free the GPU after."""
    import vla_server_v2 as S
    paths = S.weight_paths(base, adapter)
    digest, _ = S.weights_sha256(paths)
    a_digest, _ = S.weights_sha256([Path(adapter)])
    model = S.AerialVLAModel(base, adapter, precision=precision, attn=attn, merge=merge,
                             max_new_tokens=max_new_tokens)
    ident = S.serving_identity(base, adapter, precision, attn, model.merged, digest,
                               adapter_info=S.adapter_identity(adapter, a_digest))
    core = S.V2Server(model, ident, log_path=None, missing_down="refuse")
    try:
        m, outs, fid = run_requests(samples, lambda r: core.handle_request(r)[0])
    finally:
        del core, model
        gc.collect()
        try:
            import torch
            torch.cuda.empty_cache()
        except Exception:                                          # noqa: BLE001
            pass
    return {"how": f"in process {precision}/{attn}/{'merged' if merge else 'unmerged'}",
            "identity": _ident(ident), "adapter_check": "ok (built here)", "metrics": m,
            "metrics_by_hint_kind": metrics_by_hint_kind(outs), "fidelity": fid,
            "outputs": outs}


def evaluate_fake(outputs: list[str], samples: list) -> dict:
    """The scripted stand-in through V2Server's core: the whole request path
    with no model. Its metrics are NOT a measurement of any adapter."""
    import vla_server_v2 as S
    core = S.V2Server(S.ScriptedModel(outputs), dict(S.FAKE_IDENTITY), missing_down="refuse")
    m, outs, fid = run_requests(samples, lambda r: core.handle_request(r)[0])
    return {"how": "scripted stand-in (NOT a model)", "identity": _ident(S.FAKE_IDENTITY),
            "adapter_check": "none: a stand-in", "metrics": m,
            "metrics_by_hint_kind": metrics_by_hint_kind(outs), "fidelity": fid, "outputs": outs}


def _ident(ident: dict | None) -> dict | None:
    if not isinstance(ident, dict):
        return None
    return {k: ident.get(k) for k in ("id", "base", "weights_sha256", "precision", "attn",
                                      "merged", "adapter")}


def delta(a: dict, b: dict) -> dict:
    """adapter minus baseline on the key metrics and each axis's step MAE."""
    out = {}
    for k in KEY_METRICS:
        if a.get(k) is not None and b.get(k) is not None:
            out[k] = round(a[k] - b[k], 6)
    for ax in C.AXES:
        x, y = (a.get("step_mae") or {}).get(ax), (b.get("step_mae") or {}).get(ax)
        if x is not None and y is not None:
            out[f"step_mae.{ax}"] = round(x - y, 6)
        x, y = (a.get("exact_bin_rate") or {}).get(ax), (b.get("exact_bin_rate") or {}).get(ax)
        if x is not None and y is not None:
            out[f"exact_bin_rate.{ax}"] = round(x - y, 6)
    return out


def table(res: dict) -> str:
    cols = [("adapter", res["adapter"]["metrics"])]
    if res.get("baseline"):
        cols.append(("baseline", res["baseline"]["metrics"]))
    rows = [("n", lambda m: m["n"]), ("parse_ok_rate", lambda m: m["parse_ok_rate"])]
    for ax in C.AXES:
        rows.append((f"exact bin {ax}", lambda m, ax=ax: m["exact_bin_rate"][ax]))
    rows.append(("exact bin all three", lambda m: m["exact_bin_rate"]["all"]))
    for ax in C.AXES:
        rows.append((f"step MAE {ax}", lambda m, ax=ax: m["step_mae"][ax]))
    rows += [("step_mae_norm", lambda m: m["step_mae_norm"]),
             ("turn-sign agree", lambda m: m["turn_sign_agree"]),
             ("turn-sign agree on turns", lambda m: m["turn_sign_agree_on_turns"]),
             ("LAND predicted", lambda m: m["land_pred_rate"])]
    w = max(len(r[0]) for r in rows)
    lines = [f"{'metric'.ljust(w)} | " + " | ".join(c[0] for c in cols)]
    for name, fn in rows:
        lines.append(f"{name.ljust(w)} | " + " | ".join(str(fn(m)) for _, m in cols))
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Held-out metrics of an AerialVLA adapter")
    ap.add_argument("--data", required=True)
    ap.add_argument("--adapter", required=True)
    how = ap.add_mutually_exclusive_group(required=True)
    how.add_argument("--server", metavar="HOST:PORT", help="a server serving --adapter")
    how.add_argument("--in-process", action="store_true")
    how.add_argument("--fake-output", nargs="+", default=None,
                     help="the scripted stand-in (no model; NOT a measurement)")
    ap.add_argument("--baseline", default=DEFAULT_BASELINE,
                    help="in process: the baseline adapter (default the original aero_vla)")
    ap.add_argument("--baseline-server", metavar="HOST:PORT", default=None,
                    help="a server serving the baseline adapter")
    ap.add_argument("--no-baseline", action="store_true")
    ap.add_argument("--split", nargs="+", default=list(FT.HOLDOUT_SPLITS),
                    help="which samples (default the held-out splits)")
    ap.add_argument("--max", type=int, default=0, help="at most this many (a seeded subset)")
    ap.add_argument("--seed", type=int, default=20261023)
    ap.add_argument("--base", default=FT.DEFAULT_BASE)
    ap.add_argument("--precision", choices=("bf16", "nf4"), default=None)
    ap.add_argument("--attn", choices=("sdpa", "eager"), default=None)
    ap.add_argument("--merge", action=argparse.BooleanOptionalAction, default=None)
    ap.add_argument("--max-new-tokens", type=int, default=12)
    ap.add_argument("--trust-server", action="store_true",
                    help="score a server whose identity cannot be matched to --adapter "
                         "(recorded as unchecked)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--save-outputs", action="store_true", help="keep every answer in --out")
    args = ap.parse_args(argv)
    if args.in_process and None in (args.precision, args.attn, args.merge):
        ap.error("--in-process needs --precision, --attn and --merge/--no-merge (no default: "
                 "the precision changes what the model says, demo/vla_server_v2.py)")
    samples, report = FT.load_samples(Path(args.data))
    try:
        dtile = FT.dataset_bottom_tile(Path(args.data), samples)
        dsw = FT.dataset_speed_words(Path(args.data), samples)
    except FT.DataError as exc:
        raise SystemExit(f"[eval] refused: {exc}") from None
    split = set(args.split)
    chosen = [s for s in samples if s.split in split]
    chosen = FT.heldout_subset(chosen, args.max, args.seed)
    if not chosen:
        raise SystemExit(f"[eval] no samples in splits {sorted(split)}")
    print(f"[eval] {len(chosen)} samples from {sorted({s.run for s in chosen})}", flush=True)
    if args.fake_output:
        res_a = evaluate_fake(args.fake_output, chosen)
    elif args.in_process:
        res_a = evaluate_in_process(args.adapter, chosen, base=args.base,
                                    precision=args.precision, attn=args.attn, merge=args.merge,
                                    max_new_tokens=args.max_new_tokens)
    else:
        res_a = evaluate_server(args.server, args.adapter, chosen, trust=args.trust_server)
    res_b = None
    if not args.no_baseline:
        if args.baseline_server:
            res_b = evaluate_server(args.baseline_server, args.baseline, chosen,
                                    trust=args.trust_server)
        elif args.in_process:
            res_b = evaluate_in_process(args.baseline, chosen, base=args.base,
                                        precision=args.precision, attn=args.attn,
                                        merge=args.merge, max_new_tokens=args.max_new_tokens)
        else:
            print("[eval] no baseline: give --baseline-server, or --in-process", flush=True)
    for name, r in (("adapter", res_a), ("baseline", res_b)):
        if r is not None:
            r["bottom_tile_check"] = tile_check(r.get("identity"), dtile)
            if r["bottom_tile_check"] != "ok":
                print(f"[eval] *** {name}: {r['bottom_tile_check']} (the dataset's lower tile "
                      f"is {dtile['mode']}): its numbers are for inputs it never saw",
                      flush=True)
            r["speed_words_check"] = speed_words_check(r.get("identity"), dsw["setting"])
            if r["speed_words_check"] != "ok":
                print(f"[eval] *** {name}: {r['speed_words_check']}: its numbers are for "
                      f"prompts it never saw", flush=True)
    res = {"schema": SCHEMA,
           "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
           "data": report, "bottom_tile": dtile, "speed_words": dsw, "splits": sorted(split),
           "n": len(chosen),
           "adapter_path": Path(args.adapter).as_posix(), "adapter": res_a,
           "baseline_path": None if res_b is None else Path(args.baseline).as_posix(),
           "baseline": res_b,
           "delta_adapter_minus_baseline": (None if res_b is None else
                                            delta(res_a["metrics"], res_b["metrics"])),
           "command": "python tools/eval_vla_adapter.py " + " ".join(
               argv if argv is not None else sys.argv[1:])}
    if not args.save_outputs:
        for r in (res_a, res_b):
            if r is not None:
                r.pop("outputs", None)
    print(table(res))
    for name, r in (("adapter", res_a), ("baseline", res_b)):
        if r is not None and len(r["metrics_by_hint_kind"]) > 1:
            print(f"[eval] {name} by hint kind: " + "; ".join(
                f"{k} n {m['n']}, exact all {m['exact_bin_rate']['all']}, step_mae_norm "
                f"{m['step_mae_norm']}, turn-sign agree {m['turn_sign_agree']}"
                for k, m in r["metrics_by_hint_kind"].items()))
    for name, r in (("adapter", res_a), ("baseline", res_b)):
        if r is not None:
            f = r["fidelity"]
            print(f"[eval] {name}: {r['how']}, id {(r['identity'] or {}).get('id')}; answered "
                  f"{f['answered']}, refused {f['refused']}, image/prompt sha mismatches "
                  f"{f['image_sha256_mismatch']}/{f['prompt_sha256_mismatch']}")
    if args.out:
        Path(args.out).write_text(json.dumps(res, indent=1, default=str), encoding="utf-8")
        print(f"[eval] -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
