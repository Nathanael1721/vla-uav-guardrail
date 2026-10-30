"""Backbone maps for every capture of the follow expert's recordings.

    # the vla-real env (torch, transformers 4.40.1); OWL-ViT from the HF cache
    python -m training.aiotvla.build_cache demo/out/ft_expert_01 demo/out/ft_dagger_01 \\
        --cache D:/vla_drone_offload/aiotvla/cache

Each RUN_DIR is a flight folder holding vla_record/ (follow_vlm --record-vla)
or the recording folder itself. Per run, <cache>/<tag>/ gets
  G.npy  Gm.npy   (N, len(core.VOCAB), 576) float16: grounding logits of
                  every VOCAB phrase, of the frame and of the frame flipped
                  left-right (the mirror is the flipped frame's own maps)
  A.npy  Am.npy   (N, 576, PCA_DIM) float16: the appearance maps
  meta.npz        t (capture time), poses (x, y, up, yaw), vel, yaw_rate,
                  vel_ok (core.velocities), frame_ok (False where the
                  recording lost the JPEG: its maps stay zero and the trainer
                  treats it as an empty slot), i (capture index), the front
                  frame's sha256
  info.json       tag, object, N, backbone, vocab, the PCA's sha256, timing
The PCA (<cache>/pca.npz) is fitted once, from frames spread over the runs
given, before any run is cached (--pca-frames); every later run is projected
on the same components, and the model checkpoint carries a copy.
A run already cached with the same PCA is skipped unless --force.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from training.aiotvla import core                                   # noqa: E402


def record_dir(run: Path) -> Path:
    r = Path(run)
    if (r / "captures.jsonl").exists():
        return r
    if (r / "vla_record" / "captures.jsonl").exists():
        return r / "vla_record"
    raise SystemExit(f"{run}: no captures.jsonl (a follow_vlm --record-vla recording)")


def run_tag(rec: Path) -> str:
    info = rec / "record.json"
    if info.exists():
        return json.loads(info.read_text(encoding="utf-8"))["tag"]
    return rec.parent.name if rec.name == "vla_record" else rec.name


def load_captures(rec: Path) -> list[dict]:
    caps = [json.loads(l) for l in (rec / "captures.jsonl").read_text(encoding="utf-8").splitlines()
            if l.strip()]
    caps.sort(key=lambda c: c["t_capture"])
    return caps


def file_sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def fit_and_save_pca(bb, recs: list[Path], out: Path, n_frames: int) -> None:
    from training.aiotvla.backbone import fit_pca
    frames = []
    for rec in recs:
        caps = load_captures(rec)
        step = max(1, len(caps) // max(1, n_frames // len(recs)))
        frames += [p for p in (rec / "frames" / (c["front_jpg_sha256"] + ".jpg")
                               for c in caps[::step]) if p.exists()]
    rng = np.random.default_rng(20261029)
    xs = []
    for k in range(0, len(frames), 32):
        ims = [_decode(p) for p in frames[k:k + 32]]
        _, _, cls = bb.maps(ims, list(core.VOCAB[:1]), with_class=True)
        cls = cls.reshape(-1, cls.shape[-1])
        xs.append(cls[rng.choice(len(cls), size=min(len(cls), 4000), replace=False)])
    x = np.concatenate(xs)
    mean, comps, evr = fit_pca(x, core.PCA_DIM)
    np.savez(out, mean=mean, comps=comps, explained=evr, n_vectors=len(x), n_frames=len(frames))
    print(f"[pca] {len(frames)} frames, {len(x)} patch vectors -> {core.PCA_DIM} components, "
          f"explained {float(evr.sum()):.3f}")


def _decode(p: Path) -> np.ndarray | None:
    """The frame's RGB pixels, or None when the recording lost the file (a
    few hundred of the 2026-10 recordings: build_follow_dataset's
    frame_missing)."""
    from PIL import Image
    try:
        with Image.open(p) as im:
            return np.asarray(im.convert("RGB"))
    except FileNotFoundError:
        return None


def cache_run(bb, rec: Path, out: Path, pca_sha: str, batch: int) -> dict:
    from concurrent.futures import ThreadPoolExecutor
    caps = load_captures(rec)
    n = len(caps)
    out.mkdir(parents=True, exist_ok=True)
    V, P = len(core.VOCAB), core.PCA_DIM
    arrs = {k: np.lib.format.open_memmap(out / f"{k}.npy", mode="w+", dtype=np.float16,
                                         shape=(n, V, core.N_PATCHES) if k.startswith("G")
                                         else (n, core.N_PATCHES, P))
            for k in ("G", "Gm", "A", "Am")}
    t0 = time.time()
    paths = [rec / "frames" / (c["front_jpg_sha256"] + ".jpg") for c in caps]
    pool = ThreadPoolExecutor(max_workers=8)
    frame_ok = np.zeros(n, dtype=bool)
    nxt = pool.map(_decode, paths[0:batch])
    for k in range(0, n, batch):
        got = list(nxt)
        if k + batch < n:
            nxt = pool.map(_decode, paths[k + batch:k + 2 * batch])
        rows = [k + j for j, im in enumerate(got) if im is not None]
        ims = [im for im in got if im is not None]
        frame_ok[rows] = True
        if not ims:
            continue
        g, a = bb.maps(ims, list(core.VOCAB))
        gm, am = bb.maps(ims, list(core.VOCAB), flip=True)
        arrs["G"][rows] = g
        arrs["A"][rows] = a
        arrs["Gm"][rows] = gm
        arrs["Am"][rows] = am
    pool.shutdown()
    for a in arrs.values():
        a.flush()
    secs = time.time() - t0
    t = np.array([c["t_capture"] for c in caps], dtype=np.float64)
    poses = np.array([[c["pose"]["x"], c["pose"]["y"], c["pose"]["up"], c["pose"]["yaw_rad"]]
                      for c in caps], dtype=np.float64)
    vel, yr, ok = core.velocities(t, poses)
    np.savez(out / "meta.npz", t=t, poses=poses, vel=vel, yaw_rate=yr, vel_ok=ok,
             frame_ok=frame_ok, i=np.array([c["i"] for c in caps]),
             front_sha=np.array([c["front_jpg_sha256"] for c in caps]))
    rj = rec / "record.json"
    front_source = (json.loads(rj.read_text(encoding="utf-8")).get("front_source", "vla")
                    if rj.exists() else "vla")
    info = {"tag": run_tag(rec), "object": caps[0].get("object") if caps else None, "n": n,
            "front_source": front_source,
            "front_wh": caps[0].get("front_wh") if caps else None,
            "frames_missing": int(n - frame_ok.sum()),
            "backbone": bb.model_id, "dtype": str(bb.dtype), "vocab": list(core.VOCAB),
            "pca_sha256": pca_sha, "seconds": round(secs, 1),
            "ms_per_frame_pair": round(1000 * secs / max(1, n), 2), "record": rec.as_posix()}
    (out / "info.json").write_text(json.dumps(info, indent=1) + "\n", encoding="utf-8")
    return info


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="+", metavar="RUN_DIR")
    ap.add_argument("--cache", required=True)
    ap.add_argument("--pca-frames", type=int, default=400)
    ap.add_argument("--batch", type=int, default=48)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--backbone", default="google/owlvit-base-patch32",
                    help="google/owlvit-base-patch32 (default) or "
                         "google/owlv2-base-patch16-ensemble (training/aiotvla/backbone.py)")
    args = ap.parse_args(argv)
    from training.aiotvla.backbone import OwlBackbone
    root = Path(args.cache)
    root.mkdir(parents=True, exist_ok=True)
    recs = [record_dir(Path(r)) for r in args.runs]
    bb = OwlBackbone(args.backbone, device=args.device)
    pca = root / "pca.npz"
    if not pca.exists():
        fit_and_save_pca(bb, recs, pca, args.pca_frames)
    z = np.load(pca)
    bb.set_pca(z["mean"], z["comps"])
    pca_sha = file_sha(pca)
    for rec in recs:
        tag = run_tag(rec)
        out = root / tag
        info_p = out / "info.json"
        if info_p.exists() and not args.force:
            old = json.loads(info_p.read_text(encoding="utf-8"))
            if old.get("pca_sha256") == pca_sha:
                print(f"[skip] {tag}: cached ({old['n']} captures)")
                continue
        info = cache_run(bb, rec, out, pca_sha, args.batch)
        print(f"[cache] {tag}: {info['n']} captures, {info['seconds']} s "
              f"({info['ms_per_frame_pair']} ms per frame + mirror)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
