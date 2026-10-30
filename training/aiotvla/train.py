"""Behaviour cloning of AIOT-VLA's head on the follow expert's recordings.

    # labels: the expert's motion over 0.5 s (tools/build_follow_dataset.py,
    # --no-images: this trainer reads its own cache, not the mosaics)
    python tools/build_follow_dataset.py <runs> --out D:/vla_drone_offload/aiotvla/labels_v0 \\
        --no-images --horizon-s 0.5 --stride-s 0.1 --hint-modes fresh,memory,search \\
        --holdout ft_expert_03 ft_dagger_pf_5
    # backbone maps: python -m training.aiotvla.build_cache <runs> --cache <cache>
    python -m training.aiotvla.train --labels D:/vla_drone_offload/aiotvla/labels_v0 \\
        --cache D:/vla_drone_offload/aiotvla/cache --out D:/models/aiotvla/v0

A sample is one labelled capture of build_follow_dataset (its `label_raw`,
the expert's clean motion; DART and DAgger windows already dropped there).
Its input is the cache's maps of the capture and of its history captures
(core.history_indices), the run's object phrase's grounding channel, and
core.proprio of those captures. Half the training samples of every batch
are mirrored (core MIRROR: the flipped frame's own cached maps, mirrored
proprio and label). The hold-out runs are never mirrored and never trained on.

Writes <out>/head.pt (state dict, HeadConfig, the PCA, VOCAB, the core
constants it was trained with, the data it saw), <out>/metrics.json (every
epoch's hold-out numbers next to a constant baseline) and <out>/loss.csv.
The checkpoint kept is the epoch with the lowest hold-out dpsi MAE.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from training.aiotvla import core                                   # noqa: E402

AXIS_WEIGHT = {"fwd_m": 1.0, "down_m": 0.5, "dpsi_rad": 2.0}
# The proprio fields that carry the aircraft's own recent motion. --motion-
# dropout zeroes them on a share of training samples: v0 (no dropout) read
# its own yaw rate and speed so well that, with them zeroed on the hold-out,
# fwd MAE rose 0.07 -> 1.12 m - a "copycat" that repeats its last motion and
# would not start a turn the frame calls for.
MOTION_FIELDS = ("rel_fwd", "rel_right", "rel_up", "rel_yaw", "v_fwd", "v_right", "v_up",
                 "yaw_rate")
AUX_WEIGHT = {"visible": 0.3, "bearing": 0.3}
BEARING_SCALE = 0.8                     # rad


def torch_grounding_stats(torch, g):
    """core.grounding_stats for a batch: g (N, 576) -> (N, 4)."""
    s = core.GRID_SIDE
    z = g.float().reshape(-1, s * s)
    e = torch.softmax(z, dim=1).reshape(-1, s, s)
    lin = torch.linspace(-1.0, 1.0, s, device=g.device)
    col = (e.sum(1) * lin).sum(-1)
    row = (e.sum(2) * lin).sum(-1)
    top = z.topk(2, dim=1).values
    return torch.stack([col, row, top[:, 0], top[:, 0] - top[:, 1]], dim=1)


class Data:
    """Every run's maps in one table (row n = a zero frame for empty slots)
    and every sample's history rows, proprio and targets."""

    def __init__(self, labels: Path, cache: Path, device: str, offsets=core.HIST_OFFSETS_S,
                 runs: list[str] | None = None):
        import torch
        self.torch = torch
        samples = [json.loads(l) for l in (labels / "samples.jsonl").read_text(
            encoding="utf-8").splitlines() if l.strip()]
        if runs is not None:
            keep = set(runs)
            samples = [s for s in samples if s["run"] in keep]
        runs = sorted({s["run"] for s in samples})
        G, Gm, A, Am = [], [], [], []
        offset, self.run_info = 0, {}
        rows_of = {}
        meta_of = {}
        for r in runs:
            d = cache / r
            info = json.loads((d / "info.json").read_text(encoding="utf-8"))
            vi = core.VOCAB.index(info["object"])
            meta = np.load(d / "meta.npz")
            G.append(np.load(d / "G.npy", mmap_mode="r")[:, vi, :].astype(np.float16))
            Gm.append(np.load(d / "Gm.npy", mmap_mode="r")[:, vi, :].astype(np.float16))
            A.append(np.asarray(np.load(d / "A.npy", mmap_mode="r")))
            Am.append(np.asarray(np.load(d / "Am.npy", mmap_mode="r")))
            rows_of[r] = {int(i): k for k, i in enumerate(meta["i"])}
            meta_of[r] = meta
            self.run_info[r] = {"offset": offset, "n": int(info["n"]), "object": info["object"],
                                "pca_sha256": info["pca_sha256"],
                                "front_source": info.get("front_source", "vla"),
                                "front_wh": info.get("front_wh")}
            offset += int(info["n"])
        zero_g = np.zeros((1, core.N_PATCHES), np.float16)
        zero_a = np.zeros((1, core.N_PATCHES, core.PCA_DIM), np.float16)
        self.G = torch.from_numpy(np.concatenate(G + [zero_g])).to(device)
        self.Gm = torch.from_numpy(np.concatenate(Gm + [zero_g])).to(device)
        self.A = torch.from_numpy(np.concatenate(A + [zero_a])).to(device)
        self.Am = torch.from_numpy(np.concatenate(Am + [zero_a])).to(device)
        self.zero_row = offset
        hist, pr, prm, tgt, aux, split, w, lab, ids = [], [], [], [], [], [], [], [], []
        skipped = 0
        for s in samples:
            r = s["run"]
            meta = meta_of[r]
            k = rows_of[r].get(int(s["i"]))
            ok = meta["frame_ok"]
            if k is None or not ok[k]:
                skipped += 1
                continue
            idx = [j if (j >= 0 and ok[j]) else -1
                   for j in core.history_indices(meta["t"], k, offsets)]
            p = core.proprio(meta["poses"], meta["vel"], meta["yaw_rate"], idx)
            off = self.run_info[r]["offset"]
            hist.append([off + j if j >= 0 else self.zero_row for j in idx])
            pr.append(p)
            prm.append(core.mirror_proprio(p))
            L = s["label_raw"]
            lab.append([L["fwd_m"], L["down_m"], L["dpsi_rad"]])
            vis = s.get("hint_kind") == "fresh" and s.get("box") is not None
            brg = math.radians(s["box"]["bearing_deg"]) if vis else 0.0
            aux.append([1.0 if vis else 0.0, brg])
            split.append(s["split"] == "train")
            w.append(1.0 + 2.0 * min(abs(L["dpsi_rad"]) / 0.15, 1.0))
            ids.append(s["id"])
        self.skipped = skipped
        self.hist = torch.tensor(hist, dtype=torch.long, device=device)
        self.pr = torch.from_numpy(np.stack(pr)).to(device)
        self.prm = torch.from_numpy(np.stack(prm)).to(device)
        self.lab = torch.tensor(lab, dtype=torch.float32, device=device)
        self.aux = torch.tensor(aux, dtype=torch.float32, device=device)
        self.train = torch.tensor(split, dtype=torch.bool, device=device)
        self.w = torch.tensor(w, dtype=torch.float32, device=device)
        self.ids = ids
        self.grids = {ax: torch.tensor(core.GRIDS[ax], dtype=torch.float32, device=device)
                      for ax in core.AXES}
        self.samples_sha256 = hashlib.sha256((labels / "samples.jsonl").read_bytes()).hexdigest()

    def soft(self, values, ax):
        torch = self.torch
        g = self.grids[ax]
        cell = g[1] - g[0]
        v = values.clamp(g[0], g[-1]).unsqueeze(1)
        w = torch.exp(-0.5 * ((g.unsqueeze(0) - v) / (core.SOFT_SIGMA * cell)) ** 2)
        return w / w.sum(1, keepdim=True)

    def batch(self, idx, mirror):
        """Inputs and targets of samples idx; mirror (bool tensor) per sample."""
        torch = self.torch
        h = self.hist[idx]
        m3 = mirror.view(-1, 1, 1)
        g = torch.where(m3, self.Gm[h], self.G[h]).float()
        a = torch.where(mirror.view(-1, 1, 1, 1), self.Am[h], self.A[h]).float()
        b, k = h.shape
        st = torch_grounding_stats(torch, g.reshape(b * k, -1)).reshape(b, k, 4)
        pr = torch.where(m3, self.prm[idx], self.pr[idx])
        valid = h != self.zero_row
        lab = self.lab[idx].clone()
        lab[:, 2] = torch.where(mirror, -lab[:, 2], lab[:, 2])
        aux = self.aux[idx].clone()
        aux[:, 1] = torch.where(mirror, -aux[:, 1], aux[:, 1])
        return (g, a, st, pr, valid), lab, aux


def decode_batch(torch, logits: dict, grids: dict, window: int = core.DECODE_WINDOW):
    """core.decode for a batch: {axis: (B, n)} -> (B, 3) in AXES order."""
    out = []
    for ax in core.AXES:
        p = torch.softmax(logits[ax].float(), dim=1)
        g = grids[ax]
        m = p.argmax(1)
        n = p.shape[1]
        ar = torch.arange(n, device=p.device).unsqueeze(0)
        half = torch.minimum(torch.minimum(m, n - 1 - m), torch.full_like(m, window))
        win = (ar - m.unsqueeze(1)).abs() <= half.unsqueeze(1)
        pw = p * win
        out.append((pw * g).sum(1) / pw.sum(1).clamp_min(1e-12))
    return torch.stack(out, dim=1)


def evaluate(head, data: Data, idx, bs: int = 1024) -> dict:
    torch = data.torch
    head.eval()
    preds, labs, vis_p, aux_t = [], [], [], []
    with torch.no_grad():
        for k in range(0, len(idx), bs):
            j = idx[k:k + bs]
            x, lab, aux = data.batch(j, torch.zeros(len(j), dtype=torch.bool, device=j.device))
            logits, ax = head(*x)
            preds.append(decode_batch(torch, logits, data.grids))
            labs.append(lab)
            vis_p.append(torch.sigmoid(ax[:, 0]))
            aux_t.append(aux)
    p, l = torch.cat(preds), torch.cat(labs)
    vp, at = torch.cat(vis_p), torch.cat(aux_t)
    out = {"n": int(len(p))}
    for c, ax in enumerate(core.AXES):
        out[f"mae_{ax}"] = float((p[:, c] - l[:, c]).abs().mean())
    turn = l[:, 2].abs() > 0.05
    out["dpsi_sign_acc_turning"] = float(((p[turn, 2] > 0) == (l[turn, 2] > 0)).float().mean()) \
        if bool(turn.any()) else None
    out["n_turning"] = int(turn.sum())
    out["dpsi_corr"] = float(torch.corrcoef(torch.stack([p[:, 2], l[:, 2]]))[0, 1])
    out["visible_acc"] = float(((vp > 0.5) == (at[:, 0] > 0.5)).float().mean())
    head.train()
    return out


def baseline(data: Data, tr_idx, ho_idx) -> dict:
    """The constant answer: the training labels' median per axis."""
    med = data.lab[tr_idx].median(dim=0).values
    l = data.lab[ho_idx]
    return {f"mae_{ax}": float((l[:, c] - med[c]).abs().mean()) for c, ax in enumerate(core.AXES)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--labels", required=True)
    ap.add_argument("--cache", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=12)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--wd", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=20261029)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--init", default=None, help="a head.pt to start from (a DAgger round)")
    ap.add_argument("--long-history", action="store_true",
                    help="core.LONG_HIST_OFFSETS_S (10 slots, 5.5 s) instead of the 8-slot 2.5 s")
    ap.add_argument("--runs", nargs="*", default=None,
                    help="train and hold out on these runs only (default every run labelled)")
    ap.add_argument("--motion-dropout", type=float, default=0.0,
                    help="share of training samples whose motion proprio (MOTION_FIELDS, every "
                         "slot) is zeroed")
    args = ap.parse_args(argv)
    import torch
    from training.aiotvla.model import HeadConfig, AIOTVLAHead, n_params
    torch.manual_seed(args.seed)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    offsets = core.LONG_HIST_OFFSETS_S if args.long_history else core.HIST_OFFSETS_S
    data = Data(Path(args.labels), Path(args.cache), args.device, offsets, args.runs)
    backbones = {json.loads((Path(args.cache) / r / "info.json").read_text(
        encoding="utf-8"))["backbone"] for r in data.run_info}
    if len(backbones) != 1:
        raise SystemExit(f"the runs were cached with {sorted(backbones)}: one backbone only")
    backbone = next(iter(backbones))
    pcas = {v["pca_sha256"] for v in data.run_info.values()}
    if len(pcas) != 1:
        raise SystemExit(f"the runs were cached with {len(pcas)} different PCAs: re-cache them")
    fronts = {(v["front_source"], tuple(v["front_wh"] or ())) for v in data.run_info.values()}
    if len(fronts) != 1:
        raise SystemExit(f"the runs were recorded with different front cameras {sorted(fronts)}: "
                         f"train on one (--runs)")
    front_source, front_wh = next(iter(fronts))
    tr = torch.nonzero(data.train).squeeze(1)
    ho = torch.nonzero(~data.train).squeeze(1)
    print(f"[data] {len(tr)} train / {len(ho)} hold-out samples, {len(data.run_info)} runs, "
          f"{data.skipped} skipped, loaded in {time.time() - t0:.1f} s")
    cfg = HeadConfig(n_slots=len(offsets))
    head = AIOTVLAHead(cfg).to(args.device)
    if args.init:
        ck = torch.load(args.init, map_location=args.device)
        head.load_state_dict(ck["state_dict"])
    print(f"[model] {n_params(head):,} trainable parameters")
    opt = torch.optim.AdamW(head.parameters(), lr=args.lr, weight_decay=args.wd)
    steps = args.epochs * math.ceil(len(tr) / args.batch)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=args.lr, total_steps=steps,
                                                pct_start=0.1)
    base = baseline(data, tr, ho)
    print(f"[baseline] {base}")
    gen = torch.Generator(device="cpu").manual_seed(args.seed)
    motion_cols = torch.tensor([core.PROPRIO_FIELDS.index(f) for f in MOTION_FIELDS],
                               device=args.device)
    hist, best, step = [], None, 0
    loss_f = open(out / "loss.csv", "w", newline="")
    lw = csv.writer(loss_f)
    lw.writerow(["step", "epoch", "loss", "ce_fwd", "ce_down", "ce_dpsi", "aux_vis", "aux_brg",
                 "lr"])
    for ep in range(args.epochs):
        perm = tr[torch.randperm(len(tr), generator=gen).to(tr.device)]
        for k in range(0, len(perm), args.batch):
            j = perm[k:k + args.batch]
            mirror = torch.rand(len(j), generator=gen).to(j.device) < 0.5
            x, lab, aux = data.batch(j, mirror)
            if args.motion_dropout > 0:
                drop = torch.rand(len(j), generator=gen).to(j.device) < args.motion_dropout
                pr = x[3].clone()
                pr[drop.nonzero().squeeze(1).unsqueeze(1), :, motion_cols] = 0.0
                x = (x[0], x[1], x[2], pr, x[4])
            logits, ax = head(*x)
            w = data.w[j]
            parts = {}
            loss = 0.0
            for c, a in enumerate(core.AXES):
                tgt = data.soft(lab[:, c], a)
                ce = -(tgt * torch.log_softmax(logits[a], dim=1)).sum(1)
                parts[a] = (ce * w).sum() / w.sum()
                loss = loss + AXIS_WEIGHT[a] * parts[a]
            vis = torch.nn.functional.binary_cross_entropy_with_logits(ax[:, 0], aux[:, 0])
            m = aux[:, 0] > 0.5
            brg = (torch.nn.functional.smooth_l1_loss(ax[m, 1], aux[m, 1] / BEARING_SCALE)
                   if bool(m.any()) else ax.sum() * 0.0)
            loss = loss + AUX_WEIGHT["visible"] * vis + AUX_WEIGHT["bearing"] * brg
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(head.parameters(), 1.0)
            opt.step()
            sched.step()
            step += 1
            if step % 50 == 0:
                lw.writerow([step, ep, round(float(loss), 5)] +
                            [round(float(parts[a]), 5) for a in core.AXES] +
                            [round(float(vis), 5), round(float(brg), 5),
                             sched.get_last_lr()[0]])
        ev = evaluate(head, data, ho)
        ev["epoch"] = ep
        hist.append(ev)
        print(f"[epoch {ep}] hold-out {json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in ev.items()})}")
        if best is None or ev["mae_dpsi_rad"] < best["mae_dpsi_rad"]:
            best = ev
            pca = np.load(Path(args.cache) / "pca.npz")
            torch.save({"state_dict": head.state_dict(), "config": cfg.to_dict(),
                        "pca_mean": pca["mean"], "pca_comps": pca["comps"],
                        "pca_sha256": next(iter(pcas)),
                        "front_source": front_source, "front_wh": list(front_wh),
                        "core": {"HIST_OFFSETS_S": tuple(offsets),
                                 "HIST_TOL_S": core.HIST_TOL_S, "VEL_DT_S": core.VEL_DT_S,
                                 "VEL_MAX_DT_S": core.VEL_MAX_DT_S, "PCA_DIM": core.PCA_DIM,
                                 "VOCAB": core.VOCAB,
                                 "GRIDS": {a: [float(core.GRIDS[a][0]), float(core.GRIDS[a][-1]),
                                               len(core.GRIDS[a])] for a in core.AXES},
                                 "LABEL_HORIZON_S": core.LABEL_HORIZON_S},
                        "backbone": backbone,
                        "data": {"labels": Path(args.labels).as_posix(),
                                 "samples_sha256": data.samples_sha256,
                                 "runs": data.run_info, "n_train": int(len(tr)),
                                 "n_holdout": int(len(ho))},
                        "epoch": ep, "holdout": ev}, out / "head.pt")
    loss_f.close()
    res = {"baseline_constant": base, "epochs": hist, "best": best,
           "train_seconds": round(time.time() - t0, 1), "args": vars(args)}
    (out / "metrics.json").write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8")
    print(f"[done] best epoch {best['epoch']}: {best}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
