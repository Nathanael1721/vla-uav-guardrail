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

v7 (core LATERAL, MEMORY): --lateral also learns right_m (the label's
sideways component); --memory feeds the target memory's MEM_FIELDS of the
newest capture (core.run_memory over each run from its first sample, with
the phrase's box, logit and CONTRAST), zeroed on --memory-dropout of the
training samples so the head still reads the frames. v8: --text feeds the
run's object words as the backbone's text embedding (<cache>/text.npz,
build_cache), unit-norm, one more token.
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

AXIS_WEIGHT = {"fwd_m": 1.0, "down_m": 0.5, "dpsi_rad": 2.0, "right_m": 1.0}
# the axes the mirror negates (core MIRROR)
MIRROR_AXES = ("dpsi_rad", "right_m")
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


def memory_disagrees(m: np.ndarray, pose, est_xy, max_m: float) -> bool:
    """Is a valid memory (core.MEM_FIELDS row) more than max_m from the
    expert's own estimate `est_xy` (world x, y)? The memory is in the body
    frame of `pose` (x, y, up, yaw)."""
    if m[core.MEM_FIELDS.index("valid")] < 0.5:
        return False
    px, py, _, yaw = (float(v) for v in pose)
    fw = float(m[core.MEM_FIELDS.index("fwd")]) * 30.0
    rt = float(m[core.MEM_FIELDS.index("right")]) * 30.0
    wx = px + fw * math.cos(yaw) - rt * math.sin(yaw)
    wy = py + fw * math.sin(yaw) + rt * math.cos(yaw)
    return math.hypot(wx - est_xy[0], wy - est_xy[1]) > max_m


def run_memory_of(d: Path, info: dict, meta, vi: int, start: int) -> np.ndarray:
    """core.run_memory over one cached run (`d`) for its vocab's phrase vi
    (info.json "vocab"): its box (B.npy), best-patch logit and contrast
    (G.npy, core.CONTRAST), from row `start`."""
    if not (d / "B.npy").exists():
        raise SystemExit(f"{d}: no B.npy - re-cache the run (build_cache --force)")
    src = info.get("front_source", "vla")
    if src not in core.CAMERAS or not info.get("front_wh"):
        raise SystemExit(f"{d}: the memory needs a known camera; this run's is {src!r}")
    hfov, pitch = core.CAMERAS[src]
    w, h = info["front_wh"]
    G = np.load(d / "G.npy", mmap_mode="r")
    vocab = info.get("vocab", list(core.VOCAB))
    miss = [p for p in core.CONTRAST.get(vocab[vi], ()) if p not in vocab]
    if miss:
        raise SystemExit(f"{d}: contrast phrases {miss} are not cached - re-cache the run "
                         f"(build_cache --force) with this VOCAB")
    neg = tuple(vocab.index(p) for p in core.CONTRAST.get(vocab[vi], ()))
    logits = np.asarray(G[:, vi, :], dtype=np.float32).max(axis=1)
    con = None
    if neg:
        g = np.asarray(G[:, (vi,) + neg, :], dtype=np.float32)
        con = np.array([core.grounding_contrast(g[k], 0, tuple(range(1, len(neg) + 1)))
                        for k in range(len(g))])
    boxes = np.asarray(np.load(d / "B.npy", mmap_mode="r")[:, vi, :], dtype=np.float32)
    return core.run_memory(meta["t"], meta["poses"], boxes, logits, meta["frame_ok"], start,
                           h / w, hfov, pitch, con)


class Data:
    """Every run's maps in one table (row n = a zero frame for empty slots)
    and every sample's history rows, proprio and targets."""

    def __init__(self, labels: Path, cache: Path, device: str, offsets=core.HIST_OFFSETS_S,
                 runs: list[str] | None = None, onpolicy_weight: float = 1.0,
                 boxes: bool = False, axes=core.AXES, memory: bool = False,
                 text: bool = False, memory_agree_m: float = 0.0):
        import torch
        self.torch = torch
        self.axes = tuple(axes)
        samples = [json.loads(l) for l in (labels / "samples.jsonl").read_text(
            encoding="utf-8").splitlines() if l.strip()]
        if runs is not None:
            keep = set(runs)
            samples = [s for s in samples if s["run"] in keep]
        runs = sorted({s["run"] for s in samples})
        first_i = {}
        for s in samples:
            first_i[s["run"]] = min(first_i.get(s["run"], int(s["i"])), int(s["i"]))
        G, Gm, A, Am, B, Bm, M = [], [], [], [], [], [], []
        self.boxes, self.memory, self.text = bool(boxes), bool(memory), bool(text)
        if self.text:
            tz = np.load(cache / "text.npz")
            phrases = [str(p) for p in tz["phrases"]]
            emb = np.asarray(tz["embeds"], dtype=np.float32)
            emb /= np.linalg.norm(emb, axis=1, keepdims=True)
            self.T = torch.from_numpy(emb).to(device)
            self.text_backbone = str(tz["backbone"])
        text_row = {}
        offset, self.run_info = 0, {}
        rows_of = {}
        meta_of = {}
        M_of = {}
        for r in runs:
            d = cache / r
            info = json.loads((d / "info.json").read_text(encoding="utf-8"))
            vi = info.get("vocab", list(core.VOCAB)).index(info["object"])
            if self.text:
                if info["object"] not in phrases:
                    raise SystemExit(f"{cache}/text.npz has no embedding of {info['object']!r}")
                text_row[r] = phrases.index(info["object"])
            meta = np.load(d / "meta.npz")
            G.append(np.load(d / "G.npy", mmap_mode="r")[:, vi, :].astype(np.float16))
            Gm.append(np.load(d / "Gm.npy", mmap_mode="r")[:, vi, :].astype(np.float16))
            A.append(np.asarray(np.load(d / "A.npy", mmap_mode="r")))
            Am.append(np.asarray(np.load(d / "Am.npy", mmap_mode="r")))
            if self.boxes:
                if not (d / "B.npy").exists():
                    raise SystemExit(f"{d}: no B.npy - re-cache the run (build_cache --force)")
                B.append(np.load(d / "B.npy", mmap_mode="r")[:, vi, :].astype(np.float16))
                Bm.append(np.load(d / "Bm.npy", mmap_mode="r")[:, vi, :].astype(np.float16))
            rows_of[r] = {int(i): k for k, i in enumerate(meta["i"])}
            meta_of[r] = meta
            if self.memory:
                M.append(run_memory_of(d, info, meta, vi, rows_of[r].get(first_i[r], 0)))
                M_of[r] = M[-1]
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
        if self.boxes:
            zero_b = np.zeros((1, 4), np.float16)
            self.B = torch.from_numpy(np.concatenate(B + [zero_b])).to(device)
            self.Bm = torch.from_numpy(np.concatenate(Bm + [zero_b])).to(device)
        if self.memory:
            mem = np.concatenate(M + [np.zeros((1, core.MEM_DIM), np.float32)])
            self.M = torch.from_numpy(mem).to(device)
            self.Mm = torch.from_numpy(core.mirror_memory(mem)).to(device)
        self.zero_row = offset
        hist, pr, prm, tgt, aux, split, w, lab, ids, mrows, trows = ([], [], [], [], [], [], [],
                                                                     [], [], [], [])
        skipped = 0
        self.disagree = 0
        for s in samples:
            r = s["run"]
            meta = meta_of[r]
            k = rows_of[r].get(int(s["i"]))
            ok = meta["frame_ok"]
            if k is None or not ok[k]:
                skipped += 1
                continue
            if (self.memory and memory_agree_m > 0 and s.get("expert_est")
                    and memory_disagrees(M_of[r][k], meta["poses"][k], s["expert_est"],
                                         memory_agree_m)):
                # the memory is on something other than the expert's subject
                self.disagree += 1
                continue
            idx = [j if (j >= 0 and ok[j]) else -1
                   for j in core.history_indices(meta["t"], k, offsets)]
            p = core.proprio(meta["poses"], meta["vel"], meta["yaw_rate"], idx)
            off = self.run_info[r]["offset"]
            hist.append([off + j if j >= 0 else self.zero_row for j in idx])
            pr.append(p)
            prm.append(core.mirror_proprio(p))
            L = s["label_raw"]
            if "right_m" in self.axes and "right_m" not in L:
                raise SystemExit(f"{labels}: no right_m label (build_labels before 30 Oct): "
                                 f"rebuild the labels for --lateral")
            lab.append([L[ax] for ax in self.axes])
            mrows.append(off + k)
            trows.append(text_row.get(r, 0))
            vis = s.get("hint_kind") == "fresh" and s.get("box") is not None
            brg = math.radians(s["box"]["bearing_deg"]) if vis else 0.0
            aux.append([1.0 if vis else 0.0, brg])
            split.append(s["split"] == "train")
            # a capture whose executed command was not the expert's (a DART
            # offset or a VLA-flown DAgger window): a state the expert alone
            # would not have reached, weighted by --onpolicy-weight
            w.append((1.0 + 2.0 * min(abs(L["dpsi_rad"]) / 0.15, 1.0))
                     * (onpolicy_weight if s.get("executed_differs") else 1.0))
            ids.append(s["id"])
        self.skipped = skipped
        self.hist = torch.tensor(hist, dtype=torch.long, device=device)
        self.pr = torch.from_numpy(np.stack(pr)).to(device)
        self.prm = torch.from_numpy(np.stack(prm)).to(device)
        self.lab = torch.tensor(lab, dtype=torch.float32, device=device)
        self.aux = torch.tensor(aux, dtype=torch.float32, device=device)
        self.train = torch.tensor(split, dtype=torch.bool, device=device)
        self.w = torch.tensor(w, dtype=torch.float32, device=device)
        self.mrow = torch.tensor(mrows, dtype=torch.long, device=device)
        self.trow = torch.tensor(trows, dtype=torch.long, device=device)
        self.ids = ids
        self.grids = {ax: torch.tensor(core.GRIDS[ax], dtype=torch.float32, device=device)
                      for ax in self.axes}
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
        if self.boxes:
            box = torch.where(m3, self.Bm[h], self.B[h]).float()
            st = torch.cat([st, box], dim=2)
        # an empty slot's stats are zero, as the server's Ring gives them
        st = st * (h != self.zero_row).unsqueeze(-1).to(st.dtype)
        pr = torch.where(m3, self.prm[idx], self.pr[idx])
        valid = h != self.zero_row
        lab = self.lab[idx].clone()
        for c, ax in enumerate(self.axes):
            if ax in MIRROR_AXES:
                lab[:, c] = torch.where(mirror, -lab[:, c], lab[:, c])
        aux = self.aux[idx].clone()
        aux[:, 1] = torch.where(mirror, -aux[:, 1], aux[:, 1])
        x = (g, a, st, pr, valid)
        if self.memory:
            mr = self.mrow[idx]
            x = x + (torch.where(mirror.view(-1, 1), self.Mm[mr], self.M[mr]),)
        if self.text:
            x = x + (None,) * (not self.memory) + (self.T[self.trow[idx]],)
        return x, lab, aux


def decode_batch(torch, logits: dict, grids: dict, window: int = core.DECODE_WINDOW):
    """core.decode for a batch: {axis: (B, n)} -> (B, len(grids)) in the
    order of `grids` (the data's axes)."""
    out = []
    for ax in grids:
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
    for c, ax in enumerate(data.axes):
        out[f"mae_{ax}"] = float((p[:, c] - l[:, c]).abs().mean())
    d = data.axes.index("dpsi_rad")
    turn = l[:, d].abs() > 0.05
    out["dpsi_sign_acc_turning"] = float(((p[turn, d] > 0) == (l[turn, d] > 0)).float().mean()) \
        if bool(turn.any()) else None
    out["n_turning"] = int(turn.sum())
    out["dpsi_corr"] = float(torch.corrcoef(torch.stack([p[:, d], l[:, d]]))[0, 1])
    if "right_m" in data.axes:
        r = data.axes.index("right_m")
        out["right_corr"] = float(torch.corrcoef(torch.stack([p[:, r], l[:, r]]))[0, 1])
    out["visible_acc"] = float(((vp > 0.5) == (at[:, 0] > 0.5)).float().mean())
    head.train()
    return out


def baseline(data: Data, tr_idx, ho_idx) -> dict:
    """The constant answer: the training labels' median per axis."""
    med = data.lab[tr_idx].median(dim=0).values
    l = data.lab[ho_idx]
    return {f"mae_{ax}": float((l[:, c] - med[c]).abs().mean()) for c, ax in enumerate(data.axes)}


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
    ap.add_argument("--onpolicy-weight", type=float, default=1.0,
                    help="loss weight x this for captures whose executed command was not the "
                         "expert's (build_labels executed_differs: DART, DAgger windows)")
    ap.add_argument("--box-features", action="store_true",
                    help="feed the backbone's box (cx, cy, w, h) per slot (HeadConfig.n_stats 8; "
                         "the cache needs B.npy)")
    ap.add_argument("--long-history", action="store_true",
                    help="core.LONG_HIST_OFFSETS_S (10 slots, 5.5 s) instead of the 8-slot 2.5 s")
    ap.add_argument("--runs", nargs="*", default=None,
                    help="train and hold out on these runs only (default every run labelled)")
    ap.add_argument("--motion-dropout", type=float, default=0.0,
                    help="share of training samples whose motion proprio (MOTION_FIELDS, every "
                         "slot) is zeroed")
    ap.add_argument("--lateral", action="store_true",
                    help="also learn right_m (core LATERAL_AXES; labels with right_m)")
    ap.add_argument("--memory", action="store_true",
                    help="feed the target memory's MEM_FIELDS (core MEMORY; the cache needs B.npy)")
    ap.add_argument("--memory-dropout", type=float, default=0.1,
                    help="with --memory: share of training samples whose memory is zeroed")
    ap.add_argument("--memory-agree-m", type=float, default=0.0,
                    help="with --memory: drop a sample whose valid memory is more than this "
                         "from the expert's own estimate (build_labels expert_est; 0: keep all)")
    ap.add_argument("--text", action="store_true",
                    help="feed the run's object words as the backbone's text embedding "
                         "(<cache>/text.npz; HeadConfig.n_text)")
    args = ap.parse_args(argv)
    import torch
    from training.aiotvla.model import HeadConfig, AIOTVLAHead, n_params
    torch.manual_seed(args.seed)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    offsets = core.LONG_HIST_OFFSETS_S if args.long_history else core.HIST_OFFSETS_S
    axes = core.LATERAL_AXES if args.lateral else core.AXES
    data = Data(Path(args.labels), Path(args.cache), args.device, offsets, args.runs,
                args.onpolicy_weight, args.box_features, axes, args.memory, args.text,
                args.memory_agree_m)
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
          f"{data.skipped} skipped, {data.disagree} dropped (memory off the expert's subject), "
          f"loaded in {time.time() - t0:.1f} s")
    cfg = HeadConfig(n_slots=len(offsets), n_stats=8 if args.box_features else 4, axes=axes,
                     n_mem=core.MEM_DIM if args.memory else 0,
                     n_text=int(data.T.shape[1]) if args.text else 0)
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
    lw.writerow(["step", "epoch", "loss"] + [f"ce_{a}" for a in axes] + ["aux_vis", "aux_brg",
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
                x = (x[0], x[1], x[2], pr) + tuple(x[4:])
            if args.memory and args.memory_dropout > 0:
                keep = (torch.rand(len(j), generator=gen).to(j.device)
                        >= args.memory_dropout).to(x[5].dtype)
                x = tuple(x[:5]) + (x[5] * keep.unsqueeze(1),) + tuple(x[6:])
            logits, ax = head(*x)
            w = data.w[j]
            parts = {}
            loss = 0.0
            for c, a in enumerate(axes):
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
                            [round(float(parts[a]), 5) for a in axes] +
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
                                               len(core.GRIDS[a])] for a in axes},
                                 "AXES": list(axes),
                                 "LABEL_HORIZON_S": core.LABEL_HORIZON_S,
                                 **({"MEMORY": core.memory_constants()} if args.memory else {}),
                                 **({"TEXT": {"backbone": data.text_backbone,
                                              "dim": int(data.T.shape[1]), "unit_norm": True}}
                                    if args.text else {})},
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
