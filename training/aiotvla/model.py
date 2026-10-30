"""The trained part of AIOT-VLA: a small temporal head over the backbone's maps.

Per history slot (core.HIST_OFFSETS_S): the instruction's grounding map
(1 x 24 x 24, scaled), the appearance map (PCA_DIM x 24 x 24), the four
grounding_stats numbers and the slot's proprio (core.PROPRIO_DIM). A shared
convolutional encoder turns each slot into one token; a two-layer
transformer reads the slots (with a learned slot embedding and a summary
token) and three heads answer distributions over core.GRIDS (fwd_m, down_m,
dpsi_rad), plus two auxiliary outputs used only as training signals: is the
target in the newest frame (visible) and its bearing (from the expert's
detector box; neither is ever an input).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import torch
from torch import nn

from . import core

G_SHIFT, G_SCALE = 6.0, 3.0       # OWL-ViT logits sit around -8 .. 0


@dataclass
class HeadConfig:
    pca_dim: int = core.PCA_DIM
    n_slots: int = len(core.HIST_OFFSETS_S)
    d_model: int = 192
    n_layers: int = 2
    n_heads: int = 4
    dropout: float = 0.1
    # 4: core.grounding_stats; 8: and the backbone's box (cx, cy, w, h)
    n_stats: int = 4

    def to_dict(self) -> dict:
        return asdict(self)


class SlotEncoder(nn.Module):
    def __init__(self, cfg: HeadConfig):
        super().__init__()
        cin = 1 + cfg.pca_dim
        self.conv = nn.Sequential(
            nn.Conv2d(cin, 48, 3, padding=1), nn.GELU(),
            nn.Conv2d(48, 64, 3, stride=2, padding=1), nn.GELU(),      # 12 x 12
            nn.Conv2d(64, 96, 3, stride=2, padding=1), nn.GELU(),      # 6 x 6
        )
        self.proj = nn.Linear(96 * 36 + cfg.n_stats + core.PROPRIO_DIM, cfg.d_model)
        self.norm = nn.LayerNorm(cfg.d_model)

    def forward(self, g, a, stats, pr):
        # g (N, 576), a (N, 576, P), stats (N, 4), pr (N, PROPRIO_DIM)
        n = g.shape[0]
        s = core.GRID_SIDE
        x = torch.cat([((g + G_SHIFT) / G_SCALE).reshape(n, 1, s, s),
                       a.permute(0, 2, 1).reshape(n, -1, s, s)], dim=1)
        h = self.conv(x).flatten(1)
        st = torch.cat([torch.stack([stats[:, 0], stats[:, 1], (stats[:, 2] + G_SHIFT) / G_SCALE,
                                     stats[:, 3]], dim=1), stats[:, 4:]], dim=1)
        return self.norm(self.proj(torch.cat([h, st, pr], dim=1)))


class AIOTVLAHead(nn.Module):
    def __init__(self, cfg: HeadConfig | None = None):
        super().__init__()
        self.cfg = cfg = cfg or HeadConfig()
        self.enc = SlotEncoder(cfg)
        self.slot_emb = nn.Parameter(torch.zeros(cfg.n_slots + 1, cfg.d_model))
        self.summary = nn.Parameter(torch.zeros(1, 1, cfg.d_model))
        layer = nn.TransformerEncoderLayer(cfg.d_model, cfg.n_heads, 4 * cfg.d_model,
                                           cfg.dropout, batch_first=True, norm_first=True,
                                           activation="gelu")
        self.tr = nn.TransformerEncoder(layer, cfg.n_layers)
        self.out_norm = nn.LayerNorm(cfg.d_model)
        self.heads = nn.ModuleDict({ax: nn.Linear(cfg.d_model, len(core.GRIDS[ax]))
                                    for ax in core.AXES})
        self.aux = nn.Linear(cfg.d_model, 2)          # visible logit, bearing (rad / 0.8)
        nn.init.normal_(self.slot_emb, std=0.02)
        nn.init.normal_(self.summary, std=0.02)

    def forward(self, g, a, stats, pr, valid):
        """g (B, K, 576), a (B, K, 576, P), stats (B, K, 4), pr (B, K, PD),
        valid (B, K) bool -> ({axis: logits}, aux (B, 2))."""
        b, k = g.shape[:2]
        tok = self.enc(g.reshape(b * k, -1), a.reshape(b * k, a.shape[2], a.shape[3]),
                       stats.reshape(b * k, -1), pr.reshape(b * k, -1)).reshape(b, k, -1)
        tok = tok * valid.unsqueeze(-1).to(tok.dtype)
        x = torch.cat([self.summary.expand(b, -1, -1), tok], dim=1) + self.slot_emb
        pad = torch.cat([torch.zeros(b, 1, dtype=torch.bool, device=g.device), ~valid], dim=1)
        h = self.out_norm(self.tr(x, src_key_padding_mask=pad)[:, 0])
        return {ax: hd(h) for ax, hd in self.heads.items()}, self.aux(h)


def n_params(m: nn.Module) -> int:
    return sum(p.numel() for p in m.parameters())
