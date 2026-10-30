"""The frozen vision-language backbone: OWL-ViT B/32 (google/owlvit-base-
patch32, the detector's own weights, demo/detectors.py).

For a frame and a list of phrases it gives
  G  (Q, 576)   the class-head logit of every patch for every phrase - the
                detector's own score before its box head, i.e. "how much of
                this phrase is in this patch" (OWL-ViT's learned per-patch
                shift and scale included);
  A  (576, P)   the patch's class embedding (unit-norm, 512-d, the space the
                phrases live in) projected on P principal components fitted
                over recorded frames (fit_pca): what the patch looks like,
                small enough to cache for every recorded capture.
The frame is resized to 768 x 768 and normalised as OWL-ViT's processor
does, but on the GPU (pixels()); a 256 x 256 VLAFront capture is up-sampled
(6/6 probe frames put the arg-max patch of "a red car" on the detector's
box, 2026-10-29).

OWLv2 (google/owlv2-base-patch16-ensemble, 30 Oct): its processor pads the
frame to a square (bottom and right, grey 0.5) and resizes it to 960 x 960;
pixels() does the same on the GPU. Its 60 x 60 patch grid is pooled to the
head's 24 x 24 (core.GRID_SIDE): the logits by max (the best patch in each
cell keeps its score), the class embeddings by mean. OWL-ViT B/32's grid is
already 24 x 24.
"""
from __future__ import annotations

import numpy as np

MODEL_ID = "google/owlvit-base-patch32"
OWLV2_ID = "google/owlv2-base-patch16-ensemble"
BACKBONES = (MODEL_ID, OWLV2_ID)
GRID_OUT = 24


class OwlBackbone:
    def __init__(self, model_id: str = MODEL_ID, device: str = "cuda", half: bool = True,
                 local_files_only: bool = True):
        import torch
        if model_id not in BACKBONES:
            raise ValueError(f"backbone {model_id!r} is not one of {BACKBONES}")
        self.torch = torch
        self.model_id = model_id
        self.v2 = model_id == OWLV2_ID
        self.size = 960 if self.v2 else 768
        self.device = device
        self.dtype = torch.float16 if (half and device.startswith("cuda")) else torch.float32
        if self.v2:
            from transformers import Owlv2ForObjectDetection as M, Owlv2Processor as P
        else:
            from transformers import OwlViTForObjectDetection as M, OwlViTProcessor as P
        self.proc = P.from_pretrained(model_id, local_files_only=local_files_only)
        m = M.from_pretrained(model_id, local_files_only=local_files_only)
        self.model = m.to(device=device, dtype=self.dtype).eval()
        self._core = m.owlv2 if self.v2 else m.owlvit
        self._text: dict[str, "torch.Tensor"] = {}
        ip = self.proc.image_processor
        self._mean = torch.tensor(ip.image_mean, device=device).view(1, 3, 1, 1)
        self._std = torch.tensor(ip.image_std, device=device).view(1, 3, 1, 1)
        self.pca_mean = None
        self.pca_comps = None

    # -- text ----------------------------------------------------------------- #
    def text_embeds(self, phrases) -> "torch.Tensor":
        """(Q, 512) query embeddings, cached per phrase."""
        torch = self.torch
        todo = [p for p in phrases if p not in self._text]
        if todo:
            t = self.proc(text=[todo], return_tensors="pt")
            with torch.no_grad():
                e = self._core.get_text_features(
                    input_ids=t["input_ids"].to(self.device),
                    attention_mask=t["attention_mask"].to(self.device))
            for p, v in zip(todo, e):
                self._text[p] = v.to(self.dtype)
        return torch.stack([self._text[p] for p in phrases])

    # -- images --------------------------------------------------------------- #
    def pixels(self, images, flip: bool = False):
        """OWL-ViT's input for RGB frames (PIL images or H x W x 3 uint8
        arrays, all one size): resized to 768 x 768 bicubic and normalised
        with CLIP's mean and std, ON THE GPU (the processor's PIL resize was
        57 ms a frame and the bottleneck). flip mirrors left-right first.
        Training cache and server both call this, so they see the same
        pixels."""
        torch = self.torch
        x = torch.from_numpy(np.stack([np.asarray(im, dtype=np.uint8) for im in images]))
        x = x.to(self.device).permute(0, 3, 1, 2).float()
        if flip:
            x = torch.flip(x, dims=[3])
        if self.v2:
            # OWLv2's processor: pad bottom/right to a square with grey 0.5
            _, _, h, w = x.shape
            side = max(h, w)
            x = torch.nn.functional.pad(x, (0, side - w, 0, side - h), value=127.5)
        x = torch.nn.functional.interpolate(x, size=(self.size, self.size), mode="bicubic",
                                            align_corners=False).clamp_(0.0, 255.0) / 255.0
        x = (x - self._mean) / self._std
        return x.to(self.dtype)

    def _feats(self, images, flip: bool = False):
        torch = self.torch
        px = self.pixels(images, flip=flip)
        with torch.no_grad():
            fm = self.model.image_embedder(pixel_values=px)[0]
        b, h, w, d = fm.shape
        return fm.reshape(b, h * w, d)

    def maps(self, images, phrases, with_class: bool = False, flip: bool = False):
        """(G (B, Q, 576) float32, A (B, 576, P) float32 or None before a PCA
        is set[, class embeds (B, 576, 512) when with_class])."""
        torch = self.torch
        feats = self._feats(images, flip=flip)
        q = self.text_embeds(phrases)
        b = feats.shape[0]
        qe = q.unsqueeze(0).expand(b, -1, -1)
        qm = torch.ones(qe.shape[:2], device=self.device, dtype=torch.long)
        with torch.no_grad():
            logits, cls = self.model.class_predictor(feats, qe, qm)
        logits, cls = logits.float(), cls.float()
        n = int(round(logits.shape[1] ** 0.5))
        if n != GRID_OUT:
            # OWLv2's 60 x 60 grid to the head's 24 x 24 (module docstring)
            F = torch.nn.functional
            lg = logits.permute(0, 2, 1).reshape(b, -1, n, n)
            logits = F.adaptive_max_pool2d(lg, GRID_OUT).reshape(b, -1, GRID_OUT ** 2)
            logits = logits.permute(0, 2, 1)
            c = cls.permute(0, 2, 1).reshape(b, -1, n, n)
            cls = F.adaptive_avg_pool2d(c, GRID_OUT).reshape(b, -1, GRID_OUT ** 2)
            cls = cls.permute(0, 2, 1)
        g = logits.permute(0, 2, 1).cpu().numpy()
        a = None
        if self.pca_mean is not None:
            a = ((cls - self.pca_mean) @ self.pca_comps.T).cpu().numpy()
        out = (g, a)
        if with_class:
            out = out + (cls.cpu().numpy(),)
        return out

    # -- the appearance projection ---------------------------------------------- #
    def set_pca(self, mean: np.ndarray, comps: np.ndarray) -> None:
        torch = self.torch
        self.pca_mean = torch.as_tensor(np.asarray(mean, dtype=np.float32), device=self.device)
        self.pca_comps = torch.as_tensor(np.asarray(comps, dtype=np.float32), device=self.device)


def fit_pca(class_embeds: np.ndarray, dim: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(mean (512,), components (dim, 512), explained variance ratio (dim,))
    of a (n, 512) sample of patch class embeddings."""
    x = np.asarray(class_embeds, dtype=np.float64)
    mean = x.mean(axis=0)
    _, s, vt = np.linalg.svd(x - mean, full_matrices=False)
    var = s ** 2
    return (mean.astype(np.float32), vt[:dim].astype(np.float32),
            (var[:dim] / var.sum()).astype(np.float32))
