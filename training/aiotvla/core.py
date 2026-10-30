"""The parts of AIOT-VLA that training and serving must compute identically.

Stdlib + numpy only (no torch), so the server's request path and the tests
run it without a GPU.

FRAMES. World x = North, y = East, up = metres above the take-off ground,
yaw_rad = heading from North, clockwise seen from above (the recordings'
`pose`, protocol v2's `pose`). Body forward = (cos yaw, sin yaw), body right
= (-sin yaw, cos yaw).

HISTORY. One decision reads HIST_OFFSETS_S: the newest capture and the
captures nearest to t - offset (within HIST_TOL_S); a slot with no capture
in tolerance is empty (valid 0, its maps zero). The server keeps the same
ring (demo/vla_server_foreign.py, AIOTVLAModel) and picks the same way.

PROPRIO. Per capture: velocity from a BACKWARD difference only (the newest
capture has no future pose at serve time, so training must not use one
either): (p_i - p_j) / (t_i - t_j) with j the newest capture at least
VEL_DT_S older, no more than VEL_MAX_DT_S older (else zero, flagged). Per
history slot k, relative to the newest capture 0: position (forward, right,
up) in 0's body frame, heading wrap(yaw_k - yaw_0), the slot's velocity
rotated into 0's body frame, its yaw rate, its altitude, valid. PROPRIO_DIM.

LABEL. The expert's motion over the label horizon (tools/build_follow_dataset
`label_raw`): fwd_m along the new heading, down_m (+ down), dpsi_rad (+
clockwise). Each axis is learned as a distribution over its own grid
(GRIDS): soft targets, a Gaussian of SOFT_SIGMA cells around the value.
decode() reads the arg-max cell and the expectation within DECODE_WINDOW
cells of it, so a two-peaked answer (turn left or right while searching)
gives one of the peaks, not their mean.

MIRROR. The world reflected about the aircraft's long axis: frames flipped
left-right, every right-handed quantity negated (right offsets, headings,
yaw rates, dpsi). fwd and down unchanged.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

HIST_OFFSETS_S = (0.0, 0.2, 0.4, 0.7, 1.0, 1.4, 1.9, 2.5)
# v3 on (30 Oct): a longer memory, so a car that left the frame at a corner
# a few seconds ago is still in the input. A checkpoint carries the offsets
# it was trained with; the server's Ring takes them from it.
LONG_HIST_OFFSETS_S = (0.0, 0.2, 0.4, 0.7, 1.0, 1.5, 2.0, 3.0, 4.0, 5.5)
HIST_TOL_S = 0.15
VEL_DT_S = 0.2
VEL_MAX_DT_S = 0.6
GRID_SIDE = 24                          # OWL-ViT B/32 at 768 px: 24 x 24 patches
N_PATCHES = GRID_SIDE * GRID_SIDE
PCA_DIM = 8
# The phrases whose grounding maps the cache stores. A flight's instruction
# outside this list is grounded live by the same formula (backbone.ground).
VOCAB = ("a red car", "a white car", "a blue car", "a person")
LABEL_HORIZON_S = 0.5

GRIDS = {
    "fwd_m": np.linspace(0.0, 3.0, 61),          # 0.05 m cells
    "down_m": np.linspace(-1.0, 1.0, 41),        # 0.05 m cells
    "dpsi_rad": np.linspace(-0.8, 0.8, 65),      # 0.025 rad cells
}
AXES = ("fwd_m", "down_m", "dpsi_rad")
SOFT_SIGMA = 1.5
DECODE_WINDOW = 3
PROPRIO_FIELDS = ("rel_fwd", "rel_right", "rel_up", "rel_yaw", "v_fwd", "v_right", "v_up",
                  "yaw_rate", "alt", "valid")
PROPRIO_DIM = len(PROPRIO_FIELDS)
# Fields negated by the mirror (MIRROR above).
MIRROR_NEGATE = ("rel_right", "rel_yaw", "v_right", "yaw_rate")


def wrap(a: float) -> float:
    """An angle in (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    up: float
    yaw: float

    @classmethod
    def of(cls, d: dict) -> "Pose":
        return cls(float(d["x"]), float(d["y"]), float(d["up"]), float(d["yaw_rad"]))


def backward_velocity(t: np.ndarray, poses: np.ndarray, i: int) -> tuple[np.ndarray, float, bool]:
    """(world velocity [vx, vy, vup], yaw rate, ok) of capture i from the
    newest capture j < i with t_i - t_j in [VEL_DT_S, VEL_MAX_DT_S]. `poses`
    is (N, 4): x, y, up, yaw. Zero and False when there is none."""
    ti = t[i]
    j = i - 1
    while j >= 0 and ti - t[j] < VEL_DT_S:
        j -= 1
    if j < 0 or ti - t[j] > VEL_MAX_DT_S:
        return np.zeros(3), 0.0, False
    dt = ti - t[j]
    v = (poses[i, :3] - poses[j, :3]) / dt
    return v, wrap(poses[i, 3] - poses[j, 3]) / dt, True


def velocities(t: np.ndarray, poses: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """backward_velocity for every capture: (N, 3), (N,), (N,) bool."""
    n = len(t)
    v = np.zeros((n, 3))
    yr = np.zeros(n)
    ok = np.zeros(n, dtype=bool)
    for i in range(n):
        v[i], yr[i], ok[i] = backward_velocity(t, poses, i)
    return v, yr, ok


def history_indices(t: np.ndarray, i: int, offsets=HIST_OFFSETS_S,
                    tol: float = HIST_TOL_S) -> list[int]:
    """For capture i of a time-sorted capture list: the index of the capture
    nearest to t_i - offset for each of `offsets`, never newer than i, or -1
    when none is within `tol`."""
    out = []
    for off in offsets:
        if off == 0.0:
            out.append(i)
            continue
        want = t[i] - off
        k = int(np.searchsorted(t, want, side="left"))
        best, bd = -1, tol
        for c in (k - 1, k):
            if 0 <= c <= i:
                d = abs(t[c] - want)
                if d <= bd:
                    best, bd = c, d
        out.append(best)
    return out


def proprio(poses: np.ndarray, vel: np.ndarray, yaw_rate: np.ndarray, idx: list[int]) -> np.ndarray:
    """(len(idx), PROPRIO_DIM) for history slots `idx` (idx[0] the newest
    capture, -1 an empty slot), relative to idx[0] (module docstring)."""
    p0 = poses[idx[0]]
    c, s = math.cos(p0[3]), math.sin(p0[3])
    out = np.zeros((len(idx), PROPRIO_DIM), dtype=np.float32)
    for k, j in enumerate(idx):
        if j < 0:
            continue
        dx, dy = poses[j, 0] - p0[0], poses[j, 1] - p0[1]
        vx, vy = vel[j, 0], vel[j, 1]
        out[k] = (dx * c + dy * s, -dx * s + dy * c, poses[j, 2] - p0[2],
                  wrap(poses[j, 3] - p0[3]),
                  vx * c + vy * s, -vx * s + vy * c, vel[j, 2], yaw_rate[j],
                  poses[j, 2] / 10.0, 1.0)
    return out


def mirror_proprio(pr: np.ndarray) -> np.ndarray:
    out = pr.copy()
    for f in MIRROR_NEGATE:
        out[..., PROPRIO_FIELDS.index(f)] *= -1.0
    return out


def mirror_label(label: dict) -> dict:
    return {"fwd_m": label["fwd_m"], "down_m": label["down_m"], "dpsi_rad": -label["dpsi_rad"]}


def soft_target(value: float, axis: str, sigma: float = SOFT_SIGMA) -> np.ndarray:
    """A probability vector over GRIDS[axis]: a Gaussian of `sigma` cells
    centred on `value` clipped to the grid."""
    g = GRIDS[axis]
    v = min(max(float(value), g[0]), g[-1])
    cell = (g[1] - g[0])
    w = np.exp(-0.5 * ((g - v) / (sigma * cell)) ** 2)
    return (w / w.sum()).astype(np.float32)


def decode(probs: np.ndarray, axis: str, window: int = DECODE_WINDOW) -> float:
    """The value of a probability vector over GRIDS[axis] (module docstring).
    The window is symmetric about the arg-max and shrinks near a grid end, so
    an answer peaked on the end cell decodes to the end value (a hover is
    fwd 0.0, not a creep of 0.04 m)."""
    g = GRIDS[axis]
    p = np.asarray(probs, dtype=np.float64)
    m = int(p.argmax())
    half = min(window, m, len(g) - 1 - m)
    lo, hi = m - half, m + half + 1
    w = p[lo:hi]
    return float((w * g[lo:hi]).sum() / max(w.sum(), 1e-12))


def grounding_stats(g_logits: np.ndarray) -> np.ndarray:
    """Four numbers from one grounding map (N_PATCHES logits): the soft-arg-
    max column and row in [-1, 1] (softmax at temperature 1), the max logit
    and the gap to the second-highest patch. The head gets them besides the
    map itself; the server and the trainer compute them here."""
    z = np.asarray(g_logits, dtype=np.float64).reshape(GRID_SIDE, GRID_SIDE)
    e = np.exp(z - z.max())
    e /= e.sum()
    lin = np.linspace(-1.0, 1.0, GRID_SIDE)
    col = float((e.sum(axis=0) * lin).sum())
    row = float((e.sum(axis=1) * lin).sum())
    top = np.sort(z.ravel())[-2:]
    return np.array([col, row, top[1], top[1] - top[0]], dtype=np.float32)


RING_GAP_S = 2.0
RING_MAXLEN = 64


class Ring:
    """The server's history: the newest captures' (t, pose, grounding map,
    appearance map), and the head's slot inputs for the newest one, picked
    and computed exactly as the trainer does from a recording
    (history_indices, backward_velocity, proprio, grounding_stats). A
    capture older than the last, or more than RING_GAP_S after it, starts a
    new history (a new flight, a stall): push() returns why."""

    def __init__(self, maxlen: int = RING_MAXLEN, gap_s: float = RING_GAP_S,
                 offsets=HIST_OFFSETS_S, tol: float = HIST_TOL_S, with_box: bool = False):
        self.maxlen, self.gap_s = int(maxlen), float(gap_s)
        self.offsets, self.tol = tuple(float(o) for o in offsets), float(tol)
        # the head's stats are grounding_stats (4) and, with_box, the
        # backbone's box (cx, cy, w, h) of the capture (4): HeadConfig.n_stats
        self.with_box = bool(with_box)
        self.b: list[np.ndarray] = []
        self.t: list[float] = []
        self.poses: list[tuple] = []
        self.g: list[np.ndarray] = []
        self.a: list[np.ndarray] = []

    def __len__(self) -> int:
        return len(self.t)

    def clear(self) -> None:
        self.t, self.poses, self.g, self.a, self.b = [], [], [], [], []

    def push(self, t: float, pose: dict, g: np.ndarray, a: np.ndarray,
             box: np.ndarray | None = None) -> str | None:
        why = None
        if self.t:
            if t < self.t[-1]:
                why = "t_capture went backwards"
            elif t - self.t[-1] > self.gap_s:
                why = f"a gap of {t - self.t[-1]:.2f} s (more than {self.gap_s:g} s)"
        if why:
            self.clear()
        p = Pose.of(pose)
        self.t.append(float(t))
        self.poses.append((p.x, p.y, p.up, p.yaw))
        self.g.append(np.asarray(g, dtype=np.float32))
        self.a.append(np.asarray(a, dtype=np.float32))
        self.b.append(np.zeros(4, np.float32) if box is None else np.asarray(box, np.float32))
        if len(self.t) > self.maxlen:
            for lst in (self.t, self.poses, self.g, self.a, self.b):
                del lst[0]
        return why

    def slots(self) -> dict:
        t = np.asarray(self.t)
        poses = np.asarray(self.poses, dtype=np.float64)
        i = len(t) - 1
        idx = history_indices(t, i, self.offsets, self.tol)
        vel = np.zeros((len(t), 3))
        yr = np.zeros(len(t))
        for j in set(j for j in idx if j >= 0):
            vel[j], yr[j], _ = backward_velocity(t, poses, j)
        k = len(idx)
        g = np.zeros((k, N_PATCHES), np.float32)
        a = np.zeros((k, N_PATCHES, self.a[-1].shape[-1]), np.float32)
        for s, j in enumerate(idx):
            if j >= 0:
                g[s], a[s] = self.g[j], self.a[j]
        ns = 8 if self.with_box else 4
        stats = np.zeros((k, ns), np.float32)
        for s, j in enumerate(idx):
            if j >= 0:
                stats[s, :4] = grounding_stats(g[s])
                if self.with_box:
                    stats[s, 4:] = self.b[j]
        valid = np.array([j >= 0 for j in idx])
        return {"g": g, "a": a, "stats": stats, "proprio": proprio(poses, vel, yr, idx),
                "valid": valid, "idx": idx, "n_valid": int(valid.sum())}


def mirror_map(m: np.ndarray) -> np.ndarray:
    """A patch map (..., N_PATCHES[, C]) mirrored left-right. Equal to the
    map of the flipped frame only approximately (the backbone is not exactly
    equivariant): the cache stores the flipped frame's own maps and this is
    for tests and checks."""
    a = np.asarray(m)
    shp = a.shape
    if a.shape[-1] == N_PATCHES:
        return a.reshape(shp[:-1] + (GRID_SIDE, GRID_SIDE))[..., ::-1].reshape(shp)
    return a.reshape(shp[:-2] + (GRID_SIDE, GRID_SIDE, shp[-1]))[..., ::-1, :].reshape(shp)
