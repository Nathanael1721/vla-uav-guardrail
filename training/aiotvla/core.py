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

LATERAL (v7, 30 Oct). The follow expert flies a holonomic aircraft: on 35 %
of its ticks it slides sideways at about 1.3 m/s with the camera on the car
(its fence's skirt and near modes), and v3-v6 dropped that component, so
along a building they could only slow down. A v7 checkpoint also answers
right_m = v_right * H over GRIDS["right_m"] (LATERAL_AXES), flown as a
lateral step (guardrail/vla_link.py). Its axes are in its config.

MEMORY (v7). Where the named object IS, kept between frames, so a car that
turned out of the frame at a corner is still in the input (every v3/v4/v6
flight lost it there): TargetMemory, a constant-velocity Kalman filter on the
object's ground position, fed by box_measurement (the backbone's box at the
phrase's best patch, its bottom edge on flat ground) and read out as
MEM_FIELDS in the newest capture's body frame. The trainer runs it over each
recording in capture order from the run's first sample; the server runs the
same code over the requests of one flight.
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
# Appended only (31 Oct: the multi-target recordings), so a cache's row of a
# phrase is its index in that cache's own info.json "vocab".
VOCAB = ("a red car", "a white car", "a blue car", "a person", "a black car", "a box truck")
LABEL_HORIZON_S = 0.5

GRIDS = {
    "fwd_m": np.linspace(0.0, 3.0, 61),          # 0.05 m cells
    "down_m": np.linspace(-1.0, 1.0, 41),        # 0.05 m cells
    "dpsi_rad": np.linspace(-0.8, 0.8, 65),      # 0.025 rad cells
    "right_m": np.linspace(-1.0, 1.0, 41),       # 0.05 m cells: +-2 m/s over 0.5 s (LATERAL)
}
AXES = ("fwd_m", "down_m", "dpsi_rad")
LATERAL_AXES = AXES + ("right_m",)
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
    out = {"fwd_m": label["fwd_m"], "down_m": label["down_m"], "dpsi_rad": -label["dpsi_rad"]}
    if "right_m" in label:
        out["right_m"] = -label["right_m"]
    return out


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


# ------------------------------------------------------------------ memory --
# The cameras a front source names (guardrail/aerialvla_codec.FRONT_SOURCES;
# a test keeps the two equal): horizontal field of view and mount pitch, deg.
CAMERAS = {"detector": (90.0, -20.0)}
MEM_LOGIT_MIN = -3.0        # a box is a measurement only above this patch logit
                            # (cache_front: 0.958 of the boxes on the car pass, 0.22
                            # of the frames without it in view)
MEM_SEED_LOGIT = -2.5       # ... and starts a track only above this
# ... and only when the phrase beats its CONTRAST phrases at that patch
# (logit minus their best, > MEM_CONTRAST_MIN): OWL-ViT's "a red car" also
# fires on the white and blue cars; with the contrast, 0.950 of the boxes on
# the car pass and 0.026 of the frames without it in view (0.22 without).
MEM_CONTRAST_MIN = 0.0
# A checkpoint records the table it was trained with (memory_constants) and
# the server reads the contrast phrases from it, so an entry added here does
# not change what an older checkpoint grounds.
CONTRAST = {
    "a red car": ("a white car", "a blue car"),
    "a white car": ("a red car", "a blue car", "a black car", "a box truck"),
    "a black car": ("a red car", "a white car", "a blue car"),
    "a box truck": ("a red car", "a white car", "a blue car", "a black car"),
}
MEM_RANGE_M = (3.0, 70.0)
MEM_MIN_DEPRESSION_RAD = math.radians(2.0)
MEM_RANGE_SIGMA = (0.5, 0.15)   # m: 0.5 + 0.15 x range (|err| p50 0.13-0.20 at 15-40 m)
MEM_BEARING_SIGMA_RAD = math.radians(1.0)    # p50 0.4-0.5 deg on the car
MEM_ACCEL_NOISE = 1.5       # m/s^2, demo/target_state.TargetState's
MEM_GATE_SIGMA = 4.0
MEM_CONFIRM_S = 1.0         # a second measurement within this, near the first, confirms
MEM_CONFIRM_M = 6.0
MEM_MAX_AGE_S = 8.0         # unseen this long: the track is dropped
MEM_RESEED_N = 5            # this many gated-out measurements in a row that agree
MEM_RESEED_SPREAD_M = 5.0   # (within this of each other) start a new track there
MEM_V_MAX = 15.0            # m/s, a clamp on the estimate (a car, a person: less)
MEM_FIELDS = ("valid", "age", "fwd", "right", "range", "bearing", "v_fwd", "v_right",
              "range_rate", "sigma", "n_upd", "accepted", "meas", "meas_range",
              "meas_bearing", "logit", "contrast")
MEM_DIM = len(MEM_FIELDS)
MEM_NEGATE = ("right", "bearing", "v_right", "meas_bearing")


def box_measurement(box, up: float, aspect: float, hfov_deg: float, pitch_deg: float):
    """(forward m, right m) of the ground point under a backbone box's bottom
    edge, in the body frame (level, the capture's heading), or None.

    `box` (cx, cy, w, h) in [0, 1] of the frame; `aspect` = frame height /
    width; the camera pitched `pitch_deg` (negative: down) with `hfov_deg`;
    flat ground `up` metres below it. Exact pinhole geometry: the ray through
    the bottom-centre pixel, (x, y, 1) in the camera (x right, y down),
    rotated by the pitch, meets the ground at t = up / its down component.
    None for a ray within MEM_MIN_DEPRESSION_RAD of the horizon or above it,
    or a range outside MEM_RANGE_M."""
    if box is None or up is None or up <= 0.5:
        return None
    cx, cy, _, bh = (float(v) for v in box[:4])
    if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(bh)):
        return None
    k = math.tan(math.radians(hfov_deg) / 2.0)
    x = (cx - 0.5) * 2.0 * k
    y = (cy + bh / 2.0 - 0.5) * 2.0 * k * aspect
    th = -math.radians(pitch_deg)
    f = math.cos(th) - y * math.sin(th)
    d = y * math.cos(th) + math.sin(th)
    if d <= math.sin(MEM_MIN_DEPRESSION_RAD) * math.sqrt(x * x + y * y + 1.0):
        return None
    t = up / d
    fwd, right = t * f, t * x
    r = math.hypot(fwd, right)
    if not MEM_RANGE_M[0] <= r <= MEM_RANGE_M[1]:
        return None
    return fwd, right


class TargetMemory:
    """MEMORY (module docstring): the object's world position and velocity
    [x, y, vx, vy], updated with each capture's box and predicted between.

    step(t, pose, box, logit, contrast) folds in one capture and returns its
    MEM_FIELDS (numpy float32). A box is a measurement when its logit is
    above MEM_LOGIT_MIN, its contrast above MEM_CONTRAST_MIN (None: no
    contrast phrases, not checked) and box_measurement gives a ground
    point. With no track, a
    measurement above MEM_SEED_LOGIT is a candidate; a second within
    MEM_CONFIRM_S and MEM_CONFIRM_M of it confirms a track (two false boxes
    rarely agree). A confirmed track takes a measurement unless it is more
    than MEM_GATE_SIGMA from the prediction (the Mahalanobis gate of
    demo/target_state.TargetState, whose model, noise and sub-stepping this
    copies); MEM_RESEED_N gated-out measurements in a row within
    MEM_RESEED_SPREAD_M of each other replace the track (it was on the wrong
    thing, or the object moved where the model did not go); a track unseen
    for MEM_MAX_AGE_S is dropped. reset() forgets everything (a new flight,
    a gap: core.Ring's rule)."""

    def __init__(self, aspect: float, hfov_deg: float, pitch_deg: float):
        self.aspect = float(aspect)
        self.hfov_deg, self.pitch_deg = float(hfov_deg), float(pitch_deg)
        self.reset()

    def reset(self) -> None:
        self.x = None               # posterior [x, y, vx, vy] at t_upd
        self.P = None
        self.t_upd = None
        self.n_upd = 0
        self.cand = None            # (t, wx, wy): an unconfirmed first measurement
        self.rejected: list[tuple[float, float, float]] = []

    # -- the filter (demo/target_state.TargetState's) --------------------------
    @staticmethod
    def _propagate(x, P, dt, q=MEM_ACCEL_NOISE, step_s=0.1):
        n = max(1, int(math.ceil(dt / step_s - 1e-9)))
        h = dt / n
        F = np.array([[1, 0, h, 0], [0, 1, 0, h], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        q2, d2, d3, d4 = q * q, h * h, h ** 3, h ** 4
        Q = q2 * np.array([[d4 / 4, 0, d3 / 2, 0], [0, d4 / 4, 0, d3 / 2],
                           [d3 / 2, 0, d2, 0], [0, d3 / 2, 0, d2]], float)
        for _ in range(n):
            x, P = F @ x, F @ P @ F.T + Q
        return x, P

    @staticmethod
    def _R(rng, th):
        c, s = math.cos(th), math.sin(th)
        J = np.array([[c, -rng * s], [s, rng * c]], float)
        sr = MEM_RANGE_SIGMA[0] + MEM_RANGE_SIGMA[1] * rng
        return J @ np.diag([sr * sr, MEM_BEARING_SIGMA_RAD ** 2]) @ J.T

    def _predict(self, t):
        dt = t - self.t_upd
        return self._propagate(self.x, self.P, dt) if dt > 0 else (self.x.copy(), self.P.copy())

    def _seed(self, t, z, R, v=(0.0, 0.0)):
        self.x = np.array([z[0], z[1], v[0], v[1]], float)
        self.P = np.diag([R[0, 0], R[1, 1], 4.0, 4.0])
        self.t_upd, self.n_upd = t, 1
        self.cand, self.rejected = None, []

    def step(self, t: float, pose, box, logit, contrast=None) -> np.ndarray:
        """pose (x, y, up, yaw): the capture's; box (cx, cy, w, h) or None;
        logit the phrase's best patch logit (None with no frame); contrast
        (grounding_contrast) or None."""
        px, py, up, yaw = (float(v) for v in pose)
        c, s = math.cos(yaw), math.sin(yaw)
        meas = None
        if (box is not None and logit is not None and logit > MEM_LOGIT_MIN
                and (contrast is None or contrast > MEM_CONTRAST_MIN)):
            meas = box_measurement(box, up, self.aspect, self.hfov_deg, self.pitch_deg)
        accepted = False
        if self.x is not None and t - self.t_upd > MEM_MAX_AGE_S:
            self.reset()
        if meas is not None:
            fwd, right = meas
            z = np.array([px + fwd * c - right * s, py + fwd * s + right * c])
            R = self._R(math.hypot(fwd, right), yaw + math.atan2(right, fwd))
            if self.x is None:
                if (self.cand is not None and t - self.cand[0] <= MEM_CONFIRM_S
                        and math.hypot(z[0] - self.cand[1], z[1] - self.cand[2]) <= MEM_CONFIRM_M):
                    dt = max(t - self.cand[0], 1e-3)
                    v = ((z[0] - self.cand[1]) / dt, (z[1] - self.cand[2]) / dt)
                    if math.hypot(*v) > 5.0:          # two boxes 0.1 s apart: no speed yet
                        v = (0.0, 0.0)
                    self._seed(t, z, R, v)
                    self.n_upd = 2
                    accepted = True
                elif logit > MEM_SEED_LOGIT:
                    self.cand = (t, float(z[0]), float(z[1]))
            else:
                xp, Pp = self._predict(t)
                H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
                y = z - H @ xp
                S = H @ Pp @ H.T + R
                Si = np.linalg.inv(S)
                if float(y @ Si @ y) <= MEM_GATE_SIGMA ** 2:
                    K = Pp @ H.T @ Si
                    x = xp + K @ y
                    P = (np.eye(4) - K @ H) @ Pp
                    v = math.hypot(x[2], x[3])
                    if v > MEM_V_MAX:
                        x[2:] *= MEM_V_MAX / v
                    self.x, self.P, self.t_upd = x, P, t
                    self.n_upd += 1
                    self.rejected = []
                    accepted = True
                else:
                    self.rejected = (self.rejected + [(t, float(z[0]), float(z[1]))])[-MEM_RESEED_N:]
                    if len(self.rejected) == MEM_RESEED_N:
                        xs = [r[1] for r in self.rejected]
                        ys = [r[2] for r in self.rejected]
                        if (max(xs) - min(xs) <= MEM_RESEED_SPREAD_M
                                and max(ys) - min(ys) <= MEM_RESEED_SPREAD_M):
                            self._seed(t, z, R)
                            accepted = True
        return self.features(t, (px, py, up, yaw), meas, accepted, logit, contrast)

    def features(self, t, pose, meas, accepted, logit, contrast=None) -> np.ndarray:
        """MEM_FIELDS at time t in the body frame of `pose`: the track's
        prediction (zeros with none), this capture's measurement (zeros with
        none), its logit and contrast."""
        px, py, _, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        out = np.zeros(MEM_DIM, np.float32)
        if self.x is not None:
            x, P = self._predict(t)
            dx, dy = x[0] - px, x[1] - py
            fwd, right = dx * c + dy * s, -dx * s + dy * c
            r = math.hypot(dx, dy)
            vf, vr = x[2] * c + x[3] * s, -x[2] * s + x[3] * c
            out[:12] = (1.0, min(t - self.t_upd, MEM_MAX_AGE_S) / MEM_MAX_AGE_S, fwd / 30.0,
                        right / 30.0, r / 30.0, math.atan2(right, fwd), vf / 5.0, vr / 5.0,
                        ((x[2] * dx + x[3] * dy) / r if r > 1e-3 else 0.0) / 5.0,
                        math.sqrt(max(P[0, 0] + P[1, 1], 0.0)) / 10.0,
                        math.log1p(self.n_upd) / 5.0, 1.0 if accepted else 0.0)
        if meas is not None:
            out[12:15] = (1.0, math.hypot(*meas) / 30.0, math.atan2(meas[1], meas[0]))
        out[15] = 0.0 if logit is None else (float(logit) + 6.0) / 3.0
        out[16] = 0.0 if contrast is None else float(contrast)
        return out


def grounding_contrast(g_all: np.ndarray, k: int, negatives) -> float | None:
    """The phrase's logit at its best patch minus the best of its contrast
    phrases' logits there (CONTRAST). g_all (n_phrases, N_PATCHES): row k
    the phrase's map, rows `negatives` its contrast phrases'. None with no
    contrast phrases."""
    if not negatives:
        return None
    g = np.asarray(g_all, dtype=np.float64)
    p = int(g[k].argmax())
    return float(g[k, p] - max(g[j, p] for j in negatives))


def memory_constants() -> dict:
    """The memory's constants, as a checkpoint records them (train.py) and the
    server compares them (demo/vla_server_foreign.AIOTVLAModel refuses a
    checkpoint trained with other ones, "contrast" apart: it grounds the
    checkpoint's own contrast phrases), JSON-shaped."""
    out = {"fields": list(MEM_FIELDS), "cameras": {k: list(v) for k, v in CAMERAS.items()},
           "contrast": {k: list(v) for k, v in CONTRAST.items()}}
    for k, v in sorted(globals().items()):
        if k.startswith("MEM_") and k not in ("MEM_FIELDS", "MEM_NEGATE", "MEM_DIM"):
            out[k] = list(v) if isinstance(v, tuple) else v
    return out


def mirror_memory(m: np.ndarray) -> np.ndarray:
    out = np.array(m, copy=True)
    for f in MEM_NEGATE:
        out[..., MEM_FIELDS.index(f)] *= -1.0
    return out


def run_memory(t: np.ndarray, poses: np.ndarray, boxes: np.ndarray, logits: np.ndarray,
               ok: np.ndarray, start: int, aspect: float, hfov_deg: float,
               pitch_deg: float, contrasts: np.ndarray | None = None) -> np.ndarray:
    """MEM_FIELDS of every capture of one recording (rows in capture order),
    the memory started at row `start` (the run's first sample: the server's
    starts at a flight's first request) and reset on a gap of more than
    RING_GAP_S or a time that runs backwards, as core.Ring. Rows before
    `start` carry zeros; a missing frame (ok False) is no measurement.
    `contrasts` per row (grounding_contrast), or None for none."""
    out = np.zeros((len(t), MEM_DIM), np.float32)
    mem = TargetMemory(aspect, hfov_deg, pitch_deg)
    t_prev = None
    for i in range(int(start), len(t)):
        if t_prev is not None and (t[i] < t_prev or t[i] - t_prev > RING_GAP_S):
            mem.reset()
        t_prev = float(t[i])
        if ok[i]:
            out[i] = mem.step(float(t[i]), poses[i], boxes[i], float(logits[i]),
                              None if contrasts is None else float(contrasts[i]))
        else:
            out[i] = mem.step(float(t[i]), poses[i], None, None)
    return out
