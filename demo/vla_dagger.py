"""
DAgger for the follow fine-tune: AerialVLA flies windows of the expert's
recording, and the expert's recovery from wherever the VLA left the aircraft
is the training label (2026-10-24).

    # the fine-tuned model behind its socket (vla-real env, the GPU)
    python demo/vla_server_v2.py serve --precision bf16 --attn sdpa --merge \\
        --missing-down refuse --adapter D:/models/aerialvla-ft-follow/<run>/epoch<N>
    # the expert's recording flight, the VLA flying its windows
    .\\scripts\\run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 ... `
        -RecordVla -RecordPerturbVla 127.0.0.1:8790 -Seed 201
    python demo/follow_vlm.py ... --record-vla <dir> --record-perturb-vla 127.0.0.1:8790
    # then, as for any recording
    python tools/build_follow_dataset.py demo/out/<tag> ... --out <dataset>

WHY

The fine-tune cloned the hand-written follow from its own flights (tools/
build_follow_dataset.py -> training/finetune_aerialvla_follow.py: held-out
step MAE fwd 2.47 -> 0.38 m, dpsi 0.103 -> 0.050 rad). Behaviour cloning only
ever shows the model the states the EXPERT visited; flying, the model's own
small errors take it into states nobody demonstrated (covariate shift), and
it has no label for getting back. DAgger (Ross, Gordon and Bagnell, 2011, "A
Reduction of Imitation Learning and Structured Prediction to No-Regret Online
Learning") fixes that by letting the learner fly and labelling what the
expert does from the states the learner produced. DART (--record-perturb,
vla_record.Perturber) knocks the expert off with random yaw noise; this
knocks it off with the MODEL's own mistakes, which is the state distribution
the model will meet.

WHAT A WINDOW IS (WindowSchedule)

A seeded schedule in mission seconds, its own random.Random(--seed): the
first window opens 6-10 s after the mission starts, then one every 6-10 s
(onset to onset), each 2.5-4.0 s long (--perturb-vla-every / --perturb-vla-dur).
AerialVLA is asked from VLA_LEAD_S (1.5 s) before each window opens, so its
first answer - about 0.8 s capture to answer for the merged bf16 server -
is in when the window opens. A window and its lead-in never overlap the
previous window (refused otherwise).

WHAT FLIES IN A WINDOW (VLAPerturber.apply)

Placed exactly where vla_record.Perturber is (follow_vlm, one call site):
AFTER the expert's controller and its slew limiter, BEFORE the Shield, which
judges and repairs whatever is executed. Inside a window the executed command
is the one demo/vla_fly.py's follow mission would fly at that moment:

  * its StepTracker, with vla_fly's OWN follow TrackerConfig - built by
    vla_fly.tracker_config from vla_fly's own parser for `--mission follow
    --cadence pipelined` (follow_args), so turn while moving, the altitude
    held (this flight's --cruise-alt), steps anchored where the answer is
    taken, LAND held as arrival, and the smoothing (2.5 m/s^2, 0.6 rad/s^2,
    1.0 s carry) are vla_fly's, not a copy that could drift; the cruise and
    speed cap are what every VLA follow since #3 flew (4.5 / 5.0 m/s, their
    metrics.params); the altitude clamp is the policy's band, as with the
    Shield on;
  * on AerialVLA's latest step, the aircraft's pose each tick, told whether
    an answer is pending (the carry) and whether the expert's box is close
    (vla_mission.BoxClose: the horizontal motion held, as MissionLoop does);
  * asked, pipelined - the next request as soon as an answer is back - on
    this module's background thread (`vla-dagger`), which builds the
    request (vla_mission.VLAFrames: the newest VLAFront+VLADown pair paired
    by sim stamp, the capture pose from the loop's pose ring), takes the
    phrase from vla_mission.DetectorHint over THIS follow's locked box
    (vla_record.box_from_grounder, vla_fly's latest_box rule; the bearing
    rotated by the turn since the box's frame, exactly as vla_fly), and
    sends it through vla_link.RemoteVLA, whose own worker does the socket.
    The 10 Hz loop never waits for any of it: it takes the answers the
    thread collected, hands them to the tracker and ticks it.

THE FOLLOW FINE-TUNE'S TILE AND RATE (2026-10-24): follow_vlm
--perturb-vla-bottom-tile / --perturb-vla-prev-dt-s / --perturb-vla-step-as-rate
are handed to vla_fly's own parser as --vla-bottom-tile / --vla-prev-dt-s /
--step-as-rate (follow_args), so a window asks with vla_fly's VLAFrames tile
(the front frame from dt_s earlier in the down slot) and flies vla_fly's rate
tracker; a server whose adapter was trained on another lower tile is refused
before the simulator is touched (vla_mission.bottom_tile_check), and every
window tick of a rate tracker records its commanded rates (`vla_rate`).

THE HAND-OVER. An answer that arrives in the lead-in is PARKED (the newest
wins; a link failure or a server error drops it, as the tracker drops a step
on one) and taken at the window's first tick, anchored where the aircraft is
then: taken on arrival, its step would be half flown by the EXPERT before
the VLA got the aircraft. Answers inside the window are taken on arrival, as
in vla_fly. At the opening the tracker's smoothing is told the command in
force (StepTracker.note_flown: the expert's, the tick before), so the VLA
ramps from what the aircraft is doing instead of jumping, and every window
tick the expert flies (below) tells it again. At the window's end the tracker
is suspended and the expert's own command flies on the next tick - its slew
limiter never saw the VLA's command, so its state is the expert's throughout,
as with vla_record.Perturber; the executed command steps back to the
expert's, and the Shield judges that step like any other.

NO SILENT WINDOW. A window tick flies the VLA only while the tracker flies a
model decision (a step, or a LAND held as arrival; author vla_step or hold
with a seq). No answer yet, a slow or refusing server, a malformed answer, a
link failure (the tracker hovers): the expert flies that tick, and its record
says the tracker's author and reason; the summary lists every window the VLA
never flew. The prompt's memory starts empty each window (DetectorHint.forget
at the lead-in): the VLA pilot's hint has only the sightings it was asked on.

THE RECORDS. Every tick's `perturb` (follow_vlm's rows, the recorder's track
line and the captures): source "vla", `active` = the executed command was the
VLA's - what the dataset builder reads, so labels whose horizon overlaps a
VLA-flown tick are dropped and the recovery after it kept, the label rule
and its meaning unchanged (tools/build_follow_dataset.py) - `window`,
`querying`, `episode`, `flying`, the tracker's author and reason, the
decision's seq, kind, bins and decoded step, its capture-to-answer latency,
phrase and hint kind, and the tracker's own command (`vla_cmd`, also when the
expert flew). In the --record-vla folder: vla_steps.jsonl (one line per
request, vla_link's record with what became of it) and vla_frames/ (each
request's JPEGs by SHA-256); the summary goes to record.json and metrics.json
(vla_record.record_summary).

SAFETY. The Shield filters every executed command, the VLA's included (the
call site is unchanged). Nothing here raises into the loop: a fault in a tick
is counted and the expert's command returned; the background thread counts
its own; a dead server is link failures (the tracker hovers, the expert
flies).

No projectairsim, cv2, Pillow or torch at module level (VLAFrames' default
JPEG encoder imports cv2 when a frame is encoded, in the flight's env).
"""
from __future__ import annotations

import math
import random
import sys
import threading
import time
from collections import Counter, deque
from dataclasses import asdict
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.vla_link import (VLA_STEPS_SCHEMA, LinkError, Pose,   # noqa: E402
                                ProtocolError, RemoteVLA, StepLog, StepTracker,
                                TrackerConfig, build_infer_fields, combine_alt_bands,
                                decision_from_result, identity_of, make_record,
                                policy_alt_band)

import vla_mission as VM                                             # noqa: E402

SOURCE = "vla"
VLA_EVERY_S = (6.0, 10.0)        # onset to onset
VLA_DUR_S = (2.5, 4.0)
VLA_LEAD_S = 1.5                 # asked this long before a window opens
# The cruise and speed cap of every VLA follow since #3 (vla_fly
# --cruise-mps 4.5 --speed-cap-mps 5, their metrics.params; vla_fly's own
# defaults, 1.0 / 2.0, are the GOTO's).
FOLLOW_CRUISE_MPS = 4.5
FOLLOW_SPEED_CAP_MPS = 5.0
STEPS = "vla_steps.jsonl"
FRAMES_DIR = "vla_frames"
_FLYING_AUTHORS = ("vla_step", "hold")


class DaggerRefused(ValueError):
    """A DAgger recording that cannot fly honestly (no server, not AerialVLA,
    windows that do not fit): refused before the simulator is touched."""


def _r(v, n: int = 4):
    return None if v is None else round(float(v), n)


def _act(a) -> dict | None:
    if a is None:
        return None
    return {k: float(getattr(a, k)) for k in ("vx", "vy", "vz_up", "yaw_rate")}


def _spread(vals) -> dict:
    """{n, p50, p95, max} (vla_mission.spread's percentiles)."""
    sp = VM.spread(vals)
    return {"n": sp["n"], "p50": sp["median"], "p95": sp["p95"], "max": sp["max"]}


# --------------------------------------------------------------------------- #
# The windows
# --------------------------------------------------------------------------- #
class WindowSchedule:
    """Seeded windows in mission seconds (module docstring, WHAT A WINDOW IS):
    each episode {k, lead_on_s, on_s, off_s}, drawn in order from its own
    random.Random(seed) (the flight's global RNG is not touched), so asking
    late first never changes the early ones."""

    def __init__(self, seed: int, every_s=VLA_EVERY_S, dur_s=VLA_DUR_S,
                 lead_s: float = VLA_LEAD_S):
        for name, rng in (("every_s", every_s), ("dur_s", dur_s)):
            lo, hi = (float(rng[0]), float(rng[1]))
            if not (0 < lo <= hi and math.isfinite(hi)):
                raise ValueError(f"{name} must be 0 < lo <= hi, got {tuple(rng)!r}")
        if not (math.isfinite(lead_s) and lead_s >= 0):
            raise ValueError(f"lead_s must be 0 or more, got {lead_s!r}")
        if not float(dur_s[1]) + float(lead_s) < float(every_s[0]):
            raise ValueError(f"a {float(dur_s[1]):g} s window and its {float(lead_s):g} s "
                             f"lead-in must end before the next lead-in: the shortest period "
                             f"({float(every_s[0]):g} s) is too short")
        self.seed = int(seed)
        self.every_s = (float(every_s[0]), float(every_s[1]))
        self.dur_s = (float(dur_s[0]), float(dur_s[1]))
        self.lead_s = float(lead_s)
        self._rng = random.Random(self.seed)
        self.episodes: list[dict] = []

    def _draw_until(self, s: float) -> None:
        while not self.episodes or self.episodes[-1]["lead_on_s"] <= s:
            prev = self.episodes[-1]["on_s"] if self.episodes else 0.0
            on = prev + self._rng.uniform(*self.every_s)
            dur = self._rng.uniform(*self.dur_s)
            self.episodes.append({"k": len(self.episodes), "lead_on_s": on - self.lead_s,
                                  "on_s": on, "off_s": on + dur})

    def episode(self, k: int) -> dict:
        while len(self.episodes) <= k:
            self._draw_until(self.episodes[-1]["off_s"] if self.episodes else 0.0)
        return dict(self.episodes[k])

    def state(self, s: float) -> dict:
        """{window, querying, episode} at mission second s: `querying` from
        the lead-in to the window's end, `window` from its opening."""
        s = float(s)
        self._draw_until(s)
        for e in reversed(self.episodes):
            if e["lead_on_s"] <= s:
                if s < e["off_s"]:
                    return {"window": s >= e["on_s"], "querying": True, "episode": e["k"]}
                break
        return {"window": False, "querying": False, "episode": None}


def check_args(addr: str, every_s, dur_s, lead_s: float = VLA_LEAD_S, *,
               bottom_tile: str = "down", prev_dt_s: float | None = None,
               step_as_rate: float | None = None) -> str | None:
    """Why follow_vlm should refuse --record-perturb-vla with these, or None.
    The lower tile and the rate are vla_fly's own flags: vla_fly's parser
    judges them (follow_args)."""
    try:
        VM.parse_addr(addr)
    except ValueError as exc:
        return f"--record-perturb-vla: {exc}"
    try:
        WindowSchedule(0, every_s, dur_s, lead_s)
    except ValueError as exc:
        return f"--perturb-vla-every / --perturb-vla-dur: {exc}"
    try:
        follow_args("a red car", cruise_alt=10.0, bottom_tile=bottom_tile, prev_dt_s=prev_dt_s,
                    step_as_rate=step_as_rate)
    except DaggerRefused as exc:
        return (f"--perturb-vla-bottom-tile / --perturb-vla-prev-dt-s / "
                f"--perturb-vla-step-as-rate: {exc}")
    return None


# --------------------------------------------------------------------------- #
# vla_fly's follow mission, from vla_fly's own code
# --------------------------------------------------------------------------- #
def follow_args(object_text: str, *, cruise_alt: float,
                cruise_mps: float = FOLLOW_CRUISE_MPS,
                speed_cap_mps: float = FOLLOW_SPEED_CAP_MPS, bottom_tile: str = "down",
                prev_dt_s: float | None = None, step_as_rate: float | None = None):
    """(vla_fly's resolved arguments for a pipelined follow, the argv): its
    own parser and resolve_args, so every default the follow mission applies
    - the tracker's, the hint's, the link's, the lower tile (--vla-bottom-tile
    / --vla-prev-dt-s) and the rate execution (--step-as-rate) - is
    vla_fly's. The tile and rate flags go into the argv only when set, so a
    default DAgger recording's argv is as it was."""
    import vla_fly as VF
    from guardrail.aerialvla_codec import BOTTOM_TILE_MODES
    if bottom_tile not in BOTTOM_TILE_MODES:
        raise DaggerRefused(f"lower tile {bottom_tile!r} is not one of {list(BOTTOM_TILE_MODES)}")
    argv = ["--mission", "follow", "--cadence", "pipelined", f"--object={object_text}",
            "--cruise-mps", f"{float(cruise_mps):g}", "--speed-cap-mps",
            f"{float(speed_cap_mps):g}", "--cruise-alt", f"{float(cruise_alt):g}"]
    if bottom_tile != "down":
        argv += ["--vla-bottom-tile", str(bottom_tile)]
    if prev_dt_s is not None:
        argv += ["--vla-prev-dt-s", repr(float(prev_dt_s))]
    if step_as_rate is not None:
        argv += ["--step-as-rate", repr(float(step_as_rate))]
    try:
        return VF.resolve_args(VF.build_arg_parser().parse_args(argv)), argv
    except ValueError as exc:
        raise DaggerRefused(f"vla_fly refuses this follow ({' '.join(argv)}): {exc}") from None
    except SystemExit:
        # argparse itself (a lower tile that is not one of its choices)
        raise DaggerRefused(f"vla_fly's parser refuses {' '.join(argv)}") from None


def follow_tracker_config(vargs) -> TrackerConfig:
    """vla_fly.tracker_config for the follow, with the pilot band of a Shield
    that is on (vla_mission.pilot_alt_config: the policy's band is applied by
    the tracker itself, from the policy)."""
    import vla_fly as VF
    _policy, band, _rec = VM.pilot_alt_config(True, None)
    return VF.tracker_config(vargs, band, TrackerConfig)


def follow_hint_config(vargs) -> dict:
    """DetectorHint's and the close rule's settings for vla_fly's follow."""
    return {"fresh_s": vargs.det_fresh_s, "memory_s": vargs.follow_memory_s,
            "search_s": vargs.follow_search_s,
            "close": VM.BoxClose(vargs.close_bottom_frac, vargs.close_width_frac),
            "bearing_gain": vargs.bearing_gain}


def latest_box_from(source: Callable[[], dict | None]) -> Callable[[], tuple]:
    """follow_vlm.Grounder.latest -> DetectorHint's latest(): (det, t of its
    frame, the heading at its frame, t published), vla_fly's latest_box rule
    through vla_record.box_from_grounder (the recorder's own reading)."""
    from vla_record import box_from_grounder

    def latest():
        b = box_from_grounder(source())
        if b is None:
            return None, None, None, None
        return b["det"], b["t_box"], b["yaw_box"], b["t_published"]
    return latest


# --------------------------------------------------------------------------- #
# The server
# --------------------------------------------------------------------------- #
def connect_server(addr: str, *, allow_stand_in: bool = False,
                   response_timeout_s: float = 10.0, late_reply_wait_s: float = 0.0,
                   connect_timeout_s: float = 2.0) -> tuple[RemoteVLA, dict, bool, str | None]:
    """(the connected client, the hello's model identity, is it AerialVLA,
    why not). The identity is read first on a probe connection, so the
    flight's client is built strict exactly when it is a VLA (vla_fly's
    rule: strict_identity=is_vla) and pins it with its own hello. Refused
    (DaggerRefused) when nothing answers hello, or when the server is not
    AerialVLA (vla_mission.identity_is_vla: the scripted stand-in, a fake:
    id, a repeated-digit digest) unless `allow_stand_in`."""
    try:
        host, port = VM.parse_addr(addr)
    except ValueError as exc:
        raise DaggerRefused(str(exc)) from None
    probe = RemoteVLA(host, port, connect_timeout_s=connect_timeout_s,
                      response_timeout_s=response_timeout_s, strict_identity=False,
                      clock=time.time)
    try:
        identity = probe.connect()
    except (LinkError, ProtocolError, OSError) as exc:
        raise DaggerRefused(f"no VLA server answers hello at {addr}: {exc}") from None
    finally:
        probe.close()
    is_vla, why_not = VM.identity_is_vla(identity)
    if not is_vla and not allow_stand_in:
        raise DaggerRefused(f"the server at {addr} is not AerialVLA ({why_not}); "
                            f"--perturb-vla-allow-stand-in flies a stand-in, recorded as one")
    client = RemoteVLA(host, port, connect_timeout_s=connect_timeout_s,
                       response_timeout_s=response_timeout_s, strict_identity=is_vla,
                       late_reply_wait_s=late_reply_wait_s, clock=time.time)
    try:
        pinned = client.connect()
    except (LinkError, ProtocolError, OSError) as exc:
        client.close()
        raise DaggerRefused(f"the VLA server at {addr} went away after hello: {exc}") from None
    if identity_of(pinned) != identity_of(identity):
        client.close()
        raise DaggerRefused(f"the server at {addr} changed its model between two hellos")
    return client, dict(identity), is_vla, why_not


# --------------------------------------------------------------------------- #
# The perturber
# --------------------------------------------------------------------------- #
class VLAPerturber:
    """vla_record.Perturber's interface and placement, AerialVLA as the
    perturbation (module docstring). The loop calls apply(action, s,
    pose=(x, y, up, yaw)) once per tick; start(t0) / stop() bracket the
    mission (ExpertVLARecorder calls both, and feeds put_front / put_down
    with the VLA cameras' messages). Thread ownership: the tracker, the
    schedule and the records are the loop's; the client's submit and poll,
    the capture and the hint are the background thread's; they meet in one
    locked deque of answers and the episode the thread should ask for."""

    def __init__(self, seed: int, *, client, get_capture: Callable, latest_box: Callable,
                 object_text, tracker_config: TrackerConfig, policy=None,
                 every_s=VLA_EVERY_S, dur_s=VLA_DUR_S, lead_s: float = VLA_LEAD_S,
                 fresh_s: float = 1.0, memory_s: float = 6.0, search_s: float = 20.0,
                 close=None, bearing_gain: float = 1.0, identity: dict | None = None,
                 address: str | None = None, is_vla: bool | None = None,
                 why_not: str | None = None, config: dict | None = None, frames=None,
                 record_dir=None, clock: Callable[[], float] = time.time,
                 poll_s: float = 0.01, retry_s: float = 0.1,
                 log: Callable[[str], Any] | None = None):
        from vla_bridge import VLABridge
        self.seed = int(seed)
        self.schedule = WindowSchedule(seed, every_s, dur_s, lead_s)
        self.client = client
        self.get_capture = get_capture
        self.latest_box = latest_box
        self.object_text = object_text
        self.cfg = tracker_config
        self.tracker = StepTracker(tracker_config)
        info = policy_alt_band(policy)
        band, source = combine_alt_bands(tracker_config.alt_band, info["band"])
        self.tracker.set_alt_band(band)
        self.alt_band = {"band": None if band is None else list(band), "source": source,
                         "policy": None if info["band"] is None else list(info["band"]),
                         "envelopes": info["envelopes"]}
        self.close = close
        obj0 = object_text() if callable(object_text) else object_text
        bridge = VLABridge(str(obj0), hfov_deg=90.0, bearing_gain=bearing_gain)
        # vla_fly's follow hint, without its start gate: the expert's lock
        # has already acquired the subject before the mission clock starts.
        self.detector_hint = VM.DetectorHint(self.latest_box, bridge, fresh_s=fresh_s,
                                             memory_s=memory_s, search_s=search_s,
                                             close=close, record_kind=True)
        self.hint = VM.HintSource(self.detector_hint, "detector")
        self.identity = dict(identity or getattr(client, "identity", None) or {})
        self.address = address
        if is_vla is None:
            is_vla, why_not = VM.identity_is_vla(self.identity)
        self.is_vla, self.why_not = bool(is_vla), why_not
        self.config = dict(config or {})
        self.config.setdefault("tracker", asdict(tracker_config))
        self.frames = frames
        self.record_dir = None if record_dir is None else Path(record_dir)
        self.clock = clock
        self.poll_s, self.retry_s = float(poll_s), float(retry_s)
        self.log = log
        # The loop's
        self.t0: float | None = None
        self.records: list[dict] = []
        self.refused: Counter = Counter()
        self.answers: Counter = Counter()
        self.windows: list[dict] = []
        self.n_ticks = self.n_active = self.n_window = self.n_querying = 0
        self.last_link_failure: dict | None = None
        self._cur: dict | None = None            # the episode being asked / flown
        self._t_last_end = -math.inf             # when the last window ended
        self._parked = None                      # a lead-in answer, until the window
        self._last_exec: tuple | None = None     # (the command executed, when)
        self._latency: list[float] = []
        self._rtt: list[float] = []
        self.steps_log: StepLog | None = None
        # Shared with the background thread (under _lock)
        self._lock = threading.Lock()
        self._results: deque = deque()
        self._want_k: int | None = None
        self.counts: Counter = Counter()
        self.errors: list[str] = []
        self.n_errors = 0
        # The background thread's
        self._meta: dict[int, tuple[dict, int]] = {}
        self._asked_k: int | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._summary_at_stop: dict | None = None
        self.tracker.suspend(self.clock())       # nothing is taken before a lead-in

    # ---- Perturber's interface ---------------------------------------------
    @property
    def episodes(self) -> list[dict]:
        return self.schedule.episodes

    def state(self, s: float) -> dict:
        return self.schedule.state(s)

    def put_front(self, msg) -> None:
        if self.frames is not None:
            self.frames.put_front(msg)

    def put_down(self, msg) -> None:
        if self.frames is not None:
            self.frames.put_down(msg)

    def apply(self, action, s: float, *, pose=None):
        """(the action to send to the Shield, the tick's record). `pose`:
        (x, y, up, yaw rad) of this tick's kinematics read. Outside a
        window, and inside one the VLA is not flying, `action` itself is
        returned. Never raises: a fault is counted and the expert flies."""
        now = self.clock()
        try:
            out, rec = self._apply(action, float(s), pose, now)
        except Exception as exc:                                  # noqa: BLE001
            self._error(exc)
            out = action
            rec = {"source": SOURCE, "active": False, "error": f"{type(exc).__name__}: {exc}"}
        self.n_ticks += 1
        self.n_active += int(bool(rec.get("active")))
        self._last_exec = (out, now)
        return out, rec

    # ---- lifecycle ---------------------------------------------------------
    def start(self, t0: float, thread: bool = True) -> None:
        """The mission clock started at `t0`: open vla_steps.jsonl (rewritten,
        as the recorder's files) and vla_frames/, start the background thread
        (`thread=False`: the caller runs _service_once itself - the tests)."""
        if self.t0 is not None:
            return
        self.t0 = float(t0)
        if self.record_dir is not None:
            self.record_dir.mkdir(parents=True, exist_ok=True)
            p = self.record_dir / STEPS
            if p.is_file() and not p.is_symlink():
                p.unlink()
            self.steps_log = StepLog(p)
            if self.frames is not None and getattr(self.frames, "saver", None) is None:
                self.frames.saver = VM.FrameSaver(self.record_dir / FRAMES_DIR)
        if thread:
            self._thread = threading.Thread(target=self._run, name="vla-dagger", daemon=True)
            self._thread.start()

    def stop(self, timeout_s: float = 10.0) -> dict:
        """Stop asking, close the open window, give every request its line
        (one still in flight: refused "closed"), close the link, the log and
        the frames; the summary. Safe to call twice and before start()."""
        if self._summary_at_stop is not None:
            return self._summary_at_stop
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
        now = self.clock()
        try:
            if self._cur is not None:
                self._end_episode(now)
            self._drain(now, None, False)
            self.tracker.close(now)
            self._flush()
        except Exception as exc:                                  # noqa: BLE001
            self._error(exc)
        try:
            self.client.close()
            late = self.client.poll()
            if late is not None and late.seq in self._meta:
                meta, _k = self._meta.pop(late.seq)
                rec = make_record(meta, late)
                rec.update({"accepted": False, "refused": "closed", "target": None,
                            "outcome": None})
                self._emit(rec)
            for seq, (meta, _k) in sorted(self._meta.items()):
                self._emit({"schema": VLA_STEPS_SCHEMA, "seq": seq, **meta,
                            "kind": "unanswered", "accepted": False, "refused": "closed",
                            "target": None, "outcome": None})
            self._meta.clear()
        except Exception as exc:                                  # noqa: BLE001
            self._error(exc)
        for closer in (getattr(self.steps_log, "close", None),
                       getattr(getattr(self.frames, "saver", None), "close", None)):
            try:
                if closer is not None:
                    closer()
            except Exception as exc:                              # noqa: BLE001
                self._error(exc)
        self._summary_at_stop = self.summary()
        return self._summary_at_stop

    # ---- the loop's side ---------------------------------------------------
    def _apply(self, action, s: float, pose, now: float):
        st = self.schedule.state(s)
        k = st["episode"] if st["querying"] else None
        if self._cur is not None and self._cur["k"] != k:
            self._end_episode(now)
        if k is not None and self._cur is None:
            self._begin_episode(k, now)
        pose_now = None if pose is None else Pose(*(float(v) for v in pose))
        if st["window"] and pose_now is None:
            raise ValueError("a window tick needs the aircraft's pose")
        # On the opening tick an answer that just came in is parked too, so
        # the newest of the lead-in's answers is the one taken at the opening.
        opening = st["window"] and self._cur is not None and not self._cur["opened"]
        self._drain(now, pose_now, st["window"] and not opening)
        rec = {"source": SOURCE, "active": False, "window": st["window"],
               "querying": k is not None, "episode": k}
        if k is None:
            return action, rec
        self.n_querying += 1
        ep = self._cur
        if not st["window"]:
            ep["lead_ticks"] += 1
            rec.update({"flying": False,
                        "parked_seq": None if self._parked is None else self._parked.seq})
            return action, rec
        self.n_window += 1
        ep["ticks"] += 1
        if not ep["opened"]:
            self._open_window(ep, now, pose_now, action)
        hold = self._close_now(now)
        tk = self.tracker.tick(now, pose_now, pending=bool(self.client.busy),
                               hold_horizontal=hold)
        flying = (tk.seq is not None and tk.author in _FLYING_AUTHORS
                  and tk.reason != "parse_fail" and not tk.land_requested)
        rec.update({"flying": flying, "active": flying, "author": tk.author,
                    "reason": tk.reason, "seq": tk.seq, "age_s": _r(tk.age_s),
                    "close_hold": hold, "vla_cmd": _act(tk.action)})
        if self.cfg.rate_horizon_s is not None:
            # vla_fly --step-as-rate: the commanded rates (None off a rate tick)
            rec["vla_rate"] = self.tracker.last_rate
        cur = self.tracker.current_record if tk.seq is not None else None
        if cur is not None:
            rec.update({"kind": cur.get("kind"), "step": cur.get("step"),
                        "bins": cur.get("bins"), "land": cur.get("land"),
                        "latency_s": _r(cur.get("latency_client_s")),
                        "phrase": cur.get("phrase"), "hint_kind": cur.get("hint_kind")})
        if flying:
            ep["vla_ticks"] += 1
            if ep["first_vla_s"] is None:
                ep["first_vla_s"] = round(s - ep["on_s"], 3)
            return tk.action, rec
        # The expert flies this tick: the tracker's smoothing is told so.
        ep["why_not"][f"{tk.author}:{tk.reason}"] += 1
        self.tracker.note_flown(action, now)
        return action, rec

    def _begin_episode(self, k: int, now: float) -> None:
        e = self.schedule.episode(k)
        self._cur = {"k": k, "lead_on_s": round(e["lead_on_s"], 3), "on_s": round(e["on_s"], 3),
                     "off_s": round(e["off_s"], 3), "opened": False,
                     "lead_ticks": 0, "ticks": 0, "vla_ticks": 0, "first_vla_s": None,
                     "answer_at_open": None, "answers": Counter(), "why_not": Counter()}
        if self.tracker.land_requested:
            self.tracker.clear_land()
        # Decisions captured before the last window ended are from before
        # this one (its first capture is a frame older than `now`, so the
        # line is drawn there; answers to an earlier window's requests are
        # refused by their episode in _drain as well).
        self.tracker.resume(self._t_last_end)
        self._parked = None
        with self._lock:
            self._want_k = k

    def _open_window(self, ep: dict, now: float, pose_now: Pose, action) -> None:
        ep["opened"] = True
        prev, t_prev = self._last_exec if self._last_exec is not None else (action, None)
        dt = self.cfg.smooth_tick_s
        if t_prev is None or not 0.0 < now - t_prev <= 2.0 * dt:
            t_prev = now - dt
        self.tracker.note_flown(prev, t_prev)
        if self._parked is not None:
            d, self._parked = self._parked, None
            ok = self.tracker.accept(d, now, pose_now=pose_now)
            ep["answer_at_open"] = d.kind if ok else f"refused_{d.record.get('refused')}"
            self._flush()

    def _end_episode(self, now: float) -> None:
        with self._lock:
            self._want_k = None
        self._drop_parked("window_ended")
        self.tracker.suspend(now)
        self._t_last_end = now
        self._flush()
        ep, self._cur = self._cur, None
        out = {k: v for k, v in ep.items() if k != "opened"}
        out["answers"] = dict(ep["answers"])
        out["why_not"] = dict(ep["why_not"])
        with self._lock:
            out["requests"] = self.counts.get(f"requests_k{ep['k']}", 0)
        self.windows.append(out)
        if self.log is not None:
            try:
                self.log(f"[vla-dagger] window {out['k']} (t+{out['on_s']:.1f}-"
                         f"{out['off_s']:.1f} s): AerialVLA flew {out['vla_ticks']} of "
                         f"{out['ticks']} ticks"
                         + ("" if out["vla_ticks"] else
                            f" - the expert flew it ({', '.join(sorted(out['why_not']))})"))
            except Exception:                                     # noqa: BLE001
                pass

    def _drain(self, now: float, pose_now: Pose | None, in_window: bool) -> None:
        """Hand the answers the background thread collected to the tracker,
        as RemoteVLABackend._drain does - except that in a lead-in the newest
        answer is parked for the window (module docstring, THE HAND-OVER)."""
        while True:
            with self._lock:
                if not self._results:
                    break
                res, meta, k_req = self._results.popleft()
            kind = res.reply.kind if res.reply is not None else "link_failure"
            self.answers[kind] += 1
            if self._cur is not None and self._cur["k"] == k_req:
                self._cur["answers"][kind] += 1
            if res.reply is not None and res.reply.ok:
                self._latency.append(res.t_done - float(meta["t_capture"]))
            self._rtt.append(res.t_done - res.t_sent)
            try:
                dec = decision_from_result(meta, res)
            except Exception as exc:                              # noqa: BLE001
                self._error(exc)
                rec = {"schema": VLA_STEPS_SCHEMA, "seq": res.seq, **meta, "kind": kind,
                       "accepted": False, "refused": "drain_error",
                       "drain_error": f"{type(exc).__name__}: {exc}", "target": None,
                       "outcome": None}
                self._emit(rec)
                self._drop_parked("lost_in_lead_in")
                self.tracker.link_lost(now, f"drain: {type(exc).__name__}: {exc}")
                continue
            if dec is None:
                rec = make_record(meta, res)
                rec.update({"accepted": False, "refused": "link_failure", "target": None,
                            "outcome": None})
                self.last_link_failure = {"kind": res.failure, "seq": res.seq, "t": now,
                                          "detail": res.detail}
                self._emit(rec)
                self._drop_parked("lost_in_lead_in")
                self.tracker.link_lost(now, res.detail)
                continue
            if self._cur is not None and k_req != self._cur["k"]:
                # Asked for an earlier window: from a scene that is gone.
                dec.record.update({"accepted": False, "refused": "previous_window",
                                   "target": None, "outcome": None})
                self._emit(dec.record)
                continue
            if self._cur is not None and not in_window and dec.kind != "server_error":
                self._drop_parked("superseded_in_lead_in")
                self._parked = dec
                continue
            if dec.kind == "server_error":
                self._drop_parked("lost_in_lead_in")
            self.tracker.accept(dec, now, pose_now=pose_now)
        self._flush()

    def _drop_parked(self, why: str) -> None:
        if self._parked is None:
            return
        d, self._parked = self._parked, None
        d.record.update({"accepted": False, "refused": why, "target": None, "outcome": None})
        self._emit(d.record)

    def _close_now(self, now: float) -> bool:
        """MissionLoop's close hold: a fresh box (not one a reset dropped)
        that vla_mission.BoxClose calls close."""
        if self.close is None:
            return False
        try:
            det, t_box, _yaw, _pub = VM.unpack_box(self.latest_box())
            return bool(det is not None and t_box is not None
                        and now - float(t_box) <= self.detector_hint.fresh_s
                        and not self.detector_hint.box_ignored(t_box) and self.close(det))
        except Exception as exc:                                  # noqa: BLE001
            self._error(exc)
            return False

    def _flush(self) -> None:
        for rec in self.tracker.pop_finished():
            self._emit(rec)

    def _emit(self, rec: dict) -> None:
        self.records.append(rec)
        if rec.get("refused"):
            self.refused[rec["refused"]] += 1
        if self.steps_log is not None:
            self.steps_log.write(rec)           # queued; written off this thread

    def _error(self, exc: BaseException) -> None:
        with self._lock:
            self.n_errors += 1
            if len(self.errors) < 10:
                self.errors.append(f"{type(exc).__name__}: {exc}")

    # ---- the background thread ---------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            wait = self._service_once()
            if wait > 0:
                self._stop.wait(wait)

    def _service_once(self) -> float:
        """One round: collect a finished answer, then send the next request
        if the window's episode wants one and nothing is in flight. Returns
        how long to wait before the next round."""
        wait = self.poll_s
        try:
            res = self.client.poll()
            if res is not None:
                got = self._meta.pop(res.seq, None)
                if got is None:
                    self._count("unknown_reply")
                else:
                    with self._lock:
                        self._results.append((res, got[0], got[1]))
            with self._lock:
                k = self._want_k
            if k is not None and k != self._asked_k:
                # A new lead-in: the VLA pilot's memory starts with it.
                self._asked_k = k
                self.detector_hint.forget()
            if (k is not None and not self.client.busy
                    and not getattr(self.client, "backing_off", False)):
                if self._request(k) in ("no_capture", "no_hint"):
                    wait = self.retry_s
        except Exception as exc:                                  # noqa: BLE001
            self._error(exc)
            wait = self.retry_s
        return wait

    def _request(self, k: int) -> str:
        cap = self.get_capture()
        if cap is None:
            self._count("no_capture")
            return "no_capture"
        hint = self.hint(cap)
        if hint is None:
            self._count("no_hint")
            return "no_hint"
        obj = self.object_text() if callable(self.object_text) else self.object_text
        fields, meta = build_infer_fields(cap, hint, obj)
        seq = self.client.submit(fields)
        if seq is None:
            self._count("submit_refused")
            return "refused"
        self._meta[seq] = (meta, k)        # this thread polls too: no reply can beat it
        self._count("requests")
        self._count(f"requests_k{k}")
        if self.frames is not None and hasattr(self.frames, "commit"):
            self.frames.commit(cap, hint, seq)
        return "sent"

    def _count(self, key: str) -> None:
        with self._lock:
            self.counts[key] += 1

    # ---- what it did -------------------------------------------------------
    def server_record(self) -> dict:
        from guardrail.aerialvla_codec import adapter_variant
        ident = self.identity
        adapter = ident.get("adapter") if isinstance(ident.get("adapter"), dict) else None
        return {"address": self.address, "identity": identity_of(ident),
                "adapter": (None if adapter is None else
                            {k: adapter.get(k) for k in ("path", "sha256", "variant")}),
                "adapter_variant": adapter_variant(ident.get("id")),
                "model_id": VM.model_id_for(ident) if self.is_vla else None,
                "is_vla": self.is_vla, "why_not_vla": self.why_not}

    def summary(self) -> dict:
        eps = list(self.windows)
        if self._cur is not None:
            eps.append({k: (dict(v) if isinstance(v, Counter) else v)
                        for k, v in self._cur.items() if k != "opened"})
        with self._lock:
            counts = dict(self.counts)
            errors = {"n": self.n_errors, "first": list(self.errors)}
        log = self.steps_log.snapshot() if self.steps_log is not None else None
        frames = self.frames.stats() if hasattr(self.frames, "stats") else None
        saver = getattr(self.frames, "saver", None)
        # A window counts once it opened (a flight can end in a lead-in).
        opened = [e for e in eps if e["ticks"]]
        return {
            "source": SOURCE, "seed": self.seed, "every_s": list(self.schedule.every_s),
            "dur_s": list(self.schedule.dur_s), "lead_s": self.schedule.lead_s,
            "ticks": self.n_ticks, "active_ticks": self.n_active, "vla_ticks": self.n_active,
            "window_ticks": self.n_window, "querying_ticks": self.n_querying,
            "lead_ins": len(eps), "windows": len(opened),
            "windows_flown": sum(1 for e in opened if e["vla_ticks"]),
            "windows_without_vla": [e["k"] for e in opened if not e["vla_ticks"]],
            "answer_at_open": sum(1 for e in opened
                                  if e.get("answer_at_open") in ("step", "land")),
            "first_vla_s": _spread(e["first_vla_s"] for e in opened),
            "requests": counts.get("requests", 0), "no_capture": counts.get("no_capture", 0),
            "no_hint": counts.get("no_hint", 0),
            "submit_refused": counts.get("submit_refused", 0),
            "answers": dict(self.answers), "accepted": self.tracker.counts.get("accepted", 0),
            "refused": dict(self.refused),
            "latency_s": _spread(self._latency), "rtt_s": _spread(self._rtt),
            "server": self.server_record(), "config": self.config,
            "alt_band": self.alt_band, "tracker": dict(self.tracker.counts),
            "ticks_by_author": dict(self.tracker.ticks),
            "hint": self.detector_hint.stats(self.t0), "phrases": self.hint.stats(),
            "frames": frames, "saver": None if saver is None else saver.stats(),
            "link": dict(getattr(self.client, "counts", {}) or {}),
            "late_replies": list(getattr(self.client, "late", None) or []),
            "last_link_failure": self.last_link_failure,
            "steps_log": log, "errors": errors,
            # The opened windows (vla_record.record_summary counts them and
            # sums their scheduled seconds as active_s).
            "episodes": opened,
            "rule": ("windows in mission seconds; asked from lead_s before each opens; inside a "
                     "window the executed command is vla_fly's follow StepTracker on AerialVLA's "
                     "latest step whenever it flies a model decision (active), else the "
                     "expert's; applied after the expert's controller and slew limiter, before "
                     "the Shield (which filters it)")}

    def describe(self) -> str:
        sv = self.server_record()
        e, d = self.schedule.every_s, self.schedule.dur_s
        c = self.cfg
        tile = getattr(self.frames, "tile", None)
        return (f"AerialVLA {sv['identity'].get('id')} at {self.address}"
                + ("" if self.is_vla else f" (*** STAND-IN, not a VLA: {self.why_not})")
                + f" flies {d[0]:g}-{d[1]:g} s every {e[0]:g}-{e[1]:g} s (seed {self.seed}), "
                f"asked from {self.schedule.lead_s:g} s before: vla_fly's follow tracker at "
                f"{c.cruise_mps:g}/{c.speed_cap_mps:g} m/s, held at {c.hold_altitude_m:g} m"
                + (f", steps as rates over {c.rate_horizon_s:g} s" if c.rate_horizon_s else "")
                + (f", lower tile the front frame {tile.dt_s:g} s earlier"
                   if tile is not None and tile.prev_front else "")
                + ", after the controller, before the Shield")


def summary_line(s: dict | None) -> str | None:
    """One console line for a finished DAgger recording, or None."""
    if not s or s.get("source") != SOURCE:
        return None
    lat = s.get("latency_s") or {}
    miss = s.get("windows_without_vla") or []
    return (f"[vla-dagger] {s['windows']} windows, AerialVLA flew {s['vla_ticks']} of "
            f"{s['window_ticks']} window ticks; {s['requests']} requests, answers "
            f"{s.get('answers')}, latency p50 {lat.get('p50')} s p95 {lat.get('p95')} s"
            + (f"; *** {len(miss)} window(s) flown by the expert alone: {miss}" if miss else ""))


def make_vla_perturber(addr: str, *, seed: int, obs, box_source: Callable[[], dict | None],
                       policy, object_text, cruise_alt: float, record_dir=None,
                       every_s=VLA_EVERY_S, dur_s=VLA_DUR_S, lead_s: float = VLA_LEAD_S,
                       allow_stand_in: bool = False, encode: Callable | None = None,
                       cruise_mps: float = FOLLOW_CRUISE_MPS,
                       speed_cap_mps: float = FOLLOW_SPEED_CAP_MPS,
                       bottom_tile: str = "down", prev_dt_s: float | None = None,
                       step_as_rate: float | None = None,
                       log: Callable[[str], Any] | None = None) -> VLAPerturber:
    """The --record-perturb-vla perturber of a follow_vlm flight: vla_fly's
    follow configuration (follow_args, with vla_fly's --vla-bottom-tile /
    --vla-prev-dt-s / --step-as-rate when given), hello and the identity
    check (connect_server; DaggerRefused), the lower tile the adapter was
    trained on against this one (vla_mission.bottom_tile_check, vla_fly's
    refusal; DaggerRefused), the VLA cameras' own VLAFrames over the
    flight's pose ring (`obs`) with vla_fly's tile, DetectorHint over
    `box_source` (follow_vlm.Grounder.latest). Nothing is started: start(t0)
    does that (the recorder calls it when the mission clock starts)."""
    obj0 = object_text() if callable(object_text) else object_text
    vargs, argv = follow_args(obj0, cruise_alt=cruise_alt, cruise_mps=cruise_mps,
                              speed_cap_mps=speed_cap_mps, bottom_tile=bottom_tile,
                              prev_dt_s=prev_dt_s, step_as_rate=step_as_rate)
    try:
        WindowSchedule(seed, every_s, dur_s, lead_s)
    except ValueError as exc:
        raise DaggerRefused(str(exc)) from None
    tcfg = follow_tracker_config(vargs)
    client, identity, is_vla, why_not = connect_server(
        addr, allow_stand_in=allow_stand_in, response_timeout_s=vargs.decision_timeout_s,
        late_reply_wait_s=vargs.late_reply_wait_s)
    tile_check = VM.bottom_tile_check(identity, vargs.bottom_tile, is_vla=is_vla)
    if not tile_check["ok"]:
        client.close()
        raise DaggerRefused(f"the server at {addr}: {tile_check['why']} (the adapter's lower "
                            f"tile from {tile_check['source']}; --perturb-vla-bottom-tile "
                            f"{vargs.bottom_tile.mode})")
    try:
        frames = VM.VLAFrames(obs, encode=encode, bottom_tile=vargs.bottom_tile)
        return VLAPerturber(
            seed, client=client, get_capture=frames.capture,
            latest_box=latest_box_from(box_source), object_text=object_text,
            tracker_config=tcfg, policy=VM.pilot_alt_config(True, policy)[0],
            every_s=every_s, dur_s=dur_s, lead_s=lead_s, **follow_hint_config(vargs),
            identity=identity, address=addr, is_vla=is_vla, why_not=why_not,
            config={"vla_fly_argv": argv, "cadence": vargs.cadence,
                    "decision_timeout_s": vargs.decision_timeout_s,
                    "late_reply_wait_s": vargs.late_reply_wait_s,
                    "hint": {"fresh_s": vargs.det_fresh_s, "memory_s": vargs.follow_memory_s,
                             "search_s": vargs.follow_search_s,
                             "close": [vargs.close_bottom_frac, vargs.close_width_frac],
                             "start_gate": None, "forget_at_lead_in": True},
                    "tracker": asdict(tcfg), "step_as_rate_s": vargs.step_as_rate,
                    "bottom_tile": tile_check},
            frames=frames, record_dir=record_dir, log=log)
    except BaseException:
        client.close()
        raise


__all__ = ["VLAPerturber", "WindowSchedule", "DaggerRefused", "make_vla_perturber",
           "connect_server", "follow_args", "follow_tracker_config", "follow_hint_config",
           "latest_box_from", "check_args", "summary_line", "SOURCE", "VLA_EVERY_S",
           "VLA_DUR_S", "VLA_LEAD_S", "FOLLOW_CRUISE_MPS", "FOLLOW_SPEED_CAP_MPS", "STEPS",
           "FRAMES_DIR"]
