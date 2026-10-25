"""
DAgger for the follow fine-tune: AerialVLA flies windows of the expert's
recording (demo/vla_dagger.py, follow_vlm --record-vla --record-perturb-vla
HOST:PORT, scripts/run_citylife_follow.ps1 -RecordPerturbVla), and the
dataset builder treats those windows as perturbations.

Run either way:
    pytest tests/test_vla_dagger.py -v
    python tests/test_vla_dagger.py

WHAT IS AT STAKE (2026-10-24)

The fine-tuned AerialVLA was cloned from the hand-written follow's own
flights (tools/build_follow_dataset.py, held-out step MAE fwd 2.47 -> 0.38 m),
and behaviour cloning only ever shows the model the states the EXPERT
visited. Flying, the model drifts into states it has never seen (covariate
shift). DAgger closes that gap: let the model fly, label what the expert does
from the states the model produced. So, pinned here:

  1. OFF BY DEFAULT, and the flight unchanged when off: no server is asked,
     nothing is imported, the one call site and the Shield call are as they
     were; refused without --record-vla, with --record-perturb, with a bad
     HOST:PORT or windows whose lead-in would overlap the previous window.
  2. THE WINDOWS are seeded (their own RNG from --seed) and in the asked
     ranges (default every 6-10 s, 2.5-4.0 s long), each asked from 1.5 s
     before it opens.
  3. WHAT FLIES IN A WINDOW is the command vla_fly's follow mission would fly:
     its StepTracker with vla_fly's own follow TrackerConfig (built by
     vla_fly.tracker_config from vla_fly's own parser) on AerialVLA's latest
     step, the pose each tick - checked tick by tick against an independent
     StepTracker fed the same step; the request is the VLAFront+VLADown pair,
     the capture pose and DetectorHint's phrase for the expert's box,
     yaw-corrected. Outside a window, and inside one with no step to fly
     (silent, refusing, malformed or timed-out server), the expert's own
     command object flies, and the record says why.
  4. THE HAND-OVER: the tracker's smoothing starts from the command in force
     (no jump into the VLA), and at the window's end the expert's own
     limiter output flies again - the limiter never saw the VLA's command.
  5. THE RECORDS: every tick's `perturb` (source "vla", the step's bins and
     decoded values, seq, capture-to-answer latency, phrase, flying or not),
     on the track lines and captures; record.json and metrics carry the
     summary (windows, VLA-flown ticks, answers, refusals, latency p50/p95,
     the server's identity with its adapter); vla_steps.jsonl one line per
     request.
  6. SAFETY: the Shield filters whatever flies (one call site, unchanged);
     a dead server, a protocol error or a fault inside the perturber never
     raises into the loop - the expert flies.
  7. THE LAUNCHER passes -RecordPerturbVla inside the -RecordVla block and
     refuses it, and its window flags, before the simulator starts.
  8. THE BUILDER drops every label whose horizon overlaps a VLA-flown tick
     (plus the 0.3 s margin) and keeps the recovery after it, with labels
     identical to the same flight's without the records.

No simulator, GPU or model: a stub client answered by the test on a fake
clock, and tests/vla_fake_server.py on an ephemeral loopback port (never
8790) with the real protocol-v2 client and threads.
"""
from __future__ import annotations

import ast
import json
import math
import socket
import sys
import tempfile
import time
from collections import Counter, deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools", ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import vla_link as L                                  # noqa: E402
from guardrail.models import Action4D                               # noqa: E402
from guardrail.vla_link import (Capture, Decision, Pose, Step,       # noqa: E402
                                StepTracker, TrackerConfig)
import vla_fake_server as F                                          # noqa: E402

SKIP = "SKIP"
FOLLOW = ROOT / "demo" / "follow_vlm.py"
PS1 = ROOT / "scripts" / "run_citylife_follow.ps1"
JPG = b"\xff\xd8\xff\xe0fake-jpeg\xff\xd9"
T0 = 1_000_000.0                 # a wall-clock origin far from 0, as a flight's
EXPERT = Action4D(vx=4.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)
NO_PORT = "127.0.0.1:59999"      # parsed only, never connected to
# A VLA-like identity: the brief's six fields, a real-looking digest and the
# follow fine-tune's adapter, as demo/vla_server_v2.py announces one.
FT_MODEL = {"id": "aerialvla-ft-follow/r1/epoch2@1a2b3c4d", "base": "openvla-7b@47a0ec7f",
            "weights_sha256": "ab" * 32, "precision": "bf16", "attn": "sdpa", "merged": True,
            "adapter": {"path": "D:/models/aerialvla-ft-follow/r1/epoch2",
                        "sha256": "1a2b3c4d" + "0" * 56, "variant": "fine-tune"}}


def _D():
    import vla_dagger as D
    return D


def _det_at_bearing(deg: float, w_frac: float = 0.12, W: int = 768, H: int = 432):
    """A Grounder box (cx, cy, w, h, score, W, H, colour) whose pinhole bearing
    on the 90 deg FrontCamera is `deg`."""
    cx = W / 2 + (W / 2) * math.tan(math.radians(deg))
    return [cx, 0.6 * H, w_frac * W, 0.5 * w_frac * W, 0.3, W, H, 0.5]


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class _Vehicle:
    """A point multirotor: velocity follows the command with a 0.3 s lag."""

    def __init__(self, x=0.0, y=0.0, up=8.0, yaw=0.0):
        self.x, self.y, self.up, self.yaw = x, y, up, yaw
        self.vx = self.vy = 0.0

    def pose(self):
        return (self.x, self.y, self.up, self.yaw)

    def step(self, a: Action4D, dt: float = 0.1) -> None:
        k = min(1.0, dt / 0.3)
        self.vx += k * (a.vx - self.vx)
        self.vy += k * (a.vy - self.vy)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.up += a.vz_up * dt
        self.yaw = L.wrap_pi(self.yaw + a.yaw_rate * dt)


class _Stub:
    """RemoteVLA's surface, answered by the test: submit() holds the request
    (one outstanding, as the real client), answer() posts the reply - a
    scripted answer from tests/vla_fake_server.py, read by vla_link's own
    parse_infer_response - or a timeout."""

    def __init__(self, clock, identity=None):
        self.clock = clock
        self.identity = dict(identity or FT_MODEL)
        self.counts: Counter = Counter()
        self.late: deque = deque()
        self.backing_off = False
        self.sent: list[dict] = []
        self.t_sent: float | None = None
        self.closed = False
        self._seq = 1
        self._pending = None
        self._result = None

    @property
    def busy(self) -> bool:
        return self._pending is not None or self._result is not None

    def submit(self, fields):
        if self.busy:
            return None
        msg = dict(fields, seq=self._seq)
        L.check_request_fields(msg)
        self._seq += 1
        self._pending, self.t_sent = msg, self.clock()
        self.sent.append(msg)
        return msg["seq"]

    def answer(self, t_done: float, reply) -> None:
        msg, self._pending = self._pending, None
        if reply == "timeout":
            self._result = L.LinkResult(msg["seq"], msg, None, self.t_sent, t_done,
                                        "timeout", "no reply within 10 s", 10.0)
            return
        rep = L.parse_infer_response(reply, msg)
        self._result = L.LinkResult(msg["seq"], msg, rep, self.t_sent, t_done)

    def poll(self):
        r, self._result = self._result, None
        return r

    def close(self) -> None:
        self.closed = True


class _Rig:
    """A deterministic DAgger run: a fake clock, the stub client, a point
    vehicle flying what was executed, the expert's (Grounder-format) box at
    `box_deg` in its frame, 0.2 s old. One tick: the test may answer the
    pending request, the background's round runs, the loop's apply(), and
    the background's round again - in flight the thread polls every 10 ms,
    so it collects an answer and sends the next request within the tick."""

    def __init__(self, seed=7, **kw):
        D = _D()
        self.t = T0
        self.s = 0.0
        self.client = _Stub(lambda: self.t, kw.pop("identity", None))
        self.veh = _Vehicle()
        self.box_deg = 0.0
        self.box_yaw_lag = 0.0      # the heading at the box's frame, relative to now
        self.box = True
        vargs, _argv = D.follow_args("a red car", cruise_alt=8.0)
        self.cfg = D.follow_tracker_config(vargs)
        self.p = D.VLAPerturber(seed, client=self.client, get_capture=self.capture,
                                latest_box=D.latest_box_from(self.grounder),
                                object_text="a red car", tracker_config=self.cfg,
                                clock=lambda: self.t, **D.follow_hint_config(vargs), **kw)
        self.p.start(T0, thread=False)
        self.e0 = self.p.schedule.episode(0)

    def capture(self):
        return Capture(self.t - 0.05, Pose(*self.veh.pose()), JPG, JPG)

    def grounder(self):
        if not self.box:
            return None
        return {"det": _det_at_bearing(self.box_deg), "t_det": self.t - 0.02, "seq": 1,
                "tier": "ok", "cap": {"t_capture": self.t - 0.2,
                                      "pose": {"yaw": self.veh.yaw - self.box_yaw_lag}}}

    def run(self, seconds, answer=None, expert=EXPERT, dt=0.1):
        """`answer(msg, waited_s) -> None | reply dict | "timeout"`."""
        rows = []
        for _ in range(int(round(seconds / dt))):
            self.t = T0 + self.s
            c = self.client
            if c._pending is not None and answer is not None:
                got = answer(c._pending, self.t - c.t_sent)
                if got is not None:
                    c.answer(self.t, got)
            self.p._service_once()
            pose = self.veh.pose()
            out, rec = self.p.apply(expert, self.s, pose=pose)
            self.p._service_once()
            json.dumps(rec)                       # every record is plain JSON
            rows.append({"s": self.s, "t": self.t, "out": out, "rec": rec, "pose": pose})
            self.veh.step(out, dt)
            self.s = round(self.s + dt, 10)
        return rows


def _step(fwd, down, dpsi, after=0.5):
    """Answer every request `after` s after it was sent with this step."""
    return lambda msg, waited: (F.step_reply(msg, fwd, down, dpsi)
                                if waited >= after - 1e-9 else None)


def _first_only(fwd, down, dpsi, after=0.5):
    """Answer request 1 only; every later one stays in flight."""
    return lambda msg, waited: (F.step_reply(msg, fwd, down, dpsi)
                                if msg["seq"] == 1 and waited >= after - 1e-9 else None)


def _fly_ast():
    src = FOLLOW.read_text(encoding="utf-8")
    tree = ast.parse(src)
    fly = next(n for n in ast.walk(tree) if isinstance(n, ast.AsyncFunctionDef) and n.name == "fly")
    parents = {c: n for n in ast.walk(fly) for c in ast.iter_child_nodes(n)}

    def guarded(node, test):
        p = node
        while p in parents:
            p = parents[p]
            if isinstance(p, ast.If) and test(ast.unparse(p.test)):
                return True
        return False
    return src, fly, guarded


# =========================================================================== #
# 1. Off by default; refused without a recording; the flight unchanged when off
# =========================================================================== #
def test_off_by_default_and_refused_without_a_recording():
    import follow_vlm as F_
    D = _D()
    a = F_.build_arg_parser().parse_args([])
    assert a.record_perturb_vla is None and a.perturb_vla_allow_stand_in is False
    assert a.perturb_vla_every == D.VLA_EVERY_S == (6.0, 10.0)
    assert a.perturb_vla_dur == D.VLA_DUR_S == (2.5, 4.0)
    b = F_.build_arg_parser().parse_args(["--record-vla", "d", "--record-perturb-vla", NO_PORT,
                                          "--perturb-vla-every", "8,12", "--perturb-vla-dur",
                                          "3,4", "--perturb-vla-allow-stand-in"])
    assert b.record_perturb_vla == NO_PORT and b.perturb_vla_every == (8.0, 12.0)
    assert b.perturb_vla_dur == (3.0, 4.0) and b.perturb_vla_allow_stand_in
    for argv in (["--record-perturb-vla", NO_PORT],                       # no recording
                 ["--record-vla", "d", "--record-perturb", "--record-perturb-vla", NO_PORT],
                 ["--record-vla", "d", "--perturb-vla-every", "8,12"],      # a window flag alone
                 ["--record-vla", "d", "--perturb-vla-dur", "3,4"],
                 ["--record-vla", "d", "--perturb-vla-allow-stand-in"],
                 ["--record-vla", "d", "--record-perturb-vla", "nohost"],   # not HOST:PORT
                 ["--record-vla", "d", "--record-perturb-vla", "127.0.0.1:x"],
                 # a 4 s window and its 1.5 s lead-in do not fit a 4-5 s period
                 ["--record-vla", "d", "--record-perturb-vla", NO_PORT, "--perturb-vla-every",
                  "4,5"],
                 ["--record-vla", "d", "--record-perturb-vla", NO_PORT, "--perturb-vla-dur",
                  "4,3"]):
        try:
            F_.main(argv)
        except SystemExit as exc:
            assert exc.code == 2, (argv, exc.code)
        else:
            raise AssertionError(f"{argv} was accepted")


def test_the_flight_is_unchanged_when_off_and_the_shield_filters_what_flies():
    src, fly, guarded = _fly_ast()
    # vla_dagger is imported, and used, only under `if args.record_perturb_vla`.
    imports = [n for n in ast.walk(fly) if (isinstance(n, ast.Import)
                                             and any(a.name == "vla_dagger" for a in n.names))
               or (isinstance(n, ast.ImportFrom) and n.module == "vla_dagger")]
    assert imports, "vla_dagger is not wired into fly()"
    for n in imports:
        assert guarded(n, lambda s: "args.record_perturb_vla" in s), f"line {n.lineno}"
    uses = [n for n in ast.walk(fly) if isinstance(n, ast.Attribute)
            and isinstance(n.value, ast.Name) and n.value.id == "vla_dagger"]
    assert uses and all(guarded(u, lambda s: "args.record_perturb_vla" in s) for u in uses)
    # One call site for either perturber: after the controller and its slew
    # limiter (`smooth`), the pose passed, under `perturber is not None`.
    calls = [n for n in ast.walk(fly) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == "apply"
             and isinstance(n.func.value, ast.Name) and n.func.value.id == "perturber"]
    assert len(calls) == 1, [c.lineno for c in calls]
    c = calls[0]
    assert guarded(c, lambda s: "perturber is not None" in s)
    assert isinstance(c.args[0], ast.Name) and c.args[0].id == "smooth"
    assert any(k.arg == "pose" for k in c.keywords), "the VLA's tracker needs the pose"
    # What the perturber returns goes to the Shield, and only there: the
    # limiter is called on the expert's raw command and never sees it, so its
    # state stays the expert's (what vla_record.Perturber does).
    assert src.count("d = shield.filter(state, flown_in)") == 1
    for n in ast.walk(fly):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "limiter":
            names = {x.id for a in n.args for x in ast.walk(a) if isinstance(x, ast.Name)}
            assert "flown_in" not in names and "perturb_rec" not in names, n.lineno
    # The server is asked hello before the simulator is touched.
    i_vla = src.index("vla_dagger.make_vla_perturber(")
    assert i_vla < src.index("client = ProjectAirSimClient(address=args.sim_address)")
    # The DART perturbation's interface takes the pose and ignores it.
    R = __import__("vla_record")
    p = R.Perturber(3)
    assert p.apply(EXPERT, 0.5, pose=(0.0, 0.0, 8.0, 0.0))[0] is EXPERT


# =========================================================================== #
# 2. The windows
# =========================================================================== #
def test_the_windows_are_seeded_and_in_the_asked_ranges():
    D = _D()
    a, b = D.WindowSchedule(7), D.WindowSchedule(7)
    sa = [a.state(k * 0.1) for k in range(4000)]
    assert sa == [b.state(k * 0.1) for k in range(4000)], "the same seed, the same windows"
    assert sa != [D.WindowSchedule(8).state(k * 0.1) for k in range(4000)]
    eps = [e for e in a.episodes if e["on_s"] <= 390.0]
    assert len(eps) >= 390 / 10.0 - 1, len(eps)
    assert 6.0 <= eps[0]["on_s"] <= 10.0, eps[0]
    for e in eps:
        assert 2.5 <= e["off_s"] - e["on_s"] <= 4.0, e
        assert math.isclose(e["on_s"] - e["lead_on_s"], 1.5), e
    for e0, e1 in zip(eps, eps[1:]):
        assert 6.0 <= e1["on_s"] - e0["on_s"] <= 10.0, (e0, e1)
        assert e1["lead_on_s"] > e0["off_s"], "a lead-in never starts inside a window"
    for k, st in enumerate(sa):
        s = k * 0.1
        inside = [e for e in a.episodes if e["lead_on_s"] <= s < e["off_s"]]
        assert st["querying"] == bool(inside), (s, st)
        assert st["window"] == bool(inside and inside[0]["on_s"] <= s), (s, st)
        assert st["episode"] == (inside[0]["k"] if inside else None), (s, st)
    # The flags move the ranges; asking late first changes nothing early.
    c = D.WindowSchedule(7, every_s=(8.0, 12.0), dur_s=(3.0, 5.0), lead_s=1.0)
    c.state(300.0)
    ec = [e for e in c.episodes if e["on_s"] <= 300.0]
    assert all(3.0 <= e["off_s"] - e["on_s"] <= 5.0 for e in ec)
    assert all(8.0 <= e1["on_s"] - e0["on_s"] <= 12.0 for e0, e1 in zip(ec, ec[1:]))
    d = D.WindowSchedule(7)
    d.state(350.0)
    assert [d.state(k * 0.1) for k in range(4000)] == sa
    for kw in ({"every_s": (4.0, 5.0)}, {"dur_s": (4.0, 3.0)}, {"every_s": (0.0, 8.0)},
               {"lead_s": -1.0}, {"dur_s": (3.0, 5.0), "every_s": (6.0, 9.0)}):
        try:
            D.WindowSchedule(7, **kw)
        except ValueError:
            pass
        else:
            raise AssertionError(f"{kw} was accepted")
    # The perturber's own schedule is this one, from its seed.
    rig = _Rig(seed=7)
    assert rig.p.state(eps[0]["on_s"] + 0.01)["window"] and rig.e0["on_s"] == eps[0]["on_s"]


# =========================================================================== #
# 3. What flies in a window
# =========================================================================== #
def test_the_tracker_and_the_hint_are_vla_flys_follow_mission():
    D = _D()
    import vla_fly as VF
    vargs, argv = D.follow_args("a red car", cruise_alt=8.0)
    ref = VF.resolve_args(VF.build_arg_parser().parse_args(
        ["--mission", "follow", "--cadence", "pipelined", "--cruise-mps", "4.5",
         "--speed-cap-mps", "5", "--cruise-alt", "8", "--object", "a red car"]))
    cfg = D.follow_tracker_config(vargs)
    assert cfg == VF.tracker_config(ref, None, TrackerConfig), "not vla_fly's follow config"
    assert cfg.turn_while_moving and cfg.anchor_at_answer and cfg.land_as_arrival
    assert cfg.hold_altitude_m == 8.0, "the tracker holds this flight's cruise altitude"
    assert (cfg.smooth_accel_mps2, cfg.smooth_yaw_accel_rps2, cfg.carry_s) == \
        (VF.DEFAULT_SMOOTH["accel"], VF.DEFAULT_SMOOTH["yaw_accel"], VF.DEFAULT_SMOOTH["carry_s"])
    assert (cfg.cruise_mps, cfg.speed_cap_mps) == (D.FOLLOW_CRUISE_MPS, D.FOLLOW_SPEED_CAP_MPS)
    assert vargs.cadence == "pipelined" and vargs.mission == "follow"
    h = D.follow_hint_config(vargs)
    assert (h["fresh_s"], h["memory_s"], h["search_s"]) == \
        (ref.det_fresh_s, ref.follow_memory_s, ref.follow_search_s)
    assert (h["close"].bottom_frac, h["close"].width_frac) == \
        (ref.close_bottom_frac, ref.close_width_frac)
    rig = _Rig()
    dh = rig.p.detector_hint
    assert (dh.fresh_s, dh.memory_s, dh.search_s, dh.record_kind) == (1.0, 6.0, 20.0, True)
    assert dh.start_gate is None, "the expert's lock already acquired the subject"
    assert rig.p.cfg is rig.cfg and rig.p.tracker.cfg == cfg


def test_inside_a_window_the_tracker_flies_the_step_and_outside_the_expert():
    """One answer, parked in the lead-in and taken at the window's opening;
    every later request stays in flight. Each window tick's executed command
    must be what an independent StepTracker with the same config makes of
    the same step - from the same hand-over and pose, pending, no close
    hold - and every other tick the expert's own command object."""
    rig = _Rig()
    e0 = rig.e0
    rows = rig.run(e0["off_s"] + 1.0, answer=_first_only(4.0, 0.0, 0.3, after=0.5))
    c = rig.client
    before = [r for r in rows if r["s"] < e0["lead_on_s"] - 1e-9]
    assert before and all(r["out"] is EXPERT and not r["rec"]["querying"] for r in before)
    assert c.sent and abs(c.sent[0]["t_capture"] - (T0 + e0["lead_on_s"] - 0.05)) < 0.11, \
        "the first request goes out as the lead-in starts"
    lead = [r for r in rows if r["rec"]["querying"] and not r["rec"]["window"]]
    assert lead and all(r["out"] is EXPERT for r in lead)
    assert any(r["rec"]["parked_seq"] == 1 for r in lead), "the answer waits for the window"
    win = [r for r in rows if r["rec"]["window"]]
    assert len(win) >= 25 and all(r["rec"]["flying"] and r["rec"]["active"] for r in win)
    for r in win:
        assert r["out"] is not EXPERT and r["rec"]["seq"] == 1
        assert r["rec"]["vla_cmd"] == {k: getattr(r["out"], k)
                                       for k in ("vx", "vy", "vz_up", "yaw_rate")}
    # The reference: the same config, step, hand-over and poses.
    rec1 = next(x for x in rig.p.records if x["seq"] == 1)
    assert rec1["accepted"] and rec1["outcome"]["t_start"] == win[0]["t"], rec1["outcome"]
    assert rec1["outcome"]["end"] == "suspended", "the window's end ends the step"
    d = Decision.of(1, rec1["t_capture"], Pose.from_wire(rec1["pose_capture"]),
                    Step.from_wire(rec1["step"]))
    ref = StepTracker(rig.cfg)
    ref.set_alt_band(rig.p.tracker.alt_band)
    i0 = rows.index(win[0])
    ref.note_flown(rows[i0 - 1]["out"], rows[i0 - 1]["t"])
    assert ref.accept(d, win[0]["t"], Pose(*win[0]["pose"]))
    for r in win:
        tk = ref.tick(r["t"], Pose(*r["pose"]), pending=True, hold_horizontal=False)
        assert r["out"] == tk.action, (r["s"], r["out"], tk.action)
        assert r["rec"]["author"] == tk.author and r["rec"]["reason"] == tk.reason
    # The step and its record on every window tick.
    r = win[0]["rec"]
    assert r["step"] == rec1["step"] and r["bins"] == rec1["bins"] and r["kind"] == "step"
    assert r["phrase"] == "straight ahead " and r["hint_kind"] == "fresh"
    assert math.isclose(r["latency_s"], rec1["t_received"] - rec1["t_capture"], abs_tol=1e-3)
    assert 0.5 <= r["latency_s"] <= 0.7, r["latency_s"]
    # The window over: the expert's own command, at once.
    after = [x for x in rows if x["s"] >= e0["off_s"] - 1e-9]
    assert after and all(x["out"] is EXPERT and not x["rec"]["window"] for x in after)


def test_the_hand_over_ramps_from_the_command_in_force():
    """The VLA's first tick moves away from the expert's last command by at
    most the smoothing's limits (2.5 m/s^2, 0.6 rad/s^2 for 0.1 s): the
    tracker's smoothing starts from what the aircraft was told
    (StepTracker.note_flown), not from rest."""
    rig = _Rig()
    expert = Action4D(vx=0.0, vy=4.0, vz_up=0.0, yaw_rate=0.2)
    rows = rig.run(rig.e0["off_s"], answer=_first_only(5.0, 0.0, -0.4, after=0.4), expert=expert)
    win = [r for r in rows if r["rec"]["window"]]
    first, prev = win[0]["out"], rows[rows.index(win[0]) - 1]["out"]
    assert prev is expert and win[0]["rec"]["flying"]
    assert math.hypot(first.vx - prev.vx, first.vy - prev.vy) <= 2.5 * 0.1 + 1e-9, (first, prev)
    assert abs(first.yaw_rate - prev.yaw_rate) <= 0.6 * 0.1 + 1e-9, (first, prev)
    assert first != expert, "the tracker is flying, toward its own step"
    assert rig.p.tracker.counts["rail_flown"] >= 1


def test_the_newest_lead_in_answer_is_taken_at_the_opening_and_a_lost_link_drops_it():
    # Answer 1 in the lead-in, answer 2 on the opening tick itself: the
    # window flies 2, and 1 is on file as superseded before the window.
    rig = _Rig()
    on = rig.e0["on_s"]

    def answer(msg, waited):
        if msg["seq"] == 1 and waited >= 0.3 - 1e-9:
            return F.step_reply(msg, 4.0, 0.0, 0.3)
        if msg["seq"] == 2 and rig.s >= on - 1e-9:
            return F.step_reply(msg, 2.0, 0.0, -0.3)
        return None
    rows = rig.run(on + 0.5, answer=answer)
    win = [r for r in rows if r["rec"]["window"]]
    assert win[0]["rec"]["seq"] == 2 and win[0]["rec"]["flying"], win[0]["rec"]
    r1 = next(x for x in rig.p.records if x["seq"] == 1)
    assert r1["refused"] == "superseded_in_lead_in" and not r1["accepted"], r1
    assert rig.p._cur["answer_at_open"] == "step"
    # A link failure after the parked answer drops it (the tracker would have
    # dropped the step being flown): nothing to take at the opening.
    rig2 = _Rig()

    def answer2(msg, waited):
        if msg["seq"] == 1 and waited >= 0.3 - 1e-9:
            return F.step_reply(msg, 4.0, 0.0, 0.3)
        if msg["seq"] == 2 and waited >= 0.3 - 1e-9:
            return "timeout"
        return None
    rows2 = rig2.run(rig2.e0["on_s"] + 0.3, answer=answer2)
    win2 = [r for r in rows2 if r["rec"]["window"]]
    assert win2 and all(r["out"] is EXPERT and not r["rec"]["flying"] for r in win2)
    r1 = next(x for x in rig2.p.records if x["seq"] == 1)
    assert r1["refused"] == "lost_in_lead_in", r1
    assert rig2.p._cur["answer_at_open"] is None


def test_answers_in_the_window_supersede_the_step_and_are_taken_at_once():
    rig = _Rig()
    rows = rig.run(rig.e0["off_s"] + 0.5, answer=_step(3.0, 0.0, 0.1, after=0.4))
    win = [r for r in rows if r["rec"]["window"]]
    seqs = [r["rec"]["seq"] for r in win if r["rec"]["flying"]]
    assert len(set(seqs)) >= 3 and seqs == sorted(seqs), seqs
    for r in win:
        if r["rec"]["flying"]:
            assert r["out"] == Action4D(**r["rec"]["vla_cmd"])
    accepted = [x for x in rig.p.records if x.get("accepted")]
    in_window = [x for x in accepted if x["outcome"]["t_start"] > win[0]["t"] + 1e-9]
    assert in_window and all(x["outcome"]["t_start"] - x["t_received"] < 0.1 + 1e-9
                             for x in in_window), "taken on the tick it arrives"
    # Pipelined: the next request leaves on the tick an answer comes back.
    c = rig.client
    gaps = [b["t_capture"] - a["t_capture"] for a, b in zip(c.sent, c.sent[1:])]
    assert gaps and max(gaps[:5]) <= 0.4 + 0.1 + 1e-6, gaps


def _no_step_run(answer):
    rig = _Rig()
    rows = rig.run(rig.e0["off_s"] + 0.3, answer=answer)
    return rig, rows, [r for r in rows if r["rec"]["window"]]


def test_with_no_fresh_step_the_expert_flies_the_window_and_it_is_recorded():
    cases = {
        "silent": (lambda msg, waited: None, None),
        "parse_fail": (lambda msg, waited: F.parse_fail_reply(msg) if waited >= 0.3 else None,
                       "parse_fail"),
        "server_error": (lambda msg, waited: F.error_reply(msg, "inference error: CUDA")
                         if waited >= 0.3 else None, "server_error"),
        "timeout": (lambda msg, waited: "timeout" if waited >= 0.3 else None, "link_failure"),
    }
    for name, (answer, kind) in cases.items():
        rig, rows, win = _no_step_run(answer)
        assert len(win) >= 25, name
        for r in win:
            assert r["out"] is EXPERT, (name, r["rec"])
            rec = r["rec"]
            assert rec["window"] and not rec["flying"] and not rec["active"], (name, rec)
            assert rec["author"] in ("none", "hover", "hold") and rec["reason"], (name, rec)
        s = rig.p.summary()
        assert s["windows"] == 1 and s["vla_ticks"] == 0 and s["active_ticks"] == 0, (name, s)
        assert s["windows_without_vla"] == [0], (name, s)
        ep = s["episodes"][0]
        assert ep["ticks"] == len(win) and ep["vla_ticks"] == 0 and ep["first_vla_s"] is None
        assert sum(ep["why_not"].values()) == len(win), (name, ep)
        if kind is None:
            assert not s["answers"] and s["requests"] == 1, (name, s)
        else:
            assert s["answers"].get(kind, 0) >= 1, (name, s["answers"])


def test_the_phrase_is_detectorhints_for_the_experts_box_corrected_for_the_turn():
    """The box at +30 deg in a frame taken facing 0 rad; the VLA capture faces
    0.3 rad (17.2 deg) further right, so the car is 12.8 deg right of the nose
    now: "straight ahead ", not the frame's "forward-right " (DetectorHint's
    turn correction, as vla_fly applies it)."""
    rig = _Rig()
    rig.veh.yaw = 0.3
    rig.box_deg, rig.box_yaw_lag = 30.0, 0.3
    rig.run(rig.e0["lead_on_s"] + 0.2)
    m = rig.client.sent[0]
    assert m["phrase"] == "straight ahead " and m["hint_source"] == "detector", m["phrase"]
    assert math.isclose(m["pose"]["yaw_rad"], 0.3) and m["down_jpg_b64"]
    assert m["object"] == "a red car"
    rig2 = _Rig()
    rig2.box_deg = 30.0
    rig2.run(rig2.e0["lead_on_s"] + 0.2)
    assert rig2.client.sent[0]["phrase"] == "forward-right "
    # No box: no request (never ""), counted; the memory starts empty each
    # window (DetectorHint.forget at the lead-in: the VLA's own sightings).
    rig3 = _Rig()
    rig3.box = False
    rig3.run(rig3.e0["off_s"])
    assert not rig3.client.sent and rig3.p.summary()["no_hint"] >= 10
    assert rig3.p.detector_hint.kinds["forgotten"] == 1


# =========================================================================== #
# 4. Safety: nothing raises into the loop
# =========================================================================== #
def test_a_fault_never_raises_into_the_loop_and_the_expert_flies():
    rig = _Rig()
    rig.run(rig.e0["on_s"] + 0.5, answer=_step(3.0, 0.0, 0.0, after=0.3))
    out, rec = rig.p.apply(EXPERT, rig.e0["on_s"] + 0.6, pose=None)     # no pose
    assert out is EXPERT and not rec["active"] and rec["source"] == "vla" and "error" in rec
    out, rec = rig.p.apply(EXPERT, rig.e0["on_s"] + 0.7, pose=(float("nan"), 0.0, 8.0, 0.0))
    assert out is EXPERT and "error" in rec
    # A capture source and a box source that raise: counted on the background.
    rig2 = _Rig()

    def boom():
        raise RuntimeError("camera gone")
    rig2.p.get_capture = boom
    rows = rig2.run(rig2.e0["off_s"])
    assert all(r["out"] is EXPERT for r in rows)
    s = rig2.p.summary()
    assert s["errors"]["n"] >= 1 and "camera gone" in s["errors"]["first"][0], s["errors"]


# =========================================================================== #
# 5. Through the real protocol-v2 client and threads, against a loopback server
# =========================================================================== #
def _as_ft(reply: dict) -> dict:
    """A fake-server answer from the model hello announced (FakeVLAServer's
    answers carry its module's MODEL; the client pins hello's identity and
    refuses any other - a protocol failure, model_changed)."""
    return dict(reply, model=dict(FT_MODEL))


def _fly_threaded(td: Path, port: int, seconds: float, *, kill_after_s=None, server=None):
    """make_vla_perturber against 127.0.0.1:port, with the real SemanticObs pose
    ring and VLAFrames (a fake encoder), the expert's recorder around it, 10 Hz
    of wall-clock ticks. The windows are short so a few fit in seconds."""
    D = _D()
    R = __import__("vla_record")
    from semantic_demo import SemanticObs
    obs = SemanticObs()
    box = {"g": None}
    p = D.make_vla_perturber(f"127.0.0.1:{port}", seed=11, obs=obs,
                             box_source=lambda: box["g"], policy=None,
                             object_text="a red car", cruise_alt=8.0,
                             record_dir=td / "vla_record", every_s=(1.6, 1.8),
                             dur_s=(0.8, 1.0), lead_s=0.5, allow_stand_in=False,
                             encode=lambda m: bytes(m["data"]))
    rec = R.ExpertVLARecorder(td / "vla_record", obs, lambda: R.box_from_grounder(box["g"]),
                              tag="t_dagger", object_text="a red car",
                              encode=lambda m: bytes(m["data"]), perturber=p)
    veh = _Vehicle()
    t0 = time.time()
    rec.start(t0)
    stamp0 = 7_000_000_000
    outs = []
    k = 0
    while time.time() - t0 < seconds:
        if kill_after_s is not None and server is not None and time.time() - t0 > kill_after_s:
            server.stop()
            kill_after_s = None
        t = time.time()
        st = stamp0 + k * 100_000_000
        obs.put_pose_full(t, veh.x, veh.y, veh.up, veh.yaw, 0.0, 0.0, stamp=st)
        box["g"] = {"det": _det_at_bearing(0.0), "t_det": t - 0.02, "seq": k, "tier": "ok",
                    "cap": {"t_capture": t - 0.15, "pose": {"yaw": veh.yaw}}}
        out, pr = p.apply(EXPERT, t - t0, pose=veh.pose())
        rec.note_tick(t, veh.x, veh.y, veh.up, veh.yaw, mode="track", cmd=out, asked=EXPERT,
                      perturb=pr)
        msg = {"data": f"f-{st}".encode(), "width": 256, "height": 256, "time_stamp": st}
        rec.put_down(dict(msg, data=f"d-{st}".encode()))
        rec.put_front(msg)
        outs.append((t - t0, out, pr))
        veh.step(out)
        k += 1
        time.sleep(max(0.0, t0 + k * 0.1 - time.time()))
    t_stop = time.time()
    st = rec.stop()
    return p, st, outs, time.time() - t_stop


def test_through_the_real_client_the_records_and_the_summary():
    R = __import__("vla_record")
    srv = F.FakeVLAServer(lambda req: F.Delay(0.15, _as_ft(F.step_reply(req, 3.0, 0.0, 0.1))),
                          model=FT_MODEL)
    with srv, tempfile.TemporaryDirectory() as td:
        td = Path(td)
        assert srv.port != 8790
        p, st, outs, stop_s = _fly_threaded(td, srv.port, 5.2)
        assert stop_s < 12.0, stop_s
        s = st["perturb"]
        assert s["source"] == "vla" and s["windows"] >= 2, s
        assert s["vla_ticks"] > 0 and s["active_ticks"] == s["vla_ticks"]
        assert s["answers"].get("step", 0) >= 3 and not s["answers"].get("link_failure")
        # (a request submitted as the flight stopped may never have left)
        assert len(srv.requests) <= s["requests"] <= len(srv.requests) + 1
        assert srv.overlaps == 0, "one request outstanding at a time"
        lat = s["latency_s"]
        assert lat["n"] >= 3 and 0.1 <= lat["p50"] <= 1.5 and lat["p95"] >= lat["p50"], lat
        assert set(lat) >= {"n", "p50", "p95", "max"} and s["rtt_s"]["n"] >= 3
        sv = s["server"]
        assert sv["address"] == f"127.0.0.1:{srv.port}" and sv["is_vla"] is True
        assert sv["identity"]["id"] == FT_MODEL["id"] and sv["adapter_variant"] == "fine-tune"
        assert sv["adapter"]["sha256"] == FT_MODEL["adapter"]["sha256"]
        assert s["config"]["tracker"]["anchor_at_answer"] is True
        assert s["config"]["vla_fly_argv"][:4] == ["--mission", "follow", "--cadence", "pipelined"]
        assert {r["phrase"] for r in srv.requests} == {"straight ahead "}
        assert all(r["hint_source"] == "detector" for r in srv.requests)
        # Every tick's record; the VLA's ticks fly its tracker's command.
        flown = [(o, pr) for _s, o, pr in outs if pr["active"]]
        assert flown and all(o == Action4D(**pr["vla_cmd"]) and pr["window"] for o, pr in flown)
        assert all(o is EXPERT for _s, o, pr in outs if not pr["active"])
        d = td / "vla_record"
        track = [json.loads(ln) for ln in (d / R.TRACK).read_text(encoding="utf-8").splitlines()]
        assert len(track) == len(outs) and all(x["perturb"]["source"] == "vla" for x in track)
        assert sum(x["perturb"]["active"] for x in track) == s["vla_ticks"]
        fly1 = next(x["perturb"] for x in track if x["perturb"]["active"])
        assert fly1["step"] and len(fly1["bins"]) == 3 and fly1["seq"] >= 1
        assert fly1["phrase"] == "straight ahead " and fly1["latency_s"] > 0
        caps = [json.loads(ln) for ln in (d / R.CAPTURES).read_text(encoding="utf-8").splitlines()]
        assert caps and all(c["perturb"]["source"] == "vla" for c in caps)
        r = json.loads((d / R.RECORD).read_text(encoding="utf-8"))
        assert r["perturb"]["source"] == "vla" and r["perturb"]["server"]["identity"]["id"] == \
            FT_MODEL["id"]
        m = R.record_summary(st)["perturb"]
        assert m["source"] == "vla" and m["episodes"] == s["windows"] and "server" in m
        assert m["latency_s"]["p50"] == lat["p50"] and m["active_s"] > 0
        # vla_steps.jsonl: one line per request, with what became of it.
        steps = [json.loads(ln) for ln in (d / "vla_steps.jsonl").read_text(
            encoding="utf-8").splitlines()]
        assert len(steps) == s["requests"] and {x["seq"] for x in steps} == \
            set(range(1, s["requests"] + 1))
        assert all("accepted" in x and "refused" in x for x in steps)
        assert sum(1 for x in steps if x["accepted"]) == s["accepted"]
        frames = d / "vla_frames"
        assert all((frames / f"{x['front_jpg_sha256']}.jpg").is_file() for x in steps)


def test_a_dead_server_never_crashes_the_flight():
    srv = F.FakeVLAServer(lambda req: _as_ft(F.step_reply(req, 3.0, 0.0, 0.0)), model=FT_MODEL)
    srv.start()
    with tempfile.TemporaryDirectory() as td:
        try:
            p, st, outs, stop_s = _fly_threaded(Path(td), srv.port, 6.0, kill_after_s=2.6,
                                                server=srv)
        finally:
            srv.stop()
        s = st["perturb"]
        assert stop_s < 12.0, stop_s
        assert s["answers"].get("link_failure", 0) >= 1, s["answers"]
        assert any(pr["active"] for t, o, pr in outs if t < 2.6), "the VLA flew before"
        late = [(t, o, pr) for t, o, pr in outs if t > 3.6 and pr["window"]]
        assert late and all(o is EXPERT and not pr["active"] for t, o, pr in late), \
            "after the server died the expert flies every window tick"
        assert s["errors"]["n"] == 0, s["errors"]


def test_the_server_must_answer_hello_and_be_aerialvla_unless_a_stand_in_is_allowed():
    D = _D()
    from semantic_demo import SemanticObs
    kw = dict(seed=1, obs=SemanticObs(), box_source=lambda: None, policy=None,
              object_text="a red car", cruise_alt=8.0, record_dir=None)
    port = _free_port()
    try:
        D.make_vla_perturber(f"127.0.0.1:{port}", **kw)
    except D.DaggerRefused as exc:
        assert str(port) in str(exc)
    else:
        raise AssertionError("no server, yet accepted")
    with F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0)) as srv:
        try:
            D.make_vla_perturber(f"127.0.0.1:{srv.port}", **kw)
        except D.DaggerRefused as exc:
            assert "stand-in" in str(exc) or "not AerialVLA" in str(exc), exc
        else:
            raise AssertionError("a stand-in identity was accepted without being allowed")
        p = D.make_vla_perturber(f"127.0.0.1:{srv.port}", allow_stand_in=True, **kw)
        try:
            sv = p.summary()["server"]
            assert sv["is_vla"] is False and sv["why_not_vla"], sv
        finally:
            p.stop()


# =========================================================================== #
# 6. The recorder keeps a VLA record whole
# =========================================================================== #
def test_the_recorder_keeps_the_vla_record_on_the_track_line():
    R = __import__("vla_record")
    from semantic_demo import SemanticObs
    with tempfile.TemporaryDirectory() as td:
        rec = R.ExpertVLARecorder(Path(td) / "vla_record", SemanticObs(), lambda: None,
                                  tag="t", object_text="a red car",
                                  encode=lambda m: bytes(m["data"]))
        rec.start(time.time())
        pr = {"source": "vla", "active": True, "window": True, "querying": True, "episode": 2,
              "flying": True, "author": "vla_step", "reason": "translate", "seq": 5,
              "step": {"fwd_m": 3.0, "down_m": 0.0, "dpsi_rad": 0.1}, "bins": [58, 49, 53],
              "latency_s": 0.81, "phrase": "straight ahead "}
        rec.note_tick(time.time(), 0.0, 0.0, 8.0, 0.0, mode="track", perturb=pr)
        rec.note_tick(time.time(), 0.0, 0.0, 8.0, 0.0, mode="track",
                      perturb={"active": True, "yaw_rate": 0.4, "speed_scale": 1.0,
                               "episode": 0})
        rec.stop()
        track = [json.loads(ln) for ln in (Path(td) / "vla_record" / R.TRACK).read_text(
            encoding="utf-8").splitlines()]
        assert track[0]["perturb"] == pr
        assert track[1]["perturb"] == {"active": True, "yaw_rate": 0.4, "speed_scale": 1.0,
                                       "episode": 0}, "a DART line is as before"


# =========================================================================== #
# 7. The launcher
# =========================================================================== #
def test_the_launcher_flies_a_dagger_recording_only_with_a_recording():
    src = PS1.read_text(encoding="utf-8")
    for p in ("[string]$RecordPerturbVla", "[string]$PerturbVlaEvery", "[string]$PerturbVlaDur",
              "[switch]$PerturbVlaAllowStandIn"):
        assert p in src, p
    lines = src.splitlines()
    for flag, guard in (('"--record-perturb-vla"', "if ($RecordPerturbVla)"),
                        ('"--perturb-vla-every"', "if ($PerturbVlaEvery)"),
                        ('"--perturb-vla-dur"', "if ($PerturbVlaDur)"),
                        ('"--perturb-vla-allow-stand-in"', "if ($PerturbVlaAllowStandIn)")):
        assert src.count(flag) == 1, flag
        fi = next(n for n, ln in enumerate(lines) if flag in ln)
        gi = max(n for n, ln in enumerate(lines[:fi]) if ln.lstrip().startswith(guard))
        vi = max(n for n, ln in enumerate(lines[:fi]) if ln.lstrip().startswith("if ($RecordVla)"))
        assert vi < gi, f"{flag} is not inside the -RecordVla block"
        for start in (gi, vi):
            ind = len(lines[start]) - len(lines[start].lstrip())
            for ln in lines[start + 1:fi + 1]:
                assert not ln.strip() or len(ln) - len(ln.lstrip()) > ind, (flag, ln)
    # Refused, and the server checked, before a simulator is started for nothing.
    sim = src.index("if (-not $SkipSim) { Start-Sim }")
    assert src.index("if ($RecordPerturbVla -and -not $RecordVla)") < sim
    assert src.index("if ($RecordPerturbVla -and $RecordPerturb)") < sim
    assert src.index("($PerturbVlaEvery -or $PerturbVlaDur -or $PerturbVlaAllowStandIn) "
                     "-and -not $RecordPerturbVla") < sim
    assert src.index("Test-NetConnection $vlaHost -Port $vlaPort") < sim
    assert "-RecordPerturbVla 127.0.0.1:8790" in src, "the header shows a DAgger flight"
    import follow_vlm as F_
    a = F_.build_arg_parser().parse_args(["--record-vla", "d", "--record-perturb-vla", NO_PORT,
                                          "--perturb-vla-every", "6,10", "--perturb-vla-dur",
                                          "2.5,4.0", "--perturb-vla-allow-stand-in"])
    assert a.record_perturb_vla == NO_PORT and a.perturb_vla_dur == (2.5, 4.0)


# =========================================================================== #
# 8. The builder: VLA-flown ticks are perturbations, the labels unchanged
# =========================================================================== #
def test_the_builder_drops_labels_over_vla_flown_ticks_and_keeps_the_recovery():
    import build_follow_dataset as B
    import test_follow_dataset as TFD

    def vla(k):
        """A window over ticks 30..69, its lead-in from 15; the VLA's first step
        came in at tick 34, so it flew ticks 34..69 (commands at 3.4 .. 6.9 s)."""
        if 30 <= k < 70:
            fly = k >= 34
            return {"source": "vla", "active": fly, "window": True, "querying": True,
                    "episode": 0, "flying": fly, "author": "vla_step" if fly else "hover",
                    "reason": "translate" if fly else "awaiting_fresh_decision",
                    "seq": 1 if fly else None, "step": ({"fwd_m": 2.97, "down_m": 0.0,
                                                         "dpsi_rad": 0.0} if fly else None),
                    "bins": [58, 49, 49] if fly else None, "latency_s": 0.8,
                    "phrase": "straight ahead "}
        if 15 <= k < 30:
            return {"source": "vla", "active": False, "window": False, "querying": True,
                    "episode": 0, "flying": False, "parked_seq": None}
        return {"source": "vla", "active": False, "window": False, "querying": False,
                "episode": None}

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TFD._write_run(td, "runV", TFD._straight(), seconds=10.0, perturb=vla)
        st = B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False)
        # Dropped when t < 7.0 + 0.3 and t + 1.0 > 3.4: t = 2.5 .. 7.2 (48); the
        # window's first four ticks, flown by the expert, are no perturbation
        # (t = 2.4 ends on the VLA's first command and is kept).
        assert st["dropped"]["perturbed"] == 48, st["dropped"]
        rows = [json.loads(ln) for ln in (td / "ds" / "samples.jsonl").read_text(
            encoding="utf-8").splitlines()]
        ts = sorted(round(r["t_capture"] - TFD.T0, 3) for r in rows)
        assert 2.4 in ts and 7.3 in ts and not any(2.4 < t < 7.3 for t in ts), ts
        for r in rows:
            since = r["perturb"]["since_end_s"]
            t = r["t_capture"] - TFD.T0
            assert (since is None) == (t < 7.0), r["perturb"]
            if since is not None:
                assert math.isclose(since, t - 7.0, abs_tol=1e-6)
        assert st["runs"]["runV"]["perturb"] == {"intervals": 1, "active_s": 3.6,
                                                 "dropped": 48, "post_perturb_3s": 18}, \
            st["runs"]["runV"]["perturb"]
        # The labels themselves: the same as the flight's without the records.
        clean = TFD._write_run(td, "runC", TFD._straight(), seconds=10.0)
        B.build_dataset([clean], td / "dsc", horizon_s=1.0, stride_s=0.0, images=False)
        crow = {r["i"]: r for r in (json.loads(ln) for ln in (td / "dsc" / "samples.jsonl")
                                    .read_text(encoding="utf-8").splitlines())}
        for r in rows:
            c = crow[r["i"]]
            for k in ("label", "bins", "action_text", "phrase", "prompt", "label_raw"):
                assert r[k] == c[k], (r["i"], k)


# --------------------------------------------------------------------------- #
# runner
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
