"""The frozen camera and the detector's orderly stop, checked offline.

Run either way:
    pytest tests/test_frozen_camera.py -v
    python tests/test_frozen_camera.py

1. THE FROZEN CAMERA (2026-11-20). After a scene reload the simulator's
   FrontCamera went on delivering messages whose pixels never changed: in
   pab_aerialvla_04_*_nomission* OWL-ViT ran 1200-1436 inferences on one
   image and the start gate hovered its whole 330 s, ending "timed_out" with
   nothing naming the camera. follow_vlm signs every inference's frame
   (frame_signature), counts the run on one frame (FrameRun) and withholds a
   frozen frame's boxes (FROZEN_FRAME_N / _S); demo/pas_ardupilot/
   perception_node.py ends the start gate's hover as "camera_frozen" (no
   mission, whatever --start-timeout-action says) and a mission as a lost
   link ends it, at CAMERA_FROZEN_N inferences over CAMERA_FROZEN_S seconds.
   A live camera - a new frame on every inference, or a detector outrunning
   the camera - never trips it.
2. THE DETECTOR'S STOP (2026-11-20). Every no-mission run on the AGX Orin
   exited 134 ("terminate called without an active exception", core dumped)
   after its [report] lines: run() only flagged the Grounder's thread to stop
   and returned while it was still inside the GPU. stop_detector stops and
   JOINS it before the camera goes and before run() returns; MavrosLink.
   close() ends the ROS 2 node and rclpy the link created.
"""
from __future__ import annotations

import contextlib
import io
import json
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PAS = ROOT / "demo" / "pas_ardupilot"
for _p in (ROOT, ROOT / "tests", ROOT / "demo", PAS):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import follow_vlm as fv                                              # noqa: E402
import mavros_link as ml                                             # noqa: E402
import perception_node as pn                                         # noqa: E402
import test_pas_ardupilot as T                                       # noqa: E402

SKIP = "SKIP"


# =========================================================================== #
# 1a. The signature and the run on one frame (follow_vlm)
# =========================================================================== #

def test_the_frame_signature_is_exact_over_every_pixel_and_never_raises():
    try:
        from PIL import Image
    except ImportError:
        return SKIP
    a = Image.new("RGB", (768, 432), (90, 90, 90))
    b = Image.new("RGB", (768, 432), (90, 90, 90))
    assert fv.frame_signature(a) == fv.frame_signature(b) is not None
    # One pixel one level brighter is another frame: a hovering aircraft's
    # sub-pixel drift must never read as a frozen camera.
    b.putpixel((700, 400), (91, 90, 90))
    assert fv.frame_signature(a) != fv.frame_signature(b)
    # The same pixels at another size are another frame.
    assert fv.frame_signature(Image.new("RGB", (432, 768), (90, 90, 90))) != \
        fv.frame_signature(a)
    assert fv.frame_signature(None) is None and fv.frame_signature(object()) is None


def test_a_frame_run_counts_inferences_and_seconds_on_one_frame():
    r = fv.FrameRun()

    def obs(sig, t):
        n, s = r.observe(sig, t)
        return n, round(s, 6)
    assert obs("a", 100.0) == (1, 0.0)
    assert obs("a", 100.4) == (2, 0.4)
    assert obs("a", 101.0) == (3, 1.0)
    # A new frame starts a new run; the longest is kept.
    assert obs("b", 101.3) == (1, 0.0)
    assert (r.longest_n, r.longest_s) == (3, 1.0)
    # A frame that could not be signed is never one long run.
    assert [r.observe(None, 102.0 + i)[0] for i in range(5)] == [1] * 5
    assert r.longest_n == 3


def test_a_frozen_frame_is_withheld_only_past_both_limits():
    n, s = fv.FROZEN_FRAME_N, fv.FROZEN_FRAME_S
    assert fv.frame_withheld(n, s) is True
    assert fv.frame_withheld(n - 1, 1000.0) is False, "seconds alone are no verdict"
    assert fv.frame_withheld(1000, s - 0.01) is False, "inferences alone are no verdict"
    assert fv.frame_withheld(None, None) is False
    # The run's verdict is slower than the withholding a new frame undoes.
    assert pn.CAMERA_FROZEN_N >= fv.FROZEN_FRAME_N
    assert pn.CAMERA_FROZEN_S > fv.FROZEN_FRAME_S


def test_the_grounder_signs_logs_and_withholds_a_frozen_frame():
    """The real Grounder._worker against a stand-in OWL-ViT (CPU tensors, no
    model; as tests/test_range_and_lock.py drives it): four inferences on one
    frame, then a new frame. With the limits at 3 inferences and 0 s, the 3rd
    and 4th are withheld - no box, no candidate - and the row says why; the
    new frame publishes its box again. latest() and frame_stats() carry the
    run."""
    try:
        import torch
        from PIL import Image
    except ImportError:
        return SKIP
    import tempfile
    import types
    w_, h_ = 400, 225
    frozen = Image.new("RGB", (w_, h_), (128, 128, 128))
    frozen.paste((220, 20, 20), (40, 100, 120, 140))
    fresh = frozen.copy()
    fresh.putpixel((399, 0), (127, 128, 128))
    frames = [frozen, frozen, frozen, frozen, fresh]
    scores = torch.tensor([0.30])
    boxes = torch.tensor([[40, 100, 120, 140]], dtype=torch.float32)

    class Inputs(dict):
        def to(self, _dev):
            return self

    class Proc:
        @classmethod
        def from_pretrained(cls, _name):
            return cls()

        def __call__(self, text, images, return_tensors):
            return Inputs()

        def post_process_object_detection(self, out, threshold, target_sizes):
            return [{"scores": scores.clone(), "boxes": boxes.clone(),
                     "labels": torch.zeros(1, dtype=torch.long)}]

    class Model:
        @classmethod
        def from_pretrained(cls, _name):
            return cls()

        def to(self, _dev):
            return self

        def eval(self):
            return self

        def __call__(self, **kw):
            return types.SimpleNamespace(logits=None)

    class Obs:
        g = None
        seen: list = []

        def get_front_native(self):
            img = frames[len(self.seen)]
            self.seen.append(img)
            if len(self.seen) == len(frames):
                self.g._stop = True           # the last frame, then the loop ends
            return img

        def get_depth(self):
            return None

    tf = types.ModuleType("transformers")
    tf.OwlViTProcessor, tf.OwlViTForObjectDetection = Proc, Model
    log = Path(tempfile.mkdtemp()) / "detections.jsonl"
    obs = Obs()
    g = fv.Grounder(obs, "a red car", thresh=0.02, log_path=log, frozen_n=3, frozen_s=0.0)
    obs.g = g
    lat0 = g.latest()
    assert (lat0["same_frame_n"], lat0["frame_withheld"], lat0["frame_sig"]) == (0, False, None)
    saved_tf = sys.modules.get("transformers")
    to0, sync0, mem0 = torch.Tensor.to, torch.cuda.synchronize, torch.cuda.memory_allocated

    def to(self, *a, **k):                    # "cuda" stays on the CPU here
        return self if a and a[0] == "cuda" else to0(self, *a, **k)
    try:
        sys.modules["transformers"] = tf
        torch.Tensor.to = to
        torch.cuda.synchronize = lambda *a, **k: None
        torch.cuda.memory_allocated = lambda *a, **k: 0
        g._worker()
    finally:
        torch.Tensor.to, torch.cuda.synchronize, torch.cuda.memory_allocated = to0, sync0, mem0
        if saved_tf is None:
            sys.modules.pop("transformers", None)
        else:
            sys.modules["transformers"] = saved_tf
    rows = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
    assert [r["same_frame_n"] for r in rows] == [1, 2, 3, 4, 1], rows
    assert [r["withheld"] for r in rows] == [False, False, True, True, False], rows
    assert len({r["frame_sig"] for r in rows[:4]}) == 1 and rows[4]["frame_sig"] != rows[0]["frame_sig"]
    assert all(r["same_frame_s"] >= 0.0 for r in rows)
    # A withheld row publishes nothing; the frames around it do.
    assert [r["det"] is not None for r in rows] == [True, True, False, False, True], rows
    assert [r["n_cands"] for r in rows] == [1, 1, 0, 0, 1], rows
    lat = g.latest()
    assert lat["same_frame_n"] == 1 and lat["frame_withheld"] is False
    assert lat["frame_sig"] == rows[4]["frame_sig"] and lat["det"] is not None
    st = g.frame_stats()
    assert st["longest_same_frame_n"] == 4 and st["withheld_inferences"] == 2, st
    assert st["withheld_spans"] == 1, st
    assert st["withhold_rule"] == {"same_frame_n": 3, "same_frame_s": 0.0}, st


# =========================================================================== #
# 1b. The verdict (perception_node)
# =========================================================================== #

def test_the_frozen_camera_verdict_needs_both_limits():
    n, s = pn.CAMERA_FROZEN_N, pn.CAMERA_FROZEN_S
    fz = pn.camera_frozen({"same_frame_n": n, "same_frame_s": s, "frame_sig": "ab12"})
    assert fz == {"same_frame_n": n, "same_frame_s": s, "frame_sig": "ab12"}, fz
    for lat in ({"same_frame_n": n - 1, "same_frame_s": 330.0},       # a slow repeat
                {"same_frame_n": 1436, "same_frame_s": s - 0.1},       # not long enough
                {"same_frame_n": 1, "same_frame_s": 0.0},              # a live camera
                {"seq": 5, "det": None},                               # a stand-in
                None, "not a dict", {"same_frame_n": "x", "same_frame_s": 1e9}):
        assert pn.camera_frozen(lat) is None, lat
    text = pn.camera_frozen_text(fz, "start gate")
    assert text.startswith("the camera is frozen (start gate)") and str(n) in text, text
    assert text.endswith(pn.CAMERA_FROZEN_FIX), text


class FrameDetections(T.GateDetections):
    """GateDetections publishing a run on one frame, as follow_vlm.Grounder
    does: `run(k, t)` gives (same_frame_n, same_frame_s) for the k-th answer
    at stand-in time t (s since the first answer)."""

    def __init__(self, clock, run=None, **kw):
        super().__init__(clock, **kw)
        self.run = run or (lambda k, t: (1, 0.0))
        self.t_first = None

    def latest(self):
        out = super().latest()
        if self.t_first is None:
            self.t_first = self.clock.t
        n, s = self.run(self.n, self.clock.t - self.t_first)
        out.update(same_frame_n=n, same_frame_s=s, frame_sig=f"sig{self.n - n + 1}",
                   frame_withheld=False)
        return out


def frozen_from(t_freeze: float):
    """A camera live until `t_freeze` s, then one frame for ever."""
    state = {}

    def run(k, t):
        if t < t_freeze:
            return 1, 0.0
        state.setdefault("k0", k)
        state.setdefault("t0", t)
        return k - state["k0"] + 1, t - state["t0"]
    return run


def _events(run):
    return [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]


def test_a_frozen_camera_ends_the_start_gate_with_no_mission():
    """The 20 Nov runs, on the stand-ins: the camera freezes 2 s into the
    gate's hover and the gate ends "camera_frozen" CAMERA_FROZEN_S later -
    not after its whole timeout - with no mission (LAND, nothing commanded),
    the verdict on the start_gate event and in metrics.json, the fix named,
    and never KPI-grade. --start-timeout-action fly does not fly it."""
    for action in ("land", "fly"):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc, run, link = T._fly(grounder_cls=FrameDetections,
                                   grounder_kw={"run": frozen_from(2.0)},
                                   extra=("--start-when-seen", "5", "--start-timeout-s", "120",
                                          "--start-timeout-action", action))
        assert rc == 0
        m = T._json(run, "metrics.json")
        sg = m["start_gate"]
        assert sg["outcome"] == "camera_frozen" and sg["reached"] is False, sg
        assert sg["wait_s"] < 30.0, f"waited {sg['wait_s']} s: the verdict did not end it"
        fz = sg["camera_frozen"]
        assert fz["same_frame_n"] >= pn.CAMERA_FROZEN_N, fz
        assert pn.CAMERA_FROZEN_S <= fz["same_frame_s"] < pn.CAMERA_FROZEN_S + 1.0, fz
        assert m["end_reason"].startswith("no mission: the camera is frozen (start gate)"), \
            m["end_reason"]
        assert pn.CAMERA_FROZEN_FIX in m["end_reason"]
        assert m["ticks"] == 0 and link.requested == ["LAND"] and not link.sent, action
        cf = m["camera_frozen"]
        assert cf["ended"] == "start gate" and cf["verdicts"] == 1 and cf["mission"] is None, cf
        assert cf["start_gate"] == fz and cf["detector_frames"] is None, cf
        ev = [e for e in _events(run) if e["kind"] == "start_gate"]
        assert len(ev) == 1 and ev[0]["outcome"] == "camera_frozen", ev
        assert ev[0]["camera_frozen"] == fz and "camera is frozen" in ev[0]["aborted"], ev
        reasons = T._json(run, "kpi.json")["kpi_grade_reasons"]
        assert any(w.startswith("no mission was flown: the camera is frozen") for w in reasons)
        out = buf.getvalue()
        assert "*** NO SUBJECT ***" in out and "(camera_frozen)" in out, out
        assert "[report] camera: the detector keeps no frame record | frozen: *** YES, ended " \
               "the start gate" in out, out


def test_a_live_camera_never_trips_the_gate():
    """A new frame on every inference, and a detector outrunning its camera
    (each frame seen up to CAMERA_FROZEN_N - 1 times, for as long as you
    like; or CAMERA_FROZEN_N times inside a second): the gate waits out its
    timeout as before, and the camera is reported live."""
    n_lim = pn.CAMERA_FROZEN_N
    for run_fn in ((lambda k, t: (1, 0.0)),
                   (lambda k, t: ((k - 1) % (n_lim - 1) + 1, 1000.0)),
                   (lambda k, t: ((k - 1) % n_lim + 1, 0.9 * (((k - 1) % n_lim) / n_lim)))):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc, run, link = T._fly(grounder_cls=FrameDetections, grounder_kw={"run": run_fn},
                                   extra=("--start-when-seen", "5", "--start-timeout-s", "20"))
        assert rc == 0
        m = T._json(run, "metrics.json")
        assert m["start_gate"]["outcome"] == "timed_out", m["start_gate"]["outcome"]
        assert m["start_gate"]["camera_frozen"] is None
        assert m["end_reason"].startswith("no mission: start gate timed out"), m["end_reason"]
        assert m["camera_frozen"]["ended"] is None and m["camera_frozen"]["verdicts"] == 0
        assert "| frozen: no" in buf.getvalue()


def test_a_camera_that_freezes_in_the_mission_ends_it_and_lands():
    """The mission's frozen camera: the follow flies on a live camera, then
    one frame for ever. The verdict ends the episode as a lost link does -
    the mission-end LAND, the camera_frozen event, metrics.json, the
    [report] line and the KPI-grade reason - instead of searching on a dead
    picture until max_s. On a live camera the mission runs to max_s."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc, run, link = T._fly(grounder_cls=FrameDetections, max_s="60",
                               grounder_kw={"run": frozen_from(5.0)})
    assert rc == 0
    m = T._json(run, "metrics.json")
    assert m["end_reason"].startswith("the camera is frozen (mission)"), m["end_reason"]
    cf = m["camera_frozen"]
    assert cf["ended"] == "mission" and cf["verdicts"] == 1 and cf["start_gate"] is None, cf
    fz = cf["mission"]
    assert fz["same_frame_n"] >= pn.CAMERA_FROZEN_N and fz["same_frame_s"] >= pn.CAMERA_FROZEN_S
    assert fz["t"] < 30.0, fz
    rows = T._rows(run)
    assert rows and rows[-1]["t"] <= fz["t"] + 0.2, (rows[-1]["t"], fz)
    assert link.requested[-1] == "LAND", link.requested
    ev = [e for e in _events(run) if e["kind"] == "camera_frozen"]
    assert len(ev) == 1 and ev[0]["phase"] == "mission" and ev[0]["t"] == fz["t"], ev
    reasons = T._json(run, "kpi.json")["kpi_grade_reasons"]
    assert any(w.startswith("the camera froze") for w in reasons), reasons
    assert "frozen: *** YES, ended the mission" in buf.getvalue()
    rc, run, link = T._fly(grounder_cls=FrameDetections, max_s="20")
    m = T._json(run, "metrics.json")
    assert rc == 0 and m["end_reason"] == "max_s reached" and m["camera_frozen"]["ended"] is None


def test_the_report_line_says_the_detector_s_own_frame_record():
    rec = pn.camera_frozen_record(None, None, SimpleNamespace(frame_stats=lambda: {
        "longest_same_frame_n": 2, "longest_same_frame_s": 0.31, "withheld_inferences": 0,
        "withheld_spans": 0, "withhold_rule": {"same_frame_n": 10, "same_frame_s": 3.0}}))
    assert rec["ended"] is None and rec["detector_frames"]["longest_same_frame_n"] == 2
    line = pn.camera_frozen_line(rec)
    assert line == ("camera: longest run on one unchanged frame 2 inference(s) / 0.31 s; boxes "
                    "withheld on 0 inference(s) in 0 span(s) | frozen: no"), line
    broken = pn.camera_frozen_record(None, None, SimpleNamespace(
        frame_stats=lambda: 1 / 0))
    assert broken["detector_frames"]["error"].startswith("ZeroDivisionError"), broken


# =========================================================================== #
# 2. The detector's stop, and the link's close
# =========================================================================== #

class BusyThread:
    """A Grounder's thread in shape: each 'inference' holds it `infer_s`,
    and the stop flag is read only between inferences."""

    def __init__(self, infer_s=0.3, honour_stop=True):
        self._stop = False
        self.infer_s, self.honour_stop = infer_s, honour_stop
        self.inferences = 0
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self):
        while not (self._stop and self.honour_stop):
            time.sleep(self.infer_s)
            self.inferences += 1
            if not self.honour_stop and self.inferences > 20:
                return

    def stop(self):
        self._stop = True


def test_stop_detector_waits_for_the_inference_in_flight():
    g = BusyThread(0.3)
    time.sleep(0.05)                          # inside its first inference
    said = []
    r = pn.stop_detector(g, timeout_s=5.0, log=said.append)
    assert r["stopped"] is True and not g._thread.is_alive(), r
    assert r["waited_s"] >= 0.1 and not said, r
    # No thread (a stand-in, NoDetector): nothing to wait for.
    assert pn.stop_detector(pn.NoDetector())["stopped"] is None
    assert pn.stop_detector(SimpleNamespace())["stopped"] is None
    # A thread that does not stop is said so, never waited on for ever.
    stuck = BusyThread(0.05, honour_stop=False)
    r = pn.stop_detector(stuck, timeout_s=0.1, log=said.append)
    assert r["stopped"] is False and said and "did not end" in said[0], (r, said)
    stuck._thread.join(10.0)


class BusyDetections(T.GateDetections):
    """GateDetections with a real worker thread that is always mid-inference
    when stop() comes: it then takes `tail_s` more to finish it - longer than
    this stand-in run takes to write its records, so a run() that does not
    wait returns with the thread still running."""

    tail_s = 3.0

    def __init__(self, clock, **kw):
        super().__init__(clock, **kw)
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self):
        self._stop_evt.wait()
        time.sleep(self.tail_s)               # the inference in flight

    def stop(self):
        self._stop_evt.set()


def test_run_joins_the_detector_before_the_camera_goes_and_before_it_returns():
    """The exit-134 regression. run() used to flag the detector to stop and
    return; a no-mission run writes almost nothing after that, so it
    returned while the thread was mid-inference (inside the GPU on the
    Orin, where the interpreter's teardown then aborted the process). Now,
    on the no-mission path and the mission path alike, the thread has ended
    before the camera is stopped, and so before run() returns."""
    alive_at_camera_stop = []
    stop0 = T.FakeCamera.stop
    holder = {}

    def stop(self):
        g = holder.get("g")
        alive_at_camera_stop.append(None if g is None else g._thread.is_alive())

    class Recorded(BusyDetections):
        def __init__(self, clock, **kw):
            super().__init__(clock, **kw)
            holder["g"] = self
    try:
        T.FakeCamera.stop = stop
        for extra in (("--start-when-seen", "5", "--start-timeout-s", "2"),   # no mission
                      ()):                                                    # a mission
            alive_at_camera_stop.clear()
            rc, run, link = T._fly(grounder_cls=Recorded, extra=extra)
            m = T._json(run, "metrics.json")
            assert rc == 0 and (m["ticks"] == 0) == bool(extra), (extra, m["ticks"])
            assert alive_at_camera_stop == [False], (extra, alive_at_camera_stop)
            assert not holder["g"]._thread.is_alive(), extra
    finally:
        T.FakeCamera.stop = stop0


class FakeRclpy:
    def __init__(self, ok=False):
        self._ok, self.calls = ok, []

    def ok(self):
        return self._ok

    def init(self):
        self.calls.append("init")
        self._ok = True

    def create_node(self, name):
        node = T.FakeNode()
        node.destroy_node = lambda: self.calls.append("destroy_node")
        return node

    def try_shutdown(self):
        self.calls.append("try_shutdown")
        self._ok = False


def test_the_mavros_link_ends_the_node_and_rclpy_it_started_and_nothing_else():
    rp = FakeRclpy(ok=False)
    link = ml.MavrosLink(rclpy=rp, msgs=T.FAKE_MSGS, clock=T.Clock(0.01))
    assert rp.calls == ["init"]
    r = link.close()
    assert r == {"node_destroyed": True, "rclpy_shutdown": True, "errors": []}, r
    assert rp.calls == ["init", "destroy_node", "try_shutdown"]
    assert link.close()["node_destroyed"] is False and len(rp.calls) == 3, "not idempotent"
    # rclpy initialised by someone else: the node goes, rclpy stays up.
    rp = FakeRclpy(ok=True)
    r = ml.MavrosLink(rclpy=rp, msgs=T.FAKE_MSGS, clock=T.Clock(0.01)).close()
    assert r["node_destroyed"] and not r["rclpy_shutdown"] and rp.calls == ["destroy_node"]
    # A node handed in is its owner's.
    rp = FakeRclpy(ok=True)
    node = T.FakeNode()
    node.destroy_node = lambda: rp.calls.append("destroy_node")
    r = ml.MavrosLink(node=node, rclpy=rp, msgs=T.FAKE_MSGS, clock=T.Clock(0.01)).close()
    assert r == {"node_destroyed": False, "rclpy_shutdown": False, "errors": []} and not rp.calls
    # A failing destroy is reported, and rclpy is still shut down.
    rp = FakeRclpy(ok=False)
    link = ml.MavrosLink(rclpy=rp, msgs=T.FAKE_MSGS, clock=T.Clock(0.01))
    link.node.destroy_node = lambda: 1 / 0
    r = link.close()
    assert r["rclpy_shutdown"] and r["errors"] and "ZeroDivisionError" in r["errors"][0], r


def test_run_leaves_an_injected_link_open():
    """run() closes only a link it built (MavrosLink); one handed in - every
    test's FakeLink, a caller's link - is its owner's to close."""
    closed = []

    class ClosableLink(T.FakeLink):
        def close(self):
            closed.append(True)
    rc, run, link = T._fly(link_cls=ClosableLink)
    assert rc == 0 and not closed


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs torch / Pillow)")
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
