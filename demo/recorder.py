"""Record the demo off the control loop, so filming never slows the aircraft.

WHY THIS EXISTS

Recording used to happen inline, in `follow_vlm.fly()`:

    if view_dir is not None and tick % args.view_every == 0:
        obs.get_chase_native().save(...)          # decode + JPEG encode
        annotate(obs.get_front_native(), ...).save(...)   # decode + draw + encode

Two decodes, a PIL draw and two JPEG encodes, on the same thread that flies the
aircraft. At `--view-every 3` the cost was spread over three ticks and the loop
held 10 Hz. Asking for every frame put all of it on every tick and the loop fell
to 7.0-7.4 Hz, measured:

    before   586 ticks / 58.6 s = 10.0 Hz    tick dt p50 100 ms
    after    588 ticks / 79.8 s =  7.4 Hz    tick dt p50 132 ms

So the aircraft genuinely issued commands 7 times a second instead of 10. That is
the "lag" on the video, and the 7 fps recording is the "choppy". Both came from
the camera work, not from the controller.

Here the control loop only calls `set_hud()` with a small dict it has already
computed. This thread does the decoding, drawing, encoding and writing at its own
cadence.

TWO RULES THIS FILE KEEPS

* **Never block the flight.** If a frame is slow to write, drop it. A recorder
  that applies back-pressure would recreate exactly the fault it was built to fix.
* **Poll at a fixed rate.** Frames are written on a clock, not on new-message
  arrival, so `frames / flight_seconds` equals the poll rate and the video's
  duration is right by construction. A duplicated frame is invisible; a wrong
  playback speed is not, and that claim has already been made wrongly once.
"""
from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Callable, Optional

# One window, named so a second recorder cannot open a rival one.
LIVE_WINDOW = "VLA Guardrail - drone view | chase view"


class FrameRecorder(threading.Thread):
    """Third-person and annotated first-person frames, written on their own thread."""

    def __init__(self, obs, out_dir: Path, annotate_fn: Callable,
                 hz: float = 20.0, height: int = 720, quality: int = 88,
                 live_view: bool = False, live_height: int = 480):
        super().__init__(daemon=True)
        self.obs = obs
        self.tps_dir = Path(out_dir) / "tps"
        self.fpv_dir = Path(out_dir) / "fpv"
        self.tps_dir.mkdir(parents=True, exist_ok=True)
        self.fpv_dir.mkdir(parents=True, exist_ok=True)
        self.annotate = annotate_fn
        self.period = 1.0 / max(1.0, float(hz))
        self.height = int(height)
        self.quality = int(quality)
        # Show the pair being written, in a window, while the aircraft flies.
        # Costs no RPC: both frames are already decoded here for the file
        # write, so this is one hstack and one imshow on a thread that is
        # deliberately not the control loop.
        self.live_view = bool(live_view)
        self.live_height = int(live_height)
        self.n_shown = 0
        self._live_failed = False

        self._lock = threading.Lock()
        self._hud: Optional[dict] = None
        self._det = None
        # NOT `self._stop`. threading.Thread already has a private method of that
        # name and Thread.join() calls it internally, so assigning an Event over
        # it makes every join() raise `TypeError: 'Event' object is not callable`.
        # That is not cosmetic: join() is the first statement of the teardown
        # thread, so the exception killed teardown outright - the recorder
        # summary was never printed, spawned props were never destroyed, and the
        # simulator connection was never closed. It looked like a recorder bug
        # and was actually a name collision.
        self._stop_evt = threading.Event()
        self.n_written = 0
        self.n_late = 0          # arrived more than half a period after the slot
        self.n_skipped = 0       # no HUD yet, so nothing to stamp
        self.n_empty = 0         # both cameras returned nothing
        self.n_failed = 0        # an exception during capture or encode
        self.n_cleared = 0       # stale frames removed from a previous flight
        self.t0: Optional[float] = None
        self.t_stop: Optional[float] = None
        # When the FIRST frame was written, which is not when the recorder
        # started. See summary() - the gap between the two is the whole reason
        # every delivered video played a third too slow.
        self.t_first_write: Optional[float] = None

    # ---------------------------------------------------------- from the loop --

    def set_hud(self, hud: dict, det) -> None:
        """Called once per control tick. Must stay cheap - it is on the flight path.

        The recorder stamps whatever was last handed to it, so the overlay can be
        up to one recorder period stale: 50 ms at 20 Hz. That is invisible, and
        far cheaper than the 32-40 ms per tick the inline version cost.
        """
        with self._lock:
            self._hud = hud
            self._det = det

    def stop(self) -> None:
        # Stamp the wall clock here, not in summary(): summary() may be called
        # well after the thread has finished, and the gap would inflate the
        # measured duration and understate the achieved rate.
        self.t_stop = time.time()
        self._stop_evt.set()
        # The window is NOT closed here. stop() runs on the flight thread and
        # the window belongs to the recorder thread; OpenCV windows are
        # thread-affine, and destroying one from another thread blocked stop()
        # long enough that the run never reached write_sidecar(). The result
        # was a flight with 1728 frames and no recorder.json, which sent the
        # video builder down its fallback path and produced a clip that plays
        # 1.24x fast - the exact fault the sidecar exists to prevent.
        #
        # run() closes it instead, on its way out.

    # ------------------------------------------------------------- the thread --

    def _upscale(self, im, det):
        """Enlarge BEFORE drawing, so the overlay is crisp.

        The FPV frame is 400x225 and the video panel is `height` tall, so it was
        being magnified 2.1x AFTER the box and text were drawn - which blurred the
        overlay as well as the picture. Scaling first costs one resize and makes
        the HUD sharp. The underlying image is still the detector's real 400x225
        view; nothing about it is faked.
        """
        from PIL import Image
        if im.height >= self.height:
            return im, det
        s = self.height / im.height
        im = im.resize((int(im.width * s), self.height), Image.LANCZOS)
        if det is not None:
            det = list(det)
            for i in (0, 1, 2, 3):          # cx, cy, bw, bh
                det[i] = float(det[i]) * s
            if len(det) > 5:
                det[5] = float(det[5]) * s  # image width, used by the servo tag
        return im, det

    def _clear(self) -> int:
        """Remove frames from previous flights before recording new ones.

        These directories were never cleared, and tools/make_demo_video.py globs
        them, so a flight shorter than its predecessor produced a video ending in
        somebody else's footage. Measured on the delivered demos: 36, 7 and 8
        stale frames on the tail of the three clips. The frames are numbered from
        zero every run, so the leftovers are always at the END, which is exactly
        where nobody looks.
        """
        n = 0
        for d in (self.tps_dir, self.fpv_dir):
            for f in d.glob("*.jpg"):
                try:
                    f.unlink()
                    n += 1
                except OSError:
                    pass
        return n

    def run(self) -> None:
        self.n_cleared = self._clear()
        self.t0 = time.time()
        nxt = self.t0
        while not self._stop_evt.is_set():
            now = time.time()
            if now < nxt:
                time.sleep(min(0.01, nxt - now))
                continue

            # Count the slot as missed when we arrive more than half a period
            # late. The old test asked whether we were a FULL SECOND behind -
            # twenty periods at 20 Hz - and then resynchronised, which forgave
            # the accumulated debt. A recorder running steadily 25 percent below
            # its target therefore reported late_or_failed = 0, and the one
            # number that would have revealed it was the one being suppressed.
            if now - nxt > self.period * 0.5:
                self.n_late += 1
            nxt += self.period
            if now - nxt > 1.0:             # far behind; resynchronise
                nxt = now + self.period

            with self._lock:
                hud, det = self._hud, self._det
            if hud is None:
                self.n_skipped += 1
                continue

            idx = self.n_written
            try:
                wrote = False
                ann = None
                v = self.obs.get_chase_native()
                if v is not None:
                    v.save(self.tps_dir / f"{idx:05d}.jpg", quality=self.quality)
                    wrote = True
                f = self.obs.get_front_native()
                if f is not None:
                    f, det_s = self._upscale(f, det)
                    # Kept, not written inline: the live window shows this exact
                    # image, so drawing it twice would be both wasteful and a
                    # chance for the window and the file to disagree.
                    ann = self.annotate(f, det_s, hud)
                    ann.save(self.fpv_dir / f"{idx:05d}.jpg", quality=self.quality)
                    wrote = True
                if self.live_view and not self._live_failed:
                    self._show(ann, v)
                # Count WRITES, not polls. This incremented even when both
                # cameras returned None, so the index advanced without a file
                # behind it and the count disagreed with the directory.
                if wrote:
                    if self.t_first_write is None:
                        self.t_first_write = now
                    self.n_written += 1
                else:
                    self.n_empty += 1
            except Exception:
                # A recording fault must never take the flight down with it.
                self.n_failed += 1

        # Out of the loop: the thread that opened the window is the only one
        # allowed to close it.
        if self.live_view and not self._live_failed:
            try:
                import cv2
                cv2.destroyWindow(LIVE_WINDOW)
                cv2.waitKey(1)
            except Exception:                                     # noqa: BLE001
                pass

    def _show(self, ann, chase) -> None:
        """Draw the same two panels into a window. Never raises.

        A display fault must cost a frame, not the mission - the same rule the
        rest of this loop keeps. On the first failure the window is disabled
        for the remainder of the flight and said once, because a headless host
        would otherwise print a traceback twenty times a second.
        """
        try:
            import cv2
            from two_view import pil_to_bgr, side_by_side
            panel = side_by_side(pil_to_bgr(ann), pil_to_bgr(chase),
                                 height=self.live_height)
            if panel is None:
                return
            cv2.imshow(LIVE_WINDOW, panel)
            cv2.waitKey(1)          # required for the window to repaint
            self.n_shown += 1
        except Exception as exc:                                  # noqa: BLE001
            self._live_failed = True
            print(f"[recorder] live view unavailable ({type(exc).__name__}: "
                  f"{exc}); recording continues")

    def summary(self) -> dict:
        """What was captured, and how fast it was ACTUALLY captured.

        The rate is measured over the window in which frames were being
        written, not over the recorder's whole life. Those are different
        windows, and using the wrong one is not a rounding error.

        The recorder is started at scene setup but writes nothing until the
        control loop first calls `set_hud()`, which is after arming and the
        climb to cruise - about 27 s later on these missions. `_hud` is never
        cleared once set, so every skipped slot is in that opening stretch and
        the writing window is one contiguous block at the end.

        Dividing by the whole life therefore counted ~27 s of deliberate
        silence as recording time. Measured across the eleven runs on disk, it
        reported 14.6-16.7 Hz for a recorder that was hitting 20.00 Hz exactly,
        every time. tools/make_demo_video.py takes its fps from this number, so
        every video delivered up to 2026-08-25 plays about 1.3x slower than
        real time while the tool prints that the duration is correct.

        `skipped_no_hud` is still reported. It was never wrong - it was only
        the wrong thing to leave in the denominator.
        """
        end = self.t_stop or time.time()
        secs = (end - self.t0) if self.t0 else 0.0
        writing = (end - self.t_first_write) if self.t_first_write else 0.0
        target = round(1.0 / self.period, 1)
        achieved = round(self.n_written / writing, 2) if writing > 1 else None
        s = {
            "frames": self.n_written,
            "target_hz": target,
            "achieved_hz": achieved,
            "seconds": round(secs, 2),
            "writing_seconds": round(writing, 2),
            "late_slots": self.n_late,
            # Frames actually painted into the live window. Zero with the
            # window off; well below `frames` would mean the display was
            # costing the recorder slots, which is the thing it promised not
            # to do.
            "live_frames_shown": self.n_shown,
            "skipped_no_hud": self.n_skipped,
            "empty_captures": self.n_empty,
            "failed": self.n_failed,
            "stale_frames_cleared": self.n_cleared,
        }
        # State the shortfall rather than leaving it to be inferred from two
        # numbers a reader has to divide.
        #
        # This warning fired on every run of the midterm campaign and was a
        # false alarm every time: it was reading a rate computed over the wrong
        # window. A warning that is always on teaches its reader to ignore it,
        # which is worse than not having one - so the number it judges is now
        # the real capture rate.
        if achieved and target:
            s["rate_ratio"] = round(achieved / target, 3)
            if achieved < target * 0.9:
                s["WARNING"] = (f"recorded at {achieved} Hz against a {target} Hz "
                                f"target ({100 * achieved / target:.0f} %) over "
                                f"{writing:.1f} s of writing; the video's duration is "
                                f"right because fps is taken from achieved_hz, but "
                                f"it is choppier than the target asked for")
        return s

    def write_sidecar(self, path) -> dict:
        """Persist the achieved rate next to the frames.

        The video builder needs to know how fast these frames were actually
        captured, and it cannot infer that from the flight log. The recorder
        starts before the mission clock does (scene load, prop spawn, take-off)
        and stops after it, so `frames / flight_seconds` overstates the rate:
        1734 frames captured over ~87 s of recorder life divided by a 70 s
        mission gives 24.8 Hz for a recorder that was pacing at 20. A video
        written at that figure plays 1.24x fast while claiming real time.

        Only the recorder knows both numbers, so only the recorder can answer
        this. It writes them down.
        """
        import json
        from pathlib import Path
        s = self.summary()
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(s, indent=2), encoding="utf-8")
        return s
