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


class FrameRecorder(threading.Thread):
    """Third-person and annotated first-person frames, written on their own thread."""

    def __init__(self, obs, out_dir: Path, annotate_fn: Callable,
                 hz: float = 20.0, height: int = 720, quality: int = 88):
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
        self.n_slow = 0
        self.t0: Optional[float] = None
        self.t_stop: Optional[float] = None

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

    def run(self) -> None:
        self.t0 = time.time()
        nxt = self.t0
        while not self._stop_evt.is_set():
            now = time.time()
            if now < nxt:
                time.sleep(min(0.01, nxt - now))
                continue
            nxt += self.period
            if now - nxt > 1.0:             # fell far behind; resynchronise
                nxt = now + self.period
                self.n_slow += 1

            with self._lock:
                hud, det = self._hud, self._det
            if hud is None:
                continue

            idx = self.n_written
            try:
                v = self.obs.get_chase_native()
                if v is not None:
                    v.save(self.tps_dir / f"{idx:05d}.jpg", quality=self.quality)
                f = self.obs.get_front_native()
                if f is not None:
                    f, det_s = self._upscale(f, det)
                    self.annotate(f, det_s, hud).save(
                        self.fpv_dir / f"{idx:05d}.jpg", quality=self.quality)
                self.n_written += 1
            except Exception:
                # A recording fault must never take the flight down with it.
                self.n_slow += 1

    def summary(self) -> dict:
        end = self.t_stop or time.time()
        secs = (end - self.t0) if self.t0 else 0.0
        return {
            "frames": self.n_written,
            "target_hz": round(1.0 / self.period, 1),
            "achieved_hz": round(self.n_written / secs, 2) if secs > 1 else None,
            "seconds": round(secs, 2),
            "late_or_failed": self.n_slow,
        }

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
