"""The garbage collector kept out of the 10 Hz tick (GcPacer).

Written for sitl/ros2_shield_node.py (2026-11-11), where the measurements
below were taken; moved here on 2026-11-15 so the PAS perception node
(demo/pas_ardupilot/perception_node.py) paces its collector the same way: on
the AGX Orin its loop had one tick of 234-437 ms every 65-80 s of a flight
(pas_hil_feed_04: 6 of 4014 ticks, growing with the flight), the signature of
CPython's full collection walking a heap that grows with the rows the node
keeps.
"""
from __future__ import annotations

import gc
import math
import sys
import time


# THE GARBAGE COLLECTOR (--gc-pacing on, the default since 2026-11-11). On
# the grant example sweep on hil (ge1110, 36 missions) one tick in each long
# mission took 107-120 ms - all of the sweep's ticks over 100 ms, 29 of
# 82,971 - at row 1305-1339 (t 130.5-133.9 s) of EVERY mission, in Normal,
# inside the Shield's own filter() (shield_ms 101-114 ms against a p50 of
# 2.6 ms). The same row in every mission is an allocation-count trigger:
# CPython's full (generation 2) collection, which walks every tracked object
# of the process and runs once the objects promoted since the last one pass
# a quarter of those it kept - here the rows and trajectory points this node
# keeps for its report, ~1300 ticks' worth. GcPacer takes it out of the
# tick: at mission start everything alive (the imports, the policy, its CSP,
# the ROS stack) is collected once and frozen (gc.freeze: never walked
# again), the automatic full collection is switched off (its threshold out
# of reach; the young generations still collect on their own, a few
# thousand objects at a time), and every GC_FULL_EVERY_S the node runs the
# full collection itself right after a setpoint went out and freezes what
# survived, so each one walks only what the last GC_FULL_EVERY_S added. At
# mission end the thresholds come back and everything is unfrozen, before
# the report. --gc-pacing off keeps CPython's own schedule and only counts
# the collections (metrics.json `gc`, either way).
# Measured on the desktop (Python 3.12, ge1110_001's 3502 ticks replayed
# through this node's per-tick code with its rows kept): a full collection
# at row 1320 took 15.0 ms inside the tick with the replay's own heap (87k
# tracked objects) and 53.3 ms with 800k more standing in for the rig's ROS
# stack;
# paced, none ran inside a tick, the pacer's 34 ran after the publication
# in 0.37-0.58 ms each whatever the heap behind them, and its start (collect
# and freeze, before the mission clock) took 11.5 / 53.8 ms.
# tools/profile_shield_tick.py on the same mission (the Shield alone, one
# core): filter() max 8.98 ms with the collector, 3.85 ms with --no-gc.
GC_FULL_EVERY_S = 10.0
# Out of reach for the oldest generation's counter (one per gen-1 collection);
# the C int the threshold is stored in.
GC_THRESHOLD_NEVER = 2 ** 31 - 1


class GcPacer:
    """THE GARBAGE COLLECTOR. `start(t)` once, when the mission clock starts
    (pacing: collect, freeze, the full collection switched off); then
    `after_publish(t)` on every tick right after its setpoint went out (the
    full collection when GC_FULL_EVERY_S passed, returning its ms); `stop()`
    at mission end (thresholds restored, everything unfrozen). With
    `pace=False` it only watches. Every collection while started is counted
    by generation with its longest time (gc.callbacks); a full collection the
    pacer did not run itself is `automatic_full_collections`, which pacing
    keeps at 0. `gc_module` and `clock` are for tests."""

    def __init__(self, every_s: float = GC_FULL_EVERY_S, *, pace: bool = True,
                 gc_module=None, clock=time.perf_counter) -> None:
        if isinstance(every_s, bool) or not isinstance(every_s, (int, float)) \
                or not math.isfinite(every_s) or every_s <= 0:
            raise ValueError(f"GC_FULL_EVERY_S must be a positive number of seconds, "
                             f"got {every_s!r}")
        self.gc = gc if gc_module is None else gc_module
        self.every_s, self.pace, self.clock = float(every_s), bool(pace), clock
        self.active = False
        self.saved_threshold: tuple | None = None
        self.last_t: float | None = None
        self.start_collect_ms: float | None = None
        self.frozen_at_start: int | None = None
        self.frozen_at_stop: int | None = None
        self.n_full = 0
        self.full_ms_max: float | None = None
        self.full_ms_total = 0.0
        self.auto_full = 0
        self.by_gen: dict[int, list] = {}
        self._own = False
        self._c0: float | None = None

    def start(self, t: float) -> None:
        if self.active:
            return
        g = self.gc
        if self.pace:
            c0 = self.clock()
            g.collect()
            g.freeze()
            self.start_collect_ms = round((self.clock() - c0) * 1e3, 3)
            self.frozen_at_start = g.get_freeze_count()
            self.saved_threshold = tuple(g.get_threshold())
            if len(self.saved_threshold) > 2 \
                    and self.saved_threshold[2] == GC_THRESHOLD_NEVER:
                # Another pacer in this process (tests build several nodes):
                # never "restore" the switched-off threshold.
                self.saved_threshold = (self.saved_threshold[0],
                                        self.saved_threshold[1], 10)
            th = list(self.saved_threshold) + [10, 10]
            g.set_threshold(th[0], th[1], GC_THRESHOLD_NEVER)
        g.callbacks.append(self._on_gc)
        self.active, self.last_t = True, float(t)

    def after_publish(self, t: float) -> float | None:
        """The safe point, right after a setpoint went out: the paced full
        collection when one is due (its ms), else None."""
        if not (self.active and self.pace) or float(t) - self.last_t < self.every_s:
            return None
        g = self.gc
        c0 = self.clock()
        self._own = True
        try:
            g.collect()
            g.freeze()
        finally:
            self._own = False
        ms = round((self.clock() - c0) * 1e3, 3)
        self.n_full += 1
        self.full_ms_total += ms
        self.full_ms_max = ms if self.full_ms_max is None else max(self.full_ms_max, ms)
        self.last_t = float(t)
        return ms

    def stop(self) -> None:
        if not self.active:
            return
        g = self.gc
        try:
            g.callbacks.remove(self._on_gc)
        except ValueError:
            pass
        if self.pace:
            self.frozen_at_stop = g.get_freeze_count()
            g.unfreeze()
            if self.saved_threshold is not None:
                g.set_threshold(*self.saved_threshold)
        self.active = False

    def _on_gc(self, phase: str, info: dict) -> None:
        # Never raises: it runs inside the interpreter's collection.
        try:
            if phase == "start":
                self._c0 = self.clock()
                return
            if self._c0 is None:
                return
            ms = (self.clock() - self._c0) * 1e3
            self._c0 = None
            gen = int(info.get("generation", -1))
            rec = self.by_gen.setdefault(gen, [0, 0.0])
            rec[0] += 1
            rec[1] = max(rec[1], ms)
            if gen >= 2 and not self._own:
                self.auto_full += 1
        except Exception:                                   # noqa: BLE001
            pass

    def summary(self) -> dict:
        return {
            "mode": "paced" if self.pace else "cpython default (watched only)",
            "flag": "--gc-pacing " + ("on" if self.pace else "off"),
            "python": sys.version.split()[0],
            "every_s": self.every_s if self.pace else None,
            "start_collect_ms": self.start_collect_ms,
            "frozen_at_start": self.frozen_at_start,
            "frozen_at_stop": self.frozen_at_stop,
            "paced_full_collections": self.n_full,
            "paced_full_ms_max": self.full_ms_max,
            "paced_full_ms_total": round(self.full_ms_total, 3),
            "automatic_full_collections": self.auto_full,
            "collections_by_generation": {
                str(g): {"n": n, "max_ms": round(mx, 3)}
                for g, (n, mx) in sorted(self.by_gen.items())},
            "threshold_restored": (list(self.saved_threshold)
                                   if self.saved_threshold is not None else None),
            "rule": ("at mission start: collect, gc.freeze(), the automatic full "
                     "collection off; every every_s a full collection right after a "
                     "setpoint is published, then gc.freeze(); at mission end the "
                     "thresholds restored and gc.unfreeze()" if self.pace else
                     "CPython's own thresholds; collections counted only")}
