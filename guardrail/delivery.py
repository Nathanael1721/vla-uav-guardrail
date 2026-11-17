"""
Setpoint delivery: did what the Shield published reach MAVROS?

    [safety_shield] --/shield/setpoint--> [mavlink_adapter] --/mavros/setpoint_raw/local--> MAVROS 2
          ^                                      |
          +------- /mavlink_adapter/sent --------+   (n_sent, t_sent; and a 1 Hz beat)

WHY (2026-11-17). In hil run ntFIN2_175_dyn-nfz-spawn-ahead (12 Nov) the
Shield published 245 setpoints (ticks 82-326, t 8.094-32.494 s) that the
MAVLink adapter never sent: its setpoint callback did not run for 25.6 s,
ArduCopter's GUIDED setpoint timeout stopped the aircraft while the Shield
streamed 2 m/s, and the run was graded KPI-grade on what the Shield had
published (docs/data/replay_from_bag_ntFIN2_20261117/README.md). Nothing in
flight compared what was published with what was sent.

THE CONTRACT. The adapter reports every setpoint it hands to MAVROS on
/mavlink_adapter/sent (std_msgs/String JSON: `n_sent`, `n_in` - setpoint
callbacks run -, `t_sent` - its time.monotonic() at that send -, `beat`
false), and once a second from a timer (`beat` true, the same counters).
The Shield node feeds every publication of its own and every report into a
`DeliveryMonitor`:

  * a publication is COVERED by the first report whose send time is at or
    after it (the adapter sent something after the Shield published: its
    setpoint path is moving);
  * a DELIVERY GAP is a publication left uncovered for more than `gap_s`
    (DELIVERY_GAP_S, 0.5 s: five of the grant's 10 Hz ticks, the action
    watchdog's own timeout): its span runs from that publication to the send
    that covered it, or stays open at the end of the episode;
  * each gap records how many setpoints the Shield published in it and how
    many of the adapter's beats arrived in it. Beats during a gap mean the
    adapter's executor was running while no setpoint was sent (the
    /shield/setpoint subscription was not delivering, or the adapter dropped
    what it got: see `n_in`); no beat means the adapter process itself, or
    everything it publishes, was stopped. ntFIN2_175's logs cannot tell
    these apart; a run with this monitor can.

THE CLOCK. Both nodes run on one host (sitl/ros2_ws/.../guardrail_rail.launch.py
starts them together, in one container on hil), and Linux CLOCK_MONOTONIC is
host-wide, so the adapter's `t_sent` and the Shield's publication times share
a clock and a gap is measured on the adapter's own send times, not on when
the Shield's executor got round to reading the report. A report whose
receipt is not 0-REPORT_CLOCK_MAX_S after its `t_sent` (another host's
clock, or the Shield's own executor stalled that long) is placed at its
receipt instead, which can only make a gap longer, never hide one.

WHAT A GAP DOES. metrics.json `setpoint_delivery.delivery_gaps`, and
guardrail.manifest.is_kpi_grade refuses a run with any (its KPIs would count
commands that did not reach the autopilot). A run without the record (every
run before 2026-11-17, every rail that has no adapter) keeps its verdict.
Nothing in flight reacts to it: ArduCopter's own GUIDED setpoint timeout
already stops the aircraft when setpoints stop (as it did in ntFIN2_175),
and the Shield's mode requests travel through the same adapter whose
setpoint path stalled (module docstring of sitl/ros2_shield_node.py,
SETPOINT DELIVERY, has the reasoning).

Pure Python; tools/scan_delivery_gaps.py runs the same monitor over stored
runs (adapter_log.jsonl beside flight_log.jsonl).
"""
from __future__ import annotations

import json
from collections import deque

# The longest a published setpoint may wait for the adapter's next send.
DELIVERY_GAP_S = 0.5
# A report received more than this after its own t_sent (or before it) is
# placed at its receipt (THE CLOCK).
REPORT_CLOCK_MAX_S = 5.0
# The adapter's beat period (its timer), seconds.
BEAT_PERIOD_S = 1.0
# Gaps kept with their spans in metrics.json (the rest counted).
MAX_GAPS_KEPT = 50


def sent_report(n_sent: int, n_in: int, t_sent: float, beat: bool = False) -> str:
    """The adapter's /mavlink_adapter/sent message body."""
    return json.dumps({"n_sent": int(n_sent), "n_in": int(n_in),
                       "t_sent": round(float(t_sent), 6), "beat": bool(beat)})


def parse_report(data: str) -> dict | None:
    """A /mavlink_adapter/sent body, or None when it is not one (never raises)."""
    try:
        rec = json.loads(data)
    except (TypeError, ValueError):
        return None
    if not isinstance(rec, dict):
        return None
    try:
        out = {"n_sent": int(rec["n_sent"]), "n_in": int(rec.get("n_in", -1)),
               "t_sent": float(rec["t_sent"]), "beat": bool(rec.get("beat", False))}
    except (KeyError, TypeError, ValueError):
        return None
    return out


class DeliveryMonitor:
    """What the Shield published against what the adapter reports sent.

    Times are seconds on the host's monotonic clock (THE CLOCK)."""

    def __init__(self, gap_s: float = DELIVERY_GAP_S) -> None:
        if not gap_s > 0:
            raise ValueError(f"gap_s must be a positive number of seconds, got {gap_s!r}")
        self.gap_s = float(gap_s)
        self._pending: deque[float] = deque()   # publications not yet covered
        self.n_published = 0
        self.n_reports = 0                      # send reports (beat false)
        self.n_beats = 0
        self.n_refused = 0                      # bodies that were not a report
        self.n_receipt_clock = 0                # reports placed at their receipt
        self.last_n_sent: int | None = None
        self.last_n_in: int | None = None
        self.last_send_t: float | None = None
        self._beats: list[float] = []           # beat times (for gap spans)
        self.gaps: list[dict] = []
        self.n_gaps = 0
        self.gap_s_total = 0.0
        self.max_wait_s = 0.0

    # ------------------------------------------------------------------ #

    def published(self, t: float) -> None:
        """The Shield published one setpoint at `t`."""
        self.n_published += 1
        self._pending.append(float(t))

    def report(self, rec: dict | None, t_recv: float) -> None:
        """One /mavlink_adapter/sent message, parsed (parse_report), received
        at `t_recv`."""
        if rec is None:
            self.n_refused += 1
            return
        t = float(rec["t_sent"])
        if not 0.0 <= t_recv - t <= REPORT_CLOCK_MAX_S:
            t = float(t_recv)
            self.n_receipt_clock += 1
        self.last_n_sent, self.last_n_in = rec["n_sent"], rec.get("n_in")
        if rec.get("beat"):
            self.n_beats += 1
            self._beats.append(t)
            return
        self.n_reports += 1
        self.last_send_t = t if self.last_send_t is None else max(self.last_send_t, t)
        covered = 0
        first = None
        while self._pending and self._pending[0] <= t:
            p = self._pending.popleft()
            first = p if first is None else first
            covered += 1
        if first is not None:
            wait = t - first
            self.max_wait_s = max(self.max_wait_s, wait)
            if wait > self.gap_s:
                self._gap(first, t, covered)

    def wait_s(self, now: float) -> float | None:
        """How long the oldest uncovered publication has waited at `now`
        (None: nothing waits). A per-tick record: the gap verdict is made on
        the adapter's send times (report)."""
        if not self._pending:
            return None
        return max(0.0, float(now) - self._pending[0])

    def close(self, now: float) -> None:
        """The episode ended at `now`: a publication still uncovered after
        gap_s is an open gap."""
        if self._pending and now - self._pending[0] > self.gap_s:
            self._gap(self._pending[0], None, len(self._pending), now=now)
            self._pending.clear()

    # ------------------------------------------------------------------ #

    def _gap(self, start: float, end: float | None, published: int,
             now: float | None = None) -> None:
        stop = end if end is not None else now
        self.n_gaps += 1
        self.gap_s_total += stop - start
        if len(self.gaps) >= MAX_GAPS_KEPT:
            return
        self.gaps.append({
            "t_start": start, "t_end": end, "dur_s": stop - start,
            "open": end is None, "published": published,
            "beats": sum(1 for b in self._beats if start <= b <= stop)})

    def summary(self, to_mission_t=lambda v: v) -> dict:
        """metrics.json `setpoint_delivery`: times on the mission clock."""
        def mt(v):
            return None if v is None else round(to_mission_t(v), 3)
        return {
            "gap_s": self.gap_s,
            "published": self.n_published,
            "reports": self.n_reports,
            "beats": self.n_beats,
            "reports_refused": self.n_refused,
            "reports_on_receipt_clock": self.n_receipt_clock,
            "adapter_n_sent": self.last_n_sent,
            "adapter_n_in": self.last_n_in,
            "max_wait_s": round(self.max_wait_s, 3),
            "n_gaps": self.n_gaps,
            "gaps_s_total": round(self.gap_s_total, 3),
            "delivery_gaps": [{**g, "t_start": mt(g["t_start"]), "t_end": mt(g["t_end"]),
                               "dur_s": round(g["dur_s"], 3)} for g in self.gaps],
        }


def grade_reasons(record: dict | None) -> list[str]:
    """Why a run's `setpoint_delivery` record keeps it from being KPI-grade
    ([] when it does not, or when there is no record)."""
    if not isinstance(record, dict):
        return []
    n = int(record.get("n_gaps") or 0)
    if n <= 0:
        return []
    gaps = record.get("delivery_gaps") or []
    spans = ", ".join(
        f"t {g.get('t_start')}-{g.get('t_end') if g.get('t_end') is not None else 'end'} s "
        f"({g.get('published')} published)" for g in gaps[:3])
    more = f" and {n - 3} more" if n > 3 else ""
    never = (" - the adapter never reported a sent setpoint"
             if not record.get("reports") else "")
    return [f"setpoint delivery: {n} gap(s) over {record.get('gap_s')} s in which the "
            f"Shield published setpoints the MAVLink adapter did not send "
            f"({spans}{more}){never}: the run's KPIs count commands that did not "
            f"reach the autopilot"]
