"""
Setpoint delivery: did the MAVLink adapter hand to MAVROS what the Shield
published?

    [safety_shield] --/shield/setpoint--> [mavlink_adapter] --/mavros/setpoint_raw/local--> MAVROS 2 --> autopilot
          ^                                      |                                        (NOT monitored here)
          +------- /mavlink_adapter/sent --------+   (n_sent, t_sent; and a 1 Hz beat)

WHY (2026-11-17). In hil run ntFIN2_175_dyn-nfz-spawn-ahead (12 Nov) the
Shield published 245 setpoints (ticks 82-326, t 8.094-32.494 s) that the
MAVLink adapter never sent: its setpoint callback did not run for 25.6 s,
ArduCopter's GUIDED setpoint timeout stopped the aircraft while the Shield
streamed 2 m/s, and the run was graded KPI-grade on what the Shield had
published (docs/data/replay_from_bag_ntFIN2_20261117/README.md). Nothing in
flight compared what was published with what was sent.

WHAT IS CHECKED, AND WHAT IS NOT. The Shield-to-adapter hop: that the
adapter called publish() on /mavros/setpoint_raw/local for the Shield's
setpoints. Not checked: that MAVROS forwarded them or that the autopilot
received them. A clean record means "handed to MAVROS by the adapter", not
"reached the autopilot"; a MAVROS-to-FC link failure leaves it clean. The
autopilot's echo (/mavros/setpoint_raw/target_local) is in the bag but is
not compared here.

THE CONTRACT. The adapter reports every setpoint it hands to MAVROS on
/mavlink_adapter/sent (std_msgs/String JSON: `n_sent`, `n_in` - setpoint
callbacks run -, `t_sent` - its time.monotonic() at that send -, `beat`
false), and once a second from a timer (`beat` true, the same counters).
The Shield node feeds every publication of its own (stamped just BEFORE its
publish() call, so the adapter's send of it can never carry an earlier
time) and every report into a `DeliveryMonitor`. Two checks:

  * STALLS. A publication is COVERED by the first report whose send time is
    at or after it (the adapter sent something after the Shield published:
    its setpoint path is moving). A DELIVERY GAP is a publication left
    uncovered for more than `gap_s` (DELIVERY_GAP_S, 0.5 s: five of the
    grant's 10 Hz ticks, the action watchdog's own timeout): its span runs
    from that publication to the send that covered it, or stays open at the
    end of the episode. Each gap records how many setpoints the Shield
    published in it and how many of the adapter's beats arrived in it.
    Beats during a gap mean the adapter's executor was running while no
    setpoint was sent (the /shield/setpoint subscription was not delivering,
    or the adapter dropped what it got: see `n_in`); no beat means the
    adapter process itself, or everything it publishes, was stopped.
    ntFIN2_175's logs cannot tell these apart; a run with this monitor can.
  * SHORTFALL. Coverage by "any later send" cannot see setpoints lost while
    other sends continue (every other one dropped, a KEEP_LAST overflow from
    a slow callback, a heading-less or invalid body the adapter refused).
    So the counts are compared too: `published - reports` (setpoints the
    Shield published that no send report answered) over
    DELIVERY_SHORTFALL_MAX (2: the last setpoints may still be in flight
    when the episode closes) is a shortfall. On the 1,833 healthy stored hil
    runs the two counts are equal (docs/data/delivery_gaps_hil_smoke_20261117).

Not measured: how stale an individual send is when nothing is dropped (a
backlog). The adapter's subscription keeps the last 10 at 10 Hz, so a
backlog older than about 1 s overflows and shows as a shortfall; one under
that does not show.

THE CLOCK. Both nodes run on one host (sitl/ros2_ws/.../guardrail_rail.launch.py
starts them together, in one container on hil), and Linux CLOCK_MONOTONIC is
host-wide, so the adapter's `t_sent` and the Shield's publication times share
a clock and a gap is measured on the adapter's own send times, not on when
the Shield's executor got round to reading the report. A report whose
receipt is not 0-REPORT_CLOCK_MAX_S after its `t_sent` (a clock far off this
host's, or the Shield's own executor stalled that long) is placed at its
receipt instead. That protects only against a skew larger than
REPORT_CLOCK_MAX_S: a report from a clock 0-5 s behind this host's is
accepted at its own `t_sent` and could cover publications it did not follow,
hiding a stall of up to that skew. On the one-host rail that cannot happen;
the shortfall count does not depend on the clock.

WHAT A GAP OR SHORTFALL DOES. metrics.json `setpoint_delivery`, and
guardrail.manifest.is_kpi_grade refuses a run with either (its KPIs would
count commands the adapter never handed to MAVROS). Runs flown before
2026-11-17 carry no record; stored_record() derives one from the run
folder's two logs (scan_logs, the same monitor) when the folder has an
adapter log, so the regrading tools (tools/kpi_report.py,
tools/rescore_kpis.py, tools/build_eval_data.py) see ntFIN2_175's gap. A
rail without an adapter has neither and keeps its verdict. Nothing in flight
reacts: ArduCopter's own GUIDED setpoint timeout already stops the aircraft
when setpoints stop (as it did in ntFIN2_175), and the Shield's mode
requests travel through the same adapter whose setpoint path stalled
(module docstring of sitl/ros2_shield_node.py, SETPOINT DELIVERY, has the
reasoning).

Pure Python; tools/scan_delivery_gaps.py is the command line over scan_logs.
"""
from __future__ import annotations

import json
import struct
from collections import Counter, deque
from pathlib import Path

# The longest a published setpoint may wait for the adapter's next send.
DELIVERY_GAP_S = 0.5
# Setpoints published that no send report answered, beyond which a run is
# not KPI-grade (SHORTFALL): the last ones may still be in flight at close.
DELIVERY_SHORTFALL_MAX = 2
# A report received more than this after its own t_sent (or before it) is
# placed at its receipt (THE CLOCK).
REPORT_CLOCK_MAX_S = 5.0
# The adapter's beat period (its timer), seconds.
BEAT_PERIOD_S = 1.0
# History depth of /mavlink_adapter/sent on both sides: deep enough that a
# Shield executor busy for a few seconds does not lose reports (a lost report
# would read as a SHORTFALL).
SENT_QOS_DEPTH = 100
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
            # SHORTFALL: published setpoints no send report answered.
            "shortfall": self.n_published - self.n_reports,
            "n_gaps": self.n_gaps,
            "gaps_s_total": round(self.gap_s_total, 3),
            "delivery_gaps": [{**g, "t_start": mt(g["t_start"]), "t_end": mt(g["t_end"]),
                               "dur_s": round(g["dur_s"], 3)} for g in self.gaps],
        }


def grade_reasons(record: dict | None) -> list[str]:
    """Why a run's `setpoint_delivery` record keeps it from being KPI-grade
    ([] when it does not, or when there is no record): a STALL (a gap) and a
    SHORTFALL (published minus reports over DELIVERY_SHORTFALL_MAX) are each
    a reason."""
    if not isinstance(record, dict):
        return []
    out = []
    src = (f" ({record['source']})" if isinstance(record.get("source"), str)
           and record.get("source") else "")
    try:
        n = int(record.get("n_gaps") or 0)
    except (TypeError, ValueError):
        n = 0
    if n > 0:
        gaps = record.get("delivery_gaps") or []
        spans = ", ".join(
            f"t {g.get('t_start')}-{g.get('t_end') if g.get('t_end') is not None else 'end'} s "
            f"({g.get('published')} published)" for g in gaps[:3] if isinstance(g, dict))
        more = f" and {n - 3} more" if n > 3 else ""
        never = (" - the adapter never reported a sent setpoint"
                 if not record.get("reports") else "")
        out.append(f"setpoint delivery: {n} gap(s) over {record.get('gap_s')} s in which "
                   f"the Shield published setpoints the MAVLink adapter did not hand to "
                   f"MAVROS ({spans}{more}){never}{src}: the run's KPIs count commands "
                   f"that never left the rail")
    try:
        pub, rep = int(record.get("published")), int(record.get("reports"))
    except (TypeError, ValueError):
        pub = rep = None
    if pub is not None and pub - rep > DELIVERY_SHORTFALL_MAX:
        out.append(f"setpoint delivery shortfall: the Shield published {pub} setpoints "
                   f"and the MAVLink adapter reported handing {rep} to MAVROS "
                   f"({pub - rep} unanswered, more than {DELIVERY_SHORTFALL_MAX} allowed "
                   f"in flight at close; adapter setpoint callbacks run: "
                   f"{record.get('adapter_n_in')}){src}: the run's KPIs count commands "
                   f"that never left the rail")
    return out


# --------------------------------------------------------------------------- #
# Stored runs: the same monitor over a run folder's two logs
# --------------------------------------------------------------------------- #

PAIR_BIN_S = 0.05
# A publication is placed this much before its estimate (scan_logs): with the
# row's tick_ms, the estimate is the publication itself; without it, only the
# tick's start, so the margin is wider.
SCAN_MARGIN_S = 0.01
SCAN_MARGIN_NO_TICK_MS_S = 0.05
# The offset search takes the smallest send-minus-publication difference at
# most this far below the commonest one (the publication's own send, the
# shortest hop; a cluster split across two bins is still found whole).
OFFSET_WINDOW_S = 0.03
SCAN_SOURCE = ("derived after the flight from flight_log.jsonl and adapter_log.jsonl "
               "by guardrail.delivery.scan_logs")


def _f32(v) -> float:
    return struct.unpack("f", struct.pack("f", float(v)))[0]


def _key(body) -> tuple | None:
    if body is None:
        return None
    if isinstance(body, dict):
        body = [body.get("vx"), body.get("vy"), body.get("vz_up"), body.get("yaw_rate")]
    try:
        return tuple(_f32(v) for v in body)
    except (TypeError, ValueError, struct.error):
        return None


def _jsonl(path: Path) -> list[dict]:
    out = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                try:
                    rec = json.loads(line)
                except ValueError:
                    continue
                if isinstance(rec, dict):
                    out.append(rec)
    return out


def clock_offset(published: list[tuple[float, tuple]],
                 sent: list[tuple[float, tuple]]) -> float | None:
    """adapter time - mission time, from setpoints sent with a body the Shield
    published (None when no sent body matches a published one): the smallest
    difference within OFFSET_WINDOW_S below the median of the commonest
    PAIR_BIN_S bin."""
    by_key: dict[tuple, list[float]] = {}
    for t, k in published:
        by_key.setdefault(k, []).append(t)
    diffs = []
    for ts, k in sent:
        for tp in by_key.get(k, ())[:50]:
            diffs.append(ts - tp)
    if not diffs:
        return None
    bins = Counter(round(d / PAIR_BIN_S) for d in diffs)
    best = bins.most_common(1)[0][0]
    in_best = sorted(d for d in diffs if round(d / PAIR_BIN_S) == best)
    mid = in_best[len(in_best) // 2]
    return min(d for d in diffs if mid - OFFSET_WINDOW_S <= d <= mid)


def scan_logs(run, gap_s: float = DELIVERY_GAP_S) -> dict:
    """One stored run folder: its delivery record as the Shield node would
    have written it (`setpoint_delivery`, with `source`), plus how the clocks
    were tied. `status`: "ok", "gap" (a stall), "shortfall" (counts only),
    or why nothing was assessed ("no flight_log.jsonl", "no adapter_log.jsonl",
    "nothing published", "clocks not tied: ...").

    flight_log.jsonl: each row whose `emitted_body` is not null published one
    setpoint; its publication is at the row's `t` + `tick_ms` (the row's
    tick start to that publication) when the row has it, else at `t`.
    adapter_log.jsonl: each line with a `body` is one setpoint the adapter
    handed to MAVROS at its `t` (the adapter's time.monotonic()); other
    lines (mission pushes, statuses) are not sends.

    The clocks are tied by the setpoints themselves (a sent `body` equals the
    published `emitted_body` bit for bit, both float32: clock_offset), and
    each publication is placed SCAN_MARGIN_S (SCAN_MARGIN_NO_TICK_MS_S without
    tick_ms) early so that none lands after its own send. BOUNDS: a stall
    reads long by up to that margin; and it can read SHORT by up to one tick
    plus the margin, when the previous setpoint's send falls inside the margin
    before a publication (that publication is then covered by the earlier
    send and the stall is measured from the next one). The counts (published,
    sent) do not depend on the placement. A gap's `t_start` is reported at
    its first publication's estimate (the margin added back)."""
    run = Path(run)
    rec: dict = {"run": run.name}
    fl, al = run / "flight_log.jsonl", run / "adapter_log.jsonl"
    if not fl.is_file():
        return {**rec, "status": "no flight_log.jsonl"}
    if not al.is_file():
        return {**rec, "status": "no adapter_log.jsonl"}
    published, margins = [], []
    for r in _jsonl(fl):
        if r.get("emitted_body") is None or r.get("t") is None:
            continue
        k = _key(r.get("emitted_body"))
        if k is None:
            continue
        tm = r.get("tick_ms")
        if isinstance(tm, (int, float)) and not isinstance(tm, bool):
            published.append((float(r["t"]) + float(tm) / 1000.0 - SCAN_MARGIN_S, k))
            margins.append(SCAN_MARGIN_S)
        else:
            published.append((float(r["t"]) - SCAN_MARGIN_NO_TICK_MS_S, k))
            margins.append(SCAN_MARGIN_NO_TICK_MS_S)
    sent = [(float(a["t"]), _key(a.get("body"))) for a in _jsonl(al)
            if a.get("t") is not None and a.get("body") is not None]
    rec.update(published=len(published), sent=len(sent))
    if not published:
        return {**rec, "status": "nothing published"}
    if sent:
        off = clock_offset(published, sent)
        if off is None:
            return {**rec, "status": "clocks not tied: no sent body matches a published one"}
    else:
        off = 0.0
    mon = DeliveryMonitor(gap_s)
    events = ([(t + off, 0, None) for t, _ in published]
              + [(t, 1, n) for n, (t, _) in enumerate(sent, 1)])
    events.sort(key=lambda e: (e[0], e[1]))
    for t, kind, n in events:
        if kind == 0:
            mon.published(t)
        else:
            mon.report({"n_sent": n, "n_in": -1, "t_sent": t, "beat": False}, t)
    mon.close(max(t for t, _ in published) + off + SCAN_MARGIN_NO_TICK_MS_S)
    # A gap starts at a placed publication; report it at the publication's
    # estimate (its margin added back). The offset ties the adapter's clock
    # to PLACED times, so a send's mission time takes the run's usual margin
    # back too.
    margin_at = {t + off: m for (t, _), m in zip(published, margins)}
    m_ref = Counter(margins).most_common(1)[0][0]
    for g in mon.gaps:
        g["t_start"] += margin_at.get(g["t_start"], 0.0)
        if g["t_end"] is not None:
            g["t_end"] += m_ref
    s = mon.summary(lambda v: v - off)
    s["adapter_n_in"] = None                 # not in the stored logs
    s["source"] = SCAN_SOURCE
    status = ("gap" if s["n_gaps"] else
              "shortfall" if s["shortfall"] > DELIVERY_SHORTFALL_MAX else "ok")
    return {**rec, "status": status, "clock_offset_s": round(off, 4),
            "setpoint_delivery": s}


def stored_record(run, metrics: dict | None = None) -> dict | None:
    """The run's `setpoint_delivery` record for grading a stored run: the one
    in `metrics` (or in the folder's metrics.json) when the Shield wrote it,
    else, for a run flown before 2026-11-17 whose folder has both logs, the
    scan of them (scan_logs). None when neither can say (no adapter, nothing
    published, clocks not tied): the verdict then does not move."""
    if isinstance(metrics, dict) and isinstance(metrics.get("setpoint_delivery"), dict):
        return metrics["setpoint_delivery"]
    if run is None:
        return None
    run = Path(run)
    mp = run / "metrics.json"
    if mp.is_file():
        try:
            m = json.loads(mp.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            m = None
        if isinstance(m, dict) and isinstance(m.get("setpoint_delivery"), dict):
            return m["setpoint_delivery"]
    if not (run / "adapter_log.jsonl").is_file():
        return None
    try:
        scan = scan_logs(run)
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if scan.get("status") in ("ok", "gap", "shortfall"):
        return scan["setpoint_delivery"]
    return None


def with_stored_delivery(metrics: dict | None, run) -> dict:
    """`metrics` with `setpoint_delivery` filled from stored_record() when it
    lacks one, for guardrail.manifest.is_kpi_grade on a stored run."""
    m = dict(metrics) if isinstance(metrics, dict) else {}
    if not isinstance(m.get("setpoint_delivery"), dict):
        rec = stored_record(run, m)
        if rec is not None:
            m["setpoint_delivery"] = rec
    return m
