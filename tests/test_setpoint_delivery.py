"""Setpoint delivery (2026-11-17): what the Shield published against what the
MAVLink adapter sent, and the KPI grade that reads it.

    python tests/test_setpoint_delivery.py      (no pytest needed)

hil run ntFIN2_175_dyn-nfz-spawn-ahead (12 Nov) published 245 setpoints the
adapter never sent (its setpoint callback did not run for 25.6 s) and was
graded KPI-grade on them; nothing compared the two. guardrail/delivery.py is
the comparison, sitl/mavlink_adapter_node.py reports what it sent on
/mavlink_adapter/sent, sitl/ros2_shield_node.py records the gaps in
metrics.json `setpoint_delivery`, guardrail.manifest.is_kpi_grade refuses a
run with any, and tools/scan_delivery_gaps.py runs the same monitor over
stored runs.

  * the monitor alone: a healthy 10 Hz stream has no gap; a stalled adapter
    gives one gap with its span, the setpoints published in it and the
    adapter's beats in it; a stall that outlasts the episode is an open gap;
    a report on another clock is placed at its receipt; junk is counted;
  * the grade: a run that is otherwise KPI-grade is not with a gap, and a
    healthy record or no record (every run before this date) changes nothing;
  * both nodes on stand-in ROS stacks (the Shield from
    tests/test_railinputs_node.py, the adapter from tests/test_mavlink_adapter.py)
    wired back to back: the adapter's real setpoint callback, stalled for
    3 s as in ntFIN2_175 (its subscription keeps the last 10, delivered when
    it runs again), gives a delivery gap in metrics.json and a kpi.json that
    says why the run is not KPI-grade; the same flight unstalled gives none;
  * the offline scan: the same monitor over a run folder's two logs.

Shown failing first against integrate/1117 (4afd2a1): no guardrail.delivery,
the adapter had no TOPIC_SENT, and the Shield's metrics.json had no
`setpoint_delivery`.

Added after review (17 Nov), each shown failing against 7adcbaa: the
SHORTFALL count (every other setpoint dropped with no 0.5 s silence was
KPI-grade), the publication stamped before publish() (a send stamped inside
it left the setpoint waiting across a hold), the scan's placement at t +
tick_ms (a 508 ms tick read as a gap) and its sends read from `body` lines
only, and stored runs graded on the record their logs give
(guardrail.delivery.stored_record: ntFIN2_175 stayed KPI-grade).
"""
from __future__ import annotations

import json
import math
import sys
import tempfile
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl", ROOT / "tests", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.delivery import (DELIVERY_GAP_S, DELIVERY_SHORTFALL_MAX,  # noqa: E402
                                DeliveryMonitor, grade_reasons, parse_report,
                                scan_logs, sent_report, stored_record,
                                with_stored_delivery)
from guardrail.manifest import is_kpi_grade                          # noqa: E402

SKIP = "SKIP"


def _report(n, t, beat=False):
    return parse_report(sent_report(n, n, t, beat=beat))


def _stream(mon, t0, t1, n0=0, stalled=(), beats=True):
    """10 Hz publications from t0 to t1; each is sent 5 ms later unless its
    time falls in `stalled` (start, end), when the adapter sends nothing
    (and beats nothing) until `end`, then the last ten in a burst."""
    n = n0
    queue = []
    t = t0
    k = 0
    while t < t1 - 1e-9:
        mon.published(t)
        in_stall = stalled and stalled[0] <= t < stalled[1]
        if in_stall:
            queue.append(t)
        else:
            if queue:                                  # the stall ends: KEEP_LAST 10
                for i, _ in enumerate(queue[-10:]):
                    n += 1
                    mon.report(_report(n, stalled[1] + 0.002 * i), stalled[1] + 0.002 * i + 0.001)
                queue = []
            n += 1
            mon.report(_report(n, t + 0.005), t + 0.006)
            if beats and k % 10 == 0:
                mon.report(_report(n, t + 0.007, beat=True), t + 0.008)
        k += 1
        t = round(t + 0.1, 6)
    return n


# --------------------------------------------------------------------------- #
# The monitor
# --------------------------------------------------------------------------- #

def test_a_healthy_stream_has_no_gap():
    mon = DeliveryMonitor()
    _stream(mon, 100.0, 130.0)
    s = mon.summary(lambda v: v - 100.0)
    assert s["n_gaps"] == 0 and s["delivery_gaps"] == [], s
    assert s["published"] == 300 and s["reports"] == 300 and s["beats"] == 30, s
    assert s["max_wait_s"] < 0.01, s
    assert mon.wait_s(130.0) is None
    assert grade_reasons(s) == []


def test_a_stalled_adapter_is_one_gap_with_its_span():
    mon = DeliveryMonitor()
    # ntFIN2_175's shape: sends stop at 8.0 s and the next one is 33.56 s.
    _stream(mon, 0.0, 40.0, stalled=(8.05, 33.56))
    s = mon.summary()
    assert s["n_gaps"] == 1, s
    g = s["delivery_gaps"][0]
    assert abs(g["t_start"] - 8.1) < 1e-6 and abs(g["t_end"] - 33.56) < 1e-6, g
    assert g["published"] == 255 and g["beats"] == 0 and g["open"] is False, g
    assert abs(s["gaps_s_total"] - 25.46) < 1e-3, s
    why = grade_reasons(s)
    assert len(why) == 2 and "1 gap(s) over 0.5 s" in why[0] and "t 8.1-33.56 s" in why[0], why
    # 245 of the 255 published in it never sent (the last ten went in a burst).
    assert "published 400" in why[1] and "handing 155" in why[1], why


def test_beats_during_a_gap_say_the_adapters_executor_was_running():
    """Setpoints not arriving while the adapter's timer still beats: the
    record tells a subscription that stopped from a process that stopped."""
    mon = DeliveryMonitor()
    for k in range(10):
        mon.published(k * 0.1)
        mon.report(_report(k + 1, k * 0.1 + 0.005), k * 0.1 + 0.006)
    for k in range(10, 40):                            # 3 s: nothing sent ...
        mon.published(k * 0.1)
        if k % 10 == 0:                                # ... but it beats
            mon.report(_report(10, k * 0.1 + 0.05, beat=True), k * 0.1 + 0.06)
    mon.report(_report(11, 4.0), 4.001)
    g = mon.summary()["delivery_gaps"]
    assert len(g) == 1 and g[0]["beats"] == 3 and g[0]["published"] == 30, g


def test_a_stall_still_running_at_the_end_is_an_open_gap():
    mon = DeliveryMonitor()
    mon.published(1.0)
    mon.report(_report(1, 1.005), 1.006)
    for k in range(1, 20):
        mon.published(1.0 + k * 0.1)
    assert abs(mon.wait_s(3.0) - 1.9) < 1e-9
    mon.close(3.0)
    g = mon.summary()["delivery_gaps"]
    assert len(g) == 1 and g[0]["open"] is True and g[0]["t_end"] is None, g
    assert g[0]["published"] == 19 and abs(g[0]["dur_s"] - 1.9) < 1e-9, g
    assert "t 1.1-end s" in grade_reasons(mon.summary())[0]
    # A publication younger than the gap at the end is not one.
    mon2 = DeliveryMonitor()
    mon2.published(1.0)
    mon2.close(1.3)
    assert mon2.n_gaps == 0


def test_an_adapter_that_never_reports_is_a_gap_and_says_so():
    mon = DeliveryMonitor()
    for k in range(20):
        mon.published(k * 0.1)
    mon.close(2.0)
    s = mon.summary()
    assert s["n_gaps"] == 1 and s["reports"] == 0
    assert "never reported a sent setpoint" in grade_reasons(s)[0]


def test_a_report_on_another_clock_is_placed_at_its_receipt():
    """THE CLOCK: a t_sent that is not 0-5 s before its receipt (another
    host's monotonic clock) cannot cover a publication it did not follow."""
    mon = DeliveryMonitor()
    mon.published(10.0)
    mon.report(_report(1, 50_000.0), 10.004)           # far ahead: receipt used
    assert mon.n_receipt_clock == 1 and mon.wait_s(10.1) is None
    mon.published(10.1)
    mon.report(_report(2, 3.0), 10.9)                  # far behind: receipt used
    assert mon.n_receipt_clock == 2
    assert mon.n_gaps == 1 and abs(mon.gaps[0]["dur_s"] - 0.8) < 1e-9, mon.gaps


def test_setpoints_dropped_while_other_sends_continue_are_a_shortfall():
    """SHORTFALL (review, 17 Nov): every other setpoint dropped, the rest sent
    90 ms late. Coverage by any later send sees no gap; the counts do."""
    mon = DeliveryMonitor()
    n = 0
    for k in range(100):
        t = k * 0.1
        mon.published(t)
        if k % 2:
            n += 1
            mon.report(_report(n, t + 0.09), t + 0.091)
    mon.close(10.0)
    s = mon.summary()
    assert s["n_gaps"] == 0 and s["published"] == 100 and s["reports"] == 50, s
    assert s["shortfall"] == 50, s
    why = grade_reasons(s)
    assert len(why) == 1 and why[0].startswith("setpoint delivery shortfall"), why
    assert "published 100" in why[0] and "handing 50" in why[0], why
    ok, _ = is_kpi_grade(_GRADED, {"det_hz": None, "start_heading_err_deg": None,
                                   "setpoint_delivery": s}, dev_waiver="test waiver")
    assert not ok


def test_the_last_setpoints_in_flight_at_close_are_not_a_shortfall():
    mon = DeliveryMonitor()
    for k in range(50):
        mon.published(k * 0.1)
        if k < 48:                                     # the last two not yet answered
            mon.report(_report(k + 1, k * 0.1 + 0.002), k * 0.1 + 0.003)
    mon.close(4.95)
    s = mon.summary()
    assert s["shortfall"] == 2 == DELIVERY_SHORTFALL_MAX and s["n_gaps"] == 0, s
    assert grade_reasons(s) == []
    mon.published(5.0)
    assert grade_reasons(mon.summary())[0].startswith("setpoint delivery shortfall")


def test_a_send_stamped_before_its_publication_time_waits_across_a_hold():
    """Why the node reads the publication's time BEFORE publish(): a send
    stamped before the time the monitor was given does not cover it, and with
    no publication during a hold that setpoint waits for the next send - a
    3 s false gap (review probe, 17 Nov). Stamped before, it is covered."""
    late = DeliveryMonitor()
    late.published(1.0003)                             # time read after publish()
    late.report(_report(1, 1.0001), 1.0002)            # the adapter's send of it
    for k in range(3):                                 # a hold: beats only
        late.report(_report(1, 2.0 + k, beat=True), 2.0 + k + 0.001)
    late.published(4.0)
    late.report(_report(2, 4.002), 4.003)
    assert late.n_gaps == 1 and abs(late.gaps[0]["dur_s"] - 3.0017) < 1e-6, late.gaps
    early = DeliveryMonitor()
    early.published(0.9999)                            # time read before publish()
    early.report(_report(1, 1.0001), 1.0002)
    for k in range(3):
        early.report(_report(1, 2.0 + k, beat=True), 2.0 + k + 0.001)
    early.published(4.0)
    early.report(_report(2, 4.002), 4.003)
    assert early.n_gaps == 0 and early.wait_s(5.0) is None


def test_the_node_stamps_a_publication_before_publishing_it():
    """The Shield node on the stand-in stack: the adapter's send happens
    inside publish() and this thread is preempted before reading its clock
    again; the send still covers the publication."""
    import test_railinputs_node as TR
    N, node, clk, out = TR._build([])
    node.start_mission()
    stamps = []
    orig = node._publish_body

    def publish(a):
        orig(a)
        clk.t += 0.0002                                # the adapter sends it ...
        stamps.append(clk.t)
        clk.t += 0.0005                                # ... before this thread runs on

    node._publish_body = publish
    TR._act(N, node)
    clk.t += 0.1
    node._tick()
    assert len(stamps) == 1 and node.delivery.n_published == 1
    clk.t += 0.001
    node._on_adapter_sent(types.SimpleNamespace(data=sent_report(1, 1, stamps[0])))
    assert node.delivery.wait_s(clk.t) is None, "the send did not cover its own publication"
    node._finish("test over")


def test_junk_on_the_topic_is_counted_not_fatal():
    mon = DeliveryMonitor()
    for junk in ("", "[1]", "{}", '{"n_sent": "x", "t_sent": 1}', "nan"):
        mon.report(parse_report(junk), 1.0)
    assert mon.n_refused == 5 and mon.n_reports == 0
    try:
        DeliveryMonitor(0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("a zero gap was accepted")


# --------------------------------------------------------------------------- #
# The grade
# --------------------------------------------------------------------------- #

_GRADED = {"sim_speedup": 1.0, "topology": "dev", "code_revision": "abc123abc123",
           "vla_model_hash": "sha256:" + "a" * 64, "policy_hash": "sha256:" + "b" * 64}


def test_a_gap_takes_an_otherwise_kpi_grade_run_out_and_nothing_else_moves():
    base = {"det_hz": None, "start_heading_err_deg": None}
    ok, why = is_kpi_grade(_GRADED, base, dev_waiver="test waiver")
    assert ok and why == [], why
    healthy = DeliveryMonitor()
    _stream(healthy, 0.0, 10.0)
    ok, why = is_kpi_grade(_GRADED, {**base, "setpoint_delivery": healthy.summary()},
                           dev_waiver="test waiver")
    assert ok and why == [], why
    stalled = DeliveryMonitor()
    _stream(stalled, 0.0, 10.0, stalled=(3.0, 6.0))
    ok, why = is_kpi_grade(_GRADED, {**base, "setpoint_delivery": stalled.summary()},
                           dev_waiver="test waiver")
    # The stall, and the shortfall it left (30 published in it, the last 10
    # sent when the adapter ran again).
    assert not ok and len(why) == 2 and why[0].startswith("setpoint delivery: 1 gap"), why
    assert why[1].startswith("setpoint delivery shortfall: the Shield published 100 "
                             "setpoints and the MAVLink adapter reported handing 80"), why
    # Not a record: no verdict moves (every run before 2026-11-17).
    for junk in (None, "x", [], {}):
        assert is_kpi_grade(_GRADED, {**base, "setpoint_delivery": junk},
                            dev_waiver="test waiver")[0]


# --------------------------------------------------------------------------- #
# Both nodes on stand-in ROS stacks
# --------------------------------------------------------------------------- #

class _PT:
    """mavros_msgs/PositionTarget with the fields the adapter fills."""

    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.velocity = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.coordinate_frame = self.type_mask = 0
        self.yaw_rate = 0.0


def _rail(stall: tuple[float, float] | None):
    """The Shield and the adapter, back to back on one virtual clock, flown
    6 s at 10 Hz; the adapter's executor does not run during `stall`
    (seconds after mission start), and its setpoint subscription keeps the
    last 10 for when it runs again. Returns (shield node, out dir, adapter)."""
    import test_mavlink_adapter as TA
    import test_railinputs_node as TR
    N, node, clk, out = TR._build([])
    M = TA._adapter_module()
    M.time = clk                                       # one host, one monotonic clock
    ad = M.MavlinkAdapterNode(None)
    M.PositionTarget = _PT
    ad.count_publishers = lambda topic: 1
    ad.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: None))
    yaw = math.radians(90.0)                           # ENU yaw of a north heading
    ad._on_pose(types.SimpleNamespace(pose=types.SimpleNamespace(
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2),
                                          w=math.cos(yaw / 2)))))
    shield_out = node.pubs[N.TOPIC_SETPOINT].sent
    sent_out = ad.pubs[M.TOPIC_SENT].sent
    seen_sp = seen_rep = 0
    queue: list = []
    node.start_mission()
    t0 = clk.t
    last_beat = clk.t
    for _ in range(60):
        TR._act(N, node)
        clk.t += 0.1
        node._tick()
        new, seen_sp = shield_out[seen_sp:], len(shield_out)
        stalled = stall is not None and stall[0] <= clk.t - t0 < stall[1]
        queue = (queue + new)[-10:]                    # KEEP_LAST 10
        if not stalled:
            clk.t += 0.002                             # the DDS hop
            for m in queue:
                ad._on_setpoint(m)
            queue = []
            if clk.t - last_beat >= 1.0:
                ad._beat()
                last_beat = clk.t
        for r in sent_out[seen_rep:]:
            clk.t += 0.001
            node._on_adapter_sent(r)
        seen_rep = len(sent_out)
    node._finish("test over")
    return node, out, ad


def _json(out: Path, name: str) -> dict:
    return json.loads((out / name).read_text(encoding="utf-8"))


def test_a_stalled_adapter_callback_is_a_delivery_gap_and_not_kpi_grade():
    node, out, ad = _rail(stall=(2.0, 5.0))
    sd = _json(out, "metrics.json")["setpoint_delivery"]
    assert sd["topic"] == "/mavlink_adapter/sent" and sd["n_gaps"] == 1, sd
    g = sd["delivery_gaps"][0]
    # The first setpoint the adapter did not take was published at ~2.0 s;
    # the next send came when its executor ran again, at ~5.0 s.
    assert 1.9 < g["t_start"] < 2.1 and 4.9 < g["t_end"] < 5.2, g
    assert 29 <= g["published"] <= 31 and g["beats"] == 0, g
    # Sent: every setpoint outside the stall, and the ten its subscription
    # kept for when the callback ran again.
    assert sd["published"] == 60 and sd["adapter_n_sent"] == ad.n_sent \
        == 60 - g["published"] + 10, (sd, ad.n_sent)
    assert sd["shortfall"] == g["published"] - 10, sd
    kpi = _json(out, "kpi.json")
    assert kpi["kpi_grade"] is False
    assert any(r.startswith("setpoint delivery shortfall") for r in kpi["kpi_grade_reasons"]), \
        kpi["kpi_grade_reasons"]
    assert any(r.startswith("setpoint delivery: 1 gap") for r in kpi["kpi_grade_reasons"]), \
        kpi["kpi_grade_reasons"]
    rows = node.rows
    waits = [r["delivery_wait_ms"] for r in rows]
    assert max(w for w in waits if w is not None) > 2500.0, waits
    # A row carries the count reported before its own setpoint went out.
    assert rows[-1]["adapter_n_sent"] == ad.n_sent - 1, rows[-1]["adapter_n_sent"]


def test_the_same_flight_unstalled_has_no_delivery_gap():
    node, out, ad = _rail(stall=None)
    sd = _json(out, "metrics.json")["setpoint_delivery"]
    assert sd["n_gaps"] == 0 and sd["delivery_gaps"] == [], sd
    assert sd["published"] == sd["reports"] == ad.n_sent == 60 and sd["beats"] >= 5, sd
    assert sd["shortfall"] == 0, sd
    kpi = _json(out, "kpi.json")
    assert not any("setpoint delivery" in r for r in kpi["kpi_grade_reasons"]), \
        kpi["kpi_grade_reasons"]
    # The row is written right after its own publication: that one waits
    # (0 ms on the virtual clock), nothing older does.
    assert all(r["delivery_wait_ms"] == 0.0 for r in node.rows),         [r["delivery_wait_ms"] for r in node.rows]


# --------------------------------------------------------------------------- #
# The offline scan
# --------------------------------------------------------------------------- #

def _write_run(d: Path, stall: tuple[float, float] | None) -> None:
    """A run folder: flight_log rows at 10 Hz (mission time), adapter lines on
    a monotonic clock 5000 s later, each the row's body sent 4 ms on."""
    rows, lines = [], []
    for k in range(80):
        t = round(0.1 * k, 3)
        body = [2.0, 0.01 * (k % 7), 0.0, 0.0]
        rows.append({"t": t, "tick": k + 1, "emitted_body": {
            "vx": body[0], "vy": body[1], "vz_up": body[2], "yaw_rate": body[3]}})
        if stall is None or not stall[0] <= t < stall[1]:
            lines.append({"t": 5000.0 + t + 0.004, "body": body})
    rows.append({"t": 8.0, "tick": 81, "emitted_body": None})
    (d / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                        encoding="utf-8")
    (d / "adapter_log.jsonl").write_text("".join(json.dumps(x) + "\n" for x in lines),
                                         encoding="utf-8")


def test_the_scan_finds_the_gap_in_stored_logs_and_none_in_a_healthy_run():
    import scan_delivery_gaps as S
    with tempfile.TemporaryDirectory() as tmp:
        ok, bad = Path(tmp) / "ok", Path(tmp) / "bad"
        ok.mkdir(), bad.mkdir()
        _write_run(ok, None)
        _write_run(bad, (3.0, 5.0))
        r_ok, r_bad = S.scan_run(ok), S.scan_run(bad)
    assert r_ok["status"] == "ok" and r_ok["published"] == r_ok["sent"] == 80, r_ok
    # Adapter time minus the PLACED publication time of the shortest hop:
    # rows without tick_ms are placed 50 ms before their t.
    assert abs(r_ok["clock_offset_s"] - 5000.054) < 1e-6, r_ok
    assert r_bad["status"] == "gap", r_bad
    g = r_bad["setpoint_delivery"]["delivery_gaps"]
    assert len(g) == 1 and abs(g[0]["t_start"] - 3.0) < 1e-3 and abs(g[0]["t_end"] - 5.0) < 1e-3, g
    assert g[0]["published"] == 21, g                  # 3.0 .. 5.0 s inclusive
    assert r_bad["setpoint_delivery"]["shortfall"] == 20, r_bad
    assert DELIVERY_GAP_S == 0.5


def _write_rows(d: Path, rows: list[dict], lines: list[dict]) -> None:
    (d / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                        encoding="utf-8")
    (d / "adapter_log.jsonl").write_text("".join(json.dumps(x) + "\n" for x in lines),
                                         encoding="utf-8")


def test_the_scan_places_a_slow_ticks_publication_at_its_tick_ms():
    """A tick that took 508 ms published its setpoint 508 ms after the row's
    t; every setpoint was sent (dev run harness_corridor_20261104: the scan
    read a 0.558 s gap that was not there when it placed it at t)."""
    rows, lines = [], []
    t = 0.0
    for k in range(60):
        tick_ms = 508.6 if k == 30 else 4.0
        body = [2.0, 0.01 * (k % 7), 0.0, 0.0]
        rows.append({"t": round(t, 3), "tick": k + 1, "tick_ms": tick_ms,
                     "emitted_body": dict(zip(("vx", "vy", "vz_up", "yaw_rate"), body))})
        lines.append({"t": 7000.0 + t + tick_ms / 1000.0 + 0.0015, "body": body})
        t += 0.1 + (tick_ms - 4.0) / 1000.0            # the next tick waits on this one
    with tempfile.TemporaryDirectory() as tmp:
        _write_rows(Path(tmp), rows, lines)
        r = scan_logs(Path(tmp))
    assert r["status"] == "ok" and r["published"] == r["sent"] == 60, r
    assert r["setpoint_delivery"]["max_wait_s"] < 0.05, r


def test_the_scan_counts_drops_without_a_stall_and_only_body_lines_as_sends():
    rows, lines = [], [{"t": 6999.0, "mission_push": {"items": []}},
                       {"t": 6999.5, "mission_status": {"kind": "mission_upload"}}]
    for k in range(80):
        t = round(0.1 * k, 3)
        body = [2.0, 0.01 * (k % 7), 0.0, 0.0]
        rows.append({"t": t, "tick": k + 1, "tick_ms": 3.0,
                     "emitted_body": dict(zip(("vx", "vy", "vz_up", "yaw_rate"), body))})
        if k % 4:                                      # every fourth one dropped
            lines.append({"t": 7000.0 + t + 0.005, "body": body})
    with tempfile.TemporaryDirectory() as tmp:
        _write_rows(Path(tmp), rows, lines)
        r = scan_logs(Path(tmp))
    assert r["sent"] == 60 and r["published"] == 80, r
    assert r["status"] == "shortfall" and r["setpoint_delivery"]["n_gaps"] == 0, r
    why = grade_reasons(r["setpoint_delivery"])
    assert len(why) == 1 and "shortfall" in why[0] and "derived after the flight" in why[0], why


def test_a_stored_run_is_graded_on_the_record_its_logs_give():
    """WP4-01 (review, 17 Nov): a run flown before the Shield wrote
    `setpoint_delivery` (ntFIN2_175) is graded by the regrading tools on the
    scan of its logs; the Shield's own record wins when there is one; a run
    without an adapter log keeps its verdict."""
    base = {"det_hz": None, "start_heading_err_deg": None}
    with tempfile.TemporaryDirectory() as tmp:
        ok, bad, none = Path(tmp) / "ok", Path(tmp) / "bad", Path(tmp) / "none"
        for d in (ok, bad, none):
            d.mkdir()
        _write_run(ok, None)
        _write_run(bad, (3.0, 5.0))
        (none / "flight_log.jsonl").write_text('{"t": 0.0, "emitted_body": [1, 0, 0, 0]}\n',
                                               encoding="utf-8")
        assert stored_record(ok, base)["n_gaps"] == 0
        assert is_kpi_grade(_GRADED, with_stored_delivery(base, ok), dev_waiver="w")[0]
        g, why = is_kpi_grade(_GRADED, with_stored_delivery(base, bad), dev_waiver="w")
        assert not g and any(w.startswith("setpoint delivery: 1 gap") for w in why), why
        assert stored_record(none, base) is None
        assert with_stored_delivery(base, none) == base
        assert is_kpi_grade(_GRADED, with_stored_delivery(base, none), dev_waiver="w")[0]
        # The Shield's own record (in the metrics, or in the folder's
        # metrics.json) is used as it is, never re-derived.
        own = {"n_gaps": 0, "published": 5, "reports": 5, "gap_s": 0.5}
        assert stored_record(bad, {**base, "setpoint_delivery": own}) is own
        (bad / "metrics.json").write_text(json.dumps({"setpoint_delivery": own}),
                                          encoding="utf-8")
        assert stored_record(bad, base) == own
        assert with_stored_delivery(None, None) == {}


if __name__ == "__main__":
    fns = [v for k, v in list(globals().items()) if k.startswith("test_")]
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
        except (Exception, SystemExit) as e:           # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
