"""Setpoint delivery gaps in stored ROS 2 rail runs: what the Shield published
against what the MAVLink adapter sent.

    python tools/scan_delivery_gaps.py demo/out/hil_smoke/ntFIN2_175_dyn-nfz-spawn-ahead
    python tools/scan_delivery_gaps.py --root demo/out/hil_smoke --out scratch/gaps.json

WHY (2026-11-17). hil run ntFIN2_175_dyn-nfz-spawn-ahead published 245
setpoints the adapter never sent (docs/data/replay_from_bag_ntFIN2_20261117/
README.md). Runs flown since carry the Shield's own record (metrics.json
`setpoint_delivery`, guardrail/delivery.py); runs flown before it carry only
the two logs this tool reads, and it runs the same DeliveryMonitor over them.

WHAT IS READ

  flight_log.jsonl    each row whose `emitted_body` is not null published one
                      setpoint at the row's mission time `t`
  adapter_log.jsonl   each line is one setpoint the adapter sent, at its
                      `t` (the adapter's time.monotonic()), with the `body`
                      it received

The two clocks are tied by the setpoints themselves: a sent `body` equals
the published `emitted_body` bit for bit (both float32), so each sent line
is paired with the published rows of the same body, and the offset that most
pairs share (to 50 ms) is taken at its smallest value (the publication's own
send, the shortest hop). A row's `t` is its tick's time, not the instant of
publication, so publications are placed SCAN_MARGIN_S (50 ms) before
`t + offset`: no setpoint is placed after its own send (on hs04_009 one was,
by 7 ms, and the scan read a gap that was not there), and every wait reads
up to 50 ms long, never short. Sends are at their own `t`, and the monitor
finds every publication no send followed within the gap (default
DELIVERY_GAP_S, 0.5 s). A run with published rows and no adapter line is one
gap over the whole publication.

Times in the output are the flight log's mission seconds.
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail.delivery import DELIVERY_GAP_S, DeliveryMonitor   # noqa: E402

PAIR_BIN_S = 0.05
SCAN_MARGIN_S = 0.05        # publications placed this much early (module docstring)


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
                    out.append(json.loads(line))
                except ValueError:
                    continue
    return out


def clock_offset(published: list[tuple[float, tuple]],
                 sent: list[tuple[float, tuple]]) -> float | None:
    """adapter time - mission time, from setpoints sent with a body the Shield
    published (None when no sent body matches a published one)."""
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
    return min(d for d in diffs if round(d / PAIR_BIN_S) == best)


def scan_run(run: Path, gap_s: float = DELIVERY_GAP_S) -> dict:
    """One run folder: its delivery record, as the Shield node would have
    written it (`setpoint_delivery`), plus how the clocks were tied."""
    rec: dict = {"run": run.name}
    fl, al = run / "flight_log.jsonl", run / "adapter_log.jsonl"
    if not fl.is_file():
        return {**rec, "status": "no flight_log.jsonl"}
    rows = _jsonl(fl)
    published = [(float(r["t"]), _key(r.get("emitted_body"))) for r in rows
                 if r.get("emitted_body") is not None and r.get("t") is not None]
    published = [(t, k) for t, k in published if k is not None]
    sent = []
    if al.is_file():
        sent = [(float(a["t"]), _key(a.get("body"))) for a in _jsonl(al)
                if a.get("t") is not None]
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
    events = ([(t + off - SCAN_MARGIN_S, 0, None) for t, _ in published]
              + [(t, 1, n) for n, (t, _) in enumerate(sent, 1)])
    events.sort(key=lambda e: (e[0], e[1]))
    for t, kind, n in events:
        if kind == 0:
            mon.published(t)
        else:
            mon.report({"n_sent": n, "n_in": -1, "t_sent": t, "beat": False}, t)
    mon.close(published[-1][0] + off)
    s = mon.summary(lambda v: v - off)
    for g in s["delivery_gaps"]:            # a publication: back at its row's t
        g["t_start"] = round(g["t_start"] + SCAN_MARGIN_S, 3)
    return {**rec, "status": "gap" if s["n_gaps"] else "ok",
            "clock_offset_s": round(off, 4), "setpoint_delivery": s}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="*", help="run folders")
    ap.add_argument("--root", action="append", default=[],
                    help="a folder of run folders (each with adapter_log.jsonl)")
    ap.add_argument("--gap-s", type=float, default=DELIVERY_GAP_S)
    ap.add_argument("--out", default=None, help="write the per-run records here (JSON)")
    args = ap.parse_args(argv)
    runs = [Path(r) for r in args.runs]
    for root in args.root:
        runs += sorted(p.parent for p in Path(root).glob("*/adapter_log.jsonl"))
    results = []
    for i, run in enumerate(runs, 1):
        r = scan_run(run, args.gap_s)
        results.append(r)
        if r["status"] == "gap":
            g = r["setpoint_delivery"]["delivery_gaps"]
            print(f"GAP  {r['run']}: {r['setpoint_delivery']['n_gaps']} gap(s) "
                  + "; ".join(f"t {x['t_start']}-{x['t_end']} s, {x['published']} published"
                              for x in g[:3]), flush=True)
        if i % 100 == 0:
            print(f"... {i}/{len(runs)}", flush=True)
    by = Counter(r["status"] for r in results)
    summary = {"gap_s": args.gap_s, "runs": len(results), "by_status": dict(by),
               "runs_with_gaps": [r["run"] for r in results if r["status"] == "gap"]}
    print(json.dumps(summary, indent=2))
    if args.out:
        Path(args.out).write_text(json.dumps({"summary": summary, "runs": results},
                                             indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
