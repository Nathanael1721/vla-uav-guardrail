"""Setpoint delivery in stored ROS 2 rail runs: what the Shield published
against what the MAVLink adapter handed to MAVROS.

    python tools/scan_delivery_gaps.py demo/out/hil_smoke/ntFIN2_175_dyn-nfz-spawn-ahead
    python tools/scan_delivery_gaps.py --root demo/out/hil_smoke --out scratch/gaps.json \\
        --summary-out docs/data/<dated folder>/summary.json

WHY (2026-11-17). hil run ntFIN2_175_dyn-nfz-spawn-ahead published 245
setpoints the adapter never sent (docs/data/replay_from_bag_ntFIN2_20261117/
README.md). Runs flown since carry the Shield's own record (metrics.json
`setpoint_delivery`, guardrail/delivery.py); runs flown before it carry only
flight_log.jsonl and adapter_log.jsonl, and guardrail.delivery.scan_logs runs
the same DeliveryMonitor over them. This is its command line; the regrading
tools call scan_logs through guardrail.delivery.stored_record.

How the two logs are read, how the clocks are tied, where each publication
is placed and the bounds that placement gives: guardrail.delivery.scan_logs.
In short: a stall reads long by up to the placement margin (10 ms on rows
with tick_ms, 50 ms without) and can read short by up to one tick plus that
margin; the counts (published, sent: the SHORTFALL check) do not depend on
the placement. Only lines with a `body` in adapter_log.jsonl are sends.

`--root` takes every folder under it that has adapter_log.jsonl; folders
without one are listed under `no_adapter_log` in the summary (not scanned).
Times in the output are the flight log's mission seconds (a gap's t_start is
its first publication's estimate: row t + tick_ms, or row t without it).
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail.delivery import (DELIVERY_GAP_S, DELIVERY_SHORTFALL_MAX,  # noqa: E402,F401
                                SCAN_MARGIN_NO_TICK_MS_S, SCAN_MARGIN_S,
                                clock_offset, scan_logs)


def scan_run(run: Path, gap_s: float = DELIVERY_GAP_S) -> dict:
    """One run folder (guardrail.delivery.scan_logs)."""
    return scan_logs(run, gap_s)


def compact(results: list[dict], gap_s: float, no_adapter_log: list[str]) -> dict:
    """The committed form: per run its status, counts, longest wait and gaps;
    over the runs with a clean record, the totals."""
    ok = [r for r in results if r["status"] == "ok"]
    waits = [r["setpoint_delivery"]["max_wait_s"] for r in ok]
    runs = []
    for r in results:
        sd = r.get("setpoint_delivery") or {}
        runs.append({"run": r["run"], "status": r["status"],
                     "published": r.get("published"), "sent": r.get("sent"),
                     "max_wait_s": sd.get("max_wait_s"),
                     "shortfall": sd.get("shortfall"),
                     "delivery_gaps": sd.get("delivery_gaps", [])})
    return {"summary": {
        "gap_s": gap_s, "shortfall_max": DELIVERY_SHORTFALL_MAX,
        "placement_margin_s": {"with_tick_ms": SCAN_MARGIN_S,
                               "without_tick_ms": SCAN_MARGIN_NO_TICK_MS_S},
        "runs": len(results),
        "by_status": dict(Counter(r["status"] for r in results)),
        "runs_with_gaps": [r["run"] for r in results if r["status"] == "gap"],
        "runs_with_shortfall": [r["run"] for r in results
                                if (r.get("setpoint_delivery") or {}).get("shortfall", 0)
                                > DELIVERY_SHORTFALL_MAX],
        "published_ok_runs": sum(r["published"] for r in ok),
        "sent_ok_runs": sum(r["sent"] for r in ok),
        "max_wait_s_ok_runs": ({"median": round(statistics.median(waits), 3),
                                "max": max(waits)} if waits else None),
        "campaigns": dict(sorted(Counter(r["run"].split("_")[0] for r in results).items())),
        "no_adapter_log": no_adapter_log,
    }, "runs": runs}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="*", help="run folders")
    ap.add_argument("--root", action="append", default=[],
                    help="a folder of run folders (each with adapter_log.jsonl)")
    ap.add_argument("--gap-s", type=float, default=DELIVERY_GAP_S)
    ap.add_argument("--out", default=None, help="write the per-run records here (JSON)")
    ap.add_argument("--summary-out", default=None,
                    help="write the compact record (compact()) here (JSON)")
    args = ap.parse_args(argv)
    runs = [Path(r) for r in args.runs]
    no_adapter = []
    for root in args.root:
        for p in sorted(Path(root).iterdir()):
            if not p.is_dir():
                continue
            if (p / "adapter_log.jsonl").is_file():
                runs.append(p)
            else:
                no_adapter.append(p.name)
    results = []
    for i, run in enumerate(runs, 1):
        r = scan_run(run, args.gap_s)
        results.append(r)
        sd = r.get("setpoint_delivery") or {}
        if r["status"] in ("gap", "shortfall"):
            print(f"{r['status'].upper():9s} {r['run']}: published {r.get('published')}, "
                  f"sent {r.get('sent')}; "
                  + "; ".join(f"t {x['t_start']}-{x['t_end']} s, {x['published']} published"
                              for x in sd.get("delivery_gaps", [])[:3]), flush=True)
        if i % 100 == 0:
            print(f"... {i}/{len(runs)}", flush=True)
    doc = compact(results, args.gap_s, no_adapter)
    print(json.dumps({k: v for k, v in doc["summary"].items() if k != "no_adapter_log"},
                     indent=2))
    if no_adapter:
        print(f"not scanned (no adapter_log.jsonl): {', '.join(no_adapter)}")
    if args.out:
        Path(args.out).write_text(json.dumps({"summary": doc["summary"], "runs": results},
                                             indent=1), encoding="utf-8")
    if args.summary_out:
        # LF on every host (the committed record is LF).
        with open(args.summary_out, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(doc, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
