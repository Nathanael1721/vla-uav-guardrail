"""Time the hil Shield node's ticks that publish no setpoint (WP3-13).

    python tools/nonsetpoint_ticks.py demo/out/hil_smoke --prefix ntFIN5_ \
        --out docs/data/ntFIN5_nonsetpoint_ticks/nonsetpoint_ticks_ntFIN5.json

WHY THIS FILE EXISTS

The tick budget ("<= 100 ms end-to-end", safety-shield.md) is judged in
tools/kpi_report.py on `tick_ms` / `e2e_ms` / `e2e_with_wait_ms`, which the
node (sitl/ros2_shield_node.py latency_fields) writes only on a tick that
publishes a setpoint. A tick that publishes none - e.g. an RTL or Land
tick of the FSM, or a SET_MODE tick that publishes no setpoint - carries
null there, so the KPI's "0 of N over 100 ms" says nothing about them. A
SET_MODE tick that also publishes a setpoint (in ntFIN5 every LOITER and
GUIDED request and some RTL / LAND ones) carries tick_ms and is already
inside the KPI; this tool counts those separately
(`setpoint_set_mode_ticks`) and does not time them again.

Since 2026-11-23 every row also carries `tick_phases_ms`, monotonic_ns stamps of the tick's seven
phases from its entry (pre_shield, hot_apply, shield, fsm, hold, publish,
post_publish). This tool reads those for the rows with no tick_ms.

WHAT IT MEASURES (read-only; flight logs in, one JSON out)

  * `to_publish_ms` = pre_shield + hot_apply + shield + fsm + hold + publish:
    the tick's entry to the end of its publish phase. The FSM's mode
    requests (SET_MODE RTL / LAND, the hold's mode) are sent inside `hold`,
    so for a SET_MODE tick this is an UPPER bound of tick entry to the
    request's publication - the quantity tick_ms is for a setpoint tick.
  * `whole_tick_ms` = to_publish_ms + post_publish: the whole callback,
    the paced GC and any post-setpoint bookkeeping included.

Per group: all non-setpoint ticks; the non-setpoint ticks that sent a
SET_MODE request (row `set_mode` not null); and by the FSM's state after
the tick (RTL, Land, ...). Each: n, p50 / p95 / p99 / max (nearest
rank, as tools/kpi_report.py) and the ticks over 100 ms, with the slowest
rows named. Every SET_MODE tick is also counted by requested mode, split
into setpoint and non-setpoint ticks (`set_mode_ticks_by_mode`), and the
setpoint ones get their `tick_ms` stats (`setpoint_set_mode_tick_ms`), so
the record shows the whole SET_MODE population, not only the part timed
here.

WHAT IT DOES NOT MEASURE

The MAVROS-to-autopilot hop (the mode request leaving MAVROS and ArduPilot
acting on it) is outside the node's clock. A row without tick_phases_ms
(a log from before 2026-11-23) is counted as not measured, never as 0.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

BUDGET_MS = 100.0
TO_PUBLISH = ("pre_shield", "hot_apply", "shield", "fsm", "hold", "publish")


def _pct(vals: list[float], q: float) -> float | None:
    """Nearest rank, the same rule as tools/kpi_report.py _pct."""
    if not vals:
        return None
    s = sorted(vals)
    return round(s[min(len(s) - 1, max(0, math.ceil(q * len(s)) - 1))], 3)


def stats(vals: list[float]) -> dict:
    return {"n": len(vals), "p50": _pct(vals, 0.5), "p95": _pct(vals, 0.95),
            "p99": _pct(vals, 0.99), "max": round(max(vals), 3) if vals else None,
            "over_budget": sum(1 for v in vals if v > BUDGET_MS)}


def phase_times(row: dict) -> tuple[float, float] | None:
    """(to_publish_ms, whole_tick_ms) of a row, or None without phases."""
    ph = row.get("tick_phases_ms")
    if not isinstance(ph, dict):
        return None
    try:
        to_pub = sum(float(ph[k]) for k in TO_PUBLISH)
        whole = to_pub + float(ph["post_publish"])
    except (KeyError, TypeError, ValueError):
        return None
    return round(to_pub, 3), round(whole, 3)


def scan(runs: list[Path], top: int = 10) -> dict:
    setpoint_ticks = 0
    sp_set_mode_ms: list[float] = []
    by_mode: dict[str, dict[str, int]] = defaultdict(
        lambda: {"setpoint": 0, "nonsetpoint": 0})
    no_phases = 0
    bad_lines = 0
    groups: dict[str, dict[str, list[float]]] = defaultdict(
        lambda: {"to_publish_ms": [], "whole_tick_ms": []})
    slow: list[dict] = []
    for run in runs:
        log = run / "flight_log.jsonl"
        if not log.is_file():
            continue
        with log.open(encoding="utf-8") as fh:
            for line in fh:
                try:
                    r = json.loads(line)
                except ValueError:
                    bad_lines += 1
                    continue
                if not isinstance(r, dict) or "tick" not in r:
                    continue
                if r.get("tick_ms") is not None:
                    setpoint_ticks += 1
                    if r.get("set_mode"):
                        by_mode[str(r["set_mode"])]["setpoint"] += 1
                        try:
                            sp_set_mode_ms.append(float(r["tick_ms"]))
                        except (TypeError, ValueError):
                            pass
                    continue
                if r.get("set_mode"):
                    by_mode[str(r["set_mode"])]["nonsetpoint"] += 1
                pt = phase_times(r)
                if pt is None:
                    no_phases += 1
                    continue
                to_pub, whole = pt
                keys = ["all"]
                if r.get("set_mode"):
                    keys.append("set_mode_request")
                keys.append(f"fsm_after:{r.get('fsm_state_after')}")
                for k in keys:
                    groups[k]["to_publish_ms"].append(to_pub)
                    groups[k]["whole_tick_ms"].append(whole)
                slow.append({"run": run.name, "tick": r.get("tick"), "t": r.get("t"),
                             "to_publish_ms": to_pub, "whole_tick_ms": whole,
                             "set_mode": r.get("set_mode"),
                             "fsm_state_after": r.get("fsm_state_after"),
                             "tick_phases_ms": r.get("tick_phases_ms")})
                if len(slow) > 4 * top:
                    slow.sort(key=lambda s: -s["whole_tick_ms"])
                    del slow[top:]
    slow.sort(key=lambda s: -s["whole_tick_ms"])
    return {
        "budget_ms": BUDGET_MS,
        "runs_read": sum(1 for r in runs if (r / "flight_log.jsonl").is_file()),
        "setpoint_ticks": setpoint_ticks,
        "setpoint_set_mode_ticks": sum(m["setpoint"] for m in by_mode.values()),
        "setpoint_set_mode_tick_ms": stats(sp_set_mode_ms),
        "set_mode_ticks_by_mode": {k: dict(v) for k, v in sorted(by_mode.items())},
        "nonsetpoint_ticks_without_phases": no_phases,
        "unparsed_lines": bad_lines,
        "groups": {k: {m: stats(v) for m, v in g.items()}
                   for k, g in sorted(groups.items())},
        "slowest": slow[:top],
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("root", type=Path, help="folder holding the run folders")
    ap.add_argument("--prefix", default="", help="run folder name prefix, e.g. ntFIN5_")
    ap.add_argument("--top", type=int, default=10, help="slowest rows to list")
    ap.add_argument("--out", type=Path, help="write the JSON here (else stdout)")
    a = ap.parse_args(argv)
    runs = sorted(p for p in a.root.iterdir() if p.is_dir() and p.name.startswith(a.prefix))
    res = {"tool": "tools/nonsetpoint_ticks.py", "prefix": a.prefix, **scan(runs, a.top)}
    text = json.dumps(res, indent=1) + "\n"
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(text, encoding="utf-8", newline="\n")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
