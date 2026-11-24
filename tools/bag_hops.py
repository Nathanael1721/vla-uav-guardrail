"""Cross-node hop times read from each run's rosbag2 (WP3-13).

The Shield node's own `e2e_ms` spans the node only: action received to
setpoint published. The grant's tick budget is end to end, so this tool reads
the hops around the node from the run's bag, where one recorder logged every
message on one clock:

    /vla/action_4d  ->  /shield/setpoint  ->  /mavros/setpoint_raw/local
       (pilot)            (Shield node)          (MAVLink adapter)

Pairing is by receive time, because the messages carry no common id: each
/shield/setpoint is paired with the latest /vla/action_4d before it, and each
/mavros/setpoint_raw/local with the latest /shield/setpoint before it. The
recorder can log a message and the one it caused out of order by a fraction of
a millisecond (seen on hil: the adapter's MAVROS setpoint logged 0.1 ms before
the Shield setpoint it carried), so "before" allows TOLERANCE_MS; a negative
hop in the output is such a reordering, never a message sent before its cause.

LIMIT (found by the 25 Nov review). Receive-time pairing cannot see a hop
longer than the gap to the next input: a Shield setpoint published 150 ms after
its action, with the next action 100 ms after the first, pairs with the next
action and reads 50 ms. So these hops cannot exceed the input period and the
tool cannot fail a 100 ms budget on its own; it corroborates the node's own
per-action `e2e_ms` (matched by action_seq) and adds the adapter hop beside
it. Each run reports the smallest gap between inputs
(`resolves_hops_below_ms`): only a hop shorter than that is read faithfully.

Run where ROS 2 is installed (rosbag2_py), e.g. WSL:

    source /opt/ros/jazzy/setup.bash
    python3 tools/bag_hops.py "demo/out/hil_smoke/ntFIN5_*" --out hops.json

The pairing itself (pair_hops, summarise) is plain Python and tested without
ROS (tests/test_bag_hops.py).
"""
from __future__ import annotations

import argparse
import bisect
import glob
import json
import os
import sys

T_ACTION = "/vla/action_4d"
T_SHIELD = "/shield/setpoint"
T_MAVROS = "/mavros/setpoint_raw/local"
TOLERANCE_MS = 2.0
BUDGET_MS = 100.0


def pair_hops(action_ns, shield_ns, mavros_ns, tol_ms: float = TOLERANCE_MS) -> dict:
    """Hop lists in ms from three lists of receive times in ns (any order)."""
    a, s, m = sorted(action_ns), sorted(shield_ns), sorted(mavros_ns)
    tol = int(tol_ms * 1e6)
    a_s, s_m, a_m = [], [], []
    for ts in s:
        j = bisect.bisect_right(a, ts + tol) - 1
        if j >= 0:
            a_s.append((ts - a[j]) / 1e6)
    for tm in m:
        j = bisect.bisect_right(s, tm + tol) - 1
        if j < 0:
            continue
        s_m.append((tm - s[j]) / 1e6)
        k = bisect.bisect_right(a, s[j] + tol) - 1
        if k >= 0:
            a_m.append((tm - a[k]) / 1e6)
    return {"action_to_shield_ms": a_s, "shield_to_mavros_ms": s_m,
            "action_to_mavros_ms": a_m}


def min_gap_ms(times_ns) -> float | None:
    """The smallest gap between consecutive inputs: the longest hop that
    receive-time pairing reads faithfully (see LIMIT)."""
    t = sorted(times_ns)
    gaps = [(b - a) / 1e6 for a, b in zip(t, t[1:]) if b > a]
    return round(min(gaps), 3) if gaps else None


def _pct(v: list[float], q: float) -> float:
    s = sorted(v)
    if not s:
        return float("nan")
    i = (len(s) - 1) * q / 100.0
    lo, hi = int(i), min(int(i) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (i - lo)


def summarise(v: list[float], budget_ms: float = BUDGET_MS) -> dict:
    if not v:
        return {"n": 0}
    return {"n": len(v), "min_ms": round(min(v), 3), "p50_ms": round(_pct(v, 50), 3),
            "p95_ms": round(_pct(v, 95), 3), "p99_ms": round(_pct(v, 99), 3),
            "max_ms": round(max(v), 3), "over_budget": sum(1 for x in v if x > budget_ms),
            "negative_reordered": sum(1 for x in v if x < 0), "budget_ms": budget_ms}


def read_times(bag_dir: str) -> dict:
    import rosbag2_py                                    # ROS 2 only
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=[T_ACTION, T_SHIELD, T_MAVROS]))
    out = {T_ACTION: [], T_SHIELD: [], T_MAVROS: []}
    while r.has_next():
        topic, _, ts = r.read_next()
        out[topic].append(ts)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", help="glob of run folders, each with bag/")
    ap.add_argument("--out", required=True, help="summary JSON")
    ap.add_argument("--tolerance-ms", type=float, default=TOLERANCE_MS)
    args = ap.parse_args(argv)
    runs = sorted(glob.glob(args.runs))
    pooled = {"action_to_shield_ms": [], "shield_to_mavros_ms": [], "action_to_mavros_ms": []}
    per_run, errors = [], []
    resolves: dict = {}
    for d in runs:
        bag = os.path.join(d, "bag")
        name = os.path.basename(d.rstrip("/"))
        if not os.path.isdir(bag):
            errors.append({"run": name, "error": "no bag/"})
            continue
        try:
            t = read_times(bag)
        except Exception as e:                           # noqa: BLE001
            errors.append({"run": name, "error": repr(e)[:200]})
            continue
        h = pair_hops(t[T_ACTION], t[T_SHIELD], t[T_MAVROS], args.tolerance_ms)
        for k in pooled:
            pooled[k].extend(h[k])
        gap_a, gap_s = min_gap_ms(t[T_ACTION]), min_gap_ms(t[T_SHIELD])
        for g, key in ((gap_a, "action"), (gap_s, "shield_setpoint")):
            if g is not None:
                resolves[key] = g if resolves.get(key) is None else min(resolves[key], g)
        per_run.append({"run": name, "n_action": len(t[T_ACTION]),
                        "n_shield_setpoint": len(t[T_SHIELD]),
                        "n_mavros_setpoint": len(t[T_MAVROS]),
                        "resolves_hops_below_ms": {"action_to_shield": gap_a,
                                                   "shield_to_mavros": gap_s},
                        **{k: summarise(v) for k, v in h.items()}})
    res = {"schema": "vlaguard.bag_hops/1", "runs_glob": args.runs,
           "runs": len(runs), "read": len(per_run), "errors": errors,
           "tolerance_ms": args.tolerance_ms,
           "pairing": "each Shield setpoint with the latest action received before it, "
                      "each MAVROS setpoint with the latest Shield setpoint before it, "
                      f"'before' allowing {args.tolerance_ms} ms of recorder reordering",
           "limit": "receive-time pairing reads a hop faithfully only below the gap to "
                    "the next input; longer hops pair with the next input and read short, "
                    "so over_budget cannot count them. Corroborates the node's per-action "
                    "e2e_ms; it is not an independent test of the budget",
           "resolves_hops_below_ms_min_over_runs": resolves,
           "pooled": {k: summarise(v) for k, v in pooled.items()},
           "per_run": per_run}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1)
    for k, s in res["pooled"].items():
        print(f"{k:22s} " + " ".join(f"{kk}={vv}" for kk, vv in s.items()))
    print(f"{len(per_run)} of {len(runs)} runs read, {len(errors)} errors -> {args.out}")
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
