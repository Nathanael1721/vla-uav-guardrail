# AP_DDS stage 1: how closely ArduPilot followed the DDS commands (ap_dds_stage1_02)

Computed on 28 Nov 2026 from the committed records in `ap_dds_stage1_02/`
(`flight_log.jsonl`, `ap_dds_adapter_log.jsonl`). Nothing was re-flown.

## What is compared

- **Commanded**: each `flight_log.jsonl` row's `emitted`, the Shield's output
  in the world frame (North, East, up; m/s). The node published it in the body
  frame on `/shield/setpoint`, and `sitl/ap_dds_adapter_node.py` sent it on
  `/ap/cmd_vel`.
- **Measured**: the row's `state_v` (North, East, up), ArduPilot's EKF
  velocity as MAVROS publishes it (`/mavros/local_position/velocity_local`),
  read at the tick and at most 0.3 s old. In stage 1 the Shield still reads
  state over MAVROS (SERIAL2), so the measurement does not travel the DDS path
  that carried the command.
- **Error**: the Euclidean norm of the 3-D difference |measured - commanded|
  on each tick. Each measurement is paired with the command of the same tick.
  Percentiles use linear interpolation between the closest ranks (numpy's
  default).

## Results

All 113 ticks were flown in GUIDED, each with a fresh `state_v`.

| Window | Ticks | median (m/s) | p95 (m/s) | max (m/s) |
|---|---|---|---|---|
| All ticks, from hover to the target | 113 | 0.056 | 3.430 | 3.977 |
| Cruise, ticks 25-109 (t 2.41-10.81 s) | 85 | 0.049 | 0.101 | 0.172 |

- **Cruise speed**: the command was 4.00 m/s horizontal at compass bearing
  135 deg, south-east, toward the target at North -30, East 30 (about
  North -2.83, East +2.83 m/s; the script's `brg`, a compass bearing from
  North, gives 134.98 to 135.04 deg over the cruise ticks). The pilot asked
  for 6 m/s and the `kin-caps` SpeedClamp capped it to 4 m/s. The measured
  horizontal speed averaged 4.030 m/s (min 3.828, max 4.079), a ratio of
  1.0075 to the command. The bearing error had a median of
  0.12 deg, a p95 of 0.16 deg and a max of 0.17 deg. The vertical error had a
  median of 0.003 m/s and a max of 0.006 m/s.
- **Cruise window**: it starts at the first tick where the measured horizontal
  speed reached 95 % of the command, and it keeps the ticks whose command is
  within 0.05 m/s of the previous tick's. Pairing each measurement with the
  previous tick's command instead gives the same numbers (0.049 / 0.101 /
  0.172), because between ticks 1 and 109 the command never moved more than
  0.017 m/s from one tick to the next.
- **Response from hover**: the measured speed reached 90 % of the command at
  tick 22 (t = 2.11 s) and 95 % at tick 25 (t = 2.41 s).
- **What makes the all-tick p95 and max**: the acceleration from hover
  (ticks 1-24), and the last four ticks (110-113, t 10.91-11.21 s). On those
  four ticks the stub pilot slowed its command near the target (3.78 to
  2.19 m/s), and the mission ended at 11.32 s ("target reached") while the
  aircraft was still at 3.94 m/s. That is the aircraft's response time, not
  error while following a steady command.
- **The adapter hop**: each of the 113 rows matched an adapter line within
  50 ms of its monotonic send time (row `note_flown.t`, adapter `t`); none
  went unmatched. The NED command the adapter packed for `/ap/cmd_vel` was
  within 0.000 m/s (median), 0.002 m/s (p95) and 0.012 m/s (max) of the
  Shield's horizontal command, and within 0.000 m/s of its vertical command.
  The adapter turns the body setpoint at the heading it reads from
  `/ap/pose/filtered`, while the Shield's world-frame record uses the pose it
  read from MAVROS.

Limits: this is one flight on a straight line at a steady 4 m/s, with no wind
(`SIM_WIND_SPD` 0) and no hold or mode change over DDS. It ran in the `dev`
topology, so it is not KPI-grade.

## Computation

Python standard library only. No `follow_check.py` is committed: save the
block below as `follow_check.py` and run it from the repository root with
`python follow_check.py [run dir]`.

```python
import json
import math
import sys
from pathlib import Path

run = Path(sys.argv[1] if len(sys.argv) > 1 else
           "docs/data/ap_dds_20261126/ap_dds_stage1_02")
rows = [json.loads(s) for s in (run / "flight_log.jsonl").read_text("utf-8").splitlines()]
adapter = [json.loads(s) for s in
           (run / "ap_dds_adapter_log.jsonl").read_text("utf-8").splitlines()]


def pct(xs, q):
    """Linear interpolation between closest ranks (numpy's default)."""
    xs = sorted(xs)
    k = (len(xs) - 1) * q / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def stats(xs):
    return {"n": len(xs), "median": round(pct(xs, 50), 3),
            "p95": round(pct(xs, 95), 3), "max": round(max(xs), 3)}


# Commanded = `emitted` (vx North, vy East, vz_up); measured = `state_v`
# (North, East, up). Same-tick pairing.
pairs = []
for r in rows:
    if not r.get("flown") or r.get("state_v") is None:
        continue
    c = (r["emitted"]["vx"], r["emitted"]["vy"], r["emitted"]["vz_up"])
    pairs.append((r["tick"], r["t"], c, tuple(r["state_v"])))

err3 = lambda c, m: math.dist(c, m)                                 # noqa: E731
errh = lambda c, m: math.hypot(c[0] - m[0], c[1] - m[1])            # noqa: E731
hs = lambda v: math.hypot(v[0], v[1])                               # noqa: E731
brg = lambda v: math.degrees(math.atan2(v[1], v[0])) % 360.0        # noqa: E731

print("ticks in log", len(rows), "| flown with a fresh state_v", len(pairs))
print("ALL  |v_meas - v_cmd| 3D", stats([err3(c, m) for _, _, c, m in pairs]))
for f in (0.90, 0.95):
    first = next(p for p in pairs if hs(p[3]) >= f * hs(p[2]))
    print(f"first tick at >= {f:.0%} of commanded h-speed: tick {first[0]}, t = {first[1]} s")

# Cruise: from the first tick at >= 95 % of the commanded horizontal speed,
# while the command stays within 0.05 m/s of the previous tick's.
t95 = next(p for p in pairs if hs(p[3]) >= 0.95 * hs(p[2]))[1]
cruise, prev = [], None
for p in pairs:
    if prev is not None and p[1] >= t95 and math.dist(p[2], prev[2]) < 0.05:
        cruise.append(p)
    prev = p
print(f"CRUISE ticks {cruise[0][0]}-{cruise[-1][0]} (t {cruise[0][1]}-{cruise[-1][1]} s)")
print("CRUISE |v_meas - v_cmd| 3D", stats([err3(c, m) for _, _, c, m in cruise]))
print("CRUISE |v_meas - v_cmd| horizontal", stats([errh(c, m) for _, _, c, m in cruise]))
print("CRUISE |vz_meas - vz_cmd|", stats([abs(c[2] - m[2]) for _, _, c, m in cruise]))
cmd_h = [hs(c) for _, _, c, _ in cruise]
meas_h = [hs(m) for _, _, _, m in cruise]
print("CRUISE commanded h-speed mean", round(sum(cmd_h) / len(cmd_h), 3),
      "| measured h-speed mean", round(sum(meas_h) / len(meas_h), 3),
      "min", round(min(meas_h), 3), "max", round(max(meas_h), 3),
      "| ratio", round(sum(meas_h) / sum(cmd_h), 4))
print("CRUISE |bearing_meas - bearing_cmd| deg",
      stats([abs((brg(m) - brg(c) + 180) % 360 - 180) for _, _, c, m in cruise]))

# The adapter hop: the adapter's packed NED against the Shield's emitted
# command, joined on the monotonic send time (nearest within 50 ms).
d_ne, d_d, unmatched = [], [], 0
for r in rows:
    t = (r.get("note_flown") or {}).get("t")
    if t is None:
        continue
    a = min(adapter, key=lambda x: abs(x["t"] - t))
    if abs(a["t"] - t) > 0.05:
        unmatched += 1
        continue
    n, e, d, _ = a["ned"]
    d_ne.append(math.hypot(n - r["emitted"]["vx"], e - r["emitted"]["vy"]))
    d_d.append(abs(d + r["emitted"]["vz_up"]))
print("ADAPTER matched", len(d_ne), "unmatched", unmatched,
      "| horizontal |ned - emitted|", stats(d_ne), "| vertical", stats(d_d))
```

Output (vla-real env, Python 3.10):

```
ticks in log 113 | flown with a fresh state_v 113
ALL  |v_meas - v_cmd| 3D {'n': 113, 'median': 0.056, 'p95': 3.43, 'max': 3.977}
first tick at >= 90% of commanded h-speed: tick 22, t = 2.11 s
first tick at >= 95% of commanded h-speed: tick 25, t = 2.41 s
CRUISE ticks 25-109 (t 2.41-10.81 s)
CRUISE |v_meas - v_cmd| 3D {'n': 85, 'median': 0.049, 'p95': 0.101, 'max': 0.172}
CRUISE |v_meas - v_cmd| horizontal {'n': 85, 'median': 0.049, 'p95': 0.101, 'max': 0.172}
CRUISE |vz_meas - vz_cmd| {'n': 85, 'median': 0.003, 'p95': 0.006, 'max': 0.006}
CRUISE commanded h-speed mean 4.0 | measured h-speed mean 4.03 min 3.828 max 4.079 | ratio 1.0075
CRUISE |bearing_meas - bearing_cmd| deg {'n': 85, 'median': 0.12, 'p95': 0.162, 'max': 0.171}
ADAPTER matched 113 unmatched 0 | horizontal |ned - emitted| {'n': 113, 'median': 0.0, 'p95': 0.002, 'max': 0.012} | vertical {'n': 113, 'median': 0.0, 'p95': 0.0, 'max': 0.0}
```
