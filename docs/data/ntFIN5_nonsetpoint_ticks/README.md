# ntFIN5: the ticks that publish no setpoint (WP3-13)

Computed on 26 Nov 2026 from the flight logs of the KPI-grade nightly ntFIN5
(release 0.7.0 = 21c2771, `hil`, AGX Orin, 654 missions;
docs/data/kpi_ntFIN5/). No flight was made for this record.

    python tools/nonsetpoint_ticks.py demo/out/hil_smoke --prefix ntFIN5_ \
        --out docs/data/ntFIN5_nonsetpoint_ticks/nonsetpoint_ticks_ntFIN5.json

## Why

The tick budget in the KPI report (0 of 183,961 ticks over 100 ms) is read
from `tick_ms` / `e2e_ms` / `e2e_with_wait_ms`, which the Shield node writes
only on a tick that publishes a setpoint (sitl/ros2_shield_node.py
`latency_fields`). The other 119,087 ntFIN5 monitor ticks - the FSM's RTL
and Land ticks and the SET_MODE escalation ticks - carry null there, so the
KPI said nothing about them. Every ntFIN5 row also carries `tick_phases_ms`
(since 2026-11-23: pre_shield, hot_apply, shield, fsm, hold, publish,
post_publish, monotonic_ns stamps from the tick's entry). This record reads
those.

## What is measured

- `to_publish_ms` = pre_shield + hot_apply + shield + fsm + hold + publish:
  the tick's entry to the end of its publish phase. The FSM's mode requests
  are sent inside `hold` (`_request_mode` between the fsm and publish
  stamps), so on a SET_MODE tick this is an upper bound of
  tick entry to the request's publication, the counterpart of `tick_ms`.
- `whole_tick_ms` = to_publish_ms + post_publish: the whole callback,
  including the paced GC and any post-setpoint bookkeeping.

Percentiles are nearest rank, as in tools/kpi_report.py.

## Result

All 654 run logs read; 183,961 setpoint ticks (the KPI report's count) and
119,087 non-setpoint ticks, every one with `tick_phases_ms` (0 without).

| Ticks | n | to_publish_ms p50 / p99 / max | over 100 ms | whole_tick_ms p50 / p99 / max | over 100 ms |
|---|---|---|---|---|---|
| all non-setpoint | 119,087 | 6.85 / 19.59 / 41.70 | 0 | 8.37 / 22.01 / 87.76 | 0 |
| with a SET_MODE request (`set_mode` not null) | 281 | 17.85 / 30.69 / 41.70 | 0 | 21.04 / 33.07 / 43.86 | 0 |
| FSM in RTL after the tick | 107,002 | 6.68 / 18.80 / 30.69 | 0 | 8.02 / 21.37 / 61.04 | 0 |
| FSM in Land after the tick | 6,562 | 8.26 / 21.42 / 41.70 | 0 | 10.15 / 23.94 / 87.76 | 0 |
| no FSM state recorded | 5,523 | 9.11 / 18.81 / 24.61 | 0 | 11.49 / 21.70 / 71.93 | 0 |

The slowest whole tick, 87.76 ms (ntFIN5_390_failsafe-rtl-zone tick 352,
Land), spent 69.6 ms in post_publish, after the tick's decision; its
to_publish_ms was 18.13 ms. The ten slowest rows, with their phases, are in
the JSON (`slowest`).

## What this does not show

- The hops after the node are outside its clock and are not timed: the
  mode request goes to the MAVLink adapter on its topic (or to MAVROS's
  set_mode service when no adapter listens; `_request_mode`), then MAVROS
  and ArduPilot act on it.
- These ticks were flown by the stub pilot with 3-5 rules per policy, as
  all of ntFIN5; the conditions WP3-13 still lacks (a learned VLA on the
  same AGX, a ~50-rule bundle in flight) are not changed by this record.
- `to_publish_ms` ends at the end of the publish phase, not at the exact
  publication of the mode request inside `hold`; it is an upper bound.
