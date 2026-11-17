# Setpoint delivery in the stored hil runs (17 Nov 2026)

What the Shield published against what the MAVLink adapter handed to MAVROS,
for every run folder under the lab checkout's `demo/out/hil_smoke/` that has
an `adapter_log.jsonl` (1,835 folders), plus `demo/out/hil_smoke_02` and
`demo/out/hil_smoke_03` (the first KPI-grade hil run), which live outside
that folder: 1,837 folders in all. Made on 17 Nov 2026 with

    python tools/scan_delivery_gaps.py <lab>/demo/out/hil_smoke_02 <lab>/demo/out/hil_smoke_03 \
        --root <lab>/demo/out/hil_smoke --out <scratch>/hil_full.json \
        --summary-out docs/data/delivery_gaps_hil_smoke_20261117/summary.json

`summary.json` is the tool's compact record (per run: status, setpoints
published and sent, the longest wait, the shortfall, the gaps with their
spans). The scan runs the same `guardrail.delivery.DeliveryMonitor` the
Shield node now runs in flight, through `guardrail.delivery.scan_logs`, over
the two logs a run had before it: `flight_log.jsonl` (each row with an
`emitted_body` published one setpoint) and `adapter_log.jsonl` (each line
with a `body` is one setpoint handed to MAVROS; mission and status lines are
not sends). The two clocks are tied by the setpoints themselves (the same
float32 body on both sides); `scan_logs`' docstring has the rule.

## Result

- **1 run with a delivery gap: ntFIN2_175_dyn-nfz-spawn-ahead**, one gap
  from t 8.099 s (the publication of tick 82: row t 8.094 s + its tick_ms)
  to t 33.57 s (25.47 s). 255 setpoints were published in that span: 245
  never sent (ticks 82-326 of
  `docs/data/replay_from_bag_ntFIN2_20261117/README.md`) and the last ten
  sent in one burst when the adapter's callback ran again (the depth of its
  subscription's queue). Over the run: 686 published, 441 sent, a shortfall
  of 245. It is the only run with a shortfall.
- **1,835 runs with none.** Over them, 718,158 setpoints published and
  718,158 sent (equal in every run); the longest wait of any setpoint for
  the adapter's next send is 0.100 s (median per run 0.002 s), against the
  0.5 s gap.
- Not assessed: `hs11_017_dyn-nfz-spawn-ahead` has an adapter log but no
  flight_log.jsonl. Seven folders under `hil_smoke/` have no adapter log and
  were not scanned (`summary.no_adapter_log`): demoB3_001, ge1110_012,
  ge1110_017, ge1110_030, ge1110_031, hs01_001, hs01_002. None of the seven
  has a kpi.json, so none is counted as KPI-grade anywhere.

So within the stored hil runs this scan covers, the ntFIN2 / ntFIN2s
finding holds: ntFIN2_175 is the only run whose adapter did not hand the
Shield's setpoints to MAVROS. (Of the 598 ntFIN2 / ntFIN2s folders with an
adapter log, 583 and 15, it is one; the replay record's 591 is the set of
runs it replayed, `docs/data/replay_from_bag_ntFIN2_20261117/README.md`.)

## What changed on 17 Nov (review)

The first version of this record (same day, before review) placed every
publication 50 ms before its row's `t` (the tick's start). A slow tick then
read as a gap (dev run `harness_corridor_20261104`: 0.558 s on a 508.6 ms
tick, every setpoint sent), and the published waits carried the margin
(median 0.071 s, max 0.191 s). The scan now places a publication at
`t + tick_ms` less 10 ms (50 ms on rows without tick_ms), and counts as
sends only adapter lines with a `body`. The verdicts did not move (the same
one gap, the same 1,833 clean `hil_smoke/` runs); the waits are now the
publication-to-send waits (median 0.002 s, max 0.100 s).

## Limits

- What is checked is the hop from the Shield to the adapter's publish() on
  `/mavros/setpoint_raw/local`, not what MAVROS wrote to the autopilot.
- A stall reads long by up to the placement margin (10 ms; 50 ms on rows
  without tick_ms) and can read short by up to one tick plus that margin
  (when the previous setpoint's send falls inside the margin before a
  publication). The counts (published, sent) do not depend on the placement.
- Why ntFIN2_175's adapter stopped is not known from these logs: the
  Shield's ticks ran on time throughout (largest spacing 102 ms; in the gap
  each took at most 8.7 ms on a pose about 90 ms old, so its executor, the
  MAVROS-to-Shield path and the run folder's disk were live), the adapter
  logged no drop (441 sent, 0 dropped at shutdown), and no
  event of the run falls at either end of the gap (the dynamic zone's fence
  upload had finished 2.7 s before it began). Either the adapter's
  single-threaded executor did not run or its `/shield/setpoint`
  subscription delivered nothing; the adapter's 1 Hz beat on
  `/mavlink_adapter/sent` (since this date) tells these apart in a future
  run.
- The aircraft: its speed held about 2 m/s for about 3 s into the gap
  (ArduCopter's GUIDED setpoint timeout), then fell, to about zero roughly
  5 s into it (flight_log.jsonl `state_v`).
