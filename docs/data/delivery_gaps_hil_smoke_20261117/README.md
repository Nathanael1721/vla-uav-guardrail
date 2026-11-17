# Setpoint delivery gaps in the stored hil runs (17 Nov 2026)

What the Shield published against what the MAVLink adapter sent, for every
run folder in the lab checkout's `demo/out/hil_smoke/` that has an
`adapter_log.jsonl` (1,835 folders, 33 campaign prefixes). Made on
17 Nov 2026 with

    python tools/scan_delivery_gaps.py --root demo/out/hil_smoke --out <scratch>/hil_smoke_gaps.json

and compacted into `summary.json` (per run: status, setpoints published and
sent, the longest wait, the gaps with their spans). The scan runs the same
`guardrail.delivery.DeliveryMonitor` the Shield node now runs in flight, over
the two logs a run had before it: `flight_log.jsonl` (each row with an
`emitted_body` published one setpoint) and `adapter_log.jsonl` (each line one
setpoint sent). The two clocks are tied by the setpoints themselves (the same
float32 body on both sides); the tool's docstring has the rule.

## Result

- **1 run with a delivery gap: ntFIN2_175_dyn-nfz-spawn-ahead**, one gap
  from t 8.094 s to t 33.566 s (25.47 s). 256 setpoints were published in
  that span (245 never sent, the ticks 82-326 of
  `docs/data/replay_from_bag_ntFIN2_20261117/README.md`; the last ten went
  out in one burst when the adapter's callback ran again, the depth of its
  subscription's queue, and the 256th is counted by the scan's 50 ms
  placement margin, below).
- **1,833 runs with none.** Over them, 718,154 setpoints published and
  718,154 sent; the longest wait of any setpoint for the adapter's next send
  is 0.191 s (median per run 0.071 s), against the 0.5 s gap.
- 1 folder not scanned: hs11_017_dyn-nfz-spawn-ahead has no flight_log.jsonl.

So the ntFIN2 / ntFIN2s finding (1 in 591) holds across every stored hil
run: it is the only one.

## Limits

- Waits read up to 50 ms long: a row's `t` is its tick's time, not the
  instant of publication, so the scan places each publication 50 ms early
  (SCAN_MARGIN_S) and never after its own send. On healthy runs the reported
  maxima include that margin.
- The adapter log records what the adapter handed to MAVROS
  (`/mavros/setpoint_raw/local`), not what MAVROS wrote to the autopilot.
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
