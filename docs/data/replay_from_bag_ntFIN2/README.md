# Replay from the rosbag2 capture: the KPI-grade nightly ntFIN2 + ntFIN2s

`summary.json` is the output of, run on 16 Nov 2026 from the lab checkout
(rosbags 0.10.11, 284 s):

    python tools/replay_from_bag.py --campaign demo/out/hil_ntFIN2 --campaign demo/out/hil_ntFIN2s --out docs/data/replay_from_bag_ntFIN2

`per_run_records.tar.gz` holds the 591 per-run records the same command wrote
(`<run>.replay_from_bag.json`), packed into one file to keep the tree small;
`summary.json`'s `per_run` names them.

What it is: a check of each run's bag against its own flight log, trajectory
and kpi.json. It rebuilds ticks, trajectory and the KPI fields the topics carry
from the bag alone. It is not a re-flight and not a bit-for-bit replay: the
Shield and the FSM are not run again, and 43 of the 61 KPI fields (the P0
escape rate and fail-safe correctness among them) are not re-derivable from
the bag (summary.json `kpi_fields`).

Result over the 591 runs the two campaigns accepted (the KPI report counts 589
of them; the two extra are ntFIN2s_001 and ntFIN2s_007, which it supersedes):
571 identical, 18 incomplete, 2 differ.

- incomplete (18, all dyn-nfz-spawn-ahead): their log rows do not record
  `fsm_state_after`, so that field is not compared; nothing else differs.
- ntFIN2_175_dyn-nfz-spawn-ahead differs at tick 82: the log says a setpoint
  was streamed, the bag has no executed setpoint for that tick.
- ntFIN2_511_three-simultaneous-violation differs from tick 1: the bag holds
  one VLA action more than the log (597 against 596; the first actions differ
  in the fourth decimal), so the tick count and the autopilot tick count
  differ by one; trajectory identical.
- Trajectory: 591 identical (285,707 pose points: 282,383 exact, 3,324 the
  next pose message, 0 different).
- 25,045 ticks of 116 runs are past the bag's end (after the Shield's last
  message); those ticks are counted from the flight log and said so.
- No bag carries the episode manifest (`manifest_in_the_bag` absent on all
  591): the switch that writes it into the bag's metadata came after these
  flights.
