# Replay from the rosbag2 capture, ntFIN2 + ntFIN2s again (17 Nov 2026)

The same 591 runs as `docs/data/replay_from_bag_ntFIN2/` (16 Nov), checked
again after the replay fixes of 17 Nov. `summary.json` and
`per_run_records.tar.gz` are the output of, run on 17 Nov 2026 from the lab
checkout (rosbags 0.10.11, 510 s, beside another job), written to a scratch
folder and copied here:

    python tools/replay_from_bag.py --campaign demo/out/hil_ntFIN2 --campaign demo/out/hil_ntFIN2s --out docs/data/replay_from_bag_ntFIN2_20261117

Result: **590 identical, 1 differs** (16 Nov: 571 identical, 18 incomplete,
2 differ). Trajectory 591 identical, KPI fields 591 identical.

What the 20 were, and what changed:

- **The 18 incomplete dyn-nfz-spawn-ahead runs: the replay was wrong.**
  After the GeoFence takeover the node's FSM does not step;
  `build_row` then writes none of the FSM keys and the tick's intercept
  carries `fsm: null`. The replay read the absent key as a field the log
  lacked. Such a row now agrees with such an intercept and is counted
  (`fsm_not_stepped_rows`: 4,519 ticks over the 18 runs); a row without the
  keys against an intercept naming a state would still be incomplete.
- **ntFIN2_511_three-simultaneous-violation: the replay was wrong.** Its
  first row ticked at t 0.002 s on action 115, which the recorder logged
  111 ms before t = 0, and its second at t 0.101 s on action 117; no row ran
  on action 116, which the recorder logged 11 ms before t = 0, before the
  Shield's first message. The bag alone placed the first tick on 116 and
  every later tick one action late. The first tick is now the flight log's
  first `action_seq` when it is up to two actions earlier than the bag's own
  placement and its raw action is the bag's action at that count, bit for
  bit; the skipped action is left out and listed
  (`actions_no_tick_ran_on_at_the_start: [116]`), and the record says so.
  On the other 590 runs the log's first action_seq is the bag's own
  placement, so nothing changes there.
- **ntFIN2_175_dyn-nfz-spawn-ahead still differs, correctly: the flight log
  does not record that the Shield's setpoints stopped reaching MAVROS.**
  Ticks 82-326 (t 8.094-32.494 s, 245 ticks) each published a setpoint on
  /shield/setpoint, and the MAVLink adapter sent none of them to MAVROS. The
  bag holds 686 Shield setpoints and 441 /mavros/setpoint_raw/local
  messages. The adapter's own adapter_log.jsonl holds 441 sent setpoints
  and a 25.6 s gap from t 7.994 s. Its shutdown line in mission_console.log
  says 441 sent, 0 dropped, so the adapter's setpoint callback did not run
  for those 25.6 s. Why is not known: the evidence shows only that
  `_on_setpoint` did not run. A subscription that stopped delivering and an
  adapter executor blocked elsewhere would leave the same trace. The logged
  velocity falls to about zero 4-5 s into the gap while the Shield kept
  streaming 2 m/s. The flight log's `emitted_body` records what the Shield
  published, not what reached the autopilot, and no field of the log says
  delivery stopped. The record now lists the span (`executed_gaps`) and the
  adapter's count beside the bag's (`adapter_log`).

  **This is an open rig/safety item, not only a replay note.** The run is
  KPI-grade (kpi.json `kpi_grade` true; scenario expect pass, outcome
  RTL_triggered, P0 escapes 0). Its kpi.json repair and fail-safe figures
  are computed on what the Shield published, including the 245 setpoints
  that never went out. Setpoint delivery is not checked in flight: neither
  sitl/ros2_shield_node.py nor sitl/mavlink_adapter_node.py compares what
  the Shield published with what the adapter sent, or raises when the
  adapter stops sending while the Shield keeps publishing. Until something
  does, a run's KPIs can count commands that never reached the autopilot.

Unchanged limits: 25,044 ticks of 115 runs are past the Shield's last
message (counted from the flight log); no bag carries the episode manifest.
