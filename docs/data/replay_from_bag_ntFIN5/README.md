# Replay from the rosbag2 capture: the KPI-grade nightly ntFIN5

`summary.json` is the output of this command, run on 25 Nov 2026 from the lab
checkout at master 7c62c7a (CPU only; rosbags 0.10.11; Python 3.10 env
`vla-real`; 562.2 s, 13:07:09Z to 13:16:32Z):

    python tools/replay_from_bag.py --campaign demo/out/hil_ntFIN5 --out <this folder>/records

`--campaign demo/out/hil_ntFIN5` reads that campaign's sweep.json and takes
every result with a `run` that is not an error: 654 runs, all of them the
folders demo/out/hil_smoke/ntFIN5_001 ... ntFIN5_654 (each with `bag/`).
The flights are at v0.7.0 = 21c2771 (each run's stack_revisions.json:
`code_revision` 21c277170001, companion = Jetson AGX Orin, dirty false).
tools/replay_from_bag.py is byte-identical at 21c2771, 59e19a5 (the sk1122a
shakedown's revision) and 7c62c7a (`git diff` empty), so this is the same
check that produced docs/data/shakedown_20261122/sk1122a_replay_from_bag.txt.

`per_run_records.tar.gz` holds the 654 per-run records the command wrote
(`<run>.replay_from_bag.json`), packed into one file; `summary.json`'s
`per_run` names them. `console.txt` is the command's console output (exit
code 1, which the tool returns whenever any run differs).

What it is: a check of each run's bag against its own flight log, trajectory
and kpi.json. It rebuilds ticks, trajectory and the KPI fields the topics
carry from the bag alone. It is not a re-flight and not a bit-for-bit replay:
the Shield and the FSM are not run again, and 43 of the 61 KPI fields (the P0
escape rate and fail-safe correctness among them) are not re-derivable from
the bag (summary.json `kpi_fields`, "not from the bag"); 18 are compared
(11 from the bag alone, 7 from the bag plus the log's tail or the policy's
rule priorities).

This is also the "bag-based KPI replay" of App. A row ARCH-22: there is no
separate tool or flag; the KPI comparison is the `kpi` layer of the same
replay_from_bag.py run.

## Result over the 654 runs ntFIN5 accepted

630 identical, 24 differ, 0 incomplete, 0 not replayable.

- Layers: ticks 630 identical / 24 differ; trajectory 654 identical; KPI
  630 identical / 24 differ; `manifest_in_bag` (the latched
  /guardrail/manifest start and end frames against manifest.json) 654
  identical; `manifest_in_the_bag` (custom-data metadata) absent on 654
  (off by default).
- The 24 that differ are exactly the 24 gps-noise-beyond-margin runs
  (ntFIN5_589 ... ntFIN5_612), and nothing else. Each has one first
  difference, `ticks.setpoint` at the escalation tick (ticks 33 to 61;
  bag "none", log "brake"), and three KPI fields off by one tick:
  shield_ticks bag = kpi.json - 1, autopilot_ticks bag = kpi.json + 1,
  violation_free_ticks bag = kpi.json - 1 (on all 24). Ticks, repairs,
  interventions, P0 violation ticks, fail-safe expected/permitted ticks are
  equal on these runs too.
- Why (checked on ntFIN5_589, tick 54, with `--write-ticks`): the log row
  says setpoint "brake", failsafe true, FSM edge X8, set_mode RTL, cause
  position lost (3-sigma 3.0 m beyond nfz-square's 1 m margin). The bag holds
  the Shield's zero setpoint at that tick (executed 0, 0, 0, 0), but no
  /shield/intercept message, because no rule was violated. Without an
  intercept, classify() can only call a published setpoint that differs from
  the VLA action "none" (a hold's own leg), so the tick is counted as the
  autopilot's. The bag has no topic that says which side flew a tick; this
  label is inferred. The same 24-run pattern was seen on sk1122a (48/72
  identical, the 24 gps-noise-beyond-margin runs differ).
- Trajectory: 654 identical (303,048 pose points: 301,676 exact, 1,372 the
  next pose message, 0 different).
- 25,546 ticks of 138 runs are past the bag's end (after the Shield's last
  message); those ticks are counted from the flight log and said so.
- KPI fields not compared (not from the bag, all 654 runs): outcome, P0
  escape rate and its status, P0 escapes, fail-safe trigger correctness,
  false-trigger rate, repair success, time to safe, mission success,
  collisions, violations_by_type and the other fields listed in
  summary.json `kpi_fields`.

Compared with ntFIN2 + ntFIN2s (docs/data/replay_from_bag_ntFIN2, 591 runs,
16 Nov at the earlier tool: 571 identical / 18 incomplete / 2 differ; 590 / 1
after the 17 Nov fixes): ntFIN5 has no incomplete runs and no delivery-gap or
action-count differences; the only differences are the 24 inferred
"who flew it" ticks at the GPS-noise escalation.
