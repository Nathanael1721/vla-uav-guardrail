# Body frame on the pymavlink SITL rail, 20 Nov 2026 (WP3-04, ARCH-04)

The flight wf21-bodyframe (CHANGELOG 2026-11-19, wf21) left owed: the
pymavlink SITL rail handing the Shield the body action at the aircraft's
measured heading (ATTITUDE.yaw) and sending the result through
`guardrail.frames.body_to_local_ned` after the projection. Until 19 Nov this
rail used heading 0.

- **Code:** 28a594d (lab checkout, clean). ArduCopter 4.5.7 SITL, built-in
  physics, in WSL; policy bundle `fase3-sim-demo-v0.1.4` (signature
  verified); stub pilot "fly to the northeast pad at 6 m/s"; seed 20261120.
- **Command:** `~/venv-ap/bin/python sitl/run_sitl_demo.py --shield on
  --bundle bundles/fase3-sim-demo-v0.1.4.tar.gz --seed 20261120 --tag <tag>`
  after `sitl/start_sitl.sh`, with `SITL_HOME=-35.363261,149.165230,584,135`
  for the heading-135 boots and `--dynamic` where named.
- **Not KPI-grade:** topology `ardupilot-sitl-pymavlink` (functional
  evidence; KPI-grade runs come from hil).

Each subfolder holds the run's `metrics.json`, `report.md` and
`manifest.json`; the full run folders (flight log, Shield audit, events,
signed replay bundle, trajectory plot, flight-log replay video
`<tag>_track.mp4`) are in the lab checkout's `demo/out/<tag>/`.

## Flights

| Run | Boot heading | Ticks | heading_missing_ticks | Outcome | P0 escape | In zone |
|---|---|---|---|---|---|---|
| sitl_bodyframe_20261120 | -0.4 deg | 271 | 0 | RTL | 0.0 | 0.0 s |
| sitl_bodyframe_h135_20261120 | 135.6 deg | 289 | 0 | RTL | 0.0 | 0.0 s |
| sitl_bodyframe_h135_dyn_20261120 (`--dynamic`) | 135.6 deg | 74 | 0 | RTL | 0.0 | 0.0 s |
| sitl_bodyframe_h135_dyn_prof_20261120 (`--dynamic`, profiled) | 135.6 deg | 295 | 0 | RTL | 0.0 | 0.0 s |

Every run records `action_frame`: `shield_input: body`, `heading_source:
ATTITUDE.yaw`, `adapter: guardrail.frames.body_to_local_ned`, and every tick
of every flight log has `frame: body`.

- **Same decision at both headings.** At heading -0.4 and at 135.6 deg the
  FSM went Normal -> Brake (X1, repair 8.14 m over theta 2.00 m) on tick 1,
  Loiter (G3) on tick 2, RTL on the 30 s Loiter timeout (G5). This is the
  first-tick escalation this demo has flown since 2 Nov (CHANGELOG
  2026-11-02: the target sits behind `nfz-square`, head-on from home), not
  an effect of the body frame.
- **The rotation, checked offline on tick 1 of the heading-135.6 flight:**
  the world action the Shield emitted, (vx, vy) = (2.829, -2.827) m/s, is
  (-4.00, 0.04) m/s in the body frame the log records (`emitted_body`),
  which is that world vector turned by -135.6 deg. That action was never
  flown: the FSM braked on ticks 1-2, so the rail sent a zero stop.
- The two heading runs' first-tick slides go opposite ways round the
  square ((-2.83, 2.83) at heading 0, (2.83, -2.83) at 135.6). The raw
  commands differ in the last digit ((4.243, 4.242) vs (4.242, 4.243) m/s),
  which decides a symmetric tie; both slides are legal and both exceed theta.

## Limits

- The flights test the frame hand-over at a non-zero heading, not the
  adapter in flight. No tick flew the Shield's repaired action: the 2 Brake
  ticks sent a zero stop, then the autopilot held the aircraft and the only
  setpoints were hold transits with no horizontal part. No non-zero
  horizontal setpoint crossed `body_to_local_ned` at 135.6 deg, so the
  rotation is checked only offline on the logged row. `--dynamic` (a zone at
  8 s) does not change that: the static zone escalates first.
- **One loop stall, not reproduced.** On sitl_bodyframe_h135_dyn_20261120 the
  loop wrote no row for 36.6 s, between audit ticks 71 and 72 (the tick the
  dynamic zone was hot-applied; the FSM, in Loiter on the autopilot's hold,
  then timed out at 44.43 s). Offline, `EpisodeRecord.apply_zone` takes
  1-2 ms on Windows and in the WSL venv. In the profiled re-flight
  (sitl_bodyframe_h135_dyn_prof_20261120) it took 0.73 s, almost all of it in
  `write_generation` writing policy_g1.json and csp_g1.json to the run folder
  on `/mnt/d`; that re-flight's longest gap, 0.85 s (audit ticks 79-80), is
  its hot-apply tick. In the stalled flight tick 72 started on time
  (t = 8.034 s) but policy_g1.json was not written until 36.56 s later, so
  the time went by inside the hot-apply tick, before or while that first
  file was written (`link.state()`, `Shield.hot_apply`, or creating
  policy_g1.json on `/mnt/d`), not in an audit-file open. The rail also
  opens its audit file on `/mnt/d` once per tick (295 opens;
  `AuditLogger.log`, open plus encoding, 1.63 s in all, of 434 file opens
  taking 1.76 s across the run). A file-system stall on that disk (the D:
  drive, nearly full and synced) is the likely cause, not proven. The hil
  rail's KPI campaigns write to the AGX's own disk.
