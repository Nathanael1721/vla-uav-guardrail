# The escalation state machine: Brake, Loiter, RTL, Land

**Date:** 2026-10-06 (second review round the same day)
**Status:** the pure module is built and tested: `guardrail/fsm.py`, with
`tests/test_fsm.py` at 95/95 on Python 3.10 and 3.11, and 69/69 deliberate
defects caught (see "How the tests were checked"). It also includes a
read-only replay of delivered flight logs (`python -m guardrail.fsm replay`).
**Nothing in the flight path calls it yet.**
This unit did not change `guardrail/shield.py` or `sitl/ros2_shield_node.py`.
No flight has used the FSM, and no KPI number has changed. The integration is
a later wave, and the section "Integration, call site by call site" exists so
that wave is mechanical.
**Cards:** WP3-07 (this module). Groundwork for WP3-06 (theta cap), WP3-08 and
WP1-10 (breach actions), WP1-11 (hard/soft), WP1-12 (priority), and WP3-15
(fail-safe KPI).

**Update 2026-10-09.** The status above is the module's first day. Since
2026-10-07 the FSM runs inside `Shield.filter()` and on every flight rail.
The first follow flight through it on ArduPilot (8 Oct) led to four changes,
described in the next section: X6 waits for a recovery that is shown to be
working and X7 sends home one that is shown to be losing ground, N-in-T
counts at most one onset per second, the FSM's LOITER is flown as
ArduCopter's BRAKE, and StandoffHold reports a depth its ring can have.
Theta stays on each repair's own `magnitude_m`; the one-period reading is
selectable and is a question for the PI. On the labelled library,
fail-safe trigger correctness went from 0.747 to 0.823 on the default basis
(0.903 on the one-period reading), still below the never-trigger null of
0.968. (13 Oct: republished with the grant's forecast, 5 s at 0.1 s, on a
library of 176 episodes, 167 of them labelled where the 12 Oct one scored
186: 0.838, 140 of 167, with 26 false and 1 missed trigger, against a
never-trigger null of 0.940; still below the null and the grant's 0.99. The
null fell because the library changed, not because the FSM improved;
CHANGELOG 2026-10-13. 21 Oct: republished with the FSM's hold points
(docs/DESIGN-hold-points.md): 0.844, 141 of 167, with 25 false and 1 missed
trigger, the null unchanged at 0.940; still below it.) Every `fsm_config_hash` changed with the three new
config fields: the default thresholds now hash to `sha256:5ae67785...`. The
sweep published on 7 Oct carried `sha256:0136098f...`; the 12 Oct and 13 Oct
republications of `docs/data/scenario_sweep.json` carry `sha256:5ae67785...`
on every Shield-on episode (corrected 2026-10-19: this paragraph used to say
the published sweep still carried the old hash). The 8 Oct flight's log, and
its trimmed copy `tests/fixtures/pas_ap_20261008_audit.short.jsonl`, carry
`sha256:0136098f...`, the configuration that flew.

**Update 2026-11-02.** The PI delegated the open decisions as "best practice /
follow the grant". Theta's basis is DECIDED: reading A, each repair's forecast
depth in metres (the default; the one-period reading stays selectable). The
fail-safe rubric is written (docs/RUBRIC-failsafe.md), two Loiter behaviours
were fixed, and the default config hash changed again (two new fields):
`sha256:d55e1bfe...`. The next section has the decisions, the measurements and
what is still wrong.

**Update 2026-11-06.** A ninth extension edge, X8 (Normal / Brake / Loiter ->
RTL): the position the Shield is handed can no longer certify a hard P0
zone's margin. It has two new inputs a rail may report, `State.fix_ok` and
`State.pos_sigma_m`; a rail that reports neither (every rail on that date)
never takes it and flies exactly as before. With a reported accuracy, N-in-T
gains a spatial hysteresis. No FSMConfig field was added, so the default
config still hashes `sha256:d55e1bfe...`. The next section has the design,
where each rail's signal lives, and the measurements.

**Update 2026-11-07.** The ROS 2 Shield node reports `fix_ok`
(`/mavros/global_position/raw/fix`) and its FSM takes X8 from the Shield's
own verdict; `pos_sigma_m` stays unreported there, a policy decision ("Where
each rail's signal lives"). Not flown yet.

**Update 2026-11-07 (rail vehicle).** On the vehicle the hil rail flies (the
harness's measured ArduPilot response, the rig's own fit as the harness's
`rail` arm, and a slower stand-in), a speed cap that is the only rule a
request breaks no longer hides an untrusted repair from G3: the size the
repair chain made beside it is the chain's, made for that kinematic rule
(`FSMConfig.untrusted_owner`, default `chain`; `projected` flies the FSM as it
was). The default config hash changed: `sha256:1e413c8b...`. X6 on a hold
that finds no place to stop is kept, measured. The next section has the
mechanism and the numbers.

**Update 2026-11-07 (hold horizon).** A Loiter's hold point must stay legal
for as long as the Loiter can last (`FSMConfig.hold_horizon` "loiter";
"lookahead" plans as before), Loiter does not hand back a request whose
flight would carry the aircraft onto a spot where a stop is illegal
(`TickInput.handback_unsafe`), and the pymavlink and ROS 2 rails fly a moved
hold (guardrail/hold_flight.py). dyn-nfz-rotate-spin is no longer sent home by
X6 on the lagging vehicles: the rail vehicle's own labels (r2) go from
732/741 to 741/741 nightly, flown longer. The default config hash changed:
`sha256:3b54b4e3...`. docs/DESIGN-hold-points.md, "A hold legal for as
long as it is held, flown by the rails", has the mechanism and the numbers.

**Update 2026-11-07, evening (the per-tick bound).** The FSM is unchanged,
but the plan it asks for (`needs_hold`) may now arrive a few ticks later: the
hold-point search gets `holdpoint.TICK_QUERIES` units of its budget a tick
(`HoldPlanner`, for the Shield's own escalation and both rails), because on
the hil shakedown (hs02) one planning tick took 146.9 ms on the Orin, over
WP3-13's 100 ms. Until the plan arrives the hold is None on entry (held in
place; X6 / X7 judge the aircraft's own position, as for any in-place hold)
or, on a re-plan, an in-place plan given on the search's first tick, and
`needs_hold` stays True. A record of a plan found over more than one tick
carries `hold_search` (`ticks`, `units`; `running` on the ticks before). The
hand-back guard samples at most `holdpoint.HANDBACK_QUERIES` units a tick.
docs/DESIGN-hold-points.md, "The per-tick bound", has the numbers.

**Update 2026-11-07, night (the Loiter held in GUIDED).** The FSM is
unchanged and still asks for LOITER, but the rails no longer send it for a
Loiter at the hold-point search's plan: the companion holds the aircraft in
GUIDED on the plan's point (`guardrail/hold_flight.py`, `--hold-flight
guided`), and ArduCopter's BRAKE is asked for only on a stale pose (until
the next pose) or a gap in the companion's stream (for the rest of the
Loiter). On the hil shakedown hs04 every hand-over to BRAKE
carried the aircraft along GUIDED's last command (1.7 m in 2.2 s), off the
checked point and into the bar's sweep (X6). docs/DESIGN-hold-points.md,
"The Loiter held in GUIDED", has the trace, the fit and the numbers;
docs/DESIGN-ros2-interface.md records the deviation beside "LOITER flown as
BRAKE".

**Update 2026-11-10 (a hand-back judged at speed; one kind of hold work a
tick).** A new law field, `FSMConfig.handback_request`: "at_speed" (the
default) or "as_held" (the FSM as it was at 9eed35b). Under "at_speed"
Loiter's "acceptable" (G8) also needs `TickInput.handback_untrusted` to be
false: the request's repair, judged from the point the Loiter holds at the
speed the request asks for (`Shield.handback_untrusted`, read by G3's own
test, `EscalationFSM.untrusted_repair`), must be one the FSM may trust. On
the KPI-grade smoke sm1110_010 (corridor-drift-angles 8 m/s, -60 deg) the
GUIDED hold sat 0.3 m short of its point, the unchanged request's repair read
1.72-1.78 m there (theta 2.0), and Loiter handed it back 14 times, never
reaching the operator timeout its label expects. A tick that keeps a request
for this reason carries `handback_untrusted: true` on its record. The config
hash changes: the default is `sha256:e0d9267f...`, and `handback_request: as_held`
hashes `sha256:3b54b4e3...`, the default at 9eed35b. The same change bounds
the hold work on the Orin: `holdpoint.TICK_QUERIES` is 60, and a tick pays
for a search slice, or the guard's path samples, or the guard's one decision
for the request, never two of them (`HoldPlanner.handback`); while a plan is
being searched the hand-back is refused (`handback_refused` on the Shield's
record). The FSM's edges are unchanged. docs/DESIGN-hold-points.md, "The
Orin's tick, and a hand-back judged at speed", has the rows, the calibration
and the numbers.

**Update 2026-11-11 (X9, a hold that is not kept).** A tenth extension edge,
X9 (Loiter -> RTL), and a new law field, `FSMConfig.hold_kept`: "reach" (the
default) or "off" (the FSM as it was at 22d434b). Two new inputs,
`TickInput.hold_off_m` / `hold_reach_m`, from `guardrail/hold_flight.py`
HoldWatch, which the ROS 2 node and the harness feed the same way. The
config hash changes: the default is `sha256:b9103b0d...`; `hold_kept: off`
hashes `sha256:e0d9267f...`, the default at 22d434b. The next section has
the miss it answers (the hil smoke's sm1111_048), the design, the
thresholds and the measurements.

**Update 2026-11-12 (X8 on the autopilot's own estimator).** X8's input on
the ROS 2 rail is the AUTOPILOT'S verdict on its position, not the GPS fix
plus a drift constant. A new State field, `estimator_ok` (None, the default:
not reported, and every rail but the ROS 2 node flies as before), which the
node reads from ArduPilot's EKF_STATUS_REPORT. While the autopilot vouches
for its estimate the position is certified, GPS or not; X8 fires when the
autopilot declares it lost (its own EKF failsafe rule). The FSM, its edges
and its config hash are unchanged; the Shield's verdict changed. The next
section has the evidence (731 rig flights), the law and what it does not
cover.

**Update 2026-11-14 (the edges the rails had never taken).** The KPI-grade
hil nightly ntFIN2 took neither the grant's G2, G4 and G6 nor X2 and X3.
G6 had no input on the ROS 2 node (`rtl_failed=False` on every tick); it
has one now, the node's RTL watch, built from what ArduPilot reports. The
others were scenario gaps, not FSM or rail gaps. The FSM, its edges, its
thresholds and its config hash are unchanged, and so is every headless
episode. The next section says why each edge never fired and what flies it.

## 2026-11-14: the edges the rails had never taken

### Why each one never fired on hil

- **G6 (RTL -> Land, "RTL fails (battery / blocked path)").** The FSM reads
  `TickInput.rtl_failed`; the headless harness has fed it from
  `guardrail/rtl_path.py` RTLWatch since 2026-10-12, the ROS 2 Shield node
  passed `False` on every tick. So G6 could not fire on the rig, an RTL that
  made no headway home into a wind beyond authority stayed an RTL ("Not
  done" of 2026-11-11), and `rail_refusal` refused the two cells that take
  G6 (failsafe-rtl-blocked, a gate on `rtl_failed_ticks`;
  failsafe-low-battery, the battery hook).
- **X2 / X3 (Brake / Loiter -> RTL / Land on a direct rule).** No library
  cell violates an RTL or land rule while the FSM is in Brake or Loiter:
  every one (failsafe-rtl-zone, -land-zone, -rtl-blocked, -low-battery) is
  violated from Normal, which is G11 / G12. They never fired headless
  either (the 4 Nov published sweep: no X2, no X3).
- **G2 / G4 (N-in-T).** N-in-T counts hard-violation ONSETS, at most one per
  `onset_clear_s` (1 s), three within T (5 s). The rail ticks at the
  pilot's 10 Hz (the stub, `--tick-trigger action`), so (N - 1) x 1 s = 2 s
  fits in T and the edges are reachable there: not a tick or decision-period
  gap. In the published headless sweep (4 Nov, ideal vehicle) G2 fires on
  nine cells and G4 on eight: G4 only through GPS noise
  (gps-noise-beyond-margin; `gps_noise_m` is refused on the rails), G2
  through GPS noise or a noise-free forecast re-touching a ring on the
  ideal vehicle (near-edge-at-12mps, corridor-drift-angles at -60 deg) or a
  bar sweeping across it (dyn-nfz-rotate-spin). The nightly profile flies
  those families and ntFIN2, on the rig's own response, took no G2. A steady
  pilot makes one onset and holds it. (For a pilot slower than 1.1 s a decision, N-in-T needs
  `FSMConfig.for_decision_period`. Since 2026-11-17 the node can apply it
  when the pilot's period is declared (`--decision-period-s`, above
  T / (1.5 x N), 1.11 s for the grant's N and T), but only a hand-launched
  node takes that option (`sitl/run_ros2_demo.sh` EXTRA): no campaign path
  passes it, and no flight has applied it. The stub and the VLA at 1 s are
  not affected: T stays 5 s.)

### G6's input on the ROS 2 node: the RTL watch

`sitl/ros2_shield_node.py` RTLFailureWatch, judged on every tick the FSM
is in RTL (reset on every other tick); the module docstring, THE RTL WATCH,
has the detail. The RTL fails on the first tick on which one of these holds:

| reason | test | threshold, and why |
|---|---|---|
| blocked path | `rtl_path.check_leg` on the Shield's hard, enforced P0 zones and band in force now | the harness's own test, from the FSM's RTL on (the leg is where ArduCopter's RTL will fly, whatever the mode reads yet) |
| no progress home | `rtl_path.ProgressMonitor` on the climb plus the distance home, only while `/mavros/state` reads RTL and the aircraft is outside RTL_ALT / RTL_CONE_SLOPE of home (5 m by default) | the harness's 0.5 m in 5 s (T, X6's 0.1 m/s); waiting for the autopilot's own RTL keeps the mode change and the brake out of the window, and inside the cone ArduCopter cuts the climb and loiters and lands over home - an RTL that is ending, never judged |
| battery | ArduPilot's BATTERY_STATUS `charge_state` CRITICAL or worse, fresh within 5 s | ArduPilot's own failsafe verdict (BATT_CRT_VOLT / BATT_CRT_MAH); LOW is its reason to start an RTL, not to abandon one |
| RTL not entered | `/mavros/state` has not read RTL 5 s after the FSM entered it | five 1 Hz heartbeats: a refused mode change (no position, no home) never confirms |

Every threshold errs toward not failing a return; an error while reading
the rules is recorded and fails nothing. RTL_ALT, RTL_CONE_SLOPE and the
BATT_* failsafe parameters are read at take-off and recorded. Rows carry
`rtl_failed` / `rtl_confirmed` on every judged RTL tick, and
`sweep_scenarios.sitl_extra` counts `rtl_failed_ticks` from them when the
node's metrics carry `rtl_watch`, so `rail_refusal` no longer refuses a
gate on it: failsafe-rtl-blocked flies. failsafe-low-battery stayed
refused until 2026-11-17: the node reads the battery, but SITL's battery
reaches CRITICAL only through the firmware's BATT_CRT_* (0, disabled, by
default), which the simulator side, setting SIM_* parameters only, never
set. Since then the simulator side gives the SIMULATED vehicle, for that
cell alone, a battery fail-safe setup: BATT_CRT_VOLT 10.1, the 10.0 V drop
and no low or critical action (BATT_FS_LOW_ACT and BATT_FS_CRT_ACT 0, so
the FSM and not ArduPilot acts) as in ArduCopter's autotest
BatteryFailsafe; BATT_LOW_VOLT 10.5, the copter default; BATT_LOW_TIMER
1 s, the cell's own choice (a lab vehicle-setup choice of 2026-11-17; the
default is 10 s, which the autotest leaves). It drops SIM_BATT_VOLTAGE from
12.6 V to 10.0 V BATT_LOW_TIMER before the cell's time, and restores every
value after the mission (`sitl/sitl_stressors.py`, THE BATTERY CELL'S
FAIL-SAFE SETUP: only over a loopback link, only to an autopilot that
answers SIM_BATT_VOLTAGE). `rail_stress_check` scores the flight only if
ArduPilot's CRITICAL reached the node's RTL watch within 1.5 s of the cell's
time with the FSM in RTL on the node's last row before it; otherwise the
flight errors (the stress not realised), never a failed G6 gate. The cell
is opt-in on the rails (`RAIL_BATTERY_OPT_IN`): only a profile with
`rail_battery: true` flies it, fsm_edges alone, so the smoke, nightly and
broad campaigns, which list the cell, still refuse it. Not yet flown; the
KPI profiles take it only after a shakedown flight shows it realised.

### What flies them: experiments/profiles/fsm_edges.yaml

Five cells, seed 1001 (six since 2026-11-17: failsafe-low-battery, G11 G6
G10 on both vehicles); three new families flown by this profile alone
(`profiles: [fsm_edges]`: no library, smoke, nightly or broad number
moves), measured headless on the ideal and the rail vehicle
(RAIL_VEHICLE):

| cell | ideal | rail vehicle |
|---|---|---|
| failsafe-rtl-from-hold (nfz-head-on, an RTL zone over nfz-square at 8 s) | G1 X2 G9 | G1 G3 X2 G9 |
| failsafe-land-from-hold (the same, `land`) | G1 X3 G10 | G1 G3 X3 G10 |
| fsm-n-in-t-flicker (a `brake` zone 16 m ahead, switched off 0.9 s / on every 1.4 s) | X1 G2 G4 G9 | X1 G2 G4 G9 |
| failsafe-rtl-blocked | G11 G6 G10 | G11 G6 G10 |
| wind-beyond-authority, lane 15, 12 m/s | G1 G3 X9 G6 | G1 G3 X7 G6 |

The flicker's zone acts `brake`, so a switch-on is answered by the rule's
own stop and never by an untrusted repair (which would take G3 first);
onsets 1.4 s apart survive 0.4 s of tick jitter against the 1 s rate limit,
and each 0.9 s switch-off is under T_recover, so neither Brake nor Loiter
clears between them. It carries NO fail-safe label: the rubric's N1 expects
none for a planner every request of which is answered by a stop or a
trusted repair, and the grant's G4 sends home a Loiter whose N-in-T persists.
Which should govern is a question for the PI; until then the cell is the
edges' flight, not a KPI cell, and the profile says `grant_kpi_bearing:
false`.

(Since 2026-11-15 the cell is labelled: a fail-safe expected, by the rubric's
new clause E6, "N-in-T persists (the grant's G4)" - hard-violation onsets
made by the world, recurring at the N-in-T rate through a Loiter. Decided
2026-11-15 by the lab under the PI's instruction of 6 Oct (follow the grant;
best practice where it is silent); the PI's confirmation is still to be
recorded. The label followed the flight hs10_003 of 14 November; the grant's
text is the basis. docs/RUBRIC-failsafe.md, sections 2 and 9. The profile is
still `grant_kpi_bearing: false`.)

### Not done

- Not flown: every number above is headless. On hil: `--profile fsm_edges
  --backend hil` (or `--only failsafe-rtl-from-hold failsafe-land-from-hold
  fsm-n-in-t-flicker failsafe-rtl-blocked wind-beyond-authority` under it).
- The pymavlink rail (`sitl/run_sitl_demo.py`) still passes no `rtl_failed`.
- The battery reason has never fired: no rig flight has had a critical
  threshold.

## 2026-11-12: X8 on the autopilot's own estimator

### The false triggers

The KPI-grade hil nightly nt1111 scores (r2) 555/567 = 0.979. X9 removes 6
of the 12 errors (561/567). The other 6 are the FSM's X8 on gps-dropout
lane 23.6, windows 4-11 s and 6-13 s (nt1111_568-573, X8 at 9.3-10.4 s). The
same edge fired in rh03_040, sm1110_044 and sm1111_044. Under the rail
labels (docs/RUBRIC-failsafe.md section 8, "2026-11-12") no fail-safe is
expected there: SITL's truth puts EKF3's estimate 0.13-0.78 m off the
aircraft when X8 fired, under the 1 m margin.

X8 fired because of the law, not because of the estimate. On a rail with no
fix the Shield dead-reckoned on its own commands and took the gap from the
handed pose, plus `dr_drift_mps` (0.5 m/s) × the time since the loss, as
the pose's error. On this rail the handed pose is EKF3's live estimate. So
the gap measured the Shield's command model against EKF3 (3.0-3.5 m: the rig
flies the Brake's repair and its release late), not the pose's error, and X8
read the pose as "4.4-5.0 m off". Even with a perfect reference the 0.5 m/s
allowance alone fires. The measured EKF3 drift (0.23 m/s) clears the closest
trajectory by 0.01 m, so tuning that constant would be a knife edge.

### What the autopilot reports (the rig's raw MAVLink)

Every run's rosbag holds the autopilot's raw MAVLink (/uas1/mavlink_source).
That includes EKF_STATUS_REPORT (193, at the node's 10 Hz stream rate), with
flags and the four "variances". The variances are the square roots of EKF3's
innovation test ratios. The bags also hold GPS_RAW_INT, SITL's truth
(SIMSTATE) and EKF3's estimate (GLOBAL_POSITION_INT). Read from 731 flights
(nt1111, sm1111, sm1110, rh03):

| | 701 flights without a GPS denial | 30 GPS denials (24 of 7 s, 6 of 2 s) |
|---|---|---|
| flags while armed | 0x33f on every report (attitude, horizontal/vertical velocity, relative and absolute horizontal position, vertical position, both predicted positions) | 0x33f from the loss until 6.80-6.92 s after GPS_RAW_INT's loss; then 0x25 (no horizontal velocity or position) for 1-2 reports, 0.10-0.21 s, in 23 of the 24 7 s denials, until the GPS returned at 7.0 s (nt1111_570, whose GPS_RAW_INT showed 6.8 s without a fix: none) |
| pos_horiz / velocity test ratio | at most 0.049 / 0.080 | at most 0.013 / 0.023 inside the denials: frozen at their last fused value (without a measurement EKF3 computes no innovation) |
| unhealthy for the autopilot's own failsafe rule (1 s) | never (no report unhealthy) | never (longest spell 0.21 s) |
| ArduPilot EKF failsafe | none possible | none: no LAND, no EKF STATUSTEXT |
| report gaps while armed | at most 0.62 s (one flight; all others 0.22 s or less) | at most 0.16 s |
| EKF3's true error (SIMSTATE against GLOBAL_POSITION_INT) | at most 0.31 m (76 of them read) | after GPS_RAW_INT's loss 0.09-0.31 m at 2 s, 0.20-0.67 m at 4 s, 0.37-1.22 m at 6 s; at most 1.47 m before the GPS returned, and above the 1 m margin on 5 of the 24 7 s denials (peaks 1.06-1.47 m: lane 15 and lane 23.6 at 2-9 s, the aircraft 8.4 m or more from nfz-square) |

What this says:

- EKF3 keeps its absolute horizontal position through a denial for
  posRetryTimeNoVel_ms (AP_NavEKF3.h: 7 s from its last GPS fusion). It
  drops it at that timer and at nothing else.
- The flags are clean: binary, the same on every flight, never near a
  threshold.
- The variances say nothing during a denial: no measurement, no innovation,
  so they stay frozen.
- ArduCopter declares its position bad (ekf_check: 10 failing checks at
  10 Hz, 1 s, then FS_EKF_ACTION, 1 = Land on the rig) only after the timer.
  So a 7 s denial ends in a race the GPS won every time, by 0.79 s or more
  against the 1 s rule.
- The separation the report makes is the autopilot's: "within my
  dead-reckoning allowance" against "beyond it". It is not a bound on the
  error. EKF3 vouched for its estimate while it was more than 1 m off on 5
  denials (1.04-1.47 m at their peaks while vouched for).

### The law

`Shield._position_verdict` reads `State.estimator_ok` first, when a rail
reports it:

- **True** (the autopilot vouches for its estimate): the position is
  certified - it is the estimate the autopilot navigates, returns and fences
  on, valid by the autopilot's own test - also while the GPS fix is lost.
  The Shield holds no dead reckoning of its own against it, and
  `dr_drift_mps` is not read. The certified position anchors the dead
  reckoning, should the certification lapse.
- **False** (the autopilot declares its absolute horizontal position lost):
  the error is unbounded and the position certifies no margin. X8 fires on
  that tick if any hard, enforced P0 zone is in force at the altitude,
  naming the nearest. This happens with or without a GPS fix: a position
  the autopilot itself disowns is not certified by a fix.
- **None** (not reported, or the report has gone stale): `fix_ok` decides
  as before (2026-11-06). With the fix lost, the error is dead-reckoned on
  the commands from the last certified position, plus `dr_drift_mps` × its
  age. This is the law of the headless harness's frozen fix, of every rail
  that reports no estimator, and of the ROS 2 node once its EKF reports
  stop.

On the ROS 2 node (`sitl/ros2_shield_node.py`, module docstring "THE
AUTOPILOT'S ESTIMATOR"; pure helpers `ekf_status_from_mavlink`,
`ekf_position_healthy`, `EstimatorHealth`, unit-tested):

- **Source.** No MAVROS 2 plugin publishes EKF_STATUS_REPORT: sys_status
  publishes ESTIMATOR_STATUS (230), which ArduPilot does not send, and
  mavros_extras has no handler. So the node subscribes MAVROS's raw feed to
  the UAS, `/uas1/mavlink_source` (mavros_msgs/Mavlink, about 300 messages a
  second, best effort, queue 100). It keeps message 193 from system 1,
  component 1. sitl/sros2_policy.xml grants the subscription; a keystore
  signed before this change refuses it (re-sign with `tools/sros2_setup.sh
  --pki <lab PKI> --permissions-only`). A refused or absent subscription
  leaves `estimator_ok` None: the node flies on `fix_ok` as before.
- **A healthy report** has EKF_POS_HORIZ_ABS set and EKF_CONST_POS_MODE
  clear (what ArduCopter navigates on: `ekf_has_absolute_position`, armed).
  It also needs EKF_UNINITIALIZED clear, and the horizontal-position and
  velocity test ratios finite and below FS_EKF_THRESH (read from the
  autopilot at take-off; 0.8 when unread or 0). That is stricter than
  ArduCopter's `ekf_over_threshold`, which needs two of compass, velocity
  and position, or velocity at twice the threshold. EKF_GPS_GLITCHING is
  recorded and not read: the autopilot flies through it, and its failsafe
  does not count it.
- **The verdict** (`EstimatorHealth`) follows ArduCopter's ekf_check, in
  time instead of checks. An unhealthy report adds the time since the
  previous one to an account (capped at 1 s); a healthy one takes it back.
  The verdict is False from the report that brings the account to 1 s
  (`EKF_FAILSAFE_S`) until it is back at 0. It is None with no report, or
  with none for longer than the pose timeout (`EKF_STALE_S`, 1 s).
- **Records.** Rows carry `estimator_ok`, `ekf_flags`, `ekf_pos_var`,
  `ekf_vel_var` and `ekf_age_ms` once a report has arrived. metrics.json has
  `fix_quality.estimator` (spells on the mission clock; spells before t = 0
  are the EKF's initialisation before take-off). events.jsonl has
  `ekf_failsafe` (FS_EKF_THRESH and FS_EKF_ACTION as read), and report.md
  has a row.

**Stale is "not reported", not "lost".** The brief for this change read a
stale report as unhealthy. That would put an X8 on any flight whose MAVLink
feed paused for 1 s, GPS or not, and the old law already covers the case
conservatively. A report that stops during a denial falls back on the dead
reckoning from the last certified position, which fires near a zone within
seconds.

**Headless.** The harness's dead-reckoned denial (sweep_scenarios
GPS_DROPOUT_MODELS "dead_reckoned", the rail vehicle's) hands the Shield the
modelled autopilot's verdict with its estimate: True until
`RAIL_EKF_POS_VALID_S` (6.8 s + the 1 s rule) after the loss, False after
it. No library window is longer than 7 s, so no cell reaches False. The
frozen fix reports none, so the templates' cells fly as before.

### What it does not cover

- **The margin must cover the autopilot's allowance.** While the autopilot
  vouches for its estimate, that estimate's error is the autopilot's. No
  Shield action flies on a better one: RTL returns on the same estimate,
  and the autopilot's fence uses it too. On SITL that error reached 1.47 m in
  6.9 s, beyond the shipped 1 m margins. A hard P0 margin certifies its zone
  through a denial only if it covers the autopilot's dead-reckoning error
  over its own allowance (about 1.5 m on SITL; a real IMU drifts faster and
  must be measured). This is the same question as "Once a rail reports its
  accuracy, a margin is an accuracy requirement" above: a policy decision,
  and no policy was changed.
- **E3 on the rig.** Read on SITL's truth, E3 held on none of the rig's
  denials (rubric section 8). A cell that brings the aircraft within the
  autopilot's error of a boundary in the last seconds of a denial would
  expect a fail-safe that this law does not take until the autopilot's own
  failsafe does. The library has no such cell.
- **An unhealthy estimate with the GPS fix valid** (a GPS glitch EKF3
  rejects, a velocity test ratio past the threshold) is now X8. No rig
  flight showed one.
- **Not flown.** Every number above is a replay of flights flown on the
  old law (next subsection). The subscription's cost on the Orin (about 300
  callbacks a second, each returning on the message id) is not measured.
- **A deviation question for the PI.** X8 is itself an extension edge. This
  change narrows when it fires on the ROS 2 rail, from "the Shield's own
  model says the pose may be off" to "the autopilot says its position is
  lost". That moves the E3 margin question from the FSM to the policy's
  margins.

### Replayed on the rig's flights

`EstimatorHealth`, imported from this node, was fed every EKF_STATUS_REPORT
of the 731 bags at its recorded receipt. Its verdict was read every 0.05 s
over each armed span (899,299 samples) and at every logged tick whose fix
was lost:

- False 0 times, None 0 times.
- On every one of the 1,798 fix-lost ticks of the 30 denials it was True.
- So no flight can take X8 under the law, and the 9 logged X8s (nt1111_568-
  573, rh03_040, sm1110_044, sm1111_044) go.
- The law can only remove an X8 on these flights: a False or a None never
  occurred while armed.

What the flights do after the X8 they no longer take was flown headless.
The rail vehicle (`--response rail`, moved holds, the dead-reckoned
denial), FSM on, nightly seeds 1001-1003, all 24 gps-dropout cells:

| cells | new law | old law (`estimator_ok` withheld) | rig (nt1111) |
|---|---|---|---|
| lane 23.6, 4-11 s | G1 5.4, G7 8.6, no fail-safe, closest 2.85 m | X8 10.3 s, home | X8 10.2-10.4 s |
| lane 23.6, 6-13 s | G1 5.4, G7 8.6, no fail-safe, closest 2.85 m | X8 9.9 s, home | X8 9.3-9.5 s |
| lane 23.6, 2-4 s and 2-9 s | G1, G7, no fail-safe | the same | G1, G7, no fail-safe (562-567) |
| lane 15, every window | G1, G3, G5 at 35.8-36.1 s | the same | G1, G3, G5 at 35.8-36.0 s |

The new law flies 568-573 as the rig flew their 2-9 s neighbours (565-567):
correct non-triggers under the (r2) labels. Lane 15 keeps its G5 (expected
by E5). Predicted nt1111 (r2), with X9: 567/567 = 1.000 (Wilson 95 % low
0.993). This is not measured: the nightly must be flown on this code.

## 2026-11-11: X9, a hold that is not kept

### The miss

The KPI-grade hil smoke sm1111 (22d434b) has one missed fail-safe under the
adopted reading (r2: the rail vehicle's labels and E5), 44/45: sm1111_048,
wind-beyond-authority lane 15 at 12 m/s, labelled expected by the rubric's
E2 (docs/RUBRIC-failsafe.md). What the flight log says, tick by tick:

| t (s) | what | position (x, y) | ground velocity |
|---|---|---|---|
| 0-5.3 | Normal, the 4 m/s request flown in the 12 m/s tailwind | -30 to -13.7, 15.0 | 3.3-3.9 m/s along the lane |
| 5.3 | G1 (nfz-square repaired, 0.29 m) | -13.7, 15.0 | 3.87 |
| 5.8 | G3 (repair over theta, 2.23 m); the companion holds (-11.8, 15.0) in GUIDED | -11.8, 15.0 | 3.76 |
| 5.9-15.3 | Loiter. The hold commands up to 4 m/s back to its point (the adapter's NED setpoints: -0.7 to -3.9 m/s north) | carried to (9.6, -3.2): 28 m from the point, 2.4 m from nfz-square's face at 13.1 s, never inside | 2.0-4.6 m/s, receding from the point at 2-4 m/s |
| 15.3 | G8: the request clean (past the zone) and the position legal for T_recover | 9.6, -3.2 | 4.6 |
| 15.3-31.8 | Normal to the goal, downwind | | |

The SITL airframe makes 3.3-3.9 m/s over the ground with the tailwind but
cannot hold still in it: holding needs 12 m/s of airspeed, and asked for
4 m/s upwind it made 2 m/s downwind, about 10 m/s of airspeed. No edge
could see it. X6 and X7 judge an illegal position, and this one stayed
legal (the hold was carried sideways round the zone, an ArduPilot response
to a saturated position loop the harness's model does not have: the
headless hold drifts straight down the lane into the zone, where X7 sees
it). N-in-T and the operator timeout need a request that stays bad, and
this one turned clean past the zone. TransitWatch (guardrail/hold_flight.py)
judges only a MOVED hold's transit, and only gives it up. The FSM was
never told whether the hold was being kept.

The 8 m/s cell (sm1111_047) held: it overshot its point by 3.6 m in 1.8 s,
was back on it 6 s later, and the operator timeout sent it home (G5, 35.8 s;
correct by E2). Within authority (5 m/s, sm1111_045 / 046) the hold was
kept.

### The edge

X9, Loiter -> RTL: the aircraft is farther from the point its hold flies to
than a kept hold can carry it, on every tick for T_recover, and that
distance is still growing - X7's own test (a least-squares slope above
`x6_min_escape_mps` at two standard errors, and a fitted rise of at least
`x7_min_rise_m`) on the hold's distance instead of the illegal depth. It
comes after G4 (the grant's own Loiter -> RTL) and before G8: handing GUIDED
back would give the flight to stops - Brake's zero, the next Loiter - that
the aircraft has just been shown not to hold. RTL, because it is the
grant's next rung after Loiter; G6 lands the aircraft if the return fails.

The FSM stays geometry-free: it is told two numbers on every Loiter tick
(`TickInput.hold_off_m`, `hold_reach_m`), from `HoldWatch`
(guardrail/hold_flight.py), which both the ROS 2 node and the harness call
before each FSM step:

- **The point** is the one the hold flies to: the companion's GUIDED target
  (`HoldFlight.point`) while it holds the Loiter in GUIDED, else the FSM's
  plan (`fsm.hold`: the old flow, a hold handed to BRAKE, the harness's
  modelled autopilot), else where the Loiter began.
- **The reach** is set whenever the point is: its distance then, plus the
  braking distance from the horizontal speed then on the rail Shield's own
  stop model (guardrail/response.py ARDUCOPTER_GUIDED: 0.45 s, then
  1.75 m/s^2 - chosen so that its median braking error is not short), plus
  `HOLD_KEPT_SLACK_M` = 3.0 m. A new point (a re-plan, a transit given up)
  sets a new reach from its own distance, so it never starts beyond it.
- **The position and speed** are the ones the hold is flown on: the node's
  pose and measured velocity (the speed from two poses when the velocity is
  stale), the harness's true state and the ground velocity its last tick
  flew - the autopilot's own, as TransitWatch reads it there.

Why a reach and a trend, and not a distance or a velocity error alone: a
hold entered at speed overshoots its point by its braking distance (3.6 m at
3.8 m/s on the rig; 46 m of model reach at 12 m/s), and a velocity error is
largest exactly while it brakes; neither tells a hold that is braking from
one that is being carried away. The reach bounds the overshoot by the
Shield's own stop model; the trend separates "out there and still going"
from "out there and coming back" (or stuck there, a transit that cannot
close: TransitWatch's case).

### The thresholds, from the rig

Every Loiter the hil rig has flown (606 runs, 413 spells, hs02 to the nightly
nt1111 so far), replayed through HoldWatch and the FSM
(tests/test_hold_points.py replays sm1111's own rows):

- The farthest any GUIDED-hold spell (hs05, hs06, nt1107, sm1110, sm1111,
  nt1111: 324 spells) went beyond the stop model alone was 1.5 m
  (sm1110_045, wind-within-authority) - apart from the 12 m/s cells (22-24 m)
  and nt1111_001 (4.9 m, below).
- ArduCopter's BRAKE after a GUIDED stream carries the aircraft by its
  integrator, up to the airframe's 2.5 m/s^2 x 1 s^2 = 2.5 m (hold_flight.py,
  WHY NOT BRAKE); measured 2.4 m past the stop model on hs04_005 / 006, the
  flow before the GUIDED hold, which a stream fault still forces.
- 3.0 m clears the worst GUIDED hold by 1.5 m and a BRAKE hand-over by
  0.5 m. The closest any other spell then came to its reach was 0.8 m short
  (hs04_005 / 006); rh03_039 (gps-dropout) 1.4 m, sm1110_045 1.5 m.
- T_recover, 0.1 m/s at two standard errors and 0.5 m are X7's, unchanged:
  the 12 m/s holds recede at 1.7-2.8 m/s (fitted, at the firing tick), the
  harness's at 1.6 m/s (8 m/s) and 5.6 m/s (12 m/s).

X9 fires on six spells: the five 12 m/s cells of lane 15 (hs02_009,
hs03_011, rh03_044, sm1110_048, sm1111_048) at 11.2-11.4 s, 12-15 m out,
5.2-5.6 s into the Loiter and 1.8 s before sm1111_048's closest pass of the
zone; and nt1111_001 (nfz-head-on) at 12.9 s, whose SITL restarted 2.5 s
before the Loiter and handed the node a stale pose and velocity: the
aircraft accelerated to 4 m/s inside its hold and went 9.3 m past its point
(a hold not kept, for a reason that is not wind; r2 expects a fail-safe
there, which G5 gave at 38.8 s). It fires on no 8 m/s cell and no
within-authority cell.

Noise: the position X9 reads is the one the hold is flown on, a settled hold
on the rig moves by centimetres. Held beyond its reach at one distance with
0.1 m of white noise a tick, no seed of 100 fires X9 over 29 s; with 0.3 m,
30 do. A rail whose position input is that noisy would need a stronger
floor (tests/test_fsm.py, test_x9_needs_the_hold_still_receding).

### Headless

On the ideal vehicle the excess wind drifts the hold (1.6 m/s at 8 m/s,
5.6 m/s at 12 m/s), so every wind-beyond-authority cell goes G1 G3 X9: 9.0 s
at 8 m/s (then home, G9), 6.8 s at 12 m/s (then G6, no progress home), where
`hold_kept: off` went X7 at 17.3 s and 7.3 s once the drift had carried it
into nfz-square. The rail vehicle's GUIDED hold keeps the 8 m/s cell (its
command back is flown in full beside the 1.6 m/s excess, the harness's
"optimistic upwind"), which runs to G5 as on the rig; at 12 m/s it goes X7
at 7.0 s, as before. wind-within-authority and wind-gusts take no X9 on
either vehicle (tests/test_harnesstruth_wind.py). Other families: a Loiter
moves its aircraft only by the hold's own braking (within the stop model:
the response arms brake harder and sooner than it, 2.4-2.5 m/s^2 after
0.08-0.13 s), a moved hold's transit (closing on a point whose reach starts
at its distance), or BRAKE's carry (within the slack), so X9 should fire on
no other family; the library, smoke and nightly profiles were not re-run
with it (a KPI campaign was flying), and must be.

### Not done

- `sitl/run_sitl_demo.py` (the pymavlink rail) and the city loop do not pass
  the input: X9 never fires there, as X8 never fires on a rail that reports
  no fix quality.
- G6 on the ROS 2 node still has no input (`rtl_failed` is False on every
  tick): an X9 RTL that makes no headway home into the wind stays in RTL on
  the rig, carried by the wind, where the harness lands it. (Done
  2026-11-14: the node's RTL watch, "2026-11-14: the edges the rails had
  never taken".)

## 2026-11-07: the FSM on the rail vehicle

The hil rail flies ArduCopter 4.5.7 SITL in GUIDED. Labelled on a vehicle
that answers like it (the harness's measured ArduPilot response,
`--response-arms ardupilot`; the proposal branch `e5-prescribed-on-rail-
vehicle`, docs/RUBRIC-failsafe.md section 8 there), two FSM behaviours
stayed wrong under every label set: a missed fail-safe on
corridor-drift-angles at 8 m/s, -30 deg (library 1, nightly 3), and
fail-safes by X6 on dyn-nfz-rotate-spin (nightly 9) where the rubric expects
none. Both are real on the rig's own response too - the harness's `rail`
arm (rail-scorer-gaps, 3b67cbe: the fit of 32 rh01 flights, 0.13 s and
2.48 m/s^2, from a hover, holding in place as the rails do) and a slower
stand-in (the rail Shield's own 0.45 s / 1.75 m/s^2) - and the second was
flown: the hil rehearsal rh01_017 (rotate-spin, 3 deg/s) went home by X6 at
30.3 s. The first is fixed; the second is not, and why is measured below.

### An untrusted repair beside a speed cap alone is the chain's

The speed cap was the only rule the 8 m/s request broke. The Shield's repair
chain runs every operator on a violated tick, each on its own test, and
after SpeedClamp, CorridorReturn's "leaves within one lookahead" - a
straight extrapolation along the NEAREST leg's normal - moved the clamped
action by up to 4.0 m for a corridor the request does not break: the
forecast cuts across the dog-leg's corner, inside the 20 m half-width all
the way (`_check_corridor` walks the polyline and finds nothing). The FSM
read that size as an untrusted repair and stopped the aircraft - p4's
"abandons projection", rightly - but G3 escalated only an untrusted repair
of a violated rule that is not kinematic, and the only rule violated was
the speed cap: the aircraft was held at (34.6, 6.7) in Brake for the whole
70 s. On the ideal vehicle the same operator moved the action by 0.09 m
(the instant response holds the lateral offset at 2.8 m), and the flight
went on into nfz-school, over theta, G3 and G5 - the fail-safe the rubric
expects (E5, on both vehicles).

The untrusted repair of such a tick is the chain's, made for the kinematic
rule it ran for, and p4 says what follows: "the Shield abandons projection
and triggers fail-safe". `FSMConfig.untrusted_owner` (new; default
`chain`): on a tick that violates no non-kinematic `project_fix` rule, an
untrusted repair (over theta, or not converged) belongs to the violated
kinematic rules; a hard one takes G3, and Loiter keeps holding a request it
could not hand back (it is not "acceptable"), so a planner that keeps
asking reaches the operator timeout. Beside a rule theta judges nothing
changes: the speed cap is still never cited for that rule's size
(test_a_theta_escalation_is_attributed_to_a_rule_theta_judges), and a soft
cap never changes the mode. The record names the owner on such ticks
(`untrusted_chain`) and the G3 reason says it. `projected` is the reading
until today, kept for comparison; it hashes as every config did before the
field existed (`LEGACY_LAW`: today's default without it is still
`sha256:d55e1bfe...`; the new default is `sha256:1e413c8b...`). The c77c77d
identity proof, the in-place hold digests and the 2026-10-08 flight replay
fly `projected`, as they fly the old Loiter law. On the ArduPilot response
the cell now goes Brake at 0.0 s, Loiter at 1.4 s and home by G5 at 31.4 s;
on the rail arm G3 at 2.7 s and G5 at 32.7 s, on the slower stand-in G3 at
3.0 s and G5 at 33.0 s (before: Brake to the end on all three).

Two other readings were tried or weighed and not taken:

- Theta judging only the repairs of violated rules (the first version on
  this branch, a40852a): the over-theta projection then FLIES (G1, the
  Shield's action passed) on such a tick. It moved the same four episodes,
  but it breaks the conservative cap and the 2026-10-19 review's rule that a
  projection theta did not judge never flies: beside a yaw breach,
  CorridorAltitudeFix's 4.55 m cut of a legal climb flew instead of X1's
  stop (test_corridor's
  test_the_floor_never_flies_a_projection_theta_did_not_judge failed).
- Making CorridorReturn's test agree with the polyline check would remove
  the spurious size at its source, but it changes the Shield's projection,
  and with it the flights without a fail-safe that every label is read from
  (rubric section 3). Not done here.

The rubric's E5 tick (`tools/failsafe_rubric.py`, `unsafe_request`) still
mirrors the `projected` reading: it counts a hard rule's repair over theta
only where a non-kinematic rule is violated. The labels do not move with
this change (they are read on the flight without a fail-safe), and they
would not move if that tick followed the `chain` reading either: widened,
it moves no label in 1792 cells (smoke, library and nightly, the ideal and
ArduPilot vehicles); only the -30 deg cell's E5 spell on the ArduPilot
vehicle's prescribed flight grows, 67.3 s to 68.5 s. Aligning it is left to
the rubric.

### X6 on a hold that finds no place to stop: kept, and why

`stop_illegal` is "standing still here breaks a hard rule within the
lookahead" (Shield.state_is_unsafe) - the same "unsafe" the time-to-safe
KPI counts. On rotate-spin the lagging vehicle's slide goes over theta
(Loiter at 3.9-4.1 s, about 3 m nearer the bar than the ideal vehicle
stops); the head-on request then stays over theta for most of the hold, and
when it clears, the bar is sweeping toward the held aircraft: a stop is
illegal in the lookahead though the position is past no limit (unsafe depth
0.0), and after T X6 sends it home (ArduPilot response: 31.0 s, 17.1 s and
12.5 s at 3, 6 and 9 deg/s; the rail arm the same). Its progress test
cannot pass at depth 0.0, which looks like a vacuous trigger. Two changes
were measured (a40852a, c843d7c on `fsm-rail-vehicle-fixes`):

- X6 and X7 counting only the ticks PAST a limit (depth above 0.0): no other
  episode on either vehicle moved, but the three rotate-spin cells still
  ended in a fail-safe - the operator timeout (G5 at 34.0-34.1 s), landing
  after a failed RTL at 3 and 6 deg/s - and their unsafe spells grew (6
  deg/s: 8.0 s to 24.0 s; the ArduPilot arm's mean time to safe, library
  3.837 s to 4.325 s, nightly 4.283 s to 4.622 s).
- Loiter's hand-back (G8) reading the same: rotate-spin at 9 deg/s on the
  ideal vehicle was handed back at 8.4 s, into the bar's sweep, and breached
  it once (the library cell and all three nightly seeds failed their
  breach gate); as it is, it waits for the bar to pass (G8 at 20.1 s).

So X6 there ends a hold that has found no place to stop for T, the state the
time-to-safe KPI counts as unsafe, and the alternatives are a longer unsafe
hold ending the same way, or a hand-back into the sweep. It stays as it was
(44c7018). Where the hold is placed decides it: with the rails' own hold
(ArduPilot's, where the aircraft is: `--autopilot-hold in_place`) the IDEAL
vehicle goes home by X6 at 9 deg/s too (13.3 s; the rail-scorer tree's
library and nightly sweeps fail that cell's gate on 1 + 3 episodes, before
and after this change alike), while the FSM's moved hold points - which no
rail flies yet - keep it (G8 at 20.1 s); on the lagging vehicles the moved
points are not enough. The rubric's N4 ("a constraint imposed on the
aircraft that it can leave within the caps") does not credit a hold with no
legal stop, and the flights without a fail-safe it reads leave the bar in
1.5-23 s. Removing these fail-safes is a change of the hold - a point the
sweep has already passed, flown by the rails - or a label decision that a
hold with no legal stop for T is a fail-safe the KPI should expect. Not done
here.

### Measured

Headless, the grant's 5 s forecast; fail-safe trigger correctness per
episode, before (integrate/1107b, c7a69e8) and after this change, against
the label sets of the rail-vehicle proposal: (a) ideal-vehicle labels,
section 3's reading (the adopted ones); (a2) ideal, section 7's prescribed
reading; (b) ArduPilot-response labels, section 3; (c) ArduPilot, section 7;
(r) and (r2) the same two readings on the rail arm. Censored runs are
counted apart at the scenario's length, and flown 35 s longer (the Loiter
timeout plus 5 s) for the "flown longer" rows. Wilson 95 % lower bounds in
brackets. The ideal and ArduPilot rows are this branch's harness (moved hold
points); the rail rows are a scratch copy of rail-scorer-gaps (3b67cbe) with
and without this change's `guardrail/fsm.py`, flown with the rails' hold in
place (`--response-arms ideal rail --autopilot-hold in_place`).

| profile, arm | labels | before | after |
|---|---|---|---|
| smoke, ideal | (a) / (a2) | 62/63 (0.915) / 63/63 (0.943) | unchanged |
| smoke, ArduPilot, scenario length (9 censored) | (a) / (b) / (c) | 49/54 (0.801) / 52/54 (0.875) / 53/54 (0.902) | unchanged |
| smoke, ArduPilot, flown longer | (a) / (b) / (c) | 49/63 (0.661) / 61/63 (0.891) / 62/63 (0.915) | unchanged (no -30 deg cell in smoke) |
| smoke, rail, scenario length (8 censored) | (a) / (r) / (r2) | 51/55 (0.827) / 53/55 (0.877) / 54/55 (0.904) | unchanged |
| smoke, rail, flown longer | (a) / (r) / (r2) | 51/63 (0.696) / 61/63 (0.891) / 62/63 (0.915) | unchanged |
| library, ideal | (a) / (a2) | 177/179 (0.960) / 179/179 (0.979) | unchanged |
| library, ArduPilot, scenario length (10 censored) | (a) / (b) / (c) | 154/169 (0.859) / 163/169 (0.925) / 165/169 (0.941); 1 missed | 155/169 (0.866) / 164/169 (0.933) / 166/169 (0.949); 0 missed |
| library, ArduPilot, flown longer | (a) / (b) / (c) | 154/179 (0.802) / 173/179 (0.929) / 175/179 (0.944) | 155/179 (0.808) / 174/179 (0.936) / 176/179 (0.952) |
| library, rail, scenario length (9 censored) | (a) / (r) / (r2) | 156/170 (0.867) / 164/170 (0.925) / 166/170 (0.941); 1 missed on (r), (r2) | 157/170 (0.874) / 165/170 (0.933) / 167/170 (0.949); 0 missed |
| library, rail, flown longer | (a) / (r) / (r2) | 156/179 (0.815) / 173/179 (0.929) / 175/179 (0.944) | 157/179 (0.821) / 174/179 (0.936) / 176/179 (0.952) |
| nightly, ideal | (a) / (a2) | 729/741 (0.972) / 741/741 (0.995) | unchanged |
| nightly, ArduPilot, scenario length (36 censored) | (a) / (b) / (c) | 636/705 (0.878) / 681/705 (0.950) / 693/705 (0.970); 3 missed | 639/705 (0.883) / 684/705 (0.955) / 696/705 (0.976); 0 missed |
| nightly, ArduPilot, flown longer | (a) / (b) / (c) | 636/741 (0.831) / 717/741 (0.952) / 729/741 (0.972) | 639/741 (0.836) / 720/741 (0.957) / 732/741 (0.977) |
| nightly, rail, scenario length (33 censored) | (a) / (r) / (r2) | 642/708 (0.883) / 684/708 (0.950) / 696/708 (0.971); 3 missed on (r), (r2) | 645/708 (0.888) / 687/708 (0.955) / 699/708 (0.976); 0 missed |
| nightly, rail, flown longer | (a) / (r) / (r2) | 642/741 (0.840) / 717/741 (0.952) / 729/741 (0.972) | 645/741 (0.844) / 720/741 (0.957) / 732/741 (0.977) |

On every arm and profile exactly the corridor-drift-angles 8 m/s, -30 deg
episodes moved (ArduPilot: library 1, nightly 3; rail: library 1, nightly 3;
Brake to the end -> G3, G5, home; expected on every label set); no other
episode on any arm changed its FSM outcome or edges, at the scenario's
length or flown longer. P0 escapes 0 on every profile and arm, before and
after; time to safe unchanged (ideal 2.533 / 2.540 / 2.539 s, ArduPilot arm
3.807 / 3.837 / 4.283 s, rail arm 4.953 / 5.171 / 5.566 s for smoke /
library / nightly); the sweeps' own gates unchanged (this branch: smoke 55
pass / 6 tracked / 0 fail / 3 known, library 154 / 22 / 0 / 4, nightly
660 / 66 / 0 / 18; the rail-scorer tree: 55 / 6 / 0 / 3, 153 / 22 / 1 / 4,
657 / 66 / 3 / 18, its failures the in-place rotate-spin above). The hil
rehearsal rh01 (38 missions) and the shakedowns shk01 / shk03, replayed
through the new FSM, keep every edge: no flown tick of the corrected kind
(the -30 deg cell was not flown there).

## 2026-11-06: X8, the position the Shield is handed

The fail-safe KPI's remaining misses (headless harness, against
docs/RUBRIC-failsafe.md) had three causes. Two are answered here; the third
(theta on the first deep forecast, 15 false triggers) is not, because no
change to the FSM can answer it under the decided theta basis and E5 (the
rubric's section 5 says why).

### The frozen fix: what the rubric asked and the FSM could not see

Rubric E3 expects a fail-safe when "the position the Shield acts on is wrong
by more than the policy's margin around a hard P0 boundary ... while the
aircraft is within that error of the boundary". Its section 4 said plainly
that the FSM had no input for it. gps-dropout lane 23.6, windows 2-9 s and
6-13 s, were the missed triggers: the pose frozen at (-22, 23.6) for 7 s, the
Shield passing 4 m/s throughout, the aircraft ending 1.2 m from nfz-square
with 28 m of error, and no fail-safe.

**The inputs** (`guardrail/models.py` State):

| field | meaning | None (default) |
|---|---|---|
| `fix_ok` | (x, y) is a current, valid fix. False: a stale pose (the pose's stamp older than the monitor period, or a bridge re-publishing its last sample), no 3D fix, or an EKF without absolute horizontal position | not reported: the position is trusted exactly as before |
| `pos_sigma_m` | the reported 1-sigma horizontal accuracy, metres | not reported - never read as "perfect" |

**The test** (`Shield._position_verdict`) is E3 on the Shield's own estimate:

- While `fix_ok` is False the Shield dead-reckons from the last valid fix on
  what it itself streamed (its command; 0 for a stop or an in-place hold;
  a moved hold point's transit at its plan's speed).
- The handed position is wrong by err = |estimate - handed| + u, where u =
  `dr_drift_mps` x the time since the last valid fix is the drift a dead
  reckoning on commands cannot see (the measured ArduCopter GUIDED response
  flies a setpoint from 0.45 s after it is sent at 1.75 m/s^2; wind and
  gusts move the aircraft where no command did). The boundary is taken u
  nearer.
- `dr_drift_mps` is configurable (`Shield(dr_drift_mps=...)`), 0.5 m/s by
  default (`shield.DR_DRIFT_MPS`), and recorded in every X8 reason, in the
  harness's episode_start and in every bundle (`arm.dr_drift_mps`). With 0
  the test fires on the tick E3 first holds on the ideal vehicle; a larger
  allowance only makes it earlier.
- With `pos_sigma_m` reported, err is at least 3 sigma, judged at the
  position and along the request's own forecast: the rubric reads
  noise-E3 against the planned route, and the Shield's view of the plan is
  the request.
- X8 fires when err exceeds a hard, enforced P0 zone's `margin_m` with the
  zone's boundary within err of the estimate.

**Why RTL, and when Land.** A stale or noisy Shield input does not stop the
autopilot navigating: it still flies on its own estimate. So X8 returns
(RTL), and the grant's G6 lands the aircraft when the return fails
(rtl_path's blocked leg, no progress over T, a critical battery). A first
version landed instead, and landing through the 10 m altitude floor raised
the library's autopilot breaches from 10 to 22 and its true-state P0 ticks
from 1768 to 4741. Where the AUTOPILOT has lost its own fix (GPS_RAW_INT
fix_type below 3, an EKF without absolute position) RTL cannot be flown and
ArduPilot's own EKF failsafe lands (FS_EKF_ACTION = 1). That is a second
input, not modelled: the headless autopilot has a perfect position. (Since
2026-11-12 the ROS 2 node reports it, `estimator_ok`, and the rail vehicle
models it: "2026-11-12: X8 on the autopilot's own estimator".)

**Precedence.** X8 comes before every rule's own answer in Normal, Brake and
Loiter: each of those answers was computed from the position that can no
longer be certified. It does not act in RTL or Land.

**When nothing changes.** A fix that comes back before the error reaches a
boundary costs nothing; a fix frozen on a held aircraft does not move the
estimate (the N2 cell lane 15, 6-13 s). A rail that reports neither field
never takes X8.

### Noise inside the margin is not a pilot thrashing

Two nightly false triggers (gps-noise-within-margin latency 0 / pass_y 23.2 /
sigma 0.3, seeds 1001 and 1003) were N-in-T counting noise: the slide held the
aircraft 0.4-0.9 m outside nfz-square's ring, 0.3 m of white noise flipped
the request's forecast in and out of the ring, every return was a new onset
(at most one a second), and G2 then G4 sent it home on repairs of 0.02-0.43
m. With a reported accuracy, a clean tick whose request passes within 3 sigma
of a hard zone's ring (`Shield._noise_marginal`) keeps the violation "on" for
onset counting (`TickInput.noise_marginal`): the next return is the same
violation seen through noise. Without a reported accuracy nothing changes.
The hysteresis would also have removed the accidental N-in-T trigger of
gps-noise-beyond-margin (E3, 3 sigma 3-6 m against the 1 m margin); X8's
sigma branch answers those cells directly instead (a first version without
the planned-route reading left 6 of them ending in Loiter and 1 missed).

### Once a rail reports its accuracy, a margin is an accuracy requirement

`margin_m` is the position-error budget the rubric's E3 reads. A 1 m margin
certifies a zone only while 3 sigma <= 1 m (sigma <= 0.33 m, RTK class). With
ordinary GNSS accuracy (1-3 m) the request's forecast passing within 3 sigma
of such a zone returns the aircraft. That is the rubric's E3 applied
honestly; sizing every hard P0 margin to the real (EKF-fused) accuracy of the
rail that flies it is a policy decision, and no policy was changed with this.

### Where each rail's signal lives

| rail | fix validity (`fix_ok`) | the autopilot's verdict (`estimator_ok`, since 2026-11-12) | accuracy (`pos_sigma_m`) | today |
|---|---|---|---|---|
| ROS 2 + MAVROS 2 (sitl/ros2_shield_node.py; the hil topology) | **wired 2026-11-07:** `/mavros/global_position/raw/fix`, the NavSatFix MAVROS 2's global_position plugin (core MAVROS, no mavros_extras needed) builds from GPS_RAW_INT: STATUS_FIX when fix_type > 2, else STATUS_NO_FIX; `fix_ok` = status >= STATUS_FIX while the message is no older than the pose timeout (1 s), None before the first one or once stale (`fix_ok_from`). Not used: `/mavros/gpsstatus/gps1/raw` (mavros_extras' gps_status plugin), the pose's stamp age | **wired 2026-11-12:** ArduPilot's EKF_STATUS_REPORT from `/uas1/mavlink_source` (MAVROS's raw feed; no MAVROS 2 plugin publishes the message), as ArduCopter's own EKF failsafe judges it (`EstimatorHealth`; "2026-11-12: X8 on the autopilot's own estimator") | GPSRAW.h_acc, or the NavSatFix position_covariance MAVROS builds from h_acc / eph - **not fed**: a policy decision, below | `fix_ok` and `estimator_ok` reported (the latter decides); `pos_sigma_m` None |
| Project AirSim + ArduPilot (demo/pas_ardupilot/autopilot.py) | already parsed: GPS_RAW_INT.fix_type (`snap.gps_fix`), the position's receipt time (`snap.t_pos`) - used only as a pre-arm gate | EKF_STATUS_REPORT flags already parsed (`snap.ekf_flags`, `ekf_ready()`), a pre-arm gate only | GPS_RAW_INT.h_acc (not parsed yet) | not wired |
| Project AirSim simple_flight (demo/follow_vlm.py and the city rails) | none: the State is `get_ground_truth_kinematics()`, a perfect position; only a configured GPS sensor, or injection, could report one | none (no autopilot estimator) | none | None (unobservable) |
| headless harness (experiments/sweep_scenarios.py) | `stress.gps_dropout_s` with `stress.fix_quality_reported` (default true): False through the window | the dead-reckoned denial (the rail vehicle's): True until `RAIL_EKF_POS_VALID_S` (7.8 s) after the loss, then False; the frozen fix (the templates'): not reported | `stress.gps_noise_m` with the same flag | reported; withheld per scenario to keep "the Shield flying blind" measurable |

**On the ROS 2 rail (2026-11-07).** The node's Shield has no FSM of its own
(guardrail.replay.rail_shield): the node runs THE FSM, because only it sees
the autopilot. So the Shield judges X8's input on any state that reports its
fix quality and hands it out (`ShieldDecision.position_lost`, excluded from
every dump), and the node passes it to its FSM's `tick_input_from_decision`
(`position_lost=`). The Shield's dead reckoning integrates what the node
SENT, not what filter() emitted: the node's FSM decides after filter()
whether a stop, nothing (the autopilot holding) or the Shield's action flew,
and tells the Shield through `note_flown`, which on a Shield with no FSM now
also feeds the dead reckoning. A state that reports nothing - every other
rail, the harness's oracle on the true state - computes nothing new.
`pos_sigma_m` is NOT fed on this rail: with the shipped 1 m margins X8's
sigma branch certifies a zone only while sigma <= 0.33 m. SITL's u-blox
stream reports 0.3 m (SIM_GPS_ACC, libraries/SITL/SIM_GPS_UBLOX.cpp:256),
just inside; an ordinary receiver in the field reports metres, and then
every request passing within 3 sigma of a hard P0 zone returns the aircraft.
Whether margins are sized to the rail's accuracy, or the accuracy is fed, is
a policy decision left open. On SITL the fix is lost by setting
SIM_GPS_DISABLE from the simulator's side (sitl/sitl_stressors.py;
docs/RUNBOOK-orin-hil.md step 14):
there the pose the Shield is handed keeps moving on EKF3's inertial dead
reckoning instead of freezing, so X8 fires on the drift allowance alone,
later than on the headless harness's frozen fix, and ArduCopter's own EKF
failsafe lands the aircraft about 8 s after the GPS is lost (7 s
posRetryTimeNoVel_ms, then 1 s of ekf_check) - the "second input, not
modelled" above, which a flight now meets. Since 2026-11-12 the node reads
that verdict and X8 waits for it ("2026-11-12: X8 on the autopilot's own
estimator"): the drift-allowance X8 fired falsely on nt1111's lane-23.6
cells.

### The KPI invariant

guardrail/kpi.py's per-tick `false_trigger_rate` is an invariant check: the
FSM leaves Normal only on an enforced violation, so a violation-free tick
that leaves it is a logging fault. X8 leaves it on a tick whose request may
be clean, by design: its input is the position, not the request. Such a tick
is left out of the count by its reason (`kpi._position_trigger`: the row's
edge, or its record's `position_lost`), and the invariant still holds for
every request edge (tests/test_position_quality.py). Logging the uncertified
zone as a violation instead would put a rule breach on a tick whose request
broke nothing, and the P0 counters would read it.

### Measured (headless, not KPI-grade)

Against docs/RUBRIC-failsafe.md, E5 unchanged, the grant's 5 s forecast on
the ideal vehicle, before (master 23ac319) and after (this change, with the
Shield's recovery law "cap_all", docs/DESIGN-shield-enforcement.md):

| profile | before | after | Wilson 95 % low |
|---|---|---|---|
| library (seed 0) | 175/179 = 0.978 (2 false, 2 missed) | 177/179 = 0.989 (2 false, 0 missed) | 0.944 -> 0.960 |
| smoke | 62/63 = 0.984 (1 false) | 62/63 = 0.984 (1 false) | 0.915 -> 0.915 |
| nightly (seeds 1001-1003) | 721/741 = 0.973 (14 false, 6 missed) | 729/741 = 0.984 (12 false, 0 missed) | 0.959 -> 0.972 |

`tools/failsafe_rubric.py` on the final code: every declared label agrees
with the rubric (library 168 cells, smoke 56, nightly 672 cell-seeds), none
undecided in its length; no label changed.

- **Every missed trigger is a trigger now.** gps-dropout lane 23.6,
  windows 2-9 s and 6-13 s (library 2, nightly 6: every seed): X8 at 5.3 s
  and 7.6 s, when the frozen fix's error (dead-reckoned, plus the drift
  allowance) passes nfz-square's 1 m margin with the zone's boundary within
  that error; home at 9.3 s and 13.3 s. The dropout cells that expected a
  trigger and had one got it late, through the hold: X8 now returns at
  5.3-6.4 s where G5 fired at 39.1-45.0 s, and lane 23.6 window 4-11 s,
  which landed (G6), now returns.
- **gps-noise-beyond-margin returns on X8** at 2.5-4.7 s (every cell and
  seed of the three profiles), where N-in-T's G4 fired at 7.7-12.3 s.
- **The nightly's noise false triggers are gone**: gps-noise-within-margin
  latency 0 / pass_y 23.2 / sigma 0.3, seeds 1001 and 1003, finish without
  a fail-safe.
- **What remains**: the false triggers of theta on the first deep forecast,
  X1 -> G3 -> G5 at 30.1 s: rail-mission-stub-vla (library, smoke, and the
  nightly's 3 seeds) and standoff-approach-angles 7 m pedestrian (library,
  and the nightly's 3 variants x 3 seeds) - library 2, smoke 1, nightly 12.
  Under the decided theta basis and E5 these stay false triggers
  (docs/RUBRIC-failsafe.md, section 5).

## 2026-11-02: the delegated decisions, and correctness against a written rubric

Headless harness throughout; nothing here is KPI-grade (the grant measures
its KPIs on the hil topology).

### The four decisions

1. **Theta's basis: reading A, decided.** Each repair's forecast depth in
   metres (`magnitude_m`, `fsm.DEFAULT_THETA_HORIZON_S = None`). It is the
   grant's audit-example field (p5) and the PI reference's metric ("a deep
   head-on dive ... is escalated rather than projected").
2. **The fail-safe rubric**, written before any label changed:
   docs/RUBRIC-failsafe.md.
   - "Triggered" is RTL or Land; Brake and Loiter are holds.
   - A fail-safe is expected for: E1 a hard RTL / land rule, E2 a
     disturbance beyond authority, E3 a position error beyond the margin
     near a P0 boundary, E4 no legal continuation, or E5 an unsafe repair
     the planner keeps asking for past the operator timeout.
   - Each clause is judged on the flight without a fail-safe, never on
     what the FSM did.
3. **LOITER flown as ArduCopter's BRAKE: kept, as best practice.**
   - The grant's FSM, the audit record and `set_mode` say LOITER. Literal
     LOITER is not feasible without a pilot holding the throttle centred:
     measured in SITL, it went from 15 m to the ground in 6.7 s.
   - BRAKE holds position and altitude with no pilot input.
   - `--hold-mode LOITER` remains for a flight with a safety pilot on the
     sticks.
   - **N-in-T for a pilot slower than 10 Hz:** `FSMConfig.for_decision_period(p)`
     sets T = max(T, N x p x 1.5). It is option b of the 21 Oct questions,
     and every 10 Hz pilot keeps exactly the grant's thresholds. Since
     2026-11-17 the ROS 2 node can apply it for a pilot declared slower than
     T / (1.5 x N) (`--decision-period-s`; 1.11 s for the grant's N and T),
     on a hand-launched node only: no campaign path (rail_node_args, the
     harness's extra node arguments) passes the option, and no flight has
     applied it.
4. **PathRepair on by default: measured, kept off.**
   - **Cost.** The library flown with `path_repair=True` scores 150/171 =
     0.877 (19 false, 2 missed, 8 censored), against 175/179 with it off.
     It also adds 26 failing episodes:
     - all six nfz-head-on arms;
     - ten gps-noise-within-margin cells sent home;
     - wind-within-authority lane 15 sent home;
     - the N1 cells of dyn-nfz-spawn-ahead sent home.
   - **Mechanism.** PathRepair steps in where projection lost half the
     progress, which means head-on. Its `magnitude_m` is the depth of the
     forecast it replaced, so on the decided basis nearly every detour it
     proposes is over theta and escalates.
   - **What did not change:** P0 escapes (0) and polygon entries.
   - It stays selectable (`Shield(path_repair=True)`).
   - **Superseded 2026-11-17 (WP3-06): PathRepair is ON by default**, in
     the grant's order (after LateralProjection), redesigned so it no
     longer escalates by itself: its `magnitude_m` is the correction of the
     action it emits (|emitted - raw| net of the speed clamp, x 0.1 s), the
     projection records it replaces are superseded (removed from the
     decision's repairs, kept on the pass ledger), and in chain mode it
     detours round stand-off rings only (zones stay with the projection:
     zone detours cost a labelled RTL and true-state margin, measured), and
     stands aside where the tick's projection is already over theta, during
     a recovery, without a position fix, or where one control period of the
     detour would take the request's forecast deeper into the ring.
     Measured on smoke, the library and the nightly cells (seed 1001): P0
     escapes 0, fail-safe trigger correctness 63/63, 179/179, 247/247
     (unchanged), mission success unchanged; 0, 1 and 3 episodes fly
     differently (standoff-approach-angles offset 9, same outcome).
     docs/data/pathrepair_default_20261117/README.md. Every run flown
     before 2026-11-17 had PathRepair off; the next hil campaign flies it
     (`path_repair=False` restores the old chain).

### FSM behaviour that was wrong (fixed, `guardrail/fsm.py`)

**G8 (Loiter → Normal) read only "no hard violation in the request".**
In Loiter the autopilot flies and nobody flies the request. Two faults
followed:
- G8 handed back an aircraft that was still inside nfz-square, because
  its request pointed out of the zone.
- G8 never handed back a request the Shield repairs by 0 m. The 30 s
  timeout then sent near-edge-at-12mps home twice, although no repair was
  unsafe (p1).

Now (`loiter_clear` "acceptable"):
- The request must be one the Shield would fly: clean, or a repair G1
  accepts, with no `loiter` rule violated.
- No violation may START on the tick; that is the thrash G4 counts.
- The position must be legal.

**X6 / X7 in Loiter waited for "a hard rule violated on this tick".**
That condition was written for Brake, where a clean request means the
pilot is leaving on its own. On wind-beyond-authority lane 15 at 12 m/s
it let a hold be blown through the zone with no escalation: a missed
trigger. In Loiter X6 / X7 now judge the aircraft's own position
(`loiter_x67` "position"). That cell now goes G1 G3 X7 G6 and lands off
the zone.

Both old readings stay selectable. The c77c77d in-place identity test
and the 8 Oct flight replay pin the old law explicitly. On the 8 Oct log,
today's law fires X6 / X7 inside the first Loiter: the old ArduCopter
LOITER sank the logged aircraft below the floor.

### Censoring and episode length

- **Censored outcomes.** 27 library episodes ended in Loiter before the
  30 s timeout had run. The 25 labelled ones were scored "correct, not
  triggered"; rail-mission-stub-vla and altitude-ceiling-hold ended 0.1 s
  before G5. Now `summary()["ended_holding"]` reports a timeout still
  pending, and the sweep scores such an episode as censored: not
  measured, and a failed label gate.
- **Longer episodes.** 13 families fly 50 s; corridor-drift-angles flies
  70 s.

### Results

| profile | before (7bfd6dc) | after | E5 read as "not expected" |
|---|---|---|---|
| library | 145/171 = 0.848 (25 false / 1 missed); nulls 0.942 / 0.058; Wilson 0.787 | **175/179 = 0.978** (2 / 2); nulls 0.659 / 0.341; Wilson **0.944** | 136/179 = 0.760 (41 / 2) |
| smoke | 57/61 = 0.934 (3 / 1); 0.885 / 0.115; 0.843 | **62/63 = 0.984** (1 / 0); 0.635 / 0.365; **0.915** | 48/63 = 0.762 (15 / 0) |
| nightly | 547/717 = 0.763 (167 / 3); 0.941 / 0.059; 0.730 | **721/741 = 0.973** (14 / 6); 0.623 / 0.377; **0.959** | 520/741 = 0.702 (215 / 6) |

The library's 3 s comparison arm, scored against labels derived for the
grant's 5 s forecast, gives 160/177 = 0.904 (2 / 15). It is reported,
never gated.

**Below 0.99 everywhere, and not shown at 95 % on any profile.**
- False triggers left: rail-mission-stub-vla and standoff-approach-angles
  7 m pedestrian (the theta mechanism below).
- On nightly the false triggers are 3 rail-mission and 9
  standoff-approach-angles (3 variants x 3 seeds), plus 2 pinned
  gps-noise-within-margin seeds (1001, 1003). In those two, 0.3 m of noise
  makes the ring flicker, N-in-T counts it (G2), and the flicker keeps
  coming in Loiter (G4). By N2 that is noise, not a planner thrashing.
- Missed triggers left: gps-dropout lane 23.6 with windows 2-9 s and
  6-13 s (6 on nightly). The Shield has no fix-quality input.

**Safety costs.**
- P0 escapes: 0 everywhere.
- Episodes entering a P0 polygon: unchanged. On the library that is 1
  Shield-on (tracked) and 4 by the autopilot; on smoke, 1 and 2.
- Ticks inside polygons fell on the library, from 490 to 487.
- Nightly: 3 Shield-on episodes into a polygon (tracked) and 12 carried in
  by the autopilot, the same as before.
- P0 ticks flown at the true state rose. Every tick of the rise is in an
  episode the rubric expects a fail-safe for:
  - library +323: E5 +183, E3 +34, and E2 +106;
  - smoke +260: E5 +120, E3 +34, E2 +106;
  - nightly +969: E5 +549, E3 +102, E2 +318;
  - autopilot breaches (the modelled RTL or LAND moving the aircraft
    somewhere illegal) went from 7 to 10 on the library and from 21 to 30
    on nightly. The new ones are standoff-reclassified's RTL past the
    pedestrian, corridor-swap-shifted's RTL out of the swapped corridor,
    and gps-dropout's landing through the floor. All are in expected
    episodes, and the control run (old code, new lengths) flies the same;
  - the E5 and E3 ticks are the expected RTL / LAND flown after the
    timeout, now that the runs are long enough to fly it;
  - the E2 ticks are wind-beyond-authority lane 15 at 12 m/s landing
    through the 10 m floor;
  - at the old lengths the FSM change alone adds exactly those 106 E2
    ticks, and nothing else moves (a control run of 7bfd6dc's code with
    the new lengths).

**Mean time to safe:** 3.135 s over 37 recoveries became 2.997 s over 39
(library). Max 6.7 s, unchanged. Two causes:
- X7 now ends wind-12 lane 15 earlier;
- the longer runs add two recoveries.

On 4 Nov, republished from the merged tree (a586956): 2.926 s over 39, max
6.7 s. One cell moved: dyn-nfz-spawn-on-top@size_m=20 leaves in 2.8 s
instead of 5.6 s. The zone appears centred on the aircraft, so every edge is
equally near; since `shield.escape_direction` breaks that tie by the
commanded heading, the pilot's forward command is the way out and the Shield
passes it, where GeofenceEscape used to steer for a side edge.

Nothing was changed to move this figure. The longest recoveries are
escapes at the caps from imposed positions (a grown zone, a reclassified
ring, a spawn on top), so they are limited by physics.

### Theta on the first deep forecast: three depths, measured, not decided

Both remaining false triggers come from the same mechanism. So does the
Gazebo Harmonic + ArduCopter SITL flight of the rail mission (master
70ab5e8: X1 on tick 1 at 8.13 m, G3, G5 at 30.2 s, target never left):
the first forecast to meet the zone goes deep, theta escalates it, and
the hold keeps the request deep. For GeofenceSlide and StandoffHold, the
depths measured were:
- **deepest** (shipped): the deepest forecast pose, which for a head-on
  approach is at the horizon's end;
- **entry**: the depth reached one control period after the first
  predicted entry;
- **weighted**: each pose's depth times max(0, 1 - t / lookahead).

The measurement was a scratch patch, never committed. It was scored on
the library against the published labels. Those labels' E5 assumes the
deepest basis: another basis would re-derive E5.

| depth | library correctness | Shield-on episodes into a P0 polygon | autopilot polygon entries | P0 ticks at true state | gated cells failing their breach gate | 8 Oct flight replay |
|---|---|---|---|---|---|---|
| deepest | 175/179 = 0.978 (2 / 2) | 1 | 4 | 1768 | 0 | 19 ticks over theta, X1 / G3; 0 escapes |
| entry | 163/179 = 0.911 (0 / 16) | **3** (deepest 8.0 m, tracked) | 2 | 1601 | **1** (dyn-nfz-rotate-spin 9 deg/s) | 0 over theta, no Loiter; 0 escapes |
| weighted | 171/179 = 0.955 (0 / 8) | 1 | 4 | 1786 | **1** (dyn-nfz-rotate-spin 9 deg/s) | 0 over theta, no Loiter; 0 escapes |

- **Both alternatives** remove the two false triggers and the Gazebo
  escalation. Under each, 14 or 6 E5 cells no longer escalate (with the
  published labels these read as missed).
- **entry** is near-inert at the shipped caps: 0.4 m in a 0.1 s period at
  4 m/s, against theta's 2.0 m. It lets two more tracked wind cells into
  a zone.
- **weighted** keeps the polygon entries but adds one self-caused breach
  and 18 P0 ticks.

Choosing among them is a change to decision 1, and it is left to a human.

## 2026-10-08: what the first follow flight on ArduPilot changed

`demo/out/pas_ap_20261008_224440` is a 120 s red-car follow on Project
AirSim with ArduPilot SITL as the flight controller, the Shield and this FSM
in the loop (Project AirSim + ArduPilot SITL over pymavlink, none of the
grant's topologies; not KPI-grade). Its FSM went Brake -> Loiter eight
times (G3): five on stand-off repairs "over theta" of 3.6-9.8 m, three on
"no operator converged". Every LOITER spell sank 1.4-1.8 m/s. The deepest
went from 7.6 m to 1.8 m in 3.3 s, and the aircraft bottomed out at 0.36 m
after GUIDED resumed; 279 ticks were under the policy's 6 m floor. The SITL
runs of 7 Oct had shown the same descent, and the harness's labelled
fail-safe correctness was 0.73-0.87 against the grant's 0.99.

### Theta: two readings, one default

What the grant says, word for word:

- Safety Shield p4: "Conservative cap. If ||action_repaired -
  action_original|| > threshold, or no operator converges, the Shield
  abandons projection and triggers fail-safe." Defaults: "θ=2.0 m (lateral)
  / 0.5 m (vertical)".
- Safety Shield p3: LateralProjection "Project[s] the predicted point to the
  nearest feasible point outside the NFZ or inside the corridor", and the
  first operator with "magnitude ≤ threshold wins".
- Safety Shield p5, the audit record example: raw action (5.0, 1.2),
  emitted (4.7, -0.3), and `{"operator": "LateralProjection", "result":
  "ok", "magnitude_m": 0.7}`.
- Stress Testing p5-p6: the KPI "Average repair magnitude" is the "Mean of
  ||post_repair - planner_output||".
- Safety Shield p1 / Architecture constraints p1: a 10 Hz monitor matching
  the 10 Hz VLA action rate, a 5 s lookahead.

**Reading A, per-operator depth (the default, `theta_horizon_s=None`,
`fsm.DEFAULT_THETA_HORIZON_S`).** Each operator reports how far the forecast
it corrected went into the rule (`magnitude_m`), and theta caps that. It is
what the p5 example records: (5.0, 1.2) to (4.7, -0.3) is a velocity change
of 1.53 m/s, and the example's 0.7 m is not that change over one period
(0.15 m), so its metres come from somewhere else - the projected point of
p3. It is the PI reference's metric (`safety_shield/repair.py`: the
penetration depth "is escalated rather than projected" for "a deep head-on
dive").

**Reading B, one control period (`theta_horizon_s=fsm.THETA_HORIZON_S`,
`--theta-horizon-s 0.1`).** p4's ||Δaction|| read literally: each action is
flown for one 0.1 s period, so |Δv| x 0.1 s is how far the repaired setpoint
moves the aircraft from where the planner's would have put it before the
next tick. A builder made this the default on 2026-10-08 and called it "the
grant's basis". The review of 2026-10-09 reverted that, for three reasons:
the p5 example and p3 above read the other way; under every shipped policy
(4-5 m/s speed caps, 2 m/s climb) a clamped command can change by at most
8-10 m/s laterally and 4 m/s vertically, 0.8-1.0 m and 0.4 m per period, so
theta could never fire - a de facto waiver of the conservative cap, which
the 6 Oct decision ("conform rather than waive") does not allow without the
PI; and its safety cost is measurable (the tables below: more episodes
entering a P0 polygon). It stays selectable on every rail, and the basis a
run used is recorded in `metrics.json` (`fsm.theta_basis`) and read back by
`guardrail/kpi.py`.

Two of the measurements that had pointed at reading B were measurement
artefacts:

- The stand-off "repairs of 3.6-9.8 m" were StandoffHold extrapolating the
  range in a straight line (20 of the flight's 25 reported more than the
  5 m ring's radius, up to 11.0 m). Fixed in shield.py: it reports the
  forecast's closest approach. On four of the five that sent the flight to
  LOITER the line ran through the subject; the fifth (tick 480, 16.5 m out,
  closing at 3.02 m/s) ended 1.4 m short of it.
  docs/FINDING-the-escape-that-stayed-inside-the-ring.md.
- The 1696-1701 of 1763 governed ticks over theta at h = 1-5 s ("The table,
  clamp-free" below) are the lookahead read as the repair's horizon: at
  h = 5 s any change over 0.4 m/s exceeds theta.

The replayed flight (`tests/test_flight_pas_ap_20261008.py`: every tick's
logged state, subject and raw action through the Shield and this FSM):

| shield.py | theta basis | Brake -> Loiter | of them over theta | repaired ticks over theta | P0 escape ticks |
|---|---|---|---|---|---|
| 10451bc (as flown; the replay reproduces the flight's edges exactly) | per-operator `magnitude_m` | 8 | 5 | 23 | 5 |
| this one | **per-operator `magnitude_m` (default; closest-approach depth)** | **2** | 2 | 19 | 0 |
| this one | one period (opt-in) | 0 | 0 | 0 | 0 |
| this one | 1.0 s | 6 | 6 | 26 | 0 |

On the default basis the 19 ticks over theta are all StandoffHold repairs
whose forecast's closest approach goes 2.1-4.3 m into the 5 m ring: the car
approached head-on at 2-3 m/s. The two Brake -> Loiter edges now hold
altitude (BRAKE, below) instead of sinking.

When a rail opts into reading B, the measure starts from the raw action
clamped to the caps the Shield applied on that tick (`Shield.applied_caps`,
which both ArduPilot rails pass as `caps`): a `monitor_only` cap, one
switched off mid-flight, or a soft cap the chain gave up is not one. Read off
the whole policy, they made the FSM refuse the tick (a fault, a hold,
nothing streamed); a first version of `applied_caps` left out every soft
cap once any soft rule was given up, and made the FSM refuse a tick whose
soft P1 speed cap had clamped
(`test_applied_caps_leave_out_only_the_soft_caps_the_chain_gave_up`). On
reading A the caps are not read.

**What each reading costs** (headless sweeps, Shield-on episodes, true
state; everything else as shipped):

| profile | episodes entering a P0 polygon: 10451bc / A / B | P0 ticks flown at the true state: 10451bc / A / B |
|---|---|---|
| library | 1 / 1 / 4 | 957 / 934 / 630 |
| nightly | 3 / 3 / 12 | 4945 / 4718 / 1964 |
| smoke | 1 / 1 / 2 | 434 / 392 / 342 |

Reading B flies fewer ticks inside a zone but lets more episodes into one:
`wind-beyond-authority` (5 m/s tailwind, 4 m/s cap) passes on A, where theta
escalates the deep forecast and the modelled LOITER holds the aircraft
outside, and fails on B (3.0-5.6 m inside the polygon before X7 sends it
home). How much of B's cost is the harness's open-loop wind model
(ArduCopter's GUIDED controller tracks ground velocity against a steady
wind) is open.

### X6 waits for a recovery that is working; X7 sends home one that is losing ground

X6 fired on every position illegal for T, including recoveries that were
working: climbs from under the altitude floor at the 2 m/s cap and 2 m/s
exits from a grown zone were sent home at 5.0 s (`altitude-floor-recovery`,
three `altitude-recovery-depth` cells, `standoff-reclassified`,
`dyn-nfz-spawn-on-top`, `radius-scaling-grow-deep`: seven false fail-safes by
their labels on the library at 10451bc). The Shield now reports how far the
position is from legal (`Shield.unsafe_depth`, `TickInput.unsafe_depth_m`:
summed over the hard rules a stop here breaks), and the FSM fits a
least-squares line through it.

- **X6** fires only if the fitted fall rate over the last T, less two
  standard errors, does not reach `x6_min_escape_mps` (0.1 m/s, the trend
  threshold the reference's `_ESCAPE_SPEED_MPS` and the Shield's zone,
  stand-off and corridor-width checks use; its altitude, corridor-band and
  clearance checks accept any gain, and the repairs floor the speed).
  A window with a tick of no depth counts as no progress, so a caller that
  reports none keeps X6 as it was.
- **X7** (Brake / Loiter -> RTL) fires when the position has been illegal
  for T_recover, a hard rule is violated on the tick, the fitted slope over
  the last T_recover less two standard errors is above 0.1 m/s, and the
  fitted rise is at least `x7_min_rise_m` (0.5 m): something the forecast
  cannot see (a wind beyond the speed cap) is beating the recovery, and
  waiting T lets it through (with X6 and X7 off, the 5 m/s tailwind carries
  the aircraft across the 16 m square in about 3.5 s). An unknown depth
  never fires X7.

The first version compared two depth samples (T apart for X6, T_recover
apart for X7) and fired X7 on a rise of 0.2 m. With 0.2-0.3 m of position
noise (the harness's GPS noise; the perception rail's subject range is far
noisier) that sent working recoveries home: 40 of 200 seeded runs at
0.5 m/s and sigma 0.3 m, 176 of 200 at 0.3 m/s (review of 2026-10-09). With
the fit: 0 of 200 at 0.5 and at 0.3 m/s (sigma 0.3 m), 0 of 200 at 0.2 m/s
(sigma 0.2 m), while a 3 m/s push-in still fires X7 at 2.0 s through
0.3 m of noise on every seed. The price is on X6's side: a 0.2 m/s recovery
under 0.2 m of noise is not "shown" working in 5 of 200 runs, and X6 sends
it home. And a recovery whose speed falls off near the boundary (the
Shield's altitude recovery slows as the depth shrinks) can drop under
0.1 m/s before it finishes; X6 then fires if the position is still illegal
after T.

Order of checks with both: **Brake:** direct land, direct RTL, X6, X7, direct
loiter, G3, G2, G7. **Loiter:** direct land / RTL, G4, G8, G5, X6, X7.

### N-in-T: at most one onset per second

GPS noise of 0.2-0.3 m made a forecast skimming `nfz-square`'s ring violate
at 6.1, 6.3 and 6.8 s: three "onsets" in 0.7 s, N-in-T, LOITER, then RTL, on
a pass the scenario labels within the error budget. A hard violation that
starts again less than `onset_clear_s` (1.0 s) after the last COUNTED onset
is now the same violation flickering at the 10 Hz monitor
(`record["onset_merged"]` says so), not a new one.

The first version merged every return after a clean spell shorter than 1 s
into the violation before it, however long the flicker went on. The review
of 2026-10-09 showed what that cost: one-tick hard violations every 0.3, 0.5
or 0.9 s for 30 s never left Brake - the thrash N-in-T (3 in 5 s) exists to
catch, switched off, with theta inert on reading B and a stop legal, so
nothing else escalated it. As a rate limit, the same pattern reaches Loiter
(G2) at 2.4, 2.0 and 3.6 s and RTL (G4) at 6.0, 5.0 and 9.0 s
(`test_n_in_t_still_catches_a_pilot_who_re_violates_faster_than_the_debounce`).
It must leave room for N onsets in T, (N - 1) x `onset_clear_s` <= T, or
N-in-T could never fire; `FSMConfig` refuses a config that breaks that.
`event_mode: tick` ignores it; `onset_clear_s: 0` restores
counting every return. Whether one second is the right separation is a
question for the PI (below).

### The FSM's LOITER is flown as ArduCopter's BRAKE

The grant names the mode LOITER (p2, p5: "SET_MODE GUIDED / LOITER / RTL /
LAND"), and the FSM, the audit record and `set_mode` still say LOITER. What
it asks of the autopilot is to hold the aircraft where it is. ArduCopter's
LOITER does that only with a pilot on the sticks: Copter-4.5.7
`mode_loiter.cpp` sets `target_climb_rate =
get_pilot_desired_climb_rate(channel_throttle->get_control_in())`, and with
no RC input the throttle reads its minimum. The choices a companion computer
has:

| option | holds altitude with no pilot | mode change visible to the GCS | who owns the aircraft |
|---|---|---|---|
| LOITER + throttle centred by RC_CHANNELS_OVERRIDE | yes, while the override is re-sent (it lapses after RC_OVERRIDE_TIME) | yes | the companion, impersonating the pilot; the safety pilot's own throttle is overridden meanwhile |
| GUIDED with a hold setpoint | yes | no: the FSM's Loiter would be invisible in Mission Planner | the companion keeps flying it |
| **BRAKE** | **yes**: `mode_brake.cpp` stops with the position controller and `set_pos_target_z_from_climb_rate_cm(0.0f)`; no timeout unless a Solo button sets one ("full-brake using inertial/GPS system, no pilot input", `mode.h`) | yes | the autopilot |

BRAKE needs a position estimate, as LOITER does. It is what
`sitl/mavlink_adapter_node.py` (`autopilot_mode`, `DEFAULT_HOLD_MODE`), the
ROS node's no-adapter fallback and the Project AirSim node send for the FSM's
LOITER, including the hold a node requests when the FSM refuses a tick;
`--hold-mode LOITER` restores the literal mapping for a flight with a pilot
holding the throttle centred. Both rails record the requests in both
vocabularies (`mode_requests`: the FSM's words; `autopilot_mode_requests`:
ArduCopter's; `ModeRequests`), and a mode change is read as the node's own
only if a request made since the aircraft last came back to GUIDED explains
it, so a pilot's BRAKE after an FSM hold is seen as a takeover.

Measured in ArduCopter SITL (Copter-4.5.7, `sitl/start_sitl.sh` with
`GUARDRAIL_FENCE=0`, no RC input: RC3 read 1000 throughout), take-off in
GUIDED to 15 m, then 20 s in BRAKE, 12 s in LOITER, 8 s in BRAKE (pymavlink,
2026-10-09, a scratch probe, not committed):

| mode | time | altitude |
|---|---|---|
| BRAKE | 19.9 s | 14.97-15.01 m (mean +0.002 m/s) |
| LOITER | 6.7 s | 15.0 m to the ground (mean -2.23 m/s, up to 2.49 m/s down; SITL "Hit ground at 2.38 m/s") |

Not yet flown on the Project AirSim rail.

### Where the hold is (2026-10-19)

BRAKE and LOITER hold where the aircraft is; LAND lands where it is. Since
2026-10-19 the Shield's own FSM asks for the place on every Loiter and Land
entry (`EscalationFSM.needs_hold` / `hold_at`): where the aircraft is when a
stop there is legal through the lookahead, else the nearest point the
Shield's rules find legal from arrival (a landing site over the whole
descent), reached by a transit that never enters a rule the aircraft is
outside of nor goes deeper into one it is in (corrected 2026-10-20); a
Loiter whose point turns illegal is re-planned, at most twice a second. No
edge, guard or setpoint changed. The search, the measurements and what the
rails do not yet fly are in docs/DESIGN-hold-points.md. Robust on the
library: one false fail-safe fewer (26 to 25: `dyn-nfz-rotate-spin` at
9 deg/s), the P0 escape ticks unchanged; the autopilot-flown P0 ticks fall
from 1738 to 1153, but those are the harness's modelled autopilot flying a
moved hold, which no flight rail does yet. (A first count of 26 to 19 held
only because `three-simultaneous-violations` ended 0.2 s before the Loiter
timeout; at 400 ticks G5 sends it home and it stays pinned.)

### Fail-safe trigger correctness, measured again (headless harness, not KPI-grade)

`experiments/sweep_scenarios.py`, labelled episodes, the escalation FSM
entering RTL or Land counted as the trigger. Before = the code at 10451bc
(the figures CHANGELOG 2026-10-07 published). Each cell: correctness (false
triggers / missed triggers).

| configuration | library (186 labelled, 6 expected) | nightly (762, 30) | smoke (56, 4) |
|---|---|---|---|
| 10451bc | 0.747 (47 / 0) | 0.726 (209 / 0) | 0.875 (7 / 0) |
| **as shipped: reading A, onset limit 1 s, X6 / X7 fitted** | **0.823 (33 / 0)** | **0.790 (160 / 0)** | **0.964 (2 / 0)** |
| reading B, the rest as shipped | 0.903 (18 / 0) | 0.921 (60 / 0) | 0.982 (1 / 0) |
| reading A, onset limit off (0 s) | 0.785 (40 / 0) | 0.753 (188 / 0) | 0.946 (3 / 0) |
| reading B, onset limit off | 0.860 (26 / 0) | 0.883 (89 / 0) | 0.964 (2 / 0) |
| reading A, onset limit 1 s, X6 progress and X7 off | 0.785 (40 / 0) | | |
| reading A, onset 0, X6 progress and X7 off (10451bc's FSM, this unit's Shield) | 0.747 (47 / 0) | | |
| the builder's first version, as reported (reading B, merging debounce, two-sample X7) | 0.978 (4 / 0) | 0.980 (15 / 0) | 1.000 (0 / 0) |
| null: never trigger / always trigger | 0.968 / 0.032 | 0.961 / 0.039 | 0.929 / 0.071 |
| Wilson 95 % lower bound, as shipped / reading B | 0.761 / 0.852 | 0.760 / 0.900 | 0.879 / 0.906 |

Read plainly:

- **None meets 0.99, and on the library and the nightly profile nothing
  here beats a Shield that never triggers** (0.968, 0.961), on either
  reading. Only smoke beats it, on the point estimate alone: its Wilson
  lower bound (0.879) is below the null too. The builder's 0.978 rested on
  reading B, which waives theta, and on the merging debounce, which switched
  off N-in-T for thrash faster than once a second; both were reverted.
- What this unit's FSM changes do on reading A: the X6 progress test and X7
  remove 7 false triggers (40 to 33 with the onset limit on; 47 to 40 with
  it off), and the onset limit removes 7 (40 to 33). No missed trigger on
  any configuration.
- Where the 33 left on the library come from, by the first edge after G1
  and the edge into the fail-safe: G3 (theta over, or no operator converged)
  into Loiter, then the 30 s Loiter timeout G5 (8) or N-in-T persisting G4
  (7); X1 / G3 then X6 (7: all six `three-simultaneous-violations` cells
  and one wind gust); X1 / G3 then G5 (3, `dyn-nfz-spawn-ahead`); G2 then
  G5 / G4 / X7 (7); one G7 round trip then G5. By family: sensor
  degradation 9 (eight `gps-noise-beyond-margin` cells with 1-2 m of noise,
  which the library tracks rather than passes), wind 9, dynamic NFZ 6, three
  simultaneous violations 6, high speed near an edge 2, stand-off 1.
- 16 of the 33 end by the Loiter timeout (G5). In the two checked
  (`wind-within-authority` lane 15 at 1 m/s, `dyn-nfz-spawn-ahead` 3 m at
  4 m/s) the harness's pilot kept asking for the action through the zone on
  every one of its 300 Loiter ticks while the modelled LOITER held the
  aircraft, so G8's hard-clean T_recover never came. Whether "cleared for
  T_recover" in Loiter should judge the pilot's request (which nobody flies
  while the autopilot holds) or the position is a question for the PI
  (below); so are the labels of the noise and wind cells and the harness's
  open-loop wind. None of it is tuned here.

None of this is KPI-grade: the headless harness is a kinematic model, and
the grant's KPI is measured on the hil topology (the Orin), which is not
built yet. Reproduce: `python experiments/sweep_scenarios.py [--profile
smoke|nightly] --out <file> --no-bundles`; the other rows patch
`Shield(theta_horizon_s=...)` and `FSMConfig(onset_clear_s=...,
x6_min_escape_mps=1e9)` around the same entry point.

## The gap, stated exactly

The grant's Safety Shield repairs an unsafe action when the repair is small,
and steps up through fail-safes when it is not. The second half never existed
here. `Shield.filter()` has one fallback, a zero-velocity BRAKE. No code
changes the ArduPilot flight mode during a mission. The only `SET_MODE` calls
are GUIDED at bring-up and LAND at mission end, and neither is the Shield's.
`brakes = 0` on all five KPI runs, so "fail-safe trigger correctness" in
`guardrail/kpi.py` was one minus the escape rate, a number that reads 1.0 for a
Shield with no fail-safe at all.

## What the grant asks for

Safety Shield PDF p4, Escalation FSM. The mermaid source is
`kuanting-vla-uav-guardrail/docs/02-implementation/safety-shield.md`:

```
Normal --> Brake   : violation, repairable, magnitude < θ
Brake  --> Loiter  : N-in-T threshold OR magnitude > θ
Loiter --> RTL     : N-in-T persists OR operator-defined timeout
RTL    --> Land    : RTL fails (battery / blocked path)
Brake  --> Normal  : cleared for ≥ T_recover
Loiter --> Normal  : cleared for ≥ T_recover
RTL    --> [*]     : home reached
Land   --> [*]     : landed
Normal --> RTL     : violation_action == RTL (direct), unrepairable
Normal --> Land    : violation_action == Land (direct), unrepairable
```

**Defaults:** N = 3, T = 5 s, θ = 2.0 m lateral / 0.5 m vertical,
T_recover = 2 s, "config-file overridable per mission profile" (p4). The
Outputs table (p7) asks for a "Python state machine; YAML-overridable
thresholds".

Related grant text this module follows:

- p1: "detect violations → repair via projection → trigger fail-safe (RTL /
  Land) when repair is unsafe".
- p2, node diagram: "Converged? magnitude < θ?" → *no* → "Escalation FSM
  (Brake → Loiter → RTL → Land)" → "Mode setter SET_MODE
  GUIDED/LOITER/RTL/LAND".
- p3–p4: the first operator "with magnitude ≤ threshold wins". Conservative
  cap: "If ‖action_repaired − action_original‖ > threshold, or no operator
  converges, the Shield abandons projection and triggers fail-safe."
- p5, audit record: `fsm_state_before`, `fsm_state_after`, and
  `repair_attempts[].magnitude_m`.
- Policy DSL PDF p2: `constraint_type: hard | soft`, `priority: P0 | P1 | P2`,
  `violation_action: monitor_only | project_fix | brake | loiter | RTL | land`.
- Prefix Compiler PDF p4: "severity: hard ≫ soft, P0 > P1 > P2".
- Stress Testing PDF p5–p6: outcome ∈ {success, fail, RTL_triggered,
  Land_triggered}. Fail-safe trigger correctness ≥ 99 %, "triggered when
  expected, not when not expected".
- Grant overview p1, WP3 row: "ArduPilot fence / RTL / Land / Loiter as
  ultimate backstop".

## States, and what each asks of ArduPilot

| State | ArduPilot mode | Setpoint streamed | Meaning |
|---|---|---|---|
| Normal | GUIDED | `pass` (the Shield's emitted action, i.e. the raw one when nothing is violated) | nothing enforced is violated |
| Brake | GUIDED | `pass` (the repaired action) when the repair is trusted; `brake` (zero velocity) when it is not, or when a rule asks for a stop. Exception: `pass` when a stop is illegal here (`stop_illegal`) | the Shield is intervening |
| Loiter | LOITER, flown as ArduCopter's BRAKE since 2026-10-08 (see "The FSM's LOITER is flown as ArduCopter's BRAKE") | `none` | the autopilot holds position |
| RTL | RTL | `none` | the autopilot flies home |
| Land | LAND | `none` | the autopilot lands where it is |

Brake is **not** an ArduPilot mode. It is the Shield intervening inside
GUIDED. A trusted repair is still flown in Brake, which matches the PI's
reference (`safety_shield/shield.py`: state BRAKE, emitted = the repaired
action). On the tick that hands the aircraft from GUIDED to an autopilot mode,
the setpoint is `brake`, so the last GUIDED setpoint is not still flying while
SET_MODE goes through. After that tick it is `none`. `set_mode` is
edge-triggered: it is sent once, on the transition, never on every tick.

**A stop the Shield calls illegal is never streamed.** `stop_illegal` means
standing still here would break a **hard, enforced** rule, as it does inside
a no-fly polygon, a clearance ring, or under the altitude floor. The node
passes `Shield.state_is_unsafe(state)` (the list of violations) and
`tick_input_from_decision` keeps only the hard, enforced rules. A soft rule is
capped at `brake`, so a stop that breaks only a soft rule is a response the
soft rule already allows. On such a tick every `brake` above becomes `pass`,
which streams the Shield's own action. For a blocked tick that is the
Shield's BRAKE where a stop is legal and its best-effort recovery where it is
not, because shield.py brakes only "where standing still is legal". The state
still escalates exactly as the table says. Only the setpoint changes, and the
record carries `stop_withheld: true`. The reference calls the alternative,
stopping inside the zone, the "brake while inside = deadlock" failure.

## Transition table

`G` edges are the grant's. `X` edges fill gaps where the grant is silent. Each
`X` edge is labelled `extension` in the audit record, so a choice of ours is
never presented as a contract term. One transition per tick at most.

| Edge | From → To | Condition | set_mode | Source |
|---|---|---|---|---|
| G1 | Normal → Brake | enforced violation, repair converged, magnitude ≤ θ | – | grant p4 |
| G2 | Brake → Loiter | ≥ N hard-rule onsets within the last T s | LOITER | grant p4 |
| G3 | Brake → Loiter | a hard, non-kinematic `project_fix` rule's repair is over θ, or did not converge - or, on a tick that violates no non-kinematic `project_fix` rule, the repair the chain made for a hard kinematic rule (`untrusted_owner` "chain", since 2026-11-07); not on a tick whose strongest action is `brake` where a stop is legal (the Shield skipped projection, so that rule was never projected; since 2026-10-19) | LOITER | grant p4 (diagram + conservative cap) |
| G4 | Loiter → RTL | ≥ N new hard onsets within T, counted from Loiter entry | RTL | grant p4 |
| G5 | Loiter → RTL | time in Loiter ≥ `loiter_timeout_s` | RTL | grant p4 ("operator-defined timeout") |
| G6 | RTL → Land | autopilot side reports `rtl_failed` | LAND | grant p4 |
| G7 | Brake → Normal | nothing enforced violated for ≥ T_recover | – | grant p4 |
| G8 | Loiter → Normal | no **hard** enforced violation for ≥ T_recover (since 2026-11-02, `loiter_clear` "acceptable": the request one the Shield would fly and the position legal; since 2026-11-07, `hold_horizon` "loiter": and its flight carries the aircraft onto no spot where a stop is illegal within the lookahead, `handback_unsafe`) | GUIDED | grant p4 |
| G9 | RTL → [*] | `home_reached` | – | grant p4 |
| G10 | Land → [*] | `landed` | – | grant p4 |
| G11 | Normal → RTL | a violated rule's effective action is RTL | RTL | grant p4, with "unrepairable" read as "skips projection" (see Interpretations) |
| G12 | Normal → Land | a violated rule's effective action is land | LAND | grant p4, same reading |
| X1 | Normal → Brake | enforced violation whose repair is over θ or did not converge, or a `brake` rule; setpoint `brake` (`pass` if `stop_illegal`) | – | extension: p2 routes "not converged" into the chain, whose first state is Brake |
| X2 | Brake, Loiter → RTL | a violated rule's effective action is RTL | RTL | extension: a P0 RTL rule cannot be ignored because a P2 repair put the FSM in Brake one tick earlier |
| X3 | Brake, Loiter → Land | a violated rule's effective action is land | LAND | extension, same reason |
| X4 | Normal, Brake → Loiter | a violated rule's effective action is loiter | LOITER | extension: the DSL allows `loiter`; the diagram never says what it does |
| X5 | RTL → [*] | `landed` during RTL without `home_reached` | – | extension: ArduPilot's RTL ends by landing at home |
| X6 | Brake, Loiter → RTL | `stop_illegal` on every tick for ≥ T, a hard rule violated on this tick, and (since 2026-10-08) the illegal depth not falling at `x6_min_escape_mps` over that T | RTL | extension: p1 "trigger fail-safe when repair is unsafe". See "X6" below |
| X7 | Brake, Loiter → RTL | `stop_illegal` for ≥ T_recover, a hard rule violated on this tick, and the illegal depth GROWING by ≥ `x6_min_escape_mps` over the last T_recover (since 2026-10-08) | RTL | extension: a recovery losing ground to something the forecast cannot see |
| X8 | Normal, Brake, Loiter → RTL | `position_lost` (since 2026-11-06): the handed position's error (with `fix_ok` False: the dead-reckoned distance from the last certified position plus `dr_drift_mps` × its age; with `pos_sigma_m` reported: at least 3 σ, at the position and along the request's forecast) exceeds a hard, enforced P0 zone's `margin_m` while that zone's boundary lies within the error. Since 2026-11-12, where the rail reports the autopilot's own verdict (`estimator_ok`), it decides first: True certifies the position (no dead reckoning, GPS or not), False is X8 at once if any hard, enforced P0 zone is in force. Never taken on a rail that reports none of the three | RTL | extension: rubric E3 (a fail-safe when the position the Shield acts on is wrong by more than the margin near a hard P0 boundary); see "X8" above |
| X9 | Loiter → RTL | `hold_off_m` > `hold_reach_m` on every tick for ≥ T_recover, and `hold_off_m` GROWING over the last T_recover (X7's test: slope above `x6_min_escape_mps` at two standard errors, rise ≥ `x7_min_rise_m`) (since 2026-11-11, `hold_kept` "reach"). Never taken on a rail that does not pass the hold's distance | RTL | extension: rubric E2 (a disturbance the aircraft cannot reject) where the position stays legal; see "X9" above |

The order of checks inside each state is fixed, and it is the order the table
is resolved in when two conditions hold at once. It follows the strongest
response. **Normal, Brake and Loiter:** X8 first (since 2026-11-06), because
every later answer was computed from the position X8 can no longer certify.
**Brake:** direct land, direct RTL, X6, X7, direct loiter, then G3,
G2, G7. **Loiter:** direct land/RTL, then G4, X9 (since 2026-11-11), G8,
G5, X6, X7. Recovery wins over the timeout when both hold on the same tick;
a hold not kept wins over a hand-back. **RTL:** G9, X5, G6, so a tick
that reports home together with an RTL failure is a finished RTL. In RTL and
Land the pilot's proposals do not move the FSM. The autopilot is flying, and
the grant draws no edge out of RTL for them.

### X6: a position that stays illegal

The first review round found that one case could never escalate. An aircraft
held inside a P0 zone (a wind beyond the 4 m/s speed cap, say) gets a
`GeofenceEscape` every tick. The escape re-checks clean, so the tick is a
trusted repair (G1). Recovery operators are exempt from θ, so G3 cannot fire.
The violation never clears, so it is one N-in-T onset, and G2 cannot fire.
Measured on the real Shield with `sim_demo_policy`: an aircraft held 3 m
inside `nfz-square` for 600 ticks stayed in Brake with setpoint `pass` and
`failsafe_triggered = False` for all 60 s, under both magnitude sources. That
breaks p1, and `experiments/scenarios.yaml`'s `wind-beyond-authority` expects
`RTL_triggered` for exactly this.

X6 sends it to RTL when the position has been illegal on every tick for T
(the grant's own persistence window, 5 s), while the Shield is still
intervening on a hard rule. The same 600-tick run now gives G1 at t = 0 and
X6 at t = 5.0 s, RTL_triggered.

- **Why not count such ticks as N-in-T events?** At 10 Hz, three of them put
  the aircraft in Loiter 0.3 s after any incursion, and three more in RTL.
  Every GeofenceEscape that was about to succeed would be aborted. X6 gives a
  recovery the same 5 s the grant gives N-in-T.
- **Why skip Loiter?** LOITER holds position, and holding position where a
  stop is illegal keeps the aircraft in the violation: the reference's
  "brake while inside = deadlock". If the FSM is already in Loiter there (G3
  on an unconverged repair), X6 takes it to RTL T after the position became
  illegal, instead of after the 30 s timeout.
- **Why require a hard violation on the tick?** A tick on which the raw
  action is already escaping (clean) is the pilot leaving by itself. The
  clock keeps running over such ticks, but X6 fires only on a tick where the
  Shield is intervening.

## How a rule's fields become a response

The grant defines `violation_action`, `constraint_type` and `priority`. It
never says what the Shield does with the last two. `RuleHit.effective_action`
is the single place this is decided. shield.py should call it, so the repair
stack and the FSM cannot disagree.

| violation_action | hard rule | soft rule |
|---|---|---|
| `monitor_only` | recorded; no repair, no state change, no N-in-T | same |
| `project_fix` (alias `repair`) | repair; G1 if trusted, else X1, then G3 | repair; Brake at most |
| `brake` | X1 into Brake, setpoint `brake`; never G3 | same; Brake at most |
| `loiter` | X4 | **capped at `brake`**, flagged `capped: true` |
| `RTL` | G11 / X2 | **capped at `brake`**, flagged |
| `land` | G12 / X3 | **capped at `brake`**, flagged |

- **Soft = enforced, capped at `brake`.** A soft rule never changes the flight
  mode, never counts toward N-in-T, and never holds the FSM in Loiter (G8 looks
  at hard rules only). Without that last rule, a soft violation could reach RTL
  through the Loiter timeout. `brake` is the strongest action the grant itself
  gives a soft rule (`envelope-default`, Policy DSL PDF p4), so a soft `brake`
  rule is obeyed, not flagged as capped. The audit card suggested "soft =
  monitor only". That would contradict the grant's own example, so it was
  not used.
- **Priority never lightens the response.** The response is the strongest
  effective action among the enforced violations. If priority chose it, adding
  a P0 `project_fix` violation to a P2 `land` would turn a landing into a
  repair. Priority orders the record (hard before soft, then P0, P1, P2) and
  breaks ties: with equal actions, the P0 rule is cited, whatever order the
  Shield listed violations in. The governing rule's priority is written as
  `risk_level`, the grant's auto-label (Stress Testing PDF p5).
- **Kinematic rules are not cited for θ.** A kinematic envelope's repairs are
  clamps: they always converge and θ never judges them. `RuleHit.kinematic`
  (set from the policy's `kinematic_envelope` type, or from a `kinematic`
  violation category when there is no policy) keeps such a rule from making a
  tick untrusted, and an untrusted projection is attributed to a rule whose
  repair θ does judge. Before, with every rule unresolved and P0, the replay
  cited `kin-caps` for every θ escalation, because it sorts first by id.
  Where no such rule is violated (since 2026-11-07, `untrusted_owner`
  "chain"), the untrusted repair is the one the chain made for the kinematic
  rule - an operator acting on its own test for a rule the request does not
  break - and G3 escalates it when that rule is hard ("2026-11-07: the FSM
  on the rail vehicle").
- **Action names are case-insensitive.** The grant spells the same action
  `land` (DSL literal) and `Land` (FSM diagram). Unknown names are refused, not
  defaulted.
- A violated id that is not in the policy becomes hard / P0 / `project_fix`
  with `resolved: false`. In practice this means the Shield's own
  `action-finite` and `subject-finite` contract checks. `guardrail/kpi.py`
  makes the same "unknown means P0" choice.
- **A subject the Shield cannot place** (`subject-finite`, since
  2026-10-19). `Shield.set_subject` drops a NaN or inf position, and while a
  binding stand-off is hard and enforced each tick carries `subject-finite`
  (answered by `Sanitise`, so the tick is `repaired`, with nothing for θ to
  judge). Read as above, the FSM enters Brake by G1 on the first one and
  streams the Shield's action (a trusted repair is never stopped), and a
  persistent NaN holds Brake until a finite subject (or `None`) has been
  clean for T_recover (G7). A NaN that flickers is a new onset each time it
  returns, so it climbs as that hard rule flickering in and out of breach
  would: 0.5 s NaN / 0.5 s finite gives G2 (Loiter) at 2.0 s and G4 (RTL) at
  5.0 s. With only soft or `monitor_only` stand-offs binding nothing is
  raised (the drop is counted, and rule_status reads the rule unbound), so it
  is never a stronger response than a real breach of the rule: the first
  version raised it for those too, and a NaN beside a `monitor_only`
  stand-off went Normal -> Brake where a breach of it stays in Normal.
  `tests/test_shield.py::test_a_non_finite_subject_escalates_no_harder_than_the_rule`.

## Interpretations where the grant is silent

| Term | Chosen reading | Why |
|---|---|---|
| "N-in-T" | counts hard-rule **onsets** (violated after a tick with none), not violated ticks. `event_mode: tick` is kept as a knob | At 10 Hz, counting ticks escalates any violation that lasts 0.3 s, so every sustained repair, such as sliding along a fence, would put the aircraft in LOITER. "3 in 5 s" reads as three separate violations, the thrash of a pilot fighting the Shield. The grant leaves the defaults to "stress-run data" (p7). The one case onsets cannot see, a position that stays illegal, is X6's |
| an onset (since 2026-10-08) | a hard violation after a tick with none, at most one per `onset_clear_s` (1.0 s) counted: a rate limit, not a merge | a violation back within 1 s of the last counted one is the same one flickering at the 10 Hz monitor (GPS noise on a forecast skimming a ring), while a pilot who keeps re-violating still makes N onsets in T; (N - 1) x `onset_clear_s` must fit in T (validated), or G2 and G4 could never fire. See "N-in-T: at most one onset per second" |
| "N-in-T persists" | N **new** onsets within T after entering Loiter (the window is cleared on entry) | otherwise the onsets that caused the Loiter fire G4 on the next tick |
| window edges | inclusive: an onset exactly T old counts. Floating slack 1e-9 s | the more conservative reading; fifty additions of 0.1 s give 4.999999999999998 s |
| "cleared for ≥ T_recover" | from the **first clean tick** | the later of the two possible starts, so recovery is never early |
| "unrepairable" (G11, G12) | a rule whose action is RTL or land **skips projection**: G11/G12 fire even on a tick whose repair would have converged | the reference's shield.py: "Rules whose action is a direct fail-safe skip projection entirely". Repairing first and going home only if the repair fails would let a rule written to send the aircraft home be satisfied by a nudge |
| θ comparison | exactly θ is accepted; escalate on > θ | p3 "magnitude ≤ threshold wins", p4 "> threshold". The diagram's "< θ" is the only exception |
| θ per axis | escalate if lateral > 2.0 m **or** vertical > 0.5 m | the two are separate thresholds in p4 |
| θ over several operators | the per-axis **sum** of the operators that fired this tick | p4 caps the total change, ‖repaired − original‖, and the sum bounds it from above. The reference checks each attempt on its own (see "Where this differs from the PI's reference code") |
| which repairs θ judges | position operators only. Exempt under **both** magnitude sources: kinematic clamps, `Sanitise`, `Brake`, and **recovery** operators: `GeofenceEscape` and `StandoffRecover` by name, a `ClearanceEscape` only where a stop is illegal, and any repair flagged `recovery: true` | the reference exempts recovery operators: capping them "would re-introduce the 'brake while inside = deadlock' failure". An aircraft inside a zone needs a correction as large as its penetration to get out. An operator name the module has never seen is judged, not exempt |
| a rescue where a stop is legal | `ClearanceEscape` that re-checked clean but `stop_illegal` is false: **not converged** (outcome `blocked`), the legal stop is streamed, and the FSM escalates | shield.py runs it when "repair not converged", which can happen with BRAKE legal (a predicted clearance breach). That is p4's "no operator converges". The reference exempts only GeofenceEscape, which runs only for an aircraft already inside, and `guardrail/kpi.py` counts rescues apart (`converged_via_rescue`) |
| `stop_illegal` | standing still here breaks a **hard, enforced** rule | soft rules are capped at `brake`, so a stop that breaks only a soft rule is allowed |
| a stop where stopping is illegal | never streamed (see States) | standing still inside a no-fly polygon or a clearance ring keeps the aircraft in the violation |
| a position illegal for T | X6 → RTL | see "X6" |
| operator timeout | `loiter_timeout_s = 30`, `null` disables it; must exceed T_recover | no grant default. A timeout ≤ T_recover would make every Loiter an RTL |
| fail-safe "triggered" | entering RTL or Land | the outcome vocabulary has RTL_triggered and Land_triggered, and no Loiter_triggered. `summary()` reports `loiter_entered` separately. (The Grant overview lists Loiter among the "ultimate backstop" modes; PI question 5) |

## Where this differs from the PI's reference code

The reference (`kuanting-vla-uav-guardrail/packages/safety-shield/src/safety_shield/`)
ships a Phase-1 slice. Its own docstring says: "The full FSM (Brake -> Loiter
-> RTL -> Land with N-in-T thresholds and recovery) lands in Phase 2." This
module is that Phase 2, built from the grant's diagram, which is also the
reference's own design page. Where the slice and this module disagree:

| Behaviour | Reference slice (`fsm.py`, `shield.py`, `repair.py`) | This module | Why |
|---|---|---|---|
| States | Normal, Brake, RTL, Land | adds Loiter | the grant's diagram has five |
| Back to Normal | on the first clean tick, from any state, RTL and Land included | only from Brake and Loiter, after ≥ T_recover clean | grant edges G7 and G8. Nothing leaves RTL or Land except [*] |
| Unrepairable, no direct action | straight to RTL ("conservative RTL") | X1 into Brake, then G3 into Loiter on the next untrusted tick, then RTL by G4, G5 or X6 | the grant's diagram steps Brake → Loiter → RTL. The slice skips two steps |
| A position that stays illegal | not handled (the slice has no N-in-T) | X6 → RTL after T | p1; without it the case never escalates (see "X6") |
| Which rule decides | `violations[0]`, the earliest predicted hit (`checker.py` sorts by `first_hit_s`). Priority and hard/soft are unused | the strongest effective action among enforced violations. Priority breaks ties, soft is capped at `brake`, and a θ escalation is cited to a non-kinematic rule | a `land` rule hit 2 s ahead should not be repaired away because a `project_fix` rule was hit 1 s ahead. The grant is silent |
| Direct actions | `RTL` and `land`, exact spelling | also `loiter` (X4) and `brake`, case-insensitive | the DSL's six actions (Policy DSL p2). The grant spells both `land` and `Land` |
| θ check | inside the repair loop, per attempt, against the target's axis (`repair.py` `repair_action`). An over-cap attempt means "not converged" | in the FSM, per-axis sum over the tick's operators | p4 caps the total change. Both give the same edge |
| Recovery operators | `GeofenceEscape` exempt from θ | `GeofenceEscape` and `StandoffRecover` exempt; `ClearanceEscape` only where a stop is illegal | the reference's "brake while inside = deadlock"; this repo has two more inside-recoveries and a rescue the reference does not |
| Setpoint on a direct fail-safe tick | the raw action | `brake` on the hand-over tick, or the Shield's action if a stop is illegal | the last GUIDED setpoint should not keep flying while SET_MODE goes through |

## The θ decision, measured — the one that matters most

The grant states θ in **metres** of correction. Our actions are velocities.
How a velocity repair becomes metres decides whether the cap ever fires, and
it is not a detail. The module offers two sources and picks neither silently.
`tick_input_from_decision` uses the per-operator source unless `horizon_s` is
given, and that source raises while shield.py lacks the field. So no flight
can run on a default nobody chose:

1. **Per-operator `magnitude_m`** (`magnitude_from_repairs`), the grant's own
   audit field (p5). The PI's reference fills it with the position correction:
   penetration depth for lateral projection, altitude error for the altitude
   clamp. Any operator θ governs (`theta_governs`) and that has no
   `magnitude_m` is refused. **shield.py's `Repair` has no such field today.**
2. **Velocity proxy** (`magnitude_from_actions`): |Δv| × h, the gap after h
   seconds between where the clamped raw action and the repaired one would
   put the aircraft. h has no default.

Both go through `repair_magnitude`, and both apply the same exemptions. A
tick whose repairs are all exempt is 0 m under either source.

**The proxy measures from the clamped raw action.** shield.py runs the
kinematic clamps first, on the raw action, and the position operators work on
the clamped result (`_decide`: `fixed = self._repair_kinematic(raw, repairs)`
before the clearance / standoff / corridor / geofence loop). `KinematicCaps`
reproduces SpeedClamp and ClimbClamp exactly, and the proxy measures the
repaired action against `caps.clamp(raw)`. `KinematicCaps.from_policy` reads
the caps, so the node needs nothing extra. On a tick that has a SpeedClamp or
ClimbClamp and no caps, the proxy is **refused**. A policy whose caps
contradict the decision (a raw action over the caps and no clamp fired) is
refused too, because it is the wrong policy for that run.

### Retracted: the first version of this table

The first version of this section, published in this same file earlier on
2026-10-06, measured the proxy from the **unclamped** raw action. Every one of
the 1763 θ-governed ticks also carried a SpeedClamp (973 StandoffHold +
SpeedClamp, 562 GeofenceSlide + SpeedClamp, 148 StandoffHold + SpeedClamp +
ClimbClamp, 51 AltitudeFix + SpeedClamp + ClimbClamp, 16 of those with a
StandoffRecover and 13 with a StandoffHold), so most of what that table
measured was the clamp's Δv, which this module says θ never judges. What it said, and what is right:

| Claim in the first version | Correct value | What was wrong |
|---|---|---|
| h = 0.1 s: 91 / 1763 governed ticks over θ; 1 / 7 runs escalates (`sitl_ped_on`, vertical, t = 98.5 s) | 0 / 1763; 0 / 7 runs | the one escalation (tick 915) is a ClimbClamp 7.04 → 2.00 m/s alone: 5.04 m/s × 0.1 s = 0.504 m. The only position operator on that tick is StandoffHold, which is lateral |
| h ≥ 0.5 s: 1763 / 1763 over θ at every horizon | 91 / 1763 at 0.5 s; 1696 / 1763 at 1.0 s | clamp Δv counted |
| ROS runs: θ first exceeded between h = 0.277 s and 0.494 s | between 0.354 s and 0.803 s | clamp Δv counted |
| median lateral \|Δv\| on governed ticks 6.0 m/s, minimum 2.0 | 3.99 m/s, minimum 0.00 (49 governed ticks change nothing after the clamp) | 5.98 / 2.0 is the clamp-included figure |
| "642 carry speed clamps only" | 629 clamps only, plus 13 clamps with a StandoffRecover | the 13 are exempt through the recovery, not as clamps |
| "29 of the 1763 also contain a StandoffRecover" | 16 of the 1763 | 29 is the count over all 2405 repaired ticks |
| "the two sources can differ only in how the metres are measured, never in which repairs θ applies to" | true only since the clamp-free proxy | on these runs the old proxy disagreed on 100 % of governed ticks |

The fix in the first review round (sharing the exemptions) covered pure-clamp
ticks only. Mixed ticks were still measured with the clamp in.

### The table, clamp-free

What option 2 would have done to the seven delivered shield-ON flights. This
is a read-only replay of their `flight_log.jsonl` through the FSM, with the
policy each run flew (`--policy`): `sitl_pedestrian.yaml` for the three
`*_ped_*` runs and `sim_demo_policy.yaml` for the four `*_shield_on*` runs.
Both have a 4 m/s speed cap and a 2 m/s climb cap. The replay checks the
loaded policy's hash against the run's manifest: five match. The two
`_dynamic` runs do not, because a hot-applied NFZ bumped the policy
generation in flight, so that fence's id resolves as unknown (hard, P0). The
replay is counterfactual after the first escalation, because the real flight
never escalated, so only the first escalation is reported.

Of the 2405 repaired ticks, 642 are never judged by θ: **629 carry clamps
only**, and 13 carry clamps with a StandoffRecover. The other **1763** contain
a position operator (`GeofenceSlide`, `StandoffHold`, `AltitudeFix`), **every
one of them together with a SpeedClamp**, and 16 of them also with a
StandoffRecover. 408 of the 1763 are in the three ROS runs.

| h | θ-governed ticks over θ, all 7 runs | ROS runs | runs that would escalate | first escalation |
|---|---|---|---|---|
| 0.1 s (one monitor tick) | 0 / 1763 (0 %) | 0 / 408 (0 %) | 0 / 7 | – |
| 0.5 s (the Shield's `dt`) | 91 / 1763 (5.2 %) | 59 / 408 (14.5 %) | 4 / 7 (the four `sim_demo_policy` runs) | tick 2 (t = 0.20–0.24 s), G3, `nfz-square`, lateral 2.83 m |
| 1.0 s | 1696 / 1763 (96.2 %) | 408 / 408 | 7 / 7 | tick 2 (5 runs), tick 15 (`ros2_ped_on`, `sitl_ped_side_on`), all G3 |
| 3.0 s (the Shield's lookahead) | 1699 / 1763 (96.4 %) | 408 / 408 | 7 / 7 | same |
| 5.0 s (the grant's horizon) | 1701 / 1763 (96.5 %) | 408 / 408 | 7 / 7 | same |

Every escalation now cites the rule whose repair broke θ (`nfz-square`,
`standoff-pedestrian`, `standoff-any`), not `kin-caps`. The two runs that
first escalate on tick 15 spend their first 14 repaired ticks on clamps alone,
before the standoff ring comes into the lookahead.

Runs: ros2_ped_on, ros2_shield_on, ros2_shield_on_dynamic, sitl_ped_on,
sitl_ped_side_on, sitl_shield_on, sitl_shield_on_dynamic. All have `brakes = 0`.
(Note added 2026-10-21: the four `sitl_*` runs were flown by the pymavlink
SITL rail before 79bae12, 2026-10-19, on a delayed pose - its OLDEST queued
LOCAL_POSITION_NED, 3-6.6 s behind on `sitl_ped_on`; see the X6 note below.
Their repaired ticks and θ counts describe that rail as it flew, not the
aircraft's true position. The three `ros2_*` runs came over MAVROS.)
Reproduce with (one line per run; change `--horizon` for the other rows):

```
python -m guardrail.fsm replay demo/out/ros2_ped_on demo/out/sitl_ped_on \
    demo/out/sitl_ped_side_on --policy policies/sitl_pedestrian.yaml --horizon 1.0
python -m guardrail.fsm replay demo/out/ros2_shield_on demo/out/ros2_shield_on_dynamic \
    demo/out/sitl_shield_on demo/out/sitl_shield_on_dynamic \
    --policy policies/sim_demo_policy.yaml --horizon 1.0
```

Without `--policy` the replay of these runs exits 2 and says why: every
governed tick has a clamp, and the proxy needs the caps.

**The proxy is still close to a switch.** On the ROS runs, the horizon at
which a position repair first exceeds θ lies between 0.354 s and 0.803 s for
every tick (median 0.539 s). Below 0.354 s the cap never fires on any of the
seven runs; at 1 s it fires on 96 % of position repairs. The stub flies
straight at the zone or the person and keeps asking to, so the clamped
command is 4 m/s and a position operator removes most of it every tick:
median lateral |Δv| on the governed ticks is 3.99 m/s, measured from the
clamped raw action.

Read plainly: with a one-tick horizon the cap is **inert**, because no
delivered flight would ever exceed it. That is a check that cannot fail. With
any horizon from 1 s up, almost every stub flight goes to LOITER as soon as a
position operator fires. The grant may well intend the second outcome: its
reference escalates "a deep head-on dive … rather than projected". But it
would change what the KPI runs show. **Recommendation (2026-10-06, still
in force):** option 1. It is the grant's schema and the PI's reference
metric, and it needs shield.py to report `magnitude_m` and `axis` per
position operator (see Integration; done 2026-10-07). Any run that uses
option 2 must state its h beside every trigger count (`fsm.theta_basis` in
metrics.json). This is a question for the PI (below).

On 2026-10-08 a builder switched the default to option 2 at h = 0.1 s and
called it the grant's basis; the review of 2026-10-09 reverted it ("Theta:
two readings, one default" at the top has why and both readings'
numbers).

### N-in-T and X6 on the same flights

Each delivered run holds **one** violation that, once it starts, never clears
until the log ends: 1 onset per run. In onset mode N-in-T can never fire on
them. In tick mode, every one of them would loiter after three ticks
(t = 0.30–0.34 s). These flights cannot exercise N-in-T or recovery. The
labelled stress episodes have to (see "The KPI").

The delivered logs do not record `stop_illegal`, so the plain replay cannot
see X6 (`rows_recording_stop_illegal: 0`). `tools/rescore_kpis.py`
reconstructs it per row (`unsafe_rules`, from `Shield.state_is_unsafe`), and
the replay reads that field when present. With it reconstructed, at
h = 0.1 s, X6 fires on **1 of 7** runs: `sitl_ped_on`, tick 943,
t = 101.5 s. That run is under the 10 m altitude floor (`alt-band`, hard P0)
on 216 ticks in three episodes, the longest 10.5 s unbroken (96.5–107.0 s),
at about 2 m altitude, while the Shield was emitting the +2.0 m/s climb cap
on every tick. A repair that has not lifted the aircraft back over the floor
in 5 s is the case X6 exists for. The other six runs have no illegal
position. (Why the aircraft kept sinking under a +2 m/s climb command is a
question about the pymavlink SITL rail, not about this module. The run's
own `metrics.json` already reports `alt_violation_s: 21.6`. Answered on
2026-10-19: until then the pymavlink rail read pymavlink's oldest queued
LOCAL_POSITION_NED each tick instead of the newest, so on `sitl_ped_on` its
pose fell about 3.2 s behind at 33-49 s and 6.6 s behind at 97-113 s, where
the stub's altitude loop swung on the stale position; see
`MavlinkAdapter.state` in `sitl/run_sitl_demo.py` and
`tests/test_sitl_rails.py`. Every stored `sitl_*` run named in this note was
measured on such a delayed pose, so the X6 escalation above, and these runs'
dwell and θ counts, describe that rail as it was, not the aircraft's true
position.) The
reconstruction is a measurement, not a replay option. To repeat it: load the
rows and `metrics.json`, call `tools/rescore_kpis.py`
`reconstruct_unsafe(rows, policy, metrics)`, then
`guardrail.fsm.replay_rows(rows, horizon_s=0.1, policy=policy)`.

## Inputs, outputs, audit record

**Per tick in:** `TickInput(t, outcome, violations, magnitude, rtl_failed,
home_reached, landed, note, stop_illegal)`

- `t` is monotonic seconds. Repeats are allowed, and going backwards raises.
- `outcome` ∈ `clean | repaired | blocked`. `blocked` = the Shield braked, or
  its own re-check of what it emitted is not clean, or it rescued a
  non-converged chain with ClearanceEscape where a stop was legal.
- `violations` is every rule the raw action broke, as `RuleHit`s, including
  monitor_only and soft rules.
- `magnitude` is required when `repaired`. A repair of unknown size is never
  waved through as a small one.
- `stop_illegal`: give `tick_input_from_decision` the list
  `Shield.state_is_unsafe(state)` returns (it keeps only hard, enforced
  rules), or a bool. Left unset, it is read from a blocked decision (the
  Shield did not brake, or its brake still re-checks dirty) and taken as
  false on a repaired one. A ClearanceEscape that re-checked clean is the
  exception: whether it is a recovery or an unconverged repair depends on
  exactly this flag, so unset is refused there.
- `unsafe_depth_m` (since 2026-10-08): `Shield.unsafe_depth(state)`, metres
  from legal, or None. X6's progress test and X7 read its trend; None is no
  progress for X6 and never fires X7. Validated finite and >= 0.

Inconsistent inputs raise: `clean` with an enforced violation, `repaired` /
`blocked` with none, NaN anything.

**Per tick out:** `FSMOutput(state, before, transition, edge, set_mode,
setpoint, reason, terminal, record)`.

**Audit record** (a plain dict, JSON-serialisable; WP3-11 will wrap it in the
Pydantic model the grant asks for): `fsm_tick, t_s, fsm_state_before,
fsm_state_after, transition, edge, edge_source (grant | extension), terminal,
set_mode, setpoint, reason, outcome, governing_rule, risk_level, violations
(sorted, each with effective_action / capped / resolved / kinematic),
magnitude (with recovery_exempt), theta, theta_exceeded, n_in_t, n_threshold,
window_s, cleared_s, cleared_hard_s, stop_illegal_s, unsafe_depth_m,
x6_progress, x7_losing, flags (incl. stop_illegal), stop_withheld, note,
fsm_config_hash`; on the ticks that carry them only, `position_lost`,
`noise_marginal`, `untrusted_chain` (since 2026-11-07: the kinematic rules
whose chain owns an untrusted repair), `handback_unsafe` (since 2026-11-07:
Loiter kept a request whose flight would carry the aircraft onto a spot where
a stop is illegal), `cleared_loiter_s` and the hold point's fields.

`fsm_config_hash` is the full SHA-256 of the thresholds in force, over
canonical float values, so a trigger count can be traced to the N / T / θ
that produced it and `T: 5` hashes the same as `T: 5.0`.

**Per episode:** `summary()` gives `max_state`, `visited`,
`failsafe_triggered` (RTL or Land entered), `loiter_entered`, `outcome_label`
(`Land_triggered` if Land was entered, else `RTL_triggered` if RTL was, else
None), `transitions` and `fsm_config_hash`.

## Config (YAML, per mission profile)

```yaml
escalation:            # grant symbols (N, T, T_recover) or the field names
  N: 3
  T: 5.0
  theta_lateral_m: 2.0
  theta_vertical_m: 0.5
  T_recover: 2.0
  loiter_timeout_s: 30.0   # ours; null disables
  event_mode: onset        # ours; or "tick"
  onset_clear_s: 1.0       # ours (2026-10-09); onset rate limit; (N - 1) x it <= T
  x6_min_escape_mps: 0.1   # ours (2026-10-08); X6's progress and X7's losing rate
  x7_min_rise_m: 0.5       # ours (2026-10-09); X7's minimum fitted rise
  loiter_clear: acceptable # ours (2026-11-02); or "violation_free" (the law before)
  loiter_x67: position     # ours (2026-11-02); or "request" (the law before)
  untrusted_owner: chain   # ours (2026-11-07); or "projected" (the reading before)
  hold_horizon: loiter     # ours (2026-11-07); or "lookahead" (the holds before)
  profiles:
    night:
      t_recover_s: 3.0     # either spelling, in either layer
```

`FSMConfig.from_yaml(path, profile="night")`. Unknown keys are **refused**: a
misspelt `t_recovr` that silently kept the 2 s default is the zero that looks
like a setting. Aliases are resolved per layer, so a key given twice inside
one layer is refused while a profile may override the base in the other
spelling. No YAML file is shipped yet. Where it lives (its own file, or an
`escalation:` block in the policy) is open, because `load_policy` would need
to accept the key.

## Integration, call site by call site

Other units are editing these files today, so this section names each call
site by its anchor text only, not by line number. Search for the quoted text.

**Order matters.** Land steps 1 and 2 (models.py accepts the grant's actions,
and shield.py honours `effective_action`) together, and before step 3 wires
the node. If models.py accepts `monitor_only` while shield.py still repairs
every violated rule, the Shield would report a repair that no enforced rule
asked for. The FSM refuses that input as a contract break, and the node's
fault handler (below) would hold the aircraft in LOITER on the first such
tick. That is safe, but it is a bug report, not a flight.

### 1. `guardrail/models.py` (WP1-10)

- `ConstraintBase.violation_action` (`Literal["repair", "brake"] = "repair"`).
  Accept the grant's six plus `repair`, with a `field_validator` that calls
  `fsm.canonical_action` only to **check** the value. Store it as authored:
  rewriting `repair` to `project_fix` in the model would change the canonical
  IR, and with it the policy hash of every policy on disk. `RuleHit`
  canonicalises at use, so nothing downstream needs the rewritten form. Keep
  the default `repair`. (`guardrail/fsm.py` imports nothing from the package,
  so models.py can import it without a cycle.)

### 2. `guardrail/shield.py` (WP3-06, WP3-08, WP1-11, WP1-12)

Keep the FSM out of `Shield.filter()`. filter() keeps a sliding window of its
own decisions (`self._window`), but what it decides must not depend on
escalation state. Its call sites outside tests (sweep, deck builders, demos,
both SITL rails) use it for repair only and have no autopilot whose mode they
could change. The FSM is per-episode state, and the node owns it. The Shield
changes in three ways:

1. **Honour `effective_action` before repairing.** In `_decide`, after
   `violations = self._check(state, raw)`, build
   `hits = fsm.rules_from_policy(self.policy, [v.rule_id for v in violations])`.
   Drop the `monitor_only` ones from the set to repair; they stay in
   `violations` for the record. If any hit's effective action is RTL / land /
   loiter, skip projection, as the reference does ("Rules whose action is a
   direct fail-safe skip projection entirely"), and return `braked=True,
   emitted=BRAKE`. If one is `brake`, return BRAKE without projecting. (The
   FSM keeps a hard `brake` rule in Brake however long it stays blocked; that
   is pinned by a test. Since 2026-10-19 it also stays in Brake when a hard
   `project_fix` rule is violated beside the `brake` rule where a stop is
   legal: that rule was never projected, so the blocked tick is not an
   untrusted repair and does not take G3. Before, a soft P1 brake envelope
   beside a hard NFZ took the aircraft to LOITER. Pinned by tests too, with
   the stop-illegal control, where the chain does run and G3 still fires.)
2. **Report the position correction per operator.** Add
   `magnitude_m: float | None = None`,
   `axis: Literal["lateral", "vertical"] | None = None` and
   `recovery: bool = False` to `class Repair`, and fill them where each
   position operator appends its `Repair`. Vertical: `AltitudeFix`,
   `CorridorAltitudeFix` (the altitude error, m). Lateral: `GeofenceSlide`,
   `ClearanceFix`, `StandoffHold` (the predicted penetration depth, m).
   `GeofenceEscape` and `StandoffRecover` are recovery by name. The flag can
   only add an operator: `recovery=False`, the model default, never
   un-exempts a named one. `CorridorReturn` should set `recovery=True` on its
   `d > half` branch, where the aircraft is already off the corridor, and
   report a lateral magnitude on the other branch. Leave `ClearanceEscape`
   without the flag: whether it is a recovery depends on `stop_illegal`, which
   the FSM side already has. Kinematic clamps stay without a magnitude. This
   is what `fsm.magnitude_from_repairs` reads, and what the grant's audit
   record (p5) carries.
3. **Do not apply θ inside filter().** The FSM decides trust from the
   reported magnitude. filter() keeps braking only when nothing converged
   (the block under `# P0 escape guard: repaired action must re-check
   clean.`). The reference applies θ inside its repair loop instead. The
   result is the same edge, because an over-θ attempt there returns "not
   converged" and the FSM escalates. Keeping it out of filter() keeps every
   call site's repairs unchanged.

Giving up lower-priority rules first, when the stack cannot satisfy all of
them (WP1-12), is a repair-chain change inside the `for _ in
range(_REPAIR_PASSES):` loop. It is separate from the FSM.

### 3. `sitl/ros2_shield_node.py` (WP3-07 wiring, WP3-09)

- **imports** (beside `from guardrail import ...`): `from guardrail.fsm
  import EscalationFSM, FSMConfig, FAILSAFE_STATES, tick_input_from_decision`.
- **`__init__`** (after `self.shield = Shield(...)`):
  `self.fsm = EscalationFSM(FSMConfig.from_yaml(p) if p else FSMConfig())`.
  Add subscriptions next to the existing ones: `/mavros/extended_state`
  (`landed_state` gives `landed`) and `/mavros/home_position/home` (with the
  local pose, `home_reached` = within `REACH_M` of home). Store
  `MavState.mode` in `_on_state`, so a requested mode can be confirmed and an
  autopilot that leaves RTL by itself can be seen.
- **`_tick`**, right after `decision = self.shield.filter(st, raw)`, only when
  `self.shield_on` (the control arm records `fsm_state_after: null`):

  ```python
  try:
      inp = tick_input_from_decision(now, decision, self.policy,
                                     horizon_s=self.theta_horizon_s,  # None = per-operator
                                     rtl_failed=self._rtl_failed(st),
                                     home_reached=self._home_reached(st),
                                     landed=self._landed,
                                     stop_illegal=self.shield.state_is_unsafe(st))
      out = self.fsm.step(inp)
  except ValueError as e:              # a Shield/FSM contract break mid-flight
      # Never let it kill the timer: hold, say why, and stop streaming.
      self.get_logger().error(f"FSM input refused: {e}; requesting LOITER")
      if not self._fsm_fault:
          self.cli_mode.call_async(SetMode.Request(custom_mode="LOITER"))
          self._fsm_fault = str(e)     # into metrics.json; the run is not KPI-grade
      return
  ```

  `stop_illegal` takes the violation list as returned; the helper keeps only
  hard, enforced rules. The FSM raises only on a contract break: an
  inconsistent decision, time going backwards, a repaired tick with no
  magnitude, a position repair that reports no `magnitude_m` (per-operator
  source), or a clamped tick with no caps (proxy source; the policy supplies
  them). A flight that hit one is a bug report, not a data point. AP
  GeoFence stays the backstop (Safety Shield PDF p5).

- **setpoint**: replace `emitted = decision.emitted if self.shield_on else
  raw` with `pass` → `decision.emitted`, `brake` → `Action4D()`, `none` →
  skip `self.pub.publish(tw)` for this tick. Never substitute a zero for
  `pass` on a blocked tick: `decision.emitted` already is the Shield's BRAKE
  where a stop is legal.
- **mode**: `if out.set_mode: self.cli_mode.call_async(SetMode.Request(custom_mode=out.set_mode))`.
  Fire-and-forget, the same pattern as `_finish`, because `_tick` runs inside
  a timer callback. MAVROS's ArduCopter names are exactly `GUIDED`, `LOITER`,
  `RTL`, `LAND`. Log the time to `MavState.mode` confirmation.
- **`rtl_failed`**: true when, after RTL was confirmed, `MavState.mode` stops
  reading RTL without `home_reached`, or `shield.state_is_unsafe(st)` holds a
  hard geofence violation (the RTL track is blocked), or battery is below the
  failsafe level. Which of these counts is an integration choice, and it
  should be written into the manifest. (As built on 2026-11-14,
  RTLFailureWatch: a blocked leg, no progress home while the autopilot
  reports RTL, ArduPilot's battery at CRITICAL, or an RTL never entered; an
  autopilot that leaves RTL by itself is a takeover, after which the FSM no
  longer steps. "2026-11-14: the edges the rails had never taken".)
- **rows** (`self.rows.append({...})`): add `"fsm_state_before":
  out.before.value`, `"fsm_state_after": out.state.value` (the grant's audit
  field names, and what `guardrail/kpi.py` `_escalated` / `_held` read),
  `"fsm_edge": out.edge`, `"set_mode": out.set_mode`, `"setpoint":
  out.setpoint`, `"stop_illegal": inp.stop_illegal` (so later replays can see
  X6 without a reconstruction), and `"failsafe": bool(out.transition and
  out.state in FAILSAFE_STATES)`. kpi.py's `_repair_outcome` already reads
  `row["failsafe"]` and reports `failsafe_instrumented`, so this is the field
  that turns its 0 into a measured 0.
- **audit** (`self.audit.log(len(self.traj), decision)`): pass `out.record`,
  so `fsm_state_before` / `fsm_state_after` land in `audit.jsonl` (WP3-11).
- **`_finish`**: the mission-end LAND is **not** a fail-safe and must never
  set `failsafe`. If the FSM is terminal (`G9` / `G10` / `X5`), do not send
  LAND again. If it is in RTL or Land, let the autopilot finish. Write
  `self.fsm.summary()` into `metrics.json` (`_report`).
- The hard-coded mission landing and the FSM's modes both go through
  `self.cli_mode`. A separate MAVLink adapter node (WP3-09) would own that
  client, with this node publishing the requested mode on a topic instead.

### 4. `sitl/run_sitl_demo.py` (pymavlink rail)

Same pattern after `decision = shield.filter(state, raw)`. Modes go through
the existing mode helper (`self.m.set_mode(mode_id)` with
`self.m.mode_mapping()[out.set_mode]`).

### 5. `guardrail/audit.py` (WP3-11)

`AuditLogger.log` takes an optional `fsm_record` and copies
`fsm_state_before`, `fsm_state_after`, `edge`, `edge_source`, `reason`,
`fsm_config_hash` into the record. Write one file per episode, not appended
(WP4-20).

Since 2026-11-02 (audit record version 4) the record is the grant's, name for
name: `fsm_state_before` / `fsm_state_after` close the grant's ten fields, and
`raw_action` / `emitted_action` (body frame), a violation's `category` (the
grant's three) and `repair_attempts` (the operators tried) carry the grant's
content; this project's world-frame actions, finer category and repair
records are `raw_action_world` / `emitted_action_world`, `rule_category` and
`repair_results`. The file is `shield_audit.jsonl` (guardrail/audit.py).

### 6. `guardrail/kpi.py` and the harness (WP3-15, WP4)

- Per flight: `outcome` takes `RTL_triggered` / `Land_triggered` from
  `summary()["outcome_label"]`. The vocabulary is already in `kpi.OUTCOMES`.
- The grant's KPI is **per episode** with a label, not per tick:
  `fsm.score_failsafe_triggers(episodes)`, where each episode is
  `{"expected_failsafe": bool, "triggered": summary["failsafe_triggered"]}`.
  The in-progress per-tick fields in kpi.py ("expected" = a P0 on the raw
  action) are a proxy. They should be renamed so they do not carry this KPI's
  name.
- **The label exists; the definition of "triggered" does not match yet.**
  `ScenarioSpec.expected_failsafe: bool | None` (guardrail/scenario_spec.py)
  is the label, and `experiments/sweep_scenarios.py` `score()` already
  reports `failsafe_matches_label` and a `_failsafe_labels` summary. But it
  counts `triggered = any(r.get("braked") for r in rows)`, a Shield BRAKE,
  and this module counts entry into RTL or Land. When the FSM is wired in,
  the sweep should take `triggered` from `fsm.summary()["failsafe_triggered"]`
  and report `score_failsafe_triggers` beside its own summary, which adds the
  always-trigger and never-trigger nulls that `_failsafe_labels` lacks.
  Every label written while "triggered" meant BRAKE must be re-read against
  the new definition before the two are compared.
- `experiments/scenarios.yaml` `wind-beyond-authority` describes the expected
  route as "N-in-T persists -> Loiter -> RTL". With this FSM the route is X6
  (the position stays illegal for T while GeofenceEscape keeps running:
  Brake → RTL), or G3 → Loiter → X6 if the escape stops converging. Its
  labels (`RTL_triggered`, `expected_failsafe: true`) stay right. Only the
  description's route should change.
- Expected True: a breached rule whose action is RTL / land; a position that
  stays illegal for T; an unrepairable or over-θ repair that persists past
  the Loiter timeout; an RTL that fails. Expected False: clean control runs,
  and small trusted repairs along a fence. If a scenario is meant to count a
  Loiter, say so in its label.

## The KPI: fail-safe trigger correctness, and its nulls

`score_failsafe_triggers` returns
`(correct triggers + correct non-triggers) / scored episodes`, with:

| Field | Why |
|---|---|
| `null_always_trigger` = expected triggers / scored | what a Shield that RTLs every episode scores |
| `null_never_trigger` = expected non-triggers / scored | what a Shield with no fail-safe scores. This was the delivered Shield |
| `beats_null` | strictly above both. A tie is not evidence |
| `discriminating` | false when every episode has the same label. Then one of the two stubs ties whatever the Shield does, a warning names it, and **`meets_target` / `meets_target_at_95` are None**, not True |
| `wilson_low_95`, `meets_target_at_95` | 99 % shown at 95 % confidence needs **381** error-free scored episodes (`min_error_free_episodes_for_target_at_95`). With 100 perfect episodes the point estimate meets the target and the bound does not |
| `unlabelled`, `not_measured` | episodes left out are counted, never silently dropped |

The first version returned `meets_target: True` and `meets_target_at_95:
True` for 400 episodes of (expected False, triggered False): correctness 1.0
from a Shield with no fail-safe at all, and a passing flag a dashboard would
read as a pass. The correctness figure is still returned, because it is what
was measured, but the target is not judged on a set that cannot fail it.

Labels must be real booleans: `"false"` and `0`/`1` are refused. An empty set
returns `None`, not 0 and not 1.

## How the tests were checked

- **Failing first, the module.** With `guardrail.fsm` made unimportable (the
  tree before this unit), `tests/test_fsm.py` stops at import:
  `ModuleNotFoundError: import of guardrail.fsm halted`.
- **Failing first, review round 1** (three fixes). The first version of this
  module (72 tests) had three defects its own tests could not see. The same
  inputs, run through the first version and then the round-1 version:

  | Input | First version | Round 1 |
  |---|---|---|
  | real Shield decision, aircraft 3 m inside `nfz-square`, `GeofenceEscape`, proxy h = 1 s | X1, setpoint `brake`, 3.0 m over θ: freezes the aircraft inside a P0 zone | G1, `pass`, 0 m (`recovery_exempt: GeofenceEscape`) |
  | speed clamp only (8 → 4 m/s), proxy h = 3 s | X1, `brake`, 12.0 m | G1, `pass`, 0 m |
  | per-operator `GeofenceEscape` with `magnitude_m` 6.0 | 6.0 m counted | 0 m, exempt |
  | blocked decision where the Shield chose a best-effort recovery over a stop | X1, `brake` | X1, `pass` |

  The round-1 tests against the first version give **67/75**, not the 68/75
  this file first said. Eight fail or error: the real-decision test, the
  stop-illegal setpoint and inference tests, the recovery exemption, both
  random-episode tests, the replay's first-escalation test, and
  `test_replay_cli_prints_one_json_line_per_run` (whose exit-2 refusal was
  added after the 68/75 was measured).
- **Failing first, review round 2.** The current 95 tests against the round-1
  `fsm.py` give **76/95**. The behaviour failures, one per fix:
  the real Shield held inside `nfz-square` for 60 s stays in Brake (edges
  `[G1]` only, no X6); 400 all-negative episodes report `meets_target: True`;
  `FSMConfig(window_s=5)` and `FSMConfig()` hash differently; a profile that
  overrides `N` with `n_violations` is refused; a ClearanceEscape where a
  stop is legal is a trusted G1; a stop that breaks only a soft rule counts
  as illegal; the clamp-free proxy, `KinematicCaps`, `RuleHit.kinematic`
  and the replay's `--policy` do not exist. Against the first version the
  current tests give 69/95.
- **Every edge.** `test_every_edge_is_driven` runs one shortest script per
  edge id and fails if `EDGES` gains an id without a script.
- **Random-episode invariants.** 60 noisy episodes check, on every tick: only
  listed edges, mode requested only on a transition, Brake recovers only
  after T_recover clean, Loiter holds ≥ T_recover, no setpoint streamed in
  autopilot modes except the hand-over tick, no stop streamed on a tick whose
  input says a stop is illegal (2157 such ticks, 21 with a stop withheld),
  nothing moves after [*]. A further 60 soft-only episodes never leave
  GUIDED.
- **Mutation.** 69 deliberate defects, each applied to a scratch copy of
  `fsm.py` (never to the repository), and all 69 make at least one named test
  fail. They are the 37 of round 1 (patterns re-pointed where code moved),
  the nine the round-2 reviewer found surviving (a hard `brake` rule sent to
  Loiter by G3, G3 checked before a direct RTL, Loiter counted as a
  fail-safe trigger, the RTL label winning over Land after G6, a soft
  `brake` rule flagged as capped, `rtl_failed` checked before
  `home_reached`, the θ float slack removed, `meets_target` judged on a
  non-discriminating set, `discriminating` true with only positives), and
  23 new ones for X6 (each condition, its order, its clock, its slack), the
  clamp-free proxy and its two refusals, the rescue rule, the hard-only
  `stop_illegal` filter, kinematic attribution, the canonical full digest,
  per-layer aliases and the replay's new counters. The run refuses to report
  if the unmutated scratch copy is not green, because other units were
  editing `guardrail/` at the same time. The harness is a scratch script, not
  committed: each mutant is a literal (old text, new text) pair applied once
  to a copy of `fsm.py`, with `tests/test_fsm.py` run against the copy. The
  reviewer's R9 was not described, so it could not be re-created.

## Questions for the PI

**Answered 2026-11-02** (the PI delegated them as "best practice / follow the
grant"; "2026-11-02: the delegated decisions" above): 1 (reading A), 3 and 11
(onsets, at most one a second; a slow pilot scales T), 5 (RTL or Land only;
docs/RUBRIC-failsafe.md), 10 (BRAKE, kept), 13 (Loiter judges what it would
hand back and where the aircraft is). The rest stand as written.

1. **θ's unit.** Is θ the position correction per operator (the reference's
   penetration depth / altitude error), or ‖Δaction‖ over some horizon? With
   the proxy measured from the clamped command, the delivered flights never
   escalate at h = 0.1 s, escalate on 5 % of position repairs at 0.5 s, and
   on 96 % at 1 s (ROS runs: first over θ between 0.35 s and 0.80 s).
   The default is each operator's forecast depth (`magnitude_m`, the p5
   audit example's field, the reference's metric). The alternative read
   literally from p4, ‖Δaction‖ over one 0.1 s control period, is
   selectable: under 4-5 m/s caps it cannot exceed 2.0 m, so the cap would
   never fire on today's policies; it scores the library 0.903 against the
   default's 0.823, and lets `wind-beyond-authority` into its zone
   (3.0-5.6 m) where the default holds it outside. Which does the grant
   intend? (A third reading, depth into the AUTHORED polygon with no margin
   as the reference measures it, has not been measured.)
2. **Soft rules.** Is "enforced up to brake, never a mode change" what the
   grant means by soft?
3. **N-in-T.** Onsets or ticks? (The grant lists this as open, p7.)
4. **Loiter timeout.** 30 s is ours. What should the operator default be?
5. **Is Loiter a fail-safe trigger** for the ≥ 99 % KPI, or only RTL / Land?
   The outcome vocabulary (Stress Testing p5) has only RTL_triggered and
   Land_triggered, but the Grant overview lists "ArduPilot fence / RTL /
   Land / Loiter as ultimate backstop".
6. Should a **P0 `monitor_only`** rule be allowed at all? It would show up in
   the escape rate, which is honest, but it is probably an authoring mistake
   that the policy lint should catch.
7. **Nowhere legal to stop and no repair converging.** G3 still sends this
   to LOITER, which holds the aircraft in the violation. X6 now bounds that
   to T (5 s after the position became illegal) instead of the 30 s
   timeout. Should G3 go straight to RTL whenever a stop is illegal?
8. **X6.** Is "position illegal for T → RTL, skipping Loiter" acceptable, and
   is the grant's T (5 s) the right persistence time? The alternative,
   counting such ticks as N-in-T events, reaches RTL 0.6 s after any
   incursion.
9. **A rescue where a stop is legal.** When the repair chain does not
   converge and the Shield finds a clean heading by search (ClearanceEscape)
   although a stop was legal, is that "no operator converges" (stop and
   escalate, as this module reads it) or a converged repair?
10. **LOITER on ArduCopter.** The FSM's LOITER is flown as ArduCopter's BRAKE
    (LOITER descends with no pilot input; measured in SITL, 15 m to the ground
    in 6.7 s). Is BRAKE acceptable as "Loiter" for the KPI and in Mission
    Planner, or should the companion hold the throttle with RC overrides?
11. **N-in-T and noise.** Is "at most one counted onset per second" the
    right reading of a "violation" for N-in-T (3 in 5 s) at a 10 Hz monitor?
    It keeps a skimming GPS-noise flicker to one onset and still escalates a
    pilot who re-violates every 0.3-0.9 s; 0 s (every return counts) adds 7
    false triggers on the library.
12. **X6 and X7 (ours).** X6 now leaves a recovery alone while its illegal
    depth is shown to fall at 0.1 m/s or faster (least-squares over T, two
    standard errors), and X7 sends home one whose depth is shown to grow
    (over T_recover, two standard errors, at least 0.5 m). Neither is in the
    grant. Are they acceptable readings of "trigger fail-safe when repair is
    unsafe", and is 0.1 m/s the right line?
13. **"Cleared" in Loiter.** In the harness, 16 of the library's 33 false
    triggers reach RTL by the 30 s Loiter timeout because the pilot keeps
    asking for the violating action while the autopilot holds the aircraft,
    so G8 never sees T_recover of hard-clean ticks. Should Loiter's
    "cleared for T_recover" judge the pilot's request (as now) or the
    position?
14. **X9 (ours, 2026-11-11): a hold that is not kept.** The grant draws
    Loiter -> RTL only for N-in-T persisting and the operator timeout. X9
    adds one: the aircraft carried beyond what a kept hold can carry it
    (its stop on the rail's own model plus 3 m) and still receding for
    T_recover. On the hil smoke the 12 m/s wind carried the held aircraft
    round nfz-square with the position legal and G8 handed it back, a
    missed fail-safe by the rubric's E2. Is "the hold cannot be kept" an
    acceptable reading of p1's "when repair is unsafe" for the Shield's last
    repair? X9 judges the hold, not E2's "toward a hard constraint": should
    a wind beyond authority that blows away from every constraint also end
    the mission (as X9 does today)?

## Not done

- No integration: this unit did not touch shield.py, models.py, the ROS
  node, run_sitl_demo.py, audit.py, kpi.py, the sweep or scenarios.yaml
  (other units own them).
- No SITL flight per transition (card WP3-07's last step). It needs the
  wiring above, and a stop in WSL.
- No mission-profile YAML shipped. The audit record is a dict, not the
  grant's Pydantic model (WP3-11).
- `rtl_failed` / `home_reached` / `landed` detection is specified above,
  not built.
- The recovery list names this repo's operators as of 2026-10-06. A new
  recovery operator is capped until it is added to `RECOVERY_OPERATORS` or
  sets `recovery=True`. That is the safe direction, but it would show up as
  spurious escalations.
- The clamp-free proxy reproduces shield.py's first clamp pass. The clamp
  pass at the end of each repair iteration can trim a position operator's
  output again; the proxy then measures the trimmed correction. Only the
  per-operator source is exact.
- The X6 measurement on delivered flights depends on reconstructing
  `stop_illegal` offline; the plain replay cannot see it until the node logs
  the flag.
- (2026-10-09) The BRAKE hold is measured in ArduCopter SITL only; no flight
  on the Project AirSim rail has used it yet. `sitl/run_sitl_demo.py` (the
  pymavlink rail, another unit's file) still sends ArduCopter's LOITER and
  passes no `unsafe_depth_m`, so on that rail X6 has no progress test and X7
  never fires.
- X6 and X7 fit a line through the depth samples and assume the noise is
  independent from tick to tick. Noise correlated over seconds (GPS drift)
  makes the standard error too small; no test covers that, and X6's
  progress test then also errs towards "working".
- Theta's basis is decided (2026-11-02, reading A). The one-period
  reading stays one argument away (`Shield(theta_horizon_s=0.1)`,
  `--theta-horizon-s 0.1` on the ROS node and the Project AirSim node);
  the depth at first entry and the time-weighted depth were measured, not
  adopted ("Theta on the first deep forecast" above).
- Against the rubric (2026-11-02) the library and smoke beat both nulls,
  but no profile reaches 0.99, and none shows it at 95 %.
