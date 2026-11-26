# Brief for the PI: decision 7, the FSM's Loiter held in GUIDED (WP3-07)

26 Nov 2026. Facts only; each names its source. The decision is the PI's.

## The question

The grant's Safety Shield page has the escalation FSM drive
"`SET_MODE` → GUIDED / LOITER / RTL / LAND" (02-implementation/safety-shield.md,
the mode-setter box and the "Mode escalation" row). The delivered default
does not send SET_MODE LOITER for the FSM's Loiter: the companion holds the
aircraft in GUIDED at the Shield's hold point (lab decision 7, 15 Nov;
docs/FINAL-REPORT.md section 9). **Confirm the GUIDED hold, or require
literal LOITER as the default?** WP3-07 stays PARTIAL until one is chosen.

## The evidence

| What | Where it was flown | Result | Source |
|---|---|---|---|
| Literal LOITER with no pilot on the throttle | the ROS 2 rail on ArduPilot SITL (WSL, `dev` topology), 7 Oct; a short SITL probe 9 Oct (not a stored run) | descended to the ground: from 14.2 m (ros2fix2_on_fsm), from 14.5 m in about 6.5 s on the default mission; in the probe LOITER fell from 15 m to the ground in 6.7 s while BRAKE held 14.97-15.01 m for 19.9 s | docs/DESIGN-ros2-interface.md (the run table; "Changed on 2026-10-09") |
| The same in a follow on ArduPilot | pas_ap_20261008_224440, 8 Oct (Project AirSim + ArduPilot SITL over pymavlink; not a grant topology, not KPI-grade) | 8 Brake -> Loiter edges; the 7 measurable LOITER spells lost altitude at 1.35-1.77 m/s from 7.6-8.0 m; 279 ticks under the policy's 6 m floor, lowest 0.36 m | docs/DESIGN-projectairsim-ardupilot.md "(c) LOITER sinks out of the altitude band"; docs/DESIGN-escalation-fsm.md "2026-10-08" |
| The GUIDED hold (the default since 7 Nov) | every `hil` KPI campaign since, the companion on the AGX Orin | ntFIN2: 337 Loiter entries, 0 LOITER requests; ntFIN5 (release 0.7.0, 654 KPI-grade missions): fail-safe trigger correctness 649/649 under reading (r2), P0 escape 0 | Appendix A WP3-07 and section 7 of docs/FINAL-REPORT.md; docs/data/kpi_ntFIN5/ |
| Literal SET_MODE LOITER, opt-in (`--rail-hold loiter`, 29a1440) | sk1122b on `hil`, 22 Nov, 59e19a5, 2 missions, 287.5 s | the heartbeat reported LOITER; held 29.9 s, altitude within 0.021 m and 0.005 m; the throttle centred by a SITL-only stand-in (RC_CHANNELS_OVERRIDE ch3 1500 as sysid 255; 301/301 RC3 reports at 1500); P0 escape 0; dyn-nfz-spawn-ahead passed; dyn-nfz-rotate-step ended in RTL at the Loiter's 30 s timeout, because LOITER holds where the aircraft stopped, not at the moved hold point | docs/data/shakedown_20261122/README.md "sk1122b" |
| The real flight controller accepts LOITER | bench, 17 Nov, CUAV V5 Nano, ArduCopter 4.7.1, never armed | MAV_CMD_DO_SET_MODE LOITER: COMMAND_ACK 0, the next heartbeat custom_mode 5 (as for GUIDED, BRAKE, RTL, LAND, STABILIZE); the requests came from a bench script, not from the FSM over MAVROS 2; no flight | docs/data/fc_bench_20261117/README.md |

Why LOITER sinks: ArduCopter's LOITER takes its climb rate from the throttle
stick, so with no pilot input it descends (docs/DESIGN-ros2-interface.md).

## What each choice implies

**Confirm the GUIDED hold (decision 7 as delivered).**
- The KPI figures of record (ntFIN5, smREL070) describe the delivered
  default as flown; nothing is re-flown.
- During a Loiter the autopilot reports GUIDED (BRAKE on a stale pose or a
  gap in the companion's stream; guardrail/hold_flight.py,
  guardrail/response.py), not LOITER; the hold depends on the companion's
  setpoint stream.
- WP3-07's Loiter clause is recorded as a deviation accepted by the PI; the
  row can then be reviewed for DONE.
- `--hold-mode LOITER` / `--rail-hold loiter` stay for a flight with a safety
  pilot on the throttle.

**Require literal LOITER as the default.**
- The default changes in code (node `--hold-flight`, the sweep's
  DEFAULT_RAIL_HOLD), so the KPI campaigns, flown with the GUIDED hold, no
  longer describe the default: a KPI-grade nightly takes about 21.5 h on the
  one rig (ntFIN5: 77,356 s).
- LOITER needs the throttle centred. On SITL that is the stand-in above; on
  an aircraft it is a safety pilot, or the companion overriding the pilot's
  throttle with RC overrides (docs/DESIGN-escalation-fsm.md, the hold
  options table).
- LOITER holds where the aircraft stopped, not at the Shield's moved hold
  point: the rotate-step cell ended in RTL at the timeout on sk1122b.
- The literal path has flown 2 missions, both ended by the G5 timeout (no G8
  or G4 in that mode), and cannot run under the SROS 2 profile (the adapter
  enclave is not granted /uas1/mavlink_sink; sitl/sros2_policy.xml).
