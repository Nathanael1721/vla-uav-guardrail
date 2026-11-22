# Shakedowns of the 22 Nov code at 59e19a5 (hil and SITL, 22 Nov 2026)

The eleven changes of CHANGELOG "2026-11-22 (code)" flown for the first time,
at master 59e19a5, from the detached worktree flight-59e19a5, before the final
smoke and nightly at the same revision. The AGX Orin's companion image was
rebuilt at caecd89 first (the recorder's topic list is baked into it; the code
is mounted from the checkout); the previous image is kept as
`vlaguard/companion:jazzy-pre1122`.

## sk1122a (hil, nightly profile cells, 72 missions, 7665.7 s)

`fly_campaign.sh 59e19a5 nightly sk1122a --only dyn-nfz-rotate-spin
gps-noise-within-margin gps-noise-beyond-margin corridor-along`
(`sk1122a_sweep.json`): 48 passed, 24 tracked, 0 failed. P0 escape 0 over the
Shield-flown ticks of 72 Shield-on missions; fail-safe trigger correctness
(the campaign's labels) 72/72, Wilson 95 % low 0.949.

| Family | Missions | Outcome |
|---|---|---|
| dyn-nfz-rotate-spin (3, 6, 9 deg/s x 3 seeds) | 9 | no fail-safe on any; every flight G1, G3, G8 (Loiter handed back), G7 |
| gps-noise-within-margin (sigma 0.2 / 0.3 m, latency 0-2 ticks, 2 lanes) | 36 | success, no fail-safe (label: none expected) |
| gps-noise-beyond-margin (sigma 1 / 2 m, latency 5 / 10 ticks, 2 lanes) | 24 | fail-safe on every one (21 RTL, 3 Land; label: expected) |
| corridor-along | 3 | success, zero repairs |

- **Retreat ray:** not needed on these nine rotate-spin flights (each search
  found its point inside the budget); its case is rebuilt from
  smfin1120_021 in tests/test_hold_retreat.py.
- **GPS noise / latency on the rail (WP3-19, WP4-14):** the first rail flights
  of these cells. Each run's metrics.json `perception_stress` records what was
  applied (e.g. sigma 1.0 m, latency 5 ticks, seed 1001, 1113 draws over 371
  ticks, largest horizontal offset 4.53 m) and rail_stress_check matched it to
  the cell on every run.
- **Six bundle names (WP4-04, WP4-11):** the three zero-repair corridor-along
  runs carry an empty `shield_audit.jsonl`, and their episode bundles' kpi.json
  says "written" for all six names.
- **Manifest in the bag (ARCH-22):** tools/replay_from_bag.py over the 72
  runs (`sk1122a_replay_from_bag.txt`): `manifest_in_bag` identical on 72 of
  72 (the latched /guardrail/manifest start and end frames against
  manifest.json); trajectory identical on 72. Ticks and KPI identical on 48;
  the 24 gps-noise-beyond-margin runs differ by one setpoint and one tick of
  "who flew it" at the escalation (bag 50 Shield ticks vs kpi.json 51): the
  bag carries no topic that says which side flew a tick, so the replay infers
  it. The custom-data metadata (`manifest_in_the_bag`) stays absent: off by
  default.

## sk1122b (hil, literal SET_MODE LOITER, 2 missions, 287.5 s)

`fly_campaign.sh 59e19a5 smoke sk1122b --only dyn-nfz-rotate-step
dyn-nfz-spawn-ahead --rail-hold loiter` (`sk1122b_sweep.json`,
`*_literal_loiter.json`): the first hil flights in which the FSM's Loiter was
flown as ArduCopter LOITER (WP3-07). On both: mode requests LOITER then RTL,
the autopilot's heartbeat reported LOITER, the Loiter held 29.9 s, altitude
deviation 0.021 m and 0.005 m (tolerance 1.0 m), the throttle stand-in
(RC_CHANNELS_OVERRIDE ch3 1500 as sysid 255) held 30.1 s with 301 of 301 RC3
reports at 1500; the literal_loiter verdict pass on both. P0 escape 0.
dyn-nfz-spawn-ahead passed; dyn-nfz-rotate-step ended in RTL at the Loiter's
30 s timeout (LOITER holds where the aircraft stopped and does not fly the
moved hold point), a false trigger under the ideal-vehicle label, as the
GUIDED hold also reads under that label.

## MSL ridge on the SITL rail (2 missions, 292.9 s)

`sweep_scenarios.py --profile msl_dem --backend sitl --only msl-dem-zone-ridge`
from flight-59e19a5 (`msl_ridge_sitl_sweep.json`): 2 passed, outcome success
on both (target_x -45: 0 repairs, FSM Normal only; target_x -20: 146
repairs, FSM Normal and Brake), P0 escape 0, 0 ticks inside a P0 polygon.
Each run's events record the goto pilot's `terrain_feed` (horizon 1.0 s,
climb clamp 2.0 m/s, the ground's sha256). Before the feed (19 Nov) both
cells went Brake -> Loiter -> RTL short of the pad on the rails (X-11).
