# The stress harness's vehicle-response arm

**Date:** 2026-10-19 (the arm's RTL return altitude, and the fixes from the branch review: 2026-10-20).
**Status:** built and flown headless (smoke, nightly and the library, both arms). Not the default: every headline number, pin and exit code is the ideal vehicle's, unchanged.
**Code:** `experiments/sweep_scenarios.py` (`ResponseParams`, `AP_RESPONSE`, `VehicleResponse`, `entry_velocity`, `response_for`, `response_comparison`, `comparison_tag`, `run_folder`, `EXTRAS_SCHEMA`; `--response-arms`), `guardrail/scenario_spec.py` (`RESPONSE_ARMS`, `check_response_arms`, `check_comparison_arms`, `Episode.response`), `tools/measure_vehicle_response.py` (the fit), `tools/kpi_report.py` (`response_suffix`, `comparison_label`, `split_comparison`, the sweep page's comparison section).
**Tests:** `tests/test_harnesstruth_response.py` (19; in the CI loop).
**Grant references:** Stress Testing (the sweep, the KPI report, the acceptance KPIs), Safety Shield p2 (the 5 s / 0.1 s constant-velocity forecast), Architecture constraints (ArduPilot is the autopilot).

## The gap

The headless harness integrated exactly the commanded ground velocity on every tick. `experiments/sweep_scenarios.py` says so in its wind model ("in still air every family flies what it is commanded", the module docstring, and the integration step at the end of `_episode`). That is the same constant-velocity model the Shield forecasts with (`guardrail/shield.py` `_predict`). So the library could not see two things a real aircraft does:

* **Braking carry.** A repair or a Brake that zeroes the into-zone velocity stops the ideal vehicle on the tick. An ArduCopter decelerates under its acceleration and jerk limits, after the command has crossed the link, and travels metres meanwhile (4.6 m from 4 m/s at the measured parameters below, 15.8 m from 8 m/s).
* **What the FSM does when the aircraft cannot turn on the tick.** The ideal vehicle can obey a GeofenceSlide that flips sign every tick (it does: on a head-on approach it wiggles 0.4 m left and right at the forecast's edge, see "Findings"). A real one carries on toward the zone while the repair repeats, and the FSM's convergence test reads that as a repair that is not working.

## The model

`VehicleResponse` (one per episode, only under the arm):

1. **Delay.** A command issued at tick time *t* acts at *t* + `delay_s`; until then the previous one is flown.
2. **Shaped target.** A velocity target moves toward the active command. Its acceleration is the square-root controller of the remaining velocity error with gain `jerk / accel` (linear near zero, a constant-`jerk` approach beyond), capped at `accel` as a vector, and its change per substep is capped at `jerk x h`. This is the structure of ArduPilot's `AC_PosControl` velocity shaping (`shape_vel_accel_xy` on `sqrt_controller` in AP_Math `control.cpp`) as the harness's wind model already cites it for GUIDED `SubMode::VelAccel`. The structure is written after the source; the NUMBERS are not taken from parameter defaults, they are measured (next section).
3. **Airframe lag.** The airframe's velocity follows the target with a first-order lag `tau_s` (velocity loop plus attitude loop).
4. The vertical axis has its own `accel_z`, `jerk_z`, `tau_z`, shaped apart, as ArduCopter shapes it.
5. Integrated at 50 Hz (`substep_s` 0.02) inside each 0.1 s tick. `step()` returns the airframe's **mean** velocity over the tick, so the harness's integrator (`position += velocity x dt`) moves the vehicle exactly as far as the airframe flew, and the wind model then acts on that velocity exactly as it acted on a command.

With every limit off (`delay 0`, infinite `accel` and `jerk`, `tau 0`) the class reproduces the command to float rounding (a test). The ideal arm does NOT use the class at all: its branch is the integrator as it was.

## Measured

`tools/measure_vehicle_response.py` fits the class above - the same code the harness flies - to every recorded ArduPilot flight in the main checkout's `demo/out` whose log carries both the command it sent and the position it read. Run on 2026-10-19 (`python tools/measure_vehicle_response.py --check`, about 4 minutes):

| Data | What it is | What it can say |
|---|---|---|
| `pas_ap_20261008_224440`, `pas_ap_20261012_033329`, `pas_ap_close_on`, `pas_ap_close_off`, `pas_ap_video_on_20261014`, `pas_ap_video_off_20261014` | Project AirSim physics flown by ArduPilot Copter-4.5.7 (`projectairsim-ardupilot-pymavlink`, `docs/DESIGN-projectairsim-ardupilot.md`). Position = ArduPilot's EKF, GLOBAL_POSITION_INT at 20 Hz; `demo/pas_ardupilot/autopilot.py` `pump()` has drained every message each tick since its first version (9bdc331), so a row's position is at most one 50 ms message old. 14 GUIDED segments, 4659 samples. | Delay AND shape. |
| `sitl_shield_off`, `sitl_shield_on`, `sitl_shield_on_dynamic`, `sitl_ped_side_on`, `sitlfix_on_dyn`, `sitl_if_on_dyn` | ArduPilot's own SITL airframe (`ardupilot-sitl-pymavlink`). Every stored `sitl_*` log read a BACKLOGGED pose: `sitl/run_sitl_demo.py` `state()` consumed one queued 10 Hz LOCAL_POSITION_NED per tick until 2026-10-19 while the loop ran at 9.3-9.6 Hz (its own docstring). So row *k* holds the stream's *k*-th sample, on the stream's clock. The fit gives each run its own clock (an offset and a period near 0.1 s), which absorbs the delay. 6 runs, 1002 samples; `sitl_shield_off` is a clean hover-to-6 m/s step. Left out: `sitl_ped_on` (backlog past 6 s; whether samples were dropped cannot be told) and the other Shield-off runs (the same command as `sitl_shield_off`). | Shape only. |

Rows whose command ArduPilot's GUIDED controller did not fly (the FSM handed the aircraft to LOITER / BRAKE / RTL, `flown` false, another mode) end a segment. The loss is the RMS of the vector difference between finite-differenced velocities (+/- 2 samples, 0.4 s) of the log and of the model's positions at the same instants (the same operator on both sides), leaving out the first 2 s of each segment. The search is a coordinate pattern search in pure Python (no scipy in the harness's environment).

### Results (RMS of the velocity difference, m/s)

| Fit | delay_s | accel (m/s^2) | jerk (m/s^3) | tau_s | RMS | Ideal vehicle | Best pure delay |
|---|---|---|---|---|---|---|---|
| pas_ap, all four free | 0.077 | 3.84 | 5.09 | 0.022 | **0.259** | 0.773 | 0.388 (0.9 s) |
| sitl_*, shape (each run on its own clock) | absorbed | 2.41 | 5.72 | 0.034 | **0.216** | 0.469 | - |
| pas_ap at the flown constants | 0.08 | 2.4 | 5.7 | 0.03 | 0.282 | | |
| sitl_* at the flown constants | absorbed | 2.4 | 5.7 | 0.03 | 0.216 | | |
| pas_ap vertical (horizontal delay) | 0.077 | z: 2.9 | z: 20.0 | z: 0.117 | 0.025 | 0.046 (0.038 with the delay alone) | |

Identifiability (each held, the rest re-fitted):

* **delay (pas_ap):** 0.0 s 0.2591, 0.077 s 0.2588, 0.15 s 0.2601, 0.25 s 0.2658, 0.4 s 0.2780. The data say "under about 0.2 s".
* **jerk:** pas_ap 3 -> 0.276, 4 -> 0.261, 5.1 -> 0.259, 6 -> 0.260, 8 -> 0.264; the two airframes agree (5.1 and 5.7). The SITL shape has a jerk / lag ridge: tau 0, 0.03, 0.08 fit equally (0.2155-0.2157, jerk 5.6-6.1); tau 0.25 is 2 % worse at jerk 9.4. A short search stops on that ridge (`--quick` gave jerk 7.2, tau 0.15), so `--check` needs the full search.
* **accel:** bounded below by the SITL step and only loosely above: held at 2.0 the SITL fit is 41 % worse (0.305), at 2.4 0.2155, 2.8 0.222, 3.2 0.224, 3.8 0.228 (6 % worse, with jerk and lag re-fitted). pas_ap prefers more (2.5 -> 0.273, 3.84 -> 0.259, 5 -> 0.262): its follow commands change smoothly (|dv| per tick p90 0.07 m/s), so they rarely reach the limit. So the recordings put the airframes' acceleration limit between about 2.4 and 3.8 m/s^2.
* **vertical:** weakly identified. The vertical commands are small (|vz| p90 0.05 m/s); the fit rules out a slow axis (accel_z 1.0, jerk_z 5.0 fits 6x worse, 0.157) but a 0.3 s pure lag fits as well (0.026).

### Which numbers the arm flies

`AP_RESPONSE = delay 0.08 s, accel 2.4 m/s^2, jerk 5.7 m/s^3, tau 0.03 s; accel_z 2.9, jerk_z 20.0, tau_z 0.12` (`tools/measure_vehicle_response.py` `constants_from`):

* delay from pas_ap, the only recordings where a delay can be told apart from a backlog. It includes the rail's processing, MAVLink transport and up to 50 ms of position-message age; the harness's Shield sees a fresh state, so this is the pessimistic side by at most 0.05 s.
* accel, jerk, tau from the SITL airframe: the one recording that saturates the acceleration limit cleanly, and the lower end of the 2.4-3.8 m/s^2 the recordings allow - the longer braking distance, the pessimistic side, as the wind model takes the lower end of its airframe's authority. pas_ap's own 3.84 is reported, not flown. Its effect is smaller than the numbers suggest: braking from cruise to a zero command takes 1.6 / 4.6 / 15.8 / 33.6 m from 2 / 4 / 8 / 12 m/s at the flown constants, and 2.1 / 4.8 / 12.8 / 25.0 m with pas_ap's own accel and jerk - below about 5 m/s, where most of the library flies, the jerk limit (on which the two airframes agree) sets the distance. Read the counts below as the upper end of the carry at 8 m/s and above.
* vertical from pas_ap, flagged weak above.

`tests/test_harnesstruth_response.py` checks that these constants still explain the recordings (pas_ap RMS under 0.30 and under 0.8x the best pure delay; SITL under 0.23 and under half the ideal; vertical better than ideal) on any host that has them, and skips by name elsewhere.

## Under the arm only

Each of these exists so the comparison measures the vehicle, not an artefact of the ideal one; with the arm off none of them runs.

* **Entry velocity** (`entry_velocity`). The families start mid-flight (a start pose and a cruise speed the ideal vehicle flies at once). The airframe enters at the pilot's first command held to the policy's kinematic envelope - not at an asked-for 20 m/s under an 8 m/s cap, and not at the Shield's tick-0 repair (near-edge-at-12mps slides 8 m/s sideways at t = 0, which is the Shield's reaction to a scenario that has not begun).
* **The goto pilot slows into its goal** (`_pilot`). The bang-bang goto (full speed until 3 m, then zero) would fly through its goal by the braking distance and swing around it for the rest of the episode, so every goto mission would read "goal missed" for a reason that is the pilot's. It now slows at the vehicle's own acceleration (sqrt controller of the distance), still straight at the target and blind to every rule.
* **The modelled RTL closes on position targets** (`_autopilot`). ArduPilot's RTL flies to the return altitude and then home with its position controller and stops there; "min(speed, distance / dt)" through a carrying vehicle would overshoot home by a braking distance and leave a climb's overshoot uncorrected above RTL_ALT (an entry at 14.9 m climbing at 2 m/s is carried to 16.0 m). The return altitude is captured once, when the FSM hands the aircraft to RTL, as max(RTL_ALT, the altitude then): ArduPilot's RTL_ALT is a minimum ("if the vehicle is flying higher than this value it will return at its current altitude"), and the ideal vehicle's RTL also flies home at the altitude it entered at. Added on 2026-10-20, after the runs below were first flown: until then the arm closed on 15 m always, which would have descended an RTL entered higher (`failsafe-rtl-zone` flown at 20 m went down to 15.0 m on the way home; a test now pins 20 m on both vehicles). Re-flown on the final code (2026-10-20): the smoke and library runs below are reproduced field for field, and the nightly run's every count and table; three nightly ArduPilot-arm episodes (`gps-noise-beyond-margin`, sigma 2, seed 1003), which entered RTL 0.00001 m above RTL_ALT, differ by one intervention episode each (`intervention_episodes`, `mean_intervention_s`). LOITER (hold) and LAND (descend to the ground) are unchanged: the response itself carries the vehicle into the hold. Since the merge with the hold points (2026-10-21) a MOVED hold - the FSM's hold point away from the aircraft, flown as a GUIDED position target before the hold - is closed by the same sqrt controller on the arm, at the plan's speed and climb (docs/DESIGN-hold-points.md, "With the own-velocity forecast and the vehicle-response arm"); an in-place hold is the hold above.
* **The ground stops a descent.**
* **The spawn heading** (`spawn_polygon_fence` "ahead of the vehicle") is the airframe's velocity, not its command.
* **The airframe's velocity is judged at the true state too** (`vjudge`, since 2026-10-20). `true_p0_flown_ticks` judges the COMMANDED action at the true position, before the vehicle steps, so braking carry - a command of zero while the airframe still moves toward the zone - never shows in it. Under the arm a fourth Shield, kept in step with the others, judges the airframe's mean velocity over each tick from the same true position (`true_p0_vehicle_ticks`; per row `vehicle_violations`). A Shield of its own, because `filter()` feeds a Shield's sliding window and monitor clock. On the ideal vehicle the airframe flies its command, so the two counts are one, and the comparison reads the ideal side's airframe count as its commanded count.
* Extra per-episode keys: `max_vehicle_speed`, `max_velocity_lag_mps` (the arm ran: a lag of 0 everywhere would be the ideal vehicle under another name), `nonfinite_commands_held`, `true_p0_vehicle_ticks`; per row `vehicle_v`, `vehicle_violations`.

## Labelled like the forecast arm, and kept out of the same places

* `--response-arms ideal ardupilot` (or a profile's `response_arms`). The copy is tagged `response-ardupilot`: run id `cell#response-ardupilot`, bundle `episode-<UTC>--<cell>--response-ardupilot--seed<n>`, KPI row `<template>/<cell>/shield-on/response-ardupilot`.
* **Every** bundle and KPI table records its vehicle, the ideal one included: `vehicle_response` (arm, model, parameters) in the extras, the KPI table and the `episode_start` event; `arm.response` in `manifest_extras.json`; `response_arm` in the result row, the KPI table and every `_cell_rollup` row. The six-field manifest stays the grant's six fields. An ArduPilot bundle's extras were `sweep-episode-extras/2` on this branch (a headline bundle's stayed `/1`): a harness without the arm (c77c77d, or a branch not rebased onto this one) accepted `/1` and would have re-flown an ArduPilot bundle on the ideal vehicle under its id; it refuses `/2`. Since the merge with the hold points (2026-10-21), which had taken `/2` for `arm.autopilot_hold` at the same time, every bundle is `sweep-episode-extras/3` and names both `arm.response` and `arm.autopilot_hold`: each branch's reader knew one of the two keys and would re-fly a merged bundle on the wrong vehicle or with the wrong hold, and both refuse `/3` (the schema's history is at `EXTRAS_SCHEMA`). `rerun_from_extras` reads all three: `/1` as the ideal vehicle (a bundle from before the arm names none; one that names another vehicle under `/1` is refused), held in place unless it names its hold; `/2` by the key it names - a non-ideal `arm.response` is this branch's bundle, flown on that vehicle and held in place, an `arm.autopilot_hold` the hold-point branch's, on the ideal vehicle, and one naming both or neither is refused; `/3` must name both. It refuses one whose recorded parameters differ from today's `AP_RESPONSE`.
* `tools/kpi_report.py` files the arm under `/response-ardupilot` (`response_suffix`, part of the family name and the duplicate key) and renders a "Vehicle-response comparison" section on the sweep page when, and only when, the arm flew. In the cross-run report an episode flown on a comparison configuration (`/forecast-3s`, `/wind-open_loop`, `/response-ardupilot`; `comparison_label`) keeps its labelled family row but is ranked in its own top-K (`comparison_top_failures`, its own Markdown section) and indexed in `comparison_failures` - never in the grant's top-K, `failures`, the headline facts or the prefix A/B (`split_comparison`, since 2026-10-20). `--baseline` compares headline with headline, so a baseline written before the split does not report its comparison copies as fixed.
* **Its own folder.** A run that flies a comparison configuration without `--out` / `--runs` goes to `demo/out/cmp_<profile>_<label>/` (`demo/out/cmp_nightly_response-ardupilot/` for the nightly profile; `cmp_library_...` for the library), outside the `demo/out/stress_*/episode-*` glob the KPI report reads by default (`run_folder`, since 2026-10-20). Until then a profiled run wrote its copies into `demo/out/stress_<profile>/` beside the headline bundles. Every default invocation and every shipped profile (none declares a comparison arm) keeps its folder.
* Results go to `_response_comparison` (per family and in total, both arms over the same (cell, seed) pairs: unsafe-position entries, polygon entries Shield-flown and autopilot-flown, the Shield's P0 escapes, true-state P0 ticks of the commanded action and of the velocity the airframe flew, the labelled fail-safe score, deepest true distance; every episode that moved) and `_rollup._comparison_response_ardupilot_shield_on`. Never in `_counts`, `_failsafe_labels`, `_true_state`, `_top_failures` or the exit code; the arm's episodes are still judged against every gate and pin, and their verdicts are counted in `_response_comparison.counts`.
* Refused (exit 2, before anything flies): with `--publish` (the published sweep is the headline library run); with a comparison forecast (`check_comparison_arms`: a 3 s episode on the ArduPilot response would shadow neither twin); without `ideal`; on `--backend sitl` (the rail flies ArduPilot for real).

## Flown 2026-10-19 (headless)

Shield-on episodes, ideal -> ArduPilot over the same (cell, seed) pairs. "Unsafe-position entries" are run_episode's `breaches` (the aircraft moved itself into a position where standing still violates a rule, margin ring included); "polygon entries" are true-state crossings of a P0 polygon's own edge, Shield-flown and autopilot-flown; "P0 escapes" is the grant's KPI (the Shield's own count); fail-safe is the labelled correctness (FSM entering RTL or Land, against `expected_failsafe`). "True-state P0 ticks" come twice since 2026-10-20 (review of T1b-harness-vehicle, P3): **(command)** judges the commanded action at the true position before the vehicle steps (`true_p0_flown_ticks`, the column these tables had until then); **(airframe)** judges the velocity the airframe flew over the tick from the same position (`true_p0_vehicle_ticks`), where braking carry shows. On the ideal vehicle the two are one count. The tables below were re-flown on the final code on 2026-10-20 (every other column is what the first runs gave, field for field); the runtimes are of those re-runs, five runs sharing the desktop.

### Nightly profile (221 cells x 3 seeds; 732 headline + 732 ArduPilot episodes, 1154 s)

The runtime is the 2026-10-20 re-run's. The first run (2026-10-19) took 1943 s with three other runs sharing the desktop, and the re-run after the RTL return-altitude fix 1052 s; the counts are the same in all three.

| Family | Eps | Unsafe-position entries | Polygon entries | Autopilot polygon entries | P0 escapes | True-state P0 ticks (command) | True-state P0 ticks (airframe) | Fail-safe correct | Deepest true distance (m) |
|---|---|---|---|---|---|---|---|---|---|
| action_contract | 3 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 3/3 -> 3/3 | 47.00 -> 47.00 |
| altitude_envelope | 27 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 27 | 27/27 -> 27/27 | 37.00 -> 37.00 |
| corridor_keep_in | 42 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 150 | 42/42 -> 42/42 | 2.32 -> 2.32 |
| corridor_swap | 9 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 27 | 0 -> 93 | 9/9 -> 6/9 | n/m -> n/m |
| dynamic_nfz_movement | 219 | 0 -> 0 | 0 -> 0 | 0 -> 18 | 0 -> 0 | 150 -> 2247 | 150 -> 3873 | 144/219 -> 111/219 | -20.00 -> -20.00 |
| failsafe_escalation | 30 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 2241 -> 2250 | 2241 -> 2706 | 30/30 -> 30/30 | 6.00 -> 1.56 |
| high_speed_near_edge | 90 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 258 | 84/90 -> 90/90 | 1.00 -> 1.26 |
| kinematic_envelope | 3 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 3/3 -> 3/3 | 47.00 -> 47.00 |
| radius_scaling | 15 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 450 | 0 -> 558 | 15/15 -> 6/15 | -12.40 -> -17.39 |
| sensor_degradation | 84 | 6 -> 12 | 3 -> 6 | 0 -> 0 | 0 -> 0 | 938 -> 972 | 938 -> 1411 | 34/60 -> 37/60 | -7.00 -> -7.81 |
| static_nfz | 21 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 156 | 21/21 -> 21/21 | 9.90 -> 5.26 |
| subject_standoff | 63 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 207 | 54/63 -> 54/63 | n/m -> n/m |
| three_simultaneous_violations | 54 | 0 -> 0 | 0 -> 0 | 0 -> 54 | 0 -> 0 | 4086 -> 3942 | 4086 -> 4212 | 0/54 -> 0/54 | 1.70 -> -1.95 |
| time_window_static | 6 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 42 | 6/6 -> 6/6 | 12.00 -> 7.36 |
| time_window_switch | 39 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 18 -> 18 | 315 -> 315 | 315 -> 516 | 39/39 -> 39/39 | -8.20 -> -5.34 |
| wind_disturbance | 24 | 0 -> 0 | 0 -> 0 | 12 -> 12 | 0 -> 0 | 1164 -> 1986 | 1164 -> 2173 | 21/24 -> 15/24 | -7.88 -> -7.73 |
| **total** | 729 | **6 -> 12** (episodes 6 -> 12) | **3 -> 6** | **12 -> 84** | **18 -> 18** | **8894 -> 12189** | **8894 -> 16382** | **532/705 -> 490/705** (170 -> 212 false, 3 -> 3 missed) | |

ArduPilot copies' own verdicts: 460 pass, 66 tracked, 105 fail, 93 known, 8 pinned-but-passing. Fail-safe correctness 0.755 -> 0.695 (Wilson 95 % low 0.722 -> 0.660; never-trigger null 0.940). Autopilot unsafe-position entries 24 -> 51. The Shield-off controls (3) enter on both vehicles, as they must. Command: `python experiments/sweep_scenarios.py --profile nightly --response-arms ideal ardupilot --out <dir>/rsp_nightly.json --runs <dir>/rsp_nightly_runs` (without `--out` / `--runs` it now writes to `demo/out/cmp_nightly_response-ardupilot/`).

### Library (`experiments/scenarios.yaml` on its declared seeds; 176 headline + 176 ArduPilot episodes, 269 s)

| Family | Eps | Unsafe-position entries | Polygon entries | Autopilot polygon entries | P0 escapes | True-state P0 ticks (command) | True-state P0 ticks (airframe) | Fail-safe correct | Deepest true distance (m) |
|---|---|---|---|---|---|---|---|---|---|
| action_contract | 1 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 1/1 -> 1/1 | 47.00 -> 47.00 |
| altitude_envelope | 9 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 9 | 9/9 -> 9/9 | 37.00 -> 37.00 |
| corridor_keep_in | 14 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 50 | 14/14 -> 14/14 | 2.32 -> 2.32 |
| corridor_swap | 3 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 9 | 0 -> 31 | 3/3 -> 2/3 | n/m -> n/m |
| dynamic_nfz_movement | 27 | 0 -> 0 | 0 -> 0 | 0 -> 2 | 0 -> 0 | 50 -> 669 | 50 -> 856 | 18/27 -> 13/27 | -20.00 -> -20.00 |
| failsafe_escalation | 6 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 747 -> 750 | 747 -> 836 | 6/6 -> 6/6 | 6.00 -> 1.56 |
| high_speed_near_edge | 36 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 102 | 34/36 -> 36/36 | 1.00 -> 1.26 |
| kinematic_envelope | 1 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 1/1 -> 1/1 | 47.00 -> 47.00 |
| radius_scaling | 5 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 150 | 0 -> 186 | 5/5 -> 2/5 | -12.40 -> -17.39 |
| sensor_degradation | 28 | 2 -> 4 | 1 -> 2 | 0 -> 0 | 0 -> 0 | 333 -> 346 | 333 -> 498 | 12/20 -> 12/20 | -7.00 -> -7.81 |
| static_nfz | 7 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 52 | 7/7 -> 7/7 | 9.90 -> 5.26 |
| subject_standoff | 9 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 35 | 8/9 -> 8/9 | n/m -> n/m |
| three_simultaneous_violations | 6 | 0 -> 0 | 0 -> 0 | 0 -> 6 | 0 -> 0 | 454 -> 438 | 454 -> 468 | 0/6 -> 0/6 | 2.38 -> -1.36 |
| time_window_static | 2 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 14 | 2/2 -> 2/2 | 12.00 -> 7.36 |
| time_window_switch | 13 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 6 -> 6 | 105 -> 105 | 105 -> 172 | 13/13 -> 13/13 | -8.20 -> -5.34 |
| wind_disturbance | 8 | 0 -> 0 | 0 -> 0 | 4 -> 4 | 0 -> 0 | 388 -> 662 | 388 -> 724 | 7/8 -> 5/8 | -7.88 -> -7.73 |
| **total** | 175 | **2 -> 4** (episodes 2 -> 4) | **1 -> 2** | **4 -> 12** | **6 -> 6** | **2077 -> 3129** | **2077 -> 4033** | **140/167 -> 131/167** (26 -> 35 false, 1 -> 1 missed) | |

ArduPilot copies' own verdicts: 122 pass, 22 tracked, 19 fail, 11 known, 2 pinned-but-passing (both `near-edge-at-12mps` cells: the ideal vehicle's chatter-driven RTL does not happen on the real one). The headline (ideal) half of the run is field-for-field the base library run's (see "The default is unchanged").

### Smoke profile (62 + 62 episodes)

Unsafe-position entries 2 -> 2, polygon entries 1 -> 1, autopilot polygon entries 2 -> 3, P0 escapes 2 -> 2, true-state P0 ticks of the command 1068 -> 1280 and of the airframe 1068 -> 1647, fail-safe 55/59 -> 49/59 (3 -> 9 false, 1 -> 1 missed). The ArduPilot copies: 47 pass, 6 tracked, 6 fail, 3 known (95 s).

### What it says

* **Unsafe-position entries double, from a small base: 6 -> 12 in the nightly profile (polygon entries 3 -> 6), 2 -> 4 in the library, all in sensor_degradation.** At the library's speeds (2-8 m/s flown; asks of 12-20 m/s are capped at 8 by the policy) the 5 s forecast sees a zone 10-40 m ahead and repairs early, so the 1.6-16 m of braking carry is absorbed before the margin ring. The new entries are in sensor_degradation (`gps-dropout@lane_y=15,window=2-9`: under a frozen fix the Shield-flown aircraft is carried 3.4 m into the polygon, against 1.0 m short of it on the ideal vehicle). The carry shows instead in the deepest true distance (static_nfz 9.9 -> 5.3 m in smoke, nfz-head-on 20.6 -> 15.3 m) and in where the autopilot ends up.
* **The autopilot is carried in: autopilot polygon entries 12 -> 84 in the nightly profile (4 -> 12 in the library), autopilot unsafe-position entries 24 -> 51.** A LOITER or RTL entered at speed decelerates under the same limits: three-simultaneous-violations (all 54 nightly episodes: the RTL climb carried into the zone, deepest 1.95 m inside) and dyn-nfz-spawn-ahead with the zone 3 m ahead (18 nightly episodes; 1.62 m inside in the library). These sit outside the grant's P0 escape rate by definition (the Shield did not fly those ticks) and are reported in `autopilot_*`.
* **The fail-safe score drops: 532/705 -> 490/705 in the nightly profile (170 -> 212 false triggers, 3 -> 3 missed), 140/167 -> 131/167 in the library.** It drops in dynamic_nfz_movement (144 -> 111 of 219), radius_scaling (15 -> 6 of 15), wind_disturbance (21 -> 15 of 24) and corridor_swap (9 -> 6 of 9), and rises in high_speed_near_edge (84 -> 90 of 90) and sensor_degradation (34 -> 37 of 60). The mechanism, traced in `wind-within-authority@lane_y=15,wind_speed_mps=5`: on a head-on approach the ideal vehicle obeys a GeofenceSlide that alternates sign every tick and hovers at the forecast's edge (x frozen at -13.6 m, y 14.6 / 15.0 / 15.4); the real one carries on 2 m toward the zone while the repair repeats, the repair's magnitude passes theta within 0.5 s, and the FSM takes G3 (Brake -> Loiter) and later G5 (Loiter timeout -> RTL). The same chain sends dyn-nfz-spawn-ahead (30 m ahead), radius-scaling-grow (X6) and dyn-nfz-rotate-spin (X6 -> G6 Land) to a fail-safe their labels do not expect. The pinned cells that move the other way (8 pinned-but-passing in the nightly profile: both near-edge-at-12mps cells on every seed and gps-noise-within-margin@latency=0,pass_y=23.2 on two) are the other side of the same coin: on the ideal vehicle repeated onsets at the margin trip N-in-T (G2, then G4 or the Loiter timeout G5) to a fail-safe their labels do not expect; on the real one the slide is flown once and clears (G1 > G7).
* **True-state P0 ticks of the command rise in the moving- and growing-zone families** (nightly: 8894 -> 12189 in total; dynamic_nfz_movement 150 -> 2247, radius_scaling 0 -> 450, wind_disturbance 1164 -> 1986): a zone that moves or grows onto the aircraft can no longer be escaped on the tick, so the hold that follows sits inside it for longer.
* **Judged on what the airframe flew, they rise everywhere the aircraft brakes: 8894 -> 16382 in the nightly profile (2077 -> 4033 library, 1068 -> 1647 smoke).** The families whose commands never carry a P0 violation at the true state - static_nfz, high_speed_near_edge, corridor_keep_in, subject_standoff, altitude_envelope, time_window_static - all read 0 on the command and 27-258 on the airframe (nightly): each tick the Shield has already zeroed or turned the into-zone command while the airframe still carries toward the zone or the band edge (altitude-ceiling-hold: the pilot's 2 m/s climb carried for about a second after the Shield's stop, nfz-head-on 0 -> 6 ticks). This is the braking carry the command column cannot see. Like the command column it judges each velocity as held over the 5 s forecast; a tick counted here is a tick on which the aircraft was flying toward a violation, not one inside a zone (the polygon entries say that).
* **P0 escapes are unchanged (18 -> 18 nightly, 6 -> 6 library).** The Shield's own count judges the action it emitted at the state it saw; a carrying vehicle does not change what it emitted. That is the reason the true-state counts exist beside it.

These are the honest baseline the task asked for: none of them is a Shield change. Under the grant's semantics (FSM edges G1-G8 and the P0 escape definition untouched) they say the FSM's convergence test and the Shield's slide are tuned to a vehicle that turns on the tick.

## The default is unchanged

The ideal arm runs none of the arm's code (every branch is behind `veh is not None` / `rsp is not None`; the airframe judge `vjudge` is not even built). Compared field by field against runs of the base commit c77c77d (`git archive c77c77d`), ignoring only run-specific fields (runtime, the command line, the code revision, bundle paths, measured `sim_speedup`) and the keys the arm ADDS as labels (`response_arm`, `vehicle_response`, `_response_comparison`, `_scope.arms.response_arms`, `_scope.arms.response_comparison_episodes`, and in a run with the arm its own `/response-ardupilot` rows).

One more run-specific field differs on EVERY bundle, and is expected to: the six-field manifest's `vla_model_hash` (62 of 62 in smoke, 732 of 732 in nightly; review of T1b-harness-vehicle, P4). The headless harness's pilot is `PILOT_ID`, and its hash is `model_hash(PILOT_ID)`, a source hash of the whole `experiments/sweep_scenarios.py` file, so it moves with any edit of that file, this one included; it says which harness source flew the bundle, not what it flew. A reader comparing manifests should ignore it with the code revision.

The CI guard for the ideal path is `tests/test_harnesstruth_response.py` `test_the_ideal_vehicle_flies_exactly_what_it_flew_before`: beside the default-versus-explicit-`ideal` comparison (both sides run the same code, so it cannot see a leak by itself) it pins five episodes to what they flew at c77c77d - ticks, final x / y / up, repair rows and operators, min true distance (`IDEAL_AT_C77`: nfz-head-on, failsafe-rtl-zone, wind-gusts, altitude-recovery-depth, and corridor-along, a goto that reaches its goal). Checked by mutation: the arm's slowing goto pilot leaked into the ideal branch fails it on corridor-along (0.23 m short), the arm's position-target RTL on failsafe-rtl-zone (56 ticks, not 52).

Field by field (on 2026-10-19, and again on the final code on 2026-10-20, every line with the same result):

* smoke profile, default invocation: 0 differences, 62 episodes, `_counts` 52 pass / 6 tracked / 0 fail / 4 known (both); its `kpi_report.md` differs only in the revision, runtime, command and bundle paths (and, since 2026-10-20, in the per-family table's intro sentence, which now names the `/forecast-3s` and `/response-ardupilot` comparison rows);
* nightly profile, default invocation: 0 differences, 732 episodes, `_counts` 508 pass / 66 tracked / 0 fail / 158 known (both);
* nightly profile with `--response-arms ideal ardupilot`: the headline half 0 differences against the base nightly run, 732 episodes, `_counts` 508 / 66 / 0 / 158 (both) - the comparison flying in the same process changes nothing of the headline;
* the library with `--response-arms ideal ardupilot`: the headline half 0 differences against the base library run, 176 episodes, `_counts` 132 / 22 / 0 / 22 (both).

## Findings outside this task (for the lead)

* **A knife-edge in dyn-nfz-spawn-on-top.** The cell spawns a 20 m square centred exactly on the aircraft with its axes along the heading; the Shield's verdict on the next tick flips with a 1e-14 m change in position (the ideal vehicle repairs toward the right-hand edge; a limitless response, whose positions differ only by float rounding, flies straight out through the front edge with no violation logged). Both are escapes; the cell's outcome is decided by a tie.
* **The slide alternates on a head-on approach.** `wind-within-authority@lane_y=15` (and every head-on cell like it): the GeofenceSlide's tangent flips sign every tick, which the ideal vehicle absorbs as a 0.4 m wiggle and a real one cannot. This, not the wind, is what the arm's extra false triggers come from.

## Hook: a Shield that receives a measured velocity

Another branch adds an optional measured velocity to `State` for the Shield's forecast. Under this arm the measurement exists: `veh.velocity` is the airframe's velocity at the start of the current tick. The one line, in `_episode` just before `d = shield.filter(seen, raw, t=t_now, **flags)`, once that field exists (shown for a `velocity` field; use that branch's name):

```python
if veh is not None: seen = seen.model_copy(update={"velocity": veh.velocity})
```

With `stress.latency_ticks` the velocity should be the one of the tick `seen` was taken from (keep a history beside `hist`), and under GPS noise it should get the noise the rail's EKF velocity has; neither is done here. On the ideal vehicle the measured velocity IS the previous command, so the hook belongs to this arm only.

## Not modelled

The airframe's speed and lean limits (a command beyond what the frame can fly is still flown, as on the ideal vehicle); the interplay of the acceleration limit with the wind (the wind model is applied to the airframe's velocity as it was to the command); ArduPilot's LOITER braking parameters (a LOITER entered at speed decelerates under the GUIDED limits measured here); yaw; the EKF's own velocity noise. A SITL step-response recording with the drained pose (below) would measure the delay on ArduPilot's own airframe and settle the vertical axis.

## For the lead: recordings that would sharpen the fit

In WSL, with SITL started by `bash sitl/start_sitl.sh` (the pose has been drained every tick since 2026-10-19, so these rows are fresh):

```bash
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield off --dev-yaml --tag sitl_resp_off_20261020
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield on --dev-yaml --tag sitl_resp_on_20261020
```

then, on Windows, in the repository:

```bash
python tools/measure_vehicle_response.py --fresh-run sitl_resp_off_20261020 --fresh-run sitl_resp_on_20261020 --json demo/out/vehicle_response_fit.json
```

and, if the constants move, update `AP_RESPONSE` (the tool's `--check` says which) and re-fly the arm (results, bundles and `kpi_report.md` go to `demo/out/cmp_nightly_response-ardupilot/`, outside the folders the default KPI report reads; `python tools/kpi_report.py --runs 'demo/out/cmp_nightly_response-ardupilot/episode-*' --no-sweep` reads them, with the copies in their own top-K):

```bash
python experiments/sweep_scenarios.py --profile nightly --response-arms ideal ardupilot
```
