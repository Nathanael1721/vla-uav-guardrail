# ntFIN4: the KPI-grade nightly at 59e19a5 (22-23 Nov 2026)

The nightly profile on the hil rig (AGX Orin + ArduCopter 4.5.7 SITL over
MAVROS 2) at master 59e19a5: 71aabb3 plus the eleven changes of CHANGELOG
"2026-11-22 (code)", shaken down the same day (`docs/data/shakedown_20261122/`).
`--hil-controls`, from the detached worktree flight-59e19a5, companion image
rebuilt at caecd89. Launched 22 Nov 15:42 after the smoke smFIN2
(`docs/data/kpi_smFIN2/`), 78,879.2 s (21.9 h). 744 episodes planned, 654
missions flown (651 Shield on, 3 Shield-off controls), 90 refused before
take-off (paraphrase and prefix arms the stub pilot cannot read, and the
stressors the hil rail cannot realise). The 60 GPS-noise / latency missions
flew on the rail for the first time in a nightly (they were refused through
ntFIN3).

653 of 654 KPI-grade. Not KPI-grade: one dyn-nfz-translate run, whose MAVLink
adapter handed none of the Shield's setpoints to MAVROS for 25 s (t 8.35-33.51
s): "setpoint delivery: 1 gap over 0.5 s" (`guardrail.delivery`, the rule
since 17 Nov; ntFIN2_175 was the earlier case).

## Files

| File | What |
|---|---|
| `kpi_report_ntFIN4_2026-11-23.md` / `.json` | `tools/kpi_report.py --campaign demo/out/hil_ntFIN4 --rescore-rail docs/data/kpi_ntFIN4/rescore_r2_ntFIN4.json --baseline docs/data/kpi_ntFIN3/kpi_report_ntFIN3_2026-11-21.json`, made from a clean tree at 2f9d863 |
| `rescore_r2_ntFIN4.json` / `.txt` | `experiments/sweep_scenarios.py --profile nightly --rescore-rail` over the 654 runs |
| `replay_at_revision_ntFIN4.json` / `.per_run.jsonl.gz` | `tools/replay_at_revision.py --root demo/out/hil_smoke --only ntFIN4_` (the replay that rebuilds the pose the Shield judged under GPS noise, a436d32) |

## Figures, against ntFIN3

| Figure | ntFIN3 (71aabb3) | ntFIN4 (59e19a5) |
|---|---|---|
| KPI-grade missions | 594 (591 Shield on, 3 controls) | 653 (650 Shield on, 3 controls) |
| P0 violation escape rate, Shield on | 0 on 17,457 P0-violation ticks | **0** on 17,078 P0-violation ticks |
| Shield-off control / passthrough null | 0.6458 / 0.2220 | 0.6500 / 0.1935 |
| Fail-safe trigger correctness, (r2) adopted | 587/589 = 0.997, Wilson 95 % low 0.988 | **648/648 = 1.000, Wilson 95 % low 0.994** |
| Same, as before the GeoFence backstop rule | 569/571 | 630/630 |
| Same, backstop runs counted as not triggered | 569/589 = 0.966 | 630/648 = 0.972 |
| (r) / (a) | 575/589 / 516/589 | 636/648 = 0.981 / 573/648 = 0.884 |
| Triggered by the GeoFence backstop | 18 (all expected) | 18 (all expected, all correct) |
| Left out of (r2) | 2 (censored in Loiter) | 2 (censored in Loiter: ntFIN4_374, ntFIN4_375 corridor-swap-shifted) |
| `tick_ms` (hil) | p99 28.9 / max 74.52 ms, 0 of 174,186 over 100 ms | p99 28.4 / **max 105.50 ms, 1 of 183,909 over 100 ms** |
| `e2e_ms` (hil) | p99 29.0 / max 74.62 ms, 0 over | p99 28.5 / max 105.60 ms, 1 over |
| Mission success, Shield on | 252/591 | 287/650 |
| Repair success | 98.1 % (26,747/27,258) | 98.1 % (27,097/27,615) |
| Mean time to safe | 3.38 s (170 spells; 95 never recovered) | 3.39 s (169 spells, max 9.70 s; 95 never recovered) |
| Repairs per Shield-on mission | 54.4 | 50.2 |
| single_sha | 594/594 | 653/653 (companion and desktop 59e19a5e3b19, ArduCopter V4.5.7 2a3dc4b7) |

Verdicts: P0 escape rate 0 (meets); fail-safe trigger correctness 1.000
(meets 99 % as the point estimate and at 95 % confidence); the monitor tick
budget is met at p99 (28.4 ms) but not on every tick: one tick of 183,909 took
105.5 ms (below).

The report's "against the baseline" section matches episodes by run name, so
it lists every run as new or gone; its per-family table is the comparison.

## The one tick over 100 ms

`ntFIN4_236_dyn-nfz-spawn-ahead`, row 50 (t 5.03 s): tick_ms 105.495, e2e_ms
105.599, shield_ms 10.581, FSM edge G3 (Brake -> Loiter). It is the tick on
which the scenario's zone is hot-applied: before the Shield runs, the node
applies the event and writes the new policy generation (policy_g1.json, the
CSP compile with its first, cold template load, csp_g1.json) and publishes the
fences, all before the setpoint. That block normally takes 20-40 ms (median
about 32 ms over the nightly's spawn ticks; spawn ticks that are also G3:
median 51.0 ms, 31.8-58.8 ms apart from this one); here it took 87 ms (an I/O
or scheduling stall on the Orin; no GC, no REST apply, no fsync). The pose age
of 149.7 ms on that row is the tick's own delay (the pose waited in the
executor queue), not a stall before the tick. The code path is older than
71aabb3; ntFIN3's slowest tick was 74.5 ms. The timeline is from the run's bag,
events.jsonl and flight_log.jsonl.

A fix followed the flight and is not in this campaign (CHANGELOG 2026-11-23 (code), 7e6aae2): the generation write, the CSP
compile and the fence publication move after the setpoint, the templates are
compiled at bring-up, and each row carries per-phase tick stamps (CHANGELOG
2026-11-23).

## Replay at the flown revision

654 runs: 355 identical, 298 last float digits (112 plain, 186 through the
body-frame rotation; at most 6.9e-14), 0 not replayable (the 24 GPS-dropout
runs replay since WP4-11's logged pipe clock), 1 differs in a repair's pass
count (`ntFIN4_304_dyn-nfz-rotate-spin`: GeofenceSlide once vs twice on a
convergence test that sits on the last float digit, the same emitted action
to 6.9e-14; as ntFIN3_306). The first replay with the tool at 59e19a5 read the
60 GPS-noise runs as decision differences because it handed the Shield the
true pose; a436d32 rebuilds the perceived pose as the node drew it, and those
runs now replay identical or to the last float digit.

## How the files were made

The re-score and the replay ran in the lab checkout after the campaign; the
KPI report was made from a clean tree at 2f9d863 after the inputs were
committed. Local absolute paths in stored records are repo-relative, and the
scratchpad is written `<scratchpad>`.
