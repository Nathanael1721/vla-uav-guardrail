# pdyn1126: generation N's CSP to the prefix consumer on dynamic suite cells, hil (26 Nov 2026)

experiments/profiles/prefix_dyn.yaml on the hil rig (AGX Orin + ArduCopter
4.5.7 SITL over MAVROS 2) at release 0.7.1 (a3d616e), `--csp-topic on`,
from the detached worktree flight-a3d616e; 1727.8 s. 12 episodes planned, 12
flown, 0 refused; all 12 KPI-grade (per-run kpi.json). Three dynamic suite
scenarios, each with a mid-flight policy change applied by the Shield node
from the scenario's events: dyn-nfz-spawn-ahead@ahead_m=12,variant=0,
vehicle_speed_mps=4 (a stationary zone spawned ahead at ~5 s),
corridor-swap-narrowed@ask_mps=2 (the corridor swapped for a narrower one at
3 s), radius-scaling-grow@factor=3.3 (a zone spawned at 0 s and grown at
10 s); the Prefix Compiler's CSP consumed (prefix on) or not (prefix off), seeds
1001 and 1002. The pilot is the stub (corridor-swap-narrowed:
ConstantPilot; the other two: GotoPilot) inside
guardrail.prefix_consumer.PrefixConsumer, the grant's action-set adapter;
on the prefix-on arm it compiles and swaps in each generation's CSP from
`/shield/csp`, on the prefix-off arm its adapter is off. It is not a learned
VLA.

## Files

| File | What |
|---|---|
| `rescore_r2_pdyn1126.json` / `.txt` | `experiments/sweep_scenarios.py --profile prefix_dyn --rescore-rail "<store>/hil_runs/pdyn1126_*"` |
| `replay_at_revision_pdyn1126.json` / `.per_run.jsonl.gz` | `tools/replay_at_revision.py --root <store>/hil_runs --only pdyn1126_` |
| `bag_hops_pdyn1126.json` | `tools/bag_hops.py` over the 12 rosbag2 recordings (WSL, rosbag2_py) |
| `kpi_report_pdyn1126_2026-11-27.md` / `.json` | `tools/kpi_report.py --campaign <store>/hil_campaigns/hil_pdyn1126 --rescore-rail docs/data/kpi_prefix_dyn_20261126/rescore_r2_pdyn1126.json --out docs/data/kpi_prefix_dyn_20261126/kpi_report_pdyn1126_2026-11-27`, from a clean tree at a44d636 (27 Nov); machine paths replaced by `<store>` and `<scratchpad>` |
| `kpi_report_stdout_pdyn1126.txt` | that command's console output |
| `headless_prefix_dyn_2026-11-27.json` / `.txt` | `experiments/sweep_scenarios.py --profile prefix_dyn --no-bundles --out ...`, the same 12 cells headless, from a clean tree at 02a6fd9 (27 Nov), and its console output |

The campaign's own summary is `<store>/hil_campaigns/hil_pdyn1126/{sweep.json,kpi_report.md}`
(`<store>` = the offload drive). Its `_manifest_common` reads topology
`headless-kinematic` and the headless pilot's hash: a summarizer defect (every
rail campaign's summary carried the headless sweep's base), not the flights.
The 12 per-run `manifest.json` files say topology `hil`, sim_speedup 1.0 and
`guardrail.prefix_consumer.PrefixConsumer`, and all 12 are KPI-grade by
`is_kpi_grade` (the KPI report above). Fixed in a44d636
(`campaign_manifest_common`, test in tests/test_sweep.py) for campaigns flown
after it; the stored sweep.json is left as written.

## Figures

| Figure | Value |
|---|---|
| Gate status | 10 pass, 2 fail (both corridor-swap-narrowed prefix-on: pdyn1126_005, _007; gate `breaches max 0` read 1) |
| P0 escape rate | 0 on all 12 (true state: 0 Shield-on episodes flew into a P0 polygon; 4 were inside only because a zone appeared or grew under them) |
| Fail-safe (r2) | 6/6 labelled (the prefix-on arms are scored on their gates, not on labels derived for a rule-ignorant pilot) |
| Generation N's CSP at the pilot | every generation on every prefix-on run, 0.090-0.127 s after the Shield's own generation (flight_log `pilot_csp_generation`); radius-scaling-grow: generations 1 and 2 |
| Prefix on vs off (6 pairs, all distinct flights) | mission success 0.333 vs 0.000; P0 escapes 0 vs 0; repairs / episode 13.3 vs 3.3; no A/A pair flown, so no noise floor |
| `tick_ms` / `e2e_with_wait_ms` max (node) | 52.14 / 58.08 ms over 8,025 ticks, 0 over 100 ms |
| Bag, action to MAVROS setpoint | p99 26.37 ms; max 151.63 ms on pdyn1126_007 (below) |
| Replay at the flown revision | 9 identical, 3 last float digits (2 plain, 1 through the body-frame rotation), 0 not replayable |
| Headless, the same 12 cells (a3d616e) | 12/12 pass |
| Headless, the same 12 cells, re-run (02a6fd9, 27 Nov) | 12/12 pass; P0 escape 0; fail-safe labels 6/6 (6 unlabelled prefix-on arms); 3 distinct flight pairs (the headless backend does not vary these cells by seed); prefix on vs off mission success 0.667 vs 0.000 |
| KPI report (a44d636, 27 Nov) | 12 episodes, 12 KPI-grade, single_sha 12/12; P0 escape 0.0000 (0 escapes on 67 P0-violation ticks); fail-safe (r2) 6/6 = 1.000, Wilson 95 % low 0.610; `tick_ms` max 52.14 ms, `e2e_with_wait_ms` max 58.08 ms, 0 of 4,756 / 4,754 ticks over 100 ms; mission success 2/12 |

## The two failures (pdyn1126_005, _007), as diagnosed

The swap at 3.0 s narrows the corridor while the aircraft flies at 2 m/s
toward the new edge, about 1 m from it. The FSM goes Normal -> Brake on the
swap tick (X1, "repair over theta ... projection abandoned"). On the prefix-on
arm the consumer has generation 1's CSP 0.12 s later and turns away; the
Shield, still in Brake, passes that legal turned action. The aircraft's own
momentum carries it out of the narrowed corridor at about 3.5-3.6 s (first
unsafe tick 3.467 s on _005, 3.608 s on _007), to 0.81 / 0.64 m outside at
about 4.3-4.4 s (4.359 / 4.309 s); the Shield then flies the escape (body vy
about -1.8 m/s) and it is back inside by ~5.2 s (G7 to Normal). Because the
Shield was flying when the position became unsafe, under the rules already
in force, the scorer counts it as a breach. The prefix-off arms overshoot
the same way (_006 0.70 m, _008 0.72 m outside, at about 4.25 s; their
later 9.99 m depth is in RTL, about 71 s in), but
in the autopilot's Loiter after Brake, which the scorer attributes to the
autopilot, not to the flight. Headless, where the vehicle model stops on the
commanded velocity, the same cells pass. The gate is left as it was set; the
two failures are reported as flown.

## The 151 ms bag hop (pdyn1126_007)

Not a Shield delay. The stub pilot published no action for ~146 ms twice
(t = 74.5 s and 80.05 s); the node's 10 Hz watchdog ran a timer tick
(`tick_trigger` "timer", `action_age_ms` 149.9 / 150.9) and re-published the
last action. Receive-time pairing reads that setpoint against the old action.
The node's own receipt-to-setpoint time for those actions was 3.1 and 4.6
ms, and its `e2e_ms` max in that run 58.08 ms (tick 718); the tool's own
`limit` field says receive-time pairing corroborates e2e_ms and is not an
independent test of the budget.

`tools/replay_from_bag.py` (scratch): 11 of 12 identical; pdyn1126_007 differs
at tick 1 (the window's first action aligned one action early), a replay-tool
alignment on that run, not a flight difference.
