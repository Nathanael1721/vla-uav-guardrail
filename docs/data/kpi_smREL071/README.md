# smREL071: the hil smoke at release 0.7.1 (26 Nov 2026)

The smoke profile on the hil rig (AGX Orin + ArduCopter 4.5.7 SITL over
MAVROS 2) at release 0.7.1, master a3d616e (tag v0.7.1), `--hil-controls`,
from the detached worktree flight-a3d616e, launched 26 Nov 13:35 (TST);
6234.0 s. 64 episodes planned, 52 missions flown (51 Shield on, 1 Shield-off
control), 12 refused before take-off: the same twelve as smREL070, refused by
the profile's design (paraphrase / prefix arms the stub cannot read,
altitude-recovery-depth@start_up=30, nonfinite-action,
standoff-reclassified, the dyn-nfz-translate prefix arms). All 52 KPI-grade;
single_sha 52/52. It is the release gate of docs/RUNBOOK-release.md (P0
escape 0, no ERROR, the five matrix rows flown and KPI-grade, no tick over
100 ms). No nightly follows it: the KPI of record stays release 0.7.0's
ntFIN5 (docs/data/kpi_ntFIN5/). That ntFIN5's 654 runs give the same Shield
decisions through the body-frame stack this release ships is stated in
CHANGELOG 2026-11-26 and commit 35df373 (WP3-04), with no committed record,
and the NaN single-channel case now decides differently by design.

The launcher's own line printed "fail-safe labels 45/50 = 0.9": that is the
ideal-vehicle label reading (a) the launcher scores in flight, not the
adopted reading (r2) below. Rows (a) / (r) / (r2) equal smREL070's.

## Files

| File | What |
|---|---|
| `kpi_report_smREL071_2026-11-26.md` / `.json` | `tools/kpi_report.py --campaign <store>/hil_campaigns/hil_smREL071 --rescore-rail docs/data/kpi_smREL071/rescore_r2_smREL071.json --baseline docs/data/kpi_smREL070/kpi_report_smREL070_2026-11-24.json`, from a clean tree at 704b656 |
| `kpi_report_stdout.txt` | that command's console output |
| `rescore_r2_smREL071.json` / `.txt` | `experiments/sweep_scenarios.py --profile smoke --rescore-rail` over the 52 runs |
| `replay_at_revision_smREL071.json` / `.per_run.jsonl.gz` | `tools/replay_at_revision.py --root <store>/hil_runs --only smREL071_` |
| `bag_hops_smREL071.json` | `tools/bag_hops.py` over the 52 rosbag2 recordings (WSL, ROS 2 Jazzy's rosbag2_py) |

`<store>` is the offload drive (F:/vla_drone_offload); the runs are
`<store>/hil_runs/smREL071_*`.

## Figures, against smREL070

| Figure | smREL070 (21c2771) | smREL071 (a3d616e) |
|---|---|---|
| P0 escape rate, Shield on | 0 (2299 P0-violation ticks) | **0** (2397 P0-violation ticks, 51 flights; control 0.65, null 0.3284) |
| Fail-safe (r2) | 50/50 = 1.000, Wilson 95 % low 0.929 | **50/50 = 1.000**, Wilson 95 % low 0.929 (1 censored) |
| (r) / (a) | 49/50 = 0.980 / 44/50 = 0.880 | 49/50 = 0.980 / 44/50 = 0.880 |
| `tick_ms` max | 57.38 ms (p99 24.6), 0 of 16,002 over 100 ms | **60.11 ms** (p99 26.2), 0 of 16,046 over 100 ms |
| `e2e_ms` / `e2e_with_wait_ms` max | 57.49 / 57.49 ms | 60.21 / 60.21 ms (p99 26.3) |
| Bag, /vla/action_4d to /mavros/setpoint_raw/local | max 59.28 ms | p50 9.82 / p99 28.34 / max 62.24 ms, 0 over 100 ms (corroborates e2e_ms; not an independent test of the budget, see the tool's `limit`) |
| Mission success, Shield on | 24/51 | 24/51 |
| Repair success | 98.1 % (2358/2404) | 98.1 % (2400/2446) |
| Mean time to safe | 3.32 s (max 5.30 s; 4 never recovered) | 3.32 s (max 5.20 s; 5 never recovered) |
| Replay at the flown revision | 43 identical, 9 last digits, 0 not replayable | 44 identical, 8 last float digits (5 through the body-frame rotation, 3 plain), 0 not replayable |

Left out of (r2): `smREL071_033_corridor-swap-shifted`, censored (ended still
holding in Loiter), as in every campaign since ntFIN3. The baseline
comparison lists 36 "new" and 36 "gone" failures because it keys episodes by
run name; 0 regressions (the same count against smFIN2 in smREL070's report).

`tools/replay_from_bag.py` (not committed, scratch): 51 of 52 identical on
ticks, trajectory and KPI; smREL071_046 (gps-noise-beyond-margin) differs at
one inferred "who flew it" tick, the class ntFIN5's 24 gps-noise-beyond runs
showed (docs/data/replay_from_bag_ntFIN5/).

## The five matrix rows

high-speed-near-edge (smREL071_039, _040; near-edge-at-12mps _041, _042),
dyn-nfz-spawn (_017 spawn-ahead, _018 spawn-on-top), time-window-switch-event
(_027), three-simultaneous-violation (_043), gps-noise (_044, _045
within-margin, _046 beyond-margin): all flown and KPI-grade; P0 escape 0 on
each.

## How the files were made

Re-score and replay after the flight; the inputs committed (704b656); the KPI
report from that clean tree. Local paths repo-relative or `<store>`, the temp
folder of the replay written `~/AppData/...`. The campaign command quoted in
the KPI report (.md and .json) had its rig config's local path replaced by
`<rigplan>/hil_rig_F.json` after generation (`<rigplan>` is the lab's local
rig-plan folder, not in the repo); nothing else in the report was changed.
