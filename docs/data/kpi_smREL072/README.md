# smREL072: the hil smoke at release 0.7.2 (27 Nov 2026)

The smoke profile on the hil rig (AGX Orin + ArduCopter 4.5.7 SITL over
MAVROS 2) at release 0.7.2, master 418b151 (tag v0.7.2), `--hil-controls`,
from the detached worktree flight-418b151, launched 27 Nov 16:52 (TST);
6288.3 s. 64 episodes planned, 52 missions flown (51 Shield on, 1 Shield-off
control), 12 refused before take-off: the same twelve as smREL070 and
smREL071, refused by the profile's design. All 52 KPI-grade; single_sha
52/52 (the gate release 0.7.2 added to the KPI readers). It is the release
gate of docs/RUNBOOK-release.md (P0 escape 0, no ERROR, the five matrix rows
flown and KPI-grade, no tick over 100 ms): all four met. No nightly follows
it: the KPI of record stays release 0.7.0's ntFIN5 (docs/data/kpi_ntFIN5/).

What is new against smREL071:

- The first smoke with the policy bundles under the Lab CA
  (docs/data/bundle_ca_20261127/): every one of the 52 runs records the
  bundle `signed_by` `lab-ca:71d1e071d3985bc6:ed25519:715f70c4647a9701` (the
  AGX Orin's key), `chain` `verified` and `require_chain` true (kpi.json).
- The first smoke with the hot-apply generations row in each run's report.md
  (WP3-19), e.g. smREL072_035: `1: g1 t=3.05 s (scenario event: #0
  spawn_polygon_fence rtl-zone)`.
- The repair selection recorded per run (`shield_forecast.repair_mode` in
  metrics.json): `chain` in 52 of 52, the default, the selection every
  earlier flight flew.

The launcher's own line printed "fail-safe labels 45/50 = 0.9": that is the
ideal-vehicle label reading (a) the launcher scores in flight, not the
adopted reading (r2) below. Rows (a) / (r) / (r2) equal smREL071's.

## Files

| File | What |
|---|---|
| `kpi_report_smREL072_2026-11-27.md` / `.json` | `tools/kpi_report.py --campaign <store>/hil_campaigns/hil_smREL072 --rescore-rail docs/data/kpi_smREL072/rescore_r2_smREL072.json --baseline docs/data/kpi_smREL071/kpi_report_smREL071_2026-11-26.json`, from a clean tree at e5aa51d |
| `kpi_report_stdout.txt` | that command's console output |
| `rescore_r2_smREL072.json` / `.txt` | `experiments/sweep_scenarios.py --profile smoke --rescore-rail` over the 52 runs |
| `replay_at_revision_smREL072.json` / `.per_run.jsonl.gz` | `tools/replay_at_revision.py --root <store>/hil_runs --only smREL072_` |
| `bag_hops_smREL072.json` | `tools/bag_hops.py` over the 52 rosbag2 recordings (WSL, ROS 2 Jazzy's rosbag2_py) |

`<store>` is the offload drive (F:/vla_drone_offload); the runs are
`<store>/hil_runs/smREL072_*`.

## Figures, against smREL071

| Figure | smREL071 (a3d616e) | smREL072 (418b151) |
|---|---|---|
| P0 escape rate, Shield on | 0 (2397 P0-violation ticks) | **0** (2378 P0-violation ticks, 51 flights; control 0.65, null 0.3266) |
| Fail-safe (r2) | 50/50 = 1.000, Wilson 95 % low 0.929 (1 censored) | **50/50 = 1.000**, Wilson 95 % low 0.929 (1 censored) |
| (r) / (a) | 49/50 = 0.980 / 44/50 = 0.880 | 49/50 = 0.980 / 44/50 = 0.880 |
| `tick_ms` max | 60.11 ms (p99 26.2), 0 of 16,046 over 100 ms | **59.34 ms** (p99 26.0), 0 of 16,150 over 100 ms |
| `e2e_ms` / `e2e_with_wait_ms` max | 60.21 / 60.21 ms (p99 26.3) | 59.43 / 59.43 ms (p99 26.1) |
| Bag, /vla/action_4d to /mavros/setpoint_raw/local | p50 9.82 / p99 28.34 / max 62.24 ms | p50 9.99 / p99 28.43 / max 60.90 ms, 0 over 100 ms (corroborates e2e_ms; not an independent test of the budget, see the tool's `limit`) |
| Mission success, Shield on | 24/51 | 24/51 |
| Repair success | 98.1 % (2400/2446) | 98.0 % (2375/2424) |
| Mean time to safe | 3.32 s (max 5.20 s; 5 never recovered) | 3.36 s (max 5.30 s; 5 never recovered) |
| Replay at the flown revision | 44 identical, 8 last float digits, 0 not replayable | 41 identical, 11 last float digits (8 through the body-frame rotation, 3 plain), 0 not replayable |

Left out of (r2): `smREL072_029_rule-activated-mid-run`, censored (ended
still holding in Loiter 4.1 s into the 30 s operator timeout). In smREL071
the censored run was corridor-swap-shifted, which this time ended in a scored
outcome.

`smREL072_035_failsafe-rtl-zone` flew RTL, then the FSM's G6 edge (RTL
failed: no progress home, -0.10 m in 5.0 s) and Land; smREL071's flight of
the same cell reached home under RTL. The launcher's ideal-vehicle reading
(a) counts that as an outcome mismatch; the adopted reading (r2) counts it
correct. The no-progress watch (guardrail/rtl_path.py) did not change between
0.7.1 and 0.7.2; the same cell took G6 in 6 of 18 ntFIN5 runs and 9 of 18
ntFIN4 runs.

The baseline comparison lists 36 "new" and 36 "gone" failures because it
keys episodes by run name; 0 regressions (as smREL071's report against
smREL070).

`tools/replay_from_bag.py` (not committed, scratch): 51 of 52 identical on
ticks, trajectory and KPI; smREL072_046 (gps-noise-beyond-margin) differs at
one inferred "who flew it" tick, as smREL071_046 did.

## The five matrix rows

high-speed-near-edge (smREL072_039, _040; near-edge-at-12mps _041, _042),
dyn-nfz-spawn (_017 spawn-ahead, _018 spawn-on-top), time-window-switch-event
(_027), three-simultaneous-violation (_043), gps-noise (_044, _045
within-margin, _046 beyond-margin): all flown and KPI-grade; P0 escape 0 on
each.

## How the files were made

Re-score and replay after the flight; the inputs committed (e5aa51d); the KPI
report from that clean tree. Local paths repo-relative or `<store>`, the temp
folder of the replay written `~/AppData/...`. The campaign command quoted in
the KPI report (.md and .json) had its rig config's local path replaced by
`<rigplan>/hil_rig_F.json`, and the report's own `--out` path by
`docs/data/kpi_smREL072/`, after generation; nothing else in the report was
changed.
