# smREL073: the hil smoke at release 0.7.3 (28 Nov 2026)

The smoke profile on the hil rig (AGX Orin + ArduCopter 4.5.7 SITL over
MAVROS 2) at release 0.7.3, master c44a360 (tag v0.7.3), `--hil-controls`,
from the detached worktree flight-c44a360, first episode 28 Nov 06:56 (TST);
6253.3 s. 64 episodes planned, 52 missions flown (51 Shield on, 1 Shield-off
control), 12 refused before take-off: the same twelve as smREL070 to
smREL072, refused by the profile's design. All 52 KPI-grade; single_sha
52/52; the bundle chain verified under the Lab CA in 52/52 (`signed_by`
`lab-ca:71d1e071d3985bc6:ed25519:715f70c4647a9701`, the AGX Orin's key,
`require_chain` true). It is the release gate of docs/RUNBOOK-release.md
(P0 escape 0, no ERROR, the five matrix rows flown and KPI-grade with a
non-vacuous cell each, every planned near-edge-at-12mps cell non-vacuous, no
tick over 100 ms, single_sha): all met (`tools/kpi_report.py`: "Shield matrix
... near-edge-at-12mps 2/2 (planned 2/2 met): met").

What is new against smREL072:

- **The near-edge-at-12mps row is non-vacuous.** The cell's scenario clock
  is pinned (`mission.clock_start` 2026-09-07T09:00:00, a Monday, inside the
  school-yard zone's Mon-Fri 07:30-17:30 window), so the zone the pass runs
  beside is active: smREL073_041 (ask 12 m/s) 10 P0-violation ticks, 10
  repairs; smREL073_042 (ask 20 m/s) 13 P0-violation ticks, 160 repairs; P0
  escape 0 on both, mission success on both.
- **The manifest start frame goes out at arming.** 52 of 52 runs record
  `manifest_frames.start` `at: arming` with all six fields
  (code_revision, vla_model_hash, policy_hash, random_seed, sim_speedup,
  topology; topology `declared`). `tools/replay_from_bag.py` compares the
  bag's start frame with manifest.json: `manifest_in_bag` identical in 52 of
  52.
- **The episode bundle's episode.bag2 begins with that frame.** 49 of the 52
  episode bundles (`<store>/hil_campaigns/hil_smREL073/episode-*`) hold the
  recording trimmed to begin at the start frame (guardrail/episode_bag.py;
  2,107 to 3,610 messages of MAVROS telemetry and bring-up before it left
  out, the latched values carried). In 3 (altitude-floor-recovery,
  corridor-curfew-out-of-hours, gps-noise-within-margin@latency=2) one
  /vla/action_4d message lay between the autopilot's arm acknowledgement and
  the frame; the trim refuses to leave out a message of the episode, so
  those bundles keep the recorder's bag as recorded and their kpi.json
  (`episode_bag.trim.why`) says so.
- **report.md names the flown setpoint path** (`setpoint_path` in
  metrics.json, from the subscribers seen on /shield/setpoint), 52 of 52:
  `vla node -> /vla/action_4d (body) -> safety_shield -> /shield/setpoint
  (body) -> mavlink_adapter -> /mavros/setpoint_raw/local -> MAVROS 2 ->
  ArduPilot`.

The launcher's own line printed "fail-safe labels 45/50 = 0.9": that is the
ideal-vehicle label reading (a) the launcher scores in flight, not the
adopted reading (r2) below.

## Files

| File | What |
|---|---|
| `kpi_report_smREL073_2026-11-28.md` / `.json` | `tools/kpi_report.py --campaign <store>/hil_campaigns/hil_smREL073 --rescore-rail docs/data/kpi_smREL073/rescore_r2_smREL073.json --baseline docs/data/kpi_smREL072/kpi_report_smREL072_2026-11-27.json`, from a clean tree at f9cab68, where the WP1 round trip (`docs/data/wp1_roundtrip.json`) and the Prefix Compiler KPIs (`docs/data/csp_kpis.json`) are the records at c44a360, this release |
| `kpi_report_stdout.txt` | that command's console output |
| `rescore_r2_smREL073.json` / `.txt` | `experiments/sweep_scenarios.py --profile smoke --rescore-rail` over the 52 runs |
| `replay_at_revision_smREL073.json` / `.per_run.jsonl.gz` | `tools/replay_at_revision.py --root <store>/hil_runs --only smREL073_` on the desktop |
| `bag_hops_smREL073.json` | `tools/bag_hops.py` over the 52 rosbag2 recordings (WSL, ROS 2 Jazzy's rosbag2_py) |
| `../replay_agx_smREL073/` | the same replay on the AGX Orin, in the hil companion container |

`<store>` is the offload drive (F:/vla_drone_offload); the runs are
`<store>/hil_runs/smREL073_*`.

## Figures, against smREL072

| Figure | smREL072 (418b151) | smREL073 (c44a360) |
|---|---|---|
| P0 escape rate, Shield on | 0 (2378 P0-violation ticks) | **0** (2393 P0-violation ticks, 51 flights; control 0.65, null 0.3277) |
| Fail-safe (r2) | 50/50 = 1.000, Wilson 95 % low 0.929 (1 censored) | **50/50 = 1.000**, Wilson 95 % low 0.929 (1 censored) |
| (r) / (a) | 49/50 = 0.980 / 44/50 = 0.880 | 49/50 = 0.980 / 44/50 = 0.880 |
| `tick_ms` max | 59.34 ms (p99 26.0), 0 of 16,150 over 100 ms | **55.73 ms** (p99 25.5), 0 of 16,047 over 100 ms |
| `e2e_ms` / `e2e_with_wait_ms` max | 59.43 / 59.43 ms (p99 26.1) | 55.81 / 55.81 ms (p99 25.7) |
| Bag, /vla/action_4d to /mavros/setpoint_raw/local | p50 9.99 / p99 28.43 / max 60.90 ms | p50 10.10 / p99 27.69 / max 57.69 ms, 0 over 100 ms (corroborates e2e_ms; not an independent test of the budget, see the tool's `limit`) |
| Mission success, Shield on | 24/51 | 24/51 |
| Repair success | 98.0 % (2375/2424) | 98.1 % (2393/2439) |
| Mean time to safe | 3.36 s (max 5.30 s; 5 never recovered) | 3.31 s (max 5.30 s; 4 never recovered) |
| Replay at the flown revision, desktop (AMD64, Python 3.10) | 41 identical, 11 last float digits, 0 not replayable | 42 identical, 10 last float digits, 0 not replayable |
| Replay at the flown revision, AGX Orin (aarch64, Python 3.12.3, the node's runtime) | - | **52 identical**, 0 differs, 0 not replayable |

Left out of (r2): `smREL073_033_corridor-swap-shifted`, censored (ended
still holding in Loiter 2.8 s into the 30 s operator timeout).

The baseline comparison lists 36 "new" and 36 "gone" failures because it
keys episodes by run name; 0 regressions.

`tools/replay_from_bag.py` (scratch): 51 of 52 identical on ticks,
trajectory and KPI; smREL073_046 (gps-noise-beyond-margin) differs at one
inferred "who flew it" tick, as smREL071_046 and smREL072_046 did.

## The five matrix rows

high-speed-near-edge (smREL073_039, _040; near-edge-at-12mps _041, _042),
dyn-nfz-spawn (_017 spawn-ahead, _018 spawn-on-top), time-window-switch-event
(_027), three-simultaneous-violation (_043), gps-noise (_044, _045
within-margin, _046 beyond-margin): all flown and KPI-grade; P0 escape 0 on
each; each row has a non-vacuous cell (high-speed near-edge 4/4 cells,
dyn-nfz spawn 1/2, time-window switch 3/5, three simultaneous violations 1/1,
gps noise / latency 2/3).

## How the files were made

Re-score and replay after the flight; the inputs committed (5379b2a); the KPI
report from a clean tree after them (f9cab68, which also holds the WP1 and
WP2 records at c44a360). Local paths repo-relative or `<store>`, the temp
folder of the desktop replay written `~/AppData/...`. The campaign command
quoted in the KPI report (.md and .json) had its rig config's local path
replaced by `<rigplan>/hil_rig_F.json`, and the report's own `--out` path by
`docs/data/kpi_smREL073/`, after generation; nothing else in the report was
changed.
