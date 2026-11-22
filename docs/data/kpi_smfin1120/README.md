# smfin1120: the hil smoke at the final revision 71aabb3 (20-21 Nov 2026)

The smoke profile flown on the hil rig (AGX Orin + ArduCopter 4.5.7 SITL over
MAVROS 2) at master 71aabb3, the revision the final report describes, as the
regression check against the release smoke rel060 (v0.6.0 = a85dc91,
`docs/data/kpi_rel060/`). Launched 20 Nov 22:23 from the detached worktree
flight-71aabb3 with `--hil-controls`; 6059.6 s. 64 episodes planned, 49
missions flown, 15 refused before take-off (stressors the hil rail cannot
realise, as in every hil campaign). All 49 KPI-grade.

## Files

| File | What |
|---|---|
| `kpi_report_smfin1120_2026-11-21.md` / `.json` | `tools/kpi_report.py --campaign demo/out/hil_smfin1120 --rescore-rail docs/data/kpi_smfin1120/rescore_r2_smfin1120.json --baseline docs/data/kpi_rel060/kpi_report_rel060_2026-11-19.json` |
| `rescore_r2_smfin1120.json` / `.txt` | `experiments/sweep_scenarios.py --profile smoke --rescore-rail` over the 49 runs (labels `docs/data/failsafe_labels_rail.json`) |
| `replay_at_revision_smfin1120.json` / `.per_run.jsonl.gz` | `tools/replay_at_revision.py` over the 49 runs at their flown revision |
| `xrev_rel060_inputs_at_71aabb3.json` / `.per_run.jsonl.gz` | two rel060 runs replayed open loop at 71aabb3 (see "The false trigger") |

## Figures, against rel060

| Figure | rel060 (a85dc91) | smfin1120 (71aabb3) |
|---|---|---|
| KPI-grade missions | 49 (48 Shield on, 1 control) | 49 (48 Shield on, 1 control) |
| P0 violation escape rate, Shield on | 0 on 2220 P0-violation ticks | **0** on 2201 P0-violation ticks |
| Shield-off control / passthrough null | 0.65 / 0.3308 | 0.65 / 0.3417 |
| Fail-safe trigger correctness, (r2) adopted | 48/48 = 1.000, Wilson 95 % low 0.926 | **46/47 = 0.979**, Wilson 95 % low 0.889 |
| (r) / (a) / the campaign's own labels | 47/48 / 42/48 / 43/48 | 45/47 / 40/47 / 41/47 |
| Left out of (r2) | 0 | 1 (censored in Loiter: corridor-swap-shifted) |
| `tick_ms` (hil) | p99 26.2 / max 70.89 ms | p99 31.4 / max 78.84 ms, 0 of 15,389 ticks over 100 ms |
| `e2e_ms` (hil) | p99 26.3 / max 70.98 ms | p99 31.5 / max 78.93 ms, 0 of 15,389 ticks over 100 ms |
| Mission success, Shield on | 22/48 | 22/48 |
| Mean time to safe | 3.73 s | 3.23 s (13 spells; 4 never recovered) |
| Repair success | 98.2 % (2309/2351) | 98.0 % (2256/2302) |
| Repairs per Shield-on mission | 68.1 | 67.1 |
| Replay at the flown revision | 37 identical, 10 last float digits, 2 not replayable | 33 identical, 14 last float digits (at most 3.1e-15), 2 not replayable |

The report's "against the baseline" section lists 35 new failures and 35
gone: it matches episodes by run name, and rel060_NNN and smfin1120_NNN never
match. Its per-family figure table is the comparison.

The 2 runs not replayable are the gps-dropout cells: the pipe the Shield
dead-reckons on is not logged per tick (as in rel060).

PathRepair was in the default chain on every Shield-on mission
(`path_repair: true` in 49 of 49 metrics.json) and acted on 0 ticks.

## The fail-safe figure below 0.99

Under (r2) the smoke scores 46/47. On 47 scored runs one error is 0.979, so
this smoke cannot show 0.99 with any error. The grant's figure is read on the
nightly (ntFIN2, 584/585 at 116214f; the nightly at 71aabb3 is ntFIN3).

**The false trigger: `smfin1120_021_dyn-nfz-rotate-spin` (deg_per_s 3, label
no fail-safe).** Both flights entered Loiter on G3 (repair over theta).

- rel060: the Loiter began at x -188.6; the hold-point search found a point
  1 m back, (-189.6, -0.1), legal for the whole 30 s the Loiter can last
  (`hold_point` moved, checked 35 s). There the GeofenceSlide repair fell
  under theta (2.0 m) at 7.9 s and G8 handed back at 10.0 s. No fail-safe.
- smfin1120: the Loiter began at x -188.1, 0.5 m closer. The search found
  no point legal for 30 s within its budget (600 units) and held in place
  (`hold_point` in_place, checked 5 s). At 25.3 s the rotating zone made the
  held point break `bar` within the 5 s lookahead; the moved point then
  found, (-189.5, -1.5), was reached at 27.6 s. The repair stayed at 1.55-3.0 m (under theta only from
  27.5 s), G8 never came, and the Loiter's 30 s operator timeout took G5 to
  RTL at 34.3 s.

**The code is not the cause.** Two rel060 runs (rotate-spin and
corridor-swap-shifted) were replayed open loop at 71aabb3 on their own logged
inputs, with `tools/replay_at_revision.py` after their copied manifests'
`code_revision` was set to 71aabb3 (copies in the scratchpad; the run folders
were not touched): corridor-swap-shifted identical; rotate-spin differs only
in the last float digits (at most 4.0e-14 on the repair fields, 1.1e-14 on
the emitted action). 71aabb3 makes rel060's decisions on rel060's inputs.
The two flights differ in where the airframe was when the Loiter began.

**What it shows:** the hold-point search is budget-bound. A Loiter that
begins a little closer to a zone that is moving toward it can be held in
place, and then the moving zone, not the pilot, ends the mission in RTL.
Recorded as an open finding; no code changed after the flight.

**corridor-swap-shifted** (`smfin1120_033`) ended in Loiter at its flight cap
(censored, left out of every reading), as ntFIN2_374 and hh02_005 did; rel060's
flight of the cell was not censored.

## How the files were made

The re-score and the replays ran in the lab checkout at 71aabb3 the night of
the flight; the KPI report was made again on 22 Nov from a clean tree at
668d8ab (`tools/kpi_report.py`, the command in the report's header), after
the inputs above were committed. Local absolute paths in the stored records
(the campaign's own command, the run folders given to the replay) are
written repo-relative, and the scratchpad as `<scratchpad>`.
