# deploy/evidence - what each host measured

One folder per topology. A file here is a measurement, with the host facts and
the command that produced it inside it; regenerate it with that command, never
by hand.

| Path | Produced by | What it is |
|---|---|---|
| `shield_tick_pack.json` | `python tools/profile_shield_tick.py pack` (desktop, needs `demo/out`) | The portable input for the Shield tick profile: the states, actions, subjects, policies and obstacle map of four real flights on two rails, so the Orin replays exactly what the desktop replays. Each flight's `rail` is today's name for it (`dev` for the three ArduPilot SITL + MAVROS 2 flights, which recorded the retired label `canonical-hil`, kept as `rail_recorded`). None of them is KPI-bearing. |
| `vla_protocol_frames/` | copied from `experiments/out/frames` | The two camera inputs of the R1 rate protocol (`tools/vla_backend_table.py protocol`), with their poses and SHA-256. |
| `dev/shield_tick_profile.json` | `python tools/profile_shield_tick.py profile --topology dev --cpus 0,2,4,6,8,10,12,14` | The desktop baseline (performance cores, one logical CPU per core) of the Shield's per-tick cost against the grant's 5 ms query and 100 ms tick budgets. |
| `dev/shield_tick_profile-ecores.json` | the same with `--cpus 16-27` | The same on the desktop's efficiency cores. |
| `dev/vla_rate_<backend>.json` | `python tools/vla_backend_table.py measure --backend <b> --topology dev` | R1 inference rate on the desktop. |
| `hil/shield_tick_profile.json` | the profile on the Orin (`docker compose ... run --rm profile`), copied from `incoming/hil/` | **Not yet measured.** The number the grant's "Orin profiling" asks for. |
| `hil/vla_rate_<backend>.json` | `docker compose ... --profile vla-bench run --rm vla-bench` on the Orin, copied from `incoming/hil/` | **Not yet measured.** |
| `incoming/` | the Orin's containers (`VLAGUARD_EVIDENCE_OUT`) | Git-ignored landing folder, so a measurement on the Orin never modifies a tracked file. Reviewed files are moved to `<topology>/` on the desktop and committed. |
| `vla_backend_table.json`, `.md` | `python tools/vla_backend_table.py table` | The backend comparison table: what is on disk, under which conditions, and which cells wait for the Orin. |

`tools/profile_shield_tick.py` refuses to label a result `hil` or `flight` on
a host that guardrail.manifest's Orin rule does not accept. To attach a
profile to a run, put the output of
`python tools/profile_shield_tick.py attach <profile.json>` beside the
manifest in the run's `metrics.json`; the manifest itself stays the grant's six
fields.
