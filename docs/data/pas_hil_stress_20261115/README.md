# Texture and shadow perception stress on the hil rig, 15 Nov 2026

Runs (commit 8c9448e on the desktop and the AGX Orin; topology hil,
sim_speedup 1.0; AIoT-VLA v10b with `--identity --negative-query "a traffic
light" --target-estimator`, as the unstressed baseline pas_hil_aiot_est_01 of
the same day):

| run | stressor | ended | ticks (Hz) | interventions | P0 escape ticks |
|---|---|---|---|---|---|
| pas_hil_aiot_est_01 | none (baseline, 3918965) | max_s reached | 4186 (9.964) | 266 | 0 |
| pas_hil_aiot_texture_01 | texture: 347/347 road meshes on WorldGridMaterial | target_lost | 1008 (9.985) | 24 | 0 |
| pas_hil_aiot_shadow_01 | shadow: DirectionalLight_0 at pitch -8.0, yaw 49.691 deg (read back on the Orin) | max_s reached | 4186 (9.963) | 300 | 0 |

perception_stress_report.md / .json: `python tools/perception_stress_report.py
demo/out/pas_hil_aiot_est_01 demo/out/pas_hil_aiot_texture_01
demo/out/pas_hil_aiot_shadow_01 --json ... --md ...`. Stress runs are non-KPI
by design (WP4-16). Run folders on E:\vla_drone_offload\jetson_runs, linked
under demo/out.
