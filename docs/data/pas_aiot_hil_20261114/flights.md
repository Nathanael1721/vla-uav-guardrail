# AIoT-VLA hil flights, 14 Nov 2026 (f77efae on the desktop and the AGX Orin; kpi_02 adds --identity --negative-query 'a traffic light')

Parsed from each run's node log (`<run>.node.log`, its `[report]` lines); run folders kept on E:la_drone_offload\jetson_runs and linked under demo/out.

| run | stress | acquired_s | ticks_hz | ticks | interventions | p0_escape_ticks | ended | rtf | kpi_grade | vla_hz | vla_latency_p50_s | obs_hz_asked | cam_to_setpoint_p50_ms | cam_to_setpoint_p99_ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pas_hil_aiot_kpi_01 | none | 166.3 | 9.979 | 4192 | 119 | 0 | max_s reached | 0.968 | yes | 7.747015789214826 | 0.1102 | 9.498 | 161.666 | 287.102 |
| pas_hil_aiot_kpi_02 | none | 155.2 | 9.99 | 1070 | 42 | 0 | target_lost | 0.964 | yes | 2.466092760197986 | 0.1145 | 8.926 | 165.491 | 303.021 |
| pas_hil_aiot_est_01 | - | 176.0 | 9.964 | 4186 | 266 | 0 | max_s reached | 0.967 | yes | 7.830178927464151 | 0.109 | 9.162 | 161.594 | 293.415 |
| pas_hil_aiot_day_02 | day | 138.1 | 9.972 | 4189 | 118 | 0 | max_s reached | 0.965 | no | 7.940969474172927 | 0.1128 | 9.356 | 161.751 | 293.012 |
| pas_hil_aiot_day_03 | day | 163.6 | 9.98 | 4116 | 113 | 0 | target_lost | 0.963 | no | 7.565415822137767 | 0.1107 | 9.407 | 161.109 | 291.413 |
| pas_hil_aiot_day_04 | day | 161.3 | 9.976 | 3638 | 105 | 0 | autopilot took over (RTL) | 0.969 | no | 7.712883898345167 | 0.1127 | 9.39 | 161.831 | 286.764 |
| pas_hil_aiot_dusk_01 | dusk | 169.8 | 9.982 | 3094 | 22 | 0 | target_lost | 0.967 | no | 6.899190613610521 | 0.1098 | 9.494 | 159.353 | 282.239 |
| pas_hil_aiot_night_01 | night | - | - | 0 | 0 | 0 | no mission: start gate timed out after 330.0 s: no plausible 'a red car' within 30 m on 5 inferences in a row | - | no | - | - | - | - | - |
| pas_hil_aiot_rain_01 | rain | 161.6 | 9.963 | 4185 | 297 | 0 | max_s reached | 0.969 | no | 7.666601847258118 | 0.1106 | 9.098 | 163.095 | 296.445 |
| pas_hil_aiot_fog_01 | fog | - | - | 0 | 0 | 0 | no mission: start gate timed out after 330.0 s: no plausible 'a red car' within 30 m on 5 inferences in a row | - | no | - | - | - | - | - |
