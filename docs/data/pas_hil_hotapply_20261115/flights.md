# REST hot-apply and the GeoFence on the hil rail, 15 Nov 2026

Three Project AirSim + ArduPilot SITL flights with the Shield and AIoT-VLA on the AGX Orin, MAVROS 2 in the companion container (`deploy/orin/run_pas_perception.sh --save-view --identity --negative-query "a traffic light" --target-estimator --hotapply-port 8071`). Events were sent over plaintext REST on the Orin's loopback after the mission started. Parsed from each run's metrics.json, events.jsonl and node log; run folders on E:la_drone_offload\jetson_runs, linked under demo/out.

| run | commit | setpoint route | KPI-grade | ticks (Hz) | interventions | P0 escape ticks | REST events | receipt to swap ms (p50 / max) | over 50 ms | GeoFence uploads ok |
|---|---|---|---|---|---|---|---|---|---|---|
| pas_hil_hotapply_01 | 6c9698d | node | YES | 4176 (9.941) | 257 | 0 | 2 | 0.705 / 2.353 | 0 | 1/2 |
| pas_hil_adapter_01 | 5b3db86 | adapter | YES | 4187 (9.967) | 273 | 0 | 2 | 16.209 / 53.048 | 1 | 2/2 |
| pas_hil_hotapply_02 | 00264ef | node | YES | 4177 (9.944) | 300 | 0 | 10 | 3.075 / 6.347 | 0 | 10/10 |

## Events

- pas_hil_hotapply_01: spawn nfz-rest-1 g1 2.353 ms; expire nfz-rest-1 g2 0.705 ms
- pas_hil_adapter_01: spawn nfz-rest-1 g1 53.048 ms; expire nfz-rest-1 g2 16.209 ms
- pas_hil_hotapply_02: spawn nfz-rest-1 g1 3.28 ms; expire nfz-rest-1 g2 3.075 ms; spawn nfz-rest-2 g3 5.387 ms; expire nfz-rest-2 g4 0.87 ms; spawn nfz-rest-3 g5 3.656 ms; expire nfz-rest-3 g6 0.811 ms; spawn nfz-rest-4 g7 2.135 ms; expire nfz-rest-4 g8 0.83 ms; spawn nfz-rest-5 g9 4.738 ms; expire nfz-rest-5 g10 6.347 ms

## GeoFence uploads after each generation

- pas_hil_hotapply_01: g0 not sent; g1 ok (read back 4); g2 FAILED (read back None)
- pas_hil_adapter_01: g0 not sent; g1 ok (read back 4); g2 ok (read back 0, MAVROS answered failure)
- pas_hil_hotapply_02: g0 not sent; g1 ok (read back 4); g2 ok (read back 0, MAVROS answered failure); g3 ok (read back 4); g4 ok (read back 0, MAVROS answered failure); g5 ok (read back 4); g6 ok (read back 0, MAVROS answered failure); g7 ok (read back 4); g8 ok (read back 0, MAVROS answered failure); g9 ok (read back 4); g10 ok (read back 0, MAVROS answered failure)
