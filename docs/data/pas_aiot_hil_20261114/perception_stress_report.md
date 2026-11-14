Perception stress on Project AirSim, per scenario - non-KPI (perception-stress evidence, WP4-16 / WP4-18).
Baseline: day. Means over each scenario's runs; lock = estimate within 6 m of the truth, loss = lock gone >= 1 s.

| scenario | runs | detector hit rate | mean box score | detector Hz | frame luma | estimate share | target lock | estimate on subject | track losses | longest loss s | within 30 m | range median m | estimate error m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pas-follow-redcar-day | 3 | 0.240 | 0.082 | 2.9 | 97.9 | 0.353 | 0.106 | 0.258 | 3.3 | 279.1 | 0.085 | 143.3 | 116.1 |
| pas-follow-redcar-dusk | 1 | 0.123 | 0.135 | 2.5 | 43.0 | 0.175 | 0.038 | 0.217 | 3.0 | 207.7 | 0.111 | 105.8 | 131.4 |
| pas-follow-redcar-night | 1 | 0.214 | 0.029 | - | 13.5 | - | - | - | - | - | - | - | - |
| pas-follow-redcar-rain | 1 | 0.913 | 0.095 | 2.7 | 97.6 | 0.958 | 0.818 | 0.853 | 8.0 | 22.3 | 0.279 | 34.8 | 4.5 |
| unstressed | 1 | 0.230 | 0.111 | 2.4 | 90.8 | 0.298 | 0.060 | 0.201 | 4.0 | 208.8 | 0.112 | 112.9 | 83.4 |

Checks:
- pas-follow-redcar-night: no subject truth read on ['pas_hil_aiot_night_01']: lock and range not measured
