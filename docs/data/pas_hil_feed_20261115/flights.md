# The VLA feed between ticks on the hil rig (15 Nov 2026)

AIoT-VLA v10b on the AGX Orin, the PAS follow of the red car (CityLife, ArduPilot SITL
4.5.7 and Project AirSim on the desktop, FrontCamera at a 0.05 s capture interval).
Every flight is KPI-grade (hil, sim_speedup 1, clean commit, signed bundle) and has 0 P0
escapes. `obs Hz` is distinct camera frames sent to the model per second: while it was
asked (not held by the rail, not withheld because the detector had no box) and over the
whole mission. `dec Hz` is answers the model parsed, on the same two bases.

| run | commit | feed | ticks | ended | asked s | obs Hz asked | obs Hz mission | dec Hz asked | dec Hz mission | sent between ticks | detector Hz | interventions | ticks > 100 ms | tick max ms | cam to setpoint p50 / p99 ms | collector |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pas_hil_obs_02 | de7e748 | per tick | 968 | target_lost | 28.8 | 9.58 | 2.85 | 9.58 | 2.84 | - of 276 | 2.18 | 15 | 1 of 958 | 280.2 | 158 / 289 | CPython's own |
| pas_hil_feed_01 | ea8ef42 | stream, cap 12 Hz | 970 | target_lost | 29.4 | 9.97 | 3.02 | 9.97 | 3.00 | 38 of 293 | 2.39 | 14 | 1 of 970 | 226.8 | 156 / 251 | CPython's own |
| pas_hil_feed_02 | ea8ef42 | stream, cap 12 Hz | 952 | target_lost | 28.8 | 9.83 | 2.97 | 9.83 | 2.95 | 20 of 283 | 2.10 | 27 | 1 of 952 | 271.8 | 155 / 329 | CPython's own |
| pas_hil_feed_03 | d9f5ea0 | stream, cap 12 Hz | 994 | target_lost | 30.7 | 10.03 | 3.09 | 10.03 | 3.08 | 63 of 308 | 2.30 | 33 | 1 of 984 | 235.5 | 154 / 267 | CPython's own |
| pas_hil_feed_04 | d9f5ea0 | stream, cap 12 Hz | 4185 | max_s reached | 352.1 | 10.01 | 8.39 | 10.01 | 8.26 | 695 of 3524 | 2.50 | 326 | 7 of 4014 | 436.8 | 154 / 251 | CPython's own |
| pas_hil_feed_05 | d9f5ea0 | stream, no cap | 3285 | target_lost | 220.9 | 13.23 | 8.86 | 13.23 | 8.76 | 1760 of 2922 | 2.70 | 204 | 6 of 3192 | 454.1 | 131 / 239 | CPython's own |
| pas_hil_feed_06 | d9f5ea0 | stream, no cap | 980 | target_lost | 27.6 | 12.39 | 3.48 | 12.39 | 3.45 | 190 of 342 | 2.31 | 39 | 1 of 970 | 227.1 | 140 / 246 | CPython's own |
| pas_hil_feed_07 | deac4b3 | stream, no cap | 4198 | max_s reached | 353.4 | 13.15 | 11.07 | 13.15 | 10.93 | 2820 of 4648 | 2.49 | 284 | 3 of 4058 | 152.5 | 134 / 234 | paced: 0 automatic full, paced max 19.8 ms |
| pas_hil_feed_08 | deac4b3 | stream, no cap | 4198 | max_s reached | 340.4 | 13.22 | 10.72 | 13.22 | 10.62 | 2780 of 4500 | 2.64 | 332 | 0 of 4096 | 99.7 | 130 / 238 | paced: 0 automatic full, paced max 19.9 ms |

- pas_hil_obs_02 is the flight before the change (9dfe254 on the desktop, de7e748 on the
  AGX Orin): one request per tick at most.
- pas_hil_feed_01 and _02 (ea8ef42) flew the first version, whose cap applied to the feed
  alone: the tick's own request re-armed it every 100 ms and the feed stayed locked to the
  tick. Fixed in d9f5ea0 (one gate for both).
- pas_hil_feed_03 and _04 (d9f5ea0): cap 12 Hz. pas_hil_feed_05 and _06 (d9f5ea0): no cap.
- pas_hil_feed_07 and _08 (deac4b3): the rig's default since (stream, no cap) and the
  collector paced (guardrail/gc_pacing.py). Both flew the full 420 s with the car held.
- The slow ticks before the pacing were 227-454 ms, one every 65-80 s (pas_hil_feed_04:
  234, 326, 311, 407, 345, 437 ms). With it none is over 152 ms; the 3 left in
  pas_hil_feed_07 (152, 112, 148 ms) are not collections (0 automatic full collections,
  the paced ones at most 19.9 ms and run after the tick's work).
- The detector (OWL-ViT, the source of the hint's phrase) runs at 2.3-2.7 Hz with or
  without the feed. While it has no box the model is not asked, by design.
- Desktop tests were run beside pas_hil_feed_05 / _06 (tests/test_pas_vla.py,
  tests/test_sitl_rails.py, a few minutes); pas_hil_feed_07 / _08 flew alone.
