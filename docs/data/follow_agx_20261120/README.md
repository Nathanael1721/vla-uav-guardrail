# CityLife follow on the AGX Orin, 20 Nov 2026 (stand-off edge, descent through the Shield)

The re-flights the 20 Nov follow-rail code owed (CHANGELOG 2026-11-20,
"follow rail: stand-off edge, descent through the Shield"): FenceGuard no
longer holds every velocity inside a zone's 3 m stand-off, and the landing
descent goes through the Shield. Same rig and flags as
docs/data/onepath_agx_20261119/: the hand-written controller
(demo/follow_vlm.py) ON the Jetson AGX Orin against Project AirSim CityLife
on the desktop (`deploy/orin/start_sim_for_jetson.ps1 -SimMaxFps 30
-SimOffscreen`, `deploy/orin/run_follow_controller.sh`), red car, 420 s after
the start gate.

- **Code:** 28a594d on the AGX (git bundle from the lab checkout, clean).
- **Not KPI-grade, by design:** topology `projectairsim-remote-sim`
  (simple_flight, no ArduPilot); functional evidence for the follow, the
  guard and the landing.

| Run | Policy | Pilot | Within 30 m | P0 escape | NFZ | Contacts in mission |
|---|---|---|---|---|---|---|
| nfz3b_agx_hier_aerovla_20261120 | follow_car_citylife_nfz3b | AerialVLA hierarchy (:8790) | 0.48 | 0.0 | 0.0 s | 0 |
| agx_onepath_shield_06 | follow_car_citylife_nfz | hand-written | 0.945 | 0.0 | 0.0 s | 0 |

The same flights on 19 Nov at bfb4973 (docs/data/onepath_agx_20261119/):
nfz3b_agx_hier_aerovla_wgs84_20261119 0.345 (one flight); onepath n = 5
(agx_onepath_shield_01 to _05) 0.969 / 0.954 / 0.472 / 0.943 / 0.917, median
0.943. agx_onepath_shield_06's 0.945 is within that spread, not a gain.

Each subfolder holds the run's `metrics.json`, `kpi.json` and
`manifest.json` (AGX home paths written as `~/`).

## Stand-off edge

On 19 Nov FenceGuard held the nfz3b flight (`fence_hold`, cause fence)
2.62-3.00 m from nfz-north-street's margin ring for 403 ticks; the drone sat
still on 320 of them, and on the last 55 `vla_search` flew it about 13 m
east along the zone with the hold still set.

Today no tick was inside a zone's 3 m stand-off. `fence_d` was below 3 m on
only 4 ticks (2.90-2.97 m, t = 116.3-116.6 s, cause `obstacle`, mode
`skirt`): that is building clearance, not zone distance (with no zone ahead,
FenceGuard.gate() logs the building clearance as `fence_d`), on the x = 200
leg, at least 50 m from every zone. `fence_standoff_cut_ticks` 0 on both
flights follows from that: `standoff_admit` acts only inside a stand-off.
The 19 Nov trap did not recur, but this flight never reached a zone's
stand-off, so the new edge is not exercised here.

Within 30 m rose from 0.345 to 0.48. On 19 Nov the car was lost at the
north-street corner after the stand-off hold. Today the drone passed that
corner; one loss set today's 0.48: the car stood at its leg-3 stop (lapse
t = 132.25 s at (201.5, 58.1)) and was reacquired 233.1 s later, at 365.35
s. That is the same stop-loss as the 4 Nov nfz3b flight (0.494, back after
233.1 s), not the stand-off hold. One flight each does not attribute the
change to the fix.

## Landing descent through the Shield

Both flights landed at site (194.0, -28.0) through the Shield:

| Run | Descent ticks | pass | floor (altitude floor set aside) | refused | Outcome |
|---|---|---|---|---|---|
| nfz3b_agx_hier_aerovla_20261120 | 82 | 0 | 82 | 0 | cleared, end 1.19 m |
| agx_onepath_shield_06 | 86 | 0 | 86 | 0 | cleared, end 1.18 m |

Every descent tick broke only the altitude floor (`alt-band`, the one rule
the Land sets aside), so the Shield held nothing; no re-plan. The column
test cleared the last metre on both.

- **A person near the column on both flights.** On nfz3b a level
  pedestrian was 8.57 m from the landing column at descent start and 2.38 m
  at the last metre; on agx_onepath_shield_06 one was 3.05 m away at descent
  start, none within 10 m at the last metre (`persons_near_column`,
  simulator ground truth, record only). Pedestrian-aware site choice is
  still open, and neither policy has a people distance_envelope.
- **After-mission contact:** one each, `SM_jcBldBase60b_97`, the same
  building-base mesh as onepath_01 to _05 on 19 Nov; no pedestrian among the
  after-mission objects. This reflects the site, not the change: both
  flights landed at (194, -28), where the 19 Nov flights also had only this
  contact; both 19 Nov pedestrian contacts were at (192, 30). Neither policy
  has a distance_envelope over people, so no people were fed to the Shield
  during the descent; it could not react to pedestrians, and this is not
  evidence for it.

## Videos and full runs

PIP videos (`tools/make_demo_video.py --layout pip`, drone view large,
chase view small) and the full runs with view frames:
`F:/vla_drone_offload/jetson_runs/<tag>/`.
