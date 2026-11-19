# People envelope on the perception rail, hil, 19 Nov 2026 (WP1-07)

The first flight of the grant's distance_envelope over people on the PAS
perception rail: a car follow flown by AIoT-VLA v10b
(`deploy/orin/run_pas_perception.sh`, family aiotvla, stream feed) under
the signed bundle follow-car-citylife-people-v0.1.0
(experiments/policies/follow_car_citylife_people.yaml: "keep at least 4 m
horizontal from any person", P1 for its first flights), with the level's
40 pedestrians read for the record only (`--level-peds 40
--level-ped-prefix Ped_`, never a Shield input).

- **Topology:** hil, as stamped: Project AirSim CityLife
  (`run_pas_ardupilot.ps1 -Step scene -VlaCameras -SimMaxFps 30
  -FrontCaptureIntervalS 0.05`) and ArduPilot SITL 4.5.7 on the desktop;
  the perception node, Shield, MAVROS and the VLA server on the AGX Orin.
- **Code:** bfb4973 (clean). Policy hash sha256:35e3f4ae..., signature
  verified.

## Files

| File | What it is |
|---|---|
| `flight_pas_hil_people_02.log` | The flight's console log |
| `pas_hil_people_02_metrics_excerpt.json` | metrics.json: people_feed, fsm, policy source, VLA observation, camera-to-setpoint, manifest |
| `flight_pas_hil_people_01_refused.log` | The first attempt, refused before take-off: the scene had been loaded without `-VlaCameras`, and a `--pilot vla` node requires VLADown. Nothing flew |

Run folder (no frames): `demo/out/pas_hil_people_02/` (lab checkout,
gitignored).

## Result

- **Mission:** start gate reached after 154.5 s; the mission then ran
  18.2 s (180 ticks at 9.83 Hz) and ended in RTL. FSM: G1 (standoff-any
  repaired 0.61 m) -> G3 Loiter (repair 2.07 m over theta 2.0 m) -> G8
  back to Normal -> G1 -> **X7 RTL** at 9.3 s: "position illegal for 2.00 s
  and its illegal depth grew 4.85 m in 2.00 s (+2.43 m/s) while the Shield
  recovers". standoff-any is the 5 m P0 ring around the tracked subject
  (the car); the escalation is the grant's fail-safe for a recovery that
  is losing ground. Home reached.
- **P0:** P0 escape ticks 0, flown-violation ticks 0, 69 interventions.
- **People, against truth (record only):** the aircraft was within 4 m
  horizontal of a level pedestrian on 75 of 178 ticks with truth (7.5 s),
  true minimum 0.765 m horizontal (the aircraft flies in the 6-14 m band).
- **People feed recall:** 0.189 of pedestrians within 25 m matched by a
  feed track (14 of 74), 0.32 of those in view (8 of 25). The person
  threshold (0.10) is untuned, as the policy says. 11 people boxes placed
  over 37 frames, 4 tracks.
- **Latency:** VLA decisions p50 0.119 s / p95 0.225 s (capture to
  answer); frame arrival to first setpoint p50 189 / p99 327 ms.
- **KPI-grade: no** - the follow is not scored: the subject's pose was read
  on 166 of 180 ticks (needs 95 %).

## What this shows

- The people envelope flies on the hil rail from a signed bundle, and the
  record measures it against truth.
- With the detector's person recall at about 0.2-0.3, the people rule
  cannot keep the 4 m envelope: the truth record shows the aircraft over
  or beside pedestrians on 42 % of ticks with truth. Raising the rule to
  P0, as the policy's own note plans, needs a person detector whose recall
  is measured first.
- One flight, 18 s of mission: a first measurement, not a rate.
