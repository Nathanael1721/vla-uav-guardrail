# Guardrail Progress Note — 30 September 2026

**Period:** since the 16 September lab seminar.
**Project:** ITRI subcontract "Semantic-Spatial Translation and Safety-Constrained VLA for ArduPilot UAVs".
**Scope of this note:** the functional (Project AirSim) rail. None of the results below are KPI-grade. The contractual KPIs are unchanged since the canonical-hil runs of 1 September.

## Summary

The red-car follow mission in the CityLife level no longer locks onto the wrong object. On four flights with the new target-identity layer, the estimate the controller flies on stayed within 6 m of the car on 99.6-100 % of served ticks. On the seven earlier flights, the old estimator replayed on the same boxes managed 1.3-69.7 %. Three of the four flights stayed within 30 m of the car for 97-99 % of the mission. The city now has working traffic lights, cars that stop behind the zebra, and 40 pedestrians who walk routes, wait at the kerb and cross on the walk phase. It stayed stable over a 33-minute Simulate.

## What changed

| Area | Change | Evidence |
|---|---|---|
| Why the drone got "confused" | In the reference flight the HUD read TARGET LOCKED on a red pedestrian signal 110 m from the car. The ground check passed 2-3 m signals, the lock could never answer "none", and a widened estimator gate let a far box re-seed the estimate. | `docs/FINDING-the-lock-that-could-not-let-go.md` |
| Physical identity | Each candidate is judged at its own frame's pose and depth (width, aspect, bottom-edge height, distance from the street) as REJECT / DOUBTFUL / OK (labelled HARD / SOFT / OK on the slides shown on 30 Sept; renamed because the policy DSL uses hard/soft for rule types, and these tiers grade detector boxes, not manoeuvres). Held out, 86.5 % of wrong boxes are not OK, while 15.2 % of true boxes are not OK against a 15 % gate. That is a near miss, not a pass. | `tools/replay_identity.py`, 12 flights |
| Strict lock and re-acquisition | The lock can say "none". The estimate lapses after 3 s. The car is re-acquired only on 4 OK sightings in a row, within 45 m and within reach, and must be seen moving when nothing anchors it. Replayed open-loop on 12 recorded flights with the same boxes, the estimate is on the car on 94.5 % of served ticks, against 22.9 % for the old estimator. There were 0 wrong seeds out of 12, against 210 wrong first accepts out of 241. The old estimator reproduces the flown behaviour exactly only on the four flights of its own revision; on those, the figures are 10.7 % against 96.4 %. | `tools/replay_pipeline.py` |
| Flights, 30 Sept | id1/id2/id3/id4: within 30 m 99.2 / 98.0 / 49.5 / 97.0 %; estimate on the car 100 / 100 / 99.6 / 100 %; P0 escape 0 on all four; no mission collisions. | `demo/out/citylife_redcar_id1..4/metrics.json` |
| id3 | The drone lost the car twice. The first time it re-acquired it in 1.35 s. The second time the car stood at a red light 52 m away, in view, and was refused as "too far" 65 times. A new *far lead* now flies toward such a car; replayed on id3's own candidates it forms at t = 126 s, 4.3 m from the car. It has not flown yet. | `CHANGELOG.md` 30 Sept |
| Control loop | 9.99-10.0 Hz on every flight since 29 Sept (pacing fix). The detector runs at 3.05-3.63 Hz over the mission, below the project's 4.0 Hz gate. | `metrics.json` |
| City | 258 of 292 signal heads switch on a 50 s fixed-time plan. Cars obey the lights and stop 1730 cm from the junction centre, behind the zebra. Over 33 min: 0 red-light violations, 0 stops on a zebra, 294 pedestrian crossings, 0 of 40 figures off route. | Simulate counters, `docs/FINDING-crowd-pedestrians-and-traffic.md` |
| Landing | The drone descends only on a pavement cell chosen with a 1 m clearance margin. | `demo/landing.py` |

## Numbers corrected in this period

- **Detector rate.** Every `det_hz` published before 29 Sept counted start-gate inferences over mission time. Red car 6.9-7.5 Hz is really 3.5-4.0 Hz. The detector-rate range in the 14 September mid-evaluation report used the inflated values and needs a correction note.
- **Identity gate.** An earlier draft said it passed. Held out, it misses the true-box gate by 0.2 points.
- **Replay baseline.** An early draft compared against an arm that reset and re-seeded, which the flown code never did. The figures above use the estimator as flown.
- **Scene checks.** Three Simulate checks could not fail. Once repaired, they exposed two real pedestrian bugs, both fixed.

## Not covered yet

- The far lead exists only in replay, and re-acquisition has flown once.
- For people, range from depth measures the facade behind a thin box. The identity layer is therefore used for the car mission only.
- With identity and no start gate, Demo_day stayed within 30 m for 93.6 % of the mission: acquisition took 7.1 s.
- A figure can wait up to 194 s at one busy zebra. None of the 13 loop junctions has a vehicle signal head.
- All of the above is Project AirSim evidence. The phase-3 work is not committed yet.

## Against the contract

The contractual core (Policy DSL, prefix compiler, Safety Shield, the stress-test harness, and five acceptance KPIs measured on ArduPilot SITL + MAVROS 2 with P0 escape 0.0) is intact. The only Shield change this period fixed a real off-map defect. The core has not advanced since 16 September, however. This period went to the demo items requested on 16 September, plus extra scene engineering. The final-delivery item *perception-rail integration* has not started. Several grant items are also still open. The Gazebo Harmonic functional rail from the mid-term gate has no artefacts in the repository. The KPIs the grant names for WP2 (prefix-token budget, CSP coverage) and WP4 (per-paraphrase robustness) are not measured yet. The canonical KPI runs have not been re-flown since the Shield's off-map fix.

**Decisions requested:**
1. Topology for the HIL perception bridge: Option A (AirSim + MAVROS, new label) or Option B (HIL_GPS/HIL_SENSOR, fully canonical).
2. Whether our desktop SITL + MAVROS 2 "canonical-hil" satisfies the grant's "hil" configuration.
3. Priorities for the final-demo scenarios.

## Attachments

- `citylife_redcar_identity.mp4`: flight id1, 262 s, first-person view with HUD beside the chase camera.
- `progress_0930_before_after.mp4`: 30 s. Top: the old flight locked on a pedestrian signal. Bottom: id1 on the red car.
- `Guardrail-Progress-30Sep2026.pdf`: slides.
