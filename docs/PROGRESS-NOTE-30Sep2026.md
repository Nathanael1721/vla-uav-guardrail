# Guardrail Progress Note — 30 September 2026

**Period:** since the 16 September lab seminar.
**Project:** ITRI subcontract "Semantic-Spatial Translation and Safety-Constrained VLA for ArduPilot UAVs".
**Scope of this note:** the functional (Project AirSim) rail. None of the results below are KPI-grade. Our KPI runs so far are on the grant's dev topology (one desktop: ArduPilot SITL + MAVROS 2), last scored on 1 September. Since 6 October our own tooling no longer counts those runs as KPI-grade without a written waiver from the PI. There are no hil runs yet: the grant takes reported KPIs only from Stress Testing runs in the hil topology, with the VLA and the Shield on a Jetson Orin.

**Corrected 6 October 2026** after the 5 October contract audit. The 3 October version called the desktop rail "canonical-hil", said the core was "intact" with "five acceptance KPIs measured", and said Gazebo had "no artefacts". Those lines are reworded here. The same corrections to the 14 September report and the 16 September deck are listed in the correction note of 6 October (`CORRECTION-2026-10-06-mid-evaluation-and-deck`).

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

The contract work has not advanced since 16 September. This period went to the demo items requested on 16 September, plus extra scene engineering. The only Shield change fixed a real off-map defect. Where the contract work stood at the 5 October audit:

- **Policy DSL (WP1):** built, with gaps. The bundle signature is a placeholder until a signing CA is chosen.
- **Prefix compiler (WP2):** a constraint summary pack (CSP) is generated and saved. No flown VLA has read it, and the grant's CSP pipeline (filter, risk-grade, truncate, token budget) was not built.
- **Safety Shield (WP3):** the core (check, repair, brake) is built. The escalation to Loiter / RTL / Land and the ArduPilot GeoFence backstop are not.
- **Stress testing (WP4):** what exists is a headless regression sweep of 13 scenarios plus replay bundles, not the grant's stress harness.
- **KPIs:** the five acceptance KPIs were computed on desktop SITL (dev topology), not in hil Stress Testing runs, and those runs have not been re-flown since the off-map fix. Not measured at the audit: the other work-package KPIs (policy load round-trip, bundle replayability, prefix-token budget, CSP coverage, per-paraphrase robustness), the component targets, and the Shield repair success rate.
- **Gazebo:** the Gazebo Harmonic functional rail was a mid-term gate item. Its launch scripts exist (`sitl/run_gazebo_demo.sh`), but no run output is kept in the repository, so the item cannot be shown. July notes record a headless run over pymavlink; nothing has run through MAVROS 2.
- **Perception-rail integration,** a final-delivery item, has not started.

**Decisions requested:**
1. Route for the perception bridge. Option A: AirSim renders and ArduPilot flies through MAVROS, under a new label. Option B: HIL_GPS/HIL_SENSOR injection, so ArduPilot's EKF runs on AirSim sensors. Neither is the grant's hil topology unless the VLA and the Shield run on a Jetson Orin.
2. Whether the final KPIs may come from our desktop SITL + MAVROS 2 rail, which the grant calls "dev" (the code labelled it "canonical-hil" until 6 October), or whether a Jetson Orin will be available for the hil topology.
3. Priorities for the final-demo scenarios.

## Attachments

- `citylife_redcar_30Sep_identity.mp4`: flight id1, 262 s, first-person view with HUD beside the chase camera.
- `progress_0930_before_after.mp4`: 30 s. Top: the old flight locked on a pedestrian signal. Bottom: id1 on the red car.
- `Guardrail-Progress-30Sep2026.pdf`: slides.
