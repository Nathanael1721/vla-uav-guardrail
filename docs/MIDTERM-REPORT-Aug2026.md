# Midterm Report

## Semantic-Spatial Translation and Safety-Constrained VLA for ArduPilot UAVs

**Institution** National Taipei University of Technology (NTUT), AIoT Laboratory
**Principal investigator** Prof. Kuan-Ting Lai
**Author** Nathanael Tjahyadi
**Funding body** ITRI
**Project period** February – November 2026
**Reporting period** February – August 2026
**Date** 20 August 2026

---

## 1. Deliverables accompanying this report

| Artefact | File |
|---|---|
| This report | `docs/MIDTERM-REPORT-Aug2026.pdf` (and `.docx`) |
| Slide deck | `docs/VLA-Guardrail-Midterm-Aug2026.pptx` (and `.pdf`) |
| Demonstration video, tracking | `docs/video/demo_follow.mp4` |
| Demonstration video, distractors | `docs/video/demo_traffic.mp4` |
| Demonstration video, no-fly zone | `docs/video/demo_nfz.mp4` |

All numerical claims in this report are read from `demo/out/<tag>/metrics.json`
and `demo/out/<tag>/flight_log.jsonl`. Section 11 indexes the source files.

---

## 2. Objectives and scope

The project addresses a specific failure mode. Vision-Language-Action (VLA)
models can pilot a UAV from camera images and natural-language instructions, but
they are probabilistic and occasionally emit commands that violate airspace
rules. Flight safety cannot rest on a component that is sometimes wrong.

The response is not a better model. It is a deterministic layer, the **Guardrail**,
placed between any action source and the autopilot, holding one contractual
acceptance criterion:

> **P0 violation escape rate = 0.** No action that violates a P0-priority rule
> reaches the vehicle.

Scope for this reporting period:

1. Establish the Guardrail as a component independent of the model above it.
2. Demonstrate language-commanded target following in a photorealistic city.
3. Demonstrate the Guardrail intervening under a rule conflict, without losing
   the mission.
4. Build the verification apparatus needed to make KPI claims traceable.

Explicitly out of scope this period: hardware flight, LiDAR or depth-based
obstacle avoidance beyond the existing occupancy map, and multi-agent operation.

---

## 3. System architecture

### 3.1 The action contract

Every component above the Guardrail communicates through a single type:

```
Action4D(vx, vy, vz_up, yaw_rate)      at 10 Hz
    vx      m/s, positive North
    vy      m/s, positive East
    vz_up   m/s, positive up
    yaw_rate rad/s, positive clockwise from above
```

Defined at `guardrail/models.py:31`. The Guardrail accepts nothing else. Any
action source that emits `Action4D` is admissible, which makes the model above a
replaceable component rather than a dependency.

### 3.2 Pipeline

```
operator text ─┐
front camera ──┴─> OWL-ViT ─> colour gate ─> state estimator ─> guidance
                                                                    │
                            policy + occupancy map ─> SAFETY SHIELD <┘
                                                          │
                                                          v
                                        autopilot (simple_flight / ArduPilot)
                                                          │
                                                          v
                                                      aircraft
```

The Shield is the last component before the autopilot and has final authority.

### 3.3 Why the Guardrail is the deliverable

The action stage in the tracking demonstration is a hand-written proportional
controller, not a learned policy. This is a deliberate consequence of the
architecture, and it is stated plainly because the distinction matters to how
the results should be read.

Over 108 controlled forward passes (`docs/FINDING-what-drives-aerialvla.md`),
the `{object}` prompt slot of AerialVLA was measured to be **inert**: a correct
colour word and an incorrect one produce indistinguishable actions. Only the
`{direction}` compass phrase drives its output. Language grounding therefore had
to come from elsewhere, and OWL-ViT supplies it.

The Guardrail has been exercised with five different occupants of the action
slot (Section 6.4). The Shield source code is identical in all five cases. That
invariance, not any single model's performance, is the project's claim.

---

## 4. Methods

### 4.1 Perception

**Detector.** `google/owlvit-base-patch32`, 153 M parameters, 0.61 GB VRAM,
Apache-2.0. Open-vocabulary: the target is specified at runtime as a text string
(`--object "a yellow car"`) with no fixed class list and no retraining.

**Colour verification.** OWL-ViT localises the noun; a fixed rule verifies the
adjective. Each candidate box is cropped and scored on the fraction of pixels
matching the requested hue. The final ranking is

```
score x (0.25 + 0.75 x colour_match)
```

The gate originally applied an HSV **saturation** floor (`s > 90`). Saturation is
chroma divided by brightness, so a vehicle entering direct sunlight loses
saturation without changing colour. Measured on the target's own pixels: median
saturation fell from 99 in shade to 69 in sunlight, and the fraction of pixels
passing the gate fell from 56.7 % to 8.1 %. In one flight, 25 of 25 detections
were rejected by the colour gate and none by the detector score, while OWL-ViT
scored the vehicle 0.17–0.30, its highest of that flight.

The gate now applies an **absolute chroma** floor, `s x v / 255 > 40`, which is
invariant to illumination. Detector hit rate rose from 0.73 to 1.000.

### 4.2 Target-state estimation

Detections arrive at roughly 4 Hz while the control loop runs at 10 Hz, and the
box width used as a range proxy varies **37.7 %** between consecutive frames. A
proportional controller acting directly on that measurement produces rough
commands.

A constant-velocity Kalman filter (`demo/target_state.py`) converts the polar
measurement to Cartesian, gates innovations, and predicts between detections.
Velocity feed-forward replaces an integral term. Effect on the forward channel:

| Metric | From box width | From the estimate |
|---|---|---|
| Command step per tick, 95th percentile | 0.716 m/s | 0.283 m/s |
| Ticks where the rate limiter engaged | 15.6 % | 3.0 % |

A derivative term was not added: it would amplify the same measurement noise. An
integral term was not added: it would wind up whenever the Shield overrides the
commanded action. PID control is present in the system, below this layer, inside
the autopilot.

### 4.3 Guidance

Proportional control on three channels: horizontal box offset to yaw rate,
estimated range error to forward speed, altitude error to climb rate. Output
passes through a rate limiter before reaching the Shield.

### 4.4 Safety Shield

Three stages, at `guardrail/shield.py`:

1. **Monitor.** Forward-simulate the proposed action for 3 s and test it against
   every active constraint, with trend awareness so a violation is caught before
   it occurs rather than after.
2. **Repair.** Apply the minimal correction that clears the violation, by
   fixed-point iteration. The emitted action is re-checked; an action that still
   violates never leaves the Shield.
3. **Escalate.** If repair cannot clear the violation, brake.

The Shield returns a `ShieldDecision` recording the raw action, the emitted
action, every violation with its rule identifier and predicted time, every
repair operator applied, and whether it braked. This record is written per tick
to `audit.jsonl`.

**Separation of authority.** No repair operator modifies `yaw_rate`. Heading
remains the controller's; the ground track is what the Shield bends. This makes
simultaneity observable: a tick in which `emitted.yaw_rate == raw.yaw_rate` while
`emitted.(vx, vy) != raw.(vx, vy)` is one in which both systems acted.

**Constraint taxonomy.** Four types are currently expressible:
`PolygonFence`, `AltitudeEnvelope`, `KinematicEnvelope`, `ObstacleClearance`.
Each carries a priority (P0/P1/P2) and a violation action (repair or brake).
Policies are YAML, validated into a Pydantic model, and hashed to `policy_hash`.

**Defect corrected this period.** `yaw_rate_max_dps` was specified in degrees per
second and compared against a value carried in radians per second at three sites,
making the effective cap 2578 °/s. The rule could never fire. Conversion now
occurs at the boundary. The worst commanded yaw rate measured across the demo
flights is 11.8 °/s against a 45 °/s cap, so no recorded flight changes behaviour
as a result of the fix.

---

## 5. Experimental setup

| Parameter | Value |
|---|---|
| Simulator | Project AirSim on Unreal Engine 5.7 |
| Scene | JapaneseCity, `Demo_day` map |
| GPU | NVIDIA RTX 4080, 16 GB |
| Detector input (`FrontCamera`) | 400 x 225, 90° HFOV, pitched 20° down |
| Recording camera (`Chase`) | 960 x 540 at 20 Hz |
| Control loop | 10 Hz nominal |
| Cruise altitude | 9 m |
| Target vehicle | glTF taxi, 2.5 m/s, two 8 s stops |
| Route | 93 m, straight leg then a left turn onto the cross street |
| Flight duration | 70 s per scenario |
| Policies | `policies/follow_car.yaml`, `policies/follow_car_nfz.yaml` |

`FrontCamera` resolution is held fixed at 400 x 225 across all reported work.
It is the detector's input, and every measurement in the repository is
conditioned on it.

Three scenarios were flown:

- **`demo_follow`** — tracking with no fence. Isolates the perception and
  control question.
- **`demo_traffic`** — three additional vehicles of different colours on the
  same street. Tests target discrimination.
- **`demo_nfz`** — a polygon fence spanning the corridor at x ∈ [26, 54],
  y ∈ [2, 16], which the target vehicle drives through and the aircraft may not
  enter. Tests the Shield under an unavoidable conflict.

The fence in `demo_nfz` spans the whole corridor by construction. An earlier
version fenced only one leg; the aircraft tracked the target on the other leg,
never approached the boundary, and demonstrated nothing.

---

## 6. Results

All figures are read from `demo/out/<tag>/metrics.json` and `demo/out/<tag>/flight_log.jsonl`. This section is generated from those files by `tools/build_report_results.py` rather than transcribed, so it cannot disagree with the artefacts or with the slide deck.

### 6.1 Target following

| Measure | Tracking | Distractors | No-fly zone |
|---|---|---|---|
| Detector hit rate | 1.000 | 0.920 | 0.721 |
| Ticks with the target held | 100.0 % | 100.0 % | 94.1 % |
| Detector rate | 3.62 Hz | 3.97 Hz | 4.20 Hz |
| Control loop rate | 8.85 Hz | 8.14 Hz | 7.69 Hz |
| Mean separation | 16.3 m | 16.4 m | 40.7 m |
| Minimum separation | 9.8 m | 4.3 m | 12.4 m |
| Time within 30 m | 100.0 % | 93.7 % | 35.5 % |
| Flight duration | 70.0 s | 69.9 s | 70.0 s |

The two tracking scenarios held the target on every control tick. The no-fly-zone scenario holds a larger separation by design: the fence spans the corridor, the target drives through it, and the aircraft is required not to follow. It was held at the boundary for 377 ticks.

With three additional vehicles of different colours on the same street, target jumping fell from 14.0 % of detections to 0.4 %.

### 6.2 Guardrail invariants

| Invariant | Tracking | Distractors | No-fly zone | Requirement |
|---|---|---|---|---|
| P0 violation escape rate | 0.000 | 0.000 | 0.000 | 0 |
| Time inside the no-fly zone | 0.0 s | 0.0 s | 0.0 s | 0.0 s |
| Altitude envelope escape | 0.0 s | 0.0 s | 0.0 s | 0.0 s |
| Shield interventions | 0 | 0 | 50 | not bounded |

The acceptance criterion is met on every flight. An escape is counted only when the Shield neither repaired nor braked and the emitted action still violated a P0 rule; scoring the raw action would credit the system for its own inputs.

The intervention counts distinguish the two situations. In No-fly zone (50) the guidance layer proposed actions that would have violated an active rule, and the Shield corrected them; time inside the zone remained 0.0 s, which is the property being claimed. Repair is the normal outcome, not an error condition.

In Tracking, Distractors the count is zero. That means the guidance layer never proposed a violating action, not that the Shield was inactive: it evaluated every tick against every active constraint.

### 6.3 Recording resolution against detector throughput

The recording camera was raised to 1280 x 720 to improve video quality. Acceptance thresholds were fixed before the runs: detector rate at least 4.0 Hz and control loop at least 9.5 Hz.

| Chase capture | Simulator window | Detector rate | Control loop | Median inference |
|---|---|---|---|---|
| 1280 x 720 | 1280 x 720 | 2.94 Hz | 8.69 Hz | not recorded |
| 960 x 540 | 1280 x 720 | 3.66 Hz | 8.79 Hz | 286 ms |
| 960 x 540 | 960 x 540 | 3.62 Hz | 8.85 Hz | 287 ms |

OWL-ViT inference held at 286-287 ms median across a 2.4x change in Chase pixels and a 1.8x change in window pixels. A cost that is invariant to surrounding GPU load is a fixed per-inference cost, not contention. The detector rate is therefore not reachable by resolution tuning; it needs a faster detector or a different inference budget. An earlier hypothesis that the simulator window was the binding consumer was tested and rejected by the third configuration.

**Neither configuration met the threshold, and the threshold is not relaxed to fit the data.** det_hit_rate 1.000 and frac_ticks_seen 1.000 on both tracking scenarios in every configuration. The threshold exists to protect tracking quality, and tracking quality was never degraded. Detector throughput is carried into the next period as open work.

### 6.4 Independence from the action source

| Action source | Nature | Recorded outcome |
|---|---|---|
| OpenVLA-7B, 4-bit | Real 7 B camera and language VLA | 553 ticks at 10 Hz, 10 Shield interventions, NFZ 0.0 s |
| AerialVLA LoRA | UAV-tuned adapter on the same base | Target reached, NFZ 0, clean path around the zone |
| QLoRA fine-tunes (ours) | Trained on self-collected expert flights | 100 % reached, mean efficiency 0.996, goal assist off |
| Behaviour-cloning policy | Trained state and geometry policy | Flown, NFZ 0 |
| Proportional controller | Hand-written, no model | Reported in 6.1 and 6.2 |

The Shield source code is identical in all five cases; only the adapter above it differs. That invariance, rather than any single model's performance, is the result this project claims.

An eight-flight study of 150 s each established that the Shield and the action source act within the same control tick rather than alternating: on the passing flights the model commanded a direction closing on the target at cos 0.53 to 0.95 while the Shield bent the resulting ground track by 82 to 103 degrees, with the heading channel untouched throughout. The guardrail-disabled control flights scored zero such ticks and spent 32.4 s and 84.2 s outside a P0 rule respectively.

### 6.5 Demonstration recordings

| Scenario | Frames | Capture rate | Duration |
|---|---|---|---|
| Tracking | 1729 | 15.13 Hz (target 20.0) | 114.3 s |
| Distractors | 1726 | 15.12 Hz (target 20.0) | 114.2 s |
| No-fly zone | 1658 | 14.96 Hz (target 20.0) | 110.8 s |

The capture rate is measured by the recorder and written to `view/recorder.json`. It is not derived from the flight log: the recorder starts before the mission clock and stops after it, so frames divided by mission duration overstates the rate and would produce a video that plays faster than real time while being labelled real time.

---

## 7. Verification apparatus

### 7.1 Acceptance KPIs

Four KPIs are computed from each flight's artefacts by `guardrail/kpi.py`. The
binding one is the P0 violation escape rate. An escape is counted only when the
Shield neither repaired nor braked and the **emitted** action still violated a
P0 rule — scoring against the raw action would credit the system for its own
inputs.

### 7.2 Determinism manifest

Each flight emits a six-field manifest (`guardrail/manifest.py`):
`code_revision`, `vla_model_hash`, `policy_hash`, `random_seed`, `sim_speedup`,
`topology`. `sim_speedup` is derived from the scene file rather than asserted by
the caller.

`is_kpi_grade()` decides whether a run's numbers may be quoted as contractual
figures. It fails a run when the simulation is not real-time, when required
fields did not resolve, when the detector ran below 2 Hz, when the initial
heading was more than 10° off, or when the topology is not the grant's canonical
ArduPilot SITL configuration. Each check exists because the corresponding failure
has already occurred and produced a plausible-looking number.

### 7.3 Test suite

**191 tests across nine modules, all passing**, run on 20 August 2026.

| Module | Tests | Covers |
|---|---|---|
| `test_range_and_lock.py` | 42 | Range estimation, target lock, instance selection |
| `test_city_traffic.py` | 34 | Route generation, traffic circuits, heading rates |
| `test_vla_bridge.py` | 28 | Compass-phrase mapping, absence of coordinate leakage |
| `test_manifest.py` | 19 | Determinism manifest and KPI-grade gating |
| `test_guardrail_coverage.py` | 16 | Shield behaviour over randomly sampled states |
| `test_shield.py` | 16 | Monitor, repair and escalation logic |
| `test_fenceguard.py` | 14 | Polygon fence geometry and margins |
| `test_clearance.py` | 13 | Obstacle clearance against the occupancy map |
| `test_target_state.py` | 9 | Kalman filter, innovation gating, feed-forward |

`test_vla_bridge.py` includes a structural test rather than an assertion of
intent: it tokenises the bridge module, discards comments and docstrings, and
fails if any target-coordinate identifier survives in the code, then walks the
AST and fails if any function accepts an argument with such a name. The
no-coordinate-leak property is therefore checked, not claimed.

---

## 8. Limitations

Each limitation below is measured rather than anticipated.

1. **No flight recorded to date is KPI-grade.** `is_kpi_grade()` requires the
   grant's canonical topology (ArduPilot SITL with MAVROS 2). All results in
   Section 6 were produced on the Project AirSim rail and are functional-rail
   evidence, not contractual acceptance figures. Section 10 addresses this.

2. **The occupancy map contains buildings only.** It holds no trees, street
   furniture, or parked vehicles. A 9 m flight has already contacted street
   furniture at (48.3, −0.9). The Shield cannot constrain against geometry
   absent from its map; this is a data gap, not a Shield defect.

3. **Depth is quantised to one metre.** The depth stream arrives as `16UC1` at
   whole-metre granularity despite a float pixel request. It supports proximity
   detection but not a metric standoff requirement such as "hold 10 m".

4. **Per-object standoff is not expressible in policy.** The requirement "hold
   10 m from a pedestrian, with different policies per object class" is presently
   a command-line parameter (`--want-range`). It is therefore not hashed into
   `policy_hash`, not audited, and not enforced by the Shield. It is a controller
   setpoint rather than a rule.

5. **Range from apparent width assumes a car.** `implied_range_from_width()`
   uses `object_width_m = 4.0`. Applied unchanged to a pedestrian of roughly
   0.5 m width, it would report the subject at approximately eight times the true
   distance.

6. **The learned VLA path cannot track a moving vehicle in real time.** Measured
   inference cost is 225 ms per token over 12 tokens, giving 2.7 s per decision
   out-of-process and 0.37 Hz. A target at 2 m/s travels 5.4 m within one
   inference. Section 6.4 quantifies this.

---

## 9. Work package status

| WP | Component | Status |
|---|---|---|
| WP1 | Policy DSL | In use. Policies validated and hashed. Four constraint types; per-object standoff not yet expressible (Limitation 4). |
| WP2 | Prefix compiler | Reduced version in use on the VLA path. |
| WP3 | Safety Shield | Implemented, tested, and exercised under conflict. P0 escape rate 0 on every flight recorded. |
| WP4 | Stress harness and determinism | Manifest and KPI computation implemented and unit-tested. Scenario sweep harness not built. SITL rail exists but is not wired to the manifest. |

---

## 10. Plan for the next period

Ordered by contribution to the acceptance criteria rather than by effort.

**10.1 Wire the existing SITL rail to the WP4 machinery.** The rail is not
missing. `sitl/` builds ArduPilot under WSL, serves MAVLink on
`tcp:127.0.0.1:5760`, and flies the Guardrail mission using
`SET_POSITION_TARGET_LOCAL_NED` in GUIDED mode. Three configurations have been
run and their outputs are on disk. The Guardrail package is byte-identical
between the AirSim and ArduPilot rails; only the bottom adapter differs, which is
the architecture rule demonstrated. Two gaps remain: `sitl/run_sitl_demo.py`
scores with an ad-hoc pass flag instead of `guardrail/kpi.py` and never calls
`build_manifest()`; and `build_manifest()` refuses the `canonical-hil` label
unconditionally. Closing these converts existing evidence into contractual
figures without further flying.

**10.2 Add MAVROS 2 to the SITL rail.** This completes the grant's canonical
topology and is the precondition for any KPI-grade number.

**10.3 Add a per-object standoff constraint to the policy schema.** Converts a
controller setpoint into a hashed, audited, Shield-enforced rule. This is WP1
work and is the prerequisite for the pedestrian scenario being meaningful.

**10.4 Extend the scene with pedestrians and additional vehicles.** Requested at
the 19 August review. Requires 10.3 and a per-class `object_width_m` first
(Limitation 5).

**10.5 Populate the occupancy map with trees, street furniture, and parked
vehicles.** Addresses a demonstrated collision, requires no additional sensor,
and is a precondition for evaluating whether depth or LiDAR is needed.

**10.6 Detector throughput.** If detector rate becomes binding again, the
candidate is YOLO-World: open-vocabulary at 30–50 Hz, with text prompts
compiled into weights so there is no runtime language cost. Retraining a
closed-vocabulary detector to recognise colours is not recommended; it would
remove the open-vocabulary interface without addressing throughput.

---

## 11. Artefact index

| Content | Path |
|---|---|
| Per-flight metrics | `demo/out/<tag>/metrics.json` |
| Per-tick flight log | `demo/out/<tag>/flight_log.jsonl` |
| Shield audit trail | `demo/out/<tag>/audit.jsonl` |
| Safety Shield | `guardrail/shield.py` |
| Action and policy types | `guardrail/models.py` |
| KPI computation | `guardrail/kpi.py` |
| Determinism manifest | `guardrail/manifest.py` |
| Tracking mission | `demo/follow_vlm.py` |
| Target-state estimator | `demo/target_state.py` |
| ArduPilot SITL rail | `sitl/` |
| Colour gate analysis | `docs/FINDING-colour-gate-and-sunlight-aug15.md` |
| Estimator and control analysis | `docs/FINDING-target-estimator-and-control-aug17.md` |
| AerialVLA prompt-sensitivity study | `docs/FINDING-what-drives-aerialvla.md` |
| VLA and Shield simultaneity study | `docs/RESULT-vla-guardrail-simultaneity.md` |
| Fine-tuning report | `docs/aerialvla-ft-report.md` |
