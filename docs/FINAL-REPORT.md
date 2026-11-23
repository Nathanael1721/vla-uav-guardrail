# Final Report

Semantic-Spatial Translation and Safety-Constrained VLA for ArduPilot UAVs ·
NTUT AIoT Lab · draft v0.5, 2026-11-23

> **Status: DRAFT, not signed.** Written from docs/FINAL-REPORT-outline.md at
> 2f22116; updated 14 Nov for the `hil` runs hs10 and ge1113, the CI on
> Python 3.12 and the committed KPI report (branch evidence-1114); updated
> 22 Nov at 96eccf1 (integrate/1122) for the KPI-grade nightly ntFIN3 and
> the final smoke smfin1120 at 71aabb3; updated 23 Nov (docs/1123) for the
> shakedowns sk1122a / sk1122b, the final smoke smFIN2 and the KPI-grade
> nightly ntFIN4, all flown on `hil` at 59e19a5, the revision this report
> now describes (71aabb3 plus the eleven changes of CHANGELOG 2026-11-22
> (code)), and for the 23 Nov recount of the open Appendix A rows. ntFIN3
> stays beside ntFIN4 as the earlier figure. The tick fix that follows
> ntFIN4's one 105.5 ms tick (section 5.4) is not committed and is in none
> of the figures. The grant's final-delivery gate is 2026-11-30 (a signed
> final report, X-01). The signer and the signing CA are not named yet (PI,
> 12 Oct, Q5; on 22 Nov the user decided to wait for the PI to name the
> CA). No value is filled in ahead of its evidence. Every number below names
> the file or run it comes from.

## 1. Summary

- The Guardrail exists in the grant's four-part structure: a Policy DSL with
  a typed IR and signed bundles (WP1), a Prefix Compiler that emits a
  Constraint Summary Pack (WP2), a Safety Shield with an escalation FSM in
  front of ArduPilot (WP3), and a Stress Testing harness with a scenario
  library, episode bundles and a KPI report generator (WP4).
- Contract map (Appendix A, 136 obligations): **109 DONE, 26 PARTIAL,
  1 DRIFT, 0 MISSING**. DONE means the code or artefact the grant names
  exists and matches the grant's words; for a KPI row it means a KPI-grade
  value meets them.
- KPI-grade source (the figure of record): the nightly `hil` campaign
  ntFIN4 at 59e19a5, the revision this report describes, flown 22-23 Nov on
  the AGX Orin rig: 654 missions flown, 653 KPI-grade (650 Shield on, 3
  Shield-off controls; one refused by the setpoint-delivery rule) (section
  5; docs/data/kpi_ntFIN4/). It replaces the nightly ntFIN3 at 71aabb3
  (21 Nov, 594 KPI-grade; docs/data/kpi_ntFIN3/), which section 2 keeps
  beside it.
  - P0 violation escape rate **0.0**: 0 escapes on 17,078 P0-violation
    ticks over 650 Shield-on flights. Shield-off control: 0.6500.
  - Fail-safe trigger correctness **648/648 = 1.000** on the adopted reading
    (r2), Wilson 95 % lower bound **0.994**: it meets 99 % as the point
    estimate, the grant's figure, and at 95 % confidence. 18 of the
    fail-safes were triggered by the autopilot's GeoFence backstop
    (decision 14), all expected; without that rule the same runs give
    630/630 (Wilson low 0.994). Two runs censored in Loiter are left out
    (648/650 = 0.997 if both were errors). The other readings: (r) 636/648,
    (a) 573/648. ntFIN3 at 71aabb3 had 587/589 = 0.997, Wilson low 0.988.
  - Shield tick (`tick_ms`) p99 28.4 ms, action receipt to setpoint
    (`e2e_ms`) p99 28.5 ms, but **1 of 183,909 ticks took 105.5 ms**
    (`e2e_ms` 105.6 ms): the tick on which ntFIN4_236 hot-applied its zone,
    whose generation write and CSP compile ran before the setpoint. The
    budget of 100 ms is met at p99 and not on every tick (section 5.4;
    WP3-13 back to PARTIAL). A fix followed the flight and is not in this
    campaign or committed. ntFIN3 had 0 of 174,186 ticks over 100 ms on the
    same code path.
  - The final smoke at the same revision, smFIN2 (52 KPI-grade missions),
    had P0 escapes 0, fail-safe (r2) 51/51 and no tick over 100 ms; it flew
    all five rows of the grant's Shield test matrix, GPS noise / latency
    included, for the first time (section 4.3).
- Every edge of the grant's escalation ladder has flown on `hil`: G2, G4,
  G6, X2 and X3, the edges no rail had taken, in hs10 at 8a473fa on 14 Nov
  (section 4.3; CHANGELOG 2026-11-14 (hs10 ...)). SET_MODE LOITER flew on
  `hil` for the first time on 22 Nov, opt-in (sk1122b, 2 missions). WP3-07
  stays PARTIAL: the default still holds the FSM's Loiter in GUIDED from
  the companion (section 9, decision 7, PI confirmation pending).
- The KPI campaigns fly a stub pilot (guardrail/vla_stub.py). A learned
  model flew in the `hil` topology on 14 Nov: the lab's AIoT-VLA v10b on the
  AGX Orin, through MAVROS 2, with the Shield on (section 6;
  docs/data/pas_aiot_hil_20261114/flights.md). Seven missions, 0 P0 escape
  ticks in every one, 22-297 Shield interventions each; two of them
  KPI-grade (pas_hil_aiot_kpi_01: 420 s, 119 interventions, VLA 7.75
  decisions/s, camera to first setpoint p50 161.7 ms). Under perception
  stress, darkness breaks detection: at -4 EV no red car was found in 330 s
  and the mission refused to start.
- Follow quality on that rail was weak without a target estimator: target lock 0.04-0.82 against
  0.955-0.969 on simple_flight. With the city rail's estimator ported (15 Nov, pas_hil_aiot_est_01, KPI-grade) the estimate stayed within 6 m of the car on 0.907 of ticks and the aircraft-to-car gap fell to 34.4 m median (from 112.9 m). The VLA stands still when the detector has
  no fresh box of the car (in pas_hil_aiot_kpi_01 its own command was 0 m/s
  at the median), and the city rail's target estimator and re-acquisition
  are not in the PAS node yet; `--identity` without that estimator lost
  the car after 107 s (pas_hil_aiot_kpi_02). Not a locked KPI; stated as a
  limitation (section 10).
- Since 15 Nov the lab's own VLA gets its observations and decides at 10 Hz
  or faster on the hil rig: 12.4-13.2 camera frames/s reach AIoT-VLA while
  it is asked (9.6 before; a 15 Hz camera and a feed between the loop's
  ticks), and it decided 10.6 and 10.9 times a second over two full 420 s
  missions, 0 P0 escapes (X-05, ARCH-25, ARCH-01, ARCH-21; section 6).
- What this report cannot claim: a 7B VLA deciding at 10 Hz (OpenVLA about
  0.98 s per decision on the AGX Orin; ARCH-03), the `flight` topology (ARCH-14),
  multi-seed Gazebo sweeps (WP4-04), a green CI smoke per build or a
  release whose own smoke flew the five-row matrix (WP3-19, WP4-06,
  WP4-14), the monitor tick within 100 ms on every tick at the delivered
  revision (WP3-13), and SET_MODE LOITER as the FSM's default (WP3-07).

## 2. Scope and acceptance criteria

"KPI-grade" means `guardrail.manifest.is_kpi_grade` accepts the run (`hil`
or `flight` topology, sim_speedup 1.0, a clean code_revision), the signed
replay bundle beside the run verifies and is bound to it, and the campaign
accepted the flight (tools/kpi_report.py, "Definitions" section of its
output).

**The KPI-grade source (the figure of record since 23 Nov).** Profile
`nightly` at 59e19a5 (clean), the revision this report describes, flown on
the AGX Orin rig: ArduCopter 4.5.7 SITL and mavlink-router on the desktop;
MAVROS 2, the Shield and the stub pilots on the Orin.

- ntFIN4: launched 22 Nov 15:42 with `--hil-controls` from the detached
  worktree flight-59e19a5, after the smoke smFIN2; 78,879.2 s (21.9 h).
  744 episodes in the profile; 90 refused by the rail before take-off (69
  paraphrase / prefix arms the stub cannot read, 9 stressors the rail
  cannot realise: gusts and a low battery, 12 others: starts at 0.5 m and
  at 30 m, a non-finite pilot action, a relabel schedule); the 60 GPS-noise
  / latency missions flew on the rail for the first time in a nightly
  (refused through ntFIN3). 654 missions flown (651 Shield on, 3 Shield-off
  controls), 653 KPI-grade: one dyn-nfz-translate run is not, its MAVLink
  adapter having handed none of the Shield's setpoints to MAVROS for 25 s
  (the setpoint-delivery rule of 17 Nov) (docs/data/kpi_ntFIN4/README.md;
  demo/out/hil_ntFIN4/sweep.json; runs demo/out/hil_smoke/ntFIN4_*).
  single_sha 653/653: companion and desktop at 59e19a5e3b19, autopilot
  ArduCopter V4.5.7 (2a3dc4b7).
- It replaces, as the figure of record, the nightly ntFIN3 at 71aabb3
  (21 Nov, 594 KPI-grade, 591 Shield on; docs/data/kpi_ntFIN3/), which
  replaced ntFIN2 + ntFIN2s at 116214f (12-13 Nov, regraded 17 Nov;
  docs/data/kpi_ntFIN2_20261117_delivery/). 59e19a5 is 71aabb3 plus the
  eleven changes of CHANGELOG 2026-11-22 (code): the hold-point retreat
  ray, the registry repair operators in the body frame, the six bundle
  names in every bundle, the legacy AirSim scripts refusing the metre point
  form, the episode manifest as a frame of every rosbag, single_sha in the
  KPI report, GPS-dropout replay, the goto pilot's terrain feed-forward,
  GPS noise and latency on the rail, a literal SET_MODE LOITER (opt-in)
  and BitVLA's loader. They were shaken down on 22 Nov before the smoke
  and the nightly (docs/data/shakedown_20261122/). The flight code is
  unchanged from 59e19a5 to this report's commit.

| KPI (grant) | Target | KPI-grade result, ntFIN4 (59e19a5) | Earlier: ntFIN3 (71aabb3) | Verdict (ntFIN4) |
|---|---|---|---|---|
| P0 violation escape rate (WP3-14) | 0 | 0.0000: 0 escapes on 17,078 P0-violation ticks, 650 Shield-on flights; Shield-off control 0.6500 (156 of 156 P0 ticks, 3 flights); passthrough null 0.1935 | 0 escapes on 17,457 P0-violation ticks, 591 flights; control 0.6458; passthrough null 0.2220 | meets |
| Fail-safe trigger correctness (WP3-15) | ≥ 0.99 | (r2, adopted): 648/648 = 1.000, Wilson 95 % low 0.994, 18 triggered by the GeoFence backstop (decision 14), all expected; (r) 636/648 = 0.981; (a) 573/648 = 0.884; before the backstop rule 630/630; 2 runs censored in Loiter (648/650 = 0.997 if both were errors) | (r2) 587/589 = 0.997, Wilson 95 % low 0.988; before the backstop rule 569/571 | meets as the point estimate, the grant's figure, and at 95 % confidence |
| Monitor tick ≤ 100 ms end to end (WP3-13) | ≤ 100 ms | `tick_ms` p50 7.8 / p99 28.4 / max 105.50 ms; `e2e_ms` p50 7.9 / p99 28.5 / max 105.60 ms; 1 of 183,909 ticks over 100 ms on each (ntFIN4_236, a hot-apply tick, section 5.4) | `tick_ms` max 74.52 ms, `e2e_ms` max 74.62 ms, 0 of 174,186 over 100 | within at p99; not on every tick (1 tick at 105.5 ms) |
| Mean time to safe (WP3-17) | tracked | 3.39 s mean, max 9.70 s, over the 169 flights whose unsafe spell recovered; 95 never recovered (counted, not averaged) | 3.38 s, max 9.50 s (170); 95 never recovered | tracked |
| Repair success rate (WP3-20, X-02) | tracked | 98.1 % (27,097 of 27,615 repair ticks) | 98.1 % (26,747 of 27,258) | tracked |
| Mean repair magnitude (WP3-16) | tracked | 6.338 m/s (translational, pooled over repaired ticks; yaw rate apart) | 6.129 m/s | tracked |
| Average repair count per episode (X-03) | tracked | 50.2 per Shield-on flight | 54.4 | tracked |
| Mission success rate (WP4-08) | tracked | 287/650 Shield on (our rule is stricter than the grant's: no dwell in a zone, no unsafe position entered, goal reached) | 252/591 | tracked |

Source of the ntFIN4 figures: the KPI report
docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md and .json
(`python tools/kpi_report.py --campaign demo/out/hil_ntFIN4 --rescore-rail
docs/data/kpi_ntFIN4/rescore_r2_ntFIN4.json --baseline
docs/data/kpi_ntFIN3/kpi_report_ntFIN3_2026-11-21.json`), made from a clean
tree at 2f9d863, with its re-score table (rescore_r2_ntFIN4.json and .txt,
`experiments/sweep_scenarios.py --profile nightly --rescore-rail` over the
654 runs) and the replay at the flown revision
(replay_at_revision_ntFIN4.json) beside it. The earlier reports stay as
they were made: docs/data/kpi_ntFIN3/ (21 Nov), docs/data/kpi_ntFIN2/
(13 Nov), docs/data/kpi_ntFIN2_20261115/ (15 Nov, the backstop rule) and
docs/data/kpi_ntFIN2_20261117_delivery/ (17 Nov, the setpoint delivery
regrade).

Other locked KPIs, not from the nightly (docs/FINAL-REPORT-outline.md
section 2):

| KPI (grant) | Target | Evidence |
|---|---|---|
| DSL acceptance (WP1-19) | schema 100 %, golden 100 %, cold start ≤ 1 s, hot-apply ≤ 50 ms, query ≤ 5 ms | AGX Orin: cold start p95 590 ms, hot-apply p99 3.86 ms, query p99 0.977 ms; schema and golden 100 % (docs/data/wp1_roundtrip.json); in flight on hil, REST receipt to swap 0.81-6.35 ms over 10 events (00264ef; one 53.0 ms spawn before that fix) |
| Policy load round-trip (WP1-17) | 100 % | round-trip 32/32, tamper 32/32, negative corpus 39/39 refused (docs/data/wp1_roundtrip.json, 6 Nov, clean tree 4edd5d1) |
| Bundle replayability (WP1-18) | load the bundle whose hash matches | hil_smoke_03 (KPI-grade): its signed replay bundle verifies and re-derives its KPIs |
| Prefix-token budget (WP2-02) | budget respected, CSPBudgetExceeded on overflow | docs/data/csp_kpis.json; tests/test_csp.py |
| CSP P0 coverage (WP2-03) | 100 % | 122/122 P0 at 256 tokens and at 144 tokens (docs/data/csp_kpis.json, 6 Nov, clean tree 52ac56c) |
| Per-paraphrase robustness (WP4-09) | stability across paraphrases | `hil`, KPI-grade (15 Nov, AIoT-VLA on the AGX Orin, the PAS follow): 6/8 paraphrases ended as the source "follow a red car" did (target_lost, twice), 2/8 held the car to max_s, as the model's own noun phrase "a red car" did; 0 P0 escape ticks over 13 runs; the stress harness's own arms fly the text-blind stub. 7/8 with OpenVLA-7B on one AirSimNH mission (7 Nov, not KPI-grade) |

## 3. System architecture and topologies

- **Action contract.** A 4-D body-frame action (vx, vy, vz, yaw_rate) at
  10 Hz (ARCH-05, WP3-03: DONE). Since 4 Nov the Shield takes and returns the
  body frame at its own boundary (`Shield.filter_body`); body to local NED
  happens after the projection, in sitl/mavlink_adapter_node.py on the
  ROS 2 rail, and every flying rail hands the Shield the body action
  (ARCH-04: DONE since 20 Nov). Since 783e626 (21 Nov, flown in ntFIN4) the
  registry repair operators are handed and hand back the body-frame action
  at the operator interface; the last-resort escapes (ClearanceEscape, the
  last-resort GeofenceEscape) still compute in the world frame inside the
  Shield's own rotation (WP3-04: PARTIAL). Since 14 Nov a learned VLA (AIoT-VLA on the AGX Orin) emits the
  setpoint stream into ArduPilot on `hil` (ARCH-01: DONE since 15 Nov).
- **Pipeline.** Bundle → CSP → pilot → Shield (+ FSM) → MAVROS 2 →
  ArduPilot, with ArduPilot's GeoFence as the last backstop, set by the GCS
  since 3 Nov (WP3-10, ARCH-09).
- **AP_DDS (ARCH-08).** The optional Q3 migration to ArduPilot's AP_DDS has
  an opt-in output path (sitl/ap_dds/, docs/RUNBOOK-ap-dds.md) that has not
  yet run on SITL; MAVROS 2 remains the ROS 2 bridge to ArduPilot.
- **Topologies** (ARCH-11 to ARCH-14):

| Topology | Grant definition | State on 2026-11-23 |
|---|---|---|
| `dev` | everything on one desktop | flown: SITL + MAVROS 2 (7 Oct); PAS + ArduPilot + MAVROS 2 in WSL, `pas_dev_mavros_02` (29 Oct); Gazebo over pymavlink and MAVROS 2 (2 Nov). A `dev` run counts for KPIs only with the PI's written waiver. |
| `hil` | desktop runs the simulators, MAVROS and the GCS; the Orin runs the VLA and the Shield | flown since 6 Nov on the AGX Orin: SITL and mavlink-router on the desktop; MAVROS 2, the Shield and the stub pilot on the Orin. First KPI-grade run hil_smoke_03; KPI-grade nightlies ntFIN2 (116214f), ntFIN3 (71aabb3) and ntFIN4 (59e19a5, the figure of record). A learned VLA on this topology: AIoT-VLA on the Project AirSim perception rail since 14 Nov (ARCH-20). |
| `flight` | Orin companion on a real ArduPilot drone | not flown. The drone cannot carry an AGX Orin; needs the PI's written waiver (ARCH-14). The real flight controller stays on ArduCopter 4.7.1, not flashed to the rig's 4.5.7 (the user's decision, 22 Nov; ARCH-31 to the PI). |

- **Rails.** ArduPilot SITL + MAVROS 2 (the KPI rail); Project AirSim +
  ArduPilot (the perception rail; a learned VLA on it on `hil` since 14 Nov);
  Gazebo Harmonic (flown end to end on 2 Nov, ARCH-16; first functional sweep 20 Nov, 7 of 8 missions passed, WP4-04). All three
  fly ArduPilot as the inner loop (ARCH-02). Project AirSim CityLife with
  simple_flight is a comparison bench (the follow comparisons of section 6),
  not a rail of the delivery and never KPI evidence.

![What runs where in the hil topology](img/what-runs-where.png)

*What runs where in the `hil` topology (the 11 Nov progress-deck figure): the desktop PC runs Project AirSim and ArduPilot SITL 4.5.7 with mavlink-router and the scenario tools; the AGX Orin runs the pilot (AIoT-VLA with its OWL-ViT detector, or the stub), the Shield with its escalation FSM, and MAVROS 2 in the companion container. The dashed boxes (Project AirSim, AIoT-VLA) were built but not yet flown on the rig when it was drawn; they flew on 14 Nov (section 6; docs/data/pas_aiot_hil_20261114/flights.md). The KPI campaigns fly the solid boxes with the stub pilot.*

## 4. Results by work package

Each subsection: what was delivered, what is built differently, what
remains. The per-obligation rows are in Appendix A.

### 4.1 WP1 Policy DSL and IR (25 obligations)

- Delivered: the IR with a spatial index, fences, corridors, time windows,
  breach actions, priorities, layered merge, ingest tools, the round-trip
  KPI (WP1-17), the replayable bundle (WP1-18, hil_smoke_03) and the DSL
  benches on the AGX Orin (WP1-19).
- ConstraintBase requires the grant's four fields since IR schema 4.0.0
  (6 Nov; `scope` was the last, WP1-01). Every shipped rule states
  `scope: mission` (CHANGELOG 2026-11-06 (IR 4.0.0)).
- Since 3 Nov every shipped policy is stored and hashed in WGS84 with a lazy
  local view (WP1-14), and since 18-22 Nov every flight loader refuses a
  policy stored in local metres; distance_envelope is enforced for
  buildings, roads and, since 19 Nov, every person the rail reports
  (WP1-07).
- Mid-flight hot-apply over REST (WP1-15) is flown on every rail: the ROS 2
  rail (29 Oct: dynamic_nfz, time_window_switch; 3 Nov: corridor_swap
  through the harness) and, on 15 Nov, the `hil` perception rail on the AGX
  Orin: 14 dynamic_nfz events in three KPI-grade missions, each a new
  generation with its policy and CSP written, the GeoFence re-uploaded
  after each, P0 escape 0. Receipt to swap was 0.81-6.35 ms over the last 10
  events; one earlier spawn took 53.0 ms because the loop held the rail's
  lock through the whole tick, which was fixed (section 8,
  docs/data/pas_hil_hotapply_20261115/).
- The mid-term Policy DSL + IR is delivered: guardrail/models.py,
  guardrail/compiler.py and the typed IR guardrail/ir.py (WP1-21).
- Open: the signing authority (WP1-04; on 22 Nov the user decided to wait
  for the PI to name the CA); compatibility with the PI's reference
  implementation (WP1-05); people kept at the minimum distance only when
  perceived (on pas_hil_people_02 the person feed matched 0.189 of
  pedestrians within 25 m; WP1-07); metre input still accepted at the DSL,
  REST and gRPC entry, converted at ingest (WP1-14).

### 4.2 WP2 Prefix Compiler (19 obligations)

- Delivered: the CSP model and schema, the token budget (WP2-02), P0
  coverage (WP2-03), relevance explanations, the adapters, templates and
  tests. The paraphraser is a separate gRPC service with mutual TLS since
  2 Nov (WP2-13).
- On 7 Nov OpenVLA-7B flew with the CSP in its prompt: an A/B on three seeds
  and eight paraphrases from the paraphraser service (AirSimNH, not
  KPI-grade). WP2-01, WP2-09, WP2-18 and WP4-10 became DONE. On base
  OpenVLA the CSP's effect on repairs is no larger than a rule-free
  prefix's (CHANGELOG 2026-11-07 (CSP flights)).
- The WP2 and WP4 correction notes were approved by the PI on 6 Oct and
  sent to ITRI (confirmed 2 Nov; WP2-12, WP4-21).
- The Project AirSim scene steps ran on Python 3.12 for the AIoT-VLA `hil`
  missions (14 Nov), closing WP2-19 with WP1-25.

### 4.3 WP3 Safety Shield (24 obligations)

- Delivered: the ROS 2 Shield node with its projection operators (WP3-01),
  monitor / filter / projection (WP3-02), the 10 Hz ROS 2
  interface (WP3-03), the REST listener driven in flight (WP3-12), one flown
  zone per breach action (WP3-08), the audit record with the grant's fields
  (WP3-11, shield_audit.jsonl), the node architecture (WP3-23), the IR
  invariant (WP3-21), the package (WP3-24), the MAVLink adapter (WP3-09:
  since 15 Nov an in-flight mission update is written whole -
  MISSION_COUNT, MISSION_ITEM_INT per item, ACK ACCEPTED - flown in AUTO on
  mission_wp_20261115b, and the perception rail's setpoint can go through the
  adapter, flown KPI-grade on `hil` as pas_hil_adapter_01).
- KPI-grade on `hil` (ntFIN4 at 59e19a5, as in ntFIN3 at 71aabb3 and
  ntFIN2 at 116214f before it): P0 escape rate 0 (WP3-14); fail-safe
  trigger correctness 648/648, Wilson 95 % low 0.994 (WP3-15); the tracked
  KPIs repair magnitude, time to safe and repair success (WP3-16, WP3-17,
  WP3-20); section 5. The monitor tick (WP3-13) is within 100 ms at p99
  but not on every tick: one ntFIN4 tick took 105.5 ms (section 5.4), so
  WP3-13 moved back to PARTIAL on 23 Nov.
- The grant's five-row Shield test matrix (high-speed near-edge, sudden
  NFZ hot-applied, time-window switch, three simultaneous violations, GPS
  noise / latency) flew whole in one KPI-grade smoke for the first time:
  smFIN2 at 59e19a5 (52 KPI-grade missions, P0 escape 0, fail-safe (r2)
  51/51; docs/data/kpi_smFIN2/). Since 115640b (21 Nov) the GPS noise and
  latency are applied on the companion to the pose the Shield and the FSM
  read (the autopilot's EKF sees clean GPS), checked per run
  (rail_stress_check). 59e19a5 is not a release, so the per-release clause
  is still open (WP3-19, WP4-14).
- One rule-evaluation code path (WP3-22): DONE since the 23 Nov recount.
  FenceGuard, the last flown second measure, now measures with the
  Shield's field and judges with its clearance_breached; "grid" is refused
  (wf21-onepath3, 18 Nov; flown on 7 + 2 AGX Orin follow flights, 19-20 Nov,
  functional). X-13's wider "nothing else evaluates rules" stays PARTIAL:
  the autopilot's GeoFence backstop judges in flight and triggered 18 of
  ntFIN4's 648 scored fail-safes, and start_conflicts is its own
  point-in-zone lint.
- Escalation FSM edges taken in ntFIN2 (from each run's metrics.json `fsm`;
  Appendix A WP3-07): G1 348, G3 334, G5 253, G9 216, X1 146, G7 93, G8 64,
  X5 54, G11 18, G10 3, G12 3, X4 3, X9 1. X6, X7 and X8 flew on `hil`
  before (rh01, rh03, 6-7 Nov).
- The edges no rail had taken flew on 14 Nov in hs10 (8a473fa, `hil`,
  profile `fsm_edges`, one seed per cell; demo/out/hil_hs10/sweep.json, runs
  demo/out/hil_smoke/hs10_*; CHANGELOG 2026-11-14 (hs10 ...)): G2 and G4 on
  fsm-n-in-t-flicker (hs10_003: X1, G2 at 5.8 s "N-in-T: 3 onsets in 5 s",
  G4 at 10.0 s, RTL, G9); X2 (hs10_001) and X3 (hs10_002) from a hold; G6
  on a blocked leg home (hs10_004: G11, G6, G10) and on no progress home
  under wind beyond authority (hs10_005: G1, G3, X9, G6, G10; no progress
  home is our RTL-failure reason, not one the grant names). G6's input is
  the node's RTL watch (`RTLFailureWatch`, sitl/ros2_shield_node.py;
  CHANGELOG 2026-11-14 (WP3-07)), which replaced `rtl_failed=False`: the
  Shield's check of the leg home against its hard P0 zones, and progress
  home while ArduPilot reports RTL. P0 escape 0 and 0 polygon entries over
  the 5 missions; `e2e_ms` max 41.3 ms over 423 setpoint ticks (2,181
  rows).
- By default the Loiter is not SET_MODE LOITER on `hil`: since 7 Nov the
  companion holds it in GUIDED (node default `--hold-flight guided`,
  sitl/ros2_shield_node.py:6247); hs10_003 logs "LOITER held in GUIDED by
  the companion" at t = 5.828 s with mode requests ["RTL"]; ntFIN2 has 337
  Loiter entries (G3 334, X4 3) and 0 LOITER requests (RTL 272, LAND 290,
  GUIDED 64). The campaign was planned, driven and scored on Python 3.12
  (sweep.json `_runner`).
- SET_MODE LOITER flew on `hil` on 22 Nov, opt-in (`--rail-hold loiter`,
  29a1440): sk1122b at 59e19a5, 2 missions; on both the autopilot's
  heartbeat reported LOITER, the Loiter held 29.9 s within 0.021 m and
  0.005 m of its altitude, then the 30 s timeout took G5 to RTL; P0 escape
  0 (docs/data/shakedown_20261122/README.md). The throttle is centred by a
  SITL-only stand-in for the safety pilot (RC_CHANNELS_OVERRIDE ch3 1500 as
  sysid 255); no G8 or G4 flew in that mode, and the path cannot yet run
  under SROS 2 (the adapter is not granted /uas1/mavlink_sink).
- Built differently: PathRepair's boundary and zone detours are opt-in
  and the follow rails keep it off, a measured deviation (WP3-06); the PAS
  rail publishes its own setpoint by default, the adapter route being
  opt-in (WP3-09); the last-resort escapes compute in the world frame while
  the registry operators act in the body frame (WP3-04).
- Open: WP3-07 stays PARTIAL: the default does not drive SET_MODE LOITER
  (decision 7, PI confirmation pending), and a mission profile's FSM YAML
  cannot be selected on the rails (the `profiles:` overlay is reachable
  only from the library); G6's battery reason is not flown
  (failsafe-low-battery stays refused: SITL reaches CRITICAL only through
  BATT_CRT_* thresholds the rig leaves at 0); fsm-n-in-t-flicker is
  labelled by the lab's decision 13 (the rubric's N1 expects no fail-safe
  where the grant's G4 sends the aircraft home); the tick budget on every
  tick (WP3-13, section 5.4); the smoke matrix per release (WP3-19).

### 4.4 WP4 Stress Testing (21 obligations)

- Delivered: the scenario library (WP4-02), the KPI report generator
  (WP4-07), the Shield audit in every harness bundle (WP4-20), mission
  success on `hil` (WP4-08), the six-field manifest on every KPI-bearing
  run (WP4-12), the paraphrase arms through the service (WP4-10).
- The grant's worked example (`grant_example` profile, 36 episodes) flew on
  `hil` as ge1110 at 9eed35b: 32 missions flew and reached W3 with no repair
  and no unsafe spell; 4 died before the mission began, a fence-raise race
  fixed in 183bb3e (CHANGELOG 2026-11-11 (hil rig flights, 7-11 Nov) and
  (fence read and take-off)). Its "mean time to safe ≤ 2.0 s" holds only
  vacuously: no episode is ever unsafe (CHANGELOG 2026-11-07 (grant example
  sweep)). The same sweep flew again at 2f22116 as ge1113 (13-14 Nov, 36
  missions, all KPI-grade; demo/out/hil_ge1113/sweep.json; CHANGELOG
  2026-11-14 (hs10 ...)): 36 passed, mission success 27/36, no
  P0-violation tick, no repair and no unsafe spell, so its P0 rate is
  vacuous and its fail-safe labels (36/36, all "no fail-safe") do not
  discriminate (`tools/kpi_report.py --campaign demo/out/hil_ge1113`, not
  committed). It is latency evidence at the fixed commit (section 5.6), not
  Shield evidence.
- Since 22 Nov every episode bundle holds the grant's six names: all 654
  ntFIN4 episodes and all 52 smFIN2 episodes record manifest.json,
  policy_bundle.tar.gz, episode.bag2/, shield_audit.jsonl (empty on a
  zero-repair run), harness_events.jsonl and kpi.json as written, and every
  ntFIN4 bag holds the latched /guardrail/manifest frames (WP4-04, WP4-11,
  ARCH-22).
- Built differently: no release has run the five-row smoke set: v0.6.0's
  rel060 flew four rows, smFIN2 flew all five at the unreleased 59e19a5
  (WP4-14).
- Open: the KPI report's delivery with this report (WP4-17; the report
  itself is committed, section 2); KPIs in this report that do not come
  from a Stress Testing run in `hil` (DSL, CSP and per-paraphrase, section
  2; WP4-01); Gazebo functional sweeps beyond one seed and the broad sweep
  (WP4-04); a CI smoke that completes per build, a nightly cadence and the
  broad sweep (WP4-06: master was pushed on 22 Nov, and both GitHub runs
  since were red before the smoke step); bit-for-bit replay on every
  episode and the Q3 replay for reproducibility (WP4-11); the final demo
  gate with a dynamic suite scenario and the Prefix Compiler in the loop
  (WP4-19).

## 5. KPI report (WP4-17)

### 5.1 Source

The nightly `hil` campaign ntFIN4 at 59e19a5, the revision this report
describes (section 2): 653 KPI-grade missions, 650 Shield on and 3
Shield-off controls. The KPI report is
`docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md` (and .json),
generated by tools/kpi_report.py from a clean tree at 2f9d863 with the
ntFIN3 report as its baseline. Every flown episode is re-scored from its
own log with `guardrail.kpi.compute`, the function the flights use.
Fail-safe labels come from the committed label artefact
docs/data/failsafe_labels_rail.json, re-scored with
`experiments/sweep_scenarios.py --profile nightly --rescore-rail`
(docs/data/kpi_ntFIN4/rescore_r2_ntFIN4.json). The earlier figure of
record, ntFIN3 at 71aabb3 (docs/data/kpi_ntFIN3/), is quoted beside it
where a comparison helps.

### 5.2 P0 violation escape rate

- 0.0000 over 650 Shield-on flights: 0 escapes on 17,078 P0-violation
  ticks (ntFIN3: 0 on 17,457 over 591).
- Shield-off control (nfz-head-on-control, 3 flights): 0.6500, 156 escapes
  on 156 P0 ticks. Passthrough null: 0.1935.
- The count is the Shield's own log, as the grant defines it ("Counter from
  Shield repair log"). On the true position, the KPI report's table "What
  the autopilot flew on the rail runs" lists the moves ArduPilot made into
  an unsafe position after the FSM handed the aircraft to RTL, Land or the
  hold. In 22 Shield-on missions it entered a P0 polygon: the 18
  dyn-nfz-spawn-ahead runs whose GeoFence breach took them to RTL
  (ntFIN4_178-186, ntFIN4_214-222; section 5.3) and 4 wind-beyond-authority
  missions at 12 m/s (ntFIN4_647 in lane 15, ntFIN4_652-654 in lane 20), as
  in ntFIN3. In 14 more it breached a keep-out margin without entering the
  polygon (13 dyn-nfz-spawn-ahead, 1 failsafe-rtl-zone). These are outside
  the grant's escape count. The RTL drift in a 12 m/s wind, first shown by
  the shakedown hs07, is open.

### 5.3 Fail-safe trigger correctness

| Reading | Correct / scored | Point (the KPI) | Wilson 95 % low | Nulls: always / never |
|---|---|---|---|---|
| (a) ideal-vehicle labels | 573/648 | 0.884 | 0.857 | 0.418 / 0.582 |
| (r) rail-vehicle labels | 636/648 | 0.981 | 0.968 | 0.488 / 0.512 |
| **(r2) rail-vehicle labels, E5 on the prescribed flight, with the GeoFence backstop rule (adopted; the KPI)** | **648/648** | **1.000** | **0.994** | 0.506 / 0.494 |
| (r2), the 18 backstop runs read as before the rule (left out as censored) | 630/630 | 1.000 | 0.994 | - |
| (r2), the 18 backstop runs counted as not triggered | 630/648 | 0.972 | 0.957 | - |

- The verdict: 1.000 meets ≥ 99 % as the point estimate, which is the
  grant's figure, and its Wilson 95 % lower bound, 0.994, clears 0.99, so
  this campaign shows 99 % at 95 % confidence. It does so without the
  backstop rule too (630/630, Wilson low 0.994). ntFIN3 at 71aabb3
  (587/589, lower bound 0.988) did not clear it; one campaign clears the
  bound and the one before did not.
- No error under (r2). ntFIN3's two errors, missed triggers on
  dyn-nfz-spawn-ahead (ntFIN3_164, ntFIN3_168), did not recur.
- Left out: ntFIN4_374 and ntFIN4_375, corridor-swap-shifted, censored in
  Loiter. Were both errors, 648/650 = 0.997.
- 18 fail-safes were triggered by the GeoFence backstop while the FSM held
  in Loiter (decision 14): all dyn-nfz-spawn-ahead (ntFIN4_178-186,
  ntFIN4_214-222; RTL at t = 5.71-5.87 s), all expected, all correct.
- On the labels the campaign flew with: 586/649 = 0.903 (54 false, 9
  missed).
- failsafe-rtl-zone ends in Land on most seeds, as in ntFIN3: the node's
  RTL watch finds the leg home through the zone and lands (G6); the
  fail-safe is correct either way, and the scenario's outcome gate, written
  for RTL, fails them.
- (r2) was adopted on 10 Nov (docs/RUBRIC-failsafe.md sections 7-8; CHANGELOG
  2026-11-10 (fail-safe labels adopted)), after earlier rig flights had been
  scored on another reading; the gps-dropout labels were re-derived on
  12 Nov and the backstop rule was settled by the lab on 15 Nov. All three
  were committed before ntFIN4 flew, so ntFIN4 was scored on rules fixed
  before its flights. The PI's confirmation of (r2) and of the backstop
  rule is still to be recorded.

### 5.4 Tick budget

| Measure | Ticks | p50 | p95 | p99 | Max | Over 100 ms |
|---|---|---|---|---|---|---|
| `tick_ms` (tick start to setpoint) | 183,909 | 7.77 | 22.99 | 28.36 | 105.50 | 1 |
| `e2e_ms` (action receipt to its first setpoint) | 183,909 | 7.88 | 23.09 | 28.46 | 105.60 | 1 |

- The budget is met at p99 (28.4 ms) and not on every tick: one tick of
  183,909 took 105.5 ms (`e2e_ms` 105.6 ms). ntFIN3 at 71aabb3 had 0 of
  174,186 over 100 ms (max 74.52 ms); the smoke smFIN2 at 59e19a5 had 0 of
  15,662 (max 72.91 ms).
- The tick: `ntFIN4_236_dyn-nfz-spawn-ahead`, row 50 (t 5.03 s), shield_ms
  10.581, FSM edge G3 (Brake to Loiter). It is the tick on which the
  scenario's zone is hot-applied: before the setpoint, the node applies the
  event, writes the new policy generation (policy_g1.json; the CSP compile
  with its first, cold template load, csp_g1.json) and publishes the
  fences. That block normally takes 20-40 ms (median about 32 ms over the
  nightly's spawn ticks; spawn ticks that are also G3: median 51.0 ms,
  31.8-58.8 ms apart from this one); here it took 87 ms, an I/O or
  scheduling stall on the Orin (no GC, no REST apply, no fsync). The code
  path is older than 71aabb3, so ntFIN3's 0 over did not exclude it
  (docs/data/kpi_ntFIN4/README.md, "The one tick over 100 ms").
- The fix followed the flight and is in no figure of this report: the
  generation write, the CSP compile and the fence publication move after
  the setpoint, the templates are compiled at bring-up, and each row
  carries per-phase tick stamps (described in docs/data/kpi_ntFIN4/README.md;
  not committed at this report's revision). WP3-13 is PARTIAL until it is
  committed and a KPI-grade nightly with hot-applied zones flies with no
  tick over 100 ms.
- "End to end" starts at the Shield node's receipt of the pilot's message,
  not at a camera frame. `cam_age_ms` and `cam_to_setpoint_ms` are not
  measured on this rail: no camera is in the `hil` stub loop.
  Measured 14 Nov with AIoT-VLA on the AGX Orin (docs/data/pas_aiot_hil_20261114/flights.md): camera frame arrival to the first setpoint p50 159-163 ms, p99 282-296 ms over six missions; the VLA's own inference (capture to answer p50 0.110-0.113 s) and the detector account for it.
- AGX Orin profile (6 Nov at d0e91bb, 50 rules x 51 poses): query p99
  0.977 ms, Shield step p99 22.647 ms
  (deploy/evidence/hil/shield_tick_profile.json); re-run 14 Nov at bc04e4a:
  query p99 0.994 ms, step p99 26.709 ms; not re-run at 59e19a5.

### 5.5 Tracked KPIs per family (Shield on, KPI-grade)

| Family | Episodes | Mission success | Mean time to safe (s) | Repairs / ep | Repair magnitude (m/s) | Repair success |
|---|---|---|---|---|---|---|
| all Shield-on flights | 650 | 287/650 | 3.39 (max 9.70), +95 never recovered | 50.2 | 6.338 | 98.1 % |
| altitude_envelope | 21 | 18/21 | 3.19 | 27.7 | 1.969 | 99.0 % |
| corridor_keep_in | 42 | 12/42 | never unsafe | 194.1 | 5.435 | 99.4 % |
| corridor_swap | 9 | 0/9 | 1.90 | 28.9 | 2.263 | 88.8 % |
| dynamic_nfz_movement | 164 | 20/164 | 3.22, +18 never recovered | 24.1 | 3.317 | 94.2 % |
| failsafe_escalation | 27 | 0/27 | 0.22, +15 never recovered | 1.1 | 3.600 | 0.0 % |
| high_speed_near_edge | 90 | 90/90 | never unsafe | 174.3 | 7.651 | 100.0 % |
| kinematic_envelope | 3 | 3/3 | never unsafe | 300.0 | 16.000 | 100.0 % |
| radius_scaling | 15 | 6/15 | 3.90 | 5.9 | 5.656 | 83.1 % |
| sensor_degradation | 84 | 48/84 | none recovered, +3 never recovered | 9.5 | 5.561 | 98.5 % |
| static_nfz | 6 | 0/6 | never unsafe | 5.2 | 6.028 | 64.0 % |
| subject_standoff | 60 | 48/60 | never unsafe | 6.1 | 0.891 | 93.4 % |
| three_simultaneous_violations | 54 | 0/54 | 3.46, +54 never recovered | 8.0 | 10.295 | 0.0 % |
| time_window_static | 6 | 3/6 | never unsafe | 1.0 | 5.656 | 0.0 % |
| time_window_switch | 51 | 36/51 | 3.87 | 23.9 | 2.091 | 87.6 % |
| wind_disturbance | 18 | 3/18 | 6.58 (max 9.70), +5 never recovered | 5.2 | 5.656 | 84.0 % |

Nulls: a hover passes 0.0 % of the missions; an always-brake Shield's repair
success is 0.0 %. The ten worst failures are three wind-beyond-authority
missions at 12 m/s in lane 20 (ntFIN4_652-654) and seven
three-simultaneous-violations missions (ntFIN4_500-505, 512), all with the
unsafe position never recovered, each linked to its signed replay bundle in
the KPI report.

What flew differently from ntFIN3:

- sensor_degradation has 84 Shield-on flights (24 in ntFIN3): the 60
  GPS-noise / latency missions flew on the rail, the noise drawn on the
  companion from each episode's seed (metrics.json `perception_stress`);
  mission success 48/84.
- PathRepair acted on the same three standoff-approach-angles@offset_m=9
  variants (pedestrian subject) as in ntFIN3, now in the body frame: 114
  repair ticks each, mean magnitude 0.684 m/s, P0 escape 0, fail-safe 3/3,
  mission success 3/3; no other family shows a PathRepair tick.
- dynamic_nfz_movement has 164 KPI-grade flights (165 in ntFIN3): one
  dyn-nfz-translate run was refused by the setpoint-delivery rule.
- No error under (r2) on any cell (section 5.3).

### 5.6 After ntFIN2: the start-up fix (history)

- Fix: 5f5009c (the node delivers its start-up backlog before t = 0 and
  holds it), merged to master in d7bb6a2.
- Shakedown hs09 at 2f22116 (`hil`, sim_speedup 1.0, clean revision
  2f221168625c): nfz-head-on, corridor-drift-angles and
  wind-beyond-authority, seed 1001; 22 episodes planned, 5 paraphrase /
  prefix arms refused, 17 missions flown (demo/out/hil_hs09/sweep.json;
  runs demo/out/hil_smoke/hs09_*).
- Result over 7,083 ticks: `e2e_ms` p99 16.75 ms, max 24.55 ms; `tick_ms`
  max 24.46 ms; no tick over 100 ms; row 2 of all 17 missions ticked on the
  action; P0 escape rate 0 on every Shield-on mission.
- hs09 is a shakedown of three templates, not the nightly. On the labels it
  flew with, one cell failed (hs09_001, nfz-head-on canonical arm: labelled
  not to trigger, triggered RTL). Re-scored with the label artefact
  (`experiments/sweep_scenarios.py --profile nightly --rescore-rail`; output
  docs/data/kpi_ntFIN2/rescore_r2_hs09.json): (r2) 17/17 = 1.000, Wilson 95 % low 0.816;
  (r) 17/17; (a) 16/17 = 0.941, the false trigger hs09_001. Under (r) and
  (r2), hs09_001's label is read on the cell flown as long as the rail
  flies; without it 16/16.
- A full campaign at the fixed node code (5f5009c; the Shield code is
  unchanged from 116214f, the diff touches only the node's start-up path),
  unloaded: the grant-example sweep ge1113 at 2f22116 (`hil`, 36 missions,
  all KPI-grade, 3 rules plus one hot-applied zone, 0 repairs, 0 Brake
  ticks; demo/out/hil_ge1113/sweep.json;
  `tools/kpi_report.py --campaign demo/out/hil_ge1113`, committed as docs/data/kpi_ge1113/kpi_report_ge1113_2026-11-14.md, code revision 181b75cdaad3):
  over 95,160 ticks `tick_ms` p50 7.2 / p95 8.7 / p99 9.8 / max 66.28 ms and
  `e2e_ms` p50 7.3 / p95 8.8 / p99 9.8 / max 66.37 ms; 0 ticks over 100 ms;
  each mission's maximum is its hot-apply tick.
  Its missions never came near a zone (no P0-violation tick, no repair), so
  it shows the tick budget at the fixed commit and nothing about the
  Shield's other KPIs (section 4.4).
- Loaded: hs09 (above, 7,083 setpoint ticks) e2e max 24.55 ms; hs10 at
  8a473fa (section 4.3) e2e max 41.3 ms over 423 setpoint ticks (2,181
  rows); ntFIN2 (116214f, the same Shield code) e2e max 74.14 ms outside
  the start-up rows (ntFIN2_364).
- `e2e_ms` spans the Shield node only, from the /vla/action_4d callback to
  /shield/setpoint; the pilot's DDS hop and the adapter's hop to MAVROS are
  not in it (the adapter hop, read from the runs' bags, is about 1.3 ms p50,
  2 ms p99). Every `hil` figure has the stub as pilot and no VLA on the
  Orin; ~50 rules appear only in the offline Orin profile (d0e91bb, 6 Nov,
  MAVROS stopped, not re-run at 2f22116). Re-run 14 Nov at bc04e4a: the ~50-rule Orin profile (MAXN, clocks not locked, MAVROS stopped; deploy/evidence/hil/shield_tick_profile_20261114.json) _check p99 0.994 ms (max 1.056, 0 of 1,301 over 5 ms), filter p99 26.709 ms (max 48.814); the loaded cells at the fixed node code: hs11 (bc04e4a, dyn-nfz-spawn-ahead, 16 KPI-grade missions of a 55-mission plan stopped by the launcher's time limit, the 17th incomplete) e2e p99 37.1 / max 66.08 ms over 5,121 ticks, and the harness-driven hh02 (bc04e4a, 6 missions incl. radius-scaling-grow-deep) e2e p99 26.9 / max 83.57 ms over 2,669 ticks, its maximum a REST scale tick; 0 over 100 ms in either. three-simultaneous-violations was not reached in hs11.
- WP3-13 was DONE on these runs from 14 Nov. ntFIN2's 13 start-up ticks
  stay in its committed KPI reports as flown, before the fix; the nightly
  at 71aabb3, ntFIN3, flew 0 of 174,186 ticks over 100 ms. The nightly at
  59e19a5, ntFIN4, flew 1 of 183,909 over 100 ms on a code path older than
  both (section 5.4), and WP3-13 moved back to PARTIAL on 23 Nov.

### 5.7 Read with care

1. **The pilot.** Every KPI-grade run flies a stub (GotoPilot,
   ConstantPilot, StubVLA). The figures measure the Shield and its FSM in
   front of ArduPilot SITL under scripted commands, not a learned model.
2. **Independence.** Apart from the 60 GPS-noise / latency missions, whose
   noise is drawn from the episode's seed (metrics.json
   `perception_stress`), no flown cell consumes its seed: a cell's three
   seeds are three flights of one deterministic scenario that differ only
   by the rig's timing. 446 distinct KPI signatures over the 654 ntFIN4
   episodes (demo/out/hil_ntFIN4/sweep.json `_distinct_kpi_signatures`; 376
   over 594 in ntFIN3). The rates and the Wilson bound count them as
   independent.
3. **Coverage.** 90 of 744 profile episodes were refused before take-off
   (section 2): paraphrase and prefix arms the stub cannot read, gusts and
   a low battery the `hil` rail cannot realise, and 12 others. The RTL that
   fails, G6's scenario, flies (failsafe-rtl-blocked); its battery reason
   does not.
4. **Definitions.** Repair magnitude leaves the yaw rate out; mission
   success is stricter than the grant's; time to safe averages only the
   recovered spells; theta judges about a third of the converged repair
   ticks (9,202 of 27,210 in ntFIN4).
5. **Provenance.** The KPI report of record is committed in the repository,
   made from a clean tree: docs/data/kpi_ntFIN4/ (section 2). Its "against
   the baseline" section lists every run as new or gone because it matches
   episodes by run name, and ntFIN3_NNN never matches ntFIN4_NNN; its
   per-family figure table is the comparison.
6. **Replay at the flown revision.** `tools/replay_at_revision.py` replayed
   the 654 runs open loop at 59e19a5 on the desktop, 303,022 ticks
   (docs/data/kpi_ntFIN4/replay_at_revision_ntFIN4.json, with a436d32, which
   hands the Shield the pose it judged under GPS noise): 355 identical; 298
   differ only in the last float digits (at most 6.9e-14; 112 plain, 186
   through the body-frame rotation); 0 are not replayable (the 24
   gps-dropout runs replay since the node logs its pipe-clock readings); 1
   differs in a repair's pass count: ntFIN4_304 dyn-nfz-rotate-spin,
   GeofenceSlide once against twice on a convergence test that sits on the
   last float digit, with the same emitted action to 6.9e-14, as ntFIN3_306
   in ntFIN3. The Orin (aarch64) and the desktop (x86-64) round that digit
   differently.
7. **Earlier campaigns.** ntFIN3 (71aabb3) is the earlier figure of
   record, kept beside ntFIN4 in sections 2 and 5; ntFIN2 and ntFIN2s
   (116214f) before it. A first attempt at 116214f, ntFIN (stopped at 176
   of 591 missions when the session ended), is in no figure; its runs are
   kept (demo/out/hil_smoke/ntFIN_*).

## 6. VLA backend measurement set (ARCH-30, X-04)

The grant asks for a measurement set before a default VLA backend is
chosen. The protocol table in the repository
(deploy/evidence/vla_backend_table.md, `vla-backend-protocol/2`) holds the
set and, since 18 Nov, names the default (section 6.4). This section adds
to it the follow comparison of 27-29 Oct, the AGX Orin timings of 3-6 Nov
and the lab's own AIoT-VLA.

The follow comparisons of this section are on Project AirSim CityLife with
simple_flight: a comparison bench outside the delivered architecture and never
KPI evidence (AIoT-VLA's follows on ArduPilot, on `hil`, are in
docs/data/pas_aiot_hil_20261114/ and docs/data/pas_hil_feed_20261115/). The
score is the share of the mission within 30 m of the red car: hierarchical
flights (the VLA chooses the intent, the hand-written controller flies it)
from the start gate, end-to-end flights from the first acquisition
(CHANGELOG 2026-10-28).

### 6.1 Protocol table

| Backend | Host, precision | Decision time | Follow, hierarchical | Follow, end to end | Sources |
|---|---|---|---|---|---|
| Stub P-controller (no model; the KPI campaigns fly stub pilots) | desktop | p50 0.0013 ms (n=50) | - | - | [T] |
| Hand-written follow (OWL-ViT + controller; no VLA) | PC; AGX Orin | - | PC 0.972 (n=5, std 0.012); AGX 0.952 (n=1) | - | [C29] [C4] |
| AerialVLA original (OpenVLA-7B + LoRA) | PC RTX 4090, bf16 / sdpa / merged | bench p50 259 ms; in flight 0.49-0.63 s p50 | 0.854 (n=5, std 0.305; median 0.987) | 0.539, 0.153 | [T] [C29] [C28] |
| AerialVLA original | AGX Orin, bf16 / sdpa / merged | bench p50 1073.9 ms (n=50; unmerged 1520.3 ms); in flight 1.44 s | 1.000 (n=1) | 0.048 | [E1] [C4] |
| AerialVLA QLoRA fine-tunes r1-r4 | PC, bf16 / sdpa / merged | in flight 0.48 s (r1), 0.66 s (r3) | 0.882-0.917 (n=2-3 each) | 0.018-0.065 | [C27] [C29] [C28] |
| DroneCATS-2B | PC, bf16 | bench p50 1.00-1.12 s; in flight 1.45 s | 0.933 (n=5, std 0.006) | 0.132, 0.086 | [C27] [C29] [C28] |
| DroneCATS-2B | AGX Orin, bf16 | in flight 2.64 s | 0.941 (n=1) | 0.000 | [C4] |
| DroneCATS-2B | Orin Nano, int4 | 9.6 s; beside the detector it timed out 34 of 36 times | - | - | [C28n] |
| RaceVLA-7B | PC | bench nf4 0.33 s; in flight 0.40-0.42 s | 0.720 (n=2) | 0.099, 0.084 | [C27] [C28] |
| RaceVLA-7B | AGX Orin, bf16 | in flight 1.22 s | 0.715 (n=1) | 0.055 | [C4] |
| OpenVLA-7B (base; reads the CSP prompt) | AGX Orin, bf16 | median 0.975 s (8 answers, timed after the first) | not flown on the follow | not flown on the follow | [C3e] |
| OpenVLA-7B (base) | desktop, nf4 / bf16 | warm 0.86 s / 3.35 s (load probes, 14 Jul, n far below 50) | - | - | [T] |
| AIoT-VLA (the lab's own: frozen OWL-ViT + trained head; reads the object's words, not the CSP) | RTX 4090 (v1) | model time p50 10.3 ms | - | v10b with the stand-off feed, red car: 0.969, 0.960, 0.955 (n=3, P0 escape 0.0) | [D] [C1] |
| AIoT-VLA | Orin Nano Super (v1) | model time p50 48.4 ms (600 captures, server alone) | - | - | [D] |
| AIoT-VLA v10b | AGX Orin, fp16 | model time p50 32.6 ms, round trip p50 39.6 ms (n=300, server alone) | - | Flown 14 Nov (docs/data/pas_aiot_hil_20261114/flights.md): pas_hil_aiot_kpi_01 (KPI-grade, 420 s, 119 interventions, P0 0) and five perception-stress missions. | [E2] |
| CognitiveDrone, BitVLA (named by the grant) | AGX Orin (BitVLA, PyTorch eager) | adapters exist (FAMILY= cognitivedrone / bitvla, 17 Nov); CognitiveDrone has published no weights; BitVLA's LIBERO-Spatial arm checkpoint loaded on the Orin 22 Nov: median 1.479 s per answer (n=20), 12 decisions in 20 s over TCP, every Action4D finite (a conformance check, not a flight policy) | - | - | [T] [B22] |

Sources:

- [T] deploy/evidence/vla_backend_table.md (generated 2026-10-06; the
  AerialVLA serving section 2026-10-21 from docs/data/aerialvla_serving_4090.json).
- [C1] CHANGELOG 2026-11-01 (late): runs aiot_v10bs_01-03; flown from the
  clean master 55faad0, signed copies in bundles/ (CHANGELOG 2026-11-06
  (contract)).
- [C27] CHANGELOG 2026-10-27 (flown): runs `demo/out/cmp_*`.
- [C28] CHANGELOG 2026-10-28: runs `cmp2_*` and the Orin Nano client runs.
- [C28n] CHANGELOG 2026-10-28 (night).
- [C29] CHANGELOG 2026-10-29 (later): runs `rep3_*` to `rep5_*`.
- [C3e] CHANGELOG 2026-11-03 (evening): deploy/orin/vla_bf16_check.py,
  verdict in deploy/evidence/hil/vla_bf16_check.json.
- [C4] CHANGELOG 2026-11-04: the 28 Oct matrix re-flown with every VLA
  served on the AGX Orin (MAXN, 6da813d).
- [D] docs/DESIGN-aiotvla.md ("On the Jetson Orin Nano", the serving check).
- [E1] deploy/evidence/hil/aerialvla_serving_bench_agx_merged.json and
  aerialvla_serving_bench_agx.json.
- [E2] deploy/evidence/hil/aiotvla_v10b_bench_agx.json (6 Nov).
- [B22] docs/data/bitvla_loader_20261122/README.md; CHANGELOG 2026-11-22
  (code), ARCH-06.

Read with care: one to five flights per configuration; the AIoT-VLA
flights (1 Nov) used the stand-off feed added that day, so they are not
the same configuration as the 27-29 Oct matrix; a server-alone time leaves
out the camera, the detector and the transport.

### 6.2 Flights per configuration, 27-29 Oct

On the PC the whole loop ran on the desktop. On the Orin Nano only the
flight client ran on the board; every VLA answered from the desktop's GPU
(CHANGELOG 2026-10-28, Limits). n is the number of flights, std the sample
standard deviation.

| Platform | Mode | Model | n | Mean | Std | Min-max |
|---|---|---|---|---|---|---|
| PC | controller | fixed 12 m stand-off (no VLA) | 1 | 0.999 | - | 0.999 |
| PC | controller | hand-written follow (no VLA) | 5 | 0.972 | 0.012 | 0.954-0.983 |
| PC | hierarchical | DroneCATS-2B | 5 | 0.933 | 0.006 | 0.928-0.940 |
| PC | hierarchical | AerialVLA FT r2 | 2 | 0.917 | 0.002 | 0.915-0.918 |
| PC | hierarchical | AerialVLA FT r3 | 3 | 0.897 | 0.034 | 0.869-0.934 |
| PC | hierarchical | AerialVLA FT r1 | 3 | 0.882 | 0.003 | 0.879-0.884 |
| PC | hierarchical | AerialVLA FT r4 | 2 | 0.882 | 0.013 | 0.873-0.891 |
| PC | hierarchical | AerialVLA (original) | 5 | 0.854 | 0.305 | 0.309-0.999 |
| PC | hierarchical | RaceVLA-7B | 2 | 0.720 | 0.001 | 0.719-0.721 |
| PC | pure VLA | AerialVLA (original) | 2 | 0.346 | 0.273 | 0.153-0.539 |
| PC | pure VLA | DroneCATS-2B | 2 | 0.109 | 0.033 | 0.086-0.132 |
| PC | pure VLA | RaceVLA-7B | 2 | 0.091 | 0.011 | 0.084-0.099 |
| PC | pure VLA | AerialVLA FT r1-r4 | 1 each | 0.018-0.065 | - | - |
| Jetson | hierarchical | AerialVLA (original) | 1 | 0.989 | - | - |
| Jetson | controller | hand-written follow (no VLA) | 1 | 0.976 | - | - |
| Jetson | hierarchical | DroneCATS-2B | 1 | 0.969 | - | - |
| Jetson | hierarchical | AerialVLA FT r1-r4 | 1 each | 0.876-0.924 | - | - |
| Jetson | controller | fixed 12 m stand-off (no VLA) | 1 | 0.799 | - | - |
| Jetson | hierarchical | RaceVLA-7B | 1 | 0.704 | - | - |
| Jetson | pure VLA | all seven models | 1 each | 0.017-0.113 | - | - |

How to read it:

- AerialVLA original, hierarchical, scored 0.979-0.999 in four of its five
  PC flights and 0.309 in one (`cmp2_hier_aerovla`): the detector lost the
  car after a corner. Its median is 0.987.
- In hierarchical mode the VLA's only lever while the car is locked is the
  stand-off, and the score falls with the stand-off. A fixed 12 m
  controller is the ceiling on the PC.
- No VLA beats the hand-written controller consistently. The fine-tuned
  AerialVLA runs (r1-r4) sit below the original in both modes.

### 6.3 AGX Orin

The AGX Orin replaced the Orin Nano as the `hil` machine on 3 Nov
(CHANGELOG 2026-11-03 (evening), (night), (late night) and 2026-11-04).

- OpenVLA-7B unquantized in bf16 on the board: median 0.975 s per action;
  OWL-ViT at 61 ms per frame (192 ms on the Orin Nano).
- The first follow flown from the AGX Orin (`agx_follow_01`, hand-written
  controller): 0.952 within 30 m, detector 2.57 Hz.
- AerialVLA unmerged on the board: 1.52 s per decision; hierarchical follow
  0.988; end-to-end follow 0.213 with one P0 escape against a phantom
  subject (the phantom is refused on replay since 4 Nov).
- The 28 Oct matrix re-flown with every VLA served on the AGX: the served
  latency was 2.1 to 3.9 times the Nano matrix's (median 2.4); over the six
  hierarchical VLA models flown in full on both, the mean score is 0.888 on
  the AGX and 0.891 on the Nano.

AIoT-VLA v10b on the AGX Orin in the hil topology, 14 Nov (docs/data/pas_aiot_hil_20261114/flights.md): six missions, Shield on, 0 P0 escape ticks, 22-297 interventions per mission, VLA decisions 6.9-7.9 Hz, camera to first setpoint p50 about 160 ms; one KPI-grade mission (pas_hil_aiot_kpi_01). 15 Nov, with the feed between ticks and the camera at a 0.05 s capture interval (docs/data/pas_hil_feed_20261115/flights.md; four KPI-grade flights, pas_hil_feed_05-08): 12.4-13.2 camera frames/s reach the model while it is asked, 10.6 and 10.9 decisions/s over the two missions that held the car for the full 420 s, camera to first setpoint p50 130-140 ms, 0 P0 escapes. The node's garbage collector is paced out of the tick since (guardrail/gc_pacing.py): its one tick of 227-454 ms every 65-80 s is gone (ticks over 100 ms: 3 of 4058 and 0 of 4096, none over 152 ms). Follow quality is weak on this rail (target lock 0.04-0.82, docs/data/pas_aiot_hil_20261114/perception_stress_report.md), well below its 0.955-0.969 on simple_flight.

### 6.4 Default backend: AIoT-VLA v10b for the follow task

The lab named it on 18 Nov, after the measurement set (ARCH-30, X-04;
`tools/vla_backend_table.py table --default`, which refuses the line until
the set is complete). The table records no confirmation by anyone else.

**Default: AIoT-VLA v10b for the follow mission on the perception and
`hil` rails, with OpenVLA-7B kept as the text-reading backend for the CSP
experiments (WP2).** On C2 (the follow on `hil`) it held the car within
30 m for 21-35 % of the mission over 3 flights, P0 escapes 0.

Reasons:

- It is the only learned model in the table that holds the car end to end:
  0.955-0.969 within 30 m over 3 flights. Every other model flown end to
  end (the published models and our AerialVLA fine-tunes) scored
  0.000-0.539.
- It is the only one whose decision time on the AGX Orin fits inside the
  grant's 100 ms tick: 32.6 ms p50, server alone. The other learned models
  served on the board took 0.975 s (OpenVLA-7B bench) to 2.64 s
  (DroneCATS-2B in flight) per decision.
- In hierarchical mode no VLA beats the hand-written follow consistently
  (CHANGELOG 2026-10-28), so a larger model buys no score there.

Against it:

- It reads the object's words, not the CSP, so it cannot carry WP2's
  prefix. That is why OpenVLA-7B stays for the CSP A/B.
- It is trained on CityLife follows of a few targets.
- Its comparison flights are on simple_flight; on `hil` it has flown the
  Project AirSim + ArduPilot rail since 14 Nov
  (docs/data/pas_aiot_hil_20261114/flights.md).
- The grant's reference backends (CognitiveDrone, BitVLA) are not in the
  comparison: CognitiveDrone has published no weights, and BitVLA's public
  checkpoints are robot-arm models; its LIBERO-Spatial checkpoint was
  downloaded and its real loader run on the AGX Orin on 22 Nov (1.479 s
  per answer, finite 4-D actions), a conformance check, not a flight
  policy (ARCH-06).

## 7. Deviations register

| Item | Grant | What we do | State |
|---|---|---|---|
| WP1-25, WP2-19, X-06 | Python 3.11+ | the AGX Orin companion runs Python 3.12.3 on every `hil` flight; since 14 Nov the desktop's non-VLA side runs Python 3.12.16 (guardrail-py312): the full CI at 8a473fa, 148/148 files (docs/data/ci_local/8a473fa0172e-py312.json), the `hil` campaign hs10 planned, driven and scored by it (sweep.json `_runner`), and the PAS launcher's scene / check / g1 steps (scripts/run_pas_ardupilot.ps1, not yet run on a live scene); vla-real (Python 3.10.20) stays for the VLA runtime (torch, transformers), which the grant scopes out of the 3.11+ decision (docs/DESIGN-python-versions.md) | one PAS scene step on 3.12; the PI to confirm that the VLA runtime is outside the floor |
| WP1-14 | WGS84 canonical, ENU/NED computed lazily; the DSL never carries projected coordinates | all 32 shipped policies stored and hashed in WGS84 since 3 Nov; since 18-22 Nov every flight loader, the six legacy AirSim scripts included, refuses a policy stored in local metres (bundle.load_for_flight); metre input is still parsed by models.load_policy (the legacy read form) and accepted by the REST hot-apply, ScenarioEvent and gRPC bodies, converted to WGS84 at ingest | deviation 1, decided by the lab: convert metre input at ingest, or record it; the PI's acceptance pending |
| ARCH-15 | the HIL bridge carries "AirSim sensor messages (`HIL_GPS`, `HIL_SENSOR`)" into AP SITL (G1 condition 1) | Project AirSim's JSON sensor frames to ArduPilot's SIM_AirSim backend; G1's four conditions passed on it (section 7.1) | recorded deviation: the transport, not the bridge's function, and the gate's swap rule not applied |
| WP3-04, ARCH-04 | body-frame action; one body-to-NED boundary in the adapter; repair operators act in the frame the VLA emits in | the Shield takes the body frame at its boundary since 4 Nov and every flying rail hands it the body action, NED only after the projection (ARCH-04 DONE 20 Nov); since 783e626 (21 Nov, flown in smFIN2 and ntFIN4) the registry repair operators are handed and hand back the body action at the operator interface; the last-resort ClearanceEscape and GeofenceEscape still compute in the world frame inside the Shield's stack-level rotation | WP3-04 PARTIAL: the last resorts through the body-frame interface, or the PI's written acceptance |
| WP3-06 | repair operator order with PathRepair; "first operator that produces a feasible action with magnitude <= threshold wins" | PathRepair in the default chain since 17 Nov, in the body frame since 783e626, acting KPI-grade on `hil` round stand-off rings (ntFIN3, ntFIN4: 3 cells, P0 escape 0); boundary and zone detours opt-in and unflown; off on the follow rails; the default selection is the iterated chain, the grant's first-feasible mode opt-in and unmeasured | recorded deviation (the lab's); the PI's written acceptance pending |
| WP3-07 (Loiter) | FSM drives SET_MODE LOITER | by default the FSM's Loiter is held in GUIDED on the plan's point by the companion since 7 Nov (guardrail/hold_flight.py); before that LOITER was sent as BRAKE, whose hand-over carried the aircraft off its point (CHANGELOG 2026-11-07 (Loiter held in GUIDED)); since 22 Nov a literal SET_MODE LOITER is opt-in (`--rail-hold loiter`), flown on 2 shakedown missions with a SITL-only throttle stand-in (sk1122b) | decision 7 (the lab's): PI confirmation pending |
| WP3-07 (X8) | not in the grant's edges | extension edge X8 (Normal / Brake / Loiter → RTL when the position can no longer certify a hard P0 zone's margin); since 12 Nov the ROS 2 node takes it from ArduPilot's EKF_STATUS_REPORT (`estimator_ok`); a rail that reports nothing flies as before (docs/DESIGN-escalation-fsm.md, updates 2026-11-06 and 2026-11-12) | PI confirmation |
| WP3-07 (X9) | not in the grant's edges | extension edge X9 (Loiter → RTL when the hold is not kept: the aircraft beyond the hold's reach and still receding); answers sm1111_048's missed fail-safe (CHANGELOG 2026-11-11 (X9)) | PI confirmation |
| WP3-15 labels | "triggered when expected, not when not expected" | the adopted reading (r2): rubric labels on the rail vehicle, E5 on the prescribed flight (docs/RUBRIC-failsafe.md sections 7-8, adopted 10 Nov) | reported beside (a) and (r) |
| WP3-22, X-13 | exactly one rule-evaluation code path | one rule clock and one set of point verdicts in guardrail/shield.py since 3 Nov; since 18 Nov FenceGuard measures with the Shield's field and judges with its clearance_breached ("grid" refused), flown 19-20 Nov; the remaining measures are diagnostics, verification oracles, a load and authoring lint (start_conflicts) and the autopilot's GeoFence backstop | WP3-22 DONE (23 Nov); X-13 PARTIAL: the backstop judges in flight (18 of ntFIN4's 648 scored fail-safes) and the lint is its own point-in-zone test; a written acceptance of the lab's sorting, or folding |
| WP4-13 | exporters: rosbag2 to JSONL / Parquet | the exporter reads a run's rosbag2 when it exists; other rails fall back to the flight log, labelled per step | `rosbags` not in vla-real |
| WP1-07 | distance_envelope to people, buildings, roads | enforced for buildings and roads (flown on hil, de_hil2, 0 breaches) and, since 19 Nov, for every person the rail reports | people only as perceived: on pas_hil_people_02 the person feed matched 0.189 of pedestrians within 25 m and the aircraft was within 4 m of one on 75 of 178 ticks; a person detector with measured recall, or a limit the PI accepts |
| X-05, ARCH-25 | observations reach the VLA at ≥ 10 Hz | met since 15 Nov on the hil rig for AIoT-VLA (12.4-13.2 frames/s while it is asked; the feed between ticks, a 15 Hz camera); detector 2-9 Hz by host; a 7B VLA's decision 0.5-1.2 s on the desktop GPU | none needed for AIoT-VLA; a 7B backend would still need the G3 ruling |
| ARCH-08 | AP_DDS migration (Q3, optional) | opt-in AP_DDS output path and runbook, not yet run on SITL; MAVROS 2 is the bridge | optional |

Closed deviations (no action left): WP1-01 (IR 4.0.0, 6 Nov), WP3-11 (audit
record version 4, 2 Nov), WP2-16 (the bundle ships the multi-scale
geometry, 2 Nov), WP1-22 and X-09 (IR bump on every shape change, 2 Nov).

### 7.1 Gate G1 and the HIL bridge (ARCH-15)

The grant's gate G1 (04-risks-and-fallbacks/fallback-gates.md) reads: "IF
the Project AirSim ↔ ArduPilot HIL bridge passes a closed-loop integration
test by 2026-06-30 THEN keep the perception rail on Project AirSim ELSE swap
the perception rail to Colosseum, freeze Project AirSim work, document the
swap in the final report". The test has four conditions; the first names
"AirSim sensor messages (`HIL_GPS`, `HIL_SENSOR`)" reaching AP SITL.

The delivered bridge carries Project AirSim's ArduCopterApi JSON sensor
frames over UDP 9003 to ArduPilot's SIM_AirSim backend and returns PWM over
UDP 9002 (demo/pas_ardupilot/, scripts/run_pas_ardupilot.ps1). The G1 check,
demo/pas_ardupilot/g1_check.py, flew its three runs and passed all four
conditions (demo/out/g1_20261008_223259/g1_verdict.md, and again
g1_20261013_203404). The perception rail is kept on Project AirSim; the
Colosseum swap was not made. The PI's reference implementation records the
perception rail as "deferred per the 2026-06-30 fallback gate" (sim/README.md
at 9e039b0, docs/data/midterm_gate_evidence/evidence.md). ARCH-15 is recorded
as built differently (DRIFT) on two counts: the transport differs from the
grant's words (the bridge's function is met, its message set is not the one
the gate names), and the gate's swap rule was not applied.

## 8. Findings and corrections

- The defects the project found in its own claims, each with what was
  retracted: docs/FINDING-five-defects-found-by-audit.md,
  docs/FINDING-the-kpi-was-never-measured.md,
  docs/FINDING-the-kpi-report-that-scored-every-rule-as-p0.md, and the
  `#### Retracted` entries of CHANGELOG.md.
- The WP2 and WP4 correction notes (WP2-12, WP4-21) were approved by the PI
  on 6 Oct and sent to ITRI (confirmed 2 Nov).
- 29 Oct: four independent reviewers held the 64 items then marked DONE
  against the grant's wording; 27 did not meet it and were re-stated as
  PARTIAL or DRIFT (docs/FINAL-REPORT-outline.md, header).
- 12-13 Nov: the rig's own failures. The tick scheduler locked into
  deferral on an exact pilot (hs07; fixed in 116214f, CHANGELOG 2026-11-12
  (hs07)); the start-up backlog deferred row 2 (fixed in 5f5009c, section
  5.6); the desktop ran out of commit memory and lost 8 missions (re-flown
  as ntFIN2s).
- 15 Nov, from the first REST events on `hil`: MAVROS 2 answers a GeoFence
  push of zero items with failure although ArduPilot takes it (measured on
  the AGX: answered False in 53.9 ms, the list then read back empty; MAVROS 2
  serves no fence-clear service), so every clear after an expired zone was
  recorded as a failed upload and the node stopped re-uploading; a clear is
  now judged by its read-back (5b3db86). The perception loop held the
  hot-apply lock through its whole tick (p50 29.5 ms, p95 56.5 ms on the
  Orin, the Shield's part about 4 ms), so one spawn waited 53.0 ms against
  the 50 ms target; the lock now covers the Shield's decision and its
  records only (00264ef), and the next 10 events took 0.81-6.35 ms.
- 20-21 Nov, the final smoke smfin1120 at 71aabb3 (`hil`, smoke profile
  with `--hil-controls`, 6,059.6 s; 64 episodes planned, 49 missions flown,
  15 refused before take-off, all 49 KPI-grade; docs/data/kpi_smfin1120/),
  the regression check against the release smoke rel060 (v0.6.0 = a85dc91,
  docs/data/kpi_rel060/). P0 escape 0 (0 on 2,201 P0-violation ticks, 48
  Shield-on flights; control 0.65); `tick_ms` max 78.84 ms and `e2e_ms` max
  78.93 ms, 0 of 15,389 ticks over 100 ms; fail-safe (r2) 46/47 = 0.979,
  Wilson 95 % low 0.889 (rel060: 48/48), with 1 run censored in Loiter
  (corridor-swap-shifted, as in ntFIN2). On 47 scored runs one error is
  0.979, so a smoke cannot show 0.99 with any error; the grant's figure is
  read on the nightly (ntFIN4, section 5.3).
  - **The false trigger:** smfin1120_021 dyn-nfz-rotate-spin@3 (labelled no
    fail-safe). In both flights the FSM entered Loiter on G3. In rel060 the
    Loiter began at x -188.6 and the hold-point search found a point 1 m
    back, legal for the whole 30 s a Loiter can last; the repair fell under
    theta at 7.9 s and G8 handed back at 10.0 s. In smfin1120 the Loiter
    began at x -188.1, 0.5 m nearer the spinning zone; the search found no
    point legal for 30 s within its 600-unit budget and held in place. At
    25.3 s the rotating zone made the held point illegal within the 5 s
    lookahead, the moved point was reached at 27.6 s, the repair was under
    theta only from 27.5 s, G8 never came, and the Loiter's 30 s operator
    timeout took G5 to RTL at 34.3 s.
  - **Not a code regression.** rel060's rotate-spin and
    corridor-swap-shifted runs, replayed open loop at 71aabb3 on their own
    logged inputs (tools/replay_at_revision.py on copies of the runs, the
    run folders untouched), give rel060's decisions: corridor-swap-shifted
    identical, rotate-spin different only in the last float digits (at most
    4.0e-14 on the repair fields, 1.1e-14 on the emitted action). The two
    flights differ in where the airframe was when the Loiter began.
  - **What it shows:** the hold-point search is budget-bound. A Loiter that
    begins a little closer to a zone moving toward it can be held in place,
    and then the moving zone, not the pilot, ends the mission in RTL. Fixed
    after the flight by the retreat ray (guardrail/holdpoint.py: 300 more
    search units along the ray away from the hard zones once the budget is
    spent; CHANGELOG 2026-11-22 (code)). The same cell flew in ntFIN3 with
    no false trigger on any seed. The ray is in 59e19a5: in the shakedown
    sk1122a the nine rotate-spin flights had no fail-safe, each search
    finding its point inside the budget, so the ray was not needed there
    (its case is rebuilt in tests/test_hold_retreat.py); smFIN2 flew
    dyn-nfz-rotate-spin@3 without a fail-safe, and ntFIN4 has no (r2)
    error on any cell.
- 22-23 Nov, the one tick over 100 ms in ntFIN4 at 59e19a5
  (docs/data/kpi_ntFIN4/README.md, "The one tick over 100 ms"): on
  ntFIN4_236 dyn-nfz-spawn-ahead, row 50 (t 5.03 s), `tick_ms` 105.495 and
  `e2e_ms` 105.599 ms, with the Shield's own step 10.581 ms. It was the
  tick that hot-applied the scenario's zone: the node applied the event,
  wrote the new policy generation (policy_g1.json; the CSP compile with its
  first, cold template load, csp_g1.json) and published the fences, all
  before the setpoint, and that block took 87 ms against a usual 20-40 ms,
  an I/O or scheduling stall on the Orin (no GC, no REST apply, no fsync).
  - **What it shows:** the hot-apply work sits in the tick's path to the
    setpoint. The code path is older than 71aabb3; ntFIN3's 0 of 174,186
    ticks over 100 ms did not exclude it, and the budget on every tick is
    not shown at the delivered revision (WP3-13 back to PARTIAL).
  - **The fix** (after the flight, not in this campaign and not committed
    at this report's revision): the generation write, the CSP compile and
    the fence publication move after the setpoint, the templates are
    compiled at bring-up, and each row carries per-phase tick stamps
    (docs/data/kpi_ntFIN4/README.md). It needs a shakedown and a KPI-grade
    nightly with hot-applied zones before it counts (section 11).

## 9. Decisions for the PI

Settled on 2 Nov under the PI's instruction of 6 Oct (follow the grant; best
practice where it is silent): the theta basis (forecast depth in metres),
the fail-safe rubric, the IR bump rule, the two-hash contract, multi-scale
geometry in the bundle, the correction notes, the
`hil` fallback date (docs/FINAL-REPORT-outline.md section 9).

Settled on 15 Nov by the lab, under the same instruction (follow the grant;
best practice where it is silent). None of these is the PI's signature: each
is what the project now does, and the PI's confirmation is still to be
recorded. Where the grant's words are not met the Appendix A status stays as
it was. Decisions 4 and 5 were settled on 17 Nov (below).

| # | Decision | Items | Settled as | What was done |
|---|---|---|---|---|
| 1 | Bundle signer and CA | WP1-04, X-10, X-01 | the lab's Ed25519 keys sign this delivery; the CA the grant names (a Lab CA or ITRI CA reference) is added to policies/keys/trusted_signers.json when it is named | nothing re-signed; the registry is data, so the swap is one entry and a re-sign (guardrail/bundle.py); on 22 Nov the user decided to wait for the PI to name the CA (below); the items stay PARTIAL until an authority is named |
| 2 | `flight` stage | ARCH-14, ARCH-31 | the work stops at `hil` | no flight evidence exists or is claimed; the written waiver is the PI's to give; on 22 Nov the user decided not to flash the real flight controller (below), so SITL-to-firmware bit-identity (ARCH-31) goes to the PI |
| 3 | `dev` KPI evidence | section 5 | no waiver is asked for | every KPI in section 5 comes from KPI-grade `hil` runs |
| 6 | FSM extensions X8 and X9 | WP3-07 | kept: both close a missed fail-safe (estimator health; a hold that is not kept) | documented as extensions to the grant's ladder (docs/RUBRIC-failsafe.md) |
| 7 | Loiter held in GUIDED | WP3-07, WP3-09 | kept as the default: ArduCopter's LOITER sinks with no pilot on the throttle (flown 7-8 Oct: 1.4-1.8 m/s, below the policy floor), so the hold is the companion's, at the Shield's point; `--hold-mode LOITER` stays for a flight with a safety pilot | since 22 Nov a literal SET_MODE LOITER is flown opt-in on `hil` (`--rail-hold loiter`, 29a1440): sk1122b at 59e19a5, 2 missions, LOITER held 29.9 s within 0.021 m, the throttle centred by a SITL-only stand-in (section 4.3); the default is unchanged, so WP3-07 stays PARTIAL until the PI confirms this decision |
| 8 | Python 3.10 for the VLA runtime | WP1-25, WP2-19, X-06 | kept: the grant's 3.11+ binds the non-VLA stack, which runs 3.12 since 14 Nov | section 7 |
| 9 | Push to GitHub | WP3-19, WP4-06, WP4-14 | master pushed to the fork on 22 Nov at the user's instruction (it was 214 commits behind; the last green run there was on 13 Oct); GitHub Actions runs `tests` on every push (.github/workflows/tests.yml), the headless smoke its last step; CI also runs locally per merge (tools/ci_local.py, records in docs/data/ci_local; 8e73f86: 187/187 files, smoke rc 0) | the first GitHub runs were red: on 59e19a5 the job's 30-minute timeout (raised to 90 minutes, 22ddb61), on 0daa20e Linux-only test issues, fixed in the tests and the CI config (2b01e77, 132e979) and not yet pushed; the smoke step was skipped on both; the items stay PARTIAL |
| 10 | Observation rate below 10 Hz | X-05, ARCH-25 | met by engineering, not by a ruling: a faster camera alone did not reach 10 Hz, a feed between the loop's ticks did | pas_hil_obs_02 (15 Nov, KPI-grade, capture interval 0.05 s instead of 0.066 s): the camera delivered 14.7 Hz (12.8 before) and frames reached the VLA at 9.58 Hz while it was asked (9.10-9.50 before); the bound was the pilot's asking cadence, not the camera. That cadence was then changed (the feed between ticks, the same day): 12.4-13.2 frames/s while it is asked on four flights (pas_hil_feed_05-08); X-05 and ARCH-25 to DONE |
| 11 | The five final-demo scenarios | WP4-19 | the five below, all flown on `hil` | section 9.1 |
| 12 | Default VLA backend | ARCH-30, X-04 | AIoT-VLA v10b for the follow task: the only learned model that has flown KPI-grade on the AGX Orin; the stub stays the KPI campaigns' pilot | named by the lab on 18 Nov on the generated table, after the set (section 6.4); ARCH-30 and X-04 DONE |
| 13 | G4 on a flickering brake rule | WP3-07, WP3-15 | the grant's G4 governs: rubric clause E6, "N-in-T persists" (onsets made by the world at the N-in-T rate through a Loiter); N1 and N2 are unchanged elsewhere | fsm-n-in-t-flicker labelled expected (E6), the only label that changed over every profile; hs10_003 is a correct trigger on it; the clause was written after that flight (docs/RUBRIC-failsafe.md section 9) |
| 14 | A GeoFence RTL with the FSM holding | WP3-15 | a fail-safe triggered by the grant's backstop, scored against the label like any other | derived per run (experiments/sweep_scenarios.py failsafe_backstop); ntFIN2 + ntFIN2s re-scored: 584/585 = 0.998, Wilson 95 % low 0.990, 18 backstop runs, all expected; the earlier readings stay printed beside it (docs/data/kpi_ntFIN2_20261115/); WP3-15 to DONE. ntFIN3 (71aabb3), scored with the rule in place: 587/589 = 0.997, Wilson 95 % low 0.988. ntFIN4 (59e19a5): 648/648 = 1.000, Wilson 95 % low 0.994, 18 backstop runs, all expected; 630/630 without the rule (section 5.3) |
| 15 | Body-frame boundary | WP3-04, ARCH-04 | accepted where the grant's path runs (ROS 2 / `hil`: Shield.filter_body, NED only after the projection in the MAVLink adapter); every flying rail hands the Shield the body action | ARCH-04 DONE (20 Nov). Since 783e626 (21 Nov, flown in smFIN2 and ntFIN4) the registry repair operators act in the body frame at the operator interface; the last-resort ClearanceEscape and GeofenceEscape still compute in the world frame inside the Shield's own rotation, so WP3-04 stays PARTIAL |

Settled on 17 Nov by the project. These rows are reported on what was
delivered and where it is; Appendix A moves them on that basis.

| # | Decision | Items | Settled as | What was done |
|---|---|---|---|---|
| 4 | Dated deliverables | WP1-21, WP3-01, ARCH-15, ARCH-16, ARCH-17, ARCH-23 | reported on what was delivered and where it is | WP1-21, WP3-01, ARCH-16, ARCH-17 and ARCH-23 to DONE; ARCH-15 stays DRIFT on its transport and the gate's swap rule (section 7.1) |
| 5 | Team | ARCH-24 | stated as in the grant: 1 PI + 4 Master students, one team | ARCH-24 to DONE |

Decided on 22 Nov by the user, for the lab. These are the lab's decisions,
not the PI's; each is recorded here and in the rows it touches.

| Decision | Items | Decided | What was done |
|---|---|---|---|
| Wait for the PI to name the CA | WP1-04, X-10, X-01 | no lab key is certified and nothing is re-signed until the PI names the signing authority (the grant leaves Lab CA or ITRI CA open) | the CA path is built and tested (16 Nov: --issue-cert, a swap drill to a second CA, forged / wrong-CA leaves refused); the flown bundles stay signed by lab-dev keys, chain none |
| Do not flash the flight controller | ARCH-14, ARCH-31 | the real FC (CUAV V5 Nano) stays on ArduCopter 4.7.1, not flashed to the rig's 4.5.7 | the 17 Nov bench (fence and six modes on the real FC, restored) stands; SITL-to-firmware bit-identity (ARCH-31) is put to the PI |
| BitVLA downloaded and its loader run | ARCH-06 | download BitVLA's LIBERO-Spatial checkpoint (hongyuw/ft-bitvla-bitsiglipL-224px-libero_spatial-bf16, MIT) and run its real loader on the AGX Orin | loaded in 5.94 s, 5,585 MiB GPU, median 1.479 s per answer (n=20); every answer a finite Action4D, through Shield.filter and over TCP (12 decisions in 20 s); an arm checkpoint, so a conformance check, not a flight policy (docs/data/bitvla_loader_20261122/README.md) |
| Push master to the fork | WP3-19, WP4-06, WP4-14 | push master on 22 Nov | decision 9 above |

### 9.1 The five final-demo scenarios (decision 11)

All flown on `hil`; none is one flight with all four components, so WP4-19
stays PARTIAL (the suite's `hil` pilot reads no CSP).

| # | Scenario | Run | Shows |
|---|---|---|---|
| 1 | dyn-nfz-spawn-ahead (suite) | ntFIN2, hh02_001 | a zone hot-applied over REST in flight; the Shield repairs or holds; the GeoFence backstop |
| 2 | failsafe-rtl-blocked (suite) | hs10_004, demoA_001 | the ladder to its end: G11, G6 "RTL failed: blocked path", Land |
| 3 | fsm-n-in-t-flicker (suite) | hs10_003 | N-in-T: G2 then G4, home (decision 13) |
| 4 | the follow with a learned VLA and REST zones | pas_hil_aiot_est_01, pas_hil_hotapply_02 | AIoT-VLA and the Shield on the AGX Orin; 10 zones in flight, 0 P0 escape ticks |
| 5 | perception stress | pas_hil_aiot_texture_01, pas_hil_aiot_shadow_01 | the detector losing the car under a novel road texture; the Shield's record unchanged |

Only the PI or ITRI can give: the written waiver for the `flight` stage
(2), the name of the signing CA (1), a ruling on SITL-to-firmware
bit-identity without a flashed FC (ARCH-31), and the confirmation of the
rows above.

## 10. Limitations and open items

- **No learned VLA in the KPI campaigns.** The `hil` campaigns the KPI
  report is built from fly stub pilots. A learned model over MAVROS 2 in one run (ARCH-18, ARCH-20)
  was flown on 14 Nov (docs/data/pas_aiot_hil_20261114/flights.md); the learned VLA's follow on the hil rail is weak (target lock 0.06 on the KPI-grade mission).
- **The tick budget is not met on every tick at the delivered revision**
  (WP3-13, PARTIAL): 1 of ntFIN4's 183,909 ticks took 105.5 ms, a
  hot-apply tick whose generation write and CSP compile ran before the
  setpoint (section 8); the fix is not committed or flown. Even within the
  budget the figures rest on stub-pilot runs and on the Shield node alone
  (`e2e_ms` has neither the pilot's DDS hop nor the adapter's hop to
  MAVROS); no learned VLA's decision time is in it (ARCH-03); ~50 rules
  only in the offline Orin profile (last re-run 14 Nov at bc04e4a).
- **SET_MODE LOITER is not the default** (WP3-07, PARTIAL): it flew on
  `hil` only opt-in (sk1122b, 2 missions, a SITL-only throttle stand-in,
  no G8 or G4 in that mode); the default holds the FSM's Loiter in GUIDED
  from the companion (decision 7, PI confirmation pending); a mission
  profile's FSM YAML cannot be selected on the rails.
- **G6's battery reason not flown** (WP3-07): SITL reaches a CRITICAL
  battery only through BATT_CRT_* thresholds the rig leaves at 0, so
  failsafe-low-battery stays refused; the reason is unit-tested.
- **Fail-safe correctness rests on the adopted reading** (WP3-15): 648/648
  = 1.000 on ntFIN4, Wilson 95 % lower bound 0.994, under (r2); (r) gives
  0.981 and (a) 0.884 on the same runs. The reading (r2) (adopted 10 Nov)
  and the GeoFence backstop rule (decision 14) are the lab's and not yet
  confirmed by the PI; the figure holds without the backstop rule
  (630/630). The nightly before, ntFIN3, did not clear the bound (0.988).
- **Seeds are not independent** (section 5.7).
- **ArduPilot's own modes** carried the aircraft into a P0 polygon in 22
  ntFIN4 missions and through a keep-out margin in 14 more, outside the
  grant's escape count; the 12 m/s RTL drift (lanes 15 and 20) is open
  (section 5.2).
- **Repairs outside the body frame** (WP3-04): the last-resort
  ClearanceEscape and GeofenceEscape compute in the world frame.
- **One rule-evaluation path, but other judges in flight** (X-13): the
  autopilot's GeoFence backstop triggered 18 of ntFIN4's 648 scored
  fail-safes, and models.start_conflicts is its own point-in-zone lint.
- **MSL only on the ROS 2 / SITL-hil rail** (X-11): the PAS, follow,
  vla_fly and Gazebo readers refuse a ground; no committed MSL test on
  circle_fence or corridor_swap; the hil zone-ridge cells not re-flown with
  the terrain feed.
- **No `flight` topology**, no flight firmware bit-identity (ARCH-14,
  ARCH-31); the real FC is not flashed to the rig's ArduCopter (the user's
  decision, 22 Nov).
- **One Gazebo functional sweep**, one seed, 7 of 8 missions passed
  (20 Nov; WP4-04); the 123-mission gazebo_functional profile has had a
  dry run only.
- **No green CI smoke per build, no release with the five-row smoke**
  (WP3-19, WP4-06, WP4-14): master was pushed on 22 Nov and both GitHub
  runs since were red before the smoke step (decision 9); the five-row
  smoke flew at the unreleased 59e19a5.
- **Perception stress** is one run per stressor, non-KPI by design
  (WP4-16, WP4-18). The novel road texture (15 Nov) broke the follow: the
  detector found the car on 0.095 of frames against 0.865 unstressed, and
  the mission ended target_lost after about 100 s (0 P0 escape ticks); the
  low sun (shadow) cost little (0.820). Flown 14 Nov: at -4 EV (night) the detector never found the red car in 330 s and the mission refused to start (LAND), and so it did in fog (FOG 0.6, pas_hil_aiot_fog_01); at -2 EV (dusk) it found it after 170 s and lost it; mean frame luma 97.9 day, 43.0 dusk, 13.5 night (docs/data/pas_aiot_hil_20261114/perception_stress_report.md).
- **Observation rate**: 10 Hz or faster only for the lab's own VLA, with a 15 Hz camera, and while the detector has the target (X-05; the detector itself runs at 2.3-2.7 Hz on the AGX Orin).
- **Python 3.10** for the VLA runtime on the desktop (vla-real); the
  non-VLA side runs 3.12 since 14 Nov, one PAS scene step on 3.12 still to
  run (WP1-25).
- **AP_DDS** prepared, not run on SITL (ARCH-08, PARTIAL).
- **BitVLA and CognitiveDrone** (ARCH-06): BitVLA loads and conforms on the
  AGX Orin from an arm checkpoint, unflown and at 0.6-0.7 Hz; CognitiveDrone
  has no published weights.

- **The learned VLA's follow on the `hil` rail is weak** (target lock
  0.04-0.82; docs/data/pas_aiot_hil_20261114/perception_stress_report.md).
  The PAS perception node does not yet carry the city rail's target
  estimator, strict lock and re-acquisition (`follow_vlm.fly()`;
  docs/DESIGN-projectairsim-ardupilot.md, "Not reused yet"), so the VLA
  stands still when the detector loses the car; `--identity` alone, without
  the estimator, lost it sooner (pas_hil_aiot_kpi_02, 107 s). Porting the
  estimator is the next step; the Shield's own figures on these flights
  (P0 0, every intervention logged) do not depend on it.

## 11. Plan to 2026-11-30

Done 13-14 Nov: ge1113, the grant example sweep at 2f22116 (section 4.4,
5.6); the merged KPI report committed from a clean tree with its re-score
tables (WP4-17, section 2); G2, G4, G6, X2 and X3 flown on `hil` in hs10
(WP3-07, still PARTIAL on the GUIDED hold); the full CI on Python 3.12
(WP1-25).

Done 14-15 Nov: AIoT-VLA on the AGX Orin in the `hil` topology with video
and camera-to-setpoint latency, the PAS scene steps on Python 3.12 (WP1-25),
the target estimator flown (pas_hil_aiot_est_01); REST hot-apply flown on
`hil` and the in-flight mission update written whole in AUTO (WP1-15,
WP3-09 to DONE).

Done 20-21 Nov, at 71aabb3 on the `hil` rig: the final smoke smfin1120 (49
KPI-grade missions; section 8) and the KPI-grade nightly ntFIN3 (594
KPI-grade missions), the figure of record until 23 Nov
(docs/data/kpi_ntFIN3/, docs/data/kpi_smfin1120/).

Done 22-23 Nov, at 59e19a5 on the `hil` rig: the eleven code changes of
CHANGELOG 2026-11-22 (code) shaken down (sk1122a, 72 missions; sk1122b,
the literal LOITER; the MSL ridge on SITL; docs/data/shakedown_20261122/),
then the final smoke smFIN2 (52 KPI-grade missions, the five-row matrix
whole) and the KPI-grade nightly ntFIN4 (653 KPI-grade missions), the
figure of record of this report (sections 2 and 5; docs/data/kpi_ntFIN4/,
docs/data/kpi_smFIN2/); the 23 Nov recount of the open Appendix A rows
(WP3-22 to DONE, WP3-13 back to PARTIAL).

Remaining, in order:

1. The tick fix: commit it (the hot-apply's generation write, CSP compile
   and fence publication after the setpoint; templates compiled at
   bring-up; per-phase tick stamps), shake it down on `hil`, then fly a
   KPI-grade nightly ntFIN5 at that revision. If ntFIN5 is flown and scored
   with no tick over 100 ms in time, it becomes the figure of record and
   WP3-13 can return to DONE; if not, the delivered revision stays 59e19a5
   with ntFIN4 and its one 105.5 ms tick as stated here.
2. A tagged release at the flown revision with its own smoke flying the
   five rows (WP3-19, WP4-14), and the CI fixes pushed so the GitHub smoke
   completes per build (WP4-06).
3. WP3-04: route the last-resort ClearanceEscape and GeofenceEscape
   through the body-frame operator interface, with tests and a flight, or
   put the stack-level rotation for them to the PI.
4. X-11: committed MSL tests for circle_fence and corridor_swap, the
   remaining readers taking a declared ground (PAS, follow, vla_fly,
   Gazebo), and the hil zone-ridge cells re-flown with the terrain feed.
5. X-10: the flight-bundle builder passes the previous version, so every
   delivered bundle carries a changelog diff (tools/build_flight_bundles.py).
6. The Gazebo functional sweep over its 3 seeds (the 123-mission
   gazebo_functional profile; WP4-04).
7. AP_DDS: run the runbook's steps 4-6 on the DDS SITL (ARCH-08), or record
   that this optional row closes at PARTIAL.
8. The PI's decisions (section 9: the CA, the `flight` waiver, the
   firmware bit-identity, and confirmation of decisions 1-15 and of the
   user's 22 Nov decisions); then sign (X-01).

## Appendix A — obligations, evidence and what remains

Copied from docs/FINAL-REPORT-outline.md (re-assessed 23 Nov: 109 DONE, 26 PARTIAL, 1 DRIFT, 0 MISSING). The figure of record since 23 Nov is the nightly ntFIN4 at 59e19a5 (sections 2 and 5); many rows below still quote earlier campaigns' KPI figures (ntFIN2 + ntFIN2s at 116214f, ntFIN3 at 71aabb3), each named. Since 23 Nov: an independent recount of the open rows, with an adversarial check of every proposed status change, moved WP3-22 to DONE (FenceGuard measures with the Shield's field and judges with its clearance_breached, "grid" refused, flown 19-20 Nov; what remains outside the checker judges no rule in the flown Shield's place) and WP3-13 back to PARTIAL (ntFIN4 at 59e19a5: 1 of 183,909 ticks at 105.5 ms, on a code path older than 71aabb3, the fix not committed); proposed moves of WP3-04, WP3-07, WP3-19, X-11 and X-13 to DONE were refuted and stay PARTIAL (WP3-04: the last-resort escapes still world-frame; WP3-07: the default is not SET_MODE LOITER, decision 7 the lab's; WP3-19: no release has run the five rows; X-11: no MSL enforcement test for circle_fence or corridor_swap and readers off the ROS 2 rail refuse a ground; X-13: the GeoFence backstop and start_conflicts judge outside the checker); the evidence and open cells of 29 rows were refreshed, and WP3-15 holds on ntFIN4 at 648/648, Wilson 95 % low 0.994. Since 20 Nov: WP4-15 and ARCH-04 moved to DONE after an independent review of each row against the grant's wording (WP4-15 met by pab3's same-seed prefix on/off pairs in the harness and the KPI report's A/B section; ARCH-04: the body-frame contract on every flying rail, conversion to local NED only in the adapters); WP4-04 and WP3-04 stay PARTIAL (CHANGELOG 2026-11-20, flown). Since 18 Nov: ARCH-30 and X-04 moved to DONE (the lab named AIoT-VLA v10b the default backend for the follow task after the Q2 measurement set, deploy/evidence/vla_backend_table.md). Since 17 Nov: ARCH-03 moved to DONE (gate G3 judged on the AGX Orin in flight: AIoT-VLA v10b round trip p95 84.0-84.7 ms and 13.15-13.22 decisions/s while asked, the measured 7B backends fail, docs/data/g3_gate_20261117) and ARCH-08 to PARTIAL (an opt-in AP_DDS output path, not yet run on SITL); WP1-21, WP3-01, ARCH-16, ARCH-17 and ARCH-23 moved to DONE, reported on what was delivered and where it is, and ARCH-24 moved to DONE with the team stated as in the grant (decisions 4 and 5); ARCH-15 stays DRIFT on its transport (section 7.1). Since 16 Nov: ARCH-02 moved to DONE (every rail of the grant flies ArduPilot as the inner loop, the perception rail with a learned VLA on hil; the simple_flight comparisons are a bench outside the delivery). Since 15 Nov: X-05, ARCH-25, ARCH-01 and ARCH-21 moved to DONE (the feed between ticks: 12.4-13.2 camera frames/s reach AIoT-VLA while it is asked and it decides 10.6-10.9 times a second over two full missions, pas_hil_feed_05-08); WP3-15 moved to DONE (the backstop rule, decision 14: 584/585); ARCH-10 moved to DONE (Mission Planner recorded on a hil flight beside MAVROS 2); WP4-18 moved to DONE (the texture and shadow stressors flown on the hil rig, pas_hil_aiot_texture_01 and pas_hil_aiot_shadow_01); WP1-15 moved to DONE (REST events flown into the hil Shield on the AGX Orin, pas_hil_hotapply_01/02, pas_hil_adapter_01) and WP3-09 to DONE (the in-flight mission update written whole in AUTO, mission_wp_20261115b; the perception rail's adapter route flown on hil). Since 13 Nov: WP3-13 moved to DONE (no tick over 100 ms end to end at the fixed node code: ge1113's 95,160 unloaded, hs09 and hs10 loaded); WP3-07 stays PARTIAL although every FSM edge flew on hil in hs10 (SET_MODE LOITER is driven on no hil flight, lab decision 7, PI confirmation pending); WP4-17's report is committed from a clean tree and only its delivery remains; WP1-25, WP2-19 and X-06 moved to DONE once the PAS scene steps ran on Python 3.12 for the AIoT-VLA hil missions (14 Nov).

### WP1 - Policy DSL and IR

25 items: 21 DONE, 4 PARTIAL.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP1-01 | YAML/JSON DSL surface with a Pydantic v2 discriminated-union schema | DONE | guardrail/models.py grant form; tests/test_policy_dsl_grant_form.py; since IR schema 3.0.0 (2 Nov) constraint_type, priority and violation_action are required and the action is one of the grant's six (CHANGELOG 2026-11-02, Policy DSL); since IR schema 4.0.0 (6 Nov) `scope` is required too, so all four ConstraintBase fields are, as the grant writes them: refused when missing or null at every DSL entry (test_every_rule_must_state_its_scope; policies/negative/missing_scope.yaml), `scope: mission` on all 154 shipped rules (32 files, each re-versioned, lock `migrated_from` with old_hash_form sha256-canonical-v3/ir-3.x), stored documents of earlier eras read in their era with their recorded hashes (test_a_document_from_before_ir_4_reads_and_hashes_as_stored; docs/IR-MIGRATIONS.md; CHANGELOG 2026-11-06 (IR 4.0.0)) | None. Scope is declared: the Shield enforces every rule over the whole flight, so a narrower scope is enforced more widely than written, never less. |
| WP1-02 | Validation behaviour and rule-consistency lint (normalise geometry, lint alt_min<alt_max, conflicting hard rules, valid_time well-formed) | DONE | CHANGELOG 2026-10-07 Changed (negative corpus 35/35 refused); guardrail/models.py lint and conflicting-hard-rule check; since 2 Nov rings are stored in canonical order (canonical_ring_order: positive winding, smallest point first; the hash no longer depends on vertex order) and the lint refuses a hard corridor no flight can be inside and two disjoint hard corridors (policies/negative/corridor_inside_nfz.yaml, disjoint_hard_corridors.yaml) | - (holes: the grant's schema has none; the GeoJSON importer refuses them) |
| WP1-03 | IR / AST: a typed, indexed in-memory form with spatial index and multi-scale geometry, consumed by Compiler and Shield | DONE | guardrail/ir.py STRtree + coarse/fine geometry; tests/test_ir.py; CHANGELOG 2026-10-06/-07/-13 (check 0.30-0.45 ms at 50 rules, desktop) | Orin-side timing is tracked under X-07. |
| WP1-04 | Signed policy bundle (hash + semver) as the deployable artefact | PARTIAL | guardrail/bundle.py Ed25519, 64-hex SHA-256 policy_hash, semver `version`, tar.gz of ir.json + manifest.json + signature.txt (+ multiscale.json), policies/policy.lock.json; every rail flies the signed bundle by default since 2026-10-22. Flown at the delivered revision: ntFIN4 (59e19a5, 653 KPI-grade) e.g. demo/out/hil_smoke/ntFIN4_236_dyn-nfz-spawn-ahead/metrics.json policy_source kind=bundle, bundles/fase3-sim-demo-v0.1.4.tar.gz, declared = loaded policy_hash sha256:b665adba..., signature verified (lab-dev:ed25519:715f70c4647a9701, the Orin's key), chain none, require_chain false; KPI-grade needs the signed replay bundle bound to the run (kpi_report_ntFIN4_2026-11-23.md:950). CA path built 16 Nov (CHANGELOG 2026-11-16: signed_by lab-ca:<CA fp>:ed25519:<kid>, `signer` record, signer.cert.pem, swap drill to a second CA with no code change, forged / wrong-CA leaves refused; tools/build_flight_bundles.py --issue-cert, tools/bundle_ca_evidence.py, VLAGUARD_REQUIRE_CHAIN opt-in); tests/test_bundle.py 72/72 at 8e73f86 (docs/data/ci_local/8e73f86b6af6.json, 22 Nov). | No authority signs anything: policies/keys/trusted_signers.json holds two bare lab-dev keys ("not a CA") and no `authorities` entry (unchanged since d0e91bb, 6 Nov); all 48 policy bundles in the lab checkout's bundles/ are signed by lab-dev:ed25519:6ec75b535ef5c533. The user decided on 22 Nov to wait for the PI to name the CA (no keys certified, nothing re-signed); the grant leaves Lab vs ITRI CA open, so it is the PI's call. Once named: --issue-cert, re-sign the flight bundles, VLAGUARD_REQUIRE_CHAIN=1 on the hil loader, commit a tools/bundle_ca_evidence.py record (none in docs/data yet). |
| WP1-05 | Bundle compatibility with the PI's reference implementation (a byte-identical match, claimed and retracted 7 Oct) | PARTIAL | docs/data/wp1_roundtrip.json (tools/wp1_roundtrip_kpi.py, regenerated 2026-11-06 at 4edd5d1 under IR 4.0.0, 52ac56c; unchanged since): both hashes 32/32, reference loader reads ours 3/3, reference hash agrees 3/3, reference bundles loaded 1/1, forged reference hash refused 32/32 (null_record_ignored 0/32), reference_hash_representable 0/32; restated as the WP1 section of docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md:1794-1813. guardrail/reference_convert.py (2 Nov): each manifest carries `reference_projection` (exact projections non-empty for 31 of 32, none whole), two-hash contract in docs/DESIGN-policy-dsl-grant-form.md section 7. Flown: ntFIN4_236 policy_source hashes {sha256-canonical-v3: b665adba..., reference-policy-dsl-0.1: null}; the bundle's manifest reference_projection complete=false (kinematic_envelope and the margin ring omitted, reference_problems listed). Byte-identity claim retracted (CHANGELOG 2026-10-07). | Nothing new since 6 Nov. The PI still has to accept the two-hash contract (the reference's hash over the projection the reference DSL can hold) in place of byte-identity. No flown policy can be represented whole in the reference DSL (kinematic envelopes, margin rings, schedules), so the reference loader cannot fly a bundle of any KPI policy. Residual: wp1_roundtrip.json was generated at 4edd5d1, not at the delivered revision 59e19a5. |
| WP1-06 | GeoFence classes: polygon_fence and circle_fence | DONE | circle_fence enforced (CHANGELOG 2026-10-07); sitl/fence/ export; PAS rail uploads clock-proof zones (CHANGELOG 2026-10-13, FENCE_TYPE 7); tests/test_railinputs_fence_export.py; ros2fix2_off_fence fence fired | None. |
| WP1-07 | Envelopes: altitude, kinematic, and distance (minimum distance to typed object classes) | PARTIAL | since 3 Nov distance_envelope is enforced (models.RUNTIME_TYPES, shield.ENFORCED_TYPES) through the runtime of its class (models.DistanceEnvelope.runtime_rule; shield._SOURCE_OF): buildings = the clearance rule on the obstacle map, roads = the same on the road map (Shield(road_map=...), the street mask; wired on vla_fly, follow_vlm and the PAS perception node since b385513, 17 Nov, build_street_mask.road_map_for), people = since 19 Nov (wf21-people-distance) models.PeopleClearance over every person the rail reports (the PAS node's ground-projected person boxes, the subject while classified a person, a declared person on SITL/headless); an envelope whose source is missing is refused at construction (never inert); tests/test_distance_envelope_msl.py. Buildings and roads flown on hil: de_hil1 (0f4269a) entered both envelopes' corners and failed (diagnosis_de_hil1.json: clearance horizon assumed an instant vehicle); fixed by clear_response (shield.py 17e8047, 6ef5a6d); de_hil2 (06518b7, signed bundle distance-envelope-sitl-v0.1.0, world de_block.yaml sha256 503f8889...): both lanes PASS, 0 breaches, P0 escape 0, no fail-safe, replay_from_bag 2/2 identical (docs/data/distance_envelope_20261119/sweep_de_hil2.json, replay_from_bag_de_hil2; CHANGELOG 2026-11-19 (WP1-07 ...)). People flown once on hil: pas_hil_people_02 (bfb4973, follow-car-citylife-people-v0.1.0, "keep at least 4 m horizontal from any person", P1): P0 escape 0, ended in a correct X7 RTL after 18.2 s (docs/data/people_hil_20261119/README.md). | People are not kept at the minimum distance: on pas_hil_people_02 the aircraft was within 4 m of a level pedestrian on 75 of 178 ticks with truth (true minimum 0.765 m horizontal); the person feed matched 0.189 of pedestrians within 25 m (0.32 of those in view, threshold 0.10 untuned), so the rule binds only perceived people; one 18 s flight, not KPI-grade (subject pose 166/180 ticks), rule at P1. Raising it to P0 / meeting the envelope needs a person detector with measured recall, or a limit recorded and accepted by the PI. The distance_envelope profile is not KPI-bearing (grant_profile_kpi_bearing false, 2 cells, 1 seed, lab-dev key); no distance_envelope cell is in the smoke/nightly campaigns (smFIN2, ntFIN4). No new WP1-07 evidence in the 21-22 Nov flights or code. |
| WP1-08 | 3-D corridor (keep-in tube) | DONE | corridor implemented, tests/test_corridor.py (audit 5 Oct DONE) | Never flown on any rail; optional to fly corridor_survey once. |
| WP1-09 | Time windows: valid_time recurrence on rules plus the time_window_switch event | DONE | valid_time + time_window_switch: guardrail/shield.py events, guardrail/compiler.py _RuleClock, templates; rule clock on all rails (CHANGELOG 2026-10-13); tests/test_shield_events.py, tests/test_railinputs_clock.py | No flight has declared a switch in a policy file; flight evidence belongs to WP1-15/WP4-03. |
| WP1-10 | Breach actions: violation_action values monitor_only/project_fix/brake/loiter/RTL/land | DONE | guardrail/shield.py dispatch on violation_action + guardrail/fsm.py SET_MODE; tests/test_fsm.py 114/114 (run 2026-10-28); RTL flown via X7 on pas_ap_close_on (13 Oct) and pas_ap_video_on_20261014 | Per-action flight evidence is WP3-08. |
| WP1-11 | constraint_type hard/soft | DONE | guardrail/shield.py soft rules capped at brake, relaxed lowest priority first (CHANGELOG 2026-10-07); tests/test_shield.py | PI Q9 (12 Oct) on soft semantics unanswered; no shipped policy uses a soft rule in flight. |
| WP1-12 | priority P0/P1/P2 | DONE | guardrail/shield.py relaxation by priority; escalation uses priority (CHANGELOG 2026-10-07) | Priority does not arbitrate between two hard rules (documented). |
| WP1-13 | scope + layer fields and the layered merge regulation>site>mission | DONE | guardrail/models.py merge_layers/load_layered, locked_by (CHANGELOG 2026-10-07, 2026-10-19); tests/test_policy_dsl_grant_form.py | No flown run has used a layered policy. |
| WP1-14 | Units and frames: WGS84 lat/lon canonical, AGL default, ENU/NED computed lazily | PARTIAL | since 3 Nov all policies are stored and hashed in WGS84 with a stated origin (tools/convert_policies_wgs84.py; docs/data/wgs84_conversion.json: 257 points, round trip at most 5.5e-8 m, re-versioned with lock `migrated_from`); local metres are a lazy per-origin view (IR 3.1.0, models.XY, projection.local_metres); AGL default, MSL opt-in with a declared ground (X-11, flown on SITL and hil 18-19 Nov). Writers: every writer converts at ingest for a framed policy (models.rule_in_wgs84, projection.raw_points_in_wgs84, demo/gui_policy.py) or refuses on a frameless one (NoFrameOrigin, 422 on REST since 18 Nov); gen_random_scenario --metres removed; tests/test_wgs84_ingest_writers.py, test_wgs84_writes.py, test_wgs84_frameless_writers.py. Flights fly WGS84 only: since 18-19 Nov bundle.load_for_flight (guardrail/bundle.py:2447-2532) refuses a policy stored as local-metres / wgs84+local-metres (LegacyPointFormRefused, naming the WGS84 successor), used by every rail (sitl/ros2_shield_node.py:2804, ros2_mission_node, gcs_backstop, ros2_vla_bridge_node, perception_node, vla_fly, follow_vlm); since 22 Nov also the six legacy AirSim scripts (bundle.LEGACY_YAML_RAILS, refuse_or_load_yaml; tests/test_legacy_rails_wgs84_only.py, 9 tests; CHANGELOG 2026-11-22 (code)); VLAGUARD_LEGACY_READ=1 only re-runs a recorded flight, recorded as legacy_read/point_form. tools/bundle_census.py (docs/data/bundle_census_20261118/): 13 legacy + 4 frameless bundles each mapped to a flight-ready successor; the tasks last flown from legacy bundles re-flown from v0.1.3 WGS84 bundles on the AGX 19 Nov (docs/data/onepath_agx_20261119/: 5 onepath, agx_hier_aerovla_wgs84, nfz3b_agx_hier_aerovla_wgs84, signatures verified). Every hil KPI campaign since (rel060, ntFIN3, smfin1120, smFIN2, ntFIN4) loaded through load_for_flight. | The DSL can still carry projected coordinates on input: models.load_policy parses a metre (x/y) document (the legacy read form, kept for headless tools, tests and declared re-reads), and the REST hot-apply bodies (guardrail/api.py:25-41, its own example uses {"x","y"} vertices), ScenarioEvent and the gRPC forward path accept metres, converted to WGS84 at ingest. This is deviation 1 in the register ("convert metre input at ingest, or record it"), decided by the lab, not accepted by the PI. To close: accept only lat/lon at the DSL/REST/gRPC entry (metres as an explicitly named tool-side conversion) or get the PI to accept the recorded deviation. The 22 Nov legacy-script refusal is tested, not flown (those AirSim scripts are not used by any KPI rail). |
| WP1-15 | Mid-flight hot-apply (dynamic_nfz, time_window_switch, corridor_swap) with generation bump and hash re-derive, via a REST endpoint | DONE | guardrail/api.py REST + --hotapply-port on both SITL rails with mTLS (CHANGELOG 2026-10-13); tests/test_api.py, tests/test_railinputs_hotapply.py; 'Nothing here has flown'; 29 Oct: REST hot-apply into flying ArduPilot SITL + MAVROS 2 (ROS 2 rail, dev): a dynamic_nfz spawned, rotated, translated and switched off (time_window_switch) mid-flight, generations 0-4, receipt to swap 0.444-4.443 ms against 50 ms; on ros2_hotapply_03 the Shield and the FSM stopped the aircraft short of the REST zone (X1, G3), released it after the switch-off (G8), and it reached the target; 3 Nov: corridor_swap over REST in flight - harness_corridor_20261103c (re-flown 4 Nov from the main checkout, code recorded: harness_corridor_20261104b) (corridor-survey bundle, the harness forwarding to the rail's /hot_apply): a dynamic_nfz spawned (generation 1), switched off (2), the corridor swapped for a shorter one ending at x = 40 (3) and back (4), receipt to swap 0.711-1.814 ms; the Shield stopped the aircraft inside the swapped corridor (x 29.2) until the swap back, then it reached the target, P0 escape 0.0 (CHANGELOG 2026-11-03 (G)); 15 Nov (code + tests, not flown): the perception rail serves the same listener on its own Shield (demo/pas_ardupilot/rail_hotapply.py: the ROS 2 node's five --hotapply-* flags, plaintext on loopback only and mutual TLS with the harness CA otherwise; each event a generation written as policy_g<N>.json / csp_g<N>.json, metrics.json hot_apply, dwell from the arrival, the GeoFence re-uploaded on the next tick); tests/test_pas_hotapply.py; the dynamic_nfz file hold (C-16) lifted for a zone without motion (Policy.static_keepout_zones) (CHANGELOG 2026-11-15); 15 Nov flown on hil (AGX Orin, MAVROS 2; pas_hil_hotapply_01, pas_hil_adapter_01, pas_hil_hotapply_02, all three KPI-grade): 14 dynamic_nfz events over REST (spawn and expire), each a generation with its policy_g<N>.json and csp_g<N>.json, the GeoFence re-uploaded after each, 0 P0 escape ticks; receipt to swap 0.70-53.0 ms, the 53.0 ms spawn having waited for a whole tick under the loop's old lock scope, fixed (00264ef: the lock from the Shield's inputs to the tick's records), after which 10 events 0.81-6.35 ms; a clear after an expire is judged by its read-back (5b3db86: MAVROS 2 answers a zero-item push with failure although the autopilot takes it) (docs/data/pas_hil_hotapply_20261115/flights.md; CHANGELOG 2026-11-15 (hot-apply on hil; WP3-09 flown)) | a MOVING dynamic_nfz stays held from policy files (no rail records the Shield's clock per trajectory point; the grant's route, injection over REST, flies); on hil only dynamic_nfz was sent (time_window_switch and corridor_swap flew over REST on the ROS 2 rail, dev). |
| WP1-16 | Ingest outputs: JSON Schema export, KML/GeoJSON converter, ingest CLI | DONE | policies/policy_dsl.schema.json; tools/geo_to_policy.py; tests/test_geo_to_policy.py | Polygons with holes refused (minor). |
| WP1-17 | WP1 KPI: policy load round-trip | DONE | tools/wp1_roundtrip_kpi.py -> docs/data/wp1_roundtrip.json (regenerated 6 Nov at 4edd5d1, IR schema 4.0.0): round-trip 32/32, tamper 32/32, negative 39/39, nulls 0/32 and 0/39; tests/test_wp1_roundtrip_kpi.py | None. |
| WP1-18 | WP1 KPI: bundle replayability (load the bundle whose policy_hash matches, never 'the latest') | DONE | hil_smoke_03 (2026-11-06): flown from a clean git checkout on the AGX Orin (code d0e91bb, hil, KPI-grade), its replay bundle signed by the Orin's own key, holding the policy generation whose hash the manifest names, the flight log and the rosbag; `python -m guardrail.replay verify` OK and tools/kpi_report.py re-derives its KPIs from its own log (1 KPI-grade episode); kept in the tracked store as bundles/hil_smoke_03.replay.tar.gz (CHANGELOG 2026-11-06 (hil)); headless re-fly bit-for-bit (tools/rerun_episode.py) | - |
| WP1-19 | Policy DSL acceptance KPIs: schema-validation pass rate 100%, golden cases 100%, cold-start 50-rule ≤1 s, hot-apply ≤50 ms, monitor query ≤5 ms at 10 Hz with 50 rules | DONE | On the AGX Orin (hil machine, companion image, Python 3.12, MAXN): cold start of a signed 50-rule bundle to a ready Shield in fresh processes p95 590 ms (imports + verify + parse + IR + Shield, worst bundle; whole-process wall p95 867 ms) against 1 s, tampered / untrusted / bad-signature bundles refused through the same path (deploy/evidence/hil/wp1_roundtrip_agx.json); hot-apply at 50 rules p99 3.86 ms against 50 ms and _check p99 0.295-0.605 ms (deploy/evidence/hil/bench_shield_50rules_5s_agx.json); 1,301 real ticks at 50 rules x 51 poses: query p99 0.977 ms against 5 ms (deploy/evidence/hil/shield_tick_profile.json); schema validation 100 % and golden cases 100 % (docs/data/wp1_roundtrip.json, desktop; the Orin's image has no jsonschema); in flight, REST receipt to swap 0.444-4.443 ms over 18 events (desktop WSL, 29 Oct); on hil (AGX Orin, 15 Nov) 0.70-53.0 ms over 14 events, the one over 50 ms a spawn that waited for a whole tick under the PAS loop's old lock scope, and 0.81-6.35 ms over 10 events after the fix (00264ef; docs/data/pas_hil_hotapply_20261115/flights.md) | - |
| WP1-20 | Policies used in the CityLife city flights are bundled and replayable | DONE | the city rail flies signed bundles by default (CHANGELOG 2026-10-22; nfz3b bundle signed and deployed to the Jetson, 2026-10-28); replay bundles in each run folder; since 2026-11-06 the deliverable CityLife flights of the report's VLA section - AIOT-VLA v10b with the stand-off feed, aiot_v10bs_01-03, flown from the clean master 55faad0 - are in the tracked store as signed replay bundles (bundles/aiot_v10bs_0N.replay.tar.gz, lab key; each keeps its flown policy hash under its IR 2.x form and verifies, tests/test_replay.py) | - |
| WP1-21 | Mid-term gate (2026-07-20): Policy DSL + IR shipped | DONE | delivered: the Policy DSL (guardrail/models.py, guardrail/compiler.py, the shipped policies under policies/) and its typed IR (guardrail/ir.py); the mid-term report docs/MIDTERM-REPORT-Aug2026.md; the PI's reference repository also holds a Policy DSL with a typed IR at its first commit 9e039b0 (the PI's reference implementation, not the delivered code; docs/data/midterm_gate_evidence/evidence.md) | None. |
| WP1-22 | R6 freeze gate: freeze the IR shape at end of Q1, and give every IR-shape change a semver bump and migration note (the Policy DSL page cites 'R6 ... for the freeze gate at the end of Q1') | DONE | sha256-canonical-v2 + policies/policy.lock.json + docs/DESIGN-policy-identity.md (CHANGELOG 2026-10-06); generated policies get unique ids (CHANGELOG 2026-10-22, 'WP1-22'); 29 Oct: the migration-note register docs/IR-MIGRATIONS.md (7c02284), one row per IR-shape change since the freeze; since 2 Nov any IR-shape change, additive included, is refused by check_lock until IR_SCHEMA_VERSION rises: 2.2.0 versioned `locked_by`, 3.0.0 the required fields (tests/test_policy_hash.py) | - |
| WP1-23 | Single IR contract across the deployed system (Overview: Compiler and Shield 'never re-parse the YAML / JSON source') | DONE | one bundle feeds Shield and VLA nodes (sitl/run_ros2_demo.sh, --bundle); city rail too since 2026-10-22 | None. |
| WP1-24 | policy_hash on every emitted artefact (Overview: CSP, Shield repair log entry, episode bundle) | DONE | audit records carry policy_hash+generation (guardrail/audit.py AuditRecord), csp_g<N>.json on ROS 2 + PAS rails, manifest policy_hash on every run | City-rail flights write no CSP (see WP2-11). |
| WP1-25 | Locked design choice: Python 3.11+ with Pydantic v2 | DONE | pyproject.toml requires-python >=3.11, guardrail tests pass on 3.11; the Orin Nano ran Python 3.10.12 (jetson_facts_20261028.json); since 6 Nov the hil machine is the AGX Orin, whose companion image runs Python 3.12.3 for rclpy and the Shield (deploy/evidence/hil/companion_versions.txt; ARCH-28), on every hil flight since; the desktop flight env vla-real is still Python 3.10.20 14 Nov: the PAS launcher's scene steps ran on guardrail-py312 for the AIoT-VLA hil missions (scripts/run_pas_ardupilot.ps1 at f77efae). | The VLA runtime (torch, transformers) stays on vla-real 3.10, outside the grant's 3.11+ decision for the non-VLA stack; the PI to confirm that reading (decision 8). |

### WP2 - Prefix Compiler (CSP)

19 items: 19 DONE.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP2-01 | CSP injected into the VLA / planner prefix (the WP2 output artefact itself) | DONE | demo/real_vla_demo.py --csp on/off (CHANGELOG 2026-10-06); since 7 Nov flown: OpenVLA-7B (bf16) flew with the CSP in its prompt in 11 of 14 flights at c7a69e8 (the 3 CSP-on seeds of the A/B and the 8 paraphrase flights; tools/run_csp_flights.ps1 -Prefix cspab_20261107; docs/data/csp_flights_20261107.json; CHANGELOG 2026-11-07 (CSP flights)), AirSimNH (classic AirSim, simple_flight), not KPI-grade, 1.23-1.30 Hz, P0 escapes 0 and 0.0 s in the zone on every flight | On base OpenVLA (trained on arm manipulation, never on constraint text) the CSP's effect on repairs is no larger than a rule-free prefix of its length (replay: csp - neutral mean -0.032, csp above neutral in 5 of 14 runs): the model does not use it. A VLA trained to read constraint text is future work; the decision rate (G3) stays with the PI. |
| WP2-02 | Grant KPI: prefix-token budget (configurable budget, respected; CSPBudgetExceeded on P0 overflow) | DONE | guardrail/csp.py budget + CSPBudgetExceeded; docs/data/csp_kpis.json; tests/test_csp.py 99/99 (run 2026-10-28) | Measured offline only. |
| WP2-03 | Grant KPI: CSP coverage of policy IR (spec: P0 coverage = 100% across test fixtures) | DONE | docs/data/csp_kpis.json (regenerated 6 Nov at 52ac56c, 32 policies): at 144 tokens the compiler keeps 122/122 P0 against the naive cut's 120/122, null 0/122; at the default 256 tokens 122/122 for both; tests/test_csp.py | None. |
| WP2-04 | CSP schema: Pydantic v2 model with the locked field set + csp.schema.json export | DONE | guardrail/csp.py typed model, guardrail/csp.schema.json; test_the_committed_schema_has_not_drifted_from_the_model; 29 Oct: policy_id and selection made optional so the spec's own worked CSP validates against the model and csp.schema.json (tests/test_csp.py::test_the_specs_own_worked_example_validates); the flight environment's Python 3.10 stays WP2-19 | None. |
| WP2-05 | Pipeline: spatial + temporal filter on mission context, risk-grading (0.5 sev + 0.3 prox + 0.2 tcrit), order + truncate (P0 kept, P1/P2 by budget) | DONE | guardrail/csp.py time/region filter, risk 0.5/0.3/0.2, P0 kept, budget cut (CHANGELOG 2026-10-06); tests/test_csp.py; since 7 Nov the mission context carries the grant's pose, home, target and allowed modes (guardrail/compiler.py Mission.home_x/home_y/allowed_modes: the return home joins the mission region and the proximity grade, the relevance text names the home and any breach action the mission does not allow; a Mission stating neither compiles byte for byte as before; CHANGELOG 2026-11-07 (CSP flights ready)); a flown VLA has read the CSP (WP2-01) | The weights are the grant's 0.5 / 0.3 / 0.2, configurable as its Outputs ask; the PDF lists their tuning as an open question naming no objective or data, so they were not tuned (decided 7 Nov; the A/B data could serve one). |
| WP2-06 | Mission-context relevance explainable: relevance_explanations[rule_id] for every rule | DONE | guardrail/csp.py relevance_explanations; test_kpi_every_rule_in_the_csp_has_a_relevance_explanation | None. |
| WP2-07 | Per-consumer adapters: action-set adapter (body-frame vx/vy/vz/yaw_rate ranges + forbidden polygons) and cost-map adapter (YAML) | DONE | action-set adapter (guardrail/csp.py) + cost-map adapter (compiler.rasterise_cost_map, docs/DESIGN-cost-map-adapter.md, CHANGELOG 2026-10-12); 29 Oct: CostMap.save also writes the header as <stem>.yaml (the grant's 'emits YAML'), and load_cost_map reads either | No planner reads the cost map (by design). |
| WP2-08 | Templated NL adapter (Jinja2 + Python), canonical source for the paraphraser | DONE | guardrail/templates/ Jinja2 incl. event templates; test_every_rule_type_has_its_jinja2_templates | No flight has carried the sentences to a VLA (WP2-01). |
| WP2-09 | Effectiveness evaluation CLI: offline replay (violating-candidate rate before/after, repair magnitude, token utilisation) + A/B prefix on/off in stress runs | DONE | tools/prefix_eval.py + harness prefix on/off key (CHANGELOG 2026-10-07); tests/test_prefix_eval.py; since 7 Nov run on the logged tuples of all 14 OpenVLA flights, each replay judged with the flight's own recorded Shield and reproducing its flight on every compared frame: repair rate csp - off mean +0.097, rule-free neutral prefix - off +0.129, csp - neutral -0.032; the A/B flown on three seeds (WP4-15) (docs/data/csp_flights_20261107.json, CHANGELOG 2026-11-07 (CSP flights)) | Not KPI-grade (AirSimNH). The stress harness's own prefix arms fly a text-blind pilot (headless, and the hil stub), so their delta is a plumbing check; the delta with a text-reading VLA is the CSP runner's. |
| WP2-10 | CSP carries policy_hash + generation; stateless/deterministic | DONE | guardrail/csp.py policy_hash, generation, issued_at; test_the_csp_hash_is_stable_across_processes_and_hash_seeds | None. |
| WP2-11 | CSP emitted with each flight's artefacts (policy_hash-everywhere invariant) | DONE | demo/follow_vlm.py city_episode: every city follow writes events.jsonl, policy_g0.json and csp_g0.json (f6ee433); tests/test_city_episode.py 3/3 | - |
| WP2-12 | Claims made about WP2 in delivered reports vs what the code does | DONE | tools/check_claims.py (0 live claims in tracked files); docs/CORRECTION-2026-10-06-mid-evaluation-and-deck.md row A5; the correction note was sent to ITRI (researcher, 2 Nov) | - |
| WP2-13 | Paraphraser (Q3 Aug-Oct deliverable, separate service fed by the templated NL); per-paraphrase robustness is WP4's KPI | DONE | Since 2 Nov a separate service: guardrail/paraphraser_service.py, gRPC (guardrail/protos/paraphraser.proto: Paraphrase, Validate, GetStatus) with mutual TLS from the control plane's lab CA, plaintext on loopback only; templated NL in, paraphrases out, each passed by the validator, with the validator and store digests; tests/test_paraphraser_service.py 8/8 (vla-real). guardrail/paraphraser.py with 112 stored paraphrases, the 4 sets stale since C8 re-authored 2 Nov (`stale`: 0 of 21 texts without a set). Validator: the 7 mechanisms of heldout_2026_10_28's misses fixed (2/20 -> 1/20 false accepts, 5/20 -> 0/20 false refusals; a dev set since); a new set, heldout_2026_11_02 (94 probes, written and committed before the validator was read), measured 1/50 and 18/44 before, 1/50 and 14/44 after those fixes (the held-out figure), 0/50 and 3/44 after its own mechanisms were fixed (dev); mutants 3977/3977 refused (CHANGELOG 2026-11-02) | The stress harness (the orchestrator) draws its paraphrases from the service (WP4-10); no held-out estimate exists for the validator as it stands (heldout_2026_11_02 was tuned on, and its author also fixed the validator); a person has yet to read the stored set; the KPI use is WP4-09. |
| WP2-14 | Tests for the prefix compiler acceptance KPIs | DONE | tests/test_csp.py test_kpi_budget_*, test_kpi_p0_coverage_*, test_kpi_p0_overflow_raises_*, test_kpi_every_rule_* (99/99) | None. |
| WP2-15 | CSP re-issued when the policy generation changes (hot-apply). Grant: CSP is a function of 'policy bundle ( policy_hash + generation )'; Overview: 'policy_hash everywhere' | DONE | sitl/ros2_shield_node.py writes csp_g<N>.json per generation; dynamic_nfz compiles (CHANGELOG 2026-10-12); stored ros2fix_on_dyn_nfz (7 Oct) lacks generation 1's CSP; not re-flown; 29 Oct: ros2_hotapply_01-03 wrote csp_g0..csp_g4, one per generation; g0 does not name the REST zone, g1-g4 do, each with its generation's policy_hash | On hil the scenario's mid-flight events re-issue it too (e.g. ntFIN2_178, 13 Nov, KPI-grade: csp_g0.json at take-off, csp_g1.json after the hot-applied dyn-nfz); those events come from the node's scenario file, not REST. |
| WP2-16 | Locked design choice: multi-scale geometry - 'The Prefix Compiler picks the scale, never re-computes' (P0Entry.geometry_ref, e.g. 'coarse:...@v0.3.0/g0') | DONE | guardrail/ir.py multiscale geometry + geometry_ref in the CSP; computed at load, the bundle does not carry the polygons; the compiler reads the scales from guardrail/ir.py and never re-computes them (since 8 Oct); since 2 Nov the bundle ships them (multiscale.json, digest in the signed manifest; check_bundle verifies fine / bbox / coarse without recomputing) and a bundle-loaded policy compiles its CSP with the coarse computation switched off (CHANGELOG 2026-11-02, Policy DSL) | - |
| WP2-17 | Locked design choice: the CSP's action sets are in body frame (vx, vy, vz, yaw_rate); the Shield converts to local-NED before MAVLink emit | DONE | body-frame actions at the ROS 2 boundary, conversion in sitl/mavlink_adapter_node.py (CHANGELOG 2026-10-07); tests/test_mavlink_adapter.py | City rail's StepTracker emits world-frame actions (not a grant rail). |
| WP2-18 | Final-delivery scope: 'Prefix Compiler (full)' scheduled Q2 May-Jul (Overview.txt:170); final gate requires 'Prefix Compiler ... complete' (Grant overview.txt:102-103) | DONE | every compiler part built and tested (WP2-02..08, 10, 14); since 7 Nov the CSP flown in a text-reading VLA's prompt (WP2-01) and its effect measured on and off (WP2-09, WP4-15) | The measured effect on base OpenVLA is not above a neutral prefix's (WP2-01); AirSimNH, not KPI-grade. |
| WP2-19 | Locked design choice: 'Python 3.11+ with Pydantic v2 for the CSP schema' | DONE | same as WP1-25: package 3.11+; the hil companion image runs 3.12.3 (ARCH-28); the desktop env vla-real is 3.10.20 14 Nov: the PAS launcher's scene steps ran on guardrail-py312 for the AIoT-VLA hil missions (scripts/run_pas_ardupilot.ps1 at f77efae). | The VLA runtime (torch, transformers) stays on vla-real 3.10, outside the grant's 3.11+ decision for the non-VLA stack; the PI to confirm that reading (decision 8). |

### WP3 - Safety Shield

24 items: 19 DONE, 5 PARTIAL.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP3-01 | Mid-term (2026-07-20): Safety Shield ROS 2 node skeleton with at least one projection operator | DONE | delivered: the ROS 2 Shield node (sitl/ros2_shield_node.py) over guardrail/shield.py with its projection operators (guardrail/repair.py: altitude, lateral, corridor, clearance, stand-off, kinematic) (audit 5 Oct DONE); flown on the ROS 2 rail and in the `hil` topology (WP3-03, WP3-14); the Gazebo rail of the same mid-term delivery is ARCH-16; the PI's reference repository also holds a ROS 2 Shield node with two repair operators (AltitudeClamp, LateralProjection) at its first commit 9e039b0 (the PI's reference implementation, not the delivered code; docs/data/midterm_gate_evidence/evidence.md) | None. |
| WP3-02 | ROS 2 node: Monitor / Filter / Projection; violation checker reuses the policy IR; bounded repair loop (default 3) | DONE | monitor/filter/projection, bounded repair, Shield.history, Shield.rule_status (CHANGELOG 2026-10-06/-07); tests/test_shield.py | None. |
| WP3-03 | VLA->Shield on a 10 Hz ROS 2 topic (Float32MultiArray); Shield->MAVROS 2 on a 10 Hz topic (mavros_msgs/PositionTarget) | DONE | sitl/mavlink_adapter_node.py publishes mavros_msgs/PositionTarget; sitl/ros2_ws packages; tests/test_mavlink_adapter.py | None. |
| WP3-04 | Action input frame is body frame (vx, vy, vz, yaw_rate); body->local-NED conversion happens after projection, in the MAVLink adapter | PARTIAL | Body frame in: the Shield takes and returns the body action (`Shield.filter_body`, 4 Nov). Every rail and demo loop hands it the body action at the measured heading (wf21, 19 Nov). sitl/ros2_shield_node.py:1737-1752 `shield_step` calls filter_body and does no rotation of its own. Body->local-NED happens in frames.body_to_local_ned after the projection: in the adapters (sitl/mavlink_adapter_node.py; demo/pas_ardupilot/autopilot.py:195-214) and directly in the demo loops after it (e.g. demo/follow_vlm.py, demo/run_demo.py, demo/vla_fly.py), accepted under ARCH-04 (DONE 20 Nov). Registry operators in the body frame since 783e626 (21 Nov, CHANGELOG "2026-11-22 (code)" WP3-04): KinematicClamp, AltitudeClamp, ClearanceProjection, StandoffProjection, CorridorProjection, LateralProjection, PathRepair and registered or Grant operators are handed and hand back the body-frame Action4D at the tick's heading (RepairContext.heading_deg / to_world / to_body, guardrail/repair.py:150-225; HookOperator wraps each Shield._repair_* in to_world / to_body, :296-310; PathRepair returns through ctx.to_body, :494-588; GrantOperator, :762-773), the grant reference's own per-operator body_to_enu ... enu_to_body pattern (tests/test_frame_boundary.py:25-35, 255-265). The pair is added at the operator interface; Shield.filter_body's body->world / world->body rotation around the stack stays (shield.py:7560-7572), the chain carries and re-checks the world action, and to_world hands back the memoised world object, so decisions are bit-identical by construction. Tests: four new tests in tests/test_shieldcore_repair.py (e.g. :449, :493, :518; fail at 71aabb3); the old-chain bit-equality at random headings passes unchanged; the headless smoke's 64 episodes are identical before and after. 59e19a5 contains 783e626 and flew on hil: smFIN2 52 KPI-grade (single_sha 52/52; docs/data/kpi_smFIN2/) and ntFIN4 653 KPI-grade (single_sha 653/653; P0 escape 0 on 17,078 P0-violation ticks, repair success 27,097/27,615, fail-safe (r2) 648/648; docs/data/kpi_ntFIN4/README.md). These flights show the body-in / adapter-NED path, not the operator frame: their decisions equal 71aabb3's. Spot check, demo/out/hil_smoke/ntFIN4_236_dyn-nfz-spawn-ahead: flight_log frame "body", yaw 186.5 deg; adapter_log.jsonl tick 0, a SpeedClamp-only repair (8 -> 4), body [4.00,-0.03,0.01,0] -> NED [-3.977,-0.428,-0.012,0] at heading_deg 186.529. Its 357 GeofenceSlide ticks were braked (tick 51) or sent no setpoint (356 ticks, Loiter), so no GeofenceSlide-repaired action reached the adapter. Earlier: SITL at boot headings -0.4 / 135.6 deg (docs/data/sitl_bodyframe_20261120/, heading_missing_ticks 0). | "Keeps repair operators acting on the same frame the VLA emits in" holds for the registry operators only. The last resorts recorded in the same repair_attempts ledger still compute on the world action inside filter_body's stack-level rotation: ClearanceEscape (Shield._rescue, shield.py:3802), the last-resort GeofenceEscape (Shield._slower_opened_escape, shield.py:8361), SoftRelax and Brake; in the grant reference GeofenceEscape is itself a body-in / body-out operator of DEFAULT_STACK. To close: route ClearanceEscape and the last-resort GeofenceEscape through the body-frame interface, or the PI's written acceptance that the last resorts are not "repair operators" or that the stack-level rotation is enough for them. Residual: the opt-in arc forecast (`--arc-forecast`, off by default) is unflown; an operator registered through repair.register now receives forward / right instead of North / East (the same at heading 0). |
| WP3-05 | Lookahead horizon 5 s at 10 Hz = 50 future poses; time-window boundary checked at t-ε and t+ε | DONE | guardrail.shield LOOKAHEAD_S 5 s / LOOKAHEAD_DT_S 0.1 s on every rail: ROS 2 and PAS rails and the harness (CHANGELOG 2026-10-07, -13), the city rail since 2026-11-03 (demo/follow_vlm.py; fc5s_manual_r1-r3_20261103), and since 2026-11-06 the six older demo scripts (aerialvla_demo, aerialvla_pas_demo, real_vla_demo, run_demo, semantic_demo, semantic_seek), which built 3 s / 0.5 s; flown on hil: hil_smoke_03 (KPI-grade, 50 future poses at 0.1 s, the commanded-velocity forecast) escalated from the forecast's depth at the horizon; window-edge checks at t-eps and t+eps | - |
| WP3-06 | Repair operators in priority order: AltitudeClamp -> LateralProjection (SDF) -> PathRepair; conservative cap θ (2.0 m lateral / 0.5 m vertical) above which projection is abandoned for fail-safe; pluggable Protocol | PARTIAL | guardrail/repair.py: RepairOperator Protocol, `register` / `adapt` / GrantOperator accepting the grant's three-argument signature (repair.py:60-67, 755-830). The default registry is KinematicClamp, AltitudeClamp, Clearance-, Standoff-, CorridorProjection, LateralProjection, PathRepair (repair.py:730-742). Theta is applied with escalation; its basis was decided 2 Nov (reading A, forecast depth in metres; docs/DESIGN-escalation-fsm.md "2026-11-02"). tests/test_shieldcore_repair.py. 17 Nov: PathRepair joined the default chain in the grant's order (with_path_repair: right after LateralProjection, repair.py:851-856), its magnitude on theta's depth basis. Headless before/after (docs/data/pathrepair_default_20261117/): P0 0 -> 0, fail-safe 63/63, 179/179, 247/247 unchanged. Since 21 Nov (783e626, WP3-04) every operator, PathRepair included, takes and returns the body-frame action, so PathRepair's detour is handed back in the body frame. Flown on hil: rel060 (19 Nov) had it on in every Shield-on episode but it acted on 0 ticks. ntFIN3 (71aabb3, CHANGELOG 2026-11-21 (flown)) was the first time it acted: standoff-approach-angles@offset_m=9 (pedestrian), 112-114 ticks per variant, P0 0, fail-safe 3/3 each. ntFIN4 (59e19a5, docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md:1128-1130, 1601-1603): the same three cells, PathRepair on 114 ticks each, mean magnitude 0.684 m/s, P0 escape 0, fail-safe 3/3, mission success 3/3, no false triggers. No other ntFIN4 family shows a PathRepair tick. | (1) The grant's selection rule is not the default. The default repair_mode is "chain" (every operator runs, up to 3 passes). The grant's "first operator that produces a feasible action with magnitude <= threshold wins" exists as repair_mode="first_feasible", off by default, ungated and not measured since the 17 Nov PathRepair redesign (repair.py:93-112, 433-438). (2) PathRepair's grant case, a predicted segment crossing a boundary or keep-out zone, is opt-in in chain mode (PathRepair(chain_categories=("geofence","standoff"))) for measured reasons: nfz-head-on held in Loiter, and gps-dropout 178/179 (repair.py:419-431). Flown, it has acted only round stand-off rings (one family, 3 cells, hil). Boundary and zone detours are unflown on SITL or hil. (3) The follow and perception rails keep PathRepair off until a closed-loop follow A/B. (4) Depth-at-entry / time-weighted theta were measured, not adopted. Reading (1) and (2) as accepted deviations needs the PI's written acceptance. docs/FINAL-REPORT.md section 7 lists it as a recorded deviation; that is the lab's decision, not the PI's. |
| WP3-07 | Escalation FSM Normal -> Brake -> Loiter -> RTL -> Land with N-in-T (3 in 5 s), θ, T_recover = 2 s, YAML-overridable; drives SET_MODE GUIDED/LOITER/RTL/LAND | PARTIAL | guardrail/fsm.py (defaults :1302-1307: N 3, T 5.0 s, theta 2.0 / 0.5 m, T_recover 2.0 s, loiter timeout 30.0 s; tests/test_fsm.py 131/131; YAML override test_yaml_profile_overrides_with_the_grants_symbols, tests/test_fsm.py:1799-1807; node --fsm-config, sitl/ros2_shield_node.py:3023), wired to Shield and rails. Every edge of the grant's ladder flown on hil with the grant's trigger: ntFIN2 (116214f) G1 348, G3 334, G5 253, G7 93, G8 64, G9 216, G10 3, G11 18, G12 3; hs10 (8a473fa) G2 'N-in-T: 3 onsets in 5 s' (hs10_003, 5.83 s), G4 'N-in-T persists' (10.03 s), G6 on a blocked leg home (hs10_004, 5.17 s), X2, X3; the thresholds N 3, T 5 s, theta 2.0 / 0.5 m, T_recover 2 s appear in the flown reasons. SET_MODE GUIDED / RTL / LAND driven on every KPI campaign (ntFIN2: GUIDED 64, RTL 272, LAND 290). 22 Nov, SET_MODE LOITER flown on hil for the first time (CHANGELOG 2026-11-22 (code) 'literal SET_MODE LOITER, opt-in', commit 29a1440 of 21 Nov; 2026-11-22 (flown)): sk1122b at 59e19a5, `sweep_scenarios --backend hil --rail-hold loiter` (node --hold-flight brake --hold-mode LOITER --literal-loiter, sitl/ros2_shield_node.py:6247-6253; adapter --loiter-throttle centre; experiments/sweep_scenarios.py:1010-1024, :4622 literal_loiter_problems; sitl/mavlink_adapter_node.py:350ff), 2 missions (dyn-nfz-spawn-ahead entered Brake by X1, dyn-nfz-rotate-step by G1 then G3): mode_requests ['LOITER','RTL'] on both, autopilot heartbeat LOITER, Loiter held 29.9 s, altitude deviation 0.021 / 0.005 m (tol 1.0 m), RC3 stand-in 301/301 at 1500, the literal_loiter hold check passed on both, then G5 'loiter timeout: 30.10 s >= 30 s' -> RTL -> G9; P0 escape 0; sweep counts pass 1 / fail 1 (sk1122b_002 failed its gate: 'fail-safe: labelled not expected, triggered') (docs/data/shakedown_20261122/README.md 'sk1122b', sk1122b_00{1,2}_*_literal_loiter.json, sk1122b_sweep.json). 17 Nov bench: SET_MODE LOITER also accepted by the real FC (CUAV V5 Nano, AC 4.7.1, never armed; docs/data/fc_bench_20261117/). | The delivered default does not drive SET_MODE LOITER for the FSM's Loiter: the node defaults to --hold-flight guided (sitl/ros2_shield_node.py:6247) and the sweep to DEFAULT_RAIL_HOLD "guided" (experiments/sweep_scenarios.py:1023-1024), which every KPI campaign flies (ntFIN2: 337 Loiter entries, 0 LOITER requests); this rests on lab decision 7 (15 Nov), PI confirmation pending. The literal LOITER is opt-in only: 2 shakedown missions, not a KPI campaign; it relies on a SITL-only throttle stand-in for the safety pilot (RC_CHANNELS_OVERRIDE ch3 1500 as sysid 255, fixed); it holds where the aircraft stopped, not at a moved hold point; the Loiter ended only by the G5 timeout (no G8 or G4 flown in that mode); and it cannot run under the SROS 2 profile (the adapter is not granted /uas1/mavlink_sink). The per-profile FSM YAML is not selectable: FSMConfig.from_mapping's `profiles:` overlay (guardrail/fsm.py:1412-1438) is reachable only from the library; the node, sitl/run_sitl_demo.py:675 and demo/pas_ardupilot/perception_node.py:7135 call FSMConfig.from_yaml without a profile, sweep profiles cannot pass one, and no flown run used a non-default --fsm-config. G6's battery reason is flyable on fsm_edges (rail_battery) but not flown (the blocked path is). fsm-n-in-t-flicker is labelled expected by lab decision 13 (PI confirmation pending, a scoring matter for WP3-15). Closes when the PI accepts the GUIDED hold or the default drives SET_MODE LOITER, and a profile's FSM YAML can be selected on the rails. |
| WP3-08 | Shield acts on the rule's violation_action (monitor_only, project_fix, brake, loiter, RTL, land); hard vs soft constraints; P0/P1/P2 priority | DONE | dispatch on all six actions in guardrail/shield.py; experiments/templates/failsafe_escalation.yaml headless; no flown policy uses loiter/RTL/land/monitor_only actions; 29 Oct, ROS 2 rail + MAVROS, one REST-spawned zone per action: land (G12 -> LAND), RTL (G11), loiter (X4), brake (X1, held), project_fix (ros2_hotapply_03), monitor_only as a soft P2 rule (flown through, P0 0.0) and as a hard P0 rule (counted as escapes, by design) | One zone per action on the flown policy, not one policy file per action. KPI-grade on hil (nightly ntFIN2, 13 Nov): RTL rules (G11, 18 transitions), land rules (G12, 3), loiter rules (X4, 3) and project_fix rules flown; monitor_only not checked there. |
| WP3-09 | MAVLink adapter ROS 2 node: single body->local-NED boundary; SET_POSITION_TARGET_LOCAL_NED; MISSION_ITEM_INT; SET_MODE for FSM | DONE | sitl/mavlink_adapter_node.py, SET_MODE via ModeRequests; flown 7 Oct (ros2fix2_*); since 3 Nov the MISSION_ITEM_INT row: sitl/ros2_mission_node.py (STYLE=mission) checks a waypoint plan with the Shield's own position check and clamps its speed to the policy's cap, the adapter writes it through /mavros/mission/push and reads it back, and in AUTO the next waypoint is replaced in flight by a partial write (MISSION_WRITE_PARTIAL_LIST + one MISSION_ITEM_INT) when a hot-applied zone makes its leg illegal, the original written back once the leg is legal; flown mission_wp_20261103d (re-flown 6 Nov from the main checkout with the six-field manifest the mission node writes since then: mission_wp_20261106b - code_revision a3c36a51e28c, topology dev, sim_speedup 1.0 read from the autopilot; the 4 Nov re-flight mission_wp_20261104 has no manifest) (ROS 2 rail + MAVROS 2, dev, GCS-set fence): the bag's /uas1/mavlink_sink carries 6 MISSION_ITEM_INT and no MISSION_ITEM (tools/bag_mission_protocol.py), the aircraft held at the replaced waypoint (y <= 15.35 m; the zone's margin begins at 17 m), both updates confirmed by read-back, mission complete in 28.1 s, no tick with the position illegal (CHANGELOG 2026-11-03 (G)); 15 Nov (code + tests, not flown): the in-flight update is written as the whole mission with the one item replaced (MISSION_COUNT, MISSION_ITEM_INT per item, MISSION_ACK ACCEPTED; --mission-update-write partial keeps the 3 Nov write), one update at a time (MissionPushQueue) (tests/test_mavlink_adapter.py 47/47); the perception rail can send its setpoints through the adapter (--link mavros --setpoint-via adapter) (CHANGELOG 2026-11-15); 15 Nov flown: the full write in AUTO, mission_wp_20261115b (6c9698d, ROS 2 rail + MAVROS 2, dev, GCS-set fence, sim_speedup 1.0 read from the autopilot): the harness spawned a zone across the first leg (generation 1) and switched it off (generation 2); the replacement and the restore were each written whole - MISSION_COUNT 4, four MISSION_ITEM_INT, MISSION_ACK ACCEPTED, MAVROS's push answered success, read back 4 matching - and the bag's /uas1/mavlink_sink carries 12 MISSION_ITEM_INT, no MISSION_ITEM and no MISSION_WRITE_PARTIAL_LIST; the aircraft held at the replaced waypoint (y <= 15.38 m; the zone's margin begins at 17 m), last waypoint reached in 28.2 s, 0 ticks with the position unsafe (docs/data/wp3_09_20261115/); the perception rail's adapter route on hil: pas_hil_adapter_01 (KPI-grade, AGX Orin), its setpoint path /shield/setpoint (body) -> sitl/mavlink_adapter_node.py -> /mavros/setpoint_raw/local, 4,092 setpoints forwarded, 0 P0 escape ticks (CHANGELOG 2026-11-15 (hot-apply on hil; WP3-09 flown)) | LOITER is sent as BRAKE (kept as best practice, 2 Nov: a recorded decision); the mission style runs no escalation FSM and writes no kpi.json, and MISSION_ITEM_INT was flown on dev only; the perception rail's default route publishes its own PositionTarget through the same frames.py conversions (the adapter route is opt-in; the KPI campaigns fly the ROS 2 rail, whose only setpoint path is the adapter). |
| WP3-10 | ArduPilot GeoFence (FENCE_ENABLE, FENCE_TYPE, FENCE_ACTION) configured as the last-line backstop that fires only if the Shield fails | DONE | sitl/fence/guardrail_fence.parm; fence fired with the Shield off (ros2fix2_off_fence); PAS rail uploads zones before arming (CHANGELOG 2026-10-13), 0 breaches; since 3 Nov GCS-set: sitl/gcs_backstop.py, the ground station's own process on mavlink-router's GCS endpoint (run_ros2_demo.sh GCS_BACKSTOP=1, SITL without the defaults file), sets FENCE_TYPE, FENCE_ACTION, FENCE_ALT_MAX, FENCE_MARGIN, AVOID_ENABLE and FENCE_ENABLE and uploads the signed bundle's zones as MISSION_ITEM_INT, reading every value and item back (gcs_backstop.json; the Shield node's own read-back matches it); flown with the Shield off - gcs_fence_off_20261103 (re-flown 4 Nov from the main checkout, code recorded: gcs_fence_off_20261104) (6 m/s) and _20261103b (3.5 m/s): 'Fence Breached' and GUIDED->RTL at 3.37 s and 4.08 s - and on - gcs_fence_on_20261103b and gcs_fence_on_20261103c (re-flown 4 Nov from the main checkout, code recorded: gcs_fence_on_20261104) (past the zone's corner at 2 m/s: the Shield bent the route around it, the aircraft passed 1.58 m from the zone and reached its target): no breach; the 29 Oct Shield-on breach (ros2fix_on_policy55) is explained by its own records: poly-test's tri-nfz has a vertex at home, and the breach came at 26.5 s, after the FSM's RTL had brought the aircraft home and the mission had ended at 10.2 s (CHANGELOG 2026-11-03 (G)) | GCS_BACKSTOP=1 is opt-in (the default rail keeps the defaults file, so Mission Planner keeps the GCS endpoint); the GCS step is a scripted stand-in for an operator in Mission Planner; a GCS-set fence is static, so a hot-applied zone is the Shield's alone, and a zone over the pad is held back and not backstopped. |
| WP3-11 | Per-violation-tick JSONL audit record (Pydantic v2) with ts, policy_hash, generation, monitor_tick, raw_action, violations (boundary_intersect_at_s), repair_attempts (result/magnitude_m/iterations), emitted_action, fsm_state_before/after; rotated per mission | DONE | 2 Nov, audit record version 4 (guardrail/audit.py AuditRecord, Pydantic v2; policies/audit_record.schema.json): the grant's ten fields written first, in the grant's order, with the grant's content - raw_action / emitted_action body frame {vx, vy, vz, yaw_rate} (frames.to_body at the tick's heading, converted when the record is written), a violation's category geometric / envelope / time-window with boundary_intersect_at_s, repair_attempts the operators tried (ok / skip with its reason, magnitude_m, iterations = the pass); the grant's p5 example validates verbatim; a world-frame action under the grant's name is refused; this project's content under raw_action_world / emitted_action_world, rule_category, repair_results; every rail writes shield_audit.jsonl, one file per episode (a stale one moved aside); every stored record (294 files, 71,064 records, versions 1-3) reads through upgrade_record; tests/test_shieldcore_audit.py 16/16 | None for the record. The Shield still repairs in a world frame (WP3-04): the body frame is a conversion at write time. `contract` (a non-finite command zeroed) is a fourth category value, this project's extension. Flown since: the hil runs write it, e.g. demo/out/hil_smoke/ntFIN2_178_dyn-nfz-spawn-ahead/shield_audit.jsonl (13 Nov; the grant's fields first, raw_action in body frame, category geometric with boundary_intersect_at_s, repair_attempts with result and reason). |
| WP3-12 | Hot-apply listener (REST callback) bumps policy generation mid-flight | DONE | --hotapply-port on sitl/ros2_shield_node.py and sitl/run_sitl_demo.py (CHANGELOG 2026-10-13); perception_node has no listener; never driven during a flight; 29 Oct: REST hot-apply into flying ArduPilot SITL + MAVROS 2 (ROS 2 rail, dev): a dynamic_nfz spawned, rotated, translated and switched off (time_window_switch) mid-flight, generations 0-4, receipt to swap 0.444-4.443 ms against 50 ms; on ros2_hotapply_03 the Shield and the FSM stopped the aircraft short of the REST zone (X1, G3), released it after the switch-off (G8), and it reached the target | KPI-grade needs hil. |
| WP3-13 | Monitor tick budget ≤100 ms end-to-end at 10 Hz; geometry query budget ≤5 ms with ~50 rules x 50 poses | PARTIAL | Geometry query budget, ~50 rules x 51 poses on the AGX Orin (tools/profile_shield_tick.py over 1,301 real flight ticks, MAXN, clocks not locked, MAVROS stopped): _check p99 0.977 ms, max 1.047 (6 Nov at d0e91bb, deploy/evidence/hil/shield_tick_profile.json). Re-run at bc04e4a: p99 0.994, max 1.056, 0 of 1,301 over 5 ms (deploy/evidence/hil/shield_tick_profile_20261114.json). Filter p99 22.6-26.7 ms. Flown tick budget, KPI-grade hil, stub pilot: ntFIN2 (116214f) tick_ms max 74.06 ms, with 13 start-up e2e ticks over 100 ms that were fixed after. ge1113, hs09, hs10, hs11 and hh02 had 0 over at the fixed node code. rel060 slowest tick 70.89 ms. ntFIN3 (71aabb3): tick_ms p99 28.9 / max 74.52, e2e_ms max 74.62, 0 of 174,186 over 100 ms. smFIN2 (59e19a5): tick_ms max 72.91, 0 of 15,662 over. **ntFIN4 (59e19a5, current master code)**: tick_ms p50 7.8 / p99 28.4 / max 105.50 ms and e2e_ms p99 28.5 / max 105.60 ms, **1 of 183,909 ticks over 100 ms** (docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md lines 19-20, verdict "p99 within; 1 tick(s) over"). The tick is ntFIN4_236_dyn-nfz-spawn-ahead row 50, t 5.03 s, shield_ms 10.581, G3 Brake to Loiter. It was the zone's hot-apply tick: the generation write, the cold CSP compile (csp_g1.json) and the fence publication all ran before the setpoint and took 87 ms, against a usual 20-40 ms (spawn ticks that are also G3: median 51.0 ms, 31.8-58.8 ms apart from this one). The README says this code path predates 71aabb3. Camera to first setpoint with the VLA in the loop (14 Nov, AIoT-VLA hil): p50 159-163 / p99 282-296 ms, mostly VLA inference and the detector. | The 100 ms end-to-end budget is not met on every tick at the current code. The KPI-grade nightly ntFIN4 at 59e19a5 recorded one tick at 105.5 ms (e2e 105.6 ms) on a hot-apply tick, on a code path present since before 71aabb3. ntFIN3's 0 over did not exclude it. docs/data/kpi_ntFIN4/README.md describes the fix (generation write, CSP compile and fence publication moved after the setpoint; templates compiled at bring-up; per-phase tick stamps, "CHANGELOG 2026-11-23"). At 529e787 there is no such commit or CHANGELOG section. DONE when that fix is committed and a KPI-grade nightly with hot-applied zones flies with 0 ticks over 100 ms. Also still open: e2e_ms spans the Shield node only (the pilot's DDS hop and the adapter's hop to MAVROS, about 1.3 ms p50 / 2 ms p99, are outside it); every hil figure has the stub as pilot; ~50 rules only in the offline Orin profile (MAVROS stopped, last re-run at bc04e4a, not at 59e19a5). |
| WP3-14 | P0 violation escape rate, target 0 (hard limit), on the canonical KPI configuration | DONE | Grant (02-implementation/stress-testing.md): "P0 violation escape rate \| 0 (hard limit) \| Counter from Shield repair log"; safety-shield.md: "report cannot ship if non-zero on P0". 13 Nov, the KPI-grade nightly hil campaign ntFIN2 (116214f, profile nightly, the AGX Orin rig; demo/out/hil_ntFIN2/sweep.json, runs demo/out/hil_smoke/ntFIN2_*; tools/kpi_report.py --campaign): 0.0000 - 0 escapes on 16,703 P0-violation ticks over 577 Shield-on missions - against 0.6375 for the Shield-off control nfz-head-on-control (153 of 153 P0 ticks, 3 missions) and a passthrough null of 0.213; the 3 gps-dropout flights the campaign did not accept also 0. Before it, 0 P0 escapes: harness library 179 Shield-on episodes (CHANGELOG 2026-10-22), every kpi.json of the 45 flights 27-28 Oct, PAS+ArduPilot Shield-on flights 12-21 Oct, none of these KPI-grade; on `hil` since 7 Nov, KPI-grade (topology hil, sim_speedup 1.0, a clean revision, the Orin's hil_evidence): P0 escape rate 0.0 on every Shield-on mission of the smoke campaigns rh03 (44 missions, fb96fe5), sm1110 (47, 9eed35b) and sm1111 (45, 22d434b) and of the grant example sweep ge1110 (32, 9eed35b), against 0.646 and 0.650 for the Shield-off control nfz-head-on-control in sm1110 and sm1111 (CHANGELOG 2026-11-11 (hil rig flights)); and 0 escapes on all 587 Shield-on missions of the first nightly nt1111 (22d434b, demo/out/hil_nt1111/sweep.json) | None for the grant's count. To state beside it: on the true position no Shield-flown tick entered a P0 zone, but in 21 Shield-on missions ArduPilot carried the aircraft into one (sweep.json `_true_state`): 18 dyn-nfz-spawn-ahead runs whose zone appeared 3 m ahead at 4 or 8 m/s, closer than the aircraft can stop, where the Shield braked and held and ArduPilot's GeoFence reported "Fence Breached" and took RTL at 5.7-5.8 s (2.3-2.7 m deep), and 3 time-window-opens-on-sideways-pilot runs whose RTL leg flew back through the zone. The pilot is the stub (a learned VLA on hil is ARCH-20). The supplement ntFIN2s adds the 11 missions not flown or not accepted. |
| WP3-15 | Fail-safe trigger correctness (reference target ≥99%: triggered when expected, not when not expected) | DONE | guardrail/fsm.py score_failsafe_triggers with nulls. The rubric was written before relabelling (docs/RUBRIC-failsafe.md; tools/failsafe_rubric.py). Reading (r2) has rail-vehicle labels with E5 on the prescribed flight (adopted 10 Nov; labels docs/data/failsafe_labels_rail.json derived at 442397f, 49c455c, 897dac7). GeoFence backstop rule: decision 14, settled 15 Nov by the lab under the PI's 6 Oct instruction. KPI-grade nightly ntFIN4 at 59e19a5, the current master code (docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md lines 18 and 72-76; README): (r2) **648/648 = 1.000, Wilson 95 % low 0.994**, nulls always/never 0.506/0.494. 18 were triggered by the GeoFence backstop, all dyn-nfz-spawn-ahead, all expected and correct (ntFIN4_178-186, 214-222). With those 18 left out as censored, the rule not applied: 630/630, Wilson low 0.994. Counted as not triggered: 630/648 = 0.972. The other readings: (r) 636/648 = 0.981; (a) 573/648 = 0.884; the campaign's own labels 586/649. Left out: ntFIN4_374 and ntFIN4_375 corridor-swap-shifted, censored in Loiter (648/650 = 0.997 even if both were errors). smFIN2 (59e19a5): (r2) 51/51. Shakedown sk1122a: 72/72, with the 60 GPS-noise/latency cells flown on the rail for the first time (24 beyond-margin fail-safes, 36 within-margin none, as labelled). Earlier: ntFIN3 (71aabb3) 587/589 = 0.997, Wilson low 0.988 (misses ntFIN3_164, ntFIN3_168); ntFIN2 (116214f) 584/585 = 0.998. The headless nightly gives 741/741 before and after the retreat ray (CHANGELOG 2026-11-22 (code)). | The KPI holds at the point estimate and at 95 % confidence only under reading (r2): (r) gives 0.981 and (a) 0.884 on the same 648 runs. (r2) and the backstop rule were adopted by the lab after rig flights had been scored another way, and the PI's confirmation is still to be recorded (docs/RUBRIC-failsafe.md sections 1 and 9). The figure does not need the backstop rule (630/630 without it). The grant calls the GeoFence a backstop that fires "only if the Shield itself crashes or fails to emit", and the 18 backstop runs still fail their own gates. The previous nightly ntFIN3 at 71aabb3 had 2 misses and a Wilson low of 0.988, so one campaign clears the bound and one does not. Wilson treats seed-repeats as independent. The 90 refused ntFIN4 episodes (stub-unreadable arms, stressors the rail cannot realise) are scored headless only. |
| WP3-16 | Mean repair magnitude (mean //post_repair − planner_output//) | DONE | Grant (02-implementation/stress-testing.md): "Average repair magnitude \| tracked \| Mean of \|\|post_repair − planner_output\|\|". tools/kpi_report.py repair_magnitude_by_priority / _by_operator_class (CHANGELOG 2026-10-13); on `hil` since 7 Nov, KPI-grade, the mean over episodes: rh03 5.32 m/s, sm1110 5.28 m/s, sm1111 5.22 m/s (ge1110 made no repair) (CHANGELOG 2026-11-11 (hil rig flights)); 13 Nov, the KPI-grade nightly ntFIN2 (116214f, 577 Shield-on missions; tools/kpi_report.py --campaign): 6.407 m/s pooled over repaired ticks, per template (1.859 to 16.000 m/s) and per cell, and by priority and operator class | None for the tracked KPI. The norm is translational (vx, vy, vz in m/s); the yaw rate is reported per episode beside it, never folded in, so a reviewer reading the grant's norm over all four action components would get another number. |
| WP3-17 | Mean time to safe: from violation to back-in-safe-zone (reference sweep expects ≤2.0 s) | DONE | Grant (02-implementation/safety-shield.md): "Mean time to safe \| tracked; from violation to back-in-safe-zone". 13 Nov, the KPI-grade nightly ntFIN2 (116214f; tools/kpi_report.py --campaign): 3.23 s mean, max 5.39 s, over the 169 Shield-on missions whose unsafe spell recovered; 75 more never recovered before an RTL or Land (54 three-simultaneous-violations, 18 dynamic_nfz_movement, 3 failsafe_escalation), counted, not averaged; per template 0.47 to 3.86 s. Before it: 29 Oct: a recovery from outside an altitude band flies at the climb cap (the Shield's recovery_law 'cap', 5c98142); 3 Nov republished library (docs/data/scenario_sweep.json): mean time to safe 2.997 s over 39 recoveries, max 6.7 s (3.135 s over 37 before: X7 in Loiter ends one wind recovery sooner, longer runs add two); 4 Nov republished from the merged tree (a586956, clean): 2.926 s over 39, max 6.7 s - the one cell that moved is dyn-nfz-spawn-on-top@size_m=20 (5.6 s -> 2.8 s: the zone appears centred on the aircraft, every edge equally near, and since the tie is broken by the commanded heading the pilot's own forward command is the way out, so the Shield passes it; its per-tick log flags the state unsafe on every tick inside); 8 Oct run 2.81 s; none KPI-grade; 7 Nov: the grant's worked example, the one sweep its 2.0 s belongs to, flown as written as a profile of its own (experiments/profiles/grant_example.yaml, 36 episodes, headless, on the ideal and the rail vehicle; CHANGELOG 2026-11-07 (grant example sweep)): P0 escapes 0, fail-safe 36/36 (every label not expected, N1), and no time-to-safe spell at all - the zone leaves the track 4.2 s after it appears, so its 2.0 s holds with nothing to measure; our time to safe (guardrail/kpi.py) counts each spell in which a stop at the true position breaks a rule within the 5 s lookahead, which starts no later and ends no sooner than 'from violation to back-in-safe-zone' read as position | None for the tracked KPI. The grant's one figure, <= 2.0 s, is the example sweep's, met there with no spell to measure (ge1110 on hil too); the nightly's 3.23 s is not held to it. The mean leaves out the 75 spells that never recovered (31 % of the 244 missions with an unsafe spell), and our spell starts no later and ends no sooner than the grant's words read as position: a reviewer may ask for both beside it. |
| WP3-18 | Canonical KPI configuration 'hil': desktop runs sims + MAVROS + GCS, Jetson Orin runs VLA + Shield over the network | DONE | hil flown 2026-11-06 (CHANGELOG 2026-11-06 (hil)): ArduPilot SITL + mavlink-router (+ the GCS endpoint) on the desktop, MAVROS 2 + the Shield + the pilot node on the AGX Orin over the cable; the label comes from evidence (device tree, L4T, SITL elsewhere, the MAVROS chain), and hil_smoke_03 is KPI-grade. MAVROS sits on the Orin, as the PI's reference repository's topologies page and docs/DESIGN-topologies.md place it (the grant's table words it on the desktop; docs/DESIGN-projectairsim-ardupilot.md records the difference). The August 'canonical-hil' folders are read as dev by normalize_topology and refused by is_kpi_grade | - |
| WP3-19 | Smoke test matrix: high-speed near-edge, sudden NFZ (hot-applied), time-window switch instant, three simultaneous violations, GPS noise/latency | PARTIAL | experiments/profiles/smoke.yaml carries all five rows (dyn-nfz-spawn-ahead/-on-top :45-48, time-window-* :54-59, high-speed-near-edge :71, three-simultaneous-violations :78, gps-noise-within-margin :81, gps-noise-beyond-margin :85); tests/test_sweep.py. GPS noise / latency on the hil / sitl rails, companion-side (commit 115640b, 21 Nov; CHANGELOG 2026-11-22 (code) 'WP3-19 / WP4-14'): the ROS 2 Shield node applies the cell's latency_ticks delay line and seeded Gaussian noise to the pose the Shield and the FSM's queries get, as the headless harness does; metrics.json perception_stress records it (sitl/ros2_shield_node.py:1193, :5869); rail_stress_check refuses a flight whose draws are not the cell's (experiments/sweep_scenarios.py:4191). Flown: sk1122a (59e19a5, hil, 60 GPS-noise missions: 36 within-margin successes, 24 beyond-margin fail-safes as labelled; e.g. sigma 1.0 m, latency 5 ticks, seed 1001, 1113 draws, max offset 4.53 m; rail_stress_check matched every run; docs/data/shakedown_20261122/README.md). All five rows in one KPI-grade smoke on hil: smFIN2 (59e19a5, 22 Nov, 52 missions flown, 12 refused, 52/52 KPI-grade, single_sha 52/52): high-speed-near-edge smFIN2_039-040, dyn-nfz-spawn-* 017-018, time-window-* 022-027 (incl. time-window-switch-event 027), three-simultaneous-violation 043, gps-noise-within-margin 044-045 (sigma 0.3, latency 0 / 2 ticks), gps-noise-beyond-margin 046 (sigma 2, latency 5); P0 escape 0 (2239 P0-violation ticks), fail-safe (r2) 51/51, tick_ms max 72.91 ms (docs/data/kpi_smFIN2/README.md, kpi_report_smFIN2_2026-11-23.md:126-155, :198-234). The nightly ntFIN4 (59e19a5) flew the same five families, with the 60 GPS-noise missions on the rail for the first time in a nightly (docs/data/kpi_ntFIN4/README.md). Before: rh03, sm1110, sm1111, rel060, smfin1120 and ntFIN2/ntFIN3 flew four of the five (GPS noise refused). | The grant's sentence is a per-release obligation ("Every release runs this matrix in Stress Testing's smoke set"), and no release has run the five-row matrix: the only release since smoke.yaml exists, v0.6.0 (a85dc91, 19 Nov), flew four of the five in its hil smoke rel060 with GPS noise refused (CHANGELOG 2026-11-19 (release smoke rel060)), and no lab CI record exists at a85dc91; the five-row runs (sk1122a, smFIN2, ntFIN4) are at 59e19a5, an unreleased development commit (VERSION 0.6.0, CHANGELOG [Unreleased]). Closes with a release at or after 59e19a5 whose own smoke flies the five rows (as WP4-14). Residual: the GPS noise is applied to the Shield's and FSM's pose on the companion (the autopilot's EKF sees clean GPS), which models the Shield's stability under sensor degradation, not an autopilot GPS fault; each smoke cell flies once on seed 1001 (in smFIN2, 046 went straight to fail-safe with no P0 tick; 043 had repair success 0.0 %, fail-safe 1/1 as labelled). |
| WP3-20 | Repair success rate (Safety Shield page, Acceptance KPIs (locked): 'tracked; informs Prefix Compiler effectiveness eval') | DONE | Grant (02-implementation/safety-shield.md): "Repair success rate \| tracked; informs Prefix Compiler effectiveness eval". guardrail/kpi.py repair_success with theta (CHANGELOG 2026-10-06); 0.776 on the 8 Oct pymavlink run; theta basis decided 2 Nov (reading A, the default the KPI already reads); 13 Nov, the KPI-grade nightly ntFIN2 (116214f, 577 Shield-on missions; tools/kpi_report.py --campaign): 98.2 % (26,473 of 26,968 repair ticks), always-brake null 0 %, per template from 0.0 % (failsafe_escalation, three_simultaneous_violations, time_window_static) to 100 %; the effectiveness eval it informs is the CSP A/B (WP2-09, AirSimNH) | None for the tracked KPI. Theta (2.0 m / 0.5 m as forecast depth) judged about a third of the converged ticks (8,498 of 26,595 in the report's breakdown over all 580 Shield-on runs); the rest carried only repairs theta does not judge, and the report shows how far the rate moves under a velocity reading of theta, for which the grant gives no horizon. |
| WP3-21 | Single IR contract: the Shield consumes the Policy IR and never re-parses the YAML/JSON source ('hard invariant; violating it ... breaks reproducibility', Overview cross-cutting invariants) | DONE | every rail flies the signed bundle; YAML only under --dev-yaml (demo/follow_vlm.py:122-193; CHANGELOG 2026-10-22); Jetson metrics policy_source kind bundle, hash verified | None. |
| WP3-22 | Exactly one rule-evaluation code path in the system ('Violation checking' section; violation checker library shares the IR) | DONE | Since 3 Nov guardrail/shield.py is the one evaluator, built from the IR (PolicyIR, compile_fence, FenceRecord): RuleClock (the compiler's _RuleClock subclasses it), the point verdicts (zone_breached :758, corridor and band tests, ZoneRuler :943, clearance_breached :827, standoff_breached :846, standoff_ring_m :885) and escape_direction. compiler.action_violations, fence_export, rtl_path, replay.zone_dwell and FenceGuard's gate/slide all read it (CHANGELOG 2026-11-03 (E)). 17-18 Nov: every shapely, ray-cast and segment-distance rule test outside guardrail/{geometry,planar,shield,ir}.py calls the library; the semantic demo, semantic seek and A/B P0 counter ask the Shield (min_clearance_on_map, altitude_band_report, rule_status). The last open path was closed by wf21-onepath3 (d8fd243, fb6a27e, 18 Nov; CHANGELOG 2026-11-19 (wf21, code)): FenceGuard's building half measures with the Shield's field (Shield.clearance_ruler, :5856) and judges with clearance_breached against Shield.clearance_limits_now (:5880) (demo/follow_vlm.py:2556-2593); guard_field accepts only "shield" and refuses "grid" (:2419-2436, CLI choices :9355). The end-of-flight stand-off counts and the stand-off scorer call standoff_breached / standoff_ring_m; the landing column test asks the Shield (hard_violations, rule_status). The library keeps one rule measure, field_distance (:1208); nearest_cell_clearance (:1371) is diagnostic only, its one caller tools/onepath_guard_replay.py (tests/test_shieldcore_onepath.py: _CALLED_ONLY_IN :250, _LIBRARY_CLEARANCE_MEASURES :1071). Outside the library the only clearance measures are planning and target identity (city_planner.inflate/_penalty_flat, identity.__init__; _CLEARANCE_OUTSIDE :975). The scans find no wrapped primitive called elsewhere and no IR rule threshold compared outside the checker except 15 named non-verdicts (export, planning, scenario setup, lint; _THRESHOLD_COMPARISONS_NOT_VERDICTS :1108-1174, test :1524); mutation nulls planted (:1572). tests/test_shieldcore_onepath.py 27/27 in lab CI 8e73f86, 22 Nov (187/187 files; docs/data/ci_local/8e73f86b6af6.json). Flown with the new default: 7 AGX Orin CityLife follow flights at bfb4973, 19 Nov, all guard_field "shield" (runs_summary.json; docs/data/onepath_agx_20261119/README.md): onepath n=5 within-30 m 0.969/0.954/0.472/0.943/0.917, AerialVLA hierarchy 0.987, nfz3b 0.345; P0 escape 0 and 0 mission contacts on all 7; the guard's replay over the five onepath runs equals the logged mode on 99.97 % of 20,971 ticks; the 0.472 is an identity-lock loss, not the guard (diagnosis_20261119.md). 2 re-flights at 28a594d, 20 Nov (docs/data/follow_agx_20261120/, metrics.json guard_field "shield"): 0.945 and 0.48, P0 escape 0. The hil KPI campaigns run the same Shield: ntFIN4 at 59e19a5, P0 escape 0 over 650 Shield-on flights (docs/data/kpi_ntFIN4/). | Residual, outside the grant's words: (1) the follow flights are topology projectairsim-remote-sim, functional and not KPI-grade by design; (2) nearest_cell_clearance stays in the library as a replay-only diagnostic, test-pinned to one caller; (3) independent verification oracles in tools, such as tools/msl_dem_rail.py oracle(), check the declared bands without a Shield by design (with the bands copied as constants, which the threshold scan cannot see): they score runs and judge no flight; (4) models.start_conflicts is a load and authoring lint (frame origin inside a hard zone's footprint or margin, `<=`; tools/geo_to_policy.py:313-319 refuses to write such a take-off point), named in the scan, judging no flight; (5) ArduPilot's GeoFence evaluates the fences fence_export uploads from the Shield's geometry, the grant's own autopilot backstop (WP3-10; it triggered 18 of ntFIN4's 648 scored fail-safes, see X-13); (6) guardrail/fsm.py KinematicCaps.clamp (:864-874) mirrors the Shield's SpeedClamp / ClimbClamp (sharing fsm.cap_exceeded, the minimum cap across rules, no yaw clamp), used by follow_vlm.descent_ask and the scoring tools: a repair and planning mirror, not a verdict. The scans hold only the scanned forms (Python in SOURCE_DIRS, by pattern). |
| WP3-23 | Shield node architecture: sliding-window buffer (last 50 actions + predicted trajectory) and geometry queries on the IR's R-tree spatial index | DONE | since 3 Nov each filter() tick admits its action and that action's predicted trajectory into the buffer (shield.SlidingWindow, last 50) before anything is checked, and the checker reads the trajectory from it (a tampered buffer blinds the checker: tests/test_shieldcore_onepath.py); static zones through the IR's STRtree, moving dynamic_nfz through the IR's moving-zone STRtree rebuilt per tick (PolicyIR.moving_index), decisions identical to the one-by-one test on 3900 bench samples; library sweep 180/180 identical; tick profile within noise (CHANGELOG 2026-11-03 (E)) | None required. Notes: no DSL rule reads past actions, so the 49 older entries are a record; corridors, stand-offs and clearance are not in a tree (keep-in or subject-relative, checked at every pose); the per-tick rebuild costs about 0.06-0.1 ms per check on the 48-moving-zone bench |
| WP3-24 | Safety Shield ROS 2 node delivered as a 'Python package; rclpy' (Outputs table) | DONE | sitl/ros2_ws/ rclpy packages + launch files; tests/test_ros2_package.py | None. |

### WP4 - Stress Testing

21 items: 14 DONE, 7 PARTIAL.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP4-01 | Every reported KPI must come from a Stress Testing run in the canonical HIL topology | PARTIAL | dev label + KPI gate refuse non-hil runs (guardrail.manifest.is_kpi_grade); hil on the AGX Orin since 6 Nov (hil_smoke_03 first KPI-grade run, CHANGELOG 2026-11-06 (hil)); the harness flies the scenario library there (`--backend hil`) since 7 Nov (rh03, sm1110, sm1111, ge1110; CHANGELOG 2026-11-11 (hil rig flights)). KPI-grade nightlies on hil, each scored from that campaign alone by tools/kpi_report.py --campaign from a clean tree: ntFIN2 + ntFIN2s (116214f, 588 after the 17 Nov delivery regrade), ntFIN3 (71aabb3, 594 KPI-grade; docs/data/kpi_ntFIN3/), ntFIN4 (59e19a5, 653 of 654 KPI-grade, 650 Shield on; docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md:17-19: P0 0 on 17,078 P0 ticks, fail-safe (r2) 648/648 Wilson low 0.994, tick_ms max 105.50 ms, 1 of 183,909 over 100 ms); smokes smfin1120 (49) and smFIN2 (52) (docs/data/kpi_smfin1120/, docs/data/kpi_smFIN2/). Since 22 Nov the 60 GPS-noise / latency cells fly on the rail (refused through ntFIN3; ntFIN4 refused 90 vs ntFIN3's 150; CHANGELOG 2026-11-22 (code) WP3-19/WP4-14, (flown)). Prefix A/B now has a harness value on hil: pab3 (1239619, prefix_ab profile, 33 KPI-grade, hand-written CSP consumer; docs/data/kpi_prefix_ab_20261119/). docs/FINAL-REPORT.md (draft v0.5, 23 Nov) sections 2 and 5 report the nightly KPIs from ntFIN4, ntFIN3 beside it. | The final report (docs/FINAL-REPORT.md, draft v0.5, unsigned) still reports KPIs that are not from a Stress Testing run: section 2 'Other locked KPIs, not from the nightly' gives DSL acceptance and policy load round trip from docs/data/wp1_roundtrip.json (offline tool, also printed in ntFIN4's KPI report, kpi_report_ntFIN4_2026-11-23.md:1794), CSP P0 coverage / token budget from docs/data/csp_kpis.json (offline), and per-paraphrase robustness (WP4-09) from the PAS follow outside the harness (docs/data/pas_hil_paraphrase_20261115/); the nightly reports' 'A/B' and 'Per-paraphrase robustness' sections read n/m (kpi_report_ntFIN4:1896-1902) because the stub reads no text (ntFIN4 refused its paraphrase / prefix arms). Either those are labelled in the report as not from a Stress Testing run or measured in one. The prefix A/B has a harness hil value only with the hand-written consumer (pab3); pab_aerialvla_04 (learned VLA) is outside the harness. The KPI campaigns fly the stub where the grant's hil has the Orin run "VLA + Shield" (ARCH-20). PI never answered the dated fallback (21 Oct #4). |
| WP4-02 | Scenario library: declarative YAML (one file per template) validated by Pydantic ScenarioSpec, with templates dynamic_nfz_movement / time_window_switch / radius_scaling / corridor_swap, events, parameter_sweep, expected_kpis | DONE | guardrail/scenario_spec.py ScenarioSpec/ScenarioEvent, experiments/templates/ 16 files, parameter sweeps; tests/test_scenario_spec.py | Headless only (WP4-04). |
| WP4-03 | Harness drives dynamic_nfz, time_window_switch and corridor_swap mid-flight via the Policy DSL REST hot-apply endpoint; events spawn/translate/rotate fence, activate/deactivate rule, swap corridor, scale radius | DONE | every event type mid-flight in the headless harness; gRPC HotApply forwards to a bound REST endpoint (CHANGELOG 2026-10-13); not through a flying rail until 3 Nov (below); 29 Oct: REST hot-apply into flying ArduPilot SITL + MAVROS 2 (ROS 2 rail, dev): a dynamic_nfz spawned, rotated, translated and switched off (time_window_switch) mid-flight, generations 0-4, receipt to swap 0.444-4.443 ms against 50 ms; on ros2_hotapply_03 the Shield and the FSM stopped the aircraft short of the REST zone (X1, G3), released it after the switch-off (G8), and it reached the target ; 3 Nov: the harness drove a flying rail - tools/harness_client.py drive, a gRPC session over mTLS bound to the rail's REST endpoint, the bundle injected pinned and checked against the endpoint, four HotApply at mission time (dynamic_nfz spawn 2.0 s, time_window_switch off 9.0 s, corridor_swap 15.0 s and 26.0 s) each POSTed by the harness to /hot_apply and applied (generations 1-4) on harness_corridor_20261103c (re-flown 4 Nov from the main checkout, code recorded: harness_corridor_20261104b), and two (spawn, switch off) on the mission-style mission_wp_20261103d (re-flown 6 Nov from the main checkout with the six-field manifest the mission node writes since then: mission_wp_20261106b - code_revision a3c36a51e28c, topology dev, sim_speedup 1.0 read from the autopilot; the 4 Nov re-flight mission_wp_20261104 has no manifest) (CHANGELOG 2026-11-03 (G)) 14 Nov, the first harness-driven hil episodes, campaign hh02 (bc04e4a on the desktop and the AGX Orin, sim_speedup 1.0, all six KPI-grade; python -m guardrail.harness_service serve --rail hil + tools/harness_hil_episode.py; bundles demo/out/harness/hh02/): StartEpisode launched each mission on the rig without its events and the harness forwarded them over mTLS REST to the Orin's hot-apply listener (the companion's api layer, deploy/docker/companion-api.Dockerfile), then EndEpisode scored the flight and wrote the grant's six names (manifest.json, policy_bundle.tar.gz with every generation, episode.bag2/ from the run's rosbag2, shield_audit.jsonl, harness_events.jsonl, kpi.json): spawn + rotate (hh02_001 dyn-nfz-rotate-step: POST /hot_apply at 3.00 s, PATCH /dynamic_nfz/bar rotate at 8.00 s), spawn + scale (hh02_002 radius-scaling-grow-deep, scale at 10.00 s), deactivate a rule (hh02_003 time-window-switch-event, switch_off at 1.02 s), activate a rule (hh02_004 rule-activated-mid-run@at=2, switch_on at 2.00 s), swap a corridor (hh02_005 corridor-swap-shifted, 5.02 s) and translate (hh02_006, an EmitEvent translate_polygon_fence of bar at 12.02 s after the scenario's spawn and rotate, generation 3); fail-safe under the adopted reading 5/5 with 1 censored (corridor-swap-shifted ended in Loiter, as in ntFIN2); P0 escape 0. The first flight hh01_001 applied both forwards and was refused by a harness check fixed in bc04e4a (CHANGELOG 2026-11-14 (the companion's REST hot-apply layer)). | Translate has no library cell (flown through EmitEvent); every library cell's events have flown through the harness only once, one seed each; the harness forward is late by up to ~0.2 s plus a round trip against the mission's own clock, so a harness-driven flight is not tick-identical to its campaign twin. |
| WP4-04 | Harness runs scenarios on the simulation rails (Gazebo Harmonic functional sweeps, KPI-grade runs in HIL) and emits an episode bundle per episode | PARTIAL | Headless harness and modelled ArduPilot response arm (CHANGELOG 2026-10-21). Since 6 Nov the rails take the library's scenarios, one mission per scenario (`rail_refusal` names what a rail cannot fly). KPI-grade on hil through experiments/sweep_scenarios.py --backend hil: rh03, sm1110, sm1111, ge1110, nt1111, ntFIN2, rel060, smfin1120, ntFIN3. At 59e19a5 (CHANGELOG 2026-11-22 (flown)): smFIN2 flew 52 missions, all KPI-grade, and ntFIN4 flew 654 of 744 planned, 653 KPI-grade (90 refused: stub-unreadable paraphrase/prefix arms and stressors the rail cannot realise). Six-name bundle per episode: AuditLogger now creates shield_audit.jsonl at mission start (CHANGELOG 2026-11-22 (code)). Every ntFIN4 episode (654/654, demo/out/hil_ntFIN4/episode-*/kpi.json `bundle_files`) and every smFIN2 episode (52/52) records all six names (manifest.json, policy_bundle.tar.gz, episode.bag2/, shield_audit.jsonl, harness_events.jsonl, kpi.json) as written. The zero-repair corridor-along runs bundle an empty audit (e.g. episode-2026-11-22T07-44-04Z--corridor-along--seed1001/shield_audit.jsonl, 0 bytes; sk1122a 3/3, docs/data/shakedown_20261122/README.md). Each bundle sits beside the run's signed `<run>.replay.tar.gz`. Gazebo Harmonic: one functional sweep flown at 28a594d on 20 Nov (docs/data/gazebo_sweep_20261120/: 8 missions, seed 1001, 7 of 8 passed, P0 escape 0.0 on the 7 Shield-on missions; 7 of 8 wrote the six names because corridor-along had no audit then). Since 22 Nov the gazebo_functional profile covers every family the Gazebo rail admits: 38 families, 41 cells, seeds 1001-1003, 123 missions in the dry run (CHANGELOG 2026-11-22 (code)). Not flown. The gRPC Harness's own rail session wrote six-name bundles on hh02 (6 missions, 14 Nov). | Gazebo functional sweeps beyond one seed: the 123-mission gazebo_functional profile (3 seeds) has only had a dry run. No Gazebo sweep was flown after 20 Nov, so the empty-audit fix is not shown on the Gazebo rail. The 20 Nov nfz-head-on fail-safe gate failure and the GUIDED-hold setpoint stream handed to BRAKE on 2 missions are still open. The grant's broad sweep (~3000 scenarios x 5 seeds at speed > 1, functional rail) has not been flown, and speed > 1 is refused while the Shield node runs on wall time. KPI-grade campaigns are flown by experiments/sweep_scenarios.py, not by the gRPC Harness session; the Harness rail session has flown 6 missions (hh02). On the hil rail, the six-name-per-episode part is now met (ntFIN4 654/654, smFIN2 52/52). |
| WP4-05 | RPC harness: Python service + gRPC stubs over mTLS (StartSession, InjectPolicy, HotApply, StartEpisode, IssueTask, EmitEvent, EndEpisode, streams) | DONE | guardrail/harness_service.py + guardrail/protos/harness.proto, all 10 RPCs over mTLS; headless only; tests/test_harness_service.py 57/57 (2 Nov). The bundle carries the grant's shield_audit.jsonl (30 Oct) and policy_bundle.tar.gz (2 Nov: the injected file byte for byte, generation 0's and the last generation's IR, every generation by hash, an index with each member's SHA-256; write_policy_bundle); kpi.json harness.bundle_files names the grant's three files and which are written; 3 Nov: a session bound to a flying ROS 2 rail's REST endpoint forwarded HotApply into its flight over mTLS with the SROS 2 keystore's lab CA (tools/harness_client.py drive; harness_corridor_20261103c (re-flown 4 Nov from the main checkout, code recorded: harness_corridor_20261104b), mission_wp_20261103d (re-flown 6 Nov from the main checkout with the six-field manifest the mission node writes since then: mission_wp_20261106b - code_revision a3c36a51e28c, topology dev, sim_speedup 1.0 read from the autopilot; the 4 Nov re-flight mission_wp_20261104 has no manifest); WP4-03; CHANGELOG 2026-11-03 (G)) See WP4-03: StartSession, StartEpisode (hil), the scenario forwards, EmitEvent (hh02_006) and EndEpisode flown end to end over mTLS on the hil rig, 14 Nov. | IssueTask / InjectPolicy were exercised headless only (tests/test_harness_service.py); the rig's hot-apply listener needs the companion api layer (docs/RUNBOOK-orin-hil.md 15a). |
| WP4-06 | Smoke (~50 scenarios x 1 seed per build), Nightly KPI (~200 x 3 seeds, HIL, sim_speedup 1.0), Broad sweep (~3000 x 5); sweep config YAML per profile; CI smoke pipeline | PARTIAL | smoke/nightly/broad profiles defined as YAML (experiments/profiles/); smoke + nightly run headless (CHANGELOG 2026-10-22). CI smoke pipeline: .github/workflows/tests.yml runs on every push (:691-694) with the headless smoke as its last step (:1006-1009). Master was pushed to the fork on 22 Nov (was 214 commits behind). GitHub runs 44976329320 (59e19a5: 30-min job timeout in 'Standalone test files', raised to 90 at 22ddb61) and 44991484912 (0daa20e: Linux-only test failures) failed, smoke step skipped; last green 13 Oct (38471368810); fixes 2b01e77, 132e979 on docs/1123, unpushed. Local tools/ci_local.py at 8e73f86: 187/187, smoke rc 0, 64 episodes (docs/data/ci_local/8e73f86b6af6.json). nightly.yml cannot fire: GitHub schedules only the default branch `main` (tests.yml:666-670). Smoke on hil, ~50 × 1 seed at 1.0: rh03, sm1110, sm1111, rel060, smfin1120 (71aabb3, 49 flown), smFIN2 (59e19a5, 52 flown, 52 KPI-grade; docs/data/kpi_smFIN2/). Nightly on hil at sim_speedup 1.0: nt1111 (22d434b), ntFIN2 (116214f, 69,706 s), ntFIN3 (71aabb3, 21 Nov, 744 planned, 594 flown, all KPI-grade, 72,478.6 s; docs/data/kpi_ntFIN3/), ntFIN4 (59e19a5, 22-23 Nov, 744 planned / 248 cells × 3 seeds, 654 flown, 653 KPI-grade, 90 refused, 78,879.2 s = 21.9 h; P0 escape 0, fail-safe (r2) 648/648 Wilson low 0.994; docs/data/kpi_ntFIN4/README.md). Seeds: the 60 GPS-noise missions now draw their noise from the cell's seed on the rail (e.g. seed 1001, 1113 draws; docs/data/shakedown_20261122/README.md). Gusts stay refused on the rails (experiments/sweep_scenarios.py:3906). Broad: runs unattended (--resume, --shard, --merge-only, 17 Nov); only a slice has run; the Gazebo sweep of 20 Nov was 8 missions × 1 seed at speed 1 (speed > 1 refused: the Shield node runs on wall time; docs/data/gazebo_sweep_20261120/README.md:107). | Smoke per build: the GitHub smoke has not completed on any pushed build since 13 Oct. Both 22 Nov runs failed in the test step before the smoke, and the fixes are unpushed. The KPI-grade smoke is a hand-launched hil campaign, not per build. Nightly: flown four times, not on a nightly cadence (each run takes 20-22 h on the one rig); nightly.yml cannot fire from master (the default branch is main). Only the GPS-noise cells' three seeds draw different stressors; the other cells are three flights of one deterministic scenario, with gusts refused. Broad sweep (~3000 × 5 at speed > 1): never run in full; the rails refuse speed > 1, and only a headless slice has run. |
| WP4-07 | KPI auto-report generator (Markdown + JSON): per-scenario-family stats table (episode count, P0 escape rate, fail-safe correctness, mission success rate, mean repair count, mean repair magnitude) + top-K (K=10) failure cases linked to episode bundles | DONE | tools/kpi_report.py family table, top-K, Markdown+JSON, --baseline (CHANGELOG 2026-10-06, -13); tests/test_kpi_report.py, tests/test_wp4outputs_kpi_report.py | Needs KPI-grade input (WP4-01). |
| WP4-08 | Mission success rate, tracked, defined as outcome == success AND no P0 violation, rolled up per scenario family | DONE | Grant (02-implementation/stress-testing.md): "Mission success rate \| tracked, no hard target \| outcome == success AND no_P0_violation". mission_success_rate 0.603 headless library (CHANGELOG 2026-10-22); follow flights scored by within-30 m; on `hil` since 7 Nov, KPI-grade, Shield-on missions, P0 escapes 0 on every one: rh03 19/44 = 0.432, sm1110 23/47 = 0.489, sm1111 22/45 = 0.489, ge1110 32/32, each campaign's kpi_report rolled up per family (CHANGELOG 2026-11-11 (hil rig flights)); 13 Nov, the KPI-grade nightly ntFIN2 (116214f; tools/kpi_report.py --campaign): 252/577 = 0.437 Shield on, rolled up per (template, parameter cell), hover null 0 | None for the tracked KPI. Our rule is stricter than the grant's (no dwell in a zone or ring and no unsafe position entered, besides the goal reached and no P0 violation), so the figure is at or below the grant's. |
| WP4-09 | Per-paraphrase robustness, a WP4-owned KPI: mission outcome stability across paraphrased task prompts | DONE | paraphrase arms in the harness (CHANGELOG 2026-10-07); since 7 Nov measured with a text-reading VLA: OpenVLA-7B flew the canonical CSP-on mission and 8 service-issued paraphrases, outcome stability 7/8 (utterance 4/4, verb_phrase 3/4) (docs/data/csp_flights_20261107.json, CHANGELOG 2026-11-07 (CSP flights)); 15 Nov on hil with a text-reading pilot, KPI-grade: AIoT-VLA v10b on the AGX Orin flew the PAS follow mission with the source "follow a red car" and the paraphraser's 8 stored paraphrases of it as its grounding text (de7e748; docs/data/pas_hil_paraphrase_20261115/flights.md): the source ended target_lost twice, the model's own noun phrase "a red car" held the car to max_s (as pas_hil_aiot_est_01 did); all 8 paraphrases flew a mission (the 2 whose start gate never passed, which reads no paraphrase, were flown again): 6 ended as the source did and 2 held the car to max_s; 0 P0 escape ticks over the 13 runs (CHANGELOG 2026-11-15 (paraphrases on hil), (Mission Planner; paraphrase retries)) | One flight per paraphrase, one mission and one task family; the stress harness's own paraphrase arms still fly the hil stub, which reads no text. |
| WP4-10 | Paraphraser: a separate service (Q3, Aug-Oct deliverable) that turns the Prefix Compiler's templated NL into free-form paraphrases issued to the VLA | DONE | same artefact as WP2-13: since 2 Nov a separate gRPC + mTLS service (guardrail/paraphraser_service.py) serving validated paraphrases of the templated NL; validator figures as WP2-13 (heldout_2026_11_02: 1/50 false accepts, 14/44 false refusals held out; 0/50, 3/44 dev); since 7 Nov the service's paraphrases are issued to a VLA: all 8 paraphrase flights of OpenVLA-7B drew theirs from the service on loopback (each run records `via`), and the gRPC harness (guardrail/harness_service.py, the reference design's orchestrator) draws every paraphrase arm from the service with no in-process fallback: a server bound to no service refuses the arm (CHANGELOG 2026-11-07 (CSP flights ready), (CSP flights)) | Not KPI-grade (AirSimNH). |
| WP4-11 | Every episode produces a self-contained bundle replayable bit-for-bit: manifest.json, policy_bundle (incl. all hot-apply generations), episode.bag2 (rosbag2), shield_audit.jsonl, harness_events.jsonl, kpi.json; Q4 reports Q3 runs 'replayed for reproducibility' | PARTIAL | Headless episodes re-fly bit-for-bit (tools/rerun_episode.py, CHANGELOG 2026-10-13). Since 7 Nov every flown hil run holds a signed `<run>.replay.tar.gz`. Since 18-19 Nov each rail-campaign episode is also written under the grant's six names (rel060). At 59e19a5, all 654 ntFIN4 episodes and all 52 smFIN2 episodes record the six names as written in kpi.json `bundle_files` (demo/out/hil_ntFIN4/episode-*), with policy_bundle.tar.gz "written, complete" (every generation's IR, hash-checked) and an empty shield_audit.jsonl on zero-repair runs (CHANGELOG 2026-11-22 (code)). GPS-dropout and estimator-loss episodes now replay: the node logs `note_flown` pipe-clock readings, and a436d32 hands the replay the perceived pose under GPS noise. Replay at the flown revision, open loop on the Windows desktop, of runs flown on the aarch64 Orin (docs/data/kpi_ntFIN4/replay_at_revision_ntFIN4.json, 654 runs, 303,022 ticks): 355 identical; 298 differ in the last float digits (112 plain, 186 through the body-frame rotation, at most 6.9e-14); 1 differs in a repair's pass count on a float tie (ntFIN4_304 dyn-nfz-rotate-spin, as ntFIN3_306); 0 not replayable. smFIN2 (docs/data/kpi_smFIN2/replay_at_revision_smFIN2.json): 43 identical, 9 last float digits, 0 not replayable. Q3: tools/replay_q3.py on 17 Nov re-ran 15 Q3 runs through the Shield on their logged inputs (10 identical, 5 differ); none had been flown from a clean revision. | Not bit-for-bit on every episode: 299 of 654 ntFIN4 replays differ, 298 in the last float digits (aarch64 vs AMD64 rounding, at most 6.9e-14) and 1 in a repair pass count (ntFIN4_304). The replay is open loop on logged inputs. It does not re-fly the SITL/hil episode and reproduce its trajectory. Q4's replay of Q3 runs for reproducibility is not done: the Q3 runs were not flown from a clean revision, so replay_q3 is a regression check, not a replay at the flown SHA. |
| WP4-12 | Six-field manifest per episode (code_revision, vla_model_hash, policy_hash, random_seed, sim_speedup, topology); sim_speedup=1.0 mandatory for KPI-bearing runs | DONE | Grant (02-implementation/stress-testing.md): "every episode emits a six-field manifest (...); sim_speedup=1.0 is mandatory for any KPI-bearing run in the HIL topology". The KPI-bearing runs: all 583 run folders of the nightly ntFIN2 (13 Nov) carry the six fields, code_revision 116214f3e6ee (clean), sim_speedup 1.0, topology hil, vla_model_hash the stub pilot's source hash (guardrail.vla_stub GotoPilot 454, ConstantPilot 126, StubVLA 3), random_seed the cell's seed (demo/out/hil_smoke/ntFIN2_*/manifest.json). Six-field manifest on every run (guardrail/manifest.py); the mission-style ROS 2 node only since 6 Nov (sitl/mavros_evidence.py, the GUIDED Shield node's evidence; mission_wp_20261106b: code_revision a3c36a51e28c on a clean tree, topology dev, sim_speedup 1.0 read from the autopilot; mission_wp_20261103d and mission_wp_20261104 have none); but 27-28 Oct manifests read code_revision '...-dirty' (PC, e.g. demo/out/nfz3b_pc_hier_dronecats_20261028) and 'unversioned' (Jetson, git-archive sync), and Jetson runs say topology 'projectairsim-single-host'; since 28 Oct the Jetson is a git checkout (deploy/orin/sync_follow_controller.ps1 ships a git bundle), so code_revision resolves there; a remote simulator is stamped projectairsim-remote-sim (guardrail/manifest.py) | None for the KPI-bearing runs. The section 6 follow comparisons (27-29 Oct, simple_flight, never KPI evidence) keep their '-dirty' / 'unversioned' code_revision and a topology outside dev / hil / flight (ARCH-31); the untracked file that made every run since 27 Oct '-dirty' was committed on 2 Nov (e9a08d6). The seed is recorded but no flown hil cell consumes it (WP4-06). |
| WP4-13 | Per-step record (observation, policy_state with policy_hash+generation, planner_output, post_repair, executed, outcome); auto-labels violation_type and risk_level; exporters (rosbag2 -> JSONL/Parquet), both episode-based and step-based | DONE | tools/export_episodes.py JSONL + Parquet with read-back check (CHANGELOG 2026-10-21); with a run's rosbag2 it reads the bag (`rosbags`) and action.executed is the setpoint MAVROS was sent (/mavros/setpoint_raw/local) in each tick's window, the flight log's verdict beside it, the join refused when the bag does not line up; since 2026-11-06 `rosbags` 0.10.11 and pyarrow are in vla-real, tests/test_wp4outputs_exporters.py 38/38 with no skip; on the KPI-grade hil run hil_smoke_03: 303 steps, executed from the bag on 303/303, JSONL and Parquet written and read back. Rails without ROS 2 (city, headless) keep the flight log's verdict, labelled a fallback on every step; the SITL rail has no camera, so its observation is the state topics (pose, IMU, GPS) and their references | - |
| WP4-14 | Every release runs, in Stress Testing's smoke set: high-speed near-edge, sudden hot-applied NFZ, time-window switch instant, three simultaneous violations, GPS noise/latency (Safety Shield page) | PARTIAL | The smoke profile carries all five Shield stressors (experiments/profiles/smoke.yaml:45-85); since 22 Nov all five fly on the hil rail, GPS noise / latency included (115640b; see WP3-19): smFIN2 at 59e19a5 flew the whole matrix KPI-grade (52/52, P0 escape 0, fail-safe (r2) 51/51; docs/data/kpi_smFIN2/). Releases: the only release since the matrix existed, v0.6.0 (a85dc91, 19 Nov), had its hil smoke rel060 (48 Shield-on flights, P0 escape 0, fail-safe 48/48) with four of the five rows; GPS noise was refused on the rail then (CHANGELOG 2026-11-19 (release smoke rel060)). All five ran headless in the lab CI record of 55aefcd (183/183, smoke rc 0, 64 episodes; docs/data/ci_local/55aefcd0c1dd.json; code identical to the tag except CHANGELOG/VERSION). 59e19a5 is unreleased (VERSION 0.6.0; CHANGELOG [Unreleased]). CI per build: master pushed to the fork on 22 Nov; GitHub 'tests' runs on every push (.github/workflows/tests.yml:691-694) and its last step is the headless smoke (:1006-1009). Runs 44976329320 (59e19a5: the 30-min job timeout inside 'Standalone test files', raised to 90 at 22ddb61) and 44991484912 (0daa20e: Linux-only test failures) failed, so the smoke step was skipped on every job. The last green GitHub run was 13 Oct (38471368810). The Linux fixes (2b01e77, 132e979) are on docs/1123 and not pushed. Local CI tools/ci_local.py at 8e73f86: 187/187, smoke rc 0, 64 episodes (docs/data/ci_local/8e73f86b6af6.json). | No release has yet run the whole five-row matrix in the KPI-grade smoke set: v0.6.0's rel060 lacked GPS noise / latency, and smFIN2 (all five) flew unreleased 59e19a5. Closing needs a release cut at or after 59e19a5 whose own smoke flies the five rows, or 59e19a5 tagged as the release smFIN2 flew. The per-build CI smoke has not completed on GitHub since the push: both runs failed before the smoke step, which has no `if: always()`, and the fixes are unpushed. The GitHub smoke is headless (functional, never KPI-grade); the KPI-grade smoke is a 1.8 h hil campaign run by hand. No written release procedure (RUNBOOK/CONTRIBUTING) binds a release to its smoke. |
| WP4-15 | A scenario can be flagged for A/B; the harness runs it with Prefix on/off on the same seed and the KPI report shows the delta (Prefix Compiler page) | DONE | prefix on/off key in the harness (CHANGELOG 2026-10-07); since 7 Nov the A/B flown with a CSP-reading VLA: OpenVLA-7B on three seeds (each a recorded start, the same for both arms), repair rate off 0.200 / 0.127 / 0.108, on 0.392 / 0.207 / 0.437, mean delta +0.201; outcome off success 3/3, on fail 3/3 (unsafe_position_entered) (tools/score_csp_flights.py, docs/data/csp_flights_20261107.json) 16 Nov, the harness's own A/B flown on hil (campaign pab1, c7257f1, the hand-written CSP-consuming adapter guardrail/prefix_consumer.py, not a learned VLA): 33 KPI-grade missions, P0 escapes 0, fail-safe r2 15/15 on the labelled prefix-off arm, on vs off mission success 0.600 / 0.600, repairs per episode 6.6 / 130.5 (A/A largest pair 7), magnitude 1.78 / 9.38 m/s, in tools/kpi_report.py's A/B section (docs/data/kpi_prefix_ab_20261116). corridor-drift-out's prefix-on arm ended RTL 3/3: the adapter's 0.5 s samples stepped over a cut of the dog-leg's inner corner, fixed after the flight by a whole-path test (CHANGELOG 2026-11-16 evening). | Met by pab3 (19 Nov, the harness's prefix_ab profile: 15 same-seed on/off pairs on hil, the KPI report's A/B section with deltas and an A/A floor); the grant's sentence (prefix-compiler.md: "the harness runs it twice (Prefix on / off) with the same seed, and the KPI report shows the delta") does not require a learned pilot. pab_aerialvla_04 was flown outside the harness and its seed does not fix the CityLife traffic, so it supplements pab3; whether a learned VLA reading the CSP shows an effect belongs to WP4-19 (independent review 20 Nov). |
| WP4-16 | Project AirSim is the perception-stress rail; KPI numbers come only from KPI-grade (canonical HIL) stress runs | DONE | Project AirSim flown by ArduPilot (G1); KPI gate refuses its runs (kpi_grade false); no ScenarioSpec drives it 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). | One run per stressor (three for day), not a campaign through the suite's scenario format; non-KPI by design. |
| WP4-17 | Final delivery: 'KPI report on dynamic-scenario stress' (Grant overview delivery table, 2026-11-30); reference recommendation.md:34 says 'A KPI report from a nightly stress run' | PARTIAL | Generator tools/kpi_report.py --campaign (since 11 Nov), under the adopted (r2) reading and the backstop rule (decision 14). Committed KPI reports from KPI-grade hil nightlies, each from a clean tree with its r2 re-score and replay-at-revision beside it: kpi_ntFIN2 (13 Nov) / _20261115 / _20261117_delivery (588 KPI-grade, 583/584); kpi_ntFIN3 (71aabb3, 594 KPI-grade, P0 0 on 17,457 P0 ticks, (r2) 587/589 Wilson low 0.988, 0 of 174,186 ticks over 100 ms; docs/FINAL-REPORT.md section 5 was written from it up to draft v0.4); 23 Nov docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md (+ .json), generated at 2f9d863 clean over ntFIN4 (59e19a5, 78,879 s, 653 KPI-grade): P0 escape 0.0000 on 17,078 P0-violation ticks over 650 Shield-on flights (control 0.6500, null 0.1935; line 17), fail-safe (r2) 648/648 = 1.000, Wilson 95 % low 0.994 (line 18), tick_ms p99 28.4 / max 105.50 ms, 1 of 183,909 over 100 ms (line 19; ntFIN4_236 spawn tick, docs/data/kpi_ntFIN4/README.md 'The one tick over 100 ms'), per-family tables incl. dynamic_nfz_movement 164 flights, top-10 failures linked to replay bundles, replay at the flown revision 355 identical / 298 last float digits / 0 not replayable / 1 pass-count tie, single_sha 653/653. smFIN2 at the same revision (docs/data/kpi_smFIN2/, 52 KPI-grade, P0 0, 51/51). Since draft v0.5 (23 Nov) docs/FINAL-REPORT.md sections 2 and 5 are written from ntFIN4, with ntFIN3 beside it. | Remains the delivery itself on 2026-11-30, inside the signed final report (X-01: docs/FINAL-REPORT.md is draft v0.5, unsigned; signer/CA unnamed, the user decided 22 Nov to wait for the PI). ntFIN4's tick verdict is 'p99 within; 1 tick over' (105.5 ms): the fix its README cites as 'CHANGELOG 2026-11-23' is not in the tree at 529e787, so it is neither committed nor flown. The nightly reports' 'A/B: prefix on / off' and 'Per-paraphrase robustness' read n/m (stub reads no text); the A/B is pab3's own harness report (docs/data/kpi_prefix_ab_20261119/), per-paraphrase is outside the harness (WP4-09). Acceptance of the report and of the (r2)/backstop readings is the PI's. |
| WP4-18 | Perception-stress scenarios on Project AirSim (Stress Testing page: 'Project AirSim for perception stress'; reference dual-rail.md:16 'Texture / lighting / shadow stressors', :70 lighting, weather, novel textures, dynamic actors) | DONE | dynamic actors only (40 pedestrians, traffic, signals in CityLife); experiments/templates/sensor_degradation.yaml headless 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md); 15 Nov, the texture and shadow stressors (demo/pas_ardupilot/perception_stress.py, CHANGELOG 2026-11-15 (texture and shadow stressors)): texture re-skins the level's 347 road meshes with the engine's WorldGridMaterial (World.set_object_material, every call's answer recorded); shadow turns the level's own sun, DirectionalLight_0, to 8 deg above the horizon and 60 deg round (World.set_object_pose, read back by the desktop and by the attaching node on the Orin), and every stressor refuses a sun that is not its own; flown with AIoT-VLA and the target estimator on the AGX Orin (pas_hil_aiot_texture_01, pas_hil_aiot_shadow_01, same flags as the unstressed pas_hil_aiot_est_01): detector hit rate 0.865 unstressed, 0.820 shadow, 0.095 texture; target lock 0.823, 0.775, 0.094; frame luma 96.4, 69.1, 101.8; the texture mission lost the car after about 100 s and ended target_lost; 0 P0 escape ticks in both (docs/data/pas_hil_stress_20261115/) | Flown with a per-scenario metric: day, dusk (-2 EV), night (-4 EV: no detection, the mission refused), rain (RAIN 0.8), fog (FOG 0.6: no red car found in 330 s, the mission refused and landed), shadow (the sun at 8 deg), texture (novel road surface); dynamic actors are the level's own (40 pedestrians, traffic, signals). One run per stressor, non-KPI by design; night is a darker camera (exposure), not lit streets (CityLife has no SunSky). |
| WP4-19 | Final demo gate (reference recommendation.md:33): all four components exercised by a non-trivial scenario from the Stress Testing scenario suite | PARTIAL | CityLife: VLA + signed bundle + Shield (45 flights 27-28 Oct, no CSP, not a suite scenario, not ArduPilot); 7 Nov OpenVLA-7B with the CSP in its prompt (AirSimNH, not suite, not KPI-grade; WP2-01). Suite scenarios on hil with signed bundle + Shield + harness since 7 Nov, piloted by the stub (no CSP): latest ntFIN4 at 59e19a5 (653 KPI-grade, dynamic families incl. dyn-nfz-spawn-ahead with REST hot-apply). All four grant components in one flight, static cells only: pab1 (c7257f1, 16 Nov), pab2, pab3 (1239619, 19 Nov; docs/data/kpi_prefix_ab_20261119/) flew 5 cells of 4 static library templates (nfz-head-on, corridor-drift-out, altitude-ceiling-hold, altitude-floor-recovery, speed-cap) on hil, KPI-grade, with the signed bundle, the Prefix Compiler's CSP consumed by the grant's action-set adapter guardrail/prefix_consumer.py (a hand-written pilot, not a VLA), the Shield and the harness (experiments/profiles/prefix_ab.yaml). Learned VLA reading the CSP on hil: pab_aerialvla_04 (20 Nov, AerialVLA, PAS follow, outside the harness; docs/data/kpi_prefix_vla_20261120/). FINAL-REPORT.md 9.1 lists five demo scenarios, none with all four (decision 11, settled by the lab). | No dynamic suite scenario has flown with the Prefix Compiler in the loop: a CSP consumer arm of any scenario with a mid-flight event is refused because generation N's CSP reaching the pilot is not built (experiments/sweep_scenarios.py:2948 prefix_consumer_refusal, :2033; guardrail/prefix_consumer.py:117; guardrail/vla_prefix.py:88). The only four-component flights are static cells with a hand-written CSP consumer; no learned VLA has flown a suite scenario in the harness. Whether pab3's static cells count as 'non-trivial', and the five final-demo scenarios themselves, are for the PI to confirm (6 Oct Q2; decision 11 is the lab's). The gate is shown at the final delivery (2026-11-30), not yet held. |
| WP4-20 | Shield audit JSONL aggregated into the episode bundle by the harness at episode end (Safety Shield page, Audit log section) | DONE | guardrail/replay.py EPISODE_FILES audit.jsonl (shield_audit.jsonl since 2 Nov) packed per episode (CHANGELOG 2026-10-07); 30 Oct: the headless episode keeps the flying Shield's audit records (EpisodeRun.audit, AuditLogger.in_memory) and write_bundle packs them as shield_audit.jsonl at episode end, the sweep's and the gRPC EndEpisode's bundles alike (tests/test_sweep.py::test_the_episode_bundle_carries_the_shields_audit) | None. |
| WP4-21 | WP4 status as reported to the PI/ITRI versus the grant's WP4 outputs (scenario library, KPI rollups, replay harness; 7-row output table on the Stress Testing page) | DONE | WP4-built claim corrected (tools/check_claims.py; CORRECTION-2026-10-06 row A8); the correction was sent to ITRI (researcher, 2 Nov) | - |

### Architecture and topologies

31 items: 24 DONE, 6 PARTIAL, 1 DRIFT.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| ARCH-01 | VLA emits a 10 Hz setpoint; ArduPilot inner loop tracks it (canonical rail) | DONE | 10 Hz setpoints tracked by ArduPilot on SITL+MAVROS (7 Oct) and PAS+ArduPilot (loop 9.91 Hz on 21 Oct) 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the feed between ticks (--vla-feed stream; guardrail/vla_backends.py RemoteVLABackend.pump; docs/data/pas_hil_feed_20261115/flights.md): a new camera frame goes to the model without waiting for the 10 Hz tick, the answers still enter on ticks. Four KPI-grade hil flights of AIoT-VLA v10b on the AGX Orin with the feed uncapped and the camera at a 0.05 s capture interval (pas_hil_feed_05-08, d9f5ea0 / deac4b3): 12.39-13.23 distinct frames/s reached the model while it was asked (9.58 before, pas_hil_obs_02); over the whole mission 10.72 and 11.07 in the two flights that held the car for the full 420 s (pas_hil_feed_07 / _08), 3.48 and 8.86 in the two that lost it; decisions parsed 10.62 and 10.93 per second over those two missions; Shield ticks 9.96-9.99 Hz, 39-332 interventions per mission, 0 P0 escapes; the detector 2.31-2.70 Hz and the tick's work as without the feed. | Two full-length flights (pas_hil_feed_07 / _08). The model decides at 10 Hz or faster only while the detector has the target (12.4-13.2 decisions/s asked; 3.5 and 8.8 over the mission in the two flights that lost the car). An answer enters on the next tick, and an older one taken in the same tick is superseded unflown. |
| ARCH-02 | ArduPilot is the inner loop on every rail, including the perception (Project AirSim) rail | DONE | Every rail of the grant flies ArduPilot as the inner loop. KPI rail: ArduCopter 4.5.7 SITL + MAVROS 2 on `hil` (the KPI-grade nightly ntFIN2 + ntFIN2s, 589 episodes, 12-13 Nov). Functional rail: Gazebo Harmonic on ArduCopter JSON SITL, flown end to end 2 Nov over pymavlink (e74eae3) and over MAVROS 2 (0207af0) (ARCH-16). Perception rail: Project AirSim on ArduPilot SITL (G1 8 and 13 Oct; PAS+ArduPilot rail flights to 21 Oct; 29 Oct AerialVLA flew perception_node --pilot vla over pymavlink, pas_vla_goto_01/02, pas_vla_approach_01, every tick through the Shield and the FSM, 0 P0 escapes) and, since 14 Nov, a learned VLA (AIoT-VLA) on `hil` over MAVROS 2, KPI-grade, 0 P0 escapes (docs/data/pas_aiot_hil_20261114/flights.md, docs/data/pas_hil_paraphrase_20261115/flights.md, docs/data/pas_hil_feed_20261115/flights.md). Grant (03-simulation/dual-rail.md): "Both rails point at one SITL instance"; (01-context/grant-overview.md): "ArduPilot runs the inner loop at 100–400 Hz". Re-read against those lines on 16 Nov by a reader and an adversarial second reader: the same evidence already carries ARCH-18, ARCH-20 and ARCH-21 | Nothing on the grant's rails. The city follow comparisons (27-29 Oct, 4 Nov) and the 7 Nov CSP A/B flew Project AirSim's simple_flight: a comparison bench outside the delivered architecture, labelled so (section 6) and never KPI evidence. That the backend comparison rests on that bench is ARCH-30's gap, not this row's. |
| ARCH-03 | Real VLA backends produce the 4-D action at 10 Hz (and Orin latency gate G3) | DONE | AerialVLA bf16 p50 259-486 ms on the 4090 (docs/data/aerialvla_serving_4090.json); in flight decisions every 0.4-0.5 s, DroneCATS 1.0-1.45 s, held as a 10 Hz stream (guardrail/vla_link.py StepTracker); no VLA timed on an Orin (7B does not fit the Nano); 30 Oct: the lab's own AIOT-VLA (training/aiotvla, docs/DESIGN-aiotvla.md; named NTUT-VLA until 30 Oct) answers in 48.4 ms p50 on the Orin Nano Super (19.1 decisions/s over 600 replayed captures, the server alone) and 10.3 ms on the RTX 4090; on the city rail its requests go every 0.1 s (capture gap p50 0.13 s) 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the feed between ticks (--vla-feed stream; guardrail/vla_backends.py RemoteVLABackend.pump; docs/data/pas_hil_feed_20261115/flights.md): a new camera frame goes to the model without waiting for the 10 Hz tick, the answers still enter on ticks. Four KPI-grade hil flights of AIoT-VLA v10b on the AGX Orin with the feed uncapped and the camera at a 0.05 s capture interval (pas_hil_feed_05-08, d9f5ea0 / deac4b3): 12.39-13.23 distinct frames/s reached the model while it was asked (9.58 before, pas_hil_obs_02); over the whole mission 10.72 and 11.07 in the two flights that held the car for the full 420 s (pas_hil_feed_07 / _08), 3.48 and 8.86 in the two that lost it; decisions parsed 10.62 and 10.93 per second over those two missions; Shield ticks 9.96-9.99 Hz, 39-332 interventions per mission, 0 P0 escapes; the detector 2.31-2.70 Hz and the tick's work as without the feed. 17 Nov, gate G3 judged from the stored hil flights (tools/g3_gate.py --runs F:/vla_drone_offload/jetson_runs --write docs/data/g3_gate_20261117): AIoT-VLA v10b, the lab's small backend, with its server on the AGX Orin (metrics.json hil_evidence.vla_host) on three C2 hil flights (pas_hil_feed_07, pas_hil_feed_08, pas_hil_aiot_c2_01_20261116): round trip p95 84.7 / 84.2 / 84.0 ms (84.5 ms pooled over 10716 answers), 13.15-13.22 decisions/s while asked, 0 failed requests - inside the gate's 100 ms p95 and 10 Hz; the 7B backends measured fail it (OpenVLA-7B bf16 969 ms p95 on the AGX Orin at MAXN; AerialVLA and RaceVLA bf16 1.4-2.4 s round trip in flight), and the gate's action, the swap to a smaller backend, is the backend that passes. | int8 7B is not measured on the Orin (the gate's premise names int8); the 10 Hz rate needs the stream feed with a 0.05 s capture (tick-feed flights decided 6.3-9.6 times a second while asked); frame to answer p95 is 134-135 ms (not the gate's measure); AIoT-VLA is follow-only and is not one of the grant's named swap examples (BitVLA, TinyVLA). |
| ARCH-04 | 4-D action (vx, vy, vz, yaw_rate) in body frame; body-to-NED conversion lives in the MAVLink adapter | DONE | body frame through the Shield's own boundary on the ROS 2 rail (`Shield.filter_body`, 4 Nov), NED conversion only in sitl/mavlink_adapter_node.py; 18 Nov: the four dev rails hand over the body frame where that is exact to the bit (see WP3-04) | Met (independent review 20 Nov): the Shield's input contract is the body frame on every flying rail and loop (Shield.filter_body / frames.filter_world_as_body since 19 Nov), and the conversion to local NED lives only in the adapters (frames.body_to_local_ned), never in the operators; flown at measured headings on hil (pab3, pab_aerialvla_04) and on the pymavlink SITL rail at 135.6 deg (20 Nov, see WP3-04). The operator-frame clause belongs to WP3-04's wording. |
| ARCH-05 | VLA->Shield on Float32MultiArray; Shield->MAVROS on mavros_msgs/PositionTarget; MAVROS->ArduPilot over UDP 14550 | DONE | PositionTarget via adapter; MAVROS on UDP 14555 behind mavlink-router (14550 is Mission Planner's) | None. |
| ARCH-06 | Switchable backend: CognitiveDrone, OpenVLA generic, BitVLA, in-house stubs, all conforming to 4-D | PARTIAL | RemoteVLABackend (guardrail/vla_backends.py; it replaced the raising AeroVLABackend skeleton on 21 Oct, docstring l.31-36) + protocol v2, with StubBackend / PolicyBackend as the in-house slots: AerialVLA (OpenVLA-7B), DroneCATS-2B, RaceVLA-7B flown 27-28 Oct (tests/test_vla_foreign.py 48/48); sitl/ros2_vla_bridge_node.py built (ARCH-20). 16 Nov: AerialVLA bf16, DroneCATS-2B and RaceVLA-7B flown through the hil perception node by deploy/orin/run_pas_perception.sh FAMILY= (C2 of the backend table, P0 escapes 0 on every flight; deploy/evidence/vla_backend_table.md). 17 Nov: CognitiveDrone and BitVLA adapters in demo/vla_server_foreign.py behind protocol v2 (--model / FAMILY= cognitivedrone / bitvla), serve refuses without weights; scripted 4-D conformance (tests/test_vla_backends_cd_bitvla.py, seed 20261117, 500/500 finite Action4D per arm). 22 Nov (docs/data/bitvla_loader_20261122/README.md; CHANGELOG 2026-11-22 (code); commits 1c42f4a, d8b4268): BitVLA's real loader ran on the AGX Orin with BitVLA's own model class (github.com/ustcwhy/BitVLA @ a8d62df, bidirectional attention via --bitvla-code) on the downloaded LIBERO-Spatial arm checkpoint (hongyuw/ft-bitvla-bitsiglipL-224px-libero_spatial-bf16, MIT; user-approved download): load 5.94 s, 5,585 MiB GPU, median 1.479 s per answer (n=20); 5/5 frames -> finite Action4D, client strict reader agrees with the server; Shield.filter took all 15 frame/state cases (only alt-band acted on vz); CLI server + real RemoteVLA client over loopback TCP: 12 decisions in 20 s (0.60 Hz), 188 finite ticks (tcp_client.json); the 17 Nov loader had five faults and could not have run (fixed, tests/test_vla_backends_cd_bitvla.py 13/13). | CognitiveDrone: no weights published (checked 17 Nov; deploy/evidence/vla_backend_table.md:358), so its adapter has never loaded a model - scripted conformance only. BitVLA: only an arm (LIBERO) checkpoint exists, so its real output is a conformance check, not a flight policy; never flown, no hil campaign; a[0] < 0 on all five frames, so the 4-D slot flew forward as 0 every time (no backward step in the codec) and the no-fly square never acted; 0.6-0.7 Hz in PyTorch eager on the Orin, far below the 10 Hz loop (robot.cpp not tried); tools/vla_backend_table.py still lists bitvla as unavailable and says nothing of the AGX load. The 4-D slot flies a backward vx as 0 and caps the turn at 1.1 rad per horizon. |
| ARCH-07 | MAVROS 2 is the canonical bridge for Year 1 | DONE | MAVROS 2 runs on the dev rail (7 Oct); tests/test_manifest.py 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). | The KPI campaigns' pilot is the stub; the learned VLA's flights over MAVROS 2 are the perception rail's six missions of 14 Nov. |
| ARCH-08 | AP_DDS migration (Q3, optional) | PARTIAL | 17 Nov (b316a05, review fixes 791d756): an opt-in AP_DDS path - sitl/ap_dds/setup_ap_dds.sh builds ArduCopter 4.5.7 with --enable-DDS in a separate checkout (~/ardupilot-dds), start_ap_dds_sitl.sh starts it with the micro-ROS agent (domain 42), verify_ap_dds.py checks /ap/time, /ap/pose/filtered, /ap/navsat, /ap/geopose/filtered and a velocity through /ap/cmd_vel in GUIDED (writes ap_dds_verify.json); sitl/ap_dds_adapter_node.py sends the Shield's /shield/setpoint to /ap/cmd_vel with the MAVLink adapter's rotation; docs/RUNBOOK-ap-dds.md (steps 0-6, row-status table l.18-30); tests/test_ap_dds_adapter.py (a model-consistency sweep, 0 by construction, not evidence; run in CI, .github/workflows/tests.yml l.145-148). Nothing on the KPI rail launches it; no AP_DDS run or ap_dds_verify.json is committed (no docs/data entry); nothing in the 20-22 Nov changes touches it. | Not yet run on SITL (runbook steps 4-6 never executed, no ap_dds_verify.json); the KPI rail and the Shield's state inputs stay on MAVROS 2 (all ntFIN4 653 KPI-grade runs). Per the runbook, the earliest DONE candidate is step 6 (the Shield flying a scenario through ap_dds_adapter_node on the DDS SITL), or a recorded lab/PI decision that this optional row closes at PARTIAL. |
| ARCH-09 | ArduPilot GeoFence/FENCE_ACTION as ultimate backstop; MAVLink escalation to RTL/Land/Loiter | DONE | GeoFence backstop configured and shown firing; FSM drives RTL and Land via SET_MODE; its Loiter was sent as BRAKE until 7 Nov and is held in GUIDED by the companion since (see WP3-07, lab decision 7, PI confirmation pending; WP3-10) | None. |
| ARCH-10 | Mission Planner via mavlink-router fan-out, parallel to MAVROS | DONE | mavlink-router fans out to UDP 14550, 19.0 Hz measured by sitl/gcs_listen.py (CHANGELOG 2026-10-08); 15 Nov, recorded on a hil flight: Mission Planner 1.3.83 on the desktop connected by itself to the router's UDP 14550 fan-out (its AutoConnect entry; title bar 'ArduCopter V4.5.7 (2a3dc4b7) on UDP14550-1-QUADROTOR') while MAVROS 2 on the AGX Orin flew pas_hil_para_11 (Project AirSim + ArduPilot SITL, AIoT-VLA and the Shield on the Orin); screenshots before arming and in flight (Guided, 7.99 m, 2.81 m/s): docs/data/arch10_20261115/ (CHANGELOG 2026-11-15 (Mission Planner; paraphrase retries)) | Screenshots, not a recording; Mission Planner only watched (no command was sent from it). |
| ARCH-11 | dev topology: everything on one desktop | DONE | dev flown in containers 2026-11-06: deploy/compose/docker-compose.dev.yml with the images built on the desktop (vlaguard/sitl:copter-4.5.7 ArduCopter 4.5.7, vlaguard/companion:jazzy: router, MAVROS 2, mission), dev_compose_01 (code 30667cc clean, topology dev, P0 escape 0.0, refused as KPI evidence without the PI's waiver, as the rule says; deploy/evidence/hil/images.txt); the native WSL dev rail since 7 Oct; the Gazebo Harmonic functional rail flown end-to-end on 2026-11-02 over pymavlink and MAVROS 2 (its Shield-on mission is ARCH-16's item) | - |
| ARCH-12 | hil topology: desktop runs sims + MAVROS + GCS, Jetson Orin runs VLA + Shield (canonical KPI config) | DONE | hil flown 2026-11-06: SITL + mavlink-router on the desktop, MAVROS 2 + the Shield + the stub pilot on the AGX Orin per docs/RUNBOOK-orin-hil.md (companion image built on the Orin, hil.env with the cable's addresses); hil_smoke_03 KPI-grade (CHANGELOG 2026-11-06 (hil)); the VLA on the Orin is ARCH-20's item; the KPI campaign is WP4-01's | - |
| ARCH-13 | Topology field and the claim about which configuration yields contractual KPIs | DONE | labels dev/hil/flight, hil and flight only on evidence (CHANGELOG 2026-10-06/-07); 2026-11-06 the first run labelled hil from evidence (hil_smoke_03), accepted by is_kpi_grade and by tools/kpi_report.py (1 of 2 KPI-grade; hil_smoke_02 hil but refused: unsigned bundle); the 28 Oct Orin Nano manifests stay single-host/unversioned and are counted as functional evidence; tools/check_claims.py | - |
| ARCH-14 | flight topology: Orin companion with real ArduPilot autopilot; final demos | PARTIAL | deploy/topologies/flight.env + compose override defined (CHANGELOG 2026-10-07); no drone. 17 Nov bench (no flight; docs/data/fc_bench_20261117/README.md): on the real flight controller (CUAV V5 Nano, fmuv5, ArduCopter 4.7.1, on the desk over USB, no GPS fix, never armed) sitl/gcs_backstop.py set and read back the FENCE_* parameters and uploaded the bundle's keep-out nfz-square as 4 exclusion items, read back matching (gcs_backstop.json); MAV_CMD_DO_SET_MODE accepted for GUIDED, BRAKE, LOITER, RTL, LAND, STABILIZE with the next heartbeat confirming each; parameters, fence and mission restored (fc_verify.json). All KPI evidence is hil (AGX Orin companion + ArduCopter 4.5.7 SITL): ntFIN4 at 59e19a5, 653 KPI-grade (docs/data/kpi_ntFIN4/README.md). 22 Nov: the user decided not to flash the real flight controller (the user's decision, recorded in docs/FINAL-REPORT.md section 9). | No flight topology run and no final demo on a real airframe: decided 2 Nov that the work stops at hil (the drone cannot carry an AGX Orin or run a large VLA); the PI's written waiver for flight is still needed (is_kpi_grade refuses a flight/dev run without it, guardrail/manifest.py DEV_KPI_WAIVER). The FC stays on ArduCopter 4.7.1 against the rig's 4.5.7 (not flashed, user decision 22 Nov), so the hil firmware has never run on the real autopilot. |
| ARCH-15 | HIL bridge adapter (Q1, CRITICAL) and its 2026-06-30 fallback gate G1 | DRIFT | the Project AirSim - ArduPilot bridge closes the loop (demo/pas_ardupilot/, scripts/run_pas_ardupilot.ps1; ArduPilot launched as --model airsim-copter, demo/pas_ardupilot/rail.py l.610-636); gate G1's check demo/pas_ardupilot/g1_check.py passed all four of its conditions on three runs each time (demo/out/g1_20261008_223259/g1_verdict.md, g1_20261013_203404); the perception rail is kept on Project AirSim and carried the hil VLA flights (ARCH-20). The 22 Nov GPS-noise / latency cells (docs/data/shakedown_20261122/README.md, sk1122a 60 cells; ntFIN4) are applied companion-side by the ROS 2 Shield node to the pose the Shield and FSM read (CHANGELOG 2026-11-22 (code), WP3-19 / WP4-14), not through HIL_GPS, so they do not change this row. | Built differently from the grant's words: the bridge carries Project AirSim's JSON sensor frames to ArduPilot's SIM_AirSim backend (UDP 9003, PWM back over UDP 9002), where G1's condition 1 names "AirSim sensor messages (`HIL_GPS`, `HIL_SENSOR`)" (04-risks-and-fallbacks/fallback-gates.md). The gate's swap rule ("swap the perception rail to Colosseum, freeze Project AirSim work") was not applied: the Colosseum swap was not made. The PI's reference implementation records the perception rail as "deferred per the 2026-06-30 fallback gate" (sim/README.md at 9e039b0, docs/data/midterm_gate_evidence/evidence.md). A recorded deviation (section 7; docs/FINAL-REPORT.md section 7.1); needs the PI's acceptance to close. |
| ARCH-16 | Functional rail (Gazebo Harmonic) demoable end-to-end at mid-term | DONE | flown end-to-end 2026-11-02 at e74eae3 (WSL Ubuntu 24.04, Gazebo Sim 8.14, ArduCopter 4.5.7 JSON SITL, pymavlink): demo/out/gazebo_shield_on_20261102, _off_20261102, _on_dynamic_20261102 (Shield on P0 escape 0.0, off 0.584), and over MAVROS 2 in the dev topology at 0207af0 (PHYSICS=gazebo sitl/run_ros2_demo.sh): gazebo_ros2_shield_on_20261102 P0 0.0, gazebo_ros2_shield_off_20261102 P0 1.0 with the GeoFence firing; each with a flight-log replay video (tools/render_track_video.py) (CHANGELOG 2026-11-02 evening) | One functional sweep flown on it, 20 Nov at 28a594d (8 missions, one seed, 7 of 8 passed; nfz-head-on failed its fail-safe gate; WP4-04, docs/data/gazebo_sweep_20261120/); no multi-seed sweep; with the Shield on, the 2 Nov mission does not reach its target (theta escalates on the first tick, Loiter times out, RTL), the false trigger WP3-15 addresses. |
| ARCH-17 | Mid-term delivery 2026-07-20 (DSL+IR, Shield ROS 2 node with one projection operator, Gazebo rail) | DONE | the three parts are delivered: the Policy DSL + IR (WP1-21), the ROS 2 Shield node with its projection operators (WP3-01) and the Gazebo rail flown end to end (ARCH-16); the mid-term report docs/MIDTERM-REPORT-Aug2026.md, delivered 20 Aug 2026 as the 19 August review asked (docs/MEETING-PACK-Sept2026.md Part A1, item 1) | None. |
| ARCH-18 | Perception-rail integration (final delivery 2026-11-30) | DONE | PAS+ArduPilot perception node flown with detector pilot + Shield + FSM (8-21 Oct); start gate and --pilot vla built 22-23 Oct, 'none flown'; --link mavros never flown; detector boxed the wrong object on the close flights; 29 Oct: AerialVLA flew perception_node --pilot vla on ArduPilot SITL over pymavlink (pas_vla_goto_01/02, pas_vla_approach_01), every tick through the Shield and the FSM, 0 P0 escapes 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the target estimator (pas_hil_aiot_est_01 at 3918965: --identity --negative-query 'a traffic light' --target-estimator, demo/subject_track.py): KPI-grade, 420 s, 0 P0 escape ticks, 266 interventions; the estimate within 6 m of the car on 0.907 of ticks (0.971 of its own), 5 lapses and 5 re-acquisitions; aircraft-to-car gap median 34.4 m, p90 42.6 m against 112.9 / 166.1 m without it (pas_hil_aiot_kpi_01). | One KPI-grade mission each without and with the target estimator; the estimator is opt-in (--target-estimator) and its gain comes from the strict lock (AIoT-VLA does not read the compass phrase). |
| ARCH-19 | Final integration + KPI runs (Q4) in the canonical hil topology; KPI report on dynamic-scenario stress | PARTIAL | Harness, KPI report and manifests on hil since 7 Nov (smokes rh03, sm1110, sm1111; ge1110; hs04-hs06b rotate-spin through MAVROS; CHANGELOG 2026-11-11 (hil rig flights)). KPI runs, all KPI-grade nightlies on the AGX Orin rig: nt1111 (22d434b), ntFIN2 + ntFIN2s (116214f), ntFIN3 (71aabb3, 21 Nov, 594 KPI-grade; docs/data/kpi_ntFIN3/), ntFIN4 (59e19a5, 22-23 Nov, 653 KPI-grade, P0 0, (r2) 648/648 Wilson low 0.994; docs/data/kpi_ntFIN4/), with the smokes smfin1120 (71aabb3) and smFIN2 (59e19a5) before each and the shakedowns sk1122a/b (docs/data/shakedown_20261122/). ntFIN4 flew the eleven 22 Nov changes integrated (retreat ray, body-frame operators, six-name bundles, manifest frame in bags 72/72, GPS noise on the rail, single_sha 653/653: companion and desktop 59e19a5e3b19, ArduCopter V4.5.7 2a3dc4b7); flight code unchanged from 59e19a5 to 529e787 (git diff: only tools/replay_at_revision.py, tests, CI). KPI report committed (WP4-17); a learned VLA has flown the hil rail (ARCH-20 DONE). | Remains the delivery of the KPI report with the signed final report (X-01; draft v0.5, unsigned; its figure of record is ntFIN4 since 23 Nov). Final integration in the grant's hil sense is not shown in the KPI runs: they fly the text-blind stub where the Orin should run "VLA + Shield", and no run carries all four components with a dynamic suite scenario (generation N's CSP to the pilot is not built; WP4-19). The post-ntFIN4 tick fix named in docs/data/kpi_ntFIN4/README.md ('CHANGELOG 2026-11-23') is not in the tree at 529e787; if merged, the delivered code again moves past the KPI revision unflown. |
| ARCH-20 | A real VLA constrained end-to-end in the canonical KPI topology | DONE | VLA slots built on both ArduPilot rails: perception_node.py --pilot vla, sitl/ros2_vla_bridge_node.py (CHANGELOG 2026-10-23 later, 'none flown'); VLAs flown only on simple_flight; 29 Oct: AerialVLA flew perception_node --pilot vla on ArduPilot SITL over pymavlink (pas_vla_goto_01/02, pas_vla_approach_01), every tick through the Shield and the FSM, 0 P0 escapes 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the target estimator (pas_hil_aiot_est_01 at 3918965: --identity --negative-query 'a traffic light' --target-estimator, demo/subject_track.py): KPI-grade, 420 s, 0 P0 escape ticks, 266 interventions; the estimate within 6 m of the car on 0.907 of ticks (0.971 of its own), 5 lapses and 5 re-acquisitions; aircraft-to-car gap median 34.4 m, p90 42.6 m against 112.9 / 166.1 m without it (pas_hil_aiot_kpi_01). | One KPI-grade mission (pas_hil_aiot_kpi_01: 420 s, 119 interventions, P0 0); its follow quality is weak (target lock 0.06); the VLA decides at 7.7 Hz (ARCH-03). |
| ARCH-21 | Constrained-VLA evidence on the perception rail and in current demos | DONE | VLA-piloted flights behind the Shield with the Shield acting: vla_follow_redcar_01_20261023 (177 corrected ticks), nfz3b_pc_hier_dronecats / nfz3b_orin_hier_aerovla 28 Oct (13 / 257 repairs, 0 zone ticks, 0 P0 escapes) 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the feed between ticks (--vla-feed stream; guardrail/vla_backends.py RemoteVLABackend.pump; docs/data/pas_hil_feed_20261115/flights.md): a new camera frame goes to the model without waiting for the 10 Hz tick, the answers still enter on ticks. Four KPI-grade hil flights of AIoT-VLA v10b on the AGX Orin with the feed uncapped and the camera at a 0.05 s capture interval (pas_hil_feed_05-08, d9f5ea0 / deac4b3): 12.39-13.23 distinct frames/s reached the model while it was asked (9.58 before, pas_hil_obs_02); over the whole mission 10.72 and 11.07 in the two flights that held the car for the full 420 s (pas_hil_feed_07 / _08), 3.48 and 8.86 in the two that lost it; decisions parsed 10.62 and 10.93 per second over those two missions; Shield ticks 9.96-9.99 Hz, 39-332 interventions per mission, 0 P0 escapes; the detector 2.31-2.70 Hz and the tick's work as without the feed. | n = 2 full-length flights with the model deciding at 10 Hz or faster (ARCH-01's note); the follow on this rail still depends on the detector holding the car (2 of the 4 uncapped flights lost it). |
| ARCH-22 | rosbag2 capture for the determinism and replay contract (L3) | PARTIAL | rosbag2 is recorded on the ROS 2 rails (sitl/run_ros2_demo.sh; deploy entrypoint; tests/test_ros2_package.py, tests/test_deploy_configs.py). Every KPI-grade hil run carries one. For example, demo/out/hil_smoke/ntFIN4_236_dyn-nfz-spawn-ahead/bag/metadata.yaml has 24 topics: MAVROS state, pose, velocity, IMU, global/rel-alt, battery, home, extended state, statustext and setpoints; raw MAVLink both ways (/uas1/mavlink_source 28,861 msgs, /uas1/mavlink_sink 528); /vla/action_4d, /shield/setpoint, /shield/intercept, /shield/fences, /shield/mode_request; /mavlink_adapter/sent; and /guardrail/manifest. Since 22 Nov the Shield node publishes /guardrail/manifest latched (reliable, transient_local) at episode start and end with the six manifest fields, and both recorders subscribe to it first (CHANGELOG 2026-11-22 (code); sitl/bag_qos_overrides.yaml). All 654 ntFIN4 bags hold its 2 frames (demo/out/hil_ntFIN4/episode-*/episode.bag2/metadata.yaml). tools/replay_from_bag.py over sk1122a (docs/data/shakedown_20261122/sk1122a_replay_from_bag.txt): manifest_in_bag identical 72/72, trajectory identical 72/72, ticks and KPI identical 48/72. The 24 gps-noise-beyond-margin runs differ by one setpoint and one tick of "who flew it" at the escalation, which the bag cannot say. Earlier: replay_from_bag over ntFIN2+ntFIN2s read 590 identical and 1 differing (17 Nov). The exporter reads the bag (WP4-13). | No image topic in any KPI-grade bag. /camera/front and /camera/down are in the record list, but nothing on the hil/SITL rail publishes them, so the grant's "rosbag2 image + state topics" is not captured. On the stub-pilot hil campaigns, 0 of 654 ntFIN4 bags hold a camera topic. The KPI replay of record (tools/replay_at_revision.py) runs from flight_log, not the bag. Bag-based replay at 59e19a5 was run only on sk1122a: ticks and KPI identical on 48/72, because the bag carries no topic saying which side flew a tick. It was not run over ntFIN4. The custom-data manifest metadata (`manifest_in_the_bag`) stays off by default. |
| ARCH-23 | Q3 paraphraser deliverable (Aug-Oct) | DONE | same artefact as WP2-13: the stored paraphrase sets, validator and CLI (guardrail/paraphraser.py) and the separate service the grant names (guardrail/paraphraser_service.py, gRPC with mutual TLS, guardrail/protos/paraphraser.proto), with the 4 C8 sets re-authored and the validator's held-out misses fixed | The orchestrator binding is WP4-10 (DONE) and the per-paraphrase measurement is WP4-09 (DONE). |
| ARCH-24 | 1 PI + 4 Master students (2 ArduPilot/MAVLink, 2 VLA fine-tuning/prompting) | DONE | team as in the grant: 1 PI + 4 Master students (one team); the grant's Personnel line, 01-context/grant-overview.md: "1 PI + 4 Master students (two on ArduPilot/MAVLink, two on VLA fine-tuning / prompt engineering)" | None. |
| ARCH-25 | Observation (Camera / IMU / GPS) reaches the VLA at >=10 Hz (Architecture constraints diagram; ref layer-requirements L2/L5 'Surface obs at >=10 Hz') | DONE | detector 2.18-2.48 Hz on the Jetson flights, ~3 Hz on the PC; VLA frames at its decision rate (~2 Hz); 30 Oct: the lab's own AIOT-VLA (training/aiotvla, docs/DESIGN-aiotvla.md; named NTUT-VLA until 30 Oct) answers in 48.4 ms p50 on the Orin Nano Super (19.1 decisions/s over 600 replayed captures, the server alone) and 10.3 ms on the RTX 4090; on the city rail its requests go every 0.1 s (capture gap p50 0.13 s) 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the feed between ticks (--vla-feed stream; guardrail/vla_backends.py RemoteVLABackend.pump; docs/data/pas_hil_feed_20261115/flights.md): a new camera frame goes to the model without waiting for the 10 Hz tick, the answers still enter on ticks. Four KPI-grade hil flights of AIoT-VLA v10b on the AGX Orin with the feed uncapped and the camera at a 0.05 s capture interval (pas_hil_feed_05-08, d9f5ea0 / deac4b3): 12.39-13.23 distinct frames/s reached the model while it was asked (9.58 before, pas_hil_obs_02); over the whole mission 10.72 and 11.07 in the two flights that held the car for the full 420 s (pas_hil_feed_07 / _08), 3.48 and 8.86 in the two that lost it; decisions parsed 10.62 and 10.93 per second over those two missions; Shield ticks 9.96-9.99 Hz, 39-332 interventions per mission, 0 P0 escapes; the detector 2.31-2.70 Hz and the tick's work as without the feed. | As X-05: with a 15 Hz camera, and while the model is asked. |
| ARCH-26 | 10 Hz Shield monitor budget validated by Orin profiling; Rust hot-path as Q3 fallback (Overview page, locked cross-cutting decisions) | DONE | Profiled on the AGX Orin 2026-11-06 (tools/profile_shield_tick.py, 1,301 real ticks, 50 rules x 51 poses, MAXN, clocks not locked): query p99 0.977 ms / 5 ms, Shield step p99 22.647 ms / 100 ms, 0 ticks over either (deploy/evidence/hil/shield_tick_profile.json). Rust decision: no Rust hot path - the Python Shield uses under a fifth of the query budget and a quarter of the tick budget at the grant's design load | - |
| ARCH-27 | Implementation language Python + Pydantic v2 throughout (Overview page, locked cross-cutting decision) | DONE | Python + Pydantic v2 throughout (guardrail/, pyproject.toml) | 3.11+ version is X-06. |
| ARCH-28 | The same code base runs in all three topologies, all first-class (grant); reference topologies.md: identical Docker image, topology selected by config flag, a 'hard requirement' | DONE | one Dockerfile per role, built natively on both hosts from the same commit (2026-11-06): vlaguard/companion:jazzy arm64 on the AGX Orin and amd64 on the desktop, the same Ubuntu 24.04, ROS 2 Jazzy, Python 3.12.3, MAVROS 2.16.0, mavlink-router v4 and pip set with numpy pinned to 1.26.4 (deploy/evidence/hil/companion_versions.txt, deploy/evidence/dev/companion_versions_desktop.txt, deploy/evidence/hil/images.txt); flown in dev (dev_compose_01) and hil (hil_smoke_03) with the topology chosen by the env file, not a rebuild. flight is the same image with flight.env and the serial-device override (tests/test_deploy_configs.py); flying it waits on ARCH-14's waiver | - |
| ARCH-29 | Shield loads the Policy IR / signed Bundle at runtime (Architecture constraints diagram: 'Policy IR / Bundle -> loaded -> Shield'); overlaps WP1 | DONE | Shield loads the signed bundle on every rail incl. the city rail since 2026-10-22 (demo/follow_vlm.py:122-193); Jetson metrics policy_source kind=bundle, declared=loaded hash | None. |
| ARCH-30 | No default backend until the 'Q2 measurement set lands' (Architecture constraints, Action space); basis for any backend choice and for the G3 candidate | DONE | deploy/evidence/vla_backend_table.md (Default backend: none; Orin columns empty; regenerated 21-22 Oct); 42-flight PC+Jetson comparison of 7 VLA configs + hand-written follow (CHANGELOG 2026-10-28) not folded into the table 18 Nov: the lab named AIoT-VLA v10b the default for the follow task after the set (`python tools/vla_backend_table.py table --runs F:/vla_drone_offload/jetson_runs --default aiotvla_v10b --default-scope "the follow task" --default-date 2026-11-18`; the tool refuses the line unless every backend that can fly C2 has an Orin timing and 3 C2 flights, the stub has its R1 on the Orin, and the date is after the set's newest measurement, 2026-11-17); deploy/evidence/vla_backend_table.md now reads 'Default backend: AIoT-VLA v10b ... for the follow task, named by the lab on 2026-11-18' with its own C2 numbers (within 30 m 0.211-0.352, KPI outcome 3 fail on unsafe_position_entered, P0 escapes 0). | None for the grant's line (no default until the set lands: the set landed 16-17 Nov, the default was named 18 Nov). Recorded beside it: the default covers the follow task only (no backend has flown C1, and AIoT-VLA reads no CSP); its C2 score is weak (27-35 % within 30 m); the estimator null (FAMILY=estimator) is not flown; R1 does not cover the four backends served over protocol v2 (their Orin timing is a server bench). |
| ARCH-31 | ArduPilot inner loop 'Bit-identical between SITL and firmware' (VLA/autopilot interface table) together with code_revision 'across the entire stack' (ref determinism contract) | PARTIAL | ArduPilot pinned Copter-4.5.7 and recorded (sitl/setup_sitl.sh); since 28 Oct the Jetson is a git checkout (deploy/orin/sync_follow_controller.ps1 ships a git bundle); a remote simulator is stamped projectairsim-remote-sim (guardrail/manifest.py). Since 17 Nov stack_revisions.json (schema vlaguard-stack-revisions/1, guardrail/manifest.py l.1633-1700, 1949-1974) is written beside each manifest by the ROS 2 Shield and mission nodes and the PAS node: each host's revision, the autopilot firmware as it names itself, the simulator build and single_sha; 22 Nov tools/kpi_report.py reports it per campaign (c072a38; tests/test_kpi_report.py, tests/test_stack_revisions.py). Every hil KPI campaign since: smfin1120 49/49, ntFIN3 594/594 (companion and desktop 71aabb34f8dc), smFIN2 52/52 and ntFIN4 653/653 KPI-grade runs single_sha true, 0 false / pending / missing, companion and desktop 59e19a5e3b19, autopilot ArduCopter V4.5.7 (2a3dc4b7) (docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md l.64, l.936; docs/data/kpi_ntFIN4/README.md l.47; kpi_smFIN2 README l.9). | Bit-identity between SITL and flight firmware is not shown and cannot be without a flight on the real FC (ARCH-14's waiver): the only real FC (CUAV V5 Nano) runs ArduCopter 4.7.1 against the rig's 4.5.7 SITL (docs/data/fc_bench_20261117/README.md), and the user decided on 22 Nov not to flash it. single_sha is report-only: is_kpi_grade does not read stack_revisions.json, so a mismatch is recorded, not enforced; the manifest itself still holds one code_revision field; the Project AirSim server binary's build is not asked for by any rail (manifest.py projectairsim_simulator, server_build_why); runs before 17 Nov have no record and the section 6 follow comparisons (27-29 Oct) keep '-dirty' / 'unversioned'. |

### Cross-cutting locked choices

16 items: 12 DONE, 4 PARTIAL.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| X-01 | Final delivery: signed final report (2026-11-30 gate) | PARTIAL | docs/FINAL-REPORT.md, draft v0.5 of 2026-11-23, status line "DRAFT, not signed": body, KPI table (sections 2 and 5) from the KPI-grade nightly ntFIN4 at 59e19a5 (653 KPI-grade, P0 escape 0 on 17,078 ticks, fail-safe (r2) 648/648 Wilson low 0.994, 1 of 183,909 ticks at 105.5 ms; docs/data/kpi_ntFIN4/) with ntFIN3 at 71aabb3 beside it (docs/data/kpi_ntFIN3/) and the smoke smFIN2 (docs/data/kpi_smFIN2/); decisions table (section 9); Appendix A (109/26/1/0) after the 23 Nov recount; tests/test_final_report_counts.py. Build tooling exists (tools/report_to_html.py, tools/html_to_docx_pdf.ps1). | Not signed, and no DOCX/PDF built (no tracked build output and no signature block). The signer and the signature belong to the PI; the report's signer is not named (bundle CA likewise unnamed, 22 Nov user decision to wait for the PI). GitHub Actions 'tests' was red on 59e19a5 and 0daa20e (a job timeout, then Linux-only test issues, fixed in tests and CI config on docs/1123, not pushed). The tick fix that follows ntFIN4's 105.5 ms tick is not committed, so the delivered revision is 59e19a5 unless it is flown before the delivery. Due 2026-11-30. |
| X-02 | Safety Shield acceptance KPI: repair success rate | DONE | same as WP3-20: KPI-grade nightly ntFIN2 (13 Nov) 98.2 % (26,473 of 26,968 repair ticks, 577 Shield-on missions), always-brake null 0 % (guardrail/kpi.py repair_success; theta basis decided 2 Nov). Grant (02-implementation/safety-shield.md): "Repair success rate \| tracked; informs Prefix Compiler effectiveness eval" | Same as WP3-20 (theta judges about a third of the converged ticks). |
| X-03 | Stress Testing locked KPI: average repair count per episode (as a KPI, not only a report column) | DONE | Grant (02-implementation/stress-testing.md): "Average repair count / episode \| tracked \| Counter from Shield repair log". mean repair count per episode and family in tools/kpi_report.py / docs/data/kpi_rollup_2026-10-06.md; on `hil` since 7 Nov, KPI-grade, repairs per episode: rh03 72.1, sm1110 76.1, sm1111 72.1, ge1110 0 (CHANGELOG 2026-11-11 (hil rig flights)); 13 Nov, the KPI-grade nightly ntFIN2 (116214f; tools/kpi_report.py --campaign): 55.4 per Shield-on mission over 577, per template 1.0 to 300.7 and per cell, a row of the KPI summary | None for the tracked KPI. |
| X-04 | Q2 VLA-backend measurement set: the comparison that must exist before any backend becomes the default | DONE | same as ARCH-30 | Same as ARCH-30. |
| X-05 | Observation rate into the VLA: camera/IMU/GPS at 10 Hz or faster | DONE | perception to the controller at 2.2-3 Hz (Jetson det_hz 2.18-2.48; PC ~3 Hz) against the grant's >=10 Hz observation rate 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the feed between ticks (--vla-feed stream; guardrail/vla_backends.py RemoteVLABackend.pump; docs/data/pas_hil_feed_20261115/flights.md): a new camera frame goes to the model without waiting for the 10 Hz tick, the answers still enter on ticks. Four KPI-grade hil flights of AIoT-VLA v10b on the AGX Orin with the feed uncapped and the camera at a 0.05 s capture interval (pas_hil_feed_05-08, d9f5ea0 / deac4b3): 12.39-13.23 distinct frames/s reached the model while it was asked (9.58 before, pas_hil_obs_02); over the whole mission 10.72 and 11.07 in the two flights that held the car for the full 420 s (pas_hil_feed_07 / _08), 3.48 and 8.86 in the two that lost it; decisions parsed 10.62 and 10.93 per second over those two missions; Shield ticks 9.96-9.99 Hz, 39-332 interventions per mission, 0 P0 escapes; the detector 2.31-2.70 Hz and the tick's work as without the feed. | Holds with a 15 Hz camera (capture interval 0.05 s; at 0.066 s it delivers 12.8 Hz and the feed was not flown) and while the model is asked: with no detector box it is not asked, by design, and the detector that gives the hint's phrase runs at 2.3-2.7 Hz. A request carries the frame and the EKF pose at its arrival (this rail's frames have no simulator stamp). n = 4 flights. |
| X-06 | Locked implementation language: Python 3.11+ (Policy DSL, Prefix Compiler, Safety Shield and Stress Testing pages) | DONE | same as WP1-25: the hil companion image runs 3.12.3 (ARCH-28); the desktop env vla-real is 3.10.20 14 Nov: the PAS launcher's scene steps ran on guardrail-py312 for the AIoT-VLA hil missions (scripts/run_pas_ardupilot.ps1 at f77efae). | The VLA runtime (torch, transformers) stays on vla-real 3.10, outside the grant's 3.11+ decision for the non-VLA stack; the PI to confirm that reading (decision 8). |
| X-07 | Validate the Shield's 10 Hz monitor budget by Orin profiling, with a Rust hot path as the Q3 fallback | DONE | same as ARCH-26: Orin profile within budget, Rust hot path not needed (decision recorded 2026-11-06) | - |
| X-08 | One code base runs first-class in dev, hil and flight; promotion dev to hil to flight is a config flag (identical image) | DONE | same as ARCH-28: one image definition, dev and hil flown with it, flight a config change (ARCH-14) | - |
| X-09 | R6 freeze gate: IR shape frozen at the end of Q1 (2026-04-30); every later IR-shape change carries a semver bump and a migration note | DONE | same as WP1-22 (docs/IR-MIGRATIONS.md, the freeze rule in check_lock) | - |
| X-10 | Bundle identity fields: policy_hash as a full SHA-256, signed_by (Lab or ITRI CA), changelog as a diff against the previous version, issued_at, layers_merged | PARTIAL | guardrail/bundle.py manifest_for (~l.1628-1663): policy_id, version, generation, full 64-hex policy_hash + hashes, ir_schema_version, signed_by, changelog, issued_at; layers_merged in the IR of merged policies (guardrail/models.py:1558); `changelog_diff` + diff_text when written with `previous` (895b5a0, 29 Oct; CLI --previous; tests/test_bundle.py:183, 243, 270); CA `signer` record since 16 Nov (tests/test_bundle.py:970, 1035, 1131, 1263, 1562). Flown: bundles/fase3-sim-demo-v0.1.4.tar.gz (ntFIN4, 59e19a5) manifest has policy_id fase3-sim-demo, version 0.1.4, generation 0, policy_hash sha256:b665adba..., issued_at, signed_by lab-dev:ed25519:... (the Orin's copy 715f70c4647a9701, verified, chain none). tests/test_bundle.py 72/72 at 8e73f86 (docs/data/ci_local/8e73f86b6af6.json). | (1) signed_by is a bare lab-dev key, not a "Lab CA or ITRI CA reference". The CA is unnamed (PI/ITRI, 12 Oct Q5), and the user decided on 22 Nov to wait (nothing certified or re-signed). (2) The changelog of the bundles that are actually delivered and flown is not a diff. tools/build_flight_bundles.py:163 calls write_bundle without `previous`, so fase3-sim-demo-v0.1.4 (flown in ntFIN4) carries changelog "flight bundle of sim_demo_policy.yaml" and no changelog_diff. Of the 48 policy bundles in the lab checkout's bundles/, 0 carry changelog_diff. Fix: pass the previous version's bundle in build_one and rebuild. Item (2) does not need the PI. |
| X-11 | MSL as an opt-in altitude reference on geometry primitives (altitude_ref AGL/MSL) | PARTIAL | altitude_ref MSL is a field on PolygonFence (CircleFence inherits it), AltitudeEnvelope, Corridor, DynamicNFZ and CorridorSwap (guardrail/models.py:969, 1084, 1238, 1366, 1441). It is opt-in: unenforced_rules refuses MSL unless the caller declares a ground and checks the DEM's WGS84 origin (models.py:2178-2200; terrain.FlatGround, GridGround DEM, load_ground); the compiler takes MSL over a ground (compiler.py:2382 _msl_view). The Shield converts at the query point and the forecast stands over the DEM (wf20; tests/test_terrain_forecast.py 10/10). Headless: tools/msl_dem_rail.py -> docs/data/msl_dem_rail.json (independent oracle, P0 0 against the nulls) and tools/msl_dem_kpi.py -> docs/data/msl_dem_kpi.json. Without a ground, 141/141 library episodes are byte-identical. The ROS 2 / SITL-hil rail readers take a declared ground named by path and sha256 (wf21-msl-readers, CHANGELOG 2026-11-19 (wf21, code)): the Shield node, sitl/hil harness backends, mission node, gcs_backstop, bundle_origin, VLA stub and bridge, build_flight_bundles (--ground at sitl/ros2_shield_node.py:6184, sitl/gcs_backstop.py:477, ros2_mission_node.py:937). The RTL leg is judged against the ground along it. FENCE_ALT_MAX is capped by the MSL ceiling: msl_dem_sitl 40 m, corridor 26 m. The replay tarball carries the signed DEM. ground_flight_problems refuses a run whose metrics name no ground or another one. Tests msl_readers 20/20, msl_rail 21/21, msl_dem 13/13 in lab CI 8e73f86 (22 Nov, 187/187 files, 4854 of 4870 tests, 16 skipped). Flown (CHANGELOG 2026-11-19 (X-11 flown on SITL and hil)): the msl_dem profile (MSL polygon_fence, corridor and altitude_envelope over the ridge DEM experiments/worlds/dem_cmac_wgs84.json, sha256 44ed8a16...); on SITL at bdc0af6 6 missions; on hil (AGX Orin + SITL) at e56d3e7, campaign msl_dem_hil2, 6 missions (demo/out/hil_smoke/msl_dem_hil2_00N_*, each kpi.json kpi_grade true). Both: P0 escape 0 (null 0.045/0.051), 0 breaches, 0 polygon entries, true_p0_flown_ticks 0, 3 pass / 3 fail per rail, each zone-ridge cell a false trigger (fail-safe 0/1) (docs/data/msl_dem_20261119/sweep_sitl.json, sweep_hil2.json). Bag replay identical on hil 6/6 and on SITL 4/6 (the other 2 one watchdog tick off); `guardrail.replay verify` passes on all 6 DEM-carrying tarballs. The ridge passes with the goto pilot's terrain feed-forward (guardrail/vla_stub.py TerrainFeed, 27cbeee; theta and FSM unchanged): msl-dem-zone-ridge on the SITL rail at 59e19a5, 22 Nov, 2/2 reached the pad, P0 escape 0, 0 polygon ticks (docs/data/shakedown_20261122/README.md, msl_ridge_sitl_sweep.json). Earlier 18 Nov: msl_dem_sitl_01..03 (dev SITL). | (1) No committed test of MSL enforcement on circle_fence or corridor_swap: circle_fence MSL has no test, corridor_swap MSL appears only as a FENCE_ALT_MAX cap case (tests/test_msl_readers.py:753-766), and dynamic_nfz MSL only through a hot-apply over the DEM (tests/test_msl_dem.py:150); the code paths exist (inheritance, terrain.KEEP_OUT_TYPES / keep-in band). (2) Readers outside the ROS 2 / SITL-hil rail call load_for_flight with no ground and refuse an MSL policy: the PAS perception node (demo/pas_ardupilot/perception_node.py:7038), the city follow (demo/follow_vlm.py:5607), vla_fly (demo/vla_fly.py:1634), the legacy AirSim scripts and the Gazebo rail; terrain.scene_ground is called only in a test, so the opt-in is not available on the PAS stack. (3) The hil zone-ridge cells false-triggered (19 Nov, before the feed) and have not been re-flown on hil with the terrain feed, which is the test pilot's and flown on SITL only. Residual: the DEM is synthetic, a declared ridge about CMAC, not survey data; SITL's own ground stays flat, so AGL over it is the node's computed height (experiments/profiles/msl_dem.yaml:28); no MSL cell is in a KPI set (profile msl_dem grant_kpi_bearing false, msl_dem.yaml:44; in neither smFIN2 nor ntFIN4); the 0.25 m DEM band margin is the repair's aim, not a guarantee under baro or EKF noise. |
| X-12 | Hybrid hot-update rule: only dynamic_nfz, time_window_switch and corridor_swap may change mid-flight; polygon_fence and the envelopes stay locked from mission start | DONE | hot-apply of a fixed polygon fence after take-off refused; only the three classes hot-apply (guardrail/api.py; CHANGELOG 2026-10-07); tests/test_api.py, tests/test_shield_events.py | None. |
| X-13 | Single rule-evaluation code path (the checker reuses the IR and nothing else evaluates rules) | PARTIAL | As WP3-22 (DONE 23 Nov). One evaluator in guardrail/shield.py (RuleClock, point verdicts, clearance_breached, standoff_breached), built from the IR, and every reader calls it (compiler.action_violations / tools/prefix_eval, fence_export, rtl_path, replay.zone_dwell, FenceGuard). Since 18-19 Nov (wf21-onepath3) FenceGuard's building half measures with the Shield's field and judges with clearance_breached against Shield.clearance_limits_now; guard_field accepts "shield" only (demo/follow_vlm.py:2422-2436, :2565-2622, :9355). The grid scan is a diagnostic read only by tools/onepath_guard_replay.py. The AST scans find no second evaluator of the scanned forms, no wrapped primitive called elsewhere, and no IR rule threshold compared outside the checker except 15 named non-verdicts (tests/test_shieldcore_onepath.py:1025, :1108-1174, :1524; 27/27 in lab CI 8e73f86, 22 Nov, 187/187 files, 4854 of 4870 tests passed, 16 skipped). Flown with that default, functional and not KPI-grade (projectairsim-remote-sim): 7 AGX Orin CityLife follow flights at bfb4973, 19 Nov, and 2 at 28a594d, 20 Nov, all guard_field shield, P0 escape 0 (docs/data/onepath_agx_20261119/, docs/data/follow_agx_20261120/). | "Nothing else evaluates rules" is not met. (1) The autopilot GeoFence backstop, fed by guardrail/fence_export.py, is ArduPilot's own fence check on the exported zones; it acts in flight and is scored: 18 of the 648 scored fail-safes in ntFIN4 at 59e19a5 were its triggers (docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md:18, :79-96); counting them is lab decision 14 (15 Nov), PI confirmation pending (docs/RUBRIC-failsafe.md sections 1 and 9). (2) models.start_conflicts is its own point-in-zone and margin test (_point_in_ring, _dist_to_ring, `<=`, schedules and bands ignored); through tools/geo_to_policy.py:315-320 it refuses a take-off point inside a zone. (3) The 15 non-verdicts and the 3 outside clearance measures are the lab's own sorting (e.g. compiler._clearance_mask builds the cost map's blocked cells, prefix_consumer._clip clamps commands into the CSP's allowed set before the Shield; both shape what flies); no written acceptance from the PI or ITRI is recorded. (4) The CI scans cover Python in SOURCE_DIRS only and match by name and AST pattern; they cannot see the autopilot's fence code. (5) The FenceGuard one-path change has flown only on projectairsim-remote-sim, not KPI-grade. Closes with the PI's written acceptance of (1)-(3) as outside the system's rule evaluation, or their folding. |
| X-14 | Dynamic-actor stressors in parameter sweeps: wind (and GPS denial) | DONE | wind, gusts, GPS dropout stressors with a closed-loop ArduCopter wind model (CHANGELOG 2026-10-06, -13); experiments/templates/wind_disturbance.yaml, sensor_degradation.yaml; since 7 Nov both flight rails (sitl, hil) fly wind (SIM_WIND_SPD at the episode's t=0, SIM_WIND_DIR, SIM_WIND_T 1), GPS denial (SIM_GPS_DISABLE on the episode clock, aligned by the node's announced t0) and start jitter (SITL_HOME) as SITL's own parameters, set from the simulator side and checked fail-closed (sitl/sitl_stressors.py, tests/test_sitl_stressors.py; CHANGELOG 2026-11-07 (SITL stressors)); flown on the hil rig on 7 Nov in the smoke campaign rh03 (44 missions, the AGX node at fb96fe5): wind within authority 5 m/s in two lanes, beyond it 8 and 12 m/s, GPS denied from 4 to 11 s in two lanes, every parameter read back as set and applied at its time (each run's scenario_score.json sim_stressors_check: no problems); X8 sent one GPS-dropout mission home (rh03_040, the first X8 flown on hil); P0 escapes 0 on all 44. The 2 Nov pymavlink runs sitl_wind8_shield_on/off_20261102 flew 4 m/s at 15 m, not the 8 they name (SIM_WIND_T unset; retracted 2026-11-07) | Gusts stay headless (4.5.7 has no faithful stand-in: SIM_WIND_TURB redraws from SITL's own random numbers); the rails refuse them by name. GPS noise / latency fly on the hil and sitl rails since 115640b (21 Nov), applied companion-side to the pose the Shield and the FSM read, not through SITL (SIM_GPS_NOISE is altitude-only): sk1122a, smFIN2 and ntFIN4 at 59e19a5 (WP3-19). The KPI-grade nightly ntFIN2 (13 Nov) flew the wind and GPS-denial cells, but 3 of its 24 gps-dropout missions were not accepted (SIM_GPS_DISABLE never applied) and 8 wind-beyond-authority missions never flew (the desktop out of memory); the supplement ntFIN2s re-flies those cells (at the time of writing, one of its first three gps-dropout missions was again not accepted: the denial's end applied 0.602 s late). |
| X-15 | SROS 2 security on the ROS 2 graph (the harness mTLS reuses its root CA) | DONE | SROS2=1 opt-in, one enclave per process, Enforce; flown 2026-11-02 (ros2_sros2_shield_on_20261102, allow-all permissions); since 3 Nov per-topic permissions: sitl/sros2_policy.xml signed per enclave by the keystore's permissions CA = the lab CA (tools/sros2_setup.sh --permissions-only): only the Shield's enclave may publish /shield/setpoint, only the VLA slot's /vla/action_4d, MAVROS its own mavros/* and uas1/*; tools/sros2_check.sh 8/8 in WSL Jazzy, check 7 showing a VLA-enclave publisher of /shield/setpoint refused ('rt/shield/setpoint topic not found in allow rule'); flown sros2_policy_on_20261103b (re-flown 4 Nov from the main checkout, code recorded: sros2_policy_on_20261104) with the policy's permissions (permissions_scope per_topic in hil_evidence.ros2_security); the harness that drove harness_corridor_20261103c (re-flown 4 Nov from the main checkout, code recorded: harness_corridor_20261104b) ran its gRPC mTLS with the same lab CA (~/harness_pki in WSL) (CHANGELOG 2026-11-03 (G)) | The Windows harness PKI (%LOCALAPPDATA%/vla-guardrail/harness_pki) is still another CA: run the harness where the keystore's CA is, as done here, or seed both from one CA. |
| X-16 | Meaning of the vla_model_hash field (SHA-256 of the VLA's weights). This is a nuance of the six-field manifest item rated DONE | DONE | vla_model_hash carries the weights' SHA-256 on VLA flights (e.g. nfz3b_orin_hier_aerovla_20261028/manifest.json aerialvla:...@sha256:ca9e8701...), none:hand-written-controller otherwise | None. |


## Appendix B — independent-review note

Copied from docs/FINAL-REPORT-outline.md at 2f22116, the 13 Nov note. Out of date since: point 7 (the start-up fix is on master since d7bb6a2 and flew in hs09 and in the full campaign ge1113 with no tick over 100 ms, section 5.6); the supplement is merged (WP3-15 holds on 567 scored runs, section 2); on 14 Nov WP3-13 moved to DONE, WP3-07's edges all flew in hs10 (section 4.3) while the row stays PARTIAL (SET_MODE LOITER on no hil flight, lab decision 7, PI confirmation pending), and WP4-17's report is committed from a clean tree, so only its delivery remains (Appendix A: 88 DONE, 41 PARTIAL, 6 DRIFT, 1 MISSING).


Re-assessed against the grant pages' own words (kuanting-vla-uav-guardrail/docs/,
each row quoting the line it is held to), not reviewed by anyone outside
the project. It records what moved, why, and what an external reviewer
could still dispute.

**What changed.** Eight rows from PARTIAL to DONE on the KPI-grade nightly
ntFIN2 (116214f, 580 KPI-grade episodes, tools/kpi_report.py --campaign):
WP3-14 (P0 escape 0 over 16,703 P0-violation ticks, Shield-off control
0.6375); the KPIs the grant lists as tracked, each now with a KPI-grade
value - WP3-16 (6.407 m/s), WP3-17 (3.23 s), WP3-20 and X-02 (98.2 %),
X-03 (55.4 per mission), WP4-08 (252/577); and WP4-12 (the six fields on
all 583 runs at one clean revision, sim_speedup 1.0, hil). Counts 70 / 59 /
6 / 1 become 78 / 51 / 6 / 1. Evidence refreshed with no status change:
WP3-07, WP3-13, WP3-15, WP3-19, WP4-01, WP4-04, WP4-06, WP4-11, WP4-14,
WP4-17, ARCH-19, ARCH-22, ARCH-31, X-14, and the open notes of WP2-15,
WP3-08, WP3-11.

**Why the rest stay PARTIAL.** WP3-15: 0.998 holds on the 558 scored runs;
30 missions are unscored (19 censored in Loiter, 11 for the supplement) and
5 errors among them would take it below 0.99. WP3-13: 13 start-up
ticks over 100 ms. WP4-17 and ARCH-19: the KPI report and the final report
are not written. WP4-06, WP4-14, WP3-19: per build and per release need a
push and a release, and GPS noise / latency has no rail. WP4-01: the
paraphrase and prefix KPIs have `hil` values only from text-reading pilots
flown outside the stress harness. WP4-04: one Gazebo functional sweep, one
seed (20 Nov, 7 of 8); no multi-seed sweep and no speed > 1.
WP4-11: the grant's bundle names not flown. ARCH-31: no flight firmware.
WP3-07: G2, G4 and G6 not flown. Would complete: the supplement ntFIN2s and
the 19 censored runs flown long enough, WP3-15 (if 4 errors or fewer); the
start-up fix flown on `hil`, WP3-13; both in a committed clean-tree report
and section 5 written from it, WP4-17.

**What an external reviewer might still dispute.**

1. The pilot. Every KPI-grade run flies a stub (guardrail/vla_stub.py
   GotoPilot, ConstantPilot, StubVLA), where the grant's `hil` has the Orin
   run "VLA + Shield". The figures measure the Shield and its FSM in front
   of ArduPilot SITL under scripted commands, not a learned model (ARCH-20).
2. P0 escape is the Shield's own count, as the grant defines it ("Counter
   from Shield repair log"). On the true position ArduPilot carried the
   aircraft into a P0 zone in 21 Shield-on missions: 18 GeoFence breaches
   with RTL where a zone appeared 3 m ahead at 4 or 8 m/s, and 3 RTL legs
   through a time-windowed zone. A reviewer may count a zone entered.
3. Fail-safe correctness has four values on one campaign: 0.998 (r2,
   adopted), 0.977 (r), 0.892 (the labels flown with), 0.871 (a). The
   adopted reading was chosen on 10 Nov after rig flights had been scored
   on another, and the gps-dropout labels were re-derived on 12 Nov after
   nt1111. Both were committed before ntFIN2 flew (ancestors of 116214f), so
   this campaign did not tune them, but neither was pre-registered. The
   Wilson 95 % lower bound is 0.9899; the grant asks for the point value.
4. Independence. No flown cell consumes its seed: a cell's three seeds are
   three flights of one deterministic scenario that differ only by the
   rig's timing (389 distinct KPI signatures over 580 episodes), while the
   rates and the Wilson bound count them as independent.
5. Coverage. 153 of the profile's 744 episodes were refused before flight
   (69 paraphrase / prefix arms, 69 stressors SITL cannot realise, 15
   others, among them the RTL that fails, G6's scenario). The 8 missions
   that never flew are wind-beyond-authority, the family in which hs07 saw
   ArduPilot's RTL carry the aircraft into nfz-square in a 12 m/s wind. A
   first attempt at the same commit, ntFIN (175 runs, stopped part-way on
   12 Nov, no sweep written), is not in the figures; its runs are kept
   (demo/out/hil_smoke/ntFIN_*).
6. Definitions. "End to end" in the tick budget starts at the Shield
   node's receipt of the pilot's message, not at a camera frame or a VLA
   decision; repair magnitude leaves the yaw rate out; mission success is
   stricter than the grant's; time to safe averages only the recovered
   spells (75 left out); theta judges about a third of the converged
   repair ticks.
7. Provenance. The 13 Nov KPI report was generated from a tree with
   uncommitted changes (`116214f3e6ee-dirty`) and is outside the
   repository; the start-up tick fix (branch kpi-merge-startup) is not on
   master and has not flown.
