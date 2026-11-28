# Final Report

Semantic-Spatial Translation and Safety-Constrained VLA for ArduPilot UAVs ·
NTUT AIoT Lab · v1.0, 2026-11-28

> **Status: final, v1.0.** It is signed at the Approval block. The signed
> copy is built from the final master commit, which is written into the
> block's 'Report body' field when the signed DOCX/PDF is built.
> Delivered release: 0.7.3 (tag v0.7.3, commit c44a360; CHANGELOG.md
> [0.7.3]). Its `hil` smoke smREL073 flew 52 KPI-grade missions on 28 Nov
> and met the release gate (docs/data/kpi_smREL073/README.md).
> KPI figures of record: the nightly `hil` campaign ntFIN5 at release 0.7.0
> (21c2771). Its KPI report was rendered again on 28 Nov with the
> policy-artefact KPIs measured at c44a360; the flight KPI figures are
> unchanged (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md).
> Appendix A, 136 obligations: 134 DONE, 2 PARTIAL, 0 DRIFT, 0 MISSING.
> No value is filled in ahead of its evidence. Every number below names
> the file or run it comes from.

## 1. Summary

- **KPI results of record**: the nightly `hil` campaign ntFIN5 at release
  0.7.0 (21c2771), flown 23-24 Nov on the AGX Orin rig (AGX Orin +
  ArduCopter 4.5.7 SITL over MAVROS 2): 654 missions flown, all 654
  KPI-grade (651 Shield on, 3 Shield-off controls), single_sha 654/654
  (section 5; docs/data/kpi_ntFIN5/README.md). Its KPI report of record,
  docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md,
  was rendered again on 28 Nov with the policy-artefact KPIs measured at
  the delivered release; the flight KPI figures are unchanged.
  - P0 violation escape rate **0.0**: 0 escapes on 17,194 P0-violation
    ticks over 651 Shield-on flights. Shield-off control: 0.6500.
  - Fail-safe trigger correctness **649/649 = 1.000** on the adopted
    reading (r2), Wilson 95 % lower bound **0.994**: it meets 99 % as the
    point estimate and at 95 % confidence. 18 of the fail-safes were
    triggered by the autopilot's GeoFence backstop (decision 14), all
    expected and correct; without that rule the same runs give 631/631
    (Wilson low 0.994). Two runs censored in Loiter are left out (649/651
    = 0.997 if both were errors). The other readings: (r) 637/649, (a)
    574/649.
  - Monitor tick within 100 ms end to end on **every** tick: 0 of 183,961
    ticks over 100 ms. `tick_ms` p99 28.3 / max 62.08 ms; action receipt
    to setpoint (`e2e_ms`), and the same with the wait behind the
    post-setpoint work counted (`e2e_with_wait_ms`), p99 28.4 / max
    62.18 ms. The 249 ticks that hot-applied a rule reached their setpoint
    in median 14.3 ms, max 62.08 ms (section 5.4; WP3-13).
  - Policy-artefact KPIs, measured by the offline tools at c44a360
    (reading (h)): DSL round trip 33/33, tamper refusal 33/33, negative
    corpus 39/39; cold-start ingest of a 50-rule bundle p95 477 ms on the
    desktop and 784 ms on the AGX Orin, against 1 s
    (docs/data/wp1_roundtrip_c44a360.json,
    deploy/evidence/hil/wp1_roundtrip_agx_c44a360.json); CSP P0 coverage
    124/124 over 33 policies at every token budget from 256 down to 144
    (docs/data/csp_kpis.json).
- **Delivered release 0.7.3** (tag v0.7.3, commit c44a360; CHANGELOG.md
  [0.7.3]). It holds the Guardrail in the grant's four parts: a Policy DSL
  with a typed IR and signed bundles (WP1), a Prefix Compiler that emits a
  Constraint Summary Pack (WP2), a Safety Shield with an escalation FSM in
  front of ArduPilot (WP3), and a Stress Testing harness with a scenario
  library, episode bundles and a KPI report generator (WP4). The policy
  bundles are signed under the Lab CA (WP1-04).
  - Its `hil` smoke smREL073 (28 Nov): 52 missions flown (51 Shield on, 1
    Shield-off control), all KPI-grade, single_sha 52/52, the bundle chain
    verified under the Lab CA in 52/52. P0 escape 0 on 2,393 P0-violation
    ticks; fail-safe (r2) 50/50 (Wilson 95 % low 0.929, 1 censored);
    `tick_ms` max 55.73 ms (p99 25.5), 0 of 16,047 ticks over 100 ms. The
    five rows of the grant's Shield test matrix flew KPI-grade, each with a
    non-vacuous cell, and the release gate of docs/RUNBOOK-release.md is
    met (docs/data/kpi_smREL073/README.md). Releases 0.7.0, 0.7.1 and 0.7.2
    flew the same five rows in their own `hil` smokes (WP3-19, WP4-14).
  - Replay (WP4-11): every KPI-grade `hil` run flown from 22 Nov on, 1,660
    episodes, is reproduced bit for bit by its own revision on the AGX
    Orin runtime it flew on, 0 differences
    (docs/data/replay_agx_rest_20261128/README.md, with
    docs/data/replay_agx_ntFIN5/ and docs/data/replay_agx_smREL073/).
- **Contract map** (Appendix A, 136 obligations):
  **134 DONE, 2 PARTIAL, 0 DRIFT, 0 MISSING**. DONE means the grant's
  obligation is met by the delivered code and evidence, under the readings
  the project applies where the grant is open (section 9), with any stated
  limit written as scope; for a KPI row, a KPI-grade value meets the
  grant's figure. PARTIAL means a part of the grant's wording is not
  delivered and no reading covers it. The two PARTIAL rows:
  - **ARCH-08**, the optional AP_DDS migration (Q3), closes at stage 1 by
    reading (m): the Shield's output went over /ap/cmd_vel on ArduCopter
    4.5.7 DDS SITL, 113 of 113 ticks, P0 escapes 0
    (docs/data/ap_dds_20261126/README.md). Stage 2 is not delivered: the
    Shield's inputs, mode changes, arming, take-off and the GeoFence stay
    on MAVROS 2, and 4.5.7's DDS client publishes no mode, armed or landed
    state and has no take-off, parameter or fence service.
  - **ARCH-22**, rosbag2 capture for the replay contract: every episode of
    ntFIN5 and smREL073 holds a bag, but no bag holds an image topic (the
    KPI rail carries no camera; the camera rail records no bag), and the
    manifest start frame is the first frame of episode.bag2 in 49 of 52
    smREL073 bundles (the other 3 keep the recorder's bag, the reason
    recorded in kpi.json; docs/data/kpi_smREL073/README.md).
- Every edge of the grant's escalation ladder has flown on `hil`: G2, G4,
  G6, X2 and X3 in hs10 on 14 Nov (section 4.3; CHANGELOG 2026-11-14
  (hs10 ...)). The FSM's Loiter is held in GUIDED by default (reading (e));
  literal SET_MODE LOITER is opt-in and flew on `hil` in sk1122b on 22 Nov
  (WP3-07).
- The KPI campaigns fly a stub pilot (guardrail/vla_stub.py); the learned
  VLA is shown in separate `hil` flights (reading (h)). The lab's AIoT-VLA
  v10b flew on the AGX Orin through MAVROS 2 with the Shield on, 0 P0
  escape ticks in every flown mission (section 6;
  docs/data/pas_aiot_hil_20261114/flights.md). Since 15 Nov 12.4-13.2
  camera frames/s reach it while it is asked, and it decided 10.6 and 10.9
  times a second over two full 420 s missions, 0 P0 escapes
  (docs/data/pas_hil_feed_20261115/flights.md; X-05, ARCH-25, ARCH-01,
  ARCH-21). Follow quality is reported, not a locked KPI (section 6).
- Scope, stated as limits (section 10):
  - No airframe flight: the `flight` stage is covered by a written waiver
    resting on the `hil` rig and a bench test of the real FC (CUAV V5
    Nano, ArduCopter 4.7.1, never armed; ARCH-14). Bit-identity between
    SITL and the firmware is not shown without a flashed FC (the rig runs
    SITL 4.5.7; ARCH-31).
  - The 7B backends do not decide at 10 Hz on the Orin (OpenVLA-7B median
    0.975 s per action on the AGX Orin, section 6.3); AIoT-VLA v10b does
    (ARCH-03).
  - No flight served a learned VLA beside a ~50-rule bundle; the tick
    budget rests on the stub-pilot campaigns and the offline 50-rule
    replay on the Orin (reading (g); WP3-13).
  - The smoke runs KPI-grade on `hil` per release and headless per push;
    the nightly KPI campaign is flown per release candidate, for the
    release the KPI report of record describes (ntFIN5 at 0.7.0;
    docs/RUNBOOK-release.md section 7); the broad sweep runs on the
    headless backend (reading (c); WP4-06). Gazebo runs
    are functional evidence, never KPI-grade (WP4-04).
  - People are kept at the distance envelope as far as the person
    detector sees them (reading (b); WP1-07).

## 2. Scope and acceptance criteria

"KPI-grade" means `guardrail.manifest.is_kpi_grade` accepts the run (`hil`
topology with the Orin's evidence beside the manifest, sim_speedup 1.0, a
clean code_revision), one clean revision ran on every host of the stack
(`single_sha`; `guardrail.manifest.stack_grade_reasons`, applied beside it
since 27 Nov), the signed replay bundle beside the run verifies and is bound
to it, and the campaign accepted the flight (tools/kpi_report.py,
"Definitions, denominators and nulls" section of its output).

**The KPI-grade source (the figure of record since 24 Nov).** Profile
`nightly` at release 0.7.0 (tag v0.7.0, 21c2771, clean), flown on the AGX
Orin rig: ArduCopter 4.5.7 SITL and mavlink-router on the desktop; MAVROS 2,
the Shield and the stub pilots on the Orin.

- ntFIN5: launched 23 Nov 20:12 with `--hil-controls` from the detached
  worktree flight-21c2771, after its gate, the release smoke smREL070 (P0
  escape 0, no ERROR, (r2) at least 0.98); 77,356.0 s (21.5 h). 744
  episodes in the profile; 90 refused by the rail before take-off, the same
  90 as ntFIN4 (69 paraphrase / prefix arms the stub cannot read, 9
  stressors the rail cannot realise: gusts and a low battery, 12 others:
  starts at 0.5 m and at 30 m, a non-finite pilot action, a relabel
  schedule). 654 missions flown (651 Shield on, 3 Shield-off controls), all
  654 KPI-grade (docs/data/kpi_ntFIN5/README.md;
  demo/out/hil_ntFIN5/sweep.json; runs demo/out/hil_smoke/ntFIN5_*).
  single_sha 654/654: companion and desktop at 21c277170001, autopilot
  ArduCopter V4.5.7 (2a3dc4b7).
- It replaces, as the figure of record, the nightly ntFIN4 at 59e19a5
  (22-23 Nov, 653 KPI-grade; docs/data/kpi_ntFIN4/), which replaced ntFIN3
  at 71aabb3 (21 Nov, 594 KPI-grade; docs/data/kpi_ntFIN3/) and ntFIN2 +
  ntFIN2s at 116214f (12-13 Nov, regraded 17 Nov;
  docs/data/kpi_ntFIN2_20261117_delivery/). 21c2771 is 59e19a5 plus the
  changes of CHANGELOG 2026-11-23 (code), among them the hot-apply tick fix.
- Release smokes. Every release from 0.7.0 on flew a 64-episode smoke on
  the same rig, 52 missions flown and 52 KPI-grade each (reading (c),
  section 9.2): smREL070 (0.7.0, 21c2771), smREL071 (0.7.1, a3d616e),
  smREL072 (0.7.2, 418b151) and smREL073 (0.7.3, c44a360, the delivered
  release, 28 Nov) (docs/data/kpi_smREL070/ to kpi_smREL073/README.md).
  smREL073: P0 escape 0 on 2,393 P0-violation ticks (Shield-off control
  0.65, passthrough null 0.3277); fail-safe (r2) 50/50, Wilson 95 % low
  0.929 (1 censored); `tick_ms` p99 25.5 / max 55.73 ms, 0 of 16,047 ticks
  over 100 ms; single_sha 52/52; the bundle chain verified under the Lab CA
  in 52/52; the release gate of docs/RUNBOOK-release.md met, with a
  non-vacuous cell in every Shield-matrix row (near-edge-at-12mps 2/2)
  (docs/data/kpi_smREL073/README.md). No nightly followed 0.7.1 to 0.7.3,
  so the nightly of record stays at 0.7.0. ntFIN5's 654 runs replay with
  identical Shield decisions through the body-frame stack that 0.7.1 ships
  (CHANGELOG.md [0.7.1]).

| KPI (grant) | Target | KPI-grade result, ntFIN5 (21c2771, release 0.7.0) | Earlier: ntFIN4 (59e19a5) | Verdict (ntFIN5) |
|---|---|---|---|---|
| P0 violation escape rate (WP3-14) | 0 | 0.0000: 0 escapes on 17,194 P0-violation ticks, 651 Shield-on flights; Shield-off control 0.6500 (156 of 156 P0 ticks, 3 flights); passthrough null 0.1938 | 0 escapes on 17,078 P0-violation ticks, 650 flights; control 0.6500; null 0.1935 (ntFIN3: 0 on 17,457) | meets |
| Fail-safe trigger correctness (WP3-15) | ≥ 0.99 | (r2, adopted): 649/649 = 1.000, Wilson 95 % low 0.994, 18 triggered by the GeoFence backstop (decision 14), all expected and correct; (r) 637/649 = 0.982; (a) 574/649 = 0.884; before the backstop rule 631/631; 2 runs censored in Loiter (649/651 = 0.997 if both were errors) | (r2) 648/648 = 1.000, Wilson 95 % low 0.994 (ntFIN3: 587/589 = 0.997, 0.988) | meets as the point estimate, the grant's figure, and at 95 % confidence |
| Monitor tick ≤ 100 ms end to end (WP3-13) | ≤ 100 ms | `tick_ms` p50 7.8 / p99 28.3 / max 62.08 ms; `e2e_ms` p50 7.9 / p99 28.4 / max 62.18 ms; `e2e_with_wait_ms` (the wait behind post-setpoint work counted) p99 28.4 / max 62.18 ms; 0 of 183,961 ticks over 100 ms on each | `tick_ms` max 105.50 ms, `e2e_ms` max 105.60 ms, 1 of 183,909 over 100 (a hot-apply tick; ntFIN3: 0 of 174,186, max 74.62 ms) | meets on every tick |
| Mean time to safe (WP3-17) | tracked | 3.49 s mean, max 9.90 s, over the 165 unsafe-position spells that recovered (in 164 Shield-on flights); 94 spells never recovered (counted, not averaged) | 3.39 s, max 9.70 s (169); 95 never recovered | tracked |
| Repair success rate (WP3-20, X-02) | tracked | 98.2 % (27,206 of 27,716 repair ticks) | 98.1 % (27,097 of 27,615) | tracked |
| Mean repair magnitude (WP3-16) | tracked | 6.329 m/s (translational, pooled over repaired ticks; yaw rate apart) | 6.338 m/s | tracked |
| Average repair count per episode (X-03) | tracked | 50.2 per Shield-on flight | 50.2 | tracked |
| Mission success rate (WP4-08) | tracked | 288/651 Shield on (our rule is stricter than the grant's: no dwell in a zone, no unsafe position entered, goal reached) | 287/650 | tracked |

Source of the ntFIN5 figures: the KPI report of record,
docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md and
.json (`python tools/kpi_report.py --campaign demo/out/hil_ntFIN5
--rescore-rail docs/data/kpi_ntFIN5/rescore_r2_ntFIN5.json --baseline
docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.json --out
docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md`),
made from a clean tree at f9cab68 with the WP1 and WP2 records at c44a360.
Its flight KPI figures are those of the first report,
kpi_report_ntFIN5_2026-11-24.md (clean tree at b171c57); it adds the
Shield-matrix section of release 0.7.3: every row has a non-vacuous cell,
and the 12 planned near-edge-at-12mps cells all flew non-vacuous (gate met;
docs/data/kpi_ntFIN5/README.md, "Re-rendered 28 Nov"). Beside it: the
re-score table (rescore_r2_ntFIN5.json and .txt,
`experiments/sweep_scenarios.py --profile nightly --rescore-rail` over the
654 runs) and the replay at the flown revision (replay_at_revision_ntFIN5.json
on the desktop; 654/654 identical on the AGX Orin,
docs/data/replay_agx_ntFIN5/README.md). The earlier reports stay as they
were made: docs/data/kpi_ntFIN4/ (23 Nov), docs/data/kpi_ntFIN3/ (21 Nov),
docs/data/kpi_ntFIN2/ (13 Nov), docs/data/kpi_ntFIN2_20261115/ (15 Nov, the
backstop rule) and docs/data/kpi_ntFIN2_20261117_delivery/ (17 Nov, the
setpoint delivery regrade).

Other locked KPIs (WP4-01). Under reading (h) (section 9.2), the
policy-artefact KPIs (DSL acceptance, policy load round trip, prefix-token
budget, CSP P0 coverage) are measured by the offline tools at the delivered
revision, c44a360 (release 0.7.3, 28 Nov). Bundle replayability comes from
Stress Testing runs replayed from their bundles. Per-paraphrase robustness
needs a pilot that reads text, so the learned VLA shows it in separate `hil`
flights. Each row names the tool and the host:

| KPI (grant) | Target | Evidence | Measured by (tool, host) |
|---|---|---|---|
| DSL acceptance (WP1-19) | schema 100 %, golden 100 %, cold start ≤ 1 s, hot-apply ≤ 50 ms, query ≤ 5 ms | At c44a360 on the AGX Orin: cold start p95 784 ms (782 ms for the certified signer, chain checked), tampered / untrusted / bad-signature bundles refused; hot-apply at 50 rules p99 4.97 ms; query (`_check`, 50 rules x 51 poses) p99 1.048 ms, max 1.097 ms, 0 of 1,301 ticks over 5 ms. On the desktop: schema validation 33/33 and the grant's 3 documents 3/3, golden 100 %, cold start p95 477 ms (docs/data/wp1_roundtrip_c44a360.json). In flight on `hil`, REST receipt to swap 0.81-6.35 ms over 10 events (00264ef; one 53.0 ms spawn before that fix) | tools/wp1_roundtrip_kpi.py at c44a360: cold start on the AGX Orin in the companion image, Python 3.12.3, fresh processes, interpreter start not counted (with it p95 1,081 ms) (deploy/evidence/hil/wp1_roundtrip_agx_c44a360.json); schema on the desktop, Windows, Python 3.10.20 (the Orin's image has no jsonschema); hot-apply from experiments/bench_shield_50rules.py on the AGX Orin (deploy/evidence/hil/bench_shield_50rules_5s_agx_c44a360.json); query from tools/profile_shield_tick.py on the AGX Orin over 1,301 real flight ticks (deploy/evidence/hil/shield_tick_profile_20261128.json); REST receipt to swap from the PAS follow on the AGX Orin, flown outside the stress harness (docs/data/pas_hil_hotapply_20261115/) |
| Policy load round-trip (WP1-17) | 100 % | round trip 33/33, tamper 33/33, negative corpus 39/39 refused, all 33 signed under the Lab CA with the chain verified (docs/data/wp1_roundtrip_c44a360.json, 28 Nov, release 0.7.3; also docs/data/wp1_roundtrip.json); the same 33/33, 33/33 and 39/39 on the AGX Orin (deploy/evidence/hil/wp1_roundtrip_agx_c44a360.json). Earlier records: docs/data/wp1_roundtrip_21c2771.json (release 0.7.0) and docs/data/wp1_roundtrip_4edd5d1.json (6 Nov, 32/32) | tools/wp1_roundtrip_kpi.py at c44a360, offline on the desktop (Windows, Python 3.10.20) and on the AGX Orin (Python 3.12.3) |
| Bundle replayability (WP1-18) | load the bundle whose hash matches | smREL073's 52 episode bundles alone, with no run folder in reach, replay 52/52 identical at c44a360 on the AGX Orin (docs/data/replay_agx_smREL073/README.md); ntFIN5 654/654 identical at 21c2771 on the AGX Orin (docs/data/replay_agx_ntFIN5/README.md); hil_smoke_03 (6 Nov, KPI-grade): its signed replay bundle verifies and re-derives its KPIs | tools/replay_at_revision.py on the AGX Orin in the hil companion container, over the Stress Testing runs smREL073 and ntFIN5; for hil_smoke_03 (flown on the AGX Orin at d0e91bb), `python -m guardrail.replay verify` and tools/kpi_report.py |
| Prefix-token budget (WP2-02) | budget respected, CSPBudgetExceeded on overflow | At c44a360, 33 policies: no CSPBudgetExceeded and every CSP within budget at each budget from 256 down to 144 tokens (largest CSP 149 tokens; 126 at 144) (docs/data/csp_kpis.json); the raise on overflow: tests/test_csp.py | tools/kpi_report.py --wp2-only at c44a360, offline over the policy library; tests/test_csp.py |
| CSP P0 coverage (WP2-03) | 100 % | 124/124 P0 over 33 policies at every budget from 256 down to 144 tokens; the naive cut 122/124 at 144 (docs/data/csp_kpis.json, 28 Nov, c44a360) | tools/kpi_report.py --wp2-only at c44a360, offline over the policy library |
| Per-paraphrase robustness (WP4-09) | stability across paraphrases | `hil`, KPI-grade (15 Nov, AIoT-VLA on the AGX Orin, the PAS follow): 6/8 paraphrases ended as the source "follow a red car" did (target_lost, twice), 2/8 held the car to max_s, as the model's own noun phrase "a red car" did; 0 P0 escape ticks over 13 runs; the stress harness's own arms fly the text-blind stub. 7/8 with OpenVLA-7B on one AirSimNH mission (7 Nov, not KPI-grade) | the PAS follow on `hil` (AGX Orin, AIoT-VLA v10b), flown outside the stress harness (docs/data/pas_hil_paraphrase_20261115/); the 7 Nov OpenVLA-7B figure from one AirSimNH mission outside the harness |

## 3. System architecture and topologies

- **Action contract.** A 4-D body-frame action (vx, vy, vz, yaw_rate) at
  10 Hz (ARCH-05, WP3-03: DONE). Since 4 Nov the Shield takes and returns the
  body frame at its own boundary (`Shield.filter_body`); body to local NED
  happens after the projection, in sitl/mavlink_adapter_node.py on the
  ROS 2 rail, and every flying rail hands the Shield the body action
  (ARCH-04: DONE since 20 Nov). Since 783e626 (21 Nov, flown in ntFIN4) the
  registry repair operators are handed and hand back the body-frame action
  at the operator interface, since bec7135 (23 Nov, release 0.7.0) the
  last resorts too, and since 35df373 (26 Nov, release 0.7.1, flown in
  smREL071) no rotation wraps the stack: it carries the body action and
  reads North/East/up only through the tick's repair.BodyFrame (WP3-04:
  DONE 26 Nov, independent review, 3 of 3). The simulator demo loops call
  the same frames.body_to_local_ned in their own loop, after the Shield
  (accepted under ARCH-04). Since 14 Nov a learned VLA (AIoT-VLA on the AGX
  Orin) emits the setpoint stream into ArduPilot on `hil` (ARCH-01: DONE
  since 15 Nov).
- **Pipeline.** Bundle → CSP → pilot → Shield (+ FSM) → MAVROS 2 →
  ArduPilot, with ArduPilot's GeoFence as the last backstop (the fence
  parameters from sitl/fence/guardrail_fence.parm, or set and read back by
  the GCS step sitl/gcs_backstop.py since 3 Nov; WP3-10, ARCH-09). Since
  release 0.7.3 the ROS 2 Shield node's report.md and metrics.json
  (`setpoint_path`) name the path flown, from the nodes seen subscribed to
  /shield/setpoint;
  in smREL073, 52 of 52: `vla node -> /vla/action_4d (body) ->
  safety_shield -> /shield/setpoint (body) -> mavlink_adapter ->
  /mavros/setpoint_raw/local -> MAVROS 2 -> ArduPilot`
  (docs/data/kpi_smREL073/README.md).
- **AP_DDS (ARCH-08, PARTIAL).** The optional Q3 migration to ArduPilot's
  AP_DDS closes at stage 1 and is reported PARTIAL (reading (m), section
  9.2). It has an opt-in output path (sitl/ap_dds/,
  docs/RUNBOOK-ap-dds.md). Stage 1 flew on ArduCopter 4.5.7 DDS SITL on 26
  Nov (dev topology, not KPI-grade; docs/data/ap_dds_20261126/): the
  Shield's setpoints reached the autopilot only over /ap/cmd_vel on one
  clean route flight with no zone engaged; state, modes, arming, take-off,
  params and the fence stay on MAVROS 2, which remains the ROS 2 bridge to
  ArduPilot and the KPI rail. Stage 2 is not possible on 4.5.7 (no
  mode/armed/landed state, no param service).
- **Topologies** (ARCH-11 to ARCH-14), as delivered:

| Topology | Grant definition | As delivered |
|---|---|---|
| `dev` | everything on one desktop; not used for reported KPI numbers | flown: SITL + MAVROS 2 (7 Oct); PAS + ArduPilot + MAVROS 2 in WSL, `pas_dev_mavros_02` (29 Oct); Gazebo over pymavlink and MAVROS 2 (2 Nov); in containers, `dev_compose_01` (6 Nov). The Gazebo sweeps and the AP_DDS stage 1 fly it. `is_kpi_grade` refuses a `dev` run: no dev waiver is recorded, and every KPI in section 5 comes from `hil`. |
| `hil` | desktop runs the simulators, MAVROS and the GCS; the Orin runs the VLA and the Shield | the canonical KPI topology, flown since 6 Nov on the AGX Orin: SITL and mavlink-router on the desktop; MAVROS 2, the Shield and the stub pilot on the Orin. Under reading (h) the stub-pilot KPI campaigns are the canonical `hil` topology, and the learned VLA flies in separate `hil` flights: AIoT-VLA on the Project AirSim perception rail since 14 Nov (ARCH-20). First KPI-grade run hil_smoke_03 (6 Nov); KPI-grade nightlies ntFIN2 (116214f), ntFIN3 (71aabb3), ntFIN4 (59e19a5) and ntFIN5 (21c2771, release 0.7.0, the figure of record); the release smokes smREL070 to smREL073, 52 KPI-grade each, the last at the delivered release 0.7.3 (c44a360, 28 Nov). |
| `flight` | Orin companion on a real ArduPilot drone | covered by a written waiver (reading A3, section 9.2; ARCH-14 DONE). No airframe flight is claimed: the drone cannot carry an AGX Orin. The `hil` rig (AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2) and a bench test of the real flight controller stand in for it. Bench, 17 Nov: CUAV V5 Nano on ArduCopter 4.7.1, never armed; the fence parameters set and read back, the bundle's zone uploaded and read back matching; GUIDED, BRAKE, LOITER, RTL, LAND and STABILIZE accepted, each confirmed by the next heartbeat (docs/data/fc_bench_20261117/README.md). `hil` to `flight` is a configuration change on the same images (deploy/topologies/flight.env, deploy/compose/docker-compose.orin.flight.yml). The FC stays on 4.7.1, not flashed to the rig's 4.5.7, so bit-identity between SITL and firmware is a stated limit (reading A4; ARCH-31). |

- **Rails.** ArduPilot SITL + MAVROS 2 (the KPI rail); Project AirSim +
  ArduPilot (the perception rail; a learned VLA on it on `hil` since 14
  Nov); Gazebo Harmonic (flown end to end on 2 Nov, ARCH-16; the grant's
  four-step mid-term demo gate on 25 Nov at 75a6d45, 3/3, ARCH-16; the full
  gazebo_functional profile at release 0.7.2, 418b151, on 27-28 Nov: 131 of
  132 missions flown, P0 escape 0 on 128 Shield-on, functional only, never
  KPI-grade; docs/data/gazebo_sweep_20261128/README.md, WP4-04). All three
  fly ArduPilot as the inner loop (ARCH-02). The broad sweep (WP4-06) runs
  on the headless backend (topology headless-kinematic, a modelled
  autopilot), never KPI-grade: at release 0.7.2, 14,950 episodes
  (docs/data/broad_REL072/README.md); at 0.7.3, docs/data/broad_REL073/README.md: the same 14,950 episodes in 43.2 min, P0 escape 0.0, fail-safe labels 14,945/14,945, every episode's status, outcome and KPI record equal to 0.7.2's. Project AirSim CityLife with
  simple_flight is a comparison bench (the follow comparisons of section
  6), not a rail of the delivery and never KPI evidence.

![What runs where in the hil topology](img/what-runs-where.png)

*What runs where in the `hil` topology (the 11 Nov progress-deck figure): the desktop PC runs Project AirSim and ArduPilot SITL 4.5.7 with mavlink-router and the scenario tools; the AGX Orin runs the pilot (AIoT-VLA with its OWL-ViT detector, or the stub), the Shield with its escalation FSM, and MAVROS 2 in the companion container. The dashed boxes (Project AirSim, AIoT-VLA) were built but not yet flown on the rig when it was drawn; they flew on 14 Nov (section 6; docs/data/pas_aiot_hil_20261114/flights.md). The KPI campaigns fly the solid boxes with the stub pilot.*

## 4. Results by work package

Each subsection: what was delivered, the readings the project applies where
the grant is open, and the stated limits. Every WP1-WP4 row of Appendix A is
DONE; the per-obligation rows and their evidence are there.

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
  policy stored in local metres (a declared read of a policy the lock
  records excepted); since 26 Nov the DSL loaders, REST, gRPC HotApply, the
  Ingest CLI and the Shield's events refuse metre input too (a tool that
  works in metres converts it first with models.wgs84_from_metres). WP1-14
  is DONE since 26 Nov, after an independent review of the third fix; its
  one dissent was closed by a fourth fix.
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
- Signed bundles under a CA (WP1-04, X-10): the Lab CA is the bundle
  signing authority since 27 Nov (authority lab-ca; the 17 flight bundles
  re-signed on both hosts, policy_hash unchanged; the chain required on
  `hil` and `flight`; docs/data/bundle_ca_20261127/). The `hil` mission
  ca1127_001 flew KPI-grade with the chain verified and required, and so
  did every run of the release smokes smREL072 (27 Nov) and smREL073
  (28 Nov), 52/52 each (docs/data/kpi_smREL072/, docs/data/kpi_smREL073/).
  The KPI runs of record (ntFIN4, ntFIN5, smREL070, smREL071) flew before
  27 Nov under the bare lab-dev keys. The move to ITRI's CA is written down
  and tested (docs/RUNBOOK-bundle-ca.md).
- Compatibility with the reference implementation (WP1-05) is read as the
  two-hash contract in place of a byte-identical match. At release 0.7.3,
  over the 33 shipped policies: both hashes recorded and verified 33/33, a
  forged reference hash refused 33/33, and the reference's own loader reads
  our reference-form bundles of its 3 documents and computes the same hash
  3/3 (docs/data/wp1_roundtrip_c44a360.json). Stated limit: no shipped
  policy fits whole in the reference Phase-1 DSL (representable 0/33), so
  the reference loader cannot fly a KPI bundle.
- The policy-artefact KPIs at the delivered release 0.7.3 (c44a360), from
  the offline tools: round trip 33/33, tamper 33/33, negative 39/39, cold
  start p95 477 ms on the desktop with the certified signer's chain checked
  (docs/data/wp1_roundtrip_c44a360.json, copied to
  docs/data/wp1_roundtrip.json). On the AGX Orin: cold start of a 50-rule
  bundle to a ready Shield p95 784 ms against 1 s, with tampered, untrusted
  and bad-signature bundles refused
  (deploy/evidence/hil/wp1_roundtrip_agx_c44a360.json); hot-apply at 50
  rules p99 4.97 ms against 50 ms
  (deploy/evidence/hil/bench_shield_50rules_5s_agx_c44a360.json).
- distance_envelope is enforced for buildings, roads and, since 19 Nov,
  every person the rail reports (WP1-07). People are kept at the envelope
  as far as the person detector sees them, a stated limit: on
  pas_hil_people_02 the person feed matched 0.189 of pedestrians within
  25 m, and a person first reported inside 4 m is repaired from and
  escalated, not held out (docs/data/people_hil_20261119/README.md).

### 4.2 WP2 Prefix Compiler (19 obligations)

- Delivered: the CSP model and schema, the token budget (WP2-02), P0
  coverage (WP2-03), relevance explanations, the adapters, templates and
  tests. The paraphraser is a separate gRPC service with mutual TLS since
  2 Nov (WP2-13).
- At release 0.7.3 (c44a360) the CSP keeps P0 coverage 124/124 over the 33
  shipped policies at every budget from 256 down to 144 tokens, where the
  naive cut keeps 122/124; the largest CSP is 149 tokens
  (docs/data/csp_kpis.json).
- On 7 Nov OpenVLA-7B flew with the CSP in its prompt: an A/B on three seeds
  and eight paraphrases from the paraphraser service (AirSimNH, not
  KPI-grade; WP2-01, WP2-09, WP2-18, WP4-10). On base OpenVLA the CSP's
  effect on repairs is no larger than a rule-free prefix's (CHANGELOG
  2026-11-07 (CSP flights)).
- The WP2 and WP4 correction notes were sent to ITRI on 2 Nov (Appendix A,
  WP2-12 and WP4-21).
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
- KPI-grade on `hil` (ntFIN5 at release 0.7.0, 21c2771, as in ntFIN4 at
  59e19a5, ntFIN3 at 71aabb3 and ntFIN2 at 116214f before it): P0 escape
  rate 0 (WP3-14); fail-safe trigger correctness 649/649, Wilson 95 % low
  0.994 (WP3-15); the tracked KPIs repair magnitude, time to safe and
  repair success (WP3-16, WP3-17, WP3-20); section 5.
- The tick budget (WP3-13) is read as shown by the stub-pilot `hil`
  campaigns plus the offline 50-rule replay on the Orin. ntFIN5: every one
  of 183,961 ticks within 100 ms (max 62.18 ms end to end; section 5.4).
  smREL073 at release 0.7.3: tick_ms p99 25.5 / max 55.73 ms, 0 of 16,047
  over 100 ms; e2e_ms max 55.81 ms; the bag's /vla/action_4d to
  /mavros/setpoint_raw/local hop max 57.69 ms (docs/data/kpi_smREL073/).
  On the AGX Orin at c44a360, 50 rules x 51 poses over 1,301 real flight
  ticks: the geometry query (_check) p99 1.048 ms, max 1.097 ms against
  5 ms, and the whole filter step p99 27.98 ms, max 50.03 ms against
  100 ms, 0 over either
  (deploy/evidence/hil/shield_tick_profile_20261128.json).
- The grant's five-row Shield test matrix (high-speed near-edge, sudden
  NFZ hot-applied, time-window switch, three simultaneous violations, GPS
  noise / latency) flew whole first in smFIN2 at 59e19a5
  (docs/data/kpi_smFIN2/), and then in each release's own KPI-grade smoke:
  smREL070 (0.7.0, 21c2771), smREL071 (0.7.1, a3d616e), smREL072 (0.7.2,
  418b151) and smREL073 (0.7.3, c44a360) (WP3-19, WP4-14). In smREL070 and
  smREL072 the two at-speed near-edge cells flew outside the school-yard
  zone's time window and were vacuous; release 0.7.3 pins their scenario
  clock, and its release gate requires a non-vacuous cell in every row and
  in every planned at-speed cell. smREL073 (28 Nov, 52 KPI-grade missions,
  single_sha 52/52) met it: smREL073_041 and _042 had 10 and 13
  P0-violation ticks, P0 escape 0 on both; overall P0 escape 0 on 2,393
  P0-violation ticks, fail-safe (r2) 50/50 (docs/data/kpi_smREL073/).
  Since 115640b (21 Nov) the GPS noise and latency are applied on the
  companion to the pose the Shield and the FSM read (the autopilot's EKF
  sees clean GPS), checked per run (rail_stress_check).
- One rule-evaluation code path (WP3-22): FenceGuard, the last flown second
  measure, measures with the Shield's field and judges with its
  clearance_breached; "grid" is refused (wf21-onepath3, 18 Nov; flown on
  7 + 2 AGX Orin follow flights, 19-20 Nov, functional). X-13's "nothing
  else evaluates rules" is read with the autopilot's GeoFence backstop and
  the listed non-verdict checks outside the system's rule evaluation: the
  backstop, which the grant keeps, triggered 18 of ntFIN5's 649 scored
  fail-safes, and since 27 Nov the start_conflicts lint asks the Shield
  library's closed zone_footprint_reached, the in-flight verdict staying
  zone_breached.
- Escalation FSM edges taken in smREL073 at release 0.7.3 (52 missions;
  each episode's metrics.json `fsm`; Appendix A WP3-07): G1 41, G3 38,
  G5 19, G9 19, G8 18, X1 10, G7 8, G10 4, G6 3, G11 2, G12 1, X4 1, X5 1,
  X8 1, X9 1. X6 and X7 flew on `hil` before (rh01, rh03, 6-7 Nov).
- The edges no rail had taken flew on 14 Nov in hs10 (8a473fa, `hil`,
  profile `fsm_edges`, one seed per cell; demo/out/hil_hs10/sweep.json, runs
  demo/out/hil_smoke/hs10_*; CHANGELOG 2026-11-14 (hs10 ...)): G2 and G4 on
  fsm-n-in-t-flicker (hs10_003: X1, G2 at 5.8 s "N-in-T: 3 onsets in 5 s",
  G4 at 10.0 s, RTL, G9); X2 (hs10_001) and X3 (hs10_002) from a hold; G6
  on a blocked leg home (hs10_004: G11, G6, G10) and on no progress home
  under wind beyond authority (hs10_005: G1, G3, X9, G6, G10; no progress
  home is our RTL-failure reason, not one the grant names). G6's input is
  the node's RTL watch (`RTLFailureWatch`, sitl/ros2_shield_node.py;
  CHANGELOG 2026-11-14 (WP3-07)): the Shield's check of the leg home
  against its hard P0 zones, and progress home while ArduPilot reports RTL.
  P0 escape 0 and 0 polygon entries over the 5 missions; `e2e_ms` max
  41.3 ms over 423 setpoint ticks (2,181 rows). G6's battery reason flew on
  25 Nov (bat1125_001: G11, G6, G10, Land), and a profile's FSM YAML is
  selectable on the rail (fsmp1125_001, profile `grant`;
  docs/data/hil_1125_x11_wp307/README.md). fsm-n-in-t-flicker is labelled
  by the grant's G4 (rubric clause E6; section 9 decision 13).
- The FSM's Loiter is held in GUIDED by default (WP3-07): since 7 Nov the
  companion holds it (node default `--hold-flight guided`,
  sitl/ros2_shield_node.py:6915); hs10_003 logs "LOITER held in GUIDED by
  the companion" at t = 5.828 s with mode requests ["RTL"]; in smREL073, 27
  missions entered Loiter and none requested LOITER (GUIDED 18, RTL 23,
  LAND 31, mission-end landings included).
- Literal SET_MODE LOITER is opt-in (`--rail-hold loiter`, 29a1440) and
  flew on `hil` on 22 Nov: sk1122b at 59e19a5, 2 missions; on both the
  autopilot's heartbeat reported LOITER, the Loiter held 29.9 s with an
  altitude deviation of 0.021 m and 0.005 m, then the 30 s timeout took G5
  to RTL; P0 escape 0 (docs/data/shakedown_20261122/README.md).
- Repair selection (WP3-06): the iterated repair chain is the default; the
  grant's first-feasible rule is available, off by default
  (`--repair-mode first_feasible`); PathRepair's boundary and zone detours
  are opt-in. In a headless A/B (179 Shield-on episodes per arm, not
  KPI-grade) the chain had fail-safe 179/179 and first_feasible 177/179,
  both with 0 P0 escapes (docs/data/repair_mode_ab_20261127/README.md).
  The PAS rail publishes its own setpoint by default, the adapter route
  being opt-in (WP3-09). Since 35df373 (26 Nov, release 0.7.1, flown in
  smREL071) the Shield's whole stack carries the body action (WP3-04).
- Stated limits: literal LOITER holds where the aircraft stopped, not at
  the Shield's moved hold point; on SITL it relies on a throttle stand-in
  for the safety pilot (RC_CHANNELS_OVERRIDE ch3 1500 as sysid 255), and it
  does not run under SROS 2 (the adapter is not granted
  /uas1/mavlink_sink). The follow and perception rails fly the repair
  chain without PathRepair, and PathRepair does not detour around a
  corridor. No flight served a learned VLA beside a ~50-rule bundle, and
  the pilot-to-node DDS hop and the MAVROS-to-autopilot MAVLink hop are not
  timed; learned-VLA flights on the PAS perception rail had 4 of 27,582
  ticks over 100 ms (max 142.85 ms;
  docs/data/kpi_prefix_vla_20261120/README.md).

### 4.4 WP4 Stress Testing (21 obligations)

- Delivered: the scenario library (WP4-02), the KPI report generator
  (WP4-07), the Shield audit in every harness bundle (WP4-20), mission
  success on `hil` (WP4-08), the six-field manifest on every KPI-bearing
  run (WP4-12), the paraphrase arms through the service (WP4-10), the
  replay harness (WP4-11).
- The grant's worked example (`grant_example` profile, 36 episodes) flew on
  `hil` as ge1110 at 9eed35b: 32 missions flew and reached W3 with no repair
  and no unsafe spell; 4 died before the mission began, a fence-raise race
  fixed in 183bb3e (CHANGELOG 2026-11-11 (hil rig flights, 7-11 Nov) and
  (fence read and take-off)). Its "mean time to safe ≤ 2.0 s" holds only
  vacuously: no episode is ever unsafe (CHANGELOG 2026-11-07 (grant example
  sweep)). The same sweep flew again at 2f22116 as ge1113 (13-14 Nov, 36
  missions, all KPI-grade; docs/data/kpi_ge1113/): 36 passed, mission
  success 27/36, no P0-violation tick, no repair and no unsafe spell, so
  its P0 rate is vacuous and its fail-safe labels (36/36, all "no
  fail-safe") do not discriminate. It is latency evidence at the fixed
  commit (section 5.6), not Shield evidence.
- Episode bundles (WP4-04): every episode of the hil campaigns of record
  is one bundle under the grant's six names (manifest.json,
  policy_bundle.tar.gz, episode.bag2/, shield_audit.jsonl, empty on a
  zero-repair run, harness_events.jsonl and kpi.json): ntFIN4 and ntFIN5
  (654 each), smFIN2 and smREL070 to smREL073 (52 each). Since release
  0.7.3 the manifest start frame goes out at arming with all six fields
  (smREL073 52/52, the bag's start frame identical to manifest.json 52/52),
  and episode.bag2 is trimmed to begin at it in 49 of 52 bundles; the other
  3 keep the recorder's bag, because one /vla/action_4d message lay between
  the arm acknowledgement and the frame, with the reason in kpi.json
  (ARCH-22, which stays PARTIAL: no bag holds an image topic). From the
  ntFIN5 bags alone (tools/replay_from_bag.py,
  docs/data/replay_from_bag_ntFIN5/; the Shield is not re-run):
  manifest_in_bag and trajectory identical in 654/654, ticks and KPI in
  630/654, the 24 that differ being the gps-noise-beyond-margin runs at the
  escalation tick's inferred setpoint.
- Gazebo Harmonic functional sweep (WP4-04): the full gazebo_functional
  profile at release 0.7.2 (418b151) flew 131 of 132 missions, each
  manifest recording code_revision 418b1517da9f; P0 escape 0 on 128
  Shield-on missions; desktop tick_ms p99 6.48 / max 25.06 ms,
  e2e_with_wait_ms max 97.36 ms, 0 over 100 ms. One cell seed was refused
  before arming by the rail's home check on all 4 attempts, so it has no
  flight. Functional only, never KPI-grade
  (docs/data/gazebo_sweep_20261128/README.md).
- Release smoke (WP4-14): "every release" is read as every release from
  0.7.0 on, KPI-grade on `hil` per release and headless per push. smREL070
  (23 Nov), smREL071 (26 Nov), smREL072 (27 Nov) and smREL073 (28 Nov)
  each flew the five rows at their release commit, KPI-grade, P0 escape 0
  on each (section 4.3); v0.6.0's rel060 had flown four rows. The headless
  smoke runs on every
  push in GitHub's `tests` workflow (green in all 3 jobs on the 0.7.2 push,
  runs 45969020953 and 45969019967; docs/data/github_ci/) and in local CI
  (16bb416, code-identical to c44a360: 194/194 files, smoke rc 0 on 64
  episodes; docs/data/ci_local/16bb4168b391.json); it is functional, never
  KPI-grade.
- Cadence (WP4-06): the nightly KPI campaign is flown on the one rig for
  the release the KPI report of record describes (docs/RUNBOOK-release.md
  section 7: the nightly "is not part of the per-release obligation"):
  ntFIN5 at release 0.7.0 (77,356 s), after ntFIN4 at 59e19a5; releases
  0.7.1-0.7.3 flew their KPI-grade smokes and no nightly. The broad sweep
  runs on the headless backend and was run in full at releases 0.7.2 and
  0.7.3. Broad sweep at release
  0.7.2: 2,990 cells x 5 seeds = 14,950 episodes in 43.8 min on 12 shards,
  P0 escape 0.0, fail-safe labels 14,945/14,945
  (docs/data/broad_REL072/README.md); at release 0.7.3: docs/data/broad_REL073/README.md: the same 14,950 episodes in 43.2 min, P0 escape 0.0, fail-safe labels 14,945/14,945, every episode's status, outcome and KPI record equal to 0.7.2's. On
  28 Nov .github/workflows/nightly.yml gained the jobs `broad` (12 shards)
  and `broad-merge` beside the nightly job (pinned by
  tests/test_harnesstruth_ci.py). GitHub fires a schedule only from the
  default branch `main`, so neither nightly.yml job has run on GitHub.
- Replay (WP4-11): open-loop replay of the logged inputs at the flown
  revision (the audit's `ts` and the FSM's fsm_* fields left out), on the
  runtime the episode flew on (AGX Orin, hil companion container, Python
  3.12.3). Every KPI-grade `hil` run flown from 22 Nov on is identical:
  1,660 episodes, 0 differences. ntFIN5 654/654
  (docs/data/replay_agx_ntFIN5/); smREL073 52/52, and 52/52 again from the
  52 episode bundles alone with no run folder in reach
  (docs/data/replay_agx_smREL073/); ntFIN4, smFIN2, smREL070-smREL072,
  pdyn1126, wgs01_001, ca1127_001, the 22 Nov shakedowns sk1122a and
  sk1122b, and the 25 Nov runs x11r1125, bat1125 and fsmp1125, 954/954
  (docs/data/replay_agx_rest_20261128/README.md). ntFIN5 replayed at its revision
  stands for the Q3 matrix. On the desktop (AMD64, Python 3.10) replays
  differ in the last float digits (smREL073 42/52 identical). Stated limit:
  a `hil` episode is not re-flown closed loop, since SITL and the network
  are not bit-reproducible.
- KPI report (WP4-17): ntFIN5's KPI report is the "KPI report on
  dynamic-scenario stress", with the prefix A/B from pab3
  (docs/data/kpi_prefix_ab_20261119/). The render of record,
  docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md, was
  made from a clean tree with the WP1 and WP2 records at c44a360 (release
  0.7.3) and the 0.7.3 Shield-matrix section: every row non-vacuous, 12/12
  planned near-edge-at-12mps cells non-vacuous, gate met. Its KPI figures
  are unchanged (section 5). It describes release 0.7.0's code; releases
  0.7.1-0.7.3 are covered by their own KPI-grade smokes.
- KPI source (WP4-01): the KPI campaigns flown with the stub pilot on the
  Orin (Shield, harness, signed bundle) are the canonical `hil` topology,
  and the policy-artefact KPIs come from the offline tools at the delivered
  revision (sections 4.1, 4.2). The learned VLA, and with it per-paraphrase
  robustness (the stub reads no text), is shown in separate `hil` flights:
  AIoT-VLA v10b, 6/8 paraphrases ended as the source did, 0 P0 escape ticks
  over 13 runs (docs/data/pas_hil_paraphrase_20261115/flights.md).
- Final demo gate (WP4-19): the five final-demo scenarios of section 9.1,
  all flown on `hil`, plus pdyn1126 as the non-trivial scenario (26 Nov,
  release 0.7.1, a3d616e). In the suite cell dyn-nfz-spawn-ahead with the
  prefix on, pdyn1126_001 and _003 exercise all four components in one
  flight: the signed policy bundle, the new generation's CSP compiled and
  delivered to the pilot, the FSM's Brake and return to Normal, and the
  suite scenario with its scorer and six-name episode bundle. Campaign
  12/12 KPI-grade, 10 of 12 gates pass, P0 escape 0 on 67 P0-violation
  ticks, tick_ms max 52.14 ms (docs/data/kpi_prefix_dyn_20261126/);
  replayed 12/12 on the AGX Orin (docs/data/replay_agx_rest_20261128/).
  Stated limits: the zone event is
  applied by the Shield node, not sent over REST (REST hot-apply in flight
  is shown in wgs01_001, CHANGELOG 2026-11-26); the two
  corridor-swap-narrowed prefix-on arms failed their gate, momentum under
  Brake carrying the aircraft 0.81 / 0.64 m outside the narrowed corridor
  before the Shield flew it back; the CSP is read by the hand-written
  prefix consumer, not by a learned VLA.

## 5. KPI report (WP4-17)

### 5.1 Source

The nightly `hil` campaign ntFIN5 at release 0.7.0 (21c2771; section 2):
654 KPI-grade missions, 651 Shield on and 3 Shield-off controls,
single_sha 654/654. The KPI report of record is
`docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md`
(and .json), made by tools/kpi_report.py from a clean tree at f9cab68 with
the ntFIN4 report as its baseline. Its flight KPI figures are those of the
first render of 24 Nov (b171c57), which is kept beside it with the render
of 27 Nov. Its policy-artefact KPIs (the WP1 round trip and the WP2 Prefix
Compiler KPIs) are the records measured at the delivered release 0.7.3
(c44a360), and it carries the 0.7.3 Shield test-matrix section, whose gate
is met (section 5.2; docs/data/kpi_ntFIN5/README.md, "Re-rendered 28 Nov").
Every flown episode is re-scored from its own log with
`guardrail.kpi.compute`, the function the flights use. Fail-safe labels
come from the committed label artefact docs/data/failsafe_labels_rail.json,
re-scored with `experiments/sweep_scenarios.py --profile nightly
--rescore-rail` (docs/data/kpi_ntFIN5/rescore_r2_ntFIN5.json). The earlier
figures of record, ntFIN4 at 59e19a5 (docs/data/kpi_ntFIN4/) and ntFIN3 at
71aabb3 (docs/data/kpi_ntFIN3/), are quoted beside it where a comparison
helps. Releases 0.7.1 to 0.7.3 each flew their own KPI-grade `hil` smoke,
52 missions each (smREL071 to smREL073; docs/data/kpi_smREL071/ to
kpi_smREL073/); smREL072 (0.7.2) and smREL073 (0.7.3, c44a360) are quoted
beside ntFIN5 below.

### 5.2 P0 violation escape rate

- 0.0000 over 651 Shield-on flights: 0 escapes on 17,194 P0-violation
  ticks (ntFIN4: 0 on 17,078 over 650; ntFIN3: 0 on 17,457 over 591).
  Release smokes: smREL072 0 on 2,378, smREL073 0 on 2,393, over 51
  Shield-on flights each (docs/data/kpi_smREL073/README.md).
- Shield-off control (nfz-head-on-control, 3 flights): 0.6500, 156 escapes
  on 156 P0 ticks. Passthrough null: 0.1938.
- Every row of the grant's Shield test matrix has a non-vacuous cell, one
  whose raw commands had P0-violating ticks for the Shield to act on
  (high-speed near-edge 30 of 30 cells, dyn-nfz
  spawn 36 of 42, time-window switch 7 of 13, three simultaneous violations
  18 of 18, gps noise / latency 16 of 20). The 12 planned
  near-edge-at-12mps cells all flew non-vacuous (20 to 60 P0 ticks each).
  The matrix gate of docs/RUNBOOK-release.md step 5 is met (the KPI report
  of record, lines 48-64). smREL073 meets it too: near-edge-at-12mps 2 of 2
  cells non-vacuous (smREL073_041 10 P0-violation ticks, smREL073_042 13).
- The count is the Shield's own log, as the grant defines it ("Counter from
  Shield repair log"). On the true position, the KPI report's table "What
  the autopilot flew on the rail runs" lists the moves ArduPilot made into
  an unsafe position after the FSM handed the aircraft to RTL, Land or the
  hold. In 23 Shield-on missions it entered a P0 polygon: the 18
  dyn-nfz-spawn-ahead runs whose GeoFence breach took them to RTL
  (ntFIN5_178-186, ntFIN5_214-222; section 5.3) and 5 wind-beyond-authority
  missions at 12 m/s (ntFIN5_647-648 in lane 15, ntFIN5_652-654 in lane
  20; ntFIN4 had 4). In 10 more it breached a keep-out margin without
  entering the polygon (9 dyn-nfz-spawn-ahead, 1 failsafe-rtl-zone). These
  are outside the grant's escape count. The RTL drift in a 12 m/s wind,
  first shown by the shakedown hs07, is a stated limit (section 10).

### 5.3 Fail-safe trigger correctness

| Reading | Correct / scored | Point (the KPI) | Wilson 95 % low | Nulls: always / never |
|---|---|---|---|---|
| (a) ideal-vehicle labels | 574/649 | 0.884 | 0.858 | 0.418 / 0.582 |
| (r) rail-vehicle labels | 637/649 | 0.982 | 0.968 | 0.487 / 0.513 |
| **(r2) rail-vehicle labels, E5 on the prescribed flight, with the GeoFence backstop rule (the reading applied; the KPI)** | **649/649** | **1.000** | **0.994** | 0.505 / 0.495 |
| (r2), the 18 backstop runs read as before the rule (left out as censored) | 631/631 | 1.000 | 0.994 | - |
| (r2), the 18 backstop runs counted as not triggered | 631/649 | 0.972 | 0.957 | - |

- The verdict: 1.000 meets ≥ 99 % as the point estimate, which is the
  grant's figure, and its Wilson 95 % lower bound, 0.994, clears 0.99, so
  this campaign shows 99 % at 95 % confidence. It does so without the
  backstop rule too (631/631, Wilson low 0.994). ntFIN4 at 59e19a5 cleared
  the bound too (648/648, 0.994); ntFIN3 at 71aabb3 (587/589, 0.988) did
  not. Two consecutive nightlies clear it. The release smokes smREL072 and
  smREL073: (r2) 50/50 each, Wilson 95 % low 0.929, 1 censored.
- No error under (r2), as in ntFIN4. ntFIN3's two errors, missed triggers
  on dyn-nfz-spawn-ahead (ntFIN3_164, ntFIN3_168), did not recur.
- Left out: ntFIN5_373 and ntFIN5_374, corridor-swap-shifted, censored in
  Loiter. Were both errors, 649/651 = 0.997.
- 18 fail-safes were triggered by the GeoFence backstop while the FSM held
  in Loiter (decision 14): all dyn-nfz-spawn-ahead (ntFIN5_178-186,
  ntFIN5_214-222; RTL at t = 5.73-5.87 s), all expected, all correct.
- On the labels the campaign flew with: 586/649 = 0.903 (54 false, 9
  missed).
- failsafe-rtl-zone ends in Land on most seeds, as in ntFIN3: the node's
  RTL watch finds the leg home through the zone and lands (G6); the
  fail-safe is correct either way, and the scenario's outcome gate, written
  for RTL, fails them.
- The rules applied: (r2) since 10 Nov (docs/RUBRIC-failsafe.md sections
  7-8; CHANGELOG 2026-11-10 (fail-safe labels adopted)), the gps-dropout
  labels as re-derived on 12 Nov, and the GeoFence backstop rule since
  15 Nov (docs/RUBRIC-failsafe.md section 9). All three were committed
  before ntFIN5 flew, so ntFIN5 was scored on rules fixed before its
  flights.

### 5.4 Tick budget

| Measure | Ticks | p50 | p95 | p99 | Max | Over 100 ms |
|---|---|---|---|---|---|---|
| `tick_ms` (tick start to setpoint) | 183,961 | 7.8 | 23.1 | 28.3 | 62.08 | 0 |
| `e2e_ms` (action receipt to its first setpoint) | 183,961 | 7.9 | 23.2 | 28.4 | 62.18 | 0 |
| `e2e_with_wait_ms` (the same, plus any wait behind the previous tick's post-setpoint work) | 183,961 | 7.9 | 23.2 | 28.4 | 62.18 | 0 |

- The budget is met on every tick: 0 of 183,961 over 100 ms, maximum 62.18
  ms end to end. Each release smoke had none over 100 ms either: smREL070
  at the same revision 0 of 16,002 (`e2e_ms` max 57.49 ms), smREL071 0 of
  16,046, smREL072 0 of 16,150 (`e2e_ms` max 59.43 ms) and smREL073 at release 0.7.3
  (c44a360) 0 of 16,047 (`tick_ms` p99 25.5 / max 55.73 ms, `e2e_ms` max
  55.81 ms; docs/data/kpi_smREL070/ to kpi_smREL073/README.md).
- The tick fix. ntFIN4 at 59e19a5 had one tick of 183,909 at 105.5 ms
  (`ntFIN4_236_dyn-nfz-spawn-ahead`, row 50, t 5.03 s, FSM edge G3): the
  tick on which the scenario's zone was hot-applied, where the node wrote
  the new policy generation (policy_g1.json), ran the CSP compile with its
  first, cold template load (csp_g1.json) and published the fences, all
  before the setpoint; that block, usually 20-40 ms, took 87 ms there, an
  I/O or scheduling stall on the Orin (docs/data/kpi_ntFIN4/README.md, "The
  one tick over 100 ms"). The fix (7e6aae2, review fixes a0eb767; CHANGELOG
  2026-11-23 (code)) queues that bookkeeping after the setpoint and drains
  it at the end of the tick, after the row and the audit; a failure there
  is recorded (`deferred_failed`); the templates are compiled at bring-up;
  fences are published once per tick; each row carries per-phase stamps
  (`tick_phases_ms`) and the wait a tick could have spent behind the
  previous tick's deferred work (`blocked_by_deferred_ms`, an upper bound,
  folded into `e2e_with_wait_ms`).
- Flown in ntFIN5: the 249 ticks that hot-applied a rule reached their
  setpoint in median 14.3 ms, max 62.08 ms (ntFIN5's slowest tick is one of
  them); the deferred work after the setpoint took median 9.6 ms, max 20.5
  ms; `blocked_by_deferred_ms` is 0 on every row, so no action waited
  behind it (read from the 654 runs' flight_log.jsonl `tick_phases_ms`).
  WP3-13's condition, the fix committed and a KPI-grade nightly with
  hot-applied zones flown with no tick over 100 ms, is met.
- The ticks that published no setpoint. The rows above count setpoint
  ticks only; ntFIN5's other 119,087 monitor ticks (RTL, Land, and the
  SET_MODE ticks that published no setpoint) carry no `tick_ms`. Read from
  their `tick_phases_ms` (tools/nonsetpoint_ticks.py,
  docs/data/ntFIN5_nonsetpoint_ticks/), tick entry to the end of the
  publish phase, an upper bound of entry to the mode request, was p50 6.85
  / p99 19.59 / max 41.70 ms, and the whole tick p99 22.01 / max 87.76 ms:
  0 over 100 ms. 758 ticks carried an FSM SET_MODE. The 281 that published
  no setpoint took at most 41.70 ms to the end of the publish phase. The
  other 477 published a setpoint and are in the 0 of 183,961 (tick_ms max
  45.86 ms). The hops after the node (the adapter, MAVROS, the autopilot)
  are not timed.
- "End to end" starts at the Shield node's receipt of the pilot's message,
  not at a camera frame. `cam_age_ms` and `cam_to_setpoint_ms` are not
  measured on this rail: no camera is in the `hil` stub loop. Measured 14
  Nov with AIoT-VLA on the AGX Orin
  (docs/data/pas_aiot_hil_20261114/flights.md): camera frame arrival to the
  first setpoint p50 159-163 ms, p99 282-296 ms over six missions; the
  VLA's own inference (capture to answer p50 0.110-0.113 s) and the
  detector account for it.
- With a learned VLA on the Orin, outside the KPI campaigns, ticks did
  exceed 100 ms: pab_aerialvla_04 (19-20 Nov, bfb4973, the PAS perception
  rail, AerialVLA and OWL-ViT sharing the AGX Orin, 12 flights, KPI-grade
  as flown and 0 KPI-grade under the single_sha gate of 27 Nov, since the
  PAS script outside the harness records no desktop revision) had 4 of
  27,582 over (max 142.85 ms, tick p99 47.2 ms); on each the Shield step
  itself took 71.5-85.3 ms on a clearance repair, and the cause is not
  identified (docs/data/kpi_prefix_vla_20261120/README.md);
  pas_hil_feed_07 (15 Nov, AIoT-VLA) had 3 of 4,058 (152.5, 111.6,
  147.9 ms). Neither ran the ROS 2 Shield node of the KPI campaigns.
- AGX Orin profile (50 rules x 51 poses): 6 Nov at d0e91bb, query p99
  0.977 ms, Shield step p99 22.647 ms
  (deploy/evidence/hil/shield_tick_profile.json); 14 Nov at bc04e4a, query
  p99 0.994 ms, step p99 26.709 ms; 25 Nov at release 0.7.0 (21c2771, no
  VLA server), query p99 1.056 ms, max 1.100 ms, step p99 28.520 ms, max
  51.057 ms (deploy/evidence/hil/shield_tick_profile_20261125.json); 28 Nov
  at release 0.7.3 (c44a360, no VLA server), over 1,301 real flight ticks,
  query p99 1.048 ms, max 1.097 ms, step p99 27.98 ms, max 50.03 ms, 0 over
  either budget (deploy/evidence/hil/shield_tick_profile_20261128.json).

### 5.5 Tracked KPIs per family (Shield on, KPI-grade)

| Family | Episodes | Mission success | Mean time to safe (s) | Repairs / ep | Repair magnitude (m/s) | Repair success |
|---|---|---|---|---|---|---|
| all Shield-on flights | 651 | 288/651 | 3.49 (max 9.90), +94 never recovered | 50.2 | 6.329 | 98.2 % |
| altitude_envelope | 21 | 18/21 | 3.18 | 27.6 | 1.969 | 99.0 % |
| corridor_keep_in | 42 | 12/42 | never unsafe | 193.4 | 5.453 | 99.4 % |
| corridor_swap | 9 | 0/9 | 1.83, +1 never recovered | 29.2 | 2.234 | 89.7 % |
| dynamic_nfz_movement | 165 | 21/165 | 3.40, +18 never recovered | 24.0 | 3.320 | 94.2 % |
| failsafe_escalation | 27 | 0/27 | 0.13, +12 never recovered | 1.1 | 3.724 | 0.0 % |
| high_speed_near_edge | 90 | 90/90 | never unsafe | 174.6 | 7.654 | 100.0 % |
| kinematic_envelope | 3 | 3/3 | never unsafe | 300.0 | 16.000 | 100.0 % |
| radius_scaling | 15 | 6/15 | 3.91 | 6.1 | 5.657 | 83.7 % |
| sensor_degradation | 84 | 48/84 | none recovered, +4 never recovered | 9.5 | 5.561 | 98.5 % |
| static_nfz | 6 | 0/6 | never unsafe | 5.0 | 6.044 | 62.5 % |
| subject_standoff | 60 | 48/60 | never unsafe | 6.1 | 0.891 | 93.4 % |
| three_simultaneous_violations | 54 | 0/54 | 3.46, +54 never recovered | 8.0 | 10.294 | 0.0 % |
| time_window_static | 6 | 3/6 | never unsafe | 1.0 | 5.656 | 0.0 % |
| time_window_switch | 51 | 36/51 | 3.88 | 25.8 | 2.090 | 91.5 % |
| wind_disturbance | 18 | 3/18 | 6.75 (max 9.90), +5 never recovered | 5.1 | 5.656 | 83.5 % |

Nulls: a hover passes 0.0 % of the missions; an always-brake Shield's repair
success is 0.0 %. The ten worst failures are four wind-beyond-authority
missions at 12 m/s (ntFIN5_652-654 in lane 20, ntFIN5_647 in lane 15) and
six three-simultaneous-violations missions (ntFIN5_501, 502, 504, 507, 509,
514), all with the unsafe position never recovered, each linked to its
signed replay bundle in the KPI report.

Against ntFIN4, the per-family figures move by a flight or a few ticks:

- dynamic_nfz_movement has 165 KPI-grade flights (164 in ntFIN4, whose one
  dyn-nfz-translate run was refused by the setpoint-delivery rule); no
  ntFIN5 run was refused by that rule.
- sensor_degradation again has 84 Shield-on flights with the 60 GPS-noise
  / latency missions on the rail (noise drawn on the companion from each
  episode's seed, metrics.json `perception_stress`); mission success 48/84.
- PathRepair acted on the three standoff-approach-angles@offset_m=9
  variants with a pedestrian subject, as in ntFIN3 and ntFIN4: mean
  magnitude 0.685 m/s, P0 escape 0, fail-safe 3/3, mission success 3/3.
- No error under (r2) on any cell (section 5.3).

### 5.6 After ntFIN2: the start-up fix (history)

- Fix: 5f5009c (the node delivers its start-up backlog before t = 0 and
  holds it), merged to master in d7bb6a2.
- Shakedown hs09 at 2f22116 (`hil`, sim_speedup 1.0, clean revision
  2f221168625c): nfz-head-on, corridor-drift-angles and
  wind-beyond-authority, seed 1001; 17 missions flown of 22 planned, 5
  paraphrase / prefix arms refused (demo/out/hil_hs09/sweep.json; runs
  demo/out/hil_smoke/hs09_*). Over 7,083 ticks: `e2e_ms` p99 16.75 ms, max
  24.55 ms; `tick_ms` max 24.46 ms; no tick over 100 ms; row 2 of all 17
  missions ticked on the action; P0 escape rate 0 on every Shield-on
  mission. Fail-safe, re-scored with the label artefact
  (docs/data/kpi_ntFIN2/rescore_r2_hs09.json): (r2) 17/17 = 1.000, Wilson
  95 % low 0.816; (a) 16/17 = 0.941, the false trigger hs09_001
  (nfz-head-on canonical arm: labelled not to trigger, triggered RTL).
- A full campaign at the fixed node code (5f5009c; the Shield code is
  unchanged from 116214f, the diff touches only the node's start-up path),
  unloaded: the grant-example sweep ge1113 at 2f22116 (`hil`, 36 missions,
  all KPI-grade, 3 rules plus one hot-applied zone, 0 repairs, 0 Brake
  ticks; demo/out/hil_ge1113/sweep.json;
  docs/data/kpi_ge1113/kpi_report_ge1113_2026-11-14.md): over 95,160 ticks
  `tick_ms` p50 7.2 / p95 8.7 / p99 9.8 / max 66.28 ms and `e2e_ms` p50
  7.3 / p95 8.8 / p99 9.8 / max 66.37 ms; 0 ticks over 100 ms; each
  mission's maximum is its hot-apply tick. Its missions never came near a
  zone (no P0-violation tick, no repair), so it shows the tick budget at
  the fixed commit and nothing about the Shield's other KPIs (section 4.4).
- Loaded: hs09 (above) e2e max 24.55 ms; hs10 at 8a473fa (section 4.3) e2e
  max 41.3 ms over 423 setpoint ticks; hs11 (bc04e4a, dyn-nfz-spawn-ahead,
  16 KPI-grade missions) e2e p99 37.1 / max 66.08 ms over 5,121 ticks; the
  harness-driven hh02 (bc04e4a, 6 missions incl.
  radius-scaling-grow-deep) e2e p99 26.9 / max 83.57 ms over 2,669 ticks,
  its maximum a REST scale tick; 0 over 100 ms in each. ntFIN2 (116214f,
  the same Shield code) e2e max 74.14 ms outside the start-up rows
  (ntFIN2_364).
- `e2e_ms` spans the Shield node only, from the /vla/action_4d callback to
  /shield/setpoint; the pilot's DDS hop and the adapter's hop to MAVROS are
  not in it (the adapter hop, read from the runs' bags, is about 1.3 ms
  p50, 2 ms p99). Every `hil` figure has the stub as pilot and no VLA on
  the Orin; ~50 rules appear only in the offline Orin profile (section
  5.4).
- ntFIN2's 13 start-up ticks stay in its committed KPI reports as flown,
  before the fix. The nightly at 71aabb3, ntFIN3, flew 0 of 174,186 ticks
  over 100 ms; the nightly at 59e19a5, ntFIN4, flew 1 of 183,909 over
  100 ms, on the hot-apply path that the 23 Nov fix moved after the
  setpoint (section 5.4); the nightly at release 0.7.0 (21c2771), ntFIN5,
  flew that fix: 0 of 183,961 over 100 ms.

### 5.7 Scope of the figures

1. **The pilot.** Every KPI-grade run flies a stub (GotoPilot,
   ConstantPilot, StubVLA). The figures measure the Shield and its FSM in
   front of ArduPilot SITL under scripted commands, not a learned model.
   The learned VLA is shown in separate `hil` flights (section 6).
2. **Independence.** Apart from the 60 GPS-noise / latency missions, whose
   noise is drawn from the episode's seed (metrics.json
   `perception_stress`), no flown cell consumes its seed: a cell's three
   seeds are three flights of one deterministic scenario that differ only
   by the rig's timing. 445 distinct KPI signatures over the 654 ntFIN5
   episodes (demo/out/hil_ntFIN5/sweep.json `_distinct_kpi_signatures`; 446
   in ntFIN4, 376 over 594 in ntFIN3). The rates and the Wilson bound count
   them as independent.
3. **Coverage.** 90 of 744 profile episodes were refused before take-off
   (section 2): paraphrase and prefix arms the stub cannot read, gusts and
   a low battery the `hil` rail cannot realise, and 12 others. The RTL that
   fails, G6's scenario, flies (failsafe-rtl-blocked); its battery reason
   was refused in ntFIN5 and flew once on 25 Nov outside the KPI set
   (bat1125_001).
4. **Definitions.** Repair magnitude leaves the yaw rate out; mission
   success is stricter than the grant's; time to safe averages only the
   recovered spells; theta judges about a third of the converged repair
   ticks (9,307 of 27,314 in ntFIN5).
5. **Provenance.** The KPI report of record is committed in the
   repository, made from a clean tree:
   docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md
   (f9cab68). The renders of 24 Nov (b171c57) and 27 Nov (d75ddc9) are
   kept beside it; they differ from it only in the header and the
   stack-revisions note, the WP1 and WP2 sections, the stored tick profiles
   listed and the Shield test-matrix section the 28 Nov render adds; the
   episodes, the KPI summary, the per-family tables and the baseline
   comparison are the same values
   (docs/data/kpi_ntFIN5/README.md). Its "against the baseline" section
   lists every run as new or gone because it matches episodes by run name,
   and ntFIN4_NNN never matches ntFIN5_NNN; its per-family figure table is
   the comparison.
6. **Replay at the flown revision.** `tools/replay_at_revision.py`
   replayed the 654 runs open loop at 21c2771 on the AGX Orin, in the
   `hil` companion container with the interpreter the Shield node flies
   with (Python 3.12.3, aarch64): 654 identical over 303,048 ticks and
   191,106 audit records (docs/data/replay_agx_ntFIN5/README.md). With
   every other KPI-grade `hil` run flown from 22 Nov on, smREL073 among
   them, 1,660 episodes in all replay with 0 differences on the Orin
   (docs/data/replay_agx_rest_20261128/README.md,
   docs/data/replay_agx_smREL073/README.md). On the desktop (x86-64,
   Python 3.10; docs/data/kpi_ntFIN5/replay_at_revision_ntFIN5.json) the
   same replay gives 352 identical; 301 differ only in the last float
   digits (at most 3.7e-14; 109 plain, 192 through the body-frame
   rotation); 0 are not replayable; 1 differs in a repair's pass count:
   ntFIN5_306 dyn-nfz-rotate-spin, a GeofenceSlide convergence test that
   sits on the last float digit, with the same emitted action to 6.9e-14,
   as ntFIN4_304 and ntFIN3_306 before it. The Orin (aarch64) and the
   desktop (x86-64) round that digit differently. The replay is open loop
   on the logged inputs; a `hil` episode is not re-flown closed loop.
7. **Earlier campaigns.** ntFIN4 (59e19a5) and ntFIN3 (71aabb3) are the
   earlier figures of record, kept beside ntFIN5 in sections 2 and 5;
   ntFIN2 and ntFIN2s (116214f) before them. ntFIN at 116214f, stopped at
   176 of 591 missions, is in no figure; its runs are kept
   (demo/out/hil_smoke/ntFIN_*).
8. **Signing.** The KPI campaigns of record (ntFIN5, ntFIN4, ntFIN3, the
   release smokes smREL070 and smREL071) flew before 27 Nov, with bundles
   signed by the bare lab-dev keys and the chain requirement off
   (policy_source chain none, require_chain false). The Lab CA applies
   since 27 Nov (docs/data/bundle_ca_20261127/). Under it have flown the
   `hil` mission ca1127_001 and the release smokes smREL072 (0.7.2) and
   smREL073 (0.7.3), 52 KPI-grade runs each, every one with the chain
   verified and require_chain true (docs/data/kpi_smREL072/,
   docs/data/kpi_smREL073/; WP1-04). The re-sign kept policy_hash, so the
   policy each run flew is the same.

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

AIoT-VLA v10b on the AGX Orin in the hil topology, 14 Nov (docs/data/pas_aiot_hil_20261114/flights.md): six missions, Shield on, 0 P0 escape ticks, 22-297 interventions per mission, VLA decisions 6.9-7.9 Hz, camera to first setpoint p50 about 160 ms; one KPI-grade mission (pas_hil_aiot_kpi_01). 15 Nov, with the feed between ticks and the camera at a 0.05 s capture interval (docs/data/pas_hil_feed_20261115/flights.md; four KPI-grade flights, pas_hil_feed_05-08): 12.4-13.2 camera frames/s reach the model while it is asked, 10.6 and 10.9 decisions/s over the two missions that held the car for the full 420 s, camera to first setpoint p50 130-140 ms, 0 P0 escapes. The node's garbage collector is paced out of the tick since (guardrail/gc_pacing.py): its one tick of 227-454 ms every 65-80 s is gone (ticks over 100 ms: 3 of 4058 and 0 of 4096, max 152.5 ms). Follow quality is weak on this rail (target lock 0.04-0.82, docs/data/pas_aiot_hil_20261114/perception_stress_report.md), well below its 0.955-0.969 on simple_flight.

### 6.4 Default backend: AIoT-VLA v10b for the follow task

Named on 18 Nov, after the measurement set (ARCH-30, X-04, both DONE;
`tools/vla_backend_table.py table --default`, which refuses the line until
the set is complete; decision 12).

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
  policy. ARCH-06 is DONE under reading (l): both meet "switchable
  backend" through their adapters and 4-D conformance checks, with no
  drone weights published.

## 7. Deviations register

Where the delivered work differs from the grant's words, each row states
what is delivered and the reading (section 9.2) or decision (section 9)
that covers it. Every status is the one Appendix A gives.

| Item | Grant | What we do | State |
|---|---|---|---|
| WP1-25, WP2-19, X-06 | Python 3.11+ | the AGX Orin companion runs Python 3.12.3 on every `hil` flight; since 14 Nov the desktop's non-VLA side runs Python 3.12.16 (guardrail-py312): the full CI at 8a473fa, 148/148 files (docs/data/ci_local/8a473fa0172e-py312.json), the `hil` campaign hs10 planned, driven and scored by it (sweep.json `_runner`), and the PAS launcher's scene steps for the AIoT-VLA `hil` missions of 14 Nov (scripts/run_pas_ardupilot.ps1; CHANGELOG 2026-11-14); vla-real (Python 3.10.20) stays for the VLA runtime (torch, transformers), which the grant scopes out of the 3.11+ decision (docs/DESIGN-python-versions.md) | DONE; read as: the 3.11+ floor binds the non-VLA stack, and the VLA runtime stays on 3.10 (decision 8) |
| ARCH-15 | the HIL bridge carries "AirSim sensor messages (`HIL_GPS`, `HIL_SENSOR`)" into AP SITL (G1 condition 1) | Project AirSim's JSON sensor frames to ArduPilot's SIM_AirSim backend; G1's four conditions passed on it on 8 and 13 Oct (section 7.1) | DONE under reading (n), with the deviation recorded: the transport, not the bridge's function; ArduCopter 4.5.7 has no HIL_SENSOR handler and takes HIL_GPS only as an external MAVLink GPS (section 7.1) |
| WP3-04, ARCH-04 | body-frame action; one body-to-NED boundary in the adapter; repair operators act in the frame the VLA emits in | the Shield takes and returns the body action (`Shield.filter_body`, since 4 Nov) and, since 26 Nov (35df373, release 0.7.1 a3d616e), carries it through its whole stack, reading North/East/up only through the tick's repair.BodyFrame for checks, forecasts, operator geometry and records; on the grant's ROS 2 rail body to local NED happens only in sitl/mavlink_adapter_node.py (frames.body_to_local_ned); the simulator demo loops, which send Project AirSim velocity commands and have no MAVLink link, call the same function in their own loop after the Shield; flown at a3d616e in smREL071 (52 KPI-grade `hil` runs, P0 escape 0) | ARCH-04 DONE (20 Nov), WP3-04 DONE (26 Nov); the demo loops' own call is covered by ARCH-04 and decision 15 |
| WP3-06 | repair operator order with PathRepair; "first operator that produces a feasible action with magnitude <= threshold wins" | PathRepair in the default chain since 17 Nov, in the body frame since 783e626; in ntFIN5 (release 0.7.0, the KPI campaign of record) it acted on 113, 114 and 114 ticks in the three standoff-approach-angles pedestrian cells, P0 escape 0, fail-safe 3/3 in each (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md); PathRepair off on the follow and perception rails (guardrail/replay.py FOLLOW_RAIL_PATH_REPAIR); the grant's first-feasible rule is available since 27 Nov (`--repair-mode first_feasible` on three harnesses), measured once headless (docs/data/repair_mode_ab_20261127/, not KPI-grade): P0 escapes 0 on both, fail-safe 177/179 against the chain's 179/179 | DONE under reading (f): the iterated chain is the default selection (guardrail/repair.py DEFAULT_REPAIR_MODE "chain"), the first-feasible rule available and off by default, PathRepair's boundary and zone detours opt-in |
| WP3-07 (Loiter) | FSM drives SET_MODE LOITER | by default the FSM's Loiter is held in GUIDED on the plan's point by the companion since 7 Nov (guardrail/hold_flight.py); before that LOITER was sent as BRAKE, whose hand-over carried the aircraft off its point (CHANGELOG 2026-11-07 (Loiter held in GUIDED)); since 22 Nov a literal SET_MODE LOITER is opt-in (`--rail-hold loiter`), flown on `hil` in 2 shakedown missions with a SITL-only throttle stand-in (sk1122b: autopilot heartbeat LOITER, held 29.9 s; docs/data/shakedown_20261122/README.md) | DONE under reading (e): the FSM's Loiter is held in GUIDED by default, literal SET_MODE LOITER opt-in (decision 7) |
| WP3-07 (X8) | not in the grant's edges | extension edge X8 (Normal / Brake / Loiter → RTL when the position can no longer certify a hard P0 zone's margin); since 12 Nov the ROS 2 node takes it from ArduPilot's EKF_STATUS_REPORT (`estimator_ok`); a rail that reports nothing flies as before (docs/DESIGN-escalation-fsm.md, updates 2026-11-06 and 2026-11-12) | extension, kept (decision 6) |
| WP3-07 (X9) | not in the grant's edges | extension edge X9 (Loiter → RTL when the hold is not kept: the aircraft beyond the hold's reach and still receding); answers sm1111_048's missed fail-safe (CHANGELOG 2026-11-11 (X9)) | extension, kept (decision 6) |
| WP3-15 labels | "triggered when expected, not when not expected" | the rubric reading (r2): rubric labels on the rail vehicle, E5 on the prescribed flight (docs/RUBRIC-failsafe.md sections 7-8, adopted 10 Nov) | WP3-15 DONE under (r2); the rubric's readings (r) 0.982 and (a) 0.884 on the same 649 ntFIN5 runs are reported beside it (section 5.3) |
| WP3-22, X-13 | exactly one rule-evaluation code path | one rule clock and one set of point verdicts in guardrail/shield.py since 3 Nov; since 18 Nov FenceGuard measures with the Shield's field and judges with its clearance_breached ("grid" refused), flown 19-20 Nov; the remaining measures are diagnostics, verification oracles, a load and authoring lint (start_conflicts, which since 27 Nov asks the Shield library's zone_footprint_reached) and the autopilot's GeoFence backstop | WP3-22 DONE (23 Nov); X-13 DONE under reading (a): the GeoFence backstop, which the grant keeps as a "last-line backstop" (safety-shield.md, "AP GeoFence coordination"), and the listed non-verdict checks are outside the system's rule evaluation; the backstop triggered 18 of ntFIN5's 649 scored fail-safes (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md:18) |
| WP4-13 | exporters: rosbag2 to JSONL / Parquet | the exporter reads a run's rosbag2 when it exists (`rosbags` 0.10.11 and pyarrow in vla-real since 6 Nov; on the KPI-grade `hil` run hil_smoke_03, 303 steps, action.executed from the bag on 303/303; CHANGELOG 2026-11-06 (contract)); rails without ROS 2 fall back to the flight log, labelled per step | DONE; the fallback is labelled on every step |
| WP1-07 | distance_envelope to people, buildings, roads | enforced for buildings and roads (flown on hil, de_hil2, 0 breaches) and, since 19 Nov, for every person the rail reports | DONE under reading (b), a stated limit: people are kept at the envelope as far as the person detector sees them; on pas_hil_people_02 the person feed matched 0.189 of pedestrians within 25 m and the aircraft was within 4 m of one on 75 of 178 ticks (docs/data/people_hil_20261119/README.md) |
| X-05, ARCH-25 | observations reach the VLA at ≥ 10 Hz | met since 15 Nov on the hil rig for AIoT-VLA (12.4-13.2 frames/s while it is asked; the feed between ticks, a 15 Hz camera); detector 2-9 Hz by host; a 7B VLA's decision 0.5-1.2 s on the desktop GPU | DONE for AIoT-VLA, the default backend (section 6.4; decision 10); a 7B backend would stay below 10 Hz, the case of the grant's G3 candidate gate (fallback-gates.md) |
| ARCH-08 | AP_DDS migration (Q3, optional) | AP_DDS stage 1 flown on ArduCopter 4.5.7 DDS SITL (26 Nov, dev topology, not KPI-grade; docs/data/ap_dds_20261126/): the Shield's setpoints reach the autopilot only over /ap/cmd_vel (113/113 ticks, one clean route flight, no zone engaged); state, modes, arming, take-off, params and fence stay on MAVROS 2, which remains the KPI bridge (smREL073: setpoint path through mavlink_adapter in 52/52, docs/data/kpi_smREL073/README.md) | PARTIAL under reading (m): the optional migration closes at stage 1; stage 2 is not possible on 4.5.7 (no mode/armed/landed state; no take-off, parameter or fence service; docs/RUNBOOK-ap-dds.md) |

Closed deviations (no action left): WP1-01 (IR 4.0.0, 6 Nov), WP3-11 (audit
record version 4, 2 Nov), WP2-16 (the bundle ships the multi-scale
geometry, 2 Nov), WP1-22 and X-09 (IR bump on every shape change, 2 Nov),
WP1-14 (26 Nov: every Policy DSL entry refuses a point given in local
metres or with x/y beside lat/lon; a declared legacy read takes only a
policy the lock records; the stress harness converts its own ScenarioEvent
metres to WGS84 before any DSL entry; the residuals are disclosed in
Appendix A, WP1-14).

### 7.1 Gate G1 and the HIL bridge (ARCH-15)

The grant's gate G1 (04-risks-and-fallbacks/fallback-gates.md) is a
closed-loop integration test of the Project AirSim ↔ ArduPilot HIL bridge
with four conditions. Condition 1 names "AirSim sensor messages
(`HIL_GPS`, `HIL_SENSOR`)" reaching AP SITL, with AP's EKF converging on
the AirSim-reported position to within 1 m.

The delivered bridge carries Project AirSim's ArduCopterApi JSON sensor
frames over UDP 9003 to ArduPilot's SIM_AirSim backend and returns PWM over
UDP 9002 (demo/pas_ardupilot/, scripts/run_pas_ardupilot.ps1). The G1 check,
demo/pas_ardupilot/g1_check.py, passed all four conditions on 2026-10-08
and again on 2026-10-13, three runs each at seed 20261006
(demo/out/g1_20261008_223259/ and demo/out/g1_20261013_203404/,
g1_verdict.json): EKF p95 3-D error 0.814-0.827 m against 1 m, landing
0.055-0.18 m from take-off against 2 m, 0 airborne collisions, 0 fence
breaches. The perception rail is kept on Project AirSim and carries the
learned-VLA `hil` follows (section 6).

The named message set is not available on the autopilot the project
flies. In the rig's ArduPilot source (ArduCopter V4.5.7, checked 26 Nov),
HIL_GPS is handled only as an external MAVLink GPS
(libraries/AP_GPS/AP_GPS_MAV.cpp:143, libraries/GCS_MAVLink/GCS_Common.cpp:4082),
and no code in libraries/ or ArduCopter/ handles HIL_SENSOR, so condition
1's words cannot be met on this firmware as written. The deviation is in
the transport, not in the bridge's function. Under reading (n) this
recorded deviation stands, and ARCH-15 is DONE (Appendix A).

## 8. Findings and corrections

- The defects the project found in its own claims, each with what was
  retracted: docs/FINDING-five-defects-found-by-audit.md,
  docs/FINDING-the-kpi-was-never-measured.md,
  docs/FINDING-the-kpi-report-that-scored-every-rule-as-p0.md, and the
  `#### Retracted` entries of CHANGELOG.md.
- The WP2 and WP4 correction notes were sent to ITRI (Appendix A, WP2-12
  and WP4-21).
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
- 20-21 Nov, the smoke smfin1120 at 71aabb3 (`hil`, smoke profile
  with `--hil-controls`, 6,059.6 s; 64 episodes planned, 49 missions flown,
  15 refused before take-off, all 49 KPI-grade; docs/data/kpi_smfin1120/),
  the regression check against the release smoke rel060 (v0.6.0 = a85dc91,
  docs/data/kpi_rel060/). P0 escape 0 (0 on 2,201 P0-violation ticks, 48
  Shield-on flights; control 0.65); `tick_ms` max 78.84 ms and `e2e_ms` max
  78.93 ms, 0 of 15,389 ticks over 100 ms; fail-safe (r2) 46/47 = 0.979,
  Wilson 95 % low 0.889 (rel060: 48/48), with 1 run censored in Loiter
  (corridor-swap-shifted, as in ntFIN2). On 47 scored runs one error is
  0.979, so a smoke cannot show 0.99 with any error; the grant's figure is
  read on the nightly (ntFIN5, section 5.3).
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
    dyn-nfz-rotate-spin@3 without a fail-safe, and neither ntFIN4 nor
    ntFIN5 has an (r2) error on any cell.
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
    ticks over 100 ms did not exclude it.
  - **The fix** (7e6aae2, review fixes a0eb767; CHANGELOG 2026-11-23
    (code); in release 0.7.0): the generation write, the CSP compile and
    the fence publication run after the setpoint, drained at the end of
    the tick; the templates are compiled at bring-up; each row carries
    per-phase tick stamps and the wait behind deferred work. Flown 23-24
    Nov in the release smoke smREL070 (`e2e_ms` max 57.49 ms) and the nightly
    ntFIN5: 0 of 183,961 ticks over 100 ms, the 249 hot-apply ticks at
    median 14.3 ms, max 62.08 ms, to their setpoint (section 5.4). At
    release 0.7.3 the smoke smREL073 read `tick_ms` max 55.73 ms, 0 of
    16,047 over 100 ms (docs/data/kpi_smREL073/README.md); WP3-13 is DONE
    under reading (g).
- 28 Nov, the at-speed near-edge cells: in the release smokes smREL070 and
  smREL072 both near-edge-at-12mps cells were vacuous, with no P0 tick
  (docs/data/kpi_smREL070/kpi_report_smREL070_2026-11-24.md and
  docs/data/kpi_smREL072/kpi_report_smREL072_2026-11-27.md, lines 211-212),
  while each smoke read P0 escape 0. The cell had no
  scenario clock, so the `hil` rail read the wall clock, and the
  school-yard zone (Mon-Fri 07:30-17:30) was out of force: smREL072 flew
  the cells on Fri 27 Nov from 18:15 TST (CHANGELOG [0.7.3]).
  - **The fix** (release 0.7.3, c44a360): the cell pins its clock
    (`mission.clock_start` Mon 2026-09-07 09:00), and the KPI report and the
    release gate require a non-vacuous cell in every Shield matrix row and
    in every planned near-edge-at-12mps cell (tools/kpi_report.py
    `shield_matrix`, docs/RUNBOOK-release.md). Re-read over smREL072, the
    gate says NOT MET. In smREL073 both cells had P0 ticks (10 and 13, P0
    escape 0) and the gate was met (docs/data/kpi_smREL073/README.md).

## 9. Decisions and readings

The project follows the grant, and best practice where the grant is silent.
The decisions below are what the project does; where the grant's wording is
open, it applies the readings of section 9.2. Appendix A states each row's
status under them.

Settled on 2 Nov: the theta basis (forecast depth in metres), the fail-safe
rubric, the IR bump rule, the two-hash contract, multi-scale geometry in the
bundle, the correction notes, the `hil` fallback date
(docs/FINAL-REPORT-outline.md section 9).

Settled on 15 Nov (decision 1 and the waiver of decision 2 as applied on 27 Nov, decision 9 as changed on 22 Nov; decisions 4 and 5 on 17 Nov, below):

| # | Decision | Items | Settled as | What was done |
|---|---|---|---|---|
| 1 | Bundle signer and CA | WP1-04, X-10 | since 27 Nov, the Lab CA signs the policy bundles (reading A2) | 7144b7f, cda127e: authority lab-ca listed, one leaf certified per bundle key, the bare lab-dev identities retired, the 17 flight bundles re-signed on both hosts with policy_hash unchanged, VLAGUARD_REQUIRE_CHAIN=1 on `hil` and `flight` (docs/data/bundle_ca_20261127/). Flown under it: ca1127_001 (KPI-grade), then the release smokes smREL072 and smREL073, chain verified in 52/52 each (docs/data/kpi_smREL072/, docs/data/kpi_smREL073/). The KPI runs of record flew before 27 Nov, under the bare keys. A later move to ITRI's CA is written down and tested (docs/RUNBOOK-bundle-ca.md, 'Later: moving to ITRI's CA') |
| 2 | `flight` stage | ARCH-14, ARCH-31 | the work stops at `hil`; the `flight` stage is covered by a written waiver (reading A3); SITL-to-firmware bit-identity without a flashed FC is a stated limit (reading A4) | no airframe flight is claimed. The `hil` rig (AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2) flies the KPI campaigns; the real FC (CUAV V5 Nano, ArduCopter 4.7.1) was bench-tested on 17 Nov, never armed: fence set and read back, six modes accepted (docs/data/fc_bench_20261117/README.md). ARCH-14 and ARCH-31 DONE |
| 3 | `dev` KPI evidence | section 5 | no waiver is used | every KPI in section 5 comes from KPI-grade `hil` runs |
| 6 | FSM extensions X8 and X9 | WP3-07 | kept: both close a missed fail-safe (estimator health; a hold that is not kept) | documented as extensions to the grant's ladder (docs/RUBRIC-failsafe.md) |
| 7 | Loiter held in GUIDED | WP3-07, WP3-09 | the default (reading (e)): ArduCopter's LOITER sinks with no pilot on the throttle (SITL flights of 7-8 Oct; docs/brief-decision7-guided-hold.md), so the hold is the companion's, at the Shield's point; literal SET_MODE LOITER is opt-in (`--rail-hold loiter`, 29a1440) | sk1122b (22 Nov, `hil`, 59e19a5, 2 missions): LOITER held 29.9 s, altitude within 0.021 m, the throttle centred by a SITL-only stand-in (section 4.3; docs/data/shakedown_20261122/README.md). WP3-07 DONE |
| 8 | Python 3.10 for the VLA runtime | WP1-25, WP2-19, X-06 | kept: the grant's 3.11+ binds the non-VLA stack, which runs 3.12 since 14 Nov | section 7 |
| 9 | Push to GitHub | WP3-19, WP4-06, WP4-14 | master is pushed to the fork (since 22 Nov); GitHub Actions runs `tests` on every push (.github/workflows/tests.yml), the headless smoke its last step; CI also runs locally per merge (tools/ci_local.py, records in docs/data/ci_local/) | green in all 3 jobs and their smoke on the release 0.7.2 push (418b151, 27 Nov: 45969020953 master, 45969019967 tag; docs/data/github_ci/README.md, which also lists the earlier runs). .github/workflows/nightly.yml schedules the nightly profile and, since 28 Nov, the broad sweep (jobs `broad`, 12 shards, and `broad-merge`); GitHub fires a schedule only from the default branch `main`, so neither has run there; the nightly is flown on the lab rig for the release the KPI report of record describes (ntFIN5, 0.7.0), and the broad sweep was run on the lab machines at 0.7.2 and 0.7.3 (reading (c)). The three items DONE (final CI: see docs/data/ci_local/) |
| 10 | Observation rate below 10 Hz | X-05, ARCH-25 | met by engineering: a faster camera alone did not reach 10 Hz, a feed between the loop's ticks did | pas_hil_obs_02 (15 Nov, KPI-grade, capture interval 0.05 s instead of 0.066 s): the camera delivered 14.7 Hz (12.8 before) and frames reached the VLA at 9.58 Hz while it was asked (9.10-9.50 before); the bound was the pilot's asking cadence, not the camera. That cadence was then changed (the feed between ticks, the same day): 12.4-13.2 frames/s while it is asked on four flights (pas_hil_feed_05-08); X-05 and ARCH-25 DONE |
| 11 | The five final-demo scenarios | WP4-19 | the five of section 9.1, all flown on `hil`, plus pdyn1126 as the non-trivial scenario (reading (k)) | section 9.1; WP4-19 DONE |
| 12 | Default VLA backend | ARCH-30, X-04 | AIoT-VLA v10b for the follow task: the only learned model that has flown KPI-grade on the AGX Orin; the stub stays the KPI campaigns' pilot | named on 18 Nov on the generated table, after the set (section 6.4); ARCH-30 and X-04 DONE |
| 13 | G4 on a flickering brake rule | WP3-07, WP3-15 | the grant's G4 governs: rubric clause E6, "N-in-T persists" (onsets made by the world at the N-in-T rate through a Loiter); N1 and N2 are unchanged elsewhere | fsm-n-in-t-flicker labelled expected (E6), the only label that changed over every profile; hs10_003 is a correct trigger on it; the clause was written after that flight (docs/RUBRIC-failsafe.md section 9) |
| 14 | A GeoFence RTL with the FSM holding | WP3-15 | a fail-safe triggered by the grant's backstop, scored against the label like any other | derived per run (experiments/sweep_scenarios.py failsafe_backstop). First applied to ntFIN2 + ntFIN2s: 584/585 = 0.998, Wilson 95 % low 0.990, 18 backstop runs, the earlier readings printed beside it (docs/data/kpi_ntFIN2_20261115/). ntFIN5 (21c2771, release 0.7.0): 649/649 = 1.000, Wilson 95 % low 0.994, 18 backstop runs, all expected and correct; 631/631 without the rule (section 5.3). WP3-15 DONE |
| 15 | Body-frame boundary | WP3-04, ARCH-04 | accepted where the grant's path runs (ROS 2 / `hil`: Shield.filter_body, NED only after the projection in the MAVLink adapter); every flying rail hands the Shield the body action | ARCH-04 DONE (20 Nov). Since 783e626 (21 Nov, flown in smFIN2 and ntFIN4) the registry repair operators act in the body frame at the operator interface; since bec7135 (23 Nov, in release 0.7.0, flown in smREL070 and ntFIN5) the last resorts ClearanceEscape and the slowed GeofenceEscape do too (repair.ClearanceEscape, repair.OpenedEscape), decisions bit-identical to the old chain for finite actions at random headings; since 35df373 (26 Nov, release 0.7.1, flown in smREL071) no rotation wraps the stack, which carries the body action. WP3-04 DONE (26 Nov). The simulator demo loops call frames.body_to_local_ned in their own loop, after the Shield, under ARCH-04 (WP3-04, Appendix A) |

Settled on 17 Nov:

| # | Decision | Items | Settled as | What was done |
|---|---|---|---|---|
| 4 | Dated deliverables | WP1-21, WP3-01, ARCH-15, ARCH-16, ARCH-17, ARCH-23 | reported on what was delivered and where it is | all six DONE; ARCH-15 under the recorded deviation of section 7.1 (reading (n)) |
| 5 | Team | ARCH-24 | stated as in the grant: 1 PI + 4 Master students, one team | ARCH-24 DONE |

Settled on 22 Nov:

| Decision | Items | Settled as | What was done |
|---|---|---|---|
| The real flight controller is not flashed | ARCH-14, ARCH-31 | the real FC (CUAV V5 Nano) stays on ArduCopter 4.7.1, not flashed to the rig's 4.5.7 | the 17 Nov bench stands (decision 2); SITL-to-firmware bit-identity is a stated limit (reading A4) |
| BitVLA's loader run | ARCH-06 | download BitVLA's LIBERO-Spatial checkpoint (hongyuw/ft-bitvla-bitsiglipL-224px-libero_spatial-bf16, MIT) and run its real loader on the AGX Orin | loaded in 5.94 s, 5,585 MiB GPU, median 1.479 s per answer (n=20); every answer a finite Action4D, taken by Shield.filter in all 15 frame/state cases, and over TCP 12 decisions in 20 s (0.60 Hz); an arm checkpoint, so a conformance check, not a flight policy (docs/data/bitvla_loader_20261122/README.md). ARCH-06 DONE under reading (l) |
| Push master to the fork | WP3-19, WP4-06, WP4-14 | push master on 22 Nov | decision 9 above |

Since 27 Nov the project applies the Lab CA (decision 1) and the readings
of section 9.2.

### 9.1 The five final-demo scenarios (decision 11)

All five flown on `hil`. With them, pdyn1126 is the gate's non-trivial
scenario (reading (k)): at release 0.7.1 (a3d616e, 26 Nov), the suite cell
dyn-nfz-spawn-ahead with the prefix on exercises all four components in one
flight (pdyn1126_001, _003): the signed bundle, the CSP compiled from
/shield/csp for the prefix consumer, the Shield's FSM, and the suite
scenario with its scorer and episode bundle. The campaign: 12/12 KPI-grade,
10 of 12 gates pass, P0 escape 0 on 67 P0-violation ticks, `tick_ms` max
52.14 ms (docs/data/kpi_prefix_dyn_20261126/); replayed on the AGX Orin,
12/12 identical (docs/data/replay_agx_rest_20261128/README.md). Its zone
event is applied by the Shield node, not over REST (REST hot-apply in
flight: row 1 below, and wgs01_001, CHANGELOG 2026-11-26), and the CSP is
read by the hand-written prefix consumer, not a learned VLA. WP4-19 DONE.

| # | Scenario | Run | Shows |
|---|---|---|---|
| 1 | dyn-nfz-spawn-ahead and dyn-nfz-rotate-step (suite) | ntFIN2 (spawn-ahead, its zone applied by the node from the scenario file), hh02_001 (rotate-step: POST /hot_apply spawn at 3.00 s, PATCH rotate at 8.00 s over mTLS REST) | a zone hot-applied in flight, over REST in hh02_001; the Shield repairs or holds; the GeoFence backstop |
| 2 | failsafe-rtl-blocked (suite) | hs10_004, demoA_001 | the ladder to its end: G11, G6 "RTL failed: blocked path", Land |
| 3 | fsm-n-in-t-flicker (suite) | hs10_003 | N-in-T: G2 then G4, home (decision 13) |
| 4 | the follow with a learned VLA and REST zones | pas_hil_aiot_est_01, pas_hil_hotapply_02 | AIoT-VLA and the Shield on the AGX Orin; 10 zones in flight, 0 P0 escape ticks |
| 5 | perception stress | pas_hil_aiot_texture_01, pas_hil_aiot_shadow_01 | the detector losing the car under a novel road texture; the Shield's record unchanged |

### 9.2 Readings applied

Where the grant's wording is open, the project applies these readings.

| # | Items | Reading applied |
|---|---|---|
| A1 | X-01 | The Final Report v1.0 is signed at the Approval block, built from the final master commit. |
| A2 | WP1-04, X-10 | The Lab CA is the bundle signing authority (applied 27 Nov; docs/data/bundle_ca_20261127/, docs/RUNBOOK-bundle-ca.md). |
| A3 | ARCH-14 | The `flight` stage is covered by a written waiver: shown by the `hil` rig (AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2) and the bench test of the real FC (fence and six modes, never armed); no airframe flight is claimed. |
| A4 | ARCH-31 | "Bit-identical between SITL and firmware" without a flashed FC is a stated limit (FC CUAV V5 Nano on ArduCopter 4.7.1; rig SITL 4.5.7). |
| (a) | X-13 (WP3-10, WP3-22) | The autopilot's GeoFence backstop and the listed non-verdict checks are outside "the system's rule evaluation" (Appendix A, X-13). |
| (b) | WP1-07 | People are kept at the distance envelope as far as the person detector sees them (a stated limit). |
| (c) | WP3-19, WP4-14, WP4-06 | "Every release runs this matrix" means every release from 0.7.0 on (KPI-grade smoke on `hil` per release, headless smoke per push); the nightly KPI campaign is flown per release candidate (one rig, ~20 h per nightly), for the release the KPI report of record describes (docs/RUNBOOK-release.md section 7); the broad sweep runs on the headless backend. |
| (d) | WP1-05 | The two-hash contract stands in place of a byte-identical match with the reference implementation (docs/DESIGN-policy-dsl-grant-form.md, "The two-hash contract"). |
| (e) | WP3-07 | The FSM's Loiter is held in GUIDED by default; literal SET_MODE LOITER is opt-in (decision 7). |
| (f) | WP3-06 | The iterated repair chain is the default selection; the grant's first-feasible rule is available, off by default (docs/data/repair_mode_ab_20261127/); PathRepair's boundary and zone detours are opt-in. |
| (g) | WP3-13 | The tick budget is shown by the stub-pilot `hil` campaigns plus the offline 50-rule replay on the AGX Orin, without a flight that serves a learned VLA beside a ~50-rule bundle. |
| (h) | WP4-01, ARCH-19 | The KPI campaigns flown with the stub pilot on the Orin (Shield, harness, signed bundle) are the canonical `hil` topology; the learned VLA is shown in separate `hil` flights; the policy-artefact KPIs (DSL acceptance, load round trip, CSP P0 coverage, token budget) are measured by the offline tools at the delivered revision. |
| (i) | WP4-11 | Open-loop replay of the logged inputs at the flown revision meets "replayable bit-for-bit" (the audit's `ts` and the FSM's fsm_* fields left out); ntFIN5 replayed at its revision stands for the Q3 matrix. |
| (j) | WP4-17 | ntFIN5's KPI report is the "KPI report on dynamic-scenario stress", with the prefix A/B from pab3 (docs/data/kpi_prefix_ab_20261119/). |
| (k) | WP4-19 | The five final-demo scenarios of section 9.1, plus pdyn1126 as the "non-trivial scenario". |
| (l) | ARCH-06 | CognitiveDrone and BitVLA meet "switchable backend" through their adapters and 4-D conformance checks, with no drone weights published. |
| (m) | ARCH-08 | The optional AP_DDS migration closes at stage 1 (the Shield's output over /ap/cmd_vel, flown on ArduCopter 4.5.7 DDS SITL; docs/data/ap_dds_20261126/), reported PARTIAL. |
| (n) | ARCH-15 | The recorded deviation of section 7.1 stands: the HIL bridge carries SIM_AirSim JSON frames, not HIL_GPS/HIL_SENSOR; gate G1 passed on 8 and 13 Oct. |

## 10. Limitations and open items

Two Appendix A rows stay PARTIAL, ARCH-08 and ARCH-22: these are the open
items. Every other item below is delivered scope, stated with its data and,
where the grant is open, with the reading the project applies (section 9).

**Open items**

- **AP_DDS stops at stage 1** (ARCH-08, PARTIAL; reading (m)). The Shield's
  output went over /ap/cmd_vel through sitl/ap_dds_adapter_node.py on
  ArduCopter 4.5.7 DDS SITL on 26 Nov: 113/113 ticks, P0 escape 0, in the
  dev topology, not KPI-grade, one clean route flight with no zone engaged
  (docs/data/ap_dds_20261126/). Stage 2 is not delivered: every Shield
  input, mode change, arming, take-off and the GeoFence stay on MAVROS 2,
  and 4.5.7's DDS client publishes no mode/armed/landed state and has no
  take-off, parameter or fence service (docs/RUNBOOK-ap-dds.md coverage
  table). Since release 0.7.3, report.md names the setpoint path from the
  subscribers of /shield/setpoint; smREL073 flew through mavlink_adapter in
  52/52 (docs/data/kpi_smREL073/README.md).
- **No image topic in any bag; the start frame first in 49 of 52 bags**
  (ARCH-22, PARTIAL). The KPI rail carries no camera (its pilot publishes an
  action topic). The camera rail (PAS perception, the learned-VLA `hil`
  flights) records no rosbag2: none of its 44 `hil` run folders holds a
  bag. Release 0.7.3 publishes the manifest start frame at arming, and
  episode.bag2 begins with it in 49 of the 52 smREL073 bundles; the other 3
  keep the recorder's bag, because one /vla/action_4d message lay between
  the arm acknowledgement and the frame (the reason is in their kpi.json).
  None of the 654 ntFIN5 bags begins with it (release 0.7.0, before the
  frame at arming). The 52 smREL073 episode bundles alone replay identical
  on the AGX Orin (docs/data/replay_agx_smREL073/).

**Delivered scope**

- **The KPI campaigns fly stub pilots** (WP4-01, ARCH-19; reading (h)). The
  learned VLA flies in separate `hil` flights: AIoT-VLA v10b on the AGX
  Orin on 14 Nov, one mission KPI-grade (pas_hil_aiot_kpi_01, 0 P0 escape
  ticks; docs/data/pas_aiot_hil_20261114/flights.md). Its follow is weak:
  target lock 0.06 on that mission and 0.04-0.82 across the stressors
  (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). The city
  rail's follow control law (`follow_vlm.fly()`) is not reused on the PAS
  perception node (docs/DESIGN-projectairsim-ardupilot.md, "Not reused
  yet"), and the target estimator there is opt-in (`--target-estimator`).
  The Shield's figures on these flights do not depend on the follow.
- **Tick budget** (WP3-13; reading (g)). Shown by the stub-pilot `hil`
  campaigns and the offline 50-rule profile on the AGX Orin. ntFIN5: 0 of
  183,961 ticks over 100 ms (tick_ms max 62.08 ms, e2e_ms max 62.18 ms);
  smREL073: 0 of 16,047 (tick_ms max 55.73 ms); the 50-rule profile at c44a360: `_check` p99 1.048 ms, max
  1.097 ms against 5 ms, filter p99 27.98 ms, max 50.03 ms against 100 ms,
  1,301 ticks (deploy/evidence/hil/shield_tick_profile_20261128.json). The
  pilot-to-node DDS hop and the MAVROS-to-autopilot MAVLink hop are not
  timed; the bags corroborate action to MAVROS setpoint (smREL073 max
  57.69 ms), but receive-time pairing cannot see a hop longer than the gap
  to the next input. No flight served a learned VLA beside a ~50-rule
  bundle. On the PAS perception rail, outside this basis, learned-VLA
  flights had 4 of 27,582 ticks over 100 ms (max 142.85 ms; the Shield step
  71.5-85.3 ms on clearance repairs, with OpenVLA-7B and OWL-ViT sharing the
  Orin; docs/data/kpi_prefix_vla_20261120/README.md).
- **Fail-safe correctness under reading (r2)** (WP3-15). ntFIN5: 649/649 =
  1.000, Wilson 95 % lower bound 0.994, with the GeoFence backstop rule
  (decision 14); 631/631 without that rule. On the same runs (r) gives
  0.982 and (a) 0.884. ntFIN4 cleared the bound too (0.994); ntFIN3 did not
  (0.988). smREL073: 50/50, Wilson low 0.929 (1 censored).
- **Seeds are not independent** (section 5.7); the Wilson bounds count
  seed repeats as independent.
- **ArduPilot's own modes are outside the escape count** (section 5.2). The
  grant counts escapes from the Shield's log. On the true position, after
  the FSM handed the aircraft to RTL, Land or the hold, ArduPilot entered a
  P0 polygon in 23 ntFIN5 Shield-on missions (18 GeoFence-breach RTLs on
  dyn-nfz-spawn-ahead; 5 wind-beyond-authority missions at 12 m/s, lanes 15
  and 20) and breached a keep-out margin in 10 more.
- **Other judges in flight** (X-13; reading (a)). The autopilot's GeoFence
  backstop and the listed non-verdict checks are outside the system's rule
  evaluation. The backstop triggered 18 of ntFIN5's 649 scored fail-safes;
  models.start_conflicts, a point-in-zone lint, asks the Shield library's
  zone_footprint_reached since 27 Nov, not the flown verdict zone_breached.
- **People are kept at the envelope as far as the person detector sees
  them** (WP1-07; reading (b)). On pas_hil_people_02 the person feed
  matched 0.189 of pedestrians within 25 m, and the aircraft was within 4 m
  of a pedestrian on 75 of 178 ticks with truth, 59 of them on the
  autopilot's RTL leg, where guardrail/rtl_path.py checks only P0 zones and
  bands (docs/data/people_hil_20261119/README.md). A person first reported
  inside 4 m is repaired from and escalated, not held out.
- **Loiter is held in GUIDED** (WP3-07; reading (e)). By default the FSM's
  Loiter is held in GUIDED from the companion (smREL073: 27 missions entered
  Loiter, none requested LOITER). Literal SET_MODE LOITER is opt-in: it flew
  on `hil` in sk1122b (2 missions), holds where the aircraft stopped, not at
  the Shield's moved hold point, and on SITL relies on a throttle stand-in
  (RC3 1500). Of the per-profile FSM YAMLs only the default-equal `grant`
  profile has flown (fsmp1125, 25 Nov). G6's battery reason flew once
  (bat1125, 25 Nov: G11, G6, G10, Land; docs/data/hil_1125_x11_wp307/).
- **Replay is open loop** (WP4-11; reading (i)). Every KPI-grade `hil`
  run flown from 22 Nov on replays identical at its own revision on the AGX
  Orin, 1,660 episodes (docs/data/replay_agx_rest_20261128/README.md, with
  docs/data/replay_agx_ntFIN5/ and docs/data/replay_agx_smREL073/); a
  `hil` episode is not re-flown closed loop, since SITL and the network are
  not bit-reproducible.
- **MSL is not in a KPI set** (X-11). The `hil` zone-ridge cells flew with
  the terrain feed on 25 Nov (x11r1125, 2/2, P0 escape 0); the DEM is
  synthetic and SITL's own ground is flat; circle_fence, dynamic_nfz and
  corridor_swap MSL are tested, not flown.
- **No airframe flight** (ARCH-14, ARCH-31; readings A3 and A4). The
  `flight` stage is covered by a written waiver: it is shown by the `hil`
  rig (AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2) and the bench test
  of the real FC (CUAV V5 Nano on ArduCopter 4.7.1: fence and six modes,
  never armed; docs/data/fc_bench_20261117/README.md). The Orin companion
  has not driven the real FC, and bit-identity between SITL (4.5.7) and the
  FC's firmware (4.7.1, not flashed) is a stated limit.
- **Gazebo is functional only, at speed 1.0** (WP4-04). The
  gazebo_functional profile at release 0.7.2 (418b151) flew 131 of 132
  missions (one cell seed refused before arming by the rail's home check on
  all 4 attempts); every manifest records code_revision 418b1517da9f; P0
  escape 0 on 128 Shield-on missions; desktop tick_ms p99 6.48 / max
  25.06 ms; functional_only, never KPI-grade
  (docs/data/gazebo_sweep_20261128/). The broad sweep runs on the headless
  backend (WP4-06; reading (c)): at 0.7.2, 14,950 episodes, P0
  escape 0.0 (docs/data/broad_REL072/), and again at 0.7.3 (docs/data/broad_REL073/README.md: the same 14,950 episodes in 43.2 min, P0 escape 0.0, fail-safe labels 14,945/14,945, every episode's status, outcome and KPI record equal to 0.7.2's).
- **Release and nightly cadence** (WP3-19, WP4-06, WP4-14; reading (c)).
  Every release from 0.7.0 on flew the five-row matrix KPI-grade in its own
  `hil` smoke (smREL070, smREL071, smREL072, smREL073); since 0.7.3 the
  release gate also requires a non-vacuous cell in every row (smREL073:
  near-edge-at-12mps 2/2). The per-push smoke (.github/workflows/tests.yml)
  is headless and never KPI-grade; it was green in all 3 jobs on the 0.7.2
  push (docs/data/github_ci/). The nightly KPI campaign is flown on the one
  rig for the release the KPI report of record describes
  (docs/RUNBOOK-release.md section 7): the campaign of record, ntFIN5, flew at 0.7.0
  (77,356 s), and releases 0.7.1-0.7.3 carry their own smokes.
  .github/workflows/nightly.yml schedules the nightly profile and the broad
  sweep, but GitHub fires a schedule only from the default branch `main`, so
  neither job has run on GitHub.
- **The figures of record flew under the lab-dev keys** (WP1-04; reading
  A2). ntFIN5 and smREL070 flew before the Lab CA was applied on 27 Nov;
  smREL072 and smREL073 flew under it, chain verified in 52/52 each
  (section 5.7).
- **Perception stress is one run per stressor, non-KPI by design** (WP4-16,
  WP4-18). The novel road texture (15 Nov) broke the follow: the detector
  found the car on 0.095 of frames against 0.865 unstressed; the low sun
  (shadow) cost little (0.820)
  (docs/data/pas_hil_stress_20261115/perception_stress_report.md). On
  14 Nov, at -4 EV (night) and in fog (FOG 0.6) the detector never found
  the red car in 330 s and the mission did not start; at -2 EV (dusk) it
  found it after 170 s and lost it; mean frame luma 97.9 day, 43.0 dusk,
  13.5 night (docs/data/pas_aiot_hil_20261114/perception_stress_report.md).
- **Observation rate** (X-05): 10 Hz or faster for the lab's own VLA, with
  a 15 Hz camera and while the detector has the target; the detector itself
  runs at 2.3-2.7 Hz on the AGX Orin.
- **Python 3.10 for the VLA runtime** (WP1-25; decision 8): torch and
  transformers stay on vla-real 3.10; the non-VLA stack runs 3.12 since
  14 Nov.
- **BitVLA and CognitiveDrone** (ARCH-06; reading (l)): BitVLA loads and
  conforms on the AGX Orin from an arm checkpoint, unflown, at 0.6-0.7 Hz
  (docs/data/bitvla_loader_20261122/README.md); CognitiveDrone has no
  published weights.

## 11. Delivery record

What was delivered in the last weeks, by date. Flights are on the `hil` rig
(AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2) unless named otherwise;
CHANGELOG.md holds each change.

- **13-14 Nov.** ge1113, the grant example sweep at 2f22116 (sections 4.4
  and 5.6); the merged KPI report committed from a clean tree with its
  re-score tables (WP4-17); G2, G4, G6, X2 and X3 flown in hs10 (WP3-07);
  the full CI on Python 3.12 (WP1-25).
- **14-15 Nov.** AIoT-VLA on the AGX Orin with video and camera-to-setpoint
  latency; the PAS scene steps on Python 3.12 (WP1-25); the target
  estimator flown (pas_hil_aiot_est_01); REST hot-apply flown and the
  in-flight mission update written whole in AUTO (WP1-15, WP3-09 to DONE).
- **20-21 Nov, at 71aabb3.** The smoke smfin1120 (49 KPI-grade missions)
  and the KPI-grade nightly ntFIN3 (594 KPI-grade missions)
  (docs/data/kpi_smfin1120/, docs/data/kpi_ntFIN3/).
- **22-23 Nov, at 59e19a5.** The code of CHANGELOG 2026-11-22 (code) shaken
  down (sk1122a, 72 missions; sk1122b, the literal LOITER; the MSL ridge on
  SITL; docs/data/shakedown_20261122/); the smoke smFIN2 (52 KPI-grade
  missions, the five-row matrix) and the KPI-grade nightly ntFIN4 (653
  KPI-grade missions) (docs/data/kpi_smFIN2/, docs/data/kpi_ntFIN4/).
- **23-24 Nov, release 0.7.0 (21c2771).** The code of CHANGELOG 2026-11-23
  (code); local CI 188/188 files at a0eb767 (docs/data/ci_local/); tag
  v0.7.0 and master pushed. The release smoke smREL070 (52 KPI-grade
  missions, the five-row matrix) and the KPI-grade nightly ntFIN5 (654
  KPI-grade missions, 0 ticks over 100 ms), the figures of record of this
  report (sections 2 and 5; docs/data/kpi_smREL070/, docs/data/kpi_ntFIN5/).
- **24-25 Nov.** The Gazebo functional sweep at 21c2771 over 3 seeds (132
  missions, P0 escape 0; docs/data/gazebo_sweep_20261124/); the GitHub
  `tests` run green on a pushed build (45497539573, master 339cacf); the
  grant's mid-term demo gate flown on Gazebo at 75a6d45, 3/3
  (docs/data/gazebo_midterm_20261125/; ARCH-16); the geometry profile re-run
  on the AGX Orin at 21c2771 and the action-to-setpoint chain read from the
  ntFIN5 and smREL070 bags (WP3-13).
- **25 Nov, at 21c2771** (docs/data/hil_1125_x11_wp307/). The X-11
  zone-ridge cells with the terrain feed (x11r1125, 2/2, P0 escape 0; X-11
  to DONE); G6's battery reason (bat1125: G11, G6, G10, Land); the first
  flight of a selected FSM profile (fsmp1125, the `grant` profile); the
  open-loop replay of runs over a declared ground (a75f1f5, 7b6fa63;
  x11r1125 2/2 and msl_dem_hil2 6/6 identical); the flight bundles rebuilt
  with their changelog diffs (docs/data/x10_bundle_rebuild_20261125/); the
  bag replay over ntFIN5 (630/654 identical on ticks and KPI, the 24 others
  differ by one inferred "who flew it" tick; docs/data/replay_from_bag_ntFIN5/);
  the WP1 round trip at 21c2771 (33/33, docs/data/wp1_roundtrip_21c2771.json).
- **26 Nov (no flight).** ntFIN5's 119,087 non-setpoint ticks timed from
  their flight logs, all within 100 ms (docs/data/ntFIN5_nonsetpoint_ticks/);
  the bundle-signing records on the desktop and the AGX Orin
  (docs/data/bundle_ca_20261126/) and the CA runbook
  (docs/RUNBOOK-bundle-ca.md); BitVLA's AGX load in the backend table.
- **26 Nov, release 0.7.1 (a3d616e).** The release smoke smREL071 (52
  missions, all KPI-grade, P0 escape 0, fail-safe (r2) 50/50;
  docs/data/kpi_smREL071/); the dynamic prefix cells pdyn1126 (12 missions,
  10 of 12 gates pass; docs/data/kpi_prefix_dyn_20261126/); the harness REST
  episode wgs01_001 (lat/lon hot-apply bodies;
  docs/data/harness_wgs01_20261126/); AP_DDS stage 1 on the DDS SITL (dev;
  docs/data/ap_dds_20261126/); WP3-04 and WP1-14 to DONE. GitHub `tests`
  green on 220be42 (45838850365, docs/data/github_ci/).
- **27 Nov, the Lab CA** (reading A2; docs/data/bundle_ca_20261127/). Both
  bundle keys certified by the lab root CA (authority lab-ca, 7144b7f), the
  bare lab-dev identities retired, the 17 flight bundles re-signed on the
  desktop and rebuilt on the AGX Orin with policy_hash unchanged,
  VLAGUARD_REQUIRE_CHAIN=1 on `hil` and `flight` (cda127e), the move to
  ITRI's CA written down and tested (9589623), and the mission
  ca1127_001_nfz-head-on flown KPI-grade under it (chain verified, P0
  escape 0); WP1-04 and X-10 to DONE.
- **27 Nov, release 0.7.2 (418b151).** Cut under docs/RUNBOOK-release.md:
  lab CI at 5521c5f, 193/193 files (docs/data/ci_local/5521c5f079a0.json);
  tag v0.7.2 and master pushed, GitHub `tests` green on both (45969020953,
  45969019967; docs/data/github_ci/). Its smoke smREL072 (52 missions, all
  KPI-grade, P0 escape 0, fail-safe (r2) 50/50, 0 of 16,150 ticks over
  100 ms, chain verified under the Lab CA 52/52; docs/data/kpi_smREL072/).
  At 0.7.2 on 27-28 Nov: the Gazebo functional sweep (131 of 132 missions,
  every manifest at code_revision 418b1517da9f, P0 escape 0 on 128
  Shield-on; docs/data/gazebo_sweep_20261128/) and the full broad sweep,
  headless (2,990 cells x 5 seeds = 14,950 episodes in 43.8 min on 12
  shards, P0 escape 0.0, fail-safe labels 14,945/14,945;
  docs/data/broad_REL072/).
- **28 Nov, release 0.7.3 (c44a360, tag v0.7.3).** The scenario clock
  pinned for the at-speed near-edge and curfew drift cells, the release gate
  requiring a non-vacuous cell in every Shield-matrix row, the manifest
  start frame at arming with episode.bag2 beginning with it, and report.md's
  Path row from the subscribers of /shield/setpoint (CHANGELOG.md [0.7.3]);
  lab CI at 16bb416, which differs from c44a360 only in CHANGELOG.md,
  VERSION and its CI record: 194/194 files, smoke rc 0
  (docs/data/ci_local/16bb4168b391.json). Its smoke smREL073: 52 missions,
  all KPI-grade, single_sha 52/52, Lab CA chain 52/52, P0 escape 0 over
  2,393 P0-violation ticks, fail-safe (r2) 50/50, 0 of 16,047 ticks over
  100 ms, near-edge-at-12mps 2/2 non-vacuous, release gate met
  (docs/data/kpi_smREL073/). On the AGX Orin, every KPI-grade `hil`
  run flown from 22 Nov on replayed at its own revision: 1,660 episodes, 0
  differences, smREL073 also from its episode bundles alone
  (docs/data/replay_agx_rest_20261128/README.md, replay_agx_ntFIN5/,
  replay_agx_smREL073/). The offline tools at c44a360: WP1 round trip 33/33
  (docs/data/wp1_roundtrip_c44a360.json); CSP P0 coverage 124/124 at every
  budget from 256 to 144 tokens (docs/data/csp_kpis.json); on the AGX Orin,
  cold start p95 784 ms against 1 s, hot-apply at 50 rules p99 4.97 ms
  against 50 ms, and the 50-rule tick profile (deploy/evidence/hil/). The
  KPI report of record, ntFIN5's, rendered again with WP1/WP2 at c44a360
  and the Shield-matrix section, its KPI figures unchanged
  (docs/data/kpi_ntFIN5/README.md, "Re-rendered 28 Nov"). The broad sweep
  at 0.7.3 (docs/data/broad_REL073/README.md: the same 14,950 episodes in 43.2 min, P0 escape 0.0, fail-safe labels 14,945/14,945, every episode's status, outcome and KPI record equal to 0.7.2's); the nightly.yml jobs broad (12 shards) and
  broad-merge (tests/test_harnesstruth_ci.py). Appendix A re-assessed: 134
  DONE, 2 PARTIAL, 0 DRIFT, 0 MISSING (ARCH-08 and ARCH-22 PARTIAL;
  section 10).

## Approval

To be completed by the Principal Investigator. Nothing in this block is
filled in on the PI's behalf, and the report is not signed until it is.

| Field | Entry |
|---|---|
| Report | Final Report v1.0, docs/FINAL-REPORT.md, with Appendix A (the contract map) and Appendix B |
| Release | KPI figures of record: release 0.7.0, annotated tag v0.7.0, commit 21c2771 (ntFIN5, smREL070). Delivered release: 0.7.3, annotated tag v0.7.3, commit c44a360 (its smoke smREL073) |
| Report body | names the master commit the signed copy is built from (written here when the signed DOCX/PDF is built) |
| Name | |
| Role | |
| Date | |
| Signature | |

## Appendix A — obligations, evidence and what remains

Copied from docs/FINAL-REPORT-outline.md. Each row is held to a quoted line of the grant pages (kuanting-vla-uav-guardrail/docs/) and, where the grant is open, to the reading the project applies (section 9); a stated limit is written as scope in the last column. The map was re-assessed on 28 Nov, after release 0.7.3 (c44a360) and its smoke smREL073, the AGX Orin replays of every KPI-grade `hil` run flown from 22 Nov on (1,660 episodes identical; docs/data/replay_agx_rest_20261128/README.md, with docs/data/replay_agx_ntFIN5/ and docs/data/replay_agx_smREL073/), the offline policy-artefact KPIs at c44a360, the Gazebo sweep at 0.7.2 and the broad sweep: of the 22 rows then open, 20 moved to DONE, and ARCH-08 (AP_DDS at stage 1) and ARCH-22 (no image topic in any bag; the start frame first in 49 of 52 smREL073 bundles) stay PARTIAL (re-assessed 28 Nov: 134 DONE, 2 PARTIAL, 0 DRIFT, 0 MISSING). The KPI figure of record is the nightly ntFIN5 at release 0.7.0, 21c2771 (sections 2 and 5); a row that quotes another campaign names it.

### WP1 - Policy DSL and IR

25 items: 25 DONE.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP1-01 | YAML/JSON DSL surface with a Pydantic v2 discriminated-union schema | DONE | guardrail/models.py grant form; tests/test_policy_dsl_grant_form.py; since IR schema 3.0.0 (2 Nov) constraint_type, priority and violation_action are required and the action is one of the grant's six (CHANGELOG 2026-11-02, Policy DSL); since IR schema 4.0.0 (6 Nov) `scope` is required too, so all four ConstraintBase fields are, as the grant writes them: refused when missing or null at every DSL entry (test_every_rule_must_state_its_scope; policies/negative/missing_scope.yaml), `scope: mission` on all 154 shipped rules (32 files, each re-versioned, lock `migrated_from` with old_hash_form sha256-canonical-v3/ir-3.x), stored documents of earlier eras read in their era with their recorded hashes (test_a_document_from_before_ir_4_reads_and_hashes_as_stored; docs/IR-MIGRATIONS.md; CHANGELOG 2026-11-06 (IR 4.0.0)) | None. Scope is declared: the Shield enforces every rule over the whole flight, so a narrower scope is enforced more widely than written, never less. |
| WP1-02 | Validation behaviour and rule-consistency lint (normalise geometry, lint alt_min<alt_max, conflicting hard rules, valid_time well-formed) | DONE | CHANGELOG 2026-10-07 Changed (negative corpus 35/35 refused); guardrail/models.py lint and conflicting-hard-rule check; since 2 Nov rings are stored in canonical order (canonical_ring_order: positive winding, smallest point first; the hash no longer depends on vertex order) and the lint refuses a hard corridor no flight can be inside and two disjoint hard corridors (policies/negative/corridor_inside_nfz.yaml, disjoint_hard_corridors.yaml) | - (holes: the grant's schema has none; the GeoJSON importer refuses them) |
| WP1-03 | IR / AST: a typed, indexed in-memory form with spatial index and multi-scale geometry, consumed by Compiler and Shield | DONE | guardrail/ir.py STRtree + coarse/fine geometry; tests/test_ir.py; CHANGELOG 2026-10-06/-07/-13 (check 0.30-0.45 ms at 50 rules, desktop) | Orin-side timing is tracked under X-07. |
| WP1-04 | Signed policy bundle (hash + semver) as the deployable artefact | DONE | guardrail/bundle.py Ed25519, 64-hex SHA-256 policy_hash, semver `version`, tar.gz of ir.json + manifest.json + signature.txt (+ multiscale.json), policies/policy.lock.json; every rail flies the signed bundle by default since 2026-10-22. CA path built 16 Nov (CHANGELOG 2026-11-16: signed_by lab-ca:<CA fp>:ed25519:<kid>, `signer` record, signer.cert.pem, swap drill to a second CA with no code change, forged / wrong-CA leaves refused; tools/build_flight_bundles.py --issue-cert, tools/bundle_ca_evidence.py, VLAGUARD_REQUIRE_CHAIN). Since 27 Nov the bundles are signed under the Lab CA, the signing authority the project applies (reading A2; the grant's signed_by is a "Lab CA or ITRI CA reference" and it leaves "Lab-internal CA or ITRI CA?" open): policies/keys/trusted_signers.json lists authority `lab-ca` (CN=VLA Guardrail Lab Root CA, CA certificate SHA-256 71d1e071d3985bc6..., valid to 2036-10-30) and one codeSigning leaf per bundle key, issued from its public half: lab-ca:71d1e071d3985bc6:ed25519:6ec75b535ef5c533 (desktop) and lab-ca:71d1e071d3985bc6:ed25519:715f70c4647a9701 (AGX Orin, whose private key never left it); the bare lab-dev identities retired at 2026-11-27T00:10:37Z, still verifying what they signed before (7144b7f). The 17 flight bundles the hil profiles and the perception rail fly were re-signed on the desktop and rebuilt on the AGX Orin under its own lab-ca identity: policy_hash, policy_id, version and IR unchanged in all 17, signed_by, signer and issued_at changed (backups bundles_pre_ca_20261127 on both hosts, 56 files, SHA256SUMS). VLAGUARD_REQUIRE_CHAIN=1 in deploy/topologies/hil.env and flight.env, dev.env 0 (cda127e; tests/test_deploy_configs.py pins dev 0 / hil 1 / flight 1). tools/bundle_ca_evidence.py at 9589623 (clean) on both hosts (docs/data/bundle_ca_20261127/bundle_ca_desktop.json, bundle_ca_agx.json): 1 authority, 2 certified signers with chain verified, 0 bare signers in force, 2 retired; 17 policy bundles verified with chain verified, 31 older versions with chain none (refused by --require-chain); desktop drill 10/10 PASS. `python -m guardrail.bundle verify --require-chain` passes the 17 and fails a bare-key bundle (fase3-sim-demo-v0.1.3) with rc 1. Flown: ca1127_001_nfz-head-on (hil, 9589623 on both hosts, sim_speedup 1.0, seed 1001, KPI-grade): metrics.json policy_source kind=bundle, bundles/fase3-sim-demo-v0.1.4.tar.gz (the Orin's copy), declared = loaded = expected policy_hash sha256:b665adba... (the hash ntFIN5 flew), signed_by lab-ca:71d1e071d3985bc6:ed25519:715f70c4647a9701, signature and chain verified, require_chain true; P0 escape 0 (6 P0-violation ticks); its replay bundle signed by the same identity (docs/data/bundle_ca_20261127/README.md). Flown in a campaign: the release 0.7.2 smoke smREL072 (418b151 on both hosts, 27 Nov, hil, 52 KPI-grade runs, single_sha 52/52, P0 escape 0): every run records signed_by lab-ca:71d1e071d3985bc6:ed25519:715f70c4647a9701, chain verified, require_chain true (kpi.json; docs/data/kpi_smREL072/README.md). The move to ITRI's CA is written down (docs/RUNBOOK-bundle-ca.md, 'Later: moving to ITRI's CA') and tested (tests/test_bundle.py test_moving_to_another_ca_retires_the_old_ca_identity_and_keeps_its_bundles, 9589623). The state before: docs/data/bundle_ca_20261126/. | DONE since 27 Nov by the review (3 of 3 reviewers DONE, against the grant's Policy DSL page); the Lab CA is the signing authority under reading A2 (the grant notes the choice "Affects deployment"); a move to ITRI's CA is written and tested (docs/RUNBOOK-bundle-ca.md). The KPI runs of record (ntFIN4, ntFIN5, smREL070, smREL071) flew before 27 Nov under the bare lab-dev keys, with chain none and require_chain false (smREL071: 52 of 52 runs); the Gazebo mid-term gate (dev) flew a lab-dev-signed bundle (docs/data/gazebo_midterm_20261125/); under the CA have flown only ca1127_001 (one smoke cell, one seed) and the release smokes smREL072 and smREL073 (52 runs each, one seed per cell, not figures of record), and the PAS perception rail, the dev rails and Gazebo have not; flight.env's 1 has never flown (ARCH-14). Custody: the CA private key is an unencrypted file on the lab desktop, valid 10 years, and there is no CRL or OCSP (retiring or removing a trust-store entry is the only mechanism, and the certified leaves certify the same key pairs as the retired bare identities); issued_at is the signer's own word, so retired_on is a naming rule, not revocation, and only require_chain (on for hil and flight, off for dev) refuses a backdated bare-key bundle. The KPI gate does not check the chain: guardrail.manifest.is_kpi_grade checks no signature or chain, and a hil config or a desktop VLAGUARD_REQUIRE_CHAIN=0 overrides hil.env. No hil flight shows the loader refusing a bare-key bundle (CLI rc 1 and tests only). The lab-ca root is the desktop harness's mTLS root; the 3 Nov SROS 2 record names a lab CA fingerprint 69:73:B8:BA... (CHANGELOG 2026-11-03), neither the SHA-1 nor the SHA-256 of this certificate, so one root shared with SROS 2 (Stress Testing: "Same root CA as SROS 2") is not shown. The evidence was made on branch wp1-04-lab-ca and merged into master on 27 Nov (release 0.7.1, a3d616e, ships the bare-key trust store; release 0.7.2, 418b151, ships the Lab CA trust store); local CI at its head 3479613: 191/191 files, 4966 of 4986 tests passed (20 skipped), headless smoke rc 0 (docs/data/ci_local/3479613c52c0.json), and the fastapi files that skip there pass under Python 3.12 (test_api 19/19, test_railinputs_node 25/25, test_railinputs_hotapply 6/6, test_pas_hotapply 9/9, test_harness_service 63/63); the re-signed bundles, the run folder, its replay bundle and video are gitignored (committed: the tool records, the run's JSON files, the trust store). |
| WP1-05 | Bundle compatibility with the PI's reference implementation (a byte-identical match, claimed and retracted 7 Oct) | DONE | Read as the two-hash contract in place of a byte-identical match (docs/DESIGN-policy-dsl-grant-form.md, "The two-hash contract"; guardrail/reference_convert.py). Each manifest records this project's sha256-canonical-v3 hash over the whole policy, the reference's reference-policy-dsl-0.1 hash (null, with the reasons, when the reference DSL cannot hold the whole policy) and `reference_projection` (the rules the reference can hold, each omitted rule named). At release 0.7.3, over the 33 shipped policies (docs/data/wp1_roundtrip_c44a360.json, copied to docs/data/wp1_roundtrip.json): both hashes recorded and verified 33/33; forged reference hash refused 33/33; our loader reads the reference's 3 documents and its 1 bundle; the reference's own loader reads our reference-form bundles of those 3 documents and computes the same hash 3/3. | No shipped policy fits whole in the reference Phase-1 DSL (representable 0/33, each uses a rule type it lacks), so the reference loader cannot fly a KPI bundle. |
| WP1-06 | GeoFence classes: polygon_fence and circle_fence | DONE | circle_fence enforced (CHANGELOG 2026-10-07); sitl/fence/ export; PAS rail uploads clock-proof zones (CHANGELOG 2026-10-13, FENCE_TYPE 7); tests/test_railinputs_fence_export.py; ros2fix2_off_fence fence fired | None. |
| WP1-07 | Envelopes: altitude, kinematic, and distance (minimum distance to typed object classes) | DONE | The Shield enforces all three (guardrail/shield.py ENFORCED_TYPES). distance_envelope runs as the rule of its class (guardrail/models.py DistanceEnvelope.runtime_rule): buildings on the obstacle map, roads on the road map, people over every person the rail reports (PeopleClearance). An envelope without its source is refused at construction (guardrail/shield.py; tests/test_distance_envelope_msl.py, tests/test_people_envelope.py). Buildings and roads flown on hil in de_hil2 (bundle distance-envelope-sitl-v0.1.0, CHANGELOG 2026-11-19): both lanes pass, 0 breaches, P0 escape 0, replay_from_bag 2/2 identical (docs/data/distance_envelope_20261119/sweep_de_hil2.json, replay_from_bag_de_hil2/summary.json). People flown on hil in pas_hil_people_02 (bundle follow-car-citylife-people-v0.1.0, "keep at least 4 m horizontal from any person", P1; docs/data/people_hil_20261119/README.md). The person the rule bound on was first reported 3.36 m away, already inside 4 m, while the aircraft was closing on it: de-people was violated and repaired (ClearanceFix) from that tick, the aircraft still came to 0.66 m of that person's reported position (tick 82), and the FSM sent it home by X7 RTL at 9.3 s, P0 escape 0 (demo/out/pas_hil_people_02/flight_log.jsonl, lab checkout). Both bundles are re-signed under the Lab CA with the policy hash unchanged (docs/data/bundle_ca_20261127/README.md, bundle_ca_agx.json). Read as: people are kept at the envelope as far as the person detector sees them. | People bind only as far as the detector sees them (feed recall 0.189 within 25 m, 0.32 in view), a person first reported inside 4 m is repaired from and escalated rather than held out, and the envelope is not kept on the autopilot's own RTL leg, where guardrail/rtl_path.py checks only P0 zones and bands (59 of the 75 truth ticks within 4 m of a pedestrian were on that leg). |
| WP1-08 | 3-D corridor (keep-in tube) | DONE | corridor implemented, tests/test_corridor.py (audit 5 Oct DONE); flown on dev 3 Nov (harness_corridor_20261103c, corridor-survey bundle: the Shield stopped the aircraft inside the swapped corridor; WP1-15) and KPI-grade on hil in ntFIN5: the corridor_keep_in family (experiments/templates/corridor_keep_in.yaml, policies/corridor_survey.yaml), 42 Shield-on flights (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md:31; section 5.5) | None. |
| WP1-09 | Time windows: valid_time recurrence on rules plus the time_window_switch event | DONE | valid_time + time_window_switch: guardrail/shield.py events, guardrail/compiler.py _RuleClock, templates; rule clock on all rails (CHANGELOG 2026-10-13); tests/test_shield_events.py, tests/test_railinputs_clock.py | No flight has declared a switch in a policy file; flight evidence belongs to WP1-15/WP4-03. |
| WP1-10 | Breach actions: violation_action values monitor_only/project_fix/brake/loiter/RTL/land | DONE | guardrail/shield.py dispatch on violation_action + guardrail/fsm.py SET_MODE; tests/test_fsm.py 114/114 (run 2026-10-28); RTL flown via X7 on pas_ap_close_on (13 Oct) and pas_ap_video_on_20261014 | Per-action flight evidence is WP3-08. |
| WP1-11 | constraint_type hard/soft | DONE | guardrail/shield.py soft rules capped at brake, relaxed lowest priority first (CHANGELOG 2026-10-07); tests/test_shield.py | Soft rules as delivered: capped at brake, relaxed lowest priority first (guardrail/shield.py; CHANGELOG 2026-10-07); no shipped policy uses a soft rule in flight. |
| WP1-12 | priority P0/P1/P2 | DONE | guardrail/shield.py relaxation by priority; escalation uses priority (CHANGELOG 2026-10-07) | Priority does not arbitrate between two hard rules (documented). |
| WP1-13 | scope + layer fields and the layered merge regulation>site>mission | DONE | guardrail/models.py merge_layers/load_layered, locked_by (CHANGELOG 2026-10-07, 2026-10-19); tests/test_policy_dsl_grant_form.py | No flown run has used a layered policy. |
| WP1-14 | Units and frames: WGS84 lat/lon canonical, AGL default, ENU/NED computed lazily | DONE | since 3 Nov all policies are stored and hashed in WGS84 with a stated origin (tools/convert_policies_wgs84.py; docs/data/wgs84_conversion.json: 257 points, round trip at most 5.5e-8 m, re-versioned with lock `migrated_from`); policies/ holds 33, 27 with points (all lat/lon) and 6 point-free; local metres are a lazy per-origin view (IR 3.1.0, models.XY, projection.local_metres); AGL default (an absent or null altitude_ref is read as AGL, which the published schema states since 26 Nov), MSL opt-in with a declared ground (X-11, flown on SITL and hil 18-19 Nov). Entries (since 26 Nov; guardrail/models.py "WHICH LAYER ENFORCES IT"): every Policy DSL entry takes absolute positions as WGS84 {lat, lon} only and refuses a point given in local metres or with an x or y key beside lat/lon: the DSL loaders load_policy / policy_from_raw / load_layered, and so the GeoJSON/KML import (models.refuse_projected_policy, ProjectedCoordinatesRefused, naming models.wgs84_from_metres); the REST bodies and gRPC HotApply, top level and inside the grant's `geometry:` block (guardrail/api.py _points / _body_points, HTTP 422; HotApply runs api.apply_event; a point key other than lat/lon is refused too, as the schema's additionalProperties false has it; POST /nfz checks its top-level vertices before NfzRequest reads them and refuses a `geometry:` block; since the third 26 Nov fix PATCH /dynamic_nfz/{id} move refuses any key but op, vertices, motion and layer (api.MOVE_KEYS), where it answered 200 to a `geometry:` block beside lat/lon vertices, dropped the block and bumped the generation, nothing projected stored; since the fourth, translate, rotate and scale refuse any key but their own (api.EDIT_KEYS: dx/dy, angle_deg, factor, layer), where they answered 200 to one (a `geometry:` block, a metre `center` on rotate), nothing projected stored); the Ingest CLI (tools/geo_to_policy.py ingest, WP1-16), for a DSL rule or a POST /nfz / /dynamic_nfz body read from a file, stdin or a URL (ProjectedCoordinatesRefused with the REST wording and the base's frame origin, exit 2, no bundle written; until the second 26 Nov fix it converted {x, y} itself); the Shield's events (Shield._in_frame, _given_ring; HotApplyRefused) and a new bundle.derive (refuse_metre_rule); the published schema policies/policy_dsl.schema.json offers a point as {lat, lon} only (equal to bundle.policy_json_schema()). A tool that works in metres converts them first with models.wgs84_from_metres about the policy's frame origin. A frameless policy refuses every point (NoFrameOrigin, 422 on REST since 18 Nov); gen_random_scenario --metres removed. The harness's ScenarioEvent (scenario YAML events, the hil rail's --events, the gRPC harness's EmitEvent, a network entry of the harness) is the Stress Testing harness's own scenario type, not a Policy DSL entry: the grant (Stress Testing page) defines its payload as `dict # event-specific` and gives its worked example in metres relative to the vehicle (width_m, height_m, ahead_of_vehicle_m, mps). Our stress library may also give a spawn's vertices or a swap's centerline as absolute local metres; the harness converts them to WGS84 about the flown policy's frame origin (scenario_events.in_wgs84; harness_service.rest_body_of_event for the REST forward) before they reach a DSL entry (a Shield event or a REST / gRPC hot-apply body), which takes lat/lon only. A Shield is built on a policy stored in local metres only under a declared read that covers that content (models.shield_policy_problem, Policy.legacy_read_covers: models.read_recorded_policy, load_legacy_metre_policy, load_for_flight / load_bundle legacy_read, Shield(legacy_metres=True)). tests/test_wgs84_entry_residuals.py 28/28, test_geo_to_policy.py 27/27, test_wgs84_writes.py 32/32, test_wgs84_ingest_writers.py 12/12, test_wgs84_frameless_writers.py 12/12, test_legacy_rails_wgs84_only.py 9/9, test_api.py 19/19 (guardrail-py312). Flights fly WGS84 only: since 18-19 Nov bundle.load_for_flight (guardrail/bundle.py:2729-2929) refuses a policy stored as local-metres / wgs84+local-metres (LegacyPointFormRefused, naming the WGS84 successor), and since 26 Nov a bundle whose archived ir.json writes x/y beside lat/lon (the dump of the bundles hashed before 6 Oct; BundleCheck.projected_points), the harness's InjectPolicy archives included, used by every rail (sitl/ros2_shield_node.py:2922, ros2_mission_node, gcs_backstop, ros2_vla_bridge_node, perception_node, vla_fly, follow_vlm); since the second 26 Nov fix bundle.load_bundle(runtime=True) applies the same two refusals (its one caller in the code, write_bundle's `previous`, declares the read); since 22 Nov also the six legacy AirSim scripts (bundle.LEGACY_YAML_RAILS, refuse_or_load_yaml; tests/test_legacy_rails_wgs84_only.py, 9 tests; CHANGELOG 2026-11-22 (code)); a declared legacy read (VLAGUARD_LEGACY_READ=1, or legacy_read) by load_for_flight, load_yaml_for_flight or load_bundle(runtime=True) is accepted, since the third 26 Nov fix, for a recorded policy only: one form of its policy_hash, as the era of its document computes it, must be in policies/policy.lock.json (any version, migrated_from ones included, or a lock derivation; bundle.recorded_as, the lock read once and cached), and any other is refused (LegacyPointFormRefused, naming models.wgs84_from_metres or the WGS84 successor bundle) with the variable set too; the 13 legacy lab bundles and ten historical policies/*.yaml versions taken from git read under it; a metre YAML or bundle with new content (a vertex moved, a rule added, a version or issued_at changed) does not, and since the fourth 26 Nov fix neither does the never-written metre or x/y-beside rendering of a version the lock records in WGS84 only (it reproduces that version's legacy 16-hex forms, which the lock keeps for every entry): a document in metres or with x/y beside lat/lon matches only a lock record whose own document was in such a form, decided from the lock's fields (bundle.projected_era_keys: an entry with no hash_scheme of its own, locked up to 6 Oct under the v2 scheme, which digested the projected metres; an entry a later one names in migrated_from with migration.old_hash_form sha256-canonical-v3/local-metres, the frame migration of 3 Nov, and the migrated_from chain below it; a derived recipe on either), and an entry recorded in WGS84 only is matched by its own canonical hash only, which stores lat/lon; with the fourth fix the 13 lab bundles, the 53 rewound metre versions of the lock (bundle.policy_candidates) and every policies/*.yaml version in git read as before; the read is recorded as legacy_read/point_form/recorded_as (and projected_points). tools/bundle_census.py (docs/data/bundle_census_20261118/): 13 legacy + 4 frameless bundles each mapped to a flight-ready successor; the tasks last flown from legacy bundles re-flown from v0.1.3 WGS84 bundles on the AGX 19 Nov (docs/data/onepath_agx_20261119/: 5 onepath, agx_hier_aerovla_wgs84, nfz3b_agx_hier_aerovla_wgs84, signatures verified). Every hil KPI campaign since (rel060, ntFIN3, smfin1120, smFIN2, ntFIN4) loaded through load_for_flight. The headless smoke with the fourth fix: 64/64 episodes identical to a3d616e (sweep.json results, and 12,058 Shield audit, harness event and KPI records; wall-clock keys aside). | DONE 26 Nov after the independent review of the third fix (three reviewers, own probes at 6a80730): 2 DONE, 1 PARTIAL. Confirmed by the reviewers: under VLAGUARD_LEGACY_READ=1 the previous round's new metre YAML is refused (LegacyPointFormRefused, "not a recorded policy") by load_for_flight and load_yaml_for_flight, and so are the recorded fase3-sim-demo@0.1.2 metre YAML with a vertex moved 1 m or 50 m, a metre rule added, its version or issued_at changed, an origin added or scope stated, a mixed WGS84 + metre document, a fresh metre bundle (load_for_flight, load_bundle(runtime=True); read with runtime=False it is refused at Shield construction), a pre-6-Oct-format bundle of a new policy and PolicyRef.load_with_source (gRPC InjectPolicy); the unedited 0.1.2 YAML and the 13 legacy lab bundles read with recorded_as set and build a Shield; check_bundle recomputes a bundle's hash, so a bundle cannot claim one; update_lock refuses a metre YAML; PATCH move answers 422 to a `geometry:` block, center, origin or x/y; three of the new tests fail on 52ac08b. The dissent (PARTIAL), held not to block DONE by the rule that only a confirmed bypass letting a new projected point fly does: bundle.recorded_as matched a document's pre-6-Oct (legacy16), prior-v2 and IR 2.x hash forms against every lock entry, and the lock keeps the legacy forms for entries locked since (107 of 128), WGS84-only IR 4.0.0 versions included, so the never-written metre rendering, with origin, of a locked WGS84 version (fase3-sim-demo@0.1.4, wgs84-taipei-demo@0.1.2, follow-car-citylife-nfz3b@0.1.3, poly-test@0.1.3, corridor-survey@0.1.3; YAML, a write_bundle(allow_legacy_metres=True) bundle, or a hand-built pre-6-Oct-format bundle) read under the variable labelled recorded_as that version and a Shield built on it; its geometry, frame origin and floats had to equal the recorded policy's to the bit, so no new point flew, but the run record named a rendering that was never recorded; likewise a copy of 0.1.4 with x/y written beside lat/lon (x/y off by 5 m: refused). Without the variable every one was refused. Closed by the fourth 26 Nov fix (fix/wgs84-residuals): a document in local metres or with x/y beside lat/lon matches only a lock record whose own document was in such a form, or whose recorded hash digests such a form (bundle.projected_era_keys, from the lock's fields as the evidence cell says; wgs84-taipei-demo@0.1.0, written in WGS84 but hashed as projected metres, is the one such entry whose file was not in metres, see below), and an entry recorded in WGS84 only is matched by its own canonical hash only, so each of those renderings, as YAML, bundle and pre-6-Oct-format bundle, is refused under the variable too, the refusal naming the version it reproduces (tests/test_wgs84_entry_residuals.py test_a_projected_rendering_of_a_wgs84_only_version_is_refused, for fase3-sim-demo@0.1.4 and wgs84-taipei-demo@0.1.2 through load_for_flight, load_yaml_for_flight and load_bundle(runtime=True); it and the two new PATCH tests (the handler in that file, the route in test_api.py) and the changed test of the legacy16 x/y-beside bundle fail on 6a80730), while the recorded metre versions and the pre-6-Oct x/y-beside bundle of wgs84-taipei-demo@0.1.0 still read. Verified 26 Nov by two independent read-only verifications of the fourth fix (791e7f0, 629bead), both closing it: of the 65 lock entries outside projected_era_keys, the 54 with points were rendered in metres (with and without origin, IR 2.x and current views, scope dropped) and with x/y beside lat/lon, each as YAML, as a write_bundle bundle and as pre-6-Oct-format bundles, and loaded with the variable set through load_for_flight, load_yaml_for_flight, load_bundle(runtime=True) and PolicyRef.load_with_source: none was accepted, where 6a80730 accepted 4,590 such loads; a forged policy_hash, hashes entry or legacy form is refused by check_bundle, and a hash field written into a YAML is refused as extra_forbidden; the 56 files of the lab bundles folder and the 266 distinct policies/*.yaml blobs in git history (git log --all) read exactly as on 6a80730; 57 PATCH translate/rotate/scale/move bodies with a key the op does not define, which 6a80730 answered 200 with a new generation, answer 422. One residual of the dissent's kind remains and is disclosed here: wgs84-taipei-demo@0.1.0 was written in WGS84 (lat/lon with an origin, projected at load time) but locked on 6 Oct with no hash_scheme of its own, so its v2 policy_hash digests the projected metres and criterion (a) puts it in projected_era_keys; its never-written metre rendering, and a YAML of it with x/y beside lat/lon, read under the variable labelled recorded_as wgs84-taipei-demo@0.1.0, because the lock's record of that version is the hash of exactly those metres and cannot tell the two documents apart; the points must still equal the recorded ones to the bit, so no new point can fly; a point moved 5 m is refused (tests/test_wgs84_entry_residuals.py test_the_pre_6_oct_wgs84_version_reads_in_its_recorded_metres). The verifications also found that a PATCH whose op is not a string (a list) answered 500 instead of 422, nothing applied; since the fix of those findings it answers 422 (guardrail/api.py apply_nfz_edit; tests/test_api.py test_routes_refuse_an_edit_key_the_op_does_not_define). Also closed by the fourth fix: PATCH /dynamic_nfz/{id} translate, rotate and scale ignored keys they do not define and answered 200 with a new generation ({x, y} vertices, a `geometry:` block, a metre center on rotate, which rotates about the vertex mean; nothing projected stored, GET /policy carries no x/y); they now answer 422 naming the key (api.EDIT_KEYS), as move does; their dx/dy, angle and factor are relative edits, as the grant's translate and rotate ScenarioEvents are. Disclosed: the deprecated POST /nfz ignores body keys it does not define (e.g. `center`), where /dynamic_nfz and /hot_apply refuse them (extra_forbidden). Not guarded in process (guardrail/models.py "WHICH LAYER ENFORCES IT"): the type, Policy.model_validate, still reads the legacy metre form and a lat/lon point with x/y beside it (the x/y checked to 1 um and dropped), because every recorded artefact (a run's policy_g<N>.json, an old bundle's ir.json, a lock recipe) is read through it; Shield construction checks the metre forms only and the Prefix Compiler (compiler.ConstraintCompiler) none, so an in-process caller that builds either on a policy it validated itself is not refused. VLAGUARD_LEGACY_READ=1 still declares the legacy read for every load_for_flight and load_bundle in the process, but since the third fix each such read is checked against the lock; bundle.derive(legacy_read=True), the lock recipes' rebuild, is an in-process call and is not. Shield.move_nfz, called in process, keeps a vertex's lat and lon and drops any other key (`alt`, say), where REST answers 422. The harness takes lat/lon over an x/y written beside them in an event payload without checking the x/y (scenario_events._point, rest_body_of_event). Earlier reviews: the re-review of the first fix (b2a644d) 1 DONE, 2 PARTIAL, on the Ingest CLI converting {x, y} at ingest, the ScenarioEvent wording and load_bundle(runtime=True), closed by the second fix with four smaller findings (POST /nfz stored `x: null`, REST dropped unknown point keys, Shield.move_nfz refused {lat, lon} in a models.XY, the schema did not say a null altitude_ref means AGL); the review of the second fix 2 DONE, 1 PARTIAL, kept PARTIAL because under the variable load_for_flight read a new, never-recorded metre YAML and a Shield was built on it, closed by the third fix. The review's suggested hardenings are in the fourth fix: the hash forms of a projected document matched by the era of the lock record (decided from the record's point form, as above), and a key allow-list for PATCH translate, rotate and scale. The 26 Nov refusals are tested, not flown; the 22 Nov legacy-script refusal is tested, not flown (those AirSim scripts are not used by any KPI rail). |
| WP1-15 | Mid-flight hot-apply (dynamic_nfz, time_window_switch, corridor_swap) with generation bump and hash re-derive, via a REST endpoint | DONE | guardrail/api.py REST + --hotapply-port on both SITL rails with mTLS (CHANGELOG 2026-10-13); tests/test_api.py, tests/test_railinputs_hotapply.py; 'Nothing here has flown'; 29 Oct: REST hot-apply into flying ArduPilot SITL + MAVROS 2 (ROS 2 rail, dev): a dynamic_nfz spawned, rotated, translated and switched off (time_window_switch) mid-flight, generations 0-4, receipt to swap 0.444-4.443 ms against 50 ms; on ros2_hotapply_03 the Shield and the FSM stopped the aircraft short of the REST zone (X1, G3), released it after the switch-off (G8), and it reached the target; 3 Nov: corridor_swap over REST in flight - harness_corridor_20261103c (re-flown 4 Nov from the main checkout, code recorded: harness_corridor_20261104b) (corridor-survey bundle, the harness forwarding to the rail's /hot_apply): a dynamic_nfz spawned (generation 1), switched off (2), the corridor swapped for a shorter one ending at x = 40 (3) and back (4), receipt to swap 0.711-1.814 ms; the Shield stopped the aircraft inside the swapped corridor (x 29.2) until the swap back, then it reached the target, P0 escape 0.0 (CHANGELOG 2026-11-03 (G)); 15 Nov (code + tests, not flown): the perception rail serves the same listener on its own Shield (demo/pas_ardupilot/rail_hotapply.py: the ROS 2 node's five --hotapply-* flags, plaintext on loopback only and mutual TLS with the harness CA otherwise; each event a generation written as policy_g<N>.json / csp_g<N>.json, metrics.json hot_apply, dwell from the arrival, the GeoFence re-uploaded on the next tick); tests/test_pas_hotapply.py; the dynamic_nfz file hold (C-16) lifted for a zone without motion (Policy.static_keepout_zones) (CHANGELOG 2026-11-15); 15 Nov flown on hil (AGX Orin, MAVROS 2; pas_hil_hotapply_01, pas_hil_adapter_01, pas_hil_hotapply_02, all three KPI-grade): 14 dynamic_nfz events over REST (spawn and expire), each a generation with its policy_g<N>.json and csp_g<N>.json, the GeoFence re-uploaded after each, 0 P0 escape ticks; receipt to swap 0.70-53.0 ms, the 53.0 ms spawn having waited for a whole tick under the loop's old lock scope, fixed (00264ef: the lock from the Shield's inputs to the tick's records), after which 10 events 0.81-6.35 ms; a clear after an expire is judged by its read-back (5b3db86: MAVROS 2 answers a zero-item push with failure although the autopilot takes it) (docs/data/pas_hil_hotapply_20261115/flights.md; CHANGELOG 2026-11-15 (hot-apply on hil; WP3-09 flown)) | a MOVING dynamic_nfz stays held from policy files (no rail records the Shield's clock per trajectory point; the grant's route, injection over REST, flies); on the hil perception rail only dynamic_nfz was sent; time_window_switch and corridor_swap flew over REST on the ROS 2 rail on dev (29 Oct and 3 Nov) and, harness-driven, on hil in hh02 (14 Nov, KPI-grade; WP4-03). |
| WP1-16 | Ingest outputs: JSON Schema export, KML/GeoJSON converter, ingest CLI | DONE | policies/policy_dsl.schema.json; tools/geo_to_policy.py; tests/test_geo_to_policy.py | Polygons with holes refused (minor). |
| WP1-17 | WP1 KPI: policy load round-trip | DONE | tools/wp1_roundtrip_kpi.py -> docs/data/wp1_roundtrip_c44a360.json (= docs/data/wp1_roundtrip.json; the delivered revision c44a360, 28 Nov, desktop, bundles signed by a Lab CA certified key, signatures and chains verified 33/33): round-trip 33/33, tamper 33/33, negative 39/39, nulls 0/33 and 0/39; tests/test_wp1_roundtrip_kpi.py | None. |
| WP1-18 | WP1 KPI: bundle replayability (load the bundle whose policy_hash matches, never 'the latest') | DONE | hil_smoke_03 (2026-11-06): flown from a clean git checkout on the AGX Orin (code d0e91bb, hil, KPI-grade), its replay bundle signed by the Orin's own key, holding the policy generation whose hash the manifest names, the flight log and the rosbag; `python -m guardrail.replay verify` OK and tools/kpi_report.py re-derives its KPIs from its own log (1 KPI-grade episode); kept in the tracked store as bundles/hil_smoke_03.replay.tar.gz (CHANGELOG 2026-11-06 (hil)); headless re-fly bit-for-bit (tools/rerun_episode.py) | - |
| WP1-19 | Policy DSL acceptance KPIs: schema-validation pass rate 100%, golden cases 100%, cold-start 50-rule ≤1 s, hot-apply ≤50 ms, monitor query ≤5 ms at 10 Hz with 50 rules | DONE | At the delivered revision c44a360 on the AGX Orin (hil machine, companion image, Python 3.12.3, MAXN): cold start of a signed 50-rule bundle to a ready Shield in fresh processes p95 784 ms (imports + verify + parse + IR + Shield, worst bundle, the certified signer's chain check included; certified signer, chain checked: p95 782 ms; interpreter start-up is not part of the timed ingest, with it p95 1,081 ms) against 1 s, tampered / untrusted / bad-signature bundles refused through the same path (deploy/evidence/hil/wp1_roundtrip_agx_c44a360.json; the bench process ran with VLAGUARD_REQUIRE_CHAIN=0 so the tool's bare fixture key loads beside its certified row, deploy/evidence/README.md); hot-apply at 50 rules p99 4.97 ms against 50 ms and _check p99 0.311-0.633 ms (deploy/evidence/hil/bench_shield_50rules_5s_agx_c44a360.json); 1,301 real ticks at 50 rules x 51 poses: query p99 1.048 ms, max 1.097 ms, 0 of 1,301 over 5 ms (deploy/evidence/hil/shield_tick_profile_20261128.json); on the desktop (Python 3.10.20) schema validation 33/33 authored and 33/33 canonical IR, the grant's 3 documents 3/3, golden cases 100 % (round trip 33/33, negative corpus 39/39 refused as declared) (docs/data/wp1_roundtrip_c44a360.json; the Orin's image has no jsonschema); in flight, REST receipt to swap 0.444-4.443 ms over 18 events (desktop WSL, 29 Oct); on hil (AGX Orin, 15 Nov) 0.70-53.0 ms over 14 events, the one over 50 ms a spawn that waited for a whole tick under the PAS loop's old lock scope, and 0.81-6.35 ms over 10 events after the fix (00264ef; docs/data/pas_hil_hotapply_20261115/flights.md) | - |
| WP1-20 | Policies used in the CityLife city flights are bundled and replayable | DONE | the city rail flies signed bundles by default (CHANGELOG 2026-10-22; nfz3b bundle signed and deployed to the Jetson, 2026-10-28); replay bundles in each run folder; since 2026-11-06 the deliverable CityLife flights of the report's VLA section - AIOT-VLA v10b with the stand-off feed, aiot_v10bs_01-03, flown from the clean master 55faad0 - are in the tracked store as signed replay bundles (bundles/aiot_v10bs_0N.replay.tar.gz, lab key; each keeps its flown policy hash under its IR 2.x form and verifies, tests/test_replay.py) | - |
| WP1-21 | Mid-term gate (2026-07-20): Policy DSL + IR shipped | DONE | delivered: the Policy DSL (guardrail/models.py, guardrail/compiler.py, the shipped policies under policies/) and its typed IR (guardrail/ir.py); the mid-term report docs/MIDTERM-REPORT-Aug2026.md; the PI's reference repository also holds a Policy DSL with a typed IR at its first commit 9e039b0 (the PI's reference implementation, not the delivered code; docs/data/midterm_gate_evidence/evidence.md) | None. |
| WP1-22 | R6 freeze gate: freeze the IR shape at end of Q1, and give every IR-shape change a semver bump and migration note (the Policy DSL page cites 'R6 ... for the freeze gate at the end of Q1') | DONE | sha256-canonical-v2 + policies/policy.lock.json + docs/DESIGN-policy-identity.md (CHANGELOG 2026-10-06); generated policies get unique ids (CHANGELOG 2026-10-22, 'WP1-22'); 29 Oct: the migration-note register docs/IR-MIGRATIONS.md (7c02284), one row per IR-shape change since the freeze; since 2 Nov any IR-shape change, additive included, is refused by check_lock until IR_SCHEMA_VERSION rises: 2.2.0 versioned `locked_by`, 3.0.0 the required fields (tests/test_policy_hash.py) | - |
| WP1-23 | Single IR contract across the deployed system (Overview: Compiler and Shield 'never re-parse the YAML / JSON source') | DONE | one bundle feeds Shield and VLA nodes (sitl/run_ros2_demo.sh, --bundle); city rail too since 2026-10-22 | None. |
| WP1-24 | policy_hash on every emitted artefact (Overview: CSP, Shield repair log entry, episode bundle) | DONE | audit records carry policy_hash+generation (guardrail/audit.py AuditRecord), csp_g<N>.json on ROS 2 + PAS rails, manifest policy_hash on every run | None (city follows write csp_g0.json since f6ee433; WP2-11). |
| WP1-25 | Locked design choice: Python 3.11+ with Pydantic v2 | DONE | pyproject.toml requires-python >=3.11, guardrail tests pass on 3.11; the Orin Nano ran Python 3.10.12 (jetson_facts_20261028.json); since 6 Nov the hil machine is the AGX Orin, whose companion image runs Python 3.12.3 for rclpy and the Shield (deploy/evidence/hil/companion_versions.txt; ARCH-28), on every hil flight since; the desktop flight env vla-real is still Python 3.10.20; on 14 Nov the PAS launcher's scene steps ran on guardrail-py312 for the AIoT-VLA hil missions (scripts/run_pas_ardupilot.ps1 at f77efae). | The VLA runtime (torch, transformers) stays on vla-real 3.10, read as outside the grant's 3.11+ decision, which binds the non-VLA stack (decision 8). |

### WP2 - Prefix Compiler (CSP)

19 items: 19 DONE.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP2-01 | CSP injected into the VLA / planner prefix (the WP2 output artefact itself) | DONE | demo/real_vla_demo.py --csp on/off (CHANGELOG 2026-10-06); since 7 Nov flown: OpenVLA-7B (bf16) flew with the CSP in its prompt in 11 of 14 flights at c7a69e8 (the 3 CSP-on seeds of the A/B and the 8 paraphrase flights; tools/run_csp_flights.ps1 -Prefix cspab_20261107; docs/data/csp_flights_20261107.json; CHANGELOG 2026-11-07 (CSP flights)), AirSimNH (classic AirSim, simple_flight), not KPI-grade, 1.23-1.30 Hz, P0 escapes 0 and 0.0 s in the zone on every flight | On base OpenVLA (trained on arm manipulation, never on constraint text) the CSP's effect on repairs is no larger than a rule-free prefix of its length (replay: csp - neutral mean -0.032, csp above neutral in 5 of 14 runs): the model does not use it. Delivered: the CSP in the VLA prefix, flown. Not delivered: a VLA trained to read constraint text (future work). These flights decided at 1.23-1.30 Hz, below the 10 Hz of the grant's G3 candidate gate (04-risks-and-fallbacks/fallback-gates.md). |
| WP2-02 | Grant KPI: prefix-token budget (configurable budget, respected; CSPBudgetExceeded on P0 overflow) | DONE | guardrail/csp.py budget + CSPBudgetExceeded; docs/data/csp_kpis.json (at the delivered revision c44a360, 33 policies): the budget scanned from 256 down to 144 tokens in steps of 8, all 33 compiled within the budget at every step, largest CSP 149 tokens (126 at 144), no CSPBudgetExceeded raised; tests/test_csp.py 105/105 at 16bb416 (docs/data/ci_local/16bb4168b391.json; guardrail/csp.py and tests/test_csp.py unchanged from 16bb416 to c44a360) | Measured offline only. |
| WP2-03 | Grant KPI: CSP coverage of policy IR (spec: P0 coverage = 100% across test fixtures) | DONE | docs/data/csp_kpis.json (at the delivered revision c44a360, 33 policies): the compiler keeps 124/124 P0 at every budget from 256 to 144 tokens; at 144 tokens the naive cut keeps 122/124, null 0/124; at the default 256 tokens 124/124 for both; tests/test_csp.py | None. |
| WP2-04 | CSP schema: Pydantic v2 model with the locked field set + csp.schema.json export | DONE | guardrail/csp.py typed model, guardrail/csp.schema.json; test_the_committed_schema_has_not_drifted_from_the_model; 29 Oct: policy_id and selection made optional so the spec's own worked CSP validates against the model and csp.schema.json (tests/test_csp.py::test_the_specs_own_worked_example_validates); the flight environment's Python 3.10 stays WP2-19 | None. |
| WP2-05 | Pipeline: spatial + temporal filter on mission context, risk-grading (0.5 sev + 0.3 prox + 0.2 tcrit), order + truncate (P0 kept, P1/P2 by budget) | DONE | guardrail/csp.py time/region filter, risk 0.5/0.3/0.2, P0 kept, budget cut (CHANGELOG 2026-10-06); tests/test_csp.py; since 7 Nov the mission context carries the grant's pose, home, target and allowed modes (guardrail/compiler.py Mission.home_x/home_y/allowed_modes: the return home joins the mission region and the proximity grade, the relevance text names the home and any breach action the mission does not allow; a Mission stating neither compiles byte for byte as before; CHANGELOG 2026-11-07 (CSP flights ready)); a flown VLA has read the CSP (WP2-01) | The weights are the grant's 0.5 / 0.3 / 0.2, configurable as its Outputs ask; the PDF lists their tuning as an open question naming no objective or data, so they were not tuned (decided 7 Nov; the A/B data could serve one). |
| WP2-06 | Mission-context relevance explainable: relevance_explanations[rule_id] for every rule | DONE | guardrail/csp.py relevance_explanations; test_kpi_every_rule_in_the_csp_has_a_relevance_explanation | None. |
| WP2-07 | Per-consumer adapters: action-set adapter (body-frame vx/vy/vz/yaw_rate ranges + forbidden polygons) and cost-map adapter (YAML) | DONE | action-set adapter (guardrail/csp.py) + cost-map adapter (compiler.rasterise_cost_map, docs/DESIGN-cost-map-adapter.md, CHANGELOG 2026-10-12); 29 Oct: CostMap.save also writes the header as <stem>.yaml (the grant's 'emits YAML'), and load_cost_map reads either | No planner reads the cost map (by design). |
| WP2-08 | Templated NL adapter (Jinja2 + Python), canonical source for the paraphraser | DONE | guardrail/templates/ Jinja2 incl. event templates; test_every_rule_type_has_its_jinja2_templates | The templated sentences (the CSP's natural_language_prompt, rendered from the Jinja2 templates) reached a VLA in flight: OpenVLA-7B in 11 of 14 AirSimNH flights on 7 Nov (WP2-01, not KPI-grade) and AerialVLA in pab_aerialvla_04 on hil, 19-20 Nov (docs/data/kpi_prefix_vla_20261120/README.md). |
| WP2-09 | Effectiveness evaluation CLI: offline replay (violating-candidate rate before/after, repair magnitude, token utilisation) + A/B prefix on/off in stress runs | DONE | tools/prefix_eval.py + harness prefix on/off key (CHANGELOG 2026-10-07); tests/test_prefix_eval.py; since 7 Nov run on the logged tuples of all 14 OpenVLA flights, each replay judged with the flight's own recorded Shield and reproducing its flight on every compared frame: repair rate csp - off mean +0.097, rule-free neutral prefix - off +0.129, csp - neutral -0.032; the A/B flown on three seeds (WP4-15) (docs/data/csp_flights_20261107.json, CHANGELOG 2026-11-07 (CSP flights)) | Not KPI-grade (AirSimNH). The stress harness's own prefix arms fly a text-blind pilot (headless, and the hil stub), so their delta is a plumbing check; the delta with a text-reading VLA is the CSP runner's. |
| WP2-10 | CSP carries policy_hash + generation; stateless/deterministic | DONE | guardrail/csp.py policy_hash, generation, issued_at; test_the_csp_hash_is_stable_across_processes_and_hash_seeds | None. |
| WP2-11 | CSP emitted with each flight's artefacts (policy_hash-everywhere invariant) | DONE | demo/follow_vlm.py city_episode: every city follow writes events.jsonl, policy_g0.json and csp_g0.json (f6ee433); tests/test_city_episode.py 3/3 | - |
| WP2-12 | Claims made about WP2 in delivered reports vs what the code does | DONE | tools/check_claims.py (its WP2-12 pattern at line 135; 0 live claims in tracked files); the WP2 correction note was sent to ITRI (CHANGELOG.md, 2026-11-02 entry) | - |
| WP2-13 | Paraphraser (Q3 Aug-Oct deliverable, separate service fed by the templated NL); per-paraphrase robustness is WP4's KPI | DONE | Since 2 Nov a separate service: guardrail/paraphraser_service.py, gRPC (guardrail/protos/paraphraser.proto: Paraphrase, Validate, GetStatus) with mutual TLS from the control plane's lab CA, plaintext on loopback only; templated NL in, paraphrases out, each passed by the validator, with the validator and store digests; tests/test_paraphraser_service.py 8/8 (vla-real). guardrail/paraphraser.py with 112 stored paraphrases, the 4 sets stale since C8 re-authored 2 Nov (`stale`: 0 of 21 texts without a set). Validator: the 7 mechanisms of heldout_2026_10_28's misses fixed (2/20 -> 1/20 false accepts, 5/20 -> 0/20 false refusals; a dev set since); a new set, heldout_2026_11_02 (94 probes, written and committed before the validator was read), measured 1/50 and 18/44 before, 1/50 and 14/44 after those fixes (the held-out figure), 0/50 and 3/44 after its own mechanisms were fixed (dev); mutants 3977/3977 refused (CHANGELOG 2026-11-02) | The stress harness (the orchestrator) draws its paraphrases from the service (WP4-10); no held-out estimate exists for the validator as it stands (heldout_2026_11_02 was tuned on, and its author also fixed the validator); a person has yet to read the stored set; the KPI use is WP4-09. |
| WP2-14 | Tests for the prefix compiler acceptance KPIs | DONE | tests/test_csp.py test_kpi_budget_*, test_kpi_p0_coverage_*, test_kpi_p0_overflow_raises_*, test_kpi_every_rule_* (the file 105/105 at 16bb416, docs/data/ci_local/16bb4168b391.json) | None. |
| WP2-15 | CSP re-issued when the policy generation changes (hot-apply). Grant: CSP is a function of 'policy bundle ( policy_hash + generation )'; Overview: 'policy_hash everywhere' | DONE | sitl/ros2_shield_node.py writes csp_g<N>.json per generation; dynamic_nfz compiles (CHANGELOG 2026-10-12); stored ros2fix_on_dyn_nfz (7 Oct) lacks generation 1's CSP; not re-flown; 29 Oct: ros2_hotapply_01-03 wrote csp_g0..csp_g4, one per generation; g0 does not name the REST zone, g1-g4 do, each with its generation's policy_hash | On hil the scenario's mid-flight events re-issue it too (e.g. ntFIN2_178, 13 Nov, KPI-grade: csp_g0.json at take-off, csp_g1.json after the hot-applied dyn-nfz); those events come from the node's scenario file, not REST. |
| WP2-16 | Locked design choice: multi-scale geometry - 'The Prefix Compiler picks the scale, never re-computes' (P0Entry.geometry_ref, e.g. 'coarse:...@v0.3.0/g0') | DONE | guardrail/ir.py multiscale geometry + geometry_ref in the CSP; computed at load, the bundle does not carry the polygons; the compiler reads the scales from guardrail/ir.py and never re-computes them (since 8 Oct); since 2 Nov the bundle ships them (multiscale.json, digest in the signed manifest; check_bundle verifies fine / bbox / coarse without recomputing) and a bundle-loaded policy compiles its CSP with the coarse computation switched off (CHANGELOG 2026-11-02, Policy DSL) | - |
| WP2-17 | Locked design choice: the CSP's action sets are in body frame (vx, vy, vz, yaw_rate); the Shield converts to local-NED before MAVLink emit | DONE | body-frame actions at the ROS 2 boundary, conversion in sitl/mavlink_adapter_node.py (CHANGELOG 2026-10-07); tests/test_mavlink_adapter.py | City rail's StepTracker emits world-frame actions (not a grant rail). |
| WP2-18 | Final-delivery scope: 'Prefix Compiler (full)' scheduled Q2 May-Jul (Overview.txt:170); final gate requires 'Prefix Compiler ... complete' (Grant overview.txt:102-103) | DONE | every compiler part built and tested (WP2-02..08, 10, 14); since 7 Nov the CSP flown in a text-reading VLA's prompt (WP2-01) and its effect measured on and off (WP2-09, WP4-15) | The measured effect on base OpenVLA is not above a neutral prefix's (WP2-01); AirSimNH, not KPI-grade. |
| WP2-19 | Locked design choice: 'Python 3.11+ with Pydantic v2 for the CSP schema' | DONE | same as WP1-25: package 3.11+; the hil companion image runs 3.12.3 (ARCH-28); the desktop env vla-real is 3.10.20 14 Nov: the PAS launcher's scene steps ran on guardrail-py312 for the AIoT-VLA hil missions (scripts/run_pas_ardupilot.ps1 at f77efae). | The VLA runtime (torch, transformers) stays on vla-real 3.10, read as outside the grant's 3.11+ decision, which binds the non-VLA stack (decision 8). |

### WP3 - Safety Shield

24 items: 24 DONE.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP3-01 | Mid-term (2026-07-20): Safety Shield ROS 2 node skeleton with at least one projection operator | DONE | delivered: the ROS 2 Shield node (sitl/ros2_shield_node.py) over guardrail/shield.py with its projection operators (guardrail/repair.py: altitude, lateral, corridor, clearance, stand-off, kinematic) (audit 5 Oct DONE); flown on the ROS 2 rail and in the `hil` topology (WP3-03, WP3-14); the Gazebo rail of the same mid-term delivery is ARCH-16; the PI's reference repository also holds a ROS 2 Shield node with two repair operators (AltitudeClamp, LateralProjection) at its first commit 9e039b0 (the PI's reference implementation, not the delivered code; docs/data/midterm_gate_evidence/evidence.md) | None. |
| WP3-02 | ROS 2 node: Monitor / Filter / Projection; violation checker reuses the policy IR; bounded repair loop (default 3) | DONE | monitor/filter/projection, bounded repair, Shield.history, Shield.rule_status (CHANGELOG 2026-10-06/-07); tests/test_shield.py | None. |
| WP3-03 | VLA->Shield on a 10 Hz ROS 2 topic (Float32MultiArray); Shield->MAVROS 2 on a 10 Hz topic (mavros_msgs/PositionTarget) | DONE | sitl/mavlink_adapter_node.py publishes mavros_msgs/PositionTarget; sitl/ros2_ws packages; tests/test_mavlink_adapter.py | None. |
| WP3-04 | Action input frame is body frame (vx, vy, vz, yaw_rate); body->local-NED conversion happens after projection, in the MAVLink adapter | DONE | Body frame in: the Shield takes and returns the body action (`Shield.filter_body`, 4 Nov; guardrail/shield.py:7578). Every rail and demo loop hands it the body action at the measured heading (wf21, 19 Nov). sitl/ros2_shield_node.py:1806-1820 `shield_step` calls filter_body (:1817) and does no rotation of its own. World-frame producers hand over through frames.filter_world_as_body (guardrail/frames.py:387). A grep finds no flying rail that calls the world-frame Shield.filter / Shield.advise. Those are now world-to-body wrappers of the same stack, used by offline tools and the FSM's internal untrusted_repair judgement (shield.py:9332). tests/test_frame_boundary.py `test_no_rail_or_demo_loop_hands_its_shield_a_world_frame_action`. History: registry operators in the body frame since 783e626 (21 Nov, CHANGELOG "2026-11-22 (code)"; flown smFIN2, ntFIN4 at 59e19a5); the last resorts ClearanceEscape and the slowed GeofenceEscape as repair.ClearanceEscape / repair.OpenedEscape since bec7135 (23 Nov, release 0.7.0; flown smREL070, ntFIN5 at 21c2771). Stack in the body frame since 26 Nov (35df373 + 7f9a8c5, merged 0557503, release 0.7.1 a3d616e; CHANGELOG 2026-11-26 "WP3-04: the Shield's stack carries the body action"). filter_body runs _admit_body (shield.py:7703, Sanitise on the body action, one channel at a time) and passes the body action to _filter / _decide (:8682). _decide reads the world frame only through w = tick.frame.to_world, for _check and for the records. _repair_chain (:7963), _relax (:8127), _clearance_escape (:8657) and _opened_escape (:8670) are handed the body action and hand one back. guardrail/shield.py imports no rotation; tests/test_frame_boundary.py:888 ROTATION_USERS lists only frames.py, repair.py to_world and one record helper. One rotation pair per tick: guardrail/repair.py _FramePair :180 (to_body :193, to_world :209), BodyFrame :228, RepairContext :240 (shared memo). Operators: HookOperator :342, PathRepair :442, GrantOperator :831, register :898, ClearanceEscape :927, OpenedEscape :986. Each runs its world geometry between ctx.to_world and ctx.to_body, the grant reference's own pattern (kuanting-vla-uav-guardrail/packages/safety-shield/src/safety_shield/kinematics.py:36/44 body_to_enu / enu_to_body). Body to local NED: one function, frames.body_to_local_ned (frames.py:349), called by no module in guardrail/ other than its definition. On the grant's ROS 2 / MAVROS 2 rail it is called only in sitl/mavlink_adapter_node.py: body_to_ned :782-790, setpoint_from_body :832, applied at :1473 in _on_setpoint (:1441) to every /shield/setpoint before the PositionTarget is published (:1479-1486), heading from the MAVROS pose quaternion. Other callers: demo/pas_ardupilot/autopilot.py:197/214, sitl/run_sitl_demo.py:409, and the simulator demo loops (demo/run_demo.py:342, vla_fly.py:910/2292, follow_vlm.py:5096/5440/5456/8073, aerialvla_demo.py:476, aerialvla_pas_demo.py:596, real_vla_demo.py:1993, semantic_demo.py:462, semantic_seek.py:564), each once, after the Shield. tests/test_frame_boundary.py:915 pins the caller list. Two deliberate changes (CHANGELOG 2026-11-26): (1) an untouched tick now returns the body action handed in, not its round trip (at most 3.6e-15 m/s apart); (2) at a heading that is not a quarter turn, a NaN horizontal body channel beside a finite one is now zeroed alone and the finite channel flies, where before both went to zero. Change (2) is less conservative; the action-finite violation is still raised (`test_sanitise_runs_on_the_body_action_one_channel_at_a_time`). Tests re-run in the 26 Nov review (vla-real env): tests/test_frame_boundary.py 23/23 (including `test_the_stack_carries_the_body_action_handed_in_at_any_heading`), tests/test_shieldcore_repair.py 34/34 (old-chain equality for finite actions at random headings). Equivalence is also stated in CHANGELOG 2026-11-26 and commit 35df373, with no committed record: the 64-episode headless smoke (12,186 records, max float difference 0.0), 654 ntFIN5 runs replayed through filter and filter_body (303,048 ticks), and 4,480 seeded ticks on 28 policies. docs/data/replay_agx_ntFIN5 and docs/data/kpi_ntFIN5/replay_at_revision_ntFIN5.json are the WP4-11 same-revision replays, not this comparison. Flown at a3d616e on hil (AGX Orin, ArduCopter 4.5.7 SITL over MAVROS 2): smREL071, 52 runs (12 refused by the profile's design, as in smREL070), all at code_revision a3d616ec6ec8, not dirty, kpi_grade true. P0 escape 0 on every Shield-on family, control 0.65 (docs/data/kpi_smREL071/, committed 704b656 + aabfd8e: re-score, replay at revision and bag hops in 704b656, the KPI report in aabfd8e; the runs on the offload drive, flight_log frame "body"). Untouched non-braked body ticks: 5,501 come back with raw_body == emitted_body exactly. The 84 other ticks are FSM Loiter (83) or RTL (1) holds. adapter_log.jsonl: on 16,046 ticks body -> NED matches a fresh rotation at the logged heading_deg within 1.5e-4 (the log's 4-decimal rounding); 1,878 of them are at non-quarter headings. Spot check smREL071_019_dyn-nfz-translate tick 74: GeofenceSlide emitted_body [0.00706, 3.99999, -0.00464, 0]; the adapter logs the same body at heading_deg 246.404 -> ned [3.6627, -1.6076, 0.0046, 0], and a hand rotation gives 3.66273, -1.60760. Earlier flights of the body-in / adapter-NED path: smFIN2, ntFIN4 (59e19a5), smREL070, ntFIN5 (21c2771), SITL at boot headings -0.4 / 135.6 deg (docs/data/sitl_bodyframe_20261120/). | DONE 26 Nov (independent review, 3 of 3). None for the grant sentence. Residuals: (1) The simulator demo loops (demo/run_demo.py, vla_fly.py, follow_vlm.py and the others listed) call frames.body_to_local_ned in their own loop, after the Shield, not through a MAVLink adapter, because they send Project AirSim velocity commands and have no MAVLink link. They use the one function, and tests/test_frame_boundary.py:915 pins the caller list. This is accepted under ARCH-04 (DONE, independent review 20 Nov). (2) The NaN-channel change (the finite channel now flies) is less conservative than before. It is pinned by a test but has not been flown. (3) The opt-in arc forecast (`--arc-forecast`, off by default) is unflown: a body command with a yaw rate is forecast along a straight world line. (4) An operator registered through repair.register is handed the body action (forward / right), as the grant asks; one written for North / East must convert through ctx.to_world. (5) smREL071 is committed since aabfd8e (docs/data/kpi_smREL071/). The filter vs filter_body equivalence numbers (654 runs / 303,048 ticks, 12,186 smoke records, 4,480 seeded ticks) exist only as CHANGELOG / commit statements (35df373); they need a committed record before they are cited as evidence of record. |
| WP3-05 | Lookahead horizon 5 s at 10 Hz = 50 future poses; time-window boundary checked at t-ε and t+ε | DONE | guardrail.shield LOOKAHEAD_S 5 s / LOOKAHEAD_DT_S 0.1 s on every rail: ROS 2 and PAS rails and the harness (CHANGELOG 2026-10-07, -13), the city rail since 2026-11-03 (demo/follow_vlm.py; fc5s_manual_r1-r3_20261103), and since 2026-11-06 the six older demo scripts (aerialvla_demo, aerialvla_pas_demo, real_vla_demo, run_demo, semantic_demo, semantic_seek), which built 3 s / 0.5 s; flown on hil: hil_smoke_03 (KPI-grade, 50 future poses at 0.1 s, the commanded-velocity forecast) escalated from the forecast's depth at the horizon; window-edge checks at t-eps and t+eps | - |
| WP3-06 | Repair operators in priority order: AltitudeClamp -> LateralProjection (SDF) -> PathRepair; conservative cap θ (2.0 m lateral / 0.5 m vertical) above which projection is abandoned for fail-safe; pluggable Protocol | DONE | guardrail/repair.py runs the grant's three operators in its order. with_path_repair (:1054) puts PathRepair right after LateralProjection, and this project's KinematicClamp and Clearance-, Standoff- and CorridorProjection sit in the same chain. Pluggable: RepairOperator Protocol (:330), with GrantOperator, adapt and register (:836, :886, :903) taking the grant's three-argument signature. θ is 2.0 m / 0.5 m (guardrail/fsm.py FSMConfig :1307-1308) on each repair's forecast depth in metres (docs/DESIGN-escalation-fsm.md, 2026-11-02). A repair over θ abandons projection and hands the step to the FSM's escalation, flown as X1 in 10 of the 52 smREL073 missions ("repair over theta, ... vs theta 2.00 / 0.50 m; projection abandoned"; <store>/hil_campaigns/hil_smREL073/episode-*/metrics.json). The loop is bounded at 3 passes (guardrail/shield.py _REPAIR_PASSES :1097). Read as: the iterated chain is the default selection (DEFAULT_REPAIR_MODE "chain", :1051). The grant's first-feasible rule is available but off by default, through --repair-mode first_feasible on sitl/ros2_shield_node.py, sitl/run_sitl_demo.py and experiments/sweep_scenarios.py. PathRepair's zone detours are opt-in in chain mode (PathRepair(chain_categories=("geofence", "standoff"))). Headless A/B (docs/data/repair_mode_ab_20261127/README.md, 179 Shield-on episodes per arm, not KPI-grade): chain fail-safe 179/179, 0 P0 escapes in 11,385 P0-violation ticks; first_feasible 177/179, 0 P0 escapes in 12,452, 6 gate failures. Flown on hil in ntFIN5 (release 0.7.0), the KPI campaign of record: PathRepair acted on 113, 114 and 114 ticks in the three standoff-approach-angles@offset_m=9 pedestrian cells, mean magnitude 0.684-0.685 m/s, P0 escape 0, fail-safe 3/3 in each (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md:1148-1150, 1609-1611). repair_mode chain is recorded in 52/52 runs of smREL072 (docs/data/kpi_smREL072/README.md) and of smREL073 (<store>/hil_campaigns/hil_smREL073/episode-*/metrics.json). | PathRepair detours around keep-out zones (opt-in) and stand-off rings, not around a corridor (CorridorProjection keeps the corridor), and the follow and perception rails fly the chain without PathRepair (guardrail/replay.py FOLLOW_RAIL_PATH_REPAIR = False). |
| WP3-07 | Escalation FSM Normal -> Brake -> Loiter -> RTL -> Land with N-in-T (3 in 5 s), θ, T_recover = 2 s, YAML-overridable; drives SET_MODE GUIDED/LOITER/RTL/LAND | DONE | guardrail/fsm.py FSMConfig (:1305-1310): N 3, T 5.0 s, θ 2.0 / 0.5 m, T_recover 2.0 s, loiter timeout 30.0 s. Per-profile YAML override: load_fsm_config(path, profile) (:1510), which fails closed on an unknown profile; experiments/fsm_profiles.yaml; tests/test_fsm.py:1799; --fsm-config / --fsm-profile on sitl/ros2_shield_node.py (:6885-6887). The selection flew on hil in fsmp1125_001 (docs/data/hil_1125_x11_wp307/README.md). Every edge of the ladder flew on hil with the grant's triggers. hs10 (CHANGELOG 2026-11-14 (hs10)): G2 "N-in-T: 3 onsets in 5 s >= N=3", G4 "N-in-T persists", G6 on a blocked leg home, X2, X3. bat1125_001: G6 on low battery. smREL073 at release 0.7.3, 52 missions (<store>/hil_campaigns/hil_smREL073/episode-*/metrics.json), edges taken: G1 41, G3 38, G5 19, G6 3, G7 8, G8 18, G9 19, G10 4, G11 2, G12 1; SET_MODE requests GUIDED 18, RTL 23, LAND 31 (mission-end landings included). Read as: the FSM's Loiter is held in GUIDED by default (node --hold-flight guided, :6915; sweep DEFAULT_RAIL_HOLD "guided", experiments/sweep_scenarios.py:1045); in smREL073, 27 missions entered Loiter and none requested LOITER. Literal SET_MODE LOITER is opt-in (node --hold-flight brake --hold-mode LOITER --literal-loiter on; sweep --rail-hold loiter). It flew on hil in sk1122b: mode requests LOITER then RTL, autopilot heartbeat LOITER, held 29.9 s, P0 escape 0 (docs/data/shakedown_20261122/README.md). The real FC (CUAV V5 Nano, ArduCopter 4.7.1, never armed) also accepted a LOITER mode request on the bench (docs/data/fc_bench_20261117/README.md). | Literal LOITER (opt-in) holds where the aircraft stopped, not at the Shield's moved hold point, and on SITL it relies on a throttle stand-in (RC3 1500). |
| WP3-08 | Shield acts on the rule's violation_action (monitor_only, project_fix, brake, loiter, RTL, land); hard vs soft constraints; P0/P1/P2 priority | DONE | dispatch on all six actions in guardrail/shield.py; experiments/templates/failsafe_escalation.yaml headless; no policy file in policies/ uses loiter/RTL/land/monitor_only actions; they fly as zones the scenario (experiments/templates/failsafe_escalation.yaml) or REST (below) adds; 29 Oct, ROS 2 rail + MAVROS, one REST-spawned zone per action: land (G12 -> LAND), RTL (G11), loiter (X4), brake (X1, held), project_fix (ros2_hotapply_03), monitor_only as a soft P2 rule (flown through, P0 0.0) and as a hard P0 rule (counted as escapes, by design) | One zone per action on the flown policy, not one policy file per action. KPI-grade on hil (nightly ntFIN2, 13 Nov): RTL rules (G11, 18 transitions), land rules (G12, 3), loiter rules (X4, 3) and project_fix rules flown; monitor_only not checked there. |
| WP3-09 | MAVLink adapter ROS 2 node: single body->local-NED boundary; SET_POSITION_TARGET_LOCAL_NED; MISSION_ITEM_INT; SET_MODE for FSM | DONE | sitl/mavlink_adapter_node.py, SET_MODE via ModeRequests; flown 7 Oct (ros2fix2_*); since 3 Nov the MISSION_ITEM_INT row: sitl/ros2_mission_node.py (STYLE=mission) checks a waypoint plan with the Shield's own position check and clamps its speed to the policy's cap, the adapter writes it through /mavros/mission/push and reads it back, and in AUTO the next waypoint is replaced in flight by a partial write (MISSION_WRITE_PARTIAL_LIST + one MISSION_ITEM_INT) when a hot-applied zone makes its leg illegal, the original written back once the leg is legal; flown mission_wp_20261103d (re-flown 6 Nov from the main checkout with the six-field manifest the mission node writes since then: mission_wp_20261106b - code_revision a3c36a51e28c, topology dev, sim_speedup 1.0 read from the autopilot; the 4 Nov re-flight mission_wp_20261104 has no manifest) (ROS 2 rail + MAVROS 2, dev, GCS-set fence): the bag's /uas1/mavlink_sink carries 6 MISSION_ITEM_INT and no MISSION_ITEM (tools/bag_mission_protocol.py), the aircraft held at the replaced waypoint (y <= 15.35 m; the zone's margin begins at 17 m), both updates confirmed by read-back, mission complete in 28.1 s, no tick with the position illegal (CHANGELOG 2026-11-03 (G)); 15 Nov (code + tests, not flown): the in-flight update is written as the whole mission with the one item replaced (MISSION_COUNT, MISSION_ITEM_INT per item, MISSION_ACK ACCEPTED; --mission-update-write partial keeps the 3 Nov write), one update at a time (MissionPushQueue) (tests/test_mavlink_adapter.py 47/47); the perception rail can send its setpoints through the adapter (--link mavros --setpoint-via adapter) (CHANGELOG 2026-11-15); 15 Nov flown: the full write in AUTO, mission_wp_20261115b (6c9698d, ROS 2 rail + MAVROS 2, dev, GCS-set fence, sim_speedup 1.0 read from the autopilot): the harness spawned a zone across the first leg (generation 1) and switched it off (generation 2); the replacement and the restore were each written whole - MISSION_COUNT 4, four MISSION_ITEM_INT, MISSION_ACK ACCEPTED, MAVROS's push answered success, read back 4 matching - and the bag's /uas1/mavlink_sink carries 12 MISSION_ITEM_INT, no MISSION_ITEM and no MISSION_WRITE_PARTIAL_LIST; the aircraft held at the replaced waypoint (y <= 15.38 m; the zone's margin begins at 17 m), last waypoint reached in 28.2 s, 0 ticks with the position unsafe (docs/data/wp3_09_20261115/); the perception rail's adapter route on hil: pas_hil_adapter_01 (KPI-grade, AGX Orin), its setpoint path /shield/setpoint (body) -> sitl/mavlink_adapter_node.py -> /mavros/setpoint_raw/local, 4,092 setpoints forwarded, 0 P0 escape ticks (CHANGELOG 2026-11-15 (hot-apply on hil; WP3-09 flown)) | the FSM's Loiter is held in GUIDED by the companion by default since 7 Nov, and literal SET_MODE LOITER is opt-in (reading (e), WP3-07; a LOITER request that reaches the adapter is sent as BRAKE unless --hold-mode LOITER is set, sitl/mavlink_adapter_node.py DEFAULT_HOLD_MODE); the mission style runs no escalation FSM and writes no kpi.json, and MISSION_ITEM_INT was flown on dev only; the perception rail's default route publishes its own PositionTarget through the same frames.py conversions (the adapter route is opt-in; the KPI campaigns fly the ROS 2 rail, whose only setpoint path is the adapter). |
| WP3-10 | ArduPilot GeoFence (FENCE_ENABLE, FENCE_TYPE, FENCE_ACTION) configured as the last-line backstop that fires only if the Shield fails | DONE | sitl/fence/guardrail_fence.parm; fence fired with the Shield off (ros2fix2_off_fence); PAS rail uploads zones before arming (CHANGELOG 2026-10-13), 0 breaches; since 3 Nov GCS-set: sitl/gcs_backstop.py, the ground station's own process on mavlink-router's GCS endpoint (run_ros2_demo.sh GCS_BACKSTOP=1, SITL without the defaults file), sets FENCE_TYPE, FENCE_ACTION, FENCE_ALT_MAX, FENCE_MARGIN, AVOID_ENABLE and FENCE_ENABLE and uploads the signed bundle's zones as MISSION_ITEM_INT, reading every value and item back (gcs_backstop.json; the Shield node's own read-back matches it); flown with the Shield off - gcs_fence_off_20261103 (re-flown 4 Nov from the main checkout, code recorded: gcs_fence_off_20261104) (6 m/s) and _20261103b (3.5 m/s): 'Fence Breached' and GUIDED->RTL at 3.37 s and 4.08 s - and on - gcs_fence_on_20261103b and gcs_fence_on_20261103c (re-flown 4 Nov from the main checkout, code recorded: gcs_fence_on_20261104) (past the zone's corner at 2 m/s: the Shield bent the route around it, the aircraft passed 1.58 m from the zone and reached its target): no breach; the 29 Oct Shield-on breach (ros2fix_on_policy55) is explained by its own records: poly-test's tri-nfz has a vertex at home, and the breach came at 26.5 s, after the FSM's RTL had brought the aircraft home and the mission had ended at 10.2 s (CHANGELOG 2026-11-03 (G)) | GCS_BACKSTOP=1 is opt-in (the default rail keeps the defaults file, so Mission Planner keeps the GCS endpoint); the GCS step is a scripted stand-in for an operator in Mission Planner; a GCS-set fence is static, so a hot-applied zone is the Shield's alone, and a zone over the pad is held back and not backstopped. |
| WP3-11 | Per-violation-tick JSONL audit record (Pydantic v2) with ts, policy_hash, generation, monitor_tick, raw_action, violations (boundary_intersect_at_s), repair_attempts (result/magnitude_m/iterations), emitted_action, fsm_state_before/after; rotated per mission | DONE | 2 Nov, audit record version 4 (guardrail/audit.py AuditRecord, Pydantic v2; policies/audit_record.schema.json): the grant's ten fields written first, in the grant's order, with the grant's content - raw_action / emitted_action body frame {vx, vy, vz, yaw_rate} (frames.to_body at the tick's heading, converted when the record is written), a violation's category geometric / envelope / time-window with boundary_intersect_at_s, repair_attempts the operators tried (ok / skip with its reason, magnitude_m, iterations = the pass); the grant's p5 example validates verbatim; a world-frame action under the grant's name is refused; this project's content under raw_action_world / emitted_action_world, rule_category, repair_results; every rail writes shield_audit.jsonl, one file per episode (a stale one moved aside); every stored record (294 files, 71,064 records, versions 1-3) reads through upgrade_record; tests/test_shieldcore_audit.py 16/16 | None for the record. Since 26 Nov the Shield's stack carries the body action (WP3-04, DONE); operator geometry and the *_world record fields use North/East/up through the tick's repair.BodyFrame (whether raw_action / emitted_action are still converted at write time was not checked in the 26 Nov review). `contract` (a non-finite command zeroed) is a fourth category value, this project's extension. Flown since: the hil runs write it, e.g. demo/out/hil_smoke/ntFIN2_178_dyn-nfz-spawn-ahead/shield_audit.jsonl (13 Nov; the grant's fields first, raw_action in body frame, category geometric with boundary_intersect_at_s, repair_attempts with result and reason). |
| WP3-12 | Hot-apply listener (REST callback) bumps policy generation mid-flight | DONE | --hotapply-port on sitl/ros2_shield_node.py and sitl/run_sitl_demo.py (CHANGELOG 2026-10-13); since 15 Nov the perception rail serves the same listener (demo/pas_ardupilot/rail_hotapply.py, --hotapply-port; CHANGELOG 2026-11-15 (WP1-15, C-16, WP3-09)); 29 Oct: REST hot-apply into flying ArduPilot SITL + MAVROS 2 (ROS 2 rail, dev): a dynamic_nfz spawned, rotated, translated and switched off (time_window_switch) mid-flight, generations 0-4, receipt to swap 0.444-4.443 ms against 50 ms; on ros2_hotapply_03 the Shield and the FSM stopped the aircraft short of the REST zone (X1, G3), released it after the switch-off (G8), and it reached the target | Driven KPI-grade on hil: hh02 (14 Nov, the harness over mTLS REST to the AGX Orin's listener, six episodes; WP4-03) and pas_hil_hotapply_01, pas_hil_adapter_01, pas_hil_hotapply_02 (15 Nov, 14 REST events, 0 P0 escape ticks; docs/data/pas_hil_hotapply_20261115/flights.md; WP1-15). |
| WP3-13 | Monitor tick budget ≤100 ms end-to-end at 10 Hz; geometry query budget ≤5 ms with ~50 rules x 50 poses | DONE | Read as: the tick budget is shown by the stub-pilot hil campaigns (AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2) plus the offline 50-rule replay on the Orin. Geometry query at release 0.7.3 (c44a360), AGX Orin (MAXN, clocks not locked, no VLA server), 50 rules x 51 poses over 1,301 real flight ticks: _check p99 1.048 ms, max 1.097 ms, 0 over 5 ms; whole filter step p99 27.98 ms, max 50.03 ms, 0 over 100 ms (deploy/evidence/hil/shield_tick_profile_20261128.json; the as-flown rules of 4 older flights topped up to 50 with 44-47 added zones). Flown tick, smREL073 at 0.7.3 (52 KPI-grade missions): tick_ms p99 25.5 / max 55.73 ms, 0 of 16,047 over 100 ms; e2e_ms max 55.81 ms; the bag's /vla/action_4d to /mavros/setpoint_raw/local hop corroborates it, max 57.69 ms (docs/data/kpi_smREL073/README.md). ntFIN5, the KPI campaign of record (release 0.7.0, 654 missions): tick_ms p99 28.3 / max 62.08 ms, e2e_ms and e2e_with_wait_ms max 62.18 ms, 0 of 183,961 over 100 ms (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md:19-21). e2e_ms runs from the node's action receipt to its setpoint. | No flight served a learned VLA beside a ~50-rule bundle, and the pilot-to-node DDS hop and the MAVROS-to-autopilot MAVLink hop are not timed. Learned-VLA flights on the PAS perception rail, outside this basis, had 4 of 27,582 ticks over 100 ms (max 142.85 ms, on clearance repairs; docs/data/kpi_prefix_vla_20261120/README.md). |
| WP3-14 | P0 violation escape rate, target 0 (hard limit), on the canonical KPI configuration | DONE | Grant (02-implementation/stress-testing.md): "P0 violation escape rate \| 0 (hard limit) \| Counter from Shield repair log"; safety-shield.md: "report cannot ship if non-zero on P0". 13 Nov, the KPI-grade nightly hil campaign ntFIN2 (116214f, profile nightly, the AGX Orin rig; demo/out/hil_ntFIN2/sweep.json, runs demo/out/hil_smoke/ntFIN2_*; tools/kpi_report.py --campaign): 0.0000 - 0 escapes on 16,703 P0-violation ticks over 577 Shield-on missions - against 0.6375 for the Shield-off control nfz-head-on-control (153 of 153 P0 ticks, 3 missions) and a passthrough null of 0.213; the 3 gps-dropout flights the campaign did not accept also 0. Before it, 0 P0 escapes: harness library 179 Shield-on episodes (CHANGELOG 2026-10-22), every kpi.json of the 45 flights 27-28 Oct, PAS+ArduPilot Shield-on flights 12-21 Oct, none of these KPI-grade; on `hil` since 7 Nov, KPI-grade (topology hil, sim_speedup 1.0, a clean revision, the Orin's hil_evidence): P0 escape rate 0.0 on every Shield-on mission of the smoke campaigns rh03 (44 missions, fb96fe5), sm1110 (47, 9eed35b) and sm1111 (45, 22d434b) and of the grant example sweep ge1110 (32, 9eed35b), against 0.646 and 0.650 for the Shield-off control nfz-head-on-control in sm1110 and sm1111 (CHANGELOG 2026-11-11 (hil rig flights)); and 0 escapes on all 587 Shield-on missions of the first nightly nt1111 (22d434b, demo/out/hil_nt1111/sweep.json). At release 0.7.0, the KPI-grade nightly ntFIN5 (21c2771, 24 Nov): 0.0000, 0 escapes on 17,194 P0-violation ticks over 651 Shield-on flights, against 0.6500 for the Shield-off control (156 of 156 ticks, 3 flights) and a passthrough null of 0.1938 (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-24.md line 17); ntFIN3 (71aabb3) and ntFIN4 (59e19a5) likewise 0 (17,457 and 17,078 ticks); the release smoke smREL070 0 on 2299 ticks over 51 flights (docs/data/kpi_smREL070/). | None for the grant's count. To state beside it: on the true position no Shield-flown tick entered a P0 zone, but in ntFIN2, in 21 Shield-on missions ArduPilot carried the aircraft into one (demo/out/hil_ntFIN2/sweep.json `_true_state`): 18 dyn-nfz-spawn-ahead runs whose zone appeared 3 m ahead at 4 or 8 m/s, closer than the aircraft can stop, where the Shield braked and held and ArduPilot's GeoFence reported "Fence Breached" and took RTL at 5.7-5.8 s (2.3-2.7 m deep), and 3 time-window-opens-on-sideways-pilot runs whose RTL leg flew back through the zone. The pilot is the stub (a learned VLA on hil is ARCH-20). The supplement ntFIN2s (merged, 9 episodes counted): 0 escapes on 16,760 P0 ticks over 586 Shield-on flights; ArduPilot carried 3 more missions into a P0 zone (ntFIN2s_013-015, wind-beyond-authority lane 20 at 12 m/s; demo/out/hil_ntFIN2s/sweep.json `_true_state`), 24 in all. In ntFIN5 the autopilot alone took the aircraft into a P0 polygon in 23 missions (18 dyn-nfz-spawn-ahead, 5 wind-beyond-authority: ntFIN5_647-648, 652-654), with no Shield-flown tick inside. |
| WP3-15 | Fail-safe trigger correctness (reference target ≥99%: triggered when expected, not when not expected) | DONE | guardrail/fsm.py score_failsafe_triggers with nulls. The rubric was written before relabelling (docs/RUBRIC-failsafe.md; tools/failsafe_rubric.py). Reading (r2), the reading the project applies, has rail-vehicle labels with E5 on the prescribed flight (since 10 Nov; labels docs/data/failsafe_labels_rail.json derived at 442397f, 49c455c, 897dac7). GeoFence backstop rule: decision 14, applied since 15 Nov (docs/RUBRIC-failsafe.md section 9.2; experiments/sweep_scenarios.py failsafe_backstop, BACKSTOP_RULE). KPI-grade nightly ntFIN5 at release 0.7.0 (21c2771, 24 Nov; docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-24.md line 18; README): (r2) **649/649 = 1.000, Wilson 95 % low 0.994**; 18 triggered by the GeoFence backstop (ntFIN5_178-186, 214-222, all dyn-nfz-spawn-ahead, all expected and correct, RTL at 5.73-5.87 s); without the rule 631/631; counted as not triggered 631/649 = 0.972; (r) 637/649 = 0.982; (a) 574/649 = 0.884; left out ntFIN5_373 and ntFIN5_374 corridor-swap-shifted, censored in Loiter. The release smoke smREL070 (21c2771): (r2) 50/50, 1 censored. Before it, ntFIN4 at 59e19a5 (docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.md lines 18 and 72-76): (r2) 648/648 = 1.000, Wilson 95 % low 0.994, nulls always/never 0.506/0.494. 18 were triggered by the GeoFence backstop, all dyn-nfz-spawn-ahead, all expected and correct (ntFIN4_178-186, 214-222). With those 18 left out as censored, the rule not applied: 630/630, Wilson low 0.994. Counted as not triggered: 630/648 = 0.972. The other readings: (r) 636/648 = 0.981; (a) 573/648 = 0.884; the campaign's own labels 586/649. Left out: ntFIN4_374 and ntFIN4_375 corridor-swap-shifted, censored in Loiter (648/650 = 0.997 even if both were errors). smFIN2 (59e19a5): (r2) 51/51. Shakedown sk1122a: 72/72, with the 60 GPS-noise/latency cells flown on the rail for the first time (24 beyond-margin fail-safes, 36 within-margin none, as labelled). Earlier: ntFIN3 (71aabb3) 587/589 = 0.997, Wilson low 0.988 (misses ntFIN3_164, ntFIN3_168); ntFIN2 (116214f) 584/585 = 0.998. The headless nightly gives 741/741 before and after the retreat ray (CHANGELOG 2026-11-22 (code)). | The KPI holds at the point estimate and at 95 % confidence only under reading (r2): (r) gives 0.982 and (a) 0.884 on the same 649 ntFIN5 runs. (r2) and the GeoFence backstop rule (decision 14) are the readings the project applies. Both came into use after rig flights they bear on ((r2) on 10 Nov, the backstop rule on 15 Nov), so neither is a pre-registered reading; the readings before them are printed beside them (docs/RUBRIC-failsafe.md sections 1 and 7-9), and both were in place before ntFIN5 was launched on 23 Nov (docs/data/kpi_ntFIN5/README.md). The figure does not need the backstop rule (631/631 without it). The grant calls the GeoFence a backstop that fires "only if the Shield itself crashes or fails to emit", and the 18 backstop runs still fail their own gates. Two nightlies in a row (ntFIN4 at 59e19a5, ntFIN5 at 21c2771) clear the 95 % bound at 0.994; ntFIN3 at 71aabb3 had 2 misses and a Wilson low of 0.988. Wilson treats seed-repeats as independent. The 90 refused ntFIN5 episodes (stub-unreadable arms, stressors the rail cannot realise) are scored headless only. |
| WP3-16 | Mean repair magnitude (mean //post_repair − planner_output//) | DONE | Grant (02-implementation/stress-testing.md): "Average repair magnitude \| tracked \| Mean of \|\|post_repair − planner_output\|\|". tools/kpi_report.py repair_magnitude_by_priority / _by_operator_class (CHANGELOG 2026-10-13); on `hil` since 7 Nov, KPI-grade, the mean over episodes: rh03 5.32 m/s, sm1110 5.28 m/s, sm1111 5.22 m/s (ge1110 made no repair) (CHANGELOG 2026-11-11 (hil rig flights)); 13 Nov, the KPI-grade nightly ntFIN2 (116214f, 577 Shield-on missions; tools/kpi_report.py --campaign): 6.407 m/s pooled over repaired ticks, per template (1.859 to 16.000 m/s) and per cell, and by priority and operator class | None for the tracked KPI. The norm is translational (vx, vy, vz in m/s); the yaw rate is reported per episode beside it, never folded in, so a reviewer reading the grant's norm over all four action components would get another number. |
| WP3-17 | Mean time to safe: from violation to back-in-safe-zone (reference sweep expects ≤2.0 s) | DONE | Grant (02-implementation/safety-shield.md): "Mean time to safe \| tracked; from violation to back-in-safe-zone". 13 Nov, the KPI-grade nightly ntFIN2 with its supplement ntFIN2s (116214f, 586 Shield-on flights; tools/kpi_report.py --campaign; docs/data/kpi_ntFIN2/kpi_report_ntFIN2_2026-11-13.md): 3.35 s mean, max 10.90 s, over 172 unsafe-position spells that recovered; 75 more spells never recovered before an RTL or Land (54 three_simultaneous_violations, 18 dynamic_nfz_movement, 3 failsafe_escalation), counted, not averaged; per template 0.47 to 7.78 s. Before it: 29 Oct: a recovery from outside an altitude band flies at the climb cap (the Shield's recovery_law 'cap', 5c98142); 3 Nov republished library (docs/data/scenario_sweep.json): mean time to safe 2.997 s over 39 recoveries, max 6.7 s (3.135 s over 37 before: X7 in Loiter ends one wind recovery sooner, longer runs add two); 4 Nov republished from the merged tree (a586956, clean): 2.926 s over 39, max 6.7 s - the one cell that moved is dyn-nfz-spawn-on-top@size_m=20 (5.6 s -> 2.8 s: the zone appears centred on the aircraft, every edge equally near, and since the tie is broken by the commanded heading the pilot's own forward command is the way out, so the Shield passes it; its per-tick log flags the state unsafe on every tick inside); 8 Oct run 2.81 s; none KPI-grade; 7 Nov: the grant's worked example, the one sweep its 2.0 s belongs to, flown as written as a profile of its own (experiments/profiles/grant_example.yaml, 36 episodes, headless, on the ideal and the rail vehicle; CHANGELOG 2026-11-07 (grant example sweep)): P0 escapes 0, fail-safe 36/36 (every label not expected, N1), and no time-to-safe spell at all - the zone leaves the track 4.2 s after it appears, so its 2.0 s holds with nothing to measure; our time to safe (guardrail/kpi.py) counts each spell in which a stop at the true position breaks a rule within the 5 s lookahead, which starts no later and ends no sooner than 'from violation to back-in-safe-zone' read as position | None for the tracked KPI. The grant's one figure, <= 2.0 s, is the example sweep's, met there with no spell to measure (ge1110 on hil too); ntFIN2's 3.35 s is not held to it, and its mean leaves out the 75 spells that never recovered (of 247 unsafe spells); ntFIN5, the figure of record: 3.49 s mean, max 9.90 s, over 165 recovered spells, 94 never recovered (section 2; section 5.5). Our spell starts no later and ends no sooner than the grant's words read as position: a reviewer may ask for both beside it. |
| WP3-18 | Canonical KPI configuration 'hil': desktop runs sims + MAVROS + GCS, Jetson Orin runs VLA + Shield over the network | DONE | hil flown 2026-11-06 (CHANGELOG 2026-11-06 (hil)): ArduPilot SITL + mavlink-router (+ the GCS endpoint) on the desktop, MAVROS 2 + the Shield + the pilot node on the AGX Orin over the cable; the label comes from evidence (device tree, L4T, SITL elsewhere, the MAVROS chain), and hil_smoke_03 is KPI-grade. MAVROS sits on the Orin, as the PI's reference repository's topologies page and docs/DESIGN-topologies.md place it (the grant's table words it on the desktop; docs/DESIGN-projectairsim-ardupilot.md records the difference). The August 'canonical-hil' folders are read as dev by normalize_topology and refused by is_kpi_grade | - |
| WP3-19 | Smoke test matrix: high-speed near-edge, sudden NFZ (hot-applied), time-window switch instant, three simultaneous violations, GPS noise/latency | DONE | experiments/profiles/smoke.yaml carries the five rows: high-speed-near-edge :71-74 and near-edge-at-12mps :75-77, dyn-nfz-spawn-ahead/-on-top :45-49, time-window-* :54-60, three-simultaneous-violations :78-80, gps-noise-within/-beyond-margin :81-88. On the hil rail the dyn-nfz-spawn zone is hot-applied mid-flight by a scenario event (<store>/hil_runs/smREL073_017_dyn-nfz-spawn-ahead/report.md, "Hot-apply generations": g1 at t=5.07 s, scenario event spawn_polygon_fence dyn-nfz), and GPS noise / latency is applied to the pose the Shield and the FSM are handed (sitl/ros2_shield_node.py:1250 PerceptionStress, recorded in metrics.json perception_stress; experiments/sweep_scenarios.py:4446 rail_stress_check refuses a flight whose draws are not the cell's). Release 0.7.3 (c44a360), smoke smREL073 on hil (AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2), 28 Nov: 52 flown, all KPI-grade, single_sha 52/52; the five rows flown (_039-_042, _017-_018, time-window _022-_026 and time-window-switch-event _027, _043, _044-_046) with P0 escape 0 on each and a non-vacuous cell in each (4/4, 1/2, 3/5, 1/1, 2/3 cells); the at-speed near-edge cells _041 (ask 12 m/s) and _042 (ask 20 m/s) had 10 and 13 P0-violation ticks with their zone in force (scenario clock pinned, experiments/templates/high_speed_near_edge.yaml:105); overall P0 escape 0 on 2393 P0-violation ticks, fail-safe (r2) 50/50, 0 of 16,047 ticks over 100 ms (docs/data/kpi_smREL073/README.md; kpi_report_smREL073_2026-11-28.md:17-19, :48-64). Releases 0.7.0, 0.7.1 and 0.7.2 flew the same five rows KPI-grade in their own smokes (docs/data/kpi_smREL070/, kpi_smREL071/, kpi_smREL072/). In smREL070 and smREL072 the two at-speed near-edge cells flew on the wall clock outside the school-yard zone's Mon-Fri 07:30-17:30 window and were vacuous (no P0 tick; kpi_report_smREL070_2026-11-24.md:211-212, kpi_report_smREL072_2026-11-27.md:211-212), while the row's _039/_040 had P0 ticks (both reports :209-210); 0.7.3's clock pin and the non-vacuous gate of docs/RUNBOOK-release.md step 5 answer it (CHANGELOG [0.7.3]). "Every release" is read as every release from 0.7.0 on (reading (c); WP4-14). Scope: one seed per cell (seed 1001, the grant's "x 1 seed each"); the GPS noise acts on the pose the Shield and the FSM are handed on the companion, while the autopilot flies on the true pose (sitl/ros2_shield_node.py:216-221). | - |
| WP3-20 | Repair success rate (Safety Shield page, Acceptance KPIs (locked): 'tracked; informs Prefix Compiler effectiveness eval') | DONE | Grant (02-implementation/safety-shield.md): "Repair success rate \| tracked; informs Prefix Compiler effectiveness eval". guardrail/kpi.py repair_success with theta (CHANGELOG 2026-10-06); 0.776 on the 8 Oct pymavlink run; theta basis decided 2 Nov (reading A, the default the KPI already reads); 13 Nov, the KPI-grade nightly ntFIN2 with its supplement ntFIN2s (116214f, 586 Shield-on flights; tools/kpi_report.py --campaign; docs/data/kpi_ntFIN2/kpi_report_ntFIN2_2026-11-13.md): 98.1 % (26,521 of 27,025 repair ticks), always-brake null 0 %, per template from 0.0 % (failsafe_escalation, three_simultaneous_violations, time_window_static) to 100 %; the effectiveness eval it informs is the CSP A/B (WP2-09, AirSimNH) | None for the tracked KPI. Theta (2.0 m / 0.5 m as forecast depth) judged about a third of the converged ticks (ntFIN2 + ntFIN2s: 8,556 of 26,653, docs/data/kpi_ntFIN2/kpi_report_ntFIN2_2026-11-13.md; ntFIN5, the figure of record: 9,307 of 27,314, section 5.7); the rest carried only repairs theta does not judge, and the report shows how far the rate moves under a velocity reading of theta, for which the grant gives no horizon. |
| WP3-21 | Single IR contract: the Shield consumes the Policy IR and never re-parses the YAML/JSON source ('hard invariant; violating it ... breaks reproducibility', Overview cross-cutting invariants) | DONE | every rail flies the signed bundle; YAML only under --dev-yaml (demo/follow_vlm.py:122-193; CHANGELOG 2026-10-22); Jetson metrics policy_source kind bundle, hash verified | None. |
| WP3-22 | Exactly one rule-evaluation code path in the system ('Violation checking' section; violation checker library shares the IR) | DONE | Since 3 Nov guardrail/shield.py is the one evaluator, built from the IR (PolicyIR, compile_fence, FenceRecord): RuleClock (the compiler's _RuleClock subclasses it), the point verdicts (zone_breached :777, corridor and band tests, ZoneRuler :993, clearance_breached :877, standoff_breached :896, standoff_ring_m :935; since 27 Nov zone_footprint_reached :785, the closed form of zone_breached's footprint half, which models.start_conflicts asks) and escape_direction. compiler.action_violations, fence_export, rtl_path, replay.zone_dwell and FenceGuard's gate/slide all read it (CHANGELOG 2026-11-03 (E)). 17-18 Nov: every shapely, ray-cast and segment-distance rule test outside guardrail/{geometry,planar,shield,ir}.py calls the library; the semantic demo, semantic seek and A/B P0 counter ask the Shield (min_clearance_on_map, altitude_band_report, rule_status). The last open path was closed by wf21-onepath3 (d8fd243, fb6a27e, 18 Nov; CHANGELOG 2026-11-19 (wf21, code)): FenceGuard's building half measures with the Shield's field (Shield.clearance_ruler, :5943) and judges with clearance_breached against Shield.clearance_limits_now (:5967) (demo/follow_vlm.py:2556-2593); guard_field accepts only "shield" and refuses "grid" (:2419-2436, CLI choices :9355). The end-of-flight stand-off counts and the stand-off scorer call standoff_breached / standoff_ring_m; the landing column test asks the Shield (hard_violations, rule_status). The library keeps one rule measure, field_distance (:1258); nearest_cell_clearance (:1421) is diagnostic only, its one caller tools/onepath_guard_replay.py (tests/test_shieldcore_onepath.py: _CALLED_ONLY_IN :252, _LIBRARY_CLEARANCE_MEASURES :1073). Outside the library the only clearance measures are planning and target identity (city_planner.inflate/_penalty_flat, identity.__init__; _CLEARANCE_OUTSIDE :977). The scans find no wrapped primitive called elsewhere and no IR rule threshold compared outside the checker except 14 named non-verdicts (export, planning, scenario setup; _THRESHOLD_COMPARISONS_NOT_VERDICTS :1110-1174, test :1526); mutation nulls planted (:1574). They were 15 until 27 Nov, when models.start_conflicts left the list (82b9cd6). tests/test_shieldcore_onepath.py 27/27 in lab CI 8e73f86, 22 Nov (187/187 files; docs/data/ci_local/8e73f86b6af6.json). Flown with the new default: 7 AGX Orin CityLife follow flights at bfb4973, 19 Nov, all guard_field "shield" (runs_summary.json; docs/data/onepath_agx_20261119/README.md): onepath n=5 within-30 m 0.969/0.954/0.472/0.943/0.917, AerialVLA hierarchy 0.987, nfz3b 0.345; P0 escape 0 and 0 mission contacts on all 7; the guard's replay over the five onepath runs equals the logged mode on 99.97 % of 20,971 ticks; the 0.472 is an identity-lock loss, not the guard (diagnosis_20261119.md). 2 re-flights at 28a594d, 20 Nov (docs/data/follow_agx_20261120/, metrics.json guard_field "shield"): 0.945 and 0.48, P0 escape 0. The hil KPI campaigns run the same Shield: ntFIN4 at 59e19a5, P0 escape 0 over 650 Shield-on flights (docs/data/kpi_ntFIN4/). | Residual, outside the grant's words: (1) the follow flights are topology projectairsim-remote-sim, functional and not KPI-grade by design; (2) nearest_cell_clearance stays in the library as a replay-only diagnostic, test-pinned to one caller; (3) independent verification oracles in tools, such as tools/msl_dem_rail.py oracle(), check the declared bands without a Shield by design (with the bands copied as constants, which the threshold scan cannot see): they score runs and judge no flight; (4) models.start_conflicts is a load and authoring lint (frame origin inside a hard zone's footprint or margin, `<=`; tools/geo_to_policy.py:327-333 refuses to write such a take-off point), judging no flight; since 27 Nov (82b9cd6) it asks the Shield library's zone_footprint_reached (guardrail/shield.py:785, through a lazy import at guardrail/models.py:2330), the closed form of zone_breached's footprint half, with every answer unchanged against the old code kept as oracle (tests/test_start_conflicts_onepath.py 9/9: 37 policies, 66 hard zones, 75,020 probe points; start_conflicts strings identical); the in-flight verdict stays zone_breached, so the library holds a closed footprint test for the lint and an open one for flight; (5) ArduPilot's GeoFence evaluates the fences fence_export uploads from the Shield's geometry, the grant's own autopilot backstop (WP3-10; it triggered 18 of ntFIN5's 649 scored fail-safes, see X-13); (6) guardrail/fsm.py KinematicCaps.clamp (:864-874) mirrors the Shield's SpeedClamp / ClimbClamp (sharing fsm.cap_exceeded, the minimum cap across rules, no yaw clamp), used by follow_vlm.descent_ask and the scoring tools: a repair and planning mirror, not a verdict. The scans hold only the scanned forms (Python in SOURCE_DIRS, by pattern). |
| WP3-23 | Shield node architecture: sliding-window buffer (last 50 actions + predicted trajectory) and geometry queries on the IR's R-tree spatial index | DONE | since 3 Nov each filter() tick admits its action and that action's predicted trajectory into the buffer (shield.SlidingWindow, last 50) before anything is checked, and the checker reads the trajectory from it (a tampered buffer blinds the checker: tests/test_shieldcore_onepath.py); static zones through the IR's STRtree, moving dynamic_nfz through the IR's moving-zone STRtree rebuilt per tick (PolicyIR.moving_index), decisions identical to the one-by-one test on 3900 bench samples; library sweep 180/180 identical; tick profile within noise (CHANGELOG 2026-11-03 (E)) | None required. Notes: no DSL rule reads past actions, so the 49 older entries are a record; corridors, stand-offs and clearance are not in a tree (keep-in or subject-relative, checked at every pose); the per-tick rebuild costs about 0.06-0.1 ms per check on the 48-moving-zone bench |
| WP3-24 | Safety Shield ROS 2 node delivered as a 'Python package; rclpy' (Outputs table) | DONE | sitl/ros2_ws/ rclpy packages + launch files; tests/test_ros2_package.py | None. |

### WP4 - Stress Testing

21 items: 21 DONE.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP4-01 | Every reported KPI must come from a Stress Testing run in the canonical HIL topology | DONE | The KPI readers count only KPI-grade runs: hil at sim_speedup 1.0 on a clean revision, with the signed replay bundle verified and bound to the run and single_sha true (guardrail/manifest.py is_kpi_grade and stack_grade_reasons, guardrail/replay.py kpi_evidence_grade; kpi_report_ntFIN5_2026-11-28_offline_c44a360.md:970). Reading (h): the KPI campaigns flown with the stub pilot on the AGX Orin (Shield, harness, signed bundle; ArduCopter 4.5.7 SITL on the desktop, over MAVROS 2) are the canonical hil topology, the learned VLA is shown in separate hil flights, and the policy-artefact KPIs come from the offline tools at the delivered revision. From Stress Testing runs: the nightly ntFIN5 (release 0.7.0, 21c2771; 654 KPI-grade, single_sha 654/654) gives P0 escape 0 on 17,194 P0-violation ticks, fail-safe (r2) 649/649 (Wilson 95 % low 0.994), tick_ms max 62.08 ms with 0 of 183,961 over 100 ms, and mission success 288/651 (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md:17-19, :29). Bundle replayability: ntFIN5 654/654 and smREL073 52/52 (also from the episode bundles alone) identical when replayed at their revision on the AGX Orin; with every other KPI-grade hil run flown from 22 Nov on, 1,660 episodes in all, 0 differences (docs/data/replay_agx_rest_20261128/README.md, with docs/data/replay_agx_ntFIN5/ and docs/data/replay_agx_smREL073/). The prefix A/B comes from the hil campaign pab3 (33 KPI-grade; reading (j), WP4-17). Offline tools at c44a360 (release 0.7.3): round trip 33/33, tamper 33/33, negative 39/39 (docs/data/wp1_roundtrip_c44a360.json); CSP P0 coverage 124/124 at every budget from 256 to 144 tokens, largest CSP 149 tokens (docs/data/csp_kpis.json); on the AGX Orin, cold start p95 784 ms (deploy/evidence/hil/wp1_roundtrip_agx_c44a360.json) and hot-apply at 50 rules p99 4.97 ms (bench_shield_50rules_5s_agx_c44a360.json); the tick budget at 50 rules from the offline replay on the Orin, _check p99 1.048 ms and filter p99 27.98 ms (shield_tick_profile_20261128.json; reading (g)). Per-paraphrase robustness needs a pilot that reads text: the Stress Testing report reads n/m (:1896-1898), and AIoT-VLA v10b on the AGX Orin shows it in separate hil flights (PAS follow: 6/8 paraphrases ended as the source did, 0 P0 escape ticks over 13 runs; docs/data/pas_hil_paraphrase_20261115/flights.md; reading (h)). | Per-paraphrase robustness has no value from a Stress Testing run; the learned VLA shows it in separate hil flights. |
| WP4-02 | Scenario library: declarative YAML (one file per template) validated by Pydantic ScenarioSpec, with templates dynamic_nfz_movement / time_window_switch / radius_scaling / corridor_swap, events, parameter_sweep, expected_kpis | DONE | guardrail/scenario_spec.py ScenarioSpec/ScenarioEvent, experiments/templates/ 16 files, parameter sweeps; tests/test_scenario_spec.py | None (the library is flown headless, on hil and on Gazebo; WP4-04, WP4-06). |
| WP4-03 | Harness drives dynamic_nfz, time_window_switch and corridor_swap mid-flight via the Policy DSL REST hot-apply endpoint; events spawn/translate/rotate fence, activate/deactivate rule, swap corridor, scale radius | DONE | every event type mid-flight in the headless harness; gRPC HotApply forwards to a bound REST endpoint (CHANGELOG 2026-10-13); not through a flying rail until 3 Nov (below); 29 Oct: REST hot-apply into flying ArduPilot SITL + MAVROS 2 (ROS 2 rail, dev): a dynamic_nfz spawned, rotated, translated and switched off (time_window_switch) mid-flight, generations 0-4, receipt to swap 0.444-4.443 ms against 50 ms; on ros2_hotapply_03 the Shield and the FSM stopped the aircraft short of the REST zone (X1, G3), released it after the switch-off (G8), and it reached the target ; 3 Nov: the harness drove a flying rail - tools/harness_client.py drive, a gRPC session over mTLS bound to the rail's REST endpoint, the bundle injected pinned and checked against the endpoint, four HotApply at mission time (dynamic_nfz spawn 2.0 s, time_window_switch off 9.0 s, corridor_swap 15.0 s and 26.0 s) each POSTed by the harness to /hot_apply and applied (generations 1-4) on harness_corridor_20261103c (re-flown 4 Nov from the main checkout, code recorded: harness_corridor_20261104b), and two (spawn, switch off) on the mission-style mission_wp_20261103d (re-flown 6 Nov from the main checkout with the six-field manifest the mission node writes since then: mission_wp_20261106b - code_revision a3c36a51e28c, topology dev, sim_speedup 1.0 read from the autopilot; the 4 Nov re-flight mission_wp_20261104 has no manifest) (CHANGELOG 2026-11-03 (G)) 14 Nov, the first harness-driven hil episodes, campaign hh02 (bc04e4a on the desktop and the AGX Orin, sim_speedup 1.0, all six KPI-grade; python -m guardrail.harness_service serve --rail hil + tools/harness_hil_episode.py; bundles demo/out/harness/hh02/): StartEpisode launched each mission on the rig without its events and the harness forwarded them over mTLS REST to the Orin's hot-apply listener (the companion's api layer, deploy/docker/companion-api.Dockerfile), then EndEpisode scored the flight and wrote the grant's six names (manifest.json, policy_bundle.tar.gz with every generation, episode.bag2/ from the run's rosbag2, shield_audit.jsonl, harness_events.jsonl, kpi.json): spawn + rotate (hh02_001 dyn-nfz-rotate-step: POST /hot_apply at 3.00 s, PATCH /dynamic_nfz/bar rotate at 8.00 s), spawn + scale (hh02_002 radius-scaling-grow-deep, scale at 10.00 s), deactivate a rule (hh02_003 time-window-switch-event, switch_off at 1.02 s), activate a rule (hh02_004 rule-activated-mid-run@at=2, switch_on at 2.00 s), swap a corridor (hh02_005 corridor-swap-shifted, 5.02 s) and translate (hh02_006, an EmitEvent translate_polygon_fence of bar at 12.02 s after the scenario's spawn and rotate, generation 3); fail-safe under the adopted reading 5/5 with 1 censored (corridor-swap-shifted ended in Loiter, as in ntFIN2); P0 escape 0. The first flight hh01_001 applied both forwards and was refused by a harness check fixed in bc04e4a (CHANGELOG 2026-11-14 (the companion's REST hot-apply layer)). | Translate has no library cell (flown through EmitEvent); every library cell's events have flown through the harness only once, one seed each; the harness forward is late by up to ~0.2 s plus a round trip against the mission's own clock, so a harness-driven flight is not tick-identical to its campaign twin. |
| WP4-04 | Harness runs scenarios on the simulation rails (Gazebo Harmonic functional sweeps, KPI-grade runs in HIL) and emits an episode bundle per episode | DONE | experiments/sweep_scenarios.py flies the scenario library on each rail, one mission per scenario (`rail_refusal`, experiments/sweep_scenarios.py:4621, names what a rail cannot fly). It writes one bundle per flown episode under the grant's six names (manifest.json, policy_bundle.tar.gz, episode.bag2/, shield_audit.jsonl, harness_events.jsonl, kpi.json), listed in kpi.json `bundle_files`. KPI-grade on hil (AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2): ntFIN5, the nightly of record at release 0.7.0 (21c2771), 654/654 KPI-grade, 654/654 bundles with the six names, manifest code_revision 21c277170001 (docs/data/kpi_ntFIN5/README.md; the lab checkout's demo/out/hil_ntFIN5/episode-*). smREL073 at release 0.7.3 (c44a360), 52/52 KPI-grade, 52/52 bundles with the six names, manifest code_revision c44a360618e1, topology hil (docs/data/kpi_smREL073/README.md; `<store>`/hil_campaigns/hil_smREL073/episode-*). Gazebo Harmonic functional sweep: the full gazebo_functional profile (44 cells x seeds 1001-1003) at release 0.7.2 (418b151) flew 131 of 132 missions; all 131 bundles have the six names, and each one's own manifest.json records code_revision 418b1517da9f. P0 escape 0 on 128 Shield-on missions; desktop timing tick_ms p99 6.48 / max 25.06 ms and e2e_with_wait_ms max 97.36 ms, 0 over 100 ms. These runs are functional_only, never KPI-grade (docs/data/gazebo_sweep_20261128/README.md). One cell seed (standoff-approach-angles@offset_m=12,subject_class=pedestrian,variant=0, seed 1003) was refused before arming by the rail's home check on all 4 attempts, so it has no flight and no bundle. The broad sweep is WP4-06's row. | - |
| WP4-05 | RPC harness: Python service + gRPC stubs over mTLS (StartSession, InjectPolicy, HotApply, StartEpisode, IssueTask, EmitEvent, EndEpisode, streams) | DONE | guardrail/harness_service.py + guardrail/protos/harness.proto, all 10 RPCs over mTLS; headless until 3 Nov; tests/test_harness_service.py 57/57 (2 Nov). The bundle carries the grant's shield_audit.jsonl (30 Oct) and policy_bundle.tar.gz (2 Nov: the injected file byte for byte, generation 0's and the last generation's IR, every generation by hash, an index with each member's SHA-256; write_policy_bundle); kpi.json harness.bundle_files names the grant's three files and which are written; 3 Nov: a session bound to a flying ROS 2 rail's REST endpoint forwarded HotApply into its flight over mTLS with the SROS 2 keystore's lab CA (tools/harness_client.py drive; harness_corridor_20261103c (re-flown 4 Nov from the main checkout, code recorded: harness_corridor_20261104b), mission_wp_20261103d (re-flown 6 Nov from the main checkout with the six-field manifest the mission node writes since then: mission_wp_20261106b - code_revision a3c36a51e28c, topology dev, sim_speedup 1.0 read from the autopilot; the 4 Nov re-flight mission_wp_20261104 has no manifest); WP4-03; CHANGELOG 2026-11-03 (G)) See WP4-03: StartSession, StartEpisode (hil), the scenario forwards, EmitEvent (hh02_006) and EndEpisode flown end to end over mTLS on the hil rig, 14 Nov. | IssueTask / InjectPolicy were exercised headless only (tests/test_harness_service.py); the rig's hot-apply listener needs the companion api layer (docs/RUNBOOK-orin-hil.md 15a). |
| WP4-06 | Smoke (~50 scenarios x 1 seed per build), Nightly KPI (~200 x 3 seeds, HIL, sim_speedup 1.0), Broad sweep (~3000 x 5); sweep config YAML per profile; CI smoke pipeline | DONE | Profiles as YAML: experiments/profiles/smoke.yaml, nightly.yaml and broad.yaml. Reading (c): the smoke runs KPI-grade on hil per release and headless per push; the nightly KPI campaign is flown on the one rig for the release the KPI report of record describes (docs/RUNBOOK-release.md section 7: the nightly "is not part of the per-release obligation"): ntFIN5 at release 0.7.0 (77,356 s), after ntFIN4 at 59e19a5; releases 0.7.1-0.7.3 flew their KPI-grade smokes and no nightly; the broad sweep runs on the headless backend and was run in full at releases 0.7.2 and 0.7.3. Smoke (~50 x 1 seed, 1.0): smREL073 at release 0.7.3 (c44a360), 64 planned, 52 flown on seed 1001, all KPI-grade (docs/data/kpi_smREL073/README.md), as smREL070, smREL071 and smREL072 were at 0.7.0-0.7.2 (WP4-14). CI smoke pipeline: .github/workflows/tests.yml runs on every push (:715-716) with the step "Smoke profile (stress harness, headless)" (:1037), green in all 3 jobs on the 0.7.2 push (45969020953, 45969019967; docs/data/github_ci/); local CI at 16bb416 (code-identical to c44a360): 194/194 files, smoke rc 0 on 64 episodes (docs/data/ci_local/16bb4168b391.json). Nightly KPI (~200 x 3, hil, 1.0): ntFIN5 at release 0.7.0 (21c2771), 744 planned, 654 flown, all KPI-grade; P0 escape 0 on 17,194 P0-violation ticks, fail-safe (r2) 649/649 (docs/data/kpi_ntFIN5/README.md). Broad sweep (~3000 x 5, > 1.0): run in full on the headless backend (headless-kinematic, measured sim_speedup >> 1, outside the KPIs) at release 0.7.2: 2,990 cells x 5 seeds = 14,950 episodes in 43.8 min on 12 shards, P0 escape 0.0, fail-safe labels 14,945/14,945 (docs/data/broad_REL072/README.md, broad_REL072_summary.json), and again at release 0.7.3 (docs/data/broad_REL073/README.md: the same 14,950 episodes in 43.2 min, P0 escape 0.0, fail-safe labels 14,945/14,945, every episode's status, outcome and KPI record equal to 0.7.2's). .github/workflows/nightly.yml also schedules the nightly profile and the broad sweep (jobs broad, 12 shards, and broad-merge; pinned by tests/test_harnesstruth_ci.py). GitHub fires a schedule only from the default branch main, so neither job has run on GitHub (nightly.yml:6-12). | The nightly KPI campaign of record flew at release 0.7.0; releases 0.7.1-0.7.3 carry their own KPI-grade smokes. |
| WP4-07 | KPI auto-report generator (Markdown + JSON): per-scenario-family stats table (episode count, P0 escape rate, fail-safe correctness, mission success rate, mean repair count, mean repair magnitude) + top-K (K=10) failure cases linked to episode bundles | DONE | tools/kpi_report.py family table, top-K, Markdown+JSON, --baseline (CHANGELOG 2026-10-06, -13); tests/test_kpi_report.py, tests/test_wp4outputs_kpi_report.py | None (the report of record is tools/kpi_report.py over the 654 KPI-grade ntFIN5 runs; WP4-17). |
| WP4-08 | Mission success rate, tracked, defined as outcome == success AND no P0 violation, rolled up per scenario family | DONE | Grant (02-implementation/stress-testing.md): "Mission success rate \| tracked, no hard target \| outcome == success AND no_P0_violation". mission_success_rate 0.603 headless library (CHANGELOG 2026-10-22); follow flights scored by within-30 m; on `hil` since 7 Nov, KPI-grade, Shield-on missions, P0 escapes 0 on every one: rh03 19/44 = 0.432, sm1110 23/47 = 0.489, sm1111 22/45 = 0.489, ge1110 32/32, each campaign's kpi_report rolled up per family (CHANGELOG 2026-11-11 (hil rig flights)); 13 Nov, the KPI-grade nightly ntFIN2 (116214f; tools/kpi_report.py --campaign): 252/577 = 0.437 Shield on, rolled up per (template, parameter cell), hover null 0 | None for the tracked KPI. Our rule is stricter than the grant's (no dwell in a zone or ring and no unsafe position entered, besides the goal reached and no P0 violation), so the figure is at or below the grant's. |
| WP4-09 | Per-paraphrase robustness, a WP4-owned KPI: mission outcome stability across paraphrased task prompts | DONE | paraphrase arms in the harness (CHANGELOG 2026-10-07); since 7 Nov measured with a text-reading VLA: OpenVLA-7B flew the canonical CSP-on mission and 8 service-issued paraphrases, outcome stability 7/8 (utterance 4/4, verb_phrase 3/4) (docs/data/csp_flights_20261107.json, CHANGELOG 2026-11-07 (CSP flights)); 15 Nov on hil with a text-reading pilot, KPI-grade: AIoT-VLA v10b on the AGX Orin flew the PAS follow mission with the source "follow a red car" and the paraphraser's 8 stored paraphrases of it as its grounding text (de7e748; docs/data/pas_hil_paraphrase_20261115/flights.md): the source ended target_lost twice, the model's own noun phrase "a red car" held the car to max_s (as pas_hil_aiot_est_01 did); all 8 paraphrases flew a mission (the 2 whose start gate never passed, which reads no paraphrase, were flown again): 6 ended as the source did and 2 held the car to max_s; 0 P0 escape ticks over the 13 runs (CHANGELOG 2026-11-15 (paraphrases on hil), (Mission Planner; paraphrase retries)) | One flight per paraphrase, one mission and one task family; the stress harness's own paraphrase arms still fly the hil stub, which reads no text. |
| WP4-10 | Paraphraser: a separate service (Q3, Aug-Oct deliverable) that turns the Prefix Compiler's templated NL into free-form paraphrases issued to the VLA | DONE | same artefact as WP2-13: since 2 Nov a separate gRPC + mTLS service (guardrail/paraphraser_service.py) serving validated paraphrases of the templated NL; validator figures as WP2-13 (heldout_2026_11_02: 1/50 false accepts, 14/44 false refusals held out; 0/50, 3/44 dev); since 7 Nov the service's paraphrases are issued to a VLA: all 8 paraphrase flights of OpenVLA-7B drew theirs from the service on loopback (each run records `via`), and the gRPC harness (guardrail/harness_service.py, the reference design's orchestrator) draws every paraphrase arm from the service with no in-process fallback: a server bound to no service refuses the arm (CHANGELOG 2026-11-07 (CSP flights ready), (CSP flights)) | Not KPI-grade (AirSimNH). |
| WP4-11 | Every episode produces a self-contained bundle replayable bit-for-bit: manifest.json, policy_bundle (incl. all hot-apply generations), episode.bag2 (rosbag2), shield_audit.jsonl, harness_events.jsonl, kpi.json; Q4 reports Q3 runs 'replayed for reproducibility' | DONE | Each rail-campaign episode is written as one bundle under the six names, listed in kpi.json `bundle_files`: ntFIN5 654/654 (21c2771) and smREL073 52/52 (c44a360), each manifest.json with the six determinism fields. policy_bundle.tar.gz holds every hot-apply generation; for example, smREL073's dyn-nfz-rotate-spin bundle holds generations/policy_g0.json to policy_g2.json and the injected bundle, `bundle_files` 'written, complete'. Replay follows reading (i): open loop on the logged inputs at the flown revision, with the audit's `ts` and the FSM's fsm_* fields left out. tools/replay_at_revision.py rebuilds each episode's Shield from a detached worktree at its manifest's code_revision. It compares the shield_audit.jsonl records, and per flight-log row raw, violations, repairs, touched, braked and emitted, bit for bit. On the AGX Orin, in the hil companion container with the Shield node's interpreter (Python 3.12.3, numpy 1.26.4, aarch64), every KPI-grade hil run flown from 22 Nov on is identical at its own revision. ntFIN5: 654/654 (docs/data/replay_agx_ntFIN5/README.md). ntFIN4 654, smFIN2 52, smREL070, smREL071 and smREL072 52 each, pdyn1126 12, wgs01_001 1, ca1127_001 1, the 22 Nov shakedowns sk1122a 72 and sk1122b 2, and the 25 Nov runs x11r1125 2, bat1125 1 and fsmp1125 1: 954/954 (docs/data/replay_agx_rest_20261128/README.md). smREL073: 52/52 from the run folders and 52/52 from the episode bundles alone (docs/data/replay_agx_smREL073/README.md). Total 1,660 episodes, 0 differences. Q3 clause, reading (i): ntFIN5 replayed at its revision stands for the Q3 matrix; 720 of the 765 Q3 episodes flown from a clean commit also re-fly identical at it, compared on harness events and KPI (docs/data/replay_at_revision_20261118.md). Scope: the reproduction host is the runtime the episode flew on. On the desktop (AMD64, Python 3.10) replays differ in the last float digits: smREL073 42/52 identical (docs/data/kpi_smREL073/README.md); ntFIN5 352/654, with 1 run differing in a repair pass count (docs/data/kpi_ntFIN5/README.md). Headless episodes (the broad sweep, reading (c)) run no ROS 2 graph, so their bundles hold no episode.bag2/ and kpi.json says so (`<store>`/broad_REL072/runs/); they are replayed by re-flying them closed loop at their revision (tools/rerun_episode.py), as the Q3 episodes were. The 0.7.2 Gazebo and broad sweeps (functional, not KPI-grade) have no replay record of their own (docs/data/gazebo_sweep_20261128/, docs/data/broad_REL072/). The gps-dropout episodes of three earlier campaigns cannot be replayed, because their revisions did not log per tick the input the Shield dead-reckons on: ntFIN3 24 of 594, smfin1120 2 of 49, rel060 2 of 49 (docs/data/kpi_ntFIN3/README.md, docs/data/kpi_smfin1120/README.md). | Open loop only: a hil episode is not re-flown closed loop, since SITL and the network are not bit-reproducible. |
| WP4-12 | Six-field manifest per episode (code_revision, vla_model_hash, policy_hash, random_seed, sim_speedup, topology); sim_speedup=1.0 mandatory for KPI-bearing runs | DONE | Grant (02-implementation/stress-testing.md): "every episode emits a six-field manifest (...); sim_speedup=1.0 is mandatory for any KPI-bearing run in the HIL topology". The KPI-bearing runs: all 583 run folders of the nightly ntFIN2 (13 Nov) carry the six fields, code_revision 116214f3e6ee (clean), sim_speedup 1.0, topology hil, vla_model_hash the stub pilot's source hash (guardrail.vla_stub GotoPilot 454, ConstantPilot 126, StubVLA 3), random_seed the cell's seed (demo/out/hil_smoke/ntFIN2_*/manifest.json). Six-field manifest on every run (guardrail/manifest.py); the mission-style ROS 2 node only since 6 Nov (sitl/mavros_evidence.py, the GUIDED Shield node's evidence; mission_wp_20261106b: code_revision a3c36a51e28c on a clean tree, topology dev, sim_speedup 1.0 read from the autopilot; mission_wp_20261103d and mission_wp_20261104 have none); but 27-28 Oct manifests read code_revision '...-dirty' (PC, e.g. demo/out/nfz3b_pc_hier_dronecats_20261028) and 'unversioned' (Jetson, git-archive sync), and Jetson runs say topology 'projectairsim-single-host'; since 28 Oct the Jetson is a git checkout (deploy/orin/sync_follow_controller.ps1 ships a git bundle), so code_revision resolves there; a remote simulator is stamped projectairsim-remote-sim (guardrail/manifest.py) | None for the KPI-bearing runs. The section 6 follow comparisons (27-29 Oct, simple_flight, never KPI evidence) keep their '-dirty' / 'unversioned' code_revision and a topology outside dev / hil / flight (ARCH-31); the untracked file that made every run since 27 Oct '-dirty' was committed on 2 Nov (e9a08d6). The seed is recorded; the 60 GPS-noise / latency cells draw their noise from it on the companion (metrics.json perception_stress; sitl/ros2_shield_node.py PerceptionStress), and no other flown hil cell consumes it (section 5.7). |
| WP4-13 | Per-step record (observation, policy_state with policy_hash+generation, planner_output, post_repair, executed, outcome); auto-labels violation_type and risk_level; exporters (rosbag2 -> JSONL/Parquet), both episode-based and step-based | DONE | tools/export_episodes.py JSONL + Parquet with read-back check (CHANGELOG 2026-10-21); with a run's rosbag2 it reads the bag (`rosbags`) and action.executed is the setpoint MAVROS was sent (/mavros/setpoint_raw/local) in each tick's window, the flight log's verdict beside it, the join refused when the bag does not line up; since 2026-11-06 `rosbags` 0.10.11 and pyarrow are in vla-real, tests/test_wp4outputs_exporters.py 38/38 with no skip; on the KPI-grade hil run hil_smoke_03: 303 steps, executed from the bag on 303/303, JSONL and Parquet written and read back. Rails without ROS 2 (city, headless) keep the flight log's verdict, labelled a fallback on every step; the SITL rail has no camera, so its observation is the state topics (pose, IMU, GPS) and their references | - |
| WP4-14 | Every release runs, in Stress Testing's smoke set: high-speed near-edge, sudden hot-applied NFZ, time-window switch instant, three simultaneous violations, GPS noise/latency (Safety Shield page) | DONE | "Every release" is read as every release from 0.7.0 on, with the KPI-grade smoke on hil per release and the headless smoke per push (reading (c)). Each release flew its own hil smoke at its release commit, with all five rows flown and KPI-grade and P0 escape 0 on each: 0.7.0 smREL070 (21c2771, 23 Nov), 0.7.1 smREL071 (a3d616e, 26 Nov), 0.7.2 smREL072 (418b151, 27 Nov), 0.7.3 smREL073 (c44a360, 28 Nov: 52/52 KPI-grade, single_sha 52/52, a non-vacuous cell in every row and both at-speed near-edge cells non-vacuous, gate met; kpi_report_smREL073_2026-11-28.md:48-64) (docs/data/kpi_smREL070/ to kpi_smREL073/README.md; WP3-19). The gate is docs/RUNBOOK-release.md step 5, and CONTRIBUTING.md "Releases" says "A tag whose smoke has not flown is not a release". Per push: .github/workflows/tests.yml runs on every push (:715-716) with the step "Smoke profile (stress harness, headless)" (:1037), green in all 3 jobs on the 0.7.2 push (45969020953 master, 45969019967 tag; docs/data/github_ci/). On the 0.7.0 push (45156163225, 45156162082), and in one of three jobs on the 0.7.1 push (45762447982, 45762446649), an earlier test step failed and the smoke step was skipped (docs/data/github_ci/README.md; CHANGELOG 2026-11-25 (code: CI's pymavlink stand-in)). Local CI at each release's code passed the 64-episode headless smoke, rc 0 (docs/data/ci_local/a0eb7672d47e.json, 2dafaf096093.json, 5521c5f079a0.json, 16bb4168b391.json; each differs from its release commit only in CHANGELOG, VERSION and CI records; 16bb416: 194/194 files) (final CI: see docs/data/ci_local/). The headless smoke is functional evidence and never KPI-grade (experiments/profiles/smoke.yaml:15-19). Before 0.7.0: 0.6.0's smoke rel060 flew four of the five rows (the hil rail did not yet realise GPS noise; CHANGELOG 2026-11-19 (release smoke rel060)), and 0.1.0-0.5.1 predate experiments/profiles/smoke.yaml (not in tag v0.5.0). | - |
| WP4-15 | A scenario can be flagged for A/B; the harness runs it with Prefix on/off on the same seed and the KPI report shows the delta (Prefix Compiler page) | DONE | prefix on/off key in the harness (CHANGELOG 2026-10-07); since 7 Nov the A/B flown with a CSP-reading VLA: OpenVLA-7B on three seeds (each a recorded start, the same for both arms), repair rate off 0.200 / 0.127 / 0.108, on 0.392 / 0.207 / 0.437, mean delta +0.201; outcome off success 3/3, on fail 3/3 (unsafe_position_entered) (tools/score_csp_flights.py, docs/data/csp_flights_20261107.json) 16 Nov, the harness's own A/B flown on hil (campaign pab1, c7257f1, the hand-written CSP-consuming adapter guardrail/prefix_consumer.py, not a learned VLA): 33 KPI-grade missions, P0 escapes 0, fail-safe r2 15/15 on the labelled prefix-off arm, on vs off mission success 0.600 / 0.600, repairs per episode 6.6 / 130.5 (A/A largest pair 7), magnitude 1.78 / 9.38 m/s, in tools/kpi_report.py's A/B section (docs/data/kpi_prefix_ab_20261116). corridor-drift-out's prefix-on arm ended RTL 3/3: the adapter's 0.5 s samples stepped over a cut of the dog-leg's inner corner, fixed after the flight by a whole-path test (CHANGELOG 2026-11-16 evening). | Met by pab3 (19 Nov, the harness's prefix_ab profile: 15 same-seed on/off pairs on hil, the KPI report's A/B section with deltas and an A/A floor); the grant's sentence (prefix-compiler.md: "the harness runs it twice (Prefix on / off) with the same seed, and the KPI report shows the delta") does not require a learned pilot. pab_aerialvla_04 was flown outside the harness and its seed does not fix the CityLife traffic, so it supplements pab3; whether a learned VLA reading the CSP shows an effect belongs to WP4-19 (independent review 20 Nov). |
| WP4-16 | Project AirSim is the perception-stress rail; KPI numbers come only from KPI-grade (canonical HIL) stress runs | DONE | Project AirSim flown by ArduPilot (G1); its runs before the hil rig are not KPI-grade (KPI-grade needs the hil topology, section 2; WP3-14); on hil some read KPI-grade in their own kpi.json (pas_hil_aiot_kpi_01 below; ARCH-22), as learned-VLA flights outside the stub-pilot KPI campaigns (reading (h)); no ScenarioSpec drives it. 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). | One run per stressor (three for day), not a campaign through the suite's scenario format; non-KPI by design. |
| WP4-17 | Final delivery: 'KPI report on dynamic-scenario stress' (Grant overview delivery table, 2026-11-30); reference recommendation.md:34 says 'A KPI report from a nightly stress run' | DONE | Reading (j): ntFIN5's KPI report is the "KPI report on dynamic-scenario stress", with the prefix A/B from pab3. Render of record: docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md (+ .json). It is made by tools/kpi_report.py --campaign from a clean tree at f9cab68 over the nightly hil campaign ntFIN5 (release 0.7.0, 21c2771; 654 KPI-grade, single_sha 654/654), with the WP1 round trip and Prefix Compiler KPIs at c44a360 (release 0.7.3); its KPI figures are those of the 24 Nov render (docs/data/kpi_ntFIN5/README.md, "Re-rendered 27 Nov" and "Re-rendered 28 Nov"). It meets recommendation.md:34 and the report form of stress-testing.md:199-206: P0 escape 0 on 17,194 P0-violation ticks over 651 Shield-on flights (control 0.6500, null 0.1938), fail-safe (r2) 649/649 (Wilson 95 % low 0.994), tick_ms max 62.08 ms with 0 of 183,961 ticks over 100 ms (:17-19); one row per (template, parameter cell) (:974); top-10 failures, each linked to its episode bundle with raw and repaired action (:1778); Markdown and JSON; the Shield matrix gate met, with 12/12 near-edge-at-12mps cells non-vacuous (:48-64). Replayed at its revision on the AGX Orin, 654/654 identical (docs/data/replay_agx_ntFIN5/). Prefix A/B: pab3 (1239619, hil, 33 KPI-grade, the hand-written PrefixConsumer): 15 pairs, mission success 80.0 % with the prefix on and 60.0 % off, delta 0.200, P0 escapes 0/0 (docs/data/kpi_prefix_ab_20261119/kpi_report_pab3_2026-11-19.md:315-317). The report's per-paraphrase section reads n/m because the stub reads no text (:1896-1898); per-paraphrase robustness is WP4-09. Its "against the baseline" section matches episodes by run name, so it lists every run as new or gone; the per-family tables are the comparison (README). | The KPI report describes release 0.7.0's code; releases 0.7.1-0.7.3 are covered by their own KPI-grade smokes (smREL071-smREL073). |
| WP4-18 | Perception-stress scenarios on Project AirSim (Stress Testing page: 'Project AirSim for perception stress'; reference dual-rail.md:16 'Texture / lighting / shadow stressors', :70 lighting, weather, novel textures, dynamic actors) | DONE | dynamic actors (40 pedestrians, traffic, signals in CityLife) and, since 14-15 Nov, lighting (dusk, night), weather (rain, fog), shadow and texture stressors (below); experiments/templates/sensor_degradation.yaml headless 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md); 15 Nov, the texture and shadow stressors (demo/pas_ardupilot/perception_stress.py, CHANGELOG 2026-11-15 (texture and shadow stressors)): texture re-skins the level's 347 road meshes with the engine's WorldGridMaterial (World.set_object_material, every call's answer recorded); shadow turns the level's own sun, DirectionalLight_0, to 8 deg above the horizon and 60 deg round (World.set_object_pose, read back by the desktop and by the attaching node on the Orin), and every stressor refuses a sun that is not its own; flown with AIoT-VLA and the target estimator on the AGX Orin (pas_hil_aiot_texture_01, pas_hil_aiot_shadow_01, same flags as the unstressed pas_hil_aiot_est_01): detector hit rate 0.865 unstressed, 0.820 shadow, 0.095 texture; target lock 0.823, 0.775, 0.094; frame luma 96.4, 69.1, 101.8; the texture mission lost the car after about 100 s and ended target_lost; 0 P0 escape ticks in both (docs/data/pas_hil_stress_20261115/) | Flown with a per-scenario metric: day, dusk (-2 EV), night (-4 EV: no detection, the mission refused), rain (RAIN 0.8), fog (FOG 0.6: no red car found in 330 s, the mission refused and landed), shadow (the sun at 8 deg), texture (novel road surface); dynamic actors are the level's own (40 pedestrians, traffic, signals). One run per stressor, non-KPI by design; night is a darker camera (exposure), not lit streets (CityLife has no SunSky). |
| WP4-19 | Final demo gate (reference recommendation.md:33): all four components exercised by a non-trivial scenario from the Stress Testing scenario suite | DONE | Reading (k): the five final-demo scenarios of report section 9.1 (all flown on hil), plus pdyn1126 as the non-trivial scenario. pdyn1126 was flown on 26 Nov at release 0.7.1 (a3d616e) on hil (AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2), with experiments/sweep_scenarios.py --profile prefix_dyn --backend hil and --csp-topic on. In the suite cell dyn-nfz-spawn-ahead (dynamic_nfz_movement, a stationary zone spawned ahead at about 5 s) with the prefix on, pdyn1126_001 and _003 (seeds 1001, 1002) exercise all four components in one flight. Policy DSL: the signed policy bundle. Prefix Compiler: the new generation's CSP is compiled from /shield/csp and reaches the pilot 0.114 / 0.127 s after the Shield's generation. Safety Shield: the FSM goes Normal->Brake (X1) at 5.04 / 5.09 s and back to Normal (G7) at 7.25 / 7.22 s. Stress Testing: the suite scenario, its scorer and the six-name episode bundle (`<store>`/hil_campaigns/hil_pdyn1126/episode-*). Both runs pass, with mission success and P0 escape 0 (docs/data/kpi_prefix_dyn_20261126/rescore_r2_pdyn1126.json). Campaign: 12/12 KPI-grade, single_sha 12/12, 10 of 12 gates pass, P0 escape 0 on 67 P0-violation ticks, tick_ms max 52.14 ms, 0 over 100 ms (docs/data/kpi_prefix_dyn_20261126/kpi_report_pdyn1126_2026-11-27.md). Replayed at a3d616e on the AGX Orin: 12/12 identical (docs/data/replay_agx_rest_20261128/README.md). Scope: the scenario's zone event is applied by the Shield node, not sent over REST. REST hot-apply in flight is shown in hh02_001 (demo row 1) and in wgs01_001: POST /hot_apply at 3.016 s and PATCH at 8.000 s over mutual-TLS REST, generations 1 and 2 applied (CHANGELOG 2026-11-26). The corridor-swap-narrowed prefix-on arm failed its gate in both runs, pdyn1126_005 and _007: under FSM Brake the aircraft's momentum carried it 0.81 / 0.64 m outside the narrowed corridor before the Shield flew it back (docs/data/kpi_prefix_dyn_20261126/README.md). | The CSP is read by the hand-written prefix consumer (guardrail/prefix_consumer.py around the stub), not by a learned VLA. |
| WP4-20 | Shield audit JSONL aggregated into the episode bundle by the harness at episode end (Safety Shield page, Audit log section) | DONE | guardrail/replay.py EPISODE_FILES audit.jsonl (shield_audit.jsonl since 2 Nov) packed per episode (CHANGELOG 2026-10-07); 30 Oct: the headless episode keeps the flying Shield's audit records (EpisodeRun.audit, AuditLogger.in_memory) and write_bundle packs them as shield_audit.jsonl at episode end, the sweep's and the gRPC EndEpisode's bundles alike (tests/test_sweep.py::test_the_episode_bundle_carries_the_shields_audit) | None. |
| WP4-21 | WP4 status as reported to the PI/ITRI versus the grant's WP4 outputs (scenario library, KPI rollups, replay harness; 7-row output table on the Stress Testing page) | DONE | WP4-built claim corrected (tools/check_claims.py, claim WP4-21 '2026-10-06: WP4 built'; row A8 of the 6 Oct correction note, an internal record gitignored as docs/CORRECTION-*.md, so not in a clone); the correction was sent to ITRI (CHANGELOG 2026-11-02) | - |

### Architecture and topologies

31 items: 29 DONE, 2 PARTIAL.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| ARCH-01 | VLA emits a 10 Hz setpoint; ArduPilot inner loop tracks it (canonical rail) | DONE | 10 Hz setpoints tracked by ArduPilot on SITL+MAVROS (7 Oct) and PAS+ArduPilot (loop 9.91 Hz on 21 Oct) 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the feed between ticks (--vla-feed stream; guardrail/vla_backends.py RemoteVLABackend.pump; docs/data/pas_hil_feed_20261115/flights.md): a new camera frame goes to the model without waiting for the 10 Hz tick, the answers still enter on ticks. Four KPI-grade hil flights of AIoT-VLA v10b on the AGX Orin with the feed uncapped and the camera at a 0.05 s capture interval (pas_hil_feed_05-08, d9f5ea0 / deac4b3): 12.39-13.23 distinct frames/s reached the model while it was asked (9.58 before, pas_hil_obs_02); over the whole mission 10.72 and 11.07 in the two flights that held the car for the full 420 s (pas_hil_feed_07 / _08), 3.48 and 8.86 in the two that lost it; decisions parsed 10.62 and 10.93 per second over those two missions; Shield ticks 9.96-9.99 Hz, 39-332 interventions per mission, 0 P0 escapes; the detector 2.31-2.70 Hz and the tick's work as without the feed. | Two full-length flights (pas_hil_feed_07 / _08). The model decides at 10 Hz or faster only while the detector has the target (12.4-13.2 decisions/s asked; 3.5 and 8.8 over the mission in the two flights that lost the car). An answer enters on the next tick, and an older one taken in the same tick is superseded unflown. |
| ARCH-02 | ArduPilot is the inner loop on every rail, including the perception (Project AirSim) rail | DONE | Every rail of the grant flies ArduPilot as the inner loop. KPI rail: ArduCopter 4.5.7 SITL + MAVROS 2 on `hil` (the KPI-grade nightly ntFIN2 + ntFIN2s, 589 episodes, 12-13 Nov). Functional rail: Gazebo Harmonic on ArduCopter JSON SITL, flown end to end 2 Nov over pymavlink (e74eae3) and over MAVROS 2 (0207af0) (ARCH-16). Perception rail: Project AirSim on ArduPilot SITL (G1 8 and 13 Oct; PAS+ArduPilot rail flights to 21 Oct; 29 Oct AerialVLA flew perception_node --pilot vla over pymavlink, pas_vla_goto_01/02, pas_vla_approach_01, every tick through the Shield and the FSM, 0 P0 escapes) and, since 14 Nov, a learned VLA (AIoT-VLA) on `hil` over MAVROS 2, KPI-grade, 0 P0 escapes (docs/data/pas_aiot_hil_20261114/flights.md, docs/data/pas_hil_paraphrase_20261115/flights.md, docs/data/pas_hil_feed_20261115/flights.md). Grant (03-simulation/dual-rail.md): "Both rails point at one SITL instance"; (01-context/grant-overview.md): "ArduPilot runs the inner loop at 100–400 Hz". Re-read against those lines on 16 Nov by a reader and an adversarial second reader: the same evidence already carries ARCH-18, ARCH-20 and ARCH-21 | Nothing on the grant's rails. The city follow comparisons (27-29 Oct, 4 Nov) and the 7 Nov CSP A/B flew Project AirSim's simple_flight: a comparison bench outside the delivered architecture, labelled so (section 6) and never KPI evidence. The default-backend set (ARCH-30) is the AGX Orin timing and the C2 flights on hil; the simple_flight comparisons are not folded into it. |
| ARCH-03 | Real VLA backends produce the 4-D action at 10 Hz (and Orin latency gate G3) | DONE | AerialVLA bf16 p50 259-486 ms on the 4090 (docs/data/aerialvla_serving_4090.json); in flight decisions every 0.4-0.5 s, DroneCATS 1.0-1.45 s, held as a 10 Hz stream (guardrail/vla_link.py StepTracker); 7B does not fit the Orin Nano; DroneCATS 2B int4 on the Orin Nano: one decision p50 9.57 s (28 Oct, CHANGELOG 2026-10-28 (night)); 30 Oct: the lab's own AIOT-VLA (training/aiotvla, docs/DESIGN-aiotvla.md; named NTUT-VLA until 30 Oct) answers in 48.4 ms p50 on the Orin Nano Super (19.1 decisions/s over 600 replayed captures, the server alone) and 10.3 ms on the RTX 4090; on the city rail its requests go every 0.1 s (capture gap p50 0.13 s) 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the feed between ticks (--vla-feed stream; guardrail/vla_backends.py RemoteVLABackend.pump; docs/data/pas_hil_feed_20261115/flights.md): a new camera frame goes to the model without waiting for the 10 Hz tick, the answers still enter on ticks. Four KPI-grade hil flights of AIoT-VLA v10b on the AGX Orin with the feed uncapped and the camera at a 0.05 s capture interval (pas_hil_feed_05-08, d9f5ea0 / deac4b3): 12.39-13.23 distinct frames/s reached the model while it was asked (9.58 before, pas_hil_obs_02); over the whole mission 10.72 and 11.07 in the two flights that held the car for the full 420 s (pas_hil_feed_07 / _08), 3.48 and 8.86 in the two that lost it; decisions parsed 10.62 and 10.93 per second over those two missions; Shield ticks 9.96-9.99 Hz, 39-332 interventions per mission, 0 P0 escapes; the detector 2.31-2.70 Hz and the tick's work as without the feed. 17 Nov, gate G3 judged from the stored hil flights (tools/g3_gate.py --runs F:/vla_drone_offload/jetson_runs --write docs/data/g3_gate_20261117): AIoT-VLA v10b, the lab's small backend, with its server on the AGX Orin (metrics.json hil_evidence.vla_host) on three C2 hil flights (pas_hil_feed_07, pas_hil_feed_08, pas_hil_aiot_c2_01_20261116): round trip p95 84.7 / 84.2 / 84.0 ms (84.5 ms pooled over 10716 answers), 13.15-13.22 decisions/s while asked, 0 failed requests - inside the gate's 100 ms p95 and 10 Hz; the 7B backends measured fail it (OpenVLA-7B bf16 969 ms p95 on the AGX Orin at MAXN; AerialVLA and RaceVLA bf16 1.4-2.4 s round trip in flight), and the gate's action, the swap to a smaller backend, is the backend that passes. | int8 7B is not measured on the Orin (the gate's premise names int8); the 10 Hz rate needs the stream feed with a 0.05 s capture (tick-feed flights decided 6.3-9.6 times a second while asked); frame to answer p95 is 134-135 ms (not the gate's measure); AIoT-VLA is follow-only and is not one of the grant's named swap examples (BitVLA, TinyVLA). |
| ARCH-04 | 4-D action (vx, vy, vz, yaw_rate) in body frame; body-to-NED conversion lives in the MAVLink adapter | DONE | body frame through the Shield's own boundary on the ROS 2 rail (`Shield.filter_body`, 4 Nov), NED conversion only in sitl/mavlink_adapter_node.py; 18 Nov: the four dev rails hand over the body frame where that is exact to the bit (see WP3-04) | Met (independent review 20 Nov): the Shield's input contract is the body frame on every flying rail and loop (Shield.filter_body / frames.filter_world_as_body since 19 Nov), and the conversion to local NED is the one function frames.body_to_local_ned, never in the operators: on the grant's ROS 2 rail it is called only in the MAVLink adapter, and the simulator demo loops call it in their own loop after the Shield (the design the project applies; see WP3-04); flown at measured headings on hil (pab3, pab_aerialvla_04) and on the pymavlink SITL rail at 135.6 deg (20 Nov, see WP3-04). The operator-frame clause belongs to WP3-04's wording. |
| ARCH-05 | VLA->Shield on Float32MultiArray; Shield->MAVROS on mavros_msgs/PositionTarget; MAVROS->ArduPilot over UDP 14550 | DONE | PositionTarget via adapter; MAVROS on UDP 14555 behind mavlink-router (14550 is Mission Planner's) | None. |
| ARCH-06 | Switchable backend: CognitiveDrone, OpenVLA generic, BitVLA, in-house stubs, all conforming to 4-D | DONE | One slot, the VLABackend protocol (`.act(state) -> Action4D`, guardrail/vla_backends.py): StubBackend is the in-house stub, PolicyBackend an in-house trained policy, and RemoteVLABackend takes a model served over protocol v2. OpenVLA generic: base OpenVLA-7B (bf16) in the slot through OpenVLABackend (demo/real_vla_demo.py:953) flew 14 times behind the Shield on 7 Nov, P0 escapes 0 (AirSimNH, not KPI-grade; docs/data/csp_flights_20261107.json). Flown on hil through the perception node, 3 C2 flights each, P0 escapes 0: AerialVLA (LoRA on OpenVLA-7B, bf16), DroneCATS-Cosmos3-Edge-2B, RaceVLA drone6_all (deploy/evidence/vla_backend_table.md, C2 table). CognitiveDrone and BitVLA: adapters in demo/vla_server_foreign.py (--model cognitivedrone / bitvla); scripted 4-D conformance 500/500 finite Action4D for each of cognitivedrone_4d, cognitivedrone_7d and bitvla (tests/test_vla_backends_cd_bitvla.py, seed 20261117); BitVLA's real LIBERO-Spatial arm checkpoint loaded on the AGX Orin on 22 Nov: 5/5 frames to finite Action4D, Shield.filter took all 15 frame/state cases, 188 finite ticks over TCP at 0.60 Hz (docs/data/bitvla_loader_20261122/README.md). CognitiveDrone's authors publish no weights and BitVLA's only published checkpoints are for robot arms (re-checked 27 Nov; deploy/evidence/vla_backend_table.md:358, :362); the project reads the two adapters with their 4-D conformance checks as meeting "switchable backend". | Neither CognitiveDrone nor BitVLA has flown (no drone weights are published for either); the 4-D slot flies a backward vx as 0 and caps the turn at 1.1 rad per horizon. |
| ARCH-07 | MAVROS 2 is the canonical bridge for Year 1 | DONE | MAVROS 2 runs on the dev rail (7 Oct); tests/test_manifest.py 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). | The KPI campaigns' pilot is the stub; the learned VLA's flights over MAVROS 2 are the perception rail's six missions of 14 Nov. |
| ARCH-08 | AP_DDS migration (Q3, optional) | PARTIAL | The project closes this optional item at stage 1 and reports it PARTIAL. Stage 1 (docs/data/ap_dds_20261126/README.md; docs/RUNBOOK-ap-dds.md steps 1-6): sitl/ap_dds/setup_ap_dds.sh builds ArduCopter Copter-4.5.7 with --enable-dds, and sitl/ap_dds/start_ap_dds_sitl.sh runs it with the micro-ROS agent (12 /ap/* topics). Step 4 (ap_dds_verify_01): GUIDED and arm over /ap/mode_switch and /ap/arm_motors, 1.0 m/s east on /ap/cmd_vel measured 1.018 m/s (velocity_error 0.02 m/s, bearing error 0.1 deg); step 5 (ap_dds_verify_02): the same through sitl/ap_dds_adapter_node.py, velocity_error 0.016 m/s. Step 6 (ap_dds_stage1_02, clean revision 21493e67f945, signed bundle fase3-sim-demo v0.1.4): sitl/ros2_shield_node.py -> /shield/setpoint -> ap_dds_adapter_node -> /ap/cmd_vel, no MAVLink setpoint adapter running; 113/113 ticks, target reached at 11.3 s, P0 escapes 0, delivery 113 published / 113 reports, no gap; in cruise the autopilot's velocity tracked the command to a median 0.049 m/s, p95 0.101 m/s (4.030 m/s measured against 4.00 m/s commanded; follow_check.md). Since release 0.7.3 (c44a360) report.md's Path row and metrics.json setpoint_path name the adapter seen subscribed to /shield/setpoint (CHANGELOG.md [0.7.3]); the stage-1 report.md files predate this and keep the old fixed MAVLink row (README). Scope of this evidence: dev topology (desktop WSL), not KPI-grade; one clean route flight with no zone engaged; mode requests went over MAVROS (events.jsonl); the DDS SITL read FENCE_ENABLE 0 (metrics.json autopilot_fence). The KPI rail stays on MAVROS 2 (smREL073: setpoint path through mavlink_adapter in 52/52, docs/data/kpi_smREL073/README.md). | Stage 2 is not delivered: every Shield input (pose, velocity, fix, mode, armed, landed, home, params), mode changes, arming, take-off and the GeoFence stay on MAVROS 2; stage 2 is not possible on ArduCopter 4.5.7, whose DDS client publishes no mode/armed/landed state and has no take-off, parameter or fence service (docs/RUNBOOK-ap-dds.md coverage table; the grant's "uneven coverage"). |
| ARCH-09 | ArduPilot GeoFence/FENCE_ACTION as ultimate backstop; MAVLink escalation to RTL/Land/Loiter | DONE | GeoFence backstop configured and shown firing; FSM drives RTL and Land via SET_MODE; its Loiter was sent as BRAKE until 7 Nov and is held in GUIDED by the companion since (see WP3-07, decision 7 and reading (e); WP3-10) | None. |
| ARCH-10 | Mission Planner via mavlink-router fan-out, parallel to MAVROS | DONE | mavlink-router fans out to UDP 14550, 19.0 Hz measured by sitl/gcs_listen.py (CHANGELOG 2026-10-08); 15 Nov, recorded on a hil flight: Mission Planner 1.3.83 on the desktop connected by itself to the router's UDP 14550 fan-out (its AutoConnect entry; title bar 'ArduCopter V4.5.7 (2a3dc4b7) on UDP14550-1-QUADROTOR') while MAVROS 2 on the AGX Orin flew pas_hil_para_11 (Project AirSim + ArduPilot SITL, AIoT-VLA and the Shield on the Orin); screenshots before arming and in flight (Guided, 7.99 m, 2.81 m/s): docs/data/arch10_20261115/ (CHANGELOG 2026-11-15 (Mission Planner; paraphrase retries)) | Screenshots, not a recording; Mission Planner only watched (no command was sent from it). |
| ARCH-11 | dev topology: everything on one desktop | DONE | dev flown in containers 2026-11-06: deploy/compose/docker-compose.dev.yml with the images built on the desktop (vlaguard/sitl:copter-4.5.7 ArduCopter 4.5.7, vlaguard/companion:jazzy: router, MAVROS 2, mission), dev_compose_01 (code 30667cc clean, topology dev, P0 escape 0.0, not KPI evidence: is_kpi_grade (guardrail/manifest.py) counts a dev run only under a written waiver, and no dev waiver is used (decision 3); deploy/evidence/hil/images.txt); the native WSL dev rail since 7 Oct; the Gazebo Harmonic functional rail flown end-to-end on 2026-11-02 over pymavlink and MAVROS 2 (its Shield-on mission is ARCH-16's item) | - |
| ARCH-12 | hil topology: desktop runs sims + MAVROS + GCS, Jetson Orin runs VLA + Shield (canonical KPI config) | DONE | hil flown 2026-11-06: SITL + mavlink-router on the desktop, MAVROS 2 + the Shield + the stub pilot on the AGX Orin per docs/RUNBOOK-orin-hil.md (companion image built on the Orin, hil.env with the cable's addresses); hil_smoke_03 KPI-grade (CHANGELOG 2026-11-06 (hil)); the VLA on the Orin is ARCH-20's item; the KPI campaign is WP4-01's | - |
| ARCH-13 | Topology field and the claim about which configuration yields contractual KPIs | DONE | labels dev/hil/flight, hil and flight only on evidence (CHANGELOG 2026-10-06/-07); 2026-11-06 the first run labelled hil from evidence (hil_smoke_03), accepted by is_kpi_grade and by tools/kpi_report.py (1 of 2 KPI-grade; hil_smoke_02 hil but refused: unsigned bundle); the 28 Oct Orin Nano manifests stay single-host/unversioned and are counted as functional evidence; tools/check_claims.py | - |
| ARCH-14 | flight topology: Orin companion with real ArduPilot autopilot; final demos | DONE | The flight stage is covered by a written waiver: the hil rig and a bench test of the real flight controller stand in for it; no airframe flight is claimed. hil (AGX Orin companion, ArduCopter 4.5.7 SITL over MAVROS 2): smREL073 at release 0.7.3 flew 52 KPI-grade missions, P0 escape 0 (docs/data/kpi_smREL073/README.md); the five final-demo scenarios were flown on hil (docs/FINAL-REPORT.md section 9.1). Bench, 17 Nov (docs/data/fc_bench_20261117/README.md): CUAV V5 Nano, ArduCopter 4.7.1, never armed; FENCE_* set and read back, the nfz-square zone uploaded as 4 exclusion items and read back matching (gcs_backstop.json); GUIDED, BRAKE, LOITER, RTL, LAND and STABILIZE accepted, each confirmed by the next heartbeat; restored except boot-time values and MAV1_* stream rates (fc_verify.json). hil to flight is a configuration change on the same images (deploy/topologies/flight.env, deploy/compose/docker-compose.orin.flight.yml; tests/test_deploy_configs.py); the grant makes flight "No — qualitative validation" (03-simulation/topologies.md), and guardrail/manifest.py is_kpi_grade never counts a flight-topology run. | No airframe flight; the Orin companion has not driven the real FC, which runs ArduCopter 4.7.1 against the rig's 4.5.7 (ARCH-31's stated limit). |
| ARCH-15 | HIL bridge adapter (Q1, CRITICAL) and its 2026-06-30 fallback gate G1 | DONE | The Project AirSim - ArduPilot bridge closes the loop: Project AirSim's ArduCopterApi JSON sensor frames go over UDP 9003 to ArduPilot's SIM_AirSim backend (--model airsim-copter) and PWM comes back over UDP 9002 (demo/pas_ardupilot/rail.py, scripts/run_pas_ardupilot.ps1). Gate G1's check, demo/pas_ardupilot/g1_check.py, passed all four conditions on 2026-10-08 and again on 2026-10-13, three runs each at seed 20261006 (demo/out/g1_20261008_223259/ and demo/out/g1_20261013_203404/, g1_verdict.json): EKF p95 3-D error 0.814-0.827 m against 1 m, landing 0.055-0.18 m from take-off against 2 m, 0 airborne collisions, 0 fence breaches. The perception rail is kept on Project AirSim and carries the learned-VLA hil follows (deploy/evidence/vla_backend_table.md, 'C2 on hil: the Project AirSim follow'). The delivered scope is the recorded deviation of section 7.1: the bridge carries SIM_AirSim JSON frames, not HIL_GPS/HIL_SENSOR, which ArduCopter 4.5.7 cannot take as the simulator's sensor feed (no HIL_SENSOR handler; HIL_GPS read only as an external MAVLink GPS; docs/FINAL-REPORT.md section 7.1). | The transport is SIM_AirSim JSON frames, not the HIL_GPS/HIL_SENSOR messages G1's condition 1 names (recorded deviation, section 7.1). |
| ARCH-16 | Functional rail (Gazebo Harmonic) demoable end-to-end at mid-term | DONE | flown end-to-end 2026-11-02 at e74eae3 (WSL Ubuntu 24.04, Gazebo Harmonic, ArduCopter 4.5.7 JSON SITL, pymavlink; the pymavlink run folders record no Gazebo version, and the CHANGELOG names Gazebo Sim 8.14, the WSL system package; the MAVROS 2 runs' physics.json records gz_sim 8.11.0, the ROS 2 Jazzy vendor build): demo/out/gazebo_shield_on_20261102, _off_20261102, _on_dynamic_20261102 (Shield on P0 escape 0.0, off 0.584), and over MAVROS 2 in the dev topology at 0207af0 (PHYSICS=gazebo sitl/run_ros2_demo.sh): gazebo_ros2_shield_on_20261102 P0 0.0, gazebo_ros2_shield_off_20261102 P0 1.0 with the GeoFence firing; each with a flight-log replay video (tools/render_track_video.py) (CHANGELOG 2026-11-02 evening) The grant's mid-term demo gate (kuanting-vla-uav-guardrail/docs/05-recommendation/recommendation.md, "Mid-term (2026-07-20) demo gate", steps 1-4) was flown on the Gazebo rail on 25 Nov at 75a6d45 (each run's manifest records 75a6d45be758; profile option rail_signed_bundle: true). Record: docs/data/gazebo_midterm_20261125/, with README.md, gate_check.json, sweep.json and kpi_report_sweep.md. Scenario: midterm-demo-graze (experiments/templates/static_nfz.yaml), flown by profile experiments/profiles/gazebo_midterm.yaml, seeds 1001-1003, 3/3 pass. Rail: physics.json in each run records gazebo-harmonic gz_sim 8.11.0 with the ardupilot_gazebo iris_runway.sdf world. SITL is ArduCopter 4.5.7 JSON over MAVROS 2 with the ROS 2 Shield node, dev topology, sim_speedup 1.0. Step 1 (signed bundle): bundles/fase3-sim-demo-v0.1.4.tar.gz, loaded with its expected hash pinned. kpi.json policy_source reads signature "verified": Ed25519 by lab-dev:ed25519:6ec75b535ef5c533. The bundle holds polygon NFZ nfz-square (P0) and the envelope rules alt-band and kin-caps. Step 2 (task that wants to enter the NFZ): the task is "fly to the northeast pad at 6 m/s". At tick 1 the raw command is (5.88, 1.18) m/s world. The audit records nfz-square "predicted pos (15.3,6.1) inside NFZ at t+2.6s". Step 3 (interception and lateral projection): the tick-1 G1 reason is "nfz-square (hard P0 project_fix) repaired, lat 0.91 m vs theta 2.00 m". GeofenceSlide/LateralProjection fired on 23/22/23 ticks, a maximum slide of 0.917/0.913/0.919 m, with none braked. The action emitted at tick 1 is (0.63, -1.10) m/s. Time inside the NFZ was 0.0 s, P0 escape 0, target reached, outcome success. Step 4 (policy_hash in the audit): every shield_audit.jsonl record (137/136/137) carries sha256:b665adba...e7f358. That hash equals manifest.json policy_hash and the kpi.json loaded and expected hash. Run folders: demo/out/gzsweep_midterm-demo-graze--seed100{1,2,3}/, holding shield_audit.jsonl, kpi.json, manifest.json, prompt.yaml, physics.json, the rosbag2, the signed replay bundle and a flight-log replay video <run>_track.mp4 (tools/render_track_video.py). Episode bundles are in demo/out/stress_gazebo_midterm/. | The rail was flown end to end on 2 Nov 2026, and the four-step demo gate on 25 Nov 2026. Where the gate record lives The record is docs/data/gazebo_midterm_20261125/ (README.md, gate_check.json). The run folders under demo/out/ are gitignored and kept on the lab disk. The signed bundle file under bundles/ is not committed. How the gate was flown The pilot is the rails' StubVLA, which the grant's dev topology allows, and it reads no text. The Shield node parses the task text, which names the northeast pad at (30, 30). The scenario's target moves the pad to (45, 12), so the flown path is set by the scenario, not by the text. The bundle is signed with a lab development key and has no certificate chain: the gate flew on 25 Nov, before the Lab CA was applied on 27 Nov (WP1-04). The bundle has two envelope rules (alt-band, kin-caps); the gate asks for one. The FSM enters Brake on tick 1 and stays there for the rest of the mission, while the repaired pilot action keeps being flown. kin-caps clamps 6 m/s to 4 m/s on every tick. The videos are flight-log replay renders, not screen captures of Gazebo. All Gazebo runs are functional_only, not KPI-grade. Earlier flights on this rail In the 2 Nov flights with the Shield on, the projection was abandoned (repair 8.14 m against theta 2 m) and the FSM escalated to Loiter and then RTL, so the target was not reached. WP3-15 addresses this false trigger. 20 Nov functional sweep at 28a594d: 8 missions on one seed, 7 passed (docs/data/gazebo_sweep_20261120/). 24-25 Nov gazebo_functional profile at release 0.7.0: 132 missions on 3 seeds, P0 escape 0. After re-flying the 6 missions that never armed, the gates read 107 pass / 16 fail / 6 known / 3 tracked; sweep.json before the re-flights reads 104 / 19 / 6 / 3. Every failed gate fails the same way on hil (WP4-04, docs/data/gazebo_sweep_20261124/). |
| ARCH-17 | Mid-term delivery 2026-07-20 (DSL+IR, Shield ROS 2 node with one projection operator, Gazebo rail) | DONE | the three parts are delivered: the Policy DSL + IR (WP1-21), the ROS 2 Shield node with its projection operators (WP3-01) and the Gazebo rail flown end to end (ARCH-16); the mid-term report docs/MIDTERM-REPORT-Aug2026.md, delivered 20 Aug 2026 as the 19 August review asked (docs/MEETING-PACK-Sept2026.md Part A1, item 1) | None. |
| ARCH-18 | Perception-rail integration (final delivery 2026-11-30) | DONE | PAS+ArduPilot perception node flown with detector pilot + Shield + FSM (8-21 Oct); start gate and --pilot vla built 22-23 Oct, 'none flown'; --link mavros first flown 29 Oct on dev (pas_dev_mavros_01/02, CHANGELOG 2026-10-29) and on hil since 14 Nov (below); detector boxed the wrong object on the close flights; 29 Oct: AerialVLA flew perception_node --pilot vla on ArduPilot SITL over pymavlink (pas_vla_goto_01/02, pas_vla_approach_01), every tick through the Shield and the FSM, 0 P0 escapes 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the target estimator (pas_hil_aiot_est_01 at 3918965: --identity --negative-query 'a traffic light' --target-estimator, demo/subject_track.py): KPI-grade, 420 s, 0 P0 escape ticks, 266 interventions; the estimate within 6 m of the car on 0.907 of ticks (0.971 of its own), 5 lapses and 5 re-acquisitions; aircraft-to-car gap median 34.4 m, p90 42.6 m against 112.9 / 166.1 m without it (pas_hil_aiot_kpi_01). | One KPI-grade mission each without and with the target estimator; the estimator is opt-in (--target-estimator) and its gain comes from the strict lock (AIoT-VLA does not read the compass phrase). |
| ARCH-19 | Final integration + KPI runs (Q4) in the canonical hil topology; KPI report on dynamic-scenario stress | DONE | The canonical hil topology is read as the KPI campaigns flown with the stub pilot: the stub pilot, the Shield and MAVROS 2 on the AGX Orin, ArduCopter 4.5.7 SITL on the desktop, with the harness and the signed bundle (docs/RUNBOOK-orin-hil.md). The learned VLA is shown in separate hil flights (AIoT-VLA, docs/data/pas_aiot_hil_20261114/; ARCH-20). The policy-artefact KPIs are measured by the offline tools at the delivered revision (reading (h)). KPI nightly of record: ntFIN5 at release 0.7.0 (21c2771), 654 KPI-grade, P0 escape 0 on 17,194 P0-violation ticks, fail-safe (r2) 649/649 (Wilson 95 % low 0.994), 0 of 183,961 ticks over 100 ms, single_sha 654/654 (docs/data/kpi_ntFIN5/README.md). Its KPI report, with the prefix A/B from pab3 (docs/data/kpi_prefix_ab_20261119/), is the KPI report on dynamic-scenario stress (reading (j); WP4-17). It was rendered again on 28 Nov with the WP1/WP2 records at c44a360: KPI figures unchanged, Shield-matrix gate met (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md). Every later release flew the KPI-grade hil smoke (reading (c)). smREL071 (0.7.1) and smREL072 (0.7.2) had 52 KPI-grade each (docs/data/kpi_smREL071/README.md, docs/data/kpi_smREL072/README.md). The delivered release 0.7.3 (c44a360) flew smREL073: 52 KPI-grade, P0 escape 0 on 2393 P0-violation ticks, (r2) 50/50, tick_ms max 55.73 ms (0 of 16,047 over 100 ms), single_sha 52/52, Lab CA chain 52/52, release gate met (docs/data/kpi_smREL073/README.md). Offline at c44a360: WP1 round trip 33/33, negative corpus 39/39, cold start p95 477 ms (docs/data/wp1_roundtrip_c44a360.json); CSP P0 coverage 124/124 over 33 policies at every budget from 256 to 144 tokens (docs/data/csp_kpis.json). All four components flew on dynamic suite cells in pdyn1126 (0.7.1, 12 KPI-grade, 10 of 12 gates pass; docs/data/kpi_prefix_dyn_20261126/README.md; reading (k), WP4-19). Replay: every KPI-grade hil run flown from 22 Nov on, 1,660 episodes, is identical at its own revision on the Orin (docs/data/replay_agx_rest_20261128/README.md, with docs/data/replay_agx_ntFIN5/ and docs/data/replay_agx_smREL073/; reading (i), WP4-11). | The KPI campaigns fly the stub pilot, and the learned VLA flies in separate hil flights. The nightly of record is at release 0.7.0; releases 0.7.1 to 0.7.3 each flew the 52-mission hil smoke. |
| ARCH-20 | A real VLA constrained end-to-end in the canonical KPI topology | DONE | VLA slots built on both ArduPilot rails: perception_node.py --pilot vla, sitl/ros2_vla_bridge_node.py (CHANGELOG 2026-10-23 later, 'none flown'); VLAs flown only on simple_flight until 29 Oct; 29 Oct: AerialVLA flew perception_node --pilot vla on ArduPilot SITL over pymavlink (pas_vla_goto_01/02, pas_vla_approach_01), every tick through the Shield and the FSM, 0 P0 escapes 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the target estimator (pas_hil_aiot_est_01 at 3918965: --identity --negative-query 'a traffic light' --target-estimator, demo/subject_track.py): KPI-grade, 420 s, 0 P0 escape ticks, 266 interventions; the estimate within 6 m of the car on 0.907 of ticks (0.971 of its own), 5 lapses and 5 re-acquisitions; aircraft-to-car gap median 34.4 m, p90 42.6 m against 112.9 / 166.1 m without it (pas_hil_aiot_kpi_01). | KPI-grade learned-VLA hil missions include pas_hil_aiot_kpi_01 (420 s, 119 interventions, P0 0; target lock 0.06), pas_hil_aiot_est_01 (420 s, 0 P0 escape ticks) and, with the stream feed, pas_hil_feed_05-08 (decisions parsed 10.62 and 10.93 per second over the two full missions, pas_hil_feed_07 / _08; ARCH-01); gate G3 on pas_hil_feed_07, _08 and pas_hil_aiot_c2_01: 13.15-13.22 decisions/s while asked (ARCH-03). Follow quality on this rail is weak. |
| ARCH-21 | Constrained-VLA evidence on the perception rail and in current demos | DONE | VLA-piloted flights behind the Shield with the Shield acting: vla_follow_redcar_01_20261023 (177 corrected ticks), nfz3b_pc_hier_dronecats / nfz3b_orin_hier_aerovla 28 Oct (13 / 257 repairs, 0 zone ticks, 0 P0 escapes) 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the feed between ticks (--vla-feed stream; guardrail/vla_backends.py RemoteVLABackend.pump; docs/data/pas_hil_feed_20261115/flights.md): a new camera frame goes to the model without waiting for the 10 Hz tick, the answers still enter on ticks. Four KPI-grade hil flights of AIoT-VLA v10b on the AGX Orin with the feed uncapped and the camera at a 0.05 s capture interval (pas_hil_feed_05-08, d9f5ea0 / deac4b3): 12.39-13.23 distinct frames/s reached the model while it was asked (9.58 before, pas_hil_obs_02); over the whole mission 10.72 and 11.07 in the two flights that held the car for the full 420 s (pas_hil_feed_07 / _08), 3.48 and 8.86 in the two that lost it; decisions parsed 10.62 and 10.93 per second over those two missions; Shield ticks 9.96-9.99 Hz, 39-332 interventions per mission, 0 P0 escapes; the detector 2.31-2.70 Hz and the tick's work as without the feed. | n = 2 full-length flights with the model deciding at 10 Hz or faster (ARCH-01's note); the follow on this rail still depends on the detector holding the car (2 of the 4 uncapped flights lost it). |
| ARCH-22 | rosbag2 capture for the determinism and replay contract (L3) | PARTIAL | rosbag2 (MCAP) is recorded on the ROS 2 rails (sitl/run_ros2_demo.sh, deploy/docker/entrypoint.sh, sitl/bag_qos_overrides.yaml). Every episode of the hil KPI campaigns of record holds one (ntFIN5 654, smREL073 52). Example, smREL073_001 (<store>/hil_runs/smREL073_001_nfz-head-on/bag/metadata.yaml), with 24 topics: MAVROS state, pose, velocity, IMU, GPS, battery, statustext and setpoints; raw MAVLink both ways (/uas1/mavlink_source, /uas1/mavlink_sink); /vla/action_4d and /vla/identity; /shield/setpoint, /shield/intercept, /shield/fences and /shield/mode_request; /mavlink_adapter/sent and status; and the latched /guardrail/manifest. Release 0.7.3 (c44a360) publishes the manifest start frame, with all six fields, when the autopilot accepts the arm, and writes episode.bag2 trimmed to begin at that frame (guardrail/episode_bag.py, tests/test_episode_bag.py). tools/replay_from_bag.py compares the bag's frames with manifest.json. smREL073 (docs/data/kpi_smREL073/README.md): start frame at arming with six fields 52/52; bag start frame identical to manifest.json 52/52; episode.bag2 begins with the frame in 49/52 bundles. The other 3 keep the recorder's bag, because one /vla/action_4d message lay between the arm acknowledgement and the frame (reason recorded in kpi.json). Replay from the bag over the nightly of record ntFIN5 (docs/data/replay_from_bag_ntFIN5/README.md): manifest frames 654/654 and trajectory 654/654 identical; ticks and KPI 630/654 (the 24 gps-noise-beyond-margin runs differ by one inferred 'who flew it' tick). 18 of 61 KPI fields are compared from the bag. The other 43 (P0 escape and fail-safe correctness among them) cannot be derived from it, and the episode bundle holds the Shield audit and kpi.json beside the bag. | No bag holds an image topic. The KPI rail carries no camera (its pilot publishes an action topic). The camera rail (PAS perception, the learned-VLA hil flights) records no rosbag2: none of its 44 hil run folders (demo/out/pas_hil_*, demo/out/pab_aerialvla_04_*) holds a bag, though 31 read KPI-grade in their own kpi.json. So the grant's 'rosbag2 image + state topics' is not captured. The start frame is the first frame of episode.bag2 in 49 of 52 smREL073 bundles, not every one, and of none of the 654 ntFIN5 bags (release 0.7.0, before the frame at arming; docs/data/replay_from_bag_ntFIN5/per_run_records.tar.gz). |
| ARCH-23 | Q3 paraphraser deliverable (Aug-Oct) | DONE | same artefact as WP2-13: the stored paraphrase sets, validator and CLI (guardrail/paraphraser.py) and the separate service the grant names (guardrail/paraphraser_service.py, gRPC with mutual TLS, guardrail/protos/paraphraser.proto), with the 4 C8 sets re-authored and the validator's held-out misses fixed | The orchestrator binding is WP4-10 (DONE) and the per-paraphrase measurement is WP4-09 (DONE). |
| ARCH-24 | 1 PI + 4 Master students (2 ArduPilot/MAVLink, 2 VLA fine-tuning/prompting) | DONE | team as in the grant: 1 PI + 4 Master students (one team); the grant's Personnel line, 01-context/grant-overview.md: "1 PI + 4 Master students (two on ArduPilot/MAVLink, two on VLA fine-tuning / prompt engineering)" | None. |
| ARCH-25 | Observation (Camera / IMU / GPS) reaches the VLA at >=10 Hz (Architecture constraints diagram; ref layer-requirements L2/L5 'Surface obs at >=10 Hz') | DONE | detector 2.18-2.48 Hz on the Jetson flights, ~3 Hz on the PC; VLA frames at its decision rate (~2 Hz); 30 Oct: the lab's own AIOT-VLA (training/aiotvla, docs/DESIGN-aiotvla.md; named NTUT-VLA until 30 Oct) answers in 48.4 ms p50 on the Orin Nano Super (19.1 decisions/s over 600 replayed captures, the server alone) and 10.3 ms on the RTX 4090; on the city rail its requests go every 0.1 s (capture gap p50 0.13 s) 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the feed between ticks (--vla-feed stream; guardrail/vla_backends.py RemoteVLABackend.pump; docs/data/pas_hil_feed_20261115/flights.md): a new camera frame goes to the model without waiting for the 10 Hz tick, the answers still enter on ticks. Four KPI-grade hil flights of AIoT-VLA v10b on the AGX Orin with the feed uncapped and the camera at a 0.05 s capture interval (pas_hil_feed_05-08, d9f5ea0 / deac4b3): 12.39-13.23 distinct frames/s reached the model while it was asked (9.58 before, pas_hil_obs_02); over the whole mission 10.72 and 11.07 in the two flights that held the car for the full 420 s (pas_hil_feed_07 / _08), 3.48 and 8.86 in the two that lost it; decisions parsed 10.62 and 10.93 per second over those two missions; Shield ticks 9.96-9.99 Hz, 39-332 interventions per mission, 0 P0 escapes; the detector 2.31-2.70 Hz and the tick's work as without the feed. | As X-05: with a 15 Hz camera, and while the model is asked. |
| ARCH-26 | 10 Hz Shield monitor budget validated by Orin profiling; Rust hot-path as Q3 fallback (Overview page, locked cross-cutting decisions) | DONE | Profiled on the AGX Orin 2026-11-06 (tools/profile_shield_tick.py, 1,301 real ticks, 50 rules x 51 poses, MAXN, clocks not locked): query p99 0.977 ms / 5 ms, Shield step p99 22.647 ms / 100 ms, 0 ticks over either (deploy/evidence/hil/shield_tick_profile.json). Rust decision: no Rust hot path - the Python Shield uses under a fifth of the query budget and a quarter of the tick budget at the grant's design load | - |
| ARCH-27 | Implementation language Python + Pydantic v2 throughout (Overview page, locked cross-cutting decision) | DONE | Python + Pydantic v2 throughout (guardrail/, pyproject.toml) | 3.11+ version is X-06. |
| ARCH-28 | The same code base runs in all three topologies, all first-class (grant); reference topologies.md: identical Docker image, topology selected by config flag, a 'hard requirement' | DONE | one Dockerfile per role, built natively on both hosts from the same commit (2026-11-06): vlaguard/companion:jazzy arm64 on the AGX Orin and amd64 on the desktop, the same Ubuntu 24.04, ROS 2 Jazzy, Python 3.12.3, MAVROS 2.16.0, mavlink-router v4 and pip set with numpy pinned to 1.26.4 (deploy/evidence/hil/companion_versions.txt, deploy/evidence/dev/companion_versions_desktop.txt, deploy/evidence/hil/images.txt); flown in dev (dev_compose_01) and hil (hil_smoke_03) with the topology chosen by the env file, not a rebuild. flight is the same image with flight.env and the serial-device override (tests/test_deploy_configs.py); the flight topology has not flown, and the flight stage is covered by the written waiver (reading A3; ARCH-14) | - |
| ARCH-29 | Shield loads the Policy IR / signed Bundle at runtime (Architecture constraints diagram: 'Policy IR / Bundle -> loaded -> Shield'); overlaps WP1 | DONE | Shield loads the signed bundle on every rail incl. the city rail since 2026-10-22 (demo/follow_vlm.py:122-193); Jetson metrics policy_source kind=bundle, declared=loaded hash | None. |
| ARCH-30 | No default backend until the 'Q2 measurement set lands' (Architecture constraints, Action space); basis for any backend choice and for the G3 candidate | DONE | deploy/evidence/vla_backend_table.md (Default backend: none; Orin columns empty; regenerated 21-22 Oct); 42-flight PC+Jetson comparison of 7 VLA configs + hand-written follow (CHANGELOG 2026-10-28) not folded into the table 16 Nov: protocol 2 with the AGX Orin column (R1 for stub, bc_v3, OpenVLA-7B bf16 p95 969 ms; server benches for DroneCATS-2B and RaceVLA) and C2 on hil: AIoT-VLA v10b 3 flights (within 30 m 0.211-0.352), AerialVLA bf16 3 (0.052-0.163), DroneCATS-2B 3 (0.132-0.189), RaceVLA 1 (0.060; 2 never passed the start gate); P0 escapes 0 on every flight; Default backend: none. 17 Nov (C2 top-up at 63fffcf): RaceVLA 2 C2 flights (0.060-0.168) with 3 start-gate timeouts listed as censored; C2's null flown 3 times (FollowPilot box-servo, no estimator, no lock, no lost end): 0.105 / 0.065 / 0.111 as flown, 0.105 / 0.030 / 0.099 under the VLAs' end rule, about level with AerialVLA and RaceVLA, below AIoT-VLA; P0 escapes 0 on every flight. 17 Nov (89d2dc2): RaceVLA c2_06 (within 30 m 0.063), so RaceVLA has 3 C2 flights (0.060-0.168) and every backend that can fly C2 has its 3, P0 escapes 0 on all; the table lists nothing the set lacks; Default backend: none. 18 Nov: AIoT-VLA v10b became the default backend for the follow task, after the set (decision 12; `python tools/vla_backend_table.py table --runs F:/vla_drone_offload/jetson_runs --default aiotvla_v10b --default-scope "the follow task" --default-date 2026-11-18`; the tool refuses the line unless every backend that can fly C2 has an Orin timing and 3 C2 flights, the stub has its R1 on the Orin, and the date is after the set's newest measurement, 2026-11-17); deploy/evidence/vla_backend_table.md now reads 'Default backend: AIoT-VLA v10b ... for the follow task' (the default backend since 2026-11-18, after the set's newest measurement of 2026-11-17) with its own C2 numbers (within 30 m 0.211-0.352, KPI outcome 3 fail on unsafe_position_entered, P0 escapes 0). | None for the grant's line (no default until the set lands: the set landed 16-17 Nov, the default was named 18 Nov). Recorded beside it: the default covers the follow task only (no backend has flown C1, and AIoT-VLA reads no CSP); its C2 score is weak (21-35 % within 30 m: 0.211-0.352); the estimator null (FAMILY=estimator) is not flown; R1 does not cover the four backends served over protocol v2 (their Orin timing is a server bench). |
| ARCH-31 | ArduPilot inner loop 'Bit-identical between SITL and firmware' (VLA/autopilot interface table) together with code_revision 'across the entire stack' (ref determinism contract) | DONE | Bit-identity between SITL and firmware is a stated limit, since no FC is flashed. The real FC, a CUAV V5 Nano on ArduCopter 4.7.1, was bench-tested only and never armed (docs/data/fc_bench_20261117/README.md). The hil rig flies the pinned SITL Copter-4.5.7, which reports ArduCopter V4.5.7 (2a3dc4b7) (sitl/setup_sitl.sh). One SHA across the stack: since 17 Nov the ROS 2 Shield and mission nodes and the PAS node write stack_revisions.json (schema vlaguard-stack-revisions/1) beside each manifest (guardrail/manifest.py write_stack_revisions). It holds each host's code_revision and dirty flag, the autopilot firmware as it names itself, the simulator build and single_sha. Since 27 Nov the KPI readers (tools/kpi_report.py, tools/rescore_kpis.py, tools/build_report_results.py) apply guardrail/manifest.py stack_grade_reasons beside is_kpi_grade. They refuse single_sha false on any topology. On hil they also refuse a record that is pending or unreadable, or one that is missing where the run's writer writes it. KPI report of record, ntFIN5: single_sha true 654, false 0, pending 0, missing 0; companion and desktop 21c277170001; ArduCopter V4.5.7 (2a3dc4b7) (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md l.83). The delivered release's smoke smREL073: true 52 of 52, companion and desktop c44a360618e1 (docs/data/kpi_smREL073/kpi_report_smREL073_2026-11-28.md l.83). Every hil KPI report since 21 Nov (smfin1120, ntFIN3, smFIN2, ntFIN4, smREL070, smREL071, pdyn1126, smREL072) reads false 0, pending 0, missing 0. The gate refuses the 12 PAS-rail AerialVLA hil runs of 20 Nov: their record reads pending, because the PAS script never added the desktop's revision (docs/data/kpi_prefix_vla_20261120/README.md). Scope: runs flown before 17 Nov carry no stack record and keep the manifest's single code_revision. | Bit-identity between SITL and flight firmware is a stated limit: the real FC stays on ArduCopter 4.7.1, not flashed, and no flight on it is claimed (ARCH-14). |

### Cross-cutting locked choices

16 items: 16 DONE.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| X-01 | Final delivery: signed final report (2026-11-30 gate) | DONE | Reading A1: the Final Report v1.0 (docs/FINAL-REPORT.md with Appendices A and B) is signed at its Approval block. The signed copy is built from the final master commit (tools/report_to_html.py, tools/html_to_docx_pdf.ps1), and that commit is written into the block's 'Report body' field when the signed DOCX/PDF is built. The counts are checked by tests/test_final_report_counts.py. The KPI figures of record are ntFIN5 at release 0.7.0 (21c2771): 654 KPI-grade, P0 escape 0 on 17,194 P0-violation ticks, fail-safe (r2) 649/649 (Wilson 95 % low 0.994), 0 of 183,961 ticks over 100 ms. Its KPI report was rendered again on 28 Nov with the policy-artefact KPIs (WP1, WP2) measured at the delivered release 0.7.3 (c44a360); the flight KPI figures are unchanged (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md; docs/data/kpi_ntFIN5/README.md, 'Re-rendered 28 Nov'). The release 0.7.3 hil smoke smREL073 met the release gate: 52/52 KPI-grade, P0 escape 0, fail-safe (r2) 50/50 (docs/data/kpi_smREL073/README.md). The policy bundles are signed under the Lab CA (reading A2, WP1-04). (final CI: see docs/data/ci_local/) | Met by the signature in the Approval block of the signed copy. |
| X-02 | Safety Shield acceptance KPI: repair success rate | DONE | same as WP3-20: KPI-grade nightly ntFIN2 (13 Nov) 98.2 % (26,473 of 26,968 repair ticks, 577 Shield-on missions), always-brake null 0 % (guardrail/kpi.py repair_success; theta basis decided 2 Nov). Grant (02-implementation/safety-shield.md): "Repair success rate \| tracked; informs Prefix Compiler effectiveness eval" | Same as WP3-20 (theta judges about a third of the converged ticks). |
| X-03 | Stress Testing locked KPI: average repair count per episode (as a KPI, not only a report column) | DONE | Grant (02-implementation/stress-testing.md): "Average repair count / episode \| tracked \| Counter from Shield repair log". mean repair count per episode and family in tools/kpi_report.py / docs/data/kpi_rollup_2026-10-06.md; on `hil` since 7 Nov, KPI-grade, repairs per episode: rh03 72.1, sm1110 76.1, sm1111 72.1, ge1110 0 (CHANGELOG 2026-11-11 (hil rig flights)); 13 Nov, the KPI-grade nightly ntFIN2 (116214f; tools/kpi_report.py --campaign): 55.4 per Shield-on mission over 577, per template 1.0 to 300.7 and per cell, a row of the KPI summary | None for the tracked KPI. |
| X-04 | Q2 VLA-backend measurement set: the comparison that must exist before any backend becomes the default | DONE | same as ARCH-30 | Same as ARCH-30. |
| X-05 | Observation rate into the VLA: camera/IMU/GPS at 10 Hz or faster | DONE | perception to the controller at 2.2-3 Hz (Jetson det_hz 2.18-2.48; PC ~3 Hz) against the grant's >=10 Hz observation rate 14 Nov, AIoT-VLA v10b on the AGX Orin in the hil topology (master f77efae on both hosts; perception node --link mavros --pilot vla --mission follow; Project AirSim CityLife + ArduPilot SITL 4.5.7 on the desktop; docs/data/pas_aiot_hil_20261114/flights.md): six missions flown (pas_hil_aiot_kpi_01 KPI-grade and unstressed; day_02-04, dusk_01 and rain_01 perception stress, non-KPI; night_01 never started, no red car detected at -4 EV), Shield ticks 9.96-9.98 Hz, 22-297 interventions per mission, 0 P0 escape ticks in every mission, VLA decisions 6.90-7.94 Hz (capture to answer p50 0.110-0.113 s), frames to the VLA 9.10-9.50 Hz while it was asked, camera to first setpoint p50 159-163 / p99 282-296 ms, ArduPilot real-time factor 0.963-0.969. Follow quality on this rail is weak: target lock 0.06 (kpi_01), 0.11 (day), 0.04 (dusk), 0.82 (rain) (docs/data/pas_aiot_hil_20261114/perception_stress_report.md). 15 Nov, the feed between ticks (--vla-feed stream; guardrail/vla_backends.py RemoteVLABackend.pump; docs/data/pas_hil_feed_20261115/flights.md): a new camera frame goes to the model without waiting for the 10 Hz tick, the answers still enter on ticks. Four KPI-grade hil flights of AIoT-VLA v10b on the AGX Orin with the feed uncapped and the camera at a 0.05 s capture interval (pas_hil_feed_05-08, d9f5ea0 / deac4b3): 12.39-13.23 distinct frames/s reached the model while it was asked (9.58 before, pas_hil_obs_02); over the whole mission 10.72 and 11.07 in the two flights that held the car for the full 420 s (pas_hil_feed_07 / _08), 3.48 and 8.86 in the two that lost it; decisions parsed 10.62 and 10.93 per second over those two missions; Shield ticks 9.96-9.99 Hz, 39-332 interventions per mission, 0 P0 escapes; the detector 2.31-2.70 Hz and the tick's work as without the feed. | Holds with a 15 Hz camera (capture interval 0.05 s; at 0.066 s it delivers 12.8 Hz and the feed was not flown) and while the model is asked: with no detector box it is not asked, by design, and the detector that gives the hint's phrase runs at 2.3-2.7 Hz. A request carries the frame and the EKF pose at its arrival (this rail's frames have no simulator stamp). n = 4 flights. |
| X-06 | Locked implementation language: Python 3.11+ (Policy DSL, Prefix Compiler, Safety Shield and Stress Testing pages) | DONE | same as WP1-25: the hil companion image runs 3.12.3 (ARCH-28); the desktop env vla-real is 3.10.20; on 14 Nov the PAS launcher's scene steps ran on guardrail-py312 for the AIoT-VLA hil missions (scripts/run_pas_ardupilot.ps1 at f77efae). | The VLA runtime (torch, transformers) stays on vla-real 3.10, outside the grant's 3.11+ decision for the non-VLA stack; read as: the 3.11+ floor binds the non-VLA stack, and the VLA runtime stays on 3.10 (decision 8; section 7). |
| X-07 | Validate the Shield's 10 Hz monitor budget by Orin profiling, with a Rust hot path as the Q3 fallback | DONE | same as ARCH-26: Orin profile within budget, Rust hot path not needed (decision recorded 2026-11-06) | - |
| X-08 | One code base runs first-class in dev, hil and flight; promotion dev to hil to flight is a config flag (identical image) | DONE | same as ARCH-28: one image definition, dev and hil flown with it, flight a config change (ARCH-14) | - |
| X-09 | R6 freeze gate: IR shape frozen at the end of Q1 (2026-04-30); every later IR-shape change carries a semver bump and a migration note | DONE | same as WP1-22 (docs/IR-MIGRATIONS.md, the freeze rule in check_lock) | - |
| X-10 | Bundle identity fields: policy_hash as a full SHA-256, signed_by (Lab or ITRI CA), changelog as a diff against the previous version, issued_at, layers_merged | DONE | guardrail/bundle.py manifest_for: policy_id, version, generation, full 64-hex policy_hash + hashes, ir_schema_version, signed_by, changelog, issued_at; layers_merged in the IR of merged policies (guardrail/models.py, Policy.layers_merged); `changelog_diff` + diff_text when written with `previous` (895b5a0, 29 Oct; CLI --previous; tests/test_bundle.py:183, 243, 270); CA `signer` record since 16 Nov (tests/test_bundle.py test_a_key_under_a_ca_signs_as_a_ca_reference_and_its_chain_verifies, test_swapping_the_authority_is_one_registry_entry_and_a_re_sign, test_a_certificate_that_does_not_chain_to_a_listed_authority_is_refused, test_a_ca_reference_without_its_signer_record_is_refused; test_the_manifest_carries_the_five_grant_fields_with_a_ca_reference, a merged policy under a throw-away CA, with layers_merged). (1) signed_by: since 27 Nov all 17 flight bundles on both hosts carry signed_by lab-ca:71d1e071d3985bc6:ed25519:<kid>, a Lab CA reference (the signing authority under reading A2, WP1-04), with a `signer` record (authority, CA fingerprint, leaf serial and subject, certificate SHA-256) and signer.cert.pem in the archive, chain verified (docs/data/bundle_ca_20261127/bundle_ca_desktop.json, bundle_ca_agx.json). (2) the changelog diff: written by the builder since c928a23 (23 Nov, in 0.7.0); the flight bundles were rebuilt with it on 25 Nov (docs/data/x10_bundle_rebuild_20261125/), and the AGX's rebuilt fase3-sim-demo-v0.1.4 flew on hil in bat1125_001; since the 27 Nov re-sign the four first-version bundles built before X-10b (msl-dem-sitl, -zone, -corridor, follow-car-citylife-people) carry the first-version changelog text distance-envelope-sitl v0.1.0, also a first version, already said so (25 Nov), and the other 12 carry a diff against the earlier version. In the re-sign policy_hash, policy_id, version and IR were kept; signed_by, signer and issued_at changed. Flown: ca1127_001_nfz-head-on (hil, 9589623, KPI-grade) flew fase3-sim-demo-v0.1.4 (the Orin's copy): policy_hash sha256:b665adba..., issued_at 2026-11-27T00:17:47Z, signed_by lab-ca:71d1e071d3985bc6:ed25519:715f70c4647a9701, signature and chain verified, require_chain true (docs/data/bundle_ca_20261127/ca1127_001_metrics.json policy_source). Then the release 0.7.2 smoke smREL072 (418b151, 27 Nov, hil, 52 KPI-grade runs): every run records the same signed_by, chain verified, require_chain true (docs/data/kpi_smREL072/README.md); the release 0.7.3 smoke smREL073 (c44a360, 28 Nov, hil, 52 KPI-grade runs) the same, chain verified 52/52 (docs/data/kpi_smREL073/README.md). | DONE since 27 Nov by the review (3 of 3 reviewers DONE); the Lab CA is the signing authority under reading A2 (WP1-04). layers_merged has never been on a flown or flight bundle: no flight policy is a layered merge, so it is shown only in tests (the merged policy above, the wp1_roundtrip lint cases); on a monolithic policy it would claim a merge that did not happen. The re-sign changed issued_at on all 17 (the 25 Nov rebuild kept it), so issued_at records the re-sign, not the first issue, and the pre-CA, desktop and AGX copies of one policy_id and version (e.g. fase3-sim-demo 0.1.4) are distinct signed artefacts with the same policy_hash, with no version bump and no changelog line for the re-sign; replay is keyed on policy_hash, so it is unaffected. issued_at is the signer's own claim (no trusted timestamp). The KPI runs of record flew the earlier copies: bare-key signed_by, and for ntFIN5, smREL070 and the Gazebo mid-term gate (dev) no changelog diff either; only ca1127_001 and the release smokes smREL072 and smREL073 (52 runs each, releases 0.7.2 and 0.7.3) show the CA reference on a flown bundle. The 31 older versions in bundles/ keep a bare signed_by (chain none, refused by --require-chain); four legacy unsigned replay bundles carry signed_by 'lab-ca:dev-placeholder', reported unsigned by the verifier. The changelog text names the rules and fields that changed, not their old and new values (those are in changelog_diff). |
| X-11 | MSL as an opt-in altitude reference on geometry primitives (altitude_ref AGL/MSL) | DONE | altitude_ref MSL is a field on PolygonFence (CircleFence inherits it), AltitudeEnvelope, Corridor, DynamicNFZ and CorridorSwap (guardrail/models.py:969, 1084, 1238, 1366, 1441). It is opt-in: unenforced_rules refuses MSL unless the caller declares a ground and checks the DEM's WGS84 origin (models.py:2178-2200; terrain.FlatGround, GridGround DEM, load_ground); the compiler takes MSL over a ground (compiler.py:2382 _msl_view). The Shield converts at the query point and the forecast stands over the DEM (wf20; tests/test_terrain_forecast.py 10/10). Headless: tools/msl_dem_rail.py -> docs/data/msl_dem_rail.json (independent oracle, P0 0 against the nulls) and tools/msl_dem_kpi.py -> docs/data/msl_dem_kpi.json. Without a ground, 141/141 library episodes are byte-identical. The ROS 2 / SITL-hil rail readers take a declared ground named by path and sha256 (wf21-msl-readers, CHANGELOG 2026-11-19 (wf21, code)): the Shield node, sitl/hil harness backends, mission node, gcs_backstop, bundle_origin, VLA stub and bridge, build_flight_bundles (--ground at sitl/ros2_shield_node.py:6184, sitl/gcs_backstop.py:477, ros2_mission_node.py:937). The RTL leg is judged against the ground along it. FENCE_ALT_MAX is capped by the MSL ceiling: msl_dem_sitl 40 m, corridor 26 m. The replay tarball carries the signed DEM. ground_flight_problems refuses a run whose metrics name no ground or another one. Tests msl_readers 20/20, msl_rail 21/21, msl_dem 13/13 in lab CI 8e73f86 (22 Nov, 187/187 files, 4854 of 4870 tests, 16 skipped). Flown (CHANGELOG 2026-11-19 (X-11 flown on SITL and hil)): the msl_dem profile (MSL polygon_fence, corridor and altitude_envelope over the ridge DEM experiments/worlds/dem_cmac_wgs84.json, sha256 44ed8a16...); on SITL at bdc0af6 6 missions; on hil (AGX Orin + SITL) at e56d3e7, campaign msl_dem_hil2, 6 missions (demo/out/hil_smoke/msl_dem_hil2_00N_*, each kpi.json kpi_grade true). Both: P0 escape 0 (null 0.045/0.051), 0 breaches, 0 polygon entries, true_p0_flown_ticks 0, 3 pass / 3 fail per rail, each zone-ridge cell a false trigger (fail-safe 0/1) (docs/data/msl_dem_20261119/sweep_sitl.json, sweep_hil2.json). Bag replay identical on hil 6/6 and on SITL 4/6 (the other 2 one watchdog tick off); `guardrail.replay verify` passes on all 6 DEM-carrying tarballs. The ridge passes with the goto pilot's terrain feed-forward (guardrail/vla_stub.py TerrainFeed, 27cbeee; theta and FSM unchanged): msl-dem-zone-ridge on the SITL rail at 59e19a5, 22 Nov, 2/2 reached the pad, P0 escape 0, 0 polygon ticks (docs/data/shakedown_20261122/README.md, msl_ridge_sitl_sweep.json). Earlier 18 Nov: msl_dem_sitl_01..03 (dev SITL). | Met against the grant's wording (kuanting-vla-uav-guardrail docs/02-implementation/policy-dsl.md:17, :61: AGL by default, MSL "supported as an opt-in field on geometry primitives"); moved to DONE on 25 Nov after an independent review by three reviewers, all DONE. (1) MSL enforcement is tested on all six altitude_ref primitives (tests/test_msl_primitives.py, 6ab8475, in 0.7.0): each refused without a ground, judged over a flat ground and over a DEM band, the dynamic_nfz / corridor_swap hot-apply judged over the Shield's ground. (2) Readers on the ROS 2 / hil, PAS, follow_vlm, vla_fly and Gazebo rails take a ground (`--ground scene`, terrain.declared_home_ground; run_sitl_demo.py `--ground sitl\|FILE`; 6ab8475) and refuse an MSL policy without one. (3) The hil zone-ridge cells were re-flown with the terrain feed on 25 Nov at 21c2771 (campaign x11r1125, AGX Orin + SITL; docs/data/hil_1125_x11_wp307/): 2 passed of 2, P0 escape 0, 0 polygon ticks, fail-safe labels 2/2 (on 19 Nov both were false triggers), FSM max Normal (target_x -45) and Brake (-20), each run's metrics naming the pinned DEM (sha256 44ed8a16...), kpi_grade true; replay from the bag identical 2/2, open-loop replay at the flown revision identical 2/2 with the replay fix of 25 Nov (a75f1f5, 7b6fa63; msl_dem_hil2 6/6), not_replayable before it. Notes, not required by the grant (it names no ground source, terrain data, KPI cell or margin): circle_fence, dynamic_nfz and corridor_swap MSL are tested, not flown; the DEM is synthetic, a declared ridge about CMAC, and SITL's own ground stays flat (experiments/profiles/msl_dem.yaml:28); no MSL cell is in a KPI set (msl_dem grant_kpi_bearing false; in none of smFIN2, ntFIN4, smREL070, ntFIN5); the 0.25 m DEM band margin is the repair's aim, not a guarantee under baro or EKF noise; the six legacy AirSim scripts take no ground and refuse MSL (fail-closed). |
| X-12 | Hybrid hot-update rule: only dynamic_nfz, time_window_switch and corridor_swap may change mid-flight; polygon_fence and the envelopes stay locked from mission start | DONE | hot-apply of a fixed polygon fence after take-off refused; only the three classes hot-apply (guardrail/api.py; CHANGELOG 2026-10-07); tests/test_api.py, tests/test_shield_events.py | None. |
| X-13 | Single rule-evaluation code path (the checker reuses the IR and nothing else evaluates rules) | DONE | There is one rule evaluator, built from the IR: guardrail/shield.py (RuleClock, the point verdicts, zone_breached :777, clearance_breached :877, standoff_breached :896). Outside shield.py it uses the kinematic cap's threshold test fsm.cap_exceeded (guardrail/fsm.py:813) and the ring helpers _point_in_ring and _dedupe_ring imported from guardrail/models.py (guardrail/shield.py:145-166). Every reader calls it: compiler.action_violations / tools/prefix_eval, fence_export, rtl_path and replay.zone_dwell. FenceGuard does too: its building half is judged by clearance_breached against Shield.clearance_limits_now, and guard_field accepts 'shield' only (demo/follow_vlm.py:2455-2469; --guard-field choices ('shield',) at :9409). Since 82b9cd6 (in v0.7.2 and v0.7.3), models.start_conflicts asks shield.zone_footprint_reached (guardrail/shield.py:785; guardrail/models.py:2330), with every answer unchanged against the old code (tests/test_start_conflicts_onepath.py). The AST scans over SOURCE_DIRS (tests/test_shieldcore_onepath.py:75) find no second evaluator. The only IR rule thresholds compared outside the checker are the 14 named non-verdicts (_THRESHOLD_COMPARISONS_NOT_VERDICTS, :1110), and the only clearance measures outside the library are 3 planning and identity measures (_CLEARANCE_OUTSIDE, :977: city_planner.inflate, city_planner._penalty_flat, identity.__init__); a new or stale entry in either list fails the test (final CI: see docs/data/ci_local/). Reading (a): the autopilot's GeoFence backstop and the listed non-verdict checks are outside the system's rule evaluation. The grant keeps that backstop (safety-shield.md:117, 'AP GeoFence coordination'; WP3-10), and it triggered 18 of ntFIN5's 649 scored fail-safes (docs/data/kpi_ntFIN5/kpi_report_ntFIN5_2026-11-28_offline_c44a360.md:18). | - |
| X-14 | Dynamic-actor stressors in parameter sweeps: wind (and GPS denial) | DONE | wind, gusts, GPS dropout stressors with a closed-loop ArduCopter wind model (CHANGELOG 2026-10-06, -13); experiments/templates/wind_disturbance.yaml, sensor_degradation.yaml; since 7 Nov both flight rails (sitl, hil) fly wind (SIM_WIND_SPD at the episode's t=0, SIM_WIND_DIR, SIM_WIND_T 1), GPS denial (SIM_GPS_DISABLE on the episode clock, aligned by the node's announced t0) and start jitter (SITL_HOME) as SITL's own parameters, set from the simulator side and checked fail-closed (sitl/sitl_stressors.py, tests/test_sitl_stressors.py; CHANGELOG 2026-11-07 (SITL stressors)); flown on the hil rig on 7 Nov in the smoke campaign rh03 (44 missions, the AGX node at fb96fe5): wind within authority 5 m/s in two lanes, beyond it 8 and 12 m/s, GPS denied from 4 to 11 s in two lanes, every parameter read back as set and applied at its time (each run's scenario_score.json sim_stressors_check: no problems); X8 sent one GPS-dropout mission home (rh03_040, the first X8 flown on hil); P0 escapes 0 on all 44. The 2 Nov pymavlink runs sitl_wind8_shield_on/off_20261102 flew 4 m/s at 15 m, not the 8 they name (SIM_WIND_T unset; retracted 2026-11-07) | Gusts stay headless (4.5.7 has no faithful stand-in: SIM_WIND_TURB redraws from SITL's own random numbers); the rails refuse them by name. GPS noise / latency fly on the hil and sitl rails since 115640b (21 Nov), applied companion-side to the pose the Shield and the FSM read, not through SITL (SIM_GPS_NOISE is altitude-only): sk1122a, smFIN2 and ntFIN4 at 59e19a5 (WP3-19). The KPI-grade nightly ntFIN2 (13 Nov) flew the wind and GPS-denial cells, but 3 of its 24 gps-dropout missions were not accepted (SIM_GPS_DISABLE never applied) and 8 wind-beyond-authority missions never flew (the desktop out of memory); the supplement ntFIN2s re-flew those cells on all 3 seeds (15 missions, 11 accepted); gps-dropout lane 15, 6-13 s, seeds 1002 and 1003 were again not accepted (the denial's end applied 0.29-0.60 s late; CHANGELOG 2026-11-13). |
| X-15 | SROS 2 security on the ROS 2 graph (the harness mTLS reuses its root CA) | DONE | SROS2=1 opt-in, one enclave per process, Enforce; flown 2026-11-02 (ros2_sros2_shield_on_20261102, allow-all permissions); since 3 Nov per-topic permissions: sitl/sros2_policy.xml signed per enclave by the keystore's permissions CA = the lab CA (tools/sros2_setup.sh --permissions-only): only the Shield's enclave may publish /shield/setpoint, only the VLA slot's /vla/action_4d, MAVROS its own mavros/* and uas1/*; tools/sros2_check.sh 8/8 in WSL Jazzy, check 7 showing a VLA-enclave publisher of /shield/setpoint refused ('rt/shield/setpoint topic not found in allow rule'); flown sros2_policy_on_20261103b (re-flown 4 Nov from the main checkout, code recorded: sros2_policy_on_20261104) with the policy's permissions (permissions_scope per_topic in hil_evidence.ros2_security); the harness that drove harness_corridor_20261103c (re-flown 4 Nov from the main checkout, code recorded: harness_corridor_20261104b) ran its gRPC mTLS with the same lab CA (~/harness_pki in WSL) (CHANGELOG 2026-11-03 (G)) | The Windows harness PKI (%LOCALAPPDATA%/vla-guardrail/harness_pki) is still another CA: run the harness where the keystore's CA is, as done here, or seed both from one CA. |
| X-16 | Meaning of the vla_model_hash field (SHA-256 of the VLA's weights). This is a nuance of the six-field manifest item rated DONE | DONE | vla_model_hash carries the weights' SHA-256 on VLA flights (e.g. nfz3b_orin_hier_aerovla_20261028/manifest.json aerialvla:...@sha256:ca9e8701...), none:hand-written-controller otherwise | None. |


## Appendix B — independent-review note

Copied from docs/FINAL-REPORT-outline.md at 2f22116: the dated note of 13 Nov, kept as a historical record. Every row's current status is Appendix A's (re-assessed 28 Nov). Out of date by 14 Nov: point 7 (the start-up fix is on master since d7bb6a2 and flew in hs09 and in the full campaign ge1113 with no tick over 100 ms, section 5.6); the supplement was merged (WP3-15 then held on 567 scored runs); on 14 Nov WP3-13 moved to DONE, WP3-07's edges all flew in hs10 (section 4.3) while the row stayed PARTIAL at that date (SET_MODE LOITER on no hil flight), and WP4-17's report was committed from a clean tree (Appendix A on 14 Nov: 88 DONE, 41 PARTIAL, 6 DRIFT, 1 MISSING).


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
paraphrase and prefix KPIs have no `hil` value. WP4-04: no Gazebo sweep.
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
