# Final Report — outline and evidence map (draft)

Semantic-Spatial Translation and Safety-Constrained VLA for ArduPilot UAVs ·
NTUT AIoT Lab · outline v0, 2026-10-29

> **Status: OUTLINE, not the report.** The grant's final-delivery gate is
> 2026-11-30 (a signed final report, audit item X-01). This file fixes the
> report's structure and maps every contract obligation to its evidence, so
> the report can be written from it section by section. Three things are not
> ours to fill in: the **signer and the signing CA** (PI, asked 12 Oct, Q5),
> the **KPI numbers of section 5**, which by the grant come only from a
> Stress Testing run in the canonical `hil` topology (none exists yet), and
> the **PI rulings** listed in section 9.

Every status below is the contract audit's (136 obligations, first done
2026-10-05, refreshed 2026-10-28, with the 28-29 Oct work applied), then
re-checked: on 29 Oct four independent reviewers held each of the 64 items
then marked DONE against the grant's own wording, and 27 of them did not
meet it (2 more were unclear). Those are re-stated as PARTIAL or DRIFT, each
with what the review found; three of them (WP2-04, WP2-07, WP4-20) were
fixed the same night. On 2 Nov the WP2 and WP4 correction notes were
confirmed sent (WP2-12, WP4-21), the Gazebo rail first flew end-to-end
(ARCH-16, late: DRIFT), and the paraphraser became the separate service the
grant names (WP2-13), and the audit record took the grant's field meanings
(WP3-11). **42 DONE, 81 PARTIAL, 12 DRIFT, 1 MISSING.** DONE means the code
or the artefact the grant names exists and matches the grant's words; it
never means KPI-grade.

## 1. Summary (to write last)

Points the summary will carry, each already backed by section 4:

- The Guardrail exists in the grant's structure: a Policy DSL with a typed
  IR and signed bundles (WP1), a Prefix Compiler that emits a Constraint
  Summary Pack (WP2), a Safety Shield with an escalation FSM in front of
  ArduPilot (WP3), and a Stress Testing harness with a scenario library,
  episode bundles and a KPI report generator (WP4). Thirteen parts are
  built differently from the grant's words (DRIFT, section 7) and the
  report says so.
- No P0 violation has escaped the Shield on any rail flown (section 5,
  KPI-grade value pending `hil`).
- A learned model (AerialVLA) has flown ArduPilot SITL behind the Shield and
  the FSM (29 Oct); the perception rail has flown over MAVROS 2 in the
  grant's `dev` topology (29 Oct).
- What the report cannot claim yet: any KPI-grade number, a CSP-reading VLA
  (WP2-01), the Gazebo functional rail (ARCH-16), the `flight` topology
  (ARCH-14).

## 2. Scope and acceptance criteria

The locked KPIs and where each stands. "KPI-grade" means `is_kpi_grade`
accepts the run: `hil` or `flight` topology, sim_speedup 1.0, a clean
code_revision. Every stored run is refused today.

| KPI (grant) | Target | Best evidence so far (not KPI-grade) | Blocked on |
|---|---|---|---|
| P0 violation escape rate (WP3-14) | 0, hard limit | 0: harness library 179 Shield-on episodes; every kpi.json of the 45 city flights of 27-28 Oct; the PAS+ArduPilot flights of 12-21 and 29 Oct | `hil` run |
| Mean time to safe (WP3-17) | reference sweep ≤ 2.0 s | library 3.135 s mean over 37 recoveries, max 6.7 s, since recoveries climb at the cap (29 Oct; 4.308 s before); `pas_vla_goto_02` 1.155 s mean / 1.6 s max, `pas_dev_mavros_01` 1.849 / 2.499 s (one flight each) | `hil` run |
| Fail-safe trigger correctness (WP3-15) | ≥ 0.99 | library 145/171 = 0.848, nightly 544/714 = 0.762, never-trigger null 0.942 | PI rubric + theta basis (Q1/Q2); then the false-trigger fixes |
| Repair success rate (WP3-20, X-02) | tracked | 0.776 on the 8 Oct pymavlink run | theta basis (PI Q1); `hil` run |
| Mean repair magnitude (WP3-16) | tracked | computed by tools/kpi_report.py; values only from headless or non-grant rails | `hil` run |
| Average repair count per episode (X-03) | tracked | tools/kpi_report.py, docs/data/kpi_rollup_2026-10-06.md | `hil` run |
| Mission success rate (WP4-08) | tracked | 0.603 headless library | `hil` run |
| Per-paraphrase robustness (WP4-09) | outcome stability across paraphrases | paraphrase arms in the harness; the paraphraser a separate gRPC service since 2 Nov; validator on a held-out set (heldout_2026_11_02): 1/50 false accepts, 14/44 false refusals, before that set's own misses were tuned on (0/50, 3/44 after, a dev figure) | a text-reading pilot (G3 ruling) |
| Monitor tick ≤ 100 ms at 10 Hz; geometry query ≤ 5 ms at ~50 rules x 50 poses (WP3-13) | budget | Orin Nano city flights: tick work p50 22 ms / p95 61-63 ms, 15-25 of ~4200 ticks over 100 ms; desktop check 0.45-0.87 ms at 50 rules | 50 x 50 profile on the Orin |
| DSL acceptance: schema pass 100 %, golden 100 %, cold start ≤ 1 s, hot-apply ≤ 50 ms, query ≤ 5 ms (WP1-19) | budget | desktop cold start p95 376 ms; hot-apply receipt to swap 0.444-4.443 ms over 18 REST events in flight (desktop WSL, ROS 2 rail, 29 Oct) | Orin bench |
| Policy load round-trip (WP1-17) | 100 % | re-run 29 Oct: round-trip 32/32, tamper 32/32, negative 35/35 refused (the published docs/data/wp1_roundtrip.json is the 20 Oct run over 29 policies) | - (DONE; regenerate the published file) |
| Bundle replayability (WP1-18) | load the bundle whose hash matches | replay bundle per run; headless re-fly bit-for-bit (tools/rerun_episode.py) | a clean-tree `hil` run |
| Prefix-token budget (WP2-02) | budget respected, CSPBudgetExceeded on overflow | docs/data/csp_kpis.json; tests/test_csp.py | - (DONE) |
| CSP P0 coverage (WP2-03) | 100 % | re-run 29 Oct over 32 policies: 122/122 P0 at 120 tokens (naive 117/122), 0/122 null; the published docs/data/csp_kpis.json is the 21 Oct run over 29 policies | - (DONE; regenerate the published file) |

## 3. System architecture and topologies

- 3.1 The action contract: 4-D body-frame action (vx, vy, vz, yaw_rate) at
  10 Hz; body-to-NED conversion in one place (sitl/mavlink_adapter_node.py).
  DONE: ARCH-05, WP3-03. Not as the grant words it: no VLA has emitted the
  10 Hz setpoint into ArduPilot (ARCH-01); the repair operators act in a
  world frame and the frame conversion lives in several modules (ARCH-04,
  WP3-04, recorded as DRIFT); the adapter has no MISSION_ITEM_INT path
  (WP3-09).
- 3.2 Pipeline: bundle → CSP → pilot → Shield (+ FSM) → MAVROS 2 →
  ArduPilot, with ArduPilot's GeoFence as the last backstop (WP3-10,
  ARCH-09).
- 3.3 Topologies (ARCH-11, ARCH-12, ARCH-14, ARCH-13):

| Topology | Grant definition | State on 2026-10-29 |
|---|---|---|
| `dev` | everything on one desktop | flown: SITL + MAVROS 2 (7 Oct); PAS + ArduPilot + MAVROS 2 in WSL, `pas_dev_mavros_02` the first run stamped `dev` (29 Oct). A `dev` run counts only with the PI's written waiver (DEV_KPI_WAIVER). Gazebo absent (ARCH-16). |
| `hil` | desktop runs the simulators, MAVROS and the GCS; the Jetson Orin runs the VLA and the Shield | not flown. Needs Docker on the Jetson Orin Nano (installed with sudo, docs/RUNBOOK-orin-hil.md section 2); the VLA on the AGX Orin once it is set up. |
| `flight` | Orin companion on a real ArduPilot drone | not flown; a drone or a written waiver (12 Oct, Q7). |

- 3.4 Rails: ArduPilot SITL (+ MAVROS 2), Project AirSim + ArduPilot (the
  perception rail), Project AirSim CityLife with simple_flight (the follow
  comparisons; outside the ArduPilot architecture, see ARCH-02 and
  section 6), Gazebo Harmonic (not built, ARCH-16).

## 4. Results by work package

One subsection per WP, in the grant's order, each with: what the grant
asks; what was delivered, with the measured numbers and the evidence file;
the deviations; what remains. Appendix A holds the per-obligation table
the subsections are written from.

- 4.1 WP1 Policy DSL and IR: 25 obligations. The IR with a spatial index,
  fences, corridors, time windows, breach actions, priorities, layered
  merge, ingest tools and the round-trip KPI are DONE. Not as the grant
  words them: ConstraintBase lets the grant's required fields default
  (WP1-01), the lint normalises no ring direction, takes no holes and checks
  only altitude-band conflicts (WP1-02), policies are metre-canonical, not
  WGS84 (WP1-14), and the 20 July gate was missed (WP1-21). Open: the
  signing authority (WP1-04),
  compatibility with the PI's reference implementation (WP1-05),
  distance_envelope enforcement (WP1-07), a flown REST hot-apply (WP1-15),
  a KPI-grade replayable bundle (WP1-18), the CityLife bundles in the
  tracked store from a clean tree (WP1-20), the Orin benches (WP1-19), the
  PI's ruling on the one unbumped IR change in the migration register
  (WP1-22, docs/IR-MIGRATIONS.md), Python 3.11+ (WP1-25).
- 4.2 WP2 Prefix Compiler: 19 obligations. The CSP model and its schema
  (since 29 Oct it accepts the spec's own example), budget, coverage,
  relevance explanations, the adapters (the cost map now also as YAML),
  templates and their tests are DONE. The mission context lacks home,
  target and allowed modes, and the risk weights were never tuned (WP2-05).
  Every city follow writes its CSP since 28 Oct
  (WP2-11), and a flown REST hot-apply re-issues it per generation (WP2-15,
  29 Oct). Since 2 Nov the paraphraser is the separate service the grant
  names, gRPC with mutual TLS, serving validated paraphrases of the
  templated NL (WP2-13; the orchestrator does not call it yet, WP4-10).
  Open: no flown VLA has read the CSP (WP2-01), so the CSP on/off
  effectiveness delta (WP2-09, WP4-15) is unmeasured and the "Prefix
  Compiler complete" gate (WP2-18) waits on both; the PI's acceptance of multi-scale geometry derived at load
  rather than shipped in the bundle (WP2-16); the correction
  note's delivery to ITRI (WP2-12); Python 3.11+ (WP2-19).
- 4.3 WP3 Safety Shield: 24 obligations. Monitor / filter / projection
  (WP3-02), the 10 Hz ROS 2 interface (WP3-03), the IR invariant (WP3-21)
  and the package (WP3-24) are DONE, and since 29 Oct so are the REST
  listener driven during a flight (WP3-12) and one flown zone per breach
  action (WP3-08), on the ROS 2 rail with MAVROS, and since 2 Nov the audit
  record (WP3-11: the grant's field names with the grant's content,
  shield_audit.jsonl). Not as the grant words
  them: the mid-term node came after the 20 July gate (WP3-01); repairs in
  a world frame (WP3-04); the city rail flies a 3 s lookahead (WP3-05);
  no MISSION_ITEM_INT path (WP3-09); one Shield-on fence breach unexplained
  (WP3-10); no `hil` run
  (WP3-18); the smoke matrix never run per build (WP3-19); more than one
  rule-evaluation path (WP3-22); the trajectory buffer (WP3-23). Open: the
  theta basis and whether PathRepair goes on by default (WP3-06),
  the FSM edges not yet flown, among them G10 after a landing (WP3-07),
  the Orin tick profile (WP3-13), and the KPIs of section 2
  (WP3-14 to WP3-17, WP3-20).
- 4.4 WP4 Stress Testing: 21 obligations. The scenario library (WP4-02),
  the KPI report generator (WP4-07) and, since 30 Oct, the Shield audit in
  every harness bundle (WP4-20, shield_audit.jsonl) are DONE. Not as the
  grant words them: the smoke set has never run per release (WP4-14). Open:
  the exporters' rosbag2 path (2 Nov: `executed` is what MAVROS was sent,
  from the bag) run on a real bag (WP4-13); the event families driven into a flying
  SITL + MAVROS run by the harness (WP4-03; on 29 Oct a scripted client
  drove dynamic_nfz and time_window_switch events), the library on the ArduPilot and `hil` rails
  (WP4-04), the gRPC harness driving an episode (WP4-05), CI smoke per build
  and the nightly (WP4-06), the perception-stress scenarios (WP4-16,
  WP4-18), the final demo gate (WP4-19), the KPI report itself (WP4-17),
  and the KPI-bearing items of section 2 (WP4-01, WP4-08, WP4-09, WP4-11).

## 5. KPI report (the grant's final deliverable, WP4-17)

To be generated by tools/kpi_report.py from the nightly `hil` campaign
(WP4-01, WP4-06): per scenario family, every KPI of section 2 with its
episode count and seeds, against the reference targets. Nothing in this
section may come from a `dev`, pymavlink or simple_flight run unless the PI
waives it in writing (21 Oct, #4: the dated fallback, unanswered).

## 6. VLA backend measurement set (ARCH-30, X-04)

The grant asks for a Q2 measurement set before a default VLA backend is
chosen. What exists is the follow comparison on Project AirSim CityLife
(simple_flight, so outside the ArduPilot architecture and never KPI
evidence): every model flown pure-VLA and hierarchical (the VLA chooses the
intent, the hand-written controller flies it), on the desktop and on the
Jetson Orin Nano, 27-29 Oct, scored by the share of the mission within 30 m
of the car (pure VLA from its first acquisition). Runs 27-29 Oct
(`demo/out/cmp_*`, `cmp2_*`, `rep3_*` to `rep5_*`, `orin_*`); n is the
number of flights, std the sample standard deviation:

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

How to read it. AerialVLA original, hierarchical, scored 0.979-0.999 in
four of its five PC flights and 0.309 in one (`cmp2_hier_aerovla`), where
the detector had no box on the car for 2,921 of 4,169 ticks after a corner
and the aircraft ended 99.6 m behind on average; its median is 0.987. In
hierarchical mode the VLA's only lever while the car is locked is the
stand-off (12 m at fwd 5 to 22 m at fwd 0), and the score falls with the
stand-off, so a fixed 12 m controller is the ceiling on the PC; the VLA's
own search flew 0-11 % of the ticks (AerialVLA original, PC). No VLA beats the hand-written
controller consistently, and the fine-tuned AerialVLA runs (r1-r4) sit
below the original in both modes. To add: AGX Orin timings, then a default
chosen with the PI.

## 7. Deviations register

Each deviation, the grant text it departs from, and its state:

| Item | Grant | What we do | State |
|---|---|---|---|
| WP1-25, WP2-19, X-06 | Python 3.11+ | 3.10 (the Jetson's JetPack Python, and the AerialVLA stack) | PI said follow the grant (6 Oct): run the stack on 3.11+ in the Orin container, or a written acceptance |
| WP1-14 | WGS84 canonical, AGL default, ENU/NED computed lazily | a WGS84 form exists, but 31 of 32 policies are written and hashed in local metres (the WGS84 one never flown); projection runs at load; MSL cannot fly | recorded deviation 1 (PI-DECISIONS-2026-10-06 Part C) understates it: re-state it, or convert the flown policies |
| WP1-01 | ConstraintBase: constraint_type, scope, priority, violation_action required | they default (hard, absent, P0, 'repair', a seventh action value) | make them required, or a written waiver |
| WP1-21, WP3-01, ARCH-15, ARCH-17 | dated gates: mid-term 2026-07-20, G1 2026-06-30 | IR committed 2026-10-06; ROS 2 Shield node from 2026-08-19/24; G1 passed 2026-10-08 over SIM_AirSim JSON, not HIL_* | state the actual dates; a PI ruling on G1's swap rule |
| WP3-04, ARCH-04 | body-frame action, one body-to-NED boundary in the adapter, repairs in the VLA's frame | repairs act in a world (North/East) frame; the conversion is in the Shield node, the adapter and three more modules | re-frame the Shield, or a written acceptance |
| WP3-11 | audit fields raw_action / emitted_action (body), category 'geometric', repair_attempts ok/skip, file shield_audit.jsonl | since 2 Nov (audit record version 4) the grant's names carry the grant's content; this project's world-frame actions, finer category and repair records are raw_action_world / emitted_action_world, rule_category and repair_results; every rail writes shield_audit.jsonl | closed 2026-11-02; stored runs keep audit.jsonl in versions 1-3, read through guardrail.audit.upgrade_record |
| WP3-22, X-13 | exactly one rule-evaluation code path | the Shield's monitor plus compiler.action_violations / _RuleClock, fence_export, rtl_path and FenceGuard, kept equal by tests | fold them into the Shield's evaluator, or a written acceptance |
| WP4-13 | exporters: rosbag2 to JSONL / Parquet, action.executed = what MAVROS published | since 2 Nov the exporter reads the run's rosbag2 (with `rosbags`) and takes action.executed from /mavros/setpoint_raw/local; without a bag (every rail but the ROS 2 one), without `rosbags` or headless it falls back to the flight log's verdict, labelled on every step | open: the bag path has not run on a real bag (`rosbags` is not in vla-real); run it on the 19 stored ROS 2 runs |
| WP1-07 | distance_envelope to people, buildings, roads | buildings via obstacle_clearance, the followed subject via subject_standoff; distance_envelope declarable and refused by the flight loaders | open (code) |
| WP2-16 | multi-scale geometry, P0Entry.geometry_ref: "The Prefix Compiler picks the scale, never re-computes" | the fine / coarse / bbox scales are computed in one place (guardrail/ir.py) from the IR's polygons, deterministically, when the policy is compiled or loaded; the compiler reads them there and never re-computes them (since 8 Oct); the bundle carries the IR, not the scales | recorded deviation (29 Oct); PI to accept, or the bundle ships the scales |
| WP1-22, X-09 | IR shape frozen at end of Q1, every later change version-bumped | every post-freeze IR change has a row in docs/IR-MIGRATIONS.md (29 Oct); one, `locked_by` (2385f84, 11 additive fields), kept 2.1.0 by the lock's additive rule | PI to accept the additive rule, or 2.2.0 at the next release |
| X-05, ARCH-25 | observations reach the VLA at ≥ 10 Hz | the detector runs 2-9 Hz depending on host; a VLA decision takes 0.5-1.2 s on the desktop GPU (9.6 s for DroneCATS int4 on the Orin Nano) | G3 ruling (12 / 21 Oct, Q3) |
| ARCH-08 | AP_DDS migration (Q3, optional) | not done; MAVROS 2 is the bridge | deferred: one sentence in the report |
| ARCH-16, ARCH-17 | Gazebo Harmonic functional rail at mid-term | first flown end-to-end 2026-11-02, over pymavlink and over MAVROS 2 (dev topology) | late; state actual dates |
| ARCH-24 | 1 PI + 4 Master students | one developer (git log) | decided 2 Nov: the report states a one-person team |

## 8. Findings and corrections

The defects the project found in its own claims, each with what was
retracted and where: docs/FINDING-five-defects-found-by-audit.md,
docs/FINDING-the-kpi-was-never-measured.md,
docs/FINDING-the-kpi-report-that-scored-every-rule-as-p0.md, and the
`#### Retracted` entries of CHANGELOG.md. The correction notes for WP2 and
WP4 (WP2-12, WP4-21) were approved by the PI on 6 Oct and have been sent
to ITRI (confirmed by the researcher on 2 Nov).

## 9. Decisions needed from the PI

Settled on 2 Nov (the researcher, with the PI's standing instruction of
6 Oct to follow the grant and use best practice where it is silent):

- theta basis: forecast depth in metres, the shipped default (Q1);
- fail-safe rubric, IR bump on every shape change (WP1-22), two-hash
  contract (WP1-05), multi-scale geometry in the bundle (WP2-16), the DRIFT
  items: best practice, conforming to the grant rather than asking for
  waivers;
- Python: the version every environment supports, otherwise the best one
  with the reason recorded (WP1-25, X-06);
- the five final-demo scenarios: after the VLA and the system are mature;
- staffing (ARCH-24): stated as a one-person team;
- the WP2 / WP4 correction notes (WP2-12, WP4-21): sent to ITRI;
- `hil` fallback date: as soon as the AGX Orin is installed. The work
  stops at `hil`: the drone cannot carry an AGX Orin or run a large VLA, so
  the `flight` topology (ARCH-14) still needs the PI's written waiver.

Still open: the bundle signer and CA (Q5, to be confirmed); the public
repository against "code will not be published" (to be confirmed; nothing
is pushed meanwhile); the `flight` waiver above.

## 10. Plan to 2026-11-30

1. `hil`: Docker on the Jetson (user, sudo), then the runbook's bring-up and
   a first KPI-grade episode.
2. The sim items that need no PI ruling: corridor_swap over REST and the
   harness driving the events into a flight (WP1-15, WP4-03); the FSM
   edges not yet flown and G10 after a landing (WP3-07); Mission Planner on
   a PAS + ArduPilot run (ARCH-10); the VLA pilot over MAVROS 2 (ARCH-18,
   ARCH-20: needs the VLA server reachable from WSL); SROS 2 on one run
   (X-15). Done 29 Oct: REST hot-apply in flight (WP3-12, WP2-15), one
   flown zone per breach action (WP3-08).
3. Code: distance_envelope (WP1-07). Done 29 Oct: recoveries at the climb
   cap (WP3-17, library 4.308 to 3.135 s), the bundle changelog as a diff
   (X-10), the IR migration register (WP1-22, X-09), the WP2-16 deviation
   recorded.
4. The nightly `hil` campaign and the KPI report (WP4-01, WP4-17), the AGX
   Orin timings (ARCH-03, ARCH-30), then this report.

## Appendix A — obligations, evidence and what remains

Generated from the audit's status map (28 Oct) with the 28-29 Oct changes
applied; regenerate it before the report is written.

### WP1 - Policy DSL and IR

25 items: 12 DONE, 10 PARTIAL, 3 DRIFT.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP1-01 | YAML/JSON DSL surface with a Pydantic v2 discriminated-union schema | DRIFT | guardrail/models.py grant form; tests/test_policy_dsl_grant_form.py 60/60 (run 2026-10-28, vla-real env); CHANGELOG 2026-10-07 Added | ConstraintBase is looser than the grant's: constraint_type, priority and violation_action default (hard / P0 / 'repair', a seventh value) where the grant requires them; make them required or get a written waiver (independent review against the grant's wording, 29 Oct) |
| WP1-02 | Validation behaviour and rule-consistency lint (normalise geometry, lint alt_min<alt_max, conflicting hard rules, valid_time well-formed) | PARTIAL | CHANGELOG 2026-10-07 Changed (negative corpus 35/35 refused); guardrail/models.py:1346 lint, :1705 conflicting-hard-rule check; docs/data/wp1_roundtrip.json negative_refusal 35/35 | ring direction is checked but not normalised (the hash depends on vertex order); polygons cannot hold holes; the conflict lint covers altitude bands only (a hard corridor inside a hard NFZ loads clean) (independent review against the grant's wording, 29 Oct) |
| WP1-03 | IR / AST: a typed, indexed in-memory form with spatial index and multi-scale geometry, consumed by Compiler and Shield | DONE | guardrail/ir.py STRtree + coarse/fine geometry; tests/test_ir.py; CHANGELOG 2026-10-06/-07/-13 (check 0.30-0.45 ms at 50 rules, desktop) | Orin-side timing is tracked under X-07. |
| WP1-04 | Signed policy bundle (hash + semver) as the deployable artefact | PARTIAL | guardrail/bundle.py Ed25519 (lab-dev key), 64-hex hash, policies/policy.lock.json; every rail incl. the city rail flies the signed bundle by default since 2026-10-22 (Jetson metrics policy_source kind=bundle) | Signing authority (lab CA vs ITRI CA) not named; swap the key in policies/keys/trusted_signers.json once named. |
| WP1-05 | Bundle compatibility with the PI's reference implementation (a byte-identical match, claimed and retracted 7 Oct) | PARTIAL | docs/data/wp1_roundtrip.json: both hashes 29/29, reference loader reads ours 3/3, forged ref hash refused 29/29; the byte-identity claim retracted (CHANGELOG 2026-10-07) | Reference DSL can represent 0/29 shipped policies; PI to accept the two-hash contract (12 Oct Q8, unanswered) or a converter. |
| WP1-06 | GeoFence classes: polygon_fence and circle_fence | DONE | circle_fence enforced (CHANGELOG 2026-10-07); sitl/fence/ export; PAS rail uploads clock-proof zones (CHANGELOG 2026-10-13, FENCE_TYPE 7); tests/test_railinputs_fence_export.py; ros2fix2_off_fence fence fired | None. |
| WP1-07 | Envelopes: altitude, kinematic, and distance (minimum distance to typed object classes) | PARTIAL | guardrail/models.py:13-37: distance_envelope declarable only (nothing enforces it), MSL refused for flight (models.py:1424); obstacle_clearance/subject_standoff are our enforced equivalents | Enforce distance_envelope (map to clearance/standoff runtime, add roads class) and give MSL a ground-elevation source. |
| WP1-08 | 3-D corridor (keep-in tube) | DONE | corridor implemented, tests/test_corridor.py (audit 5 Oct DONE) | Never flown on any rail; optional to fly corridor_survey once. |
| WP1-09 | Time windows: valid_time recurrence on rules plus the time_window_switch event | DONE | valid_time + time_window_switch: guardrail/shield.py events, guardrail/compiler.py _RuleClock, templates; rule clock on all rails (CHANGELOG 2026-10-13); tests/test_shield_events.py, tests/test_railinputs_clock.py | No flight has declared a switch in a policy file; flight evidence belongs to WP1-15/WP4-03. |
| WP1-10 | Breach actions: violation_action values monitor_only/project_fix/brake/loiter/RTL/land | DONE | guardrail/shield.py dispatch on violation_action + guardrail/fsm.py SET_MODE; tests/test_fsm.py 114/114 (run 2026-10-28); RTL flown via X7 on pas_ap_close_on (13 Oct) and pas_ap_video_on_20261014 | Per-action flight evidence is WP3-08. |
| WP1-11 | constraint_type hard/soft | DONE | guardrail/shield.py soft rules capped at brake, relaxed lowest priority first (CHANGELOG 2026-10-07); tests/test_shield.py | PI Q9 (12 Oct) on soft semantics unanswered; no shipped policy uses a soft rule in flight. |
| WP1-12 | priority P0/P1/P2 | DONE | guardrail/shield.py relaxation by priority; escalation uses priority (CHANGELOG 2026-10-07) | Priority does not arbitrate between two hard rules (documented). |
| WP1-13 | scope + layer fields and the layered merge regulation>site>mission | DONE | guardrail/models.py merge_layers/load_layered, locked_by (CHANGELOG 2026-10-07, 2026-10-19); tests/test_policy_dsl_grant_form.py | No flown run has used a layered policy. |
| WP1-14 | Units and frames: WGS84 lat/lon canonical, AGL default, ENU/NED computed lazily | DRIFT | WGS84 canonical stored form (CHANGELOG 2026-10-07 Changed; PI-DECISIONS-2026-10-06 Part C deviation 1) | 31 of 32 policies are written and hashed in local metres, the WGS84 one has never flown, projection runs at load not lazily, MSL cannot fly (independent review against the grant's wording, 29 Oct) |
| WP1-15 | Mid-flight hot-apply (dynamic_nfz, time_window_switch, corridor_swap) with generation bump and hash re-derive, via a REST endpoint | PARTIAL | guardrail/api.py REST + --hotapply-port on both SITL rails with mTLS (CHANGELOG 2026-10-13); tests/test_api.py, tests/test_railinputs_hotapply.py; 'Nothing here has flown'; 29 Oct: REST hot-apply into flying ArduPilot SITL + MAVROS 2 (ROS 2 rail, dev): a dynamic_nfz spawned, rotated, translated and switched off (time_window_switch) mid-flight, generations 0-4, receipt to swap 0.444-4.443 ms against 50 ms; on ros2_hotapply_03 the Shield and the FSM stopped the aircraft short of the REST zone (X1, G3), released it after the switch-off (G8), and it reached the target | corridor_swap over REST in flight (the flown policy has no corridor); a listener on the perception rail; lift the dynamic_nfz file hold (C-16). |
| WP1-16 | Ingest outputs: JSON Schema export, KML/GeoJSON converter, ingest CLI | DONE | policies/policy_dsl.schema.json; tools/geo_to_policy.py; tests/test_geo_to_policy.py | Polygons with holes refused (minor). |
| WP1-17 | WP1 KPI: policy load round-trip | DONE | tools/wp1_roundtrip_kpi.py -> docs/data/wp1_roundtrip.json: round-trip 29/29, tamper 29/29, negative 35/35, nulls 0/29; tests/test_wp1_roundtrip_kpi.py | None. |
| WP1-18 | WP1 KPI: bundle replayability (load the bundle whose policy_hash matches, never 'the latest') | PARTIAL | replay bundle per run (e.g. nfz3b_pc_hier_dronecats_20261028.replay.tar.gz); headless re-fly bit-for-bit (tools/rerun_episode.py); but all 45 flights of 27-28 Oct are code_revision '-dirty' (PC) or 'unversioned' (Jetson) in manifest.json | no hil run exists: the KPI gate (is_kpi_grade) refuses every stored run; fly KPI runs from a clean checkout on the Orin (git, not git archive) and keep their bundles in the tracked store. |
| WP1-19 | Policy DSL acceptance KPIs: schema-validation pass rate 100%, golden cases 100%, cold-start 50-rule ≤1 s, hot-apply ≤50 ms, monitor query ≤5 ms at 10 Hz with 50 rules | PARTIAL | desktop: cold start p95 376 ms (docs/data/wp1_roundtrip.json), _check 0.45-0.87 ms at 50 rules; Orin Nano city flights: Shield call p50 2.5 ms / p95 6-9 ms (small city policy, 3 s/0.5 s forecast) - not the 50-rule x 50-pose load | Run tools/profile_shield_tick.py + the cold-start bench on the Orin at 50 rules x 50 poses; publish a hot-apply <=50 ms number (none in docs/data). |
| WP1-20 | Policies used in the CityLife city flights are bundled and replayable | PARTIAL | city rail flies signed bundles by default (CHANGELOG 2026-10-22; nfz3b bundle signed and deployed to the Jetson, CHANGELOG 2026-10-28 later); replay bundles in each run folder | Copy the deliverable CityLife replay bundles into tracked bundles/ (only 4 retarget replays there); fly deliverable runs from a clean tree (27-28 Oct runs are -dirty/unversioned). |
| WP1-21 | Mid-term gate (2026-07-20): Policy DSL + IR shipped | DRIFT | guardrail/ir.py delivered 2026-10-06 (after the 2026-07-20 gate); DSL since August | dated gate missed: the IR was first committed 2026-10-06, the DSL has no evidence before 2026-08-24; the final report states the actual dates (independent review against the grant's wording, 29 Oct) |
| WP1-22 | R6 freeze gate: freeze the IR shape at end of Q1, and give every IR-shape change a semver bump and migration note (the Policy DSL page cites 'R6 ... for the freeze gate at the end of Q1') | PARTIAL | sha256-canonical-v2 + policies/policy.lock.json + docs/DESIGN-policy-identity.md (CHANGELOG 2026-10-06); generated policies get unique ids (CHANGELOG 2026-10-22, 'WP1-22'); 29 Oct: the migration-note register docs/IR-MIGRATIONS.md (7c02284), one row per IR-shape change since the freeze | One change went unbumped (2385f84, locked_by, additive): the PI accepts the additive rule, or the next release raises IR_SCHEMA_VERSION to 2.2.0. |
| WP1-23 | Single IR contract across the deployed system (Overview: Compiler and Shield 'never re-parse the YAML / JSON source') | DONE | one bundle feeds Shield and VLA nodes (sitl/run_ros2_demo.sh, --bundle); city rail too since 2026-10-22 | None. |
| WP1-24 | policy_hash on every emitted artefact (Overview: CSP, Shield repair log entry, episode bundle) | DONE | audit records carry policy_hash+generation (guardrail/audit.py AuditRecord), csp_g<N>.json on ROS 2 + PAS rails, manifest policy_hash on every run | City-rail flights write no CSP (see WP2-11). |
| WP1-25 | Locked design choice: Python 3.11+ with Pydantic v2 | PARTIAL | pyproject.toml requires-python >=3.11, guardrail tests pass on 3.11; flight env vla-real is 3.10.20, the Jetson runs Python 3.10.12 (jetson_facts_20261028.json) | Run the flight stack on 3.11+ (the Orin container plan) or get the deviation accepted in writing (PI said follow the grant). |

### WP2 - Prefix Compiler (CSP)

19 items: 13 DONE, 6 PARTIAL.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP2-01 | CSP injected into the VLA / planner prefix (the WP2 output artefact itself) | PARTIAL | demo/real_vla_demo.py --csp on/off (CHANGELOG 2026-10-06, 'Not yet flown'); demo/vla_fly.py has no CSP path; AerialVLA prompt is upstream's, unchanged (guardrail/aerialvla_codec.py); none of the 45 VLA flights of 27-28 Oct carried a CSP | Fly a text-reading VLA (OpenVLA via real_vla_demo, or a VLA fine-tuned to read the CSP) with the CSP in the prompt; needs the G3 ruling on a ~2 Hz pilot. |
| WP2-02 | Grant KPI: prefix-token budget (configurable budget, respected; CSPBudgetExceeded on P0 overflow) | DONE | guardrail/csp.py budget + CSPBudgetExceeded; docs/data/csp_kpis.json; tests/test_csp.py 99/99 (run 2026-10-28) | Measured offline only. |
| WP2-03 | Grant KPI: CSP coverage of policy IR (spec: P0 coverage = 100% across test fixtures) | DONE | docs/data/csp_kpis.json: at 120 tokens compiler keeps 105/105 P0 vs naive 104/105, null 0/105 (CHANGELOG 2026-10-13); tests/test_csp.py | None. |
| WP2-04 | CSP schema: Pydantic v2 model with the locked field set + csp.schema.json export | DONE | guardrail/csp.py typed model, guardrail/csp.schema.json; test_the_committed_schema_has_not_drifted_from_the_model; 29 Oct: policy_id and selection made optional so the spec's own worked CSP validates against the model and csp.schema.json (tests/test_csp.py::test_the_specs_own_worked_example_validates); the flight environment's Python 3.10 stays WP2-19 | None. |
| WP2-05 | Pipeline: spatial + temporal filter on mission context, risk-grading (0.5 sev + 0.3 prox + 0.2 tcrit), order + truncate (P0 kept, P1/P2 by budget) | PARTIAL | guardrail/csp.py time/region filter, risk 0.5/0.3/0.2, P0 kept, budget cut (CHANGELOG 2026-10-06); tests/test_csp.py | the mission context has no home, target or allowed modes; the risk weights are the grant's placeholders, never tuned; no flown VLA has read a CSP (independent review against the grant's wording, 29 Oct) |
| WP2-06 | Mission-context relevance explainable: relevance_explanations[rule_id] for every rule | DONE | guardrail/csp.py relevance_explanations; test_kpi_every_rule_in_the_csp_has_a_relevance_explanation | None. |
| WP2-07 | Per-consumer adapters: action-set adapter (body-frame vx/vy/vz/yaw_rate ranges + forbidden polygons) and cost-map adapter (YAML) | DONE | action-set adapter (guardrail/csp.py) + cost-map adapter (compiler.rasterise_cost_map, docs/DESIGN-cost-map-adapter.md, CHANGELOG 2026-10-12); 29 Oct: CostMap.save also writes the header as <stem>.yaml (the grant's 'emits YAML'), and load_cost_map reads either | No planner reads the cost map (by design). |
| WP2-08 | Templated NL adapter (Jinja2 + Python), canonical source for the paraphraser | DONE | guardrail/templates/ Jinja2 incl. event templates; test_every_rule_type_has_its_jinja2_templates | No flight has carried the sentences to a VLA (WP2-01). |
| WP2-09 | Effectiveness evaluation CLI: offline replay (violating-candidate rate before/after, repair magnitude, token utilisation) + A/B prefix on/off in stress runs | PARTIAL | tools/prefix_eval.py + harness prefix on/off key (CHANGELOG 2026-10-07, 'not yet run on the model'); tests/test_prefix_eval.py | Run prefix_eval on logged VLA tuples with and without the CSP; needs WP2-01 first. |
| WP2-10 | CSP carries policy_hash + generation; stateless/deterministic | DONE | guardrail/csp.py policy_hash, generation, issued_at; test_the_csp_hash_is_stable_across_processes_and_hash_seeds | None. |
| WP2-11 | CSP emitted with each flight's artefacts (policy_hash-everywhere invariant) | DONE | demo/follow_vlm.py city_episode: every city follow writes events.jsonl, policy_g0.json and csp_g0.json (f6ee433); tests/test_city_episode.py 3/3 | - |
| WP2-12 | Claims made about WP2 in delivered reports vs what the code does | DONE | tools/check_claims.py (0 live claims in tracked files); docs/CORRECTION-2026-10-06-mid-evaluation-and-deck.md row A5; the correction note was sent to ITRI (researcher, 2 Nov) | - |
| WP2-13 | Paraphraser (Q3 Aug-Oct deliverable, separate service fed by the templated NL); per-paraphrase robustness is WP4's KPI | DONE | Since 2 Nov a separate service: guardrail/paraphraser_service.py, gRPC (guardrail/protos/paraphraser.proto: Paraphrase, Validate, GetStatus) with mutual TLS from the control plane's lab CA, plaintext on loopback only; templated NL in, paraphrases out, each passed by the validator, with the validator and store digests; tests/test_paraphraser_service.py 8/8 (vla-real). guardrail/paraphraser.py with 112 stored paraphrases, the 4 sets stale since C8 re-authored 2 Nov (`stale`: 0 of 21 texts without a set). Validator: the 7 mechanisms of heldout_2026_10_28's misses fixed (2/20 -> 1/20 false accepts, 5/20 -> 0/20 false refusals; a dev set since); a new set, heldout_2026_11_02 (94 probes, written and committed before the validator was read), measured 1/50 and 18/44 before, 1/50 and 14/44 after those fixes (the held-out figure), 0/50 and 3/44 after its own mechanisms were fixed (dev); mutants 3977/3977 refused (CHANGELOG 2026-11-02) | The stress harness (the orchestrator) still draws paraphrases in-process (WP4-10); no held-out estimate exists for the validator as it stands (heldout_2026_11_02 was tuned on, and its author also fixed the validator); a person has yet to read the stored set; the KPI use is WP4-09. |
| WP2-14 | Tests for the prefix compiler acceptance KPIs | DONE | tests/test_csp.py test_kpi_budget_*, test_kpi_p0_coverage_*, test_kpi_p0_overflow_raises_*, test_kpi_every_rule_* (99/99) | None. |
| WP2-15 | CSP re-issued when the policy generation changes (hot-apply). Grant: CSP is a function of 'policy bundle ( policy_hash + generation )'; Overview: 'policy_hash everywhere' | DONE | sitl/ros2_shield_node.py writes csp_g<N>.json per generation; dynamic_nfz compiles (CHANGELOG 2026-10-12); stored ros2fix_on_dyn_nfz (7 Oct) lacks generation 1's CSP; not re-flown; 29 Oct: ros2_hotapply_01-03 wrote csp_g0..csp_g4, one per generation; g0 does not name the REST zone, g1-g4 do, each with its generation's policy_hash | KPI-grade needs hil. |
| WP2-16 | Locked design choice: multi-scale geometry - 'The Prefix Compiler picks the scale, never re-computes' (P0Entry.geometry_ref, e.g. 'coarse:...@v0.3.0/g0') | PARTIAL | guardrail/ir.py multiscale geometry + geometry_ref in the CSP; computed at load, the bundle does not carry the polygons; the compiler reads the scales from guardrail/ir.py and never re-computes them (since 8 Oct); recorded deviation (outline section 7): the scales are derived at load from the IR, not shipped in the bundle | The PI accepts the derived-at-load reading, or the bundle ships the scales. |
| WP2-17 | Locked design choice: the CSP's action sets are in body frame (vx, vy, vz, yaw_rate); the Shield converts to local-NED before MAVLink emit | DONE | body-frame actions at the ROS 2 boundary, conversion in sitl/mavlink_adapter_node.py (CHANGELOG 2026-10-07); tests/test_mavlink_adapter.py | City rail's StepTracker emits world-frame actions (not a grant rail). |
| WP2-18 | Final-delivery scope: 'Prefix Compiler (full)' scheduled Q2 May-Jul (Overview.txt:170); final gate requires 'Prefix Compiler ... complete' (Grant overview.txt:102-103) | PARTIAL | every compiler part built and tested (WP2-02..08, 10, 14); effect never shown: no VLA has read a CSP in flight | Close WP2-01 + WP2-09 (CSP on/off delta) for the final 'Prefix Compiler complete' gate. |
| WP2-19 | Locked design choice: 'Python 3.11+ with Pydantic v2 for the CSP schema' | PARTIAL | same as WP1-25: package 3.11+, flight runtime 3.10 | Same as WP1-25. |

### WP3 - Safety Shield

24 items: 7 DONE, 14 PARTIAL, 3 DRIFT.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP3-01 | Mid-term (2026-07-20): Safety Shield ROS 2 node skeleton with at least one projection operator | DRIFT | ROS 2 Shield node with projection operators (audit 5 Oct DONE; sitl/ros2_shield_node.py) | dated gate missed: the first ROS 2 Shield node is from 2026-08-19/24, and the Gazebo half of the gate was first flown on 2026-11-02 (ARCH-16) (independent review against the grant's wording, 29 Oct) |
| WP3-02 | ROS 2 node: Monitor / Filter / Projection; violation checker reuses the policy IR; bounded repair loop (default 3) | DONE | monitor/filter/projection, bounded repair, Shield.history, Shield.rule_status (CHANGELOG 2026-10-06/-07); tests/test_shield.py | None. |
| WP3-03 | VLA->Shield on a 10 Hz ROS 2 topic (Float32MultiArray); Shield->MAVROS 2 on a 10 Hz topic (mavros_msgs/PositionTarget) | DONE | sitl/mavlink_adapter_node.py publishes mavros_msgs/PositionTarget; sitl/ros2_ws packages; tests/test_mavlink_adapter.py | None. |
| WP3-04 | Action input frame is body frame (vx, vy, vz, yaw_rate); body->local-NED conversion happens after projection, in the MAVLink adapter | DRIFT | body frame end to end at the ROS 2 boundary (CHANGELOG 2026-10-07; demo/out/ros2fix_on_dyn_yaw) | the repair operators act in a world (North/East) frame: the Shield node rotates body->world before filter() and back after, and the conversion is in several modules, not one adapter boundary (independent review against the grant's wording, 29 Oct) |
| WP3-05 | Lookahead horizon 5 s at 10 Hz = 50 future poses; time-window boundary checked at t-ε and t+ε | PARTIAL | guardrail.shield LOOKAHEAD_S 5 s / LOOKAHEAD_DT_S 0.1 s on ROS 2 + PAS rails and the harness (CHANGELOG 2026-10-07, -13); window-edge checks | the city rail (every 27-29 Oct flight) runs a 3 s lookahead at 0.5 s, not the locked 5 s at 10 Hz (independent review against the grant's wording, 29 Oct) |
| WP3-06 | Repair operators in priority order: AltitudeClamp -> LateralProjection (SDF) -> PathRepair; conservative cap θ (2.0 m lateral / 0.5 m vertical) above which projection is abandoned for fail-safe; pluggable Protocol | PARTIAL | guardrail/repair.py RepairOperator Protocol, PathRepair (off by default), theta applied with escalation (CHANGELOG 2026-10-07, -13); tests/test_shieldcore_repair.py | PI ruling on the theta basis (12/21 Oct Q1, unanswered); decide whether PathRepair goes on by default. |
| WP3-07 | Escalation FSM Normal -> Brake -> Loiter -> RTL -> Land with N-in-T (3 in 5 s), θ, T_recover = 2 s, YAML-overridable; drives SET_MODE GUIDED/LOITER/RTL/LAND | PARTIAL | guardrail/fsm.py (tests/test_fsm.py 114/114), wired to Shield and rails; flown on PAS+ArduPilot: Brake, Loiter (as BRAKE), RTL via X7 (pas_ap_close_on 13 Oct, pas_ap_video_on_20261014, pas_ap_ovf_on_20261021); hold points headless only; 29 Oct, ROS 2 rail + MAVROS: edges G1, G3, G5, G8, G9, G11, G12, X1, X4 flown (ros2_hotapply_01-03, ros2_action_*_01); Land reached directly (G12) | Fly G2, G4, G6, G7, G10, X2, X3, X5-X7; G10 never fired after a landing (ros2_action_land_01, open); a moved hold point (C-03); PI rulings (LOITER-as-BRAKE, N-in-T for slow pilots, rubric). |
| WP3-08 | Shield acts on the rule's violation_action (monitor_only, project_fix, brake, loiter, RTL, land); hard vs soft constraints; P0/P1/P2 priority | DONE | dispatch on all six actions in guardrail/shield.py; experiments/templates/failsafe_escalation.yaml headless; no flown policy uses loiter/RTL/land/monitor_only actions; 29 Oct, ROS 2 rail + MAVROS, one REST-spawned zone per action: land (G12 -> LAND), RTL (G11), loiter (X4), brake (X1, held), project_fix (ros2_hotapply_03), monitor_only as a soft P2 rule (flown through, P0 0.0) and as a hard P0 rule (counted as escapes, by design) | One zone per action on the flown policy, not one policy file per action; KPI-grade needs hil. |
| WP3-09 | MAVLink adapter ROS 2 node: single body->local-NED boundary; SET_POSITION_TARGET_LOCAL_NED; MISSION_ITEM_INT; SET_MODE for FSM | PARTIAL | sitl/mavlink_adapter_node.py, SET_MODE via ModeRequests; flown 7 Oct (ros2fix2_*) | the adapter has no MISSION_ITEM_INT waypoint replace/update path; LOITER is sent as BRAKE; the PAS rail bypasses the adapter (independent review against the grant's wording, 29 Oct) |
| WP3-10 | ArduPilot GeoFence (FENCE_ENABLE, FENCE_TYPE, FENCE_ACTION) configured as the last-line backstop that fires only if the Shield fails | PARTIAL | sitl/fence/guardrail_fence.parm; fence fired with the Shield off (ros2fix2_off_fence); PAS rail uploads zones before arming (CHANGELOG 2026-10-13), 0 breaches | a Shield-on run (ros2fix_on_policy55) logged 'Fence Breached' during the Shield's own RTL, unexplained; the fence parameters come from a SITL defaults file and the polygons from the Shield's own adapter, not a GCS (independent review against the grant's wording, 29 Oct) |
| WP3-11 | Per-violation-tick JSONL audit record (Pydantic v2) with ts, policy_hash, generation, monitor_tick, raw_action, violations (boundary_intersect_at_s), repair_attempts (result/magnitude_m/iterations), emitted_action, fsm_state_before/after; rotated per mission | DONE | 2 Nov, audit record version 4 (guardrail/audit.py AuditRecord, Pydantic v2; policies/audit_record.schema.json): the grant's ten fields written first, in the grant's order, with the grant's content - raw_action / emitted_action body frame {vx, vy, vz, yaw_rate} (frames.to_body at the tick's heading, converted when the record is written), a violation's category geometric / envelope / time-window with boundary_intersect_at_s, repair_attempts the operators tried (ok / skip with its reason, magnitude_m, iterations = the pass); the grant's p5 example validates verbatim; a world-frame action under the grant's name is refused; this project's content under raw_action_world / emitted_action_world, rule_category, repair_results; every rail writes shield_audit.jsonl, one file per episode (a stale one moved aside); every stored record (294 files, 71,064 records, versions 1-3) reads through upgrade_record; tests/test_shieldcore_audit.py 16/16 | None for the record. The Shield still repairs in a world frame (WP3-04): the body frame is a conversion at write time. `contract` (a non-finite command zeroed) is a fourth category value, this project's extension. No flown run has written a version-4 record yet (the rails write it in their tests; the next flight writes it). |
| WP3-12 | Hot-apply listener (REST callback) bumps policy generation mid-flight | DONE | --hotapply-port on sitl/ros2_shield_node.py and sitl/run_sitl_demo.py (CHANGELOG 2026-10-13); perception_node has no listener; never driven during a flight; 29 Oct: REST hot-apply into flying ArduPilot SITL + MAVROS 2 (ROS 2 rail, dev): a dynamic_nfz spawned, rotated, translated and switched off (time_window_switch) mid-flight, generations 0-4, receipt to swap 0.444-4.443 ms against 50 ms; on ros2_hotapply_03 the Shield and the FSM stopped the aircraft short of the REST zone (X1, G3), released it after the switch-off (G8), and it reached the target | KPI-grade needs hil. |
| WP3-13 | Monitor tick budget ≤100 ms end-to-end at 10 Hz; geometry query budget ≤5 ms with ~50 rules x 50 poses | PARTIAL | Orin Nano city flights (D:/vla_drone_offload/jetson_runs/orin_follow_02, orin_hier_dronecats_20261028): tick work p50 22 ms / p95 61-63 ms, 15-25 of ~4200 ticks over 100 ms (max 277 ms); Shield p50 2.5 / p95 6-9 ms; desktop _check 0.45-0.87 ms at 50 rules | Profile 50 rules x 50 poses on the Orin with tools/profile_shield_tick.py; explain the >100 ms ticks; log VLA-to-setpoint latency on a hil run. |
| WP3-14 | P0 violation escape rate, target 0 (hard limit), on the canonical KPI configuration | PARTIAL | 0 P0 escapes: harness library 179 Shield-on episodes (CHANGELOG 2026-10-22), every kpi.json of the 45 flights 27-28 Oct, PAS+ArduPilot Shield-on flights 12-21 Oct; none KPI-grade | no hil run exists: the KPI gate (is_kpi_grade) refuses every stored run; ROS 2/MAVROS rail not re-flown since 7 Oct. Run the hil campaign. |
| WP3-15 | Fail-safe trigger correctness (reference target ≥99%: triggered when expected, not when not expected) | PARTIAL | guardrail/fsm.py score_failsafe_triggers with nulls; docs/data/scenario_sweep.json (22 Oct): library 144/170 = 0.847, nightly 544/714 = 0.762, never-trigger null 0.941 (CHANGELOG 2026-10-22, PI-DECISIONS-2026-10-21 #2) | Below the locked >=0.99 and below its null. Needs PI rubric + theta basis (Q1/Q2, unanswered since 12 Oct), then fix the X1>G3>G5 / G1>G3>G4 false triggers and re-score; then KPI-grade on hil. |
| WP3-16 | Mean repair magnitude (mean //post_repair − planner_output//) | PARTIAL | tools/kpi_report.py repair_magnitude_by_priority / _by_operator_class (CHANGELOG 2026-10-13); values only headless or non-grant rails | no hil run exists: the KPI gate (is_kpi_grade) refuses every stored run. |
| WP3-17 | Mean time to safe: from violation to back-in-safe-zone (reference sweep expects ≤2.0 s) | PARTIAL | 29 Oct: a recovery from outside an altitude band flies at the climb cap (the Shield's recovery_law 'cap', 5c98142); the republished library (docs/data/scenario_sweep.json): mean time to safe 3.135 s over 37 recoveries, max 6.7 s (4.308 s and 11.9 s on the old law); 8 Oct run 2.81 s; none KPI-grade | Still above the reference 2.0 s, a figure the grant gives for a dynamic-NFZ sweep; measure on hil. |
| WP3-18 | Canonical KPI configuration 'hil': desktop runs sims + MAVROS + GCS, Jetson Orin runs VLA + Shield over the network | PARTIAL | desktop rail labelled dev, is_kpi_grade refuses dev (CHANGELOG 2026-10-06/-07) | no hil run exists (0 manifests); only the refusal guard; five August folders still store topology canonical-hil with kpi_grade true (independent review against the grant's wording, 29 Oct) |
| WP3-19 | Smoke test matrix: high-speed near-edge, sudden NFZ (hot-applied), time-window switch instant, three simultaneous violations, GPS noise/latency | PARTIAL | experiments/profiles/smoke.yaml covers all five rows (CHANGELOG 2026-10-06/-07); tests/test_sweep.py | the smoke matrix has never run on a release or a build (CI last ran at 1b95383, 14 Oct); the smoke set is headless, not the grant's KPI-grade smoke (independent review against the grant's wording, 29 Oct) |
| WP3-20 | Repair success rate (Safety Shield page, Acceptance KPIs (locked): 'tracked; informs Prefix Compiler effectiveness eval') | PARTIAL | guardrail/kpi.py repair_success with theta (CHANGELOG 2026-10-06); 0.776 on the 8 Oct pymavlink run; no KPI-grade value | Theta basis (PI Q1) and a hil value. |
| WP3-21 | Single IR contract: the Shield consumes the Policy IR and never re-parses the YAML/JSON source ('hard invariant; violating it ... breaks reproducibility', Overview cross-cutting invariants) | DONE | every rail flies the signed bundle; YAML only under --dev-yaml (demo/follow_vlm.py:122-193; CHANGELOG 2026-10-22); Jetson metrics policy_source kind bundle, hash verified | None. |
| WP3-22 | Exactly one rule-evaluation code path in the system ('Violation checking' section; violation checker library shares the IR) | DRIFT | guardrail.replay.zone_dwell reads Shield.rule_status on every rail (CHANGELOG 2026-10-12) | more than one rule-evaluation path: compiler._RuleClock and action_violations, fence_export, rtl_path and FenceGuard keep their own geometry or clocks (independent review against the grant's wording, 29 Oct) |
| WP3-23 | Shield node architecture: sliding-window buffer (last 50 actions + predicted trajectory) and geometry queries on the IR's R-tree spatial index | PARTIAL | Shield.history (50 ticks) + guardrail/ir.py STRtree, anchor_reach (CHANGELOG 2026-10-06, -13); tests/test_ir.py | the predicted trajectory is not buffered, the buffer is a record and not the checker's input, and moving dynamic_nfz are outside the STRtree (independent review against the grant's wording, 29 Oct) |
| WP3-24 | Safety Shield ROS 2 node delivered as a 'Python package; rclpy' (Outputs table) | DONE | sitl/ros2_ws/ rclpy packages + launch files; tests/test_ros2_package.py | None. |

### WP4 - Stress Testing

21 items: 4 DONE, 17 PARTIAL.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| WP4-01 | Every reported KPI must come from a Stress Testing run in the canonical HIL topology | PARTIAL | dev label + KPI gate refuse non-hil runs; Orin Nano Super connected (nathan@10.42.0.2) but only the city follow controller ran on it (CHANGELOG 2026-10-27 Jetson); harness never run through SITL/MAVROS; docker images never built | no hil run exists: the KPI gate (is_kpi_grade) refuses every stored run. Bring up the hil rail (SITL+MAVROS on desktop, Shield+pilot on Orin), run nightly, kpi_report. PI never answered the dated fallback (21 Oct #4). |
| WP4-02 | Scenario library: declarative YAML (one file per template) validated by Pydantic ScenarioSpec, with templates dynamic_nfz_movement / time_window_switch / radius_scaling / corridor_swap, events, parameter_sweep, expected_kpis | DONE | guardrail/scenario_spec.py ScenarioSpec/ScenarioEvent, experiments/templates/ 16 files, parameter sweeps; tests/test_scenario_spec.py | Headless only (WP4-04). |
| WP4-03 | Harness drives dynamic_nfz, time_window_switch and corridor_swap mid-flight via the Policy DSL REST hot-apply endpoint; events spawn/translate/rotate fence, activate/deactivate rule, swap corridor, scale radius | PARTIAL | every event type mid-flight in the headless harness; gRPC HotApply forwards to a bound REST endpoint (CHANGELOG 2026-10-13); never through a flying rail; 29 Oct: REST hot-apply into flying ArduPilot SITL + MAVROS 2 (ROS 2 rail, dev): a dynamic_nfz spawned, rotated, translated and switched off (time_window_switch) mid-flight, generations 0-4, receipt to swap 0.444-4.443 ms against 50 ms; on ros2_hotapply_03 the Shield and the FSM stopped the aircraft short of the REST zone (X1, G3), released it after the switch-off (G8), and it reached the target | Drive the events from the harness (gRPC HotApply), not a scripted client; corridor_swap; KPI-grade needs hil. |
| WP4-04 | Harness runs scenarios on the simulation rails (Gazebo Harmonic functional sweeps, KPI-grade runs in HIL) and emits an episode bundle per episode | PARTIAL | headless harness + modelled ArduPilot response arm (CHANGELOG 2026-10-21); --backend sitl scores rail files but flies stub missions without events; no stored sitl-backend run | Run the scenario library through ArduPilot SITL (+Gazebo for functional sweeps) and on hil, with an episode bundle each. |
| WP4-05 | RPC harness: Python service + gRPC stubs over mTLS (StartSession, InjectPolicy, HotApply, StartEpisode, IssueTask, EmitEvent, EndEpisode, streams) | PARTIAL | guardrail/harness_service.py + guardrail/protos/harness.proto, all 10 RPCs over mTLS; headless only; tests/test_harness_service.py 57/57 (2 Nov). The bundle carries the grant's shield_audit.jsonl (30 Oct) and policy_bundle.tar.gz (2 Nov: the injected file byte for byte, generation 0's and the last generation's IR, every generation by hash, an index with each member's SHA-256; write_policy_bundle); kpi.json harness.bundle_files names the grant's three files and which are written | Drive a simulator/hil episode through it. episode.bag2/ is not written: the headless backend has no ROS 2 graph (kpi.json says so), and a ROS 2 rail run records its bag as bag/, not episode.bag2/. With two or more hot-applies the generations between the first and the last are in policy_bundle.tar.gz by hash only (the headless runner keeps no copy; the index says incomplete). |
| WP4-06 | Smoke (~50 scenarios x 1 seed per build), Nightly KPI (~200 x 3 seeds, HIL, sim_speedup 1.0), Broad sweep (~3000 x 5); sweep config YAML per profile; CI smoke pipeline | PARTIAL | smoke/nightly/broad profiles defined; smoke + nightly run headless (CHANGELOG 2026-10-22); master is 214 commits ahead of myfork/master, so CI has not run since ~13 Oct; nightly.yml cannot fire from master | Push (or set the default branch) so CI smoke runs per build; nightly at sim_speedup 1.0 on hil; broad never run. |
| WP4-07 | KPI auto-report generator (Markdown + JSON): per-scenario-family stats table (episode count, P0 escape rate, fail-safe correctness, mission success rate, mean repair count, mean repair magnitude) + top-K (K=10) failure cases linked to episode bundles | DONE | tools/kpi_report.py family table, top-K, Markdown+JSON, --baseline (CHANGELOG 2026-10-06, -13); tests/test_kpi_report.py, tests/test_wp4outputs_kpi_report.py | Needs KPI-grade input (WP4-01). |
| WP4-08 | Mission success rate, tracked, defined as outcome == success AND no P0 violation, rolled up per scenario family | PARTIAL | mission_success_rate 0.603 headless library (CHANGELOG 2026-10-22); follow flights scored by within-30 m | no hil run exists: the KPI gate (is_kpi_grade) refuses every stored run. |
| WP4-09 | Per-paraphrase robustness, a WP4-owned KPI: mission outcome stability across paraphrased task prompts | PARTIAL | paraphrase arms in the harness (CHANGELOG 2026-10-07); no text-reading pilot has flown them (AerialVLA reads a fixed phrase vocabulary) | Pick the KPI pilot (G3 ruling) and fly each family x paraphrases; record outcome stability. |
| WP4-10 | Paraphraser: a separate service (Q3, Aug-Oct deliverable) that turns the Prefix Compiler's templated NL into free-form paraphrases issued to the VLA | PARTIAL | same artefact as WP2-13: since 2 Nov a separate gRPC + mTLS service (guardrail/paraphraser_service.py) serving validated paraphrases of the templated NL; validator figures as WP2-13 (heldout_2026_11_02: 1/50 false accepts, 14/44 false refusals held out; 0/50, 3/44 dev) | Nothing issues the service's paraphrases to a VLA: the stress harness, the reference design's orchestrator, still draws them in-process (guardrail/harness_service.py imports guardrail.paraphraser; binding it to the service is a change there), and no text-reading pilot has flown them (WP4-09). |
| WP4-11 | Every episode produces a self-contained bundle replayable bit-for-bit: manifest.json, policy_bundle (incl. all hot-apply generations), episode.bag2 (rosbag2), shield_audit.jsonl, harness_events.jsonl, kpi.json; Q4 reports Q3 runs 'replayed for reproducibility' | PARTIAL | headless episodes re-fly bit-for-bit (tools/rerun_episode.py, CHANGELOG 2026-10-13); ROS 2 bundles hold rosbag2/audit/events; simulator/SITL episodes re-score only | No KPI-grade episode bundle; Q4 replay-for-reproducibility of Q3 runs not done. |
| WP4-12 | Six-field manifest per episode (code_revision, vla_model_hash, policy_hash, random_seed, sim_speedup, topology); sim_speedup=1.0 mandatory for KPI-bearing runs | PARTIAL | six-field manifest on every run (guardrail/manifest.py); but 27-28 Oct manifests read code_revision '...-dirty' (PC, e.g. demo/out/nfz3b_pc_hier_dronecats_20261028) and 'unversioned' (Jetson, git-archive sync), and Jetson runs say topology 'projectairsim-single-host'; since 28 Oct the Jetson is a git checkout (deploy/orin/sync_follow_controller.ps1 ships a git bundle), so code_revision resolves there; a remote simulator is stamped projectairsim-remote-sim (guardrail/manifest.py) | Fly the deliverable runs from a clean tree. The untracked file that made every run since 27 Oct '-dirty' was committed on 2 Nov (e9a08d6); runs flown before that keep their '-dirty' stamp. |
| WP4-13 | Per-step record (observation, policy_state with policy_hash+generation, planner_output, post_repair, executed, outcome); auto-labels violation_type and risk_level; exporters (rosbag2 -> JSONL/Parquet), both episode-based and step-based | PARTIAL | tools/export_episodes.py JSONL + Parquet with read-back check (CHANGELOG 2026-10-21); 2 Nov: with a run's rosbag2 it reads the bag (`rosbags`, pure Python) and action.executed is the setpoint MAVROS was sent (/mavros/setpoint_raw/local) in each tick's window, back in the world frame, the flight log's verdict beside it; the join is placed on the mission_start event's clock and refused when the bag does not line up with the log; without a bag, without `rosbags`, with the join refused or headless the flight log's verdict stays, labelled a fallback on every step (executed_from, executed_fallback) and in the episode record; the observation takes the nearest pose MAVROS reported; tests/test_wp4outputs_exporters.py 31/38 (7 SKIP: 6 need pyarrow, 1 rosbags) | The bag path has not run on a real bag: `rosbags` is not in vla-real, so the rosbag2 round-trip test SKIPs and the join is tested through a stand-in reader; run it, and the exporter on the 19 stored ROS 2 runs with a bag (their /mavros/setpoint_raw/local counts equal the flight logs' ticks that sent something). Every other rail has no bag, so its executed is the labelled fallback. No image topic exists on the ROS 2 rail (observation: state topics and references only). Parquet not run in vla-real (pyarrow absent). |
| WP4-14 | Every release runs, in Stress Testing's smoke set: high-speed near-edge, sudden hot-applied NFZ, time-window switch instant, three simultaneous violations, GPS noise/latency (Safety Shield page) | PARTIAL | smoke profile carries all five Shield stressors (experiments/profiles/smoke.yaml); tests/test_sweep.py | configured in smoke.yaml but never run per release or build (independent review against the grant's wording, 29 Oct) |
| WP4-15 | A scenario can be flagged for A/B; the harness runs it with Prefix on/off on the same seed and the KPI report shows the delta (Prefix Compiler page) | PARTIAL | prefix on/off key in the harness (CHANGELOG 2026-10-07); pilot ignores text, no delta measured | Measure the A/B delta with a CSP-reading VLA (with WP2-01/WP2-09). |
| WP4-16 | Project AirSim is the perception-stress rail; KPI numbers come only from KPI-grade (canonical HIL) stress runs | PARTIAL | Project AirSim flown by ArduPilot (G1); KPI gate refuses its runs (kpi_grade false); no ScenarioSpec drives it | Run perception-stress scenarios from the suite on the PAS+ArduPilot rail (labelled non-KPI). |
| WP4-17 | Final delivery: 'KPI report on dynamic-scenario stress' (Grant overview delivery table, 2026-11-30); reference recommendation.md:34 says 'A KPI report from a nightly stress run' | PARTIAL | generator ready (tools/kpi_report.py, docs/data/kpi_rollup_2026-10-06.md headless); no hil stress run, report not written | Final deliverable: KPI report from a nightly hil stress run (due 2026-11-30). |
| WP4-18 | Perception-stress scenarios on Project AirSim (Stress Testing page: 'Project AirSim for perception stress'; reference dual-rail.md:16 'Texture / lighting / shadow stressors', :70 lighting, weather, novel textures, dynamic actors) | PARTIAL | dynamic actors only (40 pedestrians, traffic, signals in CityLife); experiments/templates/sensor_degradation.yaml headless | Add lighting/night/weather/texture stressors on Project AirSim with a per-scenario perception metric. |
| WP4-19 | Final demo gate (reference recommendation.md:33): all four components exercised by a non-trivial scenario from the Stress Testing scenario suite | PARTIAL | VLA (AerialVLA/DroneCATS) + signed DSL bundle + Shield flown together on CityLife (45 flights 27-28 Oct, 0 P0 escapes); no CSP, not a suite scenario, not on ArduPilot | One suite scenario with all four components (DSL, CSP-reading VLA, Shield, Stress Testing); PI to confirm the 5 final-demo scenarios (6 Oct Q2). |
| WP4-20 | Shield audit JSONL aggregated into the episode bundle by the harness at episode end (Safety Shield page, Audit log section) | DONE | guardrail/replay.py EPISODE_FILES audit.jsonl (shield_audit.jsonl since 2 Nov) packed per episode (CHANGELOG 2026-10-07); 30 Oct: the headless episode keeps the flying Shield's audit records (EpisodeRun.audit, AuditLogger.in_memory) and write_bundle packs them as shield_audit.jsonl at episode end, the sweep's and the gRPC EndEpisode's bundles alike (tests/test_sweep.py::test_the_episode_bundle_carries_the_shields_audit) | None. |
| WP4-21 | WP4 status as reported to the PI/ITRI versus the grant's WP4 outputs (scenario library, KPI rollups, replay harness; 7-row output table on the Stress Testing page) | DONE | WP4-built claim corrected (tools/check_claims.py; CORRECTION-2026-10-06 row A8); the correction was sent to ITRI (researcher, 2 Nov) | - |

### Architecture and topologies

31 items: 4 DONE, 22 PARTIAL, 1 MISSING, 4 DRIFT.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| ARCH-01 | VLA emits a 10 Hz setpoint; ArduPilot inner loop tracks it (canonical rail) | PARTIAL | 10 Hz setpoints tracked by ArduPilot on SITL+MAVROS (7 Oct) and PAS+ArduPilot (loop 9.91 Hz on 21 Oct) | no VLA has emitted 10 Hz setpoints into ArduPilot: the 10 Hz streams are the stub, the hand-written controller or a step tracker (AerialVLA on ArduPilot decided at 0.16 Hz) (independent review against the grant's wording, 29 Oct) |
| ARCH-02 | ArduPilot is the inner loop on every rail, including the perception (Project AirSim) rail | PARTIAL | ArduPilot flies Project AirSim (G1 8 and 13 Oct; PAS+ArduPilot rail flights to 21 Oct); but every VLA flight (23-28 Oct, 45 flights 27-28 Oct) ran on CityLife simple_flight, Project AirSim's own controller; 29 Oct: AerialVLA flew perception_node --pilot vla on ArduPilot SITL over pymavlink (pas_vla_goto_01/02, pas_vla_approach_01), every tick through the Shield and the FSM, 0 P0 escapes | The city follow comparisons (27-29 Oct) fly simple_flight: the final report labels them outside the ArduPilot architecture (section 6). |
| ARCH-03 | Real VLA backends produce the 4-D action at 10 Hz (and Orin latency gate G3) | PARTIAL | AerialVLA bf16 p50 259-486 ms on the 4090 (docs/data/aerialvla_serving_4090.json); in flight decisions every 0.4-0.5 s, DroneCATS 1.0-1.45 s, held as a 10 Hz stream (guardrail/vla_link.py StepTracker); no VLA timed on an Orin (7B does not fit the Nano); 30 Oct: the lab's own AIOT-VLA (training/aiotvla, docs/DESIGN-aiotvla.md; named NTUT-VLA until 30 Oct) answers in 48.4 ms p50 on the Orin Nano Super (19.1 decisions/s over 600 replayed captures, the server alone) and 10.3 ms on the RTX 4090; on the city rail its requests go every 0.1 s (capture gap p50 0.13 s) | A 4-D action at 10 Hz from a learned model in a flight on the Orin (the 30 Oct figure is the server alone); AGX Orin timing. |
| ARCH-04 | 4-D action (vx, vy, vz, yaw_rate) in body frame; body-to-NED conversion lives in the MAVLink adapter | DRIFT | body frame at the ROS 2 boundary, NED conversion in sitl/mavlink_adapter_node.py | as WP3-04 (independent review against the grant's wording, 29 Oct) |
| ARCH-05 | VLA->Shield on Float32MultiArray; Shield->MAVROS on mavros_msgs/PositionTarget; MAVROS->ArduPilot over UDP 14550 | DONE | PositionTarget via adapter; MAVROS on UDP 14555 behind mavlink-router (14550 is Mission Planner's) | None. |
| ARCH-06 | Switchable backend: CognitiveDrone, OpenVLA generic, BitVLA, in-house stubs, all conforming to 4-D | PARTIAL | RemoteVLABackend (guardrail/vla_backends.py) + protocol v2: AerialVLA (OpenVLA-7B), DroneCATS-2B, RaceVLA-7B flown 27-28 Oct (tests/test_vla_foreign.py 48/48); sitl/ros2_vla_bridge_node.py built, not flown; AeroVLABackend skeleton raises; CognitiveDrone weights unavailable, BitVLA no adapter | Fly a learned backend through the ROS 2 /vla/action_4d slot; CognitiveDrone/BitVLA adapters or a documented gap. |
| ARCH-07 | MAVROS 2 is the canonical bridge for Year 1 | PARTIAL | MAVROS 2 runs on the dev rail (7 Oct); tests/test_manifest.py | MAVROS 2 is the KPI topologies' bridge, but no VLA has flown through it: the learned-VLA ArduPilot flights are pymavlink (independent review against the grant's wording, 29 Oct) |
| ARCH-08 | AP_DDS migration (Q3, optional) | MISSING | no AP_DDS code (optional by the grant) | One sentence in the final report: deferred. |
| ARCH-09 | ArduPilot GeoFence/FENCE_ACTION as ultimate backstop; MAVLink escalation to RTL/Land/Loiter | DONE | GeoFence backstop configured and shown firing; FSM drives RTL/Land/Loiter(BRAKE) via SET_MODE (see WP3-07, WP3-10) | None. |
| ARCH-10 | Mission Planner via mavlink-router fan-out, parallel to MAVROS | PARTIAL | mavlink-router fans out to UDP 14550, 19.0 Hz measured by sitl/gcs_listen.py (CHANGELOG 2026-10-08); DESIGN-projectairsim-ardupilot.md:425 'Mission Planner's display was not recorded' | Open Mission Planner on a PAS+ArduPilot run and keep a screenshot/recording (the PI asked for this connection). |
| ARCH-11 | dev topology: everything on one desktop | PARTIAL | SITL+MAVROS dev rail flown 7 Oct (last time); Gazebo absent; no container image built (numpy ResolutionImpossible, CHANGELOG 2026-10-19); 29 Oct: the dev rail re-flown on today's code, PAS+ArduPilot with MAVROS 2 in WSL; pas_dev_mavros_02 is the first run stamped dev (real-time factor 0.968) | Gazebo (ARCH-16); build the images (C-17). |
| ARCH-12 | hil topology: desktop runs sims + MAVROS + GCS, Jetson Orin runs VLA + Shield (canonical KPI config) | PARTIAL | Orin Nano Super (JetPack 6.2.3) runs the follow controller incl. the Shield against the desktop's Project AirSim over 10.42.0.x (orin_follow_02 0.957; 16 Jetson flights 28 Oct); every VLA answered from the desktop GPU; no MAVROS/ArduPilot on that path; deploy/ images never built | Run the grant's hil: SITL+MAVROS+GCS on the desktop, Shield (+stub now, VLA on the AGX Orin) on the Orin, per docs/RUNBOOK-orin-hil.md; then the KPI campaign. |
| ARCH-13 | Topology field and the claim about which configuration yields contractual KPIs | PARTIAL | labels are dev/hil/flight; hil/flight only on evidence (CHANGELOG 2026-10-06/-07); tools/check_claims.py | the label rule is right, but no hil run exists, the 28 Oct Orin manifest nfz3b_orin_hier_aerovla_20261028 says single-host/unversioned, (independent review against the grant's wording, 29 Oct). The Final-Oct cheat-sheet is committed and gives the 3 dev / 38 functional-rail split (2 Nov). |
| ARCH-14 | flight topology: Orin companion with real ArduPilot autopilot; final demos | PARTIAL | deploy/topologies/flight.env + compose override defined (CHANGELOG 2026-10-07); no drone | Decided 2 Nov: the work stops at hil (the drone cannot carry an AGX Orin or run a large VLA); the written PI waiver for flight is still needed. |
| ARCH-15 | HIL bridge adapter (Q1, CRITICAL) and its 2026-06-30 fallback gate G1 | DRIFT | Gate G1 passed 2026-10-08 (demo/out/g1_20261008_223259/g1_verdict.md) and again 2026-10-13 (g1_20261013_203404) | G1 passed on 2026-10-08, 100 days after its 2026-06-30 gate, over SIM_AirSim JSON, not HIL_*; the gate's own rule fired; a PI ruling is needed (independent review against the grant's wording, 29 Oct) |
| ARCH-16 | Functional rail (Gazebo Harmonic) demoable end-to-end at mid-term | DRIFT | flown end-to-end 2026-11-02 at e74eae3 (WSL Ubuntu 24.04, Gazebo Sim 8.14, ArduCopter 4.5.7 JSON SITL, pymavlink): demo/out/gazebo_shield_on_20261102, _off_20261102, _on_dynamic_20261102 (Shield on P0 escape 0.0, off 0.584), and over MAVROS 2 in the dev topology at 0207af0 (PHYSICS=gazebo sitl/run_ros2_demo.sh): gazebo_ros2_shield_on_20261102 P0 0.0, gazebo_ros2_shield_off_20261102 P0 1.0 with the GeoFence firing; each with a flight-log replay video (tools/render_track_video.py) (CHANGELOG 2026-11-02 evening) | Dated gate missed by 105 days; with the Shield on the mission does not reach its target (theta escalates on the first tick, Loiter times out, RTL), the false trigger WP3-15 addresses. |
| ARCH-17 | Mid-term delivery 2026-07-20 (DSL+IR, Shield ROS 2 node with one projection operator, Gazebo rail) | PARTIAL | DSL+IR (IR late, 6 Oct), Shield node (19-24 Aug) and the Gazebo rail (first flown 2 Nov) exist, all after the gate | State the actual dates in the final report. |
| ARCH-18 | Perception-rail integration (final delivery 2026-11-30) | PARTIAL | PAS+ArduPilot perception node flown with detector pilot + Shield + FSM (8-21 Oct); start gate and --pilot vla built 22-23 Oct, 'none flown'; --link mavros never flown; detector boxed the wrong object on the close flights; 29 Oct: AerialVLA flew perception_node --pilot vla on ArduPilot SITL over pymavlink (pas_vla_goto_01/02, pas_vla_approach_01), every tick through the Shield and the FSM, 0 P0 escapes | Fly the VLA pilot over MAVROS 2 in one run (the 29 Oct MAVROS runs flew the detector servo, the VLA runs pymavlink). |
| ARCH-19 | Final integration + KPI runs (Q4) in the canonical hil topology; KPI report on dynamic-scenario stress | PARTIAL | harness, KPI report and manifests ready; nothing on hil; no multi-seed dynamic sweep through MAVROS | hil KPI campaign + KPI report + final report (WP4-01, WP4-17, X-01). |
| ARCH-20 | A real VLA constrained end-to-end in the canonical KPI topology | PARTIAL | VLA slots built on both ArduPilot rails: perception_node.py --pilot vla, sitl/ros2_vla_bridge_node.py (CHANGELOG 2026-10-23 later, 'none flown'); VLAs flown only on simple_flight; 29 Oct: AerialVLA flew perception_node --pilot vla on ArduPilot SITL over pymavlink (pas_vla_goto_01/02, pas_vla_approach_01), every tick through the Shield and the FSM, 0 P0 escapes | The learned model over MAVROS 2 in one run (the contract title); KPI-grade needs hil. |
| ARCH-21 | Constrained-VLA evidence on the perception rail and in current demos | PARTIAL | VLA-piloted flights behind the Shield with the Shield acting: vla_follow_redcar_01_20261023 (177 corrected ticks), nfz3b_pc_hier_dronecats / nfz3b_orin_hier_aerovla 28 Oct (13 / 257 repairs, 0 zone ticks, 0 P0 escapes) | no flight shows a VLA's 10 Hz 4-D action repaired by the Shield on the PAS+ArduPilot rail; the cited hierarchical flights were 0-6% VLA-flown, on simple_flight (independent review against the grant's wording, 29 Oct) |
| ARCH-22 | rosbag2 capture for the determinism and replay contract (L3) | PARTIAL | rosbag2 recorded on the ROS 2 rail (sitl/run_ros2_demo.sh, demo/out/ros2fix2_on_fsm/bag; 19 stored runs); since 2 Nov the record command also names MAVROS's sensor topics (velocity, IMU, GPS fix, relative altitude, altitude, battery, home, extended state, status text), the VLA bridge's two camera topics and raw MAVLink both ways (/uas1/mavlink_source, /uas1/mavlink_sink), the deploy entrypoint likewise (tests/test_ros2_package.py, tests/test_deploy_configs.py); the exporter reads the bag (WP4-13) | Not flown: no run has recorded the new topics yet (`/uas1/...` is MAVROS 2's router link as its source names it, unverified on this rail). A bag exists on the ROS 2 rail only, the camera topics are empty there (nothing publishes them on SITL), and replay re-scores from flight_log, not the bag (independent review against the grant's wording, 29 Oct). |
| ARCH-23 | Q3 paraphraser deliverable (Aug-Oct) | PARTIAL | same artefact as WP2-13: the stored set, validator and CLI were in Q3 (6 Oct); the separate service the grant names landed 2 Nov (guardrail/paraphraser_service.py), with the 4 C8 sets re-authored and the validator's held-out misses fixed the same day | Delivered after the Aug-Oct window: state the slip in the report; the orchestrator binding and the flown KPI as WP4-10 / WP4-09. |
| ARCH-24 | 1 PI + 4 Master students (2 ArduPilot/MAVLink, 2 VLA fine-tuning/prompting) | DRIFT | one developer (git log); PI-DECISIONS-2026-10-06 Q4.9 asked how to state it | Decided 2 Nov: the final report states a one-person team. Stays DRIFT: the grant names five people. |
| ARCH-25 | Observation (Camera / IMU / GPS) reaches the VLA at >=10 Hz (Architecture constraints diagram; ref layer-requirements L2/L5 'Surface obs at >=10 Hz') | PARTIAL | detector 2.18-2.48 Hz on the Jetson flights, ~3 Hz on the PC; VLA frames at its decision rate (~2 Hz); 30 Oct: the lab's own AIOT-VLA (training/aiotvla, docs/DESIGN-aiotvla.md; named NTUT-VLA until 30 Oct) answers in 48.4 ms p50 on the Orin Nano Super (19.1 decisions/s over 600 replayed captures, the server alone) and 10.3 ms on the RTX 4090; on the city rail its requests go every 0.1 s (capture gap p50 0.13 s) | Frames reach the VLA at about 7.5 Hz on the city rail (capture gap 0.13 s), below 10 Hz; a faster capture path. |
| ARCH-26 | 10 Hz Shield monitor budget validated by Orin profiling; Rust hot-path as Q3 fallback (Overview page, locked cross-cutting decisions) | PARTIAL | tools/profile_shield_tick.py ready; Orin Nano city flights logged Shield p50 2.5 / p95 6-9 / max 261 ms; profiler not run on the Orin, no Python-vs-Rust go/no-go | Run the profiler on the Orin and record the Rust decision. |
| ARCH-27 | Implementation language Python + Pydantic v2 throughout (Overview page, locked cross-cutting decision) | DONE | Python + Pydantic v2 throughout (guardrail/, pyproject.toml) | 3.11+ version is X-06. |
| ARCH-28 | The same code base runs in all three topologies, all first-class (grant); reference topologies.md: identical Docker image, topology selected by config flag, a 'hard requirement' | PARTIAL | one image definition, topology from evidence (deploy/, tests/test_deploy_configs.py); never built; the Jetson runs used a native env synced by git archive | Fix the numpy pin and build the images (C-17); run the same image on desktop and Orin. |
| ARCH-29 | Shield loads the Policy IR / signed Bundle at runtime (Architecture constraints diagram: 'Policy IR / Bundle -> loaded -> Shield'); overlaps WP1 | DONE | Shield loads the signed bundle on every rail incl. the city rail since 2026-10-22 (demo/follow_vlm.py:122-193); Jetson metrics policy_source kind=bundle, declared=loaded hash | None. |
| ARCH-30 | No default backend until the 'Q2 measurement set lands' (Architecture constraints, Action space); basis for any backend choice and for the G3 candidate | PARTIAL | deploy/evidence/vla_backend_table.md (Default backend: none; Orin columns empty; regenerated 21-22 Oct); 42-flight PC+Jetson comparison of 7 VLA configs + hand-written follow (CHANGELOG 2026-10-28) not folded into the table | Fold the 27-28 Oct matrix into the protocol table, add Orin timing (AGX), then choose a default with the PI. |
| ARCH-31 | ArduPilot inner loop 'Bit-identical between SITL and firmware' (VLA/autopilot interface table) together with code_revision 'across the entire stack' (ref determinism contract) | PARTIAL | ArduPilot pinned Copter-4.5.7 and recorded (sitl/setup_sitl.sh); code_revision unresolved on the split-host Jetson runs ('unversioned') and dirty on the PC runs of 27-28 Oct; since 28 Oct the Jetson is a git checkout (deploy/orin/sync_follow_controller.ps1 ships a git bundle), so code_revision resolves there; a remote simulator is stamped projectairsim-remote-sim (guardrail/manifest.py) | Fly the deliverable runs from a clean tree (as WP4-12). |

### Cross-cutting locked choices

16 items: 2 DONE, 12 PARTIAL, 2 DRIFT.

| ID | Obligation | Status | Evidence | Open |
|---|---|---|---|---|
| X-01 | Final delivery: signed final report (2026-11-30 gate) | PARTIAL | docs/FINAL-REPORT-outline.md (outline v0, 29 Oct): the report's structure, the KPI table and this evidence map | Write the report from the outline; signer and CA from the PI (12 Oct Q5); due 2026-11-30. |
| X-02 | Safety Shield acceptance KPI: repair success rate | PARTIAL | same as WP3-20 (guardrail/kpi.py repair_success) | Same as WP3-20. |
| X-03 | Stress Testing locked KPI: average repair count per episode (as a KPI, not only a report column) | PARTIAL | mean repair count per episode and family in tools/kpi_report.py / docs/data/kpi_rollup_2026-10-06.md; no KPI-grade input | no hil run exists: the KPI gate (is_kpi_grade) refuses every stored run. |
| X-04 | Q2 VLA-backend measurement set: the comparison that must exist before any backend becomes the default | PARTIAL | same as ARCH-30 | Same as ARCH-30. |
| X-05 | Observation rate into the VLA: camera/IMU/GPS at 10 Hz or faster | DRIFT | perception to the controller at 2.2-3 Hz (Jetson det_hz 2.18-2.48; PC ~3 Hz) against the grant's >=10 Hz observation rate | PI acceptance (G3) or a faster perception path; state the measured rate. |
| X-06 | Locked implementation language: Python 3.11+ (Policy DSL, Prefix Compiler, Safety Shield and Stress Testing pages) | PARTIAL | same as WP1-25 | Same as WP1-25. |
| X-07 | Validate the Shield's 10 Hz monitor budget by Orin profiling, with a Rust hot path as the Q3 fallback | PARTIAL | same as ARCH-26 | Same as ARCH-26. |
| X-08 | One code base runs first-class in dev, hil and flight; promotion dev to hil to flight is a config flag (identical image) | PARTIAL | same as ARCH-28 | Same as ARCH-28. |
| X-09 | R6 freeze gate: IR shape frozen at the end of Q1 (2026-04-30); every later IR-shape change carries a semver bump and a migration note | PARTIAL | same as WP1-22 (docs/IR-MIGRATIONS.md) | Same as WP1-22. |
| X-10 | Bundle identity fields: policy_hash as a full SHA-256, signed_by (Lab or ITRI CA), changelog as a diff against the previous version, issued_at, layers_merged | PARTIAL | full SHA-256, issued_at, layers_merged, Ed25519 signed_by lab-dev key; since 29 Oct the changelog is a diff against the previous version (guardrail/bundle.py --previous, manifest changelog_diff, 895b5a0; tests/test_bundle.py) | CA name for signed_by (PI/ITRI, 12 Oct Q5). |
| X-11 | MSL as an opt-in altitude reference on geometry primitives (altitude_ref AGL/MSL) | PARTIAL | altitude_ref AGL/MSL declarable (guardrail/models.py:156); MSL refused for flight (models.py:1424, no ground-elevation source); MSL hot-apply refused (CHANGELOG 2026-10-19) | Give the Shield a ground-elevation source so an MSL rule can fly. |
| X-12 | Hybrid hot-update rule: only dynamic_nfz, time_window_switch and corridor_swap may change mid-flight; polygon_fence and the envelopes stay locked from mission start | DONE | hot-apply of a fixed polygon fence after take-off refused; only the three classes hot-apply (guardrail/api.py; CHANGELOG 2026-10-07); tests/test_api.py, tests/test_shield_events.py | None. |
| X-13 | Single rule-evaluation code path (the checker reuses the IR and nothing else evaluates rules) | DRIFT | Shield.rule_status is the one checker; HUD and FenceGuard read it (CHANGELOG 2026-10-07) | as WP3-22: compiler.action_violations is a second rule evaluator (tools/prefix_eval uses it) (independent review against the grant's wording, 29 Oct) |
| X-14 | Dynamic-actor stressors in parameter sweeps: wind (and GPS denial) | PARTIAL | wind, gusts, GPS dropout stressors with a closed-loop ArduCopter wind model (CHANGELOG 2026-10-06, -13); experiments/templates/wind_disturbance.yaml | wind and GPS stressors act on the headless kinematic model only; no simulator run applies them (independent review against the grant's wording, 29 Oct) |
| X-15 | SROS 2 security on the ROS 2 graph (the harness mTLS reuses its root CA) | PARTIAL | SROS2=1 opt-in, one enclave per process, Enforce; tools/sros2_check.sh 6/6 in WSL Jazzy (CHANGELOG 2026-10-21, again 2026-11-02); flown 2026-11-02: demo/out/ros2_sros2_shield_on_20261102 records the enclave and the lab CA fingerprints (CHANGELOG 2026-11-02 evening) | Write per-topic permissions (the enclaves carry sros2's allow-all default); make the Windows harness read the same CA. |
| X-16 | Meaning of the vla_model_hash field (SHA-256 of the VLA's weights). This is a nuance of the six-field manifest item rated DONE | DONE | vla_model_hash carries the weights' SHA-256 on VLA flights (e.g. nfz3b_orin_hier_aerovla_20261028/manifest.json aerialvla:...@sha256:ca9e8701...), none:hand-written-controller otherwise | None. |
