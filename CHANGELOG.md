# Changelog

All notable changes to the Guardrail safety layer.

Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
versioning is [Semantic Versioning](https://semver.org/spec/v2.0.0.html), with
the caveat that this is pre-1.0 research software and the public surface is
still moving.

A note peculiar to this project: several entries below are **retractions**. The
grant names five acceptance KPIs, *P0 violation escape rate* (target 0) among them, and a safety number that is
wrong is worse than one that is missing, so a corrected claim is treated as a
shipped change and recorded as one.

---

## [Unreleased]

### 2026-11-19 (pab3, prefix A/B on the new clearance horizon) — the prefix A/B re-flown on hil at 1239619: P0 escape 0, fail-safe 15/15 under the adopted reading, slowest tick 18.10 ms; the corridor-drift-out RTL of pab2 did not recur

#### Flown
- pab3 (hil, 1239619, profile `prefix_ab`, 33 missions, 4500 s;
  docs/data/kpi_prefix_ab_20261119/): the re-fly of the prefix-on arms the
  2026-11-17 entry owed, on the Shield that sizes the clearance horizon for
  the ArduPilot vehicle's response. The pilot is the hand-written
  CSP-consuming adapter (guardrail.prefix_consumer), not a learned VLA.
  - KPIs, 33 episodes, all KPI-grade: P0 escapes 0 on 2141 P0 ticks
    (passthrough null 0.312); tick p99 14.7 / max 18.10 ms, action receipt
    to setpoint p99 14.8 / max 36.92 ms, 0 of 15783 ticks over 100 ms;
    fail-safe 15/15 under the adopted reading r2 (Wilson 95 % low 0.796),
    12/15 as flown (the 3 nfz-head-on prefix-off runs end in RTL, as on pab1
    and pab2).
  - A/B, on vs off: mission success 0.800 vs 0.600; repairs per episode
    2.333 vs 130.400 (A/A largest pair 0); mean repair magnitude 0.000 vs
    9.380 m/s; mean time to safe 4.830 vs 4.833 s (A/A 0.096 s).
  - Against pab2: corridor-drift-out prefix-on 4/4 success with 0 repairs
    (pab2 3/4: pab2_011 ended in RTL, "no operator converged"). One campaign
    does not show which change removed it; it was not reproduced in 4
    flights.
  - Track videos `<run>/<run>_track.mp4` for the 33 runs.

#### Changed
- The three 2026-11-19 sections for rel060, X-11 and WP1-07 moved from
  [0.6.0] to [Unreleased]: they record flights and changes made after the
  v0.6.0 tag (a85dc91).

### 2026-11-19 (WP1-07 distance_envelope flown on hil; clearance on a lagging vehicle) — the first hil flight of the distance_envelope entered both envelopes' corners and ended one lane in a false RTL; the clearance horizon is now sized for the ArduPilot vehicle's response and the edge repair converges; re-flown, both lanes pass

#### Flown
- de_hil1 (hil, 0f4269a, profile distance_envelope, bundle
  distance-envelope-sitl-v0.1.0 signed with the lab-dev key, world
  experiments/worlds/de_block.yaml sha256 503f8889...): both lanes FAILED.
  P0 escape 0 on the Shield-flown ticks, but the aircraft entered the
  rounded corner of each envelope: lane -1 (buildings, P0) inside for 1.9 s,
  true distance down to about 4.4 m against 5 m; lane -11 (roads, P1)
  inside for 2.4 s, down to 1.93 m against 3 m. Lane -1 then took G3
  (Brake -> Loiter, "no operator converged"), the 30 s Loiter timeout (G5)
  and RTL, whose straight line home crossed the buildings envelope again
  (2 breaches, counted while the autopilot flew). The profile defines no
  Shield-off cell.
- Diagnosis (three independent diagnoses, each adversarially verified;
  docs/data/distance_envelope_20261119/diagnosis_de_hil1.json):
  (A) the clearance look-ahead was dt + v / (2 x 3 m/s^2), 0.77 s / 3.1 m at
  4 m/s, as if the vehicle obeyed at once; the ArduPilot vehicle flies each
  setpoint 0.45 s late at most 1.75 m/s^2 and needs about 6.4 m. (B) at a
  field cell boundary ClearanceFix pushed at the 0.5 m/s floor (the depth
  divided by the 5 s lookahead) and fell 0.5 mm short; its second pass was
  a no-op, ClearanceEscape rescued and the FSM read it as not converged.
  The headless ardupilot and rail response arms reproduced the entries at
  the same corners; the ideal arm passed. The ideal-vehicle labels
  (success, no fail-safe) are the requirement and were kept.
- de_hil2 (hil, 06518b7, the fix below): both lanes PASS: 0 breaches,
  target reached, no fail-safe (G1/G7 only), P0 escape 0 (null
  passthrough 0.082), 42 and 86 repairs. `tools/replay_from_bag.py
  --campaign demo/out/hil_de_hil2`: 2 of 2 identical (ticks, trajectory,
  KPI). metrics.world.sha256 matches de_block.yaml; shield_forecast records
  the response (0.45 s, 1.75 m/s^2). Videos `<run>/<run>_track.mp4` for the
  four runs.

#### Changed
- `guardrail/shield.py` (17e8047, 6ef5a6d): a Shield given
  `clear_response` (the ArduPilot rails: sitl/ros2_shield_node.py's flying
  and warm-up Shields, sitl/run_sitl_demo.py, and the headless flying Shield
  on the ardupilot and rail response arms) sizes the clearance horizon
  dt + reaction_s + v / (2 x accel), 1.69 s at 4 m/s, with v the largest of
  the request and the setpoints still in the vehicle's pipe, so a repair
  cannot shrink its own horizon; the escape it shapes brakes the same way.
  The FSM's theta reads the repair's depth over the instant vehicle's
  horizon, so the longer look-ahead starts the repair earlier without
  inflating theta's input. Without `clear_response` (every other Shield,
  the oracle and the judges) nothing changes. ClearanceFix sizes its push
  to clear the deficit at the offending sample's time (+0.05 m/s), and a
  content stop point no longer steers a repair the commanded forecast
  breaks: grid probes around de_block, 48 non-converging repairs on master
  -> 0, and 87 escapes on the own-velocity forecast -> 0.
  `tools/replay_at_revision.py` rebuilds a recorded `clear_response`.
- Headless: smoke, distance_envelope, fsm_edges, grant_example and
  prefix_ab 141 of 141 episodes identical to master; smoke P0 0 over 63,
  fail-safe 63/63; library P0 0 over 179, fail-safe 179/179; nightly seed
  1001 (at 17e8047) P0 0 over 247, fail-safe 247/247; distance_envelope 0 breaches and
  success on the ideal, ardupilot and rail arms (on master the lagging arms
  breached both lanes). New tests in tests/test_clearance.py,
  tests/test_distance_envelope_msl.py, tests/test_window_opens_on.py and
  tests/test_replay_at_revision.py, each failing without its fix. CI
  183/183 in the worktree (6ef5a6d) and the lab checkout (804d399).
- Not changed: the PAS perception rail, the mission node and the AirSim
  demos keep the old horizon; the hil campaigns flown before 06518b7
  (rel060, msl_dem_hil2) flew it too.

### 2026-11-19 (X-11 flown on SITL and hil) — the msl_dem profile (MSL polygon_fence, corridor and altitude_envelope over the ridge DEM) flown on the desktop SITL rail and on the hil rig: P0 escape 0 on all 12 missions, every reader names the DEM by path and sha256, the bag replays on hil 6 of 6 identical

#### Flown
- SITL (desktop WSL ArduPilot SITL, master bdc0af6, `--profile msl_dem
  --backend sitl`, 1022 s): 6 missions, P0 escape 0 (null passthrough
  0.045), 0 breaches, 0 polygon entries. Every run's metrics.json records the
  ground (experiments/worlds/dem_cmac_wgs84.json, sha256 44ed8a16...); the
  six episode bundles under the grant's names are in demo/out/stress_msl_dem.
- hil (AGX Orin + desktop SITL, e56d3e7, campaign msl_dem_hil2, 1044 s):
  the same 6 missions, the same outcomes, P0 escape 0 (null 0.051), 0
  breaches, 0 entries, 6 episode bundles. `tools/replay_from_bag.py
  --campaign demo/out/hil_msl_dem_hil2`: 6 of 6 identical (ticks,
  trajectory, KPI). The first attempt, msl_dem_hil, flew nothing: the
  msl-dem bundles were never copied to the Orin (see Fixed).
- On both rails: corridor-west vz=1 and both ceiling-climb cells pass
  (RTL, as labelled); corridor-west vz=0 ends in Loiter 16.4 s into its
  30 s timeout (censored, not measured: fly longer); both zone-ridge cells
  end in RTL where the ideal-vehicle label expects none: over the ridge the
  agl-band repair reaches 0.51 m vertical against theta 0.50 m (G1 -> G3),
  and the 30 s Loiter timeout takes G5 to RTL, the grant's escalation, as
  on the three msl_dem_sitl flights of 18 Nov. The labels are the headless
  ideal vehicle's; the rail-vehicle reading has no msl_dem labels.
- FENCE_ALT_MAX: msl_dem_sitl 40 m above home (raised from 30 to 45 m),
  msl_dem_corridor 26 m (31 m), msl_dem_zone 81.725 m (86.725 m). The zone
  cells start 45 m south of the origin, where the DEM reads 583.775 m MSL,
  so their ceiling over the 605.5 m crest is 81.725 m, not the 81.5 m of a
  home at the origin (584 m MSL).
- SITL bag replay (docs/data/msl_dem_20261119/replay_from_bag_sitl): 4 of 6
  identical; 2 differ by one watchdog tick each (a tick the timer ran with
  no /vla/action_4d message of its own, which the bag cannot mark), and on
  one of them the KPI table's tick count by the same tick (839 against
  838). `python -m guardrail.replay verify` OK on all 6 replay tarballs,
  each carrying the DEM, signed in its index.
- Track videos: `<run>/<run>_track.mp4` for the 12 runs.

#### Fixed
- `fence_alt_plan` (sitl/mavlink_adapter_node.py, 1541442): a FENCE_ALT_MAX
  read back as float32 is the value set. The SITL zone cells set 86.725 and
  read back 86.7249984741211, and were recorded `stricter_than_policy` with
  a warning; on hil (with the fix) the same cells record False.
- The sweep's grant episode name note (5b903a9): the dry run and each run's
  `grant_episode_name_note` still said no file of the name is written on the
  rail and WP4-11 stays open, while the rail has written the six-name bundle
  there since 18 Nov; it now says so, and "a name only" with --no-bundles.
- hil pre-flight (5c94c16): run_hil asks the Orin, before SITL starts,
  whether the node's --bundle and --ground files are there, and records an
  ERROR naming the missing ones; before, a missing bundle killed the Shield
  node before its first event while `ros2 launch` exited 0, read as "no
  events.jsonl (the mission never started)".
- CI 183/183 in the worktree for each (1541442, 5b903a9, 5c94c16) and in
  the lab checkout (66a564e, b30b841).

### 2026-11-19 (release smoke rel060) — the hil smoke of release 0.6.0: P0 escape 0 over 48 Shield-on flights, fail-safe trigger correctness 48/48 under the adopted reading, slowest tick 70.89 ms; every episode written as a six-name bundle and replayed at its flown revision

#### Flown
- rel060 (v0.6.0 = a85dc91, hil, smoke profile with `--hil-controls`,
  launched from the detached worktree flight-a85dc91; 6139 s): 64 episodes
  planned, 49 missions flown, 15 refused before take-off (stressors the hil
  rail cannot realise: GPS noise, gusts, as in every hil campaign). All 49
  KPI-grade (signed bundle bound to the run, clean revision, speed-up 1).
- `docs/data/kpi_rel060/kpi_report_rel060_2026-11-19.md` / `.json`
  (`tools/kpi_report.py --campaign demo/out/hil_rel060 --rescore-rail
  docs/data/kpi_rel060/rescore_r2_rel060.json`, code revision 57312e9,
  clean): P0 violation escape rate 0.0 (0 escapes on 2220 P0-violation
  ticks, 48 Shield-on flights; Shield-off control 0.65); fail-safe trigger
  correctness (r2) 48/48 = 1.000, Wilson 95 % low 0.926 ((a) 42/48,
  (r) 47/48; the campaign's own ideal labels 43/48); `tick_ms` p99 26.2 /
  max 70.89 ms and `e2e_ms` p99 26.3 / max 70.98 ms, 0 of 15,208 ticks over
  100 ms. Tracked: mission success 22/48, mean time to safe 3.73 s, repair
  success 98.2 % (2309/2351 ticks), 68.1 repairs per Shield-on mission.
  Against ntFIN2: 0 regressions.
- WP3-06: every Shield-on episode flew with PathRepair in the ROS 2 rail's
  default chain (metrics `path_repair: true`), evaluated after the
  projections on every tick; it acted on 0 ticks (in chain mode it detours
  round stand-off rings only, and LateralProjection kept every one of these).
- WP4-11: every flown episode wrote its bundle under the grant's six names
  (manifest.json, policy_bundle.tar.gz, episode.bag2, shield_audit.jsonl,
  harness_events.jsonl, kpi.json): 49 under demo/out/hil_rel060/episode-*.
  `tools/replay_at_revision.py` replayed them at a85dc91
  (docs/data/kpi_rel060/replay_at_revision_rel060.json): 37 identical, 10
  differ only in the last float digits (at most 4.0e-14), 2 not replayable
  (gps-dropout: the pipe the Shield dead-reckons on is not logged per tick);
  the folders were given as absolute paths (see Fixed).
- Track videos: `<run>/<run>_track.mp4` for each of the 49 runs.

#### Fixed
- `tools/replay_at_revision.py` resolves the run folders it is given before
  the drivers start (d3737f2): they run inside the revision's worktree, so a
  relative run path named nothing there and all 49 rel060 runs first read
  "no flight log". tests/test_replay_at_revision.py 17/17 (the new test
  fails without the fix). Full CI 183/183 in the worktree (d3737f2) and in
  the lab checkout (8e36998).

## [0.6.0] — 2026-11-19

Release for the final hil smoke (WP4-14): every dated entry below, back to
0.5.1, ships in it.

### 2026-11-19 (wf21, code) — the open rows' code side: MSL readers, one evaluation path, WGS84-only flights, one body-frame boundary, replay at the flown revision, people from the feed, a prefix for a text-reading VLA

Seven branches (wf21-msl-readers, wf21-onepath3, wf21-wgs84-writers,
wf21-bodyframe, wf21-replay-rev, wf21-people-distance, wf21-prefix-vla), each
implemented, reviewed adversarially against the grant's wording and fixed,
merged as integrate/1119. Full CI 183/183 in the worktree (cead0f0) and in the
lab checkout (55aefcd). Code only: every row below stays PARTIAL until its
flights (listed per row) have flown.

#### Changed
- **X-11, every reader takes a declared ground** (wf21-msl-readers):
  - The stress harness's sitl/hil backends (rail_node_args), the headless
    harness's rotated missions, the mission node, gcs_backstop,
    sitl/bundle_origin.py and the bag's custom data take a ground named by
    path and sha256; none guesses one. A rail run of a scenario with a
    declared ground whose metrics.json names no ground, or another file, is
    refused, never scored (`ground_flight_problems`).
  - An MSL episode's replay tarball carries the DEM itself, signed in the
    index; replay checks its sha256 and re-derives heights from it.
    tools/replay_from_bag.py compares a DEM run's poses above the DEM (before:
    on a stand-in episode 0.58 m off home's elevation, 24 of 24 poses read
    "differs"); tests/test_replay_from_bag.py 30/30.
  - The RTL leg is judged against the ground sampled along it, in the harness
    and in the node (FSM forced to RTL at (0, 170), 606 m MSL: rtl_failed,
    blocked_path, agl-band); tests/test_msl_readers.py 20/20.
  - FENCE_ALT_MAX is capped by an MSL ceiling (an MSL altitude_envelope, or a
    hard keep-in MSL corridor in force all flight), less home (584 m MSL):
    msl_dem_sitl 40 m (raise 45 m), msl_dem_corridor 26 m (raise 31 m; was
    81.5 m / 86.5 m), msl_dem_zone 81.5 m (raise 86.5 m; a keep-out zone bounds
    nothing above it). A flown-default change, with --ground only.
  - Correction to the 18 Nov entry: the DEM's highest ground is 605.5 m MSL, so
    the uncapped ceiling was 81.5 m and its raise 86.5 m, as msl_dem_sitl_01..03
    recorded (not 81.26 m / 86.26 m).
  - New stress profile msl_dem (6 cells: MSL polygon_fence, corridor and
    altitude_envelope over the ridge DEM): sitl dry-run 6 FLY, 0 refused;
    RUNBOOK-orin-hil step 11a 2c.
- **WP3-22 / X-13, one rule-evaluation path** (wf21-onepath3): FenceGuard's
  building half judges with the Shield's `clearance_breached` against the
  limits of every obstacle_clearance rule in force at that instant
  (`Shield.clearance_limits_now`), on the Shield's field (guard_field
  "shield"); the end-of-flight stand-off counts and the stand-off scorer call
  `standoff_breached` / `standoff_ring_m`. Replay of the 10 stored AGX follow
  runs: 41959 ticks, 469 (1.12 %) mode or side changed, 109 (0.26 %) building
  brakes changed. 2302 stored run folders (121000 ranges): 0 ranges within
  1e-9 m inside a ring, so no stored count changes. The AST scan finds no rule
  threshold compared outside the checker except 15 named non-verdicts.
- **WP1-14, flights fly WGS84 only** (wf21-wgs84-writers):
  `bundle.load_for_flight` refuses a policy stored as local-metres or
  wgs84+local-metres (`LegacyPointFormRefused`, naming the WGS84 bundle
  beside it); a recorded flight re-runs with `legacy_read` /
  `VLAGUARD_LEGACY_READ=1`, recorded. A flown-default change for the 13
  legacy lab bundles; no policies/*.yaml is affected. Every writer refuses or
  converts metre points. tools/bundle_census.py: 17 bundles not flight-ready
  (13 legacy, 4 frameless) over 10 tasks, each with its re-fly bundle; lab
  checkout built follow-car-citylife-nfz3 / nfz3b v0.1.3 (sha256 64153e45...,
  7c458986...). The bench and msl_dem_kpi policies state a frame
  (docs/data/msl_dem_kpi.json re-measured: hot-apply p99 8.61 ms, query p99
  2.25 ms, MSL corridor query p99 1.55 ms, cold start p95 461 ms; 9/9
  verdicts).
- **WP3-04 / ARCH-04, one body-to-NED crossing** (wf21-bodyframe): every rail
  and demo loop hands the Shield the body action at the measured heading
  (city follow and pymavlink SITL no longer at heading 0) and sends through
  `frames.body_to_local_ned` after the projection; the demo loops' rate
  limiters cap the world reading as before; real_vla_demo turns OpenVLA's
  action from its capture heading. tests/test_frame_boundary.py 20/20: the
  emitted body action is the same at headings 0 / 90 / 179.9 / -90 / 37.3 /
  -123.4 to 1e-12 for every rule family (raster clearance at the quarter
  turns); crossing detectors work on structure and are mutation-tested.
  Headless: 141 episodes, 129 identical, 12 differ only at <= 2.84e-14 plus
  zero-sign flips and, on the 3 dyn-nfz-translate arms, a policy_hash from a
  1-ulp vy_mps; the KPI inputs are unchanged in all 12.
- **WP4-11, episode bundles and replay at the flown revision**
  (wf21-replay-rev): rail campaigns (--backend hil|sitl, unless --no-bundles)
  write a bundle per flown episode under the grant's six names
  (manifest.json, policy_bundle.tar.gz, episode.bag2/, shield_audit.jsonl,
  harness_events.jsonl, kpi.json). tools/replay_at_revision.py replays an
  episode at the code revision its manifest records (a temporary detached
  worktree; "-dirty" and unversioned refused with a reason): Q3 720 identical
  of 5,509 (4,724 -dirty, 20 unversioned, 45 not replayable; events and KPI
  only); hil open-loop 972 identical of 1,771 on Python 3.10 (984 on 3.12),
  513 body-frame and 283 float last digits, 3 decision (tick 460 of three
  dyn-nfz-rotate-spin runs, a no-op GeofenceSlide pass, emitted action
  bit-identical), 97 not replayable (docs/data/replay_at_revision_20261118*).
- **WP1-07, people from the feed** (wf21-people-distance): a distance_envelope
  over people is a clearance over every person the rail reports
  (models.PeopleClearance on the Shield's people field): the PAS perception
  node's ground-projected person boxes, the subject while classified as a
  person, a declared person subject on SITL / headless; refused when no
  people source exists. `--level-peds N` on the PAS rail reads the level's
  pedestrians as record-only truth (true minimum distance, time inside the
  envelope, feed recall). People cull max(min_distance_m) + lookahead x speed
  bound + 10 m, never below 100 m. New flight policy
  experiments/policies/follow_car_citylife_people.yaml.
- **WP4-15 / WP4-01, prefix A/B with a text-reading VLA** (wf21-prefix-vla):
  perception_node `--vla-prefix off|csp|neutral` and `--ab-pair`, the CSP
  budgeted to the model's prompt slot (guardrail/vla_prefix.py), paired
  flights by deploy/orin/run_pas_prefix_ab.sh (ABBA, optional A/A), a
  report section "Flown by a VLA that reads the prefix". AerialVLA: CSP 85
  against neutral 85 OpenVLA tokens. AIoT-VLA's csp arm is refused before
  take-off (CSPBudgetExceeded: the follow policy's 4 P0 sentences need 45
  CLIP tokens against 11; alt-band 10, bld-clearance 9, standoff-any 12,
  standoff-pedestrian 14), never truncated.

#### Integration (integrate/1119)
- The Shield's recovery direction search keeps its 30-degree grid from North:
  wf21-bodyframe's nose-relative grid moved 9 of 12 stored follow decision
  streams pinned at c77c77d (tests/test_own_velocity_forecast.py) with no
  flight behind it; the grid is the same body-frame set at every quarter
  turn.
- `standoff_rules` / `standoff_ring_m` read only subject_standoff rules: a
  distance_envelope over people is a clearance over every person since
  wf21-people-distance, judged by `clearance_breached`.
- transformers (the CLIP token counter) is declared in the `tokens` extra;
  tools/replay_at_revision.py is named an offline world-frame replay;
  tests/test_frame_boundary.py scans tracked modules only;
  tests/test_harnesstruth_bundles.py reruns fase3-sim-demo-v0.1.4 (WGS84).
- The AGX companion image vlaguard/companion:jazzy is rebuilt as an overlay of
  the 14 Nov build with the current deploy/docker/entrypoint.sh (sha256
  70edc29a..., the only image input changed since); the old build is kept as
  vlaguard/companion:jazzy-20261114.

#### Still open (flights, in order)
hil shakedown; release smoke on hil (episode bundles, PathRepair on the ROS 2
rail, replay at revision); msl_dem on SITL and hil; distance_envelope on hil;
pab3; follow with guard_field shield (n = 5) and the WGS84 re-flights;
the people follow on PAS; the AerialVLA prefix A/B (12 x 420 s); Gazebo
functional sweep.

### 2026-11-18 (MSL flown) — the Shield's forecast stands over the DEM, an MSL rule flown on the ArduPilot SITL rail three times, the MSL altitude panel in the track video

#### Changed
- **Terrain-aware forecast over a DEM** (X-11; guardrail/shield.py,
  guardrail/terrain.py, guardrail/rtl_path.py, experiments/sweep_scenarios.py,
  sitl/ros2_shield_node.py, tools/msl_dem_rail.py,
  tests/test_terrain_forecast.py, docs/RUNBOOK-orin-hil.md step 11a).
  - Each forecast pose is judged at its own height above the DEM (predicted
    MSL minus the ground under that pose); MSL rules stay judged on the flat
    forecast. Until now the lookahead held the ground under the aircraft
    flat.
  - Over a DEM the altitude repair aims 0.25 m inside an AGL band
    (DEM_BAND_MARGIN_M), phased in over 1.0 s; the rule still asks for the
    edge. Closed-loop scratch sim: minimum AGL 8.000 -> 8.054 m.
  - The headless harness over a declared ground models RTL above home, as
    the ROS 2 node does (RTLWatch.assess(up_home), home_reached).
  - Headless rail run (docs/data/msl_dem_rail.json): independent-oracle AGL
    ticks 189 -> 0, minimum AGL 10.327 m; msl-ceiling requested on 2061 ticks,
    agl-band on 1836; nulls unchanged (Shield off 4798, MSL read as AGL 5902).
  - Without a ground 141/141 library episodes are byte-identical
    (164658 rows) at dc7f3fb against 7b0f50c, digests saved
    (docs/data/msl_rail_identity_20261118_wf20.json; the base digests equal
    the wf19 record's). filter() with a DEM costs about 2x (0.47 -> 0.96 ms
    without velocity); without one unchanged.
  - The stored rows of msl_dem_sitl_01 replayed through the new forecast
    flag agl-band at tick 106 (5.00 s ahead), 50 ticks before the flight's
    Shield did (tests/test_terrain_forecast.py 10/10).
- **The track video draws an MSL run in MSL** (tools/render_track_video.py,
  tests/test_render_track_video.py 19/19): for an up_ref terrain run the
  altitude panel's axis is MSL, with the ground, home and the AGL band
  following the ground; intervals outside a band are shaded on every frame.
  Stills of runs without a ground are byte-identical (10/10).
- **A run flown under an experiment profile resolves to its file**
  (guardrail/bundle.py policy_candidates, tests/test_policy_hash.py):
  experiments/policies/*.yaml are offered after every shipped policy and
  lock derivation, so msl_dem_sitl_01 resolves to
  experiments/policies/msl_dem_sitl.yaml (the lab checkout's
  test_policy_hash and test_wp1_roundtrip_kpi had failed on it).
- **CI:** test_terrain_forecast runs in the loop and is named in the header.

#### Flown (ArduCopter 4.5.7 SITL + ROS 2 rail, topology dev, not KPI-grade)
Bundle msl-dem-sitl-v0.1.0 (signed), ground
experiments/worlds/dem_cmac_wgs84.json (a synthetic ridge about CMAC, home
584 m MSL), goto pilot at 3 m/s, AGL band 8-60 m, MSL ceiling 624 m. Videos
`<run>_track.mp4` beside each run (msl_dem_sitl_01 re-rendered as
`_track_mslpanel.mp4`; the original kept).

| run | code | target | ticks under the 8 m AGL floor before RTL | min AGL before RTL | max MSL | FSM |
|---|---|---|---|---|---|---|
| msl_dem_sitl_01 | 5966c5c (before wf20) | up 15 | 20 | 7.514 m | 602.752 m | G1 Brake, G3 Loiter, G5 RTL |
| msl_dem_sitl_02 | dc7f3fb | up 15 | 0 | 10.822 m | 599.568 m | G1 Brake, G3 Loiter (vertical repair 0.51 m vs theta 0.50 m), G5 RTL |
| msl_dem_sitl_03 | dc7f3fb | up 50 (634 m MSL) | 0 | 14.984 m | 612.037 m | X1 Brake (vertical repair 1.94 m vs theta 0.50 m), G3 Loiter, G5 RTL |

- Each run: P0 escape rate 0.0 (P0 violation ticks 18, 4 and 2, each acted
  on), 0 ticks above the 624 m MSL ceiling, 0 false-trigger ticks. Ticks
  under the floor after that appear only during the RTL landing at home
  (175, 176 and 180). The FSM's Loiter holds the aircraft in GUIDED
  (BRAKE on the autopilot in msl_dem_sitl_01).
- msl_dem_sitl_03 stayed under the ceiling by escalation, not by a converged
  repair: the projection was abandoned 12 m under it.
- Max tick 5.828, 2.858 and 18.979 ms (the last a stale hold during a 1.14 s
  pose gap); max e2e 5.858, 2.9 and 50.609 ms (the last on the tick after
  that gap).
- **Still open:** none of the three reached its target: over the ridge the
  forecast asks for a climb larger than the FSM's vertical theta (0.50 m),
  so the projection is abandoned and the Loiter times out to RTL. The DEM is
  synthetic; no hil or PAS flight has carried an MSL rule, and in flight
  only an altitude_envelope carried MSL (MSL zones and corridors over a DEM
  are headless tests). The stress harness's sitl/hil backends, the mission
  node, gcs_backstop, bundle_origin and BAG_MANIFEST custom data refuse a
  ground; a replay tarball does not carry the DEM.
- **Appendix A:** X-11 stays PARTIAL after an independent review against the
  grant wording (2 PARTIAL, 1 DONE); its evidence and gap text updated.

### 2026-11-18 (code rows, wave 4) — one clearance path closer, the GUI writes WGS84, dev rails hand over the body frame, MSL on the ROS 2 rail; all four rows stay PARTIAL

#### Changed
- **One rule-evaluation path** (WP3-22, X-13; guardrail/shield.py,
  demo/semantic_demo.py, demo/semantic_seek.py,
  experiments/analyze_semantic_ab.py, tests/test_shieldcore_onepath.py).
  - The semantic demo's and semantic seek's minimum building distance and
    altitude band, and the semantic A/B P0 counter, now ask the Shield
    (min_clearance_on_map, altitude_band_report, Shield.rule_status). The
    counter names the rules it cannot judge (unjudged_rules), and each A/B
    row names its clearance measure.
  - follow_vlm.grid_scan_clearance moved into the library unchanged as
    nearest_cell_clearance. Bit-identical on 12,000 seeded comparisons over
    6 stored maps, and on 653 guard decisions of 10 stored AGX follow flights.
  - The test pins the library's two obstacle_clearance measures by
    behaviour, and FenceGuard's two threshold tests.
  - **Still open:** the library measures obstacle_clearance twice and
    FenceGuard flies the grid measure by default; `--guard-field shield` is
    not flown.
- **The Mission Control GUI writes WGS84** (WP1-14; demo/gui_policy.py,
  demo/gui_mission_control.py, guardrail/models.py, guardrail/bundle.py,
  tests/test_wgs84_ingest_writers.py).
  - policies/gui_policy.yaml is written in WGS84 about the base policy's
    frame origin, with the origin stated. Redrawing the shipped zones
    reproduces gui-mission@0.1.3.
  - Writer inventory (12/12): REST and PATCH on 32 policies (worst
    5.574e-08 m), harness spawns on 32 (worst 5.509e-08 m), the 1 shipped
    corridor, the gRPC path on 3 policies.
  - The legacy metre read form is labelled (Policy.point_form; run records
    get point_form/point_note). All 33 policies/*.yaml match the lock; all 41
    lab policy bundles keep their hash.
  - A conversion error is now logged instead of silently ending the Fly
    click.
  - **Still open:** a metre-only document still loads and flies (13 of the 41
    lab bundles); events on a metre-only policy and gen_random_scenario
    --metres can still write metres.
- **Dev rails hand the Shield the body frame where that is exact** (WP3-04,
  ARCH-04; guardrail/frames.py filter_as_body, tests/test_rail_body_frame.py).
  - City follow, pymavlink SITL, PAS ArduPilot and the headless harness use
    Shield.filter_body on every tick whose round trip is exact to the bit,
    else Shield.filter as before. Every decision and log row is unchanged to
    the bit, so earlier bundles re-fly byte for byte.
  - Test (10/10): harness 130 episodes, 85794 rows equal, 80200 ticks handed
    in the body frame; PAS fake flight 270 rows, 252 body; stored flights 8
    flights, 8580 ticks, 8247 body (local only).
  - **Still open:** repairs act in the world frame; city follow and pymavlink
    SITL use heading 0.
- **MSL on the ROS 2 rail** (X-11; guardrail/terrain.py load_ground,
  sitl/ros2_shield_node.py `--ground`, sitl/mavlink_adapter_node.py,
  sitl/ros2_vla_bridge_node.py, tools/msl_dem_rail.py, docs/RUNBOOK-orin-hil.md
  step 11a).
  - A WGS84 DEM loader (resample bound 3.653 m, sampled error 0.008 m) and an
    opt-in ground in the headless harness and the ROS 2 Shield node.
  - With `--ground` the node tells the Shield the height above the DEM, gives
    FENCE_ALT_MAX its ceiling above home (msl_dem_sitl: 81.5 m, raised to
    86.5 m; corrected 19 Nov), and judges alt_violation_s per envelope reference.
  - Headless rail run (docs/data/msl_dem_rail.json, 12 episodes per arm, seed
    1001): msl-ceiling requested on 2061 ticks, MSL-band ticks 0 (Shield off
    4798 oracle P0 ticks; MSL read as AGL 5902).
  - The independent oracle counts 189 P0 ticks on the AGL floor over the DEM,
    at most 0.081 m under it, level-cross pilot only: the Shield's lookahead
    holds the ground flat.
  - Without a ground: 141/141 library episodes byte-identical
    (docs/data/msl_rail_identity_20261118.json); tests/test_msl_rail.py 17/17.
  - **Still open:** no SITL, hil or PAS flight has carried an MSL rule; the
    DEM is synthetic.
- **Appendix A:** the six rows get the new evidence and gap text; totals
  unchanged at 107 DONE, 28 PARTIAL, 1 DRIFT, 0 MISSING.

### 2026-11-18 (the default backend named) — AIoT-VLA v10b for the follow task, after the measurement set; the team stated as in the grant across the docs

#### Changed
- **Default VLA backend named** (ARCH-30, X-04 → DONE;
  deploy/evidence/vla_backend_table.md/.json).
  - `tools/vla_backend_table.py table --default aiotvla_v10b --default-scope
    "the follow task" --default-date 2026-11-18`, run after the set's newest
    measurement (2026-11-17). The tool refuses the line unless every backend
    that can fly C2 has an Orin timing and 3 C2 flights and the stub has its
    R1 on the Orin.
  - The table states its own C2 numbers beside it: within 30 m 0.211-0.352,
    P0 escapes 0, KPI outcome 3 fail (unsafe_position_entered). It covers the
    follow task only; OpenVLA-7B stays the text-reading backend for the CSP
    experiments.
  - tests/test_vla_backend_table.py checks the committed default (48/48).
- **Final report and outline.** Section 6.4 names the default; decision 12 is
  closed. **Appendix A: 107 DONE, 28 PARTIAL, 1 DRIFT, 0 MISSING.**
- **Shield explainer figure.** The FSM box says Loiter is held in GUIDED on the
  plan's point by the companion (guardrail/hold_flight.py), replacing the
  stale "flown as ArduCopter BRAKE" line; docs/img/explainer_arch_shield.svg
  and .png regenerated.
- **Team wording.** docs/scope-clarification.md,
  docs/meeting-action-items-status.md, docs/agenda-technical-discussion.md,
  docs/LEARNING-ROADMAP.md and docs/PI-DECISIONS-2026-10-06.md state the team
  as in the grant (1 PI + 4 Master students, one team).

### 2026-11-17 (wave 3 and the FC bench) — gate G3 judged on the AGX Orin, PathRepair in the default chain, MSL over a DEM, one geometry library, CognitiveDrone/BitVLA adapters, an AP_DDS path, the report's dated rows, and the real flight controller on the bench

#### Added
- **Gate G3 judged from the stored hil flights** (ARCH-03 → DONE;
  tools/g3_gate.py, docs/data/g3_gate_20261117/).
  - **Passes:** AIoT-VLA v10b, with its server on the AGX Orin, on three C2
    hil flights:
    - round trip p95 84.7 / 84.2 / 84.0 ms;
    - 13.15-13.22 decisions/s while asked;
    - 0 failed requests.
  - **Fail:** the 7B backends that were measured. OpenVLA-7B bf16 is 969 ms
    p95 on the AGX Orin.
  - **Not measured:** int8 7B on the Orin.
  - The rate needs the stream feed with a 0.05 s capture.
- **PathRepair in the default repair chain** (WP3-06; guardrail/repair.py;
  docs/data/pathrepair_default_20261117/).
  - **Order:** the grant's, AltitudeClamp → LateralProjection → PathRepair.
  - **Magnitude:** on theta's depth basis.
  - **Where it acts:** in chain mode it detours round declared stand-off
    rings; boundary and zone detours are opt-in.
  - **Headless, before vs after** (smoke, library, nightly seed 1001):
    - P0 escapes 0;
    - fail-safe 63/63, 179/179 and 247/247, unchanged;
    - mission success unchanged.
  - **Flown behaviour changes:** the ROS 2 node takes `--path-repair`, on by
    default, so the next hil campaign flies it. The follow and perception
    rails keep it off.
- **MSL over a gridded DEM** (X-11; guardrail/terrain.py GridGround;
  tools/msl_dem_kpi.py, docs/data/msl_dem_kpi.json).
  - **Shield and CSP:** the Shield converts an MSL rule at the query point,
    and the CSP compiles MSL over the mission's ground instead of refusing
    it.
  - **Headless family:** 0 P0 true-escape ticks of 4,025 requested.
  - **50-rule MSL policy:**
    - cold start p95 509 ms;
    - hot-apply p99 13.9 ms;
    - query p99 2.14 ms.
  - **Opt-in:** no rail passes a ground yet.
- **One geometry library for the rule tests** (WP3-22, X-13;
  tests/test_shieldcore_onepath.py 17/17).
  - Every shapely, ray-cast and segment-distance rule test calls
    guardrail/{geometry,planar,shield,ir}.py.
  - Four clearance measures outside it are named in the test.
  - FenceGuard can read the Shield's distance field, opt-in and unflown
    (`--guard-field shield`). Open loop over stored AGX follows, its decision
    differs on 1.12 % of ticks.
- **CognitiveDrone and BitVLA adapters** (ARCH-06;
  demo/vla_server_foreign.py, `FAMILY=cognitivedrone|bitvla`).
  - Without weights, the server refuses to start.
  - **4-D conformance:** shown on scripted outputs only, in
    tests/test_vla_backends_cd_bitvla.py.
  - **Weights:**
    - CognitiveDrone has published none.
    - BitVLA has only robot-arm checkpoints, not downloaded.
- **An opt-in AP_DDS path** (ARCH-08 → PARTIAL; sitl/ap_dds/,
  sitl/ap_dds_adapter_node.py, docs/RUNBOOK-ap-dds.md). Not yet run on
  SITL.
- **The real flight controller on the bench** (WP3-07, WP3-10, ARCH-14
  evidence; docs/data/fc_bench_20261117/).
  - Setup: CUAV V5 Nano, ArduCopter 4.7.1, over USB, never armed.
  - **GCS backstop procedure:** fence parameters set and read back, and the
    keep-out zone uploaded as 4 items and read back matching.
  - **Mode requests:** GUIDED, BRAKE, LOITER, RTL, LAND and STABILIZE, each
    accepted.
  - **Restore:** parameters, fence and mission restored.

#### Changed
- **Final report and outline (WP1-21, WP3-01, ARCH-16, ARCH-17, ARCH-23,
  ARCH-24).**
  - These dated deliverables are reported on what was delivered and where
    it is.
  - The team is stated as in the grant: 1 PI + 4 Master students, one team.
  - ARCH-15 stays DRIFT on its transport.
  - tests/test_final_report_counts.py recounts both appendices.
- **Appendix A: 105 DONE, 30 PARTIAL, 1 DRIFT, 0 MISSING.**

### 2026-11-17 (pab2 and twelve branches) — the prefix A/B re-flown on hil after the path-test fix; replay, frames, ingest, null, envelopes, rails and sweeps made ready for their flights

#### Added
- **pab2, the prefix A/B re-flown on hil at 47b3fde** (WP4-15;
  docs/data/kpi_prefix_ab_20261117/). It is the re-fly that pab1 owed: 33
  missions of the `prefix_ab` profile, all KPI-grade. The pilot is the
  hand-written CSP-consuming adapter (guardrail.prefix_consumer), not a
  learned VLA.
  - **KPIs:**
    - P0 escapes 0 on 2172 P0 ticks.
    - Tick max 26.29 ms and end-to-end max 39.45 ms; 0 of 15464 ticks over 100 ms.
    - Fail-safe 15/15 under the adopted reading r2 (Wilson 95 % low 0.796); 12/15 as flown (the 3 nfz-head-on prefix-off runs).
  - **A/B, on vs off:**
    - Mission success 0.733 vs 0.600.
    - Repairs per episode 3.133 vs 130.600 (A/A largest pair 3).
    - Mean repair magnitude 0.505 vs 9.370 m/s.
  - **Against pab1:**
    - nfz-head-on prefix-on: now 4/4 with 0 repairs.
    - corridor-drift-out prefix-on: from RTL 3/3 to success 3/4.
  - Track videos are in each run folder.
- **The corridor finding behind pab2_011's RTL** (branch wf17-corridor;
  docs/data/kpi_prefix_ab_20261117/corridor_finding.md). Reproduced headless from the logs: all 34
  Brake-fallback ticks of the four prefix-on corridor flights were paths
  leaving the corridor's end cap by 0.005-0.30 m at t+5 s.
  - **Consumer side:**
    - Its admissibility test used the Shield's tube, which contains the true tube (the wrong side for an admissibility test).
    - It decided on a pose up to one tick older than the Shield's.
    - Fix: PrefixConsumer now keeps a margin of 1.0 m inside the wall (at least the speed cap times 0.25 s), and a path starting in the band must never pass the true wall.
    - The record announces `corridor_path_test: segment_margin` and `corridor_margin_m`. The harness refuses a prefix-on flight whose announced margin differs from the arm's.
  - **Shield side (not changed, pinned by a test):** CorridorReturn reads "returning" when a path closes on the end vertex before it leaves the cap.
  - **Owed:** unflown; pab3 re-flies the prefix-on arms after the 18 Nov demo.
- **Q3 runs re-run through today's Shield** (WP4-11; tools/replay_q3.py,
  docs/data/replay_q3_20261117/). Every run whose manifest is dated
  1 Aug-31 Oct 2026, 5509 folders:
  - **Re-run, open loop on the logged inputs with every policy generation:** 15 runs, 10 identical bit for bit and 5 differ.
  - **Not replayable, each with its reason:** 5494, most of them headless episodes that keep no per-tick log.
  - **What this is not:** the grant's reproducibility replay, because none of the 15 was flown from a clean revision.
- **Estimator null for C2** (ARCH-30, X-04; FAMILY=estimator in
  deploy/orin/run_pas_perception.sh, EstimatorFollowPilot in
  demo/pas_ardupilot/perception_node.py).
  - **What it is:** the lab's hand-written follow law with its target estimator, strict lock, re-acquisition and trail-following forward direction, on the same detector feed, start gate and end rule as the C2 VLA flights.
  - **What it is not:** equal to the lab's follow. It lacks the search planner, FenceGuard and the orbit, and the protocol says so.
  - **Status:** not flown. The backend table records it as a second, separate null.
- **Arc forecast and body-frame paths, opt-in** (WP3-04, ARCH-04).
  - **Arc forecast:** with `arc_forecast` on, the Shield also forecasts a body action with a yaw rate along its arc, beside the straight forecast. It can only add caution: the straight poses are bit-identical to the forecast with the arc off.
  - **Where it can be switched on:**
    - the PAS rail (`--action-frame body`);
    - the ROS 2 rail and the campaign launcher (`--arc-forecast` / `--rail-arc-forecast`);
    - the headless API.
  - **Default:** off, so no existing flight changes.
- **Metre writes stored as WGS84** (WP1-14). Every write path into a framed policy now stores and hashes WGS84 only: REST hot-apply, the Shield's zone edits, the harness's spawned zones and `bundle.derive`.
  - **Enforcement:** by default it uses the metres given. The snapshot reproduces them to within 0.6 um under the same hash.
  - **Opt-in `VLAGUARD_WGS84_BIND`:** enforces from the WGS84 view; off until the smoke profile is re-baselined.
  - **Stored policies:** their hashes are unchanged.
- **distance_envelope on the ROS 2 hil rail** (WP1-07; `--world` with
  experiments/worlds/de_block.yaml, profile distance_envelope, signed
  bundle distance-envelope-sitl-v0.1.0).
  - **What is judged:** buildings and roads rules, against a declared world of rectangles. Nothing physical stands there in SITL.
  - **City rails:** roads are judged against the real CityLife street mask.
  - **Not flown yet.**
- **Gazebo functional sweep plumbing** (WP4-04; profile gazebo_functional,
  `--rail-physics gazebo`, `--sitl-dry-run`).
  - **Dry run:** lists 8 of 8 missions.
  - **Marking:** every mission is marked functional, never KPI-grade.
  - **Speed:** only sim_speedup 1 is accepted, because the Shield node runs on wall time. The grant's broad-sweep speed > 1 is not delivered.
  - **Not swept yet.**
- **G6's battery reason flyable on the rails, opt-in** (WP3-07; `rail_battery`, profile fsm_edges only). The simulator side sets the simulated vehicle's BATT_* fail-safe and restores it afterwards. KPI-bearing profiles refuse the cell as before. The ROS 2 node now applies `for_decision_period` when a pilot declares one.
- **Per-host revisions** (ARCH-31; stack_revisions.json beside an
  unchanged six-field manifest).
  - **What it records:** the revision of each host (desktop, companion), the autopilot firmware as AUTOPILOT_VERSION names it, and the simulator build where it is known.
  - **`single_sha`:** true only when every host the topology needs has reported on one clean commit.
  - **Writers:** the ROS 2 Shield node, the ROS 2 mission node and the Project AirSim + ArduPilot node.
- **Broad sweep runnable unattended** (WP4-06; `--resume DIR`, `--shard I/N`,
  `--merge-only`).
  - **How:** checkpoints are written per episode and resumed after a kill; four shards cover every family.
  - **Report:** it says the broad results are outside the KPI numbers.
  - **Run so far:** a 250-episode slice only; the full sweep (14,950 episodes) has not run.

#### Fixed
- **tools/replay_from_bag.py over ntFIN2 + ntFIN2s: 590 identical, 1 differs** (16 Nov: 571 / 18 incomplete / 2 differ). Two causes were in the replay itself:
  - **The 18 dyn-nfz-spawn-ahead runs:** after the GeoFence takeover the FSM does not step, and a row without FSM keys now agrees with an intercept carrying `fsm: null`.
  - **ntFIN2_511:** the first tick is placed on the flight log's first action.

- **Setpoint delivery check** (guardrail/delivery.py, sitl/mavlink_adapter_node.py, sitl/ros2_shield_node.py; docs/data/delivery_gaps_hil_smoke_20261117/).
  - **Finding:** in ntFIN2_175 the Shield published setpoints that the MAVLink adapter did not hand to MAVROS for 25.5 s. That is 686 published and 441 sent; 245 never went out. Speed held about 2 m/s for the GUIDED timeout, then fell to about zero. The run was KPI-grade.
  - **What is added:**
    - The adapter now reports each send, plus a 1 Hz beat, on /mavlink_adapter/sent.
    - The Shield node compares those reports with what it published and records `setpoint_delivery` in metrics.json.
  - **Grading:** a gap over 0.5 s, or a shortfall over 2, makes a run not KPI-grade. Stored runs are graded from their two logs.
  - **Scan of 1837 stored hil folders:** one gap (ntFIN2_175) and 1835 clean.
  - **Not known:** the gap's cause; the logs show only that the adapter's setpoint callback did not run.
  - **Not monitored:** the MAVROS-to-autopilot hop.
- **ntFIN2 KPI report regraded for delivery** (docs/data/kpi_ntFIN2_20261117_delivery/). Nothing was re-flown, and only ntFIN2_175 moves:

  | Figure | Before | After |
  |---|---|---|
  | KPI-grade episodes | 589 | 588 |
  | Fail-safe (r2) | 584/585 | 583/584 = 0.998, Wilson low 0.990 |
  | P0 escapes | – | 0, on 16,746 P0 ticks |
  | Tick max | – | 74.06 ms |

  The 15 Nov report stays as it was; this is the one to cite.

#### Changed
- **The Shield enforces what is stored: hot-applied geometry is bound to the WGS84 frame** (WP1-14, WP4-11; docs/data/wgs84_bind_default_20261117/).
  - **The problem:** every write path already stored WGS84. The Shield, however, enforced the metres an event gave, so a hot-applied generation replayed from its stored snapshot was off by up to 0.6 um.
  - **The change:** binding is now the only mode, so a generation replays bit for bit (tests/test_replay_q3.py by default).
  - **Exact ties:** an exact tie on a bound dynamic zone is judged at the geometry's resolution, 2e-6 m. Static fences keep 1e-9 m.
  - **Headless check, three sets:**

    | Set | Episodes that differ | What differs |
    |---|---|---|
    | Smoke | 8 of 64 | sub-micrometre actions, recovery records, anti-stall slide side |
    | Library | 14 of 180 | same |
    | Nightly cells, seed 1001 | 29 of 248 | same; in 5 episodes FSM edges come 0.1-0.2 s apart |

    - Every outcome, status, fail-safe label, P0 figure and FSM edge sequence is unchanged.
    - End positions move by up to 0.8 m.
  - **REST:** a body whose lat/lon and x/y disagree by more than 1 um is refused with HTTP 422.

#### Not changed
- Apart from the WGS84 binding above, no branch changes what an existing KPI scenario flies or the headless smoke result: every other new behaviour is opt-in and off by default, and the delivery check changes grading, not flight.
- The AGX stays at 47b3fde for the 18 Nov demo. The flights these branches prepare (pab3, the estimator null, distance_envelope, the battery cell, the Gazebo sweep, a delivery and stack-revision shakedown after the companion image is rebuilt) are flown after it.

### 2026-11-17 (RaceVLA's third C2 flight) — every backend that can fly C2 now has its 3 flights on hil

#### Added
- **RaceVLA c2_06 on hil at 89d2dc2** (ARCH-30, X-04;
  deploy/evidence/vla_backend_table.md regenerated). It passed the start
  gate on the first attempt, flew 899 ticks and lost the car (target_lost):
  within 30 m 0.063, 0.0223 decisions/s over the mission, P0 escapes 0,
  KPI outcome success. RaceVLA now has 3 C2 flights (within 30 m
  0.060-0.168, median 0.063); its 3 start-gate timeouts stay censored, not
  counted. Its 90 s video (and a copy under 30 MB) is in the offload folder
  videos_c2_20261116 with the other C2 videos.
- With it, AIoT-VLA v10b, AerialVLA bf16, DroneCATS-2B and RaceVLA each have
  the 3 C2 flights the protocol asks for, every one with P0 escapes 0, and
  the table no longer lists anything the set lacks before `table --default`
  can name a default. The end-rule replay behind the null's second column
  now ends each of the 12 C2 VLA flights within 2.2 s of where its own
  monitor did.

#### Fixed
- tests/test_vla_backend_table.py no longer pins the old state of the
  committed table ("the set lacks something"): it checks that the listed
  gaps are the ones the tool computes, that the md names them exactly when
  there are any, that there are none now, and that no default is named.

#### Not changed
- **Default backend: still none.** The lab's 15 Nov choice (AIoT-VLA v10b
  for the follow task) was made before this set landed; the grant asks for
  the choice on the set, so it is named with `table --default` only when
  the lab confirms it on a day after 17 Nov (the tool refuses a
  --default-date not after the set's newest measurement). The null flown in C2 is still the
  box-servo, not the lab's estimator follow.

### 2026-11-17 (C2 top-up and what it showed) — RaceVLA's fourth C2 flight and the C2 null flown on hil; the null and the start gate read for what they are; ci_local made to run what CI runs

#### Added
- **C2 top-up on hil at 63fffcf** (ARCH-30, X-04, ARCH-06;
  deploy/evidence/vla_backend_table.md regenerated). RaceVLA c2_04 flew 916
  ticks and lost the car (within 30 m 0.168); c2_05 never started (start
  gate timed out). The C2 null, the hand-written follow (FAMILY=servo), flew
  3 flights of 420 s after a shakedown whose start gate timed out: each ran
  to max_s, P0 escapes 0, within 30 m 0.105 / 0.065 / 0.111. RaceVLA now
  has 2 C2 flights (0.060-0.168) and 3 gate timeouts; 1 more is owed.
  Videos of the 12 flown C2 flights (and copies under 30 MB) are kept off
  the repository, in the offload folder videos_c2_20261116.
- **The C2 null in the table as flown** (tools/vla_backend_table.py). It is
  the perception node's FollowPilot: a box-width P-servo (want_width 0.10,
  speed_max 4.0) with no target estimator, no lock and no lost end. It
  commands at most about 2.5-3.0 m/s against a car at about 3.2 m/s, falls
  to about 45 m by 60 s, loses the box, and hovers; most of its later
  within-30 m ticks come from the car looping back past it. It is not the
  follow_vlm estimator follow behind the older 0.95-0.98 hand-written
  numbers. The table lists it in a C2 table of its own, with its share also
  under the VLAs' own end rule (90 s after the last fresh box): 0.105 /
  0.030 / 0.099, shown because a replay of that rule ends every C2 VLA
  flight within 2.2 s of its recorded end. As flown it sits about level with
  AerialVLA and RaceVLA; only AIoT-VLA is clearly ahead.
- **Start-gate timeouts counted as censored** (tools/vla_backend_table.py).
  A flight that would be C2 but whose start gate timed out is listed per
  backend and not counted toward the 3, so no backend's n shrinks
  silently: RaceVLA 3, every other backend 0.
- **`drops` on every detections.jsonl row** (demo/follow_vlm.py
  screen_candidates): what the threshold, the jump gate and the colour gate
  removed: a count for each, and the best score and box of what the
  threshold and the colour gate removed.
  It cannot say whether a dropped box was the car. The passing boxes are
  unchanged (tested against the old loop).

#### Fixed
- **The start gate stated a rule it does not decide by**
  (demo/pas_ardupilot/perception_node.py StartGate). With --identity the
  decision is the Reacquirer's: 10 identity-ok boxes in a row of one object
  within 30 m and seen moving; one inference with none acceptable restarts
  the streak. The node printed the pixel counter's streak and "5 inferences
  in a row". It now states and records the rule it decides by, the identity
  gate's streak, its step outcomes and how often the moving test refused.
  Each RaceVLA timeout was one inference with an empty box list mid-crossing
  at the same image position; that pattern is in 4 of 6 RaceVLA gates and 0
  of 13 others (16 Nov). Its cause is not known; `drops` records the next
  occurrence. No gate decision changed.
- **TARGET LOCKED with no lock.** The HUD said TARGET LOCKED for the servo
  null, which holds no lock. It now says TRACKING BOX (NO LOCK) there, and
  TARGET LOCKED only when a lock is held (perception node and
  tools/rerender_policy_hud.py).
- **ci_local did not run what CI runs** (tools/ci_local.py). It ran only
  the workflow's standalone loop, so test_policy_hud (the Pillow step) and
  the desktop-runnable files the loop leaves out never ran locally, and with
  nothing pushed they never ran at all. Two real failures went unseen:
  test_policy_hud pinned the NFZ policy at 0.1.2 (0.1.3 since 4ae620a), and
  test_two_view asserted an import line 63f9072 had changed. ci_local now
  runs the Pillow step (its env included) and those files as desktop extras,
  each once; an extra does not gate only when every failure in it is the
  missing package its exclusion names. The loop is split as bash splits it:
  tests.yml had a literal "\n" twice in the loop, which bash would run as a
  file named "n" (fixed). Both stale tests are fixed.

#### Found, not fixed
- **RaceVLA's gate misses.** One empty inference mid-crossing, RaceVLA only
  (see above); cause unknown.
- **The null is weaker than the lab's hand-written follow.** A fair null
  would be the estimator follow on the PAS rail (the node refuses
  --target-estimator for the servo today); C2's null stays as flown and
  labelled until then.

### 2026-11-16 (evening: prefix A/B and C2 flown on hil) — the prefix A/B campaign pab1 and the backend table's C2 flights, a corner cut found and fixed in the CSP-consuming pilot, and the C2 null made flyable

#### Added
- **The prefix A/B on hil, campaign pab1** (WP4-15;
  docs/data/kpi_prefix_ab_20261116/). 33 missions of the prefix_ab profile
  flown at c7257f1, all 33 KPI-grade: P0 escapes 0 on 2223 P0 ticks; tick
  max 24.23 ms and end-to-end max 24.32 ms (0 of 14756 ticks over 100 ms);
  fail-safe reading r2 15/15 on the labelled prefix-off arm (Wilson 95 % low
  0.796; the prefix-on arm carries labels_not_applied). On vs off: mission
  success 0.600 / 0.600, repairs per episode 6.6 / 130.5 (A/A largest pair
  7), mean repair magnitude 1.78 / 9.38 m/s. The pilot is the hand-written
  CSP-consuming adapter, not a learned VLA, so part of the delta holds by
  construction (the report says so).
- **The backend table's C2 flights** (ARCH-30, X-04, ARCH-06;
  deploy/evidence/vla_backend_table.md regenerated). Flown on hil at c7257f1,
  each after a 60 s shakedown: AerialVLA bf16 3 flights, DroneCATS-2B 3,
  RaceVLA 1 (2 more never started: the start gate found no red car within
  30 m in 330 s), AIoT-VLA v10b 1 (3 with feed_07/_08). P0 escapes 0 on every
  flight. Every C2 flight of 16 Nov lost the car (target_lost) before 420 s.
  Share of ticks within 30 m of the car: AIoT-VLA 0.211-0.352, AerialVLA
  0.052-0.163, DroneCATS 0.132-0.189, RaceVLA 0.060. The table still names no
  default.
- **FAMILY=servo** in deploy/orin/run_pas_perception.sh: the hand-written
  follow (--pilot servo), the protocol's C2 null, which the script could not
  fly before (no VLA option, no server check).

#### Fixed
- **The CSP-consuming pilot cut a dog-leg's inner corner between its
  samples** (guardrail/prefix_consumer.py). On pab1's corridor-drift-out the
  prefix-on arm ended in RTL 3/3 (off: success). Reproduced headless to
  5e-4 m on every repair tick of pab1_008: the consumer tested admissibility
  every 0.5 s, so its 60-degree heading cut the corner for 0.21 s and about
  0.25 m between two samples; the Shield's 0.1 s forecast caught it, and
  CorridorReturn sized the cut as a straight line along the nearest leg,
  2.48 m, over theta 2.0 m: G3 Loiter, X1, then G5 RTL on the loiter timeout.
  The consumer now tests the whole straight path to the lookahead against the
  Shield's corridor tube and the zones' margin rings. Headless,
  corridor-drift-out's prefix-on arm has 0 repairs and no FSM edge, and
  nfz-head-on's (the same mechanism at a zone's ring) 0 repairs where it had
  19 and two Loiters. A rig whose consumer does not announce the path test is
  refused as not the arm's flight. The Shield is unchanged: its corner sizing
  (about 10x the true excursion, the safe direction) is pinned by a test as
  found; changing it would move the certified readings, so it waits until
  after 30 Nov. pab1 was flown before this fix; its prefix-on arms are to
  re-fly.
- **kpi_report read an unlabelled run as correct** (tools/kpi_report.py): a
  run the re-score names in no reading list (the prefix-on arm) counted as
  correct, so the first pab1 report showed 33/33 where the table has 15/15.
  Such runs are left out by name, under every reading and in a merged
  table's run count.

#### Found, not fixed
- The node log of 5 of 53 AGX flights ends "terminate called without an
  active exception" after the report is written (pas_hil_aiot_fog_01,
  _night_01, pas_hil_obs_01, the RaceVLA shakedown and c2_01): an abort at
  process exit; no record is lost.

### 2026-11-16 (engineering for eight open rows) — the backend measurement set to protocol 2, the bundle signer under the lab CA, a replay check from the rosbag2 capture, and a pilot that consumes the CSP for the prefix A/B; no row changes status until the rig steps are run

Four branches built from the verified plans of the contract re-read
(backend-table, bundle-ca, replay-from-bag, prefix-ab), each implemented,
reviewed by a second engineer who tried to refute it, and fixed.

#### Added
- **VLA backend measurement set, protocol 2** (ARCH-30, X-04, ARCH-03,
  ARCH-06; tools/vla_backend_table.py, deploy/evidence/vla_backend_table.md
  regenerated). New rows for AIoT-VLA v10b, DroneCATS-2B, RaceVLA-7B,
  AerialVLA bf16 and OpenVLA-7B bf16; the AGX Orin benches already on disk
  folded into the Orin column, each labelled with its own inputs; closed-loop
  test C2 (the Project AirSim follow on `hil`, read from each run's own
  metrics, params and kpi.json, including the run's own KPI verdict); the
  27-28 Oct and 3-4 Nov matrices folded in as non-ArduPilot records. The
  table says C2 was written on 16 Nov, after the lab's choice of 15 Nov. It
  still reads "Default backend: none": `table --default` is refused until
  every backend that can fly C2 has an Orin timing and 3 C2 flights and the
  stub has R1 on an Orin. Measured on the AGX Orin on 16 Nov (MAXN, the
  AIoT-VLA server stopped for the run; deploy/evidence/hil/vla_rate_*.json,
  *_bench_agx.json): R1 stub p95 0.00419 ms, bc_v3 p95 4.52 ms, OpenVLA-7B
  bf16 p50 956 ms / p95 969 ms (n=50, under 10 Hz by a factor of about ten:
  the case the grant's gate G3 names); the int8 weight-only attempt refused at
  import (torchao is not installed there); server benches, not R1: DroneCATS-2B
  round trip p50 1348 ms (model p95 17526 ms), RaceVLA-7B p50 839 ms. What the
  set still lacks is C2 flights (AIoT-VLA 2 of 3; AerialVLA, DroneCATS,
  RaceVLA 0 of 3). Both C2 flights of AIoT-VLA so far
  (pas_hil_feed_07 / _08) have 0 P0 escapes and a KPI outcome of fail
  (unsafe_position_entered: 3 and 21 ticks in a P0-unsafe position); the table
  prints both. `measure` gains openvla_7b_bf16 (no bitsandbytes, loads like
  deploy/orin/vla_bf16_check.py) and an int8 weight-only attempt that records
  its refusal. deploy/orin/run_pas_perception.sh gains `FAMILY=` (aiotvla,
  the default, gives the same argv as before; aerialvla, dronecats, racevla
  are parser-checked only).
- **The bundle signer under the lab root CA** (WP1-04, X-10;
  guardrail/bundle.py, guardrail/harness_service.py issue_policy_signer,
  tools/build_flight_bundles.py --issue-cert, tools/bundle_ca_evidence.py).
  A certified signer writes signed_by = lab-ca:<CA fingerprint>:ed25519:<key
  id> and a `signer` record under the signature; the chain is checked with
  `cryptography` where it is installed and recorded as not-checked-here where
  it is not; the swap drill (a second CA: one entry and a re-sign, no code
  change) and forged, wrong-CA and mismatched leaves are tested. The real keys
  are not certified yet and no real bundle was re-signed: trusted_signers.json
  is unchanged, so every stored bundle and run verifies as before (read-only
  check on this disk: 41 policy bundles verified, 231 replay bundles verified,
  94 replay bundles unsigned as before, 0 refused). Requiring the chain on a
  `hil` loader is opt-in (VLAGUARD_REQUIRE_CHAIN, shipped 0).
- **Replay check from the rosbag2 capture** (ARCH-22; tools/replay_from_bag.py,
  `python -m guardrail.replay verify --from-bag`). From a run's bag alone it
  rebuilds each tick's pose, VLA action, Shield setpoint, executed setpoint
  and mode, re-derives the trajectory and 13 KPI fields (5 more with the
  policy's rule priorities) and compares them with the run's log and
  kpi.json; 43 KPI fields, among them the P0 escape rate and fail-safe
  correctness, are not re-derivable from the bag and are reported so. It is
  a check against the capture, not a re-flight. Run over every run the KPI-grade nightly accepted (ntFIN2 + ntFIN2s, 591 runs; docs/data/replay_from_bag_ntFIN2/): 571 identical, 18 incomplete (all dyn-nfz-spawn-ahead, whose log rows do not record the FSM state), 2 differ (ntFIN2_175: a setpoint the log says was streamed is not in the bag at tick 82; ntFIN2_511: the bag holds one VLA action more than the log), trajectory identical on all 591 (0 of 285,707 pose points different); no bag carries the episode manifest.
  sitl/bag_custom_data.py writes the manifest fields known at launch into the
  bag's metadata, off by default (BAG_MANIFEST / VLAGUARD_BAG_MANIFEST); not
  yet run on the Jazzy recorder.
- **A pilot that consumes the CSP, for the prefix A/B** (WP4-15;
  guardrail/prefix_consumer.py, sitl/ros2_vla_stub_node.py --prefix on|off,
  the harness's arm record `pilot_reads_prefix`, tools/kpi_report.py's A/B
  section with an A/A noise floor, experiments/profiles/prefix_ab.yaml). The
  stub node's default is unchanged. Headless run of the profile (functional,
  not KPI-grade; 15 on/off pairs that are 5 distinct flights, 3 A/A pairs all
  identical): mission success 1.000 on / 0.600 off, P0 escapes 0 / 0, repairs
  per episode 4.2 / 184.2. The pilot is a hand-written adapter, not a learned
  VLA, and part of the delta holds by construction; the report prints that and
  the one learned-VLA A/B (OpenVLA-7B, 7 Nov), which is adverse.

#### Limits
- The rig steps that would close rows are not run: R1 and the DroneCATS /
  RaceVLA benches on the AGX Orin, the C2 flights, certifying the two real
  keys and re-signing, a `hil` mission with the chain required, the prefix
  A/B campaign on `hil`, the bag manifest on the Jazzy recorder.
- The prefix A/B runs in its own profile, not in the nightly stress run, so
  the grant's "always-on A/B during stress runs" is not shown.

### 2026-11-16 (contract map re-read) — the 39 open rows read against the grant by five readers and five adversarial second readers: ARCH-02 to DONE (98 / 32 / 5 / 1), row texts and report sentences that had gone out of date corrected, the reference repository's 5 July history recorded for the mid-term rows

Each of the 39 rows that was not DONE (33 PARTIAL, 5 DRIFT, 1 MISSING) was
read against its grant line, the code and the evidence files, then a second
reader tried to refute the first one's conclusion. This entry holds what
that reading changes in the documents; the engineering it ranked is separate.

#### Changed
- **ARCH-02 to DONE.** "ArduPilot is the inner loop on every rail" was
  PARTIAL on evidence that stopped at 29 Oct. Every rail of the grant flies
  ArduPilot: the KPI rail (ntFIN2 + ntFIN2s, 589 episodes), Gazebo Harmonic
  (2 Nov) and the Project AirSim perception rail, with a learned VLA on `hil`
  since 14 Nov. The simple_flight follow comparisons are a bench outside the
  delivery, labelled so. Counts: 98 DONE / 32 PARTIAL / 5 DRIFT / 1 MISSING.
- **Row texts brought up to date** (no status change): WP1-05 (32/32, not
  29/29), ARCH-19, WP4-17 (the 15 Nov reading; its A/B and per-paraphrase
  sections read n/m), X-01 (the report exists), ARCH-23, WP2-13 (the harness
  draws paraphrases from the service), WP3-07 (the flicker scenario is
  labelled E6; line references), ARCH-08, ARCH-17.

#### Fixed
- **Report sentences that were no longer true** (docs/FINAL-REPORT.md): "No
  learned VLA has emitted the 10 Hz setpoint stream into ArduPilot", "A
  learned VLA on this rail: not flown", "it has not flown on `hil`", "has not
  flown. Flown 14 Nov", and simple_flight listed among the rails. The outline
  still said Gazebo Harmonic was "not built".
- **Section 7.1 said git dates nothing before 2026-08-24.** True of this
  repository only. The PI's reference repository has a Policy DSL with a
  typed IR, a Shield package with two repair operators and a ROS 2 Shield
  node at its first commit, 9e039b0 of 2026-07-05, and the developer's fork
  commits of 8-10 July (docs/data/midterm_gate_evidence/evidence.md). WP1-21,
  WP3-01 and ARCH-15 cite it and stay DRIFT: those are the reference
  implementation's dates, and whether they count for the gate is the PI's to
  say.

#### Limits
- When 9e039b0 was pushed to its origin was not checked.
- ARCH-02's DONE is this project's reading, checked by a second reader; the
  PI has not reviewed it.

### 2026-11-15 (10 Hz observation) — a new camera frame goes to the VLA between the loop's ticks: 12.4-13.2 frames/s reach AIoT-VLA on the AGX Orin while it is asked (9.58 before) and it decides 10.6-10.9 times a second over two full missions; X-05, ARCH-25, ARCH-01, ARCH-21 to DONE; the perception node's collector paced out of the tick

The grant asks for camera observations into the VLA at 10 Hz or faster. The
pilot sent at most one request per tick and the loop ticks at 9.98 Hz, so
the rate could not reach 10 however fast the camera (14.7 Hz) and the
server (round trip p50 58 ms) were.

#### Added
- **The feed between ticks** (`--vla-feed stream`; guardrail/vla_backends.py
  `RemoteVLABackend.pump`, `new_frame`, `feed_max_hz`; demo/pas_ardupilot/
  perception_node.py `pace_feeding`, `VLAPilot.pump`; demo/vla_mission.py
  `VLAFrames.front_token`). In its wait for the next tick the node pumps the
  backend every 4 ms (not in the last 15 ms): a finished reply is kept for
  the next act(), and the next request goes out when the front frame is one
  not yet considered. Answers still enter the tracker in act(), at that
  tick's pose, in the order sent; one request is outstanding at a time; a
  rail hold, LAND, the stop rules and the back-off hold as before. The node's
  default stays `tick`; deploy/orin/run_pas_perception.sh passes `stream`
  (VLA_FEED=tick for the old behaviour). `--vla-feed-max-hz` caps it
  (default 0: no cap).
- **The collector paced in the PAS perception node** (`--gc-pacing on`, the
  default; metrics.json `gc`). GcPacer moved unchanged from
  sitl/ros2_shield_node.py to guardrail/gc_pacing.py and both nodes use it.
- Tests: tests/test_vla_backends.py (2), tests/test_pas_vla.py (2),
  tests/test_deploy_configs.py (the rig's default).

#### Flown (hil, AIoT-VLA v10b on the AGX Orin, KPI-grade, camera at a 0.05 s capture interval; docs/data/pas_hil_feed_20261115/)
- **pas_hil_feed_01 / _02** (ea8ef42, cap 12 Hz): 9.97 and 9.83 frames/s
  while the model was asked. The first version capped the feed alone, so
  the tick's own request re-armed the cap every 100 ms and the feed stayed
  locked to the tick (38 of 293 and 20 of 283 requests sent between ticks).
  Fixed in d9f5ea0: one gate for both.
- **pas_hil_feed_03 / _04** (d9f5ea0, cap 12 Hz): 10.03 and 10.01.
- **pas_hil_feed_05 / _06** (d9f5ea0, no cap): 13.23 and 12.39, the
  detector's rate (2.70 and 2.31 Hz) and the tick's work (median 35.9 and
  37.9 ms) as without the feed (2.18 Hz, 35.5 ms on pas_hil_obs_02). Desktop
  tests ran beside these two flights for a few minutes.
- **pas_hil_feed_07 / _08** (deac4b3, the rig's default since: no cap, the
  collector paced): both flew the full 420 s with the car held. 13.15 and
  13.22 frames/s while asked, 11.07 and 10.72 over the whole mission;
  decisions parsed 10.93 and 10.62 per second over the mission; 284 and 332
  interventions, 0 P0 escapes; camera to first setpoint p50 134 and 130 ms
  (158 on pas_hil_obs_02).
- **The collector**: before the pacing the loop had one tick of 227-454 ms
  every 65-80 s (pas_hil_feed_04: 7 of 4014 ticks over 100 ms,
  pas_hil_feed_05: 6 of 3192). Paced: 0 automatic full collections, the 41
  paced ones at most 19.9 ms, run after the tick's work; ticks over 100 ms 3
  of 4058 (152, 112, 148 ms; not collections) and 0 of 4096.

#### Changed
- Appendix A: X-05 (was DRIFT), ARCH-25, ARCH-01 and ARCH-21 to DONE;
  ARCH-03 stays PARTIAL (the 7B backends do not decide at 10 Hz).
  97 DONE / 33 PARTIAL / 5 DRIFT / 1 MISSING. What the DONE rests on: a
  15 Hz camera, and the detector having the target (while it has no box the
  model is not asked, by design; the detector runs at 2.3-2.7 Hz); the
  mission-wide rates are from n = 2 full-length flights.

### 2026-11-15 (outline figures) — the outline's fail-safe lines brought to the KPI as it stands since the backstop rule

#### Changed
- docs/FINAL-REPORT-outline.md: the "cannot claim yet" line, the KPI table row
  and the note on WP3-15 quoted 566/567 as the current figure. They now give
  584/585 = 0.998 (the adopted reading with the GeoFence backstop rule,
  settled by the lab and not yet confirmed by the PI), keep 566/567 as the
  figure before the rule, and name what is still unscored (1 censored run,
  2 cell-seeds; 584/588 = 0.993 were all 3 errors). No status changes.
- The lab CI record of 54994fa (151/151 files) is committed.

### 2026-11-15 (PI decisions) — the 15 open decisions settled by the lab under the PI's instruction of 6 Oct (follow the grant; best practice where it is silent); two of them change the fail-safe scoring: the grant's G4 gets a rubric clause, and a GeoFence RTL with the FSM holding counts as a fail-safe triggered by the backstop (584/585 = 0.998); WP3-15 to DONE

None of this is the PI's signature. Each row of docs/FINAL-REPORT.md section 9
says what the project now does; the PI's confirmation is still to be recorded,
and an item whose grant wording is not met keeps its status.

#### Changed
- **Decision 13: the grant's G4 governs a thrash of N-in-T onsets.** New
  rubric clause **E6 "N-in-T persists"** (docs/RUBRIC-failsafe.md): onsets of
  a hard violation made by the world (a hard rule switched on or over the
  request, a world event on the tick the violation starts) recur at the FSM's
  N-in-T rate through a Loiter. A planner's forecast re-touching a ring stays
  N1, noise in the Shield's view of position stays N2. `fsm-n-in-t-flicker`
  is labelled expected (E6); over every family cell and every episode of the
  library, smoke, nightly, grant_example, fsm_edges and broad profiles it is
  the only label that changed. The clause was written after hs10_003 flew,
  which it scores as a correct trigger. tools/failsafe_rubric.py: 0 label
  mismatches on every profile.
- **Decision 14: an ArduPilot GeoFence RTL or LAND while the FSM holds is a
  fail-safe triggered by the backstop.** `experiments/sweep_scenarios.py`
  `failsafe_backstop` derives it per run from the run's own records (the
  takeover's mode, a fence-breach text within 2 s of it, the FSM in Brake or
  Loiter with no fail-safe of its own) and scores it against the scenario's
  label like any other; `failsafe_by` names who triggered it, and the count
  of backstop runs and both earlier readings are printed wherever the KPI is.
- **The KPI re-scored** (ntFIN2 + ntFIN2s, 586 KPI-grade Shield-on flights,
  at f08f16d; docs/data/kpi_ntFIN2_20261115/): reading (r2), adopted,
  **584/585 = 0.998**, Wilson 95 % lower bound 0.990; 18 of them triggered by
  the backstop, all dyn-nfz-spawn-ahead with the zone 3 m ahead and all
  expected (18 correct, 0 false). With the 18 left out, as before: 566/567;
  with them as missed: 566/585 = 0.968. Still censored: ntFIN2_374. Still
  missed: ntFIN2_177. P0 escape, the tracked KPIs and the tick budget are as
  in the 13 Nov report. The 18 runs still fail their own gates (3.4-3.6 s
  inside the hot-applied zone); the rule changes who is credited with the
  fail-safe, not the breach.
- Appendix A: WP3-15 to DONE (584/585; were the censored run and the 2
  unflown gps-dropout cell-seeds all errors the point estimate would be
  584/588 = 0.993); 93 DONE / 36 PARTIAL / 6 DRIFT / 1 MISSING. The report's
  section 9 lists the 15 decisions as settled, names the five final-demo
  scenarios (9.1) and says what only the PI or ITRI can give.

#### Flown
- **pas_hil_obs_02** (decision 10; 9dfe254 on the desktop, de7e748 on the AGX
  Orin, which differ in documents only; the PAS follow with AIoT-VLA,
  `-FrontCaptureIntervalS 0.05` instead of 0.066): KPI-grade, 968 ticks at
  9.981 Hz, 15 interventions, 0 P0 escape ticks, ended target_lost. The
  camera delivered 14.712 Hz (12.823 on pas_hil_adapter_01) and frames
  reached the VLA at 9.584 Hz while it was asked (9.266 there; 9.10-9.50 on
  14 Nov): a faster camera does not reach the grant's 10 Hz, the pilot's
  asking cadence bounds it. X-05 stays DRIFT. pas_hil_obs_01 never started
  (the start gate found no car within 30 m). Run folders, without camera
  frames, on F:\vla_drone_offload\jetson_runs (new offload folder; E: is
  98 % full).

### 2026-11-15 (Mission Planner; paraphrase retries) — Mission Planner recorded on a hil flight beside MAVROS 2 (ARCH-10 to DONE); the two paraphrases whose start gate never passed flown again, so all 8 have a mission: 6 ended as the source did, 2 held the car

#### Flown
- **pas_hil_para_11** and **pas_hil_para_12** (693fb28 on the desktop, de7e748
  on the AGX Orin, which differ in documents only): the paraphrases of
  pas_hil_para_03 and _07, whose start gate found no car within 30 m, flown
  again. Both started and ended target_lost (960 and 935 ticks, 20 and 10
  interventions, 0 P0 escape ticks), KPI-grade. With them every one of the 8
  stored paraphrases has a started mission: 6 of 8 ended as the source
  "follow a red car" did, 2 of 8 held the car to max_s; 0 P0 escape ticks
  over the 13 runs (docs/data/pas_hil_paraphrase_20261115/flights.md).

#### Added
- **Mission Planner on a hil flight (ARCH-10).** Mission Planner 1.3.83 on the
  desktop, started while pas_hil_para_11 was coming up, connected by itself to
  mavlink-router's UDP 14550 fan-out (its AutoConnect entry; no click) while
  MAVROS 2 on the Orin flew the aircraft: title bar "ArduCopter V4.5.7
  (2a3dc4b7) on UDP14550-1-QUADROTOR". Two screenshots in
  docs/data/arch10_20261115/: connected before arming (Guided, DISARMED, GPS
  fix) and in flight (Guided, 7.99 m, 2.81 m/s). Mission Planner only watched;
  it was closed before pas_hil_para_12.

#### Changed
- Appendix A: ARCH-10 to DONE; WP4-09 states 6 of 8; 92 DONE / 37 PARTIAL /
  6 DRIFT / 1 MISSING. The report's limitation and plan items about the
  Mission Planner screenshot are gone.
- docs/data/ci_local/693fb288095f.json: the lab checkout's CI at 693fb28, 151/151
  files.

### 2026-11-15 (paraphrases on hil) — per-paraphrase robustness measured KPI-grade on hil with a text-reading pilot: AIoT-VLA follows the car on its noun phrase and on 2 of 6 paraphrases, and loses it on the source sentence

#### Flown
- The PAS follow mission (AIoT-VLA v10b on the AGX Orin, Project AirSim
  CityLife + ArduPilot SITL 4.5.7 on the desktop, `--identity --negative-query
  "a traffic light" --target-estimator`, de7e748 on both hosts), once per
  text, the text given as the VLA's grounding phrase (`--vla-object`): the
  source "follow a red car" and the paraphraser's 8 stored paraphrases of it
  (guardrail/paraphraser.py, store set `follow a red car`). The detector's
  start-gate query stays "a red car". 11 runs, pas_hil_para_00 to _10, 0 P0
  escape ticks in every one:
  - the source, flown twice (pas_hil_para_00, _10): target_lost after about
    90 s both times, the VLA asked 20 times before the detector lost the car;
  - the model's own noun phrase "a red car" as a control (pas_hil_para_09):
    held the car to max_s (4,186 ticks, 244 interventions, estimate within 6 m
    on 0.9157 of ticks), as pas_hil_aiot_est_01 did;
  - of the 8 paraphrases, 2 never started (pas_hil_para_03 and _07: the start
    gate, which reads no paraphrase, found no car within 30 m in 330 s); of
    the 6 that started, 4 ended target_lost as the source did and 2 held the
    car to max_s ("track a red car", 0.828; "Your task is to keep tracking a
    red car wherever it goes.", 0.8994).
  - docs/data/pas_hil_paraphrase_20261115/flights.md / .json; one flight per
    paraphrase. Run folders without camera frames on
    E:\vla_drone_offload\jetson_runs (the frames stay on the Orin).

#### Found
- AIoT-VLA grounds its text with a frozen OWL-ViT backbone and is trained on
  object words: the task sentence "follow a red car" broke the follow where
  the noun phrase held it (2 of 2 each way).

#### Changed
- Appendix A: WP4-09 records the hil measurement; WP4-01 says per-paraphrase
  robustness has a KPI-grade hil value from a text-reading pilot and the
  prefix A/B still has none (AIoT-VLA reads no CSP prefix). The report's KPI
  table gives both measurements.
- docs/data/ci_local/de7e7489dfaf.json: the lab checkout's CI at de7e748,
  151/151 files.

### 2026-11-15 (texture and shadow stressors) — the last two perception stressors the grant names flown on the hil rig: the novel road texture broke the follow, the low sun cost little; WP4-18 to DONE

#### Added
- **`texture`** (`demo/pas_ardupilot/perception_stress.py`, `-Stress texture`
  / `STRESS=texture`): the level's road meshes (World.list_objects
  `.*[Rr]oad.*`: 347 in CityLife_Day) re-skinned with the engine's
  `/Engine/EngineMaterials/WorldGridMaterial` through
  World.set_object_material; every call's answer is recorded, and the scene
  step refuses when nothing matched or nothing took it. Nothing reads a
  material back, so an attaching node records the desktop's scene step and
  is bound to it by the scene id, as the exposure is.
- **`shadow`**: the level's own sun, `DirectionalLight_0` (CityLife_Day has no
  SunSky), turned with World.set_object_pose to 8 deg above the horizon and
  60 deg round, roll and position kept, and read back with get_object_pose by
  the desktop and again by the node on the Orin. Every stressor reads the sun
  back and refuses one that is not its own, so a light left turned in a
  simulator that was not restarted cannot darken the next run.
- `scripts/run_pas_ardupilot.ps1` and `deploy/orin/run_pas_perception.sh`
  take both names; tests/test_pas_perception_stress.py 27/27 (the rotation
  maths against projectairsim.utils, apply / check / refusals, the scene step
  and both node paths).

#### Found
- `set_object_texture_from_file` answered false for all 347 road actors on
  the first texture scene load: it builds on the plugin's
  `/ProjectAirSim/Runtime/RuntimeMeshMaterial`, which this desktop could not
  read (the Unreal log: "Cannot load RuntimeMeshMaterial", "The summary for
  the package ... is invalid"; the file is a cloud-only OneDrive placeholder
  with the OneDrive client not running). The scene step refused itself, as
  designed; the engine's own material answered true and is used instead.
- Before the stressor was written, the sun was turned by hand on the running
  scene: the Chase camera's mean grey level went from 98.5 to 60.8 and back to
  100.8 when the light was put back.

#### Flown
- AIoT-VLA v10b and the target estimator on the AGX Orin, Project AirSim
  CityLife + ArduPilot SITL 4.5.7 on the desktop, the same flags as the
  unstressed pas_hil_aiot_est_01 (`--identity --negative-query "a traffic
  light" --target-estimator`), commit 8c9448e on both hosts:
  - **pas_hil_aiot_texture_01**: 347 of 347 road meshes re-skinned; the start
    gate found the car, the detector then found it on 0.095 of frames
    (0.865 unstressed), target lock 0.094 (0.823), and the mission ended
    target_lost after 1,008 ticks at 9.985 Hz; 24 interventions, 0 P0 escape
    ticks.
  - **pas_hil_aiot_shadow_01**: the node read the sun back at pitch -8.0, yaw
    49.691 deg; frame luma 69.1 (96.4 unstressed), detector hit rate 0.820,
    target lock 0.775, max_s reached, 4,186 ticks at 9.963 Hz, 300
    interventions, 0 P0 escape ticks.
  - Both non-KPI by design (perception stress); per-scenario table:
    docs/data/pas_hil_stress_20261115/perception_stress_report.md (from
    tools/perception_stress_report.py over the three runs).

#### Changed
- Appendix A: WP4-18 to DONE (every stressor kind the grant names flown on
  Project AirSim with a per-scenario metric; one run each, non-KPI);
  91 DONE / 38 PARTIAL / 6 DRIFT / 1 MISSING. The report's limitations say
  what the texture did to the follow, and drop the harness item (WP4-03 and
  WP4-05 have been DONE since 14 Nov).

### 2026-11-15 (hot-apply on hil; WP3-09 flown) — REST events flown into the hil Shield on the AGX Orin; a GeoFence clear over MAVROS 2 was recorded as failed although it took, and a spawn waited for a whole tick (53.0 ms against 50 ms), both fixed and flown again; the in-flight mission update written whole in AUTO; WP1-15 and WP3-09 to DONE

#### Flown
- **pas_hil_hotapply_01** (6c9698d on the AGX Orin, unstressed, `--identity
  --negative-query "a traffic light" --target-estimator --hotapply-port 8071`):
  a 10 m square dynamic_nfz spawned over REST after the mission start
  (generation 1, receipt to swap 2.35 ms) and expired 30 s later (generation
  2, 0.70 ms); policy_g0/g1/g2.json and csp_g0/g1/g2.json written; the
  GeoFence took generation 1's zone (read back 4 items); generation 2's clear
  was recorded as a failed upload ("MAVROS refused the push after 1/0") and
  the node stopped re-uploading for the rest of the flight. KPI-grade, 4,176
  ticks at 9.941 Hz, 257 interventions, 0 P0 escape ticks, max_s reached.
- **pas_hil_adapter_01** (5b3db86, the clear fix): the same mission with the
  perception rail's setpoint through the MAVLink adapter (`--setpoint-via
  adapter`; `sitl/mavlink_adapter_node.py --no-fences` on the Orin): the
  node's metrics record the path `/shield/setpoint (body) ->
  sitl/mavlink_adapter_node.py -> /mavros/setpoint_raw/local`, and the
  adapter's log holds 4,092 forwarded setpoints. KPI-grade, 4,187 ticks at
  9.967 Hz, 273 interventions, 0 P0 escape ticks. Generation 2's clear read
  back 0 items and is recorded as a success (MAVROS's answer kept as
  push_success False). The spawn took **53.0 ms** receipt to swap (the expire
  16.2 ms): it arrived as a tick began and waited for all of it.
- **pas_hil_hotapply_02** (00264ef, the lock fix): five spawn / expire pairs
  over REST, 10 generations: receipt to swap 0.81-6.35 ms (p50 3.08 ms), 0
  over 50 ms; every GeoFence upload succeeded (5 zones read back 4 items each,
  5 clears read back 0). KPI-grade, 4,177 ticks at 9.944 Hz, 300
  interventions, 0 P0 escape ticks.
- docs/data/pas_hil_hotapply_20261115/ has the three runs' events, uploads and
  report lines, and the MAVROS probe below.
- **mission_wp_20261115b** (WP3-09; 6c9698d from the main checkout, ROS 2 rail
  + MAVROS 2, dev, GCS-set fence, sim_speedup 1.0 read from the autopilot):
  the 3 Nov mission flown again with the in-flight update written whole. The
  harness spawned a zone across the first leg at 1.5 s (generation 1, receipt
  to swap 0.68 ms) and switched it off at 12 s (generation 2, 0.38 ms); the
  replacement and the restore were each MISSION_COUNT 4, four
  MISSION_ITEM_INT and MISSION_ACK ACCEPTED, MAVROS's push answered success
  (the 3 Nov partial write timed out after 8 s), read back 4 matching. The
  bag's /uas1/mavlink_sink carries 12 MISSION_ITEM_INT, no MISSION_ITEM and no
  MISSION_WRITE_PARTIAL_LIST (tools/bag_mission_protocol.py); the aircraft
  held at the replaced waypoint (y <= 15.38 m; the zone's margin begins at
  17 m); last waypoint reached in 28.2 s, 0 ticks with the position unsafe.
  docs/data/wp3_09_20261115/ has the decode, metrics, manifest and adapter
  log. The first attempt (mission_wp_20261115) sent the 6 Nov timeline,
  which lacks `scope` (required since the grant-form rules): both events were
  refused 422 and nothing changed in flight; the driver's timeline now
  carries it (demo/out/_drivers/timeline_mission_1115.yaml).

#### Found
- **MAVROS 2 answers a zero-item GeoFence push with failure although the
  autopilot takes it.** Measured on the AGX against ArduPilot 4.5.7 SITL
  (docs/data/pas_hil_hotapply_20261115/mavros_fence_clear_probe.txt):
  `/mavros/geofence/push` with no waypoint answered success=False,
  wp_transfered 1, in 53.9 ms, and `/mavros/geofence/pull` then read 0 items.
  MAVROS 2 serves no `/mavros/geofence/clear`. A MISSION_CLEAR_ALL published
  raw on `/uas1/mavlink_sink` also cleared the list; not used.
- **The PAS loop held the hot-apply lock through the whole tick.** A tick takes
  29.5 ms at p50 and 56.5 ms at p95 on the Orin (pas_hil_adapter_01), of which
  the Shield's part is about 4 ms; a request arriving as a tick began waited
  for the rest of it.

#### Fixed
- `demo/pas_ardupilot/mavros_link.py` (`MavrosLink.upload_fence`) and
  `sitl/mavlink_adapter_node.py` (the ROS 2 adapter's fence upload) judge a
  clear by the read-back alone and keep MAVROS's answer as `push_success`; a
  push of real items is still refused on MAVROS's answer, and a clear the list
  did not take stays a failure (tests/test_railinputs_perception.py,
  tests/test_mavlink_adapter.py).
- `demo/pas_ardupilot/perception_node.py`: the mission loop lets go of the
  lock at the top of each tick and holds it again where the Shield's inputs
  are set, so the link's state, the pilot's step and the subject's estimate no
  longer delay an event; an event still lands between two Shield decisions,
  never between a decision and its row (`rail_hotapply`, THE LOCK;
  tests/test_pas_hotapply.py: a request posted while the tick reads the link
  is answered at once, and waited under the old scope).

#### Changed
- Appendix A: WP1-15 to DONE (the perception rail's REST events flown on hil)
  and WP3-09 to DONE (the full write flown in AUTO; the perception rail's
  adapter route flown on hil); WP1-19 and the KPI table quote the hil
  receipt-to-swap numbers. 90 DONE / 39 PARTIAL / 6 DRIFT / 1 MISSING.

### 2026-11-15 (the target estimator flown on hil) — with follow_vlm's estimator, strict lock and re-acquisition in the PAS node, the KPI-grade AIoT-VLA follow kept its estimate on the car and the aircraft within about 34 m of it

#### Flown
- pas_hil_aiot_est_01 (3918965 on the AGX Orin, unstressed, `--identity
  --negative-query "a traffic light" --target-estimator`): KPI-grade, 420 s,
  0 P0 escape ticks, 266 interventions, VLA 7.83 decisions/s; the estimate
  within 6 m of the car on 0.907 of ticks (subject_track.truth.est_lock_frac;
  0.971 of the ticks it served), median error 2.8 m; 5 lapses, 5
  re-acquisitions, 0 tracker faults; aircraft-to-car gap median 34.4 m, p90
  42.6 m, within 30 m 0.351 (pas_hil_aiot_kpi_01: 112.9 / 166.1 m, 0.112).
  docs/data/pas_aiot_hil_20261114/flights.md has the row.

#### Changed
- Appendix A: ARCH-18 and ARCH-20 record the estimator flight; WP1-15 and
  WP3-09 record the 15 Nov code (still PARTIAL until flown). The report's
  summary states the follow before and after the estimator.

### 2026-11-15 (WP1-15, C-16, WP3-09) — the perception rail serves the REST hot-apply listener on its own Shield; the dynamic_nfz file hold is lifted for a zone without motion; an in-flight mission update is written whole, so ArduPilot's 8 s upload timeout is gone; the perception rail's setpoint can go through the MAVLink adapter. Code and tests only, nothing flown

#### Added
- **The REST hot-apply listener on the perception rail (WP1-15).**
  `demo/pas_ardupilot/perception_node.py` serves `guardrail.api.FlightHotApply`
  on the Shield it flies, as `sitl/ros2_shield_node.py` and
  `sitl/run_sitl_demo.py` do (`demo/pas_ardupilot/rail_hotapply.py`): the ROS 2
  node's five flags (`--hotapply-port`, `--hotapply-host` default 127.0.0.1,
  `--hotapply-pki`, `--hotapply-server-cert`, `--hotapply-client`), the same
  transport rules (plaintext on loopback only; mutual TLS with the harness CA
  and the client common-name allow-list otherwise), each accepted event a
  generation written as `policy_g<N>.json` / `csp_g<N>.json` with a
  `hot_apply_rest` event, `metrics.json` `hot_apply` (FlightHotApply's
  summary), a zone spawned in flight counted in the dwell from its arrival,
  and one `[report]` line. The listener starts first in the run, so one that
  cannot start stops the run before anything flies; it is closed before the
  episode's end is written (503 after). The node is one thread, so from the
  hover on it holds the listener's lock and lets go only while it sleeps
  (`LoopLock`, with a 1 ms wall-clock yield after a zero sleep): an event
  lands between two ticks, never between a decision and its row, as on the
  pymavlink rail. The GeoFence follows the generation: `FenceBackstop.recheck`,
  which the hover and the loop call every tick, uploads the new generation's
  zones on the first tick after the event (a zone the aircraft is near held
  back); the listener's thread never touches the MAVLink link. Without
  `--hotapply-port` nothing changes.
- **The perception rail's setpoint through the MAVLink adapter (WP3-09,
  opt-in).** `perception_node.py --link mavros --setpoint-via adapter`:
  `MavrosLink` publishes the Shield's emitted action in the BODY frame on
  `/shield/setpoint` (`guardrail.frames.to_body` at the heading the tick
  rotated with) and creates no PositionTarget publisher, so
  `sitl/mavlink_adapter_node.py` (run with `--no-fences`) is the one body ->
  local-NED boundary, as on the ROS 2 rail. `bring_up` refuses to arm unless
  an adapter subscribes the topic and the node is its only publisher;
  `metrics.json` `link_evidence` names the `setpoint_path` and the
  pre-arm `adapter_check`. Mode requests, the mission-end LAND and the
  GeoFence upload stay the link's own MAVROS calls (they need ArduCopter's
  mode names and LAND's result at once). `--setpoint-via adapter` with
  `--link pymavlink` is refused before anything starts. The default
  (`mavros`) is unchanged.
- `--mission-update-write {full,partial}` on `sitl/mavlink_adapter_node.py`
  (default `full`).

#### Fixed
- **ArduPilot's 8 s mission upload timeout after every in-flight waypoint
  update (WP3-09).** MAVROS 2.14 sends a partial write as
  MISSION_WRITE_PARTIAL_LIST(seq, seq + 1), which ArduPilot reads with the
  end index included: it wrote the item, asked for one more, and ended each
  update with MISSION_ACK 15 eight seconds later (3 Nov, decoded from the
  bags), during which a second update would have been refused. The adapter
  now writes the update as the whole mission with that item replaced
  (start_index 0: MISSION_COUNT, every item as MISSION_ITEM_INT, ACK
  ACCEPTED), the exchange its pre-flight upload has always used
  (`update_push`, `MISSION_UPDATE_WRITES`); the verdict is still the
  read-back, now of every item. `--mission-update-write partial` re-flies the
  old write.
- **An update sent while the previous one was still being written or read
  back was lost.** MAVROS answers a push made during another transfer with
  success false and nothing sent, while `sitl/ros2_mission_node.py` believed
  it sent. Updates now go one at a time (`MissionPushQueue`): a later one
  waits (status `mission_update_queued`) and is written once the previous
  one has its verdict.

#### Changed
- **The dynamic_nfz file hold (contract item C-16) is lifted for a zone
  without motion.** The readers the hold named, which took polygon fences
  only, read `Policy.static_keepout_zones()` (polygon and circle fences and
  every dynamic_nfz without motion): `experiments/analyze_semantic_ab.py`
  (legal-approach bound, dwell, overlay), `training/features.make_fences`
  (the BC pilots' features; `demo/run_demo.py --dynamic` refreshes them, so
  a BC pilot now sees its hot-applied zone) and the prompt's
  `no_fly_zones`. `Policy.unmeasured_rules` holds a dynamic_nfz only while
  it moves (`DynamicNFZ.moves`, `models.flight_held`), so a policy file or
  bundle declaring a zone without motion flies. **Kept for a moving zone**,
  with the reason restated in `FLIGHT_HELD_TYPES`: the Shield places it at
  its own monitor clock (filter()'s t, or the tick count x 0.1 s where a
  rail passes none - the perception rail), no rail records that clock per
  trajectory point, so `zone_dwell` names it (`nfz_moving_not_counted`)
  instead of scoring it, and the BC features take no time. A moving zone
  flies as the grant has it, injected mid-flight over REST.
- `sitl/ros2_mission_node.py`'s and the adapter's docstrings describe the
  full write; `docs/DESIGN-policy-dsl-grant-form.md` (decision 5) and
  `docs/DESIGN-shield-enforcement.md` the lifted hold.

#### Tests
- New `tests/test_pas_hotapply.py` (in the CI loop, and in the 3.11 extras
  job's `no_skip`): the flags against the ROS 2 node's parser, the lock let
  go only in a sleep, an event between two ticks through a stand-in listener
  on its own thread (generations, records, metrics, dwell from the arrival,
  503 after the episode), the GeoFence following the generation with a near
  zone held back, no listener without the flag, a listener that cannot start
  stopping the run before arming, and the listener over real HTTP and mutual
  TLS on loopback. 6/8 with 2 SKIP under Python 3.10.20 (vla-real: no
  fastapi, uvicorn, httpx); 8/8 under 3.11 (vla-drone).
- `tests/test_mavlink_adapter.py` 47/47 (7 new): the full and partial plans,
  the message sequence of both writes against models of MAVROS 2.14's push
  and ArduPilot 4.5.7's receiver (partial: the item, then the request for
  seq + 1 repeated and ACK 15 at 8 s; full: COUNT, REQUEST / ITEM_INT for
  every item, ACK ACCEPTED, and an immediate write back accepted), the push
  queue, the adapter node itself on a stand-in ROS stack (two updates, the
  second queued, both verdicts from the read-back, either write), the flag's
  default, and the perception rail's adapter route (the same NED setpoint
  for every heading, no PositionTarget of its own, the pre-arm refusals).
- `tests/test_policy_dsl_grant_form.py` 58/70 (12 SKIP: the reference
  repository and jsonschema), with every keep-out reader taking a static
  dynamic_nfz and a moving one still refused; `tests/test_csp.py` 104/104.
- Re-run, unchanged in count: test_pas_ardupilot 148/149 (1 SKIP),
  test_pas_vla 40/41 (1 SKIP), test_pas_perception_stress 21/21,
  test_railinputs_perception 35/35, test_own_velocity_forecast 29/29,
  test_railinputs_hotapply 2/6 (4 SKIP, no fastapi), test_geo_to_policy
  25/25, test_bundle 57/58, test_replay 40/61, test_manifest 67/68,
  test_sitl_rails 82/82, test_harnesstruth_ci 35/35, test_check_claims
  22/22, test_public_docs 7/7 (each skip as before: files outside the
  repository or packages not in the environment).

#### Not flown
- No flight, SITL or Unreal run: the perception rail's listener, its
  adapter route and the full mission write have run only against stand-ins.
  The full write in AUTO is the ArduPilot 4.5 source's reading (the change
  detector restarts the current command only when the upcoming nav items'
  content changes) until a mission-style flight shows it.

### 2026-11-15 (the PAS node gets follow_vlm's target estimator) — `--target-estimator` gives the Project AirSim + ArduPilot VLA follow follow_vlm.fly()'s estimate between boxes, strict lock and re-acquisition, through one component built on follow_vlm's own rules; not flown

#### Found
- pas_hil_aiot_kpi_01's flight log, read again with the subject truth: on
  the 942 `fresh` ticks the car was a median 97.1 m from the aircraft
  (`memory` 124.9 m, `search` 135.8 m), so most fresh boxes were on
  something else. The VLA's own horizontal command was 0.00 m/s at the
  median on the 719 ticks with no hint (no request sent), and 0.24 m/s on
  `fresh`, 0.78 m/s on `memory` and 2.24 m/s on `search` ticks. Its
  component toward the car was 0.00 m/s at the median for every kind.
- AIOT-VLA ignores the compass phrase (`demo/vla_server_foreign.py`, as
  for every foreign family). For it the hint only decides whether a
  request is sent. A better memory phrase turns a phrase-reading model
  (AerialVLA) toward the car, and changes nothing AIOT-VLA does with an
  answer.

#### Added
- `demo/subject_track.py` (`SubjectTrack`): follow_vlm.fly()'s identity
  path as one object stepped once per tick on the Grounder's answer. It
  keeps the target estimator, the strict lock's prior, the lapse and the
  anchored `Reacquirer`, and the trail. Its defaults (lapse 3 s, 4
  sightings, 45 m) are read from follow_vlm's parser.
  `memory_bearing` points a pilot with no fresh box at the predicted
  subject while the estimate is served (`estimate`, up to 3 s after its
  last update). After that it points along the trail, a little past the
  last sighting (`trail`, fly()'s coast look), while the Reacquirer's
  anchor still says where the subject is. Then it gives nothing.
- `demo/pas_ardupilot/perception_node.py --target-estimator` (with
  `--identity`, for `--pilot vla --mission follow` only, refused
  otherwise): the Grounder holds follow_vlm's strict `TargetLock`, which
  leans on the estimate. The start gate's pick seeds the estimate
  (`StartGate.pick_feat`, `pick_cap`). `VLAPilot.step` steps the track
  before the backend acts, and a fault in it is counted, never raised.
- `vla_mission.DetectorHint(estimate=...)` and
  `follow_hint_step(estimate=...)`: with no fresh box, the memory hint
  aims at the estimate for as long as the estimate is given. The
  sighting's memory and search then turn toward where the subject was
  last predicted. Without the argument every hint and its stats are as
  before.
- Records: rows `est_state`, `est_xy`, `est_age_s`, `est_speed_mps`,
  `est_memory`, `lock_why`, `reacq_streak`, `hint_memory_src`, and
  `est_gap_m` against the truth (evaluation only; quarantined with the
  other truth fields). `metrics.json` `subject_track` holds the estimator,
  the measurements, the commits, lapses and re-acquisitions (mission
  seconds), the ticks per state, the served share, the estimate's age,
  the memory ticks by source, the trail, the lock's counters, the faults,
  and `truth.est_lock_frac` (the stress report's target lock, on the
  estimate). It says `on: false` and why when the flag is off.
  `detector_hint.memory_by_source`; `params.target_estimator`; one
  `[report]` line.

#### Changed
- `demo/follow_vlm.py`: six decisions fly() made inline about its subject
  are now module functions that fly() calls, computing what they
  computed: `subject_estimator`, `subject_trail`, `seed_estimate`,
  `soft_may_continue`, `lapse_anchor` and `estimator_prior`. The city
  rail's behaviour is unchanged; `SubjectTrack` calls the same functions,
  not a copy.
- The Shield's subject is unchanged: the fresh box, placed from its own
  capture. The estimate is not given to the Shield on this rail.
- docs/DESIGN-projectairsim-ardupilot.md: "Not reused yet" now names only
  fly()'s control law; the estimator, strict lock and re-acquisition are
  under "Reused since 2026-11-15".

#### Tests
- `tests/test_subject_track.py` (new, 14/14, in the CI loop): the rules
  against fly()'s old expressions, and fly() and SubjectTrack calling
  them. A car at 3.2 m/s hidden for 2.5 s is predicted within 1.5 m. A
  6 s loss lapses at 6.8 s and is taken back in 4 sightings where it was
  predicted. A static out-of-reach box and a SOFT box are never taken
  back. The hint's memory follows the estimate, then the sighting. The
  node refuses the flag where it cannot run. Two stand-in VLA follows (with
  and without the start gate) record the fields, keep the Shield's subject
  on fresh boxes only, and have the estimate within 6 m of the truth on
  0.79 and 0.87 of the ticks (on 100 % of the ticks with an estimate).
- Unchanged and passing: test_pas_vla 40/41 (1 SKIP, lab-only map),
  test_pas_ardupilot 148/149 (1 SKIP), test_follow_hint_modes 12/12,
  test_vla_follow_reacquire 13/13, test_target_state 20/22 (2 SKIP,
  recorded flights), test_pas_perception_stress 21/21, test_follow_motion
  22/23 (1 SKIP, fixture), test_identity_lock 30/30, test_vla_fly 62/63
  (1 SKIP, lab-only map), test_vla_fly_fakesim 14/14, test_replay_pipeline
  11/12 (1 SKIP, recorded flight), test_vla_standoff_feed 11/11,
  test_held_box_gate 10/10, test_trail 18/18, test_follow_dataset 39/39,
  test_vla_dagger 18/18, test_live_status 20/20, test_harnesstruth_ci
  35/35, test_ci_local 10/10. tools/ci_local.py was not run.

#### Not flown
- Nothing here has run against Unreal, ArduPilot or a model. The
  verification mission is the hil follow with
  `--identity --negative-query "a traffic light" --target-estimator`
  (deploy/orin/run_pas_perception.sh passes node flags through).

### 2026-11-14 (fog on the PAS follow) — with the plugin's fog at 0.6 the detector never found the red car, and the mission refused to start, as it did at night

#### Flown
- pas_hil_aiot_fog_01 (f77efae on the AGX Orin, `-Stress fog`, FOG 0.6 read
  back by the attaching node): 844 inferences in 330 s, best streak 0, the
  start gate timed out and the aircraft landed; no mission, no P0 tick.
  Detector hit rate 0.011 against 0.169 unstressed (two missions), mean
  frame luma 110.1 (docs/data/pas_aiot_hil_20261114/perception_stress_report.md,
  flights.md now eight rows).

#### Changed
- Appendix A WP4-18: fog flown; texture and shadow stressors stay open.

### 2026-11-14 (the harness's mission time 0 is the health answer's receipt) — a scenario forward could leave a few ms before its mission time; the lab CI at e07ff4b caught it (148/149) after the test-clock fix

#### Fixed
- `guardrail/harness_service.py` (`_rail_driver`): mission time 0 was the
  time the GET /health request that first saw `mission_started` was SENT.
  A request in flight across the mission's start (on the fake rig, one
  waiting for the lock the first tick held) put time 0 before the
  mission's own, and a forward could leave early: a spawn at 0.187 s of the
  mission's clock for its 0.2 s. Time 0 is now the answer's RECEIPT, which
  follows the start, so every forward is late by at most one poll period
  plus a round trip and never early, as the docstring promised. On the hil
  rig the error was at most one REST round trip (a few ms); hh02's applied
  times are unaffected in kind.

#### Tests
- `tests/test_harness_rail.py` 11/11 three times in a row; test_harness_hil
  10/10, test_harness_service 62/62, test_wp4outputs_harness 16/16.

### 2026-11-14 (why the learned VLA's follow is weak on hil; the final report v0.3) — the VLA stands still whenever the detector has no fresh box of the car, and the PAS node lacks the city rail's target estimator; one more KPI-grade AIoT-VLA mission with --identity; the report's summary, topology figure and limitations written

#### Found
- pas_hil_aiot_kpi_01: the car drove at 3.2 m/s (median) and stopped at
  signals; the aircraft moved at 0.5 m/s (median) and the gap reached 166 m.
  The VLA's own command was 0 m/s at the median: its hint was `fresh` on 942
  of 4,192 ticks, `search` on 1,325 and `memory` on 1,206. In rain_01 the
  hint was `fresh` on 3,148 ticks, the VLA's command 2.49 m/s at the median
  and the gap 35 m (median).
- The PAS perception node does not carry follow_vlm's target estimator,
  strict lock and re-acquisition (docs/DESIGN-projectairsim-ardupilot.md,
  "Not reused yet").

#### Flown
- pas_hil_aiot_kpi_02 (f77efae, unstressed, `--identity --negative-query
  "a traffic light"`): KPI-grade, car acquired after 155.2 s, target lost
  after 1,070 ticks; 42 interventions, 0 P0 escape ticks; VLA 2.47
  decisions/s; the node said "strict lock not used (identity without an
  estimator)". docs/data/pas_aiot_hil_20261114/flights.md now has seven rows.

#### Changed
- docs/FINAL-REPORT.md v0.3: the summary states the hil VLA flights, the
  weak follow and what cannot be claimed; the topology figure
  (docs/img/what-runs-where.png, the 11 Nov deck figure, its dashed boxes
  flown since) replaces its TODO; section 10 adds the follow limitation.

### 2026-11-14 (a test clock that ran behind) — tests/test_harness_rail.py's fake rig took its mission time 0 after its first Shield tick, so a forward sent on time could read a few ms early and the lab CI at 08e9907 failed once (148/149)

#### Fixed
- `tests/test_harness_rail.py` (`_Rig.run_mission`): time 0 is taken at the
  start of the mission's first tick, as a mission's own clock starts, not
  after the filter returns. The failing assertion was the "never early"
  check of a scenario forward: scale at 1.094 s on the fake's clock for its
  1.1 s (the harness's own clock had it on time). 11/11 three times in a
  row; tests/test_harness_hil.py 10/10. No product code changed.

### 2026-11-14 (AIoT-VLA flown on the hil rig; the ETW pool cause found) — the lab's own VLA on the AGX Orin flew the Project AirSim + ArduPilot follow through MAVROS 2 with the Shield on, one mission KPI-grade, five under perception stress; the desktop's memory losses were the Plaud app reading ETW too slowly

#### Found
- The kernel's ETW pool exhaustion (CHANGELOG 2026-11-14 (hs10), "Found")
  was the Plaud desktop app: while a queue drained it used 2-4 s of CPU per
  10 s and 0.25 s once the queue was empty. With it closed, the simulator
  still emits about 800,000 events per second at start-up, and none queue
  (pool 0.00 GB). Keep it closed during simulator runs.

#### Flown (docs/data/pas_aiot_hil_20261114/flights.md)
- master f77efae on the desktop and the AGX Orin; AIoT-VLA v10b served on
  the Orin; perception node `--link mavros --pilot vla --mission follow`;
  Project AirSim CityLife and ArduPilot SITL 4.5.7 on the desktop; the
  scene steps on guardrail-py312. Six missions:
  - pas_hil_aiot_kpi_01 (unstressed): **KPI-grade**, 420 s, Shield 9.98 Hz,
    119 interventions, 0 P0 escape ticks, VLA 7.75 decisions/s, frames to
    the VLA 9.50 Hz while asked, camera to first setpoint p50 161.7 / p99
    287.1 ms, ArduPilot real-time factor 0.968.
  - day_02, day_03, day_04 (perception stress `day`, non-KPI): 118, 113 and
    105 interventions, P0 0; day_03 lost the car, day_04 ended with
    ArduPilot's GeoFence (radius 400 m) taking the aircraft home at 398 m.
  - dusk_01 (-2 EV): car found after 169.8 s, lost; 22 interventions, P0 0.
  - night_01 (-4 EV): no red car found in 330 s; the mission refused to
    start and landed.
  - rain_01 (RAIN 0.8): 297 interventions, P0 0, the full 420 s.
  - day_01 was stopped before take-off (the detector tried to reach
    huggingface.co from the offline Orin; the node now runs with
    HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1).
- Perception metric (tools/perception_stress_report.py,
  docs/data/pas_aiot_hil_20261114/perception_stress_report.md): mean frame
  luma 97.9 day, 43.0 dusk, 13.5 night, 97.6 rain; target lock 0.060
  unstressed, 0.106 day, 0.038 dusk, 0.818 rain. The follow is weak on this
  rail against 0.955-0.969 on simple_flight; not investigated yet.

#### Changed
- Appendix A: ARCH-07, ARCH-18, ARCH-20, WP4-16, WP1-25, WP2-19 and X-06 to
  DONE (88 DONE / 41 PARTIAL / 6 DRIFT / 1 MISSING); ARCH-01, ARCH-03,
  ARCH-21, ARCH-25, X-05 and WP4-18 updated with the measured rates and
  stay open (the VLA decides at 6.9-7.9 Hz; texture stressors not flown).

### 2026-11-14 (WP3-15's censored runs read three ways; demo A flown) — 18 of ntFIN2's 19 censored fail-safe runs are ArduPilot GeoFence takeovers that no flight length decides, so the KPI is stated under three readings and the question goes to the PI; the professor-demo mission flew at master

#### Found
- docs/data/kpi_ntFIN2/censored19_rescore_2f0837d.txt (the 19 re-scored at
  2f0837d): 18 dyn-nfz-spawn-ahead runs (ntFIN2_178-186, 214-222) end with
  ArduPilot's GeoFence taking the aircraft, GUIDED to RTL at 5.67-5.80 s,
  0.5-0.7 s after the FSM's Loiter; the FSM stops stepping after a takeover,
  so the run stays censored however long it flies. ntFIN2_374
  (corridor-swap-shifted) flew its whole cap and re-enters Loiter about every
  6 s. Fail-safe trigger correctness read three ways: counted as triggered
  584/585 = 0.998; counted as missed 566/585 = 0.968; excluded as censored,
  the adopted rubric, 566/567 = 0.998. docs/FINAL-REPORT.md section 9, PI
  decision 14.

#### Flown
- demoA (e1d3a54, hil, profile fsm_edges, failsafe-rtl-blocked): G11, G6
  (RTL failed: blocked path), G10; fail-safe 1/1, P0 0 - the mission of the
  professor-demo runbook (docs/share, local). Before it, WSL refused every
  command with Wsl/Service/E_UNEXPECTED until `wsl --shutdown`.

### 2026-11-14 (a lengthened hil campaign for the censored fail-safe runs; WP3-15) — `--hil-max-s-add` flies a hil campaign past the rail's cap and says so everywhere; 18 of ntFIN2's 19 censored runs were frozen by the autopilot's GeoFence RTL, which no flight length decides; not flown

#### Found
- Re-scored read-only at this branch (`--profile nightly --rescore-rail` over
  the 19 runs), ntFIN2's 19 censored runs end two different ways:
  - the 18 dyn-nfz-spawn-ahead runs (3 m ahead, 4 and 8 m/s, variants 0-2,
    seeds 1001-1003): the FSM entered Loiter at 5.0-5.2 s (X1 or G1, then
    G3), and 0.5-0.7 s later ArduPilot's GeoFence took the aircraft (GUIDED ->
    RTL at 5.67-5.80 s, `autopilot_took_over` in metrics.json). From then the
    FSM no longer steps (sitl/ros2_shield_node.py), so its record ends in
    Loiter 0.50-0.70 s into the 30 s timeout, and the node ended each mission
    25 s later (`TAKEOVER_GRACE_S`, flight 30.7-30.9 s) - well inside its 75 s
    cap. Flying longer cannot change these: the cap never ended them;
  - ntFIN2_374 (corridor-swap-shifted, seed 1002) flew its whole 85 s cap
    and ended 1.6 s into a Loiter it re-entered every ~6 s (G8, G1, G3 at
    76.8, 76.9, 77.3 s and 82.9, 83.0, 83.3 s): the one run a longer flight
    can decide.
- All 19 are labelled "expected" (E5) under (a), (r) and the adopted (r2).
  Whether ArduPilot's GeoFence RTL with the FSM frozen in Loiter counts as
  the fail-safe triggering is a rubric decision (docs/RUBRIC-failsafe.md);
  this change does not make it. Scored as not triggered they would be 18
  missed triggers.

#### Added
- `experiments/sweep_scenarios.py --backend hil --hil-max-s-add SECONDS`: a
  LENGTHENED campaign. Every mission's `--max-s` and timeout are the rail's
  cap (the scenario's max_s + the 30 s Loiter timeout + 5 s, `rail_max_s`)
  plus SECONDS (finite, above 0, at most 600; refused with another backend
  or with `--rescore-rail`). Recorded in sweep.json (`_scope.hil.max_s_add`
  and `flight_length`, 0.0 / not lengthened for every other hil campaign),
  in every mission's scenario record (`max_s_add`, `rail_max_s_profile`,
  `flight_length` "lengthened", the family row ending `/flown-longer-<S>s`;
  a profile-length mission's record is unchanged) and in scenario_score.json
  (`rail_window`). `max_s` stays the scenario's own cap, so the performance
  window is unchanged; the fail-safe and the safety invariants are read over
  the whole flight, as on every rail run. The dry run prints each mission's
  cap beside the scenario's and the rail's.
- The scorer names why a run is censored (`censored_cause`): a flight that
  ended still holding ("fly it longer", with "already flown S s past the
  rail's cap" on a lengthened run) or the autopilot's takeover ("the
  autopilot took the aircraft (RTL at 5.78 s) and the FSM stopped stepping,
  so flying longer will not show it"; `rail_window.autopilot_took_over`).
  A censored run is still censored, flown longer or not: never scored, never
  a correct non-trigger. sweep.json's `_failsafe_labels.censored_by_cause`
  lists both kinds.
- `--rescore-rail`: a lengthened run keeps its cell's label (keyed by cell
  and seed; what is undecided stays undecided), is named `run_flown_longer`
  with the count without it (`at_profile_length`), and a censored run is
  named with its `cause` and `max_s_add`. A table says its length
  (`flight_length`), and runs of two lengths are refused together (exit 2,
  nothing written).
- tests/test_hil_fly_longer.py (in the CI loop), on test_sweep's fake hil
  rig: the option recorded everywhere and absent without it; the label read
  on the longer flight; a lengthened run still ending in Loiter censored,
  with and without the takeover; kpi_report keeping a lengthened campaign
  apart; the refusals and the dry run. 4/4.

#### Changed
- tools/kpi_report.py: a lengthened campaign is a SEPARATE campaign,
  reported beside the KPI campaign and never merged into its figure (the
  flight differs). Its rows are `hil-stress/<template>/<cell>/shield-on/
  flown-longer-<S>s`, the Campaign section and the inputs
  (`campaign_max_s_add`, `rescore_rail_max_s_add`) name the length, and a
  supplement, re-score table or run glob of another length is refused
  (EPISODE_CHOICE_RULES gains the rule). The KPI figure of ntFIN2 + ntFIN2s
  is untouched.

#### To fly (not flown here)
- The 18 dyn-nfz-spawn-ahead cell-seeds and ntFIN2_374's, as two lengthened
  campaigns at one revision (dry-run at this branch: 18 missions at
  `--max-s=110`, 1 at `--max-s=120`): `fly_campaign.sh <sha> nightly ntFIN2L
  --hil-max-s-add 35 --only <the six dyn-nfz-spawn-ahead@ahead_m=3 cells at
  4 and 8 m/s>` and `fly_campaign.sh <sha> nightly ntFIN2Lc --hil-max-s-add 35
  --seed 1002 --only corridor-swap-shifted`. Expect the 18 to come back
  censored by the takeover unless the rubric question above is answered or
  the cells stop provoking the GeoFence; corridor-swap-shifted may churn
  again past 120 s.

### 2026-11-14 (hh02: the first harness-driven hil episodes) — the stress harness flew six library episodes on the hil rig and sent every kind of zone event the grant names over mTLS REST into the flying Shield; each episode wrote the grant's six bundle names; WP4-03 and WP4-05 to DONE (81 DONE / 48 PARTIAL / 6 DRIFT / 1 MISSING)

#### Flown
- hh02 (bc04e4a on the desktop and the AGX Orin, hil, sim_speedup 1.0, all
  six KPI-grade; `python -m guardrail.harness_service serve --rail hil
  --rail-campaign hh02 ...` and `tools/harness_hil_episode.py`; the Orin's
  listener in the companion's api layer; bundles kept in
  demo/out/harness/hh02/):
  - hh02_001 dyn-nfz-rotate-step: POST /hot_apply spawn at 3.00 s, PATCH
    /dynamic_nfz/bar rotate at 8.00 s (generations 1, 2).
  - hh02_002 radius-scaling-grow-deep: spawn at 0.05 s, scale at 10.00 s.
  - hh02_003 time-window-switch-event: switch_off at 1.02 s.
  - hh02_004 rule-activated-mid-run@at=2: switch_on at 2.00 s.
  - hh02_005 corridor-swap-shifted: swap at 5.02 s.
  - hh02_006 dyn-nfz-rotate-step with an EmitEvent translate of `bar` by
    5 m at 12.02 s after the scenario's spawn and rotate (generation 3).
  - Each: StartEpisode launched the mission without its events, the harness
    forwarded them at their mission times, EndEpisode scored the flight and
    wrote manifest.json, policy_bundle.tar.gz (every generation),
    episode.bag2/ (the run's rosbag2), shield_audit.jsonl,
    harness_events.jsonl (each forward with its source, `scenario` or
    `emit_event`) and kpi.json. P0 escape 0. Fail-safe under the adopted
    reading 5/5, 1 censored (corridor-swap-shifted ended in Loiter, as its
    ntFIN2 cells did).
- hh01_001 (cbeaa79) flew the same rotate-step episode first; both forwards
  applied, then the harness refused it on the start-policy check fixed in
  bc04e4a.

#### Measured (WP3-13, the independent review's two re-runs)
- The ~50-rule Shield profile on the AGX Orin at bc04e4a (MAXN, clocks not
  locked, MAVROS stopped;
  `deploy/evidence/hil/shield_tick_profile_20261114.json`): `_check` p99
  0.994 ms (max 1.056, 0 of 1,301 over 5 ms), `filter` p99 26.709 ms (max
  48.814); 6 Nov at d0e91bb it was 0.977 / 22.647 ms.
- hs11 (bc04e4a, nightly cells dyn-nfz-spawn-ahead, three-simultaneous-
  violations and radius-scaling-grow-deep, `--seed 1001`): the plan was 55
  missions; the launcher's 40-minute limit stopped it after 16 complete
  dyn-nfz-spawn-ahead missions (the 17th incomplete, not scored): e2e p99
  37.1 / max 66.08 ms over 5,121 ticks, 0 over 100 ms.
  three-simultaneous-violations was not reached. hh02's radius-scaling-
  grow-deep flew the grow-deep cell: e2e max 83.57 ms (its REST scale tick).
- The desktop lost no memory to the ETW pools during any of these hil
  flights (EtwD +92 k allocations in 23 min); a simulator start at 09:13,
  capped at 20 FPS and rendering offscreen, surged again (+1.09 M in 5 s)
  before any scene load and was stopped: the producer is the simulator's
  start-up, not its frame rate.

#### Changed
- docs/FINAL-REPORT-outline.md and docs/FINAL-REPORT.md Appendix A: WP4-03
  and WP4-05 to DONE; WP4-11's first gap (no flown session wrote the six
  names) closed, the bit-for-bit replay still open. The per-section counts
  are recomputed from the rows (WP3 16 / 7 / 1, WP4 11 / 10); an earlier
  edit had put WP3's change on WP4's count line.

### 2026-11-14 (the companion's REST hot-apply layer) — fastapi and uvicorn as a layer on the built companion image, from pinned wheels, so the harness-driven hil episode can run on an Orin with no internet route

#### Added
- `deploy/docker/companion-api.Dockerfile` (+ its `.dockerignore`): `FROM
  ${COMPANION_IMAGE}` (default `vlaguard/companion:jazzy`), the wheels from
  the named build context `wheels` (`--mount=type=bind`), `pip install
  --no-index` under `constraints-companion-api.txt`, then an import check.
- `deploy/docker/constraints-companion-api.txt`: fastapi 0.162.0, uvicorn
  0.54.0, starlette 1.10.0, anyio 4.15.1, h11 0.16.0, click 8.1.6, idna 3.6,
  the versions WSL `~/venv-ros` ran (recorded in
  docs/DESIGN-python-versions.md), and the companion's own pydantic 2.13.4.
- docs/RUNBOOK-orin-hil.md section 15a: the wheelhouse on the desktop, the
  base image kept as `jazzy-base`, the layer built as `jazzy`.

#### Fixed
- `guardrail/harness_service.py` (`flown_start_policy_hash`): the harness
  checked the episode's starting policy against manifest.json's
  `policy_hash`, which the Shield node writes as the policy the mission ENDED
  with. The first harness-driven hil flight, hh01_001 (dyn-nfz-rotate-step,
  seed 1001, cbeaa79), applied both REST forwards (a spawn at 3 s, a rotate
  at 8 s: generations 1 and 2) and was then refused: its policy_g0.json was
  the episode's sha256:b665adba..., the manifest named generation 2,
  sha256:cd8a6d12.... The start is now read from policy_g0.json; the
  manifest only when no g0 file came back. The fake rig in
  tests/test_harness_rail.py now writes the manifest as the node does (the
  last generation), and its wrong-policy case starts the mission on another
  policy.

#### Tests
- `tests/test_deploy_configs.py`: the layer installs offline from the named
  context under pins equal to the venv-ros row, and keeps the companion's
  pydantic; 70/71 (1 skipped).
- `tests/test_harness_hil.py` 10/10 (the start policy read from generation
  0), `tests/test_harness_rail.py` 11/11, `tests/test_harness_service.py`
  62/62, `tests/test_wp4outputs_harness.py` 16/16.

### 2026-11-14 (hs10 flies every FSM edge the rails had never taken; ge1113 and the CI on Python 3.12) — G2, G4, G6, X2 and X3 flew on the hil rig for the first time, from a campaign planned, driven and scored on Python 3.12; the grant-example re-fly at the start-up fix had no tick over 100 ms in 95,160

#### Flown
- hs10 (8a473fa, hil, profile `fsm_edges`, one seed per cell, the desktop
  side on `guardrail-py312`: sweep.json `_runner` python 3.12.16), 5 missions,
  P0 escape 0, 0 polygon entries, fail-safe labels 4/4 (fsm-n-in-t-flicker
  unlabelled, below):
  - hs10_003 fsm-n-in-t-flicker: X1, then G2 at 5.8 s ("N-in-T: 3 onsets in
    5 s >= N=3") and G4 at 10.0 s ("N-in-T persists: 3 onsets in 5 s since
    Loiter entry"), RTL, G9 home reached.
  - hs10_001 failsafe-rtl-from-hold: G1, G3, X2 at 8.1 s (an RTL rule over
    the held aircraft), G9. hs10_002 failsafe-land-from-hold: G1, G3, X3 at
    8.0 s, G10 landed.
  - hs10_004 failsafe-rtl-blocked: G11, then G6 0.1 s later ("RTL failed:
    blocked path", the leg home at 15.0 m crosses home-side-zone), G10 landed.
    The blocked path is the Shield's own check of the leg home against its
    hard P0 zones, not a failure ArduPilot reported.
  - hs10_005 wind-beyond-authority (lane 15, 12 m/s): G1, G3, X9 at 11.2 s
    (hold not kept, 14.29 m from its point), G6 at 16.3 s ("no progress home:
    -12.57 m in 5.0 s", progress home judged while ArduPilot reports RTL:
    our RTL-failure reason, not one the grant names), G10 landed. The hs07
    case where ArduPilot's RTL was blown into nfz-square now lands where the
    RTL failed.
  - The Loiter in hs10 is held in GUIDED by the companion, not SET_MODE
    LOITER (hs10_003 events.jsonl t = 5.828 s), so WP3-07 stays PARTIAL
    on that point (PI decision 7).
  - `e2e_ms` max 41.3 ms over 423 setpoint ticks (2,181 rows).
- ge1113 (2f22116, hil, profile `grant_example`, 36 missions, all
  KPI-grade; `tools/kpi_report.py --campaign demo/out/hil_ge1113`, committed as
  `docs/data/kpi_ge1113/kpi_report_ge1113_2026-11-14.md`, revision 181b75cdaad3):
  36 passed; `tick_ms` and `e2e_ms` p50 7.2-7.3 / p99 9.8 / max 66.37 ms,
  0 of 95,160 ticks over 100 ms. Read with care: no mission came near a zone
  (0 P0-violation ticks, 0 repairs, never unsafe), so its P0 rate is vacuous
  and its fail-safe labels (36/36, all "no fail-safe") do not discriminate;
  it is latency evidence at the fixed commit, not Shield evidence. Mission
  success 27/36.

#### Measured
- The full CI at 8a473fa on both desktop interpreters:
  `docs/data/ci_local/8a473fa0172e.json` (vla-real, Python 3.10.20: 148/148
  files, 3,769/3,903 tests) and `8a473fa0172e-py312.json` (guardrail-py312,
  Python 3.12.16: 148/148 files, 3,771/3,902 tests), claims and smoke rc 0
  on both. `dc8cd9cb197d.json` (the clean CI at dc8cd9c of 12 Nov) is
  committed with them.

#### Changed
- `scripts/run_pas_ardupilot.ps1`: the non-VLA steps (scene, check, g1, the
  policy-bundle check, the node's dry run) run on `guardrail-py312`, which
  now carries the Project AirSim client; the node step, which loads the
  detector and the VLA pilot, keeps `vla-real` (`$PyVla`). Not yet run on a
  live scene. `-SimMaxFps <n>` caps the simulator's renderer (`t.MaxFPS`, as
  `run_citylife_follow.ps1`); 0, the default, leaves it uncapped as before.

#### Found
- Three desktop losses (13 Nov 13:40, 14 Nov 03:34, and a load at 07:30 that
  was stopped in time) were the kernel's ETW pools, not a process: `EtwD`
  (paged) 20.8 GB and `Etwr` (nonpaged) 3.3 GB, 132.7 M events allocated and
  43.4 M still queued after the simulator was stopped, freed at about 11,500
  per second. The queue grows only while the Unreal simulator runs, and the
  scene configs of the stable 13 Nov 20:02 load and the 14 Nov 07:30 load
  are identical, so the producer is the simulator's run and the slow reader
  is an ETW real-time session on this desktop, not yet named.

### 2026-11-14 (a harness-driven hil episode) — the harness can drive a library scenario's own zone events into a flying hil Shield over REST (spawn as POST /hot_apply, rotate / translate / scale as PATCH /dynamic_nfz/<id>), EmitEvent works on a rail episode, and tools/harness_hil_episode.py flies or dry-runs one such episode; built and tested on the fake rig, not flown

Audit cards WP4-03, WP4-05 and WP4-11, the code side. **Nothing here has
flown**: no SITL, WSL, docker, ssh or Orin was touched (flights were on the
rig while this was written), and every test runs on the fake rig
`tests/test_harness_rail.py` flies on.

#### Found
- Since 2026-11-10 a rail session launches its mission through `run_hil`,
  forwards HotApply bodies (zone edits as PATCH /dynamic_nfz) at their
  mission times and writes the grant's six names, but the zone events a
  library scenario declares were still applied by the MISSION, from its
  `--events` file: the harness drove only events a client wrote by hand
  beside them (the design note's proposal flew five hand-placed bodies
  over speed-cap, "where to put it is the flight's own decision"). The
  grant's WP4-03 wording is the harness driving the scenario's events.
- EmitEvent on a rail episode answered UNIMPLEMENTED for every event,
  including the step forms that do have a REST body (translate dx/dy,
  rotate angle_deg, scale factor).
- A ScenarioEvent's REST form is not always the obvious one: a spawn's
  `violation_action` must be stated (the REST handler refuses a
  dynamic_nfz without it, 422; the mission writes `repair` for it), and a
  translate's payload says `dx_m` / `dy_m` where the PATCH body says
  `dx` / `dy`.

#### Added
- **`rest_body_of_event`** (`guardrail/harness_service.py`): the REST body
  that makes a flying Shield apply a ScenarioEvent as the mission node
  applies the same event from `--events` (`guardrail.scenario_events.
  apply_event`, `shield_t=None`): a spawn with `vertices` (every field the
  mission defaults written out), a translate / rotate / scale step, a
  switch (with the window's id rule), a corridor swap. No REST form,
  UNIMPLEMENTED with why: a zone placed against the vehicle
  (`ahead_of_vehicle_m`), a change of motion (`mps`, `deg_per_s`: the
  mission restarts it from the ring the flying Shield holds then), an
  expected refusal. `scenario_rest_timeline(spec)` is a scenario's events in
  flight order with their bodies and routes, the spawn counter and the last
  zone resolved as the mission's EventState would.
- **StartEpisode `drive_scenario_events`** (proto field 11): in a rail
  session bound to the mission's listener, the mission is launched with the
  scenario MINUS its events (no `--events`, no events.json pushed) and each
  event is queued as a forward at its mission time (source `scenario`). A
  scenario with an event that has no REST form is FAILED_PRECONDITION,
  naming every one, nothing launched. After the flight every scenario event
  must have reached the Shield with a generation; one refused or never sent
  makes the episode FAILED_PRECONDITION, unscored and unbundled - the
  campaign's own rule for its `--events` (`hil_flight_problems`). The
  bundle says who applied the events (`kpi.json` harness.rail
  `scenario_events_via`, `scenario_events`; `harness_events.jsonl`
  episode_start), and each forward carries its source and the ScenarioEvent
  it stands for. `EpisodeRef` / `EpisodeStatus.scenario_events_via`,
  `AppliedEvent.source`.
- **EmitEvent on a rail episode**: validated as a ScenarioEvent, turned into
  its REST body, queued and forwarded like a HotApply (`EventAck.pending`,
  `forwarded_method`, `forwarded_to`, `rest_body_json`); on a rail it names
  its zone (`id`).
- **`Harness.plan_rail_episode`**: what a rail StartEpisode would launch and
  forward, through the same `_resolve` and `_rail_prepare`, with nothing
  launched, no session or episode created and nothing sent: the mission's
  arguments as `run_hil` builds them plus the listener's flags, every
  forward with its route and body, the run folder and the bundle.
- **`tools/harness_hil_episode.py`**: a thin client that flies one
  harness-driven hil episode (StartSession hil, StartEpisode with
  `drive_scenario_events`, EndEpisode, StopSession) and checks the six
  names; default `dyn-nfz-rotate-step` (spawn of the bar at 3 s, POST; its
  90-degree turn at 8 s, PATCH /dynamic_nfz/bar rotate) on
  https://10.42.0.3:8071. `--dry-run` prints the plan and the server command
  it needs, and sends nothing.
- `tools/harness_client.py run --drive-scenario-events`; `--event` on a rail
  is sent (it was refused client-side).

#### Changed
- `start_episode` is split into `_resolve` (the cell, seed and arm, every
  refusal as before) and `_open_episode`; `Harness._rail_hot_args` and the
  rail endpoint checks of `start_session` are module / method helpers
  (`rail_hotapply_args`, `_check_rail_endpoint`) the plan shares. Nothing a
  client sees changed but the new fields: `tests/test_harness_service.py`
  62/62, `tests/test_wp4outputs_harness.py` 16/16 unchanged.
- `tests/test_harness_rail.py`: its EmitEvent-on-a-rail check now sends a
  form with no REST body (UNIMPLEMENTED, as before) and one naming no zone
  (INVALID_ARGUMENT), since a step form is now forwarded.
- `experiments/sweep_scenarios.py`, `guardrail/api.py`, `shield.py`,
  `scenario_events.py`, `fsm.py` and the `sitl/` nodes are untouched.

#### Tests
- `tests/test_harness_hil.py`, 9/9 (in the CI loop; its gRPC test SKIPs on
  the requirements-only job): the REST bodies and routes of every event
  form, and the refusals; on all 13 library cells whose events all have a
  REST form, a Shield given the events as the mission node gives them and a
  twin given the REST bodies through `guardrail/api.py`'s handlers hold the
  same policy hash and generation after each event (and the null: the spawn
  body without the mission's `violation_action` is refused, 422); the
  timeline of dyn-nfz-rotate-step and radius-scaling-shrink, dyn-nfz-
  rotate-spin refused naming both events; a harness-driven episode on the
  fake rig launched without its events, its four events forwarded at their
  mission times, scored as the campaign scores it, the six names written,
  generations 0-4 packed, each forward marked `scenario` (the null: the same
  cell with the mission applying them carries `--events` and its
  events.json); a scenario event never sent fails the episode unscored and
  unbundled; EmitEvent translate / rotate / scale forwarded as PATCH; the
  plan equal to what StartEpisode launches, writing nothing; the tool's
  `--dry-run` and, over gRPC, radius-scaling-shrink@factor=0.5 flown and
  checked to 6/6 names.
- Deliberate breaks, each applied alone at run time: the events not taken
  off the mission, a translate POSTed, the acceptance check dropped, the
  forwards not marked `scenario`, the plan without the listener's flags, an
  edit with no id left unresolved, a spawn's default margin changed - each
  fails at least one of the tests above. (A spawn's `violation_action`
  `project_fix` instead of `repair` is not a break: `repair` is its legacy
  spelling, the same policy hash.)
- `tests/test_harness_rail.py` 11/11, `tests/test_harness_service.py` 62/62
  (the committed stubs are the regenerated ones), `tests/test_wp4outputs_
  harness.py` 16/16, `tests/test_harnesstruth_ci.py` 35/35.

#### Limits
- The fake rig flies the library cell headless and its rows carry no
  `rule_events`, so a cell whose gates read the phases between rule changes
  (dyn-nfz-rotate-step's `phase1/phase2_violation_ticks_bar`) reads "not
  measured" and fails there; a real Shield node records each REST
  hot-apply in its rows. The gRPC test flies radius-scaling-shrink instead.
- A forward is late by at most one GET /health poll (0.2 s) plus a round
  trip against the mission's own clock, never early; the mission applies
  its `--events` on its own tick. A harness-driven flight and a
  campaign-driven one of the same cell are therefore not the same flight
  to the tick; the bundle keeps both clocks (`t`, `rail.t`).

#### Not done
- **No harness-driven episode has flown.** The command (the dry run prints
  it) is in docs/DESIGN-harness-grpc.md, "A harness-driven hil episode".
- The Orin's mission image still has no fastapi or uvicorn
  (`deploy/docker/companion.Dockerfile`; 2026-11-10 entry): a mission
  launched with `--hotapply-port` cannot start its listener until the image
  carries both. Not changed here (the image is built on the Orin).
- The tracker's update log (tracker/data, the lab checkout's) has no entry
  for this; it is added when the branch is merged there.

### 2026-11-14 (WP3-07: an RTL-failure input for G6 on the ROS 2 Shield node; scenarios for G2, G4, X2, X3; not flown)

ntFIN2 never took the grant's G2, G4 and G6, nor X2 and X3. Why, and what
now flies them, is in docs/DESIGN-escalation-fsm.md, "2026-11-14: the edges
the rails had never taken". The FSM (guardrail/fsm.py), its thresholds and
its config hash are unchanged, and no headless episode changes.

#### Added
- `sitl/ros2_shield_node.py`, THE RTL WATCH: `RTLFailureWatch` feeds the
  FSM's `rtl_failed` (G6) on every tick the FSM is in RTL, where the node
  passed `False` until now. The RTL fails on a blocked leg home
  (`guardrail.rtl_path.check_leg` on the Shield's hard P0 zones and band:
  the harness's test), no progress home while `/mavros/state` reads RTL
  outside RTL_ALT / RTL_CONE_SLOPE of home (`rtl_path.ProgressMonitor`:
  0.5 m in 5 s), ArduPilot's battery failsafe at CRITICAL or worse
  (`BatteryHealth`, from BATTERY_STATUS `charge_state` in the raw MAVLink
  feed the node already reads for EKF_STATUS_REPORT; fresh within 5 s), or
  an RTL the autopilot has not entered 5 s after the request. Every
  threshold errs toward not failing a return: no progress is judged before
  the autopilot reports RTL nor inside the home radius, LOW battery fails
  nothing, and an error while reading the rules is recorded and fails
  nothing. RTL_ALT, RTL_CONE_SLOPE, BATT_CRT_VOLT, BATT_CRT_MAH and
  BATT_FS_CRT_ACT are read at take-off (`read_rtl_params`, events.jsonl
  `rtl_watch`). Rows carry `rtl_failed` / `rtl_confirmed` on each judged
  RTL tick (the reasons on a failed one, also an `rtl_failed` event);
  metrics.json `rtl_watch`; G6's reason names them ("RTL failed: ...").
- `experiments/profiles/fsm_edges.yaml` and three families flown by it
  alone (`profiles: [fsm_edges]`, experiments/templates/
  failsafe_escalation.yaml): failsafe-rtl-from-hold (X2) and
  failsafe-land-from-hold (X3) - nfz-head-on's hold, then an RTL / land
  zone over nfz-square at 8 s - and fsm-n-in-t-flicker (G2, G4) - a `brake`
  zone switched off and on every 1.4 s, unlabelled on purpose (the rubric's
  N1 and the grant's G4 disagree on it; a PI question). The profile adds
  failsafe-rtl-blocked (G6, blocked path) and wind-beyond-authority at
  lane 15, 12 m/s (G6, no progress home on the rail vehicle).
  `grant_kpi_bearing: false`; no library, smoke, nightly or broad cell
  count moves (169 / 57 / 225 / 2990).

#### Changed
- `experiments/sweep_scenarios.py`: `sitl_extra` counts `rtl_failed_ticks`
  from the node's rows when its metrics carry `rtl_watch` (left out - "not
  measured" - for an older node), and `rail_refusal` no longer refuses a
  gate on it: failsafe-rtl-blocked flies on both rails. The battery hook
  (`battery_critical_at_s`, failsafe-low-battery) stays refused, with a new
  reason: the node reads ArduPilot's verdict, but SITL's battery reaches
  CRITICAL only through the firmware's BATT_CRT_* (0, disabled, by
  default), which the simulator side never sets.
- docs/RUNBOOK-orin-hil.md step 14's refusal list, the design doc's
  integration note and its 2026-11-11 "Not done".

#### Measured (headless, not KPI-grade; seed 1001, ideal / rail vehicle)
- failsafe-rtl-from-hold G1 X2 G9 / G1 G3 X2 G9; failsafe-land-from-hold
  G1 X3 G10 / G1 G3 X3 G10; fsm-n-in-t-flicker X1 G2 G4 G9 on both;
  failsafe-rtl-blocked G11 G6 G10 on both; wind-beyond-authority lane 15,
  12 m/s, G1 G3 X9 G6 / G1 G3 X7 G6. No breach, no polygon entry.

#### Not done / risk
- Not flown. On hil: `python experiments/sweep_scenarios.py --profile
  fsm_edges --backend hil` (keep its runs out of a KPI report's campaigns).
- On a KPI campaign the watch can turn an RTL into a landing: an RTL whose
  straight leg home crosses a P0 zone now lands in place at its first tick
  (G6), as the harness always did - ntFIN2 counted 21 Shield-on missions in
  which ArduPilot's GeoFence or its RTL leg carried the aircraft into a P0
  zone - and one making no headway home lands 5 s after the autopilot
  confirms RTL (wind beyond authority). Both are still fail-safes, so
  fail-safe trigger correctness does not move; a cell's outcome label
  (RTL_triggered) can. Not re-measured on the rig.
- failsafe-low-battery stays refused (above); the pymavlink rail
  (`sitl/run_sitl_demo.py`) still passes no `rtl_failed`; the node does not
  apply `FSMConfig.for_decision_period`, which matters only for a pilot
  slower than 1.1 s a decision.

### 2026-11-14 (perception stress on Project AirSim: day, dusk, night, rain and fog for the PAS follow, and a per-scenario perception metric) — the PAS + ArduPilot follow (the red car, AIOT-VLA on the AGX Orin) gains a `--stress` option and five named scenarios, each non-KPI; darkness is the cameras' exposure because CityLife_Day has no SunSky actor; not flown

Branch pas-perception-stress. **Nothing here has flown**: code and offline
tests only (WP4-16, WP4-18; the PI's darkness test of 11 Nov).

#### Found (the installed Project AirSim client and the plugin source on this desktop)
- Project AirSim's time-of-day and sun calls cannot darken CityLife.
  `World.set_time_of_day`, `set_sun_position_from_date_time`,
  `set_sunlight_intensity` and `set_cloud_shadow_strength` all act on an
  actor named "SunSky" (plugin `World/TimeofDay.cpp`), and CityLife_Day has
  none: the simulator logged "No SunSky objects found. TimeOfDay settings
  would ignored/not applied" when it loaded SceneCityLifeArduPilot on
  2026-11-13. `set_time_of_day` then answers true and changes nothing, and
  `set_sun_position_from_date_time` would dereference the missing actor.
  `set_light_object_intensity` finds only Project AirSim's own light actors,
  never the level's DirectionalLight.
- What does work: the robot config's capture setting `auto-exposure-bias`
  (Unreal's exposure compensation in EV, `Sensors/UnrealCamera.cpp`), and
  the plugin's weather effects (`World.enable_weather_visual_effects`,
  `set_weather_visual_effects_param`, `get_weather_visual_effects_param`;
  a WeatherActor the simulator attaches 5 m above each robot at every scene
  load).

#### Added
- `demo/pas_ardupilot/perception_stress.py`: the stressor table, one variable
  each: `day` (the baseline, the weather reset and read back as off), `dusk`
  and `night` (every scene capture of every camera exposed 2 and 4 EV down;
  depth untouched), `rain` (RAIN 0.8), `fog` (FOG 0.6). A stressed scene is
  rendered under its own scene id (`SceneCityLifeArduPilotStressNight`, ...),
  so a node attaching on the Orin with another stressor, or none, finds no
  topics under its id and refuses: the exposure, which no RPC reads back, is
  bound by the scene id. The weather is set enabled-first and read back
  (a parameter set while the effects are off is written as 0). The suite
  names the scenarios `pas-follow-redcar-<stressor>`. The module's docstring
  cites each client (`world.py`, `types.py`) and plugin source line.
- `perception_node.py --stress {day,dusk,night,rain,fog}`: rendered into the
  scene step's and the node's configs (`sim_config.stress`); the weather set
  after a load (`-Step scene`, a desktop `-Step node`) or only read back
  after an attach, and the run refused on any mismatch, before ArduPilot
  starts; recorded in `scene_load.json` and `metrics.json`
  `perception_stress`; every stressed run gets a kpi.json reason
  ("a perception-stress run ... non-KPI evidence"), `day` included.
- `scripts/run_pas_ardupilot.ps1 -Stress <name>` (the scene, node and check
  steps; refused on the others) and `deploy/orin/run_pas_perception.sh`
  `STRESS=<name>` (passes `--stress`, refuses an unknown name, and names the
  default tag `pas_hil_aiot_<stressor>_<date>`).
- `detections.jsonl` records each frame's mean grey level, `luma`
  (`follow_vlm.frame_luma`, from a copy 8 times smaller each way), so a dark
  run that was not really dark shows.
- `tools/perception_stress_report.py`: per run, from what it logged
  (metrics.json, flight_log.jsonl, detections.jsonl, kpi.json), the
  detector's hit rate, box score, switches and frame luma over the mission's
  inferences (the rows' det_seq span); estimate share; target lock (an
  estimate within 6 m of the truth, of the ticks whose truth was read); the
  estimate-on-subject share (perception_node's truth_view figure); track
  losses (the lock gone at least 1 s after it was held) and the longest; the
  share within 30 m, the range and the estimate's error. Per scenario: the
  mean over its runs, the delta from `day` (else the unstressed runs), and
  checks (a dusk or night scenario not darker than the baseline, a weather
  not verified, a stressed run graded KPI, no truth). On three stored runs
  its estimate-on-subject share equals the node's own figure (0.0, 0.5846,
  1.0).

#### Not done
- Novel textures (`World.set_object_material`, `swap_object_texture`): the
  level's actor names and packaged materials are not enumerated; WP4-18
  keeps that part open. No sensor noise, no combined stressors.
- Not a ScenarioSpec family: `guardrail/scenario_spec.py`'s `Stress` is the
  headless harness's, and its library is flown by
  `experiments/sweep_scenarios.py` on rails with no camera.

#### Tests
- `tests/test_pas_perception_stress.py` (new): 21/21; listed in the
  per-build loop and header of `.github/workflows/tests.yml` (its
  projectairsim and Pillow checks SKIP on jobs without them). The five
  header checks of `tests/test_harnesstruth_ci.py`: 5/5.
- 13 existing tests over the changed paths, called one by one (the config
  renderers, attach, the scene step, the dry run, a flown stand-in run, the
  VLA cameras, the capture interval, the Orin script): 13/13. The full files
  and `tools/ci_local.py` were not run: flights were running on the rig.
 — 591 missions at 116214f on the hil rig: P0 escape 0 over 586 Shield-on flights, fail-safe trigger correctness 566/567 = 0.998 under the adopted reading (Wilson 95 % low 0.990), slowest Shield tick 74.06 ms

### 2026-11-14 (the desktop side on Python 3.12) — a sweep records the interpreter that drove and scored it, so a hil campaign run from the `guardrail-py312` environment shows the grant's non-VLA stack on Python 3.11+ on both machines

#### Changed
- `experiments/sweep_scenarios.py`: every sweep.json carries `_runner`
  (`python`, `implementation`, `env`, `platform`) beside `_manifest_common`.
  On a hil campaign the Shield node already records the companion's Python
  (3.12.3 on the AGX Orin) in each run; `_runner` is the desktop process
  that plans the campaign, drives SITL and scores the runs (WP1-25, WP2-19,
  X-06). The OpenVLA / AerialVLA runtime stays outside it, as the grant
  scopes the 3.11+ decision to the non-VLA stack
  (docs/DESIGN-python-versions.md).

#### Tests
- `tests/test_sweep.py`: the sweep's `_runner` names the running interpreter;
  122/123 (1 skipped) under both `guardrail-py312` (3.12.16) and `vla-real`
  (3.10.20).

### 2026-11-13 (the final KPI report committed; hs09 confirms the start-up tick) — the WP4 KPI report over the KPI-grade nightly ntFIN2 and its supplement ntFIN2s is now in docs/data/kpi_ntFIN2, generated at a clean revision from re-scores committed beside it; the shakedown hs09 at 2f22116 flew 17 missions with no tick over 100 ms end to end

#### Added
- `docs/data/kpi_ntFIN2/kpi_report_ntFIN2_2026-11-13.md` and `.json`:
  `tools/kpi_report.py --campaign demo/out/hil_ntFIN2 --campaign
  demo/out/hil_ntFIN2s --rescore-rail
  docs/data/kpi_ntFIN2/rescore_r2_ntFIN2.json --rescore-rail
  docs/data/kpi_ntFIN2/rescore_r2_ntFIN2s.json`, code revision f505f1403bef
  (clean). 589 KPI-grade episodes (580 ntFIN2, 9 ntFIN2s, 4 superseded):
  P0 violation escape rate 0.0 over 586 Shield-on flights (Shield-off
  control 0.6375); fail-safe trigger correctness (r2) 566/567 = 0.998,
  Wilson 95 % low 0.990; `tick_ms` max 74.06 ms, 0 of 173,410 over 100 ms;
  `e2e_ms` p99 30.3 ms, 13 ticks over 100 ms (the start-up backlog, below).
  Tracked, merged over both campaigns: mean time to safe 3.35 s, repair
  success 98.1 %, mission success 252/586, 54.6 repairs per Shield-on
  mission.
- `docs/data/kpi_ntFIN2/rescore_r2_*.json` / `.txt`: the adopted-reading
  re-scores of ntFIN2, ntFIN2s and hs09
  (`experiments/sweep_scenarios.py --profile nightly --rescore-rail`).

#### Flown
- hs09 (2f22116, hil, nightly profile `--only nfz-head-on
  corridor-drift-angles wind-beyond-authority --seed 1001`): 17 missions,
  every mission's row 2 ticked on its action (`tick_trigger` action 17/17,
  deferred 0), `e2e_ms` p99 16.8 / max 24.6 ms, P0 escape 0. Under the
  adopted reading 17/17 = 1.000 (Wilson 95 % low 0.816); the sweep's own
  ideal labels fail nfz-head-on canonical (RTL triggered where the ideal
  label expects none), the rail-vehicle case the r2 reading covers. The
  start-up fix (5f5009c) is therefore flown; ntFIN2's 13 start-up e2e ticks
  stay in its report as flown.

### 2026-11-13 (the KPI-grade hil nightly ntFIN2 and its supplement; the contract outline re-assessed) — 591 missions at 116214f on the hil rig: P0 escape 0 over 586 Shield-on flights, fail-safe trigger correctness 566/567 = 0.998 under the adopted reading (Wilson 95 % low 0.990), slowest Shield tick 74.06 ms

#### Flown
- ntFIN2 (116214f, hil, nightly profile with `--hil-controls`, launched
  detached from the session after a first attempt, ntFIN, was stopped at
  176 of 591 missions when the session ended; 19.4 h): 583 run folders, 580
  KPI-grade episodes. The last 8 missions (wind-beyond-authority) never flew:
  the desktop ran out of commit memory at 13:40 (WinError 1455 "the paging
  file is too small", then dwm.exe and the browsers failed with out-of-memory
  codes); 3 gps-dropout missions were not accepted (the stress was not
  realised in time).
- ntFIN2s (the same commit and profile): the affected cells on all seeds,
  15 missions, 11 accepted; gps-dropout lane 15, 6-13 s, seeds 1002 and 1003
  were again not accepted (SIM_GPS_DISABLE=0 applied 0.29-0.60 s late).

#### Measured (tools/kpi_report.py --campaign ntFIN2 --campaign ntFIN2s)
- 589 KPI-grade episodes (580 from ntFIN2, 9 from ntFIN2s; 4 supplement
  flights superseded, not counted).
- P0 violation escape rate 0.0 over 586 Shield-on flights (16,760
  P0-violation ticks); Shield-off control 0.6375.
- Fail-safe trigger correctness, adopted reading (r2): 566/567 = 0.998,
  Wilson 95 % low 0.990; (a) 495/567, (r) 554/567. The one miss is
  ntFIN2_177 (dyn-nfz-spawn-ahead, E5). 19 runs ended still holding
  (censored, not scored).
- `tick_ms` p99 29.5 / max 74.06 ms, 0 of 173,410 over 100 ms. `e2e_ms` p99
  30.3 / max 121.93 ms: 13 ticks over 100 ms, all row 2 at t = 0.101 s (the
  start-up backlog, fixed in the section below, not yet flown).
- Tracked: mean time to safe 3.23 s; repair success 98.2 %; mission success
  252/577; 55.4 repairs per Shield-on mission (ntFIN2).
- Read with care: no flown cell uses its seed, so a cell's three seeds are
  three repeats of one deterministic flight (389 distinct results over 580
  ntFIN2 episodes) and the Wilson bound treats them as independent; in 21
  Shield-on missions ArduPilot (its GeoFence backstop or its RTL leg), not the
  Shield, carried the aircraft into a P0 zone.

#### Changed
- docs/FINAL-REPORT-outline.md Appendix A re-assessed on ntFIN2 against the
  grant's own words: WP3-14, WP3-16, WP3-17, WP3-20, X-02, X-03, WP4-08 and
  WP4-12 to DONE (78 DONE / 51 PARTIAL / 6 DRIFT / 1 MISSING); WP3-15, WP3-13
  and WP4-17 stay PARTIAL with their remaining gaps stated; an
  independent-review note lists what a reviewer may still dispute.

### 2026-11-13 (the start-up tick; a KPI report from a campaign and its supplements) — ntFIN2's only ticks over 100 ms end to end were each mission's row 2, deferred behind the backlog of the node's own start-up work, which is now delivered before the mission clock starts; `tools/kpi_report.py` reads a main campaign and its re-flown supplement by one episode choice

Branch kpi-merge-startup (5f5009c, 72dd535). **The start-up change has not
flown**; the report change was run on ntFIN2 and ntFIN2s (below).

#### Found (ntFIN2, the final KPI-grade hil nightly at 116214f)
- 13 of 171,393 ticks over 100 ms end to end (e2e 100.6-121.9 ms), every
  one row 2 of its mission at t 0.101 s, `deferred`. `start_mission` runs
  ~90 ms without spinning (the Shield warm-up ~12 ms, the collector's start
  ~75 ms), so the pilot message published meanwhile waited in the DDS queue,
  was delivered at +2-3 ms after t = 0 and ticked as live; the pilot's next
  message, 12-18 ms later (its receipt is the row's t + tick_ms - e2e,
  +15-21 ms), was then taken for a pilot faster than 1 / min_gap and
  deferred to the poll 80-86 ms on. Row 2 was deferred so on 418 of the 583
  missions; the 13 over budget are the ones whose deferred tick took 15-36
  ms of work.
- ntFIN2 lost 11 missions to the rig: 8 wind-beyond-authority missions
  never flown (a SITL that did not come up, an `Errno 22`, then `WinError
  1455` / `1450` once the desktop ran out of memory) and 3 gps-dropout
  flights whose stress was not realised. They were flown again
  at the same commit as ntFIN2s, on all 3 seeds of the 5 cells (15
  missions, 4 of them cell-seeds ntFIN2 had already flown and accepted).
  Until now the report took one campaign, and a run that campaign did not
  list was not KPI-grade.

#### Changed
- `sitl/ros2_shield_node.py` (THE START-UP TICK): with `--tick-trigger
  action` the `TickScheduler` exists from the node's start and holds a
  message received before the mission clock starts (`held_t`; never ticked,
  not counted by the rate limit), so the first message after the start
  ticks at once; the first watchdog tick waits period + slack when a
  message came within one period before the start (a live stream, as after
  a message's tick). `start_mission` delivers what queued during its own
  work before the clock starts (`_deliver_backlog`: spins of no wait, at
  most 200 and 30 ms). The held action is flown only by that watchdog, for
  a quiet stream, under the stale-action rule. The steady state (the
  message-gap rate limit, the 10 Hz watchdog) is unchanged; `--tick-trigger
  timer` is unchanged. Recorded in the `mission_start` event
  (`start_backlog`) and metrics.json `tick_schedule.start`.
- `tools/kpi_report.py`: `--campaign` and `--rescore-rail` are repeatable.
  The first campaign is the main one, later ones supplements, by rules the
  report prints under Inputs (`EPISODE_CHOICE_RULES`): every campaign at the
  same code revision (the summaries' `_manifest_common` and every listed
  run's manifest), profile and backend, refused otherwise; an episode is
  (cell id, seed, Shield arm); the main campaign's accepted episode wins and
  a supplement's counts only where no earlier campaign has an accepted one;
  a supplement flight of an accepted episode is superseded (listed, not
  read, not counted); a not-accepted flight stays functional evidence with
  its replacement named. The Campaign section lists the episodes taken from
  a supplement and what the main campaign had for each. Several re-score
  tables must share the label artefact, its build, the adopted reading and
  the rail gates; they merge per run, and their own figure is recomputed
  over the same episode choice. The KPI-grade gate (`is_kpi_grade`, the
  signed bundle bound to the run) is unchanged. One campaign reads as
  before.

#### Run (ntFIN2 + ntFIN2s, both at 116214f; read-only, report not committed)
- ntFIN2s flew 15 missions: 11 accepted, 4 gps-dropout lane 15 not
  accepted (the denial's end applied 0.29-0.60 s late).
- `python tools/kpi_report.py --campaign demo/out/hil_ntFIN2 --campaign
  demo/out/hil_ntFIN2s --rescore-rail <ntFIN2.json> --rescore-rail
  <ntFIN2s.json>` (each table from `experiments/sweep_scenarios.py
  --profile nightly --rescore-rail`): 594 episodes, 589 KPI-grade (580 from
  ntFIN2, 9 from ntFIN2s: wind-beyond-authority lane 15 at 12 m/s seeds
  1002-1003, lane 20 at 8 and 12 m/s all seeds, gps-dropout lane 15 4-11 s
  seed 1003); 4 ntFIN2s flights superseded (two accepted, two not, all of
  cell-seeds ntFIN2 accepted); 5 not accepted (gps-dropout lane 15 6-13 s
  seeds 1002 and 1003 have no accepted flight in either campaign).
- P0 escape rate 0.0 over 586 Shield-on flights (0 escapes on 16,760
  P0-violation ticks) against 0.6375 on the 3 Shield-off controls.
- Fail-safe trigger correctness, adopted reading (r2): 566/567 = 0.998,
  Wilson 95 % low 0.990 (ntFIN2 alone: 557/558); the one miss is
  ntFIN2_177 (dyn-nfz-spawn-ahead, E5). The tables' own figure over the
  same episode choice agrees (566/567 over 591 runs).
- `tick_ms` max 74.06 ms, 0 of 173,410 ticks over 100 ms; `e2e_ms` p99
  30.3 / max 121.93 ms, 13 over 100 ms - the 13 start-up ticks above (the
  supplement flew the same commit).

#### Tests
- `tests/test_sitl_rails.py`: the start-up case on the pure schedule (as
  flown, the backlog after t = 0 defers the live message to the poll at
  +100 ms; held before the start, the live message ticks at once at every
  phase of the poll grid) and on the stand-in node (the drain delivers the
  backlog before t = 0, and the next message's e2e equals its tick_ms: 12.0
  of 12.0 ms of work; the control without the drain: 97 ms; a held message
  older than the stale-action timeout is not flown).
- `tests/test_kpi_campaign.py`: two synthetic campaigns and their re-score
  tables: precedence, duplicates superseded (an accepted and a
  not-accepted one), which campaign each counted episode came from, the
  merged readings with the superseded false trigger left out, refusal on
  another code revision (summary or run manifest), profile, a missing
  revision, a campaign or table given twice, and tables that disagree.

### 2026-11-12 (hs07 shakedown; the tick scheduler judged on message gaps) — the hil shakedown at dc8cd9c scored 30/30 on the adopted reading with tick_ms max 56.6 ms, and found the action-triggered tick locking into deferral; the rate limit now reads the gap between messages

#### Flown
- hs07 (dc8cd9c, hil, nightly cells on seed 1001, 30 missions: wind
  beyond and within authority, gps-dropout, corridor-drift-angles,
  dyn-nfz-rotate-spin, nfz-head-on; 70 min). P0 escape 0.0 over the 30
  Shield-on missions. Under the adopted reading (r2) 30/30 = 1.000 (Wilson
  95 % low 0.886): the wind-beyond-authority 12 m/s cells went home (X9)
  and gps-dropout lane 23.6 took no X8.
- `tick_ms` p50 9.5 / p99 31.8 / max 56.56 ms, 0 of 11386 over 100 ms
  (sm1111: max 95.98 ms): the paced garbage collection held.
- `e2e_ms` p99 98.7 / max 110.72 ms, 49 of 11386 over 100 ms.
- wind-beyond-authority lane 20 at 12 m/s: after X9's RTL, ArduPilot's
  return in the 12 m/s wind carried the aircraft into nfz-square (92 ticks
  inside, autopilot-flown, not a Shield P0 escape). A wind beyond the
  airframe's authority carries it whatever is commanded; how the fail-safe
  should choose between RTL and Land in such a wind is open.

#### Found
- The tick scheduler's rate limit was judged against the last TICK's start.
  With the rig's exact 10 Hz pilot and exact 40 Hz poll, once one tick was
  deferred (an early burst), every later message arrived under min_gap after
  that deferred tick and was deferred by the same 20-35 ms for the rest of
  the mission: 22 of 30 hs07 missions ran every tick but one "deferred", and
  all 49 ticks over 100 ms were deferred ones. Ticks run on arrival: p99
  35.9 ms, max 56.7 ms, none over 100 ms.

#### Changed
- `sitl/ros2_shield_node.py` `TickScheduler.on_message` judges the rate limit
  on the gap since the previous MESSAGE: an early burst defers one tick and
  the next message ticks at once; a pilot faster than 1 / min_gap is still
  held to it. `tests/test_sitl_rails.py`: an exact pilot with a burst at
  every phase of the poll grid takes at most one deferral (the old rule:
  1 action, 200 deferred).
- `tools/kpi_report.py`: the e2e note no longer says the node ticks on its
  own timer.

### 2026-11-12 (GPS denial as the rig flies it; X8 on the autopilot's own estimator) — the rail labels read a GPS denial as EKF3's inertial dead reckoning, and on the ROS 2 rail X8 fires when the autopilot declares its position lost instead of on the Shield's command dead reckoning plus a fixed drift allowance; replayed on the rig's own flights, not flown

Branches gps-dropout-fix (49c455c, d673ed0, 635336a) and
x8-estimator-health (1315a9b, b34bc39). **Nothing here has flown.**

#### Found (nt1111, the KPI-grade hil nightly at 22d434b)
- Under the adopted reading (r2) nt1111 scored 555/567 = 0.979 (Wilson
  95 % low 0.963). Twelve errors: the six wind-beyond-authority 12 m/s
  cells (lanes 15 and 20, which X9 answers: replayed, X9 fires on all six
  at 11.0-11.2 s and on no other nt1111 run but nt1111_001, already
  labelled expected) and six on gps-dropout lane 23.6.
- The rig realises a GPS denial through SITL (`SIM_GPS_DISABLE`): EKF3
  keeps estimating on the IMU, so the pose handed to the Shield keeps
  moving. Against SITL's own truth (SIMSTATE in every run's bag) its error
  was at most 1.47 m over the rig's 30 denials. The rail labels read a
  frozen last fix that no rail hands the Shield.
- X8 fired on lane 23.6 at 4-11 s and 6-13 s (nt1111_568-573) with EKF3
  0.13-0.78 m from the truth and the aircraft 8.5-9.0 m from the zone: its
  error budget was the Shield's own command dead reckoning (1.3-3.5 m from
  EKF3's pose) plus the 0.5 m/s drift allowance. Tuning the allowance is a
  knife edge (0.23 m/s, the measured envelope, clears by 0.01 m) and was
  rejected.
- Raw MAVLink of 731 rig flights: ArduPilot's EKF_STATUS_REPORT stayed
  healthy (flags 0x33f, test ratios at most 0.049) on every flight; in 23
  of the 24 7 s denials EKF3 dropped absolute horizontal position for
  0.10-0.21 s at 6.8-6.9 s (its 7 s posRetryTimeNoVel timer) before the GPS
  returned; ArduCopter's own 1 s ekf_check never declared the position bad.

#### Changed
- `experiments/sweep_scenarios.py`: `GPS_DROPOUT_MODELS` ("frozen", the
  default, and "dead_reckoned": the truth lagging by the measured
  `RAIL_EKF_DRIFT_MPS` 0.23 m/s along the track, `fix_ok` False);
  `RAIL_VEHICLE` flies "dead_reckoned" and hands `estimator_ok` True until
  `RAIL_EKF_POS_VALID_S` after the loss. Frozen-fix cells are unchanged.
- `tools/failsafe_rubric.py --update-labels`: re-derives only the cells
  given, keeps every other reading, logs why and what moved. The 24
  gps-dropout rail cell-seeds in `docs/data/failsafe_labels_rail.json` were
  re-derived: lane 23.6 at 2-9 and 4-11 s go from expected (E3) to not
  expected; lane 15 keeps its fail-safe by E5. Nothing else moved.
- `guardrail/models.py` `State.estimator_ok`; `guardrail/shield.py`
  `_position_verdict` reads it first: while the autopilot vouches for its
  estimate the handed pose is certified, with or without GPS; when the
  autopilot declares it lost X8 fires at once; with no or a stale report
  the old law (command dead reckoning + `DR_DRIFT_MPS`) runs from the last
  certified pose.
- `sitl/ros2_shield_node.py` subscribes `/uas1/mavlink_source` and keeps
  EKF_STATUS_REPORT (id 193) from the autopilot; "lost" follows ArduCopter's
  own rule (absolute horizontal position, no constant-position mode, test
  ratios below FS_EKF_THRESH, for 1 s; FS_EKF_THRESH / FS_EKF_ACTION read at
  take-off); rows and `metrics.json fix_quality.estimator` record it.
  `sitl/sros2_policy.xml` grants the subscription (the hil topology runs
  without SROS 2; a secured launch must re-sign its permissions).
- The FSM config hash is unchanged (comments only in `guardrail/fsm.py`).

#### Measured (replay, not flight)
- The new law never returned "lost" or "not reported" over 899,299 armed
  samples of the 731 bags, and returned healthy on all 1,798 ticks with
  the GPS fix lost: no rig flight takes X8 under it; it removes the nine
  logged X8s (nt1111_568-573, rh03_040, sm1110_044, sm1111_044).
- nt1111 re-scored with the new labels: r2 555/567 unchanged in count (the
  errors move to six X8 false triggers). Predicted with X9 and the new X8:
  567/567 (Wilson 95 % low 0.993). Headless rail vehicle: lane 23.6 at
  4-11 / 6-13 s fly G1, G7 and no fail-safe; lane 15 keeps G5 at
  35.8-36.1 s (rig 35.8-36.0 s).

#### Open
- Not flown; the callback cost of ~300 MAVLink messages a second on the
  Orin is to be measured on the rig.
- While the autopilot reports healthy its error (up to 1.47 m on SITL; a
  real IMU drifts faster) is covered only by the hard P0 margin: sizing
  margins to the rail's accuracy is a policy decision.
- A question for the PI: X8 is an extension edge, and on the ROS 2 rail it
  now means "the autopilot says its position is lost".

### 2026-11-11 (Shield node timing) — the hil Shield node waits once, bounded, for SIM_SPEEDUP before its take-off fence and stressor reads, ticks on the pilot's action instead of its own timer, runs CPython's full garbage collection after a publication instead of inside a tick, and records the camera frame's age at each setpoint; not flown

All of it is `sitl/ros2_shield_node.py` (3a3f505) and the event hold of
`guardrail/replay.py`. **Nothing here has flown**: the rig's numbers are to
be measured (a KPI campaign was in the air while it was written).

#### Found
- sm1111 (22d434b): no tick took 100 ms (`tick_ms` max 95.98 ms, p99
  43.2 ms), yet `e2e_ms` p99 was 106.1 ms: the tick ran on the node's own
  10 Hz timer, at a phase unrelated to the pilot's 10 Hz stream, so a fresh
  action waited up to one period for the next tick (91.6 ms of wait at p99).
- ge1110 (9eed35b): its 29 ticks over 100 ms all sat at row 1305-1339 of
  every long mission, inside the Shield's own `filter()`: CPython's full
  (generation 2) collection, triggered by the rows and trajectory points the
  node keeps for its report.
- The rosbags of sm1111 (48) and hs06b (9): the autopilot sent SIM_SPEEDUP
  ~0.5 s before FENCE_ALT_MAX on every run; on the 9 runs that missed it,
  MAVROS took it only on its re-request, 11.3-23.0 s after FCU connect: past
  the fence read's 5 s grace on all 9, past a 20 s wait on 5. The stressor
  read (one read) then had no answer, and two stressed sm1111 missions
  (sm1111_044, sm1111_047) were refused by the harness although their
  stressors flew.

#### Changed
- **One bounded wait for SIM_SPEEDUP** (`takeoff_speedup`,
  `SIM_SPEEDUP_WAIT_S` 40 s) runs once in bring-up, before the take-off
  fence read and the stressor read; both take its answer
  (`fence_read_done(speedup_polled)`, `speedup_is_sitl`,
  `sim_params_record`) and fail closed only after it. On SITL it ends at the
  first answer; only a hardware autopilot waits it out on the pad.
  `autopilot_fence` records `sitl` and the wait.
- **The tick on the action** (`--tick-trigger action`, the default;
  `TickScheduler`): a valid `/vla/action_4d` message runs the tick in its
  own callback; one under 80 ms after the previous tick is deferred to the
  first 25 ms poll past the gap; with no message the poll ticks 100 ms after
  a timer tick (150 ms after a message's tick). One tick per message at a
  10 Hz pilot, never over 12.5 Hz, so e2e no longer includes up to 100 ms of
  timer wait. The FSM, the watchdogs, the hold planner's slices and the
  stale-action rule are unchanged; the receipt is stamped at the callback's
  entry.
- **The paced collector** (`--gc-pacing on`, the default; `GcPacer`): at
  mission start everything alive is collected and frozen, the automatic full
  collection is switched off, a full collection runs every 10 s right after
  a publication and freezes what survived; CPython's own schedule comes back
  at mission end, before the report.
- **After the publication**: the audit record, the events.jsonl appends
  (`EpisodeRecord.hold_events` / `release_events`, per thread), the
  intercept topic, the deferred-fence re-check, the publisher count and the
  stream-rate re-assert.
- **The camera's age at the setpoint**: `camera_age_ms` on each setpoint
  flown from a camera-backed action, the bridge's `decision_age_s` plus this
  node's receipt-to-publication (two durations, each on one monotonic
  clock).
- Rows: `tick_trigger`, `camera_age_ms`. metrics.json: `tick_schedule`,
  `gc`, `camera_to_setpoint` (p50 / p95 / p99 / max; it says there is no
  camera behind a stub's actions instead of reading zero), and
  `tick_latency` says what e2e measures (with p95).
- `--tick-trigger timer` and `--gc-pacing off` fly the old behaviour (the
  A/B on the rig).

#### Measured (desktop, not the rig)
- ge1110_001's 3502 ticks replayed through the node's per-tick code with its
  rows kept (Python 3.12): a full collection at row 1320 took 15.0 ms inside
  the tick with the replay's own heap (87k tracked objects) and 53.3 ms with
  800k more standing in for the rig's ROS stack. Paced, none ran inside a
  tick; the pacer's 34 ran after the publication in 0.37-0.58 ms each, and
  its start (collect and freeze, before the mission clock) took 11.5 /
  53.8 ms.

#### Tests
- `tests/test_sitl_rails.py`, 8 new. The hand-ticked stand-in builders pass
  `--tick-trigger timer --gc-pacing off` (`HAND_TICKED` in
  `tests/test_railinputs_node.py`, used by `tests/test_sitl_stressors.py`
  too); the defaults are tested in `tests/test_sitl_rails.py`.

#### Not done
- Not flown: the rig's e2e under the action tick, the tick rate on a real
  pilot stream, the pacer's cost on the Orin and the wait at a 50 m start.

### 2026-11-11 (X9, a hold that is not kept) — Loiter goes home when the aircraft is carried beyond what a kept hold can carry it and is still receding: sm1111_048's missed fail-safe (wind beyond authority, 12 m/s) answered; replayed on the rig's own Loiters, not flown

#### Found
- sm1111_048 (wind-beyond-authority, lane 15, 12 m/s), the one wrong run of
  sm1111 under the adopted reading (r2, 44/45), missed its fail-safe: G1 at
  5.3 s, G3 at 5.8 s, then the companion's GUIDED hold commanded up to 4 m/s
  back to its point while the SITL airframe receded from it at 2-4 m/s for
  9.4 s, round nfz-square with the position legal throughout (2.4 m from its
  face); X6 and X7 never ran, and G8 handed the aircraft back at 15.3 s.
  Nothing told the FSM whether its hold was being kept.

#### Changed (3146da6)
- `guardrail/fsm.py`: edge X9 (Loiter -> RTL, an extension): the aircraft
  beyond what a kept hold carries it (`TickInput.hold_reach_m`) on every
  tick for T_recover, and its distance still growing by X7's fitted-trend
  test. After G4, before G8. New law field `FSMConfig.hold_kept` ("reach",
  the default; "off", the FSM at 22d434b): the default config hash is now
  `sha256:b9103b0d...`; "off" hashes `sha256:e0d9267f...`, the default
  before. Records carry `hold_off_m` / `hold_reach_m` / `x9_unkept` only on
  Loiter ticks judged under the law; `tick_input_from_decision` and
  `tick_input_from_row` pass the inputs through.
- `guardrail/hold_flight.py`: `HoldWatch` and `hold_reach`: the distance
  when the point was set, plus the braking distance from the speed then on
  the rail's stop model (`ARDUCOPTER_GUIDED`: 0.45 s, 1.75 m/s^2), plus
  3.0 m of settling (`HOLD_KEPT_SLACK_M`); `HoldFlight.point`.
- `guardrail/shield.py`: `filter()` and `_escalate()` take the two inputs.
- `experiments/sweep_scenarios.py`: the harness feeds `HoldWatch` the true
  position and the last tick's ground velocity, as the rails do.
- `sitl/ros2_shield_node.py`: the node asks its `HoldWatch` before each FSM
  step and writes `hold_off_m` / `hold_reach_m` on Loiter rows. Merged with
  the Shield node timing entry above: the X9 inputs are computed inside the
  tick before the FSM steps, and the two row fields are written with the
  other row fields after the publication.
- `tools/prefix_eval.py` (merged with this entry, b42b22e): a recording
  whose Shield flew the FSM on the previous default (`sha256:e0d9267f...`)
  was refused as "not the defaults". It is now judged under the law it flew
  (`hold_kept` "off", rebuilt exactly; `FSM_PREVIOUS_DEFAULTS`), never under
  today's default, and the report says which (`shield.fsm_law`, the note, a
  summary line). Any other non-default hash is still refused.
- Docs: `docs/RUBRIC-failsafe.md` (no clause or label changed; X9 flagged as
  a deviation question), `docs/DESIGN-escalation-fsm.md` (section, table
  row, edge order, PI question 14), `docs/DESIGN-hold-points.md`, the wind
  template's text.

#### Measured (the rig's own logs replayed; not flown)
- Replayed on all 413 Loiter spells the hil rig has flown, X9 fires on the
  five 12 m/s lane-15 cells (hs02_009, hs03_011, rh03_044, sm1110_048,
  sm1111_048; at 11.2-11.4 s) and on nt1111_001 (a SITL restart left the
  hold 9.3 m past its point; r2 already expects a fail-safe there), never on
  an 8 m/s or within-authority cell; the closest any other spell came to its
  reach was 0.8 m short. Predicted for sm1111 under r2: 45/45.
- Headless, the wind-beyond-authority cells on the ideal vehicle go home by
  X9 (6.8 s at 12 m/s, 9.0 s at 8 m/s) instead of X7 (7.3 s, 17.3 s).

#### Tests
- `tests/test_fsm.py` (X9's semantics, the hashes, edge coverage),
  `tests/test_hold_points.py` (`HoldWatch`; a replay of sm1111's own
  Loiters, `tests/fixtures/hil_sm1111_wind_loiters.json`),
  `tests/test_harnesstruth_wind.py` (the wind cells headless),
  `tests/test_railinputs_node.py` (the node's wiring), the c77c77d identity
  law pinned with `hold_kept` off (`tests/test_own_velocity_forecast.py`,
  `tests/test_hold_points.py`), `tests/test_prefix_eval.py` (the previous
  default judged under its law, today's unchanged, any other refused).

#### Not done
- X9 is an extension edge: the grant's FSM draws Loiter -> RTL only for
  N-in-T persisting (G4) and the operator timeout (G5). A question for the
  PI, a documented deviation (`docs/DESIGN-escalation-fsm.md`, question 14).
- Not flown; the headless library, smoke and nightly profiles were not
  re-run with it.

### 2026-11-11 (KPI report from a hil campaign) — `tools/kpi_report.py --campaign` writes the WP4 KPI report from a hil campaign under the adopted fail-safe reading; the perception rail logs the camera frame's age at each setpoint; the track video draws moving zones where they were; not flown

#### Added
- `tools/kpi_report.py --campaign <demo/out/hil_<c>> --rescore-rail
  <rescore.json>` writes the WP4 KPI report from a hil campaign (052620e,
  fbb3f4f). `--campaign` takes the runs from the campaign's store
  (`<store>/<campaign>_*`), which the default globs never reached; a run
  the campaign did not accept or does not list is demoted from KPI grade
  with the reason (the KPI-grade gate itself is unchanged). `--rescore-rail`
  (the `--out` JSON of `experiments/sweep_scenarios.py --rescore-rail`)
  applies the adopted label reading (r2) to the family table and the top-K,
  keeping the label the run flew with beside it (`failsafe_at_flight`); a
  new section scores (a), (r) and (r2) side by side over the KPI-grade
  Shield-on runs: the point estimate, the Wilson 95 % lower bound, both
  verdicts and nulls, the runs left out and the false and missed triggers by
  name. A KPI summary leads the report; its Inputs line names the
  campaign's acceptance.
- The tick budget in that report reads `tick_ms` and `e2e_ms` (hil Shield
  node) and the perception rail's `cam_age_ms` / `cam_to_setpoint_ms` from
  every row of the whole flight, pooled per rail (p50 / p95 / p99 / max, the
  count over 100 ms), KPI-grade apart; a measure nothing carries is "not
  measured", never 0.
- The perception rail (`demo/pas_ardupilot/perception_node.py --pilot vla`,
  2c7d683; WP3-13): every row carries `t_pub` and `tick_ms`; a VLA row whose
  decision's action flew carries `cam_age_ms` (`t_pub` minus that
  decision's `t_capture`), and `cam_to_setpoint_ms` on the first tick that
  flies each decision; metrics.json `camera_to_setpoint`. A lower bound: on
  this rail a capture's time is the frame's arrival, not its render
  (`t_capture_basis` "arrival").
- `tools/render_track_video.py` (2c25c9f): a `dynamic_nfz` with `motion` is
  drawn where it is at each frame, at `DynamicNFZ.ring_at` from its last
  hot-apply (or, with none, the first row of the earliest generation
  holding it); the footer names the run's real log path; `--tag` may name a
  nested run; `--stills T1,T2` draws the nearest frames as PNGs (no ffmpeg).

#### Measured
- sm1111 (22d434b) with its re-score: 46 KPI-grade runs (2 not accepted);
  P0 escape rate 0.0000 (0 of 2191 P0 ticks, 45 flights) against 0.6500 for
  the Shield-off control; fail-safe (r2) 44/45 = 0.978, Wilson low 0.884
  ((a) 39/45, (r) 43/45); `tick_ms` max 95.98 ms, 0 over 100 ms; `e2e_ms`
  p99 106.4 ms, 351 of 14589 over (pooled over the KPI-grade runs' rows; the
  campaign's own figure, 106.1 ms, is in the hil rig flights entry below).
- The track video of hs06b_004 (dyn-nfz-rotate-spin): the drawn bar's signed
  distance to the aircraft is within 0.21 m of the one the Shield logged
  (`zone_d_m`) on all 355 rows of its generation.

#### Tests
- `tests/test_kpi_campaign.py` (new: 7, and 1 on the delivered data),
  `tests/test_wp4outputs_kpi_report.py`, `tests/test_kpi_report.py`,
  `tests/test_pas_vla.py` (the field arithmetic, a VLA GOTO on the
  stand-ins, a held tick claims none), `tests/test_render_track_video.py`
  (6 new).

#### Not done
- Not flown: no simulator or Orin run of the perception rail's new fields.
  The report was made from a smoke campaign, one flight per cell: no KPI
  number for the final report until the nightly hil campaign is scored.

### 2026-11-11 (hil rig flights, 7-11 Nov) — the stress library flown on the hil rig: three KPI-grade smoke campaigns, the grant's example sweep and the hold shakedowns; P0 escapes 0 on every Shield-on mission; the rig-only defects they found, their fixes, and what is still open

The rig of the 2026-11-06 (hil) entry: ArduCopter 4.5.7 SITL and
mavlink-routerd in WSL on the desktop; MAVROS 2, the Shield node and the VLA
stub in the companion container on the AGX Orin. Every mission was flown by
`experiments/sweep_scenarios.py --backend hil`, one scenario per mission, and
scored by the campaign (`score`, `episode_verdict`). The pilot is the stub
(`guardrail/vla_stub.py`), not a learned model. Every flown run's manifest
reads topology `hil`, sim_speedup 1.0 and a clean code revision, with the
Orin's `hil_evidence` beside it, and every flown run folder holds its signed
replay bundle (`<run>.replay.tar.gz`: the episode's manifest, flight log,
rosbag2 bag, kpi.json and policy, and shield_audit.jsonl where the run wrote
one) and, once its campaign accepted it, `scenario_score.json`. Run folders `demo/out/hil_smoke/<campaign>_*`; each
campaign's `demo/out/hil_<campaign>/sweep.json` and `kpi_report.md` (the run
store, not tracked).

#### Flown

| campaign | day | code | missions | what |
|---|---|---|---|---|
| rh03 | 7 Nov | fb96fe5 | 44 | the smoke profile (64 episodes; 16 the rail cannot fly as written, the Shield-off control not asked for, 3 stand-off cells whose signed bundle was not on disk) |
| hs01 | 7 Nov | 5fc832b | 2 | hold shakedown; both missions died at FCU connect (below) |
| hs02 | 7 Nov | d186077 | 9 | hold shakedown: rotate-step, rotate-spin@3, the sideways window, radius grow, corridor swap, the land zone, three violations, wind beyond authority 8 and 12 m/s |
| hs03 | 7 Nov | 199cfdf | 11 | hs02's cells and corridor-drift-angles 8 m/s at -60 and 0 deg |
| hs04 | 7 Nov | ee356c6 | 9 | dyn-nfz-rotate-spin at 3, 6 and 9 deg/s, seeds 1001-1003 (the nightly's nine cells) |
| hs05 | 7 Nov | 9eed35b | 9 | hs04's nine |
| nt1107 | 7 Nov | 9eed35b | 9 of 591 | the nightly profile; the campaign went no further than its ninth mission (no sweep.json) |
| sm1110 | 10 Nov | 9eed35b | 48 | the smoke profile with the Shield-off control (`--hil-controls`; 16 the rail cannot fly as written) |
| ge1110 | 10-11 Nov | 9eed35b | 36, 32 flew | the grant's example sweep (`--profile grant_example`, 50 m AGL) |
| hs06a | 11 Nov | 22d434b | 4 | corridor-drift-angles 8 m/s at -60 and 0 deg, wind-within-authority 5 m/s in lanes 15 and 25 |
| hs06b | 11 Nov | 22d434b | 9 | hs04's nine |
| sm1111 | 11 Nov | 22d434b | 48 | the smoke profile with the control (16 the rail cannot fly as written) |

The nightly campaign nt1111 (the nightly profile, 591 missions at 22d434b,
`--hil-controls`) was launched on 11 Nov (first mission 05:59) and is still
flying; nothing of it is reported here.

#### Measured (the four KPI-grade campaigns)

| | rh03 | sm1110 | ge1110 | sm1111 |
|---|---|---|---|---|
| code | fb96fe5 | 9eed35b | 9eed35b | 22d434b |
| Shield-on missions scored | 44 | 47 (+1 control) | 32 (4 died before the mission) | 45 (+1 control; 2 failed closed) |
| P0 escape rate, Shield on (highest of any mission) | 0.0 | 0.0 | 0.0 | 0.0 |
| P0 escape rate, Shield-off control (nfz-head-on-control) | not flown | 0.646 (51 escapes) | not flown | 0.650 (52 escapes) |
| breaches flown by the Shield | 0 | 0 | 0 | 0 |
| mission success | 19/44 = 0.432 | 23/47 = 0.489 | 32/32 | 22/45 = 0.489 |
| repairs per episode | 72.1 | 76.1 | 0 | 72.1 |
| mean repair magnitude (mean over episodes) | 5.32 m/s | 5.28 m/s | no repair | 5.22 m/s |
| mean time to safe (episodes with a spell; max) | 5.73 s (12; 10.4 s) | 3.65 s (11; 5.3 s) | no spell | 3.64 s (11; 5.3 s) |
| fail-safe (a), ideal-vehicle labels | 37/43 | 40/45 | 32/32 | 39/45 |
| fail-safe (r), rail-vehicle labels | 41/43 | 43/45 | 32/32 | 43/45 |
| fail-safe (r2), rail + E5 on the prescribed flight | 42/43 (Wilson low 0.879) | 44/45 (0.884) | 32/32 (0.893) | 44/45 = 0.978 (0.884) |
| tick: longest, over 100 ms | 74.25 ms, 0 of 7,716 | 94.1 ms, 0 of 16,268 | 120.4 ms, 29 of 82,971 | 95.98 ms, 0 of 15,050 |
| e2e p99 (an action's receipt to its first setpoint) | 97.9 ms | 98.5 ms | 97.9 ms | 106.1 ms |
| campaign time | 1.48 h | 1.64 h | 3.40 h | 1.64 h |

- (a) is the campaign's own line, the ideal-vehicle labels its sweep prints.
  (r) and (r2) are the label artefact `docs/data/failsafe_labels_rail.json`
  scored with `--rescore-rail` on branch adopt-rail-labels (2b68516,
  0df7ee8), which adopts (r2) as the KPI-grade reading; that branch merged
  beside this entry (integrate/1112; its own entry is 2026-11-10 (fail-safe
  labels adopted)). Its commit records the adoption as the user's decision of 9-10 Nov,
  taken after the rig had shown the autopilot's lag, not a pre-registered
  reading. A run its campaign did not accept (sm1111_044, sm1111_047) is in no
  reading. The 2026-11-07 (hold horizon) entry's rh03 figures, (r) 38/40 and
  (r2) 39/40, were read on the proposal's labels before the artefact existed.
- Under (r2): one false trigger, rh03_017 (dyn-nfz-rotate-spin@3 at fb96fe5,
  holding in place: G1, G3, X6), and one miss in each later smoke campaign,
  sm1110_048 and sm1111_048 (wind-beyond-authority, lane 15, 12 m/s: G1, G3,
  then G8 at 14.3 / 15.3 s and no fail-safe). Under (a) the false triggers are
  nfz-head-on, rail-mission-stub-vla, dyn-nfz-rotate-step and
  time-window-opens-on-sideways-pilot in every smoke campaign, with
  rotate-spin@3 and wind-within-authority (lane 15) in rh03 and
  wind-within-authority (lane 15) in sm1111. ge1110's 32/32 decides nothing:
  every label is N1, so never triggering scores the same.
- What ArduPilot's own modes flew is outside the Shield's escape rate by
  definition and reported apart: rh03 5 autopilot breaches in 5 episodes and
  2 polygon entries; sm1110 and sm1111 5 and 1 each; ge1110 none.
- `tick_ms` is the Shield node's whole tick; `e2e_ms` runs from an action's
  receipt to the first setpoint flown from it and includes up to one 100 ms
  tick period of waiting for the node's own 10 Hz timer (metrics.json's
  `tick_latency` note). sm1111's e2e p99, 106.1 ms, is over WP3-13's 100 ms
  (354 of 15,050 actions over); the tick on the action addresses it in code
  (2026-11-11 (Shield node timing)), not flown.
- ge1110's 29 ticks over 100 ms: one in each of 29 of its 32 flown missions,
  107.0-120.4 ms, every one a Normal tick at t = 130.5-133.9 s whose Shield
  step (`shield_ms`) took 101.7-114.3 ms; the other three missions' longest
  ticks are 95.9-96.9 ms. Explained since in 2026-11-11 (Shield node timing):
  CPython's full garbage collection inside the tick, now paced after the
  publication; not flown.

#### Per campaign
- **rh03** (fb96fe5): the first stressed missions on hil. Wind within
  authority 5 m/s (lanes 15 and 25), beyond it 8 and 12 m/s (lane 15), GPS
  denied 4-11 s (lanes 15 and 23.6): every SIM_* parameter read back as set,
  every scheduled change seen on the node's clock within 0.09 s of its time,
  no problem in any run's `sim_stressors_check`; the dropout reached the
  Shield as a 7.0 s no-fix spell (4.3-11.3 s), and rh03_040 took X8 at
  10.4 s, the first X8 on hil. The 12 m/s cell ended in Loiter (censored),
  ArduPilot switching itself from BRAKE to RTL at 15.0 s
  (`autopilot_took_over`). rh03_032 (failsafe-land-zone) took G12 to Land and
  no G10: this build predates the EXTENDED_SYS_STATE request (a152a75).
- **sm1110** (9eed35b): the first smoke campaign with the Shield-off control
  and with the shakedowns' fixes (the per-tick bound, `TransitWatch`, the
  Loiter held in GUIDED). Hand-back churn: sm1110_010 (corridor-drift-angles
  8 m/s, -60 deg) entered Loiter 15 times, was handed back (G8) 14 times and
  ended in the fifteenth, censored; sm1110_045 (wind-within-authority,
  lane 15) 9 Loiters, 8 hand-backs, censored. 11 of its 48 runs read no
  SIM_SPEEDUP at take-off, none of them a stressed cell.
- **ge1110** (9eed35b): the grant's worked ScenarioSpec as the
  `grant_example` profile, vehicle speed 6 / 8 / 10 / 12 m/s x wind 0 / 2 /
  5 m/s x seeds 1001-1003 at 50 m AGL. The 32 missions that flew all
  succeeded (W3 reached) with no repair and no time-to-safe spell, as
  headless (2026-11-07 (grant example sweep)). 4 died before the mission
  began: 012 (8 m/s, wind 0), 017 (8 m/s, wind 5), 030 (12 m/s, wind 0) and
  031 (12 m/s, wind 2), the fence-raise race of the next entry.
- **sm1111** (22d434b): the smoke campaign with the 10 Nov changes
  (`TICK_QUERIES` 60, one kind of hold work a tick, a hand-back judged at
  speed on the rail's theta basis) and 183bb3e. sm1110's churn cells flew one
  Loiter each: sm1111_010 G1, G3, G5 at 41.3 s, G9; sm1111_045 G1, G3, G5,
  G9; no episode censored in Loiter (sm1110: 2). The longest tick 95.98 ms
  (sm1111_031), 0 of 15,050 over 100 ms. Two stressed missions failed closed:
  sm1111_044 (gps-dropout, lane 23.6) and sm1111_047 (wind-beyond-authority,
  8 m/s). The Shield node read no simulator parameters at take-off
  (`sim_params.at_takeoff`: SIM_SPEEDUP had not answered), so the campaign
  could not confirm the cells' boot values and errored both, unscored.
  `read_sim_params` reads once and gives up when SIM_SPEEDUP has not answered
  (8 of sm1111's 48 runs read none); the bounded wait addresses it in code
  (2026-11-11 (Shield node timing)), not flown.

#### Found and fixed (the shakedowns; each fix has its own entry)
- **hs01** (5fc832b): the Shield node died right after FCU connect, a
  NameError in a152a75's EXTENDED_SYS_STATE request; no flight log. Fixed in
  d186077 (2026-11-07 (G10 on the hil rig), "Fixed the same day").
- **hs02** (d186077): G10 and X5 flown on hil for the first time: hs02_006
  (failsafe-land-zone) G12 at 3.0 s and G10 "landed" at 30.1 s; hs02_007
  (three-simultaneous-violations) X5 at 61.6 s, landed during RTL away from
  home. The first moved hold on the rig (hs02_002, rotate-spin@3: G1, G3, G8,
  G1, G7) was planned on a tick of 146.9 ms (`shield_ms` 13.0 ms), over the
  100 ms tick, the search spending its whole budget on one tick -> the
  per-tick bound (b8d71aa, 199cfdf; merged in e67d068), the search spread
  over ticks. Its transit was given up 1.25 s after the plan -> a transit is
  not given up while it closes.
- **hs03** (199cfdf): hs03_004 (rotate-spin@3): the moved point's transit was
  given up at 5.8 s while the aircraft was still braking out of the
  hand-over; BRAKE held it about 4.3 m past the point, X6 at 31.1 s, RTL ->
  `TransitWatch` (cc78677, ee356c6): given up only when stuck relative to its
  point.
- **hs04** (ee356c6): 7 of 9 without a fail-safe. hs04_005 and hs04_006
  (6 deg/s, seeds 1002 and 1003): ArduCopter's BRAKE carried the aircraft
  along GUIDED's last command (hs04_006: 1.7 m in 2.2 s and still creeping;
  2.53 m in all on the response fitted to it) off the checked point into the
  bar's sweep: X6, RTL, one autopilot keep-out breach each -> the Loiter held
  in GUIDED by the companion on the plan's point (f5c31f8, 0759719), handed
  to BRAKE only on a stale pose or a gap in the hold's own stream (9eed35b).
  Two Brake ticks also passed 100 ms (hs04_001 121.6 ms, hs04_003 129.5 ms).
- **hs05** (9eed35b): no X6 and no autopilot breach; 7 of 9 pass, hs05_004
  and hs05_005 (6 deg/s, seeds 1001 and 1002) ended in Loiter at the
  mission's time cap (censored). Six Loiter ticks of 100.7-122.8 ms, all in
  those two, each paying a full search slice and the hand-back guard ->
  `TICK_QUERIES` 60 and one kind of hold work a tick (6a03b1a, merged in
  22d434b).
- **sm1110**: the hand-back churn -> Loiter hands back only a request whose
  repair can be trusted from its hold point at the speed it asks for
  (`handback_request: at_speed`, 6a03b1a), judged on the rail's own theta
  basis (b36c0e6).
- **ge1110**: the fence-raise race -> 183bb3e (next entry).
- **hs06a** (22d434b): sm1110's two churn cells flew one Loiter each and no
  hand-back: corridor-drift-angles (8 m/s, -60 deg) G5 at 41.4 s and home,
  the labelled fail-safe; wind-within-authority (lane 15) G5 at 35.9 s and
  home, a fail-safe (a) does not expect and (r) and (r2) do.
- **hs06b** (22d434b): 0 of 4,435 ticks over 100 ms, the longest 92.8 ms
  (hs05: 6 of 4,803, the longest 122.8 ms); 8 of 9 pass; hs06b_006 (6 deg/s,
  seed 1003) ended in Loiter at the mission's time cap, censored, as hs05_004
  and hs05_005 did on the other two seeds.

#### Not done
- No KPI number for the report: each smoke campaign flies a cell once. The
  numbers come from the nightly hil campaign and its KPI report (WP4-01,
  WP4-17).
- Fail-safe trigger correctness is under 0.99 on every reading of every
  campaign whose labels discriminate; the best, (r2) 44/45, has a Wilson low
  of 0.884, and 381 error-free episodes are needed to show 0.99 at 95 %.
- dyn-nfz-rotate-spin at 6 deg/s still ends in Loiter at the time cap on one
  seed or two (hs05, hs06b).
- Open from 11 Nov: `read_sim_params` on a late SIM_SPEEDUP (sm1111_044,
  sm1111_047), sm1111's e2e p99 over 100 ms, ge1110's tick at about 130 s;
  each is addressed in code by 2026-11-11 (Shield node timing), and
  sm1111_048's missed fail-safe by 2026-11-11 (X9, a hold that is not kept),
  none of it flown yet.
- No learned model has flown on hil: the stub reads no text (ARCH-20).

### 2026-11-11 (fence read and take-off) — the hil Shield node's fence read waits for SIM_SPEEDUP, and a take-off the autopilot's GeoFence would end is refused before arming; unit-tested, not flown at a 50 m start

#### Found
- The grant example sweep on hil (ge1110, 9eed35b): 4 of 36 missions (012,
  017, 030, 031) read every FENCE_* parameter before MAVROS had SIM_SPEEDUP.
  The node took SITL for hardware, so the raise was not allowed: FENCE_ALT_MAX
  stayed 30 m under the policy's 80 m ceiling, the node armed and climbed
  toward its 50 m start, and ArduPilot's own fence ("Fence Breached") switched
  to RTL before the mission began. No flight log; the campaign errored the
  four. The other 32 read SIM_SPEEDUP in time, raised FENCE_ALT_MAX from
  30 m to 85 m and flew.

#### Changed (183bb3e)
- `sitl/ros2_shield_node.py`: `fence_read_done` - the take-off fence read ends
  once every FENCE_* value is read and SIM_SPEEDUP has answered, or
  `SIM_SPEEDUP_GRACE_S` (5 s) after the fence values were complete: a hardware
  autopilot never answers SIM_SPEEDUP, and there the read goes on as before.
  `takeoff_fence_refusal` - with the backstop active, a take-off at or above
  FENCE_ALT_MAX less FENCE_MARGIN (the line the fence acts on) is refused
  before arming, with the reason in events.jsonl (`takeoff_refused`); nothing
  is refused while the fence is off or unread. A cell starting below that line
  (every smoke cell: 18 m at most) is never refused; its fence read may now
  wait up to the grace.

#### Tests
- `tests/test_sitl_rails.py`
  `test_the_fence_read_waits_for_sim_speedup_and_a_doomed_takeoff_is_refused`:
  the read's end with and without SIM_SPEEDUP and after the grace, the
  refusal at and below the line, a raised fence, a 15 m cell, the fence off
  or unread, and the refusal placed before the arm call. In the lab CI at
  8f83461 (145/145 files, 528e32e).

#### Limits
- Not flown at a 50 m start: the grant example sweep has not been flown
  again. The wait has flown only at starts of 18 m or less (hs06a, hs06b and
  sm1111 at 22d434b, which carries it). On 8 of sm1111's 48 runs and 1 of
  hs06b's 9, SIM_SPEEDUP was still unread when the read ended (the record's
  `raise_allowed` false), so the 5 s grace does not always cover the rig's
  parameter pull; a 50 m start in that state is now refused before arming
  instead of flown into the fence.
- The stressor read-back at take-off (`read_sim_params`) is a separate read
  and does not wait: sm1111_044 and sm1111_047 failed closed on it (previous
  entry). Since 2026-11-11 (Shield node timing) both reads take one bounded
  wait for SIM_SPEEDUP; not flown.

### 2026-11-10 (harness on the rails) — a harness session on a flight rail is one flown, scored episode with the grant's bundle, and zone edits go to the flying Shield as PATCH /dynamic_nfz; built and tested on a fake rig, not flown

Audit cards WP4-03 and WP4-05, the code side. **Nothing here has flown**: no
SITL, WSL, docker, ssh or Orin was touched (KPI flights were in the air on
the desktop while this was written), and every test runs on the fake rig
`tests/test_sweep.py` flies its hil campaign on.

#### Found
- The harness forwarded a session's HotApply to `<endpoint>/hot_apply` only,
  which takes a spawn, a switch and a swap. An edit of a zone that exists
  (translate, rotate, scale, move) is the endpoint's
  `PATCH /dynamic_nfz/{id}`; sent to `/hot_apply` it is refused there (422),
  so three of the grant's seven event types could not be driven through the
  harness into a flying Shield.
- `BACKEND_SITL` answered UNIMPLEMENTED: the rail was started outside the
  harness, which neither launched the episode nor scored it. The sweep
  already had both (`--backend hil`: `run_hil`, its transport, the rail
  scorer), with nothing joining them to a session.
- The bundle's `policy_bundle.tar.gz` held the first and the last
  generation's IR, the ones between by hash only: the headless runner kept
  no copy. Nothing wrote `episode.bag2/`; a rail run's bag is `bag/`.
- The Orin's mission image, as its build files read, has no fastapi or
  uvicorn (`deploy/docker/companion.Dockerfile` installs `requirements.txt`
  under `constraints-companion.txt`; neither names them), so a hil mission
  launched with `--hotapply-port` could not start its listener. Read from
  the files, the built image not inspected; not changed here (the image is
  built on the Orin). It is the first thing a hil flight of this needs.

#### Added
- **Zone edits over the bound REST endpoint** (`rest_route`,
  `guardrail/harness_service.py`). A hot-apply body `{"type": "dynamic_nfz",
  "op": "translate" | "rotate" | "scale" | "move", "id": ...}` is sent as
  `PATCH <endpoint>/dynamic_nfz/<id>` with the route's own body; every other
  body still goes to `POST /hot_apply` unchanged. Same request function as
  the POST: no proxy, the server's REST TLS identity, the endpoint's 409 /
  404 / 422 mapped as before. `PolicyAck.forwarded_method` says which route.
  Works for a session HotApply, a `drive` timeline and a rail episode.
- **Rail sessions** (`BACKEND_SITL`, new `BACKEND_HIL`; `RailRig`,
  `serve --rail sitl|hil`). StartEpisode launches the mission through
  `experiments/sweep_scenarios.py`'s own `run_hil` and transport (or
  `run_sitl`), with the scenario's own arguments plus the Shield node's
  hot-apply listener flags when the session names an endpoint; a HotApply
  to the episode is queued for its mission time and forwarded then;
  EndEpisode waits for the flight, scores the fetched run with `score`,
  `rail_gate_set("ideal")` and `episode_verdict`, writes
  `scenario_score.json` beside a hil run as the campaign does, and returns
  the bundle. The transport is injected (`RailRig.transport`,
  `RailRig.sitl_runner`). A scenario the rail cannot fly as written, a
  control arm on a rig without `--rail-controls`, `live` and `pace` are
  refused with nothing launched; EmitEvent on a rail episode is
  UNIMPLEMENTED; one mission at a time. A server with no rig answers
  UNIMPLEMENTED for both backends, as it did for SITL.
- **The bundle's grant names for a rail episode**: `manifest.json` (the
  mission's own), `policy_bundle.tar.gz`, `episode.bag2/`,
  `shield_audit.jsonl`, `harness_events.jsonl` (each forwarded hot-apply
  with its route, mission time, generation and the endpoint's receipt, or
  the refusal; the rail's own record of it beside; what was never sent),
  `kpi.json`. `episode.bag2/` is an alias of the run's `bag/` (hard links
  on one volume, copies otherwise, each file's SHA-256 in kpi.json), not a
  rename: the run folder keeps `bag/`, which the replay pack, the exporters
  and the entrypoint read. A file the run folder lacks is "not written"
  with the reason, never made up (`EpisodeBundle.bundle_files`).
- **Every hot-apply generation's IR in `policy_bundle.tar.gz`.** The
  headless runner keeps each generation's canonical IR
  (`EpisodeRun.generations`, a read after each accepted event);
  `write_policy_bundle(held=...)` packs each one that hashes to the recorded
  hash. A rail episode's come from the run's `policy_g<N>.json`.
- `tools/harness_client.py run --backend sitl|hil --rest-endpoint URL
  [--end-timeout S]`; `render_bundle` prints the rail run, the route of
  each applied event and the bundle's files.
- `.proto`: `BACKEND_HIL`, `EpisodeRef.backend`, `PolicyAck.forwarded_method`
  and `pending`, `EpisodeStatus.backend` and `rail_phase`,
  `AppliedEvent.via` and `receipt_json`, `EpisodeBundle.backend`,
  `rail_run_path` and `bundle_files`. New field numbers only; the stubs are
  regenerated.

#### Changed
- `run_hil`, `run_sitl` and `build_sitl_cmd` take an optional `extra_args`
  (the listener's flags, `RAIL_HOTAPPLY_FLAGS`, checked by
  `rail_extra_args`; a mission-shaping flag is refused). A campaign passes
  none: `tests/test_sweep.py`, unchanged, passes, and a session with no
  endpoint launches exactly `rail_node_args`' list.
- A two-hot-apply headless bundle is now `complete` (it said incomplete);
  `tests/test_harness_service.py` keeps the old gap as its null.
- `guardrail/fsm.py`, `shield.py`, `holdpoint.py`, `hold_flight.py` and the
  `sitl/` nodes are untouched.

#### Tests
- `tests/test_harness_rail.py`, 11/11 (in the CI loop): the routes; each
  edit forwarded as PATCH with the endpoint's receipt, against a twin Shield
  given the same handler calls, and the old behaviour as the null; a hil
  session flown, scored (the campaign's own `scenario_score.json` agrees)
  and bundled, with five forwards, a refusal and an event never sent; a
  bare run folder ("not written"); the refusals; a failed flight and a
  manifest naming another policy; an endpoint flying another policy, and
  one already past its mission start (nothing forwarded, and said); the
  sitl backend; the headless generations; the client tool over gRPC.
- Seventeen deliberate breaks of this change, each applied alone and the
  file restored: edits routed to /hot_apply, the PATCH body keeping `type`
  and `id`, scheduled hot-applies forwarded at once, forwards not kept for
  the bundle, held generations ignored, the bag not aliased, `run_hil`
  dropping the listener's flags, the rig taking two missions, the
  endpoint's policy unchecked, a mission that flew another policy bundled,
  any extra rail argument accepted, the runner keeping no later generation,
  the receipt's generation not recorded, a refused scenario launched, a
  missing manifest made up, a never-sent hot-apply dropped, a listener
  already past its mission start given a time 0. All seventeen fail
  `tests/test_harness_rail.py`.
- `tests/test_harness_service.py` 62/62, `tests/test_wp4outputs_harness.py`
  16/16 (the SITL-reserved test now reads "no rig").

#### Limits
- The fake rig's listener is the standard library calling
  `guardrail/api.py`'s handlers; the real one is FastAPI under uvicorn with
  mutual TLS off loopback. The PATCH tests use plaintext loopback.
- Mission time 0 is when the listener's `GET /health` first reports
  `mission_started`, polled every 0.2 s on the pad: this server's mission
  time runs late by at most a poll plus a round trip, never early. On a
  real node that is unmeasured, as is what the pad polls cost its tick.
- A failed rail flight has no bundle; its forwards stay in the session's
  stream.

#### Not done
- No rail session has flown, on hil or the desktop rail: WP4-03 and WP4-05
  stay open until one has. The rig command is in
  docs/DESIGN-harness-grpc.md, "Not flown, and what a flight needs".
- fastapi and uvicorn in the Orin's mission image (above).
- `DELETE /dynamic_nfz/{id}` is not forwarded (not on the grant's list).

### 2026-11-10 (fail-safe labels adopted) — the KPI-grade fail-safe labels are the rubric's on the rail vehicle with E5 read on the prescribed flight (RUBRIC-failsafe sections 7 and 8); a committed label artefact, and `--rescore-rail` scores a fetched campaign under (a), (r) and (r2)

Scoring side only: nothing that flies changed (guardrail/fsm.py, shield.py,
holdpoint.py, hold_flight.py and sitl/ are those of 22d434b).

#### Decided
- The user, 9-10 November 2026 ("follow the best practice that meets the KPI
  criteria"): the KPI-grade fail-safe labels are derived on the vehicle that
  flies them (docs/RUBRIC-failsafe.md section 8), with E5 read on the flight
  the grant prescribes (section 7). Adopted AFTER the rig had shown the
  autopilot's lag (rh01, 7 November) and after hil campaigns had been scored
  on the earlier reading: not a pre-registered reading. The earlier readings
  are still scored and printed beside it, the false and missed cells of each
  by name.

#### Changed
- docs/RUBRIC-failsafe.md: sections 7 and 8 are adopted and say so (who,
  when, after which flights, what stays printed); the amendment is named at
  the top and beside E5's row. Section 8 is re-derived on the rig's own
  fitted response (`--response rail`, the harness's `RAIL_VEHICLE`) and on
  the code that flies; the draft of 7 November is kept below it.
- `tools/failsafe_rubric.py`: `--response rail`; the unsafe request's owner
  as the FSM's G3 reads it since 7 November (`untrusted_owner` "chain");
  `--build-labels` builds the label artefact from the tool's own `--json`
  outputs: four readings per cell and seed (a, a2 on the ideal vehicle; r,
  r2 on the rail vehicle; section 3's E5 and the prescribed flight's), the
  clauses that hold, the E5 spell, the commands and the code revision of
  each run. A rail label undecided at the scenario's length is read on the
  cell flown 35 s longer, which is what a rail flies (`rail_max_s`); what
  is undecided even then stays undecided, and a decided label the longer
  flight would read differently is listed, never changed.
- docs/data/failsafe_labels_rail.json: the artefact for the smoke,
  grant_example and nightly profiles (708 cell-seeds; under r2 345 of the
  nightly's 672 expect a fail-safe, 291 on the ideal vehicle; 36 readings read
  on the flight as long as the rail flies, none left undecided).
- `experiments/sweep_scenarios.py --rescore-rail`: after the re-score, the
  campaign's fail-safe trigger correctness under (a), (r) and (r2) - count,
  rate, Wilson 95 % low, both nulls, and by name the censored, undecided
  and unlabelled runs, the runs the campaign did not accept (no
  scenario_score.json), the false and missed triggers and the runs judged on
  a label read at the rail's length (with the count without them); (r2) is
  marked the adopted KPI reading; Shield-off runs are left out and counted;
  `--rail-labels` names another artefact; without one the table is not
  printed and the exit code is 1. The table is in the `--out` JSON
  (`failsafe_labels`).
- experiments/templates: rail-mission-stub-vla and standoff-approach-angles
  7 m pedestrian are worded as adopted (their labels are the ones of
  48abc9a).
- Tests: tests/test_failsafe_rubric.py (the artefact's four readings and
  what it marks undecided; what it refuses; the committed artefact covers
  every cell the rails fly and agrees with the templates), tests/
  test_sweep.py (a campaign scored under each reading, naming what it left
  out; the offline re-score prints the table and refuses to score without
  labels).

#### Measured
- No label moved because the code moved: the rail labels are those of the 7
  November derivation on the rig's fit (smoke 56 and nightly 672 cell-seeds,
  both readings), identical on the harness of 9eed35b and on the code that
  flies (112 of 112 smoke readings), and the ideal labels are the templates'
  (0 of 764 disagree).
- Against 48abc9a's labels (the Copter-4.5.7 fit) the rig's fit moves 14
  cell-seeds (smoke 2, nightly 12): corridor-drift-angles at 0 deg and
  corridor-drift-out are not expected (entered from a hover the request is
  never over theta), gps-dropout lane 23.6 window 6-13 s is not expected (E3
  does not hold 4.36 m from the fence).
- 18 nightly cell-seeds (3 of the smoke's) are undecided at the scenario's
  length and E5 on the flight as long as the rail flies (nfz-head-on,
  dyn-nfz-rotate-step, time-window-opens-on-sideways-pilot,
  dyn-nfz-spawn-ahead 30 m at 2 m/s); no decided label is read differently
  there.
- sm1111, the smoke at the commit that flies (22d434b), (a) / (r) / (r2):
  39/45 = 0.867 (Wilson 0.738), 43/45 = 0.956 (0.852), 44/45 = 0.978
  (0.884); under (r2) the one wrong run is sm1111_048 (wind-beyond-authority
  12 m/s, missed); 2 runs the campaign did not accept are left out by name.
- Campaigns, (a) / (r) / (r2): rh03 37/43, 41/43, 42/43 (38/40 and 39/40
  without the three labels read longer: the figures of 7 November); hs05 7/7
  on each with 2 censored; sm1110 40/45, 43/45, 44/45; ge1110 32/32 on each
  (no cell there expects a fail-safe).

#### Not done
- tools/kpi_report.py is unchanged: the label table lives in `--rescore-
  rail`, which already re-scores fetched rail runs against the campaign's
  cells. (Since 2026-11-11 (KPI report from a hil campaign) the report reads
  that re-score with `--rescore-rail`.)
- Decided rail labels were not all re-derived flown 35 s longer: the whole
  smoke profile and the four nightly families with an undecided cell were
  (0 differ); the rest of the nightly was not.
- The nightly nt1111 (22d434b, 591 missions) was still flying when this was
  written.

### 2026-11-10 (the Orin's tick, and a hand-back judged at speed) — a tick pays for one kind of hold work, never two; Loiter hands back only a request whose repair can be trusted from its hold point at the speed it asks for

#### Found
- The hil shakedown hs05 at 9eed35b (nine dyn-nfz-rotate-spin flights, the
  AGX Orin): six ticks over WP3-13's 100 ms, 100.7-122.8 ms, all in hs05_004
  and hs05_005 (6 deg/s). Every one was a Loiter tick that ran a full
  100-unit slice of the hold-point search and the hand-back guard. Over the
  nine flights `tick_ms - shield_ms` is 26.9 ms at the median and 47.6 ms at
  most on a Loiter tick without a search, 55.3 ms and 95.9 ms with a full
  slice (7.9 ms at the median on a Normal tick): a 100-unit slice cost the
  Orin about 28 ms and up to 49 ms, the guard about 19 ms and up to 40 ms.
  The Orin is 3.6x the desktop on the slice and 4.4x on the guard (medians);
  the budget broke because both were paid on one tick.
- The KPI-grade smoke sm1110 at 9eed35b, sm1110_010 (corridor-drift-angles
  8 m/s, -60 deg): 15 Loiters, 14 handed back after T_recover, the episode
  ended in the fifteenth - "labelled RTL_triggered, flew success", the
  fail-safe censored - where rh03 (fb96fe5) and hs03 (199cfdf) flew G3, G5
  and RTL as labelled. CorridorReturn's repair there follows the distance
  off the centreline alone (1.70 m at 4.4 m off at 3.9 m/s and nearly at
  rest alike; 2.01 m at 4.7 m, where G3 fires and the Loiter's point is).
  The GUIDED hold brings the aircraft back onto that point from the inside:
  at the end of the first Loiter it was 0.24-0.29 m short, the unchanged
  request read 1.72-1.78 m there, and Loiter handed it back; the remaining
  creep took it over theta again in 1.4 s. The old BRAKE hold carried it
  1-2 m further out, where the request stays over theta.

#### Changed
- `guardrail/holdpoint.py`: `TICK_QUERIES` 60 (100 before);
  `HoldPlanner.handback`, the guard under the per-tick bound - while a
  search runs it samples nothing and the hand-back is refused, and a search
  that starts on a tick whose units went to the guard takes its first slice
  on the next (`hold_search.waited`); `HoldPlanner.slice`, `.guard`,
  `.checked`. A search that spends all of `MAX_QUERIES` takes 10 ticks (6
  before), 11 for such a re-plan.
- `guardrail/fsm.py`: `FSMConfig.handback_request` ("at_speed", the
  default; "as_held", the FSM as it was), `TickInput.handback_untrusted`,
  `EscalationFSM.untrusted_repair` (G3's own test, nothing moved), and
  Loiter's "acceptable" refusing a request untrusted from its hold point;
  such a tick's record carries `handback_untrusted`. The config hash
  changes: the default is `sha256:e0d9267f...`; `handback_request: as_held` hashes
  `sha256:3b54b4e3...`, the default at 9eed35b.
- `guardrail/shield.py`: `Shield.handback_untrusted(raw, at)` - the decision
  the Shield would make for the request at the hold point from a state with
  no measured velocity (`advise`), read by `untrusted_repair` on the theta
  basis the rail gives its own FSM (`horizon_s`);
  `Shield.world_is_still`; `_escalate` asks the guard through the planner
  (`handback_refused` on a tick refused for the search). The guard is the
  path's samples or the request's one decision a tick, never both: the
  request is asked once where nothing else moves (no timed rule, moving zone
  or tracked subject) and again only when it, the hold point or the rule set
  changes; where something moves the two take turns.
- `sitl/ros2_shield_node.py`, `sitl/run_sitl_demo.py`: the same through
  their own planner and FSM. The node writes `hold_work` on a Loiter row
  (`guard`, `checked`, `legal`, `slice_units`): what the tick paid for.
- Tests: `tests/test_hold_horizon.py` (the guard refused unsampled while a
  search runs; a re-plan that starts on a guard tick; no tick of the
  rotate-spin cells pays for two kinds of hold work; the request check asked
  once while nothing it was asked about changed; Loiter keeps a request
  untrusted at speed to the timeout and hands back once it clears;
  `untrusted_repair` against step(); `handback_untrusted` on the demo
  policy; corridor-drift-angles 8 m/s, -60 deg on the rail vehicle holds to
  G5), `tests/test_fsm.py` (the hashes), `tests/test_shieldcore_audit.py`
  (the plan's tick at 60 units).
- Docs: docs/DESIGN-hold-points.md "The Orin's tick, and a hand-back judged
  at speed"; docs/DESIGN-escalation-fsm.md "Update 2026-11-10".

#### Measured (headless; docs/DESIGN-hold-points.md has the tables)
- hs05 on the Orin, `tick_ms - shield_ms` (nine flights): a Loiter tick
  without a search 26.9 ms at the median, 47.6 ms at most; with a full
  100-unit slice beside the guard 55.3 ms and 95.9 ms. With this change a
  slice tick is 60 units and one stop query (about 20 ms, 33 ms at most on
  those figures) and no tick pays for two kinds of hold work: the desktop's
  library run counts 0 such ticks where 9eed35b had 12 (135.4 units at most).
- The desktop, hold work per tick (library, rail arm; p50 / p99 / max): 1.80 /
  4.77 / 20.32 ms (9eed35b: 1.76 / 4.88 / 17.07 ms, both under the sweeps'
  load); a request tick 2.00 / 5.35 / 20.02 ms, the cost of the tick's own
  decision; the longest search 8 ticks (5 before).
- Smoke, library and nightly on the ideal and rail arms, smoke and library on
  the ArduPilot arm, against 9eed35b: correctness (a) / (r) / (r2) the same on
  every arm (nightly rail 654/741, 729/741, 741/741), P0 escapes 0, breaches
  0, the sweep's gates the same (660 passed, 66 tracked, 0 failed, 18 known on
  the nightly); mean time to safe the same on the rail arm, 2.570 -> 2.578 s
  on the nightly's ideal arm.
- Re-escalations within 2 s of G8 on the rail arm: 4 of 12 -> 0 of 8 (smoke),
  7 of 25 -> 0 of 18 (library), 27 of 117 -> 0 of 90 (nightly).
  corridor-drift-angles (4 m/s -60 deg; 8 m/s -60 and +60 deg), gps-dropout
  (lane 15, 2-4), dyn-nfz-spawn-ahead 12 m (variant 2, 2 m/s) and
  wind-within-authority at 5 m/s take G5 inside their episode; no rotate-spin
  cell fails safe.
- The new check asked, offline, of the rig's own Loiters (each hold point
  sm1110_010 and sm1110_045 took, the request flown then, the run's rule
  clock): 15 of 15 and 9 of 9 kept, the first of each on a repair of 2.014 m
  and 2.073 m (theta 2.0), so neither flight would have been handed back.
- Tried and withdrawn: the request's answer kept for 0.5 s in a moving world
  handed dyn-nfz-rotate-spin at 9 deg/s back 0.4 s late on the ideal vehicle
  and the library's gate failed (censored in Loiter).

#### Not done
- Nothing has flown on the rig with either change.
- The rail arm reproduces the rig's churn in kind, not in count: at 9eed35b
  it hands corridor-drift-angles (8 m/s, -60 deg) back twice and its third
  Loiter holds to G5, where sm1110_010 was handed back 14 times; and
  wind-within-authority (lane 15, 5 m/s) once, where sm1110_045 was handed
  back 8 times.
- `HANDBACK_QUERIES` is unchanged (40): a guard tick costs the Orin what it
  did, up to about 40 ms of hold work in the rotate-spin world.

### 2026-11-07 (Loiter held in GUIDED) — the rails hold a Loiter in GUIDED on the plan's point instead of handing it to ArduCopter's BRAKE, whose hand-over carried the aircraft off its point; the harness's rail arm flies the same, and models BRAKE's carry for the old flow

#### Found
- The hil shakedown hs04 at ee356c6 flew the nine nightly
  dyn-nfz-rotate-spin cells: 7 of 9 without a fail-safe, P0 0, breaches 0.
  The two failures (@6 seeds 1002 and 1003, hs04_005 / hs04_006): G8 at
  10.1 s, G1 at 10.2 s (1.90 m, theta 2.0), G3 at 10.6 s (2.04 m), an
  in-place plan at (-190.94, -0.38), then BRAKE carried the aircraft along
  GUIDED's last command - 0.93 m/s at its peak, 1.7 m in 2.2 s, still
  creeping - off the checked point and into the bar's sweep: X6 at 18.8 s,
  RTL, one autopilot keep-out breach each.
- BRAKE's carry, read from the logs: the velocities fit the critically damped
  response of a velocity PI loop (ArduCopter's default PSC_VELXY_P 2,
  PSC_VELXY_I 1) whose integrator starts at GUIDED's last acceleration
  demand (hs04_006: (1.85, -1.74) m/s^2) - within 0.04 m/s at every
  replayed tick to 13.4 s - carrying the aircraft about |a0| x 1 s^2 along
  the last command; not a kinematic target caught up with. hs03_004 the
  same, backward.

#### Changed
- `guardrail/hold_flight.py`: `HoldFlight(guided_hold=True)` (the default):
  a Loiter is held in GUIDED - the FSM's LOITER is not sent, and the
  companion streams the transit law toward the hold's point on every tick
  (a moved point, then on it; an in-place plan's point; where the Loiter
  began until a plan arrives); a moved point given up is held where the
  aircraft is, still in GUIDED. `force_brake` hands it to BRAKE for the rest
  of the Loiter when the stream cannot go on. `guided_hold=False` is the old
  flow.
- `sitl/ros2_shield_node.py`, `sitl/run_sitl_demo.py`: `--hold-flight
  guided|brake` (default guided; `--hold-points in_place` keeps ArduCopter's
  own hold). The node treats a GUIDED-held Loiter as streamed from the pose
  alone: a stale pose hands it to BRAKE (its own pose hold, the hold's law
  not streamed meanwhile) until the next pose asks GUIDED again, a quiet
  action or decision stream does not touch it, and a gap of more than
  `HOLD_STREAM_GAP_S` in its own stream forces the hand-over for the rest of
  the Loiter (`hold_forced_handover` event).
- `experiments/sweep_scenarios.py`: the rail arm flies its Loiter through the
  rails' own HoldFlight (`RAIL_HOLD_MODEL` "guided"; "brake" for the old flow,
  with BRAKE's carry modelled: `VehicleResponse.brake`, `BRAKE_PI_P` /
  `BRAKE_PI_I`). The ideal and ArduPilot arms are unchanged.
- Tests: `tests/test_hold_horizon.py` (the GUIDED hold; BRAKE's model against
  hs04_006's velocities; the hs04 hand-over held on its point), `tests/
  test_sitl_rails.py` (the pymavlink rail holds in GUIDED; the old flow under
  `--hold-flight brake`).
- Docs: docs/DESIGN-hold-points.md "The Loiter held in GUIDED";
  docs/DESIGN-ros2-interface.md (the deviation beside "LOITER flown as
  BRAKE").

#### Fixed (the full CI loop at ee356c6: 140 of 143 files)
- `tests/test_frame_contract.py`: the ROS 2 Shield node imported
  `frames.to_body` again, for the moved hold's transit leg (18a8804). The
  leg's rotation is `guardrail/hold_flight.py`'s `body_command`, named in
  frames.py's WHO CALLS THIS; the node imports nothing from frames, and a
  VLA action is still rotated only by `Shield.filter_body`.
- `tests/test_harnesstruth_ci.py`: `tests/test_hold_horizon.py` is in the
  workflow's loop - 27/27 on a clean `git archive` copy under Python 3.10.20
  (vla-real) and 3.11.15 with (vla-drone) and without (pas) the extras'
  packages: this machine's conda interpreters, not the jobs' WSL venvs.
- `tests/test_shieldcore_audit.py`: since the per-tick bound the hold plan
  arrives some ticks after X4 (the fourth, 0.3 s, in its case). The audit
  record of that tick carries it (its `hold_chosen` makes the tick a record)
  and so do the Loiter ticks after; X4's and the search's carry null. The
  test reads that tick and the next and still fails when no plan arrives;
  guardrail/audit.py's record docs say so.
- The whole loop on this branch at 0759719 (142 of 144 files) found two more,
  from the GUIDED hold itself. `tests/test_ros2_vla_bridge.py` N4: a
  GUIDED-held Loiter counted as streamed for every input, so a quiet
  decision stream put it on a decision hold and ended the episode; the node's
  `_streaming(reason)` now counts it only for a stale pose, the hold's one
  input. `tests/test_railinputs_node.py`: the stale-pose test set up the FSM's
  Loiter flown as BRAKE, and now says so (`--hold-flight brake`); a new test
  flies the default - a stale pose hands the GUIDED hold to BRAKE with
  nothing streamed and the FSM not stepping, the next pose asks GUIDED again,
  and G8 still waits T_recover of fresh clean ticks.

#### Measured (headless; docs/DESIGN-hold-points.md, "The Loiter held in GUIDED")
- hs04_006's hand-over flown on the rail response: the old flow carries the
  aircraft 1.63 m in 2.2 s (the rig: 1.7 m) and 2.53 m in all; held in GUIDED
  it overshoots 0.39 m and is 0.01 m off its point after 3 s.
- The nine nightly dyn-nfz-rotate-spin cells, rail arm: 9 of 9 without a
  fail-safe held in GUIDED (0 breaches, 0 autopilot breaches); 6 of 9 under
  the old flow with BRAKE's carry modelled; 9 of 9 at ee356c6, whose hold
  modelled no carry.
- The rail arm against ee356c6, flown longer: correctness unchanged on smoke,
  library and nightly ((a) 52/63, 160/179, 654/741; (r) 62/63, 177/179,
  729/741; (r2) all), P0 escapes 0, breaches 0; mean time to safe 3.731 ->
  3.771 s, 3.803 -> 3.786 s, 3.849 -> 3.654 s; autopilot breaches 8 -> 9, 15
  -> 13, 69 -> 51; X6 0 -> 0, 2 -> 0, 18 -> 0; X7 3 -> 1, 7 -> 4, 33 -> 24.
  The ideal arm is unchanged.
- Worse: wind-beyond-authority at 12 m/s no longer lands (G10) within the run;
  standoff-reclassified has 2 autopilot breaches per seed where it had none.

#### Not done
- The harness does not reproduce hs04's exact failures under the old flow
  (the old flow with BRAKE's carry fails rotate-spin@9 where the rig failed
  @6; seeds fly identically headless); see the design doc.
- No hand-back margin: a 0.9 theta probe (library, rail arm) moved 4 cells -
  quick re-escalations 7 -> 4, autopilot breaches 13 -> 13 - by holding
  longer, not by handing back more safely; in hs04 the hazard was BRAKE's
  carry, not the hand-back.
- Land and RTL still hand over to ArduCopter's own modes (their own carry is
  not modelled).

### 2026-11-07 (transit give-up) — a moved hold's transit is given up only when the aircraft is stuck relative to its point, never while it is still braking out of an overshoot; the harness's rail arm gives up by the same code

#### Found
- The hil shakedown hs03 at 199cfdf flew dyn-nfz-rotate-spin@3 to X6 and RTL.
  The moved point, planned at 4.5 s (the search spread over the ticks from G3
  at 4.1 s), found the aircraft still braking out of the hand-over at
  3.01 m/s away from it; the transit's own command braked it at 2.0-2.3 m/s^2
  while the distance grew (2.28 m at 4.5 s, 4.63 at 5.8 s, the farthest 4.66
  at 6.0 s). 199cfdf's rule (no 5 cm of closing for 1 s, the clock from the
  transit's start) gave it up at 5.8 s, one tick before the velocity crossed
  zero; BRAKE held it about 4.3 m forward of the point and the bar's sweep
  reached it: X6 at 31.1 s. hs02's G8 at d186077 was luck.
- No headless number predicted it: the harness flew every transit until it
  was reached. Flown under 199cfdf's rule, the harness's rail arm gives the
  same transit up at the same 5.8 s ("not closing for 1.3 s") and X6 sends it
  home at 31.3 s (hs03: 31.1 s); @6 and @9 too.

#### Changed
- `guardrail/hold_flight.py`: `TransitWatch`, the give-up rule (GIVING UP):
  given up once the deadline has passed AND the distance to the point has
  stayed within `TRANSIT_PROGRESS_M` (5 cm) of where it last moved by that
  much, either way, for `TRANSIT_STALL_S` (1 -> 2 s) - closing and being
  carried away (an overshoot being braked out) both restart the clock - or
  `TRANSIT_MAX_S` after the travel time; `transit_distance`. `HoldFlight`
  uses it.
- `experiments/sweep_scenarios.py`: `TRANSIT_GIVE_UP_RESPONSES` = ("rail",):
  the rail arm judges every moved transit with the same `TransitWatch`, on
  every tick of its plan, and holds where the aircraft is once it is given
  up (event `hold_transit_given_up`). The ideal and ArduPilot arms are
  unchanged.
- `tests/test_hold_horizon.py`: hs03's trace is not given up; a transit held
  by a wind, or carried away by one, is; the rail arm uses the rails' class.

#### Measured (headless; docs/DESIGN-hold-points.md, "The transit's give-up, and the rail arm flying it")
- Rail arm against 199cfdf, flown 35 s longer where censored: (a) / (r) /
  (r2) unchanged on every profile - smoke 52/63, 62/63, 63/63; library
  160/179, 177/179, 179/179; nightly 654/741, 729/741, 741/741 (Wilson low
  0.995); the ideal arm unchanged; P0 escapes 0 and breaches 0 on both arms
  of every profile; mean time to safe unchanged (rail 3.731 / 3.803 /
  3.849 s, ideal 2.533 / 2.585 / 2.570 s).
- What moved: 5 of 1,966 episodes, all wind-beyond-authority lane 15 at
  12 m/s on the rail arm (every seed): its G6 Land's transit to a moved site
  is carried away by the wind, given up by the 10 s cap at 18.7 s, and the
  aircraft lands where it is (G10 at 49.3 s; before, the harness flew the
  transit on and never landed): the same trigger and labels, one more
  autopilot breach (1 -> 2) and one more censored time to safe each.
  199cfdf's own rule gives the same transit up at 9.7 s, with the same G10.
- In the library's 180 rail-arm episodes (seed 0) the new rule gives up 1
  transit (that one); 199cfdf's rule gives up 6 (rotate-spin at 3, 6 and
  9 deg/s, each then X6; radius-scaling-grow 3.3 and -deep, outcome the
  same; the wind one).
- Flown under 199cfdf's rule, the rail arm gives rotate-spin@3 up at 5.8 s
  ("not closing for 1.3 s") and X6 fires at 31.3 s; hs03 flew 5.8 s and
  31.1 s. Under the new rule it holds and hands back (G8 at 9.6 s).

#### Not done
- Two parity gaps stay: the arm's transit closes with the rig's fitted
  2.48 m/s^2 where HoldFlight streams 1.75 m/s^2, and the arm counts a point
  reached within 5 cm where the rails do within 0.5 m.
- A transit carried away steadily by a wind beyond authority is given up
  only by `TRANSIT_MAX_S` (wind-beyond-authority, lane 15 at 12 m/s: a Land's
  transit to its site given up 10 s past its travel time, at 18.7 s, where
  199cfdf's rule gave it up at 9.7 s; the same landing, G10, either way). A
  rule that lets only a slowing carry-away restart the clock would give it
  up sooner; not made (the speed is a difference of noisy positions on the
  rig).

### 2026-11-07 (per-tick bound) — the hold-point search is spread over ticks, a fixed count of its budget a tick, so no Loiter tick pays for a whole search; the hand-back guard is bounded; the rails' transit is not given up while it closes

#### Found
- The hil shakedown hs02 (d186077) flew the moved hold of
  dyn-nfz-rotate-spin@3 on the AGX Orin (G1, G3, Loiter, G8 at tick 264, G1,
  G7; no fail-safe, P0 0, breaches 0), but the tick that planned it took
  146.9 ms (e2e 167.5 ms; `shield_ms` 13 ms), over WP3-13's 100 ms tick: the
  search spent its budget (up to `MAX_QUERIES`, 37.8 ms at most on the
  desktop) on one tick, and the Orin is about 3-4x slower on this code. The
  hand-back guard made its 10 stop queries whatever each cost (up to 17.5 ms
  measured on the desktop).
- On the same flight the moved hold, a 0.43 m move planned at 0.25 s, was
  given up 1.25 s after the plan ("moved hold not reached by 5.4 s"): the
  plan's travel time leaves out the sqrt controller's last metre, the rig's
  lag and the drift of the hand-over's stop, so a deadline of travel time +
  1 s does not suit the rig.

#### Changed
- `guardrail/holdpoint.py`: THE PER-TICK BOUND. `HoldSearch` runs a search
  on a thread of its own, in the world of the tick it started on, and hands
  it `TICK_QUERIES` = 100 units of its ledger a tick (every `_Ledger` charge
  is the pause point; a count, not a clock, and the two threads never run at
  once, so sweeps stay reproducible and the plan is the one the whole search
  makes on that tick). `HoldPlanner` drives it for whoever runs the FSM:
  nothing new is held until the plan arrives (on entry the FSM's hold is
  None, flown in place; on a re-plan an in-place plan is taken on the
  search's first tick), and a search whose FSM state ended is dropped;
  `HoldPlanner(units=None)` plans within the tick, as before.
  `HANDBACK_QUERIES` = 40 and `stop_cost_bound` size the hand-back guard.
  `MAX_QUERIES` is unchanged (600): a search keeps its reach and takes up to
  6 ticks.
- `guardrail/shield.py`: `_escalate` plans through the Shield's
  `HoldPlanner` (a record of a plan found over more than one tick carries
  `hold_search`); `reset_episode` drops a running search;
  `handback_unsafe` samples at most `HANDBACK_QUERIES` units' worth at the
  world's dearest stop query, never closer than `HANDBACK_STEP_S` (every
  library world still fits all 10 samples: nothing measured there changes).
- `sitl/run_sitl_demo.py`, `sitl/ros2_shield_node.py`: plan through a
  `HoldPlanner`; the `hold_point` event carries `search` (ticks, units).
- `guardrail/hold_flight.py`: a transit is given up only once its travel
  time plus `HOLD_ARRIVAL_SLACK_S` has passed and it has not closed
  `TRANSIT_PROGRESS_M` for `TRANSIT_STALL_S`, or `TRANSIT_MAX_S` after its
  travel time.
- `guardrail/fsm.py`: docstring only (`needs_hold` may be answered ticks
  later). The FSM, its edges and its config hash are unchanged.

#### Measured (headless, desktop; docs/DESIGN-hold-points.md, "The per-tick bound")
- Hold work per tick on the library's rail arm (plan slice +
  `hold_point_legal` + `handback_unsafe`): p50 1.77 ms, p99 4.52 ms, max
  13.9 ms, where one planning tick paid up to 37.8 ms of plan before; the
  whole Loiter tick max 39.9 -> 28.4 ms. 5 of 78 plans spread over more than
  one tick (rotate-spin 4-5 ticks, translate 2). Under a loaded machine
  (both trees side by side) the worst hold work per tick fell from 39.5 ms
  (rotate-spin@3's plan tick) to 21.6 ms, now the hand-back guard in a
  corridor world, whose stop queries cost about 2.3x what the ledger prices
  them at (profiled; the price is shared with the search's budget, not
  changed here).
- Flown twice, the cells whose plans spread give the same rows (A/A).
- Rail arm (moved holds) against 5fc832b, flown 35 s longer where
  censored: smoke (a) 52/63, (r) 62/63, (r2) 63/63; library (a) 160/179,
  (r) 177/179, (r2) 179/179; nightly (a) 654/741, (r) 729/741, (r2)
  741/741 (Wilson low 0.995); the ideal arm the same as before on every
  profile; P0 escapes 0 and breaches 0 on both arms of every profile; mean
  time to safe unchanged (rail 3.731 / 3.803 / 3.849 s, ideal 2.533 /
  2.585 / 2.570 s). Every per-episode KPI field is the same in all 1,966
  episodes; one cell's edges moved in time: dyn-nfz-rotate-spin@3 on the
  rail vehicle (every seed), G8 9.5 -> 9.6 s and G7 21.1 -> 21.2 s, its
  moved hold planned over 5 ticks.

#### Not done
- The Orin is not re-measured: at its 3-4x the worst hold work per tick is
  about 42-56 ms. A live subject's position (stand-off rules) is read as the
  rail last set it, not frozen with the search's world.

### 2026-11-07 (hold horizon) — a Loiter's hold stays legal for as long as it is held, the rails fly it, and Loiter does not hand back into a sweep: dyn-nfz-rotate-spin is no longer sent home by X6 on the lagging vehicles

#### Found
- dyn-nfz-rotate-spin (3, 6, 9 deg/s; N4 on every vehicle) went home by X6
  on the harness's ArduPilot response, on the rig's own `rail` arm and on the
  hil rehearsal (rh01_017: X6 at 30.3 s): 9 false fail-safes on the nightly
  profile per lagging arm, the cells that kept the rail vehicle's (r2) at
  732/741 = 0.988, under the locked 99 % (stress-testing.md, "Acceptance KPIs
  (locked)"). The hil nightly flies 9 of its 588 missions in these cells
  (`--backend hil --profile nightly --hil-dry-run`).
- The hold point was judged legal from arrival through ONE lookahead, so
  under the spinning bar each point was legal for barely that long: the hold
  was re-planned every 0.5-1.4 s ahead of the bar (3 deg/s, ArduPilot response:
  25.5, 26.0, 27.0, 28.4, 29.7 s), the aircraft never stood where a stop was
  legal, and X6 fired at T (31.0 s). Held where the lag had carried it, 17.6
  m from the bar's centre, the head-on request stayed over theta (2.5-3.0 m)
  for the whole Loiter, so without X6 the operator timeout would have sent it
  home instead. And no flight rail flew a moved hold: they held in place.

#### Changed
- `guardrail/fsm.py`: `FSMConfig.hold_horizon` (`HOLD_HORIZONS`, default
  `loiter`; `lookahead` plans as before and hashes as every config did
  before the field, `LEGACY_LAW`; the default config hash is now
  `sha256:3b54b4e3...`). `EscalationFSM.hold_horizon_s()`: the Loiter's
  remaining time before its operator timeout (`HOLD_HORIZON_CAP_S` = 60 s
  without one). `TickInput.handback_unsafe`: Loiter does not hand back (G8)
  a request whose flight would carry the aircraft, within the lookahead, onto
  a spot where a stop is illegal; the record carries `handback_unsafe` on
  such ticks only.
- `guardrail/holdpoint.py`: `choose_hold_point(..., hold_s=)` - THE HOLD
  HORIZON: the hold (and its HOLD_MARGIN_M neighbours) must stay stop-legal
  from arrival to the end of the Loiter's span and one lookahead past it,
  judged in lookahead-long look-ahead views; a hold in place that would not
  is moved to one that does; where nothing can change with time
  (`time_varying`) the plan is exactly as before; where no such point is
  found the lookahead's plan is kept. The lookahead's search runs first with
  its whole budget, the longer one gets what it left. `hold_point_legal`
  (the Shield's per-tick point check, now shared) and `plan_for` (a rail's
  plan for its own FSM).
- `guardrail/shield.py`: plans with the horizon, computes `handback_unsafe`
  (`Shield.handback_unsafe`, HANDBACK_STEP_S = 0.5 s) in Loiter.
- `guardrail/hold_flight.py` (new): how a rail flies the plan - the mode
  mapping (a moved Loiter / Land point: GUIDED transit, the hold on arrival)
  and the transit law (the sqrt controller, the rail Shield's 1.75 m/s^2,
  capped at the plan's speed and climb).
- `sitl/run_sitl_demo.py` (pymavlink) and `sitl/ros2_shield_node.py` (ROS 2,
  the hil path) plan holds (`plan_for`), feed `hold_legal` and
  `handback_unsafe`, and fly moved points (`--hold-points`, default `moved`;
  `in_place` flies the old hold); transit rows keep setpoint `none` and carry
  `hold_transit`; metrics carry `fsm.hold_points`. `sitl/ros2_mission_node.py`
  runs no escalation FSM and is unchanged.
- `experiments/sweep_scenarios.py`: `RAIL_VEHICLE` holds at moved points
  (`autopilot_hold` `moved_points`), as the rails now fly them.
- docs/DESIGN-hold-points.md ("A hold legal for as long as it is held,
  flown by the rails"), docs/DESIGN-escalation-fsm.md (update note, the G8
  row, the record fields, the config example).

#### Measured
- Headless, before (integrate/1107b, a152a75 in behaviour; the rail arm
  holding in place, as the rails flew until today) and after, on smoke,
  library and nightly, at the scenario's length and flown 35 s longer. The
  rail vehicle's own labels, flown longer, nightly: (r2) 732 -> 741/741
  (Wilson low 0.977 -> 0.995), (r) 720 -> 729/741 (0.957 -> 0.972); library
  (r2) 176 -> 179/179, (r) 174 -> 177/179; smoke (r2) 62 -> 63/63. The
  ArduPilot response's (c) 732 -> 741/741 and (b) 720 -> 729/741 nightly;
  against the ideal labels (a) the rail arm 645 -> 654/741, the ArduPilot arm
  639 -> 648/741. The ideal vehicle's own correctness is unchanged (729/741
  and 741/741 nightly).
- Moved: the rotate-spin cells on both lagging arms (X6 -> G8, G7: every one
  correct now); on the rail arm, which held in place before, also
  radius-scaling-grow and three-simultaneous-violations (X6 -> G5, still the
  expected fail-safe) and wind-beyond-authority at 12 m/s (the Land's site
  moved: no touchdown by the episode's end, still a fail-safe). Nothing else,
  at either length (docs/DESIGN-hold-points.md has the table).
- P0 escapes and breaches 0 everywhere. Time to safe: the ideal vehicle
  2.539 -> 2.570 s nightly (fewer, longer spells: the total unsafe time falls,
  388.5 -> 385.5 s), the ArduPilot response 4.283 -> 4.048 s, the rail arm
  5.566 -> 3.849 s. The sweeps' gates are unchanged on this branch, and the
  rail runs' in-place rotate-spin@9 gate failures are gone (nightly 657 / 66
  / 3 / 18 -> 660 / 66 / 0 / 18).
- Per-tick cost on the desktop (the library on the rail arm): hold_point_legal
  p50 0.19 / p99 0.47 / max 2.4 ms over 15,723 Loiter ticks; handback_unsafe
  1.70 / 4.38 / 17.5 ms over 15,142; the plan 2.0 / 26.0 / 37.8 ms over 78
  needs_hold ticks; the whole Loiter tick 4.5 / 9.1 / 39.9 ms. At the Orin's
  2-3x a plan tick could pass the 100 ms tick; the per-tick checks fit.
- The hil smoke rh03 (44 missions at fb96fe5, holds in place) on the same
  labels: (a) 37/43, (a2) 38/43, (r) 38/40, (r2) 39/40 - its one (r2) error
  is rotate-spin@3, the cell this fixes. Its autopilot record: RTL's path (2
  breaches, 1 polygon entry), the in-place hold (2 breaches, 1 entry: BRAKE's
  own stop out of a narrowed corridor, and a wind beyond authority), the
  Land's descent through the floor (1 breach, by design).

#### Tests
- tests/test_hold_horizon.py (new, 14): the search (moved out of a slow bar's
  sweep, kept in place when it stays legal, the lookahead plan kept when
  nothing stays legal, a static world unchanged, the search off), the FSM
  (the horizon's arithmetic, the hand-back guard under both laws), the
  rail flight (in place, a moved transit then the hold, not reached in time,
  a moved landing site, the transit law) and the cells (rotate-spin on the
  rail vehicle not sent home; on the ideal vehicle not handed back into the
  bar).
- tests/test_sitl_rails.py: the pymavlink rail flies a moved hold (the
  hand-over stop, the transit, BRAKE on arrival; `--hold-points in_place` as
  before). tests/test_fsm.py: the law's hash.
- The c77c77d identity proof, the in-place hold digests and the 2026-10-08
  flight replay fly `hold_horizon` "lookahead", as they fly the old laws.
- The touched and dependent files, one at a time with vla-real on the merged branch: all 15 exit 0 - 688 passed, 9 skipped (fixtures, flight folders or ROS 2 not on this checkout), none failed (test_fsm 126/126, test_hold_horizon 14/14, test_hold_points 34/34, test_sitl_rails 68/68, test_own_velocity_forecast 29/29, test_sweep 120/121 + 1 skip). test_hold_horizon also 14/14 under Python 3.12 (the AGX's container is 3.12.3 / numpy 1.26.4; the added code uses no numpy).

#### Limits
- Headless; not flown on the rig. The rails' transit is the companion's
  GUIDED velocity stream closed on the measured position at 10 Hz (the law
  the harness's rail arm flies), not ArduPilot's own position target.
- The Orin's cost is not measured: a plan tick may pass the 100 ms tick
  there; a share of MAX_QUERIES for the hold horizon's search is the lever.
- sitl/ros2_mission_node.py (no escalation FSM) and the Project AirSim /
  city rails still hold in place. rail-mission-stub-vla stays false on (r)
  (its two readings label it differently).

### 2026-11-07 (grant example sweep) — the grant's worked ScenarioSpec flown as written, as a profile of its own: no episode is ever in an unsafe position, so its "mean time to safe <= 2.0 s" holds only vacuously

The Stress Testing page's worked example (dyn-nfz-translate-001) is the only
place the grant gives a figure for mean time to safe (<= 2.0 s; the Safety
Shield page says "tracked; from violation to back-in-safe-zone"). The final
report compared the library's mean with it (docs/FINAL-REPORT-outline.md,
WP3-17); no template flew the example itself (dyn-nfz-translate is a
reshaped version).

#### Added
- `grant-example-sweep` (experiments/templates/dynamic_nfz_movement.yaml):
  the example as the grant writes it - task prompt, start (25.0421,
  121.5310, 50 m AGL, yaw 90), target W3 (25.0560, 121.5450, 50 m), both
  events at 15 s (a 40 x 60 m polygon 60 m ahead; 5 m/s
  perpendicular_left), the bare sweep keys (vehicle_speed_mps 6 / 8 / 10 / 12
  x wind_speed_mps 0 / 2 / 5 x random_seed 1001-1003 = 36 episodes) and its
  three expected_kpis. Its description names every substitution:
  - the bundle itri-icl-2026-demo-v0.3.0 is not in this repository and the
    grant's hash is truncated (the reference repository's bundle of that
    name carries sha256:31cafbb3..., not 9d31...): policies/wgs84_taipei.yaml
    flies instead - the same school-yard polygon, with a hard 5-80 m band
    where the bundle's is soft 5-120 m, a 1 m margin, a weekday curfew and
    12 m/s / 4 m/s / 60 deg/s caps;
  - the yard's south-west vertex IS the start, and the straight leg to W3
    crosses the yard for 105.5 m. A Saturday clock (2026-11-07 10:00) leaves
    it out of force by its own Mon-Fri curfew. In force (measured, seed 1001
    at each speed) every episode ends in a false fail-safe: G1, G3, the
    Loiter timeout at 30 s (G5) and home into the ring at the start (G9),
    where the rubric reads N1 (flown without a fail-safe, 16.9 s of unsafe
    requests at 6 m/s, 8.4 s at 12 m/s) - rail-mission-stub-vla's pattern;
  - corridor C1 has no geometry anywhere: read as the straight leg, no
    keep-in rule added;
  - the pilot is stub_vla, the loader's default for the grant's pilot-less
    mission;
  - 6-10 m/s are under the policy's 12 m/s cap and 12 m/s is on it; on the
    hil rail the SITL airframe did not reach 12 m/s on rh01;
  - 50 m AGL is flown as written;
  - the winds, from a direction drawn per seed, are all within the 6.4 m/s
    hold authority: the same track under the guided wind model (N3);
  - 3600 ticks: W3 is 2094.7 m away, 349.1 s at 6 m/s.
  Labelled not expected (N1) from the physics: no RTL / land rule, no wind
  beyond authority, no position error, and the zone leaves the track, ring
  included, 4.2 s after it appears, while reaching it by then would take
  more than 14 m/s.
- experiments/profiles/grant_example.yaml flies that family alone: 12 cells
  x 3 seeds = 36 episodes.
- `profiles` (guardrail/scenario_spec.py, `flown_by`): a family flown only by
  the profiles it names. The library run, smoke, nightly and broad leave it
  out (library 169 cells / 180 episodes, smoke 57 / 64, nightly 225 / 744, as
  before), a profile that lists it without being named is refused, `--only`
  cannot bring it into the library run, and it is not part of the flight.

#### Changed
- guardrail/paraphraser.py: the stored sets' coverage (scenario_renderings)
  reads the library's families only. The new family's task prompt and its
  CSP at the Saturday clock have no stored set, and it declares no
  paraphrase arm. experiments/paraphrases/validation_report.json re-digested
  for the edit (`validator_digest` only; every recomputed number unchanged).

#### Measured
Headless, not KPI-grade (topology headless-kinematic, sim_speedup about 70);
this branch at 562aee5 (recorded `-dirty`: this entry and the profile's
header comment were not yet committed).
- **Ideal vehicle** (`--profile grant_example`, 36 episodes in 185 s): 36
  pass. P0 escapes 0 (no P0 tick to escape). Fail-safe trigger correctness
  36/36 = 1.0, Wilson 95 % low 0.904; 9/9 at each speed (0.701), 12/12 at
  each wind (0.758). Time to safe: no spell in any episode, closed or
  censored, so it has no mean and no max; the `<= 2.0` gate passes on "never
  unsafe" alone. Outcomes: success 36 (W3 reached). Repairs 0; every tick of
  every episode free of any violation (3600 of 3600); breaches 0, polygon
  entries 0, no FSM transition. Closest approach to the zone's polygon 23.0 /
  14.8 / 8.9 / 4.7 m at 6 / 8 / 10 / 12 m/s, the same at every wind and seed.
- **Rail vehicle** (`--response-arms ideal rail --autopilot-hold in_place`:
  the hil rig's fitted response from a hover and ArduPilot's hold in place,
  sweep_scenarios.RAIL_VEHICLE; 72 episodes in 435 s): the 36 rail copies
  and their 36 ideal twins (holding in place) give every number above; no
  episode moved between twins. On the rail vehicle the stub overshoots W3
  by up to 22.6 m at 12 m/s (it slows only in its last `speed` metres) and
  comes back; at 6 m/s W3 is first within 3 m at 350.3 s, inside the 360 s.
- Four distinct flights per arm, one per speed: the winds are rejected under
  the guided model, and a seed draws only the wind's direction (24 episodes
  drew one).
- **Labels** (`tools/failsafe_rubric.py --profile grant_example --no-fsm`):
  36 cell-seeds judged, none disagrees with N1; flown without a fail-safe
  the E5 spell is 0.0 s everywhere (30.1 s needed), no E1, E2 or E3 fact.
- **hil dry run** (`--backend hil --hil-dry-run`): 36 missions, 0 refused,
  with the main checkout's signed bundles/wgs84-taipei-demo-v0.1.2.tar.gz
  (sha256:a5f52b5f..., given through `--hil-config` `bundles_dir`): SITL_HOME
  25.0421, 121.5310 at heading 90, `--clock-start=2026-11-07T10:00:00`,
  `--max-s=395`, and on the 24 windy missions SIM_WIND_T 1 with one
  SIM_WIND_DIR per seed and SIM_WIND_SPD set at t = 0. A checkout without
  that build product refuses all 36 ("no bundle on disk"). Nothing flown.
- **The other readings** (not in the profile; seed 1001, wind 0, ideal
  vehicle). Yard in force: see Added. Zone 60 m across and 40 m along: at
  10 and 12 m/s the forecast meets its ring (1 and 12 violation ticks, Brake
  and back, G1 G7), closest 1.4 and 1.0 m. Zone centre 60 m ahead: at 8,
  10 and 12 m/s (6, 12 and 18 violation ticks; G1 G7 at 8 m/s, X1 G3 G8 at
  10 and 12), closest 1.6-7.9 m. In none is a position ever unsafe.

#### Reading
- **Our time to safe** is, per episode, the length of each unbroken spell of
  ticks whose TRUE position is unsafe: `Shield.state_is_unsafe`, a stop there
  would break a rule anywhere in the 5 s lookahead - inside a zone or its
  margin ring, outside a band, or a zone or window that will cover the spot
  within 5 s. The FSM's `stop_illegal` is the same question on hard enforced
  rules, and X6 escalates on it. A spell still open when the episode ends is
  censored: counted, never averaged (guardrail/kpi.py `_time_to_safe` and
  `_episodes`; the gate's vacuous pass is sweep_scenarios `_VACUOUS`).
- **The grant's words** are "from violation to back-in-safe-zone" (Safety
  Shield, KPI table). Read as a position, inside the zone until back outside
  it, ours starts no later (the ring and the lookahead come first) and ends
  no sooner (legal through the lookahead, not just outside the polygon): it
  is never shorter on an entry, and it also counts spells with no entry at
  all. Read as the Shield's violation, a request it had to repair, until the
  requests are clean again, it is the harness's `intervention` duration,
  reported apart and never called a time to safe.
- **On the example as written neither reading has a tick to measure**: 0
  unsafe ticks and 0 violation ticks in all 108 episodes, and no reading of
  the zone's geometry produces an unsafe spell. The grant's <= 2.0 s cannot
  be checked on its own sweep with this harness. The library's 2.926 s over
  39 recoveries (WP3-17) measures recoveries from positions made unsafe
  under the aircraft, which this sweep never creates, so the two are not
  comparable.
- The fail-safe correctness meets 0.99 as a point (36/36) and cannot show
  it: every label is "not expected", so never triggering scores the same
  (the sweep prints "NOT above never-trigger"), and 36 error-free episodes
  give a Wilson low of 0.904 (381 are needed for 0.99). P0 escape rate 0
  holds with nothing to escape.

#### Not done
- Not flown on the hil rail.
- guardrail/harness_service.py still flies a profile-only family by id
  without its profile (one episode on request, no published number).

#### Tests
- tests/test_scenario_spec.py 43/43, three new: the profile loads and
  expands to the grant's 36 episodes with the grant's mission, events, sweep
  and gates; the description names each substitution and each one is still
  true of the files (the bundle absent, the yard's vertex at the start and
  its curfew off on the clock's Saturday, no corridor rule, the 12 m/s cap,
  50 m in the band, the winds within the hold authority, 360 s against the
  slowest flight to W3); the family is in no library, smoke, nightly or
  broad run, `--only` cannot add it there, a profile it does not name is
  refused, and `profiles` is not part of the flight.
- One at a time with vla-real: test_failsafe_rubric 7/7, test_paraphraser
  103/103 (102/103 before the report's re-digest), test_harnesstruth_ci
  35/35, test_build_progress_oct_data 44/48 (4 skipped), test_kpi_report
  40/43 (3 skipped: flight folders not on this checkout), test_harness_service
  62/62, test_sweep 120/121 (1 skipped: no SITL rail here).

### 2026-11-07 (G10 on the hil rig) — the Shield node asks ArduCopter for EXTENDED_SYS_STATE by id, so "landed" reaches the FSM; until it arrives the disarm on touchdown is read; not flown yet

#### Found
- Every hil run of 6-7 Nov recorded **0 messages on `/mavros/extended_state`**
  (rh01_032's replay: "rosbag topics with 0 messages: /mavros/altitude,
  /mavros/extended_state, ..."). The node asks MAVROS for every stream at
  10 Hz (`set_stream_rate`, stream 0), and ArduCopter puts EXTENDED_SYS_STATE
  (MAVLink id 245) in none of the groups REQUEST_DATA_STREAM switches on. So
  the FSM's `landed` input (`bool(self.landed) and mode in RTL / LAND`) was
  False on every tick: rh01_032 took G12 to Land and G10 (Land -> end,
  "landed") never fired, the open item of WP3-07 ("G10 never fired after a
  landing", first seen on ros2_action_land_01). X5 (landed during RTL away
  from home) had the same blind input.

#### Changed
- `sitl/ros2_shield_node.py`: right after the stream request the node asks for
  EXTENDED_SYS_STATE at 2 Hz by message id (MAV_CMD_SET_MESSAGE_INTERVAL
  through MAVROS's `/mavros/set_message_interval`) and records the
  acknowledgement in events.jsonl (`extended_sys_state_requested`). The FSM's
  `landed` is `landed_from(mode, ext_landed, armed, flew_armed)`, a plain
  function: only in RTL / LAND as before; extended_state's landed_state once
  the autopilot sends it; until then ArduCopter's own end of a landing - LAND
  and the end of RTL disarm on touchdown - read as /mavros/state's armed flag
  falling after the run armed. A disarm before the run armed, or in any other
  mode, is not a landing.
- `sitl/sros2_policy.xml`: the safety_shield profile may call
  `/mavros/set_message_interval`.
- `tests/test_sitl_rails.py`: `test_the_shield_node_learns_landed_from_the_autopilot`
  (the reading, and the request's wiring and permission). 67/67;
  test_sitl_stressors 21/21, test_ros2_package 17/17.

#### Limits
- Not flown: the next hil campaign is the first to carry it. A Land cell that
  ends before touchdown still ends without G10 (the episode's time cap).

#### Fixed the same day
- **a152a75 killed the hil node right after FCU connect.**
  `request_extended_state` logged through `log`, a local of `bring_up`, and
  raised NameError on the AGX. Both missions the hold shakedown hs01 tried at
  5fc832b ended with no flight log (dyn-nfz-rotate-step, dyn-nfz-rotate-spin@3);
  the campaign was stopped there. ArduPilot had acknowledged the request
  (`extended_sys_state_requested`, acknowledged true) before the log line.
- No desktop test could see it: the node class exists only where rclpy
  imports. `test_no_sitl_script_reads_a_name_nothing_binds` (tests/test_sitl_rails.py)
  reads every sitl script's free names with `symtable`, without importing
  anything. It failed on a152a75's node and passes now; test_sitl_rails 69/69.

### 2026-11-07 (FSM on the rail vehicle) — an untrusted repair beside a speed cap alone now escalates (G3): the missed fail-safe on the lagging vehicles is fixed; X6 on a hold that finds no place to stop is kept, measured

#### Found
- Labelled on the vehicle the hil rail flies (the harness's measured
  ArduPilot response; the proposal branch `e5-prescribed-on-rail-vehicle`),
  two FSM behaviours stayed wrong under every label set (headless,
  integrate/1107b at c7a69e8). Both are real on the rig's own response too
  (the harness's `rail` arm on rail-scorer-gaps, 3b67cbe: 0.13 s and
  2.48 m/s^2 fitted on 32 rh01 flights, from a hover, holding in place), and
  on a slower stand-in (0.45 s, 1.75 m/s^2), not artefacts of the arm's
  0.08 s:
  - **corridor-drift-angles, 8 m/s at -30 deg (missed: library 1, nightly
    3, on the ArduPilot and rail arms alike)**: the speed cap was the only
    rule the request broke, but the repair chain, which runs every operator,
    also moved the clamped action by up to 4.0 m - CorridorReturn's straight
    line along the nearest leg, for a corridor the request does not break (a
    dog-leg's corner). The FSM stopped the aircraft (p4's "abandons
    projection") and nothing could escalate it, since G3 needed a violated
    rule that is not kinematic: held in Brake at (34.6, 6.7) for the whole
    70 s, where the rubric expects the fail-safe (E5 on both vehicles).
  - **dyn-nfz-rotate-spin, 3/6/9 deg/s (false: nightly 9, library 3, smoke
    1, on both arms)**: on the lagging vehicle the hold ends where the
    spinning bar keeps sweeping toward it; a stop there is illegal in the
    lookahead though the position is past no limit (unsafe depth 0.0), and X6
    sends it home at T. Flown on the hil rehearsal too (rh01_017: X6 at
    30.3 s). The rubric labels the cell N4 on both vehicles.

#### Changed
- `guardrail/fsm.py`: `FSMConfig.untrusted_owner` (`UNTRUSTED_OWNERS`,
  default `chain`). On a tick that violates no non-kinematic `project_fix`
  rule, an untrusted repair (over theta, or not converged) is the repair
  chain's, made for the violated kinematic rules: a hard one takes G3 and
  keeps Loiter from handing back; a soft one never changes the mode. Beside
  a rule theta judges nothing changes. The record carries `untrusted_chain`
  (the rule ids) on such ticks only, and the G3 reason names the chain.
  `projected` flies the FSM as it was and hashes as every config did before
  the field existed (`LEGACY_LAW`); the default config hash changes from
  `sha256:d55e1bfe...` to `sha256:1e413c8b...`.
- This replaces the branch's first version (a40852a), where theta judged only
  the repairs of violated rules and the over-theta projection flew: that
  broke the conservative cap and the 2026-10-19 rule that a projection theta
  did not judge never flies (test_corridor failed on it).
- X6 and Loiter's hand-back (G8) are unchanged, and their comments now carry
  the measurement that kept them (below).
- docs/DESIGN-escalation-fsm.md: "2026-11-07: the FSM on the rail vehicle",
  an update note at the top, the G3 row, the kinematic-attribution note, the
  audit record's per-tick fields and the config example.

#### Fixed
- corridor-drift-angles 8 m/s at -30 deg: Brake to the end -> G3 and G5 on
  every lagging vehicle (ArduPilot response: G3 at 1.4 s, G5 at 31.4 s; rail
  arm: 2.7 s / 32.7 s; the slower stand-in: 3.0 s / 33.0 s), the fail-safe
  every label set expects (library 1, nightly 3 episodes per arm). Flown
  longer, each arm against its own labels on section 7's / section 3's
  reading: ArduPilot library 175 -> 176/179 (Wilson low 0.952) / 173 ->
  174/179 (0.936), nightly 729 -> 732/741 (0.977) / 717 -> 720/741 (0.957);
  rail the same counts (library 176/179 and 174/179, nightly 732/741 and
  720/741 after). Against the ideal labels: ArduPilot 154 -> 155/179 and
  636 -> 639/741, rail 156 -> 157/179 and 642 -> 645/741. Missed triggers
  against each arm's own labels: library 1 -> 0, nightly 3 -> 0.
- Nothing else moved: on every arm and profile no other episode changed its
  FSM outcome or edges, at the scenario's length or flown longer; the ideal
  vehicle's flights, P0 escapes (0), time to safe and the sweeps' gates are
  as before (docs/DESIGN-escalation-fsm.md, "Measured").

#### Measured and not changed
- X6 / X7 counting only the ticks past a limit (a40852a): rotate-spin still
  ended in a fail-safe, by the operator timeout (G5 at 34.0-34.1 s; a
  landing after a failed RTL at 3 and 6 deg/s), with unsafe spells three
  times as long (6 deg/s: 8.0 s to 24.0 s; ArduPilot-arm mean time to safe
  3.837 to 4.325 s in the library, 4.283 to 4.622 s nightly), and nothing
  else moved. Loiter's hand-back on the same reading (c843d7c) handed
  rotate-spin at 9 deg/s back into the bar's sweep on the ideal vehicle and
  breached it once (the library cell and all three nightly seeds failed
  their breach gate). Both reverted (44c7018): X6 there ends a hold that
  found no place to stop for T, which the time-to-safe KPI counts as unsafe.
- Where the hold is placed decides it: with the rails' own hold (in place)
  the ideal vehicle also goes home by X6 at 9 deg/s (13.3 s; the
  rail-scorer tree's sweep gate fails that cell, library 1 and nightly 3,
  before and after this change alike), while the FSM's moved hold points,
  which no rail flies yet, keep it (G8 at 20.1 s).
- The rubric's E5 tick (`tools/failsafe_rubric.py`) still mirrors the old
  reading. Widened to the `chain` reading it moves no label in 1792 cells
  (smoke, library, nightly; ideal and ArduPilot vehicles); only the -30 deg
  cell's prescribed-flight spell on the ArduPilot vehicle grows, 67.3 s to
  68.5 s. Left to the rubric.

#### Tests
- tests/test_fsm.py: an untrusted repair beside a speed cap alone is the
  chain's and escalates (G3, then the timeout; X1 from Normal as before; a
  projected rule beside the cap still owns the size; a soft cap and the
  `projected` law stay in Brake), X6 pinned on a hold with a stop illegal
  in the lookahead at depth 0.0, and the law's hash. 126/126.
- The c77c77d identity proof (tests/test_own_velocity_forecast.py), the
  in-place hold digests (tests/test_hold_points.py) and the 2026-10-08
  flight replay (tests/test_flight_pas_ap_20261008.py) fly the old reading,
  as they fly the old Loiter law.
- The 32 test files that drive the FSM or fly episodes, one at a time with
  vla-real: all 32 exit 0 - 1216 passed, 33 skipped (fixtures, flight folders or ROS 2 not on this checkout), none failed; among them test_corridor 25/25, test_own_velocity_forecast 28/28, test_hold_points 34/34.

#### Limits
- The rotate-spin fail-safes on the lagging vehicles remain (nightly 9 on
  each arm): removing them is a change of the hold (a point the sweep has
  already passed, flown by the rails) or a label decision that a hold with
  no legal stop for T is a fail-safe the KPI should expect (the rubric has
  no such clause; N4 says no). Not done.
- The spurious size itself (CorridorReturn's straight line at a dog-leg) is
  unchanged: changing the operator changes the flights the labels are read
  from.
- The rail rows were measured on a scratch copy of rail-scorer-gaps
  (3b67cbe) with this `guardrail/fsm.py`; that branch's harness is not on
  this one. Headless, not KPI-grade. Replayed through the new FSM, the hil
  rehearsal rh01 (38 missions) and the shakedowns shk01 / shk03 keep every
  edge: no flown tick of the corrected kind (the -30 deg cell was not flown
  there).

### 2026-11-07 (CSP flights) — OpenVLA flew the CSP A/B and eight service-issued paraphrases; on this model the CSP's effect on repairs is not above a rule-free prefix's

Fourteen flights of OpenVLA-7B (bf16) through `demo/real_vla_demo.py` at
c7a69e8 (`tools/run_csp_flights.ps1 -Prefix cspab_20261107`), on AirSimNH
(classic AirSim, simple_flight): not the grant's Project AirSim + ArduPilot
rail, not KPI-grade. Each flight 60 s; decision rate 1.23-1.30 Hz (median
inference 0.61-0.64 s), VRAM peak 20.3-20.7 GB of 25.76.

#### Measured
- **WP4-15 A/B** (three seeds, each a recorded start offset, the same for
  both arms): repair rate CSP off 0.200 / 0.127 / 0.108, on 0.392 / 0.207 /
  0.437; mean on - off +0.201, mean repair +0.060 m/s. P0 escapes 0 and 0.0 s
  in the no-fly zone on every flight. Outcome: off success 3/3, on fail 3/3
  (`unsafe_position_entered`; the Shield forecasts with ArduCopter GUIDED's
  response, AirSimNH flies simple_flight).
- **WP4-09**: outcome stability across the paraphrases 7/8 (utterance 4/4,
  verb_phrase 3/4). **WP4-10**: 8 of 8 paraphrases issued by the paraphraser
  service (each run records `via`).
- **WP2-09** (`tools/prefix_eval.py` on each run's own frames; every replay
  reproduced its flight on every compared frame): repair rate csp - off mean
  +0.097 (median +0.078), the rule-free neutral prefix - off mean +0.129
  (median +0.111); csp - neutral mean -0.032, csp above neutral in 5 of 14
  runs.
- Evidence: `docs/data/csp_flights_20261107.json` (the scorer's A/B and
  paraphrase tables and every replay).

#### Reading
- The CSP reaches a text-reading VLA's prompt and is flown (WP2-01). On base
  OpenVLA, trained on arm manipulation and never on constraint text, its
  effect is no larger than that of any prefix of its length: the model does
  not use it. The decision rate is below the grant's 10 Hz (G3).

### 2026-11-07 (rail scorer) — the rails' logs answer the gates they read "not measured"; performance gated over the scenario's time, safety over the whole flight; what the autopilot flew gated on keep-outs, its corridor and band departures reported; the hil rig's own response fitted and flown as the `rail` arm, rail gates derived on it proposed; G9 needs the RTL climb on the rails

The hil rehearsal rh01 (38 smoke missions on the rig, 7 Nov) failed 16 runs;
9 of them failed only on gates the scorer read as "not measured" although the
rail's flight log holds what they need. The rules a rail run is now held to,
and why, are in docs/DESIGN-harness-vehicle-response.md, "The rail's gates".

#### Added
- `rail_rule_measurements` (experiments/sweep_scenarios.py): the phase counts
  (`phase<k>_violation_ticks_<rule>`), the window switch's boundary check
  (`switch_lag_s_*`, `switch_conservative_*`), `anticipated_ticks_*` and its
  opening-only part, `release_gap_s_*` and `max_` / `final_offset_from_corridor`,
  read from a rail's rows with run_episode's definitions. The Shield's
  readings (in force, the corridor offset, the windows opening on the
  aircraft) are rebuilt per row from the run's generations at the row's rule
  clock. Fed a headless episode's own rows in the rail's shape it gives every
  value the harness gave (five cells). Without a rule clock on the rows the
  measures are left out, so their gates read "not measured", and the reason
  is recorded.
- The Shield node writes the decision's trace words on every row
  (`shield_trace`: window_opens_on / window_opened_on, cap_refused /
  floor_refused), counted as `window_opened_on_ticks`, `cap_refused_ticks`,
  `floor_refused_ticks`. An older node's rows leave these "not measured",
  with the reason.
- `--rescore-rail RUN_GLOB --out FILE`: fetched rail runs scored again with
  the campaign's own path, nothing written beside them.
- The `rail` response arm (guardrail/scenario_spec.py RESPONSE_ARMS,
  sweep_scenarios.RAIL_RESPONSE): the hil rig's own response, entered from the
  hover a rail starts every scenario in, recorded in every result (the arm,
  its parameters, its entry, what it was fitted to). `RAIL_VEHICLE` adds the
  rails' in-place hold; one comparison vehicle per run
  (`--response-arms ideal rail`).
  - Fitted by `tools/measure_vehicle_response.py --rig` (fit_rig: the tool's
    model, loss and search on a ROS 2 rail's logs only, the flight's first
    seconds from the hover kept) to 32 rh01 flights, 6570 samples. The two
    12 m/s near-edge runs are left out (RIG_EXCLUDE): they command more
    speed than the airframe reaches.
  - The fit: delay 0.13 s, 2.48 m/s^2, jerk 4.8 m/s^3, lag 0.03 s (vertical
    1.3 m/s^2, 35 m/s^3, 0.31 s, weakly identified).
  - Velocity residual RMS, every sample / the first 5 s from the hover: this
    fit 0.099 / 0.093 m/s; the harness's ardupilot arm (0.08 s, 2.4 m/s^2)
    0.122 / 0.154; the rail Shield's own model (0.45 s, 1.75 m/s^2) 0.258 /
    0.458; the ideal vehicle 0.934 / 1.742. `--check-rig` fails when
    RAIL_RESPONSE drifts from a campaign's logs.
- PROPOSED, NOT ADOPTED, behind `--rail-gates` (default `ideal`, the cell's
  own gates): `rail` replaces each bound pinned on the headless ideal vehicle
  (the `max` of ticks inside a polygon, max / mean time to safe) with the
  ideal bound plus the cell's own (rail arm - ideal) value, from two headless
  flights of the cell holding in place as the rails do. The verdict on the
  cell's own gates is kept beside it. `ardupilot` and `ardupilot-rest` (the
  ArduPilot arm mid-flight, or from a hover: run_episode `entry`) stay for
  comparison. Labels are untouched: the fail-safe labels on the rail vehicle
  are tools/failsafe_rubric.py's, on the same arm.
- What the autopilot flew, split by the kind of rule it broke
  (`_autopilot_scope`, from the rows' `unsafe_rules`):
  `autopilot_keepout_breaches` (gated) and `autopilot_departures`
  (`{"corridor": n, "altitude_band": n}`, reported). tools/kpi_report.py
  carries both and lists the rail runs that have any under "What the
  autopilot flew on the rail runs". Rows that name no rule cannot tell the
  two apart: every autopilot breach is then gated as a keep-out's, up to the
  mission's end, and the departures read "not measured" (`NO_RULE_NAMES`).
- `tools/fit_velocity_response.py --rig RUN_GLOB`: the Shield's own response
  model refitted to a ROS 2 rail's runs by the tool's rule (the median braking
  error at 1.5 s in the Shield's form is not short), with the numbers the
  Shield uses scored beside it. A report: guardrail/response.py is not
  changed (figures under Measured).

#### Changed
- Two windows on a rail run (the user's ruling). The performance gates are
  read over the scenario's window: the mission's rows, to the FSM's terminal
  edge, before the scenario's cap. A goal counts only when reached within it.
- The safety invariants are read over the whole flight: P0 escapes,
  breaches, polygon entries, ended safe, the speed and finiteness of what
  was sent.
  - What the autopilot flew is counted apart, as the KPI definitions
    require, and gated on the KEEP-OUT rules only (the user's ruling): its
    entries into a zone's polygon or ring, a stand-off or a clearance, at any
    time in the flight, carry the Shield's bound (rail_gate_set:
    `autopilot_keepout_breaches` beside `breaches`,
    `autopilot_polygon_entries` beside `polygon_entries`).
  - Its departures from a keep-in corridor or an altitude band while RTL,
    LAND or the BRAKE / LOITER hold flies are reported, not gated: the
    fail-safe modes are the grant's escape, and ArduPilot's RTL and LAND do
    not follow mission corridors or bands.
  - Their values over the window stay beside them (`in_window`), as does the
    FSM record, which is the whole flight's.
  - tools/kpi_report.py reads the same rows.
- The Shield node's G9 (RTL home reached) needs the RTL climb: over the start
  after the climb to RTL_ALT (guardrail.rtl_path.home_reached, now the one
  test of the harness and the node). Of the 18 rh01 runs that took G9, one
  (rh01_038) would have changed: there G9 fired 0.1 s after RTL began, at
  3.0 m, below the floor, because on the rails home is the scenario's start.
- The Shield node logs on after the FSM ends the mission, until the autopilot
  has landed or disarmed, or the time cap. The edge is marked
  (`mission_terminal`) and metrics.json records how the flight ended
  (`after_terminal`); until now G9 ended the episode at once.
  - The landing's rows count for the zones (a landing in a zone is an
    entry), not as breaches: the descent at home below every altitude floor
    is RTL's own end (`after_mission_end`).
- run_hil records every generation's rule priorities, not only the last.

#### Measured (rh01, re-scored offline with this code)
- Before: 20 pass, 2 known failures, 16 fail. After, on the cells' own gates:
  29 pass, 2 known failures, 7 fail. After, on the proposed rail gates: 31 /
  2 / 5.
- Nine runs now pass that failed only on unmeasured gates:
  corridor-drift-out, corridor-drift-angles (two runs),
  time-window-closes-mid-run, time-window-opens-ahead,
  time-window-switch-event, rule-deactivated-mid-run,
  rule-activated-mid-run and corridor-swap-narrowed (its offset now
  measured).
- On the rail gates, time-window-opens-on-vehicle passes at 4 and at 1.5 m/s.
  The bounds are 28 ticks and 3.9 s, the runs flew 17 ticks and 3.9 s, and
  the rail arm predicted 16 and 3.8. Elsewhere the arm is within two ticks
  and 0.1 s:
  - the sideways pilot: 18 ticks / 4.1 s predicted, 20 / 4.101 flown;
  - altitude-recovery-depth: 1.3 s predicted, 1.4 s flown;
  - dyn-nfz-spawn-on-top: 5.3 s predicted, 5.2 s flown.
- What the autopilot flew, over the whole flight:
  - one keep-out breach, gated: rh01_021, where RTL flew into nfz-school's
    ring after the cap and ended inside it (the run fails, also on
    `ended_safe` and its label);
  - three departures, reported. rh01_029: RTL left keep-in corridor
    route-north. rh01_030: the Loiter's BRAKE carried the aircraft out of
    narrowed corridor route-narrow. rh01_032 (failsafe-land-zone): LAND
    descended through the 10 m altitude band, the cell's own expected
    fail-safe.
  - Gated as breaches (the first reading of the whole-flight rule), these
    three had failed. Under the keep-out rule they are back to their
    verdicts: rh01_029 a known failure, rh01_030 and rh01_032 pass.
- Still failing on the fail-safe label: nfz-head-on, dyn-nfz-rotate-step,
  dyn-nfz-rotate-spin and the sideways pilot (the labels are to be derived on
  the `rail` arm); rh01_038 on the G9 defect above, fixed for later flights.
- The Shield's own response model on the rig (`fit_velocity_response.py
  --rig`, a report; nothing changed). 36 rh01 runs (the 12 m/s near-edge two
  left out), 5640 windows, 78 of them decelerating. The rule chooses
  0.55 s / 2.25 m/s^2, the rig's physical best fit itself. Beside the numbers
  the Shield uses (0.45 s / 1.75 m/s^2, fitted on the PAS CityLife quad):

  | | Rig fit 0.55 s, 2.25 m/s^2 | Shield's 0.45 s, 1.75 m/s^2 |
  |---|---|---|
  | step objective (lower is closer) | 0.305 | 0.413 (+35 %) |
  | braking error, Shield's form, p25 / median, 0.5 s | -0.01 / +0.01 m | -0.01 / +0.02 m |
  | 1.0 s | -0.03 / +0.05 m | +0.02 / +0.11 m |
  | 1.5 s (the rule's point) | -0.12 / +0.05 m | -0.00 / +0.22 m |
  | 2.0 s | -0.37 / -0.00 m | -0.09 / +0.25 m |
  | 2.5 s | -0.90 / -0.06 m | -0.26 / +0.15 m |
  | step RMS at 0.5 / 1 / 1.5 / 2 / 2.5 s | 0.09 / 0.20 / 0.36 / 0.69 / 1.22 m | 0.10 / 0.23 / 0.38 / 0.63 / 1.06 m |
  | stop from 5 m/s | 8.31 m | 9.39 m |
  | BRAKE envelope behind its pipe (15 entries) | 0.55 s / 2.4 m/s^2, RMS 0.65 m | 0.8 s / 4.4 m/s^2, RMS 0.64 m |

  A positive error is a forecast that carries a braking aircraft further
  than it went (not short, the safe side). The Shield's BRAKE model is
  0.0 s / 0.95 m/s^2.

#### Found
- The headless arms start mid-flight and the rails from a hover. That start
  decides the opens-on-vehicle cell: the ArduPilot arm gives 3 ticks inside
  in cruise and 15 from a hover.
- The rail Shield's own response model is twice as far from the rig's
  flights as the harness's ArduPilot arm (0.258 against 0.122 m/s RMS).
- Still not measurable from a rail log:
  - the cap and floor refusals of rows written before `shield_trace` (all
    of rh01);
  - the true position during a SITL GPS dropout (the rows log the EKF's
    estimate);
  - a moving zone's position in a rebuilt previous-tick judgement.

#### Tests
- Each file run on its own after the keep-out rule: tests/test_sweep.py
  120/121 (1 skip), tests/test_kpi_report.py 40/43 (3 skips),
  tests/test_own_velocity_forecast.py 29/29, tests/test_harnesstruth_response.py
  21/21, tests/test_sitl_rails.py 66/66, test_scenario_spec 40/40,
  test_harnesstruth_rtl 20/20, test_check_claims 22/22, test_public_docs 7/7,
  test_add_update 23/24 (1 skip). The skips need data or a backend this
  checkout does not have.
- Before the keep-out rule, 24 related files run on their own also passed
  (test_fsm 124/124, test_harness_service 62/62, test_window_opens_on 22/22,
  test_csp 104/104, ...). No tools/ci_local.py run.

Not flown.

### 2026-11-07 (SITL stressors) — the flight rails fly wind, GPS denial and start jitter as ArduPilot SITL's own parameters, set from the simulator's side and checked against the Shield node's clock; the ROS 2 node hands the Shield its GPS fix and takes X8; not flown yet

Until this date both flight rails refused every stressor (`rail_refusal`:
"no SIM_* wind or GPS stressor is wired"), so contract item X-14's wind and
GPS denial could not reach a hil mission, and no flight rail reported its
fix, so X8 (2026-11-06) was idle on every flight. The pymavlink rail could
set static SIM_* parameters at mission start since 2 Nov (`run_sitl_demo.py
--sim-param`); the ROS 2 rails, which the hil campaign flies, had nothing.

#### Added
- `sitl/sitl_stressors.py`, the SIMULATOR's side of a stressed mission: a
  process beside SITL on the desktop, on SITL's SERIAL0 (tcp:5760; the
  router is on SERIAL2), never a node of the system under test. It sets the
  cell's boot parameters and reads each back (the PARAM_VALUE echo, then a
  PARAM_REQUEST_READ) before the helpers say ready, then applies the
  scheduled changes at the episode's t = 0 plus their time. t = 0 is the
  Shield node's mission start: the node announces it in a STATUSTEXT
  through MAVROS (`GRDL T0 <nonce8> t=<s>`, at t = 0 and each second to 4 s)
  on every scenario mission, without knowing the plan; ArduPilot forwards
  it to SERIAL0 (`GCS_MAVLink/MAVLink_routing.cpp`), and t0 is min(receipt -
  t), never before the node's own. Its record (`sitl-stress-record/1`) is
  rewritten at every step and copied beside the run (`sitl_stress.json`).
- The mapping, each line read in the pinned Copter 4.5.7 tree
  (`libraries/`; experiments/sweep_scenarios.py "THE SIMULATOR'S STRESSORS
  ON THE RAILS", docs/RUNBOOK-orin-hil.md step 14):
  - `wind_speed_mps` -> `SIM_WIND_SPD` (m/s, `SITL/SITL.cpp:74`), 0 at boot,
    the cell's speed at t = 0 (SITL low-passes it 0.95/0.05 per physics
    update: a step), with `SIM_WIND_T` 1 (no altitude profile, `SITL.cpp:
    203`) and `SIM_WIND_TURB` 0 at boot;
  - `wind_dir_deg` -> `SIM_WIND_DIR` at boot, the direction the wind comes
    FROM (`SITL.cpp:80`; `SITL/SIM_Aircraft.cpp:779-804` negates it into the
    air's velocity): (toward + 180) mod 360, toward the scenario's direction
    turned with the mission, or the harness's own draw for the seed;
  - `gps_dropout_s` -> `SIM_GPS_DISABLE` (4.5.7's name, `SITL.cpp:561`; 4.6
    renames it `SIM_GPS1_DISABLE`) 0 at boot, 1 at t0 + start, 0 at t0 + end;
    SITL's GPS then reports no lock (`SIM_GPS.cpp:333`, fix type 0);
  - `start_jitter_m` -> `SITL_HOME` and `--start`, drawn exactly as the
    headless harness draws it for the seed (`random.Random(f"{seed}:
    jitter")`): a cell's headless and rail episodes start at one point.
- `sitl/ros2_shield_node.py`:
  - `State.fix_ok` from `/mavros/global_position/raw/fix`, the NavSatFix
    MAVROS 2's global_position plugin (core MAVROS: both rails load it
    without mavros_extras) builds from GPS_RAW_INT, STATUS_FIX when
    fix_type > 2 (`fix_ok_from`: None before the first message and once one
    is older than the 1 s pose timeout, so a rail without the topic flies
    exactly as before). Rows carry `fix_ok`, `fix_status`, `fix_age_ms` and
    the Shield's `position_lost` once the topic has spoken; metrics.json
    `fix_quality`.
  - X8 on the node's FSM: `tick_input_from_decision(position_lost=)` from
    the Shield's own verdict.
  - SITL's stressor parameters read back at take-off (SITL only; two reads
    that agree, and a read that fails is recorded with why while bring-up
    goes on) and every change the autopilot broadcasts recorded on the
    mission clock (`/mavros/param/event`), with the t = 0 markers sent
    (metrics.json `sim_params`).
- `guardrail/shield.py`: a Shield with no FSM of its own (a rail's,
  `rail_shield`) judges X8's input on a state that reports its fix quality
  and hands it out (`ShieldDecision.position_lost`, excluded from every
  dump), and `note_flown` feeds its dead reckoning with what the rail SENT
  (a stop, nothing while the autopilot holds, or the Shield's action). A
  state that reports nothing computes nothing new.
- `sitl/hil_desktop.sh start --stress-plan FILE --stress-record FILE`: READY
  only after `SITL_STRESS BOOT OK`, else start fails and nothing flies; `stop`
  ends the stressor process with TERM first. `sitl/run_ros2_demo.sh`:
  `SITL_STRESS_PLAN` / `SITL_STRESS_RECORD` the same way, before MAVROS
  starts; refused with `ROUTER=0` (MAVROS then holds tcp:5760).
- `experiments/sweep_scenarios.py`: `rail_stress_plan` (the plan, kept on
  the desktop under `demo/out/_stress/<tag>/`; nothing of it goes to the
  Orin), `rail_wind`, `sim_wind_dir_deg`, `start_jitter`, and
  `rail_stress_check`, which fails closed after the mission: the episode is
  an ERROR, never scored under the cell's name, unless the simulator side's
  record is this launch's and its boot parameters read back, the node read
  the same values at take-off, every scheduled change was seen within
  -0.05 / +0.2 s of its time on the node's clock (on the simulator side's
  own t0 only when the node saw no parameter event at all), a dropout
  reached `fix_ok` within 1 s, and a jittered home lies within 0.25 m of
  `--start`; a cell that sets nothing must find SITL at its neutral values.
  `scenario_score.json` gains `sim_stressors` (every SIM_* parameter in the
  pymavlink rail's record, with its time, the node's observation and the
  error) and `sim_stressors_check`. The dry run prints each mission's boot
  and scheduled parameters and the helper's `--stress-*` arguments.
- Docs: docs/RUNBOOK-orin-hil.md step 14 (the mapping and its source lines,
  the episode's clock, fail closed, what SITL's physics changes, the labels
  it may not reach, what to fly), docs/DESIGN-topologies.md decision 12 (why
  the simulator's side and not the Shield node), docs/DESIGN-escalation-
  fsm.md (the ROS 2 row of "Where each rail's signal lives"), sitl/README.md.

#### Changed
- `rail_refusal`: wind, its direction, GPS denial and start jitter fly;
  refused with the reason: `gps_noise_m` (4.5.7's `SIM_GPS_NOISE` is a slow
  sine on the GPS altitude alone, `SIM_GPS.cpp:353`, and noise fed to
  SITL's GPS would be fused by the EKF that also flies the aircraft),
  `latency_ticks` (the rail's latency is its own, recorded per row),
  `gust_mps` (`SIM_WIND_TURB` redraws its azimuth every physics step from
  SITL's own random numbers, `SIM_Aircraft.cpp:785-800`: not the harness's
  per-tick Gaussian gust nor its seed), `battery_critical_at_s` (no RTL
  watch on the node, so no G6), and a dropout flown blind
  (`fix_quality_reported: false`). `--hil-dry-run`: smoke 64 episodes, 41
  missions, 23 refused -> 47 missions, 17 refused (+2 gps-dropout, +2
  wind-within-authority, +2 wind-beyond-authority); nightly 744 episodes,
  546 missions, 198 refused -> 588, 156 (+24 gps-dropout, +6
  wind-within-authority, +12 wind-beyond-authority). wind-gusts and both
  GPS-noise families stay refused.
- `rail_mission`, `sitl_home`, `rail_meta` take the seed (the jitter);
  `rail_meta` records the realised stress (`stress`). `sitl_extra` measures
  `gps_dropout_ticks` on a rail (rows whose `fix_ok` is False; 0, never
  None, when the denial never reached the Shield, so the cell's gate fails).
- ONE MECHANISM WITH THE PYMAVLINK RAIL'S. `parse_sim_params` moved from
  `sitl/run_sitl_demo.py` to `sitl/sitl_stressors.py` unchanged, and both
  rails share it, the read-back test (`read_back_ok`, 1e-3 x max(1, |v|))
  and the record of a stressor (`stressor_entry`: `param`, `set`,
  `read_back`, `applied` - the keys of the pymavlink rail's metrics.json
  `sim_stressors`). The pymavlink rail is the Shield's own process and sets
  its static stressors itself at mission start; the ROS 2 rails' Shield node
  is the system under test, so there the simulator side sets them, on the
  episode's clock.
- `sitl/sros2_policy.xml`: the Shield's enclave may subscribe to
  `/mavros/global_position/raw/fix` and `/mavros/param/event` and publish
  `/mavros/statustext/send` (the t = 0 marker). A keystore made before
  needs tools/sros2_setup.sh's permissions step again; until then the node
  records the refused input (metrics.json `sim_params.inputs_refused`) and
  flies without it (fix_ok None, no marker: a stressed cell errors).

#### Fixed
- `run_sitl_demo.py --sim-param SIM_WIND_SPD=<v>` now brings `SIM_WIND_T`
  1 unless a profile is given (`sitl_stressors.with_uniform_wind`):
  SITL's default square law scales the wind by sqrt(altitude / 60 m) below
  60 m (`AP_HAL_SITL/SITL_State.cpp:443-446`).
- `sitl/sros2_policy.xml` also grants the Shield's enclave the autopilot's
  home (`/mavros/home_position/home`, `/mavros/home_position/req_update`),
  which the node has used since the 2026-11-06 home check and SROS 2 would
  have refused it under `SROS2=1`.

#### Retracted
- **The 2 Nov SITL wind runs flew half the wind they name.**
  `sitl_wind8_shield_on_20261102` and `sitl_wind8_shield_off_20261102` set
  SIM_WIND_SPD 8 without SIM_WIND_T, so at their 15 m the wind was
  8 x sqrt(15 / 60) = 4 m/s (no default parameter file sets SIM_WIND_T);
  the 2026-11-02 (evening) entry says 8 m/s, and docs/FINAL-REPORT-outline.md
  row X-14 cites the runs by those names. Their reading of
  SIM_WIND_DIR 225 as "from 225 deg (toward the zone)" was right. The runs
  are not re-scored here.

#### Limits
- **Not flown.** No mission with any of this has flown: the rig was in use.
  What must be flown to show it, one mission each with the dry run's
  commands: a wind-within-authority lane, wind-beyond-authority at 8 and at
  12 m/s, gps-dropout lane 23.6 4-11 s (E3) and lane 15 6-13 s (N2); each
  checked for `sitl_stress.json`, alignment errors under 0.2 s in
  `sim_stressors`, the dropout's onset in `fix_quality.no_fix_spells`, the
  FSM's X8 edge, and whether ArduPilot's EKF failsafe took the aircraft
  (`autopilot_took_over`).
- **Read in the source, not yet seen on the rig**: ArduPilot forwarding the
  node's STATUSTEXT to SERIAL0 once the stressor process's heartbeat has
  taught the route, and MAVROS 2 publishing `/mavros/param/event` for the
  PARAM_VALUE ArduPilot broadcasts after a set. If the marker never arrives
  the schedule never fires and the episode errors; if the events never
  arrive the alignment is the simulator side's own, and says so.
- **Not the harness's model.** On SITL the wind acts through ArduPilot's
  physics (its drag model, the position controller's real authority, the
  transient of the step at t = 0), never `--wind-model`; the dropout through
  EKF3, whose pose keeps moving on inertial dead reckoning (the headless
  harness hands the Shield the frozen last fix), and ArduCopter's EKF
  failsafe lands the aircraft about 8 s after the GPS is lost (7 s
  posRetryTimeNoVel_ms, `AP_NavEKF3.h:466`, then 1 s of `ekf_check.cpp`).
- **Labels SITL may not reach** (none relabelled; docs/RUNBOOK-orin-hil.md
  step 14): wind-beyond-authority at 8 m/s (E2 is read from the harness's
  6.4 m/s authority; SITL's quad may hold 8 m/s), the gps-dropout E3 cells
  (X8 on the drift allowance alone must beat the EKF failsafe), lane 15
  6-13 s (N2, the same EKF race on the 7 s windows); wind-within-authority
  and the 2-4 s dropout windows (N2) should hold.
- `pos_sigma_m` is not fed on the rail: with 1 m margins X8's sigma branch
  needs sigma <= 0.33 m; SITL reports 0.3 m, a field receiver metres.
  Feeding it, or sizing margins to it, is a policy decision left open.
- Still refused: GPS noise, state latency, gusts (wind-gusts, both
  GPS-noise families) and the battery hook (failsafe-low-battery).

#### Tests
- New `tests/test_sitl_stressors.py` (21): the wind's FROM convention against
  SITL's air velocity, the boot / t = 0 split and the uniform profile, a
  drawn direction equal to the headless draw for four seeds, the dropout's
  schedule, plan validation, the jittered start equal to the headless start
  for four seeds (SITL_HOME and --start), the scheduler on a virtual clock
  (never early, late by the marker's delay, a boot parameter that does not
  read back stops it, no marker no schedule), the fail-closed check (13
  ways), the refusals, `fix_ok_from`, the rail Shield's X8 verdict and the
  dead reckoning on what was sent, the node on the stand-in ROS stack
  (marker, fix_ok, X8 -> RTL, `sim_params`, `fix_quality`; a node without
  the topic as before), the scripts, the transport, run_sitl's WSLENV, a
  hil campaign whose desktop side realises the plan, and the record the
  pymavlink rail shares. `.github/workflows/tests.yml` runs it.
- test_sweep (the refusals, a hil campaign whose rig realises no stress
  errors its dropout missions, the dry run's counts), test_sitl_rails
  (`parse_sim_params` from `sitl_stressors`).
- Run here one file at a time (Windows, Python 3.10), the 62 files that
  load what this changes or read the changed documents, each with the
  skips CI had at bf31b51:
  test_sitl_stressors 21/21, test_sweep 107/108, test_sitl_rails 63/63,
  test_ros2_vla_bridge 47/47, test_railinputs_node 13/16, _clock 7/7,
  _hotapply 2/6, _policy 11/11, _perception 35/35, test_ros2_package 17/17,
  test_position_quality 14/14, test_own_velocity_forecast 28/28,
  test_shield 86/86, test_fsm 124/124, test_replay 40/61,
  test_mavlink_adapter 40/40, test_pas_ardupilot 148/149,
  test_deploy_configs 69/70, test_kpi_report 38/41, test_check_claims
  22/22, test_harnesstruth_ci 35/35, test_ci_local 10/10, test_public_docs
  7/7, test_build_public_site 47/47, and the harnesstruth, wp4outputs,
  shieldcore, window, hold-point, vla and tracker-page files.
  Two failed first and were fixed: test_ros2_vla_bridge (a stand-in's reply
  made the node's new take-off read raise; it now records a failed read and
  bring-up goes on) and test_harnesstruth_ci (the workflow's header now
  names test_sitl_stressors).

### 2026-11-07 (hil shakedown) — the first stress missions on the hil rig: three rail defects fixed, and the autopilot's lag escalates head-on approaches the ideal vehicle never escalates

One smoke scenario (`nfz-head-on`, the goto pilot at 4 m/s at a square
no-fly zone, the target behind it) flown three times on the hil rig through
`experiments/sweep_scenarios.py --backend hil` (ArduCopter 4.5.7 SITL in WSL,
MAVROS 2 and the Shield on the AGX Orin). Not KPI-grade: shk01 flew master
62a4713, shk03 flew 8bfe08f, neither a release.

#### Fixed
- **Kinematic caps are judged with the float32 contract's slack**
  (`guardrail/fsm.py` `cap_exceeded`, used by the Shield's check and clamps,
  `KinematicCaps.clamp` / `exceeded_by` and `compiler.action_violations`).
  shk01: the stub asked for exactly the 4 m/s cap; quantised to float32 in
  the body frame and rotated to the world it read 2e-8 to 1.2e-7 m/s over it
  on 148 of 299 ticks, each a kin-caps violation under the old 1e-9 slack:
  G1 Normal -> Brake at 0.3 s, the onsets G2 -> Loiter at 2.9 s, the mission
  lost to rounding. The slack is a millionth of the cap (never under 1e-9),
  about eight float32 steps. Headless pins unchanged (test_sweep,
  test_check_contract's 28 800 hashed answers, the c77c77d digests).
- **The hil transport never hands ssh, scp or the WSL helper its own
  stdin.** shk02 lost its mission before take-off: `ssh ... mkdir -p` ran
  into its 60 s timeout though it answers in 0.4 s by hand.

#### Changed
- **A rail mission's time cap is the scenario's plus the node's Loiter
  timeout plus 5 s** (`rail_max_s`; the scenario meta records `rail_max_s`
  beside `max_s`, and the hil mission timeout is taken from it). shk03 entered
  Loiter at 3.4 s and its 30 s episode ended there: a censored fail-safe
  (docs/RUBRIC-failsafe.md section 3). A mission that reaches its target still
  ends at the target. docs/RUNBOOK-orin-hil.md says so.

#### Found
- **On the rig the Shield escalates a head-on approach the ideal vehicle
  slides past.** shk03, with the cap fix: the zone enters the 5 s forecast
  at 2.9 s, GeofenceSlide repairs (repair 0.27 m), G1 Brake; the autopilot
  reacts after about 0.45 s and brakes at about 1.75 m/s^2, so the aircraft
  keeps closing and the horizon-end depth grows about 0.4 m a tick: 2.20 m
  > theta 2.00 m at 3.4 s, G3 Brake -> Loiter. The headless harness's ideal
  vehicle stops closing on the tick and never escalates. The same profile
  headless on the measured ArduPilot response (`--response-arms ideal
  ardupilot`): fail-safe labels 62/63 on the ideal vehicle, 49/54 on the
  ArduPilot response (5 false triggers, 0 missed, the rest not decided),
  static_nfz 6/7 -> 0/1; P0 escapes 0 -> 0. The labels were derived on the
  ideal vehicle; which vehicle the KPI-grade labels should be derived on is
  open (a measured proposal is being prepared; the theta basis is unchanged).
- The slide's direction follows the sign of the stub's small sideways
  command (+/-0.02 m/s) at a head-on tie, so it flipped between +4 and -4 m/s
  four times in eight ticks before the escalation. Not changed here.

#### Tests
- `tests/test_shield.py` 87/87 (a float32 command at a cap is at it, an
  excess past the slack is clamped), `test_sweep.py` 107/108 (the rail's time cap and
  the hil mission timeout; 1 skip), `test_fsm` 124/124, `test_check_contract` 2/2,
  `test_own_velocity_forecast` 28/28, `test_csp` 100/100,
  `test_position_quality` 14/14, `test_sitl_rails` 63/63,
  `test_kpi_report` 38/41.

### 2026-11-07 (integration) — four branches merged and tested on two machines; the suite run on the AGX Orin under Python 3.12

#### Changed
- Merged into master: IR 4.0.0, the perception VLA rail, the hil scenario
  campaign and the fail-safe work (X8, noise hysteresis, cap_all), each
  section below. The fail-safe branch's new test file stated no `scope`,
  which IR 4.0.0 requires (fixed, 14/14).
- `docs/data/progress_oct.json` regenerated on the merged tree (WP1 round
  trip 32/32, WP2 P0 122/122 at 256 tokens, smoke 64 / nightly 744
  episodes). The October explainer pack reads `docs/data/csp_kpis.json` as
  committed with it (335ec58), like its other two sources: the 6 Nov
  regeneration (120/122 at 144 tokens) made its build refuse "104/105 at 120".
- IR 4.0.0 flight bundles built and signed on the desktop and on the AGX
  Orin (its own key), the same policy hashes on both.

#### Fixed
- `tracker/serve.py` drains a refused POST's body before answering, so the
  403 / 404 reaches the client: the socket was closed with the body unread
  and reset, and `test_tracker_serve`'s refusal tests errored on 20 of 80
  calls (WSAECONNABORTED, 10053); 0 of 120 after.

#### Measured
- Desktop, Python 3.10.20, at bf31b51: 141/142 files, 3540/3674 tests,
  smoke fail-safe 0.984127 over 64 episodes (the red file: the tracker
  reset above). Lab checkout at 65c7e4f: 140/142, 3645/3674 (the two red
  files: the derived data above).
- AGX Orin, Python 3.12.3 aarch64 (`~/vla-drone-env`, numpy 1.26.4), at
  bf31b51: 139/142 files, 3485/3672 tests, the same smoke figures to the
  last digit. Red: the venv's numpy against the 3.11+ pin (an environment
  fact), the x86-64 identity digests (2 of 500 decisions of one stream
  differ by 1 ulp: the platform's sin/cos; desktop 3.10 and 3.12 are
  byte-identical), a run store of 13 flights where the test needs 20
  (`docs/data/ci_local/bf31b5142fc3-agx.json`).

### 2026-11-07 (CSP flights ready) — OpenVLA can fly the CSP A/B and the paraphrase set in AirSimNH at bf16 and both can be scored; each seed is a start, the flights and the harness draw their paraphrases from the paraphraser service, the replay judges with the flight's Shield, the mission context carries home and allowed modes; nothing was flown

Audit cards WP2-01, WP2-05, WP2-09, WP2-18, WP4-09, WP4-10 and WP4-15. **Nothing
was flown.** No simulator, SITL, WSL, Docker or Orin was started. OpenVLA-7B
was loaded once on the RTX 4090 for a smoke check, then freed. It ran at the
new `--precision bf16`, through `tools/prefix_eval.py`'s own loader and
replay, on the committed protocol frame
(deploy/evidence/vla_protocol_frames/present_+00.png) with no simulator
running:
- load 117 s from D:, 15.1 GB of VRAM, the weights bfloat16;
- 0.55-0.61 s an inference after a 2.6 s first call;
- three greedy repeats of each prompt answered identically;
- the off, CSP (80 tokens) and neutral (85 tokens) prompts each gave a
  different action, and the replay ran end to end with the run's recorded
  Shield.

One frame is not a measurement: this checks the code path, not the CSP's
effect.

#### Decided (the orchestrator's rulings for this campaign, 2026-11-07)
- **AirSimNH is the image source for this measurement**, with no CityLife
  port now. Every record and document of these flights says "AirSimNH
  (classic AirSim, simple_flight), not the grant's Project AirSim + ArduPilot
  rail, not KPI-grade" (`real_vla_demo.rail_label`).
- **G3 (is the decision rate fast enough) stays with the PI.** Each flight
  records the rate it measured.
- **The campaign flies bf16.** If it does not fit beside AirSimNH, the flight
  refuses with the numbers; nothing falls back to nf4. The first smoke flight
  records the peak VRAM.
- **A seed is a start**: a deterministic offset of position and heading from
  the spawn, the same for both CSP arms of a seed, recorded. No wind.
- **The risk weights stay 0.5 / 0.3 / 0.2.** The PDF lists their tuning as an
  open question naming no objective or data; the A/B data could later serve
  one. Not tuned now.
- **The demo leaves home and allowed modes unstated**: its rail executes no
  mode change.
- **The flights' paraphrases come from the paraphraser service** (the runner
  starts it on loopback; each run records `via`, as the harness does), so they
  count as issued to a VLA by the service (WP4-10). The stored-set draw in
  process stays only as an explicit, recorded fallback flag.
- **The model stays on D:, the frames go to E:** (`E:\vla_drone_offload\demo_out`,
  created when missing); the runner refuses to start under 20 GB free there.

#### Found
- **`demo/real_vla_demo.py` cannot fly the CityLife rail.** It talks to
  classic AirSim through the `airsim` package (msgpack-rpc, port 41451) and
  flies `policies/urban_demo_policy.yaml`, which was drawn for AirSimNH.
  CityLife is Project AirSim (port 8989), a different protocol
  (docs/pasblocks-study.md). The campaign below therefore flies AirSimNH.
  Porting the demo to Project AirSim is a separate change, not made here.
- **The lab's classic AirSim settings declare an ArduCopter vehicle**
  (`Documents\AirSim\settings.json`, both copies, since 17-18 September, for
  the SITL rail). That vehicle waits for SITL and ignores AirSim's velocity
  API, through which `real_vla_demo` takes off and flies. The runner leaves the
  file alone and starts AirSimNH with its own settings
  (`tools/airsim_settings_real_vla.json`: SimpleFlight, the 1280x720 scene
  camera of July) through AirSim's `-settings=` argument. The AirSimNH binary
  (the v1.8.1 build of 17 July 2022) carries that argument and the log line
  "Loaded settings from commandline" among its strings; it has not been run
  with it.
- **What the grant asks of the mission context and the risk weights**
  (Prefix Compiler PDF):
  - p3, Pipeline: "Mission context (pose, home, target, allowed modes)" feeds
    the "Spatial + temporal filter (intersect mission bbox + lookahead
    window)"; p4: `prox = proximity_score(rule.geometry, mission_ctx.bbox)`
    and `return 0.5 * sev + 0.3 * prox + 0.2 * tcrit`; p2, the CSP field
    `relevance_explanations: dict[str, str] # rule_id -> mission-context fact`,
    with the example "mission target is within 200m of polygon".
  - p6, Outputs: "Risk-grading function | Python module, configurable
    weights"; p6, Open questions: "The 0.5 / 0.3 / 0.2 weights above are
    placeholders; need empirical tuning during Q2." The page names no data
    set, objective or procedure for that tuning.

#### Fixed
- **The offline replay judged with a Shield the flight never had.**
  `tools/prefix_eval.py` built every replay Shield at 3 s / 0.5 s ("what
  real_vla_demo flies with"); the demo has built the grant's 5 s / 0.1 s
  Shield since 8d1a0f3 (6 Nov). The settings now come from the run:
  `real_vla_demo` writes
  `shield` into vla_run.json (`shield_record`, read off the Shield object),
  and the replay rebuilds its Shield from it and checks the first one it
  builds against the record. It refuses a record it cannot rebuild (a rule
  clock, an obstacle or road map, an FSM config other than the defaults, an
  unknown key) and flags a recorded forecast other than the grant's. A run
  with no record gets guardrail.shield's constants, labelled unrecorded; one
  that started before 8d1a0f3 flew 3 s / 0.5 s and is refused unless
  `--allow-shield-mismatch` (then flagged `matches_flight: false`). Before
  any model loads. The model is loaded at the run's own `precision`; the
  report says when `--precision` makes it another.
- **`real_vla_demo` built its Shield with an escalation FSM it never acted
  on.** It flies `emitted` on every tick and requests no mode, so the FSM's
  Brake, Loiter, RTL and Land went into the audit without having flown, and
  guardrail.kpi would have read `RTL_triggered` off rows whose aircraft
  never turned home (docs/DESIGN-shield-enforcement.md listed the demo
  scripts that do this). It now flies a rail Shield
  (`guardrail.replay.rail_shield`: 5 s / 0.1 s, no FSM inside), as
  `demo/vla_fly.py` does.
- **A rerun under the same tag appended to the earlier run's audit**
  (`AuditLogger` appends by default). The demo now opens it fresh, with the
  run id as its episode id, and removes the earlier run's vla_run.json,
  flight log, metrics, KPIs and trajectory before it starts.
- **A model worker that died left its last action flying.** An exception in
  `real_vla_demo`'s inference thread (CUDA out of memory, a simulator that
  stops answering) ended the thread, and the control loop flew the last
  action, through the Shield, to the end of the flight; the run then read as
  complete. Now the worker holds zero and records why (`worker_error`), the
  control loop stops the flight and lands, and `metrics.json` says `stopped`
  (exit 4). Out of memory is the VRAM refusal met in flight: `refused: vram`
  with the device's peak, exit 3. The scorer leaves both out of every
  comparison. Shown failing first: with the worker's catch removed, the new
  test's flight exited 0.

#### Added
- **`real_vla_demo` records what a CSP on/off A/B and the replay need.**
  - `--precision nf4|bf16`. nf4 (bitsandbytes 4-bit) stays the script's
    default, as the docs have it; the campaign flies bf16, the checkpoint
    unquantized. Before a weight is read, the device's free VRAM (the
    simulator already on the card) is checked against 15.6 GB at bf16 / 5.2 GB
    at nf4 (the smoke check's 15.13 GB and July's 4.7 GB, plus an allowance
    for one inference) and 1 GB for the renderer (`vram_verdict`). Too little,
    or CUDA out of memory during the load, refuses the flight with the numbers
    (exit 3, `metrics.json` `refused: vram`), before take-off. The device's
    VRAM is sampled after every inference: `metrics.json` `vram.peak` (peak in
    use, least free, the model's own peak allocation).
  - `--seed N` is the start. `start_offset` draws, with `random.Random(N)`, a
    point within 3 m of the spawn and a heading within 30 deg of it; after
    the climb the drone is flown there with AirSim's position and yaw commands
    (set-up, like the take-off, not through the Shield) and the pose is read
    back (`fly_to_start`). More than 1 m or 5 deg off lands and refuses (exit
    3, `start_not_reached`). Both CSP arms of a seed, and the paraphrase runs
    at it, start from the same pose. No wind. The seed also seeds Python's,
    NumPy's and torch's generators, which changes nothing OpenVLA computes
    from one frame and prompt (greedy decoding; `SEED_NOTE`, in every run).
  - The decision rate: `metrics.json` `decision` (decisions in the flight,
    per second, the gaps between them and each inference's time, beside the
    grant's 10 Hz observation path), and a `[rate]` line in the report.
  - `--world` (AirSimNH by default) and the label `rail_label(world)` in
    vla_run.json, metrics.json, `--print-prompt` and the report.
  - `--max-s`, and in vla_run.json the precision, quantization, seed and what
    seeding did, the start offset, the Shield, the simulator check, the VRAM
    verdict at load, the tag and the argument list.
  - A paraphrase run draws its wording from the paraphraser service:
    `--paraphraser HOST:PORT` (plaintext on loopback, or mutual TLS with
    `--paraphraser-pki`). The record carries `via` (the service, the
    transport, the validator and store digests the reply carried), in
    vla_run.json and metrics.json. `--paraphrase-in-process` is the explicit
    fallback, recorded as `via.in_process`; a paraphrase run with neither, or
    with both, is refused before anything loads. Both draws give the same
    ids and text. The service's `ParaphraseRecord` gained `form` (proto field
    11, stubs regenerated with grpcio-tools 1.84.0), so a drawn item's form
    (verb_phrase, utterance) comes with it.
  - `sim_preflight`, before the model loads: a port nobody answers on is
    refused (naming Project AirSim when port 8989 is what listens), and so is
    a vehicle that is not SimpleFlight. A simulator that cannot report its
    settings is recorded as unverified.
  - A take-off that does not reach cruise height lands and refuses (exit 3),
    with `metrics.json` saying why, instead of flying the model from low.
  - Per tick `flight_log.jsonl` in guardrail/kpi.py's shape (`raw` = the
    rate-limited action the Shield judged, `vla` = the model's own; emitted,
    violations, repairs, the flown re-check, the unsafe flags),
    `metrics.json` (repair rate = ticks the Shield changed the action /
    ticks, NFZ seconds, the simulator's contacts from the VLA's start, the
    start pose) and `kpi.json` (`guardrail.kpi.compute` on those).
  - `--print-prompt` also shows the whole flight: precision, seed and start
    offset, Shield, mission, policy hash, length, label, and the paraphrase
    with its `via`.
- **`tools/run_csp_flights.ps1`** (PowerShell 7) flies the campaign in order,
  at bf16 (`-Precision`). First the A/B: `--csp off` then `on` for seeds 1001,
  1002 and 1003. Then the paraphrase set, CSP on, at seed 1001: the first 4
  stored items of each family (`verb_phrase`, `utterance`; utterances with
  their grammar confound recorded), drawn from the paraphraser service the
  script starts on loopback (`python -m guardrail.paraphraser_service serve
  --insecure`, port 50061, `-ParaphraserPort`; a port already taken is
  refused) and stops after. Each flight starts from a restarted AirSimNH.
  After each flight it prints the decisions, the rate, the median inference
  and the peak VRAM, or the refusal with its numbers; a VRAM refusal always
  stops the campaign, and so does a stopped flight (exit 4). `-DryRun`
  checks every flight without a model or a simulator. `-SkipDone` resumes a
  campaign. Each run's frames go outside OneDrive through a junction
  `demo\out\<tag>\frames` to `E:\vla_drone_offload\demo_out\<tag>\`
  (`-OffloadRoot`, created when missing); the script refuses to start with
  less than 20 GB free there (`-MinFreeGB`). It refuses to fly while
  `tools/ci_local.py` runs. Its header holds the exact sequence and the
  estimated durations (about 5-7 min a flight, 1.2-1.8 h for the 14).
- **`tools/score_csp_flights.py`**: `plan` (the flights, their tags and exact
  arguments), `check` (the dry run) and `score`. Every report carries the
  label.
  - `check` runs `--print-prompt` for every planned flight, starting a
    loopback paraphraser service in process where the plan names one and
    nothing listens there, and checks that each fits its budgets. It also
    checks that both arms of a seed fly one configuration (start offset
    included) and differ only by the CSP ahead of the question, that two
    seeds plan two starts, that every CSP-on flight reads one CSP (one
    content hash), and that every paraphrase flight reads that CSP and its
    own stored item, drawn by the service it was planned to come from.
  - `score` prints the WP4-15 table: per seed and arm, the decision rate and
    the peak VRAM, the repair rate, mean and max repair magnitude, P0
    escapes, NFZ seconds, contacts and outcome; the on - off delta, and its
    mean over the valid pairs only. A pair is invalid when an arm is
    missing, refused, stopped, unfinished or never answered by the model, or
    when the arms flew another configuration or started more than 2 m /
    15 deg apart; it is listed with why.
  - `score` also prints the WP4-09 outcome stability: each paraphrase run's
    outcome against the canonical arm's, per family and overall, and
    `guardrail.kpi.per_paraphrase_robustness`, and how many paraphrase runs
    the service issued (WP4-10). It lists the `prefix_eval` command for every
    finished run.
- **WP2-05: home and allowed modes in the mission context**
  (`guardrail/compiler.py`). `Mission.home_x` / `home_y` (None: the start, as
  before) and `Mission.allowed_modes` (the grant's SET_MODE names GUIDED /
  LOITER / RTL / LAND; None: not stated). Where the grant puts the context:
  - the mission region (the filter's bbox) and the proximity grade take in
    the return home when a home is stated and RTL is not ruled out. An RTL
    from anywhere on the path flies inside the triangle start - target - home;
  - the relevance explanations name the home ("a return home to (0, 600) can
    cross this zone") and any breach action whose mode the mission does not
    allow;
  - both join `mission_id` only when stated.

  A Mission that states neither compiles byte for byte as before: 288 CSPs
  (every shipped policy x 3 missions x 3 clocks) had identical content hashes
  before and after. The natural-language prompt never changes with the
  context, so the stored paraphrase sets stay valid. The weights stay the
  grant's 0.5 / 0.3 / 0.2 and are still untuned (Decided and Found, above;
  docs/DESIGN-prefix-compiler.md says the same). `real_vla_demo` states
  neither home nor modes, so its CSPs are unchanged.
- **WP4-10: the stress harness, and the CSP flights (above), draw their
  paraphrases from the paraphraser service** (`guardrail/harness_service.py`,
  `demo/real_vla_demo.py`). The reference data-flow page
  has the "Paraphraser service" feed the "Orchestrator service", which
  "issue[s] tasks (and paraphrased variants) to the VLA", over gRPC + mTLS.
  - `Harness(paraphraser=...)`, `serve --paraphraser HOST:PORT`: mutual TLS
    with the lab PKI's `harness-client` certificate (the name the service
    allows by default), or plaintext on loopback with `--insecure`.
  - No in-process fallback in the harness, because the reference design
    describes none. A server bound to no service refuses a paraphrase arm
    (FAILED_PRECONDITION); a service that does not answer is UNAVAILABLE.
    The flights keep `--paraphrase-in-process` only as the explicit,
    recorded fallback the orchestrator allowed.
  - The arm record carries the same ids and text an in-process draw gives,
    plus `paraphrase.via`: the service, the transport, and the validator and
    store digests the reply carried. The canonical arm and the
    protected-prompt check stay in process: they draw nothing.
  - `experiments/sweep_scenarios.build_arm` takes the drawn arm (`drawn=`);
    the headless batch sweep still draws in process.
  - `ParaphraserClient` keeps its address and transport for that record.

#### Tests
Run one file at a time in the worktree (vla-real, Python 3.10), with no
`tools/ci_local.py` running; the counts at 65c7e4f, from the lab run, in
brackets.
- `tests/test_prefix_eval.py` 21/21 (13). New:
  - the replay judges with the recorded Shield. A 3 s and a 5 s record score
    the same frame differently, so a replay that ignores the record (the old
    3 s / 0.5 s, or the constants) fails; both mutants are also refused by
    the read-back check;
  - an unrecorded run takes the constants, labelled so;
  - an unrecorded run from before 8d1a0f3 is refused, by the CLI before any
    model is built;
  - a record that cannot be rebuilt is refused, and so is a rebuild that
    drops a recorded setting;
  - the model loads at the flight's precision;
  - the campaign plan (bf16, AirSimNH, absolute policy path, the paraphrase
    flights on the service and the in-process fallback only by name);
  - the dry run on a two-seed plan, through a loopback paraphraser service it
    starts, and seven mutations it must catch (another precision or start on
    one arm, two seeds with one start, another CSP, an off arm with text, the
    canonical wording flown as a paraphrase, a paraphrase drawn in process).
    It needs grpcio and SKIPs without it (the runner now counts a SKIP);
  - the scorer on synthetic runs: a valid pair, an invalid precision, a
    missing arm, a different start, a stopped arm, paraphrases with and
    without the canonical's outcome, one reading another CSP, the decision
    rate and VRAM columns, the label, and the paraphrases the service issued.
  - `test_the_action_scale_is_the_one_the_step_flew_with` now uses scale 15:
    at the flight's 5 s forecast, 0.6 m/s reaches the margin ring.
- `tests/test_real_vla_prompt.py` 69/69 (51). New: the flight Shield and its
  record, `--print-prompt`'s flight block, precision and seed, the run
  record, the rows / metrics / KPIs (kpi.json recomputed with
  `guardrail.kpi.compute`), contacts during and before the mission, the
  take-off refusal, and the simulator preflight (an unanswered port, Project
  AirSim's port, an ArduCopter vehicle, an unverifiable one) with `main()`
  refusing before the backend is built. For the rulings:
  - the label in every record, spelled out in the test;
  - the seed's start: the draw against its stated rule, bounds, 40 seeds
    with 40 starts, and both CSP arms of seed 1001 flown to one target;
  - a start the vehicle does not reach lands and refuses;
  - `vram_verdict`, and the real `_load` against torch/transformers
    stand-ins: too little free VRAM is refused before a weight is read, an
    out-of-memory load is the same refusal, bf16 loads unquantized;
  - `main()` refusing a model that does not fit, with the numbers, without a
    second load and without taking off;
  - the decision rate and the VRAM peak per flight;
  - a worker that dies in flight (out of memory, then a lost simulator)
    stops the flight at the next tick, holds zero and lands, with exit 3 and
    exit 4 (made deterministic with simulator hooks);
  - the paraphraser service: the same ids and text as the in-process draw,
    `via` in the records of a flight, and the refusals (an utterance, an
    instruction with no stored set, neither source, both).
  Four mutants were checked against these tests and each made a test fail:
  the worker's catch removed, the loop's stop check removed, the move to the
  start removed, the VRAM check removed.
- `tests/test_csp.py` 104/104 (100). New: the context fields are checked as a
  whole and an old run record still loads; a mission without them keeps its
  old `mission_id` and gets no new clauses; a stated home brings a zone on
  the way home into the region (proximity 1, fine geometry, explained), and
  ruling out RTL takes it out again (the null); a breach mode the mission
  forbids is explained, and the modes change neither the prompt nor any
  decision.
- `tests/test_harness_service.py` 62/62 (59). The shared test harness is now
  bound to a loopback paraphraser service. New:
  - with `guardrail.paraphraser.paraphrase` made to fail, an arm is still
    drawn, from the bound client, with its record and `via`;
  - over gRPC the arm is the in-process draw's;
  - an unbound server, and a dead service, draw none (FAILED_PRECONDITION,
    UNAVAILABLE);
  - plaintext off loopback is refused.
- Passing after the last change, with no new tests: `tests/test_paraphraser_service.py`
  8/8 (the stubs carry `form`), `tests/test_paraphraser.py` 103/103,
  `tests/test_wp4outputs_harness.py` 16/16, `tests/test_harnesstruth_ci.py`
  35/35 (it reads the workflow header, which now says where the two new
  service tests SKIP). `tests/test_wp4outputs_exporters.py` 38/38 was run
  before the follow-up changes, which do not touch what it imports.
- Not run: the full CI list (`tools/ci_local.py`), and `tests/test_sweep.py`
  (`build_arm` gained an optional argument only).
- The dry run: `pwsh -File tools\run_csp_flights.ps1 -Prefix cspab_20261107
  -DryRun` passes 14 flights in 3 s.
  - The off prompt is 20 OpenVLA tokens, the on prompt 80 (the CSP 60, all
    three urban rules kept), counted with openvla-llama2-sp@8f5e2869e180.
  - Every CSP-on flight reads one CSP (content hash `sha256:367e04b88e9a...`).
  - The paraphrase prompts are 82-92 tokens, all 8 drawn by the loopback
    service the check started on 127.0.0.1:50061 (`via` on every one).
  - The starts (dx m, dy m, dyaw deg): seed 1001 +2.50 +0.96 +15.6, 1002
    -1.91 +1.03 -16.7, 1003 -1.35 +1.63 +13.3.
  - The Shield is 5 s / 0.1 s with no FSM, at bf16, on every flight, all
    labelled AirSimNH.
  - The runner's own service command (`python -m guardrail.paraphraser_service
    serve --insecure --host 127.0.0.1 --port N`) was started once by hand: it
    listened after 1.3 s, and `--print-prompt --paraphraser` drew utterance
    pp-cae5531528dec80a from it.

#### Limits
- **Nothing has flown.** Not AirSimNH with these settings through
  `-settings=` (a vehicle that is not SimpleFlight is refused before the
  model loads), not the take-off check, not the move to a seed's start, not
  the contact count, not bf16 beside the renderer (the VRAM check is tested
  on stand-ins only; 15.6 GB is the smoke check's 15.13 GB plus an
  allowance, not a measurement beside AirSimNH), not the service started by
  the runner during a campaign.
- **The flights are classic AirSim, not a grant rail**: "AirSimNH (classic
  AirSim, simple_flight), not the grant's Project AirSim + ArduPilot rail,
  not KPI-grade". The Shield's forecast keeps its default velocity response,
  ArduCopter GUIDED's (fitted on SITL), not SimpleFlight's: the same in every
  arm, so it moves the absolute repair rates, not the on - off comparison's
  footing.
- **Three seeds are three starts, one flight each.** They are not repeats of
  one condition, so a per-seed delta mixes the CSP's effect with what that
  start shows the camera; the mean over seeds is over three such deltas.
- **The service the runner starts is plaintext on loopback.** `real_vla_demo`
  can reach it over mutual TLS (`--paraphraser-pki`), but the runner does not
  use the lab PKI.
- **Whether the measured decision rate is acceptable (G3) is the PI's.** The
  grant's observation path asks 10 Hz; a decision took 0.55-0.61 s at bf16
  in the smoke check, without the renderer.
- **Base OpenVLA was trained on robot-arm data and never on rule text.**
  Whether the CSP moves its actions at all is what the A/B measures.
- **The risk weights are untuned** (Decided, above). They only reorder
  sentences and choose which P1/P2 rules a budget drops. No shipped policy
  reaches the 256-token budget: 149 tokens at most, with the region filter
  off.
- docs/FINAL-REPORT-outline.md and tracker/ are not edited here.

### 2026-11-06 (IR 4.0.0) — `scope` required on every rule: WP1-01 closed the grant's way, all 154 shipped rules state `scope: mission`, and every stored document of an earlier era still reads and re-derives its recorded hash

#### Changed
- **IR schema 4.0.0 (WP1-01): `scope` is required** (`guardrail/models.py`,
  4ae620a), the last of the four ConstraintBase fields the grant writes
  without a default; `constraint_type`, `priority` and `violation_action`
  have been since 3.0.0 (2 Nov). It had been optional since 2026-10-06, an
  absent scope meaning "mission". A rule that leaves it out, or writes null,
  is refused by name at every DSL entry: a YAML/JSON policy, a REST rule
  body (422), a rule built in code, a lock recipe or a bundle marked 4.0.0.
  A field every writer must give is a breaking change, hence the major
  version, as 3.0.0 was. `lock --update` re-pinned the schema (11 pins
  `<absent>` -> `<required>`, `ir_schema.migrations` 3.1.0 -> 4.0.0 with its
  why), and `policies/policy_dsl.schema.json` requires `scope` with the
  grant's four values and no null. docs/IR-MIGRATIONS.md has the row.
- **The rule applied to choose each value: scope follows the layer that
  authored the rule.** A regulation- or site-layer rule applies to every
  mission flown under it (`global`), a mission-layer rule to its mission
  (`mission`), `segment` / `waypoint` only to a rule written for part of a
  mission - the pairing of the grant's worked example (its site and
  regulation rules global, its mission corridor mission). No shipped file
  writes `layer`, so all 154 rules are mission-layer rules written for their
  file's mission: every one states `scope: mission`, and none is narrowed.
  The GeoJSON importer keeps its defaults, `layer: site` with
  `scope: global`, by the same rule.
- **All 32 policies raised their patch version**: `fase3-sim-demo` 0.1.3 ->
  0.1.4, `wgs84-taipei-demo` 0.1.1 -> 0.1.2, the other 30 0.1.2 -> 0.1.3. The
  lock gained 32 entries (95 -> 127), each `migrated_from` the old one with
  `migration.old_hash_form` `sha256-canonical-v3/ir-3.x` (`bundle.IR3_FORM`):
  the file with every scope left out (`Policy.ir3_form`) is the old file,
  its hash, IR 2.x and legacy forms reproduced for all 32 and re-proved by
  `check_lock` on every run. `policy_candidates` walks the chain one step
  further (scope stated -> WGS84 -> metres at 3.0.0 -> metres at 2.x).
- **A stored document of an earlier era is read in its own era**
  (`models.era_context`): scope stays absent where it wrote none, so it
  hashes exactly as recorded, and `effective_scope` reads the absence as
  "mission". `check_bundle` takes the era from the manifest's
  `ir_schema_version`, `derive` from the recipe's, `read_replay` from the
  replay's policy manifest; a run's and a replay's `policy_g<N>.json` and
  the stored packs (`tools/profile_shield_tick.py`,
  `tools/own_velocity_closed_loop.py`, `tools/export_episodes.py`) record no
  version and are read as written: only a document from before 4.0.0 can
  leave scope out, and the hash it must reproduce is that document's.
- **A manifest records the schema its IR is a document of**
  (`Policy.ir_schema_version`, `bundle.manifest_for`; new lock entries too):
  4.0.0, or 3.1.0 for a policy of an earlier era bundled again (a replay of
  an old run), which is then read back in that era.
- **Every rule built in code states scope `mission`** (a zone, switch or
  swap applied during a mission belongs to it): the REST API's deprecated
  POST /nfz body, the Shield's switch (`set_rule_active`), the rails' and
  demos' zones, the generator (`demo/gen_random_scenario.py`), the benches,
  the training scenarios, the cold-start policy, the GeoJSON importer's
  2026-07 zone body. The harness carries a hot-applied zone's scope to the
  runner (`_CARRIED`, the spawn payload may hold it, `scenario_spec._ZONE_KEYS`;
  the runner writes "mission" where a payload has none) and the runner
  writes "mission" on its switches and swaps (`_RUNNER_WRITES`; another
  scope is refused as UNIMPLEMENTED, as for the other three fields). Three
  edits fall in files other agents are changing, each only what a test
  needs: `experiments/sweep_scenarios.py` (the runner's zone, switch and
  swap, 3 lines), `sitl/ros2_shield_node.py` (`DYNAMIC_FENCE`, built at
  import, and `zone_around`, 2 lines), `guardrail/shield.py`
  (`set_rule_active` and `_swap`'s comparison corridor; not the escalation
  code).
- The negative corpus states scope on its 40 rules, so each case is still
  refused for its own defect, and gains `missing_scope.yaml` (39 cases).
  `tests/test_railinputs_node.py`'s two REST bodies, which lacked IR 3.0.0's
  three fields (the file skips without fastapi/uvicorn), state all four.
- `tools/build_flight_bundles.py` signed the new versions with the lab key
  (gitignored build products, in the worktree): `fase3-sim-demo-v0.1.4`,
  `wgs84-taipei-demo-v0.1.2`, and `v0.1.3` of corridor-survey, follow-car,
  follow-car-nfz, follow-car-citylife, follow-car-citylife-nfz, -nfz3,
  -nfz3b, follow-pedestrian, vla-goto-citylife, sitl-pedestrian-standoff
  and poly-test (13). `deploy/orin/run_vla_fly.sh` defaults to
  `follow-car-citylife-nfz-v0.1.3`; the usage examples in
  `sitl/run_sitl_demo.py`, `sitl/gcs_backstop.py`, `sitl/bundle_origin.py`
  and `guardrail/bundle.py` name the new versions.
- **Regenerated from the clean tree:** `docs/data/wp1_roundtrip.json`
  (4edd5d1; the published file was the 20 Oct run over 29 policies): round
  trip 32/32, tamper 32/32, negative corpus 39/39, grant form 32/32, the
  reference's loader 3/3 both ways, both hashes 32/32, forged reference
  hashes 32/32 refused, cold start p95 581 ms in fresh processes against
  1 s, schema 32/32, 404/404 stored manifests resolve. `docs/data/csp_kpis.json`
  (52ac56c; was the 21 Oct run over 29 policies): P0 122/122 at 256 tokens
  (the naive cut too) and at 144 tokens, the largest budget where the naive
  cut drops one (120/122); null 0/122. No other docs/data file pins a policy
  hash of today's files: `wgs84_conversion.json` records the 3 Nov
  conversion's hashes, of the 0.1.2 versions, which the lock keeps.
- docs/FINAL-REPORT-outline.md: WP1-01 PARTIAL -> DONE (Appendix A, section
  7 closed without a waiver, section 4.1), the regenerated KPI rows;
  with the dev images' ARCH-11, ARCH-28 and X-08 (master 1e15312, merged):
  63 DONE, 66 PARTIAL, 6 DRIFT, 1 MISSING.
- Two files quoted the 20 Oct wp1_roundtrip.json (29 policies, 35 corpus
  cases); the full CI at edbbe4b failed on both. The public site's curated
  WP1 numbers (`docs/progress/_src/site.json`) now cite the regenerated file
  (32/32, 0/32, 39/39, 0/39), and the four pages that show them (en / zh
  index and wp1) were rebuilt with the site generator's own build; its build
  manifest is left as it was, since its CHANGELOG digests are the curator's
  record of what was read before a publish (docs/DESIGN-public-site.md, How
  to update, step 1). `docs/data/progress_oct.json`: its wp1 section is
  recomputed by the data step's own `wp1_facts` (32/32 round trips and
  tampers, 39/39 corpus, 127 lock entries); its other sections and the bench
  need the lab checkout's flight output and stay as the 4 Nov data step
  wrote them.

#### Measured (read-only, the lab checkout's gitignored stores, with this code)
- 36/36 bundles in its bundles/ open: 31 flight bundles written under IR
  schema 2.1.0 (8), 3.0.0 (10) and 3.1.0 (13), every signature verified, each
  under the hash its manifest records (24 current, 8 under their IR 2.x
  form), and 5 replay bundles; 171 rules, every scope absent as written.
- All 405 stored manifests resolve exactly as with the base code (master
  30667cc): 392 directly, to the same id@version and canonical bytes under
  the same hash form; 13 through their own generation chain, whose base the
  label now names as the old version of its file.

#### Tests
- New: test_every_rule_must_state_its_scope (a file, a raw document, a rule
  built in code, null, an unknown value, the era context's edges, the 11
  pins, the published schema, the corpus, every shipped rule) and
  test_a_document_from_before_ir_4_reads_and_hashes_as_stored (every tracked
  replay bundle - 8 with master's KPI evidence - a run's stored policy_g0
  that resolves to the lock's follow-car-citylife@0.1.0, a 3.1.0 bundle that
  reads under a 3.0.0 or no version and is refused as 4.0.0), and
  test_every_bundle_on_disk_reads_in_its_era (tests/test_policy_dsl_grant_form.py);
  the scope step of the lock chain (32 scope, 31 frame, 32 IR 2.x steps),
  the recipes of each era, the reference delta of an earlier era
  (tests/test_policy_hash.py); the scope migration as a changelog diff
  (tests/test_bundle.py); the four REST fields and POST /nfz's scope
  (tests/test_api.py).
- The full per-build CI in the worktree (`tools/ci_local.py`, vla-real,
  Python 3.10.20) at edbbe4b, with a second worktree's CI and a nightly
  sweep running beside it: 139/141 files, 3486/3634 tests, check_claims rc 0,
  smoke rc 0 (64 episodes, fail-safe trigger correctness 0.984127). The two
  failing files, test_build_public_site (30/47) and
  test_build_progress_oct_data (43/48), were the two quotes of the old
  wp1_roundtrip.json above, not load; after the fix, run alone:
  test_build_public_site 47/47, test_build_progress_oct_data 44/48 (4
  skips, lab artefacts), test_check_claims 22/22.
- In that run, test_policy_hash 33/34, test_policy_dsl_grant_form 58/69,
  test_bundle 58/58, test_csp 100/100, test_harness_service 59/59,
  test_replay 40/61, test_api 10/15, test_railinputs_node 13/16; the skips
  are fixtures a worktree lacks. With them read from the lab checkout:
  test_policy_dsl_grant_form 69/69 against its reference repository (at
  7e79fe0); with its 31 earlier flight bundles copied in, the on-disk test
  reads 44 bundles (IR 2.1.0 x8, 3.0.0 x10, 3.1.0 x13, 4.0.0 x13) and
  test_harnesstruth_bundles 12/12 flies its 2.1.0 bundle
  fase3-sim-demo-v0.1.1 with hot-applied zones; test_replay 60/61 with a
  copy of its retarget_demo2 run, test_policy_hash 34/34 with that run's
  manifest; test_api 15/15 and test_railinputs_node 16/16 in the Python 3.12
  env with fastapi and uvicorn.

#### Limits
- Scope is declared, not enforced differently: the Shield enforces every
  rule over the whole flight, so a `segment` or `waypoint` rule would be
  enforced more widely than written, never less. The layer merge compares
  no scope.
- A dynamic_nfz of an earlier era's document cannot be edited by the Shield
  (`_edit_zone`), nor a fence of one converted (`as_dynamic_nfz`): both
  re-validate it as today's and refuse the missing scope. No rail does
  either: the zones a flight edits are the ones it hot-applied.
- The bundles are build products: the lab checkout and the Orin need the
  new ones built (`tools/build_flight_bundles.py`) before a rail starts
  without `--bundle`, since the default follows the YAML's version.
- `docs/data/progress_oct.json`: only its wp1 section is recomputed here.
  Its data step needs the lab checkout's flight output and bench; run it
  there (test_build_progress_oct_data compares every section with the tree
  in the lab). The public site's build manifest likewise waits for its
  curator's next build.
- `sitl/ros2_shield_node.py`'s usage example still names
  `fase3-sim-demo-v0.1.3` (left for the agent changing that file). No
  tracker entry (tracker/data is local to the lab checkout).

### 2026-11-06 (perception VLA) — the ArduPilot perception rail can fly a VLA follow: AIOT-VLA as the pilot, on vla_fly's M3 parts, for the hil topology on the AGX Orin; not flown

#### Changed
- **`--pilot vla --mission follow`** on `demo/pas_ardupilot/perception_node.py`
  (refused until now). It is `demo/vla_fly.py`'s M3, imported, not copied:
  - The hint is `vla_mission.DetectorHint` with the sighting's memory
    (`--follow-memory-s`, 6 s), the search turn (`--follow-search-s`, 20 s)
    and the close rule (`--close-bottom-frac` 0.92, `--close-width-frac`
    0.25). Each request records its kind (fresh, memory, search).
  - The step tracker is `vla_fly.tracker_config` on vla_fly's own parse of
    this flight (`perception_node.vla_fly_follow`, the way
    `demo/vla_dagger.py` builds its follow): turn while moving, the altitude
    held at `--cruise-alt`, steps anchored at the answer, LAND held as
    arrival, the smoothing. `--vla-step-as-rate H` and `--vla-rate-yaw rate`
    fly AIOT-VLA's label (a step over 0.5 s, its yaw rate). The defaults and
    the refusals are vla_fly's.
  - A fresh close box holds the horizontal motion, inside the tracker when
    it smooths, as `vla_mission.MissionLoop` does. `VLAPilot` now reads the
    Grounder once per tick, so the close hold, the stop rules and the row see
    one box.
  - The stop rules get the request count. A follow that never asked ends
    `subject_not_acquired`, not `no_first_decision` (a VLA failure, exit 3).
    `--lost-timeout-s` defaults to vla_fly's per mission: 90 s for a follow,
    15 s otherwise.
- **`--vla-front-source detector`.** The FrontCamera subscription that feeds
  the detector also feeds the VLA's frame store, so the request's front image
  is the detector's frame (768x432, pitched -20 deg). VLAFront is then
  neither subscribed nor required, in the robot config or among an attached
  node's live topics. `front_source_check` takes the flag. It was hard-coded
  to VLAFront, which refused every AIOT-VLA from v3 on (they read the
  detector's camera).
- **`vla_host`.** The server's hello `host` goes into the run's hil evidence
  (and `MavrosLink.hil_evidence`), as `sitl/ros2_shield_node.py` records a
  VLA node's. A server on another host, or one that reports none, keeps the
  run from hil (`guardrail.manifest._vla_host_missing`). Before this a VLA
  run's evidence had no such key, which the manifest reads as "the VLA runs
  in the Shield's process".
- **The object words** are recorded as what they are to the model that
  answered (`vla_mission.object_words_record`): AIOT-VLA's grounding phrase,
  DroneCATS' and RaceVLA's command, AerialVLA's inert slot. This rail
  recorded every model's words as inert.
- The CSP mission says "follow a red car". A follow declares no goal, so no
  approach-radius score. `frac_within_30m_after_acquire` sits beside the
  whole-mission share, because this rail's mission clock starts at the start
  gate.
- **`metrics.json` `vla_observation`**, for the grant's 10 Hz observation
  (X-05, ARCH-25). It counts the distinct front-frame sim stamps among the
  requests actually sent (`vla_captures.jsonl`) and gives:
  - the rate per wall second, per sim second, and while the pilot was being
    asked (not held by the rail, not hint-gated);
  - the duplicate requests and the gaps between frames (p50, p95, max);
  - the hint-gated spans, with why;
  - FrontCamera's delivered rate.
  It never quotes the decision rate or the request rate. Each row says what
  the backend did about a request (`vla_request`).
- **`--front-capture-interval-s S`** renders FrontCamera's capture interval at
  scene render time (`rail.write_sim_configs` / `write_attach_configs`;
  `-FrontCaptureIntervalS` on `run_pas_ardupilot.ps1`'s scene and node
  steps). The default is the template's 0.1 s, unchanged. The value and where
  it came from are recorded (`sim_config.front_capture_interval_s`).
- **`deploy/orin/run_pas_perception.sh`** runs the node natively on the AGX
  Orin (host ROS 2 Jazzy from apt, `~/vla-drone-env`) on the companion
  MAVROS's ROS 2 graph, read from `deploy/topologies/hil.env`. It refuses
  `--dev-yaml` and `--start-sitl`.
- Docs: `docs/DESIGN-topologies.md` decision 2 (host ROS for this node only,
  and why), the hil section of `docs/DESIGN-projectairsim-ardupilot.md` (the
  layout and the commands, in order), `docs/DESIGN-vla-pilot.md` section 14,
  `deploy/README.md`.

#### Tests
- `tests/test_pas_vla.py` 37/38 (one skip: the map-dependent goal check).
  Nine new tests; with the changed refusal test, the ten that touch this
  change fail against master 28c61e7 and pass here.
- `tests/test_deploy_configs.py` 69/70 (one skip). The script is in the LF
  and shebang list. A dry-run test checks its graph against `hil.env`, the
  node's own parse of its command, its pass-through, its refusals, and that
  a relative G1 verdict path still names the same file.
- Every test file that imports the node, the rail, `vla_mission` or
  `vla_fly`, and the two that master 1e15312 touched (46 files), one at a
  time in the worktree after that merge: 1494/1543, no failure. The 49 skips
  are the lab's gitignored maps, flights and bundles, which a worktree does
  not have. Two other full CI runs and a nightly sweep shared the machine.
  Before the merge, one run of `test_railvideo_recording.py`'s wall-clock
  loop-rate comparison failed under that load (9.97 Hz without the recorder
  against 8.08 Hz with it). It passed twice run alone, and in the run above.
- `tools/check_claims.py`: 330/330 files clean. The full per-build list
  (`tools/ci_local.py`) was not run on this branch.

#### Limits
- **Not flown.** Nothing here has run against Unreal, ArduPilot, MAVROS, the
  Orin or the model. The tests use the rail's stand-ins and a scripted
  protocol-v2 server that announces an aiotvla identity.
- This rail's pose ring has no sim stamps, so each image is paired with the
  EKF pose at its arrival (the flag's help: 0.1-0.4 m and 3-9 deg off while
  flying pipelined). The run says so (`--allow-unstamped-pipelined`,
  `capture_time_basis`).
- Not ported from vla_fly's follow: the StuckWatch and CloseStall perception
  resets, the identity Reacquirer's re-acquisition after a lapse, and
  vla_fly's held-box subject feed. This rail feeds the Shield its own depth
  or width estimate, as before.
- A detector-front run still renders VLAFront (an unused 256x256 capture
  every 0.1 s).
- `deploy/orin/first_boot_check.sh`'s host-ROS WARN still says the rail never
  uses host ROS; on the AGX this node now does.
- `demo/vla_fly.py`'s own records still call every model's object words
  inert.

### 2026-11-06 (hil scenarios) — the stress library can drive the hil rail, one scenario per mission; not flown yet

Until this date the ROS 2 rail took a target and a seed but no start, no
pilot law and no scenario events (docs/RUNBOOK-orin-hil.md step 14), so the
hil campaign could only repeat the node's own mission, and the harness's
`--backend sitl` refused every scenario but `rail-mission-stub-vla`.

#### Added
- `experiments/sweep_scenarios.py --backend hil` (`--hil-config`,
  `--hil-dry-run`, `--hil-controls`, `--hil-campaign`): per scenario,
  `sitl/hil_desktop.sh` restarts SITL and mavlink-routerd in WSL at
  `SITL_HOME` (the policy frame origin's projection of the scenario's start,
  584 m, its start yaw), the scenario's events and record are copied to the
  Orin, MAVROS is restarted, one mission container flies, its run folder is
  fetched into `demo/out/hil_<profile>/`, accepted by its own records (the
  episode's `mission_end`, this launch's nonce in `metrics.json`
  `scenario`) and scored with the harness's own `score()` /
  `episode_verdict`; `scenario_score.json` is written beside the run.
- The rail's nodes take the scenario: `--start X,Y[,UP]` (the policy-frame
  position of home; every pose offset by it; the node refuses to arm unless
  the autopilot's home lies within 5 m of it; take-off settles at UP),
  `--target X,Y[,UP]`, `--no-target`, `--reach-m`, `--events` (the
  library's ScenarioEvents, applied in the tick with the harness's own code,
  now `guardrail/scenario_events.py`), `--scenario-meta`; the VLA stub
  flies `--pilot stub_vla | goto | constant` (`--action`), the laws now in
  `guardrail/vla_stub.py` and shared with the harness. The adapter projects
  the zones about the home the Shield node declares (`fence_frame`).
- `tools/kpi_report.py`: such a run is filed under
  `hil-stress/<template>/<cell>/shield-<arm>` and, with its
  `scenario_score.json`, re-scored as a stress bundle with its log is.

#### Changed
- One refusal for both rails (`rail_refusal`, replacing `sitl_refusal` /
  `_mission_mismatch`): refused are the stressors (wind, gusts, start
  jitter, GPS noise, latency and dropout, the battery hook), an RTL that
  must fail (`rtl_failed_ticks`), a non-finite pilot action, a relabel
  schedule, start altitudes below 1 m or at the GeoFence ceiling, and the
  paraphrase / prefix-off arms; Shield-off controls fly only with
  `--hil-controls`. Smoke profile: 64 episodes, 41 missions, 23 refused;
  nightly: 744 episodes, 546 missions, 198 refused (`--hil-dry-run`).
- `rail_measurements`: a step into an unsafe position or a zone is the
  aircraft's own only if it was so under the previous row's rules, zones
  and rule clock and no rule changed on the row (the node logs that
  judgement per row; older logs are rebuilt from their generations). A zone
  spawned onto the aircraft counted as the flight's breach until now.

#### Fixed
- The Shield node and the VLA stub parse with `allow_abbrev` off, as the
  VLA bridge already did: both get every argument, and the Shield node read
  the stub's `--action=...` as an abbreviation of `--action-timeout-s` and
  exited 2.

Not flown: the shakedown on the hil rig is the user's. Tests:
`tests/test_sweep.py`, `test_sitl_rails.py`, `test_mavlink_adapter.py`,
`test_kpi_report.py`, `test_harnesstruth_bundles.py`.

### 2026-11-06 (fail-safe) — a frozen fix now gets its fail-safe (X8), noise inside the margin no longer counts as a pilot thrashing, and every recovery flies at its cap; theta's first-deep-forecast false triggers remain

#### Found
- Fail-safe trigger correctness (WP3-15, >= 0.99, against
  docs/RUBRIC-failsafe.md) on master 23ac319, headless: library 175/179 =
  0.978 (2 false, 2 missed), smoke 62/63 = 0.984 (1 false), nightly 721/741 =
  0.973 (14 false, 6 missed). Three causes:
  - **C1, theta on the first deep forecast** (15 false: library 2, smoke 1,
    nightly 12). rail-mission-stub-vla (every seed) and
    standoff-approach-angles 7 m pedestrian (every variant and seed): the
    first forecast that meets the zone or ring is already more than theta
    deep, so X1 -> G3 holds the aircraft, the held request stays deep, and
    G5 sends it home at 30.1 s, while the rubric's flight without a
    fail-safe (the projection flown every tick) takes it round in 10.9 s /
    1.5 s of unsafe requests (N1).
  - **C2, a frozen fix the FSM could not see** (missed: library 2, nightly
    6). gps-dropout lane 23.6, windows 2-9 s and 6-13 s: the pose frozen
    for 7 s, the Shield passing 4 m/s, the aircraft ending 1.2 m from
    nfz-square with 28 m of error. The Shield had no fix-quality input.
  - **C3, noise inside the margin counted as thrash** (nightly 2 false:
    gps-noise-within-margin latency 0 / pass_y 23.2 / sigma 0.3, seeds 1001
    and 1003). 0.3 m of white noise flipped the forecast in and out of
    nfz-square's ring; every return was a new N-in-T onset, and G2 then G4
    sent the aircraft home on repairs of 0.02-0.43 m.
- Mean time to safe (WP3-17, reference <= 2.0 s): library 2.926 s over 39
  spells. After the 29 Oct climb cap, the lateral recoveries still flew the
  reference's laws: a fixed 2.0 m/s escape from a zone imposed on the
  aircraft, stand-off and corridor returns at depth / L floored at 0.5 m/s
  (radius-scaling-grow-deep 6.7 s, standoff-reclassified 3.5 s,
  corridor-swap-shifted 4.0 s).

#### Changed
- `guardrail/models.py`: `State.fix_ok` (is (x, y) a current, valid fix)
  and `State.pos_sigma_m` (the reported 1-sigma horizontal accuracy). Both
  optional; None means not reported, and the position is trusted exactly as
  before.
- `guardrail/fsm.py`: edge X8, Normal / Brake / Loiter -> RTL, "position
  knowledge lost", checked before every rule's own answer; the grant's G6
  lands the aircraft if the return fails. `TickInput.position_lost` and
  `TickInput.noise_marginal` (a clean tick within 3 sigma of a hard zone's
  ring keeps the violation "on" for N-in-T onset counting). No FSMConfig
  field was added: the default config still hashes `sha256:d55e1bfe...`.
- `guardrail/shield.py`:
  - `_position_verdict` is rubric E3 on the Shield's own estimate. While
    `fix_ok` is False it dead-reckons from the last valid fix on the
    commands it streamed, plus a drift allowance `dr_drift_mps` x the fix's
    age (0.5 m/s by default, `Shield(dr_drift_mps=...)`, recorded in every
    X8 reason and per harness episode). With `pos_sigma_m` reported the
    error is at least 3 sigma, judged at the position and along the
    request's forecast. X8 fires when the error exceeds a hard, enforced P0
    zone's margin with the zone's boundary inside it.
  - `recovery_law="cap_all"` is the default: GeofenceEscape from a static
    hard zone at the speed cap, judged as flown (out of the ring, then a
    stop), slower only where the cap's stop breaks another rule, reported to
    the FSM if none can be flown; StandoffRecover at the cap plus the
    subject's recession; CorridorReturn at the fastest inward speed, up to
    the cap, that holds short of the far wall. A moving zone keeps the floor.
    At a 4 m/s cap the recovery takes the whole speed: the pilot's
    along-corridor or tangential motion is not kept while it lasts (the
    "cap" law kept it). "cap" (29 Oct) and "lookahead" (before) stay
    selectable.
- `guardrail/kpi.py`: the per-tick false-trigger invariant leaves out X8
  ticks by their reason (the request on such a tick may be clean by
  design); it still holds for every request edge.
- `experiments/sweep_scenarios.py`: the modelled rail reports fix quality
  (`stress.fix_quality_reported`, default true: `fix_ok` False through a
  dropout window, `pos_sigma_m` = the noise sigma); bundle extras schema /5
  names `arm.dr_drift_mps`, and a /1-/4 bundle re-flies with the fix quality
  withheld, as it was flown; `--recovery-law` defaults to cap_all.
- `experiments/templates/sensor_degradation.yaml`: gps-noise-within-margin's
  nightly known-failure pins removed (the cells pass).
- Docs: docs/DESIGN-escalation-fsm.md (X8, its inputs, the per-rail signal
  table, the margin-as-accuracy-requirement note, the KPI invariant, the
  measurements), docs/DESIGN-shield-enforcement.md ("Every recovery at its
  cap"), docs/RUBRIC-failsafe.md (section 5 re-run; sections 4 and 6 note
  the new input). No clause and no label changed.

#### Fixed
Headless, the grant's 5 s forecast on the ideal vehicle, against master
23ac319; the rubric and every label unchanged (`tools/failsafe_rubric.py`:
every declared label agrees, library 168 cells, smoke 56, nightly 672
cell-seeds). Not KPI-grade.
- **Fail-safe trigger correctness**:

  | profile | before | after |
  |---|---|---|
  | library | 175/179 = 0.978 (2 false, 2 missed), Wilson low 0.944 | 177/179 = 0.989 (2 false, 0 missed), 0.960 |
  | smoke | 62/63 = 0.984 (1 false), 0.915 | 62/63 = 0.984 (1 false), 0.915 |
  | nightly | 721/741 = 0.973 (14 false, 6 missed), 0.959 | 729/741 = 0.984 (12 false, 0 missed), 0.972 |

  - C2: every missed trigger is a trigger now (X8 at 5.3 s and 7.6 s,
    home at 9.3 s and 13.3 s). The dropout cells that already triggered
    did so late, through the hold: they now return at 5.3-6.4 s instead of
    G5 at 39.1-45.0 s, and lane 23.6 window 4-11 s returns instead of
    landing (G6).
  - C3: both noise false triggers are gone.
  - gps-noise-beyond-margin (E3) returns on X8 at 2.5-4.7 s instead of
    N-in-T's G4 at 7.7-12.3 s, on every cell and seed.
- **Mean time to safe**: library 2.926 s over 39 spells -> 2.540 s over 35
  (max 6.7 -> 6.4 s); smoke 3.216 s / 19 -> 2.533 s / 15; nightly 2.813 s /
  165 -> 2.539 s / 153. Over the episodes that recover in both runs:
  2.944 -> 2.540 s, 3.312 -> 2.533 s, 2.819 -> 2.539 s. The spells that
  are gone are gps-dropout cells X8 returns before they get anywhere unsafe,
  and standoff-reclassified's second spell. Cells:
  radius-scaling-grow-deep 6.7 -> 3.4 s, standoff-reclassified 3.5 s (2
  spells) -> 0.9 s, corridor-swap-shifted 4.0 -> 0.6 s.
- **Safety**: P0 escape 0.0 on every profile; Shield-on episodes that flew
  into a P0 polygon 1 / 1 / 3 -> 0 (library / smoke / nightly), Shield-on
  episodes with a breach 2 / 2 / 6 -> 0; true-state P0 ticks 1768 -> 1474
  (library) and 5931 -> 5120 (nightly); the modelled autopilot's breaches
  10 -> 8 and 30 -> 24. That is the condition cap_all was made the default
  on; the sweeps' own gates: library 154 pass / 22 tracked / 0 fail / 4
  known, nightly 660 / 66 / 0 / 18 (two pins fewer).

#### Tests
- New `tests/test_position_quality.py` (14): X8 fires on the tick E3 first
  holds on the ideal vehicle (drift allowance 0) and never in the N2 cells,
  and the allowance never makes it later; fix_ok None or True changes
  nothing; a blind Shield (quality withheld) flies as before; the noise
  hysteresis; the sigma branch only when a rail reports sigma; cap_all is
  the default, its escape judged as flown where the straight 20 m forecast
  refused the cap, a slower retry where the cap's stop breaks another rule,
  a moving zone at the floor; the KPI invariant; bundle /5, a /4 bundle
  re-flown blind, the refusals.
- test_fsm 124/124 (X8 from Normal, Brake and Loiter; noise_marginal
  onsets), test_shield 86/86 and test_window_opens_on 22/22 (the old floors
  pinned to "cap", the cap_all numbers beside them), test_sweep 102/103 + 1
  skip (the SITL backend, not installed here; four fixtures fly with the fix
  quality withheld, the blind case they were written for, and the
  autopilot test on "cap"), test_hold_points 34/34,
  test_harnesstruth_response 19/19, test_own_velocity_forecast 28/28,
  test_scenario_spec 40/40. `.github/workflows/tests.yml` runs the new file.
- Found by the full per-build list and fixed: test_ir's index-equivalence
  tests and experiments/bench_shield_50rules.py's agreement check compared
  the default Shield with tests/legacy_fence_loop.py, whose zone escape is
  the pre-IR 2.0 m/s; they now ask the index question on recovery law
  "cap", the law the frozen copy records (the copy is unchanged and says
  why); the bench still times the shipped default. test_corridor flies its
  floored-return case on both laws. gps-dropout's description keeps the
  marker test_harnesstruth_rtl reads for the published sweep's blind LAND
  ticks. test_ir 20/20, test_corridor 25/25, test_harnesstruth_rtl 20/20.

#### Limits
- **C1's 15 false triggers remain** (library 2, smoke 1, nightly 12), and
  stay false triggers under the decided theta basis (each repair's forecast
  depth in metres) and E5 as written (judged on the flight without a
  fail-safe). No change to the FSM answers them under those two decisions;
  judging E5 on the flight the grant prescribes (projection abandoned, the
  aircraft held) would, and is a decision for the user, not made here.
- No flight rail reports `fix_ok` or `pos_sigma_m` yet, so X8 and the noise
  hysteresis are idle on every flight. The ROS 2 node (MAVROS
  /mavros/global_position/raw/fix, GPSRAW, EKF_STATUS_REPORT, the pose
  stamp's age) is to be wired after the rail rework in progress; the PAS
  ArduPilot rail already parses GPS_RAW_INT and EKF_STATUS_REPORT, for its
  pre-arm gate only; simple_flight hands a ground-truth pose and cannot
  report one.
- The autopilot losing its own fix (ArduPilot's EKF failsafe, LAND) is a
  second input, not modelled: the headless autopilot has a perfect position.
- `dr_drift_mps` 0.5 m/s is a judgement (an eighth of the 4 m/s cap); a
  rail that reports its own position variance should set it from that.
- Once a rail reports its accuracy, every hard P0 margin is an accuracy
  requirement: a 1 m margin certifies a zone only while 3 sigma <= 1 m
  (sigma <= 0.33 m, RTK class); with ordinary GNSS the Shield will return
  the aircraft near such zones. Sizing margins is a policy decision; no
  policy was changed.
- Mean time to safe stays above the reference 2.0 s: what is left is depth
  over the policies' caps (2 m/s climb, 4 m/s speed) on geometry imposed on
  the aircraft, plus wind beyond authority.
- Headless, not KPI-grade.

### 2026-11-06 (dev images) — the dev topology flown in containers for the first time; the same companion image definition on the desktop and the Orin

#### Flown
- `dev_compose_01`: deploy/compose/docker-compose.dev.yml on Docker Desktop,
  every service in a container built on the desktop: ArduCopter 4.5.7 SITL
  (`vlaguard/sitl:copter-4.5.7`, ae721bdc), mavlink-router, MAVROS 2 and the
  mission (`vlaguard/companion:jazzy` amd64, aad0b268). Code 30667cc (clean),
  the default signed bundle (fase3-sim-demo 0.1.3), topology `dev` from the
  evidence; P0 escape 0.0; refused as KPI evidence because a dev run needs
  the PI's written waiver, as the rule says. The same first-tick escalation
  as hil_smoke_03. Video `demo/out/dev_compose_01/dev_compose_01_track.mp4`.

#### Recorded
- `deploy/evidence/hil/images.txt`: the companion image built natively on
  the AGX Orin (arm64, 8938f2d5) and on the desktop (amd64, aad0b268) from the
  same Dockerfile, and the desktop's SITL image. Both companion builds carry
  the same OS, ROS distribution, Python 3.12.3, mavlink-router and pip set;
  only the Debian build stamps of the architectures differ
  (`deploy/evidence/dev/companion_versions_desktop.txt` against
  `deploy/evidence/hil/companion_versions.txt`).
- The desktop's first build attempt of the companion image failed on
  Docker Desktop's DNS (auth.docker.io: no such host); it resolved on retry.
- docs/FINAL-REPORT-outline.md: ARCH-11, ARCH-28 and X-08 to DONE (flight is
  the same image with flight.env, waiting on ARCH-14's waiver). 62 DONE,
  67 PARTIAL, 6 DRIFT, 1 MISSING.

### 2026-11-06 (contract) — ten more items closed on the hil evidence and by small conformance fixes; the per-build CI on Python 3.12 finds a cross-version determinism bug

#### Changed
- WP3-05: the six older demo scripts (aerialvla_demo, aerialvla_pas_demo,
  real_vla_demo, run_demo, semantic_demo, semantic_seek) forecast with
  guardrail.shield's LOOKAHEAD_S / LOOKAHEAD_DT_S (5 s at 0.1 s) instead of
  3 s at 0.5 s (8d1a0f3).
- WP4-13: `rosbags` 0.10.11 and pyarrow are installed in vla-real, so
  tests/test_wp4outputs_exporters.py runs 38/38 with no skip. On the KPI-grade
  hil run hil_smoke_03 the exporter writes 303 steps as JSONL and Parquet, with
  action.executed read from the bag (/mavros/setpoint_raw/local) on 303/303.
- WP1-18, WP1-20: the tracked store bundles/ keeps evidence, not only the four
  legacy replays: hil_smoke_03's bundle (signed on the Orin) and signed copies
  of the AIOT-VLA city flights aiot_v10bs_01-03 (flown from the clean master
  55faad0; each keeps its flown policy hash under its IR 2.x form).
  tests/test_replay.py checks the legacy four as before and every other
  tracked bundle for a current hash form and a trusted signature.
- WP1-19: the WP1 cold start measured on the Orin, in the companion image
  (tools/wp1_roundtrip_kpi.py --no-runs --no-reference): a signed 50-rule
  bundle to a ready Shield in fresh processes, p95 590 ms against 1 s (whole
  process wall p95 867 ms); tampered, untrusted and badly signed bundles
  refused through the same path. `deploy/evidence/hil/wp1_roundtrip_agx.json`.
- AIOT-VLA v10b served natively on the Orin (`vla_server_foreign.py serve
  --model aiotvla --horizon-s 0.5 --front-source detector`), timed on 300
  frames of aiot_v10br_01 with tools/bench_foreign_vla.py: model p50 32.6 ms,
  p95 33.9 ms, round trip p50 39.6 ms
  (`deploy/evidence/hil/aiotvla_v10b_bench_agx.json`). The server alone, not
  a flight.
- docs/FINAL-REPORT-outline.md: WP1-18, WP1-19, WP1-20, WP3-05, WP3-18,
  WP4-13, ARCH-12, ARCH-13, ARCH-26 and X-07 to DONE (the Rust decision:
  no Rust hot path); WP3-13 keeps PARTIAL until a hil campaign logs the
  VLA-to-setpoint latency over hundreds of ticks. 59 DONE, 70 PARTIAL, 6 DRIFT,
  1 MISSING.

#### Found
- **The per-build CI on Python 3.12** (`tools/ci_local.py --python` a new env
  with requirements.txt and the 3.11-extras job's packages, numpy 2.4.6),
  at ff0ee43: 137/141 files, 3557/3628 tests, check_claims rc 0, smoke rc 0
  (fail-safe 0.984127, as on 3.10). The four failing files:
  - test_policy_dsl_grant_form: Python 3.12 made sum() of floats compensated
    (gh-100425). The frame origin of the reference demo came out
    121.5314 on 3.12 and 121.53139999999999 on 3.10, so the desktop (3.10) and
    the Orin (3.12) projected the same policy to different metres.
  - test_replay: the tracked-bundle test allowed only the four legacy bundles
    (the new hil bundle).
  - test_render_track_video: the refusal test errored on `import matplotlib`
    before reaching the refusal.
  - test_railinputs_node: two REST payloads lack IR 3.0.0's required
    constraint_type / priority / violation_action. The lab CI on 3.10 skips
    the file (no uvicorn), so it was never seen.

#### Fixed
- 28c61e7: guardrail.projection's mean adds left to right on every Python,
  the reference's arithmetic on 3.10 and 3.12 alike; test_replay knows the
  tracked KPI bundles.
- c8e2b22: tools/render_track_video.py refuses a run with no policy before
  importing the drawing stack.

#### Tests
- test_policy_dsl_grant_form 66/66 and test_replay 61/61 on both 3.10 and
  3.12; test_render_track_video 6/6 (3.10), 5/6 + 1 skip (3.12, no
  matplotlib); test_wp4outputs_exporters 38/38; the six demo scripts' tests
  (24 files) pass.

#### Limits
- test_railinputs_node's fixtures are fixed with the IR 4.0.0 scope change
  (in progress), not here.
- The two other centroid sums (guardrail/ir.py moving-zone bound,
  models.ring_at) still use sum(): runtime geometry, not identity; a
  nanometre-level difference between hosts remains possible there.

### 2026-11-06 (hil) — the hil topology flown for the first time: the Shield and MAVROS 2 on the AGX Orin, ArduPilot SITL on the desktop; the project's first KPI-grade run, and the Orin profile within the grant's budgets

#### Flown
The runbook's layout (docs/RUNBOOK-orin-hil.md): ArduCopter 4.5.7 SITL and
mavlink-routerd in WSL on the desktop, the router's MAVROS endpoint
10.42.0.3:14555 over the direct cable; MAVROS 2, the Shield node, the VLA stub
and the MAVLink adapter in the companion container on the AGX Orin 64 GB
(JetPack 7.2.1, L4T R39.2.1, MAXN). The mission is the node's own (the stub
to the northeast pad, `fase3-sim-demo` 0.1.3, CMAC home).

| run | code | label | what it showed |
|---|---|---|---|
| `hil_smoke_01` | 890b723 | - | refused before take-off: the signed bundles are build artefacts (gitignored), and the Orin's checkout had none. The 31 signed policy bundles were copied over (hashes identical). |
| `hil_smoke_02` | 890b723 | `hil`, KPI-grade no | flown; the run bundle was unsigned: the lab key lives on the desktop. |
| `hil_smoke_03` | d0e91bb | `hil`, **KPI-grade yes** | the first run any gate has accepted: six-field manifest (sim_speedup 1.0 read back from SITL, clean revision), `hil_evidence` (Orin named by its device tree, R39.2.1, MAVROS on the Shield's host, no SITL there), run bundle signed by the Orin's key, rosbag included. Re-checked on the desktop: `tools/kpi_report.py` counts 1 KPI-grade episode of 2. |

#### Measured on the Orin
- `tools/profile_shield_tick.py profile --topology hil --cpus 1-3`, the 1,301
  real flight ticks at 50 active rules and a 5 s / 0.1 s forecast (51 poses),
  MAXN, clocks not locked (schedutil): `_check` p99 0.977 ms (max 1.047 ms,
  0 ticks over) against 5 ms; `filter` p99 22.647 ms (max 25.518 ms, 0 over)
  against 100 ms. Host-speed reference steady (51.0 / 55.8 ms).
  `deploy/evidence/hil/shield_tick_profile.json`; the KPI report's profile
  table now lists it as the only Orin row, within budget.
- `experiments/bench_shield_50rules.py --horizon 5 --dt 0.1` in the same
  container: `_check` p99 0.295 / 0.605 / 0.539 ms (clear / grid / near),
  `filter` p99 1.476 / 4.967 / 5.873 ms; lifecycle at 50 rules: Shield
  construction p99 17.5 ms (the IR part of the 1 s cold start), hot-apply
  p99 3.86 ms against 50 ms. `deploy/evidence/hil/bench_shield_50rules_5s_agx.json`.
- The image as the Orin built it: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12.3
  for rclpy and the Shield, MAVROS 2.16.0, Cyclone DDS, rosbag2 MCAP,
  mavlink-router v4 (`deploy/evidence/hil/companion_versions.txt`); the host
  as the containers read it (`host_facts.json`: is_orin true, MAXN by name).
- Clocks: the Orin is NTP-synchronised (systemd-timesyncd); its UTC matched
  the desktop's to within the ssh round trip.

#### Found
- **The first KPI-grade run shows the first-tick escalation.** At t = 0.10 s
  the FSM went Normal -> Brake (X1): "nfz-square: repair over theta, lat
  8.14 m vs theta 2.00 m; projection abandoned", Loiter at 0.2 s, RTL at the
  30 s loiter timeout, home reached at 30.3 s. P0 escape 0.0, but the target
  was never approached: the false trigger WP3-15 and ARCH-16 record, now on
  the canonical rail.
- mavlink-router's own unit tests do not compile on the Jazzy base (gtest.h
  302); Docker's default AppArmor profile denies /sys/firmware/** inside a
  container even with systempaths=unconfined, so the October design never
  let a container read the board name on an AppArmor host.
- Three bag topics stayed empty on this rail: /mavros/altitude,
  /mavros/extended_state, /mavros/setpoint_raw/target_local.
- The desktop's own build of the same image failed before its first layer:
  Docker Desktop's VM could not resolve auth.docker.io (no such host).

#### Fixed
- ba18657: the companion image builds only the mavlink-routerd target.
- d2ddab3: `deploy/topologies/hil.env` carries the cable's addresses (10.42.0.1
  desktop, 10.42.0.3 Orin) and says the hil machine is the AGX Orin.
- 890b723: the evidence services mount the host's board-name file read-only at
  /run/vlaguard/host/device-tree-model (read after the device tree's own
  paths) and lift no confinement; guardrail.manifest, the compose file, the
  runbook, DESIGN-topologies and the tests follow.
- d0e91bb: the Orin's own lab-dev key (generated there; only the public half
  is in policies/keys/trusted_signers.json); /etc/nvpmodel.conf mounted so the
  power mode is recorded by name.

#### Tests
- test_deploy_configs 69/69, test_manifest 68/68, test_profile_shield_tick
  43/43, test_bundle 57/57.

#### Limits
- One mission, the node's own, repeated; the scenario library does not drive
  this rail yet (the node takes a target and a seed, no start and no pilot).
- MAXN with the clocks not locked; the runbook's MODE_50W + jetson_clocks
  session needs sudo on the Orin.
- The image is built on the Orin only; the desktop build waits on Docker
  Desktop's DNS.

### 2026-11-06 — the server-kill test reads the hover from the flight's own records: the lab run's one failure at 61a24ab was the same hover authored `none`, not a short one

#### Found
- **The lab run at 61a24ab** (`docs/data/ci_local/61a24ab455ec.json`: 141
  files, 3597 of 3619 tests, one file failed) failed
  `test_a_server_killed_mid_mission_hovers_then_lands_and_exits_3` with
  `{('none', 'link_lost')}`: all of the last 25 rows. **There were enough
  hover rows; they were labelled `none`.** The test kills the stand-in server 4 s after its
  hello. The mission starts 3.6-3.8 s after the hello on a quiet machine
  (three warm-ups to 1.1-1.3 s, the climb to 7 m 2.0 s, the turn and its
  0.5 s) and 3.9-4.1 s on a loaded one, and the stand-in's first answer is
  accepted 0.4 s into the mission. A kill before that answer leaves the
  tracker no decision to drop: `vla_link.StepTracker._no_answer` sets the
  `hover` fallback only once a decision has been accepted, and before one
  the same zero command is authored `none`/link_lost. Every failing run
  ended on 31-32 such rows (3.0-3.1 s, the dead timeout). They are not
  landing rows (flight_log.jsonl holds mission ticks only), and they are
  not rows before the hover: they are the hover.
- Reproduced with the old assertion: 2 of 9 runs on a quiet machine, 5 of 7
  with 28 busy processes beside it (100 % CPU). In one of those the server
  died 0.01 s before the mission began.
- **Not a vla_fly bug.** The monitor's dead rule counts `none` and `hover`
  alike (`vla_mission.MissionMonitor.update`); `tests/test_vla_backends.py`
  and `tests/test_vla_fly.py` pin `none`/link_lost before any decision.
  The aircraft flew zero on every such tick, ended server_dead, landed and
  exited 3. `demo/vla_fly.py` is unchanged.

#### Changed
- `tests/test_vla_fly_fakesim.py`: the test no longer reads a fixed count
  of trailing rows. From the flight's own records:
  - one `server_exited` event, and `end` (server_dead) the last of the
    monitor's events;
  - the rows' last author run is the hover: `hover` if anything was flown
    before it, `none` if not;
  - it is the only link_lost run, and it starts no earlier than the tick
    before the one where the monitor saw the exit;
  - every tick of it flies a zero command over a dead link (link or
    protocol), for the dead timeout read from metrics;
  - then a controlled landing, no abort, and exit 3.

#### Tests
- Shown failing first. Five mutations make a run that never hovers; each
  still exits 3 with server_dead, and each fails the new test, four of its
  checks between them. The kill was forced at 2.0 s (before the first
  answer) and at 6.0 s (mid-step); the last mutation only at 6.0 s, since
  before the first answer there is nothing to forget.
  - The monitor ends server_dead on the tick it sees the server gone, or a
    tracker flies on past the link loss and the mission ends 3 s after the
    exit: the last run is not link_lost.
  - The mission ends on the first link_lost tick: a hover of 1 tick, short
    of the 3 s.
  - The "hover" commands 1 m/s: flown vx 1.0. The old assertion passed this
    one at 6.0 s.
  - The tracker forgets it had flown: `none` after a flown step.
- The code as it is passes both branches: at 2.0 s, `none`/link_lost x31;
  at 6.0 s, `hover`/link_lost x31. It also passes 8 of 8 runs at the
  test's own 4.0 s (4 quiet, 4 loaded); the old assertion failed 2 of those
  8. The whole file passes 14/14.

### 2026-11-06 (mission manifest) — the mission-style ROS 2 node writes the grant's six-field manifest; one module gathers the MAVROS evidence for both ROS 2 nodes

#### Changed
- **`sitl/ros2_mission_node.py` writes `manifest.json`** beside
  `metrics.json` (WP4-12), as every other rail does. Its run folders until
  now (`mission_wp_20261103d`, `mission_wp_20261104`) carry no code_revision,
  vla_model_hash, policy_hash, random_seed, sim_speedup or topology; the
  revision of the 4 Nov re-flight is known only from the flight driver's log.
  The six fields:
  - `vla_model_hash`: `none:hand-written-controller`
    (`guardrail.manifest.NO_VLA`): no VLA flies the mission style.
  - `policy_hash`: the hash in force at the end, the last generation after a
    hot-apply, as the GUIDED Shield node writes it. It is the hash the replay
    pack binds the bundle to: `python -m guardrail.replay pack` takes the last
    generation's policy and refuses a manifest naming another. metrics.json
    keeps both hashes: `policy_hash_at_takeoff` (now the hash when take-off
    was accepted, beside `generation_at_takeoff`; until now it was the loaded
    hash) and `policy_hash_final`.
  - `random_seed`: the new `--seed` (default 0, as the Shield node's).
    Nothing in the mission style draws a random number.
  - `sim_speedup`: SIM_SPEEDUP read from the autopilot over MAVROS's
    parameter service at the end of bring-up, never a literal; `unresolved`
    when it never answers. It is read before the tick timer starts, because
    `_finish` runs from that timer and a service call there never completes.
  - `topology`: the label the evidence supports (`detect_topology`). When the
    evidence does not hold, the rail's own label is stamped and the refusal
    is recorded in metrics `topology_refused`, as on the Shield node.

  metrics.json gains `topology`, `seed` and `hil_evidence`: the MAVROS
  chain, the autopilot's AUTOPILOT_VERSION, MAVROS's link and the host. The
  host evidence was metrics `host` before.
- **`sitl/mavros_evidence.py`** (new) holds the evidence gathering, factored
  out of `sitl/ros2_shield_node.py`. It is ROS-free:
  - the initial evidence: ROS distro, the MAVROS chain's fields, the host,
    SROS 2 when on, and `vla_host` only for a node flown with a separate VLA
    node;
  - the /mavros/state note and the GetParameters decoding;
  - SIM_SPEEDUP polling, with the hardware-autopilot reading;
  - AUTOPILOT_VERSION, with the setup banner as a labelled fallback;
  - the link, and the MAVLink processes on this host;
  - the manifest, with its topology fallback.

  Both nodes keep their method names (`read_sim_speedup`,
  `read_autopilot_version`, `read_link`). They are now wrappers that hand the
  readers the node's own service calls, spin and clock. The Shield node makes
  the same calls with the same timeouts and writes the same records.

#### Tests
- `tests/test_sitl_rails.py` 57/57 (5 new):
  - the readers on canned MAVROS answers: the refill race (0.0, then 1.0), no
    answer (None, never 1.0), a hardware autopilot, AUTOPILOT_VERSION, the
    banner fallback, both fcu_url sources;
  - both nodes use the module and keep no copy of it;
  - the mission node takes the Shield node's `--seed`;
  - a mission refused before MAVROS connected still writes its manifest: the
    rail's label, `topology_refused`, sim_speedup `unresolved`;
  - the real MissionShieldNode on the stand-in ROS stack of
    `tests/test_railinputs_node.py`. It is brought up against a stand-in
    MAVROS, flown under a zone hot-applied after take-off, and ended.
    manifest.json holds the six fields in the grant's order: NO_VLA, seed 7,
    sim_speedup 1.0 after MAVROS's 0.0, topology dev (also in metrics), and
    the final hash. The final hash is the last generation's and differs from
    the take-off hash in metrics. The replay bundle binds by manifest, not
    "unverified".

  On the 0029a43 node the stand-in test fails with "the mission node wrote no
  manifest.json". The pure tests fail there too: there is no
  `write_run_records` and no `--seed`.
- Two source checks follow the moved code. In test_sitl_rails, the
  autopilot-version decoder is now in `mavros_evidence.py`, and both nodes
  ask VehicleInfoGet. In test_ros2_package 17/17, "SROS 2 only when on" is
  checked through `initial_evidence`, for both nodes.
- test_mavlink_adapter 38/38, test_ros2_vla_bridge 47/47,
  test_railinputs_manifest 6/6. test_railinputs_node 13/16 (3 SKIP: no
  uvicorn, as in the lab run). test_manifest 67/68 and test_deploy_configs
  68/69 (1 SKIP each: no reference checkout in a worktree).
- The whole list (`tools/ci_local.py`), each run alone on the machine: in
  the worktree at 26ba907, 141/141 files, 3491/3630 tests (the rest SKIP:
  lab-only data); in the main checkout at b91dbd8, 141/141 files, 3609/3630
  tests. Both: the claim scan rc 0 and the smoke profile rc 0 (64 episodes,
  fail-safe trigger correctness 0.984). Records:
  `docs/data/ci_local/26ba9074fdf1.json`, `docs/data/ci_local/b91dbd838dc6.json`.

#### Flown
- **`mission_wp_20261106b`**, from the main checkout at a3c36a5 in WSL:
  the 4 Nov run `mission_wp_20261104` flown again (STYLE=mission,
  GCS_BACKSTOP=1, the bundle fase3-sim-demo 0.1.3, waypoints (0, 30) then
  (30, 30), 6 m/s asked; the harness spawned `nfz-hot-1` across the first
  leg at 1.5 s and switched it off at 12 s). Driver:
  `demo/out/_drivers/fly_mission_1106.sh`. Replay video:
  `demo/out/mission_wp_20261106b/mission_wp_20261106b_track.mp4` (575 KB,
  28 s).
- Its `manifest.json`: code_revision `a3c36a51e28c` (a clean tree, no
  "-dirty"), vla_model_hash `none:hand-written-controller`, policy_hash the
  final generation's (generation 2, `sha256:eeccc182...`), random_seed 0,
  sim_speedup 1.0, topology `dev`. metrics.json: topology `dev` with no
  refusal; `policy_hash_at_takeoff` `sha256:1a777a0f...` (generation 0, the
  bundle's) beside the final hash. The evidence names ArduCopter V4.5.7
  (2a3dc4b7) from AUTOPILOT_VERSION, autopilot_kind `sitl`, MAVROS's link
  `udp://:14555@` (the launch script's), and the local SITL, MAVROS and
  mavlink-routerd processes.
- The evidence reads cost no flight time: the autopilot's version was read
  8 ms after the hot-apply listener came up, SIM_SPEEDUP 4 ms after the
  mission's read-back. The end-of-run records, the git revision included,
  took 1.1 s after the LAND request.
- The replay bundle now binds its policy by the manifest
  (`policy_binding: manifest`, generations 0-2, bag included);
  `mission_wp_20261104`'s says `unverified`.
- The flight itself as on 4 Nov:
  - item 2 replaced by a held point at (0.0, 15.2) 90 ms after the zone's
    hot-apply (98 ms on 4 Nov): "leg to (0, 30) breaks nfz-hot-1 at
    16.8 m";
  - while generation 1 was in force (105 ticks) the aircraft stayed at
    y <= 15.39 m; the zone's margin begins at 17 m;
  - item 2 written back after 2.1 s of a legal leg, both updates confirmed
    by read-back;
  - item 2 reached at 20.6 s, mission end at 28.0 s (28.1 s on 4 Nov);
  - no tick with the position illegal; the speed was clamped from 6 to the
    policy's 4 m/s.

#### Limits
- Not KPI-grade: the `dev` topology, desktop SITL. The mission style still
  writes no kpi.json and no Shield audit, and runs no escalation FSM.
- `demo/pas_ardupilot/mavros_link.py` (the Project AirSim + ArduPilot rail)
  still keeps its own copy of these MAVROS readers.

### 2026-11-04 (re-flown) — the 3 November ROS 2 rail runs flown again from the main checkout, through Shield.filter_body; the re-flights found three WGS84 regressions, fixed

#### Flown
Every run from the main checkout in WSL (ArduCopter 4.5.7 SITL, MAVROS 2,
the `dev` topology), each with its flight-log replay video
`demo/out/<tag>/<tag>_track.mp4` (424-784 KB). The Shield node hands
/vla/action to `Shield.filter_body` (7e0cb41).

| run | as | what it showed |
|---|---|---|
| `gcs_fence_on_20261104` | `gcs_fence_on_20261103c`: GCS_BACKSTOP=1, Shield on, to (40, 15) at 2 m/s | Code 7e0cb41. The Shield bent the route around `nfz-square`: G1 at 4.6 s, G7 at 12.5 s (12.7 s on 3 Nov); 59 P0 violation ticks repaired, P0 escape 0.0, no unsafe tick, no breach. |
| `gcs_fence_off_20261104` | `gcs_fence_off_20261103b`: Shield off, 3.5 m/s | Code 7e0cb41. "Fence Breached" at 4.07 s (4.08 s on 3 Nov), GUIDED -> RTL; P0 escape 1.0, 104 unsafe ticks: the GCS-set fence is the only backstop. |
| `sros2_policy_on_20261104` | `sros2_policy_on_20261103b`: SROS2=1, per-topic permissions, Shield on, 3.5 m/s | Code 7e0cb41. `permissions_scope: per_topic`, the Shield in its own enclave; X1 / G3 at 0.11 / 0.21 s, G5 RTL at 30.3 s, as on 3 Nov; P0 escape 0.0. |
| `harness_corridor_20261104` | `harness_corridor_20261103c`: the harness over gRPC/mTLS, four HotApply | Code 7e0cb41. The same FSM as on 3 Nov (G1 2.9 s, G3 3.5 s, G8 11.2 s, X1 / G3 15.1 / 15.2 s, G8 28.2 s), target reached at 37.3 s, P0 escape 0.0 - but every GeoFence upload refused (Found). |
| `harness_corridor_20261104b` | the same, after d2f52b6 | Code d2f52b6. SITL booted at the bundle's origin; generation 1 uploaded `nfz-hot-1` (4 items, "Fence Indexed OK"), generations 2-4 export nothing for the reasons the 3 Nov run gave; the same FSM; target 37.4 s; P0 escape 0.0. |
| `mission_wp_20261104` | `mission_wp_20261103d`: STYLE=mission, the harness spawning the zone | Flown at 7e0cb41 (the driver's log; see Found). Item 2 replaced by a held point 98 ms after the zone's hot-apply ("leg to (0, 30) breaks nfz-hot-1 at 16.81 m"), restored after 2.1 s of a legal leg, item 2 reached at 20.6 s, mission end at 28.1 s: as on 3 Nov. |

#### Found
- **A geographic bundle flew about the wrong home.** The 3 Nov conversion
  anchored corridor-survey 0.1.2 at the Project AirSim scenes'
  home-geo-point (35.6895, 139.6917); the SITL rail booted at CMAC, 7955798.7
  m away. The Shield flew the geometry about CMAC, and the adapter refused
  every GeoFence upload ("the zones would be uploaded in the wrong place"):
  the autopilot backstop was absent. 28 of the 32 policies carry that origin.
- **The ROS 2 rail's replay videos could not be drawn**: a policy dump holds a
  WGS84 point as lat/lon only, and `tools/render_track_video.py` read x/y:
  KeyError: 'x' on all six runs. The 28 Oct deck's data step read the
  WGS84 nfz3b file the same way (zone counts and map).
- `sitl/start_sitl.sh` assigned the fence parm over `SITL_EXTRA_DEFAULTS`,
  so PHYSICS=gazebo with the fence on booted without gazebo-iris.parm.
- **The mission-style node writes no six-field manifest**: its run folder
  carries no code revision or topology. Its revision here comes from the
  flight driver's log (7e0cb41, a clean tree), not from the run.

#### Fixed
- d2f52b6: `start_sitl.sh` takes SITL_HOME (CMAC by default, as the
  container's entrypoint); `run_ros2_demo.sh` boots SITL at a geographic
  bundle's frame origin (`sitl/bundle_origin.py`) unless SITL_HOME is set;
  the fence parm is appended, not assigned.
- 5e34965, a0ac6f3, e6900e3: `guardrail/projection.py` gains
  `raw_frame_origin` and `raw_point_metres` (x/y as written, else lat/lon
  through `local_metres`, the Shield's view); the renderer and the 28 Oct
  deck's zone counts and map use them. Drawn into a temporary folder, the
  map's three zones sit where the city and both tracks place them.
- `docs/FINAL-REPORT-outline.md`: WP3-04 records the flights with
  `filter_body`; the rows that cited the 3 Nov runs name their re-flights.

#### Tests
- test_deploy_configs 69/69 (the SITL home and the defaults, the latter
  shown failing on the old line), test_render_track_video 6/6,
  test_build_progress_1028_data 4/4 (the same square in lat/lon gives the
  metre square's counts; the shipped nfz3b file loads), test_frame_contract
  16/16 and test_sitl_rails 52/52 in the lab checkout.

#### Limits
- Not KPI-grade: the `dev` topology, desktop SITL.
- The mission-style manifest is open (above).

### 2026-11-04 (body frame) — the Shield takes the grant's body frame at its own boundary: WP3-04 and ARCH-04 DRIFT -> PARTIAL; 49 DONE, 80 PARTIAL, 6 DRIFT, 1 MISSING

#### Changed
- **`Shield.filter_body(state, body)`** takes the action in the grant's body
  frame (vx forward, vy right, vz up, yaw rate in rad/s) and returns the
  emitted action in the same frame beside the tick's decision. It turns the
  action into the world frame at the state's heading (`frames.from_body`),
  runs `filter`, and turns the emitted action back (`frames.to_body`). The
  repairs are geometric and rotate with the action and the scene, so this is
  the body-frame repair, computed against the rules' world geometry without
  rotating that geometry on every tick.
- **The ROS 2 Shield node turns nothing.** `shield_step` and the
  pre-mission warm-up call `filter_body`; until now the node made the two
  turns around `Shield.filter()`. The body -> local-NED conversion stays the
  MAVLink adapter's alone (`sitl/mavlink_adapter_node.py`). The behaviour is
  the same: the same rotations at the same heading (test_sitl_rails 52/52).
- `docs/DESIGN-ros2-interface.md` (the frame chain) and `guardrail/frames.py`
  (who calls it) say so.
- `docs/FINAL-REPORT-outline.md`: WP3-04 and ARCH-04 DRIFT -> PARTIAL. The
  map: 49 DONE, 80 PARTIAL, 6 DRIFT, 1 MISSING.

#### Tests
- `tests/test_frame_contract.py`: `filter_body` equals `filter` between the
  two exact rotations on 60 random states and actions, to the bit; a
  body-forward action at heading East is checked against the zone to the
  East, and the same numbers read as a world-frame action are not. Shown
  failing with the heading taken as 0 inside `filter_body`: the first random
  case (heading 86.6 degrees) parts and the zone test finds no violation. The
  docstring test now requires the Shield, not the node, among frames.py's
  users. In the worktree 14 pass and 2 SKIP (no reference checkout).
- test_shield 86/86, test_shieldcore_audit 16/16, test_mavlink_adapter 38/38,
  test_ros2_package 17/17, test_sitl_rails 52/52.

#### Limits
- The dev rails (the city follow, the pymavlink SITL rail, PAS ArduPilot,
  the headless harness) still hand the Shield world-frame actions.
- The forecast holds the action's world direction from t = 0, as before: a
  body command with a yaw rate is not forecast along the arc it would fly.
- Not yet flown on the ROS 2 rail with this code.

### 2026-11-04 (decks) — the two 14 October decks stop failing the lab run: the explainer reads the data of its date, the October data step runs again, and a sent pack is checked against its record

#### Fixed
- **test_explainer_deck, 10/24 -> 24/24.** The deck states "Status as of
  14 October 2026". From 22 to 29 Oct its builder followed three library
  republishes, so a rebuild would have printed 29 Oct numbers under that
  line, and the 3 Nov republish made the build refuse its own numbers.
  `tools/deck/build_explainer_deck.js` now reads `scenario_sweep.json` and
  `progress_oct.json` as committed with the pack (335ec58, `git show`), and
  its library facts and the sentences that print them are those of that
  build. A rebuild gives the pack's `_build/slides_text.json` word for word,
  144 of 144 facts verified. The two later fixes that are not numbers stay
  (the `shield_cv` phrase, the 19 Oct comment). Current numbers are in
  current documents: the outline's KPI table and the dashboard.
  - `facts.json` names a pinned source "<file> (git <rev>)", and the
    mutation test copies that version; an `EXPLAINER_REPO` copy is read as
    it lies.
  - The pack README test names the two files put in the pack by hand after
    its README was written (SPEAKER-SCRIPT.md and .pdf, 14 Oct 12:56 and
    12:58, against 05:08); any other unlisted file still fails.
- **test_build_progress_oct_data, 45/47 -> 48/48.**
  - The per-commit comparison leaves out the deck's status line
    (`status_date`, `release`). It is read from the newest CHANGELOG
    section and every change adds one, so it was stale from the next commit
    on. The deck build's `--verify` still compares it.
  - A sent pack (`SENT_PACKS`: the October deck, sent 27 Oct) is not
    rebuilt. When its builder or data has moved on, the bytes it was built
    from must be in a commit (`progress_oct.json`: 335ec58).
    `test_a_sent_pack_needs_its_sources_on_record` shows that bytes never
    committed are refused.
  - `docs/data/progress_oct.json` regenerated by its data step, bench
    included: WP1 29/29 round trips, WP2 P0 122/122 over 32 policies at 256
    tokens (the naive cut also 122, as before: the KPI does not discriminate
    at this budget), near `_check` 0.336 ms median under the grant load
    (0.31 before), WP4 smoke 64 and nightly 744 episodes. The pack's chart,
    which the step rewrites, came out byte for byte the same.

#### Found
- **The explainer pack's .pptx is not the build's output.** It was edited
  by hand after the build (14 Oct, 12:47: 28 slides against the build's 22,
  another cover) and no longer matches its build stamp. The builder
  reproduces the build's record, not that file.
- **Nightly on the merged tree** (5af6ddc, clean): 658 pass, 66 tracked, 0
  fail, 20 known; fail-safe 721/741 = 0.973, as on 2 Nov (never-trigger
  null 0.623); P0 escape 0.0 over 741 Shield-on episodes. Kept in
  `demo/out/stress_nightly_20261104` (gitignored), beside the 7 Oct run.

#### Limits
- `progress_oct.json` now holds 4 Nov figures under its October name, as
  its data step intends ("regenerated from the code"). The explainer prints
  its own date's figures, from a commit. The October progress deck's
  builder still reads the live file: its pack is sent and is not rebuilt,
  and a rebuild would print 4 Nov figures under its October title.

### 2026-11-04 (lab CI) — the full lab run at dea9a1e: four failing files, each fixed at its cause; the library republished from the merged tree, one cell moved

#### Fixed
- **test_policy_dsl_grant_form: a stale expectation, not a regression.** IR
  schema 3.1.0 (F) rounds the policy's local view to the micrometre on
  purpose (`guardrail/projection.py`); the test still demanded that view be
  bit-identical to the reference implementation's projection, (38.961822,
  40.341935) against (38.9618217773572, 40.341935165238766). The branch's own
  run skipped it: it has no reference checkout. The test now holds the
  projector to the last bit, the view to the rounded value and the gap to
  half a micrometre; 66/66 (59fa12b).
- **test_track_truth had been failing in the lab since the 28 October Jetson
  matrix flights, and its assert named only the first flight it met.**
  Listed in full, 206 flights scored and 53 lost to their null:
  - 31 were flown with `--teacher-truth`. Their boxes are the subject's true
    position, not a detection, so they say nothing about a detector. They are
    skipped by the flight's own `metrics.json` declaration, never by score.
  - 10 held the subject within 8 px of the frame centre and lost by more
    than the tie bound (5 % + 0.1 px): six on the Orin Nano (28 Oct), four on
    the AGX Orin (3-4 Nov), e.g. agx_follow_01 3.3 px against 3.0. A new
    shape accepts them only with the box within 8 px of the subject at the
    median AND within 25 px on at least three scored ticks in four (lowest
    such flight 0.823; the nearest loss under it that is not a crowd flight,
    0.236).
  - aiot_rec_k01, the black-car recording kept but not trained on (30 Oct),
    94.3 px against 75.1: its own controller was in TRACK on 12 % of ticks.
    Shape five: the flight said it had lost the subject.
  - This reverses the 29 Oct note on orin_hier_aerovla_20261028 ("the
    tolerance is not widened to fit it"). The tie bound is not widened; the
    new shape needs independent evidence that the box was on the subject,
    and `test_the_loss_shapes_do_not_excuse_a_box_off_the_subject` shows
    each shape refusing a flight just past it (a box 12 px off a centred
    subject, a box off it on 40 % of ticks, a controller that believed it
    was tracking while 94 px off). 37/37 (a586956).

#### Changed
- **`docs/data/scenario_sweep.json` republished from the merged tree**
  (a586956, clean; the 3 Nov publish was 11252c7-dirty). 154 pass, 22
  tracked, 0 fail, 4 known failures, as before; fail-safe 175/179 = 0.978,
  unchanged. One cell moved, dyn-nfz-spawn-on-top@size_m=20: 77 P0 violation
  ticks and 77 repairs became 0 and 0, out of the zone in 2.8 s instead of
  5.6 s, mission fail became success. Library: mean time to safe 2.997 s ->
  2.926 s (39 recoveries, max 6.7 s), mission success 0.5419 -> 0.5475,
  null-passthrough P0 rate 0.2420 -> 0.2404, mean repair magnitude 6.978 ->
  6.995 m/s.
- `docs/FINAL-REPORT-outline.md` (the KPI table and WP3-17) and
  `docs/DESIGN-escalation-fsm.md` quote the new figures.

#### Found
- **The zero in that cell was checked, not assumed.** The zone appears
  centred on the aircraft, so all four edges are equally near. Since E's
  `escape_direction` breaks such a tie by the commanded heading, the pilot's
  forward command is the way out and the Shield passes it; before, the tie
  went to a side edge and GeofenceEscape flew there. The per-tick log (a
  re-run of the two cells with `--keep-logs`) flags the state unsafe from
  t = 5.1 s until the aircraft is out, with no action violating P0 - so
  none is counted. The 40 m variant has shown the same since the cell was
  added (7 Oct).
- **The track scorer's error carries the box's age times the yaw rate.** It
  judges each box against the subject projected at the tick, not at the
  frame's capture (det_age_s, median 0.18 s on the teacher flights). On
  aiot_tch_w03/w04, flown with DART yaw-rate perturbations, the error is
  85.5 / 90.8 px on ticks where age x yaw rate exceeds 20 px and 11.6 /
  10.8 px where it is under 5 px. The term is small at ordinary yaw rates;
  it is there on every flight.

#### Tests
- Lab CI at dea9a1e (`docs/data/ci_local/dea9a1ea3bfb.json`): 137 of 141
  files, 3578 of 3617 tests, claim scan clean, smoke fail-safe 0.984127
  (never-trigger null 0.634921). Two of the four failures are fixed above;
  the two deck files (test_explainer_deck, test_build_progress_oct_data)
  have their own section.
- On the republished sweep: test_kpi_magnitudes 84/84, test_sweep 102/103
  (1 skip), test_build_eval_data 10/10, test_build_deck_data 8/8,
  test_check_claims 22/22, `tools/check_claims.py` clean.

### 2026-11-04 (merge) — the three 3 November branches on master: 49 DONE, 78 PARTIAL, 8 DRIFT, 1 MISSING

#### Changed
- The SITL branch (G), the one-path Shield branch (E) and the WGS84 /
  distance-envelope branch (F) of 3 November merged onto master in an
  integration worktree. Their sections are below:
  - 2026-11-03 (G): GCS-set fence, MISSION_ITEM_INT, SROS 2 per-topic,
    harness drive;
  - 2026-11-03 (E): RuleClock, the Shield's point verdicts, the buffer,
    the tie fix;
  - 2026-11-03 (F): WGS84 canonical, distance_envelope, MSL.
- E and F both changed `guardrail/shield.py`. The merge keeps F's
  `_runtime_rule` (a distance envelope as the runtime rule of its class, an
  MSL band over the Shield's `ground`) and E's `RuleClock` for the schedule,
  so `_build_runtime` maps each rule first and the clock reads the mapped
  rules. E's `SlidingWindow` buffer is built beside F's `ground`.
- `bundle lock`: 32 policies, 0 problems. `tools/convert_policies_wgs84.py
  --check`: every proof held. The tests where the branches meet passed
  (test_shield 86/86, test_shieldcore_onepath 10/10,
  test_distance_envelope_msl 12/12, test_policy_hud 45/45).
- The full CI loop on the merged code (`tools/ci_local.py`): 139 of 141
  files, the claim scan clean, the smoke profile's fail-safe trigger
  correctness 0.984127 against a never-trigger null of 0.634921 (headless,
  not KPI-grade). The two failures are fixed below.
- `docs/FINAL-REPORT-outline.md`: 49 DONE, 78 PARTIAL, 8 DRIFT, 1 MISSING.
  WP3-10, X-15 and WP3-23 are DONE; WP3-22, X-13 and WP1-14 move from DRIFT
  to PARTIAL. The WP3 paragraph no longer says the city rail flies a 3 s
  lookahead or that the theta basis is open.
- `tools/build_flight_bundles.py` signed the WGS84 versions with the lab
  key: `fase3-sim-demo-v0.1.3`, and `v0.1.2` of corridor-survey,
  follow-car, follow-car-nfz, follow-car-citylife, follow-car-citylife-nfz,
  -nfz3, -nfz3b, follow-pedestrian, vla-goto-citylife,
  sitl-pedestrian-standoff and poly-test (`wgs84-taipei-demo-v0.1.1` was
  already WGS84). Their hashes are the conversion table's
  (`docs/data/wgs84_conversion.json`). The earlier bundles stay for the
  runs that flew them. Three usage examples under `sitl/` name
  `fase3-sim-demo-v0.1.3`.

#### Fixed
- `tests/test_wp4outputs_harness.py`
  `test_an_idle_session_expires_and_frees_its_slot` failed on timing, not
  on the harness. Its session TTL was 0.3 s, and `start_episode` alone took
  0.20-0.55 s on a loaded machine, so the session under test expired before
  the null was asked. Every wait is now a multiple of a 2.0 s TTL; it passed
  3 times in a row.
- `tests/test_replay.py`
  `test_an_episode_with_no_log_is_refused_rather_than_packaged_empty`
  errored where the stored run is absent (any worktree), because it reads
  that run's manifest through `_flown_policy`. It now skips there, like the
  other tests of the file.

### 2026-11-04 (records) — the contract map brought up to date with the 2 November work: 46 DONE, 78 PARTIAL, 11 DRIFT, 1 MISSING

#### Changed
- `docs/FINAL-REPORT-outline.md`. The 2 November Policy DSL branch merged
  without updating its Appendix A rows; checked against the code and the
  grant's wording, they now read:
  - WP1-02 DONE: rings in canonical order (`canonical_ring_order`), and the
    lint refuses a corridor no flight can be inside and two disjoint hard
    corridors. The grant's schema has no polygon holes.
  - WP1-22 and X-09 DONE: `check_lock` refuses any IR-shape change, additive
    included, until `IR_SCHEMA_VERSION` rises (2.2.0, then 3.0.0), each with
    a row in `docs/IR-MIGRATIONS.md`.
  - WP2-16 DONE: the bundle ships the multi-scale geometry (`multiscale.json`,
    digest in the signed manifest).
  - WP1-01 from DRIFT to PARTIAL: three of the grant's required fields are
    required since IR schema 3.0.0; `scope` is still optional.
  - WP1-05 stays PARTIAL with the converter and the two-hash contract named;
    the PI's acceptance is still open.
- Also brought up to date:
  - WP4-13: the exporter's bag path ran on four stored ROS 2 bags on
    3 November.
  - X-14: the SITL wind and GPS-loss stressor runs of 2 November; still
    PARTIAL.
  - The KPI table of section 2: fail-safe correctness against the rubric
    (0.978 / 0.984 / 0.973, below 0.99), mean time to safe 2.997 s, and the
    theta basis decided.
- Not counted here: the SITL branch of 3 November (GCS-set fence WP3-10,
  SROS 2 per-topic permissions X-15), which is not merged yet.

### 2026-11-04 (CI) — the whole local suite on the day's four commits: one new failure, the workflow's own file lists, fixed

#### Tests
- `tools/ci_local.py --no-smoke` at 03718b3: 133 of 139 files, 3538 of
  3580 tests. New against the record of ea90a67: `test_harnesstruth_ci.py`
  33 of 35, because `test_held_box_gate.py` ran in the loop without a line
  in the workflow's header and `test_hud_fits.py` (Pillow step) had no
  exclusion entry; both are added (`.github/workflows/tests.yml`,
  `tests/test_harnesstruth_ci.py` EXCLUDED: 8 of its 12 tests ERROR without
  Pillow, measured with PIL blocked) and the file passes 35 of 35. The
  other five failures are the ones ea90a67 recorded, with the same counts:
  test_build_progress_oct_data 45 of 47, test_explainer_deck 10 of 24,
  test_harnesstruth_bundles 11 of 12, test_track_truth 35 of 36,
  test_wp4outputs_harness 15 of 16. The Pillow step's files, which
  ci_local does not run, pass by hand: test_policy_hud 45 of 45,
  test_hud_fits 12 of 12.

### 2026-11-04 (end of day) — the end-to-end follow refuses a held box its subject could not have reached: the phantom behind the AGX flight's one P0 escape is refused on replay

#### Fixed
- `demo/vla_fly.py`: the committed follow took every held box's map point
  as its subject's place. In `agx_e2e_aerovla_01` (2026-11-03) a soft box
  (score 0.0093, threshold 0.008; 63.7 x 178.2 px, taller than wide, where
  the car from 8 m up is wider) lay 12.4 m from the last point 0.91 s later
  and put the subject 3.4 m from the aircraft with the level car 29.9 m
  away; the stand-off repair closed on it and the Shield counted the
  flight's one P0 escape. New `take_held_box()`, the one measurement step
  for `_acquire_step` and the replay, takes a held box only within
  follow_vlm.Reacquirer's reach of the last accepted point: an "ok" box
  within its `reach_base_m` (10 m) + v_r x dt, a "soft" box - the
  identity's word for a box only allowed to continue a track it agrees
  with - within its pick-to-pick `step_m` (3 m) + v_r x dt, v_r =
  clip(1.5 x the measured speed, 3, 12) m/s. A refused box is not a
  sighting; refused for LAPSE_S the subject lapses and is re-acquired from
  its last good place. `metrics.follow.acquisition` records
  `held_jumps_refused` and the first ten.

#### Found
- Chosen on the 79 perception follows under demo/out, replayed with
  `tools/replay_standoff_feed.py` (224,255 ticks from the first
  acquisition), each refused box set against the level car's truth:
  `step_m` for every box refused 690, 460 of them within 5 m of the car (a
  slow car's map-point jitter); `reach_base_m` for every box refused 153
  and let the phantom through; the rule above refuses 173 boxes in 37
  flights, 158 of them more than 8 m from the car and 8 within 5 m. At the
  flown forecast (5 s / 0.1 s, the subject's velocity given) the replayed
  P0 escapes go from 27 to 10, `agx_e2e_aerovla_01`'s among those gone (1
  to 0); the ticks with a subject fed stay 0.301 of the total (0.302
  before), the ticks with the car truly within 5 m and the subject placed
  within 5 m stay 129, and the median distance of the subject to the car
  stays 2.06 m (closer in 15 flights, farther in none). The 10 left (in
  `aiot_v3g_01` and `aiot_v10p_03`) are not jumps: no box of theirs is
  refused.

#### Added
- `tools/replay_standoff_feed.py`: the replay takes held boxes through
  `take_held_box` (`--no-jump-gate` for the old step), counts the refusals
  per run and the P0 escapes per forecast (the Shield's re-check of what it
  emitted, as kpi.py counts them).

#### Tests
- `tests/test_held_box_gate.py` (new, 10 of 10; in
  `.github/workflows/tests.yml`): a car at 10 m/s is taken box after box; the
  20.1 m phantom is refused and moves neither the place, the velocity nor
  the last sighting; the reach grows with the time since the last good
  point; a slow car's 6 m jitter is not a jump; a soft box at 12.4 m is
  refused where an ok one is taken; the constants are the Reacquirer's;
  phantoms for LAPSE_S leave the Shield no subject, the last good place
  kept; `_acquire_step` and the replay share the step; the record carries
  the refusals. `tests/test_vla_fly.py` 63 of 63 (its pinned acquisition
  record gains the two fields), test_vla_follow_reacquire 13 of 13,
  test_vla_standoff_feed 11 of 11, test_vla_follow_smooth 14 of 14,
  test_pas_vla 29 of 29, test_vla_fly_fakesim 14 of 14 (13 of 14 once
  under the replay's load, 14 of 14 alone, as before).

#### Limits
- Open loop: the replay feeds each tick's logged command, so 10 replayed
  escapes are what the Shield would have done with that command, not what a
  flight with this step would fly. Not flown yet. The 8 refused boxes
  within 5 m of the car are a short loss each, not a wrong subject.

### 2026-11-04 (late night) — the three AGX flights that showed the gaps, flown again on the whole level's map: no tick off the map, no contact, every panel on the frame

#### Flown
- On the AGX Orin at 989f967 (MAXN, every VLA served on the board, the
  simulator on this PC, the matrix's runner and arguments), with the maps
  of the section below synced to the board:

  | flight | within 30 m | off-map ticks | contacts | smallest clearance on the path (whole-level map) |
  |---|---|---|---|---|
  | `nfz3b_agx_hier_aerovla_20261104` | 0.494 | 0 | 0 | 2.76 m |
  | `nfz3b_agx_hier_aerovla_20261103` (before) | 0.152 | 3484 | 1919 | -2.91 m |
  | `agx_e2e_r3_20261104` | 0.285 (after acquisition) | 0 | 0 | 3.00 m |
  | `agx_e2e_r3_20261103` (before) | 0.098 | 2926 | 0 | 1.99 m |
  | `agx_e2e_racevla_20261104` | 0.079 (after acquisition) | 0 | 0 | 3.03 m |

  The NFZ3b flight came within 3 m of a building on 16 ticks; the
  building-clearance rule was violated on 26 ticks and repaired. P0 escape
  rate 0.0 on all three, status `measured`, `p0_ticks_blind` 0 (the first
  kpi.json files with the field). The Orin Nano's NFZ3b flight of
  28 October read 0.454.
- The panels on the board's frames: "RaceVLA  no decision flying  pipelined"
  and "AerialVLA FT v4_r3 #27  image 1.8 s old  pipelined" on the VLA panel,
  "P0 escapes 0" on a line of its own inside the rule panel, the banner
  clear of the status box; the board draws them in DejaVu Sans Mono
  (`fc-list`). PIP videos in `E:\vla_drone_offload\videos`, copies under
  30 MB in `docs/video/<tag>_pip_small.mp4` (gitignored).

#### Limits
- One flight each; the NFZ3b difference rests on one flight before and one
  after. The 4 November NFZ3b flight never left the map, so it does not
  show what the Shield does at a building it would once have missed.

### 2026-11-04 (night) — CityLife's obstacle maps cover the whole level: the building the NFZ3b flight touched is on the map, and the rule the AGX flights were blind to sees again

#### Changed
- `demo/out/citymap_citylife/` (gitignored) is rebuilt over the whole
  level: NED x -960 to 1060 m and y -860 to 880 m at 2 m (1010 x 870
  cells, every band), from the simulator's voxels, where it was the 360 x
  200 m rectangle x -140 to 220, y -60 to 140. The level's buildings reach
  x -916 to 1008 m and y -802 to 834 m (measured on these voxels; the
  actors' positions said x -710 to 750, y -598 to 557, their pivots), so
  every band now has an empty margin of at least 40 m: no occupied cell
  within 20 m of any edge. Off a map the Shield takes every obstacle to be
  on it (`guardrail/shield.py` `_distance_at`); on CityLife that is now
  true. The earlier maps are kept in
  `demo/out/citymap_citylife_20260924/`; `map_build.json` beside the maps
  records how they were made.
- Inside the old rectangle the new maps are the old ones, cell for cell:
  the 0-2 m, 6-14 m and 15-55 m bands came back identical (0 of 18000
  cells differ), the 2-4 m band differed in 362 cells where cars and
  pedestrians stood at scan time, and those were kept from the old scan
  (`--keep-from`), so the street mask there is unchanged (0 of 18000) and
  the pedestrian crossings, landing sites and identity checks built on it
  stay as tested.

#### Added
- `demo/build_voxel_map.py`: `--tile-m` asks for the rectangle in square
  tiles that start on the whole grid's cells and stitches them (70.3
  million voxels in 16 requests, 160 s; one request for the old rectangle
  takes 8 s); `--keep-from DIR` keeps an earlier build's cells inside its
  own rectangle and reports how many the new scan had otherwise;
  `map_build.json` is written with every build (arguments, extent, the
  ground slice, what was kept). Its docstring gives both commands.

#### Found
- Re-judged on the whole level's map, the two AGX flights that left the old
  map: `nfz3b_agx_hier_aerovla_20261103` read a smallest clearance of
  2.59 m along its path on the old map (245 ticks under 3 m) and 3.01 m at
  the point of contact (64.5, 145.5), which was off it; on the new map the
  path reaches -2.91 m, inside the building, with 2241 ticks under 3 m, and
  the contact point reads -3.14 m. `agx_e2e_r3_20261103` read no tick under
  3 m on the old map and has 2931 on the new one (smallest 1.99 m), with no
  contact reported. Neither flight's Shield could have acted on them.
- The ArduPilot rail's documented example goal, `--goal 50,-78`
  (`docs/DESIGN-vla-pilot.md`), lay 17 m south of the old map: its "17.0 m
  at the goal" was that off-map estimate. On the new map its straight line
  from the spawn passes through a building (-1.86 m) and the goal has
  3.70 m. It is replaced by (46, -52), 33.8 m WNW of the spawn, 4.08 m on
  the line (the spawn's own clearance) and 10.0 m at the goal, the same on
  both maps. The flown goto runs (`pas_vla_goto_01`, `_02`,
  `vla_goto_01_20261023`) never left the old map.
- The 82 m street grid that landing's `prefer="pavement"` rests on is
  0.96-1.00 street along each line on the old rectangle, where it was
  measured, but 0.84-0.88 over the central city (y -250 to 250 m).

#### Tests
- `tests/test_voxel_map.py` 8 of 8 (3 new): tiles start on the grid and
  cover the rectangle once; a tiled query stitches to the single request
  voxel for voxel, a block across a seam included; a short tile is refused.
- Tests that assumed the old rectangle, now reading the map they are given:
  `tests/test_landing.py` 25 of 25 (off-map points 50 m past the map's own
  edges, where (-500, 0), (0, 500) and (300, 0) were off it before; the
  grid-line check on the rectangle it was measured on, from
  `map_build.json`), `tests/test_flight_pas_ap_20261008.py` 3 of 3 (the
  flight's own 360 x 200 m map cut out of the new one, so its replay reads
  the map it flew), `tests/test_pas_vla.py` 29 of 29 (the new goal, which
  must be on the map). Unchanged on the new map: test_citylife_peds 26 of
  26, test_search 20 of 20, test_identity 21 of 21, test_vla_fly 63 of 63,
  test_railvideo_recording 21 of 21, test_city_traffic 34 of 34,
  test_citylife_routes 22 of 22, test_occ_bands 14 of 14, test_clearance
  16 of 16, test_vla_standoff_feed 11 of 11, test_shieldcore_repair 25 of
  25, test_own_velocity_forecast 28 of 28, test_hold_points 34 of 34,
  test_shieldcore_audit 16 of 16, test_policy_hud 45 of 45.

#### Limits
- The maps are gitignored: another checkout keeps the old rectangle until
  it is rebuilt with the commands in `demo/build_voxel_map.py`, and the
  Jetsons get it through `deploy/orin/sync_follow_controller.ps1`.
- Outside the old rectangle the 2-4 m band and the street mask hold the
  cars and pedestrians that stood there at scan time. The flight bands do
  not move with them.
- Building the map in the simulator's voxels does not make a flight safe
  where the Shield's forecast cannot stop it in time; it makes the rule see.
  Whether an NFZ3b flight now keeps its clearance is not shown here: that
  needs a flight.

### 2026-11-04 (evening) — the video panels say which model flew and keep every word on the frame: the P0 escapes back in view on Jetson-flown videos, no banner over the status box

#### Fixed
- `demo/vla_mission.py`: the VLA panel's second line was "AerialVLA #N"
  or "AerialVLA  no decision flying" whatever the server was, so every
  end-to-end DroneCATS and RaceVLA video named the wrong model (checked:
  `cmp2_e2e_racevla_20261028` on the PC, `orin_e2e_racevla_20261028` on the
  Orin Nano, `agx_e2e_dronecats_20261103`, `agx_e2e_dronecats_t30_20261103`
  and `agx_e2e_racevla_20261103` on the AGX). New `model_label(identity)`
  names the model from the id in the server's hello: "AerialVLA",
  "AerialVLA FT v2_r1" for a follow fine-tune, "DroneCATS", "RaceVLA",
  "stand-in, not a VLA" for a `fake:` id, "VLA" with no identity.
  `vla_hud` carries it; `MissionLoop` (vla_fly) and the ArduPilot rail's
  `VLAPilot` take it from the hello, and their console lines use
  `model_words`. `demo/vla_hierarchy.py` RULE, recorded in every
  hierarchical flight's metrics, no longer names AerialVLA.
- `demo/policy_hud.py`: the rule panel was 470 px with columns at fixed
  pixel offsets, laid out for Consolas. The Jetsons have no Consolas; their
  DejaVu Sans Mono is wider, so the last line ("Shield corrections N  brakes
  N  holds N  P0 escapes N", 478.7 px of text in 460 px of room with
  one-digit counts) put the P0 escapes off the frame on every Jetson-flown
  video, and on the PC too once the counts grew (2784 corrections and 454
  holds: 478.5 px). A ten-character value ("141 m away") ran into its
  status. Now the label and limit columns are sized from the policy's own
  text, the value and status columns for the widest they can hold, and the
  panel from all of them; the P0 escapes have a line of their own, red when
  not 0. The status box grows to its longest line (at least the old 322 px,
  at most 45 % of the frame, a longer line cut with an ellipsis) and, in a
  flight's overlay cache, never shrinks. The banner is centred in the room
  right of the status box when centred it would cover it (it covered the
  speed in `nfz3b_agx_hier_aerovla_20261103`), shrunk to 14 px and only then
  cut. `draw_status_box` and `draw_policy_overlay` return the boxes they
  drew, and the overlay cache cuts its tiles from those boxes instead of
  fixed fractions of the frame, which clipped a grown panel and pasted the
  overlap of two fractions twice. `font()` asks for DejaVu Sans Mono first,
  by name or inside matplotlib, so the PC draws what the Jetson draws.
  `tools/rerender_policy_hud.py` passes its status box too.

#### Tests
- `tests/test_hud_fits.py` (new, 12 of 12 pass): drawn with the nfz3b rows
  and counts at 960, 1280 and 1920 px wide, no ink lands outside the boxes
  drawn and every box is on the frame; the P0 escapes are inside the panel
  and red only when not 0; on every row the value's ink ends half a
  character before the status's begins; a long banner leaves no ink in the
  status box and an over-long one is shrunk, cut, and kept on the frame;
  the status box's width rules and the cache's grow-only width; the model
  label for the ids the AGX matrix's servers sent; `hud_lines` with and
  without a model; both rails pass the hello's label; RULE names no model.
  Checked against two mutants: a layout that under-measures its text by
  15 % (columns touch) and an overlay that ignores the status box (the
  banner lands in it); both fail.
- `tests/test_policy_hud.py` 45 of 45: one test pinned the policy as
  "v0.1.0", stale since the file moved to 0.1.1 on 2026-11-02 (a7763e5),
  and it failed unseen because the file needs Pillow, which the CI loop does
  not install; it now reads the policy's own version. Both files join the
  workflow's Pillow step (`tools/ci_local.py` runs only the loop, so they
  are run by hand there). Unchanged: test_pas_vla 29 of 29, test_vla_fly 63
  of 63, test_vla_fly_fakesim 14 of 14, test_pas_ardupilot 149 of 149,
  test_vla_hierarchy 17 of 17, test_make_demo_video 4 of 4,
  test_render_track_video 5 of 5, test_vla_dagger 18 of 18,
  test_vla_follow_smooth 14 of 14, test_vla_follow_reacquire 13 of 13,
  test_vla_prev_front_rate 25 of 25, test_vla_speed_words 17 of 17.

#### Limits
- The panels are drawn into the frames during the flight, so videos
  already recorded keep the old text; their run index says so
  (`D:\vla_drone_offload\jetson_runs\INDEX.md`). The DejaVu face changes the
  PC videos' look slightly (wider text) from today on.

### 2026-11-04 (afternoon) — the KPIs read what the rules could not see and what the aircraft hit: a flight off the obstacle map is no longer a measured 0.0, and a flight that touched a building is no longer a success

#### Fixed
- `guardrail/kpi.py` compute(): Shield ticks flown off the obstacle map
  (`off_map`, which follow_vlm and vla_fly write on every tick the Shield
  has a map) are counted in the new `p0_ticks_blind`. Off the map the
  building-clearance rule's distance field holds no building, so no
  violation there is not a clean tick. With any such tick the escape rate's
  status reads `partially_measured`, the reasons carry `p0_blind:off_map`
  and `mission_success` is False, the rule already applied to P0 ticks the
  log could not measure. The status now also reads `partially_measured`
  for those (`p0_ticks_not_measurable`), which it never did. A log whose
  rows never say leaves the count None, not 0. The rate itself is
  unchanged.
- compute() reads the simulator's contact reports (`metrics.collisions`
  when `measured`): a contact during the mission fails the outcome
  (`collision`) and is counted in `collision_contacts` and
  `collision_objects`; contacts before t0 (take-off) and after the last tick
  (landing) stay out, as collisions_summary splits them. rollup() pools
  both (`p0_ticks_blind`, `episodes_p0_blind`, `episodes_with_collision`,
  `episodes_collision_not_measured`); `failure_categories()` names
  `p0_blind` and `collision`, and the top-K list ranks a contact beside the
  escapes and shows both counts.
- `guardrail/replay.py`: `p0_ticks_blind` joins `VERIFIED_KPI_FIELDS`, so a
  bundle's blind count is re-derived from its rows. A field missing from an
  older bundle is noted only when the log can give it (compute() not None).

#### Found
- The two AGX flights it changes, recomputed read-only from their stored
  logs (the kpi.json files stay as flown; `tools/rescore_kpis.py` only adds
  fields): `nfz3b_agx_hier_aerovla_20261103`, 3484 of 4160 ticks off the map
  and 1919 contact reports with BP_jctBldSetK33_6 from t 351.5 s, read
  outcome success, mission_success True and the escape rate measured at
  0.0; now fail (`collision`, `p0_blind:off_map`), mission_success False,
  partially_measured. `agx_e2e_r3_20261103`, 2926 of 4200 ticks off the map
  and no contact: mission_success False, partially_measured. The other 19
  AGX runs (the matrix and the first three flights of 3 November) are
  unchanged, and the P0 figures (rate, escapes, violation ticks) are equal
  on all 21.

#### Tests
- `tests/test_kpi_blind.py` (new, 14 of 14 pass; in
  `.github/workflows/tests.yml`): a log with the nfz3b counts fails with
  both reasons; the same log without `off_map` and `collisions` scores the
  old success (the finding itself); take-off and landing contacts do not
  fail; an unmeasured contact topic is None; autopilot ticks stay out; the
  rollup, top-K order and replay field; every obstacle_clearance rule under
  policies/ is P0, which is what lets an off-map tick count as a blind P0
  tick without a rule lookup. Unchanged suites: test_kpi_magnitudes 84 of
  84, test_kpi_report 40 of 40, test_replay 61 of 61, test_bundle 57 of 57,
  test_manifest 68 of 68, test_wp4outputs_kpi_report 24 of 24,
  test_build_dashboard 20 of 20, test_replay_identity 9 of 9,
  test_sitl_rails 51 of 51, test_city_episode 4 of 4.

#### Limits
- A blind tick says the aircraft was off the map, not that a building was
  near: a gap in the evidence, not a violation. A contact count is only as
  complete as the simulator's collision topic. Rails that write no
  `off_map` (the SITL rails, the stress harness) report the count as None.
  This makes the gap visible; it does not close it: the flights left the
  map because the map (x -141 to 219 m, y -61 to 139 m) does not cover the
  level they fly in.

### 2026-11-04 — the 28 October Jetson comparison matrix re-flown on the AGX Orin with every VLA served on the board: the hierarchical follows hold their Orin Nano scores at a median 2.4 times the latency; one flight left the obstacle map and pressed against a building with a P0 escape rate of 0.0

#### Added
- `deploy/orin/agx_dronecats_env.sh` (6da813d): DroneCATS' own venv on the
  AGX (transformers 5.21.0 beside the VLA env's 4.40.1, the same torch
  2.14.1 build), with its test in `tests/test_deploy_configs.py` (67 of 67
  pass; the test refuses a transformers 4 pin, another torch, torchao, or a
  torch call without the CUDA 13.2 index).
- `deploy/evidence/hil/aerialvla_serving_bench_agx_merged.json`: AerialVLA
  in the matrix's configuration (bf16, sdpa, adapter merged) on the AGX:
  p50 1073.9 ms, p95 1099.6 ms over 50 requests, 0 parse failures, peak
  15.471 GB (unmerged: 1520.3 ms). MAXN setting; not R1. Listed in
  `deploy/evidence/README.md`.

#### Flown (AGX Orin: flight client and VLA server; simulator on the PC; not KPI-grade)
The 16 steps of the 28 October Jetson matrix and its NFZ3b flight, with the
same server configurations (the AerialVLA family bf16/sdpa/merged with
`--missing-down refuse`; DroneCATS bf16 on the detector's frames; RaceVLA
bf16; the fixed 12 m stand-in), the same flight arguments and the simulator
restarted for every flight (offscreen, 30 fps), at 6da813d with the AGX in
MAXN. The difference is where the server ran: on the AGX itself, where on 28
October it ran on the PC's GPU behind an SSH tunnel. Every model's weights
digest equals 28 October's (ca9e8701, 3309cdfb, 8cbab0c9, 5080e727,
03f3bb48). Scores on the 28 October basis (pure VLA: within 30 m after
acquisition; otherwise within 30 m from the start gate):

| step | AGX | Nano | AGX / Nano VLA latency (s) |
|---|---|---|---|
| hand-written | 0.952 | 0.976 | - |
| hierarchical AerialVLA | 1.000 | 0.989 | 1.44 / 0.63 |
| hierarchical r1 | 0.647 | 0.890 | 1.47 / 0.62 |
| hierarchical r2 | 0.920 | 0.924 | 1.46 / 0.52 |
| hierarchical r3 | 0.843 | 0.876 | 1.46 / 0.62 |
| hierarchical r4 | 0.911 | 0.882 | 1.46 / 0.49 |
| hierarchical DroneCATS | 0.941 | 0.969 | 2.64 / 1.24 |
| hierarchical RaceVLA | 0.715 | 0.704 | 1.22 / 0.51 |
| hierarchical fixed 12 m (no VLA) | 0.374 | 0.799 | 0.09 / 0.10 |
| pure AerialVLA | 0.048 | 0.102 | 1.48 / 0.60 |
| pure r1 | 0.034 | 0.113 | 1.47 / 0.58 |
| pure r2 | 0.130 | 0.017 | 1.48 / 0.58 |
| pure r3 | 0.098 | 0.076 | 1.49 / 0.39 |
| pure r4 | 0.077 | 0.077 | 1.47 / 0.46 |
| pure DroneCATS | 0.000 | 0.076 | 1.71 / 0.78 |
| pure RaceVLA | 0.055 | 0.083 | 1.23 / 0.51 |
| NFZ3b, hierarchical AerialVLA | 0.152 | 0.454 | 1.39 / 0.62 |
| pure DroneCATS, 30 s timeouts (extra) | 0.081 | - | 1.75 / - |

- P0 escape rate 0.0 on all 18 flights (but see Found). The served latency
  on the board was 2.1 to 3.9 times the Nano matrix's (median 2.4); over the
  six hierarchical VLA models flown in full on both (all but r1 and the
  stand-in) the mean is 0.888 on the AGX and 0.891 on the Nano, and the
  seven pure-VLA flights stay at or below 0.13 on both.
- The runs: `D:\vla_drone_offload\jetson_runs` (the first seven) and
  `E:\vla_drone_offload\jetson_runs` (from `agx_e2e_r3_20261103` on), listed
  in the former's `INDEX.md`; full PIP videos in `E:\vla_drone_offload\videos`,
  copies under 30 MB as `docs/video/<tag>_pip_small.mp4` (git-ignored).

#### Found
- **A flight off the obstacle map pressed against a building while its P0
  escape rate stayed 0.0.** `nfz3b_agx_hier_aerovla_20261103` searched for
  the car past the map's edge (y 139) from t 68 s and was off the map on 3484
  of its 4160 ticks; from t 351.5 s to the end of the mission (420 s) the
  simulator reported 1919 contacts with `BP_jctBldSetK33_6`, and 657 more
  after it. The console said that building clearance is not checked off the
  map, yet
  `kpi.json` reports the escape rate as measured, with 0 ticks not
  measurable. The 28 October flights also left the map (two pure-VLA Nano
  flights for more than 3100 ticks each) without touching anything; this is
  the first mission contact in any Jetson flight.
- **Pure DroneCATS cannot hold vla_fly's timeouts on the board.** Its
  planning answers (up to 1024 tokens) outlasted the 10 s decision timeout,
  and the mission ended as `server_slow` after 384 ticks (0.000). Flown again
  with 30 s timeouts it lasted the mission but decided 11 times in 420 s
  (p95 10.7 s) and hovered on 3473 of 4201 ticks (0.081).
- The hierarchical r1 flight's start gate timed out (the car never came
  within 30 m in 330 s), so its 0.647 began without a subject; the stand-in
  flight lost the car for most of the mission (out of view on 69 % of its
  ticks; the Nano's 37.5 %): neither difference is the VLA's.
- D: (an SMR disk, 55 GB free) slowed to about 0.6 MB/s under the run copies,
  so the runner copies to E: since `agx_hier_r3_20261103`.

#### Limits
- One flight per configuration. In the 27-29 October set
  (`docs/FINAL-REPORT-outline.md` section 6) the hand-written follow's five
  PC flights had a sample std of 0.012, and a car lost at a corner moves one
  flight far more: hierarchical AerialVLA on the PC ranged 0.309 to 0.999
  over five.
- Two changes since 28 October apply to every hierarchical and hand-written
  flight: the city rail's forecast is the grant's 5 s / 0.1 s (3 s / 0.5 s
  then), and the policy is `follow_car_citylife_nfz` 0.1.1 (0.1.0 then).
- MAXN setting, clocks not locked; topology `projectairsim-remote-sim`.

### 2026-11-03 (late night) — AerialVLA on the AGX Orin itself: 1.52 s per decision; the hierarchical follow 0.988, the end-to-end follow 0.213 with one P0 escape against a phantom subject

#### Added
- `deploy/evidence/hil/aerialvla_serving_bench_agx.json`:
  `demo/vla_server_v2.py bench` on the AGX in the configuration the desktop
  flies (bf16, sdpa, adapter unmerged), 5 warm-up and 50 timed requests on
  the protocol frames: p50 1520.3 ms, p95 1537.8 ms, 0 parse failures, peak
  15.934 GB. MAXN setting, clocks not locked: not R1. Its
  `identity.agreement` block is the RTX 4090's probe
  (`docs/data/aerialvla_serving_4090.json`), carried by configuration; the
  AGX's own agreement with the reference outputs was not measured. Listed in
  `deploy/evidence/README.md`.

#### Flown (AerialVLA server on the AGX, simulator on the PC; not KPI-grade)
- `agx_hier_aerovla_01` (`follow_vlm --vla-hierarchy 127.0.0.1:8790` through
  `deploy/orin/run_follow_controller.sh`, 07cee03): within 30 m 0.988,
  detector 2.72 Hz, P0 escape 0.0, 10 repairs (9 kinematic, 1 clearance),
  0 mission collisions. The VLA answered in a median 1.88 s in flight (237
  requests: 231 steps, 3 parse failures, 2 LAND); answers older than the
  1.5 s hold left the controller flying alone on 909 ticks (21.7 %: 858
  expired, 51 no answer), against 77 ticks on the Nano's
  `orin_hier_aerovla_20261028` (0.989), whose server ran on the PC's GPU.
  Its replay bundle wrote itself and re-derives its KPIs, the first city
  flight to do so since 28 October.
- `agx_e2e_aerovla_01` (`demo/vla_fly.py` through `deploy/orin/run_vla_fly.sh`,
  07cee03): within 30 m 0.213 (0.184 after acquisition), 55 decisions at
  0.13 Hz with a median latency of 1.92 s, the aircraft hovering on 2346 of
  4200 ticks for want of a new decision. The Nano's `orin_e2e_aerovla_20261028`
  gave 0.102 and the desktop's `cmp2_e2e_aerovla_20261028` 0.12, both with
  the server on the PC. P0 escape rate 0.000238: one tick (Found).
- Both runs, their notes and PIP videos are in
  `D:\vla_drone_offload\jetson_runs`, listed in its `INDEX.md`.

#### Found
- **The end-to-end flight's one P0 escape was against a phantom subject.**
  At tick 3824 (382.6 s) a detection scored 0.0093 (threshold 0.008) placed
  the subject 3.4 m from the aircraft, 20.1 m from the previous estimate,
  moving at 7.29 m/s, while the level car was 29.9 m away. StandoffRecover
  asked for 0.50 m/s of opening, ClearanceEscape acted on the same tick
  (a building 2.43 m away two ticks earlier), and the repaired command still
  closed on the estimate, so the Shield's own check counted the tick as an
  escape. The Shield judged the perception, as designed
  (`vla_fly.shield_subject` never uses truth); what failed is the subject
  estimate, which accepted a 20 m jump from a low-score box.
- In the hierarchical flight the aircraft came within 5 m (horizontal; it
  flew at 8 m) of the car on 69 ticks with the stand-off rule silent, all
  while the target was lost (search and plan modes), when the Shield had no
  subject to keep away from.
- On the board, the VLA's latency, not the detector, decides how much of the
  flight it steers: at 1.9 s per answer most hierarchical answers expired
  before use, and the end-to-end pilot spent more than half the mission
  hovering.

#### Limits
- One flight of each, in the MAXN setting, on the remote-sim topology.

### 2026-11-03 (night) — the first follow flown from the AGX Orin: 0.952 within 30 m, detector 2.57 Hz; city flights bundle themselves again

#### Fixed
- `demo/follow_vlm.py` wrote the episode id into every flight-log row
  (since f6ee433, 2026-10-28) but not into `metrics.json` or `kpi.json`, so
  `guardrail.replay.write_replay` refused every city flight ("metrics.json
  carries no episode id while the log does") and none wrote its replay
  bundle: 72 city runs in `demo/out` from 28 October to this flight, 0
  bundles. `episode_fields()` now stamps the id on the rows, the metrics and
  the KPI table, as the SITL rails stamp theirs.
  `tests/test_city_episode.py`: a new test writes the three files the way a
  flight does and bundles them, and refuses them with the id missing from the
  two documents; 4 of 4 pass, and the new test fails on the code before the
  fix.

#### Flown (Project AirSim CityLife, simple_flight, sim on the PC, controller on the AGX; not KPI-grade)
- `agx_follow_01`: the hand-written follow of the red car (Car_10) with the
  controller and OWL-ViT on the AGX Orin (native environment, MAXN setting,
  clocks not locked) and the simulator on the desktop, over the 2.5 Gbit/s
  cable, at a609cf8: policy `follow_car_citylife_nfz` 0.1.1 from its signed
  bundle, the grant's 5 s / 0.1 s forecast, 420 s. Within 30 m 0.952,
  detector 2.57 Hz, P0 escape 0.0, 0 mission collisions (one contact before
  the mission and one during the landing descent), landing site reached.
  The run, its PIP video and its note are in
  `D:\vla_drone_offload\jetson_runs\agx_follow_01`, listed in that folder's
  `INDEX.md`.
- Against: the Orin Nano's `orin_follow_02` 0.957 at 2.18 Hz (63f9072, its
  MAXN_SUPER mode, the 3 s legacy forecast), and the desktop's three flights
  on the same 5 s forecast, `fc5s_manual_r1/r2/r3_20261103`, 0.980 / 0.961 /
  0.947. One flight; 0.952 lies inside the desktop's spread.

#### Found
- In flight the detector is not limited by the GPU: its forward pass took a
  median 192 ms on the AGX (252 ms on the Nano's flight, against 192 ms
  alone there in its 25W mode), while the same model alone on the idle AGX took 61 ms. The
  desktop shows the same gap: the whole inference 66 ms alone and 287 ms in
  flight (`demo/follow_vlm.py`'s grounder notes).

#### Limits
- `agx_follow_01` has no replay bundle (it flew before the fix), nor have
  the other 71 city runs since 28 October; their files are not edited after
  the fact.
- The flight ran in the MAXN setting, not the budgeted MODE_50W with
  `jetson_clocks` that timing and KPI runs use; its topology,
  `projectairsim-remote-sim`, is not the grant's `hil`.

### 2026-11-03 (evening) — the AGX Orin 64 GB on JetPack 7.2.1 becomes the hil machine: OpenVLA-7B unquantized in bf16 on the board at about 1 s per action, OWL-ViT at 61 ms; the first-boot check reads JetPack 7.2.1

#### Added
- `docs/DESIGN-topologies.md`, decision 11: the AGX Orin 64 GB Developer
  Kit, flashed on 2026-11-03 with JetPack 7.2.1 (L4T r39.2.1, Ubuntu 24.04.5,
  kernel 6.8.12, CUDA 13.2.2, cuDNN 9.20.0, TensorRT 10.16.2), replaces the
  Orin Nano for the Jetson work. What the board reported, why the VLA runs
  natively there, what that costs, the gate's numbers and the cable
  (10.42.0.1 to 10.42.0.3, 2.5 Gbit/s).
- `deploy/orin/agx_vla_env.sh`: the AGX's native environment. No published
  image carries a JetPack 7.2 Orin torch (jetson-containers' Orin JetPack 7.2
  issue #1736 open, its jp7/cu132 index 410 Gone), so the script builds a venv
  on the host's Python 3.12: torch 2.14.1 + torchvision 0.29.1 from PyTorch's
  CUDA 13.2 index, held by a constraints file every later pip call reads;
  `requirements-vla.txt` minus bitsandbytes; the follow controller's packages
  at vla-real's versions except opencv-contrib-python 4.11.0.86 (5.x needs
  numpy 2); the Project AirSim client at fa404d0, editable, no dependencies.
- `deploy/orin/vla_bf16_check.py`: OpenVLA-7B loaded in bf16 answers the R1
  protocol frame a few times with the flight scripts' call; refused unless
  the weights are bf16 on the GPU and every answer is seven finite numbers.
  A gate, not an R1 rate.
- Five tests in `tests/test_deploy_configs.py`: the AGX on JetPack 7.2.1
  passes the first-boot check (with and without the meta-package), its
  warnings and failures (JetPack 6.2's Ubuntu or TensorRT on r39, a missing
  cudart-13-2, an r39 that is not r39.2, an R40), a JetPack 6.2 board keeps
  its own expectations, the AGX script's pins and indexes, and the bf16
  check's refusals. `python tests/test_deploy_configs.py`: 66 of 66 pass.

#### Changed
- `deploy/orin/first_boot_check.sh` reads the release first: r39 expects
  JetPack 7.2's Ubuntu 24.04 and CUDA 13.2 / cuDNN 9.20 / TensorRT 10.16, and
  checks them by package even without the `nvidia-jetpack` meta-package
  (SDK Manager did not install it on the AGX); r36 keeps every JetPack 6.2
  expectation as it was.
- `deploy/README.md` ("The AGX Orin: JetPack 7.2.1", the layout's Orin
  rows), `docs/RUNBOOK-orin-hil.md` (which steps apply on r39),
  `docs/DESIGN-python-versions.md` (Jazzy is now the host's own
  distribution; the companion container stays) and `README.md` say so.

#### Measured (on the AGX, 2026-11-03; not R1)
- The three checks ran from the checkout at 81793ff; their outputs are in
  `deploy/evidence/hil/` (`first_boot_check.txt`, `gpu_check_agx_native.json`,
  `vla_bf16_check.json`), listed in `deploy/evidence/README.md`.
- `first_boot_check.sh`: 7 PASS, WARN no meta-package, FAIL Docker (Found,
  below).
- `gpu_check.py --skip-bnb` in the native environment: no problem
  (capability 8.7, bf16 supported, matrix product within 5.7e-7 of the CPU).
- `vla_bf16_check.py`: OpenVLA-7B in bf16 loaded in 8.8 s, eight greedy
  answers on the protocol frame all seven finite numbers and identical, 0.97
  to 1.00 s each after the first (median 0.975 s), peak memory 15.47 GB; the
  same procedure from a scratch copy before the commit gave the same. The
  30W mode and the unconstrained
  MAXN setting gave the same seconds per action with the GPU at its 1300.5
  MHz top clock: the decode reads the 15 GB of weights per token, so memory
  bandwidth sets the rate.
- OWL-ViT base-patch32 fp32 (the follow's detector): 61 ms per frame, against
  192 ms on the Orin Nano.
- `guardrail.manifest.collect_host_evidence()` on the board: device model
  "NVIDIA Jetson AGX Orin Developer Kit", L4T R39.2.1, `orin_missing()`
  empty.

#### Found
- The AGX's user is not in the `docker` group, so `docker info` is refused
  there; the first-boot check's Docker lines fail until the board's owner
  adds it.

#### Limits
- No flight has run on the AGX yet, and no R1 rate: R1 needs the budgeted
  power mode with the clocks locked.
- The R1 table's desktop (torch 2.6.0) and AGX (2.14.1) columns no longer
  share one torch version; PyTorch's CUDA 13.2 index has no torch older than
  2.12.0.
- A container for the AGX's VLA path needs a base image checked on r39; none
  has been.
### 2026-11-03 (G) — the grant's mission style flown through MISSION_ITEM_INT, a GCS-set GeoFence, per-topic SROS 2 permissions, and the harness driving a flying rail

Four open items of the 29 Oct review, each against the grant's wording:
WP3-09 (the MAVLink adapter's MISSION_ITEM_INT row), WP3-10 (a GCS-set
GeoFence backstop), X-15 (SROS 2 permissions narrower than sros2's
allow-all) and WP1-15 / WP4-03 (corridor_swap in flight, and the harness,
not a scripted client, driving the events).

#### Added
- **The mission style (WP3-09).** The grant's MAVLink adapter table has a
  second row: "Pre-planned waypoint mission | MISSION_ITEM_INT | For
  mission-style flight; replaces or updates the next waypoint". Until now
  nothing on any rail wrote a mission item. `sitl/ros2_mission_node.py` is
  the Shield for that style: it checks a waypoint plan leg by leg with the
  Shield's own position check (`state_is_unsafe`, hard and enforced rules,
  every 0.5 m), replaces a leg that reaches an illegal step by a held point
  1.5 m short of it, and clamps the plan's speed to the policy's cap (the
  mission's DO_CHANGE_SPEED). `sitl/mavlink_adapter_node.py` writes the plan
  through `/mavros/mission/push` (home, DO_CHANGE_SPEED, one NAV_WAYPOINT
  per waypoint) and reads it back. In AUTO, at 10 Hz, the node re-checks the
  leg to the next waypoint under the rules in force; when a hot-applied zone
  makes it illegal, that one item is replaced in the autopilot by a partial
  write (MISSION_WRITE_PARTIAL_LIST, then the item as MISSION_ITEM_INT) with
  a held point short of the zone, and the original is written back once the
  leg has been legal for 2 s. `STYLE=mission` in `sitl/run_ros2_demo.sh` and
  `style:=mission` in the launch file run it in the Shield's place and
  enclave, with no VLA node; the bag adds `/shield/mission`,
  `/shield/mission_update` and MAVROS's mission topics; the run writes
  `mission.json`.
- **A GCS-set GeoFence (WP3-10).** The grant: "GCS-set AP GeoFence
  parameters (FENCE_ENABLE, FENCE_TYPE, FENCE_ACTION) remain configured as a
  last-line backstop". `sitl/gcs_backstop.py` is the ground station's step,
  a process of its own that runs before MAVROS and the Shield on the
  router's GCS endpoint (`GCS_BACKSTOP=1`: SITL boots without the fence
  defaults file, the endpoint goes to 127.0.0.1:14560). It speaks as the GCS
  (system 255, component 190), reads the fence parameters as they stand,
  sets FENCE_TYPE, FENCE_ACTION, FENCE_ALT_MAX (raised above the policy's
  ceiling + 5 m when the operator's file sits below it), FENCE_MARGIN,
  AVOID_ENABLE and then FENCE_ENABLE, reading each back, and uploads the
  signed bundle's keep-out zones with the mission protocol (fence type,
  MISSION_ITEM_INT), reading the list back item by item. Everything goes into
  `<out>/gcs_backstop.json`; the run is refused if any value or item does
  not read back as set. On such a run the adapter uploads no zone
  (`--fences-by-gcs`), and the Shield node changes no fence parameter and
  records the GCS's read-back beside its own (`hil_evidence.autopilot_fence`:
  `set_by: gcs`, `gcs.matches_autopilot`).
- **Per-topic SROS 2 permissions (X-15).** `sitl/sros2_policy.xml`, an
  sros2 access-control policy with one entry per enclave: the Shield's
  enclave alone may publish `/shield/setpoint`, the VLA slot's alone
  `/vla/action_4d`, the adapter's alone `/mavros/setpoint_raw/local` among
  the Guardrail's, MAVROS its own `/mavros/*` and `/uas1/*` (wildcards), the
  bag recorder nothing but its own events; each node calls only the MAVROS
  services it uses. `tools/sros2_setup.sh` signs each enclave's permissions
  from it with the keystore's permissions CA (the lab CA), verifies each
  signed file and refuses one that still grants `rt/*`; `--permissions-only`
  re-signs an existing keystore. `tools/sros2_check.sh` gains check 7 (a
  process in the VLA's enclave publishing `/shield/setpoint`, and one in the
  Shield's publishing `/vla/action_4d`, are refused at the publisher in the
  security layer's words, their nodes up) and check 8 (the signed
  permissions: no blanket grant, one publisher enclave for each of the two
  topics). A flight records each process's `permissions_scope` and the
  SHA-256 of its `permissions.p7s` (`guardrail.manifest.ros2_security_evidence`).
- **The harness drives a flying rail (WP4-03).** `tools/harness_client.py
  drive` opens a session bound to the rail's REST hot-apply endpoint,
  injects the bundle the rail flies (pinned to its hash; InjectPolicy checks
  the endpoint's /health reports it), waits for the rail's own
  `mission_start`, and sends a timeline of HotApply at mission time, which
  the harness POSTs to `<endpoint>/hot_apply`; every request, answer and
  streamed harness event is a line of `<tag>.harness.jsonl`.
- `tools/bag_mission_protocol.py`: the mission-protocol exchange decoded
  from a run's raw MAVLink topics (`/uas1/mavlink_sink`,
  `/uas1/mavlink_source`).
- `tools/render_track_video.py` draws corridors as their generation flies
  them, switched-off and scheduled zones as such, the mission style's
  waypoints, and the autopilot's mode.
- WSL: `~/venv-harness` (grpcio 1.85.1, protobuf 7.36.2, cryptography
  50.0.2, pydantic 2.14.2, shapely 2.1.2, numpy 2.4.6, jinja2, PyYAML) for
  the harness service, whose stubs need protobuf >= 7.35.1 (`~/venv-ros`
  has 4.21).

#### Changed
- The WSL keystore `~/sros2_keystore` carries the policy's permissions
  (backed up first as `~/sros2_keystore.before-policy-20261102T201841Z.tar.gz`).
  Any node that adds a publisher, subscription or service client must be
  added to the policy and the keystore re-signed, or Enforce refuses it.
- `render_track_video.py`'s axis labels: the rows are x North, y East; the
  frames said x east, y north (the drawing itself is unchanged). Today's
  earlier videos were rendered again with it.
- `docs/FINAL-REPORT-outline.md`: WP3-10 and X-15 move to DONE on these
  flights; WP3-09, WP1-15, WP4-03 and WP4-05 gain them as evidence and stay
  PARTIAL for what their rows name; WP3-07 gains G7 (Brake -> Normal, flown on
  `gcs_fence_on_20261103c`). The map reads 44 DONE, 79 PARTIAL, 12 DRIFT,
  1 MISSING (42 and 81 before).
- `docs/DESIGN-sros2.md` (the per-topic permissions and the 8/8 check) and
  `sitl/README.md` (GCS_BACKSTOP=1, STYLE=mission).

#### Fixed
- The first per-topic flight (`sros2_policy_on_20261103`) lost its bag: an
  rclcpp node's time source subscribes to `/parameter_events`, which the bag
  recorder's profile did not grant, and the recorder died at start
  ("rt/parameter_events topic not found in allow rule"). Granted; re-flown.

#### Flown (the ROS 2 rail in WSL: ArduCopter 4.5.7 SITL, mavlink-routerd, MAVROS 2.14, topology `dev`; from this worktree, so `code_revision` reads "unversioned")

Every run has its flight-log replay video, `demo/out/<tag>/<tag>_track.mp4`
(`tools/render_track_video.py`), and its rail log `<tag>.rail.log`.

| Run | Set-up | What happened |
|---|---|---|
| `gcs_fence_off_20261103` | GCS_BACKSTOP=1, Shield **off**, default mission to (30, 30) at 6 m/s, signed `fase3-sim-demo` 0.1.2 | The GCS step on 127.0.0.1:14560 found the stock values and set them: FENCE_TYPE 7 -> 5, FENCE_ACTION 1, FENCE_ALT_MAX 100 -> 30 (the policy's 20 m ceiling + 5 m is below the file's 30), FENCE_MARGIN 2, AVOID_ENABLE 3 -> 0, FENCE_ENABLE 0 -> 1, each read back; `nfz-square` uploaded as 4 MISSION_ITEM_INT (ArduPilot asked with MISSION_REQUEST), 4 read back, equal. The Shield node read the same values through MAVROS (`matches_autopilot` true). **"Fence Breached" at 3.37 s, GUIDED -> RTL**; 6.4 s inside the zone (the RTL flew home across its corner); P0 escape 1.0 (Shield off). |
| `gcs_fence_off_20261103b` | the same at 3.5 m/s | **"Fence Breached" at 4.08 s, GUIDED -> RTL**; 5.4 s inside the zone. |
| `gcs_fence_on_20261103` | Shield **on**, 6 m/s | X1 and G3 at 0.11 / 0.21 s on `nfz-square` (repair 8.14 m against theta 2 m: the 5 s forecast from home reaches the zone), BRAKE, G5 RTL at 30.3 s, home: no breach, P0 escape 0.0. Its rosbag was never closed (Found). |
| `gcs_fence_on_20261103b` | Shield **on**, 3.5 m/s | The same: X1 / G3 at 0.11 / 0.21 s (repair 6.37 m), RTL at 30.3 s; no breach. The aircraft hardly left home on either Shield-on run, so neither tests the fence near a zone; the next one does. |
| `gcs_fence_on_20261103c` | GCS_BACKSTOP=1, Shield **on**, to (40, 15) at 2 m/s: a route past the zone's corner | The Shield bent the route around `nfz-square` (G1 Normal -> Brake at 4.6 s on a 0.04 m repair, **G7 Brake -> Normal** at 12.7 s after 2.09 s clear); the aircraft passed 1.58 m from the zone and reached the target at 22.4 s, landed. **No breach** - the GCS-set fence stayed silent with the Shield working beside the zone; P0 escape 0.0, no unsafe tick. |
| `harness_corridor_20261103` | the signed `corridor-survey` 0.1.1 (a 40 m corridor along (0,0)-(60,0)-(60,60)), to (60, 0) at 3.5 m/s, REST on 127.0.0.1:8071, the harness in WSL | The harness refused to inject an unpinned policy (`expected_hash is required`), so no event was sent: the flight went to the target with no event, 17.8 s. |
| `harness_corridor_20261103b` | the same | Stopped on the pad by hand (the same refusal; the driver script had not been fixed): no flight log, so no video. |
| `harness_corridor_20261103c` | the same, the bundle's hash pinned | **The harness drove the events.** gRPC over mTLS with the lab CA in `~/harness_pki` (the CA the SROS 2 keystore was seeded with), a session bound to the rail's endpoint, InjectPolicy matching the endpoint's hash, then four HotApply at mission time, each POSTed by the harness to `/hot_apply` (61-108 ms per call): `dynamic_nfz` `nfz-hot-1` across the route at 2.0 s (generation 1, receipt to swap 1.814 ms), `time_window_switch` off at 9.0 s (2, 0.761 ms), `corridor_swap` to a 10 m corridor ending at x = 40 at 15.0 s (3, 0.765 ms), `corridor_swap` back to the survey route at 26.0 s (4, 0.711 ms). The FSM: G1 at 2.9 s and G3 at 3.4 s on the zone (the aircraft held at x 13, short of the zone's margin at 23), G8 at 11.2 s, X1 and G3 at 15.1 / 15.2 s on `swap-route-short` (held at x 29.2, inside the swapped corridor, short of its end), G8 at 28.2 s; target reached at 37.6 s, landed. P0 escape 0.0, no unsafe tick. Each generation's policy and CSP written; the harness's event stream carried `hot_apply_forwarded` four times. |
| `mission_wp_20261103` | STYLE=mission, GCS_BACKSTOP=1, waypoints (0, 30) and (30, 30), speed 6 | Refused on the pad: the node waited for the adapter's upload before arming, but ArduPilot sets its home at arming and the adapter converts the plan about home. Fixed (the upload is awaited after the climb). No flight log, no video. |
| `mission_wp_20261103b` | the same | Uploaded and read back (4 items), AUTO; the first replacement raised a TypeError in the node's event record (a duplicated `t`), and the node's own fail-safe landed it at 1.6 s. Fixed. |
| `mission_wp_20261103c` | the same, the harness sending the zone | Flown as intended (below), but the adapter reported both updates failed: it took MAVROS's push result, and MAVROS 2.14's partial push always ends in an ArduPilot timeout (Found). The bag shows both items written and flown. |
| `mission_wp_20261103d` | the same, the adapter's verdict now its read-back | **The mission style, end to end.** Pre-execution: the plan legal, the speed 6 m/s clamped to the policy's 4 m/s cap (DO_CHANGE_SPEED 4). Upload: 4 items, read back, equal. AUTO from 0.0 s ("Mission: 1 ChangeSpeed", "Mission: 2 WP"). The harness spawned `nfz-hot-1` across the first leg at 1.5 s; 85 ms later the node replaced item 2 (0, 30) by a held point (0.003, 15.124) - "leg to (0, 30) breaks nfz-hot-1 at 16.84 m" - and ArduPilot answered "Auto mission changed, restarted command"; the aircraft came to rest at the held point and never went past y 15.35 m (the zone's margin begins at 17 m). The harness switched the zone off at 12.0 s; after 2.1 s of a legal leg the node wrote (0, 30) back, ArduPilot restarted the command, and the mission went on: item 2 reached at 20.6 s, item 3 (30, 30) at 28.1 s, LAND. Both updates confirmed by read-back (MAVROS's push result false both times). The bag (`/uas1/mavlink_sink`) carries 6 MISSION_ITEM_INT to the autopilot and no MISSION_ITEM: 4 for the upload, one per update (`mission_protocol_from_bag.txt` in the run folder, from `tools/bag_mission_protocol.py`). No unsafe tick; the node's checks took at most 9.4 ms a tick. Its replay pack was refused (`metrics.json` lacked the episode id; fixed after this flight). |
| `sros2_policy_on_20261103` | SROS2=1 with the per-topic permissions, Shield on, 3.5 m/s | Flew under Enforce (every Guardrail node and MAVROS in its own enclave; the Shield node records `permissions_scope: per_topic`): X1 / G3 at 0.11 / 0.21 s, RTL at 30.3 s. **The rosbag recorder died at start** (`rt/parameter_events topic not found in allow rule`), so the run has no bag. Fixed in the policy. |
| `sros2_policy_on_20261103b` | the same, the recorder's permission granted | The same flight with its bag (every recorded topic, `/uas1/*` included, 21020 MAVLink frames from the autopilot) and its replay bundle with the bag; fence upload and mode requests through MAVROS's services under the per-topic grants; `permissions_scope: per_topic`, the lab CA's fingerprint 69:73:B8:BA:...; P0 escape 0.0. |

None is KPI-grade: `dev` topology without the PI's waiver, and a worktree
run records `code_revision` "unversioned" (the worktree's `.git` file names
a Windows path WSL's git cannot follow).

#### Found
- **MAVROS 2.14's partial mission push is off by one against ArduPilot
  4.5.7.** It sends MISSION_WRITE_PARTIAL_LIST with end_index = start_index
  + count (here 2, 3); MAVLink and ArduPilot read end_index as the last
  index included, so ArduPilot asks for item 2, gets it as MISSION_ITEM_INT,
  writes it and restarts the command it is flying, then asks for item 3 nine
  times, which MAVROS refuses as out of range, and ends the upload with
  MISSION_ACK 15 and "Mission upload timeout" after 8 s; MAVROS reports the
  push failed. Decoded from `mission_wp_20261103c`'s and `_d`'s bags. The
  item does land, so the adapter's verdict is its read-back; the 8 s window
  would refuse a second update sent inside it.
- **ArduPilot 4.5.7 asks for every uploaded item with MISSION_REQUEST, not
  MISSION_REQUEST_INT** (MissionItemProtocol::queued_request_send). MAVROS
  2.14 answers with MISSION_ITEM_INT anyway ("Trying to send a
  MISSION_ITEM_INT instead", in its plugin), as the GCS step does: no
  MISSION_ITEM went to the autopilot on the decoded flights.
- **ArduCopter restarts the item it is flying when that item is rewritten**
  ("Auto mission changed, restarted command"). A first version of the
  adapter followed each replacement with MISSION_SET_CURRENT, on the reading
  of AP_Mission::update alone, which only notes the change; ModeAuto acts on
  it. Removed before any flight relied on it.
- **The 29 Oct "unexplained" Shield-on fence breach is explained by its own
  records.** `ros2fix_on_policy55` flew `poly-test`, whose `tri-nfz` has a
  vertex at home (0, 0): at take-off the node held the zone back
  (`zones_over_home: [tri-nfz]`), uploaded it at 2.0 s once clear, the FSM
  went to RTL at 5.2 s, the mission ended at 10.2 s with the aircraft at
  (0.0, -1.9) in RTL, and "Fence Breached" came at 26.5 s: the autopilot's
  RTL landing in a zone that covers home, which the adapter warns of. With a
  GCS-set fence the GCS step holds a zone over the pad back the same way,
  so it cannot fire there - and nothing backstops that zone.
- With the Shield on, both default-mission flights held the aircraft within
  0.2 s of the mission start (the 5 s forecast from home reaches
  `nfz-square`), so they show only that the fence stays quiet when the
  Shield stops early; `gcs_fence_on_20261103c` approaches the zone.
- **The harness injects only a pinned policy**: InjectPolicy without
  `expected_hash` is refused, by design; the first corridor flight flew
  without its events because the driver script sent none.
- **Editing `sitl/run_ros2_demo.sh` while it runs breaks that run**: bash
  reads a script as it executes it. `gcs_fence_on_20261103` ended with
  "syntax error near unexpected token `fi'" after the Shield node finished,
  the trap killed the bag recorder, and the bag was never closed (its replay
  bundle has no bag). Its other records are whole.

#### Limits
- `dev` topology and an unversioned worktree: nothing here is KPI-grade.
- `GCS_BACKSTOP=1` is opt-in; the default rail still loads the fence
  defaults file and lets the adapter upload the zones, so Mission Planner
  keeps the GCS endpoint. The GCS step is a scripted stand-in for an
  operator in Mission Planner, not Mission Planner (ARCH-10 is that item). A
  GCS-set fence is static: a zone hot-applied in flight is the Shield's
  alone.
- The mission style runs no escalation FSM, forecasts the straight leg (not
  ArduCopter's path), replaces only the next waypoint, writes no manifest,
  kpi.json or Shield audit, and flew without SROS 2 (its node's
  permissions are in the policy, not flown). Its only flown event was a
  dynamic zone across the leg and its switch-off.
- The harness forwards a session's HotApply to `/hot_apply`, which spawns a
  zone, switches a rule or swaps a corridor; it cannot move, rotate or scale
  a zone (PATCH /dynamic_nfz), does not start the rail or its episode
  (StartEpisode's SITL backend is reserved), and reads no telemetry from it.
- The harness ran in WSL with the lab CA of the SROS 2 keystore; the
  Windows harness's PKI is still another CA.
- Per-topic permissions are per enclave: the stub and the AerialVLA bridge
  share the VLA slot's, the Shield node and the mission-style node the
  Shield's. The Orin has no keystore yet.
### 2026-11-03 (E) — one rule-evaluation code path; the Shield node's buffer feeds the checker and moving zones are in the IR's R-tree; the way out of a tie no longer follows the ring's vertex order

#### Added
- `guardrail/shield.py` is the violation checker library the grant names
  ("there is exactly one rule-evaluation code path in the system", Safety
  Shield page, WP3-22 / X-13). `RuleClock` is the one rule clock (windows to
  the microsecond, time_window_switch, corridor_swap), built once per
  generation (`Shield.rule_clock`). The per-position verdicts are
  `zone_breached`, `in_zone_band`, `band_holds`, `corridor_offset`,
  `corridor_wide`, `corridor_in_band` and `corridor_breached`; `ZoneRuler`
  (`Shield.zone_ruler`) gives the distance `fence_distance` reports, and
  `Shield.enforces_hard` reads the FSM's own table. `escape_direction` is
  the way out of a zone (below).
- `SlidingWindow`, the Shield node's buffer (WP3-23). Each `filter()` tick
  admits its action and that action's predicted trajectory before anything
  is checked. The checker reads the trajectory from there: `_predict`
  yields the buffer's poses for the admitted (state, action). `TickRecord`
  gains `trajectory`, and `history` is the last 50.
- `PolicyIR.moving_index` and `MovingIndex` in `guardrail/ir.py` (WP3-23):
  an STRtree over the moving dynamic_nfz zones' reach boxes at one instant.
  The Shield rebuilds it per tick and horizon (`_Tick.moving_near`) and
  queries it beside the static tree in `_check_in` and `_repair_geofence`.
  `_reach` still asks the exact question, so the same zones are judged.
- `tests/test_shieldcore_onepath.py` (in the CI loop), 10 tests:
  - the compiler's clock IS the Shield's;
  - the minute-grained scope reading equals `models.active_at` at every
    minute of a week (4 offsets, 6 windows);
  - a patched `RuleClock` moves the Shield, the CSP, the fence export and
    the dwell count;
  - an AST scan, with a planted-copy null, fails if a second evaluator is
    defined or a wrapped primitive is called outside the Shield;
  - a tampered buffer blinds the checker;
  - the moving R-tree returns exactly the zones whose reach box meets the
    query, and changes no decision against the one-by-one test.
- Invariance tests for the tie: `tests/test_shield_events.py` (all eight
  vertex orders, at four rotations, and a spawn-on-top flown 8 s) and
  `tests/test_sweep.py` (the library cell flown with its ring in every
  order). Both fail on 6b80775.

#### Changed
- The copies now call the Shield's evaluator:
  - `compiler._RuleClock` is a subclass that only samples and words. The
    compiler's minute-grained scope test (`_activity`, `_in_force_sets`)
    reads the same clock at the minute's start (`RuleClock.minute_on`),
    which is `active_at`'s answer.
  - `compiler.action_violations`, the relevance facts, the lint's
    start-inside test and the cost map's band test use the Shield's
    verdicts. A corridor is now judged with the Shield's 1e-9 m slack, so
    a point within a nanometre outside reads inside, as the Shield reads it.
  - `fence_export` reads its time controls and `in_force_now` from
    `RuleClock`.
  - `replay.zone_dwell` uses `Shield.rule_clock`.
  - `rtl_path` selects zones and bands through `Shield.enforces_hard` and
    tests "already inside" with `zone_breached` on the Shield's ring.
  - FenceGuard's `gate` and `slide` measure with `Shield.zone_ruler()`.
- The equality tests became identity tests: `tests/test_csp.py` (the
  compiler's `on` is the Shield's), `tests/test_fenceguard.py` (a patched
  ruler moves the guard).

#### Fixed
- **The escape from a zone depended on the order of its vertices**
  (coordinator's report of a regression after the WP1 merge). At an exact
  tie, such as a zone spawned centred on the aircraft, push_out_direction
  takes the first nearest point along the ring. The canonical ring order
  of 2 Nov moved that choice. `dyn-nfz-spawn-on-top@size_m=40,variant=0`
  took a side edge, the pilot's forward command stopped counting as
  escaping, and the episode ended in Loiter, censored.
  `escape_direction` returns push_out_direction's own answer wherever the
  nearest point is unique. In a tie it takes the edge the commanded
  heading is going to, then the first along the ring's canonical order,
  recomputed so the stored order does not matter. A hover therefore
  answers as the canonical rings did. Every escape direction in the Shield
  uses it.
  - Library: @40 fail -> pass, 10.6 s and 107 repairs -> 5.3 s and 0
    repairs, the published figure from before 2 Nov.
  - @20 now leaves forward as well: 5.6 s, 77 repairs and a missed goal
    -> 2.8 s, 0 repairs, success. Its old escape was the ring order's
    choice too.
  - `test_what_the_autopilot_flew_through_is_counted_apart_and_reported`
    recreates the side escape it needs by spawning the same zone 0.5 m to
    the side.
  - @40 flown for 50 s (the coordinator's second case, where it RTL'd with
    the escape censored): 5.3 s to safe, no FSM edge, not censored.
  - The full CI loop found two more tests standing on the same tie.
    `tests/test_manifest.py` (the audit record's restamp after a hot apply)
    and `tests/test_railinputs_clock.py` (the rail's clock) put the
    aircraft at a square zone's exact centre, commanding 1 m/s toward an
    edge, and needed "inside and not escaping". That command now leaves
    through the edge it heads for. Their aircraft is moved 0.5 m off the
    centre, where the nearest edge is unambiguous.
- FenceGuard measured with `Shield.zone_ruler()` even when a subclass gave
  it its own zones through `polys` (`tests/test_follow_motion.py`'s
  `_ZonesGuard`). `FenceGuard._ruler()` now measures those rings with the
  same ruler (`ZoneRuler.of_rings`), and the Shield's zones otherwise.

#### Measured (desktop, not KPI-grade)
- Library sweep, `--no-bundles`, before (6b80775) and after:
  - 153 -> 154 passed, 1 -> 0 failed (spawn-on-top@40), 22 tracked and
    4 known failures unchanged;
  - 178 of 180 episodes identical (the two spawn-on-top cells changed),
    and every commit after the tie fix 180/180 identical;
  - P0 escape 0.0 over 179 Shield-on episodes, before and after. The true
    state is unchanged: 1 Shield-on episode into a P0 polygon (tracked),
    4 polygon entries by the autopilot, 1768 P0 ticks;
  - fail-safe labels 174/178 -> 175/179 (0.977528 -> 0.977654), Loiter
    censored 1 -> 0.
- `tools/profile_shield_tick.py` on the packed flights, pinned to the
  performance cores, before and after in one session:
  - design_load / grant: `_check` p99 0.594 -> 0.548 ms, `filter` p99
    8.995 -> 8.685 ms;
  - design_load_velocity / grant: `_check` p99 2.073 -> 2.047 ms, `filter`
    p99 14.70 -> 13.98 ms (max 51.0 -> 58.5 ms);
  - replay agreement unchanged (219/219, 309/309, 160/160).
- `experiments/bench_shield_50rules.py --variant moving` (48 moving zones),
  pinned. `_check` median clear 0.215 -> 0.278, grid 0.278 -> 0.391, near
  0.325 -> 0.419 ms; `filter` median 0.391 -> 0.461, 0.579 -> 0.633, 1.189
  -> 1.387 ms. The bench makes a new tick per `_check`, so each call
  rebuilds the tree (about 70 us for 48 zones). Decisions identical to the
  all-zones null on 3900 of 3900 samples.
- The whole CI loop (`tools/ci_local.py --no-smoke --no-summary`), one file
  at a time, at b52e7f2: 136 of 138 files, 3447 of 3560 tests, claims scan
  clean. Two files failed, and neither reads the code changed here:
  - `test_track_truth`, a known lab-only failure
    (docs/data/ci_local/b3796cb1c673.json). It runs only with the lab's
    flights present: three were copied into this worktree's gitignored
    demo/out so that `test_replay` runs (61/61) instead of erroring.
  - `test_wp4outputs_harness`, `test_an_idle_session_expires_and_frees_its_slot`.
    It also fails at 6b80775 on this machine now: `start_episode` takes
    0.26 s against the test's 0.3 s session TTL. It passed in the run at
    aa024dc.

  That earlier run (aa024dc) found the three files fixed above.

#### Found
- `fence_export` skips a dynamic_nfz whose `motion` is present but all zero
  as moving, while the IR and the Shield treat it as static. That is the
  safe direction for a backstop: the zone is not uploaded and the Shield
  alone holds it. Left as is.

#### Limits
- WP3-22 is PARTIAL, not DONE. These still measure zone geometry of their
  own, because their question is not a rule verdict:
  - the compiler's mission-path distance for the risk grade and the
    relevance text (`_fence_dist`, `_seg_poly`);
  - the lint's exit regions (`_ring_of`, `_exits_for`);
  - the cost map's raster;
  - the RTL leg's crossing test;
  - FenceGuard's building half: its own occupancy-grid scan against
    `min_clearance_m`, not the Shield's distance field;
  - the behaviour-cloned pilots' input features (`training/features.py`).

  Folding the last two changes flown behaviour, which needs flights.
  `models.active_at` is still defined, and nothing in the system calls it.
  The source scan works by name and shape: a copy under a new name is not
  caught.
- No DSL rule reads past actions, so the buffer's 49 older entries are a
  record. Corridors, stand-offs and clearance are not in a tree: they are
  keep-in or subject-relative and are checked at every pose.
- One file of another stream was touched: `guardrail/ir.py`, with two new
  definitions (`MovingIndex`, `PolicyIR.moving_index`) and one docstring
  sentence. `guardrail/projection.py`'s `ring_order` is imported, not
  changed.
- The tie-break reads the COMMANDED heading, not the velocity relative to
  a moving zone. That keeps "a hover in a drifting zone's middle is not
  escaping" (`tests/test_shieldcore_identity.py`).
### 2026-11-03 (F) — WGS84 is the stored form of every policy; distance_envelope enforced; MSL flies over a stated ground

#### Added
- `tools/convert_policies_wgs84.py` (WP1-14): rewrites a policy written in
  local metres as WGS84 - a stated `origin` and every point as lat/lon about
  it - and proves, per file, that it enforces the same geometry: every
  point round-trips within 1 um (unrounded error at most 5.5e-8 m over 257
  points) and the policy's view gives back the old metres exactly; the
  converted file's metre form at the old version has the old canonical
  bytes, so the same hash, IR 2.x forms and 16-hex forms; zones, tubes,
  start conflicts, flight problems and runtime notes unchanged. Record:
  `docs/data/wgs84_conversion.json`. Origins: the ArduPilot SITL home (CMAC,
  -35.363261, 149.165230) for the three policies flown on the SITL rails,
  whose MAVLink adapter refuses zones more than 5 m from home; the Project
  AirSim scenes' home-geo-point (35.6895, 139.6917) for the other 28.
- `Policy.metre_form()` and `bundle.rewind` / `FRAME_FORM`
  (`sha256-canonical-v3/local-metres`): a converted file answers for its
  metre versions; `policy_candidates` walks the chain through it, so a run
  flown under any metre version resolves to the file it flew.
- `guardrail/terrain.py` (X-11): a ground-elevation source for MSL rules.
  `FlatGround(elevation, source)`, the rails' own home altitudes
  (`SITL_CMAC` 584 m, `PAS_SCENE` 40 m, `scene_ground(scene)`), and the
  conversion of an MSL band to the AGL band over it.
- `Shield(road_map=...)` (WP1-07): a road field (the street mask,
  `demo/build_street_mask.py`) built and read exactly as the obstacle field.
- `tests/test_distance_envelope_msl.py`: 12 tests, in the CI loop and its
  header (`.github/workflows/tests.yml`). Run on Windows under 3.10
  (vla-real) and 3.11 (vla-drone), 12/12 each; not yet in the WSL job venvs
  the header describes for earlier additions.

#### Changed
- **IR schema 3.1.0** (docs/IR-MIGRATIONS.md). A point's `x` / `y` are
  optional: a WGS84 point stores lat/lon only, and its metres are a view the
  Shield or the compiler computes when it first reads them, from the frame
  origin the policy binds it to, cached per origin and rounded to the
  micrometre (`models.XY`, `projection.local_metres`). Nothing is projected
  at load. Every 3.0.0 document validates; every policy file hashed as
  under 3.0.0 before the conversion (all 32 hash forms compared).
- **The 31 metre policies are WGS84** and raised their patch version:
  `fase3-sim-demo` 0.1.2 -> 0.1.3, the other 30 0.1.1 -> 0.1.2
  (`wgs84-taipei-demo` stays 0.1.1). The lock gained 31 entries and lost
  none; each names its metre version in `migrated_from`. The scenario
  generator writes WGS84 (`--metres` for the old form); the Orin launcher
  defaults to `follow-car-citylife-nfz-v0.1.2`.
- **distance_envelope is enforced** (WP1-07; RUNTIME_TYPES, the Shield's
  ENFORCED_TYPES). The Shield enforces it as the runtime rule of its class,
  under the rule's own id, priority, action and schedule: buildings as a
  clearance rule on the obstacle map, roads as one on the road map, people
  as a stand-off from the tracked subject while it is classified as a
  person. A buildings or roads envelope without its map is refused at
  construction - never inert. The compiler renders it (three templates);
  the city rail passes the street mask when a policy needs it.
- **MSL is an opt-in that names its ground** (X-11): the loaders take
  `ground=`, the Shield `ground=` and enforces an MSL band as the AGL band
  over it; a hot-apply follows the Shield's ground. Without a ground MSL is
  refused, as before.
- shield.py, for the other stream refactoring it: imports, ENFORCED_TYPES,
  `_field_name`, `_runtime_rule` / `_build_runtime`, the constructor
  (`road_map`, `ground`, `require_maps`), `unenforceable(ground)`,
  `_field_for`, the `ground` property, a field argument on `_distance_at`,
  `_away_dir`, `_clear_min_dist`, `_best_clear_dir`, `_clear_forecast`,
  `_check_clearance`, the clearance block of `_check_in`,
  `_repair_clearance`, the geofence slide's clearance side check,
  `_depth_terms`, the rule clearance table, `rule_status`, `_commit`. With no
  road rule every call reads the obstacle map as before. compiler.py: a
  relevance branch, a cost-map branch, the by-type order and
  `require_maps=False` on its two in-force Shields. ir.py:
  `PolicyIR.from_policy(rules=...)`.

#### Fixed
- `tests/test_policy_hud.py` pinned `v0.1.0` and failed since IR 3.0.0 (it
  is not in the CI loop); it reads the file's own version.
- Tests that took a shipped file as "a policy in metres with no origin"
  (test_api, test_bundle, test_csp, test_geo_to_policy,
  test_policy_dsl_grant_form, test_scenario_spec) use its metre form;
  test_window_opens_on moves the survey corridor's point in lat/lon (one
  rule, one frame); the version pins of test_policy_hash, test_csp and
  test_railinputs_policy name the converted versions; tests that used
  distance_envelope as "the type no loader accepts" use an MSL height with
  no ground.

#### Found
- The full CI list, once, sequentially (`tools/ci_local.py --no-smoke
  --no-summary`, vla-real, at 0c0782b): 135 of 138 files, 3446 of 3559
  tests, claims scan rc 0. None of the three failures is this change's:
  test_harnesstruth_bundles fails the same way on the base 6b80775 (the
  library's `dyn-nfz-spawn-on-top@size_m=40` cell ends in Loiter, censored,
  so the sweep it runs exits 1); test_track_truth failed in the lab
  checkout's CI at b3796cb too (the known lab-only failure); and
  test_wp4outputs_harness's idle-session test (a 0.3 s wall-clock TTL)
  passed in this branch's first full run and fails on the base code in the
  same session.
- Cost: the Shield's tick did not move (median 1.84 vs 1.85 ms, 1500 ticks
  of follow_car_citylife_nfz3b, base vs this code, beside a running CI
  loop); `policy_hash` 40 -> 53 us for that policy (the point serialiser).
- The library sweep did not move: before and after, P0 escape 0.0 over the
  Shield-flown ticks of 179 shield-on episodes (null passthrough 0.244922,
  shield-off control 0.24), fail-safe labels 174/178, every episode's P0
  count, status, repairs and outcome equal. One episode on
  `wgs84_taipei.yaml` moved in the 7th decimal of a repair sum (16.702743 ->
  16.702742): that policy's view is now rounded to the micrometre.
- Stored runs in the lab checkout (read-only, 4 Nov): 386 of 396 manifests
  resolve directly with the base code and with this code, each to the same
  id@version and the same content (the 27 flown at a 0.1.1 version, the AGX
  flights among them, through the converted file's metre form); the other
  10 (ros2_hotapply_*, ros2_action_*, gazebo_shield_on_dynamic_20261102)
  changed their policy in flight and resolve through their own generation
  chain, 10 of 10 with either code, to `fase3-sim-demo@0.1.1`.
- The 19 bundles in the lab checkout's `bundles/` verify with this code; a
  pre-3.1.0 WGS84 bundle's multi-scale geometry is read within half a
  micrometre of the rounded view.
- The 13 flight bundles rebuilt with the lab key (into a scratch folder)
  load through `load_for_flight` signature-verified, and through each rail's
  path: the SITL policies' origin is 0.0 m from SITL home (the adapter
  uploads), every CityLife policy anchors the PAS rail at its scene's home
  (the identity), the city and vla_fly Shields build, every CSP compiles.
  `wgs84-taipei-demo` anchors in Taipei, so on a PAS scene its zone is where
  WGS84 puts it, about 2000 km away.

#### Limits
- people: the Shield knows one subject, so a person who is not the tracked
  subject is outside the rule. Buildings and roads exist where the maps
  cover. vla_fly and the PAS rail pass no road map, and load their obstacle
  map only for an obstacle_clearance rule, so a roads or buildings envelope
  is refused there (loudly), not flown. guardrail/kpi.py's blind-tick count
  (master, 4 Nov) assumes every rule on the obstacle map is P0; a P1
  distance_envelope over buildings would break that.
- MSL: a flat ground only (a varying one is refused); no rail passes a
  ground yet, and the CSP still refuses an MSL rule.
- A metre document still loads: the legacy read form for pre-2026-11-03
  bundles, runs and lock recipes, and what the REST API, the Shield's own
  zone edits and the stress harness's spawned zones write. Stored artefacts
  need it; new input could be converted at ingest.
- No simulator flight of a converted bundle (none was run here); the Orin
  needs `follow-car-citylife-nfz-v0.1.2` synced. `sitl/` usage text still
  names `fase3-sim-demo-v0.1.2` (left to the sitl stream);
  `docs/data/wp1_roundtrip.json` was not regenerated.

### 2026-11-03 — the 2 November branches merged; the flight bundles rebuilt for the new policy versions

#### Changed
- Three branches of 2 November are on master: the Policy DSL / IR / bundle
  work (IR schema 3.0.0), the Paraphraser service, and the audit and
  episode records. They merged in an integration worktree, where the whole
  CI loop ran one file at a time (`tools/ci_local.py`): 134 of 136 files
  passed, and both failures came from the branches meeting (Fixed, below).
  The smoke profile on the merged code scored fail-safe trigger correctness
  0.934 against a never-trigger null of 0.885 (headless, not KPI-grade).
  The fail-safe branch merged afterwards (f04bfd0); its library, smoke and
  nightly figures (0.978 / 0.984 / 0.973) were scored before the WP1 branch's
  canonical rings, so they are re-run on the merged code.
- **The city rail flies the grant's 5 s / 0.1 s forecast by default**
  (`demo/follow_vlm.py`, WP3-05). The loop now gives `Shield.set_subject`
  the estimator's velocity, as vla_fly has since 1 Nov, so the forecast
  carries the car along; the Shield keeps its escalation FSM.
  `--legacy-forecast` flies the 3 s / 0.5 s of every earlier city flight,
  `--grant-forecast` the grant forecast with the FSM off.
- `tools/build_flight_bundles.py` signed the bundles of the new policy
  versions with the lab key: `fase3-sim-demo-v0.1.2`, and `v0.1.1` of
  corridor-survey, follow-car, follow-car-nfz, follow-car-citylife,
  follow-car-citylife-nfz, -nfz3, -nfz3b, follow-pedestrian,
  vla-goto-citylife and wgs84-taipei-demo. The earlier bundles stay in
  `bundles/` for the runs that flew them. `deploy/orin/run_vla_fly.sh`
  defaults to `follow-car-citylife-nfz-v0.1.1`, and three usage examples
  name the new versions.

#### Fixed
- The audit branch's new harness-bundle test hot-applied a
  `time_window_switch` without `constraint_type`, `priority` and
  `violation_action`, which the WP1 branch made required; it now states
  them.
- `sitl/run_ros2_demo.sh` had renamed its `=== fresh SITL ===` line (the
  `PHYSICS` switch of 2 November), which `tests/test_ros2_package.py`
  orders the SROS 2 check against; the line is back and the physics is
  printed on its own line.
- `tests/test_replay.py` packed `retarget_demo2` with today's
  `follow_pedestrian.yaml`, which since IR schema 3.0.0 is version 0.1.1 with
  another hash, so 15 tests refused to bind. They now resolve the run's own
  manifest hash through `policy_candidates()`, which reconstructs the
  migrated 0.1.0 content: 61/61 in the lab checkout.

#### Flown (Project AirSim CityLife, simple_flight, PC; not KPI-grade)
Three hand-written follows of the red car (Car_10) on the new default, at
051542e, in the 29 Oct comparison's configuration (policy
`follow_car_citylife_nfz` 0.1.1 from its rebuilt bundle, 420 s, offscreen
renderer at 30 fps): `fc5s_manual_r1_20261103` 0.980, `_r2_` 0.961, `_r3_`
0.947 within 30 m (mean 0.963), against 0.972 (n=5) on 3 s / 0.5 s on 29 Oct.
P0 escape 0.0, 0 mission collisions. Each has its PIP video
(`docs/video/fc5s_manual_r*_20261103_pip_small.mp4`).

#### Found
- **These flights never challenge the Shield, at either forecast.** The
  hand-written follow's own fence guard (`FenceGuard`: near / skirt / hold)
  keeps the aircraft off the zones, so the Shield forecast no violation on any
  tick of these three flights or of `rep3_manual_20261029` and
  `nfz3b_pc_hier_dronecats_20261028`. The forecast change cost the follow
  nothing measurable; it was not exercised against a zone here. The 5 s
  Shield acting on this simulator is shown by vla_fly's flights, whose pilot
  has no fence guard.
- **The exporter's bag path ran on real bags** (the audit branch had not
  installed `rosbags`): in a separate WSL venv with `rosbags` 0.10.11,
  `tools/export_episodes.py` took `executed` from the bag on
  `ros2_action_brake_01` (899 of 899 ticks agree with the flight log),
  `ros2_hotapply_03` (89 of 89), `ros2if_on_dyn_yaw` (242 of 242) and
  `ros2_sros2_shield_on_20261102` (2 of 2), and the rosbags round-trip tests
  of `tests/test_wp4outputs_exporters.py` passed instead of skipping.

### 2026-11-02 — Policy DSL, IR and bundle to the grant: the freeze rule, required ConstraintBase fields, canonical rings, shipped multi-scale geometry, the two-hash contract

#### Added
- `guardrail/reference_convert.py` and `python -m guardrail.bundle export
  POLICY --reference`: the reference implementation's form of what it can
  hold, every other rule named with the reason (WP1-05). Every manifest
  carries `reference_projection`; `check_bundle` recomputes it.
- `multiscale.json` in every bundle, digest in the signed manifest; a
  policy loaded from a bundle compiles its CSP from the shipped scales
  (WP2-16).
- Two negative-corpus cases for the geometry lint and one for a missing
  `violation_action` (38 cases).

#### Changed
- The IR freeze rule (WP1-22 / X-09): any IR-shape change, additive ones
  included, needs an `IR_SCHEMA_VERSION` bump and a row in
  docs/IR-MIGRATIONS.md; 2.2.0 versions `locked_by` (2385f84).
- IR schema 3.0.0 (WP1-01 / WP1-02): `constraint_type`, `priority`,
  `violation_action` required; the grant's six actions only (`repair` read
  as project_fix); rings stored in canonical order. All 32 shipped policies
  raised their patch version; the lock's `migrated_from` is re-proved by
  `check_lock`; 363/363 stored manifests and 8/8 stored bundles of the main
  checkout still resolve (read-only check). REST rule bodies must state the
  three fields.
- The lint refuses a hard corridor buried in hard keep-out zones and two
  hard corridors with no common point.

#### Found
- Ring order decided the Shield's escape in an exact tie: the sweep cell
  dyn-nfz-spawn-on-top now passes a hold point at 15.6 s.

#### Limits
- `scope` is still optional (the grant requires it). The signer stays the
  lab dev key (WP1-04 / X-10 pending).
- UNFINISHED (session stopped at the usage limit): the WP1-04 swap-drill
  test, docs/FINAL-REPORT-outline.md Appendix A rows, and the full
  sequential test-list run (only parallel discovery runs were made; all
  133 runners passed there except two load-sensitive timing tests that
  pass alone).
### 2026-11-02 — the Paraphraser becomes a separate gRPC service; a new probe set written before the validator was read; the validator's held-out misses fixed by mechanism; the four C8 sets re-authored

Audit WP2-13, WP4-10, ARCH-23. The work went in five steps:

1. A new probe set was written and committed (f366913) before the validator
   was read.
2. The misses on heldout_2026_10_28 were fixed by mechanism (1e79168).
3. Those fixes were measured on the new set: the held-out figure.
4. The new set's own misses were fixed (7f76691).
5. The stale sets were re-authored (b05cfbb) and the service added (01df1b1).

#### Added
- **`guardrail/paraphraser_service.py`: the Paraphraser as a separate
  service.** The grant calls it a "separate service" (Prefix Compiler page),
  in a control plane that is "gRPC + mTLS" (data-flow page). The service
  follows `harness_service.py`'s pattern:
  - a plain-Python core;
  - a servicer onto committed stubs (`guardrail/protos/paraphraser.proto`:
    `Paraphrase`, `Validate`, `GetStatus`);
  - mutual TLS from the harness lab PKI, with a client common-name allow-list
    (default `harness-client`); plaintext is allowed on loopback only;
  - a client, and a CLI (`serve`, `paraphrase`, `validate`, `status`).

  Templated NL goes in and validated paraphrases come out. Each paraphrase
  has passed `validate()` against the text sent, and is checked again before
  the reply goes. The reply carries the validator and store digests and the
  candidates refused on the way. The paraphraser's errors arrive as gRPC
  codes: an unknown source is NOT_FOUND, n beyond the set is OUT_OF_RANGE, a
  protected prompt is FAILED_PRECONDITION. No new dependency: the stubs use
  the harness extra's grpcio, protobuf and grpcio-tools 1.84.0 (gencode
  7.35.1). `tests/test_paraphraser_service.py` 8/8 checks five things:
  - the service returns the same ids over the wire as in process;
  - the error codes;
  - the stale-stub check;
  - plaintext is refused off loopback;
  - an off-list client is PERMISSION_DENIED, and a client with no
    certificate is refused at the handshake.

  The file is in the CI loop and its header, and in the two harness jobs'
  `no_skip`.
- **`experiments/paraphrases/heldout_2026_11_02/`: 94 probes** (50 meaning
  changes, 44 faithful) over 17 sources: the 14 current renderings, with the
  post-C8 wording, plus 3 policy fixtures the store never paraphrased.
  - **Who and when.** Written by Claude (claude-opus-5-5), the agent that
    then fixed the validator, before it read the validator, its tests, the
    design doc or any earlier probe. Committed before any measurement.
  - **README.** It records what the author had read and the labelling rule.
    The rule counts a stricter rule as a change too, and avoids
    strict-versus-non-strict swaps.
  - **Loading.** `load_probes` reads a set kept in a folder of its own.
- **Tests for every mechanism below.** Each uses wordings of its own and
  failed before its fix. `tests/test_paraphraser.py` 103/103.

#### Fixed
- **The mechanisms behind heldout_2026_10_28's seven misses** (1e79168):
  - two meaning changes it accepted:
    - a limit on one speed component ("horizontal speed");
    - a time zone on the window ("07:30-17:30 UTC");
  - five faithful wordings it refused:
    - one distance shared by two subjects ("from any building as well as
      from anything you are following", or "..., and from ...");
    - a "respectively" list (read both ways round; one that does not pair up
      is refused);
    - a relative clause that only restates the position ("the waypoint that
      lies 120 m ahead");
    - "someone" for "a person";
    - a negation carried across ", or".
- **The mechanisms behind heldout_2026_11_02's misses** (7f76691):
  - one meaning change it accepted: a count on the object ("follow two
    people");
  - faithful wordings it refused:
    - "20 m either side of the centerline" (read as a target);
    - the C8 stand-off said as a condition ("when the thing you are following
      is a pedestrian", "if it is a pedestrian, ...") and a topic picked up by
      "it";
    - "a pedestrian you follow";
    - zone lists that state their ruling once ("Keep out of zones 'a', 'b'
      and 'c'");
    - "fly at 35-55 m";
    - compass spellings of a registry place ("north-east pad", "the pad in
      the northeast");
    - a go-to verb with "at cruise altitude" before "to".
- **Two gaps the mutation test found on the re-authored sets** (b05cfbb):
  - "fly at 12 m to 28 m distance" was read as an altitude. That bug came
    from the step before.
  - "stay outside 3 m/s or less" passed as a 3 m/s cap.

#### Changed
- **The four sets stale since C8 are re-authored** (b05cfbb):
  csp_sitl_pedestrian, csp_follow_car_citylife, csp_follow_pedestrian and
  csp_follow_car_citylife_nfz.
  - **What was written.** 32 new paraphrases by Claude (claude-opus-5-5),
    keyed on compile_csp's current text. Each file's provenance names the
    set it replaces and says the author was validator-aware.
  - **Checked.** All 32 pass against both renderings, and `stale` lists 0
    texts without a set, of 21.
  - **Test changes.** `STALE_SINCE_C8` is empty. Two store tests now accept
    the new provenance and the four revision notes left.
- **Decision reversed: "north-east pad" names the registry's "northeast
  pad".** It was refused as a renamed place, and a test pinned that. It is
  the same pad to a reader; another compass point is still refused.
- **`docs/data/progress_oct.json`: the paraphraser section now says 3977
  mutants, not 3979.** `tests/test_build_progress_oct_data.py` requires that
  section to equal the live validation report. The re-authored sets yield 2
  fewer mutants. The October deck as sent on 27 October is unchanged; a
  rebuild would print 3977/3977. The other sections were not regenerated:
  `--verify` needs flight output that a worktree does not have.
- `docs/DESIGN-paraphraser.md` and `docs/FINAL-REPORT-outline.md` (Appendix A
  and section 4.2) say all of the above. **WP2-13 is DONE**: the separate
  service fed by the templated NL now exists, and the KPI is WP4-09.
  **WP4-10 and ARCH-23 stay PARTIAL**: nothing issues the paraphrases to a
  VLA, and the service landed after the Aug-Oct window. The totals are 41
  DONE and 80 PARTIAL. The WP2 and WP4 section counts in Appendix A were
  behind their tables (11/8, 3/17) and now match them (13/6, 4/16).

#### Found
| Probe set | false accepts | false refusals |
|---|---|---|
| heldout_2026_10_28 (held out until today) | 2/20 -> 1/20 (dev now) | 5/20 -> 0/20 |
| heldout_2026_11_02, before any change | 1/50 | 18/44 |
| heldout_2026_11_02, after the 10_28 fixes only (**held-out figure**) | 1/50 | 14/44 |
| heldout_2026_11_02, after its own fixes (dev) | 0/50 | 3/44 |
| review_2026_10_06 (dev) | 0/28 | 3/30 -> 2/30 |
| heldout_2026_10_06, _10_21, _10_21b (dev) | 0/30, 0/22, 0/12 | 2/30, 1/22, 2/12, unchanged |
| stored paraphrases accepted; mutants refused | 112/112; 3977/3977 | - |

- **heldout_2026_10_28's one remaining accept is a labelling disagreement,
  not a missed slot.** The probe offers "under 3 m/s" for "at or below
  3 m/s". Before today it was refused, but only because of the merged
  distance in the same candidate. The validator reads strict and non-strict
  bounds alike by design, and so do the stored set and the new probes.
- **heldout_2026_11_02's three refusals are all intended.** Two candidates
  are near-identical, below the novelty floor. The third folds zones into a
  range ("'nfz-edge-1' through 'nfz-edge-4'"), which is refused on purpose:
  two ids are no longer named.
- **Re-authoring found an open false refusal.** "The following limits apply"
  reads its adjectival "following" as the follow action. One of the 32 was
  reworded around it; the validator is not changed for it.
- **The full CI list ran here (vla-real, the worktree): 134 files, one
  failure**, `tests/test_wp4outputs_harness.py`
  `test_an_idle_session_expires_and_frees_its_slot`. It is a wall-clock
  test: a 0.3 s idle TTL kept alive by calls 0.15 s apart. It failed 2 of 3
  runs on this branch and 2 of 4 on the base commit 2290aff (exported with
  `git archive`), so it is a timing flake on this machine, not this change.
  The deck, tracker and replay tests SKIP in a worktree, as expected.

#### Limits
- **No held-out estimate exists for the validator as it now stands.**
  heldout_2026_10_28 was tuned on today and is a dev set. heldout_2026_11_02
  was held out only for the first round of fixes; its own misses were then
  tuned on. Its author also wrote the fixes, so it was never independent of
  the code. The next honest number needs a set written by someone else, after
  this commit.
- **The stress harness does not use the service yet.** It is the reference
  design's orchestrator, and it still draws paraphrase arms in-process
  (`harness_service.py` imports `guardrail.paraphraser`). Binding it to the
  service is a change in that module, not made here.
- **The per-paraphrase robustness KPI is still unmeasured.** No pilot reads
  the paraphrases in flight; that needs a text-reading VLA (WP4-09).
- **No person has read the stored paraphrases yet**, the 32 new ones
  included.
- **Unfinished when the session ended.** All five steps of the protocol are
  done and committed. Open are the items above, and the adjectival
  "following" false refusal (validator not changed).
### 2026-11-02 — the audit and episode records in the grant's words: the grant's audit field names carry the grant's content (record version 4, shield_audit.jsonl); the harness bundle carries policy_bundle.tar.gz; an exported step's `executed` is what MAVROS was sent, read from the rosbag2; the ROS 2 rail's bag records MAVROS's sensors and raw MAVLink

The independent review of 29 Oct found three of the grant's audit field
names carrying this project's content and the file under another name
(WP3-11), the exporter's `executed` inferred rather than read (WP4-13), the
harness bundle without the grant's file names (WP4-05) and the bag without
sensors or MAVLink (ARCH-22). The PI: follow the grant.

#### Changed
- **Audit record version 4** (`guardrail/audit.py`, WP3-11). The grant's ten
  fields (Safety Shield page, "Audit log") are written first, in the
  grant's order, with the grant's content:
  - `raw_action` / `emitted_action` are the grant's body frame `{vx, vy, vz,
    yaw_rate}` (vx forward, vy right, vz up; `frames.to_body` at the tick's
    heading), converted when the record is written; null only for a
    decision built by hand without a heading. The world-frame actions
    (`vz_up`) are `raw_action_world` / `emitted_action_world`.
  - A violation's `category` is the grant's (geometric / envelope /
    time-window; `contract`, a non-finite command zeroed, stays this
    project's extension). Ours (geofence, altitude, ...) is `rule_category`;
    `grant_category` is gone.
  - `repair_attempts` is the operator list the grant's example shows: every
    repair operator tried, in order, with result (ok / skip / fail /
    fallback / relaxed), the reason a skip skipped, `magnitude_m` and
    `iterations` (the pass it ran in), plus our `run`, `axis` and `repairs`.
    The repair records it held are `repair_results`; `repair_passes` is
    unchanged.
  The model refuses a world-frame action under the grant's name, a
  version-3 key beside it, and the old `grant_category`. The schema
  (`policies/audit_record.schema.json`) is regenerated.
- **`shield_audit.jsonl`**, the grant's file name, is what every rail writes
  (the ROS 2 node, the pymavlink rail, the ArduPilot rail, `vla_fly`, the
  demos; `guardrail.audit.AUDIT_FILE`); the per-episode logger writes
  `shield_audit_<id>.jsonl`. `guardrail/replay.py` packs and reads either
  name, moves a stale one of either name aside when an episode starts, and
  refuses a run folder holding both (it could tell neither which was the
  episode's nor check both).
- **The exporter's `executed` comes from the rosbag2** (`tools/
  export_episodes.py`, WP4-13). With a run's bag, read with `rosbags` (pure
  Python, no ROS, optional like pyarrow), each step's `executed` is the
  `mavros_msgs/PositionTarget` on `/mavros/setpoint_raw/local` logged in the
  tick's window (the `mission_start` event's wall clock plus the row's `t`),
  converted back from the adapter's ENU packing to the world frame; a tick
  with no message executed nothing. The flight log's verdict rides beside it
  (`executed_flight_log`), with the message's log time and lag
  (`executed_bag`). The join is used only when at least half of the ticks
  where both say something was sent carry the same command (within the
  adapter's heading slack); otherwise, and with no bag, no `rosbags` or a
  headless episode, `executed` stays the flight log's verdict and every step
  says so (`executed_from` "flight_log", `executed_fallback` the reason), as
  does the episode record (`executed`). The observation gains the pose
  MAVROS reported nearest the tick and the clock its bag time is on. Each
  audit record is read through `upgrade_record`, so the world-frame actions,
  both categories and the repairs made are read under their version-4 names
  whatever version wrote them. Export schema `vlaguard.episode_export/2`: a
  step's violation carries the grant's `category` and ours as
  `rule_category` (until now `category` was ours).
- **The ROS 2 rail's bag** (`sitl/run_ros2_demo.sh`, ARCH-22) also records
  MAVROS's velocity, IMU, GPS fix, relative altitude, altitude, battery,
  home, extended state and status text, the two camera topics the VLA bridge
  reads, and MAVROS 2's raw MAVLink both ways (`/uas1/mavlink_source` from
  the autopilot, `/uas1/mavlink_sink` to it). `deploy/docker/entrypoint.sh`'s
  `BAG_TOPICS` is kept equal (outside sitl/, by the test that compares them).
- `guardrail/repair.py` (another stream's file): one docstring paragraph,
  which said `repair_attempts` lists the repair records.

#### Added
- **`policy_bundle.tar.gz`** in every gRPC harness bundle
  (`guardrail/harness_service.py write_policy_bundle`, WP4-05): the injected
  policy file byte for byte, the canonical IR of generation 0 and of the last
  generation (each only where its number and hash are the ones the episode
  recorded), and `policy_bundle.json` with every generation's time, event,
  rule and policy_hash and every member's SHA-256. Deterministic. `kpi.json`
  `harness.bundle_files` names the grant's three files and which this
  backend writes; `episode.bag2/` is not written (no ROS 2 graph on the
  headless path), and it says so.
- `guardrail.audit`: `AUDIT_FILE`, `LEGACY_AUDIT_FILE`, `GRANT_FIELDS`,
  `audit_path` / `audit_files` (the new name first, a stored run's old one
  after it), `operator_attempts`, `is_version_4`.

#### Fixed
- **The exporter never read a headless bundle's audit.** It looked for
  `audit.jsonl` / `audit_<id>.jsonl`, and the harness has written
  `shield_audit.jsonl` since 30 Oct, so those steps took their policy state
  from the generation timeline and their labels without the record's
  priority and grant category. Now joined (270 of 270 records on the test's
  hot-apply flight).
- **The observation's bag time on the ROS 2 rail was the take-off time
  early.** Its rows count `t` from `mission_start`; the exporter added it to
  the `episode_start` clock (37.9 s earlier on ros2_hotapply_01). The
  `mission_start` event's wall clock is used where the event is at t 0.
- `tests/test_deploy_configs.py`'s topic pattern did not see a topic whose
  first segment holds a digit, so `/uas1/...` would have escaped the
  comparison of the two bag lists.

#### Found
- All 294 stored audit files under the main checkout's `demo/out` (71,064
  records: 22,123 of version 1, 1,836 of version 2, 47,105 of version 3)
  read into version 4, and upgrading twice changes nothing. 48,941 carry an
  operator list; the rest predate version 2 and read `repair_attempts` null
  (not recorded), never empty.
- The 19 stored ROS 2 runs with a bag: on every one, the bag's
  `/mavros/setpoint_raw/local` count equals the flight log's ticks that sent
  something (ros2_action_brake_01 899 of 899, ros2_hotapply_01 2 of 2,
  ros2if_on_dyn_yaw 244 of 244), so their bags can serve the new
  `executed`. `/mavros/setpoint_raw/target_local` (the autopilot's report of
  its target) holds 0 messages in all 19. None has a sensor topic beyond the
  pose, or raw MAVLink.
- `tests/test_wp4outputs_harness.py::test_an_idle_session_expires_and_frees_its_slot`
  fails on this machine this evening, on this branch and on 7bfd6dc alike:
  `start_episode` took 0.20-0.55 s against the test's 0.3 s session TTL (it
  passed in the afternoon baseline). Timing under load, not this change.

#### Limits
- **The rosbag2 path has not run on a real bag.** `rosbags` is not in
  vla-real (nor any env here) and was not installed: the round trip through
  a rosbag2 written and read with `rosbags`
  (`test_a_real_rosbag2_is_read_with_rosbags`) SKIPs, and the join, the frame
  conversion, the agreement gate and the fallbacks are tested through a
  stand-in for `read_bag`. The `rosbags` calls are written to its 0.10+ API
  without being run. Commands below.
- The new bag topics are not flown. `/uas1/mavlink_source` and
  `/uas1/mavlink_sink` are MAVROS 2's router link as its source names them;
  no run on this rail has listed them. The camera topics are empty on the
  SITL-only rail.
- No flown run has written a version-4 audit record yet; the rails write it
  in their tests. The Shield still repairs in the world frame (WP3-04): the
  body frame is a conversion when the record is written.
- `AuditRecord` keeps the world-frame actions and `rule_category` optional,
  so the grant's own example validates; the writer writes them on every
  record.
- `policy_bundle.tar.gz` holds the first and the last generation's IR; with
  two or more hot-applies the ones between are named by hash only (the
  headless runner keeps no copy), and the index says `complete: false`.
- A ROS 2 rail run keeps its bag as `bag/`, not the grant's
  `episode.bag2/`; replay re-scores from `flight_log.jsonl`, not the bag.
- Every rail but the ROS 2 one has no bag, so its exported `executed` is the
  labelled fallback.
- The tutorials and older design notes still name `audit.jsonl`
  (historical walkthroughs, not edited).

#### Tests
- `tests/test_shieldcore_audit.py` 16/16 (12 before; new: the grant's example
  validates and the writer's first ten fields are the grant's, a world-frame
  action under the grant's name is refused, a version-3 record reads with
  the grant's names and upgrades once, the file name and the stored-run
  fallback). `tests/test_wp4outputs_exporters.py` 31/38, 7 SKIP (6 pyarrow, 1
  rosbags; new: executed from the bag, the fallback's five cases, a replay
  bundle's bag, the heading formula, a version-4 audit's world-frame
  actions). `tests/test_harness_service.py` 57/57 (new: the policy bundle).
  `tests/test_replay.py` 41/61 (20 SKIP need demo/out; new: two audit file
  names refused). The bag topics: `tests/test_ros2_package.py` 15/15,
  `tests/test_deploy_configs.py` 62/62.
- The bag tests fail when the join needs 100 % agreement, north and east are
  swapped, the episode-start clock is used, or `executed` is left as the
  log's (mutations run once, 2 Nov).
- The whole loop, 133 files, one at a time in the worktree under vla-real:
  132 exit 0, 44 of them skipping the parts that need the lab's data or an
  optional package; 1 fails, `tests/test_wp4outputs_harness.py` 15/16, its
  session-TTL test (see Found: it fails on 7bfd6dc too under today's load).

#### To verify the bag path (WSL, or a clone of vla-real)
```bash
# WSL, in the rail's venv (it has the guardrail's dependencies)
cd "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone"
~/venv-ros/bin/pip install "rosbags>=0.10"
~/venv-ros/bin/python tests/test_wp4outputs_exporters.py      # the rosbags test no longer SKIPs
for r in ros2_action_brake_01 ros2_hotapply_03 ros2if_on_dyn_yaw; do
  ~/venv-ros/bin/python tools/export_episodes.py "demo/out/$r" --out "/tmp/exp_$r" --format jsonl
  ~/venv-ros/bin/python -c "import json,sys; e=json.loads(open(sys.argv[1]).readline()); print(e['executed'])" "/tmp/exp_$r/episodes.jsonl"
done
```
On Windows without touching vla-real: `conda create -n vla-rosbags --clone
vla-real`, then `pip install "rosbags>=0.10"` in it and the same two
commands with its python. Each episode's `executed` should say `"from":
"bag"` with `agree` close to `comparable`.
### 2026-11-02 — fail-safe correctness against a written rubric; theta decided

The PI delegated the open escalation decisions as "best practice / follow the
grant" (2 Nov). Headless harness throughout; nothing here is KPI-grade.

#### Added
- `docs/RUBRIC-failsafe.md`, written and committed before any label changed.
  - "Triggered" = RTL or Land; Brake and Loiter are holds.
  - A fail-safe is expected for: E1 a hard RTL / land rule; E2 a disturbance
    beyond authority; E3 a position error beyond the margin near a P0
    boundary; E4 no legal continuation; E5 an unsafe repair (over theta or
    unconverged, or a loiter rule) that the planner keeps asking for past the
    operator timeout.
  - N1-N5 are their negations. Clauses are judged on the flight without a
    fail-safe, never on what the FSM did.
  - Two clarifications came before any label changed: the
    flight-without-a-fail-safe basis, and N1 / N2 as exact negations. One
    amendment came after: E3's noise / latency is read against the planned
    route, because the flown closest approach gave one cell two labels by
    seed. No label changed with it.
- `tools/failsafe_rubric.py` applies the rubric cell by cell. On the
  library, smoke and nightly (672 cell-seeds) it reports 0 mismatches.
- Labels carry their clause: `ScenarioSpec.failsafe_clause` and
  `failsafe_cells` (`guardrail/scenario_spec.py`). A clause that contradicts
  its label is refused. `tests/test_failsafe_rubric.py` (7) is added to the CI
  loop.
- `FSMConfig.for_decision_period(p)` sets T = max(T, N x p x 1.5) for a pilot
  slower than 10 Hz. Every 10 Hz pilot keeps the grant's thresholds and hash.
- `summary()["ended_holding"]`, and a CENSORED fail-safe outcome in the
  sweep: an episode that ends in Loiter with the timeout still running is not
  scored, and it fails its label gate.

#### Changed
- **Theta's basis is DECIDED**: reading A, each repair's forecast depth in
  metres (the default). Code comments no longer say it waits for the PI.
- **Loiter's G8** hands back a request the Shield would fly (clean or a repair
  within theta, no loiter rule, no new onset) once the position is legal
  (`FSMConfig.loiter_clear` "acceptable").
- **X6 / X7 in Loiter** judge the held aircraft's position
  (`loiter_x67` "position").
- Both old readings stay selectable. A config on the old law hashes
  `sha256:5ae67785...` as before; the new default hashes
  `sha256:d55e1bfe...`.
- **Labels re-derived from the rubric**: 15 families changed or gained a
  label (list in the rubric's section 5), and 5 outcome labels became
  RTL_triggered.
- 13 families fly 50 s (corridor-drift-angles 70 s), so the outcome is shown.
- **Pins.**
  - Removed: dyn-nfz-spawn-ahead (4), three-simultaneous-violations,
    near-edge-at-12mps (2).
  - Added: rail-mission-stub-vla, corridor-swap-shifted.
  - standoff-wedge now pins its goal only.
- `docs/data/scenario_sweep.json` republished.
- LOITER flown as ArduCopter's BRAKE is kept as the best-practice mapping.
  Literal LOITER descends with no RC throttle.

#### Fixed
- wind-beyond-authority, lane 15 at 12 m/s: a missed fail-safe. G8 handed the
  hold back while it was still being blown through nfz-square. It now goes
  G1 G3 X7 G6 and lands off the zone.
- near-edge-at-12mps (2 cells): false fail-safes. A steady 0 m repair never
  "cleared" Loiter, and the 30 s timeout sent the aircraft home.
- 27 library episodes ended in Loiter before the timeout and were scored as
  correct non-triggers. They now fly to resolution.

#### Found
- **Fail-safe correctness**, before (7bfd6dc) -> after; nulls never / always;
  Wilson 95 % lower bound:

  | profile | before | after | nulls (after) | Wilson (before -> after) |
  |---|---|---|---|---|
  | library | 145/171 = 0.848 | **175/179 = 0.978** | 0.659 / 0.341 | 0.787 -> 0.944 |
  | smoke | 57/61 = 0.934 | **62/63 = 0.984** | 0.635 / 0.365 | 0.843 -> 0.915 |
  | nightly | 547/717 = 0.763 | **721/741 = 0.973** | 0.623 / 0.377 | 0.730 -> 0.959 |

- **E5 sensitivity.** With the E5 labels read as "not expected": library
  0.760, smoke 0.762, nightly 0.702. E5 carries most of the change; it is the
  grant's own condition (p1, p4) and is kept.
- **Below 0.99 on every profile, and not shown at 95 % anywhere.** Remaining:
  - rail-mission-stub-vla and standoff-approach-angles 7 m pedestrian: theta
    escalates the first deep forecast, the hold keeps the request deep, and
    the timeout sends the aircraft home, where projection alone would have
    resolved it. The Gazebo + ArduCopter SITL flight of the same mission
    (70ab5e8) flew those edges.
  - gps-dropout lane 23.6 (missed): no fix-quality input.
  - two nightly noise seeds that G4 counts.
- **Safety.**
  - P0 escapes: 0.
  - Polygon entries: unchanged.
  - P0 ticks at the true state rose: library +323, smoke +260, nightly +969.
    Every tick is in an episode the rubric expects a fail-safe for: the
    expected RTL / LAND flown now that runs are long enough, plus 106
    below-floor LAND ticks per wind-12 lane-15 episode.
  - Autopilot breaches: library 7 -> 10, nightly 21 -> 30. All are in
    expected episodes, and the old code at the new lengths flies the same.
- **Time to safe** (library): 3.135 s over 37 recoveries -> 2.997 s over 39.
  Max 6.7 s. Not optimised; the longest are physics-limited escapes.
- **PathRepair on by default (decision 4): measured, kept off.** 150/171 =
  0.877 and 26 more failing episodes. Its magnitude is the replaced
  forecast's depth, so head-on detours escalate.
- **Theta depth, measured, not decided.** Depth at the first entry plus one
  period, and depth weighted by time-to-entry, both remove the two false
  triggers and the Gazebo escalation (0 over theta on the 8 Oct replay), at
  a cost:
  - entry: 3 Shield-on polygon entries instead of 1, and one breach;
  - weighted: one breach (dyn-nfz-rotate-spin 9 deg/s) and +18 P0 ticks;
  - against the published labels: 163/179 and 171/179.

#### Limits
- No flight has used the 2 Nov Loiter law or the rubric. The Gazebo re-fly
  will show the same X1 G3 G5 edges, a false trigger by the rubric.
- `for_decision_period` is not wired into a VLA rail.
- RTLWatch does not judge keep-in corridors (corridor-swap-shifted).
- No scenario has a planner that thrashes its own request.

### 2026-11-02 — the October deck's presenter cheat-sheet, committed

#### Added
- `docs/CHEATSHEET-Final-Oct2026.md`: the presenter notes for
  `docs/VLA-Guardrail-Final-Oct2026.pptx`, written with the deck on 27 October
  and left out of the commits since. It holds the 30-second version, the
  honesty lines said before anyone asks (no VLA flew the recorded
  demonstrations; no safety number is a contract KPI yet, because no run is in
  the hil topology), the numbers to know, the slide timing and the questions to
  expect. Its numbers are the records as of 27 October; the later AIOT-VLA
  follow work and the vla_fly subject fix are not in it.

#### Fixed
- **The cheat-sheet called all 41 shielded flights "dev"** (ARCH-13, the
  independent review of 29 October). Only the 3 MAVROS 2 flights ran the
  grant's dev topology; the 34 Project AirSim and 4 pymavlink SITL flights
  ran our own functional rails, which `guardrail/manifest.py` labels
  `projectairsim-single-host` and `ardupilot-sitl-pymavlink`, none of the
  grant's three. The spoken lines now say "not KPI-grade" and
  "functional-rail evidence", and the topology slide's line gives the 3 / 38
  split.

#### Changed
- **The researcher's answers of 2 November** are recorded in
  `docs/FINAL-REPORT-outline.md` (sections 7-9 and Appendix A):
  - the WP2 and WP4 correction notes were sent to ITRI, so WP2-12 and WP4-21
    are DONE (40 DONE, 81 PARTIAL, 13 DRIFT, 2 MISSING);
  - the report states a one-person team (ARCH-24; it stays DRIFT, because
    the grant names five people);
  - the work stops at the hil topology, because the drone cannot carry an
    AGX Orin or run a large VLA, so `flight` (ARCH-14) still needs the PI's
    written waiver;
  - theta is forecast depth in metres, the shipped default; the rubric, the
    IR bump, the two-hash contract, the multi-scale bundle and the DRIFT
    items follow the grant with best practice;
  - the signer and the public-repository question are still open, and
    nothing is pushed meanwhile.

### 2026-11-02 (evening) — the Gazebo Harmonic rail flies end-to-end, with its GeoFence on; the CI gates run per build on the lab machine; a video for every camera-less run

#### Added
- **`PHYSICS=gazebo` for the ROS 2 rail** (`sitl/run_ros2_demo.sh`,
  `sitl/start_sitl.sh` `SITL_MODEL` / `SITL_EXTRA_DEFAULTS`): a headless
  gz sim server and SITL on its JSON backend, so the Gazebo rail flies the
  same MAVROS 2 rail as the built-in SITL (router, fence upload, bag). Every
  ROS 2 run now writes `physics.json`.
- **`tools/ci_local.py`: the CI pipeline, run per build on the lab machine**
  (WP4-06, WP4-14, WP3-19). GitHub runs `.github/workflows/tests.yml` only
  on a push, and nothing has been pushed since 14 Oct (1b95383), so no build
  since ran the smoke profile. This runs the workflow's three gates on this
  machine: every file of its standalone loop, read from the workflow in the
  loop's order (never a copy), then `tools/check_claims.py`, then
  `experiments/sweep_scenarios.py --profile smoke`. Logs and smoke bundles go
  to `ci-local/<revision>/` (gitignored); the build's summary goes to
  `docs/data/ci_local/<revision>.json` (committed), with `-dirty` in the
  name when the tree was not clean. A lock file refuses a second runner
  while the first is alive, after a stopped runner kept writing into the
  next run's results on 2 Nov. `tests/test_ci_local.py` (10 tests) runs in
  the loop.
- **`tools/render_track_video.py`: a video for every camera-less SITL run.**
  The pymavlink, ROS 2 and Gazebo rails leave numbers and a trajectory.png,
  no video. This replays `flight_log.jsonl` tick by tick: the policy's zones
  (by generation) and altitude band, the mission target, the track, the
  pilot's command against what the Shield emitted, the FSM state and the edge
  that changed it, the forecast violations and the repairs. Every frame says
  it is a replay of the flight log, not a camera recording. Recording the
  Gazebo GUI was tried and does not work here: WSLg windows capture black
  through gdigrab, and the default GUI has no video-recorder service.
  `tests/test_render_track_video.py` (5 tests) runs in the CI loop.

#### Fixed
- **The Gazebo Harmonic rail ran with the ArduPilot GeoFence off**
  (`sitl/run_gazebo_demo.sh`, WP3-10). It loaded only the stock copter and
  gazebo-iris defaults, so its first end-to-end run read `FENCE_ENABLE 0`:
  the last-line backstop behind the Shield was missing on this rail. It now
  loads `sitl/fence/guardrail_fence.parm` as `sitl/start_sitl.sh` does
  (`GUARDRAIL_FENCE=0` for the stock autopilot) and wipes the EEPROM at each
  start (`SITL_WIPE=0` keeps it), so a fence polygon a previous flight left
  is not armed from boot.
- **A clean tree recorded as `-unknown` from WSL** (`guardrail/manifest.py`
  `code_revision`, WP4-12). The first `git status` from WSL on the OneDrive
  tree refreshes the whole index over `/mnt/d`, and on the first Gazebo run
  it outlasted the 20 s timeout; the next one took 2.4 s. The status is now
  asked twice (20 s, then 90 s), and only two failures say `-unknown`.
  `tests/test_manifest.py` shows a first timeout followed by a clean read.
- **The WSL ArduPilot tree is at its pin again, and `--verify` can no longer
  pass a binary it did not build** (`sitl/setup_sitl.sh`, ARCH-31). The
  nested `pymavlink` had sat at `ec06837a` since the master clone (found
  2026-10-06), so every WSL SITL run recorded `ardupilot_pin_ok` false. A
  re-run of the setup reset it to `8ba67079`, the commit Copter-4.5.7
  records, and its rebuild failed: the venv had no `dronecan`, which
  ArduPilot's own prerequisites install. The setup still exited and
  `--verify` then printed PIN OK for the OLD binary, whose banner is the
  same 8 hex digits. Two fixes: `dronecan==1.0.27` joins the pinned venv
  packages, and a successful build writes
  `build/sitl/guardrail_build_stamp.txt` (HEAD and every submodule
  commit), which `--verify` now requires to equal the checkout, so a binary
  built by hand or before a submodule moved fails ("no build stamp", "build
  stamp differs"; two tests in `tests/test_env_pins.py`, 41/41). The rebuild
  then passed, and measured what 2026-10-06 could not: waf regenerated the
  MAVLink headers with the pinned generator, found them unchanged and
  relinked nothing, so the binary on disk (2026-09-18) is the pinned build.
  `setup_sitl.sh --verify`: PIN OK. Runs flown before this fix keep their
  `ardupilot_pin_ok` false.

#### Flown (Gazebo Harmonic, WSL Ubuntu 24.04; not KPI-grade)
Gazebo Sim 8.14 (headless server) with ArduCopter 4.5.7 SITL on the JSON
backend and the ardupilot_gazebo plugin, flown by `sitl/run_sitl_demo.py` over
pymavlink at e74eae3 (clean tree, Python 3.12.3), policy `fase3-sim-demo`
from its signed bundle, command "fly to the northeast pad at 6 m/s", the
GeoFence backstop on (`FENCE_ENABLE 1`, `FENCE_ALT_MAX 30`):

| run (`demo/out/`) | Shield | P0 escape rate | inside the NFZ | target | FSM |
|---|---|---|---|---|---|
| `gazebo_shield_on_20261102` | on | 0.0 | 0.0 s | not reached | X1, G3, G5 (Loiter timeout), RTL, home |
| `gazebo_shield_off_20261102` | off | 0.584 | 3.2 s | reached | - |
| `gazebo_shield_on_dynamic_20261102` | on, NFZ hot-applied | 0.0 | 0.0 s | not reached | X1, G3, G5, RTL, home |

Over MAVROS 2, the grant's `dev` topology (`PHYSICS=gazebo bash
sitl/run_ros2_demo.sh on|off - --tag ...`, at 0207af0, clean tree): SITL
-> mavlink-router -> MAVROS 2, the adapter uploading the zone as a fence
polygon (4 items sent and read back), the bag recorded, `physics.json`
naming Gazebo Sim 8.11.0 and the JSON model:

| run (`demo/out/`) | Shield | P0 escape rate | inside the NFZ | target | what stopped it |
|---|---|---|---|---|---|
| `gazebo_ros2_shield_on_20261102` | on | 0.0 | 0.0 s | not reached | the FSM: X1, G3, G5, RTL |
| `gazebo_ros2_shield_off_20261102` | off | 1.0 | 6.6 s | not reached | the GeoFence: "Fence Breached" 3.6 s into the mission, RTL |

With the Shield off, the autopilot's own fence caught the aircraft, as the
grant's backstop should; it still spent 6.6 s inside the zone, because a
fence acts on the breach, not on the forecast. The node's start-up line
"no upload reported by the adapter" is true when printed: the autopilot
accepts the polygon only once its EKF has a position, 15 s after the
publish here; the record (`fence_upload`, `report.md`) has the upload.

Under SROS 2 (X-15), the built-in SITL physics, at 02dca27:
`ros2_sros2_shield_on_20261102` (`SROS2=1 bash sitl/run_ros2_demo.sh on`),
every ROS 2 process in its own enclave under `ROS_SECURITY_STRATEGY=Enforce`,
the keystore made by `tools/sros2_setup.sh` from a lab CA generated in WSL
(`gen-pki --out ~/harness_pki`); the run's `hil_evidence.ros2_security`
records the enclave and an identity and permissions CA fingerprint equal to
that CA's. `tools/sros2_check.sh` passed 6/6 first. P0 escape rate 0.0,
0.0 s inside the zone; the target was not reached, as above.

Simulator stressors (X-14), pymavlink rail, built-in physics, at 7c5547c,
set by `--sim-param` at mission start and read back: wind 8 m/s from 225
deg (toward the zone), turbulence 2 - Shield on P0 0.0, 0.0 s in the zone
(`sitl_wind8_shield_on_20261102`), Shield off P0 0.6, 3.6 s in the zone
(`sitl_wind8_shield_off_20261102`); GPS disabled - Shield on P0 0.0
(`sitl_gpsloss_shield_on_20261102`). With the Shield on the aircraft held
near home in every run (the first-tick escalation above), so these runs show
the stressor plumbing works and the Shield-off control fails; they do not
yet test a Shield flying a mission through wind. Re-fly after the WP3-15 fix.

Each folder holds its flight-log replay video (`<tag>_track.mp4`). Earlier
the same day, before the fence fix, `gazebo_shield_on` flew with the
GeoFence off (and recorded its clean tree as `-unknown`), and three test
flights (`_20261102b` to `_20261102e`) were spent on recording the GUI; their
Shield results match the table.

#### Found
- **With the Shield on, the Gazebo mission never leaves home.** The target
  (30, 30) sits behind `nfz-square` (7..23 m), head-on from home. On the
  first tick the pilot's 6 m/s command forecasts 8.13 m into the zone; the
  GeofenceSlide repair is over theta (2.0 m), so the FSM goes Normal ->
  Brake (X1) -> Loiter (G3) on ticks 1-2, holds for the 30 s Loiter timeout
  (G5) and sends the aircraft home, where it already is. Safe, and the
  false trigger the fail-safe correctness KPI counts (WP3-15; the
  headless library's X1>G3>G5 group). For comparison only: on 2026-10-07
  the same rail, with the zone hot-applied at 8 s and theta on the velocity
  proxy, reached the target in 25.6 s (`sitlfix_on_dyn`).

#### Limits
- The pymavlink runs upload no fence polygons, so only the altitude fence
  backs them up; the MAVROS 2 runs upload the zones. Topologies
  `ardupilot-sitl-pymavlink` and `dev`: none of it is KPI-grade (the grant
  takes KPIs from `hil`).
- The mid-term gate it belonged to was 2026-07-20 (ARCH-16 is DRIFT: late).
- SROS 2: the enclaves still carry sros2's default permissions (every topic
  of the domain); per-topic permissions are not written. The lab CA this
  keystore came from was generated in WSL today; the gRPC harness on Windows
  holds its own, so "one root CA" holds only where the harness reads
  `~/harness_pki`.
- Every WSL SITL run today recorded `ardupilot_pin_ok` false: the nested
  `pymavlink` submodule is at `ec06837a`, not the commit Copter-4.5.7
  records, as found on 2026-10-06 (`docs/DESIGN-python-versions.md`). The
  binary is 4.5.7 (`2a3dc4b7`) built against headers from that newer
  generator.

#### Changed
- `docs/FINAL-REPORT-outline.md`: ARCH-16 (the Gazebo functional rail) goes
  from MISSING to DRIFT (it exists, 105 days after its gate); ARCH-17 and
  WP3-01 cite it. 40 DONE, 81 PARTIAL, 14 DRIFT, 1 MISSING.

### 2026-11-01 (late) — the VLA follow's subject stand-off now runs: vla_fly gives the Shield the perception's subject every tick, with its velocity, and says INERT when it has none; every earlier VLA flight was flown with the stand-off inert

#### Fixed
- **`demo/vla_fly.py` never called `Shield.set_subject`**, so the follow
  policy's `standoff-pedestrian` (10 m, P0) and `standoff-any` (5 m, P0)
  were inert in every VLA follow flown (AerialVLA, DroneCATS, AIOT-VLA
  v7-v10b; PC and Jetson; the evening entry below, "Found"). Every mission
  tick now sets the Shield's subject before the Shield judges the command
  (`shield_subject`): the follow's perception, never the level car's truth -
  the last held box's map point (`follow_vlm.Grounder`'s identity features,
  from its own capture's depth and pose), carried from its capture to now
  at the velocity measured from the held boxes over 2 s
  (`measured_velocity`, `follow_vlm.measured_speed` as a vector, capped at
  the class's top speed), with the class the object words select
  (`follow_vlm.subject_width`: "a red car" a car, "a person" a pedestrian)
  and that velocity (`set_subject(velocity=...)`). None while the subject is
  not committed (acquiring, lapsed, reset) and once it is unseen for more
  than `LAPSE_S` (3 s) - checked by the feed itself as well, because the
  follow's lapse runs only when a detector frame arrives and the camera
  stream stalled for 7.5-10.9 s in 4 of 39 flights. Cleared when the
  mission ends. Each row carries `shield_subject` and `shield_subject_v`.
- **Inert, said out loud.** `subject_feed` decides at the mission start
  whether there is a perception subject to feed: none with `--gate-truth`
  (the client's box is the level car's TRUE position, so a stand-off from it
  would be an oracle; AIOT-VLA itself steers on its own OWL-ViT and ignores
  the hint, `vla_server_foreign.AIOTVLAFamily`), none with the identity off
  (no box is placed on the map) or on a mission with no subject. Then, and
  whenever binding rules never had a subject (the feed on, the subject never
  acquired), the console says `[standoff] *** standoff-any INERT: <why>` at
  the start and in the report; `metrics.json` `subject_standoff` records the
  rules, the ones that bind this class, the source, the ticks with a subject
  (and with its velocity), `inert` and `inert_why`, the stand-off violation
  ticks and repairs. The HUD's stand-off row reads INERT instead of "no
  target", and its OK banner "rules satisfied; STAND-OFF INERT (no subject)"
  instead of "all rules satisfied".
- `tests/test_vla_fly_fakesim.py` (lab-only; SKIPs in CI) had stale fakes:
  its `ProjectAirSimClient` took no `address` since fe171ab (`--sim-address`)
  and its `_LevelCar` no `kind` since c31f1f7, so its flights failed in the
  lab. 14/14 again in vla-real; its follow flies `follow_car_citylife_nfz.yaml`
  now and checks the INERT record and console lines end to end.

#### Found (what the flights loaded, and what said so)
- 113 of the 116 VLA flights in `demo/out` were follows on the signed
  `follow-car-citylife-nfz` bundle with both stand-offs; 2 follows and 1
  goto flew `vla-goto-citylife`, which has none.
- Nothing marked the rules as not run. The HUD row said "idle, no target"
  under a banner reading "GUARDRAIL ACTIVE - all rules satisfied" (4,015 of
  aiot_v10br_01's 4,290 HUD ticks); `audit.jsonl` logs violating ticks only
  (aiot_v10br_01: 23, nfz-frontage 15 and bld-clearance 8); `kpi.json`'s
  `violations_by_type` had no stand-off key and its P0 escape rate read 0.0.
- Against the level car's truth, the 110 follows with truth rows were within 5 m of
  their subject on 11,049 of 342,042 ticks after the first VLA decision
  (3.2 %), within 10 m on 33,452 (9.8 %), and as close as 0.02 m
  (aiot_v10bw_02).
- Earlier VLA results are unchanged as follow scores. They were flown with
  the subject stand-off inert, and their "P0 escapes 0.0" covers the zone,
  clearance, altitude and speed rules only.

#### Decided: the grant's 5 s / 0.1 s forecast stays, with the subject's velocity
`tools/replay_standoff_feed.py` replays the logged commands of every
identity-on VLA follow through the Shield with the subject this feed gives
(rebuilt from `detections.jsonl` and `metrics.follow` and handed to
`vla_fly.shield_subject` itself), open loop. 65 flights with a perception
subject, 179,155 ticks from the first acquisition, a subject fed on 53,866
(0.301):

| forecast | stand-off repair ticks | flights over 5 % | worst flight | median speed cut | at truth < 10 m | other rules' repair ticks |
|---|---|---|---|---|---|---|
| 5 s / 0.1 s, subject held still | 16,317 (0.0911) | 36 | 0.455 | 3.98 m/s | 578 | 6,181 |
| 5 s / 0.1 s, velocity given (flown) | 2,057 (0.0115) | 4 | 0.068 | 2.25 m/s | 516 | 6,346 |
| 3 s / 0.5 s, held still | 2,359 (0.0132) | 6 | 0.188 | 3.22 m/s | 552 | 9,505 |
| 3 s / 0.5 s, velocity given | 1,023 (0.0057) | 0 | 0.044 | 1.95 m/s | 423 | 9,499 |

Held still, the grant's forecast repairs the command on 9.1 % of the ticks
and would end the follow (follow_vlm's own replay of 19 Oct: 44 %); given
the subject's velocity, on 1.1 %. The 3 s / 0.5 s forecast is not needed once the
velocity is given: it halves the stand-off repairs again but changes every
rule's forecast, and on the same ticks the other rules' repairs rose by
half. The 39 `--gate-truth` flights, replayed as a what-if with their truth
box (vla_fly feeds none): 5.2 % of 140,418 ticks with the velocity, 19.6 %
held still.

#### Flown (AIOT-VLA v10b on the red car, real perception, Car_10; restored level)
| flight | within 30 m from the first acquisition | ticks with a subject, from the acquisition | subject against the truth, median | stand-off violation ticks | StandoffHold repairs | closest to the car (truth) | P0 escapes |
|---|---|---|---|---|---|---|---|
| aiot_v10bs_01 | 0.969 | 2,090 of 2,090 | 1.74 m | 18 | 18 | 11.19 m | 0.0 |
| aiot_v10bs_02 | 0.960 | 1,944 of 1,944 | 1.81 m | 56 | 54 | 10.60 m | 0.0 |
| aiot_v10bs_03 | 0.955 | 1,974 of 1,974 | 1.80 m | 29 | 32 | 11.86 m | 0.0 |

The same model and scenario before this change (aiot_v10br_01-03): 0.944,
0.985 and 0.202, the last lost to an 8.7 s camera stall (none in these
three). Flown from 55faad0 (each manifest's `code_revision`, clean): the
worktree's `vla_fly` with the main checkout's simulator, obstacle map and
signed bundle, Unreal's editor closed, a PIP video each
(`docs/video/aiot_v10bs_0N_pip_small.mp4`, local). The stand-off acted where
the aircraft closed on a car stopped ahead: in aiot_v10bs_02 at 230.5 s,
the subject 18 m ahead and the command 4.9 m/s, StandoffHold cut it to 1.5
and then 0.1-0.5 m/s while the car stood (truth 19.0 m, then 14.9 m at
232.0 s). None of the three came within 10 m of the car by the truth, so
the 5 m ring itself was never reached (aiot_v10br_02 came to 4.3 m with
no stand-off judged), and the car stayed outside the camera's 6.9 m blind
spot. Whether the hold is what spares these flights v10's stopped-car loss
(aiot_v10kw_02 closed to 7.4 m and lost the car) three flights do not show:
v10b's other two red flights came no closer than 10.0 m without it. The HUD's stand-off row showed the separation the
Shield judged ("28 m", OK) and the pedestrian row "n/a (car)". In
aiot_v10bs_02 a 0.1 s building-clearance violation at the aircraft's own
position (230.6 s: a street tree during the VLA's acceleration, before any
stand-off repair) put BREACH on the HUD for 15 ticks; nothing escaped.
Three flights: an indication, not a measure of the change's effect on the
score.

#### Limits
- The camera's blind spot (0.86 x altitude, 6.9 m at 8 m) is wider than the
  5 m ring. In the replay the truth was within 5 m on 834 ticks; a subject
  was fed on 177 of them and placed within 5 m on 129. The feed coasts a
  lost subject only until the follow's own lapse (3 s), so the 5 m rule acts
  mostly through the forecast while the car is still seen; a car already
  under the aircraft is mostly not judged.
- `--gate-truth` flights (the white car, the box truck, the black car, the
  person) still fly with the stand-off inert, now said so; a person has not
  been followed on perception, so the 10 m rule has not bound in flight.
- The replay is open loop: the logged commands at the logged poses, the
  aircraft's velocity from pose differences.

#### Tests
- New `tests/test_vla_standoff_feed.py` (11/11; on 1dd6c24 3/11: the three
  that pin the Shield's own behaviour pass, the 8 on vla_fly and the HUD
  fail), in the tests.yml loop. The whole loop, 133 files, exits 0 in the
  worktree under vla-real (44 skip the parts that need the lab's data).

### 2026-11-01 (evening) — AIOT-VLA v10 and v10b: the black car 0.940 / 0.261 / 0.855 within 30 m when it starts ahead and driving away (0.08-0.14 head-on); a person 0.25-0.37 on the one acquired; without the person data (v10b) the white car 0.926; VLA flights never ran the subject stand-off rules

All black-car and person runs below were flown while Car_07 was slowed to
320 cm/s (the 2026-11-01 temporary level change; each run's
`demo/out/<tag>/level_conditions.json` and the summary rows say "Car_07 slowed
(2026-11-01 temporary change)"). Loop B shares three junctions with loop A and
pedestrians wait for gaps in traffic, so these numbers are not to be compared
with flights on the restored level without saying so.

#### Added
- `--level-car Ped_04`: ONE figure the level walks as the subject
  (c31f1f7). `level_actors.kind_of_tags` tells a figure's tag from a car's
  and refuses a mix; `LevelCar` / `LevelCars` carry the kind. In
  `follow_vlm` a level figure is truth for the pedestrian class only, outranks
  a crowd truth, and every phrase must name a person
  (`level_car_refusal(kind="pedestrian")`); `vla_fly` builds the kind from
  its tags. `run_citylife_follow.ps1 -LevelPed Ped_17 [-TeacherTruth]` flies
  the person mission against that tag, without polling the other 39.

#### Recorded (teacher, `follow_vlm --teacher-truth`, 600 s; Car_07 slowed)
- The black car (Car_07 at a median 3.20 m/s, from 4.83): aiot_tch_k02 and
  k03 within 30 m on 1.000 of their ticks (median 16.8 and 16.6 m; the same
  teacher at 560 cm/s: 0.695), DART k04 0.959 and k05 0.983, DAgger windows
  flown by v9 k06 0.952 (1,785 of 4,722 captures) and k07 0.933.
- A person (one figure per run, chosen because it walks into the start
  camera's view within 30 m 34-145 s after the level starts): aiot_tch_p00
  (150 s smoke run, Ped_17: acquired after 15 s at 22.1 m, never inside
  10 m), p01 Ped_17 0.832, p02 Ped_21 0.845, p03 Ped_12 0.801, DART p04
  Ped_09 0.889, p05 Ped_16 0.993, p06 Ped_15 0.855. Over the first 420 s
  (a flight's length) p01-p03 are 1.000, 1.000 and 0.981: each lost its
  person between 488 and 507 s (p02 while the building fence held the
  aircraft back and the person walked on; p01 when the person left the
  frame 12 m away and the search never found it again).

#### Added (model)
- AIOT-VLA v10: v9's recipe on 38 runs (v9's 26 + the black car's six + the
  person's six). 131,082 training and 21,698 hold-out samples (aiot_rec_e02,
  aiot_tch_w02, aiot_tch_t05, aiot_tch_k03, aiot_tch_p02), 17,736 dropped as
  the memory off the expert's subject; best epoch 13, hold-out fwd_m MAE
  0.099 m, dpsi_rad 0.0139 rad, right_m 0.045 m (correlation 0.944). The
  hold-out now holds a black and a person run, so these are not v9's set.
  Weights D:/models/aiotvla/v10.

#### Flown (AIOT-VLA v10, vla_fly e2e, `--gate-truth`; Car_07 slowed)
| target | start heading | within 30 m of the instance acquired, per flight | mean |
|---|---|---|---|
| a black car (Car_07) | 90 (the follow's usual) | 0.140, 0.079, 0.144 | 0.121 |
| a black car (Car_07) | 270 | 0.940, 0.261, 0.855 | 0.685 |
| a person (24 figures, Ped_00-Ped_23) | 90 | 0.334, 0.252, 0.371 | 0.319 |

The person flights were within 30 m of SOME person on 1.000, 0.950 and
0.997 of their ticks; with 24 figures on the same pavements that column says
little, so the table scores the figure acquired.

#### Added (control)
- AIOT-VLA v10b: v10 without the six person runs (32 runs, 111,722 training
  and 17,740 hold-out samples; v9's hold-out plus aiot_tch_k03), to see
  whether the person data is what cost v10 the vehicles. Best epoch 15,
  hold-out fwd_m MAE 0.054 m, dpsi_rad 0.0042 rad, right_m 0.023 m
  (correlation 0.988; v9 0.064, 0.0043, 0.026). Weights D:/models/aiotvla/v10b.

#### Flown (the restored level, Car_07 back at 560 cm/s; as v9's flights)
| target | v10, per flight | mean | v10b, per flight | mean | v9 |
|---|---|---|---|---|---|
| a box truck (Car_23 each time) | 0.509, 0.751, 0.940 | 0.733 | 0.738, 0.824, 0.968 | 0.843 | 0.901 |
| a white car (Car_22 each time) | 0.888, 0.900, 0.258 | 0.682 | 0.912, 0.908, 0.959 | 0.926 | 0.811 (6 flights) |
| a red car (Car_10) | 0.861, 0.421*, 0.955 | 0.746 | 0.944, 0.985, 0.202* | 0.710 | 0.956 |

\* the simulator's camera stream stopped for 10.9 s (aiot_v10r_02) and 8.7 s
(aiot_v10br_03) while the car was 20-25 m ahead: no frame reached the
detector or the VLA, the step tracker held, and the car drove out of reach.
Without those two flights the red car is 0.861 and 0.955 for v10, 0.944 and
0.985 for v10b.

With the person data out, the white car rises to 0.926 (all three above 0.9)
and the trucks to 0.843; on the red car v10b's two unstalled flights are
v9's level. The person data, about a sixth of v10's samples at a 12 m
stand-off and a walking pace, costs the vehicles (three flights a target:
an indication, not a measurement of the size). Against the truth, v10's red flights came
within 0.3, 9.5 and 10.4 m of the car (17 %, 4 % and 0 % of the ticks within
10 m; v9's: never within 10 m); v10b's within 10.4, 4.3 and 10.0 m. P0
escapes 0.0 in every flight, with the stand-off rules not running (below).

#### Found
- **VLA flights never ran the subject stand-off rules.** `guardrail/shield.py`
  `_check_standoff` is inert without a subject (`set_subject`), and
  `demo/vla_fly.py` never sets one, so `standoff-pedestrian` (10 m, P0) and
  `standoff-any` (5 m, P0) did not run in any VLA flight; their "P0 escapes
  0.0" says nothing about stand-off (fixed in the "late" entry above).
  Against the truth, v9's white-car and
  truck flights came within 0.1-3.9 m of their car and spent 10-29 % of their
  ticks within 10 m; v9's red-car flights never closer than 10.4 m; v10's
  person flights within 1.0-5.4 m of the person acquired.
- **The black car comes head-on.** Loop B runs the opposite way to the
  follow's start heading: at 90 the black car is acquired 17-20 m ahead
  driving TOWARD the aircraft (about -3.5 m/s along the heading) and passes under
  it, while every red, white and truck flight acquires its car ahead and
  driving away. In aiot_v10k_01 the memory coasted the car behind-right for
  3 s, then reseeded on something 20 m ahead moving east at about 3.7 m/s
  (most likely another car), and v10 chased it east at 5 m/s; the gate then
  lapsed and vla_fly never asked again.
  Facing 270 the black car starts as the others do.
- **Where the car stops, the gate lapses.** In aiot_v10kw_02 the black car
  stopped and v10 closed to 7.4 m, inside the camera's 6.9 m blind spot at 8 m;
  the truth gate lapsed, the aircraft hovered 30 s, and when the car drove on
  it was re-acquired 27 m behind and lost at the next corner. The step has no
  reverse.
- **A person is not one person.** v10 stays among people but changes the
  nearest figure 37-54 times per flight; the target memory is position-only,
  and figures pass within a few metres of each other on the pavements.
- **The simulator's camera stream stalls.** In 4 of the 39 v9, v10 and v10b
  flights the detector produced nothing for 7.5-10.9 s (aiot_v9w_04,
  aiot_v10kw_01, aiot_v10r_02, aiot_v10br_03; in aiot_v10br_03 the VLA's
  frames stopped for the same 9.5 s), while the pose and the 10 Hz loop ran
  on; two of them lost the car to it. The cause is not found.

#### Limits
- Three flights per target and start, one level, the new targets on a truth
  gate; not KPI-grade.

### 2026-11-01 (afternoon) — the temporary level change ends: Car_07, the black car, back at 560 cm/s after the AIOT-VLA black-car and person work

#### Changed
- `CityLife_Day` (the gitignored level): Car_07's `SpeedCmS` 320 → 560 cm/s,
  its value before the morning's change. All 24 cars read back equal to the
  list in the 2026-11-01 entry below (Car_07 560, Car_10 320). The editor log
  saved one package, `/Game/CityLife/Maps/CityLife_Day`, and the `.umap`'s
  file date moved. The slowed level is kept at
  `PASBlocks/_backup/2026-11-01_pre_car07_restore/`. The re-saved file is not
  byte-identical to the level before the morning's change (a re-save never
  is); the speeds are what match.
- Flown while Car_07 was slowed, and tagged "Car_07 slowed (2026-11-01
  temporary change)" in `summary.jsonl` and in each run's
  `demo/out/<tag>/level_conditions.json`: six black-car recordings (k02-k07),
  one person smoke run (p00), six person recordings (p01-p06) and nine AIOT-VLA
  v10 evaluation flights (v10k_01-03, v10p_01-03, v10kw_01-03). Their scores
  are not comparable with flights on the level as it is now.
- Flights from here on, including the v10 regression on the red car, the white
  car and the box truck, fly on the level as it was before 2026-11-01.

### 2026-11-01 — a temporary level change: Car_07, the only black car, slowed from 560 to 320 cm/s so the AIOT-VLA black-car recordings have a car the aircraft can keep up with; to be put back afterwards

#### Changed
- `CityLife_Day` (the gitignored level): Car_07's `SpeedCmS` 560 → 320 cm/s,
  the same as Car_10. At 560 cm/s Car_07 (loop B) drove at a median 4.83 m/s,
  p90 5.61 m/s (aiot_tch_k01), above the aircraft's 5 m/s policy cap, and even
  the privileged teacher fed the car's true position stayed within 30 m on only
  0.695 of its ticks. The other 23 cars read back unchanged. Only
  `CityLife_Day.umap` was rewritten (the editor log saved one package). The
  level as it was: `PASBlocks/_backup/2026-11-01_pre_car07_speed/`.
- **Temporary.** Car_07 goes back to 560 cm/s once the AIOT-VLA black-car
  recordings and evaluation flights are done
  (`python -m tools.citylife_mcp.car_speed Car_07 560`), with its own entry.
  Black-car numbers from before 2026-11-01 (v8 0.104, the teacher's 0.695) were
  flown at 560 cm/s and are not comparable with later ones.
- Loop B shares three junctions with loop A ((4100, -4100), (4100, 4100),
  (12300, -4100)), and loop B's cars now queue behind Car_07. Flights on Car_10
  while this change stands are therefore not flown in exactly the traffic of the
  0.963 / 0.972 baselines. The size of that effect is not measured.

#### Added
- `tools/citylife_mcp/car_speed.py`: prints every CityLife car's `SpeedCmS`, or
  sets one car's and saves the level. It fails unless the read-back matches, no
  other car changed and the `.umap`'s file date moved, because `save_assets` can
  return True while the file stays locked. Top speeds read on 2026-11-01, before
  the change (cm/s): Car_00 420, Car_01 480, Car_02 390, Car_03 530, Car_04 450,
  Car_05 505, Car_06 410, Car_07 560, Car_08 460, Car_09 540, Car_10 320,
  Car_11 575, Car_12 430, Car_13 495, Car_14 385, Car_15 520, Car_16 415,
  Car_17 530, Car_18 470, Car_19 395, Car_20 555, Car_21 440, Car_22 505,
  Car_23 380.

#### Tests
- `tests/test_car_speed.py` (5 checks, added to tests.yml): a fake editor stands
  in for the endpoint. A save that never reaches the disk fails, a set the editor
  ignores fails, an unknown car is refused before anything is written, and the
  substituted editor script parses. With the two checks removed, 2 of the 5 fail.

### 2026-10-31 (morning) — AIOT-VLA v9: a box truck 0.901 within 30 m after a tighter teacher and DAgger windows; the red car 0.956; a white car 0.811 over six flights, one of them lost where the car stopped

#### Recorded (teacher, `follow_vlm --teacher-truth`, 600 s)
- Box trucks with a tighter stand-off (`-ObjectWidthM 3`: 11.9 m): aiot_tch_t05
  (Car_00) 0.846 within 30 m, t06 (Car_23) 0.971; with DART t07 0.710, t08
  0.827; with v8 flying DAgger windows t09 (Car_23, 1,774 of 4,749 captures)
  0.962, t10 (Car_00) 0.745. The teacher's median distance is now 19.6-22.5 m
  (23-27 m before).
- The white car with v8 flying DAgger windows: aiot_tch_w05 0.895 (2,262 of
  4,873 captures), w06 0.960.

#### Added
- AIOT-VLA v9: v8's recipe on 26 runs (the red car's 12, the white car's 7, the
  black car's 1, the tighter trucks' 6; the trucks at 15.8 m, t01-t04, left
  out). 92,153 training and 13,770 hold-out samples (aiot_rec_e02,
  aiot_tch_w02, aiot_tch_t05), 7,926 dropped as the memory off the expert's
  subject; hold-out fwd_m MAE 0.064 m, dpsi_rad 0.0043 rad, right_m 0.026 m
  (correlation 0.987). Weights D:/models/aiotvla/v9.

#### Flown (AIOT-VLA v9; as v8's flights)
| target | within 30 m of the instance acquired, per flight | mean | v8 |
|---|---|---|---|
| a box truck (Car_23 each time) | 0.913, 0.837, 0.952 | 0.901 | 0.615 |
| a white car (Car_22 each time) | 0.282, 0.937, 0.884, 0.896, 0.938, 0.927 | 0.811 | 0.907 (3 flights) |
| a red car (Car_10) | 0.938, 0.967, 0.963 | 0.956 | 0.978 |

P0 escapes 0.0 in all twelve. Not flown: the black car (no new recording).

#### Found
- **Where the car stops, the aircraft can arrive on top of it.** In
  aiot_v9w_01 the white car stopped and v9 flew to within 0.5 m of it, below
  the camera's view; it drifted toward the other white car (Car_15) and lost
  both, and when the gate's subject lapsed vla_fly stopped asking the model
  (hold, then hover) for the rest of the mission. Every white-car flight of
  v8 and of v9 came within 0-1.4 m of the car at least once; v9 came within
  8 m of the trucks on 21-25 % of the ticks. The step has no reverse (fwd_m
  >= 0), while the expert backs off on 12 % of its ticks.
- The tighter teacher and the DAgger windows fixed the box truck (0.615 to
  0.901); on the red car v9 is below v8 (0.956 against 0.978), still above
  0.9 in every flight.

#### Limits
- Three to six flights per target, one level; the new targets start on a truth
  gate; not KPI-grade.

### 2026-10-31 — AIOT-VLA v8 follows other targets by their words: a white car 0.907 within 30 m, the red car 0.978; a box truck 0.615 and a black car 0.104 do not yet

#### Added
- **AIOT-VLA v8**: one model for several targets. v7's recipe on 22 runs (v7's
  12 red-car recordings and 10 teacher recordings of a white car, a black car
  and a box truck), with the object words' OWL-ViT text embedding as one more
  token (`train --text`, `<cache>/text.npz`), the cache re-built with six
  phrases (`D:/vla_drone_offload/aiotvla/cache_v8`, v7's PCA), and samples whose
  target memory is more than 15 m from the expert's own estimate left out
  (`--memory-agree-m 15`: 6,754). 74,688 training and 13,563 hold-out samples
  (aiot_rec_e02, aiot_tch_w02, aiot_tch_t02); 1,827,730 parameters; hold-out
  fwd_m MAE 0.062 m, dpsi_rad 0.0049 rad, right_m 0.033 m.
- **Contrast phrases for the new targets** (`training/aiotvla/core.py`
  CONTRAST), chosen on the teacher recordings against each target's truth:
  "a white car" against a blue car, a black car and a box truck (0.629 of the
  hits pass, 0.155 of the frames without it; 0.957 and 0.752 on the logit
  alone); "a box truck" against a red, a blue and a black car (0.743, 0.161;
  not against "a white car": the trucks are white); "a black car" against a
  red and a blue car (0.653, 0.059). "a red car" keeps v7's two.
- **`vla_fly --gate-truth`**: the start gate and re-acquisition judge the
  teacher's box of the level car, for targets the hand-written detector cannot
  find; the VLA flies on its own perception (`params.follow.gate_truth`). The
  new targets' flights use it; the red car keeps the detector gate.
- Teacher recordings (`follow_vlm --teacher-truth`, 600 s): a white car
  (aiot_tch_w01-w04: 0.865, 0.878, 0.901, 0.905 within 30 m), a black car
  (aiot_tch_k01: 0.695), a box truck (aiot_tch_t01-t04: 0.797, 0.808, 0.630,
  0.677). w03, w04, t03, t04 with DART.

#### Flown (AIOT-VLA v8, `vla_fly` e2e, 420 s, start gate 30 m, Shield on; three flights each)
Headline: within 30 m of the instance it acquired (the candidate nearest at
acquisition). Also: within 30 m of any car the phrase names, and of the run's
intended tag.

| target (`--level-car`) | acquired | mean | any | intended |
|---|---|---|---|---|
| a red car (Car_10; detector gate) | 0.986, 0.972, 0.977 | 0.978 | 0.978 | 0.978 |
| a white car (Car_22, Car_15) | 0.958, 0.874, 0.889 | 0.907 | 0.913 | 0.907 |
| a box truck (Car_00, Car_23, Car_14) | 0.634, 0.560, 0.651 | 0.615 | 0.649 | 0.000 |
| a black car (Car_07) | 0.042, 0.133, 0.137 | 0.104 | 0.104 | 0.104 |

P0 escapes 0.0 in all 12. The white car was acquired as Car_22 in all three
flights; the box truck as Car_23 (the intended Car_00 never came within 30 m
first). On the red car v8 is v7's equal or better (v7: 0.968).

#### Found
- **The box truck follows its teacher's slack.** The teacher held the truck
  23-27 m behind at the median against a 15.8 m stand-off, and v8 holds it
  26.9-28.1 m behind, so 35-44 % of its ticks are over 30 m, with 72-200
  Shield repair ticks a flight. Car_23 drove at 3.79 m/s (p50) in those flights, the
  teacher's Car_00 at 4.19: not a speed problem.
- **The black car is not learned.** One recording, of a car faster than the
  aircraft (4.83 m/s p50 against the 5 m/s cap). In aiot_v8k_01 the black car
  drove south and stopped behind the aircraft, and v8 flew north
  along the route every other target takes. Slowing Car_07 is a level change
  for the user to decide.

#### Limits
- One level; three flights per target; the white car and the box truck are
  ambiguous phrases in this level and are scored against every candidate.
- The new targets' start gate is the truth (`--gate-truth`); after it the
  model flies on its own perception.
- Not KPI-grade (simple_flight on the city rail, no hil).

### 2026-10-30 (night) — the hand-written follow cannot teach a white or a black car (0.182 and 0.069 within 30 m); a privileged teacher (`follow_vlm --teacher-truth`) follows the white car at 0.997 and records the other targets' training labels

#### Found
- **The hand-written follow is a red-car follow.** Flown as AIOT-VLA's expert
  on the level's white car (aiot_rec_w01, Car_22, "a white car") and its
  black car (aiot_rec_k01, Car_07, "a black car"), 600 s each:

  | recording | within 30 m of the car | median distance | ticks in its planner (lost) |
  |---|---|---|---|
  | aiot_rec_w01, white | 0.182 | 123.6 m | 0.609 |
  | aiot_rec_k01, black | 0.069 | 166.6 m | 0.784 |
  | aiot_rec_e01, red (for comparison) | 0.973 | - | 0.000 |

  On the white car it had no box on 15.6 % of the inferences in the first
  150 s after acquisition (0 % on the red car), most of them because its
  candidates were refused as off the street: "a white car" also grounds on
  white facades. It fell behind on every
  dropout, lost the car at a corner 60 m back, and then followed other white
  objects 130-160 m from it. The car was not too fast: Car_22 drove at 3.73
  m/s (p50, gap-limited), Car_10 at 3.20. A model cannot learn a target
  from a teacher that does not follow it, so the two recordings and a
  partial box-truck one (aiot_rec_t01, stopped) are kept but not trained on.
- **Two targets are ambiguous by construction** (Environment Builder's fleet
  list of 23 Sep): the level drives two white cars (Car_22 on loop A,
  Car_15 on B) and three box trucks (Car_00 and Car_23 on A, Car_14 on B);
  only Car_07 is black. Loops A and B both pass the start point.

#### Added
- **`follow_vlm --teacher-truth`** (`run_citylife_follow.ps1 -TeacherTruth`):
  a privileged teacher for TRAINING LABELS only. Each detector frame's
  candidates are replaced by one box drawn where the `--level-car` truly is
  (`Grounder._teacher_box`: its ground point and a point
  `subject_height_m` above it through the capture's camera, the class width
  at its slant range), with its true range and bearing as identity's
  features, so the start gate, the re-acquirer and `estimator_measurement`
  get a perfect measurement; the true position is carried forward between
  the level's 0.5 s polls (`teacher_truth_fn`). The detector still runs,
  so its cadence and load are a real flight's. The controller, the fence and
  the Shield fly as always. metrics.json carries `teacher_truth` (the box
  counts); such a flight is never a follow result. Refused without
  `--level-car`. `tests/test_follow_teacher.py` (5 tests, in CI).
- `build_labels`: every sample carries `expert_est`, the expert's own
  estimate of its subject at that tick (the flight log's served `est_xy`);
  `train --memory-agree-m M` drops a sample whose target memory is more than
  M from it (the memory on another truck or another white car). On the 18
  red-car recordings the memory is within 2.9 m of the expert's estimate at
  the median, more than 15 m away on 4.3 % of the samples.

#### Flown
- aiot_teach_smoke1 (`--teacher-truth`, Car_22, "a white car", 180 s): within
  30 m on 0.997 of the ticks, median 20.1 m, every tick in track; the
  teacher's estimate within 0.61 m of the truth at the median.

#### Limits
- A teacher sees through walls: its labels keep chasing a car the camera
  cannot see. The student's target memory is what has to bridge those
  stretches; whether it does is the next flights' question.

### 2026-10-30 (evening) — AIOT-VLA v7 follows the red car as closely as the hand-written follow: a lateral step and a target memory take it from 0.265 to a mean of 0.968 within 30 m after acquisition (3 flights); protocol v2 gains a lateral step; groundwork for other targets

#### Found
- **The expert slides sideways; v3-v6 could not.** In the two clean expert
  recordings (aiot_rec_e01, e02; 11,997 ticks) the hand-written follow's
  commanded velocity has a sideways component above 0.5 m/s on 39 % of the
  ticks (p90 1.4 m/s) with the camera on the car: its fence slides along
  keep-outs ("skirt" mode, 21 % of the ticks, sideways p50 1.34 m/s,
  forward 1.65 m/s) and approaches them ("near", 14 %). It backs up on
  12 % (median -0.55 m/s). AIOT-VLA's labels kept the forward and yaw
  components only, so beside a building its lesson was to slow down
  without turning: aiot_v6_01 crept at 1.2 m/s along a building under the
  Shield's building-clearance repairs while the car pulled away.
- **A box's bottom edge is a good range.** On the 18 recordings (80,797
  frames), OWL-ViT's box at the phrase's best patch is within 3 deg of the
  car's true bearing on 0.897 of the frames with the car in view (p50
  error 0.4-0.5 deg). The flat-ground point under its bottom edge is
  11-12 % short at 15-25 m (the car's near end, not its centre).
- **The phrase against its look-alikes.** "a red car" also fires on the white
  and blue cars: a logit above -3.0 passes 0.958 of the boxes on the car and
  0.220 of the frames without it. Requiring "a red car" to beat "a white car"
  and "a blue car" at that patch keeps 0.950 and passes 0.026.

#### Added
- **A lateral step in protocol v2** (`guardrail/vla_link.py`,
  `guardrail/aerialvla_codec.py`, `demo/vla_server_foreign.py`): a fourth
  bin, `right_m` (-2.5..2.5 m), "FF DD YY RR". The server writes it
  (`encode_step(right_m=)`), the codec reads it only when asked
  (`parse_action(lateral=True)`; upstream's reader still refuses a fourth
  group), the client checks text, bins and step against each other, and the
  step tracker flies it as a body-frame velocity (fwd_m / H, right_m / H) on
  the current heading, the pair capped at the speed cap as a vector, or as an
  endpoint offset to the right. Upstream's three-bin step is unchanged.
  `tests/test_vla_lateral_step.py` (10 tests, in CI).
- **AIOT-VLA's target memory** (`training/aiotvla/core.py` TargetMemory): the
  box's ground point fed to a constant-velocity Kalman filter on the
  object's world position (demo/target_state.TargetState's model and gate),
  started on two agreeing boxes, moved by five agreeing gated-out ones,
  dropped after 8 s unseen, read out as 17 numbers in the body frame and fed
  to the head as one more token (`train --memory`). Against the truth: p50 /
  p90 position error 2.9-3.0 / 6.8-7.2 m on the clean expert recordings,
  3.8 / 10.9 m over all 18.
- `train --lateral`: the head also answers `right_m = v_right x H` (41
  cells, +-2 m/s); `build_labels` writes it. The server answers a lateral
  step for a lateral checkpoint and keeps a three-axis one's answers as
  they were.
- Groundwork for other targets (not yet trained or flown): VOCAB gains "a
  black car" and "a box truck" (appended, so cached rows keep their
  index); a checkpoint carries its contrast phrases and the server grounds
  those; `train --text` feeds the object words as the backbone's text
  embedding (`<cache>/text.npz`, `HeadConfig.n_text`).
- `vla_fly --level-car Car_00,Car_23,Car_14`: several level cars one phrase
  names (`level_actors.LevelCars`); the row's truth is the nearest
  (`subject_dist_m`), with `subject_tag` and every candidate's distance
  (`subject_dists`) for instance stability. The level has three box trucks
  and two white cars (Environment Builder's fleet list of 23 Sep).
- `run_citylife_follow.ps1 -ObjectWidthM`: the stand-off of a level vehicle
  is 0.16 of the frame width on this width; 4 gives every vehicle the car's
  15.8 m (a box truck's 6 m gave 23.7 m).

#### Changed
- **A level car is truth for the vehicle it is named as**
  (`follow_vlm.LEVEL_CAR_CLASSES` car, van, truck; `car_is_truth_for`):
  `--level-car Car_00 --object "a box truck"` was refused, since truth
  existed only for "car". The scripted car is still truth only for "car";
  a bus or a person behind a level car is still refused
  (tests/test_range_and_lock.py).

#### Flown (AIOT-VLA v7, `vla_fly` e2e, Car_10, 420 s, start gate 30 m, Shield on)
| flight | within 30 m after acquisition | median distance | Shield repairs |
|---|---|---|---|
| aiot_v7_01 | 0.961 | 20.3 m | 9 ticks |
| aiot_v7_02 | 0.983 | 19.8 m | 10 ticks |
| aiot_v7_03 | 0.961 | 20.9 m | 33 ticks |

Mean 0.968 against 0.972 for the hand-written follow (n=5) and 0.265 for v6.
All three flights started late (the car 16-17 m away at acquisition), were
flown by the model on 2,006-2,008 of their 2,015-2,016 ticks after it, and had
the car within 30 m at the end of the mission; the farthest it got was
31.4-32.2 m. All three passed the corner where every v3/v4/v6 flight lost the
car, at most 25.4 m behind it there. P0 escapes 0.0. v7 = v6's
recipe (labels R1+R2, 46,071 samples, box features, 10 slots, motion dropout
0.5) with `--lateral --memory`; hold-out right_m MAE 0.037 m (constant
baseline 0.252), dpsi MAE 0.0056 rad (v6 0.0062).

#### Limits
- One target (the red car), one level, one route; three flights. Not
  KPI-grade (simple_flight on the city rail, no hil).
- The memory is classical (a Kalman filter in the VLA package); the action
  is the trained head's. Backing up is still not in the step (fwd_m >= 0).
- Other targets: the code is ready, the recordings are not.

### 2026-10-30 — NTUT-VLA becomes AIOT-VLA and moves to the FrontCamera; 18 new training recordings, four models flown three times each (best mean 0.265; v3, v4 and v6 all lost the car at the same corner); four grounders compared on the car's true position; the "false start-gate seeds" retracted

#### Retracted
- **"False start-gate seeds" (CHANGELOG 2026-10-29 (night), Flown; the
  design doc).** Two AIOT-VLA flights (ntut_v1_search600_01,
  ntut_v2_rate_03) were called not comparable because vla_fly's start gate
  had "acquired something other than Car_10". Both were correct
  acquisitions. The level car drives from the moment the scene loads, and
  in those flights it began 29-34 m away and drove off; the gate
  (`--acquire-max-range-m 60`) committed to it at about 60 m. The same
  happened in the 28 Oct DroneCATS and r1 e2e flights (car 15-22 m away at
  take-off, acquired at 61 m). With those flights counted, v2 with
  `--rate-yaw rate` scored 0.355, 0.253 and 0.0: mean 0.203 over 3 flights,
  not 0.304 over 2.

#### Found
- **The VLA flights started under a harder gate than the hand-written
  follow.** The hand-written follow's start gate accepts a car within 30 m
  (its metrics: max_range_m 30.0, 24-44 candidates refused as too far in
  every flight of 27-29 Oct). The matrix flew every pure-VLA flight with
  `--acquire-max-range-m 60`, so a VLA could be asked to chase a car 60 m
  away and leaving. AIOT-VLA's evaluation flights from v3 on use vla_fly's
  own default, 30 m, the hand-written follow's gate. Earlier VLA scores
  (the 27-28 Oct matrix, NTUT-VLA v0-v2) were flown at 60 m.

#### Changed
- **NTUT-VLA is renamed AIOT-VLA** (`training/aiotvla`, family `aiotvla`,
  tests/test_aiotvla.py, docs/DESIGN-aiotvla.md), for the lab's identity.
  `ntutvla` stays in the codec's allow-list as a legacy family, so the
  29-30 Oct flights (`demo/out/ntut_*`) still classify as this model's; no
  server serves it.

#### Added
- `follow_vlm --record-vla-front-source detector`
  (`run_citylife_follow.ps1 -RecordVlaFrontSource detector`): the recorder
  and its DAgger windows take the detector's FrontCamera (768x432, pitched
  -20 deg) as the front frame, as `vla_fly --vla-front-source detector`
  sends it.
- AIOT-VLA: a longer memory (`train --long-history`: 10 slots back to 5.5 s);
  a checkpoint carries its history, front camera and backbone, and the
  server refuses a `--front-source` the checkpoint was not trained on;
  OWLv2 B/16 as a backbone option (pooled to the 24x24 grid);
  `build_labels --exclude-modes` (the follow's street-map planner ticks,
  which fly on knowledge no frame shows).
- AIOT-VLA box features (`train --box-features`, `HeadConfig.n_stats` 8):
  OWL-ViT's own box (cx, cy, w, h) at the phrase's best patch, cached
  (`B.npy`) and fed per slot; the server's Ring computes the same. Its
  width falls with range (median 0.072 at 15-20 m, 0.045 at 30-40 m).
- `train --onpolicy-weight`: a loss weight for captures whose executed
  command was not the expert's.

#### Recorded (training data, all on the FrontCamera, 600 s each, `aiot_rec_*`)
- R1: 2 clean expert, 4 strong DART. R2: 6 DAgger, v3 flying 4-8 s windows
  (about 46 % of the ticks). R3: 6 DAgger, v4 flying 15-25 s windows. 18
  recordings, 84,067 captures.
- Grounders on 571 FrontCamera frames, against the car's TRUE position
  projected into the frame (a hit: within 3 deg of bearing and 12 % of the
  height):

| backbone | hit rate | bearing error p50 | model time |
|---|---|---|---|
| OWL-ViT B/32 (in use) | 0.751 | 2.2 deg | 32 ms |
| OWLv2 B/16 | 0.690 | 1.5 deg | 139 ms |
| Grounding DINO tiny | 0.800 | 0.5 deg | 141 ms |
| Grounding DINO base | 0.797 | 0.5 deg | 171 ms |

#### Flown (AIOT-VLA, `vla_fly` e2e, FrontCamera, `--rate-yaw rate`, start gate 30 m, 420 s, Shield on)
| model | data | within 30 m after acquisition | mean |
|---|---|---|---|
| v3 | R1, 19,211 samples | 0.279, 0.172, 0.307 | 0.253 |
| v4 | R1+R2, 46,071 | 0.230, 0.264, 0.298 | 0.264 |
| v5 | R1+R2+R3, 64,874, on-policy x2 | 0.211, 0.189, 0.191 | 0.197 |
| v6 | as v4, with box features | 0.258, 0.269, 0.268 | 0.265 |

Every flight started late (the car 14-19 m away at acquisition), P0
escapes 0.0 in all 12. One more v3 flight at the old 60 m gate
(aiot_v3_01) started early and scored 0.0. The hand-written follow
scores 0.972 (n=5) on the same scenario and gate.

#### Found
- **AIOT-VLA loses the car at one corner.** All nine flights of v3, v4 and
  v6 lost it 60-63 s after acquisition, at the same place: the aircraft
  near (52, 106), the car turning at (80, 120). v5's three flights lost it
  sooner, 39-40 s after acquisition, on the straight before (the aircraft
  at (44, 53), the car at (44, 82)). The hand-written follow takes that stretch
  at 4.4-4.7 m/s, 20-22 m behind. AIOT-VLA slows to 1.1-1.9 m/s, with
  Shield repairs, while the car keeps 3.5 m/s. When the car turns, it stops
  28 m back, sets off late, and the Shield holds it off a roadside tree.
  Offline, on the clean hold-out flight, v4 has no speed bias by range
  (30-40 m: label 4.06 m/s, prediction 4.05 m/s). The states it fails in
  are states the expert never reaches.
- **Long DAgger windows hurt.** In 15-25 s windows v4 lost the car often;
  up to 3,855 of a recording's ticks were the expert's planner afterwards
  and were left out. v5, trained with them, lost the car sooner.

#### Limits
- Not the >0.9 the work aims at: the best mean is 0.265, against 0.972
  for the hand-written follow. Three flights per model; one level, one
  car, one route; simple_flight on the city rail, not KPI-grade.

### 2026-10-29 (night) — an independent review re-states 27 of the 64 "DONE" items; the lab's own VLA (NTUT-VLA) is built, served, flown and given five more recordings; a rate step's turn was never flown; three review findings fixed

#### Retracted
- **The outline's "64 DONE" (docs/FINAL-REPORT-outline.md, committed
  earlier on 29 Oct).** Four independent reviewers, one per area, checked each of the 64 items then marked
  DONE against the grant's own wording, not against our evidence notes:
  35 held, 27 did not, and 2 were unclear. Each of those 29 is now PARTIAL or
  DRIFT, with the reviewer's reason in Appendix A; three of them (WP2-04,
  WP2-07, WP4-20) were fixed the same night (below). The outline now reads
  **38 DONE, 83 PARTIAL, 13 DRIFT, 2 MISSING** of 136. The 13 DRIFT items
  are listed in its section 7. Among them: the repair operators act in a
  world frame, not the VLA's body frame (WP3-04, ARCH-04); more than one
  rule-evaluation path (WP3-22, X-13); policies are metre-canonical, not
  WGS84 (WP1-14); the 20 July gate was missed (WP1-21, WP3-01); and G1
  passed 100 days late over SIM_AirSim, not HIL_* (ARCH-15). Among the
  PARTIAL items: no VLA has emitted a 10 Hz setpoint into ArduPilot
  (ARCH-01, ARCH-21); the city rail flies a 3 s lookahead, not 5 s
  (WP3-05); the smoke matrix has never run on a build (WP3-19, WP4-14).

#### Fixed
- **WP2-04: the CSP schema refused the spec's own worked example.**
  `policy_id` and `selection` (our two additions) are now optional; the
  compiler still fills both. `guardrail/csp.schema.json` is regenerated, and
  `tests/test_csp.py::test_the_specs_own_worked_example_validates` checks the
  example against the model and the exported schema.
- **WP2-07: the cost-map adapter "emits YAML".** `CostMap.save` also writes
  the header as `<stem>.yaml`, the same document; `load_cost_map` reads
  either.
- **WP1-17 / WP1-18: the stored-run census failed on the 9 REST-flown runs
  of 29 Oct.** Their manifests record the last generation's hash, which no
  policy file or lock recipe rebuilds. `tools/wp1_roundtrip_kpi.py
  run_generation_chain` resolves such a run through its own
  `policy_g<N>.json` chain, and only from a `policy_g0.json` that is a known
  policy. The census is 219/219.

- **WP4-20: the Stress Testing harness's bundles carried no Shield
  audit.** The headless episode now keeps the flying Shield's audit records
  (`EpisodeRun.audit`, `guardrail.audit.AuditLogger.in_memory`). At episode
  end `write_bundle` packs them as `shield_audit.jsonl`, the grant's name, in
  both the sweep's bundles and the gRPC EndEpisode's
  (`tests/test_sweep.py::test_the_episode_bundle_carries_the_shields_audit`).
- **A rate step's turn was never flown at 10 Hz.** vla_fly's rate
  executor (`--step-as-rate`) reads dpsi as a target heading (capture
  heading + dpsi) and holds it within 0.05 rad. A model that answers ten
  times a second asks for 0.02-0.2 rad per step, which lies inside the
  tolerance or was already flown since the capture. On ntut_v2_01 the car's
  bearing swept from -2 to -110 deg at a corner while the aircraft flew a yaw
  rate of 0.00. `vla_fly --rate-yaw rate` (`TrackerConfig.rate_yaw`; the
  default `heading` is unchanged) flies dpsi/H as the yaw rate until the next
  answer. DAgger windows take the same with `follow_vlm
  --perturb-vla-rate-yaw` and `run_citylife_follow.ps1 -PerturbVlaRateYaw`.

#### Added
- **NTUT-VLA, the lab's own small VLA** (`training/ntutvla/`,
  docs/DESIGN-ntutvla.md). Frozen OWL-ViT B/32 (the detector's own
  weights) grounds the object words in the VLAFront frame; an 8-slot
  history of those maps plus the aircraft's own motion feeds a trained head
  of 1,678,505 parameters, which answers a step over 0.5 s. No detector box,
  compass hint or hand-written servo is an input. Served by
  `demo/vla_server_foreign.py --model ntutvla` (family `ntutvla` added to
  `aerialvla_codec.FOREIGN_FAMILIES` and its `tools/todo.py` copy; `fp16` added
  to `vla_link.PRECISIONS`). Model time is 10.3 ms p50 on the RTX 4090.
  Replaying hold-out `ft_expert_03` through the server reproduces the
  offline numbers (dpsi MAE 0.0027, correlation 0.994).
- **On the Jetson Orin Nano** (MAXN_SUPER): v1 answers in 48.4 ms p50 (52.0 ms
  p95), 19.1 decisions per second over 600 hold-out captures replayed on
  the board, with the PC's accuracy (dpsi correlation 0.989). That is the
  first learned model here to meet the grant's 10 Hz on an Orin (ARCH-03),
  measured for the server alone, not in a flight.
- `training/ntutvla/build_cache.py` (backbone maps of every capture and its
  mirror, 5.3 ms per frame pair), `build_labels.py` (labels from the
  expert's asked command) and `train.py` (`--motion-dropout`).
- tests/test_ntutvla.py (19 tests), on CI; a rate-yaw test in
  tests/test_vla_prev_front_rate.py and one in tests/test_vla_dagger.py.

#### Found
- **Labels from achieved motion make a slow pilot.** v0's labels were the
  expert's displacement over 0.5 s, and vla_fly flies a step as a velocity
  command. The airframe lags, so the model asked for less than the expert
  did: ntut_v0b_01 fell from 9 m to 47 m behind the car in 50 s. v1 learns
  the expert's ASKED command (54,814 training samples against 30,091, since
  DART and DAgger captures keep their label).
- **A copycat head.** v0, with its own motion zeroed on the hold-out:
  forward MAE 0.0675 to 1.1158 m. With half the training samples' motion
  dropped (v0b) it is 0.085; for v1 the motion proprio changes nothing, and
  without vision its dpsi correlation falls to 0.23.

#### Recorded (training data; the hand-written follow with `--record-vla`)
- `ntut_dagger_02`, `_03`, `_04`: v1 flies 3-6 s windows every 9-13 s and the
  expert relabels; `ntut_dart_01`, `_02`: the executed yaw replaced by
  0.6-0.9 rad/s for 1.0-1.5 s every 5-8 s, at 0.5-1.0 x speed. 12,164
  captures. `ntut_dagger_01` aborted on a simulator ConnectionReset before its
  first tick and recorded nothing. v2 trains on all 24 recordings: 66,974
  training samples.

#### Flown (CityLife, Car_10, vla_fly e2e, 420 s, Shield on; `-SimOffscreen -SimMaxFps 30`)
| flight | within 30 m after acquisition | P0 escapes |
|---|---|---|
| ntut_v0b_01 | 0.161 | 0.0 |
| ntut_v1_01 | 0.291 | 0.0 |
| ntut_v1_search600_01 | 0.031: not comparable, the start gate acquired a false seed at 3.6 s | 0.0 |
| ntut_v2_01, _02, _03 | 0.243, 0.208, 0.185 (mean 0.212) | 0.0 |
| ntut_v2_rate_01, _02 (`--rate-yaw rate`) | 0.355, 0.253 (mean 0.304); the car followed up to 119 s after acquisition, against 38-64 s without | 0.0 |
| ntut_v2_rate_03 (`--rate-yaw rate`) | 0.0: not comparable, a false seed at 2.4 s | 0.0 |

On the same scenario the 28 Oct pure-VLA flights scored 0.153 (AerialVLA)
and 0.086 (DroneCATS); the hand-written follow scored 0.972 over n=5. Every
pure VLA loses the car within about a minute, at a corner.

#### Tests
- Every file `.github/workflows/tests.yml` names, on the lab checkout: 129
  files, 3,348 passed, 60 skipped. 3 fail, the known lab-only ones
  (test_build_progress_oct_data, test_explainer_deck, test_track_truth;
  2026-10-29 later, Found). That run found three regressions, now fixed:
  test_replay (the new .yaml cost-map header was not moved aside on a
  re-run), test_policy_hash and test_kpi_report (the nine REST-flown runs).

#### Limits
- At most three valid flights per model; not a ranking. simple_flight on the city
  rail: outside the ArduPilot architecture, not KPI-grade.
- Trained on the red car in one level; language generalisation untested.

### 2026-10-29 (REST) — the grant's hot-apply channel drives a flying ArduPilot: a no-fly zone spawned, rotated, moved and switched off over REST, the Shield acting on it; one flight per breach action

The audit's open items that needed a flying SITL + MAVROS run and no
simulator: WP1-15, WP3-12, WP4-03 and WP2-15 (a REST hot-apply into a
flight, and the CSP re-issued per generation), WP3-07 (the FSM's edges,
Land among them) and WP3-08 (one flight per breach action).

#### Flown (the ROS 2 rail, `sitl/run_ros2_demo.sh on`: ArduPilot SITL Copter in WSL, mavlink-router, MAVROS 2, the VLA stub; topology `dev`)
All 9 flights carry the signed `fase3-sim-demo` bundle (one polygon fence
`nfz-square`, the 10-20 m band, a 4 m/s speed cap) and serves the REST
hot-apply API on 127.0.0.1 (`--hotapply-port`). A client
(stdlib `urllib`, in the session's scratchpad) sent each event at a fixed
time after `mission_start`; every request and answer is in
`<tag>.client.jsonl` beside the run's own records.

| Flight | Mission | Events over REST (mission time) | What the Shield and the FSM did | Ended |
|---|---|---|---|---|
| `ros2_hotapply_01` | to (30, 30), 6 m/s | spawn `nfz-hot-1` 1.1 s, rotate 15 deg 4.1 s, translate 7.1 s, switch off 12.1 s | X1 Normal->Brake and G3 ->Loiter at 0.1-0.2 s on `nfz-square` (repair 8.14 m against theta 2 m), before any event; Loiter 30 s, G5 ->RTL, G9 home reached | 30.3 s, the FSM |
| `ros2_hotapply_02` | to (0, 30), 6 m/s | the same, with the zone across this path | G1 ->Brake at 0.1 s (the speed cap repaired); G3 ->Loiter at 1.1 s on `nfz-hot-1` (repair 2.90 m); no recovery, as the 6 m/s ask kept the cap repaired; G5, G9 | 34.6 s, the FSM |
| `ros2_hotapply_03` | to (0, 30), 3.5 m/s | the same | X1 ->Brake at 1.16 s and G3 ->Loiter at 1.26 s on `nfz-hot-1` (repair 3.00 m); after the switch-off, **G8 Loiter->Normal** at 14.2 s (hard rules cleared 2.05 s); flew on | 22.0 s, target reached, landed |
| `ros2_action_land_01` | to (0, 30), 3.5 m/s | spawn a zone whose `violation_action` is `land`, 1.0 s | **G12 Normal->Land** at 1.23 s, LAND | 41.3 s, the FSM |
| `ros2_action_rtl_01` | the same | a zone with `RTL` | **G11 Normal->RTL** at 1.15 s; G9 home reached at 1.21 s | 1.2 s, the FSM |
| `ros2_action_loiter_01` | the same | a zone with `loiter` | **X4 Normal->Loiter** at 1.15 s; G5 ->RTL at 31.2 s, G9 | 34.5 s, the FSM |
| `ros2_action_brake_01` | the same | a zone with `brake` | **X1 Normal->Brake** at 1.13 s (the rule asks for brake); braked on all 889 ticks that followed, never escalated | 90.0 s, the time cap |
| `ros2_action_monitor_only_01` | the same | a HARD P0 zone with `monitor_only` | no repair; flown through: 1.2 s inside the zone, P0 escape rate 0.582418 | 9.2 s, target reached |
| `ros2_action_monitor_only_soft_01` | the same | a SOFT P2 zone with `monitor_only` | no repair; flown through: 1.2 s inside the zone, P0 escape rate 0.0 | 9.2 s, target reached |

All four events of each hot-apply flight were accepted (HTTP 200), each
one generation (0 to 4) with its own policy hash, and each wrote that
generation's `policy_g<N>.json` and `csp_g<N>.json`: the CSP of generation
0 does not name `nfz-hot-1`, those of 1-4 do (WP2-15). Receipt to swap,
against the grant's 50 ms: 0.444-4.443 ms over the 12 events, 0.66-1.035
ms for the six single spawns, 18 events in all. The rail re-published the fences and uploaded
them to ArduPilot's GeoFence backstop after every event (5 uploads per
hot-apply flight; the zone switched off was taken out of the upload).
P0 escape rate 0.0 on every flight but the hard `monitor_only` one.

#### Found
- A hard P0 rule whose action is `monitor_only` is accepted over REST and
  then flown through, and its ticks count as P0 escapes (0.582418 on
  `ros2_action_monitor_only_01`). That is the Shield's stated design
  (`guardrail/shield.py`: a P0 monitor_only rule "shows up in the escape
  count, which is the honest place for it"); the same zone as a soft P2
  rule escapes nothing. Whether the API should refuse a hard
  `monitor_only` zone is a policy question, not settled here.
- On the first flight the FSM had left Normal on the static fence within
  0.2 s of `mission_start`, so the hot-applied zone was never flown
  against; at 6 m/s the speed cap is repaired every tick and blocks G8.
  The flights that show the Shield acting on a REST zone are the 3.5 m/s
  ones.
- `ros2_action_rtl_01` reached "home" 0.06 s after RTL began, 1 s into the
  mission, because the aircraft was still within the home radius.
- `ros2_action_land_01` counts 35 s outside the altitude band: the
  autopilot's LAND descends through the 10 m floor. Those ticks are
  autopilot-flown, so they are outside the P0 rate.
- **G10 (Land -> landed) never fired.** On `ros2_action_land_01` the
  aircraft was on the ground from 25.9 s (0.27 m, then -0.09 m), but the
  FSM stayed in Land, not terminal, and the mission ended on the node's
  40 s fail-safe grace at 41.3 s. The node reads `landed` from
  /mavros/extended_state; whether that topic reached it is not recorded
  (the bag does not carry it). Open.
- A `brake` rule holds the aircraft in Brake for as long as it is
  violated: one onset, so N-in-T (G2) never counts three, and the zone
  never clears, so G7 never fires; `ros2_action_brake_01` braked until the
  90 s time cap.
- The first RTL attempt sent `"rtl"` and was refused with 422 (the DSL's
  value is `RTL`); its flight flew to the target with no event. The re-fly
  wrote into the same folder (Windows names are case-insensitive) and the
  runner moved the refused flight into `ros2_action_rtl_01/_previous`; the
  folder's client log holds both attempts' requests, in order.

#### Changed
- `docs/FINAL-REPORT-outline.md`: WP2-15, WP3-12 and WP3-08 move to DONE
  on these flights, WP1-15, WP4-03 and WP3-07 gain them as evidence; the
  map of the 136 obligations reads 64 DONE, 68 PARTIAL, 2 MISSING, 2 DRIFT
  (61 and 71 before).
- `~/venv-ros` (WSL) gained fastapi 0.162.0 and uvicorn 0.54.0, the
  pyproject `api` extra that `sitl/setup_ros2.sh` installs; the Shield
  node refused `--hotapply-port` without them.

#### Limits
- `dev` topology, a dirty tree (`e5b2e51-dirty`: an untracked document)
  and no PI waiver: none of this is KPI-grade.
- The events came from a scripted client, not the harness's gRPC
  `HotApply` (WP4-03 asks for the harness); `corridor_swap` was not flown
  (the flown policy has no corridor); the perception rail still has no
  hot-apply listener.
- Edges flown here: G1, G3, G5, G8, G9, G11, G12, X1, X4. Not flown on
  this rail: G2, G4, G6, G7, G10, X2, X3, X5, X6, X7; Land was reached
  directly (G12), not through RTL (G6).

### 2026-10-29 (later) — a recovery from outside the altitude band climbs at the cap: the library's mean time to safe 4.31 to 3.14 s; a bundle's changelog is a diff against its previous version; the IR migration register; the follow comparison repeated to five flights

Continuing the grant's open obligations (the audit's WP3-17, X-10, WP1-22 /
X-09) and the user's request to repeat the follow comparison 3-5 times.

#### Changed
- **The Shield recovers at the climb cap** (WP3-17, `guardrail/shield.py`
  `Shield(recovery_law="cap")`, the default; 5c98142). A position outside
  an altitude band used to be brought back in one lookahead,
  vz = (aim - up) / L: a P controller with time constant L, whose last 2 L
  metres take L ln(2 L) seconds, so at the grant's 5 s forecast a recovery
  slowed to a crawl near the band edge. It now flies the smallest kinematic
  climb cap in force, held only so its lookahead endpoint stays a re-entry
  margin short of the far edge, and never slower than the old law. Recoveries
  stay exempt from theta. The altitude cells (headless library, grant
  forecast):

  | Cell | old law | at the cap |
  |---|---|---|
  | altitude-recovery-depth, floor 10 m: from 0.5 / 2 / 6 / 9.5 m | 11.7 / 10.9 / 8.0 / 2.1 s | 4.8 / 4.1 / 2.1 / 0.3 s |
  | altitude-recovery-depth, ceiling 20 m: from 20.5 / 24 / 30 m | 2.1 / 8.0 / 11.9 s | 0.3 / 2.1 / 5.1 s |
  | altitude-floor-recovery, from 3 m | 10.3 s, 103 repairs | 3.6 s, 36 repairs |

  altitude-recovery-depth's gate moves from "<=12.1" to "<=5.2" (the 10 m
  deepest cell at 2 m/s plus two ticks); it is still implementation-derived.
  The old law is kept line for line as `recovery_law="lookahead"`: the
  velocity identity proof and the c77c77d row digests fly it, and so does
  every bundle from before this change (bundle extras schema
  `sweep-episode-extras/4` records `arm.recovery`; /1 to /3 re-fly on the
  old law). `experiments/sweep_scenarios.py --recovery-law lookahead` flies
  the comparison; the flag is refused with `--publish` and on the sitl
  backend.
- **docs/data/scenario_sweep.json republished** on the cap law (the
  library, 180 episodes; d9ba810, clean tree). Counts unchanged: 140 pass,
  22 tracked, 0 fail, 18 known failures; P0 escape 0.0. What moved:

  | Shield-on rollup | before | now |
  |---|---|---|
  | mean time to safe (37 recoveries, 3 censored) | 4.308 s | 3.135 s |
  | max time to safe | 11.9 s (altitude-recovery-depth, 30 m) | 6.7 s (radius-scaling-grow-deep) |
  | P0 violation ticks | 10,947 | 10,521 |
  | mean repair magnitude | 6.919 m/s | 7.114 m/s |
  | AltitudeClamp repair ticks / mean | 664 / 0.83 m/s | 238 / 2.38 m/s |
  | null passthrough P0 escape rate | 0.234356 | 0.225236 |
  | fail-safe, distinct flights | 105/125 = 0.84 | 105/124 = 0.846774 |

  The repair magnitude rises because a recovery is now one large repair
  over a few ticks instead of many small ones; the null's rate falls
  because its denominator, the violation ticks, shrank. The
  three-simultaneous-violations family (known failures) recovers 0.1-0.2 s
  sooner and now flies one KPI signature instead of two, so the
  distinct-flight count drops by one; the per-episode fail-safe score is
  145/171 = 0.847953 as before. The explainer deck's facts follow (`tools/deck/build_explainer_deck.js`:
  time to safe 3.13 s, repair magnitude 7.11 m/s, null 0.225); the deck
  file already shared is not rebuilt.

#### Added
- **A bundle's changelog is a diff against its previous version** (X-10,
  `guardrail/bundle.py`, 895b5a0). `python -m guardrail.bundle <policy> -o
  <out> --previous <bundle.tar.gz | policy file>` writes `changelog_diff` into the
  manifest (the previous policy_id, version and policy_hash; the rules
  added, removed and changed, with the changed fields; the policy-level
  fields that changed) and, in place of the default "initial", a changelog
  text made from it. A previous bundle that does not verify is refused.
- **The IR migration register** (WP1-22, X-09, `docs/IR-MIGRATIONS.md`,
  7c02284): one row per IR-shape change after the Q1 freeze, from `git log`
  of `guardrail/models.py` and the lock, with the version, whether a stored
  hash moved, and the migration. It records one deviation: `locked_by`
  (2026-10-19, 2385f84) added 11 fields without raising
  `IR_SCHEMA_VERSION` from 2.1.0, by this project's additive rule, where the
  grant asks for a bump on every change. Open with the PI.

#### Fixed
- **The CI workflow's header** (`.github/workflows/tests.yml`, d9ba810)
  names `tests/test_city_episode.py` under WP2; test_harnesstruth_ci had
  failed on it since f6ee433 added the file to the run list only.

#### Flown (Project AirSim CityLife, simple_flight, PC)
Twelve more follows of the red car, `demo/out/rep3_*`, `rep4_*`,
`rep5_*_20261029`, in the configuration of 27-28 Oct, bring the
hand-written follow, DroneCATS hierarchical and AerialVLA hierarchical to
five flights each. All twelve: 0 mission collisions, P0 escape 0.0.

| Setup | Flights 27-29 Oct | Mean (sample std) |
|---|---|---|
| hand-written follow (no VLA) | 0.966, 0.954, 0.976, 0.983, 0.980 | 0.972 (0.012) |
| DroneCATS-2B, hierarchical | 0.929, 0.939, 0.928, 0.940, 0.931 | 0.933 (0.006) |
| AerialVLA original, hierarchical | 0.987, 0.309, 0.999, 0.996, 0.979 | 0.854 (0.305) |
| AerialVLA FT r1, hierarchical | 0.884, 0.884, 0.879 | 0.882 (0.003) |
| AerialVLA FT r3, hierarchical | 0.934, 0.869, 0.887 | 0.897 (0.034) |

The commit-headroom guard of the runner had skipped seven of these on the
first attempt; it now counts the growth room of a system-managed pagefile.

#### Found
- AerialVLA hierarchical is the best VLA setup in four flights of five
  (0.979-0.999, a stand-off of about 12.4 m) and the worst in one: on
  `cmp2_hier_aerovla` (0.309) the detector had no box on the car for 2,921
  of 4,169 ticks after a corner. It is the detector losing the car, not the
  VLA's latency: answers arrived in 0.49-0.63 s (p50) on every flight.
- With the car locked, the VLA's only lever is the stand-off, and the score
  falls with it; the VLA's own search flew 0-11 % of the ticks. Training the
  stand-off cannot beat a fixed 12 m controller on this score.
- The hand-written follow spreads 0.954-0.983 over five flights: one flight
  is not a ranking.

#### Tests
- The per-build list (.github/workflows/tests.yml) on this branch: 128
  files, 3,214 checks passed and 173 skipped (tracker data, node_modules,
  flight logs and GPUs its checkout lacks), 0 failed after the two fixes
  that follow. Both were identity proofs of the old law and now fly
  `recovery_law="lookahead"`: test_shieldcore_identity (the 8 Oct flight
  replays to 4418ae0's decisions; on the new law it departs at tick 219)
  and test_build_deck_data (the September deck's altitude-curve model is
  of the old law; `tools/build_deck_data.py`).
- The same list on the lab checkout, with the flight logs and
  node_modules: three files fail, none from this work, and are left open
  (below).

#### Added (report)
- **The final report's outline and evidence map** (X-01,
  `docs/FINAL-REPORT-outline.md`, a7a0d9a): the report's sections, the
  locked-KPI table with the best evidence so far, the deviations register
  (WP2-16 recorded there: the multi-scale geometry is derived from the IR
  when a policy is compiled or loaded, and the bundle carries the IR, not
  the scales), the VLA measurement set with the repeats above, the PI's
  open decisions, the plan to 2026-11-30, and the 136 obligations with
  their evidence (61 DONE, 71 PARTIAL, 2 MISSING, 2 DRIFT). An outline,
  not the report: section 5's KPI numbers come only from a `hil` run.

#### Fixed (deck)
- **The explainer deck's build had refused since 20 Oct**
  (`tools/deck/build_explainer_deck.js`, f28c429): 6987366 rewrote
  `Shield._predict`'s docstring for the own-velocity forecast and took the
  phrase the `shield_cv` fact checks with it. The fact is about the 14 Oct
  flight, whose forecast held the commanded velocity; it now checks the
  docstring's words for that case. 147 facts, 0 mismatched.

#### Found (tests left failing)
- test_build_progress_oct_data: `docs/data/progress_oct.json` changed on
  22 Oct (33447d7) after the October progress pack was built, so the
  pack's build stamp no longer matches; rebuilding means rebuilding a
  delivered pack.
- test_explainer_deck: the explainer pack folder (gitignored) holds
  SPEAKER-SCRIPT.md and .pdf, added on 14 Oct after the builder wrote its
  README, which does not list them.
- test_track_truth: on `orin_hier_aerovla_20261028` the detector's median
  error is 4.0 px against the centre null's 3.3, with the car in frame on
  100 % of rows (rounded); that is outside the test's centre-tie tolerance (5 % + 0.1 px)
  and fits none of its four known shapes. The tolerance is not widened to
  fit it.

#### Limits
- The follows fly simple_flight, outside the ArduPilot architecture: they
  are a comparison of models, not KPI evidence. The library is the headless
  harness with a modelled autopilot; the reference sweep's 2.0 s belongs to
  a dynamic-NFZ sweep, and the library's 3.135 s is still above it.

### 2026-10-29 — a learned model flies ArduPilot for the first time, the perception rail flies over MAVROS 2 and is stamped dev, a drifting BRAKE is taken back, MAVROS's clock and text were never received

Continuing the grant's open obligations: the audit's final-delivery items
that no rail had flown (ARCH-18 the perception rail with its VLA pilot and
`--link mavros`, ARCH-20 / ARCH-02 a learned model on ArduPilot).

#### Flown (Project AirSim CityLife, ArduPilot SITL Copter 4.5.7 in WSL)
| Flight | Pilot and link | Ticks | Ended | P0 escapes | Ticks in a P0 margin | Mean / max time to safe | ArduPilot real-time factor |
|---|---|---|---|---|---|---|---|
| `pas_vla_goto_01` | AerialVLA bf16, goto (50, -78), pymavlink | 1000 | the FSM's RTL | 0 | 110 | 5.501 / 9.4 s | 0.972 |
| `pas_vla_goto_02` (after the reclaim below) | the same | 1792 | max_s, closest 5.51 m to the goal | 0 | 51 | 1.155 / 1.6 s | 0.965 |
| `pas_vla_approach_01` | AerialVLA nf4, approach of the red car, pymavlink | 84 | close_box at 10.23 m | 0 | 0 | - | 0.949 |
| `pas_dev_mavros_01` | the servo follow, MAVROS 2 in WSL | 1192 | max_s | 0 | 37 | 1.849 / 2.499 s | not read (below) |
| `pas_dev_mavros_02` (after the QoS fix) | the same | 195 | the FSM's RTL (G4, N-in-T) | 0 | 0 | - | 0.968 |
The VLA flights are the first in this project where ArduPilot, not Project
AirSim's simple_flight, flew a learned model's commands; every tick went
through the Shield and the escalation FSM. On goto_02 AerialVLA authored
0.8828 of the ticks (29 decisions, p50 1.15 s). The MAVROS flights ran the
node in a new WSL environment (`~/venv-pas`: ROS 2 Jazzy's rclpy and
MAVROS, torch 2.6 CUDA, transformers 4.40.1, the projectairsim client),
where OWL-ViT ran at 8.12-9.009 Hz. `pas_dev_mavros_02` is the first run
stamped with the grant's `dev` label; it is not KPI-grade because a dev
run needs the PI's written waiver and the tree was dirty.

#### Fixed
- **A BRAKE that drifts** (`demo/pas_ardupilot/perception_node.py`,
  `guardrail/response.HOLD_RECLAIM_MAX`, b2a2181): on goto_01 the FSM's
  Loiter was handed to ArduCopter BRAKE with its stop predicted legal, and
  BRAKE let the aircraft creep about 0.1 m/s into the 3 m building margin,
  0.36 m deep, with nothing flown, until the FSM sent it home. Such a hold
  is now taken back into GUIDED and flown as the Shield's answer to a stop
  when its position goes illegal on a hard rule after being legal in BRAKE;
  it stays in GUIDED until a forced hand-over or the end of the Loiter, at
  most 3 times per Loiter. Closed loop (BRAKE climbing 0.15 m/s under a
  7.5 m ceiling): 27 of 120 ticks unsafe and no RTL, against 92 and an RTL
  before. On goto_02 it fired once (BRAKE 0.01 m into bld-clearance).
- **MAVROS's clock and text** (`demo/pas_ardupilot/mavros_link.py`,
  659b103): /mavros/time_reference and /mavros/statustext/recv were
  subscribed reliably while MAVROS publishes them best effort, so on
  pas_dev_mavros_01 the real-time factor read None and no STATUSTEXT
  arrived (MAVROS's own log: "incompatible QoS. No messages will be sent").
  Both use the sensor profile now; the test checks every subscription's
  QoS.

#### Found
- A learned pilot on ArduPilot spends much of its time held: goto_01
  and goto_02 had 79 and 159 repaired ticks; AerialVLA stops and goes
  (0.13-0.16 decisions per second) and its steps run it towards the
  building margins of the street it flies down.
- The 2 goto flights differ in more than the reclaim (the model's path
  is not the same twice), so their two time-to-safe figures are one
  flight each, not a measured effect; the closed-loop test is the
  controlled comparison.

#### Tests
- test_own_velocity_forecast 28/28 (a new drifting-BRAKE test),
  test_pas_ardupilot 148/149 (1 skip), test_pas_vla 28/29 (1 skip),
  test_railvideo_recording 10/21 (11 skip).

#### Limits
- None of these runs is KPI-grade: pymavlink is no grant topology, a dev
  run needs the PI's waiver, and the tree was dirty (an untracked file).
  The hil topology needs Docker on the Jetson (installed with sudo).

### 2026-10-28 (night) — DroneCATS 2B on the Jetson Orin Nano itself: it fits in int4 but answers in 9.6 s and, beside the detector, timed out 34 times in 36; Jetson runs now record their commit and their two hosts; a CSP from every city flight; an independent paraphrase test

The user asked to try DroneCATS 2B on the Jetson and check the result, to
keep working on the grant's open obligations, and to move the flight data
off C:.

#### Added
- **DroneCATS quantised** (`demo/vla_server_foreign.py`, 17c3ab8, 892dc1b,
  f30bc28): `--precision nf4` (bitsandbytes) and `--precision int4` (torchao
  int4 weight-only, group 128, `tile_packed_to_4d`: torch's own tinygemm
  kernel). Both quantise the language layers and keep the vision encoder,
  projector and lm_head in bf16. Each is refused before a model loads when
  its package is missing. The PyPI bitsandbytes aarch64 wheel has no sm_87
  kernels ("named symbol not found at line 74 in ops.cu" on the Orin), so
  int4 is the Jetson's. The client accepts int4 in a server identity
  (`guardrail/vla_link.PRECISIONS`, 8f83f93).
- **`follow_vlm --vla-hierarchy-expiry-s`** (471f0f7, default 1.5): how long
  an answer's intent holds, for a model slower than 1.5 s per decision;
  recorded in the hierarchy's config.
- **Where a run really ran** (65240f5, 87cc412, de1238a): the manifest's
  topology is `projectairsim-remote-sim` when `--sim-address` is not
  loopback (follow_vlm, its hierarchy, vla_fly; `guardrail.manifest.
  projectairsim_topology`), and `deploy/orin/sync_follow_controller.ps1`
  now sends a git bundle and checks HEAD out on the Jetson (`-Ssh`/`-Scp`
  take Git for Windows' client). Every Jetson flight of 27-28 Oct recorded
  `projectairsim-single-host` and `unversioned`; the first flight after the
  fix recorded `8f83f9391adb` and `projectairsim-remote-sim`. Runs already
  on disk keep their labels.
- **An episode record and a CSP from every city flight** (f6ee433, audit
  WP2-11): `follow_vlm.city_episode` writes events.jsonl (episode_start to
  mission_end), policy_g0.json and csp_g0.json, compiled for the follow's
  mission at the Shield's own horizon, with no rule clock (the city Shield
  has none); every flight-log row carries the episode id, so the replay
  bundle binds its log and refuses an episode that never ended.
  `tests/test_city_episode.py` (new, in CI) 3/3.
- **Paraphraser** (fb2cb49, audit WP2-13 / WP4-10 / ARCH-23): a
  with/without/having phrase after the object and any noun after a place
  preposition are read as narrowings. A new held-out set,
  `heldout_2026_10_28`, 40 probes written by an independent author (an
  agent given only the 17 source texts and the probe format, with no access
  to the validator, its tests, the design doc or the earlier sets).

#### Measured
| | result |
|---|---|
| DroneCATS bf16 on the Orin Nano | does not fit: the 4.6 GB allocation fails even with nothing else of ours running |
| DroneCATS int4 on the Orin Nano, alone | loads in about 11 s, uses about 3.1 GB, one decision p50 9.57 s (prefill of 1779 prompt tokens + 30 new tokens); PC bf16 0.76 s |
| Same 40 frames, Jetson int4 against PC bf16 | the same action kind on 35/40, identical step text on 11/40, 30/40 steps within 2 bins, go point median 18.8 px apart |
| Why int4 is slow there (one 2048x6144 layer, 1779 rows) | int4 tinygemm 46.8 ms, bf16 9.9 ms, int8 weight-only 7.7 ms; one row: 0.45, 0.42, 1.94 ms |
| Paraphrase validator on heldout_2026_10_28 | meaning changes accepted 2/20, faithful refused 5/20, the same before and after this change |
| Paraphrase validator, dev and regression sets | heldout_2026_10_21b 4/12 -> 0/12 (dev now); stored 112/112, mutants 3979/3979, review 0/28, first held-out 0/30: unchanged |

#### Flown
| Flight | Within 30 m | VLA decisions | Who flew | P0 escapes |
|---|---|---|---|---|
| Jetson, everything on the board: OWL-ViT, the controller, the Shield and DroneCATS int4 (`orin_onboard_hier_dronecats_int4_20261028`) | 0.959 | 36 asked: 34 timed out after 10 s, 2 answered (8.8-9.6 s), no intent set on any tick | the controller on all 4192 ticks (`controller_fallback`, no answer) | 0 |
On that flight the Jetson peaked at 7278 of 7607 MB of RAM with 3026 MB in
swap; the detector ran at 2.04 Hz (2.18 Hz with the controller alone, 27 Oct).

#### Found
- **The 0.959 is the controller's, not DroneCATS's.** The model fits on the
  Orin Nano but does not decide: beside the detector the board swaps, its
  answers pass the 10 s decision timeout, and the hierarchy falls back on
  every tick. A VLA that runs fully on the Jetson and steers needs the AGX
  Orin (64 GB) the lab has received, or a smaller or faster model.
- **The 5 Oct audit's provenance finding was right**: Jetson runs could not
  name their code. Fixed above for every later run.
- The independent paraphrase set found two kinds of change the validator
  does not read (a limit narrowed to one component, "horizontal speed"; a
  time zone added to a schedule) and five faithful wordings it refuses
  (merged equal limits, "respectively", spelled-out numbers, "someone",
  "in front of you"). Recorded, not tuned on.

#### Moved
- WSL off D: (the user added a 1 TB disk, E:): Ubuntu by export and import,
  18.6 GB (its old disk image was 633 GB for 19 GB of files), every file's
  size and every /home, /root, /etc, /opt and /usr/local file's sha256
  checked; Ubuntu-22.04 by `wsl --manage --move`. The Ubuntu backup tar is
  kept on E:.
- `C:\vla_drone_offload` (73.6 GB, 444,881 files: 25 run frame folders,
  the Jetson runs, the full videos) copied to `D:\vla_drone_offload`,
  checked folder by folder (count and bytes, 0 differences), and the 25
  `demo\out\<tag>\view` junctions re-pointed to D:. The C: copy is kept
  until the user says otherwise.

#### Tests
- test_vla_foreign 49/49, test_city_episode 3/3 (new), test_paraphraser
  87/87, test_manifest 67/67, test_vla_link 35/35, test_vla_hierarchy
  17/17, test_vla_fly 63/63, test_build_dashboard 20/20, test_kpi_report
  40/40.

#### Limits
- One Jetson flight. The PC side of that flight ran a working tree with an
  untracked file, so PC runs still record `-dirty`.

### 2026-10-28 (later) — three no-fly zones flown by the best hierarchical VLA on the PC and on the Jetson: no zone entered; a first placement pinned the drone; progress deck

The user asked for the best hierarchical run on the PC (DroneCATS) and on the
Jetson (the original AerialVLA) to be flown again with more no-fly zones, as a
test of the guardrail, then a progress deck with proof of the Jetson runs and a
cheat-sheet in Indonesian and English.

#### Added
- `policies/follow_car_citylife_nfz3b.yaml` (6dc7216): nfz-frontage plus two
  zones whose controller band (1 m margin + FenceGuard's 3 m stand-off) covers
  the drone's usual line on legs 2 and 4, leaving a 5 m line with >= 3.91 m
  and >= 4.62 m building clearance (Shield.clearance_at). Signed bundle,
  locked, deployed to the Jetson.
- `tools/deck/build_progress_1028_data.py` and `build_progress_1028_deck.js`:
  every number on the 16-slide deck is read from the flights' metrics.json,
  kpi.json, flight_log.jsonl and the Jetson's console log; the build refuses
  a slide that would print undefined or NaN. Deck, PDF and the cheat-sheets
  (Indonesian and English) in `docs/share/2026-10-28-Progress/`; the English
  one also as `docs/PRESENTATION-CHEATSHEET-28Oct2026.md` (both gitignored,
  like every presentation cheat-sheet).
- `tests/test_build_progress_1028_data.py` (3 tests, in CI): the score's basis,
  which folders count, and the per-zone counts; each shown failing on an
  injected defect.

#### Flown (policy follow_car_citylife_nfz3b, within 30 m)
| Flight | Within 30 m | Closest to nfz-frontage / north-street / south-block | Ticks inside a zone | Controller avoided · Shield repaired | P0 escapes · collisions |
|---|---|---|---|---|---|
| PC, DroneCATS hierarchical (`nfz3b_pc_hier_dronecats`) | 0.929 | 4.78 / 4.33 / 4.47 m | 0 | 1393 · 13 | 0 · 0 |
| Jetson, original AerialVLA hierarchical (`nfz3b_orin_hier_aerovla`) | 0.454 | 4.99 / 4.21 / 27.83 m | 0 | 448 · 257 | 0 · 0 |
Without the two new zones the same models came within 2.40 / 2.45 m of where
nfz-north-street's edge now is and 3.56 / 3.54 m of nfz-south-block's.

#### Found
- **A zone's real keep-out is 4 m, not its 1 m margin.** The first placement
  (`policies/follow_car_citylife_nfz3.yaml`, 5621179, now marked superseded)
  counted the margin only; flown on the PC (`nfz3_pc_hier_dronecats`) the drone
  held 4.0 m short of nfz-north-street for 3349 ticks, never entered it, and
  lost the car (within 30 m 0.270). The rule held; the placement was wrong.
- **The Jetson flight's 0.454 is the corner, not the zones**: the car was lost
  at the north-east corner at 114 s (re-acquired at 354 s), where the same model
  also lost it on the PC without the new zones (`cmp2_hier_aerovla`, 0.309).
  nfz-south-block was never reached on that flight, so it says nothing about
  that zone.
- Over the 45 flights of 27-28 Oct (the 42 of the comparison and these three):
  0 collisions in a mission, P0 escape rate 0.0 in every kpi.json, 3793 ticks
  with a P0 violation detected in a pilot's command.

#### Limits
- One flight per platform; on the Jetson the VLA answered from the desktop's GPU.

### 2026-10-28 — every model end to end and hierarchical, on the PC and on the Jetson: no VLA beats the hand-written follow consistently; a fixed 12 m stand-off scores 0.999 on the PC and 0.799 on the Jetson

The user asked for every model flown end to end ("pure VLA") and at the top
of the hierarchy, on the PC and with the flight client on the Jetson Orin
Nano, with a video for each. 32 flights on 28 Oct (16 on the PC, 16 on the
Jetson) plus the 10 of 27 Oct: 42 flights, 0 collisions in any mission and a
P0 escape rate of 0.0 in every kpi.json. Same configuration as 27 Oct (Car_10,
`follow_car_citylife_nfz`, 420 s, offscreen simulator at 30 FPS); end-to-end
flights scored from the first acquisition, the others from the start gate.

#### Added
- `vla_fly --sim-address` (default 127.0.0.1, fe171ab) and
  `deploy/orin/run_vla_fly.sh`: the end-to-end follow runs on the Jetson
  against the desktop's simulator. The VLA server stays on the desktop's GPU
  (the 7B models do not fit the Orin Nano's 8 GB; DroneCATS needs
  transformers >= 5.18, the Jetson has 4.40.1) and is reached over an SSH
  reverse tunnel the desktop opens (9900ab5), not a firewall opening.

#### Flown (within 30 m; PC runs of 27 and 28 Oct, Jetson runs of 28 Oct)
| Pilot | PC end to end | PC hierarchical | Jetson end to end | Jetson hierarchical |
|---|---|---|---|---|
| Hand-written follow (no VLA) | - | 0.966, 0.954, 0.980 | - | 0.976 |
| Hand-written follow, fixed 12 m stand-off (scripted stand-in, no VLA) | - | 0.999 | - | 0.799 |
| AerialVLA original (aero_vla) | 0.539, 0.153 | 0.987, 0.309 | 0.102 | 0.989 |
| AerialVLA r1 | 0.018 | 0.884, 0.884 | 0.113 | 0.890 |
| AerialVLA r2 | 0.065 | 0.918 | 0.017 | 0.924 |
| AerialVLA r3 | 0.053 | 0.869, 0.934 | 0.076 | 0.876 |
| AerialVLA r4 | 0.033 | 0.873 | 0.077 | 0.882 |
| DroneCATS-2B | 0.132, 0.086 | 0.939, 0.929 | 0.076 | 0.969 |
| RaceVLA-7B | 0.099, 0.084 | 0.719, 0.721 | 0.083 | 0.704 |

#### Found
- **No VLA beats the hand-written follow consistently.** Its four flights
  scored 0.954-0.980. The steadiest VLA is DroneCATS at the top of the
  hierarchy (0.929, 0.939, 0.969); the original AerialVLA there scored 0.987,
  0.309 and 0.989.
- **A close stand-off is fast but brittle.** The scripted 12 m stand-off
  scored 0.999 on the PC and 0.799 on the Jetson (car out of view on 0.375 of
  the ticks, detector 1.88 Hz); the original AerialVLA, which always asks for
  about 12.4 m, lost the car at a corner in `cmp2_hier_aerovla` (0.309). The
  fine-tunes' 16-17 m stand-offs score 0.87-0.93 and never fell apart.
- **End to end every model loses the car** (0.017-0.539): once the car
  leaves the frame at a turn, nothing brings it back, and the score depends
  on when that happens (the original AerialVLA: 0.539 and 0.153 in two PC
  flights).
- **The Jetson flies the hand-written follow as well as the PC** (0.976)
  with OWL-ViT at about 2.2 Hz against about 3 Hz on the PC. On the Jetson
  the car was first locked within seconds of take-off instead of after about
  210 s on the PC (the simulator starts before the flight there), so its
  flights follow a different part of the car's route.

#### Limits
- One to three flights per configuration; the original AerialVLA and the
  fixed stand-off show how much one flight can differ from the next.
- On the Jetson only the flight client runs on the Orin; every VLA answered
  from the desktop's GPU.

#### Fixed (the runs, not the code)
- D: reached 0 bytes during the PC flights and killed one run (renamed
  `cmp2_e2e_r1_20261028_aborted`, re-flown); the 27 Oct frames were moved to
  `C:\vla_drone_offload` and new runs write there.

### 2026-10-27 (flown) — nine follows in one configuration: the original AerialVLA adapter at the top of the hierarchy 0.987, the hand-written follow 0.954 and 0.980, DroneCATS 0.939; the fine-tunes below the original

Every model we have, flown once each in the same configuration: the red car
Car_10, policy `follow_car_citylife_nfz`, 420 s, `-SimOffscreen -SimMaxFps 30`,
after a restart of the desktop (48 GB of commit free). The hierarchical and
hand-written flights are scored as follow_vlm scores them (from the start
gate's acquisition), the end-to-end ones from the first acquisition
(`after_acquire`), so all start counting when the car is first locked.

#### Flown (within 30 m; one flight each)
| Pilot | Within 30 m | Mean distance | Car out of view | Stand-off p50 | Decision latency p50 | Collisions |
|---|---|---|---|---|---|---|
| Hand-written follow (OWL-ViT + controller), `cmp_manual` | 0.954 | 21.1 m | 0.003 | - | - | 0 |
| Hand-written follow, as the hierarchy's fallback only (see Limits), `cmp_hier_fixed12` | 0.980 | 20.8 m | 0.006 | - | - | 0 |
| AerialVLA original (aero_vla), hierarchical, `cmp_hier_aerovla` | 0.987 | 20.5 m | 0.143 | 12.4 m | 0.49 s | 0 |
| DroneCATS-2B, hierarchical, `cmp_hier_dronecats` | 0.939 | 21.7 m | 0.009 | 16.4 m | 1.45 s | 0 |
| AerialVLA r1, hierarchical, `cmp_hier_r1` | 0.884 | 22.8 m | 0.026 | 16.4 m | 0.48 s | 0 |
| AerialVLA r3, hierarchical, `cmp_hier_r3` | 0.869 | 22.8 m | 0.044 | 16.5 m | 0.66 s | 0 |
| RaceVLA-7B, hierarchical, `cmp_hier_racevla` | 0.719 | 27.5 m | 0.033 | 21.6 m | 0.40 s | 0 |
| AerialVLA original, end-to-end, `cmp_e2e_aerovla` | 0.539 | - | - | - | 0.46 s | 0 |
| DroneCATS-2B, end-to-end, `cmp_e2e_dronecats` | 0.132 | - | - | - | 1.02 s | 0 |
| RaceVLA-7B, end-to-end, `cmp_e2e_racevla` | 0.099 | - | - | - | 0.42 s | 0 |

#### Found
- **In the hierarchy the VLA's effect is the stand-off it asks for.** In all
  five hierarchical flights the intent was FOLLOW on most ticks and the VLA
  never flew a search (`vla_flown_share` 0.0), so the controller flew every
  pursuit. The score follows the stand-off: 12.4 m gave 0.987, a steady
  16.4 m (DroneCATS, pinned by its 3 m cap) 0.939, about 16.4 m varying
  between 12 m and 22 m (r1, r3) 0.884 and 0.869, and 21.6 m (RaceVLA) 0.719.
  At 12.4 m the car left the frame on 0.143 of the ticks and still stayed
  within 30 m.
- **The original adapter follows better than our fine-tunes in this
  configuration.** End to end, aero_vla flew 0.86-0.96 of the ticks itself for
  about 100 s after the first acquisition, 10-14 m behind a car driving 2-3
  m/s, then lost it: 0.539, against 0.238 for the best fine-tune so far (r1).
  In the hierarchy it asked for the closest stand-off; r1 and r3 asked for
  farther and more variable ones.
- **The hand-written follow's own spread is about 0.03**: 0.954 and 0.980 in
  two flights of the same configuration. AerialVLA's 0.987 is inside that
  spread: equal to the hand-written follow, not shown better by one flight.
- **DroneCATS end to end** closed in, lost the car at a turn about 10 s after
  the first acquisition, answered `finished` twice (held, no landing), then
  turned left 90 deg at a time; its `think` answers took up to 9.3 s. After
  the car was lost the flight hovered, as every end-to-end follow does.
- **RaceVLA's forward commands stayed under 0.3 m/s**, so the car pulled
  away end to end and its hierarchy stand-off sat near the far end.
- **Bench (no simulator)**: DroneCATS p50 1.12 s at 256x256 and 1.00 s at the
  detector's size, its server 10.1 GB of commit; RaceVLA in nf4 0.33 s.

#### Limits
- One flight per pilot. The control flight meant to fly a scripted 12 m
  stand-off parsed none of its 31518 scripted answers (the runner passed the
  scripted text to Start-Process unquoted, which split it at its spaces), so
  the controller flew it alone as the fallback: it is a second hand-written
  flight, not the control. Whether a fixed 12 m stand-off alone gives
  AerialVLA's 0.987 is not measured yet.
- All pilots here are zero-shot or our earlier fine-tunes; none was trained on
  these flights.

### 2026-10-27 (Jetson) — the hand-written follow runs on the Jetson Orin Nano against the desktop's simulator: within 30 m on 0.957 of its ticks

The user paused the VLA training and asked for the hand-written follow
(OWL-ViT + the identity lock + the 10 Hz controller + the Shield) to run on
the Jetson Orin Nano, with Project AirSim staying on the desktop. It does,
over a direct 1 Gb/s cable, and its second flight scores as the desktop does.

#### Added
- `follow_vlm --sim-address` (default 127.0.0.1, commit 77c2245): the
  simulator on another machine (topics 8989, services 8990).
- `deploy/orin/` (7f9df26, 63f9072): `sync_follow_controller.ps1` sends HEAD
  (git archive, so the gitignored private signing key stays), the follow's
  signed bundles and the whole CityLife map folder over SSH;
  `start_sim_for_jetson.ps1` starts only the simulator and prints the address
  it listens on; `run_follow_controller.sh` runs follow_vlm with
  run_citylife_follow.ps1's car-mode arguments after checking the simulator's
  ports and the map files.
- `make_demo_video.py --layout pip` (63f9072): the drone's view at full size,
  the chase view small bottom-left - the user's layout for test videos.
- The hierarchical follow (`follow_vlm --vla-hierarchy`, merge 1e4b144, the
  user's decision of 25 Oct): AerialVLA sets the mode, the stand-off (22 m at
  fwd 0 to 12 m at fwd 5) and flies the search; the controller flies the
  pursuit. Not flown: both attempts ran out of Windows commit memory.

#### Set up (the Jetson)
- Orin Nano Super developer kit, JetPack 6.2.3 (L4T R36.5), Python 3.10.12,
  CUDA 12.6; torch 2.8.0 with CUDA from NVIDIA's Jetson index (no 2.6 build
  exists for JetPack 6), the desktop's pins otherwise (transformers 4.40.1,
  numpy 1.26.4), the Project AirSim client at the desktop's commit fa404d0.
  OWL-ViT forward on the Jetson: 192 ms fp32, 58 ms fp16.
- Cable: desktop 10.42.0.1, Jetson 10.42.0.2 (192.168.50.x would have
  collided with the Jetson's Wi-Fi), 909 Mbit/s and under 1 ms measured;
  UnrealEditor's existing inbound rule lets the Jetson reach 8989-8990.

#### Flown (the controller on the Jetson, within 30 m from the start gate)
- `orin_follow_01` (25 W): 1.0 - but with occ_day.npz as the only map file,
  so ObstacleClearance was inert, with no street mask, junction planner or
  landing site, and the detector at 1.87 Hz, which follow_vlm flags as
  unrepresentative. Kept, labelled degraded.
- `orin_follow_02` (MAXN_SUPER, the whole map folder): 0.957, detector
  2.18 Hz, P0 escape rate 0.0, mission success; no contact in the mission (one
  before the start gate and one in the landing descent). Desktop reference
  citylife_redcar_nfz_motion2_20261019: 0.963. Data and the comparison index
  in D:\vla_drone_offload\jetson_runs.

#### Found
- **The 7B server's commit is the GPU's backing, not RAM in use**: after
  loading, 17.3 GB private against 1.4 GB working set with 14.1 GB on the GPU
  (WDDM on a GeForce card). With the desktop's other programs (41.2 GB commit
  without the server) a flight with the server, Unreal and OWL-ViT passed the
  70.7-74.3 GB commit limit: the hierarchical flights failed with "paging
  file is too small" and a CUDA launch error.

### 2026-10-27 — two published drone VLAs, DroneCATS and RaceVLA, can fly the same follow behind the Shield; not yet flown

To compare our fine-tuned AerialVLA with drone VLAs that other groups have
published, two of them now sit behind the same protocol v2 server interface
(merge 62a65d4). Nothing in this entry has run a real model or the simulator
yet: the weights are downloaded, the conversion of each model's output is
written from its own sources and tested with scripted outputs.

#### Added
- **`demo/vla_server_foreign.py`**: DroneCATS-Cosmos3-Edge-2B (NAVER, OpenMDW-1.1,
  transformers >= 5.18, the separate `dronecats` env) and RaceVLA drone6_all
  (a merged OpenVLA-7B, the `vla-real` env) behind protocol v2. Each gets its
  own input (DroneCATS: its prompt files verbatim, `camera_aux` intrinsics,
  the last 5 actions as history; RaceVLA: its OpenVLA prompt, a grey frame as
  its T265's) and a documented conversion of its output into the AerialVLA
  step text: DroneCATS's `go(point, depth)` through the camera geometry and the
  paper's executor (altitude held under 0.15 of the horizontal, 3 m cap,
  x0.6 under 5 m), `rotate`, `think` (hold) and `finished` (LAND); RaceVLA's
  velocities through its robot's own scales (forward x1.875, climb x0.5, yaw
  x1.5 rad/s) over a 1 s horizon, its lateral velocity dropped and recorded.
  Every reply, server log line and client record keeps the model's own
  output (`native`) and the conversion applied (`mapping`).
- **Recorded as themselves**: the two families are allow-listed
  (`aerialvla_codec.FOREIGN_FAMILIES`), a flight records
  `dronecats:DroneCATS-Cosmos3-Edge-2B` or `racevla:drone6_all`, never
  AerialVLA's name, and the dashboard counts it as a VLA flight of that
  family. The weights' sha256 is in hello; the scripted stand-in
  (`fake:<family>-scripted`) is never counted as a VLA.
- **The front image**: `vla_fly --vla-front-source detector` and
  `follow_vlm --vla-hierarchy-front-source detector` (launcher
  `-VlaHierarchyFrontSource`) send the detector's FrontCamera frame
  (768x432, 90 deg, pitched -20 deg) to a server that reads it; the default
  stays VLAFront. A flight is refused before take-off when the camera sent is
  not the server's, and so is a foreign model with `prev_front` or the
  own-speed words, which its prompt has no place for.
- **`tools/bench_foreign_vla.py`**: latency p50/p95 and a histogram of the
  model's outputs and the resulting steps; `tests/test_vla_foreign.py`
  (48 tests, in CI).

#### Limits
- RaceVLA's whole action range gives at most 0.52 m/s forward, so it cannot
  keep up with the car end to end; DroneCATS's 3 m cap keeps its hierarchy
  stand-off at 16 m or more. Both are zero-shot here: neither was trained on
  CityLife or on following a car.
- Not available as weights: CosFly-VLA and CognitiveDrone.

#### Tests
- `tests/test_vla_foreign.py` 48/48; `test_vla_fly.py` 62/62;
  `test_vla_hierarchy.py` 17/17; `test_harnesstruth_ci.py` 35/35.

### 2026-10-25 — r3 and r4 flown fast, the follow re-acquires a car that comes back, the own-speed words copy themselves; the user chooses hierarchical control

Two more models flown in the fast simulator configuration, one perception
bug fixed, and one input idea measured and rejected. The best end-to-end
VLA follow is still r1's 0.238; the user chose to put AerialVLA at the top
of a hierarchy over the hand-written controller next.

#### Flown (from the first acquisition, within 30 m; -SimOffscreen -SimMaxFps 30)
- **follow_v4_r3/epoch1** (r3: follow_v4_pf_hm, speed changes x3, from r2;
  held-out forward step error 0.221 m against r2's 0.387 on the same set):
  `vla_follow_ft_07_r3_off` 0.085 - it closed from 54 m to 21.5 m at about
  4.7 m/s, slowed and turned too little at a corner and lost the car, then
  never took it back (see Fixed).
- **follow_v5_r4/epoch1** (r4: the own-speed words, from r3; held-out 0.214 m):
  `vla_follow_ft_08_r4` 0.044 - four re-acquisitions, but with the aircraft
  hovering the prompt read "while hovering" and the model answered about
  0.4 m forward while the car pulled away from 27 m to 69 m.

#### Found
- **The own-speed words teach "keep doing what you are doing".** 85 % of the
  "while hovering" samples in follow_v5_pf_hm_sw are labelled as staying, and
  in flight the words held the hover (the copycat problem of behaviour
  cloning with proprioception). The words stay in the code, off by default
  (merge 834288b).

#### Fixed
- **A car that comes back after a long loss is taken back** (merge 6d37457):
  in `vla_follow_ft_07_r3_off` the detector found the car again at about
  288 s (identity "ok", 9-10 m, score up to 0.67) but the start gate's "seen
  moving" test needs 10 sightings in a row and one empty frame at 2.2 Hz
  reset the streak, without recording a refusal. `vla_fly --acquire-gap-s`
  (default 3 s) lets the streak survive short misses (an opt-in
  `follow_vlm.Reacquirer(gap_s=)`, unchanged for follow_vlm), and every
  judged frame's outcome is counted; replayed on that flight's detections it
  re-commits at 287.1 s.

#### Decided
- **Hierarchical control next** (the user, 25 Oct): AerialVLA at about 2 Hz
  sets the intent - mode, the following distance, and the search flight -
  and the hand-written 10 Hz controller flies the pursuit.

### 2026-10-24 (later) — the VLA's decision rate, not its training, is what keeps it behind: ceiling flights, the renderer's share of the GPU, faster VLA flights

The fine-tuned follows kept losing the car, so the question was asked
directly: how well does the hand-written follow itself do when it must act
at the VLA's rate? Badly - and the slowness was mostly the simulator's
renderer sharing the GPU. Still not as good as the hand-written follow; the
third model is training.

#### Found
- **Ceiling flights** (`follow_vlm --decide-every-s P --decide-delay-s D`,
  merge a92f338: the expert's own command, D s old, held for P s): within 30 m
  on 0.122 of the ticks at P 1.0 / D 0.8 (the VLA's rate at the time; it
  overshot onto the car, minimum 0.5 m, and saw it on 16 % of the ticks),
  0.908 at 0.5 / 0.4, 0.956 at 0.3 / 0.3, against 0.963 at full rate. The VLA
  flights (0.158-0.238 from the first acquisition) were no worse than the
  expert held to their rate.
- **The renderer, not the model, made the VLA slow.** Alone the merged bf16
  server answers in 235 ms (generation 0.233 s, 9 tokens). Pinged through an
  expert flight it took 625-713 ms from the moment the simulator moved the
  camera; with the aircraft hovering in four simulator configurations it
  stayed at 230-242 ms. OWL-ViT is not the cause: alone it takes 30.5 ms
  (fp32), and the server stays at 248 ms beside it.
  `-SimOffscreen` (`-RenderOffscreen`) with `-SimMaxFps 30` (`t.MaxFPS`)
  brought the chase-phase latency to 370-397 ms with the expert's own score
  0.945 (commit 0a352b5; both switches default off).

#### Flown (from the first acquisition, within 30 m)
- **follow_v3_pf_r1/epoch1** (r2, the prev_front tile; its training stopped at
  epoch 2 step 1320 when the session ended - held-out epoch 1: forward step
  error 0.340 m, turn error 0.0173 rad): `vla_follow_ft_03_pf` 0.173 (it
  overshot a stopped car, which left the bottom of the frame, then flew on
  with no car in view), `vla_follow_ft_04_pf_fps30` 0.148,
  `vla_follow_ft_05_pf_off` 0.125 (capture-to-answer p50 0.43 s, decisions
  every 0.4-0.5 s: it answered forward 0.0 for about 7 s while the car pulled
  away from 8 m).
- **follow_v2_r1/epoch4** (r1) offscreen: `vla_follow_ft_06_r1_off` 0.169
  (latency p50 0.46 s); it chased for about 190 s with three re-acquisitions
  but mostly 30-40 m behind.

#### Added
- **Memory and search samples** (merge 564901c): one hint rule,
  `vla_mission.follow_hint_step`, for the flight's DetectorHint and
  `build_follow_dataset.py --hint-modes fresh,memory,search`, plus the
  flight's close rule (`--close-bottom-frac 1.0`). They add few samples: the
  expert rarely loses the car, and no capture shows the car behind the
  aircraft.
- **Five DAgger flights with r2** (ft_dagger_pf_1-5): within 30 m on 0.844,
  0.807, 0.808, 0.788 and 0.813; 52, 54, 53, 52 and 53 windows.
- **Dataset follow_v4_pf_hm** (19 recordings): 18570 train (9285 real) and
  1641 held-out samples (ft_expert_p03, ft_dagger_03, ft_dagger_pf_3); 309
  memory and 22 search samples.
- **Speed-change oversampling** (`--change-oversample N`, 0a352b5): 2370 of
  the 18570 train samples change the forward step by 1 m or more within 1 s.
  Training follow_v4_r3 (from r2, 3 epochs, changes x3, turns x2) is in
  progress.
- Videos of every flight above in `docs/video/` (`*_small.mp4`).

### 2026-10-24 — AerialVLA fine-tuned on the hand-written follow: held-out step error 2.47 m to 0.36 m, the first fine-tuned flights still lose the car, DAgger and a temporal tile next

The user's goal: the VLA-piloted follow as good as, or better than, the
hand-written follow. AerialVLA was fine-tuned on the expert's own flights
(the 23 Oct FT-D / FT-T / DART tools, merged 5269143 and ce1a61c), flown,
and the two failures it showed were turned into the next round's data and
inputs. Not yet as good as the hand-written follow; the second model is
training.

#### Added
- **Expert recordings for the fine-tune** (`follow_vlm --record-vla`, with
  `--record-perturb` DART on the p runs): ft_expert_01, 02b, 03 and p01-p06,
  within 30 m on 0.953, 0.948, 0.939, 0.972, 0.934, 0.969, 0.928, 0.985 and
  0.95 of their ticks. p06 kept 339 of its frames: C: filled during the
  batch (5096 write errors; frames now go to D:, commit 40e079e).
- **Dataset follow_v2**: 5706 train samples (2853 real, 2853 mirrored) and
  345 held-out (ft_expert_p03), labels the expert's motion over 1.0 s.
- **Fine-tune follow_v2_r1** (bf16 LoRA from upstream's aero_vla, 4 epochs,
  116.5 min). Held-out (345 samples, never trained on), original adapter
  then epochs 1-4: forward step error 2.467, 0.432, 0.377, 0.396, 0.359 m;
  turn error 0.103, 0.056, 0.053, 0.050, 0.053 rad; turn sign agreeing
  0.699, 0.791, 0.817, 0.826, 0.835. Epoch 4 merged in bf16 (the flying
  configuration): 0.360 m, 0.0526 rad, 0.838 - merging costs nothing here.
- **DAgger recording** (merge c9dc0a1): `follow_vlm --record-vla
  --record-perturb-vla HOST:PORT` (`-RecordPerturbVla`) lets the fine-tuned
  AerialVLA fly seeded 2.5-4.0 s windows every 6-10 s of the expert's
  recording, through vla_fly's own follow tracker, after the controller and
  before the Shield; the expert's recovery is the label (the DART rule). A
  window with no fresh step is flown by the expert and says so.
  `StepTracker.note_flown` starts the tracker's smoothing from the command
  in force, so the hand-over does not jump.
- **Five DAgger flights** with follow_v2_r1/epoch4 (ft_dagger_01-05): 52,
  53, 52, 50 and 51 windows; within 30 m on 0.803, 0.879, 0.799, 0.811 and
  0.846 of their ticks; capture-to-answer p50 0.78-0.83 s. Against
  follow_v2, ft_dagger_01's labels sit farther from the car (median 23.9 m
  against 20.4 m) and twice as often at full speed (fwd >= 4.5 m on 0.15
  against 0.08): the states the VLA creates and the expert's catch-up.
- **The front frame 1 s earlier as the lower tile** (merge ee4762d, off by
  default): `--bottom-tile prev_front` (builder) and `--vla-bottom-tile
  prev_front` (flight) put the front frame nearest t - 1.0 s (within
  0.25 s) where the down camera was; no such frame is no sample and no
  request, never a fallback. Checkpoints record the tile and label horizon
  they learned; the server announces both and vla_fly / the DAgger recorder
  refuse to fly a mismatch.
- **Rate execution** (`vla_fly --step-as-rate H`, off by default): a step
  flies at fwd / H and dpsi / H until the next answer replaces it - what a
  1.0 s label means - instead of flying to an endpoint at cruise and braking.
- **Dataset follow_v3_pf**: the 9 expert and 5 DAgger recordings with the
  prev_front tile, stride 0.25 s: 13762 train samples (6881 real, 6881
  mirrored) and 1076 held-out (ft_expert_p03, ft_dagger_03); measured tile
  dt p50 1.0082 s. Training follow_v3_pf_r1 on it is in progress.
- A follow's score from its first acquisition (`metrics.follow.after_acquire`,
  top-level `frac_within_30m_after_acquire`; commit 3cd51d3), see Found.

#### Flown
- **vla_follow_ft_01_20261024** (follow_v2_r1/epoch4, merged bf16, 8 m - the
  expert's altitude; the dataset's median is 8.06 m and vla_fly's default
  10 m): within 30 m on 0.121 of the flight, 0.158 from the first acquisition
  (229.3 s). The car stopped at a light about 9 m ahead and the model held,
  as the expert does; the car drove off and the model answered forward 0.0
  for about 6 s while the car went from 9.8 m to 27.6 m - one frame cannot
  tell a stopped car from one pulling away. It then matched the car's speed
  about 30 m behind and lost it.
- **vla_follow_ft_02_rate_20261024** (as ft_01 with `--step-as-rate 1.0` and
  `--acquire-max-range-m 60`): 0.196 of the flight, 0.238 from the first
  acquisition. It closed from 33.0 m to 5.5 m behind a stopping car, lagged
  again when the car pulled away, and lost the car at a corner where it had
  turned toward the car and the Shield's 3 m building clearance held it at
  0.5 m/s. P0 escape rate 0.0 in both.

#### Found
- **The VLA-to-hand-written comparison was on unequal bases.** follow_vlm
  starts its clock and score at its start gate: the reference
  citylife_redcar_nfz_motion2_20261019 (0.963) waited 196.1 s and scored
  the 2400 rows after it. vla_fly scored from take-off, and in every flight
  the level car spends about 200 s on the far side of its loop first. From
  the first acquisition: ft_01 0.158, ft_02_rate 0.238, follow #10 0.235
  (0.17 of its whole flight). Still far from 0.963.

#### Fixed
- The trainer refused the builder's whole first dataset (6051 of 6051 lines
  "no_target": the builder writes the answer as action_text); a test now
  feeds the builder's rows to the trainer's loader (f8476a3, 1762e9f).
- tests/test_track_truth.py knows a tie at the frame centre: the clean
  expert recordings tie the constant-centre null (3.4 px against 3.4, 3.0
  against 3.0, 3.3 against 3.3) while their perturbed twins win, 27.0-30.5 px
  against 37.6-39.3 (896b50d); the tie may be within 5 % + 0.1 px of the
  null (ft_dagger_01: 5.5 px against 5.3, its four siblings win).

#### Tests
- `tests/test_vla_fly.py` 62/62, `tests/test_vla_dagger.py` 17/17,
  `tests/test_vla_prev_front_rate.py` 21/24 (3 need Pillow / torch),
  `tests/test_follow_dataset.py` 35/39 (39/39 with Pillow),
  `tests/test_track_truth.py` 36/36.
- The CI list, 120 files: 2 lab-only files fail, both stale decks
  (`tests/test_explainer_deck.py`: the 14 Oct pack cites a shield.py
  sentence rewritten on 21 Oct; `tests/test_build_progress_oct_data.py`:
  the ITRI pack was not rebuilt after its data changed on 22 Oct). Left for
  a separate change: the PI chooses which deck is sent.

### 2026-10-23 (follow, reviewed) — the follow's score now counts, a lost car is re-acquired at once, follow #10

An independent review of the follow work (no P0; truth never reaches control)
found the follow's records overstating, and a 26 s blackout after a lapse.
Fixed (merge d9dbbd8), then follow #10 flown.

#### Retracted
- **Every VLA follow's kpi.json said mission_success true**: the follow score
  sat under `metrics.follow`, where guardrail/kpi.py does not look, so no follow
  test applied. Re-scored from each run's own flight log with
  `tools/rescore_vla_follow.py`: #2 and #3 (0.0 within 30 m) now fail
  `follow_proxy`; the other seven pass kpi's 5 % rule. The within-30-m shares
  quoted in the section below were computed from the flight logs and stand.

#### Fixed
- **A lost car is re-acquired at once**: with identity on, the strict lock
  lapsed after 2 s and nothing re-acquired for memory + search (26 s); now the
  lapse, or 3 s unseen, restarts follow_vlm's Reacquirer anchored at the last
  sighting (in the test with the real lock and Reacquirer: back 0.6 s after the
  car reappears, never within 20 s before).
- The follow's records: each decision's hint kind (fresh / memory / search);
  a LAND held as arrival is no longer a VLA landing; a follow that never asked
  ends `subject_not_acquired`, not as a VLA failure; the start gate is the one
  applied after the identity setup; perception resets drop the stale box.

#### Flown
- **#10** (`demo/out/vla_follow_redcar_10_20261023`, as #9 with the fixes):
  within 30 m on 0.926 of the ticks AerialVLA flew (median 12.78 m, the box on
  the car 133 of 151), the best chase yet - but 0.17 of the whole flight: the
  car started 41.77 m ahead, the drone waited for it to come round, chased it,
  and lost it when the car outran it before the corner; the re-acquisition
  range (30 m) never took it back.

#### Tests
- `tests/test_vla_fly.py` 60/61, the CI list 115 files, 0 failing.

### 2026-10-23 (follow) — AerialVLA chases the red car: nine follow flights, each fix measured on the next

The user asked for AerialVLA to follow the target as the hand-written
controller does. AerialVLA still decides every step; what changed is which box
is the car (perception) and how a step is executed, all for `--mission follow`
only (commit d6e7123). CityLife simple_flight, Car_10 as truth for scoring
only, not KPI-grade. Within 30 m, whole flight / while the VLA flew:

| # | what changed | within 30 m | while flying | closest | box on the car |
|---|---|---|---|---|---|
| 1 | first follow (3 m/s goto policy) | 0.348 | 0.506 | 0.49 m | 586 of 814 |
| 2 | fresh scene (the car started 35 m ahead) | 0.0 | 0.0 | 35.29 m | 374 of 1412 |
| 3 | the city follows' 5 m/s policy, a lock, memory and search | 0.0 | 0.0 | 41.76 m | 517 of 2384 |
| 4 | a start gate, steps anchored where the answer arrives | 0.252 | 0.195 | 0.93 m | 968 of 3834 |
| 5 | a reset when the Shield pins the aircraft | 0.315 | 1.0 | 0.08 m | 174 of 194 |
| 6 | the model's LAND held as arrival | 0.216 | 0.254 | 0.12 m | 182 of 1061 |
| 7 | identity check on (its lock never seeded: no request) | 0.139 | - | 2.74 m | none |
| 8 | identity acquisition (follow_vlm's Reacquirer) | 0.199 | 0.646 | 0.1 m | 344 of 451 |
| 9 | as #8, bf16 merged model (latency p50 0.7914 s) | 0.308 | 0.552 | 0.07 m | 473 of 763 |

The hand-written controller's 19 Oct flight: 0.963.

#### Flown
- **#9, the best**: two chases - up the street and, for the first time,
  round the right-hand corner and on along the next street; within 30 m on
  0.552 of the ticks AerialVLA flew, speed while chasing p90 3.46 m/s. Between
  the chases the drone hovered until the car came round its loop again, which
  is most of the whole-flight shortfall.
- **#5**: the closest chase - within 30 m on every tick the VLA flew, down to
  0.08 m above the car, until AerialVLA answered LAND (its training: target
  found, land), which ended that flight; #6 onwards hold instead.
- Every follow: P0 escape 0.0 and 0.0 s in the no-fly zone; #4 was pinned 225 s
  against a building by a wrong box (2510 Shield interventions).

#### Added
- Perception for the follow: follow_vlm's TargetLock and identity check (the
  VLA detector camera gains follow_vlm's depth stream), follow_vlm's
  Reacquirer as the start gate and the re-acquisition after a lapse; the box
  matched the car on 0.17-0.26 of the ticks with a box without them (#2, #3,
  #4, #6) and 0.62-0.76 with them (#8, #9).
- The follow's hint: the last sighting's bearing for 6 s after the box is
  lost, then a quarter-turn phrase toward its side for 20 s; a close box asks
  for no new step and holds the horizontal motion; the Shield pinning the
  aircraft resets the lock.
- The follow's executor (TrackerConfig, all default off): turn while moving,
  the altitude held (AerialVLA's 2-4 m descents), steps anchored where the
  answer arrives (#3 flew 1.54 m/s at a 4.5 m/s cruise), LAND as arrival.

#### Found
- The remaining limit is AerialVLA itself: steps of at most 5 m at about one
  answer per 0.8-1.3 s leave it below the car's 3.4-4 m/s on a straight, and
  it was never trained to pursue; when the car leaves the view at a corner,
  nothing but the car's own loop brings it back.

#### Tests
- `tests/test_vla_fly.py` 45/45, `tests/test_vla_link.py` 33/33; the CI list
  115 files, 0 failing.

#### Not done
- Training AerialVLA on following (expert demonstrations from the
  hand-written controller with step labels, matched to the model's decision
  latency, as the July campaign learned) is the next step and has not
  started. The merged model (#9) agrees with the upstream-precision reference
  on 88 % of the probe, not 96 %.

### 2026-10-23 (flown) — AerialVLA flies the drone: a goal reached and landed by the model, and the red car chased then lost

The first two flights piloted by AerialVLA (OpenVLA-7B + the original
adapter, bf16, on the RTX 4090) behind the Shield, on the CityLife
simple_flight rail with the signed `vla-goto-citylife` bundle. Not KPI-grade
(the simple_flight rail is none of the grant's topologies). The memory that
blocked them was freed by the user.

#### Flown
- **M1 GOTO** (`demo/out/vla_goto_01_20261023`, stop-and-go cadence): from
  the mission start (42, -18) AerialVLA flew 58 m up the street on the
  goal-derived phrase and answered LAND itself, with the empty phrase ("you
  have arrived"), 0.85 m from the goal (43, 40); closest approach 0.23 m,
  reached_goal true. 14 decisions, capture-to-answer latency p50 1.0125 s;
  0.9895 of the mission ticks authored by the VLA; 0 Shield interventions; 0
  collisions in the mission. The Dashboard now counts 1 VLA flight of 100.
- **M3 follow, the red car** (`demo/out/vla_follow_redcar_01_20261023`,
  240 s, pipelined cadence at 2.5 m/s, labelled): OWL-ViT's box gives only
  the direction phrase, AerialVLA flies, the level's Car_10 (the earlier city
  follows' red car) is truth for scoring only. The drone waited about 45 s for
  the car to come round its loop, then chased it 111 m up the street and came
  within 0.66 m (horizontally, flying over it) at 70 s; the car then pulled
  away at about 3 m/s against the drone's 1.8 m/s and turned off the street,
  the box was lost and the mission ended target_lost. Within 30 m of the car
  for 0.348 of the flight (the hand-written controller's 19 Oct flight:
  0.963); median distance 58 m. The box pointed within 10 deg of the car's
  bearing on 0.72 of the ticks with a box (150 of 151 at 40-60 s, 200 of 200
  at 80-100 s, 99 of 251 after the turn); at times it sat on something in line
  with the car, such as a road arrow. Shield: P0 escape 0.0, 0.0 s in the
  no-fly zone, 177 corrected ticks (altitude band 111 - AerialVLA kept asking
  for 2-4 m descents - no-fly-zone frontage 64, building clearance 26).

#### Added
- **`demo/vla_fly.py --mission follow`**: M2's detector phrase without M2's
  stop on a large box; ends on the time limit, a lost subject (90 s default)
  or a VLA failure; `--level-car` (default Car_10) scored as `subject_dist_m`
  per row and `metrics.follow` (the same within-30-m share as follow_vlm).
  OWL-ViT now loads beside the warm-up and the take-off.

#### Tests
- The CI list runs 115 test files, 0 failing.

#### Not done
- Following a moving car is far weaker than the hand-written controller:
  the policy's 3 m/s speed cap is at the car's speed, a rotation step stops
  translation, and nothing re-acquires the car after a turn. No Shield-off
  control arm was flown, so these flights show the Shield acting, not its
  margin over none.

### 2026-10-23 (later) — AerialVLA in the pilot slot of the ArduPilot rail and of the ROS 2 rail; none flown

Wave V2, branches b and c, each reviewed, fixed and re-reviewed (the ROS 2
branch twice), merged 37292d1 and d0eb3be. Nothing here has run in Unreal,
SITL, ROS or with the model; the ROS 2 rail has no camera publisher yet.

#### Added
- **`perception_node.py --pilot vla`** (Project AirSim + ArduPilot rail): the
  step tracker's action goes through the same Shield and escalation FSM as the
  detector pilot; FSM holds and stale-link spans suspend the tracker and pause
  the mission's stop rules; the model's LAND is landed by the node's own
  mission-end LAND. GOTO (`--goal`, scored on reaching it) and APPROACH (the
  OWL-ViT box's phrase, scored on the subject's true range). The warm-up runs
  disarmed; a refusal is recorded and exits 4. Stop-and-go only, unless
  pipelined is explicitly allowed (this rail's captures are timed by arrival).
  The default `--pilot servo` is unchanged.
- The rail's robot config carries AerialVLA's VLAFront / VLADown cameras; they
  render only in a VLA run (or with `-VlaCameras`), and the run records which.
- **`sitl/ros2_vla_bridge_node.py`**: AerialVLA in the ROS 2 / MAVROS 2 rail's
  VLA slot (`VLA=bridge` / `vla:=bridge`), publishing `/vla/action_4d` at 10 Hz
  with the decision's age, seq, author and LAND/END flags; the stub stays the
  default and its 4-value stream is read as before.

#### Changed
- **The ROS 2 Shield node refuses any VLA message that breaks the contract and
  holds the aircraft**: a NaN anywhere, a partial tail, a missing or wrong
  layout label, a seq going backwards, motion no decision authored, or 4 values
  after the bridge's 8. A sound bridge's stall is left to the receipt watchdog
  (LOITER, then GUIDED when messages return); a broken pilot ends the mission
  through the mission-end LAND after one 10 s hold or 3 holds in 30 s, never
  over the FSM's own Loiter or fail-safe. A lasting second publisher on
  `/vla/action_4d` makes the run not KPI-grade.
- `--vla-ready-timeout-s` (3300 s, added by the launch file for the bridge)
  arms only after the bridge's warm-up passes and its layout and decision
  timeout match the Shield's; arming then gets its full budget after the wait.

#### Fixed
- **The warm-up gate on both rails, the merged city rail included**: each
  request now carries the answer timeout it is recorded with (the cold first
  one gets 60 s, where the socket used to cut it at 10 s) and waits for a ready
  client after a late answer or a backoff. Before, a cold first inference over
  10 s, or one refused first request, refused a healthy server.

#### Tests
- The CI list runs 115 test files on the merged tree, 0 failing.

#### Not done
- No flight on either rail. The ROS 2 rail needs a camera publisher; the Orin
  entrypoint and compose do not yet select the bridge or serve AerialVLA.
  Small record gaps from the last reviews remain (an approach refusal's
  start-gate record, a reused scene's camera load, the Dashboard row reading
  quiet during a long warm-up).

### 2026-10-23 — the CityLife VLA flight is ready behind the Shield, and AerialVLA re-measured at its own precision; not yet flown

Wave V2, branch a: built, reviewed twice, fixed twice, merged (988c447). The
first AerialVLA-piloted flight has **not** flown: with the bf16 server loaded
the machine was at 99.8 of 104 GB committed memory, too little to start the
simulator safely, so the server was stopped and the flight waits for memory.

#### Added
- **`demo/vla_fly.py`: AerialVLA flies CityLife behind the Shield.** M1 GOTO
  (the direction phrase from the goal's bearing at each capture; the empty
  phrase within 3 m) and M2 APPROACH (the phrase from OWL-ViT's box through
  `vla_bridge`, rotated by the turn since the detector's frame). Every action
  goes through `Shield.filter`; `--shield off` is a labelled control arm
  clamped only to a recorded 2-40 m simulator band. LAND ends the mission in
  the script's own landing (never a mode request from the model).
- **Upstream's stop-and-go cadence is the default** (`-Cadence pipelined` is a
  labelled arm). On the point-vehicle probe, pipelining a constant answer for
  30 s climbed +1.77 m against +0.51 m and drifted -54.6 deg against -12.9 deg.
- **A warm-up gate before arming**: three inferences on live camera pairs, the
  last judged against 5 s; a slow or silent model refuses the flight (exit 4).
  A slow model is told apart from a dead one (`server_slow` / `server_dead`),
  and no request is stacked behind a timed-out one.
- **Every way out lands and is recorded**: an exception, a missed start, an
  overshoot or Ctrl+C (exit 130, marked interrupted); a server the flight
  started is always stopped; a records fault still leaves `metrics.json`.
- **`tools/vla_replay_gate.py`**: the flight's own model on at least 10 recorded
  frames, all seven phrases, descents judged; exit 3 when the server is not
  the flight's model, 4 for a stand-in (never PASS), 2 when it could not run.
- `policies/vla_goto_citylife.yaml` (6-14 m band, 3 m clearance, the city
  no-fly zone) and its signed bundle; VLA cameras at 256x256, 90 deg, pitch 0
  and -90, as upstream; `scripts/run_vla_fly.ps1`.

#### Measured (offline, RTX 4090)
- **AerialVLA's direction phrase in bf16** (`docs/data/aerialvla_bf16_phrase_probe_20261023.json`,
  section "Addendum, 2026-10-23" of `docs/FINDING-what-drives-aerialvla.md`):
  the mean turn is ordered by the phrase (-0.651 to +0.288 rad), but facing
  the target every phrase gives about 4.9 m forward and no turn; the phrase
  steers only when the target is off to one side. The 4 Aug effect was
  measured on a 4-bit loader. 36 requests, every answer recorded.
- Idle latency p50 391 ms per decision; about 15.8 GB of VRAM. A bf16 cold
  load took 26 min with the machine short of memory.

#### Fixed
- `tools/check_claims.py` registers the two 22 Oct retractions; the 22 Oct log
  had failed `test_every_changelog_retraction_is_registered`.
- A `fake:` model id never counts as a VLA, whatever digest it reports.

#### Tests
- The CI list runs 113 test files, 0 failing. `tests/test_vla_fly.py` 35/35,
  `tests/test_vla_replay_gate.py` 8/8; `tests/test_vla_fly_fakesim.py` 10/10 in
  the lab environment (it skips without OpenCV, as in CI).

#### Not done
- No VLA flight yet (blocked by memory, above). Open from the second review:
  the abort landing caps `land_async` at 60 s with no descent first (the
  control arm can be up to 40 m high); an aborted run's decision rate includes
  the time after landing; the replay gate can wait a full timeout per request
  on a slow server and checks its frame count only after the server is up.

### 2026-10-22 — AerialVLA becomes a pilot the Guardrail can fly, the tracker becomes the progress monitor, and open WP items closed

Wave V1: five branches, each reviewed adversarially two or three times and
fixed after each review, merged on 21-22 Oct (2aa03b1 for V1a-V1c, 01f7820
for V1d, fb8f87c for V1e). **Nothing in this wave has flown
a VLA yet**: every AerialVLA number below is offline (the model on the RTX
4090 against recorded frames), and the rail changes are tested headless, not
flown. The goal it serves is the user's of 21 Oct: a VLA that flies the
drone, not a hand-written controller with a detector.

#### Added
- **AerialVLA codec** (`guardrail/aerialvla_codec.py`): the upstream prompt
  byte for byte, the 8-phrase vocabulary, the 224x448 front-over-down mosaic
  (a missing down frame is recorded or refused) and a strict output reader.
  Anything but `FF DD YY[ LAND]</s>` is a parse failure the client holds as
  zero; the old last-three-integers reader had turned `00 49 494 49` into a
  5 m dive. Outputs are per-decision STEPS (forward 0-5 m, down -5..5 m,
  heading change +-1.1 rad), not velocities.
- **VLA server protocol v2** (`demo/vla_server_v2.py`, also
  `demo/vla_server.py --v2`): TCP on 127.0.0.1, newline JSON; refuses unknown
  fields and any field, object word or CSP text that could carry target
  coordinates; answers every request it reads (a failing log write, a
  decompression-bomb frame, a 400-digit number get `ok=false`, not a dropped
  connection); puts the model identity (weights_sha256 over the base shards
  plus the adapter, precision, attention, merge, measured agreement) on every
  answer. **No silent model default**: it refuses to load until
  `--precision`, `--attn` and `--merge`/`--no-merge` are given.
- **AerialVLA serving on the RTX 4090** (`docs/data/aerialvla_serving_4090.json`):
  all 8 precision x attention x merge configurations meet the 1.0 s p50 gate.
  bf16/sdpa/unmerged reproduces the upstream-precision reference on 96 % of
  the probe (p50 486 ms, ~16 GB); bf16/sdpa/merged is fastest (259 ms, 88 %);
  nf4/sdpa/merged is the low-VRAM option (517 ms, ~7.9 GB process VRAM) but
  agrees on only 25-33 %. CityLife loaded leaves ~19.4 GB free, so bf16 is
  expected to fit beside it (not yet measured in flight).
- **The pilot client** (`guardrail/vla_link.py`, `RemoteVLABackend` in
  `guardrail/vla_backends.py`): one outstanding request, pipelined captures, a
  `StepTracker` that turns each step and its capture pose into a 10 Hz
  world-frame action (rotate only when |dpsi| >= 0.25 rad, as upstream; cruise
  cap; altitude inside the tighter of the configured and policy bands), LAND
  reported to the rail (never a raw mode request), hover on a stale or failed
  decision. Every tick names its author (vla_step / hold / hover / none). The
  client re-reads the answer text instead of trusting `parse_ok`, pins the
  model identity (a different model mid-run is refused, `model_changed`), and
  nothing in it can stop the 10 Hz loop (logging on its own thread, callback
  errors counted).
- **Dashboard** (default page of the local tracker, `tools/build_dashboard.py`,
  `GET /api/dashboard`, `GET /api/live`): VLA in the loop, running now (with
  heartbeats from `tools/live_status.py`, written outside OneDrive), WP
  progress, grant KPIs (target / value / null / grade / as_of / source; not
  measured never shows as 0; green only when on target, KPI-grade and better
  than its null), latest flights with H.264 videos, timeline and the open
  to-do list (`tools/todo.py`, `tracker/data/todo.json`, 44 items), in
  EN/ID/zh-Hant. **A run counts as a VLA flight only on the server's own
  answers in its records** (at least 10 accepted parsed decisions carrying one
  real weights digest, and 10 ticks flown toward them); today 0 of 99 recorded
  runs qualify. A red banner shows when a flight and a workflow overlap.
- **ArduPilot rail start gate** (`demo/pas_ardupilot/perception_node.py`):
  after take-off the aircraft hovers until the detector has seen a plausible
  subject 5 times in a row within 30 m; a timeout flies no mission unless
  `--start-timeout-action fly`. The hover is watched tick by tick
  (HoverWatch): a pilot or failsafe takeover ends it at once, the Shield
  checks every hover tick, and an unsafe hover tick starts the loop so the
  Shield and the escalation FSM act. New options `--identity`,
  `--negative-query`, `--lock-gate`. Replayed on the 21 Oct flight's 345
  inferences the gate never opens: no box there was the car. Not flown.
- **Sweep**: a sideways-pilot cell (`time-window-opens-on-sideways-pilot`,
  2 m/s east, bounded at 7 ticks inside, 2.9 s to safe, never censored), and
  per-episode `cap_refused_ticks` / `floor_refused_ticks` printed beside the
  P0 headline, zero included.

#### Changed
- **The city rail flies the signed bundle of its policy by default**
  (ARCH-29); the YAML only under `--dev-yaml`, recorded as a development run.
  Build the bundles first: `python tools/build_flight_bundles.py`.
- **C2, the time-window escape.** When a zone's window opens on the aircraft
  sooner than any action within the hard caps can take it out, the zone is
  judged by the inside rule from the opening, and the escape flies at the
  speed cap until the aircraft is out of the ring. That escape is now
  forecast as flown (out of the ring, then a stop at one forecast step and
  3 m/s^2), so a rule beyond the stop no longer refuses the cap; when one
  lies inside the stop, the Shield tries the fastest clean speed, then
  reports a violation so the FSM escalates. No FSM edge or KPI formula
  changed. The library P0 escape of 0 for these cells is a re-judgement of the
  same flight path: the aircraft is still inside the polygon 3-13 ticks, as on
  7c9c9e6, at most 2.8 s to safe.
- **C8**: the stand-off CSP sentence binds the followed subject ("the
  pedestrian you are following"), not "any pedestrian". **A14**: the
  paraphraser refuses narrowed or widened objects (held-out false accepts
  5/30 -> 0/30 on the dev set; 4/12 on a fresh held-out set). **WP1-22**:
  generated policies get unique ids.
- Evaluation RPCs on the ArduPilot rail run off the loop (the 21 Oct
  flight's 1028 ms tick was a 1024.9 ms simulator truth read), and the
  detector loads before SITL. Neither change has flown yet.

#### Fixed
- **LAND on every way out from the arming** (ArduPilot rail): an exception or
  Ctrl+C anywhere after arming, including the start-gate hover, lands the
  aircraft and writes the episode, unless the autopilot or a pilot has it; the
  net's own prints cannot raise past its LAND.
- A Ctrl+C caught by the start gate's hover ended the Dashboard row "ok"; it
  now ends "stopped" (found by the V1c/V1d merge, failing-first test).
- A run that flew no mission writes None (not a 1 MHz detector rate and zero
  dwell) for its rates and counters.
- The pilot client no longer treats a server error as a decision: a server
  refusing every request for 30 s read as 299 VLA holds at 9.97 Hz, now as
  300 `none` ticks and a decision rate of 0.

#### Retracted
- **The 4 Aug AerialVLA ablation**, read strictly: the original adapter stops
  on 17 of 18 no-hint frames, not 11, and the object-presence effect is not
  supported (`docs/data/aerialvla_ablation_rescore.json`). All its numbers are
  per-step displacements, not rates.
- **"The library has no such zone"** (V1e review notes): the corridor edge at
  x = 80 is 9 m past the school zone's ring at x = 71.

#### Measured (headless sweep, republished on the merged tree)
- Library, grant arm: 140 pass / 22 tracked / 0 fail / 18 known over 180
  episodes; P0 escape 0.0 over 179 Shield-on episodes (null passthrough
  0.234); fail-safe 145/171 = 0.848; mission success 0.603; mean time to safe
  4.31 s; 0 slowed escapes. The fail-safe KPI target (>= 0.99) is still not
  met.

#### Tests
- The CI list runs 110 test files on the merged tree, 0 failing.

#### Not done
- No VLA flight. The CityLife VLA pilot (`demo/vla_fly.py`) is the next
  branch; its first flight is the milestone.
- The 4 LLM-authored stored paraphrase sets stale since C8 are pinned, not
  re-authored. PI decisions, the Orin, the fail-safe gap and Gazebo remain
  open (`docs/CHECKLIST-remaining-work.md`).

### 2026-10-21 — the Shield forecasts from the aircraft's own velocity, legal hold points, a realistic harness vehicle, and a smoother city follow, flown

The to-do list's T1 wave, four branches each reviewed adversarially and again
for the reviewers' notes (merged 0931822, e20544a, 1a9791b, 08cd0be, and the
integration review's fixes in d1e93a9; 9bf5a64..d1e93a9, 25 commits), and
four flights. None of the flights is KPI-grade, and no VLA flies them (a
hand-written controller steers by OWL-ViT's boxes); every harness number
below is headless.

#### Flown
- **The city follow with its new motion, twice** (simple_flight rail, the
  CityLife red car with the no-fly zone, the unsigned YAML policy and the
  3 s / 0.5 s forecast, as on 13 Oct; 240 s each, videos 268-269 s):
  `demo/out/citylife_redcar_nfz_motion_20261019` at `--accel-max 2.0`
  (ece38c1), then `citylife_redcar_nfz_motion2_20261019` after the defaults
  were raised (c77c77d, below). Against `citylife_redcar_nfz_20261013`:

  | | 13 Oct (old motion) | 19 Oct, 2.0 m/s^2 | 19 Oct, 2.5 m/s^2 |
  |---|---|---|---|
  | tilt over 15 deg | 9 episodes | **0** | **0** |
  | max tilt | 19.1 deg | 11.8 deg | 12.8 deg |
  | max pitch rate | 181 deg/s | 90 deg/s | **20.2 deg/s** |
  | max roll rate | 47 deg/s | 97.8 deg/s (2 ticks over 47; p95 8.3) | **15.3 deg/s** |
  | commanded acceleration, max | 11.31 m/s^2 | 8.00 m/s^2 | **5.56 m/s^2** |
  | within 30 m of the car | 0.991 | 0.94 | 0.963 |
  | minimum separation | 12.0 m | 11.0 m | 10.8 m |
  | in the zone / P0 escape / interventions | 0.0 s / 0.0 / 1 | 0.0 s / 0.0 / 0 | 0.0 s / 0.0 / 0 |

  The first flight's shortfall was catching up: 142 of 2377 ticks beyond
  30 m (at most 33.7 m) in four spells, one after the start and three after
  FenceGuard sidesteps. With the raised defaults: 89 of 2400 (at most
  32.0 m), four spells, each after a sidestep, none at the start.
- **The ArduPilot close approach with the own-velocity forecast on and off**,
  build 08cd0be, the 14 Oct arguments (want width 0.5, seed 20261006, the
  signed `follow-car-citylife` bundle, pymavlink; Project AirSim + ArduPilot
  SITL, none of the grant's topologies, not KPI-grade):

  | | on (`pas_ap_ovf_on_20261021`) | off (`pas_ap_ovf_off_20261021`) | 14 Oct (`pas_ap_video_on_20261014`) |
  |---|---|---|---|
  | ticks inside a P0 margin (building clearance) | **0** | 79 (BRAKE 21, RTL 58) | 71 (20 while the Shield flew) |
  | deepest | **0** | 3.18 m | 1.02 m |
  | end | the 120 s limit | RTL (X7) | RTL (X7) |
  | P0 escape ticks (commanded basis) | 0 (0) | 0 (0) | 0 |
  | loop | 9.91 Hz | 9.98 Hz | 8.97 Hz |

  On: 1190 ticks, 1168 flown by the Shield, the measured velocity on every
  tick (LOCAL_POSITION_NED) and never set aside; FSM edges G1 G7 G1 G7 G1 G2
  G8 G1 G7 G1 G7, no X7. Its one Loiter (G2 at 47.3 s, 1.01 m/s) was flown
  as a GUIDED stop for 9 ticks - BRAKE's stop was predicted legal, but the
  aircraft was over 0.5 m/s - and handed to BRAKE at 0.45 m/s; G8 at 50.4 s;
  no forced hand-over, 0 stop-advised ticks; the HUD read "Loiter(GUIDED)"
  on those 9 ticks. Off: G3 at 40.9 s with the aircraft at 2.5 m/s, BRAKE
  carried it into the margin (1.68 m deep), X7 sent it home at 43.0 s and
  the RTL climb reached 3.18 m - the 13 Oct pattern; none of its 79 ticks
  was flown by the Shield. Real-time factor 0.965 / 0.962 (0.935 on 14 Oct);
  `demo/out/pas_ap_ovf_compare_20261021.mp4` stacks the two (120 s).
  **Read with care:** the detector again did not box the car (0 of 186
  subject estimates within 6 m on the on run, 0 of 190 off; the nearest
  180 m away), so these runs test the Shield, not tracking; and the
  trajectories differ run to run (first building-clearance flag at 44.4 s
  on, 40.3 s off, 40.8 s on 14 Oct): one flight each is not an A/B proof.

#### Added
- **The Shield forecasts from the aircraft's own velocity** (T1a;
  `State.velocity`, optional; `guardrail/response.py`;
  `docs/DESIGN-own-velocity-forecast.md`). With a measured velocity each
  forecast pose first flies the setpoints already sent for the reaction time
  (`Shield.note_flown`, the pipe), then turns towards the candidate at the
  acceleration limit. Fitted on the six recorded ArduPilot flights
  (`tools/fit_velocity_response.py`): identified 0.35 s / 2.0 m/s^2; the
  Shield uses 0.45 s / 1.75 m/s^2, the most accurate point whose MEDIAN
  braking error at 1.5 s, in the form the Shield flies, is not short
  (+0.03 m; not a bound: p25 -0.33 m there, the median -0.16 m at 2.0 s).
  The velocity only ADDS caution: every rule is judged on the own-velocity
  forecast and on the old commanded one, and a rule either breaks is broken
  (a seeded property test: nothing the old Shield refused is emitted clean);
  from an illegal position the velocity is set aside and the decision is
  the old Shield's. Without a velocity every decision is byte-identical
  (18,000 seeded decisions against c77c77d). Replayed, the 14 and 13 Oct
  approaches flag the building 1.09 and 0.98 s earlier; in closed loop on
  the identified aircraft both stay outside the 3 m margin (3.28 / 3.48 m)
  where the commanded forecast is inside on 73 ticks of each and ends in X7.
  At 50 rules `_check` takes median 0.87 / p99 1.23 / max 1.51 ms with the
  velocity, against the 5 ms budget.
- **On by default on the Project AirSim + ArduPilot rail** (pymavlink and
  MAVROS, Shield-on arm; `--own-velocity-forecast off` restores the
  commanded forecast and BRAKE hold). The SITL rails log the velocity but
  default off (the response was measured on the Project AirSim quad); the
  harness and `demo/follow_vlm.py` pass none.
- **A hold that stops clear** on that rail: the FSM's Loiter is flown as a
  GUIDED stop through the Shield (`Shield.advise` of a zero action) while
  BRAKE's predicted stop would break a hard rule or the aircraft is over
  0.5 m/s, then handed to BRAKE; capped at 3 s (ArduCopter's GUID_TIMEOUT),
  and a stale link or a 0.3 s gap in the stream hands it over at once
  (`metrics.hold.stop_clear`, `forced_handovers`). The FSM's states, edges
  and inputs are unchanged. A stop-advised tick has its own audit record
  (`decision_kind: "stop_advised"`), rows carry `stop_advice` and
  `p0_escape_commanded_basis`, and the HUD names a deferred Loiter.
- **A vehicle-response comparison arm in the stress harness** (T1b;
  `--response-arms ideal ardupilot`; `docs/DESIGN-harness-vehicle-response.md`):
  each episode flown again on ArduPilot's measured response (delay 0.08 s,
  2.4 m/s^2, jerk 5.7 m/s^3, lag 0.03 s; `tools/measure_vehicle_response.py`
  over the recorded flights), labelled `/response-ardupilot`, ranked in its
  own top-K, written to `demo/out/cmp_*`, never in the headline counts, pins
  or exit code. Nightly, ideal -> ArduPilot: unsafe-position entries 6 -> 12,
  autopilot polygon entries 12 -> 84, P0 escapes 18 -> 18, true-state P0
  ticks 8894 -> 12189 judged on the command and 8894 -> 16382 on what the
  airframe flew, fail-safe labels 532/705 -> 490/705. Flying the arm changes
  nothing of the headline run, field for field.
- **Legal hold and landing points** (T1c; `guardrail/holdpoint.py`,
  `EscalationFSM.needs_hold` / `hold_at`; `docs/DESIGN-hold-points.md`).
  Entering Loiter or Land the FSM asks for a point and the Shield's own rule
  code answers: in place where a stop there is legal through the lookahead
  (every legal hold of the library is unchanged), else the nearest legal
  point within 60 m, reached by a straight transit judged at every instant,
  not at sampled poses - it never enters a rule in force that the aircraft
  is outside of, and leaves one it is in only by getting shallower, by the
  Shield's own depth (160 random transits in 40 seeded worlds: none called
  legal enters or dips). The search spends at most 600 units of about
  0.1 ms (a count, not a clock); a fruitless one costs 29-41 ms on the
  desktop. No FSM edge, guard or setpoint moved; Loiter / Land records gain
  `hold_point`, `hold_legal`, `hold_needed`, and the audit record (version
  3) `fsm_hold_point`. The harness's modelled autopilot flies a moved point
  as a GUIDED transit; `--autopilot-hold in_place` flies what the rails fly,
  c77c77d's rows exactly. **No flight rail flies a moved hold yet.**
- **The DSL's first lint WARNING class** (`python -m guardrail.compiler
  lint`, `ConstraintCompiler.lint_warnings`; never refuses): a hard zone
  whose window can open on an aircraft that cannot leave its margin ring
  within the lookahead, or, for a mission start, before it opens. No shipped
  policy warns at the policy level (29 files); the three
  time-window-opens-on-vehicle cells carry one each.
- **The WP1 cold start, timed and published** (T1d; `docs/data/
  wp1_roundtrip.json`, `docs/DESIGN-wp1-cold-start.md`): a signed 50-rule
  bundle to a ready Shield in fresh interpreters, p95 376 ms against the
  grant's 1000 ms (worst bundle, imports included; whole process p95
  504 ms), every run verified, and a tampered, an untrusted and a
  bad-signature bundle refused. Desktop only, not the Orin.
- **Parquet export** (`tools/export_episodes.py`, pyarrow optional): one
  typed column per field, read back against the JSONL before a file is kept
  (a missing column fails it); 6 stored episodes, 4616 step rows, equal. The
  3.11 extras CI job installs pyarrow 26.0.0.
- **SROS 2 behind `SROS2=1`** (`sitl/run_ros2_demo.sh`,
  `tools/sros2_setup.sh`, `tools/sros2_check.sh`): one enclave per ROS 2
  process under Enforce, on the lab CA; the live check 6/6 in WSL Jazzy. Not
  flown; the default `SROS2=0` changes no command line.

#### Changed
- **Stress-harness results with hold points** (grant arm; the modelled
  autopilot flies moved holds, which no rail does): library pass / tracked /
  fail / known 132 / 22 / 0 / 22 -> 133 / 22 / 0 / 21, fail-safe labels
  140/167 -> 141/167, autopilot-flown P0 ticks 1738 -> 1153; nightly
  508 / 66 / 0 / 158 -> 511 / 66 / 0 / 155 and 532/705 -> 535/705; smoke
  unchanged (52 / 6 / 0 / 4). These differences are by design, and only one
  fail-safe fix is robust: `dyn-nfz-rotate-spin` at 9 deg/s (pin removed;
  its 3 nightly seeds too). The six `three-simultaneous-violations` cells
  first read as passes only because the 300-tick run ended 0.2 s before the
  Loiter timeout; flown 400 ticks, G5 sends every one home, so they stay
  pinned on both labels (what is real: X6 no longer fires, and the climb
  into the band flies 1-2 autopilot P0 ticks where 72-78 were flown). The 6
  Shield P0 escape ticks of the time-window-opens-on-vehicle cells are
  unchanged (flown before G3); their hold moved to (60, 31.9), autopilot P0
  38 / 33 / 28 -> 18 / 13 / 8.
- With the velocity, the rail's P0 escape re-check is made with it, as the
  decision is: never laxer than the commanded basis (property test), and it
  can be stricter; rows and `metrics.shield_forecast` carry both.
- Bundle extras are `sweep-episode-extras/3` (both `arm.response` and
  `arm.autopilot_hold`); `rerun_from_extras` reads /1, /2 and /3, and a
  bundle from before the hold points is re-flown in place.
- The city follow's defaults after the first flight (c77c77d):
  `--accel-max` and `--guard-release-accel` 2.0 -> 2.5 m/s^2, `--catchup-mps`
  2.5 -> 3.0.

#### Fixed
- `tools/build_eval_data.py` counted the commits since the 2 Sept meeting
  with a bare `--since 2026-09-02`, which git reads as that date at the
  current time of day: run after 22:58, the meeting day's 22:58 commit
  (96746a9) dropped out (15 instead of 16). It now passes midnight, and a
  test checks the argument git receives (c57e007).
- Found by the wave's reviews, each shown failing first: the episode
  exporter split a run's audit at every stop-advised tick (40 of 60 rows
  joined on a test run; now 60 of 60, one flight); the Parquet read-back
  kept a file missing a whole column; the cold start read a loader that
  never verified the signature as within budget; the hold search could fly
  through a zone closing on the aircraft and judged a transit only at poses
  1 m apart (a fuzz: 27 of 195 moved holds entered a zone they started
  outside of); a candidate inside k nested zones was charged k zone reads
  (with 8-10 nested squares the hold stayed inside where 51.5 m out was
  legal; now moved, 14 ms); a speed cap's reason called the default an
  enforced cap; a zone with `valid_time: {}` was no wall to the lint; and a
  lint call with no legal exit stepped a week at 0.05 s (12.6 s; now
  0.03 s).
- At the merge: the hold plan read the tick's momentum-aware stop check and
  moved 24 of 337 seeded Loiter holds the FSM had just called legal (it now
  reads the FSM's own position check); the response arm flew a moved hold
  as the ideal vehicle's command, passing a point 10 m away by 4.4 m (now
  ArduPilot's position controller, under a metre).

#### Not fixed
- No flight rail flies a moved hold point, so the harness's autopilot
  numbers with them are modelled (`autopilot_hold_model`); the 2 Shield
  escape ticks per opens-on-vehicle cell remain (flown before G3; the lint
  names the cause).
- The response model is one form fitted to six flights of one simulated
  airframe: on an aircraft slower than the Shield's numbers (0.55 s,
  1.5 m/s^2) the closed loop's approaches entered the margin 0.8-1.0 m deep
  on 26-33 ticks. BRAKE is an envelope of two recorded entries, neither
  clean (a SITL BRAKE step test is open), and the velocity set-aside drops
  the momentum for every rule when any hard rule is broken.
- Not wired: the own-velocity forecast in `demo/follow_vlm.py` and in the
  harness's Shield, `land_site_ok` in `demo/follow_vlm.py`; the SITL rails
  default off. The cold start and the hold search are measured on the
  desktop only, not the Orin; SROS 2 has not flown, and no per-topic
  permissions are written. The ArduPilot rail still has no start gate.
- For the PI: the velocity in the P0 escape re-check; a GUIDED transit
  inside the grant's "Loiter"; whether `three-simultaneous-violations`'
  label or our 30 s Loiter timeout is wrong; a lint warning class.

#### Tests
- The CI loop runs 103 test files (98 before): new
  `tests/test_own_velocity_forecast.py` (27/27),
  `tests/test_harnesstruth_response.py` (19/19),
  `tests/test_hold_points.py` (33/33), `tests/test_hold_transit.py` (14/15,
  the opt-in wall-clock test skipped) and `tests/test_window_lead_lint.py`
  (11/11); `tests/test_build_eval_data.py` (10/10) checks the `--since`
  argument.

### 2026-10-19 — an October bug hunt: 100 verified defects fixed, a smoother city follow (not yet flown), and a complete CI list

A nine-area review on 18-19 Oct found 107 candidate defects; independent
verifiers confirmed 100 (6 refuted, 1 uncertain), fixed in nine groups, each
reviewed adversarially and again for the reviewers' notes (1b95383..2e81f21,
33 commits). Nothing here has flown: the city follow's new motion is tested
offline only, and no VLA flies the current demos. No pinned digest, decisions
fixture or published `docs/data` file was regenerated; the stress harness's
smoke profile scores 52 pass / 6 tracked / 0 fail / 4 known failures before
and after, and the smoke and nightly sweeps score as before apart from the
added `p0_violation_escape_rate_status` field.

#### Fixed
Safety Shield (`guardrail/shield.py`):
- **A crawl out of a P0 breach is floored even with nothing else wrong.** A
  legal but too-slow escape (from a zone, a stand-off or clearance ring, or
  under an altitude floor) gets the repair floors when nothing else, or only a
  monitor_only rule, is violated; before, 1 m inside an NFZ leaving at
  0.15 m/s took 6.7 s alone (0.5 s beside a yaw breach), and a 1e-6 m/s climb
  from under a floor passed every tick. The floored action flies only if it
  re-checks clean and carries only theta-exempt repairs (else the raw action
  flies, traced `floor_refused`). `filter()` can now return repairs with no
  violation, which the FSM reads as clean; `ShieldDecision.touched` counts
  them, so rails that log it show these ticks as interventions.
- **Hot-apply refuses what it would not enforce or cannot build, atomically.**
  An `altitude_ref: MSL` event (enforced as metres above ground before) is
  refused (`HotApplyRefused`, REST 422), and one whose runtime fails to
  compile changes nothing and returns 422; before, it returned 500 with the
  live policy a generation on and the rule listed but not enforced.
- A NaN or inf subject is dropped instead of silently disabling the stand-off;
  a `subject-finite` violation is raised only while a hard, enforced stand-off
  binds it. `Shield.forecast()` turned headings 57.3x too slowly (rad/s added
  to degrees); first-feasible mode judges theta on the per-axis repair sum.

Policy DSL, compiler, manifest and replay:
- A higher layer that re-declares a lower layer's hard rule can no longer
  switch it off or swap it wider, at ingest or by hot-apply; the winner
  records the overridden layers in a new optional `locked_by` (11 additive
  lock pins, no stored hash moved).
- Schedules refuse `days: []` and clocks like '10:75'. CSP altitude bands are
  intersected per instant, so day/night bands no longer crash `compile_csp`;
  a real clash raises `CSPAltitudeConflict`, now listed and counted in the
  coverage and KPI reports. A class-specific stand-off's reason binds it to
  the followed subject, not "any pedestrian" (the sentence: see Not fixed).
- An empty flight log is refused by `write_replay` / `verify_replay` (it
  verified as a perfect run) and scores outcome fail (`no_rows`); a log the
  autopilot flew entirely reads escape rate None, not 0.0 (none of 1835
  stored flight logs or 55 replay bundles is one).
- Manifest: an ambiguous HuggingFace snapshot or unreadable weights record
  'unresolved'; `sim_speedup` was inverted. `AeroVLABackend`'s skeleton
  raises on its first tick instead of hovering.

Escalation FSM and KPI report:
- A `brake` rule where a stop is legal is answered by the Shield's stop: a
  soft P1 brake beside a hard NFZ no longer takes G3 into LOITER.
- `failsafe_instrumented` reads the value, not the key (Shield-off arms log
  nulls); the per-tick false-trigger count, 0 by construction, is always a
  vacuous zero, and the report shows labelled episodes (`k of m ep`).
- A floored tick is not a repair attempt (`repair_floored_ticks`; a scan of
  644,234 stored rows found none). Bundles with no forecast label file under
  `/forecast-unlabelled`, so a comparison arm is no longer dropped.

REST / gRPC API and harness:
- **REST mTLS allow-list bypass closed.** A lab-CA certificate not on the
  hot-apply listener's common-name allow-list could get a POST applied to the
  flying Shield by sending it in the same write as its TLS Finished (only the
  response was dropped); the gate now aborts before uvicorn parses anything.
- The harness's session forward speaks mTLS (`serve --rest-pki`). StopSession
  waits for a flying episode, stops a session once, and still stops it if it
  raises. Bad lat/lon: 422 / INVALID_ARGUMENT, not 500; NaN or Infinity in a
  gRPC payload: INVALID_ARGUMENT.

The city follow (`demo/follow_vlm.py`):
- **Smoother motion against the pitch/roll jerks** when the car turns or pulls
  away: a dt-aware vector slew limiter (speed-up 2 m/s^2, braking never softer
  than the old 0.4 m/s step, turns 2.5 m/s^2, yaw-rate slew 2 rad/s^2), a
  guard smoother (urgent brakes immediate), catch-up capped at 2.5 m/s and a
  line-of-sight yaw feed-forward (0.7, capped 0.4 rad/s). Offline closed loop:
  speed-up peak 3.11 -> 1.62 m/s^2, jerk peak 8.35 -> 3.44 m/s^3, turn lag
  5.0 -> 1.4 deg, range max 24.1 -> 23.8 m. `--legacy-motion` restores the
  old motion; `params.motion` records it. Not yet flown.
- City manifests file `vla_model_hash` = `none:hand-written-controller`, with
  OWL-ViT under `pilot.detector`. Without `--identity` the estimator also gets
  each box at its capture's pose and depth; `det_hit_rate` counts mission
  inferences (13 Oct: 0.993; over all inferences, 0.495); a follow without
  subject truth reads mission_success null; an abort before the first tick
  exits non-zero.
- FenceGuard's brake bites at once inside a building's clearance ring with a
  no-fly zone in range (the ring was measured against the zone, 13 m instead
  of 4.7 m); car missions with `--level-peds` stop polling the 40-name crowd
  (429 polls a flight).

Project AirSim + ArduPilot rail (`demo/pas_ardupilot/`):
- **A frozen autopilot link fails safe.** SITL dead behind mavlink-router, a
  closed socket or MAVROS losing its FCU no longer counts as live: one
  zero-velocity setpoint, then nothing; 3 s stale ends the episode as "link
  lost" and withholds the KPI grade.
- An unclassified subject ("a child") binds the catch-all "*" stand-off
  instead of turning every stand-off off; the subject is placed from its own
  capture's pose and depth; the loop paces to a 10 Hz deadline (8.97-9.16 Hz
  before) and dwell is time-weighted from timestamps.

SITL and ROS 2 rails, deploy:
- **The pymavlink rail flew on a pose 3-6.6 s old** (pymavlink's oldest queued
  LOCAL_POSITION_NED); it now takes the newest, on a 10 Hz deadline. Every
  stored `sitl_*` and `run_gazebo_demo.sh` run was measured on the delayed
  pose (`sitl_ped_on`'s 21.6 s `alt_violation_s` and its X6 escalation too).
- **ROS 2 node: a stale pose fails safe.** A watchdog (`--pose-timeout-s`,
  1.0 s) holds the aircraft; the FSM is not stepped on a stale pose (from
  Loiter, G8 could return to GUIDED on a frozen position), and
  `EscalationFSM.unobserved()` makes G7 / G8 wait T_recover after the gap.
- ROS 2 node: a pilot or failsafe leaving the node's hold is a takeover (no
  GUIDED, no LAND after it); mode requests go to the adapter only when it
  subscribes. All rails accept `--target -20,15`. Both Docker images keep
  numpy 1.26.4: neither could be built (ResolutionImpossible), nor has been.

Stress harness, tools and tracker server:
- **Known-failure pins name their gates.** Any other gate failing in a pinned
  cell is NEW and exits 1; before, a P0 escape, a breach or an unsafe end
  there read KNOWN with exit 0. A pin can bound its gate: the three
  time-window-opens-on-vehicle cells are bounded at the measured 2 escapes
  (0.0073-0.0076).
- `--backend sitl` scores a rail flight from its own files (every gate read
  'not measured'); a failed KPI rollup exits 4 and writes no partial report.
- `tools/check_claims.py` passes on a clean clone and scans the code packages
  and root guides (288 files clean); `tracker/serve.py` refuses a foreign
  Host, Origin or non-JSON save, so a web page can no longer wipe the board.

#### Changed
- Every city flight's default motion changed; a recorded flight needs
  `--legacy-motion` (plus `--legacy-tick-pose`, `--legacy-servo-width`) for
  its old motion, and its tracking, guard-hold and intervention counts are not
  comparable with new flights'.
- Perception-rail flights should run at about 10 Hz, not 9.15 Hz, with
  time-weighted dwell (about 8-9 % above tick count x 0.1 s on a 9.16 Hz run).
- The KPI-report families of the 2396 stored legacy stress bundles are
  renamed on regeneration (a `--baseline` diff shows each as new and as gone);
  `expect: known_failure` without `known_failure_fails` is refused at load.
- `Shield.set_subject` takes an optional `velocity=(vx, vy)` and then
  forecasts the subject at constant velocity; without it every decision is
  byte-identical (18,000 seeded stand-off decisions compared). No rail passes
  it yet.
- README and `docs/index.md` say no VLA flies the recorded demos (a
  hand-written controller steers by OWL-ViT's boxes, the SITL rails fly
  StubVLA, and the last VLA-commanded flights were on 4 Aug 2026), carry the
  13/14 Oct Shield-off controls (727/1100, 736/1099), and test their sample.

#### Corrected
- The site said the Shield forecasts 5 s at 0.1 s steps; the city
  demonstration flies 3 s at 0.5 s by default, the 14 Oct recording
  (`citylife_redcar_nfz_20261013`) included, which also flew the unsigned
  YAML policy. Its P0 escape 0.0 is on that forecast; separation and time in
  the zone do not depend on it.
- The 14 Oct start range is about 151-152 m, not 119-152 m; the preamble no
  longer calls P0 escape "the hard KPI"; `docs/aerialvla-ft-report.md` headed
  all rows "pure VLA", but its three 100 % rows flew goal_blend 0.55.
- The August and September reports open with a dated note on the
  retractions; the 13 Oct BRAKE ticks were ArduPilot's BRAKE mode under the
  FSM's Loiter; the perception rail's 27.9 s is 279 ticks x 0.1 s (30.4 s by
  timestamp); RUNBOOK's Rail 3 is the grant's dev topology flown by the stub.

#### Not fixed
- The altitude and corridor-band trend monitors accept any climb and the
  clearance monitor any forecast gain, so where a floor is refused a crawl
  flies with no violation; raising them to the reference's 0.1 m/s changes
  pinned `_check` answers and needs a deliberate regeneration and re-score.
- The CSP stand-off sentence still says "any pedestrian": rewording it needs
  the LLM-written paraphrase sets and the OpenVLA tokenizer table regenerated.
- No rail passes the subject velocity (`follow_vlm`, `perception_node`,
  `ros2_shield_node`). The city rail stays on 3 s / 0.5 s until it does and
  the moving-subject stand-off repairs are measured against theta with the FSM
  on: held still, the grant's forecast fired StandoffHold on 529 of 1190
  replayed city ticks (44 %).
- FenceGuard's hysteresis, continuous k and clear_aim margin are unchanged
  (pinned single-call values); `follow_vlm` still matches "a delivery man" to
  the van class.
- Off CI: 6 test files that need OpenCV, Pillow, projectairsim or node
  (`test_range_and_lock` holds most city-follow regressions);
  `test_deck_scorer_parity` counts its JavaScript checks as passes without it.
- The ROS 2 node does not check at mission start that the adapter is alive;
  during a stale-action hold the FSM's clean streaks keep running.
- Documents citing `sitl_*` runs are not yet marked as delayed-pose; stored
  KPI reports and `docs/data/csp_kpis.json` show old cells until rebuilt; the
  sweep's SITL command passes the subject unrotated.

#### Tests
- The CI loop runs 98 test files (40 before). The 6 left out are listed with
  measured reasons, and `tests/test_harnesstruth_ci.py` fails on a file in
  neither list, a listed test that can return None early, or a runner that
  does not count SKIP; verified on Linux 3.10.22 / 3.11.17 venvs, a clean
  `git archive` copy and a depth-1 clone. The 3.11 extras job installs
  uvicorn and jsonschema and fails on a SKIP from the REST-listener tests.
- 7 new test files, and regression tests in the existing ones (53 test files
  changed in all).

### 2026-10-14 — recorded videos of both rails, the ground-truth question answered, an English explainer pack

#### Flown
- **The usual demo video, re-flown on today's code**
  (`demo/out/citylife_redcar_nfz_20261013`, simple_flight rail, the CityLife
  red car with the no-fly zone and the policy indicator): P0 escape 0.0,
  0.0 s in the zone, 0.991 of the time within 30 m of the car, minimum
  separation 12.0 m; 5011 frames at 19.28 Hz. Video
  `citylife_redcar_nfz_20261013_demo.mp4` (260 s).
- **The ArduPilot rail recorded for the first time**, the close approach
  with the Shield on and off (Project AirSim + ArduPilot SITL over pymavlink,
  none of the grant's topologies, not KPI-grade):
  - Shield on (`demo/out/pas_ap_video_on_20261014`): 517 ticks, 0 P0 escape
    ticks in 388 Shield-flown ticks; the FSM braked, then sent the aircraft
    home (RTL); real-time factor 0.935 (under the 0.95 band); 1105 frames
    at 19.19 Hz.
  - Shield off (`demo/out/pas_ap_video_off_20261014`): 1099 ticks, 736 P0
    escape ticks (rate 0.6697), the raw command flown into the
    building-clearance margin; 2372 frames at 19.75 Hz.
  - `demo/out/pas_ap_video_compare_20261014.mp4` stacks the two (120 s).
  - Where the aircraft was, not only what was flown: with the Shield on it
    was inside a P0 margin (building clearance) on 71 of 517 ticks, 20 of
    them while the Shield flew (up to 1.0 m deep, no emitted command
    breaking a rule) and 51 under RTL; with the Shield off, on 739. The
    Shield's forecast starts from the commanded velocity, not the aircraft's
    own (`_predict`; `State` carries no velocity), so the aircraft's carry
    while it slowed from about 3.2 m/s went unseen. Next: a forecast from the
    aircraft's own velocity and a hold that stops clear of the margin.
- **The ground-truth question is answered: the truth was right, the detector
  was not on the car.** The 13 Oct gap between the truth (Car_10) and the
  detector was not a frame error: the rail's Car_10 poses lie on
  follow_vlm's validated Car_10 track (median 0.52-0.54 m). The recorded
  Shield-on video shows the detector's box on a red traffic signal; across
  the close flights 0 estimates came within 6 m of the car, and Car_10 was
  in the camera's view on 10 of the 149 estimate ticks. The rail starts with
  the car about 151-152 m away (`subject_range_m` on the first tick of each close flight: 150.8-151.9 m; the 119-152 m first written here took in the Shield-off flights' nearest approach, 117-119 m, and was corrected on 2026-10-19) and has no start gate (follow_vlm waits until it
  sees the car); until one is ported, this rail's follow scores stay at 0
  and its flights test the Shield, not tracking.

#### Added
- The perception node records like the usual demos (`--save-view`,
  `--record-hz`, `--record-height`; `run_pas_ardupilot.ps1 -SaveView`): the
  drone's camera with the policy indicator (FSM state, Shield-off banner)
  and the chase camera, off the control loop (loop 9.15 -> 9.12 Hz beside a
  CPU-bound detector stand-in). `tools/make_compare_video.py` stacks two
  runs and refuses to overwrite an existing file.
- A frame check for the subject's ground truth (the aircraft read through
  the same simulator call as the car). On the CityLife build the aircraft's
  actor name (UnrealRobot) is not found, so the check reports partial: only
  the EKF against the simulator's kinematics was measured (median 0.388 m).
- An English explainer pack (`docs/share/2026-10-14-Explainer/`, local):
  a 22-slide deck built from `tools/deck/build_explainer_deck.js` on the usual
  template, an English and an Indonesian cheatsheet, and likely questions
  with model answers; five figures drawn from code
  (`tools/deck/build_explainer_figures.py`, `docs/img/explainer_*`): the
  end-to-end flowchart, the components, the topologies, the Shield per tick
  and the evidence chain.

#### Changed
- The perception node's report says what its counters count: "flown-violation
  ticks" (any priority) beside the P0 escape ticks from kpi.json.

#### Retracted
- The 749 P0 escape ticks of the 13 Oct Shield-off close approach (13 Oct
  (later) entry): 749 counted ticks that flew a violation of any priority,
  the 22 P1 speed-cap ticks included; the P0 escape ticks were 727 (building
  clearance 705, stand-off 17, both 5), which is what the 0.661 rate was
  computed from.

#### Tests
- New: `tests/test_railvideo_{recording,compare,truth_frame}.py`,
  `tests/test_explainer_{figures,deck}.py`; each failed on a pristine copy of
  5cf7a08 and passes now.

### 2026-10-13 (later) — G1 holds after the fence change; the perception rail flown close with the Shield on and off

Flights on Project AirSim + ArduPilot SITL (pymavlink, none of the grant's
topologies, not KPI-grade), after the wave-A commit, plus two CI fixes.

#### Flown
- **Gate G1 passed again** after `citylife-quad.param` changed (FENCE_TYPE 7,
  AVOID_ENABLE 0; `demo/out/g1_20261013_203404`): 3/3 runs, all four
  conditions. EKF p95 0.82-0.83 m against the 1.0 m limit (frozen-position
  null 18.96-18.98 m), climb 14.47 m from rest, landing 0.06-0.09 m from
  take-off, 0 fence breaches, 0 airborne collisions.
- **The red-car follow with an aggressive pilot** (`--want-width 0.5`, the
  pilot closes until the car fills half the frame), 120 s, flying the signed
  bundle of `follow_car_citylife` (signature verified; the first signed
  flight on this rail):
  - Shield on (`demo/out/pas_ap_close_on`): 532 ticks, 0 P0 escape ticks,
    173 interventions (AltitudeFix 103, ClearanceFix 85, YawClamp 19,
    StandoffHold 3). The FSM escalated twice to Loiter (a stand-off repair
    of 2.08 m and a building-clearance repair of 2.14 m, over theta 2.0 m);
    in the second hold the illegal depth kept growing (2.95 m in 2.07 s,
    fitted) and X7 sent the aircraft home; RTL reached home at 58.1 s. The
    hold flew as ArduCopter BRAKE and held 7.99-8.01 m over 42 ticks: the
    first flight evidence for the BRAKE hold, which had rested on a SITL
    probe.
  - Shield off, the same pilot and route (`demo/out/pas_ap_close_off`): 1100
    ticks, 749 P0 escape ticks (rate 0.661): building clearance 710,
    stand-off 22, kinematic caps 22. This is the rail's first Shield-off
    control.
  - ArduPilot real-time factor 0.966 (on) and 0.946 (off, just under the
    0.95 band); 0 fence breaches on both; the follow policy declares no
    keep-out zone, so no GeoFence zone was uploaded.

#### Found
- **The subject's ground truth and the detector disagree.** The truth reader
  logged Car_10 (the level's one red car) on every tick, 119-235 m from the
  aircraft to the north-east, while every estimate the detector produced sat
  45-164 m to the south-east; the median gap is 258-264 m. Either the truth
  pose is in another frame than the Shield's, or the detector follows
  something other than Car_10. Unresolved: no truth-based number from these
  runs is used.

#### Fixed
- Two tests added to CI with wave A failed on GitHub only:
  `tests/test_env_pins.py`'s stand-in `waf` lost its executable bit on Linux
  (the later `git add -A` re-read the file mode; Windows ignores it), and
  `tests/test_harnesstruth_bundles.py` needed the gitignored
  `bundles/fase3-sim-demo-v0.1.1.tar.gz`; it now skips visibly without it.
  `test_env_pins` was run on Linux (WSL): the eight setup and verify tests
  that failed there pass; the one left failing is that WSL venv's own numpy
  2.5.0 against the 2.4.6 pin.

#### Changed
- `scripts/run_pas_ardupilot.ps1 -NodeArgs` passes extra arguments to the
  perception node as given.

#### Tests
- `tests/test_env_pins.py` 37/37 on Windows, 36/37 on Linux (WSL's numpy);
  `tests/test_harnesstruth_bundles.py` 11/11 here, 10/11 + 1 skip in a clean
  copy.

### 2026-10-13 — the grant's forecast in the harness, rule clocks and hot-apply on the flying rails, a faster Shield, and the WP4 outputs

Four packages built in parallel on disjoint files from a re-verified inventory
of everything still open (the grant pages, the 5 Oct audit, the tracker and
the KPI evidence chain), each challenged by two reviewers who tried to break
it, then fixed. Nothing here has flown: every number is headless or a
desktop measurement. The Jetson Orin and the drone are not available.

#### Fixed
- **The stress harness flew a Shield the grant does not describe.** Its
  flying, oracle and checker Shields used a 3 s forecast at 0.5 s (6 poses);
  the grant locks 5 s at 0.1 s (50 poses). All three now use
  `guardrail.shield.LOOKAHEAD_S` / `LOOKAHEAD_DT_S` unless a spec overrides
  them, and a labelled 3 s comparison arm is kept out of every headline.
  Cells written around the 3 s lookahead were re-cut for their purpose at
  5 s, each mechanism in its description.
- **GUIDED setpoints flew open-loop under wind.** Every velocity the harness
  flies now goes through one closed-loop model of ArduCopter's position
  controller (Copter-4.5.7 `mode_guided.cpp`, `AC_PosControl`): steady wind
  is rejected up to 6.4 m/s, derived from the SITL default frame, not
  measured. `--wind-model open_loop` keeps the old model.
- **The FSM's RTL -> Land edge (G6) had no input.** `guardrail/rtl_path.py`
  judges every RTL tick: the straight leg home against the P0 zones and band,
  progress over 5 s, and a battery hook.
- **Time windows and switches were inert in flight.** No rail gave its
  Shield or its CSP a clock. Both SITL rails and the perception node now pass
  one rule clock to both (`--clock-start ISO8601` scripts it; otherwise the
  wall clock in +08:00, recorded either way), and zone dwell skips time in a
  zone the clock had switched off.
- **A YAML policy flew with no acknowledgement.** The flight rails fly the
  signed bundle of their policy (`tools/build_flight_bundles.py`, into the
  gitignored `bundles/`); a YAML flies only with `--dev-yaml`, is recorded so
  and is never KPI-grade. The stress harness passes `--dev-yaml` for YAML
  references and pins every bundle with `--expected-hash`, which the ROS 2
  node now declares and checks before take-off (it had dropped the flag).
- **A pilot whose action stream stopped kept being flown** by the ROS 2
  node. A stale-action watchdog (0.5 s, five missed 10 Hz ticks) now holds
  the aircraft through the FSM's path.
- **The Shield's moving-zone path was over the grant's 5 ms query budget**
  on GitHub's runner (5.71 ms) and close to it on a loaded desktop: 70 % of
  the time went into rebuilding every zone's polygon on every check.
  `guardrail/ir.py anchor_reach` now builds a zone only where the forecast
  can meet it; in 200 checks the Shield builds 199 zones instead of 9600.
  `_check` median at the grant's load, same session: moving 2.53-2.64 ->
  0.45 ms, static 0.35 -> 0.30 ms; decisions identical on every compared
  sample; the per-fence loop (the null) takes 222 ms.
- **The audit's per-repair iteration count was wrong** on 1324 of 4257
  seeded multi-pass records (keyed by operator name), and a relaxed chain's
  failed run vanished from the record. Both fixed.
- **A follow the perception rail could not score claimed mission success**:
  with no subject ground truth, `frac_within_30m` and `standoff_s` were None,
  which `guardrail/kpi.py` read as no failure. It is now not measurable.

#### Added
- **The REST hot-apply API on the flying Shield** (`--hotapply-port` on both
  SITL rails): loopback by default, mutual TLS with the harness CA off it,
  rules swapped under the rail's lock between ticks, each event recorded as a
  new policy generation with its receipt-to-swap time.
- **The perception rail**: a follow mission so a CSP is written per
  generation; the policy's clock-proof zones uploaded to ArduPilot's GeoFence
  before arming (`citylife-quad.param` now FENCE_TYPE 7, AVOID_ENABLE 0; G1
  not re-run since); the followed car's simulator pose logged for evaluation
  only (`--subject-actor`, Car_10 for a red car), with the true nearest range
  and the stand-off dwell; `--hold-mode LOITER` flying the literal LOITER with
  an RC3 override of 1500 on SITL (BRAKE stays the default); the MAVROS path
  brought to parity, and `dev` offered for Project AirSim + ArduPilot over
  live MAVROS evidence (never over pymavlink).
- **The audit record as a Pydantic v2 model** (`AuditRecord`,
  `policies/audit_record.schema.json`) with body-frame actions in the grant's
  keys, every operator considered per pass with its result and skip reason, a
  measured iteration count and a `time-window` category; a reader upgrades
  every stored shape (22 123 records in 73 files).
- **Repair operators as a pluggable library** (`guardrail/repair.py`, a
  `RepairOperator` Protocol), decision-identical by default, plus three modes
  off by default: the grant's first-feasible selection, PathRepair (frees
  standoff-wedge: goal reached, minimum range 10.022 m against the 10 m
  ring), and a segment check between forecast poses (catches a 0.5 m ring
  between two poses at 12 m/s).
- **Self-contained harness bundles** (the expanded cell, events, arm, seed,
  policy and generations in `manifest_extras.json`) and
  `tools/rerun_episode.py`, which re-flies one and compares the flight log by
  hash (a changed seed is the null).
- **Episode exporters** (`tools/export_episodes.py`): step and episode
  records with the grant's names, JSONL always, Parquet when pyarrow is
  installed (it is not, in either env); audit records joined only to the
  flight they belong to.
- **The KPI report's missing sections**: WP1 and WP2 (written to
  `docs/data/csp_kpis.json`: at 120 tokens the compiler keeps 105/105 P0
  rules where a naive cut keeps 104/105; null 0/105), the 100 ms tick budget
  per rail, A/B and paraphrase printed n/m until flown, repair magnitude by
  priority and operator class, vacuous zeros named, a `--baseline` diff, and
  evidence bound to the run's own signed replay bundle.
- **The grant's 10 gRPC RPCs** (StartSession, StopSession, InjectPolicy,
  IssueTask, StreamTelemetry, StreamEvents added), a live mode driving the
  episode runner, HotApply forwarded to a bound REST endpoint, idle sessions
  expiring. Headless only.
- `tools/sros2_setup.sh`: the SROS 2 keystore and enclaves seeded with the
  harness CA, checked with `bash -n` and `--dry-run`. SROS 2 is not enabled.
- Pins: shapely 2.1.2; numpy 1.26.4 on Python 3.10, 2.4.6 on 3.11+.
- `.github/workflows/nightly.yml` (the nightly profile, headless). It cannot
  run yet: GitHub fires scheduled workflows only from the default branch,
  which on this repository is `main`, while the workflows are on `master`.

#### Changed
- `docs/data/scenario_sweep.json` republished on the grant's forecast and the
  closed-loop wind model: 132 pass, 22 tracked, 0 fail, 22 known failures over 176 episodes (165
  cells; the 3 s arm reported apart); P0 escape 0.000133 over 175 Shield-on
  episodes (6 Shield-flown ticks, all in the time-window-opens-on-vehicle
  known failures; passthrough null 0.236); fail-safe trigger correctness
  0.838 (140/167; 26 false, 1 missed; 100/122 = 0.820 over distinct flights),
  below the never-trigger null 0.940 and the grant's 0.99; autopilot-flown
  P0 ticks 1738 of 14 287 (LAND 1004, LOITER 529, RTL 205) in 17 episodes,
  RTL refused (G6's input) in 3. Profiles: library 165 cells, smoke 55
  (62 episodes), nightly 221 (732).
- The KPI report keeps the 3 s comparison arm in its own `/forecast-3s`
  rows instead of pooling it with the grant's.
- `sitl/setup_sitl.sh` pins numpy 2.4.6 (as requirements.txt does for
  3.11+) and installs fastapi / uvicorn for `run_sitl_demo.py
  --hotapply-port`; `start_ardupilot.sh --mavros` starts MAVROS as system id
  255 (SYSID_MYGCS, so ArduPilot takes its RC override) and records its fcu
  url. Neither was re-run in WSL.
- The ROS 2 node's rows carry the end-to-end tick latency; no flight value
  exists yet.

#### Retracted
- The published library sweep of 12 Oct (152 pass, P0 escape 0.0 over 194
  Shield-on episodes, fail-safe correctness 0.823) was flown on a 3 s / 0.5 s
  forecast the grant does not describe; on the grant's 5 s / 0.1 s with the
  closed-loop wind model it reads as above.

#### Not fixed (found, for the FSM and Shield work next)
- When a time window opens over the aircraft and no action clears the
  forecast, the Shield flies its best effort out with the P0 violation on it:
  6 ticks over the library's three time-window-opens-on-vehicle cells, all
  known failures. This is why the library's P0 escape headline is no longer
  0.0.
- The FSM can hold LOITER on a spot about to become illegal, or one a sweep
  returns to; a hold can drift through a zone that clears before any Loiter
  -> RTL edge; G6 lands in place, which can be inside a zone.

#### Tests
- New test files: `test_harnesstruth_{forecast,rtl,wind,bundles,ci}`,
  `test_railinputs_{fence_export,clock,policy,manifest,node,hotapply,perception}`,
  `test_shieldcore_{repair,audit,identity}`,
  `test_wp4outputs_{kpi_report,exporters,harness}`; each failed on a pristine
  copy of 4418ae0 and passes now. Deliberate defects applied to copies of
  the code: harness 38 (36 caught, 2 equivalent), rail inputs 45, Shield 30,
  WP4 outputs 48, all caught. `tests/fixtures/` lets
  the 8 Oct regression run without `demo/out`.
- Full suite on Python 3.10: 2526 of 2536 over 92 test files. On 3.11 the
  guardrail package's tests pass (2494 of 2534 overall: demo tests that need
  cv2, Pillow or projectairsim cannot run there, plus visible skips). On
  3.10 the ten not passing are nine visible skips (FastAPI is not installed
  in that environment; one sweep test) and the ITRI deck-pack check, which
  passes once the deck is rebuilt from a clean tree.

### 2026-10-12 (later) — CI green: a clamp that repaired rounding errors, and a timing test fit for shared runners

GitHub's test workflow had failed on every push since 8 Oct, so its smoke
step never ran. Two causes, both fixed here.

#### Fixed
- **SpeedClamp logged "repairs" of float-rounding excesses.** The kinematic
  check allows 1e-9 over a cap, but the repair operator compared strictly, so
  an action another operator had already scaled onto the cap and that came
  out at 4.000000000000001 m/s was "clamped" to 4.0 and logged as a repair.
  Whether that happened depended on the platform's floating point: on Windows
  nfz-head-on logged 82 such repairs, on the Linux runners none, so
  `tests/test_sweep.py`'s pinned repair counts held here and failed on
  GitHub. The clamp (and the FSM's mirror of it, `KinematicCaps.clamp`) now
  has the check's tolerance. Repair counts move (nfz-head-on 346 -> 264,
  corridor-curfew-in-hours 412 -> 300, standoff-reclassified 360 -> 300);
  every final position, status and fail-safe figure is unchanged, and
  `docs/data/scenario_sweep.json` is republished with only `repair_count`
  moving (97 of 195 episodes).
- `tests/test_shield.py`'s 50-rule timing test asserted the desktop's 5 ms
  budget on GitHub's shared runners, where the 48-moving-zone path measured
  5.71 ms. On a runner its bound is now 15 ms, still well under the 31 ms of
  the per-fence loop it guards against; the 5 ms bound stays for the
  desktop. Its docstring claimed a margin of at least 10x; that holds for the
  static and window-edge samples (0.36 ms), not for the moving zones (2.6 ms
  on a quiet desktop, so about 2x). The same code at 10451bc measures the
  same, so this is no regression, but it is a risk for the slower Orin.

#### Tests
- New: `test_a_float_rounding_excess_over_a_cap_is_not_a_repair`
  (`tests/test_shield.py`), failing on the strict clamp. `tests/test_sweep.py`
  89/90 (1 visible skip) with the Linux runner's repair counts.

### 2026-10-12 — the 8 Oct follow's defects fixed in replay, the compiler and harness conform, gRPC over mTLS, the Orin defined for JetPack 6.2

Work of 9-12 Oct, in six packages and four integration packages, each
challenged by a reviewer who broke its tests on purpose, then fixed, and one
re-fly of the perception node with all of it. The Jetson Orin and the drone
are not available, so no hil or flight run exists.

#### Flown
- **The perception node re-flew the 8 Oct red-car follow, 120 s, with these changes**
  (`demo/out/pas_ap_20261012_033329`, Project AirSim + ArduPilot SITL over
  pymavlink, none of the grant's topologies, not KPI-grade): 1098 ticks at
  9.14 Hz, detector 4.26 Hz, 43 Shield interventions (yaw caps and building
  clearance), 0 escape ticks, 0 GeoFence breaches, ArduPilot real-time factor
  0.962, EKF p95 0.62 m. The FSM went Normal -> Brake 3 times and back each
  time; it never reached Loiter, and the altitude stayed at 7.99-8.15 m (on
  8 Oct, LOITER spells took it down to 0.36 m).
- What this run does not show: the car was tracked on 129 ticks (269 on
  8 Oct) and never came nearer than 16.5 m, so the 5 m stand-off ring was
  never approached. Its 0 escapes do not test the range-0 fix, and the BRAKE
  hold was never commanded in flight; both rest on the replay and the SITL
  probe above.

#### Fixed
- **The five P0 escapes of the 8 Oct follow** (`docs/FINDING-the-escape-that-stayed-inside-the-ring.md`).
  On every escape tick the perception node had placed the car under the
  aircraft (a depth return shorter than the camera's height became a range
  of 0), and at range 0 the stand-off check saw no action opening the range.
  The check now reads horizontal speed as the opening speed at range 0,
  `StandoffRecover` leaves along the commanded heading, and the node refuses
  depth returns the camera geometry rules out. A replay of the 1101 logged
  ticks gives 0 escapes (the old Shield reproduces the 5).
- **Over-escalation**: `StandoffHold.magnitude_m` is the forecast's closest
  approach inside the ring; on the replayed flight Brake -> Loiter falls from
  8 to 2. X6 leaves a recovery alone while the illegal depth is shown to
  fall (least-squares slope), a new edge X7 sends a recovery that loses
  ground to RTL, and N-in-T counts at most one onset per second.
- **The FSM's Loiter is flown as ArduCopter BRAKE** on every rail
  (`--hold-mode LOITER` keeps the literal mode): with no RC input, LOITER fell
  from 15 m to the ground in 6.7 s in SITL while BRAKE held 14.97-15.01 m.
  Mode requests are recorded in the FSM's and in ArduCopter's words, so a
  pilot's takeover after a hold is no longer read as the node's own.
- `demo/run_demo.py` died on its first shielded tick (it assigned the
  read-only `AuditLogger.policy_hash` since 26 Aug), and `--dynamic`
  hot-applied a fixed polygon fence, refused since 7 Oct; it now hot-applies
  the zone as a `dynamic_nfz`.
- The ArduPilot GeoFence export no longer uploads a zone that a clock can
  switch off (a `valid_time` window or a `time_window_switch`).
- `EpisodeRunner` no longer hangs after a tick raises.
- `scripts/run_pas_ardupilot.ps1 -Step check` reported mavlink-router as
  missing and the MAVProxy fallback as in use, although the router built into
  `~/.local/bin` is what the rail starts; it now looks there too.

#### Added
- **The three hot-apply rule types compile** (templates for `dynamic_nfz`,
  `time_window_switch`, `corridor_swap`, `circle_fence`); a policy file with
  a switch or a swap flies. A file declaring a `dynamic_nfz` is still held
  for flight (`models.FLIGHT_HELD_TYPES`): the rails count it now, but offline
  analysis and the BC pilots' features read polygon fences only.
- **Cost-map adapter** (`compiler.rasterise_cost_map`,
  `docs/DESIGN-cost-map-adapter.md`) with the grant's conventions; 0 missed
  under corridor_outside = 50 on 24 shipped policies; no planner reads it.
  Replay bundles carry each generation's map; `--cost-map` on both SITL rails
  and `demo/real_vla_demo.py`.
- **Both hashes in every bundle** (ours and the PI reference's); a forged
  reference entry is refused 29/29; no stored hash moved.
- **The stress harness's gRPC control plane over mutual TLS**
  (`guardrail/harness_service.py`, `guardrail/protos/harness.proto`,
  `tools/harness_client.py`, `docs/DESIGN-harness-grpc.md`): StartEpisode,
  HotApply, EmitEvent, GetStatus, EndEpisode on the headless harness, with a
  lab root CA meant to be shared with SROS 2 (not yet enabled).
- **One zone-dwell count on every rail** (`guardrail.replay.zone_dwell`).
- **The Orin deployment made concrete for JetPack 6.2** (Jetson Linux
  36.4.3): ROS 2 Jazzy in an Ubuntu 24.04 container so rclpy runs on Python
  3.12 (the grant asks 3.11+), the VLA image on an r36.4 base,
  bitsandbytes built for sm_87, and `deploy/orin/first_boot_check.sh` for the
  person setting the Orin up. Nothing has been built or run on an Orin.
- `tests/test_tracker_content.py` checks the local tracker's content when it
  is present.
- The perception node logs, per tick, the depth range inside the box and the
  rule that refused it (`subject_depth`, `depth_check`).

#### Found
- On the re-fly, 43 depth returns were refused and the car ranged by its box
  width instead, at 56-156 m. On the 31 of them with a depth-ranged tick
  within 5 ticks, the two ranges disagree by up to 122 m (the width range
  farther on 22, nearer on 9). The log kept neither the refused depth nor the
  rule that refused it, so whether the refusals were right cannot be told
  from this run; the next flight logs both. The depth image is also
  quantised to 1 m (the simulator does not honour the float encoding asked
  for).

#### Changed
- The public progress site is re-curated to 12 Oct (Gate G1 with its numbers
  and frame, and this entry's progress); README and `docs/index.md` scope the P0-escape figure to the 41
  mid-evaluation flights and state the 8 Oct follow's 5 escapes.
- `docs/data/scenario_sweep.json` is republished on today's FSM: 152 pass,
  20 tracked, 23 known failures, 0 fail; P0 escape 0.0 over 194 Shield-on
  episodes; fail-safe trigger correctness 0.823 (33 false, 0 missed), below
  the never-trigger null 0.968 and the grant's 0.99.
- `policies/policy_dsl.schema.json` regenerated: only the hot-apply types'
  descriptions changed.

#### Retracted
- The escape repair did not leave the ring in one tick (8 Oct entry): on all
  five ticks the node had placed the subject at the aircraft's own position
  (range 0), so no action could open the range.
- Brake to Loiter seven times (8 Oct entry): the log has eight Brake -> Loiter
  edges, five on stand-off repairs and three where no operator converged.
- Dev topology on one desktop (8 Oct entry), for the follow run: it commanded
  ArduPilot over pymavlink, which is none of the grant's topologies.
- The published library sweep's fail-safe correctness of 0.747 was measured
  on the 10451bc FSM; republished on today's FSM it is 0.823.

#### Tests
- New: `tests/test_flight_pas_ap_20261008.py` (replays the 8 Oct log on both
  theta readings; SKIP without the log), `tests/test_harness_service.py`,
  `tests/test_tracker_content.py`, and cases in `test_shield`, `test_fsm`,
  `test_csp`, `test_sweep`, `test_sitl_rails`, `test_replay`,
  `test_pas_ardupilot` and others. Each package's new tests were shown
  failing on the code before its fix, and the reviewers' deliberate defects
  were caught.
- Full suite on Python 3.10: 2291 of 2296 over 74 test files. On 3.11 the
  guardrail package's tests pass (2254 of 2294 overall: demo tests that need
  cv2, Pillow or projectairsim cannot run there, and five skips are visible).
  On 3.10 the five not passing are four visible skips (FastAPI is not
  installed in that environment; one sweep test) and the ITRI deck-pack
  check, which passes once the deck is rebuilt from a clean tree.

### 2026-10-08 — Gate G1 passes: ArduPilot flies Project AirSim, Mission Planner sees it

The first runs of the perception rail the PI asked about on 6 Oct: ArduPilot
SITL as the flight controller on Project AirSim's physics and sensors,
through the simulator's native `ardupilot-api` controller, with
mavlink-router fanning MAVLink out to Mission Planner and the perception
node. Dev topology on one desktop; not KPI-grade.

#### Flown
- **Gate G1 passed** (`demo/out/g1_20261008_223259`, the four conditions of
  the reference fallback-gates page, three runs, same seed): ArduPilot's EKF
  tracked the simulator's position with a p95 error of 0.81-0.82 m (limit
  1.0 m; a frozen-position null gives 18.9-19.0 m); its PWM lifted the
  vehicle 14.5 m from rest; each AUTO square mission (10 m sides, 15 m up)
  reached all 5 mission items with no airborne collision and landed within
  0.2 m of take-off; ArduPilot ran at 0.958-0.967 of real time. Frame:
  `docs/img/g1_pas_ardupilot_airborne.png`.
- **Perception node, red-car follow, 120 s** (`demo/out/pas_ap_20261008_224440`):
  camera, detector, Shield and escalation FSM commanding ArduPilot. 1101
  ticks at 9.16 Hz, detector 3.75 Hz, 85 Shield interventions, 0 GeoFence
  breaches, ArduPilot real-time factor 0.951. **5 ticks let a P0 stand-off
  violation through** (escape rate 0.0053): each time the aircraft was
  already inside the stand-off ring, stopping was illegal, and the escape
  repair did not leave the ring in one tick. The FSM went Brake to Loiter
  seven times because stand-off repairs of 3.6-9.8 m exceed theta (2.0 m).
  Both are open defects.
- **Mission Planner's port carried the flight**: the router's UDP stream to
  Windows port 14550 delivered GLOBAL_POSITION_INT and ATTITUDE at 19.0 Hz
  for the whole 120 s mission, largest gap 0.1 s (`sitl/gcs_listen.py`).

#### Fixed (each found on the rail on 8 Oct)
- A Windows client could not reach the router: mavlink-routerd listens on
  `[::]`, and WSL's NAT localhost forwarding does not carry an IPv6
  listener to Windows' 127.0.0.1 (measured: refused on 127.0.0.1, accepted
  on the WSL address). Under NAT the node and G1 now connect to WSL's own
  address (`NetworkPlan.router_tcp_url`).
- After a G1 run's ArduPilot stopped, the next scene load never returned:
  the steppable clock waits for ArduPilot's PWM. The simulator is now
  restarted for each G1 run.
- ArduPilot refused to arm ("Main loop slow (222Hz < 300Hz)"): CityLife
  delivers about 222 sensor frames a second against the 300 Hz loop
  airsim-quadX.parm sets, and 3 ms steps at that rate ran the clock at about
  0.67 of real time. The scene steps 5 ms and the param file sets
  SCHED_LOOP_RATE 200.
- The G1 square at 10 m flew through a street light (48-60 collisions a
  run); the passing runs fly it at 15 m.
- `start_ardupilot.sh` and `sitl/start_router.sh` now find a mavlink-routerd
  installed without sudo in `~/.local/bin`; the runner no longer exits 1
  when nothing is left to stop.

#### Changed
- The ITRI deck's Project AirSim slide states the G1 result from the newest
  verdict file (`tools/deck/build_progress_oct_data.py`, section `g1`).
- Installed for these runs: pymavlink and grpcio in the flight environment,
  cryptography, jsonschema and psutil in the 3.11 environment, jinja2 in the
  WSL ArduPilot venv, and mavlink-routerd built from source (2362c62) into
  `~/.local`.

#### Tests
- `tests/test_pas_ardupilot.py` 97/97 (the router-address test fails on the
  old code); `tests/test_build_progress_oct_data.py` gains a G1 case.
- Full suite on Python 3.10: 2075 of 2081 over 70 test files. The six not
  passing are four visible skips (FastAPI is absent in that environment; one
  sweep test) and the two deck-pack checks, which pass once the deck data and
  the pack are rebuilt from this commit.

### 2026-10-07 — the grant's own policy form, a Shield that escalates, ArduPilot on every rail

Nine packages following the PI's answers of 6 Oct: the KPI campaign moves to
the Jetson Orin (the grant's hil topology), the perception rail follows best
practice on Project AirSim, and deviations are closed by conforming to the
grant. Each package was challenged by an adversarial reviewer and fixed.
Short SITL runs were flown in WSL (dev topology, not KPI-grade); Project
AirSim, Unreal and any GPU model were not started.

#### Added
- **The grant's policy form** (`guardrail/models.py`, `guardrail/bundle.py`):
  the grant's demo bundle and the Policy DSL page's worked example load as
  written (geometry blocks, grant field names, scope, layer, altitude_ref,
  issued_at, lat/lon without origin). All nine grant rule types are
  declarable; circle fences are enforced. Layered policies
  (regulation/site/mission) merge and refuse any override that loosens a
  lower layer's hard rule. Bundles cross-load with the PI's reference loader
  in both directions (3/3). `tools/geo_to_policy.py` converts GeoJSON/KML and
  ingests REST payloads.
- **Escalation in the Shield** (`guardrail/shield.py`): every `filter()` tick
  feeds the escalation FSM (Brake, Loiter, RTL, Land); repairs report their
  size, so theta (2.0 m / 0.5 m) is applied; breach actions, hard/soft and
  priority now change behaviour.
- **Mid-flight events**: moving no-fly zones (spawn, move, translate, rotate,
  scale, expire), time-window switches and corridor swaps, each bumping the
  generation, checked against the layer rule, and exposed over REST
  (`guardrail/api.py`). Hot-applying a fixed polygon fence after take-off is
  refused, as the grant locks that class.
- **One rule checker**: `Shield.rule_status()`; the policy HUD and FenceGuard
  read it and their own geometry is deleted.
- **The grant's ROS 2 interface**: body-frame actions end to end, a separate
  `sitl/mavlink_adapter_node.py` that converts to local NED after projection
  and publishes `mavros_msgs/PositionTarget`, ROS 2 packages with launch files
  (`sitl/ros2_ws/`), a MAVLink router config for Mission Planner (UDP 14550)
  beside MAVROS, the ArduPilot GeoFence as backstop (`sitl/fence/`), and per
  episode a fresh audit file, a CSP per policy generation, an events log and
  a rosbag2 recording.
- **Project AirSim flown by ArduPilot** (`demo/pas_ardupilot/`,
  `scripts/run_pas_ardupilot.ps1`, `docs/DESIGN-projectairsim-ardupilot.md`):
  Project AirSim's native ArduPilot controller (`ardupilot-api`, ArduPilot
  SITL with `-f airsim-copter`), a one-process perception node (camera,
  detector, Shield, FSM, ArduPilot), Mission Planner through the router, and
  a script that records the four Gate G1 checks. Built and tested offline;
  not yet flown.
- **Jetson Orin portability** (`deploy/`, `docs/RUNBOOK-orin-hil.md`,
  `docs/DESIGN-topologies.md`): one image definition for dev, hil and flight
  where only a topology file differs; `tools/profile_shield_tick.py` (the
  grant's tick budgets on real flight ticks, runnable on the Orin unchanged)
  and `tools/vla_backend_table.py` (one protocol to compare VLA backends).
  No image has been built yet.
- **Stress harness**: every grant event type runs mid-flight; RTL and Land
  outcomes occur; paraphrase arms and a prefix on/off key; one YAML per
  template under `experiments/templates/` (16 files); a broad profile
  (15,025 episodes, defined, not run); a per-template KPI report.
- `tools/check_claims.py`: fails when a withdrawn claim is stated again;
  `tools/prefix_eval.py` (WP2-09 offline replay, not yet run on the model).
- A public progress site, preview only (`docs/progress/`,
  `tools/build_public_site.py`): 21 pages in English and Traditional Chinese,
  built from an explicit allow-list of fields. Not published.
- Finding notes for the silent defects found on 6-7 Oct (`docs/FINDING-*.md`).

#### Changed
- WGS84 is the canonical stored form for geographic policies; 28 of 29 policy
  hashes are unchanged and the moved one keeps its old hash verifiable.
- Policy validation is strict: the negative corpus goes from 14/25 to 35/35
  refused (10 cases added).
- The Shield's default lookahead is the grant's 5 s at 0.1 s, with checks on
  both sides of every time-window edge. At the grant's load (50 rules, 50
  poses) a check near a fence takes 0.39 ms and `filter()` 2.5 ms on the
  development desktop.
- The topology label accepts `hil` and `flight` only on evidence (an Orin
  host, the VLA host, the router layout), and records the ArduPilot version.

#### Fixed
- A window ending 17:30 stayed in force until 17:30:59.
- A mission-layer event could switch off a hard regulation-layer zone.
- The HUD showed rules the Shield did not correct as "corrected".
- A SITL tick the autopilot held was scored as a converged repair.
- A shield-off tick counted as a P0 escape even when nothing was flown.
- `parse_command("altitude: (40, 40)")` read the target as the altitude.
- `experiments/bench_shield_50rules.py` compares the fence code with the
  escalation FSM off on both sides, and times the Shield with it on.

#### Retracted
- The `guardrail/bundle.py` docstring said the two implementations could read
  each other's bundles. Until 7 Oct only the tar.gz container matched.
- The sweep's fail-safe figure (triggered = any brake). Measured with the FSM
  it is 0.747 (library), 0.726 (nightly) and 0.875 (smoke), below the grant's
  0.99 and below the never-trigger null.
- The sweep rollup published the P0-acted tick share (1.0) under the fail-safe
  KPI's name; it is the escape rate restated.
- The fine-tune result "100 % reached, efficiency 0.996" was stated without
  its goal assist: the run flew with goal_blend 0.55, under which the
  original adapter also reached 5/5 (0.942); without the assist the
  fine-tune reached 0/5 and 1/5.
- `docs/data/deck_sept_extra.json` carried stored detector rates (3.76, 5.15,
  4.03 Hz); the mission rates are 2.77, 4.06 and 3.11 Hz.
- The September deck's detector rate sentence used the stored 5.15 Hz; the
  mission rate is 4.06 Hz.
- The August midterm report's "the detector half of the threshold is met on
  every scenario" used stored rates; the flights on disk read 3.02-3.11 Hz.
- "Detections arrive at roughly 4 Hz while the control loop runs at 10 Hz":
  the detector ran 3.02-3.11 Hz and the loop 7.5-8.4 Hz.

#### Tests
- New test files: `test_policy_dsl_grant_form`, `test_geo_to_policy`,
  `test_shield_events`, `test_api`, `test_mavlink_adapter`,
  `test_ros2_package`, `test_pas_ardupilot`, `test_deploy_configs`,
  `test_profile_shield_tick`, `test_vla_backend_table`, `test_check_claims`,
  `test_prefix_eval`, `test_build_public_site`, `test_build_progress_oct_data`
  and others.
- Full suite on Python 3.10: 2073 of 2078 over 70 test files. The five not
  passing are four visible skips (FastAPI is not installed in that
  environment; one sweep test) and the ITRI deck-pack check, which passes once
  the deck is rebuilt from a clean tree. On 3.11 the guardrail package's tests
  pass, with visible skips only where an optional dependency is absent
  (cryptography, jsonschema, tokenizers, psutil); demo tests that need cv2,
  Pillow or projectairsim cannot run there (2032 of 2076 overall).

### 2026-10-06 — ten work packages toward the grant's locked spec

Ten packages taken from the re-verified audit, built in parallel on disjoint
files, then each challenged by an adversarial reviewer (tests reverted or
mutated on purpose) and fixed. Nothing here was flown; no simulator, SITL or
GPU model was started.

#### Added
- **Shield speed: a compiled fence index** (`guardrail/ir.py`). Each fence's
  margin ring is built once and a shapely STRtree answers which fences a
  forecast can reach. At the grant's 50-rule load (48 fences), on this desktop:
  `_check` near a fence 14-33 ms -> 0.13-0.26 ms median; `filter()` near a
  fence 172 ms median (p99 255 ms) -> 0.55-1.0 ms (p99 1.0-1.8 ms), against the
  grant's 5 ms check and 100 ms tick. `Shield.history` keeps the last 50 ticks
  (Safety Shield, sliding-window buffer). `experiments/bench_shield_50rules.py`
  reproduces the numbers. Not measured on a Jetson Orin.
- **Typed Constraint Summary Pack** (`guardrail/csp.py`, `guardrail/templates/`,
  `guardrail/csp.schema.json`). The 14 locked fields; time and region
  filtering; risk grading 0.5/0.3/0.2; P0 always kept, P1/P2 cut by risk inside
  a configurable 256-token budget; `CSPBudgetExceeded` when P0 alone is over;
  per-rule reasons; Jinja2 sentence templates; the action-set adapter; coarse
  and fine geometry with `geometry_ref`. `ConstraintCompiler.compile_csp()`
  returns it; `build_prompt()` keeps all its keys.
- **The CSP can reach a VLA**: `demo/real_vla_demo.py --csp on|off` (default
  off) puts the CSP before OpenVLA's instruction, logs the exact prompt, its
  token count, CSP hash and policy hash per inference, and refuses a prompt
  over budget before the model loads. 54-106 OpenVLA tokens over 29/29
  policies. Not yet flown.
- **Paraphraser** (`guardrail/paraphraser.py`, `experiments/paraphrases/`):
  112 free-form paraphrases (14 instruction sets x 8), generated once by
  claude-opus-5-5 with the prompt recorded, a seeded template fallback, and a
  validator that refuses a paraphrase changing a mission slot. Measured false
  accepts: 0/28 on the review set, 5/30 on a held-out set it was never tuned
  on.
- **Escalation state machine** (`guardrail/fsm.py`): Normal -> Brake -> Loiter
  -> RTL -> Land with the grant's defaults (N=3 violations in T=5 s, theta
  2.0 m lateral / 0.5 m vertical, T_recover 2 s), the mapping from
  violation_action, hard/soft and priority, and the fail-safe trigger
  correctness scorer with its always-trigger and never-trigger nulls. Pure
  module; not yet wired into the Shield.
- **Stress harness groundwork** (`guardrail/scenario_spec.py`,
  `experiments/profiles/`): ScenarioSpec and ScenarioEvent copied field for
  field from the Stress Testing page, expected-outcome and expected-fail-safe
  labels, seeds, a smoke profile (52 scenarios x 1 seed) and a nightly profile
  (211 x 3 seeds), stressors for all five Shield smoke-matrix rows plus wind,
  gusts, start jitter and GPS dropout, and a six-field manifest per episode.
  Smoke: 52 episodes in 15.9 s; nightly: 633 episodes in 223 s; both headless
  and never KPI-grade. `.github/workflows/tests.yml` (never run on GitHub).
- **Policy identity** (`guardrail/models.py`, `guardrail/bundle.py`):
  Ed25519-signed bundles (lab development key; private half gitignored),
  pure-Python verification so the 3.11 and SITL environments verify without
  `cryptography`, `issued_at` in the manifest, `policies/policy.lock.json`
  pinning one hash per `policy_id@version`, `--bundle` on
  `sitl/ros2_shield_node.py`, `sitl/run_sitl_demo.py` and `demo/follow_vlm.py`,
  `tools/wp1_roundtrip_kpi.py` with 25 negative policies, and
  `policies/policy_dsl.schema.json`.
- **KPIs from logs on disk** (`guardrail/kpi.py`, `tools/kpi_report.py`):
  repair success with the theta cap, mission success that requires the goal
  and no P0 unsafe position, repair count per episode and family, false
  triggers, and a rollup that never prints a zero for "not measurable".
  `docs/data/kpi_rollup_2026-10-06.md` covers 742 episodes, 0 KPI-grade.
- `pyproject.toml` for the guardrail package (requires Python >= 3.11);
  `sitl/setup_sitl.sh` pins ArduPilot to Copter-4.5.7 (2a3dc4b7) and gains a
  read-only `--verify`. `docs/DESIGN-python-versions.md` records what runs
  where.
- Design notes: `docs/DESIGN-{escalation-fsm,paraphraser,policy-identity,
  prefix-compiler,python-versions}.md`.

#### Changed
- `Policy.policy_hash` is the full 64-hex SHA-256 over canonical JSON with
  None omitted (`sha256-canonical-v2`), so adding an optional field no longer
  moves every hash. Old 16-hex forms still verify: stored manifests traceable
  to a policy 34/76 -> 76/76.
- The topology label `canonical-hil` is now written as the grant's word,
  `dev`. `is_kpi_grade()` refuses `dev` runs without a written PI waiver, so
  stored KPI-grade runs go from 5 to 0 until the hil runs exist.
- Time-windowed rules now carry their window in the prompt ("in force
  Mon-Fri 07:30-17:30") and drop out of the CSP outside it; before, the
  model was told they were permanent.
- `load_bundle` refuses unsigned bundles unless `require_signature=False`.
- README, `docs/index.md`, `docs/scope-clarification.md`,
  `docs/prof-repo-study.md`, `docs/CHECKLIST-remaining-work.md` (now marked
  superseded by the 5 Oct audit) and the progress note no longer state the
  claims retracted below.

#### Fixed
- `demo/real_vla_demo.py` sent OpenVLA's forward/right deltas straight into
  North/East: facing East, "forward" flew North. They are rotated at the
  capture heading now, and the loop reads the heading from the pose.
- A stripped or forged bundle signature could read as "unsigned" or
  "unverifiable here" and fly with `--allow-unverified-bundle`; both are now
  refused as bad-signature.
- A Shield-off control arm's counterfactual repairs scored as a measured
  repair success of 0.0; they now read `shield off`.
- `setup_sitl.sh --verify` reported PIN OK for an edited file inside a
  submodule, and for a crashed `git status`.
- `--backend sitl` in the sweep passed an option the SITL scripts reject.

#### Retracted
- "canonical HIL" for the desktop SITL + MAVROS 2 rail: it is the grant's
  `dev` topology. KPI numbers count from `hil` (Jetson Orin) runs.
- "P0 escape 0.0 on all 41 shielded flights" as the contract KPI: the 41 are
  34 Project AirSim + 4 SITL/pymavlink + 3 SITL/MAVROS 2 flights, and none is
  a contract KPI figure.
- Fail-safe trigger correctness target "1.0": the grant says >= 99 %.
- The in-flight detector rate on the five flights the mid-evaluation quoted:
  3.76-5.22 Hz -> 2.77-4.31 Hz, recomputed over the mission.
- "11713/11713 repairs converged" as the grant's repair success KPI: it is the
  Shield's own re-check; theta could not be applied to the 7660
  theta-governed ticks because no log records a repair size in metres.

#### Tests
- New: `test_ir` 14, `test_csp` 65, `test_real_vla_prompt` 43,
  `test_paraphraser` 82, `test_fsm` 95, `test_scenario_spec` 29,
  `test_policy_hash` 24, `test_wp1_roundtrip_kpi` 6, `test_kpi_report` 31,
  `test_env_pins` 34. Extended: `test_shield` 39, `test_sweep` 63,
  `test_kpi_magnitudes` 65, `test_bundle` 51, `test_manifest` 39,
  `test_replay` 22.
- Full suite on Python 3.10: 1408 of 1408 over 53 test files. On 3.11 the
  guardrail package's tests pass; the demo tests that need `cv2` or
  `projectairsim` cannot run there.

### 2026-10-05 (later) — a tracker website, and an update log on it

Record: the tracker's own Updates page (`tracker/data/updates.json`, entry #15).

#### Added
- `tracker/`: a local website that explains the project against the grant and tracks it. It is served by `tracker/serve.py` (Python standard library, 127.0.0.1:8765, HTTP Range so videos seek) and started with `tracker/start_tracker.bat`. It has seven pages in English, Indonesian and Traditional Chinese:
  - an overview;
  - clickable diagrams of the pipeline as built and as the grant specifies it;
  - the grant's work packages, KPIs, topologies and timeline;
  - a kanban board of the 148 audit items;
  - examples with what they do and do not show;
  - a glossary;
  - an Updates page.
- Board state (column, notes, history) is written to `tracker/data/state.json`. The file is gitignored as personal.
- Updates page: a versioned log, `tracker/data/updates.json`, maintained with `tools/add_update.py` (`new`, `add`, `check`, `list`, `extract`).
  - `seq` numbers the site's updates.
  - `version` copies a CHANGELOG release, or `<release>+<date>` for unreleased work, so no project release is invented.
  - It holds 15 entries: 12 backfilled from this file's sections, one from git for the 14-16 Sept work this file does not cover, and the two 2026-10-05 entries.
- Board activity: card moves and note edits with their times, from the board state.

#### Fixed (found while designing, each shown failing before the fix)
- `/data//_batches.json` and `/data/./_batches.json` served the site's working files. The hidden-file check ran on the raw path, and the static handler normalises it afterwards; the check now runs on the normalised path.
- A non-numeric `Content-Length` on a board save dropped the connection. It now gets a 400.
- A failed board write (a file held by OneDrive) left a temp file behind and dropped the connection. The temp file is now removed, the answer is a JSON 500, and the page keeps the board in the browser.

#### Fixed (found in the screenshots taken for this entry)
- Checklist: each column head was meant to stay at the top of the page while scrolling. The board scrolls sideways, so the head stuck 56 px down inside its own column instead and covered the first card. The board now scrolls in its own box (the window height minus 160 px, at least 360 px), and the heads stick to the top of that box.
- Updates timeline: releases a day apart (0.2.0 to 0.5.0, 7 to 10 Sept) printed their labels over each other. Each label now sits on the lowest free row, up to four rows.

#### Tests
- `tests/test_tracker_serve.py`: 15 cases, all pass.
- `tests/test_add_update.py`: 20 cases, all pass. One of them runs `check` on the real log.
- Full suite: 847 of 847 pass, over 43 test files. `python tools/add_update.py check` reports 0 problems.
- Checked in a browser against a second server on port 8766 with a scratch board file, so the real `tracker/data/state.json` was not touched:
  - the content API returns 148 cards and 15 updates, with no load error;
  - a board save reads back, a video answers a byte range with 206, and hidden working files answer 404;
  - moving a card and writing a note show on Board activity as 1 move and 1 note;
  - the Release filter shows 6 entries, and a link to one update (`#updates?v=12`) opens it;
  - all seven pages fit a 375 px phone screen, and requests go only to the local server.
- Screenshots in `docs/img/tracker/`: 27 desktop views (nine pages or states in three languages) and three phone views.

### 2026-10-05 — the grant audit, and claims corrected against it

Record in Indonesian: `docs/AUDIT-KONTRAK-2026-10-05.md`. Data: `docs/data/grant_audit_2026-10-05.json`.

#### Found
- Every obligation in the seven grant pages (`reference/*.pdf`) was audited, with the PI's reference design docs as the detailed spec: 136 items.
  - Result: 6 done, 56 partial, 39 missing, 35 drift (built differently from the grant, or a claim about it does not match).
  - Method: five auditors (WP1 to WP4, and architecture), each challenged by a skeptical verifier, and a completeness critic who reread every page.
- Final-gate items with no artefact:
  - perception-rail integration;
  - the KPI report from stress testing in the hil topology;
  - the Paraphraser (a Q3 deliverable);
  - the stress-test harness;
  - the signed final report.
- Prefix Compiler: the CSP is built but never put in any VLA's prompt, and its two KPIs were never measured.
- WP3 Shield:
  - There is no escalation beyond Brake (no Loiter, RTL or Land) and no ArduPilot GeoFence backstop.
  - `fail-safe trigger correctness` in `guardrail/kpi.py` restates the escape KPI.
  - `mean time to safe` has no Shield-on value.
  - The stored KPI numbers (runs of 28 to 31 Aug) were not re-measured after the Shield changes of 1 and 29 Sept.
- Deviations from the locked spec:
  - a 3 s lookahead against the grant's 5 s;
  - world-frame actions against the locked body frame;
  - `violation_action` limited to repair and brake;
  - no spatial index (14 to 36 ms per check with 50 polygon rules, against a 5 ms budget).
- `docs/CHECKLIST-remaining-work.md` (last updated 8 Sept) is out of date: it lists two open items.

#### Retracted
- "The P0-escape count on screen is the contract KPI" (`docs/GAMBARAN-SISTEM.md`, 3 Oct). The HUD uses the KPI's definition. But a Project AirSim flight is not a contractual KPI number: the grant takes KPIs only from stress-test runs in the hil topology.
- "Most compliance is done by a pilot that knows the rules, and the Shield is the backstop, exactly as the contract designs it" (`docs/GAMBARAN-SISTEM.md` and the `demo/policy_hud.py` docstring). In the grant, upstream compliance comes from the Prefix Compiler's CSP in the VLA prompt, and the backstop behind the Shield is ArduPilot's GeoFence. FenceGuard is our addition.
- "Gazebo: no artefacts in the repository" (`docs/GAMBARAN-SISTEM.md`). The scripts exist (`sitl/setup_gazebo.sh`, `sitl/run_gazebo_demo.sh`); no run was kept.
- "The city demos may use a hand-written pilot because the contract makes the VLA backend swappable" (`docs/GAMBARAN-SISTEM.md`). The clause covers VLA backends that output the 4-D action, and it names stubs only for unit tests. The text now says the demos test the Shield, not a constrained VLA.

#### Changed
- `docs/GAMBARAN-SISTEM.md` (and its PDF) corrected as above. The `demo/policy_hud.py` docstring was reworded.

### 2026-10-03 — the policy on screen, a no-fly zone in the city, and the share pack

Record in Indonesian: `docs/WORKLOG.md` (the 2026-10-03 entry). Asked for at
the 2026-09-30 meeting: show on screen when a guardrail rule acts, and send the
material for ITRI's monthly report.

#### Flown
Red car (`Car_10`), 240 s, `--identity`:

| | policy | cruise | within 30 m | estimate on the car | zone entered | min clearance | detector (mission) |
|---|---|---|---|---|---|---|---|
| `citylife_redcar_nfz1` | NFZ | 8 m | 0.977 | 0.998 | no | 3.24 m | 2.20 Hz |
| `citylife_redcar_alt10` | default | 10 m | 0.987 | 0.997 | - | 3.21 m | 2.43 Hz |
| `citylife_redcar_alt10_nohud` | default, old HUD | 10 m | 0.994 | 1.000 | - | - | 2.89 Hz |
| `citylife_redcar_nfz2` | NFZ, cached overlay | 10 m | **0.985** | **1.000** | **no** | 3.25 m | 2.79 Hz |

- P0 was 0 on every flight, the Shield made 0 corrections, and there were no
  mission collisions.
- Beside the zone the drone flew x 37.9-41.4. `clear_aim` re-aimed it on 91
  ticks, and it was 21.2 m behind the car at the first corner (24.1 m on
  nfz1, against 42.7 m in the no-re-aim replay).
- At 10 m cruise the altitude stayed 10.0-10.6 m against the 14 m ceiling.
- Drawing the overlay on every 20 Hz frame cost the detector (2.89 -> 2.43
  Hz, flown A/B). `OverlayCache` draws it at 5 Hz and pastes the tiles in
  between, which brings it back to 2.79 Hz.
- All of today's flights ran the detector slower than the 30 Sept flights,
  including the old-HUD one (2.89 vs 3.26 Hz in the mission; start-gate rates
  are equal). OBS Studio and Mission Planner were running today and were not
  on 30 Sept. This is suspected, not tested.

#### Added
- **`demo/policy_hud.py`: the policy indicator.**
  - One row per rule in the loaded policy, with its limit, the value the Shield
    itself measures (the estimator's subject, its obstacle distance field) and
    a status: idle / OK / NEAR / AVOID (the controller steering round it) /
    HOLD / ACTING (the Shield repaired the command) / BREACH (past the limit,
    or a P0 still violated by the command actually sent) / BRAKE.
  - A banner for the most important event of the last 1.5 s, ranked by event.
  - A north-up map with the zones, the aircraft's track and the target
    estimate.
  - Running counts. The P0-escape count uses `guardrail/kpi.py`'s definition.
  - On by default in `demo/follow_vlm.py`; `--no-policy-hud` restores the old
    line. It stays live for the Shield-filtered flight to the landing site and
    says plainly that the vertical descent is outside the Shield.
  - Its totals are written to `metrics.json` as `policy_hud`.
- **`policies/follow_car_citylife_nfz.yaml`**: a part-width no-fly zone on the
  red car's first leg (x 45-62, y 62-92). `run_citylife_follow.ps1
  -PolicyFile` selects it.
- **`FenceGuard.clear_aim()`.** A forward aim point that falls inside a zone's
  band (polygon + margin + stand-off) is moved to the band's edge plus 1 m, so
  a zone beside the subject's path is passed alongside. Without it, a kinematic
  replay with the real gate/slide/Shield left the drone 42.7 m behind the car
  at the next corner (16.1 m with no zone); with it, 18.7 m, with the zone never
  entered. `fence_aim_ticks` in metrics.
- `FenceGuard.last_cause` ("fence" / "obstacle"), logged per tick as
  `fence_cause`.
- `Shield.subject`, `Shield.subject_class`, `Shield.clearance_at()`:
  read-only views for the indicator.
- `frame_t` in `recorder.json`: the flight time each frame showed.
- `tools/rerender_policy_hud.py` redraws the indicator onto a recorded flight
  from its own log. It warns when the obstacle map is newer than the flight.
- `tools/deck/build_flowcharts.py` draws the tracking and whole-system
  flowcharts as SVG and PNG (`docs/img/flowchart_*`).
- `docs/GAMBARAN-SISTEM.md`: system overview in Indonesian.
- `demo/identity.TIER_LABEL`: the identity tiers are shown as REJECT /
  DOUBTFUL / OK. The stored values stay hard / soft / ok.

#### Changed
- The identity tiers are relabelled everywhere a reader sees them: the deck
  builder, the progress note, the HUD box label and the flowcharts. At the
  meeting, "HARD / SOFT" was heard as manoeuvre types, and the grant's Policy
  DSL already uses hard/soft for rule types.
- `nfz_hold_ticks` counts only holds a fence caused. With a fence declared
  anywhere, building holds used to count too.
- The legacy HUD names the hazard from `last_cause` rather than from whether
  the policy has a fence.

#### Fixed (found by a three-lens review with verification, before any flight)
- The banner named the first repair operator, so a P0 escape could read as
  "backing off from target". It now names the last operator acting for the
  violated rules' kind, and an escape outranks everything.
- A controller hold masked Shield repairs, on both the row and the banner.
- No state for "past the limit": rows showed NEAR, and the banner said "all
  rules satisfied".
- Rules the Shield repaired for, but that were not violated by the raw command,
  were not marked.
- Off the obstacle map, clearance read as OK.
- The stand-off binding test differed from `SubjectStandoff.binds`.
- Zone distance ignored the margin and the altitude band.
- The minimap was shifted half a cell.
- The zone warning distance was hard-coded.
- A display exception could abort a flight.
- The HUD froze through the landing.
- `frame_t` flooded the console.
- `-PolicyFile` refused absolute paths.
- Re-renders reused stale frames.

#### Share pack
`docs/share/2026-09-30-ITRI/` (gitignored). It holds:
- the 30 Sept deck as shown (14 slides; tiers relabelled; flowcharts
  redrawn) and its PDF;
- the 16 Sept deck and its PDF;
- the 30 Sept follow video and the before/after clip;
- the 24 Sept reference flight, re-encoded from 166 MB to 22 MB with the same
  duration;
- the flowcharts;
- the progress note;
- a README with the cover note and six corrections.

#### Tests
`tests/test_policy_hud.py` 36/36 (with `OverlayCache`). Full suite 812/812.

### 2026-09-30 — a re-review, pedestrians that no longer get lost, and the red car flown

Record in Indonesian: `docs/WORKLOG.md` (the 2026-09-30 entries).

#### Flown
The red-car mission with `--identity`. The reference flight
(`citylife_redcar_trail`) ended TARGET LOCKED on a pedestrian signal.

| | within 30 m | estimate on the car | lapses / re-acquired |
|---|---|---|---|
| `citylife_redcar_id1` | **0.992** | **1.000** | 0 / 0 |
| `citylife_redcar_id2` | **0.980** | **1.000** | 0 / 0 |
| `citylife_redcar_id3` | 0.495 | **0.996** | 2 / 1 |
| `citylife_redcar_id4` (after both fixes) | **0.970** | **1.000** | 0 / 0 |
| before (7 trail/final flights) | 0.088-0.436 | 0.013-0.697 (as flown) | - |

"Estimate on the car" is the share of ticks with a served estimate that sit
within 6 m of the car. P0 was 0 on every flight, and no flight collided
during its mission. On `_id3` the estimate never moved to anything else. It
lost the car twice. The first time it re-acquired it in 1.35 s. The second
time it saw the car standing at a light 52 m away but never took it back,
because re-acquisition stops at 45 m. That is fixed below.

#### Retracted / corrected
- **The identity gate did not pass held out.** Re-run after the bottom-clip
  guard moved into `identity.features_for` (thresholds unchanged, hash
  `567a2c79dc53`): leave-one-flight-out, 15.2 % of true boxes are not OK,
  over the 15 % gate; in-sample 14.1 % meets it. The 09-29 entry is
  corrected in place.
- **The replay's "old" arm was not what flew.** It reset and re-seeded after
  3 s. Against the estimator as flown the replay reads 22.9 % -> 94.5 % on
  the car, and 210 of 241 wrong first accepts after a gap -> 0 of 12 wrong
  seeds. The 09-29 entry is corrected in place.
- **Pinhole rescore:** `_final1`, `_final3` and `_carpolicy` (a tie) fall
  below chance; `_trail3` and `_far` were below it already.
- **"Nobody standing on a zebra"** (09-29, Simulate 1) was measured between
  two samples of positions only; the per-read check (state and speed) found
  six figures on zebras in WALK - the pure-node bug below.

#### Fixed
- **Pedestrians crossed from the kerb in WALK, with no walk-phase or gap
  test.** A `bind` of a pure node is re-evaluated at every use in Blueprint;
  the point's kind was read after `SetIdx` and so was the NEXT point's.
  Latched into `NKind`; the DSL evaluator in `tests/test_drive_signals_dsl.py`
  now re-evaluates pure binds the same way, and a test runs the old graph
  into the bug.
- **Half the pedestrians froze after ~30 minutes.** Stuck 3 s short of a
  pavement node a figure SKIPPED to the next point; from off its line that
  point was often round a building corner, so it stalled, skipped, stalled,
  until a kerb node sent it to WAIT tens of metres from its zebra, then
  across, through a building, for ever (CROSS had no stuck rule). At
  t = 1933 s, 25 of 40 figures were off their routes while every zebra
  counter read clean. Now a stall within 300 cm of the point is arriving;
  anywhere else the figure keeps its target and steps aside, alternating
  sides; after six detours it is set down on the point. Pavement nodes are
  spread +-25 cm per figure so opposite streams no longer meet dead
  head-on. Simulate over 30 minutes: 0 figures off route at
  t = 1972 s, 0 set-downs.
- **Re-review of the drone side (10 of 11 findings confirmed):** without an
  estimator the lapse anchored where the subject was committed (Reacquirer
  then refused the car as out of reach for good), and SOFT boxes kept the
  lapse clock alive indefinitely; the pedestrian centre height never applied
  ("person" vs the canonical "pedestrian"); a retarget during an inference
  could re-seed the lock with the old subject; `_prior_at` gave a column to
  a point behind or under the aircraft; the Reacquirer's fixed 2 m motion bar
  let a sign with 1.5 m of map-point jitter through on 186 of 200 seeded
  runs (now 2 m + 3 standard errors: 0 of 200, a car still 200 of 200);
  `_pooled_reproduced` could count the `_pooled` row as a flight. Logs:
  `depth_dt_ms` of the depth actually paired, `t_capture` and `stamp` per
  detection.

- **A car in plain view beyond the re-acquisition range was never taken
  back** (`_id3`, 65 "too far" refusals of OK boxes within reach). A
  candidate refused for its range alone is now a far lead: after three
  consistent sightings the search flies toward it (HUD "APPROACHING") until
  the ordinary gate can judge it. The gate still decides.
  `reacquisition.far_approach_ticks` counts the ticks. Replayed on `_id3`'s
  own candidates after the second loss, it fires at t = 126 s, 4.3 m from
  the car, with the drone 51 m away. `_id4` never lost the car, so the
  steering has not flown yet.
- **Landing sites were chosen with no room for drift.** `_id3` reached the
  nearest cell that passed and touched down 0.67 m off it, 2.94 m from an
  obstacle that needs 3.0 m. A site to fly to now keeps every clearance
  1 m wider (`SITE_MARGIN_M`), and the bare rules are tried only when no such
  cell is in reach. Staying put needs the rules alone: the first version
  also asked it of the hover point and sent `citylife_ped_id` 25 m into a
  crowd, where the Shield held it.

- **Both replays swept in the new flights.** `tools/replay_identity.py`
  globbed every `citylife_redcar_*`, so the `_id1..4` flights, whose boxes
  the identity rules had chosen, entered the tuning set and the pipeline
  replay (28,796 -> 37,551 ticks). A flight flown with `--identity` is now
  left out. Both replays reproduce their published numbers exactly, and the
  far-lead change moves neither.

#### Changed
- **`run_citylife_follow.ps1` passes `--identity` only for the car
  mission** (`-Identity` forces it for a person). The identity rules were
  tuned on car flights only. For people they supersede the presence gate's
  0.2-1.5 m width test, and on `citylife_ped_id` person boxes ranged on the
  facades behind them (40-130 m) pulled the estimate within 6 m of anyone on
  only 46 % of ticks. A person-width SOFT rule was tried and withdrawn: it
  demoted 41 % of the boxes near people, with labels too loose to tune it
  on. `--land-site` stays on for both missions.

#### Known limitations
- **Range to a person comes from depth, and a thin box measures the
  background.** The presence gate blocks those boxes, 1,502 of 1,788 ticks
  on `citylife_ped_0930`. That flight was within 30 m 98.2 %, where the
  09-23 flight managed 100 %. Its pedestrians walk their routes now instead
  of pacing. The proposed fixes are the nearest surface in the box or the
  ground ray through the box bottom. Neither is done.
- **`run_follow_vlm.ps1 -Identity` (Demo_day) has no start gate.** Acquiring
  with no anchor means waiting to see the car move, which took 7.1 s. All of
  the 6.4 % outside 30 m fell in the catch-up after it (t = 8.1-12.5 s). The
  default Demo_day run, without `-Identity`, is unchanged.
- At the zebra where loop A turns north (y = -3000 on junction (4100, -4100))
  a figure can wait at the kerb for minutes (194 s longest, 8 of 40 figures
  over 100 s in 33 minutes). The likely cause (inferred, not measured):
  loop A's turning platoon uses the whole 8 s walk window, and the figures
  yield to cars. The fix is pedestrian priority
  on the car side, and it is not done.

#### Added
- `verify_peds.py`: `off_route` (a figure more than 5 m from its own leg),
  `detours`, `teleports`, and per-figure reasons for any figure on a zebra
  that is not crossing.
- `docs/video/citylife_redcar_identity.mp4`: `citylife_redcar_id1`, the
  first-person view with the HUD beside the chase camera (262 s, 1920x540
  H.264, 23.6 MB). The duration is checked against the master.
- Tests: 793 in the suite (41 files).

### 2026-09-29 (evening) — the lock that could not let go, lights that change, people who wait

Day-by-day record in Indonesian: `docs/WORKLOG.md`. The write-ups are
`docs/FINDING-the-lock-that-could-not-let-go.md` and the fifth part of
`docs/FINDING-crowd-pedestrians-and-traffic.md`.

#### Retracted
- **Every `det_hz` published before today is inflated.** metrics.json divided
  EVERY inference since the detector loaded - including minutes of start-gate
  waiting - by ticks x 0.1 s, which is shorter than the mission whenever the
  loop ran below 10 Hz. Recomputed from the flight logs (the inference seq
  numbers the ticks consumed, over the ticks' own span):
  `citylife_follow2` 7.60 -> **3.06**, `citylife_follow3` 4.63 -> 3.18,
  `citylife_city` 4.82 -> 2.96, `people_final` 4.04 -> 3.26,
  `fixed_kpi` 4.41 -> 3.53, `city_kpi` 5.15 -> 4.06, and on the red car
  6.88-7.50 -> 3.46-3.99. So:
  - "`det_hz` clears its 4.0 Hz gate in all four" (09-23, the four
    `citylife_follow*`/`_city` flights) is wrong: none of the three whose logs
    survive does (2.96-3.18); `citylife_follow`'s log is deleted.
  - "The start gate can fire while the level is still streaming (detector at
    3.8 Hz instead of 7)" (09-29, morning) is wrong. Every red-car flight ran
    its detector at 3.5-4.0 Hz during the mission; the "7" was the gate's
    inferences counted over mission time. There is no streaming effect.
  - `docs/FINDING-decorative-pedestrians.md` (4.04 "clears 4.0") and
    `docs/FINDING-five-defects-found-by-audit.md` (4.41) inherit the same
    error. `tools/build_eval_data.py` now recomputes the rate for old flights
    (`det_hz_mission`) and keeps the reported one as `det_hz_reported`.
- **The tracking score's projection was linear, the camera is a pinhole.**
  `track_truth.project_target_cx` mapped angle to pixel linearly (off by up to
  ~4 deg between centre and edge). Re-scored with the pinhole
  (`PROJECTION = "pinhole"`), the red-car flights' `frac_on_target` moves by
  less than 0.02, but the chance floor rises by 0.02-0.05, so the margin over
  chance shrinks everywhere. Three flights drop below it: `_final1` (0.356 vs
  0.373), `_final3` (0.274 vs 0.288) and `_carpolicy` (0.442 vs 0.443, a
  tie). `_trail3` (0.059 vs 0.094) and `_far` (0.083 vs 0.097) were below it
  already on the linear score. (A first draft of this entry named only
  `_final3` and `_trail3`, and `_trail3` as a new casualty.) The default
  stays linear so published numbers reproduce; the table is in the WORKLOG.

#### Fixed
- **The drone locked onto a red pedestrian signal 110 m from the car**
  (`citylife_redcar_trail`, t = 180 s). Three causes, three fixes:
  - `TargetLock` could not say "none of these": with nothing near its
    prediction it took the best-scoring box. `select_strict` (with
    `--identity`) answers None instead; only a start gate or a re-acquisition
    seeds it.
  - The ground check only rejected box bottoms above ~2.7 m; a kerbside signal
    at 2-3 m passed. Every candidate is now judged physically at its OWN
    frame's pose and depth (`demo/identity.py`: width, aspect, bottom height,
    distance from the street; HARD / SOFT / OK). Thresholds grid-searched on
    the 12 flights: in-sample 86.4 % of wrong boxes not OK, 14.1 % of true
    ones, 0.3 % of true ones HARD. Leave-one-flight-out, 86.5 % / **15.2 %** /
    0.6 %: the held-out true-box rate is just over the 15 % gate, which only
    the in-sample figure meets. (Drafts said 15.4 % and "passed"; the 15.4
    predates the bottom-clip guard moving into `identity.features_for`.)
  - After a loss the estimator's gate widened until a far box could re-seed
    it. Now the estimate lapses after 3 s and only `Reacquirer` restarts it:
    4 OK sightings in a row, <= 45 m, within reach of the last measured
    position, and - when nothing constrains where it is - seen MOVING (the
    replay found a red fire-hydrant sign by the launch point seeding it).
  Replayed open-loop on the 12 recorded red-car flights with the same boxes
  (`tools/replay_pipeline.py`, pooled integers): the estimate is on the car on
  **94.5 %** of the ticks it is served, against 22.9 % for the estimator as
  flown, and **0 of 12** seeds (7 starts, 5 re-acquisitions) are on something
  else, where the flown estimator's first accepted box after a gap was
  wrong after 210 of 241 gaps. It is served on 7.6 % of the ticks (flown:
  45.1 %), because the replay only has the box the old lock picked - a car
  the old lock passed over is not in the log. Earlier drafts compared
  against an "old" arm that reset and re-seeded, which the flown code never
  did (95.5 % vs 22.4 %, 0 of 18 vs 260 of 293).
- **Bearings used a linear pixel-to-angle map** (up to ~4 deg off; physical
  widths ~21 % small). One pinhole model, `demo/camera_model.py`, everywhere;
  `--linear-bearing` reproduces older flights, and an AST test keeps the
  linear formula out.
- **A detection met the tick's pose and the latest depth frame**, 0.2-0.4 s
  after its image was captured. `SemanticObs.get_front_capture` now pairs it
  with the pose interpolated at capture (sim stamps when both sides have
  them) and the nearest depth frame (none beyond 70 ms), and the estimator
  folds each measurement in at its capture time (posterior kept at the last
  update; `predict` no longer moves it).
- **"NFZ AHEAD - HOLDING" at a building.** The HUD says OBSTACLE unless a
  no-fly polygon caused the hold; TARGET LOCKED only on a fresh identity-OK
  box, else TRACKING (PREDICTED) / RE-ACQUIRING n/4 / PURSUING ....
- **Search after a loss rotated in place** (the reference flight, against a
  building corner). `--search-planner`: pursue the car's street to the next
  junction, watch each exit, hold over it (`demo/search.py`).
- **Landings in a hedge and on a parked car.** `--land-site` flies, through
  the Shield, to a pavement cell `demo/landing.py` calls landable first.
- **Cars waited with their front half on the zebra** (give-way target
  JEdge - 300 put the nose 1170 cm from the junction centre, inside the
  800-1400 band). They now stop at a line 1730 cm out, 100 cm behind it.
- **The runner started flights at 43.9 m.** `-StartMaxRange` 45 -> 30 and
  `-StartTimeout` 240 -> 330 (red lights lengthen the car's lap).

#### Added
- **Traffic signals that change and are obeyed.** `BP_SignalController`
  switches 258 of the 292 heads (27 vehicle, 231 pedestrian) by a fixed-time
  plan (`tools/citylife_signals.py`: green 20 / yellow 3 / all-red 2 s per
  axis, 50 s cycle, offsets spread by grid index); `BP_CityCar.UpdateEffSpeed`
  obeys the same plan (`tools/citylife_mcp/drive_signals.py`, a node-for-node
  port of `tools/citylife_traffic_model.py`, which held RedViol 0, ZebraWait 0,
  MinGap 650 cm over 30 simulated minutes at 3/10/30 fps). Simulate, ~7 min:
  RedViol 0, ZebraWait 0, MinGap 650 cm, longest stand-still 78 s. (That
  ZebraWait could not have fired; see 2026-09-30.)
- **Pedestrians with somewhere to go.** 40 figures walk closed tours over a
  pavement graph whose only road-crossing edges are the zebras
  (`tools/citylife_peds.py`), wait at the kerb for the walk phase and a 4 s
  gap, and cross without stopping (`tools/citylife_mcp/ped_walk.py`;
  `--roam` restores the old graph, saved in `ped_roam_v1.dsl`). Simulate:
  51 crossings started, kerb wait max 42 s, nobody standing on a zebra.
  (The zebra check was too loose, and two pedestrian bugs were behind it;
  see 2026-09-30.)
- Metrics: `identity`, `reacquisition`, `estimate_on_subject`
  (`track_truth.score_estimate`), `landing`, per-tick `tier`, `lock_why`,
  `reacq`, `plan`, `est_xy`; `detections.jsonl` logs every candidate with its
  features and tier.
- Tools: `tools/replay_identity.py`, `tools/replay_pipeline.py`,
  `tools/citylife_mcp/{inspect_signals,signals,drive_signals,ped_walk,
  verify_signals,verify_peds}.py`; survey `docs/data/citylife_signals.json`.

### 2026-09-29 — a map of this city, a trail to follow, 10 Hz, and four reviews

The write-up is the fourth part of
`docs/FINDING-crowd-pedestrians-and-traffic.md`.

#### Retracted
- **"The corner stall was the aircraft pressing into a tree and a signal
  pole"** was already corrected before the 09-23 commit; the phantom wall it
  describes is now fixed at both ends (below).
- **"9.3 Hz is the Windows 15.6 ms timer"** (09-23). Half of it: the pacing
  slept `TICK - work` from each tick's own start, so every timer round-up was
  lost. An absolute deadline alone reaches 10.0 Hz at 15.6 ms.
- **`start_heading_err_deg: null` in every metrics.json since it was added.**
  It was measured and then reset to None before any metric read it.

#### Fixed
- **The Shield's phantom wall.** Off the obstacle map `_distance_at`
  extrapolated a border building into a wall to infinity (-35.16 m at a
  CityLife junction entrance). The distance off the map now stays within
  [v - off, v + off] and a border building extends past the edge only as far as
  it reaches into the map. Against HEAD, 12 of 28,800 sampled monitor answers
  in `test_check_contract` change, all on forecasts that leave the map; the
  fixture now also pins four policies it never covered.
- **The control loop: 9.99-10.0 Hz** on every flight (was 9.31-9.37 on the red
  car, 8.38 on the August Demo_day reference): absolute-deadline pacing plus
  `timeBeginPeriod(1)`; `timer_resolution_ms` measures the wait the loop does.
- **The estimator was re-fed the Grounder's held box** - up to 8 s old - on
  every inference that found nothing, paired with the current pose. It now
  takes each detection once, fresh.
- **The ground-contact check rejected the car it protects** (34 boxes on the
  red car through a corner). A box on the estimated subject is exempt, while a
  ground-checked box fed the estimate in the last 3 s.
- **A car on a zebra drove on whoever stepped out.** It now stops where it is
  for a pedestrian in its own path. Final Simulate, 4.4 min: 14 emergency-stop
  ticks, none above 50 cm/s; walking figure within 4 m at speed 0; closest two
  cars 650 cm (the design value).
- The runner fails loudly: a failed flight no longer points at the previous
  run's metrics.json, a throw or Ctrl+C stops the simulator, the map check
  runs before any simulator starts, and the teardown kills only the `-game`
  process.

#### Added
- `demo/trail.py` + `--trail-follow` (on in the car runner): fly where the
  subject DROVE - breadcrumbs from the estimator, a carrot along them in
  track, the last sighting and the subject's last direction in coast and
  search, never nearer a stopped subject than the follow's stand-off and never
  nearer anything than the camera's blind spot plus 2 m.
- `demo/build_voxel_map.py`: any rectangle, several bands from one query, raw
  voxels saved (`--from-voxels`); `demo/out/citymap_citylife/` for CityLife
  (180 x 100 cells; the +-80 m cube rebuilt through it agrees on all 5,600
  overlapping cells). `build_street_mask.py --dir`.
- Evidence in metrics.json: `collisions` (the simulator's contact reports,
  split into before t0, mission and after it), `off_map_ticks` (null with no
  map), `trail`, `presence_block_reasons`, `ground_check_waiver`,
  `timer_resolution_ms`; per tick `gate_why`, `pitch_deg`, `roll_deg`, `trail`.
- `tests/test_trail.py` (18), `tests/test_voxel_map.py` (5), new cases in
  `test_clearance.py` and `test_range_and_lock.py` (100).
- `test_track_truth`'s median test knows a third losing shape: a crowd flight
  (`citylife_ped_final`, 12.0 px against 7.4) where the class-level null is the
  distance to the nearest of ~40 figures. Allowed only with per-figure truth
  and a one-figure score at least 0.05 over its own null (0.58 vs 0.48); it
  excuses no other flight. Suite: 553/553.

#### Flown
Seven red-car flights with the trail (240 s each), a pedestrian flight on the
CityLife map (180 s), and DEMO 1 on Demo_day with and without the trail.
`p0_violation_escape_rate` 0.0 and no collision during any mission.

| red car | within 30 m | longest < 30 m | estimate on the car |
|---|---|---|---|
| `_ground` (09-23, no trail) | 0.314 | 48.8 s / 146 m | 0.55 |
| `_trail` | **0.436** | **103.4 s / 305 m, two corners** | **0.74** |
| `_trail2` / `_trail3` / `_final1..4` | 0.088-0.282 | 12.8-59.0 s | 0.11-0.36 |

Each of `_trail2`, `_trail3` and `_final1` exposed a defect fixed after it (see
the FINDING). Pedestrian: within 30 m 1.00, instance on target 0.58 against a
null of 0.48. Demo_day DEMO 1: within 30 m 1.00 with and without the trail,
mean separation 16.9-17.3 m, loop 10.0 Hz (8.38 in August).

#### Found, not fixed
- **Which red thing.** Across the seven trail flights the estimator served
  the car on 36 % of its ticks (74 % on `_trail`); the mission in this level is
  limited by target identity, not by the corner.
- Loop B waits up to 42 s to give way at its junctions.
- The 10 m pedestrian ring binds only to the tracked subject (839 ticks with
  another pedestrian inside 10 m, by design).
- The start gate can fire while the level is still streaming (detector at
  3.8 Hz instead of 7).

### 2026-09-23 — the right frame width, the left side of the road, cars that yield, and the red car

The level's logic is reproduced by `tools/citylife_routes.py` and
`tools/citylife_mcp/` (it is gitignored); the write-up is the third part of
`docs/FINDING-crowd-pedestrians-and-traffic.md`.

#### Retracted
- **The 2026-09-22 tracking columns, and "`frac_on_target` no longer measures
  anything on this level".** `demo/track_truth.py` fell back to a 400 px frame
  on a 768 x 432 camera. Re-scored with the width recovered from
  `detections.jsonl`: `citylife_follow2` 0.255 on target against a null of 0.802
  (in-shot median 128.2 px vs 31.9), `citylife_follow3` 0.365 against 0.808
  (113.5 vs 23.3) - on those two the box was mostly NOT on a person, worse than
  chance, which the video had shown and the numbers had not. `citylife_city`
  1.000 against 1.000, median 9.2 px vs 10.1: at chance. "45.5 px against 5.2 px"
  was an artefact of the width. `citylife_follow` cannot be re-scored (its log
  is deleted), and none of these flights can be instance-scored (their logs
  lack `truth.names`).
- **"Lanes sit 350 cm left of each centre line"** (2026-09-22). They sat on the
  RIGHT of travel - eastbound at X = 3750, the south half, with X = north and
  Y = east - against the level's own keep-left lane markings.

#### Fixed
- The scorer's frame width: rows carry `det.img_w`, `track_truth.load_rows`
  recovers it for older logs (commit 37514c3).
- The controller steered on boxes its own presence check rejected.
  `--presence-gates-control` (on in the CityLife runner) turns an ABSENT box into
  a miss.
- The control loop slept a full tick after its work, so it could never reach
  10 Hz. It now sleeps to a 0.1 s deadline (`--legacy-tick-sleep` keeps the old
  pacing): **9.31-9.37 Hz** on every red-car flight, against 4.04-6.89 Hz before
  on the same level. Still under the 9.5 gate.
- Cars keep left, on dense lane-centre polylines with filleted corners (loop A
  616 m, B 334 m, C 288 m; 7 m and 13 m arcs; speed limit per point from
  1.8 m/s^2 lateral acceleration and a 1.5 m/s^2 braking ramp).
- **Cars turned by pivoting.** `UpdateAim` and the fixed-rate `RInterpTo` yaw are
  replaced by `DriveTick`: path-curvature feed-forward plus lateral and heading
  correction (L = 5 m, zeta = 0.9), frames cut into <= 50 ms sub-steps. Pure
  pursuit was tried first in simulation and rejected (66 cm corner cut at
  3.2 m/s^2). In Simulate: worst lane error **10.1 cm**, worst lateral
  acceleration **1.93 m/s^2**.
- Cars crept into stopped cars (a 60 cm/s floor) and ignored junctions and
  pedestrians. Now they stop 6.5 m behind a queue, give way where their path
  really crosses another loop's (loop B, at (4100, 4100) and (12300, -4100)), and
  stop for a pedestrian on a crossing on their own path. In Simulate: closest
  approach between any two cars **650 cm**, longest stand-still 22.3 s.
- **The start gate could be captured by a distractor**: a red fire-hydrant sign
  held the jump gate and the instance lock for 300 s and the car was never a
  candidate. The gate now suspends both until it confirms a subject (`Acquirer`),
  and accepts only one within 45 m.
- **A level car crashed the flight loop** on its first tick
  (`'LevelCar' object has no attribute 'pose_at'`).

#### Added
- `policies/follow_car_citylife.yaml`: the car mission's policy - both
  stand-offs of `follow_pedestrian.yaml`, the envelope of `follow_car.yaml`. The
  runner used the pedestrian policy, whose 3.0 m/s cap is below the car's
  3.2 m/s.
- A ground-contact check in `presence_verdict` for things that stand on the
  road: the ray through a box's bottom edge must meet the road near the measured
  range. A red traffic signal passed every other check as a car.
- `tools/citylife_routes.py` (+ `follow_step`/`follow`, the reference the
  Blueprint mirrors, `crossings_on`, `give_way_junctions`) and
  `tests/test_citylife_routes.py` (22 tests); `tools/citylife_mcp/` -
  `ue_rpc.py`, `drive_tick.py`, `rewire_tick.py`, `apply_routes.py`,
  `one_red_car.py`, `verify_drive.py`.
- Exactly one red car: `Car_04` and `Car_22` repainted blue and white; `Car_10`
  drives loop A at 3.2 m/s.
- `--level-car`, `--start-when-seen`, `--start-max-range-m`,
  `--start-timeout-s` (`demo/follow_vlm.py`); gate snapshots and a truth trace in
  `metrics.json`; the red-car mode of `scripts/run_citylife_follow.ps1`.

#### Flown
Six red-car flights, 240 s after the start gate
(`-Object "a red car" -LevelCar Car_10`), each isolating one change:

| run | on target (null) | median px (null) | in shot | within 30 m | outcome |
|---|---|---|---|---|---|
| `citylife_redcar_far` | 0.083 (0.097) | 180.3 (145.9) | 0.262 | 0.095 | success |
| `citylife_redcar_pedpolicy` | 0.320 (0.250) | 57.3 (105.1) | 0.505 | 0.118 | success |
| `citylife_redcar_carpolicy` | 0.418 (0.392) | 55.5 (81.6) | 0.674 | 0.256 | success |
| `citylife_redcar_ground` | **0.532 (0.414)** | **39.0 (84.9)** | **0.762** | **0.314** | success |
| `citylife_redcar_high` (12 m) | 0.547 (0.487) | 20.0 (37.6) | 0.617 | 0.247 | **fail** |

(The first flight crashed and was deleted.) `p0_violation_escape_rate` 0.0 in
all. The best flight held the car within 30 m for **48.8 s over 133 m of
street**, then lost it where it turned the first corner. Video:
`docs/video/citylife_redcar_ground.mp4`. The ground-contact check did not fire
in that flight, so its improvement over `_carpolicy` is run-to-run variation.

#### Found, not fixed
- **Corners - a wall the Shield invented.** In `_carpolicy` and `_ground` the
  aircraft stalled at the same junction entrance for 40-187 s: every obstacle
  map the Guardrail loads covers only NED [-80, 78] m (the 25 August Demo_day
  survey), and past its edge `Shield._distance_at` extrapolated a border building
  into an ever-deeper wall (-35 m at the junction). The Shield swapped the
  follower's +3 m/s north for 5 m/s south, in bursts.
- **The aircraft left its altitude envelope** in `_high`: 3.1 s above the 14 m
  ceiling, peak 14.32 m, while every command the Shield emitted asked it to
  descend - each climb coincides with one of those 5 m/s phantom-wall reversals.
  12 m cruise was reverted to 8 m.
- 5 half-rate ticks in 4 min of Simulate where a car was on a zebra with a
  walking pedestrian on it; cause not established.

### 2026-09-22 — hands, hair, corners, and a city instead of a street

The CityLife level lives in the gitignored `PASBlocks/`, so what reproduces it
is `docs/FINDING-crowd-pedestrians-and-traffic.md`.

#### Fixed
- **The crowd figures had no hands.** `m_tal_nrw_base`, the mesh they were
  built on, is a 3-vertex stub whose only material slot is called `M_Hide`: it
  drives the pose and renders nothing. Epic pairs it with `m_tal_nrw_body`
  (10,648 vertices, skin, hands), which was never attached. Added as a `Body`
  component on all six variants and leader-posed like the clothes. Leader pose
  matches bones by NAME, so the different skeleton (`metahuman_base_skel`) is
  not an obstacle — the face already proved that.
- **The hair trailed the head.** It was parented to the mesh COMPONENT with no
  socket, so it followed the capsule and not the `head` bone. `BeginPlay` now
  attaches it to that bone with `KeepWorld`, which preserves the authored
  placement without computing an offset.
- **Cars pivoted at corners.** New `UpdateAim` slides the aim point up to 4.5 m
  into the next leg as the car arrives and scales the corner speed, so the
  heading target moves continuously. Measured: cars corner at 64-70 % of their
  own top speed and arc rather than turn on the spot.
- **Gap keeping deadlocked the fleet** in the first version of that change: at
  a corner two cars can each be "ahead" of the other, so both braked to zero and
  stayed there. A car now only brakes for cars facing the same way, and
  `EffSpeed` has a 60 cm/s floor.

#### Added
- The environment is a city rather than one street: **two car loops on the map's
  own 82 m junction grid (684 m and 300 m), 16 cars, 40 pedestrians**, nav
  bounds from 48 x 104 m to 200 x 185 m, and six new no-walk bands over the
  carriageways. Lanes sat 350 cm from each centre line, so the two loops read
  as two-way traffic. *(Corrected 2026-09-23: they sat on the RIGHT of travel,
  eastbound at X = 3750, against the level's own keep-left lane markings - never
  "left" as written here. Cars keep left since 2026-09-23.)* Measured in Simulate: 32 of 40 walking, **0 in any
  carriageway**, nearest car-to-car 567-1544 cm, **0 ticks under 4 m**.
- `demo/level_actors.py` + `--level-peds` in `demo/follow_vlm.py`: ground truth
  for actors the LEVEL owns, read back from the simulator with
  `World.get_object_poses`. Without it a CityLife flight logs an empty truth on
  every tick and scores unscorable while looking complete.
  `tests/test_level_actors.py` covers it (10 tests).
- `scripts/run_citylife_follow.ps1`: the follow mission against the level's own
  crowd, with no client-side pedestrians, parked cars or scripted car.
- Every pedestrian and car carries its name as an actor TAG.

#### Retracted
- **"Names are load-bearing: an actor's name IS its segmentation class"** in
  `docs/FINDING-citylife-level.md`. The plugin reads `GetOwner()->GetName()`,
  which for a placed Blueprint is `BP_CityPed_M1_C_1`. `Ped_00` is the editor
  LABEL, and labels do not exist in a `-game` build. The tags above are what
  make the intended names resolvable.

#### Added, later the same day
- Pedestrian crossings: the no-walk bands are cut by 600 cm at each painted
  crosswalk, so the navmesh stays walkable across the carriageway. A
  `NavLinkProxy` would need a struct-array write, which this toolset does not do
  reliably. Measured: 3 of 40 figures mid-crossing, 0 anywhere else in a road.
- 24 cars on three loops (a third, west of the corridor), each pedestrian on its
  own `GlobalAnimRateScale` (0.93-1.07, 27 distinct values) so 40 people no
  longer share a footfall, and the gap scan runs on every other tick.
- Measured after all of it: nearest car-to-car **430 cm, 0 ticks under 4 m**
  across 24 cars and ~1,900 ticks.

#### Flown
Four 180 s flights, `scripts/run_citylife_follow.ps1`:

| run | truth | poll | det_hz | loop_hz | on target | chance |
|---|---|---|---|---|---|---|
| `citylife_follow` | 16 of 40 | 0.1 s | 5.82 | 5.29 | ~~0.063~~ n/a | ~~0.571~~ n/a |
| `citylife_follow2` | 40 | 0.1 s | 7.60 | 4.04 | ~~0.739~~ 0.255 | ~~0.922~~ 0.802 |
| `citylife_follow3` | 40 | 0.5 s | 4.63 | 6.89 | ~~0.721~~ 0.365 | ~~0.811~~ 0.808 |
| `citylife_city` | 40 | 0.5 s | 4.82 | 6.17 | ~~0.821~~ 1.000 | **1.000** |

*(Corrected 2026-09-23: tracking columns re-scored with the frame width recovered from
`detections.jsonl`; the first scoring fell back to 400 px on a 768 px camera.
`citylife_follow` cannot be re-scored: its log is deleted.)*

- `det_hz` clears its 4.0 Hz gate in all four; the control loop clears 9.5 Hz in
  none, and neither did the reference flight on the old level (8.33 Hz).
- `p0_violation_escape_rate` **0.0** in all four. Not KPI-grade: the topology is
  `projectairsim-single-host` by construction.

#### Retracted, about our own measurement
- **`frac_on_target` no longer measures anything on this level.** Its chance
  baseline - a box placed with no skill, scored against "any pedestrian" - is
  **1.000** in the last flight, because 40 people fill the frame. The median
  error says the same: 45.5 px against a chance of 5.2 px. Tracking claims about
  CityLife have to be scored against the LOCKED instance (`target_lock`: 515
  ticks held, 19 switches), not the class. `subject_truth_pts` returning every
  pedestrian was right for 12 and is wrong for 40.

#### Not measured
- The control loop is under its gate (6.17-6.89 Hz vs 9.5) and the split between
  "this level" and "the truth polling" is not separated: that needs the same
  mission flown with `--level-peds 0` as a control.

#### Earlier, 2026-09-21

The CityLife level lives in the gitignored `PASBlocks/`, so what reproduces it
is `docs/FINDING-crowd-pedestrians-and-traffic.md`. Backup of the pre-change
assets: `PASBlocks/_backup/citylife_2026-09-21/`.

#### Added
- CityLife pedestrians are six City Sample Crowd variants (3 male, 3 female)
  instead of Manny. They reuse `BP_CityPed_Human`, built 2026-09-16 and never
  placed, and walk on the existing animation with no retarget because
  `SK_Base` is registered compatible with `SK_Mannequin`. Face RigLogic off,
  face forced to LOD3, hair as cards instead of grooms. Measured: 12 and 15 of
  16 walking, 0 in the carriageway.
- Gap keeping in `BP_CityCar`: target speed falls with the square of
  gap / (700 cm + 1.8 s x speed). Measured inside the engine: nearest other car
  621-771 cm, 0 ticks under 4 m.
- `citylife_level` in `docs/data/eval_sep2026.json` carries the pedestrian
  model, the car gap and the second finding.

#### Fixed
- `M_CarPaint` did not compile (two empty texture samples), so every
  `MIC_Paint_*` rendered in the default grey. Textures assigned; three cars now
  carry Gold, Purple and Black next to the five `VehicleVarietyPack` bodies in
  their own paint. Six of eight colours are confirmed by measured hue in one frame; the
  white truck and red sports car sat in shadow.
- The crowd figure's clothes and head followed the template's mesh, not their
  own body: `LeaderPoseComponent` on the template is a weak pointer to the CDO.
  It is now set in `BeginPlay`.

#### Retracted
- **"Paint is set per instance from the eight `MIC_Paint_*`"** in
  `docs/FINDING-citylife-level.md`. The saved level referenced no
  `MIC_Paint_*`, and their parent material did not compile, so no car could
  have shown one.
- **"Cars sat within 4 m of each other"**, same file. The positions were read
  one tool call at a time, and each call lets the game advance a frame, so the
  distance is not reliable. The same method reported 12 cm with gap keeping
  already on, while the in-engine meter read 621 cm minimum.

#### Not measured
- OWL-ViT `a person` did not improve with the realistic figures (10 m up:
  Manny 0.062, crowd 0.054 / 0.105; 20 m up: 0.027 vs 0.012 / 0.013; n = 1
  per cell). The frame-rate cost of the crowd is unknown: the editor world ran
  at 2.8 ticks/s with or without the figures. Needs `-game` plus a flight.

## [0.5.1] — 2026-09-14

### Added
- `tools/build_eval_data.py` → `docs/data/eval_sep2026.json`: every number the
  September decks and the mid-evaluation report quote, computed once from the
  artefacts, reusing `demo/track_truth.py` so there is one scorer.
- A visibility breakdown of stand-off ring positives (in frame / outside the
  horizontal FOV / under the nose), and a check of the served range against the
  pedestrian under the detection box.

### Retracted
- **"The rule saw 31 of 516 because the position the Shield was served was wrong
  by tens of metres"** — in the README, the Pages site, the 0.4.0 notes, the
  meeting pack and the September deck. Two errors in one sentence:
  - `standoff_score` counts a tick positive when *any* pedestrian is within
    10 m. `SubjectStandoff` protects only the subject being followed. On **498 of
    the 516** ticks the person inside 10 m was outside the camera's ±45°
    horizontal field of view; 6 were under the nose; 12 were in frame.
  - The "37 m served-range error" compared the subject's estimate with the
    *nearest* pedestrian — a bystander beside the aircraft. Against the
    pedestrian under the detection box (ticks 340–700) the estimate was 43.3 m
    and that person 49.4 m away; against the nearest visible pedestrian the gap
    is 1.8 m. The estimate was roughly right about the person being followed.
- **"Depth sampled through an 8-pixel box is the background"** was an inference
  from that comparison, never a measurement, and falls with it.

- **"P0 escape rate is 0.0 on every flight recorded here"** (README, Pages). It
  is 0.0 on all 41 shielded flights; the five unshielded control flights read
  0.63, as they are designed to.

### Still true
- The ring fired 94 times; on 63 of them no real pedestrian was within 10 m.
- The pedestrian-phase detector does not beat a centre constant (7.0 px against
  5.9 px).
- A real pedestrian came within 4.70 m of the aircraft while
  `p0_violation_escape_rate` read 0.0. The gap it exposes is **policy scope**
  (no rule protects non-subject pedestrians) and **sensor coverage** (a forward
  camera cannot see beside the aircraft) — not a wrong range estimate.

---

## [0.5.0] — 2026-09-10

### Added
- `CAMERA_HFOV_DEG`, read from the simulator config at import, and used as the
  default for `servo`, `implied_width_m`, `range_from_depth` and `TargetLock`.
- `det_gt_err_deg_median_in_shot` and its centre-constant null, plus
  `frame_widths_seen`, in `track_truth.score_rows`.
- Two regression tests: one changes the config and asserts the reader follows;
  one walks the AST for a literal `45.0`/`90.0` handed to `radians()`.

### Changed
- **FrontCamera raised from 400×225 to 768×432**, scene and depth together.
  `OwlViTProcessor` resizes every input to 768×768, so the forward pass does not
  depend on capture resolution — measured on an RTX 4090, median of 20, one
  query: 400×225 → 12.6 ms, 768×432 → 12.5 ms, 1280×720 → 13.2 ms. A 400×225
  frame was being upscaled 1.9× into the model. End to end 768×432 is *faster*
  than what it replaces (30.7 ms against 35.2 ms).

### Fixed
- `math.radians(45.0)` was inlined in the estimator feed — the camera's
  half-FOV, as a literal — while `servo()` three hundred lines away took
  `hfov_deg` as a parameter. Re-aiming the camera would have scaled every
  estimator bearing wrong, served the Shield a subject in the wrong direction,
  and raised nothing.

### Known limitations
- **The 768×432 change has not been flown.** Offline measurement says it costs
  nothing; behaviour under Unreal's GPU contention is unmeasured, and that
  contention is severe — across five recorded flights the forward pass runs
  13.8–21.5× slower in flight while preprocessing runs only 1.8–2.6× slower.
  The detector is GPU-starved, not CPU-bound.
- Raising resolution buys *detail*, not *size*: model pixels are
  `(angular width / hfov) × 768` regardless of capture resolution. A 0.5 m
  pedestrian still occupies 0.48 of one 32×32 patch at 16 m. Only a narrower
  FOV changes that, and it is not yet taken.

---

## [0.4.0] — 2026-09-09

### Added
- `SUBJECT_VMAX_MPS` and a post-update velocity clamp in `TargetState`. A
  constant-velocity filter cannot know that a person does not travel at
  13 m/s; once the estimate ran away, its own 4σ gate rejected 149 of the next
  154 measurements and it coasted the runaway velocity to the end of the
  flight. Pedestrian phase on replay: estimate served 55/331 → 257/331, error
  to the nearest real pedestrian 31.08 m → 9.15 m.
- `yaw_command` — one yaw cap in one place. The coast branch clipped to
  1.1 rad/s and the estimator branch clipped to nothing; `TargetState.observe()`
  can return a bearing behind the aircraft, so it reached 130.4 deg/s.
- `track_truth.score_standoff_firings` — TP/FP/FN and precision/recall per
  `SubjectStandoff` ring, surfaced in `metrics.json` as `standoff_score`.

### Retracted
- **The claim that the 10 m pedestrian stand-off "fired six times" was the
  demonstration the rule finally worked.** Scored against ground truth from the
  same flight log: 0 true positives, 6 false, 49 false negatives, precision
  0.00. All six fired against a diverged estimate 3.81–9.39 m away while the
  nearest real pedestrian was 16.23–16.53 m away.
- Three candidate fixes were measured and dropped rather than shipped: a
  presence-gated selection filter (the verdict marks the *car* phase's most
  accurate boxes as absent), an implied-width gate on the estimator (its
  apparent win is the speed clamp's, and together they are worse than the clamp
  alone), and an implied-width gate on selection.

### Fixed
- The re-flight comparison quoted whole-flight figures for two flights of
  different length (69.95 s against 119.95 s). On the matched window every
  figure is better than published — median |yaw| 11.5 → 1.3 deg/s, peak
  130.4 → 9.6 — and so is the thing that matters less: the aircraft came 4.70 m
  from a real person against the previous flight's 9.43 m.

### Known limitations
- `p0_violation_escape_rate` reads 0.0 on a flight where a real pedestrian came
  within 10 m on 516 ticks and the stand-off rule fired on 31. *(Corrected in
  0.5.1: 498 of the 516 were people outside the camera's view whom the
  subject-only rule does not protect — a scope and coverage gap, not a range
  error.)*

---

## [0.3.0] — 2026-09-08

### Added
- `guardrail/replay.py` — WP4 replay bundles that re-derive their own KPIs.
- `guardrail/frames.py` — the body/world Action4D boundary, with `yaw_rate`
  documented as rad/s and pinned by test.
- `demo/occ_bands.py` — altitude-band-aware obstacle map selection.

### Changed
- The headline tracking statistic moved from `frac_on_target` to the median
  pixel error against a null family (uniform draw, centre constant, best fixed
  column, lag-1 persistence).

### Retracted
- **`frac_on_target` at a 100 px tolerance is a statistic a constant can pass.**
  A "detector" that emits the frame centre and never opens the image scores
  1.000 on `city_locked` — margin exactly 0.000 — and *beats* the real detector
  on `city_full`.
- `replay.py`'s `VERIFIED_KPI_FIELDS` named two keys nothing emits, so it
  compared five fields and reported seven; one of them was a grant KPI. Its
  policy guard was also skipped whenever `manifest.json` was absent, so a real
  flight bundled cleanly under all 26 foreign policies.
- Test runners printed `PASS` for fixture-less tests: "8/8 passed" for a run in
  which seven asserted nothing.

---

## [0.2.0] — 2026-09-01 → 2026-09-07

### Added
- Scenario sweep harness (`experiments/sweep_scenarios.py`).
- Two previously unmeasured grant KPIs, and two previously unbuilt rules.
- Signed policy bundles, WGS84 policy origins, and a start-up refusal for a
  rule no `--object` phrase can reach.
- September deck built from the artefacts rather than from memory, and the
  meeting pack.

### Fixed
- **A `SubjectStandoff` rule that was hashed, audited, and completely inert.**
  The policy named class `pedestrian`; the phrase `a person` produced `person`;
  `binds()` compares exact strings. Across a whole flight with a human subject
  it bound zero times and the 5 m catch-all applied instead. No violation is
  indistinguishable from no rule.
- `--want-width` is an *angular* target, so the stand-off it asks for scales
  with the subject's width — 15.83 m for a 4 m car, 1.98 m for a 0.5 m person.
  The retarget block updated the width and not the set-point derived from it,
  so the aircraft station-kept at the car's distance and was commanded
  *backwards* at −1.18 m/s while doing so.

---

## [0.1.0] — 2026-08-24 → 2026-08-31

Initial Guardrail layer: the policy DSL (WP1), the prefix compiler (WP2), the
suffix Safety Shield (WP3) and stress testing (WP4); the first KPI-grade
results on the ArduPilot SITL rail; occupancy maps built from the simulator
rather than inferred from empty space; scripted city scenes with traffic,
parked vehicles and pedestrians; and the two-view recorder.

---

## Publication note

This history was rewritten once, at first publication on 2026-09-10, to remove
`meeting notes/` and `docs/CORRECTIONS-2026-09-02.md` — internal records naming
colleagues, withheld deliberately. Every commit SHA therefore differs from the
pre-publication working copy, and four commit hashes cited in
`docs/CHECKLIST-remaining-work.md`, `docs/FINDING-the-kpi-was-never-measured.md`
and `docs/MIDTERM-REPORT-Aug2026.md` refer to that earlier history. One commit
("Record the 2026-09-02 meeting…") touched only the removed paths and was
pruned as empty, so the published history is 50 commits where the working copy
has 51.
