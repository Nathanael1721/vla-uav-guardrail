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
