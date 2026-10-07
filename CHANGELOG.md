# Changelog

All notable changes to the Guardrail safety layer.

Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
versioning is [Semantic Versioning](https://semver.org/spec/v2.0.0.html), with
the caveat that this is pre-1.0 research software and the public surface is
still moving.

A note peculiar to this project: several entries below are **retractions**. The
hard KPI here is *P0 violation escape rate = 0*, and a safety number that is
wrong is worse than one that is missing, so a corrected claim is treated as a
shipped change and recorded as one.

---

## [Unreleased]

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
  the deck is rebuilt from a clean tree.

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
