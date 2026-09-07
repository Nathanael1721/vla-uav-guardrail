# Remaining work, against the grant and the meeting record

**Date:** 2026-08-28, updated 2026-09-01 (see the update section below)
**Sources:** the grant deliverables as restated in `docs/scope-clarification.md`,
the work-package status table in `docs/MIDTERM-REPORT-Aug2026.md`, and the
alignment table in `meeting notes/2026-08-19-resolution-tradeoff-vit-vs-vla-and-midterm-report.md`.

This records **status and evidence**, not a schedule. What to attack next is a
decision; this is the input to it.

A caveat that belongs at the top: none of those sources is the grant itself.
`scope-clarification.md` names the primary sources as the seven grant design
PDFs plus Prof. Lai's reference repository. Everything below is the grant *as
restated in our own documentation*, and one number — the ">= 99 % fail-safe
correctness" target — appears in `guardrail/kpi.py` but in no markdown document
in this repository.

---

## Closed since the 2026-08-19 meeting

| Item | Evidence |
|---|---|
| **Per-object stand-off is a real rule (WP1)** | `SubjectStandoff` is a P0 constraint type; `policies/follow_pedestrian.yaml` carries 10 m for a person and 5 m for anything else. Removing those rules changes `policy_hash`, so it is genuinely covered by the signed policy — the exact thing the meeting said it was not. |
| **SITL wired to the WP4 machinery** | `sitl/run_sitl_demo.py` calls `build_manifest()` and `guardrail/kpi.py`, replacing the ad-hoc `kpi_ok`. |
| **`canonical-hil` gate opens on evidence** | `check_hil_evidence()` requires a ROS distro, a MAVROS node and `fcu_connected`; `build_manifest` refuses the label with a scene file present. |
| **Occupancy map covers the cruise band** | Was sampled at 15-55 m AGL and therefore held buildings only. Rebuilt over 6-14 m; a street mask separates roads from open ground. |
| **The KPI is measured, not inferred** | `emitted_violations` records the Shield's re-check of the flown action. All six canonical runs now report `p0_ticks_not_measurable: 0`. See `docs/FINDING-the-kpi-was-never-measured.md`. |
| **Object width derived per class** | `implied_range_from_width()` assumed 4.0 m (a car), which would report a 0.5 m pedestrian at roughly 8x their true distance. |

Current measured position, canonical topology, `code_revision 4bafc63fab21`:

| | P0 escape | P0 ticks | unmeasurable | fail-safe | KPI-grade |
|---|---|---|---|---|---|
| shield off | 0.626506 | 52 | 0 | 0.0 | yes |
| shield on | **0.0** | 134 | 0 | 1.0 | yes |
| shield on + dynamic NFZ | **0.0** | 217 | 0 | 1.0 | yes |

---

## Correction, 2026-08-31

`det_hit_rate` is a detector-LIVENESS rate, not a tracking-accuracy rate: it
counts inferences that produced any box. Claims elsewhere in this repository
that cite it as evidence the tracker held the right vehicle do not support that
conclusion. `frac_on_target` now measures it properly. Re-measured, the run with
the best reported hit rate (0.995) was tracking the wrong vehicle for 24 % of
its mission. See `docs/FINDING-the-hit-rate-was-not-a-hit-rate.md`.

The SITL KPI figures are unaffected - no detector is in that loop.

## Update, 2026-09-01 — branch `contract-gaps`

Five of the items below are closed, and one of them was stale before the branch
started. What follows the table is what is genuinely still open.

| Was | Now | Evidence |
|---|---|---|
| **2. 10 m pedestrian stand-off never flown** | **CLOSED, and already was** | Flown 2026-08-31 on the canonical rail: shield off 7.07 m / 2.3 s inside the ring, shield on 14.95 m / 0.0 s. This entry was stale when it was written. |
| **3. Scenario sweep harness** | **CLOSED** | `experiments/sweep_scenarios.py` + `scenarios.yaml`. 12 scenarios, headless, ~1 s. 11 pass, 1 recorded known failure. |
| **4. Two KPIs never measured** | **CLOSED** | `mean_repair_magnitude_mps` and `mean_time_to_safe_s` in `guardrail/kpi.py`; all 42 delivered runs rescored by `tools/rescore_kpis.py` with every stored P0 figure reproduced exactly. |
| **5. Corridor and time-window constraints** | **CLOSED** | `Corridor` is the sixth constraint type; `valid_time` is a field on every rule. Both absent from the reference implementation too, so this is ahead of it rather than level. |
| **6. Signed policy bundle and WGS84** | **CLOSED** | `guardrail/bundle.py` matches the reference layout byte-for-byte; `guardrail/projection.py` accepts lat/lon additively. `policies/wgs84_taipei.yaml` is the first geographic policy. |
| **7. Constraint Summary Pack** | **CLOSED** (replay bundles still open) | `ConstraintCompiler.summary_pack()`. It also fixed a real gap: `build_prompt` emitted only fences, altitude and speed, so the pilot was never told about the 10 m stand-off the Shield enforces against it. |

**Two KPI results worth quoting.** Shield ON: zero unsafe-position episodes.
Shield OFF: 4.1 s to get out. Mean repair magnitude 3.5–4.1 m/s on the shielded
canonical runs.

**A defect the new work found.** `AltitudeFix` aimed a recovery climb at the band
boundary exactly, making it a decaying exponential that converged on the floor
without crossing: from 3 m against a 10 m floor it reached 9.99973 m after thirty
seconds and would have stayed below forever. The P0 escape rate was 0 throughout,
because the emitted action was legal every tick — the ACTION was fine and the
STATE never became safe. Nothing in the KPI set could see that before
`mean time to safe`, and the first sweep that ran found it. Fixed; recoveries now
aim a margin inside the band and arrive in about twelve seconds.

## Update, 2026-09-07 — three more closed, and one new defect class

| Was | Now | Evidence |
|---|---|---|
| **7. Replay bundles** | **CLOSED** | `guardrail/replay.py`. `verify_replay()` reloads the policy from the archived IR and **recomputes the KPIs from the archived log**, so "replayable" means re-derivable rather than "the files are in one place". `demo/follow_vlm.py` and `sitl/run_sitl_demo.py` now write one per flight. |
| **9. Body-frame vs world-frame `Action4D`** | **CLOSED** | `guardrail/frames.py` is the single boundary; `tests/test_frame_contract.py` checks our conversion against `vlaguard_common.body_to_local_ned` **by running theirs**, at eight headings. Both contracts are right in their own frame. |
| **Report contradictions** (both) | **CLOSED** | `object_width_m` limitation marked resolved with its date; the control-loop gate paragraph now says which half was met and which was not. |

**What closing item 7 immediately found.** Of the **44 scored runs on disk, only
2 can be bundled at all** — every other one was flown under a policy revision no
longer in `policies/`, so its numbers cannot be re-derived from this repository.
That is not a bug in anything; it is what happens when the artefact is assembled
later instead of by the run. Both remaining runs bundle and verify, and every
future flight packages itself.

**What closing item 9 found — corrected the same day.** The first version of
this entry said the units differ from the reference: deg/s here, rad/s there, a
factor of 57.3. That was wrong. Both are **rad/s**; the frame is the only
difference, and it converts exactly.

The real defect was inside our own package. `guardrail/models.py` declared
`yaw_rate` as deg/s in a comment while all six sites that ENFORCE or produce it
read radians — and two adapters had believed the comment and applied
`math.radians()` to an already-radian value, dividing every commanded yaw by
57.3 on the canonical rail.

Nothing burned: the stub pilot that flies that rail has never commanded a
non-zero yaw rate (0.0000 max across `sitl_shield_on`, `ros2_shield_on`,
`sitl_ped_on`), so **no stored KPI figure changes**. Fixed in four files and
pinned by tests that build a Shield and ask it, rather than reading a comment.
See `docs/FINDING-the-contract-disagreed-with-itself-about-yaw.md`.

I introduced the wrong version of this claim while closing the item, wrote a test
that asserted it, and shipped it green beside `test_yaw_cap_compares_degrees_with_degrees`,
which has pinned the opposite convention since 17 August. **A test that asserts a
comment is not a test.**

**A third scorer, found the same way.** `tools/deck/build_sept_deck.js` keeps its
own copy of the tracking projection so slides are re-derived from the logs rather
than from stored numbers. When the Python scorer learned that an empty `truth.pts`
means UNSCORABLE, the JS copy kept falling through to `tgt_x/tgt_y` — the car. On
the next pedestrian flight it would have printed **0.406 on target** on a slide
while `metrics.json` beside it said **0.931**. Fixed, and the two are now compared
against each other by `tests/test_deck_scorer_parity.py`, which runs the deck's
own JavaScript under node on rows built in Python.

**And item 3 was not the item it said it was.** The entry read
"`follow_pedestrian.yaml` permits descent to 4 m but loads the 6-14 m map;
`ground_2to4.npz` exists for that altitude and nothing chooses it" — filed as
tidiness. Measured, both halves are wrong and the truth is worse:

- `ground_2to4` ends at **exactly 4 m**, where the policy's band begins, so it
  never applies to a 4-10 m flight and wiring it up would have changed nothing;
- **nothing maps 4-6 m at all** — the four band maps are not contiguous, and the
  hole is inside the band this policy permits;
- neither map contains the other: **300 cells** are occupied at 2-4 m and clear
  at 6-14 m, while the building at grid cell (64, 40) — the documented 9 m
  collision — is in the cruise map and open ground at 2-4 m;
- `ground_0to2` is **100 % occupied**; it is the ground plane, and unioning it in
  would block every cell.

`demo/occ_bands.py` now unions every band covering the policy's altitude envelope,
refuses to union maps on different grids, rejects a band that is more than 90 %
occupied, and **names the uncovered slices** on start-up. Closing it properly
needs two map rebuilds (4-6 m, and 2-14 m as one band) and the simulator running.
See `docs/FINDING-the-map-was-true-for-the-wrong-altitude.md` and
`tests/test_occ_bands.py`.

### A defect class, not a defect: silence that reads as success

Three findings in six days share one shape, and it is worth naming because the
next one will look like the last three.

| What was silent | What it looked like |
|---|---|
| A stand-off rule bound to a class no phrase could produce | zero violations |
| A scorer comparing every box to the car after the subject became a person | `frac_on_target` 0.406, read as a detector failure |
| An estimator gating out the closest measurements | a clean escape rate |
| A comment declaring deg/s while six sites enforced rad/s | two green test suites, opposite conventions |
| A 2-D obstacle map true for one altitude band, loaded by a policy that flies below it | a map, correctly registered, for the wrong altitude |

In each case the artefacts were **complete, consistent and wrong**, and in each
case the check that would have caught it was cheap. The countermeasure now in
the code is the same three times: make the silent case *say something* —
`det_unscorable` for a detection with no truth, `range_agreement` for a served
position the camera disagrees with, and a start-up refusal for a rule that can
never bind.

The habit this asks for is narrow enough to state: **a zero is a claim, and a
claim needs the same checking as any other.**

## Open

### 1. Perception on the KPI-grade rail — WP4

The report calls this *"the largest remaining piece of work"*. ArduPilot SITL has
no renderer, so every tracking result — detector rate, hit rate, the follow
demos, the pedestrians — is Project AirSim evidence and **not grade-eligible**.
The KPI-grade runs are stub-pilot waypoint missions.

Closing it means feeding AirSim imagery to a Guardrail driven over MAVROS. That
is the `HIL_GPS` / `HIL_SENSOR` bridge the reference architecture describes, and
it is the single largest item on this list.

### 2. [CLOSED 2026-09-01] The 10 m pedestrian stand-off has never been flown — WP1

The rule exists and is enforced. The **demonstration used a 5 m rule against a
car**. The Prof's actual request — hold 10 m from a person — has not been flown,
and three things stand between here and there:

- a pedestrian must be the tracked *subject*, not scenery (they are decoration
  by explicit decision, and `demo/pedestrians.py` says so);
- **depth is quantised to whole metres** (`16UC1`), which supports proximity
  detection but not a metric standoff — this limitation is *not* marked resolved
  in the report;
- the camera's blind spot at 9 m cruise is 7.7 m, which is *inside* the 10 m
  ring, so the flight needs a lower altitude band.

Cheapest item with the most direct line to the meeting.

### 3. [CLOSED 2026-09-01] Scenario sweep harness — WP4

`MIDTERM-REPORT-Aug2026.md` states verbatim: **"Scenario sweep harness not
built."** The KPI machinery exists per-flight; nothing sweeps a scenario library.

### 4. [CLOSED 2026-09-01] Two named KPIs have never been measured

`scope-clarification.md` lists the contractual KPI set as *"P0 escape rate = 0,
mean repair magnitude, mean time to safe"*. Only the first has ever been
computed. **No target was ever recorded** for the other two, and no number for
them appears in any document. `guardrail/kpi.py` reports intervention *counts*,
which is not a magnitude.

### 5. [CLOSED 2026-09-01] Corridor and time-window constraints — WP1

Named in the DSL taxonomy; absent from our five types (`PolygonFence`,
`AltitudeEnvelope`, `KinematicEnvelope`, `ObstacleClearance`, `SubjectStandoff`).

### 6. [CLOSED 2026-09-01] Signed policy bundle and WGS84 frame — WP1

The reference implementation emits a signed `tar.gz` (policy id, hash,
generation, changelog, signature) and treats **WGS84 lat/lon as canonical**. We
hash in memory and work in local metres. The DSL spec says WGS84.

### 7. [CLOSED 2026-09-07] Two named artefacts — WP2 / WP4

The **Constraint Summary Pack** is now produced (`ConstraintCompiler.summary_pack()`,
`write_summary_pack()`). **Replay bundles** (WP4) are still not: the signed
POLICY bundle exists, but nothing packages a flight - log, metrics, manifest,
policy, KPI - into one replayable artefact. Our `guardrail/compiler.py` is a
reduced compiler; the reference lists WP2 as unbuilt, so we are ahead there, but
the named artefact still does not exist.

### 8. The wedge: a repaired action can still be a stuck mission

Now pinned as `standoff-wedge` in `experiments/scenarios.yaml`, marked
`expect: known_failure` so it is reported every sweep instead of living in a
document. A subject sitting exactly on the route makes every heading that
progresses also close the range. The Shield holds - no illegal action, the ring
is never broken - and the mission never arrives. The gap is between "the action
was repaired" and "the trajectory was sensible", and closing it needs a planner
that can route AROUND a constraint rather than a filter that can only veto.

### 9. [CLOSED 2026-09-07] Body-frame versus world-frame `Action4D` — WP1

Ours is world-frame (`vx` North, `vy` East); `vlaguard_common.Action4D` is
body-frame. *"Both cannot be right, and no test compares them."* This is a
contract mismatch with the reference implementation, not a bug in either.

---

## [FIXED 2026-09-07] Report contradictions, before a reviewer finds them

Both were inside `docs/MIDTERM-REPORT-Aug2026.md` and both are now corrected in
place, marked with the date rather than silently rewritten:

1. **`object_width_m`** is described as still hardcoded at 4.0 (limitation 5,
   with the 8x pedestrian error) *and* as derived per class (section 10.3). Only
   one can be true; the code says per-class, so the limitation text is stale.
2. **The control-loop gate.** Acceptance thresholds were fixed before the runs
   at "detector >= 4.0 Hz and control loop >= 9.5 Hz". The reported loop rates
   are 8.78 / 8.07 / 7.83 Hz, and the report declares the threshold met while
   citing only the detector rates. The loop gate was **not** met.

Also worth a note: `det_hz` under recording spans 3.4-4.6 Hz across runs and
does not reliably clear its own 4.0 Hz gate. Unrecorded runs clear it
comfortably. This is pre-existing and unrelated to the Shield.

---

## Not in scope, recorded so it stays that way

- **Training a VLA.** The grant names it optional; the VLA is a pluggable
  external dependency. *"The word 'VLA' names the technology we CONSTRAIN — not
  a thing we build."*
- **Obstacle avoidance, navigation, CV, GCS, dynamic-NFZ sourcing** — Prof.
  Lai's team owns the application layer. The Shield enforces *policy* geometry.
- **Modifying the core flight stack.** ROS, MAVLink, ArduPilot's built-in
  GeoFence and PX4 stay as they are.
- **Hardware flight.** A stretch goal, explicitly not a contractual gate.
- **Colour-tracking retraining**, proposed at the meeting — it would remove the
  open-vocabulary language interface the grant title depends on. Recommend
  declining.
