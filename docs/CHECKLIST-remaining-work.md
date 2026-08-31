# Remaining work, against the grant and the meeting record

**Date:** 2026-08-28
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

## Open

### 1. Perception on the KPI-grade rail — WP4

The report calls this *"the largest remaining piece of work"*. ArduPilot SITL has
no renderer, so every tracking result — detector rate, hit rate, the follow
demos, the pedestrians — is Project AirSim evidence and **not grade-eligible**.
The KPI-grade runs are stub-pilot waypoint missions.

Closing it means feeding AirSim imagery to a Guardrail driven over MAVROS. That
is the `HIL_GPS` / `HIL_SENSOR` bridge the reference architecture describes, and
it is the single largest item on this list.

### 2. The 10 m pedestrian stand-off has never been flown — WP1

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

### 3. Scenario sweep harness — WP4

`MIDTERM-REPORT-Aug2026.md` states verbatim: **"Scenario sweep harness not
built."** The KPI machinery exists per-flight; nothing sweeps a scenario library.

### 4. Two named KPIs have never been measured

`scope-clarification.md` lists the contractual KPI set as *"P0 escape rate = 0,
mean repair magnitude, mean time to safe"*. Only the first has ever been
computed. **No target was ever recorded** for the other two, and no number for
them appears in any document. `guardrail/kpi.py` reports intervention *counts*,
which is not a magnitude.

### 5. Corridor and time-window constraints — WP1

Named in the DSL taxonomy; absent from our five types (`PolygonFence`,
`AltitudeEnvelope`, `KinematicEnvelope`, `ObstacleClearance`, `SubjectStandoff`).

### 6. Signed policy bundle and WGS84 frame — WP1

The reference implementation emits a signed `tar.gz` (policy id, hash,
generation, changelog, signature) and treats **WGS84 lat/lon as canonical**. We
hash in memory and work in local metres. The DSL spec says WGS84.

### 7. Two named artefacts never produced — WP2 / WP4

"Constraint Summary Pack" (WP2) and "replay bundles" (WP4) are each named once
as deliverables and never claimed as delivered. Our `guardrail/compiler.py` is a
reduced compiler; the reference lists WP2 as unbuilt, so we are ahead there, but
the named artefact still does not exist.

### 8. Body-frame versus world-frame `Action4D` — WP1

Ours is world-frame (`vx` North, `vy` East); `vlaguard_common.Action4D` is
body-frame. *"Both cannot be right, and no test compares them."* This is a
contract mismatch with the reference implementation, not a bug in either.

---

## Report contradictions to fix before a reviewer finds them

Both are inside `docs/MIDTERM-REPORT-Aug2026.md`:

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
