"""
Write docs/MID-EVALUATION-REPORT-Sep2026.md from docs/data/eval_sep2026.json.

The prose lives here; every figure is read from the numbers file that the two
September decks also read, so the report and the slides cannot disagree. Render
it with the nathan-deck style and export Word + PDF:

    python tools/build_eval_data.py
    python tools/build_mideval_report.py
    python tools/report_to_html.py docs/MID-EVALUATION-REPORT-Sep2026.md --style nathan
    powershell -File tools/office_to_pdf.ps1 -Path docs/MID-EVALUATION-REPORT-Sep2026.html

CORRECTED 2026-10-06, IN THE GENERATOR ONLY. The report this wrote in September
was delivered, so it is not regenerated silently; its corrections go into the
final report's correction note. What changed here, so a rebuild cannot print
the old claims again:
  * topology: the desktop ArduPilot SITL + MAVROS 2 rail is the grant's `dev`
    topology, not "canonical HIL" - the grant takes KPI figures from `hil`
    (Jetson Orin). Runs stored as `canonical-hil` are counted as `dev`.
  * WP2: the Constraint Summary Pack is generated and saved; no VLA reads it.
  * WP4: a headless scenario sweep, replay bundles and manifests exist; the
    grant's stress-testing harness and its hil campaign do not - "Partial".
  * fail-safe trigger correctness: the grant's target is >= 0.99, not 1.0.
  * the P0 escape figure is one of five acceptance KPIs, measured on dev-
    topology demo and SITL flights, not on Stress Testing runs in hil.
"""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
E = json.loads((ROOT / "docs/data/eval_sep2026.json").read_text(encoding="utf-8"))
OUT = ROOT / "docs/MID-EVALUATION-REPORT-Sep2026.md"


def f1(v):
    return f"{v:.1f}"


def f2(v):
    return f"{v:.2f}"


def yn(v):
    return "Yes" if v else "No"


def main():
    K = E["kpi"]["runs"]
    R, D, L, S, G, U, RA = E["retarget"], E["detector"], E["lock"], E["sweep"], E["repo"], E["unflown"], E["rails"]
    on, dyn, ped, off, poff = (K["ros2_shield_on"], K["ros2_shield_on_dynamic"], K["ros2_ped_on"],
                               K["ros2_shield_off"], K["ros2_ped_off"])
    b, a, af = R["before_ped_matched"], R["after_ped_matched"], R["after_ped_full"]
    cov, chk = R["coverage_after_full"], R["subject_range_check"]
    tf, tc = G["tests_fast"], G["tests_coverage"]
    n_scen = S["counts"]["pass"] + S["counts"]["fail"] + S["counts"]["known_failure"]
    fl = D["flights"].values()
    det_lo, det_hi = min(x["det_hz"] for x in fl), max(x["det_hz"] for x in fl)
    loop_lo, loop_hi = min(x["loop_hz"] for x in fl), max(x["loop_hz"] for x in fl)
    ow, gd = D["bench"]["owlvit"], D["bench"]["gdino"]
    ctrl = E["kpi"]["unshielded_controls"]
    # Rail counts under today's topology names: a stored "canonical-hil" is the
    # grant's "dev". Both keys are summed so an older eval file still counts.
    n_dev = RA["counts"].get("dev", 0) + RA["counts"].get("canonical-hil", 0)
    fc, fp = R["figure_frames"]["car"], R["figure_frames"]["person"]

    md = f"""# Guardrail — Mid-Evaluation Progress Report

**Semantic-Spatial Translation and Safety-Constrained VLA for ArduPilot UAVs** · ITRI · National Taipei University of Technology · September 2026 · software version {G["version"]}

Advisor: · Author: · Numbers generated {E["generated"]} by `tools/build_eval_data.py`

---

## 1. Summary

- **The Guardrail flies ArduPilot through MAVROS 2.** {E["kpi"]["flights_kpi_grade"]} runs on ArduPilot SITL driven over MAVROS 2 on ROS 2 Jazzy met the run gate as it stood at flight time. That one-desktop configuration is the grant's `dev` topology; the grant takes contractual KPI figures from `hil` (VLA and Shield on a Jetson Orin), so these runs are evidence, and count as KPI figures only under a written waiver from the PI.
- **The five acceptance KPIs are computed** on those dev-topology runs - mission success, P0 violation escape rate, fail-safe trigger correctness, mean repair magnitude and mean time to safe - not yet in Stress Testing runs in the hil topology. The other work-package KPIs are not measured yet.
- **P0 violation escape rate was 0.0 on {E["kpi"]["flights_escape_zero"]} shielded demo and SITL flights.** These are not contract KPI figures, and P0 escape is one of five acceptance KPIs. The {len(ctrl)} deliberately unshielded control flights read {f2(ctrl[0]["rate"])}, which is what they exist to show.
- **Perception is the open half.** Tracking runs on Project AirSim, a functional rail outside the contractual gate; the pedestrian detector does not beat a centre-constant null; and no rule protects pedestrians other than the one being followed.
- **{len(G["commits_since_meeting"])} changes since the 2 September meeting**, including four silent defects found and fixed, a steadier tracker, and two published claims corrected (Section 8).

## 2. Scope and acceptance criteria

| Work package | Deliverable | Status | Evidence |
|---|---|---|---|
| WP1 | Policy DSL and intermediate representation | Built | Six constraint types, `valid_time` on every rule, signed policy bundle |
| WP2 | Prefix constraint compiler | Partial | Constraint summary pack is generated and saved; no VLA reads it yet |
| WP3 | Suffix Safety Shield | Built, dev topology | {E["kpi"]["flights_kpi_grade"]} ArduPilot SITL + MAVROS 2 runs (`dev`, not `hil`) |
| WP4 | Stress testing and evidence | Partial | {n_scen}-scenario headless sweep, replay bundles, per-flight manifests; the stress-testing harness and the hil campaign do not exist yet |

The five KPIs are named in the grant. They are computed by one function, `guardrail.kpi.compute`, used identically by the flights, the scenario sweep and the replay verifier.

| KPI | Meaning | Target |
|---|---|---|
| Mission success | The mission reached its goal | — |
| P0 violation escape rate | Detected P0 violations that reached the actuator | 0 |
| Fail-safe trigger correctness | The fail-safe fired when, and only when, it should | ≥ 0.99 |
| Mean repair magnitude | Average size of a Shield correction, m/s | — |
| Mean time to safe | Time from an unsafe state back to a safe one, s | — |

## 3. System architecture

A camera frame and an operator's phrase enter an open-vocabulary detector (OWL-ViT). An instance lock keeps the controller on one object; a target-state estimator turns bearing and range into a velocity command. That command — the **Action4D**, `(vx north, vy east, vz up, yaw_rate)` at 10 Hz with `yaw_rate` in rad/s — is the Shield's only input. The Shield checks it against the policy, repairs or brakes, writes an audit record, and passes the result to MAVROS 2 and ArduPilot.

The action source is deliberately swappable. OpenVLA-7B, AerialVLA, a behaviour-cloned policy and the hand-written controller have all flown through the same slot. The safety argument does not depend on which model is driving.

| Rail | Flights | Camera | KPI-grade |
|---|---|---|---|
| Project AirSim (Unreal) | {RA["counts"].get("projectairsim-single-host", 0)} | Yes | No |
| ArduPilot SITL · pymavlink | {RA["counts"].get("ardupilot-sitl-pymavlink", 0)} | No | No |
| ArduPilot SITL · MAVROS 2 · ROS 2 Jazzy (grant: `dev`) | {n_dev} | No | Only under a PI waiver |

## 4. Progress by work package

### WP1 — Policy DSL

| Constraint | What it enforces |
|---|---|
| `PolygonFence` | Keep-out zone, optional altitude band |
| `AltitudeEnvelope` | Floor and ceiling above ground |
| `KinematicEnvelope` | Horizontal speed, climb rate and yaw-rate caps |
| `ObstacleClearance` | Minimum distance from every mapped building |
| `SubjectStandoff` | Minimum distance from the followed subject, selected by class |
| `Corridor` | Keep-in route with width and altitude band |

Every rule may carry `valid_time`. The policy hash is written into every audit record, and a signed bundle refuses a tampered IR, a foreign manifest or a truncation on reload. (Corrected 2026-10-06: until then the bundle's "signature" was a placeholder; it is now an Ed25519 signature from a lab development key, pending the PI's choice of signing authority.)

### WP2 — Prefix constraint compiler

Operator text is compiled into a structured mission and a constraint summary pack, rendered from the same policy object the Shield enforces. The pack is generated and saved; no VLA reads it yet, so the prefix half of the guardrail has not yet constrained a model.

### WP3 — Suffix Safety Shield

Each tick the Shield predicts the next 3 s, checks every rule, applies the smallest legal repair, re-checks the repaired action, and falls back to a recovery heading or a brake. On the ArduPilot SITL + MAVROS 2 rail (`dev` topology) its fail-safe trigger correctness is {f1(on["failsafe_trigger_correctness"])} and its mean repair magnitude {f2(on["mean_repair_magnitude_mps"])} m/s (maximum {f2(on["max_repair_magnitude_mps"])} m/s).

### WP4 — Stress testing and evidence

The headless scenario sweep scores every scenario with `guardrail.kpi.compute`: **{S["counts"]["pass"]} pass, {S["counts"]["fail"]} fail, {S["counts"]["known_failure"]} known failure.**

| Scenario | Status | P0 escape rate |
|---|---|---|
""" + "\n".join(f"| `{r['id']}` | {r['status']} | {r['p0_violation_escape_rate']} |" for r in S["results"]) + f"""

`nfz-head-on-control` is an unshielded control and passes by failing its safety gate. `standoff-wedge` is the one open defect, pinned as a scenario: a subject exactly on the route makes every forward direction close the range, so the Shield never emits an illegal action but the mission cannot get past.

Every flight writes a manifest (code revision, detector weights hash, policy hash, seed, simulator speed-up, topology). A replay bundle re-derives the flight's KPIs from its own log. Tests: **{tf["passed"]}/{tf["total"]} fast** and **{tc["passed"]}/{tc["total"]} coverage**, both run on {E["generated"]}.

## 5. KPI results — dev topology (ArduPilot SITL + MAVROS 2)

These runs are the grant's `dev` configuration, not `hil`; they are evidence of the Shield's behaviour and become contractual figures only under a written PI waiver.

| KPI | No-fly zone | Dynamic no-fly zone | Pedestrian stand-off | Unshielded control |
|---|---|---|---|---|
| P0 escape rate | **{f1(on["p0_violation_escape_rate"])}** | **{f1(dyn["p0_violation_escape_rate"])}** | **{f1(ped["p0_violation_escape_rate"])}** | {f2(off["p0_violation_escape_rate"])} |
| Fail-safe correctness | {f1(on["failsafe_trigger_correctness"])} | {f1(dyn["failsafe_trigger_correctness"])} | {f1(ped["failsafe_trigger_correctness"])} | {f1(off["failsafe_trigger_correctness"])} |
| Mission success | {yn(on["mission_success"])} | {yn(dyn["mission_success"])} | {yn(ped["mission_success"])} | {yn(off["mission_success"])} |
| Mean repair magnitude (m/s) | {f2(on["mean_repair_magnitude_mps"])} | {f2(dyn["mean_repair_magnitude_mps"])} | {f2(ped["mean_repair_magnitude_mps"])} | — |
| Unsafe episodes | {on["time_to_safe_episodes"]} | {dyn["time_to_safe_episodes"]} | {ped["time_to_safe_episodes"]} | {off["time_to_safe_episodes"]} |
| Mean time to safe (s) | — | — | — | {f1(off["mean_time_to_safe_s"])} |
| Time inside no-fly zone (s) | {f1(on["nfz_s"])} | {f1(dyn["nfz_s"])} | {f1(ped["nfz_s"])} | {f1(off["nfz_s"])} |

With the Shield on, the aircraft never entered an unsafe state, so mean time to safe has no episodes to average; it is measured on the control run.

![ArduPilot SITL + MAVROS 2 (dev topology), same no-fly-zone mission. Left, shield off: straight through the zone. Right, shield on: the path skirts it and still reaches the goal.](img/mideval/hil_nfz_off_on.png){{width=500}}

On the pedestrian stand-off run the 10 m rule moved the closest approach from **{f2(poff["standoff_min_range_m"])} m** with the Shield off to **{f2(ped["standoff_min_range_m"])} m** with it on, time inside the ring from {f1(poff["standoff_s"])} s to {f1(ped["standoff_s"])} s, and the escape rate from {f2(poff["p0_violation_escape_rate"])} to {f1(ped["p0_violation_escape_rate"])}. The pedestrian's position on this rail is **declared**, not detected: ArduPilot SITL has no camera.

## 6. Perception and tracking — functional rail

These results come from Project AirSim and are evidence, not contractual KPI figures. Every tracking score is reported beside a null: a "detector" that emits the frame centre and never opens the image.

| Measure | Result | Against |
|---|---|---|
| Instance lock, median box error | {f1(L["city_full"]["box_err_px_median_in_shot"])} → **{f1(L["lock_on"]["box_err_px_median_in_shot"])} px** | null {f1(L["city_full"]["null_centre"])} → {f1(L["lock_on"]["null_centre"])} px |
| Detector latency, idle GPU | OWL-ViT {f1(ow["total_ms_median"])} ms | Grounding DINO {f1(gd["total_ms_median"])} ms |
| Detection score ratio, Grounding DINO / OWL-ViT | person {f2(D["gdino_over_owlvit_person"])}× | taxi {f2(D["gdino_over_owlvit_taxi"])}× |
| Detector rate in flight | {f2(det_lo)}–{f2(det_hi)} Hz | gate {f1(D["gate"]["det_hz_min"])} Hz |
| Control loop in flight | {f2(loop_lo)}–{f2(loop_hi)} Hz | gate {f1(D["gate"]["loop_hz_min"])} Hz |
| Camera flights meeting both gates | {RA["camera_flights_meeting_both_gates"]} of {RA["camera_flights"]} | — |

OWL-ViT stays the detector: Grounding DINO scores higher but runs at roughly a fifth of the rate, and the in-flight detector rate is already near its gate.

### Class-conditional safety

A conventional tracker is given a box and returns an ID. It cannot be told which rule applies, because it never knows what the object is. Here the operator changes a phrase mid-flight — *a yellow car* to *a person* at t+{f1(R["retarget_event"]["t"])} s — the class becomes `pedestrian`, and the enforced stand-off moves from 5 m to 10 m with the same aircraft, policy and hash. The sweep scenario `standoff-reclassified` passes.

![Left, t = {f1(fc["t"])} s, "a yellow car": class {fc["class"]}, 5 m ring. Right, t = {f1(fp["t"])} s, "a person" (p = {fp["score"]:.3f}): class {fp["class"]}, 10 m ring; the subject is a few pixels wide.](img/mideval/retarget_car_person.jpg){{width=620}}

### Tracking steadied

Pedestrian phase, compared over the window both flights flew (t ≤ {round(R["matched_window_s"])} s):

| Measure | Before | After |
|---|---|---|
| Median yaw rate (deg/s) | {f1(b["yaw_dps_median"])} | **{f1(a["yaw_dps_median"])}** |
| Peak yaw rate (deg/s) | {f1(b["yaw_dps_max"])} | **{f1(a["yaw_dps_max"])}** |
| Median box error (px) | {f1(b["box_err_px_median"])} | {f1(a["box_err_px_median"])} |
| Centre-constant null (px) | {f1(b["box_err_px_null_centre"])} | {f1(a["box_err_px_null_centre"])} |
| Ticks | {b["ticks"]} | {a["ticks"]} |

A velocity clamp on the estimator and a single yaw cap removed the spinning. The detector itself did not improve: on the pedestrian phase it still does not beat the null. At the 400×225 capture a 0.5 m person is {f1(D["pedestrian_px_400_capture"]["16m"])} px wide at 16 m — {f2(D["pedestrian_patches"]["16m"][0])} of one 32×32 OWL-ViT patch.

## 7. Updates since 2 September 2026

| Action item | Status | Evidence |
|---|---|---|
| Scenario a tracker cannot run | Done | Phrase retarget, ring 5 m → 10 m; `standoff-reclassified` |
| Architecture diagram with the lock layer | Done | `docs/architecture-v3.svg` |
| More realistic pedestrians | Built, not flown | CityLife level: {U["citylife_level"]["pedestrians"]} walking pedestrians, {U["citylife_level"]["cars"]} driving cars |
| Gazebo SITL | Open | — |
| Real sensor (camera or LiDAR) | Open | Motivated by Section 9 |

Four defects were found that had produced no error and no warning:

1. **An inert stand-off rule.** The policy named class `pedestrian`, the phrase produced `person`, and binding compares exact strings, so the 10 m rule bound zero times on a flight with a human subject. Synonyms are now canonicalised and an unreachable rule refuses start-up.
2. **A stale set-point.** After a retarget the servo kept the car's {f1(D["standoff_setpoint_m"]["car_4.0m"])} m stand-off instead of the {f2(D["standoff_setpoint_m"]["person_0.5m"])} m derived for a person.
3. **A metric a constant could pass.** At a 100 px tolerance, a frame-centre "detector" scored 1.000 on target. The headline is now the median error against a null family.
4. **A verifier that checked five of seven fields.** Two named KPI fields were never emitted, so they were never compared.

The detector camera was also re-examined on the RTX 4090. OWL-ViT resizes every input to 768×768, so capture resolution does not change the forward-pass cost (offline: 400×225 → {f1(D["offline_4090"]["rows"][0]["total_ms"])} ms, 768×432 → {f1(D["offline_4090"]["rows"][1]["total_ms"])} ms). The camera is now configured at 768×432; **it has not been flown**, and in flight the GPU forward pass runs {D["contention"]["gpu_forward_x"][0]}–{D["contention"]["gpu_forward_x"][1]}× slower than offline. The code and documentation are public at version 0.5.0, with a 0.5.1 correction prepared.

## 8. Findings and corrections

This project records a withdrawn claim as a change in its own right. Two were corrected in this period.

> **"The stand-off rule saw 31 of 516 ticks because the position the Shield was served was wrong by tens of metres."** Withdrawn. The 516 count any pedestrian within 10 m; `SubjectStandoff` protects only the subject being followed. On {cov["outside_hfov"]} of the {cov["ticks_person_inside_10m"]} ticks that person was outside the camera's field of view. Against the pedestrian actually under the detection box, the estimate was {f1(chk["est_range_m_median"])} m and the person {f1(chk["person_under_box_range_m_median"])} m away — roughly right.

> **"The 10 m rule fired six times" as a demonstration.** Scored against ground truth, {R["standoff_score_before"]["tp"]} of {R["standoff_score_before"]["fires"]} firings had a real pedestrian within 10 m.

The full record is in `CHANGELOG.md` and the {G["finding_docs"]} finding documents in `docs/`.

## 9. Limitations

- **Bystanders.** No rule protects pedestrians other than the subject, and a forward camera cannot see beside the aircraft: a bystander came within {f2(af["closest_real_m"])} m while the P0 escape rate read {f1(R["p0_escape_after"])}.
- **Pedestrian detection.** {f1(af["box_err_px_median"])} px median error against a {f1(af["box_err_px_null_centre"])} px centre-constant null over the whole pedestrian phase.
- **Rates.** {RA["camera_flights_meeting_both_gates"]} of {RA["camera_flights"]} camera flights meet both the 9.5 Hz loop gate and the 4.0 Hz detector gate.
- **Tracking is not KPI-grade.** The camera rail (Project AirSim) and the ArduPilot rail are different simulators, and neither is the grant's hil topology.
- **Not yet flown.** The 768×432 camera and the CityLife scene.
- **Known failure.** A subject exactly on the route wedges the mission (`standoff-wedge`).

## 10. Plan to completion

1. Feed Project AirSim imagery to a Guardrail driven over MAVROS 2, so tracking evidence becomes contractual.
2. Add a rule for any pedestrian and a sensor that covers the aircraft's sides.
3. Fly the 768×432 camera, then evaluate a narrower field of view: at 45° a 0.5 m person at 16 m covers {f2(D["pedestrian_patches_by_hfov_16m"]["45deg"])} patches instead of {f2(D["pedestrian_patches_by_hfov_16m"]["90deg"])}.
4. Resolve the `standoff-wedge` defect.
5. Add Gazebo as a second physics front-end on the SITL rail.

## Appendix A — Evidence index

| Figure | Artefact | Reproduce |
|---|---|---|
| KPI table (Section 5) | `demo/out/ros2_*/kpi.json` | `bash sitl/run_ros2_demo.sh on` in WSL |
| Escape rate across flights | `demo/out/*/kpi.json` | `python tools/build_eval_data.py` |
| Scenario sweep | `docs/data/scenario_sweep.json` | `python experiments/sweep_scenarios.py` |
| Tracking before / after | `demo/out/retarget_fixed`, `demo/out/retarget_smooth` | `python tools/build_eval_data.py` |
| Bystander visibility | `docs/data/eval_sep2026.json` → `ring_coverage` | `python tools/build_eval_data.py` |
| Instance lock | `demo/out/city_full`, `demo/out/lock_on` | `python tools/build_eval_data.py` |
| Detector benchmark | `docs/data/detector_bench.json` | `python experiments/bench_detectors.py` |
| Tests | `tests/test_*.py` | `python tests/test_shield.py` (each file runs itself) |

## Appendix B — Timeline

| Date (2026) | Milestone |
|---|---|
| 23 June | Project kickoff |
| 3 July | Shield prototype: policy DSL, repair operators, audit log |
| 16 July | OpenVLA-7B and AerialVLA flown through the Shield |
| 3 August | Obstacle clearance becomes a P0 rule |
| 19 August | Midterm review; Guardrail confirmed as the primary deliverable |
| 25 August | First ArduPilot SITL + MAVROS 2 runs through the Shield (dev topology) |
| 1 September | All five acceptance KPIs measured |
| 10 September | Version 0.5.0 published |
"""
    OUT.write_text(md, encoding="utf-8")
    print(f"wrote {OUT.relative_to(ROOT)} ({len(md.splitlines())} lines)")


if __name__ == "__main__":
    main()
