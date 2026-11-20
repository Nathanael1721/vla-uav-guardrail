# CityLife follow on the AGX Orin, 19 Nov 2026 (one-path guard, WGS84 bundles)

Seven red-car follow flights of the hand-written controller
(demo/follow_vlm.py: OWL-ViT detector, identity lock, 10 Hz controller,
Shield) run ON the Jetson AGX Orin against Project AirSim CityLife on the
desktop (deploy/orin/run_follow_controller.sh, deploy/orin/start_sim_for_jetson.ps1
-SimMaxFps 30 -SimOffscreen), code bfb4973, clean checkout, 420 s after the
start gate.

They are the rig flights the 18-19 Nov code owed:

- **WP3-22 / X-13, one evaluation path:** FenceGuard measures building
  clearance with the Shield's ruler (`guard_field: "shield"`, checked after
  every flight by the run script).
- **WGS84-only flights:** each run flew a v0.1.3 bundle (WGS84 points),
  signature verified.
- **WP3-04 / ARCH-04, body frame:** the city follow hands the Shield the body
  action at the pose's measured heading. follow_vlm keeps no
  `heading_missing_ticks` counter (the heading comes from the simulator pose
  on every tick); that counter exists on the pymavlink SITL rail only, whose
  flight is still owed.

**Not KPI-grade, by design:** the topology is `projectairsim-remote-sim`
(Project AirSim's simple_flight, no ArduPilot), not the grant's hil topology
(guardrail.manifest.is_kpi_grade). These are functional evidence for the
follow and the guard, not acceptance-KPI episodes.

## Files

| File | What it is |
|---|---|
| `runs_summary.json` | Per run: revision, within-30 m, P0 escape rate, detector rate, guard field, off-map ticks, contacts, bundle and signature, VLA authority shares |
| `onepath_guard_replay.json` / `.txt` | `python tools/onepath_guard_replay.py agx_onepath_shield_01 ... _05 --json ...`: the five onepath runs' logged ticks replayed through the grid-scan guard and the Shield-ruler guard |
| `standoff_edge_replay.json` / `.txt` | `python tools/standoff_edge_replay.py --root <lab checkout> --json ... --txt ...` (2026-11-20): nfz3b_agx_hier_aerovla_wgs84_20261119 replayed open-loop through FenceGuard before and after the stand-off edge. Hold-tick commands are REBUILT (the log keeps none); the before-guard reproduces the logged hold on 357 of the 403 hold ticks |
| `diagnosis_20261119.md` | Why onepath_03 and nfz3b scored low, and the after-mission contacts; every claim re-checked by two adversarial verifiers |

Run folders: `demo/out/<tag>/` (lab checkout, gitignored, no frames).
Full runs with view frames and PIP videos (`<tag>_pip.mp4`):
`F:/vla_drone_offload/jetson_runs/<tag>/`.

## Results

| Run | Policy (bundle v0.1.3) | Within 30 m | P0 escape | Detector | Mission contacts | After-mission contacts |
|---|---|---|---|---|---|---|
| agx_onepath_shield_01 | follow_car_citylife_nfz | 0.969 | 0.0 | 2.47 Hz | 0 | 1 |
| agx_onepath_shield_02 | follow_car_citylife_nfz | 0.954 | 0.0 | 2.39 Hz | 0 | 1 |
| agx_onepath_shield_03 | follow_car_citylife_nfz | 0.472 | 0.0 | 2.40 Hz | 0 | 1 |
| agx_onepath_shield_04 | follow_car_citylife_nfz | 0.943 | 0.0 | 2.32 Hz | 0 | 1 |
| agx_onepath_shield_05 | follow_car_citylife_nfz | 0.917 | 0.0 | 2.27 Hz | 0 | 6 (incl. a pedestrian) |
| agx_hier_aerovla_wgs84_20261119 | follow_car_citylife_nfz, AerialVLA hierarchy (:8790) | 0.987 | 0.0 | 2.55 Hz | 0 | 1 |
| nfz3b_agx_hier_aerovla_wgs84_20261119 | follow_car_citylife_nfz3b, AerialVLA hierarchy | 0.345 | 0.0 | 2.64 Hz | 0 | 132 (incl. a pedestrian) |

- Onepath n=5: within 30 m 0.969 / 0.954 / 0.472 / 0.943 / 0.917 (median
  0.943); 3 Nov grid-guard baseline agx_follow_01 0.952 (one flight).
- Guard replay: the flown guard's replay equals the logged mode on 99.97 %
  of 20,971 ticks (all five flew `shield`); Shield field against grid scan:
  building brake differs on 0.30 % of ticks, decision on 1.13 %.
- nfz3b: 0 off-map ticks and 0 mission contacts, against 3484 off-map ticks
  and 1919 building contacts on nfz3b_agx_hier_aerovla_20261103 (old map);
  4 Nov (989f967) scored 0.494.

## Findings (diagnosis_20261119.md)

- **onepath_03 (0.472):** one loss of the car at t = 186 s, just after its
  west turn. During ten consecutive slow detector cycles (772-1050 ms) the
  strict identity lock rejected the correct car box by 2 px over its 92 px
  pixel gate, while FenceGuard was clear at scale 1.0. The reacquirer then
  refused the car ("short pass", "not moving" while it stood at a stop,
  "too far") until t = 407 s; a replay of the Reacquirer on the logged
  detections reproduces every streak. None of the 28 ticks where the
  Shield-ruler brake differs from the grid guard falls in t = 170-201 s: not
  a regression of the one-path change (open-loop; closed-loop effects not
  excluded). Why inference was slow during the onepath flights, and not in
  the hier flight after them, is not established.
- **nfz3b (0.345):** the car was lost while FenceGuard held the drone for
  40 s (403 ticks, cause "fence") 2.65 m from nfz-north-street's margin
  ring: inside the 3 m stand-off, `FenceGuard.gate()` returns 0 for every
  velocity, including moving away, and `slide()` finds no detour along the
  commanded aim. The car drove 100 m on. The stand-off rule is unchanged
  since 989f967 (4 Nov); an open-loop replay holds under both guards.
  controller_fallback (53 %) is the planner waiting at a junction after the
  loss. The car never entered a zone.
- **The Shield's P0 count is against its own estimate:** in nfz3b the 5 m
  standoff-any rule, measured against the car's true position (horizontal,
  drone at about 8 m), was inside 5 m on 102 ticks (minimum 0.22 m) and
  flagged on 2.
- **Landing:** 0 contacts in the mission window on all 7 flights. After the
  mission, 2 flights list a level pedestrian among their contacts, both at
  landing site (192, 30), where pedestrians walked within a few metres of
  the descending aircraft. The landing site choice (demo/landing.py) uses
  static maps and car lanes only, and the vertical descent and land_async
  bypass the Shield; per-contact times after the mission are not logged.
  Pedestrian contacts at landing also appear in five runs from 30 Oct to
  3 Nov. Open safety item: the landing phase is outside the guardrail's
  pedestrian awareness.
