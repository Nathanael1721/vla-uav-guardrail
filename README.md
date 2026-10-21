# Guardrail — a safety layer for language-commanded UAVs

[![version](https://img.shields.io/badge/version-0.5.1-blue)](CHANGELOG.md)
[![P0 escape on the 41 mid-evaluation flights](https://img.shields.io/badge/P0%20escape%2C%2041%20mid--evaluation%20flights-0.0%20%28dev%20evidence%2C%20not%20a%20contract%20KPI%20figure%29-lightgrey)](#p0-escape-on-the-mid-evaluation-flights-and-what-it-does-not-say)
[![tests](https://img.shields.io/badge/tests-1400%2B%20fast%20%2B%2017%20coverage-brightgreen)](tests/)

In the grant's design, a vision-language-action model (VLA) is told to *follow
the yellow car* and answers with a velocity command. **Nothing in that
pipeline knows what a no-fly zone is.**

No VLA flies the demonstrations recorded here. In the city demo
(`demo/follow_vlm.py`) and on the Project AirSim + ArduPilot perception rail,
OWL-ViT finds the car from the words and a hand-written follow controller
turns its box into the velocity command; the ArduPilot SITL rails fly
`guardrail.vla_stub.StubVLA`, a deliberately rule-ignorant stand-in. Real VLAs
have flown through the Shield only in simulation, and none decided at 10 Hz
(the 10 Hz loop reused each decision between inferences; on 4 Aug its yaw
command faded from 1.5 s and was zero after 3 s): OpenVLA-7B (about 0.9 s an
inference, `demo/real_vla_demo.py`) and AerialVLA adapters on classic AirSim
(AirSimNH) in July 2026; AerialVLA on Project AirSim from 16 Jul 2026
(`demo/aerialvla_pas_demo.py`, mostly behind a route planner's pure-pursuit
follower or the 0.55 goal assist) and on 3 Aug (`demo/semantic_demo.py`, with
no planner); and 8 AerialVLA flights on Project AirSim on 4 Aug 2026 (6 with
the Shield on; `demo/semantic_seek.py`), the last VLA-commanded flights, at
one decision every 7-9 s and with a direction hint computed from the true
coordinates in its prompt ([result](docs/RESULT-vla-guardrail-simultaneity.md)).

Guardrail sits between the model and the aircraft. It takes a declarative
safety policy — fences, altitude envelopes, kinematic caps, obstacle
clearances, stand-off rings, corridors — and enforces it on every command at
10 Hz, repairing what it can and refusing what it cannot.

Built for the ITRI grant *Semantic-Spatial Translation and Safety-Constrained
VLA for ArduPilot UAVs* (PI: Prof. Kuan-Ting Lai, NTUT).

---

## The one interface

The Shield's only input is an **Action4D**: `(vx north, vy east, vz up,
yaw_rate)` at 10 Hz, `yaw_rate` in **rad/s**. It does not know whether that came
from a hand-written controller steering by OWL-ViT's boxes, from OpenVLA-7B, or
from a joystick, and that is the point — the safety argument must not depend on
which model is driving.

```python
from guardrail import Action4D, Shield, State, load_policy

shield = Shield(load_policy("policies/follow_pedestrian.yaml"))
shield.set_subject(x, y, subject_class="pedestrian")  # perception supplies position AND class
decision = shield.filter(state, raw_action)            # -> decision.emitted (the repaired Action4D) + audit fields
```

The subject's class matters: `follow_pedestrian.yaml`'s 10 m pedestrian
stand-off binds only to a subject reported as `pedestrian`, and without a class
only its 5 m catch-all rule applies
([the rule that never armed](docs/FINDING-the-standoff-rule-that-never-armed.md)).
`tests/test_public_docs.py` runs this block.

## What is here

| | |
|---|---|
| `guardrail/` | the policy DSL (WP1), prefix compiler (WP2), Safety Shield (WP3), replay bundles (WP4) |
| `demo/` | the flight controller, OWL-ViT grounder, target estimator, scene scripting, recorder |
| `policies/` | the shipped policies, hashed into every audit record and signed into bundles |
| `tests/` | one test file per module, each runnable on its own, plus a ~10 min coverage suite |
| `docs/` | finding documents, design notes and reports — see below |
| `sitl/` | the ArduPilot SITL rail (pymavlink, and MAVROS 2 on ROS 2), all on one desktop; the MAVROS 2 rail is the grant's *dev* topology. The same stack moves to the *hil* topology (VLA + Shield on a Jetson Orin) for the KPI campaign |
| `deploy/` | container images and topology files for the grant's *dev*, *hil* and *flight* topologies, with the Orin side defined for JetPack 6.2; not yet run on an Orin |
| `tools/` | report, deck and KPI builders; `check_claims.py` fails on a withdrawn claim stated again; `prefix_eval.py` replays recorded frames with and without the CSP |
| `scripts/` | launcher scripts (`.ps1` / `.bat`) — see `scripts/README.md` for which backend each targets |
| `reference/` | the grant's own PDFs; not this project's to edit |

## P0 escape on the mid-evaluation flights, and what it does not say

**P0 violation escape rate** (target 0) is one of the five acceptance KPIs the
grant lists, and the only one with a hard limit. It was 0.0 on the 41 shielded
flights counted for the 14 Sept mid-evaluation: 34 on Project AirSim, 4 on
ArduPilot SITL over pymavlink, 3 on ArduPilot SITL over MAVROS 2. The five
deliberately unshielded control flights read 0.63 — they exist to fail, and
they do.

On 8 Oct 2026 a 120 s follow on the Project AirSim + ArduPilot rail let 5 P0
stand-off violations through (0.0053, 5 of 937 Shield-flown ticks); see
[Project AirSim flown by ArduPilot](docs/DESIGN-projectairsim-ardupilot.md).
On every escape tick the perception node had placed the followed car at the
aircraft's own position, a range of 0, where the stand-off check saw no
action that opened the range
([finding](docs/FINDING-the-escape-that-stayed-inside-the-ring.md)). With the
fix, a replay of that flight's 1101 logged ticks through the Shield gives 0
escapes.

The rail's Shield-off controls were flown on 13 and 14 Oct, as a close
approach (`--want-width 0.5`, a more aggressive pilot than the 8 Oct follow)
from the signed bundle, over pymavlink, not KPI-grade. On 13 Oct: Shield on, 0
P0 escapes in 367 Shield-flown ticks (532 in all); Shield off, 727 of 1100
(0.661). On 14 Oct: Shield on, 0 in 388 Shield-flown ticks; Shield off, 736 of
1099 (0.670). With the Shield on, the aircraft was still inside a P0 building
margin. On 13 Oct that happened on 78 ticks, all flown by ArduPilot (20 in its
BRAKE mode, the escalation FSM's Loiter hold, and 58 in RTL); on 14 Oct it
happened on 20 Shield-flown ticks (71 in all), because the Shield's forecast
starts from the command, not from the aircraft's own velocity. The
detector boxed a red traffic signal, not the car, so these flights test the
Shield, not tracking. The range-0 fix has flown since 12 Oct, and its depth
refusal fired in flight (78 and 89 refused depth returns on the two Shield-on
flights), but on no tick of those flights did the subject estimate come
inside the 5 m ring (nearest 16.5 m and 15.3 m), so the Shield side of that
fix still rests on the replay.

**These are not contract KPI figures.** The grant takes every reported KPI from
a Stress Testing run in the *hil* topology, where a Jetson Orin runs the VLA and
the Shield (grant pages *Stress Testing* p.1, *Architecture constraints* p.4).
None of these flights came from a stress harness, and all of them ran on one
desktop. The three MAVROS 2 flights are what the grant calls the *dev*
topology; the Project AirSim flights are the perception rail, and the pymavlink
flights a direct-MAVLink variant of the desktop rail. The 8 Oct follow
commanded ArduPilot over pymavlink, so it is none of the grant's topologies.
The KPI campaign for the final report runs in the *hil* topology: the VLA and
the Shield on a Jetson Orin, which is being set up, with the same code later
moving onto the drone.
Tools for the work-package KPIs (policy round-trip, CSP token budget and
coverage, repair success with the grant's theta cap) were added on 6 Oct;
[`CHANGELOG.md`](CHANGELOG.md) lists what changed in the earlier figures.

What the metric counts: P0 violations the Shield **detected** that still reached
the actuator. It covers the rules in the policy. The stand-off rule,
`SubjectStandoff`, protects the **subject being followed**; other pedestrians
are outside that rule's scope, so `standoff_score` and its visibility breakdown
(which pedestrians were in the camera's view) are published beside the KPI.

## The findings are the deliverable

The finding documents in [`docs/`](docs/) record defects found and fixed, and
several record claims **retracted**. That is deliberate. The recurring failure
in this project has a shape:

> A check that cannot fail is not a check. No violation is indistinguishable
> from no rule; a skipped test from a passing one; a metric a constant can pass
> from a metric that measures skill.

Worked examples:

- [A stand-off rule that was hashed, audited, and completely inert](docs/FINDING-the-standoff-rule-that-never-armed.md) — the policy named class `pedestrian`, the phrase produced `person`, and `binds()` compares exact strings.
- [A tracking metric a constant could pass](docs/FINDING-the-tracking-metric-a-constant-could-pass.md) — at a 100 px tolerance, a "detector" that emits the frame centre and never opens the image scores 1.000.
- [The subject was eight pixels wide](docs/FINDING-the-subject-was-eight-pixels-wide.md) — why the pedestrian phase looked chaotic, and three plausible fixes that measurement refuted.
- [The verifier that verified five of seven](docs/FINDING-the-verifier-that-verified-five-of-seven.md) — a bundle checker naming two fields nothing emits.
- [Facing East, "forward" flew North](docs/FINDING-forward-flew-north.md) — a camera VLA's body-frame output written straight into a North/East action.

## Running it

Requires Project AirSim with an Unreal city level and an NVIDIA GPU. The
guardrail package needs Python 3.11+ (`pyproject.toml`); the Project AirSim and
OpenVLA flight environment (`vla-real`) is Python 3.10.20, a recorded deviation
([`docs/DESIGN-python-versions.md`](docs/DESIGN-python-versions.md)).
[`TUTORIAL.md`](TUTORIAL.md) is the gentle path to the city demo;
[`docs/projectairsim-setup.md`](docs/projectairsim-setup.md) sets up Project
AirSim with ArduPilot and Mission Planner, and
[the perception rail's design note](docs/DESIGN-projectairsim-ardupilot.md)
says how to run it. [`RUNBOOK.md`](RUNBOOK.md) covers the classic AirSim 1.8
demo and the ArduPilot SITL rails in WSL.

The city demo's Shield forecasts 3 s at 0.5 s steps by default (6 forecast
steps), not the grant's 5 s at 0.1 s (50): `--grant-forecast` flies the grant's
forecast with the escalation FSM off, and is not the default because the
stand-off rule forecasts a still subject, and in a replay of the 13 Oct city
flight its hold fired on 44 % of ticks (`flight_shield` in
`demo/follow_vlm.py`). Every recorded city flight flew 3 s at 0.5 s; the
13 Oct one also flew the unsigned YAML policy
`policies/follow_car_citylife_nfz.yaml`, not a signed bundle. The Project
AirSim + ArduPilot node and both ArduPilot SITL rails fly the grant's forecast.

The city demo flies the signed bundle of its `--policy`, not the YAML, and
refuses to start without it. Build the bundles once, and again after a policy
edit: `python tools/build_flight_bundles.py` builds the default set,
`follow_pedestrian` and `follow_car` among them, and naming policies builds
others (`python tools/build_flight_bundles.py orbit_building follow_car_gap`).
With `--dev-yaml` the YAML flies unsigned, as a development run that is never
KPI-grade.

```bash
python demo/follow_vlm.py \
  --object "a yellow car" --retarget "30:a person" \
  --policy policies/follow_pedestrian.yaml \
  --tag myflight --pedestrians 12 --parked 8 --lock-target --save-view
```

Results land in `demo/out/<tag>/`: `kpi.json`, `metrics.json`, the per-tick
`flight_log.jsonl`, the Shield's `audit.jsonl`, and a replay bundle that
re-derives its own KPIs.

```bash
python tests/test_shield.py   # each test file is its own runner: prints "N/M passed", exits non-zero on a failure
# The full CI set is the "Standalone test files" loop in .github/workflows/tests.yml.
# No pytest needed: it is not a dependency, and no environment here installs it.
```

## Reproducibility

Every flight writes a manifest: code revision, VLA weights hash, policy hash,
random seed, sim speed-up and topology. A run with no VLA records
`none:hand-written-controller` there. The city demo's code does so since
19 Oct (`flight_manifest` in `demo/follow_vlm.py`; no city flight has been
recorded since), and its recorded runs filed OWL-ViT's weights under that
field instead; the detector now goes to `metrics.json` (`pilot.detector`). A
run whose working tree was dirty says so, and says that checking out that
commit will not reproduce it.

Until 6 Oct only runs on the desktop ArduPilot SITL + MAVROS 2 rail could be
`kpi_grade: true`, and the manifest labelled that rail `canonical-hil`. Five
runs passed that rule. In the grant's terms the rail is the *dev* topology, and
the grant takes reported KPIs from *hil* runs only. Since 6 Oct the manifest
labels the rail `dev`, reads stored `canonical-hil` runs as `dev`, and
`is_kpi_grade()` refuses a dev run unless a written PI waiver is recorded. The
PI decided on 6 Oct that no waiver will be requested: KPI runs move to the
*hil* topology on a Jetson Orin. So today no run is KPI-grade
([KPI rollup, 6 Oct](docs/data/kpi_rollup_2026-10-06.md)). Everything else is
labelled functional-rail evidence.

## Status

Pre-1.0 research software. See [`CHANGELOG.md`](CHANGELOG.md) for what changed
and, as importantly, what was withdrawn.

Done in October 2026, none of it KPI-grade:

- **Gate G1 passed on 8 Oct.** ArduPilot SITL flew the aircraft in Project
  AirSim through Project AirSim's native ArduPilot controller on 3 of 3 runs,
  and the stream mavlink-router sends to Mission Planner's UDP port 14550
  carried position and attitude at 19.0 Hz through a 120 s flight (measured
  with a listener on that port; Mission Planner's display was not recorded).
  See [Project AirSim flown by ArduPilot](docs/DESIGN-projectairsim-ardupilot.md).
- **The Prefix Compiler follows more of the grant.** It renders the moving
  no-fly zone, the time-window switch and the corridor swap, reads its
  multi-scale geometry from the policy IR, and produces the grant's cost map
  ([cost-map adapter](docs/DESIGN-cost-map-adapter.md)), which no path
  planner reads yet.
- **The stress harness has a gRPC control plane** over mutual TLS, driving the
  headless harness ([design note](docs/DESIGN-harness-grpc.md)).
- **The Jetson Orin deployment is defined for JetPack 6.2**, with ROS 2 Jazzy
  in an Ubuntu 24.04 container ([`deploy/`](deploy/README.md),
  [topologies](docs/DESIGN-topologies.md)). Nothing has been built or run on
  an Orin yet.

Next: the *hil* topology on the Jetson Orin for the KPI campaign, the
perception node through MAVROS 2, an escalation hold step that keeps the
aircraft inside its altitude band on ArduPilot with fail-safe trigger
correctness at the grant's target of at least 99 %, scenarios with mid-flight
rule changes on ArduPilot SITL, and then the drone.
[`docs/projectairsim-setup.md`](docs/projectairsim-setup.md) covers ArduPilot,
Project AirSim and Mission Planner together.
[`docs/CHECKLIST-remaining-work.md`](docs/CHECKLIST-remaining-work.md) is
superseded.

## Contributing

[`CONTRIBUTING.md`](CONTRIBUTING.md) — the definition of done for a change
here: a regression test, a changelog entry, and (for a silent defect) a
finding document. It also has the folder layout, so a new file has an
obvious home instead of landing at the repo root.

## Licence and attribution

Research code for an ITRI-funded project at National Taipei University of
Technology. The reference implementation this work aligns its contracts to is
Prof. Lai's, tracked separately on the `main` branch of this fork.
