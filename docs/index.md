---
title: Guardrail
---

# Guardrail

**A safety layer for language-commanded UAVs.**

In the grant's design, a vision-language-action model (VLA) is told to *follow
the yellow car* and answers with a velocity command. Nothing in that pipeline
knows what a no-fly zone is.

No VLA flies the demonstrations recorded here. In the city demo and on the
Project AirSim + ArduPilot perception rail, OWL-ViT finds the car from the
words and a hand-written follow controller turns its box into the velocity
command; the ArduPilot SITL rails fly `guardrail.vla_stub.StubVLA`, a
deliberately rule-ignorant stand-in. Real VLAs have flown through the Shield
only in simulation, and none decided at 10 Hz (the 10 Hz loop reused each
decision between inferences; on 4 Aug its yaw command faded from 1.5 s and was
zero after 3 s): OpenVLA-7B (about 0.9 s an inference) and AerialVLA adapters
on classic AirSim (AirSimNH) in July 2026; AerialVLA on Project AirSim from 16
Jul 2026 (`demo/aerialvla_pas_demo.py`, mostly behind a route planner's
pure-pursuit follower or the 0.55 goal assist) and on 3 Aug
(`demo/semantic_demo.py`, with no planner); and 8 AerialVLA flights on Project
AirSim on 4 Aug 2026 (6 with the Shield on; `demo/semantic_seek.py`), the last
VLA-commanded flights, at one decision every 7-9 s and with a direction hint
computed from the true coordinates in its prompt
([result](RESULT-vla-guardrail-simultaneity.md)).

Guardrail sits between the model and the aircraft: a declarative safety policy —
fences, altitude envelopes, kinematic caps, obstacle clearances, stand-off
rings, corridors — enforced on every command at 10 Hz, repairing what it can and
refusing what it cannot.

Built for the ITRI grant *Semantic-Spatial Translation and Safety-Constrained
VLA for ArduPilot UAVs* (PI: Prof. Kuan-Ting Lai, NTUT).
Version **0.5.1** — see the [changelog]({{ site.github.repository_url }}/blob/master/CHANGELOG.md).

**[Progress site](progress/)**: what is built, what has been shown and what
comes next, in English and Traditional Chinese.

---

## P0 escape on the mid-evaluation flights, and what it does not say

**P0 violation escape rate** (target 0) is one of the five acceptance KPIs the
grant lists, and the only one with a hard limit. It was 0.0 on the 41 shielded
flights counted for the 14 Sept mid-evaluation: 34 on Project AirSim, 4 on
ArduPilot SITL over pymavlink, 3 on ArduPilot SITL over MAVROS 2. The five
deliberately unshielded control flights read 0.63 — they exist to fail, and
they do.

On 8 Oct 2026 a 120 s follow on the Project AirSim + ArduPilot rail let 5 P0
stand-off violations through (0.0053); see [Project AirSim flown by ArduPilot](DESIGN-projectairsim-ardupilot.md).
The rail's Shield-off controls were flown on 13 and 14 Oct as a close
approach from the signed bundle, over pymavlink, not KPI-grade: Shield on, 0
P0 escapes on both days; Shield off, 727 of 1100 ticks (0.661) and 736 of 1099
(0.670). With the Shield on, the aircraft was still inside a P0 building
margin on 20 Shield-flown ticks on 14 Oct and on 78 ArduPilot-flown ticks on
13 Oct (ArduPilot's BRAKE mode under the FSM's Loiter, and RTL), and the
detector boxed a red traffic signal, not the car, so these flights test the
Shield, not tracking.

The city demo's Shield forecasts 3 s at 0.5 s steps by default, not the
grant's 5 s at 0.1 s (`--grant-forecast` flies that, with the escalation FSM
off), and its 13 Oct flight flew an unsigned YAML policy. The Project AirSim +
ArduPilot node and both ArduPilot SITL rails fly the grant's forecast.

**These are not contract KPI figures.** The grant takes every reported KPI from
a Stress Testing run in the *hil* topology, where a Jetson Orin runs the VLA and
the Shield (grant pages *Stress Testing* p.1, *Architecture constraints* p.4).
None of these flights came from a stress harness, and all of them ran on one
desktop. The three MAVROS 2 flights are what the grant calls the *dev*
topology; the Project AirSim flights are the perception rail, and the pymavlink
flights a direct-MAVLink variant of the desktop rail. The KPI campaign for the
final report runs in the *hil* topology: the VLA and the Shield on a Jetson
Orin, which is being set up, with the same code later moving onto the drone.
Tools for the work-package KPIs (policy round-trip, CSP token budget and
coverage, repair success with the grant's theta cap) were added on 6 Oct; the
changelog lists what changed in the earlier figures.

What the metric counts: P0 violations the Shield **detected** that still reached
the actuator. It covers the rules in the policy. The stand-off rule,
`SubjectStandoff`, protects the **subject being followed**; other pedestrians
are outside that rule's scope, so `standoff_score` and its visibility breakdown
(which pedestrians were in the camera's view) are published beside the KPI.

---

## The findings are the deliverable

The finding documents here record defects found and fixed, and several record
claims **retracted**. That is deliberate. The recurring failure in this project
has one shape:

> **A check that cannot fail is not a check.** No violation is indistinguishable
> from no rule; a skipped test from a passing one; a metric a constant can pass
> from a metric that measures skill.

### Start here

- [A stand-off rule that was hashed, audited, and completely inert](FINDING-the-standoff-rule-that-never-armed.md)
  — the policy named class `pedestrian`, the phrase produced `person`, and
  `binds()` compares exact strings. Zero firings across a whole flight with a
  human subject, and nothing reported it.
- [A tracking metric a constant could pass](FINDING-the-tracking-metric-a-constant-could-pass.md)
  — at a 100 px tolerance, a "detector" that emits the frame centre and never
  opens the image scores 1.000 on our best flight.
- [The subject was eight pixels wide](FINDING-the-subject-was-eight-pixels-wide.md)
  — why the pedestrian phase looked chaotic, and three plausible fixes that
  measurement refuted before they shipped.
- [The verifier that verified five of seven](FINDING-the-verifier-that-verified-five-of-seven.md)
  — a bundle checker naming two fields nothing emits, one of them a grant KPI.
- [The contract disagreed with itself about yaw](FINDING-the-contract-disagreed-with-itself-about-yaw.md)
  — a comment saying deg/s over six sites enforcing rad/s.
- [Facing East, "forward" flew North](FINDING-forward-flew-north.md)
  — a camera VLA's body-frame output written straight into a North/East action.
- [A stripped signature read as "unsigned"](FINDING-a-stripped-signature-read-as-unsigned.md)
  — the weakest signature status became a way around the strongest check.

### Design notes

- [The Prefix Compiler and its CSP](DESIGN-prefix-compiler.md)
- [The escalation state machine](DESIGN-escalation-fsm.md)
- [Policy identity and signed bundles](DESIGN-policy-identity.md)
- [The Paraphraser](DESIGN-paraphraser.md)
- [Python versions, environments, and the ArduPilot pin](DESIGN-python-versions.md)
- [Project AirSim flown by ArduPilot, with Mission Planner attached](DESIGN-projectairsim-ardupilot.md)
- [The public progress site](DESIGN-public-site.md)
- [The HIL perception bridge](DESIGN-hil-perception-bridge.md)
- [The orbit-building task](DESIGN-orbit-building-task.md)
- [A native Unreal environment](DESIGN-unreal-native-environment.md)
- [The VLA pilot: AerialVLA behind the Shield](DESIGN-vla-pilot.md)
  — built and tested offline, not flown.

### Reports and status

- [Midterm report, August 2026](MIDTERM-REPORT-Aug2026.md) (as delivered; figures corrected since are listed in the changelog)
- [Mid-evaluation report, September 2026](MID-EVALUATION-REPORT-Sep2026.md) (as delivered on 14 Sept; several of its claims were withdrawn on 5-6 Oct, listed in the changelog)
- [Remaining work, to 9 September 2026](CHECKLIST-remaining-work.md) (superseded)
- [Project AirSim setup, with ArduPilot and Mission Planner](projectairsim-setup.md)

---

## The one interface

The Shield's only input is an **Action4D**: `(vx north, vy east, vz up,
yaw_rate)` at 10 Hz, `yaw_rate` in **rad/s**. It does not know whether that came
from a hand-written controller steering by OWL-ViT's boxes, from OpenVLA-7B, or
from a joystick — and that is the point. The safety argument must not depend on
which model is driving.

```python
from guardrail import Action4D, Shield, State, load_policy

shield = Shield(load_policy("policies/follow_pedestrian.yaml"))
shield.set_subject(x, y, subject_class="pedestrian")  # perception supplies position AND class
decision = shield.filter(state, raw_action)            # -> decision.emitted (the repaired Action4D) + audit fields
```

The subject's class matters: the policy's 10 m pedestrian stand-off binds only
to a subject reported as `pedestrian`; without a class only its 5 m catch-all
rule applies ([the rule that never armed](FINDING-the-standoff-rule-that-never-armed.md)).

---

## Reproducibility

Every flight writes a manifest: code revision, VLA weights hash, policy hash,
random seed, sim speed-up and topology. A run with no VLA records
`none:hand-written-controller` there; the city demo's code does so since
19 Oct (no city flight has been recorded since), and its recorded runs filed
OWL-ViT's weights under that field instead. A run whose working tree was dirty
says so, and says that checking out that commit will not reproduce it.

Until 6 Oct only runs on the desktop ArduPilot SITL + MAVROS 2 rail could be
`kpi_grade: true`, and the manifest labelled that rail `canonical-hil`. Five
runs passed that rule. In the grant's terms the rail is the *dev* topology, and
the grant takes reported KPIs from *hil* runs only. Since 6 Oct the manifest
labels the rail `dev`, reads stored `canonical-hil` runs as `dev`, and
`is_kpi_grade()` refuses a dev run unless a written PI waiver is recorded. The
PI decided on 6 Oct that no waiver will be requested: KPI runs move to the
*hil* topology on a Jetson Orin. So today no run is KPI-grade (`docs/data/kpi_rollup_2026-10-06.md`).
Everything else is labelled functional-rail evidence.

[Browse the source]({{ site.github.repository_url }}/tree/master) ·
[All documents]({{ site.github.repository_url }}/tree/master/docs)
