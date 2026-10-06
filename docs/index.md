---
title: Guardrail
---

# Guardrail

**A safety layer for language-commanded UAVs.**

A vision-language model is told to *follow the yellow car*. It produces a
velocity command. Nothing in that pipeline knows what a no-fly zone is.

Guardrail sits between the model and the aircraft: a declarative safety policy —
fences, altitude envelopes, kinematic caps, obstacle clearances, stand-off
rings, corridors — enforced on every command at 10 Hz, repairing what it can and
refusing what it cannot.

Built for the ITRI grant *Semantic-Spatial Translation and Safety-Constrained
VLA for ArduPilot UAVs* (PI: Prof. Kuan-Ting Lai, NTUT).
Version **0.5.1** — see the [changelog]({{ site.github.repository_url }}/blob/master/CHANGELOG.md).

---

## P0 escape on the recorded flights, and what it does not say

**P0 violation escape rate** (target 0) is one of the five acceptance KPIs the
grant lists, and the only one with a hard limit. It was 0.0 on the 41 shielded
flights counted for the 14 Sept mid-evaluation: 34 on Project AirSim, 4 on
ArduPilot SITL over pymavlink, 3 on ArduPilot SITL over MAVROS 2. The five
deliberately unshielded control flights read 0.63 — they exist to fail, and
they do.

**These are not contract KPI figures.** The grant takes every reported KPI from
a Stress Testing run in the *hil* topology, where a Jetson Orin runs the VLA and
the Shield (grant pages *Stress Testing* p.1, *Architecture constraints* p.4).
None of these flights came from a stress harness, and all of them ran on one
desktop. The three MAVROS 2 flights are what the grant calls the *dev*
topology; the Project AirSim and pymavlink flights are not even that. Several
of the other KPIs the
grant names have no number yet; see the
[contract audit](AUDIT-KONTRAK-2026-10-05.md) and the
[corrections to the mid-evaluation report](CORRECTION-2026-10-06-mid-evaluation-and-deck.md).

It is worth being exact, because it reads as more than it is. The metric counts P0 violations the Shield **detected** that
nonetheless reached the actuator. On the most recent flight it reads 0.0 while a
real pedestrian came within 10 m of the aircraft on 516 ticks, and the 10 m
stand-off rule fired on 31 of them.

That gap is not what it first looks like. On **498 of those 516 ticks the person
was outside the camera's field of view** — beside or behind the aircraft — and
the rule, `SubjectStandoff`, protects only the **subject being followed**, one
position per tick. The aircraft was following a different pedestrian roughly
45 m ahead, and its range estimate for that person was within a few metres.
Nothing in the policy protects the other pedestrians, and a forward camera could
not have seen them.

**The Shield was correct throughout.** What the zero cannot say is anything
about people the policy does not name and the sensor cannot see. That is a gap
in policy scope and sensor coverage, and `standoff_score` plus its visibility
breakdown are published beside the KPI so it cannot be read as more than it is.

---

## The findings are the deliverable

Twenty-seven documents here record defects found and fixed, and several record
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

### Design notes

- [The HIL perception bridge](DESIGN-hil-perception-bridge.md)
- [The orbit-building task](DESIGN-orbit-building-task.md)
- [A native Unreal environment](DESIGN-unreal-native-environment.md)

### Reports and status

- [Contract audit, 5 October 2026](AUDIT-KONTRAK-2026-10-05.md) (in Indonesian): the current list of open grant items
- [Corrections to the mid-evaluation report and deck, 6 October 2026](CORRECTION-2026-10-06-mid-evaluation-and-deck.md)
- [Midterm report, August 2026](MIDTERM-REPORT-Aug2026.md) (some claims are corrected in the note above)
- [Remaining work, to 9 September 2026](CHECKLIST-remaining-work.md) (superseded by the audit)

---

## The one interface

The Shield's only input is an **Action4D**: `(vx north, vy east, vz up,
yaw_rate)` at 10 Hz, `yaw_rate` in **rad/s**. It does not know whether that came
from OWL-ViT, from OpenVLA-7B, or from a joystick — and that is the point. The
safety argument must not depend on which model is driving.

```python
from guardrail import Shield, State, load_policy

shield = Shield(load_policy("policies/follow_pedestrian.yaml"))
shield.set_subject(x, y)                     # perception supplies this
decision = shield.step(state, raw_action)    # -> repaired action + audit record
```

---

## Reproducibility

Every flight writes a manifest: code revision, detector weights hash, policy
hash, random seed, sim speed-up and topology. A run whose working tree was dirty
says so, and says that checking out that commit will not reproduce it.

Until 6 Oct only runs on the desktop ArduPilot SITL + MAVROS 2 rail could be
`kpi_grade: true`, and the manifest labelled that rail `canonical-hil`. Five
runs passed that rule. In the grant's terms the rail is the *dev* topology, and
the grant takes reported KPIs from *hil* runs only. Since 6 Oct the manifest
labels the rail `dev`, reads stored `canonical-hil` runs as `dev`, and
`is_kpi_grade()` refuses a dev run unless the PI grants a written waiver (open
question PQ1). So today no run is KPI-grade (`docs/data/kpi_rollup_2026-10-06.md`).
Everything else is labelled functional-rail evidence.

[Browse the source]({{ site.github.repository_url }}/tree/master) ·
[All documents]({{ site.github.repository_url }}/tree/master/docs)
