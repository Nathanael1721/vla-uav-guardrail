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

## The KPI, and what it does not say

The grant's hard KPI is **P0 violation escape rate = 0**. It is 0.0 on every
flight recorded here.

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

- [Midterm report, August 2026](MIDTERM-REPORT-Aug2026.md)
- [Remaining work](CHECKLIST-remaining-work.md)

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
says so, and says that checking out that commit will not reproduce it. Only
`canonical-hil` runs are `kpi_grade: true`; everything else is labelled
functional-rail evidence, not a contractual KPI figure.

[Browse the source]({{ site.github.repository_url }}/tree/master) ·
[All documents]({{ site.github.repository_url }}/tree/master/docs)
