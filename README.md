# Guardrail — a safety layer for language-commanded UAVs

[![version](https://img.shields.io/badge/version-0.5.0-blue)](CHANGELOG.md)
[![P0 escape rate](https://img.shields.io/badge/P0%20violation%20escape%20rate-0.0-brightgreen)](#the-kpi-and-what-it-does-not-say)
[![tests](https://img.shields.io/badge/tests-458%20fast%20%2B%2017%20coverage-brightgreen)](tests/)

A vision-language model is told to *follow the yellow car*. It produces a
velocity command. **Nothing in that pipeline knows what a no-fly zone is.**

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
from OWL-ViT, from OpenVLA-7B, or from a joystick, and that is the point — the
safety argument must not depend on which model is driving.

```python
from guardrail import Shield, State, load_policy

shield = Shield(load_policy("policies/follow_pedestrian.yaml"))
shield.set_subject(x, y)                     # perception supplies this
decision = shield.step(state, raw_action)    # -> repaired action + audit record
```

## What is here

| | |
|---|---|
| `guardrail/` | the policy DSL (WP1), prefix compiler (WP2), Safety Shield (WP3), replay bundles (WP4) |
| `demo/` | the flight controller, OWL-ViT grounder, target estimator, scene scripting, recorder |
| `policies/` | 27 policies, hashed into every audit record |
| `tests/` | 23 files, 458 fast tests plus a ~10 min coverage suite |
| `docs/` | 28 finding documents — see below |
| `sitl/` | the ArduPilot SITL rail, which is the contractual KPI gate |

## The KPI, and what it does not say

The grant's hard KPI is **P0 violation escape rate = 0**. It is 0.0 on every
flight recorded here.

It is worth being exact about what that means, because it is easy to read as
more than it is. The metric counts P0 violations the Shield **detected** that
nonetheless reached the actuator. On the most recent flight it reads 0.0 while a
real pedestrian was inside the 10 m P0 stand-off ring on 516 ticks and the rule
saw 31 of them — because the position the Shield was *served* was wrong by tens
of metres.

**The Shield was correct throughout.** It answers honestly about the position it
is given. But system P0 compliance depends equally on perception, and until
`standoff_score` there was no number for that half at all. Both are now
published side by side.

## The findings are the deliverable

Twenty-eight documents in [`docs/`](docs/) record defects found and fixed, and
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

## Running it

Requires ProjectAirSim with an Unreal city level, an NVIDIA GPU, and Python 3.10.
[`RUNBOOK.md`](RUNBOOK.md) has the full setup; [`TUTORIAL.md`](TUTORIAL.md) is
the gentle path.

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
python -m pytest tests/ -q          # or: python tests/test_shield.py
```

## Reproducibility

Every flight writes a manifest: code revision, detector weights hash, policy
hash, random seed, sim speed-up and topology. A run whose working tree was dirty
says so, and says that checking out that commit will not reproduce it. Only
`canonical-hil` runs are `kpi_grade: true`; everything else is labelled
functional-rail evidence, not a contractual KPI figure.

## Status

Pre-1.0 research software. See [`CHANGELOG.md`](CHANGELOG.md) for what changed
and, as importantly, what was withdrawn. Current open items are in
[`docs/CHECKLIST-remaining-work.md`](docs/CHECKLIST-remaining-work.md).

## Licence and attribution

Research code for an ITRI-funded project at National Taipei University of
Technology. The reference implementation this work aligns its contracts to is
Prof. Lai's, tracked separately on the `main` branch of this fork.
