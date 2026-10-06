# Guardrail — a safety layer for language-commanded UAVs

[![version](https://img.shields.io/badge/version-0.5.1-blue)](CHANGELOG.md)
[![P0 escape on demo and SITL flights](https://img.shields.io/badge/P0%20escape%20on%20demo%20and%20SITL%20flights-0.0%20%28dev%20evidence%2C%20not%20a%20contract%20KPI%20figure%29-lightgrey)](#p0-escape-on-the-recorded-flights-and-what-it-does-not-say)
[![tests](https://img.shields.io/badge/tests-460%20fast%20%2B%2017%20coverage-brightgreen)](tests/)

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
| `tests/` | 23 files, 460 fast tests plus a ~10 min coverage suite |
| `docs/` | 27 finding documents — see below |
| `sitl/` | the ArduPilot SITL rail (pymavlink, and MAVROS 2 on ROS 2), all on one desktop. The MAVROS 2 rail is the grant's *dev* topology; the pymavlink rail is not even that. Five MAVROS 2 runs passed the project's own KPI-grade rule until 6 Oct; none is a contract KPI figure, which needs the *hil* topology (VLA + Shield on a Jetson Orin) |
| `scripts/` | launcher scripts (`.ps1` / `.bat`) — see `scripts/README.md` for which backend each targets |
| `reference/` | the grant's own PDFs; not this project's to edit |

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
[contract audit](docs/AUDIT-KONTRAK-2026-10-05.md) and the
[corrections to the mid-evaluation report](docs/CORRECTION-2026-10-06-mid-evaluation-and-deck.md).

It is worth being exact about what that means, because it is easy to read as
more than it is. The metric counts P0 violations the Shield **detected** that
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

## The findings are the deliverable

Twenty-seven documents in [`docs/`](docs/) record defects found and fixed, and
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
says so, and says that checking out that commit will not reproduce it.

Until 6 Oct only runs on the desktop ArduPilot SITL + MAVROS 2 rail could be
`kpi_grade: true`, and the manifest labelled that rail `canonical-hil`. Five
runs passed that rule. In the grant's terms the rail is the *dev* topology, and
the grant takes reported KPIs from *hil* runs only. Since 6 Oct the manifest
labels the rail `dev`, reads stored `canonical-hil` runs as `dev`, and
`is_kpi_grade()` refuses a dev run unless the PI grants a written waiver (open
question PQ1). So today no run is KPI-grade
([KPI rollup, 6 Oct](docs/data/kpi_rollup_2026-10-06.md)). Everything else is
labelled functional-rail evidence.

## Status

Pre-1.0 research software. See [`CHANGELOG.md`](CHANGELOG.md) for what changed
and, as importantly, what was withdrawn. The current list of open contract
items is the [5 Oct 2026 contract audit](docs/AUDIT-KONTRAK-2026-10-05.md) (in
Indonesian) and the tracker built from it (`tracker/`).
[`docs/CHECKLIST-remaining-work.md`](docs/CHECKLIST-remaining-work.md) is
superseded. Claims withdrawn from the delivered mid-evaluation report and deck
are listed in
[`docs/CORRECTION-2026-10-06-mid-evaluation-and-deck.md`](docs/CORRECTION-2026-10-06-mid-evaluation-and-deck.md).

## Contributing

[`CONTRIBUTING.md`](CONTRIBUTING.md) — the definition of done for a change
here: a regression test, a changelog entry, and (for a silent defect) a
finding document. It also has the folder layout, so a new file has an
obvious home instead of landing at the repo root.

## Licence and attribution

Research code for an ITRI-funded project at National Taipei University of
Technology. The reference implementation this work aligns its contracts to is
Prof. Lai's, tracked separately on the `main` branch of this fork.
