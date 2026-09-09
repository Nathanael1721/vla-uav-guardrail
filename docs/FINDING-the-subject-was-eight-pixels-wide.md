# The subject was eight pixels wide

**Date:** 2026-09-09
**Found by:** the operator watching the demo video and saying the tracking
looked confused. It did. Three of the four explanations I reached for first
were wrong, and each was refuted by a measurement rather than by an argument.
**Status:** two fixes shipped; the underlying limit is **not fixed** and is a
decision for the 18 September meeting.

## What was visible

On `retarget_fixed`, before and after the mid-flight retarget from a car to a
person:

| | before | after |
|---|---|---|
| yaw sign flips | 0 | 5 |
| box jumps | 0 | 7 |
| median \|yaw\| | 3.0 dps | 11.5 dps |
| max \|yaw\| | 30.6 dps | **130.4 dps** |

130 dps is the aircraft spinning on the spot. The estimator meanwhile reported
`updates: 4, gated_out: 150` and attributed **12.96 m/s** to a pedestrian.

## The four explanations, in the order I reached for them

### 1. "The selection filters are keeping the wrong box." — refuted

The theory: the instance lock's 48 px gate and the grounder's 0.35 W temporal
jump filter were written for the car phase, and after the retarget they delete
the correct pedestrian box and keep a stale one. `PresenceMonitor` said ABSENT
on 311 of 331 post-retarget ticks, so the signal to suspend them was already
there and unread.

The measurement that killed it — winner versus runner-up, scored against truth:

| phase | winner | runner-up | runner-up closer |
|---|---|---|---|
| car | **6.6 px** | 8.1 px | 42% of ticks |
| pedestrian | **44.2 px** | 39.0 px | **47% of ticks** |

Post-retarget the runner-up is closer on a coin flip. Selection is not keeping
a worse box — **the entire candidate set is noise**, and no re-ranking among
twelve bad boxes reaches a good one.

### 2. "Gate the presence verdict into selection." — refuted, and backwards

Before wiring `presence == ABSENT` into the filters, I checked what ABSENT
means in each phase. It means opposite things:

| | PRESENT | ABSENT / implied-width | ABSENT / size-unstable |
|---|---|---|---|
| car | 5.9 px | **1.2 px** (n=22) | 7.1 px |
| pedestrian | 11.7 px | **53.4 px** (n=262) | 11.2 px |

In the car phase the ticks the monitor calls ABSENT are the ones where the box
is *most* accurate. Splitting by which bound failed explains it:

| | n | implied width | box error |
|---|---|---|---|
| car, too NARROW | 13 | 0.80 m | **0.5 px** |
| car, too WIDE | 9 | 9.50 m | 6.3 px |
| pedestrian, too WIDE | 262 | 4.00 m | **53.4 px** |

A tight box on a car at 20 m implies 0.8 m because the box is tight and depth
is quantised to whole metres — the *lower* bound of `PLAUSIBLE_WIDTH_M["car"]`
misfires on the best boxes in the flight. Only the **upper** bound carries
information: a box far wider than the class contains something else.

### 3. "So gate the estimator on the upper bound." — measured, and dropped

That is a real signal, and on its own it looks excellent. It is still the wrong
change, because the speed clamp (below) gets there first and better:

| pedestrian phase | served | estimate error | max speed |
|---|---|---|---|
| today | 55/331 | 31.08 m | 13.06 m/s |
| **speed clamp only** | **257/331** | **9.15 m** | 2.00 |
| width gate only | 168/331 | 9.03 m | 16.09 |
| clamp + width gate | 168/331 | 9.85 m | 2.00 |

The gate blocks 125 of 154 measurements, so the filter coasts *more*. Measured
alone it looked like a 31.08 → 9.03 m win; almost all of that is the clamp's
win, attributed to whichever fix was measured first. Two remedies for one
symptom, and the second is harmful once the first lands — the fifth time this
project has caught that shape.

### 4. What it actually is

The FrontCamera renders **400 × 225 at 90° horizontal FOV**. That is not a
downscale for the detector; it is the render resolution.

| range | width of a 0.5 m person |
|---|---|
| 10 m (the policy's own minimum) | 12.7 px |
| 14.5 m (this flight's median slant range) | **8.8 px** |
| 20 m | 6.4 px |

The boxes the detector actually returned post-retarget have a **median width of
26.4 px** — three times too wide to be a person at any range flown. It was not
finding pedestrians. It was labelling 26-pixel pieces of city "a person".

And the frame at the flight's closest legal approach shows it plainly: the
chosen box is a **building**, the HUD says `TARGET LOCKED`, and the real
pedestrians are the six-pixel figures on the pavement.

**The Shield's own 10 m stand-off caps how close the aircraft may get**, and at
10 m the subject is still 12.7 px. So the task as configured is not achievable:
the safety rule and the perception task are in direct conflict at this camera
geometry. That conflict is the finding, and no amount of tracking logic
dissolves it.

## What shipped

Two changes, both measured, neither of which pretends to fix the geometry.

**A per-class speed ceiling on the estimator** (`SUBJECT_VMAX_MPS`, applied as a
post-update clamp in `TargetState.update`). A constant-velocity filter cannot
know that a person does not travel at 13 m/s. Once the estimate ran away, the
filter's own 4σ gate rejected 149 of the next 154 measurements, so it coasted
the runaway velocity to the end of the flight — a feedback loop, not a one-off
bad estimate. Pedestrian phase: served **55/331 → 257/331**, error to the
nearest real pedestrian **31.08 m → 9.15 m**, max attributed speed **13.06 →
2.00 m/s**. Car phase: unchanged in every statistic (the ceiling is 15 m/s and
the car never exceeded 4.7).

**One yaw cap, in one place** (`yaw_command`). The coast branch clipped to
1.1 rad/s and the estimator branch clipped to nothing. `servo()`'s bearing is
bounded by the field of view and cannot reach the cap; `TargetState.observe()`'s
comes from a world position and can point behind the aircraft, so at
`yaw_gain 1.2` it reaches 3.77 rad/s. That is the 130 dps. `servo()` is
deliberately left unclipped, with a test pinning the argument that it is bounded
by construction.

The pilot's cap is 63 dps and the policy's is 45 dps. That gap is intentional:
the moment the pilot reads the policy and pre-clips to it, the Shield stops
being what enforces the limit. What the pilot owes is internal consistency, and
it did not have it.

## What did not ship, and must not be quietly revived

- **The implied-width gate on candidate selection.** An earlier measurement of
  mine put it at 31% → 77% precision. That measurement was against a label that
  is 62% accidental alignment with occluded pedestrians; on an occlusion-checked
  label it buys 12% → 38% and costs **80% of all detections**.
- **The presence verdict as a selection gate** — §2 above.
- **The implied-width gate on the estimator feed** — §3 above.
- **A divergence reset** in the estimator. It suppresses the same symptom with
  no physical justification, where the speed prior has one.

## The options, for 18 September

None of these is free, and the choice is the PI's:

1. **Narrow the FOV.** 90° → 45° at the same 400 × 225 puts a person at
   **25.5 px at 10 m**. Costs nothing in render or inference time. Costs half
   the field of view, and `hfov_deg` is a constant six sites agree on — the
   depth capture must move with it or the box indexes the wrong pixels.
2. **Raise the render resolution.** Measured previously: 1280 × 720 drops the
   detector to 2.94–3.96 Hz against a 4.0 Hz gate.
3. **Change the subject.** A 1.5 m-wide figure, or a cyclist, is inside what
   this camera can resolve at the ranges the policy permits.
4. **Accept it and say so.** Pedestrian *following* is not itself a KPI, and
   the Shield's own behaviour is sound. But "accept it" can no longer mean
   "P0 = 0, so we are fine": the re-flight below shows that number reading zero
   while the P0 ring's condition was breached on 485 ticks it never saw.
   Accepting the perception limit therefore means **publishing
   `standoff_score` beside the KPI**, not resting on the KPI. Demonstrating the
   10 m ring
   honestly requires a subject the perception stack can actually hold.

## The re-flight, and what it uncovered

`retarget_smooth`, same command line, both fixes in. **Closed-loop**, not
replay.

| | car before | car after | ped before | ped after |
|---|---|---|---|---|
| box error, in shot | 5.7 px | 5.4 px | 46.1 px | **7.0 px** |
| ...centre-constant null | 16.9 | 15.1 | 43.2 | **5.9** |
| median \|yaw\| | 3.0 dps | 2.8 dps | 11.5 dps | **1.7 dps** |
| max \|yaw\| | 30.6 | 24.7 | **130.4** | **32.7** |
| yaw sign flips | 2.4% | 3.2% | 6.6% | 4.1% |

**The car phase is undisturbed**, which was the main risk of both changes.

**The motion is smooth.** Median yaw 11.5 → 1.7 dps and the 130 dps spin is
gone. That is the thing the operator was looking at, and it is fixed.

**The tracking is NOT demonstrated, and must not be reported as if it were.**
7.0 px looks like a large win over 46.1 — but the centre-constant null on the
same rows moved from 43.2 px to **5.9 px**, so the detector is still *worse
than a constant that never opens the image*. The aircraft now points at the
subject instead of spinning, and that geometry flatters the detector and the
null equally. Pointing improved; seeing did not.

### The range channel, which the chaos was hiding

With the bearing fixed, the range became visible, and it is worse:

```
|est.rng - nearest real person|  median 37.41 m
ticks 340-700: a real person 5.06 m away, est.rng median 43.34 m
```

The box lands on the right column and the **depth sampled through an 8-pixel
box is the background** — the road and the buildings behind the person, not the
person. So the controller believes the subject is 43 m away, wants 1.98 m, and
commands full forward speed into a building for 350 ticks. `ObstacleClearance`
held it there, 151 firings, which is the Shield doing its job.

### The KPI reads zero while the P0 rule's condition was breached 485 times

`standoff-pedestrian` is `priority: P0` — the grant's hard KPI rule.

```
closest a REAL person came = 4.70 m   (tick 324)
  estimator served there   = 70.55 m
  P0 rules that fired      = NONE
  p0_violation_escape_rate = 0.0

ticks a real person was inside the 10 m P0 ring : 516
of those, ticks the P0 rule was SILENT          : 485
```

**The Shield is not broken and the KPI is not lying about what it measures.**
`p0_violation_escape_rate` is the fraction of *detected* P0 violations that
reached the actuator, and it is honestly 0: every violation the Shield saw, it
repaired. But a reader — and a grant reviewer — will read "P0 violation escape
rate = 0" as *the aircraft never violated a P0 rule*, and on this flight that
reading is false. The condition occurred 516 times and was seen 31 times.

The KPI measures the Shield. **System P0 compliance depends equally on
perception, and until `standoff_score` there was no number for that half at
all.** This is the same shape as every other finding in this project: a zero
that means "never detected" read as "never happened".

One caveat on the recall figure: `standoff_score` counts a tick as a true
positive when ANY logged pedestrian is inside the ring, while `Shield` is told
about exactly one subject via `set_subject()`. So part of the 485 is a rule
that structurally cannot protect bystanders — 11 of the 12 people in the scene
are invisible to it. That is itself worth a decision, not a scoring artefact.

## The pattern, again

Three of my four explanations were wrong, and each was refuted by measuring
rather than arguing. The one that would have done real damage is the second:
it was reasonable, it had a plausible mechanism, the numbers supporting it were
real — and the signal it proposed to act on means the opposite thing in the
phase that currently works. It would have degraded the car phase to fix the
pedestrian phase, and the flight would still have looked confused.

The habit that caught all three is the cheap one: **before shipping a fix, ask
what it does to the case that is already working.**
