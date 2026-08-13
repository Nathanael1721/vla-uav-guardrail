# Can the system say the object is not there?

**Date:** 2026-08-11
**Task:** acknowledged limitation number two.
**Flights:** `v3_present` / `v3_absent`, `v4_present` / `v4_absent` — identical
words, identical policy, one scene with the car and one with no car at all.

**Answer: partly, at flight resolution only. Not per tick.**

---

## Why this needed doing

With no car in the scene the raw detector still fires on roughly three quarters
of frames, and only the colour gate suppresses the follow. The orbit control arm
showed what that costs: subject retention scored **1.000 on both arms**,
including the one that flew 200 m from any traffic light. A metric that scores
perfectly on a flight chasing nothing is not measuring anything.

## What was built

Four checks, none of them a confidence threshold — the detector's own confidence
is precisely what cannot separate absence from presence.

| check | rejects | needs |
|---|---|---|
| colour | pixels are not the named colour | already existed |
| scale | box covers most of the frame — a wall, not an object | box only |
| **implied size** | a "car" that must be 20 m across | **depth** |
| **size stability** | the thing keeps changing physical size | depth + memory |

The last two are new and both come from the depth range. A box is an angle; with
a range it becomes a **physical width**, which can be checked against what the
named object actually is — and which, for a real object, stays *constant* while
range and apparent width both change.

## What the measurements say

Depth was available on 466 of 466 ticks. The two arms differ clearly in the raw
signals:

| | car present | no car |
|---|---|---|
| median range to the detection | **29.8 m** | **69.0 m** |
| median implied width | **3.4 m** (a car) | 5.4 m |
| implied-width instability (rolling CV over 1 s) | **0.406** | **0.615** |
| fraction of ticks with CV > 0.35 | 0.56 | 0.84 |

Every one of those points the right way. With no car, the detector locks onto
city clutter that is twice as far away, the wrong size, and not the same thing
from one tick to the next.

## And what the ceiling is

A sweep over the whole threshold space, replayed offline against both flight
logs, gives the best this approach can do:

| `cv_max` | width band | ABSENT with a car | ABSENT with no car | gap |
|---|---|---|---|---|
| **0.35** | **1.0–8.0 m** | **0.40** | **0.75** | **0.35** |
| 0.45 | 1.0–8.0 m | 0.31 | 0.60 | 0.29 |
| 0.55 | 1.0–8.0 m | 0.24 | 0.45 | 0.21 |
| none | 1.0–8.0 m | 0.19 | 0.31 | 0.13 |

The defaults are set to that optimum. **0.40 is the problem.** The
present-arm flight tracked its target for 100% of the run within 30 m, and the
check still calls ABSENT on two ticks in five of it. A controller that gated on
this per tick would abandon a target it was following perfectly.

So the honest reading is:

> Over a whole flight, 75% ABSENT against 40% ABSENT separates "the object is
> there" from "it is not". Per tick, it does not.

That is a real result — it is the first signal in this system that responds to
absence at all, and it is built from geometry rather than from a score — but it
is an **indicator**, not a verdict, and it must not be presented as the drone
knowing the object is missing.

## Why it cannot do better as built

In a dense city there genuinely are white, car-sized, car-distance objects: parked
vans, pale signage, sections of wall at the right range. Three geometric checks
cannot separate those from a car, because geometrically they are a car. The
detector says "car-like thing here" and the geometry agrees.

What would separate them is **appearance identity** — is this the same object I
was following, not merely a similar one — which is what `TargetLock` does for
position but nothing does for appearance. A short appearance descriptor per
tracked instance is the next honest step, and it is a different piece of work
from this one.

## What is safe to say

* ✅ "With no target present the system now reports ABSENT on 75% of ticks
  against 40% when the target is there, using range-derived geometry rather than
  detector confidence."
* ✅ "It is the first thing in the pipeline that responds to absence."
* ❌ Not: "the drone knows when the object is not there."
* ❌ Do not gate the controller on it per tick.

The per-flight figure is in `metrics.json` as `frac_absent`, and the per-tick
verdict with its reason is in `flight_log.jsonl` as `presence` / `presence_why`.
