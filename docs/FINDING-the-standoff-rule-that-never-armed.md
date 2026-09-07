# The 10 m stand-off rule never armed, and nothing could have told us

**Date:** 2026-09-07
**Found by:** flying the mid-mission retarget for the 18 September demo.
**Status:** fixed, with regression tests. The demo itself is **not yet
demonstrated in flight** — see the last section, which is the honest part.

## What happened

The plan was to fly the answer to Prof. Lai's question: one flight, one policy,
the subject re-labelled part-way through, and the enforced stand-off moving from
5 m to 10 m because only the *word* changed.

The flight ran. The retarget fired at t+30 s. Safety held — P0 escape rate 0.0,
no-fly-zone 0.0 s, altitude 0.0 s. And the 10 m rule **never fired once**:

| rule | ticks it bound |
|---|---|
| `standoff-any` (5 m, `subject_class: "*"`) | 2 |
| `standoff-pedestrian` (10 m) | **0** |

Across a whole flight whose subject was a person.

## Why

`SubjectStandoff.binds()` compares class strings exactly:

```python
return self.subject_class.lower() == subject_class.lower()
```

The policy names the class **`"pedestrian"`**. `subject_width("a person")`
returned the matched word — **`"person"`**. `"pedestrian" == "person"` is False,
so the rule did not bind, and the 5 m catch-all applied instead.

Two vocabularies had drifted apart with nothing checking they agreed:

- what an operator types, via `SUBJECT_WIDTH_M` — *pedestrian, person, human,
  man, woman* all describe the same thing;
- what a policy names in `subject_class` — only ever the literal `"pedestrian"`.

Of the five synonyms, **one** armed the rule. Which one you got depended on how
the operator happened to phrase the target.

## Why nothing reported it

This is the part worth keeping. The rule was **present** in the policy,
**hashed** into `policy_hash`, and **written to the audit log** — every property
this project uses to argue a rule is real. It was also completely inert.

Nothing detected it because *no violation is indistinguishable from no rule*. A
rule that never fires and a rule that is never breached produce the same
artefacts: no violations, no repairs, a clean KPI table. The flight scored a P0
escape rate of 0.0, which was true and meant nothing about the 10 m rule.

The metric that eventually caught it was not a metric at all. It was reading the
`violations` counts per rule id and noticing that one of them was zero when it
should not have been.

## The fix

Two parts, because either alone would leave the hole open.

**Canonicalise the phrase side.** `SUBJECT_CLASS_CANON` maps every width word to
the class a policy actually names — *person / human / man / woman / pedestrian*
all become `"pedestrian"`. The fuzziness now lives where the human words are;
`binds()` stays an exact comparison, so a policy rule still means exactly one
class.

**Refuse to fly an inert rule.** Before take-off, `follow_vlm.py` now checks
every `SubjectStandoff` in the policy against the set of classes a phrase can
produce, and exits with an error naming the rule if one is unreachable. A rule
that can never bind is now a start-up failure, where it is cheap and loud.

Verified after the fix — all five synonyms arm the 10 m rule, and vehicles
correctly fall to the 5 m catch-all:

| phrase | class | ring |
|---|---|---|
| a person / a man / a woman / a human / a pedestrian | `pedestrian` | **10 m** |
| a yellow car / a taxi | `car` | 5 m |

## What is still not demonstrated

The mechanism now works and the class is right. The **flight still does not show
the ring changing**, and the reason is not the Shield.

Re-flown after the fix: the retarget fired, the class became `pedestrian`, safety
held — and no stand-off rule bound, because the position the Shield was **served**
never came inside **14.7 m** (median 20.5 m). A 10 m ring cannot fire at 14.7 m.

## Correction, 2026-09-07 — the reason I gave for that was not measured

The first version of this document explained the stand-off distance by saying the
aircraft could not hold the target: *`frac_on_target` 0.406 in the pedestrian
phase, exactly what the detector survey predicted.* That was wrong, and it was
wrong in the way this project keeps catching: a number was quoted without
checking what it had been computed against.

`track_truth.score_rows` scored every detection against `tgt_x/tgt_y`, the only
ground truth in the flight log — **the car**. After the retarget the subject was a
pedestrian and the car was behind the aircraft, so all 319 remaining detections
were compared to it. The give-away is in the numbers already published:

| phase | scored | `frac_on_target` | median err | target out of shot |
|---|---|---|---|---|
| before retarget (ticks < 248) | 247 | **0.931** | 7.6 px | 17 |
| after retarget (ticks ≥ 248) | 319 | **0.000** | 615.1 px | **319 of 319** |

319 out of 319 is not a detector result; no detector is wrong every single frame.
And the flight-wide 0.406 is approximately 248/567 — the fraction of the flight
that happened *before* the subject changed. The figure measured where the retarget
was, not how well anything was tracked.

**So the honest statement is: post-retarget tracking is UNMEASURED.** Not good,
not bad — never scored, because nothing logged where the pedestrians were. The
detector survey's finding (OWL-ViT scores our pedestrian meshes 0.062 against the
taxi's 0.182) stands on its own evidence and is a fair reason to *expect*
difficulty. It is not evidence that difficulty occurred here.

## What the artefact does support, and one thing nobody was looking at

Re-read properly, the flight says something narrower and more useful.

The estimator that feeds the Shield **rejected the measurements taken at the
closest approach**. Between t+55.9 s and t+56.3 s the monocular range read 5 m
while the served estimate held 24.7 m and the gate counter climbed on every tick.
Four consecutive ticks — 0.4 s — where the *measured* range was inside the 10 m
ring and the *served* range was outside it.

Whether the aircraft was truly that close is still unknown, for the same reason as
above. But the structural point does not depend on knowing:

> A stand-off rule is only as good as the position it is told. Nothing in the KPI
> set compared what the Shield was served against what was measured, so an
> estimator holding a subject 20 m from where the camera said it was produced a
> clean escape rate and a clean audit log.

This is the same shape as the inert rule above. Zero violations, zero repairs, and
in neither case did that mean the rule was working.

## The fixes, and how each one fails loudly now

| Defect | Fix | Where it now shows |
|---|---|---|
| Truth was the car, whatever the subject | `subject_truth_pts()` logs truth for the class in force; `truth: {class, pts}` per tick | `demo/follow_vlm.py` |
| A detection with no truth was scored anyway | rows with no truth are counted as `det_unscorable`, never scored | `demo/track_truth.py` |
| "Any person" was scored against one person | truth is a LIST; a box on any pedestrian answers the question that was asked | `truth_points()` |
| Served range never compared to measured | `range_agreement` reports `ticks_raw_inside_est_outside` and the longest blind run | `metrics.json` |
| Estimator counters mixed two tracks | `reset()` banks them; `resets` / `updates_total` / `gated_out_total` reported | `demo/target_state.py` |

Pinned by tests in `tests/test_track_truth.py`, `tests/test_range_and_lock.py`
and `tests/test_target_state.py`, including two that assert the numbers above
directly against `demo/out/retarget_demo2` so this document cannot drift from the
artefact it describes.

## What 18 September can honestly claim

The *capability* is real and provable from the sweep: `standoff-reclassified`
moves the enforced ring 5 m → 10 m on a class change alone, and no tracker that
returns an ID instead of a class can be asked to do it.

The *flight* demonstrates the retarget — the phrase changes, the class changes,
the policy re-binds — and does not yet demonstrate its consequence, because the
aircraft never closed to 10 m of the position the Shield was given.

Closing that needs the truth logging above (done, so the next flight is
measurable at all), and then either better pedestrian assets or a stronger
detector for the acquisition phase. That is still the two-rate design the
detector survey argued for. What has changed is that the next flight will
produce a number that means what it says.
