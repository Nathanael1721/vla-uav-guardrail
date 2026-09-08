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

## Why the ring never fired — the answer, found on the fifth review

The flight does not show the ring changing, and for two days this document said
the reason was that the aircraft could not close on the subject, and pointed at
the detector survey's measured weakness on pedestrian meshes.

**That was wrong. The aircraft did not close because I told it not to.**

`--want-width` is an ANGULAR target, so the stand-off it asks for scales with the
subject's real width: 0.16 is **15.83 m** against a 4 m car and **1.98 m**
against a 0.5 m person. The retarget block updated `args.object_width_m` — with a
comment explaining that a car prior left on a person misreads the range eightfold
— and never recomputed `want_range`, which is derived from it once before the
control loop and read on every tick.

So for the whole pedestrian half of the flight the servo was holding **the car's
15.83 m**. Measured on `demo/out/retarget_demo2`:

| | |
|---|---|
| post-retarget ticks with a served range | 319 |
| within one metre of the car's 15.83 m set-point | **54 (17 %)** |
| closest served range | **14.68 m** |
| commanded `vx` while in that band | **−1.18 m/s** — actively backing off |
| ticks ever inside the 10 m ring | **0** |

The aircraft was station-keeping at the range it had been given. A 10 m ring
cannot fire at 15.8 m, and nothing was going to make it, because the pilot was
never asking to be closer.

The comment two lines above the set-point had already worked this out — *"0.16
was tuned against a 4 m car and gives 15.8 m; the same flag against a 0.5 m
pedestrian asks for 2.0 m, which is not a small adjustment but a different
mission"* — and the code did not act on it.

## What the fix makes the demo do

`want_range` is now recomputed on every retarget, and the flight prints the
change. On the demo's own command line:

| phase | set-point | what happens |
|---|---|---|
| `a yellow car` | 15.83 m | holds; the 5 m catch-all is never breached |
| `a person` | **1.98 m** | the pilot drives IN |
| | | **the 10 m pedestrian ring stops it at 10 m** |

And 10 m is outside the camera's near blind spot (0.86 × 8 m = 6.9 m at cruise),
so the subject stays in frame while the ring holds it. That is the demonstration
the 2 September review asked for: one flight, one policy, one word changed, and
the enforced distance moves because the class did.

**This has not been flown yet.** The mechanism is fixed and pinned by a test that
parses the retarget block and requires it to assign `want_range`; the flight is
the next thing to do.

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
| before retarget (ticks < 248) | 247 | **0.915** | 7.6 px | 17 |
| after retarget (ticks ≥ 248) | 319 | **0.000** | 615.1 px | **319 of 319** |

*(The pre-retarget figure was published as 0.931 until 2026-09-08. It moved when
rows whose subject was out of shot stopped being credited: a 100 px tolerance is
half the 45 deg half-FOV, so a subject up to 67.5 deg off the nose could land
within tolerance of a box at the frame edge and be counted both out of shot and
on target. Seventeen rows here; all twelve of `city_kpi`'s.)*

**And 0.915 is less evidence than it looks.** The aircraft yaws to point at what
it is following, so the subject sits near the frame centre — and a detector that
simply emits the frame centre every tick, never opening the image, scores
**0.830** on this same half. The margin over that floor, +0.085, is the evidence;
at a 25 px tolerance it is +0.150. Every result now carries its floor.

319 out of 319 is not a detector result; no detector is wrong every single frame.

And the flight-wide **0.399 is exactly 226/566**: 226 detections were on target,
all 226 of them before the retarget, out of 566 scored. The figure is the
pre-retarget successes diluted by the post-retarget rows — it measured where the
retarget was, not how well anything was tracked.

*(Corrected 2026-09-08: this read "approximately 248/567", which is 0.437 and
7.7 % away. 248/567 is the fraction of TICKS before the retarget; the identity
that actually holds is the one above, and it is the stronger statement because
every on-target detection in the flight is pre-retarget.)*

**So the honest statement is: post-retarget tracking is UNMEASURED.** Not good,
not bad — never scored, because nothing logged where the pedestrians were. The
detector survey's finding (OWL-ViT scores our pedestrian meshes 0.062 against the
taxi's 0.182) stands on its own evidence and is a fair reason to *expect*
difficulty. It is not evidence that difficulty occurred here.

## The thing nobody was looking at — and the reason I first got it backwards

**Retracted 2026-09-08.** This section used to report that the estimator feeding
the Shield "rejected the measurements taken at the closest approach", and drew a
structural moral from it: *a stand-off rule is only as good as the position it is
told*. Four consecutive ticks, t+55.9 s to t+56.3 s, where the camera read 5 m
and the served estimate held 24.7 m.

**The estimator was right and the metric was wrong.** Depth is SLANT range along
the camera ray, so for a subject on the ground it can never be less than the
aircraft's own altitude. At those four ticks the aircraft was at **8.198–8.236 m**
and the reading was **5.0 m** — geometrically impossible for a pedestrian. The
estimator gated them out because they were impossible. That is the estimator
doing its job.

What was actually broken was `range_agreement`, the metric I wrote to catch the
blindness: it accepted `rng_m` as "what the camera measured" with no plausibility
test at all, so its **entire non-zero result on the only flight it is pinned
against** was produced by two bad detections. It now rejects a range below the
aircraft's altitude and counts those separately as `ticks_range_implausible`.
Re-run on the same flight:

| | before | after |
|---|---|---|
| `ticks_raw_inside_est_outside` | 4 | **0** |
| `ticks_range_implausible` | — | 4 |

So the honest statement about this flight is the plainer one: **the estimator and
the camera did not disagree anywhere that matters.** The structural point stands
as a thing worth measuring — nothing compared the served position against the
measured one, and now something does — but this flight is not an example of it,
and I published it as one.

The failure is the same shape as every other in this document: a check that could
not fail the way it was written, and a claim that outran it. A metric built to
catch a credulous estimator was itself credulous.

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

Closing that needed the truth logging above (done, so the next flight is
measurable at all) and the set-point fix (done, and it was the actual blocker).
Better pedestrian assets and a stronger acquisition detector remain worth having
- the detector survey's measured 0.062 on our pedestrian meshes is real - but
they were never what stood between this argument and a video. A stale servo
set-point was.
