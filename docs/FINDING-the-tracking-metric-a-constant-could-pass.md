# A detector that never opens the image scores 1.000 on our best flight

**Date:** 2026-09-08
**Found by:** the third round of adversarial review, pointed at the fix the
second round produced.
**Status:** fixed. Every tracking figure in the project now ships with the floor
a zero-skill detector reaches on the same rows, and the published figures moved.

## The claim that was being made

`frac_on_target` is the number this project uses to say the tracker held the
right thing. It replaced `det_hit_rate` in August, for a good reason recorded at
length: the old one counted any box at all, so a box on a building scored like a
box on the taxi. The new one projects the true subject into the frame and asks
whether the chosen box was within **100 px** of it.

Published figures: `city_locked` 1.000, `demo_traffic` 1.000, `city_kpi` 0.930,
the retarget flight's first half 0.931.

## What a constant scores

The aircraft **yaws to point at what it is following**. So the subject sits near
the middle of the frame almost always — measured mean `cx / img_w` = 0.51 — and
100 px is a quarter of a 400 px frame. A "detector" that emits the frame centre
on every tick, never opens the image, and cannot tell a car from a wall therefore
scores:

| flight | real detector | centre constant | margin |
|---|---|---|---|
| `city_locked` | 1.000 | **1.000** | **0.000** |
| `demo_traffic` | 1.000 | 0.976 | +0.024 |
| `city_kpi` | 0.908 | 0.876 | +0.032 |
| `city_full` | 0.728 | **0.753** | **−0.025** |
| retarget, first half | 0.915 | 0.830 | +0.085 |

On `city_locked` — the flight that produced the demo video, the one whose 1.000
is quoted as evidence the instance lock works — **the margin is exactly zero**.
On `city_full` the constant *wins*.

The detector is genuinely better. Its median error on `city_full` is **7.1 px**
against the constant's 11.6. The 100 px tolerance simply cannot see it:

| tolerance | real | constant | margin |
|---|---|---|---|
| 100 px | 0.779 | 0.806 | **−0.027** |
| 50 px | 0.747 | 0.735 | +0.011 |
| 25 px | 0.728 | 0.665 | +0.063 |
| 10 px | 0.581 | 0.469 | +0.112 |

So the finding is not "the tracker is bad". It is that **the statistic we chose
to prove it is good cannot distinguish it from a constant**, and we had been
quoting the statistic, not the evidence.

## Why 100 px was chosen, and why that reasoning was incomplete

The module's own docstring says 100 px is *"far wider than any plausible
box-centre error on the right vehicle (median is 3–8 px), and narrow enough that
a different vehicle in another lane fails it."* Both halves are true. The
tolerance was designed to separate **the right car from another car** — and it
does.

What it was never checked against is the null hypothesis: *what would something
with no skill at all score?* Nobody asked, because the answer felt obvious.
Uniform noise across the frame scores 0.13, which is the answer if you picture a
box thrown at random. But the right null is not a box thrown at random. It is the
cheapest thing that exploits the same geometry the real detector benefits from —
and pointing the camera at the subject is geometry the detector gets for free.

## The fix

Three changes, none of which touches how the detector works.

**Only a subject in shot can be credited.** 100 px is 22.5°, which is *half* the
45° half-FOV, so a subject up to 67.5° off the nose could land within tolerance
of a box at the frame edge — and be counted in `n_det_with_target_out_of_fov` and
in the `frac_on_target` numerator at the same time. All twelve of `city_kpi`'s
out-of-shot rows were being credited. This is what moved the published figures:
`city_kpi` 0.930 → **0.908**, the retarget first half 0.931 → **0.915**.

**Every result carries its floor.** `frac_on_target_chance` is the harder of two
null models — a uniform draw, and the centre constant — computed by the *same
code path* on the *same rows*, so it cannot drift from the statistic it bounds.
`frac_on_target_margin` is the difference, which is the actual evidence.

**A tolerance that discriminates.** `frac_on_target_25px` and its own floor and
margin, at 5.6° — still three times the median error of a correct box. Every
flight has a positive margin there, including `city_full`.

The deck computes the floor too, in its own JavaScript, and prints it beside the
score; `tests/test_deck_scorer_parity.py` runs the deck's function under node
against the Python so the two cannot diverge. The centre-constant null needs no
random draw, which is why it ports exactly.

## What to say about tracking from now on

Not *"0.915 on target"*. Either:

> 0.915 on target against a 0.830 floor — a margin of 0.085, and 0.150 at a 25 px
> tolerance — over 247 detections with a median error of 7.6 px.

or just the median error, which was always the honest number and never needed a
floor.

## The pattern, for the fourth time this week

A check that could not fail. The stand-off rule that could not fire because its
class never matched; the bundle verifier that compared five fields and reported
seven; the tests that printed PASS when they had skipped; and now a metric whose
threshold was so loose that not looking at the image scored the same as looking.

Every one was found the same way — by constructing the case that *should* break
it and watching nothing break. That is now the cheapest tool in this project, and
it should be reached for before a number is published rather than after.
