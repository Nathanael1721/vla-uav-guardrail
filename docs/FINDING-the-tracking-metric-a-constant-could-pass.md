# A detector that never opens the image scores 1.000 on our best flight

**Date:** 2026-09-08, extended 2026-09-09
**Found by:** the third round of adversarial review, pointed at the fix the
second round produced — and then the fourth round, pointed at that fix.
**Status:** fixed, after one wrong turn. The first answer was to publish a floor
beside the score; the fourth review beat the floor. The headline is now the
median pixel error, which no null on disk beats on any flight.

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

**Every result carries its floor.** `frac_on_target_chance` is the hardest of a
null *family* — a uniform draw, the centre constant, and the best fixed column
for that flight — computed by the *same code path* on the *same rows*, so it
cannot drift from the statistic it bounds. It is still only a lower bound on what
a zero-skill detector could reach, which is the honest reason it is not the
headline.

**A tolerance that discriminates — and then did not.** `frac_on_target_25px`
was added at 5.6°, on the reasoning that it is several times the median error of
a correct box. That much holds — on the retarget flight's first half 25 px is
**3.5×** the in-shot median of 7.1 px — but **two flights have a negative margin
even there** (`envactor3` −0.041, `envactor_white` −0.015).

*(An earlier version of this paragraph said "25 px is 2.1× the median, not 3×".
That ratio is not produced by any artefact: 3.5× on the pre-retarget half this
document quotes, 3.3× on its all-rows median, and 1.3× only on the whole of
`retarget_demo`, a different flight.)*

A fourth review then found the deeper problem. A fixed column at **cx = 158 px**
BEATS the real detector on `retarget_demo`, and a lag-1 baseline — emit the previous tick's box
centre, never open the current image — erases the 25 px margin on `city_kpi`
(+0.077 → −0.000). Every floor invites a harder null, because threshold counting
at any robust tolerance puts most rows inside it for everybody.

**So the headline moved to the median**, which was in the module from the start.
Measured against three nulls on every flight with scored rows, the median beats
all of them everywhere, while the fraction is within a whisker of lag-1:

| flight | n | real | centre | best constant | lag-1 |
|---|---|---|---|---|---|
| `city_locked` | 521 | **2.3** | 5.3 | 4.7 | 2.5 |
| `city_kpi` | 545 | **3.5** | 9.4 | 6.9 | 3.7 |
| `city_full` | 525 | **7.1** | 11.6 | 10.1 | 7.4 |
| `demo_traffic` | 548 | **4.9** | 7.7 | 7.7 | 5.6 |
| `retarget_demo` | 505 | **18.7** | 36.1 | 33.1 | 20.2 |
| `people_check` | 377 | **7.1** | 12.8 | 11.4 | 7.4 |

`frac_on_target` is kept for continuity with the published flights and is no
longer the evidence anywhere. The deck now prints the median against the
centre-constant on all three slides that used to print the bare fraction —
including the instance-lock comparison, which improves from a meaningless
"72.8 % → 100 %" to **7.1 px → 1.9 px** against a null of 11.6 → 4.9.

The deck computes the median and the centre-constant null in its own
JavaScript; `tests/test_deck_scorer_parity.py` runs the deck's functions under
node against the Python so the two cannot diverge. The centre-constant needs no
random draw, which is why it ports exactly — and the deck names the null it
compares against rather than calling it "the floor", because the Python's floor
is the hardest of a family and calling both "chance" hid the deck printing a
weaker one and a larger margin.

**Both medians are computed over the same rows** — the in-shot ones.
`det_gt_err_px_median` still spans every scored row, because that is what the
published flights quote; `det_gt_err_px_median_in_shot` is the one to compare.
Mixing them made `people_check` read 17.2 px against a null's 12.8 for about
twenty minutes.

## What to say about tracking from now on

Not *"0.915 on target"*. Say:

> Median error **7.1 px** with the subject in shot, against **14.9 px** for a
> detector that emits the frame centre and never opens the image — over 247
> detections, with the subject in frame on 93.1 % of them.

The fraction can follow as context, never as the claim.

## The pattern, for the fourth time this week

A check that could not fail. The stand-off rule that could not fire because its
class never matched; the bundle verifier that compared five fields and reported
seven; the tests that printed PASS when they had skipped; and now a metric whose
threshold was so loose that not looking at the image scored the same as looking.

Every one was found the same way — by constructing the case that *should* break
it and watching nothing break. That is now the cheapest tool in this project, and
it should be reached for before a number is published rather than after.
