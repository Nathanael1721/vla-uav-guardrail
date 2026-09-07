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
held — and no stand-off rule bound, because the aircraft never got closer than
**14.7 m** to the estimated subject. A 10 m ring cannot fire at 14.7 m.

It stayed out there because it could not hold the target: `frac_on_target`
**0.406** in the pedestrian phase — the box was on the wrong thing for most of
it. That is exactly what the detector survey predicted. OWL-ViT scores our
pedestrian meshes at **0.062** against the taxi's **0.182**; it is the measured
weak point, and it is now the thing standing between this argument and a video.

So the honest position for 18 September: the *capability* is real, tested and
provable from the sweep, and the flight demonstrates the retarget but not the
consequence. Closing that needs either better pedestrian assets or a stronger
detector for the acquisition phase — which is precisely the two-rate design the
detector survey already argued for, and the strongest evidence yet that it is
worth building.
