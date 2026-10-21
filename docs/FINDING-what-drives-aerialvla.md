# What actually drives AerialVLA: the camera, the object phrase, or the compass hint?

**Date:** 2026-08-04
**Status:** offline ablation complete (108 controlled forward passes per adapter);
simultaneity flights pending.

**Corrected 2026-10-21** (section "Correction, 2026-10-21" directly below): the
tables of 4 Aug read the model's text with a parser that turned malformed
landing outputs into motion, and labelled step quantities as rates. Read
strictly, the original adapter stops on **17 of 18** no-hint frames, and the
object-presence effect is **not supported**. The 4 Aug text is left as it was
written; the correction says which of its numbers changed.

## Correction, 2026-10-21: the output parser, and the units

**What was wrong.**

1. *The parser.* `experiments/ablate_image_vs_hint.py` (4 Aug, lines 82-85) took
   the last three integers anywhere in the decoded text,
   `re.findall(r"\d+", tail)[-3:]` - upstream's own `_parse_action_from_text`
   (`source/AeroVLA/src/model_wrapper/aerialvla_wrapper_ui.py:208-223`), which
   `demo/aerialvla_demo.py` copied too. On a malformed output that is not a
   misreading at the margin, it is a different action: `'00 49 494 49'` gives
   (49, 494, 49), the 494 is clamped to bin 98, and a stop becomes forward 2.5 and
   "down" +5.0; `'00 49 4949'` gives a yaw bin of 4949, clamped to 98, i.e. +1.1.
2. *The units.* Upstream defines the three numbers as a **step**: forward metres
   along the new heading, vertical metres, and a heading change in radians, per
   decision, executed at 1 m/s with a 0.5 s hover between decisions
   (`source/AeroVLA/airsim_plugin/AirVLNSimulatorClientTool_AerialVLA.py:300-385`).
   Every "rad/s" and "m/s" in the 4 Aug tables is that same number in radians or
   metres **per step**.

**Re-score.** The same stored outputs (`experiments/out/ablation_orig.json` and
`ablation_ft.json`, no new inference) read with the strict reader
`guardrail/aerialvla_codec.parse_action`: exactly three two-digit bins 00-98,
single spaces, an optional ` LAND`, then EOS; anything else is a parse failure,
which the client contract (VLA server protocol v2) holds as zero and counts.
Every row, its stored text, the old reading and the new one are in
`docs/data/aerialvla_ablation_rescore.json`
(`python experiments/rescore_ablation.py --src <dir of the two files>`).
Outcomes: *LAND*; *hold* (parse failure, flown as zero); *move*.

Original adapter, per phrase (n = 18 each; dpsi in rad/step and forward in m/step
are means over the outputs the strict reader accepts):

| direction phrase | accepted | mean dpsi | mean forward | LAND | hold | move | 4 Aug said (yaw, fwd, LAND) |
|---|---|---|---|---|---|---|---|
| `to your left` | 18 | -0.466 | 4.24 | 0 | 0 | 18 | -0.466, 4.24, 0/18 |
| `forward-left` | 18 | -0.167 | 4.20 | 0 | 0 | 18 | -0.167, 4.20, 0/18 |
| `straight ahead` | 18 | -0.035 | 3.80 | 0 | 0 | 18 | -0.035, 3.80, 0/18 |
| `forward-right` | 17 | +0.114 | 4.08 | 0 | 1 | 17 | +0.121, 3.87, 0/18 |
| `to your right` | 18 | +0.161 | 4.11 | 0 | 0 | 18 | +0.161, 4.11, 0/18 |
| **(none)** | 12 | **0.000** | 0.21 | **11** | **6** | **1** | +0.182, 0.56, 11/18 |

- **No hint: 17 of 18 frames stop** - 11 clean `00 49 49 LAND` outputs and 6
  malformed ones that all begin with the stop triple (`'00 49 4949'` twice,
  `'00 49 494 49'`, `'00 49 4944949'`, `'00 49 49 49 49 48'`, `'00 49 49 L 49'`).
  The 18th, `'49 49 49'` (forward 2.5 m), is the only no-hint motion. The 4 Aug
  row's +0.182 "yaw" and 0.56 "forward" were the malformed rows read as +1.1 and
  2.5. As the client would fly them (holds as zero) the no-hint mean forward step
  is 0.14 m. This fits how the model was trained: the empty phrase appears once
  per training trajectory, on its final frame, with a zero label and LAND.
- **The object-presence effect is not supported.** The 4 Aug comparison (no
  hint, target present vs absent) said forward 1.11 vs 0.00, LAND 4/9 vs 7/9,
  "weakly positive". Strictly: present 1/9 frames move (the one `'49 49 49'`),
  4 LAND and 4 holds; absent 0/9 move, 7 LAND and 2 holds. Three of the four
  present-scene "forward 2.5" rows were stop outputs read as motion. One frame
  in nine against none is not evidence that the object's presence changes the
  action.
- **Hinted rows: 0 LAND in 90**, as before; one hinted output is malformed
  (`'9运 48 60'` under `forward-right`, a CJK character inside a bin, read on
  4 Aug as bins 9/48/60).
- **What still stands.** The phrase orders the heading change monotonically
  (-0.466 < -0.167 < -0.035 < +0.114 < +0.161 rad/step), the object phrase does
  not steer (target present, strict: `orange barrier` +0.053, `blue barrier`
  +0.018, none -0.102 rad/step; LAND 1, 1, 2), and the factor spreads keep their
  order (dpsi range when one factor varies: hint 0.880, image 0.877, object 0.428
  rad/step, against 1.005 / 1.000 / 0.454 on 4 Aug).
- **The fine-tune (`run2/epoch1`).** 96 of 108 outputs are accepted, not "106
  of 108 parsed": the old reader also accepted outputs with a one-digit bin.
  Three of those (`'15 51 0'`, `'15 49 0'`, `'25 49 0'`) are under `to your
  left` and were read as yaw bin 0 = -1.1, so its mean is **-0.428**, not
  -0.540; `to your right` stays +0.508. The "roughly ±0.52, symmetric" reading of
  the extremes becomes -0.43 / +0.51. No hint: 16 of 18 stop (9 LAND, 7 holds),
  2 move (`'49 49 49'` twice, both target present). The conclusion that the
  fine-tune follows the phrase harder at the right extreme and leaves the object
  slot inert is unchanged.

**Where the malformed landing outputs come from (measured 2026-10-21).** The
serving measurement (`docs/data/aerialvla_serving_4090.json`, `probe_outputs`)
ran the same six mosaics with the empty phrase and the object `orange barrier`
through eight model configurations. Every bf16 configuration (upstream's
precision) answers `00 49 49 LAND` on five frames and `49 49 49` on
`present_+00`. The 4-bit, unmerged loader that every earlier AerialVLA script
here used (and that produced the 4 Aug outputs) gives `00 49 4949` on
`absent_+35` and `00 49 494 49` (cut at 12 new tokens) on `present_+35`. That
reproduces, frame for frame, the six no-hint `orange barrier` rows of 4 Aug
(`ablation_orig.json`). The same loader with SDPA attention is malformed on the
same two frames (`00 49 4949`, `00 494494`). Both 4-bit merged configurations
answer clean `LAND` there. On these inputs, then, the malformed outputs come
from 4-bit quantisation with an unmerged adapter, not from the model's
training. On `present_+00` every bf16 configuration and both 4-bit unmerged
ones say `49 49 49`, and both 4-bit merged ones `90 49 49`, so that frame's
forward command is the
model's own answer at every precision. This is six inputs and one object
phrase, not a population.

Code fixed the same day: `demo/aerialvla_demo.py` and
`experiments/ablate_image_vs_hint.py` read through the strict reader (a
malformed output is held as zero and logged, where it used to be read as motion
or to leave the previous action latched), `demo/vla_bridge.py`'s
`PHRASE_YAW_RAD_S` is documented as radians per step, and the VLA server v2
(`demo/vla_server_v2.py`) returns `parse_ok` with every action.

## Why this was investigated

The question being answered was *"can we command the drone with something like
'follow the red car' or 'circle the white building', and show the VLA and the
guardrail working side by side?"*

Answering it needed a mission where the VLA is the only thing steering — no
planner, no path follower, no goal blend, and crucially **no direction hint**.
Building that surfaced two problems, one of which changes what the whole
AerialVLA integration means.

## Problem 1 — every earlier "semantic" run was confounded

`demo/semantic_demo.py` passed `target_xy = (0.0, 0.0)` intending "no target".
But `(0, 0)` is a real coordinate, not a sentinel: `semantic_direction()`
computed a genuine bearing to the world origin and put it in the prompt. From the
spawn point (35, −20) the model was being told **`"to your right rear "`** on
every single inference.

So the script's headline claim — "NO target coordinates, the VLA steers from
camera + language only" — was false in effect. Fixed: `semantic_direction`
now returns `""` for `target=None`, and the demo passes `None`.

## Problem 2 — the prompt slot that steers is the one derived from coordinates

AerialVLA's prompt, unchanged from the upstream wrapper, is

```
<image>
Fly {direction}and find the target. {object}
Action:
```

`{direction}` is a seven-way bearing phrase computed from the **ground-truth
target coordinates**. `{object}` is the language description. To find out which
slot the model actually uses, six real 224×448 front+down mosaics were captured
at the true experimental poses — target present vs absent × three headings — and
every combination of (frame × 6 direction hints × 3 object phrases) was run
through the model. 108 forward passes, no simulator, no flight-to-flight noise.

### Result, original AerialVLA LoRA

**The direction phrase orders the commanded yaw monotonically:**

| direction phrase | mean commanded yaw (rad/s) | mean forward (m/s) | LAND |
|---|---|---|---|
| `to your left` | **−0.466** | 4.24 | 0/18 |
| `forward-left` | −0.167 | 4.20 | 0/18 |
| `straight ahead` | −0.035 | 3.80 | 0/18 |
| `forward-right` | +0.121 | 3.87 | 0/18 |
| `to your right` | +0.161 | 4.11 | 0/18 |
| **(none)** | +0.182 | **0.56** | **11/18** |

Left phrases turn left, right phrases turn right, "straight ahead" turns
essentially not at all. That is the model doing exactly what it was trained to
do — and what it was trained on is a compass bearing derived from coordinates.

**Without that phrase the model largely stops.** Mean forward speed collapses
from ~4 m/s to 0.56 m/s and it emits `LAND` on 11 of 18 real frames.

**The object phrase does almost nothing.** With the target genuinely visible in
the pixels:

| object phrase | mean yaw | mean forward | LAND |
|---|---|---|---|
| `orange barrier` (correct, matches the pixels) | +0.050 | 3.90 | 1/18 |
| `blue barrier` (wrong colour, identical pixels) | +0.076 | 3.53 | 1/18 |
| *(none)* | −0.077 | 3.64 | 2/18 |

Correct and incorrect colour words are indistinguishable. Varying the object
phrase moves the yaw command about half as much as varying the hint or the
image, and with no consistent direction.

**The pixels do matter, but not demonstrably for object grounding.** Varying the
image with hint and object held fixed moves yaw as much as varying the hint
(mean range 1.00 vs 1.01 rad/s). But those six frames span three very different
headings, so that variance is largely *viewpoint*, not *the object*. The one
comparison that isolates the object — no hint, target present vs absent — is
weakly positive but small (forward 1.11 vs 0.00 m/s, LAND 4/9 vs 7/9, n = 9 per
side).

### Result, our fine-tune (`aerialvla-ft/run2/epoch1`)

The same 108 passes on our own checkpoint quantify exactly what the fine-tuning
campaign bought and what it cost.

| factor varied (others held fixed) | original | our fine-tune |
|---|---|---|
| direction hint → yaw range | 1.005 | **1.099** (more responsive) |
| image → yaw range | 1.000 | 0.827 (less) |
| object phrase → yaw range | 0.454 | **0.321** (less) |

| direction phrase | original yaw | fine-tune yaw |
|---|---|---|
| `to your left` | −0.466 | **−0.540** |
| `forward-left` | −0.167 | −0.149 |
| `straight ahead` | −0.035 | +0.016 |
| `forward-right` | +0.121 | +0.090 |
| `to your right` | +0.161 | **+0.508** |

The fine-tune follows the compass phrase harder and more symmetrically — the
extreme phrases now command roughly ±0.52 rad/s instead of the original's
asymmetric −0.47/+0.16. That is precisely what it was trained to do: its labels
came from a coordinate P-controller, which is a compass follower.

And the object slot became *more* inert, not less:

| object phrase (target visible) | original yaw | fine-tune yaw |
|---|---|---|
| `orange barrier` (correct) | +0.050 | +0.012 |
| `blue barrier` (wrong colour) | +0.076 | +0.008 |
| *(none)* | −0.077 | +0.005 |

All three collapse to about +0.01. The trade-off predicted before running this —
better path following, weaker object grounding — is confirmed and now has
numbers. The honest framing is that the fine-tune improved the thing that was
already doing the work and further suppressed the thing that was already nearly
inert.

(One caveat: 106 of 108 fine-tune passes parsed vs 108 of 108 for the original;
two outputs did not contain three integers.)

### What this means for the original question

**"Follow the red car" is not achievable with this model**, and not because of a
missing asset — because the language slot does not steer. The steering slot is
a bearing computed from coordinates the system was already given. Asked to fly
on language alone, AerialVLA lands.

This is worth stating plainly in the write-up, because the natural reading of
"we integrated a VLA" is that the aircraft is being flown by vision and language.
On this evidence, AerialVLA as published is closer to **a learned controller for
a coordinate-derived bearing**, with the camera contributing to *how* it flies
rather than *where* it decides to go.

It also retroactively explains the fine-tuning campaign: our training data
(`training/collect_aerialvla_data.py`) filled the object slot with five generic
strings carrying zero information about the label, and the labels came from a
coordinate P-controller. That data taught the model nothing about objects — but
it did not need to unlearn much either, because the object slot was already
close to inert.

## "Follow the car", flown

The offline ablation predicted that a language-only mission would not fly. It was
worth flying anyway, because a prediction from static frames is not the same as a
result, and because the mission was asked for directly.

**The car — and a correction.** The first version of this experiment used a
4.5 × 2.0 × 1.6 m box, on the conclusion that this simulator ships no vehicle.
**That conclusion was wrong.** The Rover plugin — `PASBlocks/Plugins/Rover` — was
in the project the whole time, and it contains `SM_Offroad_Body`, a real car body
mesh that the sim reports in its spawn registry with a bounding box of
3.70 × 1.79 × 1.16 m. The registry also lists `SKM_SportsCar` and
`SKM_Car_Template`.

Two mistakes produced that error. A directory listing was truncated before
`Rover` appeared, and the plugin was then dismissed as Linux-only because the
package it also ships in contains `librover_api.a`. But the `.uplugin` has no
`"Modules"` block at all — it is content-only, so nothing needs compiling and
`.uasset` files are platform-independent.

The results below were re-measured with the real mesh, painted with `M_Orange`:
unpainted it renders dark against dark asphalt and is hard to pick out at 22 m,
while painted it keeps the vehicle silhouette and gains a colour that matches the
word in the instruction. On colour: `Yellow` applies without error but renders
pale grey, and `MI_Emissive_Red` renders pale pink-white (3 red-ish pixels, mean
RGB 225/188/177), so `M_Orange` is the only verified option. "red car" would have
been the better phrase — 7 of AerialVLA's 94 descriptions are exactly that,
against 1 orange entry — but describing an orange object as red would break the
one thing a colour ablation rests on.

The car drives at 3 m/s along the x = 38 street, which the occupancy map shows
clear of buildings for 100 m, moved by client-side teleport
(`world.set_object_pose(..., teleport=True)`) rather than an environment actor,
because env actors must be declared when the scene loads.

**Camera geometry forced a low flight.** The front camera is horizontal with a
29.2° vertical half-FOV, so a ground object is in frame only beyond 1.79 ×
altitude. Measured against the car: at 15 m altitude and 20 m range it is out of
frame; at 9 m and 22 m range it is in frame at 29 px wide. Hence a 6–14 m band
(`policies/follow_car.yaml`), which is genuinely low for a city street and is the
mission's constraint rather than a preference.

**Results — 120 s per flight, car at 3 m/s. Flown twice: once against the box,
then re-flown against the real `SM_Offroad_Body` mesh once the earlier mistake
was found.**

| target | instruction | direction hint | closest | mean separation | final | within 30 m |
|---|---|---|---|---|---|---|
| box | "orange car" | **none** | 12.4 m | 38.2 m | 48.0 m | 35% |
| box | "orange car" | ground truth | 7.0 m | 61.4 m | 102.9 m | 23% |
| **real car** | "orange car" | **none** | 12.4 m | **38.3 m** | **47.8 m** | **34.5%** |
| **real car** | "orange car" | ground truth | 7.8 m | 50.4 m | 30.7 m | 18% |

**Swapping the box for a real car mesh changed the no-hint result by nothing.**
38.2 → 38.3 m mean separation, 48.0 → 47.8 m final, and the same twelve identical
`[0, 49, 49]` outputs. That is the important number in this table: it removes the
obvious objection to the earlier negative result. The model was not failing
because there was no car to recognise. There is a car now, correctly named,
clearly visible, and the aircraft still does not move.

The hinted flight did improve — it ended 30.7 m from the car rather than 102.9 m
— but its mean separation is 50.4 m and it spent 18% of the flight within 30 m,
against 34.5% for a drone that never left its spawn point.

**Without the hint the aircraft never moved at all.** All 12 inferences returned
the identical token triple `[0, 49, 49]` — stop, with `LAND` — from the prompt
`"<image>\nFly and find the target. orange car\nAction: "`. It sat at its spawn
point for 1112 ticks while the car drove past. Its 12.4 m "closest approach" is
the car coming to the drone, not the reverse.

**With the hint it moved, and followed worse than standing still.** It ended
102.9 m from the car, having wandered to x = 138 — outside the mapped area
entirely, where the building-clearance rule has no data to work with — and spent
23% of the flight within 30 m against the stationary drone's 35%.

That second result is the more interesting one. The direction hint is a seven-way
bearing phrase, so ±22° of quantisation, refreshed every ~8 s. Against a target
moving at 3 m/s that is not a tracking controller, and the aircraft chases a
bearing that was already stale when it arrived. Feeding the model exactly the
input it was trained on does not produce following; it produces a drone that
travels confidently in roughly the wrong direction.

**Conclusion:** "follow the car" is not achievable with AerialVLA here, and the
obstruction is structural rather than a matter of tuning. The language slot does
not steer, and the slot that does steer is too coarse and too slow to track a
moving target: a seven-way bearing phrase (±22° buckets) refreshed every ~8 s
cannot close on something moving at 3 m/s.

The "it was only a box" objection has been tested and does not hold — the real
car mesh produced the same null. What remains untested is whether a *different*
model would do better; nothing here says vision-language flight is impossible,
only that this particular checkpoint does not do it.

## Consequence for the experiment design

The originally planned 16-flight matrix spent ten flights asking whether the VLA
steers toward a named object and whether the correct colour word beats the wrong
one. Those flights would have produced ten landed drones and roughly nine usable
inferences each. The ablation above answers both questions with better control
and no flying.

What still requires flying is the **simultaneity** claim, and it is untouched by
any of this: whether the pilot is *competent* has no bearing on whether it is
*active at the same instant* as the Shield. Those flights therefore run with the
direction hint enabled (`--hint-mode truth`) — the input the model was trained
on, and the only setting that produces sustained flight. Every such flight
records `hint_mode: "truth"` and `hint_used: true` so it can never be misread as
a semantic result.

See `experiments/conditions_simultaneity.yaml` for the revised 8-flight matrix.

## Side findings

* **Inference is ~3.0 s per action standalone and ~10 s with the sim rendering
  on the same GPU** (RTX 4080 SUPER, 7B at 4-bit, 11 generated tokens at
  ~290 ms/token). The earlier 0.9–1.3 s figure does not hold under this load.
  At 10 s per action a 90 s flight yields nine decisions, which is why the
  `max_vla_age_s` metric gate had to be raised from 1.0 s to 4.0 s — at 1.0 s it
  would have excluded essentially every tick for mechanical reasons.
* **The Shield never edits yaw.** Verified empirically over 419 violation ticks
  spanning all four rule categories, plus a real flight:
  `experiments/verify_shield_yaw.py`. This is what licenses reading
  `emitted.yaw_rate == raw.yaw_rate` as "the VLA still owns the heading".
* **`KinematicEnvelope.yaw_rate_max_dps` has never fired.** *(Superseded
  2026-08-17, corrected 2026-09-08: it has fired on 213 ticks across 8 runs
  since the conversion moved to the boundary.)* It is named in
  degrees per second but compared against a value the simulator consumes as
  rad/s, which never exceeds ±0.44. Documented, deliberately not changed here —
  correcting it would alter every existing result in the repo and would make the
  Shield start editing yaw.

## Reproducing

```bash
python experiments/verify_shield_yaw.py                    # no sim needed
python experiments/capture_frames.py                       # sim; 6 frames
python experiments/ablate_image_vs_hint.py --label orig
python experiments/ablate_image_vs_hint.py --adapter D:/models/aerialvla-ft/run2/epoch1 --label ft
```
