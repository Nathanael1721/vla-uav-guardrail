# What the contract locks, and what it leaves open

**Date:** 2026-09-01
**Branch:** `detector-survey`
**Status:** question answered from the primary source; detector survey run and
measured; no adoption recommended yet, for a reason the numbers make plain.

Two questions were asked: does the grant actually require ArduPilot SITL and
ROS 2, and can OWL-ViT be replaced with something better. This is the first time
they have been answered from the **seven grant PDFs in the repository root**
rather than from our own restatements of them - a caveat that has sat at the top
of `docs/CHECKLIST-remaining-work.md` since it was written.

---

## 1. ArduPilot and MAVROS 2 are locked. This is not a preference.

`Architecture constraints` opens by saying so:

> "The choices in this section are **locked inputs** to the simulation framework
> decision. They are not re-debated here."

What it locks:

| Constraint | Quotation |
|---|---|
| Hierarchical control | "The VLA does not drive motors directly. It emits a 10 Hz setpoint that ArduPilot's inner loop tracks at 100-400 Hz." |
| Where the Shield sits | "The Suffix Safety Shield sits between the VLA and MAVROS 2 and is the **last software layer before the autopilot**." |
| The bridge | "For Year 1, **MAVROS 2 is the canonical bridge**. AP_DDS ... is treated as a Q3 optional migration, not a blocker." |
| The backstop | "The autopilot's own GeoFence / FENCE_ACTION is the ultimate backstop - the Shield does not replace it, **it pre-empts it**." |
| The GCS | "Mission Planner via `mavlink-router` fan-out (parallel to MAVROS), not as a serial bottleneck." |
| Where the simulator runs | "Project AirSim is always desktop-resident in every topology - Unreal Engine 5 needs a discrete NVIDIA GPU, which the Orin doesn't have." |

`Grant overview` adds scope and acceptance:

> "End-to-end motor-PWM VLAs are out of scope for this grant."

and names five acceptance KPIs: mission success rate, **P0 violation escape rate
- target 0**, fail-safe trigger correctness, **mean repair magnitude**, **mean
time to safe**. The last two have still never been measured, which
`docs/CHECKLIST-remaining-work.md` already records.

**So: yes, and it is not optional.** ArduPilot SITL with MAVROS 2 is the
contracted path. It is also the rail this project already produces KPI-grade
numbers on, so nothing needs to change to satisfy it.

## 2. The detector is explicitly NOT locked

The same document, one paragraph later:

> "The VLA backend interface is **switchable** (CognitiveDrone, OpenVLA generic,
> BitVLA, in-house stubs for unit tests, etc.); every backend must conform to
> this 4-D output shape, but **the project does not commit to any one of them as
> a 'default'**."

Changing the detector is therefore *inside* the contract. One property must
survive: it has to stay **open-vocabulary**, because the grant title is
"Semantic-Spatial Translation" and the language interface is what that names.

---

## 3. The survey

`experiments/bench_detectors.py`, offline, on frames from the delivered
populated-city flight, at the **400x225** the flight actually feeds the
detector, with the demo's own queries. All candidates are Apache-2.0 and all run
on the **pinned** `transformers 4.40.1`, so the environment - which carries
OpenVLA's pins - does not move.

| backend | best latency | Hz ceiling | taxi | person | VRAM |
|---|---|---|---|---|---|
| **OWL-ViT** (current) | **100 ms** | **9.9** | 0.182 | 0.062 | 0.68 GB |
| OWLv2 base | 472 ms | 2.1 | 0.620 | 0.104 | 2.57 GB |
| **Grounding DINO tiny** | 285 ms *(tuned)* | 3.5 | **0.743** | **0.323** | 3.28 GB |
| Grounding DINO base | 569 ms | 1.8 | 0.783 | 0.311 | 3.53 GB |

Scores are not comparable in absolute terms across architectures, so the
within-model person/taxi ratio is also reported in
`docs/data/detector_bench.json`.

### What is genuinely better

**Grounding DINO tiny detects our pedestrians at 0.323 against OWL-ViT's
0.062 - 5.2x stronger**, and the taxi at 0.743 against 0.182. That is the exact
weakness that forced the 10 m stand-off onto the camera-free rail.

### What kills it anyway

**None of the alternatives meets the 4.0 Hz detector gate**, and tuning does not
rescue them:

- Grounding DINO's default preprocessing upscales to 800 px; capping it at 400
  nearly halved latency (472 -> 285 ms) and cost almost nothing in score
  (person 0.399 -> 0.323). It still ceilings at **3.5 Hz**.
- Autocast fp16 made it **slower**, not faster (333 ms), and shrinking further
  to 256 px barely moved it (294 ms). The cost is in the decoder, not the image.
- OWLv2 **cannot be resized at all** - its position embeddings are fixed at
  960x960, so 2.1 Hz is a floor, not a starting point.

And these are ceilings on an **idle GPU**. In flight OWL-ViT measures 4.3x worse
than its idle figure because it shares the machine with Unreal. A model that
ceilings at 3.5 Hz idle would run near 1 Hz in flight.

## 4. Recommendation

**Keep OWL-ViT in the per-tick loop.** It is the only candidate that clears the
gate, and the gate exists because a stale target position is a control problem,
not a cosmetic one.

**But the accuracy gap is large enough to design around.** The 2026-08-19 review
already floated "a two-rate variant worth proposing" for the ViT/VLA interface,
and this measurement is the argument for it: run Grounding DINO at a low rate
for acquisition and re-acquisition, where 300 ms is affordable because it
happens rarely, and keep OWL-ViT per tick for tracking a box that is already
held. The instance lock added this week is exactly the machinery that makes a
low-rate re-acquisition safe to act on.

That is a design proposal, not a change. It is not made on this branch.

## 5. A mistake worth recording

The first benchmark run reported Grounding DINO scoring **0.0000** on "a person"
and I nearly wrote it up as "cannot see pedestrians". It was my bug.

`post_process_grounded_object_detection` takes a `text_threshold` that decides
which text tokens a box matches. I had set it to 0.0 by analogy with
`box_threshold`, where 0.0 correctly means "keep everything and let the caller
filter". At 0.0 every box came back labelled with the entire prompt -
`"a yellow car. a person. [SEP]"` on all 115 boxes - so the per-phrase label was
unrecoverable, every box fell to phrase 0, and the second query looked empty.

At 0.1 the same frame yields people at 0.369. The reported conclusion would have
been the exact opposite of the truth, about the one model that is best at the
thing we are weakest at.
