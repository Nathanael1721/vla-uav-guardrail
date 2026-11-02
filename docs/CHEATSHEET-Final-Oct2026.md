# Presentation cheat-sheet — October 2026

For the deck `VLA-Guardrail-Final-Oct2026.pptx` (13 slides, 15–20 minutes).
Numbers first, then the timing, then the questions you will actually get.

Every number below is from the repo's own records. If you only remember one
rule, remember this one: **a safety number that is wrong is worse than one
that is missing.** Say "dev evidence" before anyone asks.

---

## The 30-second version, if you only get one paragraph

> We put a safety layer between a language model and a drone. The model says
> "follow the yellow car" and answers with a velocity command; the guardrail
> holds the declarative policy — fences, altitude bands, kinematic caps,
> stand-off rings, corridors — and enforces it on every command at 10 Hz,
> repairing what it can and refusing what it cannot. Across the whole project
> the pilot underneath changed completely — a 7 B action model, then a 153 M
> detector and a hand-written controller, then a fine-tuned VLA, now a
> hierarchy — and **the safety layer never changed.** P0 escape is 0.0 on 41
> shielded flights; the unshielded controls read 0.63–0.67.

## The three things that make it a result rather than a video

1. **The one-word control.** Same scene, same white car, one word changed —
   "a red car". The system does not follow badly; it **fails to acquire at
   all**: detector hit 99.3% → 22.1%, time within 30 m 100% → 24.1%. Without
   that control, "the drone ended up near the car" is just drift.
2. **The Shield-off controls exist to fail.** Same rails, same policies,
   Shield removed: P0 escape 0.63 (mid-evaluation, 5 flights), 0.661 and
   0.670 (13–14 Oct). Shield on: **0.0**, every time, 41 flights.
3. **The decision-rate ceiling.** The hand-written follow itself, held to the
   VLA's decision rate (one decision per second, 0.8 s old), scores **0.122**
   within 30 m — worse than the fine-tuned VLA's 0.238. The VLA is not bad at
   deciding; it is slow at deciding. That is why the hierarchy exists.

## The honesty line — say it before someone finds it

- **No VLA flew the recorded demonstrations.** The city demos are OWL-ViT
  plus a hand-written controller; the SITL rails fly `StubVLA`, which ignores
  rules on purpose so the Shield has something to catch. Real VLAs (OpenVLA-7B,
  AerialVLA) have flown only in simulation, behind the Shield, the last on
  4 Aug 2026.
- **None of the safety numbers are contract KPIs.** The grant measures KPIs in
  the *hil* topology (VLA + Shield on a Jetson Orin). Zero runs are KPI-grade
  today; the campaign runs before 30 Nov. Everything shown is functional-rail
  evidence.
- **The fine-tuned VLA does not yet beat the hand-written controller**
  (0.238 vs 0.963 within 30 m, scored from first acquisition). The ceiling
  flights say that gap is the decision rate, not the training.

---

## Numbers to have in your head

### The follow demo (re-flown 2026-08-11, Project AirSim CityLife)

| arm | detector hit | within 30 m | mean sep | Shield | NFZ / alt |
|---|---|---|---|---|---|
| tracking, car stops twice | 0.993 | **1.000** | 13.4 m | 0 | **0.0 s / 0.0 s** |
| +3 colour-only distractors | 0.979 | **1.000** | 17.7 m | 0 | **0.0 s / 0.0 s** |
| no-fly zone across the route | 1.000 | 0.349 | 38.8 m | 0 | **0.0 s / 0.0 s** |
| **CONTROL, "a red car"** | **0.221** | **0.241** | 71.8 m | 10 | — |

The NFZ row is the correct outcome: the aircraft stops at the fence and loses
the car. Entering the zone would not be.

### Safety (mid-evaluation, 14 Sept — 41 shielded flights: 34 Project AirSim, 4 pymavlink SITL, 3 MAVROS 2)

| measure | Shield on | Shield off |
|---|---|---|
| P0 violation escape rate | **0.0** | 0.63 (5 control flights) |
| P0 escape, 13/14 Oct rails | **0.0** (367 / 388 Shield-flown ticks) | 0.661 / 0.670 |
| Repairs converged | **11,713 / 11,713** across 47 flights | — |

Caveats to volunteer: repair success is the Shield's own re-check (theta
unmeasured on most logs); the 8 Oct Project AirSim follow had 5 range-0
escapes, root-caused and 0 on replay of its 1101 ticks.

### The VLA journey

| step | number |
|---|---|
| AerialVLA upstream: identical "stop and land" | **12 / 12** flights, 108 passes/adapter |
| …yaw for right vs wrong colour word | 0.05 vs 0.08 (words never reached the output) |
| OWL-ViT replacement | 153 M, open vocabulary, 34–56 ms |
| Fine-tune held-out forward step error | 2.47 → **0.36 m** (turn sign agreement 0.838) |
| Fine-tuned AerialVLA flown | best **0.238** within 30 m (from first acquisition) |
| Hand-written follow | **0.963** |
| Expert held to 0.5 s decisions | 0.908 |
| Expert held to 1.0 s decisions (the VLA's rate) | **0.122** |
| Hierarchy (25 Oct decision) | VLA intent ~2 Hz + controller 10 Hz |

### Latency (why the VLA looked slow)

| configuration | capture-to-answer |
|---|---|
| merged bf16 server alone | 235 ms |
| through a flight (renderer sharing the GPU) | 625–713 ms |
| offscreen renderer, 30 fps cap | **370–397 ms** (expert score 0.945) |

### Standing facts

- Interface: **Action4D** (vx, vy, vz, yaw_rate), 10 Hz. One struct, every pilot.
- Tests: 1,400+ fast, 17 with coverage; each test file is its own runner.
- Gate G1 (8 Oct): ArduPilot flew Project AirSim, 3/3 runs, 19.0 Hz to Mission Planner.
- First Orin step (25 Oct): the follow controller ran on the Jetson against the desktop's simulator.
- Grant deadline: **30 November 2026.**

---

## Timing the talk (15–20 minutes)

| # | slide | time | running |
|---|---|---|---|
| 1 | Cover | 0:30 | 0:30 |
| 2 | The Problem | 1:15 | 1:45 |
| 3 | One Pipeline, Four Work Packages | 1:30 | 3:15 |
| 4 | Who Has Flown Through the Shield | 1:30 | 4:45 |
| 5 | Following a Named Object | 1:45 | 6:30 |
| 6 | The Shield Holds | 1:45 | 8:15 |
| 7 | Words That Never Reached the Output | 1:30 | 9:45 |
| 8 | AerialVLA Returns — and Its Ceiling | 2:00 | 11:45 |
| 9 | Findings, Volunteered | 1:30 | 13:15 |
| 10 | dev · hil · flight | 1:15 | 14:30 |
| 11 | Remaining Work, Ordered | 1:15 | 15:45 |
| 12 | Conclusion | 0:45 | 16:30 |
| 13 | Thank You | — | ~16:30 |

That lands at ~16–17 minutes with no question buffer. If you are running
late, compress slides 3 and 10; never cut 5, 6 or 8 — they carry the result.

## What to say on each slide (one line each)

1. **Cover** — one sentence: what the project is, who funded it.
2. **The Problem** — the model answers a velocity command; nothing in it
   knows what a no-fly zone is; the guardrail sits in between. Point at
   Action4D: the interface is why any of the rest is possible.
3. **Pipeline** — policy in YAML, signed into a bundle, compiled into the
   VLA's prompt, enforced by the Shield on every tick. Four work packages,
   all delivered.
4. **Who flew** — read the table, then land the honesty line: no VLA flew
   the recorded demos; the stub is rule-ignorant on purpose.
5. **Follow demo** — it follows, it stops when the car stops, it survives
   distractors, it refuses the zone. Then the control: one word and it
   fails to acquire at all.
6. **Safety** — 0.0 against 0.63–0.67. Then the caveat before anyone else
   says it: dev evidence, not contract KPIs.
7. **The finding** — 12 identical outputs; the language never reached the
   action. We replaced the pilot, not the goal, and changed zero lines of
   the Shield.
8. **Fine-tune + ceiling** — the fine-tune worked (2.47 → 0.36 m) but flying
   at the VLA's rate caps anyone at 0.122. So: VLA sets intent, controller
   flies. This is the current architecture.
9. **Findings** — each card is a defect we found ourselves and a claim we
   retracted. "A check that cannot fail is not a check."
10. **Topologies** — everything so far is dev; hil on the Orin is the KPI
    configuration, and the first Orin step is done.
11. **Remaining** — KPI campaign, perception on MAVROS 2, escalation hold at
    99 %, then the drone. Ordered by contribution to acceptance.
12. **Conclusion** — the pilot changed completely; the safety layer did not
    have to.

---

## Questions you will actually get

**"So did the VLA fly the drone or not?"**
> In simulation, yes, behind the Shield: OpenVLA-7B at about 0.9 s an
> inference, and eight AerialVLA flights on 4 August, one decision every
> 7–9 s. The recorded demonstrations are OWL-ViT plus a hand-written
> controller. Since 23 October a fine-tuned AerialVLA flies the follow —
> best 0.238 within 30 m — and from 25 October it sets the intent in a
> hierarchy while the controller flies at 10 Hz. No VLA has flown hardware.

**"Is 0.0 the contract KPI?"**
> No, and we say so on the slide. The grant takes KPIs from the hil topology
> — VLA and Shield on a Jetson Orin. Those runs happen before 30 November.
> The 0.0 is dev-rail evidence across 41 shielded flights.

**"Why did you drop AerialVLA in August?"**
> Because its language input never reached its output. 108 controlled passes
> per adapter on real frames: 12 of 12 flights produced an identical
> stop-and-land, and the yaw was 0.05 for the correct colour word versus 0.08
> for the wrong one. A 7 B model that ignores its instruction was replaced by
> a 153 M detector that obeys it.

**"Then why bring it back?"**
> Because the follow could be cloned: fine-tuning on the expert's own flights
> took held-out step error from 2.47 m to 0.36 m. It flies worse than the
> expert only because it decides 0.4–0.8 s apart, and the ceiling flights
> prove the expert is equally bad at that rate — 0.122. Hence the hierarchy.

**"What does the Shield actually do each tick?"**
> Forecasts the trajectory, checks every rule on the forecast, repairs the
> action if a repair exists inside the kinematic caps, refuses — brake,
> loiter, RTL — if it does not, and writes an audit record either way. Repair
> success: 11,713 of 11,713 attempts converged.

**"What is StubVLA?"**
> A deliberately rule-ignorant stand-in that flies straight at the target.
> The Shield has to be exercised by a pilot that breaks rules, or "no
> violation" is indistinguishable from "no rule".

**"What was the hardest bug?"**
> The stand-off rule that never armed: the policy named class `pedestrian`,
> the detector produced `person`, and the bind compared exact strings. Hashed,
> audited, completely inert. That finding is why every claim now has a
> measured control behind it.

**"What happens between now and 30 November?"**
> The KPI campaign on the Orin in the hil topology — the stack is defined for
> JetPack 6.2 and the follow controller already ran on the Jetson against the
> desktop's simulator on 25 October; perception over MAVROS 2; the escalation
> hold verified at the 99 % fail-safe target. Then the drone.

**"Would you do anything differently?"**
> Measure the null of every metric from the start. Half the findings — the
> inert rule, the metric a constant could pass — would have been caught a
> month earlier by asking what a do-nothing system scores.
