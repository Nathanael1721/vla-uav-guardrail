# The Paraphraser - free-form wordings, frozen, validated, and logged by id

**Date:** 2026-10-06 (built, then reworked the same day after an independent review)
**Status:** built as an in-process module with a CLI. `guardrail/paraphraser.py`,
`experiments/paraphrases/` (14 sets, 112 paraphrases, plus `probes.json` and
`validation_report.json`), `tests/test_paraphraser.py` (82/82). No pilot reads these
paraphrases in flight yet, so the per-paraphrase robustness KPI is still unmeasured
(see "What this does not give you").
**Cards:** WP2-13, WP4-10, ARCH-23. The Q3 window for this deliverable closes 2026-10-31.

## What the grant asks for

Three sentences in the grant define this deliverable. Nothing else in it describes
the paraphraser.

| Where | What it says |
|---|---|
| Prefix Compiler PDF p5, "Templated NL adapter" | "Always-emitted, templated, never free-form. Free-form paraphrasing is the paraphraser's job (Q3 deliverable, separate service); the Prefix Compiler emits the canonical templated form so the paraphraser has a clean source." |
| Overview PDF p3 (Q3 row); Grant overview PDF p3 (Gantt) | "paraphraser deliverable", Q3 (Aug-Oct); the Gantt bar starts 2026-08-15 and runs 60 days |
| Grant overview PDF p2, WP table | WP4 owns "Mission success rate; per-paraphrase robustness" |

The PI's reference design (`kuanting-vla-uav-guardrail/docs/03-simulation/data-flow.md:12,43,57`)
places the "Paraphraser service" in the control plane, feeding the orchestrator over
gRPC+mTLS. The orchestrator "issue[s] tasks (and paraphrased variants) to the VLA".
Its recommendation (`docs/05-recommendation/recommendation.md:44`) pairs the
"Paraphraser service + Scenario YAML schema" as the Q3 item "required for the
per-paraphrase robustness KPI rollup".

**What this build is, against that.** It is a separate *module* with its own store,
validator, CLI and tests, fed only from the compiler's templated text. It is **not**
a separate *service*: there is no process boundary, no RPC and no mTLS. Any runner
imports it. Whether that counts as the grant's "separate service" is a question for
the PI.

The grant does not say how many paraphrases to make, how to seed the choice, or
what to log. Where it is silent, this design copies the Stress Testing contract
the grant already sets. A run must be reproducible from its `random_seed`
(Stress Testing PDF p1, "Determinism"), so the paraphrase a trial used is a pure
function of (source text, seed, backend). The paraphrase also carries an id that
the trial's records can store. K=8, the novelty floor 0.15 and the variety floor
0.40 are this design's choices, not the grant's.

## The shape

```
Prefix Compiler: compile_csp (risk order)  ─┐
                 build_prompt (type order)  ─┼─>  paraphrase(text, n, seed, backend)  ──>  [Paraphrase]
mission text / task_prompt                  ─┤         │                                  paraphrase_id
                                             │         ├─ stored    experiments/paraphrases/*.json   text
                                             │         ├─ template  seeded rule bank (fallback)      backend
                                             │         ├─ llm:<id>  LLMServiceBackend (slot only)    provenance
                                             │         └─ auto      stored, else template - WARNING
                                             │
                                             └─ every candidate goes through validate(source, candidate)
                                                before it is returned; refusals are logged with the reason
```

* **`paraphrase_id`** = `"pp-" + sha256(source, backend, index, paraphrase text)[:16]`.
  `index` is the paraphrase's position in the backend's own set for that source,
  not its position in one draw, so a paraphrase keeps its id whatever seed or `n`
  selected it. **The text is in the key.** Until the review it was not, and ten
  re-authored texts kept their old ids; a per-paraphrase KPI grouped by id would
  then have merged trials of two different wordings. No trial had been logged under
  the old scheme, so the ids were re-derived (the test pins the new scheme).
* **Determinism.** A draw is `random.Random("guardrail.paraphraser|seed|source_id|backend")`.
  Python seeds a string with SHA-512, so the draw does not depend on
  `PYTHONHASHSEED`. Ten seeds give more than one draw (tested). `n=K` returns the
  whole set in a seeded order.
* **Never short, never padded, never silent.** Asking for more paraphrases than
  exist raises an error. So does a backend that produces fewer valid ones than
  requested, or one that returns the same text twice. `auto` logs a WARNING when it
  falls back from the stored set to the template bank, and the fallback is in each
  paraphrase's provenance.
* **The canonical arm.** `canonical(text)` returns the source itself with
  `backend="identity"`. It is the baseline arm a per-paraphrase robustness number
  is measured against. It is never counted as a paraphrase: the validator refuses
  it as one.
* **What a flight records.** `Paraphrase.to_record()` gives `paraphrase_id`,
  `source_id`, `backend`, `index`, `seed`, `text`, `text_sha256` and `source_sha256`.
  `manifest_extras()` gives flat keys for `metrics.json` or a trial row. The manifest
  itself stays at the grant's six fields.

## Which source text: compile_csp and build_prompt

The compiler renders the same rules two ways. `build_prompt` (read by
`demo/run_demo.py` and the SITL nodes) lists one sentence per rule in rule-TYPE
order. `compile_csp`, the grant's Prefix Compiler ("policy + mission + clock -> CSP",
PDF p3) and what `demo/real_vla_demo.py` shows OpenVLA, lists the same sentences in
RISK order. That order moves with the mission, and with a clock a rule that is out
of force is left out altogether.

The first build keyed the stored sets on `build_prompt`'s text only. The review
found that `stored` then raised `UnknownSource` for all six CSPs as `compile_csp`
emits them, and `auto` quietly served template text in their place. Now:

* the stored CSP sets are keyed on `compile_csp`'s rendering;
* a rule text is looked up by its exact words or, failing that, by its **rule set**
  (`rule_set_key`: the sorted sentences). The rules are a conjunction, so the same
  sentences in another order are the same rules, and one set serves both renderings
  under the same ids. Mission texts are never matched this way: their sentences are
  steps;
* the coverage test renders every scenario cell (`experiments/scenarios.yaml`
  expanded through `guardrail.scenario_spec.Library`, rotations and sweeps
  included) with no clock and, when the cell has one, with its `clock_start`, plus
  `build_prompt`'s rendering, the three city policies and the mission texts. Each
  must be served. `python -m guardrail.paraphraser stale` prints which set serves
  each text and where it is used, computed from the current files.

Which of the two renderings is canonical is the prefix-compiler unit's decision.
This module serves both.

## The stored set (the deliverable)

14 sets of K=8, written by an LLM (this agent), with provenance:

| Set | Source |
|---|---|
| `csp_sim_demo_policy` | compiler NL for `policies/sim_demo_policy.yaml` |
| `csp_corridor_survey` | compiler NL for `policies/corridor_survey.yaml`, including "(in force Mon-Fri 07:30-17:30)" |
| `csp_corridor_survey_out_of_hours` | the same policy compiled with a clock outside the window: the school zone is left out (new) |
| `csp_sitl_pedestrian` | compiler NL for `policies/sitl_pedestrian.yaml` |
| `csp_wgs84_taipei` | compiler NL for `policies/wgs84_taipei.yaml`, which `scenarios.yaml` gained on 2026-10-06 (new) |
| `csp_follow_car_citylife` | compiler NL for the city car policy |
| `csp_follow_pedestrian` | compiler NL for the city person policy |
| `csp_follow_car_citylife_nfz` | compiler NL for the CityLife no-fly-zone policy |
| `task_follow_a_person` | "follow a person" (`run_citylife_follow.ps1`, `-Object "a person"`) |
| `task_follow_a_red_car` | "follow a red car" (`-Object "a red car"`) |
| `task_fly_to_the_waypoint_120_m_ahead_at` | "Fly to the waypoint 120 m ahead at cruise altitude." (`dyn-nfz-spawn-ahead` `mission.task_prompt`) |
| `task_fly_to_northeast_pad` | "fly to the northeast pad at 6 m/s" (the KPI rail's mission, `demo/run_demo.py`) |
| `task_fly_forward_avoid_restricted` | "fly forward and avoid restricted areas" (`demo/real_vla_demo.py` - OpenVLA, the one model here that reads text) |
| `task_fly_to_40_40` | "fly to (40, 40) at 6 m/s altitude 20" (`demo/aerialvla_demo.py --command`, parsed by the compiler) |

Where each one is used today is not frozen into the files (it went stale within a
day the first time); `stale` computes it.

Each set is varied along the axes the commissioning task named: vocabulary, word
order, politeness, verbosity and indirect phrasing, with a `style` tag per item. A
rule paraphrase is one or more full sentences. The task sets are half `verb_phrase`
and half `utterance`. A `verb_phrase` reads correctly inside OpenVLA's fixed frame,
`In: What action should the robot take to {instruction}?` (`demo/real_vla_demo.py:124`).
An `utterance` ("Could you shadow a red car for me?") placed in that frame is
ungrammatical, so feeding one to OpenVLA tests two things at once. A robustness study
should filter on `form` or report the confound.

**Provenance.** Every file carries `generator: "claude-opus-5-5 (agent, 2026-10-06)"`,
the date, the exact generation prompt (`GENERATION_PROMPT`, which is also the prompt
an LLM-service run would use) and the commissioning instruction verbatim. The six
CSP sets re-keyed to `compile_csp`'s order carry `generated_from` (the
`build_prompt` text the prompt named) and a `rekeyed` note; the texts were not
re-generated. A re-authored item carries a `revision` note saying why.

**Honesty about authoring.** The paraphrases, the validator and the mutation
generator were written by the same kind of agent, in one day, validator-aware.
"112/112 accepted" is therefore weak evidence: it shows the store agrees with the
validator, not that the store is faithful. What the review and the rework found:

* **Before the review** (first build): one correct refusal ("find" added an
  action); two rewrites for variety; 30 over-refusals fixed in the validator; seven
  stand-off subjects narrowed from "anything you are following" to a class, which the
  validator missed until review. Ten texts rewritten in all.
* **After the review:** six more re-authored, each with a `revision` note.
  `csp_corridor_survey[6]` was the serious one: "if you notice you are heading
  toward it ... turn away. Otherwise, remain in corridor ..." made the corridor, the
  band and all three kinematic limits conditional, and the validator accepted it.
  The others: "don't go anywhere near" (stricter than "never enter"), two "you'll
  want to" (softened), "5 m back from" (a fixed distance, not a minimum), and
  "chase it" for "follow".
* **Still owed:** a person other than the author should read all 112 before the PI
  signs off on the set.

## The validator

`validate(source, candidate)` extracts a fixed list of slot kinds and refuses a
candidate in which one was dropped, changed or added:

| Slot | Example of a refusal |
|---|---|
| number + unit | 4 m/s → 5 m/s, "six" → "seven metres per second", m/s → km/h, m → ft |
| bound | "at least 10 m" → "at most 10 m"; read through every negation in the clause by parity ("should not fly at an altitude no lower than 6 m" is a ceiling) |
| a ruling after the number | "a speed below 4 m/s is not allowed" (the cap became a floor); "an altitude between 10 m and 20 m is forbidden" |
| what a number limits | speed and climb swapped; "max vertical speed 4 m/s" for "speed at or below 4 m/s" (kinematic attributes must now be equal, not merely included) |
| named zone, corridor, place, coordinate | 'nfz-square' dropped or renamed; "north-east pad"; (40, 40) → (40, 30) |
| place role | "past the northeast pad", "a spot far away from the northeast pad" |
| zone polarity | keep-out → stay-inside; "It is not forbidden to enter zone 'nfz-square'"; "do not avoid restricted areas"; "inside or near corridor ..." |
| time window | "(in force Mon-Fri 07:30-17:30)" dropped, moved to another rule, or inverted with "except" |
| target object and colour | red → blue; car → truck; car → "vehicle"; an object added ("and a person") |
| where the colour sits | "the car next to the red one"; "a red car, or any car" |
| stand-off subject | "anything you are following" → "the person / car you are following" |
| direction, action, named reference - and their negation | forward → backward; follow → find; "do not follow", "stop following", "do not fly forward", "do not hold cruise altitude" |
| modality | "try to", "ideally", "generally", "you'll want to"; a condition, exception or scope limit ("unless", "if", "otherwise", "while climbing,", "near the zone only"); a suspension ("these limits are optional", "ignore ...") |
| the null | identity and near-identity: word-level edit distance below 0.15 |

Polarity is read clause by clause. A ruling word ("avoid", "off-limits",
"prohibited", "allowed", "may") decides when there is one, and a negator flips the
first ruling word after it. Otherwise the entry words decide ("never enter", "do not
fly into"). A second clause that points back at the zone ("..., so never go in
there") counts too. If the source's own ruling cannot be read, the candidate is
refused rather than passed unchecked.

A condition that only restates the rule's own window ("while it is in force",
"until 17:30") is allowed when the source has a window. "Only inside" strengthens a
stay-inside rule, "while climbing at under 2 m/s" coordinates two limits, and "until
you reach the pad" is a goal. All three are allowed.

It also refuses to touch AerialVLA's prompt. See the section after next.

## What was measured, and what each number does not mean

`python -m guardrail.paraphraser validate` writes
`experiments/paraphrases/validation_report.json`. A test recomputes it and checks it
field by field against the current store, validator, probe file and place registry.

| | result | before the review fixes | the null beside it |
|---|---|---|---|
| stored paraphrases accepted | 112/112 | 96/96 (one of them unfaithful) | - |
| identity "paraphrase" refused | 14/14 sources | 12/12 | an exact-copy check catches these, but also accepts "Never enter" → "Do not enter", which the novelty floor refuses |
| generated mutants refused | **3979/3979** (24 classes) | 1721/1721 (11 classes) | see below |
| review probes (dev set): meaning changes accepted | **0/28** | 17/28 | - |
| review probes: faithful paraphrases refused | **3/30** | 13/30 | - |
| held-out probes: meaning changes accepted | **5/30** | 11/30 | - |
| held-out probes: faithful paraphrases refused | **2/30** | 2/30 | - |

**The nulls, scored on both halves.** A null that refused everything would "catch"
every mutant, so each null is scored on the stored paraphrases too:

| null validator (compared with the source, as `validate` is) | stored accepted | mutants refused |
|---|---|---|
| accept anything but an exact copy | 112/112 | 0/3979 |
| digits multiset unchanged | 110/112 | 816/3979 |
| sets of digits, units, ids and about 30 slot words unchanged | 51/112 | 3472/3979 |

The word-bag null catches 87 % of the mutants, but only by refusing 54 % of the
faithful paraphrases. The validator does both halves.

**What the 3979/3979 means.** It means recall on the 24 mutation classes `mutants()`
generates, and nothing more. The first build reported 1721/1721 while the review's
own probes found 17 of 28 meaning changes accepted. Those changes were negations
added, permissions, rulings after a number, conditions, hedges and colour moves,
and the generator had no mutation of any of those kinds. Thirteen classes now cover
them (`negate-verb`, `negate-modal`, `post-prohibit`, `permission`, `exception`,
`conditional`, `hedge`, `phase`, `override`, `vertical`, `colour-rebind`, `widen`,
`place-role`). On their first run they found 12 survivors in the store, and the
re-authored and new items then showed 5 more (from old and new classes alike).
Each was fixed in the validator. A class nobody wrote is still not measured.

**What the probe numbers mean.** `probes.json` holds two sets.

* **Review set (dev).** 58 probes written by the independent reviewer. The validator
  was tuned on them, so 0/28 is a regression floor, not an estimate. Three of the
  reviewer's "faithful" probes stay refused. "follow someone walking" for "follow a
  person" is refused on purpose: walking narrows the class. "fly to the pad in the
  northeast" is refused because the place registry knows "northeast pad" and not
  the paraphrase. "go to the point x=40, y=40" is refused because only "(x, y)" and
  "to x, y" are read as coordinates. The labels are kept as the reviewer wrote them.
* **Held-out set.** 60 probes written after reading the review and before changing
  any code, and never used to tune it. It was measured once before (11/30, 2/30) and
  once after (5/30, 2/30). This is the closest thing here to an honest
  false-accept rate. It comes from the same author as the code, so it is not
  independent of the code's blind spots. The five meaning changes that still pass
  are all narrowings that no slot sees: "a dark red car", "the person who is
  running", "descend under 2 m/s" for "climb rate below 2 m/s" (the policy caps
  climb and descent), "pedestrians in the street" and "adult pedestrians". The two
  faithful paraphrases still refused are "stick with a red car" (not in the follow
  synonyms) and a corridor text whose band follows a semicolon.

**Silent defects found while this was built.** None crashed; each made the
validator accept a changed slot. Before the review: "at or below" split as a list;
an after-number cue read only in first position; a band read from outside; "a
distance no lower than" read as altitude; an attribute leaking across "or".
After it, by the new mutation classes:

* an entry word list that included "in" turned the compiler's own "(in force ...)"
  into a permission. The SOURCE zone lost its polarity, every polarity check
  against it was skipped, and two flipped mutants passed. Unreadable source
  polarity is now a refusal;
* a five-token negation window saw only the nearer of two negators;
* "behind" before "and" was read as "behind a" (the alternation matched the "a"
  of "and"), so "fly straight behind" passed for "fly forward";
* a "while climbing," that sat before a window was taken for a restatement of the
  window;
* a clause that points back at the zone ("..., so go in there") was not read at all.

Each has a test.

**Limits, stated so nobody over-reads any of the numbers above.** The validator is a
lexical guard, not an entailment model:

* It only sees the slot kinds in the table. A narrowing by an adjective or a
  relative clause passes (the five held-out false accepts).
* Synonyms are a closed table. For example car ≡ automobile but car ≠ vehicle,
  "stick with" is not a follow verb, and "whichever car you are following" is still
  read as the wildcard (a missed narrowing).
* A double negation across clauses, a "respectively" list, or a pronoun whose zone
  is in another sentence can be misread. The validator refuses the candidate in the
  first case and accepts it in the worst case.
* A bare metres band in a sentence with a stay-inside corridor is read as that
  corridor's altitude band. This is domain knowledge taken from
  `templates/sentence/corridor.j2`.

Widening any of these is a decision about what counts as the same mission, and
belongs in this document.

## AerialVLA's fine-tuned phrases are never paraphrased

`demo/vla_bridge.py:30-32` says: "A phrase the LoRA never saw in training is worth
nothing, so the strings are not paraphrased and not reformatted".
`tests/test_vla_bridge.py:11-15` adds: "A paraphrase costs everything - the
adapter's response to an unseen phrase is undefined".

The reason: AerialVLA has exactly one prompt slot that measurably steers it, the
seven-way `{direction}` compass phrase. Over 108 controlled forward passes, the
commanded yaw is monotonic in that phrase, from "to your left" at −0.466 rad/s to
"to your right" at +0.161 rad/s. The `{object}` slot is inert
(`docs/FINDING-what-drives-aerialvla.md`). A reworded direction phrase is outside
the vocabulary the LoRA was trained on. Its effect is undefined rather than robust
or fragile, so a "per-paraphrase robustness" number for it would measure nothing.

`paraphrase()` therefore raises `ProtectedTextError` on anything shaped like
AerialVLA's prompt (`<image>` ... `Action:`). The test builds every
direction-vocabulary prompt through `vla_bridge.build_prompt` and checks the
refusal.

## A first, honest preview: what the one text reader makes of the paraphrases

`ConstraintCompiler.parse_command` is how a typed mission reaches the stub pilot
today. It is not the per-paraphrase KPI, but it is the cheapest preview of one. The
report reads every task paraphrase twice: once with the parser's real defaults, and
once with sentinel defaults. That way a value that only matched because the parser
fell back to its default is not counted as read.

| Set | Same mission as the canonical text | Matched only by default |
|---|---|---|
| `task_fly_to_northeast_pad` | 8/8 | 0 |
| `task_fly_to_40_40` | **6/8** | 0 |

The two misses are "at an altitude of 20" and "and an altitude of 20". The
parser's altitude regex allows at most three characters between "altitude" and the
number. It then silently falls back to 15 m, so both paraphrases would fly the
(40, 40) mission 5 m low with no warning. This is the per-paraphrase fragility the
KPI exists to find, in the component that reads text today. The fix belongs in
`guardrail/compiler.py`, which this unit does not own. A test pins the 6/8, so the
day the parser is fixed the test fails and this table must change.

## What this does not give you

* **No per-paraphrase robustness number.** That needs a pilot that reads the text,
  flown once per paraphrase arm against the canonical arm. The scripted stress
  pilots read nothing (`experiments/scenarios.yaml` header). OpenVLA-7B reads text
  but needs camera frames, and the SITL rail has none (WP2-01, WP4-09, PI question 3).
* **No harness hook.** No runner issues one arm per episode or writes
  `manifest_extras()` into a trial row yet. The sweep and the KPI module are not this
  unit's files.
* **No service boundary.** The module is in-process; the reference design's
  Paraphraser is a control-plane service over gRPC+mTLS.
* **No live LLM service.** `LLMServiceBackend` is the slot. `LLMClient` is a
  protocol with `model_id` and `complete(prompt, n=, seed=)`. With no client the
  backend raises `BackendUnavailable` rather than inventing text. A live service is
  only reproducible if it honours its seed, so the intended path is: generate once,
  validate, freeze with `write_store`, then serve with the `stored` backend.
* **No guarantee of faithfulness.** See the held-out rate above, and the human read
  of the 112 that is still owed.
* **Whether the PI accepts this as the Q3 deliverable.** The stored set is
  free-form and LLM-written. The template backend is the fallback. The question
  from the plan ("is a seeded template paraphraser enough, or must it be an LLM
  service?") has not been asked yet.

## Use

```python
from guardrail.paraphraser import paraphrase, canonical

arms = [canonical(text)] + paraphrase(text, n=8, seed=run_seed, backend="stored")
for arm in arms:
    fly(instruction=arm.text, extras=arm.manifest_extras())
```

```
python -m guardrail.paraphraser validate     # store -> validation_report.json
python -m guardrail.paraphraser show "fly to the east pad at 3 m/s" --n 3 --seed 0
python -m guardrail.paraphraser stale        # every in-scope text and the set that serves it
python tests/test_paraphraser.py             # 82/82
```

If the compiler's templated NL changes, or `scenarios.yaml` gains a policy, a clock
or a `mission.task_prompt`, `test_every_instruction_in_scope_has_a_stored_set` fails
loudly and names the missing text. The fix is to write and freeze a new set with
`write_store`, not to edit stored text: the store refuses an edited text through its
hash and its id.
