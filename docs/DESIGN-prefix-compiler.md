# Prefix Compiler: the typed CSP, its budget and its four KPIs

**Status:** 2026-10-06. Built and tested offline, then revised after review
(the corridor altitude band, the wall-clock issue stamp, and the untested
halves listed under "Review fixes"). `demo/real_vla_demo.py --csp on` (another
unit, WP2-01) now injects the typed CSP's text into OpenVLA's prompt; no flight
with it was run in this unit.
**Cards:** WP2-02, WP2-03, WP2-04, WP2-05, WP2-06, WP2-07 (action-set part),
WP2-08, WP2-14, WP2-16, and the umbrella WP2-18.
**Grant:** *Prefix Compiler* page (`reference/Prefix Compiler - ….pdf`, cited as
PDF pN below) and `kuanting-vla-uav-guardrail/docs/02-implementation/prefix-compiler.md`.
**Code:** `guardrail/csp.py` (the record), `guardrail/compiler.py` (the pipeline),
`guardrail/templates/` (the wording), `tests/test_csp.py` (65 tests).

---

## What was wrong

The 2026-10-05 audit found the compiler unchanged since 1 September and well
short of the page it implements:

- `summary_pack()` took no arguments and returned a plain dict. None of the
  grant's fourteen CSP field names were in it (WP2-04, DRIFT).
- It filtered nothing, scored nothing and could not be cut to a budget. Its
  docstring said "Nothing is summarised away", the opposite of the spec (WP2-05).
- **It told the model that a weekday-only rule was permanent.** `nfz-school` in
  `policies/corridor_survey.yaml` is in force Mon-Fri 07:30-17:30, and the
  prompt said `Never enter zone 'nfz-school'.` at all times (WP2-05, WP2-08).
- There was no token budget, no `CSPBudgetExceeded`, no relevance explanation
  and no action set, so none of the four locked KPIs (PDF p5) had a test
  (WP2-02, WP2-06, WP2-07, WP2-14).

## What exists now

```python
csp = ConstraintCompiler(policy).compile_csp(mission, lookahead_s=5.0, now=clock)
csp.natural_language_prompt     # what a language model is told
csp.P0_constraints              # every in-scope P0 rule, with a geometry_ref
csp.allowed_action_set          # vx/vy/vz ranges, yaw cap, altitude band (body frame)
csp.relevance_explanations      # rule id -> a fact about THIS mission
csp.selection.rules             # one row per policy rule: kept / dropped / filtered, and why
```

The pipeline is the one on PDF p3:

| Step | Where | What it does |
|---|---|---|
| Temporal filter | `_activity` | Keeps a rule if it is in force at any instant of `[now, now + lookahead_s]`. |
| Spatial filter | `_in_region` | Sets aside keep-out zones outside the path's bounding box widened by 200 m plus one lookahead of travel. |
| Risk grade | `_severity`, `_proximity` | `0.5·severity + 0.3·proximity + 0.2·time-criticality`; the weights are configurable (`csp.RiskWeights`). |
| Order + truncate | `compile_csp` | Keeps every P0 rule. P1/P2 rules fill the budget by descending risk. If P0 alone is over the budget, it raises `CSPBudgetExceeded`. |
| Render | `templates/`, `_action_sets`, `multiscale_geometry` | Builds the sentences, the summaries, the reasons, the action sets and the geometry refs. |

Other entry points:

- `summary_pack(mission, …)`, `write_csp(path, mission, …)`;
- `action_set_adapter(csp)`, the spec's flat dict;
- `action_violations(action, csp, state, policy)`, which checks a candidate
  `Action4D` against the CSP;
- `p0_coverage(csp, policy)`, the locked KPI, and `rule_coverage(csp, policy)`,
  the same measure over every rule and priority, split into what the model
  reads (text) and what the record carries (recorded);
- `coverage_report(policies_dir, budget_tokens)`, every KPI number over a
  policy folder with its measured null, its no-skill baseline
  (`naive_prefix_baseline`), the policy hashes, the code revision and the
  command that reproduces it;
- `resolve_geometry_ref(policy, ref)`;
- `csp.write_csp_schema(path)`, `csp.csp_hash(csp)` (one issued CSP) and
  `csp.csp_content_hash(csp)` (the content, without the issue stamp);
- a CLI: `python -m guardrail.compiler {report,schema,table}`.
  `report --json <file>` writes the whole `coverage_report` to a file.

## Decisions, each with its reason

### 1. The record copies the fourteen locked fields and adds two

`csp.CSP` carries the spec's fields by name, type and optionality, in the
spec's order (`csp.LOCKED_FIELDS`, which a test checks). The spec leaves
`P0Entry`, `AllowedActionSet` and `ForbiddenActionSet` as forward references
with one worked example each; their fields here are the example's fields, plus
named additions:

- `P0Entry.in_force`: the rule's schedule in words.
- `AllowedActionSet.horizontal_speed_max_mps` and `source_rules`.
- `ForbiddenActionSet.no_translate_out_of_corridors`: the spec's example has no
  keep-in rule; this DSL does.

The CSP gets two top-level fields:

- `policy_id`.
- `selection`: one row per policy rule, with the decision, the three risk
  terms, the sentence, its token count and the reason. The spec allows P1/P2
  rules to be cut. A cut with no record is a silent zero (CONTRIBUTING.md), so
  every cut is listed with its reason.

Every model is `extra="forbid"`, so a misspelt field is refused rather than
dropped.

### 2. The budget is 256 tokens of `natural_language_prompt`, configurable

The spec leaves the number open ("1k? 2k?", PDF p6). The target model is
OpenVLA-7B. Its `llm_max_length` is 2048. The image costs 256 tokens (224 px at
patch 14 gives 16 × 16), and the prompt template, the instruction and the 7
action tokens cost roughly 20 more. That leaves about 1.7k tokens as a hard
ceiling. The ceiling is not the useful limit, though. OpenVLA was trained on
short task phrases, and every token put in front of the instruction moves its
input further from that. The 29 shipped policies need 54-106 exact tokens for
all their rules. So 256 cuts nothing today and leaves 2.4× headroom. The budget
used is written into every CSP.

The budget covers the text a language model reads, and nothing else. The
structured parts (P0 entries, action sets) are a few numbers each and are never
cut. As a result, a P1 speed cap that is dropped from the prose for space still
bounds `allowed_action_set`.

### 3. The default token counter is an upper bound, identical on every machine

There are two counters:

- `OpenVLATokenCounter` is exact. It needs `tokenizers` and
  `D:/models/openvla-7b/tokenizer.json`, so it exists only in the vla-real env
  on the lab machine.
- `TableTokenCounter` is the default. It is pure Python.

**Why it is an upper bound.**

1. The Llama-2 tokenizer turns each single space into a word-start marker and
   never merges across one. The count of a text is therefore the sum of its
   space-separated chunks. The test checks this identity on 384 rendered texts.
2. A chunk that appears in the measured table counts its exact value.
3. Any other chunk counts its UTF-8 bytes plus one. Byte fallback can never
   need more than that.

Measured on 2026-10-06: the table counter gives 1.00-1.24× the exact count on
whole prompts, and at most 2.0× on any single text.

**Why it is the default.** An exact-when-available default would make the CSP
depend on the machine. The same policy and mission would compile differently on
SITL (WSL, with no tokenizer) and on AirSim (vla-real). The counter's name is
recorded in every CSP. You can pass the exact counter explicitly.

### 4. Filtering never switches a P0 rule off by accident

**Time.**

- The filter calls each rule's own `active_at`, the function the Shield uses,
  so the two cannot disagree.
- It samples `now` and every minute boundary up to `now + lookahead_s`. A
  `Recurrence` can only change state at those instants.
- With no clock (`now=None`), every rule is treated as in force. This follows
  `models.ConstraintBase.active_at`: "absent means active".

**Space.**

- Only keep-out zones are ever filtered.
- A keep-in corridor binds everywhere, because outside it is forbidden.
  Filtering it as "far away" would hide exactly the case where the mission is
  outside it, so corridors are never filtered (a test checks this).
- Envelopes and stand-offs have no location.
- The region test is a bounding-box overlap. It can keep more zones than an
  exact test would, never fewer.
- `region_margin_m=None` switches the region filter off.

The CSP is advice to the model. The Shield enforces every rule in the policy,
whether or not the CSP mentions it.

### 5. Risk grading follows PDF p4 for weights and order; the numbers inside are ours

**Severity.** The grant gives an order: "hard ≫ soft, P0 > P1 > P2". The values
here are P0 1.0, P1 0.6 and P2 0.3, and soft rules are halved.

**Proximity.**

- For a keep-out zone, proximity is 1 on the path and falls linearly to 0 at
  the region's edge. Distance is measured to the zone's margin ring, the
  boundary the Shield enforces. The measurement is exact: distance to a buffered
  polygon equals distance to the polygon minus the margin.
- Every other rule binds wherever the vehicle is, so its proximity is 1.

**Time-criticality** is 1 if the rule switches on or off inside the lookahead,
otherwise 0, as the spec words it.

Scores are rounded to 4 decimals before ordering, so the published scores are
the ones the order used. Ties break on rule id.

### 6. Truncation is a strict prefix, and P0 overflow raises

Non-P0 rules are added in risk order until one does not fit. After that point,
no lower-risk rule is admitted, even one short enough to fit. This gives a
checkable property: every kept P1/P2 rule outranks every dropped one. A test
builds the case where the two readings disagree (a long, higher-risk speed cap
that does not fit and a short, lower-risk zone that would) and requires the
short one to be dropped; skip-and-continue fails it.

`CSPBudgetExceeded` is raised when the P0 sentences alone exceed the budget. The
test checks the exact boundary: budget = P0 tokens passes, and one token less
raises. The exception is deliberately not a `ValueError`, so a generic
`except ValueError` around policy loading cannot swallow it.

### 7. Every rule in the CSP is explained from mission facts

Each rule type has a reason template, filled from facts computed at compile
time:

| Rule type | Facts in the reason |
|---|---|
| Keep-out zone | Distance from the path and from the target, whether the path crosses it, cruise altitude against its band. |
| Corridor | Whether the start and the target are inside it. |
| Altitude envelope | Requested altitude: inside, below or above the band. |
| Kinematic envelope | Requested speed against the cap. |
| Obstacle clearance | Path length; buildings come from the runtime map. |
| Stand-off | Whether the task text asks to follow anything. |

Scheduled rules add their time fact: in force now, switches on within the
lookahead, or no clock given.

Rules dropped for budget are still in the CSP's structured part, so they are
explained too.

A constant string would satisfy "populated for every rule". The tests therefore
also require the explanation to change when the mission changes, for all six
rule types: zone distance, altitude and speed against the caps, corridor start
inside or outside (and cruise altitude against its band), path length for
clearance, and "follow" against "fly to" for the stand-off. The first version
tested three of the six; an explainer that always said "start inside" or
"asks to follow" passed it.

### 8. The wording is Jinja2, and the old wording is preserved exactly

There is one template per rule type in each of three folders:
`templates/sentence/`, `templates/summary/` and `templates/reason/`. The
environment uses:

- `StrictUndefined`: a missing variable raises instead of printing
  "Keep at least  m away".
- `autoescape=False`: this is plain text, not HTML.

**Backward compatibility.** Every HARD rule without a time window renders
byte for byte what the 2026-09-01 f-strings did. A test checks this for all 132
such sentences across the 29 fixtures, against a verbatim copy of the old code.
Two kinds of rule get a qualifier before the full stop:

- a rule with a window renders the old sentence plus its schedule:
  `Never enter zone 'nfz-school' (in force Mon-Fri 07:30-17:30).`
- a SOFT rule gains ` (soft limit)`: `Never enter zone 'z' (soft limit).` The
  old text gave a soft rule the same "Never" as a hard one. No shipped fixture
  has a soft rule, so this changed `build_prompt` for none of them; it does
  change it for any soft-rule policy, and a test pins the wording so the
  change stays deliberate. (The first version of this note claimed "every
  rule without a window" was byte-identical; that was true only because no
  fixture is soft.)

Overnight windows are marked as such ("22:00-06:00 overnight"), so that they
cannot be read as empty.

The audit (WP2-03) also noted that the sentences leave out margins and
`violation_action`. That is kept on purpose. The sentences stay as they were so
that old flights reproduce. `violation_action` is in each `P0Entry`. The
margin is not in the text. Instead, every distance in a relevance explanation
is measured to the margin ring, the boundary the Shield enforces.

`build_prompt` keeps all its keys. On the 29 fixtures its
`natural_language_prompt` changed only for the two windowed policies,
`corridor_survey` and `wgs84_taipei`. Each grew by 19 exact tokens, from 78 to
97 and from 58 to 77.

**Priority** is shown structurally: the P0 sentences come first, and P1/P2
appear in the summary fields. A `[P0]` tag on each sentence is available as
`show_priority=True`. It is off by default because the spec's worked prompt
(PDF p2) has no tags, and OpenVLA has never seen any.

### 9. Multi-scale geometry is computed in the compiler: a deviation

PDF p1 locks: "the policy bundle's IR already carries pre-computed multi-scale
polygons … The Prefix Compiler picks the scale, never re-computes." Our IR
(`models.Policy`, and since 2026-10-06 `guardrail/ir.py`) carries no
multi-scale cache; `ir.py` lists that as not done. Those are WP1 files and
were not edited in this unit. Instead, `compiler.multiscale_geometry` builds
the cache once per policy hash and memoises it. It should move into the IR at
ingest; see "Outside this unit".

**Scales.**

- *Fine* is the vertices as authored.
- *Coarse* always contains the fine polygon. It is the convex hull, or the
  minimum-area oriented rectangle when the hull has more than 8 vertices. A
  coarse keep-out zone that cut a corner would tell the model a smaller zone
  than the Shield enforces. A test checks containment with shapely,
  independently of the pure-Python hull.
- Corridors are only ever *fine*. Coarsening a keep-in shape outward would
  widen where the model believes it may fly.

**How the scale is picked.** A zone gets *fine* when its margin ring is within
one lookahead of travel of the path, and *coarse* otherwise.

**References.**

- Rules without geometry get `param:<id>@v…/g…`.
- `resolve_geometry_ref` refuses a ref from another version or generation, so a
  CSP from before a hot-apply cannot resolve against geometry it never
  described.

For all 29 shipped policies, coarse equals fine, because every zone has 3-4
vertices. The 12-gon in the test is where coarsening actually reduces a shape
(12 vertices to 4).

The helpers are pure Python on purpose. `geometry.py` stays the only shapely
importer and holds the Shield's evaluation. Nothing the Shield decides depends
on the compiler's helpers. The one safety-relevant check, whether an action
enters a zone, calls `geometry.point_in_fence`.

### 10. The action set is exact in body frame

The grant locks body frame. Every bound comes from a cap that does not depend
on rotation: a horizontal speed disc, a climb cap and a yaw-rate cap. The
numbers are therefore exact in body frame and in `Action4D`'s world frame
alike, and the frame question (WP2-17) does not block this.

`vx_range_mps` and `vy_range_mps` are the box around the speed disc, as in the
spec's example. The box admits √2 times the cap on a diagonal, so the true cap
is also given separately.

A policy with no kinematic rule gets `None`, never 0. A zero range would read
as "hover only".

`altitude_band_m_agl` is the intersection of every hard altitude envelope and
every hard corridor's floor-ceiling band. The Shield enforces a corridor's band
wherever the vehicle is (`_check_corridor`, and `CorridorAltitudeFix` in
`_repair_corridor`), and it checks each corridor on its own, so all of them
bind at once. The spec's worked example intersects them the same way: a
5-120 m P1 envelope and a 30-80 m corridor give `[30, 80]`. The first version
intersected envelopes only, so `corridor_survey` (a 10-20 m corridor, no
envelope) reported no band, which means "unbounded" by the class's own
definition, while the Shield held 10-20 m. It was the only fixture affected.
Bands that cannot all hold raise `ValueError` naming every rule involved.

`action_violations` maps the CSP onto `Action4D`. The one conversion it does is
the yaw cap, from deg/s to rad/s. Comparing the raw numbers would admit
anything under 45 rad/s, the unit confusion in
`FINDING-the-contract-disagreed-with-itself-about-yaw.md`.

### 11. The cost-map adapter is out of scope, pending the PI

PDF p6 asks whether v1 should ship all three adapters, and notes that a cost
map is "only useful if a path-planner backend joins the system". No path
planner is in this system, so `cost_map_ref` is always `None`. This needs the
PI's written confirmation (plan question PQ4, item 3).

### 12. Stateless given a clock; two hashes

Same policy, mission and clock (`now`) give a byte-identical CSP. `csp_hash`
(full SHA-256) is checked equal across two interpreters with different
`PYTHONHASHSEED`, which catches any set iteration leaking into the output. It
is also checked to change when the policy's generation changes (the null case:
a constant hash would pass the equality test). Nothing in the compiler is
random, so the spec's "rule-selection seed" has nothing to seed.

**Without a clock** the statement needs a qualifier, and the first version of
this note left it out. `issued_at` is the `issued_at` argument if given, else
`now`, and only when neither is given the wall clock; `selection.
issued_at_source` records which (`argument`, `clock`, `wall_clock`). The KPI
report and `demo/real_vla_demo.py` compile with no clock, so their CSPs differed
from run to run in that one field, and `csp_hash` covers it: the review
measured 29/29 report rows changing hash between two runs 1.1 s apart, against
a comment that said two reports "differ only where the code or the policies
do". So there are now two hashes:

- `csp_hash`: the whole CSP. It names one issued CSP, which is what a flight
  log pointing at the file it injected needs.
- `csp_content_hash`: every field except the issue stamp. "Is this the same
  CSP?" The report's rows carry it, and a test runs the report twice under a
  clock that ticks one second per call and requires identical rows (it fails
  on the first version: 29/29 rows moved).

**One convention for the stamp.** The first version wrote naive local time with
a clock (`2026-10-05T08:00:00`) and UTC with `+00:00` without one. Now the
stamp is written the way the clock is given: `now` is the policy's local time,
naive, as `Shield(now=...)` takes it, so the clocked stamp and the wall-clock
fallback are both naive local time. No offset is invented, because deriving it
from the machine's timezone would make the same clock compile differently on
the WSL SITL rail (UTC) and on Windows (+08:00). A caller that wants the spec's
UTC form (`2026-04-28T09:01:12Z`, PDF p2) passes it, or an aware datetime, as
`issued_at`, and it is written as given (deviation 6).

## KPI evidence, each with its null

All numbers measured on 2026-10-06 with `python tests/test_csp.py` and
`python -m guardrail.compiler report --exact` (vla-real env). The fixtures are
the 29 files in `policies/`. The mission flies (0, 0) → (30, 30) at 15 m and
4 m/s.

| KPI (PDF p5) | Result | Null / break test |
|---|---|---|
| Budget configurable and respected | Max **126 / 256** tokens (table counter); **106** exact. `tokens_used` is re-counted from the stored text on every fixture. | On a 7-rule mixed policy, the untruncated prompt is shown to be over the budget before the cut is checked. |
| P0 coverage = 100 % | **105 / 105** P0 rules over 29 fixtures. Strict reading: no clock, no region filter, every P0 rule in scope, each in both `P0_constraints` and the prompt text. | Null: the scorer returns 0 % on an empty CSP, 0 % with only `P0_constraints` emptied, 0 % with only the text emptied, and `None` when no P0 rule exists. **Baseline: a naive cut of the old by-type text also scores 105 / 105 at 256 tokens, so at the default budget this KPI does not discriminate** (see below). |
| `CSPBudgetExceeded`, never silent truncation | Raised at budget = P0 tokens − 1; not raised at budget = P0 tokens. | Every rule left out appears in `selection` with its reason. A test checks that kept + dropped + filtered = every rule, on every fixture, at three clocks. |
| Relevance explained for every rule | **134 / 134** rules explained across 29 fixtures (Monday 08:00). | A constant explainer passes "populated" and fails the mission-dependence test. |
| All-rule text coverage (audit WP2-03, not a locked KPI) | **134 / 134** rules in the text at the default budget. | The same scorer gives 0 on an emptied CSP. A priority with no rule in scope gives `None`. |

`python -m guardrail.compiler report` prints these numbers. Two floors are
printed beside the KPI, because they answer different questions:

- **Null** ("can the scorer say 0 %?"): the same scorer run on each CSP with
  its P0 entries and prompt emptied. It gives **0 / 105**, measured, not
  printed as a constant.
- **Baseline** ("what does no skill score?"): `naive_prefix_baseline`, the old
  by-type sentence list (`build_prompt`'s text) cut as a strict prefix to the
  same budget with the same counter. It has no priority, no risk and no
  `CSPBudgetExceeded`. The report sets `kpi_discriminates` to whether the
  compiler beats it.

If no policy compiles (every one raised `CSPBudgetExceeded`), every share is
`None` and the raised files are listed. A test checks this at a 5-token budget.

**The cut, on the real fixtures.** At the default budget of 256 tokens nothing
is cut, so the report was also run at smaller budgets:

| Budget | Compiled | Raised `CSPBudgetExceeded` | P0 coverage | Baseline P0 (same policies) | Baseline P0 on the raised files | All-rule text coverage | Dropped for budget | Explained |
|---|---|---|---|---|---|---|---|---|
| 256 | 29 | 0 | 105 / 105 | 105 / 105 (does not discriminate) | - | 134 / 134 | 0 | 134 / 134 |
| 96 | 29 | 0 | 105 / 105 | 103 / 105 | - | 124 / 134 (93 %) | 10 | 134 / 134 |
| 64 | 19 | 10 | 54 / 54 | 45 / 54 | 33 / 51, silently | 59 / 73 (81 %) | 14 | 73 / 73 |
| 5 | 0 | 29 | `None` | `None` | 0 / 105, silently | `None` | 0 | 0 / 0 |

So the honest reading of the headline: **at 256 tokens "P0 coverage 100 %"
shows only that every shipped policy fits whole**; a compiler with no skill
scores the same. The priority ordering earns its keep at 96 tokens (2 P0 rules
the naive cut loses) and at 64 (9 more on the compiled files, and on the 10
files where the compiler raises, the naive cut would have carried 33 of 51 P0
rules without saying so - the failure KPI 3 forbids). Across all 29 files at
64 tokens the naive cut carries 78 / 105, the figure the review measured.

Truncation only ever removes P1/P2 text. It never removes a P0 rule, and it
never removes a rule from the record.

Cost: `compile_csp` takes 0.78 ms median, 1.49 ms p95 and 2.14 ms at most
(145 compilations, the 29 fixtures five times each, warm templates, vla-real on
the lab desktop, re-measured after the review fixes). That is cheap enough to
recompile after every hot-apply.

Token use per policy, exact OpenVLA count in brackets: corridor_survey 119
(97), edge_patrol 114 (94), hp_30 / hp_42 / hp_8 126 (106), urban_clearance 124
(104), sitl_pedestrian 73 (73), wgs84_taipei 87 (77). The full table is printed
by `report --exact`.

## Verification beyond the tests passing

**Failing first (first version).** The test file of the first version was run
against the unchanged `compiler.py` (the HEAD copy, last changed in 445bdbe)
with `csp.py` and `templates/` removed. It scored **3/48**. All three passes are
expected on the old code: the `build_prompt` key check, and the two
`parse_command` tests, which pin behaviour that already worked (audit WP2-14
asked for them; they are not fixes). The window defect fails on its own line:
`Never enter zone 'nfz-school'.` with no schedule.

**Failing first (review fixes).** The 65-test file was run against the first
version's `compiler.py`, `csp.py` and templates (a scratch copy). It scored
**57/65**:

- `test_a_corridor_altitude_band_bounds_the_action_set` FAILS: corridor_survey's
  `altitude_band_m_agl=None`.
- `test_without_a_clock_the_kpi_report_and_the_content_hash_do_not_move` FAILS:
  "29/29 report rows changed hash between two runs".
- `test_issued_at_has_one_convention_and_records_where_it_came_from` FAILS: the
  wall-clock stamp was `...+00:00` while the clocked one was naive.
- Four tests ERROR on what did not exist: the baseline fields of the report
  (2), `naive_prefix_baseline`, and `issued_at_source`.

The other new tests pass on the first version, as they should: they pin
behaviour that was already right but that nothing checked. Their evidence is
the mutation run below.

**Mutations.** 24 mutants, each in a scratch copy of the package, never in the
project tree; the full run is in the review-fix report. Every one is killed by
a named test (unmutated: 64/65, the 1 being the visible SKIP for the
uncommitted schema):

| Mutation | Killed by |
|---|---|
| `p0_coverage` ignores the text | the coverage null (text half) |
| `p0_coverage` ignores `P0_constraints` | the coverage null (structure half) |
| Action sets built from kept rules only | the truncation and P0-overflow tests |
| Skip-and-continue instead of a strict prefix | the strict-prefix test |
| Window removed from the summary template | the windowed-P1-summary test |
| Time-criticality ignores switch-off | the switch-off test |
| Region filter ignores the fence margin | the margin-ring test |
| `SOFT_FACTOR` 1.0 | the soft-severity test |
| Altitude bands unioned; speed cap max instead of min | the envelope-intersection test (and the corridor-band test) |
| Reach ignores the speed cap | the reach test |
| `P2_summary` kept when no P2 rule is kept | the truncation test |
| Corridor reason with `start_in` forced; stand-off reason with `follow` forced | the corridor/clearance/stand-off explanation test |
| `action_violations` corridor check removed | the keep-in corridor test |
| Corridor bands left out of the action set | the corridor-band test |
| Wall-clock stamp in UTC again | the issued_at convention test |
| Content hash covers `issued_at`; report rows keyed by `csp_hash` | the ticking-clock report test |
| Baseline skips and continues; `kpi_discriminates` always False | the baseline test |
| Table entry 'Never' 1 -> 0 or 'altitude.' 3 -> 2, tokenizer hidden | the frozen chunk-count test (tokenizer-free) |
| Table entry 'zone' 1 -> 0, tokenizer present | the chunk-count test and the tokenizer test |

The table mutants matter: before the review fixes, 'zone' 1 -> 0 survived every
check even with the tokenizer, because a whole sentence like
`Never enter zone 'nfz-school'.` over-counts the quoted id by enough to absorb
one missing token. The tokenizer test now requires every table entry to equal
its exact count, and `_EXACT_CHUNKS` freezes those counts (and every fixture
chunk's) so the bound is checked where there is no tokenizer.

From the first version's run, still valid: a constant explainer, one P0 entry
dropped, a different exception type, every risk equal, the report reading
100 % when nothing compiled, and every rule counted as "in the text" are each
caught by the test named in that version.

**Both environments.**

- vla-real (3.10): 64/65, with 1 SKIP (no committed `csp.schema.json`).
- vla-drone (3.11): 62/65, with 3 SKIPs: no jsonschema, no tokenizer, no
  committed schema. The upper bound is still checked there, through the frozen
  text and chunk counts. Under pytest the same tests call `pytest.skip`
  instead of returning, so they cannot read as passed.
- Other test files that read the compiler, at the time of writing (other units
  are editing them): `tests/test_bundle.py` 40/40 in vla-real (34/40 with 6
  SKIPs in vla-drone, which has no cryptography); `tests/test_real_vla_prompt.py`
  43/43. `tests/test_paraphraser.py` was at 33/53 when last run, every failure
  a `make_paraphrase_id()` signature error from `guardrail/paraphraser.py`,
  which another unit was editing at the time; none involves the compiler.

## Deviations to record in the final report

1. Multi-scale geometry is computed by the compiler, once per policy, not
   carried in the IR (§9).
2. The token budget default (256) and the severity values (1.0 / 0.6 / 0.3) are
   our choices where the spec leaves numbers open (§2, §5).
3. The cost-map adapter is not built (§11, pending the PI).
4. The CSP adds `policy_id` and `selection`, and the three sub-models add named
   fields (§1). A reader built to the spec ignores them; ours refuses unknown
   ones.
5. Projection to the local frame happens once, at load time
   (`models.load_policy` calls `projection.project_raw`). The DSL page says the
   Prefix Compiler and the Shield compute it "lazily"
   (`kuanting-vla-uav-guardrail/docs/02-implementation/policy-dsl.md:17`).
   Those are WP1 files and were not changed here. Both consumers read the same
   projected numbers, so the CSP and the Shield cannot disagree about where a
   zone is. Audit card WP2-16 asks for this difference to be written down if
   load-time projection stays.
6. `issued_at` follows the clock's convention (naive policy-local time) rather
   than the spec example's UTC `...Z`, and without a clock it is the wall
   clock, so the CSP is a stateless function of its inputs only given a clock
   or an explicit `issued_at` (§12). `csp_content_hash` is the reproducible
   identity in every case.

## Not done (honest list)

- **No flight has used the typed CSP in this unit.** `demo/real_vla_demo.py
  --csp on` (WP2-01, another unit) now calls `compile_csp`, injects its
  `natural_language_prompt` and writes the CSP as `csp/csp-g<gen>-<hash>.json`
  next to the flight. Its tests pass (35/35). The demo's Shield has no clock,
  so that CSP's time filter is off and every rule is treated as in force.
- **The effectiveness evaluation CLI is not built** (offline replay and A/B,
  PDF p5; card WP2-09). `action_violations` is the per-candidate check such a
  replay would count with.
- **The SITL rail writes no CSP** (WP2-11/15). `sitl/ros2_shield_node.py` and
  `sitl/run_sitl_demo.py` still write only `build_prompt`'s YAML. `write_csp`
  exists for them to call.
- **R7's "structured-only" fallback** for dense P0 policies is not offered. P0
  overflow always raises.
- **`csp.schema.json` is not committed** (so WP2-04 stays PARTIAL). It is
  generated with `python -m guardrail.compiler schema <path>`. The tests check
  that the exported schema requires the locked fields, accepts real CSPs and
  rejects a CSP without `P0_constraints`. The drift test is written and SKIPs
  visibly until a copy exists at `docs/data/csp.schema.json` or
  `guardrail/csp.schema.json`; neither path is this unit's to write.
- **The KPI report is not stored in `docs/data/`** (so WP2-02 and WP2-03 are
  done in code but PARTIAL as deliverables). `report --json <file>` writes it.
  That folder belongs to another unit, so the file was written only to a
  scratch folder here.
- **`guardrail/kpi.py` has no CSP field** (WP2-02). The final KPI report reads
  `kpi.py`, which is another unit's file. The numbers above come from
  `coverage_report`.
- **The mission context has no "allowed modes"** (PDF p3 lists pose, home,
  target and allowed modes). `Mission` carries the start, the target, the
  cruise altitude and the requested speed.
- **`now` is assumed to be in the policy's local time**, as `Shield(now=...)`
  already assumes. No timezone is stored in a policy.

## Outside this unit

| File | Change wanted |
|---|---|
| `guardrail/ir.py` (WP1) | Carry the coarse/fine cache built at ingest. It can call `compiler.multiscale_geometry`. Then make the compiler read it. |
| `sitl/ros2_shield_node.py`, `sitl/run_sitl_demo.py` | Call `write_csp(out / f"csp_g{generation}.json", mission, now=<the Shield's clock>)` at start and after each hot-apply. |
| `guardrail/kpi.py` (WP2-02/03) | Add the compiler KPIs (budget, P0 coverage, raised count, explained count) by calling `compiler.coverage_report`. |
| `docs/data/csp_kpis.json` | Generate with `python -m guardrail.compiler report --exact --json docs/data/csp_kpis.json` (vla-real env). |
| `csp.schema.json` (`docs/data/csp.schema.json` or `guardrail/csp.schema.json`) | Generate with `python -m guardrail.compiler schema <path>` and commit it. The drift test in `tests/test_csp.py` already looks at both paths and stops skipping once one exists. |
| `requirements.txt` | Add `jinja2` (pin `3.1.6`, as `sitl/setup_sitl.sh` does). `build_prompt` and `summary_pack` render through Jinja2 and raise ImportError without it; `pyproject.toml` lists it and `.github/workflows/tests.yml` installs it by hand because `requirements.txt` does not. |
| `demo/real_vla_demo.py` (WP2-01) | Log `csp.csp_content_hash(csp)` beside `csp_hash`, or pass `issued_at=` explicitly, so two runs of the same flight can be matched by content. Its CSPs are compiled without a clock and are stamped from the wall clock (`selection.issued_at_source == "wall_clock"`). |

Done since the first version of this note, by other units: `demo/real_vla_demo.py`
now uses `compile_csp`, and the paraphraser store's source texts carry the
schedule clause (`experiments/paraphrases/csp_corridor_survey.json`).
