"""The Constraint Summary Pack (CSP) as a typed record - the Prefix Compiler's only output.

WHAT THE GRANT LOCKS

Prefix Compiler PDF p1: "Implementation: Python 3.11+ with Pydantic v2 for the
CSP schema", and p1-2 the class itself - fourteen fields, from `csp_version` to
`relevance_explanations`. The output table on p5-6 adds "CSP JSON Schema export
- csp.schema.json". Until 2026-10-06 `ConstraintCompiler.summary_pack()` returned
a plain dict whose fields matched none of these names (audit card WP2-04,
DRIFT): a consumer written against the grant's CSP could not parse ours.

WHAT IS COPIED, AND WHAT IS ADDED

The fourteen locked fields are copied by name, type and optionality, in the
spec's order (`LOCKED_FIELDS` pins that, and tests/test_csp.py checks it). The
spec leaves `P0Entry`, `AllowedActionSet` and `ForbiddenActionSet` as forward
references with one worked example each (PDF p2); their fields here are the
ones that example uses, plus a few named additions documented on each class.

Two top-level fields are added, and both exist so the CSP can be audited rather
than trusted:

  policy_id  - the hash identifies the policy; the id lets a person find it.
  selection  - one row per rule in the policy, with what the compiler decided
               (kept / dropped for budget / filtered as not in force / filtered
               as out of region), the risk score and its three parts, the
               sentence and its token count, and the reason. The grant's
               truncation rule says P1/P2 rules may be cut; a cut nobody can see
               is a "silent zero" (CONTRIBUTING.md), so every rule that is left
               out is listed with why. It also says where `issued_at` came
               from (the caller, the clock, or the wall clock).

TWO HASHES

  csp_hash          - the whole CSP, `issued_at` included: names one issued CSP
                      (a flight log naming the file it injected).
  csp_content_hash  - everything but the issue stamp: "is this the same CSP?".
                      Without a clock the stamp is the wall clock, so only this
                      one is reproducible from run to run.

`extra="forbid"` on every model: a misspelt field (`P0_constraint`) is refused
instead of being dropped while the real field silently takes its default.

TOKEN COUNTING

The budget KPI ("CSP token / length budget is configurable and respected",
PDF p5) needs a token count, and the target model is OpenVLA-7B, whose language
backbone is Llama-2 with a SentencePiece tokenizer (D:/models/openvla-7b,
`llm_max_length` 2048). Two counters are provided:

  OpenVLATokenCounter - exact. Needs the `tokenizers` package and the model's
                        tokenizer.json, so it exists in the vla-real env on the
                        lab machine and nowhere else (not in vla-drone, not in
                        the WSL ROS 2 Python).
  TableTokenCounter   - the default. Pure Python, identical everywhere, and an
                        UPPER BOUND on the exact count by construction (see the
                        class). Using it as the default keeps the CSP a
                        stateless function of its inputs on every rail: a
                        machine-dependent default would let the same policy and
                        mission compile to different CSPs on SITL and on AirSim.

The counter's name is written into every CSP (`selection.token_counter`), so a
reader knows what "tokens" meant.

This module holds no compiler logic. The pipeline (filter, risk grade,
truncate, render) lives in guardrail/compiler.py, which imports this module and
never the other way round.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

CSP_VERSION = "1.0"

# The spec's field list, in its order (Prefix Compiler PDF p1-2).
LOCKED_FIELDS = (
    "csp_version", "policy_hash", "generation", "issued_at", "mission_id",
    "lookahead_s", "P0_constraints", "P1_summary", "P2_summary",
    "allowed_action_set", "forbidden_action_set", "cost_map_ref",
    "natural_language_prompt", "relevance_explanations",
)

# Where the exact tokenizer lives on the lab machine; override with OPENVLA_DIR.
DEFAULT_OPENVLA_DIR = "D:/models/openvla-7b"


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


# --------------------------------------------------------------------------- #
# The parts the spec names by forward reference
# --------------------------------------------------------------------------- #

class P0Entry(_Strict):
    """One retained P0 rule. "Always full geometry refs" (PDF p1).

    The spec's example carries four fields; two are added so a consumer can act
    on an entry without loading the policy:
      constraint_type - hard/soft, which the risk grade already depends on.
      in_force        - the rule's schedule in words ("Mon-Fri 07:30-17:30"), or
                        "always". The audit found the old prompt rendering a
                        weekday-only school zone as a permanent ban.

    `geometry_ref` is "<scale>:<rule id>@v<policy version>/g<generation>", the
    spec's own form ("coarse:nfz-school-yard@v0.3.0/g0"). Rules with no geometry
    (envelopes, stand-offs) get the scale "param": the rule IS its parameters.
    `compiler.resolve_geometry_ref` turns a ref back into vertices and refuses a
    ref from another generation.
    """
    id: str
    type: str
    geometry_ref: str
    violation_action: str
    constraint_type: Literal["hard", "soft"] = "hard"
    in_force: str = "always"


class AllowedActionSet(_Strict):
    """Action bounds in the VLA's action space (PDF p2, p4 "action-set adapter").

    FRAME. The grant locks body frame (vx forward, vy right, vz up, yaw_rate).
    Every bound here comes from a rotation-invariant cap - a horizontal speed
    limit is a disc, a climb limit and a yaw-rate limit do not depend on
    heading - so the same numbers are exact in body frame and in this project's
    world frame (guardrail/models.py Action4D, x North). `vx_range_mps` and
    `vy_range_mps` are the axis-aligned box around that disc, as in the spec's
    example; the box admits sqrt(2) times the cap on a diagonal, so the true cap
    is also given as `horizontal_speed_max_mps` (an addition).

    NONE MEANS UNBOUNDED. A policy with no kinematic rule gets None, never 0: a
    zero range would read as "hover only", the opposite of the truth.

    `altitude_band_m_agl` is the intersection of every hard altitude envelope
    AND every hard corridor's floor-ceiling band, as the Shield enforces both
    and the spec's example intersects them (a 5-120 m envelope and a 30-80 m
    corridor give [30, 80]). None only when no rule bounds altitude.

    `source_rules` (an addition) names the rules each bound came from.
    """
    frame: Literal["body"] = "body"
    vx_range_mps: tuple[float, float] | None = None
    vy_range_mps: tuple[float, float] | None = None
    vz_range_mps: tuple[float, float] | None = None
    yaw_rate_max_dps: float | None = None
    altitude_band_m_agl: tuple[float, float] | None = None
    horizontal_speed_max_mps: float | None = None
    source_rules: list[str] = Field(default_factory=list)


class ForbiddenActionSet(_Strict):
    """What no action may do (PDF p2). `no_translate_out_of_corridors` is an
    addition: the spec's example has no keep-IN rule, and this project's DSL
    does (models.py Corridor)."""
    no_translate_into_polygons: list[str] = Field(default_factory=list)
    no_translate_out_of_corridors: list[str] = Field(default_factory=list)


# --------------------------------------------------------------------------- #
# The audit trail of the selection (addition)
# --------------------------------------------------------------------------- #

class RiskWeights(_Strict):
    """risk = severity*sev + proximity*prox + time_critical*tcrit (PDF p4).

    The defaults are the grant's 0.5 / 0.3 / 0.2, which the spec itself calls
    placeholders "need empirical tuning" (PDF p6). Configurable for that reason;
    the weights must sum to 1 so a score stays in [0, 1] and two CSPs' scores
    stay comparable.
    """
    severity: float = Field(0.5, ge=0.0)
    proximity: float = Field(0.3, ge=0.0)
    time_critical: float = Field(0.2, ge=0.0)

    @model_validator(mode="after")
    def _sums_to_one(self) -> "RiskWeights":
        total = self.severity + self.proximity + self.time_critical
        if abs(total - 1.0) > 1e-9:
            raise ValueError(f"risk weights must sum to 1, got {total:g}")
        return self


Decision = Literal["kept", "dropped_budget", "filtered_inactive",
                   "filtered_out_of_region"]


class RuleDecision(_Strict):
    """What happened to one rule of the policy, and why."""
    id: str
    type: str
    priority: Literal["P0", "P1", "P2"]
    constraint_type: Literal["hard", "soft"]
    decision: Decision
    severity: float | None = None
    proximity: float | None = None
    time_critical: float | None = None
    risk: float | None = None
    tokens: int | None = None          # this rule's sentence on its own
    sentence: str | None = None
    reason: str


class Selection(_Strict):
    """How the CSP was cut, so the cut can be checked instead of trusted."""
    budget_tokens: int
    tokens_used: int                   # of natural_language_prompt
    p0_tokens: int                     # of the P0 sentences alone
    token_counter: str
    weights: RiskWeights
    now: str | None                    # the clock the time filter used
    time_filter: bool                  # False: no clock, every rule in force
    region: dict[str, float] | None    # mission bbox + margin; None: no filter
    region_margin_m: float | None
    reach_m: float                     # speed bound x lookahead
    show_priority: bool
    # Where CSP.issued_at came from: "argument" (the caller's issued_at),
    # "clock" (= now) or "wall_clock" (neither given; the one field of a CSP
    # that then differs from run to run - compare by csp_content_hash).
    # None: written before 2026-10-06, when this was not recorded.
    issued_at_source: Literal["argument", "clock", "wall_clock"] | None = None
    rules: list[RuleDecision]          # one per policy rule, policy order


# --------------------------------------------------------------------------- #
# The CSP
# --------------------------------------------------------------------------- #

class CSP(_Strict):
    """Constraint Summary Pack. Locked fields first, in the spec's order."""
    csp_version: Literal["1.0"]
    policy_hash: str                   # sha256:...
    generation: int
    issued_at: str                     # ISO 8601
    mission_id: str
    lookahead_s: float

    P0_constraints: list[P0Entry]      # always full geometry refs
    P1_summary: str                    # natural-language summary
    P2_summary: str | None = None      # optional, only if budget allows

    allowed_action_set: AllowedActionSet
    forbidden_action_set: ForbiddenActionSet
    cost_map_ref: str | None = None    # out of scope: no path planner (see design doc)
    natural_language_prompt: str       # always emitted; templated, never free-form

    relevance_explanations: dict[str, str]   # rule_id -> mission-context fact

    # --- additions (see module docstring) ---
    policy_id: str
    selection: Selection


class CSPUnenforcedRules(Exception):
    """The policy carries rules the Safety Shield does not enforce.

    The four declarable-only classes (distance_envelope, dynamic_nfz,
    time_window_switch, corridor_swap; see models.RUNTIME_TYPES) can be
    written in the DSL, but nothing at runtime checks them. A CSP is the
    model's summary of the rules the Shield enforces, so neither choice the
    compiler could make on its own is safe: telling the model a rule that is
    not enforced makes the prompt promise more than the Shield delivers, and
    dropping it makes a hard rule vanish from both without a trace. The flight
    loaders refuse such a policy (Policy.unenforced_rules); so does the
    compiler, by name. Until 2026-10-06 it raised a bare TypeError from the
    relevance step (marked "pragma: no cover") or TemplateNotFound.

    Deliberately NOT a ValueError subclass, like CSPBudgetExceeded.
    """

    def __init__(self, policy_id: str, rules: dict[str, str]):
        self.policy_id = policy_id
        self.rules = dict(rules)              # rule id -> rule type
        super().__init__(
            f"{policy_id}: cannot compile a CSP - the Safety Shield does not "
            f"enforce {', '.join(f'{i} ({t})' for i, t in self.rules.items())}. "
            f"These types are declarable in the DSL only; the flight loaders "
            f"refuse this policy too (Policy.unenforced_rules).")


class CSPBudgetExceeded(Exception):
    """The P0 rules alone do not fit the token budget.

    "If P0-only emission already exceeds budget -> fail loudly (raise
    CSPBudgetExceeded); do not silently truncate" (PDF p4; risks.md R7).

    Deliberately NOT a ValueError subclass: a caller's generic
    `except ValueError` around policy loading must not swallow it.
    """

    def __init__(self, p0_tokens: int, budget_tokens: int, p0_ids: list[str],
                 token_counter: str):
        self.p0_tokens = p0_tokens
        self.budget_tokens = budget_tokens
        self.p0_ids = list(p0_ids)
        self.token_counter = token_counter
        super().__init__(
            f"the {len(self.p0_ids)} P0 rule(s) in scope need {p0_tokens} tokens "
            f"({token_counter}) but the budget is {budget_tokens}: "
            f"{', '.join(self.p0_ids)}. P0 rules are never truncated; raise the "
            f"budget or narrow the policy's scope.")


# --------------------------------------------------------------------------- #
# Token counters
# --------------------------------------------------------------------------- #

class TokenCounter:
    """Counts the tokens a model would read for a piece of text."""
    name = "abstract"

    def count(self, text: str) -> int:
        raise NotImplementedError

    def __call__(self, text: str) -> int:
        return self.count(text)


# Exact OpenVLA (Llama-2 SentencePiece) token counts for the chunks the
# templates in guardrail/templates/ can produce, measured on 2026-10-06 with
# tokenizers 0.19.1 and D:/models/openvla-7b/tokenizer.json (sha256 prefix in
# _OPENVLA_TABLE_SOURCE). A chunk is a run of characters between single spaces,
# counted on its own - which includes the leading "word start" marker the
# tokenizer adds. Only words and punctuation are tabled; anything holding a
# digit or a quote (rule ids, numbers, clock times) is left to the byte bound.
# Regenerate with `python -m guardrail.compiler table` in the vla-real env.
_OPENVLA_TABLE_SOURCE = "openvla-7b tokenizer.json sha256:8f5e2869e180"
_OPENVLA_TABLE: dict[str, int] = {
    '(Mon,': 3,
    '(Mon-Fri': 4,
    '(Mon-Fri)': 5,
    '(Sat,': 4,
    '(Sun': 3,
    '(Sun)': 4,
    '(Tue-Thu': 6,
    '(Tue-Thu)': 7,
    '(every': 2,
    '(in': 2,
    '(soft': 2,
    '(soft)': 3,
    '<=': 1,
    '>=': 1,
    'Fri': 1,
    'Fri)': 2,
    'Fri).': 2,
    'Keep': 1,
    'Mon,': 2,
    'Mon-Fri': 3,
    'Mon-Fri)': 4,
    'Mon-Fri).': 4,
    'Never': 1,
    'Sat,': 2,
    'Stay': 2,
    'Sun': 1,
    'Sun)': 2,
    'Sun).': 2,
    'Tue-Thu': 5,
    'Tue-Thu)': 6,
    'Tue-Thu).': 6,
    'Wed,': 2,
    'altitude': 2,
    'altitude.': 3,
    'and': 1,
    'any': 1,
    'anything': 1,
    'are': 1,
    'at': 1,
    'away': 1,
    'below': 1,
    'between': 1,
    'building': 1,
    'building.': 2,
    'buildings': 1,
    'centerline': 2,
    'climb': 2,
    'corridor': 3,
    'day': 1,
    'deg/s': 3,
    'deg/s.': 4,
    'enter': 1,
    'entry': 1,
    'every': 1,
    'followed': 1,
    'following': 1,
    'following.': 2,
    'force': 1,
    'from': 1,
    'in': 1,
    'inside': 1,
    'into': 1,
    'its': 1,
    'least': 1,
    'limit)': 2,
    'limit).': 2,
    'm': 1,
    'm.': 2,
    'm/s': 3,
    'm/s,': 4,
    'no': 1,
    'of': 1,
    'or': 1,
    'overnight)': 3,
    'overnight).': 3,
    'pedestrian': 3,
    'pedestrian.': 4,
    'rate': 1,
    'speed': 1,
    'stay': 1,
    'subject': 1,
    'turn': 1,
    'within': 1,
    'you': 1,
    'zone': 1,
}


class TableTokenCounter(TokenCounter):
    """The default counter: never fewer tokens than OpenVLA's tokenizer.

    WHY IT IS AN UPPER BOUND. The Llama-2 tokenizer turns each single space into
    a word-start marker and never merges across one, so the count of a text is
    the sum of the counts of its space-separated chunks (tests/test_csp.py
    checks that identity on every rendered sentence when the tokenizer is
    present). A chunk in the table counts its measured value. Any other chunk
    counts its UTF-8 byte length plus one: SentencePiece with byte fallback can
    never need more than one token per byte plus the marker. Every chunk is
    therefore counted at or above its true value, and so is the sum.

    The cost of the bound is only on rule ids and numbers, which are counted
    near one token per character. Measured 2026-10-06 over the 29 shipped
    policies: 1.00-1.24x the exact count on whole prompts, up to 2.0x on a
    single short clause that is mostly an id (docs/DESIGN-prefix-compiler.md).
    """
    name = "openvla-table-v1"

    def count(self, text: str) -> int:
        if not text:
            return 0
        n = 0
        for chunk in text.split(" "):
            if chunk == "":
                n += 1                 # a doubled space: one bare marker at most
                continue
            k = _OPENVLA_TABLE.get(chunk)
            n += k if k is not None else len(chunk.encode("utf-8")) + 1
        return n


class OpenVLATokenCounter(TokenCounter):
    """Exact count with OpenVLA's own tokenizer.json. Needs `tokenizers`.

    No special tokens are added: the CSP text is spliced into a prompt that
    already has its BOS, so counting one here would overstate every CSP by one.
    Where the text meets the surrounding prompt the boundary can shift the
    count by a token; the budget is for the CSP text itself.
    """

    def __init__(self, tokenizer_json: str | Path):
        from tokenizers import Tokenizer        # optional dependency
        p = Path(tokenizer_json)
        self._tok = Tokenizer.from_file(str(p))
        digest = hashlib.sha256(p.read_bytes()).hexdigest()[:12]
        self.name = f"openvla-llama2-sp@{digest}"

    def count(self, text: str) -> int:
        if not text:
            return 0
        return len(self._tok.encode(text, add_special_tokens=False).ids)


def openvla_token_counter(model_dir: str | Path | None = None
                          ) -> OpenVLATokenCounter | None:
    """The exact counter if this machine has the tokenizer, else None.

    Returns None rather than raising: whether to fall back is the caller's
    decision, and a test that needs the exact count must SKIP visibly, not pass.
    """
    d = Path(model_dir or os.environ.get("OPENVLA_DIR", DEFAULT_OPENVLA_DIR))
    p = d / "tokenizer.json"
    if not p.is_file():
        return None
    try:
        return OpenVLATokenCounter(p)
    except ImportError:
        return None


DEFAULT_TOKEN_COUNTER = TableTokenCounter()


# --------------------------------------------------------------------------- #
# Schema export and hashing
# --------------------------------------------------------------------------- #

def csp_schema() -> dict:
    """The CSP's JSON Schema (draft 2020-12, which Pydantic v2 emits)."""
    schema = CSP.model_json_schema()
    return {"$schema": "https://json-schema.org/draft/2020-12/schema",
            "$id": f"urn:guardrail:csp:{CSP_VERSION}", **schema}


def write_csp_schema(path: str | Path) -> Path:
    """Write csp.schema.json (PDF p5 output table). LF line endings on every
    OS, so a regenerated file diffs only where the schema changed."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(csp_schema(), indent=2) + "\n")
    return p


def _sha(doc: dict) -> str:
    canon = json.dumps(doc, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return "sha256:" + hashlib.sha256(canon.encode("utf-8")).hexdigest()


def csp_hash(csp: CSP) -> str:
    """SHA-256 over the canonical JSON of the whole CSP, `issued_at` included,
    as "sha256:" + all 64 hex digits. The identity of one ISSUED CSP: two
    issues of the same content at different times hash differently, which is
    what a flight log naming the CSP file it injected needs.

    Two compilations of the same policy, mission and clock must hash equal in
    any process - the "stateless function" property of PDF p1 - which is why
    nothing in the compiler may depend on the iteration order of a set.
    tests/test_csp.py checks it across two interpreters with different hash
    seeds. WITHOUT a clock the compiler stamps `issued_at` from the wall clock,
    so this hash then changes from run to run; use `csp_content_hash` to ask
    "is this the same CSP?".
    """
    return _sha(csp.model_dump(mode="json"))


def csp_content_hash(csp: CSP) -> str:
    """SHA-256 over every field of the CSP except the issue stamp (`issued_at`
    and `selection.issued_at_source`), as "sha256:" + 64 hex digits.

    The identity of the CONTENT: same policy, mission, lookahead, clock (`now`)
    and options give the same content hash whenever and wherever it was
    compiled, with or without a clock. The KPI report keys its rows by it, so
    two reports of unchanged code and policies are identical row for row (the
    2026-10-06 review measured 29/29 row hashes differing 1.1 s apart when the
    rows carried `csp_hash` of a wall-clock-stamped CSP).
    """
    doc = csp.model_dump(mode="json")
    doc.pop("issued_at", None)
    doc.get("selection", {}).pop("issued_at_source", None)
    return _sha(doc)
