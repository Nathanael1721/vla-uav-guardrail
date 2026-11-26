"""The Constraint Summary Pack in a text-reading VLA's prompt, for the prefix A/B
on the Project AirSim + ArduPilot follow mission (WP4-15, WP4-01).

WHAT THE GRANT ASKS (Prefix Compiler page)

  "Always-on A/B during stress runs. A scenario can be flagged for A/B; the
  harness runs it twice (Prefix on / off) with the same seed, and the KPI
  report shows the delta."
  "CSP token / length budget is configurable and respected."
  "CSPBudgetExceeded raised, never silently truncated."
  Open question: "What's the exact budget after the natural-language task
  prompt is reserved for the chosen VLA?"

The stress harness's own A/B (experiments/profiles/prefix_ab.yaml) is flown by
a hand-written adapter that reads the CSP's action sets
(guardrail/prefix_consumer.py), because the hil stub reads no text. This
module is the other half: the CSP's templated sentence (the Templated NL
adapter, "language-conditioned models") put in front of a LEARNED VLA's
instruction on the PAS follow (demo/pas_ardupilot/perception_node.py
--vla-prefix), on the arms

  off      the prompt the model always got (byte for byte);
  csp      the CSP's natural_language_prompt, then one space, then the
           model's own instruction;
  neutral  rule-free text (NEUTRAL_SENTENCE, the null of tools/prefix_eval.py)
           length-matched to the CSP within the same budget, in the same
           place: is a delta the rules, or any text in front of the words?

LENGTH-MATCHING THE NEUTRAL ARM (review of 94a7d7b)

Both texts are counted with the model's OWN tokenizer (PromptSlot.
exact_counter), never with an upper bound. AerialVLA's CSP is budgeted with
the compiler's table counter, which counts a templated rule word exactly but
any other word at its UTF-8 bytes plus one: matched under it, the neutral
sentence came out at 18 real OpenVLA tokens against the CSP's 85, while the
record said 86 against 85. So the neutral arm is matched with OpenVLA's
tokenizer.json (guardrail.csp.openvla_token_counter, OPENVLA_DIR) and refused
before take-off ("no_tokenizer") where it is not on the machine - a
mis-matched control is worse than none. Its budget check is in those exact
tokens too (the table count is an upper bound of them, so the CSP that fits
the budget under it fits in exact tokens as well). AIoT-VLA's counter is
already exact (CLIP's own BPE).

WHERE THE TEXT GOES, PER MODEL (PromptSlot, slot_for)

  AerialVLA (and its fine-tunes, and the scripted stand-in that answers in its
  place): the instruction line of its prompt, ahead of the task sentence -
  guardrail/aerialvla_codec.build_prompt, the placement the 7 Nov OpenVLA CSP
  flights used. OpenVLA's language model reads 2048 tokens; the budget is the
  compiler's default (DEFAULT_BUDGET_TOKENS, 256) counted by its default
  counter (an upper bound on OpenVLA's tokenizer), as those flights compiled.

  AIoT-VLA (training/aiotvla, the lab's own model): the object words are its
  ONLY text input. They are the query its frozen OWL-ViT backbone grounds in
  the image, and (v8 on) one more token of the head, both through OWL-ViT's
  CLIP text encoder - whose window is 16 positions, the start and end tokens
  included (text_config.max_position_embeddings; the tokenizer's
  model_max_length). The server (demo/vla_server_foreign.py AIOTVLAFamily)
  prepends the CSP text to the object words and declares the window in its
  hello (`foreign.config.csp_slot`). The budget for the CSP is therefore the
  window minus the two special tokens minus the object words' own tokens:
  "a red car" leaves 16 - 2 - 3 = 11 CLIP tokens. That is the answer to the
  grant's open question for this model, and it is small: the follow policy's
  four P0 sentences need 45 CLIP tokens (measured 18 Nov 2026), so the
  compiler raises CSPBudgetExceeded and the prefix-on arm is refused before
  take-off - the grant's rule, never a truncation. Truncating instead would
  either drop P0 rules (P0 coverage 100 % is a locked KPI) or, at the
  tokenizer, cut the object words off the end of the query, so the model would
  no longer be told what to follow. The refusal names each P0 sentence's own
  count (`p0_rules`): alt-band 10, bld-clearance 9, standoff-any 12,
  standoff-pedestrian 14 - alt-band or bld-clearance alone fits the 11, no
  two together do (19 and up). A policy with one P0 rule would fit, but only
  by demoting the other three (the building clearance and both stand-offs)
  from P0 or dropping them from the flight, which changes the rules the
  follow is flown under; no such policy is shipped. So the prefix A/B a
  text-reading VLA can fly on this follow is AerialVLA's.

  Any other foreign family: refused (its server refuses a CSP arm: none of
  their prompts has a slot).

WHAT THE A/B MEASURES

Neither model was trained with rule text in its prompt. The A/B measures what
the prompt change does to a model that never saw one - a measurement, not an
expected effect - and the record says so (`model_trained_on_prefix` False).
The CSP is the one compiled before take-off at the rule clock's reading
(generation 0): a policy hot-applied in flight does not reach this prompt.
(The stress harness's consumer A/B follows a hot-apply since 2026-11-26,
WP4-19: guardrail/csp_topic.py carries generation N to the VLA stub; this
follow mission's prompt path does not read it.)

WHAT IS RECORDED (prefix_record)

The arm, the prefix's text and its SHA-256, the CSP's issue and content hashes,
the budget, the counter, the window and the token counts. The node writes it to
metrics.json (`vla_prefix`) and to the episode's events.jsonl, and every
request's line in vla_steps.jsonl carries the csp_mode and the SHA-256 of the
text it sent (guardrail/vla_link.build_infer_fields). manifest.json stays the
grant's six fields (guardrail/manifest.build_manifest): the arm is not one.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import Any

from .csp import (DEFAULT_TOKEN_COUNTER, CSPBudgetExceeded, TokenCounter, csp_content_hash,
                  csp_hash, openvla_token_counter)

ARMS = ("off", "csp", "neutral")
# The A/B's side of each arm, as tools/kpi_report.py pairs them ("prefix"
# on / off; the neutral arm is its own side, never paired as "on").
ARM_PREFIX = {"off": "off", "csp": "on", "neutral": "neutral"}
# Rule-free filler for the neutral arm (tools/prefix_eval.py imports it from
# here). Plain description, no number, no direction, no rule word, so it cannot
# carry a constraint by accident.
NEUTRAL_SENTENCE = "The scene is an ordinary outdoor environment seen from a camera."
# One space between the prefix and the instruction, on every model
# (aerialvla_codec.build_prompt; vla_server_foreign.AIOTVLAFamily).
SEPARATOR = " "
AERIALVLA_PLACEMENT = ("the instruction line of AerialVLA's prompt, ahead of the task "
                       "sentence (guardrail/aerialvla_codec.build_prompt)")


class PrefixRefused(ValueError):
    """The prefix cannot be put in this model's prompt as the grant allows;
    `code` names why (no_slot, no_tokenizer, no_room, csp_budget_exceeded,
    compile_failed, window_exceeded, bad_text). The flight is refused before take-off."""

    def __init__(self, code: str, message: str, **facts: Any):
        self.code = code
        self.facts = facts
        super().__init__(f"{code}: {message}")


def sha256_text(text: str | None) -> str | None:
    return None if text is None else hashlib.sha256(text.encode("utf-8")).hexdigest()


# --------------------------------------------------------------------------- #
# CLIP's tokenizer, the counter of a model whose text goes through CLIP
# --------------------------------------------------------------------------- #
class ClipTokenCounter(TokenCounter):
    """Exact count with the model's own CLIP BPE tokenizer (a Hugging Face
    tokenizer object). No special tokens are counted: the window's two are
    accounted for separately (PromptSlot.special_tokens)."""

    def __init__(self, tokenizer, source: str):
        self._tok = tokenizer
        vocab = sorted(tokenizer.get_vocab().items())
        digest = hashlib.sha256(json.dumps(vocab).encode("utf-8")).hexdigest()[:12]
        self.name = f"clip-bpe:{source}@{digest}"

    def count(self, text: str) -> int:
        if not text:
            return 0
        # verbose=False: a CSP longer than the window is counted on purpose
        # (to refuse it); the tokenizer's "longer than the maximum" warning
        # would only repeat that
        return len(self._tok(text, add_special_tokens=False, verbose=False)["input_ids"])


def clip_token_counter(model_id: str) -> ClipTokenCounter | None:
    """The CLIP counter for `model_id` from this machine's files only (no
    download), or None when the tokenizer is not here: whether that refuses
    the flight is the caller's decision (slot_for refuses)."""
    try:
        from transformers import AutoTokenizer          # optional dependency
        tok = AutoTokenizer.from_pretrained(model_id, local_files_only=True)
    except Exception:                                   # noqa: BLE001
        return None
    return ClipTokenCounter(tok, model_id)


# --------------------------------------------------------------------------- #
# The slot
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class PromptSlot:
    """Where the prefix goes in one model's prompt and how much room it has.
    `window_tokens` None: no hard window to check beyond the budget.
    `counter` budgets the CSP; `exact_counter` is the model's own tokenizer
    (the same object where `counter` is already exact; None where it is not
    on this machine), which the neutral arm is length-matched with."""
    model: str
    placement: str
    counter: TokenCounter
    budget_tokens: int
    window_tokens: int | None = None
    special_tokens: int = 0
    object_tokens: int | None = None
    budget_basis: str = ""
    exact_counter: TokenCounter | None = None

    def record(self) -> dict:
        return {"model": self.model, "placement": self.placement,
                "token_counter": self.counter.name, "budget_tokens": self.budget_tokens,
                "budget_basis": self.budget_basis, "window_tokens": self.window_tokens,
                "special_tokens": self.special_tokens, "object_tokens": self.object_tokens,
                "exact_token_counter": (None if self.exact_counter is None
                                        else self.exact_counter.name)}


def _family(identity: dict | None) -> str | None:
    from .aerialvla_codec import foreign_family
    return foreign_family((identity or {}).get("id"))


def slot_for(identity: dict | None, object_text: str, *,
             counter_factory=clip_token_counter,
             exact_counter_factory=openvla_token_counter) -> PromptSlot:
    """The prompt slot of the model whose hello `identity` is (module
    docstring). PrefixRefused when it has none, when its tokenizer is not on
    this machine, or when its object words leave no room. AerialVLA's exact
    counter is OpenVLA's tokenizer.json where this machine has it
    (`exact_counter_factory`, OPENVLA_DIR), else None: only the neutral arm
    needs it, and refuses without it (compile_prompt_prefix)."""
    from .compiler import DEFAULT_BUDGET_TOKENS
    fam = _family(identity)
    if fam is None:
        return PromptSlot(model="aerialvla", placement=AERIALVLA_PLACEMENT,
                          counter=DEFAULT_TOKEN_COUNTER, budget_tokens=DEFAULT_BUDGET_TOKENS,
                          budget_basis=("the compiler's default budget (guardrail/"
                                        "compiler.DEFAULT_BUDGET_TOKENS) under its default "
                                        "counter, as the 7 Nov OpenVLA CSP flights"),
                          exact_counter=exact_counter_factory())
    foreign = (identity or {}).get("foreign") or {}
    cfg = (foreign.get("config") or {}).get("csp_slot")
    if not isinstance(cfg, dict):
        raise PrefixRefused("no_slot", f"the {fam} server declares no CSP slot in its hello "
                                       f"(demo/vla_server_foreign.py refuses a CSP arm for "
                                       f"every family but aiotvla)", family=fam)
    try:
        window = int(cfg["text_window_tokens"])
        specials = int(cfg["special_tokens"])
        tokenizer = str(cfg["tokenizer"])
        placement = str(cfg["placement"])
    except (KeyError, TypeError, ValueError) as exc:
        raise PrefixRefused("no_slot", f"the {fam} server's csp_slot is malformed: {exc!r}",
                            family=fam) from None
    counter = counter_factory(tokenizer)
    if counter is None:
        raise PrefixRefused("no_tokenizer", f"the {tokenizer} tokenizer is not on this "
                                            f"machine: the CSP's tokens cannot be counted "
                                            f"for the {fam} window, and an uncounted prefix "
                                            f"is never sent", tokenizer=tokenizer)
    obj = counter.count(object_text)
    budget = window - specials - obj
    if budget < 1:
        raise PrefixRefused("no_room", f"the object words {object_text!r} take {obj} of the "
                                       f"{window - specials} tokens of the {fam} text window: "
                                       f"no room for any prefix", window_tokens=window,
                            object_tokens=obj)
    return PromptSlot(model=fam, placement=placement, counter=counter, budget_tokens=budget,
                      window_tokens=window, special_tokens=specials, object_tokens=obj,
                      budget_basis=(f"the model's text window {window} minus its {specials} "
                                    f"special tokens minus the object words' {obj}"),
                      exact_counter=counter)


# --------------------------------------------------------------------------- #
# The prefix
# --------------------------------------------------------------------------- #
def neutral_text(n_tokens: int, counter: TokenCounter, max_tokens: int | None = None) -> str:
    """Rule-free text of NEUTRAL_SENTENCE's words, repeated as needed, of at
    least `n_tokens` tokens - or, when that would exceed `max_tokens`, the
    longest run of whole words that fits."""
    if n_tokens <= 0:
        raise ValueError("the CSP has no tokens: there is nothing to length-match")
    words = NEUTRAL_SENTENCE.split(" ")
    out: list[str] = []
    i = 0
    while not out or counter.count(" ".join(out)) < n_tokens:
        out.append(words[i % len(words)])
        i += 1
    if max_tokens is not None:
        while len(out) > 1 and counter.count(" ".join(out)) > max_tokens:
            out.pop()
        if counter.count(" ".join(out)) > max_tokens:
            raise PrefixRefused("no_room", f"not one neutral word fits {max_tokens} tokens")
    return " ".join(out)


def instruction_text(prefix: str | None, object_text: str) -> str:
    """What a windowed model (AIoT-VLA) reads: the prefix, one space, the
    object words; the object words alone with no prefix."""
    return object_text if prefix is None else f"{prefix}{SEPARATOR}{object_text}"


def compile_prompt_prefix(policy, mission, slot: PromptSlot, arm: str, object_text: str,
                          *, now=None, issued_at=None) -> tuple[dict, Any]:
    """(the record, the CSP or None) for one arm. The CSP is compiled with
    the slot's budget and counter (guardrail/compiler.compile_csp), at `now`
    (the rule clock); P0 rules that do not fit raise PrefixRefused
    "csp_budget_exceeded" - the grant's CSPBudgetExceeded, never truncated -
    with each P0 sentence's own count beside the budget (`p0_rules`).
    The neutral arm compiles the CSP too, to match its length with the
    model's own tokenizer (slot.exact_counter; "no_tokenizer" without it)."""
    if arm not in ARMS:
        raise ValueError(f"arm {arm!r} is not one of {ARMS}")
    if arm == "neutral" and slot.exact_counter is None:
        raise PrefixRefused(
            "no_tokenizer", f"the neutral arm is length-matched to the CSP in the {slot.model} "
                            f"model's own tokens, and its tokenizer is not on this machine "
                            f"(OpenVLA's tokenizer.json under OPENVLA_DIR, with the "
                            f"`tokenizers` package). The budget counter "
                            f"({slot.counter.name}) is an upper bound that counts the neutral "
                            f"words several times over, so a control matched with it would be "
                            f"a fraction of the CSP's length: refused rather than mis-matched",
            counter=slot.counter.name)
    rec: dict[str, Any] = {
        "arm": arm, "prefix": ARM_PREFIX[arm], "csp_mode": arm,
        "slot": slot.record(),
        "model_trained_on_prefix": False,
        "measures": ("what putting the text in front of the instruction does to a model "
                     "never trained with rule text in its prompt - a measurement, not an "
                     "expected effect")}
    if arm == "off":
        rec.update(prefix_text=None, prefix_sha256=None, prefix_tokens=0, csp_hash=None,
                   csp_content_hash=None, p0_ids=None, instruction_tokens=(
                       None if slot.window_tokens is None
                       else slot.counter.count(object_text) + slot.special_tokens))
        return rec, None
    from .compiler import ConstraintCompiler
    try:
        csp = ConstraintCompiler(policy).compile_csp(
            mission, now=now, issued_at=issued_at, budget_tokens=slot.budget_tokens,
            token_counter=slot.counter)
    except CSPBudgetExceeded as exc:
        each = _p0_sentence_tokens(policy, mission, slot, now=now, issued_at=issued_at)
        alone = ("" if not each else
                 "; each P0 sentence alone: " + ", ".join(
                     f"{r['id']} {r['tokens']}" for r in each))
        raise PrefixRefused(
            "csp_budget_exceeded",
            f"{exc} (the {slot.model} slot: {slot.budget_basis}{alone}). The prefix-on arm "
            f"cannot be flown on this model with this policy without dropping P0 rules, "
            f"which the grant forbids", p0_tokens=exc.p0_tokens,
            budget_tokens=exc.budget_tokens, p0_ids=exc.p0_ids,
            token_counter=exc.token_counter, p0_rules=each) from None
    except Exception as exc:                            # noqa: BLE001
        # CSPAltitudeConflict, CSPUnenforcedRules (an MSL rule without a
        # ground), ...: the compiler refused this policy and mission
        raise PrefixRefused("compile_failed", f"the Prefix Compiler refused: "
                                              f"{type(exc).__name__}: {exc}") from None
    csp_text = csp.natural_language_prompt.strip()
    csp_tokens = slot.counter.count(csp_text)
    exact = slot.exact_counter
    csp_exact = None if exact is None else exact.count(csp_text)
    text = csp_text
    # The counter the sent text is held to the budget with: the CSP's own
    # (the compiler's) for the CSP; the model's exact one for the neutral
    # text, which is matched in it (module docstring).
    held_by = slot.counter
    if arm == "neutral":
        held_by = exact
        text = neutral_text(csp_exact, exact, max_tokens=slot.budget_tokens)
    from .aerialvla_codec import CodecError, validate_csp
    try:
        validate_csp(arm, text)
    except CodecError as exc:
        raise PrefixRefused("bad_text", f"{exc.code}: {exc.detail}") from None
    n = held_by.count(text)
    if n > slot.budget_tokens:
        raise PrefixRefused("csp_budget_exceeded", f"the {arm} prefix is {n} tokens, over the "
                                                   f"{slot.budget_tokens}-token budget")
    instr = None
    if slot.window_tokens is not None:
        # The model reads prefix + space + object words in one window: counted
        # whole, so a token merged across the joint is not missed.
        instr = slot.counter.count(instruction_text(text, object_text)) + slot.special_tokens
        if instr > slot.window_tokens:
            raise PrefixRefused("window_exceeded", f"the instruction is {instr} tokens with "
                                                   f"its special tokens, over the "
                                                   f"{slot.window_tokens}-token window")
    rec.update(prefix_text=text, prefix_sha256=sha256_text(text), prefix_tokens=n,
               prefix_token_counter=held_by.name,
               # the sent text in the model's own tokens (None: its tokenizer
               # is not on this machine), the one count comparable across arms
               prefix_tokens_exact=None if exact is None else exact.count(text),
               csp_hash=csp_hash(csp), csp_content_hash=csp_content_hash(csp),
               csp_tokens=csp_tokens, policy_hash=csp.policy_hash,
               generation=csp.generation, issued_at=csp.issued_at,
               p0_ids=[p.id for p in csp.P0_constraints], instruction_tokens=instr)
    if arm == "neutral":
        rec["length_matched_to"] = {"csp_sha256": sha256_text(csp_text),
                                    "token_counter": exact.name,
                                    "csp_tokens": csp_exact, "neutral_tokens": n,
                                    "csp_tokens_budget_counter": csp_tokens}
    return rec, csp


def _p0_sentence_tokens(policy, mission, slot: PromptSlot, *, now=None,
                        issued_at=None) -> list[dict] | None:
    """Each P0 rule's sentence counted alone with the slot's counter, from a
    compile without a budget limit - what a refusal shows beside the budget
    so the reader sees which rules alone would fit. None where that compile
    fails too (the refusal stands without it)."""
    from .compiler import ConstraintCompiler
    try:
        full = ConstraintCompiler(policy).compile_csp(
            mission, now=now, issued_at=issued_at, budget_tokens=10 ** 6,
            token_counter=slot.counter)
    except Exception:                                   # noqa: BLE001
        return None
    p0 = {p.id for p in full.P0_constraints}
    return [{"id": r.id, "tokens": r.tokens, "fits_alone": (r.tokens is not None
                                                            and r.tokens <= slot.budget_tokens)}
            for r in full.selection.rules if r.id in p0]
