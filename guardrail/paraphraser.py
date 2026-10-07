"""
Paraphraser - the free-form wording service the grant keeps apart from the
Prefix Compiler (WP2-13, WP4-10, ARCH-23).

What the grant asks for
-----------------------
The Prefix Compiler's natural-language adapter is "Always-emitted, templated,
never free-form. Free-form paraphrasing is the paraphraser's job (Q3
deliverable, separate service); the Prefix Compiler emits the canonical
templated form so the paraphraser has a clean source" (Prefix Compiler PDF p5;
kuanting-vla-uav-guardrail/docs/02-implementation/prefix-compiler.md:161). The
schedule places the "paraphraser deliverable" in Q3, Aug-Oct (Overview PDF p3,
Q3 row; Grant overview PDF p3, Gantt), and WP4 owns the KPI it exists for:
"per-paraphrase robustness" (Grant overview PDF p2, WP table). In the
reference data-flow the orchestrator "issue[s] tasks (and paraphrased
variants) to the VLA" (kuanting-vla-uav-guardrail/docs/03-simulation/
data-flow.md:57).

The grant does not fix K, the seed rule or the log format. Where it is silent
this module copies the Stress Testing contract it already has: a run is
reproducible from its `random_seed` (Stress Testing PDF p1, "Determinism"), so
WHICH paraphrase a trial used is a pure function of (source, seed, backend),
and every paraphrase carries an id that can go into the flight's records.

What it does
------------
    paraphrase(text, n, seed, backend) -> [Paraphrase, ...]

  stored     serves a fixed free-form set from experiments/paraphrases/*.json.
             This is the deliverable: K=8 paraphrases per instruction in
             scope, written once by an LLM, frozen with provenance, validated.
  template   a seeded, deterministic rule bank over the compiler's own
             sentence shapes. The fallback when a source has no stored set.
  llm:<id>   the slot for a live LLM service (LLMServiceBackend). Interface
             only: this build makes no network call, and with no client the
             backend refuses rather than inventing text.
  auto       stored if the source has a set, else template - and the returned
             objects say which one served, so a fallback is never silent.

Every candidate from every backend goes through `validate()` before it is
returned. The validator extracts a fixed list of slot kinds - the target
object and the colour bound to it, every number with its unit, its bound and
what it limits, every named zone / corridor / place and the role a place plays,
the polarity of each zone (keep out vs stay in), the direction words and the
action, each with whether it is negated - and refuses a candidate in which one
of them was dropped, changed or added. It also refuses a hard rule that was
hedged, made conditional or given an exception ("try to", "unless", "if",
"otherwise"), and the identity "paraphrase", which tests nothing. Each refusal
is logged with its reason.

What it does not do
-------------------
It is a lexical guard, not an entailment model, and it only sees the slot
kinds above. A change of meaning that touches none of them - "pedestrians in
the street" for "pedestrians", "dark red" for "red" - passes. Its recall is
therefore MEASURED, two ways, and neither number is a guarantee: against the
mutation classes `mutants()` generates (recall on those classes only - a class
nobody wrote is not measured), and against hand-written probe sets in
experiments/paraphrases/probes.json, one of which was written before the
validator was last changed and never used to tune it. Both are in
`python -m guardrail.paraphraser validate`'s report; docs/DESIGN-paraphraser.md
quotes them with their limits.

It also never paraphrases AerialVLA's prompts. AerialVLA's one slot that
demonstrably steers is a fine-tuned compass phrase; "A phrase the LoRA never
saw in training is worth nothing, so the strings are not paraphrased and not
reformatted" (demo/vla_bridge.py:30-32; docs/FINDING-what-drives-aerialvla.md).
`paraphrase()` raises ProtectedTextError on anything shaped like that prompt.
"""
from __future__ import annotations

import hashlib
import json
import logging
import math
import random
import re
import unicodedata
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Iterator, Protocol

log = logging.getLogger("guardrail.paraphraser")

ROOT = Path(__file__).resolve().parents[1]
STORE_DIR = ROOT / "experiments" / "paraphrases"
STORE_SCHEMA = "guardrail.paraphrases/1"
REPORT_SCHEMA = "guardrail.paraphrases.validation/1"
PROBES_SCHEMA = "guardrail.paraphrases.probes/1"

# K per source. The grant names no number; eight gives a per-paraphrase KPI a
# spread to report without making a nightly sweep (Stress Testing PDF, "~200
# scenarios x 3 seeds") eight times its size for every arm.
K_DEFAULT = 8

# Word-level edit distance / longer length. Below this the "paraphrase" is the
# source with a word swapped - not a different wording, so not a test of one.
MIN_NOVELTY = 0.15
# A paraphrase four times the source's length is a token-budget stress (the
# Prefix Compiler's CSPBudgetExceeded concern), not a rewording.
MAX_LENGTH_RATIO = 4.0
MIN_LENGTH_RATIO = 0.25

TEMPLATE_BANK_VERSION = 1

# The city follow missions name their subject on the launcher's command line
# (scripts/run_citylife_follow.ps1: the default `$Object = "a person"`, and the
# documented car mission `-Object "a red car"`). demo/follow_vlm.py takes only
# the noun phrase; the task sentence is formed here as "follow <object>" so a
# text-reading pilot has a sentence to read.
CITY_FOLLOW_OBJECTS = ("a person", "a red car")


class ParaphraserError(Exception):
    """Base class: every failure here is loud."""


class ProtectedTextError(ParaphraserError):
    """The text must not be reworded (AerialVLA's fine-tuned prompt)."""


class StoreIntegrityError(ParaphraserError):
    """A stored set does not match the ids and hashes it was issued with."""


class UnknownSource(ParaphraserError, KeyError):
    """The stored backend has no set for this source."""


class BackendUnavailable(ParaphraserError):
    """The backend cannot produce text in this build (no client, no network)."""


class InsufficientParaphrases(ParaphraserError):
    """Fewer valid paraphrases than requested. Never a silently short list."""


# =========================================================================== #
# Ids
# =========================================================================== #

def normalise_source(text: str) -> str:
    """Whitespace-collapsed source. The id must not move because a renderer
    emitted two spaces where it used to emit one."""
    return " ".join(str(text).split())


def _sha256(s: str) -> str:
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def make_source_id(text: str) -> str:
    return "src-" + _sha256(normalise_source(text))[:16]


def rule_set_key(text: str) -> tuple:
    """The sentences of a rule text as a sorted tuple.

    The Prefix Compiler emits one sentence per rule. `build_prompt` orders
    them by rule type; `compile_csp` orders them by risk (Prefix Compiler PDF
    p4), and its order moves with the mission and the clock. The rules are a
    conjunction, so two renderings with the same sentences state the same
    rule set, and one stored paraphrase set serves both. Used only for rule
    text (store kind "csp_nl"): a mission's sentences are steps, and their
    order is part of the mission."""
    return tuple(sorted(s for s in re.split(r"(?<=[.!?])\s+", normalise_source(text)) if s))


def make_paraphrase_id(source_text: str, backend: str, index: int, text: str) -> str:
    """Stable hash of (source text, backend, index, paraphrase text).

    `index` is the paraphrase's position in the backend's own deterministic
    set for this source - not its position in one draw - so the same
    paraphrase carries the same id whatever seed or n selected it. The text IS
    in the key: a per-paraphrase KPI groups trials by this id, and a stored
    text that is re-authored is a different wording, so it must not inherit
    the trials of the old one. (Until 2026-10-06 the text was left out, and ten
    re-authored texts kept their old ids; the reviewer caught it. No trial had
    been logged under the old scheme.)
    """
    if not isinstance(index, int) or isinstance(index, bool) or index < 0:
        raise ValueError(f"index must be a non-negative int, got {index!r}")
    if not backend or "\x1f" in backend:
        raise ValueError(f"bad backend label {backend!r}")
    if not isinstance(text, str) or not text.strip():
        raise ValueError("a paraphrase id needs the paraphrase text")
    key = "\x1f".join((normalise_source(source_text), backend, str(index),
                       normalise_source(text)))
    return "pp-" + _sha256(key)[:16]


@dataclass(frozen=True)
class Paraphrase:
    paraphrase_id: str
    text: str
    backend: str
    provenance: dict
    index: int
    seed: int | None
    source_id: str
    source_text: str
    text_sha256: str
    novelty: float | None = None

    def to_record(self) -> dict:
        """What a flight log or trial row should carry: enough to trace the
        exact words the pilot read back to the set and draw they came from."""
        return {
            "paraphrase_id": self.paraphrase_id,
            "source_id": self.source_id,
            "backend": self.backend,
            "index": self.index,
            "seed": self.seed,
            "text": self.text,
            "text_sha256": self.text_sha256,
            "source_sha256": _sha256(normalise_source(self.source_text)),
        }

    def manifest_extras(self) -> dict:
        """Flat keys for a run record beside the six-field manifest. The
        manifest itself stays six fields (guardrail/manifest.py); these belong
        in metrics.json / a trial row, the way hil_evidence does.

        A per-paraphrase KPI groups by `paraphrase_id`. The id already keys on
        the text, so two wordings never share one; `paraphrase_text_sha256`
        travels beside it so a row can be checked against the store without
        re-deriving the id."""
        return {
            "paraphrase_id": self.paraphrase_id,
            "paraphrase_backend": self.backend,
            "paraphrase_index": self.index,
            "paraphrase_seed": self.seed,
            "paraphrase_source_id": self.source_id,
            "paraphrase_text_sha256": self.text_sha256,
        }


def canonical(text: str) -> Paraphrase:
    """The canonical wording as a trial arm: the per-paraphrase KPI's baseline.

    Labelled backend "identity" so it can never be counted as a paraphrase;
    `validate()` refuses it as one."""
    src = normalise_source(text)
    return Paraphrase(paraphrase_id=make_paraphrase_id(src, "identity", 0, src),
                      text=src, backend="identity",
                      provenance={"generator": "identity (canonical arm)"},
                      index=0, seed=None, source_id=make_source_id(src),
                      source_text=src, text_sha256=_sha256(src), novelty=0.0)


# =========================================================================== #
# Protected text
# =========================================================================== #

def protected_reason(text: str) -> str | None:
    """Why `text` may not be reworded, or None.

    AerialVLA's prompt is built in exactly one place (demo/aerialvla_demo.py
    build_prompt) as "<image>\\nFly {direction}and find the target.
    {object}\\nAction: ". The {direction} phrase is the slot the LoRA was
    fine-tuned on and the only one measured to steer (108 forward passes,
    docs/FINDING-what-drives-aerialvla.md). guardrail cannot import demo/, so
    the guard keys on the prompt's fixed frame, which is part of the trained
    shape too."""
    if "<image>" in text or re.search(r"\naction:\s*$", text, re.IGNORECASE):
        return ("protected: AerialVLA prompt - its {direction} phrase is a "
                "fine-tuned vocabulary and an unseen wording has undefined effect "
                "(demo/vla_bridge.py:30-32, docs/FINDING-what-drives-aerialvla.md)")
    return None


# =========================================================================== #
# Slot extraction
# =========================================================================== #

_NUMWORD = (r"(?:(?:twenty|thirty|forty|fifty|sixty|seventy|eighty|ninety)"
            r"(?:[- ](?:one|two|three|four|five|six|seven|eight|nine)\b)?"
            r"|zero|one|two|three|four|five|six|seven|eight|nine|ten|eleven|"
            r"twelve|thirteen|fourteen|fifteen|sixteen|seventeen|eighteen|"
            r"nineteen)\b(?:\s+and\s+a\s+half\b)?")
_DIGITS = r"\d+(?:\.\d+)?"
_NUM = rf"(?:{_DIGITS}|{_NUMWORD})"

_SMALL = {w: i for i, w in enumerate(
    "zero one two three four five six seven eight nine ten eleven twelve "
    "thirteen fourteen fifteen sixteen seventeen eighteen nineteen".split())}
_TENS = {"twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60,
         "seventy": 70, "eighty": 80, "ninety": 90}

# Canonical unit -> surface pattern. Order matters: m/s before m, deg/s before
# deg. A unit changed (m/s -> km/h, m -> ft) is refused, not converted: a pilot
# reading "14.4 km/h" is a different robustness test from one reading "4 m/s".
_UNITS = [
    ("m/s", r"m\s*/\s*s(?:ec(?:ond)?)?\b|mps\b|met(?:er|re)s?[\s-]*"
            r"(?:/\s*s(?:ec(?:ond)?)?\b|(?:per|a|an|each|every)[\s-]+second\b)"),
    ("km/h", r"km\s*/\s*h(?:r|our)?\b|kph\b|kmh\b|kilomet(?:er|re)s?[\s-]+"
             r"(?:per|an|a|each|every)[\s-]+hour\b"),
    ("mph", r"mph\b|miles?[\s-]+(?:per|an|a)[\s-]+hour\b"),
    ("deg/s", r"(?:deg(?:ree)?s?|°)\s*/\s*s(?:ec(?:ond)?)?\b|(?:deg(?:ree)?s?|°)"
              r"[\s-]*(?:per|a|an|each|every)[\s-]+second\b|dps\b"),
    ("km", r"km\b|kilomet(?:er|re)s?\b"),
    ("m", r"m\b|met(?:er|re)s?\b"),
    ("ft", r"ft\b|feet\b|foot\b"),
    ("deg", r"deg(?:ree)?s?\b|°"),
    ("s", r"s\b|secs?\b|seconds?\b"),
    ("%", r"%|per\s*cent\b|percent\b"),
]
_UNIT_ALT = "|".join(f"(?P<u{i}>{p})" for i, (_, p) in enumerate(_UNITS))
_UNIT_ANY = "|".join(f"(?:{p})" for _, p in _UNITS)

_QUANT_RE = re.compile(rf"(?<![\w.])(?P<num>{_NUM})(?:[\s-]*(?:{_UNIT_ALT}))?")
# A hyphen joins a range only between digits: "forty-five degrees" is one
# number, and a regex allowed to backtrack reads it as the range 40-5.
_RANGE_A = re.compile(
    rf"\b(?:between|from)\s+(?P<a>{_NUM})(?:[\s-]*(?P<ua>{_UNIT_ANY}))?\s+"
    rf"(?:and|to|up\s+to|through|until)\s+(?P<b>{_NUM})[\s-]*(?P<ub>{_UNIT_ANY})")
_RANGE_B = re.compile(
    rf"(?<![\w.])(?P<a>{_NUM})(?:[\s-]*(?P<ua>{_UNIT_ANY}))?\s+to\s+"
    rf"(?P<b>{_NUM})[\s-]*(?P<ub>{_UNIT_ANY})")
_RANGE_C = re.compile(
    rf"(?<![\w.])(?P<a>{_DIGITS})(?:\s*(?P<ua>{_UNIT_ANY}))?\s*-\s*"
    rf"(?P<b>{_DIGITS})[\s-]*(?P<ub>{_UNIT_ANY})")
_COORD_PAREN = re.compile(r"\(\s*(?P<x>-?\d+(?:\.\d+)?)\s*,\s*(?P<y>-?\d+(?:\.\d+)?)\s*\)")
_COORD_BARE = re.compile(
    r"\b(?P<pre>to|at|point|coordinates?|position|location|spot|waypoint)\s+"
    r"(?P<x>-?\d+(?:\.\d+)?)\s*,\s*(?P<y>-?\d+(?:\.\d+)?)(?![\w.])")
_QUOTED_ID = re.compile(r"(?<![a-z0-9])['\"](?P<id>[a-z0-9][a-z0-9_\-]*[a-z0-9])['\"](?![a-z0-9])")

# Object classes. Surface forms inside one class are the same slot; anything
# else is a different object. `followed` is the wildcard subject of a
# stand-off rule ("anything you are following"), and must be matched before
# the follow ACTION so the rule text does not read as a mission verb.
_FOLLOW_V = (r"(?:following|tracking|tailing|pursuing|shadowing|chasing|follow|"
             r"track|tail|pursue|shadow|chase)")
_FOLLOWER = (r"(?:that\s+|which\s+|who\s+|whom\s+)?(?:you|it|we|the\s+drone|"
             rf"the\s+aircraft)(?:'re|'s|\s+are|\s+is)?\s+(?:currently\s+|now\s+)?"
             rf"{_FOLLOW_V}\b")
_FOLLOWED_AS = r"(?:followed|tracked|tailed|pursued)"
_OBJECTS = [
    # The stand-off rule's subject is a wildcard: the compiler renders
    # `subject_class: "*"` as "anything you are following"
    # (templates/sentence/subject_standoff.j2), and the Shield binds it to
    # whatever the pilot is following (guardrail/models.py
    # SubjectStandoff.binds). "The car you are following" narrows it to cars -
    # right on a car mission, wrong the day the same policy is flown behind a
    # person - so a named class is its own slot, matched first, and only a
    # class-free phrase is the wildcard.
    ("followed_person",
     rf"(?:anyone|whoever|the\s+(?:person|pedestrian|individual|human|man|woman))"
     rf"\s+{_FOLLOWER}|{_FOLLOWED_AS}\s+(?:person|pedestrian|individual)\b"),
    ("followed_car",
     rf"the\s+(?:car|automobile)\s+{_FOLLOWER}|{_FOLLOWED_AS}\s+(?:car|automobile)\b"),
    ("followed_vehicle",
     rf"the\s+vehicle\s+{_FOLLOWER}|{_FOLLOWED_AS}\s+vehicle\b"),
    ("followed",
     rf"(?:anything|whatever|everything|what|whichever\s+\w+|"
     rf"the\s+(?:thing|object|one|subject|target))\s+{_FOLLOWER}"
     rf"|(?:the\s+)?(?:target|subject)\s+(?:being|you're|you\s+are)\s+"
     rf"(?:followed|tracked|tailed|pursued|following|tracking)\b"
     rf"|{_FOLLOWED_AS}\s+(?:target|subject|object|thing)\b"
     rf"|(?:(?:your|the)\s+)?(?:current\s+)?(?:target|subject|quarry)\b"),
    ("restricted_area",
     r"(?:restricted|no[- ]fly|off[- ]limits|prohibited|forbidden|exclusion|"
     r"banned|no[- ]go)\s+(?:air\s*space|area|zone|region|space|sector)s?\b"
     r"|restricted\s+airspace\b"),
    ("pedestrian",
     r"pedestrians?\b|(?:people|persons|anyone|someone|somebody|anybody|nobody|"
     r"person)\s+(?:on\s+foot|walking)\b|walkers?\b"),
    ("person", r"persons?\b|people\b|individuals?\b|humans?\b"),
    ("building", r"buildings?\b|structures?\b"),
    ("car", r"cars?\b|automobiles?\b"),
    ("vehicle", r"vehicles?\b"),
    ("truck", r"trucks?\b|lorr(?:y|ies)\b"),
    ("bus", r"bus(?:es)?\b"),
    ("bicycle", r"bicycles?\b|bikes?\b|cyclists?\b"),
    ("motorcycle", r"motorcycles?\b|motorbikes?\b|scooters?\b"),
    ("barrier", r"barriers?\b"),
    ("waypoint", r"way\s*-?\s*points?\b"),
    ("tower", r"towers?\b"),
    ("tree", r"trees?\b"),
    ("animal", r"dogs?\b|cats?\b|animals?\b"),
]
_OBJECT_RES = [(c, re.compile(rf"\b(?:{p})")) for c, p in _OBJECTS]

# Named values with no number: "at cruise altitude" sets the altitude as
# surely as "at 15 m" does, and a paraphrase that drops it drops that.
_REFS = [("cruise_altitude", r"cruis(?:e|ing)\s+(?:altitude|height|level)")]
_REF_RES = [(n, re.compile(rf"\b(?:{p})\b")) for n, p in _REFS]

_COLOURS = re.compile(r"\b(red|orange|yellow|green|blue|purple|violet|pink|white|"
                      r"black|gr[ae]y|silver|brown|gold|golden|beige|maroon|"
                      r"crimson|scarlet|navy|teal|cyan|magenta)\b")
_COLOUR_CANON = {"gray": "grey", "golden": "gold"}

# Phrases that LOOK like a direction or an action and are neither.
_NEUTRAL = re.compile(r"\b(?:go\s+ahead\s+and|ahead\s+of\s+(?:time|schedule)|"
                      r"right\s+(?:away|now|here|there)|all\s+right|"
                      # "stay behind a red car": where to be relative to the
                      # object, not a heading for the flight
                      r"behind(?=\s+(?:a|an|the|it|them|him|her)\b|\s+⟦)|"
                      r"straight\s*away|straight(?=\s+(?:to|for|towards?|over|at|"
                      r"there|back)\b))\b")
_DIRECTIONS = [
    ("northeast", r"north[- ]?east(?:ern|ward)?"),
    ("northwest", r"north[- ]?west(?:ern|ward)?"),
    ("southeast", r"south[- ]?east(?:ern|ward)?"),
    ("southwest", r"south[- ]?west(?:ern|ward)?"),
    ("north", r"north(?:ern|wards?)?"), ("south", r"south(?:ern|wards?)?"),
    ("east", r"east(?:ern|wards?)?"), ("west", r"west(?:ern|wards?)?"),
    ("counterclockwise", r"counter[- ]?clockwise|anti[- ]?clockwise"),
    ("clockwise", r"clockwise"),
    ("forward", r"straight\s+(?:ahead|on|forwards?)|forwards?|ahead|onwards?|straight|"
                r"in\s+front(?:\s+of\s+(?:you|the\s+(?:drone|aircraft)|us))?"),
    ("backward", r"backwards?|rearwards?|in\s+reverse|behind"),
    ("sideways", r"sideways|laterally"),
    ("left", r"left(?:wards?)?"), ("right", r"right(?:wards?)?"),
]
_DIRECTION_RES = [(c, re.compile(rf"\b(?:{p})\b")) for c, p in _DIRECTIONS]

_ACTIONS = [
    # "fly at 6 m/s to the pad" is the same goto as "fly to the pad at 6 m/s":
    # a quantity (already a placeholder here) may sit between verb and "to".
    ("goto",
     r"(?:fly|go|head|travel|proceed|move|make\s+(?:your|its)\s+way|get|navigate|"
     r"cruise|transit|take\s+(?:yourself|the\s+(?:drone|aircraft))|"
     r"bring\s+(?:yourself|the\s+(?:drone|aircraft)))(?:\s+(?:(?:over|on|across|"
     r"directly)\s+)?(?:to|towards?|for)\b|(?=\s+(?:at\s+)?⟦q\d+⟧\s+(?:(?:over|on|"
     r"across|directly)\s+)?(?:to|towards?|for)\b))|\breach(?:es|ing)?\b|"
     r"\barrive\s+at\b|\bdestination\b"),
    ("stop", r"stop(?:s|ped|ping)?\b|halt(?:s|ed|ing)?\b"),
    ("follow",
     r"follow(?:s|ing)?\b|track(?:s|ing)?\b|tail(?:s|ing)?\b|shadow(?:s|ing)?\b|"
     r"pursu(?:e|es|ing)\b|chas(?:e|es|ing)\b|keep\s+(?:up|pace)\s+with\b|"
     r"stay\s+with\b|go\s+after\b|"
     # "keep the red car in sight", "keep them in view": following, as far as
     # a negation of it is concerned ("do not keep them in view")
     r"keep(?:ing)?(?=\s+(?:\S+\s+){1,3}in\s+(?:view|sight)\b)"),
    ("land", r"land(?:s|ing)?\b|touch\s+down\b"),
    ("search", r"find\b|search\w*|look\s+for\b|locate\b|seek\b"),
    ("orbit", r"orbit\w*|circl(?:e|es|ing)\b"),
    ("hover", r"hover\w*|loiter\w*"),
    ("return", r"return\w*|come\s+back\b|rtl\b"),
]
_ACTION_RES = [(c, re.compile(rf"\b(?:{p})")) for c, p in _ACTIONS]

# Comparative cues, as token tuples. A cue is read through a negation within
# five tokens before it: "no faster than" = max, "never exceed" = max, "no
# closer than" = min. Compounds that already contain their negator ("no less
# than") are NOT listed, or the negation would be applied twice.
_MIN_CUES = [("at", "least"), ("above",), ("over",), ("past",), ("more", "than"),
             ("greater", "than"), ("higher", "than"), ("farther", "than"),
             ("further", "than"), ("beyond",), ("exceed",), ("exceeds",),
             ("exceeding",), ("faster", "than"), ("quicker", "than"),
             ("minimum",), ("min",), ("floor",), ("≥",), (">=",), (">",),
             ("upwards", "of"), ("in", "excess", "of")]
_MAX_CUES = [("at", "most"), ("below",), ("under",), ("beneath",), ("less", "than"),
             ("lower", "than"), ("slower", "than"), ("closer", "than"),
             ("nearer", "than"), ("up", "to"), ("maximum",), ("max",), ("within",),
             ("cap",), ("capped",), ("caps",), ("limit",), ("limited",),
             ("limits",), ("ceiling",), ("top",), ("tops",), ("≤",), ("<=",),
             ("<",)]
_MIN_AFTER = [("or", "more"), ("or", "higher"), ("or", "above"), ("or", "greater"),
              ("or", "further"), ("or", "farther"), ("minimum",), ("min",),
              ("at", "minimum"), ("at", "least"), ("at", "the", "least"),
              ("at", "the", "very", "least"), ("away",), ("clear",), ("back",),
              ("off",)]
_MAX_AFTER = [("or", "less"), ("or", "lower"), ("or", "below"), ("or", "under"),
              ("or", "slower"), ("or", "fewer"), ("max",), ("maximum",),
              ("at", "most"), ("at", "the", "most"), ("at", "maximum"),
              ("at", "the", "maximum"), ("tops",), ("top",)]
_NOUN_MIN = {"buffer", "gap", "margin", "berth", "cushion", "clearance",
             "clearances", "separation", "standoff", "spacing", "distance"}
_VERTICAL_CUES = {"above", "below", "over", "under", "beneath"}
_WEAK_CUES = {("above",), ("over",), ("past",), ("beyond",), ("below",), ("under",),
              ("beneath",), ("within",), ("up", "to"), ("top",), ("tops",),
              ("≥",), (">=",), (">",), ("≤",), ("<=",), ("<",)}
_OR_CUE_TAILS = {"less", "more", "lower", "higher", "below", "above", "under",
                 "greater", "further", "farther", "slower", "fewer"}
_DISTANCE_NOUNS = {"distance", "away", "apart", "separation", "gap", "spacing"}
_ALT_NOUN = re.compile(r"\b(?:altitude\w*|alt|height\w*|elevation|agl|ceiling|floor)\b")
_DISTANCE_WORDS = {"distance", "away", "apart", "separation", "gap", "clearance",
                   "spacing", "clear", "from", "buffer", "margin", "berth"}
_NEGATORS = {"not", "never", "no", "nor", "without", "cannot"}
_RESET = {"stay", "stays", "keep", "keeps", "remain", "remains", "hold", "holds",
          "maintain", "be", "is", "are", "must", "shall", "should", "always",
          "fly", "cruise", "go", "ensure", "make", "give", "leave", "leaving"}
_ARTICLES = {"a", "an", "the", "any", "every", "all", "its", "your", "each"}
_SEPS = {",", "or", "nor", "and"}

# What a quantity limits. Filtered by unit: an altitude word next to a m/s
# number says nothing about that number.
_ATTRS = {
    "speed": r"\b(?:speeds?|fast|faster|fastest|velocity|airspeed|groundspeed|"
             r"pace|quick|quicker|slow|slower|slowly)\b",
    "climb": r"\b(?:climb\w*|ascen\w*|descen\w*|vertical\w*|rise|rises|rising|sink\w*)\b",
    "turn": r"\b(?:turn\w*|yaw\w*|rotat\w*|spin\w*|heading)\b",
    "altitude": r"\b(?:altitude\w*|alt|height\w*|high|higher|highest|elevation|agl|"
                r"ground|low|lower|lowest|up|ceiling|floor)\b",
    "lateral": r"\b(?:cent(?:er|re)\s*-?\s*line|middle|axis|either\s+side|"
               r"each\s+side|sideways|off[- ]cent(?:er|re))\b",
}
_ATTR_RES = {k: re.compile(v) for k, v in _ATTRS.items()}
_SPEED_NOUN = re.compile(r"\b(?:speeds?|velocity|airspeed|groundspeed|pace)\b")
_UNIT_TAGS = {
    "m": {"altitude", "lateral", "offset"}, "ft": {"altitude", "lateral", "offset"},
    "km": {"altitude", "lateral", "offset"},
    "m/s": {"speed", "climb"}, "km/h": {"speed", "climb"}, "mph": {"speed", "climb"},
    "deg/s": {"turn"}, "deg": {"turn"}, "s": set(), "%": set(),
}

# Zone polarity cues (per clause). Three kinds of word decide whether a clause
# keeps the vehicle OUT of the zone it names:
#   modal  - a word that is itself a ruling: "avoid", "off-limits", "keep out
#            of", "prohibited" (keep out) or "allowed", "may", "can" (permit);
#   entry  - a word for going in: "enter", "into", "cross", "go"; it rules
#            nothing alone, and keeps out only when negated ("never enter");
#   negator - flips the next ruling word within a short window, once.
# When a clause has a modal word, the modal words decide and the entry words
# are their complement ("entry is prohibited", "not forbidden to enter"); only
# a clause with no modal word is decided by its entry words. Until 2026-10-06
# ANY negator or prohibition word made a clause keep-out, so "It is not
# forbidden to enter zone 'nfz-square'" and "do not avoid restricted areas"
# passed as keep-out rules (independent review, 2026-10-06).
_PROHIBIT = [("avoid",), ("avoids",), ("avoiding",), ("avoided",),
             ("off", "-", "limits"), ("off", "limits"), ("forbidden",),
             ("prohibited",), ("banned",), ("barred",), ("closed",),
             ("disallowed",), ("restricted",), ("outside",), ("out", "of"),
             ("clear", "of"), ("away", "from"), ("around",), ("wide", "berth"),
             ("no", "-", "go"), ("bypass",), ("skirt",), ("steer", "clear"),
             ("keep", "out", "of"), ("stay", "out", "of"), ("keep", "out"),
             ("stay", "out")]
_PERMIT = [("allowed",), ("permitted",), ("may",), ("can",), ("free", "to"),
           ("ok",), ("okay",), ("fine",), ("acceptable",), ("welcome", "to")]
_ENTRY = {"enter", "enters", "entering", "entered", "entry", "into", "cross",
          "crosses", "crossing", "crossed", "penetrate", "go", "goes", "going",
          "fly", "flies", "flying", "be", "venture", "stray", "wander", "near",
          "approach", "approaching", "inside"}
# "may not", "can not": the modal is the negation, not a permission.
_NEG_MODALS = {("may", "not"), ("can", "not"), ("might", "not")}
# A clause that says "inside or near corridor X" no longer confines.
_WIDEN = {"near", "nearby", "around", "alongside", "beside", "close"}
# Verbs that end an action: "stop following a red car" negates the follow.
_STOP_VERBS = {"stop", "stops", "cease", "ceases", "quit", "quits", "halt",
               "abandon", "abandons"}
_KEEP_IN = [("inside",), ("within",), ("stay", "in"), ("stays", "in"),
            ("remain", "in"), ("remains", "in"), ("keep", "to"), ("keep", "in"),
            ("stick", "to"), ("confined",), ("in", "corridor"),
            ("in", "the", "corridor")]
_EXIT_ALWAYS = {"leave", "leaves", "leaving", "exit", "exits", "exiting", "depart",
                "departing", "outside"}
_EXIT_IF_OUT = {"stray", "straying", "wander", "wandering", "drift", "drifting"}

# Schedules. The compiler renders a time-windowed rule as "Never enter zone
# 'nfz-school' (in force Mon-Fri 07:30-17:30)." (templates/sentence/_base.j2,
# the WP2-05 fix). The days and times are slots, and they belong to the rule
# they were written on: moving a window to another rule, or inverting it with
# "except", changes when the rule binds.
_DAY_ORDER = ["mon", "tue", "wed", "thu", "fri", "sat", "sun"]
_DAY = (r"(?:mon(?:day)?|tue(?:s(?:day)?)?|wed(?:nesday)?|thu(?:r(?:s(?:day)?)?)?|"
        r"fri(?:day)?|sat(?:urday)?|sun(?:day)?)")
_DAY_RANGE = re.compile(rf"\b(?P<a>{_DAY})s?\s*(?:-|to|through|thru|until|till)\s*"
                        rf"(?P<b>{_DAY})s?\b")
_DAY_ONE = re.compile(rf"\b(?P<a>{_DAY})s?\b|\b(?P<grp>weekdays?|weekends?)\b")
_TIME = re.compile(r"\b(?P<h>[01]?\d|2[0-3])(?::(?P<m>[0-5]\d))?\s*(?P<ap>a\.?m\.?|p\.?m\.?)"
                   r"(?![a-z])|\b(?P<h2>[01]?\d|2[0-3]):(?P<m2>[0-5]\d)\b")
_INVERT = re.compile(r"\b(?:except|excluding|other\s+than|apart\s+from|but\s+not)\b")

# Softeners. A hard rule reworded as "try to stay below 4 m/s" or "ideally
# above 10 m" is a different rule: the Shield will still enforce it, but the
# pilot was told it was optional. Politeness ("please", "could you") is not a
# softener; a hedge on the limit is.
_HEDGE = re.compile(
    r"\b(?:try\s+(?:to|and)|if\s+(?:at\s+all\s+)?possible|ideal(?:ly)?|"
    r"preferabl[ey]|prefer(?:red|ably)?|where(?:ver)?\s+(?:possible|practical|feasible)|"
    r"when(?:ever)?\s+(?:possible|practical|feasible|convenient)|if\s+convenient|"
    r"if\s+you\s+can|as\s+(?:much|far)\s+as\s+(?:you\s+can|possible)|aim\s+(?:to|for)|"
    r"attempt\s+to|approximately|roughly|or\s+so|more\s+or\s+less|"
    r"(?:it\s+is|it's)\s+best|would\s+be\s+(?:good|great|nice|best)|"
    # added 2026-10-06 after the independent review: tendencies and wishes
    # are not rules ("Generally hold altitude ...", "you'll want to stay ...")
    r"generally|usually|normally|typically|mostly|in\s+general|as\s+a\s+rule|"
    r"(?:you(?:'ll|\s+will|\s+may|\s+might)?|you'd)\s+want\s+to|"
    r"should\s+try|do\s+your\s+best|best\s+effort|if\s+needed|if\s+necessary|"
    r"as\s+needed|within\s+reason|reasonably|recommended|suggested|encouraged|"
    r"(?:about|around)(?=\s+(?:\d|one|two|three|four|five|six|seven|eight|nine|ten)))\b")
_SOFT = re.compile(r"\bsoft(?:\s+limit)?\b|\badvisory\b")

# Conditions, exceptions and scope limits. The compiler's rules hold at all
# times (or inside the window written on them), so a paraphrase that makes one
# apply only "if", "unless", "when", "while climbing" or "otherwise" states a
# different rule - and "Otherwise, remain in corridor ..." made the corridor,
# the band and all three kinematic limits conditional in one stored paraphrase
# that the validator accepted (independent review, 2026-10-06). Exceptions:
#   * a condition that restates the rule's own window ("while it is in force",
#     "until 17:30") is allowed when the source has a window;
#   * "only inside / within / in" strengthens a stay-inside rule;
#   * "while avoiding ...", "while climbing at under 2 m/s" coordinate two
#     instructions; only "while <a flight phase>" with no limit after it
#     narrows one ("While climbing, max speed 4 m/s");
#   * "until you reach ..." is the goal of a goto;
#   * "during this flight" / "throughout the mission" is always.
_CONDITIONAL = re.compile(
    r"\b(?:unless|otherwise|except|excluding|apart\s+from|other\s+than|but\s+not|"
    r"if|whenever|when|once|as\s+long\s+as|so\s+long\s+as|provided|in\s+case|"
    r"only|until|till|while|whilst|during)\b")
# The window must be named inside the condition's own clause (no comma
# between): "While climbing, from Monday to Friday, ..." is a flight-phase
# condition that happens to sit before a window, not a restatement of it.
_SCHED_CTX = re.compile(
    r"^[^,.;:!?]{0,40}?\b(?:in\s+force|in\s+effect|active|window|hours|weekdays?|"
    r"weekends?|mon|tue|wed|thu|fri|sat|sun|monday|tuesday|wednesday|thursday|"
    r"friday|saturday|sunday|\d{1,2}(?::\d\d|\s*[ap]\.?m))")
_PHASE = re.compile(
    r"^\s+(?:you(?:'re|\s+are)\s+|it(?:'s|\s+is)\s+|the\s+(?:drone|aircraft)\s+is\s+)?"
    r"(?:climb|descend|ascend|turn|cruis|hover|approach|pass|land|tak|follow|track|"
    r"near|close|over|above|below|in\b|inside|outside|within|heading|flying)\w*"
    r"(?!\w*\s+(?:at|under|below|above|over|slower|faster|less|more|no|⟦|\d))")
_ALWAYS = re.compile(r"^\s+(?:this|the|your|our)\s+(?:whole\s+|entire\s+)?"
                     r"(?:flight|mission|trip|survey|operation|run)\b")
# Words that suspend a rule rather than restate it.
_OVERRIDE = re.compile(
    r"\b(?:ignor\w*|disregard\w*|overrid\w*|waiv\w*|suspend\w*|relax\w*|exempt\w*|"
    r"optional|not\s+required|no\s+need|need\s+not|needn't|(?:do\s+not|don't|"
    r"doesn't|does\s+not)\s+have\s+to|temporarily|briefly|momentarily|"
    r"for\s+a\s+(?:moment|while|bit))\b")

_TOKEN_RE = re.compile(r"⟦[^⟧]*⟧|[a-z]+(?:'[a-z]+)*|≤|≥|<=|>=|<|>|[.,;:!?()\-]|[^\s]")
_SENT_BREAK = {".", "!", "?", ";"}
_CHUNK_BREAK = _SENT_BREAK | {",", "and", "but", "while", "whilst", "plus"}


def _norm(text: str) -> str:
    t = unicodedata.normalize("NFKC", str(text))
    t = (t.replace("’", "'").replace("‘", "'")
          .replace("“", '"').replace("”", '"'))
    t = t.replace("–", "-").replace("—", " - ")
    t = t.lower()
    # A spaced dash is a clause break; an unspaced one joins a word
    # ("off-limits", "north-east"), so only the former becomes a comma.
    t = re.sub(r"\s+-+\s+", " , ", t)
    return " ".join(t.split())


def _num_value(s: str) -> float:
    s = s.strip()
    if re.fullmatch(_DIGITS, s):
        return float(s)
    half = 0.0
    m = re.match(r"(.*?)\s+and\s+a\s+half$", s)
    if m:
        s, half = m.group(1), 0.5
    parts = re.split(r"[- ]", s)
    if parts[0] in _TENS:
        v = _TENS[parts[0]] + (_SMALL[parts[1]] if len(parts) > 1 else 0)
    else:
        v = _SMALL[parts[0]]
    return float(v) + half


def _unit_of(match: re.Match) -> str | None:
    for i, (canon, _) in enumerate(_UNITS):
        if match.group(f"u{i}"):
            return canon
    return None


def _unit_canon(surface: str | None) -> str | None:
    if not surface:
        return None
    for canon, pat in _UNITS:
        if re.fullmatch(rf"(?:{pat})", surface.strip()):
            return canon
    return None


def _fmt(v: float) -> str:
    return f"{v:g}"


@dataclass(frozen=True)
class Quantity:
    value: float
    unit: str | None
    bound: str                 # "min" | "max" | "target"
    tags: frozenset
    negated: bool = False

    def describe(self) -> str:
        u = f" {self.unit}" if self.unit else ""
        t = f", {'/'.join(sorted(self.tags))}" if self.tags else ""
        return f"{_fmt(self.value)}{u} ({self.bound}{t})"


@dataclass
class Slots:
    ids: dict = field(default_factory=dict)            # id -> [chunk polarity]
    id_sents: set = field(default_factory=set)         # sentences naming an id
    places: list = field(default_factory=list)         # (name, "dest" | "relative")
    coords: list = field(default_factory=list)
    quantities: list = field(default_factory=list)
    objects: dict = field(default_factory=dict)        # class -> [(polarity, sentence ids)]
    colours: set = field(default_factory=set)
    colour_binds: set = field(default_factory=set)     # (colour, object class | None)
    widened: set = field(default_factory=set)          # classes named with "any"/"a" and no colour
    directions: set = field(default_factory=set)
    neg_directions: set = field(default_factory=set)
    actions: set = field(default_factory=set)
    neg_actions: set = field(default_factory=set)
    schedule: list = field(default_factory=list)       # (kind, value, ids in sentence)
    refs: set = field(default_factory=set)             # named references ("cruise altitude")
    neg_refs: set = field(default_factory=set)
    inverted_schedule: bool = False
    hedges: list = field(default_factory=list)
    conditions: list = field(default_factory=list)     # "if", "unless", "sched:while", ...
    overrides: list = field(default_factory=list)      # "ignore", "optional", ...
    soft: int = 0
    words: list = field(default_factory=list)          # for identity / novelty


_PLACES_FALLBACK = ["northeast pad", "north pad", "east pad", "home"]
_places_cache: list[str] | None = None
PLACES_SOURCE = "unread"        # "guardrail.compiler.PLACES" | "fallback: <why>"


def _places() -> list[str]:
    """The compiler's named-place registry, longest first. Imported lazily so
    the module stays usable for slot checks without the policy stack.

    If the import fails the validator still runs on a fallback list - but it
    says so (WARNING, and PLACES_SOURCE, which the validation report records),
    because a place registry that silently diverged from the compiler's would
    let a renamed place through."""
    global _places_cache, PLACES_SOURCE
    if _places_cache is None:
        try:
            from .compiler import PLACES
            names = list(PLACES)
            PLACES_SOURCE = "guardrail.compiler.PLACES"
        except Exception as e:                         # noqa: BLE001
            names = list(_PLACES_FALLBACK)
            PLACES_SOURCE = f"fallback: {type(e).__name__}: {e}"[:200]
            log.warning("paraphraser: could not import guardrail.compiler.PLACES "
                        "(%s); place slots use the fallback list %s",
                        e, _PLACES_FALLBACK)
        _places_cache = sorted(names, key=len, reverse=True)
    return _places_cache


_PLACE_RELATIVE = {"from", "near", "past", "beside", "beyond", "around", "behind",
                   "away", "next", "opposite", "avoid", "avoiding", "nearby", "by",
                   "alongside", "outside"}


def _conditions(t: str) -> list[str]:
    """Conditional / exception / scope words in normalised text `t`; the ones
    that only restate a window are prefixed "sched:" (see _CONDITIONAL)."""
    out = []
    for m in _CONDITIONAL.finditer(t):
        w = re.sub(r"\s+", " ", m.group(0))
        rest = t[m.end():]
        if (w in ("while", "whilst", "when", "whenever", "during", "until", "till",
                  "if", "once", "only") and _SCHED_CTX.match(rest)):
            out.append("sched:" + w)
            continue
        if w == "only" and re.match(r"\s+(?:inside|within|in)\b", rest):
            continue
        if w in ("while", "whilst") and not _PHASE.match(rest):
            continue
        if w in ("until", "till") and re.match(
                r"\s+(?:you\s+|it\s+)?(?:reach|arriv|get\s+(?:there|to))", rest):
            continue
        if w == "during" and _ALWAYS.match(rest):
            continue
        out.append(w)
    return out


def _match_at(tokens: list[str], i: int, cue: tuple) -> bool:
    return tuple(tokens[i:i + len(cue)]) == cue


def _find_cues(tokens: list[str], cues: list[tuple]) -> list[tuple[int, int, tuple]]:
    out = []
    for i in range(len(tokens)):
        for c in cues:
            if _match_at(tokens, i, c):
                out.append((i, i + len(c), c))
    return out


def _is_negator(tok: str) -> bool:
    return tok in _NEGATORS or tok.endswith("n't")


def _greedy_cues(tokens: list[str], cues: list[tuple]) -> list[tuple[int, int, tuple]]:
    """Non-overlapping cue matches, longest first: "keep out of" is one ruling,
    not "keep out" plus "out of"."""
    found = sorted(_find_cues(tokens, cues), key=lambda x: (-(x[1] - x[0]), x[0]))
    taken: set[int] = set()
    out = []
    for s_, e_, c in found:
        if taken.isdisjoint(range(s_, e_)):
            taken.update(range(s_, e_))
            out.append((s_, e_, c))
    return sorted(out)


def _negators_at(toks: list[str]) -> set[int]:
    """Positions that negate: "not", "never", "no", "nor", "without", "cannot"
    and the "n't" words. ("may not" is read in _modal_reading: there the
    negator rules and the "may" is not a permission.)"""
    return {i for i, t in enumerate(toks) if _is_negator(t)}


def _modal_reading(toks: list[str]) -> tuple[set, list]:
    """Rulings in one clause: ({"avoid" | "permit"}, entry readings).

    Each negator flips the first ruling word in the five tokens after it and
    is then spent, or - when an entry word comes first - the run of entry
    words ("never fly into"), stopping before a ruling word ("never enter the
    no-go area"). Flips count by parity: "not ... not" restores. The entry
    readings are returned separately because they only count when the clause
    has no ruling word (see the cue tables)."""
    negs = _negators_at(toks)
    modal: list[tuple[int, str]] = []
    for s_, e_, _c in _greedy_cues(toks, _PROHIBIT):
        modal.append((s_, "avoid"))
        negs -= set(range(s_, e_))             # the "no" of "no-go" is the ruling
    for s_, _e, c in _find_cues(toks, _PERMIT):
        nxt = toks[s_ + 1] if s_ + 1 < len(toks) else ""
        if (c[0], nxt) in _NEG_MODALS or (c[0] in ("may", "can") and _is_negator(nxt)):
            continue                           # "may not": the negator rules, not a permit
        modal.append((s_, "permit"))
    modal.sort()
    entries = [i for i, t in enumerate(toks) if t in _ENTRY]
    targets = sorted([(i, "modal") for i, _v in modal] + [(i, "entry") for i in entries])
    flips: Counter = Counter()
    for n in sorted(negs):
        nxt = [(i, kind) for i, kind in targets if n < i <= n + 5 and i not in negs]
        if nxt and nxt[0][1] == "modal":
            flips[nxt[0][0]] += 1              # spent on the ruling word
            continue
        # otherwise it negates the going-in: every entry word after it in the
        # clause, up to the next ruling word ("is not somewhere you should not
        # ever go" - two negators, so the going-in is permitted)
        for i, kind in targets:
            if i <= n or i in negs:
                continue
            if kind == "modal":
                break
            flips[i] += 1
    rulings = set()
    for i, v in modal:
        if flips[i] % 2:
            v = "permit" if v == "avoid" else "avoid"
        rulings.add(v)
    entry_reads = ["avoid" if flips[i] % 2 else "enter" for i in entries]
    return rulings, entry_reads


def _chunk_polarity(tokens: list[str], reg: dict | None = None,
                    class_rulings: int = 0) -> dict:
    """keep-out vs keep-in (and whether the clause permits entry), for the
    clause a zone id sits in.

    `class_rulings` counts class words such as "no-fly zone" in the clause
    ("'nfz-square' is a no-fly zone"). They decide only when the clause has
    neither a ruling word nor an entry word: "The no-fly zone 'nfz-square' is
    somewhere you should go" describes the zone and then permits it."""
    toks = [t for t in tokens]

    def negated(i: int) -> bool:
        return sum(_is_negator(toks[j]) for j in range(max(0, i - 5), i)) % 2 == 1

    rulings, entry_reads = _modal_reading(toks)
    if rulings:
        prohibit = rulings == {"avoid"}
        permit = "permit" in rulings
    elif entry_reads:
        prohibit = all(r == "avoid" for r in entry_reads)
        permit = any(r == "enter" for r in entry_reads)
    else:
        prohibit, permit = class_rulings > 0, False
    pos_keep = neg_keep = False
    for i, _, _c in _find_cues(toks, _KEEP_IN):
        if negated(i):
            neg_keep = True
        else:
            pos_keep = True
    negexit = bare_exit = False
    for i, t in enumerate(toks):
        is_exit = t in _EXIT_ALWAYS or (
            t in _EXIT_IF_OUT and i + 1 < len(toks)
            and toks[i + 1] in {"out", "outside", "from", "beyond", "off"})
        if t == "out" and i + 1 < len(toks) and toks[i + 1] == "of":
            is_exit = True
        if not is_exit:
            continue
        if negated(i):
            negexit = True
        else:
            bare_exit = True
    widen = bool(set(toks) & _WIDEN)
    avoid = (prohibit or neg_keep) and not permit and not pos_keep and not negexit
    keep_in = (pos_keep or negexit) and not bare_exit and not neg_keep and not widen
    return {"avoid": avoid, "keep_in": keep_in, "permit": permit and not keep_in}


_ANAPHORS = {"it", "there", "them"}
_INDEFINITE = {"any", "another", "other", "every", "all", "some", "a", "an"}
_DEFINITE = {"the", "that", "this", "it", "same", "your", "its"}
# Words that only carry a bound, not an attribute (for tag inheritance).
_CUE_WORDS = ({w for c in _MIN_CUES + _MAX_CUES for w in c}
              | _NEGATORS | _ARTICLES | _SEPS | {"but", "also", "then"})
# A ruling AFTER the number, through a copula: "... below 4 m/s is not
# allowed", "speeds above 4 m/s are forbidden", "turns of 45 deg/s or more are
# fine". Narrower than the zone cue table on purpose: "away from" after a
# distance is the distance's direction, not a ruling on it.
_TRAIL_PROHIBIT = {"forbidden", "prohibited", "banned", "barred", "disallowed",
                   "unacceptable", "illegal"}
_TRAIL_PERMIT = {"allowed", "permitted", "fine", "ok", "okay", "acceptable", "alright"}
_COPULA = {"is", "are", "be", "'s", "remains", "remain", "stays", "stay"}
# Also read as negators in a bound's window: "it is forbidden to drop below 10 m".
_LEAD_PROHIBIT = {"forbidden", "prohibited", "banned", "disallowed"}


_REQUIRE_VERBS = {"maintained", "kept", "held", "observed", "respected", "applied",
                  "met", "followed", "enforced", "honoured", "honored", "obeyed"}


def _trail_modality(A: list[str]) -> str | None:
    for j, x in enumerate(A):
        if x in ("shall", "must", "should", "will", "is", "are", "be") and (
                set(A[j + 1:j + 5]) & _REQUIRE_VERBS):
            # "... 5 m from the tracked subject shall (not) be maintained"
            if sum(_is_negator(y) for y in A[j + 1:j + 4]) % 2 == 1:
                return "prohibit"
            continue
        if x not in _COPULA:
            continue
        rest = A[j + 1:j + 4]
        neg = sum(_is_negator(y) for y in rest) % 2 == 1
        for k, y in enumerate(rest):
            if y in _TRAIL_PROHIBIT or (y == "off" and rest[k + 1:k + 3] in (["-", "limits"],
                                                                             ["limits"])):
                return "permit" if neg else "prohibit"
            if y in _TRAIL_PERMIT:
                return "prohibit" if neg else "permit"
    return None


def _colour_target(ctoks: list[str], i: int, reg: dict) -> int | None:
    """The object a colour describes: the next object within three tokens
    ("red car", "dark red automobile") with no other slot between, or the
    object of "<object> that is <colour>". None when it describes nothing the
    extractor knows ("the car next to the red one")."""
    for j in range(i + 1, min(len(ctoks), i + 4)):
        if ctoks[j] in reg:
            return j if reg[ctoks[j]][0] == "obj" else None
    if i >= 2 and ctoks[i - 1] in ("is", "'s", "are"):
        for j in range(i - 2, max(-1, i - 5), -1):
            if ctoks[j] in reg:
                return j if reg[ctoks[j]][0] == "obj" else None
    return None


def _negated_before(ctoks: list[str], i: int, reg: dict) -> bool:
    """An action or direction under a negation in its own clause: "do not
    follow", "never track", "stop following", "do not fly forward"."""
    window = ctoks[max(0, i - 4):i]
    if sum(_is_negator(x) for x in window) % 2 == 1:
        return True
    return any(x in _STOP_VERBS or reg.get(x) == ("act", "stop")
               for x in ctoks[max(0, i - 3):i])


def extract_slots(text: str, known_ids: Iterable[str] = ()) -> Slots:
    """Every mission slot in `text`, normalised for comparison.

    `known_ids` are identifiers from the source: in a paraphrase they count
    whether or not they kept their quotes ("the nfz-square zone")."""
    t = _norm(text)
    slots = Slots()
    slots.words = re.findall(r"[a-z0-9]+(?:['.\-][a-z0-9]+)*", t)
    slots.hedges = [m.group(0) for m in _HEDGE.finditer(t)]
    slots.conditions = _conditions(t)
    slots.overrides = [re.sub(r"\s+", " ", m.group(0)) for m in _OVERRIDE.finditer(t)]
    slots.soft = len(_SOFT.findall(t))
    reg: dict[str, Any] = {}

    def put(kind: str, payload: Any) -> str:
        key = f"⟦{kind}{len(reg)}⟧"
        reg[key] = (kind, payload)
        return f" {key} "

    # 1. identifiers: quoted, plus the source's ids however they are written
    for kid in sorted(set(known_ids), key=len, reverse=True):
        t = re.sub(rf"(?<![a-z0-9_\-])['\"]?{re.escape(kid)}['\"]?(?![a-z0-9_\-])",
                   lambda m, kid=kid: put("id", kid), t)
    t = _QUOTED_ID.sub(lambda m: put("id", m.group("id")), t)
    # 2. named places (the compiler resolves them by substring)
    for name in _places():
        t = re.sub(rf"\b{re.escape(name)}\b", lambda m, n=name: put("place", n), t)
    # 3. coordinates
    t = _COORD_PAREN.sub(lambda m: put("coord", (float(m.group("x")),
                                                  float(m.group("y")))), t)
    t = _COORD_BARE.sub(lambda m: m.group("pre") + put(
        "coord", (float(m.group("x")), float(m.group("y")))), t)
    # 3b. schedules: times first (so 07:30 is not two numbers), then days
    t = _TIME.sub(lambda m: put("time", _clock(m)), t)
    t = _DAY_RANGE.sub(lambda m: put("day", _day_span(m.group("a"), m.group("b"))), t)
    t = _DAY_ONE.sub(lambda m: put("day", _day_set(m)), t)

    # 4. ranges: "between 10 m and 20 m", "10 m to 20 m"
    def _range(m: re.Match) -> str:
        ua, ub = _unit_canon(m.group("ua")), _unit_canon(m.group("ub"))
        if ua is not None and ua != ub:
            return m.group(0)
        return put("range", (_num_value(m.group("a")), _num_value(m.group("b")), ub))
    t = _RANGE_A.sub(_range, t)
    t = _RANGE_B.sub(_range, t)
    t = _RANGE_C.sub(_range, t)

    # 5. single quantities; a number WORD only counts with a unit after it
    def _quant(m: re.Match) -> str:
        unit = _unit_of(m)
        num = m.group("num")
        if unit is None and not re.fullmatch(_DIGITS, num):
            return m.group(0)
        return put("q", (_num_value(num), unit))
    t = _QUANT_RE.sub(_quant, t)

    # 6. named references, objects, colours, directions, actions
    for name, rx in _REF_RES:
        t = rx.sub(lambda m, n=name: put("ref", n), t)
    for cls, rx in _OBJECT_RES:
        t = rx.sub(lambda m, c=cls: put("obj", c), t)
    t = _COLOURS.sub(lambda m: put("col", _COLOUR_CANON.get(m.group(1), m.group(1))), t)
    t = _NEUTRAL.sub(" ", t)
    for cls, rx in _DIRECTION_RES:
        t = rx.sub(lambda m, c=cls: put("dir", c), t)
    for cls, rx in _ACTION_RES:
        t = rx.sub(lambda m, c=cls: put("act", c), t)

    # 7. tokens -> sentences -> clauses
    toks = _TOKEN_RE.findall(t)
    chunks: list[dict] = []
    cur: list[str] = []
    sent = 0
    for tok in toks:
        # "a minimum of 10 m between you and any pedestrian": that "and" pairs
        # the two ends of "between" (a band of numbers was already consumed as
        # a range), so it does not end the clause
        if tok == "and" and "between" in cur and "and" not in cur[cur.index("between"):]:
            cur.append(tok)
            continue
        if tok in _CHUNK_BREAK:
            chunks.append({"toks": cur, "sent": sent, "brk": tok})
            cur = []
            if tok in _SENT_BREAK:
                sent += 1
            continue
        cur.append(tok)
    chunks.append({"toks": cur, "sent": sent, "brk": None})

    prev_q: dict | None = None
    last_ids: tuple | None = None                      # (sentence, ids) of the last id clause
    sched: list[tuple[int, str, str]] = []             # (sentence, kind, value)
    ids_in: dict[int, set] = {}
    pending_band: list[tuple[int, int]] = []           # (quantity index, sentence)
    for ci, ch in enumerate(chunks):
        ctoks = ch["toks"]
        if not ctoks:
            continue
        pol = None
        qpos = [i for i, tk in enumerate(ctoks) if tk in reg and reg[tk][0] in ("q", "range")]
        has_sched = any(tk in reg and reg[tk][0] in ("day", "time") for tk in ctoks)
        if has_sched and _INVERT.search(" ".join(_plain(ctoks, reg))):
            slots.inverted_schedule = True
        # "zone 'nfz-square' is a no-fly zone": the class word is the ruling
        restricted_at = [i for i, tk in enumerate(ctoks)
                         if reg.get(tk) == ("obj", "restricted_area")]
        coloured: set[int] = set()
        chunk_ids = {reg[tk][1] for tk in ctoks if reg.get(tk, ("",))[0] == "id"}
        if chunk_ids:
            last_ids = (ch["sent"], chunk_ids)
        elif (last_ids and last_ids[0] == ch["sent"]
              and set(ctoks) & _ANAPHORS):
            # "zone X is closed on weekdays, so never go in there": the second
            # clause rules on the same zone. Without this, "..., so go in
            # there" passed as a keep-out rule (mutation test, 2026-10-06).
            pol_a = _chunk_polarity(ctoks, reg, len(restricted_at))
            for zid in last_ids[1]:
                slots.ids[zid].append(pol_a)
        for i, tk in enumerate(ctoks):
            if tk not in reg:
                continue
            kind, payload = reg[tk]
            if kind in ("day", "time"):
                sched.append((ch["sent"], kind, payload))
                continue
            if kind == "id":
                ids_in.setdefault(ch["sent"], set()).add(payload)
                pol = pol or _chunk_polarity(ctoks, reg, len(restricted_at))
                slots.ids.setdefault(payload, []).append(pol)
                slots.id_sents.add(ch["sent"])
            elif kind == "place":
                # "to the pad" vs "away from / past / near the pad": the same
                # name in a different role is a different destination
                before = set(_plain(ctoks[max(0, i - 3):i], reg))
                slots.places.append((payload, "relative" if before & _PLACE_RELATIVE
                                     else "dest"))
            elif kind == "coord":
                slots.coords.append(payload)
            elif kind == "obj":
                rest = [x for x in ctoks if x != tk]
                slots.objects.setdefault(payload, []).append(
                    (_chunk_polarity(rest, reg), ch["sent"]))
            elif kind == "col":
                slots.colours.add(payload)
                j = _colour_target(ctoks, i, reg)
                if j is not None:
                    coloured.add(j)
                slots.colour_binds.add((payload, None if j is None else reg[ctoks[j]][1]))
            elif kind == "dir":
                slots.directions.add(payload)
                if _negated_before(ctoks, i, reg):
                    slots.neg_directions.add(payload)
            elif kind == "act":
                slots.actions.add(payload)
                if _negated_before(ctoks, i, reg):
                    slots.neg_actions.add(payload)
            elif kind == "ref":
                slots.refs.add(payload)
                if _negated_before(ctoks, i, reg):
                    slots.neg_refs.add(payload)
        # an object named with "any"/"a" and no colour, in a text whose
        # source bound a colour to that class, widens it ("any car")
        for i, tk in enumerate(ctoks):
            if reg.get(tk, ("",))[0] == "obj" and i not in coloured:
                det = set(ctoks[max(0, i - 2):i])
                if det & _INDEFINITE and not det & _DEFINITE:
                    slots.widened.add(reg[tk][1])
        for qi, p in enumerate(qpos):
            start = qpos[qi - 1] + 1 if qi > 0 else 0
            end = qpos[qi + 1] if qi + 1 < len(qpos) else len(ctoks)
            W, A = ctoks[start:p], ctoks[p + 1:end]
            kind, payload = reg[ctoks[p]]
            if (prev_q is not None and prev_q["sent"] != ch["sent"]):
                prev_q = None
            trail = _trail_modality(_head(A))
            if kind == "range":
                lo, hi, unit = payload
                tags = _tags(W, A, unit, reg, None)
                # "between 10 m and 20 m is forbidden" names the band to stay OUT of
                if _range_inverted(W) != (trail == "prohibit"):
                    slots.quantities.append(Quantity(lo, unit, "max", tags))
                    slots.quantities.append(Quantity(hi, unit, "min", tags))
                    prev_q = {"bound": "min", "negated": False, "chunk": ci,
                              "sent": ch["sent"], "tags": tags, "unit": unit}
                    continue
                if not tags and unit in ("m", "ft") and not (
                        set(_plain(ctoks, reg)) & _DISTANCE_WORDS):
                    pending_band.append((len(slots.quantities), ch["sent"]))
                slots.quantities.append(Quantity(lo, unit, "min", tags))
                slots.quantities.append(Quantity(hi, unit, "max", tags))
                prev_q = {"bound": "max", "negated": False, "chunk": ci,
                          "sent": ch["sent"], "tags": tags, "unit": unit}
                continue
            value, unit = payload
            bound, negated, cue_word = _bound(W, A, prev_q, ci, reg)
            if trail == "prohibit":
                # "A speed below 4 m/s is not allowed": the cue names what is
                # forbidden, so the limit is the other side of it
                bound = {"min": "max", "max": "min"}.get(bound, "excluded")
            # attribute words after the last "or" belong to this quantity; the
            # ones before it belong to the previous one ("within 3 m of a
            # building or within 10 m of a pedestrian")
            cut = max(_list_ors(W), default=-1)
            tags = _tags(W[cut + 1:], A, unit, reg, cue_word)
            if (not tags and prev_q is not None and prev_q["chunk"] in (ci, ci - 1)
                    and prev_q.get("unit") == unit and prev_q.get("tags")
                    and not [x for x in W[cut + 1:] if x not in _CUE_WORDS]):
                # "altitude at least 10 m and at most 20 m": the second limit
                # carries the first one's attribute when it adds no word of its own
                tags = prev_q["tags"]
            slots.quantities.append(Quantity(value, unit, bound, tags, negated))
            prev_q = {"bound": bound, "negated": negated, "chunk": ci,
                      "sent": ch["sent"], "tags": tags, "unit": unit}

    # The compiler's corridor sentence ends "... and between 10 m and 20 m"
    # with no attribute word (templates/sentence/corridor.j2: those are the
    # corridor's altitude floor and ceiling). A bare metres band in the same
    # sentence as a stay-inside id is read as that altitude band - on both
    # sides - unless its own clause speaks of distance.
    keep_in_sents = {sent for sent, ids in ids_in.items()
                     if any(_polarity_kind(slots.ids[i]) == "keep_in" for i in ids)}
    for qi, sent in pending_band:
        if sent in keep_in_sents:
            for k in (qi, qi + 1):
                q = slots.quantities[k]
                slots.quantities[k] = Quantity(q.value, q.unit, q.bound,
                                               frozenset({"altitude"}), q.negated)

    # A window belongs to the rule named in its sentence - or, for a follow-up
    # sentence ("This applies Mon-Fri ..."), the one named just before it.
    days_by: dict[tuple, set] = {}
    for sent, kind, value in sched:
        owner = ids_in.get(sent) or ids_in.get(sent - 1) or set()
        key = frozenset(owner)
        if kind == "day":
            days_by.setdefault(key, set()).update(value)
        else:
            slots.schedule.append(("time", value, key))
    for key, days in days_by.items():
        slots.schedule.append(("days", ",".join(d for d in _DAY_ORDER if d in days), key))
    slots.schedule.sort(key=lambda x: (x[0], x[1], sorted(x[2])))
    return slots


def _day_key(word: str) -> str:
    return word[:3]


def _day_span(a: str, b: str) -> frozenset:
    i, j = _DAY_ORDER.index(_day_key(a)), _DAY_ORDER.index(_day_key(b))
    if j < i:
        j += 7
    return frozenset(_DAY_ORDER[k % 7] for k in range(i, j + 1))


def _day_set(m: re.Match) -> frozenset:
    if m.group("grp"):
        return frozenset(_DAY_ORDER[:5] if m.group("grp").startswith("weekday")
                         else _DAY_ORDER[5:])
    return frozenset({_day_key(m.group("a"))})


def _clock(m: re.Match) -> str:
    if m.group("h2") is not None:
        return f"{int(m.group('h2')):02d}:{m.group('m2')}"
    h = int(m.group("h")) % 12
    if m.group("ap").startswith("p"):
        h += 12
    return f"{h:02d}:{m.group('m') or '00'}"


def _list_ors(W: list[str]) -> list[int]:
    """Positions of a list "or"/"nor" - not the "or" inside "at or below",
    which is part of one comparative and must not split the clause."""
    return [j for j, x in enumerate(W) if x in ("or", "nor")
            and not (j > 0 and W[j - 1] == "at")]


def _plain(tokens: list[str], reg: dict) -> list[str]:
    return [t for t in tokens if t not in reg]


def _head(A: list[str]) -> list[str]:
    """The words after a quantity that still belong to it: up to the first
    list "or"/"nor" (but not "or less" / "or more", which are its cue)."""
    for j, x in enumerate(A):
        if x in ("or", "nor") and not (j + 1 < len(A) and A[j + 1] in _OR_CUE_TAILS):
            return A[:j]
    return A


def _bound(W: list[str], A: list[str], prev_q: dict | None, ci: int,
           reg: dict) -> tuple[str, bool, str | None]:
    """min / max / target for one quantity, from the words around it.

    A positional comparative ("over", "below", "within") must sit right before
    the number - "fly over to the pad at 6 m/s" is not "over 6 m/s" - while a
    word that only means a limit ("maximum distance to buildings: 5 m") may sit
    a few words back."""
    n = len(W)
    cues = []
    for cue_set, pol in ((_MIN_CUES, "min"), (_MAX_CUES, "max")):
        for s_, e_, c in _find_cues(W, cue_set):
            reach = 3 if c in _WEAK_CUES else 6
            if e_ > n - reach:
                cues.append((s_, e_, c, pol))
    adjacent = prev_q is not None and prev_q["chunk"] in (ci, ci - 1)
    # words before the last list "or" belong to the previous limit ("closer
    # than 10 m to a pedestrian or 5 m to your target": the 5 m has no word
    # of its own and inherits "closer than")
    cut = max(_list_ors(W), default=-1)
    content = [t for t in W[cut + 1:] if t not in _SEPS]
    if cues:
        s_, e_, c, pol = max(cues, key=lambda x: (x[1], x[1] - x[0]))
        # Every negation in the clause counts, by parity: "should not fly at
        # an altitude no lower than 6 m" is a ceiling, "do not make sure you
        # never drop below 10 m" is no floor. (A five-token window, the rule
        # until 2026-10-06, saw only the nearer negator of each pair.)
        negated = sum(_is_negator(W[j]) or W[j] in _LEAD_PROHIBIT
                      for j in range(max(0, s_ - 12), s_)) % 2 == 1
        # "never drop below 10 m or rise above 20 m": a negation carries across
        # an "or" inside one clause, and nowhere else - after a comma, "climb
        # slower than 2 m/s" is a new instruction, not the old one negated.
        if (not negated and prev_q is not None and prev_q["chunk"] == ci
                and prev_q["negated"] and _list_ors(W)
                and not (set(W) & _RESET)):
            negated = True
        if negated:
            pol = "max" if pol == "min" else "min"
        return pol, negated, c[-1]
    # No comparative before the number. A negation in the clause still rules
    # it: "do not keep your speed at 4 m/s max" is not a 4 m/s cap, and "do
    # not hold 6 m/s" is not a 6 m/s target (negate-verb mutants, 2026-10-06).
    tail_neg = sum(_is_negator(x) or x in _LEAD_PROHIBIT
                   for x in W[cut + 1:][-6:]) % 2 == 1
    head = _head(A)[:4]
    hits = [(i, pol) for cues_, pol in ((_MIN_AFTER, "min"), (_MAX_AFTER, "max"))
            for i, _, _c in _find_cues(head, cues_)]
    if hits:
        pol = min(hits)[1]
        if tail_neg:
            pol = "max" if pol == "min" else "min"
        return pol, tail_neg, None
    if adjacent and prev_q["bound"] in ("min", "max"):
        bare = [t for t in content if t not in _ARTICLES
                and not (t in reg and reg[t][0] == "obj")]
        if not bare:
            return prev_q["bound"], False, None
    if (set(W) | set(_head(A))) & _NOUN_MIN:
        return ("max" if tail_neg else "min"), tail_neg, None
    return ("excluded" if tail_neg else "target"), tail_neg, None


def _range_inverted(W: list[str]) -> bool:
    """"beyond the 10 m to 20 m band", "never between ..." - a band read as
    the place NOT to be. Not a min and a max any more, so it cannot match one."""
    tail = W[-3:]
    outside = bool(set(tail) & {"beyond", "outside"})
    negated = sum(_is_negator(t) or t in _LEAD_PROHIBIT for t in W[-6:]) % 2 == 1
    return outside != negated


def _tags(W: list[str], A: list[str], unit: str | None, reg: dict,
          cue_word: str | None) -> frozenset:
    """What the quantity limits: attribute words of its own clause, nearest
    side first, filtered to the ones its unit can express."""
    allowed = None if unit is None else _UNIT_TAGS.get(unit, set())
    raw_w: set = set()

    def side(tokens: list[str], record: bool = False) -> set:
        found = set()
        plain = _plain(tokens, reg)
        # "vertical speed", "climb rate", "turn speed": one attribute, the
        # first word's. Read as speed + climb, "Max vertical speed 4 m/s"
        # passed as a paraphrase of "speed at or below 4 m/s" (review,
        # 2026-10-06), because a candidate could carry MORE tags than its source.
        text = re.sub(r"\b(climb\w*|ascen\w*|descen\w*|vertical\w*|turn\w*|yaw\w*|"
                      r"rotat\w*)\s+(?:speed|velocity|rate)\b", r"\1", " ".join(plain))
        for k, rx in _ATTR_RES.items():
            if rx.search(text):
                found.add(k)
        # "climb slower than 2 m/s": the comparative says how, the verb says
        # what - a climb rate, not a speed. Only the speed NOUNS name a speed
        # beside a climb or turn word.
        if "speed" in found and found & {"climb", "turn"} and not _SPEED_NOUN.search(text):
            found.discard("speed")
        if record:
            raw_w.update(found)
        # "a distance no lower than 6 m" is a distance: "lower" only implies
        # altitude when nothing in the clause says distance and no altitude
        # noun is there to settle it.
        if ("altitude" in found and set(plain) & _DISTANCE_NOUNS
                and not _ALT_NOUN.search(text)):
            found.discard("altitude")
        for tk in tokens:
            if tk in reg and reg[tk][0] == "obj":
                found.add("obj:" + reg[tk][1])
        if allowed is not None:
            found = {f for f in found if f in allowed or
                     (f.startswith("obj:") and unit in ("m", "ft", "km"))}
        return found

    # "120 m ahead", "forward 120 m": a distance along a direction, not a
    # height - whatever altitude words come later in the clause.
    head = [x for x in _head(A) if x not in ("directly", "dead", "due")]
    tail = [x for x in W if x != "by"]
    if unit in ("m", "ft", "km") and (
            (head and head[0] in reg and reg[head[0]][0] == "dir")
            or (tail and tail[-1] in reg and reg[tail[-1]][0] == "dir")):
        return frozenset({"offset"})
    tags = side(W, record=True) or side(_head(A))
    if cue_word in _VERTICAL_CUES and unit == "m" and not (
            set(_plain(W, reg)) & _DISTANCE_NOUNS):
        tags = tags | {"altitude"}
    if not tags and not raw_w and unit in ("m/s", "km/h", "mph", "deg/s"):
        # A rate with no attribute word is the plain one: "fly at 6 m/s" is a
        # speed, "45 deg/s" a turn rate - so "never exceed 4 m/s" matches
        # "speed at or below 4 m/s" now that kinematic tags must be equal. Not
        # when the clause names something else ("max spacing 4 m/s", "4 m/s of
        # altitude"): that is a changed attribute, not a missing one.
        plain = set(_plain(W, reg)) | set(_plain(_head(A), reg))
        lead = _plain(_head(A), reg)[:2]
        foreign = (len(lead) == 2 and lead[0] in ("of", "in")
                   and any(rx.search(lead[1]) for rx in _ATTR_RES.values()))
        if not (plain & (_DISTANCE_WORDS | _NOUN_MIN)) and not foreign:
            tags = {"turn"} if unit == "deg/s" else {"speed"}
    return frozenset(tags)


# =========================================================================== #
# The validator
# =========================================================================== #

@dataclass(frozen=True)
class Verdict:
    ok: bool
    reasons: tuple
    novelty: float
    length_ratio: float

    def to_dict(self) -> dict:
        return {"ok": self.ok, "reasons": list(self.reasons),
                "novelty": round(self.novelty, 4),
                "length_ratio": round(self.length_ratio, 4)}


def _levenshtein(a: list[str], b: list[str]) -> int:
    prev = list(range(len(b) + 1))
    for i, x in enumerate(a, 1):
        cur = [i]
        for j, y in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (x != y)))
        prev = cur
    return prev[-1]


def _identity_words(text: str) -> list[str]:
    return re.findall(r"[a-z0-9]+(?:['.][a-z0-9]+)*", _norm(text))


def novelty(source: str, candidate: str) -> float:
    a, b = _identity_words(source), _identity_words(candidate)
    if not a and not b:
        return 0.0
    return _levenshtein(a, b) / max(len(a), len(b))


_KINEMATIC = frozenset({"speed", "climb", "turn"})


def _q_ok(s: Quantity, c: Quantity) -> bool:
    if abs(s.value - c.value) > 1e-9 or s.bound != c.bound:
        return False
    if s.unit != c.unit:
        # "altitude 20" (the compiler reads it as metres) may become "20 m";
        # a unit the source had may not be dropped or changed.
        if not (s.unit is None and c.unit == "m" and "altitude" in s.tags):
            return False
    # What a rate limits, and which object a distance is kept from, must be
    # the same - not merely included: a candidate tagged speed+climb used to
    # match a source tagged speed. A candidate may still name more of the
    # spatial context ("between 10 m and 20 m in altitude" inside a corridor).
    if (s.tags & _KINEMATIC) != (c.tags & _KINEMATIC):
        return False
    if ({t for t in s.tags if t.startswith("obj:")}
            != {t for t in c.tags if t.startswith("obj:")}):
        return False
    return s.tags <= c.tags


def _match_quantities(src: list[Quantity], cand: list[Quantity]) -> tuple[list, list]:
    """Maximum bipartite matching (Kuhn). Returns unmatched (src, cand)."""
    match_c: dict[int, int] = {}

    def try_src(i: int, seen: set) -> bool:
        for j, c in enumerate(cand):
            if j in seen or not _q_ok(src[i], c):
                continue
            seen.add(j)
            if j not in match_c or try_src(match_c[j], seen):
                match_c[j] = i
                return True
        return False

    for i in range(len(src)):
        try_src(i, set())
    matched_src = set(match_c.values())
    return ([q for i, q in enumerate(src) if i not in matched_src],
            [q for j, q in enumerate(cand) if j not in match_c])


def _diagnose(q: Quantity, pool: list[Quantity]) -> str:
    for c in pool:
        if abs(q.value - c.value) < 1e-9 and q.unit == c.unit:
            if q.bound != c.bound:
                return f"bound changed: {q.describe()} became {c.describe()}"
            return f"now limits something else: {q.describe()} became {c.describe()}"
    for c in pool:
        if abs(q.value - c.value) < 1e-9:
            return f"unit changed: {q.describe()} became {c.describe()}"
    return f"dropped or changed: {q.describe()}"


def _polarity_kind(pols: list[dict]) -> str:
    if any(p["keep_in"] for p in pols):
        return "keep_in"
    if any(p["avoid"] for p in pols):
        return "avoid"
    return "mention"


def validate(source: str, candidate: str) -> Verdict:
    """Is `candidate` a faithful, genuinely different wording of `source`?"""
    reasons: list[str] = []
    nov = novelty(source, candidate)
    sw, cw = _identity_words(source), _identity_words(candidate)
    ratio = (len(cw) / len(sw)) if sw else 0.0
    pr = protected_reason(source) or protected_reason(candidate)
    if pr:
        return Verdict(False, (pr,), nov, ratio)
    if not cw:
        return Verdict(False, ("empty: no words",), nov, ratio)
    if sw == cw:
        reasons.append("identity: same words as the source - not a paraphrase")
    elif nov < MIN_NOVELTY:
        reasons.append(f"identity: near-identical (novelty {nov:.2f} < "
                       f"{MIN_NOVELTY}) - not a different wording")
    # A three-word task naturally grows ("Your job is to keep track of a
    # person as they walk around."), so the cap has a floor of +12 words.
    longest = max(MAX_LENGTH_RATIO * len(sw), len(sw) + 12)
    shortest = MIN_LENGTH_RATIO * len(sw) if len(sw) >= 8 else 1
    if len(cw) > longest or len(cw) < shortest:
        reasons.append(f"length: {len(cw)} words against the source's {len(sw)} "
                       f"(allowed {shortest:g}-{longest:g})")

    s = extract_slots(source)
    c = extract_slots(candidate, known_ids=s.ids.keys())

    for zid, pols in s.ids.items():
        if zid not in c.ids:
            reasons.append(f"zone: dropped or renamed '{zid}'")
            continue
        want = _polarity_kind(pols)
        cp = c.ids[zid]
        if want == "mention":
            # Fail closed. Every zone the compiler names is a keep-out or a
            # stay-inside rule; if the SOURCE's ruling could not be read, no
            # candidate's can be checked against it, and accepting would let a
            # flipped zone through unseen. (Found 2026-10-06: a too-broad entry
            # word made "(in force ...)" read as permission, and two mutants
            # that flipped 'nfz-school' passed.)
            reasons.append(f"polarity: could not read the source's rule for '{zid}' "
                           f"- refusing rather than passing it unchecked")
        if want == "avoid" and not (any(p["avoid"] for p in cp)
                                    and not any(p["keep_in"] for p in cp)
                                    and not any(p.get("permit") for p in cp)):
            reasons.append(f"polarity: '{zid}' is no longer a keep-out rule")
        if want == "keep_in" and not (any(p["keep_in"] for p in cp)
                                      and not any(p["avoid"] for p in cp)):
            reasons.append(f"polarity: '{zid}' is no longer a stay-inside rule")
    for zid in c.ids:
        if zid not in s.ids:
            reasons.append(f"zone: added '{zid}'")

    if sorted(s.places) != sorted(c.places):
        reasons.append(f"place: {sorted(set(s.places))} became {sorted(set(c.places))}")
    if sorted(s.coords) != sorted(c.coords):
        reasons.append(f"coordinate: {sorted(s.coords)} became {sorted(c.coords)}")

    miss, extra = _match_quantities(s.quantities, c.quantities)
    for q in miss:
        reasons.append("quantity: " + _diagnose(q, c.quantities))
    for q in extra:
        reasons.append(f"quantity: added {q.describe()}")

    s_obj, c_obj = set(s.objects), set(c.objects)
    for o in sorted(s_obj - c_obj):
        reasons.append(f"object: dropped or changed '{o}'")
    for o in sorted(c_obj - s_obj):
        if o == "restricted_area" and all(sent in c.id_sents
                                          for _, sent in c.objects[o]):
            # "the square no-fly zone 'nfz-square'" describes a zone the
            # source names; it is not a second, generic prohibition
            continue
        reasons.append(f"object: added '{o}'")
    # Zone-like objects carry a keep-out polarity of their own. A pedestrian or
    # a building does not: what the rule says about it is a distance, and the
    # distance's bound is checked with the quantities.
    for o in sorted(s_obj & c_obj & {"restricted_area"}):
        want = _polarity_kind([p for p, _ in s.objects[o]])
        if want == "avoid":
            cps = [p for p, _ in c.objects[o]]
            if not (any(p["avoid"] for p in cps) and not any(p["keep_in"] for p in cps)):
                reasons.append(f"polarity: '{o}' is no longer something to keep out of")

    if s.colours != c.colours:
        reasons.append(f"colour: {sorted(s.colours)} became {sorted(c.colours)}")
    elif s.colour_binds != c.colour_binds:
        reasons.append(f"colour: now describes something else: "
                       f"{sorted(s.colour_binds, key=str)} became "
                       f"{sorted(c.colour_binds, key=str)}")
    widened = {o for _, o in s.colour_binds if o} & c.widened
    for o in sorted(widened):
        reasons.append(f"colour: '{o}' widened to any {o}, not only the coloured one")
    if s.refs != c.refs:
        reasons.append(f"reference: {sorted(s.refs)} became {sorted(c.refs)}")
    elif s.neg_refs != c.neg_refs:
        reasons.append(f"reference: negated {sorted(s.neg_refs)} became {sorted(c.neg_refs)}")
    if s.directions != c.directions:
        reasons.append(f"direction: {sorted(s.directions)} became {sorted(c.directions)}")
    elif s.neg_directions != c.neg_directions:
        reasons.append(f"direction: negated {sorted(s.neg_directions)} became "
                       f"{sorted(c.neg_directions)}")
    if s.actions != c.actions:
        reasons.append(f"action: {sorted(s.actions)} became {sorted(c.actions)}")
    elif s.neg_actions != c.neg_actions:
        reasons.append(f"action: negated {sorted(s.neg_actions)} became "
                       f"{sorted(c.neg_actions)}")

    def _sched(sl: Slots) -> list:
        return [(k, v, sorted(o)) for k, v, o in sl.schedule]
    if _sched(s) != _sched(c):
        reasons.append(f"schedule: {_sched(s)} became {_sched(c)}")
    if c.inverted_schedule and not s.inverted_schedule:
        reasons.append("schedule: the window was inverted ('except' / 'other than')")
    if c.hedges and not s.hedges:
        reasons.append(f"modality: a hard rule was softened ({', '.join(c.hedges)})")
    # A condition that restates the source's own window is the window; any
    # other condition, exception or scope limit is a new rule.
    def _conds(sl: Slots, has_window: bool) -> Counter:
        return Counter(w for w in sl.conditions
                       if not (w.startswith("sched:") and has_window))
    new_conds = _conds(c, bool(s.schedule)) - _conds(s, bool(s.schedule))
    if new_conds:
        reasons.append("modality: made conditional or given an exception ("
                       + ", ".join(sorted(w.replace("sched:", "")
                                          for w in new_conds.elements())) + ")")
    new_over = Counter(c.overrides) - Counter(s.overrides)
    if new_over:
        reasons.append("modality: a rule was suspended or made optional ("
                       + ", ".join(sorted(new_over.elements())) + ")")
    if s.soft != c.soft:
        reasons.append(f"modality: soft-limit markers {s.soft} became {c.soft}")

    return Verdict(not reasons, tuple(reasons), nov, ratio)


# =========================================================================== #
# The stored set
# =========================================================================== #

GENERATOR_ID = "claude-opus-5-5 (agent, 2026-10-06)"

# The instruction the stored sets were written under, verbatim from the task
# that commissioned them, so the provenance names what was asked for and not a
# summary of it.
TASK_INSTRUCTION = (
    "Generate the free-form set yourself now (you are an LLM): for every "
    "instruction used in experiments/scenarios.yaml and the city follow missions "
    "(grep demo/ and scripts/ for the instruction strings), write K=8 genuinely "
    "varied free-form paraphrases (vocabulary, word order, politeness, verbosity, "
    "indirect phrasing), store them with provenance {generator: \"claude-opus-5-5 "
    "(agent, 2026-10-06)\", prompt: <the exact instruction you followed>, date}, "
    "and run the validator over them.")

GENERATION_PROMPT = """\
You are writing free-form paraphrases for the Paraphraser service of the
constrained-VLA guardrail (ITRI ICL subcontract; WP4 per-paraphrase robustness).

Source ({kind}): "{source}"

Write K={k} paraphrases of the source. Make them genuinely varied in
vocabulary, word order, politeness, verbosity and indirect phrasing. {form_rule}

Every paraphrase must keep every mission slot: the target object and its
colour, every number with its unit, its bound (at least / at most / between /
at) and the thing it limits, every named zone, corridor or place (identifiers
verbatim), whether a zone is to be kept out of or stayed inside, the direction
words, and the action (fly to / follow / avoid). Do not add a rule, number,
object, zone or direction the source does not contain, and do not drop one.
Return one paraphrase per line."""

FORM_RULES = {
    "sentences": "Each paraphrase is one or more complete sentences.",
    "task": ("Half of them (form: verb_phrase) must be a bare verb phrase that "
             "reads correctly after \"What action should the robot take to ...?\" "
             "(OpenVLA's prompt frame, demo/real_vla_demo.py PROMPT_TEMPLATE); the other half "
             "(form: utterance) are free operator utterances."),
}


def generation_prompt(source: str, kind: str, k: int = K_DEFAULT) -> str:
    form = "task" if kind == "task" else "sentences"
    return GENERATION_PROMPT.format(kind=kind, source=normalise_source(source),
                                    k=k, form_rule=FORM_RULES[form])


@dataclass
class StoreEntry:
    source_key: str
    source_text: str
    kind: str
    origin: dict
    provenance: dict
    paraphrases: list
    path: Path


_ITEM_EXTRAS = ("form", "style", "revision")


def write_store(path: str | Path, *, source_key: str, source_text: str, kind: str,
                origin: dict, provenance: dict, paraphrases: list[dict]) -> Path:
    """Freeze a paraphrase set: ids and hashes are computed here, once.

    Used for the agent-written set and meant for a future LLM-service run
    (generate, validate, freeze), so a set served in a KPI sweep is a file that
    can be diffed and cited, not a live call. An item may carry `form`,
    `style` and `revision` (why and when a text was re-authored); a
    re-authored text gets a new id, because the id keys on the text."""
    src = normalise_source(source_text)
    items = []
    for i, p in enumerate(paraphrases):
        text = normalise_source(p["text"])
        item = {"index": i,
                "paraphrase_id": make_paraphrase_id(src, "stored", i, text),
                "text": text, "text_sha256": _sha256(text)}
        for k in _ITEM_EXTRAS:
            if k in p:
                item[k] = p[k]
        items.append(item)
    doc = {"schema": STORE_SCHEMA, "source_key": source_key, "kind": kind,
           "source_text": src, "source_id": make_source_id(src),
           "source_sha256": _sha256(src), "backend": "stored",
           "origin": origin, "provenance": provenance, "paraphrases": items}
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
    return p


def load_store(store_dir: str | Path | None = None) -> dict[str, StoreEntry]:
    """Every stored set, keyed by normalised source text.

    Refuses, rather than skips, anything it cannot vouch for: an unknown
    schema, a source hash that does not match, an index out of order, an id
    that does not recompute, a text edited after its hash was taken, two
    files for one source - or, for rule text, two files for one rule set in
    different sentence orders (see rule_set_key)."""
    d = Path(store_dir) if store_dir is not None else STORE_DIR
    out: dict[str, StoreEntry] = {}
    rules: dict[tuple, StoreEntry] = {}
    if not d.is_dir():
        return out
    for f in sorted(d.glob("*.json")):
        try:
            doc = json.loads(f.read_text(encoding="utf-8"))
        except json.JSONDecodeError as e:
            raise StoreIntegrityError(f"{f.name}: not JSON ({e})") from e
        schema = doc.get("schema")
        if schema in (REPORT_SCHEMA, PROBES_SCHEMA):
            continue
        if schema != STORE_SCHEMA:
            raise StoreIntegrityError(f"{f.name}: unknown schema {schema!r}")
        src = normalise_source(doc["source_text"])
        if doc.get("source_sha256") != _sha256(src):
            raise StoreIntegrityError(f"{f.name}: source_sha256 does not match source_text")
        for i, item in enumerate(doc["paraphrases"]):
            if item.get("index") != i:
                raise StoreIntegrityError(f"{f.name}[{i}]: index {item.get('index')} out of order")
            if item.get("text_sha256") != _sha256(normalise_source(item["text"])):
                raise StoreIntegrityError(
                    f"{f.name}[{i}]: text_sha256 does not match the text - it was "
                    f"edited after its id was issued")
            if item.get("paraphrase_id") != make_paraphrase_id(src, "stored", i, item["text"]):
                raise StoreIntegrityError(f"{f.name}[{i}]: paraphrase_id does not recompute")
        if src in out:
            raise StoreIntegrityError(f"{f.name}: second set for a source already in "
                                      f"{out[src].path.name}")
        e = StoreEntry(doc["source_key"], src, doc["kind"], doc.get("origin", {}),
                       doc["provenance"], doc["paraphrases"], f)
        if e.kind == "csp_nl":
            key = rule_set_key(src)
            if key in rules:
                raise StoreIntegrityError(f"{f.name}: second set for the rule set already "
                                          f"in {rules[key].path.name} (same sentences, "
                                          f"other order)")
            rules[key] = e
        out[src] = e
    return out


# =========================================================================== #
# Backends
# =========================================================================== #

class Backend(Protocol):
    name: str

    def candidates(self, source_text: str, seed: int) -> Iterator[tuple[int, str, dict]]:
        """(index, text, provenance) in the order to try. The index must be the
        candidate's position in the backend's own set for this source, so its id
        does not depend on the seed."""


def _rng(seed: int, source: str, backend: str) -> random.Random:
    # A string seed is hashed with SHA-512 inside `random`, so the draw does not
    # depend on PYTHONHASHSEED or the interpreter run.
    return random.Random(f"guardrail.paraphraser|{seed}|{make_source_id(source)}|{backend}")


def _dir_fingerprint(d: Path) -> tuple:
    if not d.is_dir():
        return ()
    out = []
    for f in sorted(d.glob("*.json")):
        st = f.stat()
        out.append((f.name, st.st_mtime_ns, st.st_size))
    return tuple(out)


class StoredBackend:
    """Serves the frozen sets. A rule text is found by its exact wording or,
    failing that, by its rule set: compile_csp and build_prompt render the
    same rules in different orders, and both are served the same set, under
    the same ids (the ids key on the stored source, not the request)."""
    name = "stored"

    def __init__(self, store_dir: str | Path | None = None):
        self.store_dir = Path(store_dir) if store_dir is not None else STORE_DIR
        self._store: dict[str, StoreEntry] | None = None
        self._rules: dict[tuple, StoreEntry] = {}
        self._fp: tuple | None = None

    @property
    def store(self) -> dict[str, StoreEntry]:
        # Re-read when a file changed: the module-level instance lives for the
        # whole process, and before 2026-10-06 a write_store in the same
        # process left it serving the old sets (or raising UnknownSource).
        fp = _dir_fingerprint(self.store_dir)
        if self._store is None or fp != self._fp:
            self._store = load_store(self.store_dir)
            self._rules = {rule_set_key(e.source_text): e for e in self._store.values()
                           if e.kind == "csp_nl"}
            self._fp = fp
        return self._store

    def lookup(self, source_text: str) -> StoreEntry | None:
        src = normalise_source(source_text)
        store = self.store
        if src in store:
            return store[src]
        return self._rules.get(rule_set_key(src))

    def has(self, source_text: str) -> bool:
        return self.lookup(source_text) is not None

    def size(self, source_text: str) -> int:
        return len(self._entry(source_text).paraphrases)

    def id_source(self, source_text: str) -> str:
        return self._entry(source_text).source_text

    def _entry(self, source_text: str) -> StoreEntry:
        e = self.lookup(source_text)
        if e is None:
            raise UnknownSource(f"no stored paraphrase set for "
                                f"{normalise_source(source_text)!r} in {self.store_dir} "
                                f"(use backend='template' or 'auto', or freeze a set "
                                f"with write_store)")
        return e

    def candidates(self, source_text: str, seed: int):
        e = self._entry(source_text)
        order = list(range(len(e.paraphrases)))
        _rng(seed, e.source_text, self.name).shuffle(order)
        for i in order:
            item = e.paraphrases[i]
            prov = dict(e.provenance)
            prov.update({"store_file": e.path.name, "source_key": e.source_key,
                         "store_source_id": make_source_id(e.source_text),
                         "matched_by": ("text" if normalise_source(source_text)
                                        == e.source_text else "rule set")})
            for k in _ITEM_EXTRAS:
                if k in item:
                    prov[k] = item[k]
            yield i, item["text"], prov


# ---------------------------------------------------------------- template -- #
#
# One rule per sentence shape the Prefix Compiler emits
# (guardrail/compiler.py ConstraintCompiler._sentences) and per task shape the
# demos issue. Alternative 0 of every rule is the source wording, so candidate
# index 0 is the identity and is never served.

_N = r"\d+(?:\.\d+)?"
_TEMPLATE_RULES: list[tuple[str, tuple[str, ...]]] = [
    (rf"never enter zone '(?P<id>[^']+)'\.", (
        "Never enter zone '{id}'.",
        "Do not fly into zone '{id}'.",
        "Zone '{id}' is off-limits.",
        "Keep out of zone '{id}'.",
        "Stay clear of zone '{id}'.")),
    (rf"stay inside corridor '(?P<id>[^']+)', within (?P<hw>{_N}) m of its "
     rf"centerline and between (?P<lo>{_N}) m and (?P<hi>{_N}) m\.", (
        "Stay inside corridor '{id}', within {hw} m of its centerline and "
        "between {lo} m and {hi} m.",
        "Remain inside corridor '{id}': no more than {hw} m from its centerline, "
        "and between {lo} m and {hi} m.",
        "Do not leave corridor '{id}'; stay within {hw} m of its centerline and "
        "between {lo} m and {hi} m.",
        "Fly only inside corridor '{id}', at most {hw} m off its centerline and "
        "between {lo} m and {hi} m.")),
    (rf"stay between (?P<lo>{_N}) m and (?P<hi>{_N}) m altitude\.", (
        "Stay between {lo} m and {hi} m altitude.",
        "Hold your altitude between {lo} m and {hi} m.",
        "Fly no lower than {lo} m and no higher than {hi} m.",
        "Keep your height within {lo} m to {hi} m.")),
    (rf"keep speed at or below (?P<v>{_N}) m/s, climb rate below (?P<c>{_N}) m/s "
     rf"and turn rate below (?P<y>{_N}) deg/s\.", (
        "Keep speed at or below {v} m/s, climb rate below {c} m/s and turn rate "
        "below {y} deg/s.",
        "Do not exceed {v} m/s in speed; climb slower than {c} m/s and turn "
        "slower than {y} deg/s.",
        "Fly no faster than {v} m/s, with climb rate under {c} m/s and turn rate "
        "under {y} deg/s.",
        "Cap your speed at {v} m/s, your climb rate below {c} m/s and your turn "
        "rate below {y} deg/s.")),
    (rf"keep at least (?P<d>{_N}) m from any building\.", (
        "Keep at least {d} m from any building.",
        "Stay at least {d} m clear of every building.",
        "Never fly closer than {d} m to a building.",
        "Maintain a {d} m buffer from buildings.")),
    (rf"keep at least (?P<d>{_N}) m away from anything you are following\.", (
        "Keep at least {d} m away from anything you are following.",
        "Stay at least {d} m back from whatever you are following.",
        "Never close within {d} m of your target.",
        "Hold at least {d} m of separation from the target you are tracking.")),
    (rf"keep at least (?P<d>{_N}) m away from any (?P<cls>[a-z]+)\.", (
        "Keep at least {d} m away from any {cls}.",
        "Stay at least {d} m from every {cls}.",
        "Never get closer than {d} m to a {cls}.",
        "Give any {cls} a berth of at least {d} m.")),
    (rf"fly to the (?P<place>[a-z]+ pad) at (?P<v>{_N}) m/s", (
        "fly to the {place} at {v} m/s",
        "head over to the {place}, keeping to {v} m/s",
        "go to the {place}, cruising at {v} m/s",
        "make your way to the {place} at a speed of {v} m/s",
        "travel to the {place} and hold {v} m/s on the way")),
    (rf"fly to \((?P<x>-?{_N}), (?P<y>-?{_N})\) at (?P<v>{_N}) m/s altitude (?P<alt>{_N})", (
        "fly to ({x}, {y}) at {v} m/s altitude {alt}",
        "head over to ({x}, {y}), holding {v} m/s at altitude {alt}",
        "go to ({x}, {y}) at altitude {alt} and {v} m/s",
        "make your way to ({x}, {y}) at altitude {alt}, keeping to {v} m/s")),
    # Any other "fly to ..." mission (a ScenarioSpec task_prompt such as "Fly to
    # the waypoint 120 m ahead at cruise altitude."): only the verb is
    # reworded, everything after it is kept, so no slot can move. Listed after
    # the specific shapes, which win.
    (r"fly to (?P<rest>.+?)(?P<end>\.?)", (
        "fly to {rest}{end}",
        "make your way to {rest}{end}",
        "go over to {rest}{end}",
        "head over to {rest}{end}",
        "take the drone to {rest}{end}")),
    (r"follow (?P<obj>an? [a-z ]+)", (
        "follow {obj}",
        "track {obj}",
        "keep following {obj}",
        "stay with {obj}",
        "tail {obj}")),
    (r"fly forward and avoid restricted areas", (
        "fly forward and avoid restricted areas",
        "fly ahead and keep out of restricted areas",
        "go forward while avoiding restricted areas",
        "head straight ahead and stay out of restricted zones")),
]
_TEMPLATE_RES = [(re.compile(rf"^{p}$", re.IGNORECASE), alts) for p, alts in _TEMPLATE_RULES]


def template_bank_digest() -> str:
    """Pinned by the tests: editing the bank re-maps every template index to new
    text, so it must come with a TEMPLATE_BANK_VERSION bump (which re-keys the
    ids) rather than happen silently under the old ids."""
    return _sha256(json.dumps(_TEMPLATE_RULES, ensure_ascii=False))[:16]


_QUAL = re.compile(r"^(?P<core>.*?)(?P<qual>(?:\s\([^()]*\))*(?:\s\[[^\]]*\])?)\.$")


def _requalify(sentence: str, qual: str) -> str:
    if not qual:
        return sentence
    return (sentence[:-1] + qual + ".") if sentence.endswith(".") else sentence + qual


def _match_case(source: str, text: str) -> str:
    """A rule matches case-insensitively; the rewording keeps the source's
    sentence-initial capital (or its lack, for a verb-phrase task)."""
    if source[:1].isupper() and text[:1].islower():
        return text[:1].upper() + text[1:]
    return text


def _split_sentences(text: str) -> list[str]:
    return [s for s in re.split(r"(?<=\.)\s+", normalise_source(text)) if s]


class TemplateBackend:
    name = f"template-v{TEMPLATE_BANK_VERSION}"

    def _plan(self, source_text: str):
        sents = _split_sentences(source_text)
        options = []
        for s in sents:
            # A qualifier the compiler appends - "(in force Mon-Fri 07:30-17:30)",
            # "(soft limit)", "[P0]" - is carried over verbatim onto whichever
            # wording is chosen, so no alternative can drop a window.
            q = _QUAL.match(s)
            core, qual = (q.group("core") + ".", q.group("qual")) if q else (s, "")
            for rx, alts in _TEMPLATE_RES:
                m = rx.match(core)
                if m:
                    options.append([_match_case(core, _requalify(
                        a.format(**m.groupdict()), qual)) for a in alts])
                    break
            else:
                options.append([s])                    # unknown shape: verbatim
        radices = [len(o) for o in options] + ([2] if len(sents) > 1 else [])
        total = 1
        for r in radices:
            total *= r
        return options, radices, total

    def capacity(self, source_text: str) -> int:
        """How many non-identity combinations exist (before validation). A
        one-sentence task has only a handful; ask for more and the call fails
        loudly rather than repeating one."""
        return self._plan(source_text)[2] - 1

    def render(self, source_text: str, index: int) -> tuple[str, dict]:
        options, radices, total = self._plan(source_text)
        if not 0 <= index < total:
            raise IndexError(index)
        digits, rest = [], index
        for r in radices:
            digits.append(rest % r)
            rest //= r
        picked = [options[i][digits[i]] for i in range(len(options))]
        order = "original"
        if len(radices) > len(options) and digits[-1] == 1:
            picked.reverse()
            order = "reversed"
        prov = {"generator": "guardrail.paraphraser.TemplateBackend",
                "bank_version": TEMPLATE_BANK_VERSION,
                "bank_digest": template_bank_digest(),
                "choices": digits[:len(options)], "order": order}
        return " ".join(picked), prov

    def candidates(self, source_text: str, seed: int):
        _, _, total = self._plan(source_text)
        if total <= 1:
            raise InsufficientParaphrases(
                f"template backend has no rule for any sentence of {source_text!r}")
        k = min(total - 1, 256)
        for idx in _rng(seed, source_text, self.name).sample(range(1, total), k):
            text, prov = self.render(source_text, idx)
            yield idx, text, prov


# --------------------------------------------------------------------- llm -- #

class LLMClient(Protocol):
    """What an LLM service must provide. Nothing in this repository implements
    it: the build makes no network calls, and the deliverable set was frozen to
    experiments/paraphrases/ instead of being fetched live."""
    model_id: str

    def complete(self, prompt: str, *, n: int, seed: int) -> list[str]:
        ...


class LLMServiceBackend:
    """The slot for a live paraphrasing service (the grant's "separate
    service"). Ids are keyed by `llm:<model_id>`, so two models never share one.

    A live service is only reproducible if it honours `seed`; most do not.
    For a KPI sweep, run it once, validate, and freeze the result with
    `write_store` - then serve it with the stored backend."""

    def __init__(self, client: LLMClient | None = None, k: int = K_DEFAULT):
        self.client = client
        self.k = k

    @property
    def name(self) -> str:
        mid = getattr(self.client, "model_id", None) or "unconfigured"
        return f"llm:{mid}"

    def candidates(self, source_text: str, seed: int):
        if self.client is None:
            raise BackendUnavailable(
                "no LLM client configured: this build makes no network calls. "
                "Pass an LLMClient, or use the stored set (backend='stored').")
        # Rule text from the compiler is several sentences; a mission is one.
        kind = "csp_nl" if len(_split_sentences(source_text)) > 1 else "task"
        prompt = generation_prompt(source_text, kind, self.k)
        outs = self.client.complete(prompt, n=self.k, seed=seed)
        for i, text in enumerate(outs):
            yield i, str(text), {"generator": self.client.model_id,
                                 "prompt": prompt, "seed": seed}


_REGISTRY: dict[str, Any] = {}


def register_backend(backend: Any, name: str | None = None) -> None:
    _REGISTRY[name or backend.name] = backend


def get_backend(name: str, store_dir: str | Path | None = None) -> Any:
    if name == "stored":
        return StoredBackend(store_dir) if store_dir is not None else _REGISTRY["stored"]
    if name in ("template", TemplateBackend.name):
        return _REGISTRY[TemplateBackend.name]
    if name in _REGISTRY:
        return _REGISTRY[name]
    if name == "llm" or name.startswith("llm:"):
        raise BackendUnavailable(f"backend {name!r} is not registered: no LLM "
                                 f"client is configured in this build")
    raise ValueError(f"unknown backend {name!r} (known: auto, "
                     f"{', '.join(sorted(_REGISTRY))})")


register_backend(StoredBackend())
register_backend(TemplateBackend())


# =========================================================================== #
# The service call
# =========================================================================== #

def paraphrase(text: str, n: int, seed: int, backend: Any = "stored", *,
               store_dir: str | Path | None = None,
               refusals: list | None = None) -> list[Paraphrase]:
    """`n` validated paraphrases of `text`, chosen by `seed`.

    Deterministic: the same (text, n, seed, backend) returns the same
    paraphrases in the same order. Never short: if fewer than `n` candidates
    pass the validator, InsufficientParaphrases is raised. Every refused
    candidate is logged (WARNING for stored and LLM text, which should never
    fail; INFO for template combinations, where near-identity rejects are the
    design) and appended to `refusals` when a list is given.
    """
    if not isinstance(n, int) or isinstance(n, bool) or n < 1:
        raise ValueError(f"n must be a positive int, got {n!r}")
    if not isinstance(seed, int) or isinstance(seed, bool):
        raise TypeError(f"seed must be an int (the run's random_seed), got {seed!r}")
    src = normalise_source(text)
    pr = protected_reason(text)
    if pr:
        raise ProtectedTextError(pr)

    fallback = None
    if isinstance(backend, str):
        if backend == "auto":
            st = get_backend("stored", store_dir)
            if st.has(src):
                be = st
            else:
                be, fallback = get_backend("template"), "stored"
                # Loud, because the free-form set is the deliverable: an auto
                # draw that quietly became template text would be measured as
                # if it were the stored set (review, 2026-10-06).
                log.warning("paraphraser auto: no stored set for %r - serving %s "
                            "instead (provenance.fallback_from='stored')", src,
                            be.name)
        else:
            be = get_backend(backend, store_dir)
    else:
        be = backend
    if isinstance(be, StoredBackend) and n > be.size(src):
        raise ValueError(f"n={n} but the stored set for this source holds "
                         f"{be.size(src)}; a draw is never padded or repeated")
    # The stored backend keys ids on its own stored source, so a rule set
    # rendered in another sentence order draws the same paraphrases under the
    # same ids; every other backend keys on the request.
    id_src = be.id_source(src) if hasattr(be, "id_source") else src

    out: list[Paraphrase] = []
    seen = set()
    level = logging.INFO if isinstance(be, TemplateBackend) else logging.WARNING
    for idx, cand, prov in be.candidates(src, seed):
        v = validate(src, cand)
        why = list(v.reasons)
        key = " ".join(_identity_words(cand))
        if v.ok and key in seen:
            why = ["duplicate: same words as an earlier paraphrase in this draw"]
        if why:
            log.log(level, "paraphraser refused %s[%d] %r: %s",
                    be.name, idx, cand, "; ".join(why))
            if refusals is not None:
                refusals.append({"backend": be.name, "index": idx, "text": cand,
                                 "reasons": why})
            continue
        seen.add(key)
        prov = dict(prov)
        if fallback:
            prov["fallback_from"] = fallback
        cand_n = normalise_source(cand)
        out.append(Paraphrase(
            paraphrase_id=make_paraphrase_id(id_src, be.name, idx, cand_n), text=cand_n,
            backend=be.name, provenance=prov, index=idx, seed=seed,
            source_id=make_source_id(src), source_text=src,
            text_sha256=_sha256(cand_n), novelty=round(v.novelty, 4)))
        if len(out) == n:
            return out
    raise InsufficientParaphrases(
        f"{be.name} produced {len(out)} valid paraphrase(s) of {src!r}, {n} requested")


# =========================================================================== #
# What must be covered, and the mutation test
# =========================================================================== #

def _legacy_render(pol) -> str:
    """build_prompt's natural_language_prompt: one sentence per rule, in rule
    TYPE order. Still what demo/run_demo.py, sitl/run_sitl_demo.py and
    sitl/ros2_shield_node.py put in front of a pilot."""
    import yaml
    from .compiler import ConstraintCompiler, Mission
    m = Mission(task_text="", target_x=0.0, target_y=0.0, cruise_alt_m=0.0,
                speed_pref_mps=0.0)
    doc = yaml.safe_load(ConstraintCompiler(pol).build_prompt(m))
    return normalise_source(doc["natural_language_prompt"])


def _csp_render(pol, mission, now=None, region: bool = True) -> str:
    """compile_csp's natural_language_prompt - the Prefix Compiler's CSP
    ("policy + mission + clock -> CSP", Prefix Compiler PDF p3), and what
    demo/real_vla_demo.py shows OpenVLA. Its sentences come in RISK order,
    which moves with the mission; with a clock, a rule out of force is left
    out altogether. Until 2026-10-06 the stored sets were keyed on the legacy
    rendering only, so `stored` raised UnknownSource for all six CSPs as
    compile_csp emits them, and `auto` quietly served template text instead
    (independent review)."""
    from .compiler import ConstraintCompiler
    kw = {} if region else {"region_margin_m": None}
    csp = ConstraintCompiler(pol).compile_csp(mission, now=now, **kw)
    return normalise_source(csp.natural_language_prompt)


def _rotate(x: float, y: float, cx: float, cy: float, deg: float) -> tuple[float, float]:
    if not deg:
        return x, y
    a = math.radians(deg)
    dx, dy = x - cx, y - cy
    return (cx + dx * math.cos(a) - dy * math.sin(a),
            cy + dx * math.sin(a) + dy * math.cos(a))


def scenario_renderings() -> list[dict]:
    """Every rule text and task text a scenario cell can put in front of a
    pilot: experiments/scenarios.yaml expanded through
    guardrail.scenario_spec.Library (parameter sweeps and rotations included),
    each cell's CSP rendered with no clock (every rule in force) and, when
    the cell has a `clock_start`, with that clock too.

    The sweep's pilots are scripted and read no text today (the file's own
    header); this is the set a text-reading pilot WOULD be shown, so it is
    the set the stored backend must cover. Loading the library validates it:
    a broken scenarios.yaml stops this loudly rather than covering nothing."""
    from .compiler import Mission
    from .scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    out = []
    for fam in lib.families:
        for cell in fam.cells():
            spec = cell.spec
            pol = spec.policy.load(ROOT)
            m = spec.mission
            sx, sy = m.start_pose.local(pol)
            tgt = m.target or m.start_pose
            tx, ty = tgt.local(pol)
            if m.rotate_deg:
                cx, cy = (m.rotate_about.x, m.rotate_about.y) if m.rotate_about else (sx, sy)
                sx, sy = _rotate(sx, sy, cx, cy, m.rotate_deg)
                tx, ty = _rotate(tx, ty, cx, cy, m.rotate_deg)
            mission = Mission(task_text=m.task_prompt or "", target_x=tx, target_y=ty,
                              cruise_alt_m=float(tgt.altitude or m.start_pose.altitude),
                              speed_pref_mps=float(m.pilot.speed or 4.0),
                              start_x=sx, start_y=sy)
            base = {"policy": spec.policy.source, "policy_hash": pol.policy_hash,
                    "episode": cell.episode_id}
            out.append({**base, "kind": "csp_nl", "renderer": "compile_csp(now=None)",
                        "text": _csp_render(pol, mission)})
            if m.clock_start is not None:
                out.append({**base, "kind": "csp_nl",
                            "renderer": f"compile_csp(now={m.clock_start.isoformat()})",
                            "text": _csp_render(pol, mission, now=m.clock_start)})
            if m.task_prompt:
                out.append({**base, "kind": "task", "renderer": "mission.task_prompt",
                            "text": normalise_source(m.task_prompt)})
    if not out:
        raise ParaphraserError("experiments/scenarios.yaml expanded to no cells - the "
                               "coverage check would pass on nothing")
    return out


def scenario_policies() -> dict[str, list[str]]:
    """policy path -> scenario cells (episode ids), from scenario_renderings."""
    out: dict[str, list[str]] = {}
    for r in scenario_renderings():
        if r["kind"] == "csp_nl" and r["episode"] not in out.get(r["policy"], []):
            out.setdefault(r["policy"], []).append(r["episode"])
    return out


def scenario_task_prompts() -> dict[str, list[str]]:
    """mission.task_prompt -> scenario cells. One family today
    (dyn-nfz-spawn-ahead). The moment another scenario gains a task prompt it
    is in scope and needs a stored set."""
    out: dict[str, list[str]] = {}
    for r in scenario_renderings():
        if r["kind"] == "task":
            out.setdefault(r["text"], []).append(r["episode"])
    return out


CITY_POLICIES = {
    "policies/follow_car_citylife.yaml":
        "scripts/run_citylife_follow.ps1 (car mission, -LevelCar)",
    "policies/follow_pedestrian.yaml":
        "scripts/run_citylife_follow.ps1 (person mission, default)",
    "policies/follow_car_citylife_nfz.yaml":
        "scripts/run_citylife_follow.ps1 -PolicyFile (CityLife no-fly zone, 2026-10-03)",
}

TASKS = {
    "task_fly_to_northeast_pad": (
        "fly to the northeast pad at 6 m/s",
        "sitl/ros2_shield_node.py:73 (the KPI rail's mission), demo/run_demo.py:90, "
        "sitl/run_sitl_demo.py:170"),
    "task_follow_a_person": (
        "follow a person",
        "scripts/run_citylife_follow.ps1 default -Object \"a person\" "
        "(demo/follow_vlm.py reads only the noun phrase)"),
    "task_follow_a_red_car": (
        "follow a red car",
        "scripts/run_citylife_follow.ps1 -Object \"a red car\" -LevelCar Car_10 "
        "(demo/follow_vlm.py reads only the noun phrase)"),
    "task_fly_forward_avoid_restricted": (
        "fly forward and avoid restricted areas",
        "demo/real_vla_demo.py main() --instruction default (framed by "
        "build_openvla_prompt) and scripts/demo_real_vla.ps1 - the OpenVLA "
        "pilot, the one model here that reads the instruction text"),
    "task_fly_to_40_40": (
        "fly to (40, 40) at 6 m/s altitude 20",
        "demo/aerialvla_demo.py:321, demo/aerialvla_pas_demo.py:736 (--command, "
        "parsed by the compiler; AerialVLA itself never sees this text)"),
}


def _key_for(stem: str, text: str, full: tuple) -> str:
    rs = rule_set_key(text)
    if rs == full:
        return f"csp_{stem}"
    return f"csp_{stem}_{_sha256(chr(10).join(rs))[:8]}"


def default_sources() -> list[tuple[str, str, dict]]:
    """(suggested source_key, text, origin) for every text in scope, one row
    per distinct exact text, with where it is used (computed now, not frozen).

    Rule texts are listed in BOTH renderings - compile_csp's (each scenario
    cell's mission, with and without its clock) and build_prompt's - because
    both reach a pilot; the stored backend serves the two from one set when
    their sentences agree. The city follow policies have no mission target
    for a region filter, so their CSP is rendered with the region filter off
    (every zone in scope)."""
    from .compiler import Mission
    from .models import load_policy
    rows: dict[str, dict] = {}

    def add(key: str, text: str, kind: str, renderer: str, where: str, **extra) -> None:
        r = rows.setdefault(text, {"key": key, "kind": kind, "renderers": [],
                                   "used_by": [], **extra})
        if renderer not in r["renderers"]:
            r["renderers"].append(renderer)
        if where not in r["used_by"]:
            r["used_by"].append(where)

    renders = scenario_renderings()
    by_policy: dict[str, list[dict]] = {}
    for r in renders:
        by_policy.setdefault(r["policy"], []).append(r)
    for pol_path, rs in by_policy.items():
        pol = load_policy(ROOT / pol_path)
        legacy = _legacy_render(pol)
        full = rule_set_key(legacy)
        stem = Path(pol_path).stem
        meta = {"policy": pol_path, "policy_hash": pol.policy_hash}
        for r in rs:
            if r["kind"] == "task":
                continue
            add(_key_for(stem, r["text"], full), r["text"], "csp_nl", r["renderer"],
                f"experiments/scenarios.yaml: {r['episode']}", **meta)
        add(f"csp_{stem}", legacy, "csp_nl", "build_prompt natural_language_prompt",
            "demo/run_demo.py, sitl/run_sitl_demo.py, sitl/ros2_shield_node.py "
            "(build_prompt)", **meta)
    origin_mission = Mission(task_text="", target_x=0.0, target_y=0.0, cruise_alt_m=0.0,
                             speed_pref_mps=0.0)
    for pol_path, where in CITY_POLICIES.items():
        pol = load_policy(ROOT / pol_path)
        legacy = _legacy_render(pol)
        stem = Path(pol_path).stem
        meta = {"policy": pol_path, "policy_hash": pol.policy_hash}
        add(f"csp_{stem}", legacy, "csp_nl", "build_prompt natural_language_prompt",
            where, **meta)
        text = _csp_render(pol, origin_mission, region=False)
        add(_key_for(stem, text, rule_set_key(legacy)), text, "csp_nl",
            "compile_csp(now=None, region_margin_m=None)", where, **meta)
    for key, (text, where) in TASKS.items():
        add(key, normalise_source(text), "task", "mission text", where)
    for r in renders:
        if r["kind"] == "task" and r["text"] not in rows:
            add("task_" + "_".join(re.findall(r"[a-z0-9]+", r["text"].lower())[:8]),
                r["text"], "task", r["renderer"],
                f"experiments/scenarios.yaml: {r['episode']}")
        elif r["kind"] == "task":
            add(rows[r["text"]]["key"], r["text"], "task", r["renderer"],
                f"experiments/scenarios.yaml: {r['episode']}")
    return [(r["key"], text, {k: v for k, v in r.items() if k != "key"})
            for text, r in rows.items()]


_FLIPS = [("at least", "at most"), ("at most", "at least"), ("below", "above"),
          ("above", "below"), ("under", "over"), ("over", "under"),
          ("no more than", "no less than"), ("no less than", "no more than"),
          ("within", "beyond"), ("lower than", "higher than"),
          ("higher than", "lower than"), ("faster than", "slower than"),
          ("slower than", "faster than"), ("closer than", "farther than"),
          ("or less", "or more"), ("or more", "or less"), ("≤", "≥"), ("≥", "≤"),
          ("<", ">"), ("maximum", "minimum"), ("minimum", "maximum"),
          ("max", "min"), ("tops", "minimum"), ("capped", "floored")]
_SWAPS = [("red", "blue"), ("car", "truck"), ("automobile", "bus"),
          ("person", "dog"), ("pedestrian", "cyclist"), ("pedestrians", "cyclists"),
          ("building", "tree"), ("buildings", "trees"), ("forward", "backward"),
          ("ahead", "behind"), ("straight", "sideways"), ("restricted", "open"),
          ("no-fly", "open"), ("prohibited", "open"), ("off-limits", "open"),
          ("northeast", "southwest"), ("follow", "find"), ("track", "find"),
          ("tail", "find"), ("chase", "find"), ("pursue", "find"),
          ("shadow", "find"), ("target", "tree"), ("subject", "tree")]
# What a number limits. Swapped only inside a clause that carries a number:
# "turn away" -> "speed away" changes no mission slot.
_ATTR_SWAPS = [("centerline", "speed"), ("altitude", "distance"),
               ("speed", "spacing"), ("climb", "speed"), ("turn", "speed")]
_HAS_NUMBER = re.compile(r"\d|\b(?:one|two|three|four|five|six|seven|eight|nine|ten|"
                         r"twenty|forty)(?:[- ]\w+)?\s+(?:met|deg|m\b)", re.IGNORECASE)
_ZONE_WORDS = {"restricted", "no-fly", "prohibited", "off-limits"}
_NEG_DROPS = ["never ", "not ", "don't ", "no ", "nor ", "without "]
_UNIT_SWAPS = [(r"\bm/s\b", "km/h"), (r"\bmetres per second\b", "kilometres per hour"),
               (r"\bdeg/s\b", "rad/s"), (r"(?<=\d) m\b", " ft"),
               (r"\bdegrees per second\b", "radians per second")]
_WORD_NEXT = {w: n for w, n in zip(
    "one two three four five six seven eight nine ten twenty forty-five".split(),
    "two three four five six seven eight nine ten eleven thirty fifty".split())}
_IMPERATIVES = ("fly|go|head|travel|move|proceed|make|take|reach|follow|track|tail|"
                "pursue|shadow|chase|keep|stay|remain|hold|avoid|give|maintain|cap|"
                "limit|stick|cruise|climb|turn|leave|navigate")
_PERMISSIONS = [(r"\bnever enter\b", "may enter"),
                (r"\bis (?:strictly )?off-limits\b", "is not off-limits"),
                (r"\bprohibited\b", "permitted"), (r"\bforbidden\b", "allowed"),
                (r"\bmust (?:never|not) be entered\b", "may be entered")]
_COLOUR_WORDS = "red|blue|green|white|black|orange|yellow|grey|gray|silver"


def mutants(text: str) -> list[tuple[str, str]]:
    """Systematic, meaning-changing mutations of a paraphrase, one at a time.

    Written without the validator's slot extractor, so the mutation test
    measures it rather than agreeing with itself: plain string rewrites of the
    surface - a number, a unit, a comparative, a negation dropped, a zone id, a
    colour, an object, a direction, an action, a whole slot-bearing sentence;
    and, since the 2026-10-06 review, a negation ADDED to an instruction or a
    modal, a ruling after a number ("is not allowed"), a permission, an
    exception ("unless told otherwise"), a condition ("Otherwise,", "While
    climbing,"), a hedge ("Generally,"), a suspension ("These limits are
    optional."), a rate widened ("vertical speed"), a colour moved off or
    widened past its object, and a place given another role ("past the pad").

    The recall it measures is recall on THESE classes. The review found 17 of
    27 hand-written meaning changes accepted while this generator, lacking the
    second group, reported 1721/1721; a class nobody wrote is not measured."""
    out: list[tuple[str, str]] = []
    low = text

    def clause(m: re.Match) -> str:
        a = max(low.rfind(ch, 0, m.start()) for ch in ",.;:!?")
        ends = [i for i in (low.find(ch, m.end()) for ch in ",.;:!?") if i >= 0]
        return low[a + 1:min(ends) if ends else len(low)]

    def sentence(m: re.Match) -> str:
        a = max(low.rfind(ch, 0, m.start()) for ch in ".;!?")
        ends = [i for i in (low.find(ch, m.end()) for ch in ".;!?") if i >= 0]
        return low[a + 1:min(ends) if ends else len(low)]

    def each(pattern: str, repl, label: str, flags=re.IGNORECASE, gate=None):
        for m in re.finditer(pattern, low, flags):
            if gate is not None and not gate(m):
                continue
            r = repl(m) if callable(repl) else repl
            mut = low[:m.start()] + r + low[m.end():]
            if mut != low:
                out.append((f"{label}:{m.group(0)!r}", mut))

    each(r"(?<![\w.'-])\d+(?:\.\d+)?(?![\w.'-]*')",
         lambda m: _fmt(float(m.group(0)) + 1), "number")
    each(r"\b(" + "|".join(sorted(_WORD_NEXT, key=len, reverse=True)) +
         r")\b(?=[\s-]*(?:met|deg|m\b|m/s))",
         lambda m: _WORD_NEXT[m.group(0).lower()], "number-word")
    for pat, rep in _UNIT_SWAPS:
        each(pat, rep, "unit")
    # a comparative only bounds something in a clause with a number in it:
    # "under any circumstances" -> "over any circumstances" changes no slot
    for a, b in _FLIPS:
        each(rf"(?<![\w-]){re.escape(a)}(?![\w-])", b, "bound",
             gate=lambda m: bool(_HAS_NUMBER.search(clause(m))))
    for neg in _NEG_DROPS:
        each(rf"\b{re.escape(neg)}", "", "negation")
    each(r"'[a-z0-9][a-z0-9_\-]*[a-z0-9]'", "the area", "zone-drop")
    each(r"'([a-z0-9][a-z0-9_\-]*[a-z0-9])'",
         lambda m: f"'{m.group(1)}-2'", "zone-rename")
    for a, b in _SWAPS:
        gate = None
        if a in _ZONE_WORDS:
            # "the square no-fly zone 'nfz-square'" describes a named zone;
            # the id carries that slot, the adjective does not
            gate = (lambda m: not re.search(r"'[a-z0-9]", sentence(m)))
        each(rf"(?<![\w-]){re.escape(a)}(?![\w-])", b, "swap", gate=gate)
    for a, b in _ATTR_SWAPS:
        each(rf"(?<![\w-]){re.escape(a)}(?![\w-])", b, "swap",
             gate=lambda m: bool(_HAS_NUMBER.search(clause(m))))
    # the stand-off wildcard narrowed to one class ("whatever you are
    # following" -> "the person you are following")
    each(r"\b(?:anything|whatever|the\s+(?:thing|one|subject|target))(?=\s+(?:that\s+|"
         r"which\s+)?(?:you|it)(?:'re|\s+are|\s+is)?\s+(?:currently\s+)?(?:follow|"
         r"track|tail|pursu|shadow|chas))", "the person", "narrow")
    each(r"\bkeep(?:ing)? out of\b|\bstay(?:ing)? out of\b|\bstay outside\b|"
         r"\bkeep clear of\b|\bstay(?:ing)? clear of\b|\bsteering clear of\b|"
         r"\bavoid(?:ing)?\b", "fly into", "polarity")
    each(r"\bstay(?:s)? inside\b|\bremain inside\b|\bfly only inside\b|"
         r"\bremain in\b|\bstay in\b|\bstick to\b",
         "stay outside", "polarity")
    sents = [s for s in re.split(r"(?<=[.;?!])\s+", text) if s]
    if len(sents) > 1:
        for i, s in enumerate(sents):
            if re.search(r"'[a-z0-9]", s) or _HAS_NUMBER.search(s):
                out.append((f"drop-sentence:{i}", " ".join(sents[:i] + sents[i + 1:])))

    # ---- classes added 2026-10-06 after the independent review accepted 17
    # of 27 meaning changes the generator above could not produce ----
    # an instruction negated in its own clause ("do not follow a red car")
    each(rf"(?:^|(?<=[.;:!?,]\s)|(?<=\band\s)|(?<=\bplease\s))(?:{_IMPERATIVES})\b",
         lambda m: "do not " + m.group(0), "negate-verb")
    each(r"\b(?:must|shall|should)\b(?!\s+(?:not|never)\b)",
         lambda m: m.group(0) + " not", "negate-modal")
    # a ruling after the number ("a speed below 4 m/s is not allowed")
    each(r"(?<![\w.])\d+(?:\.\d+)?\s*(?:m/s|deg/s|m)(?![\w/])",
         lambda m: m.group(0) + " is not allowed", "post-prohibit")
    # permission where there was a prohibition
    for a, b in _PERMISSIONS:
        each(a, b, "permission")
    # conditions, exceptions, hedges, suspensions
    for i, s in enumerate(sents):
        rest_before, rest_after = " ".join(sents[:i]), " ".join(sents[i + 1:])
        core = s[:-1] if s[-1:] in ".;?!" else s
        end = s[len(core):]
        for label, mut in (
                ("exception", core + " unless told otherwise" + end),
                ("conditional", "Otherwise, " + s[:1].lower() + s[1:]),
                ("hedge", "Generally, " + s[:1].lower() + s[1:]),
                ("phase", "While climbing, " + s[:1].lower() + s[1:])):
            if label == "phase" and not _HAS_NUMBER.search(s):
                continue
            out.append((f"{label}:{i}", " ".join(x for x in (rest_before, mut, rest_after) if x)))
    out.append(("override", text.rstrip() + " These limits are optional."))
    # what a rate limits, widened to a different rate ("vertical speed")
    each(r"(?<![\w-])speed(?![\w-])", "vertical speed", "vertical",
         gate=lambda m: bool(_HAS_NUMBER.search(clause(m))))
    # the colour moved off the object, or the object widened past it
    each(rf"\b({_COLOUR_WORDS})\s+(car|automobile|vehicle|person|truck)\b",
         lambda m: f"{m.group(2)} next to the {m.group(1)} one", "colour-rebind")
    each(rf"\b(an?|the)\s+({_COLOUR_WORDS})\s+(car|automobile|vehicle|person|truck)\b",
         lambda m: f"{m.group(0)} or any {m.group(3)}", "widen")
    # the place kept, its role changed ("past the northeast pad")
    each(r"\b(?:to|reach|reaching)\s+the\s+(northeast|north|east)\s+pad\b",
         lambda m: f"past the {m.group(1)} pad", "place-role")
    return out


# =========================================================================== #
# CLI: validate the store, write the report
# =========================================================================== #

def _parser_reading(source: str, texts: list[str]) -> dict | None:
    """What the only text reader in the repo today makes of each paraphrase.

    ConstraintCompiler.parse_command is how a typed mission reaches the stub
    pilot. It is NOT the per-paraphrase robustness KPI (that needs a pilot
    that reads text, flown) - it is the cheapest honest preview of one. Each
    reading is taken twice, with real defaults and with sentinel defaults, so a
    value that only matches because the parser fell back to its default (6 m/s
    is the default speed) is reported as such instead of counted as read."""
    from .compiler import ConstraintCompiler
    from .models import load_policy
    cc = ConstraintCompiler(load_policy(ROOT / "policies" / "sim_demo_policy.yaml"))
    SENT = -12345.0
    try:
        ref = cc.parse_command(source)
    except ValueError:
        return None

    def read(t: str) -> dict:
        try:
            m = cc.parse_command(t)
            s = cc.parse_command(t, default_alt=SENT, default_speed=SENT)
        except ValueError as e:
            return {"text": t, "parsed": False, "error": str(e)[:120]}
        same = (m.target_x, m.target_y, m.cruise_alt_m, m.speed_pref_mps) == \
               (ref.target_x, ref.target_y, ref.cruise_alt_m, ref.speed_pref_mps)
        return {"text": t, "parsed": True, "same_mission": same,
                "speed_read": s.speed_pref_mps != SENT,
                "alt_read": s.cruise_alt_m != SENT,
                "mission": [m.target_x, m.target_y, m.cruise_alt_m, m.speed_pref_mps]}

    src_s = cc.parse_command(source, default_alt=SENT, default_speed=SENT)
    rows = [read(t) for t in texts]
    agree = [r for r in rows if r.get("same_mission")]
    by_default = [r for r in agree if (src_s.speed_pref_mps != SENT and not r["speed_read"])
                  or (src_s.cruise_alt_m != SENT and not r["alt_read"])]
    return {"reference": [ref.target_x, ref.target_y, ref.cruise_alt_m, ref.speed_pref_mps],
            "n": len(rows), "same_mission": len(agree),
            "same_only_by_default": len(by_default),
            "null_identity_same_mission": True, "rows": rows}


def store_digest(store_dir: str | Path | None = None) -> str:
    """One hash over every stored set, so a report can be checked against the
    store it describes (a report older than the store is stale, not current)."""
    d = Path(store_dir) if store_dir is not None else STORE_DIR
    parts = []
    for f in sorted(d.glob("*.json")):
        doc = json.loads(f.read_text(encoding="utf-8"))
        if doc.get("schema") == STORE_SCHEMA:
            parts.append(f.name + ":" + _sha256(json.dumps(doc, sort_keys=True)))
    return _sha256("|".join(parts))[:16]


def validator_digest() -> str:
    """Hash of this module's source: the report's numbers were produced by
    exactly this validator and mutation generator."""
    return _sha256(Path(__file__).read_text(encoding="utf-8"))[:16]


# --------------------------------------------------------------- the nulls -- #
#
# What would a validator with no skill score on the same mutants? Each null is
# run against the SOURCE, exactly as validate() is, and is scored on BOTH
# halves: how many stored paraphrases it accepts (a null that refuses
# everything "catches" every mutant) and how many mutants it refuses.

_BAG_UNITS = re.compile(r"\b(?:m/s|deg/s|km/h|mph|ft|m)\b")
_BAG_WORDS = re.compile(
    r"\b(?:red|blue|green|white|black|orange|yellow|grey|gray|silver|north|south|"
    r"east|west|northeast|northwest|southeast|southwest|forward|backward|left|right|"
    r"car|person|pedestrian|pedestrians|building|buildings|truck|vehicle|never|not|"
    r"no)\b")


def _null_accept_all(source: str, cand: str) -> bool:
    """Accepts anything that is not word-for-word the source."""
    return _identity_words(source) != _identity_words(cand)


def _null_digits(source: str, cand: str) -> bool:
    """Accepts when the multiset of digit strings is unchanged."""
    return sorted(re.findall(_DIGITS, _norm(source))) == sorted(re.findall(_DIGITS, _norm(cand)))


def _null_slot_bag(source: str, cand: str) -> bool:
    """Accepts when the SETS of digits, units, quoted ids and a fixed list of
    slot words (colours, compass words, object nouns, negators) are unchanged:
    the simplest validator a person would write in an afternoon."""
    def bag(t: str) -> tuple:
        t = _norm(t)
        return (frozenset(re.findall(_DIGITS, t)), frozenset(_BAG_UNITS.findall(t)),
                frozenset(m.group("id") for m in _QUOTED_ID.finditer(t)),
                frozenset(_BAG_WORDS.findall(t)))
    return bag(source) == bag(cand)


NULL_VALIDATORS = {
    "accept_all_but_exact_copy": _null_accept_all,
    "digits_multiset": _null_digits,
    "slot_word_bag": _null_slot_bag,
}


def load_probes(store_dir: str | Path | None = None) -> dict:
    """experiments/paraphrases/probes.json: hand-written (source, candidate,
    meaning_changed) triples. Missing file -> {} (the report then says the
    probe rates were not measured, rather than reporting zero)."""
    d = Path(store_dir) if store_dir is not None else STORE_DIR
    f = d / "probes.json"
    if not f.is_file():
        return {}
    doc = json.loads(f.read_text(encoding="utf-8"))
    if doc.get("schema") != PROBES_SCHEMA:
        raise StoreIntegrityError(f"{f.name}: unknown schema {doc.get('schema')!r}")
    return doc


def score_probes(probes: dict) -> dict:
    out = {}
    for name, pset in (probes.get("sets") or {}).items():
        changed = [p for p in pset["probes"] if p["meaning_changed"]]
        faithful = [p for p in pset["probes"] if not p["meaning_changed"]]
        fa = [p for p in changed if validate(p["source"], p["candidate"]).ok]
        fr = [p for p in faithful if not validate(p["source"], p["candidate"]).ok]
        out[name] = {
            "status": pset["status"],
            "meaning_changing": len(changed), "false_accepts": len(fa),
            "faithful": len(faithful), "false_refusals": len(fr),
            "false_accept_texts": [p["candidate"] for p in fa],
            "false_refusal_texts": [p["candidate"] for p in fr]}
    return out


def validate_store(store_dir: str | Path | None = None) -> dict:
    store = load_store(store_dir)
    _places()                                  # sets PLACES_SOURCE for the report
    by_source = []
    tot = ok = mut_total = mut_killed = 0
    by_class: dict[str, list[int]] = {}
    nulls = {k: {"stored_accepted": 0, "mutants_refused": 0} for k in NULL_VALIDATORS}
    for src, e in sorted(store.items(), key=lambda kv: kv[1].source_key):
        verdicts = [validate(src, i["text"]) for i in e.paraphrases]
        tot += len(verdicts)
        ok += sum(v.ok for v in verdicts)
        for item in e.paraphrases:
            for k, fn in NULL_VALIDATORS.items():
                nulls[k]["stored_accepted"] += fn(src, item["text"])
        survivors = []
        m_n = m_k = 0
        for item in e.paraphrases:
            for label, mut in mutants(item["text"]):
                m_n += 1
                cls = label.split(":", 1)[0]
                tally = by_class.setdefault(cls, [0, 0])
                tally[0] += 1
                if not validate(src, mut).ok:
                    m_k += 1
                    tally[1] += 1
                else:
                    survivors.append({"index": item["index"], "mutation": label,
                                      "text": mut})
                for k, fn in NULL_VALIDATORS.items():
                    nulls[k]["mutants_refused"] += not fn(src, mut)
        mut_total += m_n
        mut_killed += m_k
        nov = sorted(v.novelty for v in verdicts)
        row = {"source_key": e.source_key, "file": e.path.name, "kind": e.kind,
               "source_text": src, "n": len(verdicts), "accepted": sum(v.ok for v in verdicts),
               "refused": [{"index": i, "reasons": list(v.reasons)}
                           for i, v in enumerate(verdicts) if not v.ok],
               "identity_refused": not validate(src, src).ok,
               "novelty": {"min": round(nov[0], 3), "median": round(nov[len(nov) // 2], 3),
                           "max": round(nov[-1], 3)},
               "mutants": m_n, "mutants_refused": m_k, "survivors": survivors}
        if e.kind == "task":
            row["parser_reading"] = _parser_reading(src, [i["text"] for i in e.paraphrases])
        by_source.append(row)
    probes = load_probes(store_dir)
    return {"schema": REPORT_SCHEMA, "store_digest": store_digest(store_dir),
            "validator_digest": validator_digest(),
            "probes_digest": (_sha256(json.dumps(probes, sort_keys=True))[:16]
                              if probes else None),
            "places_source": PLACES_SOURCE,
            "command": "python -m guardrail.paraphraser validate",
            "sources": len(by_source), "paraphrases": tot, "accepted": ok,
            "identity_null_refused": sum(r["identity_refused"] for r in by_source),
            "mutants": mut_total, "mutants_refused": mut_killed,
            "mutants_by_class": {k: {"generated": v[0], "refused": v[1]}
                                 for k, v in sorted(by_class.items())},
            "mutation_recall_note": (
                "recall on the mutation classes mutants() generates, and on no "
                "other class: a kind of meaning change nobody wrote a mutation "
                "for is not measured here (see probes)"),
            "null_validators": {k: {**v, "note": NULL_VALIDATORS[k].__doc__.split("\n")[0]}
                                for k, v in nulls.items()},
            "probes": (score_probes(probes) if probes
                       else "not measured: experiments/paraphrases/probes.json missing"),
            "per_source": by_source}


def coverage(store_dir: str | Path | None = None) -> list[dict]:
    """Every in-scope text and the stored set that serves it (or None)."""
    be = StoredBackend(store_dir)
    rows = []
    for key, text, origin in default_sources():
        e = be.lookup(text)
        rows.append({"key": key, "text": text, "kind": origin["kind"],
                     "renderers": origin["renderers"], "used_by": origin["used_by"],
                     "policy": origin.get("policy"), "policy_hash": origin.get("policy_hash"),
                     "served_by": e.path.name if e else None,
                     "matched_by": (None if e is None else
                                    "text" if e.source_text == normalise_source(text)
                                    else "rule set"),
                     "frozen_policy_hash": (e.origin.get("policy_hash") if e else None)})
    return rows


def _main(argv: list[str] | None = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(prog="python -m guardrail.paraphraser")
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("validate", help="validate the stored sets, write the report")
    v.add_argument("--store", default=str(STORE_DIR))
    v.add_argument("--no-write", action="store_true")
    s = sub.add_parser("show", help="print n paraphrases of a text")
    s.add_argument("text")
    s.add_argument("--n", type=int, default=3)
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--backend", default="auto")
    sub.add_parser("stale", help="list every in-scope text and the set serving it")
    args = ap.parse_args(argv)

    if args.cmd == "show":
        for p in paraphrase(args.text, args.n, args.seed, args.backend):
            print(f"{p.paraphrase_id}  [{p.backend}#{p.index}]  {p.text}")
        return 0
    if args.cmd == "stale":
        rows = coverage()
        missing = 0
        for r in rows:
            if r["served_by"] is None:
                missing += 1
                print(f"MISSING {r['key']} ({', '.join(r['renderers'])}): {r['text']}")
                continue
            drift = (r["policy_hash"] and r["frozen_policy_hash"]
                     and r["policy_hash"] != r["frozen_policy_hash"])
            print(f"ok      {r['key']:<34} <- {r['served_by']} (by {r['matched_by']}; "
                  f"{len(r['used_by'])} use(s); {', '.join(r['renderers'])})"
                  + ("  NOTE policy_hash changed since the set was frozen; the "
                     "text still matches" if drift else ""))
        print(f"{missing} in-scope text(s) without a stored set, of {len(rows)}")
        return 1 if missing else 0

    rep = validate_store(args.store)
    for r in rep["per_source"]:
        pr = r.get("parser_reading")
        extra = (f"  parser same-mission {pr['same_mission']}/{pr['n']} "
                 f"(only by default {pr['same_only_by_default']})") if pr else ""
        print(f"{r['source_key']:<40} {r['accepted']}/{r['n']} accepted  "
              f"identity refused={r['identity_refused']}  novelty median "
              f"{r['novelty']['median']:.2f}  mutants {r['mutants_refused']}/"
              f"{r['mutants']}{extra}")
    print(f"\nstored paraphrases accepted : {rep['accepted']}/{rep['paraphrases']}")
    print(f"identity null refused       : {rep['identity_null_refused']}/{rep['sources']}")
    print(f"mutants refused             : {rep['mutants_refused']}/{rep['mutants']}"
          f"  (generated classes only)")
    for k, nv in rep["null_validators"].items():
        print(f"  null {k:<26}: accepts {nv['stored_accepted']}/{rep['paraphrases']} "
              f"stored, refuses {nv['mutants_refused']}/{rep['mutants']} mutants")
    if isinstance(rep["probes"], dict):
        for name, pr in rep["probes"].items():
            print(f"probes {name} ({pr['status']}): false accepts "
                  f"{pr['false_accepts']}/{pr['meaning_changing']}, false refusals "
                  f"{pr['false_refusals']}/{pr['faithful']}")
    else:
        print(f"probes: {rep['probes']}")
    if not args.no_write:
        out = Path(args.store) / "validation_report.json"
        with open(out, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(rep, indent=2, ensure_ascii=False) + "\n")
        print(f"\nwrote {out}")
    print(f"reproduce: {rep['command']}")
    return 0 if rep["accepted"] == rep["paraphrases"] else 1


if __name__ == "__main__":
    raise SystemExit(_main())
