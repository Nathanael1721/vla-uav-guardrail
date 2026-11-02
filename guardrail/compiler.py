"""
Prefix Compiler - the "before the VLA" half of the Guardrail.

In the grant, WP2 (the Prefix Constraint Compiler) is policy bundle + mission
context -> Constraint Summary Pack (CSP), and the CSP is its only output
(Prefix Compiler PDF p1). Parsing operator text into a mission is NOT part of
the WP2 spec; it lives here only because the stub pilot needs a Mission.

Where the CSP goes today: `compile_csp` builds it and the flight scripts write
it to disk (demo/real_vla_demo.py, with `--csp on`, writes one JSON per policy
generation and prepends its text to the OpenVLA instruction). No flown VLA has
read a CSP yet: no flight has been made with that code. Say "generated and
saved", not "the prompt the VLA reads", until one has.

Three jobs live here.

1. COMMAND -> MISSION (`parse_command`). A natural-language command resolved
   into a structured Mission. v0 parsing is deliberately simple: a named-place
   registry + coordinate regex. A value the text does not state falls back to
   a default, and the Mission says which ones did (`defaults_used`), so a
   fallback is never mistaken for a reading.

2. POLICY + MISSION -> CSP (`compile_csp`), the grant's Prefix Compiler
   (Prefix Compiler PDF p3, "Pipeline"):

       policy IR + mission context
         -> spatial + temporal filter  rules not in force anywhere in
                                       [now, now + lookahead], and keep-out zones
                                       outside the mission region, are set aside
         -> risk grade                 0.5 severity + 0.3 proximity
                                       + 0.2 time-criticality (PDF p4; weights
                                       configurable, csp.RiskWeights)
         -> order + truncate           every P0 rule is kept; P1/P2 fill the
                                       token budget by descending risk; P0 alone
                                       over budget raises CSPBudgetExceeded
         -> render                     Jinja2 sentences (guardrail/templates/),
                                       allowed/forbidden action sets, geometry
                                       refs, per-rule relevance explanations
         -> CSP                        the typed record in guardrail/csp.py

   Every runtime rule type renders, the three hot-apply classes included
   (dynamic_nfz, time_window_switch, corridor_swap; templates since
   2026-10-08, when they joined models.RUNTIME_TYPES). A rule a switch or swap
   acts on is timed by `_RuleClock`, the Shield's own logic, so the CSP never
   leaves out a rule the Shield enforces. The multi-scale polygons behind each
   P0 `geometry_ref` are the IR's (guardrail/ir.py): picked here, never
   computed here (WP2-16).

   It is a stateless function, as PDF p1 requires, GIVEN A CLOCK: same policy
   (hash + generation) + same mission + same `now` -> byte-identical CSP, in
   any process. Nothing is random, so the spec's "rule-selection seed" has
   nothing to seed; ties in risk are broken by rule id. The one input that can
   come from outside the arguments is `issued_at`: it is the `issued_at`
   argument if given, else `now`, and only when neither is given the wall
   clock. So a CSP compiled without a clock differs from run to run in that one
   field, and `csp.csp_content_hash` - every field except the issue stamp - is
   the identity to compare such CSPs by (the KPI report does).

3. POLICY -> COST MAP (`rasterise_cost_map`), the grant's cost-map adapter
   ("path planners only", PDF p4): the rules in force at one instant on a 2-D
   grid with the grant's conventions (5 m cells, nfz_hard inf, corridor_pref
   0, corridor_outside 50), exported as .npz + a JSON header and referenced
   from a CSP's `cost_map_ref` when passed to compile_csp. No planner reads it
   yet; `cost_map_agreement` scores it against the Shield with two nulls.
   docs/DESIGN-cost-map-adapter.md.

What the CSP is FOR, stated so nobody over-reads it: it is advice to the model,
not enforcement. The Shield enforces every rule in the policy whether or not
the CSP mentioned it. A rule the CSP filtered or dropped is still a rule; the
CSP's `selection` says which and why, so a cut is never silent.

WHICH TEXT IS CANONICAL (decided 2026-10-06). The same rules render two ways:
`compile_csp(...).natural_language_prompt` (filtered, risk order, P0 first) and
`build_prompt`'s type-ordered sentence list. The CANONICAL text is
compile_csp's: it is the grant's CSP, the text demo/real_vla_demo.py injects,
and the text the stored paraphrase sets are keyed to (experiments/paraphrases/).
build_prompt's rendering stays for its legacy callers; the paraphraser serves it
from the same sets by rule-set matching (paraphraser.rule_set_key). One rule
follows: the sentence templates (guardrail/templates/sentence/*.j2) are frozen
against the stored sets. Changing a template changes every CSP text, and the
answer is a NEW frozen paraphrase set generated from the new text, never an
edit of stored text; tests/test_paraphraser.py
(test_every_instruction_in_scope_has_a_stored_set) fails until that is done.

The older `summary_pack()` / `build_prompt()` surface is kept exactly as it was
for the callers that read it (demo/run_demo.py, sitl/run_sitl_demo.py,
sitl/ros2_shield_node.py, tools/build_architecture_svg.py, tests/test_bundle.py).
A reader of `build_prompt` can see two changes, both qualifiers appended to the
old sentence. The fix: a rule with a time window now says so ("Never enter
zone 'nfz-school' (in force Mon-Fri 07:30-17:30).") instead of reading as a
permanent ban. And a SOFT rule now says " (soft limit)" before the full stop,
because the old text gave a soft rule the same "Never" as a hard one. Every
HARD rule without a time window renders byte for byte what it did on
2026-09-01 (all 29 shipped policies are hard-only, so only their two windowed
rules changed).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
import yaml
from pydantic import BaseModel, Field

from . import csp as _csp
from .csp import (CSP, CSP_VERSION, AllowedActionSet, CSPBudgetExceeded,
                  CSPUnenforcedRules, ForbiddenActionSet, P0Entry, RiskWeights,
                  RuleDecision, Selection, TokenCounter)
from .geometry import nearest_on_polyline, point_in_fence
from .ir import PolicyIR, is_moving
from .models import (DAYS, RUNTIME_TYPES, AltitudeEnvelope, CircleFence, Corridor,
                     CorridorSwap, DynamicNFZ, KinematicEnvelope,
                     ObstacleClearance, Policy, PolygonFence, SubjectStandoff,
                     TimeWindowSwitch)
from .shield import recurrence_active

# Keep-OUT zones and keep-IN tubes. CircleFence is a PolygonFence (its vertices
# are the derived 32-gon); DynamicNFZ and CorridorSwap are not subclasses of
# the fence and the corridor, so every test below names both. Until 2026-10-08
# the compiler tested PolygonFence and Corridor alone, which was right only
# while the loaders refused the hot-apply types (models.RUNTIME_TYPES).
_ZONES = (PolygonFence, DynamicNFZ)
_TUBES = (Corridor, CorridorSwap)

# --------------------------------------------------------------------------- #
# Defaults, each with its reason
# --------------------------------------------------------------------------- #

# The spec's worked example (PDF p2) uses lookahead_s 5.0. Also roughly what a
# 3-5 s Shield lookahead plus one OpenVLA decision (~1 s) spans.
DEFAULT_LOOKAHEAD_S = 5.0

# Tokens of natural_language_prompt, counted with csp.DEFAULT_TOKEN_COUNTER.
# The spec leaves the number open ("1k? 2k?", PDF p6). 256 because:
#   - OpenVLA-7B has llm_max_length 2048 and spends 256 tokens on the image
#     (224 px, patch 14 -> 16 x 16), plus ~20 for its prompt template, the
#     instruction and the 7 action tokens; so ~1.7k is the hard ceiling;
#   - the ceiling is not the useful limit: OpenVLA was trained on short task
#     phrases, and every token prepended moves its input further from what it
#     saw, so the prefix should be as short as the rules allow;
#   - the 29 shipped policies need 54-106 exact tokens for ALL their rules
#     (measured 2026-10-06), so 256 cuts nothing today and leaves 2.4x headroom.
# Override per call; the budget used is written into every CSP.
DEFAULT_BUDGET_TOKENS = 256

# The mission region is the path's bounding box widened by this much (plus the
# distance flyable in one lookahead). The spec's own relevance example is
# "mission target is within 200m of polygon" (PDF p2). Only keep-OUT zones are
# ever filtered by region; see _in_region.
DEFAULT_REGION_MARGIN_M = 200.0

# A coarse polygon has at most this many vertices; above it, the oriented
# bounding rectangle. The IR computes it (guardrail/ir.py); this is its limit,
# re-exported for callers that read it here. See multiscale_geometry.
from .ir import COARSE_MAX_VERTICES  # noqa: E402

# Severity in [0, 1]: "hard >> soft, P0 > P1 > P2" (PDF p4). The grant gives the
# order, not the numbers. With these, a P1 rule outranks a P2 rule of equal
# proximity and time-criticality, and severity stays the largest single term
# under the default weights (a hard P0 rule contributes 0.5 of a possible 1.0).
# P0 rules are never ranked against the others anyway: all of them are kept.
SEVERITY = {"P0": 1.0, "P1": 0.6, "P2": 0.3}
SOFT_FACTOR = 0.5

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"

# A lookahead longer than this is a mission plan, not a lookahead.
_MAX_LOOKAHEAD_S = 7 * 24 * 3600.0

_FOLLOW_WORDS = re.compile(r"\b(follow|track|chase|escort|shadow)\w*", re.I)


def _count_types(policy: Policy) -> dict:
    """How many rules of each kind, so a reader can see at a glance whether a
    class they expected is missing entirely."""
    out: dict = {}
    for c in policy.constraints:
        out[c.type] = out.get(c.type, 0) + 1
    return out


# Named places the operator may refer to (stand-in for a map service).
PLACES = {
    "northeast pad": (30.0, 30.0),
    "north pad": (35.0, 0.0),
    "east pad": (0.0, 35.0),
    "home": (0.0, 0.0),
}


# The altitude phrase. Until 2026-10-06 this was
#   (?:alt|altitude|height|tinggi)\D{0,3}(\d+)  |  (\d+)\s*m\s+(?:alt|altitude|height)
# which reads "altitude 20" and "alt: 20" but not "at an altitude of 20" or
# "at a height of 20 m": " of " is four non-digits, so the text fell through
# to the 15 m default with nothing to say so (two of the eight stored
# paraphrases of "fly to (40, 40) at 6 m/s altitude 20" would have flown 5 m
# low). The old forms are all still read: the connective is optional and the
# old \D{0,3} tail is kept after it.
#
# The first version of this widening (also 2026-10-06) read the TARGET as the
# altitude: "fly at low altitude to (5, 5)" gave 5 m and "hold altitude to
# (40, 40)" gave 40 m, with defaults_used empty - a wrong number presented as
# read from the text. Three guards now, each enough for those commands:
# parse_command strips the coordinate it used as the target before looking
# for an altitude (as it strips the speed); the gap after the connective may
# not contain "("; and a number followed by ", <number>" is a coordinate
# pair, never an altitude. The number may not stop part-way through a longer
# one ("(?!\d)(?!\.\d)"), so the look-ahead cannot be dodged by backtracking.
_ALT_WORD = r"(?:altitude|alt|height|ketinggian|tinggi)"
_ALT_NUM = r"(\d+(?:\.\d+)?)(?!\d)(?!\.\d)(?!\s*,\s*-?\d)"
_ALT_RE = re.compile(
    _ALT_WORD + r"\s*(?:(?:of|is|at|to|ke|sebesar|setinggi)\b\s*)?[^\d(]{0,3}?" + _ALT_NUM
    + r"|(\d+(?:\.\d+)?)\s*(?:m|meters?|metres?)\s+"
    r"(?:altitude|alt|height|ketinggian|tinggi|high|agl)\b")


class Mission(BaseModel):
    task_text: str                 # the operator's original words
    target_x: float
    target_y: float
    cruise_alt_m: float
    speed_pref_mps: float          # what the operator ASKED for (may be illegal!)
    # Added 2026-10-06 for the Prefix Compiler's mission context ("pose, home,
    # target", PDF p3). Optional with the old implied values, so every existing
    # caller and every Mission built before keeps meaning the same thing.
    start_x: float = 0.0           # where the mission starts: home / current pose
    start_y: float = 0.0
    mission_id: str | None = None  # None: derived from the mission's content
    # Added 2026-10-06: which fields parse_command did NOT read from the text
    # and filled from its defaults ("cruise_alt_m", "speed_pref_mps"). Until
    # then "at an altitude of 20" silently flew at the 15 m default. Not part
    # of the mission's identity (_mission_id ignores it) and not read by
    # compile_csp, so no stored CSP changes.
    defaults_used: list[str] = Field(default_factory=list)


# --------------------------------------------------------------------------- #
# Templates
# --------------------------------------------------------------------------- #

_ENV = None


def _fmt_g(x) -> str:
    """The legacy f-string's `:g`, so templated text matches it byte for byte."""
    return format(float(x), "g")


def _fmt_m(x) -> str:
    """A distance in metres to 0.1 m, without a trailing '.0'."""
    s = f"{float(x):.1f}"
    if s.endswith(".0"):
        s = s[:-2]
    return "0" if s == "-0" else s


def _fmt_rate(x) -> str:
    """A speed (m/s) or turn rate (deg/s) to 0.1, as `_fmt_m`, except that a
    non-zero rate never prints as "0": a zone creeping at 0.04 m/s is not
    still (one significant digit then, "0.04")."""
    s = _fmt_m(x)
    if s == "0" and float(x) != 0.0:
        return format(float(x), ".1g")
    return s


def _env():
    """The Jinja2 environment, built once.

    autoescape OFF: this is plain text for a model, and HTML escaping would turn
    every quoted rule id into &#39;. StrictUndefined: a template that names a
    variable nobody passed must raise, not print "Keep at least  m away".
    Imported lazily so `parse_command` keeps working where jinja2 is absent
    (it is present in vla-real, vla-drone and the WSL ROS 2 Python).
    """
    global _ENV
    if _ENV is None:
        try:
            import jinja2
        except ImportError as e:              # pragma: no cover
            raise ImportError(
                "guardrail.compiler renders rule text from Jinja2 templates "
                "(Prefix Compiler PDF p6: 'NL template renderer - Jinja2 "
                "templates + Python function'); install jinja2") from e
        env = jinja2.Environment(
            loader=jinja2.FileSystemLoader(str(TEMPLATE_DIR)),
            autoescape=False, undefined=jinja2.StrictUndefined,
            keep_trailing_newline=False)
        env.filters["g"] = _fmt_g
        env.filters["m"] = _fmt_m
        env.filters["rate"] = _fmt_rate
        _ENV = env
    return _ENV


def render_template(name: str, ctx: dict) -> str:
    """Render one template to a single line of text."""
    text = _env().get_template(name).render(**ctx).strip()
    if "\n" in text:
        raise ValueError(f"template {name} rendered more than one line: {text!r}")
    return text


def window_text(rule) -> str | None:
    """A rule's schedule in words ("Mon-Fri 07:30-17:30"), or None if always on.

    Days are written as runs (Mon-Fri, Sat-Sun) when contiguous; a window that
    ends before it starts wraps past midnight, as Recurrence defines, and is
    marked "overnight" so "22:00-06:00" cannot be misread as an empty window.
    A schedule that covers every minute of every day is "always", so it gets
    no clause.
    """
    vt = getattr(rule, "valid_time", None)
    if vt is None or vt.recurrence is None:
        return None
    rec = vt.recurrence
    idx = sorted({DAYS.index(d) for d in rec.days})
    full_day = rec.start_time == "00:00" and rec.end_time in ("23:59", "24:00")
    if len(idx) == 7 and full_day:
        return None
    if len(idx) == 7:
        days = "every day"
    elif idx and idx == list(range(idx[0], idx[-1] + 1)) and len(idx) > 2:
        days = f"{DAYS[idx[0]]}-{DAYS[idx[-1]]}"
    else:
        days = ", ".join(DAYS[i] for i in idx) or "no day"
    if full_day:
        return days
    clock = f"{rec.start_time}-{rec.end_time}"
    if rec._mins(rec.end_time) < rec._mins(rec.start_time):
        clock += " overnight"
    return f"{days} {clock}"


def template_type(kind: str, rule_or_cls) -> str:
    """The rule type whose `<kind>/<type>.j2` template renders this rule: its
    own, else the nearest base class's that has one.

    A rule type that subclasses another inherits its rendering. CircleFence is
    a PolygonFence (its vertices are the derived 32-gon, models.py), so until
    it has its own templates it reads as the keep-out zone it is: "Never enter
    zone 'x'." Without this the lookup used `rule.type` alone, and a policy
    with one circle_fence could not compile a CSP at all (TemplateNotFound),
    though the Shield enforced the circle. A type with no template on its whole
    chain returns its own name, so the loader names the file that is missing.
    """
    cls = rule_or_cls if isinstance(rule_or_cls, type) else type(rule_or_cls)
    own = None
    for c in cls.__mro__:
        f = (getattr(c, "model_fields", None) or {}).get("type")
        args = getattr(getattr(f, "annotation", None), "__args__", ()) if f else ()
        if not args or not isinstance(args[0], str):
            continue
        own = own or args[0]
        if (TEMPLATE_DIR / kind / f"{args[0]}.j2").is_file():
            return args[0]
    return own or getattr(rule_or_cls, "type", "unknown")


def render_sentence(rule, show_priority: bool = False) -> str:
    """One rule as one sentence (templates/sentence/<type>.j2)."""
    return render_template(f"sentence/{template_type('sentence', rule)}.j2", {
        "r": rule, "window": window_text(rule),
        "soft": rule.constraint_type == "soft",
        "show_priority": show_priority, "priority": rule.priority})


def render_summary(rule) -> str:
    """One rule as a short clause for P1_summary / P2_summary."""
    return render_template(f"summary/{template_type('summary', rule)}.j2", {
        "r": rule, "window": window_text(rule),
        "soft": rule.constraint_type == "soft"})


# --------------------------------------------------------------------------- #
# Geometry for SUMMARISING, not enforcing.
#
# The distance helpers below are pure Python: they only score and describe,
# and nothing the Shield decides depends on them. Where a check does matter for
# safety (does this action enter a zone?) the code calls geometry.point_in_fence,
# the Shield's own test. The multi-scale polygons are not computed here at all:
# they are read from the compiled IR (guardrail/ir.py), the grant's "the Prefix
# Compiler picks the scale, never re-computes" (card WP2-16).
# --------------------------------------------------------------------------- #

Pt = tuple[float, float]


def _cross(o: Pt, a: Pt, b: Pt) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _point_in_poly(x: float, y: float, poly: list[Pt]) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


def _pt_seg(px: float, py: float, a: Pt, b: Pt) -> float:
    vx, vy = b[0] - a[0], b[1] - a[1]
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px - a[0]) * vx + (py - a[1]) * vy) / L2))
    return math.hypot(px - (a[0] + t * vx), py - (a[1] + t * vy))


def _segs_cross(a: Pt, b: Pt, c: Pt, d: Pt) -> bool:
    d1, d2 = _cross(c, d, a), _cross(c, d, b)
    d3, d4 = _cross(a, b, c), _cross(a, b, d)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def _seg_seg(a: Pt, b: Pt, c: Pt, d: Pt) -> float:
    if _segs_cross(a, b, c, d):
        return 0.0
    return min(_pt_seg(*a, c, d), _pt_seg(*b, c, d),
               _pt_seg(*c, a, b), _pt_seg(*d, a, b))


def _seg_poly(a: Pt, b: Pt, poly: list[Pt]) -> float:
    """Horizontal distance from segment ab to a polygon (0 if they touch)."""
    if _point_in_poly(*a, poly) or _point_in_poly(*b, poly):
        return 0.0
    n = len(poly)
    return min(_seg_seg(a, b, poly[i], poly[(i + 1) % n]) for i in range(n))


# The compiled IR, once per policy hash (a hot-apply bumps the generation and
# with it the hash, so a stale IR is never served). The grant puts the
# multi-scale cache in the IR ("the policy bundle's IR already carries
# pre-computed multi-scale polygons ... The Prefix Compiler picks the scale,
# never re-computes", PDF p1). Until 2026-10-08 this module computed its own
# hull and oriented rectangle beside the IR's (guardrail/ir.py, the same rule
# twice, kept equal by a test); it now reads the IR's.
_IR_CACHE: dict[str, PolicyIR] = {}
_IR_CACHE_MAX = 64


def compiled_ir(policy: Policy) -> PolicyIR:
    """The PolicyIR compiled from `policy`, memoised by its hash - and by
    whether its multi-scale geometry came with it from a bundle
    (`policy.shipped_multiscale`, WP2-16): a policy loaded from a bundle reads
    the bundle's scales, and is never served an IR that computed its own."""
    key = (policy.policy_hash, policy.shipped_multiscale is not None)
    hit = _IR_CACHE.get(key)
    if hit is not None and hit.is_current_for(policy):
        return hit
    ir = PolicyIR.from_policy(policy)
    if len(_IR_CACHE) >= _IR_CACHE_MAX:
        _IR_CACHE.pop(next(iter(_IR_CACHE)))
    _IR_CACHE[key] = ir
    return ir


def multiscale_geometry(policy: Policy) -> dict[str, dict[str, list[Pt]]]:
    """{rule id: {scale: vertices}} for every zone and tube, READ FROM THE IR
    (`PolicyIR.multiscale`); nothing is computed here.

    fine   - the vertices as authored (the Shield adds margin_m itself); a
             corridor's or corridor_swap's centerline;
    coarse - zones only: a polygon that CONTAINS the fine one (its convex hull,
             or the minimum-area oriented rectangle when the hull has more than
             COARSE_MAX_VERTICES vertices). Containment is the point: a coarse
             keep-out zone that cut a corner would tell the model a smaller
             zone than the Shield enforces;
    bbox   - zones only: the axis-aligned bounding rectangle.
    A keep-IN tube is never coarsened: that would widen where the model
    believes it may fly. A moving dynamic_nfz is listed at its declared
    position.
    """
    return compiled_ir(policy).multiscale


_REF = re.compile(r"^(fine|coarse|bbox|param):(.+)@v([^/@]+)/g(\d+)$")


def resolve_geometry_ref(policy: Policy, ref: str) -> list[Pt] | None:
    """Vertices for a P0Entry.geometry_ref, or None for a "param:" ref.

    Refuses a ref whose version or generation is not this policy's: after a
    mid-flight hot-apply bumps the generation, a CSP from before must not
    quietly resolve against geometry it never described. Geometric refs are
    resolved by the IR (`PolicyIR.resolve_geometry_ref`).
    """
    m = _REF.match(ref)
    if not m:
        raise ValueError(f"not a geometry ref: {ref!r}")
    scale, rid, ver, gen = m.group(1), m.group(2), m.group(3), int(m.group(4))
    if ver != policy.version or gen != policy.generation:
        raise ValueError(f"{ref!r} is for v{ver}/g{gen}; policy "
                         f"{policy.policy_id} is v{policy.version}/g{policy.generation}")
    if scale == "param":
        if not any(c.id == rid for c in policy.constraints):
            raise ValueError(f"no rule {rid!r} in {policy.policy_id}")
        return None
    return compiled_ir(policy).resolve_geometry_ref(ref)


# --------------------------------------------------------------------------- #
# Filter and grade
# --------------------------------------------------------------------------- #

def _activity(rule, now: datetime | None, lookahead_s: float) -> dict:
    """Is the rule in force at `now`, at any time in [now, now + lookahead],
    and does it switch inside that window?

    Recurrence is minute-grained, so the window is sampled at `now` and at
    every minute boundary up to its end - exactly the instants at which a
    schedule can change state. The test is the rule's own active_at, the same
    one the Shield uses, so the CSP and the Shield cannot disagree about when a
    rule is on. No clock means in force (models.py: "absent means active").
    """
    if now is None or rule.valid_time is None:
        return {"now": True, "any": True, "on": False, "off": False}
    end = now + timedelta(seconds=lookahead_s)
    states = [rule.active_at(now)]
    t = now.replace(second=0, microsecond=0) + timedelta(minutes=1)
    while t <= end:
        states.append(rule.active_at(t))
        t += timedelta(minutes=1)
    later = states[1:]
    return {"now": states[0], "any": any(states),
            "on": (not states[0]) and any(later),
            "off": states[0] and bool(later) and not all(later)}


# Padding of the time window for rules that switches and swaps act on. The
# Shield judges each forecast instant on both sides (t - dt/2 and t + dt/2;
# 0.05 s at its defaults), so it can enforce a rule a few hundredths of a
# second outside [now, now + lookahead]. One second on each side keeps the
# CSP's set a SUPERSET of what the Shield enforces in that window.
_SWITCH_PAD_S = 1.0
_JUST_AFTER = timedelta(microseconds=1)


class _RuleClock:
    """Which rules are in force at an instant, with time_window_switch and
    corridor_swap applied EXACTLY as the Shield applies them
    (guardrail/shield.py `_Schedule.on`, the same `recurrence_active`, to the
    microsecond).

    Why a second implementation of `on` and not a call into the Shield: the
    Shield's schedule hangs off its per-generation runtime (fence index,
    anchors, FSM) and is private. This is the ~20 lines of rule logic only,
    and tests/test_csp.py holds it to the Shield's own answer
    (`Shield.rule_status(...)["in_force"]`) at every instant around every edge
    of a policy built to have them. If they ever disagree the test names the
    rule and the instant.

    Used only for rules a switch or swap touches (and for those events
    themselves). Every other rule keeps `_activity`'s minute-grained test,
    unchanged since 2026-10-06, so no stored CSP moves. That test is the
    conservative one for a rule's OWN window (models.Recurrence.active_at keeps
    a window ending 17:30 open until 17:30:59; the Shield closes it at
    17:30:00) - but it is NOT conservative for a switch that turns a rule OFF,
    which it would hold off 59 s longer than the Shield: a rule the Shield
    enforces would be missing from the CSP. Hence the exact clock here.
    """

    def __init__(self, policy: Policy):
        self.sw: dict[str, list] = {}
        self.sp: dict[str, list] = {}
        for c in policy.constraints:
            if isinstance(c, TimeWindowSwitch):
                self.sw.setdefault(c.target_id, []).append(c)
            elif isinstance(c, CorridorSwap):
                self.sp.setdefault(c.target_id, []).append(c)

    def involved(self, rule) -> bool:
        return (isinstance(rule, (TimeWindowSwitch, CorridorSwap))
                or rule.id in self.sw or rule.id in self.sp)

    @staticmethod
    def _base(rule, x):
        """The rule's own window at x: True, False, or None (it has a window
        and there is no clock). shield._Schedule._base, verbatim."""
        vt = rule.valid_time
        if vt is None or vt.recurrence is None:
            return True
        if x is None:
            return None
        return recurrence_active(vt.recurrence, x)

    def on(self, rule, x) -> bool:
        """Is `rule` in force at instant x (None: no clock)?

        A time_window_switch is "in force" while its own window holds (it then
        acts); the Shield never enforces it as geometry. Everything else is
        shield._Schedule.on: a corridor_swap holds while its window holds and
        switches its target off meanwhile; of the switches on a rule that are
        in force, the last in policy order decides; a switch whose window
        cannot be read turns a rule ON but never OFF; no clock counts as in
        force."""
        if isinstance(rule, TimeWindowSwitch):
            return self._base(rule, x) is not False
        if isinstance(rule, CorridorSwap):
            return self._base(rule, x) is not False
        for sp in self.sp.get(rule.id, ()):
            if self._base(sp, x) is True:
                return False
        sws = self.sw.get(rule.id, ())
        if sws:
            if any(s.active and self._base(s, x) is None for s in sws):
                return True
            known = [s.active for s in sws if self._base(s, x) is True]
            if known:
                return known[-1]
        return self._base(rule, x) is not False

    def acted_on_text(self, rule) -> str | None:
        """Every switch and swap that acts on `rule`, with its window, in
        words ("switched off by sw-b-off (Mon 08:00-08:30)"); None when none
        does. For P0Entry.in_force, which otherwise reads only the rule's own
        window: a switch target read "always" there although a switch turns
        it off for part of the week (found in review, 2026-10-08)."""
        parts = [f"switched {'on' if s.active else 'off'} by {s.id} "
                 f"({window_text(s) or 'always'})" for s in self.sw.get(rule.id, ())]
        parts += [f"replaced by {sp.id} ({window_text(sp) or 'always'})"
                  for sp in self.sp.get(rule.id, ())]
        return "; ".join(parts) or None

    def holders(self, rule, x) -> list[str]:
        """The ids of the switches and swaps holding `rule` OFF at x."""
        out = [sp.id for sp in self.sp.get(rule.id, ()) if self._base(sp, x) is True]
        sws = [s for s in self.sw.get(rule.id, ()) if self._base(s, x) is True]
        if sws and not sws[-1].active:
            out.append(sws[-1].id)
        return out

    def activity(self, rule, now: datetime | None, lookahead_s: float) -> dict:
        """`_activity`'s answer, from exact instants.

        Every schedule changes state only at a minute boundary (a window
        opens AT hh:mm:00 and closes just AFTER it; days change at midnight),
        so the state is constant on each open stretch between boundaries. The
        window [now - pad, now + lookahead + pad] is therefore sampled
        exhaustively by: its two ends, `now`, every boundary inside it, and
        one microsecond after each."""
        if now is None:
            v = self.on(rule, None)
            return {"now": v, "any": v, "on": False, "off": False}
        pad = timedelta(seconds=_SWITCH_PAD_S)
        lo, end = now - pad, now + timedelta(seconds=lookahead_s) + pad
        ts = [now, lo, end]
        t = lo.replace(second=0, microsecond=0) + timedelta(minutes=1)
        while t <= end:
            ts.append(t)
            if t + _JUST_AFTER <= end:
                ts.append(t + _JUST_AFTER)
            t += timedelta(minutes=1)
        state = {x: self.on(rule, x) for x in ts}
        later = [state[x] for x in sorted(ts) if x > now]
        return {"now": state[now], "any": any(state.values()),
                "on": (not state[now]) and any(later),
                "off": state[now] and bool(later) and not all(later)}


class _Ctx:
    """The mission context, computed once per compilation."""

    def __init__(self, mission: Mission, lookahead_s: float, now, reach_m: float,
                 region_margin_m: float | None, policy: Policy | None = None,
                 clock: "_RuleClock | None" = None):
        self.policy = policy
        self.clock = clock
        self.m = mission
        self.a: Pt = (mission.start_x, mission.start_y)
        self.b: Pt = (mission.target_x, mission.target_y)
        self.lookahead_s = lookahead_s
        self.now = now
        self.reach_m = reach_m
        self.region_margin_m = region_margin_m
        widen = (region_margin_m if region_margin_m is not None
                 else DEFAULT_REGION_MARGIN_M) + reach_m
        self.prox_scale_m = widen
        if region_margin_m is None:
            self.region = None
        else:
            self.region = {"x_min": min(self.a[0], self.b[0]) - widen,
                           "x_max": max(self.a[0], self.b[0]) + widen,
                           "y_min": min(self.a[1], self.b[1]) - widen,
                           "y_max": max(self.a[1], self.b[1]) + widen}


def _fence_dist(f, ctx: _Ctx) -> tuple[float, float]:
    """(path distance, target distance) to the zone's MARGIN RING - the edge
    the Shield actually enforces. Exact: the distance to a polygon buffered by
    m is the distance to the polygon minus m, floored at 0. A dynamic_nfz is
    measured at its declared position."""
    poly = [(v.x, v.y) for v in f.vertices]
    d_path = max(0.0, _seg_poly(ctx.a, ctx.b, poly) - f.margin_m)
    d_tgt = max(0.0, _seg_poly(ctx.b, ctx.b, poly) - f.margin_m)
    return d_path, d_tgt


def _in_region(rule, ctx: _Ctx) -> bool:
    """Only keep-OUT zones are ever filtered by region, and never a MOVING one.

    A corridor is keep-IN: everywhere outside it is forbidden, so "far from the
    mission" is precisely the case where it matters most (the mission is
    outside it). Envelopes and stand-offs have no location. A moving
    dynamic_nfz has no fixed place the compiler knows: the Shield moves it from
    the event time, which a CSP issued from the policy alone cannot see. The
    test is a bounding-box overlap, which can only keep MORE zones than an
    exact test would, never fewer.
    """
    if ctx.region is None or not isinstance(rule, _ZONES) or is_moving(rule):
        return True
    xs = [v.x for v in rule.vertices]
    ys = [v.y for v in rule.vertices]
    g = rule.margin_m
    r = ctx.region
    return not (max(xs) + g < r["x_min"] or min(xs) - g > r["x_max"]
                or max(ys) + g < r["y_min"] or min(ys) - g > r["y_max"])


def _severity(rule) -> float:
    s = SEVERITY[rule.priority]
    return s if rule.constraint_type == "hard" else s * SOFT_FACTOR


def _proximity(rule, ctx: _Ctx) -> float:
    """In [0, 1]. Keep-out zones: 1 on the path, falling linearly to 0 at the
    region's edge. Everything else binds wherever the vehicle is - a corridor
    because outside it is forbidden, envelopes and stand-offs because they
    have no location, a moving zone because its place is not known here, a
    switch because it acts on a rule wherever that rule binds - so 1."""
    if not isinstance(rule, _ZONES) or is_moving(rule):
        return 1.0
    d, _ = _fence_dist(rule, ctx)
    return max(0.0, min(1.0, 1.0 - d / ctx.prox_scale_m))


def _r4(x: float) -> float:
    return round(x, 4) + 0.0


# --------------------------------------------------------------------------- #
# Relevance explanations (KPI 4)
# --------------------------------------------------------------------------- #

def _time_fact(rule, act: dict, ctx: _Ctx) -> str | None:
    w = window_text(rule)
    if w is None:
        return None
    L = _fmt_g(ctx.lookahead_s)
    if ctx.now is None:
        return f"schedule {w}; no clock was given, so it is treated as in force"
    if act["on"]:
        return f"not in force at issue time but switches on within the next {L} s ({w})"
    if act["off"]:
        return f"in force at issue time but switches off within the next {L} s ({w})"
    return f"in force at issue time ({w})"


def _relevance(rule, act: dict, ctx: _Ctx) -> str:
    """Why this rule matters to THIS mission, from mission facts (PDF p5)."""
    m, (sx, sy), (tx, ty) = ctx.m, ctx.a, ctx.b
    tf = _time_fact(rule, act, ctx)
    if isinstance(rule, _ZONES):
        d_path, d_tgt = _fence_dist(rule, ctx)
        facts = {"crosses": d_path <= 0.0, "sx": sx, "sy": sy, "tx": tx, "ty": ty,
                 "d_path": d_path, "d_target": d_tgt, "alt": m.cruise_alt_m,
                 "floor": rule.altitude_floor_m, "ceil": rule.altitude_ceiling_m,
                 "in_band": rule.altitude_floor_m <= m.cruise_alt_m <= rule.altitude_ceiling_m}
        if isinstance(rule, CircleFence):
            facts["r"] = rule
        elif isinstance(rule, DynamicNFZ):
            mv = rule.motion
            speed = math.hypot(mv.vx_mps, mv.vy_mps) if mv else 0.0
            facts.update(moving=is_moving(rule), speed=speed,
                         turn=abs(mv.yaw_rate_dps) if mv else 0.0,
                         travel=speed * ctx.lookahead_s, lookahead=ctx.lookahead_s)
    elif isinstance(rule, _TUBES):
        hw, pts = rule.half_width_m, rule.points()
        facts = {"start_in": nearest_on_polyline(sx, sy, pts)[0] <= hw,
                 "target_in": nearest_on_polyline(tx, ty, pts)[0] <= hw,
                 "hw": hw, "alt": m.cruise_alt_m, "floor": rule.altitude_floor_m,
                 "ceil": rule.altitude_ceiling_m,
                 "alt_in_band": rule.altitude_floor_m <= m.cruise_alt_m <= rule.altitude_ceiling_m}
        if isinstance(rule, CorridorSwap):
            facts["target"] = rule.target_id
    elif isinstance(rule, TimeWindowSwitch):
        tgt = next((c for c in (ctx.policy.constraints if ctx.policy else ())
                    if c.id == rule.target_id), None)
        state = None
        if tgt is not None and ctx.clock is not None:
            state = ctx.clock.activity(tgt, ctx.now, ctx.lookahead_s)["any"]
        facts = {"target": rule.target_id, "active": rule.active,
                 "target_kind": (tgt.type if tgt is not None else "rule"),
                 "target_state": state}
    elif isinstance(rule, AltitudeEnvelope):
        a = m.cruise_alt_m
        where = ("inside" if rule.alt_min_m <= a <= rule.alt_max_m
                 else "below" if a < rule.alt_min_m else "above")
        facts = {"alt": a, "lo": rule.alt_min_m, "hi": rule.alt_max_m, "where": where}
    elif isinstance(rule, KinematicEnvelope):
        facts = {"v": m.speed_pref_mps, "cap": rule.speed_max_mps,
                 "ok": m.speed_pref_mps <= rule.speed_max_mps}
    elif isinstance(rule, ObstacleClearance):
        facts = {"length": math.hypot(tx - sx, ty - sy)}
    elif isinstance(rule, SubjectStandoff):
        # A class-specific stand-off binds to the ONE subject being followed,
        # and only while perception classifies it as that class
        # (models.SubjectStandoff.binds; the Shield holds one subject). Until
        # 2026-10-19 this said "binds to any pedestrian": in the red-car demo
        # the CSP justified a 10 m bubble around every person that the Shield
        # never enforced (standoff-pedestrian: 0 bound ticks on a car follow).
        star = rule.subject_class == "*"
        facts = {"follow": bool(_FOLLOW_WORDS.search(m.task_text)),
                 "who": "any followed subject" if star else "the subject being followed",
                 "cls": None if star else rule.subject_class}
    else:                                     # pragma: no cover - new rule type
        raise TypeError(f"no relevance template for {type(rule).__name__}")
    facts["time_fact"] = tf
    return render_template(f"reason/{template_type('reason', rule)}.j2", facts)


# --------------------------------------------------------------------------- #
# Action sets
# --------------------------------------------------------------------------- #

class CSPAltitudeConflict(ValueError):
    """The hard altitude bands (envelopes and corridors) in force TOGETHER
    have no height in common: no CSP's altitude band can hold.

    A ValueError, so a caller's `except ValueError` (demo/real_vla_demo.py's
    no-prompt path) still catches it as before; its own class so
    coverage_report can list the file and carry on instead of dying, as it
    does for CSPBudgetExceeded. `rule_ids` names the clashing rules, `when`
    the instant they clash (None: no clock, every rule read as in force).
    """

    def __init__(self, bands: list, when: datetime | None = None):
        self.rule_ids = [b[0] for b in bands]
        self.when = when
        at = "" if when is None else f" at {when.isoformat(timespec='seconds')}"
        super().__init__("altitude bands in scope (envelopes and corridors) "
                         f"do not overlap{at}: "
                         + ", ".join(f"{i} {b0:g}-{b1:g} m" for i, b0, b1 in bands))


def _in_force_sets(rules: list, clock: "_RuleClock", now: datetime,
                   lookahead_s: float) -> list[tuple[datetime, frozenset]]:
    """The ids of `rules` in force at each instant the time filter samples,
    `now` first.

    The instants and the test are the ones that put each rule in scope:
    `_activity`'s (`now` and every minute boundary up to now + lookahead,
    `active_at`) for an ordinary rule, `_RuleClock.activity`'s (the window
    padded by _SWITCH_PAD_S, each boundary and one microsecond after it,
    `_RuleClock.on`) for a rule a switch or swap touches. An instant outside a
    rule's own sampled window counts it as not in force there, which can only
    make a set smaller, never add a rule the scope test did not see."""
    end = now + timedelta(seconds=lookahead_s)
    pad = timedelta(seconds=_SWITCH_PAD_S)
    ts = {now}
    t = now.replace(second=0, microsecond=0) + timedelta(minutes=1)
    while t <= end:
        ts.add(t)
        t += timedelta(minutes=1)
    if any(clock.involved(r) for r in rules):
        lo, hi = now - pad, end + pad
        ts.update((lo, hi))
        t = lo.replace(second=0, microsecond=0) + timedelta(minutes=1)
        while t <= hi:
            ts.add(t)
            if t + _JUST_AFTER <= hi:
                ts.add(t + _JUST_AFTER)
            t += timedelta(minutes=1)

    def on(r, x) -> bool:
        if clock.involved(r):
            return now - pad <= x <= end + pad and clock.on(r, x)
        return now <= x <= end and r.active_at(x)
    return [(x, frozenset(r.id for r in rules if on(r, x)))
            for x in [now] + sorted(ts - {now})]


def _action_sets(rules: list, in_force: list | None = None
                 ) -> tuple[AllowedActionSet, ForbiddenActionSet]:
    """The action-set adapter's content, from every in-scope HARD rule.

    In-scope, not just kept: the token budget limits the text a language model
    reads, and these structured sets are a few numbers. A P1 speed cap cut from
    the prose for space still bounds the action set.

    The altitude band is the INTERSECTION of every hard altitude envelope and
    every hard corridor's floor-ceiling band. A corridor is a tube, and the
    Shield enforces its band wherever the vehicle is (shield.py
    _check_corridor, and CorridorAltitudeFix in _repair_corridor), checking
    each corridor on its own - so all of them bind at once. The spec's worked
    example does the same: a 5-120 m P1 envelope and a 30-80 m corridor give
    altitude_band_m_agl [30, 80] (prefix-compiler.md, CSP example). Until
    2026-10-06 only envelopes were intersected, and corridor_survey (a 10-20 m
    corridor, no envelope) reported no band at all - "unbounded" by this
    class's own definition while the Shield held 10-20 m.

    PER INSTANT, NOT ACROSS SCHEDULES. "All of them bind at once" holds only
    for the rules in force at the same instant. With a clock, `in_force`
    (`_in_force_sets`, `now` first) says which in-scope rules are in force
    at each sampled instant of the window. The band published is the
    intersection over every in-scope band, as before, whenever that is not
    empty (a height legal for the whole window); when it is empty only
    because the window crosses a schedule edge (a day band 10-30 m until
    17:59, a night band 40-60 m from 18:00, `now` 17:59:58), it is the band in
    force at `now` - or, when no band is in force at `now` (a gap between the
    schedules inside a lookahead over a minute), the band in force at the
    next sampled instant that has one - and the other rules stay in
    `source_rules` and in their P0 entries' `in_force` text. Never a hull:
    that would admit heights no rule allows, and never None while a band is
    in scope. CSPAltitudeConflict is raised only when the rules in force at ONE
    instant have no height in common - with no clock (`in_force` None) every
    rule is read as in force, so that is the intersection of them all, as
    compile_csp documents. Until 2026-10-19 every in-scope band was
    intersected whatever its schedule, so a policy the lint accepts (disjoint
    bands that are never in force together) raised a bare ValueError: on
    every clockless compile, which took the whole coverage_report down, and
    with a clock in the seconds before the switch.
    """
    hard = [r for r in rules if r.constraint_type == "hard"]
    kins = [r for r in hard if isinstance(r, KinematicEnvelope)]
    alts = [r for r in hard if isinstance(r, AltitudeEnvelope)]
    # A corridor_swap is a keep-in tube like a corridor; the corridor it
    # replaces is out of scope while the swap holds (_RuleClock), so it is not
    # in `rules` and does not narrow the band.
    cors = [r for r in hard if isinstance(r, _TUBES)]
    src: list[str] = []
    kw: dict = {}
    if kins:
        v = min(k.speed_max_mps for k in kins)
        c = min(k.climb_rate_max_mps for k in kins)
        kw.update(vx_range_mps=(-v, v), vy_range_mps=(-v, v),
                  vz_range_mps=(-c, c), horizontal_speed_max_mps=v,
                  yaw_rate_max_dps=min(k.yaw_rate_max_dps for k in kins))
        src += [k.id for k in kins]
    bands = ([(a.id, a.alt_min_m, a.alt_max_m) for a in alts]
             + [(c.id, c.altitude_floor_m, c.altitude_ceiling_m) for c in cors])
    if bands:
        def meet(group):
            return max(b[1] for b in group), min(b[2] for b in group)
        lo, hi = meet(bands)
        if lo > hi:
            if in_force is None:
                raise CSPAltitudeConflict(bands)
            now_x = in_force[0][0]
            held = []                 # (instant, band) wherever some band holds
            for x, ids in in_force:
                group = [b for b in bands if b[0] in ids]
                if not group:
                    continue
                g_lo, g_hi = meet(group)
                if g_lo > g_hi:
                    raise CSPAltitudeConflict(group, x)
                held.append((x, (g_lo, g_hi)))
            if not held:
                # Every in-scope rule is in force at one of these instants
                # (_in_force_sets samples exactly what the scope test did), so
                # this cannot happen; if it ever does, read the bands as the
                # no-clock case does rather than publish "unbounded".
                raise CSPAltitudeConflict(bands)
            # The band at `now`; when no band holds at `now` (a gap between two
            # schedules, which a lookahead over a minute can straddle: nothing
            # 07:58, 10-30 m 07:59-08:00, 40-60 m from 08:01), the band of the
            # NEXT instant one holds; failing that (a rule in scope only through
            # a switch's 1 s pad before `now`) the nearest earlier one. Until
            # the 2026-10-19 review only `now` was looked at, so that gap
            # published no band at all - "unbounded" by this class's own
            # definition - while both bands bound the window.
            kw["altitude_band_m_agl"] = min(
                held, key=lambda h: (h[0] < now_x, abs(h[0] - now_x)))[1]
        else:
            kw["altitude_band_m_agl"] = (lo, hi)
        src += [b[0] for b in bands]
    allowed = AllowedActionSet(source_rules=src, **kw)
    forbidden = ForbiddenActionSet(
        no_translate_into_polygons=[r.id for r in hard if isinstance(r, _ZONES)],
        no_translate_out_of_corridors=[r.id for r in hard if isinstance(r, _TUBES)])
    return allowed, forbidden


def action_set_adapter(csp: CSP) -> dict:
    """The spec's flat action-set dict (PDF p4, "Action-set adapter (default)")
    for a VLA backend whose action head takes allowed / forbidden sets."""
    a, f = csp.allowed_action_set, csp.forbidden_action_set

    def lst(t):
        return None if t is None else [t[0], t[1]]
    return {"frame": a.frame,
            "vx_range_mps": lst(a.vx_range_mps),
            "vy_range_mps": lst(a.vy_range_mps),
            "vz_range_mps": lst(a.vz_range_mps),
            "yaw_rate_max_dps": a.yaw_rate_max_dps,
            "altitude_band_m_agl": lst(a.altitude_band_m_agl),
            "horizontal_speed_max_mps": a.horizontal_speed_max_mps,
            "no_translate_into_polygons": list(f.no_translate_into_polygons),
            "no_translate_out_of_corridors": list(f.no_translate_out_of_corridors)}


def action_violations(action, csp: CSP, state=None, policy: Policy | None = None,
                      dt: float = 1.0, ir=None) -> list[str]:
    """Which parts of the CSP's action sets a candidate Action4D breaks.

    This maps the CSP onto the VLA's action vocabulary, models.Action4D, and the
    one conversion that matters is the yaw rate: the CSP states deg/s (as the
    policy does), Action4D.yaw_rate is rad/s. Comparing the two raw admits
    anything under 45 rad/s - the unit confusion recorded in
    docs/FINDING-the-contract-disagreed-with-itself-about-yaw.md.

    The speed checks need no rotation (see AllowedActionSet: the caps are
    rotation-invariant). With `state`, the altitude after `dt` is checked; with
    `state` and the `policy` the CSP was compiled from, the position after `dt`
    is checked against every forbidden zone using geometry.point_in_fence - the
    Shield's own test - and against every keep-in corridor. Advisory: this
    counts would-be violations (the offline-replay metric of PDF p5); the
    Shield, not this function, decides what flies. A MOVING dynamic_nfz is
    tested where it was declared: the CSP cannot know how far the Shield has
    moved it, so the count for such a zone is advisory twice over.

    `ir`: a guardrail.ir.PolicyIR compiled from `policy`. With it each fence's
    margin ring is the one built once at compile time instead of a rebuild per
    call (30-45 us each), which matters to a replay that calls this for every
    frame and candidate (tools/prefix_eval.py). Same answer either way; an IR
    compiled from another policy is ignored rather than trusted.
    """
    a, f = csp.allowed_action_set, csp.forbidden_action_set
    out: list[str] = []
    for name, val, rng in (("vx", action.vx, a.vx_range_mps),
                           ("vy", action.vy, a.vy_range_mps),
                           ("vz", action.vz_up, a.vz_range_mps)):
        if rng is not None and not (rng[0] - 1e-9 <= val <= rng[1] + 1e-9):
            out.append(name)
    if (a.horizontal_speed_max_mps is not None
            and math.hypot(action.vx, action.vy) > a.horizontal_speed_max_mps + 1e-9):
        out.append("speed")
    if (a.yaw_rate_max_dps is not None
            and abs(action.yaw_rate) > math.radians(a.yaw_rate_max_dps) + 1e-12):
        out.append("yaw_rate")
    if state is None:
        return out
    x, y = state.x + action.vx * dt, state.y + action.vy * dt
    up = state.up + action.vz_up * dt
    band = a.altitude_band_m_agl
    if band is not None and not (band[0] - 1e-9 <= up <= band[1] + 1e-9):
        out.append("altitude")
    if policy is None:
        return out
    if not policy.matches_hash(csp.policy_hash):
        raise ValueError(f"CSP is for {csp.policy_hash}, policy is {policy.policy_hash}")
    by_id = {c.id: c for c in policy.constraints}
    rings = ({r.rule.id: r.buffered for r in ir.fences}
             if ir is not None and ir.is_current_for(policy) else {})
    for fid in f.no_translate_into_polygons:
        if point_in_fence(x, y, up, by_id[fid], buffered=rings.get(fid)):
            out.append(f"polygon:{fid}")
    for cid in f.no_translate_out_of_corridors:
        c = by_id[cid]
        d, _, _ = nearest_on_polyline(x, y, c.points())
        if d > c.half_width_m or not (c.altitude_floor_m <= up <= c.altitude_ceiling_m):
            out.append(f"corridor:{cid}")
    return out


# --------------------------------------------------------------------------- #
# Cost-map adapter (Prefix Compiler PDF p4, "Cost-map adapter (path planners
# only)": "Emitted as a separate artefact referenced by cost_map_ref"; PDF p6
# lists it among the outputs). docs/DESIGN-cost-map-adapter.md has the why.
# --------------------------------------------------------------------------- #

COST_MAP_FORMAT = "guardrail-cost-map-v1"
# The grant's example: resolution_m 5, nfz_hard inf, corridor_pref 0,
# corridor_outside 50. COST_SOFT is this project's: a soft keep-out (zone,
# stand-off, clearance, altitude) is discouraged at the same price as leaving
# a corridor, and never made impassable.
COST_MAP_RESOLUTION_M = 5.0
COST_FREE = 0.0
COST_HARD = math.inf
COST_SOFT = 50.0
COST_CORRIDOR_PREF = 0.0
COST_CORRIDOR_OUTSIDE = 50.0
# A grid larger than this is refused rather than allocated (2000 x 2000 cells
# is 10 km square at 5 m).
COST_MAP_MAX_CELLS = 4_000_000
# A stand-off disc is drawn as a polygon that CONTAINS the circle (vertices on
# a circle of r / cos(pi / n)), never the inscribed one shapely's buffer gives.
_DISC_SEGMENTS = 64


def _cost_text(v: float):
    """A cost as the header writes it: JSON has no infinity, so "inf"."""
    return "inf" if math.isinf(v) else float(v)


def _sha_array(a: np.ndarray) -> str:
    a = np.ascontiguousarray(a)
    h = hashlib.sha256(f"{a.dtype.str}{list(a.shape)}".encode())
    h.update(a.tobytes())
    return "sha256:" + h.hexdigest()


def _obstacle_digest(obstacle_map: dict) -> dict:
    occ = (np.asarray(obstacle_map["occ"]) > 0).astype(np.uint8)
    return {"res": float(obstacle_map["res"]), "ox": float(obstacle_map["ox"]),
            "oy": float(obstacle_map["oy"]), "shape": list(occ.shape),
            "sha256": _sha_array(occ)}


@dataclass(frozen=True)
class CostMap:
    """A 2-D cost grid rasterised from a policy's active constraints.

    `cost[i, j]` is the cost of the cell CENTRED on (x, y) = (ox + i * res,
    oy + j * res) in the policy's local frame (x metres North, y metres East
    of the frame origin), the indexing of the Shield's obstacle map
    ({"occ", "res", "ox", "oy"}). `layers` holds one boolean mask per layer, in
    the header's `cost_map.layers` order; the cost is the maximum over the
    layers a cell is in (0 in none). `header` is everything else, JSON-ready,
    with the grant's keys (`cost_map: {resolution_m, origin: {lat, lon},
    layers: [{name, source, cost}]}`) at its top; `ref` is what a CSP's
    cost_map_ref holds."""
    cost: np.ndarray
    layers: tuple
    header: dict = field(compare=False)

    @property
    def ref(self) -> str:
        return self.header["ref"]

    @property
    def policy_hash(self) -> str:
        return self.header["policy_hash"]

    @property
    def generation(self) -> int:
        return int(self.header["generation"])

    @property
    def resolution_m(self) -> float:
        return float(self.header["cost_map"]["resolution_m"])

    @property
    def origin_xy(self) -> tuple[float, float]:
        ox, oy = self.header["origin_xy_m"]
        return float(ox), float(oy)

    def cell_of(self, x: float, y: float) -> tuple[int, int] | None:
        """The (i, j) of the cell holding (x, y), or None off the grid."""
        res = self.resolution_m
        ox, oy = self.origin_xy
        i = int(math.floor((x - ox) / res + 0.5))
        j = int(math.floor((y - oy) / res + 0.5))
        nx, ny = self.cost.shape
        return (i, j) if (0 <= i < nx and 0 <= j < ny) else None

    def cost_at(self, x: float, y: float) -> float | None:
        ij = self.cell_of(x, y)
        return None if ij is None else float(self.cost[ij])

    def save(self, stem: str | Path) -> tuple[Path, Path]:
        """Write `<stem>.npz` (the cost grid and the layer masks) and the
        header twice, LF: `<stem>.json` and `<stem>.yaml` (the same document;
        the grant's output table names the cost-map adapter "Python function
        (emits YAML)", Prefix Compiler PDF p5, and until 2026-10-29 only the
        JSON was written). The header names the npz by file name and carries
        the SHA-256 of every array, so `load_cost_map` (either file) refuses a
        grid that is not the one this header describes. Returns (npz, json)."""
        stem = Path(stem)
        stem.parent.mkdir(parents=True, exist_ok=True)
        npz, js = stem.with_name(stem.name + ".npz"), stem.with_name(stem.name + ".json")
        np.savez_compressed(npz, cost=self.cost,
                            **{f"layer_{k:03d}": m for k, m in enumerate(self.layers)})
        doc = dict(self.header)
        doc["npz"] = npz.name
        with open(js, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(doc, indent=2, sort_keys=True) + "\n")
        with open(stem.with_name(stem.name + ".yaml"), "w", encoding="utf-8", newline="\n") as fh:
            fh.write(yaml.safe_dump(doc, sort_keys=True, default_flow_style=None))
        return npz, js


def _cost_map_ref(core: dict) -> str:
    digest = hashlib.sha256(json.dumps(core, sort_keys=True, separators=(",", ":"),
                                       ensure_ascii=True).encode("ascii")).hexdigest()
    return (f"costmap:{core['policy_id']}@v{core['version']}/g{core['generation']}"
            f"#sha256:{digest}")


def load_cost_map(json_path: str | Path, policy: Policy | None = None, *,
                  ref: str | None = None) -> CostMap:
    """Read a cost map written by `CostMap.save`, refusing one whose arrays,
    header or ref do not agree - and, with `policy`, one rasterised from
    another policy or generation (as `resolve_geometry_ref` refuses a stale
    ref). With `ref` (a CSP's cost_map_ref), also one that is not the map
    that CSP names: the same policy and generation is not enough, two maps
    of one generation differ by clock, height, subject or zone age."""
    js = Path(json_path)
    text = js.read_text(encoding="utf-8")
    doc = yaml.safe_load(text) if js.suffix in (".yaml", ".yml") else json.loads(text)
    npz_name = doc.pop("npz", None)
    if not npz_name:
        raise ValueError(f"{js.name}: no npz named in the header")
    with np.load(js.with_name(npz_name)) as z:
        cost = z["cost"]
        layers = tuple(z[f"layer_{k:03d}"] for k in range(len(doc["cost_map"]["layers"])))
    if _sha_array(cost) != doc.get("cost_sha256"):
        raise ValueError(f"{js.name}: the cost grid in {npz_name} is not the one "
                         f"this header describes")
    if [_sha_array(m) for m in layers] != doc.get("layer_sha256"):
        raise ValueError(f"{js.name}: a layer mask in {npz_name} is not the one "
                         f"this header describes")
    held = doc.pop("ref", None)
    if held != _cost_map_ref(doc):
        raise ValueError(f"{js.name}: the header does not hash to its ref; it was "
                         f"edited after the map was made")
    doc["ref"] = held
    if policy is not None and not (policy.matches_hash(doc["policy_hash"])
                                   and int(doc["generation"]) == policy.generation):
        raise ValueError(f"{js.name} was rasterised from {doc['policy_hash']} "
                         f"g{doc['generation']}; this policy is {policy.policy_hash} "
                         f"g{policy.generation}")
    if ref is not None and held != ref:
        raise ValueError(f"{js.name} holds {held}, not the map {ref}")
    return CostMap(cost=cost, layers=layers, header=doc)


def _check_cost_map_for(cm: CostMap, policy: Policy, mission: Mission,
                        now: datetime | None) -> None:
    """Refuse a cost map that is not the map of the CSP being compiled.

    The same policy and generation is not enough (found in review,
    2026-10-08): two maps of one generation differ by clock and by height. A
    CSP issued Mon 08:15 for a 15 m cruise accepted a map rasterised at Mon
    12:30 - fence-a not drawn although the Shield enforces it at 08:15,
    fence-b drawn although a switch holds it off - and one drawn at 200 m.
    So the map must match on all three:

      policy  its policy_hash and generation are this policy's;
      clock   its `clock` is this CSP's `now` (both None, or the same
              instant to the microsecond);
      height  it was drawn at the mission's cruise height. A map drawn at no
              particular height (altitude_m None) is refused too: that
              reading takes every keep-out zone whatever its band, but reads
              a corridor laterally, which is PERMISSIVE for the corridor's
              altitude band (see rasterise_cost_map), and it never prices an
              altitude envelope the cruise height breaks.

    What a CSP cannot check - where the subject is, how old a moving zone is,
    which obstacle map - is recorded in the map's header and is the
    caller's to match."""
    if not (cm.policy_hash == policy.policy_hash and cm.generation == policy.generation):
        raise ValueError(f"the cost map was rasterised from {cm.policy_hash} "
                         f"g{cm.generation}; this policy is {policy.policy_hash} "
                         f"g{policy.generation}")
    want = None if now is None else now.isoformat()
    got = cm.header.get("clock")
    if got != want:
        raise ValueError(f"the cost map was rasterised at clock {got}; this CSP is "
                         f"issued at clock {want}. Which rules are in force (time "
                         f"windows, switches, swaps) depends on the clock: "
                         f"rasterise it with now= the CSP's clock")
    alt = cm.header.get("altitude_m")
    if alt is None or abs(float(alt) - float(mission.cruise_alt_m)) > 1e-9:
        raise ValueError(f"the cost map was drawn at altitude_m={alt}; this CSP's "
                         f"mission cruises at {_fmt_g(mission.cruise_alt_m)} m. "
                         f"Zone and corridor bands and altitude envelopes depend "
                         f"on the height: rasterise it with altitude_m= the "
                         f"cruise height")


def _disc(x: float, y: float, r: float):
    from shapely.geometry import Point
    q = _DISC_SEGMENTS // 4
    return Point(x, y).buffer(r / math.cos(math.pi / _DISC_SEGMENTS), quad_segs=q)


def _clearance_mask(obstacle_map: dict, r: float, xs: np.ndarray, ys: np.ndarray,
                    res: float) -> np.ndarray:
    """Cells whose square comes within `r` of a blocked map cell's SQUARE.

    The Shield measures clearance to the nearest blocked cell's CENTRE through
    a chamfer distance field (shield._build_signed_distance_field); measuring
    to the whole blocked square is never farther than to its centre, by up to
    half a map cell, which absorbs the field's approximation. Cells off the
    map are free, as the Shield reads them (every obstacle lies on the map)."""
    occ = np.asarray(obstacle_map["occ"]) > 0
    mres = float(obstacle_map["res"])
    mox, moy = float(obstacle_map["ox"]), float(obstacle_map["oy"])
    out = np.zeros((len(xs), len(ys)), dtype=bool)
    hb, h = mres / 2.0, res / 2.0
    for bi, bj in zip(*np.nonzero(occ)):
        bx, by = mox + bi * mres, moy + bj * mres
        reach = r + hb + h
        i0 = max(0, int(math.floor((bx - reach - xs[0]) / res)))
        i1 = min(len(xs), int(math.ceil((bx + reach - xs[0]) / res)) + 1)
        j0 = max(0, int(math.floor((by - reach - ys[0]) / res)))
        j1 = min(len(ys), int(math.ceil((by + reach - ys[0]) / res)) + 1)
        if i0 >= i1 or j0 >= j1:
            continue
        dx = np.maximum(np.abs(xs[i0:i1] - bx) - (h + hb), 0.0)[:, None]
        dy = np.maximum(np.abs(ys[j0:j1] - by) - (h + hb), 0.0)[None, :]
        out[i0:i1, j0:j1] |= (dx * dx + dy * dy) < r * r
    return out


def rasterise_cost_map(policy: Policy, *, now: datetime | None = None,
                       zone_age_s: float | dict | None = None,
                       altitude_m: float | None = None,
                       resolution_m: float = COST_MAP_RESOLUTION_M,
                       bounds: tuple[float, float, float, float] | None = None,
                       pad_m: float | None = None,
                       subject: tuple | None = None,
                       obstacle_map: dict | None = None) -> CostMap:
    """The cost-map adapter: the policy's constraints in force at one instant,
    rasterised onto a 2-D grid (Prefix Compiler PDF p4).

    now          the clock for valid_time windows, switches and swaps, read by
                 the SHIELD (Shield(now=...).rule_status()["in_force"], both
                 sides of the instant), so the map and the Shield cannot
                 disagree about which rules are on. None: no clock, as the
                 Shield reads it.
    zone_age_s   seconds since each MOVING dynamic_nfz appeared (a number for
                 all, or {zone id: age}); the zone is drawn at ring_at(age),
                 the geometry the Shield materialises (ir.compile_fence).
                 Default 0: where it was declared. Recorded in the header.
    altitude_m   draw only what binds at this height (the Shield's band test).
                 None is a 2-D reading for a planner that does not know its
                 height, and it is NOT conservative throughout: it draws every
                 keep-out zone whatever its band (conservative), but counts a
                 cell inside a corridor laterally, ignoring the corridor's
                 altitude band (PERMISSIVE: at a height outside the band the
                 Shield forbids the whole corridor, and the map prices its
                 interior 0), and it prices no altitude envelope. So
                 compile_csp refuses a map drawn at None; a CSP's map is drawn
                 at the mission's cruise height.
    bounds       (x_min, y_min, x_max, y_max) in metres; default the extent of
                 everything drawn, padded by `pad_m` (default 2 cells).
    subject      (x, y) or (x, y, class): where the followed subject is, for
                 subject_standoff. Without it the rule is inert, as in the
                 Shield, and the header says so.
    obstacle_map {"occ", "res", "ox", "oy"} for obstacle_clearance, the
                 Shield's own map format. Without it the rule is inert.

    Layers (the grant's names first): nfz_hard (inf) / nfz_soft (50) for each
    keep-out zone with its margin ring; corridor_pref (0) for each keep-in
    tube and corridor_outside (50) for every cell not inside ALL of them - the
    Shield checks each corridor on its own, so the vehicle must be in all at
    once. corridor_outside is 50 for a HARD corridor too, as in the grant's
    example: outside a hard corridor the Shield refuses the position, while
    the map only prices it, so a planner trusting the map may cross out at a
    price (a planner starting outside, on the take-off pad, could not plan
    into an inf field at all). cost_map_agreement counts those points
    separately (`hard_rule_priced_finite`); standoff_hard / standoff_soft; clearance_hard / clearance_soft;
    altitude_hard / altitude_soft over the whole grid when `altitude_m` breaks
    an altitude envelope. A keep-out layer takes every cell whose square
    MEETS the zone; a tube counts a cell inside only when the square lies
    WHOLLY in it - both the conservative side. Rules with no footprint
    (kinematic caps, switches, an envelope the height respects) are listed
    under `not_drawn` with why, never dropped in silence.
    """
    import shapely
    from .geometry import corridor_tube
    from .ir import compile_fence
    from .models import State
    from .shield import Shield

    res = float(resolution_m)
    if not (res > 0 and math.isfinite(res)):
        raise ValueError(f"resolution_m must be a positive number, got {resolution_m!r}")
    sh = Shield(policy, now=(lambda: now) if now is not None else None, escalation=False)
    up = float(altitude_m) if altitude_m is not None else 0.0
    status = {r["id"]: r for r in sh.rule_status(State(x=0.0, y=0.0, up=up))}

    def in_force(c) -> bool:
        return bool(status[c.id]["in_force"])

    def age_of(c) -> float:
        if isinstance(zone_age_s, dict):
            return float(zone_age_s.get(c.id, 0.0))
        return float(zone_age_s or 0.0)

    def in_band(c) -> bool:
        return altitude_m is None or (c.altitude_floor_m <= altitude_m <= c.altitude_ceiling_m)

    drawn: list[tuple[str, str | None, object, float]] = []   # name, source, geometry, cost
    tubes: list[tuple] = []                                     # (rule, geometry or None)
    not_in_force, out_of_band, not_drawn = [], [], []
    ages: dict[str, float] = {}
    whole: list[tuple[str, str, float]] = []                    # layers over every cell
    clear: list[tuple] = []

    ir = compiled_ir(policy)
    zone_recs = {r.rule.id: r for r in ir.fences}
    for c in policy.constraints:
        hard = c.constraint_type == "hard"
        if isinstance(c, TimeWindowSwitch):
            not_drawn.append({"id": c.id, "type": c.type,
                              "why": f"an event: it acts through whether "
                                     f"'{c.target_id}' is in force, which the map "
                                     f"already reflects"})
            continue
        if not in_force(c):
            not_in_force.append(c.id)
            continue
        if isinstance(c, _ZONES):
            if not in_band(c):
                out_of_band.append(c.id)
                continue
            rec = zone_recs[c.id]
            if rec.moving:
                ages[c.id] = age_of(c)
                rec = compile_fence(rec.index, c, ring=c.ring_at(ages[c.id]),
                                    multiscale=False)
            drawn.append(("nfz_hard" if hard else "nfz_soft", c.id, rec.buffered,
                          COST_HARD if hard else COST_SOFT))
        elif isinstance(c, _TUBES):
            if not in_band(c):
                out_of_band.append(c.id)
                tubes.append((c, None))        # nowhere inside it at this height
            else:
                tubes.append((c, corridor_tube(c)))
        elif isinstance(c, SubjectStandoff):
            if subject is None:
                not_drawn.append({"id": c.id, "type": c.type,
                                  "why": "no subject position given; the rule is "
                                         "inert until perception supplies one, as in "
                                         "the Shield"})
                continue
            cls = subject[2] if len(subject) > 2 else None
            if not c.binds(cls):
                not_drawn.append({"id": c.id, "type": c.type,
                                  "why": f"binds {c.subject_class!r}; the subject "
                                         f"is {cls!r}"})
                continue
            drawn.append(("standoff_hard" if hard else "standoff_soft", c.id,
                          _disc(float(subject[0]), float(subject[1]), c.min_range_m),
                          COST_HARD if hard else COST_SOFT))
        elif isinstance(c, ObstacleClearance):
            if obstacle_map is None:
                not_drawn.append({"id": c.id, "type": c.type,
                                  "why": "no obstacle map given; the rule is inert "
                                         "without one, as in the Shield"})
                continue
            clear.append(c)
        elif isinstance(c, AltitudeEnvelope):
            if altitude_m is not None and not (c.alt_min_m - 1e-9 <= altitude_m
                                               <= c.alt_max_m + 1e-9):
                whole.append(("altitude_hard" if hard else "altitude_soft", c.id,
                              COST_HARD if hard else COST_SOFT))
            else:
                not_drawn.append({"id": c.id, "type": c.type,
                                  "why": (f"no 2-D footprint: bounds height to "
                                          f"{c.alt_min_m:g}-{c.alt_max_m:g} m"
                                          + ("" if altitude_m is None else
                                             f", which {altitude_m:g} m respects"))})
        elif isinstance(c, KinematicEnvelope):
            not_drawn.append({"id": c.id, "type": c.type,
                              "why": "bounds speed, climb and turn rate, not position"})
        else:                                   # pragma: no cover - a new type
            raise TypeError(f"the cost map has no layer for {c.type} ({c.id})")

    # ---- the grid ----
    if bounds is None:
        boxes = [g.bounds for _, _, g, _ in drawn if not g.is_empty]
        boxes += [g.bounds for _, g in tubes if g is not None and not g.is_empty]
        if obstacle_map is not None and clear:
            occ = np.asarray(obstacle_map["occ"])
            mres, mox, moy = (float(obstacle_map["res"]), float(obstacle_map["ox"]),
                              float(obstacle_map["oy"]))
            boxes.append((mox - mres / 2, moy - mres / 2,
                          mox + (occ.shape[0] - 0.5) * mres,
                          moy + (occ.shape[1] - 0.5) * mres))
        if not boxes:
            raise ValueError(f"{policy.policy_id}: nothing with a footprint is in "
                             f"force to rasterise; pass bounds= to grid the area "
                             f"anyway")
        pad = 2 * res if pad_m is None else float(pad_m)
        bounds = (min(b[0] for b in boxes) - pad, min(b[1] for b in boxes) - pad,
                  max(b[2] for b in boxes) + pad, max(b[3] for b in boxes) + pad)
    x0, y0, x1, y1 = (float(v) for v in bounds)
    if not all(math.isfinite(v) for v in (x0, y0, x1, y1)) or x1 < x0 or y1 < y0:
        raise ValueError(f"bounds must be finite (x_min, y_min, x_max, y_max), got {bounds!r}")
    ox, oy = math.floor(x0 / res) * res, math.floor(y0 / res) * res
    nx = int(math.ceil((x1 - ox) / res - 1e-9)) + 1
    ny = int(math.ceil((y1 - oy) / res - 1e-9)) + 1
    if nx * ny > COST_MAP_MAX_CELLS:
        raise ValueError(f"a {nx} x {ny} grid at {res:g} m is over the "
                         f"{COST_MAP_MAX_CELLS} cell limit; coarsen the resolution "
                         f"or pass smaller bounds")
    xs = ox + res * np.arange(nx)
    ys = oy + res * np.arange(ny)
    h = res / 2.0
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    cells = shapely.box(X - h, Y - h, X + h, Y + h)

    layers: list[dict] = []
    masks: list[np.ndarray] = []
    cost = np.zeros((nx, ny), dtype=np.float32)

    def add(name: str, source: str | None, mask: np.ndarray, value: float) -> None:
        nonlocal cost
        mask = np.asarray(mask, dtype=bool)
        layers.append({"name": name, "source": source, "cost": _cost_text(value),
                       "cells": int(mask.sum())})
        masks.append(mask)
        if value > 0:
            cost = np.maximum(cost, np.where(mask, np.float32(value), np.float32(0)))

    for name, source, geom, value in drawn:
        shapely.prepare(geom)
        add(name, source, shapely.intersects(geom, cells), value)
    for c in clear:
        hard = c.constraint_type == "hard"
        add("clearance_hard" if hard else "clearance_soft", c.id,
            _clearance_mask(obstacle_map, c.min_clearance_m, xs, ys, res),
            COST_HARD if hard else COST_SOFT)
    if tubes:
        inside_all = np.ones((nx, ny), dtype=bool)
        for c, geom in tubes:
            if geom is None:
                inside = np.zeros((nx, ny), dtype=bool)
            else:
                shapely.prepare(geom)
                inside = shapely.covers(geom, cells)
            inside_all &= inside
            add("corridor_pref", c.id, inside, COST_CORRIDOR_PREF)
        add("corridor_outside", None, ~inside_all, COST_CORRIDOR_OUTSIDE)
    for name, source, value in whole:
        add(name, source, np.ones((nx, ny), dtype=bool), value)

    proj = policy.projection()
    origin_ll = None
    if proj is not None:
        la, lo = proj.to_latlon(ox, oy)
        origin_ll = {"lat": la, "lon": lo}
    fo = policy.frame_origin
    subj = None
    if subject is not None:
        subj = {"x": float(subject[0]), "y": float(subject[1]),
                "class": subject[2] if len(subject) > 2 else None}
    core = {
        "cost_map": {"resolution_m": res, "origin": origin_ll, "layers": layers},
        "format": COST_MAP_FORMAT,
        "frame": "local metres: x North, y East of the policy frame origin",
        "frame_origin": None if fo is None else {"lat": fo[0], "lon": fo[1]},
        "origin_xy_m": [ox, oy],
        "shape": [nx, ny],
        "index": ("cost[i, j] is the cell centred on (x, y) = (ox + i * res, "
                  "oy + j * res), covering +-res/2 on each axis; the Shield's "
                  "obstacle-map indexing"),
        "costs": {"free": COST_FREE, "hard": _cost_text(COST_HARD), "soft": COST_SOFT,
                  "corridor_pref": COST_CORRIDOR_PREF,
                  "corridor_outside": COST_CORRIDOR_OUTSIDE},
        "combine": "max over the layers a cell is in; 0 in none",
        "rasterisation": {
            "keep_out": "a cell is in the layer when its square meets the zone "
                        "(margin ring included)",
            "keep_in": "a cell is inside a tube only when its square lies wholly "
                       "within it"},
        "policy_id": policy.policy_id, "version": policy.version,
        "generation": policy.generation, "policy_hash": policy.policy_hash,
        "clock": None if now is None else now.isoformat(),
        "in_force_by": "Shield.rule_status (both sides of the instant)",
        "zone_age_s": ages, "altitude_m": altitude_m, "subject": subj,
        "obstacle_map": None if obstacle_map is None else _obstacle_digest(obstacle_map),
        "not_in_force": not_in_force, "out_of_band": out_of_band,
        "not_drawn": not_drawn,
        "cost_sha256": _sha_array(cost),
        "layer_sha256": [_sha_array(m) for m in masks],
    }
    core = json.loads(json.dumps(core))         # exactly what the JSON will hold
    header = dict(core, ref=_cost_map_ref(core))
    return CostMap(cost=cost, layers=tuple(masks), header=header)


def cost_map_agreement(cm: CostMap, policy: Policy, *, n_points: int = 2000,
                       seed: int = 0, now: datetime | None = None,
                       obstacle_map: dict | None = None) -> dict:
    """How the map agrees with the Shield, with both nulls.

    Points are drawn uniformly over the grid (seeded). At each, the Shield's
    own `rule_status` (a Shield built over the same policy, clock, subject and
    obstacle map, with every moving zone at the map's age) says which rules
    standing there breaks; the map's cell says what a planner pays.

      missed     the Shield forbids the point (a hard rule: needs inf; only
                 soft rules or a corridor: needs at least 50) and the cell is
                 cheaper. A sound map has 0: a planner trusting it would plan
                 through a place the Shield then repairs away from. "A
                 corridor needs 50" is the grant's corridor_outside price,
                 for a hard corridor too; so `missed` alone does not say
                 that the map prices a place the Shield REFUSES:
      hard_rule_priced_finite
                 a HARD rule forbids the point (a hard corridor included)
                 and the cell's cost is finite. Every such point the map
                 does not miss is outside a hard corridor, priced 50; a
                 planner trusting the map may go there at that price and the
                 Shield will refuse it. Reported beside `missed`, never
                 folded into it.
      over_cost  the Shield allows the point and the cell costs more than 0:
                 the price of the conservative cell rule (a cell straddling a
                 zone's edge is all zone).

    Nulls, scored the same way: an all-free grid (zero skill: misses every
    forbidden point) and an all-inf grid (refuses everything: misses none,
    over-costs every allowed point). A map is worth something only where it
    beats both: missed 0 like the all-inf grid, over_cost far below it.
    """
    from .models import Action4D, State
    from .shield import Shield
    hdr = cm.header
    if cm.policy_hash != policy.policy_hash or cm.generation != policy.generation:
        raise ValueError("the cost map is not this policy's")
    if (hdr.get("clock") is not None) != (now is not None) or (
            now is not None and now.isoformat() != hdr["clock"]):
        raise ValueError(f"score with the map's own clock ({hdr.get('clock')})")
    if (hdr.get("obstacle_map") is None) != (obstacle_map is None) or (
            obstacle_map is not None and _obstacle_digest(obstacle_map) != hdr["obstacle_map"]):
        raise ValueError("score with the obstacle map the cost map was made from")
    ages = set((hdr.get("zone_age_s") or {}).values())
    if len(ages) > 1:
        raise ValueError("the Shield moves every authored zone on one clock; a map "
                         "with different ages per zone cannot be scored against it")
    sh = Shield(policy, now=(lambda: now) if now is not None else None,
                escalation=False, obstacle_map=obstacle_map)
    alt = hdr.get("altitude_m")
    up = float(alt) if alt is not None else None
    age = ages.pop() if ages else 0.0
    if age > 0:
        far = State(x=-1e6, y=-1e6, up=up if up is not None else 10.0)
        sh.filter(far, Action4D(), t=0.0)
        sh.filter(far, Action4D(), t=float(age))
    subj = hdr.get("subject")
    if subj is not None:
        sh.set_subject(subj["x"], subj["y"], subj["class"])
    rng = np.random.default_rng(seed)
    nx, ny = cm.cost.shape
    res = cm.resolution_m
    ox, oy = cm.origin_xy
    xs = rng.uniform(ox - res / 2, ox + (nx - 0.5) * res, n_points)
    ys = rng.uniform(oy - res / 2, oy + (ny - 0.5) * res, n_points)
    need = []                                    # per point: the cost it needs
    hard = []                                    # per point: a hard rule forbids it
    got = []
    for x, y in zip(xs, ys):
        ij = cm.cell_of(float(x), float(y))
        if ij is None:
            continue
        rows = sh.rule_status(State(x=float(x), y=float(y),
                                    up=up if up is not None else 0.0))
        req = 0.0
        by_hard = False
        for r in rows:
            c = r["rule"]
            if up is not None:
                hit = r["breach"]
            elif not r["in_force"]:
                hit = False
            # A map drawn for no particular height takes every zone and tube
            # whatever its band, so the Shield's test is read laterally.
            elif isinstance(c, _ZONES):
                hit = bool(r["inside"] or r["in_margin"])
            elif isinstance(c, _TUBES):
                hit = r["value"] > c.half_width_m + 1e-9
            elif isinstance(c, AltitudeEnvelope):
                hit = False
            else:
                hit = r["breach"]
            if hit:
                req = max(req, COST_HARD if (c.constraint_type == "hard"
                                             and not isinstance(c, _TUBES))
                          else COST_SOFT)
                by_hard = by_hard or c.constraint_type == "hard"
        need.append(req)
        hard.append(by_hard)
        got.append(float(cm.cost[ij]))
    need_a, got_a, hard_a = np.asarray(need), np.asarray(got), np.asarray(hard, dtype=bool)
    forb = need_a > 0
    allow = ~forb

    def score(cells: np.ndarray) -> dict:
        missed = int(np.sum(forb & (cells < need_a)))
        priced = int(np.sum(hard_a & np.isfinite(cells)))
        over = int(np.sum(allow & (cells > 0)))
        return {"missed": missed, "hard_rule_priced_finite": priced,
                "over_cost": over,
                "missed_rate": (missed / int(forb.sum())) if forb.any() else None,
                "hard_rule_priced_finite_rate": ((priced / int(hard_a.sum()))
                                                 if hard_a.any() else None),
                "over_cost_rate": (over / int(allow.sum())) if allow.any() else None}
    return {"n_points": int(len(need_a)), "n_forbidden": int(forb.sum()),
            "n_forbidden_by_hard": int(hard_a.sum()),
            "n_allowed": int(allow.sum()), "seed": seed,
            "map": score(got_a),
            "null_all_free": score(np.zeros_like(got_a)),
            "null_all_inf": score(np.full_like(got_a, np.inf))}


def p0_coverage(csp: CSP, policy: Policy) -> dict:
    """KPI 2 (PDF p5): the share of in-scope P0 rules the CSP carries.

    A P0 rule counts as covered only if it is in P0_constraints AND its
    sentence is in natural_language_prompt - the structured record and the text
    the model reads must both have it. In scope means the CSP did not filter it
    as not in force or out of region; a rule missing from `selection` entirely
    counts as in scope and uncovered. Zero P0 rules in scope gives coverage
    None: "nothing to cover" is not 100 %, and it is not 0 % either.
    """
    if not policy.matches_hash(csp.policy_hash):
        raise ValueError(f"CSP is for {csp.policy_hash}, policy is {policy.policy_hash}")
    dec = {r.id: r.decision for r in csp.selection.rules}
    filtered = ("filtered_inactive", "filtered_out_of_region")
    scope = [c for c in policy.constraints
             if c.priority == "P0" and dec.get(c.id) not in filtered]
    in_p0 = {p.id for p in csp.P0_constraints}
    show = csp.selection.show_priority
    covered = [c for c in scope if c.id in in_p0
               and render_sentence(c, show) in csp.natural_language_prompt]
    return {"n_p0_in_scope": len(scope), "n_p0_covered": len(covered),
            "coverage": (len(covered) / len(scope)) if scope else None,
            "missing": [c.id for c in scope if c not in covered],
            "filtered": [c.id for c in policy.constraints
                         if c.priority == "P0" and dec.get(c.id) in filtered]}


def rule_coverage(csp: CSP, policy: Policy) -> dict:
    """Coverage of EVERY in-scope rule, per priority (audit card WP2-03).

    `p0_coverage` is the locked KPI. The audit also asked for coverage over all
    rules, "structured and natural-language", because a CSP can carry a rule in
    its record while the model never reads it. Two shares per priority:

      text      - the rule's sentence is in natural_language_prompt (what the
                  model reads);
      recorded  - the rule has a selection row that keeps it in the CSP (kept
                  or dropped_budget) AND a relevance explanation (what an
                  auditor reads).

    P1/P2 text coverage below 1.0 is allowed - the budget may cut them (PDF
    p4) - but it is reported, never hidden, and `recorded` must stay at 1.0
    whatever the budget does. Scope is as in `p0_coverage`; a priority with no
    rule in scope gives None for both shares, not 1.0.
    """
    if not policy.matches_hash(csp.policy_hash):
        raise ValueError(f"CSP is for {csp.policy_hash}, policy is {policy.policy_hash}")
    dec = {r.id: r.decision for r in csp.selection.rules}
    filtered = ("filtered_inactive", "filtered_out_of_region")
    show = csp.selection.show_priority
    out: dict = {}
    for prio in ("P0", "P1", "P2", "all"):
        scope = [c for c in policy.constraints
                 if (prio == "all" or c.priority == prio)
                 and dec.get(c.id) not in filtered]
        n_text = sum(1 for c in scope
                     if render_sentence(c, show) in csp.natural_language_prompt)
        n_rec = sum(1 for c in scope
                    if dec.get(c.id) in ("kept", "dropped_budget")
                    and (csp.relevance_explanations.get(c.id) or "").strip())
        n = len(scope)
        out[prio] = {"n_in_scope": n, "n_in_text": n_text, "n_recorded": n_rec,
                     "text": (n_text / n) if n else None,
                     "recorded": (n_rec / n) if n else None}
    return out


def _wall_clock() -> datetime:
    """The machine's local time, naive. The ONLY place the compiler reads the
    wall clock, and only for `issued_at` when the caller gave neither an
    `issued_at` nor a `now` (see _issue_stamp)."""
    return datetime.now()


def _issue_stamp(issued_at, now: datetime | None) -> tuple[str, str]:
    """(CSP.issued_at, where it came from).

    One convention: the stamp is written the way the clock was given. `now` is
    the policy's local time, naive, as Shield(now=...) takes it (no timezone is
    stored in a policy), so a clocked CSP's stamp is naive local time too, and
    so is the wall-clock fallback. No offset is invented: deriving one from the
    machine's timezone setting would make the same clock compile to different
    CSPs on the WSL SITL rail (UTC) and on Windows (+08:00). A caller that
    wants the spec's UTC form ("2026-04-28T09:01:12Z", PDF p2) passes an aware
    datetime or that string as `issued_at`, and it is written as given.
    """
    if issued_at is not None:
        if isinstance(issued_at, datetime):
            return issued_at.isoformat(timespec="seconds"), "argument"
        s = str(issued_at)
        try:
            # 3.10's fromisoformat does not read a trailing "Z"; 3.11's does.
            datetime.fromisoformat(s[:-1] + "+00:00" if s.endswith("Z") else s)
        except ValueError:
            raise ValueError(f"issued_at must be ISO 8601, got {issued_at!r}") from None
        return s, "argument"
    if now is not None:
        return now.isoformat(timespec="seconds"), "clock"
    return _wall_clock().isoformat(timespec="seconds"), "wall_clock"


def _by_type_order(policy: Policy) -> list:
    """The rules in the 2026-09-01 compiler's order: zones, corridors, altitude,
    kinematics, clearance, stand-off. `build_prompt` keeps it, and the naive
    baseline below cuts it.

    The hot-apply types sit beside their kin (dynamic_nfz after the fences,
    corridor_swap after the corridors) and switches last; no shipped policy
    has one, so no existing text moves. Until 2026-10-08 the list stopped at
    stand-off, which would have dropped those three types from build_prompt
    WITHOUT A WORD the day they became flyable. A runtime type missing here
    now raises instead."""
    out: list = []
    for cls in (PolygonFence, DynamicNFZ, Corridor, CorridorSwap, AltitudeEnvelope,
                KinematicEnvelope, ObstacleClearance, SubjectStandoff,
                TimeWindowSwitch):
        out += policy.by_type(cls)
    left = [c.id for c in policy.constraints
            if c.type in RUNTIME_TYPES and all(c is not o for o in out)]
    if left:
        raise RuntimeError(f"rule types with no place in the by-type order: {left}")
    return out


def naive_prefix_baseline(policy: Policy, budget_tokens: int,
                          token_counter: TokenCounter | None = None) -> dict:
    """KPI 2's no-skill baseline: what a compiler with no priority, no risk and
    no CSPBudgetExceeded would carry.

    It takes the old by-type sentence list (`build_prompt`'s text, schedules
    included) and cuts it as a strict prefix to the same budget, with the same
    token counter. A P0 rule counts as covered if its sentence survives the
    cut. Every rule is in scope (no clock, no region filter), as in
    `coverage_report`.

    Why it is reported beside the KPI: "P0 coverage = 100 %" alone cannot tell
    skill from none when the budget is loose. At 256 tokens every shipped
    policy fits whole, so this baseline also scores 100 % and the KPI does not
    discriminate there; the compiler's priority ordering only shows at a budget
    the full text does not fit (measured: docs/DESIGN-prefix-compiler.md, "KPI
    evidence"). Where the compiler raises CSPBudgetExceeded, this baseline
    silently drops P0 rules instead - which is the failure KPI 3 forbids.
    """
    counter = token_counter or _csp.DEFAULT_TOKEN_COUNTER
    parts: list[str] = []
    for rule in _by_type_order(policy):
        s = render_sentence(rule)
        if counter.count(" ".join(parts + [s])) > budget_tokens:
            break
        parts.append(s)
    text = " ".join(parts)
    p0 = [c for c in policy.constraints if c.priority == "P0"]
    covered = {c.id for c in p0 if render_sentence(c) in text}
    return {"n_p0": len(p0), "n_p0_covered": len(covered),
            "coverage": (len(covered) / len(p0)) if p0 else None,
            "missing": [c.id for c in p0 if c.id not in covered],
            "n_rules_in_text": len(parts), "tokens_used": counter.count(text)}


# --------------------------------------------------------------------------- #
# Lint WARNINGS (never refusals): a time window that can open on an aircraft
# that cannot get out in time
# --------------------------------------------------------------------------- #
#
# WHY. The library's only P0 escape ticks until 2026-10-21 (6 in the three
# time-window-opens-on-vehicle cells, experiments/templates/
# time_window_switch.yaml) were a window opening over an aircraft that cannot
# leave before it opens: from the zone's middle the ring is 10 m away, 2.5 s
# at the 4 m/s cap, and the window opens 1.0-2.0 s in. On the first tick no
# action cleared the forecast, so the Shield flew the fastest way out with the
# violation on it and its own counter scored the escape. Since then the
# Shield judges such an opening by the inside rule from the moment it opens
# (Shield._opening_imposed: escaping is the recovery, a stop is the
# violation), when it can PROVE no action leaves in time - a hard speed cap,
# a zone that does not move; without them the escape is still counted. Either
# way the position is made illegal under the aircraft and it is inside a zone
# in force until it is out, which only the policy (or the mission's timing)
# can prevent, so the DSL says so up front.
# The grant asks whether the lint has severity classes ("Lint severity
# classes. Errors only, or warnings vs errors?", Policy DSL page, open
# questions): `Policy.lint()` stays errors-only (every finding refuses), and
# this is the first warning class - reported, never refused.
#
# THE TEST. A keep-out zone whose being in force can change with time (its own
# `valid_time` recurrence, or a time_window_switch aimed at it) can come into
# force over any point of its margin ring that the aircraft may legally be at
# before it does. From the deepest such point (within the hard keep-in
# corridors, if the policy has any), the nearest legal exit (outside the ring,
# inside a corridor, outside every zone that is always in force) is `depth_m`
# away; at the speed assumption that takes depth / speed seconds.
#   * policy level: the Shield's forecast sees an opening `lookahead_s` ahead
#     (5 s, the grant's), so if leaving takes longer, an aircraft inside when
#     the opening comes into view cannot be out in time - WARNING;
#   * mission level (`start` and `now` given, the compile-time case): the
#     mission starts inside the ring and the window opens sooner than it can
#     leave from there - WARNING. This is the opens-on-vehicle cell.
# An event-driven switch (a hot-applied time_window_switch with no window)
# gives no lead time at all and cannot be linted from the policy file.
#
# SPEED ASSUMPTION. `speed_mps` when given, else the tightest hard, enforced
# speed cap in the policy (the Shield clamps every action to it), else
# LINT_SPEED_MPS (4 m/s, the Shield's own default speed for a rescue heading;
# guardrail/holdpoint.py plans a hold transit at the same). Every warning says
# which it used.
LINT_SPEED_MPS = 4.0
_LINT_MAX_POINTS = 4000


def _lint_speed(policy: Policy, speed_mps: float | None) -> tuple[float, str]:
    if speed_mps is not None:
        v = float(speed_mps)
        if not (math.isfinite(v) and v > 0):
            raise ValueError(f"speed_mps must be a finite number > 0, got {speed_mps!r}")
        return v, f"{v:g} m/s as given"
    caps = [c.speed_max_mps for c in policy.by_type(KinematicEnvelope)
            if c.constraint_type == "hard" and c.violation_action != "monitor_only"]
    if caps:
        v = min(caps)
        return v, f"{v:g} m/s, the policy's hard speed cap"
    return LINT_SPEED_MPS, (f"{LINT_SPEED_MPS:g} m/s assumed: the policy has no hard "
                            f"speed cap")


def _opening_outcome(policy: Policy, z) -> str:
    """What the Shield makes of a window opening on an aircraft that cannot
    leave in time, for the warning's text: since 2026-10-21 it judges the way
    out by the inside rule (guardrail/shield.py Shield._opening_imposed) when
    it can prove the opening imposed - a hard, enforced kinematic cap with no
    window of its own and a zone that does not move; otherwise its counter
    still scores the ticks it flies out as P0 escapes."""
    from .ir import is_moving
    caps = [c for c in policy.by_type(KinematicEnvelope)
            if c.constraint_type == "hard" and c.violation_action != "monitor_only"
            and (c.valid_time is None or c.valid_time.recurrence is None)]
    if caps and not is_moving(z):
        # "at the speed cap": since the review of 3660383 (P1) the escape
        # from such an opening is raised to the cap until the aircraft is out
        # of the ring (GeofenceEscape, Shield._opened_on), not to the 2.0 m/s
        # floor of a zone the aircraft flew into - where the cap re-checks
        # clean: since the review of b1d3aca (P1) a cap that breaks another
        # rule is flown as fast as re-checks clean down to that floor, and
        # below it the tick is reported to the FSM (Shield._floor_the_escape).
        # Since the review of 69c27bd (P2) "re-checks clean" is judged on the
        # escape as flown - out of the ring, then a stop (Shield.
        # _shaped_escape) - not on the cap held for the whole 5 s: the
        # library's own corridor edge, 9 m past the school zone's ring,
        # refused the straight 20 m forecast. The lint does not walk the
        # other rules, so it says which applies, not which will.
        return ("until it is out it is inside the zone in force, and only the way "
                "out passes the Shield, flown at the speed cap where that escape, "
                "judged out of the ring and then braked to a stop, re-checks clean, "
                "else as fast as does down to 2.0 m/s, else reported to the "
                "escalation FSM (a stop there is the violation; an escape at that "
                "floor or faster is judged as the recovery it is, not as a P0 escape)")
    return ("the Shield cannot prove that no action leaves in time (no hard speed "
            "cap without a window of its own, or a moving zone), so it flies the "
            "way out with the P0 violation on and its counter scores those ticks "
            "as P0 escapes")


def _timed_zones(policy: Policy) -> list:
    """Hard, enforced keep-out zones whose being in force can change with
    time: their own weekly window, or a time_window_switch aimed at them."""
    switched = {c.target_id for c in policy.constraints if isinstance(c, TimeWindowSwitch)}
    return [c for c in policy.constraints
            if isinstance(c, _ZONES) and c.constraint_type == "hard"
            and c.violation_action != "monitor_only"
            and ((c.valid_time is not None and c.valid_time.recurrence is not None)
                 or c.id in switched)]


def _ring_of(zone):
    from shapely.geometry import Polygon
    return Polygon([(v.x, v.y) for v in zone.vertices]).buffer(max(zone.margin_m, 0.0))


def _exits_for(policy: Policy, zone, ring):
    """(region the aircraft may be in inside the ring, the exits from it): the
    ring within the hard keep-in corridors (all of the ring when there are
    none), and the legal places outside it - inside a corridor, outside every
    hard zone that is always in force."""
    from shapely.geometry import LineString, box
    from shapely.ops import unary_union
    tubes = [LineString(list(c.points())).buffer(c.half_width_m)
             for c in policy.constraints if isinstance(c, _TUBES)
             and c.constraint_type == "hard" and c.violation_action != "monitor_only"]
    keep_in = unary_union(tubes) if tubes else None
    timed = {z.id for z in _timed_zones(policy)}
    # Always in force: no window of its own - `valid_time` absent, or present
    # with no recurrence (models.ValidTime: "recurrence absent means
    # always") - and no switch aimed at it. Until 2026-10-20 only an absent
    # `valid_time` counted, so a zone written `valid_time: {}` was neither
    # timed nor a wall: exits could land inside it and the depth came out
    # too small (review of T1c, P5).
    always = [_ring_of(z) for z in policy.constraints
              if isinstance(z, _ZONES) and z is not zone and z.constraint_type == "hard"
              and z.violation_action != "monitor_only" and z.id not in timed
              and (z.valid_time is None or z.valid_time.recurrence is None)]
    minx, miny, maxx, maxy = ring.bounds
    pad = max(maxx - minx, maxy - miny) + 1.0
    world = box(minx - pad, miny - pad, maxx + pad, maxy + pad)
    if keep_in is not None:
        world = world.intersection(keep_in)
    region = ring if keep_in is None else ring.intersection(keep_in)
    blocked = unary_union([ring] + always)
    return region, world.difference(blocked)


def _deepest(region, exits) -> tuple[float, float, float] | None:
    """(depth_m, x, y): the point of `region` farthest from every exit, on a
    grid of at most _LINT_MAX_POINTS points (resolution reported by the
    caller's tolerance), or None when the region is empty. inf depth when
    there is no exit at all."""
    import numpy as np
    import shapely
    if region.is_empty:
        return None
    minx, miny, maxx, maxy = region.bounds
    step = max(0.25, math.sqrt(max(region.area, 1e-9) / _LINT_MAX_POINTS))
    xs = np.arange(minx + step / 2, maxx, step)
    ys = np.arange(miny + step / 2, maxy, step)
    if len(xs) == 0 or len(ys) == 0:
        c = region.representative_point()
        xs, ys = np.array([c.x]), np.array([c.y])
    gx, gy = np.meshgrid(xs, ys)
    pts = shapely.points(gx.ravel(), gy.ravel())
    inside = shapely.contains(region, pts)
    pts = pts[inside]
    if len(pts) == 0:
        c = region.representative_point()
        pts = shapely.points(np.array([c.x]), np.array([c.y]))
    if exits.is_empty:
        p = pts[0]
        return math.inf, float(shapely.get_x(p)), float(shapely.get_y(p))
    d = shapely.distance(exits, pts)
    i = int(np.argmax(d))
    bx, by = float(shapely.get_x(pts[i])), float(shapely.get_y(pts[i]))
    # A second, ten times finer pass around the best grid point: the depth
    # is then within about a twentieth of the first step (the first unit
    # test read 25.9 m for a 26 m ring half-width on the coarse grid alone).
    fx = np.linspace(bx - step, bx + step, 21)
    fy = np.linspace(by - step, by + step, 21)
    gx, gy = np.meshgrid(fx, fy)
    fine = shapely.points(gx.ravel(), gy.ravel())
    fine = fine[shapely.contains(region, fine)]
    if len(fine):
        fd = shapely.distance(exits, fine)
        j = int(np.argmax(fd))
        if fd[j] > d[i]:
            return (float(fd[j]), float(shapely.get_x(fine[j])),
                    float(shapely.get_y(fine[j])))
    return float(d[i]), bx, by


# How far ahead the mission-level test looks for the window's next opening
# when leaving would take longer still: a week, the period of every weekly
# schedule, so an opening beyond it never comes.
_LINT_HORIZON_S = 7 * 24 * 3600.0


def _next_opening(clock: "_RuleClock", zone, now: datetime,
                  horizon_s: float) -> float | None:
    """Seconds from `now` until `zone` comes into force (it is not in force
    at `now`), if within `horizon_s`; else None.

    Exact, not sampled. Whether a rule is in force (`_RuleClock.on`: its own
    weekly window and every switch and swap acting on it, the Shield's
    recurrence_active) can change only at a whole minute of the clock:
    windows are written "HH:MM", open AT their start minute, close just
    after their end minute, and a day turns at midnight. So the clock is
    read at each minute boundary in the horizon and just after it (a zone a
    switch-off releases comes on a microsecond after the switch's end
    minute), and an opening is reported at its boundary.

    Until 2026-10-20 the mission-level test stepped every 0.05 s up to the
    time it takes to leave, capped at a week: with no legal exit at all that
    is 12 million clock reads, 12.6 s for one call, which the sweep makes
    once per episode (review of T1c, P4). Now at most 2 x 10080 reads."""
    if horizon_s < 0:
        return None
    first = now.replace(second=0, microsecond=0)
    k = 0
    while True:
        b = first + timedelta(minutes=k)
        lead = (b - now).total_seconds()
        if lead > horizon_s + 1e-9:
            return None
        for x in (b, b + timedelta(microseconds=1)):
            if x > now and clock.on(zone, x):
                return max(0.0, lead)
        k += 1


def time_window_lead_warnings(policy: Policy, *, lookahead_s: float = DEFAULT_LOOKAHEAD_S,
                              speed_mps: float | None = None,
                              start: tuple[float, float, float] | None = None,
                              now: datetime | None = None) -> list[str]:
    """The lint WARNINGS above: time windows that can open on an aircraft that
    cannot leave in time. Never raises on a finding; returns [] when there is
    none. `start` (x, y, up) and `now` add the mission-level test (both
    needed). A dynamic_nfz is judged at its declared position."""
    if not (math.isfinite(float(lookahead_s)) and lookahead_s > 0):
        raise ValueError(f"lookahead_s must be a finite number > 0, got {lookahead_s!r}")
    from shapely.geometry import Point
    speed, why = _lint_speed(policy, speed_mps)
    clock = _RuleClock(policy)
    out: list[str] = []
    for z in _timed_zones(policy):
        ring = _ring_of(z)
        region, exits = _exits_for(policy, z, ring)
        deep = _deepest(region, exits)
        if deep is not None:
            depth, x, y = deep
            fly = depth / speed
            if fly > lookahead_s + 1e-9:
                # (No "leaving takes inf s": with no exit there is no time.)
                # "at any speed (planned at ...)": `why` is a clause of its
                # own ("4 m/s assumed: the policy has no hard speed cap"),
                # and spliced before "or any other speed" it read as one
                # garbled sentence (re-review of T1c, P4).
                exit_txt = (f"there is no legal exit at all: it cannot leave at any "
                            f"speed (planned at {why}), so no forecast (the Shield's "
                            f"sees {lookahead_s:g} s ahead) sees an opening early enough"
                            if math.isinf(depth) else
                            f"the nearest legal exit is {depth:.1f} m away: leaving "
                            f"takes {fly:.1f} s at {why}, longer than the "
                            f"{lookahead_s:g} s the Shield's forecast sees an opening "
                            f"coming")
                out.append(
                    f"{z.id}: its time window can open with the aircraft at "
                    f"({x:.1f}, {y:.1f}), inside its {z.margin_m:g} m margin ring, "
                    f"where {exit_txt}. An aircraft there when the opening comes into "
                    f"view cannot be out in time: {_opening_outcome(policy, z)} (a "
                    f"warning, not a refusal)")
        if start is None or now is None:
            continue
        sx, sy, sup = (float(v) for v in start)
        if not (ring.contains(Point(sx, sy))
                and z.altitude_floor_m <= sup <= z.altitude_ceiling_m):
            continue
        if clock.on(z, now):
            continue        # already in force at the start: start_conflicts' case
        d_out = exits.distance(Point(sx, sy)) if not exits.is_empty else math.inf
        if math.isinf(d_out):
            # Nowhere to go at all (a keep-in corridor wholly inside the ring,
            # walls of always-on zones round it): whenever the window opens,
            # it opens on the aircraft. Said as such - not as a lead time.
            nxt = _next_opening(clock, z, now, _LINT_HORIZON_S)
            when = ("not within a week" if nxt is None
                    else f"next in {nxt:.0f} s, at "
                         f"{(now + timedelta(seconds=nxt)).isoformat(timespec='minutes')}")
            out.append(
                f"{z.id}: the mission starts at ({sx:.1f}, {sy:.1f}, {sup:.1f}), "
                f"inside its margin ring, and there is no legal exit from there at "
                f"all (outside the ring, nowhere inside the hard keep-in corridors "
                f"is clear of the zones always in force): whenever its time window "
                f"opens ({when}; clock {now.isoformat(timespec='milliseconds')}), "
                f"it opens on the aircraft, and no action clears the forecast from "
                f"then on (a warning, not a refusal)")
            continue
        fly = d_out / speed
        opens = _next_opening(clock, z, now, min(fly, _LINT_HORIZON_S))
        if opens is not None:
            out.append(
                f"{z.id}: the mission starts at ({sx:.1f}, {sy:.1f}, {sup:.1f}), "
                f"inside its margin ring, {opens:.2f} s before its time window opens "
                f"(clock {now.isoformat(timespec='milliseconds')}), but leaving from "
                f"there takes {fly:.1f} s ({d_out:.1f} m at {why}): the window opens "
                f"on the aircraft, and {_opening_outcome(policy, z)} (a warning, "
                f"not a refusal)")
    return out


def _mission_id(m: Mission) -> str:
    if m.mission_id:
        return m.mission_id
    canon = json.dumps({"task": m.task_text, "start": [m.start_x, m.start_y],
                        "target": [m.target_x, m.target_y],
                        "alt": m.cruise_alt_m, "speed": m.speed_pref_mps},
                       sort_keys=True, separators=(",", ":"))
    return "m-" + hashlib.sha256(canon.encode("utf-8")).hexdigest()[:12]


class ConstraintCompiler:
    def __init__(self, policy: Policy):
        self.policy = policy

    def lint_warnings(self, mission: Mission | None = None, now: datetime | None = None,
                      lookahead_s: float = DEFAULT_LOOKAHEAD_S,
                      speed_mps: float | None = None) -> list[str]:
        """The policy's lint WARNINGS (never refusals; `Policy.lint()` holds
        the errors): today the time-window lead test
        (`time_window_lead_warnings`). With a mission and a clock, the
        mission-level test too, from its start at its cruise altitude."""
        start = (None if mission is None
                 else (mission.start_x, mission.start_y, mission.cruise_alt_m))
        return time_window_lead_warnings(self.policy, lookahead_s=lookahead_s,
                                         speed_mps=speed_mps, start=start, now=now)

    # ---------------- command -> Mission ---------------- #

    def parse_command(self, text: str, default_alt: float = 15.0,
                      default_speed: float = 6.0) -> Mission:
        """Resolve ambiguous human text into an exact, structured mission."""
        low = text.lower()

        target = None
        for name, xy in PLACES.items():
            if name in low:
                target = xy
                break
        m = re.search(r"\(?\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)?", low)
        if target is None and m:
            target = (float(m.group(1)), float(m.group(2)))
            # The target's numbers are spent: "fly at low altitude to (5, 5)"
            # must not read 5 m as the altitude (see _ALT_RE).
            low = low.replace(m.group(0), " ")
        if target is None:
            raise ValueError(f"cannot resolve a target from: {text!r} "
                             f"(known places: {list(PLACES)})")

        # Parse speed FIRST and strip it, so "6 m/s" can never be mistaken
        # for an altitude (bit us once: "... 6 m/s altitude 20" parsed alt=6).
        speed = default_speed
        m = re.search(r"(\d+(?:\.\d+)?)\s*m/s", low)
        speed_read = m is not None
        if m:
            speed = float(m.group(1))
            low = low.replace(m.group(0), " ")

        alt = default_alt
        m = _ALT_RE.search(low)
        if m:
            alt = float(m.group(1) or m.group(2))

        defaults = []
        if m is None:
            defaults.append("cruise_alt_m")
        if not speed_read:
            defaults.append("speed_pref_mps")
        return Mission(task_text=text, target_x=target[0], target_y=target[1],
                       cruise_alt_m=alt, speed_pref_mps=speed,
                       defaults_used=defaults)

    # ---------------- policy (+ mission) -> CSP ---------------- #

    def summary_pack(self, mission: Mission | None = None,
                     lookahead_s: float = DEFAULT_LOOKAHEAD_S,
                     now: datetime | None = None, **csp_kw) -> dict:
        """Every rule in force, in one dict, with the hash of its policy.

        WITHOUT a mission this is the original pack, unchanged, because
        `build_prompt` embeds it and tools/build_architecture_svg.py and
        tests/test_bundle.py read it: policy_id, version, generation,
        policy_hash, origin, n_rules, rules_by_type and every rule in full.
        Nothing is summarised away here - this half is the record.

        WITH a mission it adds one key, "csp": the typed Constraint Summary Pack
        from `compile_csp`, dumped to JSON-ready form. That is the grant's CSP
        (filtered, risk-graded, budgeted, explained); use `compile_csp` directly
        to get the Pydantic object.
        """
        rules = []
        for c in self.policy.constraints:
            d = c.model_dump(mode="json")
            rules.append(d)
        pack = {
            "policy_id": self.policy.policy_id,
            "version": self.policy.version,
            "generation": self.policy.generation,
            "policy_hash": self.policy.policy_hash,
            "origin": (self.policy.origin.model_dump()
                       if self.policy.origin else None),
            "n_rules": len(rules),
            "rules_by_type": _count_types(self.policy),
            "rules": rules,
        }
        if mission is not None:
            pack["csp"] = self.compile_csp(mission, lookahead_s, now,
                                           **csp_kw).model_dump(mode="json")
        return pack

    def compile_csp(self, mission: Mission,
                    lookahead_s: float = DEFAULT_LOOKAHEAD_S,
                    now: datetime | None = None, *,
                    budget_tokens: int = DEFAULT_BUDGET_TOKENS,
                    token_counter: TokenCounter | None = None,
                    weights: RiskWeights | None = None,
                    region_margin_m: float | None = DEFAULT_REGION_MARGIN_M,
                    show_priority: bool = False,
                    issued_at: datetime | str | None = None,
                    cost_map: "CostMap | None" = None) -> CSP:
        """The Prefix Compiler: policy + mission + clock -> CSP (PDF p3).

        cost_map        - a CostMap rasterised from THIS policy and generation,
                          at THIS clock (`now`) and at the mission's cruise
                          height (`ConstraintCompiler.cost_map(now=now,
                          altitude_m=mission.cruise_alt_m)`); its `ref`
                          becomes the CSP's cost_map_ref. Any other map is
                          refused (_check_cost_map_for). Default None, as in
                          the grant ("for path planners; lazy-loaded
                          artefact"): no CSP compiled without one changed on
                          2026-10-08.

        now             - the clock the time filter uses, in the policy's local
                          time (the same convention as Shield(now=...)). None
                          switches the time filter off: every rule is treated as
                          in force, never the reverse.
        issued_at       - the CSP's issue stamp. Default: `now`, else the wall
                          clock (the only non-argument input; see _issue_stamp).
                          Where it came from is recorded in
                          selection.issued_at_source.
        budget_tokens   - limit on natural_language_prompt, counted by
                          token_counter (default csp.DEFAULT_TOKEN_COUNTER, an
                          upper bound on OpenVLA's tokenizer).
        region_margin_m - widening of the mission bbox for the spatial filter;
                          None switches the region filter off.
        show_priority   - append "[P0]" etc. to each sentence. Off by default,
                          matching the spec's worked prompt (PDF p2); priority is
                          always carried structurally (P0_constraints vs the
                          summaries, and the order of the sentences).

        Raises CSPAltitudeConflict (a ValueError) if the hard altitude bands
        in force at one instant have no height in common (_action_sets),
        CSPBudgetExceeded if the P0 sentences alone exceed the budget,
        CSPUnenforcedRules if the policy carries a declarable-only rule
        type the Shield does not enforce (models.RUNTIME_TYPES) or an MSL
        altitude, and ValueError for a cost map that is not this CSP's
        (another policy, generation, clock or height).
        """
        if not (0.0 < float(lookahead_s) <= _MAX_LOOKAHEAD_S):
            raise ValueError(f"lookahead_s must be in (0, {_MAX_LOOKAHEAD_S:g}], "
                             f"got {lookahead_s!r}")
        if int(budget_tokens) != budget_tokens or budget_tokens < 1:
            raise ValueError(f"budget_tokens must be a positive integer, got {budget_tokens!r}")
        if region_margin_m is not None and region_margin_m < 0:
            raise ValueError(f"region_margin_m must be >= 0 or None, got {region_margin_m!r}")
        issued, issued_src = _issue_stamp(issued_at, now)
        pol = self.policy
        # A rule the Shield does not enforce can be neither told to the model
        # nor dropped silently (csp.CSPUnenforcedRules). Checked before any
        # rendering, so the error names the rules instead of a missing template.
        # An MSL altitude is named too (2026-10-08): the type is enforced, the
        # height reference is not (Policy.unenforced_rules refuses it for
        # flight; a Policy built directly reached the compiler without that).
        unenforced = {c.id: c.type for c in pol.constraints if c.type not in RUNTIME_TYPES}
        unenforced.update({c.id: f"{c.type} with altitude_ref MSL"
                           for c in pol.constraints
                           if getattr(c, "altitude_ref", None) == "MSL"
                           and c.id not in unenforced})
        if unenforced:
            raise CSPUnenforcedRules(pol.policy_id, unenforced)
        if cost_map is not None:
            _check_cost_map_for(cost_map, pol, mission, now)
        # The CSP refers to rules by id (geometry_ref, relevance_explanations,
        # selection). Two rules sharing an id would overwrite each other here
        # and one would vanish from the record without a trace.
        ids = [c.id for c in pol.constraints]
        dup = sorted({i for i in ids if ids.count(i) > 1})
        if dup:
            raise ValueError(f"{pol.policy_id}: rule ids must be unique to be "
                             f"referenced from a CSP; duplicated: {dup}")
        counter = token_counter or _csp.DEFAULT_TOKEN_COUNTER
        w = weights or RiskWeights()

        # Rules a time_window_switch or corridor_swap touches are timed by the
        # Shield's own logic (_RuleClock); every other rule as before.
        clock = _RuleClock(pol)
        acts = {c.id: (clock.activity(c, now, lookahead_s) if clock.involved(c)
                       else _activity(c, now, lookahead_s))
                for c in pol.constraints}
        # How far the vehicle can get in one lookahead: the tightest hard speed
        # cap among caps that can be in force, else what the operator asked for.
        caps = [c.speed_max_mps for c in pol.by_type(KinematicEnvelope)
                if c.constraint_type == "hard" and acts[c.id]["any"]]
        reach = (min(caps) if caps else mission.speed_pref_mps) * lookahead_s
        ctx = _Ctx(mission, lookahead_s, now, reach, region_margin_m, pol, clock)

        # ---- filter, grade, render each rule ----
        rows: dict[str, dict] = {}
        for c in pol.constraints:
            row = {"rule": c, "act": acts[c.id]}
            if not acts[c.id]["any"]:
                held = clock.holders(c, now) if clock.involved(c) else []
                if now is None:
                    row["reason"] = (f"held off by {', '.join(held) or 'its schedule'}, "
                                     f"in force at every time (no clock was given); "
                                     f"the Shield does not enforce it while that "
                                     f"holds")
                else:
                    start = now.isoformat(timespec="seconds")
                    why = (f"held off by {', '.join(held)}" if held
                           else window_text(c))
                    row["reason"] = (f"not in force at any time from {start} to "
                                     f"{_fmt_g(lookahead_s)} s later ({why}); "
                                     f"the rule stays in the policy and the Shield "
                                     f"applies it whenever it is in force")
                row["decision"] = "filtered_inactive"
            elif not _in_region(c, ctx):
                d, _ = _fence_dist(c, ctx)
                row["decision"] = "filtered_out_of_region"
                row["reason"] = (f"zone lies outside the mission region (the "
                                 f"path's bounding box widened by "
                                 f"{_fmt_m(ctx.prox_scale_m)} m); its margin "
                                 f"ring is {_fmt_m(d)} m from the straight path")
            else:
                sev = _r4(_severity(c))
                prox = _r4(_proximity(c, ctx))
                tc = 1.0 if (acts[c.id]["on"] or acts[c.id]["off"]) else 0.0
                row.update(severity=sev, proximity=prox, time_critical=tc,
                           risk=_r4(w.severity * sev + w.proximity * prox
                                    + w.time_critical * tc),
                           sentence=render_sentence(c, show_priority),
                           relevance=_relevance(c, acts[c.id], ctx))
                row["tokens"] = counter.count(row["sentence"])
            rows[c.id] = row

        # ---- order + truncate (PDF p4) ----
        def rank(r):
            return (-r["risk"], r["rule"].id)
        inscope = [r for r in rows.values() if "risk" in r]
        p0 = sorted([r for r in inscope if r["rule"].priority == "P0"], key=rank)
        rest = sorted([r for r in inscope if r["rule"].priority != "P0"], key=rank)
        parts = [r["sentence"] for r in p0]
        p0_tokens = counter.count(" ".join(parts))
        if p0_tokens > budget_tokens:
            raise CSPBudgetExceeded(p0_tokens, budget_tokens,
                                    [r["rule"].id for r in p0], counter.name)
        for r in p0:
            r["decision"], r["reason"] = "kept", r["relevance"]
        used, full = p0_tokens, False
        for r in rest:
            cand = counter.count(" ".join(parts + [r["sentence"]]))
            # Strict prefix by risk: once one rule does not fit, no lower-risk
            # rule is let in after it, so every kept P1/P2 rule outranks every
            # dropped one (tests check min kept risk >= max dropped risk).
            if not full and cand <= budget_tokens:
                parts.append(r["sentence"])
                used = cand
                r["decision"], r["reason"] = "kept", r["relevance"]
            else:
                full = True
                r["decision"] = "dropped_budget"
                r["reason"] = (f"omitted from the text: with its {r['tokens']} "
                               f"tokens the prompt would need {cand} of the "
                               f"{budget_tokens}-token budget ({counter.name}); "
                               f"still in the action sets. Why it matters: "
                               f"{r['relevance']}")
        prompt = " ".join(parts)

        # ---- render the CSP ----
        kept = [r for r in p0 + rest if r["decision"] == "kept"]
        # The scale is PICKED here and the geometry is the IR's (WP2-16): fine
        # for a zone the vehicle can reach in one lookahead, coarse beyond; a
        # keep-in tube is never coarsened; a moving zone is fine (its coarse
        # ring would sit at a position it has left).
        ir = compiled_ir(pol)
        p0_entries = []
        for r in p0:
            c = r["rule"]
            if isinstance(c, _ZONES) and not is_moving(c):
                d, _ = _fence_dist(c, ctx)
                scale = "fine" if d <= ctx.reach_m else "coarse"
            elif isinstance(c, _ZONES + _TUBES):
                scale = "fine"
            else:
                scale = "param"
            # The IR mints the ref and refuses a scale it does not carry.
            ref = (f"param:{c.id}@v{pol.version}/g{pol.generation}" if scale == "param"
                   else ir.geometry_ref(scale, c.id))
            # in_force: the rule's own window, then whatever switches or
            # replaces it. A P0 time_window_switch is itself a P0 entry (scale
            # "param", in_force = its own window) and counts in P0 coverage:
            # the vehicle cannot violate an event, but its sentence is what
            # tells the model when its target is off, so a CSP that dropped it
            # would leave the model believing the target always binds - or
            # never does.
            in_force = window_text(c) or "always"
            acted = clock.acted_on_text(c)
            if acted is not None:
                in_force = f"{in_force}; {acted}"
            p0_entries.append(P0Entry(
                id=c.id, type=c.type, geometry_ref=ref,
                violation_action=c.violation_action, constraint_type=c.constraint_type,
                in_force=in_force))

        def summary(prio: str) -> tuple[str | None, list[str]]:
            k = [r["rule"] for r in kept if r["rule"].priority == prio]
            dropped = [r["rule"].id for r in rest
                       if r["rule"].priority == prio and r["decision"] == "dropped_budget"]
            text = "; ".join(render_summary(c) for c in k)
            if dropped:
                text = (text + "; " if text else "") + \
                    "omitted for budget: " + ", ".join(dropped)
            return (text + ".") if text else None, dropped

        p1, _ = summary("P1")
        if p1 is None:
            p1 = "no P1 rules in scope."
        p2, _ = summary("P2")
        if p2 is not None and not any(r["rule"].priority == "P2" for r in kept):
            p2 = None                 # "only if budget allows"; selection says why

        in_scope_rules = [r["rule"] for r in rows.values() if "risk" in r]
        allowed, forbidden = _action_sets(
            in_scope_rules,
            None if now is None else _in_force_sets(in_scope_rules, clock, now,
                                                    lookahead_s))

        decisions = []
        for c in pol.constraints:
            r = rows[c.id]
            decisions.append(RuleDecision(
                id=c.id, type=c.type, priority=c.priority,
                constraint_type=c.constraint_type, decision=r["decision"],
                severity=r.get("severity"), proximity=r.get("proximity"),
                time_critical=r.get("time_critical"), risk=r.get("risk"),
                tokens=r.get("tokens"), sentence=r.get("sentence"),
                reason=r["reason"]))

        return CSP(
            csp_version=CSP_VERSION,
            policy_hash=pol.policy_hash,
            generation=pol.generation,
            issued_at=issued,
            mission_id=_mission_id(mission),
            lookahead_s=float(lookahead_s),
            P0_constraints=p0_entries,
            P1_summary=p1,
            P2_summary=p2,
            allowed_action_set=allowed,
            forbidden_action_set=forbidden,
            cost_map_ref=None if cost_map is None else cost_map.ref,
            natural_language_prompt=prompt,
            relevance_explanations={c.id: rows[c.id]["relevance"]
                                    for c in pol.constraints
                                    if rows[c.id].get("relevance") is not None},
            policy_id=pol.policy_id,
            selection=Selection(
                budget_tokens=int(budget_tokens), tokens_used=used,
                p0_tokens=p0_tokens, token_counter=counter.name, weights=w,
                now=None if now is None else now.isoformat(timespec="seconds"),
                time_filter=now is not None, region=ctx.region,
                region_margin_m=region_margin_m, reach_m=_r4(reach),
                show_priority=show_priority, issued_at_source=issued_src,
                rules=decisions),
        )

    def cost_map(self, **kw) -> CostMap:
        """The cost-map adapter over this policy (`rasterise_cost_map`); pass
        the result to `compile_csp(cost_map=...)` to set cost_map_ref (drawn
        with the CSP's `now` and `altitude_m` = the mission's cruise height,
        or compile_csp refuses it), and `CostMap.save(stem)` to export it
        (.npz + JSON header)."""
        return rasterise_cost_map(self.policy, **kw)

    def write_csp(self, path: str | Path, mission: Mission,
                  lookahead_s: float = DEFAULT_LOOKAHEAD_S,
                  now: datetime | None = None, **csp_kw) -> Path:
        """Emit the typed CSP as JSON, so it can travel with a flight's
        artefacts (one per policy generation is what the KPIs need).

        With `cost_map=` (a CostMap rasterised from this policy and
        generation, see `cost_map`), the map is exported beside the CSP as
        `costmap_<csp stem>.npz` + `costmap_<csp stem>.json` (CostMap.save)
        and the CSP's cost_map_ref names it, so the reference travels with a
        file that `load_cost_map` resolves. The prefix keeps the map out of
        the `csp_g*.json` pattern the replay bundle reads (guardrail/replay.py).
        The CSP is compiled first, so a map of another policy, clock or
        height is refused before anything is written."""
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        csp = self.compile_csp(mission, lookahead_s, now, **csp_kw)
        cm = csp_kw.get("cost_map")
        if cm is not None:
            cm.save(p.with_name(f"costmap_{p.stem}"))
        text = csp.model_dump_json(indent=2)
        with open(p, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text + "\n")
        return p

    # ---------------- Mission + policy -> YAML prompt ---------------- #

    def _sentences(self) -> list[str]:
        """One plain sentence per rule, for the model rather than the reviewer.

        Rendered from templates/sentence/*.j2, in the original by-type order
        (zones, corridors, altitude, kinematics, clearance, stand-off) so
        `build_prompt` keeps its shape. A rule with a time window now carries
        it, and a soft rule says "(soft limit)"; a hard rule without a window
        reads exactly as on 2026-09-01.
        """
        return [render_sentence(rule) for rule in _by_type_order(self.policy)]

    def build_prompt(self, mission: Mission) -> str:
        """Render the structured YAML prompt for the VLA slot (the demos write
        it to disk as prompt.yaml; the stub and BC pilots take the Mission and
        policy objects, and no flown VLA has read this text).
        Templated, never free-form — same rule as the grant's NL adapter."""
        pack = self.summary_pack()
        alts = self.policy.by_type(AltitudeEnvelope)
        kins = self.policy.by_type(KinematicEnvelope)

        doc = {
            "mission": {
                "task": mission.task_text,
                "target": {"x": mission.target_x, "y": mission.target_y},
                "cruise_alt_m": mission.cruise_alt_m,
            },
            "constraints": {
                "policy_id": pack["policy_id"],
                "policy_hash": pack["policy_hash"],
                # The shorthands the demos already read, kept so this stays a
                # drop-in replacement...
                "no_fly_zones": [
                    {"id": f.id, "vertices": [{"x": v.x, "y": v.y} for v in f.vertices]}
                    for f in self.policy.by_type(PolygonFence)],
                "altitude_band_m": (
                    [alts[0].alt_min_m, alts[0].alt_max_m] if alts else None),
                "speed_max_mps": kins[0].speed_max_mps if kins else None,
                # ...and the full pack underneath them, so no rule is invisible
                # merely because it has no shorthand.
                "summary_pack": pack,
            },
        }
        doc["natural_language_prompt"] = " ".join(self._sentences())
        return yaml.safe_dump(doc, sort_keys=False, allow_unicode=True)

    def write_summary_pack(self, path: str | Path) -> Path:
        """Emit the CSP as a file, so it can travel with a flight's artefacts."""
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.summary_pack(), indent=2) + "\n",
                     encoding="utf-8")
        return p


# --------------------------------------------------------------------------- #
# CLI: python -m guardrail.compiler {report,schema,table,costmap,costmap-report}
# --------------------------------------------------------------------------- #

# The mission the KPI report compiles every policy for. Fixed, and the report
# compiles with no clock, so its CSPs are stamped from the wall clock; its rows
# therefore carry csp_content_hash (every field but the stamp). With both, two
# reports differ only where the code or the policies do (a test runs it twice
# under a clock that ticks between the runs).
REPORT_MISSION =Mission(task_text="fly to (30, 30) altitude 15", target_x=30.0,
                         target_y=30.0, cruise_alt_m=15.0, speed_pref_mps=4.0)

_REPO_ROOT = Path(__file__).resolve().parents[1]


def _rel(p: Path) -> str:
    try:
        return Path(p).resolve().relative_to(_REPO_ROOT).as_posix()
    except ValueError:
        return Path(p).resolve().as_posix()


def _share(num: int, den: int) -> float | None:
    return (num / den) if den else None


def coverage_report(policies_dir: str | Path,
                    budget_tokens: int = DEFAULT_BUDGET_TOKENS,
                    exact: TokenCounter | None = None) -> dict:
    """The compiler's KPI numbers over a folder of policies, each with its null
    (audit cards WP2-02 and WP2-03: "no script, KPI file or report computes
    coverage across all fixtures").

    No clock and no region filter, so every P0 rule in every file is in scope:
    the strict reading of "P0 coverage = 100% across the test fixtures" (PDF
    p5). The null is MEASURED, not asserted: the same scorer is run on each CSP
    with its P0 entries and prompt emptied, so a scorer that could not say 0 %
    would show here as a non-zero null.

    A policy whose P0 rules alone exceed the budget raises CSPBudgetExceeded
    (KPI 3). It has no CSP, so it is listed under `raised` and left out of
    every coverage share - and if no policy compiled, the shares are None, not
    100 %: "nothing measured" must not read as a pass. A policy whose hard
    altitude bands cannot all hold at once (CSPAltitudeConflict: with no clock
    every rule is in force, so two scheduled bands the lint accepts because
    they never meet in time clash here) is listed the same way, under
    `altitude_conflict` with the rule ids. Until 2026-10-19 that one file
    raised a bare ValueError and stopped the whole report.

    Two floors are reported, because they answer different questions:
      null      - the same scorer on an emptied CSP: can the scorer say 0 %?
      baseline  - `naive_prefix_baseline` at the same budget: what does a
                  compiler with no priority and no risk score? Where it equals
                  the KPI (every policy fits whole, as at 256 tokens),
                  `kpi_discriminates` is False and the 100 % shows only that
                  nothing had to be cut. On the raised files it is reported
                  separately: there the baseline drops P0 rules silently.
    """
    from .manifest import code_revision
    from .models import load_policy
    files = sorted(Path(policies_dir).glob("*.yaml"))
    rows: list[dict] = []
    raised: list[dict] = []
    not_flyable: list[dict] = []
    altitude_conflict: list[dict] = []
    for f in files:
        # A file the Shield cannot fly as written (a declarable-only rule type,
        # an unplaceable frame: Policy.flight_problems) has no CSP - the CSP
        # summarises the rules the Shield enforces (csp.CSPUnenforcedRules).
        # It is listed by name with its reasons and left out of every share,
        # like `raised`. Until 2026-10-07 one such file in the folder stopped
        # the whole report at load_policy. Every flyable file loads exactly as
        # before (runtime=True); only a refused one is re-read as declared.
        try:
            pol = load_policy(f)
        except ValueError:
            pol = load_policy(f, runtime=False)
            problems = pol.flight_problems()
            if not problems:
                raise
            not_flyable.append({"file": _rel(f), "policy_id": pol.policy_id,
                                "policy_hash": pol.policy_hash,
                                "n_rules": len(pol.constraints), "problems": problems})
            continue
        base = {"file": _rel(f), "policy_id": pol.policy_id,
                "policy_hash": pol.policy_hash, "n_rules": len(pol.constraints)}
        naive = naive_prefix_baseline(pol, budget_tokens)
        try:
            csp = ConstraintCompiler(pol).compile_csp(
                REPORT_MISSION, now=None, region_margin_m=None,
                budget_tokens=budget_tokens)
        except CSPBudgetExceeded as e:
            raised.append({**base, "p0_tokens": e.p0_tokens,
                           "budget_tokens": e.budget_tokens, "p0_ids": e.p0_ids,
                           "baseline_n_p0": naive["n_p0"],
                           "baseline_n_p0_covered": naive["n_p0_covered"],
                           "baseline_p0_missing": naive["missing"]})
            continue
        except CSPAltitudeConflict as e:
            altitude_conflict.append({**base, "rule_ids": e.rule_ids,
                                      "problem": str(e)})
            continue
        cov = p0_coverage(csp, pol)
        null = p0_coverage(csp.model_copy(update={"P0_constraints": [],
                                                  "natural_language_prompt": ""}), pol)
        rc = rule_coverage(csp, pol)["all"]
        in_csp = [r for r in csp.selection.rules
                  if r.decision in ("kept", "dropped_budget")]
        rows.append({**base,
                     "n_p0_in_scope": cov["n_p0_in_scope"],
                     "n_p0_covered": cov["n_p0_covered"],
                     "p0_coverage": cov["coverage"],
                     "p0_missing": cov["missing"],
                     "null_n_p0_covered": null["n_p0_covered"],
                     "baseline_n_p0_covered": naive["n_p0_covered"],
                     "baseline_p0_missing": naive["missing"],
                     "n_rules_in_scope": rc["n_in_scope"],
                     "n_rules_in_text": rc["n_in_text"],
                     "n_dropped_budget": sum(1 for r in in_csp
                                             if r.decision == "dropped_budget"),
                     "n_rules_in_csp": len(in_csp),
                     "n_rules_explained": sum(1 for r in in_csp
                                              if r.id in csp.relevance_explanations),
                     "tokens_used": csp.selection.tokens_used,
                     "tokens_exact": (exact.count(csp.natural_language_prompt)
                                      if exact else None),
                     "csp_content_hash": _csp.csp_content_hash(csp)})

    def tot(k: str) -> int:
        return sum(r[k] for r in rows)
    cov = _share(tot("n_p0_covered"), tot("n_p0_in_scope"))
    base_cov = _share(tot("baseline_n_p0_covered"), tot("n_p0_in_scope"))
    raised_p0 = sum(r["baseline_n_p0"] for r in raised)
    raised_base = sum(r["baseline_n_p0_covered"] for r in raised)
    cmd = (f"python -m guardrail.compiler report --policies {_rel(Path(policies_dir))}"
           f" --budget {budget_tokens}" + (" --exact" if exact else ""))
    return {
        "what": "Prefix Compiler acceptance KPIs over a policy folder "
                "(Prefix Compiler PDF p5)",
        "command": cmd,
        "code_revision": code_revision(),
        "policies_dir": _rel(Path(policies_dir)),
        "mission": REPORT_MISSION.model_dump(mode="json"),
        "clock": None,
        "region_filter": None,
        "scope_note": "no clock and no region filter: every P0 rule of every "
                      "file is in scope",
        "budget_tokens": int(budget_tokens),
        "token_counter": _csp.DEFAULT_TOKEN_COUNTER.name,
        "exact_counter": exact.name if exact else None,
        "totals": {
            "n_policies": len(files),
            "n_compiled": len(rows),
            "n_raised_budget_exceeded": len(raised),
            "n_not_flyable": len(not_flyable),
            "n_altitude_conflict": len(altitude_conflict),
            "n_p0_in_scope": tot("n_p0_in_scope"),
            "n_p0_covered": tot("n_p0_covered"),
            "p0_coverage": cov,
            "null_n_p0_covered": tot("null_n_p0_covered"),
            "null_p0_coverage": _share(tot("null_n_p0_covered"), tot("n_p0_in_scope")),
            "baseline_n_p0_covered": tot("baseline_n_p0_covered"),
            "baseline_p0_coverage": base_cov,
            # None when nothing compiled; False when the naive cut scores the
            # same, i.e. the budget never forced a choice.
            "kpi_discriminates": (None if cov is None or base_cov is None
                                  else cov > base_cov),
            "raised_baseline_n_p0": raised_p0,
            "raised_baseline_n_p0_covered": raised_base,
            "n_rules_in_scope": tot("n_rules_in_scope"),
            "n_rules_in_text": tot("n_rules_in_text"),
            "rule_text_coverage": _share(tot("n_rules_in_text"), tot("n_rules_in_scope")),
            "n_dropped_budget": tot("n_dropped_budget"),
            "n_rules_in_csp": tot("n_rules_in_csp"),
            "n_rules_explained": tot("n_rules_explained"),
            "max_tokens_used": max((r["tokens_used"] for r in rows), default=None),
            "max_tokens_exact": (max((r["tokens_exact"] for r in rows), default=None)
                                 if exact else None),
        },
        "rows": rows,
        "raised": raised,
        "not_flyable": not_flyable,
        "altitude_conflict": altitude_conflict,
    }


def _pct(x: float | None) -> str:
    return "-" if x is None else f"{x:.0%}"


def _report(args) -> int:
    exact = _csp.openvla_token_counter() if args.exact else None
    if args.exact and exact is None:
        print("exact counter unavailable (needs `tokenizers` and "
              f"{_csp.DEFAULT_OPENVLA_DIR}/tokenizer.json)", file=sys.stderr)
        return 2
    rep = coverage_report(args.policies, args.budget, exact)
    print(f"{'policy':34s} rules  P0  cover   text  tokens" + ("  exact" if exact else ""))
    for r in rep["rows"]:
        line = (f"{Path(r['file']).stem:34s} {r['n_rules']:5d} {r['n_p0_in_scope']:3d} "
                f"{_pct(r['p0_coverage']):>6s} {_pct(_share(r['n_rules_in_text'], r['n_rules_in_scope'])):>6s}"
                f" {r['tokens_used']:7d}")
        if exact:
            line += f" {r['tokens_exact']:6d}"
        print(line)
    for r in rep["raised"]:
        print(f"{Path(r['file']).stem:34s} {r['n_rules']:5d}   CSPBudgetExceeded: P0 needs "
              f"{r['p0_tokens']} of {r['budget_tokens']} tokens")
    for r in rep.get("not_flyable", []):
        print(f"{Path(r['file']).stem:34s} {r['n_rules']:5d}   not flyable as written: "
              + "; ".join(r["problems"]))
    # A file whose hard altitude bands clash has no CSP either. Until the
    # 2026-10-19 review coverage_report listed it but this printout did not,
    # so the summary read "over N of M policies" with nothing saying which
    # file was missing or why - a report that is silent about what it left
    # out reads as a pass.
    for r in rep.get("altitude_conflict", []):
        print(f"{Path(r['file']).stem:34s} {r['n_rules']:5d}   CSPAltitudeConflict: "
              f"{r['problem']}")
    t = rep["totals"]
    print(f"\nP0 coverage {t['n_p0_covered']}/{t['n_p0_in_scope']} "
          f"({_pct(t['p0_coverage'])}) over {t['n_compiled']} of {t['n_policies']} "
          f"policies; null (same scorer, P0 entries and prompt emptied) "
          f"{t['null_n_p0_covered']}/{t['n_p0_in_scope']} ({_pct(t['null_p0_coverage'])})")
    disc = {True: "the KPI discriminates at this budget",
            False: "the KPI does NOT discriminate at this budget: the naive cut "
                   "scores the same, nothing had to be chosen",
            None: "nothing compiled"}[t["kpi_discriminates"]]
    print(f"baseline (old by-type text cut as a prefix to the same budget) "
          f"{t['baseline_n_p0_covered']}/{t['n_p0_in_scope']} "
          f"({_pct(t['baseline_p0_coverage'])}); {disc}")
    if rep["raised"]:
        print(f"on the {len(rep['raised'])} raised file(s) the naive cut would have "
              f"carried {t['raised_baseline_n_p0_covered']} of their "
              f"{t['raised_baseline_n_p0']} P0 rules, without saying so")
    print(f"all-rule text coverage {t['n_rules_in_text']}/{t['n_rules_in_scope']} "
          f"({_pct(t['rule_text_coverage'])}); {t['n_dropped_budget']} rule(s) dropped "
          f"for budget; {t['n_rules_explained']}/{t['n_rules_in_csp']} rules in the "
          f"CSPs explained")
    print(f"max tokens {t['max_tokens_used']} of {rep['budget_tokens']} "
          f"({rep['token_counter']})"
          + (f"; exact max {t['max_tokens_exact']} ({rep['exact_counter']})" if exact else ""))
    if rep["raised"]:
        print(f"{len(rep['raised'])} policy file(s) raised CSPBudgetExceeded and are "
              f"not in the coverage shares")
    if rep.get("not_flyable"):
        print(f"{len(rep['not_flyable'])} policy file(s) are not flyable as written "
              f"(Policy.flight_problems), have no CSP, and are not in the coverage shares")
    if rep.get("altitude_conflict"):
        print(f"{len(rep['altitude_conflict'])} policy file(s) raised CSPAltitudeConflict "
              f"(hard altitude bands with no height in common; no clock, so every "
              f"rule is read as in force), have no CSP, and are not in the coverage "
              f"shares")
    print(f"code {rep['code_revision']}; reproduce: {rep['command']}")
    if args.json:
        out = Path(args.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(rep, indent=2) + "\n")
        print(f"wrote {out}")
    return 0


def _table(args) -> int:
    """Regenerate csp._OPENVLA_TABLE: exact counts for every template chunk
    without digits or quotes, over every policy and every schedule shape."""
    exact = _csp.openvla_token_counter()
    if exact is None:
        print("needs the exact tokenizer", file=sys.stderr)
        return 2
    from .models import ValidTime, load_policy
    rules = []
    for f in sorted(Path(args.policies).glob("*.yaml")):
        rules += load_policy(f).constraints
    # Every schedule shape window_text can print, on every rule type, hard and
    # soft, so the qualifier words ("(in", "force", "Mon-Fri", "overnight)."...)
    # are tabled wherever they land.
    shapes = [list(DAYS[:5]), list(DAYS[5:]), list(DAYS), ["Mon", "Wed", "Fri"],
              list(DAYS[1:4]), ["Sun"]]
    variants = []
    for kind in sorted({type(c) for c in rules}, key=lambda k: k.__name__):
        base = next(c for c in rules if type(c) is kind)
        for days in shapes:
            for start, end in (("07:30", "17:30"), ("22:00", "06:00"), ("00:00", "23:59")):
                vt = ValidTime.model_validate({"recurrence": {
                    "days": days, "start_time": start, "end_time": end}})
                for ctype in ("hard", "soft"):
                    variants.append(base.model_copy(update={
                        "constraint_type": ctype, "valid_time": vt}))
    texts = []
    for c in rules + variants:
        texts += [render_sentence(c), render_sentence(c, True), render_summary(c)]
    words = sorted({w for t in texts for w in t.split(" ")
                    if w and not re.search(r"[0-9'\"]", w)})
    digest = hashlib.sha256(
        (Path(_csp.DEFAULT_OPENVLA_DIR) / "tokenizer.json").read_bytes()).hexdigest()[:12]
    print(f'_OPENVLA_TABLE_SOURCE = "openvla-7b tokenizer.json sha256:{digest}"')
    print("_OPENVLA_TABLE: dict[str, int] = {")
    for w in words:
        print(f"    {w!r}: {exact.count(w)},")
    print("}")
    return 0


def cost_map_report(policies_dir: str | Path, *, resolution_m: float = COST_MAP_RESOLUTION_M,
                    n_points: int = 1000, seed: int = 0) -> dict:
    """The cost map's agreement with the Shield over a folder of policies,
    each with its two nulls (see cost_map_agreement): no clock, every moving
    zone where declared, no subject and no obstacle map, and no particular
    height (altitude_m None: every keep-out zone drawn whatever its band,
    every corridor read laterally - see rasterise_cost_map for why that
    reading is permissive for a corridor's band). A policy with nothing to
    draw is listed, not scored.

    `missed` is scored under the grant's corridor_outside = 50, for hard
    corridors too; `hard_rule_priced_finite` counts the points a hard rule
    forbids that the map prices finitely, so the headline "missed 0" is
    never read without it."""
    from .manifest import code_revision
    from .models import load_policy
    rows, skipped = [], []
    for f in sorted(Path(policies_dir).glob("*.yaml")):
        try:
            pol = load_policy(f)
        except ValueError as exc:
            skipped.append({"file": _rel(f), "why": str(exc).splitlines()[0][:160]})
            continue
        try:
            cm = rasterise_cost_map(pol, resolution_m=resolution_m)
        except ValueError as exc:
            skipped.append({"file": _rel(f), "why": str(exc).splitlines()[0][:160]})
            continue
        ag = cost_map_agreement(cm, pol, n_points=n_points, seed=seed)
        rows.append({"file": _rel(f), "policy_hash": pol.policy_hash,
                     "shape": cm.header["shape"], "ref": cm.ref,
                     "layers": [(L["name"], L["source"], L["cells"])
                                for L in cm.header["cost_map"]["layers"]],
                     "not_drawn": [d["id"] for d in cm.header["not_drawn"]],
                     **ag})

    def tot(part: str, k: str) -> int:
        return sum(r[part][k] for r in rows)
    n_f = sum(r["n_forbidden"] for r in rows)
    n_h = sum(r["n_forbidden_by_hard"] for r in rows)
    n_a = sum(r["n_allowed"] for r in rows)
    totals = {"n_policies_scored": len(rows), "n_skipped": len(skipped),
              "n_points": sum(r["n_points"] for r in rows),
              "n_forbidden": n_f, "n_forbidden_by_hard": n_h, "n_allowed": n_a}
    for part in ("map", "null_all_free", "null_all_inf"):
        hp = tot(part, "hard_rule_priced_finite")
        totals[part] = {"missed": tot(part, "missed"),
                        "hard_rule_priced_finite": hp,
                        "over_cost": tot(part, "over_cost"),
                        "missed_rate": (tot(part, "missed") / n_f) if n_f else None,
                        "hard_rule_priced_finite_rate": (hp / n_h) if n_h else None,
                        "over_cost_rate": (tot(part, "over_cost") / n_a) if n_a else None}
    totals["hard_rule_priced_finite_by_policy"] = {
        Path(r["file"]).stem: r["map"]["hard_rule_priced_finite"]
        for r in rows if r["map"]["hard_rule_priced_finite"]}
    return {"what": "cost-map adapter vs the Shield's own rule_status, per point",
            "command": (f"python -m guardrail.compiler costmap-report --policies "
                        f"{_rel(Path(policies_dir))} --resolution {resolution_m:g} "
                        f"--points {n_points} --seed {seed}"),
            "code_revision": code_revision(), "resolution_m": resolution_m,
            "totals": totals, "rows": rows, "skipped": skipped}


def _costmap(args) -> int:
    from .models import load_policy
    pol = load_policy(args.policy)
    now = datetime.fromisoformat(args.now) if args.now else None
    subject = None
    if args.subject:
        parts = args.subject.split(",")
        subject = (float(parts[0]), float(parts[1])) + ((parts[2],) if len(parts) > 2 else ())
    cm = rasterise_cost_map(pol, now=now, zone_age_s=args.zone_age,
                            altitude_m=args.altitude, resolution_m=args.resolution,
                            subject=subject)
    npz, js = cm.save(args.out)
    hdr = cm.header
    print(f"{js}  +  {npz.name}")
    print(f"  grid      {hdr['shape'][0]} x {hdr['shape'][1]} at {cm.resolution_m:g} m, "
          f"origin (x, y) = {tuple(hdr['origin_xy_m'])}")
    for L in hdr["cost_map"]["layers"]:
        print(f"  layer     {L['name']:18s} {str(L['source'] or ''):28s} cost {L['cost']!s:>4s} "
              f"{L['cells']} cells")
    for d in hdr["not_drawn"]:
        print(f"  not drawn {d['id']}: {d['why']}")
    for k in ("not_in_force", "out_of_band"):
        if hdr[k]:
            print(f"  {k:9s} {', '.join(hdr[k])}")
    print(f"  ref       {cm.ref}")
    ag = cost_map_agreement(cm, pol, now=now, n_points=args.points)
    print(f"  vs Shield {ag['n_points']} points ({ag['n_forbidden']} forbidden, "
          f"{ag['n_forbidden_by_hard']} by a hard rule): missed {ag['map']['missed']}, "
          f"hard-forbidden but priced finite {ag['map']['hard_rule_priced_finite']}, "
          f"over-cost {ag['map']['over_cost']}; nulls: all-free missed "
          f"{ag['null_all_free']['missed']}, all-inf over-cost "
          f"{ag['null_all_inf']['over_cost']}")
    return 0


def _costmap_report(args) -> int:
    rep = cost_map_report(args.policies, resolution_m=args.resolution,
                          n_points=args.points, seed=args.seed)
    for r in rep["rows"]:
        print(f"{Path(r['file']).stem:34s} {r['shape'][0]:4d}x{r['shape'][1]:<4d} "
              f"forbidden {r['n_forbidden']:4d}/{r['n_points']:<5d} missed "
              f"{r['map']['missed']:3d} hard-priced-finite "
              f"{r['map']['hard_rule_priced_finite']:3d} over-cost "
              f"{r['map']['over_cost']:4d}")
    for s in rep["skipped"]:
        print(f"{Path(s['file']).stem:34s} skipped: {s['why']}")
    t = rep["totals"]
    print(f"\n{t['n_points']} points over {t['n_policies_scored']} policies, "
          f"{t['n_forbidden']} forbidden by the Shield ({t['n_forbidden_by_hard']} "
          f"by a hard rule):")
    for part in ("map", "null_all_free", "null_all_inf"):
        p = t[part]
        print(f"  {part:14s} missed {p['missed']:5d} ({_pct(p['missed_rate'])})  "
              f"hard-forbidden priced finite {p['hard_rule_priced_finite']:5d} "
              f"({_pct(p['hard_rule_priced_finite_rate'])})  "
              f"over-cost {p['over_cost']:5d} ({_pct(p['over_cost_rate'])})")
    if t["map"]["hard_rule_priced_finite"]:
        print(f"  'missed' uses the grant's corridor_outside = 50 for hard corridors "
              f"too; the map prices {t['map']['hard_rule_priced_finite']} points the "
              f"Shield refuses (by policy: {t['hard_rule_priced_finite_by_policy']})")
    print(f"code {rep['code_revision']}; reproduce: {rep['command']}")
    if args.json:
        out = Path(args.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(rep, indent=2) + "\n")
        print(f"wrote {out}")
    return 0 if t["map"]["missed"] == 0 else 1


# The X,Y options of this CLI, and what a negative X looks like.
_XY_FLAGS = ("--subject",)
_NEGATIVE_XY = re.compile(r"^-\.?\d")


def join_negative_xy(argv: list[str], flags=_XY_FLAGS) -> list[str]:
    """`["--subject", "-20,15"]` -> `["--subject=-20,15"]` (review of
    2026-10-19). argparse takes a separate value that starts with '-' only
    when it reads as a negative NUMBER, and "-20,15" does not (the comma), so
    `costmap --subject -20,15` - a subject west of the origin - exited with
    status 2 ("expected one argument"). The same fix as the rails'
    (sitl/mavlink_adapter_node.join_negative_xy, which this package does not
    import); safe because no option here starts with '-' and a digit.
    Public since the review of F8-harness-tools: demo/follow_vlm.py
    (`--park-at`) and demo/semantic_seek.py (`--target-xy`, `--target-scale`)
    pass their argv through it with their own `flags`, which must likewise
    hold no option that starts with '-' and a digit."""
    out: list[str] = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in flags and i + 1 < len(argv) and _NEGATIVE_XY.match(argv[i + 1]):
            out.append(f"{a}={argv[i + 1]}")
            i += 2
            continue
        out.append(a)
        i += 1
    return out


_join_negative_xy = join_negative_xy      # the name tests/test_csp.py calls


def _lint(args) -> int:
    """`python -m guardrail.compiler lint POLICY...`: print each policy's lint
    warnings. Warnings never refuse, so this exits 0 whatever it finds (a
    policy that fails the DSL's error lint does not load, and that is
    reported by load_policy as always)."""
    from .models import load_policy
    start = None
    if args.start:
        start = tuple(float(v) for v in args.start.split(","))
        if len(start) != 3:
            raise SystemExit("--start takes X,Y,UP")
    now = datetime.fromisoformat(args.now) if args.now else None
    for p in args.policy:
        pol = load_policy(p, runtime=False)
        ws = time_window_lead_warnings(pol, lookahead_s=args.lookahead,
                                       speed_mps=args.speed, start=start, now=now)
        if not ws:
            print(f"{p}: no lint warnings")
        for w in ws:
            print(f"{p}: WARNING {w}")
    return 0


def _main(argv: list[str] | None = None) -> int:
    argv = _join_negative_xy(list(sys.argv[1:] if argv is None else argv))
    ap = argparse.ArgumentParser(prog="python -m guardrail.compiler")
    sub = ap.add_subparsers(dest="cmd", required=True)
    cm = sub.add_parser("costmap", help="rasterise one policy (.npz + JSON header)")
    cm.add_argument("policy")
    cm.add_argument("-o", "--out", required=True, help="output stem (writes STEM.npz, STEM.json)")
    cm.add_argument("--resolution", type=float, default=COST_MAP_RESOLUTION_M)
    cm.add_argument("--now", default=None, help="ISO time for valid_time windows")
    cm.add_argument("--zone-age", type=float, default=None,
                    help="seconds since each moving dynamic_nfz appeared")
    cm.add_argument("--altitude", type=float, default=None)
    cm.add_argument("--subject", default=None, help="X,Y[,CLASS] of the followed subject")
    cm.add_argument("--points", type=int, default=1000)
    cr = sub.add_parser("costmap-report", help="cost map vs the Shield over a folder")
    cr.add_argument("--policies", default=str(Path(__file__).resolve().parents[1] / "policies"))
    cr.add_argument("--resolution", type=float, default=COST_MAP_RESOLUTION_M)
    cr.add_argument("--points", type=int, default=1000)
    cr.add_argument("--seed", type=int, default=0)
    cr.add_argument("--json", default=None)
    r = sub.add_parser("report", help="P0 coverage and token use over a policy folder")
    r.add_argument("--policies", default=str(Path(__file__).resolve().parents[1] / "policies"))
    r.add_argument("--budget", type=int, default=DEFAULT_BUDGET_TOKENS)
    r.add_argument("--exact", action="store_true", help="also count with OpenVLA's tokenizer")
    r.add_argument("--json", default=None,
                   help="also write the whole report (rows, totals, nulls, code "
                        "revision, reproducing command) to this JSON file")
    s = sub.add_parser("schema", help="write csp.schema.json")
    s.add_argument("out")
    t = sub.add_parser("table", help="print a regenerated token table for csp.py")
    t.add_argument("--policies", default=str(Path(__file__).resolve().parents[1] / "policies"))
    lw = sub.add_parser("lint", help="the DSL's lint WARNINGS for one or more policies "
                                     "(time windows that can open on an aircraft that "
                                     "cannot leave in time); always exits 0")
    lw.add_argument("policy", nargs="+")
    lw.add_argument("--lookahead", type=float, default=DEFAULT_LOOKAHEAD_S,
                    help="the Shield's forecast, s (the grant's 5)")
    lw.add_argument("--speed", type=float, default=None,
                    help="m/s to leave a zone at (default: the policy's hard speed "
                         f"cap, else {LINT_SPEED_MPS:g})")
    lw.add_argument("--start", default=None, help="X,Y,UP of a mission's start")
    lw.add_argument("--now", default=None, help="ISO clock at the mission's start")
    args = ap.parse_args(argv)
    if args.cmd == "lint":
        return _lint(args)
    if args.cmd == "costmap":
        return _costmap(args)
    if args.cmd == "costmap-report":
        return _costmap_report(args)
    if args.cmd == "report":
        return _report(args)
    if args.cmd == "schema":
        print(_csp.write_csp_schema(args.out))
        return 0
    return _table(args)


if __name__ == "__main__":
    sys.exit(_main())
