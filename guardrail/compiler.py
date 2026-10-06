"""
Prefix Compiler - the "before the VLA" half of the Guardrail.

Meeting architecture slot:

    User Command -> [Constraint Compiler] -> YAML Prompt / CSP -> VLA -> ...

Two jobs live here.

1. COMMAND -> MISSION (`parse_command`). A natural-language command resolved
   into a structured Mission. v0 parsing is deliberately simple: a named-place
   registry + coordinate regex. A real deployment would use map data / an LLM
   here; the *interface* (text in, validated Mission out) is what matters.

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

   It is a stateless function, as PDF p1 requires, GIVEN A CLOCK: same policy
   (hash + generation) + same mission + same `now` -> byte-identical CSP, in
   any process. Nothing is random, so the spec's "rule-selection seed" has
   nothing to seed; ties in risk are broken by rule id. The one input that can
   come from outside the arguments is `issued_at`: it is the `issued_at`
   argument if given, else `now`, and only when neither is given the wall
   clock. So a CSP compiled without a clock differs from run to run in that one
   field, and `csp.csp_content_hash` - every field except the issue stamp - is
   the identity to compare such CSPs by (the KPI report does).

What the CSP is FOR, stated so nobody over-reads it: it is advice to the model,
not enforcement. The Shield enforces every rule in the policy whether or not
the CSP mentioned it. A rule the CSP filtered or dropped is still a rule; the
CSP's `selection` says which and why, so a cut is never silent.

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
from datetime import datetime, timedelta
from pathlib import Path

import yaml
from pydantic import BaseModel

from . import csp as _csp
from .csp import (CSP, CSP_VERSION, AllowedActionSet, CSPBudgetExceeded,
                  ForbiddenActionSet, P0Entry, RiskWeights, RuleDecision,
                  Selection, TokenCounter)
from .geometry import nearest_on_polyline, point_in_fence
from .models import (DAYS, AltitudeEnvelope, Corridor, KinematicEnvelope,
                     ObstacleClearance, Policy, PolygonFence, SubjectStandoff)

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
# bounding rectangle. See multiscale_geometry.
COARSE_MAX_VERTICES = 8

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


def render_sentence(rule, show_priority: bool = False) -> str:
    """One rule as one sentence (templates/sentence/<type>.j2)."""
    return render_template(f"sentence/{rule.type}.j2", {
        "r": rule, "window": window_text(rule),
        "soft": rule.constraint_type == "soft",
        "show_priority": show_priority, "priority": rule.priority})


def render_summary(rule) -> str:
    """One rule as a short clause for P1_summary / P2_summary."""
    return render_template(f"summary/{rule.type}.j2", {
        "r": rule, "window": window_text(rule),
        "soft": rule.constraint_type == "soft"})


# --------------------------------------------------------------------------- #
# Geometry for SUMMARISING, not enforcing.
#
# Pure Python on purpose. guardrail/geometry.py is the only module that imports
# shapely, and it holds the Shield's rule evaluation; these helpers only score
# and describe, and nothing the Shield decides depends on them. Where a check
# does matter for safety (does this action enter a zone?) the code below calls
# geometry.point_in_fence, the Shield's own test.
# --------------------------------------------------------------------------- #

Pt = tuple[float, float]


def _cross(o: Pt, a: Pt, b: Pt) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _convex_hull(pts: list[Pt]) -> list[Pt]:
    """Andrew's monotone chain, counter-clockwise, no repeated first point."""
    p = sorted(set(pts))
    if len(p) <= 2:
        return p
    lower: list[Pt] = []
    for q in p:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    upper: list[Pt] = []
    for q in reversed(p):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return lower[:-1] + upper[:-1]


def _min_area_rect(hull: list[Pt]) -> list[Pt]:
    """Smallest-area rectangle containing a convex hull (one side lies on a hull
    edge - rotating calipers, done by brute force over the edges)."""
    best = None
    n = len(hull)
    for i in range(n):
        (ax, ay), (bx, by) = hull[i], hull[(i + 1) % n]
        L = math.hypot(bx - ax, by - ay)
        if L < 1e-12:
            continue
        ux, uy = (bx - ax) / L, (by - ay) / L          # along the edge
        vx, vy = -uy, ux                               # normal
        us = [x * ux + y * uy for x, y in hull]
        vs = [x * vx + y * vy for x, y in hull]
        area = (max(us) - min(us)) * (max(vs) - min(vs))
        if best is None or area < best[0] - 1e-12:
            best = (area, ux, uy, vx, vy, min(us), max(us), min(vs), max(vs))
    _, ux, uy, vx, vy, u0, u1, v0, v1 = best
    return [(u * ux + v * vx, u * uy + v * vy)
            for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]


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


# Built once per policy hash. The grant puts this cache in the IR ("the policy
# bundle's IR already carries pre-computed multi-scale polygons ... The Prefix
# Compiler picks the scale, never re-computes", PDF p1); our IR (models.Policy)
# does not carry it, and models.py / bundle.py are WP1's files. Until the IR
# does, it is computed here once per policy and memoised - recorded as a
# deviation in docs/DESIGN-prefix-compiler.md.
_GEOMETRY_CACHE: dict[str, dict] = {}
_GEOMETRY_CACHE_MAX = 64


def _r6(pts) -> list[Pt]:
    return [(round(x, 6) + 0.0, round(y, 6) + 0.0) for x, y in pts]


def multiscale_geometry(policy: Policy) -> dict[str, dict[str, list[Pt]]]:
    """{rule id: {"fine": [...], "coarse": [...]}} for every zone and corridor.

    fine   - the vertices as authored (the Shield adds margin_m itself).
    coarse - a polygon that CONTAINS the fine one: its convex hull, or, if the
             hull still has more than COARSE_MAX_VERTICES vertices, the
             minimum-area oriented rectangle around it. Containment is the
             point: a coarse keep-out zone that cut a corner would tell the
             model a smaller zone than the Shield enforces.
    Corridors get "fine" only (their centerline). Coarsening a keep-IN shape
    outward would widen where the model believes it may fly.
    """
    key = policy.policy_hash
    hit = _GEOMETRY_CACHE.get(key)
    if hit is not None:
        return hit
    out: dict[str, dict[str, list[Pt]]] = {}
    for c in policy.constraints:
        if isinstance(c, PolygonFence):
            fine = [(v.x, v.y) for v in c.vertices]
            hull = _convex_hull(fine)
            coarse = hull if len(hull) <= COARSE_MAX_VERTICES else _min_area_rect(hull)
            out[c.id] = {"fine": _r6(fine), "coarse": _r6(coarse)}
        elif isinstance(c, Corridor):
            out[c.id] = {"fine": _r6(c.points())}
    if len(_GEOMETRY_CACHE) >= _GEOMETRY_CACHE_MAX:
        _GEOMETRY_CACHE.pop(next(iter(_GEOMETRY_CACHE)))
    _GEOMETRY_CACHE[key] = out
    return out


_REF = re.compile(r"^(fine|coarse|param):(.+)@v([^/@]+)/g(\d+)$")


def _geometry_ref(scale: str, rule_id: str, policy: Policy) -> str:
    return f"{scale}:{rule_id}@v{policy.version}/g{policy.generation}"


def resolve_geometry_ref(policy: Policy, ref: str) -> list[Pt] | None:
    """Vertices for a P0Entry.geometry_ref, or None for a "param:" ref.

    Refuses a ref whose version or generation is not this policy's: after a
    mid-flight hot-apply bumps the generation, a CSP from before must not
    quietly resolve against geometry it never described.
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
    geo = multiscale_geometry(policy).get(rid)
    if geo is None or scale not in geo:
        raise ValueError(f"no {scale} geometry for {rid!r} in {policy.policy_id}")
    return list(geo[scale])


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


class _Ctx:
    """The mission context, computed once per compilation."""

    def __init__(self, mission: Mission, lookahead_s: float, now, reach_m: float,
                 region_margin_m: float | None):
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


def _fence_dist(f: PolygonFence, ctx: _Ctx) -> tuple[float, float]:
    """(path distance, target distance) to the fence's MARGIN RING - the edge
    the Shield actually enforces. Exact: the distance to a polygon buffered by
    m is the distance to the polygon minus m, floored at 0."""
    poly = [(v.x, v.y) for v in f.vertices]
    d_path = max(0.0, _seg_poly(ctx.a, ctx.b, poly) - f.margin_m)
    d_tgt = max(0.0, _seg_poly(ctx.b, ctx.b, poly) - f.margin_m)
    return d_path, d_tgt


def _in_region(rule, ctx: _Ctx) -> bool:
    """Only keep-OUT zones are ever filtered by region.

    A corridor is keep-IN: everywhere outside it is forbidden, so "far from the
    mission" is precisely the case where it matters most (the mission is
    outside it). Envelopes and stand-offs have no location. The test is a
    bounding-box overlap, which can only keep MORE zones than an exact test
    would, never fewer.
    """
    if ctx.region is None or not isinstance(rule, PolygonFence):
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
    have no location - so 1."""
    if not isinstance(rule, PolygonFence):
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
    if isinstance(rule, PolygonFence):
        d_path, d_tgt = _fence_dist(rule, ctx)
        facts = {"crosses": d_path <= 0.0, "sx": sx, "sy": sy, "tx": tx, "ty": ty,
                 "d_path": d_path, "d_target": d_tgt, "alt": m.cruise_alt_m,
                 "floor": rule.altitude_floor_m, "ceil": rule.altitude_ceiling_m,
                 "in_band": rule.altitude_floor_m <= m.cruise_alt_m <= rule.altitude_ceiling_m}
    elif isinstance(rule, Corridor):
        hw, pts = rule.half_width_m, rule.points()
        facts = {"start_in": nearest_on_polyline(sx, sy, pts)[0] <= hw,
                 "target_in": nearest_on_polyline(tx, ty, pts)[0] <= hw,
                 "hw": hw, "alt": m.cruise_alt_m, "floor": rule.altitude_floor_m,
                 "ceil": rule.altitude_ceiling_m,
                 "alt_in_band": rule.altitude_floor_m <= m.cruise_alt_m <= rule.altitude_ceiling_m}
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
        who = ("any followed subject" if rule.subject_class == "*"
               else f"any {rule.subject_class}")
        facts = {"follow": bool(_FOLLOW_WORDS.search(m.task_text)), "who": who}
    else:                                     # pragma: no cover - new rule type
        raise TypeError(f"no relevance template for {type(rule).__name__}")
    facts["time_fact"] = tf
    return render_template(f"reason/{rule.type}.j2", facts)


# --------------------------------------------------------------------------- #
# Action sets
# --------------------------------------------------------------------------- #

def _action_sets(rules: list) -> tuple[AllowedActionSet, ForbiddenActionSet]:
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
    """
    hard = [r for r in rules if r.constraint_type == "hard"]
    kins = [r for r in hard if isinstance(r, KinematicEnvelope)]
    alts = [r for r in hard if isinstance(r, AltitudeEnvelope)]
    cors = [r for r in hard if isinstance(r, Corridor)]
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
        lo = max(b[1] for b in bands)
        hi = min(b[2] for b in bands)
        if lo > hi:
            raise ValueError("altitude bands in scope (envelopes and corridors) "
                             "do not overlap: "
                             + ", ".join(f"{i} {b0:g}-{b1:g} m" for i, b0, b1 in bands))
        kw["altitude_band_m_agl"] = (lo, hi)
        src += [b[0] for b in bands]
    allowed = AllowedActionSet(source_rules=src, **kw)
    forbidden = ForbiddenActionSet(
        no_translate_into_polygons=[r.id for r in hard if isinstance(r, PolygonFence)],
        no_translate_out_of_corridors=[r.id for r in hard if isinstance(r, Corridor)])
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
                      dt: float = 1.0) -> list[str]:
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
    Shield, not this function, decides what flies.
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
    if policy.policy_hash != csp.policy_hash:
        raise ValueError(f"CSP is for {csp.policy_hash}, policy is {policy.policy_hash}")
    by_id = {c.id: c for c in policy.constraints}
    for fid in f.no_translate_into_polygons:
        if point_in_fence(x, y, up, by_id[fid]):
            out.append(f"polygon:{fid}")
    for cid in f.no_translate_out_of_corridors:
        c = by_id[cid]
        d, _, _ = nearest_on_polyline(x, y, c.points())
        if d > c.half_width_m or not (c.altitude_floor_m <= up <= c.altitude_ceiling_m):
            out.append(f"corridor:{cid}")
    return out


def p0_coverage(csp: CSP, policy: Policy) -> dict:
    """KPI 2 (PDF p5): the share of in-scope P0 rules the CSP carries.

    A P0 rule counts as covered only if it is in P0_constraints AND its
    sentence is in natural_language_prompt - the structured record and the text
    the model reads must both have it. In scope means the CSP did not filter it
    as not in force or out of region; a rule missing from `selection` entirely
    counts as in scope and uncovered. Zero P0 rules in scope gives coverage
    None: "nothing to cover" is not 100 %, and it is not 0 % either.
    """
    if policy.policy_hash != csp.policy_hash:
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
    if policy.policy_hash != csp.policy_hash:
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
    baseline below cuts it."""
    out: list = []
    for cls in (PolygonFence, Corridor, AltitudeEnvelope, KinematicEnvelope,
                ObstacleClearance, SubjectStandoff):
        out += policy.by_type(cls)
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
        if target is None:
            raise ValueError(f"cannot resolve a target from: {text!r} "
                             f"(known places: {list(PLACES)})")

        # Parse speed FIRST and strip it, so "6 m/s" can never be mistaken
        # for an altitude (bit us once: "... 6 m/s altitude 20" parsed alt=6).
        speed = default_speed
        m = re.search(r"(\d+(?:\.\d+)?)\s*m/s", low)
        if m:
            speed = float(m.group(1))
            low = low.replace(m.group(0), " ")

        alt = default_alt
        m = re.search(r"(?:alt|altitude|height|tinggi)\D{0,3}(\d+(?:\.\d+)?)|"
                      r"(\d+(?:\.\d+)?)\s*m\s+(?:alt|altitude|height)", low)
        if m:
            alt = float(m.group(1) or m.group(2))

        return Mission(task_text=text, target_x=target[0], target_y=target[1],
                       cruise_alt_m=alt, speed_pref_mps=speed)

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
                    issued_at: datetime | str | None = None) -> CSP:
        """The Prefix Compiler: policy + mission + clock -> CSP (PDF p3).

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

        Raises CSPBudgetExceeded if the P0 sentences alone exceed the budget.
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

        acts = {c.id: _activity(c, now, lookahead_s) for c in pol.constraints}
        # How far the vehicle can get in one lookahead: the tightest hard speed
        # cap among caps that can be in force, else what the operator asked for.
        caps = [c.speed_max_mps for c in pol.by_type(KinematicEnvelope)
                if c.constraint_type == "hard" and acts[c.id]["any"]]
        reach = (min(caps) if caps else mission.speed_pref_mps) * lookahead_s
        ctx = _Ctx(mission, lookahead_s, now, reach, region_margin_m)

        # ---- filter, grade, render each rule ----
        rows: dict[str, dict] = {}
        for c in pol.constraints:
            row = {"rule": c, "act": acts[c.id]}
            if not acts[c.id]["any"]:
                start = now.isoformat(timespec="seconds")
                row["decision"] = "filtered_inactive"
                row["reason"] = (f"not in force at any time from {start} to "
                                 f"{_fmt_g(lookahead_s)} s later ({window_text(c)}); "
                                 f"the rule stays in the policy and the Shield "
                                 f"applies it whenever it is in force")
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
        geo = multiscale_geometry(pol)
        p0_entries = []
        for r in p0:
            c = r["rule"]
            if isinstance(c, PolygonFence):
                d, _ = _fence_dist(c, ctx)
                scale = "fine" if d <= ctx.reach_m else "coarse"
            elif isinstance(c, Corridor):
                scale = "fine"
            else:
                scale = "param"
            if scale != "param" and scale not in geo.get(c.id, {}):
                raise RuntimeError(f"no {scale} geometry cached for {c.id}")
            p0_entries.append(P0Entry(
                id=c.id, type=c.type, geometry_ref=_geometry_ref(scale, c.id, pol),
                violation_action=c.violation_action, constraint_type=c.constraint_type,
                in_force=window_text(c) or "always"))

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
        allowed, forbidden = _action_sets(in_scope_rules)

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
            cost_map_ref=None,
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

    def write_csp(self, path: str | Path, mission: Mission,
                  lookahead_s: float = DEFAULT_LOOKAHEAD_S,
                  now: datetime | None = None, **csp_kw) -> Path:
        """Emit the typed CSP as JSON, so it can travel with a flight's
        artefacts (one per policy generation is what the KPIs need)."""
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        text = self.compile_csp(mission, lookahead_s, now, **csp_kw).model_dump_json(indent=2)
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
        """Render the structured YAML prompt the VLA receives.
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
# CLI: python -m guardrail.compiler {report,schema,table}
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
    100 %: "nothing measured" must not read as a pass.

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
    for f in files:
        pol = load_policy(f)
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


def _main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m guardrail.compiler")
    sub = ap.add_subparsers(dest="cmd", required=True)
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
    args = ap.parse_args(argv)
    if args.cmd == "report":
        return _report(args)
    if args.cmd == "schema":
        print(_csp.write_csp_schema(args.out))
        return 0
    return _table(args)


if __name__ == "__main__":
    sys.exit(_main())
