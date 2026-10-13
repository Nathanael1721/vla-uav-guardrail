"""Draw the explainer figures (end-to-end flowchart, four architecture views) as SVG,
and PNG through Edge headless.

    python tools/deck/build_explainer_figures.py            # SVG only, plus checks
    python tools/deck/build_explainer_figures.py --png      # also docs/img/explainer_*.png
    python tools/deck/build_explainer_figures.py --out DIR  # write somewhere else

Writes five figures under docs/img/, all named explainer_*:

    explainer_flowchart_end_to_end   policy file to KPI report, one mission
    explainer_arch_components        the four work packages, their files, the data between them
    explainer_arch_topologies        dev / hil / flight: what runs where, flown vs planned
    explainer_arch_shield            one tick inside the Safety Shield
    explainer_arch_evidence          from a flight to a KPI number, and the harness beside it

WHY THEY ARE DRAWN FROM CODE

tools/deck/build_flowcharts.py says why: a figure that is a picture of a chat
widget cannot be corrected. These go into a light-background deck
(tools/deck/nathan_theme.js), so they use its palette and font (Poppins) and a
type size that stays readable where the deck shows them. The deck fits each
figure into a 9.36 x 4.44 in box (SLIDE_BOX_IN, figureSlide in
build_explainer_deck.js), so each canvas is about twice as wide as it is tall
and its smallest text comes out at MIN_PT (8.5 pt) or more on the slide
(`size_problems`). Step numbers and WP badges sit on box corners; no other text
may run under one.

EVERY NUMBER HAS A SOURCE

Each figure carries `facts`: for every number printed on it, the repository file
it comes from and a pattern that must match there (`Fact`). `check_facts()`
re-reads those files, so a figure whose number has drifted from the code fails
the build instead of being presented. Step badges (the circled 1, 2, 3 ...) are
ordinals, marked `class="ordinal"`, and are the only digits exempt. Every file
path printed on a figure is in `files` and must exist; every code name (an
operator, a state, a manifest field) is in `names` with the file that defines it.

LAYOUT CHECKS

`layout_problems()` is font-free: every text block fits its box's height, and
no arrow runs through a box it does not start or end on. `fit_problems()`
measures each string with the Poppins / Consolas files PIL can open (installed
on the lab machine; elsewhere the check reports SKIP and does not pass): text
fits its box, labels clear boxes, arrows and other text, and nothing leaves the
canvas. Both run on every build; a problem fails it.
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs" / "img"

# ---------------------------------------------------------------------------- #
# Palette: tools/deck/nathan_theme.js `T`, so figure and slide read as one.
# ---------------------------------------------------------------------------- #
TEAL = "#249DB2"
TEAL_DK = "#1A7484"
TEAL_DEEP = "#12545F"
TEAL_TINT = "#EAF6F8"
TEAL_TINT2 = "#D6EDF1"
BLACK = "#1A1A1A"
INK = "#2B2B2B"
GREY = "#6B7280"
LGREY = "#9AA3AD"
LINE = "#E3E8EC"
WHITE = "#FFFFFF"
THEME_HEX = (TEAL, TEAL_DK, TEAL_DEEP, TEAL_TINT, TEAL_TINT2, BLACK, INK, GREY,
             LGREY, LINE, WHITE)
# Not in the theme: a darker body grey than GREY (contrast on the tints), the
# external-component fill, the band background and the arrow colour.
SUB = "#3F4A54"
EXT_FILL = "#F2F4F6"
PAPER = "#F6F9FA"
ARROW = "#56626C"

FONT = "Poppins, 'Segoe UI', Arial, sans-serif"
MONO = "Consolas, 'Cascadia Mono', monospace"

T_SIZE = 25       # box titles
B_SIZE = 22       # box lines
L_SIZE = 22       # arrow labels
H_SIZE = 22       # band headers (bold, letter-spaced)
TAG_SIZE = 21     # WP badges, step numbers
SMALL = 21        # the smallest text: legends, module roles, small boxes
LEAD = 27         # line pitch inside a box

# Where the deck shows a figure (tools/deck/build_explainer_deck.js,
# figureSlide: bw, bh): a box of 9.36 x 4.44 in. The figure is scaled to fit
# it, so the points one SVG unit becomes depend on the canvas; the smallest
# text on every figure must still come out at MIN_PT or more on the slide.
SLIDE_BOX_IN = (9.36, 4.44)
MIN_PT = 8.5


def slide_pt_per_unit(w: int, h: int) -> float:
    """Points per SVG unit when a w x h canvas fills the deck's figure box."""
    return min(SLIDE_BOX_IN[0] * 72 / w, SLIDE_BOX_IN[1] * 72 / h)

# style -> (fill, stroke, dash, title colour, line colour, badge fill, badge text)
STYLES = {
    "ours": (TEAL_TINT, TEAL, None, BLACK, SUB, TEAL_DK, WHITE),
    "core": (TEAL_DEEP, TEAL_DEEP, None, WHITE, "#DDF0F3", WHITE, TEAL_DEEP),
    "ext": (EXT_FILL, LGREY, None, BLACK, SUB, GREY, WHITE),
    "out": (WHITE, TEAL_DK, None, BLACK, SUB, TEAL_DK, WHITE),
    "flown": (TEAL_TINT, TEAL_DK, None, BLACK, SUB, TEAL_DK, WHITE),
    "defined": (WHITE, TEAL, "8,6", BLACK, SUB, TEAL_DK, WHITE),
    "planned": (WHITE, LGREY, "8,6", "#3F4A54", "#4B5563", GREY, WHITE),
}


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


# ---------------------------------------------------------------------------- #
# Facts: every printed number, with the file it comes from
# ---------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Fact:
    """`num` as printed on the figure; `src` a repo-relative file; `pattern` a
    regex that must match in `src`. kind "text": the match must contain `num`.
    kind "count": the number of matches must equal int(num)."""
    num: str
    src: str
    pattern: str
    kind: str = "text"

    def problem(self) -> str | None:
        p = ROOT / self.src
        if not p.is_file():
            return f"{self.num!r}: source {self.src} does not exist"
        text = p.read_text(encoding="utf-8", errors="replace")
        hits = list(re.finditer(self.pattern, text, re.M))
        if self.kind == "count":
            if len(hits) != int(self.num):
                return (f"{self.num!r}: {self.src} has {len(hits)} matches of "
                        f"{self.pattern!r}, not {self.num}")
            return None
        if not hits:
            return f"{self.num!r}: {self.pattern!r} not found in {self.src}"
        if not any(re.search(rf"(?<![\d.]){re.escape(self.num)}(?![\d])", h.group(0))
                   for h in hits):
            return f"{self.num!r}: no match of {self.pattern!r} in {self.src} contains it"
        return None


# A number on a figure: digits not glued to a letter, digit or dot on the left
# (so P0, X6, WP3, Ed25519, sha256, 7B, 4418ae0 are names, not numbers), and
# not followed by a letter or digit. Versions such as 4.5.7 are one token.
NUM_RE = re.compile(r"(?<![\w.])\d+(?:\.\d+)*(?![\w])")


def numbers_in(s: str) -> list[str]:
    return NUM_RE.findall(s)


# ---------------------------------------------------------------------------- #
# The canvas
# ---------------------------------------------------------------------------- #

class Fig:
    def __init__(self, name: str, w: int, h: int):
        self.name, self.w, self.h = name, w, h
        self.parts: list[str] = []
        self.texts: list[dict] = []     # every string drawn, for the checks
        self.boxes: list[dict] = []     # solid boxes: arrows and labels keep out
        self.containers: list[dict] = []
        self.paths: list[dict] = []
        self.facts: list[Fact] = []
        self.files: list[str] = []      # repo paths printed on the figure
        self.names: list[tuple[str, str]] = []   # (code name, file defining it)
        self._n = 0

    # -- bookkeeping ---------------------------------------------------------- #
    def fact(self, num, src, pattern, kind="text"):
        self.facts.append(Fact(num, src, pattern, kind))

    def file(self, *paths):
        self.files.extend(paths)

    def name_in(self, src, *names):
        self.names.extend((n, src) for n in names)

    def _id(self, prefix):
        self._n += 1
        return f"{prefix}{self._n}"

    # -- primitives ----------------------------------------------------------- #
    def text(self, x, y, s, size=B_SIZE, weight=400, fill=INK, anchor="start",
             family=FONT, owner=None, kind="free", cls=None, spacing=None):
        sp = f' letter-spacing="{spacing}"' if spacing else ""
        c = f' class="{cls}"' if cls else ""
        halo = (' stroke="#FFFFFF" stroke-width="7" stroke-linejoin="round" '
                'paint-order="stroke"') if kind == "label" else ""
        fam = family.replace("'", "&apos;")
        self.parts.append(
            f'<text x="{x:g}" y="{y:g}" font-family="{fam}" font-size="{size}" '
            f'font-weight="{weight}" fill="{fill}" text-anchor="{anchor}"{sp}{c}{halo}>'
            f'{esc(s)}</text>')
        self.texts.append(dict(s=s, x=x, y=y, size=size, weight=weight, family=family,
                               anchor=anchor, owner=owner, kind=kind, cls=cls,
                               spacing=spacing or 0))

    def rect(self, x, y, w, h, fill, stroke=None, sw=2.0, rx=12, dash=None):
        st = f' stroke="{stroke}" stroke-width="{sw:g}"' if stroke else ""
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(f'<rect x="{x:g}" y="{y:g}" width="{w:g}" height="{h:g}" '
                          f'rx="{rx:g}" fill="{fill}"{st}{d}/>')

    def band(self, x, y, w, h, fill=PAPER):
        """A background panel: decoration only, never a collision target."""
        self.rect(x, y, w, h, fill, LINE, 1.5, 18)

    def header(self, x, y, s, fill=TEAL_DK, anchor="start", size=H_SIZE):
        self.text(x, y, s.upper(), size=size, weight=700, fill=fill, anchor=anchor,
                  spacing=1.5, kind="header")

    def badge(self, x, y, s, fill, colour, owner):
        """A WP badge straddling a box's top edge, right-aligned at x."""
        w = 22 + 14 * len(s)
        self.rect(x - w, y - 16, w, 32, fill, WHITE, 2, 16)
        self.text(x - w / 2, y + 7.5, s, size=TAG_SIZE, weight=700, fill=colour,
                  anchor="middle", owner=owner, kind="badge")
        self.texts[-1]["rect"] = (x - w, y - 16, w, 32)

    def ordinal(self, cx, cy, n, fill=TEAL_DEEP, colour=WHITE):
        r = 19 if n < 10 else 21
        self.parts.append(f'<circle cx="{cx:g}" cy="{cy:g}" r="{r}" fill="{fill}" '
                          f'stroke="{WHITE}" stroke-width="2.5"/>')
        self.text(cx, cy + 7.5, str(n), size=TAG_SIZE, weight=700, fill=colour,
                  anchor="middle", kind="ordinal", cls="ordinal")
        self.texts[-1]["rect"] = (cx - r, cy - r, 2 * r, 2 * r)

    def box(self, x, y, w, h, title, lines=(), style="ours", tag=None, step=None,
            size=B_SIZE, tsize=T_SIZE, lead=LEAD, title_family=FONT,
            line_family=FONT, solid=True, align="middle", valign="middle", pad=24):
        """A titled box. With align="start", a line that begins with two spaces
        continues the bullet above it and is indented to match its text."""
        fill, stroke, dash, tcol, lcol, bfill, bcol = STYLES[style]
        bid = self._id("b")
        self.rect(x, y, w, h, fill, stroke, 2.2, 12, dash)
        n = len(lines)
        gap = lead + 6 if n else 0
        top_ink = 0.74 * tsize
        content = top_ink + (gap + (n - 1) * lead + 0.26 * size if n else 0.26 * tsize)
        if content + 12 > h:
            raise ValueError(f"{self.name}: box {title!r} needs {content + 12:.0f} px, has {h}")
        tb = y + (h - content) / 2 + top_ink if valign == "middle" else y + pad + top_ink
        tx = x + w / 2 if align == "middle" else x + 18
        self.text(tx, tb, title, size=tsize, weight=600, fill=tcol, anchor=align,
                  family=title_family, owner=bid, kind="box")
        for i, s in enumerate(lines):
            indent = 0
            if align == "start" and s.startswith("  "):
                indent, s = round(0.62 * size), s.strip()
            self.text(tx + indent, tb + gap + i * lead, s, size=size, fill=lcol,
                      anchor=align, family=line_family, owner=bid, kind="box")
        rec = dict(id=bid, x=x, y=y, w=w, h=h, title=title)
        (self.boxes if solid else self.containers).append(rec)
        if tag:
            self.badge(x + w - 14, y, tag, bfill, bcol, bid)
        if step is not None:
            self.ordinal(x + 4, y + 2, step)
        return rec

    def container(self, x, y, w, h, title, style="ours", sub=None, tsize=22, inline=0,
                  tx=18):
        """A machine or a column: a titled frame that holds boxes."""
        fill, stroke, dash, tcol, lcol, *_ = STYLES[style]
        bid = self._id("c")
        self.rect(x, y, w, h, fill, stroke, 2.2, 16, dash)
        self.text(x + tx, y + 32, title, size=tsize, weight=700, fill=tcol, owner=bid,
                  kind="box")
        if sub and inline:
            # beside the title; `inline` is the x offset where it starts
            self.text(x + inline, y + 32, sub, size=SMALL, fill=lcol, owner=bid, kind="box")
        elif sub:
            self.text(x + 18, y + 58, sub, size=SMALL, fill=lcol, owner=bid, kind="box")
        rec = dict(id=bid, x=x, y=y, w=w, h=h, title=title)
        self.containers.append(rec)
        return rec

    def module(self, x, y, w, path, role, shown=None, h=60):
        """One file in a work-package column: its path (mono) and its job."""
        bid = self._id("m")
        self.rect(x, y, w, h, WHITE, LINE, 1.5, 8)
        self.text(x + 12, y + 25, shown or path, size=22, weight=700, fill=TEAL_DEEP,
                  family=MONO, owner=bid, kind="box")
        self.text(x + 12, y + 50, role, size=SMALL, fill=SUB, owner=bid, kind="box")
        self.boxes.append(dict(id=bid, x=x, y=y, w=w, h=h, title=path))
        self.file(path)

    def diamond(self, cx, cy, w, h, title):
        bid = self._id("d")
        pts = f"{cx},{cy - h / 2} {cx + w / 2},{cy} {cx},{cy + h / 2} {cx - w / 2},{cy}"
        self.parts.append(f'<polygon points="{pts}" fill="{WHITE}" stroke="{TEAL_DK}" '
                          f'stroke-width="2.2"/>')
        self.text(cx, cy + 8, title, size=22, weight=600, fill=BLACK, anchor="middle",
                  owner=bid, kind="box")
        # the text's own box: the inner rectangle of the diamond
        self.boxes.append(dict(id=bid, x=cx - w / 4, y=cy - h / 4, w=w / 2, h=h / 2,
                               title=title, diamond=(cx, cy, w, h)))

    def arrow(self, pts, label=None, at=None, anchor="middle", dashed=False,
              colour=ARROW, head=True, width=2.6, both=False, size=L_SIZE):
        """A polyline. `label` is one string or a list (one line each, 26 px
        apart from `at`); both=True puts a head on each end (a two-way link)."""
        pid = self._id("p")
        d = " ".join(f"{'M' if i == 0 else 'L'}{x:g},{y:g}" for i, (x, y) in enumerate(pts))
        self.parts.append(
            f'<path d="{d}" fill="none" stroke="{colour}" stroke-width="{width}"'
            + (' stroke-dasharray="9,7"' if dashed else '')
            + (' marker-end="url(#ah)"' if head else '')
            + (' marker-start="url(#ah)"' if both else '') + '/>')
        self.paths.append(dict(id=pid, pts=list(pts)))
        if label:
            lx, ly = at
            for i, s in enumerate([label] if isinstance(label, str) else label):
                self.text(lx, ly + 26 * i, s, size=size, weight=500, fill=TEAL_DK,
                          anchor=anchor, owner=pid, kind="label")
        return pid

    def legend(self, x, y, items):
        """items: (label, style). Each swatch is a small box in that style."""
        for label, style in items:
            fill, stroke, dash, *_ = STYLES[style]
            self.rect(x, y - 19, 34, 24, fill, stroke, 2, 6, dash)
            self.text(x + 46, y, label, size=SMALL, fill=SUB, kind="legend")
            x += 46 + 11 * len(label) + 44

    def svg(self) -> str:
        head = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" '
                f'height="{self.h}" viewBox="0 0 {self.w} {self.h}">')
        defs = ('<defs><marker id="ah" viewBox="0 0 12 12" refX="11" refY="6" '
                'markerWidth="12" markerHeight="12" markerUnits="userSpaceOnUse" '
                f'orient="auto-start-reverse"><path d="M0,0.5 L12,6 L0,11.5 z" '
                f'fill="{ARROW}"/></marker></defs>')
        bg = f'<rect width="{self.w}" height="{self.h}" fill="{WHITE}"/>'
        return "\n".join([head, defs, bg] + self.parts + ["</svg>"]) + "\n"

    def save(self, path: Path):
        # newline="\n": the same bytes on every platform (the test compares them)
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(self.svg())


# ---------------------------------------------------------------------------- #
# 1. End-to-end flowchart
# ---------------------------------------------------------------------------- #

def flowchart_end_to_end() -> Fig:
    f = Fig("explainer_flowchart_end_to_end", 1600, 784)

    # ---- row 0: band headers, and the legend between them -------------------- #
    f.header(40, 28, "Before take-off")
    f.header(1280, 28, "Any time in flight")
    f.legend(330, 30, [("guardrail (ours)", "ours"), ("Safety Shield", "core"),
                       ("not ours", "ext"), ("evidence", "out")])

    # ---- band A: before take-off --------------------------------------------- #
    # Titles sit below the corner discs (step number, WP badge): valign top.
    top = dict(valign="top", pad=22)
    ay, ah, aw = 56, 138, 280
    xs = [40, 350, 660, 970, 1280]
    f.box(xs[0], ay, aw, ah, "Policy in YAML",
          ["the grant's own form;", "regulation, site and", "mission layers"],
          tag="WP1", step=1, **top)
    f.box(xs[1], ay, aw, ah, "Validated, merged",
          ["Pydantic v2: unknown", "keys refused; no layer", "relaxes a hard rule"],
          tag="WP1", step=2, **top)
    f.box(xs[2], ay, aw, ah, "Signed bundle",
          ["policy_hash: SHA-256", "of the IR; Ed25519", "signature; semver"],
          tag="WP1", step=3, **top)
    f.box(xs[3], ay, aw, ah, "Mission start",
          ["the rule clock starts;", "the CSP is compiled", "for the VLA's prompt"],
          tag="WP2", step=4, **top)
    f.box(xs[4], ay, aw, ah, "Hot-apply events",
          ["dynamic_nfz,", "time_window_switch,", "corridor_swap"], tag="WP3", **top)
    for a, b in zip(xs[:3], xs[1:4]):
        f.arrow([(a + aw, ay + ah / 2), (b, ay + ah / 2)])
    f.fact("256", "guardrail/bundle.py", r"SHA-256 over the canonicalised IR")
    f.name_in("guardrail/models.py",
              'LAYER_ORDER: tuple[str, ...] = ("regulation", "site", "mission")',
              'ConfigDict(extra="forbid"', "def merge_layers")
    f.name_in("guardrail/shield.py", "a mission-layer rule cannot")
    f.name_in("CHANGELOG.md", "one rule clock to both")
    f.name_in("guardrail/bundle.py", "policy_hash", "Ed25519", "semver")
    f.name_in("guardrail/shield.py", "dynamic_nfz", "time_window_switch", "corridor_swap")

    # ---- band B: the loop ----------------------------------------------------- #
    f.band(20, 236, 1560, 344)
    f.header(40, 260, "The loop, every 0.1 s (10 Hz)")
    f.fact("0.1", "guardrail/fsm.py", r"^CONTROL_PERIOD_S = 0\.1$")
    f.fact("10", "guardrail/shield.py", r"and a 10 Hz monitor")
    sx, sw = [40, 430, 820, 1210], 350
    ty, th = 284, 112
    by, bh = 456, 112
    f.box(sx[0], ty, sw, th, "Camera frame + task",
          ["image + instruction,", "with the CSP in the prompt"], style="ext", step=5, **top)
    f.box(sx[1], ty, sw, th, "VLA: swappable pilot",
          ["a 4-D body-frame action:", "vx, vy, vz, yaw_rate"], style="ext", step=6, **top)
    f.fact("4", "guardrail/shield.py", r"validate a 4-D action")
    f.name_in("sitl/mavlink_adapter_node.py", "a = (vx, vy, vz, yaw_rate) # body-frame")
    f.box(sx[2], ty, sw, th, "Safety Shield",
          ["forecast 5 s at 0.1 s, 50 poses;", "checks rules, repairs"],
          style="core", tag="WP3", step=7, **top)
    f.fact("5", "guardrail/shield.py", r"^LOOKAHEAD_S = 5\.0$")
    f.fact("0.1", "guardrail/shield.py", r"^LOOKAHEAD_DT_S = 0\.1$")
    f.fact("50", "guardrail/shield.py", r"50 future poses")
    f.box(sx[3], ty, sw, th, "Escalation FSM",
          ["Normal, Brake, Loiter, RTL,", "Land; Loiter flown as BRAKE"],
          tag="WP3", step=8, **top)
    f.name_in("guardrail/fsm.py", 'NORMAL = "Normal"', 'BRAKE = "Brake"',
              'LOITER = "Loiter"', 'RTL = "RTL"', 'LAND = "Land"')
    f.name_in("sitl/mavlink_adapter_node.py", 'DEFAULT_HOLD_MODE = "BRAKE"')
    f.box(sx[3], by, sw, bh, "MAVLink adapter",
          ["body frame to local NED;", "SET_MODE, fence upload"], tag="WP3", step=9, **top)
    f.name_in("sitl/mavlink_adapter_node.py", "FRAME_LOCAL_NED", "/mavros/set_mode",
              "/mavros/geofence/push")
    f.box(sx[2], by, sw, bh, "ArduPilot",
          ["flies the setpoint; its", "GeoFence is the backstop"], style="ext", step=10,
          **top)
    f.box(sx[1], by, sw, bh, "Drone / simulator",
          ["SITL or Project AirSim", "today; an airframe later"], style="ext", step=11,
          **top)
    mid = by + bh / 2
    f.diamond(215, mid, 300, 100, "Mission over?")

    for i in range(3):        # top row, left to right
        f.arrow([(sx[i] + sw, ty + th / 2), (sx[i + 1], ty + th / 2)])
    gap_label = ty + th + 34   # labels between the two rows
    f.arrow([(sx[3] + sw / 2, ty + th), (sx[3] + sw / 2, by)],
            "setpoint + mode", at=(sx[3] + sw / 2 - 14, gap_label), anchor="end")
    f.arrow([(sx[3], mid), (sx[2] + sw, mid)])
    f.arrow([(sx[2], mid), (sx[1] + sw, mid)])
    f.arrow([(sx[1], mid), (365, mid)])
    f.arrow([(sx[2] + sw / 2, by), (sx[2] + sw / 2, ty + th)],
            "state: pose, heading", at=(sx[2] + sw / 2 - 14, gap_label), anchor="end")
    f.arrow([(215, mid - 50), (215, ty + th)],
            "no: next tick", at=(229, gap_label), anchor="start")
    f.name_in("demo/pas_ardupilot/robot_citylife_ardupilot.jsonc", "ardupilot-api")

    # rules + clock into the Shield and the CSP into the prompt, both from the
    # mission start; hot-apply into the Shield. The two horizontal runs share a
    # height (they never overlap in x), their labels above them; all three
    # land clear of the Shield's WP3 badge.
    tx, seg = 1010, ay + ah + 30
    f.arrow([(tx, ay + ah), (tx, ty)], "rules + clock", at=(tx - 12, 264), anchor="end")
    f.arrow([(tx, seg), (sx[1] + sw / 2, seg), (sx[1] + sw / 2, ty)],
            "CSP into the prompt", at=(800, seg - 10), anchor="middle")
    f.arrow([(xs[4] + aw / 2, ay + ah), (xs[4] + aw / 2, seg), (1066, seg), (1066, ty)],
            "new generation", at=(xs[4] + aw / 2 - 12, seg - 10), anchor="end")

    # ---- band C: what every run leaves behind ------------------------------- #
    cy, ch = 630, 124
    f.header(290, 606, "What every run leaves behind")
    f.arrow([(215, mid + 50), (215, cy)], "yes", at=(229, 600), anchor="start")
    f.box(40, cy, 400, ch, "Run folder",
          ["flight_log.jsonl: every tick", "audit.jsonl: each changed action",
           "manifest.json: the six fields"], style="out", tag="WP4", step=12)
    f.box(490, cy, 350, ch, "Replay bundle",
          ["signed; every file", "digested; it re-derives", "its own KPIs"],
          style="out", tag="WP4", step=13)
    f.box(890, cy, 300, ch, "KPI report",
          ["n/m where nothing", "ran; a null beside", "each score"],
          style="out", tag="WP4", step=14)
    f.box(1240, cy, 320, ch, "Episode exporters",
          ["step and episode", "records, JSONL"], style="out", tag="WP4", step=15)
    f.arrow([(440, cy + ch / 2), (490, cy + ch / 2)])
    f.arrow([(840, cy + ch / 2), (890, cy + ch / 2)])
    f.arrow([(790, cy + ch), (790, cy + ch + 16), (1400, cy + ch + 16), (1400, cy + ch)])
    f.name_in("sitl/ros2_shield_node.py", "flight_log.jsonl", "audit.jsonl",
              "manifest.json")
    f.name_in("guardrail/manifest.py", "six-field determinism manifest")
    f.name_in("tools/kpi_report.py", "`n/m`", "The null beside every score")
    return f


# ---------------------------------------------------------------------------- #
# 2. Components: four work packages, their files, the data between them
# ---------------------------------------------------------------------------- #

def arch_components() -> Fig:
    f = Fig("explainer_arch_components", 1600, 760)
    ext = dict(style="ext", tsize=24, size=SMALL, lead=26)

    # not ours, on top
    f.box(500, 8, 440, 78, "VLA (swappable pilot)", ["OpenVLA-7B, stub, follow controller"],
          **ext)
    f.name_in("demo/real_vla_demo.py", "OpenVLA-7B")
    f.file("sitl/ros2_vla_stub_node.py", "demo/pas_ardupilot/perception_node.py")
    f.box(1200, 8, 360, 78, "ArduPilot + simulator", ["SITL, Project AirSim"], **ext)

    top, pitch = 120, 64
    col_h = 72 + 5 * pitch - 4 + 10

    # WP1
    f.container(40, top, 330, col_h, "WP1 · Policy DSL", sub="rules in, signed bundle out")
    y = top + 72
    for path, role, shown in [
            ("guardrail/models.py", "rules, the grant's YAML", "models.py"),
            ("guardrail/projection.py", "WGS84 to local metres", "projection.py"),
            ("guardrail/ir.py", "compiled zones, STRtree", "ir.py"),
            ("guardrail/bundle.py", "SHA-256 + Ed25519", "bundle.py"),
            ("tools/geo_to_policy.py", "GeoJSON / KML import", "geo_to_policy.py")]:
        f.module(54, y, 302, path, role, shown)
        y += pitch
    f.fact("256", "guardrail/bundle.py", r"SHA-256 over the canonicalised IR")
    f.name_in("guardrail/ir.py", "STRtree")
    f.name_in("guardrail/projection.py", "WGS84")
    f.name_in("tools/geo_to_policy.py", "GeoJSON / KML")

    # WP2
    f.container(430, top, 322, col_h, "WP2 · Prefix Compiler", sub="rules into the prompt")
    y = top + 72
    for path, role, shown in [
            ("guardrail/compiler.py", "ranks rules; 256 tokens", "compiler.py"),
            ("guardrail/csp.py", "typed CSP, Pydantic v2", "csp.py"),
            ("guardrail/paraphraser.py", "stored paraphrases", "paraphraser.py"),
            ("guardrail/templates", "a sentence per rule", "templates/")]:
        f.module(444, y, 294, path, role, shown)
        y += pitch
    f.fact("256", "guardrail/compiler.py", r"^DEFAULT_BUDGET_TOKENS = 256$")
    f.name_in("guardrail/csp.py", "Pydantic v2 for the")

    # WP3, two sub-columns
    f.container(812, top, 748, col_h, "WP3 · Safety Shield",
                sub="check, repair, escalate, hand over to ArduPilot")
    left = [("guardrail/shield.py", "forecast, checks, hot-apply", "shield.py"),
            ("guardrail/repair.py", "repair operator library", "repair.py"),
            ("guardrail/fsm.py", "escalation state machine", "fsm.py"),
            ("guardrail/rtl_path.py", "is the way home clear?", "rtl_path.py"),
            ("guardrail/fence_export.py", "zones for the GeoFence", "fence_export.py")]
    right = [("guardrail/frames.py", "body <-> world frame", "frames.py"),
             ("guardrail/audit.py", "the audit record", "audit.py"),
             ("guardrail/api.py", "REST hot-apply", "api.py"),
             ("sitl/ros2_shield_node.py", "the ROS 2 Shield node", "ros2_shield_node.py"),
             ("sitl/mavlink_adapter_node.py", "body to NED, MAVROS 2",
              "mavlink_adapter_node.py")]
    for col, xx in ((left, 826), (right, 1194)):
        y = top + 72
        for path, role, shown in col:
            f.module(xx, y, 352, path, role, shown)
            y += pitch
    f.fact("2", "docs/DESIGN-topologies.md", r"MAVROS 2")
    f.fact("2", "sitl/ros2_shield_node.py", r"ROS 2 node")

    # WP4, full width at the bottom, title and subtitle on one line
    w4 = top + col_h + 50
    f.container(40, w4, 1520, 46 + 2 * pitch - 4 + 10, "WP4 · Stress Testing",
                sub="run, score, package", inline=270)
    wp4 = [("guardrail/scenario_spec.py", "ScenarioSpec, events", "scenario_spec.py"),
           ("experiments/sweep_scenarios.py", "the headless harness", "sweep_scenarios.py"),
           ("guardrail/harness_service.py", "gRPC over mutual TLS", "harness_service.py"),
           ("guardrail/manifest.py", "six-field manifest", "manifest.py"),
           ("guardrail/kpi.py", "the KPIs of one flight", "kpi.py"),
           ("tools/kpi_report.py", "the KPI report", "kpi_report.py"),
           ("guardrail/replay.py", "signed replay bundle", "replay.py"),
           ("tools/export_episodes.py", "step + episode records", "export_episodes.py")]
    for i, (path, role, shown) in enumerate(wp4):
        xx = 56 + (i % 4) * 374
        yy = w4 + 46 + (i // 4) * pitch
        f.module(xx, yy, 362, path, role, shown)
    f.name_in("guardrail/harness_service.py", "mutual TLS")
    f.name_in("guardrail/scenario_spec.py", "ScenarioSpec")
    f.name_in("guardrail/manifest.py", "six-field determinism manifest")

    # data between them: across the top gap ...
    f.arrow([(370, top + col_h / 2), (430, top + col_h / 2)], "IR",
            at=(400, top + col_h / 2 - 10))
    f.arrow([(640, top), (640, 86)], "CSP prefix", at=(628, 110), anchor="end")
    f.arrow([(880, 86), (880, top)], "raw 4-D action", at=(868, 110), anchor="end")
    f.fact("4", "guardrail/shield.py", r"validate a 4-D action")
    f.arrow([(1290, top), (1290, 86)], "setpoint, mode, fences", at=(1278, 110),
            anchor="end")
    f.arrow([(1500, 86), (1500, top)], "telemetry", at=(1488, 110), anchor="end")
    # ... and across the bottom gap
    gy = top + col_h
    f.arrow([(330, gy), (330, gy + 14), (900, gy + 14), (900, gy)],
            "rules: the signed bundle", at=(615, gy + 38))
    f.arrow([(140, gy), (140, w4)], "bundle, pinned by hash", at=(152, gy + 38),
            anchor="start")
    f.arrow([(1010, gy), (1010, w4)], "audit + flight logs", at=(1022, gy + 32),
            anchor="start")
    f.arrow([(1500, w4), (1500, gy)], "scenarios, hot-apply", at=(1488, gy + 32),
            anchor="end")
    f.name_in("guardrail/protos/harness.proto", "expected_hash")
    return f


# ---------------------------------------------------------------------------- #
# 3. Topologies
# ---------------------------------------------------------------------------- #

def arch_topologies() -> Fig:
    f = Fig("explainer_arch_topologies", 1600, 780)
    pw = 494
    px = [26, 553, 1080]
    chip = dict(tsize=22, size=SMALL, lead=22)
    ch, cp = 62, 66            # a component box, and its pitch

    # panel headers: the grant's name, where it runs, and its status today
    heads = [("dev", "one desktop", "flown today as two rails", "flown"),
             ("hil", "desktop + Jetson Orin", "KPI config · defined, not run",
              "defined"),
             ("flight", "Orin on the drone", "planned · no airframe yet", "planned")]
    for x, (t, sub, status, st) in zip(px, heads):
        f.text(x + 4, 40, t, size=34, weight=700, fill=BLACK, family=MONO, kind="header")
        f.text(x + 4 + 19 * len(t) + 18, 38, sub, size=22, fill=SUB, kind="header")
        fill, stroke, dash, *_ = STYLES[st]
        f.rect(x, 54, pw, 38, fill, stroke, 2, 19, dash)
        f.text(x + pw / 2, 80, status.upper(), size=SMALL, weight=700,
               fill=TEAL_DK if st != "planned" else SUB, anchor="middle", spacing=1.5,
               kind="header")
    f.name_in("guardrail/manifest.py", 'TOPOLOGY_DEV = "dev"', 'TOPOLOGY_HIL = "hil"',
              'TOPOLOGY_FLIGHT = "flight"')
    f.name_in("docs/DESIGN-topologies.md", "**Canonical KPI configuration**")
    f.name_in("deploy/topologies/flight.env", "the airframe does not exist yet")
    f.file("deploy/topologies/dev.env", "deploy/topologies/hil.env",
           "deploy/topologies/flight.env", "deploy/compose/docker-compose.orin.yml")

    # ---- dev ----------------------------------------------------------------- #
    x = px[0]
    cw, cx = pw - 36, x + 18
    rows = [("Project AirSim", "the CityLife city, camera", "out"),
            ("ArduPilot SITL", "Copter-4.5.7", "out"),
            ("mavlink-router", "fans MAVLink out", "out"),
            ("Mission Planner", "port measured; app not attached", "defined"),
            ("MAVROS 2", "ROS 2 Jazzy, in WSL", "out"),
            ("Safety Shield", "ROS 2 node or perception node", "out"),
            ("Pilot", "VLA stub or follow controller", "out")]
    dev_h = 46 + len(rows) * cp - 4 + 10
    f.container(x, 104, pw, dev_h, "Desktop (Windows + WSL 2)", style="flown")
    y = 150
    for t, s, st in rows:
        f.box(cx, y, cw, ch, t, [s], style=st, **chip)
        y += cp
    f.fact("2", "docs/DESIGN-topologies.md", r"addresses across WSL 2")
    f.fact("2", "docs/DESIGN-topologies.md", r"ROS 2 Jazzy")
    f.fact("4.5.7", "docs/DESIGN-topologies.md", r"\(Copter-4\.5\.7\)")
    f.name_in("docs/DESIGN-ros2-interface.md",
              "Mission Planner itself has not yet been attached to a run")
    f.name_in("sitl/gcs_listen.py", "Stand in for Mission Planner on its UDP port")
    f.file("sitl/ros2_shield_node.py", "demo/pas_ardupilot/perception_node.py",
           "sitl/ros2_vla_stub_node.py", "sitl/gcs_listen.py")
    f.box(x, 104 + dev_h + 10, pw, 780 - (104 + dev_h + 10) - 8, "Flown as two rails",
          ["• A: SITL + MAVROS 2 + Shield", "  node; its manifest says dev",
           "• B: Project AirSim + SITL over", "  pymavlink: no grant topology"],
          style="out", size=SMALL, lead=26, tsize=22, align="start")
    f.name_in("CHANGELOG.md", "pymavlink, none of the grant's topologies")
    f.name_in("CHANGELOG.md", "live MAVROS evidence (never over pymavlink)")

    # ---- hil ----------------------------------------------------------------- #
    x = px[1]
    cw, cx = pw - 36, x + 18
    hrows = [("Project AirSim", "UDP 9002/9003 with SITL"),
             ("ArduPilot SITL", "Copter-4.5.7"),
             ("mavlink-router", "from SITL on UDP 14550"),
             ("Mission Planner", "UDP 14550")]
    desk_h = 46 + len(hrows) * cp - 4 + 10
    f.container(x, 104, pw, desk_h, "Desktop", style="defined")
    y = 150
    for t, s in hrows:
        f.box(cx, y, cw, ch, t, [s], style="defined", **chip)
        y += cp
    f.fact("9002", "docs/DESIGN-topologies.md",
           r"exchanges sensor and motor packets with ArduPilot SITL over UDP 9002/9003")
    f.fact("9003", "docs/DESIGN-topologies.md", r"over UDP 9002/9003")
    f.fact("14550", "docs/DESIGN-topologies.md", r"sends to the router on UDP 14550")
    f.fact("14550", "docs/DESIGN-topologies.md", r"mavlink-router to Mission Planner \| UDP 14550")
    orin_y = 104 + desk_h + 74
    f.container(x, orin_y, pw, 46 + 3 * cp - 4 + 10, "Jetson Orin (JetPack 6.2)",
                style="defined")
    y = orin_y + 46
    for t, s in [("MAVROS 2", "its fcu_url names the desktop"),
                 ("Safety Shield node", "the same image as flight"),
                 ("VLA", "in its own container")]:
        f.box(cx, y, cw, ch, t, [s], style="defined", **chip)
        y += cp
    f.fact("6.2", "docs/DESIGN-topologies.md", r"\*\*JetPack 6\.2\*\*")
    f.arrow([(x + 80, 104 + desk_h), (x + 80, orin_y)],
            ["Ethernet: MAVLink, UDP 14555", "camera frames, TCP 8989/8990"],
            at=(x + 96, 104 + desk_h + 30), anchor="start", both=True, size=SMALL)
    f.fact("14555", "docs/DESIGN-topologies.md", r"mavlink-router to MAVROS 2 \| UDP 14555")
    f.fact("8989", "docs/DESIGN-topologies.md",
           r"camera frames for a VLA on the Orin\) \| TCP 8989/8990")
    f.fact("8990", "docs/DESIGN-topologies.md", r"TCP 8989/8990")
    f.name_in("docs/DESIGN-topologies.md", "**the same** arm64 build",
              "udp://:14555@<DESKTOP_IP>:14555")
    f.name_in("CHANGELOG.md", "Nothing has been built or run on an Orin.")

    # ---- flight -------------------------------------------------------------- #
    x = px[2]
    cw, cx = pw - 36, x + 18
    inner_y, inner_h = 178, 44 + 3 * cp - 4 + 10
    ap_y = inner_y + inner_h + 50
    drone_h = ap_y + 64 + 16 - 104
    f.container(x, 104, pw, drone_h, "Drone", style="planned", sub="no simulator, no desktop")
    f.container(cx, inner_y, cw, inner_h, "Orin companion computer", style="planned",
                tsize=SMALL)
    y = inner_y + 44
    for t, s in [("MAVROS 2", "serial link to the autopilot"),
                 ("Safety Shield node", "the same image as hil"),
                 ("VLA", "the same image as hil")]:
        f.box(cx + 16, y, cw - 32, ch, t, [s], style="planned", **chip)
        y += cp
    f.box(cx, ap_y, cw, 64, "ArduPilot autopilot", ["a real flight controller"],
          style="planned", **chip)
    f.arrow([(cx + cw / 2, inner_y + inner_h), (cx + cw / 2, ap_y)], "serial",
            at=(cx + cw / 2 + 16, inner_y + inner_h + 33), anchor="start", both=True)
    mp_y = 104 + drone_h + 60
    f.box(x, mp_y, pw, 64, "Mission Planner on the ground", ["over its own telemetry radio"],
          style="planned", **chip)
    f.arrow([(x + pw / 2, mp_y), (x + pw / 2, 104 + drone_h)], dashed=True, both=True)
    f.name_in("deploy/topologies/flight.env", "telemetry radio", "serial link",
              "There is no simulator and no desktop")
    return f


# ---------------------------------------------------------------------------- #
# 4. Inside the Safety Shield, one tick
# ---------------------------------------------------------------------------- #

def arch_shield() -> Fig:
    f = Fig("explainer_arch_shield", 1600, 786)
    f.header(40, 30, "One tick of the Safety Shield, every 0.1 s")
    f.fact("0.1", "guardrail/shield.py", r"^MONITOR_PERIOD_S = 0\.1$")
    # six stages in two rows of three, read left to right, top row first
    w, gx, h = 490, 25, 164
    xs = [40, 40 + w + gx, 40 + 2 * (w + gx)]
    ya, yb = 56, 56 + h + 40
    stage = dict(align="start", valign="top", size=B_SIZE, lead=26)
    f.box(xs[0], ya, w, h, "Inputs",
          ["• vehicle state: pose, heading", "• raw action, body to world frame",
           "• the rules in force on the", "  tick's clock"], step=1, **stage)
    f.name_in("guardrail/frames.py", "def from_body")
    f.box(xs[1], ya, w, h, "Forecast",
          ["• 5 s ahead in 0.1 s steps: 50 poses", "• each pose judged by the rules",
           "  in force at its own time"], step=2, **stage)
    f.fact("5", "guardrail/shield.py", r"^LOOKAHEAD_S = 5\.0$")
    f.fact("0.1", "guardrail/shield.py", r"^LOOKAHEAD_DT_S = 0\.1$")
    f.fact("50", "guardrail/shield.py", r"\(50 future poses\)")
    f.name_in("guardrail/shield.py", "judged against the rules in force at ITS OWN time")
    f.box(xs[2], ya, w, h, "Spatial index",
          ["• STRtree over the zones' margins", "• moving zone: reach bound first,",
           "  its polygon only where the", "  forecast can meet it"], step=3, **stage)
    f.name_in("guardrail/shield.py", "STRtree", "anchor_reach")
    f.name_in("guardrail/ir.py", "def anchor_reach")
    f.box(xs[0], yb, w, h, "Rule checks",
          ["• hard before soft, P0 > P1 > P2", "• zones, altitude, speed,",
           "  clearance, stand-off, corridor", "• monitor_only: logged, not fixed"],
          step=4, **stage)
    f.name_in("guardrail/fsm.py", '"hard >> soft, P0 > P1 > P2"', "monitor_only")
    f.box(xs[1], yb, w, h, "Repair chain",
          ["• KinematicClamp, AltitudeClamp,", "  then up to 3 passes: Clearance,",
           "  Standoff, Corridor, Lateral", "• opt-in: first-feasible, PathRepair"],
          step=5, **stage)
    f.fact("3", "guardrail/repair.py", r"up to 3 passes of\s+Clearance, Standoff, Corridor, Lateral")
    f.name_in("guardrail/repair.py", "KinematicClamp", "AltitudeClamp", "ClearanceProjection",
              "StandoffProjection", "CorridorProjection", "LateralProjection", "PathRepair",
              '"first_feasible"', "Both new behaviours are off by default")
    f.box(xs[2], yb, w, h, "Re-check",
          ["• the repaired action is checked", "  again; still illegal: a recovery",
           "• soft rules given up, lowest first", "• else BRAKE, only where legal"],
          step=6, **stage)
    f.name_in("guardrail/shield.py", "lowest priority first", "a stop only where a stop is legal",
              "still illegal? recovery")
    for row in (ya, yb):
        for a, b in zip(xs[:-1], xs[1:]):
            f.arrow([(a + w, row + h / 2), (b, row + h / 2)])
    # row A -> row B: from Spatial index down and back to Rule checks
    turn = ya + h + 12
    f.arrow([(xs[2] + w / 2, ya + h), (xs[2] + w / 2, turn), (xs[0] + w / 2, turn),
             (xs[0] + w / 2, yb)])

    # row C: the FSM and the output
    fy = yb + h + 34
    fh = 786 - fy - 8
    fw_ = 1070
    fsm = f.container(40, fy, fw_, fh, "Escalation FSM", style="ours", tx=34)
    f.ordinal(44, fy + 2, 7)
    states = ["Normal", "Brake", "Loiter", "RTL", "Land"]
    sw_, sg = 130, 92
    sy = fy + 86
    sxs = [64 + i * (sw_ + sg) for i in range(5)]
    for sx, s in zip(sxs, states):
        f.box(sx, sy, sw_, 50, s, [], style="core" if s in ("RTL", "Land") else "out",
              tsize=22)
    for i, lab in enumerate(["G1", "G2, G3", "G4, G5", "G6"]):
        x1, x2 = sxs[i] + sw_, sxs[i + 1]
        f.arrow([(x1, sy + 34), (x2, sy + 34)], lab, at=((x1 + x2) / 2, sy + 18),
                size=SMALL)
    # back to Normal (below), X6/X7 straight to RTL (above)
    f.arrow([(sxs[1] + 40, sy + 50), (sxs[1] + 40, sy + 72), (sxs[0] + 90, sy + 72),
             (sxs[0] + 90, sy + 50)], "G7", at=(sxs[1] - 8, sy + 72), size=SMALL)
    f.arrow([(sxs[2] + 64, sy + 50), (sxs[2] + 64, sy + 98), (sxs[0] + 40, sy + 98),
             (sxs[0] + 40, sy + 50)], "G8", at=(sxs[2] - 8, sy + 98), size=SMALL)
    f.arrow([(sxs[1] + 96, sy), (sxs[1] + 96, sy - 26), (sxs[3] + 40, sy - 26),
             (sxs[3] + 40, sy)], "X6, X7 (ours)", at=(sxs[2] + sw_ / 2, sy - 34), size=SMALL)
    f.arrow([(sxs[2] + 40, sy), (sxs[2] + 40, sy - 26)], head=False)   # ... from Loiter too
    f.name_in("guardrail/fsm.py", '"G1"', '"G2"', '"G3"', '"G4"', '"G5"', '"G6"', '"G7"',
              '"G8"', '"X6"', '"X7"', 'NORMAL = "Normal"', 'LAND = "Land"')
    lines = [
        "G1 a repair within theta: 2.0 m lateral, 0.5 m vertical;  G2 N = 3 onsets in T = 5 s",
        "G3 a repair over theta;  G4 N new onsets in Loiter;  G5 30 s in Loiter;  G6 RTL fails",
        "G7, G8 back to Normal after T_recover = 2 s clear;  X6, X7 a position illegal or worse",
        "Loiter is flown as ArduCopter BRAKE: LOITER sinks with no pilot on the throttle",
    ]
    for i, s in enumerate(lines):
        f.text(60, sy + 134 + i * 26, s, size=SMALL, fill=SUB, owner=fsm["id"], kind="box")
    f.fact("2.0", "guardrail/fsm.py", r"theta_lateral_m: float = 2\.0")
    f.fact("0.5", "guardrail/fsm.py", r"theta_vertical_m: float = 0\.5")
    f.fact("3", "guardrail/fsm.py", r"n_violations: int = 3")
    f.fact("5", "guardrail/fsm.py", r"window_s: float = 5\.0")
    f.fact("2", "guardrail/fsm.py", r"t_recover_s: float = 2\.0")
    f.fact("30", "guardrail/fsm.py", r"loiter_timeout_s: float \| None = 30\.0")
    f.name_in("guardrail/fsm.py", "N-in-T counts violation ONSETS by default",
              "N-in-T persists (N new onsets since entry)", "operator-defined timeout",
              "RTL fails (battery / blocked path)")
    f.name_in("sitl/mavlink_adapter_node.py", 'DEFAULT_HOLD_MODE = "BRAKE"',
              "LOITER sinks with no pilot on the throttle")

    ex = 40 + fw_ + 40
    f.box(ex, fy, 1560 - ex, fh, "Emitted action + audit",
          ["• the action, back to body frame", "• SET_MODE when the FSM",
           "  changes state", "• an audit record: policy_hash,",
           "  generation, raw and emitted", "  action, every operator tried,",
           "  pass by pass"], style="out", step=8, align="start", valign="top",
          size=B_SIZE, lead=26)
    f.name_in("guardrail/audit.py", "policy_hash", "generation", "raw_action")
    f.name_in("guardrail/frames.py", "def to_body")
    f.name_in("guardrail/repair.py", "`repair_passes` lists these operators")

    # flows into row C
    f.arrow([(xs[0] + w / 2 + 60, yb + h), (xs[0] + w / 2 + 60, fy)], "no violation",
            at=(xs[0] + w / 2 + 74, yb + h + 28), anchor="start")
    f.arrow([(1090, yb + h), (1090, fy)], "repaired action", at=(1076, yb + h + 28),
            anchor="end")
    f.arrow([(40 + fw_, fy + fh / 2), (ex, fy + fh / 2)])
    return f


# ---------------------------------------------------------------------------- #
# 5. The evidence chain
# ---------------------------------------------------------------------------- #

def arch_evidence() -> Fig:
    f = Fig("explainer_arch_evidence", 1600, 760)
    f.header(40, 30, "Where runs come from")
    f.header(476, 30, "What each run writes")
    f.header(1190, 30, "What a reader gets")

    f.box(40, 54, 360, 126, "Flight rails",
          ["the ROS 2 Shield node", "(SITL), the perception", "node (Project AirSim)"],
          style="ext", step=1)
    f.fact("2", "sitl/ros2_shield_node.py", r"ROS 2 node: the Safety Shield")
    f.file("sitl/ros2_shield_node.py", "demo/pas_ardupilot/perception_node.py")
    f.box(40, 212, 360, 110, "gRPC control plane",
          ["10 RPCs + GetStatus,", "mutual TLS"], tag="WP4", step=2,
          valign="top", pad=22)
    f.fact("10", "guardrail/protos/harness.proto", r"^\s*rpc (?!GetStatus)\w+", kind="count")
    f.name_in("guardrail/protos/harness.proto", "rpc GetStatus")
    f.name_in("docs/DESIGN-harness-grpc.md", "the page's ten RPCs in its message names, "
                                             "plus GetStatus")
    f.name_in("guardrail/harness_service.py", "mutual TLS")
    f.box(40, 352, 360, 168, "Stress harness",
          ["headless: scenarios ×", "seeds, smoke, nightly", "and broad profiles;",
           "never KPI-grade"], tag="WP4", step=3, valign="top", pad=22)
    f.arrow([(220, 322), (220, 352)])
    f.file("experiments/sweep_scenarios.py", "guardrail/harness_service.py")
    f.name_in("experiments/sweep_scenarios.py", "--profile smoke", "--profile nightly",
              "--profile broad")
    f.name_in(".github/workflows/nightly.yml", "is_kpi_grade refuses every episode")

    files = [("flight_log.jsonl", "one row per tick"),
             ("audit.jsonl", "a record per changed action"),
             ("events.jsonl", "generations, modes, fences"),
             ("policy_g<N>, csp_g<N>.json", "each rule generation"),
             ("manifest.json", "the six fields below")]
    y = 54
    centres = []
    for name, role in files:
        f.box(476, y, 414, 64, name, [role], style="out", title_family=MONO, tsize=22,
              size=SMALL, lead=22, align="start")
        centres.append(y + 32)
        y += 72
    f.box(476, y + 8, 414, 124, "The six manifest fields",
          ["code_revision, vla_model_hash,", "policy_hash, random_seed,",
           "sim_speedup, topology"], style="ours", line_family=MONO, size=SMALL, lead=26,
          tsize=22, align="start")
    f.name_in("guardrail/manifest.py", "code_revision, vla_model_hash,\n     policy_hash, "
                                       "random_seed, sim_speedup, topology")
    f.name_in("sitl/ros2_shield_node.py", "flight_log.jsonl", "audit.jsonl",
              "events.jsonl", "policy_g<N>.json, csp_g<N>.json", "one row per tick")
    f.name_in("guardrail/audit.py", "one JSONL record per Shield decision that touched the action")
    # sources -> a bus -> every file
    bx = 440
    f.arrow([(400, 117), (bx, 117)], head=False)
    f.arrow([(400, 434), (bx, 434)], head=False)
    f.arrow([(bx, centres[0]), (bx, 434)], head=False)
    for c in centres:
        f.arrow([(bx, c), (476, c)])
    # every file -> a bus -> the replay bundle
    rx = 920
    for c in centres:
        f.arrow([(890, c), (rx, c)], head=False)
    f.arrow([(rx, centres[0]), (rx, centres[-1])], head=False)
    f.arrow([(rx, 253), (950, 253)])

    f.box(950, 158, 200, 190, "Replay bundle",
          ["signed; each", "file digested;", "re-derives its", "own KPIs"],
          style="out", tag="WP4", step=4, tsize=23, size=SMALL, lead=26)
    f.name_in("guardrail/replay.py", "Every member in the archive is digested",
              "**recomputes the KPIs\nfrom the archived log**")

    f.box(1190, 54, 370, 206, "KPI report",
          ["KPI-grade runs apart,", "never pooled", "n/m where nothing ran,",
           "never a silent 0", "a null beside each score", "top-K failures, K = 10"],
          style="out", tag="WP4", step=5, size=SMALL, lead=26)
    f.fact("0", "tools/kpi_report.py", r"`n/m` or `none attempted`, never 0")
    f.fact("10", "tools/kpi_report.py", r"K=10 by default")
    f.name_in("tools/kpi_report.py", "tabulated SEPARATELY and never pooled",
              "The null beside every score")
    f.box(1190, 286, 370, 150, "KPI-grade gate",
          ["sim_speedup 1.0, topology", "hil (dev only by waiver);", "bundle binds the log"],
          style="ours", tag="WP4", size=SMALL, lead=26)
    f.fact("1.0", "guardrail/manifest.py", r"sim_speedup=1\.0 is\s+mandatory")
    f.name_in("guardrail/manifest.py", "def is_kpi_grade", "written PI waiver")
    f.name_in("tools/kpi_report.py", "bundle_binding")
    f.box(1190, 462, 370, 150, "Episode exporters",
          ["step and episode records,", "JSONL; Parquet when", "pyarrow is installed"],
          style="out", tag="WP4", step=6, size=SMALL, lead=26)
    f.name_in("tools/export_episodes.py", "`<out>/steps.jsonl`", "With\n`pyarrow` importable")
    f.file("tools/kpi_report.py", "tools/export_episodes.py")
    # one trunk from the bundle to the three readers
    f.arrow([(1150, 253), (1170, 253)], head=False)
    f.arrow([(1170, 157), (1170, 537)], head=False)
    f.arrow([(1170, 157), (1190, 157)])
    f.arrow([(1170, 361), (1190, 361)])
    f.arrow([(1170, 537), (1190, 537)])

    # CI band
    f.box(40, 648, 1520, 100, "CI on GitHub",
          ["tests.yml, every push: the standalone test files and the smoke profile;",
           "nightly.yml: the nightly profile, headless; it cannot run until it is on main"],
          style="ext", size=SMALL, lead=27, tsize=24)
    f.file(".github/workflows/tests.yml", ".github/workflows/nightly.yml")
    f.name_in(".github/workflows/tests.yml", "The smoke profile of the stress harness",
              "on:\n  push:")
    f.name_in(".github/workflows/nightly.yml", "IT DOES NOT RUN YET")
    f.arrow([(220, 648), (220, 520)], dashed=True)
    return f


FIGURES = (flowchart_end_to_end, arch_components, arch_topologies, arch_shield,
           arch_evidence)


# ---------------------------------------------------------------------------- #
# Checks
# ---------------------------------------------------------------------------- #

def check_facts(fig: Fig) -> list[str]:
    """Every number on the figure has a fact, every fact holds, every file
    exists, every code name is in the file named for it."""
    out = []
    covered = {f.num for f in fig.facts}
    ordinals = []
    for t in fig.texts:
        if t["cls"] == "ordinal":
            ordinals.append(int(t["s"]))
            continue
        for n in numbers_in(t["s"]):
            if n not in covered:
                out.append(f"{fig.name}: {n!r} in {t['s']!r} has no source")
    if ordinals and ordinals != list(range(1, len(ordinals) + 1)):
        out.append(f"{fig.name}: step badges are not 1..n in order: {ordinals}")
    printed = {n for t in fig.texts if t["cls"] != "ordinal" for n in numbers_in(t["s"])}
    for fact in fig.facts:
        if fact.num not in printed:
            out.append(f"{fig.name}: fact {fact.num!r} ({fact.src}) is not printed")
        p = fact.problem()
        if p:
            out.append(f"{fig.name}: {p}")
    for rel in fig.files:
        if not (ROOT / rel).exists():
            out.append(f"{fig.name}: printed path {rel} does not exist")
    cache: dict[str, str] = {}
    for name, src in fig.names:
        if src not in cache:
            p = ROOT / src
            cache[src] = p.read_text(encoding="utf-8", errors="replace") if p.is_file() else ""
        if name not in cache[src]:
            out.append(f"{fig.name}: {name!r} not found in {src}")
    return out


def _seg_hits_rect(p, q, r, inset=3.0) -> bool:
    """Does segment pq pass through the interior of rect r (x, y, w, h) shrunk
    by `inset`? Liang-Barsky clipping."""
    x0, y0 = r[0] + inset, r[1] + inset
    x1, y1 = r[0] + r[2] - inset, r[1] + r[3] - inset
    if x1 <= x0 or y1 <= y0:
        return False
    (px, py), (qx, qy) = p, q
    dx, dy = qx - px, qy - py
    t0, t1 = 0.0, 1.0
    for pp, qq in ((-dx, px - x0), (dx, x1 - px), (-dy, py - y0), (dy, y1 - py)):
        if pp == 0:
            if qq < 0:
                return False
            continue
        t = qq / pp
        if pp < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return t1 - t0 > 1e-9


def size_problems(fig: Fig) -> list[str]:
    """The smallest text, scaled into the deck's figure box, is MIN_PT or more."""
    scale = slide_pt_per_unit(fig.w, fig.h)
    least = min(t["size"] for t in fig.texts)
    if least * scale < MIN_PT - 1e-9:
        return [f"{fig.name}: {least} units of text come out at {least * scale:.2f} pt "
                f"on the slide (minimum {MIN_PT} pt)"]
    return []


def layout_problems(fig: Fig) -> list[str]:
    """Font-free checks: arrows do not run through boxes."""
    out = []
    for path in fig.paths:
        pts = path["pts"]
        for p, q in zip(pts, pts[1:]):
            for b in fig.boxes:
                r = (b["x"], b["y"], b["w"], b["h"])
                if _seg_hits_rect(p, q, r):
                    out.append(f"{fig.name}: arrow {pts[0]}->{pts[-1]} runs through "
                               f"box {b['title']!r}")
    for r in fig.boxes + fig.containers:
        if r["x"] < 0 or r["y"] < 0 or r["x"] + r["w"] > fig.w or r["y"] + r["h"] > fig.h:
            out.append(f"{fig.name}: box {r['title']!r} leaves the canvas")
    return out


def _font_files() -> dict | None:
    dirs = [Path(os.environ.get("LOCALAPPDATA", "")) / "Microsoft/Windows/Fonts",
            Path(os.environ.get("WINDIR", r"C:\Windows")) / "Fonts",
            Path.home() / ".fonts", Path("/usr/share/fonts/truetype/poppins")]
    want = {("sans", 400): "Poppins-Regular.ttf", ("sans", 500): "Poppins-Medium.ttf",
            ("sans", 600): "Poppins-SemiBold.ttf", ("sans", 700): "Poppins-Bold.ttf",
            ("mono", 400): "consola.ttf", ("mono", 700): "consolab.ttf"}
    found = {}
    for key, fn in want.items():
        for d in dirs:
            if (d / fn).is_file():
                found[key] = d / fn
                break
        else:
            return None
    return found


def fit_problems(fig: Fig) -> list[str] | None:
    """Measured checks; None when the fonts (or PIL) are not available."""
    files = _font_files()
    if files is None:
        return None
    try:
        from PIL import ImageFont
    except ImportError:
        return None
    cache = {}

    def font(t):
        fam = "mono" if t["family"] == MONO else "sans"
        wt = t["weight"] if fam == "sans" else (700 if t["weight"] >= 600 else 400)
        key = (fam, wt, t["size"])
        if key not in cache:
            cache[key] = ImageFont.truetype(str(files[(fam, wt)]), t["size"])
        return cache[key]

    boxes = {b["id"]: b for b in fig.boxes + fig.containers}
    rects = []
    for t in fig.texts:
        fnt = font(t)
        l, top, r, bot = fnt.getbbox(t["s"], anchor="ls")
        width = fnt.getlength(t["s"]) + t["spacing"] * max(0, len(t["s"]) - 1)
        x0 = t["x"] - {"start": 0, "middle": width / 2, "end": width}[t["anchor"]]
        rects.append((x0 + l, t["y"] + top, x0 + max(r, width), t["y"] + bot))
    out = []
    for t, (x0, y0, x1, y1) in zip(fig.texts, rects):
        tag = f"{fig.name}: {t['s']!r}"
        if x0 < 2 or y0 < 2 or x1 > fig.w - 2 or y1 > fig.h - 2:
            out.append(f"{tag} leaves the canvas")
        if t["kind"] in ("badge", "ordinal"):
            bx, by, bw, bh = t["rect"]
            if x0 < bx + 3 or x1 > bx + bw - 3:
                out.append(f"{tag} overflows its badge")
            for path in fig.paths:
                if any(_seg_hits_rect(p, q, t["rect"], 0)
                       for p, q in zip(path["pts"], path["pts"][1:])):
                    out.append(f"{tag} badge is crossed by an arrow")
                    break
            continue
        if t["kind"] == "box":
            b = boxes[t["owner"]]
            if "diamond" in b:
                cx, cy, dw, dh = b["diamond"]
                # inside the diamond: |x-cx|/(w/2) + |y-cy|/(h/2) <= 1 at each corner
                for (px, py) in ((x0, y0), (x1, y0), (x0, y1), (x1, y1)):
                    if abs(px - cx) / (dw / 2) + abs(py - cy) / (dh / 2) > 0.98:
                        out.append(f"{tag} overflows its diamond")
                        break
            elif x0 < b["x"] + 8 or x1 > b["x"] + b["w"] - 8 \
                    or y0 < b["y"] + 4 or y1 > b["y"] + b["h"] - 4:
                out.append(f"{tag} overflows its box {b['title']!r} by "
                           f"{max(b['x'] + 8 - x0, x1 - b['x'] - b['w'] + 8, 0):.0f} px")
        for o in fig.texts:
            if o is not t and o["kind"] in ("badge", "ordinal"):
                bx, by, bw, bh = o["rect"]
                if x0 < bx + bw and x1 > bx and y0 < by + bh and y1 > by:
                    out.append(f"{tag} runs under the {o['kind']} {o['s']!r}")
        if t["kind"] in ("label", "header", "free", "legend"):
            for b in fig.boxes:
                if x0 < b["x"] + b["w"] and x1 > b["x"] and y0 < b["y"] + b["h"] \
                        and y1 > b["y"]:
                    out.append(f"{tag} overlaps box {b['title']!r}")
        if t["kind"] in ("label", "header", "free", "legend"):
            for path in fig.paths:
                if path["id"] == t["owner"]:
                    continue
                for p, q in zip(path["pts"], path["pts"][1:]):
                    if _seg_hits_rect(p, q, (x0 - 3, y0 - 3, x1 - x0 + 6, y1 - y0 + 6), 0):
                        out.append(f"{tag} sits on an arrow it does not label")
                        break
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            a, b = rects[i], rects[j]
            if a[0] < b[2] - 1 and a[2] > b[0] + 1 and a[1] < b[3] - 1 and a[3] > b[1] + 1:
                out.append(f"{fig.name}: {fig.texts[i]['s']!r} overlaps "
                           f"{fig.texts[j]['s']!r}")
    return out


# ---------------------------------------------------------------------------- #
# Main
# ---------------------------------------------------------------------------- #

def build_all() -> list[Fig]:
    return [fn() for fn in FIGURES]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", action="store_true", help="also render PNGs with Edge")
    ap.add_argument("--out", default=str(OUT), help="output folder (default docs/img)")
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    figs = build_all()
    problems: list[str] = []
    measured = True
    for fig in figs:
        problems += check_facts(fig) + layout_problems(fig) + size_problems(fig)
        fp = fit_problems(fig)
        if fp is None:
            measured = False
        else:
            problems += fp
    for fig in figs:
        p = out / f"{fig.name}.svg"
        fig.save(p)
        print("wrote", p)
    if not measured:
        print("SKIP fit check: Poppins / Consolas or PIL not available here")
    if problems:
        print(f"{len(problems)} problem(s):")
        for p in problems:
            print("  -", p)
        return 1
    if a.png:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from build_flowcharts import to_png
        for fig in figs:
            p = out / f"{fig.name}.svg"
            to_png(p, p.with_suffix(".png"), fig.w, fig.h)
            print("wrote", p.with_suffix(".png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
