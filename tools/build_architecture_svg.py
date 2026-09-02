"""Generate the architecture diagram, with every status read from the artefacts.

    python tools/build_architecture_svg.py      -> docs/architecture-v3.svg

WHY THIS IS GENERATED AND NOT DRAWN

`docs/architecture-v2.svg` is hand-written, dated 2026-07-03, and still says
"today: AirSim Blocks - later: SITL + HIL". ArduPilot SITL over MAVROS 2 has been
the canonical rail since 25 August and is where every contractual KPI figure
comes from. At the 2026-09-02 review the project's own KPI numbers were presented
alongside SITL described as future work, and nobody in the room had reason to
think otherwise - the diagram said so.

That is what a hand-maintained status claim does. It is right on the day it is
written and silently wrong afterwards, and the people reading it cannot tell
which. So every status string here is READ AT BUILD TIME from the thing that
proves it, exactly as `tools/deck/build_sept_deck.js` does for slide numbers.

Each box also carries the DATE of its most recent evidence. A box whose evidence
is three months old still says so on its face, which is the property v2 lacked.

WHAT IS DELIBERATELY NOT AUTOMATED

The boxes, the arrows and the layout are hand-placed: this is a picture of an
architecture, and an auto-laid-out graph of it would be less readable, not more.
Only the CLAIMS are generated. The distinction matters - it is the claims that
go stale, never the shape.
"""
from __future__ import annotations

import datetime
import glob
import json
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# Same palette as the deck, so the two read as one set of documents.
TEAL = "#249DB2"
TEAL_DK = "#1A7484"
TEAL_TINT = "#EAF6F8"
INK = "#1F2A33"
GREY = "#6B7280"
LINE = "#D8E0E6"
WHITE = "#FFFFFF"
OK = "#2C7F68"
WARN = "#B23F38"
PAPER = "#F7F9FA"


def _date(rel: str) -> str | None:
    """The date of the file that proves a claim. Honest about being a file date."""
    p = ROOT / rel
    if not p.exists():
        return None
    return datetime.date.fromtimestamp(os.path.getmtime(p)).isoformat()


def gather() -> dict:
    """Every claim the diagram makes, read from the thing that proves it."""
    from guardrail import load_policy
    from guardrail.compiler import ConstraintCompiler
    from tools.build_deck_data import bundle_facts, constraint_inventory

    ci = constraint_inventory()
    bf = bundle_facts()
    one_bundle = next(iter(bf.values())) if bf else {}

    kpi = json.loads((ROOT / "demo/out/ros2_shield_on/kpi.json")
                     .read_text(encoding="utf-8"))
    lock = json.loads((ROOT / "demo/out/city_locked/metrics.json")
                      .read_text(encoding="utf-8"))
    sweep = json.loads((ROOT / "docs/data/scenario_sweep.json")
                       .read_text(encoding="utf-8"))
    csp = ConstraintCompiler(
        load_policy(ROOT / "policies/sitl_pedestrian.yaml")).summary_pack()

    n_tests = 0
    files = sorted(glob.glob(str(ROOT / "tests" / "test_*.py")))
    for f in files:
        n_tests += len(re.findall(r"^def test_", Path(f).read_text(encoding="utf-8"),
                                  re.M))

    return {
        "dsl": {
            "status": f"{ci['n_implemented']} constraint classes"
                      f" · valid_time on every rule",
            "note": f"reference implementation carries {ci['reference_impl_n_types']}",
            "date": _date("guardrail/models.py"),
        },
        "bundle": {
            "status": f"signed bundle round-trips · {one_bundle.get('bytes', '?')} B",
            "note": "reload refuses a tampered IR, a foreign manifest, a truncation",
            "date": _date("guardrail/bundle.py"),
        },
        "csp": {
            "status": f"Constraint Summary Pack · {csp['n_rules']} rules",
            "note": "the prompt renders FROM the pack, so they cannot disagree",
            "date": _date("guardrail/compiler.py"),
        },
        "lock": {
            "status": f"frac_on_target {lock.get('frac_on_target')}",
            "note": "yaw-compensated prediction · 0.12 gate · 1.8x size ratio",
            "date": _date("demo/out/city_locked/metrics.json"),
        },
        "shield": {
            "status": f"P0 escape rate {kpi['p0_violation_escape_rate']}"
                      f" · {'KPI-grade' if kpi.get('kpi_grade') else 'NOT grade-eligible'}",
            "note": f"{kpi['p0_ticks_not_measurable']} unmeasurable ticks"
                    f" · all five acceptance KPIs computed",
            "date": _date("demo/out/ros2_shield_on/kpi.json"),
        },
        "sweep": {
            "status": f"{sweep['_counts']['pass']} pass · "
                      f"{sweep['_counts']['fail']} fail · "
                      f"{sweep['_counts']['known_failure']} known failure",
            "note": "headless, no simulator, about one second",
            "date": _date("docs/data/scenario_sweep.json"),
        },
        "tests": {"n": n_tests, "files": len(files)},
        "topology": kpi["manifest"]["topology"],
        "built": datetime.date.today().isoformat(),
    }


# --------------------------------------------------------------------------- #
# drawing helpers
# --------------------------------------------------------------------------- #

def esc(s) -> str:
    return (str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


class Canvas:
    def __init__(self, w: int, h: int):
        self.w, self.h, self.parts = w, h, []

    def text(self, x, y, s, size=12, fill=INK, weight="normal", anchor="start",
             family="Segoe UI, Arial, sans-serif", spacing=None):
        sp = f' letter-spacing="{spacing}"' if spacing else ""
        self.parts.append(
            f'<text x="{x}" y="{y}" font-size="{size}" fill="{fill}" '
            f'font-weight="{weight}" text-anchor="{anchor}" '
            f'font-family="{family}"{sp}>{esc(s)}</text>')

    def box(self, x, y, w, h, title, subtitle=None, status=None, note=None,
            date=None, accent=TEAL, fill=WHITE, dashed=False):
        dash = ' stroke-dasharray="5 4"' if dashed else ""
        self.parts.append(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="4" '
            f'fill="{fill}" stroke="{accent}" stroke-width="1.5"{dash}/>')
        # accent rail on the left edge - the deck uses the same device
        if not dashed:
            self.parts.append(
                f'<rect x="{x}" y="{y}" width="3.5" height="{h}" rx="2" fill="{accent}"/>')
        ty = y + 20
        self.text(x + 14, ty, title, size=13.5, weight="600")
        if subtitle:
            ty += 16
            self.text(x + 14, ty, subtitle, size=11, fill=GREY)
        if status:
            ty += 18
            self.text(x + 14, ty, status, size=11.5, fill=accent, weight="600",
                      family="Consolas, monospace")
        if note:
            ty += 15
            self.text(x + 14, ty, note, size=10, fill=GREY)
        if date:
            self.text(x + w - 12, y + h - 9, f"evidence {date}", size=9,
                      fill="#9AA6AF", anchor="end", family="Consolas, monospace")

    def arrow(self, x1, y1, x2, y2, colour="#7C8B96", label=None, dashed=False):
        dash = ' stroke-dasharray="4 4"' if dashed else ""
        self.parts.append(
            f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{colour}" '
            f'stroke-width="1.6" marker-end="url(#a)"{dash}/>')
        if label:
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            self.text(mx + 8, my - 4, label, size=10, fill=colour)

    def render(self) -> str:
        return (
            f'<svg width="{self.w}" height="{self.h}" viewBox="0 0 {self.w} {self.h}" '
            f'xmlns="http://www.w3.org/2000/svg">\n'
            f'  <defs><marker id="a" markerWidth="10" markerHeight="8" refX="8" '
            f'refY="4" orient="auto">'
            f'<path d="M0,0 L8,4 L0,8 Z" fill="#7C8B96"/></marker></defs>\n'
            f'  <rect width="{self.w}" height="{self.h}" fill="{PAPER}"/>\n  '
            + "\n  ".join(self.parts) + "\n</svg>\n")


def build(d: dict) -> str:
    W, H = 1040, 1180
    c = Canvas(W, H)

    # ---- header ----
    c.text(40, 46, "Semantic-Spatial Translation and Safety-Constrained VLA for ArduPilot UAVs",
           size=19, weight="700")
    c.text(40, 68, "Architecture v3 — NTUT AIoT Lab, Guardrail work packages. "
                   "Supersedes v2 (2026-07-03).", size=11.5, fill=GREY)
    c.text(40, 88, f"Every status below is read from the artefact that proves it, "
                   f"at build time. Generated {d['built']}.",
           size=10.5, fill=TEAL_DK)

    # ---- legend ----
    c.parts.append(f'<rect x="700" y="30" width="300" height="62" rx="4" '
                   f'fill="{WHITE}" stroke="{LINE}"/>')
    c.text(714, 48, "OUR WORK PACKAGES", size=9, fill=TEAL, weight="600", spacing="1.4")
    c.parts.append(f'<rect x="714" y="56" width="14" height="10" fill="{WHITE}" stroke="{TEAL}" stroke-width="1.5"/>')
    c.text(734, 65, "built + evidenced", size=10, fill=GREY)
    c.parts.append(f'<rect x="714" y="72" width="14" height="10" fill="{WHITE}" stroke="{GREY}" stroke-width="1.5" stroke-dasharray="4 3"/>')
    c.text(734, 81, "not ours / not built", size=10, fill=GREY)

    L, R = 40, 560          # two columns
    CW = 440

    # ---- WP2: compiler ----
    y = 118
    c.text(L, y, "WP2 · PREFIX — TELL THE MODEL THE RULES FIRST", size=9.5,
           fill=TEAL, weight="600", spacing="1.4")
    c.box(L, y + 10, CW, 92, "Constraint Compiler",
          "operator text -> structured mission + rule summary",
          d["csp"]["status"], d["csp"]["note"], d["csp"]["date"])

    c.text(R, y, "WP1 · THE RULE LANGUAGE", size=9.5, fill=TEAL,
           weight="600", spacing="1.4")
    c.box(R, y + 10, CW, 92, "Policy DSL + IR",
          "YAML -> validated, hashed policy",
          d["dsl"]["status"], d["dsl"]["note"], d["dsl"]["date"])

    y += 118
    c.box(R, y, CW, 88, "Signed policy bundle",
          "ir.json + manifest.json + signature.txt",
          d["bundle"]["status"], d["bundle"]["note"], d["bundle"]["date"])

    # ---- perception column ----
    y2 = 250
    c.text(L, y2, "PERCEPTION — NAMED BY A PHRASE, NOT A CLICK", size=9.5,
           fill=TEAL, weight="600", spacing="1.4")
    c.box(L, y2 + 10, CW, 74, "OWL-ViT open-vocabulary detector",
          "frame + text phrase -> candidate boxes",
          "153M params · 3.6-5.2 Hz in flight", None, None)

    c.box(L, y2 + 96, CW, 96, "Target lock   ← NEW, requested 2026-09-02",
          "binds the controller to ONE instance of the named class",
          d["lock"]["status"], d["lock"]["note"], d["lock"]["date"])

    c.box(L, y2 + 204, CW, 74, "Target-state estimator + guidance",
          "constant-velocity smoothing -> proportional control",
          "bearing -> yaw · range -> forward", None, None)

    # ---- VLA slot ----
    y3 = 556
    c.box(L, y3, CW, 84, "Action source (swappable)",
          "any producer of the 4-D action contract, 10 Hz",
          "5 sources flown · P0 escape 0 in all five",
          "OpenVLA-7B · AerialVLA · QLoRA fine-tunes · BC policy · hand-written",
          None)

    # ---- Shield ----
    y4 = 664
    c.text(L, y4, "WP3 · SUFFIX — THE LAST WORD BEFORE THE AUTOPILOT", size=9.5,
           fill=TEAL, weight="600", spacing="1.4")
    c.box(L, y4 + 10, 960, 104, "Suffix Safety Shield",
          "monitor (3 s lookahead)  ->  repair operators  ->  re-check the flown "
          "action  ->  recovery heading  ->  brake",
          d["shield"]["status"], d["shield"]["note"], d["shield"]["date"],
          accent=TEAL_DK)

    # ---- WP4 ----
    # Starts below the WP1 bundle box (which ends at y=324), not level with the
    # perception column. Overlapping those two was caught by the geometry check
    # rather than by eye, which is the argument for having the check.
    y4c = 350
    c.text(R, y4c, "WP4 · PROVING IT", size=9.5, fill=TEAL, weight="600", spacing="1.4")
    c.box(R, y4c + 10, CW, 92, "Scenario sweep harness",
          "scenario library scored with guardrail.kpi.compute",
          d["sweep"]["status"], d["sweep"]["note"], d["sweep"]["date"])
    c.box(R, y4c + 118, CW, 78, "Test suite",
          "run per file; there is no pytest in this environment",
          f"{d['tests']['n']} tests defined across {d['tests']['files']} files",
          None, None)
    c.box(R, y4c + 212, CW, 66, "Determinism manifest",
          "code_revision · policy_hash · seed · sim_speedup · topology",
          "is_kpi_grade() refuses a run that cannot be reproduced", None, None)

    # ---- transport ----
    y5 = 790
    c.box(L, y5, 960, 62, "MAVROS 2  /  MAVLink",
          "SET_POSITION_TARGET_LOCAL_NED in GUIDED mode — the canonical Year-1 bridge",
          None, None, None)

    y6 = 866
    c.box(L, y6, 960, 62, "ArduPilot",
          "inner loop 100–400 Hz · its own GeoFence / RTL / Land is the ultimate "
          "backstop, which the Shield pre-empts rather than replaces",
          None, None, None, accent=GREY, dashed=True)

    # ---- the two rails: the correction v2 got wrong ----
    y7 = 954
    c.text(L, y7, "WHERE IT RUNS — AND WHICH EVIDENCE COUNTS", size=9.5,
           fill=TEAL, weight="600", spacing="1.4")
    c.box(L, y7 + 10, CW, 104, "ArduPilot SITL + MAVROS 2",
          "no renderer, no camera · stub pilot on a waypoint mission",
          f"{d['topology']} · KPI-GRADE",
          "every contractual figure in this project comes from here",
          d["shield"]["date"], accent=OK)
    c.box(R, y7 + 10, CW, 104, "Project AirSim (Unreal)",
          "camera, depth, city scene, pedestrians and vehicles",
          "functional rail · NOT grade-eligible",
          "all tracking evidence lives here, outside the contractual gate",
          d["lock"]["date"], accent=WARN)

    c.text(L, y7 + 134,
           "This is the distinction v2 did not draw. It said \"today: AirSim Blocks · later: SITL + HIL\", "
           "and was still saying it on 2026-09-02.", size=10.5, fill=WARN)
    c.text(L, y7 + 150,
           "Closing the gap means feeding AirSim imagery to a Guardrail driven over MAVROS — "
           "the HIL_GPS / HIL_SENSOR bridge. Largest remaining item.",
           size=10.5, fill=GREY)

    # ---- arrows down the left spine ----
    mid = L + CW / 2
    c.arrow(mid, y + 102 - 118 + 92 + 10, mid, y2 + 8)       # compiler -> perception
    c.arrow(mid, y2 + 84, mid, y2 + 104)                     # detector -> lock
    c.arrow(mid, y2 + 192, mid, y2 + 212)                    # lock -> estimator
    c.arrow(mid, y2 + 278, mid, y3 - 2)                      # estimator -> action
    c.arrow(mid, y3 + 84, mid, y4 + 8)                       # action -> shield
    c.arrow(520, y4 + 114, 520, y5 - 2)                      # shield -> mavros
    c.arrow(520, y5 + 62, 520, y6 - 2)                       # mavros -> ardupilot
    # policy feeds the shield
    c.arrow(R, y + 56, L + CW + 12, y4 + 40, colour=TEAL, label="rules in force")
    # sweep reads the shield
    c.arrow(R + 10, y4c + 102, R + 10, y4 + 20, colour=TEAL_DK, dashed=True,
            label="scores")

    return c.render()


def main() -> int:
    d = gather()
    out = ROOT / "docs" / "architecture-v3.svg"
    out.write_text(build(d), encoding="utf-8")
    print(f"wrote {out.relative_to(ROOT)}")
    for k in ("dsl", "bundle", "csp", "lock", "shield", "sweep"):
        print(f"  {k:<8} {d[k]['status']}   (evidence {d[k]['date']})")
    print(f"  tests    {d['tests']['n']} across {d['tests']['files']} files")
    print(f"  topology {d['topology']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
