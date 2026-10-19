"""
Fine-tuning campaign report — loss curve + before/after comparison charts.

Reads:
    training/ft_loss_log.csv          (from finetune_aerialvla.py)
    training/eval_<name>.json         (from eval_aerialvla.py, >=2 of them)

Writes:
    docs/aerialvla-ft-report.md       tables + findings
    docs/img/ft_loss_curve.png
    docs/img/ft_comparison.png

Run:  python training/ft_report.py --evals baseline ft_e0 ft_e1 [--date 2026-07-17]

The committed report is reproduced, byte for byte, by
    python training/ft_report.py --evals baseline ft2_yg04 base_deploy \
        ft2_deploy ft3_deploy --date 2026-07-17
(tests/test_ft_report.py checks it): a rebuild keeps the existing report's
dated '>' note under the title when the date line is unchanged, and keeps
the file's line endings (write_report). `--no-note` drops the note.

EACH ROW SAYS HOW IT FLEW (2026-10-19). Until then the report's header and
note said every row was "pure VLA", goal assist off (`--goal-blend 0`), and
the chart's title said "no guidance assist". The July campaign
(training/ft_campaign_stage2.ps1) flew base_deploy, ft2_deploy and ft3_deploy
with `--goal-blend 0.55`: 55 % of a velocity toward the target mixed into the
VLA's horizontal action (demo/aerialvla_demo.py). Those are the three 100 %
rows; the pure rows reached 0-1 of 5 (CHANGELOG 2026-10-07, Retracted). The
assist was never missing from the data - every episode records its
`params.goal_blend` and `params.yaw_gain` - only from the text, which was
hard-coded. render_markdown() now reads it per row and never defaults to
"pure": an eval whose episodes carry no params reads "unknown".

REVIEW OF F8-HARNESS-TOOLS (2026-10-19). Two gaps the first fix left:
render_markdown() did not write the report's dated correction note, so the
regeneration the note itself describes dropped it; and the chart hatched
every row that was not pure VLA - a "mixed" or "unknown" row too - under a
title that says hatching means "goal assist on". Now only an assisted row is
hatched (//); mixed (xx) and unknown (..) rows have marks of their own, and
the title names each mark that is on the chart (bar_hatch).
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IMG = ROOT / "docs" / "img"
UNKNOWN = "unknown"


def _param(e: dict, key: str):
    """The one value of results[*].params[key] across an eval's episodes:
    a number, "mixed" when episodes differ, UNKNOWN when any lacks it."""
    vals = set()
    for r in e.get("results") or []:
        p = r.get("params") if isinstance(r, dict) else None
        if not isinstance(p, dict) or p.get(key) is None:
            return UNKNOWN
        vals.add(float(p[key]))
    if not vals:
        return UNKNOWN
    return vals.pop() if len(vals) == 1 else "mixed"


def flown_as(e: dict) -> str:
    """"pure VLA" (goal_blend 0 on every episode), "assisted" (goal_blend
    above 0 on every episode), "mixed" or UNKNOWN. Never "pure" by default."""
    gb = _param(e, "goal_blend")
    if gb in (UNKNOWN, "mixed"):
        return gb
    return "assisted" if gb > 0 else "pure VLA"


def _fmt(v) -> str:
    return v if isinstance(v, str) else f"{v:g}"


# Each way a row can have flown -> its bars' hatch, and what the chart title
# says the mark means. Only an assisted row is hatched "//": a row whose
# episodes differ, or that never recorded goal_blend, is not known to have
# flown with the assist and must not wear its mark.
HATCH = {"pure VLA": "", "assisted": "//", "mixed": "xx", UNKNOWN: ".."}
HATCH_MEANING = {"assisted": "hatched bars: goal assist on, goal_blend > 0",
                 "mixed": "cross-hatched bars: goal_blend differs between episodes",
                 UNKNOWN: "dotted bars: goal_blend not recorded"}


def bar_hatch(e: dict) -> str:
    """The hatch of an eval's bars (HATCH)."""
    return HATCH[flown_as(e)]


def comparison_title(evals: list[dict]) -> str:
    kinds = {flown_as(e) for e in evals}
    if kinds == {"pure VLA"}:
        how = "every row pure VLA (goal_blend 0)"
    else:
        how = "; ".join(HATCH_MEANING[k] for k in ("assisted", "mixed", UNKNOWN)
                        if k in kinds) or "no rows"
    return f"AerialVLA fine-tuning — closed-loop eval ({how})"


def title_line(report_date: str) -> str:
    return f"# AerialVLA Fine-Tuning Report — {report_date}"


def leading_note(text: str) -> str | None:
    """The '>' block right under a report's title line and its blank line
    (the dated correction note), unquoted; None when there is none."""
    lines = text.replace("\r\n", "\n").split("\n")
    note = []
    for ln in lines[2:]:
        if not ln.startswith(">"):
            break
        note.append(ln[2:] if ln.startswith("> ") else ln[1:])
    return "\n".join(note) if note else None


def render_markdown(evals: list[dict], report_date: str, has_loss: bool,
                    note: str | None = None) -> str:
    """The report's text, from the evals alone (no files written). `note`,
    when given, is quoted ('> ') under the title: the dated correction note
    (leading_note reads it back)."""
    md = [title_line(report_date), ""]
    if note:
        md += [f"> {ln}" if ln else ">" for ln in note.split("\n")] + [""]
    md += ["Base: openvla-7b (4-bit NF4) + AerialVLA LoRA (r=64), fine-tuned with "
           "QLoRA on self-collected AirSimNH expert flights "
           "(front+down mosaic, semantic-direction prompt, 99-bin action text).", ""]
    md += ["## Closed-loop evaluation (how each row flew is in its own columns)", ""]
    md += ["| adapter | episodes | goal_blend | yaw gain | flown as | reached | "
           "mean efficiency | mean interventions | NFZ s |",
           "|---|---|---|---|---|---|---|---|---|"]
    for e in evals:
        md.append(f"| {e['name']} | {e['episodes']} | {_fmt(_param(e, 'goal_blend'))} | "
                  f"{_fmt(_param(e, 'yaw_gain'))} | {flown_as(e)} | "
                  f"{e['reached_rate']:.0%} | {e['mean_efficiency']} | "
                  f"{e['mean_interventions']} | {e['nfz_total_s']} |")
    md += ["", "![comparison](img/ft_comparison.png)", ""]
    if has_loss:
        md += ["## Training loss", "", "![loss](img/ft_loss_curve.png)", ""]
    pure = [e["name"] for e in evals if flown_as(e) == "pure VLA"]
    assisted = [e["name"] for e in evals if flown_as(e) == "assisted"]
    other = [e["name"] for e in evals if flown_as(e) not in ("pure VLA", "assisted")]
    md += ["## Notes",
           "- `goal_blend` is the mission-direction assist of `demo/aerialvla_demo.py`:",
           "  it mixes that share of a velocity toward the target into the VLA's",
           "  horizontal action. It and the yaw gain are read from every episode's",
           "  recorded `params`. A row with goal_blend above 0 is ASSISTED: its score",
           "  is not the model's alone."]
    if pure:
        md.append(f"- Flown as pure VLA (goal_blend 0): {', '.join(pure)}.")
    if assisted:
        md.append(f"- Flown with the goal assist: {', '.join(assisted)}. Compare a reached "
                  f"rate only with rows flown the same way.")
    if other:
        md.append(f"- Not recorded, or not the same on every episode: {', '.join(other)}.")
    md += ["- The NFZ column is the time inside the no-fly zone; keeping the aircraft",
           "  out of it is the guardrail's job in every configuration."]
    return "\n".join(md)


def plot_comparison(plt, evals: list[dict], path: Path) -> None:
    """Three bar panels; an assisted row's bar is hatched, a mixed or unknown
    row's carries its own mark (bar_hatch), and the title says what each mark
    means (never "no guidance assist" over assisted bars)."""
    names = [e["name"] for e in evals]
    hatch = [bar_hatch(e) for e in evals]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
    for ax, vals, title, fmt in [
        (axes[0], [e["reached_rate"] * 100 for e in evals], "Target reached (%)", "{:.0f}%"),
        (axes[1], [e["mean_efficiency"] for e in evals],
         "Path efficiency (straight/actual)", "{:.3f}"),
        (axes[2], [e["mean_interventions"] for e in evals],
         "Shield interventions (mean)", "{:.0f}"),
    ]:
        colors = ["#9AA3AD"] + ["#249DB2"] * (len(vals) - 1)
        bars = ax.bar(names, vals, color=colors)
        for b, v, h in zip(bars, vals, hatch):
            b.set_hatch(h)
            ax.text(b.get_x() + b.get_width() / 2, b.get_height(),
                    fmt.format(v), ha="center", va="bottom", fontsize=10)
        ax.set_title(title)
        ax.grid(alpha=0.3, axis="y")
    fig.suptitle(comparison_title(evals))
    fig.tight_layout()
    fig.savefig(path, dpi=130)


def write_report(out: Path, evals: list[dict], report_date: str, has_loss: bool,
                 keep_note: bool = True) -> tuple[str | None, str | None]:
    """Write the report to `out`; (the note carried over, a note left out).

    The existing report's dated '>' note (leading_note) is carried over when
    its title line is the one being written: the note is about THAT report,
    and render_markdown() alone does not know it, so until the review of
    F8-harness-tools a rebuild with the July evals dropped the correction
    the note records. A note under another date is left out and returned as
    such. The file's line endings are kept (the committed report is CRLF;
    Path.write_text would write the platform's)."""
    old_text, newline = None, None
    if out.exists():
        raw = out.read_bytes()
        newline = "\r\n" if b"\r\n" in raw else "\n"
        old_text = raw.decode("utf-8").replace("\r\n", "\n")
    old_note = leading_note(old_text) if old_text is not None else None
    same_report = (old_text is not None
                   and old_text.split("\n", 1)[0] == title_line(report_date))
    note = old_note if keep_note and same_report else None
    with open(out, "w", encoding="utf-8", newline=newline) as fh:
        fh.write(render_markdown(evals, report_date, has_loss, note=note))
    return note, (old_note if old_note and note is None else None)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--evals", nargs="+", required=True,
                    help="eval names in order, baseline first")
    ap.add_argument("--date", default=str(date.today()),
                    help="the report's date line (default today); pass the "
                         "campaign's date when rebuilding an older report")
    ap.add_argument("--no-note", action="store_true",
                    help="drop the existing report's dated '>' note (kept by "
                         "default when the date line is unchanged)")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    IMG.mkdir(parents=True, exist_ok=True)

    # ---- loss curve ----
    steps, losses = [], []
    csv_path = ROOT / "training" / "ft_loss_log.csv"
    if csv_path.exists():
        with csv_path.open(encoding="utf-8") as f:
            for row in csv.DictReader(f):
                steps.append(int(row["step"]))
                losses.append(float(row["loss"]))
    if steps:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(steps, losses, color="tab:blue")
        ax.set_xlabel("optimizer step")
        ax.set_ylabel("action-token loss")
        ax.set_title("AerialVLA QLoRA fine-tuning — training loss")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(IMG / "ft_loss_curve.png", dpi=130)
        print(f"[report] {IMG / 'ft_loss_curve.png'}")

    # ---- comparison ----
    evals = []
    for name in args.evals:
        p = ROOT / "training" / f"eval_{name}.json"
        evals.append(json.loads(p.read_text(encoding="utf-8")))

    plot_comparison(plt, evals, IMG / "ft_comparison.png")
    print(f"[report] {IMG / 'ft_comparison.png'}")

    # ---- markdown ----
    out = ROOT / "docs" / "aerialvla-ft-report.md"
    kept, dropped = write_report(out, evals, args.date, bool(steps),
                                 keep_note=not args.no_note)
    print(f"[report] {out}")
    if kept:
        print("[report] kept the existing report's dated note under the title")
    if dropped:
        print("[report] WARNING left out the existing report's dated note "
              + ("(--no-note)" if args.no_note else
                 f"(it is about that report's date, not {args.date})"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
