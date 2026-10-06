"""Charts for the 2026-09-30 progress deck, read from the flight artefacts.

    C:/Users/natha/.conda/envs/pas/python.exe tools/deck/progress_0930_charts.py

Writes docs/img/progress_0930_*.png. Every value is read at build time:
the new flights from demo/out/<tag>/metrics.json, the old ones from the
pipeline replay's as-flown arm (demo/out/identity_replay/pipeline.json) and
their own metrics.json. Nothing is typed in. Colours and font follow
tools/deck/nathan_theme.js (Poppins, teal #249DB2). Each chart is drawn at the
size the deck places it (8.9 x 2.72 in on slide 8, 8.9 x 3.3 in on slide 19),
so its 11 pt labels stay 11 pt on the slide.
"""
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt                                   # noqa: E402
from matplotlib import font_manager                              # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "demo" / "out"
IMG = ROOT / "docs" / "img"

TEAL, TEAL_DK, TEAL_TINT2 = "#249DB2", "#1A7484", "#D6EDF1"
GREY, LGREY, INK, LINE = "#6B7280", "#9AA3AD", "#2B2B2B", "#E3E8EC"

OLD = ["citylife_redcar_trail", "citylife_redcar_trail2", "citylife_redcar_trail3",
       "citylife_redcar_final1", "citylife_redcar_final2", "citylife_redcar_final3",
       "citylife_redcar_final4"]
NEW = ["citylife_redcar_id1", "citylife_redcar_id2", "citylife_redcar_id3",
       "citylife_redcar_id4"]


def _font():
    for d in (Path.home() / "AppData/Local/Microsoft/Windows/Fonts", Path("C:/Windows/Fonts")):
        for f in d.glob("Poppins-*.ttf"):
            font_manager.fontManager.addfont(str(f))
    if any("Poppins" in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = "Poppins"


def metrics(tag):
    return json.loads((OUT / tag / "metrics.json").read_text(encoding="utf-8"))


def label(tag):
    return tag.replace("citylife_redcar_", "")


def data():
    pipe = json.loads((OUT / "identity_replay" / "pipeline.json").read_text(encoding="utf-8"))
    rows = []
    for tag in OLD:
        af = pipe[tag]["old_as_flown"]
        rows.append({"tag": tag, "group": "before", "w30": metrics(tag)["frac_within_30m"],
                     "est": af["on_frac_of_served"]})
    for tag in NEW:
        m = metrics(tag)
        rows.append({"tag": tag, "group": "after", "w30": m["frac_within_30m"],
                     "est": m["estimate_on_subject"]["on_subject_frac"]})
    return rows


FS = 11          # tick labels, bar values, group labels: true points at the placed size


def style(ax):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(LINE)
    ax.tick_params(colors=GREY, labelsize=FS)
    ax.yaxis.grid(True, color=LINE, linewidth=0.8)
    ax.set_axisbelow(True)


def bars(rows, key, title, before_label, fname, size):
    """`size` is the (w, h) in inches the deck places the chart at, so a point
    here is a point on the slide (tools/deck/build_progress_0930_deck.js)."""
    fig, ax = plt.subplots(figsize=size, dpi=220)
    xs = list(range(len(rows)))
    cols = [LGREY if r["group"] == "before" else TEAL for r in rows]
    vals = [100.0 * r[key] for r in rows]
    ax.bar(xs, vals, color=cols, width=0.68, edgecolor="none")
    for x, v in zip(xs, vals):
        txt = f"{v:.1f}" if (v < 1.95 or 99.0 < v < 99.95) else f"{v:.0f}"
        ax.text(x, v + 1.5, txt, ha="center", va="bottom",
                fontsize=FS, color=INK, fontweight="bold")
    ax.set_xticks(xs, [label(r["tag"]) for r in rows])
    ax.set_ylim(0, 126)
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.set_ylabel("% of ticks", color=GREY, fontsize=FS)
    n_old = sum(1 for r in rows if r["group"] == "before")
    n_new = len(rows) - n_old
    ax.axvline(n_old - 0.5, color=LINE, linewidth=1.2)
    ax.text((n_old - 1) / 2, 117, before_label.format(n=n_old), ha="center",
            color=GREY, fontsize=FS)
    ax.text(n_old + (n_new - 1) / 2, 117, f"after: --identity, flown ({n_new} flights)",
            ha="center", color=TEAL_DK, fontsize=FS, fontweight="bold")
    ax.set_title(title, loc="left", color=INK, fontsize=13, fontweight="bold")
    style(ax)
    fig.tight_layout(pad=0.4)
    IMG.mkdir(parents=True, exist_ok=True)
    fig.savefig(IMG / fname, facecolor="white")
    plt.close(fig)
    return IMG / fname


def main():
    _font()
    rows = data()
    # The "before" estimate is the 1d09786 estimator REPLAYED on each flight's
    # boxes (pipeline.json old_as_flown); it reproduces the flown estimate only
    # on final1..4 (demo/out/identity_replay/pipeline.md). Within 30 m is flown.
    out = [bars(rows, "est", "Estimate on the red car (within 6 m), share of served ticks",
                "before: old estimator, replayed ({n} flights)",
                "progress_0930_estimate_on_car.png", (8.9, 2.72)),
           bars(rows, "w30", "Drone within 30 m of the red car, share of mission ticks",
                "before: old selection, flown ({n} flights)",
                "progress_0930_within_30m.png", (8.9, 3.3))]
    for r in rows:
        print(f"{r['tag']:26s} {r['group']:7s} w30 {r['w30']:.3f} est {r['est']:.3f}")
    for p in out:
        print("wrote", p.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
