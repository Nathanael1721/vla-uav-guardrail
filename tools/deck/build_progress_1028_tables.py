"""Table images of every 28 Oct flight for the progress deck: a one-slide
summary (pilot x scenario, each run's score) and the full list of all flights
on two slides. Numbers from docs/data/progress_1028.json
(tools/deck/build_progress_1028_data.py), nathan-deck colours, Poppins.

  C:/Users/natha/.conda/envs/vla-real/python.exe tools/deck/build_progress_1028_tables.py

Writes docs/share/2026-10-28-Progress/tables/{summary,all_flights_1,all_flights_2}.png
(2400 x 1350, 16:9 - paste full-slide or scale down).
"""
from __future__ import annotations

import json
import statistics
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle

REPO = Path(__file__).resolve().parents[2]
D = json.loads((REPO / "docs" / "data" / "progress_1028.json").read_text(encoding="utf-8"))
OUT = REPO / "docs" / "share" / "2026-10-28-Progress" / "tables"
FONTS = Path.home() / "AppData" / "Local" / "Microsoft" / "Windows" / "Fonts"
for f in FONTS.glob("Poppins-*.ttf"):
    font_manager.fontManager.addfont(str(f))
plt.rcParams["font.family"] = "Poppins" if list(FONTS.glob("Poppins-*.ttf")) else "Segoe UI"

TEAL, TEAL_DK, TINT, TINT2 = "#249DB2", "#1A7484", "#EAF6F8", "#D6EDF1"
INK, GREY, LGREY, LINE, ORANGE = "#2B2B2B", "#6B7280", "#9AA3AD", "#E3E8EC", "#E08A1E"
NAMES = {"aerovla": "AerialVLA (original)", "r1": "AerialVLA FT r1", "r2": "AerialVLA FT r2",
         "r3": "AerialVLA FT r3", "r4": "AerialVLA FT r4", "dronecats": "DroneCATS-2B",
         "racevla": "RaceVLA-7B", "manual": "Our controller", "fixed12": "Our controller"}
MODE = {"pure": "Pure VLA", "hier": "Hierarchical", "manual": "No VLA", "fixed12": "No VLA, fixed 12 m"}
f3 = lambda v: "-" if v is None else f"{v:.3f}"


def canvas(title: str, subtitle: str):
    fig = plt.figure(figsize=(16, 9), dpi=150)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 16); ax.set_ylim(9, 0); ax.axis("off")
    fig.patch.set_facecolor("white")
    ax.text(0.6, 0.62, title, fontsize=24, fontweight="bold", color="#1A1A1A", va="center")
    ax.plot([0.6, 1.5], [1.02, 1.02], color=TEAL, lw=3)
    ax.text(0.6, 1.32, subtitle, fontsize=11.5, color=GREY, va="center")
    return fig, ax


def grid(ax, x0, y0, widths, rows, rh, header_rows=1, aligns=None, bold_cells=(), color_cells=None,
         fontsize=11.5, sep_after=()):
    """Draw a table: teal header, TINT zebra, hairlines. rows[i][j] is a str
    or a (str, small_str) pair drawn as two lines."""
    color_cells = color_cells or {}
    y = y0
    for i, row in enumerate(rows):
        head = i < header_rows
        x = x0
        zebra = (not head) and (i - header_rows) % 2 == 1
        for j, (w, cell) in enumerate(zip(widths, row)):
            ax.add_patch(Rectangle((x, y), w, rh, facecolor=TEAL if head else (TINT if zebra else "white"),
                                   edgecolor=LINE, lw=0.8))
            al = (aligns[j] if aligns else ("left" if j == 0 else "center"))
            tx = x + 0.15 if al == "left" else x + w / 2
            col = "white" if head else color_cells.get((i, j), INK)
            bold = head or (i, j) in bold_cells
            if isinstance(cell, tuple):
                ax.text(tx, y + rh * 0.38, cell[0], ha=al, va="center", fontsize=fontsize, color=col,
                        fontweight="bold" if bold else "normal")
                ax.text(tx, y + rh * 0.74, cell[1], ha=al, va="center", fontsize=fontsize - 3, color=GREY)
            else:
                ax.text(tx, y + rh / 2, cell, ha=al, va="center", fontsize=fontsize, color=col,
                        fontweight="bold" if bold else "normal")
            x += w
        y += rh
        if i in sep_after:
            ax.plot([x0, x0 + sum(widths)], [y, y], color=TEAL, lw=2)
    return y


def summary():
    rows_by = {}
    for r in D["matrix_rows"]:
        rows_by.setdefault((r["kind"], r["model"], r["platform"]), []).append(r["score"])
    cols = [("pure", "PC"), ("hier", "PC"), ("pure", "Jetson"), ("hier", "Jetson")]

    def cell(kind, model, plat, kinds=None):
        v = []
        for k in (kinds or [kind]):
            v += rows_by.get((k, model, plat), [])
        if not v:
            return "-"
        runs = " / ".join(f3(x) for x in v)
        return (f"{statistics.mean(v):.3f}", f"runs {runs}") if len(v) > 1 else f3(v[0])

    order = ["aerovla", "r1", "r2", "r3", "r4", "dronecats", "racevla"]
    body = [["Our controller (no VLA)", "-", cell("manual", "manual", "PC"), "-", cell("manual", "manual", "Jetson")],
            ["Our controller, fixed 12 m (no VLA)", "-", cell("fixed12", "fixed12", "PC"), "-", cell("fixed12", "fixed12", "Jetson")]]
    body += [[NAMES[m]] + [cell(k, m, p) for k, p in cols] for m in order]
    N = D["nfz"]
    body += [["DroneCATS-2B + 3 no-fly zones", "-", f3(N["PC"]["within30"]), "-", "-"],
             ["AerialVLA (original) + 3 no-fly zones", "-", "-", "-", f3(N["Jetson"]["within30"])],
             ["First zone try (DroneCATS-2B)", "-", f3(N["first_attempt"]["within30"]), "-", "-"]]
    head = [["Pilot", "PC · Pure VLA", "PC · Hierarchical", "Jetson · Pure VLA", "Jetson · Hierarchical"]]
    rows = head + body
    # best VLA per column (rows 3..9 = VLAs), by mean
    bold, colr = set(), {}
    for j in range(1, 5):
        best, bi = -1, None
        for i in range(3, 10):
            c = rows[i][j]
            v = float(c[0] if isinstance(c, tuple) else c) if c != "-" else -1
            if v > best:
                best, bi = v, i
        if bi:
            bold.add((bi, j)); colr[(bi, j)] = TEAL
    for i in (1, 2):
        for j in (2, 4):
            bold.add((i, j))
    fig, ax = canvas("All Flights at a Glance",
                     f"{D['safety']['flights']} flights, 27-28 Oct 2026 · score = share of time within 30 m of the car "
                     f"(1.000 = always) · several runs: average, runs below")
    widths = [4.6, 2.6, 2.6, 2.6, 2.6]
    rh = [0.52] + [(8.2 - 1.75 - 0.52) / len(body)] * len(body)
    y = 1.75
    for i, row in enumerate(rows):
        y = grid(ax, 0.6, y, widths, [row], rh[i], header_rows=1 if i == 0 else 0,
                 bold_cells={(0, j) for (ii, j) in bold if ii == i},
                 color_cells={(0, j): c for (ii, j), c in colr.items() if ii == i},
                 fontsize=11.5) if i == 0 else _row(ax, 0.6, y, widths, row, rh[i], i, bold, colr)
        if i in (2, 9):
            ax.plot([0.6, 0.6 + sum(widths)], [y, y], color=TEAL, lw=2)
    s = D["safety"]
    ax.text(0.6, 8.5, f"All {s['flights']} flights: {s['mission_collisions']} collisions in a mission · P0 escape rate "
            f"{' / '.join(s['p0_escape_rates'])} in every flight · {s['p0_violation_ticks']} rule-breaking commands caught by the guardrail.",
            fontsize=11, color=TEAL_DK, fontweight="bold")
    ax.text(0.6, 8.85, "Teal = best VLA in the column (by average). Pure VLA scored from the first lock on the car; "
            "the others from the start gate's lock. On the Jetson the VLA model answers from the PC's GPU.",
            fontsize=9.5, color=LGREY)
    fig.savefig(OUT / "summary.png", dpi=150)
    plt.close(fig)


def _row(ax, x0, y, widths, row, rh, i, bold, colr):
    zebra = i % 2 == 0
    x = x0
    for j, (w, c) in enumerate(zip(widths, row)):
        ax.add_patch(Rectangle((x, y), w, rh, facecolor=TINT if zebra else "white", edgecolor=LINE, lw=0.8))
        al = "left" if j == 0 else "center"
        tx = x + 0.15 if al == "left" else x + w / 2
        b = (i, j) in bold
        col = colr.get((i, j), INK)
        if isinstance(c, tuple):
            ax.text(tx, y + rh * 0.34, c[0], ha=al, va="center", fontsize=11, color=col, fontweight="bold" if b else "normal")
            ax.text(tx, y + rh * 0.76, c[1], ha=al, va="center", fontsize=7.5, color=GREY)
        else:
            ax.text(tx, y + rh / 2, c, ha=al, va="center", fontsize=11 if j else 10.5, color=col if c != "-" else LGREY,
                    fontweight="bold" if b else "normal")
        x += w
    return y + rh


def all_flights():
    order_kind = {"manual": 0, "fixed12": 1, "pure": 2, "hier": 3}
    order_model = {m: i for i, m in enumerate(["manual", "fixed12", "aerovla", "r1", "r2", "r3", "r4", "dronecats", "racevla"])}
    rows = []
    for r in D["matrix_rows"]:
        date = "27 Oct" if r["tag"].endswith("20261027") else "28 Oct"
        note = "1 zone"
        if r["tag"] == "cmp_hier_fixed12_20261027":
            note = "1 zone · 12 m test, flew as controller"
        rows.append((r["platform"], date, order_kind[r["kind"]], order_model.get(r["model"], 9), r["tag"],
                     [r["platform"], date, MODE[r["kind"]], NAMES.get(r["model"], r["model"]), note,
                      f3(r["score"]), str(r["collisions_mission"]), str(r["p0_escape_rate"])]))
    N = D["nfz"]
    for key, plat, note in (("first_attempt", "PC", "3 zones, first try (held at zone edge)"),
                            ("PC", "PC", "3 zones (guardrail test)"), ("Jetson", "Jetson", "3 zones (guardrail test)")):
        e = N[key]
        model = "dronecats" if key != "Jetson" else "aerovla"
        rows.append((plat, "28 Oct", 4, order_model[model], e["tag"],
                     [plat, "28 Oct", "Hierarchical", NAMES[model], note, f3(e["within30"]),
                      str(e["collisions_mission"]), str(e["p0_escape_rate"])]))
    rows.sort(key=lambda t: (t[0] != "PC", t[2], t[3], t[1]))
    head = ["#", "Computer", "Date", "Mode", "Pilot", "No-fly zones", "Score", "Collisions", "P0 escape"]
    numbered = [[str(i + 1)] + r[5] for i, r in enumerate(rows)]
    half = (len(numbered) + 1) // 2
    widths = [0.55, 1.2, 1.05, 2.15, 2.5, 3.65, 1.1, 1.2, 1.2]
    for part, chunk in ((1, numbered[:half]), (2, numbered[half:])):
        fig, ax = canvas(f"All {len(numbered)} Flights ({part}/2)",
                         "Every flight of the comparison and the guardrail test · score = share of time within 30 m of the car")
        y = grid(ax, 0.6, 1.7, widths, [head] + chunk, 0.29, fontsize=10,
                 aligns=["center", "left", "left", "left", "left", "left", "center", "center", "center"])
        ax.text(0.6, y + 0.32, "Pure VLA scored from the first lock on the car; the others from the start gate's lock. "
                "On the Jetson the VLA model answers from the PC's GPU.", fontsize=9.5, color=LGREY)
        fig.savefig(OUT / f"all_flights_{part}.png", dpi=150)
        plt.close(fig)
    return len(numbered)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    summary()
    n = all_flights()
    print(f"wrote {OUT}: summary.png, all_flights_1.png, all_flights_2.png ({n} flights)")
