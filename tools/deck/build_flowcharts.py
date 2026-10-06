"""Draw the two pipeline flowcharts as SVG (and PNG, through Edge headless).

    python tools/deck/build_flowcharts.py            # SVG only
    python tools/deck/build_flowcharts.py --png      # also docs/img/*.png

Writes docs/img/flowchart_tracking.svg and docs/img/flowchart_system.svg.

The 30 Sept deck carried these as screenshots of a chat widget, so they could
not be corrected. They had to be: the identity tiers were labelled HARD / SOFT
there, and at the meeting that was heard as manoeuvre types, while the grant's
Policy DSL already uses hard/soft for POLICY rules (constraint_type). The tiers
are REJECT / DOUBTFUL / OK in everything a reader sees from 2026-10-03 on; the
values stored in flight logs stay "hard" / "soft" / "ok" (demo/identity.py,
TIER_LABEL). Drawn from code here so the next correction is a text edit.
"""
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs" / "img"

BG = "#1b1b1b"
GREY = ("#3d3d3d", "#6a6a6a", "#f0f0f0", "#c8c8c8")
GREEN = ("#0f4a3a", "#1f8a6a", "#f0f0f0", "#7fe0bf")
PURPLE = ("#3a2d8f", "#6d5ce0", "#f0f0f0", "#cfc6ff")
TEAL = ("#0e4b57", "#249db2", "#f0f0f0", "#9fe3ef")
FONT = "Segoe UI, Arial, sans-serif"


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Svg:
    def __init__(self, w: int, h: int):
        self.w, self.h = w, h
        self.parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
            f'viewBox="0 0 {w} {h}" font-family="{FONT}">',
            '<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
            'markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" '
            'fill="#bdbdbd"/></marker></defs>',
            f'<rect width="{w}" height="{h}" fill="{BG}"/>']

    def box(self, x, y, w, h, title, sub, pal):
        fill, stroke, tcol, scol = pal
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="4" '
                          f'fill="{fill}" stroke="{stroke}" stroke-width="1.5"/>')
        cx = x + w / 2
        self.parts.append(f'<text x="{cx}" y="{y + h / 2 - 3}" text-anchor="middle" '
                          f'font-size="17" font-weight="600" fill="{tcol}">{esc(title)}</text>')
        self.parts.append(f'<text x="{cx}" y="{y + h / 2 + 19}" text-anchor="middle" '
                          f'font-size="14" fill="{scol}">{esc(sub)}</text>')
        return (x, y, w, h)

    def line(self, pts, arrow=True, dash=False):
        d = " ".join(f"{'M' if i == 0 else 'L'}{x},{y}" for i, (x, y) in enumerate(pts))
        self.parts.append(f'<path d="{d}" fill="none" stroke="#bdbdbd" stroke-width="1.6"'
                          + (' stroke-dasharray="6,5"' if dash else '')
                          + (' marker-end="url(#a)"' if arrow else '') + '/>')

    def label(self, x, y, s, anchor="middle", size=14, col="#d8d8d8"):
        self.parts.append(f'<text x="{x}" y="{y}" text-anchor="{anchor}" font-size="{size}" '
                          f'fill="{col}">{esc(s)}</text>')

    def legend(self, x, y, items):
        for name, pal in items:
            self.parts.append(f'<rect x="{x}" y="{y - 13}" width="16" height="16" rx="3" '
                              f'fill="{pal[0]}" stroke="{pal[1]}"/>')
            self.label(x + 26, y, name, anchor="start", size=15)
            x += 26 + 9 * len(name) + 40

    def save(self, path: Path):
        path.write_text("\n".join(self.parts + ["</svg>"]) + "\n", encoding="utf-8")


def tracking() -> Svg:
    s = Svg(790, 840)
    cx, W = 210, 420
    s.box(cx, 20, W, 68, "Camera frame", "pose and depth at capture time", GREY)
    s.line([(cx + W / 2, 88), (cx + W / 2, 124)])
    s.box(cx, 126, W, 68, "OWL-ViT detector", 'prompt "a red car", up to 12 boxes', GREY)
    s.line([(cx + W / 2, 194), (cx + W / 2, 230)])
    s.box(cx, 232, W, 68, "1. Physical identity", "width, bottom height, street -> tier", GREEN)
    s.line([(cx + W, 266), (642, 266)])
    s.box(644, 232, 132, 68, "REJECT", "never steers", GREY)
    s.line([(cx + W / 2, 300), (cx + W / 2, 336)])
    s.box(cx, 338, W, 68, "2. Strict lock", "OK, or DOUBTFUL near the prediction?", GREEN)
    # yes -> estimator (left), no -> lapse (right)
    s.line([(cx + 110, 406), (cx + 110, 440), (180, 440), (180, 474)])
    s.label(cx + 140, 434, "yes")
    s.line([(cx + 310, 406), (cx + 310, 440), (590, 440), (590, 474)])
    s.label(cx + 362, 434, "no, for 3 s")
    s.box(20, 476, 340, 68, "Estimator update", "car position at capture time", GREY)
    s.line([(190, 544), (190, 580)])
    s.box(20, 582, 340, 68, "Controller + Safety Shield", "follow at stand-off", GREY)
    s.box(420, 476, 340, 68, "3. Lapse", "estimate dropped, search starts", GREEN)
    s.line([(590, 544), (590, 580)])
    s.box(420, 582, 340, 68, "4. Reacquirer", "4 OK in a row, <= 45 m, in reach", GREEN)
    s.line([(420, 616), (390, 616), (390, 510), (362, 510)])
    s.line([(560, 650), (560, 686)])
    s.label(500, 674, "too far")
    s.box(420, 688, 340, 68, "5. Far lead", "fly closer, the gate decides", GREEN)
    s.line([(660, 688), (660, 652)])
    s.legend(24, 800, [("new (29-30 Sept)", GREEN), ("existing / unchanged", GREY)])
    return s


def system() -> Svg:
    s = Svg(820, 935)
    s.box(20, 20, 250, 70, "Policy DSL (YAML)", "fence, altitude, stand-off", PURPLE)
    s.line([(145, 90), (145, 124)])
    s.box(20, 126, 250, 70, "Policy IR + bundle", "hashed and versioned", PURPLE)
    s.line([(145, 196), (145, 230)])
    s.box(20, 232, 250, 70, "Constraint Summary Pack", "prompt prefix for the VLA", PURPLE)

    s.box(550, 20, 250, 70, "Camera + depth", "pose at capture time", GREEN)
    s.line([(675, 90), (675, 124)])
    s.box(550, 126, 250, 70, "OWL-ViT detector", "phrase -> candidate boxes", GREEN)
    s.line([(675, 196), (675, 230)])
    s.box(550, 232, 250, 70, "Identity + strict lock", "drop REJECT, lock one car", GREEN)
    s.line([(675, 302), (675, 336)])
    s.box(550, 338, 250, 70, "Estimator + Reacquirer", "lapse 3 s, re-seed on proof", GREEN)

    s.box(185, 450, 450, 70, "Pilot (swappable)", "OpenVLA-7B or follow controller", GREY)
    s.line([(230, 302), (230, 448)])
    s.label(262, 380, "prefix", anchor="start")
    s.line([(585, 408), (585, 448)])
    s.label(596, 432, "target", anchor="start")
    s.line([(410, 520), (410, 554)])
    s.box(185, 556, 450, 70, "Action4D command", "vx, vy, vz, yaw rate at 10 Hz", GREY)
    s.line([(410, 626), (410, 660)])
    s.box(185, 662, 450, 70, "Safety Shield", "predict, check, repair, brake", PURPLE)
    # rules from the IR, subject from the estimator
    s.line([(120, 302), (120, 697), (183, 697)])
    s.label(80, 520, "rules")
    s.line([(760, 408), (760, 697), (637, 697)])
    s.label(785, 520, "subject")
    # outputs
    s.line([(260, 732), (260, 760), (120, 760), (120, 788)])
    s.box(20, 790, 200, 70, "KPI + audit log", "replay bundle", GREY)
    s.line([(410, 732), (410, 788)])
    s.box(240, 790, 340, 70, "ArduPilot via MAVROS 2", "SITL (KPI rail) or Project AirSim", GREY)
    s.line([(560, 732), (560, 760), (700, 760), (700, 788)])
    s.box(600, 790, 200, 70, "Policy HUD (3 Oct)", "rule status, Shield banner", TEAL)
    s.line([(580, 825), (590, 825), (590, 880), (810, 880), (810, 55), (802, 55)], dash=True)
    s.legend(24, 920, [("guardrail (contract core)", PURPLE), ("perception", GREEN),
                           ("pilot and autopilot", GREY)])
    return s


def to_png(svg: Path, png: Path, w: int, h: int) -> None:
    edge = next((p for p in (Path(r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"),
                             Path(r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"))
                 if p.exists()), None)
    if edge is None:
        raise SystemExit("Edge not found; open the SVG in a browser instead")
    # Old --headless, the SVG itself as the page, 2x device scale. The
    # --headless=new + HTML-wrapper form exited 0 and wrote nothing, so the
    # file is checked rather than the exit code trusted.
    # A private profile, or the launcher hands the job to a running Edge and
    # returns before anything is written.
    import tempfile
    png.unlink(missing_ok=True)
    prof = tempfile.mkdtemp(prefix="edge_flowchart_")
    # ...and the shot goes to that temp folder first: given a path under
    # "VLA Drone" (a space, on OneDrive) Edge exited 0 and wrote nothing.
    shot = Path(prof) / "shot.png"
    subprocess.run([str(edge), "--headless", "--disable-gpu", "--hide-scrollbars",
                    f"--user-data-dir={prof}",
                    "--force-device-scale-factor=2", f"--window-size={w},{h}",
                    f"--screenshot={shot}", svg.resolve().as_uri()],
                   check=True, timeout=60, capture_output=True)
    # The launcher returns at once and a child process writes the file, so
    # wait for it rather than trusting the return.
    import time
    for _ in range(100):
        if shot.is_file() and shot.stat().st_size > 0:
            time.sleep(0.5)
            break
        time.sleep(0.2)
    if not shot.is_file() or shot.stat().st_size == 0:
        raise SystemExit(f"Edge wrote no PNG for {svg.name}")
    png.write_bytes(shot.read_bytes())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", action="store_true")
    a = ap.parse_args()
    for name, fn in (("flowchart_tracking", tracking), ("flowchart_system", system)):
        svg = fn()
        p = OUT / f"{name}.svg"
        svg.save(p)
        print("wrote", p)
        if a.png:
            to_png(p, p.with_suffix(".png"), svg.w, svg.h)
            print("wrote", p.with_suffix(".png"))


if __name__ == "__main__":
    main()
