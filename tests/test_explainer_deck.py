"""The explainer deck (tools/deck/build_explainer_deck.js): it builds, it has the
expected slides, every number on a slide is a fact checked against its file, and
no slide hedges, overflows, or states a result that does not exist.

    python tests/test_explainer_deck.py

The deck is built once into a temporary folder with --no-office (no PowerPoint,
no font embedding) and read back from the .pptx itself: what is checked is what
a reader gets. Needs Node.js and tools/deck/node_modules; without them the build
tests SKIP visibly. The text-fit test also needs Pillow and the Poppins fonts.
The pack in docs/share/2026-10-14-Explainer/ is gitignored; its test SKIPs
where the pack is absent.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import zipfile
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILDER = ROOT / "tools" / "deck" / "build_explainer_deck.js"
PACK = ROOT / "docs" / "share" / "2026-10-14-Explainer"
DECK = "Guardrail-Explainer-13Oct2026.pptx"
SKIP = "SKIP"
# Slides without an optional image: cover, what it is, terms, the VLA, five
# figures, escalation, the section slide, close approach, G1, harness, Shield
# speed, interfaces, two KPI tables, next steps, closing.
BASE_SLIDES = 20
# Words per cheatsheet: a cheap guard against growth that runs everywhere.
# The proof of "one page" is test_the_cheatsheets_print_on_one_a4_page, which
# prints both sheets with the recipe below (SHEET_CSS) where pandoc and Edge
# are installed. With that recipe on 14 Oct, both sheets at 637 words print on
# one page each; the same sheets with equal-width table columns
# ("|---|---|---|---|") took two, which is why the builder writes the number
# table's separator in proportion to its columns.
CHEATSHEET_WORDS = 640
# The print recipe: pandoc's HTML fragment of the sheet (-f markdown, so a
# wrapping pipe table is sized by its separator's dashes) inside this CSS,
# printed by Edge headless to an A4 PDF without header or footer.
SHEET_CSS = """
@page { size: A4; margin: 12mm 14mm; }
html, body { margin: 0; padding: 0; background: #fff; color: #111; }
body { font-family: "Segoe UI", Arial, sans-serif; font-size: 10pt; line-height: 1.3; }
h1 { font-size: 14pt; margin: 0 0 4pt; }
h2 { font-size: 11.5pt; margin: 7pt 0 3pt; }
p { margin: 3pt 0; }
ul { margin: 3pt 0; padding-left: 16pt; }
li { margin: 1pt 0; }
table { border-collapse: collapse; width: 100%; font-size: 9pt; margin: 3pt 0; }
th, td { border: 0.5pt solid #bbb; padding: 1.5pt 4pt; vertical-align: top; text-align: left; }
code { font-family: Consolas, monospace; font-size: 8.5pt; }
"""
EDGE_PATHS = (Path(r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"),
              Path(r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"))

sys.path.insert(0, str(ROOT / "tools"))

# --------------------------------------------------------------------------- #
# Wording rules
# --------------------------------------------------------------------------- #

# Hedges and filler (the user's style rule: assertive, direct, no apology).
HEDGES = [r"we believe", r"i believe", r"we think", r"seems?\b", r"seemingly", r"somewhat",
          r"hopefully", r"we hope", r"perhaps", r"maybe", r"might\b", r"arguably", r"probably",
          r"fairly", r"quite", r"rather", r"kind of", r"sort of", r"a bit\b", r"basically",
          r"essentially", r"it should be noted", r"it is important to note", r"in order to",
          r"unfortunately", r"sorry", r"apologi", r"regrettabl", r"we tried", r"try to",
          r"attempt to", r"very\b", r"really", r"actually", r"just\b", r"mungkin", r"sepertinya"]
HEDGE_RX = re.compile(r"(?<![a-z])(" + "|".join(HEDGES) + r")", re.I)

# Results that do not exist on 14 Oct 2026: no KPI-grade run, no hil or Orin
# run, no drone flight, no VLA flight with the CSP, no tracking result on the
# ArduPilot rail, and the city clip's zone was avoided by the follow
# controller, not the Shield.
BANNED_CLAIMS = [
    r"\bKPI[- ]grade (results?|numbers?|figures?|values?|runs?) (show|shows|showed|prove|proves|met|meets|pass|passed)",
    r"\b(measured|flown|ran|tested|profiled|benchmarked|validated) on (the |a )?(Jetson )?Orin\b",
    r"\bOrin (results?|numbers?|measurements?|profile) (show|shows|showed|confirm)",
    r"\bhil (runs?|campaign|flights?) (show|shows|showed|measured|passed|produced|confirmed)",
    r"\breal[- ]drone (flights?|tests?|results?)\b",
    r"\b(flew|flown|tested) (on|with) (the|a|our) (real )?(drone|airframe)\b",
    r"\b(OpenVLA|the VLA|a VLA) (flew|flies|has flown) (with|reading|using) the (CSP|rules|rule summary)",
    r"\bShield (avoided|steered around|routed around|kept [^.]{0,20}out of) the (no-fly )?zone",
    r"\bcontract(ual)? KPI (results?|numbers?|figures?|values?)\b",
    r"\b(meets|met|meeting) (the|all|every) (grant's )?(acceptance )?KPIs?\b",
    r"\bfail-safe[^.]{0,40}\b(meets|met|achieved|reached|passes) (the |its )?(target|0\.99)",
    r"\bGazebo\b",
    # The close approach: the Shield FLEW no P0-breaking command (0 of the 367
    # ticks it flew); the aircraft still stood inside the building margin on 78
    # ticks under ArduPilot's BRAKE hold and RTL. Neither "stopped every P0
    # violation" nor a 0 over all 532 ticks is true.
    r"\bstopped (every|all) P0 violations?\b",
    r"\b0 of (the )?532\b",
    r"\bP0 ticks flown\b",
    r"\bone known-failure case\b",
    # Nothing has run on an Orin: no present or past tense for it.
    r"\bOrin profile (repeats|repeated|confirms|confirmed|shows|showed|matches|matched)\b",
    r"\b(runs|ran|has run|have run|repeats|repeated|was run|were run) unchanged on the (Jetson )?Orin\b",
    r"\b(is|are|was|were) repeated on the (Jetson )?Orin\b",
    r"\bscripts? (runs?|ran) unchanged on the (Jetson )?Orin\b",
    r"\bdijalankan tanpa perubahan di Orin\b",
    r"\breal flight ticks\b",
    # Mission Planner itself was never attached; a listener stood on its port.
    r"\bMission Planner (shows|showed|sees|saw|displays|displayed|receives|received|talks)\b",
    # BRAKE held altitude; its stopping drift moved the aircraft.
    r"\bBRAKE holds position\b",
    r"\bmenahan posisi dan ketinggian\b",
    # Two acceptance KPIs are below target (fail-safe, library P0 escape).
    r"\bthe (one|only one|only KPI) below (its|the) target\b",
    r"\bsatu-satunya yang di bawah target\b",
    # On the ArduPilot (perception) rail the detector boxed a red traffic
    # signal, not the car (CHANGELOG 2026-10-14): no tracking result exists there.
    r"\b(followed|tracked|kept up with) the (red )?car on the (ArduPilot|perception) rail\b",
    r"\bthe (ArduPilot|perception) rail (followed|tracked) the (red )?car\b",
]

# --------------------------------------------------------------------------- #
# Shared build
# --------------------------------------------------------------------------- #

_BUILD: dict = {}


def _node_ready() -> str | None:
    """None when Node.js and the deck's node_modules are present, else why not."""
    if shutil.which("node") is None:
        return "node is not on PATH"
    if not (ROOT / "tools" / "deck" / "node_modules" / "pptxgenjs").is_dir():
        return "tools/deck/node_modules is not installed (npm install in tools/deck)"
    return None


def _run_builder(out: Path, *extra: str, env: dict | None = None) -> subprocess.CompletedProcess:
    cmd = ["node", str(BUILDER), "--out", str(out), "--no-office", *extra]
    e = dict(os.environ)
    e.setdefault("EXPLAINER_PY", sys.executable)
    if env:
        e.update(env)
    return subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, encoding="utf-8",
                          errors="replace", timeout=600, env=e)


def _build() -> dict | str:
    """Build once (into a temp dir); SKIP when Node.js is not usable here."""
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    if _BUILD:
        return _BUILD
    why = _node_ready()
    if why:
        print(f"      ({why})")
        return SKIP
    out = Path(tempfile.mkdtemp(prefix="explainer_deck_"))
    r = _run_builder(out)
    _BUILD.update(out=out, proc=r)
    return _BUILD


def _slides(pptx: Path) -> list[dict]:
    """[{n, text, notes}] read from the package XML, in slide order."""
    with zipfile.ZipFile(pptx) as z:
        names = z.namelist()
        nums = sorted(int(m.group(1)) for n in names if (m := re.match(r"ppt/slides/slide(\d+)\.xml$", n)))
        out = []
        for n in nums:
            xml = z.read(f"ppt/slides/slide{n}.xml").decode("utf-8")
            notes = ""
            rels = f"ppt/slides/_rels/slide{n}.xml.rels"
            if rels in names:
                m = re.search(r'Target="\.\./notesSlides/(notesSlide\d+\.xml)"', z.read(rels).decode("utf-8"))
                if m:
                    notes = _xml_text(z.read(f"ppt/notesSlides/{m.group(1)}").decode("utf-8"))
            out.append({"n": n, "text": _xml_text(xml), "notes": notes})
        return out


def _xml_text(xml: str) -> str:
    paras = re.findall(r"<a:p>(.*?)</a:p>", xml, flags=re.S)
    lines = []
    for p in paras:
        runs = re.findall(r"<a:t>([^<]*)</a:t>", p)
        t = "".join(runs)
        t = (t.replace("&amp;", "&").replace("&lt;", "<").replace("&gt;", ">")
             .replace("&quot;", '"').replace("&apos;", "'").replace("\u00a0", " ")
             .replace("\u2011", "-"))           # the deck's non-breaking hyphen in ranges
        if t.strip():
            lines.append(t)
    return "\n".join(lines)


def _png(path: Path, w: int = 64, h: int = 36) -> None:
    """A plain teal PNG, written without Pillow."""
    raw = b"".join(b"\x00" + b"\x24\x9d\xb2" * w for _ in range(h))
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def _built() -> tuple[Path, list[dict], dict] | str:
    b = _build()
    if b == SKIP:
        return SKIP
    r = b["proc"]
    assert r.returncode == 0, f"builder exit {r.returncode}:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}"
    pptx = b["out"] / DECK
    stamp = json.loads((b["out"] / "_build" / "build_stamp.json").read_text(encoding="utf-8"))
    return pptx, _slides(pptx), stamp


# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #

def test_the_builder_runs_and_writes_a_deck():
    got = _built()
    if got == SKIP:
        return SKIP
    pptx, slides, stamp = got
    assert pptx.is_file() and pptx.stat().st_size > 100_000, "no deck written"
    assert stamp["deck"] == DECK and stamp["slides"] == len(slides), stamp
    for f in ("slides_text.json", "layout.json", "facts.json"):
        assert (pptx.parent / "_build" / f).is_file(), f"_build/{f} missing"


def test_the_deck_has_the_expected_slide_count():
    got = _built()
    if got == SKIP:
        return SKIP
    _, slides, stamp = got
    opt = stamp["optional"]
    want = BASE_SLIDES + int(bool(opt["still"])) + int(bool(opt["onoff"]))
    assert len(slides) == want, f"{len(slides)} slides, expected {want} (optional: {opt})"


def test_a_missing_shield_on_off_image_drops_only_that_slide():
    """The default build already carries the on/off slide where the pack has its
    stills, so the counts are taken from BASE_SLIDES and the still flag, not
    from the default build's own count."""
    b = _build()
    if b == SKIP:
        return SKIP
    base = _built()[2]
    with_still = BASE_SLIDES + int(bool(base["optional"]["still"]))
    tmp = Path(tempfile.mkdtemp(prefix="explainer_onoff_"))
    on, off = tmp / "on.png", tmp / "off.png"
    _png(on), _png(off)
    both = _run_builder(tmp / "both", "--onoff-on", str(on), "--onoff-off", str(off))
    assert both.returncode == 0, both.stderr[-1500:]
    one = _run_builder(tmp / "one", "--onoff-on", str(on), "--onoff-off", str(tmp / "absent.png"))
    assert one.returncode == 0, one.stderr[-1500:]
    n_both = len(_slides(tmp / "both" / DECK))
    n_one = len(_slides(tmp / "one" / DECK))
    assert n_both == with_still + 1, (n_both, with_still)
    assert n_one == with_still, (n_one, with_still)
    titles = [s["text"] for s in _slides(tmp / "both" / DECK)]
    assert any("the Shield corrects, the control arm breaks P0" in t for t in titles)
    assert not any("the Shield corrects, the control arm breaks P0" in s["text"] for s in _slides(tmp / "one" / DECK))


def test_every_slide_has_speaker_notes():
    got = _built()
    if got == SKIP:
        return SKIP
    empty = [s["n"] for s in got[1] if len(s["notes"].strip()) < 40]
    assert not empty, f"slides without speaker notes: {empty}"


def test_no_hedging_or_filler_on_the_slides():
    got = _built()
    if got == SKIP:
        return SKIP
    hits = [(s["n"], part, m.group(0)) for s in got[1] for part in ("text", "notes")
            for m in HEDGE_RX.finditer(s[part])]
    assert not hits, f"hedges: {hits}"


def test_no_withdrawn_claim_on_the_slides():
    got = _built()
    if got == SKIP:
        return SKIP
    import check_claims as cc
    hits = [(s["n"], part, h.claim, h.text) for s in got[1] for part in ("text", "notes")
            for h in cc.scan_text(s[part], "slide.md")]
    assert not hits, f"withdrawn claims (tools/check_claims.py): {hits}"


def test_no_claim_of_a_result_that_does_not_exist():
    got = _built()
    if got == SKIP:
        return SKIP
    hits = [(s["n"], m.group(0)) for s in got[1] for part in ("text", "notes")
            for rx in BANNED_CLAIMS for m in re.finditer(rx, s[part], flags=re.I)]
    assert not hits, f"claims of results that do not exist: {hits}"


# Names that carry digits and are not numbers; dates and plan windows.
NAME_RX = re.compile(r"\b(?:MAVROS|ROS) 2\b|\bJetPack \d+(?:\.\d+)*|\bCopter-[\d.]+|\bPython 3\.\d+\+?|"
                     r"\bUbuntu \d+\.\d+|\b\d-D\b|\bSHA-256\b|\bEd25519\b|OpenVLA-7B|\b4-bit\b")
DATE_RX = re.compile(r"\b\d{1,2}(?:-\d{1,2})?\s+(?:Oct|Nov|October|November|Sept|September)\b(?:\s*-\s*\d{1,2}\s+(?:Oct|Nov))?(?:\s+2026)?"
                     r"|\b2026-\d\d-\d\d\b|\b9-13\b|\bt = \d+(?:\.\d+)? s\b", re.I)
NUM_RX = re.compile(r"(?<![A-Za-z_\d.])\d[\d,]*(?:\.\d+)?(?![A-Za-z_\d])")
# Small structural numbers: step labels and counts written into the slide's own
# wording (eleven terms, three things), and the axis of the rate figure.
STRUCTURAL = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0}


def _numbers(text: str) -> set[float]:
    t = DATE_RX.sub(" ", NAME_RX.sub(" ", text.replace("\u00a0", " ")))
    t = re.sub(r"(\d)-(\d)", r"\1 \2", t)             # a range is two numbers
    out = set()
    for m in NUM_RX.finditer(t):
        try:
            out.add(float(m.group(0).replace(",", "")))
        except ValueError:
            pass
    return out


def test_every_number_on_a_slide_comes_from_a_checked_fact():
    got = _built()
    if got == SKIP:
        return SKIP
    pptx, slides, _ = got
    facts = json.loads((pptx.parent / "_build" / "facts.json").read_text(encoding="utf-8"))
    allowed = set(STRUCTURAL)
    for f in facts:
        for s in (f["value"], f.get("phrase") or ""):
            allowed |= _numbers(s)
    stray = []
    for s in slides:
        # The page badge is the slide's own number.
        text = "\n".join(ln for ln in s["text"].splitlines() if ln.strip() != f"{s['n']:02d}")
        stray += [(s["n"], v) for v in sorted(_numbers(text) - allowed)]
    assert not stray, f"numbers on slides that no checked fact carries: {stray}"


def test_the_number_check_reads_numbers_and_skips_names_and_dates():
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    assert _numbers("Status as of 13 October 2026 · WP3 · ROS 2 · 4-D · OpenVLA-7B") == set()
    assert _numbers("0.86-1.96 s per decision; 1,301 ticks; P0 at 99.1 %") == {0.86, 1.96, 1301.0, 99.1}
    assert _numbers("PLAN TO 30 NOVEMBER 2026 · 27 Oct-13 Nov · Copter-4.5.7") == set()
    assert _numbers("p99 7.77 ms") == {7.77}, "an unsourced number must be seen"


def test_every_fact_matches_its_file_or_its_files_are_absent():
    got = _built()
    if got == SKIP:
        return SKIP
    facts = json.loads((got[0].parent / "_build" / "facts.json").read_text(encoding="utf-8"))
    bad = [f["id"] for f in facts if f["status"] not in ("verified", "unverified")]
    assert not bad, f"facts not matching their source: {bad}"
    for f in facts:
        if f["status"] == "unverified":
            for src in f["source"].split(" | "):
                p = Path(src) if Path(src).is_absolute() else ROOT / src
                assert not p.exists(), f"{f['id']}: {src} exists but the fact was not checked"
    assert sum(f["status"] == "verified" for f in facts) >= 60, "too few facts checked"


def test_the_builder_refuses_a_number_that_no_longer_matches():
    b = _build()
    if b == SKIP:
        return SKIP
    facts = json.loads((b["out"] / "_build" / "facts.json").read_text(encoding="utf-8"))
    tmp = Path(tempfile.mkdtemp(prefix="explainer_mut_"))
    for f in facts:
        for src in f["source"].split(" | "):
            p = Path(src)
            if p.is_absolute() or not (ROOT / p).is_file():
                continue
            (tmp / p).parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / p, tmp / p)
    same = _run_builder(tmp / "out", "--check-facts-only", env={"EXPLAINER_REPO": str(tmp)})
    assert same.returncode == 0, f"an unchanged copy was refused:\n{same.stdout}\n{same.stderr}"
    sweep = tmp / "docs" / "data" / "scenario_sweep.json"
    d = json.loads(sweep.read_text(encoding="utf-8"))
    d["_rollup"]["_all_shield_on"]["failsafe_labelled_episodes"]["failsafe_trigger_correctness"] = 0.9
    sweep.write_text(json.dumps(d), encoding="utf-8")
    cl = tmp / "CHANGELOG.md"
    cl.write_text(cl.read_text(encoding="utf-8").replace("held 7.99-8.01 m", "held 7.5-8.5 m"), encoding="utf-8")
    moved = _run_builder(tmp / "out", "--check-facts-only", env={"EXPLAINER_REPO": str(tmp)})
    assert moved.returncode == 1, f"a changed number was not refused (exit {moved.returncode})"
    assert "failsafe:" in moved.stderr and "brake_hold:" in moved.stderr, moved.stderr


def _fonts():
    try:
        from PIL import ImageFont
    except ImportError:
        return None
    base = Path(os.environ.get("LOCALAPPDATA", "")) / "Microsoft" / "Windows" / "Fonts"
    reg, bold = base / "Poppins-Regular.ttf", base / "Poppins-Bold.ttf"
    seg = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts" / "segoeui.ttf"
    if not (reg.is_file() and bold.is_file()):
        return None
    f = {"reg": ImageFont.truetype(str(reg), 100), "bold": ImageFont.truetype(str(bold), 100)}
    f["seg"] = ImageFont.truetype(str(seg), 100) if seg.is_file() else f["reg"]
    return f


# PowerPoint's single line for Poppins measured on the rendered slides: about
# 1.2 em (9 pt lines 24 px apart at 160 px per inch); 1.22 keeps a margin.
LINE_EM = 1.22


def _fit_problems(layout: list[dict], fonts) -> list[str]:
    def width_in(word, run):
        f = fonts["seg"] if run["face"] == "Segoe UI" else fonts["bold" if run["bold"] else "reg"]
        return f.getlength(word) / 100 * run["size"] / 72
    out = []
    for r in layout:
        x, y, w, h = r["x"], r["y"], r["w"], r["h"]
        if x < -0.01 or y < -0.01 or x + w > 10.01 or y + h > 5.635:
            out.append(f"slide {r['slide']}: box off the slide: {r['text'][:50]!r}")
        if r["margin"] is None:
            lr, tb = 0.1, 0.05
        else:
            lr = tb = float(r["margin"]) / 72
        avail_w = w - 2 * lr - (0.2 if r.get("bullet") else 0)
        avail_h = h - 2 * tb
        total = 0.0
        paras = [p for p in r["paras"] if p]
        for p in paras:
            words = []                       # (width, size) per word, with its trailing space
            for run in p:
                parts = run["t"].split(" ")
                for k, part in enumerate(parts):
                    if part == "" and k > 0:
                        continue
                    # PowerPoint also breaks a line after a hyphen.
                    pieces = re.findall(r"[^-]*-|[^-]+", part) or [part]
                    for j, piece in enumerate(pieces):
                        cs = r.get("cs", 0) * len(piece) / 72
                        gap = width_in(" ", run) if j == len(pieces) - 1 else 0.0
                        words.append((width_in(piece, run) + cs, gap, run["size"], piece))
            line_w, line_size, lines = 0.0, 0.0, []
            prev = ""
            for ww, sp, size, piece in words:
                if ww > avail_w + 0.02:
                    out.append(f"slide {r['slide']}: a word is wider than its box: {r['text'][:50]!r}")
                if line_w and line_w + ww > avail_w:
                    # A break after "2026-10-" or "7.99-" splits a date or a range.
                    if re.search(r"\d-$", prev) and re.match(r"\d", piece):
                        out.append(f"slide {r['slide']}: a date or range breaks across lines at "
                                   f"{prev + piece!r}: {r['text'][:50]!r}")
                    lines.append(line_size)
                    line_w, line_size = 0.0, 0.0
                line_w += ww + sp
                line_size = max(line_size, size)
                prev = piece
            lines.append(line_size or max((run["size"] for run in p), default=10))
            total += sum(s * LINE_EM * r["lsm"] / 72 for s in lines)
        total += r.get("paraAfter", 0) * max(0, len(paras) - 1) / 72
        if total > avail_h + 0.04:
            out.append(f"slide {r['slide']}: text needs {total:.2f} in, box has {avail_h:.2f} in: {r['text'][:60]!r}")
    return out


def test_no_text_overflows_its_box():
    got = _built()
    if got == SKIP:
        return SKIP
    fonts = _fonts()
    if fonts is None:
        print("      (Pillow or the Poppins fonts are not available here: fit not measured)")
        return SKIP
    layout = json.loads((got[0].parent / "_build" / "layout.json").read_text(encoding="utf-8"))
    assert len(layout) > 200, "layout record looks empty"
    probs = _fit_problems(layout, fonts)
    assert not probs, "\n      ".join(probs)


def test_the_fit_check_catches_an_overflowing_box():
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    fonts = _fonts()
    if fonts is None:
        return SKIP
    run = {"t": "Every VLA command is checked before ArduPilot flies it, ten times a second", "size": 20, "bold": True, "face": "Poppins"}
    box = {"slide": 0, "x": 0.5, "y": 1.0, "w": 4.0, "h": 0.5, "lsm": 1.0, "margin": None, "text": run["t"], "paras": [[run]]}
    assert _fit_problems([box], fonts), "a three-line headline in a one-line box passed"
    box["h"] = 1.6
    assert not _fit_problems([box], fonts), "the same headline in a tall enough box failed"


def test_the_fit_check_catches_a_date_split_across_lines():
    """A footer that wraps inside '2026-10-08' (PowerPoint breaks after a hyphen)
    is reported; the same text in a wide box is not."""
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    fonts = _fonts()
    if fonts is None:
        return SKIP
    # The 13 Oct G1 slide's footer, in its 8.65 in box.
    t = ("Source: demo/out/g1_20261013_203404 (g1_verdict.json); docs/data/progress_oct.json (g1); "
         "CHANGELOG.md 2026-10-13 (later) and 2026-10-08.")
    run = {"t": t, "size": 8.5, "bold": False, "face": "Poppins"}
    box = {"slide": 0, "x": 0.55, "y": 4.95, "w": 8.65, "h": 0.6, "lsm": 1.0, "margin": None, "text": t,
           "paras": [[run]]}
    probs = _fit_problems([box], fonts)
    assert any("breaks across lines" in p for p in probs), probs
    run["t"] = t.replace("2026-10-13 (later) and 2026-10-08", "13 Oct (later) and 8 Oct")
    assert not _fit_problems([box], fonts)


def _said_text(md: str) -> str:
    """What a reader of a pack note is told to say: everything outside the
    "lines not to say" table, plus that table's last column ("say instead")."""
    out = re.sub(r"<!-- dont-say:start -->.*?<!-- dont-say:end -->", "", md, flags=re.S)
    for block in re.findall(r"<!-- dont-say:start -->(.*?)<!-- dont-say:end -->", md, flags=re.S):
        for row in block.splitlines():
            cells = [c.strip() for c in row.strip().strip("|").split("|")]
            if len(cells) >= 2 and not set(cells[-1]) <= set("-: "):
                out += "\n" + cells[-1]
    return out


def _units(md: str) -> list[str]:
    """A pack note cut into the pieces a reader takes in one go: a heading, one
    language's answer, a bullet, a table row, a paragraph."""
    units, cur = [], []
    for line in md.splitlines():
        starts = re.match(r"(#+ |\*\*(EN|ID)\.\*\*|- |\|)", line.strip())
        if not line.strip() or starts:
            if cur:
                units.append(" ".join(cur))
            cur = [line.strip()] if line.strip() else []
        else:
            cur.append(line.strip())
    if cur:
        units.append(" ".join(cur))
    return units


def test_the_units_split_answers_by_language():
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    md = "### Q1. Why?\n**EN.** One line\nwraps here.\n**ID.** Satu baris\nlanjut.\n\n- a bullet\n| a | row |\n"
    assert _units(md) == ["### Q1. Why?", "**EN.** One line wraps here.", "**ID.** Satu baris lanjut.",
                          "- a bullet", "| a | row |"]


def test_the_said_text_keeps_the_say_instead_column():
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    md = ("Intro.\n<!-- dont-say:start -->\n| Do not say | Say instead |\n|---|---|\n"
          "| \"We flew on the drone.\" | \"Desktop figures; the same scripts run unchanged on the Orin.\" |\n"
          "<!-- dont-say:end -->\n")
    said = _said_text(md)
    assert "flew on the drone" not in said and "run unchanged on the Orin" in said
    assert any(re.search(rx, said, flags=re.I) for rx in BANNED_CLAIMS)


def test_the_close_approach_breakdown_adds_up():
    """Slides 12 and 18 split the Shield-off P0 ticks by rule, each tick once;
    the parts must add up to the P0 total, and the Shield-flown and
    ArduPilot-flown ticks to the flight. The 13 Oct flight's first total, which
    took in the P1-only ticks, was retracted on 14 Oct and is no fact."""
    got = _built()
    if got == SKIP:
        return SKIP
    facts = {f["id"]: f for f in json.loads((got[0].parent / "_build" / "facts.json").read_text(encoding="utf-8"))}
    n = lambda k: int(facts[k]["value"].replace(",", ""))  # noqa: E731
    assert n("off_bld_only") + n("off_so_only") + n("off_both") == n("off_p0")
    assert n("on_shield_ticks") + n("on_ap_ticks") == n("on_ticks")
    assert n("voff_bld_only") + n("voff_so_only") + n("voff_both") == n("voff_p0")
    assert n("von_shield_ticks") + n("von_ap_ticks") == n("von_ticks")
    assert n("von_unsafe_shield") + n("von_unsafe_ap") == n("von_unsafe")
    assert round(n("voff_p0") / n("voff_ticks"), 4) == float(facts["voff_rate"]["value"])
    assert "off_any" not in facts and not any(f["value"] == "749" for f in facts.values())
    text = "\n".join(s["text"] for s in got[1])
    assert f"{facts['on_unsafe']['value']} ticks" in text, "the Shield-on unsafe-position count is not on a slide"
    # The 14 Oct pair's position count is on its slide's face too (the PDF has
    # no notes): the Shield-on aircraft stood inside the building margin while
    # the Shield itself still flew. Only where the build has that slide.
    faces = [s["text"] for s in got[1] if "RECORDED CLOSE APPROACH" in s["text"]]
    if faces:
        for k in ("von_unsafe", "von_unsafe_shield", "von_unsafe_ap", "voff_unsafe", "von_depth"):
            assert facts[k]["value"] in faces[0], f"the 14 Oct slide face does not show {k} ({facts[k]['value']})"
        assert f"{facts['von_unsafe']['value']} ticks" in faces[0], "the 14 Oct unsafe-position count is not on its slide"


# The subject question of 13 Oct was answered on 14 Oct (CHANGELOG): the truth
# was right, the detector boxed a red traffic signal, the rail has no start
# gate. These are the open-question wordings that must not come back.
STALE_TRUTH = [
    r"\b(ground[- ]truth|truth)[^.\n]{0,40}\bunresolved\b",
    r"\bunresolved\b[^.\n]{0,40}\btruth\b",
    r"\beither (that |the )?truth\b",
    r"\btruth (pose |frame )?(is )?in another frame\b",
    r"\btruth frame or (the )?detector\b",
    r"\btraced into the Shield's frame\b",
    r"\btruth and detector in one frame\b",
    r"\bmedian gap (now |is )?258\b",
    r"\bpenelusuran frame\b",
    r"\bentah pose truth\b",
]
# "10 of the 149 estimate ticks" is the 13 Oct Shield-on flight alone
# (tests/test_railvideo_truth_frame.py pins it there); written without its
# flight, as the CHANGELOG first had it, it reads as a count over every close
# flight. The pack names the flight ("10 of its 149 ... 13 Oct Shield-on").
MIS_SCOPED = [r"\b10 of the 149\b", r"\b10 dari 149 tick\s+estimasi(?![^.]{0,80}13\s+Okt)"]


def test_no_retracted_count_and_no_open_truth_question_on_the_slides():
    got = _built()
    if got == SKIP:
        return SKIP
    hits = [(s["n"], part, m.group(0)) for s in got[1] for part in ("text", "notes")
            for rx in STALE_TRUTH + MIS_SCOPED + [r"(?<![\d.,])749(?![\d.,])"] for m in re.finditer(rx, s[part], flags=re.I)]
    assert not hits, f"retracted 749, the 13 Oct open truth question or a mis-scoped count on a slide: {hits}"
    allt = "\n".join(s["text"] + "\n" + s["notes"] for s in got[1])
    # The 14 Oct answers: the subject, and why the Shield-on aircraft entered
    # the margin while the Shield flew (its forecast advances the command).
    for must in (r"red traffic signal", r"start gate", r"aircraft's\s+own\s+velocity", r"advances the commanded velocity"):
        assert re.search(must, allt), f"the deck no longer says {must!r}"


def test_the_scope_check_catches_the_unscoped_count():
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    bad = ["across the close flights 0 estimates came within 6 m of the car, which was in the camera's view on 10 of the 149 estimate ticks.",
           "dan mobil hanya terlihat kamera pada 10 dari 149 tick\nestimasi. Rail itu mulai"]
    for line in bad:
        assert any(re.search(rx, line, flags=re.I) for rx in MIS_SCOPED), line
    good = ["On the 13 Oct Shield-on flight the car was in the camera's view on 10 of its 149 estimate ticks.",
            "Mobil terlihat kamera pada 10 dari 149 tick estimasi\npenerbangan Shield-on 13 Okt, dan pada 4 dari 156."]
    for line in good:
        assert not any(re.search(rx, line, flags=re.I) for rx in MIS_SCOPED), line


def test_the_stale_wording_check_catches_the_old_lines():
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    old = ["Either the truth pose is in another frame than the Shield's, or the detector follows something else.",
           "Unresolved: no truth-based number from these runs is used.",
           "the ground truth of the followed car is traced into the Shield's frame",
           "truth and detector in one frame; median gap now 258-264 m"]
    for line in old:
        assert any(re.search(rx, line, flags=re.I) for rx in STALE_TRUTH), line
    new = "The truth was right; the detector boxed a red traffic signal, and the rail has no start gate."
    assert not any(re.search(rx, new, flags=re.I) for rx in STALE_TRUTH)


def test_the_cheatsheets_fit_one_page():
    """One page each: a word budget that prints on one A4 page (checked by a
    print of both sheets when the budget was set)."""
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    sheets = [PACK / "CHEATSHEET-EN.md", PACK / "CHEATSHEET-ID.md"]
    if not all(p.is_file() for p in sheets):
        print("      (the share pack is not on this machine; it is gitignored)")
        return SKIP
    for p in sheets:
        words = len(re.findall(r"\S+", p.read_text(encoding="utf-8")))
        assert words <= CHEATSHEET_WORDS, f"{p.name}: {words} words, budget {CHEATSHEET_WORDS}"


def _print_pages(md: Path) -> int:
    """Pages of `md` printed with the recipe (SHEET_CSS): pandoc's fragment in
    the CSS, Edge headless to an A4 PDF. As tools/deck/build_flowcharts.py
    found: the old --headless with a private profile, the output written to
    that temp folder (Edge wrote nothing to a path with a space on OneDrive),
    and the file polled for rather than the exit code trusted."""
    import time
    edge = next(p for p in EDGE_PATHS if p.is_file())
    frag = subprocess.run(["pandoc", "-f", "markdown", "-t", "html", str(md)], capture_output=True,
                          text=True, encoding="utf-8", check=True).stdout
    tmp = Path(tempfile.mkdtemp(prefix="sheet_print_"))
    html = tmp / f"{md.stem}.html"
    html.write_text(f"<!doctype html><html><head><meta charset='utf-8'><style>{SHEET_CSS}</style>"
                    f"</head><body>{frag}</body></html>", encoding="utf-8")
    pdf = tmp / "sheet.pdf"
    subprocess.run([str(edge), "--headless", "--disable-gpu", "--no-pdf-header-footer", "--print-to-pdf-no-header",
                    f"--user-data-dir={tmp / 'profile'}", f"--print-to-pdf={pdf}", html.as_uri()],
                   capture_output=True, timeout=120)
    for _ in range(150):
        if pdf.is_file() and pdf.stat().st_size > 0:
            time.sleep(0.5)
            break
        time.sleep(0.2)
    assert pdf.is_file() and pdf.stat().st_size > 0, f"Edge wrote no PDF for {md.name}"
    return len(re.findall(rb"/Type\s*/Page(?!s)", pdf.read_bytes()))


def test_the_cheatsheets_print_on_one_a4_page():
    """The proof behind the word budget: both sheets printed with the recorded
    recipe come out as one A4 page each. SKIPs without the pack, pandoc or Edge."""
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    sheets = [PACK / "CHEATSHEET-EN.md", PACK / "CHEATSHEET-ID.md"]
    if not all(p.is_file() for p in sheets):
        print("      (the share pack is not on this machine; it is gitignored)")
        return SKIP
    if shutil.which("pandoc") is None or not any(p.is_file() for p in EDGE_PATHS):
        print("      (pandoc or Microsoft Edge is not installed here: not printed)")
        return SKIP
    pages = {p.name: _print_pages(p) for p in sheets}
    assert all(n == 1 for n in pages.values()), f"pages per sheet: {pages}"


def test_the_pack_notes_are_clean_and_their_numbers_match_the_deck():
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    md = {n: PACK / n for n in ("CHEATSHEET-EN.md", "CHEATSHEET-ID.md", "LIKELY-QUESTIONS.md")}
    if not all(p.is_file() for p in md.values()):
        print("      (the share pack is not on this machine; it is gitignored)")
        return SKIP
    import check_claims as cc
    for name, p in md.items():
        text = p.read_text(encoding="utf-8")
        hits = cc.scan_text(text, f"docs/share/2026-10-14-Explainer/{name}")
        assert not hits, f"{name}: withdrawn claims {[(h.line, h.claim) for h in hits]}"
        said = _said_text(text)
        hedges = [m.group(0) for m in HEDGE_RX.finditer(said)]
        assert not hedges, f"{name}: hedges {hedges}"
        banned = [m.group(0) for rx in BANNED_CLAIMS for m in re.finditer(rx, said, flags=re.I)]
        assert not banned, f"{name}: claims of results that do not exist {banned}"
    facts = json.loads((PACK / "_build" / "facts.json").read_text(encoding="utf-8"))
    values = {f["value"] for f in facts}
    for name in ("CHEATSHEET-EN.md", "CHEATSHEET-ID.md"):
        text = md[name].read_text(encoding="utf-8")
        block = re.search(r"<!-- numbers:start -->(.*?)<!-- numbers:end -->", text, flags=re.S)
        assert block, f"{name}: number table markers missing"
        rows = [r for r in block.group(1).splitlines() if re.match(r"\|\s*\d+\s*\|", r)]
        assert len(rows) == 10, f"{name}: {len(rows)} numbers, expected ten"
        for r in rows:
            shown = re.search(r"\*\*(.+?)\*\*", r).group(1)
            parts = re.split(r" vs | to | menjadi | = | \(target | \(limit ", shown)
            if len(parts) > 1 and parts[-1].endswith(")"):
                parts[-1] = parts[-1][:-1]
            assert all(p.strip() in values for p in parts), f"{name}: {shown!r} is not the deck's fact"
    q = md["LIKELY-QUESTIONS.md"].read_text(encoding="utf-8")
    heads = re.findall(r"^### ", q, flags=re.M)
    assert 25 <= len(heads) <= 40, f"{len(heads)} questions"
    blocks = re.split(r"^### ", q, flags=re.M)[1:]
    for b in blocks:
        assert "**EN.**" in b and "**ID.**" in b, f"question without both answers: {b.splitlines()[0]}"
    # A blank line before each Indonesian answer, so a Markdown viewer shows the
    # two answers as separate paragraphs instead of one mixed block.
    glued = [m.group(0)[:40] for m in re.finditer(r"[^\n]\n\*\*ID\.\*\*", q)]
    assert not glued, f"an ID answer runs on from its EN answer: {glued}"
    for hard in (r"0\.99", r"Orin", r"10 Hz", r"0\.86 s", r"BRAKE", r"ground[- ]truth", r"KPI-grade",
                 r"stream stops", r"hot-apply", r"signature", r"red traffic signal", r"start gate",
                 r"\b736\b", r"\bHUD\b", r"\bRTL\b", r"control arm", r"aircraft's\s+own\s+velocity"):
        assert re.search(hard, q, flags=re.I), f"no question covers {hard}"
    # The 14 Oct answer replaced the 13 Oct open question, the retracted 749
    # appears only where a line says it was retracted, and the 13 Oct flight's
    # in-view count names its flight.
    for name, p in md.items():
        said = _said_text(p.read_text(encoding="utf-8"))
        stale = [m.group(0) for rx in STALE_TRUTH + MIS_SCOPED for m in re.finditer(rx, said, flags=re.I)]
        assert not stale, f"{name}: the 13 Oct open truth question or a mis-scoped count is back: {stale}"
        for unit in _units(said):
            if re.search(r"(?<![\d.,])749(?![\d.,])", unit):
                assert re.search(r"retracted|ditarik", unit, flags=re.I), f"{name}: 749 without its retraction: {unit!r}"
    for name in ("CHEATSHEET-EN.md", "CHEATSHEET-ID.md", "LIKELY-QUESTIONS.md"):
        text = md[name].read_text(encoding="utf-8")
        assert re.search(r"\b736\b", text) and re.search(r"\b727\b", text), f"{name}: both close-approach pairs are not there"
        assert re.search(r"red traffic signal|lampu lalu lintas merah", text), f"{name}: the 14 Oct truth answer is not there"


def test_the_pack_readme_lists_every_file_and_video():
    """The pack README is written by the builder: every file in the pack folder
    (the _build records by name) and every video the builder knows is listed."""
    assert BUILDER.is_file(), f"{BUILDER.relative_to(ROOT)} does not exist"
    readme = PACK / "README.md"
    if not readme.is_file():
        print("      (the share pack is not on this machine; it is gitignored)")
        return SKIP
    text = readme.read_text(encoding="utf-8")
    missing = []
    for p in sorted(PACK.rglob("*")):
        if p.is_dir() or "_build" in p.relative_to(PACK).parts:
            continue
        rel = p.relative_to(PACK).as_posix()
        if f"`{rel}`" not in text:
            missing.append(rel)
    assert not missing, f"files in the pack the README does not list: {missing}"
    for rec in ("facts.json", "slides_text.json", "layout.json", "build_stamp.json", "render/slide_NN.png"):
        assert f"`_build/{rec}`" in text, f"_build/{rec} is not listed"
    js = BUILDER.read_text(encoding="utf-8")
    videos = re.findall(r"\"(demo/out/pas_ap_video_[\w/]+\.mp4)\"", js)
    assert len(videos) >= 6, videos
    for v in videos:
        assert f"`{v}`" in text, f"video not listed: {v}"
    for v in ("citylife_redcar_nfz_20261013_demo.mp4", "_demo_share.mp4", "_demo_phone.mp4"):
        assert v in text, f"video not listed: {v}"
    # Every video of the pack's runs on this machine is listed, not only the
    # ones the builder already knows (the compare video's _phone re-encode was
    # once missed that way).
    out = ROOT / "demo" / "out"
    on_machine = (sorted(out.glob("pas_ap_video_*.mp4")) + sorted(out.glob("pas_ap_video_*/*.mp4"))
                  + sorted(out.glob("citylife_redcar_nfz_20261013/*.mp4")))
    unlisted = [p.relative_to(ROOT).as_posix() for p in on_machine if f"`{p.relative_to(ROOT).as_posix()}`" not in text]
    assert not unlisted, f"videos of the pack's runs the README does not list: {unlisted}"
    for row in re.findall(r"^\| `demo/out/[^`]+\.mp4` \|.*$", text, flags=re.M):
        cells = [c.strip() for c in row.strip("|").split("|")]
        assert len(cells) == 6, f"video row without its six cells (video, length, size, frame, codec, what): {row}"
        on_disk = (ROOT / cells[0].strip("`")).is_file()
        if on_disk:
            assert re.fullmatch(r"\d+ s", cells[1]) and re.fullmatch(r"\d+\.\d MB", cells[2]), row
            assert cells[4] in ("H.264", "MPEG-4 Part 2"), f"codec not read: {row}"
    assert re.search(r"H\.264 files, which play in a browser", text), "the README does not say which videos to send"


if __name__ == "__main__":
    fns = [v for k, v in globals().items() if k.startswith("test_") and callable(v)]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
