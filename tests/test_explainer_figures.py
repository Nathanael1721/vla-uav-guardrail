"""The explainer figures (tools/deck/build_explainer_figures.py): they build, they
parse, the committed files are what the script draws, and every number printed
on them comes from the repository file named for it.

    python tests/test_explainer_figures.py

The number check reads the committed SVGs, not the script's memory: what is
checked is what a reader sees. A digit in a <text> is a number unless it is
glued to a letter (P0, X6, WP3, Ed25519, OpenVLA-7B are names); the circled
step badges carry class="ordinal" and must count 1, 2, 3 ... in order.
"""
from __future__ import annotations

import importlib.util
import re
import struct
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "deck" / "build_explainer_figures.py"
IMG = ROOT / "docs" / "img"
NAMES = ("explainer_flowchart_end_to_end", "explainer_arch_components",
         "explainer_arch_topologies", "explainer_arch_shield", "explainer_arch_evidence")
SVG_NS = "{http://www.w3.org/2000/svg}"
SKIP = "SKIP"
_MOD = None


def _mod():
    """The build script as a module (loaded once; a missing script fails here)."""
    global _MOD
    assert SCRIPT.is_file(), f"{SCRIPT.relative_to(ROOT)} does not exist"
    if _MOD is None:
        spec = importlib.util.spec_from_file_location("build_explainer_figures", SCRIPT)
        _MOD = importlib.util.module_from_spec(spec)
        sys.modules["build_explainer_figures"] = _MOD
        spec.loader.exec_module(_MOD)
    return _MOD


def _svg_texts(path: Path) -> list[tuple[str, str | None, float]]:
    """(text, class, font-size) of every <text> in an SVG file."""
    root = ET.parse(path).getroot()
    out = []
    for t in root.iter(f"{SVG_NS}text"):
        out.append(("".join(t.itertext()), t.get("class"), float(t.get("font-size", 0))))
    return out


def _png_size(path: Path) -> tuple[int, int]:
    data = path.read_bytes()[:24]
    assert data[:8] == b"\x89PNG\r\n\x1a\n", f"{path.name} is not a PNG"
    return struct.unpack(">II", data[16:24])


def test_the_script_runs_and_writes_only_explainer_svgs():
    """The script exits 0 and writes exactly the five explainer_*.svg files into
    the folder it is given: it never writes (so never overwrites) another image."""
    with tempfile.TemporaryDirectory() as td:
        r = subprocess.run([sys.executable, str(SCRIPT), "--out", td],
                           capture_output=True, text=True, timeout=300, cwd=str(ROOT))
        assert r.returncode == 0, f"exit {r.returncode}\n{r.stdout}\n{r.stderr}"
        got = sorted(p.name for p in Path(td).iterdir())
        assert got == sorted(f"{n}.svg" for n in NAMES), got


def test_each_committed_svg_exists_and_parses_as_xml():
    for n in NAMES:
        p = IMG / f"{n}.svg"
        assert p.is_file(), f"{p.relative_to(ROOT)} missing"
        root = ET.parse(p).getroot()
        assert root.tag == f"{SVG_NS}svg", (n, root.tag)
        w, h = int(root.get("width")), int(root.get("height"))
        assert root.get("viewBox") == f"0 0 {w} {h}", (n, root.get("viewBox"))
        assert len(_svg_texts(p)) > 20, f"{n} has almost no text"


def test_the_committed_svgs_are_what_the_script_draws():
    """A figure edited by hand, or a script changed without rebuilding, fails."""
    m = _mod()
    figs = {f.name: f for f in m.build_all()}
    assert sorted(figs) == sorted(NAMES), sorted(figs)
    for n in NAMES:
        on_disk = (IMG / f"{n}.svg").read_text(encoding="utf-8").replace("\r\n", "\n")
        assert on_disk == figs[n].svg(), f"{n}.svg differs from what the script draws now"


def test_every_number_on_a_figure_comes_from_its_named_source():
    """Each number in a committed SVG has a Fact for that figure, and every Fact
    matches in the file it names (the number inside the match, or a count)."""
    m = _mod()
    figs = {f.name: f for f in m.build_all()}
    problems = []
    for n in NAMES:
        facts = figs[n].facts
        assert facts, f"{n} declares no facts"
        covered = {f.num for f in facts}
        for s, cls, _ in _svg_texts(IMG / f"{n}.svg"):
            if cls == "ordinal":
                continue
            for num in m.numbers_in(s):
                if num not in covered:
                    problems.append(f"{n}: {num!r} in {s!r} has no source")
        for f in facts:
            p = f.problem()
            if p:
                problems.append(f"{n}: {p}")
    assert not problems, "\n".join(problems)


def test_step_badges_count_from_one_in_order():
    for n in NAMES:
        steps = [s for s, cls, _ in _svg_texts(IMG / f"{n}.svg") if cls == "ordinal"]
        assert all(s.isdigit() for s in steps), (n, steps)
        assert [int(s) for s in steps] == list(range(1, len(steps) + 1)), (n, steps)


def test_printed_paths_and_code_names_exist_where_cited():
    """check_facts: every printed file path exists, every code name (operator,
    state, manifest field, topic) is in the file cited for it, and no Fact
    names a number the figure does not print."""
    m = _mod()
    problems = [p for f in m.build_all() for p in m.check_facts(f)]
    assert not problems, "\n".join(problems)
    n_files = sum(len(f.files) for f in m.build_all())
    n_names = sum(len(f.names) for f in m.build_all())
    assert n_files >= 30 and n_names >= 80, (n_files, n_names)


def test_the_fact_checker_catches_drift():
    """The checker is not vacuous: a changed number, a wrong count, an
    unsourced number and a missing file are each reported."""
    m = _mod()
    assert m.Fact("5", "guardrail/shield.py", r"^LOOKAHEAD_S = 5\.0$").problem() is None
    assert m.Fact("7", "guardrail/shield.py", r"^LOOKAHEAD_S = 5\.0$").problem()
    assert m.Fact("5", "guardrail/shield.py", r"^LOOKAHEAD_S = 7\.0$").problem()
    assert m.Fact("11", "guardrail/protos/harness.proto", r"^\s*rpc \w+",
                  kind="count").problem() is None
    assert m.Fact("10", "guardrail/protos/harness.proto", r"^\s*rpc \w+",
                  kind="count").problem()
    f = m.Fig("probe", 400, 200)
    f.text(10, 50, "forecast of 7 poses")
    f.file("guardrail/no_such_module.py")
    f.name_in("guardrail/fsm.py", "NoSuchState")
    got = m.check_facts(f)
    assert any("'7'" in p and "no source" in p for p in got), got
    assert any("no_such_module" in p for p in got), got
    assert any("NoSuchState" in p for p in got), got
    # names, not numbers
    assert m.numbers_in("P0 X6 WP3 Ed25519 OpenVLA-7B sha256 4418ae0") == []
    assert m.numbers_in("UDP 9002/9003, Copter-4.5.7, 0.1 s, 4-D") == \
        ["9002", "9003", "4.5.7", "0.1", "4"]


def test_no_arrow_runs_through_a_box():
    m = _mod()
    problems = [p for f in m.build_all() for p in m.layout_problems(f)]
    assert not problems, "\n".join(problems)


def test_text_fits_and_nothing_overlaps_when_measured():
    """With Poppins and Consolas available (the lab machine), every string fits
    its box, labels clear boxes and arrows, and no two strings overlap."""
    m = _mod()
    results = [m.fit_problems(f) for f in m.build_all()]
    if any(r is None for r in results):
        print("      Poppins / Consolas or PIL not available: fit not measured")
        return SKIP
    problems = [p for r in results for p in r]
    assert not problems, "\n".join(problems)


def test_no_text_is_smaller_than_19_px():
    """A floor in canvas units (the on-slide size is the next test)."""
    for n in NAMES:
        p = IMG / f"{n}.svg"
        w = int(ET.parse(p).getroot().get("width"))
        small = [(s, sz) for s, _, sz in _svg_texts(p) if sz < 19]
        assert not small, (n, small[:5])
        assert w <= 1600, (n, w)


def _deck_figure_box() -> tuple[float, float]:
    """The figure box of build_explainer_deck.js figureSlide, in inches."""
    js = (ROOT / "tools" / "deck" / "build_explainer_deck.js").read_text(encoding="utf-8")
    m = re.search(r"const bx = [\d.]+, by = [\d.]+, bw = ([\d.]+), bh = ([\d.]+);", js)
    assert m, "figureSlide's box (bx, by, bw, bh) not found in build_explainer_deck.js"
    return float(m.group(1)), float(m.group(2))


def test_every_figure_text_is_readable_where_the_deck_shows_it():
    """The deck fits each figure into figureSlide's box; there the smallest text
    of every figure must still be 8.5 pt or more (the slide's own footer is 7.5
    pt). The box the script assumes must be the box the deck uses."""
    m = _mod()
    bw, bh = _deck_figure_box()
    assert (bw, bh) == m.SLIDE_BOX_IN, f"deck box {bw} x {bh} in, script assumes {m.SLIDE_BOX_IN}"
    small = []
    for n in NAMES:
        p = IMG / f"{n}.svg"
        root = ET.parse(p).getroot()
        w, h = int(root.get("width")), int(root.get("height"))
        scale = min(bw * 72 / w, bh * 72 / h)
        least = min(sz for _, _, sz in _svg_texts(p))
        if least * scale < 8.5:
            small.append(f"{n}: {least:g} units -> {least * scale:.2f} pt on the slide")
    assert not small, "\n".join(small)


def test_each_png_is_the_svg_rendered_at_twice_its_size():
    for n in NAMES:
        svg, png = IMG / f"{n}.svg", IMG / f"{n}.png"
        assert png.is_file(), f"{png.relative_to(ROOT)} missing (build with --png)"
        root = ET.parse(svg).getroot()
        w, h = int(root.get("width")), int(root.get("height"))
        assert _png_size(png) == (2 * w, 2 * h), (n, _png_size(png), (w, h))


def test_the_palette_is_the_deck_theme():
    m = _mod()
    theme = (ROOT / "tools" / "deck" / "nathan_theme.js").read_text(encoding="utf-8")
    for hexcol in m.THEME_HEX:
        assert f'"{hexcol[1:].upper()}"' in theme, f"{hexcol} is not in nathan_theme.js"
    for n in NAMES:
        txt = (IMG / f"{n}.svg").read_text(encoding="utf-8")
        assert "Poppins" in txt and m.TEAL in txt, n


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
