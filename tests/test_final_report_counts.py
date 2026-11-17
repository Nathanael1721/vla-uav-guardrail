"""The contract map's totals in docs/FINAL-REPORT.md and
docs/FINAL-REPORT-outline.md agree with their own Appendix A.

    python tests/test_final_report_counts.py      (no pytest needed)

WHY THIS FILE EXISTS

Both files print the Appendix A totals in several places (the outline's
header, the report's summary, the report's Appendix A preamble, one
"N items:" line per section) and copy the same rows. Before 17 Nov the
outline's WP4 and architecture section lines read 12 / 9 and 18 / 9 while
their rows counted 13 / 8 and 17 / 10: hand-edited totals drift from the
rows they summarise. This test recounts the rows and holds every printed
total to the recount, and holds the two appendices to the same status per
row. It also pins the 17 Nov wording decisions: the team is stated as in
the grant (1 PI + 4 Master students, one team), never as one person, and
the dated deliverables are reported on what was delivered, without a
lateness column.
"""
from __future__ import annotations

import re
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPORT = "docs/FINAL-REPORT.md"
OUTLINE = "docs/FINAL-REPORT-outline.md"
STATUSES = ("DONE", "PARTIAL", "DRIFT", "MISSING")
ROW = re.compile(r"^\| ([A-Z]+[0-9]*-[0-9]+) \| [^|]* \| (DONE|PARTIAL|DRIFT|MISSING) \|")
SECTION_LINE = re.compile(r"^(\d+) items: (.*)\.$")


def _text(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


def appendix(rel: str) -> tuple[dict[str, str], list[tuple[str, Counter, str]]]:
    """Return {row id: status} and, per section, (heading, recount, printed line)."""
    rows: dict[str, str] = {}
    sections: list[tuple[str, Counter, str]] = []
    in_a = False
    for line in _text(rel).split("\n"):
        if line.startswith("## Appendix A"):
            in_a = True
            continue
        if in_a and line.startswith("## "):
            break
        if not in_a:
            continue
        if line.startswith("### "):
            sections.append((line, Counter(), ""))
            continue
        m = SECTION_LINE.match(line)
        if m and sections:
            head, cnt, _ = sections[-1]
            sections[-1] = (head, cnt, line)
            continue
        m = ROW.match(line)
        if m and sections:
            assert m.group(1) not in rows, f"{rel}: row {m.group(1)} appears twice in Appendix A"
            rows[m.group(1)] = m.group(2)
            sections[-1][1][m.group(2)] += 1
    return rows, sections


def _printed(line: str) -> Counter:
    m = SECTION_LINE.match(line)
    assert m, f"not a section total line: {line!r}"
    out: Counter = Counter()
    for part in m.group(2).split(", "):
        n, status = part.split(" ")
        out[status] = int(n)
    assert sum(out.values()) == int(m.group(1)), f"{line!r}: its parts do not add up to its total"
    return out


def _totals(rel: str) -> Counter:
    rows, _ = appendix(rel)
    return Counter(rows.values())


def _phrase(c: Counter) -> str:
    return ", ".join(f"{c[s]} {s}" for s in STATUSES)


def test_each_section_line_matches_its_rows():
    for rel in (REPORT, OUTLINE):
        _, sections = appendix(rel)
        assert len(sections) == 6, f"{rel}: expected 6 Appendix A sections, found {len(sections)}"
        for head, cnt, line in sections:
            assert line, f"{rel} {head}: no 'N items:' line"
            assert _printed(line) == +cnt, f"{rel} {head}: prints {line!r}, the rows count {dict(cnt)}"


def test_the_two_appendices_agree_row_by_row():
    rep, _ = appendix(REPORT)
    out, _ = appendix(OUTLINE)
    assert len(out) == 136, f"the outline's Appendix A has {len(out)} rows, not 136"
    assert rep == out, {k: (rep.get(k), out.get(k)) for k in set(rep) | set(out) if rep.get(k) != out.get(k)}


def test_every_printed_total_is_the_recount():
    tot = _totals(OUTLINE)
    phrase = _phrase(tot)
    flat_outline = " ".join(_text(OUTLINE).split())
    flat_report = " ".join(_text(REPORT).split())
    assert f"**{phrase}.**" in flat_outline, f"the outline's header does not print **{phrase}.**"
    assert f"**{phrase}**" in flat_report, f"the report's summary does not print **{phrase}**"
    assert re.search(rf"re-assessed \d+ \w+: {re.escape(phrase)}\)", flat_report), \
        f"the report's Appendix A preamble does not print {phrase}"
    # no stale total of the four statuses survives in either file's current
    # text (the dated independent-review notes at the end keep their own)
    any_total = re.compile(r"\b(\d+) DONE, (\d+) PARTIAL, (\d+) DRIFT, (\d+) MISSING\b")
    for rel, flat, note in ((OUTLINE, flat_outline, "## Independent-review note"),
                            (REPORT, flat_report, "## Appendix B")):
        assert note in flat, f"{rel}: no {note!r} heading"
        for m in any_total.finditer(flat.split(note)[0]):
            assert m.group(0) == phrase, f"{rel}: a stale total {m.group(0)!r} (recount: {phrase})"


def test_the_rows_the_17_nov_decision_fixes():
    # the totals are not pinned here: test_every_printed_total_is_the_recount
    # holds every printed total to the rows, whatever later row moves bring
    rows, _ = appendix(OUTLINE)
    for rid in ("WP1-21", "WP3-01", "ARCH-16", "ARCH-17", "ARCH-23", "ARCH-24"):
        assert rows[rid] == "DONE", f"{rid} is {rows[rid]}"
    # G1's test passed over SIM_AirSim JSON, not HIL_GPS / HIL_SENSOR, and the
    # gate's swap rule was not applied: DRIFT, with no claim that its THEN branch held
    assert rows["ARCH-15"] == "DRIFT"
    for rel in (REPORT, OUTLINE):
        assert "THEN branch" not in " ".join(_text(rel).split()), f"{rel}: claims G1's THEN branch"


def test_the_team_is_stated_as_in_the_grant():
    one_person = re.compile(r"one[- ]person|one developer|a single developer|reported as one person", re.I)
    for rel in (REPORT, OUTLINE):
        text = _text(rel)
        hit = one_person.search(text)
        assert hit is None, f"{rel}: {text[max(0, hit.start() - 80):hit.end() + 40]!r}"
        row = next(l for l in text.split("\n") if l.startswith("| ARCH-24 |") and "| DONE |" in l)
        assert "1 PI + 4 Master students (one team)" in row, row


def test_no_lateness_column_or_narrative():
    late = re.compile(r"\| Late by \||dated gate missed|gate missed by|\d+ days after its|"
                      r"after the 20 July gate|the 20 July gate was missed|Dated gates: the grant's dates")
    for rel in (REPORT, OUTLINE):
        text = _text(rel)
        hit = late.search(text)
        assert hit is None, f"{rel}: {text[max(0, hit.start() - 80):hit.end() + 40]!r}"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
