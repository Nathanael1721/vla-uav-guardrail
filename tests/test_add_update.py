"""The tracker's update log rules: tools/add_update.py.

Run either way:
    pytest tests/test_add_update.py -v
    python tests/test_add_update.py

The log is how the project reports what changed, in three languages. These
tests pin that `check` refuses each way it could quietly go wrong: a gap or a
duplicate in the numbering, an invented release, a missing language, a
Simplified character, a number the source does not contain or that differs
between languages, a retraction merged away, a reference to nothing. Each
case breaks one rule of a valid fixture and expects that rule's message.
"""
import copy
import json
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import add_update as A                                          # noqa: E402

CHANGELOG = """# Changelog

## [Unreleased]

### 2026-09-20 — second piece of work

#### Retracted
- The old number 7.5 was wrong; it is 6.2.
- A second claim, withdrawn.

#### Added
- A thing that measured 98.5 % on 12 runs.

### 2026-09-20 (evening) — later the same day

#### Added
- Another thing, 3 files.

## [0.2.0] — 2026-09-10

### Added
- Corridor rule, 24 tests.

## [0.1.0] — 2026-09-01 → 2026-09-05

### Added
- The first release, 368 tests.
"""


def tri(en, id_=None, zh=None):
    return {"en": en, "id": id_ if id_ is not None else en, "zh": zh if zh is not None else en}


def valid_doc():
    return {"schema": 1, "entries": [
        {"seq": 1, "id": "u1", "kind": "release", "version": "0.1.0", "base_version": None,
         "date": "2026-09-05", "date_start": "2026-09-01", "source": {"file": "CHANGELOG.md", "heading": "## [0.1.0] — 2026-09-01 → 2026-09-05"},
         "changes": [{"cat": "added", "i18n": tri("First release with 368 tests.", "Rilis pertama dengan 368 tes.", "第一版，368 項測試。")}],
         "files": ["x.py"], "cards": ["A-01"], "pages": ["board"], "media": ["hud"],
         "highlights": [{"value": "368", "i18n": tri("tests", "tes", "測試")}], "proof": [],
         "i18n": {"en": {"title": "First release", "summary": "The first release."},
                  "id": {"title": "Rilis pertama", "summary": "Rilis pertama."},
                  "zh": {"title": "第一版", "summary": "第一個版本。"}}},
        {"seq": 2, "id": "u2", "kind": "work", "version": "0.2.0+2026-09-20", "base_version": "0.2.0",
         "date": "2026-09-20", "source": {"file": "CHANGELOG.md", "heading": "### 2026-09-20 — second piece of work"},
         "changes": [
             {"cat": "retracted", "i18n": tri("The old number 7.5 was wrong; it is 6.2.", "Angka lama 7.5 salah; yang benar 6.2.", "舊數字 7.5 有誤，正確為 6.2。")},
             {"cat": "retracted", "i18n": tri("A second claim was withdrawn.", "Klaim kedua ditarik.", "第二項說法已撤回。")},
             {"cat": "added", "i18n": tri("Measured 98.5 % on 12 runs.", "Terukur 98.5 % pada 12 run.", "12 次飛行中量測到 98.5 %。")}],
         "files": [], "cards": [], "pages": [], "media": ["img/a.png"], "highlights": [], "proof": [],
         "i18n": {"en": {"title": "Second piece", "summary": "Two retractions and an addition."},
                  "id": {"title": "Bagian kedua", "summary": "Dua retraksi dan satu tambahan."},
                  "zh": {"title": "第二部分", "summary": "兩項撤回與一項新增。"}}},
        {"seq": 3, "id": "u3", "kind": "work", "version": "0.2.0+2026-09-20.2", "base_version": "0.2.0",
         "date": "2026-09-20", "source": {"file": "CHANGELOG.md", "heading": "### 2026-09-20 (evening) — later the same day"},
         "changes": [{"cat": "added", "i18n": tri("Another thing in 3 files.", "Hal lain di 3 file.", "另一項變更，共 3 個檔案。")}],
         "files": [], "cards": [], "pages": [], "media": [], "highlights": [], "proof": [],
         "i18n": {"en": {"title": "Later the same day", "summary": "One more addition."},
                  "id": {"title": "Kemudian di hari yang sama", "summary": "Satu tambahan lagi."},
                  "zh": {"title": "同一天稍晚", "summary": "再一項新增。"}}},
    ]}


class Fixture:
    def __enter__(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="upd_test_"))
        (self.tmp / "CHANGELOG.md").write_text(CHANGELOG, encoding="utf-8")
        (self.tmp / "VERSION").write_text("0.2.0\n", encoding="utf-8")
        (self.tmp / "x.py").write_text("x = 1\n", encoding="utf-8")
        (self.tmp / "tracker" / "data" / "cards").mkdir(parents=True)
        (self.tmp / "tracker" / "data" / "cards" / "batch_00.json").write_text(
            json.dumps([{"id": "A-01", "evidence": ["x.py"]}]), encoding="utf-8")
        (self.tmp / "tracker" / "assets").mkdir(parents=True)
        (self.tmp / "tracker" / "assets" / "app.js").write_text('const MEDIA = { hud: ["img", "a.png"] };', encoding="utf-8")
        (self.tmp / "docs" / "img").mkdir(parents=True)
        (self.tmp / "docs" / "img" / "a.png").write_bytes(b"png")
        self.ctx = A.load_ctx(self.tmp)
        return self

    def __exit__(self, *a):
        shutil.rmtree(self.tmp, ignore_errors=True)


def errs_after(mutate):
    with Fixture() as f:
        d = valid_doc()
        mutate(d)
        return A.check(d, f.ctx)


def expect(mutate, needle):
    e = errs_after(mutate)
    assert any(needle in m for m in e), f"expected an error containing {needle!r}, got {e}"


# ------------------------------------------------------------------ the good case
def test_the_valid_fixture_passes():
    with Fixture() as f:
        assert A.check(valid_doc(), f.ctx) == []


def test_the_changelog_parser_finds_releases_dates_and_retractions():
    secs = A.parse_changelog(CHANGELOG)
    assert [s.version for s in secs] == [None, None, "0.2.0", "0.1.0"]
    assert secs[0].retracted == 2 and secs[1].retracted == 0
    assert secs[3].date == "2026-09-05" and secs[3].date_start == "2026-09-01"
    assert A.releases(secs) == [("0.1.0", "2026-09-05"), ("0.2.0", "2026-09-10")]
    assert A.base_version("2026-09-20", A.releases(secs)) == "0.2.0"
    assert A.base_version("2026-09-07", A.releases(secs)) == "0.1.0"


def test_versions_count_same_day_entries():
    with Fixture() as f:
        d = valid_doc()
        assert A.next_version({"entries": d["entries"][:1]}, "2026-09-20", f.ctx, "work") == "0.2.0+2026-09-20"
        assert A.next_version({"entries": d["entries"][:2]}, "2026-09-20", f.ctx, "work") == "0.2.0+2026-09-20.2"
        assert A.next_version(d, "2026-09-20", f.ctx, "work") == "0.2.0+2026-09-20.3"


def test_numbers_are_normalised():
    assert A.numbers("2,399 ticks at 98.5 % and 0.985") == {"2399", "98.5", "0.985"}
    assert A.numbers("run 03 of 10") == {"3", "10"}
    assert A.numbers("flight id4") == set()                        # glued to a word: an identifier
    assert A.numbers("`run 7` and 12") == {"7", "12"}              # strip_code is the caller's job
    assert A.numbers(A.strip_code("`run 7` and 12")) == {"12"}


def test_add_writes_atomically_with_lf_endings():
    with Fixture() as f:
        p = f.tmp / "u.json"
        A.write_atomic(p, valid_doc())
        raw = p.read_bytes()
        assert b"\r\n" not in raw and json.loads(raw.decode("utf-8")) == valid_doc()
        assert not list(f.tmp.glob(".u.json.*"))


# ------------------------------------------------------------------ each rule, broken on purpose
def test_a_gap_in_seq_is_refused():
    expect(lambda d: d["entries"][2].update(seq=4), "seq must be 3")


def test_a_duplicate_id_is_refused():
    expect(lambda d: d["entries"][1].update(id="u1"), "id missing or duplicate")


def test_dates_out_of_order_are_refused():
    expect(lambda d: d["entries"][2].update(date="2026-09-04", version="0.1.0+2026-09-04", base_version="0.1.0"),
           "is before the previous entry")


def test_an_invented_release_is_refused():
    expect(lambda d: d["entries"][1].update(kind="release", version="0.2.1"), "is not a CHANGELOG release heading")


def test_a_release_with_the_wrong_date_is_refused():
    expect(lambda d: d["entries"][0].update(date="2026-09-04"), "is dated 2026-09-05 in CHANGELOG")


def test_unreleased_work_must_not_look_like_a_release():
    expect(lambda d: d["entries"][1].update(version="0.2.1"), "must be <release>+<date>")


def test_the_wrong_base_version_is_refused():
    expect(lambda d: d["entries"][1].update(version="0.1.0+2026-09-20"), "base version 0.1.0 should be 0.2.0")


def test_a_missing_language_is_refused():
    def m(d):
        del d["entries"][0]["i18n"]["zh"]
    expect(m, "zh title/summary missing")
    expect(lambda d: d["entries"][1]["changes"][2]["i18n"].update(id=""), "change 3 has no id text")


def test_simplified_chinese_is_refused():
    expect(lambda d: d["entries"][0]["i18n"]["zh"].update(title="这个版本"), "Simplified characters")


def test_an_english_number_not_in_the_source_is_refused():
    expect(lambda d: d["entries"][1]["changes"][2]["i18n"].update(en="Measured 99.1 % on 12 runs."), "99.1")


def test_a_translation_with_different_numbers_is_refused():
    expect(lambda d: d["entries"][1]["changes"][2]["i18n"].update(id="Terukur 98,5 % pada 12 run."), "id numbers differ")
    expect(lambda d: d["entries"][1]["changes"][2]["i18n"].update(zh="13 次飛行中量測到 98.5 %。"), "zh numbers differ")


def test_a_merged_retraction_is_refused():
    def m(d):
        ch = d["entries"][1]["changes"]
        ch[0]["i18n"] = tri("Two claims withdrawn: 7.5 was 6.2.", "Dua klaim ditarik: 7.5 menjadi 6.2.", "撤回兩項說法：7.5 改為 6.2。")
        del ch[1]
    expect(m, "1 retracted change(s), but the source has 2")


def test_unknown_references_are_refused():
    expect(lambda d: d["entries"][0].update(cards=["Z-99"]), "unknown card Z-99")
    expect(lambda d: d["entries"][0].update(files=["nope.py"]), "file does not exist")
    expect(lambda d: d["entries"][0].update(media=["img/missing.png"]), "media not found")
    expect(lambda d: d["entries"][0].update(media=["nosuchid"]), "unknown media id")
    expect(lambda d: d["entries"][0].update(pages=["admin"]), "unknown page admin")
    expect(lambda d: d["entries"][0]["changes"][0].update(cat="misc"), "unknown category misc")


def test_a_missing_source_heading_is_refused():
    expect(lambda d: d["entries"][1]["source"].update(heading="### 2026-09-20 — renamed"), "source heading not found")


def test_the_real_log_passes_check():
    doc = A.load_doc()
    if not doc["entries"]:
        return "SKIP"
    errs = A.check(doc, A.load_ctx())
    assert errs == [], errs[:10]


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == "SKIP":
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
