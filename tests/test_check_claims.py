"""tools/check_claims.py: the withdrawn-claim and corrected-value scan.

Run either way:
    pytest tests/test_check_claims.py -v
    python tests/test_check_claims.py

WHY THIS FILE EXISTS

A scan that cannot fail is not a guard. On 6 Oct the first version of this
scan (then a scratchpad script) excused any hit on a line that also said
"corrected", and a reviewer re-introduced six withdrawn claims it could not see.
So each test below breaks something on purpose and requires the scan to say no:

  * every pattern must still match a plain statement of its claim, and every
    CHANGELOG.md retraction must be registered (a new retraction with no
    pattern fails here, by design);
  * an unquoted claim, and a quoted one with no retraction marker, are
    live; a quoted one marked as withdrawn, or one in an "Old wording"
    column, is not. A source word ("the grant says", "published result:")
    is not a retraction marker (review, 6 Oct);
  * in a table the attribution must be in the same row;
  * in code and JSON, a string literal's own delimiters are not quotation
    marks: a claim in a printed string is live unless the printed text
    quotes it and marks it as withdrawn;
  * deleting any corrected value a file must keep is reported;
  * the reviewer's re-introductions (appended to copies of the living
    documents) are all caught;
  * frozen records are counted and listed, and fail under --strict;
  * the living documents and generators in the repository are clean today;
  * (2026-10-19) on a clean clone, without the gitignored internal documents
    (check_claims.LOCAL_ONLY), the scan passes and names each one it
    skipped, while a deleted TRACKED required file still fails; the
    LOCAL_ONLY set is exactly the gitignored REQUIRED files; the default
    scope covers the code packages, and the whole default scan is clean.

A test that could not check what it is for on this checkout (a local-only
document absent) says so: it returns SKIP, counted apart, or prints which
files it skipped - never a silent PASS.
"""
import contextlib
import fnmatch
import io
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import check_claims as C                                           # noqa: E402

SKIP = "SKIP"

# Living documents the 6 Oct corrections went into (the v2 scan's file list).
# The four gitignored ones (C.LOCAL_ONLY) exist only on the lab machine.
LIVING = [
    "README.md", "docs/index.md", "docs/scope-clarification.md",
    "docs/prof-repo-study.md", "docs/CHECKLIST-remaining-work.md",
    "docs/GAMBARAN-SISTEM.md", "docs/PROGRESS-NOTE-30Sep2026.md",
    "docs/PRESENTATION-CHEATSHEET-30Sep2026.md",
    "docs/share/2026-09-30-ITRI/README.md",
    "docs/CORRECTION-2026-10-06-mid-evaluation-and-deck.md",
]
# Generators and code whose wording this unit corrected.
GENERATORS = [
    "tools/build_mideval_report.py", "tools/build_report_results.py",
    "tools/build_eval_data.py", "tools/build_deck_data.py",
    "tools/deck/build_mideval_deck.js", "tools/deck/build_updates_deck.js",
    "tools/deck/build_sept_deck.js", "tools/deck/build_progress_0930_deck.js",
    "tools/deck/build_midterm_deck.js", "tools/prefix_eval.py",
    "guardrail/compiler.py", "guardrail/csp.py", "demo/detectors.py", "demo/run_demo.py",
]
CORRECTED_DOCS = [
    "docs/MEETING-PACK-Sept2026.md", "docs/architecture-v2.md",
    "docs/initial-report-draft.md",
    "docs/FINDING-what-the-contract-locks-and-what-it-does-not.md",
    "docs/FINDING-forward-flew-north.md",
    "docs/FINDING-the-kpi-report-that-scored-every-rule-as-p0.md",
    "docs/FINDING-a-stripped-signature-read-as-unsigned.md",
    "docs/FINDING-the-paraphrase-sets-keyed-to-the-wrong-rendering.md",
    "docs/FINDING-the-control-arm-that-scored-a-repair-success.md",
    "docs/FINDING-the-mission-that-succeeded-inside-the-ring.md",
    "docs/projectairsim-setup.md",
]


def _scan(text, rel="docs/x.md"):
    return C.scan_text(text, rel)


def _root_with(files: dict) -> Path:
    root = Path(tempfile.mkdtemp())
    for rel, text in files.items():
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8", newline="\n")
    return root


def _copy(rels) -> Path:
    root = Path(tempfile.mkdtemp())
    for rel in rels:
        src = ROOT / rel
        if src.is_file():
            (root / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src, root / rel)
    return root


# --------------------------------------------------------------------------- #

def test_every_pattern_catches_a_plain_statement_of_its_claim():
    dead = [(cid, rx[:60]) for cid, rx, _ in C.CLAIMS
            if not any(re.search(rx, e, flags=re.M) for e in C.CLAIM_EXAMPLES)]
    assert not dead, f"patterns that match none of the examples: {dead}"
    uncaught = [e for e in C.CLAIM_EXAMPLES
                if not any(re.search(rx, e, flags=re.M) for _, rx, _ in C.CLAIMS)]
    assert not uncaught, f"example claims no pattern catches: {uncaught}"
    # ...and as live claims in a document, not only as regex matches
    for e in C.CLAIM_EXAMPLES:
        assert _scan(e + "\n"), f"scan let a plain claim through: {e!r}"


def test_every_changelog_retraction_is_registered():
    """Every top-level bullet under a Retracted heading in CHANGELOG.md names a
    registered key; every registry value is a claim id that exists or says why
    there is no sentence to scan for."""
    ids = {cid for cid, _, _ in C.CLAIMS}
    for key, val in C.RETRACTIONS.items():
        if val.startswith("not a sentence"):
            continue
        for cid in (v.strip() for v in val.split(",")):
            assert cid in ids, f"RETRACTIONS[{key!r}] names unknown claim id {cid!r}"
    lines = (ROOT / "CHANGELOG.md").read_text(encoding="utf-8").split("\n")
    in_ret, bullets = False, []
    for ln in lines:
        if re.match(r"^#{2,4} ", ln):
            in_ret = bool(re.match(r"^#{3,4} Retracted", ln))
            continue
        if in_ret and ln.startswith("- "):
            bullets.append(ln)
    assert len(bullets) >= 30, f"only {len(bullets)} retraction bullets found: parser broke?"
    missing = [b[:100] for b in bullets
               if not any(k in b.lower() for k in C.RETRACTIONS)]
    assert not missing, ("CHANGELOG retractions with no entry in "
                         "tools/check_claims.py RETRACTIONS: " + repr(missing))


def test_unquoted_and_unattributed_quotes_are_live_attributed_quotes_are_not():
    claim = "The contractual gate is cleared."
    assert _scan(f"Status: {claim}\n"), "unquoted claim passed"
    assert _scan(f'Retracted: the slide read "{claim}"\n') == [], "quoted + retracted was flagged"
    assert _scan(f'Our note: "{claim}"\n'), "a quotation with no attribution passed"
    assert _scan(f"Withdrawn: ~~{claim}~~\n") == []
    # the attribution must be near: four lines away does not count
    far = f'Retracted.\n\n\n\n\n\nThe deck: "{claim}"\n'
    assert _scan(far), "an attribution five lines up excused the claim"


def test_quotes_may_open_on_the_line_before_the_claim():
    text = ('> Retracted: it said `bundle.py` "matches the\n'
            '>   reference layout byte-for-byte". Only the container matches.\n')
    assert _scan(text) == [], "a quotation spanning two lines was misread"


def test_correction_tables_old_column_and_same_row_attribution():
    tbl = ("| # | Old wording | Correct wording |\n|---|---|---|\n"
           "| A1 | The contractual gate is cleared. | Dev topology only. |\n")
    assert _scan(tbl) == [], "a claim in the Old wording column was flagged"
    tbl2 = ("| # | Where | Note |\n|---|---|---|\n"
            "| A1 | report | \"The contractual gate is cleared.\" was withdrawn |\n"
            "| A2 | deck | \"Hard KPI: P0 = 0\" |\n")
    hits = _scan(tbl2)
    assert [h.line for h in hits] == [4], f"row attribution leaked across rows: {hits}"


def test_code_strings_are_live_unless_attributed():
    js = 's.addText("Hard KPI: P0 violation escape rate = 0.", { x: 1 });\n'
    assert C.scan_text(js, "tools/deck/x.js"), "a printed claim in a JS string passed"
    ok = 's.addText("\\"Hard KPI\\" was withdrawn", {});  // retracted claim, quoted\n'
    assert C.scan_text(ok, "tools/deck/x.js") == []
    py = 'x = 1\n# The flight behind the withdrawn "fired six times".\n'
    assert C.scan_text(py, "tools/x.py") == []
    py_live = 'print("The 10 m stand-off fired six times.")\n'
    assert C.scan_text(py_live, "tools/x.py")
    # An attribution word nearby does not excuse a claim that is NOT quoted:
    # a comment restating the claim under a "Retracted" heading is a claim.
    near = "# Retracted items are listed in CHANGELOG.md.\n# The 10 m stand-off fired six times.\n"
    assert C.scan_text(near, "tools/x.py"), "an unquoted comment claim was excused"


def test_a_source_word_does_not_excuse_a_quoted_claim():
    """Review, 6 Oct: 'said', 'says', 'published', 'reported as' and 'memo'
    counted as attribution, so a withdrawn claim attributed to the grant or to
    a published result scanned clean - which is how C-02 was made."""
    for text in ('The grant says "Hard KPI: P0 violation escape rate = 0".',
                 'As the contract says, "the acceptance criterion is met on every flight".',
                 'Published result: "All five KPIs measured on the canonical rail".',
                 'The memo reported as "The contractual gate is cleared."',
                 'Our slide said: "Swapping the detector is inside the contract."'):
        assert _scan(text + "\n"), f"a source word excused the claim: {text!r}"
    # ...while a dated correction or a correction-table row id still does
    assert _scan('Corrected 2026-10-06 (A3): "It is 0.0 on all 41 shielded flights."\n') == []


def test_a_json_string_is_not_a_quotation():
    """Review, 6 Oct: JSON used prose quote parity, so every string value was
    'quoted' and a site.json claim next to a "published" key passed."""
    site = ('{\n  "published": "2026-10-06",\n'
            '  "items": ["The contractual gate is cleared on the dev rail."]\n}\n')
    hits = C.scan_text(site, "docs/progress/_src/site.json")
    assert [h.claim for h in hits] == ["C-01"], hits
    marked = ('{\n  "note": "Retracted 2026-10-06: \\"The contractual gate is cleared.\\""\n}\n')
    assert C.scan_text(marked, "docs/progress/_src/site.json") == []


def test_code_literal_delimiters_are_not_quotation_marks():
    """A JS or Python string literal prints its content without its quotes;
    a nearby 'Retracted' comment does not excuse a claim the slide prints."""
    js = ('// Retracted claims are listed on slide 12.\n'
          's.addText("The contractual gate is cleared.", {});\n')
    assert C.scan_text(js, "tools/deck/x.js"), "a printed claim passed beside a comment"
    js_ok = "const OLD = '\"The contractual gate is cleared.\"';  // Retracted (A1)\n"
    assert C.scan_text(js_ok, "tools/deck/x.js") == []
    # a Python raw string is a regular expression (a scanner's pattern table)
    assert C.scan_text('PATS = [r"(?i)byte-for-byte"]\n', "tools/x.py") == []
    assert C.scan_text('MSG = "The bundle matches byte-for-byte."\n', "tools/x.py")
    # a docstring is prose: a quoted, retracted claim inside it passes
    doc = 'def f():\n    """Retracted: "The contractual gate is cleared."\n    """\n'
    assert C.scan_text(doc, "tools/x.py") == []


def test_a_directly_negated_claim_is_its_correction():
    assert _scan("Replay is not byte-for-byte.\n") == []
    assert _scan("P0 escape is one of five acceptance KPIs, not 'the hard KPI'.\n") == []
    assert _scan("Replay is byte-for-byte.\n"), "the claim itself passed"


def test_the_review_sept_deck_and_updates_deck_wordings_are_caught():
    """Wordings the 6 Oct review found still printed by generators."""
    sept = ('"... the detector, whose rate only just clears its 4.0 Hz gate. '
            'Measured with 29 objects on the street: detector 5.15 Hz, all boxes."')
    assert {h.claim for h in C.scan_text("s.addText(" + sept + ");\n", "tools/deck/x.js")} == {"det"}
    upd = 's.addText(`P0 escape rate is 0.0 on all ${E.kpi.flights_escape_zero} shielded flights`);\n'
    assert [h.claim for h in C.scan_text(upd, "tools/deck/x.js")] == ["C-02"]
    assert C.scan_text('print("Detector rate: 4.56 Hz")\n', "tools/x.py")
    assert C.scan_text('s.addText("detector 3.63 Hz", {});\n', "tools/deck/x.js")
    # the recomputed values are not flagged
    assert _scan("Detector rate over the mission: 4.06 Hz (city_kpi).\n") == []


def test_main_survives_a_cp1252_console():
    """Review, 6 Oct: main() raised UnicodeEncodeError on a Windows console
    when a LIVE line held a character cp1252 lacks; the list was lost."""
    root = _root_with({"docs/u.md": "Fail-safe ≥ 0.99 → The contractual gate is cleared.\n"})
    raw = io.BytesIO()
    out = io.TextIOWrapper(raw, encoding="cp1252", errors="strict")
    with contextlib.redirect_stdout(out):
        code = C.main(["docs/u.md", "--root", str(root)])
    out.flush()
    text = raw.getvalue().decode("cp1252")
    assert code == 1 and "LIVE" in text and "1 live claims" in text, text
    raw = io.BytesIO()
    out = io.TextIOWrapper(raw, encoding="cp1252", errors="strict")
    with contextlib.redirect_stdout(out):
        C.main(["docs/u.md", "--root", str(root), "--json"])
    out.flush()
    import json as _json
    assert len(_json.loads(raw.getvalue().decode("cp1252"))["live"]) == 1


def test_deleting_any_required_value_is_reported():
    present = [rel for rel in C.REQUIRED if (ROOT / rel).is_file()]
    gone = sorted(set(C.REQUIRED) - set(present) - C.LOCAL_ONLY)
    assert not gone, f"tracked REQUIRED files missing: {gone}"
    skipped = sorted(set(C.REQUIRED) - set(present))
    if skipped:
        print(f"      (local-only, not in this checkout, not checked: {skipped})")
    for rel in present:
        text = (ROOT / rel).read_text(encoding="utf-8").replace("\r\n", "\n")
        assert C.check_required(text, rel) == [], f"{rel} is already missing a value"
        for rx in C.REQUIRED[rel]:
            cut = re.sub(rx, "", text)
            assert rx in C.check_required(cut, rel), f"{rel}: removing /{rx}/ went unnoticed"


def test_the_reviewers_reintroductions_are_caught():
    """The v2 mutants M3-M6, M9, M11, M13, M14 (appended re-introductions),
    plus the generator-side ones this unit removed."""
    mutants = {
        "docs/PROGRESS-NOTE-30Sep2026.md": "All five acceptance KPIs are measured on the canonical topology.",
        "README.md": "The contractual gate is cleared on canonical-hil.",
        "docs/index.md": "The grant's hard KPI is P0 escape = 0 (corrected wording).",
        "docs/prof-repo-study.md": "Four rails ran end-to-end, Gazebo included.",
        "docs/share/2026-09-30-ITRI/README.md":
            "> We have not yet recorded a city flight in which the Shield itself acts.",
        "docs/CHECKLIST-remaining-work.md": "Five KPI-grade runs exist.",
        "docs/GAMBARAN-SISTEM.md": 'Status Gazebo: "belum pernah dijalankan".',
        "docs/scope-clarification.md": "Fail-safe trigger correctness, target 1.0.",
        "tools/build_mideval_report.py": '    md += "Detector rate in flight: 3.76-5.22 Hz"\n',
        "tools/deck/build_mideval_deck.js": '      ["WP2", "Prefix compiler", "Built", "rules into prompt"],\n',
    }
    synthetic = []
    for rel, extra in mutants.items():
        root = _copy([rel])
        p = root / rel
        if not p.is_file() and rel in C.LOCAL_ONLY:
            # A gitignored document absent from this checkout (2026-10-19):
            # the mutant is still scanned under the file's own path, alone,
            # so the rule that catches it there is still exercised.
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text("", encoding="utf-8")
            synthetic.append(rel)
        assert p.is_file(), f"{rel} is gone from the repository"
        s = p.read_text(encoding="utf-8")
        p.write_text(s.rstrip("\n") + "\n\n" + extra + "\n", encoding="utf-8", newline="\n")
        res = C.run([p], root)
        assert res["live"], f"re-introduction in {rel} was not caught: {extra!r}"
    if synthetic:
        print(f"      (local-only, absent: mutants scanned in an empty file at "
              f"their path: {synthetic})")


def test_frozen_records_are_counted_and_fail_under_strict():
    root = _root_with({"docs/MID-EVALUATION-REPORT-Sep2026.md":
                       "The contractual gate is cleared.\n"})
    p = root / "docs/MID-EVALUATION-REPORT-Sep2026.md"
    res = C.run([p], root)
    assert res["live"] == [] and len(res["frozen_hits"]) == 1, res
    assert "delivered" in res["frozen_hits"][0]["frozen"]
    strict = C.run([p], root, strict=True)
    assert len(strict["live"]) == 1, strict
    # a corrected finding doc is guarded even though FINDING-* is frozen
    rel = "docs/FINDING-what-the-contract-locks-and-what-it-does-not.md"
    assert C.frozen_reason(rel) is None and C.frozen_reason("docs/FINDING-x.md")


def test_main_exit_code_and_summary_line():
    root = _root_with({"docs/a.md": "All good here.\n",
                       "docs/b.md": "Swapping the detector is inside the contract.\n"})
    for rel, want in (("docs/a.md", 0), ("docs/b.md", 1)):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = C.main([rel, "--root", str(root)])
        assert code == want, (rel, code, out.getvalue())
        assert re.search(r"\d+/\d+ files clean, \d+ live claims, \d+ missing required, "
                         r"\d+ frozen hits", out.getvalue()), out.getvalue()


def test_the_living_documents_and_generators_are_clean():
    rels = [r for r in LIVING + GENERATORS + CORRECTED_DOCS if (ROOT / r).is_file()]
    gone = sorted(set(LIVING + GENERATORS + CORRECTED_DOCS) - set(rels))
    # A gitignored document is absent from every clean clone (2026-10-19);
    # it is named, not silently passed, and a tracked one still fails.
    assert not (set(gone) - C.LOCAL_ONLY), f"guarded files missing: {gone}"
    if gone:
        print(f"      (local-only, not in this checkout, not scanned: {gone})")
    res = C.run([ROOT / r for r in rels], ROOT)
    msgs = [f"{h['file']}:{h['line']} [{h['claim']}] {h['text']}" for h in res["live"]]
    msgs += [f"{m['file']}: missing /{m['pattern']}/" for m in res["missing_required"]]
    assert not msgs, "\n      ".join(["withdrawn claims or missing values:"] + msgs)


# --------------------------------------------------------------------------- #
# a clean clone, the local-only documents, the default scope (2026-10-19)
# --------------------------------------------------------------------------- #

def _tracked_required_root() -> Path:
    """A temporary checkout holding the tracked REQUIRED files only - what a
    clone or a `git archive` copy has of them."""
    return _copy([rel for rel in C.REQUIRED if rel not in C.LOCAL_ONLY])


def test_a_clean_clone_skips_the_local_only_documents_visibly_and_passes():
    """REVIEW 2026-10-19 (check-claims-required-files-gitignored). On any
    checkout without the four gitignored internal documents the full scan
    reported 4 MISSING-REQUIRED and exited 1. Now it skips them by name and
    passes; a deleted TRACKED required file still fails."""
    root = _tracked_required_root()
    res = C.run(C.default_targets(root), root)
    assert res["missing_required"] == [], res["missing_required"]
    assert {s["file"] for s in res["skipped_local"]} == set(C.LOCAL_ONLY), res
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = C.main(["--root", str(root)])
    text = out.getvalue()
    assert code == 0, text
    assert text.count("SKIP-LOCAL") == 4 and "4 local-only skipped" in text, text
    # --require-local: the lab machine's guard against a deleted document
    with contextlib.redirect_stdout(io.StringIO()):
        assert C.main(["--root", str(root), "--require-local"]) == 1
    # a tracked REQUIRED file deleted: still MISSING-REQUIRED, exit 1
    (root / "docs" / "index.md").unlink()
    res = C.run(C.default_targets(root), root)
    assert {m["file"] for m in res["missing_required"]} == {"docs/index.md"}, res
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        assert C.main(["--root", str(root)]) == 1
    assert "MISSING-REQUIRED docs/index.md" in out.getvalue()


def test_a_local_only_document_deleted_where_the_others_exist_is_missing():
    """The skip is for a checkout with NONE of the internal documents. Where
    one is present (the lab machine), an absent one was deleted and fails as
    before - the deletion guard the REQUIRED rule exists for."""
    root = _tracked_required_root()
    keep = "docs/GAMBARAN-SISTEM.md"
    (root / keep).parent.mkdir(parents=True, exist_ok=True)
    (root / keep).write_text("≥ 99 %\ntidak ada hasil run yang disimpan\n",
                             encoding="utf-8")
    res = C.run(C.default_targets(root), root)
    assert res["skipped_local"] == [], res["skipped_local"]
    assert {m["file"] for m in res["missing_required"]} == C.LOCAL_ONLY - {keep}, res
    # ...and an explicit target that is a local-only file, absent on a clean
    # clone, is skipped the same way
    root = _tracked_required_root()
    res = C.run([root / keep], root)
    assert res["missing_required"] == [] and res["skipped_local"][0]["file"] == keep, res


def _gitignored(rels) -> set[str] | None:
    """The paths among `rels` that .gitignore ignores: `git check-ignore` when
    this is a git checkout, else the plain patterns of .gitignore (a `git
    archive` copy has no .git). None if neither can answer."""
    rels = sorted(rels)
    if (ROOT / ".git").exists():
        try:
            r = subprocess.run(["git", "-C", str(ROOT), "check-ignore", *rels],
                               capture_output=True, text=True, timeout=30)
            if r.returncode in (0, 1):
                return {ln.strip() for ln in r.stdout.splitlines() if ln.strip()}
        except (OSError, subprocess.SubprocessError):
            pass
    gi = ROOT / ".gitignore"
    if not gi.is_file():
        return None
    pats = [ln.strip() for ln in gi.read_text(encoding="utf-8").splitlines()
            if ln.strip() and not ln.startswith(("#", "!"))]
    out = set()
    for rel in rels:
        for pat in pats:
            pat = pat.lstrip("/")
            # "dir/" ignores everything under it; fnmatch's * crosses "/".
            if fnmatch.fnmatch(rel, pat + "*" if pat.endswith("/") else pat):
                out.add(rel)
    return out


def test_local_only_is_exactly_the_gitignored_required_files():
    """LOCAL_ONLY must not become a place to hide a tracked file (whose
    deletion would then be skipped on CI), nor miss a gitignored one (which
    would turn every clean clone red again)."""
    ignored = _gitignored(C.REQUIRED)
    if ignored is None:
        print("      (no .git and no .gitignore here: cannot tell what is ignored)")
        return SKIP
    assert set(C.LOCAL_ONLY) == ignored, (sorted(C.LOCAL_ONLY), sorted(ignored))


def test_the_default_scope_covers_the_code_and_the_whole_scan_is_clean():
    """REVIEW 2026-10-19 (claim-scan-scope-misses-live-retracted-wording). The
    default run read README, docs and tools only, and printed "180/180 files
    clean" while guardrail/kpi.py's docstring called P0 escape the grant's
    hard KPI and demo/parked_cars.py said det_hz just clears its gate."""
    targets = {C._rel(p, ROOT) for p in C.default_targets(ROOT)}
    for rel in ("guardrail/kpi.py", "guardrail/shield.py", "demo/parked_cars.py",
                "demo/follow_vlm.py", "experiments/bench_detectors.py",
                "sitl/ros2_shield_node.py", "README.md", "tools/build_deck_data.py"):
        assert rel in targets, f"{rel} is outside the default scope"
    assert not any(t.startswith("tests/") for t in targets), "tests/ holds the examples"
    assert "tools/check_claims.py" not in targets
    assert not any("/out/" in t for t in targets), "an output tree is scanned"
    res = C.run(C.default_targets(ROOT), ROOT)
    msgs = [f"{h['file']}:{h['line']} [{h['claim']}] {h['text']}" for h in res["live"]]
    assert not msgs, "\n      ".join(["live withdrawn claims:"] + msgs)
    # The newly scanned code can fail: a claim planted in a guardrail docstring.
    root = _copy(["guardrail/kpi.py"])
    p = root / "guardrail" / "kpi.py"
    p.write_text(p.read_text(encoding="utf-8").replace(
        '"""', '"""P0 violation escape rate = 0 is the grant\'s hard KPI.\n', 1),
        encoding="utf-8")
    assert [h["claim"] for h in C.run([p], root)["live"]] == ["C-02"]


def test_only_the_changelog_preamble_is_scanned():
    """The CHANGELOG's preamble states the project's position, so a claim
    there is live; its body's Retracted bullets quote every withdrawn claim
    and are not scanned (they would be ~20 false hits)."""
    text = ("# Changelog\n\nThe contractual gate is cleared.\n\n## [0.1.0]\n\n"
            "### Retracted\n- \"Swapping the detector is inside the contract.\"\n")
    cut = C.scanned_text(text, "CHANGELOG.md")
    assert "Swapping" not in cut and cut == text[:text.index("## [")]
    root = _root_with({"CHANGELOG.md": text})
    res = C.run([root / "CHANGELOG.md"], root)
    assert [(h["claim"], h["line"]) for h in res["live"]] == [("C-01", 3)], res["live"]
    assert C.scanned_text(text, "docs/x.md") == text


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
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
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed, {skipped} skipped")
    sys.exit(1 if failed else 0)
