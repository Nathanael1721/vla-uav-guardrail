"""The public progress site generator (tools/build_public_site.py).

Run either way:
    pytest tests/test_build_public_site.py -v
    python tests/test_build_public_site.py

WHY THIS FILE EXISTS

docs/progress/ is published on GitHub Pages for readers outside the lab, and
it is built partly from the local tracker, which holds internal records
(audit verdicts, PI decisions, card notes, retraction lists). A generator
that copies one field too many leaks them, and it would do so silently: the
page still renders and still looks right.

So most tests here are about REFUSAL. Each one feeds the generator something
it must not publish - a disallowed field, a withdrawn claim, a person's name,
a number no source supports, a quantity written as a word, an unreviewed or
changed image, a command naming a missing script - and requires it to say
no. Each refusal test is paired with a case that must pass, so a generator
that refuses everything cannot pass either.

A middle group runs the whole build: the public overrides must reach every
page and the committed snapshot (fed raw tracker wording, the build must not
show it), and `--check` must fail on a hand-edited page and on new work
written under a CHANGELOG heading the site already cites.

The last group builds the real site from the committed sources (the curated
site.json and the tracker snapshot, so it runs on a clone without the
gitignored tracker data) and checks every page: well-formed, both languages,
every local link and image resolves, and the committed pages match a fresh
build, so nobody edits generated HTML by hand.

No names of real people appear in this file except the PI's; the name check
is tested with invented names.
"""
import contextlib
import copy
import io
import json
import re
import sys
import tempfile
from html.parser import HTMLParser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import build_public_site as B                                  # noqa: E402

SITE = ROOT / "docs" / "progress"
TASK_DISALLOWED = ("verdict", "needs_pi", "evidence", "notes", "retracted", "decisions")


# ------------------------------------------------------------ fixtures
def _tr(**fields):
    """A tracker i18n block with the same fields in all three languages."""
    return {lang: {k: f"{v} [{lang}]" for k, v in fields.items()} for lang in ("en", "id", "zh")}


def fake_tracker():
    """Synthetic tracker data: every allow-listed field carries an ALLOWED_
    marker, every internal field a LEAK_ marker."""
    overview = {
        "project": {"media": ["LEAK_media"], "i18n": {
            lang: {"one_liner": "ALLOWED_one", "problem": "ALLOWED_problem",
                   "solution": "ALLOWED_solution", "contract_facts": ["LEAK_team"],
                   "honesty": ["LEAK_honesty"]} for lang in ("en", "id", "zh")}},
        "mental_model": [{"role_key": "driver", "file": "LEAK_file", "media": [],
                          "i18n": _tr(analogy="ALLOWED_analogy", in_project="ALLOWED_in",
                                      detail="LEAK_detail")}],
        "wps": [{"key": "WP1", "grant_page": "Policy DSL", "grant_quote": "ALLOWED_quote",
                 "files": ["LEAK_files"], "kpis": ["LEAK_kpi"], "media": [],
                 "i18n": _tr(name="ALLOWED_name", plain="ALLOWED_plain",
                             status="LEAK_status audit: 1 DONE", grant_asks="LEAK_asks")}],
        "decisions": [{"i18n": _tr(question="LEAK_decision", why="LEAK_why")}],
        "kpis": [{"key": "k", "status": "partial", "value": "LEAK_value",
                  "i18n": _tr(name="n", gap="LEAK_gap")}],
    }
    flow = {
        "grant_architecture": [{"key": "shield", "built": "partial", "file": "LEAK_file",
                                "i18n": _tr(label="ALLOWED_label", short="ALLOWED_short",
                                            detail="LEAK_detail")}],
        "demo_pipeline": [{"key": "camera", "rate": "LEAK_rate", "file": "f",
                           "i18n": _tr(label="ALLOWED_pl", short="ALLOWED_ps",
                                       detail="LEAK_pdetail")}],
        "shield_steps": [{"key": "predict", "i18n": _tr(label="ALLOWED_step", detail="LEAK_step")}],
        "hud_callouts": [{"key": "banner", "x": 640, "y": 26,
                          "i18n": _tr(label="ALLOWED_hl", detail="ALLOWED_hd")}],
        "statuses": [{"key": "ok", "color": "#46cd64", "i18n": _tr(label="OK", meaning="ALLOWED_m")}],
        "examples": [{"media": "m", "i18n": _tr(title="t", not_proves="LEAK_not_proves")}],
        "results": [{"run": "r", "notes": {"en": "LEAK_notes"}}],
        "glossary": [{"term": "VLA", "i18n": {"en": "ALLOWED_gloss", "id": "x", "zh": "ALLOWED_gloss_zh"}},
                     {"term": "Paraphraser", "i18n": {"en": "LEAK_not_built", "id": "x", "zh": "x"}}],
    }
    return overview, flow


def load_committed():
    content = B.read_json(B.CONTENT_FILE)
    extract = B.read_json(B.SNAPSHOT_FILE)
    return content, extract


_BUILD_CACHE = {}


def committed_build():
    """Build the real site once, from committed sources only."""
    if "pages" not in _BUILD_CACHE:
        content, extract = load_committed()
        _BUILD_CACHE["pages"], _BUILD_CACHE["stats"] = B.build(content, extract)
    return _BUILD_CACHE["pages"], _BUILD_CACHE["stats"]


def refuses(fn, exc=(B.PolicyViolation, B.BuildError)):
    try:
        fn()
    except exc as e:
        return str(e)
    return None


# --------------------------------------------------- the allow-list
def test_each_named_disallowed_field_in_the_allow_list_is_refused():
    assert B.check_allow_list(B.ALLOW_LIST) is None          # the real list passes
    for field in TASK_DISALLOWED:
        for slot in (3, 4):                                  # plain fields, text fields
            allow = copy.deepcopy(B.ALLOW_LIST)
            src, key, idf, plain, text = allow["wps"]
            spec = [src, key, idf, plain, text]
            spec[slot] = tuple(spec[slot]) + (field,)
            allow["wps"] = tuple(spec)
            msg = refuses(lambda: B.check_allow_list(allow), B.PolicyViolation)
            assert msg and field in msg, f"allow-list with {field!r} in slot {slot} was accepted"
            ov, fl = fake_tracker()
            assert refuses(lambda: B.extract_tracker(ov, fl, ["VLA"], allow), B.PolicyViolation), \
                f"extract_tracker ran with {field!r} on the allow-list"


def test_the_extract_copies_allowed_fields_and_nothing_else():
    ov, fl = fake_tracker()
    x = B.extract_tracker(ov, fl, ["VLA"])
    dump = json.dumps(x, ensure_ascii=False)
    leaks = sorted(set(re.findall(r"LEAK_\w+", dump)))
    assert not leaks, f"internal fields copied: {leaks}"
    # The null: an extractor that copied nothing would pass the line above.
    for marker in ("ALLOWED_one", "ALLOWED_problem", "ALLOWED_analogy", "ALLOWED_quote",
                   "ALLOWED_plain", "ALLOWED_label", "ALLOWED_short", "ALLOWED_pl", "ALLOWED_step",
                   "ALLOWED_hd", "ALLOWED_m", "ALLOWED_gloss", "ALLOWED_gloss_zh"):
        assert marker in dump, f"{marker} was dropped"
    assert "[id]" not in dump, "the Indonesian text is not part of the public site"
    assert x["hud_callouts"][0]["x"] == 640 and x["statuses"][0]["color"] == "#46cd64"
    assert [g["id"] for g in x["glossary"]] == ["VLA"], "an unlisted glossary term was copied"


def test_a_field_the_list_names_but_the_source_lacks_is_an_error_not_a_gap():
    ov, fl = fake_tracker()
    del ov["wps"][0]["i18n"]["zh"]["plain"]
    assert refuses(lambda: B.extract_tracker(ov, fl, ["VLA"]), B.BuildError)
    ov, fl = fake_tracker()
    assert refuses(lambda: B.extract_tracker(ov, fl, ["VLA", "No such term"]), B.BuildError)


def test_a_disallowed_key_anywhere_in_the_build_inputs_is_refused():
    content, extract = load_committed()
    for field in TASK_DISALLOWED:
        c = copy.deepcopy(content)
        c["wps"]["WP2"]["shown"][0][field] = "x"
        msg = refuses(lambda: B.build(c, extract, names=set()), B.PolicyViolation)
        assert msg and field in msg, f"{field} in site.json was accepted"
        x = copy.deepcopy(extract)
        x["roles"][0][field] = "x"
        msg = refuses(lambda: B.build(content, x, names=set()), B.PolicyViolation)
        assert msg and field in msg, f"{field} in the extract was accepted"
        # Nested inside a field the allow-list does copy: the extract itself
        # refuses, before anything (such as the snapshot) is made from it.
        ov, fl = fake_tracker()
        ov["wps"][0]["i18n"]["en"]["plain"] = {"text": "nested", field: "LEAK_nested"}
        msg = refuses(lambda: B.extract_tracker(ov, fl, ["VLA"]), B.PolicyViolation)
        assert msg and field in msg, f"{field} nested in an allowed field was copied"


# --------------------------------------------------- forbidden text
WITHDRAWN = (
    "the rail is canonical-hil", "our canonical HIL runs", "the hard KPI is met",
    "0.0 on all 41 shielded flights", "detector 3.76-5.22 Hz", "1 of 34 camera flights",
    "11713/11713 repairs converged", "fail-safe target 1.0", "a contract KPI figure",
    "five KPI-grade runs", "verdict PARTIAL", "card WP3-07", "open question PQ1",
    "a written waiver", "claims retracted", "as agreed at the meeting",
    "see the meeting notes", "raised in the ITRI meeting", "meeting of 30 Sep",
    "write to someone@example.org", "see the grant audit", "four fully-run rails",
    "hardware flight is a stretch goal", "合約 KPI 數字", "會議中決定",
    "flown by AirSim's own autopilot", "使用 AirSim 內建飛控", "由AirSim飛行",
)
NEUTRAL = (
    "Obstacle clearance is 3 m.", "Risk grading 0.5 / 0.3 / 0.2.", "Shield corrections 0",
    "The audit log (audit.jsonl).", "The grant's dev topology.", "hil: VLA and Shield on a Jetson Orin.",
    "Acceptance KPI: P0 violation escape rate.", "Prof. Kuan-Ting Lai, NTUT AIoT Lab",
    "Project AirSim's own flight controller", "Project AirSim 內建的飛行控制器", "`projectairsim` client",
    "a Stress Testing run executed in the [hil] topology",
    # Ordinary English and an accurate name for the July simulator.
    "meeting the 5 ms budget", "a check meeting its deadline",
    "the July OpenVLA run used classic AirSim (AirSimNH)",
)


def test_withdrawn_claims_and_internal_markers_are_refused():
    for text in WITHDRAWN:
        assert refuses(lambda: B.scan_text({"en": text}, set(), "t"), B.PolicyViolation), \
            f"accepted: {text!r}"


def test_neutral_project_wording_is_not_refused():
    for text in NEUTRAL:
        msg = refuses(lambda: B.scan_text({"en": text}, set(), "t"))
        assert msg is None, f"refused {text!r}: {msg}"


def test_person_names_are_refused_but_the_pi_is_not():
    names = {"alice", "zhangwei"}
    assert refuses(lambda: B.scan_text({"en": "Built by Alice."}, names, "t"), B.PolicyViolation)
    assert refuses(lambda: B.scan_text({"zh": "由 ZhangWei 撰寫"}, names, "t"), B.PolicyViolation)
    assert refuses(lambda: B.scan_text({"zh": "由ZhangWei撰寫"}, names, "t"), B.PolicyViolation), \
        "a name glued to Chinese text slipped through"
    assert refuses(lambda: B.scan_text({"en": "Alicetown is fine"}, names, "t")) is None, \
        "a name inside a longer word is not that name"
    assert refuses(lambda: B.scan_text({"en": "PI: Prof. Kuan-Ting Lai"}, names, "t")) is None
    with tempfile.TemporaryDirectory() as d:
        deny = Path(d) / "deny.txt"
        deny.write_text("Carol Example\nKuan-Ting Lai\n", encoding="utf-8")
        got = B.person_names(deny)
    assert {"carol", "example"} <= got, "the local deny list was not read"
    assert not ({"kuan", "ting", "lai", "kuan-ting"} & got), "the PI must stay nameable"


def test_simplified_characters_are_refused_in_chinese_text_only():
    assert refuses(lambda: B.scan_text({"zh": "这是问题"}, set(), "t"), B.PolicyViolation)
    assert refuses(lambda: B.scan_text({"zh": "這是問題"}, set(), "t")) is None
    assert refuses(lambda: B.scan_page("zh/x.html", "<p>飞机</p>", set()), B.PolicyViolation)
    assert refuses(lambda: B.scan_page("index.html", "<p>飞机</p>", set()), B.PolicyViolation), \
        "the bilingual chooser page was not checked"


def test_deny_list_names_in_chinese_characters_and_short_names_are_refused():
    """A colleague's name on the Traditional Chinese pages is most likely
    written in Chinese characters, and a romanized name can be short."""
    with tempfile.TemporaryDirectory() as d:
        deny = Path(d) / "deny.txt"
        deny.write_text("王小明\nIan Lin\nBo Li\n陳大文 (Tai-man Chan)\nKuan-Ting Lai\n", encoding="utf-8")
        names = B.person_names(deny)
    for text in ("本頁由王小明撰寫", "Written by Ian Lin", "感謝 Bo Li", "感謝 Bo  Li 的協助",
                 "Lin's script", "由陳大文撰寫", "Tai-man wrote it"):
        assert refuses(lambda: B.scan_text({"zh": text}, names, "t"), B.PolicyViolation), \
            f"accepted {text!r} with {sorted(names)}"
    for text in ("a linear path", "Bolivia", "PI: Prof. Kuan-Ting Lai", "小明顯的差異"):
        msg = refuses(lambda: B.scan_text({"en": text}, names, "t"))
        assert msg is None, f"refused {text!r}: {msg}"


def test_git_is_required_unless_the_build_is_told_otherwise():
    """Without git the commit-author check cannot run. That must stop the
    build, not switch the check off with a printed line."""
    real = B.git
    B.git = lambda *a: None
    try:
        assert refuses(lambda: B.person_names(None), B.BuildError)
        assert refuses(lambda: B.Media(), B.BuildError)
        content, extract = load_committed()
        assert refuses(lambda: B.build(content, extract), B.BuildError)
        # The explicit opt-out builds, for a local look only.
        assert B.person_names(None, require_git=False) == set()
        assert B.Media(require_git=False).tracked is None
    finally:
        B.git = real


def test_a_commit_author_from_git_is_refused_in_the_build():
    """The author list really comes from `git log` and really reaches the
    page scan: an invented author is injected and named in the content."""
    real = B.git

    def fake_git(*args):
        if args and args[0] == "log":
            return "Zed Quimby\x00zq@example.org\nDependabot\x0049699333+dependabot[bot]@users.noreply.github.com\n"
        return real(*args)

    B.git = fake_git
    try:
        names = B.person_names(None)
        assert {"zed", "quimby", "zed quimby"} <= names, sorted(names)
        content, extract = load_committed()
        ok, _ = B.build(content, extract)                     # the null: no author named
        assert ok
        c = copy.deepcopy(content)
        c["glossary"]["extra"][0]["zh"] += "（Quimby 撰寫）"
        msg = refuses(lambda: B.build(c, extract), B.PolicyViolation)
        assert msg and "Quimby" in msg, "a commit author's name was published"
    finally:
        B.git = real


def test_the_rendered_page_is_scanned_too():
    """The last line of defence: text that reached the HTML by any route."""
    assert refuses(lambda: B.scan_page("en/x.html", "<p>canonical-hil</p>", set()), B.PolicyViolation)
    assert refuses(lambda: B.scan_page("en/x.html", "<p>by Alice</p>", {"alice"}), B.PolicyViolation)
    assert refuses(lambda: B.scan_page("en/x.html", "<p>fine</p>", {"alice"})) is None


def test_build_scans_every_page_it_renders():
    """Not only scan_page itself: build() must call it on every page. A
    renderer is replaced so the bad text exists only in the HTML, never in
    site.json or the extract."""
    content, extract = load_committed()
    real = B.Site.page_glossary
    try:
        for html, names in (("<p>canonical-hil</p>", set()), ("<p>by Alice</p>", {"alice"}),
                            ("<p>本頁由王小明撰寫</p>", {"王小明"}), ("<p>飞机</p>", set())):
            B.Site.page_glossary = lambda self, lang, html=html: html
            assert refuses(lambda: B.build(content, extract, names=names), B.PolicyViolation), \
                f"build() published a page containing {html!r}"
        B.Site.page_glossary = lambda self, lang: "<p>fine</p>"
        pages, _ = B.build(content, extract, names={"alice"})      # the null
        assert pages["en/glossary.html"] == "<p>fine</p>"
    finally:
        B.Site.page_glossary = real


# --------------------------------------------------- numbers
def test_numbers_are_parsed_as_claims_not_names():
    n = B.numbers
    assert set(n("one check takes 0.13-0.26 ms (was 14-33 ms)")) == {"0.13", "0.26", "14", "33"}
    assert set(n("P0, WP1, Ed25519, OpenVLA-7B, ROS 2, MAVROS 2, 3D, 4-D")) == set()
    assert set(n("`--csp on` in `tools/x_2.py`")) == set()
    assert set(n("flown 6 Oct 2026, 2026-10-06, 2026 年 10 月 6 日")) == set()
    assert n("on 99.6-100 % of ticks") == {"99.6": True, "100": True}
    assert n("29/29 refused") == {"29": False}
    assert set(n("共7點，1408/1408")) == {"7", "1408"}, "CJK text does not glue to digits"


def _sources(text):
    return B.Sources(root=ROOT, changelog_text=text)


CHANGELOG_FIXTURE = """# Changelog
## [Unreleased]
### 2026-10-06 — work
- check 0.13-0.26 ms (was 14-33 ms); share 0.985.
### 2026-10-03 — older
- 42 flights.
## [0.5.1] — 2026-09-14
### Added
- 0.63 control.
"""


def test_a_number_must_appear_in_the_section_it_cites():
    src = _sources(CHANGELOG_FIXTURE)
    ok = {"changes": [{"changelog": "2026-10-06", "points": [
        {"en": "0.13-0.26 ms, was 14-33 ms", "zh": "0.13-0.26 ms，原為 14-33 ms"},
        {"en": "98.5 % of the mission", "zh": "98.5 % 的任務時間"}]}]}
    assert B.check_numbers(ok, src) == 5, "98.5 % must match the source's 0.985"
    bad = copy.deepcopy(ok)
    bad["changes"][0]["points"][0] = {"en": "42 flights", "zh": "42 次飛行"}
    msg = refuses(lambda: B.check_numbers(bad, src), B.BuildError)
    assert msg and "42" in msg, "a number from another section was accepted"
    nosrc = {"wps": {"WP1": {"done": [{"text": {"en": "7 rules", "zh": "7 條規則"}}]}}}
    assert refuses(lambda: B.check_numbers(nosrc, src), B.BuildError), "a number with no source"
    drift = {"x": {"en": "0.13 ms", "zh": "0.14 ms", "src": ["CHANGELOG.md#2026-10-06"]}}
    assert refuses(lambda: B.check_numbers(drift, src), B.BuildError), "en/zh disagree"
    tile = {"highlights": [{"value": "0.64", "src": ["CHANGELOG.md#0.5.1"]}]}
    assert refuses(lambda: B.check_numbers(tile, src), B.BuildError)
    tile["highlights"][0]["value"] = "0.63"
    assert B.check_numbers(tile, src) == 1


RETRACTED_FIXTURE = """# Changelog
## [Unreleased]
### 2026-10-06 — work
#### Added
- the check takes 0.26 ms.
#### Retracted
- the old 0.71 escape rate.
#### Found, not fixed
- 0.73 found.
#### Fixed (found while designing)
- 0.74 fixed.
## [0.5.1] — 2026-09-14
### Retracted
- the five control flights read 0.63.
#### detail
- 0.64 under a heading nested in the retraction.
### Still true
- 0.88 holds.
### Known limitations
- 0.91 open.
"""


def test_a_number_only_in_a_retracted_subsection_supports_nothing():
    """A figure that survives only in a Retracted (or Found, or Known
    limitations) list must not certify a published sentence."""
    src = _sources(RETRACTED_FIXTURE)

    def item(num, anchor):
        return {"x": {"en": f"reads {num}", "zh": f"為 {num}", "src": [f"CHANGELOG.md#{anchor}"]}}

    for num, anchor in (("0.71", "2026-10-06"), ("0.73", "2026-10-06"), ("0.63", "0.5.1"),
                        ("0.64", "0.5.1"), ("0.91", "0.5.1")):
        msg = refuses(lambda: B.check_numbers(item(num, anchor), src), B.BuildError)
        assert msg and num in msg, f"{num} was certified from a non-claim subsection of {anchor}"
    # The nulls: claims around them still count, including a heading after
    # the retraction and a 'Fixed (found ...)' heading.
    for num, anchor in (("0.26", "2026-10-06"), ("0.74", "2026-10-06"), ("0.88", "0.5.1")):
        assert B.check_numbers(item(num, anchor), src) == 1, f"{num} in a claim subsection was refused"
    tile = {"highlights": [{"value": "0.63", "src": ["CHANGELOG.md#0.5.1"]}]}
    assert refuses(lambda: B.check_numbers(tile, src), B.BuildError), "a tile used a retracted value"
    # The whole section is still there for anything that is not a number check.
    assert "0.71" in B.parse_changelog(RETRACTED_FIXTURE)["2026-10-06"]["text"]


def test_changelog_sections_newer_than_as_of_are_reported():
    secs = B.parse_changelog(CHANGELOG_FIXTURE + "### 2026-10-09 — later\n- x\n"
                             "## [0.6.0] — 2026-10-01 → 2026-10-08\n- y\n")
    assert secs["0.5.1"]["date"] == "2026-09-14" and secs["Unreleased"]["date"] is None
    assert secs["0.6.0"]["date"] == "2026-10-08", "a release range is dated by its end"
    assert B.sections_after(secs, "2026-10-06") == ["0.6.0", "2026-10-09"]
    assert B.sections_after(secs, "2026-10-09") == []
    # The build reports it (and `--check` fails on it); the suite does not,
    # so a later CHANGELOG entry by someone else never turns these tests red.
    _, stats = committed_build()
    assert isinstance(stats["changelog_sections_after_as_of"], list)


def test_tracker_text_must_carry_the_same_numbers_in_both_languages():
    content, extract = load_committed()
    B.check_parity(extract)                                   # the null: today's extract passes
    x = copy.deepcopy(extract)
    # A field no override replaces, or the override would hide the mismatch.
    assert not any(k.startswith(f"statuses.{x['statuses'][0]['id']}.") for k in content["overrides"])
    x["statuses"][0]["en"]["meaning"] = "a 3 m margin"
    x["statuses"][0]["zh"]["meaning"] = "4 m 邊距"
    assert refuses(lambda: B.check_parity(x), B.BuildError)
    msg = refuses(lambda: B.build(content, x, names=set()), B.BuildError)
    assert msg and "differ" in msg, "build() did not run the en/zh parity check on the extract"


def test_a_headline_tile_may_not_present_a_dev_run_as_a_kpi():
    """The grant measures every reported KPI on the hil topology. A tile on
    the overview that names an acceptance KPI must come from a hil run."""
    content, extract = load_committed()
    for label in ({"en": "P0 violation escape rate with the Shield on / off", "zh": "P0 違規逃逸率"},
                  {"en": "repair success", "zh": "修正成功率"},
                  {"en": "x", "zh": "任務成功率"}):
        c = copy.deepcopy(content)
        c["highlights"][0]["label"] = label
        assert refuses(lambda: B.check_highlights(c), B.PolicyViolation), f"accepted {label}"
        assert refuses(lambda: B.build(c, extract, names=set()), B.PolicyViolation)
        c["highlights"][0]["topology"] = "hil"
        assert refuses(lambda: B.check_highlights(c)) is None, "a hil tile was refused"
    B.check_highlights(content)                               # the real tiles pass


def test_changelog_sections_are_addressed_by_date_or_release():
    secs = B.parse_changelog(CHANGELOG_FIXTURE + "### 2026-09-29 (evening) — x\n- 7\n")
    assert {"2026-10-06", "2026-10-03", "0.5.1", "2026-09-29 (evening)", "Unreleased"} <= set(secs)
    assert "Added" not in secs, "an undated ### heading is part of its release, not a section"
    assert "0.63" in secs["0.5.1"]["text"]
    assert refuses(lambda: _sources(CHANGELOG_FIXTURE).text("CHANGELOG.md#2026-12-01"), B.BuildError)
    # A second heading with the same anchor extends the section; it must not
    # replace it, or every number cited from the first would stop verifying.
    twice = B.parse_changelog(CHANGELOG_FIXTURE + "### 2026-10-06 — more\n- 77 more.\n")
    assert "0.13-0.26" in twice["2026-10-06"]["text"] and "77" in twice["2026-10-06"]["text"]


# --------------------------------------------------- media
def test_videos_link_only_when_committed_and_small():
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        (root / "docs" / "video").mkdir(parents=True)
        (root / "docs" / "img").mkdir(parents=True)
        small, big = root / "docs/video/s.mp4", root / "docs/video/b.mp4"
        small.write_bytes(b"0" * 1000)
        big.write_bytes(b"0" * 2000)
        old = B.MAX_VIDEO_BYTES
        B.MAX_VIDEO_BYTES = 1500
        try:
            m = B.Media(root=root, tracked={"docs/video/s.mp4", "docs/video/b.mp4"})
            assert m.video("docs/video/s.mp4", 1000)["embed"] is True
            assert m.video("docs/video/b.mp4", 2000)["embed"] is False, "a large video was linked"
            m2 = B.Media(root=root, tracked=set())
            assert m2.video("docs/video/s.mp4", 1000)["embed"] is False, "an uncommitted video was linked"
            assert m2.video("docs/video/gone.mp4", 5)["embed"] is False
            assert refuses(lambda: m.video("docs/video/s.mp4", 999), B.BuildError), "size mismatch"
        finally:
            B.MAX_VIDEO_BYTES = old
        (root / "docs/img/a.png").write_bytes(b"x")
        assert refuses(lambda: B.Media(root=root, tracked=set()).image("docs/img/a.png"), B.BuildError)
        assert refuses(lambda: B.Media(root=root, tracked=set()).image("docs/img/none.png"), B.BuildError)
        assert B.Media(root=root, tracked={"docs/img/a.png"}).image("docs/img/a.png") == "docs/img/a.png"


def test_a_placeholder_names_the_file_and_its_size():
    pages, stats = committed_build()
    html = pages["en/gallery.html"]
    for name in stats["video_placeholders"]:
        assert re.search(re.escape(name) + r"</code> \(\d+\.\d MB\)", html), f"{name}: no name/size"
    assert "<video" not in html or stats["video_placeholders"] == []


def test_overrides_replace_one_field_and_refuse_unknown_paths():
    ov, fl = fake_tracker()
    x = B.extract_tracker(ov, fl, ["VLA"])
    B.apply_overrides(x, {"wps.WP1.plain": {"en": "E", "zh": "Z"}})
    assert x["wps"][0]["en"]["plain"] == "E" and x["wps"][0]["zh"]["plain"] == "Z"
    assert refuses(lambda: B.apply_overrides(x, {"wps.WP9.plain": {"en": "", "zh": ""}}), B.BuildError)
    assert refuses(lambda: B.apply_overrides(x, {"wps.WP1.status": {"en": "", "zh": ""}}), B.BuildError)
    # A plain (language-free) field such as the grant quote takes one string.
    B.apply_overrides(x, {"wps.WP1.grant_quote": "the whole sentence"})
    assert x["wps"][0]["grant_quote"] == "the whole sentence"
    assert refuses(lambda: B.apply_overrides(x, {"wps.WP1.grant_quote": {"en": "a", "zh": "b"}}),
                   B.BuildError), "a plain field took a language pair"
    assert refuses(lambda: B.apply_overrides(x, {"wps.WP1.plain": "one language only"}),
                   B.BuildError), "a text field took a single string"
    assert refuses(lambda: B.apply_overrides(x, {"wps.WP1.id": "WP9"}), B.BuildError), \
        "an override renamed the item"


def test_the_wp4_grant_quote_is_complete_and_names_hil():
    """The grant's sentence ends '...executed in the canonical HIL topology'.
    The site quotes all of it, with the grant's own word for that topology in
    brackets, instead of cutting it before the topology."""
    pages, _ = committed_build()
    for lg in B.LANGS:
        assert "comes from a Stress Testing run executed in the [hil] topology" in pages[f"{lg}/wp4.html"]


def test_a_code_path_that_does_not_exist_is_refused():
    content, extract = load_committed()
    c = copy.deepcopy(content)
    c["wps"]["WP1"]["done"][0]["files"].append("guardrail/no_such_module.py")
    assert refuses(lambda: B.build(c, extract, names=set()), B.BuildError)


def test_a_reproduce_command_must_name_a_script_that_exists():
    """A 'Reproduce' line that names a missing script tells the reader how to
    re-run a number that cannot be re-run."""
    content, extract = load_committed()
    assert B.check_files(content) > 0                         # the null: today's commands pass
    shown = content["wps"]["WP4"]["shown"]
    i = next(k for k, s in enumerate(shown) if s.get("command"))
    c = copy.deepcopy(content)
    c["wps"]["WP4"]["shown"][i]["command"] = "python tools/no_such_script.py --all"
    msg = refuses(lambda: B.build(c, extract, names=set()), B.BuildError)
    assert msg and "no_such_script.py" in msg, "a command naming a missing script was published"
    c["wps"]["WP4"]["shown"][i]["command"] = "make the report"
    assert refuses(lambda: B.check_files(c), B.BuildError), "a command with no script was accepted"


# --------------------------------------------------- precise sources
def _json_root(d):
    root = Path(d)
    (root / "data").mkdir()
    (root / "data" / "run.json").write_text(json.dumps({
        "runs": {"off": {"nfz_s": 3.7, "mean_time_to_safe_s": 4.1, "rate": 0.626506,
                         "rows": [{"n": 29}, {"n": None}]}}}), encoding="utf-8")
    return B.Sources(root=root, changelog_text=CHANGELOG_FIXTURE)


def _unit(en, zh, *src):
    return {"x": {"en": en, "zh": zh, "src": list(src)}}


def test_a_json_source_is_cited_by_the_path_of_its_value():
    """A data file holds hundreds of numbers, so 'the number is somewhere in
    the file' certifies almost anything: 3.7 s changed to 4.1 s passed,
    because 4.1 is another field of the same run."""
    with tempfile.TemporaryDirectory() as d:
        src = _json_root(d)
        at = "data/run.json#runs.off."
        assert B.check_numbers(_unit("3.7 s inside", "在區內 3.7 s", at + "nfz_s"), src) == 1
        msg = refuses(lambda: B.check_numbers(_unit("4.1 s inside", "在區內 4.1 s", at + "nfz_s"), src),
                      B.BuildError)
        assert msg and "4.1" in msg, "a number from another field of the cited file was accepted"
        msg = refuses(lambda: B.check_numbers(_unit("3.7 s", "3.7 s", "data/run.json"), src), B.BuildError)
        assert msg and "data/run.json#" in msg, "a whole data file was accepted as a source"
        assert refuses(lambda: B.check_numbers(_unit("3.7 s", "3.7 s", at + "no_such"), src), B.BuildError)
        assert B.check_numbers(_unit("29 of 29", "29 / 29", at + "rows.0"), src) == 1, "list index"
        assert refuses(lambda: B.check_numbers(_unit("0 runs", "0 次", at + "rows.1.n"), src),
                       B.BuildError), "a null value certified a zero"
        # A rounded figure matches the one value it rounds, and nothing near it.
        for right in ("0.6265", "62.7 %", "0.63"):
            assert B.check_numbers(_unit(right, right, at + "rate"), src) == 1, f"{right} refused"
        for wrong in ("0.6266", "62.6 %", "0.62"):
            u = _unit(wrong, wrong, at + "rate")
            assert refuses(lambda: B.check_numbers(u, src), B.BuildError), f"{wrong} was accepted"


def test_quantities_in_curated_text_are_written_as_digits():
    """'three shielded runs' changed to 'four' passed both the source check
    and the en/zh check, because neither sees a word."""
    src = _sources(CHANGELOG_FIXTURE)
    for en, zh in (("three shielded runs", "3 次"), ("3 runs", "三次飛行"),
                   ("Seven earlier flights", "7 次飛行"), ("the two missions", "兩條任務")):
        u = _unit(en, zh, "CHANGELOG.md#2026-10-06")
        msg = refuses(lambda: B.check_numbers(u, src), B.BuildError)
        assert msg and "as digits" in msg, f"accepted {en!r} / {zh!r}"
    # The null: words that are not counts stay allowed.
    ok = _unit("one check on the same mission, both runs, a second look",
               "同一條任務，兩者皆是，第二次檢查", "CHANGELOG.md#2026-10-06")
    assert B.check_numbers(ok, src) == 0
    content, _ = load_committed()
    B.check_numbers(content, B.Sources())                     # the real curated text passes


# --------------------------------------------------- images
def test_every_image_is_reviewed_and_unchanged():
    """The text scan cannot read an image. A figure that restated a corrected
    claim, or showed a different run than its caption, was published because
    nothing looked at it. Now each image carries the digest of the file a
    person reviewed, and a changed file is refused until reviewed again."""
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        (root / "docs" / "img").mkdir(parents=True)
        img = root / "docs" / "img" / "a.png"
        img.write_bytes(b"first picture")
        m = B.Media(root=root, tracked={"docs/img/a.png"})
        m.reviewed = {"docs/img/a.png": B.image_digest(b"first picture")}
        assert m.image("docs/img/a.png") == "docs/img/a.png"          # the null
        img.write_bytes(b"another picture under the same name")
        msg = refuses(lambda: m.image("docs/img/a.png"), B.PolicyViolation)
        assert msg and "changed since it was reviewed" in msg
        m.reviewed = {}
        assert refuses(lambda: m.image("docs/img/a.png"), B.PolicyViolation), "an unreviewed image"
        # The site's own copies are committed with the pages, not tracked yet,
        # and still need a review.
        own = root / "docs" / "progress" / "assets" / "img" / "b.png"
        own.parent.mkdir(parents=True)
        own.write_bytes(b"b")
        m2 = B.Media(root=root, tracked=set())
        m2.reviewed = {"docs/progress/assets/img/b.png": B.image_digest(b"b")}
        assert m2.image("docs/progress/assets/img/b.png")
        m2.reviewed = {}
        assert refuses(lambda: m2.image("docs/progress/assets/img/b.png"), B.PolicyViolation)
    content, extract = load_committed()
    c = copy.deepcopy(content)
    c["reviewed_images"].pop(c["hud_image"])
    assert refuses(lambda: B.build(c, extract, names=set()), B.PolicyViolation)
    c = copy.deepcopy(content)
    c["reviewed_images"]["docs/img/flowchart_tracking_old.png"] = "sha256:0"
    msg = refuses(lambda: B.build(c, extract, names=set()), B.BuildError)
    assert msg and "no page shows" in msg, "a stale review entry was kept"
    # The two figures withdrawn from the site stay off it: one repeats wording
    # the project corrected, the other comes from a different run set.
    pages, _ = committed_build()
    for gone in ("flowchart_system.png", "fig2_simultaneity.png", "hil_nfz_off_on.png"):
        assert not any(gone in h for h in pages.values()), f"{gone} is on a page"


# --------------------------------------------------- the build end to end
SENTINEL = "RAW TRACKER WORDING"


def _raw_tracker_text(extract, overrides):
    """The extract with every overridden field set back to text the tracker
    could hold, so a build that skips the overrides shows it."""
    x = copy.deepcopy(extract)
    for spec, val in overrides.items():
        section, _, rest = spec.partition(".")
        ident, _, field = rest.rpartition(".")
        items = x[section]
        for t in ([items] if isinstance(items, dict) else [i for i in items if i["id"] == ident]):
            if isinstance(val, dict):
                for lg in B.LANGS:
                    t[lg][field] = f"{SENTINEL} {lg}"
            else:
                t[field] = SENTINEL
    return x


def _tracker_files(extract):
    """Tracker data files that the allow-list turns back into `extract`."""
    files = {"content_overview.json": {}, "content_flow.json": {}}
    for name, (fname, key, id_field, plain, text) in B.ALLOW_LIST.items():
        items = extract[name]
        if isinstance(items, dict):
            files[fname][key] = {"i18n": {lg: dict(items[lg]) for lg in B.LANGS}}
            continue
        files[fname][key] = [
            {id_field: it["id"], **{p: it[p] for p in plain},
             "i18n": {lg: (it[lg]["text"] if text is None else dict(it[lg])) for lg in B.LANGS}}
            for it in items]
    return files


def run_main(args):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = B.main(args)
    return rc, buf.getvalue()


def test_overrides_reach_every_page_and_the_snapshot():
    """The committed snapshot already holds the overridden wording, so a
    build that skipped the overrides passed every other test, while a build
    from the real tracker data would have published the tracker's wording
    (for example that the GeoFence 'stays configured', which is a Next item)."""
    content, extract = load_committed()
    overrides = content["overrides"]
    assert overrides
    raw = _raw_tracker_text(extract, overrides)
    pages, _ = B.build(content, raw, names=set())
    leaked = sorted(rel for rel, h in pages.items() if SENTINEL in h)
    assert not leaked, f"tracker wording an override replaces is on {leaked}"
    every = "".join(pages.values())
    for spec, val in overrides.items():
        for text in ([val[lg] for lg in B.LANGS] if isinstance(val, dict) else [val]):
            assert B.inline(text) in every, f"override {spec} is on no page"
    # The same through main(), from tracker data files: the pages and the
    # snapshot it writes for the repository both carry the public wording.
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        tracker = d / "tracker"
        tracker.mkdir()
        for fname, data in _tracker_files(raw).items():
            (tracker / fname).write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
        out = d / "out"
        rc, log = run_main(["--out", str(out), "--tracker-dir", str(tracker)])
        assert rc == 0, log
        assert "tracker text from tracker" in log
        snap = (out / "_src" / B.SNAPSHOT_FILE.name).read_text(encoding="utf-8")
        assert SENTINEL not in snap, "the snapshot was written without the overrides"
        assert B.read_json(out / "_src" / B.SNAPSHOT_FILE.name) == B.public_extract(raw, content)
        written = [p for p in out.rglob("*.html") if SENTINEL in p.read_text(encoding="utf-8")]
        assert not written, f"tracker wording on {written}"


def test_check_fails_on_an_edited_page_and_on_a_grown_changelog_section():
    """`--check` is the pre-publish gate. It compared section dates only, so
    new work written under an existing dated heading (the usual way a day's
    entry grows) left a stale site reported as up to date."""
    content, _ = load_committed()
    anchor = content["changes"][0]["changelog"]
    original = B.CHANGELOG.read_bytes()
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        log = d / "CHANGELOG.md"
        log.write_bytes(original)
        out = d / "out"
        args = ["--out", str(out), "--tracker-dir", str(d / "no_tracker_here"), "--changelog", str(log)]
        rc, text = run_main(args)
        assert rc == 0, text
        rc, text = run_main(["--check", *args])
        assert "changelog changed" not in text and "stale:" not in text, text
        n = len(B.PAGES) * len(B.LANGS) + 1
        assert f"{n}/{n} pages up to date" in text
        # 1. New work under the existing heading of a cited section.
        lines = original.decode("utf-8").splitlines(keepends=True)
        at = next(i for i, ln in enumerate(lines)
                  if re.match(r"#{2,3} \[?" + re.escape(anchor) + r"\]?( |$)", ln))
        lines.insert(at + 1, "- A policy-layer feature landed today.\n")
        log.write_bytes("".join(lines).encode("utf-8"))
        rc, text = run_main(["--check", *args])
        assert rc == 1 and f"changelog changed: {anchor}" in text, text
        # 2. The CHANGELOG as built, and a hand-edited page.
        log.write_bytes(original)
        page = out / "en" / "index.html"
        with open(page, "a", encoding="utf-8", newline="\n") as fh:
            fh.write("<!-- edited by hand -->\n")
        rc, text = run_main(["--check", *args])
        assert rc == 1 and "stale: en/index.html" in text and "changelog changed" not in text, text


def test_a_page_the_generator_no_longer_makes_is_removed():
    with tempfile.TemporaryDirectory() as d:
        out = Path(d)
        (out / "en").mkdir()
        orphan = out / "en" / "old.html"
        orphan.write_text("<p>old</p>", encoding="utf-8")
        other = out / "en" / "notes.txt"
        other.write_text("x", encoding="utf-8")
        B.write_pages({"index.html": "<p>i</p>", "en/index.html": "<p>e</p>"}, out)
        assert not orphan.exists(), "a page the generator no longer makes stayed on the site"
        assert other.exists() and (out / "en" / "index.html").is_file()


def test_the_repository_owner_named_in_the_remote_is_refused():
    real = B.git

    def fake_git(*args):
        if args and args[0] == "remote":
            return ("origin\thttps://github.com/Zed99/vla.git (fetch)\n"
                    "origin\tgit@github.com:Zed99/vla.git (push)\n")
        if args and args[0] == "log":
            return "Kuan-Ting Lai\x00lai@example.org\n"
        return real(*args)

    B.git = fake_git
    try:
        assert "zed99" in B.person_names(None)
        content, extract = load_committed()
        ok, _ = B.build(content, extract)                     # the null
        assert ok
        c = copy.deepcopy(content)
        c["glossary"]["extra"][0]["en"] += " Maintained by Zed99."
        msg = refuses(lambda: B.build(c, extract), B.PolicyViolation)
        assert msg and "Zed99" in msg, "the repository owner's handle was published"
    finally:
        B.git = real


def test_the_pi_exemption_needs_every_word_of_his_name():
    """Only the PI's own name is exempt. An author who shares one word with
    it (here 'Ting') is still a name the site must not carry."""
    assert B.name_parts("Kuan-Ting Lai") == set()
    other = B.name_parts("Yu Ting Chen")
    assert {"chen", "yu ting chen"} <= other, other
    assert refuses(lambda: B.scan_text({"en": "scripts by Chen"}, other, "t"), B.PolicyViolation)
    assert refuses(lambda: B.scan_text({"en": "PI: Prof. Kuan-Ting Lai"}, other, "t")) is None


# --------------------------------------------------- the real site
VOID = {"area", "base", "br", "col", "embed", "hr", "img", "input", "link", "meta", "source", "track", "wbr"}


class Structure(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.stack, self.errors, self.ids, self.links = [], [], [], []
        self.h1 = 0
        self.lang = None
        self.title = ""
        self._in_title = False
        self.headings, self._in_h = [], None

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if tag == "html":
            self.lang = a.get("lang")
        if tag == "h1":
            self.h1 += 1
        if tag in ("h1", "h2", "h3"):
            self._in_h = [tag, ""]
        if tag == "title":
            self._in_title = True
        if "id" in a:
            self.ids.append(a["id"])
        for k in ("href", "src", "poster"):
            if a.get(k):
                self.links.append(a[k])
        for u in re.findall(r"url\('([^']+)'\)", a.get("style", "")):
            self.links.append(u)
        if tag not in VOID:
            self.stack.append(tag)

    def handle_startendtag(self, tag, attrs):
        if "id" in dict(attrs):
            self.ids.append(dict(attrs)["id"])

    def handle_endtag(self, tag):
        if tag in ("h1", "h2", "h3") and self._in_h:
            self.headings.append(self._in_h[1].strip())
            self._in_h = None
        if tag == "title":
            self._in_title = False
        if not self.stack or self.stack[-1] != tag:
            self.errors.append(f"</{tag}> closes <{self.stack[-1] if self.stack else None}>")
            return
        self.stack.pop()

    def handle_data(self, data):
        if self._in_title:
            self.title += data
        if self._in_h:
            self._in_h[1] += data


def parse(html):
    p = Structure()
    p.feed(html)
    p.close()
    return p


def test_the_real_site_builds_every_page_in_both_languages():
    pages, stats = committed_build()
    want = {"index.html"} | {f"{lg}/{p}.html" for lg in B.LANGS for p in B.PAGES}
    assert set(pages) == want, f"missing {want - set(pages)}, extra {set(pages) - want}"
    assert stats["numbers_checked"] > 50, "the number check ran on almost nothing"


def test_every_page_is_well_formed_and_titled():
    pages, _ = committed_build()
    for rel, html in pages.items():
        p = parse(html)
        assert not p.errors, f"{rel}: {p.errors[:3]}"
        assert not p.stack, f"{rel}: unclosed {p.stack}"
        assert p.title.strip(), f"{rel}: no <title>"
        want_lang = "en" if rel == "index.html" or rel.startswith("en/") else "zh-Hant-TW"
        assert p.lang == want_lang, f"{rel}: lang={p.lang}"
        if rel != "index.html":
            assert p.h1 == 1, f"{rel}: {p.h1} <h1>"
        dup = sorted({i for i in p.ids if p.ids.count(i) > 1})
        assert not dup, f"{rel}: duplicate ids {dup}"


def test_every_local_link_and_image_resolves():
    pages, _ = committed_build()
    ids = {rel: set(parse(html).ids) for rel, html in pages.items()}
    checked = 0
    for rel, html in pages.items():
        here = (SITE / rel).parent
        for link in parse(html).links:
            if re.match(r"(?i)(https?:|data:|mailto:)", link):
                continue
            path, _, frag = link.partition("#")
            target = (here / path).resolve() if path else (SITE / rel).resolve()
            try:
                site_rel = target.relative_to(SITE.resolve()).as_posix()
            except ValueError:
                site_rel = None
            if site_rel in pages:
                if frag:
                    assert frag in ids[site_rel], f"{rel}: #{frag} not in {site_rel}"
            else:
                assert target.is_file(), f"{rel}: {link} does not resolve"
            checked += 1
    assert checked > 300, f"only {checked} links checked"


def test_no_heading_lists_obstacles_and_open_work_is_labelled_next():
    pages, _ = committed_build()
    # "risk grading" is a CSP feature, so only the plural form counts.
    bad = re.compile(r"(?i)limitation|shortcoming|obstacle|blocker|known issue|\bgaps?\b|\brisks\b|"
                     r"hambatan|kendala|限制|風險(?!分級)|障礙|缺口")
    for rel, html in pages.items():
        for h in parse(html).headings:
            assert not bad.search(h), f"{rel}: heading {h!r}"
    for lg in B.LANGS:
        for k in B.WP_KEYS:
            html = pages[f"{lg}/{k.lower()}.html"]
            block = html.split('<ul class="next-list">', 1)[1].split("</ul>", 1)[0]
            items = block.count("<li>")
            assert items >= 1, f"{lg}/{k}: no next items"
            assert block.count('class="pill next"') == items, f"{lg}/{k}: a next item without the Next label"


def test_the_build_is_deterministic():
    content, extract = load_committed()
    a, _ = B.build(content, extract)
    b, _ = B.build(copy.deepcopy(content), copy.deepcopy(extract))
    assert a == b


def test_the_committed_pages_match_a_fresh_build():
    """Generated pages are never edited by hand: what is on disk is exactly
    what the generator makes from the committed sources."""
    pages, _ = committed_build()
    stale = [rel for rel, html in pages.items()
             if not (SITE / rel).is_file() or (SITE / rel).read_text(encoding="utf-8") != html]
    assert not stale, f"rebuild with python tools/build_public_site.py: {stale}"


def test_the_committed_snapshot_holds_only_allow_listed_fields():
    snap = B.read_json(B.SNAPSHOT_FILE)
    B.assert_no_disallowed_keys(snap, "snapshot")
    # The snapshot sits in the public repository, so it must pass the same
    # text scan as the pages (it is stored with the public overrides applied).
    B.scan_text(snap, B.person_names(), "snapshot")
    for name, items in snap.items():
        spec = B.ALLOW_LIST[name]
        allowed = {"id", "en", "zh", *spec[3]}
        for it in (items if isinstance(items, list) else [items]):
            assert set(it) <= allowed, f"{name}: {set(it) - allowed}"
            text_fields = set(spec[4]) if spec[4] else {"text"}
            for lang in B.LANGS:
                assert set(it[lang]) == text_fields, f"{name}.{lang}: {set(it[lang])}"


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
