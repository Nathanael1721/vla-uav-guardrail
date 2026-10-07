"""tools/deck/build_progress_oct_data.py: the numbers and the claims guard of the
October 2026 ITRI progress deck.

Run either way:
    pytest tests/test_build_progress_oct_data.py -v
    python tests/test_build_progress_oct_data.py

WHY THIS FILE EXISTS

The deck goes to ITRI with the PI's rule: progress-focused, but every statement
true and no retracted claim repeated. The builder enforces that with
`withdrawn_claims` / `check_deck` / `check_cover`, run on the deck's own text
and on the README cover note before the deck is written, and with `verify`,
which refuses a deck built from numbers the repository no longer produces. A
guard that never says no is not a guard (CONTRIBUTING.md), so:

* every claim this project has retracted (CHANGELOG.md "Retracted" sections,
  29 Sept - 6 Oct), and the paraphrases a review slipped past the first guard
  ("What the VLA reads: ...", "measured on an Orin"), must be refused;
* the 30 Sept deck builder's source as committed at 401305a, which says
  "canonical-hil" and "KPI-grade" (audit card C-01), must be refused - the
  failing-first case: the guard would have stopped that slide;
* true sentences from the October deck, including plans in the future tense,
  must pass, or the guard would be switched off the first time it got in the way.

The fact functions are tested on inputs whose answer is known and differs from
the total, so a function that copies the total, reads the wrong key, or types
a default in by hand fails: a WP1 file with 28 of 29 refused, a compiler
report whose P0 count, empty-CSP null and naive-cut null all differ, a
synthetic flight with a known closest approach, altitude band and estimate
share, and a longer one whose answers depend on its later ticks. Each score is
checked beside its null (a hover drone, an estimate fixed at the car's start).

On 7 Oct the Shield bench refused the tree's own code, and a review draft was
still needed: `measure_bench` keeps the stored runs only when asked and
records why, `verify_status` lets a draft through only when nothing but the
Shield timing is of other code, and `profile_facts` (the replay of real
flight ticks that slide 6 now quotes beside the synthetic benchmark) reads the
design-load arm and refuses a profile that distrusts itself.

Two tests tie the stored files to the code (7 Oct review, where six mutants
of the fact functions survived every test that read the stored JSON): the
stored sections must be what the fact functions compute now, and the built
pack must carry the hashes of the builder and data it was built from. Tests
that need local artefacts (gitignored) print SKIP, never PASS.
"""
import dataclasses
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools" / "deck"))

import build_progress_oct_data as P                                 # noqa: E402

SKIP = "SKIP"

RETRACTED = [
    "Our KPI runs used the canonical HIL rail.",
    "WP3 Safety Shield: KPI-grade (canonical-hil, 1 Sept)",
    "Hard KPI: P0 violation escape rate = 0",
    "Detector rate in flight 3.76-5.22 Hz",
    "the detector ran at 6.9-7.5 Hz on the red car",
    "Fail-safe trigger correctness target 1.0",
    "11713/11713 repairs converged",
    "P0 escape 0.0 on all 41 shielded flights",
    "P0 escape rate = 0.0 is the contract KPI",
    "All five KPIs measured on the dev rail",
    "The five ROS 2 runs are KPI-grade.",
    "Gazebo: no artefacts in the repository",
    "Gazebo has never been run",
    "In the city clip the Shield avoided the no-fly zone.",
    "The demos may use a hand-written pilot: the backend is swappable by design.",
    "1 of 34 camera flights meet both rate gates",
    "Replay 22.4 % to 95.5 % on the car",
    "The CSP reaches the VLA in every flight.",
    "OpenVLA reads the CSP before each decision.",
    "OpenVLA flies the city demo.",
    "Shield check measured on the Jetson Orin: 0.3 ms",
    # Slipped past the first guard (review, 6 Oct).
    "What the VLA reads: the city no-fly-zone policy",
    "OpenVLA follows the constraint summary pack.",
    "Shield timing measured on an Orin",
    # CHANGELOG 2026-10-05, Retracted.
    "The P0-escape count on screen is the contract KPI.",
    "Most compliance is done by a pilot that knows the rules, and the Shield is "
    "the backstop, exactly as the contract designs it.",
    # Paraphrases that got past the guard in the 7 Oct review: past tense
    # excused by "re-" or a bare "be ", and wordings outside every pattern.
    "The Shield timings were re-measured on the Jetson Orin: 0.4 ms.",
    "Both figures can be trusted: measured on the Orin.",
    "These are contract KPI numbers.",
    "The Shield steered the drone round the no-fly zone.",
    "Fail-safe trigger correctness: 100 % target.",
    "P0 escape rate 0 on 41 flights, the contract KPI.",
    "The CSP was in OpenVLA's prompt on every flight.",
    # Past-tense claims the CSP rule's old excuse words ("can ", "first ",
    # "planned") let through (7 Oct).
    "We can confirm the CSP reached OpenVLA on every flight.",
    "On the first flight OpenVLA followed the CSP.",
    "As planned, OpenVLA followed the CSP.",
]

TRUE_SENTENCES = [
    "The same benchmark will be repeated on the Jetson Orin in the hil campaign.",
    "Acceptance KPIs: mission success rate · P0 violation escape rate (target 0) · "
    "fail-safe trigger correctness · mean repair magnitude · mean time to safe",
    "Typed 14-field CSP; 105/105 P0 rules carried within a 256-token budget; OpenVLA prompt path",
    "The follow controller routed beside the zone (banner: AVOID) while the Safety "
    "Shield checked every command.",
    "Everything on a single desktop is the grant's dev topology.",
    "real_vla_demo.py --csp on places this text before OpenVLA's instruction.",
    "Fail-safe trigger correctness beside always- and never-trigger baselines; "
    "target 99 % or better.",
    "Recomputed over the mission, the detector ran at 2.77-4.31 Hz.",
    "VLA reading the CSP on the ArduPilot rail",
    "ACTING: the Shield repairs a command",
    "Rule check at the grant's 50-rule load: 87 ms to 0.35 ms",
    "The rule summary compiled for the VLA prompt: city no-fly-zone policy",
    "The P0-escape counter under the panel uses the same definition as the KPI code.",
    "A flight in Project AirSim is not a contract KPI figure.",
    # Slide 2's lead line, labelled as the grant's design.
    "The grant's design: a declarative flight policy is compiled into the VLA's "
    "prompt, and a Safety Shield checks every 10 Hz command against the same "
    "policy before ArduPilot flies it.",
    "dev: Development and testing. hil: the grant's KPI configuration.",
    "The status words separate the pilot steering round a rule (AVOID) from the "
    "Shield correcting a command (ACTING).",
    # Plans and negations: each matches a withdrawn-claim pattern and is
    # excused only by its future word or its "not", so a guard that ignored
    # tense or negation would refuse them.
    "Shield timing will be measured on the Jetson Orin in the hil campaign.",
    "Next, OpenVLA follows the CSP in the flights planned for November.",
    "These are development measurements, not contract KPI figures.",
]


def _slides(n=10, **over):
    out = [{"n": i + 1, "title": f"Slide {i + 1}", "text": "Progress.", "notes": "A note."}
           for i in range(n)]
    for k, v in over.items():
        i, field = k.split("_", 1)
        out[int(i[1:]) - 1][field] = v
    return out


# --------------------------------------------------------------------------- #
# The claims guard
# --------------------------------------------------------------------------- #

def test_every_retracted_claim_is_refused():
    missed = [s for s in RETRACTED if not P.withdrawn_claims(s)]
    assert not missed, f"guard let these through: {missed}"


def test_true_sentences_and_plans_pass():
    hits = {s: P.withdrawn_claims(s) for s in TRUE_SENTENCES}
    bad = {s: h for s, h in hits.items() if h}
    assert not bad, f"guard refused true sentences: {bad}"


def _committed(path: str, rev: str) -> str | None:
    """A file as committed at `rev`, or None without git or the revision."""
    try:
        p = subprocess.run(["git", "show", f"{rev}:{path}"], cwd=ROOT,
                           capture_output=True, text=True, encoding="utf-8")
    except FileNotFoundError:
        return None
    return p.stdout if p.returncode == 0 else None


# The 30 Sept builder as committed on 6 Oct, before its wording was corrected.
# Pinned to the commit, not the working tree: the correction removes exactly
# the text this test needs, and the test must not depend on whether it landed.
OLD_DECK_REV = "401305a"


def test_guard_refuses_the_30_sept_deck_source():
    """Failing first: the 30 Sept builder carried the 'canonical-hil' /
    'KPI-grade' slide (audit C-01). The October guard must stop it."""
    src = _committed("tools/deck/build_progress_0930_deck.js", OLD_DECK_REV)
    if src is None:
        return SKIP
    hits = P.withdrawn_claims(src)
    claims = " | ".join(h["claim"].lower() for h in hits)
    assert "canonical-hil" in claims and "kpi-grade" in claims, claims


def test_a_plan_word_far_from_the_claim_does_not_excuse_it():
    """The excuse word must sit just before the claim. Here 'will be ' is one
    of the Orin rule's own excuse words, about 50 characters before the claim:
    a window widened past that (the 7 Oct review tried 200) lets it through.
    The previous version of this test used 'will ', which that rule did not
    list at all, so it passed whatever the window."""
    s = "The Orin will be set up next month; the Shield check was measured on the Jetson Orin: 0.3 ms."
    far = s.index("measured") - s.index("will be ")
    assert P.EXCUSE_WINDOW < far < 200, far
    assert "will be " in next(f for p, _, f in P.WITHDRAWN if "orin" in p)
    assert P.withdrawn_claims(s), f"a 'will be' {far} characters earlier excused a claim"
    near = "The Shield check will be measured on the Jetson Orin."
    assert not P.withdrawn_claims(near), "the same words right before the claim did not excuse it"


def test_check_deck_flags_count_title_and_missing_notes():
    assert P.check_deck(_slides(10)) == []
    assert any("9 slides" in p for p in P.check_deck(_slides(9)))
    assert any("15 slides" in p for p in P.check_deck(_slides(15)))
    probs = P.check_deck(_slides(10, s3_title="Limits"))
    assert any("shortcomings" in p for p in probs), probs
    probs = P.check_deck(_slides(10, s4_title="Known issues and risks"))
    assert any("slide 4" in p for p in probs), probs
    probs = P.check_deck(_slides(10, s5_notes="  "))
    assert any("no speaker note" in p for p in probs), probs
    probs = P.check_deck(_slides(10, s6_notes="This is the canonical HIL rail."))
    assert any("slide 6 notes" in p for p in probs), probs


def test_check_deck_refuses_a_number_the_template_could_not_fill():
    """A JS template prints a missing value as 'undefined', 'NaN' or 'null'.
    The notes use 'null' as a word, so only a null where a number goes counts."""
    for text in ("car within 30 m NaN %", "closest undefined m", "refused null / 29",
                 "29 / null tampered bundles", "check Infinity ms"):
        probs = P.check_deck(_slides(10, s2_text=text))
        assert any("missing number" in p for p in probs), (text, probs)
    for text in ("The null on the right: a reader that verifies nothing.",
                 "against a hover null of 10.9 %", "a copy-only check refuses 0"):
        assert P.check_deck(_slides(10, s2_notes=text)) == [], text


def test_check_deck_cli_exit_codes():
    d = Path(tempfile.mkdtemp())
    good, bad = d / "good.json", d / "bad.json"
    good.write_text(json.dumps(_slides(12)), encoding="utf-8")
    bad.write_text(json.dumps(_slides(12, s2_text="Hard KPI met.")), encoding="utf-8")
    assert P.main(["--check-deck", str(good)]) == 0
    assert P.main(["--check-deck", str(bad)]) == 1


README = """# Share pack

## What to send
Text.

## Cover note (paste into the email)

> Dear [name],
> {body}
> Best regards,
> [sender]

## How it was built
Our KPI runs used the canonical HIL rail.
"""


def test_the_cover_note_is_checked_and_only_the_cover_note():
    clean = README.format(body="A rule check takes 0.42 ms on a desktop CPU.")
    assert P.check_cover(clean) == [], "the claim below the block was checked as cover note"
    for body in ("The Shield avoided the no-fly zone.", "Shield timing measured on an Orin.",
                 "Tamper refusal undefined / 29."):
        probs = P.check_cover(README.format(body=body))
        assert probs and all(p.startswith("cover note") for p in probs), (body, probs)
    assert P.check_cover("# README without the block\n"), "a missing block read as clean"
    d = Path(tempfile.mkdtemp())
    (d / "s.json").write_text(json.dumps(_slides(12)), encoding="utf-8")
    (d / "R.md").write_text(README.format(body="The CSP reaches the VLA."), encoding="utf-8")
    assert P.main(["--check-deck", str(d / "s.json"), "--cover", str(d / "R.md")]) == 1


# --------------------------------------------------------------------------- #
# Release record
# --------------------------------------------------------------------------- #

CL = """## [Unreleased]

### 2026-10-06 — ten packages

#### Tests
- Full suite on Python 3.10: 1408 of 1408 over 53 test
  files. On 3.11 the
  guardrail package's tests pass; the demo tests cannot run there.

### 2026-10-05 — the audit
- Full suite on Python 3.10: 1 of 2 over 3 test files.
"""


def test_changelog_section_stops_at_the_next_heading():
    sec = P.changelog_section(CL, "2026-10-06")
    assert sec.startswith("### 2026-10-06") and "the audit" not in sec
    assert P.changelog_section(CL, "2026-09-01") is None


def test_suite_counts_reads_a_wrapped_line_and_never_invents_a_zero():
    got = P.suite_counts(P.changelog_section(CL, "2026-10-06"))
    assert got == {"python": "3.10", "passed": 1408, "total": 1408, "files": 53,
                   "guardrail_passes_on_311": True}, got
    assert P.suite_counts("### 2026-10-06\n- no suite line here") is None
    assert P.suite_counts(None) is None


def test_suite_counts_takes_the_last_result_of_the_day():
    """A second wave appends its own suite line to the same section; the
    first line is then out of date."""
    sec = P.changelog_section(CL, "2026-10-06") + (
        "\n- Second wave. Full suite on Python 3.10: 1500 of 1502 over 60 test files.\n")
    got = P.suite_counts(sec)
    assert (got["passed"], got["total"], got["files"]) == (1500, 1502, 60), got
    assert got["guardrail_passes_on_311"] is False, "the 3.11 clause of the first line was carried over"


def test_release_facts_dates_the_deck_by_the_newest_section():
    """The status date is the newest dated section, so work committed on
    7 Oct is not reported as 'delivered by 6 October'; the suite count is the
    newest one recorded, with the section it came from, or None, never 0."""
    later = CL.replace("## [Unreleased]\n", "## [Unreleased]\n\n### 2026-10-07 — the second wave\n- Shield fixes.\n", 1)
    got = P.release_facts(later)
    assert got["status_date"] == "2026-10-07" and got["sections"] == ["2026-10-07", "2026-10-06"], got
    assert got["section"].startswith("2026-10-07") and got["suite_section"] == "2026-10-06", got
    assert got["suite"]["passed"] == 1408, got
    own = later.replace("- Shield fixes.", "- Full suite on Python 3.10: 1501 of 1502 over 61 test files.")
    got = P.release_facts(own)
    assert (got["suite"]["passed"], got["suite"]["total"], got["suite_section"]) == (1501, 1502, "2026-10-07"), got
    # Sections before 2026-10-06 never count, and none at all stops the build.
    old_only = "## [Unreleased]\n\n### 2026-10-05 — the audit\n- Full suite on Python 3.10: 1 of 2 over 3 test files.\n"
    try:
        P.release_facts(old_only)
    except SystemExit:
        pass
    else:
        raise AssertionError("a changelog with nothing from 2026-10-06 on was accepted")
    got = P.release_facts(CL.replace("- Full suite on Python 3.10: 1408 of 1408 over 53 test\n  files.", "- no suite run"))
    assert got["suite"] is None and got["suite_section"] is None, "the 2026-10-05 suite line was carried forward"


# --------------------------------------------------------------------------- #
# WP1, WP2, WP3 FSM, paraphraser: the counts, not the totals
# --------------------------------------------------------------------------- #

def _wp1_doc(refused=28, run=29, null="3/29", neg="33/35"):
    return {"command": "python tools/wp1_roundtrip_kpi.py", "code_revision": "abc",
            "hash_scheme": "sha256-canonical-v3",
            "kpi": {"negative_refusal": neg},
            "round_trip": {"n": 29, "passed": 27, "signatures": {"verified": 26},
                           "tamper_refusal": {"run": run, "refused": refused}},
            "null_accept_everything": {"tamper_refusal": null, "negative_refusal": "1/35",
                                       "why": "w"}}


def test_wp1_facts_reads_each_count_not_the_total():
    d = Path(tempfile.mkdtemp())
    src, lock = d / "wp1.json", d / "lock.json"
    src.write_text(json.dumps(_wp1_doc()), encoding="utf-8")
    lock.write_text(json.dumps({"policies": [{}, {}]}), encoding="utf-8")
    f = P.wp1_facts(src, lock)
    got = {k: f[k] for k in ("policies", "round_trip_passed", "signatures_verified",
                             "tamper_run", "tamper_refused", "null_tamper_refused",
                             "null_tamper_run", "negative_refused", "negative_run",
                             "null_negative_refused", "lock_entries")}
    assert got == {"policies": 29, "round_trip_passed": 27, "signatures_verified": 26,
                   "tamper_run": 29, "tamper_refused": 28, "null_tamper_refused": 3,
                   "null_tamper_run": 29, "negative_refused": 33, "negative_run": 35,
                   "null_negative_refused": 1, "lock_entries": 2}, got
    # An older file without the broken-policy column: absent, never zero.
    doc = _wp1_doc()
    del doc["kpi"]
    src.write_text(json.dumps(doc), encoding="utf-8")
    f = P.wp1_facts(src, lock)
    assert f["negative_refused"] is None and f["null_negative_refused"] is None, f


def test_grant_csp_fields_counts_the_grant_list_only():
    model = list(P.GRANT_CSP_FIELDS) + ["policy_id", "selection", "a_field_added_later"]
    assert P.grant_csp_fields(model) == 14
    try:
        P.grant_csp_fields([f for f in model if f != "relevance_explanations"])
    except SystemExit as e:
        assert "relevance_explanations" in str(e)
    else:
        raise AssertionError("a CSP missing a locked field was counted")


def test_the_csp_model_carries_the_grant_fields():
    try:
        from guardrail import csp as C
    except ImportError:
        return SKIP
    assert P.grant_csp_fields(C.CSP.model_fields) == len(P.GRANT_CSP_FIELDS) == 14


EXAMPLE = {"policy": "policies/follow_car_citylife_nfz.yaml", "policy_id": "nfz",
           "rules": 6, "prompt": "Never enter zone 'z'."}


def _report(rows=None, exact="openvla-llama2-sp@x"):
    """A coverage report in which every count the slide uses differs from the
    others, so a mapping that reads the wrong key gets a wrong number."""
    rows = rows if rows is not None else [
        {"file": "policies/a.yaml", "tokens_used": 80, "tokens_exact": 61},
        {"file": "policies/follow_car_citylife_nfz.yaml", "tokens_used": 120, "tokens_exact": 93},
        {"file": "policies/b.yaml", "tokens_used": 40, "tokens_exact": 30},
    ]
    return {"command": "python -m guardrail.compiler report --budget 256 --exact",
            "code_revision": "abc", "budget_tokens": 256, "exact_counter": exact,
            "token_counter": "openvla-table-v1",
            "totals": {"n_policies": 4, "n_compiled": 3, "n_p0_in_scope": 11,
                       "n_p0_covered": 9, "null_n_p0_covered": 2,
                       "baseline_n_p0_covered": 5, "n_rules_in_scope": 17,
                       "n_rules_in_text": 16, "n_rules_in_csp": 15,
                       "n_rules_explained": 14, "n_dropped_budget": 1,
                       "kpi_discriminates": True, "max_tokens_used": 120},
            "rows": rows}


def test_wp2_from_report_reads_each_figure_from_its_own_key():
    """Slide 5's headline figures: P0 carried with the empty-CSP null and the
    naive-cut null beside it, OpenVLA's token range, the example's count. On
    the real report the compiler and the naive cut both carry 105 / 105, so
    only a report where they differ shows which key each figure came from."""
    model = list(P.GRANT_CSP_FIELDS) + ["policy_id", "selection"]
    f = P.wp2_from_report(_report(), model, (0.5, 0.3, 0.2), EXAMPLE)
    got = {k: f[k] for k in ("policies", "compiled", "p0_in_scope", "p0_covered",
                             "null_p0_covered", "baseline_p0_covered", "rules_in_scope",
                             "rules_in_text", "rules_in_csp", "rules_explained",
                             "dropped_for_budget", "kpi_discriminates",
                             "tokens_exact_range", "tokens_table_max", "budget_tokens",
                             "exact_counter", "table_counter", "csp_fields_locked",
                             "csp_fields_added", "risk_weights")}
    assert got == {"policies": 4, "compiled": 3, "p0_in_scope": 11, "p0_covered": 9,
                   "null_p0_covered": 2, "baseline_p0_covered": 5, "rules_in_scope": 17,
                   "rules_in_text": 16, "rules_in_csp": 15, "rules_explained": 14,
                   "dropped_for_budget": 1, "kpi_discriminates": True,
                   "tokens_exact_range": [30, 93], "tokens_table_max": 120,
                   "budget_tokens": 256, "exact_counter": "openvla-llama2-sp@x",
                   "table_counter": "openvla-table-v1", "csp_fields_locked": 14,
                   "csp_fields_added": ["policy_id", "selection"],
                   "risk_weights": [0.5, 0.3, 0.2]}, got
    assert f["example"] == {**EXAMPLE, "tokens_exact": 93}, f["example"]


def test_wp2_from_report_leaves_an_unmeasured_count_empty_never_zero():
    """Without the exact counter, or without the example's row, the slide must
    fall back to its other wording, not print '0-0 OpenVLA tokens'."""
    model = list(P.GRANT_CSP_FIELDS)
    rows = [{"file": "policies/a.yaml", "tokens_used": 80, "tokens_exact": None},
            {"file": "policies/follow_car_citylife_nfz.yaml", "tokens_used": 120,
             "tokens_exact": None}]
    f = P.wp2_from_report(_report(rows, exact=None), model, (0.5, 0.3, 0.2), EXAMPLE)
    assert f["tokens_exact_range"] is None and f["example"]["tokens_exact"] is None, f
    assert f["tokens_table_max"] == 120
    other = dict(EXAMPLE, policy="policies/not_in_the_report.yaml")
    f = P.wp2_from_report(_report(), model, (0.5, 0.3, 0.2), other)
    assert f["example"]["tokens_exact"] is None, f["example"]


def test_fsm_facts_are_the_fsm_defaults_field_by_field():
    try:
        from guardrail import fsm as F
    except ImportError:
        return SKIP
    c = F.FSMConfig()
    f = P.fsm_facts()
    reported = [fl.name for fl in dataclasses.fields(c) if fl.name in f]
    assert len(reported) >= 6, f"fsm_facts reports only {reported} of FSMConfig"
    for name in reported:
        assert f[name] == getattr(c, name), (name, f[name], getattr(c, name))
    assert f["states"] == [s.value for s in F.FSMState]
    d = _stored()
    if d is not None:
        assert d["wp3_fsm"] == f, "the stored FSM section is not the FSM's defaults"


def test_paraphraser_section_is_the_validation_report():
    rep = ROOT / "experiments" / "paraphrases" / "validation_report.json"
    if not rep.exists():
        return SKIP
    v = json.loads(rep.read_text(encoding="utf-8"))
    f = P.paraphraser_facts()
    assert (f["sources"], f["paraphrases"], f["accepted"]) == (v["sources"], v["paraphrases"], v["accepted"])
    assert (f["mutants"], f["mutants_refused"]) == (v["mutants"], v["mutants_refused"])
    assert f["mutation_classes"] == len(v["mutants_by_class"])
    assert f["null_copy_only_refused"] == v["null_validators"]["accept_all_but_exact_copy"]["mutants_refused"]
    assert f["per_source"] * f["sources"] == f["paraphrases"]
    d = _stored()
    if d is not None:
        assert d["paraphraser"] == json.loads(json.dumps(f))


# --------------------------------------------------------------------------- #
# Number helpers
# --------------------------------------------------------------------------- #

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


def test_point_in_poly_and_distance():
    assert P.point_in_poly(5, 5, SQUARE) and not P.point_in_poly(12, 5, SQUARE)
    assert P.dist_to_poly(5, 5, SQUARE) == 0.0
    assert abs(P.dist_to_poly(12, 5, SQUARE) - 2.0) < 1e-9
    assert abs(P.dist_to_poly(13, 14, SQUARE) - 5.0) < 1e-9     # to the corner


def test_zone_summary_tells_entering_from_passing():
    beside = [(x, -3.0) for x in range(-5, 16)]
    through = [(x, 5.0) for x in range(-5, 16)]
    a = P.zone_summary(beside, SQUARE)
    b = P.zone_summary(through, SQUARE)
    assert a["ticks_inside"] == 0 and abs(a["min_distance_m"] - 3.0) < 1e-9, a
    assert b["ticks_inside"] > 0 and b["min_distance_m"] == 0.0, b


def test_within_returns_none_without_a_car_and_scores_the_hover_null():
    drone = [(0.0, 0.0)] * 4
    assert P.within(drone, [None] * 4, 30.0) is None
    car = [(10.0, 0.0), (20.0, 0.0), (40.0, 0.0), None]
    assert abs(P.within(drone, car, 30.0) - 2 / 3) < 1e-9


def test_token_range_is_none_when_the_exact_counter_was_missing():
    assert P.token_range([{"tokens_exact": 54}, {"tokens_exact": 106}]) == [54, 106]
    assert P.token_range([{"tokens_exact": 54}, {"tokens_exact": None}]) is None
    assert P.token_range([]) is None


# A synthetic flight whose answers are known and differ from each null.
# Zone: the square x, y in 0..10. Car truth, estimate and drone per tick:
FLIGHT = [
    # t,   drone xy,     up,   car xy,       estimate xy
    (0.0, (-3.0, 5.0), 8.0, (-3.0, 20.0), (-3.0, 21.0)),    # car 15 m, est 1 m off
    (0.1, (-5.0, 5.0), 9.5, (-5.0, 40.0), (-5.0, 42.0)),    # car 35 m, est 2 m off
    (0.2, (-4.0, -4.0), 12.0, (-4.0, -30.0), (20.0, -30.0)),  # car 26 m, est 24 m off
    (0.3, (-3.0, 8.0), 7.5, (-3.0, 27.0), (-3.0, 57.0)),    # car 19 m, est 30 m off
]
# within 30 m: ticks 0, 2, 3 -> 0.75. Hover at (-3, 5): 15, 35.1, 35.0, 22 m -> 0.5.
# Estimate within 6 m: ticks 0, 1 -> 0.5. Fixed at the car's start (-3, 20):
# 0, 20.1, 50.0, 7 m -> 0.25. Closest to the zone: 3.0 m (ticks 0 and 3).
POLICY_YAML = """policy_id: synthetic-nfz
version: 0.1.0
constraints:
  - id: zone
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: repair
    vertices:
      - {x: 0, y: 0}
      - {x: 10, y: 0}
      - {x: 10, y: 10}
      - {x: 0, y: 10}
    altitude_floor_m: 0
    altitude_ceiling_m: 60
    margin_m: 1.0
  - id: clearance
    type: obstacle_clearance
    constraint_type: hard
    priority: P0
    violation_action: repair
    min_clearance_m: 3.0
    soft_margin_m: 2.0
  - id: band
    type: altitude_envelope
    constraint_type: hard
    priority: P0
    violation_action: repair
    alt_min_m: 6
    alt_max_m: 14
"""


def _flight_dir(flight=None, **metric_over):
    d = Path(tempfile.mkdtemp()) / "synthetic_flight"
    d.mkdir()
    rows = [{"t": t, "x": dx, "y": dy, "up": up, "truth": {"pts": [list(car)]}, "est_xy": list(est)}
            for t, (dx, dy), up, car, est in (flight or FLIGHT)]
    (d / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    m = {"object": "a red car", "frac_within_30m": 0.75, "nfz_entered": False, "nfz_s": 0.0,
         "estimate_on_subject": {"ticks": 4, "on_subject_frac": 0.5, "near_m": 6.0},
         "fence_aim_ticks": 2, "sep_mean_m": 23.75, "params": {"cruise_alt": 10.0},
         "collisions": {"mission": 0},
         "policy_hud": {"banner_ticks": {"AVOID": 1}, "counts": {"repaired": 0}}}
    for k, v in metric_over.items():
        if v is None:
            m.pop(k, None)
        else:
            m[k] = v
    (d / "metrics.json").write_text(json.dumps(m), encoding="utf-8")
    (d / "manifest.json").write_text(json.dumps({"topology": "dev"}), encoding="utf-8")
    pol = d.parent / "policy.yaml"
    pol.write_text(POLICY_YAML, encoding="utf-8")
    return d, pol


def _load_policy_available():
    try:
        from guardrail.models import load_policy                    # noqa: F401
        return True
    except ImportError:
        return False


def test_nfz_flight_on_a_synthetic_flight_with_known_answers():
    if not _load_policy_available():
        return SKIP
    d, pol = _flight_dir()
    f, track = P.nfz_flight(d, pol)
    assert f["zone_ticks_inside"] == 0 and abs(f["zone_min_distance_m"] - 3.0) < 1e-9, f
    assert (f["alt_min_m"], f["alt_max_m"]) == (7.5, 12.0), f
    assert f["within_30m"] == 0.75 and f["within_30m_recomputed"] == 0.75, f
    assert f["within_30m_null_hover"] == 0.5, f
    assert f["estimate_on_car"] == 0.5 and f["estimate_on_car_recomputed"] == 0.5, f
    assert f["estimate_null_fixed_start"] == 0.25, f
    assert (f["sep_mean_m"], f["cruise_alt_m"], f["fence_aim_ticks"]) == (23.75, 10.0, 2), f
    assert f["alt_band_m"] == [6, 14] and f["duration_s"] == 0.3 and f["ticks"] == 4, f
    assert len(track["car"]) == 4 and track["verts"] == SQUARE


# FLIGHT, then eleven ticks hovering beside the zone and a last tick INSIDE it,
# higher than any before: every answer depends on the ticks after the tenth.
LONG_FLIGHT = FLIGHT + [
    (round(0.4 + 0.1 * i, 1), (-3.0, 5.0), 8.0, (-3.0, 20.0), (-3.0, 21.0)) for i in range(11)
] + [(1.5, (5.0, 5.0), 13.0, (5.0, 20.0), (5.0, 21.0))]


def test_nfz_flight_reads_the_whole_log():
    """A review mutant scored only the first ten ticks and passed every test,
    because the synthetic flight above has four. Here the whole log, and only
    the whole log, gives: 16 ticks over 1.5 s; one tick inside the zone (the
    last); altitude up to 13 m; car within 30 m on 15 of 16 ticks (hover null
    14 of 16); estimate within 6 m on 14 of 16 (fixed-at-start null 12 of 16)."""
    if not _load_policy_available():
        return SKIP
    d, pol = _flight_dir(LONG_FLIGHT, frac_within_30m=15 / 16,
                         estimate_on_subject={"ticks": 16, "on_subject_frac": 14 / 16, "near_m": 6.0})
    f, track = P.nfz_flight(d, pol)
    assert (f["ticks"], f["duration_s"]) == (16, 1.5), f
    assert f["zone_ticks_inside"] == 1 and f["zone_min_distance_m"] == 0.0, f
    assert (f["alt_min_m"], f["alt_max_m"]) == (7.5, 13.0), f
    assert f["within_30m_recomputed"] == round(15 / 16, 3), f
    assert f["within_30m_null_hover"] == round(14 / 16, 3), f
    assert f["estimate_on_car_recomputed"] == round(14 / 16, 3), f
    assert f["estimate_null_fixed_start"] == round(12 / 16, 3), f
    assert len(track["drone"]) == 16 and len(track["car"]) == 16


def test_nfz_flight_refuses_a_missing_or_contradicted_figure():
    if not _load_policy_available():
        return SKIP
    cases = {"metrics without within-30 m": {"frac_within_30m": None},
             "metrics without fence-aim ticks": {"fence_aim_ticks": None},
             "within-30 m the log does not support": {"frac_within_30m": 0.4},
             "estimate share the log does not support": {
                 "estimate_on_subject": {"on_subject_frac": 1.0, "near_m": 6.0}}}
    let_through = []
    for name, over in cases.items():
        d, pol = _flight_dir(**over)
        try:
            P.nfz_flight(d, pol)
            let_through.append(name)
        except SystemExit:
            pass
    assert not let_through, f"quoted anyway: {let_through}"


def test_estimate_null_fixed_start_uses_the_estimate_ticks_only():
    rows = [{"truth": {"pts": [[0.0, 0.0]]}, "est_xy": None},          # no estimate served
            {"truth": {"pts": [[1.0, 0.0]]}, "est_xy": [1.0, 0.0]},
            {"truth": {"pts": [[20.0, 0.0]]}, "est_xy": [20.0, 0.0]},
            {"truth": None, "est_xy": [5.0, 5.0]}]                      # no truth logged
    assert P.estimate_on(rows, 6.0) == 1.0
    assert P.estimate_null_fixed_start(rows, 6.0) == 0.5
    assert P.estimate_on([{"truth": None, "est_xy": [0, 0]}], 6.0) is None
    assert P.estimate_null_fixed_start([{"truth": None}], 6.0) is None


# --------------------------------------------------------------------------- #
# The Shield benchmark
# --------------------------------------------------------------------------- #

def _bench_doc():
    stat = {"n": 10, "median_ms": 1.0, "p99_ms": 2.0, "max_ms": 3.0, "mean_ms": 1.0}
    keys = ("clear/_check", "near/_check", "near/filter", "grid/_check")
    return {"horizon_s": 5.0, "dt_s": 0.1, "rules": 50, "fences": 48,
            "budgets_ms": {"_check": 5.0, "filter": 100.0}, "failures": [],
            "results": {k: {"current": dict(stat), "legacy": dict(stat), "floor": dict(stat)}
                        for k in keys},
            "agreement": {k: {"identical": 10, "compared": 10} for k in keys},
            "sanity": {"clear/_check": {"samples": 10, "with_geofence_violation": 0},
                       "near/_check": {"samples": 10, "with_geofence_violation": 7}}}


def test_bench_summary_accepts_a_sound_run():
    got = P.bench_summary(_bench_doc())
    assert got["poses_per_check"] == 50 and got["near/_check"]["current"]["median_ms"] == 1.0


def test_bench_summary_refuses_a_run_that_cannot_be_trusted():
    def mutate(f):
        d = _bench_doc()
        f(d)
        return d
    mutants = {
        "failures": mutate(lambda d: d.update(failures=["agreement"])),
        "disagreement": mutate(lambda d: d["agreement"]["near/filter"].update(identical=9)),
        "nothing compared": mutate(lambda d: d.update(agreement={})),
        "near raised nothing (skipped every fence)":
            mutate(lambda d: d["sanity"]["near/_check"].update(with_geofence_violation=0)),
        "clear raised a violation":
            mutate(lambda d: d["sanity"]["clear/_check"].update(with_geofence_violation=1)),
    }
    let_through = []
    for name, doc in mutants.items():
        try:
            P.bench_summary(doc)
            let_through.append(name)
        except P.BenchRefused:
            pass
    assert not let_through, f"accepted: {let_through}"


def _run(floor, current, measured, sha="c0de"):
    d = P.bench_summary(_bench_doc())
    d["near/_check"]["floor"]["median_ms"] = floor
    d["near/_check"]["current"]["median_ms"] = current
    d["measured"] = measured
    d["code_sha"] = sha
    return d


def test_pick_bench_sets_aside_a_run_measured_under_load():
    quiet, loaded = _run(0.07, 0.35, "t1"), _run(0.15, 0.83, "t2")
    got = P.pick_bench(quiet, loaded)
    assert got["measured"] == "t1" and got["near/_check"]["current"]["median_ms"] == 0.35
    lc = got["load_check"]
    assert isinstance(lc, list) and len(lc) == 1, lc
    assert lc[0]["measured"] == "t2" and lc[0]["floor_ms"] == 0.15, lc
    assert lc[0]["near_check_ms"]["current"] == 0.83 and "under load" in lc[0]["why"], lc


def test_pick_bench_quotes_the_fresh_run_between_quiet_runs_and_keeps_every_run():
    """No ratchet: of two quiet runs the fresh one is quoted, so rebuilding
    does not drift towards the fastest run ever made; and every run of the
    same code stays on record."""
    a, b, c = _run(0.073, 0.35, "t1"), _run(0.077, 0.42, "t2"), _run(0.20, 0.90, "t3")
    ab = P.pick_bench(a, b)
    assert ab["measured"] == "t2", "the older, faster quiet run was kept"
    abc = P.pick_bench(ab, c)
    assert abc["measured"] == "t2"
    assert [r["measured"] for r in abc["load_check"]] == ["t1", "t3"], abc["load_check"]


def test_pick_bench_never_compares_other_code_or_configuration():
    """Failing first: the first pick_bench compared configuration only, so a
    stored run of older code with a lower floor outlived every fresh run."""
    stored, fresh = _run(0.01, 0.1, "t1", sha="old0"), _run(0.074, 1.05, "t2", sha="new1")
    got = P.pick_bench(stored, fresh)
    assert got["measured"] == "t2" and got["code_sha"] == "new1" and "load_check" not in got, got
    stored = _run(0.01, 0.1, "t1")
    stored["horizon_s"] = 3.0
    got = P.pick_bench(stored, _run(0.5, 9.0, "t2"))
    assert got["measured"] == "t2" and "load_check" not in got
    assert P.pick_bench(None, _run(0.5, 9.0, "t2"))["measured"] == "t2"


BENCH_FAIL = ("bench exited 1:\nagree  near/filter      identical decisions 0/40\n"
              "FAIL  near/filter: new and legacy disagree on 40 samples\n")


def test_measure_bench_keeps_stale_runs_only_when_asked_and_says_why():
    """7 Oct: the bench refused the tree's own code (new and legacy `filter`
    decisions disagreed after Repair.magnitude_m was added). A refused bench
    must stop the data step; only --keep-stale-bench keeps the stored runs of
    earlier code, and then the record says why, so the draft can say it."""
    stored = {k: _run(0.07, 0.35, "t1", sha="old0") for k in P.BENCH_RUNS}

    def refusing(**kw):
        raise P.BenchRefused(BENCH_FAIL)
    for keep, st in ((False, stored), (True, {}), (True, {"grant_load": stored["grant_load"]})):
        try:
            P.measure_bench(st, keep_stale=keep, runner=refusing)
        except P.BenchRefused:
            pass
        else:
            raise AssertionError(f"a refused bench was quoted (keep_stale={keep}, stored {sorted(st)})")
    bench, refused = P.measure_bench(stored, keep_stale=True, runner=refusing)
    assert bench == stored, "the kept runs are not the stored runs"
    assert refused["why"] == "FAIL  near/filter: new and legacy disagree on 40 samples", refused
    assert refused["tree_code"] and refused["measured"].endswith("Z"), refused
    # A sound fresh run is picked as before, and no refusal is recorded.
    bench, refused = P.measure_bench(stored, keep_stale=True,
                                     runner=lambda **kw: _run(0.074, 0.40, "t2", sha="new1"))
    assert refused is None and {b["code_sha"] for b in bench.values()} == {"new1"}, bench
    assert P.refusal_lines("bench exited 2:\nTraceback ...") == "bench exited 2:"


def test_verify_status_tells_a_stale_bench_from_drifted_numbers():
    """Exit 3 lets a DRAFT be built while only the Shield timing is of other
    code; any drifted number, or no bench at all, is exit 1 and stops it."""
    stored = {"wp1": {"a": 1}, "wp3_bench": {k: {"code_sha": "old0"} for k in P.BENCH_RUNS}}
    assert P.verify_status(P.verify(stored, {"wp1": {"a": 1}}, "old0")) == 0
    assert P.verify_status(P.verify(stored, {"wp1": {"a": 1}}, "new1")) == 3
    assert P.verify_status(P.verify(stored, {"wp1": {"a": 2}}, "new1")) == 1
    assert P.verify_status(P.verify({"wp1": {"a": 1}}, {"wp1": {"a": 1}}, "old0")) == 1, \
        "an absent bench read as merely stale"


def _profile(**over):
    """A tick profile whose headline, design-load arm and as-flown arm all
    differ, so a reader of the wrong one gets a wrong number."""
    d = {"command": "python tools/profile_shield_tick.py profile --cpus 0,2", "created_utc": "2026-10-06T21:55:53Z",
         "code_revision": "abc", "topology": "dev", "cpu_affinity": [0, 2], "failures": [],
         "headline": {"condition": "50 active rules, 5 s horizon at 0.1 s", "host_steady": True,
                      "check_p99_ms": 9.9},
         "pooled": {
             "design_load/grant": {
                 "check": {"n": 1301, "median_ms": 0.9162, "p99_ms": 3.3798, "max_ms": 4.2416,
                           "over_budget": 0, "budget_ms": 5.0},
                 "filter": {"n": 1301, "median_ms": 2.1098, "p99_ms": 11.8931, "max_ms": 27.4012,
                            "over_budget": 0, "budget_ms": 100.0}},
             "as_flown/grant": {
                 "check": {"n": 1301, "median_ms": 0.1, "p99_ms": 0.2, "max_ms": 0.3,
                           "over_budget": 0, "budget_ms": 5.0}}}}
    d.update(over)
    return d


def test_profile_facts_reads_the_design_load_arm_and_refuses_an_untrustworthy_run():
    d = Path(tempfile.mkdtemp())
    f = d / "p.json"
    f.write_text(json.dumps(_profile()), encoding="utf-8")
    got = P.profile_facts(f)
    assert got["quotable"] and got["why_not"] is None, got
    assert got["check"] == {"n": 1301, "median_ms": 0.916, "p99_ms": 3.38, "max_ms": 4.242,
                            "over_budget": 0, "budget_ms": 5.0}, got["check"]
    assert got["filter"]["p99_ms"] == 11.893 and got["cpu_affinity"] == [0, 2], got
    assert P.profile_facts(d / "absent.json") is None
    assert P.wp3_profile_facts({"a": d / "absent.json"}) == {"a": None}
    bad = {"a failure": _profile(failures=["host speed moved 2.1x"]),
           "an unsteady host": _profile(headline={"host_steady": False}),
           "a missing p99": _profile(pooled={"design_load/grant": {
               "check": {"n": 3, "median_ms": 1.0, "max_ms": 2.0, "over_budget": 0, "budget_ms": 5.0},
               "filter": _profile()["pooled"]["design_load/grant"]["filter"]}})}
    quoted = []
    for name, doc in bad.items():
        f.write_text(json.dumps(doc), encoding="utf-8")
        r = P.profile_facts(f)
        if r["quotable"] or not r["why_not"]:
            quoted.append(name)
    assert not quoted, f"quotable despite {quoted}"


def test_code_fingerprint_follows_the_code():
    d = Path(tempfile.mkdtemp())
    (d / "pkg").mkdir()
    (d / "pkg" / "a.py").write_bytes(b"x = 1\n")
    (d / "bench.py").write_bytes(b"run()\n")
    pats = ("pkg/*.py", "bench.py")
    a = P.code_fingerprint(d, pats)
    assert a == P.code_fingerprint(d, pats)
    (d / "pkg" / "a.py").write_bytes(b"x = 2\n")
    b = P.code_fingerprint(d, pats)
    assert a != b, "an edited file left the fingerprint unchanged"
    (d / "pkg" / "b.py").write_bytes(b"")
    assert P.code_fingerprint(d, pats) != b, "a new file left the fingerprint unchanged"
    try:
        P.code_fingerprint(d, pats + ("missing.py",))
    except SystemExit:
        pass
    else:
        raise AssertionError("a missing bench file was fingerprinted as if present")


# --------------------------------------------------------------------------- #
# verify: a deck is never built from numbers the repository no longer makes
# --------------------------------------------------------------------------- #

def test_verify_names_each_drifted_field_and_a_bench_of_other_code():
    """Failing first: the 6 Oct deck said sha256-canonical-v2 after
    wp1_roundtrip.json had moved to v3, and nothing noticed."""
    fresh = {"wp1": {"hash_scheme": "sha256-canonical-v3", "tamper_refused": 29},
             "nfz_flight": {"zone_vertices": [(45.0, 62.0), (62.0, 62.0)]}}
    bench = {k: {"code_sha": "c0de"} for k in P.BENCH_RUNS}
    stored = {"wp1": {"hash_scheme": "sha256-canonical-v2", "tamper_refused": 29},
              "nfz_flight": {"zone_vertices": [[45.0, 62.0], [62.0, 62.0]]},
              "wp3_bench": bench}
    probs = P.verify(stored, fresh, "c0de")
    assert len(probs) == 1 and probs[0].startswith("wp1.hash_scheme"), probs
    stored["wp1"]["hash_scheme"] = "sha256-canonical-v3"
    assert P.verify(stored, fresh, "c0de") == [], "tuples against stored lists read as drift"
    probs = P.verify(stored, fresh, "beef")
    assert len(probs) == len(P.BENCH_RUNS) and all("timed code" in p for p in probs), probs
    probs = P.verify({"wp3_bench": bench}, {"wp4": {"n": 1}}, "c0de")
    assert probs == ['wp4: stored "<absent>", now {"n": 1}'], probs


# --------------------------------------------------------------------------- #
# The stored data against its sources (local artefacts; SKIP when absent)
# --------------------------------------------------------------------------- #

def _stored():
    return json.loads(P.OUT_JSON.read_text(encoding="utf-8")) if P.OUT_JSON.exists() else None


def test_stored_wp1_matches_the_roundtrip_file():
    d, src = _stored(), P.WP1_SRC
    if d is None or not src.exists():
        return SKIP
    w = json.loads(src.read_text(encoding="utf-8"))
    assert d["wp1"]["round_trip_passed"] == w["round_trip"]["passed"]
    assert d["wp1"]["tamper_refused"] == w["round_trip"]["tamper_refusal"]["refused"]
    assert d["wp1"]["hash_scheme"] == w["hash_scheme"]


def test_stored_flight_is_recomputed_from_its_log():
    """Every flight figure on slide 11, recomputed here from the log and the
    policy without nfz_flight: the closest approach to the zone, the altitude
    band, the within-30 m and estimate shares, each beside its null."""
    d = _stored()
    fd = ROOT / "demo" / "out" / P.NFZ_TAG
    if d is None or not (fd / "flight_log.jsonl").exists() or not _load_policy_available():
        return SKIP
    from guardrail.models import load_policy
    mm = json.loads((fd / "metrics.json").read_text(encoding="utf-8"))
    rows = [json.loads(l) for l in (fd / "flight_log.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
    zone = next(c for c in load_policy(P.NFZ_POLICY).constraints if c.type == "polygon_fence")
    verts = [(float(v.x), float(v.y)) for v in zone.vertices]
    f = d["nfz_flight"]
    closest = min(P.dist_to_poly(r["x"], r["y"], verts) for r in rows)
    assert abs(f["zone_min_distance_m"] - round(closest, 2)) < 1e-9, (f["zone_min_distance_m"], closest)
    assert f["zone_ticks_inside"] == sum(P.point_in_poly(r["x"], r["y"], verts) for r in rows)
    assert f["alt_min_m"] == round(min(r["up"] for r in rows), 1)
    assert f["alt_max_m"] == round(max(r["up"] for r in rows), 1)
    assert f["within_30m"] == mm["frac_within_30m"] and f["zone_entered"] == mm["nfz_entered"]
    est = mm["estimate_on_subject"]
    assert f["estimate_on_car"] == est["on_subject_frac"]
    assert abs(P.estimate_on(rows, est["near_m"]) - est["on_subject_frac"]) <= 0.001
    assert f["within_30m_null_hover"] < f["within_30m"], "the hover null beat the flight"
    assert f["estimate_null_fixed_start"] < f["estimate_on_car"], "the fixed-start null beat the estimate"


def test_stored_profile_counts_match_the_expansion():
    d = _stored()
    if d is None:
        return SKIP
    try:
        from guardrail.scenario_spec import Library, expand, load_profile
    except ImportError:
        return SKIP
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    for name in ("smoke", "nightly"):
        eps, _ = expand(lib, load_profile(ROOT / "experiments" / "profiles" / f"{name}.yaml"))
        assert d["wp4"]["profiles"][name]["episodes"] == len(eps), name


def test_wp4_facts_counts_the_nightly_expansion():
    """Slide 8's template count and episode counts, from wp4_facts as it runs
    now (not the stored file), against a recount here. A review mutant that
    added one to the template count passed every test that read the stored
    JSON, because the stored JSON was written before the mutation."""
    try:
        from guardrail.scenario_spec import Library, expand, load_profile
        f = P.wp4_facts()
    except ImportError:
        return SKIP
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    counts = {}
    for name in ("smoke", "nightly"):
        eps, _ = expand(lib, load_profile(ROOT / "experiments" / "profiles" / f"{name}.yaml"))
        counts[name] = eps
        assert f["profiles"][name]["episodes"] == len(eps), name
    names = sorted({e.cell.spec.template for e in counts["nightly"]})
    assert f["templates"] == names and f["n_templates"] == len(names), (f["n_templates"], names)
    assert len(f["manifest_fields"]) == 6, f["manifest_fields"]


def _sections_or_skip():
    """compute_sections() as the tree computes it now, or SKIP when this
    machine lacks what it reads: the gitignored flight output, the OpenVLA
    tokenizer (without it the token range is None, not the stored one), or a
    package the 3.11 environment does not carry."""
    d = _stored()
    if d is None or not (ROOT / "demo" / "out" / P.NFZ_TAG / "flight_log.jsonl").exists():
        return None, None
    try:
        from guardrail import csp as C
        if C.openvla_token_counter() is None:
            return None, None
        fresh, _ = P.compute_sections()
    except ImportError:
        return None, None
    return d, json.loads(json.dumps(fresh))


def test_stored_sections_are_what_the_tree_computes_now():
    """Every section of progress_oct.json except the bench, recomputed by the
    fact functions as they are now. This is what kills a mutation in a fact
    function (the 7 Oct review had six survive the suite): the stored file was
    written by the unmutated code. It also fails when a source changed after
    the data step ran; then re-run the data step and the deck build.

    The bench's code_sha is not checked here. It hashes every guardrail/*.py
    file, so any edit anywhere in the package would fail this suite until a
    two-minute benchmark is re-run; the deck build refuses a stale bench
    itself (`--verify`), so it cannot reach a deck unnoticed."""
    stored, fresh = _sections_or_skip()
    if stored is None:
        return SKIP
    probs = [p for k, v in fresh.items() for p in P.diff(stored.get(k, "<absent>"), v, k)]
    assert not probs, ("docs/data/progress_oct.json no longer matches the tree; re-run "
                       "tools/deck/build_progress_oct_data.py, then the deck build:\n      "
                       + "\n      ".join(probs))


def test_the_built_pack_is_the_builders_current_output():
    """Failing first (7 Oct review): the builder was saved again a minute
    after the build, and the pack that would have been sent was the older
    text. The build stamp records the hashes of the builder and the data it
    read and of the deck it wrote; the PDF must be exported after the deck."""
    stamp_f = P.BUILD / "build_stamp.json"
    if not stamp_f.exists():
        return SKIP
    import hashlib
    st = json.loads(stamp_f.read_text(encoding="utf-8"))

    def sha(p):
        return hashlib.sha256(p.read_bytes()).hexdigest()
    builder = ROOT / "tools" / "deck" / "build_progress_oct_deck.js"
    deck = P.SHARE / st["deck"]
    stale = [what for what, ok in (
        ("the builder changed after the build", sha(builder) == st["builder_sha256"]),
        ("progress_oct.json changed after the build", sha(P.OUT_JSON) == st["data_sha256"]),
        ("the deck is not the one the build wrote", deck.exists() and sha(deck) == st["deck_sha256"]),
    ) if not ok]
    pdf = deck.with_suffix(".pdf")
    if pdf.exists() and deck.exists() and pdf.stat().st_mtime < deck.stat().st_mtime:
        stale.append("the PDF is older than the deck")
    assert not stale, "rebuild the pack (deck step, then PDF): " + "; ".join(stale)


def test_stored_bench_carries_the_code_it_timed():
    d = _stored()
    if d is None:
        return SKIP
    for name in P.BENCH_RUNS:
        run = d["wp3_bench"][name]
        assert run.get("code_sha") and run.get("measured", "").endswith("Z"), (name, run.get("measured"))
        lc = run.get("load_check", [])
        assert isinstance(lc, list), f"{name}: load_check {type(lc).__name__}, not the run list"


def test_the_built_deck_text_is_clean():
    dump = P.BUILD / "slides_text.json"
    if not dump.exists():
        return SKIP
    probs = P.check_deck(json.loads(dump.read_text(encoding="utf-8")))
    readme = P.SHARE / "README.md"
    if readme.exists():
        probs += P.check_cover(readme.read_text(encoding="utf-8"))
    assert probs == [], probs


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs local artefacts)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except SystemExit as e:
            # The fact functions refuse bad input with SystemExit, which is not
            # an Exception: without this clause one regression ends the run
            # before the 'N/M passed' line (7 Oct review).
            failed += 1
            print(f"ERROR {fn.__name__}\n      SystemExit: {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
