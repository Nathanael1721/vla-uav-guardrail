"""Numbers for the October 2026 ITRI progress deck, gathered in one file.

    C:/Users/natha/.conda/envs/vla-real/python.exe tools/deck/build_progress_oct_data.py
    ... --skip-bench        reuse the stored Shield timing (refused unless it
                            timed the code in the tree now)
    ... --bench-replace     quote the fresh bench run, drop the stored ones
    ... --keep-stale-bench  run the bench; if the bench refuses itself, keep
                            the stored runs (which timed other code) and record
                            why. For a review draft only: --verify then exits 3
                            and the deck step builds nothing but a DRAFT
    ... --verify            recompute every section, compare with the stored
                            JSON; writes nothing. Exit 0: current. Exit 3: only
                            the stored Shield timing measured other code. Exit
                            1: any other difference
    ... --check-deck FILE [--cover README]
                            scan the deck's slide text (written by the deck
                            builder) and the README cover note for withdrawn
                            claims and unfilled numbers; exit 1 on any hit

Writes docs/data/progress_oct.json and the flight chart
docs/share/2026-10-ITRI/_build/nfz2_track.png (the share folder is gitignored).
The deck builder runs --verify before it reads the JSON, so a deck is never
built from numbers the repository no longer produces.

WHERE EACH NUMBER COMES FROM

Nothing about a measurement is typed in here. Each section reads the artefact
that produced it, or re-runs the code that does:

  release      CHANGELOG.md, the dated sections from 2026-10-06 on: the newest
               one's date is the deck's status date, and the newest recorded
               suite result is the slide's test count
  wp1          docs/data/wp1_roundtrip.json (tools/wp1_roundtrip_kpi.py),
               policies/policy.lock.json
  wp2          guardrail.compiler.coverage_report over policies/, counted with
               OpenVLA's own tokenizer when this machine has it; the example
               prompt is compiled here from the city no-fly-zone policy
  wp3_bench    experiments/bench_shield_50rules.py, RUN by this script at the
               grant's load (5 s horizon at 0.1 s = 50 future poses, 50 rules)
               and at the default 3 s / 0.5 s. Each run carries `code_sha`, a
               hash of the code it timed; a stored run of other code is never
               quoted. Of two runs of the same code, the fresh one is quoted
               unless it ran under load, and the other is appended to
               `load_check` (`pick_bench`)
  wp3_profile  deploy/evidence/dev/shield_tick_profile*.json
               (tools/profile_shield_tick.py): the Shield replayed over real
               flight ticks at the same load, pinned to the desktop's
               performance cores and, separately, its efficiency cores. Absent
               files give None; the deck then leaves the sentence out
  wp3_fsm      guardrail.fsm.FSMConfig defaults and FSMState
  wp4          guardrail.scenario_spec.expand over experiments/profiles/*.yaml
               (counted, not run); docs/data/kpi_rollup_2026-10-06.json for the
               KPI report's definitions
  paraphraser  experiments/paraphrases/validation_report.json and the stored
               paraphrase files
  nfz_flight   demo/out/citylife_redcar_nfz2/{metrics,manifest}.json and its
               flight_log.jsonl; the zone from policies/follow_car_citylife_nfz.yaml.
               The within-30 m share and the estimate share are recomputed from
               the log, each with a null (a drone hovering at its start; an
               estimate fixed at the car's first position)

WHAT THE DECK MUST NOT SAY

The deck goes to ITRI. The PI's rule (6 Oct): progress-focused, but every
stated fact true, with corrected numbers, and never a retracted claim.
`WITHDRAWN` lists the claims this project has retracted (CHANGELOG.md
"Retracted" sections, 29 Sept - 6 Oct) and the claims docs/PI-DECISIONS-2026-
10-06.md says not to make. The deck builder dumps every slide's text and
speaker notes to JSON, writes the README cover note, and runs `--check-deck`
on both before it writes the deck; a hit fails the build. A guard that never
says no is not a guard, so
tests/test_build_progress_oct_data.py feeds it each retracted sentence and the
30 Sept deck's own source, and both must be refused.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT_JSON = ROOT / "docs" / "data" / "progress_oct.json"
SHARE = ROOT / "docs" / "share" / "2026-10-ITRI"
BUILD = SHARE / "_build"
CHART = BUILD / "nfz2_track.png"
CHANGELOG = ROOT / "CHANGELOG.md"
RELEASE_DATE = "2026-10-06"
NFZ_TAG = "citylife_redcar_nfz2"
NFZ_POLICY = ROOT / "policies" / "follow_car_citylife_nfz.yaml"
OCC_BAND = ROOT / "demo" / "out" / "citymap_citylife" / "occ_day_flightband_6to14.npz"
# The grant's locked CSP schema, field by field (grant page "Prefix Compiler",
# `class CSP(BaseModel)`). guardrail/csp.py carries these plus fields of its
# own (`policy_id` for traceability, `selection` as the audit trail of what was
# kept and cut; docs/DESIGN-prefix-compiler.md section 1), which the deck does
# not count as the grant's.
GRANT_CSP_FIELDS = (
    "csp_version", "policy_hash", "generation", "issued_at", "mission_id",
    "lookahead_s", "P0_constraints", "P1_summary", "P2_summary",
    "allowed_action_set", "forbidden_action_set", "cost_map_ref",
    "natural_language_prompt", "relevance_explanations",
)
# The bench at the grant's load: Safety Shield page, "~50 active rules and 50
# future-pose checks per tick, total query budget <= 5 ms" -> 5 s at 0.1 s.
BENCH_RUNS = {
    "grant_load": {"horizon": 5.0, "dt": 0.1, "legacy_cap": 40},
    "default_3s": {"horizon": 3.0, "dt": 0.5, "legacy_cap": 100},
}

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def jload(p: Path):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def rel(p: Path) -> str:
    try:
        return Path(p).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return str(p)


# --------------------------------------------------------------------------- #
# Withdrawn claims: the guard the deck build runs on its own text
# --------------------------------------------------------------------------- #

# (pattern, why it is withdrawn, words that excuse a match - a plan in the
# future tense, or a negation - checked in the 20 characters before the match).
# An excuse word must not be one a past-tense claim also uses: "re-" let
# "timings were re-measured on the Orin" through, and a bare "be " let "can
# be trusted: measured on the Orin" through (review, 7 Oct).
EXCUSE_WINDOW = 20
WITHDRAWN: list[tuple[str, str, tuple[str, ...]]] = [
    # Over-refuses on purpose: the grant itself calls hil "the canonical KPI
    # configuration", but the same words were the retracted claim about the
    # desktop rail, so the deck says "the grant's KPI configuration" instead.
    (r"canonical[\s-]*(hil|rail|kpi|topology|configuration)",
     "the desktop SITL + MAVROS 2 rail is the grant's 'dev' topology; KPIs come "
     "from 'hil' runs (CHANGELOG 2026-10-06, Retracted)", ()),
    (r"\bhard kpi\b",
     "P0 escape (target 0) is one of five acceptance KPIs, not 'the hard KPI' "
     "(docs/CORRECTION-2026-10-06, item 9)", ()),
    (r"(?<![\d.])(3\.76|5\.22|5\.15)(?![\d])|6\.9\s*-\s*7\.5",
     "inflated detector rate; the mid-evaluation flights ran 2.77-4.31 Hz over "
     "the mission (CHANGELOG 2026-09-29 and 2026-10-06, Retracted)", ()),
    (r"fail-?safe[^.\n]{0,60}?(target|=|of|is)\s*1\.0\b|fail-?safe[^.\n]{0,60}?\b100\s*%",
     "the grant's fail-safe target is >= 99 %, not 1.0 or 100 % (CHANGELOG 2026-10-06)", ()),
    (r"11[,\s]?713\s*(/|of)\s*11[,\s]?713",
     "the Shield's own re-check, not the grant's repair success KPI "
     "(CHANGELOG 2026-10-06, Retracted)", ()),
    (r"\b41 shielded\b|p0[^.\n]{0,30}?(=|is|of|was)\s*0(\.0)?\b[^.\n]{0,60}?\b(kpi|contract)",
     "a P0 of zero from dev or Project AirSim runs is not a contract KPI figure "
     "(CHANGELOG 2026-10-05 and 2026-10-06, Retracted)", ()),
    (r"\ball (five|5) (acceptance )?kpis? (were |are |have been )?(measured|computed)",
     "the five KPIs were computed on dev runs, not in hil Stress Testing runs "
     "(docs/CORRECTION-2026-10-06, item 9)", ()),
    (r"kpi[\s-]grade",
     "no stored run is KPI-grade under the grant's definition; ITRI material "
     "does not use the term (CHANGELOG 2026-10-06, Changed)", ()),
    (r"gazebo[^.\n]{0,60}(never (been )?run|no artefacts?)",
     "the Gazebo scripts exist and no run output is kept (CHANGELOG 2026-10-05, "
     "Retracted)", ()),
    (r"\bshield\b[^.\n]{0,40}?\b(avoided|kept (the drone )?out"
     r"|(steer|rout)\w*\b[^.\n]{0,30}?\b(a)?round\b)",
     "in the 3 Oct no-fly-zone flights the controller avoided the zone and the "
     "Shield corrected no command (docs/share/2026-09-30-ITRI/README.md)", ()),
    (r"swappable by design|backend is swappable",
     "the clause covers 4-D VLA backends, not a hand-written pilot "
     "(CHANGELOG 2026-10-05, Retracted)", ()),
    (r"\b1 of 34\b",
     "0 of 34 camera flights meet both rate gates (docs/CORRECTION-2026-10-06, "
     "item 7)", ()),
    (r"22\.4\s*%?\s*(to|->|\u2192)\s*95\.5|\b0\s*/\s*18\b",
     "the replay's old arm was not what flew: 22.9 % -> 94.5 %, 0 of 12 wrong "
     "seeds (CHANGELOG 2026-09-30, Retracted / corrected)", ()),
    (r"(csp|rule summary|constraint summary)[^.\n]{0,25}?\breach(es|ed)\b[^.\n]{0,10}?\b(the )?(open)?vla\b"
     r"|\b(open)?vla\b[^.\n]{0,25}?\b(reads|read|obeys|obeyed|follows|followed) (the )?(csp|rule summary)"
     # A heading such as "What the VLA reads: <policy>" says the same thing
     # without a verb next to "CSP", and so does "follows the constraint
     # summary pack" with no VLA named in the clause (review, 6 Oct).
     r"|\bwhat the (open)?vla reads\b"
     r"|\b(reads|obeys|obeyed|follows|followed) (the )?(csp|rule summary|constraint summary( pack)?)\b"
     # "The CSP was in OpenVLA's prompt on every flight" (review, 7 Oct).
     r"|\b(csp|rule summary)\b[^.\n]{0,30}\bin (the )?(open)?vla.?s prompt\b[^.\n]{0,30}\b(every|each|all)\b",
     "no flight has yet flown a VLA with the CSP in its prompt "
     "(docs/PI-DECISIONS-2026-10-06.md, claims not to make)",
     # Not "can ", "first " or "planned": each also opens a past-tense claim
     # ("We can confirm the CSP reached OpenVLA", "On the first flight OpenVLA
     # followed the CSP"), and no true sentence of the deck needs them (7 Oct).
     ("will ", "to be ", "next")),
    (r"\b(open)?vla\b[^.\n]{0,20}?\bfl(ies|ew) (the|in the) city",
     "the city flights are flown by the follow controller, not a VLA "
     "(docs/PI-DECISIONS-2026-10-06.md)", ()),
    (r"\b(measured|timed|benchmarked|flown|tested|verified) on (an? |the )?(jetson )?orin",
     "nothing has been measured on the Orin yet (CHANGELOG 2026-10-06: 'Not "
     "measured on a Jetson Orin')", ("will be ", "to be ", "will ")),
    (r"\bis (the |a )?contract(ual)? kpi",
     "no number from a dev run or a Project AirSim flight is a contract KPI figure; "
     "the grant takes KPIs from hil Stress Testing runs (CHANGELOG 2026-10-05, Retracted)", ()),
    # Any other way of calling a number a contract KPI ("These are contract
    # KPI numbers", "P0 0 on 41 flights, the contract KPI"); a negation
    # ("is not a contract KPI figure") is the corrected wording.
    (r"\bcontract(ual)? kpis?\b",
     "no number in this pack is a contract KPI figure; the grant takes KPIs from "
     "hil Stress Testing runs (CHANGELOG 2026-10-05 and 2026-10-06, Retracted)",
     ("not ", "no ", "never ")),
    (r"exactly as the (contract|grant) designs",
     "upstream compliance in the grant comes from the CSP in the VLA prompt, and "
     "the backstop behind the Shield is ArduPilot's GeoFence (CHANGELOG 2026-10-05, "
     "Retracted)", ()),
]

# Text a template prints when a number is missing. A slide that says
# "undefined %" or "NaN ms" has a silent hole in it; the build refuses it.
# "null" is a word the notes use ("the null on the right"), so only a null
# standing where a number goes - before a unit or a slash, or after a slash -
# counts.
UNFILLED_RE = re.compile(r"\b(undefined|NaN|Infinity)\b"
                         r"|\bnull ?(%|ms\b|m\b|s\b|/|tokens?\b)|/ ?null\b")

# The PI's rule for this pack (6 Oct): no slide of obstacles or shortcomings.
FORBIDDEN_TITLE_WORDS = ("limit", "obstacle", "shortcoming", "weakness", "known issue",
                         "open issue", "problem", "risk", "gap", "caveat", "backup")
SLIDES_MIN, SLIDES_MAX = 10, 14


def withdrawn_claims(text: str) -> list[dict]:
    """Every withdrawn claim in `text`, as {claim, why}. Empty means clean.

    Case-insensitive. A match preceded (within EXCUSE_WINDOW characters) by
    one of the rule's excuse words - "will be measured on the Orin", "is not
    a contract KPI figure" - is a plan or a negation, not a claim, and is let
    through.
    """
    hits = []
    low = text.lower()
    for pat, why, future in WITHDRAWN:
        for m in re.finditer(pat, low, flags=re.IGNORECASE):
            before = low[max(0, m.start() - EXCUSE_WINDOW):m.start()]
            if future and any(w in before for w in future):
                continue
            hits.append({"claim": text[m.start():m.end()], "why": why})
    return hits


def forbidden_title(title: str) -> str | None:
    low = title.lower()
    for w in FORBIDDEN_TITLE_WORDS:
        if re.search(r"\b" + re.escape(w), low):
            return w
    return None


def unfilled(text: str) -> list[str]:
    """Places where a template printed a missing number ('undefined', 'NaN',
    'null %'). Empty means every number was filled in."""
    return [m.group(0) for m in UNFILLED_RE.finditer(text or "")]


def check_deck(slides: list[dict]) -> list[str]:
    """Problems with the deck's text: withdrawn claims in any slide's text or
    notes, a missing number printed as 'undefined' / 'NaN' / 'null %', a
    forbidden slide title, a slide with no speaker note, or a slide count
    outside 10-14. Each slide is {n, title, text, notes}."""
    problems = []
    if not SLIDES_MIN <= len(slides) <= SLIDES_MAX:
        problems.append(f"{len(slides)} slides; this pack is {SLIDES_MIN}-{SLIDES_MAX}")
    for s in slides:
        n = s.get("n")
        w = forbidden_title(s.get("title") or "")
        if w:
            problems.append(f"slide {n}: title {s['title']!r} reads as a "
                            f"shortcomings slide ({w!r})")
        if not (s.get("notes") or "").strip():
            problems.append(f"slide {n}: no speaker note")
        for part in ("title", "text", "notes"):
            for h in withdrawn_claims(s.get(part) or ""):
                problems.append(f"slide {n} {part}: {h['claim']!r} - {h['why']}")
            for u in unfilled(s.get(part) or ""):
                problems.append(f"slide {n} {part}: a missing number printed as {u!r}")
    return problems


COVER_START, COVER_END = "## Cover note", "## How it was built"


def cover_note(readme: str) -> str | None:
    """The cover-note block of the share README: from its '## Cover note'
    heading to '## How it was built'. None when either heading is missing -
    a README without the block is not 'a clean cover note'."""
    a = readme.find(COVER_START)
    b = readme.find(COVER_END, a + 1) if a >= 0 else -1
    if a < 0 or b < 0:
        return None
    return readme[a:b]


def check_cover(readme: str) -> list[str]:
    """Problems with the cover note that goes to ITRI with the deck: the same
    withdrawn-claims and missing-number checks the slides get."""
    block = cover_note(readme)
    if block is None:
        return [f"README has no '{COVER_START}' ... '{COVER_END}' block"]
    probs = [f"cover note: {h['claim']!r} - {h['why']}" for h in withdrawn_claims(block)]
    probs += [f"cover note: a missing number printed as {u!r}" for u in unfilled(block)]
    return probs


# --------------------------------------------------------------------------- #
# Release record
# --------------------------------------------------------------------------- #

def changelog_section(text: str, date: str) -> str | None:
    """The `### <date> ...` section of the changelog, up to the next ### or ##
    heading. None when there is no such section."""
    lines = text.splitlines()
    start = next((i for i, l in enumerate(lines) if l.startswith(f"### {date}")), None)
    if start is None:
        return None
    end = next((j for j in range(start + 1, len(lines))
                if lines[j].startswith("### ") or lines[j].startswith("## ")), len(lines))
    return "\n".join(lines[start:end])


SUITE_RE = re.compile(r"Full suite on Python ([\d.]+):\s*(\d+) of (\d+) over (\d+) test files")
# The 2026-10-06 entry's own words for the 3.11 run (the grant locks 3.11+).
PY311_RE = re.compile(r"On 3\.11 the guardrail package's tests pass")


def suite_counts(section: str | None) -> dict | None:
    """The suite result the release section records, or None when it records
    none. Never a zero: a missing line is 'not recorded', not '0 passed'.

    The LAST suite line of the section is the current one: a section that is
    added to later in the day (a second wave) appends its own result below
    the first."""
    if not section:
        return None
    # A changelog line may wrap with an indent: collapse every run of whitespace.
    flat = re.sub(r"\s+", " ", section)
    found = list(SUITE_RE.finditer(flat))
    if not found:
        return None
    m = found[-1]
    return {"python": m.group(1), "passed": int(m.group(2)), "total": int(m.group(3)),
            "files": int(m.group(4)),
            "guardrail_passes_on_311": bool(PY311_RE.search(flat[m.end():m.end() + 200]))}


SECTION_DATE_RE = re.compile(r"^### (\d{4}-\d{2}-\d{2})\b")


def dated_sections(text: str) -> list[tuple[str, str]]:
    """(date, text) of every '### YYYY-MM-DD ...' changelog section, newest
    date first; sections of the same date keep their order in the file."""
    lines = text.splitlines()
    heads = [(i, m.group(1)) for i, l in enumerate(lines) if (m := SECTION_DATE_RE.match(l))]
    out = []
    for k, (i, date) in enumerate(heads):
        end = next((j for j in range(i + 1, len(lines))
                    if lines[j].startswith("### ") or lines[j].startswith("## ")), len(lines))
        out.append((date, "\n".join(lines[i:end])))
    return sorted(out, key=lambda ds: ds[0], reverse=True)


def release_facts(text: str | None = None) -> dict:
    """What the changelog records from RELEASE_DATE on.

    `status_date` is the newest dated section, not a constant: work that is
    committed later (a second section, 7 Oct) must not be reported as
    'delivered by 6 October'. The suite result is the newest one recorded,
    with the date of the section it is in; None when no section from
    RELEASE_DATE on records one - never a zero."""
    text = CHANGELOG.read_text(encoding="utf-8") if text is None else text
    secs = [(d, s) for d, s in dated_sections(text) if d >= RELEASE_DATE]
    if not secs:
        raise SystemExit(f"CHANGELOG.md has no '### <date>' section on or after {RELEASE_DATE}")
    suite, suite_date = None, None
    for d, s in secs:
        c = suite_counts(s)
        if c:
            suite, suite_date = c, d
            break
    return {"source": "CHANGELOG.md", "status_date": secs[0][0],
            "section": secs[0][1].splitlines()[0].lstrip("#").strip(),
            "sections": [d for d, _ in secs], "suite": suite, "suite_section": suite_date}


# --------------------------------------------------------------------------- #
# WP1 - signed policy bundles
# --------------------------------------------------------------------------- #

def frac(s: str) -> tuple[int, int]:
    a, b = s.split("/")
    return int(a), int(b)


WP1_SRC = ROOT / "docs" / "data" / "wp1_roundtrip.json"
LOCK_SRC = ROOT / "policies" / "policy.lock.json"


def wp1_facts(src: Path = WP1_SRC, lock_src: Path = LOCK_SRC) -> dict:
    d = jload(src)
    rt, tr = d["round_trip"], d["round_trip"]["tamper_refusal"]
    null = d["null_accept_everything"]
    lock = jload(lock_src)
    # The broken-policy corpus (policies/negative/) is reported by newer runs
    # of tools/wp1_roundtrip_kpi.py only; an older file has no such column,
    # and then the deck leaves the figure out rather than print a zero.
    neg = (d.get("kpi") or {}).get("negative_refusal")
    neg_null = null.get("negative_refusal")
    return {
        "source": rel(src), "command": d["command"], "code_revision": d["code_revision"],
        "hash_scheme": d["hash_scheme"],
        "policies": rt["n"], "round_trip_passed": rt["passed"],
        # KeyError, not a zero, if the file stops recording verified signatures.
        "signatures_verified": rt["signatures"]["verified"],
        "tamper_run": tr["run"], "tamper_refused": tr["refused"],
        "null_tamper_refused": frac(null["tamper_refusal"])[0],
        "null_tamper_run": frac(null["tamper_refusal"])[1],
        "negative_refused": frac(neg)[0] if neg else None,
        "negative_run": frac(neg)[1] if neg else None,
        "null_negative_refused": frac(neg_null)[0] if neg and neg_null else None,
        "null_why": null["why"],
        "lock_entries": len(lock["policies"]),
    }


# --------------------------------------------------------------------------- #
# WP2 - the Constraint Summary Pack
# --------------------------------------------------------------------------- #

def token_range(rows: list[dict], key: str = "tokens_exact") -> list[int] | None:
    """[min, max] of a per-policy token count, or None if any row lacks it
    (the exact counter was not available) - never a range built from zeros."""
    vals = [r.get(key) for r in rows]
    if not vals or any(v is None for v in vals):
        return None
    return [min(vals), max(vals)]


def grant_csp_fields(model_fields) -> int:
    """How many of the grant's locked CSP fields the model carries - all
    of them, or the build stops. Counted from GRANT_CSP_FIELDS, never from the
    model: a field another change adds to guardrail/csp.py must not turn the
    slide into 'the grant's 15 locked fields'."""
    missing = [f for f in GRANT_CSP_FIELDS if f not in set(model_fields)]
    if missing:
        raise SystemExit(f"guardrail/csp.py CSP lacks the grant's locked field(s) {missing}")
    return len(GRANT_CSP_FIELDS)


def wp2_from_report(rep: dict, model_fields, risk_weights, example: dict) -> dict:
    """The WP2 section from one `guardrail.compiler.coverage_report` result.

    Pure: no tokenizer, no policy files. It is kept apart from `wp2_facts` so
    the mapping from the report to the slide's figures can be tested on a
    report in which every count differs. On the real report several of them
    coincide (the compiler and the naive cut both carry 105 of 105), so a
    mapping that printed the naive cut as the null, or the table counter's
    tokens as OpenVLA's, would pass any test run on the real report alone.

    `example` is {policy, policy_id, rules, prompt} for the compiled example
    policy. Its OpenVLA token count is the report row for the same file; it is
    None, never a zero, when the report has no such row or ran without the
    exact counter.
    """
    t = rep["totals"]
    fields = list(model_fields)
    name = Path(example["policy"]).name
    ex_rows = [r for r in rep["rows"] if Path(r["file"]).name == name]
    return {
        "command": rep["command"], "code_revision": rep["code_revision"],
        "budget_tokens": rep["budget_tokens"], "exact_counter": rep["exact_counter"],
        "table_counter": rep["token_counter"],
        "policies": t["n_policies"], "compiled": t["n_compiled"],
        "p0_in_scope": t["n_p0_in_scope"], "p0_covered": t["n_p0_covered"],
        # The empty-CSP null: the same scorer on each CSP with its P0 entries
        # and prompt emptied (guardrail/compiler.py coverage_report).
        "null_p0_covered": t["null_n_p0_covered"],
        # The discriminating null: the old by-type text cut to the same
        # budget. While every policy fits whole it scores the same as the
        # compiler, and `kpi_discriminates` is False (guardrail/compiler.py).
        "baseline_p0_covered": t["baseline_n_p0_covered"],
        "rules_in_scope": t["n_rules_in_scope"], "rules_in_text": t["n_rules_in_text"],
        "rules_in_csp": t["n_rules_in_csp"], "rules_explained": t["n_rules_explained"],
        "dropped_for_budget": t["n_dropped_budget"],
        "kpi_discriminates": t["kpi_discriminates"],
        # OpenVLA's own count per policy; the table counter's is an upper
        # bound and is kept apart as `tokens_table_max`.
        "tokens_exact_range": token_range(rep["rows"]),
        "tokens_table_max": t["max_tokens_used"],
        "csp_fields_locked": grant_csp_fields(fields),
        "csp_fields_added": [f for f in fields if f not in GRANT_CSP_FIELDS],
        "risk_weights": list(risk_weights),
        "example": {**{k: example[k] for k in ("policy", "policy_id", "rules", "prompt")},
                    "tokens_exact": ex_rows[0].get("tokens_exact") if ex_rows else None},
    }


def wp2_facts() -> dict:
    from guardrail import compiler as Cmp
    from guardrail import csp as C
    from guardrail.models import load_policy
    exact = C.openvla_token_counter()
    rep = Cmp.coverage_report(ROOT / "policies", Cmp.DEFAULT_BUDGET_TOKENS, exact)
    w = C.RiskWeights()
    pol = load_policy(NFZ_POLICY)
    csp = Cmp.ConstraintCompiler(pol).compile_csp(Cmp.REPORT_MISSION, now=None,
                                                  region_margin_m=None)
    example = {"policy": rel(NFZ_POLICY), "policy_id": pol.policy_id,
               "rules": len(pol.constraints), "prompt": csp.natural_language_prompt}
    return wp2_from_report(rep, C.CSP.model_fields,
                           [w.severity, w.proximity, w.time_critical], example)


# --------------------------------------------------------------------------- #
# WP3 - Shield timing and the escalation FSM
# --------------------------------------------------------------------------- #

class BenchRefused(RuntimeError):
    pass


def _stat(r: dict, key: str, impl: str) -> dict:
    s = r["results"][key][impl]
    return {"n": s["n"], "median_ms": round(s["median_ms"], 3), "p99_ms": round(s["p99_ms"], 3)}


def bench_summary(doc: dict) -> dict:
    """The few numbers the deck shows, from one bench JSON - refused unless the
    bench itself says the timing means something: no failures, the new Shield
    and the legacy loop agree on every compared sample, and the `near`
    scenario really raised geofence violations while `clear` raised none (a
    Shield that skipped every fence would post the best time)."""
    if doc.get("failures"):
        raise BenchRefused(f"bench reported failures: {doc['failures']}")
    agree = doc.get("agreement") or {}
    if not agree:
        raise BenchRefused("bench compared nothing against the legacy loop")
    for k, a in agree.items():
        if a["identical"] != a["compared"] or a["compared"] == 0:
            raise BenchRefused(f"{k}: {a['identical']}/{a['compared']} identical decisions")
    san = doc.get("sanity") or {}
    if san.get("near/_check", {}).get("with_geofence_violation", 0) == 0:
        raise BenchRefused("the 'near' scenario raised no geofence violation")
    if san.get("clear/_check", {}).get("with_geofence_violation", 1) != 0:
        raise BenchRefused("the 'clear' scenario raised a geofence violation")
    out = {"horizon_s": doc["horizon_s"], "dt_s": doc["dt_s"], "rules": doc["rules"],
           "fences": doc["fences"], "budgets_ms": doc["budgets_ms"],
           "poses_per_check": int(round(doc["horizon_s"] / doc["dt_s"])),
           "machine": doc.get("machine"), "git": doc.get("git"), "command": doc.get("command"),
           "agreement": {k: f"{a['identical']}/{a['compared']}" for k, a in agree.items()}}
    for key in ("clear/_check", "near/_check", "near/filter", "grid/_check"):
        out[key] = {impl: _stat(doc, key, impl)
                    for impl in ("current", "legacy", "floor") if impl in doc["results"][key]}
    return out


# The code a Shield timing measures: the whole guardrail package (the tick
# path runs through models, projection, ir, shield), the benchmark itself and
# the verbatim copy of the old per-fence loop it times as 'legacy'.
BENCH_CODE = ("guardrail/*.py", "experiments/bench_shield_50rules.py",
              "tests/legacy_fence_loop.py")


def code_fingerprint(root: Path = ROOT, patterns=BENCH_CODE) -> str:
    """sha256 over the bytes (and repo paths) of the files a bench run
    measures. Two runs with the same fingerprint timed the same code; a git
    revision cannot say that while the working tree is dirty."""
    files = sorted({p for pat in patterns for p in root.glob(pat) if p.is_file()})
    if not files:
        raise SystemExit(f"no bench code found under {root} for {patterns}")
    for pat in patterns:
        if "*" not in pat and not (root / pat).is_file():
            raise SystemExit(f"bench code file missing: {pat}")
    h = hashlib.sha256()
    for p in files:
        h.update(p.relative_to(root).as_posix().encode("utf-8") + b"\0")
        h.update(p.read_bytes() + b"\0")
    return h.hexdigest()[:16]


def run_bench(horizon: float, dt: float, legacy_cap: int) -> dict:
    sha = code_fingerprint()
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "bench.json"
        cmd = [sys.executable, str(ROOT / "experiments" / "bench_shield_50rules.py"),
               "--horizon", str(horizon), "--dt", str(dt),
               "--legacy-cap", str(legacy_cap), "--json", str(out)]
        p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
        if p.returncode != 0 or not out.exists():
            raise BenchRefused(f"bench exited {p.returncode}:\n{p.stdout[-2000:]}\n{p.stderr[-2000:]}")
        doc = jload(out)
    if code_fingerprint() != sha:
        raise BenchRefused("the measured code changed while the bench ran; run it again")
    s = bench_summary(doc)
    s["code_sha"] = sha
    s["command"] = (f"python experiments/bench_shield_50rules.py --horizon {horizon:g} "
                    f"--dt {dt:g} --legacy-cap {legacy_cap}")
    s["measured"] = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    return s


def _floor_ms(run: dict) -> float:
    return run["near/_check"]["floor"]["median_ms"]


# The configuration and code two runs must share before their timings are
# compared at all.
BENCH_SAME = ("horizon_s", "dt_s", "rules", "fences", "code_sha")
# A fresh run is set aside as 'measured under load' only when its floor is
# this much slower than the stored run's. Between two quiet runs the fresh one
# is quoted, so repeated rebuilds do not ratchet towards the fastest run ever
# made.
LOAD_TOLERANCE = 1.25


def _brief(run: dict, why: str) -> dict:
    return {"why": why, "measured": run.get("measured"), "floor_ms": _floor_ms(run),
            "near_check_ms": {i: run["near/_check"][i]["median_ms"] for i in ("legacy", "current")},
            "near_filter_ms": {i: run["near/filter"][i]["median_ms"] for i in ("legacy", "current")}}


def pick_bench(stored: dict | None, fresh: dict) -> dict:
    """Which of two runs of the same benchmark the deck quotes.

    Timings on a shared desktop move with whatever else is running: on
    6 Oct a second run under concurrent test suites measured every column,
    the unchanged `floor` included, about twice as slow. The `floor` (the
    same samples with the fences removed) does not run the Shield, so it
    measures the machine. Rules:

    * Runs are compared only if they share configuration AND code
      (`BENCH_SAME`, `code_sha`). Otherwise the fresh run is quoted and the
      stored one, which timed other code, is dropped with its record.
    * Of two comparable runs the fresh one is quoted, unless its floor is
      more than `LOAD_TOLERANCE` times the stored floor: then it ran under
      load, the stored run stays quoted, and the fresh one is recorded.
    * `load_check` is a list: every comparable run not quoted is appended,
      so nothing measured on this code is dropped across rebuilds.
    """
    same = stored is not None and all(stored.get(k) == fresh.get(k) for k in BENCH_SAME)
    if not same:
        return {k: v for k, v in fresh.items() if k != "load_check"}
    history = list(stored.get("load_check") or []) if isinstance(stored.get("load_check"), list) else []
    if _floor_ms(fresh) > LOAD_TOLERANCE * _floor_ms(stored):
        keep, other = stored, fresh
        why = (f"floor {_floor_ms(fresh)} ms > {LOAD_TOLERANCE} x the quoted run's "
               f"{_floor_ms(stored)} ms: measured under load")
    else:
        keep, other = fresh, stored
        why = "an earlier run of the same code, superseded by a run on an equally or less loaded machine"
    out = {k: v for k, v in keep.items() if k != "load_check"}
    out["load_check"] = history + [_brief(other, why)]
    return out


def refusal_lines(msg: str, limit: int = 4) -> str:
    """The bench's own FAIL lines from a refusal message, or the message's
    first line: the reason a draft names, without two pages of bench output."""
    fails = [l.strip() for l in msg.splitlines() if l.strip().startswith("FAIL")]
    return "; ".join(fails[:limit]) if fails else (msg.strip().splitlines() or [""])[0][:300]


def measure_bench(stored: dict, keep_stale: bool = False, runner=None) -> tuple[dict, dict | None]:
    """The wp3_bench section and, when the fresh bench refused itself and
    `keep_stale` was asked for, the record of that refusal.

    Normally a refused bench stops the data step: the deck must not quote
    timing for code nobody could time. On 7 Oct the bench refused the tree's
    own code (the new Shield and the frozen legacy loop disagreed on `filter`
    decisions after another change added `Repair.magnitude_m`), while the
    review still needed a deck. `keep_stale` keeps the stored runs, which
    timed earlier code, and returns {measured, tree_code, why}; `verify` still
    reports the runs as other code (exit 3), so only a DRAFT can be built from
    them. Without stored runs of both configurations there is nothing to keep,
    and the refusal stands."""
    runner = runner or run_bench
    try:
        return {k: pick_bench(stored.get(k), runner(**v)) for k, v in BENCH_RUNS.items()}, None
    except BenchRefused as e:
        if not keep_stale or not all(stored.get(k) for k in BENCH_RUNS):
            raise
        return ({k: stored[k] for k in BENCH_RUNS},
                {"measured": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
                 "tree_code": code_fingerprint(), "why": refusal_lines(str(e))})


PROFILE_SRC = {
    "performance_cores": ROOT / "deploy" / "evidence" / "dev" / "shield_tick_profile.json",
    "efficiency_cores": ROOT / "deploy" / "evidence" / "dev" / "shield_tick_profile-ecores.json",
}
# The arm and horizon the grant states its budget at: 50 rules, 5 s at 0.1 s.
PROFILE_ARM = "design_load/grant"
PROFILE_STATS = ("n", "median_ms", "p99_ms", "max_ms", "over_budget", "budget_ms")


def profile_facts(src: Path) -> dict | None:
    """One tools/profile_shield_tick.py result at the grant's load: the
    Shield replayed over real flight ticks with no-fly zones added along each
    path up to 50 rules, 5 s horizon at 0.1 s.

    None when the file is absent; the deck then leaves the sentence out,
    never prints a zero. `quotable` is False, with `why_not`, when the
    profile itself says its timing cannot be trusted: a failure, a host whose
    speed moved during the run, or a missing figure."""
    if not Path(src).exists():
        return None
    d = jload(src)
    arm = (d.get("pooled") or {}).get(PROFILE_ARM) or {}
    why = []
    if d.get("failures"):
        why.append(f"failures: {d['failures']}")
    if (d.get("headline") or {}).get("host_steady") is not True:
        why.append("the host's speed was not steady")
    out = {"source": rel(src), "command": d.get("command"), "created": d.get("created_utc"),
           "code_revision": d.get("code_revision"), "topology": d.get("topology"),
           "condition": (d.get("headline") or {}).get("condition"),
           "cpu_affinity": d.get("cpu_affinity")}
    for fn in ("check", "filter"):
        s = arm.get(fn) or {}
        missing = [k for k in PROFILE_STATS
                   if isinstance(s.get(k), bool) or not isinstance(s.get(k), (int, float))]
        if missing:
            why.append(f"{PROFILE_ARM} {fn} has no {missing}")
        out[fn] = {k: (round(s[k], 3) if isinstance(s.get(k), float) else s.get(k))
                   for k in PROFILE_STATS}
    out["quotable"] = not why
    out["why_not"] = "; ".join(why) or None
    return out


def wp3_profile_facts(srcs: dict = PROFILE_SRC) -> dict:
    return {k: profile_facts(p) for k, p in srcs.items()}


def verify_status(probs: list[str]) -> int:
    """--verify's exit code: 0 when the stored file is current; 3 when the
    only difference is stored Shield timing of other code (a DRAFT may still
    be built from it); 1 for any other difference, an absent bench included."""
    if not probs:
        return 0
    return 3 if all(p.startswith("wp3_bench.") and ": timed code " in p for p in probs) else 1


def fsm_facts() -> dict:
    from guardrail import fsm as F
    c = F.FSMConfig()
    return {"source": "guardrail/fsm.py FSMConfig, FSMState",
            "states": [s.value for s in F.FSMState],
            "n_violations": c.n_violations, "window_s": c.window_s,
            "theta_lateral_m": c.theta_lateral_m, "theta_vertical_m": c.theta_vertical_m,
            "t_recover_s": c.t_recover_s, "loiter_timeout_s": c.loiter_timeout_s,
            "actions": list(F.ACTIONS),
            "failsafe_target": 0.99}


# --------------------------------------------------------------------------- #
# WP4 - stress profiles, KPI report, paraphraser
# --------------------------------------------------------------------------- #

def wp4_facts() -> dict:
    import collections
    from guardrail.scenario_spec import Library, expand, load_profile
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    prof = {}
    templates: collections.Counter = collections.Counter()
    stressors: collections.Counter = collections.Counter()
    for name in ("smoke", "nightly"):
        p = load_profile(ROOT / "experiments" / "profiles" / f"{name}.yaml")
        eps, info = expand(lib, p)
        prof[name] = {"cells": info["cells"], "episodes": len(eps), "seeds": list(p.seeds),
                      "grant_scope": info.get("grant_scope"), "cadence": info.get("cadence")}
        if name == "nightly":
            for e in eps:
                templates[e.cell.spec.template] += 1
                st = e.cell.spec.stress
                for k, v in (st.model_dump().items() if st else ()):
                    if v not in (0, 0.0, None):
                        stressors[k] += 1
    from experiments.sweep_scenarios import MANIFEST_FIELDS
    roll = jload(ROOT / "docs" / "data" / "kpi_rollup_2026-10-06.json")
    return {"profiles": prof, "templates": sorted(templates), "n_templates": len(templates),
            "stressors": sorted(stressors), "manifest_fields": list(MANIFEST_FIELDS),
            "kpi_report": {"source": "docs/data/kpi_rollup_2026-10-06.json",
                           "command": roll["command"],
                           "kpis": [k for k in roll["definitions"] if k != "Family"]}}


def paraphraser_facts() -> dict:
    d = ROOT / "experiments" / "paraphrases"
    v = jload(d / "validation_report.json")
    null = v["null_validators"]["accept_all_but_exact_copy"]
    ex = jload(d / "task_follow_a_red_car.json")
    kinds: dict[str, int] = {}
    for f in sorted(d.glob("*.json")):
        doc = jload(f)
        if isinstance(doc, dict) and doc.get("schema") == "guardrail.paraphrases/1":
            kinds[doc["kind"]] = kinds.get(doc["kind"], 0) + 1
    if sum(kinds.values()) != v["sources"]:
        raise SystemExit(f"paraphrase files ({sum(kinds.values())}) disagree with the "
                         f"validation report ({v['sources']} sources)")
    return {"source": rel(d / "validation_report.json"), "command": v["command"],
            "source_kinds": kinds,
            "sources": v["sources"], "paraphrases": v["paraphrases"], "accepted": v["accepted"],
            "per_source": v["paraphrases"] // v["sources"] if v["sources"] else None,
            "mutants": v["mutants"], "mutants_refused": v["mutants_refused"],
            "mutation_classes": len(v["mutants_by_class"]),
            "null_copy_only_refused": null["mutants_refused"],
            "generator": ex["provenance"]["generator"],
            "example": {"source": ex["source_text"],
                        "paraphrases": [p["text"] for p in ex["paraphrases"]]}}


# --------------------------------------------------------------------------- #
# The city no-fly-zone flight
# --------------------------------------------------------------------------- #

def point_in_poly(x: float, y: float, verts: list[tuple[float, float]]) -> bool:
    inside = False
    n = len(verts)
    for i in range(n):
        x1, y1 = verts[i]
        x2, y2 = verts[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


def dist_to_poly(x: float, y: float, verts: list[tuple[float, float]]) -> float:
    """Distance to the polygon's boundary; 0 inside it."""
    if point_in_poly(x, y, verts):
        return 0.0
    best = math.inf
    n = len(verts)
    for i in range(n):
        (x1, y1), (x2, y2) = verts[i], verts[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
        best = min(best, math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)))
    return best


def zone_summary(track: list[tuple[float, float]], verts: list[tuple[float, float]]) -> dict:
    inside = sum(1 for x, y in track if point_in_poly(x, y, verts))
    return {"ticks": len(track), "ticks_inside": inside,
            "min_distance_m": round(min(dist_to_poly(x, y, verts) for x, y in track), 2)
            if track else None}


def within(drone: list[tuple[float, float]], car: list[tuple[float, float]], r: float) -> float | None:
    """Share of ticks with the car within r metres of the drone; None with no ticks."""
    pairs = [(d, c) for d, c in zip(drone, car) if c is not None]
    if not pairs:
        return None
    return sum(1 for d, c in pairs if math.hypot(d[0] - c[0], d[1] - c[1]) <= r) / len(pairs)


def truth_points(row: dict) -> list[tuple[float, float]] | None:
    """Where the car truly is on this tick (demo/track_truth.truth_points'
    rule, copied so this module needs no demo/ import); None when unlogged."""
    t = row.get("truth")
    if isinstance(t, dict):
        pts = [(float(a), float(b)) for a, b in (t.get("pts") or [])]
        return pts or None
    if row.get("tgt_x") is not None and row.get("tgt_y") is not None:
        return [(float(row["tgt_x"]), float(row["tgt_y"]))]
    return None


def estimate_on(rows: list[dict], near_m: float, est_of=None) -> float | None:
    """Share of ticks on which the target estimate was within `near_m` of the
    car, over the ticks that have both a served estimate (`est_xy`) and a
    logged truth - demo/track_truth.score_estimate's definition. `est_of(row)`
    replaces the estimate (a null estimator) on the same ticks. None when no
    tick has both."""
    n = on = 0
    for r in rows:
        pts = truth_points(r)
        if not r.get("est_xy") or not pts:
            continue
        e = est_of(r) if est_of else r["est_xy"]
        n += 1
        on += int(min(math.hypot(e[0] - a, e[1] - b) for a, b in pts) <= near_m)
    return on / n if n else None


def estimate_null_fixed_start(rows: list[dict], near_m: float) -> float | None:
    """The estimate's null: an estimator that never moves from the car's first
    true position, scored on the same ticks by the same rule."""
    first = next((truth_points(r)[0] for r in rows if truth_points(r)), None)
    if first is None:
        return None
    return estimate_on(rows, near_m, est_of=lambda r: first)


# The flight figures the deck prints. A missing one stops the build: a slide
# must not show 'car within 30 m 0.0 %' because a metrics key was renamed.
NFZ_QUOTED = ("zone_entered", "zone_ticks_inside", "zone_min_distance_m", "within_30m",
              "within_30m_recomputed", "within_30m_null_hover", "estimate_on_car",
              "estimate_on_car_recomputed", "estimate_null_fixed_start", "estimate_near_m",
              "alt_min_m", "alt_max_m", "fence_aim_ticks", "sep_mean_m", "cruise_alt_m")


def nfz_flight(d: Path | None = None, policy: Path = NFZ_POLICY) -> tuple[dict, dict]:
    from guardrail.models import load_policy
    d = d or (ROOT / "demo" / "out" / NFZ_TAG)
    m = jload(d / "metrics.json")
    man = jload(d / "manifest.json")
    rows = [json.loads(l) for l in (d / "flight_log.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
    if not rows:
        raise SystemExit(f"{rel(d / 'flight_log.jsonl')} has no ticks")
    pol = load_policy(policy)
    zone = next(c for c in pol.constraints if c.type == "polygon_fence")
    alt = next(c for c in pol.constraints if c.type == "altitude_envelope")
    clr = next(c for c in pol.constraints if c.type == "obstacle_clearance")
    verts = [(float(v.x), float(v.y)) for v in zone.vertices]
    drone = [(r["x"], r["y"]) for r in rows]
    car = [(truth_points(r) or [None])[0] for r in rows]
    start = drone[0]
    hud = m.get("policy_hud") or {}
    est = m.get("estimate_on_subject") or {}
    near = est.get("near_m")
    zs = zone_summary(drone, verts)
    w30 = within(drone, car, 30.0)
    w30_null = within([start] * len(drone), car, 30.0)
    est_re = estimate_on(rows, near) if near is not None else None
    est_null = estimate_null_fixed_start(rows, near) if near is not None else None
    fact = {
        "tag": d.name, "source": rel(d / "metrics.json"),
        "date": "2026-10-03", "sim": "Project AirSim, CityLife level",
        "pilot": "follow controller (demo/follow_vlm.py) with the Safety Shield in the loop",
        "topology": man.get("topology"), "object": m.get("object"),
        "duration_s": round(rows[-1]["t"], 1), "ticks": len(rows),
        "policy_id": pol.policy_id, "policy_version": pol.version,
        "policy_rules": len(pol.constraints),
        "zone_id": zone.id, "zone_vertices": verts, "zone_margin_m": zone.margin_m,
        "zone_entered": m.get("nfz_entered"), "zone_s": m.get("nfz_s"),
        "zone_ticks_inside": zs["ticks_inside"], "zone_min_distance_m": zs["min_distance_m"],
        "within_30m": m.get("frac_within_30m"),
        "within_30m_recomputed": round(w30, 3) if w30 is not None else None,
        "within_30m_null_hover": round(w30_null, 3) if w30_null is not None else None,
        "estimate_on_car": est.get("on_subject_frac"),
        "estimate_on_car_recomputed": round(est_re, 3) if est_re is not None else None,
        "estimate_null_fixed_start": round(est_null, 3) if est_null is not None else None,
        "estimate_near_m": near,
        "sep_mean_m": m.get("sep_mean_m"),
        "cruise_alt_m": (m.get("params") or {}).get("cruise_alt"),
        "alt_min_m": round(min(r["up"] for r in rows), 1), "alt_max_m": round(max(r["up"] for r in rows), 1),
        "alt_band_m": [alt.alt_min_m, alt.alt_max_m], "clearance_rule_m": clr.min_clearance_m,
        "collisions_mission": (m.get("collisions") or {}).get("mission"),
        "fence_aim_ticks": m.get("fence_aim_ticks"),
        "banner_ticks": hud.get("banner_ticks"), "hud_counts": hud.get("counts"),
        "shield_repaired_ticks": (hud.get("counts") or {}).get("repaired"),
    }
    missing = [k for k in NFZ_QUOTED if fact[k] is None]
    if missing:
        raise SystemExit(f"{d.name}: no value for {missing}; the deck would print a hole "
                         f"or a zero there")
    # The deck quotes metrics.json; the log must say the same thing.
    for a, b in (("within_30m", "within_30m_recomputed"),
                 ("estimate_on_car", "estimate_on_car_recomputed")):
        if abs(fact[a] - fact[b]) > 0.02:
            raise SystemExit(f"{d.name}: {a} {fact[a]} in metrics.json, {fact[b]} from the "
                             f"flight log; refusing to quote either")
    track ={"drone": drone, "car": [c for c in car if c is not None], "verts": verts}
    return fact, track


def draw_track_chart(track: dict, out: Path) -> dict:
    """Top-down chart of the flight: buildings at flight height, the zone, the
    car's path and the drone's track, whole flight and beside the zone."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    import numpy as np
    from matplotlib.patches import Polygon as MplPoly
    fam = "DejaVu Sans"
    for f in Path.home().joinpath("AppData/Local/Microsoft/Windows/Fonts").glob("Poppins-*.ttf"):
        font_manager.fontManager.addfont(str(f))
        fam = "Poppins"
    plt.rcParams.update({"font.family": fam, "font.size": 9.5})
    DRONE, ZONE, CAR, BLD, INK, GREY = "#0E93AB", "#C0392B", "#6B7280", "#DDE2E7", "#2B2B2B", "#6B7280"
    occ = np.load(OCC_BAND)
    grid, res, ox, oy = occ["occ"], float(occ["res"]), float(occ["origin_x"]), float(occ["origin_y"])
    ext = [oy - res / 2, oy + (grid.shape[1] - 0.5) * res, ox - res / 2, ox + (grid.shape[0] - 0.5) * res]
    # x is North, y is East (NED); north up: the plot's horizontal is y.
    dy = [p[1] for p in track["drone"]]; dx = [p[0] for p in track["drone"]]
    cy = [p[1] for p in track["car"]]; cx = [p[0] for p in track["car"]]
    zv = [(v[1], v[0]) for v in track["verts"]]
    fig = plt.figure(figsize=(8.9, 2.85), dpi=220)
    gs = fig.add_gridspec(1, 3, width_ratios=[1.0, 1.42, 0.8], wspace=0.08,
                          left=0.01, right=0.99, top=0.9, bottom=0.04)
    zy0, zy1 = min(v[0] for v in zv), max(v[0] for v in zv)
    zx0, zx1 = min(v[1] for v in zv), max(v[1] for v in zv)
    views = [("Whole flight", (min(dy + cy) - 8, max(dy + cy) + 8), (min(dx + cx) - 8, max(dx + cx) + 8)),
             ("Beside the zone", (zy0 - 34, zy1 + 30), (zx0 - 28, zx1 + 14))]
    axes = []
    for k, (title, (y0, y1), (x0, x1)) in enumerate(views):
        ax = fig.add_subplot(gs[0, k])
        cmap = matplotlib.colors.ListedColormap(["#FFFFFF", BLD])
        ax.imshow((grid > 0).astype(int), origin="lower", extent=ext, cmap=cmap,
                  interpolation="nearest", zorder=0)
        ax.add_patch(MplPoly(zv, closed=True, facecolor=ZONE, alpha=0.22, edgecolor="none", zorder=1))
        ax.add_patch(MplPoly(zv, closed=True, facecolor="none", edgecolor=ZONE, lw=1.6, zorder=2))
        ax.plot(cy, cx, color=CAR, lw=1.2, ls=(0, (4, 3)), zorder=3)
        ax.plot(dy, dx, color="#FFFFFF", lw=3.6, zorder=4, solid_capstyle="round")
        ax.plot(dy, dx, color=DRONE, lw=2.0, zorder=5, solid_capstyle="round")
        ax.plot([dy[0]], [dx[0]], "o", ms=6, color=DRONE, mec="white", mew=1.2, zorder=6)
        ax.set_xlim(y0, y1); ax.set_ylim(x0, x1); ax.set_aspect("equal")
        ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_color("#E3E8EC")
        axes.append((ax, title))
        if k == 0:
            ax.annotate("start", (dy[0], dx[0]), xytext=(6, -11), textcoords="offset points",
                        fontsize=8.5, color=GREY)
        else:
            ax.text((zy0 + zy1) / 2, zx1 + 2.0, f"no-fly zone", ha="center", va="bottom",
                    fontsize=9, color=ZONE, fontweight="bold")
            ax.plot([y0 + 4, y0 + 14], [x0 + 4, x0 + 4], color=INK, lw=1.2)
            ax.text(y0 + 9, x0 + 5.2, "10 m", ha="center", va="bottom", fontsize=8, color=GREY)
    # Equal-aspect panels end at different heights: put both titles on one line.
    fig.canvas.draw()
    top = max(a.get_position().y1 for a, _ in axes)
    for a, title in axes:
        fig.text(a.get_position().x0, top + 0.025, title, fontsize=10, color=INK,
                 fontweight="bold", va="bottom")
    lax = fig.add_subplot(gs[0, 2]); lax.axis("off")
    # North arrow in the legend column, where it covers no data.
    lax.annotate("", (0.1, 0.95), xytext=(0.1, 0.78), xycoords="axes fraction",
                 arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.1))
    lax.text(0.2, 0.86, "North up", transform=lax.transAxes, fontsize=9, color=GREY,
             va="center")
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    handles = [Line2D([], [], color=DRONE, lw=2.0, label="drone track"),
               Line2D([], [], color=CAR, lw=1.2, ls=(0, (4, 3)), label="red car, ground truth"),
               Patch(facecolor=ZONE, alpha=0.35, edgecolor=ZONE, label="no-fly zone"),
               Patch(facecolor=BLD, edgecolor="none", label="buildings, 6-14 m")]
    lax.legend(handles=handles, loc="center left", frameon=False, fontsize=9, labelcolor=INK,
               handlelength=2.2, borderaxespad=0.0)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=220, facecolor="white")
    plt.close(fig)
    return {"path": rel(out), "size_in": [8.9, 2.85]}


# --------------------------------------------------------------------------- #

def compute_sections() -> tuple[dict, dict]:
    """Every section of progress_oct.json except the bench, recomputed from
    its sources (no file is written), and the flight track for the chart."""
    from guardrail.manifest import code_revision
    flight, track = nfz_flight()
    release = release_facts()
    return {
        "code_revision": code_revision(),
        "status_date": release["status_date"],
        "release": release,
        "wp1": wp1_facts(),
        "wp2": wp2_facts(),
        "wp3_profile": wp3_profile_facts(),
        "wp3_fsm": fsm_facts(),
        "wp4": wp4_facts(),
        "paraphraser": paraphraser_facts(),
        "nfz_flight": flight,
    }, track


def _short(v) -> str:
    s = json.dumps(v, ensure_ascii=False)
    return s if len(s) <= 70 else s[:67] + "..."


def diff(a, b, path: str = "") -> list[str]:
    """Every place two JSON values differ, as 'path: stored X, now Y'."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b), key=str):
            out += diff(a.get(k, "<absent>"), b.get(k, "<absent>"), f"{path}.{k}" if path else str(k))
        return out
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        out = []
        for i, (x, y) in enumerate(zip(a, b)):
            out += diff(x, y, f"{path}[{i}]")
        return out
    return [] if a == b else [f"{path}: stored {_short(a)}, now {_short(b)}"]


def verify(stored: dict, fresh: dict, sha: str) -> list[str]:
    """Why the stored progress_oct.json no longer describes the repository:
    each recomputed section that differs, and each bench run that timed code
    other than what is in the tree now (`code_sha`). Empty means current."""
    fresh = json.loads(json.dumps(fresh))          # tuples -> lists, as stored
    probs = []
    for k, v in fresh.items():
        probs += diff(stored.get(k, "<absent>"), v, k)
    bench = stored.get("wp3_bench") or {}
    for name in BENCH_RUNS:
        run = bench.get(name)
        if run is None:
            probs.append(f"wp3_bench.{name}: absent")
        elif run.get("code_sha") != sha:
            probs.append(f"wp3_bench.{name}: timed code {run.get('code_sha')}, the tree is "
                         f"{sha} now; re-run the data step (the bench runs again)")
    return probs


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--skip-bench", action="store_true",
                    help="reuse wp3_bench from the existing progress_oct.json (refused "
                         "if the stored runs timed other code)")
    ap.add_argument("--bench-replace", action="store_true",
                    help="quote the fresh bench run and drop the stored ones")
    ap.add_argument("--keep-stale-bench", action="store_true",
                    help="if the fresh bench refuses itself, keep the stored runs (other "
                         "code) and record why; review drafts only")
    ap.add_argument("--check-deck", type=Path, default=None,
                    help="scan a slides_text.json for withdrawn claims and exit")
    ap.add_argument("--cover", type=Path, default=None,
                    help="with --check-deck: also check this README's cover note")
    ap.add_argument("--verify", action="store_true",
                    help="recompute every section but the bench, compare with the "
                         "stored progress_oct.json, check the bench's code_sha. Exit 0 "
                         "current, 3 only the bench timed other code, 1 otherwise. "
                         "Writes nothing.")
    args = ap.parse_args(argv)

    if args.check_deck:
        slides = jload(args.check_deck)
        probs = check_deck(slides)
        if args.cover:
            probs += check_cover(args.cover.read_text(encoding="utf-8"))
        for p in probs:
            print("REFUSED:", p)
        cov = " and the cover note" if args.cover else ""
        print(f"deck text check: {len(slides)} slides{cov}, {len(probs)} problem(s)")
        return 1 if probs else 0

    old = jload(OUT_JSON) if OUT_JSON.exists() else {}
    sha = code_fingerprint()

    if args.verify:
        if not old:
            print("REFUSED: no", rel(OUT_JSON), "to verify")
            return 1
        fresh, _ = compute_sections()
        probs = verify(old, fresh, sha)
        for p in probs:
            print("STALE:", p)
        status = verify_status(probs)
        print(f"verify {rel(OUT_JSON)} (generated {old.get('generated')}): "
              f"{len(probs)} difference(s)"
              + ("; only the stored Shield timing is of other code (exit 3)" if status == 3 else ""))
        return status

    refused = None
    if args.skip_bench:
        stale = [k for k in BENCH_RUNS
                 if ((old.get("wp3_bench") or {}).get(k) or {}).get("code_sha") != sha]
        if stale:
            print(f"--skip-bench refused: no stored bench run of the code in the tree now "
                  f"({', '.join(stale)}); run without --skip-bench")
            return 2
        bench = old["wp3_bench"]
    else:
        stored = {} if args.bench_replace else (old.get("wp3_bench") or {})
        try:
            bench, refused = measure_bench(stored, keep_stale=args.keep_stale_bench)
        except BenchRefused as e:
            print(f"REFUSED: the Shield bench refused itself, so there is no timing of the "
                  f"code in the tree to quote: {refusal_lines(str(e))}")
            print("Nothing written. Fix the bench, or build a review draft with "
                  "--keep-stale-bench (the deck step then needs --allow-dirty).")
            return 2
        if refused:
            print(f"KEPT STALE BENCH (draft only): {refused['why']}")

    secs, track = compute_sections()
    data = {
        "generated_by": "tools/deck/build_progress_oct_data.py",
        "generated": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        **{k: secs[k] for k in ("code_revision", "status_date", "release", "wp1", "wp2")},
        "wp3_bench": bench,
        # Present only when --keep-stale-bench kept runs of other code.
        **({"bench_refused": refused} if refused else {}),
        # The rule that chose the quoted run, for the README that explains it.
        "bench_pick_rule": {"load_tolerance": LOAD_TOLERANCE, "same": list(BENCH_SAME)},
        **{k: secs[k] for k in ("wp3_profile", "wp3_fsm", "wp4", "paraphraser", "nfz_flight")},
        "chart": draw_track_chart(track, CHART),
    }
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_JSON, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(data, indent=1, ensure_ascii=False) + "\n")
    print("wrote", rel(OUT_JSON), "and", rel(CHART))
    g = bench["grant_load"]
    w1 = data["wp1"]
    print(f"WP1 round trip {w1['round_trip_passed']}/{w1['policies']}, "
          f"tamper refused {w1['tamper_refused']}/{w1['tamper_run']} (null {w1['null_tamper_refused']}), "
          f"broken policies refused {w1['negative_refused']}/{w1['negative_run']} "
          f"(null {w1['null_negative_refused']})")
    w2 = data["wp2"]
    print(f"WP2 P0 {w2['p0_covered']}/{w2['p0_in_scope']} (empty CSP {w2['null_p0_covered']}, "
          f"naive cut {w2['baseline_p0_covered']}; discriminates {w2['kpi_discriminates']}), "
          f"exact tokens {w2['tokens_exact_range']} of {w2['budget_tokens']}")
    print(f"WP3 grant load ({g['poses_per_check']} poses, code {g['code_sha']}): near _check "
          f"{g['near/_check']['legacy']['median_ms']} -> {g['near/_check']['current']['median_ms']} ms "
          f"(floor {g['near/_check']['floor']['median_ms']}), near filter "
          f"{g['near/filter']['legacy']['median_ms']} -> {g['near/filter']['current']['median_ms']} ms; "
          f"{len(g.get('load_check') or [])} run(s) in load_check"
          + (f"; STALE (timed {g['code_sha']}, tree {refused['tree_code']})" if refused else ""))
    for name, pf in data["wp3_profile"].items():
        if pf is None:
            print(f"WP3 replay profile, {name}: absent")
        else:
            c = pf["check"]
            print(f"WP3 replay profile, {name}: check p99 {c['p99_ms']} ms, max {c['max_ms']} ms, "
                  f"{c['over_budget']} of {c['n']} ticks over {c['budget_ms']} ms"
                  + ("" if pf["quotable"] else f"; NOT QUOTABLE: {pf['why_not']}"))
    w4 = data["wp4"]["profiles"]
    print(f"WP4 smoke {w4['smoke']['episodes']}, nightly {w4['nightly']['episodes']}; "
          f"paraphrases {data['paraphraser']['paraphrases']}")
    f = data["nfz_flight"]
    print(f"NFZ {f['tag']}: within 30 m {f['within_30m']} "
          f"(recomputed {f['within_30m_recomputed']}, hover null {f['within_30m_null_hover']}), "
          f"estimate on car {f['estimate_on_car']} (recomputed {f['estimate_on_car_recomputed']}, "
          f"fixed-at-start null {f['estimate_null_fixed_start']}), "
          f"zone ticks inside {f['zone_ticks_inside']}, min distance {f['zone_min_distance_m']} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
