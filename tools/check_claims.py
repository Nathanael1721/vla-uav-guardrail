"""Fail when a withdrawn claim is stated again, or a corrected value goes missing.

    python tools/check_claims.py                 # default scope: README, docs, tools
    python tools/check_claims.py docs/X.md ...   # just these files
    python tools/check_claims.py --json          # machine-readable result

WHY THIS EXISTS

`CHANGELOG.md` records every claim this project withdrew, under `Retracted`.
Withdrawing a claim once does not stop it coming back: a report generator
rebuilt from an old template, a deck slide copied from September, or a summary
paraphrased from a document that still carried it. On 6 Oct the docs-claims
work found the same withdrawn sentences in four generators, three living
documents and two docstrings, after they had been "corrected". This scan turns
that search into a check that runs with the tests.

THE RULES

1. CLAIMS. Each withdrawn claim has a pattern and the CHANGELOG retraction it
   comes from. A hit is a LIVE claim unless it is clearly a quotation of the
   old wording:
     (a) it sits in a table column whose header says it holds old wording
         ("Old wording", "Jangan bilang", ...); or
     (b) it is inside quotation marks, ~~strikethrough~~ or `code`, AND a
         retraction marker ("Retracted", "withdrawn", "Was:", "Corrected
         2026-10-06", "old wording", a correction row id such as "(A14)")
         is within three lines. In a table, the marker must be in the same
         row: correction tables are full of them and would otherwise excuse
         any row.
   Quoting without a retraction marker is still a live claim: a sentence in
   quotes on a slide is a sentence on a slide, and a quotation attributed to
   a source ("the grant says", "published result:") is the claim restated
   with a source, which is how the C-02 error was made in the first place.
   Until 2026-10-06 (review) generic words - "said", "says", "published",
   "reported as", "memo" - counted as attribution and let exactly that pass.
   In code (.py, .js) and JSON (.json) a string literal's own delimiters are
   not quotation marks: "Hard KPI" in a JS string is printed as Hard KPI.
   Only a quote mark INSIDE the printed text (\" or a " in a '...' or `...`
   literal, or curly quotes) quotes it. Comments and docstrings are prose.
   Until the same review every JSON string value counted as "quoted", so a
   site.json claim next to a "published" key passed.
2. REQUIRED. Some files must keep a corrected value (the recomputed detector
   range, the >= 99 % fail-safe target, the "of 33 measurable" denominator,
   the flight split 34 + 4 + 3). Deleting or reverting it fails even when no
   withdrawn wording is added.
3. FROZEN. Delivered documents and historical records (the 14 Sept
   mid-evaluation report, the August midterm report, the finding documents,
   the audit, the worklog) are not edited after delivery; their corrections
   live in docs/CORRECTION-*.md and CHANGELOG.md. Hits there are COUNTED and
   listed, never hidden, but do not fail the scan. `--strict` makes them fail.

WHAT IT CANNOT DO

It matches fixed patterns. A withdrawn claim reworded outside its pattern gets
past it; a new retraction needs a new pattern here (tests/test_check_claims.py
fails if a CHANGELOG Retracted bullet has no pattern entry). It is a guard
against regression, not a substitute for reading the text.

Exit 1 on any live claim or missing required value. Prints
"N/M files clean, L live claims, R missing required, F frozen hits".
"""
from __future__ import annotations

import argparse
import bisect
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# --------------------------------------------------------------------------- #
# Withdrawn claims: (id, pattern, CHANGELOG retraction it comes from)
# --------------------------------------------------------------------------- #

CLAIMS: list[tuple[str, str, str]] = [
    # 2026-10-06 retractions and the contract audit cards behind them
    # "canonical HIL" is the grant's own name for the hil topology, so the
    # pattern is the desktop rail CALLED that, not the words alone.
    ("C-01", r"canonical[- ]HIL runs?|[Cc]anonical[- ]HIL\s*[·:]\s*ArduPilot|CANONICAL HIL|"
             r"KPI results\s*[—-]+\s*canonical HIL|[Cc]anonical-HIL KPI gate|"
             r"KPI-grade \(canonical-hil|[Oo]ur canonical-hil is|"
             r"(desktop|SITL \+ MAVROS 2?)[^.\n]{0,40}(is|as|called) (the )?(grant's )?canonical",
     "2026-10-06: 'canonical HIL' for the desktop rail (it is the grant's dev topology)"),
    ("C-01", r"grant's canonical (topology|ArduPilot)|(on|from) the canonical (rail|topology)|"
             r"measured[^.\n]{0,40}on the canonical", "2026-10-06: canonical HIL"),
    ("C-01", r"contractual KPI gate|contractual gate is cleared|KPI gate cleared",
     "2026-10-06: canonical HIL / gate cleared"),
    ("C-01", r"contractual KPIs are unchanged since the canonical", "2026-10-06: canonical HIL"),
    ("C-01", r"rail (KPI )?kanonik|[Ss]emua angka kontrak dari sana|fully canonical|"
             r"sepenuhnya kanonik", "2026-10-06: canonical HIL"),
    ("C-01", r"Tracking evidence becomes contractual|so tracking evidence becomes contractual",
     "2026-10-06: canonical HIL (correction A17)"),
    ("C-02", r"\b[Hh]ard KPI\b",
     "2026-10-06: P0 escape presented as the one contract KPI"),
    ("C-02", r"on all (41|\d+|\$\{[^}]*\})\s+shielded flights|0\.0 on every flight",
     "2026-10-06 and 0.4.x: P0 escape on all flights"),
    ("C-02", r"Ukuran keberhasilan utama di kontrak", "2026-10-06: P0 escape as the contract KPI"),
    ("C-02", r"P0[- ]escape count on screen is the contract KPI|on[- ]screen .{0,40}is the contract KPI",
     "2026-10-05: HUD count is the contract KPI"),
    ("C-03", r"[Tt]arget\W{0,4}1\.0\b|Fail-safe correctness\",\s*\"1\.0\"",
     "2026-10-06: fail-safe target 1.0 (grant: >= 99 %)"),
    ("C-03", r"one contractual acceptance criterion|[Tt]he acceptance criterion is met on every flight",
     "2026-10-06: one acceptance KPI (grant lists five)"),
    ("C-04", r"five acceptance KPIs (are |were )?(all )?measured|[Aa]ll five (acceptance )?KPIs measured|"
             r"Five of five KPIs now measured|Kelimanya terukur|kelima KPI terukur",
     "2026-10-06: every contract KPI measured"),
    ("C-04", r"[Cc]ontractual core .{0,120}is intact|[Ii]nti kontrak utuh|Core intact but not advanced",
     "2026-10-06: contractual core intact"),
    ("C-05", r"natural language → structured mission|NL → mission|Operator text is compiled into a structured mission",
     "2026-10-06: WP2 described as NL-to-mission"),
    ("C-06", r"is a stretch goal|= stretch goal|(?<!~~\*\*Hardware flight\.\*\* )A stretch goal",
     "2026-10-06: hil hardware called a stretch goal"),
    ("C-07", r"CV, GCS, dynamic-NFZ", "2026-10-06: GCS out of scope"),
    ("C-08", r"Current open items are in|\[CLOSED 2026-09-07\] Body-frame", "2026-10-06: checklist current / item 9 closed"),
    ("C-09", r"Ini sah karena kontrak meminta|the backend is swappable by design|"
             r"[Ss]wapping the detector is inside the contract|[Cc]hanging the detector is therefore inside the contract",
     "2026-10-05: detector swap justified by the VLA-backend clause"),
    ("C-10", r"exactly as the contract designs it|Panggung uji itu sendiri tidak diminta",
     "2026-10-05: hand-written pilot plus Shield is the contract design"),
    ("C-11", r"4 rails run|Four fully-run rails|[Ff]our rails ran|Gazebo included|\(b\) WP4 evidence|"
             r"passes the mid-term gate \(P0 escape = 0\)", "2026-10-06: four rails / WP4 evidence"),
    ("C-12", r"no artefacts in the repository|no artefacts in the repo|tanpa artefak\)|tidak ada artefak di repo|"
             r"Gazebo.{0,60}never (been )?r[au]n|Gazebo never did|Gazebo.{0,40}(belum|tidak) pernah (di)?jalan",
     "2026-10-05/06: Gazebo 'no artefacts' / 'never run' (scripts exist, no run output kept)"),
    ("WP1-05", r"byte-for-byte", "2026-10-06: bundle matches the reference byte for byte"),
    ("WP2-12", r"[Cc]onstraint summary pack rendered into the model prompt|rules into prompt|"
               r"the compiled YAML prompt the VLA received|prompt the VLA receives|"
               r"Reduced version in use on the VLA path",
     "2026-10-06: the CSP reaches the model (no flown VLA has read it)"),
    ("WP4-21", r"\*\*Harness built|WP4 \| Stress testing and evidence \| Built", "2026-10-06: WP4 built"),
    # The count itself is true; the withdrawn claim is the count AS the grant's
    # KPI. The regenerated rollup states it with what it is (the Shield's own
    # re-check, theta not applied) on the same line, and that form passes.
    ("KPI-11713", r"11713\s*/\s*11713(?![^\n]*(?:[Oo][Ww][Nn] re-check|theta))",
     "2026-10-06: repair success 11713/11713 as the grant KPI"),
    # detector rate, 2026-09-29 evening
    ("det", r"3\.76\s*[-–]\s*5\.22|3\.4\s*[-–]\s*5\.2 Hz|3\.68\s*[-–]\s*5\.15",
     "2026-09-29: inflated det_hz (mission 2.77-4.31)"),
    ("det", r"\b1 (of|dari) 34\b", "2026-09-29: '1 of 34' camera flights meet both gates (0 of 33 measurable)"),
    ("det", r"clears its 4\.0 Hz gate|3\.8 Hz instead of 7", "2026-09-29: det_hz gate claims"),
    # A single stored det_hz quoted as the detector's rate (review 2026-10-06:
    # the September deck printed "detector 5.15 Hz" for city_kpi, mission
    # 4.06). The five mid-evaluation flights' stored values, the midterm
    # flights' stored values, and the midterm report's printed ones.
    ("det", r"[Dd]etector(?: rate)?\b[^.\n\d]{0,24}(3\.76|5\.15|5\.22|4\.72|4\.03|3\.63|4\.15|"
            r"4\.07|4\.56|4\.80) ?Hz",
     "2026-09-29: every det_hz published before then is inflated (CORRECTION row A14)"),
    ("det", r"DETECTOR half of the threshold is met on every scenario",
     "2026-10-06: the midterm report's detector gate (stored det_hz, not the mission rate)"),
    # The midterm deck and report's methods line (stored det_hz; mission rate of
    # those three flights 3.02-3.11 Hz, loop 7.5-8.4 Hz). Review, 2026-10-07.
    ("det", r"[Dd]etections arrive at roughly 4 Hz",
     "2026-10-07: 'roughly 4 Hz' for the midterm flights (mission 3.02-3.11 Hz)"),
    # earlier retractions
    ("09-29", r"[Nn]obody standing on a zebra", "2026-09-29: nobody on a zebra"),
    ("09-29", r"identity gate (passed|passes) held[- ]out", "2026-09-29: identity gate held out"),
    ("09-23", r"pressing into a tree and a signal pole", "2026-09-23: corner stall"),
    ("09-23", r"9\.3 Hz is the Windows", "2026-09-23: 9.3 Hz is the timer"),
    ("09-22", r"no longer measures anything on this level", "2026-09-22/23: frac_on_target measures nothing"),
    ("09-22", r"350 cm left of each cent(re|er) line", "2026-09-22: lanes left of centre"),
    ("09-2x", r"name IS its segmentation class|Paint is set per instance|[Cc]ars sat within 4 m",
     "2026-09: CityLife level claims"),
    ("0.5.1", r"wrong by tens of metres|37 m served-range error|8[- ]pixel box is the background",
     "0.5.1: standoff 31 of 516 explanation"),
    ("0.5.0", r"fired six times|fired 6 times", "0.5.0: the stand-off 'fired six times' demonstration"),
    ("share", r"Total about 58 MB", "2026-10-06: share pack size"),
    ("share", r"not yet recorded a city flight in which the\s+Shield|Shield itself acts", "2026-10-06: share note"),
    ("A10", r"Project AirSim in July|July runs on Project AirSim", "2026-10-06: OpenVLA flew on AirSimNH (classic)"),
    ("ARCH-13", r"manifest labels that rail `canonical-hil`|Its runs are this project's KPI-grade evidence|"
                r"Five KPI-grade runs exist", "2026-10-06: canonical-hil label as KPI-grade evidence"),
    ("ARCH-12", r"count as KPI figures only under a written waiver|only under a PI waiver",
     "2026-10-06 PI decision: no dev waiver; KPI runs move to hil on a Jetson Orin"),
    ("WP2-par", r"parse_command reads 6/8|6/8 as the same mission",
     "2026-10-06: the parser preview's 6/8 (the altitude fallback is fixed; 8/8)"),
    # Corrected in the chase study itself on 2026-08-20 (_inference_breakdown:
    # 66 ms idle), but still printed by the midterm deck generator until the
    # review of 2026-10-06. The correction's own wording ("was first read as a
    # fixed per-inference cost") does not match.
    ("08-20", r"\bis (a )?fixed per-inference cost",
     "2026-08-20 chase study: 287 ms read as a fixed per-inference cost (66 ms idle)"),
    # 2026-10-07: the run behind the fine-tune result flew with the
    # mission-direction assist on (goal_blend 0.55), under which the original
    # adapter also reached 5/5. Stating the result with no condition, or with
    # "goal assist off", is the withdrawn claim.
    ("FT-assist", r"100 ?% reached,? (mean )?efficiency 0\.996|"
                  r"efficiency 0\.996,? goal assist off",
     "2026-10-07: fine-tune 5/5 (efficiency 0.996) stated without its goal assist"),
]

# One sentence per pattern that states the withdrawn claim plainly. The test
# file checks that every pattern catches at least one of these and that each
# of these is caught, so a pattern that has rotted into matching nothing fails.
CLAIM_EXAMPLES: list[str] = [
    "QLoRA fine-tunes: 100 % reached, mean efficiency 0.996, goal assist off.",
    "We flew 5 canonical-HIL runs.",
    "CANONICAL HIL · ARDUPILOT SITL + MAVROS 2",
    "The SITL + MAVROS 2 rail is the grant's canonical topology.",
    "All runs were measured on the canonical rail.",
    "The contractual gate is cleared.",
    "The contractual KPIs are unchanged since the canonical-hil runs.",
    "Pengukuran di rail kanonik.",
    "Tracking evidence becomes contractual.",
    "The grant's hard KPI is P0 violation escape rate = 0.",
    "It is 0.0 on all 41 shielded flights.",
    "Ukuran keberhasilan utama di kontrak adalah P0.",
    "The P0-escape count on screen is the contract KPI.",
    "Fail-safe correctness, target 1.0.",
    "The acceptance criterion is met on every flight.",
    "All five acceptance KPIs are measured, not inferred.",
    "The contractual core (DSL, compiler, Shield) is intact.",
    "WP2 is natural language → structured mission.",
    "Hardware flight is a stretch goal.",
    "Obstacle avoidance, navigation, CV, GCS, dynamic-NFZ sourcing belong to the other team.",
    "Current open items are in the checklist.",
    "Swapping the detector is inside the contract.",
    "Most compliance comes from the pilot, exactly as the contract designs it.",
    "Four fully-run rails end-to-end.",
    "The Gazebo rail has no artefacts in the repository.",
    "The Gazebo scripts have never been run.",
    "The bundle matches the reference layout byte-for-byte.",
    "Constraint summary pack rendered into the model prompt.",
    "WP4 | Stress testing and evidence | Built",
    "Repair success: 11713/11713 repairs converged.",
    "Detector rate in flight: 3.76-5.22 Hz.",
    "Camera flights meeting both gates: 1 of 34.",
    "det_hz clears its 4.0 Hz gate in all four flights.",
    "Nobody standing on a zebra.",
    "The identity gate passed held out.",
    "The corner stall was the aircraft pressing into a tree and a signal pole.",
    "9.3 Hz is the Windows 15.6 ms timer.",
    "frac_on_target no longer measures anything on this level.",
    "Lanes sit 350 cm left of each centre line.",
    "An actor's name IS its segmentation class.",
    "The position the Shield was served was wrong by tens of metres.",
    "The 10 m stand-off fired six times.",
    "Total about 58 MB.",
    "We have not yet recorded a city flight in which the Shield itself acts.",
    "OpenVLA-7B and AerialVLA in July runs on Project AirSim.",
    "Five KPI-grade runs exist.",
    "These runs count as KPI figures only under a written waiver from the PI.",
    "parse_command reads 6/8 as the same mission.",
    "The DETECTOR half of the threshold is met on every scenario reported here.",
    "Measured with 29 objects on the street: detector 5.15 Hz.",
    "A detector whose rate only just clears its 4.0 Hz gate.",
    "Detections arrive at roughly 4 Hz while the control loop runs at 10 Hz.",
    "The P0 escape rate is 0.0 on all ${E.kpi.flights_escape_zero} shielded flights.",
    "A cost invariant to surrounding load is fixed per-inference cost, not contention.",
]

# --------------------------------------------------------------------------- #
# Every CHANGELOG.md Retracted bullet, and what guards it
# --------------------------------------------------------------------------- #
#
# Each top-level bullet under a "Retracted" heading in CHANGELOG.md must
# contain one of these keys (case-insensitive). The value names the CLAIMS ids
# that scan for it, or says why there is no sentence to scan for. A new
# retraction without an entry here fails tests/test_check_claims.py: add the
# key, and a pattern above if the withdrawn claim is a sentence someone could
# write again.
RETRACTIONS: dict[str, str] = {
    "canonical hil": "C-01",
    "contractual gate is cleared": "C-01",
    "41 shielded flights": "C-02",
    "fail-safe trigger correctness target": "C-03",
    "fail-safe target": "C-03",
    "detector rate": "det",
    "11713/11713": "KPI-11713",
    "p0-escape count on screen is the contract kpi": "C-02",
    "exactly as the contract designs it": "C-10",
    "gazebo: no artefacts": "C-12",
    "gazebo scripts have never been run": "C-12",
    "hand-written pilot because the contract makes the vla backend swappable": "C-09",
    "all five acceptance kpis measured": "C-04",
    "openvla-7b flew on project airsim": "A10",
    "shield itself acts": "share",
    "wp2/wp4 \"built\"": "WP4-21, WP2-12, C-06, C-07, C-08, WP1-05, C-11, C-05",
    "identity gate did not pass held out": "09-29",
    "replay's \"old\" arm was not what flew": "not a sentence: the replay percentages were "
                                             "re-measured; the old ones appear only in the "
                                             "corrections slide, attributed",
    "pinhole rescore": "not a sentence: a re-scoring of tracking flights",
    "nobody on a zebra": "09-29",
    "nobody standing on a zebra": "09-29",
    "every `det_hz` published before today is inflated": "det",
    "tracking score's projection was linear": "not a sentence: a scoring method, re-scored",
    "corner stall was the aircraft pressing": "09-23",
    "9.3 hz is the windows": "09-23",
    "start_heading_err_deg: null": "not a sentence: a metrics field, now measured",
    "2026-09-22 tracking columns": "09-22",
    "frac_on_target` no longer measures anything": "09-22",
    "lanes sit 350 cm": "09-22",
    "names are load-bearing": "09-2x",
    "paint is set per instance": "09-2x",
    "cars sat within 4 m": "09-2x",
    "31 of 516": "0.5.1",
    "depth sampled through an 8-pixel box": "0.5.1",
    "0.0 on every flight recorded here": "C-02",
    "fired six times": "0.5.0",
    "frac_on_target` at a 100 px tolerance": "not a sentence: a metric replaced by the "
                                             "median error against a null",
    "verified_kpi_fields": "not a sentence: a code defect, fixed with tests",
    "test runners printed `pass`": "not a sentence: a test-runner defect, fixed",
    # wave-1 (2026-10-06) unit retractions, in case they are recorded verbatim
    "validator refuses any drop": "not a sentence: the paraphraser validator's recall, re-measured",
    "1721/1721": "not a sentence: mutation recall, re-measured",
    "stored set exists for every in-scope instruction": "not a sentence: fixed by re-keying "
                                                        "(docs/FINDING-the-paraphrase-sets-keyed-"
                                                        "to-the-wrong-rendering.md)",
    "96/96 stored paraphrases": "not a sentence: one stored paraphrase was replaced",
    "text-differs-only null": "not a sentence: a null renamed",
    "theta table": "not a sentence: DESIGN-escalation-fsm.md numbers, corrected there",
    "failing-first count": "not a sentence: a test count",
    "fail-safe correctness 0.934783": "not a sentence: re-measured with its null",
    "600 passed": "not a sentence: re-measured",
    "wp3-19 done": "not a sentence: a tracker status",
    "renders byte for byte": "not a sentence: DESIGN-prefix-compiler.md, corrected there",
    "two reports differ only": "not a sentence: rows now carry csp_content_hash",
    "altitude_band_m_agl": "not a sentence: a code defect, fixed",
    "p0 coverage 105/105": "not a sentence: the KPI now reported with its baseline",
    "canonical_json is the reference's canonicalize()": "not a sentence: recorded as a deviation",
    "mid-evaluation and mid-term generators": "C-01, WP4-21, WP2-12, C-03",
    "first kpi rollup": "KPI-11713",
    "first version's false-trigger definition": "not a sentence: a definition, replaced",
    "three candidate fixes were measured and dropped": "not a sentence: fixes measured and "
                                                       "not shipped",
    "parse_command reads 6/8": "WP2-par",
    # this unit's own retractions (2026-10-06, followups-integration)
    "deck_sept_extra.json": "det",
    "detector half of the threshold": "det",
    "fixed per-inference cost": "08-20",
    "roughly 4 hz": "det",
    # 2026-10-07 (wave 2) retractions
    "read each other's bundles": "WP1-05",
    "two implementations could read": "WP1-05",
    "the fine-tune result": "FT-assist",
    "triggered = any brake": "not a sentence: the fail-safe figure, re-measured with the FSM",
    "p0-acted tick share": "not a sentence: a field published under the KPI's name, renamed",
    "goal assist": "FT-assist",
}

# The quotation is marked as withdrawn when one of these is near it. Only
# retraction markers: a dated correction, "retracted"/"withdrawn"/"was:", an
# "old wording" label, "inflated", or a correction-table row id (A1-B99).
# Source words ("said", "says", "published", "reported as", "memo") are NOT
# here: attributing a withdrawn claim to the grant or to a published result
# restates it (review, 2026-10-06).
ANNOT = re.compile(
    r"\[Dikoreksi|Dikoreksi 6 Okt|[Cc]orrected 2026-\d\d-\d\d|CORRECTED|The July text said|"
    r"The 3 October version|\bsebelumnya\b|\bWas:|\(was\b|\bwas \"|"
    r"REOPENED|NOT CLOSED|[Rr]etracted|RETRACTED|[Ww]ithdrawn|WITHDRAWN|"
    r"[Jj]angan klaim|[Oo]ld wording|[Ii]nflated|menggelembung|\bditarik\b|"
    r"\bbasi\b|[Jj]angan kutip|no longer print|old label|[Rr]etraction|used to read|"
    r"[Uu]ntil 2026-\d\d-\d\d|\(\s*[AB]\d{1,2}[),]|[ ,][AB]\d{1,2}\)|rows? [AB]\d|"
    # A slide citation in parentheses is how the correction lists number their
    # items ('**"Hard KPI ..."** (slide 2). P0 escape is one of five ...'); it
    # works like a row id. It still needs the claim to be quoted.
    r"\((deck )?slides? \d{1,2}\b")
OLD_HDR = re.compile(r"Old wording|Jangan bilang|JANGAN bilang|Published \(of 34\)")

# --------------------------------------------------------------------------- #
# Corrected values that must stay
# --------------------------------------------------------------------------- #

REQUIRED: dict[str, list[str]] = {
    "README.md": [r"not (a )?contract KPI figures?", r"no run is KPI-grade",
                  r"34 on Project AirSim, 4 on\s+ArduPilot SITL over pymavlink, 3 on ArduPilot SITL over MAVROS 2",
                  r"Python 3\.11\+"],
    "docs/index.md": [r"no run is KPI-grade",
                      r"34 on Project AirSim, 4 on\s+ArduPilot SITL over pymavlink, 3 on ArduPilot SITL over MAVROS 2"],
    "docs/scope-clarification.md": [r"≥ 99 %", r"hil"],
    "docs/prof-repo-study.md": [r"Not converged", r"no run output is kept"],
    "docs/CHECKLIST-remaining-work.md": [r"Superseded 2026-10-05", r"Item 9 is not closed",
                                         r"no run is\s+KPI-grade"],
    "docs/GAMBARAN-SISTEM.md": [r"≥ 99 %", r"tidak ada hasil run yang disimpan"],
    "docs/PROGRESS-NOTE-30Sep2026.md": [r"dev topology", r"no run output is kept"],
    "docs/PRESENTATION-CHEATSHEET-30Sep2026.md": [r"2\.77-4\.31 Hz", r"≥ 99 %"],
    "docs/share/2026-09-30-ITRI/README.md": [r"2\.77-4\.31 Hz", r"\*\*0 of 34\*\*", r"at least 99 %",
                                             r"ACTING 0", r"141 ticks", r"AirSimNH"],
    "docs/CORRECTION-2026-10-06-mid-evaluation-and-deck.md": [
        r"2\.77–4\.31 Hz", r"\*\*0 of 34\*\*", r"≥ 0\.99", r"of 33 measurable", r"1d09786",
        r"no run output is kept", r"AirSimNH"],
    "tools/build_mideval_report.py": [r"≥ 0\.99", r"measurable", r"det_hz_mission"],
    "tools/deck/build_mideval_deck.js": [r"≥ 0\.99", r"Dev topology", r"counts\[\"dev\"\]"],
    "tools/build_deck_data.py": [r"det_hz_mission", r"def bundle_issue_time"],
}

# --------------------------------------------------------------------------- #
# Frozen records: hits counted, not failing (see rule 3)
# --------------------------------------------------------------------------- #

FROZEN_GLOBS = {
    "docs/MID-EVALUATION-REPORT-Sep2026.md": "delivered 14 Sept; corrected by docs/CORRECTION-2026-10-06-*",
    "docs/MIDTERM-REPORT-Aug2026.md": "delivered 25 Aug; corrected by docs/CORRECTION-2026-10-06-* section C",
    "docs/CORRECTIONS-2026-09-02.md": "memo already given to the PI",
    "docs/AUDIT-KONTRAK-2026-10-05.md": "the audit quotes every claim it checked",
    "docs/WORKLOG.md": "chronological log",
    "docs/FINDING-*.md": "finding documents narrate the claim they disprove",
    "docs/RESULT-*.md": "dated result records",
    "docs/PRESENTATION-CHEATSHEET-Sep2026.md": "16 Sept presenter notes, unpublished",
    "docs/PRESENTATION-CHEATSHEET.md": "older presenter notes",
    "docs/PROGRESS-CHECKPOINT-v1.md": "July checkpoint",
    "docs/DEMO-SCRIPT.md": "July demo script, unpublished",
    "docs/agenda-technical-discussion.md": "meeting agenda record",
    "docs/meeting-action-items-status.md": "meeting record",
    "docs/aerialvla-ft-report.md": "July fine-tuning report",
    "docs/PI-DECISIONS-2026-10-06.md": "the questions quote the claims being corrected",
    "docs/DESIGN-*.md": "design notes record what the code did before",
}

# Records that match a frozen glob but were corrected in place on 2026-10-06
# (with a dated note) and must stay clean.
GUARDED = {
    "docs/FINDING-what-the-contract-locks-and-what-it-does-not.md",
    "docs/FINDING-forward-flew-north.md",
    "docs/FINDING-the-kpi-report-that-scored-every-rule-as-p0.md",
    "docs/FINDING-a-stripped-signature-read-as-unsigned.md",
    "docs/FINDING-the-paraphrase-sets-keyed-to-the-wrong-rendering.md",
    "docs/FINDING-the-control-arm-that-scored-a-repair-success.md",
    "docs/FINDING-the-mission-that-succeeded-inside-the-ring.md",
}

TEXT_EXT = {".md", ".py", ".js", ".json"}


def default_targets(root: Path) -> list[Path]:
    out = [root / "README.md"]
    out += sorted((root / "docs").rglob("*.md"))
    for site in ("docs/share", "docs/progress"):
        out += sorted((root / site).rglob("_src/*.json"))
    out += sorted((root / "tools").rglob("*.py"))
    out += sorted((root / "tools" / "deck").glob("*.js"))
    me = Path(__file__).resolve()
    return [p for p in out if p.is_file() and p.resolve() != me
            and "node_modules" not in p.parts]


def _rel(p: Path, root: Path) -> str:
    try:
        return p.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return p.as_posix()


def frozen_reason(rel: str) -> str | None:
    if rel in GUARDED:
        return None
    for pat, why in FROZEN_GLOBS.items():
        if Path(rel).match(pat) or rel == pat:
            return why
    return None


# --------------------------------------------------------------------------- #
# Scanning
# --------------------------------------------------------------------------- #

def _strip_quote_prefix(line: str) -> str:
    m = re.match(r"^(\s*>\s?)+", line)
    return line[m.end():] if m else line


# A single quote opens a quotation at the start of a word and closes one at
# the end of a word; the apostrophe in "grant's" does neither.
_SQ_OPEN = re.compile(r"(?<![\w'])'(?=[\w\"(\[$])")
_SQ_CLOSE = re.compile(r"(?<=[\w.,!?%)\]\"])'(?![\w'])")

# The claim directly negated is its correction: "not 'the hard KPI'", "replay
# is not byte-for-byte". Only the words right before the match count.
NEGATED = re.compile(r"(?:\bnot|\bnever|\bno longer|\binstead of)\s+(?:the\s+)?"
                     r"[\"'“‘`*]{0,3}(?:the\s+)?$", re.I)


def inside_quote(text: str, pos: int) -> bool:
    """Is `pos` inside "...", “...”, '...', ~~...~~ or `...` on this text?"""
    before = text[:pos]
    if before.count('"') % 2 == 1:
        return True
    if before.rfind("“") > before.rfind("”"):
        return True
    if before.count("~~") % 2 == 1:
        return True
    if before.count("`") % 2 == 1:
        return True
    opens = [m.start() for m in _SQ_OPEN.finditer(before)]
    if opens and not _SQ_CLOSE.search(before, opens[-1] + 1):
        return True
    return False


SOURCE_KIND = {".py": "py", ".js": "js", ".json": "json"}


def source_segments(text: str, kind: str) -> list[tuple[int, int, str]]:
    """The string-literal CONTENTS and comments of a .py / .js / .json text,
    as (start, end, delimiter) spans; the delimiter is the literal's quote
    ('"', "'", '`', '\"\"\"', "'''") or the comment marker ('#', '//', '/*').
    Everything outside the spans is code. A one-line literal left open ends
    at its line, so one odd quote cannot swallow the rest of the file."""
    delims = {"py": "\"'", "js": "\"'`", "json": "\""}[kind]
    out: list[tuple[int, int, str]] = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if (kind == "py" and c == "#") or (kind == "js" and text.startswith("//", i)):
            mark = "#" if c == "#" else "//"
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append((i + len(mark), j, mark))
            i = j
            continue
        if kind == "js" and text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j
            out.append((i + 2, j, "/*"))
            i = j + 2
            continue
        if c in delims:
            d = c * 3 if kind == "py" and text.startswith(c * 3, i) else c
            multiline = len(d) == 3 or d == "`"
            prefix = re.search(r"(?<![\w])[A-Za-z]{1,2}$", text[max(0, i - 3):i])
            raw = kind == "py" and prefix is not None and "r" in prefix.group(0).lower()
            j = i + len(d)
            start = j
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text.startswith(d, j) or (text[j] == "\n" and not multiline):
                    break
                j += 1
            j = min(j, n)
            out.append((start, j, ("r" if raw else "") + d))
            i = j + len(d) if text.startswith(d, j) else j + 1
            continue
        i += 1
    return out


def source_context(text: str, segs: list[tuple[int, int, str]], pos: int) -> str:
    """For .py / .js / .json: how does `pos` appear in what the source prints?

    "quoted": inside a quotation in the printed text. Inside a string literal
    the literal's own delimiters do not count: only a quote mark that is part
    of the printed text does (an escaped \\", a " inside a '...' or `...`
    literal, curly quotes, `code` or ~~strike~~). A comment or a docstring is
    prose and is read like a document, within its paragraph.
    "pattern": inside a Python raw string (r"..."), which in this repository
    is a regular expression - a scanner's pattern table, not a sentence.
    "plain": anything else, including code outside any literal."""
    starts = [s for s, _, _ in segs]
    k = bisect.bisect_right(starts, pos) - 1
    if k < 0:
        return "plain"
    s, e, d = segs[k]
    if not (s <= pos < e):
        return "plain"
    if d.startswith("r"):
        return "pattern"
    body = text[s:pos]
    if d in ("#", "//", "/*", '"""', "'''"):
        body = re.split(r"\n[ \t#*/]*\n", body)[-1]
    body = body.replace('\\"', '"').replace("\\'", "'")
    return "quoted" if inside_quote(body, len(body)) else "plain"


@dataclass
class Hit:
    file: str
    line: int
    claim: str
    source: str
    text: str


def _paragraphs(lines: list[str]):
    cur, start = [], None
    for i, ln in enumerate(lines):
        body = _strip_quote_prefix(ln)
        if body.strip() == "":
            if cur:
                yield start, cur
            cur, start = [], None
        else:
            if start is None:
                start = i
            cur.append(body)
    if cur:
        yield start, cur


def scan_text(text: str, rel: str) -> list[Hit]:
    lines = text.split("\n")
    kind = SOURCE_KIND.get(Path(rel).suffix)
    segs = source_segments(text, kind) if kind else []
    line_at = [0]
    for ln in lines:
        line_at.append(line_at[-1] + len(ln) + 1)
    hits: list[Hit] = []
    for start, para in _paragraphs(lines):
        is_table = all(p.lstrip().startswith("|") for p in para)
        header = [c.strip() for c in para[0].split("|")] if is_table else None
        joined = "\n".join(para)
        offs = [0]
        for p in para:
            offs.append(offs[-1] + len(p) + 1)
        for cid, rx, src in CLAIMS:
            for m in re.finditer(rx, joined, flags=re.M):
                k = max(i for i in range(len(para)) if offs[i] <= m.start())
                n = start + k
                ok = False
                if NEGATED.search(joined[max(0, m.start() - 24): m.start()]):
                    continue
                if is_table and k > 1:
                    row = para[k]
                    col = row[: m.start() - offs[k]].count("|")
                    cells = row.split("|")
                    if header and col < len(header) and OLD_HDR.search(header[col]):
                        ok = True
                    else:
                        cell = cells[col] if col < len(cells) else ""
                        cpos = m.start() - offs[k] - len("|".join(cells[:col])) - 1
                        ok = inside_quote(cell, max(cpos, 0)) and bool(ANNOT.search(row))
                else:
                    # Quotes are counted over the paragraph: a quoted sentence
                    # may open on the line before the match.
                    window = "\n".join(lines[max(0, n - 3): n + 4])
                    if kind:
                        lead = len(lines[n]) - len(para[k])   # a stripped "> " prefix
                        pos = line_at[n] + lead + (m.start() - offs[k])
                        ctx = source_context(text, segs, pos)
                        if ctx == "pattern":
                            continue
                        quoted = ctx == "quoted"
                    else:
                        quoted = inside_quote(joined, m.start())
                    ok = quoted and bool(ANNOT.search(window))
                if not ok:
                    hits.append(Hit(rel, n + 1, cid, src, lines[n].strip()[:140]))
    return hits


def check_required(text: str, rel: str) -> list[str]:
    return [rx for rx in REQUIRED.get(rel, []) if not re.search(rx, text)]


def run(targets: list[Path], root: Path, strict: bool = False) -> dict:
    live, frozen, missing, clean = [], [], [], 0
    seen = set()
    for p in targets:
        rel = _rel(p, root)
        seen.add(rel)
        if not p.is_file():
            missing.append({"file": rel, "pattern": "<file missing>"})
            continue
        if p.suffix not in TEXT_EXT:
            continue
        text = p.read_text(encoding="utf-8", errors="replace").replace("\r\n", "\n")
        hits = scan_text(text, rel)
        miss = check_required(text, rel)
        why = frozen_reason(rel)
        if why and not strict:
            frozen += [dict(h.__dict__, frozen=why) for h in hits]
            hits = []
        live += [h.__dict__ for h in hits]
        missing += [{"file": rel, "pattern": rx} for rx in miss]
        clean += int(not hits and not miss)
    # A required file that was not scanned (deleted, or outside the targets of
    # a full scan) is reported, never assumed clean.
    full = {_rel(p, root) for p in default_targets(root)}
    if seen >= full:
        for rel in REQUIRED:
            if rel not in seen:
                missing.append({"file": rel, "pattern": "<file missing>"})
    return {"files": len(targets), "clean": clean, "live": live,
            "missing_required": missing, "frozen_hits": frozen}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Withdrawn-claim and corrected-value scan.")
    ap.add_argument("paths", nargs="*", help="files to scan (default: README, docs, tools)")
    ap.add_argument("--root", default=str(ROOT))
    ap.add_argument("--strict", action="store_true", help="frozen records fail too")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--show-frozen", action="store_true")
    args = ap.parse_args(argv)
    root = Path(args.root)
    targets = [Path(p) if Path(p).is_absolute() else root / p for p in args.paths] \
        or default_targets(root)
    res = run(targets, root, strict=args.strict)
    # A Windows console is cp1252: a LIVE line quoting "≥ 0.99" or "★" used to
    # raise UnicodeEncodeError half-way through the list, so the hits after it
    # were lost (the exit code was still 1). Replace what the console cannot
    # show; --json is ASCII-escaped and survives any console.
    reconf = getattr(sys.stdout, "reconfigure", None)
    if reconf is not None:
        try:
            reconf(errors="replace")
        except (ValueError, OSError):            # pragma: no cover - closed/odd stream
            pass
    if args.json:
        print(json.dumps(res, indent=2, ensure_ascii=True))
    else:
        for h in res["live"]:
            print(f"  LIVE {h['claim']:9s} {h['file']}:{h['line']}: {h['text']}")
        for m in res["missing_required"]:
            print(f"  MISSING-REQUIRED {m['file']}: /{m['pattern']}/")
        if args.show_frozen:
            for h in res["frozen_hits"]:
                print(f"  frozen {h['claim']:9s} {h['file']}:{h['line']} ({h['frozen']})")
        print(f"{res['clean']}/{res['files']} files clean, {len(res['live'])} live claims, "
              f"{len(res['missing_required'])} missing required, "
              f"{len(res['frozen_hits'])} frozen hits")
    return 1 if (res["live"] or res["missing_required"]) else 0


if __name__ == "__main__":
    sys.exit(main())
