"""The public entry points: README.md, the GitHub Pages index (docs/index.md),
RUNBOOK.md and the CHANGELOG's preamble.

    python tests/test_public_docs.py      (no pytest needed)

WHY THIS FILE EXISTS

The review of 2026-10-19 found the front page wrong in ways no test could see:

  * "The one interface" called `shield.step(state, raw_action)`. Shield has no
    step(); the method is filter(). The same sample called
    `set_subject(x, y)` with no class, which on the very policy it loads
    (follow_pedestrian.yaml) leaves the 10 m pedestrian stand-off unarmed:
    the failure docs/FINDING-the-standoff-rule-that-never-armed.md is about,
    taught by the README;
  * the opening said a vision-language model produces the velocity command and
    that the Shield's input might have "come from OWL-ViT", while no VLA flies
    any current demo (a hand-written controller steers by OWL-ViT's boxes, the
    SITL rails fly StubVLA);
  * README and the Pages index still said no Shield-off control had been flown
    on the Project AirSim + ArduPilot rail, after the 13 and 14 Oct controls;
  * RUNBOOK.md called the one-desktop MAVROS 2 rail "the grant's exact
    topology" (it is the grant's dev topology) and was never scanned;
  * the CHANGELOG's preamble still called P0 escape "the hard KPI" (C-02);
  * the two delivered reports, published on Pages, were excused from the scan
    as "corrected by docs/CORRECTION-2026-10-06-*", a file .gitignore keeps
    out of every clone, and carried no pointer to any correction.

tools/check_claims.py scans README and docs/, but at f93283d its own test
(tests/test_check_claims.py) failed on a clean clone, because it guards
gitignored files, so it was not in CI's loop. This file runs the same scan on
these four texts only, which a clone has, plus the code sample itself. Shown
failing first, on a `git archive` copy of f93283d with this file and this
commit's tools/check_claims.py copied in: five of the seven fail. The
README-equals-index test passes there (both pages showed the same wrong
sample), and so does the test that the new patterns catch the old wording,
which needs only check_claims.py.
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import check_claims as C                                           # noqa: E402

SKIP = "SKIP"
FRONT = ("README.md", "docs/index.md")


def _text(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8").replace("\r\n", "\n")


def _sample(rel: str) -> str:
    """The first ```python block after the "## The one interface" heading."""
    text = _text(rel)
    assert "## The one interface" in text, f"{rel} lost its 'The one interface' section"
    after = text.split("## The one interface", 1)[1]
    m = re.search(r"```python\n(.*?)```", after, flags=re.S)
    assert m, f"{rel}: no python block under 'The one interface'"
    return m.group(1)


def _run_sample(rel: str):
    from guardrail import Action4D, State
    env = {"x": 8.0, "y": 0.0,                          # the subject 8 m ahead
           "state": State(x=0.0, y=0.0, up=7.0),
           "raw_action": Action4D(vx=2.0)}              # closing on it at 2 m/s
    cwd = os.getcwd()
    os.chdir(ROOT)                                       # the sample's path is relative
    try:
        exec(compile(_sample(rel), f"{rel}:one-interface", "exec"), env)
    finally:
        os.chdir(cwd)
    return env["decision"]


def test_the_one_interface_sample_runs_and_arms_the_pedestrian_standoff():
    """Executed as written, with concrete values for its placeholders: it must
    not raise (shield.step did), and the policy's 10 m pedestrian rule must
    bind (with no class only the 5 m catch-all did: ['standoff-any'])."""
    for rel in FRONT:
        decision = _run_sample(rel)
        rules = [v.rule_id for v in decision.violations]
        assert "standoff-pedestrian" in rules, (
            f"{rel}: the sample's subject 8 m ahead does not arm the 10 m "
            f"pedestrian stand-off; violations {rules}")
        assert decision.emitted is not None, f"{rel}: the comment's decision.emitted"


def test_the_readme_and_the_pages_index_show_the_same_sample():
    assert _sample("README.md") == _sample("docs/index.md"), \
        "README.md and docs/index.md show different code under 'The one interface'"


def test_the_front_page_documents_state_no_withdrawn_claim():
    """check_claims' patterns, on the files a clone has: no live claim, and
    every corrected value they must keep is still there."""
    msgs = []
    for rel in FRONT + ("RUNBOOK.md",):
        text = _text(rel)
        msgs += [f"{h.file}:{h.line} [{h.claim}] {h.text}" for h in C.scan_text(text, rel)]
        msgs += [f"{rel}: missing /{rx}/" for rx in C.check_required(text, rel)]
    assert not msgs, "\n      ".join(["withdrawn claims or missing values:"] + msgs)


def test_the_new_patterns_catch_what_the_front_page_used_to_say():
    """The withdrawn sentences, as they stood at f93283d (line breaks and all:
    check_claims joins a paragraph with newlines, so a pattern written with
    literal spaces would miss them)."""
    old = {
        "README.md": ("A vision-language model is told to *follow the yellow car*. It produces a\n"
                      "velocity command.\n"),
        "docs/index.md": ("It does not know whether that came\n"
                          "from OWL-ViT, from OpenVLA-7B, or from a joystick.\n"),
        "docs/x.md": ("On 8 Oct 2026 a 120 s follow let 5 P0 violations through (0.0053, "
                      "no\nShield-off control has been flown there yet).\n"),
        "docs/y.md": "That is a replay, not a flight: no flight has used the fix yet.\n",
        "docs/z.md": "Rail 3 (ROS 2) — grant's exact topology, one command.\n",
        "docs/w.md": "Four rails, all show the same thing: reckless VLA + Guardrail.\n",
    }
    for rel, text in old.items():
        assert C.scan_text(text, rel), f"not caught any more: {text!r}"


def test_the_front_page_says_what_flies_today():
    """The disclosure that no VLA flies the current demos must stay on both
    front pages, next to the grant's design it qualifies."""
    for rel in FRONT:
        flat = " ".join(_text(rel).split())
        assert "No VLA flies the demonstrations recorded here" in flat, rel
        assert "hand-written follow controller" in flat and "StubVLA" in flat, rel
        assert "4 Aug 2026" in flat, f"{rel}: the date of the last VLA-commanded flights"


def test_the_changelog_preamble_states_no_withdrawn_claim():
    """The preamble (before the first `## `) is the project describing itself,
    not a quotation; the rest of CHANGELOG.md quotes withdrawn claims on
    purpose and is not scanned (it holds 19 dated live hits)."""
    lines = _text("CHANGELOG.md").split("\n")
    head = []
    for ln in lines:
        if ln.startswith("## "):
            break
        head.append(ln)
    assert len(head) > 3, "no preamble found"
    hits = C.scan_text("\n".join(head), "CHANGELOG.md")
    assert hits == [], [f"{h.line} [{h.claim}] {h.text}" for h in hits]


def _in_clone(pattern: str) -> bool:
    """Whether a clone holds `pattern` (a path, or a glob with `*`). Inside a
    git work tree: `git ls-files` (a gitignored file on this disk does not
    count). Outside one, as in a `git archive` copy, which holds tracked
    files only: whether it is on disk."""
    try:
        inside = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--is-inside-work-tree"],
                                capture_output=True, text=True, timeout=20)
        if inside.returncode == 0 and inside.stdout.strip() == "true":
            r = subprocess.run(["git", "-C", str(ROOT), "ls-files", "--", pattern],
                               capture_output=True, text=True, timeout=20)
            return r.returncode == 0 and bool(r.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return any(p.is_file() for p in ROOT.glob(pattern))


DELIVERED = ("docs/MID-EVALUATION-REPORT-Sep2026.md", "docs/MIDTERM-REPORT-Aug2026.md")


def test_a_frozen_report_points_to_corrections_a_clone_has():
    """tools/check_claims.py counts a frozen record's withdrawn claims without
    failing, on the strength of the reason FROZEN_GLOBS gives. At f93283d
    the reason for both delivered reports was "corrected by
    docs/CORRECTION-2026-10-06-*", which .gitignore keeps out of every clone
    and of the Pages site, and neither report said a word about it. Every
    file a reason names must be in a clone, and each delivered report must
    open with a dated note that sends the reader to CHANGELOG.md."""
    for glob, why in C.FROZEN_GLOBS.items():
        for tok in re.findall(r"\b[\w./-]+\.md\b|\bdocs/[\w./-]*\*", why):
            assert _in_clone(tok), (f"FROZEN_GLOBS[{glob!r}] says the claims are corrected "
                                    f"in {tok}, which a clone does not have")
    for rel in DELIVERED:
        head = "\n".join(_text(rel).split("\n")[:4])
        assert re.search(r"^> \*\*Note added 20\d\d-\d\d-\d\d", head, flags=re.M), \
            f"{rel} does not open with a dated note"
        note = _text(rel).split("\n\n", 2)[1]
        assert note.startswith("> ") and "CHANGELOG.md" in note, \
            f"{rel}: its note does not point to CHANGELOG.md:\n{note[:300]}"
        assert C.frozen_reason(rel), f"{rel} is no longer frozen: the note says it is as delivered"


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
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
