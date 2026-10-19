"""The CI workflows: the scheduled nightly profile, and a per-build header that
says what is true.

    python tests/test_harnesstruth_ci.py      (no pytest needed)

WHY THIS FILE EXISTS

The grant's test schedule (Stress Testing, "Test scheduling") has three rows:
smoke per build, nightly KPI nightly, broad nightly. Until 2026-10-12 only the
first was a workflow; the nightly profile ran when somebody started it. And
the per-build workflow's header still said it had never been pushed, four
days after GitHub had been running it (red until 4418ae0). A workflow file is
read by people deciding what CI covers, so what it says is checked here.

Shown failing first: every test below fails when this file is run from a
pristine copy of HEAD 4418ae0 (`git archive HEAD | tar -x`, this file copied
into its tests/): nightly.yml does not exist there, tests.yml's header says
"NOT YET RUN ON GITHUB", and the harnesstruth test files are not listed.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WF = ROOT / ".github" / "workflows"
SKIP = "SKIP"


def _yaml(p: Path) -> dict:
    import yaml
    return yaml.safe_load(p.read_text(encoding="utf-8"))


def _on(doc: dict) -> dict:
    # PyYAML reads the bare key `on` as the boolean True (YAML 1.1).
    return doc.get("on") or doc.get(True) or {}


def _steps(doc: dict) -> list[dict]:
    return [s for job in doc["jobs"].values() for s in job["steps"]]


def test_the_nightly_profile_runs_on_a_schedule_headless_and_keeps_its_results():
    p = WF / "nightly.yml"
    assert p.is_file(), f"{p.relative_to(ROOT)} does not exist"
    doc = _yaml(p)
    sched = _on(doc).get("schedule") or []
    assert sched and all(re.fullmatch(r"[\d*/,\- ]+", s["cron"]) for s in sched), sched
    assert "workflow_dispatch" in _on(doc)
    runs = [s.get("run", "") for s in _steps(doc)]
    sweep = [r for r in runs if "sweep_scenarios.py" in r]
    assert len(sweep) == 1, runs
    assert "--profile nightly" in sweep[0] and "--backend sitl" not in sweep[0], sweep[0]
    assert "--out nightly/" in sweep[0] and "--runs nightly/" in sweep[0], sweep[0]
    assert "--no-bundles" not in sweep[0], "the bundles are what makes an episode re-runnable"
    assert any("-r requirements.txt" in r for r in runs), "the pinned requirements"
    up = [s for s in _steps(doc) if "upload-artifact" in str(s.get("uses", ""))]
    assert up and up[0].get("if") == "always()" and up[0]["with"]["path"] == "nightly/", up
    for job in doc["jobs"].values():
        assert int(job["timeout-minutes"]) <= 360, "GitHub-hosted jobs stop at 6 h"


def test_the_nightly_job_runs_the_profile_the_repository_defines():
    """The profile file it names must exist and keep the grant's scope."""
    sys.path.insert(0, str(ROOT))
    from guardrail.scenario_spec import Library, expand, load_profile
    prof = load_profile(ROOT / "experiments" / "profiles" / "nightly.yaml")
    _, info = expand(Library(ROOT / prof.library), prof)
    assert info["cells_within_expected"], (info["cells"], info["expected_cells"])
    assert prof.seeds == [1001, 1002, 1003]


def test_the_per_build_header_says_what_is_true():
    """A DOC CHECK, not a regression guard: it reads the workflow headers, so
    it can say only that they no longer claim what is false. tests.yml must
    not say it never ran (it ran red from 8 Oct, green at 4418ae0), and
    neither file may say the nightly job runs on GitHub: scheduled workflows
    fire only from the default branch, `main` on GitHub, while these files
    are on `master` (review of 2026-10-12, checked with gh)."""
    text = (WF / "tests.yml").read_text(encoding="utf-8")
    head = text.split("\nname:", 1)[0]
    assert "NOT YET RUN ON GITHUB" not in head and "nothing has been pushed" not in head, \
        "the header still says the workflow never ran"
    assert "4418ae0" in head, "the first green commit is named"
    assert "nightly.yml" in head, "the header says where the nightly profile is defined"
    nightly = (WF / "nightly.yml").read_text(encoding="utf-8").split("\nname:", 1)[0]
    prof = (ROOT / "experiments" / "profiles" / "nightly.yaml").read_text(encoding="utf-8")
    for name, doc in (("tests.yml", head), ("nightly.yml", nightly),
                      ("profiles/nightly.yaml", prof)):
        flat = " ".join(doc.replace("#", " ").split())
        assert "runs every night on GitHub" not in flat, f"{name} claims a nightly run"
        assert "default branch" in flat, f"{name} does not say why it has not run"


def test_every_harnesstruth_test_file_runs_in_ci_and_exists():
    text = (WF / "tests.yml").read_text(encoding="utf-8")
    loop = text.split("for t in", 1)[1].split("; do", 1)[0]
    listed = set(re.findall(r"tests/(test_[a-z0-9_]+\.py)", loop))
    for name in listed:
        assert (ROOT / "tests" / name).is_file(), f"tests.yml lists missing tests/{name}"
    mine = {p.name for p in (ROOT / "tests").glob("test_harnesstruth_*.py")}
    assert mine and mine <= listed, f"not run in CI: {sorted(mine - listed)}"
    assert "--profile smoke" in text, "the smoke step is still there"


# --------------------------------------------------------------------------- #
# The header's list against the loop (review of 2026-10-19)
#
# Shown failing first: at f93283d the header had no list (it named families
# and a few files), said "tests/test_env_pins.py is not listed" four lines
# after saying the loop ran it, and the loop held none of the files below.
# --------------------------------------------------------------------------- #
def _tests_yml() -> str:
    return (WF / "tests.yml").read_text(encoding="utf-8")


def _flat_header() -> str:
    head = _tests_yml().split("\nname:", 1)[0]
    return " ".join(head.replace("#", " ").split())


def _loop() -> set[str]:
    loop = _tests_yml().split("for t in", 1)[1].split("; do", 1)[0]
    return set(re.findall(r"tests/(test_[a-z0-9_]+\.py)", loop))


def _block(flat: str, start: str, end: str) -> str:
    assert start in flat and end in flat, f"the header has no {start!r} ... {end!r} block"
    return flat.split(start, 1)[1].split(end, 1)[0]


def _names(block: str) -> set[str]:
    """Test file names in a header block, bare or with .py; `test_x_*` stands
    for every tests/test_x_*.py, and must stand for at least one."""
    out: set[str] = set()
    for tok in re.findall(r"\b(test_[a-z0-9_]+\*?)(?:\.py)?", block):
        if tok.endswith("*"):
            fam = {p.name for p in (ROOT / "tests").glob(tok + ".py")}
            assert fam, f"the header names {tok} and no tests/{tok}.py exists"
            out |= fam
        else:
            out.add(tok + ".py")
    return out


def test_the_header_lists_exactly_the_files_the_loop_runs():
    """A reader deciding what CI covers reads the header, not the bash loop.
    `test_x_*` in the header must mean every such file, so a family member
    left off the loop shows up here too."""
    said = _names(_block(_flat_header(), "THE LOOP RUNS", "END OF THE LIST"))
    ran = _loop()
    assert said == ran, (f"the header names but the loop does not run: "
                         f"{sorted(said - ran)}; the loop runs but the header "
                         f"does not name: {sorted(ran - said)}")


def test_a_file_the_header_calls_unlisted_is_not_in_the_loop():
    """The flattened header (the env_pins sentence broke its line between
    "is not" and "listed"), matched case-blind ("is NOT listed"), compared by
    exact name (a substring test would call test_kpi_report.py listed because
    of test_wp4outputs_kpi_report.py)."""
    flat = _flat_header()
    excluded = _names(_block(flat, "NOT LISTED, on purpose:", "END OF THE EXCLUSIONS"))
    excluded |= {m + ".py" if not m.endswith(".py") else m for m in re.findall(
        r"(test_[a-z0-9_]+(?:\.py)?) is not listed", flat, flags=re.I)}
    assert excluded, "the header names no excluded file: did the wording move?"
    both = excluded & _loop()
    assert not both, f"the header says these are not listed, but the loop runs them: {sorted(both)}"


def test_the_files_the_october_review_asked_for_run_in_ci():
    """Each was run first in a clean `git archive` copy on Linux (WSL, one
    venv per job) and on Windows; see the header. test_fsm drives every FSM
    edge (guardrail/fsm.py says so), test_pas_ardupilot is the ArduPilot
    perception rail's own file (type_mask, frames, takeover), test_sitl_rails
    and test_mavlink_adapter hold the pymavlink rail's pacing and the mode
    routing, test_public_docs runs the README's code sample."""
    want = {"test_fsm.py", "test_policy_hash.py", "test_csp.py",
            "test_policy_dsl_grant_form.py", "test_replay_identity.py",
            "test_vla_backends.py", "test_prefix_eval.py", "test_fenceguard.py",
            "test_follow_motion.py", "test_identity.py", "test_identity_lock.py",
            "test_pas_ardupilot.py", "test_sitl_rails.py", "test_mavlink_adapter.py",
            "test_deploy_configs.py", "test_public_docs.py", "test_ft_report.py"}
    missing = want - _loop()
    assert not missing, f"not run in CI: {sorted(missing)}"


def test_the_rest_listener_tests_cannot_skip_on_the_job_that_installs_their_imports():
    """K4: the hot-apply-over-REST and mTLS allow-list tests import fastapi,
    uvicorn and httpx (and cryptography for the PKI). Until 2026-10-19 no job
    installed uvicorn, so all three SKIPped on every job and the loop, which
    reads only exit codes, stayed green. Now one job installs all four and
    lists both files in `no_skip`, and the loop fails on their SKIP lines."""
    doc = _yaml(WF / "tests.yml")
    jobs = doc["jobs"]["guardrail"]["strategy"]["matrix"]["include"]
    need = ("fastapi", "uvicorn", "httpx", "cryptography")
    full = [j for j in jobs
            if all(re.search(rf"(^|\s){n}(==|\s|$)", j.get("extras", "")) for n in need)]
    assert full, f"no job installs {need} together: {[j.get('extras') for j in jobs]}"
    files = {"tests/test_railinputs_node.py", "tests/test_railinputs_hotapply.py"}
    for j in full:
        assert files <= set(str(j.get("no_skip", "")).split()), j
    run = next(s["run"] for s in _steps(doc) if s.get("name") == "Standalone test files")
    assert "matrix.no_skip" in run and "grep -q '^SKIP'" in run, \
        "the loop does not read the SKIP lines of the no_skip files"
    assert "PIPESTATUS[0]" in run, "the loop must fail on python's exit code, not tee's"
    for f in files:
        src = (ROOT / f).read_text(encoding="utf-8")
        assert "import uvicorn" in src, f"{f} no longer imports uvicorn: re-check `need`"


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
