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


# --------------------------------------------------------------------------- #
# Complete by construction (review of the 2026-10-19 merges: docs-ci P1, P4,
# P8; harness-tools P1)
#
# Shown failing first: at 2b3ad56 the header said "Every other test file stays
# off CI for the reasons under WHAT DOES NOT RUN HERE" (demo/out, simulators,
# WSL, ROS 2, SITL, GPU), while test_paraphraser (84/84, the WP2 Paraphraser's
# own file), test_geo_to_policy, test_vla_bridge and seven more ran clean on a
# clean copy, and the harness-tools review's regression tests (the claims
# scan's, the tracker server's CSRF guard, the deliverable encoder's, the
# update log's) were neither run nor excluded. Its NOT LISTED reasons had gone stale too:
# test_replay ran 40 of its 60 tests on a clean copy, not "2 of its 20".
# no_skip named the two railinputs files only, so test_harness_service's gRPC
# tests and test_api's routes could turn into SKIPs on the jobs that install
# their imports while the job stayed green.
# --------------------------------------------------------------------------- #

# Every tests/test_*.py the loop does not run: the pip package whose absence
# makes it fail on every job ("" when the cause is not a package), and why.
# The header's NOT LISTED block names exactly these files, with the same
# reasons. Measured 2026-10-19 on a clean `git archive` copy under WSL Ubuntu,
# in one bare venv per job (requirements.txt plus that job's extras).
EXCLUDED: dict[str, tuple[str, str]] = {
    "test_capture_pairing.py": (
        "opencv", "5 of its 8 tests ERROR: demo/semantic_demo.py's capture path imports cv2"),
    "test_city_traffic.py": (
        "projectairsim", "its update-stagger test FAILs: the fake world's pose calls come only "
                         "after demo/moving_car.py imports projectairsim"),
    "test_deck_scorer_parity.py": (
        "", "7 of its 13 tests need flight logs under demo/out/, and its JavaScript "
            "comparisons return early, counted as passes, where node is absent (WSL has none)"),
    "test_policy_hud.py": ("pillow", "6 of its 45 tests ERROR without Pillow (the overlay drawing)"),
    "test_range_and_lock.py": ("opencv", "16 of its 112 tests ERROR without OpenCV"),
    "test_two_view.py": ("opencv", "6 of its 9 tests ERROR without OpenCV and Pillow"),
}


def _matrix_jobs() -> list[dict]:
    return _yaml(WF / "tests.yml")["jobs"]["guardrail"]["strategy"]["matrix"]["include"]


def _package(spec: str) -> str:
    """`'protobuf>=7.35.1'` -> `protobuf`; `numpy==1.26.4; python_version < "3.11"`
    -> `numpy`."""
    return re.split(r"[<>=!~;\[\s]", spec.strip().strip("'\""), maxsplit=1)[0].lower()


def _installed(job: dict) -> set[str]:
    """What a job's install step names: requirements.txt plus its extras."""
    req = [ln for ln in (ROOT / "requirements.txt").read_text(encoding="utf-8").splitlines()
           if ln.strip() and not ln.lstrip().startswith("#")]
    return {_package(s) for s in req + str(job.get("extras", "")).split()} - {""}


def test_every_test_file_is_in_the_loop_or_excluded_with_its_reason():
    """The header cannot drift again: a new tests/test_*.py must go into the
    loop or into NOT LISTED (and EXCLUDED) with a reason, and an exclusion
    whose package a job starts installing must be re-run there."""
    files = {p.name for p in (ROOT / "tests").glob("test_*.py")}
    ran = _loop()
    said = _names(_block(_flat_header(), "NOT LISTED, on purpose:", "END OF THE EXCLUSIONS"))
    assert said == set(EXCLUDED), (f"the header's NOT LISTED block and EXCLUDED differ: header "
                                   f"only {sorted(said - set(EXCLUDED))}, EXCLUDED only "
                                   f"{sorted(set(EXCLUDED) - said)}")
    assert set(EXCLUDED) <= files, f"excluded files that do not exist: {sorted(set(EXCLUDED) - files)}"
    both = ran & set(EXCLUDED)
    assert not both, f"excluded and in the loop: {sorted(both)}"
    left = files - ran - set(EXCLUDED)
    assert not left, (f"neither run in CI nor excluded with a reason: {sorted(left)}; run each "
                      f"on a clean copy in the three job configurations, then list or exclude it")
    jobs = _matrix_jobs()
    for name, (pkg, why) in EXCLUDED.items():
        assert len(why.split()) >= 6, f"{name}: say what fails and why, not {why!r}"
        if pkg:
            for j in jobs:
                assert not any(p.startswith(pkg) for p in _installed(j)), \
                    (f"the {j['python']} job installs {pkg} now: run {name} there and list it, "
                     f"or say why it still cannot run")


def test_the_harness_and_api_tests_cannot_skip_where_their_imports_are_installed():
    """docs-ci P4: the header said test_harness_service runs in full on the
    3.10 job and the 3.11 extras job, and that test_api's routes run on the
    extras job, and nothing enforced either; an import regression would SKIP
    them under a green job (the K4 failure mode). Measured with the loop's own
    step in WSL: test_harness_service 56/56 on both jobs, test_api 14/14."""
    jobs = _matrix_jobs()
    grpc = [j for j in jobs if {"grpcio", "protobuf", "grpcio-tools", "cryptography"} <= _installed(j)]
    api = [j for j in jobs if {"fastapi", "httpx", "uvicorn"} <= _installed(j)]
    assert sorted(j["python"] for j in grpc) == ["3.10", "3.11"], [j.get("extras") for j in jobs]
    assert api, "no job installs fastapi, httpx and uvicorn together"
    for j in grpc:
        assert "tests/test_harness_service.py" in str(j.get("no_skip", "")).split(), j
    for j in api:
        assert "tests/test_api.py" in str(j.get("no_skip", "")).split(), j
    for j in jobs:
        for f in str(j.get("no_skip", "")).split():
            assert Path(f).name in _loop(), f"no_skip names {f}, which the loop does not run"
    assert "grpc_tools" in (ROOT / "tests/test_harness_service.py").read_text(encoding="utf-8")
    assert "fastapi" in (ROOT / "tests/test_api.py").read_text(encoding="utf-8")


def test_the_job_name_does_not_carry_the_matrix_lists():
    """docs-ci P8: without a `name`, GitHub names each job after every matrix
    value, so adding a file to `no_skip` renamed the job and would break any
    required check keyed on its name. The name must say only the interpreter
    and whether extras are installed, and tell the three jobs apart."""
    job = _yaml(WF / "tests.yml")["jobs"]["guardrail"]
    name = str(job.get("name", ""))
    assert name.startswith("guardrail (") and "matrix.python" in name, name
    assert "no_skip" not in name and "matrix.extras }}" not in name, name
    shown = {(j["python"], j.get("extras", "") == "") for j in _matrix_jobs()}
    assert len(shown) == len(_matrix_jobs()), "two jobs would share a name"


def test_the_claims_scan_runs_as_its_own_step():
    """harness-tools P1: check_claims passes on a clean clone since
    2026-10-19 (272/272 files clean, 4 SKIP-LOCAL, exit 0 in all three WSL
    venvs), so CI runs the tool itself, whose exit code covers the missing
    corrected values the test file does not check over the whole scope."""
    runs = [str(s.get("run", "")).strip() for s in _steps(_yaml(WF / "tests.yml"))]
    assert "python tools/check_claims.py" in runs, runs


def test_the_files_the_second_october_review_found_off_ci_run_in_ci():
    """Named by the two reviews of the 2026-10-19 merges: the harness-tools
    regression tests, and the files the docs-ci review ran clean on a clean
    copy (plus test_replay and test_kpi_report, whose stated exclusions were
    stale)."""
    want = {"test_check_claims.py", "test_tracker_serve.py", "test_encode_deliverable.py",
            "test_add_update.py", "test_make_demo_video.py", "test_track_truth.py",
            "test_paraphraser.py", "test_geo_to_policy.py", "test_drive_signals_dsl.py",
            "test_vla_bridge.py", "test_ros2_package.py", "test_search.py",
            "test_target_state.py", "test_trail.py", "test_camera_model.py",
            "test_occ_bands.py", "test_real_vla_prompt.py", "test_replay.py",
            "test_kpi_report.py"}
    missing = want - _loop()
    assert not missing, f"not run in CI: {sorted(missing)}"


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
