"""tools/ci_local.py: the workflow's gates, run on this machine, one runner at a time.

Run either way:
    pytest tests/test_ci_local.py -v
    python tests/test_ci_local.py

What these guard against: a local list that drifts from the workflow's loop
(the hand-picked subsets that missed two regressions on 29 Oct), a test file
that exits 0 without printing its count being read as a pass, and two runners
writing the same results at once (the false 133/133 of 2 Nov).
"""
import json
import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import ci_local as CI                              # noqa: E402

YML = """
      - name: Standalone test files
        run: |
          for t in tests/test_b.py \\
              tests/test_a.py \\
              tests/test_b.py; do
            python "$t"
          done
"""


def test_the_list_is_the_workflows_loop_in_its_order():
    assert CI.loop_files(YML) == ["test_b.py", "test_a.py"]


def test_the_real_workflow_loop_is_read_and_matches_the_ci_parity_test():
    text = CI.WORKFLOW.read_text(encoding="utf-8")
    files = CI.loop_files(text)
    import re
    loop = text.split("for t in", 1)[1].split("; do", 1)[0]
    assert set(files) == set(re.findall(r"tests/(test_[a-z0-9_]+\.py)", loop))
    assert "test_ci_local.py" in files, "this file must run in CI too"
    assert all((ROOT / "tests" / f).is_file() for f in files)


def test_a_stray_word_in_a_loop_is_refused_as_bash_would_run_it():
    # tests.yml:804 had ` \n ` twice inside the loop from 70ab5e8 to 17 Nov:
    # bash reads that `\n` as the word `n` and GitHub would run `python n`,
    # while a regex over the loop saw only the test files.
    stray = YML.replace("tests/test_a.py \\\n", "tests/test_a.py \\n ")
    assert "\\n " in stray and stray != YML
    try:
        CI.loop_files(stray)
    except ValueError as e:
        assert "'n'" in str(e), e
    else:
        raise AssertionError("a loop with a literal \\n was read as test files only")
    pil = WF.replace("tests/test_q.py tests/test_p.py", "tests/test_q.py \\n tests/test_p.py")
    try:
        CI.pillow_step(pil)
    except ValueError as e:
        assert "'n'" in str(e), e
    else:
        raise AssertionError("the Pillow loop with a literal \\n was accepted")


def test_a_workflow_without_the_loop_is_refused():
    for bad in ("steps: []", "for t in nothing; do", "for t in tests/README.md; do"):
        try:
            CI.loop_files(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad!r}")


def test_a_count_line_is_read_and_a_missing_one_is_not_a_pass_of_unknown_size():
    r = CI.parse_result("PASS  x\nSKIP y (no torch)\n\n5/6 passed\n", 1)
    assert (r["passed"], r["total"], r["skips"], r["ok"], r["counted"]) == (5, 6, 1, False, True)
    r = CI.parse_result("nothing printed\n", 0)
    assert r["ok"] and not r["counted"] and r["passed"] is None


def test_the_last_count_line_wins():
    r = CI.parse_result("1/2 passed\n...\n7/7 passed\n", 0)
    assert (r["passed"], r["total"]) == (7, 7)


def test_a_live_lock_refuses_a_second_runner():
    with tempfile.TemporaryDirectory() as d:
        lock = Path(d) / "ci.lock"
        lock.write_text(json.dumps({"pid": 4242, "started": "x"}), encoding="utf-8")
        try:
            CI.acquire_lock(lock, alive=lambda pid: pid == 4242)
        except RuntimeError as e:
            assert "4242" in str(e)
        else:
            raise AssertionError("a live lock was taken over")
        assert json.loads(lock.read_text(encoding="utf-8"))["pid"] == 4242


def test_a_stale_lock_is_taken_over_and_said_so():
    with tempfile.TemporaryDirectory() as d:
        lock = Path(d) / "ci.lock"
        lock.write_text(json.dumps({"pid": 4242, "started": "x"}), encoding="utf-8")
        note = CI.acquire_lock(lock, alive=lambda pid: False)
        assert note and "4242" in note
        assert json.loads(lock.read_text(encoding="utf-8"))["pid"] == os.getpid()
        CI.release_lock(lock)
        assert not lock.exists()


def test_release_leaves_another_runners_lock_alone():
    with tempfile.TemporaryDirectory() as d:
        lock = Path(d) / "ci.lock"
        lock.write_text(json.dumps({"pid": os.getpid() + 1}), encoding="utf-8")
        CI.release_lock(lock)
        assert lock.exists()


def test_the_smoke_summary_reads_the_sweeps_figures_without_inventing_any():
    sweep = {"profile": "smoke", "kpi": {"failsafe_trigger_correctness": 0.9,
                                         "null_never_trigger": 0.93, "episodes": 56}}
    s = CI.smoke_summary(sweep)
    assert s == {"failsafe_trigger_correctness": 0.9, "null_never_trigger": 0.93,
                 "episodes": 56}
    assert CI.smoke_summary({}) == {}


# --------------------------------------------------------------------------- #
# The Pillow step and the NOT LISTED files (17 Nov)
#
# Until 17 Nov this ran the loop only. The 3.10 job's Pillow step and the
# header's NOT LISTED files never ran here, although the desktop has OpenCV,
# Pillow and node, and two of those files failed unseen: test_policy_hud
# pinned the NFZ policy's version at 0.1.2 after 4ae620a moved it to 0.1.3,
# and test_two_view asserted an import line 63f9072 had widened.
# --------------------------------------------------------------------------- #
WF = """# tests.yml header
#      NOT LISTED, on purpose:
#        test_x.py: 2 of its 5 tests ERROR without OpenCV, as test_a.py's
#          capture does (a name inside a reason is not an entry).
#        test_p.py: 1 of its 3 tests ERROR without Pillow.
#      END OF THE EXCLUSIONS
#
name: tests
jobs:
  guardrail:
    steps:
      - name: Standalone test files
        run: |
          for t in tests/test_a.py \\
              tests/test_q.py; do
            python "$t"
          done
      - name: Something else with images
        run: |
          for img in tests/test_wrong.py; do echo "$img"; done
      - name: AerialVLA image tests (Pillow)
        if: matrix.python == '3.10'
        env:
          PYTHONUNBUFFERED: "1"
          GUARDRAIL_REQUIRE_PILLOW: "1"
        run: |
          for img in tests/test_q.py tests/test_p.py tests/test_q.py; do
            PYTHONPATH="$RUNNER_TEMP/pillow" python "$img"
          done
      - name: Withdrawn-claim scan
        env:
          NOT_THIS: "1"
        run: |
          for img in tests/test_not_this.py; do echo; done
"""


def test_the_pillow_step_is_read_with_its_own_loop_and_env():
    files, env = CI.pillow_step(WF)
    assert files == ["test_q.py", "test_p.py"]
    assert env == {"PYTHONUNBUFFERED": "1", "GUARDRAIL_REQUIRE_PILLOW": "1"}


def test_a_workflow_without_the_pillow_step_or_its_loop_is_refused():
    no_step = WF.replace("AerialVLA image tests (Pillow)", "Renamed")
    no_loop = WF.replace("for img in tests/test_q.py", "for x in tests/test_q.py")
    for bad in (no_step, no_loop):
        try:
            CI.pillow_step(bad)
        except ValueError:
            continue
        raise AssertionError("accepted a workflow without the Pillow step's loop")


def test_the_not_listed_entries_are_read_and_a_name_in_a_reason_is_not_one():
    assert CI.excluded_files(WF) == ["test_x.py", "test_p.py"]
    try:
        CI.excluded_files(WF.replace("END OF THE EXCLUSIONS", "the end"))
    except ValueError:
        pass
    else:
        raise AssertionError("accepted a NOT LISTED block without its end marker")


def test_each_file_runs_once_in_its_first_lists_slot_with_every_lists_env():
    p = CI.plan(WF)
    assert [e["file"] for e in p] == ["test_a.py", "test_q.py", "test_p.py", "test_x.py"]
    by = {e["file"]: e for e in p}
    assert by["test_q.py"]["group"] == "loop" and by["test_q.py"]["lists"] == ["loop", "pillow"]
    assert by["test_q.py"]["env"]["GUARDRAIL_REQUIRE_PILLOW"] == "1"
    assert by["test_p.py"]["group"] == "pillow" and by["test_p.py"]["lists"] == ["pillow", "extras"]
    assert by["test_p.py"]["env"]["GUARDRAIL_REQUIRE_PILLOW"] == "1"
    assert by["test_a.py"]["env"] == {} and by["test_x.py"]["env"] == {}
    assert by["test_x.py"]["group"] == "extras"
    assert [e["file"] for e in CI.plan(WF, extras=False)] == ["test_a.py", "test_q.py", "test_p.py"]


def test_an_extra_may_miss_only_the_package_its_entry_names():
    p = {e["file"]: e for e in CI.plan(WF)}
    assert p["test_x.py"]["may_miss"] == ["cv2"]            # "without OpenCV"
    # test_p.py's entry says Pillow, but it is in the Pillow step: it gates.
    assert p["test_p.py"]["may_miss"] == [] and p["test_a.py"]["may_miss"] == []
    assert CI.may_miss("6 of its 9 tests ERROR without OpenCV and Pillow") == ["PIL", "cv2"]
    assert CI.may_miss("7 of its 13 tests need flight logs; node is absent") == []


ERR = "ERROR {0}\n      ModuleNotFoundError: No module named '{1}'\n"


def test_only_a_run_whose_every_failure_is_an_allowed_module_missing_does_not_gate():
    ok = "PASS  test_a\n" + ERR.format("test_draw", "cv2") + ERR.format("test_d2", "cv2.ximgproc")
    assert CI.not_gating_modules(ok + "\n1/3 passed\n", 1, True, ["cv2"]) == ["cv2"]
    two = ERR.format("t1", "cv2") + ERR.format("t2", "PIL") + "\n0/2 passed\n"
    assert CI.not_gating_modules(two, 1, True, ["PIL", "cv2"]) == ["PIL", "cv2"]
    # A genuine FAIL beside the missing module: it gates (review of 17 Nov).
    mixed = (ERR.format("test_cv", "cv2") + "FAIL  test_orbit\n      12.0 != 10.0\n"
             "\n1/3 passed\n")
    assert CI.not_gating_modules(mixed, 1, True, ["cv2"]) == []
    # A SKIP line naming a module is not a failure block and excuses nothing.
    skip = ("SKIP  test_overlay: No module named 'PIL'\n"
            "FAIL  test_lock\n      AssertionError\n\n1/3 passed\n")
    assert CI.not_gating_modules(skip, 1, True, ["PIL"]) == []
    # A module the entry does not name - a deleted or renamed demo/ module
    # looks just like a third-party one - gates.
    for gone in ("two_view", "two_view_gone", "guardrail.gone", "PIL"):
        assert CI.not_gating_modules(ERR.format("t", gone) + "\n0/1 passed\n", 1, True,
                                     ["cv2"]) == [], gone
    # Another error type, a failure without a block, a timeout, a crash, a pass.
    assert CI.not_gating_modules("ERROR t\n      ImportError: DLL load failed\n\n0/1 passed\n",
                                 1, True, ["cv2"]) == []
    assert CI.not_gating_modules("\n2/2 passed\n", 1, True, ["cv2"]) == []
    assert CI.not_gating_modules(ok + "\nTIMEOUT after 9 s", 124, True, ["cv2"]) == []
    assert CI.not_gating_modules(ok + "\n1/3 passed\n", -11, True, ["cv2"]) == []
    assert CI.not_gating_modules(ok + "\n1/3 passed\n", 1, True, []) == []


def test_a_file_dying_at_import_does_not_gate_only_for_an_allowed_module():
    tb = ("Traceback (most recent call last):\n  File \"tests/test_x.py\", line 9\n"
          "    import cv2\nModuleNotFoundError: No module named '{0}'\n")
    assert CI.not_gating_modules(tb.format("cv2"), 1, False, ["cv2"]) == ["cv2"]
    assert CI.not_gating_modules(tb.format("two_view"), 1, False, ["cv2"]) == []
    assert CI.not_gating_modules(tb.format("cv2") + "AssertionError\n", 1, False,
                                 ["cv2"]) == []


def _res(ok, deps=()):
    return {"ok": ok, "rc": 0 if ok else 1, "missing_deps": list(deps)}


def test_a_red_extra_gates_and_one_missing_a_dependency_here_does_not():
    p = CI.plan(WF)
    results = {"test_a.py": _res(True), "test_q.py": _res(True), "test_p.py": _res(True),
               "test_x.py": _res(False)}
    t = CI.tally(p, results)
    assert (t["files_ok"], t["files"], t["failed"]) == (3, 4, ["test_x.py"])
    assert t["groups"]["extras"]["failed"] == ["test_x.py"]
    results["test_x.py"] = _res(False, ["cv2"])
    t = CI.tally(p, results)
    assert (t["files_ok"], t["files"], t["failed"]) == (3, 3, [])
    assert t["not_gating"] == {"test_x.py": ["cv2"]}
    # The same missing module in a loop or Pillow file gates: CI runs those.
    results["test_p.py"] = _res(False, ["PIL"])
    t = CI.tally(p, results)
    assert t["failed"] == ["test_p.py"] and t["groups"]["pillow"]["failed"] == ["test_p.py"]


def test_the_summary_line_keeps_what_the_ci_chain_reads_and_adds_the_groups():
    import re
    p = CI.plan(WF)
    results = {"test_a.py": _res(True), "test_q.py": _res(True), "test_p.py": _res(True),
               "test_x.py": _res(False)}
    line = CI.summary_line(CI.tally(p, results), 10, 11, 0, {"rc": 0})
    assert line == ("3/4 files ok (loop 2/2, pillow 1/1, extras 0/1), 10/11 tests passed, "
                    "claims rc 0, smoke rc 0"), line
    m = re.match(r"^([0-9]+)/([0-9]+) files ok", line)
    assert m and m.group(1) != m.group(2), "a red extra must make it N-1/N"
    results["test_x.py"] = _res(False, ["cv2"])
    line = CI.summary_line(CI.tally(p, results), 10, 11, 0, None)
    assert line.startswith("3/3 files ok (loop 2/2, pillow 1/1, extras 0/0, "
                           "1 not gating: test_x.py missing cv2), "), line
    assert "smoke rc" not in line


def test_the_real_workflows_lists_agree_with_the_ci_parity_test():
    sys.path.insert(0, str(ROOT / "tests"))
    import test_harnesstruth_ci as H
    text = CI.WORKFLOW.read_text(encoding="utf-8")
    extras = CI.excluded_files(text)
    assert set(extras) == set(H.EXCLUDED), (sorted(extras), sorted(H.EXCLUDED))
    pil, env = CI.pillow_step(text)
    assert env.get("GUARDRAIL_REQUIRE_PILLOW") == "1", env
    assert {"test_aerialvla_codec.py", "test_vla_server.py"} <= set(pil), pil
    p = CI.plan(text)
    names = [e["file"] for e in p]
    assert len(names) == len(set(names)), "a file is planned twice"
    assert set(names) == set(CI.loop_files(text)) | set(pil) | set(extras)
    assert all((ROOT / "tests" / f).is_file() for f in names)
    assert all(e["group"] == "pillow" for e in p if e["file"] in pil
               and e["file"] not in CI.loop_files(text))
    # Each extra may miss exactly the package EXCLUDED names for it, as a module.
    assert {pkg for pkg, _ in H.EXCLUDED.values() if pkg} <= set(CI.DEP_MODULES)
    for e in p:
        if e["group"] == "extras":
            pkg = H.EXCLUDED[e["file"]][0]
            assert (CI.DEP_MODULES[pkg] in e["may_miss"]) if pkg else not e["may_miss"], e
        else:
            assert e["may_miss"] == [], e


class _Fake:
    """Stands in for ci_local._run: records each call, answers from a table."""

    def __init__(self, outputs):
        self.outputs, self.calls = outputs, []

    def __call__(self, cmd, log, timeout_s, extra_env=None):
        self.calls.append((cmd[1], dict(extra_env or {})))
        name = cmd[1].rsplit("/", 1)[-1]
        return self.outputs.get(name, (0, ""))


def _main(outputs, extra_args=()):
    """Run ci_local.main on three real-workflow files with _run faked: one
    loop+Pillow file, one Pillow file and one extra."""
    import contextlib
    import io
    fake = _Fake(outputs)
    saved = CI._run, CI.revision, CI.OUT_ROOT
    buf = io.StringIO()
    with tempfile.TemporaryDirectory() as d:
        CI._run, CI.revision, CI.OUT_ROOT = fake, (lambda: "rev-test"), Path(d)
        try:
            with contextlib.redirect_stdout(buf):
                rc = CI.main(["--only", "test_aerialvla_codec", "test_policy_hud",
                              "test_two_view", "--no-smoke", "--no-summary", *extra_args])
        finally:
            CI._run, CI.revision, CI.OUT_ROOT = saved
    line = next(ln for ln in buf.getvalue().splitlines() if " files ok " in ln)
    return rc, line, fake.calls


def test_main_runs_each_planned_file_once_with_its_env_and_gates_a_red_extra():
    # Review of 17 Nov: reverting main() to the loop only, or dropping the
    # Pillow env on the way to _run, left every helper test green.
    green = (0, "PASS  t\n\n1/1 passed\n")
    red = (1, "PASS  t\nFAIL  test_layout\n      AssertionError: 2 != 3\n\n9/10 passed\n")
    rc, line, calls = _main({"test_aerialvla_codec.py": green, "test_policy_hud.py": green,
                             "test_two_view.py": red})
    ran = [c for c, _ in calls if c.startswith("tests/")]
    assert sorted(ran) == ["tests/test_aerialvla_codec.py", "tests/test_policy_hud.py",
                           "tests/test_two_view.py"], ran
    env = dict(calls)
    assert env["tests/test_aerialvla_codec.py"].get("GUARDRAIL_REQUIRE_PILLOW") == "1"
    assert env["tests/test_policy_hud.py"].get("GUARDRAIL_REQUIRE_PILLOW") == "1"
    assert env["tests/test_two_view.py"] == {}
    assert "tools/check_claims.py" in env
    assert rc == 1 and line.startswith("2/3 files ok (loop 1/1, pillow 1/1, extras 0/1)"), line


def test_main_lets_an_extra_missing_only_its_named_module_pass_and_nothing_else():
    green = (0, "\n1/1 passed\n")
    cv2_only = (1, "PASS  t\nERROR test_draw\n      ModuleNotFoundError: No module named "
                   "'cv2'\n\n9/10 passed\n")
    rc, line, _ = _main({"test_aerialvla_codec.py": green, "test_policy_hud.py": green,
                         "test_two_view.py": cv2_only})
    assert rc == 0 and line.startswith(
        "2/2 files ok (loop 1/1, pillow 1/1, extras 0/0, 1 not gating: "
        "test_two_view.py missing cv2)"), (rc, line)
    mixed = (1, cv2_only[1].replace("PASS  t\n", "FAIL  test_pip\n      off by 4 px\n"))
    rc, line, _ = _main({"test_aerialvla_codec.py": green, "test_policy_hud.py": green,
                         "test_two_view.py": mixed})
    assert rc == 1 and line.startswith("2/3 files ok"), (rc, line)
    # The same missing module in a Pillow-step file gates.
    rc, line, _ = _main({"test_aerialvla_codec.py": green, "test_policy_hud.py": cv2_only,
                         "test_two_view.py": green})
    assert rc == 1 and line.startswith("2/3 files ok"), (rc, line)


def test_run_passes_the_extra_env_to_the_child():
    with tempfile.TemporaryDirectory() as d:
        rc, out = CI._run([sys.executable, "-c",
                           "import os;print(os.environ['CI_LOCAL_PROBE'])"],
                          Path(d) / "probe.log", 60, extra_env={"CI_LOCAL_PROBE": "y42"})
        assert rc == 0 and out.strip() == "y42", (rc, out)
        assert (Path(d) / "probe.log").read_text(encoding="utf-8").strip() == "y42"


def test_ci_local_output_is_gitignored():
    ignore = (ROOT / ".gitignore").read_text(encoding="utf-8")
    assert any(line.strip().rstrip("/") == "ci-local" for line in ignore.splitlines())


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
