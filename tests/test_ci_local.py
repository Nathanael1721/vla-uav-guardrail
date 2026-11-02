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
