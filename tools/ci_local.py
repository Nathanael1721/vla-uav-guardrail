"""Run the CI pipeline on this machine, per build, and keep one record per commit.

    python tools/ci_local.py                      # tests loop + claim scan + smoke
    python tools/ci_local.py --no-smoke           # tests loop + claim scan only
    python tools/ci_local.py --only test_ir test_bundle
    python tools/ci_local.py --python C:/Users/natha/.conda/envs/vla-real/python.exe

WHY THIS EXISTS

The grant's Stress Testing page asks for a "CI smoke pipeline - GitHub Actions
or similar" whose smoke profile runs "per build (CI)" (WP4-06, WP4-14,
WP3-19). `.github/workflows/tests.yml` is that pipeline, but GitHub only runs
it on a push, and nothing has been pushed since 14 Oct (1b95383): every build
since was tested when somebody ran a hand-picked list. Twice that list missed
a regression the full loop caught (29 Oct), and once a stopped runner kept
writing into the same results file as the next run, which read as a false
133/133 (2 Nov).

This runs the same three gates as the workflow, from the same list:

  1. every file in the workflow's "Standalone test files" loop, in the
     loop's order, one at a time, each as `python tests/<file>` (the repo's
     tests are their own runners and print "N/M passed");
  2. `tools/check_claims.py`, the withdrawn-claim scan;
  3. the smoke profile, `experiments/sweep_scenarios.py --profile smoke`.

The list is read from the workflow, never copied, so the two cannot drift.

WHAT IT WRITES

`ci-local/<revision>/` (gitignored): one log per test file, the smoke sweep
and its episode bundles. `docs/data/ci_local/<revision>.json` (committed):
the summary for that build - revision, clean or dirty, interpreter, per-file
pass/fail/skip counts, the claim scan's exit code and the smoke profile's
fail-safe and P0 figures. A dirty tree is recorded as `-dirty`: that record
does not describe any commit and says so.

One runner at a time: a lock file in ci-local/ refuses a second run while the
first is alive, and a lock whose process is gone is taken over and reported.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = ROOT / ".github" / "workflows" / "tests.yml"
OUT_ROOT = ROOT / "ci-local"
SUMMARY_DIR = ROOT / "docs" / "data" / "ci_local"
LOCK_NAME = "ci_local.lock"

PASSED_RE = re.compile(r"^\s*(\d+)/(\d+) passed", re.M)
SKIP_RE = re.compile(r"^SKIP", re.M)


def loop_files(workflow_text: str) -> list[str]:
    """The test files of the workflow's standalone loop, in the loop's order."""
    if "for t in" not in workflow_text or "; do" not in workflow_text:
        raise ValueError("the workflow has no `for t in ...; do` loop")
    loop = workflow_text.split("for t in", 1)[1].split("; do", 1)[0]
    names = re.findall(r"tests/(test_[a-z0-9_]+\.py)", loop)
    if not names:
        raise ValueError("the workflow's loop names no test file")
    seen: set[str] = set()
    out = []
    for n in names:
        if n not in seen:
            seen.add(n)
            out.append(n)
    return out


def parse_result(output: str, rc: int) -> dict:
    """Read one test file's output: its own "N/M passed" line, SKIP lines, exit code.

    A file that exits 0 but never prints its count is reported as such
    (`counted` False), not as a pass of unknown size.
    """
    m = PASSED_RE.findall(output)
    passed, total = (int(m[-1][0]), int(m[-1][1])) if m else (None, None)
    return {"rc": rc, "passed": passed, "total": total, "counted": bool(m),
            "skips": len(SKIP_RE.findall(output)),
            "ok": rc == 0}


def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    if os.name == "nt":
        r = subprocess.run(["tasklist", "/FI", f"PID eq {pid}", "/NH"],
                           capture_output=True, text=True)
        return str(pid) in r.stdout
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def acquire_lock(lock: Path, alive=_pid_alive) -> str | None:
    """Take the lock, or raise if a live runner holds it.

    Returns a note when a stale lock (its process gone) was taken over.
    """
    lock.parent.mkdir(parents=True, exist_ok=True)
    note = None
    if lock.exists():
        try:
            held = json.loads(lock.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            held = {}
        pid = int(held.get("pid", -1))
        if alive(pid):
            raise RuntimeError(f"another ci_local run holds {lock} (pid {pid}, "
                               f"started {held.get('started')}); one runner at a time")
        note = f"took over a stale lock from pid {pid} (started {held.get('started')})"
    lock.write_text(json.dumps({"pid": os.getpid(),
                                "started": time.strftime("%Y-%m-%dT%H:%M:%S")}),
                    encoding="utf-8")
    return note


def release_lock(lock: Path) -> None:
    try:
        held = json.loads(lock.read_text(encoding="utf-8"))
        if int(held.get("pid", -1)) == os.getpid():
            lock.unlink()
    except (OSError, ValueError):
        pass


def revision(root: Path = ROOT) -> str:
    sys.path.insert(0, str(root))
    from guardrail.manifest import code_revision
    return code_revision(root)


def _run(cmd: list[str], log: Path, timeout_s: float) -> tuple[int, str]:
    log.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
    try:
        r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True,
                           encoding="utf-8", errors="replace", env=env,
                           timeout=timeout_s)
        rc, out = r.returncode, (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired as e:
        rc = 124
        out = ((e.stdout or b"").decode("utf-8", "replace") if isinstance(e.stdout, bytes)
               else (e.stdout or "")) + f"\nTIMEOUT after {timeout_s} s"
    log.write_text(out, encoding="utf-8")
    return rc, out


def smoke_summary(sweep: dict) -> dict:
    """The smoke profile's headline figures, as the sweep wrote them."""
    keys = ("failsafe_trigger_correctness", "null_never_trigger", "null_always_trigger",
            "p0_violation_escape_rate", "episodes")
    found: dict = {}

    def walk(d, depth=0):
        if depth > 4 or not isinstance(d, dict):
            return
        for k, v in d.items():
            if k in keys and k not in found and not isinstance(v, (dict, list)):
                found[k] = v
            elif isinstance(v, dict):
                walk(v, depth + 1)
    walk(sweep)
    return found


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--python", default=sys.executable,
                    help="interpreter for every gate (default: this one)")
    ap.add_argument("--only", nargs="+", default=None,
                    help="run only these test files (names, with or without .py)")
    ap.add_argument("--no-smoke", action="store_true")
    ap.add_argument("--no-summary", action="store_true",
                    help="do not write docs/data/ci_local/<revision>.json")
    ap.add_argument("--timeout", type=float, default=1800.0,
                    help="per-gate timeout in seconds")
    a = ap.parse_args(argv)

    files = loop_files(WORKFLOW.read_text(encoding="utf-8"))
    if a.only:
        want = {n if n.endswith(".py") else n + ".py" for n in a.only}
        unknown = want - set(files)
        if unknown:
            print(f"not in the workflow's loop: {sorted(unknown)}", file=sys.stderr)
            return 2
        files = [f for f in files if f in want]

    rev = revision()
    out_dir = OUT_ROOT / rev
    lock = OUT_ROOT / LOCK_NAME
    try:
        note = acquire_lock(lock)
    except RuntimeError as e:
        print(e, file=sys.stderr)
        return 3
    if note:
        print(note)
    started = time.time()
    try:
        results: dict[str, dict] = {}
        for i, f in enumerate(files, 1):
            rc, out = _run([a.python, f"tests/{f}"], out_dir / "tests" / f"{f}.log", a.timeout)
            res = parse_result(out, rc)
            results[f] = res
            tag = "ok  " if res["ok"] else "FAIL"
            count = f"{res['passed']}/{res['total']}" if res["counted"] else "no count"
            print(f"[{i}/{len(files)}] {tag} {f}  {count}  skips {res['skips']}", flush=True)
        rc_claims, _ = _run([a.python, "tools/check_claims.py"], out_dir / "check_claims.log",
                            a.timeout)
        print(f"check_claims rc {rc_claims}")
        smoke = None
        if not a.no_smoke:
            sweep_path = out_dir / "smoke" / "sweep.json"
            rc_smoke, _ = _run([a.python, "experiments/sweep_scenarios.py", "--profile", "smoke",
                                "--out", str(sweep_path), "--runs",
                                str(out_dir / "smoke" / "episodes")],
                               out_dir / "smoke.log", a.timeout)
            smoke = {"rc": rc_smoke}
            if sweep_path.is_file():
                smoke.update(smoke_summary(json.loads(sweep_path.read_text(encoding="utf-8"))))
            print(f"smoke rc {rc_smoke} {smoke}")
        failed = sorted(f for f, r in results.items() if not r["ok"])
        summary = {
            "revision": rev,
            "dirty": rev.endswith("-dirty"),
            "python": subprocess.run([a.python, "-c", "import sys;print(sys.version.split()[0])"],
                                     capture_output=True, text=True).stdout.strip(),
            "host": platform.node(),
            "started": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(started)),
            "duration_s": round(time.time() - started, 1),
            "subset": bool(a.only),
            "files": len(results),
            "files_failed": failed,
            "tests_passed": sum(r["passed"] or 0 for r in results.values()),
            "tests_total": sum(r["total"] or 0 for r in results.values()),
            "skip_lines": sum(r["skips"] for r in results.values()),
            "uncounted": sorted(f for f, r in results.items() if not r["counted"]),
            "check_claims_rc": rc_claims,
            "smoke": smoke,
            "per_file": results,
        }
        if not a.no_summary and not a.only:
            SUMMARY_DIR.mkdir(parents=True, exist_ok=True)
            with open(SUMMARY_DIR / f"{rev}.json", "w", encoding="utf-8", newline="\n") as fh:
                json.dump(summary, fh, indent=1, sort_keys=True)
                fh.write("\n")
        print(f"\n{len(results) - len(failed)}/{len(results)} files ok, "
              f"{summary['tests_passed']}/{summary['tests_total']} tests passed, "
              f"claims rc {rc_claims}" + (f", smoke rc {smoke['rc']}" if smoke else ""))
        if failed:
            print("failed: " + ", ".join(failed))
        bad = bool(failed) or rc_claims != 0 or (smoke is not None and smoke["rc"] != 0)
        return 1 if bad else 0
    finally:
        release_lock(lock)


if __name__ == "__main__":
    sys.exit(main())
