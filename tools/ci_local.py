"""Run the CI pipeline on this machine, per build, and keep one record per commit.

    python tools/ci_local.py                      # tests + claim scan + smoke
    python tools/ci_local.py --no-smoke           # tests + claim scan only
    python tools/ci_local.py --no-extras          # without the NOT LISTED files
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

and, in step 1, two more groups of test files:

  - "pillow": the files of the 3.10 job's step "AerialVLA image tests
    (Pillow)" (its `for img in ...; do` loop), with that step's env
    (GUARDRAIL_REQUIRE_PILLOW=1, so a Pillow that does not import fails
    instead of SKIPping). They gate like the loop.
  - "extras": the header's "NOT LISTED, on purpose" files, which no job can
    run (OpenCV, Pillow, node, projectairsim) but this desktop can. Until
    17 Nov nothing ran them, and two of them failed unseen for weeks
    (test_policy_hud's pinned policy version, test_two_view's literal import
    line). An extra gates like the rest, with one exception: on a machine
    without a package its NOT LISTED entry names (OpenCV -> cv2, Pillow ->
    PIL, projectairsim), a file whose every FAIL/ERROR is an ERROR
    "ModuleNotFoundError: No module named '<that module>'" (or which dies at
    import with exactly that) is reported as not gating, with the module, and
    is left out of the "N/M files ok" count. Anything else - one FAIL, another
    module (a renamed demo/ module is not a dependency), a timeout, a crash -
    gates.

A file named in more than one list runs once, in the first list's slot (loop,
then pillow, then extras), with the env of every list it is in: a loop file
that is also in the Pillow step runs once, with GUARDRAIL_REQUIRE_PILLOW=1.
CI runs such a file twice, and its loop run (no Pillow on the path) is the
only one that checks the file's SKIP guards for a missing Pillow; this
desktop always has Pillow, so that run is not reproduced here. The plan says
which files were folded.

The lists are read from the workflow, never copied, so they cannot drift.
A `for ... in ...; do` list is split as bash splits it, and every word must
be a tests/test_*.py file: a stray word (a literal `\\n` in the list is the
word `n` to bash, which GitHub would run as `python n`) is refused.

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
import shlex
import shutil
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
TEST_RE = r"tests/(test_[a-z0-9_]+\.py)"
# The message line the runners print under an ERROR block (`Type: message`),
# or the last line of a traceback, for a module that is not installed.
MISSING_LINE_RE = re.compile(r"ModuleNotFoundError: No module named '([A-Za-z0-9_.]+)'")
BLOCK_RE = re.compile(r"(FAIL|ERROR)\s+\S")

PILLOW_STEP = "AerialVLA image tests (Pillow)"
EXTRAS_START = "NOT LISTED, on purpose:"
EXTRAS_END = "END OF THE EXCLUSIONS"
GROUPS = ("loop", "pillow", "extras")
# A package a NOT LISTED entry names -> the module whose absence it explains
# (the package names are the EXCLUDED dict's of tests/test_harnesstruth_ci.py).
# Only these may be missing for an extra not to gate.
DEP_MODULES = {"opencv": "cv2", "pillow": "PIL", "projectairsim": "projectairsim"}


def _unique(names: list[str]) -> list[str]:
    seen: set[str] = set()
    out = []
    for n in names:
        if n not in seen:
            seen.add(n)
            out.append(n)
    return out


def _loop_words(body: str, what: str) -> list[str]:
    """The test files of a `for x in <body>; do` list, split as bash splits
    it (a backslash-newline continues the line, a lone backslash escapes the
    next character). Every word must be a tests/test_*.py file: bash would run
    any other word as a test (a literal `\\n` is the word `n`)."""
    words = shlex.split(body.replace("\\\r\n", " ").replace("\\\n", " "), posix=True)
    names = []
    for w in words:
        m = re.fullmatch(TEST_RE, w)
        if not m:
            raise ValueError(f"{what}: {w!r} is not a tests/test_*.py file, "
                             f"and bash would run it as one")
        names.append(m.group(1))
    if not names:
        raise ValueError(f"{what} names no test file")
    return _unique(names)


def loop_files(workflow_text: str) -> list[str]:
    """The test files of the workflow's standalone loop, in the loop's order."""
    if "for t in" not in workflow_text or "; do" not in workflow_text:
        raise ValueError("the workflow has no `for t in ...; do` loop")
    loop = workflow_text.split("for t in", 1)[1].split("; do", 1)[0]
    return _loop_words(loop, "the workflow's loop")


def _step_text(workflow_text: str, name: str) -> str:
    """The body of the step `- name: <name>`, up to the next step at its indent."""
    m = re.search(rf"^([ \t]*)- name:[ \t]*{re.escape(name)}[ \t]*$", workflow_text, re.M)
    if not m:
        raise ValueError(f"the workflow has no step named {name!r}")
    rest = workflow_text[m.end():]
    nxt = re.search(rf"^{re.escape(m.group(1))}- ", rest, re.M)
    return rest[:nxt.start()] if nxt else rest


def _step_env(step: str) -> dict[str, str]:
    """The step's `env:` mapping (KEY: value lines indented under it)."""
    lines = step.splitlines()
    for i, line in enumerate(lines):
        if line.strip() == "env:":
            indent = len(line) - len(line.lstrip())
            env = {}
            for sub in lines[i + 1:]:
                if not sub.strip() or sub.lstrip().startswith("#"):
                    continue
                if len(sub) - len(sub.lstrip()) <= indent:
                    break
                k, sep, v = sub.strip().partition(":")
                if sep:
                    env[k.strip()] = v.strip().strip("'\"")
            return env
    return {}


def pillow_step(workflow_text: str, name: str = PILLOW_STEP) -> tuple[list[str], dict[str, str]]:
    """The test files of the Pillow step's `for img in ...; do` loop, in its
    order, and the env the step sets (GUARDRAIL_REQUIRE_PILLOW=1)."""
    step = _step_text(workflow_text, name)
    if "for img in" not in step or "; do" not in step:
        raise ValueError(f"the step {name!r} has no `for img in ...; do` loop")
    loop = step.split("for img in", 1)[1].split("; do", 1)[0]
    return _loop_words(loop, f"the step {name!r}'s loop"), _step_env(step)


def excluded_entries(workflow_text: str) -> dict[str, str]:
    """The header's "NOT LISTED, on purpose:" block, per file, in its order:
    file name -> the text of its entry (its reason).

    Read as tests/test_harnesstruth_ci.py reads it (comment marks dropped,
    whitespace folded, the header ends at the top-level `name:`): `test_x.py:`
    starts an entry, a name inside a reason does not."""
    head = workflow_text.split("\nname:", 1)[0]
    flat = " ".join(head.replace("#", " ").split())
    if EXTRAS_START not in flat or EXTRAS_END not in flat.split(EXTRAS_START, 1)[1]:
        raise ValueError(f"the header has no {EXTRAS_START!r} ... {EXTRAS_END!r} block")
    block = flat.split(EXTRAS_START, 1)[1].split(EXTRAS_END, 1)[0]
    parts = re.split(r"\b(test_[a-z0-9_]+\.py):", block)
    entries: dict[str, str] = {}
    for i in range(1, len(parts) - 1, 2):
        entries.setdefault(parts[i], parts[i + 1].strip())
    if not entries:
        raise ValueError("the NOT LISTED block names no test file")
    return entries


def excluded_files(workflow_text: str) -> list[str]:
    """The files of the header's "NOT LISTED, on purpose:" block, in its order."""
    return list(excluded_entries(workflow_text))


def may_miss(entry: str) -> list[str]:
    """The modules a NOT LISTED entry's reason says are missing on CI: the
    package it names (OpenCV, Pillow, projectairsim), as a module."""
    low = entry.lower()
    return sorted(mod for pkg, mod in DEP_MODULES.items() if pkg in low)


def plan(workflow_text: str, extras: bool = True) -> list[dict]:
    """Every test file to run, once each, in run order.

    Each entry: `file`; `group`, the first list it is in (loop, pillow,
    extras), which decides whether it gates; `lists`, every list it is in;
    `env`, the env of those lists (the Pillow step's for a Pillow file);
    `may_miss`, for an extras-group file only, the modules whose absence here
    does not gate it (from its NOT LISTED entry; [] for every other file)."""
    pil, pil_env = pillow_step(workflow_text)
    reasons = excluded_entries(workflow_text) if extras else {}
    lists = {"loop": loop_files(workflow_text), "pillow": pil, "extras": list(reasons)}
    out: dict[str, dict] = {}
    for g in GROUPS:
        for f in lists[g]:
            e = out.setdefault(f, {"file": f, "group": g, "lists": [], "env": {},
                                   "may_miss": may_miss(reasons[f]) if g == "extras" else []})
            e["lists"].append(g)
            if g == "pillow":
                e["env"].update(pil_env)
    return list(out.values())


def _failure_blocks(output: str) -> list[tuple[str, str]]:
    """The runners' failure blocks, `FAIL  name` / `ERROR name` at the start
    of a line, each with its first message line (stripped; "" when none)."""
    lines = output.splitlines()
    blocks = []
    for i, line in enumerate(lines):
        m = BLOCK_RE.match(line)
        if m:
            nxt = lines[i + 1] if i + 1 < len(lines) else ""
            blocks.append((m.group(1), nxt.strip() if nxt[:1].isspace() else ""))
    return blocks


def not_gating_modules(output: str, rc: int, counted: bool, allowed: list[str]) -> list[str]:
    """The missing modules, when this failed run failed ONLY because modules in
    `allowed` are not installed here; [] otherwise (the failure then gates).

    Counted (the runner printed "N/M passed"): every FAIL/ERROR block must be
    an ERROR whose message is "ModuleNotFoundError: No module named 'X'" with
    X (its top level) in `allowed`; one FAIL, one other error, a module not in
    `allowed` (a renamed repo module), or no block at all, and it gates.
    Not counted (the file died before its runner): the traceback's last line
    must be that error. Only exit code 1 qualifies: 0 is a pass, 124 a
    timeout, anything else a crash. SKIP lines are not read."""
    if rc != 1 or not allowed:
        return []

    def module(msg: str) -> str | None:
        m = MISSING_LINE_RE.fullmatch(msg)
        top = m.group(1).split(".")[0] if m else None
        return top if top in allowed else None

    if counted:
        blocks = _failure_blocks(output)
        mods = [module(msg) if kind == "ERROR" else None for kind, msg in blocks]
        return sorted(set(mods)) if mods and all(mods) else []
    last = next((ln.strip() for ln in reversed(output.splitlines()) if ln.strip()), "")
    top = module(last)
    return [top] if top else []


def tally(entries: list[dict], results: dict[str, dict]) -> dict:
    """The verdict per group. A failed extra that failed only for a module its
    NOT LISTED entry names (`missing_deps`, from not_gating_modules) does not
    gate: it is listed under `not_gating` and left out of the `files`/
    `files_ok` count; every other failure gates."""
    groups = {g: {"files": 0, "ok": 0, "failed": [], "not_gating": {}} for g in GROUPS}
    for e in entries:
        r = results.get(e["file"])
        if r is None:
            continue
        g = groups[e["group"]]
        if r["ok"]:
            g["files"] += 1
            g["ok"] += 1
        elif e["group"] == "extras" and r.get("missing_deps"):
            g["not_gating"][e["file"]] = list(r["missing_deps"])
        else:
            g["files"] += 1
            g["failed"].append(e["file"])
    return {"groups": groups,
            "files": sum(g["files"] for g in groups.values()),
            "files_ok": sum(g["ok"] for g in groups.values()),
            "failed": sorted(f for g in groups.values() for f in g["failed"]),
            "not_gating": {f: d for g in groups.values() for f, d in g["not_gating"].items()}}


def summary_line(t: dict, tests_passed: int, tests_total: int, rc_claims: int,
                 smoke: dict | None) -> str:
    """`N/M files ok (loop a/b, pillow c/d, extras e/f), ...`: the CI chain
    reads `^N/M files ok`, "claims rc 0" and "smoke rc 0" from this line."""
    gs = t["groups"]
    parts = [f"{g} {gs[g]['ok']}/{gs[g]['files']}" for g in GROUPS]
    ng = t["not_gating"]
    if ng:
        parts.append(f"{len(ng)} not gating: " + "; ".join(
            f"{f} missing {'+'.join(d)}" for f, d in sorted(ng.items())))
    return (f"{t['files_ok']}/{t['files']} files ok ({', '.join(parts)}), "
            f"{tests_passed}/{tests_total} tests passed, "
            f"claims rc {rc_claims}" + (f", smoke rc {smoke['rc']}" if smoke else ""))


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


def _run(cmd: list[str], log: Path, timeout_s: float,
         extra_env: dict[str, str] | None = None) -> tuple[int, str]:
    log.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
    env.update(extra_env or {})
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
    ap.add_argument("--no-extras", action="store_true",
                    help="do not run the header's NOT LISTED files (the desktop extras)")
    ap.add_argument("--no-summary", action="store_true",
                    help="do not write docs/data/ci_local/<revision>.json")
    ap.add_argument("--timeout", type=float, default=1800.0,
                    help="per-gate timeout in seconds")
    a = ap.parse_args(argv)

    entries = plan(WORKFLOW.read_text(encoding="utf-8"), extras=not a.no_extras)
    if a.only:
        want = {n if n.endswith(".py") else n + ".py" for n in a.only}
        unknown = want - {e["file"] for e in entries}
        if unknown:
            print(f"not in the workflow's loop, its Pillow step or its NOT LISTED block: "
                  f"{sorted(unknown)}", file=sys.stderr)
            return 2
        entries = [e for e in entries if e["file"] in want]
    sizes = {g: sum(e["group"] == g for e in entries) for g in GROUPS}
    print("plan: " + ", ".join(f"{g} {sizes[g]}" for g in GROUPS) + f" = {len(entries)} files")
    folded = [e for e in entries if len(e["lists"]) > 1]
    if folded:
        print("run once, in the first list's slot, with every list's env: " + ", ".join(
            f"{e['file']} ({'+'.join(e['lists'])})" for e in folded))
    node = shutil.which("node")
    print(f"node: {node or 'absent (test_deck_scorer_parity JS comparisons return early)'}")

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
        for i, e in enumerate(entries, 1):
            f = e["file"]
            rc, out = _run([a.python, f"tests/{f}"], out_dir / "tests" / f"{f}.log", a.timeout,
                           extra_env=e["env"])
            res = parse_result(out, rc)
            res.update(group=e["group"], lists=e["lists"], env=e["env"],
                       missing_deps=not_gating_modules(out, rc, res["counted"],
                                                       e["may_miss"]))
            results[f] = res
            tag = "ok  " if res["ok"] else "FAIL"
            count = f"{res['passed']}/{res['total']}" if res["counted"] else "no count"
            dep = (f"  not gating: only {'+'.join(res['missing_deps'])} missing"
                   if res["missing_deps"] and e["group"] == "extras" else "")
            print(f"[{i}/{len(entries)}] {tag} {e['group']:<6} {f}  {count}  "
                  f"skips {res['skips']}{dep}", flush=True)
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
        t = tally(entries, results)
        failed = t["failed"]
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
            "files_gating": t["files"],
            "files_ok": t["files_ok"],
            "files_failed": failed,
            "files_failed_not_gating": t["not_gating"],
            "groups": t["groups"],
            "ran_once": {e["file"]: e["lists"] for e in entries if len(e["lists"]) > 1},
            "extras_run": not a.no_extras,
            "node": node,
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
        print("\n" + summary_line(t, summary["tests_passed"], summary["tests_total"],
                                  rc_claims, smoke))
        if failed:
            print("failed: " + ", ".join(failed))
        if t["not_gating"]:
            print("failed, not gating (only a module its NOT LISTED entry names is missing "
                  "here): " + ", ".join(
                f"{f} ({'+'.join(d)})" for f, d in sorted(t["not_gating"].items())))
        bad = bool(failed) or rc_claims != 0 or (smoke is not None and smoke["rc"] != 0)
        return 1 if bad else 0
    finally:
        release_lock(lock)


if __name__ == "__main__":
    sys.exit(main())
