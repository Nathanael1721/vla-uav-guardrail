"""The Dashboard's live status: tools/live_status.py and its hooks.

Run either way:
    python tests/test_live_status.py
    pytest tests/test_live_status.py -v

Every test writes into its own temporary folder (never the real
%LOCALAPPDATA%\\vla-guardrail\\live). Pinned, each because a monitor that
misbehaves inside a flight is worse than no monitor:
- beat() writes nothing: the file appears only when the writer flushes, at
  most once per second, and the write is atomic (no temp file left);
- a failing write (os.replace raising, an unwritable folder, a bad kind)
  never raises into the caller;
- a heartbeat older than STALE_S reads "stale", a gone process "dead", and a
  CLI-started workflow (no PID, no beats) stays "running" until its TTL;
- the flight + workflow banner flag is set only when both are running;
- stop() moves the file to _done/ with its status and keeps the newest 20;
- the default folder is %LOCALAPPDATA%\\vla-guardrail\\live (or
  VLAGUARD_LIVE_DIR), never inside the repository, whichever worktree runs;
- what the Dashboard may show is whitelisted: no log folder path (a demo/out
  tag at most), no extra keys, free text cut;
- the three hooked scripts (demo/follow_vlm.py, demo/pas_ardupilot/
  perception_node.py, experiments/sweep_scenarios.py) build their heartbeat
  inside try/except that falls back to a no-op, call only beat() inside the
  loop, and the perception node's run() leaves it off unless main() asks
  (its tests call run() directly).
"""
import ast
import json
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import live_status as LS                                        # noqa: E402

SKIP = "SKIP"


class Tmp:
    def __enter__(self):
        self.p = Path(tempfile.mkdtemp(prefix="live_test_"))
        return self.p

    def __exit__(self, *a):
        shutil.rmtree(self.p, ignore_errors=True)


class Clock:
    def __init__(self, t=1_800_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


# ------------------------------------------------------------------ writer
def test_beat_writes_nothing_until_the_writer_flushes_and_the_write_is_atomic():
    with Tmp() as d:
        hb = LS.Heartbeat("flight", "pas_ap_x", folder=d, start_thread=False, log_dir=d / "out")
        for i in range(500):
            hb.beat(progress=i / 500)
        assert not list(d.glob("*.json")), "beat() touched the disk"
        assert hb.flush(force=True)
        files = list(d.glob("*.json"))
        assert len(files) == 1 and files[0].name.startswith("flight-pas_ap_x-")
        rec = json.loads(files[0].read_text(encoding="utf-8"))
        assert rec["kind"] == "flight" and rec["pid"] == os.getpid() and abs(rec["progress"] - 499 / 500) < 1e-9
        assert not [p for p in d.iterdir() if p.name.endswith(".tmp")], "a temp file was left"


def test_at_most_one_write_per_second():
    with Tmp() as d:
        clk = Clock()
        hb = LS.Heartbeat("harness", "sweep", folder=d, start_thread=False, clock=clk)
        assert hb.flush()                         # first write
        for _ in range(9):                        # 0.9 s of 10 Hz beats
            clk.t += 0.1
            hb.beat(progress=0.5)
            assert not hb.flush(), "wrote again within a second"
        clk.t += 0.15
        assert hb.flush() and hb.writes == 2
        assert not hb.flush(), "nothing changed, nothing to write"


def test_a_failed_write_never_raises():
    with Tmp() as d:
        real = LS.os.replace

        def boom(a, b):
            raise PermissionError("held by a reader")
        LS.os.replace = boom
        try:
            hb = LS.Heartbeat("flight", "f", folder=d, start_thread=False)
            hb.beat(progress=0.1)
            assert hb.flush(force=True) is False
            hb.beat(progress=0.2)                 # still fine
            hb.stop("ok")                         # never raises either
        finally:
            LS.os.replace = real
        assert not [p for p in d.rglob("*.tmp")], "temp files left after failed writes"
        # a folder that is a FILE, and a bad kind: a no-op object, no exception
        blocker = d / "blocker"
        blocker.write_text("x", encoding="utf-8")
        hb = LS.heartbeat("flight", "f", folder=blocker / "live", start_thread=False)
        hb.beat(progress=0.3)
        assert hb.flush(force=True) is False
        hb.stop("failed")
        nb = LS.heartbeat("not-a-kind", "f", folder=d)
        assert isinstance(nb, LS.NullBeat)
        nb.beat(progress=1.0)
        nb.stop()


def test_the_off_switch_returns_a_no_op():
    with Tmp() as d:
        os.environ[LS.ENV_OFF] = "1"
        try:
            hb = LS.heartbeat("flight", "f", folder=d)
        finally:
            del os.environ[LS.ENV_OFF]
        assert isinstance(hb, LS.NullBeat)
        hb.beat()
        assert not list(d.iterdir())


def test_the_writer_thread_writes_and_stop_moves_the_file_to_done():
    with Tmp() as d:
        hb = LS.Heartbeat("flight", "threaded", folder=d, min_interval_s=0.05)
        hb.beat(progress=0.4)
        t_end = time.time() + 5.0
        while time.time() < t_end and not list(d.glob("*.json")):
            time.sleep(0.02)
        assert list(d.glob("*.json")), "the writer thread never wrote"
        hb.stop("ok")
        assert not list(d.glob("*.json")), "the live file is still there after stop()"
        done = list((d / "_done").glob("*.json"))
        assert len(done) == 1 and json.loads(done[0].read_text(encoding="utf-8"))["status"] == "ok"
        time.sleep(0.2)                           # the thread has exited: nothing reappears
        assert not list(d.glob("*.json"))


def test_done_keeps_the_newest_twenty():
    with Tmp() as d:
        dd = d / "_done"
        dd.mkdir()
        for i in range(25):
            (dd / f"20261021-0000{i:02d}-old.json").write_text("{}", encoding="utf-8")
        hb = LS.Heartbeat("build", "b", folder=d, start_thread=False)
        hb.flush(force=True)
        hb.stop("ok")
        assert len(list(dd.glob("*.json"))) == LS.KEEP_DONE


# ------------------------------------------------------------------ reader
def _rec(kind="flight", age=1.0, now=1_800_000_000.0, **kw):
    r = {"schema": 1, "id": f"{kind}-x", "kind": kind, "name": "x", "pid": 4242, "cli": False,
         "started": now - 100, "heartbeat": now - age, "progress": 0.5, "detail": None,
         "status": "running"}
    r.update(kw)
    return r


def test_stale_dead_and_cli_states():
    now = 1_800_000_000.0
    alive = lambda pid: True                                 # noqa: E731
    gone = lambda pid: False                                 # noqa: E731
    assert LS.sanitize(_rec(age=2), now, alive=alive)["state"] == "running"
    assert LS.sanitize(_rec(age=LS.STALE_S + 1), now, alive=alive)["state"] == "stale"
    assert LS.sanitize(_rec(age=2), now, alive=gone)["state"] == "dead"
    cli = dict(pid=None, cli=True)
    assert LS.sanitize(_rec("workflow", age=3600, **cli), now, alive=gone)["state"] == "running"
    assert LS.sanitize(_rec("workflow", age=LS.CLI_TTL_S + 1, **cli), now, alive=gone)["state"] == "stale"
    assert LS.sanitize(_rec(status="ok", ended=now), now, alive=alive)["state"] == "ended"
    assert LS.sanitize({"kind": "nope"}, now) is None and LS.sanitize("x", now) is None


def test_the_overlap_flag_needs_a_running_flight_and_a_running_workflow():
    now = 1_800_000_000.0
    alive = lambda pid: True                                 # noqa: E731

    def snap(*recs):
        with Tmp() as d:
            for i, r in enumerate(recs):
                (d / f"j{i}.json").write_text(json.dumps(r), encoding="utf-8")
            return LS.snapshot(d, now=now, alive=alive)
    wf = _rec("workflow", pid=None, cli=True, age=10)
    assert snap(_rec("flight"), wf)["overlap"] is True
    assert snap(_rec("flight"), _rec("harness"))["overlap"] is False
    assert snap(_rec("flight", age=LS.STALE_S + 5), wf)["overlap"] is False
    assert snap(wf)["overlap"] is False
    s = snap(_rec("flight"), wf)
    assert s["flight_running"] is True and s["running_kinds"] == ["flight", "workflow"]


def test_a_half_written_or_foreign_file_is_skipped():
    with Tmp() as d:
        (d / "bad.json").write_text("{not json", encoding="utf-8")
        (d / "other.json").write_text(json.dumps({"hello": 1}), encoding="utf-8")
        s = LS.snapshot(d)
        assert s["jobs"] == [] and s["dir_ok"] is True
        assert LS.snapshot(d / "missing")["dir_ok"] is False


def test_only_whitelisted_fields_reach_the_dashboard():
    now = 1_800_000_000.0
    with Tmp() as d:
        demo = d / "demo" / "out"
        (demo / "pas_ap_x").mkdir(parents=True)
        r = _rec(log_dir=str(demo / "pas_ap_x"), detail="y" * 900,
                 env={"SECRET_TOKEN": "hunter2"}, cmd="python secret.py --key abc")
        s = LS.sanitize(r, now, demo_out=demo, alive=lambda p: True)
        assert s["run_tag"] == "pas_ap_x" and len(s["detail"]) == LS.MAX_DETAIL
        text = json.dumps(s)
        for leak in ("hunter2", "SECRET_TOKEN", "secret.py", str(demo), "log_dir"):
            assert leak not in text, leak
        outside = LS.sanitize(_rec(log_dir=str(d / "elsewhere")), now, demo_out=demo, alive=lambda p: True)
        assert outside["run_tag"] is None


def test_the_default_folder_is_outside_the_repository():
    saved = {k: os.environ.get(k) for k in (LS.ENV_DIR, "LOCALAPPDATA")}
    try:
        os.environ.pop(LS.ENV_DIR, None)
        os.environ["LOCALAPPDATA"] = r"C:\Users\someone\AppData\Local"
        assert LS.live_dir() == Path(r"C:\Users\someone\AppData\Local") / "vla-guardrail" / "live"
        os.environ[LS.ENV_DIR] = "X:/elsewhere/live"
        assert LS.live_dir() == Path("X:/elsewhere/live")
    finally:
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    if os.environ.get(LS.ENV_DIR) is None and os.environ.get("LOCALAPPDATA"):
        real = LS.live_dir().resolve()
        assert ROOT.resolve() not in real.parents and real != ROOT.resolve(), real
        assert "onedrive" not in str(real).lower(), real


def test_pid_alive_knows_this_process():
    assert LS.pid_alive(os.getpid()) is True
    assert LS.pid_alive(2 ** 30) is not True
    assert LS.pid_alive(None) is None and LS.pid_alive(-3) is None


def test_the_cli_start_beat_stop_round_trip():
    with Tmp() as d:
        p = LS.cli_start("review wave V1", "workflow", detail="3 reviewers", folder=d)
        assert p.is_file()
        rec = json.loads(p.read_text(encoding="utf-8"))
        assert rec["cli"] is True and rec["pid"] is None and rec["kind"] == "workflow"
        assert LS.cli_beat("review wave V1", "workflow", progress=0.5, folder=d)
        assert LS.snapshot(d)["jobs"][0]["progress"] == 0.5
        assert LS.cli_stop("review wave V1", "workflow", "ok", folder=d)
        assert not p.exists() and len(list((d / "_done").glob("*.json"))) == 1
        assert LS.cli_stop("never started", "workflow", folder=d) is False


def test_a_cli_stop_with_the_wrong_kind_says_so_and_keeps_the_row():
    """Found while probing /api/live on 2026-10-21: `stop <name>` without the
    `--kind flight` it was started with exited 1 silently and the row stayed
    "running"; the CLI now says why on stderr."""
    import contextlib
    import io
    with Tmp() as d:
        old = os.environ.get(LS.ENV_DIR)
        os.environ[LS.ENV_DIR] = str(d)
        try:
            LS.cli_start("probe", "flight", folder=d)
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                rc = LS.main(["stop", "probe"])
            assert rc == 1 and "no running workflow job named 'probe'" in err.getvalue(), err.getvalue()
            assert LS.snapshot(d)["flight_running"] is True
            assert LS.main(["stop", "probe", "--kind", "flight"]) == 0
            assert LS.snapshot(d)["flight_running"] is False
        finally:
            if old is None:
                os.environ.pop(LS.ENV_DIR, None)
            else:
                os.environ[LS.ENV_DIR] = old


# ------------------------------------------------------------------ hooks
HOOKED = ["demo/follow_vlm.py", "demo/pas_ardupilot/perception_node.py", "experiments/sweep_scenarios.py"]


def _fn(tree, name):
    return next((n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name), None)


def test_each_hook_falls_back_to_a_no_op_and_beats_in_its_loop():
    for rel in HOOKED:
        src = (ROOT / rel).read_text(encoding="utf-8")
        tree = ast.parse(src)
        fn = _fn(tree, "_live_heartbeat")
        assert fn is not None, f"{rel}: no _live_heartbeat"
        tries = [n for n in fn.body if isinstance(n, ast.Try)]
        assert tries, f"{rel}: _live_heartbeat is not wrapped in try"
        h = tries[0].handlers
        assert h and isinstance(h[0].type, ast.Name) and h[0].type.id == "Exception", rel
        ret = h[0].body[-1]
        assert isinstance(ret, ast.Return) and "_NoBeat" in ast.unparse(ret), f"{rel}: the fallback is not the no-op"
        assert "live_status.py" in ast.unparse(fn), rel
        # inside a for/while loop only beat() is called on the heartbeat
        calls = [n for n in ast.walk(tree) if isinstance(n, (ast.For, ast.While))]
        beats = [c for loop in calls for c in ast.walk(loop)
                 if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
                 and c.func.attr in ("beat", "stop", "flush")
                 and isinstance(c.func.value, ast.Name) and c.func.value.id in ("live", "_LIVE")]
        assert beats and all(c.func.attr == "beat" for c in beats), \
            f"{rel}: the loop calls {[c.func.attr for c in beats]}"
    # the no-op classes really are no-ops
    for rel in HOOKED:
        tree = ast.parse((ROOT / rel).read_text(encoding="utf-8"))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "_NoBeat")
        for m in cls.body:
            assert isinstance(m, ast.FunctionDef) and ast.unparse(m.body[-1]) == "return None", rel


def test_the_perception_node_writes_no_heartbeat_unless_main_asks():
    tree = ast.parse((ROOT / "demo/pas_ardupilot/perception_node.py").read_text(encoding="utf-8"))
    run = _fn(tree, "run")
    kw = {a.arg: d for a, d in zip(run.args.kwonlyargs, run.args.kw_defaults)}
    assert "live_status" in kw and isinstance(kw["live_status"], ast.Constant) and kw["live_status"].value is False
    main = _fn(tree, "main")
    assert "live_status=" in ast.unparse(main)
    sweep = ast.parse((ROOT / "experiments/sweep_scenarios.py").read_text(encoding="utf-8"))
    mod_live = [n for n in sweep.body if isinstance(n, ast.Assign) and any(
        isinstance(t, ast.Name) and t.id == "_LIVE" for t in n.targets)]
    assert mod_live and "_NoBeat()" in ast.unparse(mod_live[0]), "sweep's module default is not the no-op"


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
