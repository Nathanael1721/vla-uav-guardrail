"""Live status for the tracker's Dashboard: one small heartbeat file per job.

    # in a flight, a harness run or a training script
    hb = live_status.heartbeat("flight", "pas_ap_ovf_on_20261021", log_dir=out_dir,
                               phase="starting", detail="connecting")
    hb.beat(detail="scene loaded")       # at each slow step outside the loop
    hb.beat(phase="airborne")
    hb.beat(progress=t / max_s)          # every tick if you like: no file IO here
    hb.beat(phase="landing")
    hb.stop("ok")                        # or let the process end

    # from a shell, a workflow script or PowerShell
    python tools/live_status.py start review-wave-v1 --kind workflow
    python tools/live_status.py beat  review-wave-v1 --progress 0.5
    python tools/live_status.py stop  review-wave-v1 --status ok
    python tools/live_status.py list

Standard library only. The Dashboard (tracker/serve.py GET /api/live) reads
the folder and shows one row per job, greys a row whose process is gone (or
whose heartbeat is older than STALE_S when the reader cannot tell whether the
process lives), and raises a banner when a FLIGHT and a WORKFLOW run at the
same time: review workflows halved OWL-ViT's rate during the 29 Sep flights
(memory note no-workflows-during-flights), so the overlap is the thing worth
seeing.

A JOB IS RUNNING WHILE ITS PROCESS LIVES, not only while it beats. The first
version judged by beat age alone, and the flights beat only inside their
mission loops: the 21 Oct ovf_on run took about 225 s, of which 120 s was
mission, and connect, scene load, SITL boot, EKF, OWL-ViT load, take-off and
landing went 30 s and more without a beat. The row read "no heartbeat",
`flight_running` was False, the server's "no rebuild during a flight" and the
flight + workflow banner switched off silently in the heaviest phases (review
of 8035d0a, P1). Now a record whose PID is alive counts as running, shown as
"quiet" (alive, no beat for N s) once its last beat is older than STALE_S,
unless that process was created after the record started (a reused PID:
pid_alive), and only up to QUIET_MAX_S where the creation time is unknown.
The flight rails also beat at their phase boundaries, with a `phase`
(starting / airborne / landing) and a short `detail`, so the row says where
the flight is.

WHY IT CANNOT DISTURB A FLIGHT (the writer runs inside demo/follow_vlm.py,
demo/pas_ardupilot/perception_node.py and experiments/sweep_scenarios.py):
  * `beat()` only stores numbers in memory under a lock. The file is written
    by a daemon thread, at most once per second (MIN_WRITE_INTERVAL_S), and
    only when something changed, so the control loop never waits on a disk,
    an antivirus scan or a sync client.
  * Every write is tmp + os.replace with a short retry, and every failure is
    swallowed: training/status.py:44-58 learned that monitoring must never
    kill a run (a transient PermissionError on Windows when a reader holds
    the file). A lost heartbeat is harmless; a crashed flight is not.
  * `heartbeat()` never raises either: on any error it returns a NullBeat
    whose methods do nothing.
  * The folder is %LOCALAPPDATA%\\vla-guardrail\\live (VLAGUARD_LIVE_DIR
    overrides it), outside OneDrive and outside every git worktree: a file
    rewritten every second inside a synced folder would churn the sync
    client and invite the PermissionError above (inferred, not measured).

A finished job's file moves to `_done/` with its end status; the newest
KEEP_DONE are kept. Nothing here reads or writes the tracker's board
(tracker/data/state.json).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

ENV_DIR = "VLAGUARD_LIVE_DIR"
ENV_OFF = "VLAGUARD_LIVE_OFF"
DEAD_HIDE_S = 6 * 3600.0        # a dead or stale job's row is dropped after this
KINDS = ("flight", "harness", "training", "workflow", "build")
STATUSES = ("ok", "failed", "stopped")
PHASES = ("starting", "airborne", "landing")    # a flight's phase, when it reports one
STALE_S = 30.0                  # an in-process heartbeat older than this is "quiet" or "stale"
# A live PID with no beat for longer than this is not taken on trust: an
# hour of silence is no flight phase this project has (the longest, a SITL
# boot plus EKF convergence, takes minutes), and Windows reuses PIDs, so the
# row reads "stale" again instead of "running" forever.
QUIET_MAX_S = 3600.0
# A process created more than this after the record's start is another
# process under a reused PID (pid_alive). The record's start is taken inside
# the process, so the real one is always older; the slack only absorbs the
# clocks' resolution (Linux btime is whole seconds).
PID_REUSE_SLACK_S = 2.0
LIVE_STATES = ("running", "quiet")   # the states that count as running
CLI_TTL_S = 6 * 3600.0          # a CLI-started job (no PID, no beats) after this is "stale"
MIN_WRITE_INTERVAL_S = 1.0      # at most one write per second per job
KEEP_DONE = 20
MAX_DETAIL = 200                # characters kept of a free-text detail
_SAFE = re.compile(r"[^A-Za-z0-9._-]+")


# ------------------------------------------------------------------ where
def live_dir() -> Path:
    """The heartbeat folder: VLAGUARD_LIVE_DIR, else %LOCALAPPDATA%\\vla-
    guardrail\\live (Windows), else ~/.local/state/vla-guardrail/live. Never
    inside the repository, whichever worktree this file is run from."""
    env = os.environ.get(ENV_DIR)
    if env:
        return Path(env)
    base = os.environ.get("LOCALAPPDATA")
    if base:
        return Path(base) / "vla-guardrail" / "live"
    return Path.home() / ".local" / "state" / "vla-guardrail" / "live"


def safe_name(name: str) -> str:
    s = _SAFE.sub("-", str(name or "")).strip("-.")
    return (s or "job")[:80]


def job_id(kind: str, name: str, pid: int | None) -> str:
    return f"{kind}-{safe_name(name)}" + (f"-{pid}" if pid else "")


def _iso(t: float) -> str:
    """Local time with its UTC offset, to the second."""
    return datetime.fromtimestamp(t).astimezone().isoformat(timespec="seconds")


# ------------------------------------------------------------------ io
def write_json_atomic(path: Path, obj: dict, retries: int = 3) -> bool:
    """tmp + os.replace with a short retry. Returns False instead of raising:
    a heartbeat that could not be written is skipped, never fatal."""
    for attempt in range(retries):
        tmp = path.with_name(f".{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp.write_text(json.dumps(obj, ensure_ascii=False), encoding="utf-8")
            os.replace(tmp, path)
            return True
        except Exception:                                  # noqa: BLE001 - never raise
            try:
                tmp.unlink()
            except Exception:                              # noqa: BLE001
                pass
            time.sleep(0.05 * (attempt + 1))
    return False


def _finish_file(path: Path, record: dict, folder: Path) -> None:
    """Move a job's file to _done/ with its end status; keep the newest
    KEEP_DONE there. Swallows every error."""
    try:
        done = folder / "_done"
        done.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        if write_json_atomic(done / f"{stamp}-{path.name}", record):
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        old = sorted(done.glob("*.json"), key=lambda p: p.name)
        for p in old[:-KEEP_DONE] if len(old) > KEEP_DONE else []:
            try:
                p.unlink()
            except Exception:                              # noqa: BLE001
                pass
    except Exception:                                      # noqa: BLE001
        pass


# ------------------------------------------------------------------ writer
class NullBeat:
    """What heartbeat() returns when it cannot work: every call is a no-op."""
    path = None

    def beat(self, progress=None, detail=None, phase=None, **_):
        return None

    def stop(self, status="ok", detail=None):
        return None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class Heartbeat:
    """One job's heartbeat. `beat()` touches memory only; a daemon thread
    writes the file at most once per MIN_WRITE_INTERVAL_S, when something
    changed (module docstring)."""

    def __init__(self, kind: str, name: str, *, log_dir: str | os.PathLike | None = None,
                 total_s: float | None = None, folder: Path | None = None,
                 pid: int | None = None, start_thread: bool = True,
                 min_interval_s: float = MIN_WRITE_INTERVAL_S, clock=time.time,
                 phase: str | None = None, detail: str | None = None):
        if kind not in KINDS:
            raise ValueError(f"kind must be one of {KINDS}, not {kind!r}")
        self._clock = clock
        self.folder = Path(folder) if folder is not None else live_dir()
        self.pid = os.getpid() if pid is None else pid
        now = clock()
        self.record = {
            "schema": 1, "id": job_id(kind, name, self.pid), "kind": kind,
            "name": str(name)[:120], "pid": self.pid, "cli": False,
            "started": now, "heartbeat": now, "progress": None,
            "detail": None if detail is None else str(detail)[:MAX_DETAIL],
            "phase": phase if phase in PHASES else None,
            "total_s": total_s, "log_dir": str(log_dir) if log_dir else None,
            "status": "running", "ended": None,
        }
        self.path = self.folder / f"{self.record['id']}.json"
        self._lock = threading.Lock()          # the record (beat)
        self._io = threading.Lock()            # the file (flush / stop)
        self._dirty = True
        self._stopped = False
        self._min = float(min_interval_s)
        self._last_write = 0.0
        self.writes = 0
        self._wake = threading.Event()
        self._thread = None
        if start_thread:
            self._thread = threading.Thread(target=self._run, name="live-status", daemon=True)
            self._thread.start()

    # called from the control loop: memory only
    def beat(self, progress: float | None = None, detail: str | None = None,
             phase: str | None = None, **_) -> None:
        """Memory only. `phase` is kept only when it is one of PHASES (a
        typo is ignored, never raised)."""
        try:
            with self._lock:
                self.record["heartbeat"] = self._clock()
                if progress is not None:
                    self.record["progress"] = max(0.0, min(1.0, float(progress)))
                if detail is not None:
                    self.record["detail"] = str(detail)[:MAX_DETAIL]
                if phase in PHASES:
                    self.record["phase"] = phase
                self._dirty = True
        except Exception:                                  # noqa: BLE001
            pass

    def flush(self, force: bool = False) -> bool:
        """Write the record if it changed and the interval has passed (or
        force). Called by the writer thread; tests call it directly. Holds
        the IO lock, so stop() never races a write into a ghost file."""
        try:
            with self._io:
                if self._stopped:
                    return False
                now = self._clock()
                with self._lock:
                    if not self._dirty or (not force and now - self._last_write < self._min):
                        return False
                    rec = dict(self.record)
                    self._dirty = False
                    self._last_write = now
                ok = write_json_atomic(self.path, rec)
                if ok:
                    self.writes += 1
                else:
                    with self._lock:
                        self._dirty = True
                return ok
        except Exception:                                  # noqa: BLE001
            return False

    def _run(self) -> None:
        while not self._stopped:
            self._wake.wait(self._min)
            self._wake.clear()
            if self._stopped:
                break
            self.flush()

    def stop(self, status: str = "ok", detail: str | None = None) -> None:
        """End the job: its file moves to _done/ with the status. Not for the
        control loop (it may wait for a write in progress); never raises."""
        try:
            with self._io:
                if self._stopped:
                    return
                self._stopped = True
                with self._lock:
                    self.record["status"] = status if status in STATUSES else "stopped"
                    self.record["ended"] = self._clock()
                    if detail is not None:
                        self.record["detail"] = str(detail)[:MAX_DETAIL]
                    rec = dict(self.record)
                _finish_file(self.path, rec, self.folder)
            self._wake.set()
        except Exception:                                  # noqa: BLE001
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop("failed" if exc_type else "ok")
        return False


def heartbeat(kind: str, name: str, **kw):
    """A Heartbeat, or a NullBeat when anything goes wrong or when
    VLAGUARD_LIVE_OFF is set (e.g. for a scripted run that should not show
    on the Dashboard). Never raises."""
    try:
        if os.environ.get(ENV_OFF):
            return NullBeat()
        return Heartbeat(kind, name, **kw)
    except Exception:                                      # noqa: BLE001
        return NullBeat()


# ------------------------------------------------------------------ reader
def pid_created(pid) -> float | None:
    """When the process with this PID was created (epoch seconds), or None
    when it cannot tell. Windows: GetProcessTimes (no psutil); Linux:
    /proc/<pid>/stat starttime + /proc/stat btime (1 s resolution)."""
    try:
        pid = int(pid)
    except (TypeError, ValueError):
        return None
    if pid <= 0:
        return None
    try:
        if os.name == "nt":
            import ctypes
            from ctypes import wintypes
            k32 = ctypes.WinDLL("kernel32", use_last_error=True)
            k32.OpenProcess.restype = wintypes.HANDLE
            h = k32.OpenProcess(0x1000, False, pid)        # PROCESS_QUERY_LIMITED_INFORMATION
            if not h:
                return None
            try:
                ft = [wintypes.FILETIME() for _ in range(4)]   # creation, exit, kernel, user
                if not k32.GetProcessTimes(h, *(ctypes.byref(f) for f in ft)):
                    return None
                ticks = (ft[0].dwHighDateTime << 32) | ft[0].dwLowDateTime   # 100 ns since 1601
                return ticks / 1e7 - 11644473600.0
            finally:
                k32.CloseHandle(h)
        stat = Path(f"/proc/{pid}/stat").read_text()
        start_ticks = int(stat.rsplit(")", 1)[1].split()[19])   # field 22, starttime
        btime = next(int(ln.split()[1]) for ln in Path("/proc/stat").read_text().splitlines()
                     if ln.startswith("btime "))
        return btime + start_ticks / os.sysconf("SC_CLK_TCK")
    except Exception:                                      # noqa: BLE001 - macOS, no /proc, a race
        return None


def pid_alive(pid, started: float | None = None) -> bool | None:
    """True / False when it can tell, None when it cannot. Windows: OpenProcess
    + GetExitCodeProcess (no psutil); elsewhere os.kill(pid, 0).

    With `started` (the record's own start time), a live process CREATED
    after it (by more than PID_REUSE_SLACK_S) is not that job's process but
    a reuse of its PID: False. A heartbeat is made inside its process, so
    the real one was always created first (re-review of d180e01, P4: a
    hard-killed flight's file read "quiet", alive, for up to QUIET_MAX_S
    when Windows handed its PID to another process, which kept the rebuild
    deferred and could show the overlap banner for a flight long gone).
    QUIET_MAX_S stays as the backstop where the creation time is unknown."""
    lives = _pid_running(pid)
    if lives is True and started is not None:
        born = pid_created(pid)
        if born is not None and born > started + PID_REUSE_SLACK_S:
            return False
    return lives


def _pid_running(pid) -> bool | None:
    try:
        pid = int(pid)
    except (TypeError, ValueError):
        return None
    if pid <= 0:
        return None
    if os.name == "nt":
        try:
            import ctypes
            from ctypes import wintypes
            k32 = ctypes.WinDLL("kernel32", use_last_error=True)
            k32.OpenProcess.restype = wintypes.HANDLE
            h = k32.OpenProcess(0x1000, False, pid)        # PROCESS_QUERY_LIMITED_INFORMATION
            if not h:
                err = ctypes.get_last_error()
                return False if err == 87 else None        # ERROR_INVALID_PARAMETER: no such pid
            try:
                code = wintypes.DWORD()
                if not k32.GetExitCodeProcess(h, ctypes.byref(code)):
                    return None
                return code.value == 259                   # STILL_ACTIVE
            finally:
                k32.CloseHandle(h)
        except Exception:                                  # noqa: BLE001
            return None
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except Exception:                                      # noqa: BLE001
        return None


def _num(x):
    return x if isinstance(x, (int, float)) and not isinstance(x, bool) else None


def run_tag_of(log_dir, demo_out: Path | None) -> str | None:
    """The demo/out run folder a job writes to, as a bare tag, or None. The
    path itself is never shown (it may name a user folder)."""
    if not log_dir or demo_out is None:
        return None
    try:
        p = Path(str(log_dir)).resolve()
        rel = p.relative_to(Path(demo_out).resolve())
        tag = rel.parts[0] if rel.parts else None
        return tag if tag and not tag.startswith("_") else None
    except Exception:                                      # noqa: BLE001
        return None


def sanitize(rec: dict, now: float, demo_out: Path | None = None,
             alive=None) -> dict | None:
    """The fields the Dashboard may show, with its state judged; None for a
    record that is not a heartbeat. Free text is cut to MAX_DETAIL; the log
    folder becomes a demo/out tag or nothing. `alive(pid)` stands in for the
    process check in tests; by default it is pid_alive with the record's own
    start, so a reused PID reads "dead" (pid_alive)."""
    if not isinstance(rec, dict) or rec.get("kind") not in KINDS:
        return None
    started, beat = _num(rec.get("started")), _num(rec.get("heartbeat"))
    if started is None or beat is None:
        return None
    cli = bool(rec.get("cli"))
    pid = rec.get("pid") if isinstance(rec.get("pid"), int) else None
    age = max(0.0, now - beat)
    state = "running"
    if rec.get("status") not in (None, "running"):
        state = "ended"
    else:
        lives = None
        if not cli and pid is not None:
            lives = pid_alive(pid, started) if alive is None else alive(pid)
        if lives is False:
            state = "dead"
        elif age > (CLI_TTL_S if cli else STALE_S):
            # Between beats is not stopped: a flight loading its scene,
            # booting SITL or landing beats seldom, and its process is the
            # proof it is still there (module docstring). Without that proof
            # (no PID, a reader that cannot tell, an hour of silence) the
            # beat age alone decides, as before.
            state = "quiet" if (lives is True and age <= QUIET_MAX_S) else "stale"
    prog = _num(rec.get("progress"))
    return {
        "id": safe_name(rec.get("id") or ""), "kind": rec["kind"],
        "name": str(rec.get("name") or "")[:120], "pid": pid, "cli": cli,
        "started": _iso(started), "heartbeat": _iso(beat), "age_s": round(age, 1),
        "progress": None if prog is None else max(0.0, min(1.0, prog)),
        "detail": None if rec.get("detail") is None else str(rec["detail"])[:MAX_DETAIL],
        "phase": rec.get("phase") if rec.get("phase") in PHASES else None,
        "total_s": _num(rec.get("total_s")), "state": state,
        "status": rec.get("status") if rec.get("status") in STATUSES + ("running",) else None,
        "ended": _iso(rec["ended"]) if _num(rec.get("ended")) is not None else None,
        "run_tag": run_tag_of(rec.get("log_dir"), demo_out),
    }


def snapshot(folder: Path | None = None, now: float | None = None,
             demo_out: Path | None = None, alive=None, done_n: int = 5) -> dict:
    """Everything GET /api/live returns except the GPU row."""
    folder = Path(folder) if folder is not None else live_dir()
    now = time.time() if now is None else now
    jobs, done = [], []
    readable = folder.is_dir()
    if readable:
        for p in sorted(folder.glob("*.json")):
            try:
                rec = json.loads(p.read_text(encoding="utf-8"))
            except Exception:                              # noqa: BLE001 - half-written or foreign
                continue
            s = sanitize(rec, now, demo_out, alive)
            # A job that died without stop() keeps its file; its row says
            # "stopped unexpectedly" for DEAD_HIDE_S, then drops out.
            if s and not (s["state"] in ("dead", "stale") and s["age_s"] > DEAD_HIDE_S):
                jobs.append(s)
        dd = folder / "_done"
        if dd.is_dir():
            for p in sorted(dd.glob("*.json"), key=lambda q: q.name, reverse=True)[:done_n]:
                try:
                    s = sanitize(json.loads(p.read_text(encoding="utf-8")), now, demo_out, alive)
                except Exception:                          # noqa: BLE001
                    continue
                if s:
                    s["state"] = "ended"
                    done.append(s)
    # "quiet" counts: a live flight between beats still defers the server's
    # rebuild and still raises the overlap banner (review of 8035d0a, P1).
    running = [j for j in jobs if j["state"] in LIVE_STATES]
    kinds = {j["kind"] for j in running}
    return {
        "now": _iso(now), "dir_ok": readable, "jobs": jobs, "done": done,
        "running_kinds": sorted(kinds),
        "overlap": "flight" in kinds and "workflow" in kinds,
        "flight_running": "flight" in kinds,
    }


# ------------------------------------------------------------------ cli
def _cli_path(folder: Path, kind: str, name: str) -> Path:
    return folder / f"{job_id(kind, name, None)}.json"


def cli_start(name: str, kind: str = "workflow", detail: str | None = None,
              folder: Path | None = None, now: float | None = None) -> Path:
    folder = Path(folder) if folder is not None else live_dir()
    now = time.time() if now is None else now
    rec = {"schema": 1, "id": job_id(kind, name, None), "kind": kind, "name": str(name)[:120],
           "pid": None, "cli": True, "started": now, "heartbeat": now, "progress": None,
           "detail": None if detail is None else str(detail)[:MAX_DETAIL], "total_s": None,
           "log_dir": None, "status": "running", "ended": None}
    p = _cli_path(folder, kind, name)
    write_json_atomic(p, rec)
    return p


def cli_beat(name: str, kind: str = "workflow", progress=None, detail=None,
             folder: Path | None = None, now: float | None = None) -> bool:
    folder = Path(folder) if folder is not None else live_dir()
    p = _cli_path(folder, kind, name)
    try:
        rec = json.loads(p.read_text(encoding="utf-8"))
    except Exception:                                      # noqa: BLE001
        return False
    rec["heartbeat"] = time.time() if now is None else now
    if progress is not None:
        rec["progress"] = max(0.0, min(1.0, float(progress)))
    if detail is not None:
        rec["detail"] = str(detail)[:MAX_DETAIL]
    return write_json_atomic(p, rec)


def cli_stop(name: str, kind: str = "workflow", status: str = "ok", detail=None,
             folder: Path | None = None, now: float | None = None) -> bool:
    folder = Path(folder) if folder is not None else live_dir()
    p = _cli_path(folder, kind, name)
    try:
        rec = json.loads(p.read_text(encoding="utf-8"))
    except Exception:                                      # noqa: BLE001
        return False
    rec["status"] = status if status in STATUSES else "stopped"
    rec["ended"] = time.time() if now is None else now
    if detail is not None:
        rec["detail"] = str(detail)[:MAX_DETAIL]
    _finish_file(p, rec, folder)
    return not p.exists()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Heartbeat files for the tracker's Dashboard.")
    sp = ap.add_subparsers(dest="cmd", required=True)
    for c in ("start", "beat", "stop"):
        x = sp.add_parser(c)
        x.add_argument("name")
        x.add_argument("--kind", choices=KINDS, default="workflow")
        x.add_argument("--detail", default=None)
        if c == "beat":
            x.add_argument("--progress", type=float, default=None)
        if c == "stop":
            x.add_argument("--status", choices=STATUSES, default="ok")
    sp.add_parser("list")
    sp.add_parser("where")
    a = ap.parse_args(argv)
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass
    if a.cmd == "where":
        print(live_dir())
        return 0
    if a.cmd == "start":
        print(cli_start(a.name, a.kind, a.detail))
        return 0
    if a.cmd in ("beat", "stop"):
        ok = (cli_beat(a.name, a.kind, a.progress, a.detail) if a.cmd == "beat"
              else cli_stop(a.name, a.kind, a.status, a.detail))
        if not ok:
            # Said, not only an exit code: a stop with the wrong --kind used
            # to leave the row running without a word.
            print(f"no running {a.kind} job named {a.name!r} (start it with the same --kind)",
                  file=sys.stderr)
        return 0 if ok else 1
    snap = snapshot()
    for j in snap["jobs"]:
        prog = "" if j["progress"] is None else f" {100 * j['progress']:.0f}%"
        phase = f" [{j['phase']}]" if j.get("phase") else ""
        print(f"{j['state']:<8} {j['kind']:<9} {j['name']}{phase}{prog}  beat {j['age_s']} s ago")
    if snap["overlap"]:
        print("WARNING: a flight and a workflow are running at the same time")
    if not snap["jobs"]:
        print(f"nothing running ({live_dir()})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
