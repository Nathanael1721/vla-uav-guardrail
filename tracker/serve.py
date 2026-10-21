"""Local server for the grant tracker site.

    python tracker/serve.py            # http://127.0.0.1:8765
    python tracker/serve.py --port 9000
    python tracker/serve.py --state some/other/state.json   # e.g. for tests
    python tracker/serve.py --open     # open the browser once the port is bound
    python tracker/serve.py --demo-out D:/path/demo/out     # the recorded runs elsewhere

Standard library only. Serves:
  /                  the site (tracker/index.html, tracker/assets/*)
  /data/...          the content JSON (tracker/data/*)
  /media/img/...     docs/img        (read only)
  /media/video/...   docs/video      (read only, with HTTP Range so videos seek)
  /media/share/...   docs/share      (read only)
  /media/run/...     demo/out        (read only, videos and images only, with
                     HTTP Range: the recorded runs, which are too large to
                     copy into docs/video)
  GET  /api/content  every content file merged into one JSON (cards, pages,
                     the update log and the project VERSION)
  GET  /api/state    the board's saved state (tracker/data/state.json)
  POST /api/state    replace that state (JSON body); written atomically
  GET  /api/dashboard  tracker/data/dashboard.json (tools/build_dashboard.py)
                     plus a "server" block: is it stale against its inputs,
                     and did a background `--quick` rebuild start. The
                     rebuild never makes posters or probes codecs, waits 30 s
                     between attempts, and is deferred while a flight runs
                     (its heartbeat is fresh or its process is alive between
                     beats: tools/live_status.py) or the live status cannot
                     be read (a possible flight), so it cannot slow a flight.
  GET  /api/live     the heartbeat files of running jobs (tools/live_status.py,
                     %LOCALAPPDATA%\\vla-guardrail\\live) and one nvidia-smi
                     snapshot, cached 5 s (30 s, without the process list,
                     while a flight runs). Only whitelisted fields: no log
                     content, no paths (a job's folder is shown as its
                     demo/out tag), no environment.
Both new endpoints are GET only (a POST to them is a 404) and pass the same
Host / Origin refusal as everything else. /api/content and /api/dashboard
answer with an ETag and a 304 to a matching If-None-Match, so the page's
visibility reload no longer downloads 1.2 MB to learn nothing changed.

Binds 127.0.0.1 only, so it is not exposed to the network. That alone does
not protect the board (review of 2026-10-19, harness-tools/tracker-state-
csrf): any web page open in the user's browser can make the BROWSER send a
request to 127.0.0.1:8765. A no-cors fetch or a plain auto-submitting form
with a text/plain body (no CORS preflight) replaced state.json with
`{"cards": {}}` - reproduced with Origin https://attacker.example - and a
DNS-rebinding page (Host attacker.example) could also read /api/state and
/api/content. tracker/data/ is gitignored, so that board had no backup. So,
before anything is read or written:
  * every request must carry Host 127.0.0.1:<port> or localhost:<port>
    (defeats DNS rebinding: a rebound name arrives with its own Host);
  * an Origin header, when present, must be this server's own origin
    (a browser always sends it on a cross-origin POST; "null" is refused);
  * POST /api/state must be Content-Type application/json, which a page
    cannot send cross-origin without a CORS preflight this server never
    answers (the board's own app.js already sends it).
A refused request gets a JSON 403. Requests with no Origin (curl, scripts,
the tests) are not a browser and are still accepted.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import mimetypes
import os
import posixpath
import re
import shutil
import subprocess
import tempfile
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DATA = HERE / "data"
STATE = DATA / "state.json"
VERSION_FILE = ROOT / "VERSION"
MEDIA = {
    "img": ROOT / "docs" / "img",
    "video": ROOT / "docs" / "video",
    "share": ROOT / "docs" / "share",
    "run": ROOT / "demo" / "out",
}
# demo/out also holds flight logs, audit records and replay bundles; from
# there only media files are served.
MEDIA_SUFFIXES = {"run": {".mp4", ".webm", ".png", ".jpg", ".jpeg"}}
MAX_STATE_BYTES = 5 * 1024 * 1024
_state_lock = threading.Lock()
TOOLS = ROOT / "tools"
BUILD_ROOT = ROOT               # the repository the dashboard builder reads
LIVE_DIR: Path | None = None    # None: tools/live_status.py live_dir()
AUTO_REBUILD = True
REBUILD_MIN_AGE_S = 30.0
GPU_CACHE_S = 5.0
# While a flight runs, nvidia-smi is spawned once (no process list) at most
# every GPU_CACHE_FLIGHT_S: the same reason the rebuild waits for a flight to
# end (review workflows halved OWL-ViT's rate, memory note no-workflows-
# during-flights) applies to periodic process spawns (review of 8035d0a, P9).
GPU_CACHE_FLIGHT_S = 30.0
GPU_APPS = re.compile(r"(?i)python|unreal|airsim|ffmpeg|ollama")
_rebuild = {"running": False, "last_try": 0.0, "error": None}
_rebuild_lock = threading.Lock()
_gpu = {"t": 0.0, "value": None, "flight": False}
_gpu_lock = threading.Lock()
_tools: dict = {}

mimetypes.add_type("video/mp4", ".mp4")
mimetypes.add_type("image/svg+xml", ".svg")
mimetypes.add_type("application/json", ".json")


def load_content() -> dict:
    """Cards from data/cards/batch_*.json in id order, plus the page files."""
    cards = []
    for f in sorted((DATA / "cards").glob("batch_*.json")):
        try:
            cards.extend(json.loads(f.read_text(encoding="utf-8")))
        except (OSError, ValueError) as exc:
            cards.append({"id": f.stem, "load_error": f"{type(exc).__name__}: {exc}"})
    out = {"cards": cards}
    for key, name in (("overview", "content_overview"), ("flow", "content_flow"), ("updates", "updates")):
        p = DATA / f"{name}.json"
        try:
            out[key] = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            out[key] = {"load_error": f"{type(exc).__name__}: {exc}"}
    try:
        out["project_version"] = VERSION_FILE.read_text(encoding="utf-8").strip()
    except OSError:
        out["project_version"] = None
    return out


def is_hidden(path: str) -> bool:
    """Working files are data/_*. Judged on the NORMALISED path: the static
    handler collapses '//', '/./' and '..' before it opens anything, so a
    check on the raw path let /data//_batches.json through."""
    norm = "/" + posixpath.normpath(path).lstrip("/")
    parts = norm.split("/")
    return len(parts) > 2 and parts[1] == "data" and any(seg.startswith("_") for seg in parts[2:])


def safe_join(base: Path, rel: str) -> Path | None:
    """`base / rel`, or None when rel escapes base."""
    p = (base / rel).resolve()
    try:
        p.relative_to(base.resolve())
    except ValueError:
        return None
    return p


def _tool(name: str):
    """tools/<name>.py, loaded by path once (standard library only)."""
    if name not in _tools:
        spec = importlib.util.spec_from_file_location(f"tracker_tool_{name}", TOOLS / f"{name}.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _tools[name] = mod
    return _tools[name]


def _live_snapshot() -> dict:
    ls = _tool("live_status")
    return ls.snapshot(folder=LIVE_DIR, demo_out=MEDIA["run"])


def _live_or_error() -> dict:
    """The live snapshot, or {"error", "jobs": []} when it cannot be read."""
    try:
        return _live_snapshot()
    except Exception as exc:                           # noqa: BLE001
        return {"error": f"live status unavailable ({type(exc).__name__})", "jobs": []}


def _flight_possible(snap: dict) -> bool:
    """A flight runs, or the live status could not be read and one may.
    The ONE reading of the snapshot for both the rebuild deferral and the
    GPU throttle (re-review of d180e01, P5: the deferral read an unreadable
    status as "no flight" and rebuilt, while /api/live read the same failure
    as a possible flight and throttled nvidia-smi). Defer when unsure: the
    reason either waits at all is that background work during a flight
    halved OWL-ViT's rate (memory note no-workflows-during-flights)."""
    return "error" in snap or bool(snap.get("flight_running"))


def _run_rebuild(P: dict) -> None:
    try:
        bd = _tool("build_dashboard")
        bd.write_atomic(P["out"], bd.build(P, quick=True))
        _rebuild["error"] = None
    except Exception as exc:                           # noqa: BLE001 - never kill the server
        _rebuild["error"] = type(exc).__name__         # the class only: no path, no text
    finally:
        _rebuild["running"] = False


def dashboard_payload() -> dict:
    """dashboard.json plus the server's view of it (module docstring)."""
    bd = _tool("build_dashboard")
    P = bd.paths(BUILD_ROOT, demo_out=MEDIA["run"], data_dir=DATA)
    doc = bd.read_json(P["out"])
    reasons = bd.stale_reasons(P, doc if isinstance(doc, dict) else None)
    # Only "an input is newer" or "no file" asks for a rebuild; stale card
    # text needs a person, not a build.
    want = any("newer than dashboard.json" in r or "does not exist" in r for r in reasons)
    action = "not_needed"
    if want:
        try:
            age = time.time() - P["out"].stat().st_mtime
        except OSError:
            age = 1e9
        with _rebuild_lock:
            if not AUTO_REBUILD:
                action = "off"
            elif _rebuild["running"]:
                action = "running"
            elif age < REBUILD_MIN_AGE_S or time.time() - _rebuild["last_try"] < REBUILD_MIN_AGE_S:
                action = "too_recent"
            elif _flight_possible(live := _live_or_error()):
                # the page says which: a flight, or a status it could not read
                action = "deferred_live_unreadable" if "error" in live else "deferred_flight"
            else:
                _rebuild.update(running=True, last_try=time.time())
                threading.Thread(target=_run_rebuild, args=(P,), daemon=True).start()
                action = "started"
    out = dict(doc) if isinstance(doc, dict) else {"missing": True}
    out["server"] = {"stale": bool(reasons), "stale_reasons": reasons, "rebuild": action,
                     "rebuild_error": _rebuild["error"]}
    return out


def _csv_rows(text: str) -> list:
    return [[c.strip() for c in ln.split(",")] for ln in (text or "").splitlines() if ln.strip()]


def _int(x):
    try:
        return int(float(x))
    except (TypeError, ValueError):
        return None


def gpu_snapshot(flight: bool = False) -> dict | None:
    """One nvidia-smi reading, cached GPU_CACHE_S; None without a GPU tool.
    Process names are reduced to their file name (no folders). While a
    flight runs (`flight`), one spawn without the process list, cached
    GPU_CACHE_FLIGHT_S (module constants); the reading says so."""
    now = time.time()
    with _gpu_lock:
        max_age = GPU_CACHE_FLIGHT_S if flight else GPU_CACHE_S
        # a reading reduced for a flight is not reused once the flight is over
        if now - _gpu["t"] < max_age and (flight or not _gpu.get("flight")):
            return _gpu["value"]
        _gpu["t"] = now
        _gpu["flight"] = bool(flight)
    value = None
    exe = shutil.which("nvidia-smi")
    if exe:
        kw = {"capture_output": True, "text": True, "timeout": 3}
        if os.name == "nt":
            kw["creationflags"] = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        try:
            g = subprocess.run([exe, "--query-gpu=name,utilization.gpu,memory.used,memory.total",
                                "--format=csv,noheader,nounits"], **kw)
            a = None if flight else subprocess.run(
                [exe, "--query-compute-apps=pid,process_name,used_memory",
                 "--format=csv,noheader,nounits"], **kw)
            gpus = []
            for row in _csv_rows(g.stdout)[:4]:
                if len(row) >= 4:
                    gpus.append({"name": row[0][:60], "util_pct": _int(row[1]),
                                 "mem_used_mib": _int(row[2]), "mem_total_mib": _int(row[3])})
            # On Windows every graphics process is listed, mostly with an
            # unknown memory size: keep the ones this project runs (Python,
            # Unreal, ffmpeg) and any with a known size; count the rest.
            apps, other = [], 0
            for row in _csv_rows(a.stdout if a is not None else "")[:64]:
                if len(row) >= 3:
                    name = re.split(r"[\\/]", row[1])[-1][:60]
                    mem = _int(row[2])
                    if mem is not None or GPU_APPS.search(name):
                        apps.append({"pid": _int(row[0]), "name": name, "mem_mib": mem})
                    else:
                        other += 1
            value = ({"gpus": gpus, "apps": None if a is None else apps[:16],
                      "other_apps": None if a is None else other,
                      "reduced_for_flight": a is None} if gpus else None)
        except (OSError, subprocess.SubprocessError, ValueError):
            value = None
    with _gpu_lock:
        _gpu["value"] = value
    return value


class Handler(SimpleHTTPRequestHandler):
    server_version = "GrantTracker/1.0"

    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(HERE), **kw)

    def log_message(self, fmt, *args):          # quiet: server errors only
        if len(args) > 1 and str(args[1]).startswith("5"):
            super().log_message(fmt, *args)

    def end_headers(self):
        # Content changes while the site is open; never serve a stale copy.
        if not self.path.startswith("/media/"):
            self.send_header("Cache-Control", "no-store")
        super().end_headers()

    # -------------------------------------------------------------- API
    def _json(self, obj, code=HTTPStatus.OK, etag=False):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        tag = None
        if etag and code == HTTPStatus.OK:
            tag = '"' + hashlib.sha256(body).hexdigest()[:24] + '"'
            if (self.headers.get("If-None-Match") or "").strip() == tag:
                self.send_response(HTTPStatus.NOT_MODIFIED)
                self.send_header("ETag", tag)
                self.end_headers()
                return
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        if tag:
            self.send_header("ETag", tag)
        self.end_headers()
        self.wfile.write(body)

    def _refusal(self, require_json: bool = False) -> str | None:
        """Why this request is not from the board's own page (module
        docstring), or None. Host and Origin are compared case-insensitively;
        the media type is the part of Content-Type before any ';', so
        `application/json; charset=utf-8` is accepted."""
        port = self.server.server_address[1]
        host = (self.headers.get("Host") or "").strip().lower()
        allowed = {f"127.0.0.1:{port}", f"localhost:{port}"}
        if port == 80:
            allowed |= {"127.0.0.1", "localhost"}
        if host not in allowed:
            return (f"Host {host or '(none)'!r} is not this server "
                    f"(127.0.0.1:{port} or localhost:{port})")
        origin = self.headers.get("Origin")
        if origin is not None and origin.strip().lower() != f"http://{host}":
            return f"Origin {origin!r} is not this server"
        if require_json:
            ctype = (self.headers.get("Content-Type") or "").split(";")[0].strip().lower()
            if ctype != "application/json":
                return f"Content-Type {ctype or '(none)'!r} is not application/json"
        return None

    def _forbidden(self, why: str):
        return self._json({"error": f"refused: {why}"}, HTTPStatus.FORBIDDEN)

    def do_HEAD(self):
        why = self._refusal()
        if why:
            return self._forbidden(why)
        return super().do_HEAD()

    def do_GET(self):
        why = self._refusal()
        if why:
            return self._forbidden(why)
        path = unquote(urlsplit(self.path).path)
        if path == "/api/content":
            return self._json(load_content(), etag=True)
        if path == "/api/dashboard":
            try:
                return self._json(dashboard_payload(), etag=True)
            except Exception as exc:                  # noqa: BLE001
                return self._json({"error": f"dashboard unavailable ({type(exc).__name__})"},
                                  HTTPStatus.INTERNAL_SERVER_ERROR)
        if path == "/api/live":
            snap = _live_or_error()
            # an unreadable live status is treated as a possible flight
            snap["gpu"] = gpu_snapshot(flight=_flight_possible(snap))
            return self._json(snap)
        if path == "/api/state":
            with _state_lock:
                try:
                    return self._json(json.loads(STATE.read_text(encoding="utf-8")))
                except FileNotFoundError:
                    return self._json({"cards": {}, "updated": None})
                except ValueError:
                    return self._json({"error": "state.json is not valid JSON"},
                                      HTTPStatus.INTERNAL_SERVER_ERROR)
        if path.startswith("/media/"):
            return self._media(path)
        if is_hidden(path):                       # working files, not content
            return self.send_error(HTTPStatus.NOT_FOUND)
        return super().do_GET()

    def do_POST(self):
        path = urlsplit(self.path).path
        if path != "/api/state":
            return self.send_error(HTTPStatus.NOT_FOUND)
        # Checked before the body is read (module docstring).
        why = self._refusal(require_json=True)
        if why:
            return self._forbidden(why)
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = -1                                # not a number: refuse, do not crash
        if n <= 0 or n > MAX_STATE_BYTES:
            return self._json({"error": "bad length"}, HTTPStatus.BAD_REQUEST)
        try:
            obj = json.loads(self.rfile.read(n).decode("utf-8"))
            if not isinstance(obj, dict) or not isinstance(obj.get("cards", {}), dict):
                raise ValueError("expected {cards: {...}}")
        except ValueError as exc:
            return self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        with _state_lock:
            STATE.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=STATE.parent, prefix=".state.", suffix=".tmp")
            try:
                with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as fh:
                    json.dump(obj, fh, ensure_ascii=False, indent=1)
                os.replace(tmp, STATE)
            except OSError as exc:
                # OneDrive can hold the file. Clean up and say so; the page
                # keeps the board in the browser and shows a warning.
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
                return self._json({"error": f"could not write state: {exc}"},
                                  HTTPStatus.INTERNAL_SERVER_ERROR)
        return self._json({"ok": True})

    # ------------------------------------------------------------ media
    def _media(self, path: str):
        m = re.match(r"^/media/([a-z]+)/(.+)$", path)
        if not m or m.group(1) not in MEDIA:
            return self.send_error(HTTPStatus.NOT_FOUND)
        f = safe_join(MEDIA[m.group(1)], m.group(2))
        if f is None or not f.is_file():
            return self.send_error(HTTPStatus.NOT_FOUND)
        allowed = MEDIA_SUFFIXES.get(m.group(1))
        if allowed is not None and f.suffix.lower() not in allowed:
            return self.send_error(HTTPStatus.NOT_FOUND)
        size = f.stat().st_size
        ctype = mimetypes.guess_type(f.name)[0] or "application/octet-stream"
        start, end = 0, size - 1
        rng = self.headers.get("Range")
        partial = False
        if rng:
            mm = re.match(r"bytes=(\d*)-(\d*)$", rng.strip())
            if mm:
                if mm.group(1):
                    start = int(mm.group(1))
                    if mm.group(2):
                        end = min(int(mm.group(2)), size - 1)
                elif mm.group(2):                 # suffix range: last N bytes
                    start = max(0, size - int(mm.group(2)))
                if start > end or start >= size:
                    self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.end_headers()
                    return
                partial = True
        length = end - start + 1
        self.send_response(HTTPStatus.PARTIAL_CONTENT if partial else HTTPStatus.OK)
        self.send_header("Content-Type", ctype)
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "max-age=3600")
        if partial:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        try:
            with f.open("rb") as fh:
                fh.seek(start)
                left = length
                while left > 0:
                    chunk = fh.read(min(1 << 20, left))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    left -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass                                     # the browser seeked away


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--state", default=None,
                    help="board state file (default tracker/data/state.json)")
    ap.add_argument("--open", action="store_true", help="open the browser once the port is bound")
    ap.add_argument("--demo-out", default=None,
                    help="the recorded runs served at /media/run/ and read by the dashboard "
                         "(default demo/out)")
    ap.add_argument("--live-dir", default=None, help="heartbeat folder (default: tools/live_status.py)")
    ap.add_argument("--no-rebuild", action="store_true",
                    help="never rebuild dashboard.json in the background")
    a = ap.parse_args()
    global STATE, LIVE_DIR, AUTO_REBUILD
    if a.state:
        STATE = Path(a.state).resolve()
        print(f"board state: {STATE}")
    if a.demo_out:
        MEDIA["run"] = Path(a.demo_out).resolve()
        print(f"recorded runs: {MEDIA['run']}")
    if a.live_dir:
        LIVE_DIR = Path(a.live_dir).resolve()
    AUTO_REBUILD = not a.no_rebuild
    try:
        httpd = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    except OSError as exc:
        print(f"could not listen on 127.0.0.1:{a.port} ({exc}); is the tracker already running?")
        raise SystemExit(1)
    url = f"http://127.0.0.1:{a.port}/"
    print(f"Grant tracker: {url}  (Ctrl+C to stop)")
    if a.open:
        # After the bind, so the first page load cannot race the server
        # (the old launcher opened the browser first).
        threading.Timer(0.2, webbrowser.open, args=(url,)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
