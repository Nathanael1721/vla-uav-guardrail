"""Local server for the grant tracker site.

    python tracker/serve.py            # http://127.0.0.1:8765
    python tracker/serve.py --port 9000
    python tracker/serve.py --state some/other/state.json   # e.g. for tests

Standard library only. Serves:
  /                  the site (tracker/index.html, tracker/assets/*)
  /data/...          the content JSON (tracker/data/*)
  /media/img/...     docs/img        (read only)
  /media/video/...   docs/video      (read only, with HTTP Range so videos seek)
  /media/share/...   docs/share      (read only)
  GET  /api/content  every content file merged into one JSON (cards, pages,
                     the update log and the project VERSION)
  GET  /api/state    the board's saved state (tracker/data/state.json)
  POST /api/state    replace that state (JSON body); written atomically

Binds 127.0.0.1 only: the state file is written by whoever can reach the
port, so it is not exposed to the network.
"""
from __future__ import annotations

import argparse
import json
import mimetypes
import os
import posixpath
import re
import tempfile
import threading
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
}
MAX_STATE_BYTES = 5 * 1024 * 1024
_state_lock = threading.Lock()

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
    def _json(self, obj, code=HTTPStatus.OK):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = unquote(urlsplit(self.path).path)
        if path == "/api/content":
            return self._json(load_content())
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
        m = re.match(r"^/media/(img|video|share)/(.+)$", path)
        if not m:
            return self.send_error(HTTPStatus.NOT_FOUND)
        f = safe_join(MEDIA[m.group(1)], m.group(2))
        if f is None or not f.is_file():
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
    a = ap.parse_args()
    global STATE
    if a.state:
        STATE = Path(a.state).resolve()
        print(f"board state: {STATE}")
    httpd = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    print(f"Grant tracker: http://127.0.0.1:{a.port}/  (Ctrl+C to stop)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
