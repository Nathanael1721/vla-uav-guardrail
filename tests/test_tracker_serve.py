"""The grant tracker's local server: tracker/serve.py.

Run either way:
    pytest tests/test_tracker_serve.py -v
    python tests/test_tracker_serve.py

Every test starts its own server on a free port, with the site, data, state
and media folders redirected to a temporary directory, so the real board
(tracker/data/state.json) is never read or written here.

Pinned:
- the board state round-trips byte-for-byte, Indonesian and Chinese included,
  and leaves no temp file behind;
- bad requests are refused with JSON errors instead of dropping the connection;
- videos seek: HTTP Range answers 206 with the right bytes, 416 when
  unsatisfiable;
- nothing outside the three media folders and the site is reachable, and the
  working files under data/_* stay hidden however the path is spelled;
- /api/content merges every content file and turns a broken one into a
  load_error entry instead of a 500.
"""
import http.client
import json
import shutil
import sys
import tempfile
import threading
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tracker"))

import serve                                                    # noqa: E402

CLIP = bytes(range(256)) * 3 + bytes(range(232))                # 1000 bytes


def _fixture(tmp: Path) -> None:
    (tmp / "index.html").write_text("<!doctype html><title>t</title>", encoding="utf-8")
    d = tmp / "data"
    (d / "cards").mkdir(parents=True)
    cards = [{"id": "A-01", "status": "DONE", "i18n": {"en": {"title": "a"}, "id": {"title": "a"}, "zh": {"title": "甲"}}},
             {"id": "A-02", "status": "MISSING", "i18n": {"en": {"title": "b"}, "id": {"title": "b"}, "zh": {"title": "乙"}}}]
    (d / "cards" / "batch_00.json").write_text(json.dumps(cards[:1]), encoding="utf-8")
    (d / "cards" / "batch_01.json").write_text(json.dumps(cards[1:]), encoding="utf-8")
    (d / "_batches.json").write_text("[]", encoding="utf-8")
    (d / "content_overview.json").write_text(json.dumps({"project": {}}), encoding="utf-8")
    (d / "content_flow.json").write_text(json.dumps({"glossary": []}), encoding="utf-8")
    (d / "updates.json").write_text(json.dumps({"schema": 1, "entries": [{"seq": 1}]}), encoding="utf-8")
    for k in ("img", "video", "share"):
        (tmp / k).mkdir()
    (tmp / "video" / "clip.mp4").write_bytes(CLIP)
    (tmp / "img" / "a.png").write_bytes(b"\x89PNG....")
    (tmp / "secret.txt").write_text("secret", encoding="utf-8")
    (tmp / "VERSION").write_text("0.5.1\n", encoding="utf-8")


@contextmanager
def server():
    tmp = Path(tempfile.mkdtemp(prefix="tracker_test_"))
    saved = (serve.HERE, serve.DATA, serve.STATE, serve.MEDIA, serve.VERSION_FILE)
    try:
        _fixture(tmp)
        serve.HERE, serve.DATA, serve.STATE = tmp, tmp / "data", tmp / "data" / "state.json"
        serve.MEDIA = {"img": tmp / "img", "video": tmp / "video", "share": tmp / "share"}
        serve.VERSION_FILE = tmp / "VERSION"
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), serve.Handler)
        th = threading.Thread(target=httpd.serve_forever, daemon=True)
        th.start()
        try:
            yield httpd.server_address[1], tmp
        finally:
            httpd.shutdown()
            httpd.server_close()
    finally:
        serve.HERE, serve.DATA, serve.STATE, serve.MEDIA, serve.VERSION_FILE = saved
        shutil.rmtree(tmp, ignore_errors=True)


def req(port, method, path, body=None, headers=None):
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
    c.request(method, path, body=body, headers=headers or {})
    r = c.getresponse()
    data = r.read()
    c.close()
    return r.status, dict(r.getheaders()), data


def raw_post(port, path, length_header, body=b""):
    """A POST with a hand-written Content-Length (http.client would fix it)."""
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
    c.putrequest("POST", path)
    c.putheader("Content-Length", length_header)
    c.putheader("Content-Type", "application/json")
    c.endheaders()
    if body:
        c.send(body)
    r = c.getresponse()
    data = r.read()
    c.close()
    return r.status, data


# ------------------------------------------------------------------ state
def test_state_round_trips_with_indonesian_and_chinese_text():
    with server() as (port, tmp):
        st = {"cards": {"A-01": {"state": "doing", "note": "Catatan: sudah dicek · 已檢查", "history": [],
                                 "note_t": "2026-10-05T12:00:00Z", "note_log": [{"t": "x", "from": 0, "to": 10}]}},
              "events": [{"t": "2026-10-05T12:00:00Z", "kind": "reset"}], "updated": "2026-10-05T12:00:00Z"}
        s, _, b = req(port, "POST", "/api/state", json.dumps(st, ensure_ascii=False).encode("utf-8"),
                      {"Content-Type": "application/json"})
        assert s == 200 and json.loads(b) == {"ok": True}
        s, _, b = req(port, "GET", "/api/state")
        assert s == 200 and json.loads(b) == st
        assert not list((tmp / "data").glob(".state.*")), "a temp file was left behind"


def test_state_get_without_a_file_is_an_empty_board():
    with server() as (port, _):
        s, _, b = req(port, "GET", "/api/state")
        assert s == 200 and json.loads(b) == {"cards": {}, "updated": None}


def test_a_corrupt_state_file_is_a_json_500_not_a_crash():
    with server() as (port, tmp):
        (tmp / "data" / "state.json").write_text("{not json", encoding="utf-8")
        s, h, b = req(port, "GET", "/api/state")
        assert s == 500 and "error" in json.loads(b)


def test_bad_posts_are_refused_and_leave_the_state_unchanged():
    with server() as (port, tmp):
        good = json.dumps({"cards": {"A-01": {"state": "done"}}}).encode()
        assert req(port, "POST", "/api/state", good, {"Content-Type": "application/json"})[0] == 200
        before = (tmp / "data" / "state.json").read_bytes()
        cases = [b"{not json", json.dumps({"cards": []}).encode(), json.dumps([1, 2]).encode()]
        for body in cases:
            s, _, b = req(port, "POST", "/api/state", body, {"Content-Type": "application/json"})
            assert s == 400 and "error" in json.loads(b), body
        s, b = raw_post(port, "/api/state", "0")
        assert s == 400, s
        s, b = raw_post(port, "/api/state", str(serve.MAX_STATE_BYTES + 1))
        assert s == 400, s
        s, b = raw_post(port, "/api/state", "abc")          # not a number: must not drop the connection
        assert s == 400 and "error" in json.loads(b), (s, b)
        assert (tmp / "data" / "state.json").read_bytes() == before


def test_a_failed_write_cleans_up_and_answers_json():
    with server() as (port, tmp):
        real = serve.os.replace

        def boom(a, b):
            raise PermissionError("locked by OneDrive")
        serve.os.replace = boom
        try:
            s, _, b = req(port, "POST", "/api/state", b'{"cards": {}}', {"Content-Type": "application/json"})
        finally:
            serve.os.replace = real
        assert s == 500 and "error" in json.loads(b), (s, b)
        assert not list((tmp / "data").glob(".state.*")), "temp file left after a failed write"


def test_concurrent_posts_leave_valid_json():
    with server() as (port, tmp):
        def post(i):
            req(port, "POST", "/api/state", json.dumps({"cards": {f"A-{i:02d}": {"state": "todo"}}}).encode(),
                {"Content-Type": "application/json"})
        ths = [threading.Thread(target=post, args=(i,)) for i in range(10)]
        [t.start() for t in ths]
        [t.join() for t in ths]
        d = json.loads((tmp / "data" / "state.json").read_text(encoding="utf-8"))
        assert len(d["cards"]) == 1


# ------------------------------------------------------------------ range
def test_a_full_media_request_is_200_with_accept_ranges():
    with server() as (port, _):
        s, h, b = req(port, "GET", "/media/video/clip.mp4")
        assert s == 200 and b == CLIP and h.get("Accept-Ranges") == "bytes"
        assert "max-age" in h.get("Cache-Control", "")


def test_range_requests_return_exactly_the_bytes_asked_for():
    with server() as (port, _):
        s, h, b = req(port, "GET", "/media/video/clip.mp4", headers={"Range": "bytes=0-99"})
        assert s == 206 and b == CLIP[:100] and h["Content-Range"] == "bytes 0-99/1000"
        s, h, b = req(port, "GET", "/media/video/clip.mp4", headers={"Range": "bytes=900-"})
        assert s == 206 and b == CLIP[900:]
        s, h, b = req(port, "GET", "/media/video/clip.mp4", headers={"Range": "bytes=-100"})
        assert s == 206 and b == CLIP[-100:]
        s, h, b = req(port, "GET", "/media/video/clip.mp4", headers={"Range": "bytes=0-99999"})
        assert s == 206 and b == CLIP and h["Content-Range"] == "bytes 0-999/1000"


def test_unsatisfiable_ranges_are_416():
    with server() as (port, _):
        for r in ("bytes=2000-", "bytes=5-2"):
            s, h, _ = req(port, "GET", "/media/video/clip.mp4", headers={"Range": r})
            assert s == 416 and h.get("Content-Range") == "bytes */1000", (r, s)


def test_the_api_is_never_cached():
    with server() as (port, _):
        for p in ("/api/content", "/api/state"):
            _, h, _ = req(port, "GET", p)
            assert h.get("Cache-Control") == "no-store", p


# ------------------------------------------------------------------ paths
def test_nothing_outside_the_site_and_media_is_reachable():
    with server() as (port, _):
        for p in ("/media/img/../secret.txt", "/media/img/%2e%2e/secret.txt", "/media/img/..%2fsecret.txt",
                  "/media/foo/a.png", "/media/img/", "/media/video/missing.mp4"):
            s, _, b = req(port, "GET", p)
            assert s == 404 and b"secret" not in b, (p, s)


def test_working_files_stay_hidden_however_the_path_is_spelled():
    with server() as (port, _):
        for p in ("/data/_batches.json", "/data/%5Fbatches.json", "/data//_batches.json",
                  "/data/./_batches.json", "//data/_batches.json", "/data/cards/../_batches.json"):
            s, _, _ = req(port, "GET", p)
            assert s == 404, (p, s)
        assert req(port, "GET", "/")[0] == 200


# ------------------------------------------------------------------ content
def test_content_merges_every_file_in_order():
    with server() as (port, _):
        s, _, b = req(port, "GET", "/api/content")
        d = json.loads(b)
        assert s == 200 and [c["id"] for c in d["cards"]] == ["A-01", "A-02"]
        assert d["overview"] == {"project": {}} and d["flow"] == {"glossary": []}
        assert d["updates"]["entries"][0]["seq"] == 1
        assert d["project_version"] == "0.5.1"


def test_a_broken_content_file_is_a_load_error_not_a_500():
    with server() as (port, tmp):
        (tmp / "data" / "cards" / "batch_01.json").write_text("[{broken", encoding="utf-8")
        (tmp / "data" / "updates.json").unlink()
        s, _, b = req(port, "GET", "/api/content")
        d = json.loads(b)
        assert s == 200
        assert d["cards"][0]["id"] == "A-01" and "load_error" in d["cards"][1]
        assert "load_error" in d["updates"]


def test_the_real_content_has_148_trilingual_cards():
    # tracker/data/ is local-only (.gitignore), so a fresh clone has no cards.
    # Report that as a skip, never as a pass over zero cards.
    batches = sorted((ROOT / "tracker" / "data" / "cards").glob("batch_*.json"))
    if not batches:
        return "SKIP"
    cards = []
    for f in batches:
        cards += json.loads(f.read_text(encoding="utf-8"))
    assert len(cards) == 148 and len({c["id"] for c in cards}) == 148
    for c in cards:
        for lang in ("en", "id", "zh"):
            x = c["i18n"][lang]
            assert x.get("title") and x.get("summary"), (c["id"], lang)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == "SKIP":
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
