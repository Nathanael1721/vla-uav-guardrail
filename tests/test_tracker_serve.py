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
- nothing outside the media folders and the site is reachable, and the
  working files under data/_* stay hidden however the path is spelled;
- the recorded runs (demo/out, /media/run/) seek like the other videos, and
  only media files are served from there: the flight logs, audit records and
  replay bundles beside them are not;
- /api/content merges every content file and turns a broken one into a
  load_error entry instead of a 500;
- (2026-10-19) a request another web page can make the browser send - a
  text/plain POST, a foreign Origin, a DNS-rebound Host - is a JSON 403 and
  leaves the board unchanged; the board's own same-origin JSON POST works.
- (2026-10-21) the Dashboard's two endpoints, /api/dashboard and /api/live,
  pass the same refusal (403 on a foreign Host or Origin), are GET only (a
  POST is a 404 that changes nothing), and /api/live returns whitelisted
  fields only: no log content, no log path, no environment value; a missing
  dashboard.json is reported, not a 500; the background rebuild is deferred
  while a flight's heartbeat is live; /api/content and /api/dashboard answer
  304 to their own ETag; no GPU tool means no GPU row.
- (review of 8035d0a) a flight whose process is alive between beats still
  defers the rebuild and raises the overlap flag (P1); nvidia-smi runs once,
  without the process list, at most every 30 s during a flight (P9); the
  page never shows a dead server's data as current nor a failed live read
  as "nothing is running" (P3, under node; skipped without it).
"""
import http.client
import json
import os
import shutil
import sys
import tempfile
import threading
import time
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
    for k in ("img", "video", "share", "run"):
        (tmp / k).mkdir()
    (tmp / "video" / "clip.mp4").write_bytes(CLIP)
    (tmp / "run" / "r1").mkdir()
    (tmp / "run" / "r1" / "r1_demo.mp4").write_bytes(CLIP)
    (tmp / "run" / "r1" / "still.JPG").write_bytes(b"\xff\xd8....")
    (tmp / "run" / "r1" / "flight_log.jsonl").write_text('{"tick": 0}\n', encoding="utf-8")
    (tmp / "run" / "r1" / "kpi.json").write_text("{}", encoding="utf-8")
    (tmp / "img" / "a.png").write_bytes(b"\x89PNG....")
    (tmp / "secret.txt").write_text("secret", encoding="utf-8")
    (tmp / "VERSION").write_text("0.5.1\n", encoding="utf-8")
    (d / "dashboard.json").write_text(json.dumps(
        {"meta": {"schema": 1, "generated": "2026-10-21T00:00:00+00:00"}, "kpis": [], "flights": []}),
        encoding="utf-8")
    (tmp / "live").mkdir()


@contextmanager
def server():
    tmp = Path(tempfile.mkdtemp(prefix="tracker_test_"))
    saved = (serve.HERE, serve.DATA, serve.STATE, serve.MEDIA, serve.VERSION_FILE,
             serve.BUILD_ROOT, serve.LIVE_DIR, serve.AUTO_REBUILD)
    try:
        _fixture(tmp)
        serve.HERE, serve.DATA, serve.STATE = tmp, tmp / "data", tmp / "data" / "state.json"
        serve.MEDIA = {"img": tmp / "img", "video": tmp / "video", "share": tmp / "share",
                       "run": tmp / "run"}
        serve.VERSION_FILE = tmp / "VERSION"
        # The dashboard builder reads this temp folder, the heartbeats come
        # from tmp/live, and no rebuild starts unless a test asks for one.
        serve.BUILD_ROOT, serve.LIVE_DIR, serve.AUTO_REBUILD = tmp, tmp / "live", False
        serve._rebuild.update(running=False, last_try=0.0, error=None)
        serve._gpu.update(t=0.0, value=None)
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), serve.Handler)
        th = threading.Thread(target=httpd.serve_forever, daemon=True)
        th.start()
        try:
            yield httpd.server_address[1], tmp
        finally:
            httpd.shutdown()
            httpd.server_close()
    finally:
        (serve.HERE, serve.DATA, serve.STATE, serve.MEDIA, serve.VERSION_FILE,
         serve.BUILD_ROOT, serve.LIVE_DIR, serve.AUTO_REBUILD) = saved
        t_end = time.time() + 20                 # a test's rebuild thread may still be writing
        while serve._rebuild["running"] and time.time() < t_end:
            time.sleep(0.05)
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


def _with_host(port, method, path, host, body=None, headers=None):
    """A request with a hand-written Host (http.client writes its own)."""
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
    c.putrequest(method, path, skip_host=True)
    c.putheader("Host", host)
    for k, v in (headers or {}).items():
        c.putheader(k, v)
    if body is not None:
        c.putheader("Content-Length", str(len(body)))
    c.endheaders()
    if body is not None:
        c.send(body)
    r = c.getresponse()
    data = r.read()
    c.close()
    return r.status, data


def test_another_web_page_cannot_overwrite_or_read_the_board():
    """REVIEW 2026-10-19 (harness-tools/tracker-state-csrf). Binding 127.0.0.1
    does not stop the user's browser: a no-cors fetch or a form posting
    text/plain from any page replaced the board with {"cards": {}} (200),
    and a DNS-rebound name read it back. Each is now a JSON 403, checked
    before the body is read, and the state file is byte-identical."""
    with server() as (port, tmp):
        good = json.dumps({"cards": {"A-01": {"state": "done", "note": "keep"}}}).encode()
        assert req(port, "POST", "/api/state", good, {"Content-Type": "application/json"})[0] == 200
        state = tmp / "data" / "state.json"
        before = state.read_bytes()
        wipe = b'{"cards": {}}'
        attacks = [
            # fetch(..., {mode: "no-cors", body}) and <form enctype=text/plain>
            ({"Content-Type": "text/plain;charset=UTF-8",
              "Origin": "https://attacker.example"}, f"127.0.0.1:{port}"),
            ({"Content-Type": "text/plain"}, f"127.0.0.1:{port}"),
            # a JSON body from a foreign origin (needs a preflight a browser
            # would not get answered; refused here anyway)
            ({"Content-Type": "application/json",
              "Origin": "https://attacker.example"}, f"127.0.0.1:{port}"),
            ({"Content-Type": "application/json", "Origin": "null"}, f"127.0.0.1:{port}"),
            # DNS rebinding: same-origin to the browser, a foreign Host here
            ({"Content-Type": "application/json",
              "Origin": f"http://attacker.example:{port}"}, f"attacker.example:{port}"),
            ({"Content-Type": "application/json"}, f"attacker.example:{port}"),
        ]
        for headers, host in attacks:
            s, b = _with_host(port, "POST", "/api/state", host, wipe, headers)
            assert s == 403 and "refused" in json.loads(b)["error"], (headers, host, s, b)
            assert state.read_bytes() == before, (headers, host)
        for path in ("/api/state", "/api/content", "/", "/media/video/clip.mp4"):
            s, b = _with_host(port, "GET", path, f"attacker.example:{port}")
            assert s == 403 and b"A-01" not in b, (path, s)
        # the board's own requests still work: same origin, with or without a
        # charset, through either loopback name, and without an Origin (curl)
        for host, ctype in ((f"127.0.0.1:{port}", "application/json"),
                            (f"localhost:{port}", "application/json; charset=utf-8"),
                            (f"LOCALHOST:{port}", "application/json")):
            s, b = _with_host(port, "POST", "/api/state", host, good,
                              {"Content-Type": ctype, "Origin": f"http://{host}"})
            assert s == 200 and json.loads(b) == {"ok": True}, (host, s, b)
            s, b = _with_host(port, "GET", "/api/state", host, None,
                              {"Origin": f"http://{host}"})
            assert s == 200 and json.loads(b)["cards"]["A-01"]["note"] == "keep", (host, s)
        assert req(port, "POST", "/api/state", good, {"Content-Type": "application/json"})[0] == 200


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


def test_a_recorded_run_seeks_like_the_other_videos():
    with server() as (port, _):
        s, h, b = req(port, "GET", "/media/run/r1/r1_demo.mp4", headers={"Range": "bytes=100-199"})
        assert s == 206 and b == CLIP[100:200] and h["Content-Range"] == "bytes 100-199/1000"
        assert h.get("Content-Type") == "video/mp4"
        s, _, b = req(port, "GET", "/media/run/r1/still.JPG")
        assert s == 200 and b.startswith(b"\xff\xd8")


# ------------------------------------------------------------------ paths
def test_only_media_files_are_served_from_the_run_folder():
    with server() as (port, _):
        for p in ("/media/run/r1/flight_log.jsonl", "/media/run/r1/kpi.json", "/media/run/r1/",
                  "/media/run/../secret.txt", "/media/run/r1/%2e%2e/%2e%2e/secret.txt",
                  "/media/run/r1/missing.mp4"):
            s, _, b = req(port, "GET", p)
            assert s == 404 and b"secret" not in b and b"tick" not in b, (p, s)
        # The other folders keep serving any file in them, as before.
        assert req(port, "GET", "/media/video/clip.mp4")[0] == 200


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


# ------------------------------------------------------------------ dashboard + live (2026-10-21)
def _heartbeat(tmp, name, kind="flight", age=1.0, **extra):
    now = time.time()
    rec = {"schema": 1, "id": f"{kind}-{name}", "kind": kind, "name": name, "pid": os.getpid(),
           "cli": False, "started": now - 60, "heartbeat": now - age, "progress": 0.25,
           "detail": None, "status": "running"}
    rec.update(extra)
    (tmp / "live" / f"{kind}-{name}.json").write_text(json.dumps(rec), encoding="utf-8")


def test_the_dashboard_and_live_endpoints_refuse_a_foreign_host_or_origin():
    with server() as (port, tmp):
        _heartbeat(tmp, "pas_ap_x")
        for path in ("/api/dashboard", "/api/live"):
            for host, headers in ((f"attacker.example:{port}", {}),
                                  (f"127.0.0.1:{port}", {"Origin": "https://attacker.example"}),
                                  (f"127.0.0.1:{port}", {"Origin": "null"}),
                                  (f"attacker.example:{port}", {"Origin": f"http://attacker.example:{port}"})):
                s, b = _with_host(port, "GET", path, host, None, headers)
                assert s == 403 and "refused" in json.loads(b)["error"], (path, host, headers, s)
                assert b"pas_ap_x" not in b and b"schema" not in b, (path, b[:80])
            for host in (f"127.0.0.1:{port}", f"localhost:{port}"):
                s, b = _with_host(port, "GET", path, host, None, {"Origin": f"http://{host}"})
                assert s == 200, (path, host, s)


def test_the_dashboard_and_live_endpoints_are_get_only():
    with server() as (port, tmp):
        before = (tmp / "data" / "dashboard.json").read_bytes()
        for path in ("/api/dashboard", "/api/live"):
            s, _, _ = req(port, "POST", path, b'{"meta": {}}', {"Content-Type": "application/json"})
            assert s == 404, (path, s)
        assert (tmp / "data" / "dashboard.json").read_bytes() == before
        assert not (tmp / "data" / "state.json").exists(), "a POST to a dashboard endpoint wrote the board"


def test_live_returns_no_log_content_path_or_environment():
    with server() as (port, tmp):
        (tmp / "run" / "pas_ap_x").mkdir()
        (tmp / "run" / "pas_ap_x" / "flight_log.jsonl").write_text('{"tick": 0, "SECRETLOG": 1}\n', encoding="utf-8")
        _heartbeat(tmp, "pas_ap_x", log_dir=str(tmp / "run" / "pas_ap_x"),
                   env={"API_TOKEN": "hunter2"}, cmd="python fly.py --token hunter2", detail="d" * 900)
        _heartbeat(tmp, "review", kind="workflow", pid=None, cli=True)
        s, h, b = req(port, "GET", "/api/live")
        d = json.loads(b)
        assert s == 200 and h.get("Cache-Control") == "no-store"
        job = next(j for j in d["jobs"] if j["name"] == "pas_ap_x")
        assert job["run_tag"] == "pas_ap_x" and job["state"] == "running" and len(job["detail"]) == 200
        for leak in (b"hunter2", b"API_TOKEN", b"SECRETLOG", b"fly.py", str(tmp).encode(),
                     json.dumps(str(tmp)).strip('"').encode(), b"log_dir"):
            assert leak not in b, leak
        assert d["overlap"] is True, "a flight and a workflow ran together and no banner flag"
        assert set(d) >= {"jobs", "done", "overlap", "gpu", "now"}


def test_the_dashboard_answers_with_its_file_a_stale_flag_and_an_etag():
    with server() as (port, tmp):
        s, h, b = req(port, "GET", "/api/dashboard")
        d = json.loads(b)
        assert s == 200 and d["meta"]["schema"] == 1 and h.get("Cache-Control") == "no-store"
        assert set(d["server"]) == {"stale", "stale_reasons", "rebuild", "rebuild_error"}
        assert d["server"]["rebuild"] in ("not_needed", "off")
        tag = h.get("ETag")
        assert tag
        s, h2, b2 = req(port, "GET", "/api/dashboard", headers={"If-None-Match": tag})
        assert s == 304 and b2 == b"", (s, b2[:60])
        s, h3, _ = req(port, "GET", "/api/content")
        assert s == 200 and h3.get("ETag")
        s, _, b4 = req(port, "GET", "/api/content", headers={"If-None-Match": h3["ETag"]})
        assert s == 304 and b4 == b""
        (tmp / "data" / "dashboard.json").unlink()
        s, _, b = req(port, "GET", "/api/dashboard")
        d = json.loads(b)
        assert s == 200 and d["missing"] is True and d["server"]["stale"] is True


def test_the_background_rebuild_waits_while_a_flight_runs():
    with server() as (port, tmp):
        serve.AUTO_REBUILD = True
        out = tmp / "data" / "dashboard.json"
        old = time.time() - 3600
        os.utime(out, (old, old))
        (tmp / "data" / "updates.json").write_text(json.dumps({"schema": 1, "entries": []}), encoding="utf-8")
        _heartbeat(tmp, "pas_ap_live")
        s, _, b = req(port, "GET", "/api/dashboard")
        d = json.loads(b)
        assert d["server"]["stale"] is True and d["server"]["rebuild"] == "deferred_flight", d["server"]
        assert out.stat().st_mtime == old, "a rebuild ran during a flight"
        (tmp / "live" / "flight-pas_ap_live.json").unlink()
        s, _, b = req(port, "GET", "/api/dashboard")
        assert json.loads(b)["server"]["rebuild"] == "started"
        t_end = time.time() + 20
        while serve._rebuild["running"] and time.time() < t_end:
            time.sleep(0.05)
        assert serve._rebuild["error"] is None, serve._rebuild["error"]
        rebuilt = json.loads(out.read_text(encoding="utf-8"))
        assert out.stat().st_mtime > old and rebuilt["meta"]["mode"] == "quick"
        s, _, b = req(port, "GET", "/api/dashboard")
        assert json.loads(b)["server"]["rebuild"] in ("not_needed", "too_recent")


def test_a_live_flight_between_beats_still_defers_the_rebuild():
    """Review of 8035d0a (P1): the flights beat only inside their mission
    loops, so a flight loading its scene, booting SITL or landing had a
    heartbeat older than STALE_S, read "stale", and the background rebuild
    started in the middle of it (~100 s of a 225 s run). A record whose
    process is alive (this test's own PID) defers it however old its beat."""
    with server() as (port, tmp):
        serve.AUTO_REBUILD = True
        out = tmp / "data" / "dashboard.json"
        old = time.time() - 3600
        os.utime(out, (old, old))
        (tmp / "data" / "updates.json").write_text(json.dumps({"schema": 1, "entries": []}), encoding="utf-8")
        _heartbeat(tmp, "pas_ap_takeoff", age=serve._tool("live_status").STALE_S + 45, phase="starting")
        _heartbeat(tmp, "review", kind="workflow", pid=None, cli=True)
        s, _, b = req(port, "GET", "/api/dashboard")
        d = json.loads(b)
        assert d["server"]["rebuild"] == "deferred_flight", d["server"]
        assert out.stat().st_mtime == old, "a rebuild ran while a flight was between beats"
        s, _, b = req(port, "GET", "/api/live")
        live = json.loads(b)
        job = next(j for j in live["jobs"] if j["name"] == "pas_ap_takeoff")
        assert job["state"] == "quiet" and job["phase"] == "starting", job
        assert live["flight_running"] is True and live["overlap"] is True, live


def test_an_unreadable_live_status_is_a_possible_flight_on_both_paths():
    """Re-review of d180e01 (P5). The rebuild deferral and the GPU throttle
    read an unreadable live status in opposite ways: _flight_running()
    returned False on any exception, so the background rebuild went ahead,
    while /api/live treated the same failure as a possible flight and
    throttled nvidia-smi. Both now treat it as a possible flight (defer when
    unsure, the reason the rebuild waits at all: review workflows halved
    OWL-ViT's rate), and the page says why the rebuild waits
    ("deferred_live_unreadable", not "a flight is running")."""
    seen = []
    real_snap, real_gpu = serve._live_snapshot, serve.gpu_snapshot

    def broken():
        raise PermissionError("live folder")

    def gpu(flight=False):
        seen.append(flight)
        return None
    with server() as (port, tmp):
        serve.AUTO_REBUILD = True
        out = tmp / "data" / "dashboard.json"
        old = time.time() - 3600
        os.utime(out, (old, old))
        (tmp / "data" / "updates.json").write_text(json.dumps({"schema": 1, "entries": []}), encoding="utf-8")
        serve._live_snapshot, serve.gpu_snapshot = broken, gpu
        try:
            s, _, b = req(port, "GET", "/api/dashboard")
            d = json.loads(b)
            assert d["server"]["rebuild"] == "deferred_live_unreadable", d["server"]
            assert not serve._rebuild["running"] and out.stat().st_mtime == old, \
                "a rebuild ran while the live status could not be read"
            s, _, b = req(port, "GET", "/api/live")
            live = json.loads(b)
            assert s == 200 and live["error"].startswith("live status unavailable"), live
            assert seen == [True], seen
        finally:
            serve._live_snapshot, serve.gpu_snapshot = real_snap, real_gpu
        # readable again, no flight: the rebuild goes ahead
        s, _, b = req(port, "GET", "/api/dashboard")
        assert json.loads(b)["server"]["rebuild"] == "started"
    i18n = (ROOT / "tracker" / "assets" / "i18n.js").read_text(encoding="utf-8")
    assert i18n.count("dash_rebuild_unreadable:") == 3
    assert '"deferred_live_unreadable"' in (ROOT / "tracker" / "assets" / "app.js").read_text(encoding="utf-8")


def test_gpu_polling_slows_down_while_a_flight_runs():
    """Review of 8035d0a (P9): /api/live spawned nvidia-smi twice every 5 s
    while the Dashboard was open, flight or not, although the rebuild was
    already deferred during flights for the same reason (review workflows
    halved OWL-ViT's rate). During a flight it is one spawn (no process
    list) at most every GPU_CACHE_FLIGHT_S."""
    calls = []

    class R:
        def __init__(self, out):
            self.stdout = out

    def fake_run(cmd, **kw):
        calls.append(cmd[1])
        return R("NVIDIA GeForce RTX 4090, 37, 9000, 24564\n" if cmd[1].startswith("--query-gpu")
                 else "1234, C:\\Users\\x\\python.exe, 8000\n")
    real_which, real_run = serve.shutil.which, serve.subprocess.run
    serve.shutil.which = lambda name: "nvidia-smi"
    serve.subprocess.run = fake_run
    try:
        serve._gpu.update(t=0.0, value=None, flight=False)
        g = serve.gpu_snapshot(flight=False)
        assert len(calls) == 2 and g["apps"] and g["apps"][0]["name"] == "python.exe"
        calls.clear()
        serve._gpu.update(t=0.0, value=None, flight=False)
        g = serve.gpu_snapshot(flight=True)
        assert len(calls) == 1 and calls[0].startswith("--query-gpu"), calls
        assert g["gpus"][0]["util_pct"] == 37 and g["apps"] is None and g["reduced_for_flight"] is True
        # within GPU_CACHE_FLIGHT_S of that reading, nothing is spawned
        serve._gpu["t"] = time.time() - (serve.GPU_CACHE_S + 1)
        assert serve.gpu_snapshot(flight=True) == g and len(calls) == 1, calls
        # the flight ends: the next reading is a full one again
        serve._gpu["t"] = time.time() - (serve.GPU_CACHE_S + 1)
        assert serve.gpu_snapshot(flight=False)["apps"] and len(calls) == 3, calls
    finally:
        serve.shutil.which, serve.subprocess.run = real_which, real_run
        serve._gpu.update(t=0.0, value=None, flight=False)


def test_no_gpu_tool_means_no_gpu_row():
    real = serve.shutil.which
    serve.shutil.which = lambda name: None
    try:
        serve._gpu.update(t=0.0, value={"gpus": ["stale"]})
        assert serve.gpu_snapshot() is None
    finally:
        serve.shutil.which = real
        serve._gpu.update(t=0.0, value=None)


# ------------------------------------------------------------------ the page (node)
def _js_function(src: str, name: str) -> str:
    """`function name(...) {...}` from app.js, by brace count (the bodies
    are template literals whose `${...}` braces balance)."""
    start = src.index(f"function {name}(")
    i, depth = src.index("{", start), 0
    for j in range(i, len(src)):
        depth += {"{": 1, "}": -1}.get(src[j], 0)
        if depth == 0:
            return src[start:j + 1]
    raise AssertionError(f"{name}: unbalanced braces")


def _page(scenarios: dict) -> dict:
    """Run app.js's dashHeader / liveHTML under node with the real UI
    strings (i18n.js) and helpers (esc, t, tf, ftime) on each scenario's
    S; None when node is absent."""
    import re
    import subprocess
    node = shutil.which("node")
    if node is None:
        return None
    app = (ROOT / "tracker" / "assets" / "app.js").read_text(encoding="utf-8")
    helpers = [m.group(0) for m in re.finditer(r"^\s*const (LOCALE|FINAL|esc|t|tf|ftime) = .*;$", app, re.M)]
    assert len(helpers) == 6, helpers
    js = "\n".join([
        "const window = {};", (ROOT / "tracker" / "assets" / "i18n.js").read_text(encoding="utf-8"),
        "const UI = window.UI; let S = {lang: 'en'};", *helpers,
        _js_function(app, "dashHeader"), _js_function(app, "liveHTML"),
        f"const SC = {json.dumps(scenarios)}; const out = {{}};",
        "for (const [k, v] of Object.entries(SC)) { S = Object.assign({lang: 'en'}, v.S);"
        " out[k] = v.fn === 'dashHeader' ? dashHeader(v.d) : liveHTML(); }",
        "console.log(JSON.stringify(out));"])
    tmp = Path(tempfile.mkdtemp(prefix="tracker_js_"))
    try:
        f = tmp / "page.js"
        f.write_text(js, encoding="utf-8")
        r = subprocess.run([node, str(f)], capture_output=True, text=True, encoding="utf-8", timeout=60)
        assert r.returncode == 0, r.stderr[-800:]
        return json.loads(r.stdout)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_the_page_never_shows_old_data_as_current_or_a_failed_read_as_idle():
    """Review of 8035d0a (P3). When /api/dashboard stopped answering, the page
    kept the last data AND its server block, so the header still said "Data
    current" in green; a failed /api/live read printed "could not be read"
    and then "Nothing is running." and "No GPU reading", asserting that no
    flight ran. Now a failed dashboard read shows a red "server unreachable,
    data from <generated>" badge (never "Data current", never the old
    server block's badges), and a failed live read shows only that it
    failed. Run with node (skipped without it)."""
    meta = {"generated": "2026-10-21T00:00:00+00:00", "version": "0.5.1"}
    fresh = {"meta": meta, "server": {"stale": False, "rebuild": "not_needed"}}
    deferred = {"meta": meta, "server": {"stale": True, "rebuild": "deferred_flight"}}
    job = {"id": "flight-x-1", "kind": "flight", "name": "pas_ap_x", "state": "quiet", "phase": "airborne",
           "age_s": 45.0, "started": "2026-10-22T01:00:00+08:00", "progress": None, "detail": "climbing"}
    out = _page({
        "dead_server": {"fn": "dashHeader", "d": fresh, "S": {"dashErr": True}},
        "dead_server_old_block": {"fn": "dashHeader", "d": deferred, "S": {"dashErr": True}},
        "live_server": {"fn": "dashHeader", "d": fresh, "S": {"dashErr": False}},
        "live_read_failed": {"fn": "liveHTML", "S": {"live": {"error": "unreachable", "jobs": []}}},
        "nothing_running": {"fn": "liveHTML", "S": {"live": {"jobs": [], "dir_ok": True, "gpu": None}}},
        "quiet_flight": {"fn": "liveHTML", "S": {"live": {"jobs": [job], "dir_ok": True, "gpu": None,
                                                          "flight_running": True}}},
    })
    if out is None:
        print("      SKIP: node is not installed")
        return "SKIP"
    assert "Server unreachable" in out["dead_server"] and "Data current" not in out["dead_server"], out["dead_server"]
    assert "badge bad" in out["dead_server"]
    for stale_word in ("Rebuild waits", "Data stale", "Data current"):
        assert stale_word not in out["dead_server_old_block"], (stale_word, out["dead_server_old_block"])
    assert "Data current" in out["live_server"] and "Server unreachable" not in out["live_server"]
    failed = out["live_read_failed"]
    assert "could not be read" in failed, failed
    for claim in ("Nothing is running", "No heartbeat folder", "No GPU reading"):
        assert claim not in failed, (claim, failed)
    assert "Nothing is running" in out["nothing_running"] and "No GPU reading" in out["nothing_running"]
    q = out["quiet_flight"]
    assert "alive, no beat" in q and "airborne" in q and "Nothing is running" not in q, q


def test_a_304_after_an_outage_clears_the_server_unreachable_badge():
    """Re-review of d180e01 (P1). loadDash returned on a 304 BEFORE it
    cleared S.dashErr. The server's ETag hashes the whole body, server block
    included, so a server that comes back with unchanged data answers the
    60 s poll (and the visibility reload) with 304: dashErr stayed true and
    the header kept the red "Server unreachable" badge over a live server
    until the data itself changed or the page was reloaded. Now a 304 is an
    answer like a 200: it clears dashErr and re-renders the page when the
    badge was up; a 304 with no outage before it does not re-render (the
    60 s poll must not repaint an unchanged page). Run with node (skipped
    without it)."""
    import subprocess
    node = shutil.which("node")
    if node is None:
        print("      SKIP: node is not installed")
        return "SKIP"
    app = (ROOT / "tracker" / "assets" / "app.js").read_text(encoding="utf-8")
    js = "\n".join([
        "let S = {content: {}, dashEtag: 'W/\"e1\"', dash: {meta: {}}, dashErr: false};",
        "let renders = 0; const render = () => { renders += 1; };",
        "const route = () => 'dashboard';",
        "const window = {scrollY: 0, scrollTo() {}};",
        "let answers = [];",
        "async function fetch(url, opts) { const a = answers.shift(); if (a === 'down') throw new Error('refused');"
        " return {status: a, ok: a === 200, headers: {get: () => 'W/\"e1\"'}, json: async () => ({meta: {}})}; }",
        "async " + _js_function(app, "loadDash"),
        "(async () => { const out = {};",
        "  answers = ['down']; await loadDash(false); out.after_outage = [S.dashErr, renders];",
        "  answers = [304]; await loadDash(false); out.after_304 = [S.dashErr, renders];",
        "  answers = [304]; await loadDash(false); out.second_304 = [S.dashErr, renders];",
        "  answers = ['down']; await loadDash(false, true); answers = [304]; await loadDash(false, true);",
        "  out.quiet_304 = [S.dashErr, renders];",
        "  console.log(JSON.stringify(out)); })();"])
    tmp = Path(tempfile.mkdtemp(prefix="tracker_js_"))
    try:
        f = tmp / "load.js"
        f.write_text(js, encoding="utf-8")
        r = subprocess.run([node, str(f)], capture_output=True, text=True, encoding="utf-8", timeout=60)
        assert r.returncode == 0, r.stderr[-800:]
        out = json.loads(r.stdout)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    assert out["after_outage"] == [True, 1], out
    assert out["after_304"] == [False, 2], f"a 304 after an outage left the badge up: {out}"
    assert out["second_304"] == [False, 2], f"a plain 304 repainted the page: {out}"
    assert out["quiet_304"] == [False, 2], f"a quiet load rendered or kept the error: {out}"


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
