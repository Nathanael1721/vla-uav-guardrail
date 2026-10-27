"""
Latency and native outputs of a running foreign-VLA server
(demo/vla_server_foreign.py), over its socket, on a few stored frames.

    # the server bench's own inputs: the two committed protocol mosaics, their
    # front tiles (Pillow; vla-real or dronecats env)
    python tools/bench_foreign_vla.py --server 127.0.0.1:8791 --n 30
    # a flight's sent frames instead (no Pillow: the JPEG bytes as they flew)
    python tools/bench_foreign_vla.py --server 127.0.0.1:8791 --n 30 \\
        --jpg-dir demo/out/<tag>/vla_frames --out bench_dronecats.json

What it measures, per request after --warmup untimed ones: the round trip
on this side, the server's model time (latency_s) and its whole server time
(timing.server_s); and what the model said, in its own terms - the native
kind (DroneCATS go / rotate / think / finished, RaceVLA "velocity",
"malformed" for an answer its reader refused, "refused" for a request the
server turned down) - and the step text the mapping made of it. The JSON
also keeps every native answer and mapping, so a histogram can be re-cut.

The frames: --frames (default deploy/evidence/vla_protocol_frames, the
AerialVLA server bench's two 224x448 mosaics, of which the front tile is
sent - vla_server_v2.bench_requests) or --jpg-dir (every *.jpg, sent as the
front frame as it is). Each request names --object ("a red car") and the
phrase "straight ahead " from a "detector" hint, which a foreign server
ignores; t_capture advances 1 s per request, inside the server's history
gap, so a DroneCATS server sees one episode. It speaks protocol v2 with
vla_server_v2's blocking client; it imports the codec as the foreign server
does (without guardrail/__init__ where that cannot import).
"""
from __future__ import annotations

import argparse
import base64
import json
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import vla_server_foreign as F                                       # noqa: E402

S = F.S
SCHEMA = "vlaguard.foreign_vla_bench/1"
DEFAULT_FRAMES = ROOT / "deploy" / "evidence" / "vla_protocol_frames"


def native_kind(resp: dict) -> str:
    """One answer's native kind: the model's action word (DroneCATS),
    "velocity" (RaceVLA's 7-vector), "malformed" (its reader refused it) or
    "refused" (an ok=false answer)."""
    if not isinstance(resp, dict) or not resp.get("ok"):
        return "refused"
    nat = resp.get("native") if isinstance(resp.get("native"), dict) else {}
    if not nat.get("ok"):
        return "malformed"
    act = (nat.get("action") or {}).get("action")
    if isinstance(act, str):
        return act
    return "velocity" if isinstance(act, list) else "unknown"


def jpg_requests(folder: Path, n: int, object_text: str) -> list[dict]:
    """n infer requests cycling over folder/*.jpg (sorted), each JPEG the
    front frame as it is, no down frame."""
    files = sorted(Path(folder).glob("*.jpg"))
    if not files:
        raise SystemExit(f"no *.jpg in {folder}")
    frames = [(f.name, base64.b64encode(f.read_bytes()).decode("ascii")) for f in files]
    out = []
    for k in range(n):
        name, b64 = frames[k % len(frames)]
        out.append({"v": 2, "op": "infer", "seq": k + 1, "t_capture": float(k),
                    "pose": {"x": 0.0, "y": 0.0, "up": 10.0, "yaw_rad": 0.0},
                    "front_jpg_b64": b64, "down_jpg_b64": None, "phrase": "straight ahead ",
                    "hint_source": "detector", "object": object_text, "csp_text": None,
                    "csp_mode": "off", "_frame": name})
    return out


def mosaic_requests(frames: Path, n: int, object_text: str) -> list[dict]:
    """The AerialVLA server bench's requests (vla_server_v2.bench_requests:
    the front and down tiles of the committed mosaics), relabelled with a
    detector hint and the object words."""
    reqs = S.bench_requests(Path(frames), n, object_phrase=object_text)
    for r in reqs:
        r.update({"phrase": "straight ahead ", "hint_source": "detector"})
    return reqs


def run(cli, reqs: list[dict], warmup: int) -> list[dict]:
    rows = []
    for k, r in enumerate(reqs):
        req = {kk: v for kk, v in r.items() if not kk.startswith("_")}
        t0 = time.perf_counter()
        resp = cli.call(req)
        rtt = time.perf_counter() - t0
        if k < warmup:
            continue
        rows.append({"frame": r.get("_frame"), "rtt_s": rtt, "ok": resp.get("ok"),
                     "error": resp.get("error"), "kind": native_kind(resp),
                     "raw_text": resp.get("raw_text"), "parse_ok": resp.get("parse_ok"),
                     "latency_s": resp.get("latency_s"),
                     "server_s": (resp.get("timing") or {}).get("server_s"),
                     "native": resp.get("native"), "mapping": resp.get("mapping")})
    return rows


def summarize(rows: list[dict]) -> dict:
    ok = [r for r in rows if r["ok"]]
    return {"n": len(rows),
            "roundtrip": S._stats([r["rtt_s"] for r in rows]),
            "server_latency": S._stats([r["latency_s"] for r in ok
                                        if isinstance(r["latency_s"], (int, float))]),
            "server_time": S._stats([r["server_s"] for r in ok
                                     if isinstance(r["server_s"], (int, float))]),
            "native_kinds": dict(Counter(r["kind"] for r in rows)),
            "step_texts": dict(Counter(r["raw_text"] for r in ok if r["parse_ok"])),
            "parse_fail": sum(1 for r in ok if not r["parse_ok"]),
            "refused": len(rows) - len(ok),
            "errors": sorted({str(r["error"]) for r in rows if not r["ok"]})}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="latency and native outputs of a foreign-VLA server")
    ap.add_argument("--server", required=True, metavar="HOST:PORT")
    ap.add_argument("--n", type=int, default=30, help="timed requests (default 30)")
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--frames", default=str(DEFAULT_FRAMES),
                    help="a directory of protocol mosaics + frames.json (Pillow)")
    ap.add_argument("--jpg-dir", default=None, help="send these *.jpg as the front frames")
    ap.add_argument("--object", default="a red car")
    ap.add_argument("--timeout-s", type=float, default=120.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--shutdown", action="store_true", help="send shutdown at the end")
    args = ap.parse_args(argv)
    if args.n <= 0 or args.warmup < 0:
        ap.error("--n must be positive and --warmup not negative")
    host, _, port = args.server.rpartition(":")
    if not host or not port.isdigit():
        ap.error(f"--server {args.server!r} is not HOST:PORT")
    total = args.warmup + args.n
    reqs = (jpg_requests(Path(args.jpg_dir), total, args.object) if args.jpg_dir
            else mosaic_requests(Path(args.frames), total, args.object))
    cli = S.V2Client(host, int(port), timeout=args.timeout_s)
    try:
        hello = cli.call({"v": 2, "op": "hello"})
        rows = run(cli, reqs, args.warmup)
        if args.shutdown:
            cli.call({"v": 2, "op": "shutdown"})
    finally:
        cli.close()
    model = dict(hello.get("model") or {})
    model.pop("health", None)
    res = {"schema": SCHEMA, "created_utc": datetime.now(timezone.utc).strftime(
               "%Y-%m-%dT%H:%M:%SZ"),
           "server": args.server, "identity": model, "warmup": args.warmup,
           "frames": args.jpg_dir or args.frames, "object": args.object,
           **summarize(rows), "rows": rows,
           "command": "python tools/bench_foreign_vla.py " + " ".join(
               argv if argv is not None else sys.argv[1:])}
    lat = res["server_latency"]
    print(f"[bench-foreign] {model.get('id')}: model p50 {lat.get('p50_ms')} ms p95 "
          f"{lat.get('p95_ms')} ms, round trip p50 {res['roundtrip'].get('p50_ms')} ms "
          f"(n={res['n']}); native {res['native_kinds']}; steps {res['step_texts']}", flush=True)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(res, indent=1, default=repr), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
