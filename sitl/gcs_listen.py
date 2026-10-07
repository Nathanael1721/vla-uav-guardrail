"""
Stand in for Mission Planner on its UDP port, and check that the operator's
view actually carried the flight.

    python sitl/gcs_listen.py listen --seconds 170 --out gcs.json
    python sitl/gcs_listen.py check gcs.json demo/out/<tag>/events.jsonl

WHY. The 2026-10-06 runs claimed the MAVLink router "fans SITL out to Mission
Planner" on the strength of a listener that counted 1651 packets on Windows
UDP 14550. A review of the capture found 1393 of them were PARAM_VALUE, 197
FTP and 33 heartbeats, no position or attitude at all, and nothing after 32 s:
the stream to the Windows port had stopped before the mission began. A packet
count cannot tell those apart. So `listen` records WHEN each message type
arrived, and `check` passes only if GLOBAL_POSITION_INT and ATTITUDE (what
Mission Planner draws the aircraft from) arrived throughout the mission window
that the run's own events.jsonl records, with no gap longer than `--max-gap-s`.

`listen` only receives; it never sends, so it cannot become a second ground
station that changes stream rates on the shared link. It runs on any Python
(no pymavlink): the MAVLink v1/v2 framing is parsed here.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

# MAVLink common message ids this tool names.
MSG_NAMES = {0: "HEARTBEAT", 1: "SYS_STATUS", 22: "PARAM_VALUE", 24: "GPS_RAW_INT",
             30: "ATTITUDE", 32: "LOCAL_POSITION_NED", 33: "GLOBAL_POSITION_INT",
             74: "VFR_HUD", 77: "COMMAND_ACK", 110: "FILE_TRANSFER_PROTOCOL",
             111: "TIMESYNC", 148: "AUTOPILOT_VERSION", 253: "STATUSTEXT"}
REQUIRED = ("GLOBAL_POSITION_INT", "ATTITUDE")
_IDS = {v: k for k, v in MSG_NAMES.items()}


def parse_datagram(data: bytes) -> list[tuple[int, int]]:
    """[(sysid, msgid), ...] for every MAVLink v1 / v2 frame in a datagram.
    Stops at the first byte that is not a frame start (a router sends whole
    frames, so anything else is not MAVLink)."""
    out, i = [], 0
    while i < len(data):
        if data[i] == 0xFD and i + 10 <= len(data):            # v2
            plen, incompat = data[i + 1], data[i + 2]
            msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
            out.append((data[i + 5], msgid))
            i += 12 + plen + (13 if incompat & 1 else 0)
        elif data[i] == 0xFE and i + 6 <= len(data):           # v1
            out.append((data[i + 3], data[i + 5]))
            i += 8 + data[i + 1]
        else:
            break
    return out


def listen(port: int, seconds: float, bind: str = "0.0.0.0") -> dict:
    """Receive for `seconds`; the arrival wall time of every frame by type."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((bind, port))
    s.settimeout(0.5)
    t_end = time.time() + seconds
    arrivals: dict[str, list[float]] = {}
    counts: Counter = Counter()
    sources: Counter = Counter()
    packets = 0
    while time.time() < t_end:
        try:
            data, addr = s.recvfrom(65535)
        except socket.timeout:
            continue
        now = time.time()
        packets += 1
        sources[f"{addr[0]}:{addr[1]}"] += 1
        for sysid, msgid in parse_datagram(data):
            name = MSG_NAMES.get(msgid, str(msgid))
            counts[name] += 1
            if name in REQUIRED or name == "HEARTBEAT":
                arrivals.setdefault(name, []).append(round(now, 3))
    s.close()
    return {"port": port, "listened_s": seconds, "packets": packets,
            "sources": dict(sources), "counts": dict(counts.most_common()),
            "arrivals": arrivals}


def _wall(iso: str) -> float:
    return datetime.fromisoformat(iso).timestamp()


def mission_window(events: list[dict]) -> tuple[float, float] | None:
    """(start, end) wall seconds of the mission, from the run's own events."""
    start = next((e for e in events if e.get("kind") == "mission_start"), None)
    end = next((e for e in events if e.get("kind") == "mission_end"), None)
    if not start or not end or not start.get("wall") or not end.get("wall"):
        return None
    return _wall(start["wall"]), _wall(end["wall"])


def check(capture: dict, window: tuple[float, float] | None,
          required: tuple[str, ...] = REQUIRED,
          max_gap_s: float = 2.0) -> tuple[bool, list[str]]:
    """Did the GCS port carry the flight? `(ok, findings)`.

    For each required message type, the largest gap between arrivals across
    the window (counting the window's edges) must be at most `max_gap_s`.
    """
    if window is None:
        return False, ["no mission window: events.jsonl lacks mission_start "
                       "or mission_end"]
    t0, t1 = window
    ok, out = True, []
    for name in required:
        ts = sorted(t for t in capture.get("arrivals", {}).get(name, [])
                    if t0 <= t <= t1)
        if not ts:
            ok = False
            out.append(f"{name}: none received during the mission "
                       f"({t1 - t0:.1f} s)")
            continue
        edges = [t0] + ts + [t1]
        gap = max(b - a for a, b in zip(edges, edges[1:]))
        rate = len(ts) / max(t1 - t0, 1e-9)
        good = gap <= max_gap_s
        ok &= good
        out.append(f"{name}: {len(ts)} in {t1 - t0:.1f} s ({rate:.1f} Hz), "
                   f"largest gap {gap:.1f} s" + ("" if good else
                                                 f" > {max_gap_s:g} s"))
    return ok, out


def _jsonl(path: Path) -> list[dict]:
    return [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines()
            if l.strip()]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="gcs_listen.py")
    sub = ap.add_subparsers(dest="cmd", required=True)
    li = sub.add_parser("listen")
    li.add_argument("--port", type=int, default=14550)
    li.add_argument("--seconds", type=float, default=170.0)
    li.add_argument("--out", required=True)
    ch = sub.add_parser("check")
    ch.add_argument("capture")
    ch.add_argument("events")
    ch.add_argument("--max-gap-s", type=float, default=2.0)
    ch.add_argument("--clock-offset-s", type=float, default=0.0,
                    help="seconds the capturing host's clock is AHEAD of the "
                         "clock that wrote events.jsonl (WSL vs Windows)")
    args = ap.parse_args(argv)
    if args.cmd == "listen":
        cap = listen(args.port, args.seconds)
        Path(args.out).write_text(json.dumps(cap, indent=1), encoding="utf-8")
        print(json.dumps({k: cap[k] for k in ("packets", "sources", "counts")},
                         indent=1))
        return 0
    cap = json.loads(Path(args.capture).read_text(encoding="utf-8"))
    win = mission_window(_jsonl(Path(args.events)))
    if win is not None:
        win = (win[0] + args.clock_offset_s, win[1] + args.clock_offset_s)
    ok, findings = check(cap, win, max_gap_s=args.max_gap_s)
    print(("PASS" if ok else "FAIL") + ": the GCS port "
          + ("carried" if ok else "did not carry") + " the flight")
    for f in findings:
        print(f"  - {f}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
