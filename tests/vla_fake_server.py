"""A scripted protocol-v2 VLA server, for the tests only (never shipped).

tests/test_vla_link.py and tests/test_vla_backends.py drive the pilot-side
client (guardrail/vla_link.py) against it. It validates every request the way
the real server must (protocol v2 in docs/DESIGN-vla-pilot.md and the brief):
an unknown field or one that could carry a target's coordinates is refused
with ok=false, and so is a phrase outside the training vocabulary. Its own
copy of the vocabulary and field lists is deliberate: a client bug that
changed guardrail/vla_link's constants would otherwise be validated against
itself.

`respond(request) -> reply` scripts each infer answer. A reply is a dict (sent
as one JSON line), `Delay(seconds, reply)`, `CLOSE` (drop the connection
without answering) or raw bytes (sent as they are, for malformed lines).
`step_reply` builds a well-formed answer from a step the way the real server
decodes one: quantize to 99 bins with int truncation, then dequantize.
"""
from __future__ import annotations

import base64
import hashlib
import json
import math
import select
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

VOCAB = {"straight ahead ", "forward-right ", "forward-left ", "to your right ",
         "to your left ", "to your right rear ", "to your left rear ", ""}
INFER_KEYS = {"v", "op", "seq", "t_capture", "pose", "front_jpg_b64", "down_jpg_b64",
              "phrase", "hint_source", "object", "csp_text", "csp_mode"}
COORD_KEYS = {"target", "goal", "object_position", "xy", "lat", "lon"}
RANGES = {"fwd_m": (0.0, 5.0), "down_m": (-5.0, 5.0), "dpsi_rad": (-1.1, 1.1)}
MODEL = {"id": "aerialvla-lora/aero_vla@fake", "base": "openvla-7b@47a0ec7f",
         "weights_sha256": "0" * 64, "precision": "nf4", "attn": "eager",
         "merged": False}


@dataclass
class Delay:
    seconds: float
    reply: Any


CLOSE = object()


def quantize(v: float, lo: float, hi: float) -> int:
    return max(0, min(98, int((v - lo) / (hi - lo) * 98)))


def dequantize(b: int, lo: float, hi: float) -> float:
    return b / 98 * (hi - lo) + lo


def _common(req: dict) -> dict:
    img = base64.b64decode(req["front_jpg_b64"])
    if req.get("down_jpg_b64"):
        img += base64.b64decode(req["down_jpg_b64"])
    prompt = (f"<image>\n{req.get('csp_text') or ''}Fly {req['phrase']}and find the "
              f"target. {req['object']}\nAction: ")
    return {"v": 2, "seq": req["seq"], "ok": True, "error": None,
            "latency_s": 0.01, "t_capture": req["t_capture"], "pose": req["pose"],
            "prompt_sha256": hashlib.sha256(prompt.encode()).hexdigest(),
            "image_sha256": hashlib.sha256(img).hexdigest(), "model": dict(MODEL)}


def step_reply(req: dict, fwd: float, down: float, dpsi: float, land: bool = False) -> dict:
    bins = [quantize(fwd, *RANGES["fwd_m"]), quantize(down, *RANGES["down_m"]),
            quantize(dpsi, *RANGES["dpsi_rad"])]
    step = {k: dequantize(b, *RANGES[k]) for k, b in zip(RANGES, bins)}
    out = _common(req)
    out.update({"raw_text": " ".join(f"{b:02d}" for b in bins) + (" LAND" if land else ""),
                "bins": bins, "step": step, "land": land, "parse_ok": True})
    return out


def land_reply(req: dict) -> dict:
    return step_reply(req, 0.0, 0.0, 0.0, land=True)


def parse_fail_reply(req: dict, raw_text: str = "00 49 494 49") -> dict:
    out = _common(req)
    out.update({"raw_text": raw_text, "bins": None, "step": None, "land": False,
                "parse_ok": False})
    return out


def error_reply(req: dict, error: str) -> dict:
    return {"v": 2, "seq": req.get("seq"), "ok": False, "error": error}


def scripted_step(pose: dict, phrase: str, goal, goal_up: float):
    """What a perfect AerialVLA would answer from a capture at `pose`: None
    (LAND) on the "you have arrived" phrase, else the raw step toward the goal
    under upstream's semantics (rotate only at |dpsi| >= 0.25 rad). The goal
    is the TEST's knowledge, standing in for what the model sees in the
    image; it never travels in a request."""
    if phrase == "":
        return None
    dx, dy = goal[0] - pose["x"], goal[1] - pose["y"]
    b = math.atan2(dy, dx) - pose["yaw_rad"]
    b = (b + math.pi) % (2 * math.pi) - math.pi
    dpsi = max(-1.1, min(1.1, b))
    fwd = 0.0 if abs(dpsi) >= 0.25 else min(math.hypot(dx, dy), 5.0)
    down = max(-5.0, min(5.0, pose["up"] - goal_up))
    return fwd, down, dpsi


def goto_responder(goal, goal_up: float):
    """`respond` for a FakeVLAServer that answers like scripted_step."""
    def respond(req):
        s = scripted_step(req["pose"], req["phrase"], goal, goal_up)
        return land_reply(req) if s is None else step_reply(req, *s)
    return respond


def _refusal(req: dict) -> str | None:
    """Why the real server would refuse this infer request, or None."""
    coord = sorted(COORD_KEYS & set(req))
    if coord:
        return f"refused field {coord[0]}"
    unknown = sorted(set(req) - INFER_KEYS)
    if unknown:
        return f"unknown field {unknown[0]}"
    missing = sorted(INFER_KEYS - set(req))
    if missing:
        return f"missing field {missing[0]}"
    if req["phrase"] not in VOCAB:
        return "phrase not in vocabulary"
    if req["hint_source"] not in ("goal", "detector", "none"):
        return "bad hint_source"
    if req["csp_mode"] not in ("off", "csp", "neutral"):
        return "bad csp_mode"
    if set(req["pose"]) != {"x", "y", "up", "yaw_rad"} or not all(
            isinstance(v, (int, float)) and math.isfinite(v) for v in req["pose"].values()):
        return "bad pose"
    if type(req["seq"]) is not int:
        return "bad seq"
    return None


class FakeVLAServer:
    """Listens on 127.0.0.1 (a free port); one thread per connection."""

    def __init__(self, respond: Callable[[dict], Any], model: dict | None = None):
        self.respond = respond
        self.model = dict(model or MODEL)
        self.requests: list[dict] = []     # every infer request received
        self.refused: list[str] = []       # why the server refused any
        self.hellos = 0
        self.connections = 0
        self.overlaps = 0                  # a request arrived before the last reply
        self._lsock: socket.socket | None = None
        self._threads: list[threading.Thread] = []
        self._socks: list[socket.socket] = []
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def start(self) -> tuple[str, int]:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("127.0.0.1", 0))
        s.listen(8)
        s.settimeout(0.1)
        self._lsock = s
        t = threading.Thread(target=self._accept_loop, daemon=True)
        t.start()
        self._threads.append(t)
        return s.getsockname()

    @property
    def port(self) -> int:
        return self._lsock.getsockname()[1]

    def stop(self) -> None:
        self._stop.set()
        if self._lsock is not None:
            self._lsock.close()
        with self._lock:
            for c in self._socks:
                try:
                    c.close()
                except OSError:
                    pass
        for t in self._threads:
            t.join(timeout=2.0)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                c, _ = self._lsock.accept()
            except (socket.timeout, OSError):
                continue
            with self._lock:
                self.connections += 1
                self._socks.append(c)
            t = threading.Thread(target=self._serve, args=(c,), daemon=True)
            t.start()
            self._threads.append(t)

    def _send(self, c: socket.socket, obj) -> None:
        data = obj if isinstance(obj, bytes) else json.dumps(obj).encode() + b"\n"
        c.sendall(data)

    def _serve(self, c: socket.socket) -> None:
        buf = b""
        c.settimeout(0.2)
        try:
            while not self._stop.is_set():
                if b"\n" not in buf:
                    try:
                        chunk = c.recv(1 << 20)
                    except socket.timeout:
                        continue
                    if not chunk:
                        return
                    buf += chunk
                    continue
                line, _, buf = buf.partition(b"\n")
                req = json.loads(line)
                op = req.get("op")
                if op == "hello":
                    self.hellos += 1
                    self._send(c, {"v": 2, "ok": True, "model": self.model})
                    continue
                if op == "shutdown":
                    self._send(c, {"v": 2, "ok": True})
                    return
                why = _refusal(req)
                if why is not None:
                    self.refused.append(why)
                    self._send(c, error_reply(req, why))
                    continue
                self.requests.append(req)
                reply = self.respond(req)
                if isinstance(reply, Delay):
                    time.sleep(reply.seconds)
                    reply = reply.reply
                # One outstanding request: nothing more may have arrived
                # before this reply goes out.
                if buf:
                    self.overlaps += 1
                elif select.select([c], [], [], 0)[0]:
                    peek = c.recv(1 << 20)
                    if not peek:
                        return          # the client gave up and closed
                    self.overlaps += 1
                    buf += peek
                if reply is CLOSE:
                    return
                self._send(c, reply)
        except (OSError, ValueError):
            # ValueError: select() on a socket stop() already closed.
            return
        finally:
            try:
                c.close()
            except OSError:
                pass
