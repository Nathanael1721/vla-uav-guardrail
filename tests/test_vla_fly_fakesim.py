"""demo/vla_fly.py's whole flight path, against a FAKE projectairsim module.

Run either way:
    pytest tests/test_vla_fly_fakesim.py -v
    python tests/test_vla_fly_fakesim.py

LAB-ONLY: the camera frames go through OpenCV (vla_mission.jpeg_from_msg)
and the server's scripted stand-in decodes them with Pillow, and no CI job
installs either, so there every test here SKIPs, saying why. On the lab
machine (vla-real, Python 3.10) it runs.

WHAT IS BEING CLAIMED

fly() itself - the part tests/test_vla_fly.py cannot reach - runs from the
server's start to its stop with a simulator stand-in: a point vehicle that
follows move_by_velocity, synthetic 256x256 camera messages on the VLA
topics (and 768x432 / 960x540 on the detector and Chase topics), and the
real demo/vla_server_v2.py answering with its scripted stand-in:

  * M1 takes off, transfers through the Shield to the mission start, faces
    the goal, flies the stand-in's steps on the goal phrase until its time,
    lands, writes every record (rows with authors, vla_steps.jsonl, the
    frames, the captures at 256x256, metrics/manifest/kpi, the video frames
    with the HUD), stops the server by op shutdown - and records the
    stand-in as a stub, which the dashboard rule never calls a VLA;
  * M2 asks only while a (fake) detector box is fresh, ends on the box's
    size, and scores against the parked car's truth;
  * a server killed mid-mission makes the tracker hover on link_lost, the
    mission end server_dead after --dead-timeout-s, the aircraft land, and
    the exit code 3.

Shown failing first: run before the fixes of 2026-10-22 it reported the
decision rate over the landing too (stand-in at ~2.5 Hz read as 1.5 Hz) -
test_m1_flies_lands_and_records_a_stand_in_as_a_stub's rate check.
"""
from __future__ import annotations

import asyncio
import contextlib
import io
import json
import math
import os
import socket
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "tests", ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

SKIP = "SKIP"


def _missing() -> str | None:
    try:
        import cv2                                                      # noqa: F401
        import PIL                                                      # noqa: F401
    except ImportError as exc:
        return f"{exc.name} is not installed (lab-only: OpenCV and Pillow)"
    return None


def _done():
    f = asyncio.get_event_loop().create_future()
    f.set_result(None)
    return f


class _Vehicle:
    def __init__(self, x=35.0, y=-20.0):
        self.x, self.y, self.up, self.yaw = x, y, 0.0, 0.0
        self.v = [0.0, 0.0, 0.0]
        self.cmd = (0.0, 0.0, 0.0, 0.0)
        self.t = time.time()
        self.lock = threading.Lock()

    def advance(self):
        with self.lock:
            now = time.time()
            dt = min(0.5, now - self.t)
            self.t = now
            vx, vy, vz_up, wz = self.cmd
            k = min(1.0, dt / 0.3)
            for i, c in enumerate((vx, vy, vz_up)):
                self.v[i] += k * (c - self.v[i])
            self.x += self.v[0] * dt
            self.y += self.v[1] * dt
            self.up = max(0.0, self.up + self.v[2] * dt)
            self.yaw = (self.yaw + wz * dt + math.pi) % (2 * math.pi) - math.pi


def fake_projectairsim(veh: _Vehicle) -> types.ModuleType:
    """The four names demo/vla_fly.py imports, with just enough behaviour."""
    import numpy as np

    class ProjectAirSimClient:
        def __init__(self):
            self.subs, self._stop = {}, False

        def connect(self):
            threading.Thread(target=self._cams, daemon=True).start()

        def disconnect(self):
            self._stop = True

        def subscribe(self, topic, cb):
            self.subs.setdefault(topic, []).append(cb)

        def _cams(self):
            sizes = {"front": (256, 256), "down": (256, 256), "det": (768, 432),
                     "chase": (960, 540)}
            k = 0
            while not self._stop:
                k += 1
                for topic, (w, h) in sizes.items():
                    for cb in self.subs.get(topic, []):
                        img = np.zeros((h, w, 3), dtype=np.uint8)
                        img[:, :, 0] = (k * 7) % 255
                        cb(topic, {"data": img.tobytes(), "width": w, "height": h,
                                   "encoding": "BGR", "time_stamp": int(time.time() * 1e9)})
                time.sleep(0.1)

    class World:
        def __init__(self, client, scene, delay_after_load_sec=2, sim_config_path=None):
            assert (Path(sim_config_path) / scene).is_file(), scene
            self.scene = scene

        def switch_streaming_view(self):
            pass

        def import_ned_trajectory(self, *a, **k):
            pass

    class Drone:
        def __init__(self, client, world, name):
            self.sensors = {"VLAFront": {"scene_camera": "front"},
                            "VLADown": {"scene_camera": "down"},
                            "FrontCamera": {"scene_camera": "det"},
                            "Chase": {"scene_camera": "chase"}}
            self.robot_info = {"collision_info": "coll"}

        def enable_api_control(self):
            pass

        disable_api_control = arm = disarm = enable_api_control

        def get_ground_truth_kinematics(self):
            veh.advance()
            h = veh.yaw / 2.0
            return {"time_stamp": int(time.time() * 1e9),
                    "pose": {"position": {"x": veh.x, "y": veh.y, "z": -veh.up},
                             "orientation": {"w": math.cos(h), "x": 0.0, "y": 0.0,
                                             "z": math.sin(h)}},
                    "twist": {"linear": {"x": veh.v[0], "y": veh.v[1], "z": -veh.v[2]}}}

        async def takeoff_async(self):
            veh.up = 3.0
            return _done()

        async def move_by_velocity_async(self, vx, vy, vz, duration=0.3, yaw_is_rate=False,
                                         yaw=0.0):
            veh.advance()
            veh.cmd = (vx, vy, -vz, yaw if yaw_is_rate else 0.0)
            return _done()

        async def rotate_to_yaw_async(self, yaw):
            veh.advance()
            veh.yaw = yaw
            return _done()

        async def land_async(self):
            veh.up, veh.cmd = 0.0, (0, 0, 0, 0)
            return _done()

    class EnvActor:
        def __init__(self, client, world, name):
            self.name = name

        def set_trajectory(self, name, to_loop=False):
            pass

    mod = types.ModuleType("projectairsim")
    mod.ProjectAirSimClient, mod.World, mod.Drone, mod.EnvActor = (
        ProjectAirSimClient, World, Drone, EnvActor)
    return mod


class _GrowingBox:
    """A Grounder stand-in (no OWL-ViT on the GPU): a box a little right of
    centre whose width grows 0.05 -> 0.15 of the frame in about 5 s."""

    def __init__(self, obs, query, **kw):
        self.t0, self.n = None, 0

    def start(self):
        self.t0 = time.time()

    def stop(self):
        pass

    def latest(self):
        now = time.time()
        self.n += 1
        w = (0.05 + 0.02 * (now - (self.t0 or now))) * 768
        return {"seq": self.n, "det": (430.0, 250.0, w, w / 2, 0.1, 768, 432, 0.6),
                "t_det": now - 0.2, "cap": {"t_capture": now - 0.25}}


def _port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _fly(argv, grounder=None, kill_after_s=None):
    """vla_fly.main(argv) with the fake simulator in sys.modules (restored
    after), the Dashboard heartbeat off, and optionally the Grounder or the
    server's life replaced. Returns (exit code, the run folder, stdout)."""
    import vla_fly
    import vla_mission as VM
    import follow_vlm as FV
    out = Path(tempfile.mkdtemp(prefix="vlafakesim_"))
    saved = {k: sys.modules.get(k) for k in ("projectairsim",)}
    sys.modules["projectairsim"] = fake_projectairsim(_Vehicle())
    old_env = os.environ.get("VLAGUARD_LIVE_OFF")
    os.environ["VLAGUARD_LIVE_OFF"] = "1"
    old_g, old_wait = FV.Grounder, VM.VLAServer.wait_hello
    if grounder is not None:
        FV.Grounder = grounder
    if kill_after_s is not None:
        def wait_and_arm(self, *a, **k):
            hello = old_wait(self, *a, **k)

            def kill():
                time.sleep(kill_after_s)
                self.proc.kill()
            threading.Thread(target=kill, daemon=True).start()
            return hello
        VM.VLAServer.wait_hello = wait_and_arm
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            code = vla_fly.main(argv + ["--out-dir", str(out), "--vla-port", str(_port()),
                                        "--dev-yaml", "--citymap", str(out / "no_map.npz"),
                                        "--cruise-alt", "7", "--record-hz", "5"])
    finally:
        FV.Grounder, VM.VLAServer.wait_hello = old_g, old_wait
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
        if old_env is None:
            os.environ.pop("VLAGUARD_LIVE_OFF", None)
        else:
            os.environ["VLAGUARD_LIVE_OFF"] = old_env
    return code, out, buf.getvalue()


def _rows(out):
    return [json.loads(x) for x in (out / "flight_log.jsonl").read_text(encoding="utf-8").splitlines()]


def test_m1_flies_lands_and_records_a_stand_in_as_a_stub():
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    import todo as TODO
    code, out, said = _fly(["--mission", "goto", "--tag", "fakesim_goto", "--max-s", "8",
                            "--vla-fake-output", "96 48 49</s>", "96 48 52</s>"])
    assert code == 0, said[-3000:]
    m = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert m["end_reason"] == "timeout" and m["aborted"] is None, said[-2000:]
    assert m["pre_mission"]["reached"] and abs(m["pre_mission"]["start_heading_deg"] - 89.01) < 0.1
    rows = _rows(out)
    assert len(rows) == m["ticks"] >= 60
    assert {r["author"] for r in rows} <= {"vla_step", "hold", "hover", "none"}
    steps = [r for r in rows if r["author"] == "vla_step"]
    assert steps and all(r["hint_source"] == "goal" and r["phrase"] is not None for r in steps)
    recs = [json.loads(x) for x in (out / "vla_steps.jsonl").read_text(encoding="utf-8").splitlines()]
    assert recs and all(r["hint_source"] == "goal" for r in recs)
    caps = [json.loads(x) for x in (out / "vla_captures.jsonl").read_text(encoding="utf-8").splitlines()]
    assert caps and all(c["front_wh"] == [256, 256] and c["down_wh"] == [256, 256] for c in caps)
    frames = {p.stem for p in (out / "vla_frames").iterdir()}
    assert {r["front_jpg_sha256"] for r in recs} <= frames
    p = m["pilot"]
    assert p["kind"] == "stub" and p["stand_in"] and "NOT a VLA" in p["warning"]
    assert p["server"]["mode"] == "started" and p["server"]["stop"] == "shutdown"
    assert p["server"]["exit_code"] == 0 and p["vla_config"] is None
    s = p["summary"]
    # The rate is over the mission, not the landing: the stand-in answers in
    # about 0.3 s, so pipelined decisions come at well over 1.5 Hz.
    assert s["decision_hz"] > 1.8, s["decision_hz"]
    assert s["vla_authored_tick_share"] > 0.8
    man = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
    assert man["vla_model_hash"].startswith("fake:scripted")
    assert json.loads((out / "kpi.json").read_text(encoding="utf-8"))["p0_violation_escape_rate"] == 0.0
    assert TODO.classify_run(man["vla_model_hash"], TODO.vla_evidence(out))[0] != "vla"
    fpv = list((out / "view" / "fpv").glob("*.jpg"))
    assert len(fpv) > 20 and (out / "view" / "recorder.json").is_file()
    assert (out / "vla_server_stdout.log").is_file()


def test_m2_asks_only_on_a_fresh_box_and_ends_on_its_size():
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    code, out, said = _fly(["--mission", "approach", "--tag", "fakesim_appr", "--max-s", "30",
                            "--no-transfer", "--no-land-site",
                            "--vla-fake-output", "96 48 49</s>"], grounder=_GrowingBox)
    assert code == 0, said[-3000:]
    m = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert m["end_reason"] == "close_box", m["end_reason"]
    assert m["mission"]["hint_source"] == "detector" and m["mission"]["park_at"] == [48.0, 12.0]
    assert m["reached_goal"] is not None and "target_truth_dist_end_m" in m
    recs = [json.loads(x) for x in (out / "vla_steps.jsonl").read_text(encoding="utf-8").splitlines()]
    assert recs and {r["hint_source"] for r in recs} == {"detector"}
    assert all(r["phrase"] != "" for r in recs)
    rows = _rows(out)
    assert rows[-1]["box_frac"] >= 0.15 and rows[-1]["det"] is not None


def test_a_server_killed_mid_mission_hovers_then_lands_and_exits_3():
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    code, out, said = _fly(["--mission", "goto", "--tag", "fakesim_dead", "--max-s", "60",
                            "--no-transfer", "--no-land-site", "--no-save-view",
                            "--dead-timeout-s", "3", "--vla-fake-output", "96 48 49</s>"],
                           kill_after_s=4.0)
    assert code == 3, said[-3000:]
    m = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert m["end_reason"] == "server_dead" and m["mission"]["monitor"]["vla_failure"]
    ev = [e["event"] for e in m["mission"]["monitor"]["events"]]
    assert "server_exited" in ev and ev[-1] == "end"
    rows = _rows(out)
    tail = rows[-25:]
    assert all(r["author"] == "hover" and r["author_reason"] == "link_lost" for r in tail), \
        {(r["author"], r["author_reason"]) for r in tail}
    assert m["pilot"]["server"]["exit_code"] not in (None, 0)


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
        except Exception as e:                        # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
