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
  * a server killed mid-mission makes the tracker hover on link_lost (as
    `none` when it dies before the first answer: nothing flown to drop), the
    mission end server_dead after --dead-timeout-s, the aircraft land, and
    the exit code 3.

And the review of the CityLife flight (2026-10-23), every one of which
failed on 46ca291:
  * --shield off is recorded with the simulator safety band as the pilot's
    clamp, no flown interventions, and a policy HUD that read no repairs
    (P1-1, P1-2); the default cadence is stop_and_go (P2-3);
  * a warm-up inference slower than --warmup-max-latency-s refuses the
    flight before arming - exit 4, records written, the server stopped
    (P2-5); a transfer that misses the mission start refuses the mission
    and lands - exit 4 (P3-15);
  * an exception in the mission lands through the abort path and exits 1;
    Ctrl+C (KeyboardInterrupt) lands the same way, writes the records
    marked interrupted and exits 130 (P3-11);
  * an exception between the server's start and the simulator still stops
    the server (P3-10); a fault while writing the records still leaves
    metrics.json with the error and the manifest (P3-12).

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
        self.armed = False
        self.lands = 0
        self.kin_after_arm = 0
        # (n, exception): raise it once, on the n-th kinematics read after arming
        self.fault = None
        self.connect_fault = None

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
        def __init__(self, address=None):
            # vla_fly --sim-address (fe171ab) passes the simulator's address
            self.address = address
            self.subs, self._stop = {}, False

        def connect(self):
            if veh.connect_fault is not None:
                raise veh.connect_fault
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

        disable_api_control = disarm = enable_api_control

        def arm(self):
            veh.armed = True

        def get_ground_truth_kinematics(self):
            veh.advance()
            if veh.armed:
                veh.kin_after_arm += 1
                if veh.fault is not None and veh.kin_after_arm >= veh.fault[0]:
                    exc, veh.fault = veh.fault[1], None
                    raise exc
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
            veh.lands += 1
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


def _fly(argv, grounder=None, kill_after_s=None, veh=None, patch=None):
    """vla_fly.main(argv) with the fake simulator in sys.modules (restored
    after), the Dashboard heartbeat off, and optionally the Grounder or the
    server's life replaced, a prepared _Vehicle (`veh`, for its faults), or
    attributes of vla_fly / vla_mission patched for the run (`patch`:
    {(module, name): value}). Returns (exit code, the run folder, stdout);
    a SystemExit from main() is returned as its message in place of the
    code."""
    import vla_fly
    import vla_mission as VM
    import follow_vlm as FV
    out = Path(tempfile.mkdtemp(prefix="vlafakesim_"))
    saved = {k: sys.modules.get(k) for k in ("projectairsim",)}
    sys.modules["projectairsim"] = fake_projectairsim(veh or _Vehicle())
    old_env = os.environ.get("VLAGUARD_LIVE_OFF")
    os.environ["VLAGUARD_LIVE_OFF"] = "1"
    old_g, old_wait = FV.Grounder, VM.VLAServer.wait_hello
    patched = {}
    for (mod, name), value in (patch or {}).items():
        patched[(mod, name)] = getattr(mod, name)
        setattr(mod, name, value)
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
            try:
                code = vla_fly.main(argv + ["--out-dir", str(out), "--vla-port", str(_port()),
                                            "--dev-yaml", "--citymap", str(out / "no_map.npz"),
                                            "--cruise-alt", "7", "--record-hz", "5"])
            except SystemExit as exc:
                code = str(exc)
    finally:
        FV.Grounder, VM.VLAServer.wait_hello = old_g, old_wait
        for (mod, name), value in patched.items():
            setattr(mod, name, value)
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
    import vla_mission as VM
    code, out, said = _fly(["--mission", "goto", "--tag", "fakesim_goto", "--max-s", "8",
                            "--cadence", "pipelined",
                            "--vla-fake-output", "96 48 49</s>", "96 48 52</s>"])
    assert code == 0, said[-3000:]
    m = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert m["end_reason"] == "timeout" and m["aborted"] is None, said[-2000:]
    assert m["pre_mission"]["reached"] and abs(m["pre_mission"]["start_heading_deg"] - 89.01) < 0.1
    assert m["pre_mission"]["start_error_m"] < 2.0 and m["pre_mission"]["start_pose_actual"]
    assert m["warmup"]["pass"] and m["warmup"]["answered"] and m["warmup"]["rtt_s"] < 5.0
    assert m["params"]["cadence"] == "pipelined" and m["mission"]["object"] == VM.GOTO_OBJECT_TEXT
    assert m["pilot_alt_band"]["clamp"] == "policy" and not m["interrupted"]
    rows = _rows(out)
    assert len(rows) == m["ticks"] >= 60
    assert {r["author"] for r in rows} <= {"vla_step", "hold", "hover", "none"}
    steps = [r for r in rows if r["author"] == "vla_step"]
    assert steps and all(r["hint_source"] == "goal" and r["phrase"] is not None for r in steps)
    recs = [json.loads(x) for x in (out / "vla_steps.jsonl").read_text(encoding="utf-8").splitlines()]
    assert recs and all(r["hint_source"] == "goal" for r in recs)
    assert all(r["object"] == VM.GOTO_OBJECT_TEXT for r in recs)
    caps = [json.loads(x) for x in (out / "vla_captures.jsonl").read_text(encoding="utf-8").splitlines()]
    assert caps and all(c["front_wh"] == [256, 256] and c["down_wh"] == [256, 256] for c in caps)
    # One line per request SENT (the warm-ups first: VM.WARMUP_REQUESTS of
    # them, re-review N1), never a capture that was not.
    nw = VM.WARMUP_REQUESTS
    assert all(c.get("warmup") for c in caps[:nw]) and [c["seq"] for c in caps[:nw]] == list(range(1, nw + 1))
    assert not any(c.get("warmup") for c in caps[nw:])
    assert sorted(c["seq"] for c in caps[nw:]) == sorted(r["seq"] for r in recs)
    assert len(m["warmup"]["attempts"]) == nw and m["warmup"]["judged"] == "last of attempts"
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


def test_the_detector_camera_is_the_front_image_with_vla_front_source_detector():
    """--vla-front-source detector (2026-10-27): the detector's FrontCamera
    frame (768x432) goes in the request's front slot from the one
    subscription that also feeds OWL-ViT, the down frame is paired as
    always, and the params record the choice (a stand-in: not checked)."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    code, out, said = _fly(["--mission", "approach", "--tag", "fakesim_detfront", "--max-s", "20",
                            "--no-transfer", "--no-land-site", "--vla-front-source", "detector",
                            "--vla-fake-output", "96 48 49</s>"], grounder=_GrowingBox)
    assert code == 0, said[-3000:]
    m = _metrics(out)
    fs = m["params"]["front_source"]
    assert fs["flight"] == "detector" and fs["checked"] is False, fs
    caps = [json.loads(x) for x in (out / "vla_captures.jsonl").read_text(encoding="utf-8").splitlines()]
    assert caps and all(c["front_wh"] == [768, 432] and c["down_wh"] == [256, 256] for c in caps)
    assert "front image FrontCamera" in said, said[-2000:]


def _author_runs(rows):
    """The rows' author transitions: each run of consecutive rows with one
    (author, author_reason), as {"author", "reason", "first", "last"} (row
    indexes, inclusive)."""
    runs = []
    for i, r in enumerate(rows):
        if runs and (runs[-1]["author"], runs[-1]["reason"]) == (r["author"], r["author_reason"]):
            runs[-1]["last"] = i
        else:
            runs.append({"author": r["author"], "reason": r["author_reason"], "first": i, "last": i})
    return runs


def test_a_server_killed_mid_mission_hovers_then_lands_and_exits_3():
    """The server killed 4 s after its hello: once the pilot finds the link
    dead the aircraft hovers without a break, the dead rule ends the mission
    server_dead --dead-timeout-s later, the aircraft lands, exit code 3.

    Stated from the flight's own records (2026-11-06), not from the last 25
    rows, which the lab CI at 61a24ab once found all ('none', 'link_lost').
    The kill is wall-clock and lands about when the mission's first answer
    does (warm-ups, climb and turn take ~3.6 s alone, ~4 s on a shared CPU;
    the stand-in answers ~0.3 s into the mission). A server dead before that
    answer leaves the tracker nothing to drop, and it authors the same zero
    command `none`/link_lost (vla_link.StepTracker._no_answer, pinned in
    tests/test_vla_backends.py); the monitor's dead rule counts both.

    So: the server's exit is in the monitor's events and the end is the last
    of them; the rows' LAST author run is the hover - `hover` if anything
    was flown before it, `none` if not - the only link loss, beginning no
    earlier than the tick before the monitor saw the exit (the reset can
    reach the link first), every tick a zero command over a dead link, for
    dead_timeout_s; then the landing. A run that never hovers fails, shown
    failing first: with the monitor ending server_dead on the tick it saw
    the exit (exit 3 and server_dead as before), the last run is the step
    being flown, or a link_lost run of one tick."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    veh = _Vehicle()
    code, out, said = _fly(["--mission", "goto", "--tag", "fakesim_dead", "--max-s", "60",
                            "--no-transfer", "--no-land-site", "--no-save-view",
                            "--dead-timeout-s", "3", "--vla-fake-output", "96 48 49</s>"],
                           kill_after_s=4.0, veh=veh)
    assert code == 3, said[-3000:]
    m = _metrics(out)
    mon = m["mission"]["monitor"]
    assert m["end_reason"] == "server_dead" and mon["vla_failure"], m["end_reason"]
    assert m["pilot"]["server"]["exit_code"] not in (None, 0)        # killed, not stopped
    ev = mon["events"]
    exits = [e["t"] for e in ev if e["event"] == "server_exited"]
    assert len(exits) == 1 and ev[-1]["event"] == "end" and ev[-1]["reason"] == "server_dead", ev
    rows = _rows(out)
    runs = _author_runs(rows)
    seen = [f"{x['author']}/{x['reason']} x{x['last'] - x['first'] + 1} from t={rows[x['first']]['t']}"
            for x in runs]
    # The tick the monitor saw the server gone, in the rows' clock: the
    # events count from vla_fly's t0, the rows from the loop's first tick,
    # and the end tick is the last row in both.
    shift = ev[-1]["t"] - rows[-1]["t"]
    died = min(range(len(rows)), key=lambda i: abs(rows[i]["t"] + shift - exits[0]))
    assert abs(rows[died]["t"] + shift - exits[0]) <= 0.002, (shift, exits, seen)
    hover = runs[-1]
    assert hover["reason"] == "link_lost" and hover["author"] in ("hover", "none"), seen
    assert not any(x["reason"] == "link_lost" for x in runs[:-1]), seen
    assert hover["first"] >= died - 1, (rows[died]["t"], seen)
    flew = any(x["author"] != "none" for x in runs[:-1])
    assert hover["author"] == ("hover" if flew else "none"), seen
    streak = rows[hover["first"]:]
    moved = [r for r in streak if any(r["flown_action"][k] for k in ("vx", "vy", "vz_up", "yaw_rate"))]
    assert not moved, (len(moved), moved[0]["t"], moved[0]["flown_action"])
    assert all(r["link_failure"] in ("link", "protocol") for r in streak), streak[-1]
    dead_s = mon["config"]["dead_timeout_s"]
    assert streak[-1]["t"] - streak[0]["t"] >= dead_s - 0.002, (dead_s, seen)
    # ... then the controlled landing, not an abort.
    assert veh.lands >= 1 and m["aborted"] is None and m["abort_landing"] is None, \
        (veh.lands, m["aborted"], m["abort_landing"])


def _metrics(out):
    return json.loads((out / "metrics.json").read_text(encoding="utf-8"))


def test_shield_off_records_the_sim_band_no_flown_repair_and_stop_and_go():
    """P1-1, P1-2, P2-3 through fly(): --shield off records the simulator
    safety band as the pilot's clamp (source "configured", no policy band),
    0 flown interventions with the shadow's apart, a policy HUD that read
    the no-repair decision; and the default cadence is stop_and_go, in the
    params and the pilot record."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    import vla_mission as VM
    code, out, said = _fly(["--mission", "goto", "--tag", "fakesim_off", "--max-s", "10",
                            "--no-transfer", "--no-land-site", "--shield", "off",
                            "--vla-fake-output", "96 30 49</s>"])
    assert code == 0, said[-3000:]
    m = _metrics(out)
    assert m["shield"] == "off" and m["params"]["cadence"] == "stop_and_go"
    assert m["pilot"]["cadence"] == "stop_and_go" and m["pilot"]["vla"]["cadence"] == "stop_and_go"
    band = m["pilot_alt_band"]
    assert band["clamp"] == "sim_safety_band" and band["band_m"] == list(VM.SIM_SAFETY_BAND_M)
    assert band["applied"]["source"] == "configured" and band["applied"]["policy"] is None
    assert band["applied"]["band"] == list(VM.SIM_SAFETY_BAND_M)
    assert m["interventions"] == 0 and isinstance(m["shadow_interventions"], int)
    hud = m["policy_hud"]
    assert hud is not None and hud["counts"].get("repaired", 0) == 0, hud
    assert hud["decision_read"].startswith("shadow")
    k = json.loads((out / "kpi.json").read_text(encoding="utf-8"))
    assert k["pilot_alt_band"]["clamp"] == "sim_safety_band" and k["cadence"] == "stop_and_go"
    assert "OFF (control arm)" in said and "simulator safety band 2-40 m" in said
    # "96 30 49": fwd 4.9 m, 1.9 m up per step: stop-and-go applies it once per step.
    rows = _rows(out)
    assert max(r["up"] for r in rows) > 7.5, max(r["up"] for r in rows)


def test_a_slow_warm_up_refuses_the_flight_before_take_off():
    """P2-5: one inference on a live pair before arming; a round trip over
    --warmup-max-latency-s refuses the flight (exit 4): never armed, no
    mission rows, the records say why, the server stopped by op shutdown."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    veh = _Vehicle()
    code, out, said = _fly(["--mission", "goto", "--tag", "fakesim_warm", "--max-s", "20",
                            "--vla-fake-output", "96 48 49</s>", "--vla-fake-delay-s", "1.0",
                            "--warmup-max-latency-s", "0.4"], veh=veh)
    assert code == 4, said[-3000:]
    assert not veh.armed and veh.lands == 0
    m = _metrics(out)
    assert m["end_reason"] == "warmup_refused" and m["refused"]["kind"] == "warmup_refused"
    assert m["ticks"] == 0 and not m["warmup"]["pass"] and m["warmup"]["rtt_s"] > 0.9
    assert "over the 0.4 s gate" in m["warmup"]["why"] and "REFUSED" in said
    # The last of the warm-up requests is the one judged (re-review N1).
    import vla_mission as VM
    assert m["pilot"]["server"]["stop"] == "shutdown"
    assert m["pilot"]["warmup"]["seq"] == VM.WARMUP_REQUESTS
    assert len(m["warmup"]["attempts"]) == VM.WARMUP_REQUESTS
    assert (out / "manifest.json").is_file()


def test_a_transfer_that_misses_the_start_refuses_the_mission_and_lands():
    """P3-15: the mission begins only within --start-tolerance-m of its
    start; a transfer that ends short is refused, landed, exit 4, with the
    error and where it was recorded."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    import vla_fly

    async def stuck(drone, shield, State, Action4D, quat_yaw, target_xy, cruise_up, args,
                    live, timeout_s=40.0):
        return {"author": "test", "target": list(target_xy), "reached": False, "ticks": 0,
                "shield_touched": 0}
    veh = _Vehicle()
    code, out, said = _fly(["--mission", "goto", "--tag", "fakesim_start", "--max-s", "20",
                            "--vla-fake-output", "96 48 49</s>"], veh=veh,
                           patch={(vla_fly, "_transfer"): stuck})
    assert code == 4, said[-3000:]
    m = _metrics(out)
    assert m["end_reason"] == "start_not_reached" and m["ticks"] == 0
    assert m["pre_mission"]["start_error_m"] > 7.0 and veh.armed and veh.lands >= 1
    assert m["abort_landing"].startswith("landed")


def test_an_exception_in_the_mission_lands_and_exits_1():
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    veh = _Vehicle()
    veh.fault = (60, RuntimeError("the simulator dropped the connection"))
    code, out, said = _fly(["--mission", "goto", "--tag", "fakesim_abort", "--max-s", "30",
                            "--no-transfer", "--no-land-site", "--no-save-view",
                            "--vla-fake-output", "96 48 49</s>"], veh=veh)
    assert code == 1, said[-3000:]
    m = _metrics(out)
    assert m["aborted"].startswith("RuntimeError") and not m["interrupted"]
    assert veh.lands >= 1 and m["abort_landing"].startswith("landed") and m["ticks"] > 0
    assert m["pilot"]["server"]["stop"] == "shutdown"


def test_ctrl_c_lands_writes_the_records_marked_interrupted_and_exits_130():
    """P3-11: KeyboardInterrupt is not an Exception, so `except Exception`
    let Ctrl+C past the landing; now it lands through the abort path, the
    records are written with interrupted true, and the exit code is 130."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    veh = _Vehicle()
    veh.fault = (60, KeyboardInterrupt())
    code, out, said = _fly(["--mission", "goto", "--tag", "fakesim_int", "--max-s", "30",
                            "--no-transfer", "--no-land-site", "--no-save-view",
                            "--vla-fake-output", "96 48 49</s>"], veh=veh)
    assert code == 130, said[-3000:]
    m = _metrics(out)
    assert m["interrupted"] is True and m["end_reason"] == "interrupted", m["end_reason"]
    assert veh.lands >= 1 and m["abort_landing"].startswith("landed") and m["ticks"] > 0
    assert m["pilot"]["server"]["stop"] == "shutdown" and (out / "manifest.json").is_file()
    assert json.loads((out / "kpi.json").read_text(encoding="utf-8"))["interrupted"] is True


def test_a_failure_before_the_simulator_still_stops_the_server():
    """P3-10: after the server said hello and before the simulator's try,
    46ca291 had code that could raise unguarded - here the frame store's
    folder cannot be made (a full disk) - and the server it had started was
    left running, about 17 GB on the GPU. Now the flight exits (SystemExit
    with the reason) and that server is stopped by op shutdown. So is it
    when the simulator's connect fails."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    import vla_mission as VM

    def no_disk(*a, **k):
        raise OSError("disk full")
    for case, veh, patch in (("frames", _Vehicle(), {(VM, "FrameSaver"): no_disk}),
                             ("simulator", _Vehicle(), {})):
        if case == "simulator":
            veh.connect_fault = ConnectionRefusedError("no simulator on 8989")
        seen = {}
        real_start, real_stop = VM.VLAServer.start, VM.VLAServer.stop

        def spy_start(self, _seen=seen, _real=real_start):
            _seen["server"] = self
            return _real(self)

        def spy_stop(self, *a, _seen=seen, _real=real_stop, **k):
            rec = _real(self, *a, **k)
            _seen.setdefault("first_stop", dict(rec))
            return rec
        try:
            code, out, said = _fly(["--mission", "goto", "--tag", f"fakesim_orphan_{case}",
                                    "--max-s", "10", "--vla-fake-output", "96 48 49</s>"],
                                   veh=veh, patch={(VM.VLAServer, "start"): spy_start,
                                                   (VM.VLAServer, "stop"): spy_stop, **patch})
            srv = seen["server"]
            assert isinstance(code, str) and "before take-off" in code, (case, code)
            assert ("disk full" if case == "frames" else "no simulator") in code, code
            assert seen.get("first_stop", {}).get("stop") == "shutdown", (case, seen)
            assert not srv.alive and not VM.port_open("127.0.0.1", srv.port), case
        finally:
            srv = seen.get("server")           # never leave a test's server behind
            if srv is not None and srv.proc is not None and srv.proc.poll() is None:
                srv.proc.kill()


class _LevelCar:
    """level_actors.LevelCar stand-in: the level's car 12 m ahead of the
    spawn, standing (truth for scoring only)."""

    def __init__(self, world, tag, desc=None, min_period_s=0.1, kind="car"):
        # `kind` since c31f1f7 (a level pedestrian as the subject)
        self.tag, self.pos, self.kind = tag, (35.0, -8.0), kind

    def spawn(self):
        return 1

    def update(self, t):
        pass

    def stats(self):
        return {"tag": self.tag, "fake": True}


def test_a_follow_whose_identity_fails_flies_the_detector_gate_and_says_so():
    """M3 through fly() (review of d6e7123): the fake robot's detector camera
    has no depth topic, so the identity setup fails - and the follow flies
    with the DetectorHint's start gate (P2-5: it used to fly with none),
    the plain lock, the failure named in params.follow.identity, the gate
    there agreeing with mission.detector_hint's; the score at the top of
    metrics where kpi reads it (P1-1); measured outcomes in metrics.follow
    (P3); the hint kind on every request (P2-2); sim_config saying the depth
    stream was not read (P3); and (2026-11-01) the policy's stand-off
    reported INERT, with no subject given to the Shield on any tick."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    import level_actors
    # The follow flights' policy (scripts\run_vla_fly.ps1 -PolicyFile), with
    # its two subject stand-offs (2026-11-01).
    code, out, said = _fly(["--mission", "follow", "--tag", "fakesim_follow", "--max-s", "12",
                            "--no-transfer", "--no-land-site", "--no-save-view",
                            "--policy", str(ROOT / "policies" / "follow_car_citylife_nfz.yaml"),
                            "--cadence", "pipelined", "--vla-fake-output", "96 48 49</s>"],
                           grounder=_GrowingBox,
                           patch={(level_actors, "LevelCar"): _LevelCar})
    assert code == 0, said[-3000:]
    assert "[identity] *** OFF" in said and "depth_camera" in said, said[-3000:]
    m = _metrics(out)
    pf = m["params"]["follow"]
    assert pf["identity"]["requested"] and not pf["identity"]["on"], pf
    assert "depth_camera" in pf["identity"]["error"], pf
    assert pf["start_gate"] == {"kind": "detector_hint", "n": 5, "width_frac": 0.04}, pf
    g = m["mission"]["detector_hint"]["start_gate"]
    assert g["n"] == 5 and g["open"] and g["t_open_s"] is not None and g["t_open_s"] >= 0, g
    assert pf["lock_gate"] == 0.12 and pf["perception_applied"]
    assert not {"acquisition", "stuck_resets", "close_hold_ticks"} & set(pf)
    f = m["follow"]
    assert f["acquisition"] is None and f["perception_resets"] == [] and "close_hold_ticks" in f
    assert m["follow_scored"] is True and m["frac_within_30m"] == 1.0, (m["follow_scored"],
                                                                         m["frac_within_30m"])
    k = json.loads((out / "kpi.json").read_text(encoding="utf-8"))
    assert k["mission_success"] is not None and "follow_not_scored" not in k["mission_fail_reasons"]
    assert m["sim_config"]["detector_depth_read"] is False
    assert m["sim_config"]["detector_depth_rendered"] is True
    recs = [json.loads(x) for x in (out / "vla_steps.jsonl").read_text(encoding="utf-8").splitlines()]
    assert recs and all(r.get("hint_kind") in ("fresh", "memory", "search") for r in recs)
    rows = _rows(out)
    assert all("hint_kind" in r and "close_box" in r and "pilot_raw" in r for r in rows)
    # Wave FT (2026-10-23): the follow's tracker smooths by default, every
    # row carries the attitude (the fake kinematics' quaternion: level), and
    # metrics carry the motion on tools/compare_motion.py's definitions.
    tr = m["params"]["tracker"]
    assert (tr["smooth_accel_mps2"], tr["smooth_yaw_accel_rps2"], tr["carry_s"]) == (2.5, 0.6, 1.0)
    assert m["params"]["smooth"]["on"] is True
    assert all(r.get("pitch_deg") == 0.0 and r.get("roll_deg") == 0.0 for r in rows), rows[0]
    mo = m["motion"]
    assert mo["attitude_recorded"] is True and mo["ticks"] == len(rows), mo
    assert mo["cmd_accel_max"] <= 2.5 + 1e-6 or any(r["touched"] for r in rows), mo
    # 2026-11-01: with the identity off no box is placed on the map, so the
    # Shield is given no subject - and the flight says so, in the console
    # (before and after) and in metrics, rather than letting the policy's
    # stand-off read as a pass (tests/test_vla_standoff_feed.py).
    ss = m["subject_standoff"]
    assert ss["rules"] == ["standoff-pedestrian", "standoff-any"], ss
    assert ss["binding"] == ["standoff-any"] and ss["subject_class"] == "car", ss
    assert ss["inert"] and not ss["on"] and "identity is off" in ss["inert_why"], ss
    assert ss["ticks"] == len(rows) and ss["ticks_fed"] == 0 and ss["violation_ticks"] == 0
    assert all("shield_subject" in r and r["shield_subject"] is None for r in rows)
    assert said.count("[standoff] *** standoff-any INERT") == 2, said[-3000:]


def test_a_prev_front_follow_flown_as_rates_sends_the_earlier_front_and_records_it():
    """2026-10-24 through fly(): --vla-bottom-tile prev_front --step-as-rate
    1.0. Every request's down slot is the VLAFront frame from about 1 s
    earlier (the fake cameras give front and down the same image at each
    tick, so a down-mode request's two JPEGs are equal and a prev_front one's
    are not), its record says so (vla_steps.jsonl `bottom_tile`, the
    measured dt within the tolerance), every row carries the commanded rates
    (`vla_rate`), and params record the rate and the tile check (a stand-in
    reads no image: not checked)."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    import level_actors
    code, out, said = _fly(["--mission", "follow", "--tag", "fakesim_pf", "--max-s", "10",
                            "--no-transfer", "--no-land-site", "--no-save-view",
                            "--cadence", "pipelined", "--vla-fake-output", "58 49 52</s>",
                            "--vla-bottom-tile", "prev_front", "--step-as-rate", "1.0"],
                           grounder=_GrowingBox,
                           patch={(level_actors, "LevelCar"): _LevelCar})
    assert code == 0, said[-3000:]
    m = _metrics(out)
    p = m["params"]
    assert p["step_as_rate_s"] == 1.0 and p["tracker"]["rate_horizon_s"] == 1.0, p["tracker"]
    assert p["bottom_tile"]["flight"] == {"mode": "prev_front", "dt_s": 1.0, "tol_s": 0.25}
    assert p["bottom_tile"]["checked"] is False and "stand-in" in p["bottom_tile"]["source"]
    recs = [json.loads(x) for x in (out / "vla_steps.jsonl").read_text(encoding="utf-8").splitlines()]
    assert recs
    for r in recs:
        bt = r["bottom_tile"]
        assert bt["mode"] == "prev_front" and 0.75 <= bt["dt_actual_s"] <= 1.25, bt
        assert r["down_jpg_sha256"] != r["front_jpg_sha256"], "the down camera was sent"
    fc = m["frames"]["capture"]["bottom_tile"]
    assert fc["mode"] == "prev_front" and fc["dt_actual_s"]["n"] >= len(recs) - 1, fc
    rows = _rows(out)
    assert rows and all("vla_rate" in r for r in rows)
    rate = [r for r in rows if r["author_reason"] == "rate"]
    assert rate and all(r["vla_rate"]["horizon_s"] == 1.0 for r in rate), rows[-1]
    assert all(r["vla_rate"] is None for r in rows if r["author_reason"] != "rate")


def test_a_speed_words_follow_sends_its_own_speed_after_the_object_and_records_it():
    """2026-10-24 through fly(): --vla-speed-words. Every request's object is
    the follow's object words plus the class words of the aircraft's own
    speed from the pose ring at or before its capture (vla_steps.jsonl
    `object`, `own_speed`: the rule's words for its speed, the base words),
    the warm-up's too, VLAFrames' stats count the words, and params record
    the check (a stand-in reads no prompt: not checked)."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    import level_actors
    from guardrail import aerialvla_codec as C
    code, out, said = _fly(["--mission", "follow", "--tag", "fakesim_sw", "--max-s", "10",
                            "--no-transfer", "--no-land-site", "--no-save-view",
                            "--cadence", "pipelined", "--vla-fake-output", "58 49 49</s>",
                            "--vla-speed-words"],
                           grounder=_GrowingBox,
                           patch={(level_actors, "LevelCar"): _LevelCar})
    assert code == 0, said[-3000:]
    m = _metrics(out)
    sw = m["params"]["speed_words"]
    assert sw["flight"] == C.SpeedWords().as_dict(), sw
    assert sw["checked"] is False and "stand-in" in sw["source"], sw
    recs = [json.loads(x) for x in (out / "vla_steps.jsonl").read_text(encoding="utf-8").splitlines()]
    assert recs
    for r in recs:
        os_ = r["own_speed"]
        assert os_["words"] == C.own_speed_words(os_["speed_mps"]), os_
        assert os_["object_base"] == "a red car" and os_["window_s"] == 0.5
        assert r["object"] == f"a red car {os_['words']}", r["object"]
        assert os_["t_end"] <= r["t_capture"] + 1e-3, "a pose after the capture"
    fc = m["frames"]["capture"]["speed_words"]
    assert sum(fc["words"].values()) >= len(recs), fc
    assert "speed words on" in said


def test_a_fault_in_the_records_still_leaves_metrics_and_the_manifest():
    """P3-12: pilot_entry raising after a real flight - metrics.json is
    still written, with the error in records_errors, the manifest and the
    KPIs too, and the exit code says the records are incomplete (1)."""
    why = _missing()
    if why:
        print(f"      ({why})")
        return SKIP
    import vla_mission as VM

    def broken(*a, **k):
        raise KeyError("summary")
    code, out, said = _fly(["--mission", "goto", "--tag", "fakesim_rec", "--max-s", "5",
                            "--no-transfer", "--no-land-site", "--no-save-view",
                            "--vla-fake-output", "96 48 49</s>"],
                           patch={(VM, "pilot_entry"): broken})
    assert code == 1, said[-3000:]
    m = _metrics(out)
    assert any(e.startswith("pilot_entry: KeyError") for e in m["records_errors"]), m
    assert m["pilot"]["kind"] == "unrecorded" and m["ticks"] > 0
    assert (out / "manifest.json").is_file() and (out / "kpi.json").is_file()
    assert "RECORDS INCOMPLETE" in said


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
